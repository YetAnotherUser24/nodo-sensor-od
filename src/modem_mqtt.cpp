/**
 * @file modem_mqtt.cpp
 * @brief Implementation of A7670 modem control, GPRS attach, MQTT handling,
 *        and ThingsBoard RPC processing.
 *
 * Guarded by #ifdef GPRS_MODULE — disabled when GPRS feature is toggled off.
 */

#ifdef GPRS_MODULE

#include "modem_mqtt.h"
#include "n_logger_config.h"
#include "board.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <RtcDS3231.h>
#include <TinyGsmClient.h>
#include <Wire.h>
#include <esp_task_wdt.h>

#include "config.h"
#include "log.h"
#include "rtc_utils.h"

#define SerialMon Serial
#define SerialAT Serial1

/* ------------------------- External Definitions ------------------------- */

extern ModuleConfig moduleCfg;
extern TinyGsm modem;
extern PubSubClient mqtt;

extern RtcDS3231<TwoWire> rtc;
extern RtcDateTime rtcNow;
extern char dateStr[20];

extern bool firstRun;
extern bool forceModemReset;

void formatDateTime(const RtcDateTime& dt);

/* ----------------------- Internal RPC Queue Structure -------------------- */

struct RpcMsg {
  String topic;
  String payload;
};

static RpcMsg rpcQueue[8];
static uint8_t rpcCount = 0;

/* ---------------------------- JSON Helpers ------------------------------- */

/**
 * @brief Extracts the "method" field from a ThingsBoard RPC payload.
 */
static String getRpcMethod(const String& payload) {
  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, payload)) {
    Logln("JSON parse error");
    return "";
  }
  return doc["method"] | "";
}

/* ------------------------------ MQTT Callback ---------------------------- */

/**
 * @brief Handles incoming MQTT messages and enqueues RPC commands.
 *
 * Stores messages in `rpcQueue` for deferred processing.
 * Auto-ACK is sent back to ThingsBoard widgets.
 */
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  String t = String(topic);
  String p;
  p.reserve(length);

  for (unsigned i = 0; i < length; i++) {
    p += (char)payload[i];
  }

  if (t.startsWith("v1/devices/me/rpc/request/")) {

    if (rpcCount < (sizeof(rpcQueue) / sizeof(rpcQueue[0]))) {
      rpcQueue[rpcCount].topic = t;
      rpcQueue[rpcCount].payload = p;
      rpcCount++;
    }

    Log("RPC queued -> ");
    Log(t.c_str());
    Log(" :: ");
    Logln(p.c_str());

    // Send ACK response
    int slash = t.lastIndexOf('/');
    if (slash > 0) {
      String reqId = t.substring(slash + 1);
      String respTopic = "v1/devices/me/rpc/response/" + reqId;

      StaticJsonDocument<64> doc;
      doc["ok"] = true;
      String out;
      serializeJson(doc, out);

#ifdef DEBUG
      Logln("MQTT -> RPC Response:");
      Log("   Topic: ");
      Logln(respTopic.c_str());
      Log("   Payload: ");
      Logln(out.c_str());
#endif

      mqtt.publish(respTopic.c_str(), out.c_str());
    }
  }
}

/* --------------------------- Modem Initialization ------------------------ */

/**
 * @brief Initializes the A7670 modem, resets (if first boot), wakes it,
 *        attaches GPRS and attempts to connect MQTT.
 *
 * @return true if modem and MQTT connected, false on any failure.
 */
bool initA7670() {
  Serial.begin(115200);
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

#ifdef BOARD_POWERON_PIN
  pinMode(BOARD_POWERON_PIN, OUTPUT);
  digitalWrite(BOARD_POWERON_PIN, HIGH);
#endif

  if (firstRun || forceModemReset) {

#ifdef DEBUG
    if (forceModemReset) {
      Logln("Performing forced modem reset...");
    }
#endif

#ifdef MODEM_RESET_PIN
    gpio_hold_dis((gpio_num_t)MODEM_RESET_PIN);
    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
    delay(100);
    digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL);
    delay(2600);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
#endif

    pinMode(MODEM_DTR_PIN, OUTPUT);
    digitalWrite(MODEM_DTR_PIN, LOW);

    pinMode(BOARD_PWRKEY_PIN, OUTPUT);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);
    delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, HIGH);
    delay(1000);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);

    forceModemReset = false;

  } else {
    Logln("Wakeup modem !");
    gpio_hold_dis((gpio_num_t)MODEM_DTR_PIN);

#ifdef BOARD_POWERON_PIN
    gpio_hold_dis((gpio_num_t)BOARD_POWERON_PIN);
#endif

    pinMode(MODEM_DTR_PIN, OUTPUT);
    digitalWrite(MODEM_DTR_PIN, LOW);
    delay(2000);
    modem.sleepEnable(false);
    delay(1000);
  }

  esp_task_wdt_reset();
  Logln("Start modem...");
  delay(3000);
  esp_task_wdt_reset();

  if (!modem.init()) {
    DBG("Failed to init modem");
    return false;
  }

  esp_task_wdt_reset();
  DBG("Modem Name:", modem.getModemName());
  DBG("Modem Info:", modem.getModemInfo());

  if (GSM_PIN && modem.getSimStatus() != 3) {
    modem.simUnlock(GSM_PIN);
  }

  Log("Waiting for network...");
  esp_task_wdt_reset();
  if (!modem.waitForNetwork()) {
    Logln(" fail");
    delay(10000);
    return false;
  }
  Logln(" success");
  esp_task_wdt_reset();

  if (!modem.gprsConnect(APN, APN_USER, APN_PASS)) {
    Logln("GPRS attach fail");
    delay(10000);
    return false;
  }

  Logln("GPRS connected");
  esp_task_wdt_reset();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  bool mqttOk = connectMQTT();

  if (!mqttOk) {
    Logln("MQTT connection failed, continuing in offline mode.");
  }

  return mqttOk;
}

/* ---------------------------- Modem Sleep Mode --------------------------- */

/**
 * @brief Puts the modem into full low-power mode using DTR and GPIO holds.
 */
void sleepA7670() {
  esp_task_wdt_reset();
  if (mqtt.connected())
    mqtt.disconnect();

  modem.gprsDisconnect();
  Logln("Enter modem sleep mode!");

  pinMode(MODEM_DTR_PIN, OUTPUT);
  digitalWrite(MODEM_DTR_PIN, HIGH);
  gpio_hold_en((gpio_num_t)MODEM_DTR_PIN);
  gpio_deep_sleep_hold_en();

  if (!modem.sleepEnable(true)) {
    Logln("Modem sleep failed!");
  } else {
    Logln("Modem entered sleep mode.");
  }

  delay(1000);
  esp_task_wdt_reset();

  uint32_t t0 = millis();
  while (modem.testAT(500) && (millis() - t0) < 5000UL) {
    esp_task_wdt_reset();
    Log(".");
    delay(250);
  }
  Logln("Modem is not responding, modem has slept!");
  esp_task_wdt_reset();

#ifdef BOARD_POWERON_PIN
  gpio_hold_en((gpio_num_t)BOARD_POWERON_PIN);
  gpio_deep_sleep_hold_en();
#endif

#ifdef MODEM_RESET_PIN
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
  gpio_hold_en((gpio_num_t)MODEM_RESET_PIN);
  gpio_deep_sleep_hold_en();
#endif
}

/* ------------------------------ MQTT Connect ----------------------------- */

/**
 * @brief Attempts MQTT connection with retry logic.
 *
 * @param maxAttempts Maximum number of connection attempts.
 * @param delayMs Delay between retry attempts in milliseconds.
 * @return true if connected, false if all attempts fail.
 */
bool connectMQTT(uint8_t maxAttempts, uint16_t delayMs) {
  if (mqtt.connected()) {
    return true;
  }

  Log("Connecting MQTT...");

  for (uint8_t attempt = 1; attempt <= maxAttempts; attempt++) {
    esp_task_wdt_reset();
    Logf(" (attempt %u/%u)", attempt, maxAttempts);

    if (mqtt.connect(MQTT_CLIENT_ID, MQTT_TOKEN, "")) {
      Logln(" connected.");
      mqtt.setCallback(onMqttMessage);
      mqtt.subscribe(TB_RPC_SUB_TOPIC);
      return true;
    }

    Log(".");

    if (attempt < maxAttempts) {
      Logf(", retrying in %ums...\n", delayMs);
      delay(delayMs);
    }
  }

  Logln(" All attempts failed.");
  return false;
}

/* --------------------------- RPC Command Handler ------------------------- */

/**
 * @brief Parses and executes all queued RPC messages.
 *
 * Supported RPC formats:
 *  - T-x-y-z  → update timers (measure, BT, GPRS)
 *  - D-YYYY-MM-DD-hh-mm-ss → update RTC
 */
void processQueuedRpcCommands() {
  if (rpcCount == 0) {
    Logln("No RPC queued.");
    return;
  }

  Logf("Processing %u RPC command(s):\n", rpcCount);

  for (uint8_t i = 0; i < rpcCount; i++) {

    Logf("  [%u] Topic: %s\n", i + 1, rpcQueue[i].topic.c_str());
    Logf("      Payload: %s\n", rpcQueue[i].payload.c_str());

    String cmd = getRpcMethod(rpcQueue[i].payload);
    cmd.trim();
    cmd.toUpperCase();

    /* ---- Timer update: T-x-y-z ---- */
    if (cmd.startsWith("T-")) {
      int i1 = cmd.indexOf('-', 2);
      int i2 = cmd.indexOf('-', i1 + 1);

      if (i1 != -1 && i2 != -1) {
        moduleCfg.measure_time = cmd.substring(2, i1).toInt();
        moduleCfg.bluetooth_time = cmd.substring(i1 + 1, i2).toInt();
        moduleCfg.GPRS_time = cmd.substring(i2 + 1).toInt();

        EEPROM.begin(sizeof(ModuleConfig));
        EEPROM.put(0, moduleCfg);
        EEPROM.commit();
        EEPROM.end();

#ifdef DEBUG
        Logln("Updated configuration:");
        Logf(" - Measure:   %d\n", moduleCfg.measure_time);
        Logf(" - Bluetooth: %d\n", moduleCfg.bluetooth_time);
        Logf(" - GPRS:      %d\n\n", moduleCfg.GPRS_time);
#endif
      }

      digitalWrite(LED_PIN, LOW);
      delay(500);
      digitalWrite(LED_PIN, HIGH);

      /* ---- RTC update: D-YYYY-MM-DD-hh-mm-ss ---- */
    } else if (cmd.startsWith("D-")) {

      int year = atoi(cmd.substring(2, 6).c_str());
      int month = atoi(cmd.substring(7, 9).c_str());
      int day = atoi(cmd.substring(10, 12).c_str());
      int hour = atoi(cmd.substring(13, 15).c_str());
      int minute = atoi(cmd.substring(16, 18).c_str());
      int second = atoi(cmd.substring(19, 21).c_str());

      RtcDateTime newDt(year, month, day, hour, minute, second);
      setRtcDateTime(newDt);

#ifdef DEBUG
      Log("New date: ");
      formatDateTime(rtcNow);
      Logln(dateStr);
#endif

      digitalWrite(LED_PIN, LOW);
      delay(500);
      digitalWrite(LED_PIN, HIGH);

    } else {
      // Unknown RPC command → blink three times
      for (int j = 0; j < 3; j++) {
        digitalWrite(LED_PIN, LOW);
        delay(100);
        digitalWrite(LED_PIN, HIGH);
        delay(100);
      }
    }
  }

  rpcCount = 0;
}

/* ------------------------------ JSON Publish ------------------------------ */

/**
 * @brief Publishes JSON payloads to ThingsBoard.
 * @param jsonOut JSON string to publish.
 * @return true if published successfully, false if connection failed or publish failed.
 */
bool publishJsonLineToTB(const String& jsonOut) {
  if (!mqtt.connected()) {
    if (!connectMQTT()) {
      Logln("Publish failed: MQTT unavailable.");
      return false;
    }
  }

#ifdef DEBUG
  Logln("MQTT -> Publishing to ThingsBoard:");
  Log("   Topic: ");
  Logln(MQTT_TOPIC);
  Log("   Payload: ");
  Logln(jsonOut.c_str());
#endif

  bool ok = mqtt.publish(MQTT_TOPIC, jsonOut.c_str());

  // Minimal delay for network transmission (power optimized)
  for (int i = 0; i < 3; i++) {
    mqtt.loop();
    delay(50);
  }

#ifdef DEBUG
  if (ok) {
    Logln("   Published successfully (queued)");
    Log("   MQTT State: ");
    Logln(String(mqtt.state()).c_str());
  } else {
    Logln("   Publish failed");
    Log("   MQTT State: ");
    Logln(String(mqtt.state()).c_str());
  }
#endif

  return ok;
}

#endif // GPRS_MODULE