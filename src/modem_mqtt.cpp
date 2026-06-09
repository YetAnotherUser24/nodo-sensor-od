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
#include "utilities.h"

#include <Arduino.h>
#include <Wire.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <RtcDS3231.h>

#include "config.h"

#define SerialMon Serial
#define SerialAT  Serial1

/* ------------------------- External Definitions ------------------------- */

extern ModuleConfig moduleCfg;
extern TinyGsm       modem;
extern PubSubClient  mqtt;

extern RtcDS3231<TwoWire> rtc;
extern RtcDateTime        rtcNow;
extern char               dateStr[20];

extern bool firstRun;
extern bool forceModemReset;

void formatDateTime(const RtcDateTime& dt);

/* ----------------------- Internal RPC Queue Structure -------------------- */

struct RpcMsg {
  String topic;
  String payload;
};

static RpcMsg  rpcQueue[8];
static uint8_t rpcCount = 0;

/* ---------------------------- JSON Helpers ------------------------------- */

/**
 * @brief Extracts the "method" field from a ThingsBoard RPC payload.
 */
static String getRpcMethod(const String& payload) {
  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, payload)) {
    Serial.println("JSON parse error");
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
      rpcQueue[rpcCount].topic   = t;
      rpcQueue[rpcCount].payload = p;
      rpcCount++;
    }

    Serial.println("📥 RPC queued -> " + t + " :: " + p);

    // Send ACK response
    int slash = t.lastIndexOf('/');
    if (slash > 0) {
      String reqId     = t.substring(slash + 1);
      String respTopic = "v1/devices/me/rpc/response/" + reqId;

      StaticJsonDocument<64> doc;
      doc["ok"] = true;
      String out;
      serializeJson(doc, out);

#ifdef DEBUG
      Serial.println("📤 MQTT → RPC Response:");
      Serial.print("   Topic: ");
      Serial.println(respTopic);
      Serial.print("   Payload: ");
      Serial.println(out);
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
      Serial.println("🔄 Performing forced modem reset...");
    }
#endif

#ifdef MODEM_RESET_PIN
    gpio_hold_dis((gpio_num_t)MODEM_RESET_PIN);
    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL); delay(100);
    digitalWrite(MODEM_RESET_PIN,  MODEM_RESET_LEVEL); delay(2600);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
#endif

    pinMode(MODEM_DTR_PIN, OUTPUT);
    digitalWrite(MODEM_DTR_PIN, LOW);

    pinMode(BOARD_PWRKEY_PIN, OUTPUT);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);   delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, HIGH);  delay(1000);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);

    forceModemReset = false;  // Clear flag after reset

  } else {
    Serial.println("Wakeup modem !");
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

  Serial.println("Start modem...");
  delay(3000);

  if (!modem.init()) {
    DBG("Failed to init modem");
    return false;
  }

  DBG("Modem Name:", modem.getModemName());
  DBG("Modem Info:", modem.getModemInfo());

  if (GSM_PIN && modem.getSimStatus() != 3) {
    modem.simUnlock(GSM_PIN);
  }

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return false;
  }
  SerialMon.println(" success");

  if (!modem.gprsConnect(APN, APN_USER, APN_PASS)) {
    SerialMon.println("GPRS attach fail");
    delay(10000);
    return false;
  }

  SerialMon.println("GPRS connected");

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  bool mqttOk = connectMQTT();

  if (!mqttOk) {
    SerialMon.println("MQTT connection failed, continuing in offline mode.");
  }

  return mqttOk;
}

/* ---------------------------- Modem Sleep Mode --------------------------- */

/**
 * @brief Puts the modem into full low-power mode using DTR and GPIO holds.
 */
void sleepA7670() {
  if (mqtt.connected()) mqtt.disconnect();

  modem.gprsDisconnect();
  Serial.println("Enter modem sleep mode!");

  pinMode(MODEM_DTR_PIN, OUTPUT);
  digitalWrite(MODEM_DTR_PIN, HIGH);
  gpio_hold_en((gpio_num_t)MODEM_DTR_PIN);
  gpio_deep_sleep_hold_en();

  if (!modem.sleepEnable(true)) {
    Serial.println("Modem sleep failed!");
  } else {
    Serial.println("Modem entered sleep mode.");
  }

  delay(1000);

  uint32_t t0 = millis();
  while (modem.testAT() && (millis() - t0) < 5000UL) {
    Serial.print(".");
    delay(250);
  }
  Serial.println("\nModem is not responding, modem has slept!");

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

  SerialMon.print("Connecting MQTT...");

  for (uint8_t attempt = 1; attempt <= maxAttempts; attempt++) {
    SerialMon.printf(" (attempt %u/%u)", attempt, maxAttempts);

    if (mqtt.connect(MQTT_CLIENT_ID, MQTT_TOKEN, "")) {
      SerialMon.println(" ✓ connected.");
      mqtt.setCallback(onMqttMessage);
      mqtt.subscribe(TB_RPC_SUB_TOPIC);
      return true;
    }

    SerialMon.print(".");

    if (attempt < maxAttempts) {
      SerialMon.printf(", retrying in %ums...\n", delayMs);
      delay(delayMs);
    }
  }

  SerialMon.println(" All attempts failed.");
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
    Serial.println("ℹ️ No RPC queued.");
    return;
  }

  Serial.printf("🧾 Processing %u RPC command(s):\n", rpcCount);

  for (uint8_t i = 0; i < rpcCount; i++) {

    Serial.printf("  [%u] Topic: %s\n", i + 1, rpcQueue[i].topic.c_str());
    Serial.printf("      Payload: %s\n", rpcQueue[i].payload.c_str());

    String cmd = getRpcMethod(rpcQueue[i].payload);
    cmd.trim();
    cmd.toUpperCase();

    /* ---- Timer update: T-x-y-z ---- */
    if (cmd.startsWith("T-")) {
      int i1 = cmd.indexOf('-', 2);
      int i2 = cmd.indexOf('-', i1 + 1);

      if (i1 != -1 && i2 != -1) {
        moduleCfg.measure_time   = cmd.substring(2, i1).toInt();
        moduleCfg.bluetooth_time = cmd.substring(i1 + 1, i2).toInt();
        moduleCfg.GPRS_time      = cmd.substring(i2 + 1).toInt();

        EEPROM.begin(sizeof(ModuleConfig));
        EEPROM.put(0, moduleCfg);
        EEPROM.commit();
        EEPROM.end();

#ifdef DEBUG
        Serial.println("Updated configuration:");
        Serial.printf(" - Measure:   %d\n", moduleCfg.measure_time);
        Serial.printf(" - Bluetooth: %d\n", moduleCfg.bluetooth_time);
        Serial.printf(" - GPRS:      %d\n\n", moduleCfg.GPRS_time);
#endif
      }

      digitalWrite(LED_PIN, LOW); delay(500);
      digitalWrite(LED_PIN, HIGH);

    /* ---- RTC update: D-YYYY-MM-DD-hh-mm-ss ---- */
    } else if (cmd.startsWith("D-")) {

      int year   = atoi(cmd.substring(2, 6).c_str());
      int month  = atoi(cmd.substring(7, 9).c_str());
      int day    = atoi(cmd.substring(10, 12).c_str());
      int hour   = atoi(cmd.substring(13, 15).c_str());
      int minute = atoi(cmd.substring(16, 18).c_str());
      int second = atoi(cmd.substring(19, 21).c_str());

      RtcDateTime newDt(year, month, day, hour, minute, second);
      rtc.SetDateTime(newDt);

#ifdef DEBUG
      Serial.print("New date: ");
      rtcNow = rtc.GetDateTime();
      formatDateTime(rtcNow);
      Serial.println(dateStr);
#endif

      digitalWrite(LED_PIN, LOW); delay(500);
      digitalWrite(LED_PIN, HIGH);

    } else {
      // Unknown RPC command → blink three times
      for (int j = 0; j < 3; j++) {
        digitalWrite(LED_PIN, LOW);  delay(100);
        digitalWrite(LED_PIN, HIGH); delay(100);
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
      SerialMon.println("Publish failed: MQTT unavailable.");
      return false;
    }
  }

#ifdef DEBUG
  Serial.println("📤 MQTT → Publishing to ThingsBoard:");
  Serial.print("   Topic: ");
  Serial.println(MQTT_TOPIC);
  Serial.print("   Payload: ");
  Serial.println(jsonOut);
#endif

  bool ok = mqtt.publish(MQTT_TOPIC, jsonOut.c_str());

  // Minimal delay for network transmission (power optimized)
  for (int i = 0; i < 3; i++) {
    mqtt.loop();
    delay(50);
  }

#ifdef DEBUG
  if (ok) {
    Serial.println("   ✅ Published successfully (queued)");
    Serial.print("   MQTT State: ");
    Serial.println(mqtt.state());
  } else {
    Serial.println("   ❌ Publish failed");
    Serial.print("   MQTT State: ");
    Serial.println(mqtt.state());
  }
#endif

  return ok;
}

#endif // GPRS_MODULE