/**
 * @file ble_manager.cpp
 * @brief BLE service implementation for LilyGO A7670SA.
 *
 * This module initializes BLE, manages advertising, handles connection events,
 * receives commands from BLE clients, and triggers configuration or sensor actions.
 *
 * BLE is always included (non-toggleable).
 */

#include "ble_manager.h"

/* ------------------------------ Includes ------------------------------ */
#include <esp_bt.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <EEPROM.h>
#include <Wire.h>
#include <RtcDS3231.h>

#include "n_logger_config.h"
#include "config.h"
#include "error_sensor.h"
#include "utilities.h"

/* ------------------------ External Dependencies ------------------------ */

extern ModuleConfig moduleCfg;

extern RtcDS3231<TwoWire> rtc;
extern RtcDateTime        rtcNow;
extern char               dateStr[20];

/** Formats RTC time into dateStr (defined in main module). */
void formatDateTime(const RtcDateTime& dt);

/* BLE globals shared with main */
extern bool           bleActive;
extern unsigned long  bleStartTime;
extern bool           commandMode;
extern unsigned long  commandStart;

extern BLEServer*        pServer;
extern BLECharacteristic* pCharacteristic;

/* Declared in main module */
extern bool bleDeviceConnected;

RTC_DATA_ATTR bool bleStreamingActive = false;

/* ---------------------------- BLE Callbacks ---------------------------- */

/**
 * @brief Handles characteristic write events from BLE clients.
 */
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String rxValue = pChar->getValue().c_str();
    if (rxValue.length() > 0) {
      Log("Command received: ");
      Logln(rxValue.c_str());
      bleStartTime = millis();
      processCommand(rxValue);
    }
  }
};

/**
 * @brief Handles connect/disconnect events.
 */
class ServerConnectionCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    Logln("BLE: Device connected");
    bleDeviceConnected = true;
    digitalWrite(LED_PIN, HIGH);
  }

  void onDisconnect(BLEServer* server) override {
    Logln("BLE: Device disconnected");
    bleDeviceConnected = false;
    digitalWrite(LED_PIN, LOW);
    server->startAdvertising();
  }
};

/* ------------------------- Public BLE Functions ------------------------ */

/**
 * @brief Initializes BLE, configures service/characteristic and starts advertising.
 */
void startBLE() {
  Logln("Starting BLE...");

  BLEDevice::init(N_LOGGER_BLE_NAME);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerConnectionCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->start();

  bleActive    = true;
  bleStartTime = millis();

  Logln("BLE active and advertising");

  if (!bleDeviceConnected) {
    for (int i = 0; i < 10 && !bleDeviceConnected; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);

      if (bleDeviceConnected) {
        break;
      }

      digitalWrite(LED_PIN, LOW);
      delay(100);
    }

    if (bleDeviceConnected) {
      digitalWrite(LED_PIN, HIGH);
    }
  }
}

/**
 * @brief Fully deinitializes the BLE stack.
 */
void stopBLE() {
  Logln("Stopping BLE...");
  BLEDevice::deinit(true);
  bleActive = false;
}

/* --------------------------- Command Handling -------------------------- */

/**
 * @brief Parses and executes commands received over BLE.
 *
 * Supported commands:
 *  - `T-A-B-C` updates measurement, BLE, and GPRS intervals.
 *  - `D-YYYY-MM-DD-HH-MM-SS` sets RTC date/time.
 *  - `SEND` starts UART sensor streaming via BLE notifications.
 *  - `STOP` disables sensor streaming.
 *  - `DATA` placeholder for future data actions.
 *
 * LED blink is used for simple acknowledgement.
 */
void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  /* ---------------------- T-A-B-C : Timing Config ---------------------- */
  if (cmd.length() > 2 && cmd[0] == 'T' && cmd[1] == '-') {
    int i1 = cmd.indexOf('-', 2);
    int i2 = cmd.indexOf('-', i1 + 1);
    if (i1 != -1 && i2 != -1) {

      String v1 = cmd.substring(2, i1);
      String v2 = cmd.substring(i1 + 1, i2);
      String v3 = cmd.substring(i2 + 1);

      moduleCfg.measure_time   = v1.toInt();
      moduleCfg.bluetooth_time = v2.toInt();
      moduleCfg.GPRS_time      = v3.toInt();

      EEPROM.begin(sizeof(ModuleConfig));
      EEPROM.put(0, moduleCfg);
      EEPROM.commit();
      EEPROM.end();

      Logln("NEW CONFIGURATION:");
      Logf("- measure_time:   %d\n", moduleCfg.measure_time);
      Logf("- bluetooth_time: %d\n", moduleCfg.bluetooth_time);
      Logf("- GPRS_time:      %d\n", moduleCfg.GPRS_time);

      digitalWrite(LED_PIN, LOW);  delay(200);
      digitalWrite(LED_PIN, HIGH);
    }
  }

  /* ----------------------- D-YYYY-MM-DD-HH-MM-SS ----------------------- */
  else if (cmd.length() > 2 && cmd[0] == 'D' && cmd[1] == '-') {
    int year   = atoi(cmd.substring(2, 6).c_str());
    int month  = atoi(cmd.substring(7, 9).c_str());
    int day    = atoi(cmd.substring(10, 12).c_str());
    int hour   = atoi(cmd.substring(13, 15).c_str());
    int minute = atoi(cmd.substring(16, 18).c_str());
    int second = atoi(cmd.substring(19, 21).c_str());

    RtcDateTime newDT(year, month, day, hour, minute, second);
    rtc.SetDateTime(newDT);

    Log("New date: ");
    rtcNow = rtc.GetDateTime();
    formatDateTime(rtcNow);
    Logln(dateStr);

    digitalWrite(LED_PIN, LOW);  delay(200);
    digitalWrite(LED_PIN, HIGH);
  }

  else if (cmd == "START") {
    bleStreamingActive = true;
    Logln("Streaming ENABLED - all Serial output will be mirrored over BLE");
    digitalWrite(LED_PIN, LOW);  delay(200);
    digitalWrite(LED_PIN, HIGH);
  }

  else if (cmd == "STOP") {
    bleStreamingActive = false;
    Logln("Streaming DISABLED - Serial output will NOT be mirrored over BLE");
    digitalWrite(LED_PIN, LOW);  delay(200);
    digitalWrite(LED_PIN, HIGH);
  }

  else if (cmd == "C") {
    // Reserved for future use
  }

  /* ------------------------------ DATA --------------------------------- */
  else if (cmd == "DATA") {
    Logln("DOWNLOADING DATA (placeholder)");
  }

  /* -------------------------- Unknown command --------------------------- */
  else {
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_PIN, LOW);  delay(100);
      digitalWrite(LED_PIN, HIGH); delay(100);
    }
  }
}

/* ---------------------------------------------------------------------- */
/*  Serial-to-BLE streaming helpers                                       */
/* ---------------------------------------------------------------------- */

static void bleNotifyIfStreaming(const char *s) {
#if !defined(NO_BLE_STREAMING)
  if (bleStreamingActive && bleDeviceConnected && pCharacteristic != nullptr) {
    pCharacteristic->setValue((uint8_t*)s, strlen(s));
    pCharacteristic->notify();
  }
#else
  (void)s;
#endif
}

void Log(const char *s) {
  Serial.print(s);
  bleNotifyIfStreaming(s);
}

void Logln(const char *s) {
  Serial.println(s);
  String withNl = String(s) + "\r\n";
  bleNotifyIfStreaming(withNl.c_str());
}

void Logf(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (len > 0) {
    Serial.print(buf);
    bleNotifyIfStreaming(buf);
  }
}