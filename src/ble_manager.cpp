/**
 * @file ble_manager.cpp
 * @brief BLE service implementation for LilyGO A7670SA.
 *
 * This module initializes BLE, manages advertising, handles connection events,
 * receives commands from BLE clients, and triggers configuration or sensor actions.
 */

#include "ble_manager.h"

/* ------------------------------ Includes ------------------------------ */
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <EEPROM.h>
#include <Wire.h>
#include <RtcDS3231.h>

#include "utilities.h"

/* ---------------------------- BLE Settings ----------------------------- */

#define BLE_DEVICE_NAME     "LILYGO_A7670_BLE"
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define LED_PIN        32
#define SENSOR_PWR_PIN 18

#define SENSOR_RX_PIN  23
#define SENSOR_TX_PIN  19

#define DEBUG

/* ------------------------ External Dependencies ------------------------ */

struct ModuleConfig {
  int measure_time;
  int bluetooth_time;
  int GPRS_time;
};

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

extern bool           sendData;
extern HardwareSerial SensorSerial;

/* Declared in main .ino */
extern bool bleDeviceConnected;

/* ---------------------------- BLE Callbacks ---------------------------- */

/**
 * @brief Handles characteristic write events from BLE clients.
 */
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    String rxValue = pChar->getValue().c_str();
    if (rxValue.length() > 0) {
      Serial.print("📩 Command received: ");
      Serial.println(rxValue);

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
    Serial.println("🟢 BLE: Device connected");
    bleDeviceConnected = true;
  }

  void onDisconnect(BLEServer* server) override {
    Serial.println("🔴 BLE: Device disconnected");
    bleDeviceConnected = false;
    server->startAdvertising();
  }
};

/* ------------------------- Public BLE Functions ------------------------ */

/**
 * @brief Initializes BLE, configures service/characteristic and starts advertising.
 */
void startBLE() {
  Serial.println("🔵 Starting BLE...");

  BLEDevice::init(BLE_DEVICE_NAME);

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

  Serial.println("✅ BLE active and advertising");
}

/**
 * @brief Fully deinitializes the BLE stack.
 */
void stopBLE() {
  Serial.println("🟠 Stopping BLE...");
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

#ifdef DEBUG
      Serial.println("NEW CONFIGURATION:");
      Serial.println("- measure_time:   " + String(moduleCfg.measure_time));
      Serial.println("- bluetooth_time: " + String(moduleCfg.bluetooth_time));
      Serial.println("- GPRS_time:      " + String(moduleCfg.GPRS_time));
      Serial.println();
#endif

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

#ifdef DEBUG
    Serial.print("New date: ");
    rtcNow = rtc.GetDateTime();
    formatDateTime(rtcNow);
    Serial.println(dateStr);
#endif

    digitalWrite(LED_PIN, LOW);  delay(200);
    digitalWrite(LED_PIN, HIGH);
  }

  /* ------------------------- Reserved command C ------------------------- */
  else if (cmd == "C") {
    // Reserved for future use
  }

  /* ------------------------------ SEND --------------------------------- */
  else if (cmd == "SEND") {
    SensorSerial.begin(9600, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);
    digitalWrite(SENSOR_PWR_PIN, HIGH);
    delay(160);
    sendData = true;
  }

  /* ------------------------------ STOP --------------------------------- */
  else if (cmd == "STOP") {
    digitalWrite(SENSOR_PWR_PIN, LOW);
    sendData = false;
  }

  /* ------------------------------ DATA --------------------------------- */
  else if (cmd == "DATA") {
#ifdef DEBUG
    Serial.println("DOWNLOADING DATA (placeholder)");
#endif
  }

  /* -------------------------- Unknown command --------------------------- */
  else {
    for (int i = 0; i < 3; i++) {
      digitalWrite(LED_PIN, LOW);  delay(100);
      digitalWrite(LED_PIN, HIGH); delay(100);
    }
  }
}
