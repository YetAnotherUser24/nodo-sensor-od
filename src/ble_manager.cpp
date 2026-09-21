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

#if !defined(DISABLE_BLE)

/* ------------------------------ Includes ------------------------------ */
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <EEPROM.h>
#include <RtcDS3231.h>
#include <Wire.h>
#include <esp_bt.h>

#include "config.h"
#include "n_logger_config.h"
#include "board.h"
#include "measurement_storage.h"
#include "oled_display.h"
#include "rtc_utils.h"

/* ------------------------ External Dependencies ------------------------ */

extern ModuleConfig moduleCfg;

extern RtcDS3231<TwoWire> rtc;
extern RtcDateTime rtcNow;
extern char dateStr[20];

/** Formats RTC time into dateStr (defined in main module). */
void formatDateTime(const RtcDateTime& dt);

/* BLE globals shared with main */
extern bool bleActive;
extern unsigned long bleStartTime;
extern bool commandMode;
extern unsigned long commandStart;

extern BLEServer* pServer;
extern BLECharacteristic* pCharacteristic;
BLECharacteristic* pNotifyCharacteristic = nullptr;

/* Declared in main module */
extern bool bleDeviceConnected;

RTC_DATA_ATTR bool bleStreamingActive = false;
bool bleRealtimeSendActive = false;
uint32_t bleMonitorIntervalMs = 5000;
bool bleRestartAdvertisingRequested = false;

/** Tracks whether a BLE client has ever connected (for better log messages). */
bool bleWasEverConnected = false;

/* --------------------- LED Blink State Machine ------------------------ */

static uint8_t bleLedBlinks = 0;
static unsigned long bleLedLastToggle = 0;
static bool bleLedState = false;

/** Guard flag: true while executing inside a BLE callback (onConnect/onDisconnect/onWrite).
 *  Prevents bleNotifyIfStreaming() from calling BLE APIs (notify) from callback context,
 *  which would deadlock the BLE task. */
static volatile bool bleInCallback = false;

void bleProcessLedBlinks() {
  if (bleLedBlinks == 0)
    return;
  unsigned long now = millis();

  // First call: turn LED ON and start the 200ms timer
  if (bleLedLastToggle == 0) {
    bleLedLastToggle = now;
    bleLedState = true;
    digitalWrite(LED_PIN, LOW); // LED ON (active-low)
    return;
  }

  if (now - bleLedLastToggle >= 200) {
    bleLedLastToggle = now;
    if (bleLedState) {
      // Was ON → turn OFF
      digitalWrite(LED_PIN, HIGH);
      bleLedState = false;
    } else {
      // Was OFF → turn ON, decrement blink count
      bleLedBlinks--;
      if (bleLedBlinks == 0) {
        digitalWrite(LED_PIN, HIGH); // ensure LED ends OFF
        bleLedLastToggle = 0;        // reset for next command
        return;
      }
      digitalWrite(LED_PIN, LOW);
      bleLedState = true;
    }
  }
}

/* ---------------------------- BLE Callbacks ---------------------------- */

/**
 * @brief Handles characteristic write events from BLE clients.
 */
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    bleInCallback = true;
    std::string val = pChar->getValue();
    if (!val.empty()) {
      const char* cmd = val.c_str();
      Log("Command received: ");
      Logln(cmd);
      bleStartTime = millis();
      processCommand(String(cmd)); // one-time String creation, passed by value then consumed
    }
    bleInCallback = false;
  }
};

/**
 * @brief Handles connect/disconnect events.
 */
class ServerConnectionCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    bleInCallback = true;
    Logln("BLE: Device connected");
    bleDeviceConnected = true;
    bleWasEverConnected = true;
    bleRestartAdvertisingRequested = false;
    digitalWrite(LED_PIN, HIGH);
    bleInCallback = false;
  }

  void onDisconnect(BLEServer* server) override {
    bleInCallback = true;
    Logln("BLE: Device disconnected");
    bleDeviceConnected = false;
    bleRestartAdvertisingRequested = true;
    if (!bleRealtimeSendActive) {
      oledPowerOff();
    }
    // Refresh active window so advertising remains alive for reconnect
    bleStartTime = millis();
    digitalWrite(LED_PIN, LOW);
    bleInCallback = false;
  }
};

/* ------------------------- Public BLE Functions ------------------------ */

/**
 * @brief Initializes BLE, configures service/characteristics and starts advertising.
 */
void startBLE() {
  if (bleActive) {
    Logln("BLE already active.");
    return;
  }

  Logln("Starting BLE...");

  BLEDevice::init(N_LOGGER_BLE_NAME);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerConnectionCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  // COMMAND & NOTIFY characteristic (beb5483e-36e1-4688-b7f5-ea07361b26a8) for commands & streaming
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
                               BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());

  // NOTIFY/READ characteristic (beb5483e-36e1-4688-b7f5-ea07361b26a9) for real-time measurements
  pNotifyCharacteristic = pService->createCharacteristic(CHARACTERISTIC_NOTIFY_UUID,
                                                         BLECharacteristic::PROPERTY_READ |
                                                             BLECharacteristic::PROPERTY_NOTIFY);
  pNotifyCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  pServer->getAdvertising()->start();

  bleActive = true;
  bleRestartAdvertisingRequested = false;
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

bool isBleNotifyEnabled() {
  if (!bleActive || !bleDeviceConnected || pServer == nullptr || pNotifyCharacteristic == nullptr) {
    return false;
  }
  if (pServer->getConnectedCount() == 0) {
    return false;
  }
  BLE2902* p2902 = (BLE2902*)pNotifyCharacteristic->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
  if (p2902 != nullptr && !p2902->getNotifications()) {
    return false;
  }
  return true;
}

/**
 * @brief Safely stops BLE advertising and clears BLE state flags without crashing.
 */
void stopBLE() {
  if (!bleActive)
    return;
  Logln("Stopping BLE...");

  if (pServer != nullptr) {
    pServer->getAdvertising()->stop();
    delay(50);
  }

  pServer = nullptr;
  pCharacteristic = nullptr;
  pNotifyCharacteristic = nullptr;
  bleActive = false;
  bleDeviceConnected = false;
  bleRealtimeSendActive = false;
  bleRestartAdvertisingRequested = false;
  bleWasEverConnected = false;
}

/* --------------------------- Command Handling -------------------------- */

/**
 * @brief Parses and executes commands received over BLE.
 *
 * Supported commands:
 *  - `T-A-B-C` updates measurement, BLE, and GPRS intervals.
 *  - `D-YYYY-MM-DD-HH-MM-SS` sets RTC date/time.
 *  - `SEND` starts real-time distance sensor notifications (values like "1645").
 *  - `STOP` disables real-time sensor notifications.
 *  - `DATA` placeholder for future data actions.
 *
 * LED blink is used for simple acknowledgement.
 */
void processCommand(const String& cmdIn) {
  // Work on a mutable uppercase copy (single allocation)
  String cmd = cmdIn;
  cmd.trim();
  cmd.toUpperCase();
  const char* c = cmd.c_str();
  int len = cmd.length();

  /* ---------------------- T-A-B-C : Timing Config ---------------------- */
  if (len > 2 && c[0] == 'T' && c[1] == '-') {
    // Parse with pointer arithmetic instead of substring()
    const char* p1 = strchr(c + 2, '-');
    const char* p2 = p1 ? strchr(p1 + 1, '-') : nullptr;
    if (p1 && p2) {
      moduleCfg.measure_time = atoi(c + 2);    // c+2 .. p1
      moduleCfg.bluetooth_time = atoi(p1 + 1); // p1+1 .. p2
      moduleCfg.GPRS_time = atoi(p2 + 1);      // p2+1 .. end

      EEPROM.begin(sizeof(ModuleConfig));
      EEPROM.put(0, moduleCfg);
      EEPROM.commit();
      EEPROM.end();

      Logln("NEW CONFIGURATION:");
      Logf("- measure_time:   %d\n", moduleCfg.measure_time);
      Logf("- bluetooth_time: %d\n", moduleCfg.bluetooth_time);
      Logf("- GPRS_time:      %d\n", moduleCfg.GPRS_time);

      bleLedBlinks = 1;
    }
  }

  /* ----------------------- D-YYYY-MM-DD-HH-MM-SS ----------------------- */
  else if (len > 2 && c[0] == 'D' && c[1] == '-') {
    // Parse with pointer arithmetic: positions 2,7,10,13,16,19 are fixed
    int year = atoi(c + 2);
    int month = atoi(c + 7);
    int day = atoi(c + 10);
    int hour = atoi(c + 13);
    int minute = atoi(c + 16);
    int second = atoi(c + 19);

    RtcDateTime newDT(year, month, day, hour, minute, second);
    setRtcDateTime(newDT);

    Log("New date: ");
    formatDateTime(rtcNow);
    Logln(dateStr);

    bleLedBlinks = 1;
  }

  /* ------------------- MONITOR / SEND [ -<seconds> ] -------------------- */
  else if (strncmp(c, "MONITOR", 7) == 0 || strncmp(c, "SEND", 4) == 0 || strncmp(c, "REC", 3) == 0) {
    uint32_t intervalSec = 5; // default 5 seconds
    const char* dash = strchr(c, '-');
    if (dash != nullptr) {
      int parsed = atoi(dash + 1);
      if (parsed >= 1 && parsed <= 3600) {
        intervalSec = static_cast<uint32_t>(parsed);
      }
    }
    bleMonitorIntervalMs = intervalSec * 1000UL;
    bleRealtimeSendActive = true;
    oledShowMonitorStart(intervalSec);
    Logf("Continuous monitoring & recording ENABLED (interval: %u s)\n", intervalSec);
    bleLedBlinks = 1;
  }

  /* ------------------------------- STOP --------------------------------- */
  else if (cmd == "STOP") {
    bleRealtimeSendActive = false;
    bleStreamingActive = false;
    oledShowMonitorStop();
    Logln("Continuous monitoring DISABLED (STOP)");
    bleLedBlinks = 1;
  }

  else if (cmd == "START") {
    bleStreamingActive = true;
    Logln("Legacy streaming ENABLED");
    bleLedBlinks = 1;
  }


  /* ------------------------------ DATA --------------------------------- */
  else if (cmd == "DATA") {
    Logln("DOWNLOADING DATA (placeholder)");
  }

  /* -------------------------- Unknown command --------------------------- */
  else {
    bleLedBlinks = 3;
  }
}

/* ---------------------------------------------------------------------- */
/*  Serial-to-BLE streaming helpers                                       */
/* ---------------------------------------------------------------------- */

static void bleNotifyIfStreaming(const char* s) {
#if !defined(NO_BLE_STREAMING)
  // Never call BLE APIs from within BLE callbacks — that would deadlock the BLE task
  if (bleInCallback)
    return;
  if (bleStreamingActive && bleDeviceConnected && pCharacteristic != nullptr) {
    pCharacteristic->setValue((uint8_t*)s, strlen(s));
    pCharacteristic->notify();
  }
#else
  (void)s;
#endif
}

void Log(const char* s) {
  // When inside a BLE callback, never block on Serial (TX buffer full => BLE task deadlock)
  if (!bleInCallback || Serial.availableForWrite() > (int)strlen(s)) {
    Serial.print(s);
  }
  bleNotifyIfStreaming(s);
}

void Logln(const char* s) {
  if (!bleInCallback || Serial.availableForWrite() > (int)(strlen(s) + 2)) {
    Serial.println(s);
  }
  bleNotifyIfStreaming(s);
  bleNotifyIfStreaming("\r\n");
}

void Logf(const char* fmt, ...) {
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

#else

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

void Log(const char* s) {
  Serial.print(s);
}

void Logln(const char* s) {
  Serial.println(s);
}

void Logf(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  if (len > 0) {
    Serial.print(buf);
  }
}

#endif // DISABLE_BLE