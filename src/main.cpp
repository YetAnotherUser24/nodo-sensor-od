/**
 * @file main.cpp
 * @brief Main application entry point for OD-Logger telemetry system.
 *
 * Dedicated to the Aqualabo DIGISENS Optical Dissolved Oxygen (OD) sensor.
 * Modular components with compile-time feature toggles (SD_MODULE, GPRS_MODULE).
 * BLE and EEPROM are always included.
 *
 * Build identity: N_LOGGER_DEVICE_NAME (default "od-logger")
 */

#include "ble_manager.h"
#include "board.h"
#include "eeprom_i2c.h"
#include "error_sensor.h"
#include "measurement_storage.h"
#include "modem_mqtt.h"
#include "n_logger_config.h"
#include "oled_display.h"
#include "power_sleep.h"
#include "rtc_utils.h"
#include "sensor_engine.h"
#include "wifi_http.h"
#include "esp_now_sender.h"

/* -------------------------- Dependencies --------------------------- */

#include <Arduino.h>
#include <EEPROM.h>
#include <RtcDS3231.h>
#include <Wire.h>

#ifdef SD_MODULE
#include <SPI.h>
#endif

#include <driver/gpio.h>
#include <esp_bt.h>
#include <esp_task_wdt.h>
#include <string.h>
#include <ArduinoJson.h>

#ifdef GPRS_MODULE
#include <WiFi.h>
#endif

#include "config.h"

/* ------------------------- BLE Includes ---------------------------- */

#if !defined(DISABLE_BLE)
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#endif

/* ------------------------- Conditional Includes -------------------- */

#ifdef SD_MODULE
#include <SdFat.h>
#endif

#ifdef GPRS_MODULE
#include <ESP_SSLClient.h>
#include <PubSubClient.h>
#include <TinyGsmClient.h>
#endif

#ifdef WIFI_MODULE
#include <WiFi.h>
#endif

/* --------------------------- BLE Status ---------------------------- */

bool bleDeviceConnected = false;

uint64_t device_ID = ESP.getEfuseMac();

/* ------------------------------ Pins ------------------------------- */

#define SerialMon Serial
#define SerialAT Serial1

/* --------------------------- Modem/Network ------------------------- */

#ifdef GPRS_MODULE
#define TINY_GSM_DEBUG SerialMon
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

#define GSM_PIN ""

#define TINY_GSM_RX_BUFFER 1024
#define MODEM_POWERON_PULSE_WIDTH_MS 100

#if TINY_GSM_USE_GPRS && not defined TINY_GSM_MODEM_HAS_GPRS
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS false
#define TINY_GSM_USE_WIFI true
#endif

#if TINY_GSM_USE_WIFI && not defined TINY_GSM_MODEM_HAS_WIFI
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#endif

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

TinyGsmClient gsmClient(modem);
PubSubClient mqtt(gsmClient);
#endif /* GPRS_MODULE */

/* ---------------------------- SD ---------------------------------- */

#ifdef SD_MODULE
SdFat SD;
SdFile sdFile;
SPIClass sdSPI(HSPI);
#endif

/* ------------------------------ RTC -------------------------------- */

RtcDS3231<TwoWire> rtc(Wire);
RtcDateTime rtcNow;
char dateStr[20];

/* ----------------------------- Globals ----------------------------- */

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR uint16_t eepromWritePage = 0;
RTC_DATA_ATTR uint8_t failedTransmissionCount = 0;
RTC_DATA_ATTR bool forceModemReset = false;

bool measureDone = false;
bool bluetoothDone = false;
bool gprsDone = false;
bool wifiAvailable = false;
bool deviceConnected = false;
bool readyForSleep = false;
bool firstRun = false;

uint32_t startMillis = 0;
bool x = false;

volatile uint16_t interruptCount = 0;
volatile bool interruptFlag = false;

byte eepromPageBuf[EEPROM_PAGE_SIZE];
bool needFlushToSD = false;

char csvFilename[24];
bool fileOpen = false;

String payload = "";

/* ------------------------- Config Structure ------------------------ */

ModuleConfig moduleCfg;

/* ------------------------- Telemetry Packet ------------------------ */

TelemetryPayload telemetryPayload;

/* ------------------------- BLE Globals ----------------------------- */

#if !defined(DISABLE_BLE)
bool bleActive = false;
unsigned long bleStartTime = 0;
bool commandMode = false;
unsigned long commandStart = 0;

BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
#endif

/* -------------------- Experiment & Web Monitor --------------------- */

bool experimentActive = false;
uint32_t experimentIntervalMs = 2000;
char activeExperimentId[40] = {0};
char activeExperimentName[48] = {0};
char activeExperimentFilename[16] = "EXP_01.CSV";

bool webMonitorActive = false;
uint32_t webMonitorIntervalMs = 5000;
bool manualSampleRequested = false;

#ifdef WIFI_MODULE
void onSensorCommandReceived(const char* cmdId, const char* action, const char* payloadJson) {
  StaticJsonDocument<384> pDoc;
  DeserializationError err = deserializeJson(pDoc, payloadJson);
  if (err) {
    Logf("[CMD] Payload parse warning: %s\n", err.c_str());
  }

  Logf("[CMD] Dispatching: %s (ID: %s)\n", action, cmdId ? cmdId : "none");

  bool success = true;
  StaticJsonDocument<256> stateDoc;

  if (strcmp(action, "start_experiment") == 0) {
    const char* expId = pDoc["experiment_id"] | "exp_default";
    const char* expName = pDoc["name"] | "Experimento";
    const char* fname = pDoc["csv_filename"] | "EXP_01.CSV";
    int intervalSec = pDoc["interval_sec"] | 2;
    if (intervalSec < 1) intervalSec = 1;
    if (intervalSec > 60) intervalSec = 60;

    strncpy(activeExperimentId, expId, sizeof(activeExperimentId) - 1);
    strncpy(activeExperimentName, expName, sizeof(activeExperimentName) - 1);
    strncpy(activeExperimentFilename, fname, sizeof(activeExperimentFilename) - 1);

    experimentIntervalMs = intervalSec * 1000UL;
    experimentActive = true;
    webMonitorActive = true;

#ifdef SD_MODULE
    startExperimentFile(activeExperimentFilename);
#endif
    oledShowMonitorStart(intervalSec);

    stateDoc["monitor_active"] = true;
    stateDoc["monitor_interval_sec"] = intervalSec;
    JsonObject expObj = stateDoc["active_experiment"].to<JsonObject>();
    expObj["id"] = activeExperimentId;
    expObj["name"] = activeExperimentName;
    expObj["sampling_rate_sec"] = intervalSec;
    expObj["csv_filename"] = activeExperimentFilename;
    expObj["status"] = "active";

  } else if (strcmp(action, "stop_experiment") == 0) {
    experimentActive = false;
    webMonitorActive = false;
#ifdef SD_MODULE
    stopExperimentFile();
#endif
    oledShowMonitorStop();

    stateDoc["monitor_active"] = false;
    stateDoc["active_experiment"] = nullptr;

  } else if (strcmp(action, "start_monitor") == 0 || strcmp(action, "start") == 0) {
    int intervalSec = pDoc["interval_sec"] | 5;
    if (intervalSec < 1) intervalSec = 1;
    if (intervalSec > 60) intervalSec = 60;

    webMonitorIntervalMs = intervalSec * 1000UL;
    webMonitorActive = true;
    oledShowMonitorStart(intervalSec);

    stateDoc["monitor_active"] = true;
    stateDoc["monitor_interval_sec"] = intervalSec;

  } else if (strcmp(action, "stop_monitor") == 0 || strcmp(action, "stop") == 0) {
    webMonitorActive = false;
    if (experimentActive) {
      experimentActive = false;
#ifdef SD_MODULE
      stopExperimentFile();
#endif
    }
    oledShowMonitorStop();

    stateDoc["monitor_active"] = false;
    stateDoc["active_experiment"] = nullptr;

  } else if (strcmp(action, "set_sampling_rate") == 0) {
    int intervalSec = pDoc["interval_sec"] | 5;
    if (intervalSec < 1) intervalSec = 1;
    if (intervalSec > 60) intervalSec = 60;

    webMonitorIntervalMs = intervalSec * 1000UL;
    if (experimentActive) {
      experimentIntervalMs = intervalSec * 1000UL;
    }
    stateDoc["monitor_interval_sec"] = intervalSec;

  } else if (strcmp(action, "manual_sample") == 0) {
    manualSampleRequested = true;
    Logln("[CMD] Manual sample queued for execution in loop().");
    stateDoc["manual_sample"] = "queued";

  } else if (strcmp(action, "set_sleep_cycle") == 0) {
    int cycleMin = pDoc["measure_time_min"] | 15;
    if (cycleMin >= 1 && cycleMin <= 1440) {
      moduleCfg.measure_time = cycleMin;
      EEPROM.begin(sizeof(ModuleConfig));
      EEPROM.put(0, moduleCfg);
      EEPROM.commit();
      EEPROM.end();
      Logf("[CONFIG] Sleep cycle updated to %d min\n", cycleMin);
    }
    stateDoc["sleep_cycle_min"] = moduleCfg.measure_time;

  } else if (strcmp(action, "sleep") == 0) {
    int sleepMin = pDoc["minutes"] | 30;
    bool indefinite = pDoc["indefinite"] | false;
    Logf("[PWR] Sleep command: %d min (indefinite: %d)\n", sleepMin, indefinite ? 1 : 0);

    stateDoc["monitor_active"] = false;
    String stateStr;
    serializeJson(stateDoc, stateStr);
    acknowledgeSensorCommand(cmdId, true, stateStr.c_str());

    if (indefinite) {
      prepareForDeepSleep(0);
    } else {
      prepareForDeepSleep(sleepMin);
    }
    return;
  }

  String stateStr;
  serializeJson(stateDoc, stateStr);
  acknowledgeSensorCommand(cmdId, success, stateStr.c_str());
}
#endif

/* ---------------------------------------------------------------- */
/*                           HELPERS                                */
/* ---------------------------------------------------------------- */

#define countof(a) (sizeof(a) / sizeof(a[0]))

/* ============================== Setup ============================== */

/**
 * @brief Initializes pins, RTC, SD, configuration and communication modules.
 */
void setup() {
  // Release hardware GPIO hold locks latched from deep sleep
  gpio_deep_sleep_hold_dis();

  boardInit();

  // Always initialize Serial so Log()/Logln()/Logf() work regardless of DEBUG
  Serial.begin(115200);

// Initialize Task Watchdog Timer (60s) for hardware auto-recovery
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 60000,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
      .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(&wdt_config);
  esp_task_wdt_add(NULL);
#else
  esp_task_wdt_init(60, true);
  esp_task_wdt_add(NULL);
#endif

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

#ifdef DEBUG
  Logln("Scanning I2C bus...");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Log("Found I2C device at 0x");
      if (address < 16)
        Log("0");
      Logln(String(address, HEX).c_str());
    }
  }
#endif

#ifdef EEPROM_USE_WIRE1
  Wire1.begin(I2C_EEPROM_SDA_PIN, I2C_EEPROM_SCL_PIN);
#endif

  // Load config from EEPROM
  EEPROM.begin(sizeof(ModuleConfig));
  EEPROM.get(0, moduleCfg);
  EEPROM.end();

  normalizeConfig();

  // Initialize ESP-NOW for direct controller link
  espNowSenderInit();

  // RTC init
  rtcInit();
  rtcNow = getRtcDateTime();

#ifdef WIFI_MODULE
  setSensorCommandCallback(onSensorCommandReceived);

  // If time is not calibrated (year < 2020), synchronize via WiFi NTP
  if (rtcNow.Year() < 2020) {
    Logln("RTC time uncalibrated. Attempting initial WiFi NTP sync...");
    syncTimeNtp(10000);
    rtcNow = getRtcDateTime();
  }
#endif

  // SD bus init
#ifdef SD_MODULE
#if SD_EN_PIN != -1
  setPinActive(SD_EN_PIN, SD_EN_ACTIVE_HIGH);
  delay(10); // let SD module power stabilize
#endif
  sdSPI.begin(SD_PIN_SCLK, SD_PIN_MISO, SD_PIN_MOSI, SD_PIN_CS);
#endif

  // Make sure radios are off at startup
  boardRadiosOff();

#ifdef DEBUG
  Serial.println();
  Serial.printf("WELCOME TO %s ", N_LOGGER_DEVICE_NAME);
  Serial.printf("ChipID: %04X%08X\n", (uint16_t)(device_ID >> 32), (uint32_t)device_ID);
#ifdef SD_MODULE
  Serial.print("SD");
#endif
#ifdef GPRS_MODULE
  Serial.print("GPRS");
#endif
  Serial.printf(" v%d.%d\n", CODE_VERSION >> 4, CODE_VERSION & 0x0F);

  Serial.print(F("Sensor type: "));
  Serial.println(SENSOR_NAME);

  Serial.print(F("Current time is "));
  formatDateTime(rtcNow);
  Serial.print(dateStr);
  Serial.printf(" %s\n\n", LOCAL_TZ_NAME);

  Serial.println("CURRENT CONFIGURATION:");
  Serial.print("(Measure time: ");
  Serial.print(moduleCfg.measure_time);

#ifdef GPRS_MODULE
  Serial.print(" | GPRS time: ");
  Serial.print(moduleCfg.GPRS_time);
#endif

  Serial.println(")\n");
#endif

  // Wake-up flow
  esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
  if (wakeupCause == ESP_SLEEP_WAKEUP_EXT1 || wakeupCause == ESP_SLEEP_WAKEUP_TIMER) {
    rtcNow = getRtcDateTime();

#ifdef WIFI_MODULE
    if (rtcNow.Year() < 2020) {
      syncTimeNtp(5000);
      rtcNow = getRtcDateTime();
    }
#endif

    // Optional daily flush at 00:00 (only when SD and EEPROM are present)
#ifdef SD_MODULE
    if (isEepromAvailable() && rtcNow.Hour() == 0 && rtcNow.Minute() == 0) {
      if (dumpEEPROM()) {
        resetEEPromHeader(EEPROM_ADDRESS);
        eepromWritePage = 0;
#ifdef DEBUG
        Logln("- EEPROM flushed.");
#endif
      } else {
#ifdef DEBUG
        Logln("Failed to flush EEPROM. SD card missing? Continuing anyway.");
#endif
        error(3, LED_PIN);
      }
    }
#endif

    // Measurement window
    if (moduleCfg.measure_time > 0 && (rtcNow.Minute() % moduleCfg.measure_time == 0)) {
#ifdef DEBUG
      Logln("SAVE MEASURE LEVEL IN EEPROM");
#endif
      esp_task_wdt_reset();
      takeMeasurement();
      esp_task_wdt_reset();
      Logln("MT TIME...");
    }

#if !defined(DISABLE_BLE) && !defined(WEB_MASTER_MODE)
    /* Bluetooth window — forced ON if streaming is active */
    if (moduleCfg.bluetooth_time > 0 && (rtcNow.Minute() % moduleCfg.bluetooth_time == 0)) {
#ifdef DEBUG
      Logln("TRY TO CONNECT TO BLUETOOTH");
#endif
      digitalWrite(LED_PIN, LOW);
      esp_task_wdt_reset();
      startBLE();
    }

    /* Force BLE ON every cycle if streaming was previously active */
    if (bleStreamingActive) {
      esp_task_wdt_reset();
      startBLE();
    }
#endif

#ifdef WIFI_MODULE
    if (moduleCfg.GPRS_time > 0 && (rtcNow.Minute() % moduleCfg.GPRS_time == 0)) {
#ifdef DEBUG
      Logln("TRY TO CONNECT TO WIFI AND SEND STORED DATA");
#endif
      esp_task_wdt_reset();
      wifiAvailable = wifiConnect();
      esp_task_wdt_reset();

      if (wifiAvailable) {
        failedTransmissionCount = 0;

        sendSDPending();
        uint16_t sentPages = sendEEPROMPending();
#if defined(SENSOR_VIRTUAL)
        if (sentPages == 0) {
          publishToAquaControl(rtcNow.TotalSeconds(), telemetryPayload.sensorReadings,
                               telemetryPayload.temperature, telemetryPayload.status);
        }
#endif

        // ponytail: No RPC replacement needed. BLE commands cover T-x-y-z and D-YYYY-MM-DD.
      } else {
        failedTransmissionCount++;

#ifdef DEBUG
        Log("WiFi unavailable. Data remains in offline storage. (Failure ");
        Log(String(failedTransmissionCount).c_str());
        Log("/");
        Log(String(MAX_FAILED_TRANSMISSIONS).c_str());
        Logln(")");
#endif

        if (failedTransmissionCount >= MAX_FAILED_TRANSMISSIONS) {
#ifdef DEBUG
          Logln("Too many consecutive transmission failures!");
#endif
          failedTransmissionCount = 0;
        }
      }

      gprsDone = true;
    }
#endif /* WIFI_MODULE */

  } else {
    // First boot after power-on/reset
    firstRun = true;

#if !defined(WEB_MASTER_MODE)
#ifdef SD_MODULE
    if (isEepromAvailable()) {
#ifdef DEBUG
      uint16_t freePages = getEEPROMFreePages();
      uint16_t totalPages = EEPROM_MAX_PAGES - EEPROM_HEADER_SIZE;
      Logf("FLUSHING EEPROM TO SD CARD (Used: %u / %u pages)...\n", totalPages - freePages,
           totalPages);
#endif

      if (dumpEEPROM()) {
        resetEEPromHeader(EEPROM_ADDRESS);
#ifdef DEBUG
        Logln("- EEPROM flushed.");
#endif
      } else {
#ifdef DEBUG
        Logln("Failed to flush EEPROM. SD card missing? Continuing anyway.");
#endif
        error(3, LED_PIN);
      }
    }
#endif

    esp_task_wdt_reset();
    delay(1000);

#if defined(DEBUG) && defined(GPRS_MODULE)
    Logln("- Connecting to Internet: ");
#endif

    Logln("First run...");
    esp_task_wdt_reset();
    takeMeasurement();
    esp_task_wdt_reset();

#ifdef WIFI_MODULE
    wifiAvailable = wifiConnect();
    esp_task_wdt_reset();

    if (wifiAvailable) {
      failedTransmissionCount = 0;

      sendSDPending();
      uint16_t sentPages = sendEEPROMPending();
#if defined(SENSOR_VIRTUAL)
      if (sentPages == 0) {
        publishToAquaControl(rtcNow.TotalSeconds(), telemetryPayload.sensorReadings,
                             telemetryPayload.temperature, telemetryPayload.status);
      }
#endif
    } else {
      failedTransmissionCount++;

#ifdef DEBUG
      Log("WiFi unavailable on first run. Data saved to offline storage. (Failure ");
      Log(String(failedTransmissionCount).c_str());
      Log("/");
      Log(String(MAX_FAILED_TRANSMISSIONS).c_str());
      Logln(")");
#endif
    }

    gprsDone = true;
#endif /* WIFI_MODULE */

#else // WEB_MASTER_MODE
    Logln("[MASTER] Web Master Mode: Skipping offline EEPROM flush & initial forced sample.");
#ifdef WIFI_MODULE
    wifiAvailable = wifiConnect();
    esp_task_wdt_reset();
    if (wifiAvailable) {
      syncTimeNtp(5000);
    }
    gprsDone = true;
#endif
#endif
  }

#ifdef WIFI_MODULE
#if !defined(SENSOR_VIRTUAL) && !defined(WEB_MASTER_MODE)
  if (gprsDone && !experimentActive && !webMonitorActive) {
    wifiDisconnect();
    esp_task_wdt_reset();
  }
#endif
#endif

#ifdef GPRS_MODULE
  if (gprsDone) {
    // Final flush before sleep (power optimized)
    for (int i = 0; i < 5; i++) {
      mqtt.loop();
      delay(100);
    }
    sleepA7670();
    esp_task_wdt_reset();
  }
#endif

#if !defined(DISABLE_BLE) && !defined(WEB_MASTER_MODE)
  if (firstRun && !experimentActive && !webMonitorActive) {
    esp_task_wdt_reset();
    startBLE();
  }
#endif

#if defined(WEB_MASTER_MODE)
  webMonitorActive = false;
  experimentActive = false;
  oledShowMonitorStop();
  Logln("[MASTER] Web Dashboard Master Control ready. Device in IDLE (waiting for command)...");
#endif
}

/* =============================== Loop ============================== */

/**
 * @brief Main loop: manages BLE active window and deep sleep transitions.
 */
void loop() {
  esp_task_wdt_reset();

#if !defined(DISABLE_BLE)
  bleProcessLedBlinks();

  if (bleRestartAdvertisingRequested && bleActive && pServer != nullptr) {
    bleRestartAdvertisingRequested = false;
    Logln("Restarting BLE advertising...");
    pServer->getAdvertising()->start();
  }
#endif

#ifdef WIFI_MODULE
  if (manualSampleRequested) {
    manualSampleRequested = false;
    Logln("[MASTER] Executing queued manual sample...");
    takeMeasurement();
    if (WiFi.status() == WL_CONNECTED) {
      publishToAquaControl(rtcNow.TotalSeconds(), telemetryPayload.sensorReadings,
                           telemetryPayload.temperature, telemetryPayload.status);
    }
  }
#endif

  /* Real-time loop for Active Experiment or Web Monitor */
  static bool wasActive = false;

  if (experimentActive || webMonitorActive) {
    uint32_t activeInterval = experimentActive ? experimentIntervalMs : webMonitorIntervalMs;
    static unsigned long lastActiveSample = 0;

    if (!wasActive) {
      wasActive = true;
      lastActiveSample = 0; // Trigger measurement immediately upon activation
    }

    if (millis() - lastActiveSample >= activeInterval) {
      lastActiveSample = millis();

      // 1. Sample sensor, RTC, battery & broadcast ESP-NOW to ODrive
      takeMeasurement();

      // 2. Log to dedicated experiment CSV file on SD card
#ifdef SD_MODULE
      if (experimentActive) {
        logExperimentRow(rtcNow.TotalSeconds(), telemetryPayload.sensorReadings,
                         telemetryPayload.temperature, (uint16_t)telemetryPayload.battery);
      }
#endif

      // 3. Update OLED display
      oledUpdateMonitor(telemetryPayload, rtcNow, activeInterval / 1000);

      // 4. Stream to AquaControl dashboard via WiFi HTTPS
#ifdef WIFI_MODULE
      if (WiFi.status() != WL_CONNECTED) {
        wifiConnect();
      }
      if (WiFi.status() == WL_CONNECTED) {
        publishToAquaControl(rtcNow.TotalSeconds(), telemetryPayload.sensorReadings,
                             telemetryPayload.temperature, telemetryPayload.status);
      }
#endif

#if !defined(DISABLE_BLE)
      // 5. Stream to BLE characteristic if client connected
      if (bleDeviceConnected && pServer != nullptr) {
        int16_t primaryReading = telemetryPayload.sensorReadings[2];
        static char readingBuf[16];
        itoa(primaryReading, readingBuf, 10);
        if (pNotifyCharacteristic != nullptr) {
          pNotifyCharacteristic->setValue((const char*)readingBuf);
          pNotifyCharacteristic->notify();
        }
        if (pCharacteristic != nullptr) {
          pCharacteristic->setValue((const char*)readingBuf);
          pCharacteristic->notify();
        }
      }
#endif
    }

    delay(20);
    return; // Do not enter deep sleep while experiment or web monitor is active
  } else {
    wasActive = false;
  }

#if defined(WEB_MASTER_MODE)
  // In Web Master Mode, if no command is active, stay IDLE and poll for commands every 2 seconds
  static unsigned long lastIdlePoll = 0;
  if (millis() - lastIdlePoll >= 2000) {
    lastIdlePoll = millis();
#ifdef WIFI_MODULE
    if (WiFi.status() == WL_CONNECTED) {
      pollPendingCommands();
    }
#endif
  }
  delay(50);
  return;
#endif

#if defined(SENSOR_VIRTUAL)
  /* Continuous streaming loop for virtual dashboard test (sends every 5 seconds) */
  static unsigned long lastVirtualSend = 0;
  if (millis() - lastVirtualSend >= 5000) {
    lastVirtualSend = millis();

    takeMeasurement();

#ifdef WIFI_MODULE
    if (WiFi.status() != WL_CONNECTED) {
      Logln("WiFi reconnecting...");
      wifiConnect();
    }

    publishToAquaControl(rtcNow.TotalSeconds(), telemetryPayload.sensorReadings,
                         telemetryPayload.temperature, telemetryPayload.status);
#endif

    oledUpdateMonitor(telemetryPayload, rtcNow, 5);

#if !defined(DISABLE_BLE)
    if (bleDeviceConnected && pServer != nullptr) {
      int16_t primaryReading = telemetryPayload.sensorReadings[1];
      static char readingBuf[16];
      itoa(primaryReading, readingBuf, 10);
      if (pNotifyCharacteristic != nullptr) {
        pNotifyCharacteristic->setValue((const char*)readingBuf);
        pNotifyCharacteristic->notify();
      }
      if (pCharacteristic != nullptr) {
        pCharacteristic->setValue((const char*)readingBuf);
        pCharacteristic->notify();
      }
    }
#endif
  }

  delay(50);
  return;
#endif

#if !defined(DISABLE_BLE) && !defined(WEB_MASTER_MODE)
  if (bleActive) {
    static unsigned long lastBleReport = 0;
    static bool wasStreaming = false;
    if (!bleDeviceConnected && (millis() - lastBleReport >= 5000)) {
      lastBleReport = millis();
      unsigned long elapsed = millis() - bleStartTime;
      if (elapsed < BLE_ACTIVE_TIME) {
        unsigned long remainingSec = (BLE_ACTIVE_TIME - elapsed) / 1000;
        const char* mode = bleWasEverConnected ? "Reconnection window" : "Waiting for connection";
        Logf("[BLE] %s... %lu s remaining before sleep\n", mode, remainingSec);
      }
    }

    /* Continuous monitoring and storage loop ("MONITOR" / "SEND" command) */
    if (bleRealtimeSendActive) {
      static unsigned long lastMonitorSample = 0;
      if (millis() - lastMonitorSample >= bleMonitorIntervalMs) {
        lastMonitorSample = millis();

        // 1. Full measurement cycle: samples all sensors, RTC, battery, and saves to EEPROM / SD
        takeMeasurement();

        // 2. Constant OLED visualization on SH1106 display for MONITOR mode
        oledUpdateMonitor(telemetryPayload, rtcNow, bleMonitorIntervalMs / 1000);

        // 3. Extract primary measurement for BLE notification (if client connected)
        if (bleDeviceConnected && pServer != nullptr) {
          int16_t primaryReading =
              telemetryPayload.sensorReadings[2]; // DO in milli-mg/L (e.g. 8456 -> 8.456 mg/L)

          static char readingBuf[16];
          itoa(primaryReading, readingBuf, 10);

          if (pNotifyCharacteristic != nullptr) {
            pNotifyCharacteristic->setValue((const char*)readingBuf);
            pNotifyCharacteristic->notify();
          }
          if (pCharacteristic != nullptr) {
            pCharacteristic->setValue((const char*)readingBuf);
            pCharacteristic->notify();
          }
          delay(10); // yield briefly to let BLE BTC task flush GATT packet safely

          Logf("[MONITOR] Saved & Sent DO: %d.%03d mg/L (%s)\n", primaryReading / 1000,
               abs(primaryReading % 1000), readingBuf);
        } else {
          Logf("[MONITOR] Saved DO: %d.%03d mg/L (Client disconnected)\n",
               telemetryPayload.sensorReadings[2] / 1000,
               abs(telemetryPayload.sensorReadings[2] % 1000));
        }

        bleStartTime = millis(); // Refresh BLE active window while monitoring is active
      }
    }

    // If streaming was enabled but client stays disconnected past active window, disable
    // streaming mode
    if (bleStreamingActive && !bleDeviceConnected && (millis() - bleStartTime > BLE_ACTIVE_TIME)) {
      Logln("BLE streaming active but no client connected. Disabling streaming mode.");
      bleStreamingActive = false;
    }

    if (!bleStreamingActive && !bleRealtimeSendActive && !bleDeviceConnected &&
        (millis() - bleStartTime > BLE_ACTIVE_TIME)) {
      Logln(" BLE time expired, entering sleep...");
      stopBLE();
      prepareForDeepSleep(moduleCfg.measure_time);
    }
  } else {
    stopPrimarySensorRealtime();
    prepareForDeepSleep(moduleCfg.measure_time);
  }
#elif !defined(WEB_MASTER_MODE)
  stopPrimarySensorRealtime();
  prepareForDeepSleep(moduleCfg.measure_time);
#endif

  delay(50);
}
