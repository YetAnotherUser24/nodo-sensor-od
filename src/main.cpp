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
#include "measurement_storage.h"
#include "n_logger_config.h"
#include "oled_display.h"
#include "power_sleep.h"
#include "rtc_utils.h"
#include "sensor_engine.h"
#include "wifi_http.h"
#include "aqua_link.h"
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



#ifdef WIFI_MODULE
#include <WiFi.h>
#include <ArduinoOTA.h>
#endif

/* --------------------------- BLE Status ---------------------------- */

bool bleDeviceConnected = false;

uint64_t device_ID = ESP.getEfuseMac();

/* ------------------------------ Pins ------------------------------- */

#define SerialMon Serial
#define SerialAT Serial1


/* ---------------------------- SD ---------------------------------- */

#ifdef SD_MODULE
SdFat SD;
SdFile sdFile;
#ifndef SD_SPI_HOST
#define SD_SPI_HOST HSPI
#endif
SPIClass sdSPI(SD_SPI_HOST);
#endif

/* ------------------------------ RTC -------------------------------- */

RtcDS3231<TwoWire> rtc(Wire);
RtcDateTime rtcNow;
char dateStr[20];

/* ----------------------------- Globals ----------------------------- */

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR uint8_t failedTransmissionCount = 0;

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

char csvFilename[24];
bool fileOpen = false;

String payload = "";

bool isOtaUpdating = false;

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
uint32_t experimentIntervalMs = 5000;
char activeExperimentId[40] = {0};
char activeExperimentName[48] = {0};
char activeExperimentFilename[16] = "EXP_01.CSV";

bool webMonitorActive = false;
uint32_t webMonitorIntervalMs = 5000;
bool manualSampleRequested = false;

/** V4: indica que existe al menos una muestra válida en telemetryPayload (habilita el heartbeat IDLE). */
bool hasValidTelemetry = false;

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
    // Reaching this without an id means the run cannot be identified afterwards, and the
    // samples would be grouped under a name nothing else knows. Say so, rather than letting a
    // silent fallback look like a working experiment in the archive.
    const char* expId = pDoc["experiment_id"] | "";
    if (expId[0] == '\0') {
      Logln("[CMD] start_experiment without experiment_id: samples will not be attributable.");
    }
    const char* expName = pDoc["name"] | "Experimento";
    const char* fname = pDoc["csv_filename"] | "EXP_01.CSV";
    int intervalSec = pDoc["interval_sec"] | 5;
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
    stopPrimarySensorRealtime();

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
    stopPrimarySensorRealtime();

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

  } else if (strcmp(action, "set_state") == 0) {
    // Orquestador V4: IDLE | ACTIVE_EXPERIMENT | MANUAL_OVERRIDE
    const char* sysState = pDoc["state"] | "IDLE";
    int intervalSec = pDoc["interval_sec"] | 5;
    if (intervalSec < 1) intervalSec = 1;
    if (intervalSec > 60) intervalSec = 60;

    if (strcmp(sysState, "ACTIVE_EXPERIMENT") == 0) {
      const char* expId = pDoc["experiment_id"] | "";
      if (expId != nullptr && strlen(expId) > 0) {
        strncpy(activeExperimentId, expId, sizeof(activeExperimentId) - 1);
        activeExperimentId[sizeof(activeExperimentId) - 1] = '\0';
      }
      experimentIntervalMs = intervalSec * 1000UL;
      webMonitorIntervalMs = intervalSec * 1000UL;
      experimentActive = true;
      webMonitorActive = true;
      oledShowMonitorStart(intervalSec);
      stateDoc["monitor_active"] = true;
      stateDoc["monitor_interval_sec"] = intervalSec;
    } else {
      // IDLE o MANUAL_OVERRIDE: cesa el muestreo automático; el operador toma el control directo.
      experimentActive = false;
      webMonitorActive = false;
#ifdef SD_MODULE
      stopExperimentFile();
#endif
      oledShowMonitorStop();
      stopPrimarySensorRealtime();
      stateDoc["monitor_active"] = false;
      stateDoc["active_experiment"] = nullptr;
    }
    stateDoc["system_state"] = sysState;

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
  } else {
    // An unrecognised action must not be acknowledged as applied. Without this the chain fell
    // through with `success` still true, so the gateway recorded a green "acknowledged" for an
    // action this firmware never performed. That is worse than an error: a mistyped action
    // looked successful, and an ack that can mean nothing makes the whole channel untrustworthy
    // -- including for the commands it does perform.
    Logf("[CMD] Unsupported action: %s\n", action);
    success = false;
    stateDoc["error"] = "unsupported_action";
    stateDoc["action"] = action;
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
      Logf("Found I2C device at 0x%02X\n", address);
    }
  }
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

    // Measurement window
    if (moduleCfg.measure_time > 0 && (rtcNow.Minute() % moduleCfg.measure_time == 0)) {
#ifdef DEBUG
      Logln("TAKING SCHEDULED MEASUREMENT");
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

        // ponytail: No RPC replacement needed. BLE commands cover T-x-y-z and D-YYYY-MM-DD.
      } else {
        failedTransmissionCount++;

#ifdef DEBUG
        Logf("WiFi unavailable. Data remains in offline storage. (Failure %u/%u)\n",
             failedTransmissionCount, MAX_FAILED_TRANSMISSIONS);
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
    esp_task_wdt_reset();
    delay(1000);

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
    } else {
      failedTransmissionCount++;

#ifdef DEBUG
      Logf("WiFi unavailable on first run. Data saved to offline storage. (Failure %u/%u)\n",
           failedTransmissionCount, MAX_FAILED_TRANSMISSIONS);
#endif
    }

    gprsDone = true;
#endif /* WIFI_MODULE */

#else // WEB_MASTER_MODE
    Logln("[MASTER] Web Master Mode: Skipping initial forced sample.");
#ifdef WIFI_MODULE
    wifiAvailable = wifiConnect(false); // Async non-blocking connection
    esp_task_wdt_reset();
    
    ArduinoOTA.onStart([]() {
      isOtaUpdating = true;
      Logln("[OTA] Update started.");
      stopPrimarySensorRealtime();
      experimentActive = false;
      webMonitorActive = false;
    });
    ArduinoOTA.onEnd([]() {
      isOtaUpdating = false;
      Logln("[OTA] Update finished.");
    });
    ArduinoOTA.onError([](ota_error_t error) {
      isOtaUpdating = false;
      Logf("[OTA] Error: %d\n", error);
    });
    ArduinoOTA.setHostname(N_LOGGER_DEVICE_NAME);
    ArduinoOTA.begin();

    gprsDone = true;
#endif
#endif
  }

#ifdef WIFI_MODULE
#if !defined(WEB_MASTER_MODE)
  if (gprsDone && !experimentActive && !webMonitorActive) {
    wifiDisconnect();
    esp_task_wdt_reset();
  }
#endif
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
  stopPrimarySensorRealtime();
  ledSignalIdle();
  Logln("[MASTER] Web Dashboard Master Control ready. Device in IDLE (waiting for command)...");
#endif
}

/* =============================== Loop ============================== */

/**
 * @brief Main loop: manages BLE active window and deep sleep transitions.
 */
void loop() {
  esp_task_wdt_reset();

#if defined(WIFI_MODULE) && AQUA_TRANSPORT_MQTT
  // Serviced before anything can return, so the MQTT keepalive and the command subscription
  // stay alive on every path through the loop — including the OTA and experiment early-outs.
  aquaLinkLoop();
#endif

#if defined(WEB_MASTER_MODE) && defined(WIFI_MODULE)
  if (isOtaUpdating) {
    ArduinoOTA.handle();
    return;
  }
  wifiMaintain();
  ArduinoOTA.handle();
#endif

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

      // 4. Acumular en RAM y enviar en lote (Bulk) cada 5 s — V4 §2.2
#ifdef WIFI_MODULE
      if (WiFi.status() != WL_CONNECTED) {
        wifiConnect();
      }
      queueTelemetrySample(rtcNow.TotalSeconds(), telemetryPayload.sensorReadings,
                           telemetryPayload.temperature, telemetryPayload.status);
      hasValidTelemetry = true;
      {
        static unsigned long lastBulkFlush = 0;
        if (millis() - lastBulkFlush >= 5000 || telemetryQueueCount() >= 12) {
          lastBulkFlush = millis();
          if (WiFi.status() == WL_CONNECTED) {
            flushTelemetryBulk();
          }
        }
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
    if (wasActive) {
      wasActive = false;
      stopPrimarySensorRealtime();
      ledSignalIdle();
    }
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

#ifdef WIFI_MODULE
  // V4 §2.1: heartbeat IDLE cada 20 s reenviando la última lectura válida (sin re-muestrear ni
  // emitir ESP-NOW, para no reactuar el lazo del ODrive mientras el sistema está en reposo).
  static unsigned long lastIdleHeartbeat = 0;
  if (hasValidTelemetry && millis() - lastIdleHeartbeat >= 20000) {
    lastIdleHeartbeat = millis();
    if (WiFi.status() != WL_CONNECTED) {
      wifiConnect(false);
    }
    if (WiFi.status() == WL_CONNECTED) {
      publishToAquaControl(rtcNow.TotalSeconds(), telemetryPayload.sensorReadings,
                           telemetryPayload.temperature, telemetryPayload.status);
    }
  }
#endif

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
