/**
 * @file main.cpp
 * @brief Main application entry point for n-logger multi-sensor platform.
 *
 * Replaces the monolithic n_level_gprs.ino. Composes modular components
 * with compile-time feature toggles (SD_MODULE, GPRS_MODULE).
 * BLE and EEPROM are always included.
 *
 * Build identity: N_LOGGER_DEVICE_NAME (default "n-logger")
 * Sensor type: selected via descriptive build flags (e.g. -DSENSOR_MAXBOTIX_UART)
 */

#include "n_logger_config.h"
#ifdef GPRS_MODULE
#include "utilities.h"
#endif
#include "sensor_engine.h"
#include "ble_manager.h"
#include "modem_mqtt.h"
#include "measurement_storage.h"
#include "eeprom_i2c.h"
#include "error_sensor.h"
#include "rtc_utils.h"
#include "power_sleep.h"

/* -------------------------- Dependencies --------------------------- */

#include <Arduino.h>
#include <Wire.h>
#include <RtcDS3231.h>
#include <EEPROM.h>

#ifdef SD_MODULE
#include <SPI.h>
#endif

#include <esp_bt.h>
#include <esp_task_wdt.h>
#include <driver/gpio.h>
#include <string.h>

#ifdef GPRS_MODULE
#include <WiFi.h>
#endif

#include "config.h"

/* ------------------------- BLE Includes ---------------------------- */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/* ------------------------- Conditional Includes -------------------- */

#ifdef SD_MODULE
#include <SdFat.h>
#endif

#ifdef GPRS_MODULE
#include <TinyGsmClient.h>
#include <ESP_SSLClient.h>
#include <PubSubClient.h>
#endif

/* --------------------------- BLE Status ---------------------------- */

bool bleDeviceConnected = false;

uint64_t device_ID = ESP.getEfuseMac();

/* ------------------------------ Pins ------------------------------- */

#define SerialMon  Serial
#define SerialAT   Serial1

/* --------------------------- Modem/Network ------------------------- */

#ifdef GPRS_MODULE
#define TINY_GSM_DEBUG      SerialMon
#define TINY_GSM_USE_GPRS   true
#define TINY_GSM_USE_WIFI   false

#define GSM_PIN ""

#define TINY_GSM_RX_BUFFER           1024
#define MODEM_POWERON_PULSE_WIDTH_MS 100

#if TINY_GSM_USE_GPRS && not defined TINY_GSM_MODEM_HAS_GPRS
  #undef  TINY_GSM_USE_GPRS
  #undef  TINY_GSM_USE_WIFI
  #define TINY_GSM_USE_GPRS false
  #define TINY_GSM_USE_WIFI true
#endif

#if TINY_GSM_USE_WIFI && not defined TINY_GSM_MODEM_HAS_WIFI
  #undef  TINY_GSM_USE_GPRS
  #undef  TINY_GSM_USE_WIFI
  #define TINY_GSM_USE_GPRS true
  #define TINY_GSM_USE_WIFI false
#endif

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm        modem(debugger);
#else
  TinyGsm        modem(SerialAT);
#endif

TinyGsmClient gsmClient(modem);
PubSubClient  mqtt(gsmClient);
#endif /* GPRS_MODULE */

/* ---------------------------- SD ---------------------------------- */

#ifdef SD_MODULE
SdFat   SD;
SdFile  sdFile;
SPIClass sdSPI(HSPI);
#endif

/* ------------------------------ RTC -------------------------------- */

RtcDS3231<TwoWire> rtc(Wire);
RtcDateTime        rtcNow;
char               dateStr[20];

/* ----------------------------- Globals ----------------------------- */

RTC_DATA_ATTR int      bootCount       = 0;
RTC_DATA_ATTR uint16_t eepromWritePage = 0;
RTC_DATA_ATTR uint8_t  failedTransmissionCount = 0;
RTC_DATA_ATTR bool     forceModemReset = false;

bool measureDone      = false;
bool bluetoothDone    = false;
bool gprsDone         = false;
bool mqttAvailable    = false;
bool deviceConnected  = false;
bool readyForSleep    = false;
bool firstRun         = false;

uint32_t startMillis  = 0;

volatile uint16_t interruptCount = 0;
volatile bool     interruptFlag  = false;

byte      eepromPageBuf[EEPROM_PAGE_SIZE];
bool      needFlushToSD = false;

char      csvFilename[24];
bool      fileOpen = false;

String    payload = "";

/* ------------------------- Config Structure ------------------------ */

ModuleConfig moduleCfg;

/* ------------------------- Telemetry Packet ------------------------ */

TelemetryPayload telemetryPayload;

/* ------------------------- BLE Globals ----------------------------- */

bool bleActive = false;
unsigned long bleStartTime = 0;
bool commandMode = false;
unsigned long commandStart = 0;

BLEServer*        pServer        = nullptr;
BLECharacteristic* pCharacteristic = nullptr;

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

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Always initialize Serial so Log()/Logln()/Logf() work regardless of DEBUG
  Serial.begin(115200);


  // Initialize Task Watchdog Timer (30s) for hardware auto-recovery
  #if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 30000,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
      .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(&wdt_config);
  esp_task_wdt_add(NULL);
  #else
  esp_task_wdt_init(30, true);
  esp_task_wdt_add(NULL);
  #endif

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

#ifdef DEBUG
  Logln("Scanning I2C bus...");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Log("Found I2C device at 0x");
      if (address < 16) Log("0");
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

  // RTC init
  rtc.Begin();
  rtc.Enable32kHzPin(false);
  rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmBoth);
  rtcNow = rtc.GetDateTime();

  // SD bus init
#ifdef SD_MODULE
#if SD_EN_PIN != -1
  pinMode(SD_EN_PIN, OUTPUT);
  digitalWrite(SD_EN_PIN, LOW);   // active-low enable
  delay(10);                       // let SD module power stabilize
#endif
  sdSPI.begin(SD_PIN_SCLK, SD_PIN_MISO, SD_PIN_MOSI, SD_PIN_CS);
#endif

  // Make sure radios are off at startup
  boardRadiosOff();

#ifdef DEBUG
  Serial.println();
  Serial.printf("WELCOME TO %s ", N_LOGGER_DEVICE_NAME);
  Serial.printf("ChipID: %04X%08X\n",
                (uint16_t)(device_ID >> 32),
                (uint32_t)device_ID);
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
  Serial.println(F(" GMT\n"));

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
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {

    // Optional daily flush at 00:00
    if (rtcNow.Hour() == 0 && rtcNow.Minute() == 0) {
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

    // Measurement window
    if (moduleCfg.measure_time > 0 &&
        (rtcNow.Minute() % moduleCfg.measure_time == 0)) {
#ifdef DEBUG
      Logln("SAVE MEASURE LEVEL IN EEPROM");
#endif
      takeMeasurement();
      Logln("MT TIME...");
    }

    /* Bluetooth window — forced ON if streaming is active */
    if (moduleCfg.bluetooth_time > 0 &&
        (rtcNow.Minute() % moduleCfg.bluetooth_time == 0)) {
#ifdef DEBUG
      Logln("TRY TO CONNECT TO BLUETOOTH");
#endif
      digitalWrite(LED_PIN, LOW);
      startBLE();
    }

    /* Force BLE ON every cycle if streaming was previously active */
    if (bleStreamingActive) {
      startBLE();
    }

#ifdef GPRS_MODULE
    if (moduleCfg.GPRS_time > 0 &&
        (rtcNow.Minute() % moduleCfg.GPRS_time == 0)) {
#ifdef DEBUG
      Logln("TRY TO CONNECT TO GPRS AND SEND STORED DATA");
#endif
      mqttAvailable = initA7670();

      if (mqttAvailable) {
        failedTransmissionCount = 0;

        sendSDPendingToThingsboard();
        sendEEPROMToThingsboard();

        unsigned long t0 = millis();
        const unsigned long RPC_LISTEN_MS = 5000;
        while (millis() - t0 < RPC_LISTEN_MS) {
          mqtt.loop();
          delay(10);
        }
        processQueuedRpcCommands();
      } else {
        failedTransmissionCount++;

#ifdef DEBUG
        Log("MQTT unavailable. Data remains in offline storage. (Failure ");
        Log(String(failedTransmissionCount).c_str());
        Log("/");
        Log(String(MAX_FAILED_TRANSMISSIONS).c_str());
        Logln(")");
#endif

        // Force full modem reset after consecutive failures
        if (failedTransmissionCount >= MAX_FAILED_TRANSMISSIONS) {
#ifdef DEBUG
          Logln("Too many consecutive transmission failures!");
          Logln(" Forcing full modem reset on next cycle...");
#endif
          forceModemReset = true;
          failedTransmissionCount = 0;
        }
      }

      gprsDone = true;
    }
#endif /* GPRS_MODULE */

  } else {
    // First boot after power-on/reset
    firstRun = true;
#ifdef DEBUG
    Logln("UPLOADING TO SD CARD:");
    i2c_eeprom_read_buffer(EEPROM_ADDRESS, 0, eepromPageBuf, 32);
    for (int n = 0; n < 32; n++) {
      Log(String(eepromPageBuf[n]).c_str());
      Log(", ");
    }
    Logln("");
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

    delay(1000);

#ifdef DEBUG
    Logln("- Connecting to Internet: ");
#endif

    Logln("First run...");
    takeMeasurement();

#ifdef GPRS_MODULE
    mqttAvailable = initA7670();

    if (mqttAvailable) {
      failedTransmissionCount = 0;

      sendSDPendingToThingsboard();
      sendEEPROMToThingsboard();

      unsigned long t0 = millis();
      const unsigned long RPC_LISTEN_MS = 5000;
      while (millis() - t0 < RPC_LISTEN_MS) {
        mqtt.loop();
        delay(10);
      }
      processQueuedRpcCommands();
    } else {
      failedTransmissionCount++;

#ifdef DEBUG
      Log("MQTT unavailable on first run. Data saved to offline storage. (Failure ");
      Log(String(failedTransmissionCount).c_str());
      Log("/");
      Log(String(MAX_FAILED_TRANSMISSIONS).c_str());
      Logln(")");
#endif

      // Note: firstRun already true, will retry with full reset next cycle
    }

    gprsDone = true;
#endif /* GPRS_MODULE */
  }

#ifdef GPRS_MODULE
  if (gprsDone) {
    // Final flush before sleep (power optimized)
    for (int i = 0; i < 5; i++) {
      mqtt.loop();
      delay(100);
    }
    sleepA7670();
  }
#endif

  if (firstRun) {
    startBLE();
  }
}

/* =============================== Loop ============================== */

/**
 * @brief Main loop: manages BLE active window and deep sleep transitions.
 */
void loop() {
  esp_task_wdt_reset();
  bleProcessLedBlinks();

  if (bleRestartAdvertisingRequested && bleActive && pServer != nullptr) {
    bleRestartAdvertisingRequested = false;
    Logln("Restarting BLE advertising...");
    pServer->getAdvertising()->start();
  }

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

    /* Real-time sensor preview stream ("SEND" command) */
    if (bleRealtimeSendActive && bleDeviceConnected && pServer != nullptr) {
      static unsigned long lastRealtimeSample = 0;
      // Only read sensor and notify when client has subscribed to notifications
      if (isBleNotifyEnabled() && millis() - lastRealtimeSample >= 1000) {
        lastRealtimeSample = millis();
        int16_t dist = readPrimarySensorRealtime();
        if (dist > 0 && isBleNotifyEnabled()) {
          static char distBuf[8];
          itoa(dist, distBuf, 10);
          pNotifyCharacteristic->setValue((const char*)distBuf);
          pNotifyCharacteristic->notify();
          delay(10); // ponytail: yield briefly to let BLE BTC task flush GATT packet safely
#ifdef DEBUG
          Logf("[BLE][REALTIME] Sent: %s mm\n", distBuf);
#endif
        }
        bleStartTime = millis(); // Refresh BLE active window while real-time streaming
      }
      wasStreaming = true;
    } else {
      // Only kill streaming on actual disconnect, not on missing notification subscription
      if (!bleDeviceConnected) {
        bleRealtimeSendActive = false;
      }
      if (wasStreaming) {
        stopPrimarySensorRealtime();
        wasStreaming = false;
      }
    }

    // If streaming was enabled but client stays disconnected past active window, disable streaming mode
    if (bleStreamingActive && !bleDeviceConnected && (millis() - bleStartTime > BLE_ACTIVE_TIME)) {
      Logln("BLE streaming active but no client connected. Disabling streaming mode.");
      bleStreamingActive = false;
    }

    if (!bleStreamingActive && !bleRealtimeSendActive && !bleDeviceConnected && (millis() - bleStartTime > BLE_ACTIVE_TIME)) {
      Logln(" BLE time expired, entering sleep...");
      stopBLE();
      prepareForDeepSleep(moduleCfg.measure_time);
    }
  } else {
    stopPrimarySensorRealtime();
    prepareForDeepSleep(moduleCfg.measure_time);
  }

  delay(50);
}


