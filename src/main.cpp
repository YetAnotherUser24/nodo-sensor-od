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
#include "utilities.h"
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
#include <SPI.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <WiFi.h>
#include <esp_bt.h>
#include <driver/gpio.h>
#include <string.h>

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
unsigned long lastBleLedMillis = 0;
bool bleLedState = false;

/** Blink period in milliseconds when BLE is connected. */
const unsigned long BLE_CONNECTED_BLINK = 1000;
const unsigned long BLE_WAIT_BLINK = 1000;   // 1s blink cuando espera conexión

uint64_t device_ID = ESP.getEfuseMac();

/* ------------------------------ Pins ------------------------------- */

#define SerialMon  Serial
#define SerialAT   Serial1

/* --------------------------- On-board SD --------------------------- */

// SD pins now defined in n_logger_config.h as SD_PIN_MOSI, SD_PIN_MISO, etc.

/* --------------------------- Modem/Network ------------------------- */

#ifdef GPRS_MODULE
#define TINY_GSM_DEBUG      SerialMon
#define TINY_GSM_USE_GPRS   true
#define TINY_GSM_USE_WIFI   false

#define GSM_PIN ""              /**< SIM unlock PIN if any. */

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
RTC_DATA_ATTR uint16_t eepromWritePage = 0;   /**< Next data page after header. */
RTC_DATA_ATTR uint8_t  failedTransmissionCount = 0;  /**< Consecutive GPRS/MQTT failures. */
RTC_DATA_ATTR bool     forceModemReset = false;  /**< Force full modem reset on next cycle. */

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

/* ------------------------ BLE LED Pattern -------------------------- */

/**
 * @brief Controls LED indication according to BLE activity:
 *        - OFF when BLE inactive
 *        - ON solid when BLE active and connected
 *        - Blinking while BLE active and not connected.
 */
void handleBleLedPattern() {
  unsigned long now = millis();

  // BLE off → LED OFF
  if (!bleActive) {
    digitalWrite(LED_PIN, LOW);
    return;
  }

  // BLE active but not connected → BLINK
  if (!bleDeviceConnected) {
    if (now - lastBleLedMillis >= BLE_WAIT_BLINK) {
      bleLedState = !bleLedState;
      digitalWrite(LED_PIN, bleLedState);
      lastBleLedMillis = now;
    }
    return;
  }

  // BLE connected → LED ON solid
  digitalWrite(LED_PIN, HIGH);
}

/* ============================== Setup ============================== */

/**
 * @brief Initializes pins, RTC, SD, configuration and communication modules.
 */
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin();

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
  sdSPI.begin(SD_PIN_SCLK, SD_PIN_MISO, SD_PIN_MOSI, SD_PIN_CS);
#endif

  // Make sure radios are off at startup
  boardRadiosOff();

#ifdef DEBUG
  Serial.begin(115200);
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
        Serial.println(F("- EEPROM flushed."));
#endif
      } else {
#ifdef DEBUG
        Serial.println(F("Failed to flush EEPROM. SD card missing? Continuing anyway."));
#endif
        error(3, LED_PIN);
      }
    }

    // Measurement window
    if (moduleCfg.measure_time > 0 &&
        (rtcNow.Minute() % moduleCfg.measure_time == 0)) {
#ifdef DEBUG
      Serial.println("SAVE MEASURE LEVEL IN EEPROM");
#endif
      takeMeasurement();
      Serial.println("MT TIME...");
    }

    // Bluetooth window
    if (moduleCfg.bluetooth_time > 0 &&
        (rtcNow.Minute() % moduleCfg.bluetooth_time == 0)) {
#ifdef DEBUG
      Serial.println("TRY TO CONNECT TO BLUETOOTH");
#endif
      startBLE();
    }

    // GPRS window
#ifdef GPRS_MODULE
    if (moduleCfg.GPRS_time > 0 &&
        (rtcNow.Minute() % moduleCfg.GPRS_time == 0)) {
#ifdef DEBUG
      Serial.println("TRY TO CONNECT TO GPRS AND SEND STORED DATA");
#endif
      mqttAvailable = initA7670();

      if (mqttAvailable) {
        failedTransmissionCount = 0;  // Reset on successful connection

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
        Serial.print("MQTT unavailable. Data remains in offline storage. (Failure ");
        Serial.print(failedTransmissionCount);
        Serial.print("/");
        Serial.print(MAX_FAILED_TRANSMISSIONS);
        Serial.println(")");
#endif

        // Force full modem reset after consecutive failures
        if (failedTransmissionCount >= MAX_FAILED_TRANSMISSIONS) {
#ifdef DEBUG
          Serial.println("⚠️ Too many consecutive transmission failures!");
          Serial.println("🔄 Forcing full modem reset on next cycle...");
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
    Serial.println("UPLOADING TO SD CARD:");
    i2c_eeprom_read_buffer(EEPROM_ADDRESS, 0, eepromPageBuf, 32);
    for (int n = 0; n < 32; n++) {
      Serial.print(eepromPageBuf[n]);
      Serial.print(F(", "));
    }
    Serial.println();
#endif

    if (dumpEEPROM()) {
      resetEEPromHeader(EEPROM_ADDRESS);
#ifdef DEBUG
      Serial.println(F("- EEPROM flushed."));
#endif
    } else {
#ifdef DEBUG
      Serial.println(F("Failed to flush EEPROM. SD card missing? Continuing anyway."));
#endif
      error(3, LED_PIN);
    }

    delay(1000);

#ifdef DEBUG
    Serial.println("- Connecting to Internet: ");
#endif

    Serial.println("First run...");
    takeMeasurement();

#ifdef GPRS_MODULE
    mqttAvailable = initA7670();

    if (mqttAvailable) {
      failedTransmissionCount = 0;  // Reset on successful connection

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
      Serial.print("MQTT unavailable on first run. Data saved to offline storage. (Failure ");
      Serial.print(failedTransmissionCount);
      Serial.print("/");
      Serial.print(MAX_FAILED_TRANSMISSIONS);
      Serial.println(")");
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

  handleBleLedPattern();

  if (bleActive) {
    if (millis() - bleStartTime > BLE_ACTIVE_TIME) {
      Serial.println("⏱️ BLE time expired, entering sleep...");
      stopBLE();
      prepareForDeepSleep(moduleCfg.measure_time);
    }
  } else {
    digitalWrite(LED_PIN, LOW);
    prepareForDeepSleep(moduleCfg.measure_time);
  }

  delay(50);
}

/* ============================ Commands ============================= */

/**
 * @brief Extracts JSON payload from an HTTP-like response string.
 *
 * @param response Input response string including headers.
 * @return JSON substring starting after the header separator.
 */
String extractJSON(String response) {
  int jsonStart = response.indexOf("\r\n\r\n");
  return (jsonStart != -1) ? response.substring(jsonStart + 4) : "";
}