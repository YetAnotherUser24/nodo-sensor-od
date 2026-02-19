/**
 * @file n_level_gprs.ino
 * @brief Main application for LilyGO A7670SA: measurement, storage, BLE and GPRS/MQTT.
 */

#include "utilities.h"
#include "ble_manager.h"
#include "modem_mqtt.h"
#include "measurement_storage.h"
#include "eeprom_i2c.h"
#include "error_sensor.h"
#include "rtc_utils.h"
#include "power_sleep.h"

/* -------------------------- Dependencies --------------------------- */
#include <TinyGsmClient.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <RtcDS3231.h>
#include <SPI.h>
#include <SdFat.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <ESP_SSLClient.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <esp32-hal-adc.h>
#include <WiFi.h>
#include <esp_bt.h>
#include <driver/gpio.h>
#include <string.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/* --------------------------- BLE Status ---------------------------- */

bool bleDeviceConnected = false;
unsigned long lastBleLedMillis = 0;
bool bleLedState = false;

/** Blink period in milliseconds when BLE is connected. */
const unsigned long BLE_CONNECTED_BLINK = 1000;
const unsigned long BLE_WAIT_BLINK = 1000;   // 1s blink cuando espera conexión

/* ----------------------- Build-Time Configuration ------------------ */

/** Feature switches for build-time enabling of modules. */
#define SD_MODULE   true
#define BT_MODULE   true
#define GPRS_MODULE true

/** Sampling and storage configuration. */
#define NUM_READINGS             5     /**< Number of sensor readings per measurement. */
#define EEPROM_FLUSH_AFTER       288   /**< Number of records before flushing EEPROM to SD. */

#define EEPROM_SENT_FLAG_OFFSET (EEPROM_PAGE_SIZE - 1)
#define FLAG_NOT_SENT 0x00
#define FLAG_SENT     0x01

uint64_t device_ID = ESP.getEfuseMac();
#define CODE_VERSION 0X01

/** Enable serial debug logs. */
#ifndef DEBUG
#define DEBUG
#endif
#ifndef BOARD_BAT_ADC_PIN
#error "No support this board"
#endif

/* ------------------------------ Pins ------------------------------- */

HardwareSerial SensorSerial(2);
const uint8_t SENSOR_RX_PIN = 23;
const uint8_t SENSOR_TX_PIN = 19;

/** Water level sensor type: 0 = 5 m, 1 = 10 m. */
#define WATER_LEVEL_SENSOR_TYPE 1

#define SerialMon  Serial
#define SerialAT   Serial1

#define LED_PIN        32
#define SENSOR_PWR_PIN 18
#define VBAT_PIN       35

#define WAKEUP_PIN           GPIO_NUM_39
#define WAKEUP_PIN_BITMASK   (1ULL << WAKEUP_PIN)

#define EEPROM_ADDRESS     0x50
#define EEPROM_PAGE_SIZE   32
#define EEPROM_HEADER_SIZE 8
#define EEPROM_MAX_PAGES   2048

/* --------------------------- On-board SD --------------------------- */

static const int SD_MOSI = 15;
static const int SD_MISO = 2;
static const int SD_SCLK = 14;
static const int SD_CS   = 13;

/* --------------------------- Modem/Network ------------------------- */

#define TINY_GSM_DEBUG      SerialMon
#define TINY_GSM_USE_GPRS   true
#define TINY_GSM_USE_WIFI   false

#define GSM_PIN ""              /**< SIM unlock PIN if any. */

const uint32_t UNIX_OFFSET_2000     = 946684800UL;
const int32_t  LOCAL_TZ_OFFSET_SEC = 5 * 3600;

/* APN configuration */
#define APN = "claro.pe";
#define APN_USER ""
#define APN_PASS ""

/* MQTT (ThingsBoard) configuration */
const char* MQTT_HOST   = "viewer.natek.lat";
#define MQTT_PORT 1883
const char* MQTT_TOKEN  = "os9u2cyuOGdOW3WfxPsg";
const char* MQTT_TOPIC  = "v1/devices/me/telemetry";

/* ThingsBoard RPC topics */
const char* TB_RPC_SUB_TOPIC = "v1/devices/me/rpc/request/+";

/* ------------------------- Bluetooth Config ------------------------ */

#define BLE_DEVICE_NAME     "LILYGO_A7670_BLE"
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

/** BLE active window in milliseconds. */
#define BLE_ACTIVE_TIME 30000

bool bleActive = false;
unsigned long bleStartTime = 0;
bool commandMode = false;
unsigned long commandStart = 0;

BLEServer*        pServer        = nullptr;
BLECharacteristic* pCharacteristic = nullptr;

/* ----------------------------- Helpers ----------------------------- */

#define countof(a) (sizeof(a) / sizeof(a[0]))

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

/* --------------------------- TinyGSM Setup ------------------------- */

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm        modem(debugger);
#else
  TinyGsm        modem(SerialAT);
#endif

TinyGsmClient gsmClient(modem);
PubSubClient  mqtt(gsmClient);

/* ---------------------------- SD / EEPROM -------------------------- */

SdFat   SD;
SdFile  sdFile;
SPIClass sdSPI(HSPI);

/* ------------------------------ RTC -------------------------------- */

RtcDS3231<TwoWire> rtc(Wire);
RtcDateTime        rtcNow;
char               dateStr[20];

/* ----------------------------- Globals ----------------------------- */

RTC_DATA_ATTR int      bootCount       = 0;
RTC_DATA_ATTR uint16_t eepromWritePage = 0;   /**< Next data page after header. */
RTC_DATA_ATTR uint8_t  failedTransmissionCount = 0;  /**< Consecutive GPRS/MQTT failures. */

const uint8_t MAX_FAILED_TRANSMISSIONS = 3;  /**< Force modem reset after this many failures. */

bool measureDone      = false;
bool bluetoothDone    = false;
bool gprsDone         = false;
bool mqttAvailable    = false;
bool deviceConnected  = false;
bool sendData         = false;
bool readyForSleep    = false;
bool firstRun         = false;

uint32_t startMillis  = 0;
int16_t  levelReadings[NUM_READINGS];
uint8_t  nread        = 0;

volatile uint16_t interruptCount = 0;
volatile bool     interruptFlag  = false;

byte      eepromPageBuf[EEPROM_PAGE_SIZE];
bool      needFlushToSD = false;

char      csvFilename[24];
bool      fileOpen = false;

String    payload = "";
uint32_t  battery_voltage = 0;

/* -------------------------- Status Bits ---------------------------- */

#define STATUS_INTERNET_BIT           0
#define STATUS_FULL_MEMORY_BIT        1
#define STATUS_CORRUPTED_MEMORY_BIT   2
#define STATUS_CODE_ERROR_BIT         3
#define STATUS_WATER_LEVEL_SENSOR_BIT 16
#define STATUS_SD_CARD_BIT            17
#define STATUS_TEMPERATURE_BIT        18
#define STATUS_GRPS_BIT               19

/* ------------------------- Config Structure ------------------------ */

/**
 * @brief Runtime configuration for module scheduling.
 */
struct ModuleConfig {
  int measure_time;
  int bluetooth_time;
  int GPRS_time;
};
ModuleConfig moduleCfg;

/* ------------------------- Telemetry Packet ------------------------ */

/**
 * @brief Main telemetry payload structure sent to the server.
 */
struct TelemetryPayload {
  int16_t  freeStorage;
  int16_t  temperature;
  int16_t  humidity;
  int16_t  waterLevelSensor[NUM_READINGS];
  uint32_t battery;
  uint32_t status;
} telemetryPayload;

/* --------------------------- Prototypes ---------------------------- */

String   extractJSON(String response);
int      WaterLevelRead();
uint32_t changeBit(uint32_t value, uint8_t bitIndex, bool setBit);

/* -------------------------- File Name Utils ------------------------ */

/**
 * @brief Converts a pending SD filename (e.g. "/YYYYMMDD_P.CSV") into
 *        its final name ("/YYYYMMDD.CSV").
 *
 * @param pendingName Input pending filename.
 * @param out         Output buffer for final name.
 * @param outSize     Size of the output buffer.
 */
void toFinalName(const char* pendingName, char* out, size_t outSize) {
  strncpy(out, pendingName, outSize);
  out[outSize - 1] = '\0';

  size_t len = strlen(out);
  if (len < 8) {
    return;
  }

  char* underscore = strrchr(out, '_');
  if (!underscore) {
    return;
  }
  if (strncmp(underscore, "_P.CSV", 6) == 0) {
    memmove(underscore, underscore + 2, strlen(underscore + 2) + 1);
  }
}

/* ------------------------ BLE LED Pattern -------------------------- */

/**
 * @brief Controls LED indication according to BLE activity:
 *        - OFF when BLE inactive
 *        - ON solid when BLE active and connected
 *        - Blinking while BLE active and not connected.
 */
void handleBleLedPattern() {
  unsigned long now = millis();

  // --- BLE apagado → LED OFF ---
  if (!bleActive) {
    digitalWrite(LED_PIN, LOW);
    return;
  }

  // --- BLE activo pero SIN conexión → PARPADEAR ---
  if (!bleDeviceConnected) {
    if (now - lastBleLedMillis >= BLE_WAIT_BLINK) {
      bleLedState = !bleLedState;
      digitalWrite(LED_PIN, bleLedState);
      lastBleLedMillis = now;
    }
    return;
  }

  // --- BLE conectado → LED ON FIJO ---
  digitalWrite(LED_PIN, HIGH);
}


/* ============================== Setup ============================== */

/**
 * @brief Initializes pins, RTC, SD, configuration and communication modules.
 */
void setup() {
  pinMode(SENSOR_PWR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(SENSOR_PWR_PIN, LOW);

  Wire.begin();

  // Example of default configuration persistence (kept commented)
  /*
  moduleCfg.measure_time   = 1;
  moduleCfg.bluetooth_time = 2;
  moduleCfg.GPRS_time      = 3;

  EEPROM.begin(sizeof(ModuleConfig));
  EEPROM.put(0, moduleCfg);
  EEPROM.commit();
  EEPROM.end();
  */

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
  sdSPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);

  // Make sure radios are off at startup
  boardRadiosOff();

#ifdef DEBUG
  Serial.begin(115200);
  Serial.println();
  Serial.print(F("WELCOME TO N-LEVEL "));
  Serial.printf("ChipID: %04X%08X\n",
                (uint16_t)(device_ID >> 32),
                (uint32_t)device_ID);
  if ((SD_MODULE) && !(BT_MODULE) && !(GPRS_MODULE)) Serial.print("SD");
  else if ((SD_MODULE) && (BT_MODULE) && !(GPRS_MODULE)) Serial.print("BT");
  else if ((SD_MODULE) && (BT_MODULE) && (GPRS_MODULE)) Serial.print("GPRS");
  Serial.println(" v0.1.1");

  Serial.print(F("Current time is "));
  formatDateTime(rtcNow);
  Serial.print(dateStr);
  Serial.println(F(" GMT\n"));

  Serial.println("CURRENT CONFIGURATION:");
  Serial.print("(Measure time: ");
  Serial.print(moduleCfg.measure_time);
  if ((BT_MODULE) && !(GPRS_MODULE)) {
    Serial.print(" | BT time: ");
    Serial.print(moduleCfg.bluetooth_time);
  } else if ((BT_MODULE) && (GPRS_MODULE)) {
    Serial.print(" | BT time: ");
    Serial.print(moduleCfg.bluetooth_time);
    Serial.print(" | GPRS time: ");
    Serial.print(moduleCfg.GPRS_time);
  }
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
    if (BT_MODULE &&
        (moduleCfg.bluetooth_time > 0) &&
        (rtcNow.Minute() % moduleCfg.bluetooth_time == 0)) {
#ifdef DEBUG
      Serial.println("TRY TO CONNECT TO BLUETOOTH");
#endif
      startBLE();
    }

    // GPRS window
    if (GPRS_MODULE &&
        moduleCfg.GPRS_time > 0 &&
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
          firstRun = true;
          failedTransmissionCount = 0;
        }
      }

      gprsDone = true;
    }

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
  }

  if (gprsDone) {
    // Final flush before sleep (power optimized)
    for (int i = 0; i < 5; i++) {
      mqtt.loop();
      delay(100);
    }
    sleepA7670();
  }
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

    if (sendData) {
      if (SensorSerial.available()) {
        int distance = SensorSerial.parseInt();
        pCharacteristic->setValue(std::string(String(distance).c_str()));
        pCharacteristic->notify();
      }
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

/* ============================ Utilities ============================ */

/**
 * @brief Sets or clears a specific bit in a 32-bit value.
 *
 * @param value    Current value.
 * @param bitIndex Bit index [0..31] to modify.
 * @param setBit   True to set bit, false to clear.
 * @return Updated value with the bit modified.
 */
uint32_t changeBit(uint32_t value, uint8_t bitIndex, bool setBit) {
  if (bitIndex > 31) {
    return value;
  }
  if (setBit) {
    value |= (1UL << bitIndex);
  } else {
    value &= ~(1UL << bitIndex);
  }
  return value;
}

/* ------------------------- Sensor (MaxBotix) ----------------------- */

/**
 * @brief Reads distance from MaxBotix water level sensor over UART.
 *
 * @return Distance value parsed from sensor output, or -2 on timeout/error.
 */
int WaterLevelRead() {
  int result = -2;
  char inData[6] = {0};
  int index = 0;
  unsigned long t0 = millis();
  bool foundR = false;

  // Wait for 'R' start character
  while (!foundR && (millis() - t0) < 500) {
    if (SensorSerial.available()) {
      if (SensorSerial.read() == 'R') {
        foundR = true;
        break;
      }
    }
  }

  if (!foundR) return -2;

  // Read up to 5 digits after 'R'
  t0 = millis();
  while (index < 5 && (millis() - t0) < 200) {
    if (SensorSerial.available()) {
      char c = SensorSerial.read();
      if (c >= '0' && c <= '9') {
        inData[index++] = c;
      } else if (c == '\r' || c == '\n') {
        break;  // End of reading
      }
    }
  }

  if (index > 0) {
    inData[index] = '\0';
    result = atoi(inData);
    
    // Validate range (MaxBotix typically 300-5000mm)
    if (result < 0 || result > 9999) {
      result = -2;
    }
  }

  return result;
}
