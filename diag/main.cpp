/**
 * @file diag/main.cpp
 * @brief [DEPRECATED] Interactive serial diagnostics console from legacy n-logger platform.
 *
 * NOTE: This diagnostic utility was used in the legacy multi-sensor platform
 * and is deprecated in the specialized OD-Logger project.
 */

/* ============================== Includes ============================== */

#include "board.h"
#include "n_logger_config.h"
#include "sensor_engine.h"
#include "power_sleep.h"
#include "rtc_utils.h"
#include "eeprom_i2c.h"
#include "measurement_storage.h"
#include "ble_manager.h"
#include "modem_mqtt.h"
#include "log.h"
#include "error_sensor.h"
#include "config.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <RtcDS3231.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <esp_sleep.h>
#include <esp_bt.h>
#include <HardwareSerial.h>

#ifdef SD_MODULE
#include <SPI.h>
#include <SdFat.h>
#endif

#ifdef GPRS_MODULE
#include <WiFi.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ESP_SSLClient.h>
#endif

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

/* ==================== Globals (satisfy production externs) ==================== */

bool bleDeviceConnected = false;
uint64_t device_ID = ESP.getEfuseMac();

#ifdef GPRS_MODULE
#define SerialMon Serial
#define SerialAT Serial1

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

#ifdef SD_MODULE
SdFat SD;
SdFile sdFile;
SPIClass sdSPI(HSPI);
#endif

RtcDS3231<TwoWire> rtc(Wire);
RtcDateTime rtcNow;
char dateStr[20];

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR uint16_t eepromWritePage = 0;
RTC_DATA_ATTR uint8_t failedTransmissionCount = 0;
RTC_DATA_ATTR bool forceModemReset = false;

bool measureDone = false;
bool bluetoothDone = false;
bool gprsDone = false;
bool mqttAvailable = false;
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

ModuleConfig moduleCfg;
TelemetryPayload telemetryPayload;

bool bleActive = false;
unsigned long bleStartTime = 0;
bool commandMode = false;
unsigned long commandStart = 0;

BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;

/* ==================== Duplicated static sensor helpers ==================== */
/* These are file-static in sensor_engine.cpp; duplicated here for direct control. */

#ifdef SENSOR_MAXBOTIX_UART
static HardwareSerial diagMaxbotixSerial(2);
static bool diagMaxbotixOn = false;

static int16_t diagMaxbotixReadSingle() {
  char inData[6] = {0};
  int index = 0;
  unsigned long t0 = millis();
  bool foundR = false;

  while (diagMaxbotixSerial.available()) diagMaxbotixSerial.read();

  while (!foundR && (millis() - t0) < 500) {
    if (diagMaxbotixSerial.available()) {
      if (diagMaxbotixSerial.read() == 'R') { foundR = true; break; }
    }
    yield();
  }
  if (!foundR) return -2;

  t0 = millis();
  while (index < 5 && (millis() - t0) < 200) {
    if (diagMaxbotixSerial.available()) {
      char c = diagMaxbotixSerial.read();
      if (c >= '0' && c <= '9') inData[index++] = c;
      else if (c == '\r' || c == '\n') break;
    }
    yield();
  }
  if (index > 0) {
    inData[index] = '\0';
    int16_t r = atoi(inData);
    return (r >= 0 && r <= 9999) ? r : -2;
  }
  return -2;
}
#endif

#if defined(SENSOR_WATER_TEMP) || defined(SENSOR_MLX90614)
static bool diagWaterTempOn = false;

static int16_t diagMlx90614ReadTemp(uint8_t reg) {
  Wire.beginTransmission(WATER_TEMP_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -999;
  if (Wire.requestFrom((uint8_t)WATER_TEMP_ADDR, (uint8_t)3) < 3) return -999;
  uint8_t lsb = Wire.read();
  uint8_t msb = Wire.read();
  Wire.read(); // PEC
  uint16_t raw = ((uint16_t)msb << 8) | lsb;
  if (raw == 0 || (msb & 0x80)) return -999;
  return (int16_t)(((int32_t)raw * 2 - 27315) / 10);
}
#endif

#ifdef SENSOR_RADAR
#include "modbus_rtu.h"
static HardwareSerial diagRadarSerial(2);
static bool diagRadarOn = false;

static int16_t diagRadarReadSingle() {
  modbus_rtu::ModbusRtu mb(diagRadarSerial);
  modbus_rtu::ModbusRtu::Config cfg;
  cfg.responseTimeoutMs = 500;
  cfg.interFrameGapMs = 4;
  mb.configure(cfg);

  while (diagRadarSerial.available()) diagRadarSerial.read();

  auto res = mb.readRegisters(RADAR_SLAVE_ID, 0x03, RADAR_MODBUS_REGISTER, 1);
  if (!res.success || res.isException || res.count < 1) return -2;
  return static_cast<int16_t>(res.values[0]);
}
#endif

#ifdef SENSOR_SOIL_3TIP
#include "modbus_rtu.h"
static HardwareSerial diagSoilSerial(2);
static bool diagSoilOn = false;

static int16_t diagSoil3ReadBoth(int16_t& moisture, int16_t& temperature) {
  modbus_rtu::ModbusRtu mb(diagSoilSerial);
  modbus_rtu::ModbusRtu::Config cfg;
  cfg.directionPin = SOIL3_DE_RE_PIN;
  cfg.manualDirection = true;
  cfg.txActiveHigh = true;
  cfg.responseTimeoutMs = 500;
  cfg.txEnableGuardUs = 250;
  cfg.txDisableGuardUs = 250;
  cfg.interFrameGapMs = 4;
  cfg.rxTurnaroundUs = 0;
  mb.configure(cfg);
  mb.begin(SOIL3_BAUD, SOIL3_RX_PIN, SOIL3_TX_PIN);

  while (diagSoilSerial.available()) diagSoilSerial.read();

  auto res = mb.readRegisters(SOIL3_SLAVE_ID, 0x03, SOIL3_MODBUS_START_ADDR, SOIL3_MODBUS_READ_COUNT);
  if (!res.success || res.isException || res.count < 2) return -2;
  moisture = static_cast<int16_t>(res.values[0]);
  temperature = static_cast<int16_t>(res.values[1]);
  return 0;
}
#endif

#ifdef SENSOR_INA226
static void diagIna226WriteReg(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  Wire.endTransmission();
}

static uint16_t diagIna226ReadReg(uint8_t reg) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  if (Wire.requestFrom((uint8_t)INA226_ADDR, (uint8_t)2) >= 2) {
    uint16_t msb = Wire.read();
    uint16_t lsb = Wire.read();
    return (msb << 8) | lsb;
  }
  return 0;
}

static uint16_t diagIna226ReadBusVoltage() {
  uint16_t raw = diagIna226ReadReg(0x02);
  return (uint16_t)(((uint32_t)raw * 5UL) / 4UL);
}
#endif

/* ======================== Serial Input Helpers ======================== */

static String readLine() {
  while (!Serial.available()) { delay(10); }
  String s = Serial.readStringUntil('\n');
  s.trim();
  return s;
}

static int readInt(const char* prompt) {
  Serial.print(prompt);
  return readLine().toInt();
}

static void pressAnyKey() {
  Serial.println(F("\nPress Enter to continue..."));
  readLine();
}

static void printNA(const char* name) {
  Serial.printf("[%s] Not enabled in this build\n", name);
}

/* ============================ Submenu: GPIO ============================ */

static void menuGPIO() {
  while (true) {
    Serial.println(F("\n--- GPIO Control ---"));
    Serial.println(F(" [1] Set pin mode"));
    Serial.println(F(" [2] Digital write"));
    Serial.println(F(" [3] Digital read"));
    Serial.println(F(" [4] Analog read"));
    Serial.println(F(" [5] Toggle pin"));
    Serial.println(F(" [6] Scan all GPIOs (read state 0-39)"));
    Serial.println(F(" [0] Back"));
    int ch = readInt("> ");

    switch (ch) {
      case 0: return;

      case 1: {
        int pin = readInt("Pin: ");
        Serial.println(F(" 1=INPUT 2=OUTPUT 3=INPUT_PULLUP 4=INPUT_PULLDOWN"));
        int mode = readInt("Mode: ");
        const uint8_t modes[] = {INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN};
        if (mode >= 1 && mode <= 4) {
          pinMode(pin, modes[mode - 1]);
          Serial.printf("Pin %d mode set.\n", pin);
        } else Serial.println(F("Invalid mode."));
        break;
      }

      case 2: {
        int pin = readInt("Pin: ");
        int val = readInt("State (0/1): ");
        pinMode(pin, OUTPUT);
        digitalWrite(pin, val ? HIGH : LOW);
        Serial.printf("Pin %d -> %s\n", pin, val ? "HIGH" : "LOW");
        break;
      }

      case 3: {
        int pin = readInt("Pin: ");
        Serial.printf("Pin %d = %d\n", pin, digitalRead(pin));
        break;
      }

      case 4: {
        int pin = readInt("Pin: ");
        analogSetAttenuation(ADC_11db);
        analogReadResolution(12);
        int raw = analogRead(pin);
        uint32_t mv = analogReadMilliVolts(pin);
        Serial.printf("Pin %d: raw=%d, %u mV\n", pin, raw, mv);
        break;
      }

      case 5: {
        int pin = readInt("Pin: ");
        pinMode(pin, OUTPUT);
        int cur = digitalRead(pin);
        digitalWrite(pin, cur ? LOW : HIGH);
        Serial.printf("Pin %d toggled: %s -> %s\n", pin,
                       cur ? "HIGH" : "LOW", cur ? "LOW" : "HIGH");
        break;
      }

      case 6: {
        Serial.println(F("GPIO state scan (0-39):"));
        for (int i = 0; i <= 39; i++) {
          Serial.printf("  GPIO%02d = %d", i, digitalRead(i));
          if ((i + 1) % 4 == 0) Serial.println();
        }
        Serial.println();
        break;
      }
    }
  }
}

/* ============================ Submenu: I2C ============================ */

static void i2cScan(TwoWire& wire, const char* name) {
  Serial.printf("Scanning %s...\n", name);
  int found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    wire.beginTransmission(addr);
    if (wire.endTransmission() == 0) {
      Serial.printf("  Found device at 0x%02X\n", addr);
      found++;
    }
  }
  Serial.printf("Scan complete: %d device(s) found.\n", found);
}

static void menuI2C() {
  while (true) {
    Serial.println(F("\n--- I2C Bus ---"));
    Serial.println(F(" [1] Scan main bus (Wire)"));
#ifdef EEPROM_USE_WIRE1
    Serial.println(F(" [2] Scan EEPROM bus (Wire1)"));
#endif
    Serial.println(F(" [3] Read register (addr, reg)"));
    Serial.println(F(" [4] Write register (addr, reg, value)"));
    Serial.println(F(" [0] Back"));
    int ch = readInt("> ");

    switch (ch) {
      case 0: return;
      case 1: i2cScan(Wire, "Wire"); break;
#ifdef EEPROM_USE_WIRE1
      case 2: i2cScan(Wire1, "Wire1"); break;
#endif
      case 3: {
        int addr = readInt("I2C addr (decimal): ");
        int reg = readInt("Register (decimal): ");
        Wire.beginTransmission(addr);
        Wire.write(reg);
        if (Wire.endTransmission(false) == 0 &&
            Wire.requestFrom((uint8_t)addr, (uint8_t)1) >= 1) {
          Serial.printf("0x%02X reg 0x%02X = 0x%02X (%d)\n", addr, reg, Wire.read(), Wire.read());
        } else {
          Serial.println(F("Read failed / no ACK."));
        }
        break;
      }
      case 4: {
        int addr = readInt("I2C addr (decimal): ");
        int reg = readInt("Register (decimal): ");
        int val = readInt("Value (decimal): ");
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.write(val);
        int err = Wire.endTransmission();
        Serial.printf("Write 0x%02X -> reg 0x%02X = 0x%02X: %s\n",
                       addr, reg, val, err == 0 ? "OK" : "FAIL");
        break;
      }
    }
  }
}

/* ============================ Submenu: RTC ============================ */

static void menuRTC() {
  while (true) {
    Serial.println(F("\n--- RTC (DS3231) ---"));
    Serial.println(F(" [1] Read date/time"));
    Serial.println(F(" [2] Read temperature"));
    Serial.println(F(" [3] Set date/time (YYYY-MM-DD HH:MM:SS)"));
    Serial.println(F(" [4] Configure alarm (minutes)"));
    Serial.println(F(" [5] Read alarm status flags"));
    Serial.println(F(" [0] Back"));
    int ch = readInt("> ");

    switch (ch) {
      case 0: return;

      case 1: {
        rtcNow = rtc.GetDateTime();
        formatDateTime(rtcNow);
        Serial.printf("RTC: %s\n", dateStr);
        break;
      }

      case 2: {
        RtcTemperature temp = rtc.GetTemperature();
        Serial.printf("RTC temp: %.2f C\n", temp.AsFloatDegC());
        break;
      }

      case 3: {
        Serial.print(F("Enter YYYY-MM-DD HH:MM:SS: "));
        String s = readLine();
        int Y, M, D, h, m, sc;
        if (sscanf(s.c_str(), "%d-%d-%d %d:%d:%d", &Y, &M, &D, &h, &m, &sc) == 6) {
          RtcDateTime dt(Y, M, D, h, m, sc);
          rtc.SetDateTime(dt);
          Serial.println(F("RTC set OK."));
        } else {
          Serial.println(F("Parse error. Use YYYY-MM-DD HH:MM:SS"));
        }
        break;
      }

      case 4: {
        int mins = readInt("Alarm in N minutes: ");
        if (mins > 0) {
          configureAlarm(mins);
          Serial.printf("Alarm set for %d min.\n", mins);
        }
        break;
      }

      case 5: {
        DS3231AlarmFlag af = rtc.LatchAlarmsTriggeredFlags();
        Serial.printf("Alarm flags: A1=%d A2=%d\n",
                       (af & DS3231AlarmFlag_Alarm1) ? 1 : 0,
                       (af & DS3231AlarmFlag_Alarm2) ? 1 : 0);
        break;
      }
    }
  }
}

/* ============================ Submenu: EEPROM ============================ */

static void menuEEPROM() {
  while (true) {
    Serial.println(F("\n--- EEPROM (I2C) ---"));
    Serial.println(F(" [1] Status (used/free pages)"));
    Serial.println(F(" [2] Read page (hex dump)"));
    Serial.println(F(" [3] Write test pattern, verify"));
    Serial.println(F(" [4] Peek page (decoded record)"));
    Serial.println(F(" [5] Clear header (resetEEPromHeader)"));
    Serial.println(F(" [0] Back"));
    int ch = readInt("> ");

    switch (ch) {
      case 0: return;

      case 1: {
        uint16_t free = getEEPROMFreePages();
        uint16_t total = EEPROM_MAX_PAGES - EEPROM_HEADER_SIZE;
        Serial.printf("EEPROM: %u/%u pages used, %u free, writePtr=%u\n",
                       total - free, total, free, eepromWritePage);
        break;
      }

      case 2: {
        int page = readInt("Page index: ");
        uint16_t addr = (EEPROM_HEADER_SIZE + page) * EEPROM_PAGE_SIZE;
        byte buf[EEPROM_PAGE_SIZE];
        i2c_eeprom_read_buffer(EEPROM_ADDRESS, addr, buf, EEPROM_PAGE_SIZE);
        Serial.printf("Page %d (addr 0x%04X):\n", page, addr);
        for (int i = 0; i < EEPROM_PAGE_SIZE; i++) {
          Serial.printf("%02X ", buf[i]);
          if ((i + 1) % 16 == 0) Serial.println();
        }
        break;
      }

      case 3: {
        // Use a high page to avoid clobbering data
        uint16_t testPage = EEPROM_MAX_PAGES - 1;
        uint16_t addr = testPage * EEPROM_PAGE_SIZE;

        // Save original
        byte orig[EEPROM_PAGE_SIZE];
        i2c_eeprom_read_buffer(EEPROM_ADDRESS, addr, orig, EEPROM_PAGE_SIZE);

        // Write test pattern
        byte pattern[EEPROM_PAGE_SIZE];
        for (int i = 0; i < EEPROM_PAGE_SIZE; i++) pattern[i] = (byte)(i ^ 0xAA);
        i2c_eeprom_write_page(EEPROM_ADDRESS, addr, pattern, EEPROM_PAGE_SIZE);
        delay(10);

        // Read back and verify
        byte readback[EEPROM_PAGE_SIZE];
        i2c_eeprom_read_buffer(EEPROM_ADDRESS, addr, readback, EEPROM_PAGE_SIZE);

        bool ok = true;
        for (int i = 0; i < EEPROM_PAGE_SIZE; i++) {
          if (readback[i] != pattern[i]) { ok = false; break; }
        }
        Serial.printf("EEPROM write/read test: %s\n", ok ? "PASS" : "FAIL");

        // Restore original
        i2c_eeprom_write_page(EEPROM_ADDRESS, addr, orig, EEPROM_PAGE_SIZE);
        delay(10);
        break;
      }

      case 4: {
        int page = readInt("Page index (-1 for status): ");
        peekEEPROM(page);
        break;
      }

      case 5: {
        Serial.println(F("Clearing EEPROM header..."));
        resetEEPromHeader(EEPROM_ADDRESS);
        eepromWritePage = 0;
        Serial.println(F("Done."));
        break;
      }
    }
  }
}

/* ============================ Submenu: SD ============================ */

static void menuSD() {
#ifndef SD_MODULE
  printNA("SD"); return;
#else
  while (true) {
    Serial.println(F("\n--- SD Card ---"));
    Serial.println(F(" [1] Mount / detect"));
    Serial.println(F(" [2] Card info (free space)"));
    Serial.println(F(" [3] List root directory"));
    Serial.println(F(" [4] Write test file, read back, delete"));
    Serial.println(F(" [5] Dump EEPROM to SD"));
    Serial.println(F(" [6] Power OFF SD"));
    Serial.println(F(" [7] Power ON SD"));
    Serial.println(F(" [0] Back"));
    int ch = readInt("> ");

    switch (ch) {
      case 0: return;

      case 1: {
        bool ok = SD.begin(SdSpiConfig(SD_PIN_CS, SHARED_SPI, SD_SCK_MHZ(20), &sdSPI));
        Serial.printf("SD mount: %s\n", ok ? "OK" : "FAIL");
        break;
      }

      case 2: {
        uint32_t mb = getSDFreeMB();
        Serial.printf("SD free: %u MB\n", mb);
        break;
      }

      case 3: {
        SdFile root;
        if (root.open("/", O_READ)) {
          SdFile entry;
          Serial.println(F("Root directory:"));
          while (entry.openNext(&root, O_READ)) {
            char name[64];
            entry.getName(name, sizeof(name));
            if (entry.isDir()) {
              Serial.printf("  [DIR]  %s\n", name);
            } else {
              Serial.printf("  %8lu  %s\n", (unsigned long)entry.fileSize(), name);
            }
            entry.close();
          }
          root.close();
        } else {
          Serial.println(F("Failed to open root. Is SD mounted?"));
        }
        break;
      }

      case 4: {
        const char* testName = "/_diag_test.txt";
        const char* testData = "n-logger diag test OK";
        SdFile f;
        if (f.open(testName, O_WRITE | O_CREAT | O_TRUNC)) {
          f.println(testData);
          f.close();
          Serial.println(F("Write OK."));
        } else {
          Serial.println(F("Write FAIL.")); break;
        }
        if (f.open(testName, O_READ)) {
          char buf[64];
          int n = f.read(buf, sizeof(buf) - 1);
          buf[n > 0 ? n : 0] = '\0';
          f.close();
          Serial.printf("Read back: \"%s\"\n", buf);
        }
        SD.remove(testName);
        Serial.println(F("Test file deleted."));
        break;
      }

      case 5: {
        Serial.println(F("Dumping EEPROM to SD..."));
        uint8_t ok = dumpEEPROM();
        Serial.printf("Dump: %s\n", ok ? "OK" : "FAIL");
        break;
      }

      case 6:
#if SD_EN_PIN != -1
        setPinInactive(SD_EN_PIN, SD_EN_ACTIVE_HIGH, false);
        Serial.println(F("SD power OFF."));
#else
        Serial.println(F("SD_EN_PIN not defined (-1)."));
#endif
        break;

      case 7:
#if SD_EN_PIN != -1
        setPinActive(SD_EN_PIN, SD_EN_ACTIVE_HIGH);
        delay(10);
        Serial.println(F("SD power ON."));
#else
        Serial.println(F("SD_EN_PIN not defined (-1)."));
#endif
        break;
    }
  }
#endif
}

/* ========================= Submenu: Sensors ========================= */

/* Continuous read loop helper: reads at 1 Hz, any serial input stops it */
#define CONTINUOUS_LOOP(label, readExpr, fmtExpr) do { \
  Serial.println(F("Continuous " label " (any key to stop)...")); \
  while (!Serial.available()) { \
    auto _val = (readExpr); \
    fmtExpr; \
    delay(1000); \
  } \
  while (Serial.available()) Serial.read(); \
} while (0)

#ifdef SENSOR_MAXBOTIX_UART
static void menuMaxBotix() {
  while (true) {
    Serial.println(F("\n--- MaxBotix ---"));
    Serial.printf(" State: %s\n", diagMaxbotixOn ? "ON" : "OFF");
    Serial.println(F(" [1] Power ON"));
    Serial.println(F(" [2] Power OFF"));
    Serial.println(F(" [3] Read single sample"));
    Serial.println(F(" [4] Continuous read (1/sec)"));
    Serial.println(F(" [5] Power cycle (OFF->ON->warmup)"));
    Serial.println(F(" [0] Back"));
    int ch = readInt("> ");

    switch (ch) {
      case 0: return;
      case 1:
        gpio_hold_dis((gpio_num_t)MAXBOTIX_PWR_PIN);
        setPinActive(MAXBOTIX_PWR_PIN, MAXBOTIX_PWR_ACTIVE_HIGH);
        delay(MAXBOTIX_WARMUP_MS);
        diagMaxbotixSerial.begin(9600, SERIAL_8N1, MAXBOTIX_RX_PIN, -1);
        while (diagMaxbotixSerial.available()) diagMaxbotixSerial.read();
        diagMaxbotixOn = true;
        Serial.println(F("MaxBotix ON."));
        break;
      case 2:
        diagMaxbotixSerial.end();
        setPinInactive(MAXBOTIX_PWR_PIN, MAXBOTIX_PWR_ACTIVE_HIGH, false);
        diagMaxbotixOn = false;
        Serial.println(F("MaxBotix OFF."));
        break;
      case 3:
        if (!diagMaxbotixOn) { Serial.println(F("Power ON first.")); break; }
        Serial.printf("Reading: %d mm\n", diagMaxbotixReadSingle());
        break;
      case 4:
        if (!diagMaxbotixOn) { Serial.println(F("Power ON first.")); break; }
        CONTINUOUS_LOOP("MaxBotix",
          diagMaxbotixReadSingle(),
          Serial.printf("[MaxBotix] %d mm\n", _val));
        break;
      case 5:
        diagMaxbotixSerial.end();
        setPinInactive(MAXBOTIX_PWR_PIN, MAXBOTIX_PWR_ACTIVE_HIGH, false);
        delay(100);
        gpio_hold_dis((gpio_num_t)MAXBOTIX_PWR_PIN);
        setPinActive(MAXBOTIX_PWR_PIN, MAXBOTIX_PWR_ACTIVE_HIGH);
        Serial.printf("Warming up %d ms...\n", MAXBOTIX_WARMUP_MS);
        delay(MAXBOTIX_WARMUP_MS);
        diagMaxbotixSerial.begin(9600, SERIAL_8N1, MAXBOTIX_RX_PIN, -1);
        while (diagMaxbotixSerial.available()) diagMaxbotixSerial.read();
        diagMaxbotixOn = true;
        Serial.println(F("Power cycle done."));
        break;
    }
  }
}
#endif

#if defined(SENSOR_WATER_TEMP) || defined(SENSOR_MLX90614)
static void menuWaterTemp() {
  while (true) {
    Serial.println(F("\n--- MLX90614 ---"));
    Serial.printf(" State: %s\n", diagWaterTempOn ? "ON" : "OFF");
    Serial.println(F(" [1] Power ON (+ I2C bus enable)"));
    Serial.println(F(" [2] Power OFF"));
    Serial.println(F(" [3] Read object temp"));
    Serial.println(F(" [4] Read ambient temp"));
    Serial.println(F(" [5] Read both"));
    Serial.println(F(" [6] Continuous read (1/sec)"));
    Serial.println(F(" [0] Back"));
    int ch = readInt("> ");

    switch (ch) {
      case 0: return;
      case 1:
        setPinActive(I2C_EN_PIN, I2C_EN_ACTIVE_HIGH);
#if WATER_TEMP_PWR_PIN != -1
        setPinActive(WATER_TEMP_PWR_PIN, WATER_TEMP_PWR_ACTIVE_HIGH);
#endif
        delay(WATER_TEMP_WARMUP_MS);
        diagWaterTempOn = true;
        Serial.println(F("MLX90614 ON."));
        break;
      case 2:
#if WATER_TEMP_PWR_PIN != -1
        setPinInactive(WATER_TEMP_PWR_PIN, WATER_TEMP_PWR_ACTIVE_HIGH, false);
#endif
        diagWaterTempOn = false;
        Serial.println(F("MLX90614 OFF."));
        break;
      case 3:
        if (!diagWaterTempOn) { Serial.println(F("Power ON first.")); break; }
        { int16_t t = diagMlx90614ReadTemp(0x07);
          Serial.printf("Object: %d.%d C\n", t / 10, abs(t % 10)); }
        break;
      case 4:
        if (!diagWaterTempOn) { Serial.println(F("Power ON first.")); break; }
        { int16_t t = diagMlx90614ReadTemp(0x06);
          Serial.printf("Ambient: %d.%d C\n", t / 10, abs(t % 10)); }
        break;
      case 5:
        if (!diagWaterTempOn) { Serial.println(F("Power ON first.")); break; }
        { int16_t obj = diagMlx90614ReadTemp(0x07);
          int16_t amb = diagMlx90614ReadTemp(0x06);
          Serial.printf("Object: %d.%d C  Ambient: %d.%d C\n",
                         obj / 10, abs(obj % 10), amb / 10, abs(amb % 10)); }
        break;
      case 6:
        if (!diagWaterTempOn) { Serial.println(F("Power ON first.")); break; }
        Serial.println(F("Continuous MLX90614 (any key to stop)..."));
        while (!Serial.available()) {
          int16_t obj = diagMlx90614ReadTemp(0x07);
          int16_t amb = diagMlx90614ReadTemp(0x06);
          Serial.printf("[MLX] Obj=%d.%d  Amb=%d.%d C\n",
                         obj / 10, abs(obj % 10), amb / 10, abs(amb % 10));
          delay(1000);
        }
        while (Serial.available()) Serial.read();
        break;
    }
  }
}
#endif

#ifdef SENSOR_RADAR
static void menuRadar() {
  while (true) {
    Serial.println(F("\n--- Radar ---"));
    Serial.printf(" State: %s\n", diagRadarOn ? "ON" : "OFF");
    Serial.println(F(" [1] Power ON"));
    Serial.println(F(" [2] Power OFF"));
    Serial.println(F(" [3] Read single sample"));
    Serial.println(F(" [4] Continuous read (1/sec)"));
    Serial.println(F(" [0] Back"));
    int ch = readInt("> ");

    switch (ch) {
      case 0: return;
      case 1:
        setPinActive(RADAR_PWR_PIN, RADAR_PWR_ACTIVE_HIGH);
        delay(RADAR_WARMUP_MS);
        diagRadarSerial.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
        while (diagRadarSerial.available()) diagRadarSerial.read();
        diagRadarOn = true;
        Serial.println(F("Radar ON."));
        break;
      case 2:
        diagRadarSerial.end();
        setPinInactive(RADAR_PWR_PIN, RADAR_PWR_ACTIVE_HIGH, false);
        diagRadarOn = false;
        Serial.println(F("Radar OFF."));
        break;
      case 3:
        if (!diagRadarOn) { Serial.println(F("Power ON first.")); break; }
        Serial.printf("Reading: %d mm\n", diagRadarReadSingle());
        break;
      case 4:
        if (!diagRadarOn) { Serial.println(F("Power ON first.")); break; }
        CONTINUOUS_LOOP("Radar",
          diagRadarReadSingle(),
          Serial.printf("[Radar] %d mm\n", _val));
        break;
    }
  }
}
#endif

#ifdef SENSOR_SOIL_3TIP
static void menuSoil() {
  while (true) {
    Serial.println(F("\n--- Soil 3-Probe ---"));
    Serial.printf(" State: %s\n", diagSoilOn ? "ON" : "OFF");
    Serial.println(F(" [1] Power ON (+ RS-485)"));
    Serial.println(F(" [2] Power OFF (+ RS-485)"));
    Serial.println(F(" [3] Read single (moisture + temp)"));
    Serial.println(F(" [4] Continuous read (1/sec)"));
    Serial.println(F(" [0] Back"));
    int ch = readInt("> ");

    switch (ch) {
      case 0: return;
      case 1:
#ifdef RS485_EN_PIN
        setPinActive(RS485_EN_PIN, RS485_EN_ACTIVE_HIGH);
#endif
        setPinActive(SOIL3_PWR_PIN, SOIL3_PWR_ACTIVE_HIGH);
        delay(SOIL3_WARMUP_MS);
        diagSoilSerial.begin(SOIL3_BAUD, SERIAL_8N1, SOIL3_RX_PIN, SOIL3_TX_PIN);
        while (diagSoilSerial.available()) diagSoilSerial.read();
        diagSoilOn = true;
        Serial.println(F("Soil sensor ON."));
        break;
      case 2:
        diagSoilSerial.end();
        setPinInactive(SOIL3_PWR_PIN, SOIL3_PWR_ACTIVE_HIGH, false);
#ifdef RS485_EN_PIN
        setPinInactive(RS485_EN_PIN, RS485_EN_ACTIVE_HIGH, false);
#endif
        diagSoilOn = false;
        Serial.println(F("Soil sensor OFF."));
        break;
      case 3:
        if (!diagSoilOn) { Serial.println(F("Power ON first.")); break; }
        { int16_t m, t;
          int16_t rc = diagSoil3ReadBoth(m, t);
          if (rc == 0) Serial.printf("Moisture=%d (/10=%%RH)  Temp=%d (/10=C)\n", m, t);
          else Serial.printf("Read error: %d\n", rc); }
        break;
      case 4:
        if (!diagSoilOn) { Serial.println(F("Power ON first.")); break; }
        Serial.println(F("Continuous Soil (any key to stop)..."));
        while (!Serial.available()) {
          int16_t m, t;
          if (diagSoil3ReadBoth(m, t) == 0)
            Serial.printf("[Soil] M=%d T=%d\n", m, t);
          else
            Serial.println(F("[Soil] read error"));
          delay(1000);
        }
        while (Serial.available()) Serial.read();
        break;
    }
  }
}
#endif

#ifdef SENSOR_BATTERY_ADC
static void menuBatteryADC() {
  while (true) {
    Serial.println(F("\n--- Battery ADC ---"));
    Serial.println(F(" [1] Read once"));
    Serial.println(F(" [2] Continuous read (1/sec)"));
    Serial.println(F(" [0] Back"));
    int ch = readInt("> ");

    switch (ch) {
      case 0: return;
      case 1: {
        analogSetAttenuation(ADC_11db);
        analogReadResolution(12);
        uint32_t mv = analogReadMilliVolts(BATTERY_ADC_PIN) * BATTERY_ADC_MULTIPLIER;
        Serial.printf("Battery: %u mV\n", mv);
        break;
      }
      case 2:
        Serial.println(F("Continuous Battery ADC (any key to stop)..."));
        analogSetAttenuation(ADC_11db);
        analogReadResolution(12);
        while (!Serial.available()) {
          uint32_t mv = analogReadMilliVolts(BATTERY_ADC_PIN) * BATTERY_ADC_MULTIPLIER;
          Serial.printf("[BAT] %u mV\n", mv);
          delay(1000);
        }
        while (Serial.available()) Serial.read();
        break;
    }
  }
}
#endif

#ifdef SENSOR_INA226
static void menuINA226() {
  while (true) {
    Serial.println(F("\n--- INA226 ---"));
    Serial.println(F(" [1] Read bus voltage (triggered)"));
    Serial.println(F(" [2] Continuous read (1/sec)"));
    Serial.println(F(" [3] Power down chip"));
    Serial.println(F(" [0] Back"));
    int ch = readInt("> ");

    switch (ch) {
      case 0: return;
      case 1: {
        diagIna226WriteReg(0x00, 0x4726);
        delay(INA226_WARMUP_MS);
        Serial.printf("Bus voltage: %u mV\n", diagIna226ReadBusVoltage());
        break;
      }
      case 2:
        Serial.println(F("Continuous INA226 (any key to stop)..."));
        while (!Serial.available()) {
          diagIna226WriteReg(0x00, 0x4726);
          delay(INA226_WARMUP_MS);
          Serial.printf("[INA226] %u mV\n", diagIna226ReadBusVoltage());
          delay(1000 - INA226_WARMUP_MS);
        }
        while (Serial.available()) Serial.read();
        break;
      case 3:
        diagIna226WriteReg(0x00, 0x4700);
        Serial.println(F("INA226 powered down."));
        break;
    }
  }
}
#endif

static void menuSensors() {
  while (true) {
    Serial.println(F("\n--- Sensors ---"));
#ifdef SENSOR_MAXBOTIX_UART
    Serial.println(F(" [1] MaxBotix (UART)"));
#else
    Serial.println(F(" [1] MaxBotix (UART)        [N/A]"));
#endif
#if defined(SENSOR_WATER_TEMP) || defined(SENSOR_MLX90614)
    Serial.println(F(" [2] Water Temp / MLX90614"));
#else
    Serial.println(F(" [2] Water Temp / MLX90614  [N/A]"));
#endif
#ifdef SENSOR_RADAR
    Serial.println(F(" [3] Radar (Modbus)"));
#else
    Serial.println(F(" [3] Radar (Modbus)         [N/A]"));
#endif
#ifdef SENSOR_SOIL_3TIP
    Serial.println(F(" [4] Soil 3-Probe (RS-485)"));
#else
    Serial.println(F(" [4] Soil 3-Probe (RS-485)  [N/A]"));
#endif
#ifdef SENSOR_BATTERY_ADC
    Serial.println(F(" [5] Battery ADC"));
#else
    Serial.println(F(" [5] Battery ADC            [N/A]"));
#endif
#ifdef SENSOR_INA226
    Serial.println(F(" [6] INA226"));
#else
    Serial.println(F(" [6] INA226                 [N/A]"));
#endif
    Serial.println(F(" [7] Full sensor cycle (sensorReadAll)"));
    Serial.println(F(" [0] Back"));
    int ch = readInt("> ");

    switch (ch) {
      case 0: return;
#ifdef SENSOR_MAXBOTIX_UART
      case 1: menuMaxBotix(); break;
#else
      case 1: printNA("MaxBotix"); break;
#endif
#if defined(SENSOR_WATER_TEMP) || defined(SENSOR_MLX90614)
      case 2: menuWaterTemp(); break;
#else
      case 2: printNA("MLX90614"); break;
#endif
#ifdef SENSOR_RADAR
      case 3: menuRadar(); break;
#else
      case 3: printNA("Radar"); break;
#endif
#ifdef SENSOR_SOIL_3TIP
      case 4: menuSoil(); break;
#else
      case 4: printNA("Soil 3-Probe"); break;
#endif
#ifdef SENSOR_BATTERY_ADC
      case 5: menuBatteryADC(); break;
#else
      case 5: printNA("Battery ADC"); break;
#endif
#ifdef SENSOR_INA226
      case 6: menuINA226(); break;
#else
      case 6: printNA("INA226"); break;
#endif
      case 7: {
        Serial.println(F("Running sensorReadAll()..."));
        memset(&telemetryPayload, 0, sizeof(telemetryPayload));
        sensorReadAll(telemetryPayload);
        Serial.println(F("Results:"));
        for (int i = 0; i < MAX_SENSOR_READINGS; i++) {
          Serial.printf("  reading[%d] = %d\n", i, telemetryPayload.sensorReadings[i]);
        }
#if defined(SENSOR_BATTERY_ADC) || defined(SENSOR_INA226)
        Serial.printf("  battery = %u\n", telemetryPayload.battery);
#endif
        Serial.printf("  status  = 0x%08X\n", telemetryPayload.status);
        break;
      }
    }
  }
}

/* ============================ Submenu: BLE ============================ */

static void menuBLE() {
  while (true) {
    Serial.println(F("\n--- BLE ---"));
    Serial.printf(" Active: %s  Connected: %s\n",
                   bleActive ? "yes" : "no",
                   bleDeviceConnected ? "yes" : "no");
    Serial.println(F(" [1] Start advertising"));
    Serial.println(F(" [2] Stop BLE"));
    Serial.println(F(" [3] Status"));
    Serial.println(F(" [4] Wait for connection (30s)"));
    Serial.println(F(" [0] Back"));
    int ch = readInt("> ");

    switch (ch) {
      case 0: return;
      case 1:
        startBLE();
        Serial.println(F("BLE advertising started."));
        break;
      case 2:
        stopBLE();
        Serial.println(F("BLE stopped."));
        break;
      case 3:
        Serial.printf("BLE active=%d connected=%d streaming=%d\n",
                       bleActive, bleDeviceConnected, bleStreamingActive);
        break;
      case 4: {
        if (!bleActive) startBLE();
        Serial.println(F("Waiting 30s for BLE connection..."));
        unsigned long t0 = millis();
        while (!bleDeviceConnected && (millis() - t0) < 30000) {
          delay(100);
        }
        Serial.printf("Result: %s\n", bleDeviceConnected ? "CONNECTED" : "TIMEOUT");
        break;
      }
    }
  }
}

/* ============================ Submenu: GPRS ============================ */

static void menuGPRS() {
#ifndef GPRS_MODULE
  printNA("GPRS"); return;
#else
  while (true) {
    Serial.println(F("\n--- GPRS Modem ---"));
    Serial.println(F(" [1] Power ON + init"));
    Serial.println(F(" [2] Signal quality"));
    Serial.println(F(" [3] Sleep modem"));
    Serial.println(F(" [4] Send AT command (raw)"));
    Serial.println(F(" [0] Back"));
    int ch = readInt("> ");

    switch (ch) {
      case 0: return;
      case 1: {
        Serial.println(F("Initializing modem..."));
        bool ok = initA7670();
        Serial.printf("initA7670: %s\n", ok ? "OK" : "FAIL");
        break;
      }
      case 2: {
        int16_t sq = modem.getSignalQuality();
        Serial.printf("Signal quality: %d\n", sq);
        break;
      }
      case 3:
        sleepA7670();
        Serial.println(F("Modem sleeping."));
        break;
      case 4: {
        Serial.print(F("AT cmd: "));
        String cmd = readLine();
        SerialAT.println(cmd);
        delay(500);
        while (SerialAT.available()) {
          Serial.write(SerialAT.read());
        }
        Serial.println();
        break;
      }
    }
  }
#endif
}

/* ====================== Submenu: Deep Sleep & Power ====================== */

static void menuSleep() {
  while (true) {
    Serial.println(F("\n--- Deep Sleep & Power ---"));
    Serial.println(F(" [1] Enter deep sleep (via prepareForDeepSleep)"));
    Serial.println(F(" [2] Enter deep sleep (raw timer, no RTC alarm)"));
    Serial.println(F(" [3] Hold pin state (gpio_hold_en)"));
    Serial.println(F(" [4] Release pin hold (gpio_hold_dis)"));
    Serial.println(F(" [5] Release all holds (gpio_deep_sleep_hold_dis)"));
    Serial.println(F(" [6] Show wake cause"));
    Serial.println(F(" [7] All power rails OFF"));
    Serial.println(F(" [8] All power rails ON"));
    Serial.println(F(" [9] Radios off"));
    Serial.println(F(" [0] Back"));
    int ch = readInt("> ");

    switch (ch) {
      case 0: return;

      case 1: {
        int mins = readInt("Sleep minutes: ");
        if (mins <= 0) mins = 1;
        Serial.printf("Entering deep sleep for %d min (RTC alarm)...\n", mins);
        Serial.flush();
        delay(100);
        prepareForDeepSleep(mins);
        // never returns
        break;
      }

      case 2: {
        int secs = readInt("Sleep seconds: ");
        if (secs <= 0) secs = 10;
        Serial.printf("Entering raw deep sleep for %d sec (timer only)...\n", secs);
        Serial.flush();
        delay(100);
        esp_sleep_enable_timer_wakeup((uint64_t)secs * 1000000ULL);
        esp_deep_sleep_start();
        break;
      }

      case 3: {
        int pin = readInt("Pin to hold: ");
        int val = readInt("State to set first (0/1): ");
        pinMode(pin, OUTPUT);
        digitalWrite(pin, val ? HIGH : LOW);
        gpio_hold_en((gpio_num_t)pin);
        Serial.printf("Pin %d set to %s and HELD.\n", pin, val ? "HIGH" : "LOW");
        break;
      }

      case 4: {
        int pin = readInt("Pin to release: ");
        gpio_hold_dis((gpio_num_t)pin);
        Serial.printf("Pin %d hold released.\n", pin);
        break;
      }

      case 5:
        gpio_deep_sleep_hold_dis();
        Serial.println(F("All deep-sleep holds released."));
        break;

      case 6: {
        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
        const char* reasons[] = {
          "Undefined", "Undefined", "EXT0", "EXT1", "Timer",
          "Touchpad", "ULP", "GPIO", "UART", "WiFi", "Cocpu", "CocpuInt", "BT"
        };
        int idx = (int)cause;
        Serial.printf("Wake cause: %d (%s)\n", idx,
                       (idx >= 0 && idx < 13) ? reasons[idx] : "Unknown");
        break;
      }

      case 7:
        Serial.println(F("Turning all power rails OFF..."));
        setPinInactive(EN_5V_PIN, EN_5V_ACTIVE_HIGH, false);
        setPinInactive(I2C_EN_PIN, I2C_EN_ACTIVE_HIGH, false);
#if SD_EN_PIN != -1
        setPinInactive(SD_EN_PIN, SD_EN_ACTIVE_HIGH, false);
#endif
#ifdef RS485_EN_PIN
        setPinInactive(RS485_EN_PIN, RS485_EN_ACTIVE_HIGH, false);
#endif
        Serial.println(F("Done."));
        break;

      case 8:
        Serial.println(F("Turning all power rails ON..."));
        setPinActive(EN_5V_PIN, EN_5V_ACTIVE_HIGH);
        setPinActive(I2C_EN_PIN, I2C_EN_ACTIVE_HIGH);
#if SD_EN_PIN != -1
        setPinActive(SD_EN_PIN, SD_EN_ACTIVE_HIGH);
#endif
        Serial.println(F("Done."));
        break;

      case 9:
        boardRadiosOff();
        Serial.println(F("Radios off."));
        break;
    }
  }
}

/* ============================ Submenu: LED ============================ */

static void menuLED() {
  while (true) {
    Serial.println(F("\n--- LED ---"));
    Serial.println(F(" [1] ON"));
    Serial.println(F(" [2] OFF"));
    Serial.println(F(" [3] Blink N times"));
    Serial.println(F(" [0] Back"));
    int ch = readInt("> ");

    switch (ch) {
      case 0: return;
      case 1:
        digitalWrite(LED_PIN, HIGH);
        Serial.println(F("LED ON."));
        break;
      case 2:
        digitalWrite(LED_PIN, LOW);
        Serial.println(F("LED OFF."));
        break;
      case 3: {
        int n = readInt("Blink count: ");
        int ms = readInt("Interval ms: ");
        if (ms <= 0) ms = 200;
        for (int i = 0; i < n; i++) {
          digitalWrite(LED_PIN, HIGH);
          delay(ms);
          digitalWrite(LED_PIN, LOW);
          delay(ms);
        }
        Serial.printf("Blinked %d times.\n", n);
        break;
      }
    }
  }
}

/* ============================ Main Menu ============================ */

static void printMainMenu() {
  Serial.println(F("\n========================================"));
  Serial.println(F("       N-LOGGER DIAGNOSTICS"));
  Serial.println(F("========================================"));
  Serial.printf("Board: %s | Sensor: %s\n", BOARD_NAME, SENSOR_NAME);
  Serial.printf("ChipID: %04X%08X\n", (uint16_t)(device_ID >> 32), (uint32_t)device_ID);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause != ESP_SLEEP_WAKEUP_UNDEFINED) {
    Serial.printf("Woke from deep sleep (cause %d)\n", (int)cause);
  }

  Serial.println();
  Serial.println(F(" [1] GPIO Control"));
  Serial.println(F(" [2] I2C Bus"));
  Serial.println(F(" [3] RTC (DS3231)"));
  Serial.println(F(" [4] EEPROM"));
  Serial.println(F(" [5] SD Card"));
  Serial.println(F(" [6] Sensors"));
  Serial.println(F(" [7] BLE"));
  Serial.println(F(" [8] GPRS Modem"));
  Serial.println(F(" [9] Deep Sleep & Power"));
  Serial.println(F("[10] LED"));
  Serial.println();
  Serial.println(F("Enter choice (0 = reprint menu):"));
}

/* ============================== Setup ============================== */

void setup() {
  // Release any GPIO holds from deep sleep
  gpio_deep_sleep_hold_dis();

  boardInit();

  Serial.begin(115200);
  delay(500); // let USB-CDC settle

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

#ifdef EEPROM_USE_WIRE1
  Wire1.begin(I2C_EEPROM_SDA_PIN, I2C_EEPROM_SCL_PIN);
#endif

  // Load config
  EEPROM.begin(sizeof(ModuleConfig));
  EEPROM.get(0, moduleCfg);
  EEPROM.end();
  normalizeConfig();

  // RTC init
  rtc.Begin();
  rtc.Enable32kHzPin(false);
  rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmBoth);

  // SD SPI init
#ifdef SD_MODULE
#if SD_EN_PIN != -1
  setPinActive(SD_EN_PIN, SD_EN_ACTIVE_HIGH);
  delay(10);
#endif
  sdSPI.begin(SD_PIN_SCLK, SD_PIN_MISO, SD_PIN_MOSI, SD_PIN_CS);
#endif

  boardRadiosOff();

  printMainMenu();
}

/* =============================== Loop =============================== */

void loop() {
  int ch = readInt("> ");

  switch (ch) {
    case 0:  printMainMenu(); break;
    case 1:  menuGPIO(); break;
    case 2:  menuI2C(); break;
    case 3:  menuRTC(); break;
    case 4:  menuEEPROM(); break;
    case 5:  menuSD(); break;
    case 6:  menuSensors(); break;
    case 7:  menuBLE(); break;
    case 8:  menuGPRS(); break;
    case 9:  menuSleep(); break;
    case 10: menuLED(); break;
    default: Serial.println(F("Invalid choice. Enter 0 for menu.")); break;
  }
}
