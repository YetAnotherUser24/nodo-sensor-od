/**
 * @file measurement_storage.cpp
 * @brief Measurement capture, EEPROM buffering, SD batching and telemetry upload.
 *
 * This module centralizes:
 *  - Water-level sampling and temperature/battery acquisition.
 *  - Compact EEPROM page storage.
 *  - Dumping buffered pages to SD.
 *  - Converting CSV records to JSON for ThingsBoard.
 *  - Uploading pending data from SD and EEPROM.
 *
 *  All functionality is preserved exactly as in original implementation.
 */


#include <Arduino.h>
#include <HardwareSerial.h>
#include "utilities.h"
#include "measurement_storage.h"
#include "eeprom_i2c.h"
#include "modem_mqtt.h"
#include "error_sensor.h"
#include "rtc_utils.h"

#include <SdFat.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <Wire.h>
#include <PubSubClient.h>

/* --------------------------------------------------------------------------
 *  External objects and configuration (declared in main .ino)
 * -------------------------------------------------------------------------- */

#ifndef SerialMon
  #define SerialMon Serial
#endif

extern HardwareSerial SensorSerial;

#ifndef MQTT_TOPIC
  #define MQTT_TOPIC "v1/devices/me/telemetry"
#endif

#ifndef UNIX_OFFSET_2000
  #define UNIX_OFFSET_2000 946684800UL
#endif

#ifndef LOCAL_TZ_OFFSET_SEC
  #define LOCAL_TZ_OFFSET_SEC (5 * 3600)
#endif

#ifndef NUM_READINGS
#define NUM_READINGS             5     // Number of sensor readings per measurement
#endif

// Compile-time defaults
#ifndef NUM_READINGS
  #define NUM_READINGS 5
#endif

#ifndef EEPROM_PAGE_SIZE
  #define EEPROM_PAGE_SIZE 32
#endif

#ifndef EEPROM_ADDRESS
  #define EEPROM_ADDRESS 0x50
#endif

#ifndef EEPROM_HEADER_SIZE
  #define EEPROM_HEADER_SIZE 8
#endif

#ifndef EEPROM_MAX_PAGES
  #define EEPROM_MAX_PAGES 2048
#endif

#ifndef EEPROM_FLUSH_AFTER
  #define EEPROM_FLUSH_AFTER 288
#endif

#ifndef EEPROM_SENT_FLAG_OFFSET
  #define EEPROM_SENT_FLAG_OFFSET (EEPROM_PAGE_SIZE - 1)
#endif

#ifndef FLAG_NOT_SENT
  #define FLAG_NOT_SENT 0x00
#endif

#ifndef FLAG_SENT
  #define FLAG_SENT 0x01
#endif

#ifndef LED_PIN
  #define LED_PIN 32
#endif

// Status bits
#ifndef STATUS_SD_CARD_BIT
  #define STATUS_SD_CARD_BIT 17
#endif

#ifndef STATUS_TEMPERATURE_BIT
  #define STATUS_TEMPERATURE_BIT 18
#endif

#ifndef STATUS_WATER_LEVEL_SENSOR_BIT
  #define STATUS_WATER_LEVEL_SENSOR_BIT 16
#endif

// SD Pins
#ifndef SD_MOSI
  #define SD_MOSI 15
#endif
#ifndef SD_MISO
  #define SD_MISO 2
#endif
#ifndef SD_SCLK
  #define SD_SCLK 14
#endif
#ifndef SD_CS
  #define SD_CS 13
#endif

/* --------------------------------------------------------------------------
 *  External globals from main file
 * -------------------------------------------------------------------------- */
struct ModuleConfig {
  int measure_time;
  int bluetooth_time;
  int GPRS_time;
};

struct TelemetryPayload {
  int16_t  freeStorage;
  int16_t  temperature;
  int16_t  humidity;
  int16_t  waterLevelSensor[NUM_READINGS];
  uint32_t battery;
  uint32_t status;
};

extern ModuleConfig moduleCfg;
extern TelemetryPayload telemetryPayload;

extern RTC_DATA_ATTR uint16_t eepromWritePage;
extern byte      eepromPageBuf[EEPROM_PAGE_SIZE];
extern bool      needFlushToSD;

extern SdFat     SD;
extern SdFile    sdFile;
extern SPIClass  sdSPI;
extern char      csvFilename[24];
extern bool      fileOpen;

extern RtcDS3231<TwoWire> rtc;
extern RtcDateTime        rtcNow;
extern char               dateStr[20];

extern PubSubClient mqtt;

// External helpers
extern uint32_t changeBit(uint32_t value, uint8_t bitIndex, bool setBit);
extern int      WaterLevelRead();
extern void     error(int errorNum, int errpin);
extern bool     publishJsonLineToTB(const String& jsonOut);
extern bool     connectMQTT(uint8_t maxAttempts, uint16_t delayMs);
extern void     toFinalName(const char* pendingName, char* out, size_t outSize);
extern bool     mqttAvailable;

/* --------------------------------------------------------------------------
 *  SD / EEPROM Helpers (unchanged logic, documented)
 * -------------------------------------------------------------------------- */

String markLineSent(const String& line) {
  String s = line;

  while (s.length() && s[s.length()-1] == ' ')
    s.remove(s.length()-1);

  int lastComma = s.lastIndexOf(',');
  if (lastComma <= 0) return line;

  return s.substring(0, lastComma) + ", 1";
}

/**
 * @brief Count free EEPROM pages using bit-mapped header.
 */
uint16_t getEEPROMFreePages() {
  uint8_t header[EEPROM_HEADER_SIZE];

  i2c_eeprom_read_buffer(EEPROM_ADDRESS, 0, header, EEPROM_HEADER_SIZE);

  uint16_t used = 0;
  for (uint16_t i = 0; i < EEPROM_HEADER_SIZE; i++) {
    uint8_t b = header[i];
    used +=
      (b & 1) + ((b >> 1) & 1) + ((b >> 2) & 1) + ((b >> 3) & 1) +
      ((b >> 4) & 1) + ((b >> 5) & 1) + ((b >> 6) & 1) + ((b >> 7) & 1);
  }

  uint16_t totalPages = EEPROM_MAX_PAGES - EEPROM_HEADER_SIZE;
  return totalPages - used;
}

/**
 * @brief Returns available SD capacity (MB).
 */
uint32_t getSDFreeMB() {
  if (!SD.begin(SdSpiConfig(SD_CS, SHARED_SPI, SD_SCK_MHZ(20), &sdSPI))) {
    return 0;
  }

  FsVolume* vol = SD.vol();
  if (!vol) return 0;

  uint64_t totalBytes =
    (uint64_t)vol->clusterCount() *
    (uint64_t)vol->sectorsPerCluster() * 512ULL;

  SdFile dir, entry;
  uint64_t usedBytes = 0;

  if (dir.open("/")) {
    while (entry.openNext(&dir, O_READ)) {
      if (!entry.isDir()) usedBytes += entry.fileSize();
      entry.close();
    }
    dir.close();
  }

  uint64_t freeBytes = (totalBytes > usedBytes) ? (totalBytes - usedBytes) : 0;
  return freeBytes / (1024ULL * 1024ULL);
}

/**
 * @brief Ensures valid timing values in module configuration.
 */
void normalizeConfig() {
  if (moduleCfg.measure_time <= 0)
    moduleCfg.measure_time = 1;

  if (moduleCfg.bluetooth_time < 0)
    moduleCfg.bluetooth_time = 0;

  if (moduleCfg.GPRS_time < 0)
    moduleCfg.GPRS_time = 0;
}

/* --------------------------------------------------------------------------
 *  Measurement Capture and EEPROM Packing (takeMeasurement)
 * -------------------------------------------------------------------------- */

void takeMeasurement() {
  /**
   * Reset telemetry structure.
   */
  telemetryPayload.freeStorage = 0;
  telemetryPayload.temperature = 0;
  telemetryPayload.humidity    = 0;
  for (int i = 0; i < NUM_READINGS; i++)
    telemetryPayload.waterLevelSensor[i] = 0;
  telemetryPayload.battery = 0;
  telemetryPayload.status  = 0x00000000;

  rtcNow = rtc.GetDateTime();
  uint32_t secondsSince2000 = rtcNow.TotalSeconds();

  /* Battery reading */
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);
  int16_t measuredVbat = analogReadMilliVolts(BOARD_BAT_ADC_PIN) * 2;
  telemetryPayload.battery = measuredVbat;

  /* Low-battery emergency config update */
  if (measuredVbat < 3400) {
    moduleCfg.measure_time   = 60;
    moduleCfg.bluetooth_time = 1;
    moduleCfg.GPRS_time      = 1440;

    EEPROM.begin(sizeof(ModuleConfig));
    EEPROM.put(0, moduleCfg);
    EEPROM.commit();
    EEPROM.end();
  }

  /* Temperature from RTC */
  int16_t tempCenti = rtc.GetTemperature().AsCentiDegC();
  telemetryPayload.temperature = tempCenti;

  if (tempCenti < -4000 || tempCenti > 8500)
    telemetryPayload.status = changeBit(telemetryPayload.status, STATUS_TEMPERATURE_BIT, true);

  /* Water-level sampling */
  uint8_t nread = 0;
  SensorSerial.begin(9600, SERIAL_8N1, 23, 19);

  digitalWrite(LED_PIN, HIGH);
  digitalWrite(18, HIGH);
  
  // Sensor warmup: MaxBotix needs ~175ms to stabilize after power-on
  delay(200);
  
  // Clear serial buffer of any startup noise
  while (SensorSerial.available()) {
    SensorSerial.read();
  }
  
  digitalWrite(LED_PIN, LOW);

  // Collect exactly 5 readings, exit immediately when done
  uint32_t startMillis = millis();
  const uint32_t TIMEOUT = 1500;  // 1.5s max for all samples
  
  while (nread < NUM_READINGS && (millis() - startMillis) < TIMEOUT) {
    int16_t sample = WaterLevelRead();
    
    if (sample > -2) {
      telemetryPayload.waterLevelSensor[nread] = sample;
      nread++;
      
      // Wait for next sensor output (~100ms cycle on MaxBotix)
      if (nread < NUM_READINGS) {
        delay(120);
      }
    } else {
      // Failed reading, retry after brief delay
      delay(50);
    }
  }
  
  // Mark sensor failure if incomplete
  if (nread < NUM_READINGS) {
    telemetryPayload.status = changeBit(
      telemetryPayload.status,
      STATUS_WATER_LEVEL_SENSOR_BIT,
      true
    );
  }

  digitalWrite(18, LOW);
  SensorSerial.end();

#ifdef DEBUG
  Serial.println("📊 ========== MEASUREMENT TAKEN ==========");
  Serial.print("   Timestamp: ");
  Serial.print(secondsSince2000);
  Serial.print(" (");
  formatDateTime(rtcNow);
  Serial.print(dateStr);
  Serial.println(" GMT)");
  Serial.print("   Battery: ");
  Serial.print(measuredVbat);
  Serial.println(" mV");
  Serial.print("   Temperature: ");
  Serial.print(tempCenti / 100.0, 2);
  Serial.println(" °C");
  Serial.print("   Water Level Readings: [");
  for (int i = 0; i < NUM_READINGS; i++) {
    Serial.print(telemetryPayload.waterLevelSensor[i]);
    if (i < NUM_READINGS - 1) Serial.print(", ");
  }
  Serial.println("]");
  Serial.print("   Status: 0x");
  Serial.println(telemetryPayload.status, HEX);
  Serial.println("==========================================");
#endif

  /* EEPROM page building (unchanged) */
  for (int i = 0; i < 4; i++)
    eepromPageBuf[i] = ((byte*)&secondsSince2000)[i];

  for (int i = 0; i < NUM_READINGS * 2; i++)
    eepromPageBuf[i + 4] = ((byte*)telemetryPayload.waterLevelSensor)[i];

  for (int i = 0; i < 2; i++)
    eepromPageBuf[i + 4 + NUM_READINGS*2] = ((byte*)&measuredVbat)[i];

  for (int i = 0; i < 2; i++)
    eepromPageBuf[i + 4 + NUM_READINGS*2 + 2] = ((byte*)&tempCenti)[i];

  for (int i = 0; i < 4; i++)
    eepromPageBuf[i + 4 + NUM_READINGS*2 + 2 + 2] = ((byte*)&telemetryPayload.status)[i];

  for (int i = 21; i < EEPROM_PAGE_SIZE - 1; i++)
    eepromPageBuf[i] = 0;

  eepromPageBuf[EEPROM_SENT_FLAG_OFFSET] = FLAG_NOT_SENT;

  unsigned int dataAddr =
    (eepromWritePage + EEPROM_HEADER_SIZE) * EEPROM_PAGE_SIZE;

  i2c_eeprom_write_page(EEPROM_ADDRESS, dataAddr, eepromPageBuf, EEPROM_PAGE_SIZE);
  delay(50);

  /* Update header bit */
  byte headerByte =
    i2c_eeprom_read_byte(EEPROM_ADDRESS, (uint16_t)eepromWritePage / 8);

  bitWrite(headerByte, eepromWritePage % 8, 1);
  i2c_eeprom_write_byte(
    EEPROM_ADDRESS,
    (uint16_t)eepromWritePage / 8,
    headerByte
  );

  eepromWritePage++;

  if (eepromWritePage >= EEPROM_FLUSH_AFTER)
    needFlushToSD = true;

  if (needFlushToSD) {
    if (dumpEEPROM()) {
      resetEEPromHeader(EEPROM_ADDRESS);
      eepromWritePage = 0;
      needFlushToSD = false;
    }
  }

  if (eepromWritePage > (EEPROM_MAX_PAGES - EEPROM_HEADER_SIZE))
    eepromWritePage = 0;
}

/* --------------------------------------------------------------------------
 *  Upload routines: SD pending + EEPROM pending
 * -------------------------------------------------------------------------- */

void sendSDPendingToThingsboard() {
  if (!mqttAvailable) {
    Serial.println("MQTT unavailable, skipping SD uploads.");
    return;
  }

  if (!mqtt.connected()) {
    if (!connectMQTT()) {
      Serial.println("MQTT connection lost, skipping SD uploads.");
      mqttAvailable = false;
      return;
    }
  }

  SdFile dir, entry;
  if (!SD.begin(SdSpiConfig(SD_CS, SHARED_SPI, SD_SCK_MHZ(20), &sdSPI)))
    return;

  if (!dir.open("/")) return;

  char name[32];
  uint16_t stillPending = 0;

  while (entry.openNext(&dir, O_READ)) {
    if (!entry.isDir()) {
      entry.getName(name, sizeof(name));
      String fname = name;

      if (fname.endsWith("_P.CSV") && fname.startsWith("20")) {
        String full = String("/") + fname;
        bool pending = processPendingCsvFile(full.c_str());
        if (pending) stillPending++;
      }
    }
    entry.close();
  }

  dir.close();
  
  // Final flush for batch (power optimized)
  for (int i = 0; i < 5; i++) {
    mqtt.loop();
    delay(50);
  }
}

void sendEEPROMToThingsboard() {
  if (!mqttAvailable) {
    Serial.println("MQTT unavailable, skipping EEPROM uploads.");
    return;
  }

  uint16_t pageIndex = 0;
  byte pageBuf[EEPROM_PAGE_SIZE];

  while (pageIndex < (EEPROM_MAX_PAGES - EEPROM_HEADER_SIZE)) {
    unsigned int addr =
      (pageIndex + EEPROM_HEADER_SIZE) * EEPROM_PAGE_SIZE;

    i2c_eeprom_read_buffer(EEPROM_ADDRESS, addr, pageBuf, EEPROM_PAGE_SIZE);

    if (pageBuf[EEPROM_SENT_FLAG_OFFSET] == FLAG_NOT_SENT) {
      uint32_t seconds2000 = *((uint32_t*)&pageBuf[0]);
      int16_t* water = (int16_t*)&pageBuf[4];
      int16_t  vbat  = *((int16_t*)&pageBuf[4 + NUM_READINGS*2]);
      int16_t  temp  = *((int16_t*)&pageBuf[4 + NUM_READINGS*2 + 2]);
      uint32_t status = *((uint32_t*)&pageBuf[4 + NUM_READINGS*2 + 4]);

      uint64_t tsUnix =
        ((uint64_t)seconds2000 + UNIX_OFFSET_2000 + LOCAL_TZ_OFFSET_SEC) * 1000ULL;

      StaticJsonDocument<512> doc;
      doc["ts"] = tsUnix;

      JsonObject vals = doc.createNestedObject("values");
      vals["battery"] = vbat;
      vals["temperature"] = temp;
      vals["status"] = status;

      JsonArray arr = vals.createNestedArray("waterLevelSensor");
      for (int i = 0; i < NUM_READINGS; i++) arr.add(water[i]);

      vals["eepromFreePages"] = getEEPROMFreePages();
      vals["sdFreeMB"]        = getSDFreeMB();

      String jsonOut;
      serializeJson(doc, jsonOut);

#ifdef DEBUG
      Serial.println("📤 MQTT → EEPROM Data:");
      Serial.print("   Payload: ");
      Serial.println(jsonOut);
#endif

      if (!mqtt.connected()) {
        if (!connectMQTT()) {
          Serial.println("MQTT connection lost, stopping EEPROM uploads.");
          mqttAvailable = false;
          return;
        }
      }
      
      bool ok = mqtt.publish(MQTT_TOPIC, jsonOut.c_str());
      
      // Minimal delay for transmission
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
      
      if (ok)
        i2c_eeprom_write_byte(EEPROM_ADDRESS, addr + EEPROM_SENT_FLAG_OFFSET, FLAG_SENT);
    }

    pageIndex++;
  }
  
  // Final flush for batch (power optimized)
  for (int i = 0; i < 5; i++) {
    mqtt.loop();
    delay(50);
  }
}

/* --------------------------------------------------------------------------
 *  dumpEEPROM(): writes all marked pages to SD
 * -------------------------------------------------------------------------- */

uint8_t dumpEEPROM() {
  uint16_t i = 0, j = 0;
  bool keep = true;
  bool failure = false;

  digitalWrite(LED_PIN, HIGH);

  if (!SD.begin(SdSpiConfig(SD_CS, SHARED_SPI, SD_SCK_MHZ(20), &sdSPI))) {
    telemetryPayload.status =
      changeBit(telemetryPayload.status, STATUS_SD_CARD_BIT, true);

    digitalWrite(LED_PIN, LOW);
    return 0;
  }

  while (keep) {
    byte headerByte = i2c_eeprom_read_byte(EEPROM_ADDRESS, i);

    for (j = 0; j < 8; j++) {
      if ((headerByte >> j) & 1) {
        failure = !writeEEPROMLine(i * 8 + j);
        if (failure) { keep = false; break; }
      } else {
        keep = false;
        break;
      }
    }

    i++;
  }

  if (fileOpen) {
    sdFile.close();
    fileOpen = false;
  }

  digitalWrite(LED_PIN, LOW);
  return failure ? 0 : 1;
}

/* --------------------------------------------------------------------------
 *  CSV filename generation
 * -------------------------------------------------------------------------- */

void getPendingFilename(const RtcDateTime& ts) {
  int y = ts.Year();
  int m = ts.Month();
  int d = ts.Day();

  csvFilename[0]  = '/';
  csvFilename[1]  = '2';
  csvFilename[2]  = '0';
  csvFilename[3]  = (y - 2000) / 10 + '0';
  csvFilename[4]  = y % 10 + '0';
  csvFilename[5]  = m / 10 + '0';
  csvFilename[6]  = m % 10 + '0';
  csvFilename[7]  = d / 10 + '0';
  csvFilename[8]  = d % 10 + '0';
  csvFilename[9]  = '_';
  csvFilename[10] = 'P';
  csvFilename[11] = '.';
  csvFilename[12] = 'C';
  csvFilename[13] = 'S';
  csvFilename[14] = 'V';
  csvFilename[15] = '\0';
}

/**
 * @brief Writes one EEPROM row into the proper daily CSV file.
 */
uint8_t writeEEPROMLine(uint16_t n) {
  byte pageBuf[EEPROM_PAGE_SIZE];
  RtcDateTime ts;

  i2c_eeprom_read_buffer(
    EEPROM_ADDRESS,
    (n + EEPROM_HEADER_SIZE) * EEPROM_PAGE_SIZE,
    pageBuf,
    EEPROM_PAGE_SIZE
  );

  uint32_t seconds2000 = ((uint32_t*)pageBuf)[0];
  ts = RtcDateTime(seconds2000);

  int16_t* ptr = (int16_t*)pageBuf;

  uint32_t status32 =
    ((uint32_t)ptr[NUM_READINGS + 4] & 0xFFFF) |
    ((uint32_t)ptr[NUM_READINGS + 5] << 16);

  uint8_t sentFlag = (pageBuf[EEPROM_SENT_FLAG_OFFSET] == FLAG_SENT);

  static uint32_t dayCache = 40;
  if (!fileOpen) {
    getPendingFilename(ts);
    fileOpen = sdFile.open(csvFilename, O_CREAT | O_WRITE | O_APPEND);
    dayCache = ts.Day();
  } else if (dayCache != ts.Day()) {
    sdFile.close();
    getPendingFilename(ts);
    fileOpen = sdFile.open(csvFilename, O_CREAT | O_WRITE | O_APPEND);
    dayCache = ts.Day();
  }

  if (!fileOpen) {
    error(3, LED_PIN);
    return 0;
  }

  formatDateTime(ts);
  sdFile.print(dateStr);
  sdFile.print(", ");
  sdFile.print(seconds2000);

  for (uint16_t k = 0; k < NUM_READINGS + 2; k++) {
    sdFile.print(", ");
    sdFile.print(ptr[k + 2]);
  }

  sdFile.print(", ");
  sdFile.print(status32);

  sdFile.print(", ");
  sdFile.print(sentFlag);

  sdFile.println();
  sdFile.flush();
  return 1;
}

/* --------------------------------------------------------------------------
 *  markEepromSentByTimestamp()
 * -------------------------------------------------------------------------- */

bool markEepromSentByTimestamp(uint32_t secondsSince2000) {
  byte pageBuf[EEPROM_PAGE_SIZE];

  for (uint16_t page = 0; page < (EEPROM_MAX_PAGES - EEPROM_HEADER_SIZE); page++) {
    unsigned int addr = (page + EEPROM_HEADER_SIZE) * EEPROM_PAGE_SIZE;

    i2c_eeprom_read_buffer(EEPROM_ADDRESS, addr, pageBuf, EEPROM_PAGE_SIZE);
    uint32_t ts = ((uint32_t*)pageBuf)[0];

    if (ts == secondsSince2000) {
      if (pageBuf[EEPROM_SENT_FLAG_OFFSET] == FLAG_NOT_SENT) {
        i2c_eeprom_write_byte(
          EEPROM_ADDRESS,
          addr + EEPROM_SENT_FLAG_OFFSET,
          FLAG_SENT
        );
      }
      return true;
    }
  }
  return false;
}

/* --------------------------------------------------------------------------
 *  csvLineToJson() — CSV → JSON conversion
 * -------------------------------------------------------------------------- */

bool csvLineToJson(const String& line, String& jsonOut, uint32_t& outSeconds2000) {
  int idx = 0, start = 0, field = 0;
  int values[NUM_READINGS + 2];
  uint32_t status = 0;
  int sent = 0;
  uint32_t seconds2000 = 0;

  while (true) {
    idx = line.indexOf(',', start);
    String token = (idx == -1) ? line.substring(start) : line.substring(start, idx);
    token.trim();

    if (field == 1)
      seconds2000 = token.toInt();
    else if (field >= 2 && field <= (1 + NUM_READINGS))
      values[field - 2] = token.toInt();
    else if (field == (2 + NUM_READINGS))
      values[NUM_READINGS] = token.toInt();
    else if (field == (3 + NUM_READINGS))
      values[NUM_READINGS + 1] = token.toInt();
    else if (field == (4 + NUM_READINGS))
      status = token.toInt();
    else if (field == (5 + NUM_READINGS))
      sent = token.toInt();

    field++;
    if (idx == -1) break;
    start = idx + 1;
  }

  if (field < (6 + NUM_READINGS))
    return false;

  StaticJsonDocument<512> doc;
  uint64_t tsUnix =
    ((uint64_t)seconds2000 + UNIX_OFFSET_2000 + LOCAL_TZ_OFFSET_SEC) * 1000ULL;

  doc["ts"] = tsUnix;

  JsonObject vals = doc.createNestedObject("values");
  vals["battery"] = values[NUM_READINGS];
  vals["temperature"] = values[NUM_READINGS + 1];
  vals["status"] = status;

  JsonArray arr = vals.createNestedArray("waterLevelSensor");
  for (int i = 0; i < NUM_READINGS; i++)
    arr.add(values[i]);

  jsonOut.clear();
  serializeJson(doc, jsonOut);

  outSeconds2000 = seconds2000;
  return (sent == 0);
}

/* --------------------------------------------------------------------------
 *  processPendingCsvFile() — Handles *_P.CSV
 * -------------------------------------------------------------------------- */

bool processPendingCsvFile(const char* filename) {
  SdFile inFile, outFile;

  char tmpName[28];
  snprintf(tmpName, sizeof(tmpName), "%s.tmp", filename);

  if (!inFile.open(filename, O_READ))
    return true;

  if (!outFile.open(tmpName, O_CREAT | O_TRUNC | O_WRITE)) {
    inFile.close();
    return true;
  }

  bool pendingLeft = false;
  String line;

  while (inFile.available()) {
    int c = inFile.read();

    if (c == '\r') continue;
    if (c == '\n') {
      String json;
      uint32_t sec2000 = 0;

      String newLine = line;
      bool wasPending = csvLineToJson(line, json, sec2000);

      if (wasPending) {
        if (publishJsonLineToTB(json)) {
          newLine = markLineSent(line);
          markEepromSentByTimestamp(sec2000);
        } else {
          pendingLeft = true;
        }
      }

      outFile.print(newLine);
      outFile.print("\n");
      line = "";
    } else {
      line += (char)c;
    }
  }

  if (line.length() > 0) {
    String json;
    uint32_t sec2000 = 0;
    String newLine = line;

    bool wasPending = csvLineToJson(line, json, sec2000);

    if (wasPending) {
      if (publishJsonLineToTB(json)) {
        newLine = markLineSent(line);
        markEepromSentByTimestamp(sec2000);
      } else {
        pendingLeft = true;
      }
    }

    outFile.print(newLine);
    outFile.print("\n");
  }

  outFile.flush();
  inFile.close();
  outFile.close();

  if (pendingLeft) {
    SD.remove(filename);
    SD.rename(tmpName, filename);
    return true;
  }

  char finalName[28];
  toFinalName(filename, finalName, sizeof(finalName));

  SD.remove(finalName);

  if (!SD.rename(tmpName, finalName)) {
    SD.remove(filename);
    SD.rename(tmpName, filename);
    return true;
  }

  SD.remove(filename);
  return false;
}

/* --------------------------------------------------------------------------
 *  Final filename builder: /YYYYMMDD.CSV
 * -------------------------------------------------------------------------- */

void getFilename(const RtcDateTime& ts) {
  int y = ts.Year();
  int m = ts.Month();
  int d = ts.Day();

  csvFilename[0]  = '/';
  csvFilename[1]  = '2';
  csvFilename[2]  = '0';
  csvFilename[3]  = (y - 2000) / 10 + '0';
  csvFilename[4]  = y % 10 + '0';
  csvFilename[5]  = m / 10 + '0';
  csvFilename[6]  = m % 10 + '0';
  csvFilename[7]  = d / 10 + '0';
  csvFilename[8]  = d % 10 + '0';
  csvFilename[9]  = '.';
  csvFilename[10] = 'C';
  csvFilename[11] = 'S';
  csvFilename[12] = 'V';
}
