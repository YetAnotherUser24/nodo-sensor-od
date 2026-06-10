/**
 * @file measurement_storage.cpp
 * @brief Measurement capture, EEPROM buffering, SD batching and telemetry upload.
 *
 * This module centralizes:
 *  - Sensor sampling and temperature/battery acquisition.
 *  - Compact EEPROM page storage.
 *  - Dumping buffered pages to SD.
 *  - Converting CSV records to JSON for ThingsBoard.
 *  - Uploading pending data from SD and EEPROM.
 *
 * Feature guards:
 *  - #ifdef SD_MODULE: SD card operations (dump, write, free space)
 *  - #ifdef GPRS_MODULE: MQTT/network operations (publish, upload)
 *
 * All functionality is preserved exactly as in original implementation.
 */

#include <Arduino.h>
#include <HardwareSerial.h>

#include "n_logger_config.h"
#include "measurement_storage.h"
#include "sensor_engine.h"
#include "eeprom_i2c.h"
#include "modem_mqtt.h"
#include "error_sensor.h"
#include "rtc_utils.h"

#include "config.h"
#include "log.h"

#include <ArduinoJson.h>
#include <EEPROM.h>
#include <Wire.h>

#ifdef SD_MODULE
#include <SdFat.h>
#endif

#ifdef GPRS_MODULE
#include <PubSubClient.h>
#endif

/* --------------------------------------------------------------------------
 *  External objects and configuration (declared in main.cpp)
 * -------------------------------------------------------------------------- */

#ifndef SerialMon
  #define SerialMon Serial
#endif

extern ModuleConfig moduleCfg;
extern TelemetryPayload telemetryPayload;

extern RTC_DATA_ATTR uint16_t eepromWritePage;
extern byte      eepromPageBuf[EEPROM_PAGE_SIZE];
extern bool      needFlushToSD;

#ifdef SD_MODULE
extern SdFat     SD;
extern SdFile    sdFile;
extern SPIClass  sdSPI;
#endif
extern char      csvFilename[24];
extern bool      fileOpen;

extern RtcDS3231<TwoWire> rtc;
extern RtcDateTime        rtcNow;
extern char               dateStr[20];

#ifdef GPRS_MODULE
extern PubSubClient mqtt;
#endif

// External helpers
extern void     error(int errorNum, int errpin);
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
#ifdef SD_MODULE
  if (!SD.begin(SdSpiConfig(SD_PIN_CS, SHARED_SPI, SD_SCK_MHZ(20), &sdSPI))) {
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
#else
  return 0;
#endif
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
  for (int i = 0; i < MAX_SENSOR_READINGS; i++)
    telemetryPayload.sensorReadings[i] = 0;
#ifdef SENSOR_BATTERY_ADC
  telemetryPayload.battery = 0;
#endif
  telemetryPayload.status  = 0x00000000;

  rtcNow = rtc.GetDateTime();
  uint32_t secondsSince2000 = rtcNow.TotalSeconds();

  /* Temperature from RTC (always included) */
  int16_t tempCenti = rtc.GetTemperature().AsCentiDegC();
  telemetryPayload.temperature = tempCenti;

  if (tempCenti < -4000 || tempCenti > 8500)
    telemetryPayload.status = changeBit(telemetryPayload.status, STATUS_TEMPERATURE_BIT, true);

  /* Sensor sampling via sensor engine (handles all enabled sensors) */
  sensorReadAll(telemetryPayload);

  /* Low-battery emergency config update (battery populated by sensor engine if SENSOR_BATTERY_ADC) */
#ifdef SENSOR_BATTERY_ADC
  if (telemetryPayload.battery < 3400) {
    moduleCfg.measure_time   = 60;
    moduleCfg.bluetooth_time = 1;
    moduleCfg.GPRS_time      = 1440;

    EEPROM.begin(sizeof(ModuleConfig));
    EEPROM.put(0, moduleCfg);
    EEPROM.commit();
    EEPROM.end();
  }
#endif

#ifdef DEBUG
  Logln("========== MEASUREMENT TAKEN ==========");
  Log("   Timestamp: ");
  Log(String(secondsSince2000).c_str());
  Log(" (");
  formatDateTime(rtcNow);
  Log(dateStr);
  Logln(" GMT)");
#ifdef SENSOR_BATTERY_ADC
  Log("   Battery: ");
  Log(String(telemetryPayload.battery).c_str());
  Logln(" mV");
#endif
  Log("   Temperature: ");
  Log(String(tempCenti / 100.0, 2).c_str());
  Logln(" C");
  Log("   Sensor Readings: [");
  for (int i = 0; i < MAX_SENSOR_READINGS; i++) {
    Log(String(telemetryPayload.sensorReadings[i]).c_str());
    if (i < MAX_SENSOR_READINGS - 1) Log(", ");
  }
  Logln("]");
  Log("   Status: 0x");
  Logln(String(telemetryPayload.status, HEX).c_str());
  Logln("==========================================");
#endif

  /* EEPROM page building: ts(4) + readings(N*2) + temp(2) + status(4) + pad + sent */
  for (int i = 0; i < 4; i++)
    eepromPageBuf[i] = ((byte*)&secondsSince2000)[i];

  for (int i = 0; i < MAX_SENSOR_READINGS * 2; i++)
    eepromPageBuf[i + 4] = ((byte*)telemetryPayload.sensorReadings)[i];

  for (int i = 0; i < 2; i++)
    eepromPageBuf[i + 4 + MAX_SENSOR_READINGS*2] = ((byte*)&tempCenti)[i];

  for (int i = 0; i < 4; i++)
    eepromPageBuf[i + 4 + MAX_SENSOR_READINGS*2 + 2] = ((byte*)&telemetryPayload.status)[i];

  for (int i = 4 + MAX_SENSOR_READINGS*2 + 2 + 4; i < EEPROM_PAGE_SIZE - 1; i++)
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
#ifdef GPRS_MODULE
  if (!mqttAvailable) {
    Logln("MQTT unavailable, skipping SD uploads.");
    return;
  }

  if (!mqtt.connected()) {
    if (!connectMQTT()) {
      Logln("MQTT connection lost, skipping SD uploads.");
      mqttAvailable = false;
      return;
    }
  }

#ifdef SD_MODULE
  SdFile dir, entry;
  if (!SD.begin(SdSpiConfig(SD_PIN_CS, SHARED_SPI, SD_SCK_MHZ(20), &sdSPI)))
    return;

  if (!dir.open("/")) return;

  char name[32];
  uint16_t stillPending = 0;

  while (entry.openNext(&dir, O_READ)) {
    if (!entry.isDir()) {
      entry.getName(name, sizeof(name));
      String fname = name;

      // Process CSV files that match /YYYYMMDD.CSV
      if (fname.endsWith(".CSV") && fname.startsWith("20")) {
        String full = String("/") + fname;
        bool pending = processCsvFile(full.c_str());
        if (pending) stillPending++;
      }
    }
    entry.close();
  }

  dir.close();
#endif // SD_MODULE

  // Final flush for batch (power optimized)
  for (int i = 0; i < 5; i++) {
    mqtt.loop();
    delay(50);
  }
#else
  (void)0;
#endif
}

void sendEEPROMToThingsboard() {
#ifdef GPRS_MODULE
  if (!mqttAvailable) {
    Logln("MQTT unavailable, skipping EEPROM uploads.");
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
      int16_t* readings = (int16_t*)&pageBuf[4];
      int16_t  temp  = *((int16_t*)&pageBuf[4 + MAX_SENSOR_READINGS*2]);
      uint32_t status = *((uint32_t*)&pageBuf[4 + MAX_SENSOR_READINGS*2 + 2]);

      uint64_t tsUnix =
        ((uint64_t)seconds2000 + UNIX_OFFSET_2000 + LOCAL_TZ_OFFSET_SEC) * 1000ULL;

      StaticJsonDocument<512> doc;
      doc["ts"] = tsUnix;

      JsonObject vals = doc.createNestedObject("values");

#ifdef SENSOR_BATTERY_ADC
      // Battery is the last sensor reading if enabled
      vals["battery"] = readings[MAX_SENSOR_READINGS - 1];
#endif
      vals["temperature"] = temp;
      vals["status"] = status;

      JsonArray arr = vals.createNestedArray("sensorReadings");
      for (int i = 0; i < MAX_SENSOR_READINGS; i++) arr.add(readings[i]);

      vals["eepromFreePages"] = getEEPROMFreePages();
      vals["sdFreeMB"]        = getSDFreeMB();

      String jsonOut;
      serializeJson(doc, jsonOut);

#ifdef DEBUG
      Logln("MQTT -> EEPROM Data:");
      Log("   Payload: ");
      Logln(jsonOut.c_str());
#endif

      if (!mqtt.connected()) {
        if (!connectMQTT()) {
          Logln("MQTT connection lost, stopping EEPROM uploads.");
          mqttAvailable = false;
          return;
        }
      }

      bool ok = mqtt.publish(MQTT_TOPIC, jsonOut.c_str());

      for (int i = 0; i < 3; i++) {
        mqtt.loop();
        delay(50);
      }

#ifdef DEBUG
      if (ok) {
        Logln("   Published successfully (queued)");
      } else {
        Logln("   Publish failed");
      }
#endif

      if (ok)
        i2c_eeprom_write_byte(EEPROM_ADDRESS, addr + EEPROM_SENT_FLAG_OFFSET, FLAG_SENT);
    }

    pageIndex++;
  }

  for (int i = 0; i < 5; i++) {
    mqtt.loop();
    delay(50);
  }
#else
  (void)0;
#endif
}

/* --------------------------------------------------------------------------
 *  dumpEEPROM(): writes all marked pages to SD
 * -------------------------------------------------------------------------- */

uint8_t dumpEEPROM() {
#ifdef SD_MODULE
  uint16_t i = 0, j = 0;
  bool keep = true;
  bool failure = false;

  digitalWrite(LED_PIN, HIGH);

  if (!SD.begin(SdSpiConfig(SD_PIN_CS, SHARED_SPI, SD_SCK_MHZ(20), &sdSPI))) {
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
#else
  return 1;
#endif
}

/* --------------------------------------------------------------------------
 *  CSV filename: /YYYYMMDD.CSV (no pending _P prefix)
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
  csvFilename[13] = '\0';
}

/**
 * @brief Writes one EEPROM row into the proper daily CSV file.
 *
 * CSV format: dateStr, seconds2000, reading[0..N-1], temperature, status, sent
 */
uint8_t writeEEPROMLine(uint16_t n) {
#ifdef SD_MODULE
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

  int16_t* readings = (int16_t*)&pageBuf[4];

  uint32_t status =
    ((uint32_t)readings[MAX_SENSOR_READINGS + 2] & 0xFFFF) |
    ((uint32_t)readings[MAX_SENSOR_READINGS + 3] << 16);

  uint8_t sentFlag = (pageBuf[EEPROM_SENT_FLAG_OFFSET] == FLAG_SENT);

  static uint32_t dayCache = 40;
  if (!fileOpen) {
    getFilename(ts);
    fileOpen = sdFile.open(csvFilename, O_CREAT | O_WRITE | O_APPEND);
    dayCache = ts.Day();
  } else if (dayCache != ts.Day()) {
    sdFile.close();
    getFilename(ts);
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

  for (uint16_t k = 0; k < MAX_SENSOR_READINGS + 2; k++) {
    sdFile.print(", ");
    sdFile.print(readings[k + 2]);
  }

  sdFile.print(", ");
  sdFile.print(status);

  sdFile.print(", ");
  sdFile.print(sentFlag);

  sdFile.println();
  sdFile.flush();
  return 1;
#else
  return 1;
#endif
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
 *
 *  CSV format: dateStr, seconds2000, reading[0..N-1], temperature, status, sent
 *  → JSON with sensorReadings array, battery (if SENSOR_BATTERY_ADC), temperature, status
 * -------------------------------------------------------------------------- */

bool csvLineToJson(const String& line, String& jsonOut, uint32_t& outSeconds2000) {
  int idx = 0, start = 0, field = 0;
  int values[MAX_SENSOR_READINGS + 2];
  uint32_t status = 0;
  int sent = 0;
  uint32_t seconds2000 = 0;

  while (true) {
    idx = line.indexOf(',', start);
    String token = (idx == -1) ? line.substring(start) : line.substring(start, idx);
    token.trim();

    if (field == 1)
      seconds2000 = token.toInt();
    else if (field >= 2 && field <= (1 + MAX_SENSOR_READINGS))
      values[field - 2] = token.toInt();
    else if (field == (2 + MAX_SENSOR_READINGS))
      values[MAX_SENSOR_READINGS] = token.toInt();
    else if (field == (3 + MAX_SENSOR_READINGS))
      status = token.toInt();
    else if (field == (4 + MAX_SENSOR_READINGS))
      sent = token.toInt();

    field++;
    if (idx == -1) break;
    start = idx + 1;
  }

  if (field < (5 + MAX_SENSOR_READINGS))
    return false;

  StaticJsonDocument<512> doc;
  uint64_t tsUnix =
    ((uint64_t)seconds2000 + UNIX_OFFSET_2000 + LOCAL_TZ_OFFSET_SEC) * 1000ULL;

  doc["ts"] = tsUnix;

  JsonObject vals = doc.createNestedObject("values");

#ifdef SENSOR_BATTERY_ADC
  // Battery is the last sensor reading if enabled
  vals["battery"] = values[MAX_SENSOR_READINGS - 1];
#endif
  vals["temperature"] = values[MAX_SENSOR_READINGS];
  vals["status"] = status;

  JsonArray arr = vals.createNestedArray("sensorReadings");
  for (int i = 0; i < MAX_SENSOR_READINGS; i++)
    arr.add(values[i]);

  jsonOut.clear();
  serializeJson(doc, jsonOut);

  outSeconds2000 = seconds2000;
  return (sent == 0);
}

/* --------------------------------------------------------------------------
 *  processCsvFile() — Reads CSV file, publishes unsent lines, rewrites in place
 * -------------------------------------------------------------------------- */

bool processCsvFile(const char* filename) {
#ifdef SD_MODULE
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

  // Replace original with updated file
  SD.remove(filename);
  SD.rename(tmpName, filename);

  return pendingLeft;
#else
  return true;
#endif
}