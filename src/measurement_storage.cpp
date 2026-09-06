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

#include "eeprom_i2c.h"
#include "error_sensor.h"
#include "measurement_storage.h"
#include "n_logger_config.h"
#include "wifi_http.h"
#include "esp_now_sender.h"
#include "rtc_utils.h"
#include "sensor_engine.h"

#include "config.h"
#include "log.h"

#include <ArduinoJson.h>
#include <EEPROM.h>
#include <Wire.h>

#ifdef SD_MODULE
#include <SdFat.h>
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
extern byte eepromPageBuf[EEPROM_PAGE_SIZE];
extern bool needFlushToSD;

#ifdef SD_MODULE
extern SdFat SD;
extern SdFile sdFile;
extern SPIClass sdSPI;
#endif
extern char csvFilename[24];
extern bool fileOpen;

extern RtcDS3231<TwoWire> rtc;
extern RtcDateTime rtcNow;
extern char dateStr[20];

extern bool wifiAvailable;

/* --------------------------------------------------------------------------
 *  SD / EEPROM Helpers (unchanged logic, documented)
 * -------------------------------------------------------------------------- */

String markLineSent(const String& line) {
  String s = line;

  while (s.length() && s[s.length() - 1] == ' ')
    s.remove(s.length() - 1);

  int lastComma = s.lastIndexOf(',');
  if (lastComma <= 0)
    return line;

  return s.substring(0, lastComma) + ", 1";
}

/**
 * @brief Count free EEPROM pages using bit-mapped header.
 */
uint16_t getEEPROMFreePages() {
  if (!isEepromAvailable()) {
    return 0;
  }

  uint8_t header[EEPROM_HEADER_SIZE];

  i2c_eeprom_read_buffer(EEPROM_ADDRESS, 0, header, EEPROM_HEADER_SIZE);

  uint16_t used = 0;
  for (uint16_t i = 0; i < EEPROM_HEADER_SIZE; i++) {
    uint8_t b = header[i];
    used += (b & 1) + ((b >> 1) & 1) + ((b >> 2) & 1) + ((b >> 3) & 1) + ((b >> 4) & 1) +
            ((b >> 5) & 1) + ((b >> 6) & 1) + ((b >> 7) & 1);
  }

  uint16_t totalPages = EEPROM_MAX_PAGES - EEPROM_HEADER_SIZE;
  return totalPages - used;
}

/**
 * @brief Clears external I2C EEPROM header and resets page write pointer.
 */
void clearEEPROM() {
  if (!isEepromAvailable()) return;
  resetEEPromHeader(EEPROM_ADDRESS);
  eepromWritePage = 0;
  needFlushToSD = false;
  Logln("EEPROM cleared.");
}

/**
 * @brief Logs EEPROM storage status or dumps an unpacked page record.
 */
void peekEEPROM(int16_t pageIndex) {
  if (!isEepromAvailable()) {
    Logln("No external EEPROM detected.");
    return;
  }
  uint16_t totalPages = EEPROM_MAX_PAGES - EEPROM_HEADER_SIZE;
  uint16_t freePages = getEEPROMFreePages();
  uint16_t usedPages = totalPages - freePages;

  if (pageIndex < 0) {
    Logf("--- EEPROM STORAGE (NextWrite: %u/%u | Used: %u | Free: %u) ---\n",
         eepromWritePage, totalPages, usedPages, freePages);

    if (usedPages == 0) {
      Logln("No active pages stored in EEPROM.");
      return;
    }

    uint8_t header[EEPROM_HEADER_SIZE * EEPROM_PAGE_SIZE];
    for (uint16_t h = 0; h < EEPROM_HEADER_SIZE; h++) {
      i2c_eeprom_read_buffer(EEPROM_ADDRESS, h * EEPROM_PAGE_SIZE,
                             header + (h * EEPROM_PAGE_SIZE), EEPROM_PAGE_SIZE);
    }

    byte pageBuf[EEPROM_PAGE_SIZE];
    for (uint16_t p = 0; p < totalPages; p++) {
      bool isUsed = (header[p / 8] >> (p % 8)) & 1;
      if (!isUsed)
        continue;

      uint16_t addr = (p + EEPROM_HEADER_SIZE) * EEPROM_PAGE_SIZE;
      i2c_eeprom_read_buffer(EEPROM_ADDRESS, addr, pageBuf, EEPROM_PAGE_SIZE);

      uint32_t seconds2000 = ((uint32_t*)pageBuf)[0];
      if (seconds2000 == 0 || seconds2000 == 0xFFFFFFFF) {
        Logf("[P%u] (Empty/Corrupt)\n", p);
        continue;
      }

      RtcDateTime ts(seconds2000);
      formatDateTime(ts);

      int16_t* readings = (int16_t*)&pageBuf[4];
      int16_t tempRaw = readings[MAX_SENSOR_READINGS];
      float tempC = tempRaw / 100.0f;
      uint8_t sentFlag = pageBuf[EEPROM_SENT_FLAG_OFFSET];

      char sensStr[64] = "";
      int offset = 0;
      for (int s = 0; s < MAX_SENSOR_READINGS; s++) {
        if (s > 0) {
          offset += snprintf(sensStr + offset, sizeof(sensStr) - offset, ", ");
        }
        offset += snprintf(sensStr + offset, sizeof(sensStr) - offset, "%d", readings[s]);
      }

      Logf("[P%u] %s | S:[%s] | %.2fC | Sent:%s\n", p, dateStr, sensStr, tempC,
           (sentFlag == FLAG_SENT) ? "YES" : "NO");
    }
    return;
  }

  if (pageIndex >= (int16_t)totalPages) {
    Logf("[EEPROM] Invalid page %d (max index is %d)\n", pageIndex, totalPages - 1);
    return;
  }

  byte pageBuf[EEPROM_PAGE_SIZE];
  uint16_t addr = (pageIndex + EEPROM_HEADER_SIZE) * EEPROM_PAGE_SIZE;
  i2c_eeprom_read_buffer(EEPROM_ADDRESS, addr, pageBuf, EEPROM_PAGE_SIZE);

  uint32_t seconds2000 = ((uint32_t*)pageBuf)[0];
  if (seconds2000 == 0 || seconds2000 == 0xFFFFFFFF) {
    Logf("--- EEPROM PAGE %d: EMPTY / UNWRITTEN ---\n", pageIndex);
    return;
  }

  RtcDateTime ts(seconds2000);
  formatDateTime(ts);

  int16_t* readings = (int16_t*)&pageBuf[4];
  int16_t tempRaw = readings[MAX_SENSOR_READINGS];
  float tempC = tempRaw / 100.0f;

  uint32_t status = ((uint32_t)(uint16_t)readings[MAX_SENSOR_READINGS + 1]) |
                    (((uint32_t)(uint16_t)readings[MAX_SENSOR_READINGS + 2]) << 16);
  uint8_t sentFlag = pageBuf[EEPROM_SENT_FLAG_OFFSET];

  Logf("--- EEPROM PAGE %d ---\n", pageIndex);
  Logf("Time: %s (ts=%u)\n", dateStr, seconds2000);
  for (int i = 0; i < MAX_SENSOR_READINGS; i++) {
    Logf("Sensor[%d]: %d\n", i, readings[i]);
  }
  Logf("Temp: %.2f C | Status: 0x%08X | Sent: %s\n", tempC, status,
       (sentFlag == FLAG_SENT) ? "YES" : "NO");
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
  if (!vol)
    return 0;

  uint64_t totalBytes = (uint64_t)vol->clusterCount() * (uint64_t)vol->sectorsPerCluster() * 512ULL;

  SdFile dir, entry;
  uint64_t usedBytes = 0;

  if (dir.open("/")) {
    while (entry.openNext(&dir, O_READ)) {
      if (!entry.isDir())
        usedBytes += entry.fileSize();
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
#if defined(SENSOR_BATTERY_ADC) || defined(SENSOR_INA226) || defined(SENSOR_VIRTUAL)
  telemetryPayload.battery = 0;
#endif
  telemetryPayload.status = 0x00000000;

  rtcNow = getRtcDateTime();
  if (isRtcAvailable()) {
    for (int retry = 0; retry < 3 && rtcNow.Year() < 2020; retry++) {
      delay(20);
      rtcNow = rtc.GetDateTime();
    }
  }
  uint32_t secondsSince2000 = rtcNow.TotalSeconds();

  /* Temperature from RTC (always included if RTC present) */
  int16_t tempCenti = 0;
  if (isRtcAvailable()) {
    tempCenti = rtc.GetTemperature().AsCentiDegC();
    if (tempCenti == 0 && rtcNow.Year() >= 2020) {
      delay(10);
      tempCenti = rtc.GetTemperature().AsCentiDegC();
    }
    if (tempCenti < -4000 || tempCenti > 8500)
      telemetryPayload.status = changeBit(telemetryPayload.status, STATUS_TEMPERATURE_BIT, true);
  } else {
    tempCenti = 2500; // 25.0 C ambient fallback for bare ESP32 / virtual mode
  }
  telemetryPayload.temperature = tempCenti;

  /* Sensor sampling via sensor engine (handles all enabled sensors) */
  sensorReadAll(telemetryPayload);

#if !defined(DISABLE_ESPNOW)
  /* Broadcast real-time DO packet via ESP-NOW to ODrive Motor Controller */
  static uint32_t espNowSeqCounter = 0;
  OdTelemetryPacket espNowPkt;
  espNowPkt.magic = AQUACONTROL_PACKET_MAGIC;
  espNowPkt.sequence = ++espNowSeqCounter;
  espNowPkt.timestamp_sec = secondsSince2000;
  espNowPkt.water_temperature = telemetryPayload.sensorReadings[0] / 100.0f;  // centi-C to C
  espNowPkt.oxygen_saturation = telemetryPayload.sensorReadings[1] / 10.0f;   // deci-% to %
  espNowPkt.dissolved_oxygen = telemetryPayload.sensorReadings[2] / 1000.0f; // milli-mg/L to mg/L
  espNowPkt.battery_voltage = telemetryPayload.battery / 1000.0f;             // mV to V
  espNowPkt.status_flags = telemetryPayload.status;
  espNowBroadcastDO(espNowPkt);
#endif


  /* Low-battery emergency config update (battery populated by sensor engine) */
#if defined(SENSOR_BATTERY_ADC) || defined(SENSOR_INA226)
  if (telemetryPayload.battery < 3400) {
    moduleCfg.measure_time = 60;
    moduleCfg.bluetooth_time = 1;
    moduleCfg.GPRS_time = 1440;

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
#if defined(SENSOR_BATTERY_ADC) || defined(SENSOR_INA226)
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
    if (i < MAX_SENSOR_READINGS - 1)
      Log(", ");
  }
  Logln("]");
  Log("   Status: 0x");
  Logln(String(telemetryPayload.status, HEX).c_str());
  Logln("==========================================");
#endif

  /* EEPROM storage only if external I2C EEPROM hardware is present */
  if (isEepromAvailable()) {
    /* EEPROM page building: ts(4) + readings(N*2) + temp(2) + status(4) + pad + sent */
    for (int i = 0; i < 4; i++)
      eepromPageBuf[i] = ((byte*)&secondsSince2000)[i];

    for (int i = 0; i < MAX_SENSOR_READINGS * 2; i++)
      eepromPageBuf[i + 4] = ((byte*)telemetryPayload.sensorReadings)[i];

    for (int i = 0; i < 2; i++)
      eepromPageBuf[i + 4 + MAX_SENSOR_READINGS * 2] = ((byte*)&tempCenti)[i];

    for (int i = 0; i < 4; i++)
      eepromPageBuf[i + 4 + MAX_SENSOR_READINGS * 2 + 2] = ((byte*)&telemetryPayload.status)[i];

    for (int i = 4 + MAX_SENSOR_READINGS * 2 + 2 + 4; i < EEPROM_PAGE_SIZE - 1; i++)
      eepromPageBuf[i] = 0;

    eepromPageBuf[EEPROM_SENT_FLAG_OFFSET] = FLAG_NOT_SENT;

    unsigned int dataAddr = (eepromWritePage + EEPROM_HEADER_SIZE) * EEPROM_PAGE_SIZE;

    i2c_eeprom_write_page(EEPROM_ADDRESS, dataAddr, eepromPageBuf, EEPROM_PAGE_SIZE);
    delay(50);

    /* Update header bit */
    byte headerByte = i2c_eeprom_read_byte(EEPROM_ADDRESS, (uint16_t)eepromWritePage / 8);

    bitWrite(headerByte, eepromWritePage % 8, 1);
    i2c_eeprom_write_byte(EEPROM_ADDRESS, (uint16_t)eepromWritePage / 8, headerByte);

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

  error(1, LED_PIN);
}

/* --------------------------------------------------------------------------
 *  Upload routines: SD pending + EEPROM pending (via WiFi HTTPS)
 * -------------------------------------------------------------------------- */

void sendSDPending() {
#if defined(WIFI_MODULE) && defined(SD_MODULE)
  if (!wifiAvailable) {
    Logln("WiFi unavailable, skipping SD uploads.");
    return;
  }

  SdFile dir, entry;
  if (!SD.begin(SdSpiConfig(SD_PIN_CS, SHARED_SPI, SD_SCK_MHZ(20), &sdSPI)))
    return;

  if (!dir.open("/"))
    return;

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
        if (pending)
          stillPending++;
      }
    }
    entry.close();
  }

  dir.close();
#else
  (void)0;
#endif
}

uint16_t sendEEPROMPending() {
#ifdef WIFI_MODULE
  if (!wifiAvailable || !isEepromAvailable()) {
    if (!wifiAvailable) {
      Logln("WiFi unavailable, skipping EEPROM uploads.");
    }
    return 0;
  }

  uint16_t sentCount = 0;
  uint16_t pageIndex = 0;
  byte pageBuf[EEPROM_PAGE_SIZE];

  while (pageIndex < (EEPROM_MAX_PAGES - EEPROM_HEADER_SIZE)) {
    unsigned int addr = (pageIndex + EEPROM_HEADER_SIZE) * EEPROM_PAGE_SIZE;

    i2c_eeprom_read_buffer(EEPROM_ADDRESS, addr, pageBuf, EEPROM_PAGE_SIZE);

    if (pageBuf[EEPROM_SENT_FLAG_OFFSET] == FLAG_NOT_SENT) {
      uint32_t seconds2000 = *((uint32_t*)&pageBuf[0]);
      int16_t* readings = (int16_t*)&pageBuf[4];
      int16_t temp = *((int16_t*)&pageBuf[4 + MAX_SENSOR_READINGS * 2]);
      uint32_t status = *((uint32_t*)&pageBuf[4 + MAX_SENSOR_READINGS * 2 + 2]);

      bool ok = publishToAquaControl(seconds2000, readings, temp, status);

#ifdef DEBUG
      Logf("   EEPROM page %u: %s\n", pageIndex, ok ? "sent" : "failed");
#endif

      if (ok) {
        i2c_eeprom_write_byte(EEPROM_ADDRESS, addr + EEPROM_SENT_FLAG_OFFSET, FLAG_SENT);
        sentCount++;
      }
    }

    pageIndex++;
  }
  return sentCount;
#else
  return 0;
#endif
}


/* --------------------------------------------------------------------------
 *  dumpEEPROM(): writes all marked pages to SD
 * -------------------------------------------------------------------------- */

uint8_t dumpEEPROM() {
#ifdef SD_MODULE
  if (!isEepromAvailable()) {
    return 1; // Nothing to dump
  }

  uint16_t i = 0, j = 0;
  bool keep = true;
  bool failure = false;
  bool wroteAny = false;
  const uint16_t headerBytes = EEPROM_HEADER_SIZE * EEPROM_PAGE_SIZE;

  // Power ON sensor / I2C bus VCC pin before communicating with EEPROM
#if defined(MAXBOTIX_PWR_PIN) && MAXBOTIX_PWR_PIN != -1
  setPinActive(MAXBOTIX_PWR_PIN, MAXBOTIX_PWR_ACTIVE_HIGH);
  delay(20);
#elif defined(WATER_TEMP_PWR_PIN) && WATER_TEMP_PWR_PIN != -1
  setPinActive(WATER_TEMP_PWR_PIN, WATER_TEMP_PWR_ACTIVE_HIGH);
  delay(20);
#endif

#ifdef DEBUG
  Serial.println(F("[dumpEEPROM] start"));
  Serial.println(F("[dumpEEPROM] SD.begin()..."));
#endif

  if (!SD.begin(SdSpiConfig(SD_PIN_CS, SHARED_SPI, SD_SCK_MHZ(20), &sdSPI))) {
#ifdef DEBUG
    Serial.println(F("[dumpEEPROM] SD.begin() failed"));
#endif
    telemetryPayload.status = changeBit(telemetryPayload.status, STATUS_SD_CARD_BIT, true);

    return 0;
  }

#ifdef DEBUG
  Serial.println(F("[dumpEEPROM] SD.begin() ok"));
#endif

  while (keep && i < headerBytes) {
    byte headerByte = i2c_eeprom_read_byte(EEPROM_ADDRESS, i);

    for (j = 0; j < 8; j++) {
      if ((headerByte >> j) & 1) {
        uint8_t res = writeEEPROMLine(i * 8 + j);
        if (res == 1) {
          wroteAny = true;
        } else if (res == 2) {
          // Reached end of valid EEPROM data (empty page/zero timestamp) -> stop dump
          // cleanly
          keep = false;
          break;
        } else {
          // res == 0: SD card file open or write error
          failure = true;
          keep = false;
          break;
        }
      } else {
#ifdef DEBUG
        Serial.println(F("[dumpEEPROM] no more pending pages"));
#endif
        keep = false;
        break;
      }
    }

    i++;
  }

#ifdef DEBUG
  if (keep && i >= headerBytes) {
    Serial.println(F("[dumpEEPROM] reached end of header bitmap"));
  }
#endif

  if (!failure && wroteAny) {
    error(1, LED_PIN);
  }

  if (fileOpen) {
    sdFile.close();
    fileOpen = false;
  }

#ifdef DEBUG
  Serial.println(F("[dumpEEPROM] end"));
#endif

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

  csvFilename[0] = '/';
  csvFilename[1] = '2';
  csvFilename[2] = '0';
  csvFilename[3] = (y - 2000) / 10 + '0';
  csvFilename[4] = y % 10 + '0';
  csvFilename[5] = m / 10 + '0';
  csvFilename[6] = m % 10 + '0';
  csvFilename[7] = d / 10 + '0';
  csvFilename[8] = d % 10 + '0';
  csvFilename[9] = '.';
  csvFilename[10] = 'C';
  csvFilename[11] = 'S';
  csvFilename[12] = 'V';
  csvFilename[13] = '\0';
}

#ifdef SD_MODULE
static void writeCsvHeader(SdFile& file) {
  file.print(F("datetime, seconds_since_2000"));
  file.print(F(", water_temp_centi, do_sat_deci_pct, do_milli_mg_l, param3_centi, param4_centi"));
#if defined(SENSOR_BATTERY_ADC) || defined(SENSOR_INA226)
  file.print(F(", battery_mv"));
#endif
  file.println(F(", rtc_temp_centi, status, sent"));
  file.flush();
}
#endif

/**
 * @brief Writes one EEPROM row into the proper daily CSV file.
 *
 * CSV format: dateStr, seconds2000, reading[0..N-1], temperature, status, sent
 */
uint8_t writeEEPROMLine(uint16_t n) {
#ifdef SD_MODULE
  byte pageBuf[EEPROM_PAGE_SIZE];
  RtcDateTime ts;
  /*
  #ifdef DEBUG
    Serial.print(F("[writeEEPROMLine] page "));
    Serial.println(n);
  #endif
  */
  i2c_eeprom_read_buffer(EEPROM_ADDRESS, (n + EEPROM_HEADER_SIZE) * EEPROM_PAGE_SIZE, pageBuf,
                         EEPROM_PAGE_SIZE);

  uint32_t seconds2000 = ((uint32_t*)pageBuf)[0];

  RtcDateTime now = rtc.GetDateTime();
  uint32_t nowSeconds2000 = now.TotalSeconds();

  if (seconds2000 == 0 || seconds2000 > (nowSeconds2000 + 86400UL)) {
#ifdef DEBUG
    Serial.print(F("[writeEEPROMLine] invalid ts raw="));
    Serial.print(seconds2000);
    Serial.println(F(" -> stop dump"));
#endif
    return 2; // 2 = end of valid EEPROM records (not a failure)
  }

  ts = RtcDateTime(seconds2000);

  int16_t* readings = (int16_t*)&pageBuf[4];

  uint32_t status = ((uint32_t)(uint16_t)readings[MAX_SENSOR_READINGS + 1]) |
                    (((uint32_t)(uint16_t)readings[MAX_SENSOR_READINGS + 2]) << 16);

  uint8_t sentFlag = (pageBuf[EEPROM_SENT_FLAG_OFFSET] == FLAG_SENT);

  static uint32_t dayCache = 40;
  if (!fileOpen) {
    getFilename(ts);
#ifdef DEBUG
    Serial.print(F("[writeEEPROMLine] ts = "));
    formatDateTime(ts);
    Serial.println(dateStr);
    Serial.print(F("[writeEEPROMLine] opening "));
    Serial.println(csvFilename);
#endif
    fileOpen = sdFile.open(csvFilename, O_CREAT | O_WRITE | O_APPEND);
    if (fileOpen && sdFile.fileSize() == 0) {
      writeCsvHeader(sdFile);
    }
    dayCache = ts.Day();
  } else if (dayCache != ts.Day()) {
    getFilename(ts);
#ifdef DEBUG
    Serial.print(F("[writeEEPROMLine] ts = "));
    formatDateTime(ts);
    Serial.println(dateStr);
    Serial.print(F("[writeEEPROMLine] day change, reopening "));
    Serial.println(csvFilename);
#endif
    sdFile.close();
    fileOpen = sdFile.open(csvFilename, O_CREAT | O_WRITE | O_APPEND);
    if (fileOpen && sdFile.fileSize() == 0) {
      writeCsvHeader(sdFile);
    }
    dayCache = ts.Day();
  }

  if (!fileOpen) {
#ifdef DEBUG
    Serial.println(F("[writeEEPROMLine] file open failed"));
#endif
    return 0;
  }

  formatDateTime(ts);
  sdFile.print(dateStr);
  sdFile.print(", ");
  sdFile.print(seconds2000);

  for (uint16_t k = 0; k < MAX_SENSOR_READINGS; k++) {
    sdFile.print(", ");
    sdFile.print(readings[k]);
  }

  sdFile.print(", ");
  sdFile.print(readings[MAX_SENSOR_READINGS]);

  sdFile.print(", ");
  sdFile.print(status);

  sdFile.print(", ");
  sdFile.print(sentFlag);

  sdFile.println();
  sdFile.flush();
  return 1;
#else
  return 0;
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
        i2c_eeprom_write_byte(EEPROM_ADDRESS, addr + EEPROM_SENT_FLAG_OFFSET, FLAG_SENT);
      }
      return true;
    }
  }
  return false;
}

/* --------------------------------------------------------------------------
 *  csvLineParse() — CSV field extraction
 *
 *  CSV format: dateStr, seconds2000, reading[0..N-1], temperature, status, sent
 *  Extracts raw fields for publishToAquaControl().
 * -------------------------------------------------------------------------- */

bool csvLineParse(const String& line, uint32_t& outSeconds2000,
                  int16_t* outReadings, int16_t& outTemp, uint32_t& outStatus) {
  if (line.length() < 10)
    return false;

  // Skip CSV header line (starts with non-digit character)
  if (!isdigit(line.charAt(0)))
    return false;

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
    if (idx == -1)
      break;
    start = idx + 1;
  }

  if (field < (5 + MAX_SENSOR_READINGS))
    return false;

  outSeconds2000 = seconds2000;
  for (int i = 0; i < MAX_SENSOR_READINGS; i++)
    outReadings[i] = (int16_t)values[i];
  outTemp = (int16_t)values[MAX_SENSOR_READINGS];
  outStatus = status;

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

    if (c == '\r')
      continue;
    if (c == '\n') {
      uint32_t sec2000 = 0;
      int16_t csvReadings[MAX_SENSOR_READINGS];
      int16_t csvTemp = 0;
      uint32_t csvStatus = 0;

      String newLine = line;
      bool wasPending = csvLineParse(line, sec2000, csvReadings, csvTemp, csvStatus);

      if (wasPending) {
        if (publishToAquaControl(sec2000, csvReadings, csvTemp, csvStatus)) {
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
    uint32_t sec2000 = 0;
    int16_t csvReadings[MAX_SENSOR_READINGS];
    int16_t csvTemp = 0;
    uint32_t csvStatus = 0;
    String newLine = line;

    bool wasPending = csvLineParse(line, sec2000, csvReadings, csvTemp, csvStatus);

    if (wasPending) {
      if (publishToAquaControl(sec2000, csvReadings, csvTemp, csvStatus)) {
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