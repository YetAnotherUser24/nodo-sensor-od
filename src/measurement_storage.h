/**
 * @file measurement_storage.h
 * @brief Measurement acquisition, EEPROM buffering, SD batching, and upload helpers.
 *
 * This module handles:
 *  - Sampling sensor and temperature.
 *  - Preparing compact EEPROM records.
 *  - Dumping EEPROM pages to SD as CSV.
 *  - Processing pending CSV files and converting to JSON.
 *  - Uploading telemetry to ThingsBoard (batch and non-batch).
 */

#pragma once
#include <Arduino.h>
#include <RtcDS3231.h>

/**
 * @brief Marks a CSV line as sent (updates last field to "1").
 *
 * @param line Original CSV line.
 * @return Updated line with sent-flag = 1.
 */
String markLineSent(const String& line);

/**
 * @brief Returns number of free pages in external I2C EEPROM.
 */
uint16_t getEEPROMFreePages();

/**
 * @brief Clears external I2C EEPROM header and resets page write pointer.
 */
void clearEEPROM();

/**
 * @brief Logs EEPROM storage status or dumps an unpacked page record.
 * @param pageIndex Page number to peek (0 to EEPROM_MAX_PAGES - EEPROM_HEADER_SIZE - 1), or -1 for status.
 */
void peekEEPROM(int16_t pageIndex = -1);

/**
 * @brief Returns available SD storage in megabytes.
 */
uint32_t getSDFreeMB();

/**
 * @brief Normalizes module configuration (disallows invalid timings).
 */
void normalizeConfig();

/**
 * @brief Performs full measurement cycle and stores result in EEPROM buffer.
 *
 * Includes:
 *  - Temperature reading from RTC
 *  - Sensor sampling via sensor engine
 *  - Status flags
 *  - EEPROM page packing
 */
void takeMeasurement();

/**
 * @brief Sends all pending CSV files from SD to ThingsBoard.
 */
void sendSDPendingToThingsboard();

/**
 * @brief Sends all unsent EEPROM pages to ThingsBoard.
 */
void sendEEPROMToThingsboard();

/**
 * @brief Dumps all marked EEPROM pages to SD in CSV format.
 *
 * @return 1 on success, 0 on failure.
 */
uint8_t dumpEEPROM();

/**
 * @brief Builds a filename of form /YYYYMMDD.CSV.
 */
void getFilename(const RtcDateTime& ts);

/**
 * @brief Writes one EEPROM page (index n) into the appropriate CSV file.
 *
 * @param n Page index relative to data section (not absolute address).
 */
uint8_t writeEEPROMLine(uint16_t n);

/**
 * @brief Marks the EEPROM page that corresponds to the given timestamp as sent.
 */
bool markEepromSentByTimestamp(uint32_t secondsSince2000);

/**
 * @brief Converts one CSV line to ThingsBoard JSON payload.
 *
 * @param line Input CSV.
 * @param jsonOut Generated JSON string.
 * @param outSeconds2000 Extracted timestamp.
 * @return true if line was pending (sent=0), false otherwise.
 */
bool csvLineToJson(const String& line, String& jsonOut, uint32_t& outSeconds2000);

/**
 * @brief Processes CSV file: sends unsent lines and rewrites the file.
 *
 * @return true if file still has pending lines, false if fully cleared.
 */
bool processCsvFile(const char* filename);