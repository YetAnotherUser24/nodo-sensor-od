/**
 * @file n_logger_config.h
 * @brief Central type and constant definitions for n-logger multi-sensor platform.
 *
 * This replaces duplicated structs, status bits, and EEPROM constants
 * previously scattered across multiple source files.
 *
 * All defines use #ifndef guards to allow per-build overrides via
 * PlatformIO build_flags.
 */

#pragma once
#include <Arduino.h>

/* --------------------------------------------------------------------------
 *  Device Identity
 * -------------------------------------------------------------------------- */

#ifndef N_LOGGER_DEVICE_NAME
#define N_LOGGER_DEVICE_NAME "n-logger"
#endif

#ifndef N_LOGGER_BLE_NAME
#define N_LOGGER_BLE_NAME "n-logger-BLE"
#endif

#ifndef CODE_VERSION
#define CODE_VERSION 0x02
#endif

/* --------------------------------------------------------------------------
 *  Timestamp Epoch
 * -------------------------------------------------------------------------- */

#ifndef UNIX_OFFSET_2000
#define UNIX_OFFSET_2000 946684800UL
#endif

#ifndef LOCAL_TZ_OFFSET_SEC
#define LOCAL_TZ_OFFSET_SEC (5 * 3600)
#endif

/* --------------------------------------------------------------------------
 *  EEPROM Geometry (always included)
 * -------------------------------------------------------------------------- */

#ifndef EEPROM_ADDRESS
#define EEPROM_ADDRESS 0x50
#endif

#ifndef EEPROM_PAGE_SIZE
#define EEPROM_PAGE_SIZE 32
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

#define FLAG_NOT_SENT 0x00
#define FLAG_SENT     0x01

/* --------------------------------------------------------------------------
 *  Sensor Configuration — Descriptive Sensor Flags
 *
 *  Enable sensors by defining one or more of these flags at build time:
 *    -DSENSOR_MAXBOTIX_UART     = MaxBotix MB12xx ultrasonic distance (UART)
 *    -DSENSOR_WATER_TEMP        = Water surface temperature (I2C)
 *    -DSENSOR_RADAR             = Radar distance sensor (Modbus RTU, direct UART)
 *    -DSENSOR_SOIL_3TIP         = 3-probe soil moisture/temperature (Modbus RTU, RS-485)
 *    -DSENSOR_SOIL_5TIP         = 5-probe soil comprehensive sensor (Modbus RTU, RS-485)
 *    -DSENSOR_BATTERY_ADC       = Battery voltage via ADC pin
 *
 *  NOTE: SENSOR_RADAR, SENSOR_SOIL_3TIP, and SENSOR_SOIL_5TIP share UART pins
 *  (GPIO 34/32) and power pin (GPIO 19). They are mutually exclusive — do NOT
 *  enable more than one in the same build.
 *
 *  Each sensor defines its own pins, number of readings, and warmup time.
 *  Multiple compatible sensors can be active simultaneously.
 *  MAX_SENSOR_READINGS is auto-computed from all enabled sensors.
 *
 *  To override a pin at build time, define it before this header is included
 *  (e.g. -DMAXBOTIX_RX_PIN=22 in platformio.ini build_flags).
 * -------------------------------------------------------------------------- */

// Radar, 3-probe soil, and 5-probe soil sensors share UART pins (GPIO 34/32) and
// power pin (GPIO 19). At most one of these may be enabled in a single build.
#if defined(SENSOR_RADAR) && (defined(SENSOR_SOIL_3TIP) || defined(SENSOR_SOIL_5TIP))
#error "SENSOR_RADAR, SENSOR_SOIL_3TIP, and SENSOR_SOIL_5TIP share hardware pins and are mutually exclusive."
#endif
#if defined(SENSOR_SOIL_3TIP) && defined(SENSOR_SOIL_5TIP)
#error "SENSOR_SOIL_3TIP and SENSOR_SOIL_5TIP share hardware pins and are mutually exclusive."
#endif

// --- MaxBotix MB12xx ultrasonic sensor over UART ---
#ifdef SENSOR_MAXBOTIX_UART
  #define MAXBOTIX_RX_PIN 23
  #define MAXBOTIX_TX_PIN 19
  #define MAXBOTIX_PWR_PIN 18
  #define MAXBOTIX_NUM_READINGS 5
  #define MAXBOTIX_WARMUP_MS 175
  #define MAXBOTIX_SAMPLE_TIMEOUT_MS 500
  #define SENSOR_NAME "maxbotix-uart"
#else
  #define MAXBOTIX_NUM_READINGS 0
#endif

// --- Water Surface Temperature sensor (I2C) ---
#ifdef SENSOR_WATER_TEMP
  #define WATER_TEMP_ADDR 0x44
  #define WATER_TEMP_NUM_READINGS 3
  #define SENSOR_NAME "water-temp"
#else
  #define WATER_TEMP_NUM_READINGS 0
#endif

// --- Radar distance sensor (Modbus RTU over direct UART) ---
#ifdef SENSOR_RADAR
  #define RADAR_RX_PIN 34
  #define RADAR_TX_PIN 32
  #define RADAR_PWR_PIN 19
  #define RADAR_BAUD 9600
  #define RADAR_SLAVE_ID 1
  #define RADAR_NUM_READINGS 5
  #define RADAR_MODBUS_REGISTER 0x0001
  #define RADAR_WARMUP_MS 200
  #define RADAR_SAMPLE_TIMEOUT_MS 1500
  #define SENSOR_NAME "radar"
#else
  #define RADAR_NUM_READINGS 0
#endif

// --- 3-Probe Soil Moisture/Temperature sensor (Modbus RTU over RS-485) ---
#ifdef SENSOR_SOIL_3TIP
  #define SOIL_RX_PIN 34
  #define SOIL_TX_PIN 32
  #define SOIL_PWR_PIN 19
  #define SOIL_DE_RE_PIN 0
  #define SOIL_BAUD 4800
  #define SOIL_SLAVE_ID 1
  #define SOIL_NUM_READINGS 2
  #define SOIL_MODBUS_READ_COUNT 2
  #define SOIL_MODBUS_START_ADDR 0x0000
  #define SOIL_WARMUP_MS 200
  #define SENSOR_NAME "soil-3probe"
#else
  #define SOIL_NUM_READINGS 0
#endif

// --- 5-Probe Soil Comprehensive sensor (Modbus RTU over RS-485) ---
// Measures moisture, temperature, conductivity (EC), pH, N, P, K.
// Reads 7 holding registers starting at 0x0000.
#ifdef SENSOR_SOIL_5TIP
  #define SOIL5_RX_PIN 34
  #define SOIL5_TX_PIN 32
  #define SOIL5_PWR_PIN 19
  #define SOIL5_DE_RE_PIN 0
  #define SOIL5_BAUD 4800
  #define SOIL5_SLAVE_ID 1
  #define SOIL5_SAMPLE_COUNT 2
  #define SOIL5_MODBUS_READ_COUNT 7
  #define SOIL5_MODBUS_START_ADDR 0x0000
  #define SOIL5_WARMUP_MS 200
  #define SOIL5_NUM_READINGS (SOIL5_SAMPLE_COUNT * SOIL5_MODBUS_READ_COUNT)
  #define SENSOR_NAME "soil-5probe"
#else
  #define SOIL5_NUM_READINGS 0
#endif

// --- Battery voltage via ADC ---
#ifdef SENSOR_BATTERY_ADC
  #define BATTERY_ADC_PIN 35
  #define BATTERY_NUM_READINGS 1
  #define BATTERY_ADC_MULTIPLIER 2
  #define SENSOR_NAME "battery-adc"
#else
  #define BATTERY_NUM_READINGS 0
#endif

// --- Compute total measurement slots ---
#ifndef MAX_SENSOR_READINGS
#define MAX_SENSOR_READINGS (MAXBOTIX_NUM_READINGS + WATER_TEMP_NUM_READINGS + RADAR_NUM_READINGS + BATTERY_NUM_READINGS + SOIL_NUM_READINGS + SOIL5_NUM_READINGS)
#endif

// --- Fallback banner name when no sensor is defined ---
#ifndef SENSOR_NAME
#define SENSOR_NAME "none"
#endif

/* --------------------------------------------------------------------------
 *  Runtime Module Scheduling Configuration
 * -------------------------------------------------------------------------- */

/**
 * @brief Runtime configuration for module scheduling.
 *
 * Stored in EEPROM at offset 0, sizeof(ModuleConfig) bytes.
 */
struct ModuleConfig {
  int measure_time;
  int bluetooth_time;
  int GPRS_time;
};

/* --------------------------------------------------------------------------
 *  Telemetry Payload Structure
 *
 *  Fields adapt to enabled sensors:
 *  - temperature: always from RTC
 *  - battery: populated by SENSOR_BATTERY_ADC if enabled
 *  - sensorReadings[]: sized to sum of all enabled sensor readings
 * -------------------------------------------------------------------------- */

/**
 * @brief Main telemetry payload structure sent to the server.
 *
 * Field layout matches EEPROM page format and CSV column order.
 */
struct TelemetryPayload {
  int16_t  freeStorage;
  int16_t  temperature;
  int16_t  sensorReadings[MAX_SENSOR_READINGS];
#ifdef SENSOR_BATTERY_ADC
  uint32_t battery;
#endif
  uint32_t status;
};

/* --------------------------------------------------------------------------
 *  Status Bit Definitions
 * -------------------------------------------------------------------------- */

#define STATUS_INTERNET_BIT           0
#define STATUS_FULL_MEMORY_BIT        1
#define STATUS_CORRUPTED_MEMORY_BIT   2
#define STATUS_CODE_ERROR_BIT         3
#define STATUS_WATER_LEVEL_SENSOR_BIT 16
#define STATUS_SD_CARD_BIT            17
#define STATUS_TEMPERATURE_BIT        18
#define STATUS_GRPS_BIT               19
#define STATUS_RADAR_SENSOR_BIT       20
#define STATUS_SOIL_SENSOR_BIT        21

/* --------------------------------------------------------------------------
 *  Transmission Recovery
 * -------------------------------------------------------------------------- */

#ifndef MAX_FAILED_TRANSMISSIONS
#define MAX_FAILED_TRANSMISSIONS 3
#endif

/* --------------------------------------------------------------------------
 *  Pin Defaults (overridable per build)
 * -------------------------------------------------------------------------- */

#ifndef LED_PIN
#define LED_PIN 32
#endif

#ifndef WAKEUP_PIN
#define WAKEUP_PIN GPIO_NUM_39
#endif

#ifndef WAKEUP_PIN_BITMASK
#define WAKEUP_PIN_BITMASK (1ULL << WAKEUP_PIN)
#endif

/* --------------------------------------------------------------------------
 *  SD Card Pin Defaults
 * -------------------------------------------------------------------------- */

#ifndef SD_PIN_MOSI
#define SD_PIN_MOSI 15
#endif

#ifndef SD_PIN_MISO
#define SD_PIN_MISO 2
#endif

#ifndef SD_PIN_SCLK
#define SD_PIN_SCLK 14
#endif

#ifndef SD_PIN_CS
#define SD_PIN_CS 13
#endif

/* --------------------------------------------------------------------------
 *  Utility Function — Bit manipulation
 * -------------------------------------------------------------------------- */

/**
 * @brief Sets or clears a specific bit in a 32-bit value.
 *
 * @param value    Current value.
 * @param bitIndex Bit index [0..31] to modify.
 * @param setBit   True to set bit, false to clear.
 * @return Updated value with the bit modified.
 */
static inline uint32_t changeBit(uint32_t value, uint8_t bitIndex, bool setBit) {
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