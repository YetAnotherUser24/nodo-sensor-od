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
#include "board.h"
#include <Arduino.h>

/* --------------------------------------------------------------------------
 *  Device Identity
 * -------------------------------------------------------------------------- */

#ifndef N_LOGGER_DEVICE_NAME
#define N_LOGGER_DEVICE_NAME "od-logger"
#endif

#ifndef N_LOGGER_BLE_NAME
#define N_LOGGER_BLE_NAME "od-logger-BLE"
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
#define FLAG_SENT 0x01

/* --------------------------------------------------------------------------
 *  Sensor Configuration: Aqualabo DIGISENS Optical Dissolved Oxygen (OD)
 *
 *  Communicates via Modbus RTU over RS-485 at 9600 baud (8N1).
 *  Readings:
 *    [0] = Water Temperature (centi-°C)
 *    [1] = Dissolved Oxygen (milli-mg/L)
 *    [2] = DO Saturation (deci-%Sat)
 *    [3] = Parameter 3 (centi)
 *    [4] = Parameter 4 (centi)
 *    [5] = Battery Voltage (mV)
 * -------------------------------------------------------------------------- */

// --- Optical Dissolved Oxygen (OD) sensor (Modbus RTU over RS-485) ---
#ifndef OD_RX_PIN
#define OD_RX_PIN 34
#endif
#ifndef OD_TX_PIN
#define OD_TX_PIN 32
#endif
#ifndef OD_PWR_PIN
#define OD_PWR_PIN 19
#endif
#ifndef OD_DE_RE_PIN
#define OD_DE_RE_PIN 0
#endif
#ifndef OD_BAUD
#define OD_BAUD 9600
#endif
#ifndef OD_SLAVE_ID
#define OD_SLAVE_ID 10
#endif
#ifndef OD_SAMPLE_COUNT
#define OD_SAMPLE_COUNT 1
#endif
#ifndef OD_MODBUS_REGISTER_ORDER
#define OD_MODBUS_REGISTER_ORDER 0x0001
#endif
#ifndef OD_MODBUS_ORDER_VALUE
#define OD_MODBUS_ORDER_VALUE 0x001F
#endif
#ifndef OD_MODBUS_START_ADDR
#define OD_MODBUS_START_ADDR 0x0053
#endif
#ifndef OD_MODBUS_READ_COUNT
#define OD_MODBUS_READ_COUNT 10
#endif
#ifndef OD_SAMPLING_DELAY_REG
#define OD_SAMPLING_DELAY_REG 0x00A4
#endif
#ifndef OD_DEFAULT_SAMPLING_DELAY_MS
#define OD_DEFAULT_SAMPLING_DELAY_MS 250
#endif
#ifndef OD_WARMUP_MS
#define OD_WARMUP_MS 1500
#endif
#ifndef OD_NUM_READINGS
// 5 decoded float readings from 10 holding registers:
// [0]=Temp (centi °C), [1]=DO (milli-mg/L), [2]=DO Sat (deci %Sat), [3]=P3, [4]=P4
#define OD_NUM_READINGS 5
#endif

#ifndef SENSOR_NAME
#define SENSOR_NAME "od-digisens"
#endif

/* --------------------------------------------------------------------------
 *  Battery Monitoring Configuration
 *  - SENSOR_BATTERY_ADC: ADC divider on GPIO 35 (Natek-GPRS v0.2.0 board)
 *  - SENSOR_INA226: I2C voltage/current monitor at 0x45 (Natek-SD v2.0 board)
 * -------------------------------------------------------------------------- */

#if defined(SENSOR_INA226) && defined(SENSOR_BATTERY_ADC)
#error "SENSOR_INA226 and SENSOR_BATTERY_ADC are mutually exclusive (both monitor battery voltage)."
#endif

#ifdef SENSOR_BATTERY_ADC
#ifndef BATTERY_ADC_PIN
#define BATTERY_ADC_PIN 35
#endif
#ifndef BATTERY_ADC_MULTIPLIER
#define BATTERY_ADC_MULTIPLIER 2
#endif
#define BATTERY_NUM_READINGS 1
#elif defined(SENSOR_INA226)
#ifndef INA226_ADDR
#define INA226_ADDR 0x45
#endif
#ifndef INA226_WARMUP_MS
#define INA226_WARMUP_MS 200
#endif
#define BATTERY_NUM_READINGS 1
#else
#define BATTERY_NUM_READINGS 0
#endif

// --- Total measurement slots: 5 OD readings + 1 battery reading = 6 ---
#ifndef MAX_SENSOR_READINGS
#define MAX_SENSOR_READINGS (OD_NUM_READINGS + BATTERY_NUM_READINGS)
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
 *  Field layout matches EEPROM page format and CSV column order.
 *  Total size: 2 + 2 + (6 * 2) + 4 + 4 = 24 bytes (fits within 32-byte EEPROM page).
 * -------------------------------------------------------------------------- */

/**
 * @brief Main telemetry payload structure sent to the server.
 */
struct TelemetryPayload {
  int16_t freeStorage;
  int16_t temperature;
  int16_t sensorReadings[MAX_SENSOR_READINGS];
#if defined(SENSOR_BATTERY_ADC) || defined(SENSOR_INA226)
  uint32_t battery;
#endif
  uint32_t status;
};

/* --------------------------------------------------------------------------
 *  Status Bit Definitions
 * -------------------------------------------------------------------------- */

#define STATUS_INTERNET_BIT 0
#define STATUS_FULL_MEMORY_BIT 1
#define STATUS_CORRUPTED_MEMORY_BIT 2
#define STATUS_CODE_ERROR_BIT 3
#define STATUS_SD_CARD_BIT 17
#define STATUS_TEMPERATURE_BIT 18
#define STATUS_GRPS_BIT 19
#define STATUS_OD_SENSOR_BIT 22

/* --------------------------------------------------------------------------
 *  Transmission Recovery
 * -------------------------------------------------------------------------- */

#ifndef MAX_FAILED_TRANSMISSIONS
#define MAX_FAILED_TRANSMISSIONS 3
#endif

/* --------------------------------------------------------------------------
 *  I2C Pin Defaults (overridable per build)
 *  Set to -1 to use Wire.begin() defaults (SDA=21, SCL=22 on ESP32).
 * -------------------------------------------------------------------------- */

#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 21
#endif

#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 22
#endif

/* --------------------------------------------------------------------------
 *  EEPROM I2C Bus Selection
 *
 *  By default, the external I2C EEPROM shares the main Wire bus (SDA=21/SCL=22).
 *  Some hardware revisions (e.g. LILYGO T-A7670 GPRS boards) have the EEPROM's
 *  SDA/SCL traces swapped on the PCB. For those boards, define:
 *
 *      -DI2C_EEPROM_SDA_PIN=22
 *      -DI2C_EEPROM_SCL_PIN=21
 *
 *  When both pins are set to valid GPIO numbers (not -1), a second I2C bus
 *  (Wire1) is activated for the EEPROM with the specified (reversed) pin mapping.
 *  The main Wire bus remains unchanged for the RTC and any other I2C peripherals.
 * -------------------------------------------------------------------------- */

#ifndef I2C_EEPROM_SDA_PIN
#define I2C_EEPROM_SDA_PIN -1
#endif

#ifndef I2C_EEPROM_SCL_PIN
#define I2C_EEPROM_SCL_PIN -1
#endif

#if I2C_EEPROM_SDA_PIN != -1 && I2C_EEPROM_SCL_PIN != -1
#define EEPROM_USE_WIRE1
#define EEPROM_WIRE Wire1
#else
#define EEPROM_WIRE Wire
#endif

/* --------------------------------------------------------------------------
 *  Pin Defaults (overridable per build)
 * -------------------------------------------------------------------------- */

#ifndef LED_PIN
#define LED_PIN 23
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

#ifndef SD_EN_PIN
#define SD_EN_PIN -1
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