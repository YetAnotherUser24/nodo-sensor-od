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
#define FLAG_SENT 0x01

/* --------------------------------------------------------------------------
 *  Sensor Configuration — Descriptive Sensor Flags
 *
 *  Enable sensors by defining one or more of these flags at build time:
 *    -DSENSOR_MAXBOTIX_UART     = MaxBotix MB12xx ultrasonic distance (UART)
 *    -DSENSOR_WATER_TEMP        = Water surface temperature (I2C)
 *    -DSENSOR_RADAR             = Radar distance sensor (Modbus RTU, direct UART)
 *    -DSENSOR_SOIL_3TIP         = 3-probe soil moisture/temperature (Modbus RTU, RS-485)
 *    -DSENSOR_SOIL_5TIP         = 5-probe soil comprehensive sensor (Modbus RTU, RS-485)
 *    -DSENSOR_OD                = Optical dissolved oxygen DIGISENS sensor (Modbus RTU, RS-485)
 *    -DSENSOR_BATTERY_ADC       = Battery voltage via ADC pin
 *
 *  NOTE: SENSOR_RADAR, SENSOR_SOIL_3TIP, SENSOR_SOIL_5TIP, and SENSOR_OD share
 *  UART pins (GPIO 34/32 or 16/17) and power pin (GPIO 19 or 12).
 *  They are mutually exclusive — do NOT enable more than one in the same build.
 *
 *  Each sensor defines its own pins, number of readings, and warmup time.
 *  Multiple compatible sensors can be active simultaneously.
 *  MAX_SENSOR_READINGS is auto-computed from all enabled sensors.
 *
 *  To override a pin at build time, define it before this header is included
 *  (e.g. -DMAXBOTIX_RX_PIN=22 in platformio.ini build_flags).
 * -------------------------------------------------------------------------- */

// Radar, 3-probe soil, 5-probe soil, and OD sensors share UART pins and power pin.
// At most one of these may be enabled in a single build.
#if (defined(SENSOR_RADAR) + defined(SENSOR_SOIL_3TIP) + defined(SENSOR_SOIL_5TIP) +               \
     defined(SENSOR_OD)) > 1
#error                                                                                             \
    "SENSOR_RADAR, SENSOR_SOIL_3TIP, SENSOR_SOIL_5TIP, and SENSOR_OD share hardware pins and are mutually exclusive."
#endif

// --- MaxBotix MB12xx ultrasonic sensor over UART ---
#ifdef SENSOR_MAXBOTIX_UART
#ifndef MAXBOTIX_RX_PIN
#define MAXBOTIX_RX_PIN 34
#endif
#ifndef MAXBOTIX_TX_PIN
#define MAXBOTIX_TX_PIN 32
#endif
#ifndef MAXBOTIX_PWR_PIN
#define MAXBOTIX_PWR_PIN 18
#endif
#ifndef MAXBOTIX_NUM_READINGS
#define MAXBOTIX_NUM_READINGS 5
#endif
#ifndef MAXBOTIX_WARMUP_MS
#define MAXBOTIX_WARMUP_MS 2500
#endif
#ifndef MAXBOTIX_SAMPLE_TIMEOUT_MS
#define MAXBOTIX_SAMPLE_TIMEOUT_MS 1000
#endif
#undef SENSOR_NAME
#define SENSOR_NAME "maxbotix-uart"
#else
#define MAXBOTIX_NUM_READINGS 0
#endif

// --- Water / IR Surface Temperature sensor (MLX90614ESF-DCI over I2C) ---
#if defined(SENSOR_WATER_TEMP) || defined(SENSOR_MLX90614)
#ifndef WATER_TEMP_ADDR
#define WATER_TEMP_ADDR 0x5A
#endif
#ifndef WATER_TEMP_PWR_PIN
#define WATER_TEMP_PWR_PIN 27
#endif
#ifndef WATER_TEMP_PWR_ACTIVE_HIGH
#define WATER_TEMP_PWR_ACTIVE_HIGH true
#endif
#ifndef WATER_TEMP_NUM_READINGS
#define WATER_TEMP_NUM_READINGS 2 // 0: Object Temp, 1: Ambient Temp
#endif
#ifndef WATER_TEMP_WARMUP_MS
#define WATER_TEMP_WARMUP_MS 100
#endif
#undef SENSOR_NAME
#define SENSOR_NAME "mlx90614"
#else
#define WATER_TEMP_NUM_READINGS 0
#endif

// --- Radar distance sensor (Modbus RTU over direct UART) ---
#ifdef SENSOR_RADAR
#ifndef RADAR_RX_PIN
#define RADAR_RX_PIN 34
#endif
#ifndef RADAR_TX_PIN
#define RADAR_TX_PIN 32
#endif
#ifndef RADAR_PWR_PIN
#define RADAR_PWR_PIN 19
#endif
#ifndef RADAR_BAUD
#define RADAR_BAUD 115200
#endif
#ifndef RADAR_SLAVE_ID
#define RADAR_SLAVE_ID 1
#endif
#ifndef RADAR_NUM_READINGS
#define RADAR_NUM_READINGS 5
#endif
#ifndef RADAR_MODBUS_REGISTER
#define RADAR_MODBUS_REGISTER 0x0001
#endif
#ifndef RADAR_WARMUP_MS
#define RADAR_WARMUP_MS 2500
#endif
#ifndef RADAR_SAMPLE_TIMEOUT_MS
#define RADAR_SAMPLE_TIMEOUT_MS 8000
#endif
#undef SENSOR_NAME
#define SENSOR_NAME "radar"
#else
#define RADAR_NUM_READINGS 0
#endif

// --- 3-Probe Soil Moisture/Temperature sensor (Modbus RTU over RS-485) ---
#ifdef SENSOR_SOIL_3TIP
#ifndef SOIL3_RX_PIN
#define SOIL3_RX_PIN 34
#endif
#ifndef SOIL3_TX_PIN
#define SOIL3_TX_PIN 32
#endif
#ifndef SOIL3_PWR_PIN
#define SOIL3_PWR_PIN 19
#endif
#ifndef SOIL3_BAUD
#define SOIL3_BAUD 9600
#endif
#ifndef SOIL3_SLAVE_ID
#define SOIL3_SLAVE_ID 1
#endif
#ifndef SOIL3_SAMPLE_COUNT
#define SOIL3_SAMPLE_COUNT 2
#endif
#ifndef SOIL3_NUM_READINGS
// Each sample reads 2 registers (moisture + temperature), so total
// readings = sample_count * 2.
#define SOIL3_NUM_READINGS (SOIL3_SAMPLE_COUNT * SOIL3_MODBUS_READ_COUNT)
#endif
#ifndef SOIL3_MODBUS_READ_COUNT
#define SOIL3_MODBUS_READ_COUNT 2
#endif
#ifndef SOIL3_MODBUS_START_ADDR
#define SOIL3_MODBUS_START_ADDR 0x0000
#endif
#ifndef SOIL3_WARMUP_MS
#define SOIL3_WARMUP_MS 500
#endif
#undef SENSOR_NAME
#define SENSOR_NAME "soil-3probe"
#else
#define SOIL3_NUM_READINGS 0
#endif

// --- 5-Probe Soil Comprehensive sensor (Modbus RTU over RS-485) ---
// Measures moisture, temperature, conductivity (EC), pH, N, P, K.
// Reads 7 holding registers starting at 0x0000.
#ifdef SENSOR_SOIL_5TIP
#ifndef SOIL5_RX_PIN
#define SOIL5_RX_PIN 34
#endif
#ifndef SOIL5_TX_PIN
#define SOIL5_TX_PIN 32
#endif
#ifndef SOIL5_PWR_PIN
#define SOIL5_PWR_PIN 19
#endif
#ifndef SOIL5_DE_RE_PIN
#define SOIL5_DE_RE_PIN 0
#endif
#ifndef SOIL5_BAUD
#define SOIL5_BAUD 4800
#endif
#ifndef SOIL5_SLAVE_ID
#define SOIL5_SLAVE_ID 1
#endif
#ifndef SOIL5_SAMPLE_COUNT
#define SOIL5_SAMPLE_COUNT 2
#endif
#ifndef SOIL5_MODBUS_READ_COUNT
#define SOIL5_MODBUS_READ_COUNT 7
#endif
#ifndef SOIL5_MODBUS_START_ADDR
#define SOIL5_MODBUS_START_ADDR 0x0000
#endif
#ifndef SOIL5_WARMUP_MS
#define SOIL5_WARMUP_MS 200
#endif
#ifndef SOIL5_NUM_READINGS
#define SOIL5_NUM_READINGS (SOIL5_SAMPLE_COUNT * SOIL5_MODBUS_READ_COUNT)
#endif
#undef SENSOR_NAME
#define SENSOR_NAME "soil-5probe"
#else
#define SOIL5_NUM_READINGS 0
#endif

// --- Optical Dissolved Oxygen (OD) sensor (Modbus RTU over RS-485) ---
#ifdef SENSOR_OD
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
// 5 floats decoded from 10 registers = 5 int16 readings:
// [0]=Temp (centi °C), [1]=DO (centi mg/L), [2]=DO Sat (deci %Sat), [3]=P3, [4]=P4
#define OD_NUM_READINGS 5
#endif
#undef SENSOR_NAME
#define SENSOR_NAME "od-digisens"
#else
#define OD_NUM_READINGS 0
#endif

// --- Battery voltage via ADC ---
#ifdef SENSOR_BATTERY_ADC
#ifndef BATTERY_ADC_PIN
#define BATTERY_ADC_PIN 35
#endif
#ifndef BATTERY_NUM_READINGS
#define BATTERY_NUM_READINGS 1
#endif
#ifndef BATTERY_ADC_MULTIPLIER
#define BATTERY_ADC_MULTIPLIER 2
#endif
#undef SENSOR_NAME
#define SENSOR_NAME "battery-adc"
// Note: SENSOR_BATTERY_ADC is a secondary sensor and does NOT set SENSOR_NAME.
// If another primary sensor (e.g. SENSOR_SOIL_3TIP, SENSOR_RADAR, etc.) is enabled,
// its SENSOR_NAME definition takes precedence due to include order.
#ifdef SENSOR_SOIL_3TIP
#undef SENSOR_NAME
#define SENSOR_NAME "soil-3probe"
#elif defined(SENSOR_RADAR)
#undef SENSOR_NAME
#define SENSOR_NAME "radar"
#elif defined(SENSOR_MAXBOTIX_UART)
#undef SENSOR_NAME
#define SENSOR_NAME "maxbotix-uart"
#elif defined(SENSOR_WATER_TEMP)
#undef SENSOR_NAME
#define SENSOR_NAME "water-temp"
#elif defined(SENSOR_SOIL_5TIP)
#undef SENSOR_NAME
#define SENSOR_NAME "soil-5probe"
#elif defined(SENSOR_OD)
#undef SENSOR_NAME
#define SENSOR_NAME "od-digisens"
#else
#define SENSOR_NAME "battery-adc"
#endif
#else
#define BATTERY_NUM_READINGS 0
#endif

// --- INA226 voltage/current monitor (I2C) ---
// Mutually exclusive with SENSOR_BATTERY_ADC — both populate the `battery` field.
#if defined(SENSOR_INA226) && defined(SENSOR_BATTERY_ADC)
#error "SENSOR_INA226 and SENSOR_BATTERY_ADC are mutually exclusive (both use the battery field)."
#endif

#ifdef SENSOR_INA226
#ifndef INA226_ADDR
#define INA226_ADDR 0x45
#endif
#ifndef INA226_NUM_READINGS
#define INA226_NUM_READINGS 1
#endif
#ifndef INA226_WARMUP_MS
#define INA226_WARMUP_MS 200
#endif
#undef SENSOR_NAME
#define SENSOR_NAME "ina226"
#ifdef SENSOR_SOIL_3TIP
#undef SENSOR_NAME
#define SENSOR_NAME "soil-3probe"
#elif defined(SENSOR_RADAR)
#undef SENSOR_NAME
#define SENSOR_NAME "radar"
#elif defined(SENSOR_MAXBOTIX_UART)
#undef SENSOR_NAME
#define SENSOR_NAME "maxbotix-uart"
#elif defined(SENSOR_WATER_TEMP) || defined(SENSOR_MLX90614)
#undef SENSOR_NAME
#define SENSOR_NAME "mlx90614"
#elif defined(SENSOR_SOIL_5TIP)
#undef SENSOR_NAME
#define SENSOR_NAME "soil-5probe"
#elif defined(SENSOR_OD)
#undef SENSOR_NAME
#define SENSOR_NAME "od-digisens"
#else
#define SENSOR_NAME "ina226"
#endif
#else
#define INA226_NUM_READINGS 0
#endif

// --- Compute total measurement slots ---
#ifndef MAX_SENSOR_READINGS
#define MAX_SENSOR_READINGS                                                                        \
  (MAXBOTIX_NUM_READINGS + WATER_TEMP_NUM_READINGS + RADAR_NUM_READINGS + BATTERY_NUM_READINGS +   \
   INA226_NUM_READINGS + SOIL3_NUM_READINGS + SOIL5_NUM_READINGS + OD_NUM_READINGS)
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
#define STATUS_WATER_LEVEL_SENSOR_BIT 16
#define STATUS_SD_CARD_BIT 17
#define STATUS_TEMPERATURE_BIT 18
#define STATUS_GRPS_BIT 19
#define STATUS_RADAR_SENSOR_BIT 20
#define STATUS_SOIL3_SENSOR_BIT 21
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