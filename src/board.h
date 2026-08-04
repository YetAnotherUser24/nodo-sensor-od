/**
 * @file board.h
 * @brief Board pin definitions and peripheral isolation for n-logger platform.
 *
 * Replaces legacy utilities.h. Select board via build flags:
 *   -DBOARD_NATEK_SD_V2    : Natek-SD v2.0 board (bare ESP32-WROOM-32E MCU)
 *   -DBOARD_NATEK_GPRS_V02 : Natek-GPRS v0.2.0 board (LilyGO T-A7670SA base + hat)
 *
 * If no board flag is set, defaults to BOARD_NATEK_SD_V2.
 */

#pragma once

#include <Arduino.h>
#include <driver/gpio.h>

// Fallback default board if no build flag is set
#if !defined(BOARD_NATEK_SD_V2) && !defined(BOARD_NATEK_GPRS_V02)
#define BOARD_NATEK_SD_V2
#endif

/* ==========================================================================
 *  Pin Power & Polarity Helper Functions
 * ========================================================================== */

/**
 * @brief Sets an enable/power control pin to its ACTIVE (ON) state.
 * @param pin GPIO pin number (-1 if pin is not used/present on board).
 * @param activeHigh True if pin is active-HIGH, false if active-LOW.
 */
inline void setPinActive(int pin, bool activeHigh) {
  if (pin != -1) {
    gpio_hold_dis((gpio_num_t)pin);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, activeHigh ? HIGH : LOW);
  }
}

/**
 * @brief Sets a GPIO pin to INPUT mode safely.
 * @param pin GPIO pin number (-1 if pin is not used/present on board).
 */
inline void setPinInput(int pin) {
  if (pin != -1) {
    pinMode(pin, INPUT);
  }
}

/**
 * @brief Sets an enable/power control pin to its INACTIVE (OFF) state.
 * @param pin GPIO pin number (-1 if pin is not used/present on board).
 * @param activeHigh True if pin is active-HIGH, false if active-LOW.
 * @param holdInSleep If true, applies gpio_hold_en to retain pin state during deep sleep.
 */
inline void setPinInactive(int pin, bool activeHigh, bool holdInSleep = false) {
  if (pin != -1) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, activeHigh ? LOW : HIGH);
    if (holdInSleep) {
      gpio_hold_en((gpio_num_t)pin);
    }
  }
}

/* ==========================================================================
 *  BOARD 1: Natek-SD v2.0 (Bare ESP32-WROOM-32E MCU)
 * ========================================================================== */
#if defined(BOARD_NATEK_SD_V2)

#define BOARD_NAME "Natek-SD v2.0"

#ifndef LED_PIN
#define LED_PIN 32
#endif

#ifndef WAKEUP_PIN
#define WAKEUP_PIN GPIO_NUM_4
#endif

#ifndef WAKEUP_PIN_BITMASK
#define WAKEUP_PIN_BITMASK (1ULL << WAKEUP_PIN)
#endif

/* Power Enable Control Pins & Active Polarities */
#ifndef EN_5V_PIN
#define EN_5V_PIN 12
#endif
#ifndef EN_5V_ACTIVE_HIGH
#define EN_5V_ACTIVE_HIGH true
#endif

/* SD Card SPI Pins */
#ifndef SD_PIN_CS
#define SD_PIN_CS 5
#endif

#ifndef SD_PIN_SCLK
#define SD_PIN_SCLK 18
#endif

#ifndef SD_PIN_MISO
#define SD_PIN_MISO 19
#endif

#ifndef SD_PIN_MOSI
#define SD_PIN_MOSI 23
#endif

#ifndef SD_EN_PIN
#define SD_EN_PIN 15
#endif
#ifndef SD_EN_ACTIVE_HIGH
#define SD_EN_ACTIVE_HIGH false
#endif

/* Main I2C Bus Pins */
#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 22
#endif

#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 21
#endif

#ifndef I2C_EN_PIN
#define I2C_EN_PIN 27
#endif
#ifndef I2C_EN_ACTIVE_HIGH
#define I2C_EN_ACTIVE_HIGH true
#endif

/* EEPROM Swapped I2C Bus Pins (Wire1) */
#ifndef I2C_EEPROM_SDA_PIN
#define I2C_EEPROM_SDA_PIN -1
#endif

#ifndef I2C_EEPROM_SCL_PIN
#define I2C_EEPROM_SCL_PIN -1
#endif

/* Main RS-485 Bus Pins */
#ifndef RS485_EN_PIN
#define RS485_EN_PIN 26
#endif
#ifndef RS485_EN_ACTIVE_HIGH
#define RS485_EN_ACTIVE_HIGH false
#endif

/* Soil Sensor Pins */
#ifndef SOIL3_RX_PIN
#define SOIL3_RX_PIN 16
#endif

#ifndef SOIL3_TX_PIN
#define SOIL3_TX_PIN 17
#endif

#ifndef SOIL3_DE_RE_PIN
#define SOIL3_DE_RE_PIN 2
#endif

#ifndef SOIL3_PWR_PIN
#define SOIL3_PWR_PIN 12
#endif
#ifndef SOIL3_PWR_ACTIVE_HIGH
#define SOIL3_PWR_ACTIVE_HIGH true
#endif

/* MaxBotix Sensor Pins */
#ifndef MAXBOTIX_RX_PIN
#define MAXBOTIX_RX_PIN 34
#endif

#ifndef MAXBOTIX_TX_PIN
#define MAXBOTIX_TX_PIN -1
#endif

#ifndef MAXBOTIX_PWR_PIN
#define MAXBOTIX_PWR_PIN 27
#endif
#ifndef MAXBOTIX_PWR_ACTIVE_HIGH
#define MAXBOTIX_PWR_ACTIVE_HIGH true
#endif

/* Radar Sensor Pins */
#ifndef RADAR_RX_PIN
#define RADAR_RX_PIN 34
#endif

#ifndef RADAR_TX_PIN
#define RADAR_TX_PIN 14
#endif

#ifndef RADAR_PWR_PIN
#define RADAR_PWR_PIN 12
#endif
#ifndef RADAR_PWR_ACTIVE_HIGH
#define RADAR_PWR_ACTIVE_HIGH true
#endif

/* ==========================================================================
 *  BOARD 2: Natek-GPRS v0.2.0 (LilyGO T-A7670SA base module + hat)
 * ========================================================================== */
#elif defined(BOARD_NATEK_GPRS_V02)

#define BOARD_NAME "Natek-GPRS v0.2.0"

#ifndef LED_PIN
#define LED_PIN 23
#endif

#ifndef WAKEUP_PIN
#define WAKEUP_PIN GPIO_NUM_39
#endif

#ifndef WAKEUP_PIN_BITMASK
#define WAKEUP_PIN_BITMASK (1ULL << WAKEUP_PIN)
#endif

/* Modem Pins & Controls */
#ifndef MODEM_BAUDRATE
#define MODEM_BAUDRATE 115200
#endif

#ifndef MODEM_DTR_PIN
#define MODEM_DTR_PIN 25
#endif

#ifndef MODEM_TX_PIN
#define MODEM_TX_PIN 26
#endif

#ifndef MODEM_RX_PIN
#define MODEM_RX_PIN 27
#endif

#ifndef BOARD_PWRKEY_PIN
#define BOARD_PWRKEY_PIN 4
#endif

#ifndef BOARD_POWERON_PIN
#define BOARD_POWERON_PIN 12
#endif

#ifndef MODEM_RING_PIN
#define MODEM_RING_PIN 33
#endif

#ifndef MODEM_RESET_PIN
#define MODEM_RESET_PIN 5
#endif

#ifndef MODEM_RESET_LEVEL
#define MODEM_RESET_LEVEL HIGH
#endif

#ifndef SerialAT
#define SerialAT Serial1
#endif

#ifndef MODEM_GPS_ENABLE_GPIO
#define MODEM_GPS_ENABLE_GPIO (-1)
#endif

#ifndef MODEM_GPS_ENABLE_LEVEL
#define MODEM_GPS_ENABLE_LEVEL (-1)
#endif

/* Power Enable Control Pins & Active Polarities */
#ifndef EN_5V_PIN
#define EN_5V_PIN 18
#endif
#ifndef EN_5V_ACTIVE_HIGH
#define EN_5V_ACTIVE_HIGH true
#endif

/* Onboard SD Card SPI Pins */
#ifndef SD_PIN_CS
#define SD_PIN_CS 13
#endif

#ifndef SD_PIN_SCLK
#define SD_PIN_SCLK 14
#endif

#ifndef SD_PIN_MISO
#define SD_PIN_MISO 2
#endif

#ifndef SD_PIN_MOSI
#define SD_PIN_MOSI 15
#endif

#ifndef SD_EN_PIN
#define SD_EN_PIN -1
#endif
#ifndef SD_EN_ACTIVE_HIGH
#define SD_EN_ACTIVE_HIGH true
#endif

/* Main I2C Bus Pins */
#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 21
#endif

#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 12
#endif

#ifndef I2C_EN_PIN
#define I2C_EN_PIN 18
#endif
#ifndef I2C_EN_ACTIVE_HIGH
#define I2C_EN_ACTIVE_HIGH true
#endif

/* EEPROM Swapped I2C Bus Pins (Wire1) */
#ifndef I2C_EEPROM_SDA_PIN
#define I2C_EEPROM_SDA_PIN 22
#endif

#ifndef I2C_EEPROM_SCL_PIN
#define I2C_EEPROM_SCL_PIN 21
#endif

/* Main RS-485 Bus Pins */
#ifndef RS485_EN_PIN
#define RS485_EN_PIN 18
#endif
#ifndef RS485_EN_ACTIVE_HIGH
#define RS485_EN_ACTIVE_HIGH true
#endif

/* Soil Sensor Pins */
#ifndef SOIL3_RX_PIN
#define SOIL3_RX_PIN 34
#endif

#ifndef SOIL3_TX_PIN
#define SOIL3_TX_PIN 32
#endif

#ifndef SOIL3_DE_RE_PIN
#define SOIL3_DE_RE_PIN 0
#endif

#ifndef SOIL3_PWR_PIN
#define SOIL3_PWR_PIN 19
#endif
#ifndef SOIL3_PWR_ACTIVE_HIGH
#define SOIL3_PWR_ACTIVE_HIGH true
#endif

/* MaxBotix Sensor Pins */
#ifndef MAXBOTIX_RX_PIN
#define MAXBOTIX_RX_PIN 34
#endif

#ifndef MAXBOTIX_TX_PIN
#define MAXBOTIX_TX_PIN -1
#endif

#ifndef MAXBOTIX_PWR_PIN
#define MAXBOTIX_PWR_PIN 18
#endif
#ifndef MAXBOTIX_PWR_ACTIVE_HIGH
#define MAXBOTIX_PWR_ACTIVE_HIGH true
#endif

/* Radar Sensor Pins */
#ifndef RADAR_RX_PIN
#define RADAR_RX_PIN 34
#endif

#ifndef RADAR_TX_PIN
#define RADAR_TX_PIN 32
#endif

#ifndef RADAR_PWR_PIN
#define RADAR_PWR_PIN 19
#endif
#ifndef RADAR_PWR_ACTIVE_HIGH
#define RADAR_PWR_ACTIVE_HIGH true
#endif

/* Battery ADC Pin */
#ifndef BOARD_BAT_ADC_PIN
#define BOARD_BAT_ADC_PIN 35
#endif

#endif

/* ==========================================================================
 *  Board Hardware Initialization & Power Control
 * ========================================================================== */

/**
 * @brief Initializes board status LEDs and puts all board power rails into low-power inactive
 * state.
 *
 * Safe for any board revision (if a pin is set to -1 or missing, it is safely ignored).
 * Active HIGH / LOW polarities are handled automatically per pin.
 * Call at the top of setup().
 */
inline void boardInit() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

#ifndef GPRS_MODULE
#ifdef BOARD_POWERON_PIN
  setPinInactive(BOARD_POWERON_PIN, true);
#endif
#ifdef BOARD_PWRKEY_PIN
  setPinInactive(BOARD_PWRKEY_PIN, true);
#endif
#ifdef MODEM_RESET_PIN
  setPinInactive(MODEM_RESET_PIN, MODEM_RESET_LEVEL == HIGH);
#endif
#endif

#ifdef SD_MODULE
  setPinActive(SD_EN_PIN, SD_EN_ACTIVE_HIGH);
#else
  setPinInactive(SD_EN_PIN, SD_EN_ACTIVE_HIGH);
#endif

  // Put all defined board power rails into default inactive OFF state at boot
#ifdef RS485_EN_PIN
  setPinInactive(RS485_EN_PIN, RS485_EN_ACTIVE_HIGH);
#endif
  setPinInactive(EN_5V_PIN, EN_5V_ACTIVE_HIGH);
#ifdef SOIL3_PWR_PIN
  setPinInactive(SOIL3_PWR_PIN, SOIL3_PWR_ACTIVE_HIGH);
#endif
#ifdef SOIL5_PWR_PIN
  setPinInactive(SOIL5_PWR_PIN, SOIL5_PWR_ACTIVE_HIGH);
#endif
#ifdef MAXBOTIX_PWR_PIN
  setPinInactive(MAXBOTIX_PWR_PIN, MAXBOTIX_PWR_ACTIVE_HIGH);
#endif
#ifdef RADAR_PWR_PIN
  setPinInactive(RADAR_PWR_PIN, RADAR_PWR_ACTIVE_HIGH);
#endif
#ifdef WATER_TEMP_PWR_PIN
  setPinInactive(WATER_TEMP_PWR_PIN, WATER_TEMP_PWR_ACTIVE_HIGH);
#endif
  setPinInactive(I2C_EN_PIN, I2C_EN_ACTIVE_HIGH);
}
