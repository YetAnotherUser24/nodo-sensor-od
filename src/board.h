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
#include <SPI.h>
#include <driver/gpio.h>

// Fallback default board if no build flag is set
#if !defined(BOARD_NATEK_SD_V2) && !defined(BOARD_NATEK_GPRS_V02) && !defined(BOARD_LILYGO_LORA32)
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
    if (GPIO_IS_VALID_OUTPUT_GPIO(pin)) {
      gpio_hold_dis((gpio_num_t)pin);
    }
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
    if (holdInSleep && GPIO_IS_VALID_OUTPUT_GPIO(pin)) {
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

/* SD Card SPI Pins (HSPI) */
#ifndef SD_SPI_HOST
#define SD_SPI_HOST HSPI
#endif
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

/* OLED Display Configuration (1.3" SH1106) */
#ifndef OLED_CONTROLLER_SH1106
#define OLED_CONTROLLER_SH1106
#endif
#ifndef OLED_COLUMN_OFFSET
#define OLED_COLUMN_OFFSET 0x02
#endif
#ifndef OLED_TYPE_NAME
#define OLED_TYPE_NAME "1.3\" SH1106"
#endif
#ifndef OLED_RST_PIN
#define OLED_RST_PIN -1
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

/* OD Sensor Pins (Modbus RTU over RS-485) */
#ifndef OD_RX_PIN
#define OD_RX_PIN 16
#endif

#ifndef OD_TX_PIN
#define OD_TX_PIN 17
#endif

#ifndef OD_DE_RE_PIN
#define OD_DE_RE_PIN 2
#endif

#ifndef OD_PWR_PIN
#define OD_PWR_PIN 12
#endif
#ifndef OD_PWR_ACTIVE_HIGH
#define OD_PWR_ACTIVE_HIGH true
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
#define EN_5V_PIN 19
#endif
#ifndef EN_5V_ACTIVE_HIGH
#define EN_5V_ACTIVE_HIGH true
#endif

/* Onboard SD Card SPI Pins (HSPI) */
#ifndef SD_SPI_HOST
#define SD_SPI_HOST HSPI
#endif
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
#define I2C_SCL_PIN 22
#endif

#ifndef I2C_EN_PIN
#define I2C_EN_PIN 18
#endif
#ifndef I2C_EN_ACTIVE_HIGH
#define I2C_EN_ACTIVE_HIGH true
#endif

/* OLED Display Configuration (1.3" SH1106) */
#ifndef OLED_CONTROLLER_SH1106
#define OLED_CONTROLLER_SH1106
#endif
#ifndef OLED_COLUMN_OFFSET
#define OLED_COLUMN_OFFSET 0x02
#endif
#ifndef OLED_TYPE_NAME
#define OLED_TYPE_NAME "1.3\" SH1106"
#endif
#ifndef OLED_RST_PIN
#define OLED_RST_PIN -1
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

/* OD Sensor Pins (Modbus RTU over RS-485) */
#ifndef OD_RX_PIN
#define OD_RX_PIN 34
#endif

#ifndef OD_TX_PIN
#define OD_TX_PIN 32
#endif

#ifndef OD_DE_RE_PIN
#define OD_DE_RE_PIN 0
#endif

#ifndef OD_PWR_PIN
#define OD_PWR_PIN 19
#endif
#ifndef OD_PWR_ACTIVE_HIGH
#define OD_PWR_ACTIVE_HIGH true
#endif

/* Battery ADC Pin */
#ifndef BOARD_BAT_ADC_PIN
#define BOARD_BAT_ADC_PIN 35
#endif

#endif

/* ==========================================================================
 *  BOARD 3: LilyGO T3 LoRa32 + Custom Instrumentation HAT
 * ========================================================================== */
#if defined(BOARD_LILYGO_LORA32)

#define BOARD_NAME "LilyGO T3 LoRa32 + HAT"

#ifndef LED_PIN
#define LED_PIN -1
#endif

#ifndef WAKEUP_PIN
#define WAKEUP_PIN GPIO_NUM_34
#endif

#ifndef WAKEUP_PIN_BITMASK
#define WAKEUP_PIN_BITMASK (1ULL << WAKEUP_PIN)
#endif

/* Power Enable Control Pins */
#ifndef EN_5V_PIN
#define EN_5V_PIN -1
#endif
#ifndef EN_5V_ACTIVE_HIGH
#define EN_5V_ACTIVE_HIGH true
#endif

/* LoRa SX1278 Dedicated SPI Controller (VSPI / SPI3) */
#ifndef LORA_SPI_HOST
#define LORA_SPI_HOST VSPI
#endif

#ifndef LORA_CS_PIN
#define LORA_CS_PIN 18
#endif

#ifndef LORA_PIN_SCLK
#define LORA_PIN_SCLK 5
#endif

#ifndef LORA_PIN_MISO
#define LORA_PIN_MISO 19
#endif

#ifndef LORA_PIN_MOSI
#define LORA_PIN_MOSI 27
#endif

#ifndef LORA_RST_PIN
#define LORA_RST_PIN 23
#endif

#ifndef LORA_DIO0_PIN
#define LORA_DIO0_PIN 26
#endif


/* Main I2C Bus Pins (DS3231MZ + OLED SSD1306) */
#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 21
#endif
#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 22
#endif
#ifndef I2C_EN_PIN
#define I2C_EN_PIN -1
#endif
#ifndef I2C_EN_ACTIVE_HIGH
#define I2C_EN_ACTIVE_HIGH true
#endif

/* OLED Display Configuration (0.96" SSD1306 onboard) */
#ifndef OLED_CONTROLLER_SSD1306
#define OLED_CONTROLLER_SSD1306
#endif
#ifndef OLED_COLUMN_OFFSET
#define OLED_COLUMN_OFFSET 0x00
#endif
#ifndef OLED_TYPE_NAME
#define OLED_TYPE_NAME "0.96\" SSD1306"
#endif
#ifndef OLED_RST_PIN
#define OLED_RST_PIN -1
#endif

/* RS-485 Modbus RTU Pins (SP3485EN on HAT) */
#ifndef OD_RX_PIN
#define OD_RX_PIN 39
#endif
#ifndef OD_TX_PIN
#define OD_TX_PIN 4
#endif
#ifndef OD_DE_RE_PIN
#define OD_DE_RE_PIN 0
#endif
#ifndef OD_PWR_PIN
#define OD_PWR_PIN 25
#endif
#ifndef OD_PWR_ACTIVE_HIGH
#define OD_PWR_ACTIVE_HIGH true
#endif

/* Onboard MicroSD Dedicated SPI Controller (HSPI / SPI2) */
#ifndef SD_SPI_HOST
#define SD_SPI_HOST HSPI
#endif

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

/* RTC DS3231MZ Interrupt Pin */
#ifndef RTC_INT_PIN
#define RTC_INT_PIN 34
#endif

/* Battery ADC Pin */
#ifndef BOARD_BAT_ADC_PIN
#define BOARD_BAT_ADC_PIN 36
#endif

#endif

/* ==========================================================================
 *  Fallback Defaults for Display Configuration
 * ========================================================================== */
#ifndef OLED_COLUMN_OFFSET
#if defined(OLED_CONTROLLER_SSD1306)
#define OLED_COLUMN_OFFSET 0x00
#else
#define OLED_COLUMN_OFFSET 0x02
#endif
#endif

#ifndef OLED_TYPE_NAME
#if defined(OLED_CONTROLLER_SSD1306)
#define OLED_TYPE_NAME "0.96\" SSD1306"
#else
#define OLED_TYPE_NAME "1.3\" SH1106"
#endif
#endif

#ifndef OLED_RST_PIN
#define OLED_RST_PIN -1
#endif

#ifndef SD_SPI_HOST
#define SD_SPI_HOST HSPI
#endif

#ifndef LORA_SPI_HOST
#define LORA_SPI_HOST VSPI
#endif

/* ==========================================================================
 *  Board Hardware Initialization & Power Control
 * ========================================================================== */

/**
 * @brief Displays a repeated blink pattern to indicate error codes.
 */
inline void error(int errorNum, int errPin = LED_PIN) {
  for (int i = 0; i < errorNum; i++) {
    digitalWrite(errPin, HIGH);
    delay(150);
    digitalWrite(errPin, LOW);
    delay(150);
  }
}

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

#ifdef BOARD_POWERON_PIN
  setPinInactive(BOARD_POWERON_PIN, true);
#endif
#ifdef BOARD_PWRKEY_PIN
  setPinInactive(BOARD_PWRKEY_PIN, true);
#endif
#ifdef MODEM_RESET_PIN
  setPinInactive(MODEM_RESET_PIN, MODEM_RESET_LEVEL == HIGH);
#endif

#ifdef SD_MODULE
  setPinActive(SD_EN_PIN, SD_EN_ACTIVE_HIGH);
#else
  setPinInactive(SD_EN_PIN, SD_EN_ACTIVE_HIGH);
#endif

  // Ensure both independent SPI Chip Select lines are deselected (HIGH) at boot
#ifdef SD_PIN_CS
  if (SD_PIN_CS != -1) {
    pinMode(SD_PIN_CS, OUTPUT);
    digitalWrite(SD_PIN_CS, HIGH);
  }
#endif
#ifdef LORA_CS_PIN
  if (LORA_CS_PIN != -1) {
    pinMode(LORA_CS_PIN, OUTPUT);
    digitalWrite(LORA_CS_PIN, HIGH);
  }
#endif

  // Put all defined board power rails into default inactive OFF state at boot
#ifdef RS485_EN_PIN
  setPinInactive(RS485_EN_PIN, RS485_EN_ACTIVE_HIGH);
#endif
  setPinInactive(EN_5V_PIN, EN_5V_ACTIVE_HIGH);
#ifdef OD_PWR_PIN
  setPinInactive(OD_PWR_PIN, OD_PWR_ACTIVE_HIGH);
#endif
  setPinInactive(I2C_EN_PIN, I2C_EN_ACTIVE_HIGH);
}

/**
 * @brief Initializes the dedicated LoRa SX1278 SPI bus on VSPI (SPI3).
 */
inline void loraSpiInit() {
#if defined(LORA_CS_PIN) && (LORA_CS_PIN != -1)
  pinMode(LORA_CS_PIN, OUTPUT);
  digitalWrite(LORA_CS_PIN, HIGH);
#endif
#if defined(LORA_RST_PIN) && (LORA_RST_PIN != -1)
  pinMode(LORA_RST_PIN, OUTPUT);
  digitalWrite(LORA_RST_PIN, HIGH);
#endif
#if defined(LORA_PIN_SCLK) && (LORA_PIN_SCLK != -1)
  SPI.begin(LORA_PIN_SCLK, LORA_PIN_MISO, LORA_PIN_MOSI, LORA_CS_PIN);
#endif
}

/**
 * @brief LED pattern signaling entry into IDLE state (3 quick pulses).
 */
inline void ledSignalIdle() {
#if defined(LED_PIN) && (LED_PIN != -1)
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    if (i < 2) delay(100);
  }
#endif
}
