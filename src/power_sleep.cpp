/**
 * @file power_sleep.cpp
 * @brief Implementation of low-power helpers for SD bus, radio shutdown,
 *        and deep-sleep transition.
 *
 * Feature guards:
 *  - #ifdef SD_MODULE: SD bus suspend/resume
 *  - #ifdef GPRS_MODULE: modem sleep call
 *  - WiFi/BT radio shutdown always included (needed for deep sleep)
 */

#include "power_sleep.h"

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <driver/gpio.h>
#include <esp_bt.h>
#include <esp_sleep.h>

#include "board.h"
#include "log.h"
#include "n_logger_config.h"
#include "oled_display.h"
#include "rtc_utils.h"
#include "sensor_engine.h"

#ifdef SD_MODULE
#include <SdFat.h>
#endif

/* ------------------------ External Objects from main.cpp ------------------- */

#ifdef SD_MODULE
extern SdFat SD;
extern SdFile sdFile;
extern SPIClass sdSPI;
#endif

extern ModuleConfig moduleCfg;

/* ------------------------------ SD Power Helpers ------------------------------ */

/**
 * @brief Suspends the SD SPI bus to prevent leakage during deep sleep.
 */
void sdBusSuspend() {
#ifdef SD_MODULE
  sdSPI.end();

  pinMode(SD_PIN_CS, OUTPUT);
  digitalWrite(SD_PIN_CS, HIGH);

  pinMode(SD_PIN_SCLK, INPUT);
  pinMode(SD_PIN_MOSI, INPUT);
  pinMode(SD_PIN_MISO, INPUT);

#if SD_EN_PIN != -1
  setPinInactive(SD_EN_PIN, SD_EN_ACTIVE_HIGH, true);
#endif
#else
  // No SD module — no-op
#endif
}

/**
 * @brief Resumes the SD bus after waking from deep sleep.
 */
void sdBusResume() {
#ifdef SD_MODULE
#if SD_EN_PIN != -1
  setPinActive(SD_EN_PIN, SD_EN_ACTIVE_HIGH);
  delay(5);
#endif
  sdSPI.begin(SD_PIN_SCLK, SD_PIN_MISO, SD_PIN_MOSI, SD_PIN_CS);
#else
  // No SD module — no-op
#endif
}

/* --------------------------- Radio Power Management --------------------------- */

/**
 * @brief Ensures both WiFi and Bluetooth controllers are fully disabled.
 */
void boardRadiosOff() {
  WiFi.mode(WIFI_OFF);
  btStop();
#ifdef CONFIG_BT_ENABLED
  esp_bt_controller_disable();
#endif
}

/* ------------------------------- Deep Sleep Prep ------------------------------ */

/**
 * @brief Prepares all peripherals and enters ESP32 deep sleep.
 *
 * Operations performed:
 *  - Validate interval (fallback to 1 min)
 *  - Suspend SD
 *  - Disable sensor and UART
 *  - Disable LEDs
 *  - Configure DS3231 alarm
 *  - Enable EXT1 wakeup (same mask as user sketch)
 *  - Call `esp_deep_sleep_start()`
 */
void prepareForDeepSleep(int minutes) {
  int interval = (minutes <= 0) ? 1 : minutes;

  oledPowerOff();

  sdBusSuspend();

  digitalWrite(LED_PIN, LOW);

// Lock sensor power/control pins inactive through deep sleep with GPIO hold enabled
#if defined(SENSOR_SOIL_3TIP) || defined(SENSOR_SOIL_5TIP) || defined(SENSOR_OD)
#ifdef RS485_EN_PIN
  setPinInactive(RS485_EN_PIN, RS485_EN_ACTIVE_HIGH, true);
#endif
#endif
#ifdef SENSOR_SOIL_3TIP
  setPinInactive(SOIL3_PWR_PIN, SOIL3_PWR_ACTIVE_HIGH, true);
  setPinInactive(SOIL3_DE_RE_PIN, true, true);
  setPinInput(SOIL3_RX_PIN);
  setPinInput(SOIL3_TX_PIN);
#endif
#ifdef SENSOR_RADAR
  setPinInactive(RADAR_PWR_PIN, RADAR_PWR_ACTIVE_HIGH, true);
  setPinInput(RADAR_RX_PIN);
  setPinInput(RADAR_TX_PIN);
#endif
#ifdef SENSOR_SOIL_5TIP
  setPinInactive(SOIL5_PWR_PIN, SOIL5_PWR_ACTIVE_HIGH, true);
  setPinInactive(SOIL5_DE_RE_PIN, true, true);
  setPinInput(SOIL5_RX_PIN);
  setPinInput(SOIL5_TX_PIN);
#endif
#ifdef SENSOR_OD
  setPinInactive(OD_PWR_PIN, OD_PWR_ACTIVE_HIGH, true);
  setPinInactive(OD_DE_RE_PIN, true, true);
  setPinInput(OD_RX_PIN);
  setPinInput(OD_TX_PIN);
#endif
#ifdef SENSOR_MAXBOTIX_UART
  setPinInactive(MAXBOTIX_PWR_PIN, MAXBOTIX_PWR_ACTIVE_HIGH, true);
  setPinInput(MAXBOTIX_RX_PIN);
  setPinInput(MAXBOTIX_TX_PIN);
#endif
#if defined(SENSOR_WATER_TEMP) || defined(SENSOR_MLX90614)
#if WATER_TEMP_PWR_PIN != -1
  setPinInactive(WATER_TEMP_PWR_PIN, WATER_TEMP_PWR_ACTIVE_HIGH, true);
#endif
#endif

// Put INA226 into power-down mode before deep sleep
// (I2C command, no GPIO needed — chip enters low-quiescent-current shutdown)
#ifdef SENSOR_INA226
  ina226PowerDown();
#endif

#ifdef DEBUG
  Logln("");
  Log("WAKE UP IN ");
  Log(String(moduleCfg.measure_time).c_str());
  Logln(" MIN");
#endif

  // Ensure wakeup pin is input (RTC EXT1 handles it during deep sleep,
  // but explicitly setting INPUT prevents leakage from floating state)
  setPinInput(WAKEUP_PIN);

  configureAlarm(interval);

  esp_sleep_enable_ext1_wakeup(WAKEUP_PIN_BITMASK, ESP_EXT1_WAKEUP_ALL_LOW);

  // Double blink LED to visually signal transition to deep sleep
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(150);
  digitalWrite(LED_PIN, LOW);
  delay(150);
  digitalWrite(LED_PIN, HIGH);
  delay(150);
  digitalWrite(LED_PIN, LOW);
  delay(150);

  Serial.flush();

  esp_deep_sleep_start();
}
