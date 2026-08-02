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
#include <WiFi.h>
#include <esp_bt.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <SPI.h>

#include "n_logger_config.h"
#include "sensor_engine.h"
#ifdef GPRS_MODULE
#include "utilities.h"
#endif
#include "rtc_utils.h"
#include "log.h"

#ifdef SD_MODULE
#include <SdFat.h>
#endif

/* ------------------------ External Objects from main.cpp ------------------- */

#ifdef SD_MODULE
extern SdFat     SD;
extern SdFile    sdFile;
extern SPIClass  sdSPI;
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
  pinMode(SD_EN_PIN, OUTPUT);
  digitalWrite(SD_EN_PIN, HIGH);  // active-low disable
  gpio_hold_en((gpio_num_t)SD_EN_PIN);
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
  gpio_hold_dis((gpio_num_t)SD_EN_PIN);
  pinMode(SD_EN_PIN, OUTPUT);
  digitalWrite(SD_EN_PIN, LOW);   // active-low enable
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

  sdBusSuspend();

  digitalWrite(LED_PIN, LOW);

  // Lock sensor power/control pins LOW through deep sleep.
  // ESP32 retains OUTPUT state during deep sleep — setting pins to
  // OUTPUT + LOW is sufficient for most scenarios.  For pins that
  // need extra protection against leakage we additionally enable
  // the GPIO hold function via gpio_hold_en (works on all GPIOs,
  // not just RTC-capable ones).
  #ifdef SENSOR_SOIL_3TIP
  pinMode(SOIL_PWR_PIN, OUTPUT);
  digitalWrite(SOIL_PWR_PIN, LOW);
  gpio_hold_en((gpio_num_t)SOIL_PWR_PIN);
  pinMode(SOIL_DE_RE_PIN, OUTPUT);
  digitalWrite(SOIL_DE_RE_PIN, LOW);
  gpio_hold_en((gpio_num_t)SOIL_DE_RE_PIN);
  pinMode(SOIL_RX_PIN, INPUT);
  pinMode(SOIL_TX_PIN, INPUT);
  #endif
  #ifdef SENSOR_RADAR
  pinMode(RADAR_PWR_PIN, OUTPUT);
  digitalWrite(RADAR_PWR_PIN, LOW);
  gpio_hold_en((gpio_num_t)RADAR_PWR_PIN);
  pinMode(RADAR_RX_PIN, INPUT);
  pinMode(RADAR_TX_PIN, INPUT);
  #endif
  #ifdef SENSOR_SOIL_5TIP
  pinMode(SOIL5_PWR_PIN, OUTPUT);
  digitalWrite(SOIL5_PWR_PIN, LOW);
  gpio_hold_en((gpio_num_t)SOIL5_PWR_PIN);
  pinMode(SOIL5_DE_RE_PIN, OUTPUT);
  digitalWrite(SOIL5_DE_RE_PIN, LOW);
  gpio_hold_en((gpio_num_t)SOIL5_DE_RE_PIN);
  pinMode(SOIL5_RX_PIN, INPUT);
  pinMode(SOIL5_TX_PIN, INPUT);
  #endif
  #ifdef SENSOR_MAXBOTIX_UART
  pinMode(MAXBOTIX_PWR_PIN, OUTPUT);
  digitalWrite(MAXBOTIX_PWR_PIN, LOW);
  gpio_hold_en((gpio_num_t)MAXBOTIX_PWR_PIN);
  pinMode(MAXBOTIX_RX_PIN, INPUT);
  pinMode(MAXBOTIX_TX_PIN, INPUT);
  #endif
  #if defined(SENSOR_WATER_TEMP) || defined(SENSOR_MLX90614)
  #if WATER_TEMP_PWR_PIN != -1
  pinMode(WATER_TEMP_PWR_PIN, OUTPUT);
  digitalWrite(WATER_TEMP_PWR_PIN, LOW);
  gpio_hold_en((gpio_num_t)WATER_TEMP_PWR_PIN);
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
  pinMode(WAKEUP_PIN, INPUT);

  configureAlarm(interval);

  esp_sleep_enable_ext1_wakeup(WAKEUP_PIN_BITMASK, ESP_EXT1_WAKEUP_ALL_LOW);

  // Double blink LED to visually signal transition to deep sleep
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); delay(150);
  digitalWrite(LED_PIN, LOW);  delay(150);
  digitalWrite(LED_PIN, HIGH); delay(150);
  digitalWrite(LED_PIN, LOW);  delay(150);

  Serial.flush();

  esp_deep_sleep_start();
}
