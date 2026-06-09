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
#include <SPI.h>

#include "n_logger_config.h"
#include "utilities.h"
#include "rtc_utils.h"

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
  digitalWrite(SD_PIN_CS, HIGH);   // De-select card

  pinMode(SD_PIN_SCLK, INPUT);
  pinMode(SD_PIN_MOSI, INPUT);
  pinMode(SD_PIN_MISO, INPUT);
#else
  // No SD module — no-op
#endif
}

/**
 * @brief Resumes the SD bus after waking from deep sleep.
 */
void sdBusResume() {
#ifdef SD_MODULE
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

#ifdef DEBUG
  Serial.println();
  Serial.print("WAKE UP IN ");
  Serial.print(moduleCfg.measure_time);
  Serial.println(" MIN");
#endif

  configureAlarm(interval);

  esp_sleep_enable_ext1_wakeup(WAKEUP_PIN_BITMASK, ESP_EXT1_WAKEUP_ALL_LOW);

  esp_deep_sleep_start();
}