/**
 * @file power_sleep.cpp
 * @brief Implementation of low-power helpers for SD bus, radio shutdown,
 *        and deep-sleep transition.
 *
 * Only formatting and documentation were improved. No functional changes
 * were introduced.
 */

#include "power_sleep.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_bt.h>
#include <esp_sleep.h>
#include <SPI.h>
#include <SdFat.h>

#include "utilities.h"
#include "rtc_utils.h"

/* ---------------------------- SD Bus Pin Defaults ---------------------------- */
/* These defaults match the user’s main sketch; redefining here does not alter behavior. */

#ifndef SD_MOSI
  #define SD_MOSI 15
#endif
#ifndef SD_MISO
  #define SD_MISO 2
#endif
#ifndef SD_SCLK
  #define SD_SCLK 14
#endif
#ifndef SD_CS
  #define SD_CS   13
#endif

/* ----------------------------- Wakeup Pin Config ------------------------------ */

#ifndef WAKEUP_PIN
  #define WAKEUP_PIN GPIO_NUM_39
#endif
#ifndef WAKEUP_PIN_BITMASK
  #define WAKEUP_PIN_BITMASK (1ULL << WAKEUP_PIN)
#endif

/* ------------------------------- Sensor & LED -------------------------------- */

#ifndef SENSOR_PWR_PIN
  #define SENSOR_PWR_PIN 18
#endif

#ifndef LED_PIN
  #define LED_PIN 32
#endif

/* ------------------------ External Objects from Main Sketch ------------------- */

extern SdFat     SD;
extern SdFile    sdFile;
extern SPIClass  sdSPI;

extern HardwareSerial SensorSerial;

struct ModuleConfig {
  int measure_time;
  int bluetooth_time;
  int GPRS_time;
};
extern ModuleConfig moduleCfg;

/* ------------------------------ SD Power Helpers ------------------------------ */

/**
 * @brief Suspends the SD SPI bus to prevent leakage during deep sleep.
 */
void sdBusSuspend() {
  sdSPI.end();

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);   // De-select card

  pinMode(SD_SCLK, INPUT);
  pinMode(SD_MOSI, INPUT);
  pinMode(SD_MISO, INPUT);
}

/**
 * @brief Resumes the SD bus after waking from deep sleep.
 */
void sdBusResume() {
  sdSPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
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

  digitalWrite(SENSOR_PWR_PIN, LOW);
  SensorSerial.end();

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
