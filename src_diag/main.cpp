/**
 * @file main.cpp
 * @brief Diagnostic firmware — tests basic Serial + deep sleep cycle.
 *
 * No sensors, no I2C, no EEPROM, no SD.
 * Prints boot cause, sleeps 30s via timer wakeup, repeats.
 */
#include <Arduino.h>
#include <esp_sleep.h>

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== DIAG: BOOT OK ===");
  Serial.print("Wakeup cause: ");
  switch (esp_sleep_get_wakeup_cause()) {
    case ESP_SLEEP_WAKEUP_EXT0:  Serial.println("EXT0"); break;
    case ESP_SLEEP_WAKEUP_EXT1:  Serial.println("EXT1"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("TIMER"); break;
    default:                     Serial.println("POWERON/RESET"); break;
  }

  Serial.println("Sleeping for 30 seconds...");
  Serial.flush();
  delay(100);

  esp_sleep_enable_timer_wakeup(30 * 1000000ULL);
  esp_deep_sleep_start();
}

void loop() {}