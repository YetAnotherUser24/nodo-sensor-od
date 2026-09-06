/**
 * @file wifi_http.h
 * @brief WiFi connectivity and HTTPS telemetry upload to AquaControl dashboard.
 *
 * Guarded by #ifdef WIFI_MODULE — disabled when WiFi feature is toggled off.
 */

#pragma once
#include <Arduino.h>

#ifdef WIFI_MODULE

/**
 * @brief Connects to WiFi using credentials from config.h.
 * @return true if connected, false on timeout.
 */
bool wifiConnect();

/**
 * @brief Disconnects WiFi cleanly before deep sleep.
 */
void wifiDisconnect();

/**
 * @brief Posts a single sensor reading to AquaControl via HTTPS.
 *
 * Builds the flat JSON the /api/telemetry/sensor endpoint expects
 * from the raw sensorReadings array.
 *
 * @param seconds2000  Timestamp as seconds since 2000-01-01.
 * @param readings     Array of MAX_SENSOR_READINGS int16_t values.
 * @param rtcTempCenti RTC temperature in centi-°C.
 * @param status       32-bit status bitmask.
 * @return true if HTTP 200 received, false otherwise.
 */
bool publishToAquaControl(uint32_t seconds2000, int16_t* readings,
                          int16_t rtcTempCenti, uint32_t status);

/**
 * @brief Synchronizes ESP32 internal clock via WiFi NTP, updating rtcNow and DS3231 if present.
 * @param timeoutMs Maximum milliseconds to wait for NTP sync (default 5000).
 * @return true if time is synchronized, false otherwise.
 */
bool syncTimeNtp(uint32_t timeoutMs = 5000);

#else

static inline bool wifiConnect() { return false; }
static inline void wifiDisconnect() {}
static inline bool syncTimeNtp(uint32_t = 0) { return false; }
static inline bool publishToAquaControl(uint32_t, int16_t*, int16_t, uint32_t) {
  return false;
}

#endif // WIFI_MODULE
