/**
 * @file modem_mqtt.h
 * @brief Modem (SIM A7670) and MQTT management utilities.
 *
 * Provides:
 *  - Modem initialization and GPRS attach
 *  - MQTT connection handling
 *  - ThingsBoard RPC processing
 *  - JSON publishing helpers
 *
 * All declarations are guarded with #ifdef GPRS_MODULE.
 * When GPRS_MODULE is not defined, stub implementations are used.
 */

#pragma once
#include <Arduino.h>

#ifdef GPRS_MODULE

/**
 * @brief Initializes the A7670 modem, powers it, configures network,
 *        and attempts to connect to MQTT.
 *
 * Must be called during GPRS time window.
 * @return true if modem initialized and MQTT connected, false otherwise.
 */
bool initA7670();

/**
 * @brief Puts the A7670 modem into low-power sleep mode.
 *
 * Handles DTR pull-up, GPIO hold, and modem AT sleep command.
 * Used before ESP32 deep-sleep.
 */
void sleepA7670();

/**
 * @brief Attempts to establish MQTT connectivity with retry logic.
 *
 * @param maxAttempts Maximum number of connection attempts (default: 3).
 * @param delayMs Delay in milliseconds between attempts (default: 2000).
 * @return true if connected successfully, false if all attempts fail.
 *
 * Automatically subscribes to ThingsBoard RPC topic on success.
 */
bool connectMQTT(uint8_t maxAttempts = 3, uint16_t delayMs = 2000);

/**
 * @brief Processes all queued RPC commands.
 *
 * Parses the RPC payloads stored from `onMqttMessage()`, updates:
 *  - measure_time
 *  - bluetooth_time
 *  - GPRS_time
 *  - RTC date/time
 *
 * Writes updated configuration to EEPROM.
 */
void processQueuedRpcCommands();

/**
 * @brief Publishes a JSON line to ThingsBoard using MQTT.
 *
 * @param jsonOut String with JSON payload.
 * @return true if published successfully.
 */
bool publishJsonLineToTB(const String& jsonOut);

#else // !GPRS_MODULE

/* Stub implementations for when GPRS module is disabled */

static inline bool initA7670() {
  return false;
}
static inline void sleepA7670() {}
static inline bool connectMQTT(uint8_t maxAttempts = 3, uint16_t delayMs = 2000) {
  return false;
}
static inline void processQueuedRpcCommands() {}
static inline bool publishJsonLineToTB(const String& jsonOut) {
  return false;
}

#endif // GPRS_MODULE