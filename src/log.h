/**
 * @file log.h
 * @brief Standalone logging declarations for Serial and optional BLE streaming.
 *
 * Provides lightweight logging functions (Log/Logln/Logf) that always print
 * to Serial. When BLE streaming is active and a BLE client is connected,
 * output is also pushed via BLE Notify for real-time remote monitoring.
 *
 * Unlike ble_manager.h, this header has NO dependency on the BLE library.
 * Include it from any module that needs Log/Logln/Logf without pulling in
 * BLEDevice, BLEServer, etc.
 *
 * The implementations live in ble_manager.cpp.
 */

#pragma once
#include <Arduino.h>

/**
 * @brief Prints to Serial AND (when streaming is active and BLE connected)
 *        pushes via BLE notify(). Always sends to Serial.
 */
void Log(const char *s);

/**
 * @brief Same as Log() with a trailing newline.
 */
void Logln(const char *s);

/**
 * @brief printf-style output to Serial AND BLE notify().
 */
void Logf(const char *fmt, ...);