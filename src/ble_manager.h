/**
 * @file ble_manager.h
 * @brief BLE control interface for LilyGO A7670SA.
 *
 * Provides APIs for starting/stopping BLE, handling incoming BLE commands,
 * and transparently mirroring Serial output over BLE notifications (streaming).
 *
 * WARNING: This header pulls in BLE library dependencies.
 * For lighter-weight access to Log()/Logln()/Logf() use log.h instead.
 */

#pragma once
#include <Arduino.h>
#include "log.h"

/** Indicates if a BLE client is currently connected. */
extern bool bleDeviceConnected;

/**
 * @brief Flag persisted across deep sleep (RTC memory).
 * When true, all Log()/Logln()/Logf() output is also
 * pushed to the connected BLE client via notify().
 */
extern bool bleStreamingActive;

/** Set by BLE disconnect callback when advertising needs to be safely restarted from loop(). */
extern bool bleRestartAdvertisingRequested;

/**
 * @brief Starts BLE stack, initializes advertising and enables write/notify characteristic.
 */
void startBLE();

/**
 * @brief Fully disables BLE stack and stops advertising.
 */
void stopBLE();

/**
 * @brief Processes a command received via BLE characteristic.
 *
 * Commands include:
 *  - T-<measure>-<ble>-<gprs> : Update module timing configuration.
 *  - D-YYYY-MM-DD-HH-MM-SS   : Set RTC date and time.
 *  - START / STOP             : Enable/disable Serial-to-BLE streaming.
 *  - DATA                     : Reserved.
 *
 * @param cmd Command string received over BLE.
 */
void processCommand(String cmd);

