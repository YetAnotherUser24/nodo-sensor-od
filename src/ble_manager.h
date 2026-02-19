/**
 * @file ble_manager.h
 * @brief BLE control interface for LilyGO A7670SA.
 *
 * Provides APIs for starting/stopping BLE and handling incoming BLE commands.
 */

#pragma once
#include <Arduino.h>

/** Indicates if a BLE client is currently connected. */
extern bool bleDeviceConnected;

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
 *  - SEND / STOP             : Enable/disable sensor streaming.
 *  - DATA                    : Reserved.
 *
 * @param cmd Command string received over BLE.
 */
void processCommand(String cmd);

