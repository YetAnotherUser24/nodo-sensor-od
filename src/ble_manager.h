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
#include "log.h"
#include <Arduino.h>

/** Indicates if a BLE client is currently connected. */
extern bool bleDeviceConnected;

/**
 * @brief Flag persisted across deep sleep (RTC memory).
 * When true, all Log()/Logln()/Logf() output is also
 * pushed to the connected BLE client via notify().
 */
extern bool bleStreamingActive;

/** Characteristic for real-time sensor NOTIFY output. */
class BLECharacteristic;
extern BLECharacteristic* pNotifyCharacteristic;

/** Flag indicating active real-time sensor streaming requested by "Send" command. */
extern bool bleRealtimeSendActive;

/** Set by BLE disconnect callback when advertising needs to be safely restarted from loop(). */
extern bool bleRestartAdvertisingRequested;

/** Set to true on first BLE connection; used to differentiate "waiting" vs "reconnection" log
 * messages. */
extern bool bleWasEverConnected;

/**
 * @brief Starts BLE stack, initializes advertising and enables write/notify characteristic.
 */
void startBLE();

/**
 * @brief Checks if BLE is connected and notifications are subscribed by client.
 */
bool isBleNotifyEnabled();

/**
 * @brief Fully disables BLE stack and stops advertising.
 */
void stopBLE();

/**
 * @brief Non-blocking LED blink state machine. Call from loop() to process
 *        acknowledgement blinks requested by processCommand().
 */
void bleProcessLedBlinks();

/**
 * @brief Processes a command received via BLE characteristic.
 *
 * Commands include:
 *  - T-<measure>-<ble>-<gprs> : Update module timing configuration.
 *  - D-YYYY-MM-DD-HH-MM-SS   : Set RTC date and time.
 *  - START / STOP             : Enable/disable Serial-to-BLE streaming.
 *  - CLEAR_EEPROM / CLEAR / C : Clear external I2C EEPROM storage contents.
 *  - PEEK                     : Print compact 1-line overview for all active EEPROM pages.
 *  - PEEK-<page>              : Print detailed record view for a specific EEPROM page index.
 *  - DATA                     : Reserved.
 *
 * @param cmd Command string received over BLE.
 */
void processCommand(const String& cmd);
