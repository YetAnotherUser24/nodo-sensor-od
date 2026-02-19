/**
 * @file rtc_utils.h
 * @brief RTC formatting and DS3231 alarm configuration helpers.
 *
 * Provides:
 *  - Human-readable timestamp formatting into the global `dateStr` buffer.
 *  - DS3231 Alarm2 setup for waking the ESP32 after a minutes-based interval.
 */

#pragma once
#include <Arduino.h>
#include <RtcDS3231.h>

/**
 * @brief Formats an RTC timestamp into the global `dateStr` buffer.
 *
 * Format: "YYYY/MM/DD HH:MM:SS"
 *
 * @param dt RtcDateTime structure to format.
 */
void formatDateTime(const RtcDateTime& dt);

/**
 * @brief Configures DS3231 Alarm2 to wake the system in a given number of minutes.
 *
 * Alarm2 uses minute-match mode, waking on the next aligned interval.
 *
 * @param minutes Number of minutes until next wake. Must be >0.
 */
void configureAlarm(int minutes);