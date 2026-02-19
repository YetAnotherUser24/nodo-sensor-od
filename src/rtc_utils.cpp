/**
 * @file rtc_utils.cpp
 * @brief RTC formatting and DS3231 alarm configuration functions.
 *
 * Only formatting and documentation were improved; all functional logic
 * remains identical to the original implementation.
 */

#include "rtc_utils.h"

#include <Wire.h>
#include <RtcDS3231.h>
#include "utilities.h"

/* --------------------------------------------------------------------------
 *  External variables defined in main program
 * -------------------------------------------------------------------------- */

extern RtcDS3231<TwoWire> rtc;
extern RtcDateTime        rtcNow;
extern char               dateStr[20];

#ifndef countof
  #define countof(a) (sizeof(a) / sizeof(a[0]))
#endif

/* --------------------------------------------------------------------------
 *  Timestamp Formatting
 * -------------------------------------------------------------------------- */

/**
 * @brief Converts an RtcDateTime into a human-readable string.
 *
 * Format example: "2025/12/05 21:37:10"
 */
void formatDateTime(const RtcDateTime& dt) {
  snprintf_P(
    dateStr, countof(dateStr),
    PSTR("%04u/%02u/%02u %02u:%02u:%02u"),
    dt.Year(), dt.Month(), dt.Day(),
    dt.Hour(), dt.Minute(), dt.Second()
  );
}

/* --------------------------------------------------------------------------
 *  DS3231 Alarm Configuration
 * -------------------------------------------------------------------------- */

/**
 * @brief Sets DS3231 Alarm2 so the ESP32 will wake after a given number of minutes.
 *
 * Steps:
 *  - Reads current RTC time.
 *  - Computes new alarm timestamp.
 *  - Aligns alarm to the closest `minutes` interval.
 *  - Writes Alarm2 to DS3231.
 */
void configureAlarm(int minutes) {
  rtcNow = rtc.GetDateTime();

  RtcDateTime alarmTime = rtcNow + (minutes * 60);

  // Align the minute field to the configured interval
  int adjust = alarmTime.Minute() % minutes;

  DS3231AlarmTwo alarm2(
    0,
    0,
    alarmTime.Minute() - adjust,
    DS3231AlarmTwoControl_MinutesMatch
  );

  rtc.SetAlarmTwo(alarm2);

  // Clear any previous alarm flags
  rtc.LatchAlarmsTriggeredFlags();
}
