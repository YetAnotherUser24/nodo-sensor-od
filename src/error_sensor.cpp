/**
 * @file error_sensor.cpp
 * @brief Implementation of LED error signaling and ISR handler.
 */

#include "error_sensor.h"
#include <Arduino.h>
#include "utilities.h"

/* ----------------------- External Runtime Flags ----------------------- */
/** Incremented each time the ISR triggers (declared in main project). */
extern volatile uint16_t interruptCount;

/** Flag set true on ISR trigger (declared in main project). */
extern volatile bool     interruptFlag;

/* ------------------------------ Error LED ----------------------------- */

/**
 * @brief Displays a repeated blink pattern to indicate error codes.
 *
 * The blink logic is identical to the original implementation:
 *  - Blink `errorNum` times (100 ms ON, 100 ms OFF).
 *  - Wait (200 ms) for the remaining of a 10-cycle frame.
 *  - Repeat the block 3 times.
 *
 * This is typically used only for diagnostic fault signaling.
 */
void error(int errorNum, int errpin) {
  for (int j = 0; j < 3; j++) {

    for (int i = 0; i < errorNum; i++) {
      digitalWrite(errpin, HIGH);
      delay(100);
      digitalWrite(errpin, LOW);
      delay(100);
    }

    for (int i = errorNum; i < 10; i++) {
      delay(200);
    }
  }
}

/* ----------------------------- ISR Handler ---------------------------- */

/**
 * @brief ISR for external sensor pulses or events.
 *
 * Increments `interruptCount` and sets `interruptFlag` so the main loop
 * can process the event outside interrupt context.
 *
 * This ISR must be attached with:
 * @code
 * attachInterrupt(digitalPinToInterrupt(pin), InterruptServiceRoutine, mode);
 * @endcode
 */
void IRAM_ATTR InterruptServiceRoutine() {
  interruptCount++;
  interruptFlag = true;
}
