/**
 * @file error_sensor.h
 * @brief Error signaling and interrupt service routine utilities.
 *
 * Provides:
 *  - A configurable LED error blink pattern.
 *  - An ISR for external interrupt counting.
 */

#pragma once
#include <Arduino.h>

/**
 * @brief Displays an error pattern through LED blinking.
 *
 * Produces `errorNum` quick blinks, repeated in blocks, separated by delays.
 * This function is blocking and meant only for fault signaling.
 *
 * @param errorNum Number of blink pulses (1–10).
 * @param errpin   GPIO pin connected to LED.
 */
void error(int errorNum, int errpin);

/**
 * @brief Interrupt Service Routine for external event counting.
 *
 * Increments interrupt counters and sets a flag.
 * Must be registered through `attachInterrupt()`.
 */
void IRAM_ATTR InterruptServiceRoutine();