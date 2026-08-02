/**
 * @file sensor_engine.h
 * @brief Sensor abstraction layer for n-logger multi-sensor platform.
 *
 * Provides a single function that collects all sensor readings into the
 * TelemetryPayload. Each enabled sensor type is handled via #ifdef blocks
 * inside sensor_engine.cpp — no virtual functions, minimal public API.
 *
 * Supported sensor flags (define any combination in platformio.ini):
 *   -DSENSOR_MAXBOTIX_UART     = MaxBotix MB12xx ultrasonic (UART)
 *   -DSENSOR_WATER_TEMP        = Water surface temperature (I2C)
 *   -DSENSOR_RADAR             = Radar distance sensor (Modbus RTU)
 *   -DSENSOR_SOIL_3TIP         = 3-probe soil moisture/temperature (Modbus RTU)
 *   -DSENSOR_SOIL_5TIP         = 5-probe soil comprehensive (Modbus RTU)
 *   -DSENSOR_BATTERY_ADC       = Battery voltage via ADC
 *   -DSENSOR_INA226            = INA226 voltage/current monitor (I2C)
 */

#pragma once
#include <Arduino.h>
#include "n_logger_config.h"

/**
 * @brief Performs a full measurement cycle on all enabled sensors.
 *
 * For each sensor: powers on → waits warmup → reads N samples →
 * powers off. All readings are stored sequentially in tp.sensorReadings[].
 * Status bits are set on failure.
 *
 * @param tp Telemetry payload to fill with readings.
 */
void sensorReadAll(TelemetryPayload& tp);

#ifdef SENSOR_INA226
void ina226PowerDown();
#endif