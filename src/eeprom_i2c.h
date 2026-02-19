/**
 * @file eeprom_i2c.h
 * @brief I2C EEPROM utilities: page read/write, byte access and header reset.
 *
 * These functions provide low-level EEPROM interaction for external I2C chips
 * (e.g., 24LC256/24LC512). No buffering or logic is modified; only documentation
 * and formatting were improved.
 */

#pragma once
#include <Arduino.h>

/**
 * @brief Writes a full page into I2C EEPROM.
 *
 * @param deviceaddress I2C address of EEPROM (e.g., 0x50).
 * @param eeaddresspage Absolute EEPROM address to start writing.
 * @param data Pointer to data buffer to write.
 * @param length Number of bytes to write (typically page size).
 */
void i2c_eeprom_write_page(int deviceaddress,
                           unsigned int eeaddresspage,
                           byte* data,
                           byte length);

/**
 * @brief Reads a single byte from I2C EEPROM.
 *
 * @param deviceaddress I2C device address.
 * @param eeaddress Absolute memory address to read.
 * @return Read byte value.
 */
byte i2c_eeprom_read_byte(int deviceaddress, uint16_t eeaddress);

/**
 * @brief Writes a single byte into I2C EEPROM.
 *
 * @param deviceaddress I2C device address.
 * @param eeaddress Absolute memory address to write.
 * @param data Byte value to store.
 */
void i2c_eeprom_write_byte(int deviceaddress, uint16_t eeaddress, byte data);

/**
 * @brief Clears the EEPROM header (first pages) by writing zeros.
 *
 * @param deviceaddress I2C device address of EEPROM.
 */
void resetEEPromHeader(int deviceaddress);

/**
 * @brief Reads multiple bytes into a buffer.
 *
 * @param deviceaddress I2C address of EEPROM.
 * @param eeaddress Start address for read operation.
 * @param buffer Output buffer to store bytes.
 * @param length Number of bytes to read.
 */
void i2c_eeprom_read_buffer(int deviceaddress,
                            uint16_t eeaddress,
                            byte *buffer,
                            int length);
