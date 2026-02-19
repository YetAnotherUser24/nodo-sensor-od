/**
 * @file eeprom_i2c.cpp
 * @brief Implementation of basic I2C EEPROM operations.
 *
 * Contains byte-level and page-level EEPROM read/write helpers used throughout
 * the storage pipeline of the device. Logic remains exactly as original.
 */

#include "eeprom_i2c.h"
#include <Wire.h>
#include "utilities.h"

#ifndef EEPROM_HEADER_SIZE
  #define EEPROM_HEADER_SIZE 8
#endif

#ifndef EEPROM_PAGE_SIZE
  #define EEPROM_PAGE_SIZE 32
#endif

/**
 * @brief Writes a complete page to the I2C EEPROM.
 *
 * Performs a standard two-byte address write followed by sequential data bytes.
 * A mandatory write-cycle delay is included.
 *
 * @param deviceaddress EEPROM I2C address (e.g., 0x50).
 * @param eeaddresspage Absolute EEPROM page start address.
 * @param data Pointer to data buffer.
 * @param length Number of bytes to write.
 */
void i2c_eeprom_write_page(int deviceaddress,
                           unsigned int eeaddresspage,
                           byte* data,
                           byte length)
{
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddresspage >> 8));      // MSB of address
  Wire.write((int)(eeaddresspage & 0xFF));    // LSB of address

  for (byte c = 0; c < length; c++) {
    Wire.write(data[c]);
  }

  Wire.endTransmission();
  delay(5);   // EEPROM write cycle time
}

/**
 * @brief Reads a single byte from the EEPROM.
 *
 * @param deviceaddress EEPROM I2C address.
 * @param eeaddress Absolute address of byte to read.
 * @return The byte read from EEPROM.
 */
byte i2c_eeprom_read_byte(int deviceaddress, uint16_t eeaddress)
{
  byte rdata = 0xFF;

  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();

  Wire.requestFrom(deviceaddress, 1);
  if (Wire.available()) {
    rdata = Wire.read();
  }

  return rdata;
}

/**
 * @brief Writes a single byte into EEPROM.
 *
 * @param deviceaddress I2C address of EEPROM.
 * @param eeaddress Absolute memory address.
 * @param data Byte to write.
 */
void i2c_eeprom_write_byte(int deviceaddress, uint16_t eeaddress, byte data)
{
  Wire.beginTransmission(deviceaddress);
  Wire.write(eeaddress >> 8);        // MSB
  Wire.write(eeaddress & 0xFF);      // LSB
  Wire.write((int)data);
  Wire.endTransmission();
  delay(5);
}

/**
 * @brief Resets the EEPROM header region.
 *
 * Writes 0x00 over the number of bytes defined by:
 * (EEPROM_HEADER_SIZE * EEPROM_PAGE_SIZE)
 *
 * @param deviceaddress I2C device address.
 */
void resetEEPromHeader(int deviceaddress)
{
  uint16_t headerLen = EEPROM_HEADER_SIZE * EEPROM_PAGE_SIZE;

  for (uint16_t i = 0; i < headerLen; i++) {
    i2c_eeprom_write_byte(deviceaddress, i, 0x00);
  }
}

/**
 * @brief Reads a block of `length` bytes from EEPROM into a buffer.
 *
 * @param deviceaddress EEPROM I2C address.
 * @param eeaddress Start address for read.
 * @param buffer Output buffer to fill.
 * @param length Number of bytes to read.
 */
void i2c_eeprom_read_buffer(int deviceaddress,
                            uint16_t eeaddress,
                            byte *buffer,
                            int length)
{
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();

  Wire.requestFrom(deviceaddress, length);
  for (int c = 0; c < length; c++) {
    if (Wire.available()) {
      buffer[c] = Wire.read();
    }
  }
}
