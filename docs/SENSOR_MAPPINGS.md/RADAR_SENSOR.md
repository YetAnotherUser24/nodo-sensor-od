# Radar Sensor - Measurements

## Overview
Outputs empty height (distance from radar to water surface) and water level (calculated from installation height minus empty height).

| Register | PLC Address | Parameter | Scale | Current Raw | Current Value |
|----------|-------------|-----------|-------|------------|---------------|
| 0x0001 | 40002 | Empty Height | direct | 0 | 0 mm |
| 0x0003 | 40004 | Water Level | direct | 0 | 0 mm |

## Detailed Mappings

### Empty Height (Register 0x0001)
- **Measurement**: Distance from radar to water surface
- **Access**: Read-only (FC03)
- **Raw Format**: Unsigned 16-bit integer
- **Scale**: direct (no scaling)
- **Formula**: `empty_height_mm = raw`
- **Units**: mm (millimeters)
- **Notes**: Filtered data
- **Source**: Datasheet map `0001H = empty height`

### Water Level (Register 0x0003)
- **Measurement**: Water level relative to installation reference point
- **Access**: Read-only (FC03)
- **Raw Format**: Unsigned 16-bit integer
- **Scale**: direct (no scaling)
- **Formula**: `water_level_mm = raw`
- **Units**: mm (millimeters)
- **Notes**: Filtered data. Water level equals installation height minus empty height. The installation height must be set first (see Configuration).
- **Source**: Datasheet map `0003H = water level`

## Protocol
- **Interface**: Modbus-RTU over RS485
- **Function Code**: FC03 (Read Holding Registers)
- **Data Format**: 8N1
- **Validation**: CRC16 (polynomial A001)
- **Default Device Address**: 1
- **Default Serial**: See Baud Rate Configuration below

---

# Radar Sensor - Configuration

## Overview
Configuration registers for the Radar Sensor. These are modifiable settings that control installation height, device address, baud rate, and measurement range.

| Register | PLC Address | Parameter | Range | Factory Default | Current |
|----------|-------------|-----------|-------|-----------------|---------|
| 0x0005 | 40006 | Installation Height | 0–65535 | Not set | 0 cm |
| 0x03F4 | 41013 | Device Address | 1–253 (0xFD) | 1 | 1 |
| 0x03F6 | 41015 | Baud Rate Code | 48, 96, 144, 192, 384, 560, 576, 1152, 1290 | Not specified | 96 (9600) |
| 0x07D4 | 42005 | Range | Product-dependent | 10 (0x0A) | 10 m |

## Detailed Configuration Registers

### Installation Height (Register 0x0005)
- **Function**: Sets the distance from the radar sensor to the water bottom (zero reference)
- **Access**: Read/Write (FC03 to read, FC06 to write)
- **Raw Format**: Unsigned 16-bit integer
- **Scale**: direct (no scaling)
- **Formula**: `installation_height_cm = raw`
- **Units**: cm (centimeters)
- **Notes**:
  - Water level is calculated as: `water_level = installation_height - empty_height` (consistent units)
  - Must be set before water level readings are meaningful
  - Source: Datasheet map `0005H = installation height`

**Read Command:** `01 03 00 05 00 01 94 0B`
**Write Command:** `01 06 00 05 [AA] [BB] [CC] [DD]`

### Device Address (Register 0x03F4)
- **Function**: Sets the Modbus slave address for this sensor on the RS485 bus
- **Access**: Read/Write (FC03 to read, FC06 to write)
- **Raw Format**: Unsigned 16-bit integer
- **Range**: 1–253 (0x01–0xFD). Broadcast address 0xFF is supported for discovery.
- **Factory Default**: 1
- **Units**: Modbus slave address
- **Source**: Datasheet map `03F4H = device address`

**Read Command:** `01 03 03 F4 00 01 C5 BC`
**Write Command:** `01 06 03 F4 [AA] [BB] [CC] [DD]`

### Baud Rate (Register 0x03F6)
- **Function**: Sets the serial communication baud rate
- **Access**: Read/Write (FC03 to read, FC06 to write)
- **Raw Format**: Unsigned 16-bit integer
- **Baud Rate Mapping**:

  | Code | Actual Baud Rate |
  |------|-----------------|
  | 48   | 4800            |
  | 96   | 9600            |
  | 144  | 14400           |
  | 192  | 19200           |
  | 384  | 38400           |
  | 560  | 56000           |
  | 576  | 57600           |
  | 1152 | 115200          |
  | 1290 | 129000          |

- **Factory Default**: Not specified (check product datasheet)
- **Units**: Baud code (actual baud rate = code × 100)
- **Notes**:
  - After changing baud rate, the device will switch to the new rate immediately
  - Host must match the new baud rate to continue communication
  - Source: Datasheet map `03F6H = baud rate adjustment`

**Read Command:** `01 03 03 F6 00 01 64 7C`
**Write Command:** `01 06 03 F6 [AA] [BB] [CC] [DD]`

### Range (Register 0x07D4)
- **Function**: Sets the maximum measurement range of the radar sensor
- **Access**: Read/Write (FC03 to read, FC06 to write)
- **Raw Format**: Unsigned 16-bit integer
- **Factory Default**: 10 (0x0A)
- **Units**: m (meters)
- **Notes**:
  - Maximum range depends on the product model
  - Must be set appropriately for the installation environment
  - Source: Datasheet map `07D4H = range`

**Read Command:** `01 03 07 D4 00 01 C5 46`
**Write Command:** `01 06 07 D4 00 [AA] [BB] [CC]`

## Frame Structure

All Modbus frames follow this structure:

| Byte(s) | Description |
|---------|-------------|
| 1st byte | Device address |
| 2nd byte | Function code (`0x03` = read, `0x06` = write) |
| 3rd–4th bytes | Register address |
| 5th–6th bytes | **Read:** Number of registers to read / **Write:** Data to write |
| 7th+ bytes | CRC16 check (polynomial A001) |

## Commissioning Procedure

### Setting Installation Height
1. Read current empty height (register 0x0001) to measure distance to water surface
2. Set installation height (register 0x0005) using FC06 write
3. Verify water level (register 0x0003) returns correct value

### Changing Device Address and Baud Rate
1. Connect to the radar sensor using the known current address and baud rate
2. Write the new device address to register 0x03F4 using FC06
3. Write the new baud rate code to register 0x03F6 using FC06
4. Update the host UART to match the new baud rate
5. Verify communication at the new address and baud rate

### Field Search Mode
To open field search mode (for discovery on the bus), send:
```text
01 06 07 DB 00 05 38 86
```
This writes to register 0x07DB (field search mode control).

## Factory Reset

To restore factory defaults:
- **Address**: Write address = 1 to register 0x03F4
- **Baud**: Write baud code = 96 (9600) to register 0x03F6
- **Range**: Write range = 10 (0x0A) to register 0x07D4