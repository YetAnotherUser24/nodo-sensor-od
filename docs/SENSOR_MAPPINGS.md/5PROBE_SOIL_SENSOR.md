# 5-Probe Soil Sensor - Measurements

## Overview
Outputs all 7 measurements: moisture, temperature, conductivity (EC), pH, nitrogen (N), phosphorus (P), and potassium (K).

| Register | PLC Address | Parameter | Scale | Current Raw | Current Value |
|----------|-------------|-----------|-------|------------|---------------|
| 0x0000 | 40001 | Moisture | ÷10 | 0 | 0.0 %RH |
| 0x0001 | 40002 | Temperature | ÷10 | 315 | 31.5 °C |
| 0x0002 | 40003 | Conductivity (EC) | direct | 0 | 0 µS/cm |
| 0x0003 | 40004 | pH | ÷10 | 30 | 3.0 pH |
| 0x0004 | 40005 | Nitrogen (N) | direct | 0 | 0 mg/kg |
| 0x0005 | 40006 | Phosphorus (P) | direct | 0 | 0 mg/kg |
| 0x0006 | 40007 | Potassium (K) | direct | 0 | 0 mg/kg |

## Detailed Mappings

### Moisture (Register 0x0000)
- **Measurement**: Soil Moisture
- **Access**: Read-only (FC03)
- **Raw Format**: Unsigned 16-bit integer
- **Scale**: divide by 10
- **Formula**: `moisture_percent = raw / 10.0`
- **Units**: %RH (percent relative humidity)
- **Range**: 0–100%
- **Source**: Datasheet map `0000H = moisture content (x10)`

### Temperature (Register 0x0001)
- **Measurement**: Soil Temperature
- **Access**: Read-only (FC03)
- **Raw Format**: Signed 16-bit integer (two's complement for negatives)
- **Scale**: divide by 10
- **Formula**: `temperature_c = signed_raw / 10.0`
- **Units**: °C (Celsius)
- **Range**: -40 to +80 °C
- **Source**: Datasheet map `0001H = temperature value (x10)`

### Conductivity (Register 0x0002)
- **Measurement**: Electrical Conductivity (EC)
- **Access**: Read-only (FC03)
- **Raw Format**: Unsigned 16-bit integer
- **Scale**: direct (no scaling)
- **Formula**: `ec_us_cm = raw`
- **Units**: µS/cm (microSiemens per centimeter)
- **Range**: 0–20000 µS/cm
- **Source**: Datasheet map `0002H = conductivity real-time value`

### pH (Register 0x0003)
- **Measurement**: Soil pH
- **Access**: Read-only (FC03)
- **Raw Format**: Unsigned 16-bit integer
- **Scale**: divide by 10
- **Formula**: `ph_value = raw / 10.0`
- **Units**: pH scale
- **Range**: 3–9 pH
- **Source**: Datasheet map `0003H = PH value (x10)`

### Nitrogen (Register 0x0004)
- **Measurement**: Nitrogen Content (N)
- **Access**: Read-only (FC03)
- **Raw Format**: Unsigned 16-bit integer
- **Scale**: direct (no scaling)
- **Formula**: `nitrogen_mgkg = raw`
- **Units**: mg/kg (milligrams per kilogram)
- **Range**: 1–1999 mg/kg
- **Source**: Datasheet map `0004H = nitrogen content actual value`

### Phosphorus (Register 0x0005)
- **Measurement**: Phosphorus Content (P)
- **Access**: Read-only (FC03)
- **Raw Format**: Unsigned 16-bit integer
- **Scale**: direct (no scaling)
- **Formula**: `phosphorus_mgkg = raw`
- **Units**: mg/kg (milligrams per kilogram)
- **Range**: 1–1999 mg/kg
- **Source**: Datasheet map `0005H = phosphorus content actual value`

### Potassium (Register 0x0006)
- **Measurement**: Potassium Content (K)
- **Access**: Read-only (FC03)
- **Raw Format**: Unsigned 16-bit integer
- **Scale**: direct (no scaling)
- **Formula**: `potassium_mgkg = raw`
- **Units**: mg/kg (milligrams per kilogram)
- **Range**: 1–1999 mg/kg
- **Source**: Datasheet map `0006H = potassium content actual value`

## Protocol
- **Interface**: Modbus-RTU over RS485
- **Function Code**: FC03 (Read Holding Registers)
- **Default Serial**: 4800 baud, 8 data bits, no parity, 1 stop bit
- **Default Slave Address**: 1 (or use 0xFF for broadcast discovery)

## Commissioning
To set device address to 2 and baud rate to 9600:
- Use the `c` command in `modbus_rs485_scope_debug`
- Config registers: 0x07D0 (address), 0x07D1 (baud code)
- Target: address = 2, baud code = 2 (9600)

# 5-Probe Soil Sensor - Configuration

## Overview
Configuration registers for the 5-Probe Soil Sensor. These are modifiable settings that control device address and serial baud rate.

| Register | PLC Address | Parameter | Range | Factory Default | Current |
|----------|-------------|-----------|-------|-----------------|---------|
| 0x07D0 | 42001 | Device Address | 1–254 | 1 | 1 |
| 0x07D1 | 42002 | Baud Rate Code | 0, 1, 2 | 1 | 1 |

## Detailed Configuration Registers

### Device Address (Register 0x07D0)
- **Function**: Sets the Modbus slave address for this sensor on the RS485 bus
- **Access**: Read/Write (FC03 to read, FC06 to write)
- **Raw Format**: Unsigned 16-bit integer
- **Range**: 1–254
- **Factory Default**: 1
- **Units**: Modbus slave address (1-indexed)
- **Notes**:
  - Use broadcast address 0xFF to discover or query a sensor when only one device is on the bus
  - After changing address, the device will respond with its new address on subsequent queries
  - Source: Datasheet `07D0H = device address (1–254, default 1)`

### Baud Rate Code (Register 0x07D1)
- **Function**: Sets the serial communication baud rate
- **Access**: Read/Write (FC03 to read, FC06 to write)
- **Raw Format**: Unsigned 16-bit integer
- **Baud Code Mapping**:
  | Code | Baud Rate |
  |------|-----------|
  | 0 | 2400 |
  | 1 | 4800 |
  | 2 | 9600 |
- **Factory Default**: 1 (4800 baud)
- **Units**: Baud code
- **Notes**:
  - After changing baud rate, the device will switch to the new rate immediately
  - Host must match the new baud rate to continue communication
  - Source: Datasheet `07D1H = device baud rate (0=2400, 1=4800, 2=9600)`

## Commissioning Procedure

### Using modbus_rs485_scope_debug

1. **Connect** the 5-Probe Soil Sensor to the RS485 bus (only one sensor)
2. **Upload and Monitor** the `modbus_rs485_scope_debug` environment
3. **Send command** `c` in Serial Monitor
4. The device will:
   - Query current address and baud code (broadcasts as 0xFF)
   - Report responder ID and current settings
   - Set address to 2 (if not already set)
   - Set baud code to 2 (9600)
   - Reopen UART to 9600 for continued communication

### Manual Procedure (Using FC06)

1. **Broadcast Query** (address 0xFF):
   - Read 0x07D0–0x07D1 to discover current settings
   - Note the responder's actual address and baud code

2. **Write New Address** (if changing):
   - Use function code 0x06 (Write Single Register)
   - Target register: 0x07D0
   - New value: desired address (2–254)
   - Acknowledge echoed response

3. **Write New Baud Code** (if changing):
   - Use function code 0x06
   - Target register: 0x07D1
   - New value: baud code (0, 1, or 2)
   - Switch host UART to new baud rate immediately after write

4. **Verify** with a new query at the new address and baud rate

## Factory Reset

To restore factory defaults:
- Address: Send address = 1 to register 0x07D0
- Baud: Send baud code = 1 to register 0x07D1 (4800 baud)