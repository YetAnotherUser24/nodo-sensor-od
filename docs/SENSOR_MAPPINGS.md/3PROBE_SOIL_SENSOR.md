# 3-Probe Soil Sensor - Measurements

## Overview
Outputs humidity (soil moisture) and temperature only.

| Register | PLC Address | Parameter | Scale | Current Raw | Current Value |
|----------|-------------|-----------|-------|------------|---------------|
| 0x0000 | 40001 | Moisture | ÷10 | 0 | 0.0 %RH |
| 0x0001 | 40002 | Temperature | ÷10 | 315 | 31.5 °C |

## Detailed Mappings

### Moisture (Register 0x0000)
- **Measurement**: Soil Moisture
- **Access**: Read-only (FC03)
- **Raw Format**: Unsigned 16-bit integer
- **Scale**: divide by 10
- **Formula**: `moisture_percent = raw / 10.0`
- **Units**: %RH (percent relative humidity)
- **Source**: Datasheet map `0000H = moisture content (x10)`

### Temperature (Register 0x0001)
- **Measurement**: Soil Temperature
- **Access**: Read-only (FC03)
- **Raw Format**: Signed 16-bit integer (two's complement for negatives)
- **Scale**: divide by 10
- **Formula**: `temperature_c = signed_raw / 10.0`
- **Units**: °C (Celsius)
- **Source**: Datasheet map `0001H = temperature value (x10)`

## Protocol
- **Interface**: Modbus-RTU over RS485
- **Function Code**: FC03 (Read Holding Registers)
- **Default Serial**: 4800 baud, 8 data bits, no parity, 1 stop bit
- **Default Slave Address**: 1 (or use 0xFF for broadcast discovery)