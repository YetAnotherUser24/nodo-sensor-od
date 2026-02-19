# BLE Command Guide

This document defines the complete BLE command protocol used to configure and
interact with the **LilyGO A7670SA Water Level Telemetry System**.  
All commands are exchanged through a single BLE characteristic in plain text.

---

# 1. Overview

BLE is used for:

- Setting measurement intervals  
- Setting BLE and GPRS windows  
- Setting RTC date/time  
- Forcing manual upload  
- Live sensor streaming  
- Reading current configuration  

BLE activates only when requested and shuts down automatically to save power.

---

# 2. BLE Device Information

### Device Name
```
LILYGO_A7670_BLE
```

### Service UUID
```
6E400001-B5A3-F393-E0A9-E50E24DCCA9E
```

### RX Characteristic UUID (Write)
```
6E400002-B5A3-F393-E0A9-E50E24DCCA9E
```

### TX Characteristic UUID (Notify)
```
6E400003-B5A3-F393-E0A9-E50E24DCCA9E
```

---

# 3. Command Format

All commands follow the syntax:

```
<COMMAND>-<ARGS>
```

Commands are case-sensitive.

Examples:
```
T-5-1-10
D-2025-11-28-23-38-00
SEND
STOP
```

---

# 4. Timing Configuration Command

### Format
```
T-<MEASURE_INTERVAL>-<BLE_WINDOW>-<GPRS_WINDOW>
```

### Parameters

| Field | Meaning |
|-------|---------|
| MEASURE_INTERVAL | minutes between measurements |
| BLE_WINDOW | minutes BLE stays active after a command session |
| GPRS_WINDOW | minutes between modem upload attempts |

### Example
```
T-5-1-10
```

Meaning:
- Take one measurement every **5 minutes**  
- Keep BLE active for **1 minute**  
- Connect modem every **10 minutes** to upload data  

---

# 5. RTC Time‑Setting Command

### Format
```
D-YYYY-MM-DD-HH-MM-SS
```

### Example
```
D-2025-11-28-23-38-00
```

On success, BLE returns:
```
RTC SET OK
```

The RTC time is used for:
- EEPROM timestamps  
- CSV naming  
- MQTT payload timestamps  

---

# 6. Manual Upload Commands

### Upload pending files to cloud:
```
SEND
```

Triggers:
- Start sending measurements to device  

### Cancel ongoing upload:
```
STOP
```

Stops:
- Stop sending measurements to device  

---
# 7. BLE Troubleshooting

### 1. Device does not appear in scan  
✔ Ensure BLE window is open  
✔ Ensure board is not sleeping  
✔ Check LED indicator (steady ON = waiting for BLE)

### 2. Commands not accepted  
✔ Ensure correct format  
✔ Check MTU settings in your BLE app  

### 3. Streaming stops abruptly  
✔ BLE window expired  
✔ Weak signal  
✔ Device entered measurement or upload cycle  

---

# 8. Related Documentation

- **ARCHITECTURE.md** – how BLE integrates with whole system  
- **INSTALLATION.md** – setup and building  
- **HARDWARE_SETUP.md** – wiring and board modification  
- **EEPROM_SD_FORMAT.md** – data storage formats  
- **MQTT_RPC_GUIDE.md** – cloud integrations