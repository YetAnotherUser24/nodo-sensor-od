# EEPROM & SD Storage Format

This document provides a complete specification of how measurements are stored
in **EEPROM** and **SD card CSV files** inside the  
**LilyGO A7670SA Water Level Telemetry System**.

It defines the exact byte layout, file rotation rules, sent-flag operations,
and the full lifecycle of measurement data.

---

# 1. Storage Philosophy

The system is designed for **offline-first reliability**.

1. **EEPROM** = short‑term binary buffer (fast writes, low power)  
2. **SD Card** = long‑term storage (CSV archives)  
3. **MQTT Upload** = cloud synchronization  

Data survives:
- network outages  
- power cycles  
- long sleep periods  
- modem failures  

---

# 2. EEPROM Binary Page Format (32 bytes)

Each measurement is stored in a **32‑byte fixed-size page**.

```
Offset | Size | Field
-------|------|-------------------------------
0      | 4    | timestamp (seconds since 2000)
4      | 10   | sensor samples (5 × int16)
14     | 2    | battery (mV)
16     | 2    | rtcTemp (centi °C)
18     | 4    | statusFlags (uint32)
22     | 9    | reserved (future use)
31     | 1    | sentFlag (0 = pending, 1 = uploaded)
```

### 2.1 Timestamp Format  
A 32‑bit integer representing:

```
seconds since 2000-01-01 00:00:00
```

This avoids leap-year problems and is compact.

### 2.2 Sensor Samples  
Five measurements from the water level sensor:

```
int16_t sample[5]
```

If a sample fails, value is set to `-1`.

### 2.3 Battery  
Raw ADC result converted to **millivolts**.

### 2.4 RTC Temperature  
Reported in **hundredths of degrees Celsius**.

Example:
```
2450 → 24.50 °C
```

### 2.5 Status Flags  
32-bit field containing diagnostic bits:

| Bit | Meaning |
|-----|---------|
| 0 | Internet failure |
| 1 | Full Memory|
| 2 | Corrupted Memory |
| 3 | Code Error |
| 4 | RTC connection |
| … | (extendable) |
| 16 | Water level sensor failure |
| 17 | SD card failure |
| 18 | Temperature sensor failure |
| 19 | GRPS connection failure |
| … | (extendable) |

### 2.6 Sent Flag  
Set to `1` after successful MQTT publish.

---

# 3. EEPROM Header Structure

The first 32 bytes of EEPROM contain header metadata:

```
Offset | Size | Field
-------|------|-----------------------
0      | 4    | numberOfPagesWritten
4      | 4    | numberOfPendingPages
8      | 4    | nextWriteIndex
12     | 20   | reserved
```

The EEPROM acts as a **ring buffer**.

---

# 4. EEPROM Write Logic

1. Convert measurement into 32-byte page  
2. Write to `nextWriteIndex`  
3. If at end → wrap to 0  
4. Increment counters  
5. If `EEPROM_FLUSH_AFTER` reached → dump to SD  

Writes are atomic and recoverable after resets.

---

# 5. SD Card CSV File Format

Measurements stored in SD card after EEPROM flush use:

### **Filename**
```
YYYYMMDD_P.CSV
```

Suffix `_P` = **pending file**  
After full upload → renamed to `.CSV`

### **CSV Header**
```
timestamp,level1,level2,level3,level4,level5,battery,temp,status,sent
```

### **Each Line**
```
2025-11-28 23:38:00,128,129,130,131,132,3980,2450,0,0
```

| Field | Meaning |
|--------|---------|
| timestamp | human‑readable RTC timestamp |
| level1..5 | raw sensor samples |
| battery | battery mV |
| temp | RTC temperature |
| status | status flags |
| sent | 0 = pending, 1 = uploaded |

---

# 6. SD File Rotation and Lifecycle

### State 1 — Pending File:  
```
20251128_P.CSV
```
Contains unsent data.

### State 2 — Uploading  
For each row:
- Convert CSV line → JSON  
- Upload via MQTT  
- If success → mark `sent = 1`

### State 3 — File Completed  
If all lines sent:
- Rename to:
```
20251128.CSV
```

Archived permanently.

---

# 7. MQTT Payload Conversion

CSV → JSON (used in MQTT publish):

```json
{
  "ts": 1764376680000,
  "values": {
    "waterLevelSensor": [128,129,128,130,131],
    "battery": 3980,
    "temperature": 2450,
    "status": 0,
    "eepromFreePages": 1500,
    "sdFreeMB": 28700
  }
}
```

Notes:
- `"ts"` is in **milliseconds**, converted from RTC timestamp.
- Field names match ThingsBoard expected format.

---

# 8. How “sent” Lines Are Marked

In `measurement_storage.cpp`:

1. Read line  
2. Publish JSON  
3. If MQTT ACK:
```
update column 'sent' → 1
```
4. Rewrite the file  
5. Only when **all lines** are `1`, rename to final `.CSV`.

This ensures resilience against crashes.

---

# 9. Data Recovery Logic

## If device resets unexpectedly:

### EEPROM:
- Header keeps counters → resume writing  
- Partially written pages detected and cleaned  

### SD Card:
- Pending files remain  
- Upload resumes on next GPRS window  

### MQTT:
- Duplicate writes prevented by `sent=1` mechanism  

---

# 10. When Is EEPROM Dumped to SD?

Triggered when:

```
pagesWritten % EEPROM_FLUSH_AFTER == 0
```

---

# 11. Summary of Storage Workflow

```
MEASUREMENT  → EEPROM PAGE → SD CSV → MQTT → ARCHIVE
```

Failsafe hierarchy:
```
SD >> EEPROM >> Cloud
```

No measurement is ever lost unless:
- RTC is invalid → timestamps questionable  
- Sensor fails → status flag raised  

---

# 13. Related Documentation

- **ARCHITECTURE.md**  
- **INSTALLATION.md**  
- **HARDWARE_SETUP.md**  
- **BLE_COMMANDS.md**  
- **MQTT_RPC_GUIDE.md**  