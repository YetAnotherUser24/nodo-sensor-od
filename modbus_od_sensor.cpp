#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ModbusRtu485.h>

static const uint8_t RS485_TX_PIN = 3;
static const uint8_t RS485_RX_PIN = 2;
static const uint8_t RS485_DE_RE_PIN = 1;
static const bool RS485_MANUAL_DIRECTION = true;

static const uint32_t MODBUS_BAUD = 9600;
static const uint8_t DEFAULT_MODBUS_SLAVE_ID = 10;
static const uint8_t MODBUS_MIN_SLAVE_ID = 1;
static const uint8_t MODBUS_MAX_SLAVE_ID = 247;
static const uint8_t MODBUS_FUNCTION_HOLDING = 0x03;
static const uint8_t MODBUS_FUNCTION_INPUT = 0x04;
static const uint8_t MODBUS_FUNCTION_WRITE_SINGLE = 0x06;

static const uint8_t OLED_SDA_PIN = 8;
static const uint8_t OLED_SCL_PIN = 9;
static const int8_t OLED_RESET_PIN = -1;
static const uint8_t OLED_I2C_ADDRESS = 0x3C;
static const uint8_t OLED_WIDTH = 128;
static const uint8_t OLED_HEIGHT = 64;

static const uint32_t POLL_INTERVAL_MS = 1000;
static const uint32_t MODBUS_RESPONSE_TIMEOUT_MS = 180;
static const uint16_t TX_ENABLE_GUARD_US = 250;
static const uint16_t TX_DISABLE_GUARD_US = 250;
static const uint16_t INTER_FRAME_GAP_MS = 4;
static const uint16_t MEASUREMENT_ORDER_REGISTER = 0x0001;
static const uint16_t MEASUREMENT_ORDER_ALL = 0x001F;
static const uint16_t SAMPLING_DELAY_REGISTER = 0x00A4;
static const uint32_t DEFAULT_SAMPLING_DELAY_MS = 250;

static bool discoveryMode = false;
static const uint16_t DISCOVERY_START_ADDR = 0x0000;
static const uint16_t DISCOVERY_END_ADDR = 0x0030;
static const uint16_t DISCOVERY_BLOCK_REGS = 2;

static const uint8_t SENSOR_READ_FUNCTION = MODBUS_FUNCTION_HOLDING;
static const uint16_t OD_REG_ADDR = 0x0055;
static const uint16_t TEMPERATURE_REG_ADDR = 0x0053;
static const uint16_t PARAM2_REG_ADDR = 0x0057;
static const uint16_t PARAM3_REG_ADDR = 0x0059;
static const uint16_t PARAM4_REG_ADDR = 0x005B;
static const uint16_t FLOAT_WORDS_PER_VALUE = 2;
static const uint16_t MEASUREMENT_FLOAT_COUNT = 5;
static const uint16_t MEASUREMENT_REGISTER_COUNT = FLOAT_WORDS_PER_VALUE * MEASUREMENT_FLOAT_COUNT;

static modbus_rtu485::Config modbusConfig = []() {
    modbus_rtu485::Config config;
    config.directionPin = RS485_DE_RE_PIN;
    config.manualDirection = RS485_MANUAL_DIRECTION;
    config.txActiveHigh = true;
    config.responseTimeoutMs = MODBUS_RESPONSE_TIMEOUT_MS;
    config.txEnableGuardUs = TX_ENABLE_GUARD_US;
    config.txDisableGuardUs = TX_DISABLE_GUARD_US;
    config.interFrameGapMs = INTER_FRAME_GAP_MS;
    config.rxTurnaroundUs = 0;
    return config;
}();

static modbus_rtu485::Master modbus(Serial1, modbusConfig);

static float lastValidOD = NAN;
static float lastValidTemperature = NAN;
static float lastValidParam2 = NAN;
static float lastValidParam3 = NAN;
static float lastValidParam4 = NAN;
static unsigned long lastPollMs = 0;

static uint8_t activeSlaveId = DEFAULT_MODBUS_SLAVE_ID;

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN);
static bool oledReady = false;

struct MeasurementBlock {
    float temperature;
    float parameter1;
    float parameter2;
    float parameter3;
    float parameter4;
};

bool isValidUnicastSlaveId(uint8_t slaveId);

bool isDigisensMeasurementError(uint16_t value) {
    return value == 9999 || value == 9998 || value == 9996;
}

const char *digisensMeasurementErrorDescription(uint16_t value) {
    switch (value) {
        case 9999:
            return "value out of range";
        case 9998:
            return "init default value at power up / no measure yet available";
        case 9996:
            return "measurement overflowed SDI12 digits";
        default:
            return nullptr;
    }
}
    bool isValidUnicastSlaveId(uint8_t slaveId) {
        return slaveId >= MODBUS_MIN_SLAVE_ID && slaveId <= MODBUS_MAX_SLAVE_ID;
    }

    bool detectSlaveIdByBroadcastProbe(uint8_t &detectedId) {
        modbus_rtu485::ReadResult response =
            modbus.readRegisters(DEFAULT_MODBUS_SLAVE_ID, SENSOR_READ_FUNCTION, SAMPLING_DELAY_REGISTER, 1);
        if (!response.success) {
            return false;
        }

        if (!isValidUnicastSlaveId(response.responderId)) {
            return false;
        }

        detectedId = response.responderId;
        return true;
    }

    bool scanSlaveId(uint8_t &detectedId) {
        for (uint8_t slaveId = MODBUS_MIN_SLAVE_ID; slaveId <= MODBUS_MAX_SLAVE_ID; slaveId++) {
            modbus_rtu485::ReadResult response = modbus.readRegisters(slaveId, SENSOR_READ_FUNCTION, SAMPLING_DELAY_REGISTER, 1);
            if (response.success && response.responderId == slaveId) {
                detectedId = slaveId;
                return true;
            }
        }

        return false;
    }

    bool ensureSlaveIdResolved(bool verbose = true) {
        if (isValidUnicastSlaveId(activeSlaveId)) {
            return true;
        }

        uint8_t detectedId = DEFAULT_MODBUS_SLAVE_ID;
        if (detectSlaveIdByBroadcastProbe(detectedId) || scanSlaveId(detectedId)) {
            activeSlaveId = detectedId;
            if (verbose) {
                Serial.print("[ADDR] Using MODBUS slave ID ");
                Serial.println(activeSlaveId);
            }
            return true;
        }

        if (verbose) {
            Serial.println("[ADDR] Could not detect a MODBUS slave ID. Check wiring or set ID manually.");
        }
        return false;
    }

bool triggerDigisensMeasurement() {
        if (!ensureSlaveIdResolved()) {
            return false;
        }

        modbus_rtu485::WriteResult result = modbus.writeSingleRegister(activeSlaveId, MEASUREMENT_ORDER_REGISTER, MEASUREMENT_ORDER_ALL);
    if (!result.success) {
        if (result.isException) {
            Serial.print("[TRIGGER] MODBUS exception code: ");
            Serial.println(result.exceptionCode);
        } else {
            Serial.print("[TRIGGER] Communication error: ");
            Serial.println(result.error);
        }
        return false;
    }

    return true;
}

uint32_t readSamplingDelayMs() {
    if (!ensureSlaveIdResolved()) {
        return DEFAULT_SAMPLING_DELAY_MS;
    }

    modbus_rtu485::ReadResult response = modbus.readRegisters(activeSlaveId, SENSOR_READ_FUNCTION, SAMPLING_DELAY_REGISTER, 1);
    if (!response.success || response.count == 0) {
        return DEFAULT_SAMPLING_DELAY_MS;
    }

    return response.values[0];
}

float decodeFloatFromRegisters(uint16_t firstRegister, uint16_t secondRegister, bool swapWords) {
    union {
        uint32_t u32;
        float f32;
    } value = {};

    if (swapWords) {
        value.u32 = (static_cast<uint32_t>(secondRegister) << 16) | static_cast<uint32_t>(firstRegister);
    } else {
        value.u32 = (static_cast<uint32_t>(firstRegister) << 16) | static_cast<uint32_t>(secondRegister);
    }

    return value.f32;
}

bool isPlausibleMeasurement(float temperature, float od) {
    return temperature >= -40.0f && temperature <= 125.0f && od >= 0.0f && od <= 500.0f;
}

bool allParametersFinite(const MeasurementBlock &values) {
    return !isnan(values.temperature) && !isnan(values.parameter1) && !isnan(values.parameter2) &&
           !isnan(values.parameter3) && !isnan(values.parameter4);
}

void initOled() {
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);

    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
        Serial.println("[OLED] SSD1306 init failed.");
        oledReady = false;
        return;
    }

    oledReady = true;
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.println("OD sensor ready");
    oled.println("Waiting for data...");
    oled.display();
}

void showOledValues(float temperature, float p1, float p2, float p3, float p4) {
    if (!oledReady) {
        return;
    }

    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.println("DIGISENS OD");
    oled.print("ID: ");
    if (isValidUnicastSlaveId(activeSlaveId)) {
        oled.println(activeSlaveId);
    } else {
        oled.println("auto");
    }
    oled.print("T: ");
    oled.print(temperature, 2);
    oled.println(" C");
    oled.print("P1: ");
    oled.print(p1, 3);
    oled.println(" mg/L");
    oled.print("P2: ");
    oled.print(p2, 2);
    oled.println(" %Sat");
    oled.print("P3: ");
    oled.print(p3, 3);
    oled.print(" P4: ");
    oled.println(p4, 3);
    oled.display();
}

void printHexWord(uint16_t value) {
    char text[7];
    snprintf(text, sizeof(text), "0x%04X", value);
    Serial.print(text);
}

void printActiveSlaveIdTag() {
    Serial.print("[ID=");
    if (isValidUnicastSlaveId(activeSlaveId)) {
        Serial.print(activeSlaveId);
    } else {
        Serial.print("auto");
    }
    Serial.print("] ");
}

void printStartupInfo() {
    Serial.println();
    Serial.println("==============================================");
    Serial.println("ESP32-C3 MODBUS RTU RS485 OD Reader");
    Serial.println("==============================================");
    Serial.println("MAX485 Control Pin:");
    Serial.print("  DE/RE D");
    Serial.print(RS485_DE_RE_PIN);
    Serial.println(" (LOW=RX, HIGH=TX)");
    Serial.print("  Direction mode: ");
    Serial.println(RS485_MANUAL_DIRECTION ? "MANUAL GPIO control" : "AUTO transceiver control");
    Serial.println("UART1 MODBUS wiring:");
    Serial.print("  TX D");
    Serial.print(RS485_TX_PIN);
    Serial.println(" -> MAX485 DI");
    Serial.print("  RX D");
    Serial.print(RS485_RX_PIN);
    Serial.println(" <- MAX485 RO");
    Serial.println("MODBUS settings:");
    Serial.print("  Configured slave ID: ");
    Serial.println(DEFAULT_MODBUS_SLAVE_ID);
    Serial.print("  Active slave ID: ");
    if (isValidUnicastSlaveId(activeSlaveId)) {
        Serial.println(activeSlaveId);
    } else {
        Serial.println("auto-detect pending");
    }
    Serial.print("  Baud: ");
    Serial.println(MODBUS_BAUD);
    Serial.println("  Frame: 8N1");

    if (discoveryMode) {
        Serial.println("Mode: DISCOVERY (register scan active)");
        Serial.print("  Range: ");
        printHexWord(DISCOVERY_START_ADDR);
        Serial.print(" .. ");
        printHexWord(DISCOVERY_END_ADDR);
        Serial.println();
        Serial.println("  Functions scanned: FC03 and FC04");
        Serial.println("  Command: send 'n' in serial monitor to switch to normal mode");
    } else {
        Serial.println("Mode: NORMAL (fixed map polling)");
        Serial.print("  Function code: 0x");
        Serial.println(SENSOR_READ_FUNCTION, HEX);
        Serial.print("  OD reg: ");
        printHexWord(OD_REG_ADDR);
        Serial.println();
        Serial.print("  Temperature reg: ");
        printHexWord(TEMPERATURE_REG_ADDR);
        Serial.println();
        Serial.println("  Command: send 'd' in serial monitor to switch to discovery mode");
    }
    Serial.println("==============================================");
}

void handleSerialCommand() {
    if (Serial.available() <= 0) {
        return;
    }

    char cmd = static_cast<char>(Serial.read());
    if (cmd == 'd' || cmd == 'D') {
        discoveryMode = true;
        Serial.println("[CMD] Discovery mode enabled.");
    } else if (cmd == 'n' || cmd == 'N') {
        discoveryMode = false;
        Serial.println("[CMD] Normal polling mode enabled.");
    } else if (cmd == 'h' || cmd == 'H') {
        Serial.println("[CMD] Available commands:");
        Serial.println("  d -> discovery mode");
        Serial.println("  n -> normal mode");
        Serial.println("  s -> rescan MODBUS slave ID");
        Serial.println("  h -> help");
    } else if (cmd == 's' || cmd == 'S') {
        activeSlaveId = DEFAULT_MODBUS_SLAVE_ID;
        ensureSlaveIdResolved(true);
    }
}

bool readOdAndTemperature(float &odOut, float &temperatureOut) {
    if (!ensureSlaveIdResolved()) {
        return false;
    }

    if (!triggerDigisensMeasurement()) {
        return false;
    }

    uint32_t samplingDelayMs = readSamplingDelayMs();
    if (samplingDelayMs == 0) {
        samplingDelayMs = DEFAULT_SAMPLING_DELAY_MS;
    }
    delay(samplingDelayMs);

    modbus_rtu485::ReadResult raw =
        modbus.readRegisters(activeSlaveId, SENSOR_READ_FUNCTION, TEMPERATURE_REG_ADDR, MEASUREMENT_REGISTER_COUNT);
    if (!raw.success) {
        if (raw.isException) {
            Serial.print("[READ] MODBUS exception code: ");
            Serial.println(raw.exceptionCode);
        } else {
            Serial.print("[READ] Communication error: ");
            Serial.println(raw.error);
        }
        return false;
    }

    if (raw.count < MEASUREMENT_REGISTER_COUNT) {
        Serial.print("[READ] Incomplete float payload. registers=");
        Serial.println(raw.count);
        return false;
    }

    MeasurementBlock normal = {};
    normal.temperature = decodeFloatFromRegisters(raw.values[0], raw.values[1], false);
    normal.parameter1 = decodeFloatFromRegisters(raw.values[2], raw.values[3], false);
    normal.parameter2 = decodeFloatFromRegisters(raw.values[4], raw.values[5], false);
    normal.parameter3 = decodeFloatFromRegisters(raw.values[6], raw.values[7], false);
    normal.parameter4 = decodeFloatFromRegisters(raw.values[8], raw.values[9], false);

    MeasurementBlock swapped = {};
    swapped.temperature = decodeFloatFromRegisters(raw.values[0], raw.values[1], true);
    swapped.parameter1 = decodeFloatFromRegisters(raw.values[2], raw.values[3], true);
    swapped.parameter2 = decodeFloatFromRegisters(raw.values[4], raw.values[5], true);
    swapped.parameter3 = decodeFloatFromRegisters(raw.values[6], raw.values[7], true);
    swapped.parameter4 = decodeFloatFromRegisters(raw.values[8], raw.values[9], true);

    bool useSwappedOrder = false;
    if (!isPlausibleMeasurement(normal.temperature, normal.parameter1) &&
        isPlausibleMeasurement(swapped.temperature, swapped.parameter1)) {
        useSwappedOrder = true;
    }

    MeasurementBlock selected = useSwappedOrder ? swapped : normal;

    temperatureOut = selected.temperature;
    odOut = selected.parameter1;

    if (useSwappedOrder) {
        lastValidParam2 = selected.parameter2;
        lastValidParam3 = selected.parameter3;
        lastValidParam4 = selected.parameter4;
    } else {
        lastValidParam2 = selected.parameter2;
        lastValidParam3 = selected.parameter3;
        lastValidParam4 = selected.parameter4;
    }

    Serial.print("[READ] ");
    printActiveSlaveIdTag();
    Serial.print("Raw regs: T=");
    printHexWord(raw.values[0]);
    Serial.print(" ");
    printHexWord(raw.values[1]);
    Serial.print(" | P1=DO");
    printHexWord(raw.values[2]);
    Serial.print(" ");
    printHexWord(raw.values[3]);
    Serial.print(" | P2=%Sat");
    printHexWord(raw.values[4]);
    Serial.print(" ");
    printHexWord(raw.values[5]);
    Serial.print(" | P3=");
    printHexWord(raw.values[6]);
    Serial.print(" ");
    printHexWord(raw.values[7]);
    Serial.print(" | P4=");
    printHexWord(raw.values[8]);
    Serial.print(" ");
    printHexWord(raw.values[9]);
    Serial.print(" | order=");
    Serial.println(useSwappedOrder ? "swapped" : "normal");

    if (!isPlausibleMeasurement(selected.temperature, selected.parameter1) || !allParametersFinite(selected)) {
        Serial.print("[READ] Implausible decoded floats. temp=");
        Serial.print(selected.temperature, 6);
        Serial.print(" p1=");
        Serial.print(selected.parameter1, 6);
        Serial.print(" p2=");
        Serial.print(selected.parameter2, 6);
        Serial.print(" p3=");
        Serial.print(selected.parameter3, 6);
        Serial.print(" p4=");
        Serial.println(selected.parameter4, 6);
        return false;
    }

    if (isDigisensMeasurementError(static_cast<uint16_t>(temperatureOut)) || isDigisensMeasurementError(static_cast<uint16_t>(odOut))) {
        const char *temperatureDescription = digisensMeasurementErrorDescription(static_cast<uint16_t>(temperatureOut));
        if (temperatureDescription != nullptr) {
            Serial.print("[READ] Digisens temperature code: ");
            Serial.print(static_cast<uint16_t>(temperatureOut));
            Serial.print(" -> ");
            Serial.println(temperatureDescription);
        }

        const char *odDescription = digisensMeasurementErrorDescription(static_cast<uint16_t>(odOut));
        if (odDescription != nullptr) {
            Serial.print("[READ] Digisens OD code: ");
            Serial.print(static_cast<uint16_t>(odOut));
            Serial.print(" -> ");
            Serial.println(odDescription);
        }
    }

    return true;
}

void runDiscoveryStep() {
    static uint16_t currentAddress = DISCOVERY_START_ADDR;
    static uint8_t functionIndex = 0;
    const uint8_t functions[2] = {MODBUS_FUNCTION_HOLDING, MODBUS_FUNCTION_INPUT};

    uint8_t functionCode = functions[functionIndex];
    uint8_t slaveIdForScan = isValidUnicastSlaveId(activeSlaveId) ? activeSlaveId : DEFAULT_MODBUS_SLAVE_ID;
    modbus_rtu485::ReadResult response = modbus.readRegisters(slaveIdForScan, functionCode, currentAddress, DISCOVERY_BLOCK_REGS);

    Serial.print("[SCAN] FC");
    Serial.print(functionCode, HEX);
    Serial.print(" addr=");
    printHexWord(currentAddress);
    Serial.print(" -> ");

    if (response.success) {
        Serial.print("OK");
        for (uint8_t i = 0; i < response.count; i++) {
            Serial.print(" | R");
            printHexWord(currentAddress + i);
            Serial.print("=");
            Serial.print(response.values[i]);
            Serial.print(" (");
            printHexWord(response.values[i]);
            Serial.print(")");
        }
        Serial.println();
    } else if (response.isException) {
        Serial.print("EXCEPTION code=");
        Serial.println(response.exceptionCode);
    } else {
        Serial.println(response.error);
    }

    functionIndex++;
    if (functionIndex >= 2) {
        functionIndex = 0;
        currentAddress = static_cast<uint16_t>(currentAddress + DISCOVERY_BLOCK_REGS);
        if (currentAddress > DISCOVERY_END_ADDR) {
            currentAddress = DISCOVERY_START_ADDR;
            Serial.println("[SCAN] End of range reached, restarting discovery window.");
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    modbus.configure(modbusConfig);
    modbus.begin(MODBUS_BAUD, RS485_RX_PIN, RS485_TX_PIN);

    ensureSlaveIdResolved(false);
    initOled();

    printStartupInfo();
    lastPollMs = millis();
}

void loop() {
    handleSerialCommand();

    unsigned long now = millis();
    if ((now - lastPollMs) < POLL_INTERVAL_MS) {
        return;
    }
    lastPollMs = now;

    if (discoveryMode) {
        runDiscoveryStep();
        return;
    }

    float od = NAN;
    float temperature = NAN;
    if (readOdAndTemperature(od, temperature)) {
        lastValidOD = od;
        lastValidTemperature = temperature;
        showOledValues(temperature, od, lastValidParam2, lastValidParam3, lastValidParam4);

        Serial.print("[DATA] ");
        printActiveSlaveIdTag();
        Serial.print("T: ");
        Serial.print(temperature, 2);
        Serial.print(" C | P1 DO: ");
        Serial.print(od, 3);
        Serial.print(" mg/L (or ppm) | P2 DO sat: ");
        Serial.print(lastValidParam2, 2);
        Serial.print(" % | P3: ");
        Serial.print(lastValidParam3, 3);
        Serial.print(" | P4: ");
        Serial.println(lastValidParam4, 3);
    } else {
        if (oledReady && !isnan(lastValidTemperature) && !isnan(lastValidOD)) {
            showOledValues(lastValidTemperature, lastValidOD, lastValidParam2, lastValidParam3, lastValidParam4);
        }
        Serial.print("[DATA] ");
        printActiveSlaveIdTag();
        Serial.println("Read failed, keeping last valid values.");
    }

    if (!isnan(lastValidOD) && !isnan(lastValidTemperature) && !isnan(lastValidParam2) && !isnan(lastValidParam3) &&
        !isnan(lastValidParam4)) {
        Serial.print("[LAST] ");
        printActiveSlaveIdTag();
        Serial.print("T: ");
        Serial.print(lastValidTemperature, 2);
        Serial.print(" C | P1 DO: ");
        Serial.print(lastValidOD, 3);
        Serial.print(" mg/L (or ppm) | P2 DO sat: ");
        Serial.print(lastValidParam2, 2);
        Serial.print(" % | P3: ");
        Serial.print(lastValidParam3, 3);
        Serial.print(" | P4: ");
        Serial.println(lastValidParam4, 3);
    }
}