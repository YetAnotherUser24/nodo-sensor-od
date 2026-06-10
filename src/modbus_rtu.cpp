#include "modbus_rtu.h"

namespace modbus_rtu {

namespace {

// Protocol and frame-layout constants kept local to this translation unit.

constexpr uint8_t ExceptionMask = static_cast<uint8_t>(0x80U);
constexpr uint8_t WriteSingleRegisterFn = static_cast<uint8_t>(0x06U);
constexpr uint8_t RequestFrameBytes = static_cast<uint8_t>(8U);
constexpr uint8_t RequestPayloadBytes = static_cast<uint8_t>(6U);
constexpr uint8_t MinResponseFrameBytes = static_cast<uint8_t>(5U);
constexpr uint8_t WriteResponseFrameBytes = static_cast<uint8_t>(8U);
constexpr uint8_t ResponseBufferBytes = static_cast<uint8_t>(64U);
constexpr uint8_t BytePairSize = static_cast<uint8_t>(2U);

constexpr uint16_t CrcInit = static_cast<uint16_t>(0xFFFFU);
constexpr uint16_t CrcPolynomial = static_cast<uint16_t>(0xA001U);
constexpr uint16_t CrcLsbMask = static_cast<uint16_t>(0x0001U);
constexpr uint8_t BitsPerByte = static_cast<uint8_t>(8U);

}  // namespace

ModbusRtu::ModbusRtu(HardwareSerial &serialPort) : serial_(&serialPort) {}

ModbusRtu::ModbusRtu(HardwareSerial &serialPort, const Config &sessionConfig) : serial_(&serialPort), config_(sessionConfig) {}

void ModbusRtu::configure(const Config &config) {
    config_ = config;
}

void ModbusRtu::begin(uint32_t baud, int8_t rxPin, int8_t txPin, uint32_t serialConfig) {
    if (serial_ == nullptr) {
        return;
    }

    if (config_.manualDirection && config_.directionPin != InvalidDirectionPin) {
        pinMode(config_.directionPin, OUTPUT);
        setReceiveMode();
    }

    serial_->begin(baud, serialConfig, rxPin, txPin);
}

void ModbusRtu::setTiming(uint32_t responseTimeoutMs,
                          uint16_t txEnableGuardUs,
                          uint16_t txDisableGuardUs,
                          uint16_t interFrameGapMs,
                          uint16_t rxTurnaroundUs) {
    config_.responseTimeoutMs = responseTimeoutMs;
    config_.txEnableGuardUs = txEnableGuardUs;
    config_.txDisableGuardUs = txDisableGuardUs;
    config_.interFrameGapMs = interFrameGapMs;
    config_.rxTurnaroundUs = rxTurnaroundUs;
}

void ModbusRtu::setDirectionControl(uint8_t directionPin, bool manualDirection, bool txActiveHigh) {
    config_.directionPin = directionPin;
    config_.manualDirection = manualDirection;
    config_.txActiveHigh = txActiveHigh;

    if (config_.manualDirection && config_.directionPin != InvalidDirectionPin) {
        pinMode(config_.directionPin, OUTPUT);
        setReceiveMode();
    }
}

void ModbusRtu::setTxActiveHigh(bool txActiveHigh) {
    config_.txActiveHigh = txActiveHigh;
}

void ModbusRtu::setManualDirection(bool manualDirection) {
    config_.manualDirection = manualDirection;
    if (config_.manualDirection && config_.directionPin != InvalidDirectionPin) {
        pinMode(config_.directionPin, OUTPUT);
        setReceiveMode();
    }
}

void ModbusRtu::clearRxBuffer() {
    if (serial_ == nullptr) {
        return;
    }

    while (serial_->available() > 0) {
        (void)serial_->read();
    }
}

void ModbusRtu::setReceiveMode() {
    applyDirection(false);
}

void ModbusRtu::setTransmitMode() {
    applyDirection(true);
}

bool ModbusRtu::isBroadcastAddress(uint8_t address) {
    return (address == BroadcastAddressLow) || (address == BroadcastAddressHigh);
}

uint16_t ModbusRtu::crc16(const uint8_t *data, size_t length) {
    // Defensive guard: avoid dereferencing a null pointer when length is non-zero.
    if ((data == nullptr) && (length > 0U)) {
        return 0U;
    }

    uint16_t crc = CrcInit;
    for (size_t i = 0U; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < BitsPerByte; ++bit) {
            if ((crc & CrcLsbMask) != 0U) {
                crc = static_cast<uint16_t>((crc >> 1U) ^ CrcPolynomial);
            } else {
                crc = static_cast<uint16_t>(crc >> 1U);
            }
        }
    }
    return crc;
}

void ModbusRtu::applyDirection(bool transmit) {
    if (!config_.manualDirection || config_.directionPin == InvalidDirectionPin) {
        return;
    }

    bool pinLevel = transmit ? config_.txActiveHigh : !config_.txActiveHigh;
    digitalWrite(config_.directionPin, pinLevel ? HIGH : LOW);
}

bool ModbusRtu::collectResponse(uint8_t *response, size_t responseCapacity, size_t &responseLength) {
    responseLength = 0U;
    if ((serial_ == nullptr) || (response == nullptr) || (responseCapacity == 0U)) {
        return false;
    }

    unsigned long startMs = millis();
    unsigned long lastRxMs = startMs;

    while ((millis() - startMs) < config_.responseTimeoutMs) {
        while ((serial_->available() > 0) && (responseLength < responseCapacity)) {
            int incoming = serial_->read();
            if (incoming >= 0) {
                response[responseLength++] = static_cast<uint8_t>(incoming);
                lastRxMs = millis();
            }
        }

        // End frame collection after an inter-frame silent gap is detected.
        if ((responseLength >= MinResponseFrameBytes) && ((millis() - lastRxMs) >= config_.interFrameGapMs)) {
            break;
        }
    }

    return responseLength > 0U;
}

bool ModbusRtu::sendRequestAndCollectResponse(const uint8_t *request,
                                              size_t requestLength,
                                              uint8_t *response,
                                              size_t responseCapacity,
                                              size_t &responseLength) {
    clearRxBuffer();

    setTransmitMode();
    delayMicroseconds(config_.txEnableGuardUs);
    if (serial_ != nullptr) {
        serial_->write(request, requestLength);
        serial_->flush();
    }
    delayMicroseconds(config_.txDisableGuardUs);
    setReceiveMode();
    delayMicroseconds(config_.rxTurnaroundUs);

    return collectResponse(response, responseCapacity, responseLength);
}

void ModbusRtu::buildRequestFrame(uint8_t slaveId,
                                  uint8_t functionCode,
                                  uint16_t firstValue,
                                  uint16_t secondValue,
                                  uint8_t *request) {
    request[0] = slaveId;
    request[1] = functionCode;
    request[2] = static_cast<uint8_t>((firstValue >> 8) & 0xFF);
    request[3] = static_cast<uint8_t>(firstValue & 0xFF);
    request[4] = static_cast<uint8_t>((secondValue >> 8) & 0xFF);
    request[5] = static_cast<uint8_t>(secondValue & 0xFF);

    uint16_t crc = crc16(request, RequestPayloadBytes);
    request[6] = static_cast<uint8_t>(crc & 0xFF);
    request[7] = static_cast<uint8_t>((crc >> 8) & 0xFF);
}

ModbusRtu::ResponseCheck ModbusRtu::validateResponse(uint8_t slaveId,
                                                     uint8_t functionCode,
                                                     const uint8_t *response,
                                                     size_t responseLen) {
    ResponseCheck check = {};

    if ((response == nullptr) || (responseLen < MinResponseFrameBytes)) {
        check.error = "timeout or short response";
        return check;
    }

    uint16_t receivedCrc = static_cast<uint16_t>(response[responseLen - 2]) |
                           (static_cast<uint16_t>(response[responseLen - 1]) << 8);
    uint16_t computedCrc = crc16(response, responseLen - 2);
    if (receivedCrc != computedCrc) {
        check.error = "crc mismatch";
        return check;
    }

    if (!isBroadcastAddress(slaveId) && response[0] != slaveId) {
        check.error = "unexpected slave id";
        return check;
    }

    check.responderId = response[0];

    if (response[1] == static_cast<uint8_t>(functionCode | ExceptionMask)) {
        check.valid = true;
        check.isException = true;
        check.exceptionCode = response[2];
        check.error = "modbus exception";
        return check;
    }

    if (response[1] != functionCode) {
        check.error = "unexpected function code";
        return check;
    }

    check.valid = true;
    check.error = "ok";
    return check;
}

ModbusRtu::ReadResult ModbusRtu::readRegisters(uint8_t slaveId,
                                               uint8_t functionCode,
                                               uint16_t startAddress,
                                               uint16_t registerCount) {
    ReadResult result = {};

    if (registerCount == 0 || registerCount > MaxRegisters) {
        result.error = "invalid registerCount";
        return result;
    }

    uint8_t request[RequestFrameBytes];
    buildRequestFrame(slaveId, functionCode, startAddress, registerCount, request);

    uint8_t response[ResponseBufferBytes];
    size_t responseLen = 0U;
    if (!sendRequestAndCollectResponse(request, sizeof(request), response, sizeof(response), responseLen)) {
        result.error = "timeout or short response";
        return result;
    }

    ResponseCheck check = validateResponse(slaveId, functionCode, response, responseLen);
    if (!check.valid) {
        result.error = check.error;
        return result;
    }

    result.responderId = check.responderId;
    if (check.isException) {
        result.isException = true;
        result.exceptionCode = check.exceptionCode;
        result.error = check.error;
        return result;
    }

    uint8_t byteCount = response[2];
    size_t expectedLen = static_cast<size_t>(byteCount) + static_cast<size_t>(MinResponseFrameBytes);
    if (responseLen < expectedLen) {
        result.error = "incomplete payload";
        return result;
    }

    if ((byteCount % BytePairSize) != 0U) {
        result.error = "invalid byte count";
        return result;
    }

    uint8_t registerBytes = static_cast<uint8_t>(byteCount / BytePairSize);
    if (registerBytes > registerCount || registerBytes > MaxRegisters) {
        result.error = "unexpected register payload size";
        return result;
    }

    result.count = registerBytes;
    for (uint8_t i = 0U; i < registerBytes; ++i) {
        uint8_t hi = response[3U + (BytePairSize * i)];
        uint8_t lo = response[4U + (BytePairSize * i)];
        result.values[i] = (static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo);
    }

    result.success = true;
    result.error = "ok";
    return result;
}

ModbusRtu::WriteResult ModbusRtu::writeSingleRegister(uint8_t slaveId, uint16_t registerAddress, uint16_t value) {
    WriteResult result = {};
    result.registerAddress = registerAddress;
    result.registerValue = value;

    uint8_t request[RequestFrameBytes];
    buildRequestFrame(slaveId, WriteSingleRegisterFn, registerAddress, value, request);

    uint8_t response[ResponseBufferBytes];
    size_t responseLen = 0U;
    bool haveResponse = sendRequestAndCollectResponse(request, sizeof(request), response, sizeof(response), responseLen);

    // Modbus broadcast writes do not require a slave response.
    if ((!haveResponse) || (responseLen == 0U)) {
        if (isBroadcastAddress(slaveId)) {
            result.success = true;
            result.error = "ok";
            return result;
        }

        result.error = "timeout or short response";
        return result;
    }

    ResponseCheck check = validateResponse(slaveId, WriteSingleRegisterFn, response, responseLen);
    if (!check.valid) {
        result.error = check.error;
        return result;
    }

    result.responderId = check.responderId;
    if (check.isException) {
        result.isException = true;
        result.exceptionCode = check.exceptionCode;
        result.error = check.error;
        return result;
    }

    if (responseLen < WriteResponseFrameBytes) {
        result.error = "unexpected write response";
        return result;
    }

    uint16_t echoedRegister = (static_cast<uint16_t>(response[2]) << 8) | response[3];
    uint16_t echoedValue = (static_cast<uint16_t>(response[4]) << 8) | response[5];
    if (echoedRegister != registerAddress || echoedValue != value) {
        result.error = "write echo mismatch";
        return result;
    }

    result.success = true;
    result.error = "ok";
    return result;
}

}  // namespace modbus_rtu