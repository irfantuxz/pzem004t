/*
 * PZEM004Tv30.cpp
 *
 * Fork version 2.0.0, based on upstream version 1.2.1.
 * Copyright (c) 2019 Jakub Mandula
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 */

#include "PZEM004Tv30.h"
#include <math.h>
#include <stdio.h>

#if defined(ESP32) || defined(ESP8266)
  #define PZEM_YIELD() yield()
#else
  #define PZEM_YIELD() ((void)0)
#endif

#define CMD_RHR          0x03
#define CMD_RIR          0x04
#define CMD_WSR          0x06
#define CMD_REST         0x42

#define WREG_ALARM_THR   0x0001
#define WREG_ADDR        0x0002

#define MEASUREMENT_FRAME_SIZE 25
#define MODBUS_FRAME_SIZE      8
#define USE_CURRENT_ADDRESS    0xFF
#define GENERIC_ADDRESS        0xF8
#define BROADCAST_ADDRESS      0x00

#if defined(PZEM004_SOFTSERIAL)
const PZEM004Tv30::SoftwareSerialTag PZEM004Tv30::SoftwareSerial =
    PZEM004Tv30::SoftwareSerialTag();
#endif

// -----------------------------------------------------------------------------
// Constructors / initialization
// -----------------------------------------------------------------------------

PZEM004Tv30::PZEM004Tv30()
    : _serial(nullptr),
      _hardwareSerial(nullptr),
#if defined(PZEM004_SOFTSERIAL)
      _softwareSerial(nullptr),
      _ownedSoftwareSerial(nullptr),
      _softwareSerialInternal(false),
      _softwareSerialNeedsBegin(false),
#endif
      _serialKind(SerialKind::NONE),
      _addr(PZEM_DEFAULT_ADDR),
      _isConnected(false),
      _hasData(false),
      _pinConfigValid(true),
      _rxPin(-1),
      _txPin(-1),
      _softRxPin(-1),
      _softTxPin(-1),
      _baudRate(PZEM_BAUD_RATE),
      _currentValues{NAN, NAN, NAN, NAN, NAN, NAN, 0},
      _updateInterval(PZEM_DEFAULT_INTERVAL),
      _responseTimeout(PZEM_DEFAULT_TIMEOUT),
      _lastRequestTime(0),
      _lastDataTime(0),
      _requestStarted(false),
      _lastServiceMillis(0),
      _hasServiced(false),
      _rxOverflowCount(0),
      _rxState(RxState::IDLE),
      _rxStartTime(0),
      _rxBuffer{0},
      _rxIndex(0),
      _lastError(Error::NOT_INITIALIZED),
      _logOutput(nullptr) {
}

PZEM004Tv30::PZEM004Tv30(Stream& port, uint8_t addr)
    : PZEM004Tv30() {
#if defined(PZEM004_SOFTSERIAL)
    // Backward compatibility: the original library treated Stream& as a
    // SoftwareSerial stream (some ESP32 sketches explicitly cast to Stream&).
    _softwareSerial = static_cast<::SoftwareSerial*>(&port);
    init(&port, SerialKind::SOFTWARE, addr);
    // Legacy Stream& overload deliberately does not call begin(). The caller
    // owns initialization (e.g. pzemSerial.begin(...) in setup()).
    _softwareSerialNeedsBegin = false;
#else
    init(&port, SerialKind::STREAM, addr);
#endif
}

#if defined(ESP32)
PZEM004Tv30::PZEM004Tv30(HardwareSerial& port, int8_t receivePin,
                         int8_t transmitPin, uint8_t addr, bool beginPort)
    : PZEM004Tv30() {
    _hardwareSerial = &port;
    _rxPin = receivePin;
    _txPin = transmitPin;
    init(&port, SerialKind::HARDWARE, addr);

    _pinConfigValid = isHardwareSerialPinValid(receivePin, transmitPin);
    if (!_pinConfigValid) {
        _lastError = Error::INVALID_PIN;
        logError(_lastError, "HardwareSerial RX/TX pin validation");
        return;
    }
    if (beginPort && !begin(PZEM_BAUD_RATE)) {
        _lastError = Error::BEGIN_FAILED;
        logError(_lastError, "HardwareSerial begin");
    }
}
#else
PZEM004Tv30::PZEM004Tv30(HardwareSerial& port, uint8_t addr)
    : PZEM004Tv30() {
    _hardwareSerial = &port;
    init(&port, SerialKind::HARDWARE, addr);
    port.begin(PZEM_BAUD_RATE);
    _baudRate = PZEM_BAUD_RATE;
}
#endif

#if defined(PZEM004_SOFTSERIAL)
PZEM004Tv30::PZEM004Tv30(const SoftwareSerialTag&, uint8_t receivePin,
                         uint8_t transmitPin, uint8_t addr)
    : PZEM004Tv30() {
    initSoftwareSerial(receivePin, transmitPin, addr);
}

// Legacy form: PZEM004Tv30 pzem(rxPin, txPin, address).
PZEM004Tv30::PZEM004Tv30(uint8_t receivePin, uint8_t transmitPin, uint8_t addr)
    : PZEM004Tv30() {
    initSoftwareSerial(receivePin, transmitPin, addr);
}

PZEM004Tv30::PZEM004Tv30(::SoftwareSerial& port, uint8_t addr)
    : PZEM004Tv30() {
    // Do not call SoftwareSerial::begin() here. Global PZEM objects are
    // constructed before Arduino setup()/initArduino() on many cores.
    // Starting EspSoftwareSerial here may attach GPIO interrupts too early.
    // begin() is deferred until the first update()/getter, at runtime.
    _softwareSerial = &port;
    _softRxPin = -1;
    _softTxPin = -1;
    init(&port, SerialKind::SOFTWARE, addr);
    _softwareSerialInternal = false;
    _softwareSerialNeedsBegin = true;
}

void PZEM004Tv30::initSoftwareSerial(uint8_t receivePin,
                                     uint8_t transmitPin, uint8_t addr) {
    // Defer allocation and begin() until runtime, after GPIO initialization.
    _softwareSerial = nullptr;
    _ownedSoftwareSerial = nullptr;
    _softwareSerialInternal = true;
    _softwareSerialNeedsBegin = true;
    _softRxPin = static_cast<int8_t>(receivePin);
    _softTxPin = static_cast<int8_t>(transmitPin);
    _baudRate = PZEM_BAUD_RATE;
    init(nullptr, SerialKind::SOFTWARE, addr);
    _pinConfigValid = isSoftwareSerialPinValid(receivePin, transmitPin);
    if (!_pinConfigValid) {
        _lastError = Error::INVALID_PIN;
        logError(_lastError, "SoftwareSerial RX/TX pin validation");
    }
}
#endif

PZEM004Tv30::~PZEM004Tv30() {
#if defined(PZEM004_SOFTSERIAL)
    if (_ownedSoftwareSerial != nullptr) {
        delete _ownedSoftwareSerial;
        _ownedSoftwareSerial = nullptr;
        _softwareSerial = nullptr;
    }
#endif
}

void PZEM004Tv30::init(Stream* port, SerialKind kind, uint8_t addr) {
    // 0xF8 is the PZEM generic/default address. Device addresses are 0x01..0xF7.
    if (!((addr >= 0x01 && addr <= 0xF7) || addr == GENERIC_ADDRESS)) {
        addr = PZEM_DEFAULT_ADDR;
    }

    _serial = port;
    _serialKind = kind;
    _addr = addr;
    _isConnected = false;
    _hasData = false;
    _currentValues = {NAN, NAN, NAN, NAN, NAN, NAN, 0};

    _updateInterval = PZEM_DEFAULT_INTERVAL;
    _responseTimeout = PZEM_DEFAULT_TIMEOUT;
    _requestStarted = false;       // First update() call starts a query immediately.
    _lastServiceMillis = 0;
    _hasServiced = false;
    _rxOverflowCount = 0;
    _rxState = RxState::IDLE;
    _rxIndex = 0;
    _lastRequestTime = 0;
    _lastDataTime = 0;
    _lastError = (port != nullptr) ? Error::NONE : Error::NOT_INITIALIZED;
}

// -----------------------------------------------------------------------------
// Serial configuration
// -----------------------------------------------------------------------------

bool PZEM004Tv30::ensureSerialReady() {
#if defined(PZEM004_SOFTSERIAL)
    if (_serialKind == SerialKind::SOFTWARE && _softwareSerialNeedsBegin) {
        return begin(_baudRate);
    }
#endif
    if (_serial == nullptr) {
        _lastError = Error::NOT_INITIALIZED;
        return false;
    }
    return true;
}

bool PZEM004Tv30::begin(uint32_t baud) {
    if (!_pinConfigValid) {
        _lastError = Error::INVALID_PIN;
        logError(_lastError, "Serial begin rejected");
        return false;
    }
    // A library-owned SoftwareSerial port is intentionally allocated lazily,
    // so _serial is expected to be null before the first begin().
    const bool lazyOwnedSoftware =
#if defined(PZEM004_SOFTSERIAL)
        (_serialKind == SerialKind::SOFTWARE &&
         _softwareSerialInternal && _softwareSerialNeedsBegin);
#else
        false;
#endif
    if (_serial == nullptr && !lazyOwnedSoftware) {
        _lastError = Error::NOT_INITIALIZED;
        return false;
    }
    if (baud == 0) {
        return false;
    }

    switch (_serialKind) {
        case SerialKind::HARDWARE:
            if (_hardwareSerial == nullptr) {
                _lastError = Error::NOT_INITIALIZED;
                return false;
            }
#if defined(ESP32)
            if (!isHardwareSerialPinValid(_rxPin, _txPin)) {
                _lastError = Error::INVALID_PIN;
                logError(_lastError, "HardwareSerial begin rejected");
                return false;
            }
            _hardwareSerial->begin(baud, SERIAL_8N1, _rxPin, _txPin);
#else
            _hardwareSerial->begin(baud);
#endif
            break;

#if defined(PZEM004_SOFTSERIAL)
        case SerialKind::SOFTWARE:
            // Create an internally-owned port only now, after setup() has
            // started. This is important for EspSoftwareSerial GPIO interrupts.
            if (_softwareSerial == nullptr && _softwareSerialInternal) {
#if defined(ESP32) || defined(ESP8266)
                _ownedSoftwareSerial = new ::SoftwareSerial();
#else
                _ownedSoftwareSerial = new ::SoftwareSerial(
                    static_cast<uint8_t>(_softRxPin),
                    static_cast<uint8_t>(_softTxPin));
#endif
                _softwareSerial = _ownedSoftwareSerial;
            }
            if (_softwareSerial == nullptr) {
                _lastError = Error::NOT_INITIALIZED;
                return false;
            }
#if defined(ESP32) || defined(ESP8266)
            if (_softwareSerialInternal) {
                // Internally-managed instance: explicitly configure pins and
                // use enlarged byte/ISR buffers. Config is always 8N1.
                _softwareSerial->begin(baud, SWSERIAL_8N1,
                                       _softRxPin, _softTxPin, false,
                                       PZEM_SOFT_RX_BUFFER_SIZE,
                                       PZEM_SOFT_ISR_BUFFER_SIZE);
            } else {
                // External instance: preserve pins previously configured on
                // that object (the -1/default pin arguments keep stored pins).
                _softwareSerial->begin(baud, SWSERIAL_8N1);
            }
#else
            _softwareSerial->begin(baud);
#endif
            _serial = _softwareSerial;
            _softwareSerialNeedsBegin = false;
            break;
#endif

        case SerialKind::STREAM:
        case SerialKind::NONE:
        default:
            // A generic Stream does not expose a portable begin() method.
            return false;
    }

    _baudRate = baud;
    cancelPendingRead();
    clearInputBuffer();
    _requestStarted = false;
    _isConnected = false;
    _lastError = Error::NONE;
    return true;
}

#if defined(PZEM004_SOFTSERIAL)
void PZEM004Tv30::listenSoftwareSerial() {
#if defined(__AVR__)
    // AVR SoftwareSerial can listen to only one instance at a time.
    if (_serialKind == SerialKind::SOFTWARE && _softwareSerial != nullptr) {
        _softwareSerial->listen();
    }
#else
    // EspSoftwareSerial enables RX in begin(). Its instances can receive
    // concurrently, so do not switch/listen on every update() call.
    (void)0;
#endif
}

bool PZEM004Tv30::checkSoftwareSerialOverflow() {
#if defined(PZEM004_SOFTSERIAL)
    if (_serialKind == SerialKind::SOFTWARE && _softwareSerial != nullptr) {
        if (_softwareSerial->overflow()) {
            ++_rxOverflowCount;
            _lastError = Error::RX_OVERFLOW;
            logError(_lastError, "SoftwareSerial receive buffer overflow");
            _isConnected = false;
            _rxIndex = 0;
            _rxState = RxState::IDLE;
            // Let the next polling interval start a clean transaction.
            _requestStarted = true;
            _lastRequestTime = millis();
            return true;
        }
    }
#endif
    return false;
}
#endif

void PZEM004Tv30::clearInputBuffer() {
    if (_serial == nullptr) {
        return;
    }
#if defined(PZEM004_SOFTSERIAL)
    listenSoftwareSerial();
#endif

#if (defined(ESP32) || defined(ESP8266)) && defined(PZEM004_SOFTSERIAL)
    if (_serialKind == SerialKind::SOFTWARE && _softwareSerial != nullptr) {
        // EspSoftwareSerial offers a non-blocking bulk read. Avoid its
        // available() path here because that may invoke optimistic_yield().
        uint8_t discard[128];
        _softwareSerial->read(discard, sizeof(discard));
        return;
    }
#endif

    // AVR SoftwareSerial / HardwareSerial: drain bytes already buffered.
    uint16_t drained = 0;
    while (_serial->available() > 0 && drained < 128) {
        if (_serial->read() < 0) break;
        ++drained;
    }
}

void PZEM004Tv30::cancelPendingRead() {
    _rxState = RxState::IDLE;
    _rxIndex = 0;
}

// -----------------------------------------------------------------------------
// Non-blocking measurement polling and cache
// -----------------------------------------------------------------------------

bool PZEM004Tv30::update() {
    // SoftwareSerial::begin() is deliberately lazy: it must run after the
    // Arduino runtime has initialized GPIO/interrupt infrastructure.
    if (!ensureSerialReady()) {
        return false;
    }

    const uint32_t now = millis();
    _lastServiceMillis = now;
    _hasServiced = true;

#if defined(PZEM004_SOFTSERIAL)
    listenSoftwareSerial();
    if (checkSoftwareSerialOverflow()) {
        return false;
    }
#endif

    if (_rxState == RxState::WAITING) {
        bool frameReceived = false;

#if (defined(ESP32) || defined(ESP8266)) && defined(PZEM004_SOFTSERIAL)
        // EspSoftwareSerial::read(buffer, size) is non-blocking and internally
        // runs its edge-to-byte decoder without available()'s yield path.
        // This reduces servicing overhead and avoids repeated available() calls.
        if (_serialKind == SerialKind::SOFTWARE && _softwareSerial != nullptr) {
            uint8_t incoming[32];
            const int count = _softwareSerial->read(incoming, sizeof(incoming));
            for (int i = 0; i < count; ++i) {
                if (pushRxByte(incoming[i])) {
                    frameReceived = true;
                    break;
                }
            }
        } else
#endif
        {
            // HardwareSerial / AVR SoftwareSerial: consume only bytes already
            // present in the receive buffer. Never wait for missing bytes.
            while (_serial->available() > 0) {
                const int value = _serial->read();
                if (value < 0) {
                    break;
                }
                if (pushRxByte(static_cast<uint8_t>(value))) {
                    frameReceived = true;
                    break;
                }
            }
        }

#if defined(PZEM004_SOFTSERIAL)
        if (checkSoftwareSerialOverflow()) {
            return false;
        }
#endif

        if (frameReceived) {
            _rxState = RxState::IDLE;
            _isConnected = true;
            _hasData = true;
            _lastDataTime = millis();
            _lastError = Error::NONE;
            return true;
        }

        if (static_cast<uint32_t>(now - _rxStartTime) >= _responseTimeout) {
            _rxState = RxState::IDLE;
            _rxIndex = 0;
            _isConnected = false;
            _lastError = Error::TIMEOUT;
            logError(_lastError, "Measurement response timeout");
        }
        return false;
    }

    // No pending transaction: initiate a new measurement request when due.
    if (!_requestStarted ||
        static_cast<uint32_t>(now - _lastRequestTime) >= _updateInterval) {
        clearInputBuffer();
        _rxIndex = 0;

        if (!sendCmd8(CMD_RIR, 0x0000, 0x000A, false, USE_CURRENT_ADDRESS)) {
            _isConnected = false;
            _lastError = Error::TX_FAILED;
            logError(_lastError, "Measurement request transmission");
            _requestStarted = true;
            _lastRequestTime = now;
            return false;
        }

        _lastRequestTime = millis();
        _requestStarted = true;
        _rxStartTime = _lastRequestTime;
        _rxState = RxState::WAITING;
    }

    return false;
}

void PZEM004Tv30::requestUpdate() {
    if (_rxState == RxState::IDLE) {
        _requestStarted = false;
    }
}

void PZEM004Tv30::setUpdateInterval(uint32_t intervalMs) {
    // Prevent a zero interval from continuously flooding the PZEM serial port.
    _updateInterval = (intervalMs == 0) ? 1 : intervalMs;
}

void PZEM004Tv30::setResponseTimeout(uint32_t timeoutMs) {
    _responseTimeout = (timeoutMs == 0) ? 1 : timeoutMs;
}

bool PZEM004Tv30::isUpdateDue() const {
    return !_requestStarted ||
           static_cast<uint32_t>(millis() - _lastRequestTime) >= _updateInterval;
}

void PZEM004Tv30::setLogOutput(Stream* output) {
    _logOutput = output;
    if (_logOutput != nullptr && _lastError != Error::NONE &&
        _lastError != Error::NOT_INITIALIZED) {
        logError(_lastError, "Current library state");
    }
}

const char* PZEM004Tv30::errorToString(Error error) {
    switch (error) {
        case Error::NONE: return "No error";
        case Error::NOT_INITIALIZED: return "Serial port is not initialized";
        case Error::TX_FAILED: return "Serial transmission failed";
        case Error::TIMEOUT: return "Response timeout";
        case Error::CRC_ERROR: return "Response CRC mismatch";
        case Error::INVALID_FRAME: return "Invalid response frame";
        case Error::RX_OVERFLOW: return "SoftwareSerial RX buffer overflow";
        case Error::INVALID_PIN: return "Unsupported or invalid RX/TX pin";
        case Error::BEGIN_FAILED: return "Serial port initialization failed";
        default: return "Unknown error";
    }
}

void PZEM004Tv30::logError(Error error, const char* context) {
    if (_logOutput == nullptr || error == Error::NONE) return;
    _logOutput->print(F("[PZEM][ERROR] "));
    if (context != nullptr) {
        _logOutput->print(context);
        _logOutput->print(F(": "));
    }
    _logOutput->print(errorToString(error));
    _logOutput->print(F(" (code "));
    _logOutput->print(static_cast<uint8_t>(error));
    _logOutput->println(F(")"));
}

const char* PZEM004Tv30::boardName() {
#if defined(PZEM_BOARD_ESP32_CLASSIC)
    return "ESP32 classic";
#elif defined(PZEM_BOARD_ESP32_C3)
    return "ESP32-C3";
#elif defined(PZEM_BOARD_ESP32_S3)
    return "ESP32-S3";
#elif defined(PZEM_BOARD_ESP32_S2)
    return "ESP32-S2";
#elif defined(PZEM_BOARD_ESP8285)
    return "ESP8285";
#elif defined(PZEM_BOARD_ESP8266)
    return "ESP8266";
#elif defined(PZEM_BOARD_AVR_UNO)
    return "Arduino Uno";
#elif defined(PZEM_BOARD_AVR_NANO)
    return "Arduino Nano";
#elif defined(PZEM_BOARD_AVR_MEGA)
    return "Arduino Mega 2560";
#elif defined(PZEM_BOARD_AVR)
    return "AVR board";
#else
    return "Unknown board";
#endif
}

bool PZEM004Tv30::decodeMeasurement(const uint8_t* frame) {
    if (!checkCRC(frame, MEASUREMENT_FRAME_SIZE)) {
        _lastError = Error::CRC_ERROR;
        logError(_lastError, "Measurement frame CRC validation");
        return false;
    }

    // Function 0x04, byte count = 20 bytes (10 registers).
    if (frame[0] != _addr || frame[1] != CMD_RIR || frame[2] != 20) {
        _lastError = Error::INVALID_FRAME;
        logError(_lastError, "Measurement frame validation");
        return false;
    }

    const uint32_t rawCurrent =
        (static_cast<uint32_t>(frame[7]) << 24) |
        (static_cast<uint32_t>(frame[8]) << 16) |
        (static_cast<uint32_t>(frame[5]) << 8)  |
        static_cast<uint32_t>(frame[6]);

    const uint32_t rawPower =
        (static_cast<uint32_t>(frame[11]) << 24) |
        (static_cast<uint32_t>(frame[12]) << 16) |
        (static_cast<uint32_t>(frame[9]) << 8)   |
        static_cast<uint32_t>(frame[10]);

    const uint32_t rawEnergy =
        (static_cast<uint32_t>(frame[15]) << 24) |
        (static_cast<uint32_t>(frame[16]) << 16) |
        (static_cast<uint32_t>(frame[13]) << 8)  |
        static_cast<uint32_t>(frame[14]);

    _currentValues.voltage =
        static_cast<float>((static_cast<uint16_t>(frame[3]) << 8) | frame[4]) / 10.0f;
    _currentValues.current = static_cast<float>(rawCurrent) / 1000.0f;
    _currentValues.power = static_cast<float>(rawPower) / 10.0f;
    _currentValues.energy = static_cast<float>(rawEnergy) / 1000.0f;
    _currentValues.frequency =
        static_cast<float>((static_cast<uint16_t>(frame[17]) << 8) | frame[18]) / 10.0f;
    _currentValues.pf =
        static_cast<float>((static_cast<uint16_t>(frame[19]) << 8) | frame[20]) / 100.0f;
    _currentValues.alarms =
        (static_cast<uint16_t>(frame[21]) << 8) | frame[22];

    return true;
}

// Feed one received byte into a simple Modbus response synchronizer.
// Expected response prefix: [slave address][0x04][0x14] followed by 20 data bytes and CRC.
// This recovers alignment if noise or a dropped byte occurs before a later frame.
bool PZEM004Tv30::pushRxByte(uint8_t value) {
    if (_rxIndex == 0) {
        if (value == _addr) {
            _rxBuffer[0] = value;
            _rxIndex = 1;
        }
        return false;
    }

    if (_rxIndex == 1) {
        if (value == CMD_RIR) {
            _rxBuffer[1] = value;
            _rxIndex = 2;
        } else if (value == _addr) {
            // Possible repeated address byte: restart the prefix match.
            _rxBuffer[0] = value;
            _rxIndex = 1;
        } else {
            _rxIndex = 0;
        }
        return false;
    }

    if (_rxIndex == 2) {
        if (value == 20) {
            _rxBuffer[2] = value;
            _rxIndex = 3;
        } else if (value == _addr) {
            _rxBuffer[0] = value;
            _rxIndex = 1;
        } else {
            _rxIndex = 0;
        }
        return false;
    }

    _rxBuffer[_rxIndex++] = value;
    if (_rxIndex < MEASUREMENT_FRAME_SIZE) {
        return false;
    }

    _rxIndex = 0;
    if (!decodeMeasurement(_rxBuffer)) {
        _isConnected = false;
        // Keep scanning any remaining buffered bytes for the next frame prefix.
        return false;
    }
    return true;
}

void PZEM004Tv30::serviceFromGetter() {
    const uint32_t now = millis();
    if (!_hasServiced || now != _lastServiceMillis) {
        update();
    }
}

// -----------------------------------------------------------------------------
// Cached measurement getters - deliberately no serial I/O here
// -----------------------------------------------------------------------------

float PZEM004Tv30::voltage() {
    serviceFromGetter();
    return _hasData ? _currentValues.voltage : NAN;
}

float PZEM004Tv30::current() {
    serviceFromGetter();
    return _hasData ? _currentValues.current : NAN;
}

float PZEM004Tv30::power() {
    serviceFromGetter();
    return _hasData ? _currentValues.power : NAN;
}

float PZEM004Tv30::energy() {
    serviceFromGetter();
    return _hasData ? _currentValues.energy : NAN;
}

float PZEM004Tv30::frequency() {
    serviceFromGetter();
    return _hasData ? _currentValues.frequency : NAN;
}

float PZEM004Tv30::pf() {
    serviceFromGetter();
    return _hasData ? _currentValues.pf : NAN;
}

PZEM004Tv30::Measurements PZEM004Tv30::getData() {
    serviceFromGetter();
    Measurements result;
    result.voltage = _hasData ? _currentValues.voltage : NAN;
    result.current = _hasData ? _currentValues.current : NAN;
    result.power = _hasData ? _currentValues.power : NAN;
    result.energy = _hasData ? _currentValues.energy : NAN;
    result.frequency = _hasData ? _currentValues.frequency : NAN;
    result.pf = _hasData ? _currentValues.pf : NAN;
    result.alarms = _currentValues.alarms;
    result.timestamp = _lastDataTime;
    result.valid = _hasData;
    return result;
}

bool PZEM004Tv30::getPowerAlarm() {
    serviceFromGetter();
    return _hasData && (_currentValues.alarms != 0x0000);
}

// -----------------------------------------------------------------------------
// Blocking control / configuration commands
// These methods are intentionally synchronous because they are infrequent.
// The normal measurement path above remains non-blocking.
// -----------------------------------------------------------------------------

bool PZEM004Tv30::setAddress(uint8_t addr) {
    return setAddress(addr, GENERIC_ADDRESS);
}

bool PZEM004Tv30::setAddress(uint8_t addr, uint8_t targetAddress) {
    if (addr < 0x01 || addr > 0xF7) {
        _lastError = Error::INVALID_FRAME;
        logError(_lastError, "New slave address must be 0x01..0xF7");
        return false;
    }
    if (!((targetAddress >= 0x01 && targetAddress <= 0xF7) ||
          targetAddress == GENERIC_ADDRESS)) {
        _lastError = Error::INVALID_FRAME;
        logError(_lastError, "Target address must be 0x01..0xF7 or 0xF8");
        return false;
    }
    if (!ensureSerialReady()) return false;

    cancelPendingRead();
    clearInputBuffer();

    // Use 0xF8 by default, or the explicit target address supplied by caller.
    if (!sendCmd8(CMD_WSR, WREG_ADDR, addr, true, targetAddress)) {
        if (_lastError == Error::NONE) _lastError = Error::TIMEOUT;
        logError(_lastError, "Setting PZEM slave address");
        _isConnected = false;
        return false;
    }

    _addr = addr;
    _isConnected = true;
    _lastError = Error::NONE;
    _requestStarted = false;
    return true;
}

uint8_t PZEM004Tv30::getAddress() const {
    return _addr;
}

uint8_t PZEM004Tv30::readAddress(bool update) {
    uint8_t address = 0x00;
    if (!readAddress(address, update)) {
        return 0x00; // Legacy INVALID_ADDRESS sentinel.
    }
    return address;
}

bool PZEM004Tv30::readAddress(uint8_t& address, bool update) {
    if (!ensureSerialReady()) {
        return false;
    }

    uint8_t response[7];
    cancelPendingRead();
    clearInputBuffer();

    // Query the PZEM general address. This is NOT Modbus broadcast 0x00.
    if (!sendCmd8(CMD_RHR, WREG_ADDR, 0x0001, false, GENERIC_ADDRESS)) {
        _lastError = Error::TX_FAILED;
        logError(_lastError, "Read address request transmission");
        return false;
    }

    if (receiveBlocking(response, sizeof(response)) != sizeof(response)) {
        _isConnected = false;
        return false; // receiveBlocking has already stored TIMEOUT or CRC_ERROR.
    }

    // Verify the function response header before interpreting the data bytes.
    if (response[0] != GENERIC_ADDRESS || response[1] != CMD_RHR ||
        response[2] != 0x02) {
        _lastError = Error::INVALID_FRAME;
        logError(_lastError, "Read address response header");
        _isConnected = false;
        return false;
    }

    const uint16_t rawAddress =
        (static_cast<uint16_t>(response[3]) << 8) | response[4];

    // The holding register stores a custom slave address (0x0001..0x00F7).
    // 0x00 is not a valid Modbus slave address. The device may nevertheless
    // respond to 0xF8, its PZEM general address, in a single-device setup.
    if (rawAddress < 0x01 || rawAddress > 0xF7) {
        _lastError = Error::INVALID_FRAME;
        logError(_lastError, "Custom address register value");
        _isConnected = true; // A valid response frame arrived; its value is invalid.
        return false;
    }

    address = static_cast<uint8_t>(rawAddress);
    if (update) {
        _addr = address;
    }

    _isConnected = true;
    _lastError = Error::NONE;
    _requestStarted = false;
    return true;
}

bool PZEM004Tv30::setPowerAlarm(uint16_t watts) {
    if (!ensureSerialReady()) {
        return false;
    }
    if (watts > 25000) {
        watts = 25000;
    }

    cancelPendingRead();
    clearInputBuffer();
    const bool result = sendCmd8(CMD_WSR, WREG_ALARM_THR, watts,
                                 true, USE_CURRENT_ADDRESS);
    _requestStarted = false;
    if (!result) {
        _isConnected = false;
        _lastError = Error::TIMEOUT;
        logError(_lastError, "Setting power alarm");
    } else {
        _isConnected = true;
        _lastError = Error::NONE;
    }
    return result;
}

bool PZEM004Tv30::resetEnergy() {
    if (!ensureSerialReady()) {
        return false;
    }

    cancelPendingRead();
    clearInputBuffer();
#if defined(PZEM004_SOFTSERIAL)
    listenSoftwareSerial();
#endif

    // The reset command is a 4-byte frame and commonly has no reply.
    uint8_t frame[4] = {_addr, CMD_REST, 0x00, 0x00};
    setCRC(frame, sizeof(frame));
    const size_t written = _serial->write(frame, sizeof(frame));
    _requestStarted = false;

    if (written != sizeof(frame)) {
        _isConnected = false;
        _lastError = Error::TX_FAILED;
        logError(_lastError, "Reset energy command transmission");
        return false;
    }

    _lastError = Error::NONE;
    return true;
}

// -----------------------------------------------------------------------------
// Modbus frame helpers
// -----------------------------------------------------------------------------

bool PZEM004Tv30::sendCmd8(uint8_t cmd, uint16_t regAddr, uint16_t value,
                           bool check, uint8_t slaveAddr) {
    if (!ensureSerialReady()) {
        return false;
    }

    // 0xFF is an internal sentinel meaning "use this object's current address".
    // 0x00 (broadcast) and 0xF8 (generic) are valid explicit target addresses.
    if (slaveAddr == USE_CURRENT_ADDRESS) {
        slaveAddr = _addr;
    } else if (!((slaveAddr >= 0x01 && slaveAddr <= 0xF7) ||
                 slaveAddr == GENERIC_ADDRESS ||
                 slaveAddr == BROADCAST_ADDRESS)) {
        return false;
    }

    uint8_t sendBuffer[MODBUS_FRAME_SIZE] = {
        slaveAddr,
        cmd,
        static_cast<uint8_t>((regAddr >> 8) & 0xFF),
        static_cast<uint8_t>(regAddr & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>(value & 0xFF),
        0x00,
        0x00
    };
    setCRC(sendBuffer, sizeof(sendBuffer));

#if defined(PZEM004_SOFTSERIAL)
    listenSoftwareSerial();
#endif
    const size_t written = _serial->write(sendBuffer, sizeof(sendBuffer));
    if (written != sizeof(sendBuffer)) {
        _lastError = Error::TX_FAILED;
        logError(_lastError, "Modbus command transmission");
        return false;
    }

    if (check) {
        uint8_t response[MODBUS_FRAME_SIZE];
        if (receiveBlocking(response, sizeof(response)) != sizeof(response)) {
            _lastError = Error::TIMEOUT;
            return false;
        }

        for (uint8_t i = 0; i < MODBUS_FRAME_SIZE; ++i) {
            if (sendBuffer[i] != response[i]) {
                _lastError = Error::INVALID_FRAME;
                return false;
            }
        }
    }

    return true;
}

uint16_t PZEM004Tv30::receiveBlocking(uint8_t* response, uint16_t length,
                                     uint32_t timeoutMs) {
    if (_serial == nullptr || response == nullptr || length == 0) {
        return 0;
    }

#if defined(PZEM004_SOFTSERIAL)
    listenSoftwareSerial();
#endif

    const uint32_t startTime = millis();
    uint16_t index = 0;

    while (index < length &&
           static_cast<uint32_t>(millis() - startTime) < timeoutMs) {
        while (_serial->available() > 0 && index < length) {
            const int value = _serial->read();
            if (value >= 0) {
                response[index++] = static_cast<uint8_t>(value);
            }
        }
        PZEM_YIELD();
    }

    if (index != length) {
        _isConnected = false;
        _lastError = Error::TIMEOUT;
        logError(_lastError, "Synchronous command response");
        return 0;
    }
    if (!checkCRC(response, length)) {
        _isConnected = false;
        _lastError = Error::CRC_ERROR;
        logError(_lastError, "Synchronous command CRC validation");
        return 0;
    }

    _isConnected = true;
    return index;
}

bool PZEM004Tv30::checkCRC(const uint8_t* buffer, uint16_t length) {
    if (buffer == nullptr || length <= 2) {
        return false;
    }
    const uint16_t crc = CRC16(buffer, length - 2);
    const uint16_t receivedCRC =
        static_cast<uint16_t>(buffer[length - 2]) |
        (static_cast<uint16_t>(buffer[length - 1]) << 8);
    return receivedCRC == crc;
}

void PZEM004Tv30::setCRC(uint8_t* buffer, uint16_t length) {
    if (buffer == nullptr || length <= 2) {
        return;
    }
    const uint16_t crc = CRC16(buffer, length - 2);
    buffer[length - 2] = static_cast<uint8_t>(crc & 0xFF);
    buffer[length - 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);
}

static const uint16_t crcTable[] PROGMEM = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
};

uint16_t PZEM004Tv30::CRC16(const uint8_t* data, uint16_t length) {
    uint8_t tableIndex;
    uint16_t crc = 0xFFFF;
    while (length--) {
        tableIndex = static_cast<uint8_t>(*data++ ^ crc);
        crc >>= 8;
        crc ^= static_cast<uint16_t>(pgm_read_word(&crcTable[tableIndex]));
    }
    return crc;
}

// -----------------------------------------------------------------------------
// Board-specific GPIO capability checks
// -----------------------------------------------------------------------------

namespace {

#if defined(ESP32)
bool isEsp32HardwareInput(int pin) {
#if defined(PZEM_BOARD_ESP32_C3)
    // GPIO2, GPIO8, and GPIO9 are boot-strapping pins. GPIO11-17 are
    // reserved for flash/VDD_SPI on common ESP32-C3 modules.
    const bool supported = pin >= 0 && pin <= 21 &&
                           !(pin >= 11 && pin <= 17) &&
                           pin != 2 && pin != 8 && pin != 9;
  #if !defined(PZEM004_ALLOW_USB_GPIO)
    return supported && pin != 18 && pin != 19;
  #else
    return supported;
  #endif
#elif defined(PZEM_BOARD_ESP32_S3)
    // Exclude flash/PSRAM pins and boot-strapping pins by default.
    const bool supported = pin >= 0 && pin <= 48 &&
                           !(pin >= 26 && pin <= 37) &&
                           pin != 0 && pin != 3 && pin != 45;
  #if !defined(PZEM004_ALLOW_USB_GPIO)
    return supported && pin != 19 && pin != 20;
  #else
    return supported;
  #endif
#elif defined(PZEM_BOARD_ESP32_S2)
    // Exclude flash pins and the main boot-strapping pins.
    const bool supported = pin >= 0 && pin <= 46 &&
                           !(pin >= 26 && pin <= 32) &&
                           pin != 0 && pin != 45;
  #if !defined(PZEM004_ALLOW_USB_GPIO)
    return supported && pin != 19 && pin != 20;
  #else
    return supported;
  #endif
#else
    // ESP32 classic: avoid flash pins and common strapping pins.
    return pin >= 0 && pin <= 39 && !(pin >= 6 && pin <= 11) &&
           pin != 0 && pin != 2 && pin != 5 && pin != 12 && pin != 15;
#endif
}

bool isEsp32HardwareOutput(int pin) {
#if defined(PZEM_BOARD_ESP32_CLASSIC)
    return isEsp32HardwareInput(pin) && pin <= 33;
#elif defined(PZEM_BOARD_ESP32_S2) || defined(PZEM_BOARD_ESP32_S3)
    return isEsp32HardwareInput(pin) && pin != 46;
#else
    return isEsp32HardwareInput(pin);
#endif
}

bool isEsp32SoftwareInput(int pin) {
#if defined(PZEM_BOARD_ESP32_CLASSIC)
    // Keep this list conservative: exclude flash, console UART, and common
    // strapping pins while retaining GPIO18/19 used by the tested setup.
    return pin == 4 || (pin >= 13 && pin <= 14) ||
           (pin >= 18 && pin <= 19) ||
           (pin >= 21 && pin <= 23) || (pin >= 25 && pin <= 27) ||
           (pin >= 32 && pin <= 39);
#elif defined(PZEM_BOARD_ESP32_C3)
    const bool supported = (pin >= 0 && pin <= 1) ||
                           (pin >= 3 && pin <= 7) ||
                           (pin >= 18 && pin <= 21);
  #if !defined(PZEM004_ALLOW_USB_GPIO)
    return supported && pin != 18 && pin != 19;
  #else
    return supported;
  #endif
#elif defined(PZEM_BOARD_ESP32_S2)
    const bool supported = (pin >= 1 && pin <= 21) ||
                           (pin >= 33 && pin <= 44);
  #if !defined(PZEM004_ALLOW_USB_GPIO)
    return supported && pin != 19 && pin != 20;
  #else
    return supported;
  #endif
#elif defined(PZEM_BOARD_ESP32_S3)
    const bool supported = (pin >= 1 && pin <= 2) ||
                           (pin >= 4 && pin <= 18) ||
                           (pin >= 21 && pin <= 25) ||
                           (pin >= 38 && pin <= 48);
  #if !defined(PZEM004_ALLOW_USB_GPIO)
    return supported && pin != 19 && pin != 20 && pin != 45 && pin != 46;
  #else
    return supported && pin != 45 && pin != 46;
  #endif
#else
    (void)pin;
    return false;
#endif
}

bool isEsp32SoftwareOutput(int pin) {
    if (!isEsp32SoftwareInput(pin)) return false;
#if defined(PZEM_BOARD_ESP32_CLASSIC)
    return pin <= 33;
#elif defined(PZEM_BOARD_ESP32_S2)
    return pin <= 45;
#elif defined(PZEM_BOARD_ESP32_S3)
    return pin != 46;
#else
    return true;
#endif
}
#endif // ESP32

#if defined(ESP8266)
bool isEsp8266GpioInput(int pin) {
    // GPIO0, GPIO2, and GPIO15 are boot-strapping pins; GPIO16 lacks the
    // normal GPIO interrupt capability required by SoftwareSerial RX.
    return pin >= 0 && pin <= 15 && !(pin >= 6 && pin <= 11) &&
           pin != 0 && pin != 2 && pin != 15;
}

bool isEsp8266GpioOutput(int pin) {
    return pin >= 0 && pin <= 16 && !(pin >= 6 && pin <= 11) &&
           pin != 0 && pin != 2 && pin != 15;
}
#endif // ESP8266

#if defined(__AVR__)
bool isAvrDigitalPin(int pin) {
    if (pin < 0) return false;
#if defined(digitalPinToPort) && defined(NOT_A_PIN)
    return digitalPinToPort(pin) != NOT_A_PIN;
#else
    #if defined(PZEM_BOARD_AVR_MEGA)
      return pin <= 69;
    #else
      return pin <= 19;
    #endif
#endif
}

bool isAvrSoftwareRxPin(int pin) {
    if (!isAvrDigitalPin(pin)) return false;
#if defined(digitalPinToPCICR)
    return digitalPinToPCICR(pin) != nullptr;
#else
    // The common Uno/Nano receive pins expose pin-change interrupts.
    #if defined(PZEM_BOARD_AVR_MEGA)
      return (pin >= 10 && pin <= 15) || (pin >= 50 && pin <= 53) ||
             (pin >= 62 && pin <= 69);
    #else
      return pin >= 0 && pin <= 19;
    #endif
#endif
}
#endif // AVR

} // namespace

bool PZEM004Tv30::isSoftwareSerialPinValid(int receivePin, int transmitPin) {
#ifdef PZEM004_DISABLE_PIN_FILTER
    return receivePin >= 0 && transmitPin >= 0 && receivePin != transmitPin;
#else
    if (receivePin < 0 || transmitPin < 0 || receivePin == transmitPin) return false;
#if defined(ESP32)
    return isEsp32SoftwareInput(receivePin) &&
           isEsp32SoftwareOutput(transmitPin);
#elif defined(ESP8266)
    return isEsp8266GpioInput(receivePin) && isEsp8266GpioOutput(transmitPin);
#elif defined(__AVR__)
    return isAvrSoftwareRxPin(receivePin) && isAvrDigitalPin(transmitPin);
#else
    return true;
#endif
#endif
}

bool PZEM004Tv30::isHardwareSerialPinValid(int receivePin, int transmitPin) {
#ifdef PZEM004_DISABLE_PIN_FILTER
    return receivePin >= 0 && transmitPin >= 0 && receivePin != transmitPin;
#else
    if (receivePin < 0 || transmitPin < 0 || receivePin == transmitPin) return false;
#if defined(ESP32)
    return isEsp32HardwareInput(receivePin) &&
           isEsp32HardwareOutput(transmitPin);
#elif defined(ESP8266)
    // UART0 default pins (GPIO3/GPIO1). Swapped UART0 uses GPIO15,
    // a boot-strapping pin, so it is intentionally not accepted by default.
    return receivePin == 3 && transmitPin == 1;
#elif defined(__AVR__)
  #if defined(PZEM_BOARD_AVR_MEGA)
    return (receivePin == 0 && transmitPin == 1) ||
           (receivePin == 19 && transmitPin == 18) ||
           (receivePin == 17 && transmitPin == 16) ||
           (receivePin == 15 && transmitPin == 14);
  #else
    return receivePin == 0 && transmitPin == 1;
  #endif
#else
    return true;
#endif
#endif
}

// -----------------------------------------------------------------------------
// Diagnostic address search (blocking; use only during commissioning)
// -----------------------------------------------------------------------------

void PZEM004Tv30::search() {
#ifndef PZEM004T_DISABLE_SEARCH
    if (_serial == nullptr) {
        return;
    }

    uint8_t response[7];
    cancelPendingRead();

    for (uint16_t address = 0x01; address <= 0xF7; ++address) {
        clearInputBuffer();
        if (!sendCmd8(CMD_RIR, 0x0000, 0x0001, false,
                      static_cast<uint8_t>(address))) {
            continue;
        }

        if (receiveBlocking(response, sizeof(response)) == sizeof(response)) {
            Serial.print(F("PZEM device on address: 0x"));
            Serial.println(address, HEX);
        }
        PZEM_YIELD();
    }
    _requestStarted = false;
#endif
}
