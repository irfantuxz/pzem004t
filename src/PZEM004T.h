/*
 * PZEM004T.h
 *
 * Fork of the PZEM-004T v3.0 Arduino library by Jakub Mandula version 1.2.1.
 * Fork version: 2.0.1
 *
 * Copyright (c) 2026 Irfan_TuxZ
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

#ifndef PZEM004T_H
#define PZEM004T_H

#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

// -----------------------------------------------------------------------------
// Board-family detection macros
// -----------------------------------------------------------------------------

#if defined(ESP32)
  #if defined(CONFIG_IDF_TARGET_ESP32C3)
    #define PZEM_BOARD_ESP32_C3 1
  #elif defined(CONFIG_IDF_TARGET_ESP32S3)
    #define PZEM_BOARD_ESP32_S3 1
  #elif defined(CONFIG_IDF_TARGET_ESP32S2)
    #define PZEM_BOARD_ESP32_S2 1
  #else
    #define PZEM_BOARD_ESP32_CLASSIC 1
  #endif
#elif defined(ESP8266)
  #define PZEM_BOARD_ESP8266 1
  #if defined(ESP8285)
    #define PZEM_BOARD_ESP8285 1
  #endif
#elif defined(__AVR__)
  #if defined(ARDUINO_AVR_UNO)
    #define PZEM_BOARD_AVR_UNO 1
  #elif defined(ARDUINO_AVR_NANO)
    #define PZEM_BOARD_AVR_NANO 1
  #elif defined(ARDUINO_AVR_MEGA2560)
    #define PZEM_BOARD_AVR_MEGA 1
  #else
    #define PZEM_BOARD_AVR 1
  #endif
#endif

// -----------------------------------------------------------------------------
// Optional SoftwareSerial support
// -----------------------------------------------------------------------------
// AVR boards use Arduino SoftwareSerial. ESP8266 uses the implementation
// provided by its Arduino core. ESP32 boards require EspSoftwareSerial by
// plerup. Define PZEM004_NO_SWSERIAL to build without software UART support.

#ifndef PZEM004_NO_SWSERIAL
  #if defined(__AVR__) || defined(ESP8266) || defined(ESP32)
    // Keep this as a direct include. Arduino's library discovery scans include
    // directives recursively; __has_include() alone can prevent discovery of
    // EspSoftwareSerial when its include path has not been added yet.
    // On ESP32, install the EspSoftwareSerial library by plerup.
    #include <SoftwareSerial.h>
    #define PZEM004_SOFTSERIAL 1
  #endif
#endif

#define PZEM_DEFAULT_ADDR             0xF8
#define PZEM_BAUD_RATE                9600UL
#define PZEM_DEFAULT_INTERVAL         1000UL
#define PZEM_DEFAULT_TIMEOUT          150UL
#define PZEM_SOFT_DEFAULT_TIMEOUT     300UL
#define PZEM_SOFT_RX_BUFFER_SIZE      128
#define PZEM_SOFT_ISR_BUFFER_SIZE     1280

class PZEM004T {
public:
    struct SoftwareSerialTag {};

#if defined(PZEM004_SOFTSERIAL)
    static const SoftwareSerialTag SoftwareSerial;

    // Preferred form: the library owns and initializes the software UART.
    PZEM004T(const SoftwareSerialTag& tag, uint8_t receivePin,
                uint8_t transmitPin, uint8_t addr = PZEM_DEFAULT_ADDR);

    // Legacy pin-based constructor, retained for compatibility.
    PZEM004T(uint8_t receivePin, uint8_t transmitPin,
                uint8_t addr = PZEM_DEFAULT_ADDR);

    // Use an application-owned SoftwareSerial object.
    PZEM004T(::SoftwareSerial& port, uint8_t addr = PZEM_DEFAULT_ADDR);
#endif

    // Generic Stream constructor. For backward compatibility, a Stream cast
    // of a SoftwareSerial object is treated as a SoftwareSerial port.
    PZEM004T(Stream& port, uint8_t addr = PZEM_DEFAULT_ADDR);

#if defined(ESP32)
    // ESP32 UART constructor. The library initializes the UART on first use.
    // Multiple PZEM objects may share the same HardwareSerial instance when
    // they use the same RX/TX pins and baud rate. Do not call port.begin()
    // separately; the library tracks initialization for shared PZEM objects.
    PZEM004T(HardwareSerial& port, int8_t receivePin, int8_t transmitPin,
                uint8_t addr = PZEM_DEFAULT_ADDR);
#else
    PZEM004T(HardwareSerial& port, uint8_t addr = PZEM_DEFAULT_ADDR);
#endif

    PZEM004T();
    ~PZEM004T();

    PZEM004T(const PZEM004T&) = delete;
    PZEM004T& operator=(const PZEM004T&) = delete;

    // Serial configuration uses 9600 baud and 8N1 by default.
    bool begin(uint32_t baud = PZEM_BAUD_RATE);
    bool setBaudRate(uint32_t baud) { return begin(baud); }

    // Call frequently from loop(). Returns true only when a new valid frame
    // has been received and decoded.
    bool update();
    void requestUpdate();
    void setUpdateInterval(uint32_t intervalMs);
    uint32_t getUpdateInterval() const { return _updateInterval; }
    void setResponseTimeout(uint32_t timeoutMs);

    // Cached getters also service update() once per millisecond as a fallback
    // for sketches that do not explicitly call update().
    float voltage();
    float current();
    float power();
    float energy();
    float frequency();
    float pf();

    struct Measurements {
        float voltage;
        float current;
        float power;
        float energy;
        float frequency;
        float pf;
        uint16_t alarms;
        uint32_t timestamp;
        bool valid;
    };

    Measurements getData();
    bool hasData() const { return _hasData; }
    bool isDataReady() const { return _hasData; }
    bool isConnected() const { return _isConnected; }
    bool isBusy() const { return _rxState == RxState::WAITING; }
    bool isUpdateDue() const;
    uint32_t lastUpdateMillis() const { return _lastDataTime; }

    enum class Error : uint8_t {
        NONE = 0,
        NOT_INITIALIZED,
        TX_FAILED,
        TIMEOUT,
        CRC_ERROR,
        INVALID_FRAME,
        RX_OVERFLOW,
        INVALID_PIN,
        BEGIN_FAILED
    };

    Error getLastError() const { return _lastError; }
    uint32_t getRxOverflowCount() const { return _rxOverflowCount; }
    static const char* errorToString(Error error);
    static const char* boardName();

    // Optional diagnostic output. Pass &Serial (or another Stream) to enable
    // messages prefixed with [PZEM][ERROR]. Logging is disabled by default.
    void setLogOutput(Stream* output);

    // Pin capability checks. Pins are raw GPIO numbers, not board aliases
    // such as D5, unless those aliases resolve to the intended GPIO number.
    static bool isSoftwareSerialPinValid(int receivePin, int transmitPin);
    static bool isHardwareSerialPinValid(int receivePin, int transmitPin);

    // setAddress(newAddress) writes through the PZEM general address 0xF8.
    // setAddress(newAddress, targetAddress) allows an explicit current address.
    bool setAddress(uint8_t addr);
    bool setAddress(uint8_t addr, uint8_t targetAddress);

    // Reads the meter's custom-address register through the PZEM general
    // address (0xF8). This operation reports the stored address but never
    // changes this object's communication target (_addr).
    // Use the bool overload to distinguish a valid address from a communication
    // error; inspect getLastError() when it returns false.
    // The uint8_t overload returns 0x00 on failure.
    uint8_t readAddress();
    bool readAddress(uint8_t& address);

    bool setPowerAlarm(uint16_t watts);
    bool getPowerAlarm();
    bool resetEnergy();
    void search();

private:
    enum class SerialKind : uint8_t { NONE = 0, HARDWARE, SOFTWARE, STREAM };
    enum class RxState : uint8_t { IDLE = 0, WAITING };

    struct CurrentValues {
        float voltage;
        float current;
        float power;
        float energy;
        float frequency;
        float pf;
        uint16_t alarms;
    };

    Stream* _serial;
    HardwareSerial* _hardwareSerial;
#if defined(ESP32)
    bool _hardwareSerialNeedsBegin;
    bool _hardwareSerialRegistered;
#endif
#if defined(PZEM004_SOFTSERIAL)
    ::SoftwareSerial* _softwareSerial;
    ::SoftwareSerial* _ownedSoftwareSerial;
    bool _softwareSerialInternal;
    bool _softwareSerialNeedsBegin;
#endif
    SerialKind _serialKind;
    uint8_t _addr;
    bool _isConnected;
    bool _hasData;
    bool _pinConfigValid;

    int8_t _rxPin;
    int8_t _txPin;
    int8_t _softRxPin;
    int8_t _softTxPin;
    uint32_t _baudRate;

    CurrentValues _currentValues;
    uint32_t _updateInterval;
    uint32_t _responseTimeout;
    uint32_t _lastRequestTime;
    uint32_t _lastDataTime;
    bool _requestStarted;
    uint32_t _lastServiceMillis;
    bool _hasServiced;
    uint32_t _rxOverflowCount;

    RxState _rxState;
    uint32_t _rxStartTime;
    uint8_t _rxBuffer[25];
    uint8_t _rxIndex;
    Error _lastError;
    Stream* _logOutput;

    void init(Stream* port, SerialKind kind, uint8_t addr);
    bool ensureSerialReady();
#if defined(PZEM004_SOFTSERIAL)
    void initSoftwareSerial(uint8_t receivePin, uint8_t transmitPin,
                            uint8_t addr);
    void listenSoftwareSerial();
#endif
    void clearInputBuffer();
    void cancelPendingRead();
    bool decodeMeasurement(const uint8_t* frame);
    bool pushRxByte(uint8_t value);
    void serviceFromGetter();
    bool checkSoftwareSerialOverflow();
    void logError(Error error, const char* context);

    bool sendCmd8(uint8_t cmd, uint16_t regAddr, uint16_t value,
                  bool check = false, uint8_t slaveAddr = 0xFF);
    uint16_t receiveBlocking(uint8_t* response, uint16_t length,
                             uint32_t timeoutMs = PZEM_DEFAULT_TIMEOUT);
    void setCRC(uint8_t* buffer, uint16_t length);
    bool checkCRC(const uint8_t* buffer, uint16_t length);
    uint16_t CRC16(const uint8_t* data, uint16_t length);
};

// Source-compatibility alias for sketches written against the former class
// name. This is the same type, not a second class implementation.
typedef PZEM004T PZEM004Tv30;

// Compact constructor tag: PZEM004T pzem(SoftSerial, rxPin, txPin);
#if defined(PZEM004_SOFTSERIAL)
  #ifndef SoftSerial
    #define SoftSerial PZEM004T::SoftwareSerial
  #endif
#endif

#endif // PZEM004T_H
