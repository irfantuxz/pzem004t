# Changelog

## 2.0.0 - Fork release

This release continues the work of the upstream PZEM-004T v3.0 Arduino library by Jakub Mandula, version 1.2.1. The upstream implementation and MIT license notice are retained; this fork adds the following changes.

### Added

- SoftwareSerial support for AVR boards (Uno, Nano, Mega), ESP8266/ESP8285, and ESP32 classic, ESP32-C3, ESP32-S2, and ESP32-S3.
- ESP32 SoftwareSerial integration through the external EspSoftwareSerial library by plerup.
- `SoftSerial` constructor tag as a shorter alias for `PZEM004T::SoftwareSerial`.
- Board-family detection macros: `PZEM_BOARD_ESP32_CLASSIC`, `PZEM_BOARD_ESP32_C3`, `PZEM_BOARD_ESP32_S2`, `PZEM_BOARD_ESP32_S3`, `PZEM_BOARD_ESP8266`, `PZEM_BOARD_ESP8285`, `PZEM_BOARD_AVR_UNO`, `PZEM_BOARD_AVR_NANO`, and `PZEM_BOARD_AVR_MEGA`.
- SoftwareSerial and HardwareSerial GPIO capability checks with conservative exclusions for flash, USB, input-only, and strapping pins.
- Optional runtime error logging through `setLogOutput()`, structured `Error` enum codes via `getLastError()`, and readable descriptions via `errorToString()`.
- `readAddress(uint8_t& address)` overload to distinguish a failed read from a returned address value.
- `setAddress(newAddress, targetAddress)` for explicit address targeting. The one-argument overload uses the PZEM general address `0xF8`.
- `isBusy()`, `isUpdateDue()`, `hasData()`, `isConnected()`, and `lastUpdateMillis()` helpers for non-blocking state tracking and sequential polling on shared UARTs.
- Automatic shared `HardwareSerial` state tracking on ESP32 so multiple PZEM objects can share a single UART without duplicate `begin()` calls or RX buffer flushes.
- Source-compatibility `PZEM004Tv30` type alias for sketches written against the upstream class name.
- Six examples covering HardwareSerial, SoftwareSerial, non-blocking polling, address configuration, a shared UART bus, and three independent serial ports.

### Changed

- Measurement polling is non-blocking. `update()` sends a request when due and reads only bytes already available.
- Measurement getters return cached values and automatically service `update()` at most once per millisecond as a backward-compatibility fallback.
- SoftwareSerial construction and initialization are deferred until runtime to avoid attaching GPIO interrupts inside global constructors.
- The address command path explicitly supports the PZEM general address `0xF8` and separates it from Modbus broadcast address `0x00`.
- ESP32 HardwareSerial constructors validate the RX/TX pin pair before starting the UART.

### Fixed

- Fixed address-setting and address-reading request handling so the generic address `0xF8` is passed explicitly where required.
- Fixed handling of explicit slave addresses `0x00` and `0xF8` in the internal command sender.
- Added frame-prefix synchronization and CRC16 validation for measurement responses.
- Added RX overflow detection and reporting (`getRxOverflowCount()`) for supported SoftwareSerial implementations.

### Compatibility notes

- Default serial format remains 9600 baud, 8 data bits, no parity, 1 stop bit (8N1).
- `readAddress()` returning `0x00` remains the legacy failure sentinel. Prefer `readAddress(address)` for unambiguous status.
- On AVR, SoftwareSerial can listen to only one instance at a time. Use HardwareSerial ports on Mega when available.
- Multiple PZEM devices sharing one UART must have unique addresses and must be polled sequentially. Do not issue concurrent requests on the same shared UART.
- ESP32 SoftwareSerial requires EspSoftwareSerial by plerup unless `PZEM004_NO_SWSERIAL` is defined before including `PZEM004T.h`.

## 2.0.1 - Update

### Minor fix and metadata
