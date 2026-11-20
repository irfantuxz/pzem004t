# PZEM004T - Version 2.0.0

An extended fork of the PZEM-004T v3.0 Arduino library by Jakub Mandula. This release continues from upstream version **1.2.1** and retains the original MIT license notice.

## Fork changes in 2.0.0

1. Fixed address handling in `setAddress()` and `readAddress()`, including explicit use of the PZEM general address `0xF8`.
2. Added SoftwareSerial support for ESP32 families via **EspSoftwareSerial by plerup**.
3. Added automatic shared UART management on ESP32 and examples for polling multiple PZEM devices on a shared UART bus.
4. Added non-blocking measurement polling and cached measurement getters.
5. Added board-family detection macros and conservative GPIO validation for SoftwareSerial and ESP32 HardwareSerial pins.
6. Added optional runtime error logging, structured error codes (`getLastError()`), and readable error labels (`errorToString()`).
7. Added `SoftSerial` as a compact constructor tag and retained `PZEM004Tv30` as a type alias for backward compatibility.
8. Added six examples for common wiring and application patterns.
9. Renamed the primary class to `PZEM004T` for compatibility across PZEM-004T v3.0 and v4.0 hardware revisions.

## Supported board families

The library detects these board families at compile time:

| Detection macro | Board family |
|---|---|
| `PZEM_BOARD_ESP32_CLASSIC` | ESP32 classic / original ESP32 |
| `PZEM_BOARD_ESP32_C3` | ESP32-C3 |
| `PZEM_BOARD_ESP32_S2` | ESP32-S2 |
| `PZEM_BOARD_ESP32_S3` | ESP32-S3 |
| `PZEM_BOARD_ESP8266` | ESP8266 family |
| `PZEM_BOARD_ESP8285` | ESP8285 variant, when exposed by the core |
| `PZEM_BOARD_AVR_UNO` | Arduino Uno |
| `PZEM_BOARD_AVR_NANO` | Arduino Nano |
| `PZEM_BOARD_AVR_MEGA` | Arduino Mega 2560 |

The ESP8266 family includes common ESP8266EX and ESP8285-based boards.

## Dependencies

- Arduino framework for the selected board.
- **ESP32 SoftwareSerial:** install [EspSoftwareSerial by plerup](https://github.com/plerup/espsoftwareserial). If it is absent, compilation will fail on `<SoftwareSerial.h>` unless `PZEM004_NO_SWSERIAL` is defined.
- **ESP8266:** SoftwareSerial implementation supplied by the ESP8266 Arduino core.
- **AVR Uno/Nano/Mega:** Arduino SoftwareSerial supplied by the AVR core.

To disable SoftwareSerial support explicitly (for example, on ESP32 when using only `HardwareSerial` without installing `EspSoftwareSerial`), define `PZEM004_NO_SWSERIAL` before including `PZEM004T.h`:

```cpp
#define PZEM004_NO_SWSERIAL
#include <PZEM004T.h>
```

## Installation

1. Download this repository as a ZIP archive.
2. In Arduino IDE, choose **Sketch > Include Library > Add .ZIP Library...**.
3. Select the downloaded ZIP.
4. Install **EspSoftwareSerial** via Library Manager when compiling for ESP32 (unless `PZEM004_NO_SWSERIAL` is defined).
5. Open one of the examples under **File > Examples > PZEM004T**.

## Serial defaults

- Baud rate: `9600` (`PZEM_BAUD_RATE`)
- Data format: `8N1`
- Default device address: `0xF8` (`PZEM_DEFAULT_ADDR`, PZEM general/default address)
- Default measurement interval: `1000 ms` (`PZEM_DEFAULT_INTERVAL`)
- Default HardwareSerial response timeout: `150 ms` (`PZEM_DEFAULT_TIMEOUT`)
- Default SoftwareSerial response timeout: `300 ms` (`PZEM_SOFT_DEFAULT_TIMEOUT`)

## Constructor examples

### ESP32 HardwareSerial

```cpp
#include <PZEM004T.h>

PZEM004T pzem(Serial2, 16, 17);
```

### SoftwareSerial on ESP32

```cpp
#include <PZEM004T.h>

PZEM004T pzem(SoftSerial, 18, 19);
```

`SoftSerial` is a macro alias for `PZEM004T::SoftwareSerial`.

### SoftwareSerial on AVR Uno/Nano

```cpp
#include <PZEM004T.h>

PZEM004T pzem(SoftSerial, 10, 11);
```

### External SoftwareSerial object

```cpp
#include <PZEM004T.h>
#include <SoftwareSerial.h>

SoftwareSerial pzemPort;
PZEM004T pzem(pzemPort);

void setup() {
  Serial.begin(115200);
  pzemPort.begin(9600, SWSERIAL_8N1, 18, 19);
}
```

The library initializes an externally supplied `SoftwareSerial` object lazily on the first service call. On ESP32/ESP8266, it preserves RX/TX pins already configured in the object.

## Non-blocking measurement polling

Call `update()` frequently from `loop()`:

```cpp
void loop() {
  if (pzem.update()) {
    const PZEM004T::Measurements data = pzem.getData();
    Serial.println(data.voltage);
  }
}
```

`update()` sends a request when the configured interval expires and consumes only bytes currently buffered. It does not block waiting for a response, returning `true` only when a new valid measurement frame has been decoded.

The measurement getters (`voltage()`, `current()`, `power()`, `energy()`, `frequency()`, and `pf()`) read the cache and service `update()` at most once per millisecond as a compatibility fallback for sketches that do not explicitly call `update()`. Before the first valid frame, getters return `NAN`.

## Address handling

The PZEM general address `0xF8` is distinct from Modbus broadcast address `0x00`:

- `0x01` to `0xF7`: valid custom slave addresses.
- `0xF8`: PZEM general/default address used for single-device commissioning and generic access.
- `0x00`: Modbus broadcast address; a broadcast request does not return a response.

Read a stored custom address using the boolean overload:

```cpp
uint8_t address = 0;
if (pzem.readAddress(address)) {
  Serial.print("Address: 0x");
  Serial.println(address, HEX);
} else {
  Serial.println(PZEM004T::errorToString(pzem.getLastError()));
}
```

Set an address using the general address `0xF8`:

```cpp
if (pzem.setAddress(0x02)) {
  Serial.println("Address write acknowledged");
}
```

Or explicitly target a currently configured device:

```cpp
pzem.setAddress(0x02, 0x01);  // Change device 0x01 to 0x02
```

**Important:** when using `0xF8`, connect only one PZEM to the communication bus during commissioning. Multiple devices on the same bus must have unique addresses before being polled together.

## Pin validation and board-specific notes

The library exposes:

```cpp
PZEM004T::isSoftwareSerialPinValid(rxPin, txPin);
PZEM004T::isHardwareSerialPinValid(rxPin, txPin);
PZEM004T::boardName();
```

The filter is conservative and checks common GPIO restrictions:

- **ESP32 classic:** excludes GPIO6-11 (flash), strapping pins GPIO0/2/5/12/15, and rejects output use on GPIO34-39 (input-only). The default SoftwareSerial list also avoids UART0 console pins (GPIO1/3).
- **ESP32-C3:** excludes GPIO11-17 (flash/VDD_SPI), strapping pins GPIO2/8/9, and GPIO18/19 (USB Serial/JTAG) by default.
- **ESP32-S2:** excludes GPIO19/20 (USB), GPIO26-32 (flash), strapping pins GPIO0/45, and output on GPIO46 (input-only).
- **ESP32-S3:** excludes GPIO19/20 (USB), GPIO26-37 (flash/PSRAM), strapping pins GPIO0/3/45, and output on GPIO46 (input-only).
- **ESP8266/ESP8285:** excludes GPIO6-11 (flash), strapping pins GPIO0/2/15, and GPIO16 as SoftwareSerial RX because it lacks GPIO interrupt support.
- **AVR:** SoftwareSerial RX is checked against pin-change interrupt support (`digitalPinToPCICR`).

Define `PZEM004_ALLOW_USB_GPIO` before including `PZEM004T.h` to allow GPIO18/19 on ESP32-C3 or GPIO19/20 on ESP32-S2/S3 when USB Serial/JTAG is unused. Define `PZEM004_DISABLE_PIN_FILTER` to bypass the built-in filter completely.

## Optional runtime error logging

Logging is disabled by default. Enable it by passing an output stream:

```cpp
void setup() {
  Serial.begin(115200);
  pzem.setLogOutput(&Serial);
}
```

Errors are printed with the `[PZEM][ERROR]` prefix. Use `getLastError()` and `errorToString()` to inspect status in application code.

## Multiple PZEM devices

### Multiple devices on one shared UART

Each device on a shared bus must have a unique custom address (`0x01`..`0xF7`). On ESP32, multiple `PZEM004T` objects can pass the same `HardwareSerial` instance and RX/TX pins; the library initializes the UART once on first use and prevents redundant restarts or RX buffer flushes:

```cpp
PZEM004T pzemR(Serial2, 16, 17, 0x01);
PZEM004T pzemS(Serial2, 16, 17, 0x02);
PZEM004T pzemT(Serial2, 16, 17, 0x03);
```

Poll shared-UART devices sequentially (one active transaction at a time using `isBusy()` and `isUpdateDue()`). Do not issue concurrent requests on the same physical UART.

**Hardware note:** A shared RX/TX bus must be electrically isolated or open-collector/tri-state buffered so multiple PZEM TX lines do not contend on the MCU RX pin.

### Three independent serial links

The `ThreePZEM_SeparatePorts` example uses one `HardwareSerial` port and two independent `SoftwareSerial` instances on ESP32. Because each link is physically separate, devices can be polled independently without address conflicts.

## Examples

1. `BasicHardwareSerial` - read all measurements and the custom address over HardwareSerial.
2. `BasicSoftwareSerial` - read all measurements and the custom address over SoftwareSerial.
3. `NonBlockingPolling` - demonstrate asynchronous request/response servicing and cached getters.
4. `AddressConfiguration` - read or change the address using Serial Monitor commands (`read`, `set=0x02`, `help`).
5. `ThreePZEM_SharedUART` - poll three devices sequentially on one shared UART using addresses `0x01`, `0x02`, and `0x03`.
6. `ThreePZEM_SeparatePorts` - use HardwareSerial for phase R and two SoftwareSerial ports for phases S and T.

## Upstream attribution

This project is a fork/continuation of the PZEM-004T v3.0 Arduino library by [Jakub Mandula](https://github.com/mandulaj). The upstream copyright notice and MIT license terms are retained in `LICENSE`.