# PZEM004Tv30 - Version 2.0.0

An extended fork of the PZEM-004T v3.0 Arduino library by Jakub Mandula. This release continues from upstream version **1.2.1** and retains the original MIT license notice.

## Fork changes in 2.0.0

1. Fixed address handling in `setAddress()` and `readAddress()`, including explicit use of the PZEM general address `0xF8`.
2. Added SoftwareSerial support for ESP32 families. ESP32 users must install **EspSoftwareSerial by plerup**.
3. Added support and examples for multiple PZEM devices connected to a shared UART pair, using unique device addresses.
4. Added non-blocking measurement polling and cached measurement getters.
5. Added board-family macros and conservative GPIO validation for SoftwareSerial and ESP32 HardwareSerial pins.
6. Added optional runtime error logs and readable error labels.
7. Added `SoftSerial` as a compact constructor tag.
8. Added six examples for common wiring and application patterns.

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

The term ESP8266 family includes common ESP8266EX and ESP8285-based boards. There is no separate Espressif MCU family named ESP8288; check the module marking if the board vendor uses that text.

## Dependencies

- Arduino framework for the selected board.
- **ESP32 SoftwareSerial:** install [EspSoftwareSerial by plerup](https://github.com/plerup/espsoftwareserial). If it is absent, the library emits a compile-time warning and SoftwareSerial constructors are not enabled; HardwareSerial remains available.
- **ESP8266:** SoftwareSerial implementation supplied by the ESP8266 Arduino core.
- **AVR Uno/Nano/Mega:** Arduino SoftwareSerial supplied by the AVR core.

To disable SoftwareSerial support explicitly, define `PZEM004_NO_SWSERIAL` before including `PZEM004Tv30.h`.

## Installation

1. Download this repository as a ZIP archive.
2. In Arduino IDE, choose **Sketch > Include Library > Add .ZIP Library...**.
3. Select the downloaded ZIP.
4. Install EspSoftwareSerial separately when compiling for ESP32.
5. Open one of the examples under **File > Examples > PZEM004Tv30**.

## Serial defaults

- Baud rate: `9600`
- Data format: `8N1`
- Default device address: `0xF8` (PZEM general/default address)
- Default measurement interval: `1000 ms`
- Default measurement response timeout: `150 ms`

## Constructor examples

### ESP32 HardwareSerial

```cpp
#include <PZEM004Tv30.h>

PZEM004Tv30 pzem(Serial2, 16, 17);
```

### SoftwareSerial on ESP32

```cpp
#include <PZEM004Tv30.h>

PZEM004Tv30 pzem(SoftSerial, 18, 19);
```

`SoftSerial` is a macro alias for `PZEM004Tv30::SoftwareSerial`.

### SoftwareSerial on AVR Uno/Nano

```cpp
#include <PZEM004Tv30.h>

PZEM004Tv30 pzem(SoftSerial, 10, 11);
```

### Legacy SoftwareSerial object

```cpp
#include <PZEM004Tv30.h>
#include <SoftwareSerial.h>

SoftwareSerial pzemPort;
PZEM004Tv30 pzem(pzemPort);

void setup() {
  Serial.begin(115200);
  pzemPort.begin(9600, SWSERIAL_8N1, 18, 19);
}
```

The library initializes an externally supplied SoftwareSerial object lazily on the first service call. On ESP32/ESP8266, it preserves RX/TX pins already configured in the object.

## Non-blocking measurement polling

Call `update()` frequently from `loop()`:

```cpp
void loop() {
  if (pzem.update()) {
    const PZEM004Tv30::Measurements data = pzem.getData();
    Serial.println(data.voltage);
  }
}
```

`update()` sends a request when the configured interval expires and consumes only bytes currently buffered. It does not wait for the response. It returns `true` only when a new valid measurement frame has been received.

The measurement getters (`voltage()`, `current()`, `power()`, `energy()`, `frequency()`, and `pf()`) read the cache and service `update()` at most once per millisecond. This is a compatibility fallback for older sketches that do not explicitly call `update()`. Getter calls do not wait for a fresh response. Before the first valid frame, getters return `NAN`.

## Address handling

The PZEM general address `0xF8` is not the same as Modbus broadcast address `0x00`.

- `0x01` to `0xF7`: valid custom slave addresses.
- `0xF8`: PZEM general/default address used for single-device commissioning and generic access.
- `0x00`: Modbus broadcast address; a broadcast request is not expected to receive a normal response.

Read an address using the unambiguous API:

```cpp
uint8_t address = 0;
if (pzem.readAddress(address, true)) {
  Serial.print("Address: 0x");
  Serial.println(address, HEX);
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
PZEM004Tv30::isSoftwareSerialPinValid(rxPin, txPin);
PZEM004Tv30::isHardwareSerialPinValid(rxPin, txPin);
PZEM004Tv30::boardName();
```

The filter is conservative and checks common GPIO restrictions:

- ESP32 classic: excludes GPIO6-11 (flash), common strapping pins GPIO0/2/5/12/15, and rejects output use on GPIO34-39 (input-only). The default SoftwareSerial list also avoids UART0 console pins.
- ESP32-C3: excludes GPIO11-17 (flash/VDD_SPI), strapping pins GPIO2/8/9, and GPIO18/19 (USB Serial/JTAG) by default.
- ESP32-S2: excludes GPIO19/20 (USB), GPIO26-32 (flash), and strapping pins GPIO0/45; GPIO46 is input-only. The software-UART filter follows the more restrictive GPIO range supported by EspSoftwareSerial.
- ESP32-S3: excludes GPIO19/20 (USB), GPIO26-37 (flash/PSRAM reservations), and strapping pins GPIO0/3/45; GPIO46 is input-only. The software-UART filter also excludes the strapping pins.
- ESP8266/ESP8285: excludes GPIO6-11 (flash), strapping pins GPIO0/2/15, and GPIO16 as SoftwareSerial RX because it does not provide the required GPIO interrupt capability.
- AVR: SoftwareSerial RX is checked using the core's pin-change interrupt mapping when available. On Mega, not every digital pin supports SoftwareSerial reception.

GPIO usage is module-dependent. ESP32 modules can reserve additional pins for flash, PSRAM, USB, JTAG, boot strapping, or onboard peripherals. Check the datasheet and board schematic. Define `PZEM004_ALLOW_USB_GPIO` to opt into GPIO18/19 on C3 or GPIO19/20 on S2/S3 when USB Serial/JTAG is not using them. Define `PZEM004_DISABLE_PIN_FILTER` before including the library to bypass the generic filter when you have verified the board-specific pin assignment yourself.

## Optional runtime error logging

Logging is disabled by default. Enable it by passing an output stream:

```cpp
void setup() {
  Serial.begin(115200);
  pzem.setLogOutput(&Serial);
}
```

Errors are printed with the `[PZEM][ERROR]` prefix. Use `getLastError()` and `errorToString()` to read the most recent status in application code.

## Multiple PZEM devices

### Multiple devices on one shared UART

Each device must have a unique custom address. The three-device shared-UART example uses addresses `0x01`, `0x02`, and `0x03` and explicitly services one device at a time. Do not call `update()` for all devices concurrently when they share one physical UART; each object would otherwise compete to consume bytes from the same receive buffer.

On ESP32, the HardwareSerial constructor provides a `beginPort` parameter. Initialize a shared UART once and pass `false` for the remaining objects:

```cpp
PZEM004Tv30 pzemR(Serial2, 16, 17, 0x01, true);
PZEM004Tv30 pzemS(Serial2, 16, 17, 0x02, false);
PZEM004Tv30 pzemT(Serial2, 16, 17, 0x03, false);
```

A single shared RX/TX pair must be electrically suitable for the PZEM UART interface. Do not directly tie push-pull outputs together unless the interface design explicitly supports it; use the appropriate bus/isolation circuitry when required.

### Three independent serial links

The `ThreePZEM_SeparatePorts` example uses one HardwareSerial port and two independent EspSoftwareSerial instances on ESP32 classic. Since these are physically separate links, each device can be polled independently and does not require a unique address from the other links.

## Examples

1. `BasicHardwareSerial` - read all measurements and the custom address over HardwareSerial.
2. `BasicSoftwareSerial` - read all measurements and the custom address over SoftwareSerial.
3. `NonBlockingPolling` - demonstrate asynchronous request/response servicing and cached getters.
4. `AddressConfiguration` - read or change the address using Serial Monitor commands (`read`, `set=0x02`, `help`).
5. `ThreePZEM_SharedUART` - poll three devices sequentially on one shared UART using addresses `0x01`, `0x02`, and `0x03`.
6. `ThreePZEM_SeparatePorts` - use HardwareSerial for phase R and two SoftwareSerial ports for phases S and T.

## Upstream attribution

This project is a fork/continuation of the PZEM-004T v3.0 Arduino library by [Jakub Mandula](https://github.com/mandulaj). The upstream copyright notice and MIT license terms are retained in `LICENSE`.
