/*
 * BasicHardwareSerial.ino
 *
 * Contoh dasar membaca PZEM-004T menggunakan HardwareSerial.
 * Mendukung Arduino Uno, Nano, Mega 2560, ESP8266, dan ESP32 series.
 *
 * Mapping UART:
 *
 * Arduino Uno / Nano:
 *   Serial  : RX=0, TX=1
 *
 * Arduino Mega 2560:
 *   Serial1 : RX1=19, TX1=18
 *
 * ESP8266 / NodeMCU:
 *   Serial.swap() : RX=GPIO13 (D7), TX=GPIO15 (D8)
 *
 * ESP32 Classic / ESP32 Dev Module:
 *   Serial2 : RX=GPIO16, TX=GPIO17
 *
 * ESP32-C3 / ESP32-S2 / ESP32-S3:
 *   Serial1 : RX=GPIO4, TX=GPIO5
 *
 * Catatan:
 * - Uno/Nano menggunakan UART yang sama dengan USB programming. Lepaskan
 *   koneksi PZEM dari RX0/TX0 saat upload sketch.
 * - Pada ESP8266, GPIO15 (D8) adalah boot-strapping pin dan harus tetap LOW
 *   selama proses boot. Pastikan input RX PZEM tidak memberikan pull-up ke pin ini.
 * - ESP32-WROVER dapat menggunakan GPIO16/17 untuk PSRAM. Mapping ESP32 Classic
 *   di bawah ditujukan untuk ESP32 Dev Module / ESP32-WROOM-32 yang menyediakan
 *   GPIO16/17 untuk pemakaian umum.
 */

#include <PZEM004T.h>

// -----------------------------------------------------------------------------
// Pilih HardwareSerial dan pin berdasarkan board.
// -----------------------------------------------------------------------------

#if defined(PZEM_BOARD_AVR_UNO) || defined(PZEM_BOARD_AVR_NANO)

  #define PZEM_HW_SERIAL Serial
  #define PZEM_RX_PIN    0   // RX0
  #define PZEM_TX_PIN    1   // TX0

#elif defined(PZEM_BOARD_AVR_MEGA)

  #define PZEM_HW_SERIAL Serial1
  #define PZEM_RX_PIN    19  // RX1
  #define PZEM_TX_PIN    18  // TX1

#elif defined(PZEM_BOARD_ESP8266)

  #define PZEM_HW_SERIAL Serial
  #define PZEM_RX_PIN    D7  // GPIO13 / D7 / UART0 RX setelah swap
  #define PZEM_TX_PIN    D8  // GPIO15 / D8 / UART0 TX setelah swap
  #define PZEM_USE_UART_SWAP

#elif defined(PZEM_BOARD_ESP32_CLASSIC)

  #define PZEM_HW_SERIAL Serial2
  #define PZEM_RX_PIN    16  // GPIO16 / RX2
  #define PZEM_TX_PIN    17  // GPIO17 / TX2

#elif defined(PZEM_BOARD_ESP32_C3) || \
      defined(PZEM_BOARD_ESP32_S2)  || \
      defined(PZEM_BOARD_ESP32_S3)

  #define PZEM_HW_SERIAL Serial1
  #define PZEM_RX_PIN    4   // GPIO4 / RX1
  #define PZEM_TX_PIN    5   // GPIO5 / TX1

#else
  #error "BasicHardwareSerial: board belum didukung. Tambahkan mapping HardwareSerial untuk board ini."
#endif

// ESP32 mendukung pemetaan pin UART melalui constructor library.
// AVR dan ESP8266 menggunakan pin UART tetap dari core masing-masing.
#if defined(ESP32)
  PZEM004T pzem(PZEM_HW_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN);
#else
  PZEM004T pzem(PZEM_HW_SERIAL);
#endif

void setup() {
  Serial.begin(115200);
#if defined(PZEM_USE_UART_SWAP)
  // Pindahkan UART0 dari GPIO1/GPIO3 ke GPIO15/GPIO13.
  // Constructor PZEM telah menginisialisasi Serial pada 9600 baud.
  Serial.swap();
#endif
}

void loop() {
  // update() dipanggil sesering mungkin; proses komunikasi bersifat non-blocking.
  // Nilai baru disalin hanya ketika satu frame respons valid telah diterima.
  if (pzem.update()) {
    const PZEM004T::Measurements data = pzem.getData();

    Serial.println(F("\n--- PZEM measurements ---"));
    Serial.print(F("Voltage     : ")); Serial.print(data.voltage, 1); Serial.println(F(" V"));
    Serial.print(F("Current     : ")); Serial.print(data.current, 3); Serial.println(F(" A"));
    Serial.print(F("Power       : ")); Serial.print(data.power, 1); Serial.println(F(" W"));
    Serial.print(F("Energy      : ")); Serial.print(data.energy, 3); Serial.println(F(" kWh"));
    Serial.print(F("Frequency   : ")); Serial.print(data.frequency, 1); Serial.println(F(" Hz"));
    Serial.print(F("Power factor: ")); Serial.println(data.pf, 2);
    Serial.print(F("Alarm       : 0x")); Serial.println(static_cast<unsigned int>(data.alarms), HEX);
  }
}
