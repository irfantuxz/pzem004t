/*
 * BasicSoftwareSerial.ino
 * Read every measurement and the configured slave address using SoftwareSerial.
 * Install EspSoftwareSerial on ESP32 boards before compiling this example.
 */

#include <PZEM004Tv30.h>

#if defined(PZEM_BOARD_ESP32_CLASSIC)
  constexpr int PZEM_RX_PIN = 18;
  constexpr int PZEM_TX_PIN = 19;
#elif defined(PZEM_BOARD_ESP32_C3) || defined(PZEM_BOARD_ESP32_S3) || defined(PZEM_BOARD_ESP32_S2)
  constexpr int PZEM_RX_PIN = 4;
  constexpr int PZEM_TX_PIN = 5;
#elif defined(PZEM_BOARD_ESP8266)
  constexpr int PZEM_RX_PIN = 14;  // GPIO14 / D5
  constexpr int PZEM_TX_PIN = 12;  // GPIO12 / D6
#elif defined(PZEM_BOARD_AVR_UNO) || defined(PZEM_BOARD_AVR_NANO) || defined(PZEM_BOARD_AVR_MEGA)
  constexpr int PZEM_RX_PIN = 10;
  constexpr int PZEM_TX_PIN = 11;
#else
  #error "Select RX/TX pins for this board before using the SoftwareSerial example."
#endif

// SoftSerial is a short alias for PZEM004Tv30::SoftwareSerial.
PZEM004Tv30 pzem(SoftSerial, PZEM_RX_PIN, PZEM_TX_PIN);

void setup() {
  Serial.begin(115200);
  pzem.setLogOutput(&Serial);

  Serial.println(F("PZEM004Tv30 v2.0.0 - SoftwareSerial example"));
  Serial.print(F("Board: ")); Serial.println(PZEM004Tv30::boardName());
  Serial.print(F("Software RX/TX: "));
  Serial.print(PZEM_RX_PIN); Serial.print('/'); Serial.println(PZEM_TX_PIN);

  if (!PZEM004Tv30::isSoftwareSerialPinValid(PZEM_RX_PIN, PZEM_TX_PIN)) {
    Serial.println(F("ERROR: The selected SoftwareSerial pin pair is not supported."));
  }

  uint8_t address = 0;
  if (pzem.readAddress(address, true)) {
    Serial.print(F("Detected custom address: 0x"));
    Serial.println(address, HEX);
  } else {
    Serial.print(F("Address read failed: "));
    Serial.println(PZEM004Tv30::errorToString(pzem.getLastError()));
  }
}

void loop() {
  if (pzem.update()) {
    const PZEM004Tv30::Measurements data = pzem.getData();

    Serial.print(F("V=")); Serial.print(data.voltage, 1); Serial.print(F(" V, "));
    Serial.print(F("I=")); Serial.print(data.current, 3); Serial.print(F(" A, "));
    Serial.print(F("PF=")); Serial.print(data.pf, 2); Serial.print(F(", "));
    Serial.print(F("F=")); Serial.print(data.frequency, 1); Serial.print(F(" Hz, "));
    Serial.print(F("P=")); Serial.print(data.power, 1); Serial.print(F(" W, "));
    Serial.print(F("E=")); Serial.print(data.energy, 3); Serial.println(F(" kWh"));
  }
}
