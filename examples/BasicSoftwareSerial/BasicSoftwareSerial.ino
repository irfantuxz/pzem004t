/*
 * BasicSoftwareSerial.ino
 * Read measurements and the PZEM custom address using SoftwareSerial.
 * Install EspSoftwareSerial on ESP32 boards before compiling this example.
 *
 * ESP32 dev / classic        ->  RX=18 ; TX=19
 * ESP32 c3 s2 s3             ->  RX= 4 ; TX= 5
 * ESP8266 / nodemcu          ->  RX=D5 ; TX=D6
 * AVR arduino uno nano mega  ->  RX=10 ; TX=11
 */

#include <PZEM004T.h>

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

// SoftSerial is a short alias for PZEM004T::SoftwareSerial.
PZEM004T pzem(SoftSerial, PZEM_RX_PIN, PZEM_TX_PIN);

void setup() {
  Serial.begin(115200);
  pzem.setLogOutput(&Serial);

  Serial.println(F("PZEM004T v2.0.0 - SoftwareSerial example"));
  Serial.print(F("Board: ")); Serial.println(PZEM004T::boardName());
  Serial.print(F("Software RX/TX: "));
  Serial.print(PZEM_RX_PIN); Serial.print('/'); Serial.println(PZEM_TX_PIN);

  if (!PZEM004T::isSoftwareSerialPinValid(PZEM_RX_PIN, PZEM_TX_PIN)) {
    Serial.println(F("ERROR: The selected SoftwareSerial pin pair is not supported."));
  }

  uint8_t address = 0;
  if (pzem.readAddress(address)) {
    Serial.print(F("Detected custom address: 0x"));
    Serial.println(address, HEX);
  } else {
    Serial.print(F("Address read failed: "));
    Serial.println(PZEM004T::errorToString(pzem.getLastError()));
  }
}

void loop() {
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