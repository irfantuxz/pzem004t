/*
 * BasicHardwareSerial.ino
 * Read every measurement and the configured slave address using HardwareSerial.
 * Target: ESP32 classic Dev Module.
 */

#include <PZEM004Tv30.h>

// ESP32 classic UART2 pins: RX=GPIO16, TX=GPIO17.
PZEM004Tv30 pzem(Serial2, 16, 17);

void printMeasurements() {
  const PZEM004Tv30::Measurements data = pzem.getData();

  Serial.println(F("\n--- PZEM measurements ---"));
  Serial.print(F("Voltage     : ")); Serial.print(data.voltage, 1); Serial.println(F(" V"));
  Serial.print(F("Current     : ")); Serial.print(data.current, 3); Serial.println(F(" A"));
  Serial.print(F("Power       : ")); Serial.print(data.power, 1); Serial.println(F(" W"));
  Serial.print(F("Energy      : ")); Serial.print(data.energy, 3); Serial.println(F(" kWh"));
  Serial.print(F("Frequency   : ")); Serial.print(data.frequency, 1); Serial.println(F(" Hz"));
  Serial.print(F("Power factor: ")); Serial.println(data.pf, 2);
  Serial.print(F("Alarm       : 0x")); Serial.println(static_cast<unsigned int>(data.alarms), HEX);
  Serial.print(F("Address     : 0x")); Serial.println(pzem.getAddress(), HEX);
}

void setup() {
  Serial.begin(115200);
  pzem.setLogOutput(&Serial);

  Serial.println(F("PZEM004Tv30 v2.0.0 - HardwareSerial example"));
  Serial.print(F("Board: ")); Serial.println(PZEM004Tv30::boardName());

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
    printMeasurements();
  }
}
