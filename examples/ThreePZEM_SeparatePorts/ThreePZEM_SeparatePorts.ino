/*
 * ThreePZEM_SeparatePorts.ino
 *
 * Three PZEM meters using three independent serial ports.
 * Target: ESP32 classic / ESP32 Dev Module (ESP32-WROOM-32).
 *
 * Phase R: SoftwareSerial  RX=GPIO18, TX=GPIO19
 * Phase S: HardwareSerial2 RX=GPIO16, TX=GPIO17
 * Phase T: HardwareSerial1 RX=GPIO27, TX=GPIO26
 *
 * Install EspSoftwareSerial by plerup. The library includes SoftwareSerial.h.
 * Wiring convention for each meter: PZEM TX -> MCU RX, PZEM RX -> MCU TX.
 */

#include <PZEM004T.h>

#if !defined(PZEM_BOARD_ESP32_CLASSIC)
  #error "This example is intended for ESP32 classic / ESP32 Dev Module."
#endif

// The requested pin and UART assignment:
PZEM004T pzemR(SoftSerial, 18, 19);
PZEM004T pzemS(Serial2, 16, 17);
PZEM004T pzemT(Serial1, 27, 26);


PZEM004T* const meters[3] = {&pzemR, &pzemS, &pzemT};
const char phaseName[3] = {'R', 'S', 'T'};


void printPhase(uint8_t index) {
  const PZEM004T::Measurements data = meters[index]->getData();

  Serial.print('[');
  Serial.print(phaseName[index]);
  Serial.print(F("] V="));
  Serial.print(data.voltage, 1);
  Serial.print(F(" V, I="));
  Serial.print(data.current, 3);
  Serial.print(F(" A, P="));
  Serial.print(data.power, 1);
  Serial.print(F(" W, E="));
  Serial.print(data.energy, 3);
  Serial.print(F(" kWh, f="));
  Serial.print(data.frequency, 1);
  Serial.print(F(" Hz, PF="));
  Serial.print(data.pf, 2);
  Serial.print(F(", Alarm=0x"));
  Serial.println(data.alarms, HEX);
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("PZEM004T - Three independent serial ports"));
  Serial.println(F("R=SoftwareSerial 18/19"));
  Serial.println(F("S=Serial2 16/17"));
  Serial.println(F("T=Serial1 27/26"));

  // Explicit initialization is performed after the Arduino runtime starts.
  const bool readyR = pzemR.begin();
  const bool readyS = pzemS.begin();
  const bool readyT = pzemT.begin();

  Serial.print(F("R: ")); Serial.println(readyR ? F("ready") : F("FAILED"));
  Serial.print(F("S: ")); Serial.println(readyS ? F("ready") : F("FAILED"));
  Serial.print(F("T: ")); Serial.println(readyT ? F("ready") : F("FAILED"));
}

void loop() {
  // These are independent serial ports, so each meter can be serviced on
  // every loop pass. No delay() is required.
  for (uint8_t i = 0; i < 3; ++i) {
    if (meters[i]->update()) {
      printPhase(i);
    }
  }
}
