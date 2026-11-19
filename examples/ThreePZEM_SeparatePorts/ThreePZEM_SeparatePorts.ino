/*
 * ThreePZEM_SeparatePorts.ino
 * Three independent serial links for three PZEM units.
 * Target: ESP32 classic Dev Module.
 *
 * Phase R: HardwareSerial UART2, RX=GPIO16, TX=GPIO17.
 * Phase S: SoftwareSerial, RX=GPIO18, TX=GPIO19.
 * Phase T: SoftwareSerial, RX=GPIO25, TX=GPIO26.
 * Each PZEM has an independent serial link, so each may use its own address.
 * Install EspSoftwareSerial by plerup before compiling.
 */

#include <PZEM004Tv30.h>

#if !defined(PZEM_BOARD_ESP32_CLASSIC)
  #error "This example uses ESP32 classic pin assignments. Adjust pins for other ESP32 variants."
#endif

PZEM004Tv30 pzemR(Serial2, 16, 17);
PZEM004Tv30 pzemS(SoftSerial, 18, 19);
PZEM004Tv30 pzemT(SoftSerial, 25, 26);

PZEM004Tv30* meters[3] = {&pzemR, &pzemS, &pzemT};
const char phaseName[3] = {'R', 'S', 'T'};

void printPhase(uint8_t index) {
  const PZEM004Tv30::Measurements data = meters[index]->getData();
  Serial.print('['); Serial.print(phaseName[index]); Serial.print(F("] "));
  Serial.print(F("V=")); Serial.print(data.voltage, 1); Serial.print(F(" V, "));
  Serial.print(F("I=")); Serial.print(data.current, 3); Serial.print(F(" A, "));
  Serial.print(F("P=")); Serial.print(data.power, 1); Serial.print(F(" W, "));
  Serial.print(F("E=")); Serial.print(data.energy, 3); Serial.print(F(" kWh, "));
  Serial.print(F("F=")); Serial.print(data.frequency, 1); Serial.print(F(" Hz, "));
  Serial.print(F("PF=")); Serial.println(data.pf, 2);
}

void setup() {
  Serial.begin(115200);
  pzemR.setLogOutput(&Serial);
  pzemS.setLogOutput(&Serial);
  pzemT.setLogOutput(&Serial);

  // Start both SoftwareSerial receivers before any device is polled.
  const bool softwareSReady = pzemS.begin();
  const bool softwareTReady = pzemT.begin();

  Serial.println(F("Three PZEM units on three independent serial ports"));
  Serial.println(F("R=HardwareSerial, S/T=EspSoftwareSerial"));
  Serial.print(F("SoftwareSerial S: "));
  Serial.println(softwareSReady ? F("ready") : F("ERROR"));
  Serial.print(F("SoftwareSerial T: "));
  Serial.println(softwareTReady ? F("ready") : F("ERROR"));
}

void loop() {
  if (pzemR.update()) printPhase(0);
  if (pzemS.update()) printPhase(1);
  if (pzemT.update()) printPhase(2);
}
