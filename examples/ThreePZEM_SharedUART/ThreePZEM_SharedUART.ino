/*
 * ThreePZEM_SharedUART.ino
 * Three PZEM units share one ESP32 HardwareSerial UART pair.
 * Each PZEM must have a unique custom address: 0x01, 0x02, and 0x03.
 *
 * Wiring:
 *   ESP32 TX GPIO17 -> RX of all PZEM units
 *   ESP32 RX GPIO16 <- TX of all PZEM units
 *   Connect GND together.
 *
 * IMPORTANT:
 *   This example assumes the electrical interface and wiring allow a shared
 *   UART bus. Do not leave all PZEM units at the same default address. Configure
 *   each one individually before connecting them to the shared bus.
 *   Poll devices sequentially; do not call update() on all three in parallel.
 */

#include <PZEM004Tv30.h>

#if !defined(PZEM_BOARD_ESP32_CLASSIC)
  #error "This example is written for the ESP32 classic UART2 pin mapping."
#endif

// Initialize the shared UART only once. Other PZEM objects reuse that UART.
PZEM004Tv30 pzemR(Serial2, 16, 17, 0x01, true);
PZEM004Tv30 pzemS(Serial2, 16, 17, 0x02, false);
PZEM004Tv30 pzemT(Serial2, 16, 17, 0x03, false);

PZEM004Tv30* meters[3] = {&pzemR, &pzemS, &pzemT};
const char phaseName[3] = {'R', 'S', 'T'};
uint8_t activeMeter = 0;
uint32_t lastBusSwitch = 0;
constexpr uint32_t INTER_FRAME_GAP_MS = 20;

void selectNextMeter() {
  activeMeter = static_cast<uint8_t>((activeMeter + 1) % 3);
  lastBusSwitch = millis();
}

void printFreshMeasurement(uint8_t index) {
  const PZEM004Tv30::Measurements data = meters[index]->getData();
  Serial.print('['); Serial.print(phaseName[index]); Serial.print(F("] "));
  Serial.print(F("V=")); Serial.print(data.voltage, 1); Serial.print(F(" V, "));
  Serial.print(F("I=")); Serial.print(data.current, 3); Serial.print(F(" A, "));
  Serial.print(F("P=")); Serial.print(data.power, 1); Serial.print(F(" W, "));
  Serial.print(F("E=")); Serial.print(data.energy, 3); Serial.print(F(" kWh, "));
  Serial.print(F("PF=")); Serial.print(data.pf, 2);
  Serial.print(F(", address=0x")); Serial.println(meters[index]->getAddress(), HEX);
}

void setup() {
  Serial.begin(115200);
  pzemR.setLogOutput(&Serial);
  pzemS.setLogOutput(&Serial);
  pzemT.setLogOutput(&Serial);
  Serial.println(F("Three PZEM units on one shared HardwareSerial bus"));
}

void loop() {
  if (static_cast<uint32_t>(millis() - lastBusSwitch) < INTER_FRAME_GAP_MS) {
    return;
  }

  PZEM004Tv30& selected = *meters[activeMeter];
  const bool freshFrame = selected.update();

  if (freshFrame) {
    printFreshMeasurement(activeMeter);
    selectNextMeter();
    return;
  }

  // Move on after a timeout/TX error, or while this device is not due yet.
  // While a transaction is pending, keep servicing only the selected object.
  if (!selected.isBusy() &&
      (selected.getLastError() != PZEM004Tv30::Error::NONE ||
       !selected.isUpdateDue())) {
    selectNextMeter();
  }
}
