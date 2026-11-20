/*
 * ThreePZEM_SharedUART.ino
 *
 * Three PZEM meters sharing one ESP32 HardwareSerial port.
 * Target: ESP32 classic / ESP32 Dev Module.
 *
 * Shared UART2: RX=GPIO16, TX=GPIO17
 * Meter addresses: 0x01, 0x02, and 0x03 (must be unique).
 *
 * IMPORTANT:
 * 1. Configure each PZEM address individually before connecting all meters.
 * 2. The electrical interface must support a shared bus / multiple slaves.
 * 3. Only one meter is queried at a time. Do not poll all three simultaneously.
 * 4. PZEM TX lines must not be connected in parallel as push-pull TTL outputs
 *    unless the interface hardware explicitly supports this arrangement.
 */

#include <PZEM004T.h>

#if !defined(PZEM_BOARD_ESP32_CLASSIC)
  #error "This example is intended for ESP32 classic / ESP32 Dev Module."
#endif

// All objects use the same physical UART and pin mapping, but distinct slave
// addresses. The library's ESP32 HardwareSerial registry prevents each object
// from reinitializing the shared UART.
PZEM004T pzemR(Serial2, 16, 17, 0x01);
PZEM004T pzemS(Serial2, 16, 17, 0x02);
PZEM004T pzemT(Serial2, 16, 17, 0x03);

PZEM004T* const meters[3] = {&pzemR, &pzemS, &pzemT};
const char phaseName[3] = {'R', 'S', 'T'};

uint8_t activeMeter = 0;
uint32_t lastBusSwitch = 0;
constexpr uint32_t INTER_FRAME_GAP_MS = 20;

void selectNextMeter() {
  activeMeter = static_cast<uint8_t>((activeMeter + 1) % 3);
  lastBusSwitch = millis();
}

void printMeasurement(uint8_t index) {
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
  Serial.println(F("PZEM004T - Three meters on one shared UART2"));
  Serial.println(F("UART2 RX=GPIO16, TX=GPIO17"));
  Serial.println(F("Addresses: R=0x01, S=0x02, T=0x03"));

  // Each object attaches to the same UART configuration. begin() initializes
  // the physical UART once and registers the other objects against that port.
  const bool readyR = pzemR.begin();
  const bool readyS = pzemS.begin();
  const bool readyT = pzemT.begin();

  Serial.print(F("R: ")); Serial.println(readyR ? F("ready") : F("FAILED"));
  Serial.print(F("S: ")); Serial.println(readyS ? F("ready") : F("FAILED"));
  Serial.print(F("T: ")); Serial.println(readyT ? F("ready") : F("FAILED"));

  lastBusSwitch = millis();
}

void loop() {
  // Keep a quiet gap between transactions on the shared bus.
  if (static_cast<uint32_t>(millis() - lastBusSwitch) < INTER_FRAME_GAP_MS) {
    return;
  }

  PZEM004T& selected = *meters[activeMeter];
  const bool freshFrame = selected.update();

  if (freshFrame) {
    printMeasurement(activeMeter);
    selectNextMeter();
    return;
  }

  // Keep servicing a request while its response is pending. Switch to the
  // next meter after completion/timeout, or when the selected meter is not due.
  if (!selected.isBusy() &&
      (selected.getLastError() != PZEM004T::Error::NONE ||
       !selected.isUpdateDue())) {
    selectNextMeter();
  }
}
