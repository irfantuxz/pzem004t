/*
 * NonBlockingPolling.ino
 * Demonstrate non-blocking polling with getter fallback enabled.
 * The loop remains available for other application tasks.
 */

#include <PZEM004Tv30.h>

PZEM004Tv30 pzem(Serial2, 16, 17);
uint32_t lastApplicationTask = 0;

void setup() {
  Serial.begin(115200);
  pzem.setLogOutput(&Serial);
  pzem.setUpdateInterval(1000);
  pzem.setResponseTimeout(150);
  Serial.println(F("Non-blocking polling example"));
}

void loop() {
  // The update() call only sends a request or consumes currently buffered bytes.
  if (pzem.update()) {
    const PZEM004Tv30::Measurements data = pzem.getData();
    Serial.print(F("Fresh frame: "));
    Serial.print(data.voltage, 1); Serial.print(F(" V, "));
    Serial.print(data.current, 3); Serial.print(F(" A, "));
    Serial.print(data.power, 1); Serial.print(F(" W, "));
    Serial.print(data.energy, 3); Serial.print(F(" kWh, "));
    Serial.print(data.frequency, 1); Serial.print(F(" Hz, PF="));
    Serial.println(data.pf, 2);
  }

  // Example of another application task running without waiting for PZEM data.
  if (millis() - lastApplicationTask >= 100) {
    lastApplicationTask = millis();
    // Place sensor, display, control, or communication tasks here.
  }

  // Backward compatibility: these getters also call update() as a fallback.
  // They return the most recently cached data and never wait for a new frame.
  // float cachedVoltage = pzem.voltage();
}
