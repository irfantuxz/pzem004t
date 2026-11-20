/*
 * AdvancedFeatures.ino
 *
 * A collection of less-frequently-used features:
 * - CSV-style measurement logging over Serial
 * - Energy counter reset
 * - Power alarm threshold configuration and alarm-state reading
 * - Reading and changing the PZEM custom address
 * - Connection, timeout, and SoftwareSerial overflow diagnostics
 *
 * The serial transport and pin mapping follow BasicRead, so this example
 * supports AVR, ESP8266, and ESP32 board families covered there.
 *
 * Serial Monitor: 115200 baud, send commands ending with newline or CR.
 */

#include <PZEM004T.h>

// Select the same example pins as BasicRead.
#if defined(PZEM_BOARD_ESP32_CLASSIC)
  #define PZEM_RX_PIN 16
  #define PZEM_TX_PIN 17
  PZEM004T pzem(Serial2, PZEM_RX_PIN, PZEM_TX_PIN);
#elif defined(PZEM_BOARD_ESP32_C3) || \
      defined(PZEM_BOARD_ESP32_S2) || \
      defined(PZEM_BOARD_ESP32_S3)
  #define PZEM_RX_PIN 4
  #define PZEM_TX_PIN 5
  PZEM004T pzem(SoftSerial, PZEM_RX_PIN, PZEM_TX_PIN);
#elif defined(PZEM_BOARD_ESP8266)
  #define PZEM_RX_PIN 14
  #define PZEM_TX_PIN 12
  PZEM004T pzem(SoftSerial, PZEM_RX_PIN, PZEM_TX_PIN);
#elif defined(PZEM_BOARD_AVR_UNO) || \
      defined(PZEM_BOARD_AVR_NANO) || \
      defined(PZEM_BOARD_AVR_MEGA) || \
      defined(PZEM_BOARD_AVR)
  #define PZEM_RX_PIN 10
  #define PZEM_TX_PIN 11
  PZEM004T pzem(SoftSerial, PZEM_RX_PIN, PZEM_TX_PIN);
#else
  #error "AdvancedFeatures: unsupported board. Select RX/TX and serial interface."
#endif

// -----------------------------------------------------------------------------
// User configuration
// -----------------------------------------------------------------------------

constexpr uint32_t POLL_INTERVAL_MS = 200;   // 200 ms = 5 measurement requests/s
constexpr uint16_t POWER_ALARM_WATTS = 2000;
constexpr uint8_t NEW_PZEM_ADDRESS = 0x02;    // Valid custom address: 0x01..0xF7

bool csvLoggingEnabled = true;
PZEM004T::Error previousError = PZEM004T::Error::NONE;

void printHelp() {
  Serial.println();
  Serial.println(F("Commands:"));
  Serial.println(F("  L - toggle CSV measurement logging"));
  Serial.println(F("  R - reset the accumulated energy counter"));
  Serial.println(F("  A - set power alarm threshold"));
  Serial.println(F("  G - show the latest alarm state"));
  Serial.println(F("  D - read the installed custom address"));
  Serial.println(F("  S - set the custom address to NEW_PZEM_ADDRESS"));
  Serial.println(F("  C - show connection/receive diagnostics"));
  Serial.println(F("  H - show this help"));
  Serial.println();
  Serial.println(F("Note: D and S use the PZEM general address 0xF8."));
  Serial.println(F("Use D/S only with one meter connected to this serial link."));
}

void printCsvHeader() {
  Serial.println(F("time_ms,voltage_V,current_A,power_W,energy_kWh,frequency_Hz,pf,alarm_hex"));
}

void printCsvRecord(const PZEM004T::Measurements& data) {
  Serial.print(data.timestamp);
  Serial.print(','); Serial.print(data.voltage, 1);
  Serial.print(','); Serial.print(data.current, 3);
  Serial.print(','); Serial.print(data.power, 1);
  Serial.print(','); Serial.print(data.energy, 3);
  Serial.print(','); Serial.print(data.frequency, 1);
  Serial.print(','); Serial.print(data.pf, 2);
  Serial.print(F(",0x")); Serial.println(data.alarms, HEX);
}

void resetEnergyCounter() {
  Serial.println(F("Sending energy-counter reset command..."));

  if (pzem.resetEnergy()) {
    // The reset command normally has no response frame. true indicates that
    // the command frame was transmitted, not that the meter acknowledged it.
    Serial.println(F("Reset command transmitted. Check the next energy readings."));
  } else {
    Serial.print(F("Reset command failed: "));
    Serial.println(PZEM004T::errorToString(pzem.getLastError()));
  }
}

void configurePowerAlarm() {
  Serial.print(F("Setting power alarm to "));
  Serial.print(POWER_ALARM_WATTS);
  Serial.println(F(" W..."));

  if (pzem.setPowerAlarm(POWER_ALARM_WATTS)) {
    Serial.println(F("Power alarm threshold write acknowledged."));
  } else {
    Serial.print(F("Power alarm write failed: "));
    Serial.println(PZEM004T::errorToString(pzem.getLastError()));
  }
}

void showAlarmState() {
  if (!pzem.hasData()) {
    Serial.println(F("No measurement received yet; alarm state is not available."));
    return;
  }

  const PZEM004T::Measurements data = pzem.getData();
  Serial.print(F("Alarm register: 0x"));
  Serial.println(data.alarms, HEX);
  Serial.println(data.alarms == 0 ? F("No power alarm") : F("Power alarm active"));
}

void readInstalledAddress() {
  uint8_t address = 0;
  if (pzem.readAddress(address)) {
    Serial.print(F("Installed custom address: 0x"));
    if (address < 0x10) Serial.print('0');
    Serial.println(address, HEX);
  } else {
    Serial.print(F("Address read failed: "));
    Serial.println(PZEM004T::errorToString(pzem.getLastError()));
  }
}

void changeAddress() {
  Serial.print(F("Changing custom address to 0x"));
  if (NEW_PZEM_ADDRESS < 0x10) Serial.print('0');
  Serial.println(NEW_PZEM_ADDRESS, HEX);

  // setAddress(address) writes through the PZEM general address 0xF8.
  // This example assumes only one meter is connected during configuration.
  if (pzem.setAddress(NEW_PZEM_ADDRESS)) {
    Serial.println(F("Address write acknowledged."));
    readInstalledAddress();
  } else {
    Serial.print(F("Address write failed: "));
    Serial.println(PZEM004T::errorToString(pzem.getLastError()));
  }
}

void showDiagnostics() {
  Serial.print(F("Connected: "));
  Serial.println(pzem.isConnected() ? F("yes") : F("no"));

  Serial.print(F("Has data: "));
  Serial.println(pzem.hasData() ? F("yes") : F("no"));

  Serial.print(F("Request pending: "));
  Serial.println(pzem.isBusy() ? F("yes") : F("no"));

  Serial.print(F("Update interval: "));
  Serial.print(pzem.getUpdateInterval());
  Serial.println(F(" ms"));

  Serial.print(F("Last error: "));
  Serial.println(PZEM004T::errorToString(pzem.getLastError()));

  Serial.print(F("SoftwareSerial RX overflows: "));
  Serial.println(pzem.getRxOverflowCount());

  if (pzem.hasData()) {
    Serial.print(F("Last valid frame age: "));
    Serial.print(static_cast<uint32_t>(millis() - pzem.lastUpdateMillis()));
    Serial.println(F(" ms"));
  }
}

void processCommand(char command) {
  if (command >= 'a' && command <= 'z') {
    command = static_cast<char>(command - 'a' + 'A');
  }

  switch (command) {
    case 'L':
      csvLoggingEnabled = !csvLoggingEnabled;
      Serial.print(F("CSV logging: "));
      Serial.println(csvLoggingEnabled ? F("ON") : F("OFF"));
      if (csvLoggingEnabled) printCsvHeader();
      break;

    case 'R': resetEnergyCounter(); break;
    case 'A': configurePowerAlarm(); break;
    case 'G': showAlarmState(); break;
    case 'D': readInstalledAddress(); break;
    case 'S': changeAddress(); break;
    case 'C': showDiagnostics(); break;
    case 'H': case '?': printHelp(); break;

    default:
      Serial.println(F("Unknown command. Send H for help."));
      break;
  }
}

void processSerialInput() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == '\r' || command == '\n' || command == ' ') {
      continue;
    }
    processCommand(command);
  }
}

void reportErrorChange() {
  const PZEM004T::Error currentError = pzem.getLastError();
  if (currentError == previousError) return;

  previousError = currentError;
  if (currentError != PZEM004T::Error::NONE) {
    Serial.print(F("[PZEM] "));
    Serial.println(PZEM004T::errorToString(currentError));
  }
}

void setup() {
  Serial.begin(115200);

  Serial.println(F("PZEM004T - AdvancedFeatures"));
  Serial.print(F("Board: "));
  Serial.println(PZEM004T::boardName());

  if (!pzem.begin()) {
    Serial.print(F("PZEM initialization failed: "));
    Serial.println(PZEM004T::errorToString(pzem.getLastError()));
  }

  pzem.setUpdateInterval(POLL_INTERVAL_MS);
  Serial.print(F("Polling interval: "));
  Serial.print(POLL_INTERVAL_MS);
  Serial.println(F(" ms"));

  printHelp();
  if (csvLoggingEnabled) printCsvHeader();
}

void loop() {
  // Keep servicing the PZEM receive state machine as often as possible.
  if (pzem.update()) {
    const PZEM004T::Measurements data = pzem.getData();
    if (csvLoggingEnabled && data.valid) {
      printCsvRecord(data);
    }
  }

  // The normal measurement path remains non-blocking. Configuration commands
  // such as readAddress(), setAddress(), and setPowerAlarm() are synchronous;
  // they are used only when the user explicitly enters the corresponding key.
  processSerialInput();
  reportErrorChange();
}
