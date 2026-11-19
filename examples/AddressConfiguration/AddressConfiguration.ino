/*
 * AddressConfiguration.ino
 * Configure or read one PZEM slave address from the Serial Monitor.
 * Target: ESP32 classic, Serial2 RX=GPIO16 and TX=GPIO17.
 * Commands are case-insensitive:
 *   read       - read the custom-address register
 *   set=0x02   - write a new address through the PZEM general address 0xF8
 *   help       - print this command list
 */

#include <PZEM004Tv30.h>
#include <stdlib.h>
#include <string.h>

PZEM004Tv30 pzem(Serial2, 16, 17, PZEM_DEFAULT_ADDR);
char commandBuffer[48];
uint8_t commandLength = 0;

void printHelp() {
  Serial.println(F("\nPZEM address tool"));
  Serial.println(F("  read       : read the installed PZEM custom address"));
  Serial.println(F("  set=0x02   : set address to 0x02 (valid range 0x01..0xF7)"));
  Serial.println(F("  help       : show this help"));
  Serial.println(F("Address write target: 0xF8 (PZEM general address)"));
}

void readInstalledAddress() {
  uint8_t address = 0;
  if (pzem.readAddress(address, true)) {
    Serial.print(F("[OK] Installed custom address: 0x"));
    if (address < 0x10) Serial.print('0');
    Serial.println(address, HEX);
  } else {
    Serial.print(F("[ERROR] Address read failed: "));
    Serial.println(PZEM004Tv30::errorToString(pzem.getLastError()));
  }
}

void processCommand(char* command) {
  // Convert the command to uppercase for case-insensitive matching.
  for (char* p = command; *p; ++p) {
    if (*p >= 'a' && *p <= 'z') *p = static_cast<char>(*p - 'a' + 'A');
  }

  if (strcmp(command, "HELP") == 0) {
    printHelp();
    return;
  }

  if (strcmp(command, "READ") == 0) {
    readInstalledAddress();
    return;
  }

  if (strncmp(command, "SET=", 4) == 0) {
    char* endPointer = nullptr;
    const unsigned long requested = strtoul(command + 4, &endPointer, 0);
    if (endPointer == command + 4 || *endPointer != '\0' ||
        requested < 0x01 || requested > 0xF7) {
      Serial.println(F("[ERROR] Address must be in the range 0x01..0xF7."));
      return;
    }

    const uint8_t newAddress = static_cast<uint8_t>(requested);
    Serial.print(F("Writing address 0x"));
    if (newAddress < 0x10) Serial.print('0');
    Serial.println(newAddress, HEX);

    // setAddress(newAddress) intentionally targets the PZEM general address 0xF8.
    if (!pzem.setAddress(newAddress)) {
      Serial.print(F("[ERROR] Write failed: "));
      Serial.println(PZEM004Tv30::errorToString(pzem.getLastError()));
      return;
    }

    Serial.println(F("Write command acknowledged. Verifying register..."));
    readInstalledAddress();
    return;
  }

  Serial.println(F("[ERROR] Unknown command. Type help."));
}

void setup() {
  Serial.begin(115200);
  pzem.setLogOutput(&Serial);

  Serial.println(F("PZEM004Tv30 v2.0.0 address configuration"));
  printHelp();
  Serial.println(F("Reading the installed PZEM once at startup..."));
  readInstalledAddress();
  Serial.println(F("Enter command and press Enter:"));
}

void loop() {
  while (Serial.available() > 0) {
    const char value = static_cast<char>(Serial.read());
    if (value == '\r') continue;

    if (value == '\n') {
      commandBuffer[commandLength] = '\0';
      if (commandLength > 0) processCommand(commandBuffer);
      commandLength = 0;
      continue;
    }

    if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = value;
    } else {
      commandLength = 0;
      Serial.println(F("[ERROR] Command too long; input cleared."));
    }
  }
}
