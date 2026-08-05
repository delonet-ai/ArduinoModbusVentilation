#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "EEPROM.h"
#include "PersistentTargets.h"

EEPROMClass EEPROM;
unsigned long fakeMillis = 0;

unsigned long millis() {
  return fakeMillis;
}

void assertAll(const uint8_t *values, uint8_t expected) {
  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    assert(values[i] == expected);
  }
}

void resetEnvironment() {
  EEPROM.reset();
  fakeMillis = 0;
}

void testCleanEepromUsesFailOpen() {
  resetEnvironment();
  PersistentTargets storage;
  uint8_t targets[HardwareConfig::kServoCount] = {};

  storage.begin(targets);

  assertAll(targets, HardwareConfig::kDefaultOpenPercent);
  assert(EEPROM.read(16) == 0xA5);
}

void testLegacyValuesAreMigratedOnce() {
  resetEnvironment();
  uint8_t expected[HardwareConfig::kServoCount];
  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    expected[i] = i * 10;
    EEPROM.update(i, expected[i]);
  }

  PersistentTargets storage;
  uint8_t targets[HardwareConfig::kServoCount] = {};
  storage.begin(targets);

  assert(memcmp(targets, expected, sizeof(expected)) == 0);
  assert(EEPROM.read(16) == 0xA5);
}

void testInvalidLegacyUsesFailOpen() {
  resetEnvironment();
  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    EEPROM.update(i, i);
  }
  EEPROM.update(4, 101);

  PersistentTargets storage;
  uint8_t targets[HardwareConfig::kServoCount] = {};
  storage.begin(targets);

  assertAll(targets, HardwareConfig::kDefaultOpenPercent);
}

void testSaveIsDelayedAndNewestSlotWins() {
  resetEnvironment();
  PersistentTargets storage;
  uint8_t targets[HardwareConfig::kServoCount] = {};
  storage.begin(targets);

  uint8_t changed[HardwareConfig::kServoCount];
  memset(changed, 42, sizeof(changed));
  storage.scheduleSave(changed);

  fakeMillis = 1999;
  storage.tick();
  PersistentTargets beforeDelay;
  uint8_t loadedBeforeDelay[HardwareConfig::kServoCount] = {};
  beforeDelay.begin(loadedBeforeDelay);
  assertAll(loadedBeforeDelay, HardwareConfig::kDefaultOpenPercent);

  fakeMillis = 2000;
  storage.tick();
  PersistentTargets afterDelay;
  uint8_t loadedAfterDelay[HardwareConfig::kServoCount] = {};
  afterDelay.begin(loadedAfterDelay);
  assertAll(loadedAfterDelay, 42);
}

void testCorruptNewestSlotFallsBackToPreviousSlot() {
  resetEnvironment();
  PersistentTargets storage;
  uint8_t targets[HardwareConfig::kServoCount] = {};
  storage.begin(targets);

  uint8_t changed[HardwareConfig::kServoCount];
  memset(changed, 25, sizeof(changed));
  storage.scheduleSave(changed);
  fakeMillis = 2000;
  storage.tick();

  // Slot 1 starts at 64. Byte 5 is the first target in the packed record.
  EEPROM.update(64 + 5, 26);

  PersistentTargets recovered;
  uint8_t recoveredTargets[HardwareConfig::kServoCount] = {};
  recovered.begin(recoveredTargets);
  assertAll(recoveredTargets, HardwareConfig::kDefaultOpenPercent);
}

void testMigrationMarkerPreventsLegacyReinterpretation() {
  resetEnvironment();
  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    EEPROM.update(i, 0);
  }
  EEPROM.update(16, 0xA5);

  PersistentTargets storage;
  uint8_t targets[HardwareConfig::kServoCount] = {};
  storage.begin(targets);

  assertAll(targets, HardwareConfig::kDefaultOpenPercent);
}

int main() {
  testCleanEepromUsesFailOpen();
  testLegacyValuesAreMigratedOnce();
  testInvalidLegacyUsesFailOpen();
  testSaveIsDelayedAndNewestSlotWins();
  testCorruptNewestSlotFallsBackToPreviousSlot();
  testMigrationMarkerPreventsLegacyReinterpretation();
  return 0;
}
