#include "PersistentTargets.h"

#include <EEPROM.h>
#include <stddef.h>
#include <string.h>

constexpr int PersistentTargets::kSlotAddresses[2];

void PersistentTargets::begin(uint8_t targets[HardwareConfig::kServoCount]) {
  Record records[2];
  const bool slot0Valid = readValidRecord(0, records[0]);
  const bool slot1Valid = readValidRecord(1, records[1]);

  if (slot0Valid || slot1Valid) {
    if (slot1Valid && (!slot0Valid || isSequenceNewer(records[1].sequence, records[0].sequence))) {
      activeSlot_ = 1;
    } else {
      activeSlot_ = 0;
    }

    activeSequence_ = records[activeSlot_].sequence;
    memcpy(targets, records[activeSlot_].targets, HardwareConfig::kServoCount);
    return;
  }

  bool migratedLegacy = false;
  if (EEPROM.read(kMigrationMarkerAddress) != kMigrationCompleteValue) {
    migratedLegacy = loadLegacyTargets(targets);
  }

  if (!migratedLegacy) {
    memset(targets, HardwareConfig::kDefaultOpenPercent, HardwareConfig::kServoCount);
  }

  writeRecord(targets);
  EEPROM.update(kMigrationMarkerAddress, kMigrationCompleteValue);
}

void PersistentTargets::scheduleSave(const uint8_t targets[HardwareConfig::kServoCount]) {
  memcpy(pendingTargets_, targets, HardwareConfig::kServoCount);
  dirtySince_ = millis();
  dirty_ = true;
}

void PersistentTargets::tick() {
  if (dirty_ && static_cast<unsigned long>(millis() - dirtySince_) >= kSaveDelayMs) {
    writeRecord(pendingTargets_);
    dirty_ = false;
  }
}

bool PersistentTargets::readValidRecord(uint8_t slot, Record &record) const {
  EEPROM.get(kSlotAddresses[slot], record);

  if (record.magic != kRecordMagic || record.version != kRecordVersion) {
    return false;
  }

  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    if (record.targets[i] > HardwareConfig::kMaxOpenPercent) {
      return false;
    }
  }

  return record.crc == calculateCrc(record);
}

bool PersistentTargets::loadLegacyTargets(uint8_t targets[HardwareConfig::kServoCount]) const {
  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    const uint8_t value = EEPROM.read(kLegacyTargetsAddress + i);
    if (value > HardwareConfig::kMaxOpenPercent) {
      return false;
    }
    targets[i] = value;
  }

  return true;
}

void PersistentTargets::writeRecord(const uint8_t targets[HardwareConfig::kServoCount]) {
  Record record;
  record.magic = kRecordMagic;
  record.version = kRecordVersion;
  record.sequence = static_cast<uint16_t>(activeSequence_ + 1);
  memcpy(record.targets, targets, HardwareConfig::kServoCount);
  record.crc = calculateCrc(record);

  const uint8_t nextSlot = activeSlot_ == 0 ? 1 : 0;
  EEPROM.put(kSlotAddresses[nextSlot], record);

  Record verification;
  if (readValidRecord(nextSlot, verification) && verification.sequence == record.sequence) {
    activeSlot_ = nextSlot;
    activeSequence_ = record.sequence;
  }
}

uint16_t PersistentTargets::calculateCrc(const Record &record) const {
  const uint8_t *data = reinterpret_cast<const uint8_t *>(&record);
  const size_t length = offsetof(Record, crc);
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }

  return crc;
}

bool PersistentTargets::isSequenceNewer(uint16_t candidate, uint16_t reference) {
  return static_cast<int16_t>(candidate - reference) > 0;
}
