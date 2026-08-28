#pragma once

#include <Arduino.h>

#include "HardwareConfig.h"

class PersistentTargets {
 public:
  void begin(uint8_t targets[HardwareConfig::kServoCount]);
  void scheduleSave(const uint8_t targets[HardwareConfig::kServoCount]);
  void tick();

 private:
  static constexpr uint16_t kRecordMagic = 0x564D;
  static constexpr uint8_t kRecordVersion = 1;
  static constexpr uint8_t kMigrationCompleteValue = 0xA5;
  static constexpr int kLegacyTargetsAddress = 0;
  static constexpr int kMigrationMarkerAddress = 16;
  static constexpr int kSlotAddresses[2] = {32, 64};
  static constexpr uint32_t kSaveDelayMs = 2000UL;

  struct __attribute__((packed)) Record {
    uint16_t magic;
    uint8_t version;
    uint16_t sequence;
    uint8_t targets[HardwareConfig::kServoCount];
    uint16_t crc;
  };

  bool readValidRecord(uint8_t slot, Record &record) const;
  bool loadLegacyTargets(uint8_t targets[HardwareConfig::kServoCount]) const;
  void writeRecord(const uint8_t targets[HardwareConfig::kServoCount]);
  uint16_t calculateCrc(const Record &record) const;
  static bool isSequenceNewer(uint16_t candidate, uint16_t reference);

  uint8_t pendingTargets_[HardwareConfig::kServoCount] = {};
  uint16_t activeSequence_ = 0;
  int8_t activeSlot_ = -1;
  unsigned long dirtySince_ = 0;
  bool dirty_ = false;
};
