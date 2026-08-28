#pragma once

#include <Arduino.h>

#include "HardwareConfig.h"

namespace AirflowGuard {

constexpr uint16_t kMinimumGroupTotal = 200;
constexpr uint8_t kRoomCount = 5;
constexpr uint8_t kExhaustAdjustedFlag = 0x01;
constexpr uint8_t kSupplyAdjustedFlag = 0x02;

struct Result {
  uint16_t exhaustTotal;
  uint16_t supplyTotal;
  uint8_t flags;
};

Result apply(const uint8_t requested[HardwareConfig::kServoCount], bool fireplaceMode,
             uint8_t applied[HardwareConfig::kServoCount]);
void normalizeRoomTargets(uint8_t roomTargets[kRoomCount]);

}  // namespace AirflowGuard
