#include "AirflowGuard.h"

namespace {

constexpr uint8_t kGroupSize = 5;

const uint8_t kExhaustChannels[kGroupSize] = {0, 1, 2, 3, 4};
const uint8_t kSupplyChannels[kGroupSize] = {5, 6, 7, 8, 9};

// Bedroom and child room receive the reserve airflow first.
const uint8_t kExhaustPriority[kGroupSize] = {2, 4, 0, 1, 3};
const uint8_t kSupplyPriority[kGroupSize] = {7, 9, 5, 6, 8};

uint16_t totalFor(const uint8_t values[HardwareConfig::kServoCount],
                  const uint8_t channels[kGroupSize]) {
  uint16_t total = 0;
  for (uint8_t i = 0; i < kGroupSize; ++i) {
    total += values[channels[i]];
  }
  return total;
}

bool raiseToMinimum(uint8_t values[HardwareConfig::kServoCount],
                    const uint8_t channels[kGroupSize],
                    const uint8_t priority[kGroupSize]) {
  uint16_t total = totalFor(values, channels);
  if (total >= AirflowGuard::kMinimumGroupTotal) {
    return false;
  }

  uint16_t deficit = AirflowGuard::kMinimumGroupTotal - total;
  for (uint8_t i = 0; i < kGroupSize && deficit > 0; ++i) {
    const uint8_t channel = priority[i];
    const uint8_t available = HardwareConfig::kMaxOpenPercent - values[channel];
    const uint8_t added = deficit < available ? static_cast<uint8_t>(deficit) : available;
    values[channel] += added;
    deficit -= added;
  }

  return true;
}

}  // namespace

AirflowGuard::Result AirflowGuard::apply(
    const uint8_t requested[HardwareConfig::kServoCount],
    bool fireplaceMode,
    uint8_t applied[HardwareConfig::kServoCount]) {
  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    applied[i] = requested[i] > HardwareConfig::kMaxOpenPercent
                     ? HardwareConfig::kMaxOpenPercent
                     : requested[i];
  }

  uint8_t flags = 0;
  if (raiseToMinimum(applied, kExhaustChannels, kExhaustPriority)) {
    flags |= kExhaustAdjustedFlag;
  }
  if (raiseToMinimum(applied, kSupplyChannels, kSupplyPriority)) {
    flags |= kSupplyAdjustedFlag;
  }

  if (fireplaceMode) {
    applied[5] = HardwareConfig::kMaxOpenPercent;  // Kitchen supply
    applied[8] = HardwareConfig::kMaxOpenPercent;  // Living room supply
  }

  Result result;
  result.exhaustTotal = totalFor(applied, kExhaustChannels);
  result.supplyTotal = totalFor(applied, kSupplyChannels);
  result.flags = flags;
  return result;
}

void AirflowGuard::normalizeRoomTargets(uint8_t roomTargets[kRoomCount]) {
  uint8_t requested[HardwareConfig::kServoCount];
  uint8_t applied[HardwareConfig::kServoCount];

  for (uint8_t i = 0; i < kRoomCount; ++i) {
    requested[i] = roomTargets[i];
    requested[i + kRoomCount] = roomTargets[i];
  }

  apply(requested, false, applied);
  for (uint8_t i = 0; i < kRoomCount; ++i) {
    roomTargets[i] = applied[i];
  }
}
