#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "AirflowGuard.h"

void testNoAdjustmentWhenBothGroupsAreLargeEnough() {
  const uint8_t requested[HardwareConfig::kServoCount] = {
      40, 40, 40, 40, 40, 50, 50, 50, 50, 50};
  uint8_t applied[HardwareConfig::kServoCount] = {};

  const AirflowGuard::Result result = AirflowGuard::apply(requested, false, applied);

  assert(memcmp(requested, applied, sizeof(requested)) == 0);
  assert(result.exhaustTotal == 200);
  assert(result.supplyTotal == 250);
  assert(result.flags == 0);
}

void testBedroomThenChildRoomReceiveExhaustReserve() {
  const uint8_t requested[HardwareConfig::kServoCount] = {
      0, 0, 0, 0, 0, 40, 40, 40, 40, 40};
  uint8_t applied[HardwareConfig::kServoCount] = {};

  const AirflowGuard::Result result = AirflowGuard::apply(requested, false, applied);

  assert(applied[2] == 100);
  assert(applied[4] == 100);
  assert(applied[0] == 0);
  assert(result.exhaustTotal == 200);
  assert(result.flags == AirflowGuard::kExhaustAdjustedFlag);
}

void testSupplyReserveUsesPriorityAndKeepsRequestedValues() {
  const uint8_t requested[HardwareConfig::kServoCount] = {
      40, 40, 40, 40, 40, 10, 10, 80, 10, 20};
  uint8_t applied[HardwareConfig::kServoCount] = {};

  const AirflowGuard::Result result = AirflowGuard::apply(requested, false, applied);

  assert(applied[7] == 100);
  assert(applied[9] == 40);
  assert(applied[5] == 10);
  assert(applied[6] == 10);
  assert(applied[8] == 10);
  assert(result.supplyTotal == 200);
  assert(result.flags == AirflowGuard::kSupplyAdjustedFlag);
}

void testBothGroupsAreAdjustedIndependently() {
  const uint8_t requested[HardwareConfig::kServoCount] = {};
  uint8_t applied[HardwareConfig::kServoCount] = {};

  const AirflowGuard::Result result = AirflowGuard::apply(requested, false, applied);

  assert(result.exhaustTotal == 200);
  assert(result.supplyTotal == 200);
  assert(result.flags ==
         (AirflowGuard::kExhaustAdjustedFlag | AirflowGuard::kSupplyAdjustedFlag));
  assert(applied[2] == 100 && applied[4] == 100);
  assert(applied[7] == 100 && applied[9] == 100);
}

void testFireplaceModeOnlyForcesKitchenAndLivingSupply() {
  const uint8_t requested[HardwareConfig::kServoCount] = {
      40, 40, 40, 40, 40, 50, 50, 50, 50, 50};
  uint8_t applied[HardwareConfig::kServoCount] = {};

  const AirflowGuard::Result result = AirflowGuard::apply(requested, true, applied);

  assert(applied[5] == 100);
  assert(applied[8] == 100);
  assert(applied[0] == 40);
  assert(applied[3] == 40);
  assert(applied[6] == 50);
  assert(applied[7] == 50);
  assert(applied[9] == 50);
  assert(result.exhaustTotal == 200);
  assert(result.supplyTotal == 350);
  assert(result.flags == 0);
}

void testRoomTargetsAreNormalizedAndRemainPaired() {
  uint8_t rooms[AirflowGuard::kRoomCount] = {};

  AirflowGuard::normalizeRoomTargets(rooms);

  assert(rooms[0] == 0);
  assert(rooms[1] == 0);
  assert(rooms[2] == 100);
  assert(rooms[3] == 0);
  assert(rooms[4] == 100);
}

void testRoomTargetsAboveMinimumAreNotChanged() {
  uint8_t rooms[AirflowGuard::kRoomCount] = {100, 0, 0, 0, 100};

  AirflowGuard::normalizeRoomTargets(rooms);

  assert(rooms[0] == 100);
  assert(rooms[1] == 0);
  assert(rooms[2] == 0);
  assert(rooms[3] == 0);
  assert(rooms[4] == 100);
}

int main() {
  testNoAdjustmentWhenBothGroupsAreLargeEnough();
  testBedroomThenChildRoomReceiveExhaustReserve();
  testSupplyReserveUsesPriorityAndKeepsRequestedValues();
  testBothGroupsAreAdjustedIndependently();
  testFireplaceModeOnlyForcesKitchenAndLivingSupply();
  testRoomTargetsAreNormalizedAndRemainPaired();
  testRoomTargetsAboveMinimumAreNotChanged();
  return 0;
}
