#include <ModbusRtu.h>

#include "AirflowGuard.h"
#include "HardwareConfig.h"
#include "PersistentTargets.h"
#include "ServoBank.h"

constexpr uint8_t ROOM_REGISTER_START = 0;
constexpr uint8_t APPLIED_REGISTER_START = 5;
constexpr uint8_t EXHAUST_TOTAL_REGISTER = 15;
constexpr uint8_t SUPPLY_TOTAL_REGISTER = 16;
constexpr uint8_t SAFETY_FLAGS_REGISTER = 17;
constexpr uint8_t CONTROLLER_READY_REGISTER = 18;
constexpr uint8_t FIREPLACE_MODE_REGISTER = 19;
constexpr uint8_t MODBUS_REGISTER_COUNT = 20;

Modbus bus(HardwareConfig::kModbusSlaveId, Serial, HardwareConfig::kRs485TxEnablePin);
ServoBank servoBank;
PersistentTargets persistentTargets;

uint16_t modbusRegisters[MODBUS_REGISTER_COUNT] = {};
uint8_t roomOpenPercent[AirflowGuard::kRoomCount] = {};
uint8_t savedOpenPercent[HardwareConfig::kServoCount] = {};
uint8_t appliedOpenPercent[HardwareConfig::kServoCount] = {};
AirflowGuard::Result airflowResult = {};
bool fireplaceModeEnabled = false;

uint8_t clampOpenPercent(uint16_t value) {
  return value > HardwareConfig::kMaxOpenPercent
             ? HardwareConfig::kMaxOpenPercent
             : static_cast<uint8_t>(value);
}

void copyRoomsToPhysicalTargets() {
  for (uint8_t i = 0; i < AirflowGuard::kRoomCount; ++i) {
    savedOpenPercent[i] = roomOpenPercent[i];
    savedOpenPercent[i + AirflowGuard::kRoomCount] = roomOpenPercent[i];
  }
}

void updateAppliedTargets() {
  uint8_t nextApplied[HardwareConfig::kServoCount];
  airflowResult =
      AirflowGuard::apply(savedOpenPercent, fireplaceModeEnabled, nextApplied);

  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    if (nextApplied[i] != appliedOpenPercent[i]) {
      appliedOpenPercent[i] = nextApplied[i];
      servoBank.setTargetPercent(i, appliedOpenPercent[i]);
    }
  }
}

void publishRegisters() {
  for (uint8_t i = 0; i < AirflowGuard::kRoomCount; ++i) {
    modbusRegisters[ROOM_REGISTER_START + i] = roomOpenPercent[i];
  }
  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    modbusRegisters[APPLIED_REGISTER_START + i] = appliedOpenPercent[i];
  }
  modbusRegisters[EXHAUST_TOTAL_REGISTER] = airflowResult.exhaustTotal;
  modbusRegisters[SUPPLY_TOTAL_REGISTER] = airflowResult.supplyTotal;
  modbusRegisters[SAFETY_FLAGS_REGISTER] = airflowResult.flags;
  modbusRegisters[CONTROLLER_READY_REGISTER] = servoBank.isReady() ? 1 : 0;
}

void handleCommands() {
  uint8_t requestedRooms[AirflowGuard::kRoomCount];
  for (uint8_t i = 0; i < AirflowGuard::kRoomCount; ++i) {
    requestedRooms[i] = clampOpenPercent(modbusRegisters[ROOM_REGISTER_START + i]);
  }
  AirflowGuard::normalizeRoomTargets(requestedRooms);

  bool targetsChanged = false;
  for (uint8_t i = 0; i < AirflowGuard::kRoomCount; ++i) {
    if (requestedRooms[i] != roomOpenPercent[i]) {
      roomOpenPercent[i] = requestedRooms[i];
      targetsChanged = true;
    }
  }

  if (targetsChanged) {
    copyRoomsToPhysicalTargets();
    persistentTargets.scheduleSave(savedOpenPercent);
  }

  const bool requestedFireplaceMode = modbusRegisters[FIREPLACE_MODE_REGISTER] != 0;
  modbusRegisters[FIREPLACE_MODE_REGISTER] = requestedFireplaceMode ? 1 : 0;
  const bool fireplaceModeChanged = requestedFireplaceMode != fireplaceModeEnabled;
  fireplaceModeEnabled = requestedFireplaceMode;

  if (targetsChanged || fireplaceModeChanged) {
    updateAppliedTargets();
  }
}

void setup() {
  persistentTargets.begin(savedOpenPercent);

  bool migratedTargets = false;
  for (uint8_t i = 0; i < AirflowGuard::kRoomCount; ++i) {
    roomOpenPercent[i] = max(savedOpenPercent[i],
                             savedOpenPercent[i + AirflowGuard::kRoomCount]);
  }
  AirflowGuard::normalizeRoomTargets(roomOpenPercent);

  for (uint8_t i = 0; i < AirflowGuard::kRoomCount; ++i) {
    if (savedOpenPercent[i] != roomOpenPercent[i] ||
        savedOpenPercent[i + AirflowGuard::kRoomCount] != roomOpenPercent[i]) {
      migratedTargets = true;
    }
  }
  copyRoomsToPhysicalTargets();
  if (migratedTargets) {
    persistentTargets.scheduleSave(savedOpenPercent);
  }

  modbusRegisters[FIREPLACE_MODE_REGISTER] = 0;
  airflowResult = AirflowGuard::apply(savedOpenPercent, false, appliedOpenPercent);
  servoBank.begin(appliedOpenPercent);
  publishRegisters();

  Serial.begin(HardwareConfig::kModbusBaudRate, SERIAL_8N1);
  bus.start();
}

void loop() {
  bus.poll(modbusRegisters, MODBUS_REGISTER_COUNT);
  handleCommands();
  servoBank.tick();
  publishRegisters();
  persistentTargets.tick();
}
