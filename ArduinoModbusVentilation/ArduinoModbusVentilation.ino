#include <ModbusRtu.h>

#include "HardwareConfig.h"
#include "PersistentTargets.h"
#include "ServoBank.h"

constexpr uint8_t TEST_MODE_REGISTER = HardwareConfig::kServoCount;
constexpr uint8_t MODBUS_REGISTER_COUNT = HardwareConfig::kServoCount + 1;
constexpr uint16_t TEST_ENDPOINT_HOLD_MS = 250;

enum class TestState : uint8_t {
  Idle,
  PreparingZero,
  Opening,
  HoldingOpen,
  Closing,
  HoldingClosed,
};

Modbus bus(HardwareConfig::kModbusSlaveId, Serial, HardwareConfig::kRs485TxEnablePin);
ServoBank servoBank;
PersistentTargets persistentTargets;

uint16_t modbusRegisters[MODBUS_REGISTER_COUNT] = {};
uint8_t savedOpenPercent[HardwareConfig::kServoCount] = {};
uint8_t testSnapshot[HardwareConfig::kServoCount] = {};

TestState testState = TestState::Idle;
uint8_t testServoIndex = 0;
unsigned long testStateStartedAt = 0;

uint8_t clampOpenPercent(uint16_t value) {
  return value > HardwareConfig::kMaxOpenPercent
             ? HardwareConfig::kMaxOpenPercent
             : static_cast<uint8_t>(value);
}

bool isTestModeEnabled() {
  return modbusRegisters[TEST_MODE_REGISTER] != 0;
}

void enterTestMode() {
  bool targetsChanged = false;

  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    const uint8_t snapshotValue = clampOpenPercent(modbusRegisters[i]);
    modbusRegisters[i] = snapshotValue;
    testSnapshot[i] = snapshotValue;

    if (savedOpenPercent[i] != snapshotValue) {
      savedOpenPercent[i] = snapshotValue;
      targetsChanged = true;
    }

    servoBank.setTargetPercent(i, HardwareConfig::kMinOpenPercent);
  }

  if (targetsChanged) {
    persistentTargets.scheduleSave(savedOpenPercent);
  }

  modbusRegisters[TEST_MODE_REGISTER] = 1;
  testServoIndex = 0;
  testState = TestState::PreparingZero;
}

void exitTestMode() {
  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    modbusRegisters[i] = testSnapshot[i];
    savedOpenPercent[i] = testSnapshot[i];
    servoBank.setTargetPercent(i, testSnapshot[i]);
  }

  modbusRegisters[TEST_MODE_REGISTER] = 0;
  testState = TestState::Idle;
}

void preserveTestSnapshotInRegisters() {
  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    modbusRegisters[i] = testSnapshot[i];
  }
  modbusRegisters[TEST_MODE_REGISTER] = 1;
}

void handleTestMode() {
  const unsigned long now = millis();

  switch (testState) {
    case TestState::PreparingZero:
      if (servoBank.areAllAtTarget()) {
        servoBank.setTargetPercent(testServoIndex, HardwareConfig::kMaxOpenPercent);
        testState = TestState::Opening;
      }
      break;

    case TestState::Opening:
      if (servoBank.isChannelAtTarget(testServoIndex)) {
        testStateStartedAt = now;
        testState = TestState::HoldingOpen;
      }
      break;

    case TestState::HoldingOpen:
      if (static_cast<unsigned long>(now - testStateStartedAt) >= TEST_ENDPOINT_HOLD_MS) {
        servoBank.setTargetPercent(testServoIndex, HardwareConfig::kMinOpenPercent);
        testState = TestState::Closing;
      }
      break;

    case TestState::Closing:
      if (servoBank.isChannelAtTarget(testServoIndex)) {
        testStateStartedAt = now;
        testState = TestState::HoldingClosed;
      }
      break;

    case TestState::HoldingClosed:
      if (static_cast<unsigned long>(now - testStateStartedAt) >= TEST_ENDPOINT_HOLD_MS) {
        testServoIndex = (testServoIndex + 1) % HardwareConfig::kServoCount;
        servoBank.setTargetPercent(testServoIndex, HardwareConfig::kMaxOpenPercent);
        testState = TestState::Opening;
      }
      break;

    case TestState::Idle:
      break;
  }
}

void handleNormalMode() {
  bool targetsChanged = false;

  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    const uint8_t requestedOpenPercent = clampOpenPercent(modbusRegisters[i]);
    modbusRegisters[i] = requestedOpenPercent;

    if (requestedOpenPercent != savedOpenPercent[i]) {
      savedOpenPercent[i] = requestedOpenPercent;
      servoBank.setTargetPercent(i, requestedOpenPercent);
      targetsChanged = true;
    }
  }

  if (targetsChanged) {
    persistentTargets.scheduleSave(savedOpenPercent);
  }
}

void setup() {
  persistentTargets.begin(savedOpenPercent);

  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    modbusRegisters[i] = savedOpenPercent[i];
  }
  modbusRegisters[TEST_MODE_REGISTER] = 0;

  servoBank.begin(savedOpenPercent);

  Serial.begin(HardwareConfig::kModbusBaudRate, SERIAL_8N1);
  bus.start();
}

void loop() {
  bus.poll(modbusRegisters, MODBUS_REGISTER_COUNT);

  const bool requestedTestMode = isTestModeEnabled();
  const bool testModeActive = testState != TestState::Idle;

  if (requestedTestMode && !testModeActive) {
    enterTestMode();
  } else if (!requestedTestMode && testModeActive) {
    exitTestMode();
  }

  if (testState != TestState::Idle) {
    preserveTestSnapshotInRegisters();
    handleTestMode();
  } else {
    handleNormalMode();
  }

  servoBank.tick();
  persistentTargets.tick();
}
