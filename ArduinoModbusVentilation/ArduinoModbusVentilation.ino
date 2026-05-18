#include <EEPROM.h>
#include <ModbusRtu.h>
#include <ServoSmooth.h>

const uint8_t SERVO_COUNT = 10;
const uint8_t TEST_MODE_REGISTER = 10;
const uint8_t MODBUS_REGISTER_COUNT = SERVO_COUNT + 1;

const uint8_t MODBUS_SLAVE_ID = 128;
const uint8_t RS485_TX_ENABLE_PIN = 2;
const uint32_t MODBUS_BAUD_RATE = 9600;

const uint8_t EEPROM_TARGETS_ADDRESS = 0;
const uint8_t MIN_OPEN_PERCENT = 0;
const uint8_t MAX_OPEN_PERCENT = 100;

const uint8_t SERVO_SPEED = 50;
const float SERVO_ACCEL = 0.1;
const uint16_t SERVO_MIN_PULSE_US = 100;
const uint16_t SERVO_MAX_PULSE_US = 2500;

const uint8_t servoPins[SERVO_COUNT] = {8, 9, 10, 11, 12, A0, A1, A2, A3, A4};

ServoSmooth servos[SERVO_COUNT];
Modbus bus;

uint16_t modbusRegisters[MODBUS_REGISTER_COUNT];
uint8_t savedOpenPercent[SERVO_COUNT];
bool wasTestModeEnabled = false;

uint8_t clampOpenPercent(uint16_t value) {
  if (value > MAX_OPEN_PERCENT) {
    return MAX_OPEN_PERCENT;
  }

  return value;
}

int percentToServoTarget(uint8_t openPercent) {
  return map(openPercent, MIN_OPEN_PERCENT, MAX_OPEN_PERCENT, 0, 180);
}

void setServoOpenPercent(uint8_t servoIndex, uint8_t openPercent) {
  servos[servoIndex].setTargetDeg(percentToServoTarget(openPercent));
}

void applyStoredTargets() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    setServoOpenPercent(i, savedOpenPercent[i]);
  }
}

void loadTargetsFromEeprom() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    savedOpenPercent[i] = clampOpenPercent(EEPROM.read(EEPROM_TARGETS_ADDRESS + i));
    modbusRegisters[i] = savedOpenPercent[i];
  }

  modbusRegisters[TEST_MODE_REGISTER] = 0;
}

void handleNormalMode() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    uint8_t requestedOpenPercent = clampOpenPercent(modbusRegisters[i]);

    if (modbusRegisters[i] != requestedOpenPercent) {
      modbusRegisters[i] = requestedOpenPercent;
    }

    if (requestedOpenPercent != savedOpenPercent[i]) {
      savedOpenPercent[i] = requestedOpenPercent;
      EEPROM.update(EEPROM_TARGETS_ADDRESS + i, requestedOpenPercent);
      setServoOpenPercent(i, requestedOpenPercent);
    }
  }
}

// SERVO_TEST_MODE: temporary WB-controlled test mode.
// Delete this block, TEST_MODE_REGISTER, and the WB "Test Mode" channel when it is no longer needed.
const uint16_t TEST_SERVO_CYCLE_MS = 5000;
const uint16_t TEST_SERVO_HALF_CYCLE_MS = TEST_SERVO_CYCLE_MS / 2;

uint8_t testServoIndex = 0;
unsigned long testServoStartedAt = 0;

void resetServoTest() {
  testServoIndex = 0;
  testServoStartedAt = millis();
}

uint8_t testOpenPercentForElapsed(unsigned long elapsedMs) {
  if (elapsedMs >= TEST_SERVO_CYCLE_MS) {
    return MIN_OPEN_PERCENT;
  }

  if (elapsedMs <= TEST_SERVO_HALF_CYCLE_MS) {
    return map(elapsedMs, 0, TEST_SERVO_HALF_CYCLE_MS, MIN_OPEN_PERCENT, MAX_OPEN_PERCENT);
  }

  return map(elapsedMs, TEST_SERVO_HALF_CYCLE_MS, TEST_SERVO_CYCLE_MS, MAX_OPEN_PERCENT, MIN_OPEN_PERCENT);
}

void handleServoTestMode() {
  unsigned long now = millis();
  unsigned long elapsedMs = now - testServoStartedAt;

  if (elapsedMs >= TEST_SERVO_CYCLE_MS) {
    testServoIndex = (testServoIndex + 1) % SERVO_COUNT;
    testServoStartedAt = now;
    elapsedMs = 0;
  }

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    setServoOpenPercent(i, i == testServoIndex ? testOpenPercentForElapsed(elapsedMs) : MIN_OPEN_PERCENT);
  }
}
// End SERVO_TEST_MODE.

bool isTestModeEnabled() {
  return modbusRegisters[TEST_MODE_REGISTER] != 0;
}

void setup() {
  loadTargetsFromEeprom();

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    servos[i].attach(servoPins[i], SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
    servos[i].setSpeed(SERVO_SPEED);
    servos[i].setAccel(SERVO_ACCEL);
  }

  applyStoredTargets();

  bus = Modbus(MODBUS_SLAVE_ID, 1, RS485_TX_ENABLE_PIN);
  bus.begin(MODBUS_BAUD_RATE);
}

void loop() {
  bus.poll(modbusRegisters, MODBUS_REGISTER_COUNT);

  bool testModeEnabled = isTestModeEnabled();

  if (testModeEnabled && !wasTestModeEnabled) {
    resetServoTest();
  }

  if (!testModeEnabled && wasTestModeEnabled) {
    applyStoredTargets();
  }

  wasTestModeEnabled = testModeEnabled;

  if (testModeEnabled) {
    handleServoTestMode();
  } else {
    handleNormalMode();
  }

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    servos[i].tick();
  }
}
