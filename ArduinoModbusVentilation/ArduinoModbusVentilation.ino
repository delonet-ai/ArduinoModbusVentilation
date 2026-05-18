#include <EEPROM.h>
#include <ModbusRTU.h>
#include <ServoSmooth.h>

const uint8_t SERVO_COUNT = 10;
const uint8_t TEST_MODE_REGISTER = 10;
const uint8_t CLOSED_ENDPOINT_REGISTER_BASE = 11;
const uint8_t OPEN_ENDPOINT_REGISTER_BASE = 21;
const uint8_t MODBUS_REGISTER_COUNT = OPEN_ENDPOINT_REGISTER_BASE + SERVO_COUNT;
const uint8_t UPTIME_INPUT_REGISTER_BASE = 100;
const uint8_t UPTIME_INPUT_REGISTER_COUNT = 2;

const uint8_t MODBUS_SLAVE_ID = 128;
const uint8_t RS485_TX_ENABLE_PIN = 2;
const uint32_t MODBUS_BAUD_RATE = 9600;

const uint8_t EEPROM_TARGETS_ADDRESS = 0;
const uint8_t EEPROM_CLOSED_ENDPOINTS_ADDRESS = EEPROM_TARGETS_ADDRESS + SERVO_COUNT;
const uint8_t EEPROM_OPEN_ENDPOINTS_ADDRESS = EEPROM_CLOSED_ENDPOINTS_ADDRESS + SERVO_COUNT;
const uint8_t EEPROM_MAGIC_ADDRESS = EEPROM_OPEN_ENDPOINTS_ADDRESS + SERVO_COUNT;
const uint8_t EEPROM_MAGIC_VALUE = 0xA1;
const uint8_t MIN_OPEN_PERCENT = 0;
const uint8_t MAX_OPEN_PERCENT = 100;

const uint8_t SERVO_SPEED = 50;
const float SERVO_ACCEL = 0.1;
const uint16_t SERVO_MIN_PULSE_US = 100;
const uint16_t SERVO_MAX_PULSE_US = 2500;

const uint8_t servoPins[SERVO_COUNT] = {8, 9, 10, 11, 12, A0, A1, A2, A3, A4};

ServoSmooth servos[SERVO_COUNT];
ModbusRTU mb;

uint8_t savedOpenPercent[SERVO_COUNT];
uint8_t savedClosedEndpoint[SERVO_COUNT];
uint8_t savedOpenEndpoint[SERVO_COUNT];
bool wasTestModeEnabled = false;

bool isTestModeEnabled();

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
  uint8_t calibratedOpenPercent = map(
    openPercent,
    MIN_OPEN_PERCENT,
    MAX_OPEN_PERCENT,
    savedClosedEndpoint[servoIndex],
    savedOpenEndpoint[servoIndex]
  );

  servos[servoIndex].setTargetDeg(percentToServoTarget(calibratedOpenPercent));
}

void applyStoredTargets() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    setServoOpenPercent(i, savedOpenPercent[i]);
  }
}

void initializeEndpointDefaultsIfNeeded() {
  if (EEPROM.read(EEPROM_MAGIC_ADDRESS) == EEPROM_MAGIC_VALUE) {
    return;
  }

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    EEPROM.update(EEPROM_CLOSED_ENDPOINTS_ADDRESS + i, MIN_OPEN_PERCENT);
    EEPROM.update(EEPROM_OPEN_ENDPOINTS_ADDRESS + i, MAX_OPEN_PERCENT);
  }

  EEPROM.update(EEPROM_MAGIC_ADDRESS, EEPROM_MAGIC_VALUE);
}

void loadTargetsFromEeprom() {
  initializeEndpointDefaultsIfNeeded();

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    savedOpenPercent[i] = clampOpenPercent(EEPROM.read(EEPROM_TARGETS_ADDRESS + i));
    savedClosedEndpoint[i] = clampOpenPercent(EEPROM.read(EEPROM_CLOSED_ENDPOINTS_ADDRESS + i));
    savedOpenEndpoint[i] = clampOpenPercent(EEPROM.read(EEPROM_OPEN_ENDPOINTS_ADDRESS + i));

    if (savedClosedEndpoint[i] == savedOpenEndpoint[i]) {
      savedClosedEndpoint[i] = MIN_OPEN_PERCENT;
      savedOpenEndpoint[i] = MAX_OPEN_PERCENT;
    }
  }
}

uint16_t handleEndpointRegisterWrite(uint8_t servoIndex, bool isOpenEndpoint, uint16_t value) {
  uint8_t requestedEndpoint = clampOpenPercent(value);
  uint8_t currentEndpoint = isOpenEndpoint ? savedOpenEndpoint[servoIndex] : savedClosedEndpoint[servoIndex];
  uint8_t oppositeEndpoint = isOpenEndpoint ? savedClosedEndpoint[servoIndex] : savedOpenEndpoint[servoIndex];

  if (requestedEndpoint == oppositeEndpoint) {
    return currentEndpoint;
  }

  if (requestedEndpoint != currentEndpoint) {
    if (isOpenEndpoint) {
      savedOpenEndpoint[servoIndex] = requestedEndpoint;
      EEPROM.update(EEPROM_OPEN_ENDPOINTS_ADDRESS + servoIndex, requestedEndpoint);
    } else {
      savedClosedEndpoint[servoIndex] = requestedEndpoint;
      EEPROM.update(EEPROM_CLOSED_ENDPOINTS_ADDRESS + servoIndex, requestedEndpoint);
    }

    if (!isTestModeEnabled()) {
      setServoOpenPercent(servoIndex, savedOpenPercent[servoIndex]);
    }
  }

  return requestedEndpoint;
}

uint16_t handleServoRegisterWrite(TRegister* reg, uint16_t value) {
  uint16_t address = reg->address.address;

  if (address == TEST_MODE_REGISTER) {
    return value == 0 ? 0 : 1;
  }

  if (address < SERVO_COUNT) {
    uint8_t servoIndex = address;
    uint8_t requestedOpenPercent = clampOpenPercent(value);

    if (requestedOpenPercent != savedOpenPercent[servoIndex]) {
      savedOpenPercent[servoIndex] = requestedOpenPercent;
      EEPROM.update(EEPROM_TARGETS_ADDRESS + servoIndex, requestedOpenPercent);

      if (!isTestModeEnabled()) {
        setServoOpenPercent(servoIndex, requestedOpenPercent);
      }
    }

    return requestedOpenPercent;
  }

  if (address >= CLOSED_ENDPOINT_REGISTER_BASE && address < CLOSED_ENDPOINT_REGISTER_BASE + SERVO_COUNT) {
    return handleEndpointRegisterWrite(address - CLOSED_ENDPOINT_REGISTER_BASE, false, value);
  }

  if (address >= OPEN_ENDPOINT_REGISTER_BASE && address < OPEN_ENDPOINT_REGISTER_BASE + SERVO_COUNT) {
    return handleEndpointRegisterWrite(address - OPEN_ENDPOINT_REGISTER_BASE, true, value);
  }

  return value;
}

uint16_t handleUptimeRegisterRead(TRegister* reg, uint16_t value) {
  uint32_t uptimeSeconds = millis() / 1000;
  uint16_t* uptimeWords = (uint16_t*)&uptimeSeconds;

  if (reg->address.address == UPTIME_INPUT_REGISTER_BASE) {
    return uptimeWords[1];
  }

  if (reg->address.address == UPTIME_INPUT_REGISTER_BASE + 1) {
    return uptimeWords[0];
  }

  return value;
}

void configureModbusRegisters() {
  mb.addHreg(0, 0, MODBUS_REGISTER_COUNT);
  mb.addIreg(UPTIME_INPUT_REGISTER_BASE, 0, UPTIME_INPUT_REGISTER_COUNT);

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    mb.Hreg(i, savedOpenPercent[i]);
    mb.Hreg(CLOSED_ENDPOINT_REGISTER_BASE + i, savedClosedEndpoint[i]);
    mb.Hreg(OPEN_ENDPOINT_REGISTER_BASE + i, savedOpenEndpoint[i]);
  }

  mb.Hreg(TEST_MODE_REGISTER, 0);
  mb.onSetHreg(0, handleServoRegisterWrite, MODBUS_REGISTER_COUNT);
  mb.onGetIreg(UPTIME_INPUT_REGISTER_BASE, handleUptimeRegisterRead, UPTIME_INPUT_REGISTER_COUNT);
}

void configureModbusTransport() {
  Serial.begin(MODBUS_BAUD_RATE, SERIAL_8N1);
  mb.begin(&Serial, RS485_TX_ENABLE_PIN);
  mb.setBaudrate(MODBUS_BAUD_RATE);
  mb.slave(MODBUS_SLAVE_ID);
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
  return mb.Hreg(TEST_MODE_REGISTER) != 0;
}

void setup() {
  loadTargetsFromEeprom();

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    servos[i].attach(servoPins[i], SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
    servos[i].setSpeed(SERVO_SPEED);
    servos[i].setAccel(SERVO_ACCEL);
  }

  applyStoredTargets();
  configureModbusRegisters();
  configureModbusTransport();
}

void loop() {
  mb.task();

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
  }

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    servos[i].tick();
  }

  yield();
}
