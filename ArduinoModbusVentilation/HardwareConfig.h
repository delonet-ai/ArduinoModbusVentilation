#pragma once

#include <Arduino.h>

#define SERVO_BACKEND_DIRECT 1
#define SERVO_BACKEND_PCA9685 2

#ifndef SERVO_BACKEND
#define SERVO_BACKEND SERVO_BACKEND_DIRECT
#endif

#if SERVO_BACKEND != SERVO_BACKEND_DIRECT && SERVO_BACKEND != SERVO_BACKEND_PCA9685
#error "SERVO_BACKEND must be SERVO_BACKEND_DIRECT or SERVO_BACKEND_PCA9685"
#endif

namespace HardwareConfig {

constexpr uint8_t kServoCount = 10;
constexpr uint8_t kModbusSlaveId = 128;
constexpr uint8_t kRs485TxEnablePin = 2;
constexpr uint32_t kModbusBaudRate = 9600;

constexpr uint8_t kMinOpenPercent = 0;
constexpr uint8_t kMaxOpenPercent = 100;
constexpr uint8_t kDefaultOpenPercent = 100;

constexpr uint8_t kServoSpeedDegreesPerSecond = 50;
constexpr uint16_t kServoAccelerationDegreesPerSecondSquared = 100;

extern const uint8_t kDirectServoPins[kServoCount];
extern const uint8_t kPca9685Channels[kServoCount];
extern const uint16_t kServoMinPulseUs[kServoCount];
extern const uint16_t kServoMaxPulseUs[kServoCount];

constexpr uint8_t kPca9685Address = 0x40;
constexpr uint16_t kPca9685FrequencyHz = 50;
constexpr uint32_t kPca9685OscillatorHz = 25000000UL;
constexpr uint32_t kPca9685RetryIntervalMs = 5000UL;

}  // namespace HardwareConfig
