#pragma once

#include "HardwareConfig.h"

#if SERVO_BACKEND == SERVO_BACKEND_PCA9685

#include <Arduino.h>
#include <Wire.h>

class Pca9685Driver {
 public:
  explicit Pca9685Driver(uint8_t address);

  bool begin(uint32_t oscillatorHz, uint16_t pwmFrequencyHz);
  bool setPwm(uint8_t channel, uint16_t onTick, uint16_t offTick);

 private:
  static constexpr uint8_t kMode1Register = 0x00;
  static constexpr uint8_t kLed0OnLowRegister = 0x06;
  static constexpr uint8_t kPrescaleRegister = 0xFE;
  static constexpr uint8_t kMode1Restart = 0x80;
  static constexpr uint8_t kMode1AutoIncrement = 0x20;
  static constexpr uint8_t kMode1Sleep = 0x10;

  bool setFrequency(uint32_t oscillatorHz, uint16_t pwmFrequencyHz);
  bool write8(uint8_t reg, uint8_t value);
  bool read8(uint8_t reg, uint8_t &value);

  uint8_t address_;
};

#endif
