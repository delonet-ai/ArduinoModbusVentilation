#include "Pca9685Driver.h"

#if SERVO_BACKEND == SERVO_BACKEND_PCA9685

Pca9685Driver::Pca9685Driver(uint8_t address) : address_(address) {}

bool Pca9685Driver::begin(uint32_t oscillatorHz, uint16_t pwmFrequencyHz) {
  Wire.begin();
  Wire.setWireTimeout(25000UL, true);

  Wire.beginTransmission(address_);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  if (!write8(kMode1Register, kMode1Restart)) {
    return false;
  }
  delayMicroseconds(500);

  return setFrequency(oscillatorHz, pwmFrequencyHz);
}

bool Pca9685Driver::setPwm(uint8_t channel, uint16_t onTick, uint16_t offTick) {
  if (channel >= 16 || onTick > 4096 || offTick > 4096) {
    return false;
  }

  Wire.beginTransmission(address_);
  Wire.write(static_cast<uint8_t>(kLed0OnLowRegister + 4 * channel));
  Wire.write(static_cast<uint8_t>(onTick));
  Wire.write(static_cast<uint8_t>(onTick >> 8));
  Wire.write(static_cast<uint8_t>(offTick));
  Wire.write(static_cast<uint8_t>(offTick >> 8));
  return Wire.endTransmission() == 0;
}

bool Pca9685Driver::setFrequency(uint32_t oscillatorHz, uint16_t pwmFrequencyHz) {
  if (pwmFrequencyHz == 0) {
    return false;
  }

  const uint32_t denominator = 4096UL * pwmFrequencyHz;
  uint32_t prescale = (oscillatorHz + denominator / 2) / denominator;
  if (prescale == 0) {
    return false;
  }
  --prescale;
  prescale = constrain(prescale, 3UL, 255UL);

  uint8_t oldMode;
  if (!read8(kMode1Register, oldMode)) {
    return false;
  }

  const uint8_t sleepMode = (oldMode & ~kMode1Restart) | kMode1Sleep;
  if (!write8(kMode1Register, sleepMode) ||
      !write8(kPrescaleRegister, static_cast<uint8_t>(prescale)) ||
      !write8(kMode1Register, oldMode)) {
    return false;
  }

  delayMicroseconds(500);
  return write8(kMode1Register, oldMode | kMode1Restart | kMode1AutoIncrement);
}

bool Pca9685Driver::write8(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address_);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool Pca9685Driver::read8(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(address_);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(address_, static_cast<uint8_t>(1)) != 1) {
    return false;
  }
  value = Wire.read();
  return true;
}

#endif
