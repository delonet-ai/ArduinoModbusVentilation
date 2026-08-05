#pragma once

#include <Arduino.h>
#include <Servo.h>

#include "HardwareConfig.h"

#if SERVO_BACKEND == SERVO_BACKEND_PCA9685
#include "Pca9685Driver.h"
#endif

class SmoothServoChannel {
 public:
  bool begin(uint8_t output, uint16_t minPulseUs, uint16_t maxPulseUs,
             uint8_t initialPercent);
  void setTargetPercent(uint8_t openPercent);
  bool tick();
  void stop();
  bool isAtTarget() const;
  bool isHealthy() const;

 protected:
  virtual void primeOutput(uint16_t pulseUs) = 0;
  virtual bool attachOutput(uint8_t output, uint16_t minPulseUs,
                            uint16_t maxPulseUs) = 0;
  virtual bool writePulse(uint16_t pulseUs) = 0;
  virtual bool detachOutput() = 0;

 private:
  static constexpr uint16_t kUpdatePeriodMs = 20;
  static constexpr uint8_t kDetachDelayTicks = 10;

  uint16_t percentToPulse(uint8_t openPercent) const;
  bool ensureAttached();

  uint8_t output_ = 0;
  uint16_t minPulseUs_ = 500;
  uint16_t maxPulseUs_ = 2400;
  uint16_t targetPulseUs_ = 500;
  float currentPulseUs_ = 500.0f;
  float velocityUsPerSecond_ = 0.0f;
  unsigned long previousTickAt_ = 0;
  uint8_t settledTicks_ = 0;
  bool configured_ = false;
  bool attached_ = false;
  bool atTarget_ = false;
  bool healthy_ = true;
};

class DirectSmoothServo : public SmoothServoChannel {
 protected:
  void primeOutput(uint16_t pulseUs) override;
  bool attachOutput(uint8_t output, uint16_t minPulseUs,
                    uint16_t maxPulseUs) override;
  bool writePulse(uint16_t pulseUs) override;
  bool detachOutput() override;

 private:
  Servo servo_;
};

#if SERVO_BACKEND == SERVO_BACKEND_PCA9685
class Pca9685SmoothServo : public SmoothServoChannel {
 public:
  void configure(Pca9685Driver *driver);

 protected:
  void primeOutput(uint16_t pulseUs) override;
  bool attachOutput(uint8_t output, uint16_t minPulseUs,
                    uint16_t maxPulseUs) override;
  bool writePulse(uint16_t pulseUs) override;
  bool detachOutput() override;

 private:
  uint16_t pulseUsToTicks(uint16_t pulseUs) const;

  Pca9685Driver *driver_ = nullptr;
  uint8_t channel_ = 0;
};
#endif

class ServoBank {
 public:
  bool begin(const uint8_t initialPercent[HardwareConfig::kServoCount]);
  void setTargetPercent(uint8_t channel, uint8_t openPercent);
  void tick();
  void stop();
  bool isReady() const;
  bool isChannelAtTarget(uint8_t channel) const;
  bool areAllAtTarget() const;

 private:
  bool startChannel(uint8_t channel, uint8_t initialPercent);

#if SERVO_BACKEND == SERVO_BACKEND_DIRECT
  DirectSmoothServo servos_[HardwareConfig::kServoCount];
#else
  bool initializePca9685();
  Pca9685Driver driver_{HardwareConfig::kPca9685Address};
  Pca9685SmoothServo servos_[HardwareConfig::kServoCount];
  unsigned long lastRetryAt_ = 0;
#endif

  uint8_t targets_[HardwareConfig::kServoCount] = {};
  bool reached_[HardwareConfig::kServoCount] = {};
  bool ready_ = false;
};
