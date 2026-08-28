#include "ServoBank.h"

#include <math.h>
#include <string.h>

bool SmoothServoChannel::begin(uint8_t output, uint16_t minPulseUs,
                               uint16_t maxPulseUs, uint8_t initialPercent) {
  output_ = output;
  minPulseUs_ = minPulseUs;
  maxPulseUs_ = maxPulseUs;
  targetPulseUs_ = percentToPulse(initialPercent);
  currentPulseUs_ = targetPulseUs_;
  velocityUsPerSecond_ = 0.0f;
  settledTicks_ = 0;
  atTarget_ = true;
  healthy_ = true;

  primeOutput(targetPulseUs_);
  attached_ = attachOutput(output_, minPulseUs_, maxPulseUs_);
  configured_ = attached_;
  previousTickAt_ = millis();

  if (!attached_ || !writePulse(targetPulseUs_)) {
    healthy_ = false;
    return false;
  }

  return true;
}

void SmoothServoChannel::setTargetPercent(uint8_t openPercent) {
  if (!configured_) {
    return;
  }

  targetPulseUs_ = percentToPulse(openPercent);
  settledTicks_ = 0;
  atTarget_ = fabsf(currentPulseUs_ - targetPulseUs_) < 0.5f;

  if (!atTarget_ && !ensureAttached()) {
    healthy_ = false;
  }
}

bool SmoothServoChannel::tick() {
  if (!configured_ || !healthy_) {
    return false;
  }

  const unsigned long now = millis();
  if (static_cast<unsigned long>(now - previousTickAt_) < kUpdatePeriodMs) {
    return atTarget_;
  }
  previousTickAt_ = now;

  const float error = static_cast<float>(targetPulseUs_) - currentPulseUs_;
  if (fabsf(error) < 0.5f) {
    currentPulseUs_ = targetPulseUs_;
    velocityUsPerSecond_ = 0.0f;
    atTarget_ = true;

    if (attached_ && ++settledTicks_ >= kDetachDelayTicks) {
      healthy_ = detachOutput();
      attached_ = false;
    }
    return atTarget_ && healthy_;
  }

  if (!ensureAttached()) {
    healthy_ = false;
    return false;
  }

  const float pulseRange = static_cast<float>(maxPulseUs_ - minPulseUs_);
  const float maxSpeed = pulseRange * HardwareConfig::kServoSpeedDegreesPerSecond / 180.0f;
  const float acceleration =
      pulseRange * HardwareConfig::kServoAccelerationDegreesPerSecondSquared / 180.0f;
  const float direction = error > 0.0f ? 1.0f : -1.0f;
  const float desiredMagnitude = min(maxSpeed, sqrtf(2.0f * acceleration * fabsf(error)));
  const float desiredVelocity = direction * desiredMagnitude;
  const float velocityStep = acceleration * (kUpdatePeriodMs / 1000.0f);

  if (velocityUsPerSecond_ < desiredVelocity) {
    velocityUsPerSecond_ = min(velocityUsPerSecond_ + velocityStep, desiredVelocity);
  } else {
    velocityUsPerSecond_ = max(velocityUsPerSecond_ - velocityStep, desiredVelocity);
  }

  float nextPulse = currentPulseUs_ + velocityUsPerSecond_ * (kUpdatePeriodMs / 1000.0f);
  if ((direction > 0.0f && nextPulse >= targetPulseUs_) ||
      (direction < 0.0f && nextPulse <= targetPulseUs_)) {
    nextPulse = targetPulseUs_;
    velocityUsPerSecond_ = 0.0f;
    atTarget_ = true;
  } else {
    atTarget_ = false;
  }

  currentPulseUs_ = nextPulse;
  settledTicks_ = 0;
  healthy_ = writePulse(static_cast<uint16_t>(currentPulseUs_ + 0.5f));
  return atTarget_ && healthy_;
}

void SmoothServoChannel::stop() {
  if (attached_) {
    healthy_ = detachOutput();
    attached_ = false;
  }
  velocityUsPerSecond_ = 0.0f;
  atTarget_ = true;
}

bool SmoothServoChannel::isAtTarget() const {
  return configured_ && healthy_ && atTarget_;
}

bool SmoothServoChannel::isHealthy() const {
  return healthy_;
}

uint16_t SmoothServoChannel::percentToPulse(uint8_t openPercent) const {
  if (openPercent > HardwareConfig::kMaxOpenPercent) {
    openPercent = HardwareConfig::kMaxOpenPercent;
  }
  const uint32_t scaled = static_cast<uint32_t>(maxPulseUs_ - minPulseUs_) * openPercent;
  return minPulseUs_ + static_cast<uint16_t>((scaled + 50UL) / 100UL);
}

bool SmoothServoChannel::ensureAttached() {
  if (attached_) {
    return true;
  }

  attached_ = attachOutput(output_, minPulseUs_, maxPulseUs_);
  return attached_;
}

void DirectSmoothServo::primeOutput(uint16_t pulseUs) {
  servo_.writeMicroseconds(pulseUs);
}

bool DirectSmoothServo::attachOutput(uint8_t output, uint16_t minPulseUs,
                                     uint16_t maxPulseUs) {
  return servo_.attach(output, minPulseUs, maxPulseUs) != INVALID_SERVO;
}

bool DirectSmoothServo::writePulse(uint16_t pulseUs) {
  servo_.writeMicroseconds(pulseUs);
  return true;
}

bool DirectSmoothServo::detachOutput() {
  servo_.detach();
  return true;
}

#if SERVO_BACKEND == SERVO_BACKEND_PCA9685
void Pca9685SmoothServo::configure(Pca9685Driver *driver) {
  driver_ = driver;
}

void Pca9685SmoothServo::primeOutput(uint16_t pulseUs) {
  (void)pulseUs;
}

bool Pca9685SmoothServo::attachOutput(uint8_t output, uint16_t minPulseUs,
                                      uint16_t maxPulseUs) {
  (void)minPulseUs;
  (void)maxPulseUs;
  channel_ = output;
  return driver_ != nullptr;
}

bool Pca9685SmoothServo::writePulse(uint16_t pulseUs) {
  return driver_ != nullptr && driver_->setPwm(channel_, 0, pulseUsToTicks(pulseUs));
}

bool Pca9685SmoothServo::detachOutput() {
  return driver_ != nullptr && driver_->setPwm(channel_, 0, 4096);
}

uint16_t Pca9685SmoothServo::pulseUsToTicks(uint16_t pulseUs) const {
  const uint32_t scaled = static_cast<uint32_t>(pulseUs) *
                          HardwareConfig::kPca9685FrequencyHz * 4096UL;
  return static_cast<uint16_t>((scaled + 500000UL) / 1000000UL);
}
#endif

bool ServoBank::begin(const uint8_t initialPercent[HardwareConfig::kServoCount]) {
  memcpy(targets_, initialPercent, HardwareConfig::kServoCount);
  memset(reached_, false, sizeof(reached_));

#if SERVO_BACKEND == SERVO_BACKEND_DIRECT
  ready_ = true;
  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    if (!startChannel(i, targets_[i])) {
      ready_ = false;
    }
  }
#else
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  ready_ = initializePca9685();
  lastRetryAt_ = millis();
#endif

  return ready_;
}

void ServoBank::setTargetPercent(uint8_t channel, uint8_t openPercent) {
  if (channel >= HardwareConfig::kServoCount) {
    return;
  }

  if (openPercent > HardwareConfig::kMaxOpenPercent) {
    openPercent = HardwareConfig::kMaxOpenPercent;
  }
  targets_[channel] = openPercent;
  reached_[channel] = false;

  if (ready_) {
    servos_[channel].setTargetPercent(openPercent);
  }
}

void ServoBank::tick() {
#if SERVO_BACKEND == SERVO_BACKEND_PCA9685
  if (!ready_) {
    digitalWrite(LED_BUILTIN, HIGH);
    if (static_cast<unsigned long>(millis() - lastRetryAt_) >=
        HardwareConfig::kPca9685RetryIntervalMs) {
      lastRetryAt_ = millis();
      ready_ = initializePca9685();
    }
    return;
  }
#endif

  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    reached_[i] = servos_[i].tick();
    if (!servos_[i].isHealthy()) {
      ready_ = false;
    }
  }

#if SERVO_BACKEND == SERVO_BACKEND_PCA9685
  if (!ready_) {
    memset(reached_, false, sizeof(reached_));
    lastRetryAt_ = millis();
    digitalWrite(LED_BUILTIN, HIGH);
  }
#endif
}

void ServoBank::stop() {
  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    servos_[i].stop();
    reached_[i] = true;
  }
}

bool ServoBank::isReady() const {
  return ready_;
}

bool ServoBank::isChannelAtTarget(uint8_t channel) const {
  return channel < HardwareConfig::kServoCount && ready_ &&
         servos_[channel].isAtTarget();
}

bool ServoBank::areAllAtTarget() const {
  if (!ready_) {
    return false;
  }

  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    if (!servos_[i].isAtTarget()) {
      return false;
    }
  }
  return true;
}

bool ServoBank::startChannel(uint8_t channel, uint8_t initialPercent) {
#if SERVO_BACKEND == SERVO_BACKEND_DIRECT
  const uint8_t output = HardwareConfig::kDirectServoPins[channel];
#else
  servos_[channel].configure(&driver_);
  const uint8_t output = HardwareConfig::kPca9685Channels[channel];
#endif

  return servos_[channel].begin(output, HardwareConfig::kServoMinPulseUs[channel],
                                HardwareConfig::kServoMaxPulseUs[channel],
                                initialPercent);
}

#if SERVO_BACKEND == SERVO_BACKEND_PCA9685
bool ServoBank::initializePca9685() {
  ready_ = driver_.begin(HardwareConfig::kPca9685OscillatorHz,
                         HardwareConfig::kPca9685FrequencyHz);
  if (!ready_) {
    return false;
  }

  for (uint8_t channel = 0; channel < 16; ++channel) {
    if (!driver_.setPwm(channel, 0, 4096)) {
      ready_ = false;
      return false;
    }
  }

  for (uint8_t i = 0; i < HardwareConfig::kServoCount; ++i) {
    if (!startChannel(i, targets_[i])) {
      ready_ = false;
      return false;
    }
    reached_[i] = false;
  }

  digitalWrite(LED_BUILTIN, LOW);
  return true;
}
#endif
