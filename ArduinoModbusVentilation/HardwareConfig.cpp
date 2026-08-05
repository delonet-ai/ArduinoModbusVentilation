#include "HardwareConfig.h"

namespace HardwareConfig {

const uint8_t kDirectServoPins[kServoCount] = {8, 9, 10, 11, 12, A0, A1, A2, A3, A4};
const uint8_t kPca9685Channels[kServoCount] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

// Tune these per channel only from the servo datasheet and bench measurements.
const uint16_t kServoMinPulseUs[kServoCount] = {
    500, 500, 500, 500, 500, 500, 500, 500, 500, 500};
const uint16_t kServoMaxPulseUs[kServoCount] = {
    2400, 2400, 2400, 2400, 2400, 2400, 2400, 2400, 2400, 2400};

}  // namespace HardwareConfig
