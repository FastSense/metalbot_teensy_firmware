#ifndef COMMON_362942634
#define COMMON_362942634

#include <cstdint>
#include <math.h>

struct MotorPins {
  uint8_t pwm, front, back, encA, encB;
};

namespace config {

constexpr uint8_t pid_dt = 10;
constexpr uint16_t stop_dt = 215;
constexpr uint16_t pub_dt = 100;
constexpr uint16_t setup_delay = 2000;
constexpr float fade = 0.01f;
} // namespace config

#endif /* end of include guard: COMMON_362942634 */
