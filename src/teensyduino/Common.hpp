#pragma once

#include <cstdint>
#include <math.h>

struct MotorPins {
  uint8_t pwm, front, back, encA, encB;
};

enum Wheels2 { L_wheel, R_wheel };
// enum Wheels4 { FL, FR, RL, RR };
enum kalmanX { KF_distance, KF_speed, KF_acceleration };

namespace config {
constexpr size_t ros_domain_id = 81; // {80,81}
constexpr int ipr = 1836;
constexpr float pi = 3.141593f;
constexpr float wheel_diameter = 0.195f; // 0.133f
constexpr float k_pwm = 200.0f;

constexpr float pwm_frequency = 36621.0f; // 585937.5f

constexpr uint8_t motors_count = 2; // 4
constexpr float base_width = 0.42f;

constexpr uint8_t pid_dt = 10;
// constexpr uint16_t stop_dt = 115;
constexpr uint16_t stop_dt = 205; //__DBG
constexpr uint16_t pub_dt = 200;  // 100
constexpr uint16_t setup_delay = 3000;
constexpr float fade = 0.01f;
} // namespace config
