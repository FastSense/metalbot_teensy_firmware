#pragma once

#include <cstdint>
#include <math.h>

#define DBG_ACTIVE true
#define DBG                                                                    \
  if (DBG_ACTIVE)                                                              \
  Serial5

struct MotorPins {
  uint8_t pwm, front, back, encA, encB;
};

// Enumeration of filter output values (vector X):
enum kalmanX { KF_distance, KF_speed, KF_acceleration };

enum Wheels { L_wheel, R_wheel };

namespace config {
constexpr float deadzone = 1.0f;
constexpr float p_max = 0.35f;
constexpr float i_max = 0.5f;
constexpr float d_max = 0.15f;
constexpr float kp = 0.015f;
constexpr float ki = 2.4f;
constexpr float kd = 0.005f;
constexpr float dt = 0.01f;
constexpr float model_noise = 25.0f;
constexpr float measurement_noise = 0.001f;

constexpr size_t ros_domain_id = 80; // {80,81}

constexpr int ipr = 1836;       // interrupts per rev
constexpr float k_pwm = 200.0f; // pid to pwm conversion coeff
constexpr float pi = 3.141593f;
constexpr float wheel_diameter = 0.195f;

constexpr float pwm_frequency = 36621.0f; // 585937.5f

constexpr uint8_t motors_count = 2; // 4
constexpr float base_width = 0.42f;

constexpr uint16_t pid_dt = 10;
constexpr uint16_t stop_dt = 115;
constexpr uint16_t pub_dt = 200;
constexpr uint16_t setup_delay = 100;
constexpr uint16_t reconnection_delay = 100;

constexpr float fade = 0.003f;      // integral error reduction coeff
constexpr float fade_cap = 0.0001f; // fade activation threshold (speed)

constexpr float soft_stop_cap = 0.1f; // maximum motors shutdown speed
} // namespace config

namespace pins {
constexpr MotorPins left_motor = {14, 15, 18, 5, 4};
constexpr MotorPins right_motor = {19, 22, 23, 6, 7};

constexpr uint8_t led_1 = 39;       // Green (dir_5 activation)
constexpr uint8_t led_2 = 37;       // Yellow (reinitialisation)
constexpr uint8_t dir_5 = 32;       // future:power key
constexpr uint8_t temperature = 26; // oneWire sensor pin
} // namespace pins
