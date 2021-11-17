#include "Common.hpp"
#include "Ros.hpp"

#include "Filter.hpp"
#include "Pid.hpp"
#include <Array.h>

void setup() {
  float deadzone = 0;
  float limit = 1;
  float kp = 0.2;
  float ki = 2.5;
  float kd = 0.0015;
  float dt = 0.01;
  float model_noise = 25.0;
  float measurement_noise = 0.001;

  delay(config::setup_delay);
  rclSetup();
}

void loop() { rclc_executor_spin(&exe); }
