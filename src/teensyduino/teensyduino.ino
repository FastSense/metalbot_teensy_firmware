#include "Common.hpp"
#include "ros-twist.hpp"

void setup() {
  delay(SETUP_DELAY);
  rclSetup();
}
void loop() { rclc_executor_spin(&exe); }
