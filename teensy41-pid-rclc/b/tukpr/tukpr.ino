#include "ros-twist.hpp"
#define EXE_TIMEOUT 500 // ms

void setup() {
  delay(2000);
  rclSetup();
}

void loop() {
  delay(1);                                                 // ms
  rclc_executor_spin_some(&exe, EXE_TIMEOUT * 1000 * 1000); // ns
}
