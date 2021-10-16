#ifndef ROBOT_247502943
#define ROBOT_247502943

#include "Motor.hpp"

struct vector3_t {
  double x, y, z;
};

struct Twist_t {
  vector3_t linear, angular;
};

class Robot_t {

  float base_width_ = 0.51; // virtual

public:
  double target_[4] = {0, 0, 0, 0}; // _DBG!!!!!!
  Motor motors_[4] = {
      // _DBG!!!!!!
      Motor({8, 10, 9, 1, 0}),   // A
      Motor({11, 13, 12, 2, 3}), // B
      Motor({14, 18, 15, 5, 4}), // C
      Motor({19, 23, 22, 6, 7})  // D
  };
  void start() {
    for (size_t i = 0; i < 4; i++) {
      motors_[i].start();
    }
  }
  void reset() {
    for (size_t i = 0; i < 4; i++) {
      motors_[i].reset();
    }
  }

  void updateTargetWheelsSpeed(Twist_t twist) {
    target_[0] = target_[2] = twist.linear.x - twist.angular.z * base_width_;
    target_[1] = target_[3] = twist.linear.x + twist.angular.z * base_width_;
  }

  void updateSpeedRegulation() {
    for (size_t i = 0; i < 4; i++)
      motors_[i].updateSpeed(target_[i]);
  }
  double getV() {
    return (motors_[0].getX(1) + motors_[1].getX(1) + motors_[2].getX(1) +
            motors_[3].getX(1)) /
           4;
  }
  double getW() {
    return (motors_[1].getX(1) - motors_[0].getX(1) + motors_[3].getX(1) -
            motors_[2].getX(1)) /
           2;
  };
};

#endif /* end of include guard: ROBOT_247502943 */
