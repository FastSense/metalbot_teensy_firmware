#pragma once

#include "Common.hpp"
#include "Motor.hpp"

template <uint8_t N> class Robot {
public:
  Robot(float base_width) : base_width_(base_width) {}
  void start() {
    for (size_t i = 0; i < N; i++) {
      motors_[i].start();
    }
  }
  void reset() {
    for (size_t i = 0; i < N; i++) {
      motors_[i].reset();
    }
  }

  void updateTargetWheelsSpeed(double linear, double angular) {
    motors_targets_[FL] = motors_targets_[RL] = linear - angular * base_width_;
    motors_targets_[FR] = motors_targets_[RR] = linear + angular * base_width_;
  }

  void updateSpeedRegulation() {
    for (size_t i = 0; i < N; i++)
      motors_[i].updateSpeed(motors_targets_[i]);
  }
  void updateOdometry() {

    speed_ = (motors_[FL].getX(KF_speed) + motors_[FR].getX(KF_speed) +
              motors_[RL].getX(KF_speed) + motors_[RR].getX(KF_speed)) /
             N;

    angular_speed_ = (motors_[FR].getX(KF_speed) - motors_[FL].getX(KF_speed) +
                      motors_[RR].getX(KF_speed) - motors_[RL].getX(KF_speed)) /
                     base_width_ / N;

    angle_ = (motors_[FR].getX(KF_distance) - motors_[FL].getX(KF_distance) +
              motors_[RR].getX(KF_distance) - motors_[RL].getX(KF_distance)) /
             base_width_ / N;

    /*test calc of X and Y. In final it must to be calculated from kalman[0]!*/
    position_X_ += getSpeed() * config::pid_dt / 1000 * cos(getAngle());
    position_Y_ += getSpeed() * config::pid_dt / 1000 * sin(getAngle());

    quaternion_Z_ = sin(getAngle() / 2);
    quaternion_W_ = cos(getAngle() / 2);
  }

  void hardStopLoop() { /// when connection lost after ping
    while (1) {
      for (size_t i = 0; i < N; i++)
        motors_[i].setSpeed(0);
      delay(10);
    }
  }

  float getSpeed() { return speed_; }
  float getAngularSpeed() { return angular_speed_; }

  float getAngle() { return angle_; }

  float getQuaternionZ() { return quaternion_Z_; }
  float getQuaternionW() { return quaternion_W_; }

  float getPositionX() { return position_X_; }
  float getPositionY() { return position_Y_; }

private:
  float motors_targets_[N] = {0, 0, 0, 0};

  Motor motors_[N] = {
      Motor({8, 10, 9, 1, 0}),   // FL
      Motor({11, 13, 12, 2, 3}), // FR
      Motor({14, 18, 15, 5, 4}), // RL
      Motor({19, 23, 22, 6, 7}), // RR

      // Motor({14, 15, 18, 5, 4}),
      // Motor({19, 22, 23, 6, 7}),
      // Motor({8, 10, 9, 3, 2}),
      // Motor({11, 13, 12, 0, 1}),
  };

  float base_width_;

  float speed_ = 0;
  float angular_speed_ = 0;
  float position_X_ = 0;
  float position_Y_ = 0;
  float angle_ = 0;
  float quaternion_Z_ = 0;
  float quaternion_W_ = 1;
};
