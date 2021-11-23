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
    motors_targets_[L_wheel] = linear - angular * base_width_ / 2;
    motors_targets_[R_wheel] = linear + angular * base_width_ / 2;
  }

  void updateSpeedRegulation() {
    for (size_t i = 0; i < N; i++)
      motors_[i].updateSpeed(motors_targets_[i]);
  }
  void updateOdometry() {

    speed_ = (motors_[L_wheel].getX(KF_speed) + //
              motors_[R_wheel].getX(KF_speed)) /
             N;

    angular_speed_ = (motors_[R_wheel].getX(KF_speed) - //
                      motors_[L_wheel].getX(KF_speed)) /
                     base_width_;

    angle_ = (motors_[R_wheel].getX(KF_distance) -
              motors_[L_wheel].getX(KF_distance)) /
             base_width_;

    /*TODO: positon from kalman[KF_distance]*/
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
  float motors_targets_[N] = {0, 0};

  /*TODO: move pins to config */
  Motor motors_[N] = {
      Motor({14, 15, 18, 5, 4}), // L
      Motor({19, 22, 23, 6, 7}), // R

      // Motor({8, 10, 9, 1, 0}),   // L
      // Motor({11, 13, 12, 2, 3}), // R

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
