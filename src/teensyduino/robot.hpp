#pragma once

#include "battery.hpp"
#include "common.hpp"
#include "motor.hpp"
#include "temperature_sensor.hpp"

template <uint8_t N> class Robot {
public:
  Robot(float base_width) : base_width_(base_width) {}

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

    position_X_ += getSpeed() * config::pid_dt / 1000 * cos(getAngle());
    position_Y_ += getSpeed() * config::pid_dt / 1000 * sin(getAngle());

    quaternion_Z_ = sin(getAngle() / 2);
    quaternion_W_ = cos(getAngle() / 2);
  }

  bool softStop() {
    DBG.print("Мягкая остановка робота.. ");
    updateTargetWheelsSpeed(0, 0);
    size_t safe_state = 0;
    for (size_t i = 0; i < N; i++) {
      float remaining = motors_[i].getX(KF_speed);
      if (remaining < config::soft_stop_cap &&
          remaining > -config::soft_stop_cap)
        safe_state++;
    }
    if (safe_state == N) {
      DBG.println("завершена");
      return true;
    } else
      return false;
  }
  void resetOdom() {
    position_X_ = 0;
    position_Y_ = 0;
    quaternion_W_ = 0;
    quaternion_Z_ = 0;
  }

  void stop() { /// when connection lost after ping
    DBG.print("Выключение робота.. ");
    updateTargetWheelsSpeed(0, 0);
    for (size_t i = 0; i < N; i++)
      motors_[i].stop();
    DBG.println("завершено");
    resetOdom();
  }

  void activate() { /// when connection lost after ping
    DBG.print("Активация робота.. ");
    updateTargetWheelsSpeed(0, 0);
    for (size_t i = 0; i < N; i++)
      motors_[i].activate();
    DBG.println("завершена");
    resetOdom();
  }

  void init() {
    TemperatureSensor::start();
    battery::start();
    for (size_t i = 0; i < N; i++) {
      motors_[i].init();
    }
    resetOdom();
  }

  void reset() {
    updateTargetWheelsSpeed(0, 0);
    for (size_t i = 0; i < N; i++) {
      motors_[i].reset();
    }
    resetOdom();
  }

  float getSpeed() { return speed_; }
  float getAngularSpeed() { return angular_speed_; }

  float getAngle() { return angle_; }

  float getQuaternionZ() { return quaternion_Z_; }
  float getQuaternionW() { return quaternion_W_; }

  float getPositionX() { return position_X_; }
  float getPositionY() { return position_Y_; }

  float getBatteryVoltage() { return battery::getVoltage(); }
  float getBatteryCurrent() { return battery::getCurrent(); }
  float getBatteryPercentage() { return battery::getPercentage(); }

  float getTemperature() { return TemperatureSensor::getTemperature(); }

private:
  float motors_targets_[N] = {0, 0};

  Motor motors_[N] = {
      Motor(pins::left_motor),
      Motor(pins::right_motor),
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
