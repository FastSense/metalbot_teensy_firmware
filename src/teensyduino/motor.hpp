#pragma once

#include "common.hpp"
#include "filter.hpp"
#include "pid.hpp"
#include <Encoder.h>

class Motor {
public:
  Motor(MotorPins pins)
      : pins_(pins), encoder_(pins.encA, pins.encB),
        deadzone_(config::deadzone),
        pid_(config::dt, config::p_max, config::i_max, config::d_max,
             config::kp, config::ki, config::kd),
        kalman_(config::dt, config::model_noise, config::measurement_noise),
        ipr_(config::ipr), pi_(config::pi),
        wheel_diameter_(config::wheel_diameter), k_pwm_(config::k_pwm){};

  void setSpeed(int speed) {
    if (speed * active_ > deadzone_) {
      analogWrite(pins_.pwm, speed);
      digitalWrite(pins_.front, HIGH);
      digitalWrite(pins_.back, LOW);

    } else if (speed * active_ < -deadzone_) {
      analogWrite(pins_.pwm, -speed);
      digitalWrite(pins_.front, LOW);
      digitalWrite(pins_.back, HIGH);

    } else {
      analogWrite(pins_.pwm, 0);
      digitalWrite(pins_.front, LOW);
      digitalWrite(pins_.back, LOW);
    }
  }

  int getTickCount() { return encoder_.read(); }

  void updateSensor() {
    kalman_.update(getTickCount() * pi_ * wheel_diameter_ / ipr_);
  }

  float getX(size_t n) { return kalman_.getX(n); }

  void resetTick() { encoder_.write(0); }

  void updateSpeed(double target) {
    pid_.updateTgt(target, getX(KF_distance));
    updateSensor();
    pid_.updateRes(getX(KF_distance), getX(KF_speed), getX(KF_acceleration));
    setSpeed(pid_.getRes() * k_pwm_);
  }

  void reset() {
    setSpeed(0);
    kalman_.reset();
    resetTick();
    pid_.reset();
  }

  void stop() {
    reset();
    active_ = false;
  }

  void activate() {
    reset();
    active_ = true;
  }

  void init() {
    pinMode(pins_.pwm, OUTPUT);
    pinMode(pins_.front, OUTPUT);
    pinMode(pins_.back, OUTPUT);
    analogWriteFrequency(pins_.pwm, config::pwm_frequency);
    stop();
  }

private:
  MotorPins pins_;
  Encoder encoder_;
  float deadzone_;
  Regulator pid_;
  Filter kalman_;
  int ipr_;
  float pi_;
  float wheel_diameter_;
  float k_pwm_;
  bool active_ = false;
};
