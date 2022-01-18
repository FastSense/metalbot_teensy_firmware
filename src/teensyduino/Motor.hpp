#pragma once

#include "Common.hpp"
#include "Filter.hpp"
#include "Pid.hpp"
#include <Encoder.h>

class Motor {
public:
  /*TODO: init obj-s from main*/
  Motor(MotorPins pins,
        //
        float deadzone = 1,
        //
        float i_max = 0.15, float p_max = 0.3, float d_max = 0.12,
        //
        float kp = 0.015, float ki = 2.1, float kd = 0.005,
        //
        float dt = 0.01,
        //
        float model_noise = 25.0, float measurement_noise = 0.001)
      : pins_(pins), encoder_(pins.encA, pins.encB), deadzone_(deadzone),
        pid(p_max, i_max, d_max, kp, ki, kd, dt),
        kalman(dt, model_noise, measurement_noise), ipr_(config::ipr),
        pi_(config::pi), wheel_diameter_(config::wheel_diameter),
        k_pwm_(config::k_pwm){};

  void init() {
    pinMode(pins_.pwm, OUTPUT);
    pinMode(pins_.front, OUTPUT);
    pinMode(pins_.back, OUTPUT);
    analogWriteFrequency(pins_.pwm, config::pwm_frequency);
    stop();
  }

  void reset() {
    setSpeed(0);
    kalman.reset();
    resetTick();
    pid.reset();
  }

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
  //вызывать только периодически для корректной работы фильтра:
  void updateSensor() {
    kalman.update(getTickCount() * pi_ * wheel_diameter_ / ipr_);
  }

  float getX(size_t n) { return kalman.getX(n); }

  int getTickCount() { return encoder_.read(); }

  void resetTick() { encoder_.write(0); }

  void updateSpeed(double target) {
    pid.updateTgt(target, getX(KF_distance));
    updateSensor();
    pid.updateRes(getX(KF_distance), getX(KF_speed), getX(KF_acceleration));
    setSpeed(pid.getRes() * k_pwm_);
  }
  void stop() {
    reset();
    active_ = false;
  }

  void activate() {
    reset();
    active_ = true;
  }

private:
  MotorPins pins_;
  Encoder encoder_;
  float deadzone_;
  Regulator pid; // __// DBG
  Filter kalman; //__// DBG
  int ipr_;
  float pi_;
  float wheel_diameter_;
  float k_pwm_;
  bool active_ = false;
};
