#pragma once

#include "Common.hpp"
#include "Filter.hpp"
#include "Pid.hpp"
#include <Encoder.h>

class Motor {
public:
  /*TODO: init obj-s from main*/
  Motor(MotorPins pins, float deadzone = 2, float limit = 20, float kp = 0.01,
        float ki = 2.35, float kd = 0.005, float dt = 0.01,
        float model_noise = 25.0, float measurement_noise = 0.001)
      : pins_(pins), encoder_(pins.encA, pins.encB), deadzone_(deadzone),
        pid(limit, kp, ki, kd, dt), kalman(dt, model_noise, measurement_noise),
        ipr_(config::ipr), pi_(config::pi),
        wheel_diameter_(config::wheel_diameter), k_pwm_(config::k_pwm){};

  void start() {
    pinMode(pins_.pwm, OUTPUT);
    analogWriteFrequency(pins_.pwm, config::pwm_frequency);
    pinMode(pins_.front, OUTPUT);
    pinMode(pins_.back, OUTPUT);
    reset();
  }

  void reset() {
    setSpeed(0);
    resetTick();
    pid.reset();
  }

  void setSpeed(int speed) {
    if (speed > deadzone_) {
      analogWrite(pins_.pwm, speed);
      digitalWrite(pins_.front, HIGH);
      digitalWrite(pins_.back, LOW);

    } else if (speed < -deadzone_) {
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

private:
  MotorPins pins_;
  Encoder encoder_;
  float deadzone_;
  Regulator pid; // __DBG
  Filter kalman; //__DBG
  int ipr_;
  float pi_;
  float wheel_diameter_;
  float k_pwm_;
};
