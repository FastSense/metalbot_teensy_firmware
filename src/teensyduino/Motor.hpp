#ifndef MOTOR_97634894045734
#define MOTOR_97634894045734
#include "Common.hpp"
#include "Filter.hpp"
#include "Pid.hpp"
#include <Encoder.h>

class Motor {

  const int IPR_ = 1836;
  const float pi_ = 3.141593;
  const float wheel_diameter_ = 0.195; // 0.133
  const float k_pwm = 200;

  MotorPins pins_;
  Encoder encoder_;
  float deadzone_;
  int tick_count_;
  int past_tick_count_;
  float tick_speed_;

public:
  Regulator pid;   // __DBG!!!!!
  Filter_t kalman; //__DBG!!!!
  Motor(MotorPins pins, float deadzone = 0, float limit = 1, float kp = 0.2,
        float ki = 2.5, float kd = 0.0015, float dt = 0.01,
        float model_noise = 25.0, float measurement_noise = 0.001)
      : pins_(pins), encoder_(pins.encA, pins.encB), deadzone_(deadzone),
        pid(limit, kp, ki, kd), kalman(dt, model_noise, measurement_noise){};

  void start() {
    pinMode(pins_.pwm, OUTPUT);
    // analogWriteFrequency(pins_.pwm, 585937.5);
    analogWriteFrequency(pins_.pwm, 36621); // __DBG
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
  //вызывать периодически для корректной работы фильтра:
  void updateSensor() {
    kalman.update(getTickCount() * pi_ * wheel_diameter_ / IPR_);
  }

  float getX(size_t n) { return kalman.getX(n); }

  int getTickCount() { return encoder_.read(); }

  void resetTick() { encoder_.write(0); }

  void updateSpeed(double target) {
    pid.updateTgt(target, getX(0));
    updateSensor();
    pid.updateRes(getX(0), getX(1), getX(2));
    setSpeed(pid.getRes() * k_pwm);
  }
};

#endif /* end of include guard: MOTOR_97634894045734 */
