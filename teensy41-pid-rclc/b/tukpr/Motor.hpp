#ifndef MOTOR_97634894045734
#define MOTOR_97634894045734
#include "Regulator.hpp"
#include <Encoder.h>

struct MotorPins {
  uint8_t pwm, front, back, encA, encB;
};

class Motor {

  MotorPins pins_;
  Encoder encoder_;
  int deadzone_;
  int tick_count_;
  int past_tick_count_;
  float tick_speed_;

public:
  Regulator pider; // __DBG!!!!!

  Motor(MotorPins pins, int deadzone = 2, float limit = 300, float kp = 0.9,
        float ki = 0.8, float kd = 0.02)
      : pins_(pins), encoder_(pins.encA, pins.encB), deadzone_(deadzone),
        pider(limit, kp, ki, kd){
            // Serial.println(("M")); //DBG
        };

  void start() {
    pinMode(pins_.pwm, OUTPUT);
    // analogWriteFrequency(pins_.pwm, 585937.5);
    analogWriteFrequency(pins_.pwm, 36621);
    pinMode(pins_.front, OUTPUT);
    pinMode(pins_.back, OUTPUT);
    reset();
  }

  void reset() {
    setSpeed(0);
    resetTick();
    pider.reset();
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

  float getSpeed(size_t dt) {
    tick_count_ = encoder_.read();
    tick_speed_ = (tick_count_ - past_tick_count_) / (float)dt;
    past_tick_count_ = tick_count_;
    return tick_speed_;
  }

  int getTickCount() { return encoder_.read(); }

  void resetTick() { encoder_.write(0); }
};

#endif /* end of include guard: MOTOR_97634894045734 */
