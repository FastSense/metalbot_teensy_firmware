#pragma once

#include "Common.hpp"

class Regulator {
public:
  Regulator(float limit_ = 0, float kp_ = 0.0, float ki_ = 0.0, float kd_ = 0.0)
      : limit(limit_), kp(kp_), ki(ki_), kd(kd_) {
    reset();
  }

  void updateTgt(float tgt_V, float xS) {
    tgt_A = (tgt_V - tgt_V_past) / dt;
    tgt_S += tgt_V * dt + (xS - tgt_S) * config::fade; // add config::fade
    tgt_V_past = tgt_V;
  };
  void updateRes(float xS, float xV, float xA) {
    result = kp * (tgt_V - xV) + ki * (tgt_S - xS) + kd * (tgt_A - xA);
  }
  float getRes() { return result; }

  void reset() {
    tgt_S = 0;
    tgt_V = 0;
    tgt_A = 0;
    tgt_V_past = 0;
    result = 0;
  }

private:
  float kp;
  float ki;
  float kd;
  float limit;

  float dt = 0.01f;

  float tgt_S;
  float tgt_V;
  float tgt_A;
  float tgt_V_past;

  float result;
};
