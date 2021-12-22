#pragma once

#include "Common.hpp"

class Regulator {
public:
  Regulator(float limit_, float kp_, float ki_, float kd_, float dt_)
      : limit(limit_), kp(kp_), ki(ki_), kd(kd_), dt(dt_) {
    reset();
  }

  void updateTgt(float tgt_V, float xS) {
    tgt_A = (tgt_V - tgt_V_past) / dt;
    // tgt_S += tgt_V * dt + (xS - tgt_S) * config::fade;
    tgt_S += tgt_V * dt; // DEBUG

    tgt_V_past = tgt_V;
  }

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
  float limit;
  float kp;
  float ki;
  float kd;
  float dt;

  float tgt_S;
  float tgt_V;
  float tgt_A;
  float tgt_V_past;

  float result;
};
