#pragma once

#include "Common.hpp"

class Regulator {
public:
  Regulator(float p_max_, float i_max_, float d_max_, float k_p_, float k_i_,
            float k_d_, float dt_)
      : p_max(p_max_), i_max(i_max_), d_max(d_max_), k_p(k_p_), k_i(k_i_),
        k_d(k_d_), dt(dt_) {
    reset();
  }

  void updateTgt(float tgt_V, float xS) {
    tgt_A = (tgt_V - tgt_V_past) / dt;
    tgt_V_past = tgt_V;

    // tgt_S += tgt_V * dt + (xS - tgt_S) * config::fade;
    tgt_S += tgt_V * dt; // DBG

    // LIMITATION INTEGRAL
    if (tgt_S > xS + i_max)
      tgt_S = xS + i_max;
    if (tgt_S < xS - i_max)
      tgt_S = xS - i_max;
  }

  void updateRes(float xS, float xV, float xA) {
    past_result = result;
    result = k_p * (tgt_V - xV) + k_i * (tgt_S - xS) + k_d * (tgt_A - xA);

    // LIMITATION DIFFERENTIAL
    if (result > past_result + d_max)
      result = past_result + d_max;
    if (result < past_result - d_max)
      result = past_result - d_max;

    // LIMITATION PROPORTIONAL
    if (result > p_max)
      result = p_max;
    if (result < -p_max)
      result = -p_max;
  }

  float getRes() { return result; }

  void reset() {
    tgt_S = 0;
    tgt_V = 0;
    tgt_A = 0;
    tgt_V_past = 0;
    result = 0;
    past_result = 0;
  }

private:
  float p_max;
  float i_max;
  float d_max;
  float k_p;
  float k_i;
  float k_d;
  float dt;

  float tgt_S;
  float tgt_V;
  float tgt_A;
  float tgt_V_past;

  float result;
  float past_result;
};
