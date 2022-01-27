#pragma once

#include "common.hpp"

/* PID-controller which limited in differential (d_max), integral (i_max)
 * and proportional (p_max) functional parts.
 * It also smoothly reduces the integral component error in idle.
 */
class Controller {
public:
  Controller(float dt_, float p_max_, float i_max_, float d_max_, float k_p_,
             float k_i_, float k_d_)
      : dt(dt_), p_max(p_max_), i_max(i_max_), d_max(d_max_), k_p(k_p_),
        k_i(k_i_), k_d(k_d_) {
    reset();
  }

  // finding acceleration tgt_A and distance tgt_S from target speed tgt_V
  void updateTgt(float tgt_V, float xS) {
    tgt_A = (tgt_V - tgt_V_past) / dt;
    tgt_V_past = tgt_V;

    if (tgt_V < config::fade_cap && tgt_V > -config::fade_cap)
      tgt_S += tgt_V * dt + (xS - tgt_S) * config::fade;
    else
      tgt_S += tgt_V * dt;

    // INTEGRAL LIMITATION
    if (tgt_S > xS + i_max)
      tgt_S = xS + i_max;
    if (tgt_S < xS - i_max)
      tgt_S = xS - i_max;
  }

  // main PID function
  void updateRes(float xS, float xV, float xA) {
    past_result = result;
    result = k_p * (tgt_V - xV) + k_i * (tgt_S - xS) + k_d * (tgt_A - xA);

    // DIFFERENTIAL LIMITATION
    if (result > past_result + d_max)
      result = past_result + d_max;
    if (result < past_result - d_max)
      result = past_result - d_max;

    // PROPORTIONAL LIMITATION
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
  float dt;

  float p_max; // maximum output value
  float i_max; // maximum difference between target and output distance
  float d_max; // maximum difference between the output value and the past
  float k_p;   // proportional component weight
  float k_i;   // integral component weight
  float k_d;   // differencial component weight

  float tgt_S; // target distanse
  float tgt_V; // target speed
  float tgt_A; // target acceleration
  float tgt_V_past;

  float result;
  float past_result;
};
