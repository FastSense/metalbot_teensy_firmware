#ifndef KF_PID_485367532
#define KF_PID_485367532

class Regulator {

  float kp = 0.7;
  float ki = 0.3;
  float kd = 0.075;
  float result = 0;
  float dt = 0.01;
  float tgt_V = 0;
  float tgt_V_past = 0;
  float tgt_S = 0;
  float tgt_A = 0;

public:
  void updateTgt(float tgt_V) { // Twist_msg -> i_motor: tgt_V -> tgt_A, tgt_S..
    tgt_A = (tgt_V - tgt_V_past) / dt;
    tgt_S += tgt_V * dt;
    tgt_V_past = tgt_V;
  };
  void updateRes(float xS, float xV, float xA) {
    result = kp * (tgt_V - xV) + ki * (tgt_S - xS) + kd * (tgt_A - xA);
  }
  float getRes() { return result; }
};

#endif /* end of include guard: KF_PID_485367532 */
