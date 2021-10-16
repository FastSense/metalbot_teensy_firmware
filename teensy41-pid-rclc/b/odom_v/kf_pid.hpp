#ifndef KF_PID_485367532
#define KF_PID_485367532

class Regulator {

  float kp;
  float ki;
  float kd;
  float limit;

  float dt = 0.01;

  float tgt_V = 0;
  float tgt_V_past = 0;
  float tgt_S = 0;
  float tgt_A = 0;

  float result = 0;

public:
  Regulator(float limit_ = 1, float kp_ = 0.01, float ki_ = 0.00,
            float kd_ = 0.00)
      : limit(limit_), kp(kp_), ki(ki_), kd(kd_) {}

  void updateTgt(float tgt_V) { // Twist_msg -> i_motor: tgt_V -> tgt_A, tgt_S..
    tgt_A = (tgt_V - tgt_V_past) / dt;
    tgt_S += tgt_V * dt;
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
};

#endif /* end of include guard: KF_PID_485367532 */
