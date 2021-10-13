#ifndef REGULATOR_3502472090341
#define REGULATOR_3502472090341
#define IELS 3
class Regulator {

  float limit_;
  float kp_;
  float ki_;
  float kd_;
  float error_ = 0;
  float past_error_ = 0;
  float error_summary_ = 0;
  float result_ = 0;

public:
  Regulator(float limit = 0, float kp = 0, float ki = 0, float kd = 0)
      : limit_(limit), kp_(kp), ki_(ki), kd_(kd) {}

  float update(float error = 0, float dts = 0.02) {
    past_error_ = error_;
    error_ = error;
    if (error_summary_ < limit_ && error_summary_ > -limit_)
      error_summary_ += error_;
    if (error_summary_ > limit_)
      error_summary_ = limit_ - IELS;
    if (error_summary_ < -limit_)
      error_summary_ = -limit_ + IELS;
    result_ = kp_ * error_ + ki_ * error_summary_ +
              kd_ * (error_ - past_error_) / dts;
    return result_;
  }

  float getResult() { return result_; }

  void reset() {
    error_ = 0;
    past_error_ = 0;
    error_summary_ = 0;
    result_ = 0;
  }
};

#endif /* end of include guard: REGULATOR_3502472090341 */
