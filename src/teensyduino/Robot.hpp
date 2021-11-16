#ifndef ROBOT_247502943
#define ROBOT_247502943

#include "Common.hpp"
#include "Motor.hpp"

class Robot_t {
public:
  void start() {
    for (size_t i = 0; i < 4; i++) {
      motors_[i].start();
    }
  }
  void reset() {
    for (size_t i = 0; i < 4; i++) {
      motors_[i].reset();
    }
  }

  void updateTargetWheelsSpeed(double linear, double angular) {
    motors_targets_[0] = motors_targets_[2] =
        linear - angular * base_width_half_;
    motors_targets_[1] = motors_targets_[3] =
        linear + angular * base_width_half_;
  }

  void updateSpeedRegulation() {
    for (size_t i = 0; i < 4; i++)
      motors_[i].updateSpeed(motors_targets_[i]);
  }
  void updateOdometry() {

    speed_ = (motors_[0].getX(1) + motors_[1].getX(1) + motors_[2].getX(1) +
              motors_[3].getX(1)) /
             4;

    angular_speed_ = (motors_[1].getX(1) - motors_[0].getX(1) +
                      motors_[3].getX(1) - motors_[2].getX(1)) /
                     base_width_ / 4;

    angle_ = (motors_[1].getX(0) - motors_[0].getX(0) + motors_[3].getX(0) -
              motors_[2].getX(0)) /
             base_width_ / 4;

    /*test calc of X and Y. In final it must to be calculated from kalman[0]!*/
    position_X_ += getSpeed() * PID_DT / 1000 * cos(getAngle());
    position_Y_ += getSpeed() * PID_DT / 1000 * sin(getAngle());

    quaternion_Z_ = sin(getAngle() / 2);
    quaternion_W_ = cos(getAngle() / 2);
  }

  void hardStopLoop() { /// when connection lost
    while (1) {
      for (size_t i = 0; i < 4; i++)
        motors_[i].setSpeed(0);
      delay(10);
    }
  }

  float getSpeed() { return speed_; }
  float getAngularSpeed() { return angular_speed_; }

  float getAngle() { return angle_; }

  float getQuaternionZ() { return quaternion_Z_; }
  float getQuaternionW() { return quaternion_W_; }

  float getPositionX() { return position_X_; }
  float getPositionY() { return position_Y_; }

private:
  float motors_targets_[4] = {0, 0, 0, 0};

  Motor motors_[4] = {
      /* pwm, front, back, encA, encB */
      // /* metal: */
      // Motor({8, 10, 9, 1, 0}),   // A
      // Motor({11, 13, 12, 2, 3}), // B
      // Motor({14, 18, 15, 5, 4}), // C
      // Motor({19, 23, 22, 6, 7})  // D
      /* fanera: */
      Motor({14, 15, 18, 5, 4}), // A
      Motor({19, 22, 23, 6, 7}), // B
      Motor({8, 10, 9, 3, 2}),   // C
      Motor({11, 13, 12, 0, 1}), // D
  };

  const float base_width_ = 0.44;
  const float base_width_half_ = base_width_ / 2;
  float speed_ = 0;
  float angular_speed_ = 0;
  float position_X_ = 0;
  float position_Y_ = 0;
  float angle_ = 0;
  float quaternion_Z_ = 0;
  float quaternion_W_ = 1;
};

#endif /* end of include guard: ROBOT_247502943 */
