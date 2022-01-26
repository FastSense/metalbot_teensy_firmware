#pragma once

#include "common.hpp"
#include <BasicLinearAlgebra.h>

// Kalman filter with constant-acceleration motion model. Filters the wheel path
class Filter {
public:
  Filter(float dt, float model_noise, float measurement_noise)
      : dt_(dt), model_noise_(model_noise),
        measurement_noise_(measurement_noise) {
    reset();
    matE = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    matH = {1, 0, 0};
    matF = {1, dt, 0.5 * dt * dt, 0, 1, dt, 0, 0, 1};
    matQ = {0, 0, 0, 0, 0, 0, 0, 0, model_noise_ * model_noise_};
  };

  void reset() {
    matZ(0) = 0;
    matY(0) = 0;
    matS(0) = 0;
    matK = {0, 0, 0};
    matX = {0, 0, 0};
    matP = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  }

  void update(float z) {
    matZ(0) = z;
    matX = matF * matX;
    matP = matF * (matP + matQ) * (~matF);
    matY = matZ + ((~matH * matX) * (-1));
    matS = ~matH * matP * matH + measurement_noise_;
    matK = matP * matH * BLA::Invert(matS);
    matX = matX + matK * matY;
    matP = (matE - matK * ~matH) * matP;
  };

  // method to get one of the filter output values:
  // distance, speed, acceleration (kalmanX enum)
  float getX(size_t n) { return matX(n); };

private:
  float dt_;
  float model_noise_;
  float measurement_noise_;
  BLA::Matrix<1> matZ;    // input value (wheel path)
  BLA::Matrix<1> matY;    // predicted state
  BLA::Matrix<1> matS;    // system uncertainty
  BLA::Matrix<3> matK;    // kalman gain
  BLA::Matrix<3> matH;    // transposed observation
  BLA::Matrix<3> matX;    // estimated system state
  BLA::Matrix<3, 3> matE; // identity matrix
  BLA::Matrix<3, 3> matP; // covariance matrix
  BLA::Matrix<3, 3> matF; // state transition matrix
  BLA::Matrix<3, 3> matQ; // process noise matrix
};
