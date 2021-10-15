#ifndef FILTER_4534532
#define FILTER_4534532
#include <BasicLinearAlgebra.h>

class Filter_t {

  float dt_;
  float model_noise_;
  float measurement_noise_;
  BLA::Matrix<1> matZ;
  BLA::Matrix<1> matY;
  BLA::Matrix<1> matS;
  BLA::Matrix<3> matK;
  BLA::Matrix<3> matH;
  BLA::Matrix<3> matX;
  BLA::Matrix<3, 3> matE;
  BLA::Matrix<3, 3> matP;
  BLA::Matrix<3, 3> matF;
  BLA::Matrix<3, 3> matQ;

public:
  Filter_t(float dt = 0.01, float model_noise = 300,
           float measurement_noise = 2)
      : dt_(dt), model_noise_(model_noise),
        measurement_noise_(measurement_noise) {

    matE = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    matZ(0) = 0;
    matY(0) = 0;
    matS(0) = 0;
    matK = {0, 0, 0};
    matH = {1, 0, 0};
    matX = {0, 0, 0};

    matP = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    matF = {1, dt, 0.5 * dt * dt, 0, 1, dt, 0, 0, 1};
    matQ = {0, 0, 0, 0, 0, 0, 0, 0, model_noise_ * model_noise_};
  };

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

  float getX(size_t n) { return matX(n); }; //
};

#endif /* end of include guard: FILTER_4534532 */
