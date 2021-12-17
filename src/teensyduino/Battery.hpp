#pragma once

#include <Wire.h>
#include <infinityPV_INA233.h>

namespace battery {

INA233 IC1(0x40);

constexpr float R_shunt = 0.002;
constexpr float I_max = 20;
constexpr float min_voltage = 18.0;
constexpr float max_voltage = 24.0;

void start() {
  uint16_t CAL = 0;
  int16_t m_c = 0;
  int16_t m_p = 0;
  int8_t R_c = 0;
  int8_t R_p = 0;
  uint8_t Set_ERROR = 0;
  float Current_LSB = 0;
  float Power_LSB = 0;
  // uint16_t Read_CAL = 0;

  IC1.begin();
  CAL = IC1.setCalibration(R_shunt, I_max, &Current_LSB, &Power_LSB, &m_c, &R_c,
                           &m_p, &R_p, &Set_ERROR);
}

float getVoltage() { return IC1.getBusVoltage_V(); }

float getCurrent() { return IC1.getCurrent_mA() / 1000; }

float getPercentage() {
  return (getVoltage() - min_voltage) / (max_voltage - min_voltage);
}

} // namespace battery
