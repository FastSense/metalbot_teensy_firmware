// Temperature measurement module

#pragma once

#include "common.hpp"
#include <DallasTemperature.h>
#include <OneWire.h>

namespace {
constexpr uint8_t precision = 9;
OneWire oneWire(pins::temperature);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;
} // namespace

namespace TemperatureSensor {

void start() {
  // interface launch:
  sensors.begin();
  sensors.getAddress(tempDeviceAddress, 0);

  // setting low resolution for data processing period < 100ms:
  sensors.setResolution(tempDeviceAddress, precision);

  // makes it asynchronous:
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
}

float getTemperature() {
  float temp = sensors.getTempCByIndex(0);
  sensors.requestTemperatures();
  return temp;
}

} // namespace TemperatureSensor
