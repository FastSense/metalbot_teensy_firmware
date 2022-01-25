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
  sensors.begin(); // запуск интерфеса
  sensors.getAddress(tempDeviceAddress, 0); // получение адреса нулевого датчика
  sensors.setResolution(tempDeviceAddress,
                        precision); // установка низшего разрешения для
                                    // обеспечения скорости чтения < 100мс
  sensors.setWaitForConversion(false); // makes it async
  sensors.requestTemperatures();       // Send the command to get temperatures
}

float getTemperature() {
  float temp = sensors.getTempCByIndex(0);
  sensors.requestTemperatures(); // Send the command to get temperatures
  return temp;
}

} // namespace TemperatureSensor
