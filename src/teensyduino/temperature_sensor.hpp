#pragma once
#include <DallasTemperature.h>
#include <OneWire.h>

namespace {
constexpr uint8_t pin = 26;
constexpr uint8_t precision = 9;
OneWire oneWire(pin);
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
}

float getTemperature() {
  sensors.requestTemperatures(); // Send the command to get temperatures
  return sensors.getTempC(tempDeviceAddress);
}

} // namespace TemperatureSensor
