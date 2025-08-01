#pragma once

#include "types.hpp"

#define PIN_SDA 5
#define PIN_SCL 4
#define I2C_PORT i2c0
#define ADDR 0x5E

class Magnetometer {
 public:
  Magnetometer() {}
  virtual void setup() = 0;
  virtual bool get_measurement(MeasurementVector& meas) = 0;
};

class TLV493D : public Magnetometer {
 public:
  TLV493D();
  void setup();
  bool get_measurement(MeasurementVector& meas);

 private:
  uint8_t read_buffer_[10];
};