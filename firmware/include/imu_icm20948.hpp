#pragma once

#include "imu.hpp"
#include "types.hpp"

extern "C" {
#include "pico-icm20948.h"
}

// may have to swap back and forth between this address, a pin was
// unfortunately left floating
#define ICM20948_I2CADDR_DEFAULT 0x69
// #define ICM20948_I2CADDR_DEFAULT 0x68

#define PICO_I2C_SDA_PIN 2
#define PICO_I2C_SCL_PIN 3

class ICM20948 : IMU {
 public:
  ICM20948();
  bool setup();
  bool get_gyro_accel(float32_t& gx, float32_t& gy, float32_t& gz,
                      float32_t& ax, float32_t& ay, float32_t& az);

 private:
  i2c_inst_t icm20948_i2c_;
  icm20948_config_t config_;
  int16_t accel_raw_[3];
  int16_t gyro_raw_[3];
};