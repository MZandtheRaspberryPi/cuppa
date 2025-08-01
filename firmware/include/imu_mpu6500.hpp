#pragma once

#include "imu.hpp"
#include "types.hpp"

class MPU6500 : IMU {
 public:
  MPU6500();
  bool setup();
  bool get_gyro_accel(float32_t& gx, float32_t& gy, float32_t& gz,
                      float32_t& ax, float32_t& ay, float32_t& az);

 private:
  float32_t g_[3];
  float32_t dps_[3];
};