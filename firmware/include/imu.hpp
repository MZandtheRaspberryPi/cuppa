#pragma once

#include "types.hpp"

class IMU {
 public:
  IMU() {};
  virtual bool setup() = 0;
  virtual bool get_gyro_accel(float32_t& gx, float32_t& gy, float32_t& gz,
                              float32_t& ax, float32_t& ay, float32_t& az) = 0;
  //   virtual bool get_mag(float32_t& mx, float32_t& my, float32_t& mz);
};