#include "imu_mpu6500.hpp"

#include "cmath"
#include "driver_mpu6500_basic.h"

MPU6500::MPU6500() {}

bool MPU6500::setup() {
  uint8_t ret = mpu6500_basic_init(mpu6500_interface_t::MPU6500_INTERFACE_SPI,
                                   MPU6500_ADDRESS_AD0_LOW);
  if (ret != 0) {
    return ret;
  }

  // ret = mpu6500_set_loop_rate(100);
  return ret;
}

bool MPU6500::get_gyro_accel(float32_t& gx, float32_t& gy, float32_t& gz,
                             float32_t& ax, float32_t& ay, float32_t& az) {
  int8_t ret = mpu6500_basic_read(g_, dps_);
  if (ret != 0) {
    printf("issue reading sensors");
  }
  // printf("raw gyro data: %f, %f, %f\n", dps_[0], dps_[1], dps_[2]);
  // printf("raw accel data: %f, %f, %f\n", g_[0], g_[1], g_[2]);

  bool tmp_enabled;
  uint8_t is_temp_enabled_ret = is_temp_enabled(&tmp_enabled);
  if (is_temp_enabled_ret != 0) {
    printf("can't read tmp enabled");
  }

  // printf("tmp enabled: %d\n", tmp_enabled);

  float tmp = 0.0;
  mpu6500_basic_read_temperature(&tmp);
  // printf("tmp: %f\n", tmp);

  bool is_sleep = mpu6500_is_sleep();
  // printf("slp: %d\n", is_sleep);

  uint8_t i2c_add = mpu6500_whoami();
  // printf("i2c_add: %d\n", i2c_add);

  bool g_enabl = mpu6500_is_gyro_standby();
  // printf("gyro standby enabled: %d\n", g_enabl);

  int16_t gyro_x;
  uint8_t mod_ret = modified_read_gyrox(&gyro_x);
  if (mod_ret != 0) {
    printf("issue reading modified sensors\n");
  }
  // printf("gyro x manual: %d\n", gyro_x);

  // the sensor coord system is different from our model system
  // y is positive x, and x is negative y in right handed system
  gx = dps_[1];
  gx *= M_PI / 180;  // to radians per second, from degrees per second
  gy = -dps_[0];
  gy *= M_PI / 180;  // to radians per second, from degrees per second
  gz = dps_[2];
  gz *= M_PI / 180;  // to radians per second, from degrees per second

  ax = g_[1];
  ax *= 9.81;  // to m/s^2, as opposed to gravities
  ay = -g_[0];
  ay *= 9.81;  // to m/s^2, as opposed to gravities
  az = g_[2];
  az *= 9.81;  // to m/s^2, as opposed to gravities
  return ret;
}