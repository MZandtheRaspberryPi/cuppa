#include "imu_icm20948.hpp"

#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

ICM20948::ICM20948() {}

bool ICM20948::setup() {
  icm20948_i2c_ = {i2c1_hw, false};
  config_ = {ICM20948_I2CADDR_DEFAULT, 0x0C, &icm20948_i2c_};
  i2c_init(&icm20948_i2c_, 400 * 1000);  // 400kHz
  gpio_set_function(PICO_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_I2C_SCL_PIN, GPIO_FUNC_I2C);

  uint8_t rx_data[1] = {0};

  uint8_t cur_try = 0;
  uint8_t max_tries = 5;
  while (cur_try < max_tries &&
         i2c_read_blocking(&icm20948_i2c_, ICM20948_I2CADDR_DEFAULT, rx_data, 1,
                           false) < 0) {
    printf("error in init imu");
    sleep_ms(1000);
    cur_try += 1;
  }

  if (cur_try == max_tries) {
    return false;
  }

  uint8_t ret = icm20948_init(&config_);
  return ret == 0;
}

bool ICM20948::get_gyro_accel(float32_t& gx, float32_t& gy, float32_t& gz,
                              float32_t& ax, float32_t& ay, float32_t& az) {
  icm20948_read_raw_accel(&config_, accel_raw_);
  icm20948_read_raw_gyro(&config_, gyro_raw_);
  // the sensor coord system is different from our model system
  // y is positive x, and x is negative y in right handed system
  gx = (float32_t)gyro_raw_[1] / 131.0f;
  gx *= M_PI / 180;  // to radians per second, from degrees per second
  gy = -(float32_t)gyro_raw_[0] / 131.0f;
  gy *= M_PI / 180;  // to radians per second, from degrees per second
  gz = (float32_t)gyro_raw_[2] / 131.0f;
  gz *= M_PI / 180;  // to radians per second, from degrees per second

  ax = (float32_t)accel_raw_[1] / 16384.0f;
  ax *= 9.81;  // to m/s^2, as opposed to gravities
  ay = -(float32_t)accel_raw_[0] / 16384.0f;
  ay *= 9.81;  // to m/s^2, as opposed to gravities
  az = (float32_t)accel_raw_[2] / 16384.0f;
  az *= 9.81;  // to m/s^2, as opposed to gravities
  return true;
}
