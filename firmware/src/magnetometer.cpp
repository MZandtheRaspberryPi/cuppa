#include "magnetometer.hpp"

#include "hardware/i2c.h"
#include "pico/stdlib.h"

TLV493D::TLV493D() {}

void TLV493D::setup() {
  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
  gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
  sleep_ms(10);

  read_buffer_[0] = 0x00;

  // this sensor does weird stuff on reset to change address
  i2c_write_blocking(I2C_PORT, ADDR, read_buffer_, 1, true);
  i2c_write_blocking(I2C_PORT, ADDR, read_buffer_, 1, false);

  // now we need to read the sensor bitmap

  i2c_write_blocking(I2C_PORT, ADDR, read_buffer_, 1, true);
  i2c_read_blocking(I2C_PORT, ADDR, read_buffer_, 10, false);
  printf("got bytes from sensor");
}

bool TLV493D::get_measurement(MeasurementVector& meas) {
  for (uint8_t i = 0; i < 9; i++) {
    printf("%d\n", read_buffer_[i]);
  }
}
//      void setup();
//      bool get_measurement(MeasurementVector& meas);
//    };