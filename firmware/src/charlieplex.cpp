#include "charlieplex.hpp"

#include "pico/stdlib.h"

void set_pin_high_impedence(const uint8_t& pin_num) {
  gpio_set_dir(pin_num, false);
  gpio_set_pulls(pin_num, 0, 0);
  gpio_set_input_enabled(pin_num, false);
}

void set_pin_output(const uint8_t& pin_num, const bool& dir) {
  gpio_set_dir(pin_num, true);
  gpio_put(pin_num, dir);
  gpio_set_drive_strength(pin_num,
                          gpio_drive_strength::GPIO_DRIVE_STRENGTH_12MA);
}

LEDViz::LEDViz() {
  cur_on_led_idx_ = 0;

  for (uint8_t i = 0; i < NUM_ROWS; i++) {
    set_pin_high_impedence(LED_PINS[i]);
  }

  on_leds_row_cols_.resize(0, 2);
}

void LEDViz::update_display(const DisplayMatrix& mat) {
  // we could cut run time with a bit more storage here if needed by having
  // on_leds be 255 long with an enum or something and doing one loop over
  // display matrix and storing into on leds and updating index, then set last
  // one to unset at end of loop
  uint8_t tmp = cur_on_led_idx_;
  cur_on_led_idx_ = 0;
  for (uint8_t i = 0; i < NUM_ROWS; i++) {
    for (uint8_t j = 0; j < NUM_COLS; j++) {
      cur_on_led_idx_ += mat(i, j);
    }
  }

  set_pin_high_impedence(on_leds_row_cols_(tmp, 0));
  set_pin_high_impedence(on_leds_row_cols_(tmp, 1));
  on_leds_row_cols_.resize(cur_on_led_idx_, 2);
  cur_on_led_idx_ = 0;

  for (uint8_t i = 0; i < NUM_ROWS; i++) {
    for (uint8_t j = 0; j < NUM_COLS; j++) {
      if (mat(i, j)) {
        on_leds_row_cols_(cur_on_led_idx_, 0) = ARR_HIGH_PIN_MAP[i][j];
        on_leds_row_cols_(cur_on_led_idx_, 1) = ARR_LOW_PIN_MAP[i][j];
        cur_on_led_idx_ += 1;
      }
    }
  }
  cur_on_led_idx_ = tmp < on_leds_row_cols_.rows() ? tmp : 0;
  set_pin_output(on_leds_row_cols_(cur_on_led_idx_, 0), true);
  set_pin_output(on_leds_row_cols_(cur_on_led_idx_, 1), false);
}

void LEDViz::tick() {
  if (on_leds_row_cols_.rows() < 2) {
    return;
  }

  set_pin_high_impedence(on_leds_row_cols_(cur_on_led_idx_, 0));
  set_pin_high_impedence(on_leds_row_cols_(cur_on_led_idx_, 1));

  if (on_leds_row_cols_.rows() > 1) {
    cur_on_led_idx_ = cur_on_led_idx_ == on_leds_row_cols_.rows() - 1
                          ? 0
                          : cur_on_led_idx_ + 1;
  }

  set_pin_output(on_leds_row_cols_(cur_on_led_idx_, 0), true);
  set_pin_output(on_leds_row_cols_(cur_on_led_idx_, 1), false);
}
