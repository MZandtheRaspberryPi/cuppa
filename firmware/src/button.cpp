#include "button.hpp"

#include "pico/stdlib.h"

Button::Button(uint8_t pin) : pin_(pin) {
  gpio_set_input_enabled(pin_, true);
  // gpio_set_pulls(pin_, 1, 0);
  gpio_pull_up(pin_);
  // gpio_set_input_enabled(pin_, true);
  last_press_time_ms_ = 0;
}

bool Button::is_pressed() {
  uint32_t ms_since_boot = to_ms_since_boot(get_absolute_time());
  if ((ms_since_boot - last_press_time_ms_ > DEBOUNCE_MS) && !gpio_get(pin_)) {
    last_press_time_ms_ = ms_since_boot;
    return true;
  }
  return false;
  return gpio_get(pin_);
}
