#pragma once

#include "cstdint"

#define INPUT1_PIN 8
#define INPUT2_PIN 9

const uint32_t DEBOUNCE_MS = 300;

class Button {
 public:
  Button(uint8_t pin);
  bool is_pressed();

 private:
  uint32_t last_press_time_ms_;
  uint8_t pin_;
};