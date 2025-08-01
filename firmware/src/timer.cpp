#include "timer.hpp"

#include "pico/stdlib.h"
// #include "pico/time.h"

Timer::Timer(uint64_t loop_time_us) : loop_time_us_(loop_time_us) {
  last_run_time_us_from_boot_ = 0;
}

bool Timer::is_ready() {
  uint64_t us_since_boot = to_us_since_boot(get_absolute_time());
  return us_since_boot - last_run_time_us_from_boot_ > loop_time_us_;
}

void Timer::set_run_time() {
  uint64_t tmp = to_us_since_boot(get_absolute_time());
  last_n_run_times_[last_n_idx_] = (tmp - last_run_time_us_from_boot_);
  last_n_idx_ = last_n_idx_ == N_TIMES - 1 ? 0 : last_n_idx_ + 1;
  last_run_time_us_from_boot_ = tmp;
}

uint16_t Timer::get_actual_loop_rate() {
  uint64_t loop_times = 0;
  for (uint8_t i = 0; i < N_TIMES; i++) {
    loop_times += last_n_run_times_[i];
  }
  return (float32_t)1000000 / ((float32_t)loop_times / (float32_t)N_TIMES);
}