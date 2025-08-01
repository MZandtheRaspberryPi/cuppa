#pragma once

#include "types.hpp"

const uint8_t N_TIMES = 20;

class Timer {
 public:
  Timer(uint64_t loop_time_us);
  bool is_ready();
  void set_run_time();
  uint16_t get_actual_loop_rate();

 private:
  uint64_t loop_time_us_;
  uint64_t last_run_time_us_from_boot_;
  uint32_t last_n_idx_ = 0;
  uint64_t last_n_run_times_[N_TIMES];
};