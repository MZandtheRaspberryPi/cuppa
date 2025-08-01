#pragma once

#include <cstdint>

// we have orientation as euler angles, 3 dimensional
// then we have angular velocity, 3 dimensional
const uint8_t N_STATE = 3;
const uint8_t N_CTRL = 3;
const uint8_t ORIENTATION_IDX = 0;
const uint8_t N_MEASUREMENT = 3;