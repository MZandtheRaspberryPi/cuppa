#pragma once

#include "types.hpp"

union FloatUnion {
  float32_t f;
  uint8_t fBuff[sizeof(float32_t)];
};

void get_z_rotation_matrix(const float32_t& euler_z_radians,
                           RotationMatrix& rot_out);

void get_y_rotation_matrix(const float32_t& euler_y_rads,
                           RotationMatrix& rot_out);

void get_x_rotation_matrix(const float32_t& euler_x_rads,
                           RotationMatrix& rot_out);

void wrap_angle(float32_t& x);

void normalize_angles(StateVector& state);

void float_to_bytes(const float32_t& flt, uint8_t* buffer);

void subtract_angles(const float32_t& x, const float32_t& y, float32_t& out);