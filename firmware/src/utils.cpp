#include "utils.hpp"

#include <cmath>

void get_z_rotation_matrix(const float32_t& euler_z_radians,
                           RotationMatrix& rot_out) {
  rot_out = RotationMatrix::Identity();
  rot_out(0, 0) = cos(euler_z_radians);
  rot_out(0, 1) = -sin(euler_z_radians);
  rot_out(1, 0) = sin(euler_z_radians);
  rot_out(1, 1) = cos(euler_z_radians);
}

void get_y_rotation_matrix(const float32_t& euler_y_rads,
                           RotationMatrix& rot_out) {
  rot_out = RotationMatrix::Identity();
  rot_out(0, 0) = cos(euler_y_rads);
  rot_out(0, 2) = sin(euler_y_rads);
  rot_out(2, 0) = -sin(euler_y_rads);
  rot_out(2, 2) = cos(euler_y_rads);
}

void get_x_rotation_matrix(const float32_t& euler_x_rads,
                           RotationMatrix& rot_out) {
  rot_out = RotationMatrix::Identity();
  rot_out(1, 1) = cos(euler_x_rads);
  rot_out(1, 2) = -sin(euler_x_rads);
  rot_out(2, 1) = sin(euler_x_rads);
  rot_out(2, 2) = cos(euler_x_rads);
}

void wrap_angle(float32_t& x) { x = atan2(sin(x), cos(x)); }

void normalize_angles(StateVector& state) {
  wrap_angle(state(ORIENTATION_IDX));
  wrap_angle(state(ORIENTATION_IDX + 1));
  wrap_angle(state(ORIENTATION_IDX + 2));
  // interestingly the x rotation of pi and y rotation of negative pi give
  // gravity as expected, but in reality gravity is inversed
  // we nuke it here to get the solution closer to what we want
  if (state(ORIENTATION_IDX + 1) < -3.0) {
    state(ORIENTATION_IDX + 1) = 0.0;
  }
  if (state(ORIENTATION_IDX) > 3.0) {
    state(ORIENTATION_IDX) = 0.0;
  }
}

void float_to_bytes(const float32_t& flt, uint8_t* buffer) {
  FloatUnion flt_union;
  flt_union.f = flt;
  // pi pico is little endian by default
  buffer[0] = flt_union.fBuff[0];
  buffer[1] = flt_union.fBuff[1];
  buffer[2] = flt_union.fBuff[2];
  buffer[3] = flt_union.fBuff[3];
}

void subtract_angles(const float32_t& x, const float32_t& y, float32_t& out) {
  // here angles are involved so we have to handle subtraction specially. 2
  // degrees - 361 degrees = 3 degrees
  out = x - y;
  if (out > M_PI) {
    out = out - 2 * M_PI;
  }
  if (out < -M_PI) {
    out = out + 2 * M_PI;
  }
  2 * M_PI;
}