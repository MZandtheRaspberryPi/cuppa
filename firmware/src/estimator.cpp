#include "estimator.hpp"

#include "utils.hpp"

Estimator::Estimator(const StateCovarianceMatrix& Q,
                     const MeasurementCovarianceMatrix& R,
                     const float32_t& dt) {
  gravity_ << 0, 0, 9.81;
  Q_ = Q;
  R_ = R;
  dt_ = dt;
  A_ = Eigen::Matrix<float32_t, N_STATE, N_STATE>::Identity();
  B_ = Eigen::Matrix<float32_t, N_STATE, N_STATE>::Identity();
  B_ *= dt;
}

void simple_dynamics(const StateVector& initial_state, StateVector& state_out,
                     const Eigen::Matrix<float32_t, N_STATE, N_STATE>& A,
                     const Eigen::Matrix<float32_t, N_STATE, N_STATE>& B,
                     const ControlVector& ctrl) {
  state_out = A * initial_state + B * ctrl;
}

void Estimator::predict_accelerometer_measurement(
    const StateVector& state, MeasurementVector& measurement_out) {
  // doing rotation with first z, then y, then x. This is because
  // with accelerometer we have two degrees of freedom. Our yaw estimate
  // doesn't impact gravity vector. Therefore in this order, the Z rotation is
  // doesn't impact gravity, which helps us with the filter.

  // interestingly the x rotation of pi and y rotation of negative pi give
  // gravity as expected, but in reality gravity is inversed

  RotationMatrix R_y;
  RotationMatrix R_x;

  get_y_rotation_matrix(state(ORIENTATION_IDX + 1), R_y);
  get_x_rotation_matrix(state(ORIENTATION_IDX), R_x);
  measurement_out = R_x.transpose() * (R_y.transpose() * gravity_);
}

// void Estimator::predict_gyro_measurement(const StateVector& state,
//                                          MeasurementVector& measurement_out)
//                                          {
//   measurement_out = state.block<N_MEASUREMENT, 1>(ANGULAR_VEL_IDX, 0);
// }