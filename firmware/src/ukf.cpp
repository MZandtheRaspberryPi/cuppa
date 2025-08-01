#include "ukf.hpp"

#include <Eigen/Cholesky>

#include "utils.hpp"

void do_unscented_transform(const StateVector& state,
                            const StateCovarianceMatrix& cov,
                            SigmaPoints& sigma_points_out,
                            SigmaWeights& sigma_weights_out) {
  sigma_points_out(0, Eigen::placeholders::all) = state;
  sigma_weights_out(0) = LAMBDA / (LAMBDA + N_STATE);

  float32_t fixed_weight = 1 / (2 * (LAMBDA + N_STATE));

  StateVector new_state = StateVector::Zero();
  StateCovarianceMatrix cov_helper = (LAMBDA + N_STATE) * cov;
  Eigen::LLT<StateCovarianceMatrix, Eigen::UpLoType::Lower> lltOfA(
      cov_helper);  // compute the Cholesky decomposition of cov
  StateCovarianceMatrix L =
      lltOfA.matrixL();  // retrieve factor L  in the decomposition

  // printf("L\n");
  // printf("%f, %f, %f\n", L(0, 0), L(0, 1), L(0, 2));
  // printf("%f, %f, %f\n", L(1, 0), L(1, 1), L(1, 2));
  // printf("%f, %f, %f\n", L(2, 0), L(2, 1), L(2, 2));

  uint8_t cur_idx = 1;
  for (uint8_t i = 0; i < N_STATE; i++) {
    // new_state = state + L(Eigen::placeholders::all, i);

    // here angles are involved so we have to handle subtraction//addition
    // specially. 2 degrees - 361 degrees = 3 degrees
    for (uint8_t j = 0; j < N_STATE; j++) {
      subtract_angles(state(j), -L(j, i), new_state(j));
    }

    sigma_points_out(cur_idx, Eigen::placeholders::all) = new_state.transpose();
    sigma_weights_out(cur_idx) = fixed_weight;
    // here angles are involved so we have to handle subtraction//addition
    // specially. 2 degrees - 361 degrees = 3 degrees
    for (uint8_t j = 0; j < N_STATE; j++) {
      subtract_angles(state(j), L(j, i), new_state(j));
    }

    // new_state = state - L(Eigen::placeholders::all, i);
    sigma_points_out(cur_idx + 1, Eigen::placeholders::all) =
        new_state.transpose();
    sigma_weights_out(cur_idx + 1) = fixed_weight;
    cur_idx += 2;
  }
}

void do_inverse_unscented_transform(const SigmaPoints& sigma_points,
                                    const SigmaWeights& sigma_weights,
                                    StateVector& state,
                                    StateCovarianceMatrix& cov) {
  Eigen::Matrix<float32_t, 3, 2> state_helper =
      Eigen::Matrix<float32_t, 3, 2>::Zero();

  // we do mean differently with angles because they are non linear and can't be
  // summed
  for (uint8_t i = 0; i < N_SIGMA_PTS; i++) {
    state_helper(0, 0) += sin(sigma_points(i, 0)) * sigma_weights(i);
    state_helper(0, 1) += cos(sigma_points(i, 0)) * sigma_weights(i);
    state_helper(1, 0) += sin(sigma_points(i, 1)) * sigma_weights(i);
    state_helper(1, 1) += cos(sigma_points(i, 1)) * sigma_weights(i);
    state_helper(2, 0) += sin(sigma_points(i, 2)) * sigma_weights(i);
    state_helper(2, 1) += cos(sigma_points(i, 2)) * sigma_weights(i);
  }

  cov(Eigen::placeholders::all, Eigen::placeholders::all) =
      StateCovarianceMatrix::Zero();
  for (uint8_t i = 0; i < N_SIGMA_PTS; i++) {
    StateVector sigma_pt =
        sigma_points(i, Eigen::placeholders::all).transpose();
    StateVector xi_min_mu;

    // here angles are involved so we have to handle subtraction specially. 2
    // degrees - 361 degrees = 3 degrees
    for (uint8_t j = 0; j < N_STATE; j++) {
      subtract_angles(sigma_pt(j), state(j), xi_min_mu(j));
    }

    // printf("sigma pt: %f, %f, %f, diff_from_state_est: %f, %f, %f\n",
    //        sigma_pt(0), sigma_pt(1), sigma_pt(2), xi_min_mu(0), xi_min_mu(1),
    //        xi_min_mu(2));

    cov(Eigen::placeholders::all, Eigen::placeholders::all) +=
        sigma_weights(i) * (xi_min_mu * xi_min_mu.transpose());
  }
  state.block<N_STATE, 1>(0, 0) = StateVector::Zero();
  state(0) = atan2(state_helper(0, 0), state_helper(0, 1));
  state(1) = atan2(state_helper(1, 0), state_helper(1, 1));
  state(2) = atan2(state_helper(2, 0), state_helper(2, 1));
}

UKF::UKF(const StateCovarianceMatrix& Q, const MeasurementCovarianceMatrix& R,
         const float32_t& dt)
    : Estimator(Q, R, dt) {}

bool UKF::predict(StateVector& state, StateCovarianceMatrix& cov,
                  const ControlVector& ctrl) {
  do_unscented_transform(state, cov, sigma_points_, sigma_weights_);

  SigmaPoints new_sigma_pts = SigmaPoints::Zero();
  for (uint8_t i = 0; i < N_SIGMA_PTS; i++) {
    StateVector tmp = StateVector::Zero();
    simple_dynamics(sigma_points_(i, Eigen::placeholders::all), tmp, A_, B_,
                    ctrl);
    new_sigma_pts(i, Eigen::placeholders::all) = tmp;

    // printf("sigma pt: %f, %f, %f, sigma_post_dynamics: %f, %f, %f\n",
    //        sigma_points_(i, 0), sigma_points_(i, 1), sigma_points_(i, 2),
    //        new_sigma_pts(i, 0), new_sigma_pts(i, 1), new_sigma_pts(i, 2));
  }

  do_inverse_unscented_transform(new_sigma_pts, sigma_weights_, state, cov);
  cov += Q_;
  // normalize_angles(state);
  return true;
}

bool UKF::update_accel(const MeasurementVector& measurement, StateVector& state,
                       StateCovarianceMatrix& cov) {
  do_unscented_transform(state, cov, sigma_points_, sigma_weights_);
  get_predicted_measurement_accel(sigma_points_, predicted_measurements_);
  update_state_cov(measurement, predicted_measurements_, sigma_points_,
                   sigma_weights_, state, cov);
  return true;
}

// bool UKF::update_gyro(const MeasurementVector& measurement, StateVector&
// state,
//                       StateCovarianceMatrix& cov) {
//   do_unscented_transform(state, cov, sigma_points_, sigma_weights_);
//   get_predicted_measurement_accel(sigma_points_, predicted_measurements_);
//   update_state_cov(measurement, predicted_measurements_, sigma_points_,
//                    sigma_weights_, state, cov);
//   return true;
// }

void UKF::get_predicted_measurement_accel(
    const SigmaPoints& sigma_points,
    PredictedSigmaMeasurements& predicted_measurements) {
  predicted_measurements = PredictedSigmaMeasurements::Zero();
  MeasurementVector tmp = MeasurementVector::Zero();
  for (uint8_t i = 0; i < N_SIGMA_PTS; i++) {
    predict_accelerometer_measurement(sigma_points(i, Eigen::placeholders::all),
                                      tmp);
    predicted_measurements(i, Eigen::placeholders::all) = tmp;
    // printf("sigma pt: %f, %f, %f, predicted measurement: %f, %f, %f\n",
    //        sigma_points(i, 0), sigma_points(i, 1), sigma_points(i, 2),
    //        tmp(0), tmp(1), tmp(2));
  }
}

// void UKF::get_predicted_measurement_gyro(
//     const SigmaPoints& sigma_points,
//     PredictedSigmaMeasurements& predicted_measurements) {
//   predicted_measurements = PredictedSigmaMeasurements::Zero();
//   MeasurementVector tmp = MeasurementVector::Zero();
//   for (uint8_t i = 0; i < N_SIGMA_PTS; i++) {
//     predict_gyro_measurement(sigma_points(i, Eigen::placeholders::all), tmp);
//     predicted_measurements(i, Eigen::placeholders::all) = tmp;
//   }
// }

void UKF::update_state_cov(
    const MeasurementVector& measurement,
    const PredictedSigmaMeasurements& predicted_measurements,
    const SigmaPoints& sigma_points, const SigmaWeights& sigma_weights,
    StateVector& state, StateCovarianceMatrix& cov) {
  MeasurementVector expected_measurement = MeasurementVector::Zero();
  for (uint8_t i = 0; i < N_SIGMA_PTS; i++) {
    expected_measurement +=
        sigma_weights(i) *
        predicted_measurements(i, Eigen::placeholders::all).transpose();
  }

  sigma_y_ = MeasurementCovarianceMatrix::Zero();
  MeasurementVector meas_diff;
  for (uint8_t i = 0; i < N_SIGMA_PTS; i++) {
    meas_diff =
        predicted_measurements(i, Eigen::placeholders::all).transpose() -
        expected_measurement;
    sigma_y_ += sigma_weights(i) * meas_diff * meas_diff.transpose();
  }

  sigma_y_ += R_;

  // printf("Sigma Y\n");
  // printf("%f, %f, %f\n", sigma_y_(0, 0), sigma_y_(0, 1), sigma_y_(0, 2));
  // printf("%f, %f, %f\n", sigma_y_(1, 0), sigma_y_(1, 1), sigma_y_(1, 2));
  // printf("%f, %f, %f\n", sigma_y_(2, 0), sigma_y_(2, 1), sigma_y_(2, 2));

  sigma_xy_ = CrossCovMatrix::Zero();
  StateVector state_diff;
  for (uint8_t i = 0; i < N_SIGMA_PTS; i++) {
    // here angles are involved so we have to handle subtraction specially. 2
    // degrees - 361 degrees = 3 degrees
    for (uint8_t j = 0; j < N_STATE; j++) {
      subtract_angles(sigma_points(i, j), state(j), state_diff(j));
    }
    meas_diff =
        predicted_measurements(i, Eigen::placeholders::all).transpose() -
        expected_measurement;
    sigma_xy_ += sigma_weights(i) * state_diff * meas_diff.transpose();
  }

  // printf("Sigma XY\n");
  // printf("%f, %f, %f\n", sigma_xy_(0, 0), sigma_xy_(0, 1), sigma_xy_(0, 2));
  // printf("%f, %f, %f\n", sigma_xy_(1, 0), sigma_xy_(1, 1), sigma_xy_(1, 2));
  // printf("%f, %f, %f\n", sigma_xy_(2, 0), sigma_xy_(2, 1), sigma_xy_(2, 2));

  // printf("measurement was %f, %f, %f, expected measurement was %f %f %f\n\n",
  //        measurement(0), measurement(1), measurement(2),
  //        expected_measurement(0), expected_measurement(1),
  //        expected_measurement(2));

  state = state + (sigma_xy_ * sigma_y_.inverse()) *
                      (measurement - expected_measurement);
  // cov = cov - (sigma_xy_ * sigma_y_.inverse()) * sigma_xy_.transpose();
  cov = cov - (sigma_xy_ * sigma_y_.inverse()) *
                  (sigma_y_ * (sigma_xy_ * sigma_y_.inverse()).transpose());

  normalize_angles(state);
}