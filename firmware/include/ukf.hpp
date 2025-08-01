#pragma once
/*
See: Julier, Simon J., and Jeffrey K. Uhlmann. "New extension of the Kalman
filter to nonlinear systems." Signal processing, sensor fusion, and target
recognition VI. Vol. 3068. Spie, 1997.
*/

#include "estimator.hpp"
#include "utils.hpp"

const float32_t LAMBDA = 0.0;

const uint8_t N_SIGMA_PTS = 1 + 2 * N_STATE;

typedef Eigen::Matrix<float32_t, N_SIGMA_PTS, N_STATE> SigmaPoints;
typedef Eigen::Matrix<float32_t, N_SIGMA_PTS, 1> SigmaWeights;
typedef Eigen::Matrix<float32_t, N_STATE, N_MEASUREMENT> CrossCovMatrix;
typedef Eigen::Matrix<float32_t, N_SIGMA_PTS, N_MEASUREMENT>
    PredictedSigmaMeasurements;

void do_unscented_transform(const StateVector& state,
                            const StateCovarianceMatrix& cov,
                            SigmaPoints& sigma_points_out,
                            SigmaWeights& sigma_weights_out);
void do_inverse_unscented_transform(const SigmaPoints& sigma_points,
                                    const SigmaWeights& sigma_weights,
                                    StateVector& state,
                                    StateCovarianceMatrix& cov);

class UKF : protected Estimator {
 public:
  UKF(const StateCovarianceMatrix& Q, const MeasurementCovarianceMatrix& R,
      const float32_t& dt);
  bool predict(StateVector& state, StateCovarianceMatrix& cov,
               const ControlVector& ctrl);
  bool update_accel(const MeasurementVector& measurement, StateVector& state,
                    StateCovarianceMatrix& cov);
  //   bool update_gyro(const MeasurementVector& measurement, StateVector&
  //   state,
  //                    StateCovarianceMatrix& cov);

 private:
  void update_state_cov(
      const MeasurementVector& measurement,
      const PredictedSigmaMeasurements& predicted_measurements,
      const SigmaPoints& sigma_points, const SigmaWeights& sigma_weights,
      StateVector& state, StateCovarianceMatrix& cov);
  //   void get_predicted_measurement_gyro(
  //       const SigmaPoints& sigma_points,
  //       PredictedSigmaMeasurements& predicted_measurements);
  void get_predicted_measurement_accel(
      const SigmaPoints& sigma_points,
      PredictedSigmaMeasurements& predicted_measurements);
  SigmaPoints sigma_points_;
  SigmaWeights sigma_weights_;
  PredictedSigmaMeasurements predicted_measurements_;
  MeasurementCovarianceMatrix sigma_y_;
  CrossCovMatrix sigma_xy_;
};