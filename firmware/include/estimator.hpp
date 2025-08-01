#pragma once

#include "constants.hpp"
#include "types.hpp"

void simple_dynamics(const StateVector& initial_state, StateVector& state_out,
                     const Eigen::Matrix<float32_t, N_STATE, N_STATE>& A,
                     const Eigen::Matrix<float32_t, N_STATE, N_STATE>& B,
                     const ControlVector& ctrl);



class Estimator {
 public:
  Estimator(const StateCovarianceMatrix& Q,
            const MeasurementCovarianceMatrix& R, const float32_t& dt);
  virtual bool predict(StateVector& state, StateCovarianceMatrix& cov,
                       const ControlVector& ctrl) = 0;
  virtual bool update_accel(const MeasurementVector& measurement,
                            StateVector& state, StateCovarianceMatrix& cov) = 0;
  // virtual bool update_gyro(const MeasurementVector& measurement,
  //                          StateVector& state, StateCovarianceMatrix& cov) =
  //                          0;
  // this has some notes on common measurement models in section B Measurements:
  // Mahony, Robert, Tarek Hamel, and Jean-Michel Pflimlin. "Nonlinear
  // complementary filters on the special orthogonal group." IEEE Transactions
  // on automatic control 53.5 (2008): 1203-1218. this also:
  // https://stanford.edu/class/ee267/notes/ee267_notes_imu.pdf
  void predict_accelerometer_measurement(const StateVector& state,
                                         MeasurementVector& measurement_out);
  void predict_gyro_measurement(const StateVector& state,
                                MeasurementVector& measurement_out);

 protected:
  Eigen::Matrix<float32_t, N_STATE, N_STATE> A_;
  Eigen::Matrix<float32_t, N_STATE, N_STATE> B_;
  StateCovarianceMatrix Q_;
  MeasurementCovarianceMatrix R_;
  float32_t dt_;
  MeasurementVector gravity_;
};