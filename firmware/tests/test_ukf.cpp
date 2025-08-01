#include <gtest/gtest.h>
#include <stdio.h>

#include <cmath>

#include "constants.hpp"
#include "types.hpp"
#include "ukf.hpp"
#include "utils.hpp"

class UKFTest : public testing::Test
{
protected:
  UKFTest()
      : dt(0.1),
        state(StateVector::Zero()),
        cov(StateCovarianceMatrix::Identity()),
        ukf(StateCovarianceMatrix::Identity(),
            MeasurementCovarianceMatrix::Identity(), 0.1) {}
  UKF ukf;
  StateVector state;
  StateCovarianceMatrix cov;
  float32_t dt;
};

TEST_F(UKFTest, IsPredictCorrect)
{
  // lets say we are rotating around x at pi / 4 radians per second
  ControlVector ctrl;
  ctrl(0) = M_PI / 4;

  // we will do 1 second of time steps
  float32_t cur_time = 0.0;

  while (cur_time < 1.0)
  {
    ukf.predict(state, cov, ctrl);
    cur_time += dt;
  }

  EXPECT_NEAR(state(ORIENTATION_IDX), M_PI / 4, 0.01);
}

TEST_F(UKFTest, IsUpdateCorrectAccel)
{
  state(ORIENTATION_IDX) = M_PI / 6;
  float32_t x_rot = M_PI / 4;
  float32_t y_rot = 0.0;
  float32_t z_rot = 0.0;

  MeasurementVector gravity;
  MeasurementVector measurement;
  gravity << 0, 0, 9.81;

  RotationMatrix R;

  get_y_rotation_matrix(y_rot, R);
  measurement = R.transpose() * gravity;
  // std::cout << "measurement y rot" << std::endl;
  // std::cout << measurement << std::endl;
  get_x_rotation_matrix(x_rot, R);
  measurement = R.transpose() * measurement;
  // std::cout << "measurement x rot" << std::endl;
  // std::cout << measurement << std::endl;

  float32_t cur_time = 0.0;

  while (cur_time < 1.0)
  {
    // std::cout << "state: " << std::endl;
    // std::cout << state << std::endl;
    // std::cout << "cov: " << std::endl;
    // std::cout << cov << std::endl;
    // ukf.predict(state, cov);
    ukf.update_accel(measurement, state, cov);
    cur_time += dt;
  }

  EXPECT_NEAR(state(ORIENTATION_IDX), M_PI / 4, 0.02);
}