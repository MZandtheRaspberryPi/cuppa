#pragma once

#include <Eigen/Dense>
#include <cstdint>

#include "constants.hpp"

typedef float float32_t;

typedef Eigen::Matrix<float32_t, N_STATE, 1> StateVector;
typedef Eigen::Matrix<float32_t, N_STATE, N_STATE> StateCovarianceMatrix;
typedef Eigen::Matrix<float32_t, N_MEASUREMENT, 1> MeasurementVector;
typedef Eigen::Matrix<float32_t, N_CTRL, 1> ControlVector;
typedef Eigen::Matrix<float32_t, N_MEASUREMENT, N_MEASUREMENT>
    MeasurementCovarianceMatrix;
typedef Eigen::Matrix<float32_t, 3, 3> RotationMatrix;
