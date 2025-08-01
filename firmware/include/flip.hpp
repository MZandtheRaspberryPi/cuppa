#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "charlieplex.hpp"
#include "types.hpp"

enum CellType { FLUID_CELL = 0, AIR_CELL = 1, SOLID_CELL = 2 };

// for small display
// constexpr uint32_t DISP_WIDTH = NUM_COLS;
// constexpr uint32_t DISP_HEIGHT = NUM_ROWS;
// constexpr float32_t DISP_SPACING = 0.02;
// constexpr float32_t REL_WATER_HEIGHT = 0.9;
// constexpr float32_t REL_WATER_WIDTH = 0.9;

// for large display
constexpr uint32_t DISP_WIDTH = 15;
constexpr uint32_t DISP_HEIGHT = 16;
constexpr float32_t DISP_SPACING = 0.02;
constexpr float32_t REL_WATER_HEIGHT = 0.5;
constexpr float32_t REL_WATER_WIDTH = 0.5;

// below when we cast to integer we floor

// for all displays
constexpr float32_t SIM_HEIGHT = DISP_HEIGHT * DISP_SPACING + DISP_SPACING;
constexpr float32_t SIM_WIDTH = DISP_WIDTH * DISP_SPACING + DISP_SPACING;
constexpr float32_t H = DISP_SPACING;
constexpr float32_t DENSITY = 1000.0;
constexpr float32_t R = H * 0.3;

constexpr uint32_t CELL_NUM_X = SIM_WIDTH / H + 1 + 0.0001;
constexpr uint32_t CELL_NUM_Y = SIM_HEIGHT / H + 1 + 0.0001;
constexpr uint32_t CELL_NUM = CELL_NUM_X * CELL_NUM_Y;

// for particles
constexpr float32_t DX = 2.0 * R;
// 1.732050808 is sqrt(3)
constexpr float32_t DY = 1.732050808 / 2.0 * DX;
constexpr uint32_t PART_NUMX =
    ((float32_t)REL_WATER_WIDTH * SIM_WIDTH - 2.0 * H - 2.0 * R) / DX + 0.0001;
constexpr uint32_t PART_NUMY =
    ((float32_t)REL_WATER_HEIGHT * SIM_HEIGHT - 2.0 * H - 2.0 * R) / DY +
    0.0001;
constexpr uint32_t MAX_PARTICLES = PART_NUMX * PART_NUMY;

constexpr float32_t PINV_SPACING = 1.0 / (2.2 * R);
constexpr uint32_t P_NUMX = (SIM_WIDTH * PINV_SPACING + 0.0001) + 1;
constexpr uint32_t P_NUMY = (SIM_HEIGHT * PINV_SPACING + 0.0001) + 1;
constexpr uint32_t P_NUMCELLS = P_NUMX * P_NUMY;

float32_t clamp(const float32_t x, const float32_t min_x,
                const float32_t max_x);
int32_t clamp(const int32_t x, const int32_t min_x, const int32_t max_x);

class FlipSim {
 public:
  FlipSim();

  void step(const float32_t& dt, const Eigen::Matrix<float32_t, 2, 1>& gravity,
            const float32_t& flip_ratio, const uint32_t& num_pressure_iters,
            const uint32_t& num_particle_iters, const float32_t& overrelaxation,
            const bool& compensate_drift, const bool& separate_particles);

  void get_display_mat(DisplayMatrix& mat);
  void zero_arrays();

 private:
  void create_particles();
  void setup_gridcells();

  void integrate_particles(const float32_t& dt,
                           const Eigen::Matrix<float32_t, 2, 1>& gravity);

  void push_particles_apart(const uint32_t& num_iters);

  void handle_particle_collisions();

  void transfer_velocities(const bool& to_grid, const float32_t& flip_ratio);

  void update_particle_density();

  void solve_incompressibility(const uint32_t& num_iters, const float32_t& dt,
                               const float32_t& overrelaxation,
                               const bool& compensate_drift = true);

  float32_t h_;
  float32_t h2_;
  float32_t f_inv_spacing_;

  // cells
  Eigen::Matrix<float32_t, CELL_NUM, 1> u_;
  Eigen::Matrix<float32_t, CELL_NUM, 1> v_;
  Eigen::Matrix<float32_t, CELL_NUM, 1> du_;
  Eigen::Matrix<float32_t, CELL_NUM, 1> dv_;
  Eigen::Matrix<float32_t, CELL_NUM, 1> prev_u_;
  Eigen::Matrix<float32_t, CELL_NUM, 1> prev_v_;
  Eigen::Matrix<float32_t, CELL_NUM, 1> p_;
  Eigen::Matrix<float32_t, CELL_NUM, 1> s_;
  Eigen::Matrix<CellType, CELL_NUM, 1> cell_type_;

  // particles
  Eigen::Matrix<float32_t, 2 * MAX_PARTICLES, 1> particle_pos_;
  Eigen::Matrix<float32_t, 2 * MAX_PARTICLES, 1> particle_vel_;
  Eigen::Matrix<float32_t, CELL_NUM, 1> particle_density_;
  float32_t particle_rest_density_;

  float32_t p_inv_spacing_;

  Eigen::Matrix<int32_t, P_NUMCELLS, 1> num_cell_particles_;
  Eigen::Matrix<int32_t, P_NUMCELLS + 1, 1> first_cell_particles_;
  Eigen::Matrix<int32_t, MAX_PARTICLES, 1> cell_particle_ids_;
};