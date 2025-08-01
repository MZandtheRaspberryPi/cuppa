#include "flip.hpp"

#include "pico/stdlib.h"

float32_t clamp(const float32_t x, const float32_t min_x,
                const float32_t max_x) {
  if (x < min_x) {
    return min_x;
  } else if (x > max_x) {
    return max_x;
  } else {
    return x;
  }
}

int32_t clamp(const int32_t x, const int32_t min_x, const int32_t max_x) {
  if (x < min_x) {
    return min_x;
  } else if (x > max_x) {
    return max_x;
  } else {
    return x;
  }
}

void FlipSim::zero_arrays() {
  h_ = std::max(SIM_WIDTH / CELL_NUM_X, SIM_HEIGHT / CELL_NUM_Y);
  h2_ = 0.5 * h_;
  f_inv_spacing_ = 1 / h_;
  u_ = Eigen::Matrix<float32_t, CELL_NUM, 1>::Zero();
  v_ = Eigen::Matrix<float32_t, CELL_NUM, 1>::Zero();
  du_ = Eigen::Matrix<float32_t, CELL_NUM, 1>::Zero();
  dv_ = Eigen::Matrix<float32_t, CELL_NUM, 1>::Zero();
  prev_u_ = Eigen::Matrix<float32_t, CELL_NUM, 1>::Zero();
  prev_v_ = Eigen::Matrix<float32_t, CELL_NUM, 1>::Zero();
  p_ = Eigen::Matrix<float32_t, CELL_NUM, 1>::Zero();
  s_ = Eigen::Matrix<float32_t, CELL_NUM, 1>::Zero();

  particle_pos_ = Eigen::Matrix<float32_t, 2 * MAX_PARTICLES, 1>::Zero();
  particle_vel_ = Eigen::Matrix<float32_t, 2 * MAX_PARTICLES, 1>::Zero();
  particle_density_ = Eigen::Matrix<float32_t, CELL_NUM, 1>::Zero();
  particle_rest_density_ = 0.0;

  p_inv_spacing_ = 1 / R;

  num_cell_particles_ = Eigen::Matrix<int32_t, P_NUMCELLS, 1>::Zero();
  first_cell_particles_ = Eigen::Matrix<int32_t, P_NUMCELLS + 1, 1>::Zero();
  cell_particle_ids_ = Eigen::Matrix<int32_t, MAX_PARTICLES, 1>::Zero();
}

FlipSim::FlipSim() {
  zero_arrays();
  create_particles();
  setup_gridcells();
}

void FlipSim::create_particles() {
  uint32_t p = 0;
  float32_t xp_diff = 0.0;
  for (uint32_t i = 0; i < PART_NUMX; i++) {
    for (uint32_t j = 0; j < PART_NUMY; j++) {
      if (j % 2 == 0) {
        xp_diff = 0.0;
      } else {
        xp_diff = R;
      }

      particle_pos_[p] = h_ + R + DX * i + xp_diff;
      particle_pos_[p + 1] = h_ + R + DY * j;
      p += 2;
    }
  }
}

void FlipSim::setup_gridcells() {
  float32_t s = 0.0;
  for (uint32_t i = 0; i < CELL_NUM_X; i++) {
    for (uint32_t j = 0; j < CELL_NUM_Y; j++) {
      s = 1.0;
      if ((i == 0) || (i == CELL_NUM_X - 1) || (j == 0)) {
        s = 0.0;
      }
      s_[i * CELL_NUM_Y + j] = s;
    }
  }
}

void FlipSim::integrate_particles(
    const float32_t& dt, const Eigen::Matrix<float32_t, 2, 1>& gravity) {
  for (uint32_t i = 0; i < MAX_PARTICLES; i++) {
    particle_vel_(2 * i) += dt * gravity(0);
    particle_vel_(2 * i + 1) += dt * gravity(1);
    particle_pos_(2 * i) += particle_vel_(2 * i) * dt;
    particle_pos_(2 * i + 1) += particle_vel_(2 * i + 1) * dt;
  }
}

void FlipSim::push_particles_apart(const uint32_t& num_iters) {
  // count particles per cell
  num_cell_particles_ = num_cell_particles_.Zero();

  for (uint32_t i = 0; i < MAX_PARTICLES; i++) {
    const float32_t& x = particle_pos_(2 * i);
    const float32_t& y = particle_pos_(2 * i + 1);

    int32_t xi =
        clamp((int32_t)((x * p_inv_spacing_ + 0.0001)), 0, (int32_t)P_NUMX - 1);
    int32_t yi =
        clamp((int32_t)((y * p_inv_spacing_) + 0.0001), 0, (int32_t)P_NUMY - 1);

    uint32_t cell_nr = xi * P_NUMY + yi;
    num_cell_particles_(cell_nr) += 1;
  }

  // partial sums
  uint32_t first = 0;

  for (uint32_t i = 0; i < P_NUMCELLS; i++) {
    first += num_cell_particles_(i);
    first_cell_particles_(i) = first;
  }

  first_cell_particles_(P_NUMCELLS) = first;  // guard

  // fill particles into cells

  for (uint32_t i = 0; i < MAX_PARTICLES; i++) {
    const float32_t& x = particle_pos_(2 * i);
    const float32_t& y = particle_pos_(2 * i + 1);

    int32_t xi =
        clamp((int32_t)(x * p_inv_spacing_ + 0.0001), 0, (int32_t)P_NUMX - 1);
    int32_t yi =
        clamp((int32_t)(y * p_inv_spacing_ + 0.0001), 0, (int32_t)P_NUMY - 1);
    uint32_t cell_nr = xi * P_NUMY + yi;

    first_cell_particles_(cell_nr) -= 1;
    cell_particle_ids_(first_cell_particles_(cell_nr)) = i;
  }

  // push particles apart

  float32_t min_dist = 2.0 * R;
  float32_t min_dist_2 = min_dist * min_dist;

  for (uint32_t it = 0; it < num_iters; it++) {
    for (uint32_t i = 0; i < MAX_PARTICLES; i++) {
      const float32_t& px = particle_pos_(2 * i);
      const float32_t& py = particle_pos_(2 * i + 1);

      int32_t pxi = px * p_inv_spacing_ + 0.0001;
      int32_t pyi = py * p_inv_spacing_ + 0.0001;

      int32_t x0 = std::max(pxi - 1, 0L);
      int32_t y0 = std::max(pyi - 1, 0L);
      int32_t x1 = std::min(pxi + 1, (int32_t)(P_NUMX - 1));
      int32_t y1 = std::min(pyi + 1, (int32_t)(P_NUMY - 1));

      for (uint32_t xi = x0; xi <= x1; xi++) {
        for (uint32_t yi = y0; yi <= y1; yi++) {
          uint32_t cell_nr = xi * P_NUMY + yi;
          uint32_t first = first_cell_particles_(cell_nr);
          uint32_t last = first_cell_particles_(cell_nr + 1);

          for (uint32_t j = first; j < last; j++) {
            uint32_t p_id = cell_particle_ids_(j);
            if (p_id == i) {
              continue;
            }

            const float32_t qx = particle_pos_(2 * p_id);
            const float32_t qy = particle_pos_(2 * p_id + 1);

            float32_t dx = qx - px;
            float32_t dy = qy - py;
            float32_t d2 = dx * dx + dy * dy;
            if (d2 > min_dist_2 || d2 == 0.0) {
              continue;
            }

            float32_t d = sqrt(d2);
            float32_t s = 0.5 * (min_dist - d) / d;
            dx *= s;
            dy *= s;
            particle_pos_(2 * i) -= dx;
            particle_pos_(2 * i + 1) -= dy;
            particle_pos_(2 * p_id) += dx;
            particle_pos_(2 * p_id + 1) += dy;
          }
        }
      }
    }
  }
}

void FlipSim::handle_particle_collisions() {
  float32_t min_x = h_ + R;
  float32_t max_x = (CELL_NUM_X - 1) * h_ - R;
  float32_t min_y = h_ + R;
  float32_t max_y = (CELL_NUM_Y - 1) * h_ - R;

  for (uint32_t i = 0; i < MAX_PARTICLES; i++) {
    float32_t& x = particle_pos_(2 * i);
    float32_t& y = particle_pos_(2 * i + 1);
    // wall collisions
    if (x < min_x) {
      x = min_x;
      particle_vel_(2 * i) = 0.0;
    }
    if (x > max_x) {
      x = max_x;
      particle_vel_(2 * i) = 0.0;
    }

    if (y < min_y) {
      y = min_y;
      particle_vel_(2 * i + 1) = 0.0;
    }

    if (y > max_y) {
      y = max_y;
      particle_vel_(2 * i + 1) = 0.0;
    }
  }
}

void FlipSim::transfer_velocities(const bool& to_grid,
                                  const float32_t& flip_ratio) {
  if (to_grid) {
    prev_u_(Eigen::placeholders::all, Eigen::placeholders::all) =
        u_(Eigen::placeholders::all, Eigen::placeholders::all);
    prev_v_(Eigen::placeholders::all, Eigen::placeholders::all) =
        v_(Eigen::placeholders::all, Eigen::placeholders::all);
    du_ = du_.Zero();
    dv_ = dv_.Zero();
    u_ = u_.Zero();
    v_ = v_.Zero();

    for (uint32_t i = 0; i < CELL_NUM; i++) {
      if (s_(i) == 0.0) {
        cell_type_(i) = CellType::SOLID_CELL;
      } else {
        cell_type_(i) = CellType::AIR_CELL;
      }
    }

    for (uint32_t i = 0; i < MAX_PARTICLES; i++) {
      const float32_t& x = particle_pos_(2 * i);
      const float32_t& y = particle_pos_(2 * i + 1);

      int32_t xi = clamp((int32_t)(x * f_inv_spacing_ + 0.0001), 0,
                         (int32_t)CELL_NUM_X - 1);
      int32_t yi = clamp((int32_t)(y * f_inv_spacing_ + 0.0001), 0,
                         (int32_t)CELL_NUM_Y - 1);
      uint32_t cell_nr = xi * CELL_NUM_Y + yi;
      if (cell_type_(cell_nr) == CellType::AIR_CELL) {
        cell_type_(cell_nr) = CellType::FLUID_CELL;
      }
    }
  }

  for (uint32_t component = 0; component < 2; component++) {
    float32_t dx;
    float32_t dy;
    Eigen::Matrix<float32_t, CELL_NUM, 1>* f_ptr;
    Eigen::Matrix<float32_t, CELL_NUM, 1>* prev_f_ptr;
    Eigen::Matrix<float32_t, CELL_NUM, 1>* d_ptr;
    if (component == 0) {
      dx = 0.0;
      dy = h2_;
      f_ptr = &u_;
      prev_f_ptr = &prev_u_;
      d_ptr = &du_;
    } else {
      dx = h2_;
      dy = 0.0;
      f_ptr = &v_;
      prev_f_ptr = &prev_v_;
      d_ptr = &dv_;
    }

    Eigen::Matrix<float32_t, CELL_NUM, 1>& f = *f_ptr;
    Eigen::Matrix<float32_t, CELL_NUM, 1>& prev_f = *prev_f_ptr;
    Eigen::Matrix<float32_t, CELL_NUM, 1>& d = *d_ptr;

    for (uint32_t i = 0; i < MAX_PARTICLES; i++) {
      float32_t x = particle_pos_(2 * i);
      float32_t y = particle_pos_(2 * i + 1);
      x = clamp(x, h_, (float32_t)(CELL_NUM_X - 1) * h_);
      y = clamp(y, h_, (float32_t)(CELL_NUM_Y - 1) * h_);

      int32_t x0 = std::min((int32_t)((x - dx) * f_inv_spacing_ + 0.0001),
                            (int32_t)CELL_NUM_X - 2);
      float32_t tx = ((x - dx) - (float32_t)x0 * h_) * f_inv_spacing_;
      int32_t x1 = std::min(x0 + 1, (int32_t)CELL_NUM_X - 2);

      int32_t y0 = std::min((int32_t)((y - dy) * f_inv_spacing_ + 0.0001),
                            (int32_t)CELL_NUM_Y - 2);
      float32_t ty = ((y - dy) - (float32_t)y0 * h_) * f_inv_spacing_;
      int32_t y1 = std::min(y0 + 1, (int32_t)CELL_NUM_Y - 2);

      float32_t sx = 1.0 - tx;
      float32_t sy = 1.0 - ty;

      float32_t d0 = sx * sy;
      float32_t d1 = tx * sy;
      float32_t d2 = tx * ty;
      float32_t d3 = sx * ty;

      int32_t nr0 = x0 * CELL_NUM_Y + y0;
      int32_t nr1 = x1 * CELL_NUM_Y + y0;
      int32_t nr2 = x1 * CELL_NUM_Y + y1;
      int32_t nr3 = x0 * CELL_NUM_Y + y1;

      if (to_grid) {
        const float32_t& pv = particle_vel_(2 * i + component);
        f(nr0) += pv * d0;
        d(nr0) += d0;
        f(nr1) += pv * d1;
        d(nr1) += d1;
        f(nr2) += pv * d2;
        d(nr2) += d2;
        f(nr3) += pv * d3;
        d(nr3) += d3;
      } else {
        int32_t offset;
        if (component == 0) {
          offset = CELL_NUM_Y;
        } else {
          offset = 1;
        }
        bool valid0 = cell_type_(nr0) == CellType::FLUID_CELL ||
                      cell_type_(nr0 - offset) == CellType::FLUID_CELL;
        bool valid1 = cell_type_(nr1) == CellType::FLUID_CELL ||
                      cell_type_(nr1 - offset) == CellType::FLUID_CELL;
        bool valid2 = cell_type_(nr2) == CellType::FLUID_CELL ||
                      cell_type_(nr2 - offset) == CellType::FLUID_CELL;
        bool valid3 = cell_type_(nr3) == CellType::FLUID_CELL ||
                      cell_type_(nr3 - offset) == CellType::FLUID_CELL;

        float32_t v = particle_vel_(2 * i + component);
        float32_t new_d = (float32_t)valid0 * d0 + (float32_t)valid1 * d1 +
                          (float32_t)valid2 * d2 + (float32_t)valid3 * d3;

        if (new_d > 0.0) {
          float32_t pic_v = ((float32_t)valid0 * d0 * f(nr0) +
                             (float32_t)valid1 * d1 * f(nr1) +
                             (float32_t)valid2 * d2 * f(nr2) +
                             (float32_t)valid3 * d3 * f(nr3)) /
                            new_d;
          float32_t corr = ((float32_t)valid0 * d0 * (f(nr0) - prev_f(nr0)) +
                            (float32_t)valid1 * d1 * (f(nr1) - prev_f(nr1)) +
                            (float32_t)valid2 * d2 * (f(nr2) - prev_f(nr2)) +
                            (float32_t)valid3 * d3 * (f(nr3) - prev_f(nr3))) /
                           new_d;
          float32_t flip_v = v + corr;
          particle_vel_(2 * i + component) =
              (1.0 - flip_ratio) * pic_v + flip_ratio * flip_v;
        }
      }
    }

    if (to_grid) {
      for (uint32_t i = 0; i < CELL_NUM; i++) {
        if (d(i) > 0.0) {
          f(i) /= d(i);
        }
      }

      // restore solid
      for (uint32_t i = 0; i < CELL_NUM_X; i++) {
        for (uint32_t j = 0; j < CELL_NUM_Y; j++) {
          bool solid = cell_type_(i * CELL_NUM_Y + j) == CellType::SOLID_CELL;
          if (solid || (i > 0 && cell_type_((i - 1) * CELL_NUM_Y + j) ==
                                     CellType::SOLID_CELL)) {
            u_(i * CELL_NUM_Y + j) = prev_u_(i * CELL_NUM_Y + j);
          }
          if (solid || (j > 0 && cell_type_(i * CELL_NUM_Y + j - 1) ==
                                     CellType::SOLID_CELL)) {
            v_(i * CELL_NUM_Y + j) = prev_v_(i * CELL_NUM_Y + j);
          }
        }
      }
    }
  }
}

void FlipSim::update_particle_density() {
  particle_density_ = particle_density_.Zero();

  for (uint32_t i = 0; i < MAX_PARTICLES; i++) {
    float32_t x = particle_pos_(2 * i);
    float32_t y = particle_pos_(2 * i + 1);
    x = clamp(x, h_, (float32_t)(CELL_NUM_X - 1) * h_);
    y = clamp(y, h_, (float32_t)(CELL_NUM_Y - 1) * h_);

    int32_t x0 = (int32_t)((x - h2_) * f_inv_spacing_ + 0.0001);
    float32_t tx = ((float32_t)(x - h2_) - (float32_t)x0 * h_) * f_inv_spacing_;
    int32_t x1 = std::min(x0 + 1, (int32_t)CELL_NUM_X - 2);

    int32_t y0 = (int32_t)((y - h2_) * f_inv_spacing_ + 0.0001);
    float32_t ty = ((float32_t)(y - h2_) - (float32_t)y0 * h_) * f_inv_spacing_;
    int32_t y1 = std::min(y0 + 1, (int32_t)CELL_NUM_Y - 2);

    float32_t sx = 1.0 - tx;
    float32_t sy = 1.0 - ty;

    if (x0 < (int32_t)CELL_NUM_X && y0 < (int32_t)CELL_NUM_Y) {
      particle_density_(x0 * CELL_NUM_Y + y0) += sx * sy;
    }

    if (x1 < (int32_t)CELL_NUM_X && y0 < (int32_t)CELL_NUM_Y) {
      particle_density_(x1 * CELL_NUM_Y + y0) += tx * sy;
    }

    if (x1 < (int32_t)CELL_NUM_X && y1 < (int32_t)CELL_NUM_Y) {
      particle_density_(x1 * CELL_NUM_Y + y1) += tx * ty;
    }

    if (x0 < (int32_t)CELL_NUM_X && y1 < (int32_t)CELL_NUM_Y) {
      particle_density_(x0 * CELL_NUM_Y + y1) += sx * ty;
    }
  }

  if (particle_rest_density_ == 0.0) {
    float32_t running_sum = 0.0;
    uint32_t num_fluid_cells = 0;
    for (uint32_t i = 0; i < CELL_NUM; i++) {
      if (cell_type_(i) == CellType::FLUID_CELL) {
        running_sum += particle_density_(i);
        num_fluid_cells += 1;
      }
    }
    if (num_fluid_cells > 0) {
      particle_rest_density_ = running_sum / num_fluid_cells;
    }
  }
}

void FlipSim::solve_incompressibility(const uint32_t& num_iters,
                                      const float32_t& dt,
                                      const float32_t& overrelaxation,
                                      const bool& compensate_drift) {
  p_ = p_.Zero();
  prev_u_ = u_;
  prev_v_ = v_;
  const float32_t cp = DENSITY * h_ / dt;

  for (uint32_t it = 0; it < num_iters; it++) {
    for (uint32_t i = 1; i < CELL_NUM_X - 1; i++) {
      for (uint32_t j = 1; j < CELL_NUM_Y - 1; j++) {
        if (cell_type_(i * CELL_NUM_Y + j) != CellType::FLUID_CELL) {
          continue;
        }

        int32_t center = i * CELL_NUM_Y + j;
        int32_t left = (i - 1) * CELL_NUM_Y + j;
        int32_t right = (i + 1) * CELL_NUM_Y + j;
        int32_t bottom = i * CELL_NUM_Y + j - 1;
        int32_t top = i * CELL_NUM_Y + j + 1;

        float32_t s = s_(center);
        float32_t sx0 = s_(left);
        float32_t sx1 = s_(right);
        float32_t sy0 = s_(bottom);
        float32_t sy1 = s_(top);

        s = sx0 + sx1 + sy0 + sy1;
        if (s == 0.0) {
          continue;
        }

        float32_t div = u_(right) - u_(center) + v_(top) - v_(center);
        if (particle_rest_density_ > 0.0 && compensate_drift) {
          float32_t k = 1.0;
          float32_t compression =
              particle_density_(i * CELL_NUM_Y + j) - particle_rest_density_;
          if (compression > 0.0) {
            div = div - k * compression;
          }
        }

        float32_t p = -div / s;
        p *= overrelaxation;

        p_(center) += cp * p;
        u_(center) -= sx0 * p;
        u_(right) += sx1 * p;
        v_(center) -= sy0 * p;
        v_(top) += sy1 * p;
      }
    }
  }
}

void FlipSim::step(
    const float32_t& dt, const Eigen::Matrix<float32_t, 2, 1>& gravity,
    const float32_t& flip_ratio, const uint32_t& num_pressure_iters,
    const uint32_t& num_particle_iters, const float32_t& overrelaxation,
    const bool& compensate_drift, const bool& separate_particles) {
  integrate_particles(dt, gravity);
  if (separate_particles) {
    push_particles_apart(num_particle_iters);
  }
  handle_particle_collisions();
  transfer_velocities(true, flip_ratio);
  update_particle_density();
  solve_incompressibility(num_pressure_iters, dt, overrelaxation,
                          compensate_drift);
  transfer_velocities(false, flip_ratio);
}

void FlipSim::get_display_mat(DisplayMatrix& mat) {
  mat = mat.Zero();
  for (uint32_t i = 0; i < CELL_NUM_X; i++) {
    if (i == 0 || i == CELL_NUM_X - 1) {
      continue;
    }
    for (uint32_t j = 0; j < CELL_NUM_Y; j++) {
      if (j == 0 || j == CELL_NUM_Y - 1) {
        continue;
      }

      // for our cell_type_ structure it is a one dimensional array, storing x/y
      // values where the first CELL_NUM_Y are the first elements and represent
      // the first row, and then so on. The first row is at 0,0 of a coordinate
      // system, and x goes on the bottom left/right axis and y goes on the top
      // up/down axis.

      // compared to our display matrix which has rows as the first index,
      // columns as second index, and 0 is top left, we need to change this
      // also, note we don't display the edges of the cell_type_

      if (cell_type_(i * CELL_NUM_Y + j) == CellType::FLUID_CELL) {
        // our x is their Y, plus axis is inversed for us/them
        uint32_t disp_x_idx = (NUM_ROWS - 1) - (j - 1);
        // our y is their x
        uint32_t disp_y_idx = i - 1;
        mat(disp_x_idx, disp_y_idx) = true;
      }
    }
  }

  // printf("cell types\n");
  // for (uint32_t i = 0; i < CELL_NUM_X; i++) {
  //   for (uint32_t j = 0; j < CELL_NUM_Y; j++) {
  //     uint32_t disp_x_idx = (NUM_ROWS - 1) - (j - 1);
  //     // our y is their x
  //     uint32_t disp_y_idx = i - 1;

  //     if (disp_x_idx == 0 && disp_y_idx == 0) {
  //       printf("x ");
  //       continue;
  //     }

  //     CellType typ = cell_type_(i * CELL_NUM_Y + j);
  //     if (typ == CellType::FLUID_CELL) {
  //       printf("0 ");
  //     } else if (typ == CellType::AIR_CELL) {
  //       printf("1 ");
  //     } else {
  //       printf("2 ");
  //     }
  //   }
  //   printf("\n");
  // }
  // printf("\n\n");

  // printf("display mat\n");
  // for (uint32_t i = 0; i < NUM_ROWS; i++) {
  //   for (uint32_t j = 0; j < NUM_COLS; j++) {
  //     printf("%d ", mat(i, j));
  //   }
  //   printf("\n");
  // }
  // printf("\n\n");
}