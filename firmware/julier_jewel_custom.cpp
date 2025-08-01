

#include <Tlv493d.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <pico/sync.h>
#include <stdio.h>

#include <Eigen/Dense>

#include "DFRobot_ICM42688.h"
#include "button.hpp"
#include "charlieplex.hpp"
#include "comms.hpp"
#include "flip.hpp"
#include "fluid_sim.hpp"
#include "hardware/i2c.h"
#include "imu_icm20948.hpp"
#include "imu_mpu6500.hpp"
#include "timer.hpp"
#include "types.hpp"
#include "ukf.hpp"

mutex_t display_mat_mtx;
mutex_t gravity_mtx;
mutex_t sim_hz_mtx;
DisplayMatrix display_mat_global = DisplayMatrix::Ones();
uint16_t sim_hz_rate_global = 0;
Eigen::Matrix<float32_t, 2, 1> gravity_global =
    Eigen::Matrix<float32_t, 2, 1>::Zero();

void core1_entry() {
  float32_t sim_dt = 1.0 / 120;
  Timer sim_timer((uint32_t)(sim_dt * 1000));
  Timer update_loop_rate_timer(1000);
  FlipSim sim = FlipSim();

  Eigen::Matrix<float32_t, 2, 1> gravity2 =
      Eigen::Matrix<float32_t, 2, 1>::Zero();
  gravity2(1) = -9.81;
  DisplayMatrix display_mat2 = DisplayMatrix::Zero();
  display_mat2(3, 0) = true;

  uint16_t loop_rate2 = 0;

  while (true) {
    if (sim_timer.is_ready()) {
      if (mutex_enter_timeout_ms(&gravity_mtx, 10)) {
        gravity2(0) = gravity_global(1);
        gravity2(1) = -gravity_global(0);
        mutex_exit(&gravity_mtx);
      }

      sim.step(sim_dt, gravity2, 0.9, 50, 2, 1.9, true, true);
      sim.get_display_mat(display_mat2);

      if (mutex_enter_timeout_ms(&display_mat_mtx, 100)) {
        display_mat_global = display_mat2;
        mutex_exit(&display_mat_mtx);
      }

      sim_timer.set_run_time();
    }

    if (update_loop_rate_timer.is_ready()) {
      loop_rate2 = sim_timer.get_actual_loop_rate();
      if (mutex_enter_timeout_ms(&sim_hz_mtx, 10)) {
        sim_hz_rate_global = loop_rate2;
        mutex_exit(&sim_hz_mtx);
      }

      update_loop_rate_timer.set_run_time();
    }
  }
}

int main() {
  stdio_init_all();
  mutex_init(&display_mat_mtx);
  mutex_init(&gravity_mtx);
  mutex_init(&sim_hz_mtx);
  uint32_t imu_timer_us = 1000000 * 1 / 100;
  Timer imu_timer(imu_timer_us);
  Timer led_timer(10000);
  Timer led_timer2(250);
  Timer comms_timer(100000);
  Timer comms_timer_rates(1000000);
  Timer button_check_timer(100000);
  Timer update_gravity_timer(1000000 * (1 / 60));
  Timer viz_update_timer(100000);

  PicoSerialUSBComms comms;

  MeasurementVector accel;
  MeasurementVector gyro;
  accel(0) = 1.1;
  accel(1) = 2.1;
  accel(2) = 4.3;
  gyro(0) = -1.3;
  gyro(1) = 10.0;
  gyro(2) = -5.5;

  uint8_t cur_row = 0;
  uint8_t cur_col = 0;

  uint8_t buffer[4] = {0};

  for (uint8_t i = 0; i < NUM_ROWS; i++) {
    gpio_init(LED_PINS[i]);
  }

  DisplayMatrix display_mat = DisplayMatrix::Zero();
  display_mat = DisplayMatrix::Zero();
  // display_mat(0, 0) = 1;
  // display_mat(4, 0) = 1;

  // display_mat(0, 2) = 1;
  // display_mat(1, 3) = 1;
  // display_mat(2, 3) = 1;
  // display_mat(3, 3) = 1;
  // display_mat(4, 2) = 1;
  // display_mat = DisplayMatrix::Ones();
  // display_mat(5, 5) = true;
  // display_mat(5, 6) = true;
  // display_mat(6, 6) = true;
  // display_mat(7, 7) = true;
  // display_mat(8, 8) = true;
  // display_mat(9, 9) = true;

  // has to be after GPIO init
  LEDViz led_viz = LEDViz();
  led_viz.update_display(display_mat);

  // ICM20948 imu = ICM20948();
  MPU6500 imu = MPU6500();
  uint8_t ret = 1;
  ret = imu.setup();

  while (ret != 0) {
    printf("failed imu setup...");
    sleep_ms(200);
    ret = imu.setup();
  }

  // DFRobot_ICM42688_SPI ICM42688(/* csPin= */ 5);
  // int ret = ICM42688.begin();
  // while (ret != 0) {
  //   if (ret == -1) {
  //     printf("imu init failure...");
  //   } else
  //     printf("Chip versions do not match %d...", ret);
  //   sleep_ms(1000);
  //   ret = ICM42688.begin();
  // }

  float32_t dt = (float32_t)imu_timer_us / 1000000.0;

  MeasurementVector meas;
  ControlVector ctrl;

  StateCovarianceMatrix cov = StateCovarianceMatrix::Identity() * M_PI / 12;
  StateVector state = StateVector::Zero();

  StateCovarianceMatrix Q = StateCovarianceMatrix::Identity() * M_PI / 12;
  MeasurementCovarianceMatrix R =
      MeasurementCovarianceMatrix::Identity() * M_PI / 12;

  UKF ukf(Q, R, dt);

  // Button button1 = Button(INPUT1_PIN);
  // Button button2 = Button(INPUT2_PIN);

  Eigen::Matrix<float32_t, 3, 1> normal_gravity =
      Eigen::Matrix<float32_t, 3, 1>::Zero();
  normal_gravity(2) = -9.81;

  uint16_t sim_hz_rate = 0;

  multicore_launch_core1(core1_entry);

  uint8_t cur_col_idx, cur_row_idx;
  cur_col_idx = cur_row_idx = 0;

  Tlv493d Tlv493dMagnetic3DSensor = Tlv493d();
  Tlv493dMagnetic3DSensor.begin(TLV493D_ADDRESS1, true);

  Tlv493dMagnetic3DSensor.setAccessMode(Tlv493d::AccessMode_e::LOWPOWERMODE);

  bool do_predict = true;
  bool do_update = true;

  while (true) {
    if (led_timer2.is_ready()) {
      led_viz.tick();
      led_timer2.set_run_time();
    }

    // if (led_timer.is_ready()) {
    //   display_mat(cur_row_idx, cur_col_idx) = false;
    //   if (cur_col_idx == NUM_COLS - 1) {
    //     cur_row_idx = cur_row_idx == NUM_ROWS - 1 ? 0 : cur_row_idx + 1;
    //     cur_col_idx = 0;
    //   } else {
    //     cur_col_idx += 1;
    //   }
    //   display_mat(cur_row_idx, cur_col_idx) = true;
    //   led_viz.update_display(display_mat);

    //   led_timer.set_run_time();
    // }

    if (imu_timer.is_ready()) {
      ret = imu.get_gyro_accel(gyro(0), gyro(1), gyro(2), accel(0), accel(1),
                               accel(2));
      printf("ret %d, got imu, gyr: %f, %f, %f, acc: %f, %f, %f\n", ret,
             gyro(0), gyro(1), gyro(2), accel(0), accel(1), accel(2));
      ctrl(0) = gyro(0);
      ctrl(1) = gyro(1);
      ctrl(2) = gyro(2);

      if (do_predict) {
        ukf.predict(state, cov, ctrl);
      }
      meas(0) = accel(0);
      meas(1) = accel(1);
      meas(2) = accel(2);

      if (do_update) {
        ukf.update_accel(meas, state, cov);
      }

      // accel(1) = -ICM42688.getAccelDataX();
      // accel(0) = ICM42688.getAccelDataY();
      // accel(2) = ICM42688.getAccelDataZ();
      // gyro(1) = -ICM42688.getGyroDataX();
      // gyro(0) = ICM42688.getGyroDataY();
      // gyro(2) = ICM42688.getGyroDataZ();

      // printf("acc data: x %f y %f z %f\n", accel(0), accel(1), accel(2));

      // uint8_t pwr_mgmt0;
      // ICM42688.readRandomReg(0, 0x4E, &pwr_mgmt0, 1);
      // printf("pwr mgmt 0: %d\n", pwr_mgmt0);
      // printf("gyro data: x %f y %f z %f\n", gyro(0), gyro(1), gyro(2));

      // Tlv493dMagnetic3DSensor.updateData();

      // float mag_data[3] = {0};
      // mag_data[0] = Tlv493dMagnetic3DSensor.getX();
      // mag_data[1] = Tlv493dMagnetic3DSensor.getY();
      // mag_data[2] = Tlv493dMagnetic3DSensor.getX();
      // printf("mag data: x %f y %f z %f\n", mag_data[0], mag_data[1],
      //        mag_data[2]);

      // float temp = Tlv493dMagnetic3DSensor.getTemp();
      // printf("mag data: temp %f\n", temp);

      imu_timer.set_run_time();
    }

    if (update_gravity_timer.is_ready()) {
      RotationMatrix R;
      get_x_rotation_matrix(state(0), R);
      Eigen::Matrix<float32_t, 3, 1> new_grav = R * normal_gravity;
      get_y_rotation_matrix(state(1), R);
      new_grav = R * new_grav;

      if (mutex_enter_timeout_ms(&gravity_mtx, 10)) {
        gravity_global(0) = new_grav(0);
        gravity_global(1) = new_grav(1);
        mutex_exit(&gravity_mtx);
      }

      if (mutex_enter_timeout_ms(&display_mat_mtx, 10)) {
        display_mat = display_mat_global;
        mutex_exit(&display_mat_mtx);
      }

      if (mutex_enter_timeout_ms(&sim_hz_mtx, 10)) {
        sim_hz_rate = sim_hz_rate_global;
        mutex_exit(&sim_hz_mtx);
      }

      update_gravity_timer.set_run_time();
    }

    if (comms_timer.is_ready()) {
      comms.send_accel_gyro(accel, gyro);
      comms.send_state_and_cov(state, cov);
      comms_timer.set_run_time();
    }

    if (viz_update_timer.is_ready()) {
      led_viz.update_display(display_mat);
      viz_update_timer.set_run_time();
    }

    // if (button_check_timer.is_ready()) {
    //   if (button1.is_pressed()) {
    //     do_predict = !do_predict;

    //     std::string debug_str = " button1 pressed";
    //     comms.send_str(debug_str);
    //   }
    //   if (button2.is_pressed()) {
    //     std::string debug_str = " button2 pressed";
    //     comms.send_str(debug_str);
    //     do_update = !do_update;
    //   }
    //   button_check_timer.set_run_time();
    // }

    if (comms_timer_rates.is_ready()) {
      uint16_t led_loop_rate = led_timer2.get_actual_loop_rate();
      uint16_t ukf_loop_rate = imu_timer.get_actual_loop_rate();
      std::string debug_str =
          "led loop rate: " + std::to_string(led_loop_rate) +
          " Hz, ukf loop rate: " + std::to_string(ukf_loop_rate) +
          " Hz, sim loop rate: " + std::to_string(sim_hz_rate) + " Hz";
      comms.send_str(debug_str);

      comms_timer_rates.set_run_time();
    }
  }
}
