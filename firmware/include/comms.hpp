#pragma once
#include <cstdint>
#include <string>

#include "constants.hpp"
#include "types.hpp"

const uint8_t START_BYTE = 255;

const uint8_t BUFFER_SIZE = 128;
const uint8_t HEADER_SIZE = 3;

enum MsgType { ACCEL_GYRO = 0, STATE_EST_COV = 1, DEBUG_STR = 2, END_MSG = 3 };

// we have accel gyro which has 6 floats plus header
// we have state and cov which has N_STATE + N_STATE * N_STATE + header
const uint8_t MSG_SIZES_BYTES[MsgType::END_MSG] = {
    N_MEASUREMENT * 2 * 4, N_STATE * 4 + N_STATE* N_STATE * 4, BUFFER_SIZE};

class Communicator {
 public:
  Communicator();
  bool send_accel_gyro(const MeasurementVector& accel,
                       const MeasurementVector& gyro);
  bool send_state_and_cov(const StateVector& state,
                          const StateCovarianceMatrix& cov);
  bool send_str(const std::string& str);

 protected:
  virtual bool send_buffer() = 0;
  bool header_to_buffer(MsgType msg_type, uint8_t msg_size);
  bool accel_gyro_to_buffer(const MeasurementVector& accel,
                            const MeasurementVector& gyro);
  bool state_est_cov_to_buffer(const StateVector& state,
                               const StateCovarianceMatrix& cov);
  bool debug_str_to_buffer(const std::string& str);
  uint8_t buffer_[BUFFER_SIZE];
  uint8_t buffer_size_;
};

class PicoSerialUSBComms : public Communicator {
 public:
  PicoSerialUSBComms();

 private:
  bool send_buffer();
};