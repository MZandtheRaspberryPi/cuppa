#include "comms.hpp"

#include <pico/stdio/driver.h>
#include <pico/stdio_usb.h>

#include "utils.hpp"

Communicator::Communicator() {}
bool Communicator::send_accel_gyro(const MeasurementVector& accel,
                                   const MeasurementVector& gyro) {
  buffer_size_ = 0;
  if ((MSG_SIZES_BYTES[0]) > BUFFER_SIZE) {
    return false;
  }
  header_to_buffer(MsgType::ACCEL_GYRO, MSG_SIZES_BYTES[0]);
  buffer_size_ += HEADER_SIZE;
  accel_gyro_to_buffer(accel, gyro);
  buffer_size_ += MSG_SIZES_BYTES[0];
  send_buffer();
  return true;
}

bool Communicator::send_state_and_cov(const StateVector& state,
                                      const StateCovarianceMatrix& cov) {
  buffer_size_ = 0;
  if ((MSG_SIZES_BYTES[1]) > BUFFER_SIZE) {
    return false;
  }
  header_to_buffer(MsgType::STATE_EST_COV, MSG_SIZES_BYTES[1]);
  buffer_size_ += HEADER_SIZE;
  state_est_cov_to_buffer(state, cov);
  buffer_size_ += MSG_SIZES_BYTES[1];
  send_buffer();
  return true;
}

bool Communicator::send_str(const std::string& str) {
  buffer_size_ = 0;
  if ((str.size() + HEADER_SIZE) > BUFFER_SIZE) {
    return false;
  }
  header_to_buffer(MsgType::DEBUG_STR, str.size());
  buffer_size_ += HEADER_SIZE;
  debug_str_to_buffer(str);
  buffer_size_ += str.size();
  send_buffer();
  return true;
}

bool Communicator::header_to_buffer(MsgType msg_type, uint8_t msg_size) {
  buffer_[0] = START_BYTE;
  buffer_[1] = static_cast<uint8_t>(msg_type);
  buffer_[2] = msg_size;
  return true;
}

bool Communicator::accel_gyro_to_buffer(const MeasurementVector& accel,
                                        const MeasurementVector& gyro) {
  uint8_t cur_idx = HEADER_SIZE;
  uint8_t tmp[4];
  for (uint8_t i = 0; i < 2; i++) {
    const MeasurementVector& mes = (i == 0) ? accel : gyro;
    for (uint8_t j = 0; j < N_MEASUREMENT; j++) {
      float_to_bytes(mes[j], tmp);
      for (uint8_t k = 0; k < 4; k++) {
        buffer_[cur_idx] = static_cast<unsigned char>(tmp[k]);
        cur_idx += 1;
      }
    }
  }
  buffer_[cur_idx] = '\0';
  return true;
}

bool Communicator::state_est_cov_to_buffer(const StateVector& state,
                                           const StateCovarianceMatrix& cov) {
  uint8_t cur_idx = HEADER_SIZE;
  uint8_t tmp[4];
  for (uint8_t i = 0; i < N_STATE; i++) {
    float_to_bytes(state[i], tmp);
    for (uint8_t k = 0; k < 4; k++) {
      buffer_[cur_idx] = tmp[k];
      cur_idx += 1;
    }
  }

  for (uint8_t i = 0; i < N_STATE; i++) {
    for (uint8_t j = 0; j < N_STATE; j++) {
      float_to_bytes(cov(i, j), tmp);
      for (uint8_t k = 0; k < 4; k++) {
        buffer_[cur_idx] = tmp[k];
        cur_idx += 1;
      }
    }
  }
  buffer_[cur_idx] = '\0';
  return true;
}

bool Communicator::debug_str_to_buffer(const std::string& str) {
  uint8_t cur_idx = HEADER_SIZE;
  for (uint8_t i = 0; i < str.size(); i++) {
    buffer_[cur_idx] = static_cast<uint8_t>(str[i]);
    cur_idx += 1;
  }
  buffer_[cur_idx] = '\0';
  return true;
}

PicoSerialUSBComms::PicoSerialUSBComms() {
  stdio_set_translate_crlf(&stdio_usb, false);
}

bool PicoSerialUSBComms::send_buffer() {
  if (!stdio_usb_connected()) {
    return true;
  }
  for (uint8_t i = 0; i < buffer_size_; i++) {
    printf("%c", buffer_[i]);
  }
  // printf("%s", buffer_);
  // printf("%.*hhu", buffer_size_, buffer_);
  return true;
}
