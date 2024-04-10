#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <cstdint>
#include <string>
#include <memory>
#include <unordered_map>
#include <set>

#include "iec61107uart.h"
#include "iec61107sensor.h"

namespace esphome {
namespace iec61107 {

static const size_t MAX_IN_BUF_SIZE = 256;
static const size_t MAX_OUT_BUF_SIZE = 84;

const uint8_t VAL_NUM = 4;
using ValuesArray = std::array<const char *, VAL_NUM>;
using ParamName = char *;

struct CharPtrEqual {
  bool operator()(const char *a, const char *b) const { return std::strcmp(a, b) == 0; }
};
struct CharPtrComparator {
  bool operator()(const char *a, const char *b) const { return std::strcmp(a, b) < 0; }
};
struct CharPtrHash {
  std::size_t operator()(const char *str) const {
    std::size_t hash = 0;
    while (*str) {
      hash = (hash * 131) + *str;
      ++str;
    }
    return hash;
  }
};

using SensorMap = std::unordered_multimap<const char *, IEC61107SensorBase *, CharPtrHash, CharPtrEqual>;
using RequestsSet = std::set<const char *, CharPtrComparator>;
using FrameStopFunction = std::function<bool(uint8_t *buf, size_t size)>;

// using SensorMap = std::unordered_multimap<std::string, IEC61107SensorBase *>;
// using RequestsSet = std::set<std::string>;

class IEC61107Component : public PollingComponent, public uart::UARTDevice {
 public:
  IEC61107Component() = default;

  void setup() override;
  void dump_config() override;
  void loop() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; };

  void set_receive_timeout_ms(uint32_t timeout) { this->receive_timeout_ms_ = timeout; };
  void set_delay_between_requests_ms(uint32_t delay) { this->delay_between_requests_ms_ = delay; };
  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; };
  void set_enable_readout(bool readout_mode) { this->readout_mode_ = readout_mode; };

  void register_sensor(IEC61107SensorBase *sensor);
  void set_indicator(binary_sensor::BinarySensor *indicator) { this->indicator_ = indicator; }
  void set_reboot_after_failure(uint16_t number_of_failures) { number_of_failures_before_reboot_ = number_of_failures; }

 protected:
  uint32_t receive_timeout_ms_{750};
  uint32_t delay_between_requests_ms_{350};

  GPIOPin *flow_control_pin_{nullptr};
  std::unique_ptr<IEC61107UART> iuart_;
  SensorMap sensors_;
  RequestsSet requests_;
  bool readout_mode_{false};
  binary_sensor::BinarySensor *indicator_{};

  enum class State : uint8_t {
    NOT_INITIALIZED,
    IDLE,
    WAIT,
    OPEN_SESSION,
    OPEN_SESSION_GET_ID,
    ACK_START_GET_INFO,
    DATA_ENQ,
    DATA_RECV,
    DATA_FAIL,
    DATA_NEXT,
    CLOSE_SESSION,
    PUBLISH,
  } state_{State::NOT_INITIALIZED}, next_state_after_wait_{State::IDLE};

  void set_next_state_(State next_state) { state_ = next_state; };
  void set_next_state_delayed_(uint32_t ms, State next_state);
  bool check_wait_period_() { return millis() - wait_start_timestamp_ >= wait_period_ms_; }
  bool is_idling() const { return this->state_ == State::WAIT || this->state_ == State::IDLE; };

  void report_failure(bool set_or_clear);
  uint8_t number_of_failures_{0};
  uint8_t number_of_failures_before_reboot_{0};

  uint32_t last_transmission_from_meter_timestamp_;
  uint32_t wait_start_timestamp_;
  uint32_t wait_period_ms_;
  char baud_rate_identification_{'5'};
  char mode_{'C'};

  uint8_t in_buf_[MAX_IN_BUF_SIZE];
  size_t data_in_size_;
  uint8_t in_bcc_;

  uint8_t out_buf_[MAX_OUT_BUF_SIZE];
  size_t data_out_size_;
  uint8_t bcc_;

  size_t receive_frame_(FrameStopFunction stop_fn);
  size_t receive_frame_ascii_();
  size_t receive_frame_r1_(uint8_t start_byte);

  void prepare_frame_(const uint8_t *data, size_t length);
  void send_frame_(const uint8_t *data, size_t length);
  void send_frame_();
  void prepare_request_frame_(const char *request);
  void clear_uart_input_buffer_();

  char *get_id_(size_t frame_size);
  
  uint8_t get_values_from_brackets_(char *line, ValuesArray &vals);
  bool set_sensor_value_(IEC61107SensorBase *sensor, ValuesArray &vals);
  void update_last_transmission_from_meter_timestamp_() { last_transmission_from_meter_timestamp_ = millis(); }
  void reset_bcc_();
  void update_bcc_(const uint8_t *data, size_t size);
  uint8_t r1_frame_crc(const uint8_t *data, size_t length);

  void report_state_();
  
  uint32_t identification_to_baud_rate_(char z);
  char baud_rate_to_identification_(uint32_t baud_rate);

  void abort_mission_();
};

}  // namespace iec61107
}  // namespace esphome
