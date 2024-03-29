#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

#include <cstdint>
#include <string>
#include <memory>
#include <unordered_map>

#include "iec61107uart.h"
#include "iec61107sensor.h"

namespace esphome {
namespace iec61107 {

static const size_t MAX_IN_BUF_SIZE = 128;
static const size_t MAX_OUT_BUF_SIZE = 84;

class IEC61107Component : public PollingComponent, public uart::UARTDevice {
 public:
  IEC61107Component() = default;

  void setup() override;
  void dump_config() override;
  void loop() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; };

  void set_receive_timeout_ms(uint32_t timeout) { this->receive_timeout_ms_ = timeout; };
  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }

  void register_sensor(IEC61107SensorBase *sensor);

 protected:
  uint32_t receive_timeout_ms_{3000};
  GPIOPin *flow_control_pin_{nullptr};
  std::unique_ptr<IEC61107UART> iuart_;
  std::unordered_multimap<std::string, IEC61107SensorBase *> sensors_;

  enum class State : uint8_t {
    NOT_INITIALIZED,
    IDLE,
    WAIT,
    OPEN_SESSION,
    OPEN_SESSION_GET_ID,
    ACK_START,
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

  enum class Cmd : uint8_t {
    OPEN_SESSION = 0,
    CLOSE_SESSION,
    ACK_START,
    SNUMB,
    VOLTA,
    //   CURRE,
    //   ET0PE,
    //   ET0PE_T1,
    //   ET0PE_T2,
    //   ET0PE_T3,
    //   ET0PE_T4
    NUMBER_OF_COMMANDS,
  };

  bool receive_proper_reply_(uint8_t *buffer, size_t buffer_length, size_t &received_bytes, const uint8_t stop_marker);
  bool send_command_(Cmd cmd);

  enum class ResponseErr : uint8_t { OK = 0, TIMEOUT, INVALID, CRC_ERROR };
  ResponseErr get_response_(Cmd cmd);

  uint32_t last_transmission_from_meter_timestamp_;
  uint32_t wait_start_timestamp_;
  uint32_t wait_period_ms_;

  uint8_t in_buf_[MAX_IN_BUF_SIZE];
  size_t data_in_size_;
  uint8_t in_bcc_;

  uint8_t out_buf_[MAX_OUT_BUF_SIZE];
  size_t data_out_size_;
  uint8_t out_bcc_;

  size_t receive_frame_();
  void send_frame_();
  void send_frame_(const uint8_t *data, size_t length);
  void prepare_request_frame_(const std::string &request);
  void clear_uart_input_buffer_();

  char *get_id_(size_t frame_size);
  bool parse_line_(const char *line, std::string &out_obis, std::string &out_value1, std::string &out_value2);

  void update_last_transmission_from_meter_timestamp_() { last_transmission_from_meter_timestamp_ = millis(); }
  void reset_bcc_() { in_bcc_ = 0; }

  void abort_mission_();
};

}  // namespace iec61107
}  // namespace esphome
