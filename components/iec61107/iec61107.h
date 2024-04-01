#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

#include <cstdint>
#include <string>
#include <memory>
//#include <unordered_map>
#include <list>

#include "iec61107uart.h"
#include "iec61107sensor.h"

namespace esphome {
namespace iec61107 {

static const size_t MAX_IN_BUF_SIZE = 128;
static const size_t MAX_OUT_BUF_SIZE = 84;

const uint8_t VAL_NUM = 4;
using ValuesArray = std::array<std::string, VAL_NUM>;
using SensorMap = std::list<IEC61107SensorBase *>;  // std::unordered_multimap<std::string, IEC61107SensorBase *>;

class IEC61107Component : public PollingComponent, public uart::UARTDevice {
 public:
  IEC61107Component() = default;

  void setup() override;
  void dump_config() override;
  void loop() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; };

  void set_receive_timeout_ms(uint32_t timeout) { this->receive_timeout_ms_ = timeout; };
  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; };
  void set_enable_readout(bool readout_mode) { this->readout_mode_ = readout_mode; };

  void register_sensor(IEC61107SensorBase *sensor);

 protected:
  uint32_t receive_timeout_ms_{3000};
  GPIOPin *flow_control_pin_{nullptr};
  std::unique_ptr<IEC61107UART> iuart_;
  SensorMap sensors_;
  bool readout_mode_{true};

  enum class State : uint8_t {
    NOT_INITIALIZED,
    IDLE,
    WAIT,
    OPEN_SESSION,
    OPEN_SESSION_GET_ID,
    SET_BAUD_RATE,
    ACK_START_GET_INFO,
    DATA_ENQ,
    DATA_RECV,
    DATA_FAIL,
    DATA_NEXT,
    READOUT,
    CLOSE_SESSION,
    PUBLISH,
  } state_{State::NOT_INITIALIZED}, next_state_after_wait_{State::IDLE};

  void set_next_state_(State next_state) { state_ = next_state; };
  void set_next_state_delayed_(uint32_t ms, State next_state);
  bool check_wait_period_() { return millis() - wait_start_timestamp_ >= wait_period_ms_; }
  bool is_idling() const { return this->state_ == State::WAIT || this->state_ == State::IDLE; };

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

  size_t receive_frame_();
  void prepare_frame_(const uint8_t *data, size_t length);
  void send_frame_(const uint8_t *data, size_t length);
  void send_frame_();
  void prepare_request_frame_(const std::string &request);
  void clear_uart_input_buffer_();

  char *get_id_(size_t frame_size);
  bool parse_line_(const char *line, std::string &out_obis, std::string &out_value1, std::string &out_value2);
  uint8_t get_values_from_brackets_(const char *line, std::string &param, ValuesArray &vals);
  bool set_sensor_value_(IEC61107SensorBase *sensor, ValuesArray &vals);
  void update_last_transmission_from_meter_timestamp_() { last_transmission_from_meter_timestamp_ = millis(); }
  void reset_bcc_();
  void update_bcc_(const uint8_t *data, size_t size);
  void report_state_();
  uint32_t identification_to_baud_rate_(char z);
  char baud_rate_to_identification_(uint32_t baud_rate);

  void abort_mission_();
};

}  // namespace iec61107
}  // namespace esphome
