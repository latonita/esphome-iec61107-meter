#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <cstdint>
#include <string>
#include <memory>
#include <map>

#include "iec61107uart.h"
#include "iec61107sensor.h"

namespace esphome {
namespace iec61107 {

static const size_t MAX_IN_BUF_SIZE = 256;
static const size_t MAX_OUT_BUF_SIZE = 84;

const uint8_t VAL_NUM = 12;
using ValueRefsArray = std::array<const char *, VAL_NUM>;

using SensorMap = std::multimap<std::string, IEC61107SensorBase *>;
using FrameStopFunction = std::function<bool(uint8_t *buf, size_t size)>;

class IEC61107Component : public PollingComponent, public uart::UARTDevice {
 public:
  IEC61107Component() = default;

  void setup() override;
  void dump_config() override;
  void loop() override;
  void update() override;
  float get_setup_priority() const override { return setup_priority::DATA; };

  void set_meter_address(const std::string &addr) { this->meter_address_ = addr; };
  void set_baud_rates(uint32_t baud_rate_handshake, uint32_t baud_rate) {
    this->baud_rate_handshake_ = baud_rate_handshake;
    this->baud_rate_ = baud_rate;
  };
  void set_receive_timeout_ms(uint32_t timeout) { this->receive_timeout_ms_ = timeout; };
  void set_delay_between_requests_ms(uint32_t delay) { this->delay_between_requests_ms_ = delay; };
  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; };

  void register_sensor(IEC61107SensorBase *sensor);
  void set_indicator(binary_sensor::BinarySensor *indicator) { this->indicator_ = indicator; }
  void set_stat_err_crc(sensor::Sensor *sensor) { this->stat_err_crc_ = sensor; }
  void set_reboot_after_failure(uint16_t number_of_failures) {
    this->number_of_failures_before_reboot_ = number_of_failures;
  }

 protected:
  std::string meter_address_{""};
  uint32_t receive_timeout_ms_{750};
  uint32_t delay_between_requests_ms_{350};

  GPIOPin *flow_control_pin_{nullptr};
  std::unique_ptr<IEC61107UART> iuart_;
  SensorMap sensors_;
  binary_sensor::BinarySensor *indicator_{};
  sensor::Sensor *stat_err_crc_{};

  enum class State : uint8_t {
    NOT_INITIALIZED,
    IDLE,
    WAIT,
    OPEN_SESSION,
    OPEN_SESSION_GET_ID,
    SET_BAUD,
    ACK_START_GET_INFO,
    DATA_ENQ,
    DATA_RECV,
    DATA_FAIL,
    DATA_NEXT,
    CLOSE_SESSION,
    PUBLISH,
  } state_{State::NOT_INITIALIZED}, next_state_after_wait_{State::IDLE};

  bool is_idling() const { return this->state_ == State::WAIT || this->state_ == State::IDLE; };
  void set_next_state_(State next_state) { state_ = next_state; };
  void set_next_state_delayed_(uint32_t ms, State next_state);
  void log_state_();

  uint8_t number_of_failures_{0};
  uint8_t number_of_failures_before_reboot_{0};

  uint32_t number_of_connections_tried_{0};
  uint32_t number_of_crc_errors_{0};
  uint32_t number_of_crc_errors_recovered_{0};
  uint32_t number_of_invalid_frames_{0};

  uint8_t retry_counter_{0};

  uint32_t baud_rate_handshake_{9600};
  uint32_t baud_rate_{9600};

  uint32_t last_rx_time_{0};
  uint32_t wait_start_time_;
  uint32_t wait_period_ms_;

  uint8_t in_buf_[MAX_IN_BUF_SIZE];
  size_t data_in_size_;
  uint8_t out_buf_[MAX_OUT_BUF_SIZE];
  size_t data_out_size_;

  void clear_buffers_();
  void set_baud_rate_(uint32_t baud_rate);
  bool are_baud_rates_different_() const { return baud_rate_handshake_ != baud_rate_; }
  uint8_t calculate_crc_frame_r1_(const uint8_t *data, size_t length);
  void prepare_frame_(const uint8_t *data, size_t length);
  void prepare_frame_r1_(const char *request);
  void send_frame_(const uint8_t *data, size_t length);
  void send_frame_prepared_();
  size_t receive_frame_(FrameStopFunction stop_fn);
  size_t receive_frame_ascii_();
  size_t receive_frame_r1_(uint8_t start_byte);
  void retry_or_fail_(bool unclear = false);

  inline void update_last_rx_time_() { last_rx_time_ = millis(); }
  bool check_wait_timeout_() { return millis() - wait_start_time_ >= wait_period_ms_; }
  bool check_rx_timeout_() { return millis() - this->last_rx_time_ >= receive_timeout_ms_; }

  char *extract_meter_id_(size_t frame_size);

  uint8_t get_values_from_brackets_(char *line, ValueRefsArray &vals);
  bool set_sensor_value_(IEC61107SensorBase *sensor, ValueRefsArray &vals);

  void report_failure(bool set_or_clear);
  void abort_mission_();
};

}  // namespace iec61107
}  // namespace esphome
