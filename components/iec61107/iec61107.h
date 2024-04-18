#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#endif

#include <cstdint>
#include <string>
#include <memory>
#include <map>
#include <list>

#include "iec61107uart.h"
#include "iec61107sensor.h"

namespace esphome {
namespace iec61107 {

static const size_t MAX_IN_BUF_SIZE = 256;
static const size_t MAX_OUT_BUF_SIZE = 84;

const uint8_t VAL_NUM = 12;
using ValueRefsArray = std::array<const char *, VAL_NUM>;

using SensorMap = std::multimap<std::string, IEC61107SensorBase *>;
using SingleRequests = std::list<std::string>;

enum class ProgMode : uint8_t { READ = 'R', WRITE = 'W' };

// using FrameStopFunction = std::function<bool(uint8_t *buf, size_t size)>;
// using FrameReadFunction = std::function<size_t()>;

struct Result {
  size_t size{0};
  enum Code : uint8_t { NO_RESPONSE = 0b00, NACK = 0b01, ACK = 0b10, DATA = 0b11 } status{NO_RESPONSE};

  Result(Code c) : size{0}, status{c} {};
  Result(size_t s, Code c) : size{s}, status{c} {};

  operator size_t() const { return size; };

  bool is_ack() const { return status == Code::ACK; }
  bool is_nack() const { return status == Code::NACK; }
  bool is_data_received() const { return status == Code::DATA; }
  bool is_response_received() const { return status != Code::NO_RESPONSE; }
};

using FrameStopFunction = std::function<Result(uint8_t *buf, size_t size)>;
using ReadFrameFunction = std::function<Result()>;

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

#ifdef USE_TIME
  void set_time(time::RealTimeClock *srctime) { this->time_ = srctime; };
#endif

  void register_sensor(IEC61107SensorBase *sensor);
  void set_indicator(binary_sensor::BinarySensor *indicator) { this->indicator_ = indicator; }
  void set_reboot_after_failure(uint16_t number_of_failures) {
    this->number_of_failures_before_reboot_ = number_of_failures;
  }

  void queue_single_read(const std::string &req);
  void queue_single_write(const std::string &req);
  void queue_time_sync() { this->time_sync_requested_ = true; }

 protected:
  std::string meter_address_{""};
  uint32_t receive_timeout_ms_{750};
  uint32_t delay_between_requests_ms_{350};

  GPIOPin *flow_control_pin_{nullptr};
  std::unique_ptr<IEC61107UART> iuart_;
  time::RealTimeClock *time_{nullptr};

  SensorMap sensors_;
  SingleRequests single_requests_;
  bool time_sync_requested_{false};

  binary_sensor::BinarySensor *indicator_{};
  sensor::Sensor *stat_err_crc_{};

  enum class State : uint8_t {
    NOT_INITIALIZED,
    IDLE,
    WAIT,
    WAITING_FOR_RESPONSE,
    OPEN_SESSION,
    OPEN_SESSION_GET_ID,
    SET_BAUD,
    ACK_START_GET_INFO,
    DATA_ENQ,
    DATA_RECV,
    DATA_NEXT,
    TIME_SYNC,
    TIME_CORRECTION,
    TIME_CORRECTION_ACK,
    CLOSE_SESSION,
    PUBLISH,
    SINGLE_READ,
    SINGLE_READ_ACK,
  } state_{State::NOT_INITIALIZED};

  struct {
    uint32_t start_time;
    uint32_t delay_ms;
    State next_state{State::IDLE};
  } wait_{0};

  bool is_idling() const { return this->state_ == State::WAIT || this->state_ == State::IDLE; };

  void set_next_state_(State next_state) { state_ = next_state; };
  void set_next_state_delayed_(uint32_t ms, State next_state);

  void read_reply_and_go_next_state_(ReadFrameFunction read_frame_fn, State next_state, uint8_t retries, bool mission_critical,
                                     bool check_crc);
  struct {
    ReadFrameFunction read_frame_fn;
    State next_state;
    bool mission_critical;
    bool check_crc;
    uint8_t tries_max;
    uint8_t tries_counter;
    uint32_t err_crc;
    uint32_t err_invalid_frames;
  } reading_state_{nullptr, State::IDLE, false, false, 0, 0, 0, 0};
  size_t received_frame_size_{0};

  const char *state_to_string(State state);
  void log_state_(State *next_state = nullptr);

  uint8_t number_of_failures_{0};
  uint8_t number_of_failures_before_reboot_{0};

  uint32_t number_of_connections_tried_{0};
  uint32_t number_of_crc_errors_{0};
  uint32_t number_of_crc_errors_recovered_{0};
  uint32_t number_of_invalid_frames_{0};

  uint32_t baud_rate_handshake_{9600};
  uint32_t baud_rate_{9600};

  uint32_t last_rx_time_{0};

  uint8_t in_buf_[MAX_IN_BUF_SIZE];
  size_t data_in_size_;
  uint8_t out_buf_[MAX_OUT_BUF_SIZE];
  size_t data_out_size_;

  void clear_rx_buffers_();
  void set_baud_rate_(uint32_t baud_rate);
  bool are_baud_rates_different_() const { return baud_rate_handshake_ != baud_rate_; }

  uint8_t calculate_crc_prog_frame_(uint8_t *data, size_t length, bool set_crc = false);
  bool check_crc_prog_frame_(uint8_t *data, size_t length);

  void prepare_frame_(const uint8_t *data, size_t length);
  void prepare_prog_frame_(const char *request, ProgMode mode = ProgMode::READ);
  void prepare_non_session_prog_frame_(const char *request, ProgMode mode = ProgMode::READ);

  void send_frame_(const uint8_t *data, size_t length);
  void send_frame_prepared_();

  Result receive_frame_(FrameStopFunction stop_fn);
  Result receive_frame_ascii_();
  Result receive_prog_frame_(uint8_t start_byte, bool accept_ack_and_nack = false);

  inline void update_last_rx_time_() { last_rx_time_ = millis(); }
  bool check_wait_timeout_() { return millis() - wait_.start_time >= wait_.delay_ms; }
  bool check_rx_timeout_() { return millis() - this->last_rx_time_ >= receive_timeout_ms_; }

  char *extract_meter_id_(size_t frame_size);

  uint8_t get_values_from_brackets_(char *line, ValueRefsArray &vals);
  bool set_sensor_value_(IEC61107SensorBase *sensor, ValueRefsArray &vals);

  void report_failure(bool set_or_clear);
  void abort_mission_();

  bool prepare_time_sync_frame_();
  void print_time_();
};

}  // namespace iec61107
}  // namespace esphome
