#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/application.h"
#include "esphome/core/time.h"
#include "iec61107.h"
#include "iec61107_protocol.h"
#include <sstream>

namespace esphome {
namespace iec61107 {

static const char *TAG0 = "iec61107_";
#define TAG (this->tag_.c_str())

using protocol::SOH;
using protocol::STX;
using protocol::ETX;
using protocol::EOT;
using protocol::ENQ;
using protocol::ACK;
using protocol::CR;
using protocol::LF;
using protocol::NAK;

static constexpr uint8_t BOOT_WAIT_S = 10;

static char empty_str[] = "";

static char format_hex_char(uint8_t v) { return v >= 10 ? 'A' + (v - 10) : '0' + v; }

static std::string format_frame_pretty(const uint8_t *data, size_t length) {
  if (length == 0)
    return "";
  std::string ret;
  ret.resize(3 * length - 1);
  std::ostringstream ss(ret);

  for (size_t i = 0; i < length; i++) {
    switch (data[i]) {
      case 0x00:
        ss << "<NUL>";
        break;
      case 0x01:
        ss << "<SOH>";
        break;
      case 0x02:
        ss << "<STX>";
        break;
      case 0x03:
        ss << "<ETX>";
        break;
      case EOT:
        ss << "<EOT>";
        break;
      case ENQ:
        ss << "<ENQ>";
        break;
      case ACK:
        ss << "<ACK>";
        break;
      case CR:
        ss << "<CR>";
        break;
      case LF:
        ss << "<LF>";
        break;
      case NAK:
        ss << "<NAK>";
        break;
      case 0x20:
        ss << "<SP>";
        break;
      default:
        if (data[i] <= 0x20 || data[i] >= 0x7f) {
          ss << "<" << format_hex_char((data[i] & 0xF0) >> 4) << format_hex_char(data[i] & 0x0F) << ">";
        } else {
          ss << (char) data[i];
        }
        break;
    }
  }
  if (length > 4)
    ss << " (" << length << ")";
  return ss.str();
}

uint8_t baud_rate_to_byte(uint32_t baud) {
  constexpr uint16_t BAUD_BASE = 300;
  constexpr uint8_t BAUD_MULT_MAX = 6;

  uint8_t idx = 0;  // 300
  for (size_t i = 0; i <= BAUD_MULT_MAX; i++) {
    if (baud == BAUD_BASE * (1 << i)) {
      idx = i;
      break;
    }
  }
  return idx + '0';
}

void Iec61107Component::set_baud_rate_(uint32_t baud_rate) {
  ESP_LOGV(TAG, "Setting baud rate %u bps", baud_rate);
  if (!iuart_->update_baudrate(baud_rate)) {
    ESP_LOGE(TAG, "Failed to update baud rate");
  }
}

void Iec61107Component::setup() {
  ESP_LOGD(TAG, "setup");
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
  }
#ifdef USE_ESP32
  iuart_ = make_unique<Iec61107Uart>(*static_cast<uart::IDFUARTComponent *>(this->parent_));
  // if (this->flow_control_pin_ != nullptr) {
  //   if (this->flow_control_pin_->is_internal()) {
  //     ESP_LOGI(TAG, "Flow control pin is internal GPIO pin, half-duplex mode is enabled in UART driver");
  //     auto pin = static_cast<InternalGPIOPin *>(this->flow_control_pin_);
  //     if (pin != nullptr && pin->get_pin() >= 0) {
  //       ESP_LOGI(TAG, "Flow control pin: GPIO%d", pin->get_pin());
  //       iuart_->setup_half_duplex(32);  //
  //       // pin->get_pin());
  //     } else {
  //       ESP_LOGW(TAG, "Flow control pin is not set, using default GPIO");
  //     }
  //   } else {
  //     ESP_LOGW(TAG, "Flow control pin is not internal GPIO pin, half-duplex mode is manual");
  //   }
  // }
#endif

#if USE_ESP8266
  iuart_ = make_unique<Iec61107Uart>(*static_cast<uart::ESP8266UartComponent *>(this->parent_));
#endif

  this->set_baud_rate_(this->baud_rate_handshake_);
  this->set_timeout(BOOT_WAIT_S * 1000, [this]() {
    ESP_LOGD(TAG, "Boot timeout, component is ready to use");
    this->clear_rx_buffers_();
    this->set_next_state_(State::IDLE);
  });

}

void Iec61107Component::dump_config() {
  ESP_LOGCONFIG(TAG, "IEC 61107: %p", this);

  LOG_UPDATE_INTERVAL(this);
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  ESP_LOGCONFIG(TAG, "  Receive Timeout: %ums", this->receive_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Sensors:");
  for (const auto &sensors : sensors_) {
    auto &s = sensors.second;
    ESP_LOGCONFIG(TAG, "    REQUEST: %s", s->get_request().c_str());
  }
}

void Iec61107Component::register_sensor(Iec61107SensorBase *sensor) {
  this->sensors_.insert({sensor->get_request(), sensor});
}

void Iec61107Component::abort_mission_(bool send_close_session) {
  if (send_close_session) {
    ESP_LOGE(TAG, "Abort mission. Closing session");
    uint8_t close_cmd[protocol::CLOSE_SESSION_FRAME_SIZE];
    size_t close_cmd_len =
        protocol::build_close_session_frame(close_cmd, sizeof(close_cmd), this->crc_method_ == IecCrcMethod::CRC_XOR);
    this->send_frame_(close_cmd, close_cmd_len);
  } else {
    ESP_LOGE(TAG, "Abort mission.");
  }
  if (this->are_baud_rates_different_()) {
    this->set_baud_rate_(this->baud_rate_handshake_);
  }
  this->unlock_uart_session_();
  this->set_next_state_(State::IDLE);
  this->report_failure(true);
}

void Iec61107Component::report_failure(bool failure) {
  if (!failure) {
    this->stats_.failures_ = 0;
    return;
  }

  this->stats_.failures_++;
  if (this->failures_before_reboot_ > 0 && this->stats_.failures_ > this->failures_before_reboot_) {
    ESP_LOGE(TAG, "Too many failures in a row. Let's try rebooting device.");
    delay(100);
    App.safe_reboot();
  }
}

void Iec61107Component::loop() {
  if (!this->is_ready() || this->state_ == State::NOT_INITIALIZED)
    return;

  switch (this->state_) {
    case State::IDLE:
      this->handle_idle_();
      break;
    case State::TRY_LOCK_BUS:
      this->handle_try_lock_bus_();
      break;
    case State::WAIT:
      this->handle_wait_();
      break;
    case State::WAITING_FOR_RESPONSE:
      this->handle_waiting_for_response_();
      break;
    case State::OPEN_SESSION:
      this->handle_open_session_();
      break;
    case State::OPEN_SESSION_GET_ID:
      this->handle_open_session_get_id_();
      break;
    case State::SET_BAUD:
      this->handle_set_baud_();
      break;
    case State::ACK_READ_P0_CONFIRMATION:
      this->handle_ack_read_p0_confirmation_();
      break;
    case State::ACK_START_GET_INFO:
      this->handle_ack_start_get_info_();
      break;
    case State::PROGRAMMING_MODE_REQ:
      this->handle_programming_mode_req_();
      break;
    case State::PROGRAMMING_MODE_ACK:
      this->handle_programming_mode_ack_();
      break;
    case State::MAIN_SESSION:
      this->handle_main_session_();
      break;
    case State::GET_DATE:
      this->handle_get_date_();
      break;
    case State::GET_TIME:
      this->handle_get_time_();
      break;
    case State::CORRECT_TIME:
      this->handle_correct_time_();
      break;
    case State::RECV_CORRECTION_RESULT:
      this->handle_recv_correction_result_();
      break;
    case State::DATA_ENQ:
      this->handle_data_enq_();
      break;
    case State::DATA_RECV:
      this->handle_data_recv_();
      break;
    case State::DATA_NEXT:
      this->handle_data_next_();
      break;
    case State::CLOSE_SESSION:
      this->handle_close_session_();
      break;
    case State::PUBLISH:
      this->handle_publish_();
      break;
    case State::SINGLE_READ:
      this->handle_single_read_();
      break;
    case State::SINGLE_READ_ACK:
      this->handle_single_read_ack_();
      break;

    default:
      break;
  }
}

void Iec61107Component::handle_idle_() {
  this->update_last_rx_time_();
  // auto request = this->single_requests_.front();

  // if (this->single_requests_.empty())
  //   return;

  // this->single_requests_.pop_front();
  // ESP_LOGD(TAG, "Performing single request '%s'", request.c_str());
  // this->prepare_non_session_prog_frame_(request.c_str());
  // this->send_frame_prepared_();
  // auto read_fn = [this]() { return this->receive_prog_frame_(STX, true); };
  // this->read_reply_and_go_next_state_(read_fn, State::SINGLE_READ_ACK, 3, false, true);
}

void Iec61107Component::handle_try_lock_bus_() {
  this->log_state_();
  if (this->try_lock_uart_session_()) {
    this->set_baud_rate_(this->baud_rate_handshake_);
    this->set_next_state_delayed_(50, State::OPEN_SESSION);
  } else {
    ESP_LOGV(TAG, "UART Bus is busy, waiting ...");
    this->set_next_state_delayed_(1000, State::TRY_LOCK_BUS);
  }
}

void Iec61107Component::handle_wait_() {
  if (this->check_wait_timeout_()) {
    this->set_next_state_(this->wait_.next_state);
    this->update_last_rx_time_();
  }
}

void Iec61107Component::handle_waiting_for_response_() {
  this->log_state_(&reading_state_.next_state);
  received_frame_size_ = reading_state_.read_fn();

  bool crc_is_ok = true;
  if (reading_state_.check_crc && received_frame_size_ > 0) {
    crc_is_ok = check_crc_prog_frame_(this->buffers_.in, received_frame_size_);
  }

  // happy path first
  if (received_frame_size_ > 0 && crc_is_ok) {
    this->set_next_state_(reading_state_.next_state);
    this->update_last_rx_time_();
    this->stats_.crc_errors_ += reading_state_.err_crc;
    this->stats_.crc_errors_recovered_ += reading_state_.err_crc;
    this->stats_.invalid_frames_ += reading_state_.err_invalid_frames;
    return;
  }

  // ESP_LOGVV(TAG, "buffers in %d",this->buffers_.amount_in);
  //  half-happy path
  //  if not timed out yet, wait for data to come a little more
  if (crc_is_ok && !this->check_rx_timeout_()) {
    return;
  }

  if (received_frame_size_ == 0) {
    this->reading_state_.err_invalid_frames++;
    ESP_LOGW(TAG, "RX timeout.");
  } else if (!crc_is_ok) {
    this->reading_state_.err_crc++;
    ESP_LOGW(TAG, "Frame received, but CRC failed.");
  } else {
    this->reading_state_.err_invalid_frames++;
    ESP_LOGW(TAG, "Frame corrupted.");
  }

  // if we are here, we have a timeout and no data
  // it means we have a failure
  // - either no reply from the meter at all
  // - or corrupted data and id doesn't trigger stop function
  if (this->buffers_.amount_in > 0) {
    // most likely its CRC error in STX/SOH/ETX. unclear.
    this->stats_.crc_errors_++;
    ESP_LOGV(TAG, "RX: %s", format_frame_pretty(this->buffers_.in, this->buffers_.amount_in).c_str());
    ESP_LOGVV(TAG, "RX: %s", format_hex_pretty(this->buffers_.in, this->buffers_.amount_in).c_str());
  }
  this->clear_rx_buffers_();

  if (reading_state_.mission_critical) {
    this->stats_.crc_errors_ += reading_state_.err_crc;
    this->stats_.invalid_frames_ += reading_state_.err_invalid_frames;
    this->abort_mission_();
    return;
  }

  if (reading_state_.tries_counter < reading_state_.tries_max) {
    reading_state_.tries_counter++;
    ESP_LOGW(TAG, "Retrying [%d/%d]...", reading_state_.tries_counter, reading_state_.tries_max);
    this->send_frame_prepared_();
    this->update_last_rx_time_();
    return;
  }
  received_frame_size_ = 0;
  // failure, advancing to next state with no data received (frame_size = 0)
  this->stats_.crc_errors_ += reading_state_.err_crc;
  this->stats_.invalid_frames_ += reading_state_.err_invalid_frames;
  this->set_next_state_(reading_state_.next_state);
}

void Iec61107Component::handle_open_session_() {
  this->stats_.connections_tried_++;
  this->loop_state_.session_started_ms = millis();
  this->log_state_();

  this->clear_rx_buffers_();

  uint8_t open_cmd[32]{0};
  uint8_t open_cmd_len = snprintf((char *) open_cmd, 32, "/?%s!\r\n", this->meter_address_.c_str());
  this->loop_state_.request_iter = this->sensors_.begin();
  this->send_frame_(open_cmd, open_cmd_len);
  this->set_next_state_(State::OPEN_SESSION_GET_ID);
  auto read_fn = [this]() { return this->receive_frame_ascii_(); };
  this->read_reply_and_go_next_state_(read_fn, State::OPEN_SESSION_GET_ID, 3, true, false);
}

void Iec61107Component::handle_open_session_get_id_() {
  this->log_state_();

  if (received_frame_size_) {
    char *id = this->extract_meter_id_and_baud_(received_frame_size_);
    if (id == nullptr) {
      ESP_LOGE(TAG, "Meter identification frame invalid");
      this->stats_.invalid_frames_++;
      this->abort_mission_();
      return;
    }

    uint8_t baud_cmd[protocol::ACK_SET_BAUD_AND_MODE_FRAME_SIZE];
    size_t baud_cmd_len = protocol::build_ack_set_baud_and_mode_frame(
        baud_cmd, sizeof(baud_cmd), baud_rate_to_byte(this->baud_rate_negotiated_));

    this->send_frame_(baud_cmd, baud_cmd_len);
    this->update_last_rx_time_();
    if (this->are_baud_rates_different_()) {
      this->flush();
      this->set_next_state_delayed_(50, State::SET_BAUD);

    } else {
      this->set_next_state_(State::ACK_READ_P0_CONFIRMATION);
    }
  }
}

void Iec61107Component::handle_set_baud_() {
  this->log_state_();
  this->update_last_rx_time_();
  this->set_baud_rate_(this->baud_rate_negotiated_);
  this->set_next_state_delayed_(50, State::ACK_READ_P0_CONFIRMATION);
}

void Iec61107Component::handle_ack_read_p0_confirmation_() {
  auto read_fn = [this]() { return this->receive_prog_frame_(SOH); };
  this->read_reply_and_go_next_state_(read_fn, State::ACK_START_GET_INFO, 3, true, true);
}

void Iec61107Component::handle_ack_start_get_info_() {
  this->log_state_();

  if (received_frame_size_ == 0) {
    ESP_LOGE(TAG, "No response from meter.");
    this->stats_.invalid_frames_++;
    this->abort_mission_();
  }

  ValueRefsArray vals;
  char *in_param_ptr = (char *) &this->buffers_.in[1];
  if (!get_values_from_brackets_(in_param_ptr, vals)) {
    ESP_LOGE(TAG, "Invalid frame format. '%s', size = %d bytes", in_param_ptr, received_frame_size_);
    this->stats_.invalid_frames_++;
    this->abort_mission_();
    return;

    ESP_LOGD(TAG, "P0 Meter info: %s", vals[0]);
  }

  this->set_next_state_(this->programming_mode_required_ ? State::PROGRAMMING_MODE_REQ : State::MAIN_SESSION);
}

void Iec61107Component::handle_programming_mode_req_() {
  this->log_state_();
  this->prepare_prog_password_frame_(this->password_.c_str());
  this->send_frame_prepared_();
  this->update_last_rx_time_();
  auto read_fn = [this]() { return this->receive_frame_ack_nak_close_(); };
  this->read_reply_and_go_next_state_(read_fn, State::PROGRAMMING_MODE_ACK, 0, true, false);
  ESP_LOGD(TAG, "Requesting programming mode (P1)");
}

void Iec61107Component::handle_programming_mode_ack_() {
  this->log_state_();
  this->set_next_state_(State::MAIN_SESSION);

  if (received_frame_size_ == 0) {
    ESP_LOGW(TAG, "No response from meter after P1 request");
    this->stats_.invalid_frames_++;
    return;
  }
  char reply = this->buffers_.in[0];
  if (reply == ACK) {
    ESP_LOGD(TAG, "Programming mode acknowledged");
  } else if (reply == NAK) {
    ESP_LOGD(TAG, "Programming mode declined");
    this->abort_mission_();
  } else if (reply == SOH) {
    ESP_LOGD(TAG, "Programming mode declined, session closed");
    this->abort_mission_(false);
  } else {
    ESP_LOGD(TAG, "Programming mode failed");
    this->abort_mission_();
  }
}

void Iec61107Component::handle_main_session_() {
  if (this->time_to_set_ != 0) {
    this->set_next_state_(State::GET_DATE);
  } else {
    this->set_next_state_(State::DATA_ENQ);
  }
}

void Iec61107Component::handle_get_date_() {
  this->log_state_();
  this->update_last_rx_time_();

  this->meter_datetime_str_[0] = '\0';
  this->set_next_state_(State::GET_TIME);
  this->prepare_prog_frame_("DATE_()");
  this->send_frame_prepared_();
  auto read_fn = [this]() { return this->receive_prog_frame_(STX); };
  this->read_reply_and_go_next_state_(read_fn, State::GET_TIME, 3, false, true);
}

void Iec61107Component::handle_get_time_() {
  this->log_state_();
  this->update_last_rx_time_();

  if (received_frame_size_ != 22 && received_frame_size_ != 23) {
    ESP_LOGE(TAG, "No response or wrong response from meter. Can't get date, skipping sync.");
    this->stats_.invalid_frames_++;
    this->set_next_state_(State::DATA_ENQ);
    return;
  }

  char *in_param_ptr = (char *) &this->buffers_.in[1];
  size_t d = 23 - received_frame_size_;
  this->meter_datetime_str_[0] = '2';
  this->meter_datetime_str_[1] = '0';
  this->meter_datetime_str_[2] = in_param_ptr[d + 15];
  this->meter_datetime_str_[3] = in_param_ptr[d + 16];
  this->meter_datetime_str_[4] = '-';
  this->meter_datetime_str_[5] = in_param_ptr[d + 12];
  this->meter_datetime_str_[6] = in_param_ptr[d + 13];
  this->meter_datetime_str_[7] = '-';
  this->meter_datetime_str_[8] = in_param_ptr[d + 9];
  this->meter_datetime_str_[9] = in_param_ptr[d + 10];
  this->meter_datetime_str_[10] = ' ';
  this->meter_datetime_str_[11] = '\0';

  this->set_next_state_(State::CORRECT_TIME);
  this->prepare_prog_frame_("TIME_()");
  this->send_frame_prepared_();
  auto read_fn = [this]() { return this->receive_prog_frame_(STX); };
  this->read_reply_and_go_next_state_(read_fn, State::CORRECT_TIME, 3, false, true);
}

void Iec61107Component::handle_correct_time_() {
  this->log_state_();

  // do not care, not real point in this
  // auto time_received_delay_ms = millis() - this->last_rx_time_;

  this->update_last_rx_time_();
  this->set_next_state_(State::DATA_ENQ);

  char *in_param_ptr = (char *) &this->buffers_.in[1];

  // here we expect to receive time in format HH:MM:SS
  //      0         1         2         3
  //      01234567890123456789012345678901234567890
  // <STX>TIME_(23:40:10)<CR><LF><ETX><17> (20)

  if (received_frame_size_ != 20) {
    // no data or something wrong. error or malformed response
    ESP_LOGE(TAG, "No response or wrong response from meter. Can't get time, skipping sync.");
    this->stats_.invalid_frames_++;
    return;
  }
  memcpy(this->meter_datetime_str_ + 11, in_param_ptr + 6, 8);  // copy HH:MM:SS
  this->meter_datetime_str_[19] = '\0';                         // null-terminate the string

  ESPTime meter_datetime;
  meter_datetime.day_of_week = 1;
  meter_datetime.day_of_year = 1;
  int num = 0;
  {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    num = sscanf(this->meter_datetime_str_, "%04hu-%02hhu-%02hhu %02hhu:%02hhu:%02hhu",
                 &year,    // NOLINT
                 &month,   // NOLINT
                 &day,     // NOLINT
                 &hour,    // NOLINT
                 &minute,  // NOLINT
                 &second);
    meter_datetime.year = year;
    meter_datetime.month = month;
    meter_datetime.day_of_month = day;
    meter_datetime.hour = hour;
    meter_datetime.minute = minute;
    meter_datetime.second = second;
    meter_datetime.day_of_week = 1;
    meter_datetime.day_of_year = 1;  // not used, but set to avoid uninitialized value
  }
  meter_datetime.recalc_timestamp_local();
  if (num != 6) {
    ESP_LOGE(TAG, "Invalid time received from meter: %s %d", this->meter_datetime_str_, num);
    this->stats_.invalid_frames_++;
    return;
  }

  if (!meter_datetime.is_valid()) {
    ESP_LOGE(TAG, "Invalid time received from meter: %s", this->meter_datetime_str_);
    this->stats_.invalid_frames_++;
    return;
  }

  // check if time is 2 mins before or after midnight, then skip correction, wait for next data request
  // just to make sure we have proper date/time
  if ((meter_datetime.hour == 0 && meter_datetime.minute < 2) || (meter_datetime.hour == 23 && meter_datetime.minute > 58)) {
    ESP_LOGD(TAG, "Time is too close to midnight, skipping correction.");
    return;
  }

  auto now_ms = millis();
#ifdef USE_TIME
  if (this->time_source_ != nullptr) {
    auto tm = this->time_source_->now();
    if (!tm.is_valid()) {
      ESP_LOGE(TAG, "Time sync requested, but time provider is not yet ready");
      return;
    }
    this->time_to_set_ = this->time_source_->now().timestamp;
    this->time_to_set_requested_at_ms_ = now_ms;
  }
#endif
  // if we are here, we have a valid time
  // find what is real time now
  uint32_t ms_since_asked = now_ms - this->time_to_set_requested_at_ms_;

  meter_datetime.recalc_timestamp_local();
  int32_t correction_seconds = (this->time_to_set_ + ms_since_asked / 1000) - meter_datetime.timestamp;

  this->time_to_set_ = 0;
  this->time_to_set_requested_at_ms_ = 0;

  if (correction_seconds > -2 && correction_seconds < 2) {
    ESP_LOGD(TAG, "No time correction needed (less than 2 seconds)");
    return;
  }

  ESP_LOGD(TAG, "Time correction needed: %d seconds", correction_seconds);

  constexpr int32_t SECONDS_IN_24H = 24 * 3600;

  // if correction is more than 24 hours,
  // it is serious meter failure, meter shall be replaced or time is completely wrong
  if (correction_seconds > SECONDS_IN_24H || correction_seconds < -SECONDS_IN_24H) {
    ESP_LOGE(TAG, "Time correction is more than 24 hours, meter is broken or time is completely wrong.");
    return;
  }

  if (correction_seconds > 29) {
    correction_seconds = 29;
  } else if (correction_seconds < -29) {
    correction_seconds = -29;
  }
  ESP_LOGD(TAG, "Setting time correction within +/- 29 seconds: %d", correction_seconds);

  char set_time_cmd[16]{0};
  size_t len = snprintf(set_time_cmd, sizeof(set_time_cmd), "CTIME(%d)", correction_seconds);
  (void) len;
  this->prepare_prog_frame_(set_time_cmd, true);
  this->send_frame_prepared_();
  auto read_fn = [this]() { return this->receive_frame_ack_nak_(); };
  this->read_reply_and_go_next_state_(read_fn, State::RECV_CORRECTION_RESULT, 0, false, false);
}

void Iec61107Component::handle_recv_correction_result_() {
  this->log_state_();
  this->set_next_state_(State::DATA_ENQ);

  if (received_frame_size_ == 0) {
    ESP_LOGW(TAG, "No response from meter after time correction request. Not supported?");
    this->stats_.invalid_frames_++;
    return;
  }
  char reply = this->buffers_.in[0];
  if (reply == ACK) {
    ESP_LOGD(TAG, "Time correction acknowledged");
  } else if (reply == NAK) {
    ESP_LOGD(TAG, "Time correction declined");
  } else {
    ESP_LOGD(TAG, "Time correction failed");
  }
}

void Iec61107Component::handle_data_enq_() {
  this->log_state_();
  if (this->loop_state_.request_iter == this->sensors_.end()) {
    ESP_LOGD(TAG, "All requests done");
    this->set_next_state_(State::CLOSE_SESSION);
    return;
  }

  auto req = this->loop_state_.request_iter->first;
  ESP_LOGD(TAG, "Request '%s'", req.c_str());
  this->prepare_prog_frame_(req.c_str());
  this->send_frame_prepared_();
  auto read_fn = [this]() { return this->receive_prog_frame_(STX); };
  this->read_reply_and_go_next_state_(read_fn, State::DATA_RECV, 3, false, true);
}

void Iec61107Component::handle_data_recv_() {
  this->log_state_();
  this->set_next_state_(State::DATA_NEXT);

  if (received_frame_size_ == 0) {
    ESP_LOGD(TAG, "Response not received or corrupted. Next.");
    this->update_last_rx_time_();
    this->clear_rx_buffers_();
    return;
  }

  auto req = this->loop_state_.request_iter->first;
  ValueRefsArray vals;
  char *in_param_ptr = (char *) &this->buffers_.in[1];

  uint8_t brackets_found = get_values_from_brackets_(in_param_ptr, vals);
  if (!brackets_found) {
    ESP_LOGE(TAG, "Invalid frame format: '%s'. Received no data groups.", in_param_ptr);
    this->stats_.invalid_frames_++;
    return;
  }

  switch (brackets_found) {
    case 1: {
      ESP_LOGV(TAG, "Received name: '%s', 1 block: (%s)", in_param_ptr, vals[0]);
    } break;
    case 2: {
      ESP_LOGV(TAG, "Received name: '%s', %d blocks: #1(%s), #2(%s)", in_param_ptr, brackets_found, vals[0], vals[1]);
    } break;
    case 3: {
      ESP_LOGV(TAG, "Received name: '%s', %d blocks: #1(%s), #2(%s), #3(%s)", in_param_ptr, brackets_found, vals[0],
               vals[1], vals[2]);
    } break;
    case 4: {
      ESP_LOGV(TAG, "Received name: '%s', %d blocks: #1(%s), #2(%s), #3(%s), #4(%s)", in_param_ptr, brackets_found,
               vals[0], vals[1], vals[2], vals[3]);
    } break;
    default: {
      ESP_LOGV(TAG,
               "Received name: '%s', %d blocks: #1(%s), #2(%s), #3(%s), #4(%s), #5(%s), #6(%s), #7(%s), #8(%s), #9(%s), "
               "#10(%s), #11(%s), #12(%s)",
               in_param_ptr, brackets_found, vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7],
               vals[8], vals[9], vals[10], vals[11]);
    }
  }

  if (in_param_ptr[0] == '\0') {
    if (vals[0][0] != '\0') {
      ESP_LOGW(TAG, "Request '%s' either not supported or malformed. Error returned: '%s'", req.c_str(), vals[0]);
    } else {
      ESP_LOGW(TAG, "Request '%s' either not supported or malformed.", req.c_str());
    }
    return;
  }

  if (this->loop_state_.request_iter->second->get_function() != in_param_ptr) {
    ESP_LOGW(TAG, "Returned data name mismatch. Requested '%s', Received '%s'. Skipping frame.", req.c_str(), in_param_ptr);
    return;
  }

  auto range = sensors_.equal_range(req);
  for (auto it = range.first; it != range.second; ++it) {
    if (!it->second->is_failed())
      set_sensor_value_(it->second, vals);
  }
}

void Iec61107Component::handle_data_next_() {
  this->log_state_();
  this->loop_state_.request_iter = this->sensors_.upper_bound(this->loop_state_.request_iter->first);
  if (this->loop_state_.request_iter != this->sensors_.end()) {
    this->set_next_state_delayed_(this->delay_between_requests_ms_, State::DATA_ENQ);
  } else {
    this->set_next_state_delayed_(this->delay_between_requests_ms_, State::CLOSE_SESSION);
  }
}

void Iec61107Component::handle_close_session_() {
  this->log_state_();
  ESP_LOGD(TAG, "Closing session");
  uint8_t close_cmd[protocol::CLOSE_SESSION_FRAME_SIZE];
  size_t close_cmd_len =
      protocol::build_close_session_frame(close_cmd, sizeof(close_cmd), this->crc_method_ == IecCrcMethod::CRC_XOR);
  this->send_frame_(close_cmd, close_cmd_len);
  if (this->are_baud_rates_different_()) {
    this->set_baud_rate_(this->baud_rate_handshake_);
  }
  this->set_next_state_(State::PUBLISH);

  ESP_LOGD(TAG, "Total connection time: %u ms", millis() - this->loop_state_.session_started_ms);
  this->loop_state_.sensor_iter = this->sensors_.begin();
}

void Iec61107Component::handle_publish_() {
  this->log_state_();
  ESP_LOGV(TAG, "Publishing data");
  this->update_last_rx_time_();

  if (this->loop_state_.sensor_iter != this->sensors_.end()) {
    this->loop_state_.sensor_iter->second->publish();
    this->loop_state_.sensor_iter++;
  } else {
    this->stats_dump_();
    if (this->crc_errors_per_session_sensor_ != nullptr) {
      this->crc_errors_per_session_sensor_->publish_state(this->stats_.crc_errors_per_session());
    }
    this->report_failure(false);
    this->unlock_uart_session_();
    this->set_next_state_(State::IDLE);
  }
}

void Iec61107Component::handle_single_read_() {
  // Reserved for future single-read FSM path; keep current behavior as no-op.
}

void Iec61107Component::handle_single_read_ack_() {
  this->log_state_();
  if (received_frame_size_) {
    ESP_LOGD(TAG, "Single read frame received");
  } else {
    ESP_LOGE(TAG, "Failed to make single read call");
  }
  this->set_next_state_(State::IDLE);
}

void Iec61107Component::update() {
  if (this->state_ != State::IDLE) {
    ESP_LOGD(TAG, "Starting data collection impossible - component not ready");
    return;
  }
  ESP_LOGD(TAG, "Starting data collection");
  this->set_next_state_(State::TRY_LOCK_BUS);
}

void Iec61107Component::queue_single_read(const std::string &request) {
  ESP_LOGD(TAG, "Queueing single read for '%s'", request.c_str());
  this->single_requests_.push_back(request);
}

#ifdef USE_TIME
void Iec61107Component::sync_device_time() {
  if (this->time_source_ == nullptr) {
    ESP_LOGE(TAG, "Time source not set. Time can not be synced.");
    return;
  }
  auto time = this->time_source_->now();
  if (!time.is_valid()) {
    ESP_LOGW(TAG, "Time is not yet valid.  Time can not be synced.");
    return;
  }
  this->set_device_time(1);
}
#endif

void Iec61107Component::set_device_time(uint32_t timestamp) {
  ESP_LOGD(TAG, "set_device_time: %u", timestamp);
  if (!timestamp)
    return;
  this->time_to_set_ = timestamp;
  this->time_to_set_requested_at_ms_ = millis();
}

bool Iec61107Component::set_sensor_value_(Iec61107SensorBase *sensor, ValueRefsArray &vals) {
  auto type = sensor->get_type();
  bool ret = true;

  uint8_t idx = sensor->get_index() - 1;
  if (idx >= VAL_NUM) {
    ESP_LOGE(TAG, "Invalid sensor index %u", idx);
    return false;
  }
  char str_buffer[128] = {'\0'};
  strncpy(str_buffer, vals[idx], 128);

  char *str = str_buffer;
  uint8_t sub_idx = sensor->get_sub_index();
  if (sub_idx == 0) {
    ESP_LOGD(TAG, "Got for '%s' (idx = %d) : '%s'", sensor->get_request().c_str(), idx + 1, str);
  } else {
    str = this->get_nth_value_from_csv_(str, sub_idx);
    if (str == nullptr) {
      ESP_LOGE(TAG, "Failed for '%s' (idx = %d, sub_idx = %d)", sensor->get_request().c_str(), idx + 1, sub_idx);
      ESP_LOGE(TAG, "Cannot extract sensor value by sub-index. Is data comma-separated? "
                    "Also note that sub-index starts from 1");
      str_buffer[0] = '\0';
      str = str_buffer;
    }
    ESP_LOGD(TAG, "Got for '%s' (idx = %d, sub_idx = %d) : '%s'", sensor->get_request().c_str(), idx + 1, sub_idx, str);
  }

  if (type == SensorType::SENSOR) {
    float f = 0;
    // todo: for non-energomeras... value can be "100.0" or "100.0*kWh" or "100.0#A"
    ret = str && str[0] && protocol::char2float(str, f);
    if (ret) {
      static_cast<Iec61107Sensor *>(sensor)->set_value(f);
    } else {
      ESP_LOGE(TAG, "Cannot convert incoming data to a number. Consider using a text sensor. Invalid data: '%s'", str);
    }
  } else {
#ifdef USE_TEXT_SENSOR
    static_cast<Iec61107TextSensor *>(sensor)->set_value(str);
#endif
  }
  return ret;
}

bool Iec61107Component::check_crc_prog_frame_(uint8_t *data, size_t length) {
  uint8_t crc = protocol::calculate_crc_prog_frame(data, length, false, this->crc_method_ == IecCrcMethod::CRC_XOR);
  if (crc != data[length - 1]) {
    ESP_LOGD(TAG, "CRC Mismatch. Expected 0x%02X, Received 0x%02X", crc, data[length - 1]);
  }
  return crc == data[length - 1];
}

void Iec61107Component::set_next_state_delayed_(uint32_t ms, State next_state) {
  if (ms == 0) {
    set_next_state_(next_state);
  } else {
    ESP_LOGV(TAG, "Short delay for %u ms", ms);
    set_next_state_(State::WAIT);
    wait_.start_time = millis();
    wait_.delay_ms = ms;
    wait_.next_state = next_state;
  }
}

void Iec61107Component::read_reply_and_go_next_state_(ReadFunction read_fn, State next_state, uint8_t retries,
                                                      bool mission_critical, bool check_crc) {
  reading_state_ = {};
  reading_state_.read_fn = read_fn;
  reading_state_.mission_critical = mission_critical;
  reading_state_.tries_max = retries;
  reading_state_.tries_counter = 0;
  reading_state_.check_crc = check_crc;
  reading_state_.next_state = next_state;
  received_frame_size_ = 0;

  set_next_state_(State::WAITING_FOR_RESPONSE);
}

void Iec61107Component::prepare_prog_password_frame_(const char *password) {
  this->buffers_.amount_out = protocol::build_prog_password_frame(
      this->buffers_.out, MAX_OUT_BUF_SIZE, password, this->crc_method_ == IecCrcMethod::CRC_XOR);
}

void Iec61107Component::prepare_prog_frame_(const char *request, bool write) {
  this->buffers_.amount_out = protocol::build_prog_frame(
      this->buffers_.out, MAX_OUT_BUF_SIZE, request, write, this->crc_method_ == IecCrcMethod::CRC_XOR);
}

void Iec61107Component::prepare_non_session_prog_frame_(const char *request) {
  this->buffers_.amount_out =
      protocol::build_non_session_prog_frame(this->buffers_.out, MAX_OUT_BUF_SIZE, this->meter_address_.c_str(),
                                             request, this->crc_method_ == IecCrcMethod::CRC_XOR);
}

void Iec61107Component::prepare_ctime_frame_(uint8_t hh, uint8_t mm, uint8_t ss) {
  this->buffers_.amount_out = protocol::build_ctime_frame(this->buffers_.out, MAX_OUT_BUF_SIZE, hh, mm, ss);
}

void Iec61107Component::send_frame_prepared_() {
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  this->received_frame_size_ = 0;
  this->write_array(this->buffers_.out, this->buffers_.amount_out);
  this->flush();

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);

  ESP_LOGV(TAG, "TX: %s", format_frame_pretty(this->buffers_.out, this->buffers_.amount_out).c_str());
  ESP_LOGVV(TAG, "TX: %s", format_hex_pretty(this->buffers_.out, this->buffers_.amount_out).c_str());
}

void Iec61107Component::prepare_frame_(const uint8_t *data, size_t length) {
  memcpy(this->buffers_.out, data, length);
  this->buffers_.amount_out = length;
}

void Iec61107Component::send_frame_(const uint8_t *data, size_t length) {
  this->prepare_frame_(data, length);
  this->send_frame_prepared_();
}

size_t Iec61107Component::receive_frame_(FrameStopFunction stop_fn) {
  const uint32_t read_time_limit_ms = 40;  // 25;
  size_t ret_val;

  auto count = this->available();
  if (count <= 0)
    return 0;

  uint32_t read_start = millis();
  uint8_t *p;
  while (count-- > 0) {
    if (millis() - read_start > read_time_limit_ms) {
      return 0;
    }

    if (this->buffers_.amount_in < MAX_IN_BUF_SIZE) {
      p = &this->buffers_.in[this->buffers_.amount_in];
      if (!iuart_->read_one_byte(p)) {
        return 0;
      }
      this->buffers_.amount_in++;
    } else {
      memmove(this->buffers_.in, this->buffers_.in + 1, this->buffers_.amount_in - 1);
      p = &this->buffers_.in[this->buffers_.amount_in - 1];
      if (!iuart_->read_one_byte(p)) {
        return 0;
      }
    }

    if (stop_fn(this->buffers_.in, this->buffers_.amount_in)) {
      ESP_LOGV(TAG, "RX: %s", format_frame_pretty(this->buffers_.in, this->buffers_.amount_in).c_str());
      ESP_LOGVV(TAG, "RX: %s", format_hex_pretty(this->buffers_.in, this->buffers_.amount_in).c_str());
      ret_val = this->buffers_.amount_in;
      this->buffers_.amount_in = 0;
      this->update_last_rx_time_();
      return ret_val;
    }

    yield();
    App.feed_wdt();
  }
  return 0;
}

size_t Iec61107Component::receive_frame_ascii_() {
  // "data<CR><LF>"
  ESP_LOGVV(TAG, "Waiting for ASCII frame");
  auto frame_end_check_crlf = [this](uint8_t *b, size_t s) {
    auto ret = protocol::frame_end_ascii_crlf(b, s);
    if (ret) {
      ESP_LOGVV(TAG, "Frame CRLF Stop");
    }
    return ret;
  };
  return receive_frame_(frame_end_check_crlf);
}

size_t Iec61107Component::receive_frame_ack_nak_() {
  // "<ACK/NAK>"
  //  ESP_LOGVV(TAG, "Waiting for ACK/NAK frame");
  auto frame_end_check_ack_nak = [this](uint8_t *b, size_t s) {
    auto ret = protocol::frame_end_ack_or_nak(b, s, ACK, NAK);
    if (ret) {
      if (b[0] == ACK) {
        ESP_LOGVV(TAG, "Frame ACK Stop");
      } else {
        ESP_LOGVV(TAG, "Frame NAK Stop");
      }
    }
    return ret;
  };
  return receive_frame_(frame_end_check_ack_nak);
}

size_t Iec61107Component::receive_frame_ack_nak_close_() {
  // "<ACK>", "<NAK>" or "<SOH>B0<ETX><BCC>"
  auto frame_end_check_ack_nack_close = [this](uint8_t *b, size_t s) {
    auto ret = protocol::frame_end_ack_nak_or_close(b, s, ACK, NAK, this->crc_method_ == IecCrcMethod::CRC_XOR);
    if (ret) {
      if (b[0] == ACK) {
        ESP_LOGVV(TAG, "Frame ACK Stop");
      } else if (b[0] == NAK) {
        ESP_LOGVV(TAG, "Frame NAK Stop");
      } else {
        ESP_LOGVV(TAG, "Frame CLOSE SESSION Stop");
      }
    }
    return ret;
  };
  return receive_frame_(frame_end_check_ack_nack_close);
}

size_t Iec61107Component::receive_prog_frame_(uint8_t start_byte, bool accept_ack_and_nack) {
  // "<start_byte>data<ETX><BCC>"
  //  ESP_LOGVV(TAG, "Waiting for R1 frame, start byte: 0x%02x", start_byte);
  auto frame_end_check_iec = [this, start_byte, accept_ack_and_nack](uint8_t *b, size_t s) {
    auto ret = protocol::frame_end_prog_reply(b, s, start_byte, accept_ack_and_nack, ACK, NAK, ETX);
    if (ret) {
      if (s == 1 && b[0] == ACK) {
        ESP_LOGVV(TAG, "Frame ACK Stop");
      } else if (s == 1 && b[0] == NAK) {
        ESP_LOGVV(TAG, "Frame NAK Stop");
      } else {
        ESP_LOGVV(TAG, "Frame ETX Stop");
      }
    }
    return ret;
  };
  return receive_frame_(frame_end_check_iec);
}

void Iec61107Component::clear_rx_buffers_() {
  int available = this->available();
  if (available > 0) {
    ESP_LOGVV(TAG, "Cleaning garbage from UART input buffer: %d bytes", available);
  }

  int len;
  while (available > 0) {
    len = std::min(available, (int) MAX_IN_BUF_SIZE);
    this->read_array(this->buffers_.in, len);
    available -= len;
  }
  memset(this->buffers_.in, 0, MAX_IN_BUF_SIZE);
  this->buffers_.amount_in = 0;
}

char *Iec61107Component::extract_meter_id_and_baud_(size_t frame_size) {
  auto parsed = protocol::parse_identification_frame(this->buffers_.in, frame_size, MAX_IN_BUF_SIZE);
  if (parsed.ident == nullptr) {
    return nullptr;
  }

  ESP_LOGD(TAG, "Meter identification string: '%s'", parsed.ident);
  this->iec_manufacturer_.assign(parsed.ident + 1, 3);
  this->iec_device_.assign(parsed.ident + 5);
  ESP_LOGD(TAG, " - Manufacturer: '%s', Device: '%s'", this->iec_manufacturer_.c_str(), this->iec_device_.c_str());

  if (!parsed.type_c) {
    ESP_LOGE(TAG, "Meter is not Type C (IEC-61107/IEC-62056). Reported baud code = '%c'", parsed.baud_code);
    this->status_set_error(LOG_STR("Meter is not Type C (IEC-61107/IEC-62056)"));
    return nullptr;
  }

  ESP_LOGD(TAG, " - Supported baud rate: %d (code '%c')", parsed.baud_rate, parsed.baud_code);
  this->baud_rate_negotiated_ = parsed.baud_rate;
  return parsed.ident;
}

uint8_t Iec61107Component::get_values_from_brackets_(char *line, ValueRefsArray &vals) {
  return protocol::get_values_from_brackets(line, vals, empty_str);
}

// Get N-th value from comma-separated string, 1-based index
// line = "20.08.24,0.45991"
// get_nth_value_from_csv_(line, 1) -> "20.08.24"
// get_nth_value_from_csv_(line, 2) -> "0.45991"
char *Iec61107Component::get_nth_value_from_csv_(char *line, uint8_t idx) {
  return protocol::get_nth_value_from_csv(line, idx);
}

const char *Iec61107Component::state_to_string(State state) {
  switch (state) {
    case State::NOT_INITIALIZED:
      return "NOT_INITIALIZED";
    case State::IDLE:
      return "IDLE";
    case State::TRY_LOCK_BUS:
      return "TRY_LOCK_BUS";
    case State::WAIT:
      return "WAIT";
    case State::WAITING_FOR_RESPONSE:
      return "WAITING_FOR_RESPONSE";
    case State::OPEN_SESSION:
      return "OPEN_SESSION";
    case State::OPEN_SESSION_GET_ID:
      return "OPEN_SESSION_GET_ID";
    case State::SET_BAUD:
      return "SET_BAUD";
    case State::ACK_READ_P0_CONFIRMATION:
      return "ACK_READ_P0_CONFIRMATION";
    case State::ACK_START_GET_INFO:
      return "ACK_START_GET_INFO";
    case State::PROGRAMMING_MODE_REQ:
      return "PROGRAMMING_MODE_REQ";
    case State::PROGRAMMING_MODE_ACK:
      return "PROGRAMMING_MODE_ACK";
    case State::MAIN_SESSION:
      return "MAIN_SESSION";
    case State::GET_DATE:
      return "GET_DATE";
    case State::GET_TIME:
      return "GET_TIME";
    case State::CORRECT_TIME:
      return "CORRECT_TIME";
    case State::RECV_CORRECTION_RESULT:
      return "RECV_CORRECTION_RESULT";
    case State::DATA_ENQ:
      return "DATA_ENQ";
    case State::DATA_RECV:
      return "DATA_RECV";
    case State::DATA_NEXT:
      return "DATA_NEXT";
    case State::CLOSE_SESSION:
      return "CLOSE_SESSION";
    case State::PUBLISH:
      return "PUBLISH";
    case State::SINGLE_READ_ACK:
      return "SINGLE_READ_ACK";
    default:
      return "UNKNOWN";
  }
}

void Iec61107Component::log_state_(State *next_state) {
  if (this->state_ != this->last_reported_state_) {
    if (next_state == nullptr) {
      ESP_LOGV(TAG, "State::%s", this->state_to_string(this->state_));
    } else {
      ESP_LOGV(TAG, "State::%s -> %s", this->state_to_string(this->state_), this->state_to_string(*next_state));
    }
    this->last_reported_state_ = this->state_;
  }
}

void Iec61107Component::stats_dump_() {
  ESP_LOGV(TAG, "============================================");
  ESP_LOGV(TAG, "Data collection and publishing finished.");
  ESP_LOGV(TAG, "Total number of sessions ............. %u", this->stats_.connections_tried_);
  ESP_LOGV(TAG, "Total number of invalid frames ....... %u", this->stats_.invalid_frames_);
  ESP_LOGV(TAG, "Total number of CRC errors ........... %u", this->stats_.crc_errors_);
  ESP_LOGV(TAG, "Total number of CRC errors recovered . %u", this->stats_.crc_errors_recovered_);
  ESP_LOGV(TAG, "CRC errors per session ............... %f", this->stats_.crc_errors_per_session());
  ESP_LOGV(TAG, "Number of failures ................... %u", this->stats_.failures_);
  ESP_LOGV(TAG, "============================================");
}

bool Iec61107Component::try_lock_uart_session_() {
  if (AnyObjectLocker::try_lock(this->parent_)) {
    ESP_LOGVV(TAG, "UART bus %p locked by %s", this->parent_, this->tag_.c_str());
    return true;
  }
  ESP_LOGVV(TAG, "UART bus %p busy", this->parent_);
  return false;
}

void Iec61107Component::unlock_uart_session_() {
  AnyObjectLocker::unlock(this->parent_);
  ESP_LOGVV(TAG, "UART bus %p released by %s", this->parent_, this->tag_.c_str());
}

uint8_t Iec61107Component::next_obj_id_ = 0;

std::string Iec61107Component::generateTag() { return str_sprintf("%s%03d", TAG0, ++next_obj_id_); }

}  // namespace iec61107
}  // namespace esphome
