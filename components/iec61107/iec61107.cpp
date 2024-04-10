#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/application.h"
#include "iec61107.h"

namespace esphome {
namespace iec61107 {

static const char *TAG = "iec61107";

static constexpr uint8_t SOH = 0x01;
static constexpr uint8_t STX = 0x02;
static constexpr uint8_t ETX = 0x03;
static constexpr uint8_t EOT = 0x04;
static constexpr uint8_t ENQ = 0x05;
static constexpr uint8_t ACK = 0x06;
static constexpr uint8_t CR = 0x0D;
static constexpr uint8_t LF = 0x0A;
static constexpr uint8_t NAK = 0x15;

static const uint8_t CMD_OPEN_SESSION[] = {0x2f, 0x3f, 0x21, 0x0d, 0x0a};
static const uint8_t CMD_ACK_SET_BAUD_AND_MODE[] = {ACK, '0', '5',
                                                    '1', CR,  LF};  // [2] = 5 = 9600, [3] 1 = one-by-one, 0 - readout
static const uint8_t CMD_CLOSE_SESSION[] = {SOH, 0x42, 0x30, ETX, 0x75};

static constexpr uint8_t BOOT_WAIT_S = 10;

enum ProtocolMode { PROTOCOL_MODE_A = 'A', PROTOCOL_MODE_B = 'B', PROTOCOL_MODE_C = 'C', PROTOCOL_MODE_D = 'D' };
const uint32_t BAUDRATES[] = {300, 600, 1200, 2400, 4800, 9600, 19200};
#define MAX_BAUDRATE (BAUDRATES[sizeof(BAUDRATES) / sizeof(uint32_t) - 1])
#define PROTO_B_MIN_BAUDRATE (BAUDRATES[1])
static const char PROTO_B_RANGE_BEGIN = 'A';
static const char PROTO_B_RANGE_END = 'F';
static const char PROTO_C_RANGE_BEGIN = '0';
static const char PROTO_C_RANGE_END = '6';

void IEC61107Component::setup() {
  ESP_LOGD(TAG, "setup");
#ifdef USE_ESP32_FRAMEWORK_ARDUINO
  iuart_ = make_unique<IEC61107UART>(*static_cast<uart::ESP32ArduinoUARTComponent *>(this->parent_));
#endif

#ifdef USE_ESP_IDF
  iuart_ = make_unique<IEC61107UART>(*static_cast<uart::IDFUARTComponent *>(this->parent_));
#endif

#if USE_ESP8266
  iuart_ = make_unique<IEC61107UART>(*static_cast<uart::ESP8266UartComponent *>(this->parent_));
#endif
  this->clear_uart_input_buffer_();
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
  }

  this->set_timeout(BOOT_WAIT_S * 1000, [this]() {
    ESP_LOGD(TAG, "Boot timeout, component is ready to use");
    this->clear_uart_input_buffer_();
    this->set_next_state_(State::IDLE);
  });
}

void IEC61107Component::dump_config() {
  ESP_LOGCONFIG(TAG, "IEC61107:");
  LOG_UPDATE_INTERVAL(this);
  //   ESP_LOGCONFIG(TAG, "  Flow Control Pin: %s",
  //                 this->flow_control_pin_ ? this->flow_control_pin_->to_string().c_str() : "N/A");
  ESP_LOGCONFIG(TAG, "  Receive Timeout: %ums", this->receive_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Meter Type: CE102M");
  ESP_LOGCONFIG(TAG, "  Sensors:");
  for (const auto &sensors : sensors_) {
    auto &s = sensors.second;
    ESP_LOGCONFIG(TAG, "    REQUEST: %s", s->get_request());
  }
}

void IEC61107Component::register_sensor(IEC61107SensorBase *sensor) {
  this->sensors_.insert({sensor->get_request(), sensor});
  this->requests_.insert(sensor->get_request());
}

void IEC61107Component::abort_mission_() {
  // try close connection ?
  ESP_LOGD(TAG, "Closing session");
  this->send_frame_(CMD_CLOSE_SESSION, sizeof(CMD_CLOSE_SESSION));
  this->set_next_state_(State::IDLE);
  this->report_failure(true);
}

void IEC61107Component::report_failure(bool set_or_clear) {
  if (set_or_clear) {
    if (this->indicator_ != nullptr) {
      this->indicator_->publish_state(true);
    }

    number_of_failures_++;
    if (number_of_failures_ > this->number_of_failures_before_reboot_ && this->number_of_failures_before_reboot_ != 0) {
      ESP_LOGE(TAG, "Too many failures. Let's try rebooting device.");
      delay(100);
      App.safe_reboot();
    }
  } else {
    number_of_failures_ = 0;
    if (this->indicator_ != nullptr) {
      this->indicator_->publish_state(false);
    }
  }
}

void IEC61107Component::loop() {
  if (!this->is_ready() || this->state_ == State::NOT_INITIALIZED)
    return;
  const uint32_t now = millis();
  static uint32_t started_ms{0};
  // uint32_t baud_rate = 4800;

  static auto req_iterator = this->requests_.end();
  static auto sens_iterator = this->sensors_.end();

  if (!this->is_idling() && now - this->last_transmission_from_meter_timestamp_ >= receive_timeout_ms_) {
    ESP_LOGE(TAG, "No transmission from the meter.");
    // shall we retry?
    if (this->state_ == State::DATA_RECV) {
      this->set_next_state_(State::DATA_FAIL);
    } else {
      this->abort_mission_();
      return;
    }
  }

  static char param_buff[32];
  static ValuesArray vals;
  size_t frame_size;

  switch (this->state_) {
    case State::IDLE:
      this->update_last_transmission_from_meter_timestamp_();
      break;

    case State::WAIT:
      if (this->check_wait_period_()) {
        this->set_next_state_(this->next_state_after_wait_);
      }
      this->update_last_transmission_from_meter_timestamp_();
      break;

    case State::OPEN_SESSION:
      started_ms = millis();
      this->report_state_();
      this->clear_uart_input_buffer_();
      // iuart_->update_baudrate(300);
      this->send_frame_(CMD_OPEN_SESSION, sizeof(CMD_OPEN_SESSION));
      this->set_next_state_(State::OPEN_SESSION_GET_ID);
      req_iterator = this->requests_.begin();
      break;

    case State::OPEN_SESSION_GET_ID:
      this->report_state_();
      if ((frame_size = this->receive_frame_())) {
        char *packet = this->get_id_(frame_size);

        if (packet == nullptr) {
          ESP_LOGE(TAG, "Invalid meter identification frame");
          this->abort_mission_();
          return;
        }

        baud_rate_identification_ = frame_size >= 5 ? packet[4] : 0;
        if (baud_rate_identification_ >= PROTO_B_RANGE_BEGIN && baud_rate_identification_ <= PROTO_B_RANGE_END) {
          mode_ = PROTOCOL_MODE_B;
        } else if (baud_rate_identification_ >= PROTO_C_RANGE_BEGIN && baud_rate_identification_ <= PROTO_C_RANGE_END) {
          mode_ = PROTOCOL_MODE_C;
        } else {
          mode_ = PROTOCOL_MODE_A;
        }

        ESP_LOGD(TAG, "Meter reported protocol: %c", (char) mode_);
        if (mode_ != PROTOCOL_MODE_A) {
          ESP_LOGD(TAG, "Meter reported max baud rate: %u bps ('%c')",
                   identification_to_baud_rate_(baud_rate_identification_), baud_rate_identification_);
        }

        auto negotiated_bps = identification_to_baud_rate_(baud_rate_identification_);
        if (mode_ != PROTOCOL_MODE_C) {
          ESP_LOGE(TAG, "Unsupported protocol mode '%c'.", (char) mode_);
          this->abort_mission_();
          return;
        }
        this->prepare_frame_(CMD_ACK_SET_BAUD_AND_MODE, sizeof(CMD_ACK_SET_BAUD_AND_MODE));
        this->out_buf_[2] = '5';  // 9600 // this->baud_rate_identification_;
        this->out_buf_[3] = '1';  // this->readout_mode_ ? '0' : '1';
        this->send_frame_();

        this->set_next_state_delayed_(this->delay_between_requests_ms_, State::ACK_START_GET_INFO);  // SET_BAUD_RATE
      }
      break;

    case State::SET_BAUD_RATE:
      // baud_rate = identification_to_baud_rate_(this->baud_rate_identification_);
      // ESP_LOGV(TAG, "Baudrate set to: %u bps", baud_rate);
      // iuart_->update_baudrate(baud_rate);

      this->set_next_state_(this->readout_mode_ ? State::READOUT : State::ACK_START_GET_INFO);
      break;

    case State::ACK_START_GET_INFO:
      this->report_state_();
      if ((frame_size = this->receive_frame_())) {
        if (!get_values_from_brackets_((char *) in_buf_, vals)) {
          ESP_LOGE(TAG, "Invalid frame format: '%s'", in_buf_);
          break;
        }

        this->set_next_state_delayed_(this->delay_between_requests_ms_, State::DATA_ENQ);
      }
      break;

    case State::READOUT:
      this->report_state_();
      if ((frame_size = this->receive_frame_())) {
        if (in_buf_[0] == ETX) {
          ESP_LOGD(TAG, "ETX Received");
          // bcc_ ^= ETX;  // faster than update_bcc_(in_buf_,1);
          update_bcc_(in_buf_, 1);

          bool bcc_failed = false;
          if (bcc_ == in_bcc_) {
            ESP_LOGD(TAG, "BCC verification is OK");
          } else {
            ESP_LOGE(TAG, "BCC verification failed. Expected 0x%02x, got 0x%02x", bcc_, in_bcc_);
            bcc_failed = true;
          }

          this->set_next_state_(State::CLOSE_SESSION);
        } else {
          update_bcc_(in_buf_, frame_size);
          if (in_buf_[0] == '!' && frame_size == 3) {
            ESP_LOGD(TAG, "End of readout marker received");
            this->update_last_transmission_from_meter_timestamp_();
            break;
          }

          if (frame_size > 2)
            in_buf_[frame_size - 2] = 0;

          ESP_LOGD(TAG, "Data received: '%s' (%d)", in_buf_, frame_size);

          if (!get_values_from_brackets_((char *) in_buf_, vals)) {
            ESP_LOGE(TAG, "Invalid frame format: '%s'", in_buf_);
            break;
          }

          // ESP_LOGD(TAG, "Data received: param '%s', value1 '%s', value2 '%s', value3 '%s', value4 '%s'",
          //          param_name.c_str(), vals[0].c_str(), vals[1].c_str(), vals[2].c_str(), vals[3].c_str());
          //          std::string param = std::string(param_name) + "()";
          param_buff[31] = 0;
          snprintf(param_buff, 31, "%s()", in_buf_);

          // Update all matching sensors
          auto range = sensors_.equal_range(param_buff);
          for (auto it = range.first; it != range.second; ++it) {
            if (!it->second->is_failed())
              set_sensor_value_(it->second, vals);
          }

          // Update all matching sensors
          this->update_last_transmission_from_meter_timestamp_();
        }
      }
      break;

    case State::DATA_ENQ:
      this->report_state_();
      if (req_iterator == this->requests_.end()) {
        ESP_LOGD(TAG, "All requests done");
        this->set_next_state_(State::CLOSE_SESSION);
        break;
      } else {
        ESP_LOGD(TAG, "Requesting data for '%s'", *req_iterator);
        this->prepare_request_frame_(*req_iterator);
        this->send_frame_();
        this->set_next_state_delayed_(this->delay_between_requests_ms_, State::DATA_RECV);
      }
      break;

    case State::DATA_RECV:
      this->report_state_();
      if ((frame_size = this->receive_frame_())) {
        ESP_LOGD(TAG, "Data received for '%s'", *req_iterator);

        if (in_bcc_ == in_buf_[frame_size - 1]) {
          ESP_LOGV(TAG, "BCC OK");
        } else {
          ESP_LOGE(TAG, "BCC error. Skipping data packet.");
          this->set_next_state_(State::DATA_NEXT);
          break;
        }

        if (!get_values_from_brackets_((char *) in_buf_, vals)) {
          ESP_LOGE(TAG, "Invalid frame format: '%s'", in_buf_);
          break;
        }
        ESP_LOGD(TAG, "Data received: param '%s', value1 '%s', value2 '%s', value3 '%s', value4 '%s'", in_buf_, vals[0],
                 vals[1], vals[2], vals[3]);

        // std::string param = std::string(param_name) + "()";
        static const size_t param_buff_size = IEC61107SensorBase::MAX_REQUEST_SIZE + 2;
        static char param_buff[param_buff_size]{0};
        snprintf(param_buff, param_buff_size, "%s()", in_buf_);
        param_buff[param_buff_size - 1] = 0;

        if (param_buff[0]) {
          // Update all matching sensors
          auto range = sensors_.equal_range(param_buff);  // std::string(param_buff));
          for (auto it = range.first; it != range.second; ++it) {
            ESP_LOGD(TAG, "Sensor %s, idx %d", it->second->get_request(), it->second->get_index());
            if (!it->second->is_failed())
              set_sensor_value_(it->second, vals);
          }
        }
        this->set_next_state_(State::DATA_NEXT);
      }
      break;

    case State::DATA_FAIL:
      this->report_state_();
      ESP_LOGW(TAG, "Data request failed. Value for '%s' not received. (Not supported ?)", *req_iterator);
      this->update_last_transmission_from_meter_timestamp_();
      this->set_next_state_(State::DATA_NEXT);
      break;

    case State::DATA_NEXT:
      this->report_state_();
      req_iterator++;
      if (req_iterator != this->requests_.end()) {
        this->set_next_state_delayed_(this->delay_between_requests_ms_, State::DATA_ENQ);
      } else {
        this->set_next_state_delayed_(this->delay_between_requests_ms_, State::CLOSE_SESSION);
      }
      break;
    case State::CLOSE_SESSION:
      this->report_state_();
      ESP_LOGD(TAG, "Closing session");
      this->send_frame_(CMD_CLOSE_SESSION, sizeof(CMD_CLOSE_SESSION));
      this->set_next_state_(State::PUBLISH);
      ESP_LOGD(TAG, "Total connection time: %u ms", millis() - started_ms);
      sens_iterator = this->sensors_.begin();
      break;

    case State::PUBLISH:
      this->report_state_();
      ESP_LOGD(TAG, "Publishing data");
      this->update_last_transmission_from_meter_timestamp_();

      if (sens_iterator != this->sensors_.end()) {
        sens_iterator->second->publish();
        sens_iterator++;
      } else {
        ESP_LOGD(TAG, "Data collection and publishing finished.");
        ESP_LOGD(TAG, "Total time: %u ms", millis() - started_ms);

        this->report_failure(false);
        this->set_next_state_(State::IDLE);
      }
      break;

    default:
      break;
  }
}

void IEC61107Component::update() {
  if (this->state_ != State::IDLE) {
    ESP_LOGD(TAG, "Starting data collection impossible - component not ready");
    return;
  }
  ESP_LOGD(TAG, "Starting data collection");
  this->set_next_state_(State::OPEN_SESSION);
}

bool char2float(const char *str, float &value) {
  char *end;
  value = strtof(str, &end);
  return *end == '\0';
}

bool IEC61107Component::set_sensor_value_(IEC61107SensorBase *sensor, ValuesArray &vals) {
  auto type = sensor->get_type();
  bool ret = true;

  uint8_t idx = sensor->get_index() - 1;
  if (idx >= 4) {
    ESP_LOGE(TAG, "Invalid index %u", idx);
    return false;
  }
  if (vals[idx][0] == 'E' && vals[idx][1] == 'R' && vals[idx][2] == 'R') {
    ESP_LOGE(TAG,
             "Parameter %s either not supported or request is improperly formed. Sensor will be disabled after few "
             "tries. %s",
             sensor->get_request(), vals[idx]);
    sensor->record_failure();
    return false;
  }

  const char *str = vals[idx];  //.c_str();

  ESP_LOGV(TAG, "Setting value for sensor '%s' to '%s', idx = %d", sensor->get_request(), str, idx + 1);

  if (type == SensorType::SENSOR) {
    float f = 0;
    ret = vals[idx][0] && char2float(str, f);
    if (ret) {
      static_cast<IEC61107Sensor *>(sensor)->set_value(f);
    } else {
      ESP_LOGE(TAG, "Cannot convert incoming data to a number. Consider using a text sensor. Invalid data: '%s'", str);
    }
  } else {
#ifdef USE_TEXT_SENSOR
    static_cast<IEC61107TextSensor *>(sensor)->set_value(str);
#endif
  }
  return ret;
}

uint8_t checksum_7f(const uint8_t *data, size_t length) {
  uint8_t crc = 0;
  if (length < 2) {
    return 0;
  }
  for (size_t i = 1; i < length - 1; i++) {
    crc += data[i];
  }
  return crc & 0x7f;
}

void IEC61107Component::set_next_state_delayed_(uint32_t ms, State next_state) {
  ESP_LOGD(TAG, "Short delay before next step for %u ms", ms);
  set_next_state_(State::WAIT);
  wait_start_timestamp_ = millis();
  wait_period_ms_ = ms;
  next_state_after_wait_ = next_state;
}

void IEC61107Component::prepare_request_frame_(const char *request) {
  // assume request has format "XXXX(params)" if there is closing bracket
  // if not - assume it is "XXXX" and add ()
  //  std::string frame = "\x01R1\x02" + request + (request.back() == ')' ? "\x03" : "()\x03");

  auto len = strlen(request);
  if (request[len - 1] == ')') {
    snprintf((char *) out_buf_, MAX_OUT_BUF_SIZE, "\x01R1\x02%s\x03", request);
  } else {
    snprintf((char *) out_buf_, MAX_OUT_BUF_SIZE, "\x01R1\x02%s()\x03", request);
    len += 2;
  }
  data_out_size_ = len + 6;
  uint8_t bcc = checksum_7f(out_buf_, data_out_size_);
  out_buf_[data_out_size_ - 1] = bcc;
}

void IEC61107Component::send_frame_() {
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  this->write_array(out_buf_, data_out_size_);

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);

  ESP_LOGV(TAG, "TX: %s", format_hex_pretty(out_buf_, data_out_size_).c_str());
}

void IEC61107Component::prepare_frame_(const uint8_t *data, size_t length) {
  memcpy(out_buf_, data, length);
  data_out_size_ = length;
}

void IEC61107Component::send_frame_(const uint8_t *data, size_t length) {
  this->prepare_frame_(data, length);
  this->send_frame_();
}

size_t IEC61107Component::receive_frame_() {
  const uint32_t max_while_ms = 25;
  size_t ret_val;
  auto count = this->available();
  if (count <= 0)
    return 0;

  static bool soh_detected = false;
  static bool stx_detected = false;
  static bool etx_detected = false;

  uint32_t while_start = millis();
  uint8_t *p;
  while (count > 0) {
    // Make sure loop() is <30 ms
    if (millis() - while_start > max_while_ms) {
      return 0;
    }

    if (data_in_size_ < MAX_IN_BUF_SIZE) {
      p = &in_buf_[data_in_size_];
      if (!iuart_->read_one_byte(p)) {
        return 0;
      }
      // if (etx_detected && *p == 0) {
      //   // skip zeroes after ETX
      //   ESP_LOGV(TAG, "Skipping zeroes after ETX");
      //   yield();
      //   App.feed_wdt();
      //   continue;
      // }
      data_in_size_++;
    } else {
      memmove(in_buf_, in_buf_ + 1, data_in_size_ - 1);
      p = &in_buf_[data_in_size_ - 1];
      if (!iuart_->read_one_byte(p)) {
        return 0;
      }
    }

    // it is not possible to have \r\n and ETX in buffer at one time
    if (data_in_size_ >= 2 && ETX == in_buf_[data_in_size_ - 2]) {
      ESP_LOGV(TAG, "RX.1: %s", format_hex_pretty(in_buf_, data_in_size_).c_str());
      ESP_LOGV(TAG, "Detected ETX");
      etx_detected = true;

      in_bcc_ = in_buf_[data_in_size_ - 1];
      ESP_LOGV(TAG, "BCC: 0x%02x", in_bcc_);

      soh_detected = false;
      stx_detected = false;
      update_last_transmission_from_meter_timestamp_();
      ret_val = data_in_size_;
      data_in_size_ = 0;
      return ret_val;
    }

    if (SOH == in_buf_[data_in_size_ - 1]) {
      ESP_LOGV(TAG, "RX.2: %s", format_hex_pretty(in_buf_, data_in_size_).c_str());
      ESP_LOGV(TAG, "Detected SOH");
      reset_bcc_();
      update_last_transmission_from_meter_timestamp_();
      ret_val = data_in_size_;
      data_in_size_ = 0;
      soh_detected = true;
      stx_detected = false;
      return 0;  // ret_val;
    }

    if (STX == in_buf_[data_in_size_ - 1]) {
      ESP_LOGV(TAG, "RX.3: %s", format_hex_pretty(in_buf_, data_in_size_).c_str());
      if (soh_detected) {
        ESP_LOGV(TAG, "Detected STX after SOH");
        soh_detected = false;
        stx_detected = true;
      } else {
        if (data_in_size_ == 1) {
          ESP_LOGV(TAG, "Detected STX w/o data yet");
          this->bcc_ = STX;
          stx_detected = true;
          data_in_size_ = 0;  /////////////////////////////////////////////////////////// test
          return 0;
        }

        // todo: check its not part of transmission....
        ESP_LOGV(TAG, "Detected STX with data before it");
        if (data_in_size_ > 2 && in_buf_[data_in_size_ - 2] == 0) {
          reset_bcc_();
          update_last_transmission_from_meter_timestamp_();
          ESP_LOGV(TAG, ".. zeroes before STX. Wait for more data.");
          return 0;
        }
        ESP_LOGV(TAG, "STX with data before it. Should not be like that... Let's wait for more data");
        return 0;

        // ret_val = data_in_size_;
        // data_in_size_ = 0;
        // return ret_val;
      }
    }
    bool crlfdata_out = !stx_detected || this->readout_mode_;
    if (crlfdata_out &&
        (data_in_size_ >= 2 && '\r' == in_buf_[data_in_size_ - 2] && '\n' == in_buf_[data_in_size_ - 1])) {
      ESP_LOGV(TAG, "RX.4: %s", format_hex_pretty(in_buf_, data_in_size_).c_str());

      // check echo
      if (data_in_size_ == data_out_size_ && 0 == memcmp(out_buf_, in_buf_, data_out_size_)) {
        data_out_size_ = data_in_size_ = 0;
        ESP_LOGV(TAG, "Echo. Ignore frame.");
        return 0;
      }

      update_last_transmission_from_meter_timestamp_();
      ret_val = data_in_size_;
      // if (data_in_size_ < MAX_IN_BUF_SIZE)
      //   in_buf_[data_in_size_] = 0;
      data_in_size_ = 0;
      return ret_val;
    }
  }
  return 0;
}

void IEC61107Component::clear_uart_input_buffer_() {
  int available = this->available();
  int len;

  if (available > 0) {
    ESP_LOGVV(TAG, "Cleaning garbage from UART input buffer: %d bytes", available);
  }

  while (available > 0) {
    len = std::min(available, (int) MAX_IN_BUF_SIZE);

    this->read_array(in_buf_, len);
    available -= len;
  }
  data_in_size_ = 0;
}

char *IEC61107Component::get_id_(size_t frame_size) {
  uint8_t *p = &in_buf_[frame_size - 1 - 2 /*\r\n*/];
  size_t min_id_data_size = 7;  // min packet is '/XXXZ\r\n'

  while (p >= in_buf_ && frame_size >= min_id_data_size) {
    if ('/' == *p) {
      if ((size_t) (&in_buf_[MAX_IN_BUF_SIZE - 1] - p) < min_id_data_size) {
        ESP_LOGV(TAG, "Invalid Meter ID packet.");

        // garbage, ignore
        break;
      }
      in_buf_[frame_size - 2] = '\0';  // terminate string and remove \r\n
      ESP_LOGD(TAG, "Meter identification: '%s'", p);

      return (char *) p;
    }

    p--;
  }

  return nullptr;
}

uint8_t IEC61107Component::get_values_from_brackets_(char *line, ValuesArray &vals) {
  // line = "VOLTA(100.1)VOLTA(200.1)VOLTA(300.1)VOLTA(400.1)"
  static char empty_str[] = "";
  vals.fill(empty_str);
  //  vals[0] = vals[1] = vals[2] = vals[3] = empty_str;

  uint8_t idx = 0;
  bool got_param_name{false};
  char *p = line;
  while (*p && idx < VAL_NUM) {
    if (*p == '(') {
      if (!got_param_name) {
        got_param_name = true;
        // if (p != line) {
        //   param.assign(std::string(line, p));
        // }
        *p = 0;  // null-terminate param name
      }
      char *start = p + 1;
      char *end = strchr(start, ')');
      if (end) {
        *end = 0;  // null-terminate value
                   //        std::string value = std::string(start, end);
        if (idx < VAL_NUM) {
          //          vals[idx++].assign(value);
          vals[idx++] = start;
        }
        p = end;
      }
    }
    p++;
  }
  return idx;  // at least one bracket found
}

void IEC61107Component::reset_bcc_() { this->bcc_ = 0; }

void IEC61107Component::update_bcc_(const uint8_t *data, size_t size) {
  for (size_t i = 0; i < size; i++) {
    this->bcc_ = (this->bcc_ + data[i]) & 0x7f;
  }
}

void IEC61107Component::report_state_() {
  static State last_reported_state{State::NOT_INITIALIZED};
  const char *state_txt;

  switch (this->state_) {
    case State::NOT_INITIALIZED:
      state_txt = "NOT_INITIALIZED";
      break;
    case State::IDLE:
      state_txt = "IDLE";
      break;
    case State::WAIT:
      state_txt = "WAIT";
      break;
    case State::OPEN_SESSION:
      state_txt = "OPEN_SESSION";
      break;
    case State::OPEN_SESSION_GET_ID:
      state_txt = "OPEN_SESSION_GET_ID";
      break;
    case State::SET_BAUD_RATE:
      state_txt = "SET_BAUD_RATE";
      break;
    case State::ACK_START_GET_INFO:
      state_txt = "ACK_START_GET_INFO";
      break;
    case State::DATA_ENQ:
      state_txt = "DATA_ENQ";
      break;
    case State::DATA_RECV:
      state_txt = "DATA_RECV";
      break;
    case State::DATA_FAIL:
      state_txt = "DATA_FAIL";
      break;
    case State::DATA_NEXT:
      state_txt = "DATA_NEXT";
      break;
    case State::READOUT:
      state_txt = "READOUT";
      break;
    case State::CLOSE_SESSION:
      state_txt = "CLOSE_SESSION";
      break;
    case State::PUBLISH:
      state_txt = "PUBLISH";
      break;
    default:
      state_txt = "UNKNOWN";
      break;
  }

  if (state_ != last_reported_state) {
    ESP_LOGV(TAG, "%s", state_txt);
    last_reported_state = state_;
  }
}

uint32_t IEC61107Component::identification_to_baud_rate_(char z) {
  uint32_t rate;

  if (z >= PROTO_B_RANGE_BEGIN && z <= PROTO_B_RANGE_END) {
    rate = BAUDRATES[1 + z - PROTO_B_RANGE_BEGIN];
  } else if (z >= PROTO_C_RANGE_BEGIN && z <= PROTO_C_RANGE_END) {
    rate = BAUDRATES[z - PROTO_C_RANGE_BEGIN];
  } else {
    rate = 0;
  }

  return rate;
}

char IEC61107Component::baud_rate_to_identification_(uint32_t baud_rate) {
  // PROTOCOL_MODE_C == mode_
  for (size_t i = 0; i < (sizeof(BAUDRATES) / sizeof(uint32_t)); i++) {
    if (baud_rate == BAUDRATES[i]) {
      return PROTO_C_RANGE_BEGIN + i;
    }
  }

  // return lowest baudrate for unknown char
  return PROTO_C_RANGE_BEGIN;
}

}  // namespace iec61107
}  // namespace esphome
