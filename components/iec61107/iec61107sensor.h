#pragma once

#include "esphome/components/sensor/sensor.h"
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome {
namespace iec61107 {

static constexpr uint8_t MAX_TRIES = 10;

enum SensorType { SENSOR, TEXT_SENSOR };

class IEC61107SensorBase {
 public:
  static const uint8_t MAX_REQUEST_SIZE = 15;

  virtual SensorType get_type() const = 0;
  virtual void publish() = 0;

  void set_request(const char *req) {
    strncpy(request_, req, MAX_REQUEST_SIZE);
    request_[MAX_REQUEST_SIZE] = '\0';
  };
  const char *get_request() const { return request_; }

  void set_index(const uint8_t idx) { idx_ = idx; };
  uint8_t get_index() const { return idx_; };

  void reset() {
    has_value_ = false;
    tries_ = 0;
  }

  bool has_value() { return has_value_; }

  void record_failure() {
    if (tries_ < MAX_TRIES) {
      tries_++;
    } else {
      has_value_ = false;
    }
  }
  bool is_failed() { return tries_ == MAX_TRIES; }

 protected:
  char request_[MAX_REQUEST_SIZE + 1];
  uint8_t idx_{1};
  bool has_value_;
  uint8_t tries_{0};
};

class IEC61107Sensor : public IEC61107SensorBase, public sensor::Sensor {
 public:
  SensorType get_type() const override { return SENSOR; }
  void publish() override { publish_state(value_); }

  void set_value(float value) {
    value_ = value;
    has_value_ = true;
    tries_ = 0;
  }

 protected:
  float value_;
};

#ifdef USE_TEXT_SENSOR
class IEC61107TextSensor : public IEC61107SensorBase, public text_sensor::TextSensor {
 public:
  static const uint8_t MAX_TEXT_SIZE = 63;
  SensorType get_type() const override { return TEXT_SENSOR; }
  void publish() override { publish_state(value_); }

  void set_value(const char *value) {
    strncpy(value_, value, MAX_TEXT_SIZE);
    request_[MAX_TEXT_SIZE] = '\0';
    //    value_ = value;
    has_value_ = true;
    tries_ = 0;
  }

 protected:
  //  std::string value_;
  char value_[MAX_TEXT_SIZE + 1]{0};
};
#endif

}  // namespace iec61107
}  // namespace esphome
