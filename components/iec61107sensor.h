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
  virtual SensorType get_type() const = 0;
  virtual void publish() = 0;

  void set_request(const char *req) { request_ = req; };
  const std::string &get_request() const { return request_; }

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
  std::string request_;
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
  SensorType get_type() const override { return TEXT_SENSOR; }
  void publish() override { publish_state(value_); }

  void set_value(const char *value) {
    value_ = value;
    has_value_ = true;
    tries_ = 0;
  }

  //   void set_group(int group) { group_ = group % 3; }

  //   uint8_t get_group() { return group_; }

 protected:
  std::string value_;
  //   // 0 - entire frame, 1 value from the first () group, 2 value from the second () group
  //   // 1-0:1.6.0(00000001000.000*kW)(2000-10-01 00:00:00)
  //   uint8_t group_;
};
#endif

}  // namespace iec61107
}  // namespace esphome
