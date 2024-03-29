#pragma once

#include "esphome/components/sensor/sensor.h"
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif 

namespace esphome {
namespace iec61107 {

enum SensorType { SENSOR, TEXT_SENSOR };

class IEC61107SensorBase {
 public:
  virtual SensorType get_type() = 0;
  virtual void publish() = 0;

  void set_request(const char *req) { request_ = req; };
  std::string get_request() { return request_; }

  void reset() { has_value_ = false; }

  bool has_value() { return has_value_; }

 protected:
  std::string request_;
  bool has_value_;
};

class IEC61107Sensor : public IEC61107SensorBase, public sensor::Sensor {
 public:
  SensorType get_type() override { return SENSOR; }
  void publish() override { publish_state(value_); }

  void set_value(float value) {
    value_ = value;
    has_value_ = true;
  }

 protected:
  float value_;
};

#ifdef USE_TEXT_SENSOR
class IEC61107TextSensor : public IEC61107SensorBase, public text_sensor::TextSensor {
 public:
  SensorType get_type() override { return TEXT_SENSOR; }
  void publish() override { publish_state(value_); }

  void set_value(const char *value) {
    value_ = value;
    has_value_ = true;
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
