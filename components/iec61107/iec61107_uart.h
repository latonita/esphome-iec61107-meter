#pragma once
#include <cstdint>

#ifdef USE_ESP_IDF
#include "esphome/components/uart/uart_component_esp_idf.h"
#include "esphome/core/log.h"
#endif

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
#include "esphome/components/uart/uart_component_esp32_arduino.h"
#include <HardwareSerial.h>
#endif

#ifdef USE_ESP8266
#include "esphome/components/uart/uart_component_esp8266.h"
#endif

#ifdef USE_ESP_IDF
// backward compatibility with old IDF versions
#ifndef portTICK_PERIOD_MS
#define portTICK_PERIOD_MS portTICK_RATE_MS
#endif
#endif

namespace esphome {
namespace iec61107 {

static const uint32_t TIMEOUT = 30;  // default value in uart implementation is 100ms

#ifdef USE_ESP32_FRAMEWORK_ARDUINO

class Iec61107Uart final : public uart::ESP32ArduinoUARTComponent {
 public:
  Iec61107Uart(uart::ESP32ArduinoUARTComponent const &uart) : uart_(uart), hw_(uart.*(&Iec61107Uart::hw_serial_)) {}

  // Reconfigure baudrate
  bool update_baudrate(uint32_t baudrate) {
    this->hw_->updateBaudRate(baudrate);
    return true;
  }

  /// @brief Reads one byte. Uses 20ms inter-character timeout.
  /// @param data Pointer to one byte buffer to store data
  /// @retval true byte received
  /// @retval false no data
  /// @remarks
  /// Default @c read_byte() function waits 100 ms when no data in input buffer.
  /// This increase time spent in @c loop() function above accepted value (50ms).
  bool read_one_byte(uint8_t *data) {
    if (!this->check_read_timeout_quick_(1))
      return false;
    this->hw_->readBytes(data, 1);
    return true;
  }

 protected:
  /// @brief Helper function for @ref read_one_byte()
  /// @remarks
  /// Uses 20ms timeout instead of default 100ms.
  bool check_read_timeout_quick_(size_t len) {
    if (this->hw_->available() >= int(len))
      return true;

    uint32_t start_time = millis();
    while (this->hw_->available() < int(len)) {
      if (millis() - start_time > TIMEOUT) {
        return false;
      }
      yield();
    }
    return true;
  }

  uart::ESP32ArduinoUARTComponent const &uart_;
  HardwareSerial *const hw_;
};
#endif

#ifdef USE_ESP8266

class XSoftSerial : public uart::ESP8266SoftwareSerial {
 public:
  void set_bit_time(uint32_t bt) { bit_time_ = bt; }
};

class Iec61107Uart final : public uart::ESP8266UartComponent {
 public:
  Iec61107Uart(uart::ESP8266UartComponent const &uart)
      : uart_(uart), hw_(uart.*(&Iec61107Uart::hw_serial_)), sw_(uart.*(&Iec61107Uart::sw_serial_)) {}

  bool update_baudrate(uint32_t baudrate) {
    if (baudrate == 0) {
      return false;
    }
    if (this->hw_ != nullptr) {
      this->hw_->updateBaudRate(baudrate);
    } else {
      ((XSoftSerial *) sw_)->set_bit_time(F_CPU / baudrate);
    }
    return true;
  }

  bool read_one_byte(uint8_t *data) {
    if (this->hw_ != nullptr) {
      if (!this->check_read_timeout_quick_(1))
        return false;
      this->hw_->readBytes(data, 1);
    } else {
      if (sw_->available() < 1)
        return false;
      assert(this->sw_ != nullptr);
      optional<uint8_t> b = this->sw_->read_byte();
      if (b) {
        *data = *b;
      } else {
        return false;
      }
    }
    return true;
  }

 protected:
  bool check_read_timeout_quick_(size_t len) {
    if (this->hw_->available() >= int(len))
      return true;

    uint32_t start_time = millis();
    while (this->hw_->available() < int(len)) {
      if (millis() - start_time > TIMEOUT) {
        return false;
      }
      yield();
    }
    return true;
  }

  uart::ESP8266UartComponent const &uart_;
  HardwareSerial *const hw_;               // hardware Serial
  uart::ESP8266SoftwareSerial *const sw_;  // software serial
};
#endif

#ifdef USE_ESP_IDF
class Iec61107Uart final : public uart::IDFUARTComponent {
 public:
  Iec61107Uart(uart::IDFUARTComponent &uart)
      : uart_(uart), iuart_num_(uart.*(&Iec61107Uart::uart_num_)), ilock_(uart.*(&Iec61107Uart::lock_)) {}

  void setup_half_duplex(int8_t flow_control_pin = -1) {
    if (flow_control_pin == -1) {
      return;
    }
    // do we need to uninstall uart_driver and then install again?
    int8_t tx = this->tx_pin_ != nullptr ? this->tx_pin_->get_pin() : -1;
    int8_t rx = this->rx_pin_ != nullptr ? this->rx_pin_->get_pin() : -1;
    auto err = uart_set_pin(this->uart_num_, tx, rx, flow_control_pin, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
      ESP_LOGW("iec61107_uart", "setup_half_duplex() uart_set_pin failed: %s", esp_err_to_name(err));
      this->mark_failed();
      return;
    }

    uart_set_mode(iuart_num_, UART_MODE_RS485_HALF_DUPLEX);

    ESP_LOGI("rs485", "UART reconfigured to RS485 half-duplex mode.");
  }

  // Reconfigure baudrate
  bool update_baudrate(uint32_t baudrate) {
    int err = ESP_OK;
    xSemaphoreTake(ilock_, portMAX_DELAY);
    uart_flush(iuart_num_);
    err = uart_set_baudrate(iuart_num_, baudrate);
    xSemaphoreGive(ilock_);
    return err == ESP_OK;
  }

  bool read_one_byte(uint8_t *data) { return read_array_quick_(data, 1); }

 protected:
  bool check_read_timeout_quick_(size_t len) {
    if (uart_.available() >= int(len))
      return true;

    uint32_t start_time = millis();
    while (uart_.available() < int(len)) {
      if (millis() - start_time > TIMEOUT) {
        return false;
      }
      yield();
    }
    return true;
  }

  bool read_array_quick_(uint8_t *data, size_t len) {
    size_t length_to_read = len;
    if (!this->check_read_timeout_quick_(len))
      return false;
    xSemaphoreTake(this->ilock_, portMAX_DELAY);
    if (this->has_peek_) {
      length_to_read--;
      *data = this->peek_byte_;
      data++;
      this->has_peek_ = false;
    }
    if (length_to_read > 0)
      uart_read_bytes(this->iuart_num_, data, length_to_read, TIMEOUT / portTICK_PERIOD_MS);
    xSemaphoreGive(this->ilock_);

    return true;
  }

  uart::IDFUARTComponent &uart_;
  uart_port_t iuart_num_;
  SemaphoreHandle_t &ilock_;
};
#endif

}  // namespace iec61107
}  // namespace esphome
