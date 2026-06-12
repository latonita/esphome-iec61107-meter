#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace esphome {
namespace iec61107 {
namespace protocol {

// Protocol parsing/wiring constants and utilities live here.
// Component orchestration (state machine, retries, UART session ownership)
// stays in iec61107.cpp.
inline constexpr uint8_t SOH = 0x01;
inline constexpr uint8_t STX = 0x02;
inline constexpr uint8_t ETX = 0x03;
inline constexpr uint8_t EOT = 0x04;
inline constexpr uint8_t ENQ = 0x05;
inline constexpr uint8_t ACK = 0x06;
inline constexpr uint8_t LF = 0x0A;
inline constexpr uint8_t CR = 0x0D;
inline constexpr uint8_t NAK = 0x15;
inline constexpr size_t ACK_SET_BAUD_AND_MODE_FRAME_SIZE = 6;
inline constexpr size_t CLOSE_SESSION_FRAME_SIZE = 5;

static constexpr uint8_t VALUE_SLOTS = 12;
using ValueRefsArray = std::array<char *, VALUE_SLOTS>;

struct IdentificationFrame {
  char *ident{nullptr};
  bool type_c{false};
  char baud_code{'0'};
  uint32_t baud_rate{300};
};

uint8_t calculate_crc_prog_frame(uint8_t *data, size_t length, bool set_crc, bool use_xor);
uint8_t get_values_from_brackets(char *line, ValueRefsArray &vals, char *empty_value);
char *get_nth_value_from_csv(char *line, uint8_t idx);
IdentificationFrame parse_identification_frame(uint8_t *buffer, size_t frame_size, size_t buffer_size);
size_t build_prog_password_frame(uint8_t *out, size_t out_size, const char *password, bool use_xor);
size_t build_prog_frame(uint8_t *out, size_t out_size, const char *request, bool write, bool use_xor);
size_t build_non_session_prog_frame(uint8_t *out, size_t out_size, const char *meter_address, const char *request,
                                    bool use_xor);
size_t build_ctime_frame(uint8_t *out, size_t out_size, uint8_t hh, uint8_t mm, uint8_t ss);
size_t build_ack_set_baud_and_mode_frame(uint8_t *out, size_t out_size, uint8_t baud_code);
size_t build_close_session_frame(uint8_t *out, size_t out_size, bool use_xor);

bool char2float(const char *str, float &value);

bool frame_end_ascii_crlf(const uint8_t *buffer, size_t size);
bool frame_end_ack_or_nak(const uint8_t *buffer, size_t size, uint8_t ack, uint8_t nak);
bool frame_end_ack_nak_or_close(const uint8_t *buffer, size_t size, uint8_t ack, uint8_t nak, bool use_xor);
bool frame_end_prog_reply(const uint8_t *buffer, size_t size, uint8_t start_byte, bool accept_ack_nak, uint8_t ack,
                          uint8_t nak, uint8_t etx);

}  // namespace protocol
}  // namespace iec61107
}  // namespace esphome
