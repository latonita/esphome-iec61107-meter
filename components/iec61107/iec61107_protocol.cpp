#include "iec61107_protocol.h"

#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <cstring>

namespace esphome {
namespace iec61107 {
namespace protocol {

static constexpr uint32_t BAUD_BASE = 300;
static constexpr std::array<uint8_t, ACK_SET_BAUD_AND_MODE_FRAME_SIZE> ACK_SET_BAUD_AND_MODE_TEMPLATE = {
    ACK, '0', '5', '1', CR, LF};
static constexpr std::array<uint8_t, CLOSE_SESSION_FRAME_SIZE - 1> CLOSE_SESSION_PREFIX = {SOH, 0x42, 0x30, ETX};

static uint32_t byte_to_baud_rate(uint8_t baud_code) {
  baud_code -= '0';
  return (baud_code > 6) ? BAUD_BASE : BAUD_BASE * (1 << baud_code);
}

uint8_t calculate_crc_prog_frame(uint8_t *data, size_t length, bool set_crc, bool use_xor) {
  uint8_t crc = 0;
  if (length < 2) {
    return 0;
  }

  for (size_t i = 1; i < length - 1; i++) {
    if (use_xor) {
      crc ^= data[i];
    } else {
      crc = (crc + data[i]) & 0x7f;
    }
  }
  crc &= 0x7f;

  if (set_crc) {
    data[length - 1] = crc;
  }
  return crc;
}

uint8_t get_values_from_brackets(char *line, ValueRefsArray &vals, char *empty_value) {
  // line = "VOLTA(100.1)VOLTA(200.1)VOLTA(300.1)VOLTA(400.1)"
  vals.fill(empty_value);

  uint8_t idx = 0;
  bool got_param_name{false};
  char *p = line;
  while (*p && idx < VALUE_SLOTS) {
    if (*p == '(') {
      if (!got_param_name) {
        got_param_name = true;
        *p = '\0';  // null-terminate param name
      }
      char *start = p + 1;
      char *end = std::strchr(start, ')');
      if (end) {
        *end = '\0';  // null-terminate value
        if (idx < VALUE_SLOTS) {
          vals[idx++] = start;
        }
        p = end;
      }
    }
    p++;
  }
  return idx;
}

char *get_nth_value_from_csv(char *line, uint8_t idx) {
  if (idx == 0) {
    return line;
  }

  char *ptr = std::strtok(line, ",");
  while (ptr != nullptr) {
    if (idx-- == 1) {
      return ptr;
    }
    ptr = std::strtok(nullptr, ",");
  }
  return nullptr;
}

IdentificationFrame parse_identification_frame(uint8_t *buffer, size_t frame_size, size_t buffer_size) {
  static constexpr size_t MIN_ID_DATA_SIZE = 7;  // min packet is '/XXXZ\r\n'

  IdentificationFrame out{};
  if (frame_size < MIN_ID_DATA_SIZE) {
    return out;
  }

  uint8_t *p = &buffer[frame_size - 1 - 2 /*\r\n*/];
  while (p >= buffer) {
    if (*p == '/') {
      if ((size_t) (&buffer[buffer_size - 1] - p) < MIN_ID_DATA_SIZE) {
        break;
      }

      buffer[frame_size - 2] = '\0';  // terminate string and remove \r\n
      out.ident = reinterpret_cast<char *>(p);
      out.baud_code = out.ident[4];
      out.type_c = out.baud_code >= '0' && out.baud_code <= '6';
      out.baud_rate = byte_to_baud_rate((uint8_t) out.baud_code);
      break;
    }
    p--;
  }

  return out;
}

size_t build_prog_password_frame(uint8_t *out, size_t out_size, const char *password, bool use_xor) {
  // "P1(password)"
  const int len =
      std::snprintf((char *) out, out_size, "%cP1%c(%s)%c\xFF", SOH, STX, password != nullptr ? password : "", ETX);
  if (len <= 0) {
    return 0;
  }
  const size_t out_len = std::min((size_t) len, out_size);
  calculate_crc_prog_frame(out, out_len, true, use_xor);
  return out_len;
}

size_t build_prog_frame(uint8_t *out, size_t out_size, const char *request, bool write, bool use_xor) {
  const int len = std::snprintf((char *) out, out_size, "%c%c1%c%s%c\xFF", SOH, (write ? 'W' : 'R'), STX,
                                request != nullptr ? request : "", ETX);
  if (len <= 0) {
    return 0;
  }
  const size_t out_len = std::min((size_t) len, out_size);
  calculate_crc_prog_frame(out, out_len, true, use_xor);
  return out_len;
}

size_t build_non_session_prog_frame(uint8_t *out, size_t out_size, const char *meter_address, const char *request,
                                    bool use_xor) {
  // "/?!<SOH>R1<STX>NAME()<ETX><BCC>" broadcast
  // "/?<address>!<SOH>R1<STX>NAME()<ETX><BCC>" direct
  const int len = std::snprintf((char *) out, out_size, "/?%s!%cR1%c%s%c\xFF",
                                meter_address != nullptr ? meter_address : "", SOH, STX,
                                request != nullptr ? request : "", ETX);
  if (len <= 0) {
    return 0;
  }
  const size_t out_len = std::min((size_t) len, out_size);

  uint8_t *r1_ptr = std::find(out, out + out_len, SOH);
  if (r1_ptr == out + out_len) {
    return out_len;
  }
  const size_t r1_size = (size_t) (r1_ptr - out);
  calculate_crc_prog_frame(r1_ptr, out_len - r1_size, true, use_xor);
  return out_len;
}

size_t build_ctime_frame(uint8_t *out, size_t out_size, uint8_t hh, uint8_t mm, uint8_t ss) {
  // "/?CTIME(HH:MM:SS)!\r\n"
  // no crc
  const int len = std::snprintf((char *) out, out_size, "/?CTIME(%02d:%02d:%02d)!\r\n", hh, mm, ss);
  if (len <= 0) {
    return 0;
  }
  return std::min((size_t) len, out_size);
}

size_t build_ack_set_baud_and_mode_frame(uint8_t *out, size_t out_size, uint8_t baud_code) {
  if (out_size < ACK_SET_BAUD_AND_MODE_FRAME_SIZE) {
    return 0;
  }
  std::memcpy(out, ACK_SET_BAUD_AND_MODE_TEMPLATE.data(), ACK_SET_BAUD_AND_MODE_FRAME_SIZE);
  out[2] = baud_code;
  return ACK_SET_BAUD_AND_MODE_FRAME_SIZE;
}

size_t build_close_session_frame(uint8_t *out, size_t out_size, bool use_xor) {
  if (out_size < CLOSE_SESSION_FRAME_SIZE) {
    return 0;
  }
  std::memcpy(out, CLOSE_SESSION_PREFIX.data(), CLOSE_SESSION_PREFIX.size());
  out[CLOSE_SESSION_FRAME_SIZE - 1] = 0;
  calculate_crc_prog_frame(out, CLOSE_SESSION_FRAME_SIZE, true, use_xor);
  return CLOSE_SESSION_FRAME_SIZE;
}

bool char2float(const char *str, float &value) {
  char *end;
  value = strtof(str, &end);
  return *end == '\0' || *end == '*' || *end == '#';
}

bool frame_end_ascii_crlf(const uint8_t *buffer, size_t size) {
  return size >= 2 && buffer[size - 1] == '\n' && buffer[size - 2] == '\r';
}

bool frame_end_ack_or_nak(const uint8_t *buffer, size_t size, uint8_t ack, uint8_t nak) {
  return size == 1 && (buffer[0] == ack || buffer[0] == nak);
}

bool frame_end_ack_nak_or_close(const uint8_t *buffer, size_t size, uint8_t ack, uint8_t nak, bool use_xor) {
  if (frame_end_ack_or_nak(buffer, size, ack, nak)) {
    return true;
  }

  if (size != CLOSE_SESSION_FRAME_SIZE) {
    return false;
  }

  if (std::memcmp(buffer, CLOSE_SESSION_PREFIX.data(), CLOSE_SESSION_PREFIX.size()) != 0) {
    return false;
  }

  uint8_t close_frame[CLOSE_SESSION_FRAME_SIZE];
  build_close_session_frame(close_frame, sizeof(close_frame), use_xor);
  return std::memcmp(buffer, close_frame, CLOSE_SESSION_FRAME_SIZE) == 0;
}

bool frame_end_prog_reply(const uint8_t *buffer, size_t size, uint8_t start_byte, bool accept_ack_nak, uint8_t ack,
                          uint8_t nak, uint8_t etx) {
  return (accept_ack_nak && frame_end_ack_or_nak(buffer, size, ack, nak)) ||
         (size > 3 && buffer[0] == start_byte && buffer[size - 2] == etx);
}

}  // namespace protocol
}  // namespace iec61107
}  // namespace esphome
