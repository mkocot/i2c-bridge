#include <cstdint>
#include <fsm.hpp>
#include <stdio.h>

// [LENGTH][CONTENT][CRC8]

uint8_t update_crc8(uint8_t *crc, const uint8_t *data, size_t length)
{
    while (length--) {
        *crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            if (*crc & 0x80) {
                *crc = (*crc << 1) ^ 0x31; // CRC8 polynomial: x^8 + x^5 + x^4 + 1 (0x31)
            } else {
                *crc <<= 1;
            }
        }
    }
    return *crc;
}

uint8_t calc_crc8(const uint8_t* data, size_t length) {
    uint8_t crc = 0xFF;
    return update_crc8(&crc, data, length);
}

class ProtocolParser
{
  public:
  enum message_t : uint8_t
  {
    GET_SENSOR_DATA,
    RET_SENSOR_DATA,
    RET_ERROR,
  };

  enum status_t : uint8_t
  {
    OK,
    INVALID_MSG_ID,
    INVALID_PAYLOAD_LENGTH,
    INVALID_CRC,
    TIMEOUT,
  };

  static constexpr const char* result_name(status_t r)
  {
    switch (r) {
      case INVALID_MSG_ID:
        return "invalid id";
      case INVALID_PAYLOAD_LENGTH:
        return "invalid payload";
      case INVALID_CRC:
        return "invalid crc";
      case OK:
        return "OK";
      case TIMEOUT:
        return "TIMEOUT";
      default:
        return "??";
    }
  }

  enum class feed_result_t : uint8_t
  {
    PARSED,
    NEED_MORE,
    ERROR
  };

  uint8_t _buffer[16] = {0};
  uint8_t buffer_index = 0;
  status_t message_status = status_t::INVALID_MSG_ID;

  feed_result_t feed(uint8_t data)
  {
    if (buffer_index > envelope_len())
    {
      reset();
    }

    if (buffer_index == 0)
    {
      if (data < 1 || data >= sizeof(_buffer))
      {
        message_status = status_t::INVALID_PAYLOAD_LENGTH;
        return feed_result_t::ERROR;
      }
      _buffer[buffer_index++] = data;
      return feed_result_t::NEED_MORE;
    }

    if (buffer_index < envelope_len())
    {
      _buffer[buffer_index++] = data;
      return feed_result_t::NEED_MORE;
    }

    auto crc8 = calc_crc8(_buffer, buffer_index);
    ++buffer_index;
    if (crc8 == data)
    {
      message_status = status_t::OK;
    }
    else
    {
      message_status = status_t::INVALID_CRC;
    }

    return feed_result_t::PARSED;
  }
  void reset()
  {
    buffer_index = 0;
    message_status = status_t::INVALID_MSG_ID;
  }

  uint8_t envelope_len() const
  {
    return msg_len() + 1;
  }

  uint8_t envelope_len_with_crc8() const {
    return envelope_len() + 1;
  }

  uint8_t msg_len() const
  {
    return _buffer[0];
  }

  const uint8_t *buffer() const
  {
    return _buffer + 1;
  }

  message_t message() const
  {
    return static_cast<message_t>(buffer()[0]);
  }

  status_t msg_status() const
  {
    return message_status;
  }

};
