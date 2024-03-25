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
  /* parser */
  enum message_t : uint8_t
  {
    GET_SENSOR_DATA,
    RET_SENSOR_DATA,
    RET_ERROR,
  };

  enum message_parse_result_t : uint8_t
  {
    OK,
    INVALID_MSG_ID,
    INVALID_PAYLOAD_LENGTH,
    INVALID_CRC,
    TIMEOUT,
  };
private:
  enum cmd_event_t : uint8_t
  {
    EVENT_DATA,
    EVENT_RESET,
  };

  enum cmd_state_t : uint8_t
  {
    INIT,
    RECEIVED_ID,
    RECEIVED_PAYLOAD_LEN,
    NEED_MORE,
    RECEIVED_PAYLOAD,

    RECEIVED_MESSAGE,
  };

  using cmd_fsm_t = fsm::statemachine<cmd_event_t, cmd_state_t>;

  static constexpr const char* state_name(cmd_state_t state)
  {
    switch (state) {
      case INIT:
        return "INIT";
      case RECEIVED_ID:
        return "RECEIVED_ID";
      case RECEIVED_PAYLOAD:
        return "RECEIVED_PAYLOAD";
      case RECEIVED_PAYLOAD_LEN:
        return "RECEIVED_PAYLOAD_LEN";
      case NEED_MORE:
        return "NEED_MORE";
      case RECEIVED_MESSAGE:
        return "RECEIVED_MESSAGE";
      default:
        return "??";
    }
  }


  message_parse_result_t m_crcok;
  uint8_t m_buffer[16] = { 0 };
  uint8_t m_buffer_index = 0;
  uint8_t m_last_data;

  inline const constexpr uint8_t payload_len() const
  {
    return m_buffer[0];
  }

  inline constexpr const uint8_t message_len() const
  {
    return payload_len()
      + 1 /* MSG_ID */
      + 1; /* PAYLOAD_LENGTH */
      // + 1 /* CRC8 */
  }

  static inline cmd_fsm_t::filter_t accept_any()
  {
    return [](auto) constexpr { return true; };
  }

  inline cmd_fsm_t::filter_t is_data(cmd_fsm_t::filter_t &&and_then)
  {
    return [and_then](const auto& e) {
      if (e != EVENT_DATA) {
        return false;
      }
      return and_then(e);
    };
  }

  inline cmd_fsm_t::callback_t append_buffer()
  {
    return [this](auto, auto, auto) constexpr
    {
      m_buffer[m_buffer_index++] = m_last_data;
    };
  }
  cmd_fsm_t cmd_fsm = cmd_fsm_t
{
  .state = INIT,
  .transitions = {
    /* move from init to received_id only if event data is known */
    {
      .filter = cmd_fsm_t::match({EVENT_RESET}),
      .source = INIT,
      .target = INIT,
    },
    {
      .filter = cmd_fsm_t::match({EVENT_RESET}),
      .source = RECEIVED_PAYLOAD_LEN,
      .target = INIT,
    },
    {
      .filter = cmd_fsm_t::match({EVENT_RESET}),
      .source = NEED_MORE,
      .target = INIT,
    },
    {
      .filter = cmd_fsm_t::match({EVENT_RESET}),
      .source = RECEIVED_PAYLOAD,
      .target = INIT,
    },
    {
      .filter = cmd_fsm_t::match({EVENT_RESET}),
      .source = RECEIVED_MESSAGE,
      .target = INIT,
    },
    {
      .filter = [this](auto)
      {
        return m_last_data > 0 && m_last_data < sizeof(m_buffer);
      },
      .source = INIT,
      .target = RECEIVED_PAYLOAD_LEN,
      .transit = [this](auto, auto, auto)
      {
        m_buffer_index = 0;
        m_buffer[m_buffer_index++] = m_last_data;
      }
    },
    {
      .filter = [this](auto)
      {
        return payload_len() < m_buffer_index;
      },
      .source = RECEIVED_PAYLOAD_LEN,
      .target = NEED_MORE,
      .transit = append_buffer(),
    },
    {
      .filter = [this](auto)
      {
        return payload_len() == m_buffer_index;
      },
      .source = RECEIVED_PAYLOAD_LEN,
      .target = RECEIVED_PAYLOAD,
      .transit = append_buffer(),
    },
    {
      .filter = [this](auto)
      {
        return payload_len() < m_buffer_index;
      },
      .source = NEED_MORE,
      .target = NEED_MORE,
      .transit = append_buffer(),
    },
    {
      .filter = [this](auto)
      {
        return payload_len() == m_buffer_index;
      },
      .source = NEED_MORE,
      .target = RECEIVED_PAYLOAD,
      .transit = append_buffer(),
    },
    {
      .filter = [this](auto) {
        return true;
      },
      .source = RECEIVED_PAYLOAD,
      .target = RECEIVED_MESSAGE,
      .transit = [this](auto, auto, auto)
      {
        auto crc8 = calc_crc8(m_buffer, payload_len());
        printf("Calculated %02X, required %02X\n", crc8, m_last_data);
      }
    }
  },
};
public:
  constexpr message_t message() const
  {
    return static_cast<message_t>(m_buffer[0]);
  }
  static constexpr const char* result_name(message_parse_result_t r)
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
  void reset()
  {
    cmd_fsm(EVENT_RESET);
  }

  enum class feed_result_t
  {
    PARSED,
    NEED_MORE,
    ERROR
  };

  feed_result_t feed(uint8_t data)
  {
    m_last_data = data;
    if (!cmd_fsm(EVENT_DATA))
    {
      reset();
      return feed_result_t::ERROR;
    }

    if (cmd_fsm.state == RECEIVED_MESSAGE)
    {
      reset();
      return feed_result_t::PARSED;
    }

    if (cmd_fsm.state == INIT)
    {
      return feed_result_t::ERROR;
    }

    return feed_result_t::NEED_MORE;
  }

  const message_parse_result_t msg_status() const {
    return m_crcok;
  }

  const uint8_t msg_len() const {
    return m_buffer_index;
  }

  const uint8_t *buffer() const {
    return m_buffer;
  }
};

class ProtocolParser2
{
  public:
  using message_t = ProtocolParser::message_t;
  using feed_result_t = ProtocolParser::feed_result_t;
  using status_t = ProtocolParser::message_parse_result_t;

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
