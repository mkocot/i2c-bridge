#include "ProtocolParser.hpp"
#include <cstdint>
#include <fsm.hpp>
#include <functional>

extern "C" unsigned long millis();
extern int obtain(uint8_t *data);

typedef void (*callback_t) (ProtocolParser::status_t, ProtocolParser::message_t, const uint8_t *, uint8_t);
#define SENSOR_PROTOCOL_DEBUG ((x)) while {} do(0)
//#define SENSOR_PROTOCOL_DEBUG ((x)) printf((x))
class SensorProtocol
{
  private:
  /* watchdog */
  enum watchdog_event_t : uint8_t
  {
    HAS_DATA,
    HAS_CMD,
    TICK,
    RESET,
  };

  enum watchdog_state_t : uint8_t
  {
    SLEEPING,
    AWAKE,
    BAKRING,
  };

  using fsm_watchdog_t = fsm::statemachine<watchdog_event_t, watchdog_state_t>;

  /* commands */
  static constexpr time_t WATCHDOG_TIME = 1000;
  time_t last_peting;
  ProtocolParser parser;

  // WITHOUT CRC8

  fsm_watchdog_t fsm_watchdog = {
    .state = SLEEPING,
    .transitions =       {
      /* comment to fix formating */
         {
          .filter =[this](auto e)
          {
             if (e != TICK)
             {
                return false;
             }

             return millis() - last_peting > WATCHDOG_TIME;
          },
          .source = AWAKE,
          .target = BAKRING,
          .transit = [this](auto, auto, auto)
          {
             printf("BARKING \n");
             fsm_watchdog(RESET);
             // Should it be event on it's own? Or just hard reset
             // by hand?
             printf("!! RESET TO INIT !!\n");
             parser.reset();

             on_command(ProtocolParser::TIMEOUT, parser.message(), nullptr, 0);
          }},
         {
             .filter = fsm_watchdog_t::match({HAS_DATA}),
             .source = SLEEPING,
             .target = AWAKE,
         },
         {
             .filter = fsm_watchdog_t::match({HAS_DATA}),
             .source = AWAKE,
             .target = AWAKE,
         },
         {
             .filter = fsm_watchdog_t::match({HAS_CMD, RESET}),
             .source = AWAKE,
             .target = SLEEPING,
         },
         {
             .filter = fsm_watchdog_t::match({RESET}),
             .source = BAKRING,
             .target = SLEEPING,
         }
    },
    .transit = [this](auto event, auto from_state, auto to_state)
    {
      // printf("WATCHDOG Transition %d -> %d\n", from_state, fsm_watchdog.state);
      if (event != HAS_DATA)
      {
         return;
      }

      last_peting = millis();

      uint8_t last_data;
      obtain(&last_data);

      auto accepted = parser.feed(last_data);

      if (accepted == ProtocolParser::feed_result_t::PARSED)
      {
        //  printf("accepted && received\n");
         fsm_watchdog(HAS_CMD);
      }
      else if (accepted == ProtocolParser::feed_result_t::ERROR)
      {
        //  printf("accepted && init\n");
         fsm_watchdog(RESET);
      }

      if (accepted != ProtocolParser::feed_result_t::NEED_MORE)
      {
        printf("ANUS %d\n", static_cast<int>(accepted));
        on_command(parser.msg_status(), parser.message(), parser.buffer(), parser.msg_len());
      }
    }};

  void on_command(const ProtocolParser::status_t result,
                  const ProtocolParser::message_t& msg,
                  const uint8_t* payload,
                  const uint8_t len)
  {
    printf("Parsed message: result=%s msg=%d, payload=%d\n",
           ProtocolParser::result_name(result),
           msg,
           len);
           if (message_callback)
           {
            message_callback(result, msg, payload, len);
           }
  }
public:

  using message_t = ProtocolParser::message_t;
  // using event_t = uint8_t;
  callback_t message_callback{nullptr};

  void tick() {
    fsm_watchdog(TICK);
  }
  void has_data()
  {
    fsm_watchdog(HAS_DATA);
  }
};