#include "ProtocolParser.hpp"
#if ARDUINO
#include <Arduino.h>
#else

class Stream
{
  public:
  void print(const char* s){
    printf("%s", s);
  }
  void print(const uint8_t d)
  {
    printf("%d", d);
  }
  void println(const char* s)
  {
    printf("%s\n", s);
  }
  void println(const int n)
  {
    printf("%d\n", n);
  }
};
extern unsigned long millis();
#endif
extern int obtain(uint8_t *data);

typedef void (*callback_t) (ProtocolParser::status_t, ProtocolParser::message_t, const uint8_t *, uint8_t);
#define SENSOR_PROTOCOL_DEBUG ((x)) while {} do(0)
//#define SENSOR_PROTOCOL_DEBUG ((x)) printf((x))
class SensorProtocol
{
  private:
  Stream &debug;
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


  /* commands */
  static constexpr unsigned long WATCHDOG_TIME = 1000;
  unsigned long last_peting;
  ProtocolParser parser;
  watchdog_state_t state = SLEEPING;

  // WITHOUT CRC8
#if 0
  using fsm_watchdog_t = fsm::statemachine<watchdog_event_t, watchdog_state_t>;
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
            //  printf("BARKING \n");
             fsm_watchdog(RESET);
             // Should it be event on it's own? Or just hard reset
             // by hand?
            //  printf("!! RESET TO INIT !!\n");
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
        on_command(parser.msg_status(), parser.message(), parser.buffer(), parser.msg_len());
      }
    }};
#endif 
  void on_command(const ProtocolParser::status_t result,
                  const ProtocolParser::message_t& msg,
                  const uint8_t* payload,
                  const uint8_t len)
  {
    debug.print("Parsed messag: result=");
    debug.print(result);
    debug.print(" msg=");
    debug.print(msg);
    debug.print(" payload=");
    debug.println(len);
    if (message_callback)
    {
      message_callback(result, msg, payload, len);
    }
  }
public:

SensorProtocol(Stream &debug_output): debug(debug_output)
{
}

  using message_t = ProtocolParser::message_t;
  // using event_t = uint8_t;
  callback_t message_callback{nullptr};

  void tick() {
    if (state != AWAKE)
    {
      return;
    }

    if (millis() - last_peting > WATCHDOG_TIME)
    {
      state = BAKRING;
      //  printf("BARKING \n");
      // Should it be event on it's own? Or just hard reset
      // by hand?
      //  printf("!! RESET TO INIT !!\n");
      parser.reset();

      on_command(ProtocolParser::TIMEOUT, parser.message(), nullptr, 0);
      state = SLEEPING;
    }
  }

  void has_data()
  {
    state = AWAKE;

    last_peting = millis();

    uint8_t last_data;

    obtain(&last_data);

    auto accepted = parser.feed(last_data);

    debug.print("Has data: ");
    debug.print(last_data);
    debug.print(" accepted: ");
    debug.println((int)accepted);

    if (accepted == ProtocolParser::feed_result_t::PARSED)
    {
      //  printf("accepted && received\n");
      state = SLEEPING;
    }
    else if (accepted == ProtocolParser::feed_result_t::ERROR)
    {
      //  printf("accepted && init\n");
      state = SLEEPING;
    }

    if (accepted != ProtocolParser::feed_result_t::NEED_MORE)
    {
      on_command(parser.msg_status(), parser.message(), parser.buffer(), parser.msg_len());
    }
  }
};