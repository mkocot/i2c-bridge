
#include "aht20.hpp"
#include "aht30.hpp"
#include <bank.hpp>
#include "bmp280.hpp"
#include "common_driver.hpp"
#include "config.hpp"
#include "shtxx.hpp"

#include <Adafruit_MAX31865.h>
#include <Arduino.h>
#include <array>
#include <FlexWire.h>
#include <LinkedList.h>
#include <SensorProtocol.hpp>
#include <SoftwareSerial.h> /* for debug output */
#include <SHTSensor.h>
// #include <SoftWire.h>
#include <TaskScheduler.h>
#include <PrintEx.h>
#include <Wire.h>

#define WITH_SOFTWIRE 1
#define WITH_WIRE 2
#define WITH_SENSOR_MUNCHING 1

constexpr auto BANKS = 4;

constexpr auto DEBUG_RX = A6;
constexpr auto DEBUG_TX = A7;

constexpr auto DEBUG_BAUND = 9600;
constexpr auto PROTO_BAUND = 9600;

typedef struct
{
  float temp;
  float hum;
  float pres;
} reading_t;

std::array<bank_t, BANKS> software_i2c = {
    bank_t{SDA_BANK_O, SCL_BANK_0},
    bank_t{SDA_BANK_1, SCL_BANK_1},
    bank_t{SDA_BANK_2, SCL_BANK_2},
    bank_t{SDA_BANK_3, SCL_BANK_3},
    // bank_t{SDA_BANK_4, SCL_BANK_4},
};

static void fetch_values(LinkedList<reading_t> &readings);
static uint8_t detect_sensor();
static void led_on()
{
  digitalWrite(LED_BUILTIN, HIGH);
}

static void led_off()
{
  digitalWrite(LED_BUILTIN, LOW);
}

/* initialize "Wire" to custom FlexiWire */
TwoWire Wire{};

#if X_DEBUG
Stream &SerialDebug = Serial;
#endif
// SoftwareSerial SerialDebug = SoftwareSerial(DEBUG_RX, DEBUG_TX);

static SensorProtocol prot_handler{};
static Scheduler scheduler{};
static Aht2x aht2x{};
static Aht3x aht3x{};
static Bmp280 bmp280{};
static Shtxx<SHTSensor::SHT3X> sht3x{};
static Shtxx<SHTSensor::SHT4X> sht4x{};
static Shtxx<SHTSensor::SHTC3> shtc3{};
constexpr auto RNOMINAL = 100; // Ohm
constexpr auto RREF = 430;     // Ohm
static Adafruit_MAX31865 pt100{SS};
static bool pt100_detected = false;

#if WITH_SENSOR_MUNCHING
static Task task_forced_measure{TASK_SECOND * 10, TASK_FOREVER, []()
                                {
                                  static LinkedList<reading_t> tmp;
                                  fetch_values(tmp);
                                },
                                &scheduler};
#endif

static Task task_led_blink{TASK_SECOND / 4, TASK_FOREVER, []()
                           {
                             if (task_led_blink.getRunCounter() & 1)
                             {
                               // 1, 3, ...
                               led_on();
                             }
                             else
                             {
                               // 0, 2, 4, ...
                               led_off();
                             }
                           },
                           &scheduler};

extern int obtain(uint8_t *data)
{
  auto s = Serial.readBytes(data, 1);
  // Serial.printf("obtain: %d %d\n", s, *data);
  return s;
}

// HW I2C
//      ESP8266 LGT
// SDA  D2      A4
// SCL  D1      A5

// static SCL_BANK_T aht_bank;

static void emit_error()
{
  const constexpr static uint8_t err_msg[] = {0x01 /* length 1*/, ProtocolParser::RET_ERROR /* id */, 0x74 /* CRC8 */};
  Serial.write(err_msg, sizeof(err_msg));
  Serial.flush();
}
static void emit_ok()
{
  const constexpr static uint8_t ok_msg[] = {0x01 /* length 1*/, ProtocolParser::RET_OK /* id */, 0xA1 /* CRC8 */};
  Serial.write(ok_msg, sizeof(ok_msg));
  Serial.flush();
}
static void message_callback(ProtocolParser::status_t result, ProtocolParser::message_t cmd, const uint8_t *buff, uint8_t buff_len)
{
  debug_println("callback");
  if (result != ProtocolParser::status_t::OK)
  {
    emit_error();

    return;
  }

  // GET_DETECT
  if (cmd == ProtocolParser::GET_DETECT)
  {
    if (detect_sensor() == 0)
    {
      emit_error(); // nothing detected
    }
    else
    {
      emit_ok(); // something detected
    }

    return;
  }

  // GET_SENSOR_DATA
  if (cmd == ProtocolParser::GET_SENSOR_DATA)
  {
    // message:
    //   uint8_t msg_len
    //   uint8_t msg_id = ProtocolParser::RET_SENSOR_DATA;
    //   reading_container_t readings;
    //   uint8_t crc8;
    // } __attribute__((packed));

    // static_assert(sizeof(std::array<std::array<float, 3>, 5>) == 3 * 4 * 5, "XX");
    // static_assert(sizeof(message_frame) == 15, "X");

    // message_frame tmp{};

    static LinkedList<reading_t> readings;
    fetch_values(readings);

    /* WITHOUT CRC8 */
    uint8_t message_len = 1 + sizeof(float) * 3 * readings.size();
    Serial.write(&message_len, 1);
    uint8_t crc8 = crc8_dvb_s2(&message_len, 1);

    uint8_t tmp = ProtocolParser::RET_SENSOR_DATA;
    Serial.write(&tmp, 1);
    crc8 = crc8_dvb_s2_stream(crc8, &tmp, 1);

    while (readings.size())
    {
      auto r = readings.shift();

      Serial.write(reinterpret_cast<uint8_t *>(&r), sizeof(r));
      crc8 = crc8_dvb_s2_stream(crc8, reinterpret_cast<uint8_t *>(&r), sizeof(r));
    }

    Serial.write(&crc8, 1);
    Serial.flush();

    return;
  }

  debug_println("Unknown message: ", cmd);

  emit_error();
}

enum class scan_result
{
  OK,
  INVALID_RANGE,
  NOT_FOUND,
};

static scan_result scan_i2c_inner(FlexWire &sw, const uint8_t from, uint8_t &found)
{
  if (from == 0 || from >= 127)
  {
    return scan_result::INVALID_RANGE;
  }

  for (uint8_t a = from; a < 127; ++a)
  {
    sw.beginTransmission(a);
    if (sw.endTransmission())
    {
      // Error -> nothing detected
      continue;
    }

    found = a;

    return scan_result::OK;
  }

  return scan_result::NOT_FOUND;
}

enum class sensor_id_t : uint8_t
{
  AHTxx = 56,
  SHTxx = 68,
  SHTCx = 112,
  BME280 = 118,
  BMP280 = 119,
};

static scan_result scan_i2c(FlexWire &sw, uint8_t from, uint8_t &found)
{
  // 56  - AHTxx
  // 68  - SHT3x / SHT4x
  // 112 - SHTC3
  // 118 - BME280
  // 119 - BMP280
  if (from <= 56)
  {
    from = 56;
  }
  else if (from <= 68)
  {
    from = 68;
  }
  else if (from <= 112)
  {
    from = 112;
  }
  else if (from <= 118)
  {
    from = 118;
  }
  else if (from <= 119)
  {
    from = 119;
  }
  else
  {
    return scan_result::NOT_FOUND;
  }
  // NOTE(m): setTimeout is for Stream Read/Write not I2C transmission
  // auto originalTimeout = sw.getTimeout_ms();

  // sw.setTimeout_ms(2);

  auto result = scan_i2c_inner(sw, from, found);

  // sw.setTimeout_ms(originalTimeout);

  return result;
}

enum class sensor_result_t: uint8_t
{
  NEW,
  OLD,
  ERROR,
};

static sensor_result_t obtain_sensor(uint8_t address, const bank_t &bank, Sensor **sensor)
{
  switch (static_cast<sensor_id_t>(address))
  {
  case sensor_id_t::AHTxx:
    /* does it has aht30 or aht20 ? */
    if (bank.has(aht2x) || bank.has(aht3x))
    {
      debug_println("AHT[2|3]x already added");
      return sensor_result_t::OLD;
    }

    aht2x.end();
    if (!aht2x.begin())
    {
      debug_println("AHT2x detected");
      *sensor = &aht2x;

      return sensor_result_t::NEW;
    }

    /* looks like i dont have this sensor on shelf */
    aht3x.end();
    if (!aht3x.begin())
    {
      debug_println("AHT3x detected");
      *sensor = &aht3x;

      return sensor_result_t::NEW;
    }
    
    break;
  case sensor_id_t::BMP280:
    if (bank.has(bmp280))
    {
      debug_println("BMP already added");

      return sensor_result_t::OLD;
    }

    bmp280.end();
    if (!bmp280.begin())
    {
      debug_println("BMP280 detected");

      *sensor = &bmp280;

      return sensor_result_t::NEW;
    }

    debug_println("BMP280 add failed");

    break;
  case sensor_id_t::BME280:
    debug_println("BME280 detected *unsupported");
    break;
  case sensor_id_t::SHTCx:
    if (bank.has(shtc3))
    {
      debug_println("SHTC3 already known");

      return sensor_result_t::OLD;
    }

    if (!shtc3.begin())
    {
      debug_println("SHTCx detected");
      *sensor = &shtc3;

      return sensor_result_t::NEW;
    }
    else
    {
      debug_println("Initializing SHTC3 failed");
    }
    break;
  case sensor_id_t::SHTxx:
    if (bank.has(sht3x) || bank.has(sht4x))
    {
      debug_println("SHTxx already known");
      
      return sensor_result_t::OLD;
    }

    /* see sensiron source why this order is important */
    if (!sht4x.begin())
    {
      debug_println("SHT4x detected");

      *sensor =  &sht4x;
      
      return sensor_result_t::NEW;
    }

    if (!sht3x.begin())
    {
      debug_println("SHT3x detected");
      *sensor =  &sht3x;

      return sensor_result_t::NEW;
    }

    debug_println("Initializing SHTXX failed");

    break;
  default:
    return sensor_result_t::ERROR;
  }
}

static uint8_t detect_sensor()
{
  debug_println("Start detecting sensors");

  task_led_blink.disable();
  led_on();

  // 0 1 4 2 3
  // 56  - AHTxx
  // 68  - SHT3x / SHT4x
  // 112 - SHTC3
  // 118 - BME280
  // 119 - BMP280

  // 89  - Gas sensor (VOC)

  uint8_t detected = 0;
  uint8_t bank_id = 0;
  for (auto &bank : software_i2c)
  {
    bank.begin();

    uint8_t address = 1;

    debug_println("Start detecting ", bank_id, " ", address);

    while (scan_i2c(Wire, address, address) == scan_result::OK)
    {
      debug_println("Found device on bank_", bank_id, " and address: ", address);

      Sensor *sensor;
      switch(obtain_sensor(address, bank, &sensor))
      {
        case sensor_result_t::ERROR:
          continue; /* outer loop */
        case sensor_result_t::OLD:
          ++detected;
          continue; /* outer loop */
        case sensor_result_t::NEW:
          break;
      }

      assert(sensor != nullptr);

      ++address;
      ++detected;

      debug_println("Inserint sensor");

      bank.sensors.insert(sensor);
    }

    ++bank_id;
  }

  // check PT100
  if (!pt100_detected)
  {
    if (pt100.begin(MAX31865_3WIRE))
    {
      pt100_detected = true;
      ++detected;
    }
  }

  return !detected;
}

void setup()
{
  prot_handler.message_callback = message_callback;

  Serial.begin(PROTO_BAUND);
  // SerialDebug.begin(DEBUG_BAUND); // GPIO2 -> D4, [A6, A7 on lgt]

  while (!Serial)
  {
    delay(100);
  }

  delay(100);

  detect_sensor();

  #if WITH_SENSOR_MUNCHING
  task_forced_measure.enable();
  #endif
}

static void fetch_values(LinkedList<reading_t> &readings) // float *temp, float *hum, float *pressure)
{
  readings.clear();

  for (auto &bank : software_i2c)
  {
    if (bank.sensors.empty())
    {
      continue;
    }

    bank.begin();

    for (auto &s : bank.sensors)
    {
      reading_t r{NAN, NAN, NAN};
      uint8_t result = 1;
      if (s->sensor_id() == Sensor::BMP280)
      {
        // t_and_h will return temperature and pressure
        result = s->t_and_h(&r.temp, &r.pres);
      }
      else
      {
        result = s->t_and_h(&r.temp, &r.hum);
      }

      if (result)
      {
        bank.sensors.erase(s);
        debug_println("Erase sensor", s->sensor_id());

        continue;
      }

      debug_println(millis(), " t=", r.temp, " h=", r.hum, " p=", r.pres);

      readings.add(r);
    }
  }
  // now attempt to read PT100@SPI
  if (pt100_detected)
  {
    float t = pt100.temperature(RNOMINAL, RREF);
    if (pt100.readFault())
    {
      debug_println("PT100 FAULT");
      pt100_detected = false;
    }
    else
    {
      readings.add(reading_t{t, NAN, NAN});
    }
    pt100.clearFault();
  }

  if (!readings.size())
  {
    // Start blinking if NAN is received
    task_led_blink.enable();
  }
  else
  {
    // Stop blinking if all ok
    led_off();
    task_led_blink.disable();
  }
}

void loop()
{
  scheduler.execute();

  if (Serial.available() > 0)
  {
    prot_handler.has_data();
  }
  else
  {
    prot_handler.tick();
    delay(1);
  }
}