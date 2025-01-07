
#include "aht20.hpp"
#include "aht30.hpp"
#include "bmp280.hpp"
#include "shtxx.hpp"

#include <Arduino.h>
#include <array>
#include <SensorProtocol.hpp>
#include <SoftwareSerial.h> /* for debug output */
// #include <SoftWire.h>
#include <TaskScheduler.h>
#include <PrintEx.h>
#include <Wire.h>
#include <FlexWire.h>
#include <SHTSensor.h>
#include <LinkedList.h>

#define WITH_SOFTWIRE 1
#define WITH_WIRE 2

constexpr auto BANKS = 4;

constexpr auto DEBUG_RX = A6;
constexpr auto DEBUG_TX = A7;

constexpr auto DEBUG_BAUND = 9600;
constexpr auto PROTO_BAUND = 9600;

// For early debug purpose, reuse normal serial
using fetch_func_t = uint8_t (*)(float *, float *);

typedef struct
{
  float temp;
  float hum;
  float pres;
} reading_t;

using reading_container_t = std::array<reading_t, 5>;

enum BANK_T : uint8_t
{
  BANK_0 = 0,
  BANK_1 = 1,
  BANK_2 = 2,
  BANK_3 = 3,
};

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

HardwareSerial &SerialDebug = Serial;
// SoftwareSerial SerialDebug = SoftwareSerial(DEBUG_RX, DEBUG_TX);

static SensorProtocol prot_handler(SerialDebug);

static Scheduler scheduler{};

static Aht2x aht2x{};
static Aht3x aht3x{};
static Bmp280 bmp280{};
static Shtxx<SHTSensor::SHT3X> sht3x{};
static Shtxx<SHTSensor::SHT4X> sht4x{};
static Shtxx<SHTSensor::SHTC3> shtc3{};

static Task task_forced_measure{TASK_SECOND * 10, TASK_FOREVER, []()
                                {
                                  static LinkedList<reading_t> tmp;
                                  fetch_values(tmp);
                                },
                                &scheduler};

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

// static uint8_t buf_ic[8]; // max payload is 7 bytes

// /* extern */ FlexWire theSoftWire{SDA_BANK_O, SCL_BANK_0};
static inline bool setupI2c();

static bool setupI2c()
{
  // Reuse same buffer
  // DON'T MIX BANK_X with BANK_Y
  // DON'T INTERLEAVE READ in begin/end transmission

  // Wire.setRxBuffer(buf_ic, sizeof(buf_ic));
  // Wire.setTxBuffer(buf_ic, sizeof(buf_ic));
  // Wire.setTimeout_ms(2000);

  return true;
}

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
  SerialDebug.println("callback");
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

  SerialDebug.print("Unknown message: ");
  SerialDebug.println(cmd);

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
static uint8_t detect_sensor()
{
  SerialDebug.println("Start detecting sensors");

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

    SerialDebug.print("Start detecting ");
    SerialDebug.print(bank_id);
    SerialDebug.print(" ");
    SerialDebug.println(address);

    while (scan_i2c(Wire, address, address) == scan_result::OK)
    {
      SerialDebug.print("Found device on bank_");
      SerialDebug.print(bank_id);
      SerialDebug.print(" and address: ");
      SerialDebug.println(address);

      Sensor *sensor = nullptr;

      switch (static_cast<sensor_id_t>(address))
      {
      case sensor_id_t::AHTxx:
        /* does it has aht30 or aht20 ? */
        if (bank.has(aht2x) || bank.has(aht3x))
        {
          ++detected;
          SerialDebug.println("AHT[2|3]x already added");
          break;
        }

        aht2x.end();
        if (!aht2x.begin())
        {
          SerialDebug.println("AHT2x detected");
          sensor = &aht2x;
          break;
        }

        /* looks like i dont have this sensor on shelf */
        aht3x.end();
        if (!aht3x.begin())
        {
          SerialDebug.println("AHT3x detected");
          sensor = &aht3x;
        }

        break;
      case sensor_id_t::BMP280:
        if (bank.has(bmp280))
        {
          ++detected;
          SerialDebug.println("BMP already added");
          break;
        }

        bmp280.end();
        if (bmp280.begin())
        {
          SerialDebug.println("BMP280 add failed");
          break;
        }
        SerialDebug.println("BMP280 detected");

        break;
      case sensor_id_t::BME280:
        SerialDebug.println("BME280 detected *unsupported");
        break;
      case sensor_id_t::SHTCx:
        if (bank.has(shtc3))
        {
          ++detected;
          SerialDebug.println("SHTC3 already known");
          break;
        }
        if (!shtc3.begin())
        {
          SerialDebug.println("SHTCx detected");
          sensor = &shtc3;
        }
        else
        {
          SerialDebug.println("Initializing SHTC3 failed");
        }
        break;
      case sensor_id_t::SHTxx:
        if (bank.has(sht3x) || bank.has(sht4x))
        {
          ++detected;
          SerialDebug.println("SHTxx already known");
          break;
        }

        /* see sensiron source why this order is important */
        if (!sht4x.begin())
        {
          SerialDebug.println("SHT4x detected");
          sensor = &sht4x;
          break;
        }

        if (!sht3x.begin())
        {
          SerialDebug.println("SHT3x detected");
          sensor = &sht3x;
          break;
        }

        SerialDebug.println("Initializing SHTXX failed");

        break;
      default:
        break;
      }

      ++address;

      if (sensor == nullptr)
      {
        continue;
      }

      ++detected;
      SerialDebug.println("Inserint sensor");

      bank.sensors.insert(sensor);
    }

    ++bank_id;
  }

  return !detected;
}

void setup()
{
  setupI2c();

  prot_handler.message_callback = message_callback;

  Serial.begin(PROTO_BAUND);
  // SerialDebug.begin(DEBUG_BAUND); // GPIO2 -> D4, [A6, A7 on lgt]

  while (!Serial)
  {
    delay(100);
  }

  delay(100);

  detect_sensor();
  task_forced_measure.enable();
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
        SerialDebug.print("Erase sensor");
        SerialDebug.println(s->sensor_id());

        continue;
      }

      SerialDebug.print(millis());
      SerialDebug.print(" t=");
      SerialDebug.print(r.temp);
      SerialDebug.print(" h=");
      SerialDebug.print(r.hum);
      SerialDebug.print(" p=");
      SerialDebug.println(r.pres);

      readings.add(r);
    }
  }
  // now attempt to read PT100@SPI
  // TODO(m): PT100 reading

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