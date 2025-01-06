
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

#define WITH_SOFTWIRE 1
#define WITH_WIRE 2

constexpr auto DEBUG_RX = A6;
constexpr auto DEBUG_TX = A7;

constexpr auto DEBUG_BAUND = 9600;
constexpr auto PROTO_BAUND = 9600;

// For early debug purpose, reuse normal serial
using fetch_func_t = uint8_t (*)(float *, float *);

enum BANK_T : uint8_t
{
  BANK_0 = 0,
  BANK_1 = 1,
  BANK_2 = 2,
  BANK_3 = 3,
};

enum SCL_BANK_T : uint8_t
{
  SCL_BANK_0 = D2,
  SCL_BANK_1 = D3,
  SCL_BANK_2 = D5,
  SCL_BANK_3 = D6,
  SCL_BANK_4 = D7,
};

enum SDA_BANK_T : uint8_t
{
  SDA_BANK_O = D4,
  SDA_BANK_1 = D4,
  SDA_BANK_2 = D4,
  SDA_BANK_3 = D4,
  SDA_BANK_4 = D4,
};

std::array<bank_t, 5> software_i2c = {
    bank_t{SDA_BANK_O, SCL_BANK_0},
    bank_t{SDA_BANK_1, SCL_BANK_1},
    bank_t{SDA_BANK_2, SCL_BANK_2},
    bank_t{SDA_BANK_3, SCL_BANK_3},
    bank_t{SDA_BANK_4, SCL_BANK_4},
};

static void fetch_values(float *temp, float *hum, float *pressure);
static uint8_t detect_sensor();
static void led_on()
{
  digitalWrite(LED_BUILTIN, HIGH);
}

static void led_off()
{
  digitalWrite(LED_BUILTIN, LOW);
}

TwoWire Wire{};

HardwareSerial &SerialDebug = Serial;
// SoftwareSerial SerialDebug = SoftwareSerial(DEBUG_RX, DEBUG_TX);

static SensorProtocol prot_handler(SerialDebug);

static fetch_func_t extra_fetch = nullptr;

static Scheduler scheduler{};

static Aht2x aht2x{};
static Aht3x aht3x{};
static Bmp280 bmp280{};
static Sht3x sht3x{};
static Sht4x sht4x{};

static Task task_forced_measure{TASK_SECOND * 10, TASK_FOREVER, []()
                                {
                                  float tmp;
                                  fetch_values(&tmp, &tmp, &tmp);
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
    struct message_frame
    {
      uint8_t msg_len = sizeof(float) * 3 + 1;
      uint8_t msg_id = ProtocolParser::RET_SENSOR_DATA;
      float temp;
      float hum;
      float pres;
      uint8_t crc8;
    } __attribute__((packed));

    static_assert(sizeof(message_frame) == 15, "X");

    float temp, hum, pres;

    fetch_values(&temp, &hum, &pres);

    message_frame tmp{};

    tmp.temp = temp;
    tmp.hum = hum;
    tmp.pres = pres;
    tmp.crc8 = crc8_dvb_s2(reinterpret_cast<uint8_t *>(&tmp), sizeof(tmp) - 1);

    Serial.write(reinterpret_cast<uint8_t *>(&tmp), sizeof(tmp));
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

  // at ic0 can only be aht20 (it's bmp280+aht20 combo)
  // so check for aht30 and then aht20 on ic1

  extra_fetch = nullptr;
  // deinit_aht20();
  // deinit_aht30();
  // deinit_bmp280();

  // 0 1 4 2 3
  // 56  - AHTxx
  // 68  - SHT3x / SHT4x
  // 112 - SHTC3
  // 118 - BME280
  // 119 - BMP280

  // 89  - Gas sensor (VOC)

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
        SerialDebug.println("SHTCx detected *unsupported");
        break;
      case sensor_id_t::SHTxx:
        if (bank.has(sht3x) || bank.has(sht4x))
        {
          SerialDebug.println("SHTxx already known");
          break;
        }

        if (!sht3x.begin())
        {
          SerialDebug.println("SHT3x detected");
          sensor = &sht3x;
          break;
        }

        if (!sht4x.begin())
        {
          SerialDebug.println("SHT4x detected");
          sensor = &sht4x;
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

      SerialDebug.println("Inserint sensor");

      bank.sensors.insert(sensor);
    }

    ++bank_id;

#if 0
    if (bank == BANK_1)
    {
      // 1) Check AHT30 (and deinit on failure)
      if(aht30_probe(&ic) == 0)
      {
        extra_fetch = aht30_fetch;
        SerialDebug.print("AHT30: at ");
        SerialDebug.println(bank);
        // Found AHT30, exit loop
        break;
      }
      // AHT30 can only be on BANK_1
      // deinit if not found
      deinit_aht30();
    }

    // 2) Check AHT20
    if (aht20_probe(&ic) == 0)
    {
      extra_fetch = aht20_fetch;
      SerialDebug.print("AHT20: at ");
      SerialDebug.println(bank);
      break;
    }
    else if (bank == BANK_0)
    {
      // Deinit AHT20 if not found
      deinit_aht20();
    }
#endif
  }
#if 0
  uint8_t detected_sensors = 0;

  if (extra_fetch == nullptr)
  {
    SerialDebug.println("AHTx0: not found");
  }
  else
  {
    ++detected_sensors;
  }

  // bmp280 can only be on bank0 (bmp280 + aht20 combo)
  if (bmp280_probe(&getBank(BANK_0)) != 0)
  {
    SerialDebug.println("BMP280: not found");
    deinit_bmp280();
  }
  else
  {
    ++detected_sensors;
    SerialDebug.println("BMP280: ok");
  }

  if (detected_sensors == 2)
  {
    led_off();
  }
  else if (detected_sensors == 1)
  {
    task_led_blink.enable();
  }
  // keep led on otherwise

  return detected_sensors;
#endif

  return 1;
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

  software_i2c[BANK_2].begin();

  delay(100);

  SHTSensor sensor{SHTSensor::AUTO_DETECT};

  while (1)
  {
    // Reset sensor type to prevent stucking in first-detected sensor
    sensor.mSensorType = SHTSensor::AUTO_DETECT;
    if (!sensor.init())
    {
      Serial.println("init failed");
    }
    else
    {

    Serial.print("Sensor type: ");
    Serial.println(sensor.mSensorType);

    if (sensor.readSample())
    {

    float h = sensor.getHumidity();
    float t = sensor.getTemperature();
    Serial.print(h);
    Serial.print(" ");
    Serial.println(t);
    }
    else
    {
      Serial.println("reading sample failed");
    }
    }

    delay(10000);
    
  }
  

  // task_forced_measure.enable();

  while (1)
  {
    detect_sensor();
    for (int i = 0; i < software_i2c.size(); ++i)
    {
      auto &bank = software_i2c[i];
      if (bank.sensors.empty())
      {
        SerialDebug.print("Bank ");
        SerialDebug.print(i);
        SerialDebug.println(" is empty");
        continue;
      }
      bank.begin();

      float t, h;
      for (auto s : bank.sensors)
      {
        if (s->t_and_h(&t, &h))
        {
          SerialDebug.println("Error with sensor, deinit");
          s->end();
          bank.sensors.erase(s);
          continue;
        }
        SerialDebug.print("Bank ");
        SerialDebug.print(i);
        SerialDebug.print(" data: ");
        SerialDebug.print(t);
        SerialDebug.print(" ");
        SerialDebug.println(h);
      }
    }
    SerialDebug.println("BLOCKED START");
    delay(10000);
  }
}

static void fetch_values(float *temp, float *hum, float *pressure)
{
  *temp = NAN;
  *hum = NAN;

  // if (bmp280_fetch(temp, pressure) != 0)
  // {
  //   SerialDebug.println("No BMP data");
  //   // try detecing sensors?
  //   *pressure = NAN;
  // }

  if (extra_fetch != nullptr && extra_fetch(temp, hum))
  {
    SerialDebug.println("No AHTx0 data");
  }

  if (*temp == NAN || *hum == NAN || *pressure == NAN)
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

  SerialDebug.print(millis());
  SerialDebug.print(" t=");
  SerialDebug.print(*temp);
  SerialDebug.print(" h=");
  SerialDebug.print(*hum);
  SerialDebug.print(" p=");
  SerialDebug.println(*pressure);
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