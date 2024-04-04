#include "aht20.hpp"
#include "aht30.hpp"
#include "bmp280.hpp"

#include <SensorProtocol.hpp>

#include <Arduino.h>
#include <array>

#ifdef ARDUINO_ARCH_LGT

#include <SoftwareSerial.h> /* for debug output */

#define WITH_SOFTWIRE 1
#define WITH_WIRE 2

#define WIRE_LIB WITH_SOFTWIRE

constexpr auto DEBUG_RX = A6;
constexpr auto DEBUG_TX = A7;

constexpr auto DEBUG_BAUND = 9600;
constexpr auto PROTO_BAUND = 9600;

// For early debug purpose, reuse normal serial
using fetch_func_t = uint8_t(*)(float*, float*);

enum SCL_BANK_T : uint8_t {
  SCL_BANK_0 = D8,
  SCL_BANK_1 = D11,
};

enum SDA_BANK_T : uint8_t {
  SDA_BANK_O = D9,
  SDA_BANK_1 = D10,
};

enum BANK_T : uint8_t {
  BANK_0 = 0,
  BANK_1 = 1
};

static void fetch_values(float *temp, float *hum, float *pressure);
static uint8_t detect_sensor();


// HardwareSerial &SerialDebug = Serial;
SoftwareSerial SerialDebug = SoftwareSerial(DEBUG_RX, DEBUG_TX);

static SensorProtocol prot_handler(SerialDebug);

static fetch_func_t extra_fetch = nullptr;

unsigned long last_forced;


#if WIRE_LIB == WITH_SOFTWIRE
#include <SoftWire.h>
static uint8_t buf_ic[8]; // max payload is 7 bytes
static std::array<SoftWire, 2> software_ic = {
  SoftWire{SDA_BANK_O, SCL_BANK_0},
  SoftWire{SDA_BANK_1, SCL_BANK_1},
};
#else
#error ???
#endif

static inline bool setupI2c();

static bool setupI2c()
{
  static_assert(SCL == A5, "SCL"); // zolty
  static_assert(SDA == A4, "SDA"); //zielony

#if WIRE_LIB == WITH_WIRE
  Wire.begin();
#elif WIRE_LIB == WITH_SOFTWIRE
  // Reuse same buffer
  // DON'T MIX BANK_X with BANK_Y
  // DON'T INTERLEAVE READ in begin/end transmission
  for (auto &ic: software_ic)
  {
    ic.setRxBuffer(buf_ic, sizeof(buf_ic));
    ic.setTxBuffer(buf_ic, sizeof(buf_ic));
    ic.begin();
  }

#else
#error ??
#endif

  return true;
}
#else
#error Unknown MCU
#endif


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
  const constexpr static uint8_t err_msg[] = {0x01 /* length 1*/, ProtocolParser::RET_ERROR /* id */, 0x12 /* CRC8 */};
  Serial.write(err_msg, sizeof(err_msg));
  Serial.flush();
}
static void emit_ok()
{
  const constexpr static uint8_t ok_msg[] = {0x01 /* length 1*/, ProtocolParser::RET_OK /* id */, 0x4F /* CRC8 */};
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
    tmp.crc8 = calc_crc8(reinterpret_cast<uint8_t*>(&tmp), sizeof(tmp) - 1);

    Serial.write(reinterpret_cast<uint8_t*>(&tmp), sizeof(tmp));
    Serial.flush();

    return;
  }

  SerialDebug.print("Unknown message: ");
  SerialDebug.println(cmd);

  emit_error();
}

static uint8_t detect_sensor()
{
  SerialDebug.println("Start detecting sensors");

  // at ic0 can only be aht20 (it's bmp280+aht20 combo)
  // so check for aht30 and then aht20 on ic1

  extra_fetch = nullptr;
  deinit_aht20();
  deinit_aht30();
  deinit_bmp280();

  for (uint8_t bank = 1; bank >= 0; --bank)
  {
    auto &ic = software_ic[bank];

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
  }

  if (extra_fetch == nullptr)
  {
    SerialDebug.println("AHTx0: not found");
  }

  // bmp280 can only be on bank0 (bmp280 + aht20 combo)
  if (bmp280_probe(&software_ic[BANK_0]) != 0)
  {
    SerialDebug.println("BMP280: not found");
    deinit_bmp280();

    return extra_fetch == nullptr ? 0 : 1;
  }
  else
  {
    SerialDebug.println("BMP280: ok");
    return extra_fetch == nullptr ? 1 : 2;
  }
}

void setup()
{
  setupI2c();

  prot_handler.message_callback = message_callback;

  Serial.begin(PROTO_BAUND);
  SerialDebug.begin(DEBUG_BAUND); // GPIO2 -> D4, [A6, A7 on lgt]

  while(!Serial) {
    delay(100);
  }

  delay(100);

  detect_sensor();
}

static void fetch_values(float *temp, float *hum, float *pressure)
{
  *temp = NAN;
  *hum = NAN;

  if (bmp280_fetch(temp, pressure) != 0)
  {
    SerialDebug.println("No BMP data");
    // try detecing sensors?
    *pressure = NAN;
  }

  if (extra_fetch != nullptr && extra_fetch(temp, hum))
  {
    SerialDebug.println("No AHTx0 data");
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
  if (Serial.available() > 0) {
    prot_handler.has_data();
  } else {
    prot_handler.tick();
    delay(1);
  }


  // if (millis() - last_forced > 10000) {
  //   last_forced = millis();
  //   float a, b, c;
  //   fetch_values(&a, &b, &c);
  // }
}