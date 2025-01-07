#ifndef I2C_BRIDGE_COMMON_DRIVER_H
#define I2C_BRIDGE_COMMON_DRIVER_H

#include <Arduino.h>
#include <array>
// #include <SoftWire.h>
#include <Wire.h>
#include <roo_collections/flat_small_hash_set.h>

// extern SoftWire theSoftWire;

static uint8_t dummy_uint8_t_no_op()
{
  return 0;
}

static void dummy_debug_print(const char *fmt, ...)
{
  // Serial1.println(__FUNCTION__);
  // va_list args;
  // va_start(args, fmt);
  // Serial1.printf(fmt, args);
  // va_end(args);
  Serial.println(fmt);
}

static void delay_ms(const uint32_t ms)
{
  delay(ms);
}

typedef uint8_t (*i2c_op)(uint8_t addr, uint8_t *buf, uint16_t len);
using i2c_write_op = i2c_op;
using i2c_read_op = i2c_op;

struct i2c_op_t
{
  i2c_read_op read;
  i2c_write_op write;
};

class Sensor
{
public:
  enum SensorType : uint8_t
  {
    NONE = 0,
    SHTxx = 1 << 0,
    SHTCx = 1 << 1,
    BMP280 = 1 << 2,
    BME280 = 1 << 3,
    AHTxx = 1 << 4,
  };
  Sensor() = default;
  virtual ~Sensor() = default;
  // virtual float temperature() { return 0; };
  // virtual float humidity() { return 0; };
  // virtual float pressure() { return 0; };
  virtual uint8_t begin();
  virtual uint8_t end();
  virtual uint8_t t_and_h(float *t, float *h);
  virtual constexpr const SensorType sensor_id() const;
};

typedef enum scl_bank_e : uint8_t
{
  SCL_BANK_0 = D2,
  SCL_BANK_1 = D3,
  SCL_BANK_2 = D5,
  SCL_BANK_3 = D6,
  SCL_BANK_4 = D7,
} scl_bank_t;

typedef enum sda_bank_e : uint8_t
{
  SDA_BANK_O = D4,
  SDA_BANK_1 = D4,
  SDA_BANK_2 = D4,
  SDA_BANK_3 = D4,
  SDA_BANK_4 = D4,
} sda_bank_t;


struct bank_t
{
  sda_bank_t sda : 4;
  scl_bank_t scl : 4;

  roo_collections::FlatSmallHashSet<Sensor *> sensors{};

  bank_t(sda_bank_t sda, scl_bank_t scl) : sda(sda), scl(scl)
  {
  }

  void begin()
  {
    Wire.setPins(sda, scl);

    Wire.begin();
  }

  bool has(Sensor &sensor)
  {
    return sensors.find(&sensor) != sensors.end();
  }
};

static size_t readBytes(uint8_t *buf, size_t len)
{
  if (Wire.available() < len)
  {
    return 0;
  }

  for (size_t i = 0; i < len; ++i)
  {
    buf[i] = Wire.read();
  }

  return len;
}

static uint8_t generic_i2c_read_cmd(uint8_t addr, uint8_t *buf, uint16_t len)
{
  (void)Wire.requestFrom((int)(addr >> 1), (int)len);
  auto d = readBytes(buf, len);

  return d != len;
}

static uint8_t generic_i2c_write_cmd(uint8_t addr, uint8_t *buf, uint16_t len)
{
  Wire.beginTransmission((addr >> 1));
  auto w = Wire.write(buf, len);

  if (Wire.endTransmission())
  {
    Serial.print(addr);
    Serial.print(" ");
    // Serial.print(Wire.getTimeout_ms());
    Serial.print(" ");
    Serial.println("fialed");
    return 1;
  }

  return w != len;
}

// TODO(m): Check if this is "generic" or "bmp" specific
template <typename reg_t = uint8_t>
static uint8_t generic_i2c_write_reg_cmd(uint8_t addr, reg_t reg, uint8_t *buf, uint16_t len)
{

  Wire.beginTransmission(addr >> 1);

  // if (sizeof(reg_t) == 2)
  // {
  //   Serial.print(reg);
  //   reg = (reg >> 8) | (reg << 8);
  //   Serial.print(" SWAP ");
  //   Serial.println(reg);
  // }

  auto w = Wire.write(reinterpret_cast<uint8_t *>(&reg), sizeof(reg_t));
  if (len != 0)
  {
    w += Wire.write(buf, len);
  }

  Wire.endTransmission();

  Serial.println(reg, 16);
  Serial.println(w != (len + sizeof(reg_t)));

  return w != (len + sizeof(reg_t));
}

template <typename reg_t = uint8_t>
static uint8_t generic_i2c_read_reg_cmd(uint8_t addr, reg_t reg, uint8_t *buf, uint16_t len)
{

  if (generic_i2c_write_reg_cmd(addr, reg, nullptr, 0))
  {
    Serial.println("NOPE");
    return -1;
  }

  auto r = Wire.requestFrom((int)(addr >> 1), (int)len);
  auto d = readBytes(buf, len);

  Serial.println(d != len);
  return d != len;
}

#endif