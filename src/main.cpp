#include <SensorProtocol.hpp>

#include <Arduino.h>
#include <Wire.h>
// #include <SimpleCLI.h>
// #include <SPI.h>
// #include <SPISlave.h>
#include <driver_aht20.h>
#include <interface/driver_aht20_interface.h>
#include <driver_bmp280.h>
#include <interface/driver_bmp280_interface.h>

#ifdef ARDUINO_ARCH_ESP8266
#define A6 (D5)
#define A7 (-1)
#define D11 (-1)
#define D12 (-1)
#define D13 (-1)
#endif
// Choosen becase they fit nicely on front of board
// [                D13]
// [                D12]
// [                D11]
// [A4  A5  A6  A7  D10]
// A4 - SDA
// A5 - SCL
// A6 - SCL_BANK_1 (bit-bang)
// A7 - SCL_BANK_2 (bit-bang)
// D10..D13 as more alternate I2C, they will nicely create long line
// one after another
constexpr auto SCL_BANK_0 = SCL;
constexpr auto SCL_BANK_1 = A6;
constexpr auto SCL_BANK_2 = A7;
constexpr auto SCL_BANK_3 = D10;
constexpr auto SCL_BANK_4 = D11;
constexpr auto SCL_BANK_5 = D12;
constexpr auto SCL_BANK_6 = D13;
static void fetch_values(float *temp, float *hum, float *pressure);

// Only D2 and D3 supports INT interrupts
// we might later choose PCINT if beneficial
constexpr auto CMD_PIN = D2;

extern int obtain(uint8_t *data)
{
  auto s = Serial.readBytes(data, 1);
  Serial.printf("obtain: %d %d\n", s, *data);
  return s;
}

constexpr auto AHT20_ADDRESS = 0x38;

aht20_handle_t *aht20{nullptr};
bmp280_handle_t *bmp280{nullptr};


uint8_t aht20_interface_iic_init()
{
  // NO-OP
  return 0;
}

uint8_t aht20_interface_iic_deinit()
{
  return 0;
}

uint8_t aht20_interface_iic_read_cmd(uint8_t addr, uint8_t *buf, uint16_t len)
{
  // For some obscure reasons, addr is shifted by 1 to right
  auto r = Wire.requestFrom((int)(addr >> 1), (int)len);
  auto d = Wire.readBytes(buf, len);
  return d != len;
}

uint8_t aht20_interface_iic_write_cmd(uint8_t addr, uint8_t *buf, uint16_t len)
{
  // For some obscure reasons, addr is shifted by 1 to right
  Wire.beginTransmission((addr >> 1));
  auto w = Wire.write(buf, len);
  Wire.endTransmission();
  return w != len;
}

void aht20_interface_delay_ms(uint32_t ms)
{
  delay(ms);
}

void aht20_interface_debug_print(const char *fmt, ...)
{
  // Serial.println(__FUNCTION__);
  // va_list args;
  // va_start(args, fmt);
  // Serial.printf(fmt, args);
  // va_end(args);
}

uint8_t bmp280_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  if (bmp280_interface_iic_write(addr, reg, nullptr, 0))
  {
    return -1;
  }

  auto r = Wire.requestFrom((int)(addr >> 1), (int)len);
  auto d = Wire.readBytes(buf, len);
  return d != len;
}

uint8_t bmp280_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  Wire.beginTransmission(addr >> 1);
  auto w = Wire.write(reg);
  if (len != 0) {
    w += Wire.write(buf, len);
  }
  Wire.endTransmission();
  return w != (len + 1u);
}

// I2C
//      ESP8266 LGT
// SDA  D2      A4
// SCL  D1      A5

static uint8_t noop(uint8_t x, uint8_t *xx, uint16_t xxx)
{
  return 0;
}

static SensorProtocol prot_handler;
static int aht_bank;


void setup()
{
  prot_handler.message_callback = [](auto result, auto cmd, auto buff, auto buff_len)
  {
    if (result != ProtocolParser::status_t::OK || cmd != ProtocolParser::GET_SENSOR_DATA)
    {
      uint8_t err_msg[] = {0x01 /* length 1*/, 0x02 /* id RET_ERROR */, 0x17 /* CRC8 */};
      Serial.write(err_msg, sizeof(err_msg));
      Serial.flush();
      return;
    }

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

    message_frame tmp = {
      .temp = temp,
      .hum = hum,
      .pres = pres,
    };
    tmp.crc8 = calc_crc8(reinterpret_cast<uint8_t*>(&tmp), sizeof(tmp) - 1);

    Serial.write(reinterpret_cast<uint8_t*>(&tmp), sizeof(tmp));
    Serial.flush();
  };
  Serial.begin(9600);
  while(!Serial) {
    delay(100);
  }
  delay(2000);

  Serial.println("Start detecting sensors");

  Serial.println("Detecting AHTx0 at BANK1 ADDR");
  aht20 = new aht20_handle_t;// Adafruit_AHTX0();
  DRIVER_AHT20_LINK_INIT(aht20, aht20_handle_t);
  DRIVER_AHT20_LINK_IIC_INIT(aht20, aht20_interface_iic_init);
  DRIVER_AHT20_LINK_IIC_DEINIT(aht20, aht20_interface_iic_deinit);
  DRIVER_AHT20_LINK_IIC_READ_CMD(aht20, aht20_interface_iic_read_cmd);
  DRIVER_AHT20_LINK_IIC_WRITE_CMD(aht20, aht20_interface_iic_write_cmd);
  DRIVER_AHT20_LINK_DELAY_MS(aht20, aht20_interface_delay_ms);
  DRIVER_AHT20_LINK_DEBUG_PRINT(aht20, aht20_interface_debug_print);

  // Detecting AHT on alternate BANK
  aht_bank = SCL_BANK_1;
  Wire.begin(SDA, aht_bank);

  bool is_ok = false;
  if (aht20_init(aht20))
  {
    aht_bank = SCL_BANK_0;
    Wire.begin(SDA, aht_bank);
    Serial.println("No AHTx0 at BANK1");
  } else {
    Serial.println("AHTx0 detected at BANK1");
    is_ok = true;
  }

  if (!is_ok && aht20_init(aht20) != 0)
  {
    aht20_deinit(aht20);
    Serial.println("AHTx0 NOT detected on BANK0");
    delete aht20;
    aht20 = nullptr;
  }
  else
  {
    Serial.println("AHTx0 detected");
  }


  Wire.begin(SDA, SCL_BANK_0);

  bmp280 = new bmp280_handle_t;
  DRIVER_BMP280_LINK_INIT(bmp280, bmp280_handle_t);

  DRIVER_BMP280_LINK_IIC_INIT(bmp280, aht20_interface_iic_init);
  DRIVER_BMP280_LINK_IIC_DEINIT(bmp280, aht20_interface_iic_deinit);
  DRIVER_BMP280_LINK_IIC_READ(bmp280, bmp280_interface_iic_read);
  DRIVER_BMP280_LINK_IIC_WRITE(bmp280, bmp280_interface_iic_write);

  DRIVER_BMP280_LINK_SPI_INIT(bmp280, aht20_interface_iic_init);
  DRIVER_BMP280_LINK_SPI_DEINIT(bmp280, aht20_interface_iic_deinit);
  DRIVER_BMP280_LINK_SPI_READ(bmp280, noop);
  DRIVER_BMP280_LINK_SPI_WRITE(bmp280, noop);

  DRIVER_BMP280_LINK_DELAY_MS(bmp280, aht20_interface_delay_ms);
  DRIVER_BMP280_LINK_DEBUG_PRINT(bmp280, aht20_interface_debug_print);
  bmp280_set_interface(bmp280, BMP280_INTERFACE_IIC);
  // confising name, its not PIN as PIN but just i2c address...
  // anyway its wrong, because someone tought shifting it to right by
  // one byte is flawless idea
  bmp280_set_addr_pin(bmp280, BMP280_ADDRESS_ADO_HIGH);
  auto fail_code = bmp280_init(bmp280);
  if (fail_code != 0)
  {
    Serial.print("BMP280 NOT detected: ");
    Serial.println(fail_code);
    bmp280_deinit(bmp280);
    delete bmp280;
  }
  else
  {
    Serial.println("Detected BMP280 at default ADDR");
    bmp280_set_mode(bmp280, BMP280_MODE_FORCED);
    bmp280_set_pressure_oversampling(bmp280, BMP280_OVERSAMPLING_x1);
    bmp280_set_temperatue_oversampling(bmp280, BMP280_OVERSAMPLING_x1);
    bmp280_set_filter(bmp280, BMP280_FILTER_OFF);
  }
}

static void fetch_values(float *temp, float *hum, float *pressure)
{
  *temp = NAN;
  *hum = NAN;
  uint32_t temp_raw, humidity_raw;
  Wire.begin(SDA, SCL_BANK_0);
  if (bmp280 != nullptr && bmp280_read_temperature_pressure(bmp280, &temp_raw, temp, &humidity_raw, pressure) == 0)
  {
  }
  else
  {
    *pressure = NAN;
  }


  Wire.begin(SDA, aht_bank);
  uint8_t throwaway;
  if (aht20 != nullptr && aht20_read_temperature_humidity(aht20, &temp_raw, temp, &humidity_raw, &throwaway) == 0)
  {
    // for unknown reason humidity is calculated with hiher precission, but then it's cramed into uint8_t
    // this is code from aht20_read_temperature_humidity
    *hum = (static_cast<float>(humidity_raw) / 1048576.0f * 100.0f);               /* convert the humidity */
  }

  Serial.print("Data: ");
  Serial.print(*temp);
  Serial.print(" ");
  Serial.print(*hum);
  Serial.print(" ");
  Serial.println(*pressure);
}

time_t last_forced;
void loop()
{
  if (Serial.available() > 0) {
    prot_handler.has_data();
  } else {
    prot_handler.tick();
  }

  if (millis() - last_forced > 10000) {
    last_forced = millis();
    float a, b, c;
    fetch_values(&a, &b, &c);
  }
}