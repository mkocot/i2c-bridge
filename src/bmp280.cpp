#include "bmp280.hpp"
#include "common_driver.hpp"

#include <driver_bmp280.h>

static bmp280_handle_t *bmp280 = nullptr;
static SoftWire *software_ic_current = nullptr;
static uint8_t bmp280_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{

  software_ic_current->beginTransmission(addr >> 1);

  auto w = software_ic_current->write(reg);
  if (len != 0) {
    w += software_ic_current->write(buf, len);
  }
  software_ic_current->endTransmission();

  return w != (len + 1u);
}


static uint8_t bmp280_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{

  if (bmp280_interface_iic_write(addr, reg, nullptr, 0))
  {
    return -1;
  }

  // // return i2c_read(addr >> 1, buf, len);

  auto r = software_ic_current->requestFrom((int)(addr >> 1), (int)len);
  auto d = software_ic_current->readBytes(buf, len);
  return d != len;
}

static uint8_t noop_spi(uint8_t x, uint8_t *xx, uint16_t xxx)
{
  return 0;
}

void deinit_bmp280()
{
    if (bmp280 == nullptr)
    {
        return;
    }

    bmp280_deinit(bmp280);

    delete bmp280;

    bmp280 = nullptr;

    software_ic_current = nullptr;
}

uint8_t bmp280_probe(SoftWire *sw)
{
  software_ic_current = sw;

  init_bmp280();

  bmp280_deinit(bmp280);

  if (bmp280_init(bmp280) != 0)
  {
    return 1;
  }

  bmp280_set_mode(bmp280, BMP280_MODE_FORCED);
  bmp280_set_pressure_oversampling(bmp280, BMP280_OVERSAMPLING_x1);
  bmp280_set_temperatue_oversampling(bmp280, BMP280_OVERSAMPLING_x1);
  bmp280_set_filter(bmp280, BMP280_FILTER_OFF);

  return 0;
}

uint8_t bmp280_fetch(float *temp, float *pressure)
{
    static uint32_t buffer;

    if (bmp280 == nullptr)
    {
        return 1;
    }

    return bmp280_read_temperature_pressure(bmp280, &buffer, temp, &buffer, pressure);
}

void init_bmp280()
{
  if (bmp280 != nullptr)
  {
    return;
  }

  bmp280 = new bmp280_handle_t;
  DRIVER_BMP280_LINK_INIT(bmp280, bmp280_handle_t);

  DRIVER_BMP280_LINK_IIC_INIT(bmp280, dummy_uint8_t_no_op);
  DRIVER_BMP280_LINK_IIC_DEINIT(bmp280, dummy_uint8_t_no_op);
  DRIVER_BMP280_LINK_IIC_READ(bmp280, bmp280_interface_iic_read);
  DRIVER_BMP280_LINK_IIC_WRITE(bmp280, bmp280_interface_iic_write);

  DRIVER_BMP280_LINK_SPI_INIT(bmp280, dummy_uint8_t_no_op);
  DRIVER_BMP280_LINK_SPI_DEINIT(bmp280, dummy_uint8_t_no_op);
  DRIVER_BMP280_LINK_SPI_READ(bmp280, noop_spi);
  DRIVER_BMP280_LINK_SPI_WRITE(bmp280, noop_spi);

  DRIVER_BMP280_LINK_DELAY_MS(bmp280, delay_ms);
  DRIVER_BMP280_LINK_DEBUG_PRINT(bmp280, dummy_debug_print);
  bmp280_set_interface(bmp280, BMP280_INTERFACE_IIC);
  // confising name, its not PIN as PIN but just i2c address...
  // anyway its wrong, because someone tought shifting it to right by
  // one byte is flawless idea
  bmp280_set_addr_pin(bmp280, BMP280_ADDRESS_ADO_HIGH);
}
