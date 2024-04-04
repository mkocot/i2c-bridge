#include "aht20.hpp"
#include "common_driver.hpp"

static aht20_handle_t *aht20 = nullptr;

static SoftWire *software_ic_current = nullptr;

static uint8_t ahtXX_interface_iic_read_cmd(uint8_t addr, uint8_t *buf, uint16_t len)
{
  auto r = software_ic_current->requestFrom((int)(addr >> 1), (int)len);
  auto d = software_ic_current->readBytes(buf, len);

  return d != len;
}

static uint8_t ahtXX_interface_iic_write_cmd(uint8_t addr, uint8_t *buf, uint16_t len)
{
  software_ic_current->beginTransmission((addr >> 1));
  auto w = software_ic_current->write(buf, len);
  software_ic_current->endTransmission();

  return w != len;
}

void init_aht20()
{
  if (software_ic_current == nullptr)
  {
    return;
  }

  if (aht20 != nullptr)
  {
    return;
  }

  aht20 = new aht20_handle_t;// Adafruit_AHTX0();
  DRIVER_AHT20_LINK_INIT(aht20, aht20_handle_t);
  DRIVER_AHT20_LINK_IIC_INIT(aht20, dummy_uint8_t_no_op);
  DRIVER_AHT20_LINK_IIC_DEINIT(aht20, dummy_uint8_t_no_op);
  DRIVER_AHT20_LINK_IIC_READ_CMD(aht20, ahtXX_interface_iic_read_cmd);
  DRIVER_AHT20_LINK_IIC_WRITE_CMD(aht20, ahtXX_interface_iic_write_cmd);
  DRIVER_AHT20_LINK_DELAY_MS(aht20, delay_ms);
  DRIVER_AHT20_LINK_DEBUG_PRINT(aht20, dummy_debug_print);
}

void deinit_aht20()
{
  if (aht20 == nullptr)
  {
    return;
  }

  aht20_deinit(aht20);

  delete aht20;

  aht20 = nullptr;

  software_ic_current = nullptr;
}

uint8_t aht20_probe(SoftWire *sw)
{
    software_ic_current = sw;

    init_aht20();

    aht20_deinit(aht20);

    return aht20_init(aht20);
}

uint8_t aht20_fetch(float *temp, float *hum)
{
    static uint32_t temp_raw;
    static uint32_t humidity_raw;
    static uint8_t humidity_percent;

    if (aht20 == nullptr)
    {
        return 1;
    }
    
    if (aht20_read_temperature_humidity(aht20, &temp_raw, temp, &humidity_raw, &humidity_percent) != 0)
    {
        return 1;
    }

    /* convert the humidity */
    /* code from aht20_read_temperature_humidity */

    *hum = (static_cast<float>(humidity_raw) / 1048576.0f * 100.0f); 

    return 0;
}
