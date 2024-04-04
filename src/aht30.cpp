#include "aht30.hpp"
#include "common_driver.hpp"

static aht30_handle_t *aht30 = nullptr;

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

void init_aht30()
{
  if (software_ic_current == nullptr)
  {
    return;
  }

  if (aht30 != nullptr)
  {
    return;
  }

  aht30 = new aht30_handle_t;// Adafruit_AHTX0();
  DRIVER_AHT30_LINK_INIT(aht30, aht30_handle_t);
  DRIVER_AHT30_LINK_IIC_INIT(aht30, dummy_uint8_t_no_op);
  DRIVER_AHT30_LINK_IIC_DEINIT(aht30, dummy_uint8_t_no_op);
  DRIVER_AHT30_LINK_IIC_READ_CMD(aht30, ahtXX_interface_iic_read_cmd);
  DRIVER_AHT30_LINK_IIC_WRITE_CMD(aht30, ahtXX_interface_iic_write_cmd);
  DRIVER_AHT30_LINK_DELAY_MS(aht30, delay_ms);
  DRIVER_AHT30_LINK_DEBUG_PRINT(aht30, dummy_debug_print);
}

void deinit_aht30()
{
  if (aht30 == nullptr)
  {
    return;
  }

  aht30_deinit(aht30);

  delete aht30;

  aht30 = nullptr;

  software_ic_current = nullptr;
}

uint8_t aht30_probe(SoftWire *sw)
{
    software_ic_current = sw;

    init_aht30();

    aht30_deinit(aht30);

    return aht30_init(aht30);
}

uint8_t aht30_fetch(float *temp, float *hum)
{
    static uint32_t temp_raw;
    static uint32_t humidity_raw;
    static uint8_t humidity_percent;

    if (aht30 == nullptr)
    {
        return 1;
    }
    
    if (aht30_read_temperature_humidity(aht30, &temp_raw, temp, &humidity_raw, &humidity_percent) != 0)
    {
        return 1;
    }

    /* convert the humidity */
    /* code from aht30_read_temperature_humidity */

    *hum = (static_cast<float>(humidity_raw) / 1048576.0f * 100.0f); 

    return 0;
}
