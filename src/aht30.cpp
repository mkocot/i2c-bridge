#include "aht30.hpp"
#include "common_driver.hpp"


Aht3x::Aht3x():
  aht30(new aht30_handle_t)
{
  if (aht30 != nullptr)
  {
    return;
  }

  DRIVER_AHT30_LINK_INIT(aht30, aht30_handle_t);
  DRIVER_AHT30_LINK_IIC_INIT(aht30, dummy_uint8_t_no_op);
  DRIVER_AHT30_LINK_IIC_DEINIT(aht30, dummy_uint8_t_no_op);
  DRIVER_AHT30_LINK_IIC_READ_CMD(aht30, generic_i2c_read_cmd);
  DRIVER_AHT30_LINK_IIC_WRITE_CMD(aht30, generic_i2c_write_cmd);
  DRIVER_AHT30_LINK_DELAY_MS(aht30, delay_ms);
  DRIVER_AHT30_LINK_DEBUG_PRINT(aht30, dummy_debug_print);
}

Aht3x::~Aht3x()
{
  if (aht30 == nullptr)
  {
    return;
  }

  end();

  delete aht30;

  aht30 = nullptr;
}

uint8_t Aht3x::end()
{
  return aht30_deinit(aht30);
}

uint8_t Aht3x::begin()
{
    aht30_deinit(aht30);

    return aht30_init(aht30);
}

uint8_t Aht3x::t_and_h(float *temp, float *hum)
{
    if (aht30_read_temperature_humidity(aht30, &temp_raw, temp, &humidity_raw, &humidity_percent) != 0)
    {
        return 1;
    }

    /* convert the humidity */
    /* code from aht30_read_temperature_humidity */

    *hum = (static_cast<float>(humidity_raw) / 1048576.0f * 100.0f); 

    return 0;
}
