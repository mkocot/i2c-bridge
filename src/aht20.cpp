// #include "aht20.hpp"
// #include "common_driver.hpp"

// Aht2x::Aht2x() : aht20(new aht20_handle_t)
// {
//   DRIVER_AHT20_LINK_INIT(aht20, aht20_handle_t);
//   DRIVER_AHT20_LINK_IIC_INIT(aht20, dummy_uint8_t_no_op);
//   DRIVER_AHT20_LINK_IIC_DEINIT(aht20, dummy_uint8_t_no_op);
//   DRIVER_AHT20_LINK_IIC_READ_CMD(aht20, generic_i2c_read_cmd);
//   DRIVER_AHT20_LINK_IIC_WRITE_CMD(aht20, generic_i2c_write_cmd);
//   DRIVER_AHT20_LINK_DELAY_MS(aht20, delay_ms);
//   DRIVER_AHT20_LINK_DEBUG_PRINT(aht20, dummy_debug_print);
// }

// Aht2x::~Aht2x()
// {
//   if (aht20 == nullptr)
//   {
//     return;
//   }

//   end();

//   delete aht20;

//   aht20 = nullptr;
// }

// uint8_t Aht2x::begin()
// {
//   if (end() == 1)
//   {
//     return 1;
//   }

//   return aht20_init(aht20);
// }

// uint8_t Aht2x::end()
// {
//   return aht20_deinit(aht20);
// }

// uint8_t Aht2x::t_and_h(float *temp, float *hum)
// {
//   if (aht20_read_temperature_humidity(aht20, &temp_raw, temp, &humidity_raw, &humidity_percent))
//   {
//     return 1;
//   }

//   /* convert the humidity */
//   /* code from aht20_read_temperature_humidity */

//   *hum = (static_cast<float>(humidity_raw) / 1048576.0f * 100.0f);

//   return 0;
// }
