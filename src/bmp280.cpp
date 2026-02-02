// #include "bmp280.hpp"
// #include "common_driver.hpp"

// #include <driver_bmp280.h>

// static uint8_t noop_spi(uint8_t x, uint8_t *xx, uint16_t xxx)
// {
//   return 0;
// }

// Bmp280::~Bmp280()
// {
//   if (bmp280 == nullptr)
//   {
//     return;
//   }

//   end();

//   delete bmp280;

//   bmp280 = nullptr;
// }

// uint8_t Bmp280::end()
// {
//   return bmp280_deinit(bmp280);
// }

// uint8_t Bmp280::begin()
// {
//   bmp280_deinit(bmp280);

//   if (bmp280_init(bmp280) != 0)
//   {
//     return 1;
//   }

//   bmp280_set_mode(bmp280, BMP280_MODE_FORCED);
//   // Ultra High Precision configuration (from manual)
//   bmp280_set_pressure_oversampling(bmp280, BMP280_OVERSAMPLING_x16);
//   bmp280_set_temperatue_oversampling(bmp280, BMP280_OVERSAMPLING_x16);
//   bmp280_set_filter(bmp280, BMP280_FILTER_COEFF_16);

//   return 0;
// }

// uint8_t Bmp280::t_and_h(float *temp, float *pressure)
// {
//   // return bmp280_read_temperature_pressure(bmp280, &buffer, temp, &buffer, pressure);
// }

// Bmp280::Bmp280():
//   bmp280(new bmp280_handle_t)
// {
//   DRIVER_BMP280_LINK_INIT(bmp280, bmp280_handle_t);

//   DRIVER_BMP280_LINK_IIC_INIT(bmp280, dummy_uint8_t_no_op);
//   DRIVER_BMP280_LINK_IIC_DEINIT(bmp280, dummy_uint8_t_no_op);
//   DRIVER_BMP280_LINK_IIC_READ(bmp280, generic_i2c_read_reg_cmd);
//   DRIVER_BMP280_LINK_IIC_WRITE(bmp280, generic_i2c_write_reg_cmd);

//   DRIVER_BMP280_LINK_SPI_INIT(bmp280, dummy_uint8_t_no_op);
//   DRIVER_BMP280_LINK_SPI_DEINIT(bmp280, dummy_uint8_t_no_op);
//   DRIVER_BMP280_LINK_SPI_READ(bmp280, noop_spi);
//   DRIVER_BMP280_LINK_SPI_WRITE(bmp280, noop_spi);

//   DRIVER_BMP280_LINK_DELAY_MS(bmp280, delay_ms);
//   DRIVER_BMP280_LINK_DEBUG_PRINT(bmp280, dummy_debug_print);
//   bmp280_set_interface(bmp280, BMP280_INTERFACE_IIC);
//   // confising name, its not PIN as PIN but just i2c address...
//   // anyway its wrong, because someone tought shifting it to right by
//   // one byte is flawless idea
//   bmp280_set_addr_pin(bmp280, BMP280_ADDRESS_ADO_HIGH);
// }
