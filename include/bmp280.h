#ifndef I2C_BRIDGE_BMP280_H
#define I2C_BRIDGE_BMP280_H

#include "common_driver.h"
#include "driver_bmp280.h"

#define BMP280_ADDRESS (BMP280_ADDRESS_ADO_HIGH >> 1)
#define BME280_ADDRESS (BME280_ADDRESS_ADO_HIGH >> 1)

static bmp280_handle_t bmp280;

static inline uint8_t bmp280_iic_init()
{
  i2c.addr = BMP280_ADDRESS;

  return 0;
}

static inline void init_bmp280(void)
{
  DRIVER_SET_DEFAULT_IIC_ADDR(BMP280, &bmp280, bmp280_handle_t);
  bmp280_set_interface(&bmp280, BMP280_INTERFACE_IIC);
  bmp280_set_addr_pin(&bmp280, BMP280_ADDRESS_ADO_HIGH);
}

static uint8_t setupBMP280()
{
  bmp280_iic_init();
  if (bmp280_init(&bmp280))
  {
    printf("bmp280 failed\n");
    return 1;
  }

  if (bmp280_set_mode(&bmp280, BMP280_MODE_FORCED))
  {
    printf("F1\n");
    return 2;
  }

  if (bmp280_set_filter(&bmp280, BMP280_FILTER_OFF))
  {
    printf("F2\n");
    return 3;
  }

  // Thes both method ar botched, you cannot set pressure and temperature
  // only first one is selected and 2nd ignored
  // if (bmp280_set_pressure_temperature_oversampling(&bmp280, BMP280_OVERSAMPLING_x1, BMP280_OVERSAMPLING_x1))
  // {
  //   printf("F3\n");
  // }

  if (bmp280_set_pressure_oversampling(&bmp280, BMP280_OVERSAMPLING_x1))
  {
    printf("F3\n");
    return 4;
  }

  Delay_Ms(1);

  if (bmp280_set_temperatue_oversampling(&bmp280, BMP280_OVERSAMPLING_x1))
  {
    printf("F4\n");
    return 5;
  }

  return 0;
}


#endif