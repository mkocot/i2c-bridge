#ifndef I2C_BRIDGE_BMP280_H
#define I2C_BRIDGE_BMP280_H

#include "common_driver.h"
#include "driver_bmp280.h"
#include "sensor.h"

#define BMP280_ADDRESS (BMP280_ADDRESS_ADO_HIGH >> 1)
#define BME280_ADDRESS (BME280_ADDRESS_ADO_HIGH >> 1)


static inline uint8_t bmp280_iic_init()
{
  i2c.addr = BMP280_ADDRESS;

  return 0;
}


static uint8_t sensor_bmp280_probe(any_sensor_t *ctx)
{
  bmp280_handle_t *bmp280 = (bmp280_handle_t*)ctx->sensor;

  if (bmp280 == NULL)
  {
    return OBTAIN_ERROR;
  }

  i2c.addr = BMP280_ADDRESS;

  DO_OR(bmp280_init(bmp280));
  DO_OR(bmp280_set_mode(bmp280, BMP280_MODE_FORCED));
  DO_OR(bmp280_set_filter(bmp280, BMP280_FILTER_OFF));

  // Thes both method ar botched, you cannot set pressure and temperature
  // only first one is selected and 2nd ignored
  // if (bmp280_set_pressure_temperature_oversampling(&bmp280, BMP280_OVERSAMPLING_x1, BMP280_OVERSAMPLING_x1))
  // {
  //   printf("F3\n");
  // }

  if (bmp280_set_pressure_oversampling(bmp280, BMP280_OVERSAMPLING_x1))
  {
    // printf("F3\n");
    return 4;
  }

  Delay_Ms(1);

  if (bmp280_set_temperatue_oversampling(bmp280, BMP280_OVERSAMPLING_x1))
  {
    // printf("F4\n");
    return 5;
  }

  return 0;
}

static obtain_t sensor_bmp280_obtain(any_sensor_t *ctx, int32_t *out_t, uint16_t *out_p, uint16_t *out_h)
{
  bmp280_handle_t *bmp280 = (bmp280_handle_t*)ctx->sensor;
  if (bmp280 == NULL)
  {
    return OBTAIN_ERROR;
  }

  bmp280_temperature_t t;
  bmp280_pressure_t p;
  if (bmp280_read_temperature_pressure(bmp280, &tmp_raw_temperature, &t, &tmp_raw_pressure, &p))
  {
    return OBTAIN_ERROR;
  }
#if 1
  *out_t = QUANTIZE_TEMP(t);
  *out_p = QUANTIZE_PRESSURE(p);
  #endif

  return OBTAIN_TP;
}


static inline any_sensor_t *sensor_factory_bmp280_new(arena_t *arena)
{
  // TODO: allocate!!
  any_sensor_t *sensor_bmp280 = (any_sensor_t*)arena_alloc(arena, sizeof(any_sensor_t));
  if (sensor_bmp280 == NULL)
  {
    return NULL;
  }

  sensor_bmp280->obtain = sensor_bmp280_obtain;
  sensor_bmp280->probe = sensor_bmp280_probe;


  bmp280_handle_t *bmp280 = (bmp280_handle_t*)arena_alloc(arena, sizeof(bmp280_handle_t));
  if (bmp280 == NULL)
  {
    return NULL;
  }

  DRIVER_SET_DEFAULT_IIC_ADDR(BMP280, bmp280, bmp280_handle_t);
  bmp280_set_interface(bmp280, BMP280_INTERFACE_IIC);
  bmp280_set_addr_pin(bmp280, BMP280_ADDRESS_ADO_HIGH);

  sensor_bmp280->sensor = bmp280;

  return sensor_bmp280;
}

SENSOR_FACTORY(BMP280, BMP280_ADDRESS, sensor_factory_bmp280_new, NULL);


#endif