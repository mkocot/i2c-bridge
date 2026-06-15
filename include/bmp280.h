#ifndef I2C_BRIDGE_BMP280_H
#define I2C_BRIDGE_BMP280_H

#include "common_driver.h"
#include "driver_bmp280.h"
#include "sensor.h"

#define BMP280_ADDRESS (BMP280_ADDRESS_ADO_HIGH >> 1)
#define BME280_ADDRESS (BME280_ADDRESS_ADO_HIGH >> 1)

#define TRACE_X(FMT, ARGS...) printf(FMT, ##ARGS)
#define TRACE(FMT, Z...) TRACE_X("[%s:%04d]" ## FMT "\n", __FUNCTION__, __LINE__, ##Z)

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

static obtain_t sensor_bmp280_obtain(any_sensor_t *ctx, temperature_t *out_t, pressure_t *out_p, humidity_t *out_h)
{
  (void)out_h;

  TRACE();
  bmp280_handle_t *bmp280 = (bmp280_handle_t*)ctx->sensor;
  TRACE();
  if (bmp280 == NULL)
  {
  TRACE();
    return OBTAIN_ERROR;
  }

  bmp280_temperature_t t;
  bmp280_pressure_t p;
  TRACE();
  if (bmp280_read_temperature_pressure(bmp280, &tmp_raw_temperature, &t, &tmp_raw_pressure, &p))
  {
  TRACE();
    return OBTAIN_ERROR;
  }
  TRACE();
#if 1
	#ifdef DRIVER_BMP280_WITH_INT
    printf("T=%ld\n", t);
    printf("P=%lu\n", p);
  #else
    *out_t = fl2fpt(t);
    *out_p = fl2fpt_q17(p);

    // *out_t = fl2fpt(t);
    // *out_p = fl2fpt(p);

    // printf("T=" PR_FPT " %s %lx\n", F2PRINTF(t), fpt_cstr(*out_t, -1), *out_t);
    // printf("P=" PR_FPT "\n", F2PRINTF(p));
  #endif

  TRACE();
  #endif

  TRACE();
  return OBTAIN_TP;
}


static inline any_sensor_t *sensor_factory_bmp280_new(arena_t *arena)
{
  // TODO: allocate!!
  TRACE();
  any_sensor_t *sensor_bmp280 = (any_sensor_t*)arena_alloc(arena, sizeof(any_sensor_t));
  TRACE();
  if (sensor_bmp280 == NULL)
  {
  TRACE();
    return NULL;
  }

  TRACE();
  sensor_bmp280->obtain = sensor_bmp280_obtain;
  TRACE();
  sensor_bmp280->probe = sensor_bmp280_probe;
  TRACE();


  bmp280_handle_t *bmp280 = (bmp280_handle_t*)arena_alloc(arena, sizeof(bmp280_handle_t));
  TRACE();
  if (bmp280 == NULL)
  {
  TRACE();
    return NULL;
  }

  TRACE();
  DRIVER_SET_DEFAULT_IIC_ADDR(BMP280, bmp280, bmp280_handle_t);
  TRACE();
  bmp280_set_interface(bmp280, BMP280_INTERFACE_IIC);
  TRACE();
  bmp280_set_addr_pin(bmp280, BMP280_ADDRESS_ADO_HIGH);
  TRACE();

  sensor_bmp280->sensor = bmp280;
  TRACE();

  return sensor_bmp280;
}

SENSOR_FACTORY(BMP280, BMP280_ADDRESS, sensor_factory_bmp280_new, NULL);


#endif