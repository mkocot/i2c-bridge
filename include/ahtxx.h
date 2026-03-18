#ifndef I2C_BRIDGE_AHTXX_H
#define I2C_BRIDGE_AHTXX_H

#include "common_driver.h"
#include "sensor.h"

#include <driver_aht30.h>

static inline fpt ahtxx_t_fpt(uint32_t val)
{
    fpt as_fpt = val;
    as_fpt = fpt_mul(as_fpt, fl2fpt(12.5f));
    as_fpt = fpt_sub(as_fpt, i2fpt(50));

    return as_fpt;
}

static inline fpt athxx_h_fpt(uint32_t val)
{
    fpt as_fpt = val;
    as_fpt = fpt_mul(as_fpt, fl2fpt(6.25f));
    return as_fpt;
}

#define AHTXX_ADDRESS (0x38)

static aht30_handle_t aht30;


static uint8_t sensor_aht30_probe(any_sensor_t *ctx)
{
  ((void)ctx);

  uint8_t status;

  i2c.addr = AHTXX_ADDRESS;

  aht30_deinit(&aht30);

  DO_OR(aht30_init(&aht30));
  DO_OR(aht30_get_status(&aht30, &status));

  return 0;
}


static obtain_t sensor_aht30_obtain(any_sensor_t *ctx, int32_t *temperature, uint16_t *pressure, uint16_t *humidity)
{
  ((void)ctx);

  i2c.addr = AHTXX_ADDRESS;

  aht30.inited = 1;

  if (aht30_read_temperature_humidity(&aht30, &tmp_raw_temperature, NULL, &tmp_raw_humidity, NULL))
  {
    return OBTAIN_ERROR;
  }

  *temperature = QUANTIZE_TEMP(ahtxx_t_fpt(tmp_raw_temperature));
  *humidity = QUANTIZE_HUM(athxx_h_fpt(tmp_raw_humidity));

  return OBTAIN_TEMPERATURE | OBTAIN_HUMIDITY;
}


// INIT -> PROBE|OBTAIN -> DEINIT

static any_sensor_t sensor_aht30 = SENSOR_INIT(
  sensor_aht30_probe,
  sensor_aht30_obtain
);


any_sensor_t *sensor_ahtxx_new(arena_t *arena)
{
  if (sensor_aht30.sensor == NULL)
  {
    DRIVER_SET_DEFAULT_IIC(AHT30, &aht30, aht30_handle_t);
    sensor_aht30.sensor = &aht30;
  }

  return &sensor_aht30;
}

SENSOR_FACTORY(AHTXX, AHTXX_ADDRESS, sensor_ahtxx_new, NULL);

#endif