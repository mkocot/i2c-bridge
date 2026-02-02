#ifndef I2C_BRIDGE_AHTXX_H
#define I2C_BRIDGE_AHTXX_H

#include "common_driver.h"
#include "sensor.h"

#include <driver_aht30.h>

#define AHTXX_ADDRESS (0x38)

static aht30_handle_t aht30;


static uint8_t sensor_aht30_init(any_sensor_t *ctx)
{
  ((void)ctx);

  i2c.addr = AHTXX_ADDRESS;

  return 0;

  // switch(aht30_deinit(&aht30))
  // {
  //   case 0:
  //   case 3:
  //     break;
  //   default:
  //     return 1;
  // }


  // DO_OR(aht30_init(&aht30));

}


static uint8_t sensor_aht30_probe(any_sensor_t *ctx)
{
  ((void)ctx);

  uint8_t init_state = aht30.inited;

  aht30_deinit(&aht30);

  uint8_t err = aht30_init(&aht30);

  aht30.inited = init_state;

  return err;
}


static obtain_t sensor_aht30_obtain(any_sensor_t *ctx, int32_t *temperature, uint16_t *pressure, uint16_t *humidity)
{
  ((void)ctx);

  if (aht30_read_temperature_humidity(&aht30, &tmp_raw_temperature, &tmp_temperature, &tmp_raw_humidity, &tmp_humidity8))
  {
    return OBTAIN_ERROR;
  }

  return OBTAIN_TEMPERATURE | OBTAIN_HUMIDITY;
}

// INIT -> PROBE|OBTAIN -> DEINIT

static any_sensor_t sensor_aht30 = SENSOR_INIT_ONLY(
  sensor_aht30_init,
  sensor_aht30_probe,
  sensor_aht30_obtain
);


any_sensor_t *sensor_aht30_new(arena_t *arena)
{
  if (sensor_aht30.sensor == NULL)
  {
    DRIVER_SET_DEFAULT_IIC(AHT30, &aht30, aht30_handle_t);
    sensor_aht30.sensor = &sensor_aht30;
  }

  return &sensor_aht30;
}


#endif