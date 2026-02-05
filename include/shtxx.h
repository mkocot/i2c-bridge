#ifndef I2C_BRIDGE_SHTXX_H
#define I2C_BRIDGE_SHTXX_H

#include "common_driver.h"

#include <driver_sht35.h>
#include <driver_shtc3.h>
#include <driver_sht4x.h>


static sht35_handle_t sht35;
static sht4x_handle_t sht4x;
static shtc3_handle_t shtc3;


#define SHTXX_ADDRESS (0x44)
#define SHTC3_ADDRESS (0x70)

static inline uint8_t sensor_shtc3_init()
{
  i2c.addr = SHTC3_ADDRESS;

  return 0;
}

static inline uint8_t sensor_shtxx_init()
{
  /* library address is shifted leaving direction as 0 we need raw address */
  i2c.addr = SHTXX_ADDRESS;

  return 0;
}

static inline uint8_t sensor_sht3x_probe(any_sensor_t *ctx)
{
  uint8_t old_init = sht35.inited;
  sht35.inited = 0;

  if (sht35_init(&sht35))
  {
    goto err;
  }

  if (sht35_set_heater(&sht35, SHT35_BOOL_FALSE))
  {
    printf("SHT35: heaters gonna heat\n");
    goto err;
  }

  if (sht35_set_repeatability(&sht35, SHT35_REPEATABILITY_HIGH))
  {
    printf("SHT35: repatability failed\n");
    goto err;
  }

  uint16_t status;
  if (sht35_get_status(&sht35, &status))
  {
    printf("SHT35: reading status failed\n");
    goto err;
  }
  else
  {
    printf("SHT35: status %X\n", status);
  }

  int32_t t;
  uint16_t p, h;
  if (ctx->obtain(ctx, &t, &p, &h) == OBTAIN_ERROR)
  {
    goto err;
  }

  return 0;

  err:
  sht35_deinit(&sht35);
  sht35.inited = old_init;
  
  return 1;
}

static inline obtain_t sensor_sht3x_obtain(any_sensor_t *ctx, int32_t *temperature, uint16_t *pressure, uint16_t *humidity)
{
  if (sht35_single_read(&sht35, SHT35_BOOL_TRUE, &tmp_raw_temperature16, &tmp_temperature, &tmp_raw_humidity16, &tmp_humidity_f))
  {
    return OBTAIN_ERROR;
  }

  return OBTAIN_HUMIDITY | OBTAIN_TEMPERATURE;
}


static inline uint8_t sensor_sht4x_probe(any_sensor_t *ctx)
{
  uint8_t old_init = sht4x.inited;
  sht35.inited = 0;

  if (sht4x_init(&sht4x))
  {
    goto err;
  }

  int32_t t;
  uint16_t p, h;
  if (ctx->obtain(ctx, &t, &p, &h) == OBTAIN_ERROR)
  {
    goto err;
  }

  return 0;

  err:
  sht4x.inited = old_init;
  sht4x_deinit(&sht4x);

  return 1;
}


static inline obtain_t sensor_sht4x_obtain(any_sensor_t *ctx, int32_t *temperature, uint16_t *pressure, uint16_t *humidity)
{
  if (sht4x_read(&sht4x, SHT4X_MODE_HIGH_PRECISION_WITH_NO_HEATER, &tmp_raw_temperature16, &tmp_temperature, &tmp_raw_humidity16, &tmp_humidity_f))
  {
    return OBTAIN_ERROR;
  }

  return OBTAIN_HUMIDITY | OBTAIN_TEMPERATURE;
}

static inline obtain_t sensor_shtc3_obtain(any_sensor_t *ctx, int32_t *temperature, uint16_t *pressure, uint16_t *humidity)
{
  if (shtc3_read(&shtc3, SHTC3_BOOL_TRUE, &tmp_raw_temperature16, &tmp_temperature, &tmp_raw_humidity16, &tmp_humidity_f))
  {
    return OBTAIN_ERROR;
  }

  return OBTAIN_HUMIDITY | OBTAIN_TEMPERATURE;
}

generate_generic_probe(shtc3);

static any_sensor_t sensor_sht3x = SENSOR_INIT(
  sensor_sht3x_probe,
  sensor_sht3x_obtain
);

static any_sensor_t sensor_sht4x = SENSOR_INIT(
  sensor_sht4x_probe,
  sensor_sht4x_obtain
);

static any_sensor_t sensor_shtc3 = SENSOR_INIT(
  sensor_shtc3_probe,
  sensor_shtc3_obtain
);

static any_sensor_t* sensor_sht3x_new(arena_t *arena)
{
  if (sensor_sht3x.sensor == NULL)
  {
    DRIVER_SET_DEFAULT_IIC_ADDR16(SHT35, &sht35, sht35_handle_t);
    // DRIVER_SHT35_LINK_INIT(&sht35, sht35_handle_t);
    // DRIVER_SHT35_LINK_DEBUG_PRINT(&sht35, debug_print);
    // DRIVER_SHT35_LINK_DELAY_MS(&sht35, libdriver_delay_ms);
    // DRIVER_SHT35_LINK_IIC_INIT(&sht35, libdriver_nop_void);
    // DRIVER_SHT35_LINK_IIC_DEINIT(&sht35, libdriver_nop_void);
    // DRIVER_SHT35_LINK_IIC_READ_ADDRESS16(&sht35, libdriver_iic_addr16_read);
    // DRIVER_SHT35_LINK_IIC_WRITE_ADDRESS16(&sht35, libdriver_iic_addr16_write);
    sht35_set_addr_pin(&sht35, SHT35_ADDRESS_0);

    sensor_sht3x.sensor = &sht35;
  }

  return &sensor_sht3x;
}

static any_sensor_t* sensor_sht4x_new(arena_t *arena)
{

  if (sensor_sht4x.sensor == NULL)
  {
    // WHY THERE IS SUCH DISCREPARENCY BETWEEN MODULES
    // READ_COMMAND vs READ_CMD
    // WRITE_COMMAND vs WRITE_CMD
    // somethimes there is addres sometimes it's not...
    DRIVER_SHT4X_LINK_INIT(&sht4x, sht4x_handle_t);
    DRIVER_SHT4X_LINK_DEBUG_PRINT(&sht4x, debug_print);
    DRIVER_SHT4X_LINK_DELAY_MS(&sht4x, libdriver_delay_ms);
    DRIVER_SHT4X_LINK_IIC_INIT(&sht4x, libdriver_nop_void);
    DRIVER_SHT4X_LINK_IIC_DEINIT(&sht4x, libdriver_nop_void);
    DRIVER_SHT4X_LINK_IIC_READ_COMMAND(&sht4x, libdriver_iic_addr_read_noreg);
    DRIVER_SHT4X_LINK_IIC_WRITE_COMMAND(&sht4x, libdriver_iic_addr_write_noreg);
    sht4x_set_addr(&sht4x, SHT4X_ADDRESS_0);

    sensor_sht4x.sensor = &sht4x;
  }

  return &sensor_sht4x;
}

static any_sensor_t* sensor_shtc3_new(arena_t *arena)
{
  if (sensor_shtc3.sensor == NULL)
  {
    DRIVER_SET_DEFAULT_IIC_ADDR16(SHTC3, &shtc3, shtc3_handle_t);

    sensor_shtc3.sensor = &shtc3;
  }

  return &sensor_shtc3;
}

SENSOR_FACTORY(SHT3X, sensor_sht3x_new, NULL);
SENSOR_FACTORY(SHT4X, sensor_sht4x_new, NULL);
SENSOR_FACTORY(SHTC3, sensor_shtc3_new, NULL);

#endif