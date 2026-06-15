#ifndef I2C_BRIDGE_SHTXX_H
#define I2C_BRIDGE_SHTXX_H

#include "common_driver.h"
#include "sensor.h"

#include <driver_sht35.h>
#include <driver_shtc3.h>
#include <driver_sht4x.h>

#include <fptc.h>

static inline fpt shtxx_t_fpt(uint16_t val) {
    #if 1
    fpt as_fpt = val;
    // 0.030727 0.001053
    // as_fpt = fpt_div(as_fpt, i2fpt(65535 / 5));
    // NOTE(m): Why using +176 improves accuracy?
    as_fpt = fpt_mul(as_fpt, i2fpt(175) + 176);
    #else
    // 0.084133 0.002625
    as_fpt = fpt_div(as_fpt, i2fpt(65535));
    as_fpt = fpt_mul(as_fpt, i2fpt(175));
    #endif
    as_fpt = fpt_sub(as_fpt, i2fpt(45));

    return as_fpt;
}

static inline float sht4x_h_fpt(uint16_t val) {
    fpt as_fpt = val;
    as_fpt = fpt_mul(as_fpt, i2fpt(125));
    as_fpt = fpt_sub(as_fpt, i2fpt(6));

    return as_fpt;
}

static sht35_handle_t sht35;
static sht4x_handle_t sht4x;
static shtc3_handle_t shtc3;


#define SHTXX_ADDRESS (0x44)
#define SHTC3_ADDRESS (0x70)

static uint8_t sensor_sht3x_iic_addr16_read(uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t len)
{
  i2c.addr = addr >> 1;
  i2c.regb = 2;
  printf("SHT3X: read: %X\n", i2c.addr);

  uint8_t ret;

  /*
  * fetching data in 'single shot' is taking absurd 12ms. Setting this timeout
  * in i2c driver will be hard penalty during communication with other devices
  * to fix it we are using delayed read only for 2 commands. Anyway MCU could
  * do something usefull in that time...
  */
  switch (reg >> 8)
  {
    case 0x24: /* single read without clock stretching */
      /* fallthrough */
    case 0x2C: /* single read with clock stretching */
      ret = i2c_read_reg_delay(&i2c, reg, buf, len, 12);
      break;
    case 0xE0: /* sht35_continuous_read */
      /* fallthrough */
    default:
      ret = i2c_read_reg(&i2c, reg, buf, len);
      break;
  }

  // printf("A16R: %X %X %u %u =%d\n", i2c.addr, reg, i2c.regb, len, ret);

  return ret;
}

static uint8_t sensor_shtc3_iic_addr16_read(uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t len)
{
  i2c.addr = addr >> 1;
  i2c.regb = 2;

  uint8_t ret;
  /* SHTC3 driver, see above
    * Low Power mode: ~ 0.7ms  0.8ms max
    * Normal mode   : 10.8ms   12.1ms max
    */
  switch (reg >> 8)
  {
    case 0x78: /* T without clock stretch */
      /* fallthrough */
    case 0x7C: /* T with clock stretch */
      /* fallthrough */
    case 0x58: /* RH without clock stretch */
      /* fallthrough */
    case 0x5C: /* RH with clock stretch */
      ret = i2c_read_reg_delay(&i2c, reg, buf, len, 12); /* NOTE(m): This is probably too high */
      break;
    default:
      ret = i2c_read_reg(&i2c, reg, buf, len);
      break;
  }

  // printf("SHTC3R: addr=0x%X reg=0x%X regb=%u len=%u ret=%d\n", i2c.addr, reg, i2c.regb, len, ret);

  return ret;
}

static inline uint8_t sensor_sht3x_probe(any_sensor_t *ctx)
{
  uint8_t old_init = sht35.inited;
  sht35.inited = 0;

  if (sht35_init(&sht35))
  {
    printf("SHT35: init failed\n");
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

  temperature_t t;
  pressure_t p;
  humidity_t h;

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

static inline obtain_t sensor_sht3x_obtain(any_sensor_t *ctx, temperature_t *temperature, pressure_t *pressure, humidity_t *humidity)
{
  sht35.inited = 1;

  if (sht35_single_read(&sht35, SHT35_BOOL_TRUE, &tmp_raw_temperature16, NULL, &tmp_raw_humidity16, NULL))
  {
    return OBTAIN_ERROR;
  }

  *humidity = raw_hum_to_fpt(tmp_raw_humidity16);
  *temperature = shtxx_t_fpt(tmp_raw_temperature16);

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

  temperature_t t;
  pressure_t p;
  humidity_t h;
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


static inline obtain_t sensor_sht4x_obtain(any_sensor_t *ctx, temperature_t *temperature, pressure_t *pressure, humidity_t *humidity)
{
  sht4x.inited = 1;

  if (sht4x_read(&sht4x, SHT4X_MODE_HIGH_PRECISION_WITH_NO_HEATER, &tmp_raw_temperature16, NULL, &tmp_raw_humidity16, NULL))
  {
    return OBTAIN_ERROR;
  }

  *humidity = sht4x_h_fpt(tmp_raw_humidity16);
  *temperature = shtxx_t_fpt(tmp_raw_temperature16);

  return OBTAIN_HUMIDITY | OBTAIN_TEMPERATURE;
}

static inline obtain_t sensor_shtc3_obtain(any_sensor_t *ctx, temperature_t *temperature, pressure_t *pressure, humidity_t *humidity)
{
  shtc3.inited = 1;

  if (shtc3_read(&shtc3, SHTC3_BOOL_TRUE, &tmp_raw_temperature16, NULL, &tmp_raw_humidity16, NULL))
  {
    return OBTAIN_ERROR;
  }

  *humidity = raw_hum_to_fpt(tmp_raw_humidity16);
  *temperature = shtxx_t_fpt(tmp_raw_temperature16);

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
    /* SHT35 driver requirese special handling of read delays */
    DRIVER_SHT35_LINK_IIC_READ_ADDRESS16(&sht35, sensor_sht3x_iic_addr16_read);
    // DRIVER_SHT35_LINK_INIT(&sht35, sht35_handle_t);
    // DRIVER_SHT35_LINK_DEBUG_PRINT(&sht35, debug_print);
    // DRIVER_SHT35_LINK_DELAY_MS(&sht35, libdriver_delay_ms);
    // DRIVER_SHT35_LINK_IIC_INIT(&sht35, libdriver_nop_void);
    // DRIVER_SHT35_LINK_IIC_DEINIT(&sht35, libdriver_nop_void);
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
    DRIVER_SHTC3_LINK_IIC_READ_ADDRESS16(&shtc3, sensor_shtc3_iic_addr16_read);

    sensor_shtc3.sensor = &shtc3;
  }

  return &sensor_shtc3;
}

SENSOR_FACTORY(SHT3X, SHTXX_ADDRESS, sensor_sht3x_new, NULL);
SENSOR_FACTORY(SHT4X, SHTXX_ADDRESS, sensor_sht4x_new, NULL);
SENSOR_FACTORY(SHTC3, SHTC3_ADDRESS, sensor_shtc3_new, NULL);

#endif