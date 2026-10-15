#ifndef W_ADT7410_H
#define W_ADT7410_H


#include "common_driver.h"
#include "sensor.h"

#include <driver_adt7410.h>

#include <fptc.h>

#define DRIVER_ADT7410_ADDRESS (ADT7410_ADDRESS_A0A1_00 >> 1)

static adt7410_handle_t adt7410;

/* T: raw/128 degC (16-bit resolution: 0.0078 degC/LSB) */
static inline fpt adt7410_t_fpt(int16_t val) {
    fpt as_fpt = fpt_mul(i2fpt(val), fl2fpt(1.0f/128.0f));

    return as_fpt;
}

static obtain_t sensor_adt7410_obtain(any_sensor_t *ctx, temperature_t *t, pressure_t *p, humidity_t *h)
{
    ((void)ctx);
    ((void)p);
    ((void)h);

    adt7410.inited = 1;

    /* one-shot conversion: driver triggers, waits 288 ms (1.2x typ 240 ms),
       reads the 16-bit temperature register */
    if (adt7410_read_temperature(&adt7410, &tmp_raw_temperaturei16, NULL))
    {
        return OBTAIN_ERROR;
    }

    *t = adt7410_t_fpt(tmp_raw_temperaturei16);

    return OBTAIN_TEMPERATURE;
}

static uint8_t sensor_adt7410_probe(any_sensor_t *ctx)
{
    ((void)ctx);

    adt7410.inited = 0;

    /* reads ID register (0x0B), verifies manufacturer ID bits (0xC8..0xCF),
       writes config for 16-bit resolution + one-shot mode; the first
       power-up conversion completes within the 10 ms wait */
    uint8_t err = adt7410_init(&adt7410);
    if (err)
    {
        return err;
    }

    /* one-shot trigger at probe end wakes the device from shutdown and
       starts the first conversion; 10 ms wait covers the 6 ms typ
       power-up conversion (docs/sensor-timing.md) */
    uint8_t cfg = ADT7410_CFG_ONE_SHOT | ADT7410_CFG_RESOLUTION;
    return adt7410_set_reg(&adt7410, ADT7410_REG_CONFIG, &cfg, 1);
}

static any_sensor_t sensor_adt7410 = SENSOR_INIT(
    sensor_adt7410_probe,
    sensor_adt7410_obtain
);

static any_sensor_t* sensor_adt7410_new(arena_t *arena)
{
    if (sensor_adt7410.sensor == NULL)
    {
        DRIVER_SET_DEFAULT_IIC_ADDR(ADT7410, &adt7410, adt7410_handle_t);
        DRIVER_ADT7410_LINK_DEBUG_PRINT(&adt7410, debug_print);
        adt7410_set_addr_pin(&adt7410, ADT7410_ADDRESS_A0A1_00);

        sensor_adt7410.sensor = &adt7410;
    }

    return &sensor_adt7410;
}

SENSOR_FACTORY(ADT7410, DRIVER_ADT7410_ADDRESS, sensor_adt7410_new, NULL);

#endif
