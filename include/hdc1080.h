#ifndef W_HDC1080_H
#define W_HDC1080_H

#include "common_driver.h"
#include "sensor.h"

#include <driver_hdc1080.h>

#define DRIVER_HDC1080_ADDRESS 0x40
#define TEMP_RESOLUTION HDC1080_TEMPERATURE_RESOLUTION_14_BIT
#define HUMI_RESOLUTION HDC1080_HUMIDITY_RESOLUTION_14_BIT

static inline fpt hdc1080_t_fpt(uint16_t val) {
    #if 1
    // 0.010492 0.000332
    fpt as_fpt = val;
    // as_fpt = fpt_div(as_fpt, i2fpt(65535 / 15));
    // +166 is magic adjustement
    as_fpt = fpt_mul(as_fpt, i2fpt(165) + 166);
    #else
    fpt as_fpt = i2fpt(val);
    // 0.079309 0.002472
    as_fpt = fpt_div(as_fpt, i2fpt(65535));
    as_fpt = fpt_mul(as_fpt, i2fpt(165));
    #endif

    as_fpt = fpt_sub(as_fpt, i2fpt(40));

    return as_fpt;
}

static hdc1080_handle_t hdc1080;

static uint8_t sensor_hdc1080_probe(any_sensor_t *ctx)
{
    hdc1080.inited = 0;

    i2c.addr = DRIVER_HDC1080_ADDRESS;

    DO_OR(hdc1080_init(&hdc1080));
    DO_OR(hdc1080_set_heater(&hdc1080, HDC1080_BOOL_FALSE));
    DO_OR(hdc1080_set_mode(&hdc1080, HDC1080_MODE_SEQUENCE));
    DO_OR(hdc1080_set_humidity_resolution(&hdc1080, HUMI_RESOLUTION));
    DO_OR(hdc1080_set_temperature_resolution(&hdc1080, TEMP_RESOLUTION));

    return 0;
}

static obtain_t sensor_hdc1080_obtain(any_sensor_t *ctx, temperature_t *t, pressure_t *p, humidity_t *h)
{
    int err = 0 ;
    hdc1080.inited = 1;

    i2c.addr = DRIVER_HDC1080_ADDRESS;

    if (( err = hdc1080_read_temperature_humidity(&hdc1080, &tmp_raw_temperature16, NULL, &tmp_raw_humidity16, NULL)) != 0)
    {
        TRACE("err = %d\n", err);
        return OBTAIN_ERROR;
    }
#if 1
    *t = hdc1080_t_fpt(tmp_raw_temperature16);
    *h = raw_hum_to_fpt(tmp_raw_humidity16);
#endif

    return OBTAIN_TH;
}

static any_sensor_t sensor_hdc1080 = SENSOR_INIT(sensor_hdc1080_probe, sensor_hdc1080_obtain);

static uint8_t sensor_hdc1080_iic_read_with_wait(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    /* 
     *  NOTE(m): Delay can be lower when CLK is running lower.
     *  It might be related to some kind of rounding error.
     */
    uint8_t delay = 0;
    if (reg == 0x00) /* temperature */
    {
        switch(TEMP_RESOLUTION)
        {
            case HDC1080_TEMPERATURE_RESOLUTION_11_BIT:
                delay = 5;
                break;
            case HDC1080_TEMPERATURE_RESOLUTION_14_BIT:
            default:
                delay = 14;
                break;
        }
    }
    else if (reg == 0x01) /* humidity */
    {
        switch(HUMI_RESOLUTION)
        {
            case HDC1080_HUMIDITY_RESOLUTION_8_BIT:
                delay = 3;
                break;
            case HDC1080_HUMIDITY_RESOLUTION_11_BIT:
                delay = 4;
                break;
            case HDC1080_HUMIDITY_RESOLUTION_14_BIT:
            /* fallthrough */
            default:
                delay = 7;
                break;
        }
    }
    else
    {
        return libdriver_iic_addr_read(addr, reg, buf, len);
    }

    return libdriver_iic_addr_read_delay(addr, reg, buf, len, delay);
}

static any_sensor_t *sensor_hdc1080_new(arena_t *arena)
{
    if (sensor_hdc1080.sensor == NULL)
    {
        DRIVER_SET_DEFAULT_IIC_ADDR(HDC1080, &hdc1080, hdc1080_handle_t);
        DRIVER_HDC1080_LINK_IIC_READ_WITH_WAIT(&hdc1080, sensor_hdc1080_iic_read_with_wait);

        sensor_hdc1080.sensor = &hdc1080;
    }

    return &sensor_hdc1080;
}

SENSOR_FACTORY(HDC1080, DRIVER_HDC1080_ADDRESS, sensor_hdc1080_new, NULL);

#endif