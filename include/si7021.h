#ifndef W_SI7021_H
#define W_SI7021_H

#include "common_driver.h"
#include "sensor.h"

#include <driver_si7021.h>

static inline float si7021_h_fpt(uint16_t val) {
    fpt as_fpt = val;
    as_fpt = fpt_mul(as_fpt, i2fpt(125));
    as_fpt = fpt_sub(as_fpt, i2fpt(6));

    return as_fpt;
}

static inline fpt si7021_t_fpt(uint16_t val) {
    fpt as_fpt = val;
    // +1 reduces maximum error from 0.000031 to 0.000023
    as_fpt = fpt_mul(as_fpt, fl2fpt(175.72f) + 1);
    as_fpt = fpt_sub(as_fpt, fl2fpt(46.85f));

    return as_fpt;
}

#define DRIVER_SI7021_ADDRESS 0x40

static si7021_handle_t si7021;

static uint8_t sensor_si7021_probe(any_sensor_t *ctx)
{
    si7021.inited = 0;

    DO_OR(si7021_init(&si7021));
    DO_OR(si7021_set_heater(&si7021, SI7021_BOOL_FALSE));
    DO_OR(si7021_set_mode(&si7021, SI7021_MODE_NO_HOLD_MASTER));
    DO_OR(si7021_set_resolution(&si7021, SI7021_RESOLUTION_RH_12BIT_T_14_BIT));

    return 0;
}

static obtain_t sensor_si7021_obtain(any_sensor_t *ctx, temperature_t *t, pressure_t *p, humidity_t *h)
{
    si7021.inited = 1;

    if (si7021_read(&si7021, &tmp_raw_temperature16, NULL, &tmp_raw_humidity16, NULL))
    {
        return OBTAIN_ERROR;
    }

    *t = si7021_t_fpt(tmp_raw_temperature16);
    *h = si7021_h_fpt(tmp_raw_humidity16);

    return OBTAIN_TH;
}

static any_sensor_t sensor_si7021 = SENSOR_INIT(sensor_si7021_probe, sensor_si7021_obtain);

static any_sensor_t *sensor_si7021_new(arena_t *arena)
{
    if (sensor_si7021.sensor == NULL)
    {
        DRIVER_SET_DEFAULT_IIC_ADDR(SI7021, &si7021, si7021_handle_t);
        DRIVER_SI7021_LINK_IIC_READ_ADDRESS16(&si7021, libdriver_iic_addr16_read);
        DRIVER_SI7021_LINK_IIC_READ_WITH_DELAY(&si7021, libdriver_iic_addr_read_delay);

        sensor_si7021.sensor = &si7021;
    }

    return &sensor_si7021;
}

SENSOR_FACTORY(SI7021, DRIVER_SI7021_ADDRESS, sensor_si7021_new, NULL);

#endif