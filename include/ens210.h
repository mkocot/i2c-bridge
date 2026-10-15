#ifndef W_ENS210_H
#define W_ENS210_H


#include "common_driver.h"
#include "sensor.h"

#include <driver_ens210.h>

#include <fptc.h>

#define DRIVER_ENS210_ADDRESS (ENS210_ADDRESS >> 1)

static ens210_handle_t ens210;

/* T: raw/64 K (resolution: 1/64 K) */
static inline fpt ens210_t_fpt(uint16_t val) {
    fpt as_fpt = val;
    // B*2^32/65536 = fl2fpt(B) for the /65536 family (exact)
    as_fpt = fpt_mul(as_fpt, i2fpt(1024));
    as_fpt = fpt_sub(as_fpt, fl2fpt(273.15f));

    return as_fpt;
}

/* H: raw/512 %RH (resolution: 1/512 %RH) */
static inline fpt ens210_h_fpt(uint16_t val) {
    fpt as_fpt = val;
    as_fpt = fpt_mul(as_fpt, i2fpt(128));

    return as_fpt;
}

static obtain_t sensor_ens210_obtain(any_sensor_t *ctx, temperature_t *t, pressure_t *p, humidity_t *h)
{
    ((void)ctx);
    ((void)p);

    ens210.inited = 1;

    // T: -40 .. 100 (resolution: 1/64 K), bytes: 2
    // H: 0..100 (resolution: 1/512 %), bytes: 2

    /* one single shot step covers both sensors; the driver waits the
       datasheet maximum conversion time plus margin (150 ms) and reads both
       readout registers in one transaction (auto increment walks from T_VAL
       into H_VAL) */
    if (ens210_read_temperature_humidity(&ens210, &tmp_raw_temperature16, NULL, &tmp_raw_humidity16, NULL))
    {
        return OBTAIN_ERROR;
    }

    *t = ens210_t_fpt(tmp_raw_temperature16);
    *h = ens210_h_fpt(tmp_raw_humidity16);

    return OBTAIN_TH;
}

static uint8_t sensor_ens210_probe(any_sensor_t *ctx)
{
    ((void)ctx);

    ens210.inited = 0;

    /* init soft-resets the chip, boots it and reads PART_ID (0x0210) while
       active (boot-then-identify); 0x43 is unique in the fleet, no
       shared-address guard needed */
    return ens210_init(&ens210);
}

static any_sensor_t sensor_ens210 = SENSOR_INIT(
    sensor_ens210_probe,
    sensor_ens210_obtain
);

static any_sensor_t* sensor_ens210_new(arena_t *arena)
{
    if (sensor_ens210.sensor == NULL)
    {
        DRIVER_SET_DEFAULT_IIC_ADDR(ENS210, &ens210, ens210_handle_t);
        DRIVER_ENS210_LINK_DEBUG_PRINT(&ens210, debug_print);
        /* not required, as i2c address is set per sensor */
        ens210_set_addr_pin(&ens210, ENS210_ADDRESS);

        sensor_ens210.sensor = &ens210;
    }

    return &sensor_ens210;
}

SENSOR_FACTORY(ENS210, DRIVER_ENS210_ADDRESS, sensor_ens210_new, NULL);

#endif
