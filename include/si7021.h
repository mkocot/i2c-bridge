#ifndef W_SI7021_H
#define W_SI7021_H

#include "common_driver.h"
#include "sensor.h"

#include <driver_si7021.h>

#define DRIVER_SI7021_ADDRESS 0x40

static si7021_handle_t si7021;

static uint8_t sensor_si7021_probe(any_sensor_t *ctx)
{
    uint8_t serial[8] = {0};
    /* set to "undefined" value */
    si7021_version_t version = (si7021_version_t)0;

    si7021.inited = 0;

    DO_OR(si7021_init(&si7021));

    DO_OR(si7021_get_firmware_revision(&si7021, &version));
    if (version != SI7021_VERSION_1 && version != SI7021_VERSION_2)
    {
        return 1;
    }

    DO_OR(si7021_get_serial_number(&si7021, serial));

    int zeros = 0;
    for (int i = 0; i < sizeof(serial) / sizeof(serial[0]); ++i)
    {
        zeros += serial[i] == 0;
    }

    if (zeros == sizeof(serial) / sizeof(serial[0]))
    {
        printf("???\n");
        return 1;
    }

    printf("Serial: %x%x%x%x%x%x%x%x\n", serial[0], serial[1], serial[2], serial[3], serial[4], serial[5], serial[6], serial[7]);

    DO_OR(si7021_set_heater(&si7021, SI7021_BOOL_FALSE));
    DO_OR(si7021_set_mode(&si7021, SI7021_MODE_NO_HOLD_MASTER));
    DO_OR(si7021_set_resolution(&si7021, SI7021_RESOLUTION_RH_12BIT_T_14_BIT));


    return 0;
}

static obtain_t sensor_si7021_obtain(any_sensor_t *ctx, int32_t *t, uint16_t *p, uint16_t *h)
{
    si7021.inited = 1;

    if (si7021_read(&si7021, &tmp_raw_temperature16, &tmp_temperature, &tmp_raw_humidity16, &tmp_humidity_f))
    {
        return OBTAIN_ERROR;
    }

    printf("T: %d, H: %d\n", (int)tmp_temperature, (int)tmp_humidity_f);


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