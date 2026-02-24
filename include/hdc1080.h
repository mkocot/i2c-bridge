#ifndef W_HDC1080_H
#define W_HDC1080_H

#include "common_driver.h"
#include "sensor.h"

#include <driver_hdc1080.h>

#define DRIVER_HDC1080_ADDRESS 0x40
#define TEMP_RESOLUTION HDC1080_TEMPERATURE_RESOLUTION_14_BIT
#define HUMI_RESOLUTION HDC1080_HUMIDITY_RESOLUTION_14_BIT

static hdc1080_handle_t hdc1080;

static uint8_t sensor_hdc1080_probe(any_sensor_t *ctx)
{
    uint8_t serial[6] = {0};

    hdc1080.inited = 0;

    i2c.addr = DRIVER_HDC1080_ADDRESS;

    DO_OR(hdc1080_init(&hdc1080));
    DO_OR(hdc1080_get_serial_id(&hdc1080, serial));


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

    printf("Serial: %x%x%x%x%x%x\n", serial[0], serial[1], serial[2], serial[3], serial[4], serial[5]);

    DO_OR(hdc1080_set_heater(&hdc1080, HDC1080_BOOL_FALSE));
    DO_OR(hdc1080_set_mode(&hdc1080, HDC1080_MODE_SEQUENCE));
    DO_OR(hdc1080_set_humidity_resolution(&hdc1080, HUMI_RESOLUTION));
    DO_OR(hdc1080_set_temperature_resolution(&hdc1080, TEMP_RESOLUTION));

    return 0;
}

static obtain_t sensor_hdc1080_obtain(any_sensor_t *ctx, int32_t *t, uint16_t *p, uint16_t *h)
{
    hdc1080.inited = 1;

    i2c.addr = DRIVER_HDC1080_ADDRESS;

    if (hdc1080_read_temperature_humidity(&hdc1080, &tmp_raw_temperature16, &tmp_temperature, &tmp_raw_humidity16, &tmp_humidity_f))
    {
        return OBTAIN_ERROR;
    }

    int full = (int)tmp_temperature;
    int fraction = (int)(tmp_temperature * 100 - full * 100);

    printf("T: %d.%d, H: %d\n", full, fraction, (int)tmp_humidity_f);

    return OBTAIN_TH;
}

static any_sensor_t sensor_hdc1080 = SENSOR_INIT(sensor_hdc1080_probe, sensor_hdc1080_obtain);

static uint8_t sensor_hdc1080_iic_read_with_wait(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t delay = 0;
    if (reg == 0x00) /* temperature */
    {
        switch(TEMP_RESOLUTION)
        {
            case HDC1080_TEMPERATURE_RESOLUTION_11_BIT:
                delay = 1; //5;
                break;
            case HDC1080_TEMPERATURE_RESOLUTION_14_BIT:
            default:
                delay = 2; //7;
                break;
        }
    }
    else if (reg == 0x01) /* humidity */
    {
        switch(HUMI_RESOLUTION)
        {
            case HDC1080_HUMIDITY_RESOLUTION_8_BIT:
                delay = 1;
                break;
            case HDC1080_HUMIDITY_RESOLUTION_11_BIT:
                delay = 1;
                break;
            case HDC1080_HUMIDITY_RESOLUTION_14_BIT:
            /* fallthrough */
            default:
                delay = 1;
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