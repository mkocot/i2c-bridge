#ifndef W_HTUXX_H
#define W_HTUXX_H


#include "common_driver.h"
#include "sensor.h"

#include <driver_htu21d.h>
#include <driver_htu31d.h>

#define DRIVER_HTU21D_ADDRESS (0x80 >> 1)
#define DRIVER_HTU31D_ADDRESS (HTU31D_ADDR_PIN_LOW >> 1)

static htu21d_handle_t htu21d;
static htu31d_handle_t htu31d;

static obtain_t sensor_htu21d_obtain(any_sensor_t *ctx, int32_t *t, uint16_t *p, uint16_t *h)
{
    i2c.addr = DRIVER_HTU21D_ADDRESS;
    htu21d.inited = 1;

    if (htu21d_read_temperature_humidity(&htu21d, &tmp_raw_temperature16, &tmp_temperature, &tmp_raw_humidity16, &tmp_humidity_f))
    {
        return OBTAIN_ERROR;
    }

    return OBTAIN_TH;
}

static obtain_t sensor_htu31d_obtain(any_sensor_t *ctx, int32_t *t, uint16_t *p, uint16_t *h)
{
    htu31d.inited = 1;

    // Temp: Q7.9 -64 ... ~64 (resolution: 0.001953125), bytes: 2
    // P: (500 .. 1524), hPa (resolution: 4), bytes: 1
    // H: 0..100 (resolution: 100/255%), bytes: 1

    if (htu31d_read_temperature_humidity(&htu31d, &tmp_raw_temperature16, &tmp_temperature, &tmp_raw_humidity16, &tmp_humidity_f))
    {
        return OBTAIN_ERROR;
    }

    int full_degrees = (int)tmp_temperature;
    int decimals = ((int)(tmp_temperature * 100)) & 100;
    printf("T: %d.%d\n", full_degrees, decimals);
    return OBTAIN_TH;
}

static uint8_t sensor_htu21d_probe(any_sensor_t *ctx)
{
    ((void)ctx);

    uint64_t serial;

    /* TODO:
        What is "better"
        Reset inited to 0 and then set to 1 in "obtain" or restore state?
    */
    i2c.addr = DRIVER_HTU21D_ADDRESS;

    htu21d.inited = 0;

    DO_OR(htu21d_init(&htu21d));
    /* TODO: check difference, repetivity and reliability */
    DO_OR(htu21d_set_mode(&htu21d, HTU21D_MODE_HOLD_MASTER));

    /* quick check if connection is OK */
    DO_OR(htu21d_get_serial_number(&htu21d, &serial));

    /* default values */
    #if 0
    DO_OR(htu21d_set_heater(&htu21d, HTU21D_BOOL_FALSE));
    DO_OR(htu21d_set_resolution(&htu21d, HTU21D_RESOLUTION_TEMP_14_BITS_RH_12_BITS));
    DO_OR(htu21d_set_disable_otp_reload(&htu21d, HTU21D_BOOL_TRUE));
    #endif

    return 0;
}

static uint8_t sensor_htu31d_probe(any_sensor_t *ctx)
{
    ((void)ctx);
    uint8_t serial[3];

    htu31d.inited = 0;

    DO_OR(htu31d_init(&htu31d));
    /* it's default ON or OFF ?!*/
    DO_OR(htu31d_set_heater_off(&htu31d));
    /* TODO: get_humidity_osr and compare */
    DO_OR(htu31d_set_humidity_osr(&htu31d, HTU31D_HUMIDITY_OSR_LOW));
    /* TODO: get_temperatrure_osr and compare */
    DO_OR(htu31d_set_temperature_osr(&htu31d, HTU31D_TEMPERATURE_OSR_VERY_HIGH));

    /* Connection check */
    DO_OR(htu31d_get_serial_number(&htu31d, serial));

    printf("SERIAL: %x%x%x\n", serial[0], serial[1], serial[2]);

    // All "0" is invalid
    return !(serial[0] != 0 && serial[1] != 0 && serial[2] != 0);
}


static any_sensor_t sensor_htu21d = SENSOR_INIT(
    sensor_htu21d_probe,
    sensor_htu21d_obtain
);

static any_sensor_t sensor_htu31d = SENSOR_INIT(
    sensor_htu31d_probe,
    sensor_htu31d_obtain
);

static any_sensor_t* sensor_htu21d_new(arena_t *arena)
{
    if (sensor_htu21d.sensor == NULL)
    {
        DRIVER_SET_DEFAULT_IIC(HTU21D, &htu21d, htu21d_handle_t);
        DRIVER_HTU21D_LINK_IIC_READ_WITH_SCL(&htu21d, libdriver_iic_addr_read);

        sensor_htu21d.sensor = &htu21d;
    }

    return &sensor_htu21d;
}

static uint8_t sensor_htu21d_match(i2c_addr_t addr)
{
    return addr == DRIVER_HTU21D_ADDRESS;
}

static any_sensor_t* sensor_htu31d_new(arena_t *arena)
{
    if (sensor_htu31d.sensor == NULL)
    {
        // DRIVER_SET_DEFAULT_IIC(HTU31D, &htu31d, htu31d_handle_t);
        DRIVER_SET_DEFAULT_IIC_ADDR(HTU31D, &htu31d, htu31d_handle_t);
        DRIVER_HTU31D_LINK_DEBUG_PRINT(&htu31d, debug_print);
        /* not required, as i2c address is set per sensor */
        htu31d_set_addr_pin(&htu31d, DRIVER_HTU31D_ADDRESS << 1);

        sensor_htu31d.sensor = &htu31d;
    }

    return &sensor_htu31d;
}

static uint8_t sensor_htu31d_match(i2c_addr_t addr)
{
    return addr == DRIVER_HTU31D_ADDRESS;
}

SENSOR_FACTORY(HTU21D, DRIVER_HTU21D_ADDRESS, sensor_htu21d_new, NULL);
SENSOR_FACTORY(HTU31D, DRIVER_HTU31D_ADDRESS, sensor_htu31d_new, NULL);

#endif