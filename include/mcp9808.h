#ifndef W_MCP9808_H
#define W_MCP9808_H

#include "common_driver.h"
#include "sensor.h"

#include <driver_mcp9808.h>

static inline fpt mcp9808_t_fpt(int16_t val) {
    fpt as_fpt = fpt_mul(i2fpt(val), fl2fpt(1.0f/16.0f));

    return as_fpt;
}

static mcp9808_handle_t mcp9808;

static uint8_t sensor_mcp9808_probe(any_sensor_t *ctx)
{
    uint8_t revision;
    mcp9808.inited = 0;

    DO_OR(mcp9808_init(&mcp9808));
    DO_OR(mcp9808_set_resolution(&mcp9808, MCP9808_RESOLUTION_0P0625));
    DO_OR(mcp9808_get_device_revision(&mcp9808, &revision));

    return 0;
}

static obtain_t sensor_mcp9808_obtain(any_sensor_t *ctx, int32_t *t, uint16_t *p, uint16_t *h)
{
    if (mcp9808_read(&mcp9808, &tmp_raw_temperaturei16, NULL))
    {
        return OBTAIN_ERROR;
    }

    *t = QUANTIZE_TEMP(mcp9808_t_fpt(tmp_raw_temperaturei16));

    // printf("T: %d\n", (int)tmp_temperature);

    return OBTAIN_TEMPERATURE;
}

static any_sensor_t sensor_mcp9808 = SENSOR_INIT(sensor_mcp9808_probe, sensor_mcp9808_obtain);

static void mcp9808_callback(uint8_t type)
{

}

static any_sensor_t *sensor_mcp9808_new(arena_t *arena)
{
    if (sensor_mcp9808.sensor == NULL)
    {
        DRIVER_SET_DEFAULT_IIC_ADDR(MCP9808, &mcp9808, mcp9808_handle_t);
        DRIVER_MCP9808_LINK_RECEIVE_CALLBACK(&mcp9808, mcp9808_callback);
        mcp9808_set_addr(&mcp9808, MCP9808_ADDRESS_A2A1A0_000);

        sensor_mcp9808.sensor = &mcp9808;
    }

    return &sensor_mcp9808;
}

SENSOR_FACTORY(MCP9808, MCP9808_ADDRESS_A2A1A0_000 >> 1, sensor_mcp9808_new, NULL);

#endif