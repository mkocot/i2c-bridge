#ifndef W_ENS160_H
#define W_ENS160_H


#include "common_driver.h"
#include "sensor.h"

#include <driver_ens160.h>

#include <fptc.h>

#define DRIVER_ENS160_ADDRESS (ENS160_ADDRESS_1 >> 1)

/* wake to readable data: up to 1 s per datasheet (ENS160 DS, Table 9),
 * house rule waits past the published max (docs/sensor-timing.md) */
#define ENS160_MEASURE_WAIT_MS (1200)

/* bench test switch: continuous keeps the sensor measuring, reads need no
 * wake or wait; without it the sensor deep sleeps between reads (ADR-0011) */
#ifdef ENS160_TEST_CONTINUOUS
#define ENS160_MODE_TAG "cont"
#else
#define ENS160_MODE_TAG "ds"
#endif

static ens160_handle_t ens160;

/* unused but required by ens160_init's interface validation */
static void sensor_ens160_receive_nop(uint8_t type, uint16_t *dat)
{
    ((void)type);
    ((void)dat);
}

static obtain_t sensor_ens160_obtain(any_sensor_t *ctx, temperature_t *t, pressure_t *p, humidity_t *h)
{
    ((void)ctx);
    ((void)t);
    ((void)p);
    ((void)h);

    ens160.inited = 1;

#ifndef ENS160_TEST_CONTINUOUS
    /* deep sleep between reads: wake, wait out the first measurement, read,
       sleep again; on error the mode is left as-is, the next probe
       soft-resets into a known state */
    if (ens160_set_mode(&ens160, ENS160_MODE_STANDARD))
    {
        return OBTAIN_ERROR;
    }

    Delay_Ms(ENS160_MEASURE_WAIT_MS);
#endif

    uint8_t status;
    uint8_t aqi;
    uint16_t tvoc;
    uint16_t eco2;

    if (ens160_get_status(&ens160, &status) ||
        ens160_read_aqi(&ens160, &aqi) ||
        ens160_read_tvoc(&ens160, &tvoc) ||
        ens160_read_eco2(&ens160, &eco2))
    {
        return OBTAIN_ERROR;
    }

#ifndef ENS160_TEST_CONTINUOUS
    ens160_set_mode(&ens160, ENS160_MODE_DEEP_SLEEP);
#endif

    /* print-only (OBTAIN_GAS): values are logged here, never encoded into
       the HC-12 packet; status validity flags warm-up/start-up phases, and
       reads faster than the 1 s measurement period repeat the last values */
    DPRINTF("ens160: %s status 0x%02x aqi %d tvoc %u ppb eco2 %u ppm\n",
        ENS160_MODE_TAG, status, aqi, tvoc, eco2);

    return OBTAIN_GAS;
}

static uint8_t sensor_ens160_probe(any_sensor_t *ctx)
{
    ((void)ctx);

    ens160.inited = 0;

    /* PART_ID (0x0160) is readable in every mode: plain identity read, no
       boot-then-identify (unlike ENS210, ADR-0009); 0x53 (ADDR pin high) is
       unique in the fleet, no shared-address guard (ADR-0002 does not apply) */
    if (ens160_init(&ens160))
    {
        return 1;
    }

    /* known state: soft reset, then the idle mode */
    if (ens160_soft_reset(&ens160))
    {
        return 1;
    }

#ifdef ENS160_TEST_CONTINUOUS
    return ens160_set_mode(&ens160, ENS160_MODE_STANDARD);
#else
    return ens160_set_mode(&ens160, ENS160_MODE_DEEP_SLEEP);
#endif
}

static any_sensor_t sensor_ens160 = SENSOR_INIT(
    sensor_ens160_probe,
    sensor_ens160_obtain
);

static any_sensor_t* sensor_ens160_new(arena_t *arena)
{
    if (sensor_ens160.sensor == NULL)
    {
        DRIVER_SET_DEFAULT_IIC_ADDR(ENS160, &ens160, ens160_handle_t);
        DRIVER_ENS160_LINK_DEBUG_PRINT(&ens160, debug_print);
        /* unused but required by ens160_init */
        DRIVER_ENS160_LINK_SPI_INIT(&ens160, libdriver_nop_void);
        DRIVER_ENS160_LINK_SPI_DEINIT(&ens160, libdriver_nop_void);
        DRIVER_ENS160_LINK_SPI_READ(&ens160, libdriver_spi_read);
        DRIVER_ENS160_LINK_SPI_WRITE(&ens160, libdriver_spi_write);
        DRIVER_ENS160_LINK_RECEIVE_CALLBACK(&ens160, sensor_ens160_receive_nop);
        /* not required, as i2c address is set per sensor */
        ens160_set_addr_pin(&ens160, ENS160_ADDRESS_1);

        sensor_ens160.sensor = &ens160;
    }

    return &sensor_ens160;
}

SENSOR_FACTORY(ENS160, DRIVER_ENS160_ADDRESS, sensor_ens160_new, NULL);

#endif
