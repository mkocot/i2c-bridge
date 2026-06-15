#ifndef W_MCP9808_H
#define W_MCP9808_H

#include "common_driver.h"
#include "sensor.h"

#include <driver_max31865.h>

static inline fpt mcp9808_t_fpt(int16_t val) {
    fpt as_fpt = fpt_mul(i2fpt(val), fl2fpt(1.0f/16.0f));

    return as_fpt;
}

static max31865_handle_t max31865;

static uint8_t sensor_mcp9808_probe(any_sensor_t *ctx)
{
    max31865.inited = 0;

  uint8_t err;
  if ((err = max31865_init(&max31865)))
  {
    DPRINTF("PT100 init failed: %d\n", err);
    return err;
  }

  if ((err = max31865_set_fault_detection_cycle_control(&max31865, MAX31865_FAULT_DETECTION_CYCLE_CONTROL_NO_ACTION)))
  {
    DPRINTF("PT100 sfdcc: %d\n", err);
    return err;
  }
  /* we are using manual fault check, */
  #if 0
  max31865_set_high_fault_threshold
  max31865_set_low_fault_threshold
  #endif

  /* should not matter at all, it's battery powered device */
  #if 0
  max31865_set_filter_select(&max31865, MAX31865_FILTER_SELECT_50HZ);
  #endif


  /* required when fetching temperature directly, debug only */
  if ((err = max31865_set_reference_resistor(&max31865, 430.0f)))
  {
    DPRINTF("reference fail\n");
    return err;
  }
  if ((err = max31865_set_resistor(&max31865, MAX31865_RESISTOR_100PT)))
  {
    DPRINTF("resistor failed\n");
    return err;
  }

  /* ensure voltage is not enabled by default, it might be on when only MCU has gone
   * through power cycle
   */
  if ((err = max31865_set_vbias(&max31865, MAX31865_BOOL_FALSE)))
  {
    DPRINTF("vbias failed\n");
    return err;
  }

  if ((err = max31865_set_wire(&max31865, MAX31865_WIRE_3)))
  {
    DPRINTF("wire failed\n");
  }

  return err;
}

static obtain_t sensor_mcp9808_obtain(any_sensor_t *ctx, int32_t *t, uint16_t *p, uint16_t *h)
{
    max31865.inited = 1;

    if (max31865_single_read(&max31865, &tmp_raw_temperature16, NULL))
    {
        return OBTAIN_ERROR;
    }

    *t = mcp9808_t_fpt(tmp_raw_temperaturei16);

    // printf("T: %d\n", (int)tmp_temperature);

    return OBTAIN_TEMPERATURE;
}

static any_sensor_t sensor_mcp9808 = SENSOR_INIT(sensor_mcp9808_probe, sensor_mcp9808_obtain);

static void mcp9808_callback(uint8_t type)
{

}

static any_sensor_t *sensor_max31865_new(arena_t *arena)
{
    if (sensor_mcp9808.sensor == NULL)
    {
        DRIVER_MAX31865_LINK_INIT(&max31865, max31865_handle_t);
        DRIVER_MAX31865_LINK_DEBUG_PRINT(&max31865, debug_print);
        DRIVER_MAX31865_LINK_DELAY_MS(&max31865, libdriver_delay_ms);
        DRIVER_MAX31865_LINK_SPI_DEINIT(&max31865, libdriver_nop_void);
        DRIVER_MAX31865_LINK_SPI_INIT(&max31865, libdriver_nop_void);
        DRIVER_MAX31865_LINK_SPI_READ(&max31865, libdriver_spi_read);
        DRIVER_MAX31865_LINK_SPI_WRITE(&max31865, &libdriver_spi_write);

        sensor_mcp9808.sensor = &mcp9808;
    }

    return &sensor_mcp9808;
}

SENSOR_FACTORY(MAX31865, 0xFF, sensor_max31865_new, NULL);

#endif