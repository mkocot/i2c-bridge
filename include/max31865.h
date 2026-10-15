#ifndef W_MAX31865_H
#define W_MAX31865_H

#include "common_driver.h"
#include "sensor.h"

#include <driver_max31865.h>

/* same 0.0625 C/LSB output format as MCP9808; the packet carries the raw
 * value (PACKET_BANK_PT100), conversion happens on the received side */
static inline fpt max31865_t_fpt(int16_t val) {
    fpt as_fpt = fpt_mul(i2fpt(val), fl2fpt(1.0f/16.0f));

    return as_fpt;
}

static max31865_handle_t max31865;

static uint8_t sensor_max31865_driver_init()
{
  DRIVER_MAX31865_LINK_INIT(&max31865, max31865_handle_t);
  DRIVER_MAX31865_LINK_DEBUG_PRINT(&max31865, debug_print);
  DRIVER_MAX31865_LINK_DELAY_MS(&max31865, libdriver_delay_ms);
  DRIVER_MAX31865_LINK_SPI_DEINIT(&max31865, libdriver_nop_void);
  DRIVER_MAX31865_LINK_SPI_INIT(&max31865, libdriver_nop_void);
  DRIVER_MAX31865_LINK_SPI_READ(&max31865, libdriver_spi_read);
  DRIVER_MAX31865_LINK_SPI_WRITE(&max31865, &libdriver_spi_write);

  return 0;
}

/* better name for this, not "init" per se, more like create? */
static uint8_t sensor_max31865_init()
{
  sensor_max31865_driver_init();

  DPRINTF("SPI init\n");

  spi_init_t cfg = {
    .byte_order = SPI_MSB_FIRST,
    .frame_size = SPI_FRAME_8BITS,
    .mode = SPI_MODE1, // 1 or 3
    .nss_pin = PC0,
  };

  if (spi_init(&spi, &cfg) != SPI_ERR_OK)
  {
    DPRINTF("SPI init failed\n");

    return 1;
  }

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

#endif
