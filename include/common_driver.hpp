#ifndef I2C_BRIDGE_COMMON_DRIVER_H
#define I2C_BRIDGE_COMMON_DRIVER_H

#include <Arduino.h>

static uint8_t dummy_uint8_t_no_op()
{
    return 0;
}

static void dummy_debug_print(const char *fmt, ...)
{
  // Serial1.println(__FUNCTION__);
  // va_list args;
  // va_start(args, fmt);
  // Serial1.printf(fmt, args);
  // va_end(args);
}

static void delay_ms(const uint32_t ms)
{
    delay(ms);
}

#endif