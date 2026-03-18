#ifndef I2C_BRIDGE_COMMON_DRIVER_H
#define I2C_BRIDGE_COMMON_DRIVER_H

#include <stdint.h>
#include <stdio.h> /* printf */
#include <lib_i2c.h>
#include <ch32fun.h>
#include "spi.h"
#include <fptc.h>

static i2c_device_t i2c = {
    .clkr = I2C_CLK_400KHZ, /* "default" */
    .type = I2C_ADDR_7BIT,  /* common addr type */
    .addr = 0x00,           /* device addres */
    .regb = 1,              /* register bytes, most devices uses 1 byte for register */
    .tout = 2000            /* cycles ?*/
};

static spi_device_t spi = {
  .nss_pin = -1,
};

/* temporal values used for reading sensor data */
static union {
  uint32_t u32;
  uint16_t u16;
  int16_t i16;
} tmp_raw_t;

#define tmp_raw_temperature (tmp_raw_t.u32)
#define tmp_raw_temperature16 (tmp_raw_t.u16)
#define tmp_raw_temperaturei16 (tmp_raw_t.i16)

static uint32_t tmp_raw_pressure;
static float tmp_temperature;
static float tmp_pressure;
static union {
  float f;
  uint16_t u16;
  uint8_t u8;
} tmp_h;

#define tmp_humidity_f (tmp_h.f)
#define tmp_humidity16 (tmp_h.u16)
#define tmp_humidity8 (tmp_h.u8)

static union {
  uint32_t u32;
  uint16_t u16;
} tmp_raw_h;

#define tmp_raw_humidity (tmp_raw_h.u32)
#define tmp_raw_humidity16 (tmp_raw_h.u16)

static inline void libdriver_delay_ms(uint32_t delay) { Delay_Ms(delay); }

static void debug_print(const char *const fmt, ...)
{
  printf(fmt);
}


uint8_t libdriver_iic_write(uint8_t reg, uint8_t *buf, uint16_t len);

uint8_t libdriver_iic_read(uint8_t reg, uint8_t *buf, uint16_t len);

uint8_t libdriver_iic_addr_read_noreg(uint8_t addr, uint8_t *buf, uint16_t len);

uint8_t libdriver_iic_addr_write_noreg(uint8_t addr, uint8_t *buf, uint16_t len);

uint8_t libdriver_iic_addr_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t libdriver_iic_addr_read_delay(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len, uint16_t ms);

uint8_t libdriver_iic_addr_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

uint8_t libdriver_iic_addr16_read(uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t len);

uint8_t libdriver_iic_addr16_write(uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t len);

static inline uint8_t libdriver_nop_void(void) { return 0; }

#define DRIVER_SET_DEFAULT_IIC(DRIVER, HANDLE, TYPE)                   \
  DRIVER_## DRIVER ##_LINK_INIT((HANDLE), TYPE);                       \
  DRIVER_## DRIVER ##_LINK_DEBUG_PRINT((HANDLE), NULL);                \
  DRIVER_## DRIVER ##_LINK_DELAY_MS((HANDLE), libdriver_delay_ms);     \
  DRIVER_## DRIVER ##_LINK_IIC_INIT((HANDLE), libdriver_nop_void);     \
  DRIVER_## DRIVER ##_LINK_IIC_DEINIT((HANDLE), libdriver_nop_void);   \
  DRIVER_## DRIVER ##_LINK_IIC_READ_CMD((HANDLE), libdriver_iic_read); \
  DRIVER_## DRIVER ##_LINK_IIC_WRITE_CMD((HANDLE), libdriver_iic_write)

#define DRIVER_SET_DEFAULT_IIC_ADDR(DRIVER, HANDLE, TYPE)               \
  DRIVER_## DRIVER ##_LINK_INIT((HANDLE), TYPE);                        \
  DRIVER_## DRIVER ##_LINK_DEBUG_PRINT((HANDLE), NULL);                 \
  DRIVER_## DRIVER ##_LINK_DELAY_MS((HANDLE), libdriver_delay_ms);      \
  DRIVER_## DRIVER ##_LINK_IIC_INIT((HANDLE), libdriver_nop_void);      \
  DRIVER_## DRIVER ##_LINK_IIC_DEINIT((HANDLE), libdriver_nop_void);    \
  DRIVER_## DRIVER ##_LINK_IIC_READ((HANDLE), libdriver_iic_addr_read); \
  DRIVER_## DRIVER ##_LINK_IIC_WRITE((HANDLE), libdriver_iic_addr_write)

#define DRIVER_SET_DEFAULT_IIC_ADDR16(DRIVER, HANDLE, TYPE)                         \
  DRIVER_## DRIVER ##_LINK_INIT((HANDLE), TYPE);                                    \
  DRIVER_## DRIVER ##_LINK_DEBUG_PRINT((HANDLE), debug_print);                      \
  DRIVER_## DRIVER ##_LINK_DELAY_MS((HANDLE), libdriver_delay_ms);                  \
  DRIVER_## DRIVER ##_LINK_IIC_INIT((HANDLE), libdriver_nop_void);                  \
  DRIVER_## DRIVER ##_LINK_IIC_DEINIT((HANDLE), libdriver_nop_void);                \
  DRIVER_## DRIVER ##_LINK_IIC_READ_ADDRESS16((HANDLE), libdriver_iic_addr16_read); \
  DRIVER_## DRIVER ##_LINK_IIC_WRITE_ADDRESS16((HANDLE), libdriver_iic_addr16_write)

static inline uint8_t libdriver_spi_write(uint8_t reg, uint8_t *buf, uint16_t len);
static inline uint8_t libdriver_spi_read(uint8_t reg, uint8_t *buf, uint16_t len);

#endif