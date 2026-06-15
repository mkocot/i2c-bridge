#ifndef I2C_BRIDGE_COMMON_DRIVER_H
#define I2C_BRIDGE_COMMON_DRIVER_H

#include <stdint.h>
#include <stdio.h> /* printf */
#include <lib_i2c.h>
#include <ch32fun.h>
#include "spi.h"
#include <fptc.h>

/* shared i2c struct */
extern i2c_device_t i2c;

/* shared spi struct */
extern spi_device_t spi;

/* temporal values used for reading sensor data */
typedef union {
  uint32_t u32;
  uint16_t u16;
  int16_t i16;
} tmp_raw_t_t;

extern tmp_raw_t_t tmp_raw_t;

#define tmp_raw_temperature (tmp_raw_t.u32)
#define tmp_raw_temperature16 (tmp_raw_t.u16)
#define tmp_raw_temperaturei16 (tmp_raw_t.i16)

extern uint32_t tmp_raw_pressure;
extern float tmp_temperature;
extern float tmp_pressure;

typedef union {
  float f;
  uint16_t u16;
  uint8_t u8;
} tmp_h_t;

extern tmp_h_t tmp_h;

#define tmp_humidity_f (tmp_h.f)
#define tmp_humidity16 (tmp_h.u16)
#define tmp_humidity8 (tmp_h.u8)

typedef union {
  uint32_t u32;
  uint16_t u16;
} tmp_raw_h_t;

extern tmp_raw_h_t tmp_raw_h;

#define tmp_raw_humidity (tmp_raw_h.u32)
#define tmp_raw_humidity16 (tmp_raw_h.u16)

static inline void libdriver_delay_ms(uint32_t delay) { Delay_Ms(delay); }

static void debug_print(const char *const fmt, ...)
{
  printf("%s", fmt);
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

uint8_t libdriver_spi_write(uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t libdriver_spi_read(uint8_t reg, uint8_t *buf, uint16_t len);

static uint8_t i2c_send_stop(i2c_device_t *i2c, bool enable)
{
  if (enable) {
    I2C1->CTLR1 |= CTLR1_STOP_Set;
  } else {
    I2C1->CTLR1 &= CTLR1_STOP_Reset;
  }
}

static uint8_t i2c_send_start(i2c_device_t *i2c, bool enable)
{
  if (enable) {
    I2C1->CTLR1 |= CTLR1_START_Set;
  } else {
    I2C1->CTLR1 &= CTLR1_START_Reset;
  }
}

static uint8_t i2c_clock_stretch(i2c_device_t *i2c, bool enable)
{
  if (enable) {
    I2C1->CTLR1 |= CTLR1_NOSTRETCH_Set;
  } else {
    I2C1->CTLR1 &= CTLR1_NOSTRETCH_Reset;
  }
}

#endif