#include "common_driver.h"

#include <lib_i2c.h>


inline uint8_t libdriver_iic_write(uint8_t reg, uint8_t *buf, uint16_t len)
{
  i2c.regb = 1;
  return i2c_write_reg(&i2c, reg, buf, len);
}
inline uint8_t libdriver_iic_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
  i2c.regb = 1;
  return i2c_read_reg(&i2c, reg, buf, len);
}

inline uint8_t libdriver_iic_addr_read_noreg(uint8_t addr, uint8_t *buf, uint16_t len)
{
  i2c.addr = addr >> 1;

  return i2c_read_raw(&i2c, buf, len);
}

inline uint8_t libdriver_iic_addr_write_noreg(uint8_t addr, uint8_t *buf, uint16_t len)
{
  i2c.addr = addr >> 1;

  return i2c_write_raw(&i2c, buf, len);
}

// Somehow this function is special as compared to aht...
inline uint8_t libdriver_iic_addr_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  i2c.addr = addr >> 1;
  return libdriver_iic_read(reg, buf, len);
}

inline uint8_t libdriver_iic_addr_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  i2c.addr = addr >> 1;
  return libdriver_iic_write(reg, buf, len);
}

uint8_t libdriver_iic_addr16_read(uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t len)
{
  i2c.addr = addr >> 1;
  i2c.regb = 2;

  uint8_t ret;
  if (i2c.addr == 0x44)
  {
    /* this should be handled in SHT3X driver...
    * fetching data in 'single shot' is taking absurd 12ms. Setting this timeout
    * in i2c driver will be hard penalty during communication with other devices
    * to fix it we are using delayed read only for 2 commands. Anyway MCU could
    * do something usefull in that time...
    */
    switch (reg >> 8)
    {
      case 0x24: /* single read without clock stretching */
        /* fallthrough */
      case 0x2C: /* single read with clock stretching */
        ret = i2c_read_reg_delay(&i2c, reg, buf, len, 12);
        break;
      case 0xE0: /* sht35_continuous_read */
        /* fallthrough */
      default:
        goto regular_read;
    }
  }
  else if (i2c.addr == 0x70)
  {
    /* SHTC3 driver, see above
     * Low Power mode: ~ 0.7ms  0.8ms max
     * Normal mode   : 10.8ms   12.1ms max
     */
    switch (reg >> 8)
    {
      case 0x78: /* T without clock stretch */
        /* fallthrough */
      case 0x7C: /* T with clock stretch */
        /* fallthrough */
      case 0x58: /* RH without clock stretch */
        /* fallthrough */
      case 0x5C: /* RH with clock stretch */
        ret = i2c_read_reg_delay(&i2c, reg, buf, len, 11);
        break;
      default:
        goto regular_read;
    }
  }
  else
  {
    regular_read:
    ret = i2c_read_reg(&i2c, reg, buf, len);
  }

  printf("A16R: %X %X %u %u =%d\n", i2c.addr, reg, i2c.regb, len, ret);
  return ret;
}

uint8_t libdriver_iic_addr16_write(uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t len)
{
  i2c.addr = addr >> 1;
  i2c.regb = 2;

  uint8_t ret = i2c_write_reg(&i2c, reg, buf, len);
  printf("A16W: %X, %X, %u %u =%u\n", i2c.addr, reg, i2c.regb, len, ret);
  return ret;
}
