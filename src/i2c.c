#include "i2c.h"

i2c_device_t i2c = {
    .clkr = I2C_CLK_400KHZ, /* "default" */
    .type = I2C_ADDR_7BIT,  /* common addr type */
    .addr = 0x00,           /* device addres */
    .regb = 1,              /* register bytes, most devices uses 1 byte for register */
    .tout = 2000            /* cycles ?*/
};
