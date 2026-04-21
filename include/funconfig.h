#ifndef _FUNCONFIG_H
#define _FUNCONFIG_H

// defined in platformio.ini
//#define CH32V003                    (1)
 
#define FUNCONF_DEBUG_HARDFAULT     (0)
#define FUNCONF_USE_DEBUGPRINTF     (1)

// #define FUNCONF_USE_HSE 0 // external crystal on PA1 PA2
#define FUNCONF_SYSTEM_CORE_CLOCK   (4000000)
#define FUNCONF_USE_HSI             (1) // internal 24MHz clock oscillator
// #define FUNCONF_USE_PLL             (1) // use PLL x2
#define FUNCONF_USE_PLL             (0) // use PLL x1
#define FUNCONF_PLL_MULTIPLIER      (1) // required when PLL is disabled?
#define FUNCONF_INIT_ANALOG         (0) // no analog is required
// #define FUNCONF_SYSTICK_USE_HCLK (1) // consider if this will have meaning

// I2C lib configuration
// PC1 (Pin 11) SDA
// PC2 (Pin 12) SCL
#define I2C_PINOUT_DEFAULT

/* for built-in library */

#define CH32V003_SPI_SPEED_HZ 1000000
#define CH32V003_SPI_CLK_MODE_POL0_PHA0
/* bollocks, it has only PC1 support */
#define CH32V003_SPI_NSS_HARDWARE_PC1
// #define CH32V003_SPI_NSS_SOFTWARE_PC3
#define CH32V003_SPI_DIRECTION_2LINE_TXRX

#endif