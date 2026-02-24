#ifndef W_SPI_H
#define W_SPI_H

#include <ch32fun.h>
#include <ch32v003_GPIO_branchless.h>
#include <stdio.h>

/**
 * All methods return conforms to standard C rules where 0 means OK
 */

/* 1：Tx buffer empty */
#define SPI_WAIT_FOR_EMPTY_TX() \
  do {} while((SPI1->STATR & SPI_STATR_TXE) != SPI_STATR_TXE)

/* 1：SPI is busy in communication or Tx buffer is not empty */
#define SPI_WAIT_FOR_IDLE() \
	do {} while((SPI1->STATR & SPI_STATR_BSY) == SPI_STATR_BSY)

/* 1：Rx buffer not empty */
#define SPI_WAIT_FOR_RX_AVAILABLE() \
	do {} while((SPI1->STATR & SPI_STATR_RXNE) != SPI_STATR_RXNE)


#define SPI_WAIT_TRANSFER_COMPLETE() \
  do { SPI_WAIT_FOR_EMPTY_TX(); SPI_WAIT_FOR_IDLE() } while (0)

typedef struct spi_init_s spi_init_t;
typedef struct spi_device_s spi_device_t;
typedef enum spi_err_e spi_err_t;

/**
 * @brief Initialize SPI peripheral in Master mode
 * @param spi_device_t device handle
 * @param spi_init_t device config
 * 
 * @return spi_err_t, SPI_ERR_OK (0) when OK
 */
static spi_err_t spi_init(spi_device_t *handle, const spi_init_t *cfg);

/**
 * @brief Begin SPI transaction
 * @param spi_device_t handle to initialized SPI device
 * 
 * @return spi_err_t, SPI_ERR_OK (0) when OK
 */
static spi_err_t spi_begin_transaction(spi_device_t *handle);

/**
 * @brief Begin SPI transaction
 * @param spi_device_t handle to initialized SPI device
 * 
 * @return spi_err_t, SPI_ERR_OK (0) when OK
 */
static spi_err_t spi_end_transaction(spi_device_t *handle);

/**
 * @brief Receive 8 bits of data. Requires active transaction.
 *
 * @param uint8_t dummy value to trigger transaction
 * 
 * @return received data
 */
static uint8_t spi_recv8(uint8_t dummy);

/**
 * @brief Sends 8 bits of data. Requires active transaction.
 *
 * @param uint8_t data to send
 */
static void spi_send8(uint8_t data);


/** 
 * @brief SPI Mode:
 * First digit is CPOL LOW[0] or HIGH[1]
 * 
 * Second digit is SPHA 1st Edge[0] or 2nd Edge[1]
 * 
 * Single digit mode is alias eg. 3 is 0b11, 1 is 0b01
*/
typedef enum spi_mode_e {
  SPI_MODE00 = SPI_CPOL_Low | SPI_CPHA_1Edge,
  SPI_MODE01 = SPI_CPOL_Low | SPI_CPHA_2Edge,
  SPI_MODE10 = SPI_CPOL_High | SPI_CPHA_1Edge,
  SPI_MODE11 = SPI_CPOL_High | SPI_CPHA_2Edge,
  /* aliases */
  SPI_MODE0 = SPI_MODE00,
  SPI_MODE1 = SPI_MODE01,
  SPI_MODE2 = SPI_MODE10,
  SPI_MODE3 = SPI_MODE11,
} spi_mode_t;

/**
 * @brief SPI frame size:
 * SPI_FRAME_8BITS transfers 8bits
 * 
 * SPI_FRAME_16BITS transfers 16bits
 * 
 */
typedef enum spi_frame_e {
  SPI_FRAME_8BITS = SPI_DataSize_8b,
  SPI_FRAME_16BITS = SPI_DataSize_16b,
} spi_frame_t;

/**
 * @brief SPI byte order:
 * SPI_MSB_FIRST Most Significant bit first
 * SPI_LSB_FIRST Least Significant bit first
 */
typedef enum spi_byte_order_e {
  SPI_MSB_FIRST = SPI_FirstBit_MSB,
  SPI_LSB_FIRST = SPI_CTLR1_LSBFIRST,
} spi_byte_order_t;


struct spi_device_s
{
  /* use hardware or software NSS */
  uint8_t nss_pin;
};


static uint8_t spi_off()
{
  /* Disable clock for SPI1 (GPIO C might be used somewhere else) */
  RCC->APB2PCENR &= ~RCC_APB2Periph_SPI1;

  return 0;
}

static uint8_t spi_on()
{
  /* enable GPIO C */
  funGpioInitC();

  /* Start clocks for: GPIO C and SPI1 */
	RCC->APB2PCENR |= RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOC;

  return 0;
}

typedef struct spi_init_s {
  /* 0 - default */
  uint8_t nss_pin;
  /* 0 - CPOL0 CPHA0 */
  spi_mode_t mode;
  /* 0 - 8b */
  spi_frame_t frame_size;
  /* 0 - MSB first */
  spi_byte_order_t byte_order;
  /* 0 - full duplex 2 lines */
} spi_init_t;

typedef enum spi_err_e {
  SPI_ERR_OK = 0, /* no error, all good */
  SPI_INIT_ERR_FRAME_SIZE, /* init related */
  SPI_INIT_ERR_NSS_PIN, /* init releated */
  SPI_INIT_ERR_BYTE_ORDER, /* init related */
  SPI_INIT_ERR_MODE, /* init related */
  SPI_NOT_INITIALIZED, /* spi device is not initialized */
} spi_err_t;

#define FUN_OUTPUT_MULTIPLEXED (GPIO_CFGLR_OUT_10Mhz_AF_PP)
#define SPI_IS_HW_PIN(PIN) \
  (((PIN) == PC0) || ((PIN) == PC1))

#define SPI_INITIALIZED(DEVICE) \
  (!!(DEVICE)->nss_pin)


static spi_err_t spi_init(spi_device_t *handle, const spi_init_t *cfg)
{

  uint16_t config = SPI_Mode_Master;
  config |= SPI_Direction_2Lines_FullDuplex;
  /* 24Mhz@48MHz MCU
  BR -> Configure clock */
  config |= SPI_BaudRatePrescaler_16; // 48 / 16 -> 3
  /*  SSI (1-> NSS pin is HIGH, 0 -> NSS pin is LOW on selection)
  REQUIRED TO BE 1 */
  config |= SPI_NSSInternalSoft_Set;

  if (cfg->frame_size != SPI_FRAME_8BITS && cfg->frame_size != SPI_FRAME_16BITS)
  {
    return SPI_INIT_ERR_FRAME_SIZE;
  }

  if (cfg->nss_pin >= PC5 && cfg->nss_pin <= PC7 || cfg->nss_pin == 0)
  {
    /* not allowed to be any of HW SPI pins */
    return SPI_INIT_ERR_NSS_PIN;
  }

  if (cfg->byte_order != SPI_MSB_FIRST && cfg->byte_order != SPI_LSB_FIRST)
  {
    return SPI_INIT_ERR_BYTE_ORDER;
  }

  if (cfg->mode < SPI_MODE00 || cfg->mode > SPI_MODE11)
  {
    return SPI_INIT_ERR_MODE;
  }

  handle->nss_pin = cfg->nss_pin;

  funGpioInitC();
  /* Enable clock for PORTC, SPI1 */
  RCC->APB2PCENR |= RCC_IOPCEN | RCC_APB2Periph_SPI1;

  funPinMode(PC7, FUN_INPUT);               /* MISO (17) */
  funPinMode(PC6, FUN_OUTPUT_MULTIPLEXED);  /* MOSI (16) */
  funPinMode(PC5, FUN_OUTPUT_MULTIPLEXED);  /* SCK (15) */

  /*
    Hardware Master or Slave mode:  Float, pull-up or pull-down input
    Hardware Master mode/NSS output enable mode: Push-pull multiplexed output */
  if (SPI_IS_HW_PIN(handle->nss_pin))
  {
    /* SSM 1: software control, 0: hardware controle */
    config |= SPI_NSS_Hard;

    if (!handle->nss_pin || handle->nss_pin == PC1)
    {
      funPinMode(PC1, FUN_OUTPUT_MULTIPLEXED);  /* NSS (11) */
    }
    else
    {
      /* remap HW NSS to PC0 */
      AFIO->PCFR1 |= GPIO_Remap_SPI1;
      funPinMode(PC0, FUN_OUTPUT_MULTIPLEXED);  /* NSS (10) */
    }
    /* Enable SS output */
    SPI1->CTLR2 = CTLR2_SSOE_Set;
  }
  else
  {
    /* SSM 1: software control, 0: hardware controle */
    config |= SPI_NSS_Soft;

    funPinMode(handle->nss_pin, FUN_OUTPUT_MULTIPLEXED);
    /* SPI_Mode_Master can only be selected when using HARDWARE control
     or SS is alredy pulled HIGH on SOFTWARE control */
    spi_end_transaction(handle);
  }

  SPI1->CTLR1 = 0;

  config |= cfg->mode;
  /* DEF set to 8bits (value 0) or 16 */
  config |= cfg->frame_size;

  /* Frame Format */
  config |= cfg->byte_order;

  /*Set SPI1, max clock 48Mhz/2 = 24Mhz, master mode, full-duplex mode,8bit data length
	Internal slave select and software slave managment
	SPI1->CTLR1 = SPI_CTLR1_SSI | SPI_CTLR1_SSM | SPI_CTLR1_MSTR; //| SPI_CTLR1_BR_1 | SPI_CTLR1_BR_0;
  */
  SPI1->CTLR1 = config;

  return SPI_ERR_OK;
}


static void spi_send8(uint8_t data)
{
  if (!(SPI1->CTLR1 & SPI_CTLR1_SPE))
  {
    /* abort when SPI is not enabled */
    return;
  }

  SPI_WAIT_FOR_EMPTY_TX();

	SPI1->DATAR = data;
  SPI_WAIT_FOR_IDLE();

  // not required?
	// while((SPI1->STATR & SPI_STATR_RXNE) != SPI_STATR_RXNE){};

  /* WARNING: discaring result is mandatory */
  data = (uint8_t)SPI1->DATAR;
}

static uint8_t spi_recv8(uint8_t dummy)
{
  if (!(SPI1->CTLR1 & SPI_CTLR1_SPE))
  {
    /* abort when SPI is not enabled */
    return dummy;
  }

	SPI1->DATAR = dummy;
  SPI_WAIT_FOR_RX_AVAILABLE();
	
	return (uint8_t)SPI1->DATAR;
}

spi_err_t spi_begin_transaction(spi_device_t *handle)
{
  if (!SPI_INITIALIZED(handle))
  {
    return SPI_NOT_INITIALIZED;
  }

  {
    SPI1->CTLR1 |= SPI_CTLR1_SPE;
  }

  if (SPI_IS_HW_PIN(handle->nss_pin))
  {
    return SPI_ERR_OK;
  }

  funDigitalWrite(handle->nss_pin, FUN_HIGH);

  return SPI_ERR_OK;

}

spi_err_t spi_end_transaction(spi_device_t *handle)
{
  if (!SPI_INITIALIZED(handle))
  {
    return SPI_NOT_INITIALIZED;
  }

  {
    SPI1->CTLR1 &= ~SPI_CTLR1_SPE;
  }

  if (SPI_IS_HW_PIN(handle->nss_pin))
  {
    return SPI_ERR_OK;
  }

  funDigitalWrite(handle->nss_pin, FUN_LOW);
  
  return SPI_ERR_OK;
}

/* de pollute namespace */
#undef FUN_OUTPUT_MULTIPLEXED
#undef SPI_IS_HW_PIN
#undef SPI_WAIT_FOR_EMPTY_TX
#undef SPI_WAIT_FOR_IDLE
#undef SPI_WAIT_FOR_RX_AVAILABLE
#undef SPI_WAIT_TRANSFER_COMPLETE
#undef SPI_INITIALIZED

#endif
