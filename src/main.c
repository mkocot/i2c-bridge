#include "ahtxx.h"
#include "bmp280.h"
#include "common_driver.h"
#include "shtxx.h"

#include <stdio.h>
#include <ch32fun.h>
#include <ch32v003_GPIO_branchless.h>

#include <lib_i2c.h>
#include <driver_aht20.h>
#include <driver_aht30.h>
#include <driver_sht35.h>
#include <driver_sht4x.h>

#include <driver_bmp280.h>
#include <driver_bme280.h>
#include <driver_max31865.h>
#include <driver_tca9548a.h>
#include <driver_shtc3.h>


union any_sensor_u {
  aht30_handle_t aht;
  bmp280_handle_t bmp280;
};

typedef union any_sensor_u any_sensor_t;

static max31865_handle_t max31865;
static tca9548a_handle_t tca9548a;
// sht, dht, ...

/* SPI Mode Definition */
#define HOST_MODE 0
#define SLAVE_MODE 1

/* SPI Communication Mode Selection */
#define SPI_MODE HOST_MODE

#define SPI_CS_PIN GPIO_Pin_3
#define SPI_CS_PORT GPIOC

struct spi_device_s
{
  /* use hardware or software NSS */
  uint8_t nss_pin;
  uint8_t regb;
};

typedef struct spi_device_s spi_device_t;

typedef enum spi_mode_e {
  SPI_MODE00 = SPI_CPOL_Low | SPI_CPHA_1Edge,
  SPI_MODE01 = SPI_CPOL_Low | SPI_CPHA_2Edge,
  SPI_MODE10 = SPI_CPOL_High | SPI_CPHA_1Edge,
  SPI_MODE11 = SPI_CPOL_High | SPI_CPHA_2Edge,
  /* aliases */
  SPI_MODE0 = SPI_MODE00,
  SPI_MODE3 = SPI_MODE11,
  /* mask */
  SPI_MODE_MASK = SPI_MODE11,
} spi_mode_t;

typedef enum spi_frame_e {
  SPI_FRAME_8BITS,
  SPI_FRAME_16BITS,
} spi_frame_t;


typedef enum spi_byte_order_e {
  SPI_MSB_FIRST,
  SPI_LSB_FIRST,
} spi_byte_order_t;

uint8_t spi_begin_transaction(spi_device_t *handle);
uint8_t spi_end_transaction(spi_device_t *handle);

spi_device_t spi;

#define FUN_OUTPUT_MULTIPLEXED (GPIO_CFGLR_OUT_10Mhz_AF_PP)

// static uint8_t spi_init_software_nss(uint8_t pin)
// {
//   funPinMode(pin, GPIO_CFGLR_IN_PUPD);

//   return 0;
// }

static uint8_t spi_init(spi_device_t *handle, uint8_t nss_pin, spi_mode_t mode, spi_frame_t frame_size, spi_byte_order_t msb_first)
{
  if (frame_size != 8 && frame_size != 16)
  {
    return 1;
  }

  handle->nss_pin = nss_pin;
  handle->regb = frame_size;

  funGpioInitC();
  //Enable clock for PORTC, SPI1
  RCC->APB2PCENR |= RCC_IOPCEN | RCC_SPI1EN;

  funPinMode(PC7, FUN_INPUT);               // MISO (17)
  funPinMode(PC6, FUN_OUTPUT_MULTIPLEXED);  // MOSI (16)
  funPinMode(PC5, FUN_OUTPUT_MULTIPLEXED);  // SCK (15)
  // Hardware Master or Slave mode:
  //  Float, pull-up or pull-down input
  // Hardware Master mode/NSS output enable mode:
  //  Push-pull multiplexed output 
  if (!handle->nss_pin)
  {
    funPinMode(PC1, FUN_OUTPUT_MULTIPLEXED);  // NSS (11)
    // Enable SS output
    SPI1->CTLR2 = CTLR2_SSOE_Set;
  }
  else
  {
    funPinMode(handle->nss_pin, GPIO_CFGLR_IN_PUPD);
    // SPI_Mode_Master can only be selected when using HARDWARE control
    // or SS is alredy pulled HIGH on SOFTWARE control
    spi_end_transaction(handle);
  }

  // on ESP
  // 1MHz, MSB, MODE0
  // MODE0 == CPHA=0, cpol=0
  uint16_t config = SPI_Mode_Master;
  // full duplex
  config |= SPI_Direction_2Lines_FullDuplex;
  // 24Mhz@48MHz MCU
  // BR -> Configure clock
  config |= SPI_BaudRatePrescaler_16; // 48 / 16 -> 3
  // // SPIMODE(0,0) AKA MODE0: CPOL and CPHA is 0
  // config |= SPI_CPOL_Low | SPI_CPHA_1Edge;
  config |= mode & SPI_MODE_MASK;
  // DEF set to 8bits (value 0)
  config |= handle->regb == 8 ? SPI_DataSize_8b : SPI_DataSize_16b;

  // Frame Format is MSB (value 0)
  // config |= msb_first ? SPI_FirstBit_MSB : SPI_CTLR1_LSBFIRST; 
  config |= SPI_FirstBit_MSB;

  // SSM 1: software control, 0: hardware controle
  config |= handle->nss_pin ? SPI_NSS_Soft : SPI_NSS_Hard;
  // config |= SPI_NSS_Soft;

  // SSI (1-> NSS pin is HIGH, 0 -> NSS pin is LOW on selection)
  // REQUIRED TO BE 1
  config |= SPI_NSSInternalSoft_Set;

  // Enable SPI
  config |= SPI_CTLR1_SPE;


  //Set SPI1, max clock 48Mhz/2 = 24Mhz, master mode, full-duplex mode,8bit data length
	//Internal slave select and software slave managment
	// SPI1->CTLR1 = SPI_CTLR1_SSI | SPI_CTLR1_SSM | SPI_CTLR1_MSTR; //| SPI_CTLR1_BR_1 | SPI_CTLR1_BR_0;
  SPI1->CTLR1 = config;

}
#define SPI_DELAY 100
#if 0
void SPISendBytes(uint8_t *sendData, uint32_t length)
{
  uint32_t loop = 0;
  uint8_t tmp = 0;
  for (loop = 0; loop < length; loop++)
  {
    // Send SPI Byte
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
      ; // wait while flag is zero or TX buffer not empty
    SPI_I2S_SendData(SPI1, sendData[loop]);

    // Receive SPI Byte
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
      ; // wait while flag is zero or RX buffer is empty
    tmp = SPI_I2S_ReceiveData(SPI1);
  }
}

void SPIReceiveBytes(uint8_t *getData, uint32_t length)
{
  uint32_t loop = 0;
  for (loop = 0; loop < length; loop++)
  {
    // Send SPI Byte
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
      ; // wait while flag is zero or TX buffer not empty
    SPI_I2S_SendData(SPI1, 0x00);

    // Receive SPI Byte
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
      ; // wait while flag is zero or RX buffer is empty
    getData[loop] = SPI_I2S_ReceiveData(SPI1);
  }
}
void SPISendReceiveBytes(uint8_t *sendData, uint8_t *getData, uint32_t length)
{
  uint32_t loop = 0;
  for (loop = 0; loop < length; loop++)
  {
    // Send SPI Byte
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
      ; // wait while flag is zero or TX buffer not empty
    SPI_I2S_SendData(SPI1, sendData[loop]);

    // Receive SPI Byte
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
      ; // wait while flag is zero or RX buffer is empty
    getData[loop] = SPI_I2S_ReceiveData(SPI1);
  }
}
#endif

/* 1：Tx buffer empty */
#define SPI_WAIT_FOR_EMPTY_TX() \
  do { } while((SPI1->STATR & SPI_STATR_TXE) != SPI_STATR_TXE)

/* 1：SPI is busy in communication or Tx buffer is not empty */
#define SPI_WAIT_FOR_IDLE() \
	do {} while((SPI1->STATR & SPI_STATR_BSY) == SPI_STATR_BSY)

/* 1：Rx buffer not empty */
#define SPI_WAIT_FOR_DATA() \
	do {} while((SPI1->STATR & SPI_STATR_RXNE) != SPI_STATR_RXNE)


void spi_send8(uint8_t data)
{
  SPI_WAIT_FOR_EMPTY_TX();

	SPI1->DATAR = data;
  SPI_WAIT_FOR_IDLE();

  // not required?
	// while((SPI1->STATR & SPI_STATR_RXNE) != SPI_STATR_RXNE){};

  /* WARNING: discaring result is mandatory */
  data = (uint8_t)SPI1->DATAR;
}

uint8_t spi_recv8(uint8_t dummy)
{
	SPI1->DATAR = dummy;
  SPI_WAIT_FOR_DATA();
	
	return (uint8_t)SPI1->DATAR;
}

static void loop();

static inline uint8_t libdriver_spi_write(uint8_t reg, uint8_t *buf, uint16_t len)
{
  spi_begin_transaction(&spi);

  printf("W: reg=%X, len=%d\n", reg, len);
  spi_send8(reg);
  for (int i = 0; i < len; ++i)
  {
    spi_send8(buf[i]);
  }

  spi_end_transaction(&spi);

  return 0;
}

static inline uint8_t libdriver_spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
  /* NOTE(m): libdriver will is putting 1 at 8th bit, it's ok as BME280 is using
     7bit register
  */
  spi_begin_transaction(&spi);

  printf("R: reg=%X, len=%d\n", reg, len);
  spi_send8(reg);
  printf("Data: ");
  // 2240 0 4 2244 8c4
  for (int i = 0; i < len; ++i)
  {
    buf[i] = spi_recv8(buf[i]);

    printf("%X ", buf[i]);
  }
  printf("\n");

  spi_end_transaction(&spi);

  return 0;
}

uint8_t spi_begin_transaction(spi_device_t *handle)
{
  if (handle->nss_pin)
  {
    funDigitalWrite(handle->nss_pin, FUN_LOW);
  }
  else
  {
    SPI1->CTLR1 |= SPI_CTLR1_SPE;
  }

  return 0;

}

uint8_t spi_end_transaction(spi_device_t *handle)
{
  if (handle->nss_pin)
  {
    funDigitalWrite(handle->nss_pin, FUN_HIGH);
  }
  else
  {
    SPI1->CTLR1 &= ~SPI_CTLR1_SPE;
  }
  
  return 0;
}

uint8_t spi_send_receive(const uint8_t *data, const uint8_t data_len, uint8_t *recv, const uint8_t recv_len)
{
    // spi_send8(0xD0);

    // // wait for data?
    // printf("Wait for data receive\n");
    // while (!(SPI1->STATR & SPI_STATR_RXNE))
    // {
    //   __NOP();
    // }
    // // Delay_Ms(200);
    // printf("X: %X\n", SPI1->DATAR);

    // // Send dommy data and wait for response of 1 byte
    // spi_send8(0x00);
    // printf("Wait for data receive (real)\n");
    // while (!(SPI1->STATR & SPI_STATR_RXNE))
    // {
    //   __NOP();
    // }
    // // Delay_Ms(200);
    // printf("R: %X\n", SPI1->DATAR);

  spi_send8(data[0]);
  // printf("1: %X\n", SPI1->DATAR);

    // while (!(SPI1->STATR & SPI_STATR_RXNE))
    // {
    //   __NOP();
    // }
    // WARNING: You have to read data or it will read correct value
    // after 2nd send_receive invocation
  // printf("2: %X\n", SPI1->DATAR);

  recv[0] = spi_recv8(0x00);

  return 0;
}

void spi_fun(void)
{
  #if 0
  // SS: 1
  // begin()
  // SS: 0
  // write(0xD0)
  // transfer(0xFF) // aka read
  // SS: 1
  // end()
  // SS: 1

  printf("0: SPI STATR: %X\n", SPI1->STATR);
  // TODO ? Software mode is not working as expected and no data is received
  // from sensor. Hardware works OK, but requires pin PC1 or PC0 (alternate)
  spi_init(&spi, 0, SPI_MODE0, 8, 1);
  printf("1: SPI STATR: %X\n", SPI1->STATR);
  // Enable SPI
  // SPI1->CTLR1 |= SPI_CTLR1_SPE;
  // printf("2: SPI STATR: %X\n", SPI1->STATR);
  // // 
  // // Enable SPI
  // while(1) {
  //   printf("ENABLE\n");
  //   SPI1->CTLR1 |= SPI_CTLR1_SPE;
  //   Delay_Ms(4000);
  //   printf("DISABLE\n");
  //   SPI1->CTLR1 &= ~SPI_CTLR1_SPE;
  //   Delay_Ms(4000);
  // }

  Delay_Ms(1000);

  // spi_recv8(0xFF);
  volatile uint16_t dummy;
  while(1)
  {

    #if 1
    // SPI1->CTLR1 |= SPI_CTLR1_SPE;
    // printf("START: SPI STATR: %X\n", SPI1->STATR);
    // Delay_Ms(100);
    // ensure nothing is pending?
    // printf("Wait for Transmit buffer empty\n");
    // while(!(SPI1->STATR & SPI_STATR_TXE))
    // {
    //   __NOP();
    // }

    // SPI1->DATAR = 0xD0;

    // // Delay_Ms(100);
    // // wait for completion
    // printf("Wait for transfer completion\n");
    // while(!(SPI1->STATR & SPI_STATR_TXE))
    // {
    //   __NOP();
    // }
    // // SPI1->CTLR1 &= ~SPI_CTLR1_SPE;
    // // SPI1->CTLR1 |= SPI_CTLR1_SPE;

    // // write dummy data and then read response for first command
    // while((SPI1->STATR & SPI_STATR_BSY) == SPI_STATR_BSY){};

    //begin transaction

    #if 1
    uint8_t resp = 0xFF;
    libdriver_spi_read(0xD0, &resp, 1);
    printf("R: %X\n", resp);
    #elif 1
    uint8_t reg = 0xD0;
    spi_send_receive(&reg, 1, &reg, 1);
    printf("R: %X\n", reg);
    #else

    spi_begin_transaction(&spi);

    spi_send8(0xD0);

    // wait for data?
    printf("Wait for data receive\n");
    while (!(SPI1->STATR & SPI_STATR_RXNE))
    {
      __NOP();
    }
    // Delay_Ms(200);
    printf("X: %X\n", SPI1->DATAR);

    // Send dommy data and wait for response of 1 byte
    spi_send8(0x00);
    printf("Wait for data receive (real)\n");
    while (!(SPI1->STATR & SPI_STATR_RXNE))
    {
      __NOP();
    }
    // Delay_Ms(200);
    printf("R: %X\n", SPI1->DATAR);
    spi_end_transaction(&spi);
    #endif


    #else

    // spi_send8(0xD0);
    printf("R: %X\n", spi_recv8(0xD0));
    // SPI1->CTLR1 &= ~SPI_CTLR1_SPE;
    // printf("STOP: SPI STATR: %X\n", SPI1->STATR);
    #endif

    // end transaction

    Delay_Ms(4000);
  }
#endif

  static bme280_handle_t bme;
  DRIVER_BME280_LINK_INIT(&bme, bme280_handle_t);
  DRIVER_BME280_LINK_DEBUG_PRINT(&bme, debug_print);
  DRIVER_BME280_LINK_DELAY_MS(&bme, libdriver_delay_ms);
  DRIVER_BME280_LINK_SPI_DEINIT(&bme, libdriver_nop_void);
  DRIVER_BME280_LINK_SPI_INIT(&bme, libdriver_nop_void);
  DRIVER_BME280_LINK_SPI_READ(&bme, libdriver_spi_read);
  DRIVER_BME280_LINK_SPI_WRITE(&bme, libdriver_spi_write);
  // not used but required
  DRIVER_BME280_LINK_IIC_DEINIT(&bme, libdriver_nop_void);
  DRIVER_BME280_LINK_IIC_INIT(&bme, libdriver_nop_void);
  DRIVER_BME280_LINK_IIC_READ(&bme, libdriver_iic_addr_read);
  DRIVER_BME280_LINK_IIC_WRITE(&bme, libdriver_iic_addr_write);

  bme280_set_interface(&bme, BME280_INTERFACE_SPI);

  spi_init(&spi, 0, SPI_MODE0, 8, 1);

  printf("I = %d\n", bme.inited);

  while(1)
  {
    if (!bme.inited)
    {
      if (bme280_init(&bme))
      {
        printf("F\n");
      }
      else
      {
        printf("I\n");
        bme280_set_mode(&bme, BME280_MODE_FORCED);
        bme280_set_filter(&bme, BME280_FILTER_OFF);
        bme280_set_humidity_oversampling(&bme, BME280_OVERSAMPLING_x1);
        bme280_set_temperatue_oversampling(&bme, BME280_OVERSAMPLING_x1);
        bme280_set_pressure_oversampling(&bme, BME280_OVERSAMPLING_x1);
      }
    }
    else
    {
      printf("Already done\n");
      uint32_t tr, pr, hr;
      float t, p, h;

      bme280_read_temperature_pressure_humidity(&bme, &tr, &t, &pr, &p, &hr, &h);
      printf("T: %d P: %d H: %d\n", (int)t, (int)p, (int)h);
    }

    Delay_Ms(1000);
  }
}

static inline uint8_t tca9458a_iic_setup()
{
  tca9548a_address_t addr;
  if (tca9548a_get_addr_pin(&tca9548a, &addr))
  {
    return 1;
  }

  i2c.addr = (uint8_t)addr;

  return 0;
}

static inline uint8_t tca9548a_iic_write(uint8_t *buf, uint16_t len)
{
  uint8_t err;
  if (err = tca9458a_iic_setup())
  {
    printf("iic_setup: fialed: %d\n", err);
    return 1;
  }

  if (err = i2c_write_raw(&i2c, buf, len))
  {
    printf("iic_write_raw: fialed: %d\n", err);
  }
  return err;
}

static inline uint8_t tca9548a_iic_read(uint8_t *buf, uint16_t len)
{
  uint8_t err;
  if (err = tca9458a_iic_setup())
  {
    printf("iic_setup: fialed: %d\n", err);
    return 1;
  }

  if (err = i2c_read_raw(&i2c, buf, len))
  {
    printf("iic_read_raw: fialed: %d\n", err);
  }

  return err;
}

#define DEBUG_DATA0_ADDRESS ((volatile uint32_t *)0xE00000F4)

typedef enum
{
  SENSOR_AHT = 1 << 0,
  SENSOR_BMP280 = 1 << 1,
  SENSOR_BME280 = 1 << 2,
  SENSOR_SHT3X = 1 << 3,
  SENSOR_SHT4X = 1 << 4,
  SENSOR_SHTC3 = 1 << 5,
} sensor_t;

void i2c_scan_callback(const uint8_t addr)
{
  printf("Address: 0x%02X Responded.\n", addr);
}

struct bank_t
{
  tca9548a_channel_t channel;
  // If sensor is present then it was inited before, otherwise initialization
  // is required before acquiring any data
  sensor_t sensors;
};

struct bank_t banks[4] = {
    {TCA9548A_CHANNEL_5, 0},
    {TCA9548A_CHANNEL_4, 0},
    {TCA9548A_CHANNEL_3, 0},
    {TCA9548A_CHANNEL_2, 0},
};

struct xxx_t
{
  uint8_t address;
  uint8_t (*init)();
  uint8_t (*deinit)();
  uint8_t (*fetch)(int *, int *, int *);
};

static void initializeGPIO()
{
  GPIO_port_enable(GPIO_port_C);
  GPIO_pinMode(GPIOv_from_PORT_PIN(GPIO_port_C, GPIO_PinSource4), GPIO_pinMode_O_openDrain, GPIO_Speed_2MHz);

  // Default state for reset pin is HIGH or HI-Z
  GPIO_digitalWrite_1(GPIOv_from_PORT_PIN(GPIO_port_C, GPIO_PinSource4));
}

static void tca9548a_reset(tca9548a_handle_t *handle)
{
  // pull down, and return to default mode for reset cycle
  GPIO_digitalWrite_0(GPIOv_from_PORT_PIN(GPIO_port_C, GPIO_PinSource4));
  Delay_Us(10);
  GPIO_digitalWrite_1(GPIOv_from_PORT_PIN(GPIO_port_C, GPIO_PinSource4));

  tca9548a_channel_set(handle, TCA9548A_CHANNEL_NONE);
}


static void check_bank(struct bank_t *bank)
{
  printf("Check bank: %d\n", bank->channel);

  if (tca9548a_channel_set(&tca9548a, bank->channel))
  {
    printf("channel failed");
    return;
  }

  // ping known addresses?
  sensor_t new_config = 0;

  if (i2c_ping(BMP280_ADDRESS) == I2C_OK)
  {
    new_config |= SENSOR_BMP280;
  }

  if (i2c_ping(AHTXX_ADDRESS) == I2C_OK)
  {
    // copy config
    new_config |= SENSOR_AHT;
  }

  if (i2c_ping(SHTXX_ADDRESS) == I2C_OK)
  {
    /* only is possible, but unknown at this point */
    new_config |= SENSOR_SHT3X | SENSOR_SHT4X;
  }

  if (i2c_ping(SHTC3_ADDRESS) == I2C_OK)
  {
    new_config |= SENSOR_SHTC3;
  }

  sensor_t sensor_diff = new_config ^ bank->sensors;

  // Sensor changed it state and is currently active
  sensor_t now_active = sensor_diff & new_config;

  printf("%X %X %X %X\n", new_config, bank->sensors, sensor_diff, now_active);

  // Changed and is now active
  if (now_active & SENSOR_BMP280)
  {
    // this is tricky and smelly as first you need to deinit it (due to shared struct)
    bmp280_iic_init();
    bmp280_deinit(&bmp280);

    if (setupBMP280())
    {
      printf("Bummer no BMP280\n");
      new_config &= ~SENSOR_BMP280;
    }
    else
    {
      printf("BMP280 activated\n");
    }
  }

  if (now_active & SENSOR_AHT)
  {
    // there is only one 3x or 2x
    aht30_iic_init();
    aht30_deinit(&aht30);

    if (aht30_init(&aht30))
    {
      printf("No AHT30\n");
      new_config &= ~SENSOR_AHT;
    }
    else
    {
      printf("Found AHT30\n");
    }
  }

  bank->sensors = new_config;
}

/* MUX configuration is out of this scope */

// DMA transfer completion interrupt. It will fire when the DMA transfer is
// complete. We use it just to blink the LED
__attribute__((interrupt)) __attribute__((section(".srodata")))
void DMA1_Channel4_IRQHandler(void)
{
	// Clear flag
	DMA1->INTFCR |= DMA_CTCIF4;
}


#define RX_BUF_LEN 16 // size of receive circular buffer

u8 rx_buf[RX_BUF_LEN] = {0}; // DMA receive buffer for incoming data
u8 cmd_buf[RX_BUF_LEN] = {0}; // buffer for complete command strings

void uart_fun()
{
  funGpioInitC();
	funPinMode( PC7, GPIO_CFGLR_OUT_2Mhz_PP);
	
	// enable rx pin
	USART1->CTLR1 |= USART_CTLR1_RE;

	// enable usart's dma rx requests
	USART1->CTLR3 |= USART_CTLR3_DMAR;

	// enable dma clock
	RCC->AHBPCENR |= RCC_DMA1EN;

	// configure dma for UART reception, it should fire on RXNE
	DMA1_Channel5->MADDR = (u32)&rx_buf;
	DMA1_Channel5->PADDR = (u32)&USART1->DATAR;
	DMA1_Channel5->CNTR = RX_BUF_LEN;

	// MEM2MEM: 0 (memory to peripheral)
	// PL: 0 (low priority since UART is a relatively slow peripheral)
	// MSIZE/PSIZE: 0 (8-bit)
	// MINC: 1 (increase memory address)
	// PINC: 0 (peripheral address remains unchanged)
	// CIRC: 1 (circular)
	// DIR: 0 (read from peripheral)
	// TEIE: 0 (no tx error interrupt)
	// HTIE: 0 (no half tx interrupt)
	// TCIE: 0 (no transmission complete interrupt)
	// EN: 1 (enable DMA)
	DMA1_Channel5->CFGR = DMA_CFGR1_CIRC | DMA_CFGR1_MINC | DMA_CFGR1_EN;

  while(1)
	{
		static u32 tail = 0; // current read position in rx_buf
		static u32 cmd_end = 0; // end index of current command in rx_buf
		static u32 cmd_st = 0; // start index of current command in rx_buf

		// calculate head position based on DMA counter (modulo when DMA1_Channel5->CNTR = 0)
		u32 head = (RX_BUF_LEN - DMA1_Channel5->CNTR) % RX_BUF_LEN; // current write position in rx_buf
    
    if (head) {
      printf("HEAD: %lu\n", head);
    }
		
		// process new bytes in rx_buf. when a newline character is detected, the command is copied to cmd_buf
		while (tail != head)
		{
			if ( rx_buf[tail] == '\n' ) 
			{
        printf("%s\n", rx_buf);
				cmd_end = tail;
				u32 cmd_i = 0; // carret position in cmd_buf
				if (cmd_end > cmd_st)
				{
					for (u32 rx_i = cmd_st; rx_i < cmd_end + 1; rx_i++, cmd_i++) {
						cmd_buf[cmd_i] = rx_buf[rx_i];
					}
				} else if (cmd_st > cmd_end) { // handle wrap around
					for (u32 rx_i = cmd_st; rx_i < RX_BUF_LEN; rx_i++, cmd_i++) {
						cmd_buf[cmd_i] = rx_buf[rx_i];
					}
					for (u32 rx_i = 0; rx_i < cmd_end + 1; rx_i++, cmd_i++) {
						cmd_buf[cmd_i] = rx_buf[rx_i];
					}
				}

				// null terminate
				cmd_buf[cmd_i] = '\0';

        printf("'%s'\n", cmd_buf);

				// update start position for next command
				cmd_st = (cmd_end + 1) % RX_BUF_LEN;
			}

			// move to next position 
			tail = (tail+1) % RX_BUF_LEN;
		}
	}
}

#define UART_BR 9600
static void uart_setup(void)
{
	// Enable UART and GPIOD
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1;

	// Push-Pull, 10MHz Output on D5, with AutoFunction
	GPIOD->CFGLR = (GPIOD->CFGLR & ~(0xF<<(4*5))) |
			((GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*5));

	// Setup UART for Tx 8n1
	USART1->CTLR1 = USART_WordLength_8b | USART_Parity_No | USART_Mode_Tx | USART_Mode_Rx;
	USART1->CTLR2 = USART_StopBits_1;
	// Enable Tx DMA event
	USART1->CTLR3 = USART_DMAReq_Tx;

	// Set baud rate and enable UART
	USART1->BRR = ((FUNCONF_SYSTEM_CORE_CLOCK) + (UART_BR)/2) / (UART_BR);
	USART1->CTLR1 |= CTLR1_UE_Set;
}
static void dma_uart_setup(void)
{
	// Enable DMA peripheral
	RCC->AHBPCENR = RCC_AHBPeriph_SRAM | RCC_AHBPeriph_DMA1;

	// Disable channel just in case there is a transfer in progress
	DMA1_Channel4->CFGR &= ~DMA_CFGR1_EN;

	// USART1 TX uses DMA channel 4
	DMA1_Channel4->PADDR = (uint32_t)&USART1->DATAR;
	// MEM2MEM: 0 (memory to peripheral)
	// PL: 0 (low priority since UART is a relatively slow peripheral)
	// MSIZE/PSIZE: 0 (8-bit)
	// MINC: 1 (increase memory address)
	// CIRC: 0 (one shot)
	// DIR: 1 (read from memory)
	// TEIE: 0 (no tx error interrupt)
	// HTIE: 0 (no half tx interrupt)
	// TCIE: 1 (transmission complete interrupt enable)
	// EN: 0 (do not enable DMA yet)
	DMA1_Channel4->CFGR = DMA_CFGR1_MINC | DMA_CFGR1_DIR | DMA_CFGR1_TCIE;

	// Enable channel 4 interrupts
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

static void dma_uart_tx(const void *data, uint32_t len)
{
	// Disable DMA channel (just in case a transfer is pending)
	DMA1_Channel4->CFGR &= ~DMA_CFGR1_EN;
	// Set transfer length and source address
	DMA1_Channel4->CNTR = len;
	DMA1_Channel4->MADDR = (uint32_t)data;
	// Enable DMA channel to start the transfer
	DMA1_Channel4->CFGR |= DMA_CFGR1_EN;
}

void asdf() {
  static const char message[] = "Hello World!\r\n";
  uart_setup();
	dma_uart_setup();

	while (1)
	{
		// dma_uart_tx(message, sizeof(message) - 1);
    // printf(USART_Read)
    uart_fun();
		Delay_Ms(1000);
	}

}

int main()
{
  SystemInit();

  // uart_fun();
  // asdf();

  spi_fun();


  initializeGPIO();

  // SetupDebugPrintf();

  // *(DEBUG_DATA0_ADDRESS) = 0;
  // PD1
  // 1) Enable GPIOD
  // PIN1 -> IPU
  // while(1) {
  // printf("ASDF\n");
  // Delay_Ms(1000);
  // }


  init_ahtxx();
  init_bmp280();
  init_sht3x();
  init_sht4x();
  init_shtc3();

  DRIVER_MAX31865_LINK_INIT(&max31865, max31865_handle_t);
  DRIVER_MAX31865_LINK_DEBUG_PRINT(&max31865, NULL);
  DRIVER_MAX31865_LINK_DELAY_MS(&max31865, libdriver_delay_ms);
  DRIVER_MAX31865_LINK_INIT(&max31865, libdriver_nop_void);
  DRIVER_MAX31865_LINK_SPI_DEINIT(&max31865, libdriver_nop_void);
  DRIVER_MAX31865_LINK_SPI_READ(&max31865, libdriver_spi_read);
  DRIVER_MAX31865_LINK_SPI_WRITE(&max31865, libdriver_spi_write);


  DRIVER_TCA9548A_LINK_INIT(&tca9548a);
  DRIVER_TCA9548A_LINK_DELAY_MS(&tca9548a, libdriver_delay_ms);
  DRIVER_TCA9548A_LINK_IIC_READ(&tca9548a, tca9548a_iic_read);
  DRIVER_TCA9548A_LINK_IIC_WRITE(&tca9548a, tca9548a_iic_write);
  tca9548a_set_addr_pin(&tca9548a, TCA9548A_ADDRESS_A0);

  i2c_err_t err = i2c_init(&i2c);
  if (err)
  {
    printf("Error in init: %d\n", err);
  }

  Delay_Ms(250);

  // printf("Let's the scan begin\n");
  // i2c_scan(i2c_scan_callback);
  // printf("Scan done\n");

  // It's OPEN DRAIN mode, so by default it uses TCA5498A board resistor
  // to keep reset pin high
  Delay_Ms(2000);

  while (1)
  {
    err = tca9548a_init(&tca9548a);
    if (!err)
    {
      printf("mux OK\n");
      break;
    }

    printf("mux failed: %d\n", err);

    tca9548a_reset(&tca9548a);
    Delay_Ms(1000);
  }

  if (tca9548a_channel_set(&tca9548a, banks[2].channel) == 0)
  {
    while (1) {
      printf("blah\n");
      // sht4x and sht3x has different protocol so when communication is
      // impossible for 4x try 3x and then call it a day

      // setupSHT4X();
      // setupSHT3X();
      setupSHTC3();
      // i2c_scan(i2c_scan_callback);
       Delay_Ms(2000);
    }
  }

  // Mux is using channels from 2..5
  // 5 -> 1st bank
  // 4 -> 2nd bank
  // 3 -> 3rd bank
  // 2 -> 4th bank
  // PC4 is reset pin

#if 0

  aht20_iic_init();
  if (aht20_init(&aht20))
  {
    printf("aht20 failed\n");
  }

  aht30_iic_init();
  if (aht30_init(&aht30))
  {
    printf("aht30 failed\n");
  }
#endif

  // tca9548a_init(&tca9548a);

  /* safety, if someone will call deepsleep */
  Delay_Ms(2000);

  while (1)
  {
    loop();
  }

  return 1;
}

void loop()
{
  printf("LOOP\n");
  for (unsigned b = 0; b < sizeof(banks)/sizeof(banks[0]); ++b)
  {
    printf("Set bank: %d@%d\n", b, banks[b].channel);
    if (tca9548a_channel_set(&tca9548a, banks[b].channel))
    {
      printf("failed to enable channel: %d\n", banks[b].channel);
      continue;
    }

    i2c_scan(i2c_scan_callback);
  }

  tca9548a_reset(&tca9548a);
  Delay_Ms(2000);

  return;

  for (unsigned i = 0; i < sizeof(banks) / sizeof(banks[0]); ++i)
  {
    struct bank_t *b = &banks[i];

    // it will set correct mux configuration
    printf("Begin bank: %d\n", i);
    check_bank(b);

    if (b->sensors & SENSOR_BMP280)
    {
      // bmp280 cannot be shared with all sensor, there is internal state
      bmp280_iic_init();

      uint32_t t_raw, p_raw;
      bmp280_temperature_t ti;
      bmp280_pressure_t pi;
      if (!bmp280_read_temperature_pressure(&bmp280, &t_raw, &ti, &p_raw, &pi))
      {
        printf("BMP280: T=%lu.%lu P=%lu\n", ti / 100, ti % 100, pi / 256);
      } else {
        printf("BMP280: FIALED\n");
      }

      // if (h_raw != 0x80000)
      // {
      //   // Q24 -> 8 fractional
      //   p = (float)(uint32_t)(pi) / (float)(1 << 8);
      // }
      // else
      // {
      //   p = -1.0f;
      // }

      // if (t_raw != 0x80000)
      // {
      //   t = (float)(ti * 0.01f);
      // }
      // else
      // {
      //   t = -280.0;
      // }

    }

    // NOTE(m): AHT20 and AHT30 uses same code
    // just use AHT30 for everything

    if (b->sensors & SENSOR_AHT)
    {
      uint32_t t_raw, h_raw;
      float t;
      uint8_t h;

      aht30_iic_init();
      // We know there is no per-chip values required for reading data
      // just ensure struct is marked as 'inited' before trying to read
      // from sensor
      aht30.inited = 1;
      if (aht30_read_temperature_humidity(&aht30, &t_raw, &t, &h_raw, &h))
      {
        printf("AHT: FIALED\n");
      }
      else
      {
        printf("AHT: H=%d\n", h);
      }
    }
  }
  // printf("Let's the scan begin\n");
  // i2c_scan(i2c_scan_callback);
  // printf("Scan done\n");

  // Delay_Ms(5000);
  // return;

  Delay_Ms(6000);
}