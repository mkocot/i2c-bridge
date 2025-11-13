#include <stdio.h>
#include <ch32fun.h>
#include <ch32v003_GPIO_branchless.h>

#include <lib_i2c.h>
#include <driver_aht20.h>
#include <driver_aht30.h>
#include <driver_sht35.h>
#include <driver_sht4x.h>

#include <driver_bmp280.h>
// #include <driver_bme280.h>
#include <driver_max31865.h>
#include <driver_tca9548a.h>
#include <driver_shtc3.h>

union any_sensor_u {
  aht30_handle_t aht;
  bmp280_handle_t bmp280;
};

typedef union any_sensor_u any_sensor_t;

static aht30_handle_t aht30;
static sht35_handle_t sht35;
static sht4x_handle_t sht4x;
static shtc3_handle_t shtc3;
static bmp280_handle_t bmp280;
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
  uint8_t regb;
};

typedef struct spi_device_s spi_device_t;

static void spi_init(spi_device_t *handle)
{
  // return;

  // if (handle->regb > 4)
  // {
  //   handle->regb = 4;
  // }
  // if (handle->regb == 0)
  // {
  //   handle->regb = 1;
  // }
#if 0
	// Toggle the I2C Reset bit to init Registers
	RCC->APB2PRSTR |=  RCC_APB2Periph_SPI1;
	RCC->APB2PRSTR &= ~RCC_APB2Periph_SPI1;

	// Enable the I2C Peripheral Clock
	RCC->APB2PCENR |= RCC_APB2Periph_SPI1;

	// Enable the selected I2C Port, and the Alternate Function enable bit
	RCC->APB2PCENR |= SPI_PORT_RCC | RCC_APB2Periph_AFIO;

	// Reset the AFIO_PCFR1 register, then set it up
	AFIO->PCFR1 &= ~(0x04400002);
	AFIO->PCFR1 |= I2C_AFIO_REG;

    GPIO_InitTypeDef GPIO_InitStructure={0};
    SPI_InitTypeDef SPI_InitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init( GPIOC, &GPIO_InitStructure );

#if (SPI_MODE == HOST_MODE)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

#elif (SPI_MODE == SLAVE_MODE)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

#endif

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;

#if (SPI_MODE == HOST_MODE)
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

#elif (SPI_MODE == SLAVE_MODE)
    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;

#endif

    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init( SPI1, &SPI_InitStructure );

    SPI_Cmd( SPI1, ENABLE );
#endif
}
#define SPI_DELAY 100

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

i2c_device_t i2c = {
    .clkr = I2C_CLK_400KHZ, /* "default" */
    .type = I2C_ADDR_7BIT,  /* common addr type */
    .addr = 0x00,           /* device addres */
    .regb = 1,              /* register bytes, most devices uses 1 byte for register */
    .tout = 2000            /* cycles ?*/
};

static void loop();

static inline void libdriver_delay_ms(uint32_t delay) { Delay_Ms(delay); }
static inline uint8_t libdriver_iic_write(uint8_t reg, uint8_t *buf, uint16_t len)
{
  i2c.regb = 1;
  return i2c_write_reg(&i2c, reg, buf, len);
}
static inline uint8_t libdriver_iic_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
  i2c.regb = 1;
  return i2c_read_reg(&i2c, reg, buf, len);
}

static inline uint8_t libdriver_nop_void(void) { return 0; }

#define AHT20_ADDRESS (0x38)
#define AHT30_ADDRESS (0x38)
#define BMP280_ADDRESS (0x77)

static inline uint8_t aht20_iic_init()
{
  // i2c.regb = 1;
  i2c.addr = AHT20_ADDRESS;

  return 0;
}

static inline uint8_t aht20_iic_deinit()
{
  return 0;
}

static inline uint8_t aht30_iic_init()
{
  i2c.addr = AHT30_ADDRESS;
  // i2c.regb = 1;

  return 0;
}
static inline uint8_t bmp280_iic_init()
{
  i2c.addr = BMP280_ADDRESS;
  // i2c.regb = 1;

  return 0;
}

static inline uint8_t sht3x_iic_init()
{
  /* library address is shifted leaving direction as 0 we need raw address */
  i2c.addr = SHT35_ADDRESS_0 >> 1;
  // i2c.regb = 2;

  return 0;
}

static inline uint8_t sht4x_iic_init()
{
  i2c.addr = SHT4X_ADDRESS_0 >> 1;
  // i2c.regb = 1;

  return 0;
}

static inline uint8_t shtc3_iic_init()
{
  // i2c.regb = 2;

  return 0;
}

static inline uint8_t libdriver_iic_addr_read_noreg(uint8_t addr, uint8_t *buf, uint16_t len)
{
  i2c.addr = addr >> 1;

  return i2c_read_raw(&i2c, buf, len);
}

static inline uint8_t libdriver_iic_addr_write_noreg(uint8_t addr, uint8_t *buf, uint16_t len)
{
  i2c.addr = addr >> 1;

  return i2c_write_raw(&i2c, buf, len);
}

// Somehow this function is special as compared to aht...
static inline uint8_t libdriver_iic_addr_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  i2c.addr = addr >> 1;
  return libdriver_iic_read(reg, buf, len);
}

static inline uint8_t libdriver_iic_addr_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  i2c.addr = addr >> 1;
  return libdriver_iic_write(reg, buf, len);
}

static inline uint8_t libdriver_iic_addr16_read(uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t len)
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

static inline uint8_t libdriver_iic_addr16_write(uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t len)
{
  i2c.addr = addr >> 1;
  i2c.regb = 2;

  uint8_t ret = i2c_write_reg(&i2c, reg, buf, len);
  printf("A16W: %X, %X, %u %u =%u\n", i2c.addr, reg, i2c.regb, len, ret);
  return ret;
}

static inline uint8_t libdriver_spi_write(uint8_t reg, uint8_t *buf, uint16_t len)
{
  return 0;
}
static inline uint8_t libdriver_spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
  return 0;
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

static uint8_t setupBMP280();

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

  if (i2c_ping(AHT20_ADDRESS) == I2C_OK)
  {
    // copy config
    new_config |= SENSOR_AHT;
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
static uint8_t setupBMP280()
{
  bmp280_iic_init();
  if (bmp280_init(&bmp280))
  {
    printf("bmp280 failed\n");
    return 1;
  }

  if (bmp280_set_mode(&bmp280, BMP280_MODE_FORCED))
  {
    printf("F1\n");
    return 2;
  }

  if (bmp280_set_filter(&bmp280, BMP280_FILTER_OFF))
  {
    printf("F2\n");
    return 3;
  }

  // Thes both method ar botched, you cannot set pressure and temperature
  // only first one is selected and 2nd ignored
  // if (bmp280_set_pressure_temperature_oversampling(&bmp280, BMP280_OVERSAMPLING_x1, BMP280_OVERSAMPLING_x1))
  // {
  //   printf("F3\n");
  // }

  if (bmp280_set_pressure_oversampling(&bmp280, BMP280_OVERSAMPLING_x1))
  {
    printf("F3\n");
    return 4;
  }

  Delay_Ms(1);

  if (bmp280_set_temperatue_oversampling(&bmp280, BMP280_OVERSAMPLING_x1))
  {
    printf("F4\n");
    return 5;
  }

  return 0;
}

static uint8_t setupSHT4X()
{
  uint8_t ret;
  uint16_t t_raw, h_raw;
  float t, h;

  sht4x_iic_init();
  if (!sht4x.inited)
  {
    if ((ret = sht4x_init(&sht4x)))
    {
      printf("SHT4X: init failed: %d\n", ret);
      return 1;
    }
  }

  if (sht4x_read(&sht4x, SHT4X_MODE_HIGH_PRECISION_WITH_NO_HEATER, &t_raw, &t, &h_raw, &h))
  {
    printf("SHT4X: read failed\n");
  }
  else
  {
    printf("SHT4X: reading OK\n");
    printf("SHT4X:");
    if (t_raw)
    {
      int dec = (t - (int)t) * 100;
      printf(" T=%d.%d", (int)(t), dec);
    }
    if (h_raw)
    {
      printf(" H=%d", (int)(h));
    }
    printf("\n");
  }

  return 0;
}

static uint8_t setupSHT3X()
{
  uint16_t t_raw, h_raw;
  float t, h;

  sht3x_iic_init();
  if (!sht35.inited)
  {
    if (sht35_init(&sht35))
    {
      printf("SHT35: init failed\n");
      return 1;
    }

    if (sht35_set_heater(&sht35, SHT35_BOOL_FALSE))
    {
      printf("SHT35: heaters gonna heat\n");
    }

    if (sht35_set_repeatability(&sht35, SHT35_REPEATABILITY_HIGH))
    {
      printf("SHT35: repatability failed\n");
    }
  }

  uint16_t status;
  if (sht35_get_status(&sht35, &status))
  {
    printf("SHT35: reading status failed\n");
  }
  else
  {
    printf("SHT35: status %X\n", status);
  }


  // Without stretching we only get temp + crc and garbage humidity
  // with clock stretch there is no data at all
  if (sht35_single_read(&sht35, SHT35_BOOL_TRUE, &t_raw, &t, &h_raw, &h))
  {
    printf("SHT35: reading fialed\n");
  }
  else
  {
    printf("SHT35: reading OK\n");
    printf("SHT35:");
    if (t_raw)
    {
      int dec = (t - (int)t) * 100;
      printf(" T=%d.%d", (int)(t), dec);
    }
    if (h_raw)
    {
      printf(" H=%d", (int)(h));
    }
    printf("\n");
  }

  return 0;
}

static uint8_t setupSHTC3()
{
  uint16_t t_raw, h_raw;
  float t, h;

  shtc3_iic_init();
  if (!shtc3.inited)
  {
    if (shtc3_init(&shtc3))
    {
      printf("SHTC3: init fail\n");

      return 1;
    }
  }


  if (shtc3_read(&shtc3, SHTC3_BOOL_TRUE, &t_raw, &t, &h_raw, &h))
  {
    printf("SHTC3: read failed\n");
  }
  else
  {
    printf("SHTC3: reading OK\n");
    printf("SHTC3:");
    if (t_raw)
    {
      int dec = (t - (int)t) * 100;
      printf(" T=%d.%d", (int)(t), dec);
    }
    if (h_raw)
    {
      printf(" H=%d", (int)(h));
    }
    printf("\n");
  }

  return 0;
}

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

static void debug_print(const char *const fmt, ...)
{
  printf(fmt);
}

int main()
{
  SystemInit();

  // uart_fun();
  // asdf();


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

  DRIVER_SET_DEFAULT_IIC(AHT30, &aht30, aht30_handle_t);

  DRIVER_SET_DEFAULT_IIC_ADDR(BMP280, &bmp280, bmp280_handle_t);
  bmp280_set_interface(&bmp280, BMP280_INTERFACE_IIC);
  bmp280_set_addr_pin(&bmp280, 0x77);

  DRIVER_MAX31865_LINK_INIT(&max31865, max31865_handle_t);
  DRIVER_MAX31865_LINK_DEBUG_PRINT(&max31865, NULL);
  DRIVER_MAX31865_LINK_DELAY_MS(&max31865, libdriver_delay_ms);
  DRIVER_MAX31865_LINK_INIT(&max31865, libdriver_nop_void);
  DRIVER_MAX31865_LINK_SPI_DEINIT(&max31865, libdriver_nop_void);
  DRIVER_MAX31865_LINK_SPI_READ(&max31865, libdriver_spi_read);
  DRIVER_MAX31865_LINK_SPI_WRITE(&max31865, libdriver_spi_write);

  DRIVER_SET_DEFAULT_IIC_ADDR16(SHT35, &sht35, sht35_handle_t);
  // DRIVER_SHT35_LINK_INIT(&sht35, sht35_handle_t);
  // DRIVER_SHT35_LINK_DEBUG_PRINT(&sht35, debug_print);
  // DRIVER_SHT35_LINK_DELAY_MS(&sht35, libdriver_delay_ms);
  // DRIVER_SHT35_LINK_IIC_INIT(&sht35, libdriver_nop_void);
  // DRIVER_SHT35_LINK_IIC_DEINIT(&sht35, libdriver_nop_void);
  // DRIVER_SHT35_LINK_IIC_READ_ADDRESS16(&sht35, libdriver_iic_addr16_read);
  // DRIVER_SHT35_LINK_IIC_WRITE_ADDRESS16(&sht35, libdriver_iic_addr16_write);
  sht35_set_addr_pin(&sht35, SHT35_ADDRESS_0);


  // WHY THERE IS SUCH DISCREPARENCY BETWEEN MODULES
  // READ_COMMAND vs READ_CMD
  // WRITE_COMMAND vs WRITE_CMD
  // somethimes there is addres sometimes it's not...
  DRIVER_SHT4X_LINK_INIT(&sht4x, sht4x_handle_t);
  DRIVER_SHT4X_LINK_DEBUG_PRINT(&sht4x, debug_print);
  DRIVER_SHT4X_LINK_DELAY_MS(&sht4x, libdriver_delay_ms);
  DRIVER_SHT4X_LINK_IIC_INIT(&sht4x, libdriver_nop_void);
  DRIVER_SHT4X_LINK_IIC_DEINIT(&sht4x, libdriver_nop_void);
  DRIVER_SHT4X_LINK_IIC_READ_COMMAND(&sht4x, libdriver_iic_addr_read_noreg);
  DRIVER_SHT4X_LINK_IIC_WRITE_COMMAND(&sht4x, libdriver_iic_addr_write_noreg);
  sht4x_set_addr(&sht4x, SHT4X_ADDRESS_0);


  DRIVER_SET_DEFAULT_IIC_ADDR16(SHTC3, &shtc3, shtc3_handle_t);

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
    // black bloke responds with address 0x00 and it's not helpful at all...
    // what kind of sensor it is?!
    // purple without name responds with 0x00 and 0x44 is it's kind of
    // SHT family
    while (1) {
      // shtc3 default address is 0x70 so is mux... FFS
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