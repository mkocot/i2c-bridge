#include "ahtxx.h"
#include "bank.h"
#include "bmp280.h"
#include "common_driver.h"
#include "hdc1080.h"
#include "packet.h"
#include "shtxx.h"
#include "spi.h"
#include "si7021.h"
#include "htuxx.h"
#include "mcp9808.h"

#include <alloca.h>
#include <stdio.h>
#include <ch32fun.h>

#include <lib_i2c.h>
#include <driver_tca9548a.h>
#include <driver_max31865.h>

/* for debug */
#include <driver_bme280.h>

#if FUNCONF_USE_DEBUGPRINTF
#define DPRINTF(FMT, ARGS...) printf((FMT), ## ARGS)
#else
#define DPRINTF(FMT, ARGS...) ((void)0)
#endif

/* get maximum bytes required per struct */
#if 0
#define ss(X) sizeof(X)
#define SS(X, Y) MAX(ss(X), ss(Y))
#define MS(X, Y) MAX(X, ss(Y))

typedef struct combo_sensor_s {
  bmp280_handle_t;
  aht20_handle_t;
} combo_sensor_t;

const int max_struct = MS(MS(MS(MS(MS(MS(MS(MS(MS(MS(SS(aht20_handle_t, aht30_handle_t), bmp280_handle_t), sht35_handle_t), sht4x_handle_t), shtc3_handle_t), si7021_handle_t), hdc1080_handle_t), htu21d_handle_t), htu31d_handle_t), mcp9808_handle_t), combo_sensor_t);

// const int almost_max_struct = MAX(MAX(MAX(MAX(MAX(sizeof(aht20_handle_t), sizeof(aht30_handle_t)), 0), sizeof(sht35_handle_t)), sizeof(sht4x_handle_t)), sizeof(shtc3_handle_t));
#endif
/* 
 * total maximum bytes usage will be max_struct * banks + almost_max * banks
 * for now it's 4 * 120 + 4 * 56 -> 704
 * BUT: only few ONE handles require unique instances:
 *  * BMP280
 * aht20 and aht30 is same driver with slightly different handling so onse only aht30.
 * 
 * Reducing max usage to 4 * 120 + (48 (aht) + 56 (sht3x) + 48 (sht4x) + 56 (shtc3)) -> 688
*/

typedef struct supported_sensor_e {
  sensor_t sensor;
  any_sensor_factory_t *factory;
} supported_sensor_t;

// NOTE: BMP280 can be mixed with AHT30
static supported_sensor_t supported_sensors[] = {
  {SENSOR_AHT, &sensor_factory_AHTXX},
  {SENSOR_BMP280, &sensor_factory_BMP280},
  {SENSOR_HDC1080, &sensor_factory_HDC1080},
  {SENSOR_HTU31D, &sensor_factory_HTU31D},
  {SENSOR_HTU21D, &sensor_factory_HTU21D},
  {SENSOR_SHT3X, &sensor_factory_SHT3X},
  {SENSOR_SHT4X, &sensor_factory_SHT4X},
  {SENSOR_SHTC3, &sensor_factory_SHTC3},
  {SENSOR_SI7021, &sensor_factory_SI7021},
  {SENSOR_MCP9808, &sensor_factory_MCP9808},
};

#define supported_sensors_length (sizeof(supported_sensors) / sizeof(supported_sensors[0]))

static uint8_t packet_pool[sizeof(packet_t)];
static uint8_t arena_pool[128];

static packet_t packet;

/* 
 * single SPI connected sensor
 * just output "raw" reading and do convoluted conversion on received side
 */
static max31865_handle_t max31865;

/* single I2C multiplexer */
static tca9548a_handle_t tca9548a;
// sht, dht, ...

#define BANKS_COUNT (4)

bank_t banks[BANKS_COUNT] = {
    {TCA9548A_CHANNEL_5, {{0, NULL}, {0, NULL}}},
    {TCA9548A_CHANNEL_4, {{0, NULL}, {0, NULL}}},
    {TCA9548A_CHANNEL_3, {{0, NULL}, {0, NULL}}},
    {TCA9548A_CHANNEL_2, {{0, NULL}, {0, NULL}}},
};

static void loop();


void spi_fun(void)
{
  #if 1
  spi_init_t cfg = {
    .byte_order = SPI_MSB_FIRST,
    .frame_size = SPI_FRAME_8BITS,
    .mode = SPI_MODE0,
    .nss_pin = PC0,
  };

  spi_init(&spi, &cfg);

  while(1) {
    uint64_t cycles = SysTick->CNT;

    spi_begin_transaction(&spi);

    spi_send8(0xD0);

    uint8_t chip_id = spi_recv8(0x00);


    spi_end_transaction(&spi);

    uint64_t elapsed = SysTick->CNT - cycles;

    printf("Chip id: %X (%04d)\n", chip_id, (int)elapsed);
    Delay_Ms(2000);

    // break;
  }

  return;
  #endif

  #if 0


  SPI_init();

  while(1) {
    uint64_t cycles = SysTick->CNT;

    SPI_begin_8();

    uint8_t chip_id = SPI_transfer_8(0xD0);
    chip_id = SPI_transfer_8(0x00); /* dummy read */

    SPI_end();

    uint64_t elapsed = SysTick->CNT - cycles;

    printf("Chip id: %X (%04d)\n", chip_id, (int)elapsed);

    Delay_Ms(2000);
  }

  return;
  #endif
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

  spi_init(&spi, &cfg);

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
  if ((err = tca9458a_iic_setup()))
  {
    printf("iic_setup: fialed: %d\n", err);
    return 1;
  }

  if ((err = i2c_write_raw(&i2c, buf, len)))
  {
    printf("iic_write_raw: fialed: %d\n", err);
  }
  return err;
}

static inline uint8_t tca9548a_iic_read(uint8_t *buf, uint16_t len)
{
  uint8_t err;
  if ((err = tca9458a_iic_setup()))
  {
    printf("iic_setup: fialed: %d\n", err);
    return 1;
  }

  if ((err = i2c_read_raw(&i2c, buf, len)))
  {
    printf("iic_read_raw: fialed: %d\n", err);
  }

  return err;
}

#define DEBUG_DATA0_ADDRESS ((volatile uint32_t *)0xE00000F4)


// SHTXX share same address, only one is possible
#define SENSOR_SHTXX (SENSOR_SHT3X | SENSOR_SHT4X)
#define SENSOR_ALL   (SENSOR_AHT | SENSOR_BMP280 | SENSOR_BME280 | \
                      SENSOR_SHT3X | SENSOR_SHT4X | SENSOR_SHTC3)

void i2c_scan_callback(const uint8_t addr)
{
  printf("Address: 0x%02X Responded.\n", addr);
}

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


// 0 - OK
// 1 - nothing detected
static uint8_t sensor_check(supported_sensor_t *supported_sensor, uint8_t *addr, uint8_t *response, arena_t *arena)
{
    if (supported_sensor->factory->address != *addr)
    {
      *addr = supported_sensor->factory->address;
      *response = i2c_ping(*addr)== I2C_OK;
    }

    if (!*response)
    {
      return 1;
    }

    any_sensor_t *sensor = supported_sensor->factory->construct(arena);
    if (sensor == NULL)
    {
      return 1;
    }

    if (sensor->probe(sensor))
    {
      return 0;
    }

    if (supported_sensor->factory->destroy)
    {
      supported_sensor->factory->destroy(sensor, arena);
    }

    return 1;
}

static supported_sensor_t *find_sensor(sensor_t type)
{
  for (int i = 0; i < supported_sensors_length; ++i)
  {
    if (supported_sensors[i].sensor == type)
    {
      return &supported_sensors[i];
    }
  }
  return NULL;
}

static uint8_t bank_check_new(bank_t *bank)
{
  DPRINTF("Check bank: %d\n", bank->channel);

  if (tca9548a_channel_set(&tca9548a, bank->channel))
  {
    DPRINTF("channel failed");
    return 1;
  }

  sensor_t sensors = bank_active_sensors(bank);

  if ((sensors & (SENSOR_BMP280 | SENSOR_AHT)) == (SENSOR_BMP280 | SENSOR_AHT))
  {
    /* bank has bmp280 and aht nothing more to check*/
    DPRINTF("Both AHTxx and BMP280 already detected\n");

    return 0;
  }

  uint8_t addr = 0;
  uint8_t response = 0;
  uint8_t sensor_id = 0;

  sensor_t sensors_to_check = ~0; /* check everything */

  if (sensors & SENSOR_BMP280)
  {
    DPRINTF("Has BMP280, check for AHTxx\n");
    sensor_id = 1;
    /* check for companion */
    sensors_to_check = SENSOR_AHT;
  }
  else if (sensors & SENSOR_AHT)
  {
    DPRINTF("Has AHTxx, check for BMP280\n");
    sensor_id = 1;
    /* has bmp280 or aht check for possible companion */
    sensors_to_check = SENSOR_BMP280;
  } 
  else if (sensors)
  {
    DPRINTF("Bank already has detected sensor\n");
    /* if bank has any other sensor then nothing else to check */
    return 0;
  }


  for (int i = 0; i < supported_sensors_length; ++i)
  {
    supported_sensor_t *supported_sensor = &supported_sensors[i];

    if ((supported_sensor->sensor & sensors_to_check) == 0)
    {
      DPRINTF("skip sensor\n");
      /* sensor is not allowed to check, skip */
      continue;
    }

    if (sensor_check(supported_sensor, &addr, &response, &bank->arena))
    {
      DPRINTF("not detected\n");
      /* sensor is not detected */
      continue;
    }

    bank->sensors[sensor_id].type = supported_sensor->sensor;
    bank->sensors[sensor_id++].sensor = supported_sensor->factory->construct(&bank->arena);

    if (sensor_id >= BANK_MAX_SENSORS)
    {
      DPRINTF("WARNING: Too much sensors on bank\n");
      /* WARNING: Too much sensor on given bank */
      return 1;
    }
  }

  return 0;
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

#if 0
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
#endif

#define CALCULATED_HPRE_DIV ((((HSI_VALUE) / (FUNCONF_SYSTEM_CORE_CLOCK)) - 1) << 4)

#if HSI_VALUE * FUNCONF_PLL_MULTIPLIER != FUNCONF_SYSTEM_CORE_CLOCK
# if (CALCULATED_HPRE_DIV + 1) * HSI_VALUE == FUNCONF_SYSTEM_CORE_CLOCK
#   error Requested clock is not multiplication or division of 24MHz
# endif
# if CALCULATED_HPRE_DIV & ~0xF0
#   error Expected system core clock is too low
# endif
#endif

int main()
{
  SystemInit();

  #if 1
  printf("Awaiting DOOM\n");
  #define DOOM 10
  for (int i = 0; i < DOOM; ++i) {
    Delay_Ms(1000);
    printf("%d/%d\n", i + 1, DOOM);
  }
  printf("Doom\n");

  // Check if there is ANY reasons to run it clocks other than 8MHz
  // Running at native is supported by default
  #if 0 && defined(FUNCONF_SYSTEM_CORE_CLOCK) && FUNCONF_SYSTEM_CORE_CLOCK != HSI_VALUE * (FUNCONF_PLL_MULTIPLIER + FUNCONF_USE_PLL)
    // no-os framework
    // RCC->CTLR |= (uint32_t)0x00000001;
    // RCC->CFGR0 &= (uint32_t)0xF8FF0000;
    // RCC->CTLR &= (uint32_t)0xFEF6FFFF;
    // RCC->CTLR &= (uint32_t)0xFFFBFFFF;
    // RCC->CFGR0 &= (uint32_t)0xFFFEFFFF;
    // RCC->CFGR0 &= (uint32_t)0xF8FF0000;

    // Flash configuration will be already at the correct value

    // HCLK for Periph 1 and 1 is set as 0 (no division) check if this will not backfire 
    RCC->CFGR0 &= ~RCC_HPRE; /* reset divider to 0 ( no division) */
    // RCC->CFGR0 |= RCC_HPRE_DIV3; /* set divider to 3 -> 24 / 3 == 8*/
    RCC->CFGR0 |= CALCULATED_HPRE_DIV; /* set divider to 3 -> 24 / 3 == 8*/


    // printf("System clock: ");
    // if ((RCC->CFGR0 & RCC_SW) == RCC_SW_HSE) {
    //   printf("HSE\n");
    // } else if ((RCC->CFGR0 & RCC_SW) == RCC_SW_HSI) {
    //   printf("HSI\n");
    // } else {
    //   printf("PLL\n");
    // }
    

    // printf("MCO source: ");
    // switch (RCC->CFGR0 & RCC_CFGR0_MCO)
    // {
    //   case RCC_CFGR0_MCO_HSE:
    //     printf("HSE\n"); break;
    //   case RCC_CFGR0_MCO_PLL:
    //     printf("PLL\n"); break;
    //   case RCC_CFGR0_MCO_HSI:
    //     printf("MCO\n"); break;
    //   case RCC_CFGR0_MCO_SYSCLK:
    //     printf("SYSCLK\n"); break;
    //   default:
    //     printf("NONE\n"); break;
    // }

  #endif

  printf("Hello?\n");

# if 1
  printf("Testing clock; each output should be printed in 1 second delay\n");
  for (int i = 0; i < DOOM; ++i) {
    Delay_Ms(1000);
    printf("TICK\n");
  }
# endif /* 0 */
#endif
  // #define RCC_CSS 0 /* disable */
  // #define HSEBYP 0 /* HSE bypass (disable) */

  // #define BASE_CFGR0 RCC_HPRE_DIV1 | RCC_PPRE2_DIV1 | RCC_PPRE1_DIV1
  // #define BASE_CTLR	(((FUNCONF_HSITRIM) << 3) | RCC_HSION | HSEBYP | RCC_CSS)
  // RCC->CFGR0 = BASE_CFGR0;
  // RCC->CTLR  = BASE_CTLR | RCC_HSION | RCC_PLLON; 			// Use HSI, enable PLL.

  // uart_fun();
  // asdf();

  // spi_fun();


  initializeGPIO();

  // SetupDebugPrintf();


  // DRIVER_TCA9548A_LINK_INIT(&tca9548a);
  // DRIVER_TCA9548A_LINK_DELAY_MS(&tca9548a, libdriver_delay_ms);
  // DRIVER_TCA9548A_LINK_IIC_READ(&tca9548a, tca9548a_iic_read);
  // DRIVER_TCA9548A_LINK_IIC_WRITE(&tca9548a, tca9548a_iic_write);
  // tca9548a_set_addr_pin(&tca9548a, TCA9548A_ADDRESS_A0);

  i2c_err_t err = i2c_init(&i2c);
  if (err)
  {
    printf("Error in init: %d\n", err);
  }

  spi_init_t cfg = {
    .byte_order = SPI_MSB_FIRST,
    .frame_size = SPI_FRAME_8BITS,
    .mode = SPI_MODE00,
    .nss_pin = PC0,
  };
  spi_init(&spi, &cfg);

  Delay_Ms(250);

  // "szeroki" 0x18
  // "malutki" 0x40
  // htu31d 0x40
  // while(1) {
  //   printf("SKAN\n");
  //   i2c.tout *= 1;
  //   i2c_init(&i2c);
  //   Delay_Ms(1000);
  //   i2c_scan(i2c_scan_callback);
  //   printf("SKAN END\n");
  //   Delay_Ms(2000);
  // }

  // printf("Let's the scan begin\n");
  // i2c_scan(i2c_scan_callback);
  // printf("Scan done\n");

  // It's OPEN DRAIN mode, so by default it uses TCA5498A board resistor
  // to keep reset pin high
  Delay_Ms(2000);

  int32_t t;
  uint16_t p;
  uint16_t h;

  int probed = 0;
  for (int i = 0; i < supported_sensors_length; ++i) {
    any_sensor_t* sensor = supported_sensors[i].factory->construct(NULL);

    while(1)
    {
      if (probed == 0 && sensor->probe(sensor))
      {
        printf("probe failed\n");
      }
      else
      {
        if (probed == 0) {
          printf("probe OK\n");
        }

        probed = 1;

        if (sensor->obtain(sensor, &t, &p, &h) == OBTAIN_ERROR)
        {
          printf("obtain failed: %ld %d %d\n", t, p, h);
          probed = 0;
        }
        else
        {
          printf("obtain OK\n");
          break;
        }
      }
      Delay_Ms(2000);
    }

    supported_sensors[i].factory->destroy(sensor, NULL);
  }


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
      // setupSHTC3();
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

static void bank_fetch(bank_t *bank)
{
  bank_check_new(bank);

  int32_t temp;
  uint16_t pres;
  uint16_t hum;

  int bank_id;
  for (int i = 0; i < BANKS_COUNT; ++i)
  {
    if (&banks[i] == bank)
    {
      bank_id = i;
      break;
    }
  }

  if (bank_active_count(bank) == 0)
  {
    return;
  }

  uint8_t active = 0;
  for (int i = 0; i < BANK_MAX_SENSORS; i++)
  {
    active_sensor_t *sensor = &bank->sensors[i];
    if (sensor->type == 0)
    {
      break;
    }

    // fetch data and put to storage
    obtain_t result = sensor->sensor->obtain(sensor->sensor, &temp, &pres, &hum);
    if (result == OBTAIN_ERROR)
    {
      find_sensor(sensor->type)->factory->destroy(sensor->sensor, &bank->arena);

      sensor->type = SENSOR_NONE;
      sensor->sensor = NULL;
    }

    if (result & OBTAIN_HUMIDITY)
    {
      packet_put_reading(&packet, 0, i, OBTAIN_HUMIDITY, hum);
    }

    if (result & OBTAIN_PRESSURE)
    {
      packet_put_reading(&packet, 0, i, OBTAIN_PRESSURE, pres);
    }

    if (result & OBTAIN_TEMPERATURE)
    {
      packet_put_reading(&packet, 0, i, OBTAIN_TEMPERATURE, temp);
    }

    ++active;
  }

  if (active == 0)
  {
    arena_clear(&bank->arena);
  }
}

void loop()
{
  printf("LOOP\n");
  packet_clear_readings(&packet);

  if (max31865_single_read(&max31865, &tmp_raw_temperature16, NULL) == 0)
  {
    packet_put_reading(&packet, PACKET_BANK_PT100, 0, OBTAIN_TEMPERATURE, tmp_raw_humidity16);
  }

  for (int b = 0; b < BANKS_COUNT; ++b)
  {
    bank_fetch(&banks[b]);
  }
  // for (unsigned b = 0; b < sizeof(banks)/sizeof(banks[0]); ++b)
  // {
  //   bank_check_new(&banks[b]);

  //   arena_pool[b] = banks[b].sensors;
  //   printf("Set bank: %d@%d\n", b, banks[b].channel);
  //   if (tca9548a_channel_set(&tca9548a, banks[b].channel))
  //   {
  //     printf("failed to enable channel: %d\n", banks[b].channel);
  //     continue;
  //   }

  //   i2c_scan(i2c_scan_callback);
  // }

  // tca9548a_reset(&tca9548a);
  Delay_Ms(2000);

  return;
  #if 0

  for (unsigned i = 0; i < sizeof(banks) / sizeof(banks[0]); ++i)
  {
    struct bank_t *b = &banks[i];

    // it will set correct mux configuration
    printf("Begin bank: %d\n", i);
    bank_check_new(b);

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
    #endif
  // printf("Let's the scan begin\n");
  // i2c_scan(i2c_scan_callback);
  // printf("Scan done\n");

  // Delay_Ms(5000);
  // return;

  Delay_Ms(6000);
}