#include "ahtxx.h"
#include "bank.h"
#include "bmp280.h"
#include "common_driver.h"
#include "debug.h"
#include "ens210.h"
#include "hdc1080.h"
#include "packet.h"
#include "shtxx.h"
#include "spi.h"
#include "si7021.h"
#include "htuxx.h"
#include "mcp9808.h"
#include "max31865.h"
#include "wtocol.h"
#include "sleep.h"

#include <alloca.h>
#include <stdio.h>
#include <ch32fun.h>

#include <lib_i2c.h>
#include <driver_tca9548a.h>

/* for debug */
#include <driver_bme280.h>


#define CALCULATED_HPRE_DIV ((((HSI_VALUE) / (FUNCONF_SYSTEM_CORE_CLOCK)) - 1) << 4)

#if HSI_VALUE * FUNCONF_PLL_MULTIPLIER != FUNCONF_SYSTEM_CORE_CLOCK
# if HSI_VALUE / ((CALCULATED_HPRE_DIV >> 4) + 1) != FUNCONF_SYSTEM_CORE_CLOCK
#   error Requested clock is not multiplication or division of 24MHz
# endif
# if CALCULATED_HPRE_DIV & ~0xF0
#   error Expected system core clock is too low
# endif
#endif

#define DEBUG_DATA0_ADDRESS ((volatile uint32_t *)0xE00000F4)
#define TCA9548A_RESET_PIN  (PC4)
#define HC12_SET_PIN        (PD6)
/* HC12_CMD_SETTLE_MS: SET low to first AT byte. The manual's 40ms was verified
 * failing (AT+SLEEP ignored: no OK+SLEEP, module stays awake); 150ms verified
 * working (LA: OK+SLEEP received). */
#ifndef HC12_CMD_SETTLE_MS
#define HC12_CMD_SETTLE_MS (150)
#endif

#ifndef LOOP_WAIT
#define LOOP_WAIT          (60000 * 5)
#endif

/* Set UART baud rate here */
#define UART_BR            (9600)
#define UART_SIMPLEX       (1)
#define UART_DUPLEX        (2)
#define RX_BUF_LEN 16 // size of receive circular buffer

u8 rx_buf[RX_BUF_LEN] = {0}; // DMA receive buffer for incoming data
u8 cmd_buf[RX_BUF_LEN] = {0}; // buffer for complete command strings

#define USART_MODE UART_SIMPLEX



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
  const sensor_t sensor;
  const any_sensor_factory_t *factory;
} supported_sensor_t;

/* NOTE: BMP280 can be mixed with AHT30 */
/* HTU/SI/SHT could share most of it's logic */
static supported_sensor_t supported_sensors[] = {
  {SENSOR_AHT, &sensor_factory_AHTXX},
  {SENSOR_BMP280, &sensor_factory_BMP280}, // crashing 003?
  {SENSOR_HDC1080, &sensor_factory_HDC1080},
  {SENSOR_HTU31D, &sensor_factory_HTU31D},
  {SENSOR_HTU21D, &sensor_factory_HTU21D},
  {SENSOR_SI7021, &sensor_factory_SI7021},
  #ifdef CH32V006
  {SENSOR_SHT3X, &sensor_factory_SHT3X},
  {SENSOR_SHT4X, &sensor_factory_SHT4X},
  {SENSOR_SHTC3, &sensor_factory_SHTC3},
  {SENSOR_MCP9808, &sensor_factory_MCP9808},
  {SENSOR_ENS210, &sensor_factory_ENS210},
  #endif
};

static void dma_uart_tx(const void *data, size_t len);
static void dma_uart_tx_wait(void);

#define supported_sensors_length (sizeof(supported_sensors) / sizeof(supported_sensors[0]))

static uint8_t packet_pool[sizeof(hc12_wire_t) + sizeof(packet_t)];
static uint8_t *packet_payload = packet_pool + sizeof(hc12_wire_t) - 1;
/* maximum allowed payload size (with exclusion on CRC8) */
#define packet_payload_max_size sizeof(packet_t)

static uint8_t arena_pool[128];
static arena_t sensor_arena = ARENA_INIT(arena_pool, sizeof(arena_pool));

static packet_t packet;

/* single I2C multiplexer 
 * Reset on PC4 (pin 14)
 */
static tca9548a_handle_t tca9548a = {0};

#define BANKS_COUNT (4)

bank_t banks[BANKS_COUNT] = {
    {TCA9548A_CHANNEL_5, {{0, NULL}, {0, NULL}}, &sensor_arena},
    {TCA9548A_CHANNEL_4, {{0, NULL}, {0, NULL}}, &sensor_arena},
    {TCA9548A_CHANNEL_3, {{0, NULL}, {0, NULL}}, &sensor_arena},
    {TCA9548A_CHANNEL_2, {{0, NULL}, {0, NULL}}, &sensor_arena},
};

static void loop();


void spi_fun2(void)
{
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

  // spi_init(&spi, 0, SPI_MODE0, 8, 1);
  spi_init_t cfg = {
    .byte_order = SPI_MSB_FIRST,
    .frame_size = SPI_FRAME_8BITS,
    .mode = SPI_MODE0,
    .nss_pin = PC0,
  };
  DPRINTF("XX_1 = %d\n", spi.is_hw);
  spi_init(&spi, &cfg);
  DPRINTF("XX_2 = %d\n", spi.is_hw);

  DPRINTF("I = %d\n", bme.inited);

  while(1)
  {
    if (!bme.inited)
    {
      if (bme280_init(&bme))
      {
        DPRINTF("F\n");
      }
      else
      {
        DPRINTF("I\n");
        bme280_set_mode(&bme, BME280_MODE_FORCED);
        bme280_set_filter(&bme, BME280_FILTER_OFF);
        bme280_set_humidity_oversampling(&bme, BME280_OVERSAMPLING_x1);
        bme280_set_temperatue_oversampling(&bme, BME280_OVERSAMPLING_x1);
        bme280_set_pressure_oversampling(&bme, BME280_OVERSAMPLING_x1);
      }
    }
    else
    {
      DPRINTF("Already done\n");
      uint32_t tr, pr, hr;
      float t, p, h;

      bme280_read_temperature_pressure_humidity(&bme, &tr, &t, &pr, &p, &hr, &h);
      DPRINTF("T: %d P: %d H: %d\n", (int)t, (int)p, (int)h);
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
    DPRINTF("iic_setup: fialed: %d\n", err);
    return 1;
  }

  if ((err = i2c_write_raw(&i2c, buf, len)))
  {
    DPRINTF("iic_write_raw: fialed: %d\n", err);
  }
  return err;
}

static inline uint8_t tca9548a_iic_read(uint8_t *buf, uint16_t len)
{
  uint8_t err;
  if ((err = tca9458a_iic_setup()))
  {
    DPRINTF("iic_setup: fialed: %d\n", err);
    return 1;
  }

  if ((err = i2c_read_raw(&i2c, buf, len)))
  {
    DPRINTF("iic_read_raw: fialed: %d\n", err);
  }

  return err;
}

void i2c_scan_callback(const uint8_t addr)
{
  DPRINTF("Address: 0x%02X Responded.\n", addr);
}

/* 
 * Enter passthrough mode, but do not skip recommended amount of time
 */
static void hc12_enter_passthrough_fast()
{
  GPIO_digitalWrite(HC12_SET_PIN, 1);
}

static void hc12_enter_passthrough()
{
  hc12_enter_passthrough_fast();
  /* give it a while to settle */
  Delay_Ms(80); /* test delay, 80: value from manual */
}


static void hc12_enter_cmd()
{
  GPIO_digitalWrite(HC12_SET_PIN, 0);
  Delay_Ms(HC12_CMD_SETTLE_MS);
}

static void hc12_wake_up()
{
  #if 1
  hc12_enter_cmd();
  hc12_enter_passthrough();
  #endif
}

static void hc12_sleep()
{
  #if 1
  hc12_enter_cmd();

  static const uint8_t sleep_cmd[] = "AT+SLEEP";
  dma_uart_tx(sleep_cmd, sizeof(sleep_cmd) - 1); /* ignore '\0' */
  dma_uart_tx_wait();

  /* SET high arms sleep on cmd-mode exit; no reply line (RX wired to SET), so
   * blind-wait past the module's worst response: measured 29.5ms, 3.4x margin */
  Delay_Ms(100);
  /* fast variant as we do not send anything over long period after sleep */
  hc12_enter_passthrough_fast();
  // hc12_enter_passthrough();
  #endif
}

static void hc12_send_old(const uint8_t *data, size_t len)
{
  //  RAM: [= ] 11.9% (used 972 bytes from 8192 bytes)
  //Flash: [==== ] 44.7% (used 28376 bytes from 63488 bytes)
  /* there is small problem with HC-12 where it's capable of sending 17 bytes
    * at once and truncating everything above
    */
  size_t packet_chunks = len >> 4;
  if ((packet_chunks >> 4) != len) {
    packet_chunks++;
  }

  const uint8_t *offset = data;
  for (size_t i = 0; i < packet_chunks; ++i) {
    size_t chunk = MIN(16, len - (i << 4));
    dma_uart_tx(offset, chunk);
    DPRINTF("%lu: packet dma request done: %u\n", SysTick->CNT, chunk);
    dma_uart_tx_wait();
    DPRINTF("%lu: wait done\n", SysTick->CNT);
    offset += 16;
    Delay_Ms(40 * 2);
  }
}

static void hc12_send_new(const uint8_t *data, size_t len)
{
# define HC12_PACKET_SIZE (17)

  if (len == 0) return;

  DPRINTF("len: %d\n", len);

  const uint8_t *ptr = data;
  size_t remaining = len;

  while (remaining > 0) {
      // Send either 17 bytes or whatever is left
      size_t chunk = (remaining >= HC12_PACKET_SIZE) ? HC12_PACKET_SIZE : remaining;

      dma_uart_tx(ptr, chunk);

      // DPRINTF("%lu: packet dma request done: %u\n", SysTick->CNT, chunk);

      uint32_t now = SysTick->CNT;
      dma_uart_tx_wait();
      now = SysTick->CNT - now;
      // DPRINTF("wait done: %lu ticks\n", now);

      Delay_Ms(40);

      ptr += chunk;
      remaining -= chunk;
  }

# undef HC12_PACKET_SIZE
}  

#define hc12_send(data, len) hc12_send_new(data, len)

static void initializeGPIO()
{
  GPIO_port_enable(GPIO_port_C);

  /* 2MHz is NOT working on 006 (no change on pin) */
  GPIO_pinMode(TCA9548A_RESET_PIN, GPIO_pinMode_O_openDrain, GPIO_Speed_10MHz);
  /* Default state for reset pin is HIGH or HI-Z */
  GPIO_digitalWrite(TCA9548A_RESET_PIN, 1);

  GPIO_port_enable(GPIO_port_D);
  /* it can also be driven by openDrain */
  // GPIO_pinMode(HC12_SET_PIN, GPIO_pinMode_O_pushPull, GPIO_Speed_10MHz);
  GPIO_pinMode(HC12_SET_PIN, GPIO_pinMode_O_openDrain, GPIO_Speed_10MHz);
  /* PULL-UP to ensure we are in pass-through mode */
  hc12_enter_passthrough();
}

// A slave half-served mid-byte holds SDA low; only clock pulses let it finish.
// Up to 9 SCL toggles, then a STOP, release the wire. i2c_init() restores AF.
static void i2c_bus_recover(void)
{
  const uint32_t scl = 1 << I2C_PIN_SCL;
  const uint32_t sda = 1 << I2C_PIN_SDA;

  I2C1->CTLR1 &= ~I2C_CTLR1_PE;

  // high SDA: no wire wedge, peripheral re-init alone is enough
  if (I2C_PORT->INDR & sda) return;

  I2C_PORT->CFGLR = (I2C_PORT->CFGLR & ~(0x0F << (4 * I2C_PIN_SCL))) |
                    (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD) << (4 * I2C_PIN_SCL);
  I2C_PORT->CFGLR = (I2C_PORT->CFGLR & ~(0x0F << (4 * I2C_PIN_SDA))) |
                    (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD) << (4 * I2C_PIN_SDA);

  for (int i = 0; i < 9 && (I2C_PORT->INDR & sda) == 0; ++i)
  {
    I2C_PORT->OUTDR &= ~scl;
    Delay_Us(5);
    I2C_PORT->OUTDR |= scl;
    Delay_Us(5);
  }

  I2C_PORT->OUTDR &= ~sda;
  Delay_Us(5);
  I2C_PORT->OUTDR |= sda;
  Delay_Us(5);
}

static uint8_t tca9548a_reset(tca9548a_handle_t *handle)
{
  // pull down, and return to default mode for reset cycle
  GPIO_digitalWrite(TCA9548A_RESET_PIN, 0);
  // Delay_Us(10);
  Delay_Ms(1);
  GPIO_digitalWrite(TCA9548A_RESET_PIN, 1);
  
  DPRINTF("reset channels after reset: ");
  while(tca9548a_channel_set(handle, TCA9548A_CHANNEL_NONE))
  {
    DPRINTF(".");
    i2c_bus_recover();
    // i2c_send_stop(&i2c, true);

    Delay_Ms(100);
    RCC->APB1PCENR &= ~RCC_APB1Periph_I2C1;
    I2C1->CTLR1 &= ~I2C_CTLR1_PE;
    i2c_init(&i2c);
  }
  DPRINTF("\n");

  return 0;
}


/*
 * 0 - OK
 * 1 - nothing detected
 * On failure content of "sensor" is unchanged or NULL
 */
static uint8_t sensor_check(const supported_sensor_t *supported_sensor, uint8_t *addr, uint8_t *response, arena_t *arena, any_sensor_t **sensor)
{
    if (supported_sensor->factory->address != *addr)
    {
      // DPRINTF("Address changed: %x -> %x\n", *addr, supported_sensor->factory->address);
      *addr = supported_sensor->factory->address;
      *response = i2c_ping(*addr)== I2C_OK;
    }

    // DPRINTF("ping response: %d\n", *response);

    if (!*response)
    {
      return 1;
    }

    *sensor = supported_sensor->factory->construct(arena);
    if (*sensor == NULL)
    {
      DPRINTF("no sensor constructed: %s\n", sensor_to_str(supported_sensor->sensor));
      return 1;
    }

    DPRINTF("probe: %s\n", sensor_to_str(supported_sensor->sensor));

    if ((*sensor)->probe(*sensor) == 0)
    {
      DPRINTF("Sensor detected: %s\n", sensor_to_str(supported_sensor->sensor));
      return 0;
    }

    DPRINTF("Sensor not detected: %s\n", sensor_to_str(supported_sensor->sensor));

    if (supported_sensor->factory->destroy)
    {
      DPRINTF("sensor destroy: %s\n", sensor_to_str(supported_sensor->sensor));
      supported_sensor->factory->destroy(*sensor, arena);
    }

    return 1;
}

static const supported_sensor_t *find_sensor(sensor_t type)
{
  for (size_t i = 0; i < supported_sensors_length; ++i)
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
  if (I2C1->STAR2 & I2C_STAR2_BUSY)
  {
    /* previous bank's probing can leave the bus held; its channel is still
       open here, so recovery reaches the device that wedged the bus */
    DPRINTF("i2c bus busy, recovering\n");
    i2c_bus_recover();
    Delay_Ms(100);
    RCC->APB1PCENR &= ~RCC_APB1Periph_I2C1;
    I2C1->CTLR1 &= ~I2C_CTLR1_PE;
    i2c_init(&i2c);
  }

  if (tca9548a_channel_set(&tca9548a, bank->channel))
  {
    DPRINTF("channel failed\n");
    return 1;
  }

  sensor_t sensors = bank_active_sensors(bank);

  if ((sensors & (SENSOR_BMP280 | SENSOR_AHT)) == (SENSOR_BMP280 | SENSOR_AHT))
  {
    /* bank has bmp280 and aht nothing more to check*/
    DPRINTF("both AHTxx and BMP280 already detected\n");

    return 0;
  }

  uint8_t addr = 0;
  uint8_t response = 0;
  uint8_t sensor_id = 0;

  sensor_t sensors_to_check = ~0; /* check everything */

  if (sensors & SENSOR_BMP280)
  {
    DPRINTF("has BMP280, check for AHTxx\n");
    sensor_id = 1;
    /* check for companion */
    sensors_to_check = SENSOR_AHT;
  }
  else if (sensors & SENSOR_AHT)
  {
    DPRINTF("has AHTxx, check for BMP280\n");
    sensor_id = 1;
    /* has bmp280 or aht check for possible companion */
    sensors_to_check = SENSOR_BMP280;
  } 
  else if (sensors)
  {
    DPRINTF("bank already has detected sensor, skip check\n");
    /* if bank has any other sensor then nothing else to check */
    return 0;
  }


  for (size_t i = 0; i < supported_sensors_length; ++i)
  {
    supported_sensor_t *current_sensor = &supported_sensors[i];

    if ((current_sensor->sensor & sensors_to_check) == 0)
    {
      DPRINTF("skip sensor: %s\n", sensor_to_str(current_sensor->sensor));
      /* sensor is not allowed to check, skip */
      continue;
    }

    any_sensor_t *sensor = NULL;
    if (sensor_check(current_sensor, &addr, &response, bank->arena, &sensor))
    {
      // DPRINTF("not detected: %s\n", sensor_to_str(current_sensor->sensor));
      /* sensor is not detected */
      continue;
    }

    DPRINTF("found: %s\n", sensor_to_str(current_sensor->sensor));

    bank->sensors[sensor_id].type = current_sensor->sensor;
    bank->sensors[sensor_id++].sensor = sensor;

    /* drop any sensors that share I2C address */
    DPRINTF("check remaining sensors\n");

    for (size_t j = i + 1; j < supported_sensors_length; ++j)
    {
      const supported_sensor_t *check = &supported_sensors[j];
      if ((sensors_to_check & check->sensor) && check->factory->address == addr)
      {
        DPRINTF("Disable: %s (share addr)\n", sensor_to_str(check->sensor));
        sensors_to_check &= ~check->sensor;
      }
    }

    /* sensor_id here is 1, 2, ... */
    if (sensor_id > BANK_MAX_SENSORS)
    {
      DPRINTF("WARNING: Too much sensors on bank\n");
      /* WARNING: Too much sensor on given bank */
      return 1;
    }
  }

  return 0;
}

static void uart_setup(void)
{
#if USART_MODE == UART_DUPLEX
  // TODO: test half duplex?
	// Enable UART and GPIOD
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1;
  funPinMode(PD5, GPIO_CNF_OUT_PP_AF | GPIO_Speed_10MHz);
  funPinMode(PD6, GPIO_CNF_IN_FLOATING); /* PU is not working, use floating */

	// GPIOD->CFGLR = 
	// // Push-Pull, 10MHz Output on D5, with AutoFunction
  //   ((GPIOD->CFGLR & ~(0xF<<(4*5))) | ((GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*5)));
  //     // /* input pull up on D6 */
  //   // | ((GPIOD->CFGLR & ~(0xF<<(5*5))));
  //     // | (GPIOD->CFGLR & ~(0xF << (4 * 6))) | ((GPIO_CNF_IN_PUPD) << (4 * 6));

	// Setup UART for Tx 8n1 and Rx
	USART1->CTLR1 = USART_WordLength_8b | USART_Parity_No | USART_Mode_Tx | USART_Mode_Rx;
	USART1->CTLR2 = USART_StopBits_1;
	// Enable Tx and Rx DMA event
	USART1->CTLR3 = USART_DMAReq_Tx | USART_DMAReq_Rx;

	// Set baud rate and enable UART
	USART1->BRR = ((FUNCONF_SYSTEM_CORE_CLOCK) + (UART_BR)/2) / (UART_BR);
	USART1->CTLR1 |= CTLR1_UE_Set;
#else
  // TODO: test half duplex?
	// Enable UART and GPIOD
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1;
  funPinMode(PD5, GPIO_CNF_OUT_PP_AF | GPIO_Speed_10MHz);

	// Setup UART for Tx 8n1
	USART1->CTLR1 = USART_WordLength_8b | USART_Parity_No | USART_Mode_Tx;
	USART1->CTLR2 = USART_StopBits_1;
	// Enable Tx DMA event
	USART1->CTLR3 = USART_DMAReq_Tx;

	// Set baud rate and enable UART
	USART1->BRR = ((FUNCONF_SYSTEM_CORE_CLOCK) + (UART_BR)/2) / (UART_BR);
	USART1->CTLR1 |= CTLR1_UE_Set;
#endif
}

static void dma_uart_setup(void)
{
	// Enable DMA peripheral
	RCC->AHBPCENR = RCC_AHBPeriph_SRAM | RCC_AHBPeriph_DMA1;

	// Disable channel just in case there is a transfer in progress
	DMA1_Channel4->CFGR &= ~DMA_CFGR1_EN;

	// USART1 TX uses DMA channel 4
	DMA1_Channel4->PADDR = (uint32_t)&USART1->DATAR;
	// MEM2MEM: 0 (memory to peripheral)''x
	// PL: 0 (low priority since UART is a relatively slow peripheral)
	// MSIZE/PSIZE: 0 (8-bit)
	// MINC: 1 (increase memory address)
	// CIRC: 0 (one shot)
	// DIR: 1 (read from memory)
	// TEIE: 0 (no tx error interrupt)
	// HTIE: 0 (no half tx interrupt)
	// TCIE: 1 (transmission complete interrupt enable)
	// EN: 0 (do not enable DMA yet)
	DMA1_Channel4->CFGR = DMA_CFGR1_MINC | DMA_CFGR1_DIR;// | DMA_CFGR1_TCIE;

  // USART1 RX uses DMA channel 5
  DMA1_Channel5->CFGR &= ~DMA_CFGR1_EN;
  DMA1_Channel5->PADDR = (uint32_t)&USART1->DATAR;
  DMA1_Channel5->CNTR = RX_BUF_LEN;
  DMA1_Channel5->MADDR = (uint32_t)&rx_buf;
  // MINC: 1 (increase memory address)
  // CIRC: 1 (circular)
  // EN: 1 (enable DMA)
  DMA1_Channel5->CFGR = DMA_CFGR1_CIRC | DMA_CFGR1_MINC;

	// Enable channel 4 interrupts
	// NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

static void process_cmd(const uint8_t *cmd)
{
  DPRINTF("CMD: %s\n", cmd);
  dma_uart_tx("hi\r\n", 4);
  dma_uart_tx_wait();
}


static void dma_uart_rx()
{
  DMA1_Channel5->CFGR |= DMA_CFGR1_EN;

  while(1)
	{
		static u32 tail = 0; // current read position in rx_buf
		static u32 cmd_end = 0; // end index of current command in rx_buf
		static u32 cmd_st = 0; // start index of current command in rx_buf

		// calculate head position based on DMA counter (modulo when DMA1_Channel5->CNTR = 0)
		u32 head = (RX_BUF_LEN - DMA1_Channel5->CNTR) % RX_BUF_LEN; // current write position in rx_buf
		
		// process new bytes in rx_buf. when a newline character is detected, the command is copied to cmd_buf
		while (tail != head)
		{
      DPRINTF("buf: %d '%c'\n", rx_buf[tail], rx_buf[tail]);
      // minicom mill send '\r' as [ENTER] not '\n'
			if ( rx_buf[tail] == '\n' || rx_buf[tail] == '\r' ) 
			{
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

				process_cmd(cmd_buf);

				// update start position for next command
				cmd_st = (cmd_end + 1) % RX_BUF_LEN;
			}

			// move to next position 
			tail = (tail+1) % RX_BUF_LEN;
		}
	}
  DMA1_Channel5->CFGR &= ~DMA_CFGR1_EN;
}

static void dma_uart_tx(const void *data, size_t len)
{
	// Disable DMA channel (just in case a transfer is pending)
	DMA1_Channel4->CFGR &= ~DMA_CFGR1_EN;

	// Set transfer length and source address
	DMA1_Channel4->CNTR = len;
	DMA1_Channel4->MADDR = (uint32_t)data;
	// Enable DMA channel to start the transfer
	DMA1_Channel4->CFGR |= DMA_CFGR1_EN;
}

static void dma_uart_tx_wait()
{
  while(DMA1_Channel4->CNTR)
  {
    /* wait until all data is flushed */
    __NOP();
  }
}


int main()
{
  /* safety, if someone will call deepsleep */
  Delay_Ms(2000);

  SystemInit();
#if HSI_VALUE * FUNCONF_PLL_MULTIPLIER != FUNCONF_SYSTEM_CORE_CLOCK
  // SystemInit sets HPRE to DIV1, restore the configured core clock
  RCC->CFGR0 = (RCC->CFGR0 & ~RCC_HPRE) | CALCULATED_HPRE_DIV;
#endif

#if 0
# if 0
  DPRINTF("Awaiting DOOM\n");
  #define DOOM 10
  for (int i = 0; i < DOOM; ++i) {
    Delay_Ms(1000);
    DPRINTF("%d/%d\n", i + 1, DOOM);
  }
  DPRINTF("Doom\n");
# endif

  // Check if there is ANY reasons to run it clocks other than 8MHz
  // Running at native is supported by default
# if 0 && defined(FUNCONF_SYSTEM_CORE_CLOCK) && FUNCONF_SYSTEM_CORE_CLOCK != HSI_VALUE * (FUNCONF_PLL_MULTIPLIER + FUNCONF_USE_PLL)
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


    // DPRINTF("System clock: ");
    // if ((RCC->CFGR0 & RCC_SW) == RCC_SW_HSE) {
    //   DPRINTF("HSE\n");
    // } else if ((RCC->CFGR0 & RCC_SW) == RCC_SW_HSI) {
    //   DPRINTF("HSI\n");
    // } else {
    //   DPRINTF("PLL\n");
    // }
    

    // DPRINTF("MCO source: ");
    // switch (RCC->CFGR0 & RCC_CFGR0_MCO)
    // {
    //   case RCC_CFGR0_MCO_HSE:
    //     DPRINTF("HSE\n"); break;
    //   case RCC_CFGR0_MCO_PLL:
    //     DPRINTF("PLL\n"); break;
    //   case RCC_CFGR0_MCO_HSI:
    //     DPRINTF("MCO\n"); break;
    //   case RCC_CFGR0_MCO_SYSCLK:
    //     DPRINTF("SYSCLK\n"); break;
    //   default:
    //     DPRINTF("NONE\n"); break;
    // }

  DPRINTF("Hello?\n");

# endif /* core setup */
#   if 0
  DPRINTF("Testing clock; each output should be printed in 1 second delay\n");
  for (int i = 0; i < DOOM; ++i) {
    Delay_Ms(1000);
    DPRINTF("TICK\n");
  }
#   endif /* 0 */
#endif /* 1 */

  uart_setup();
  dma_uart_setup();

  initializeGPIO();

  /* disable radio ASAP */
  hc12_wake_up();
  hc12_sleep();

  /* Configure I2C for sensors and mux */
  i2c_err_t err = i2c_init(&i2c);
  if (err)
  {
    DPRINTF("Error in init: %d\n", err);
  }
  else
  {
    DPRINTF("I2C OK!\n");
  }

  /* PT100 will configure SPI for itself */
  sensor_max31865_init();

  DRIVER_TCA9548A_LINK_INIT(&tca9548a);
  DRIVER_TCA9548A_LINK_DELAY_MS(&tca9548a, libdriver_delay_ms);
  DRIVER_TCA9548A_LINK_IIC_READ(&tca9548a, tca9548a_iic_read);
  DRIVER_TCA9548A_LINK_IIC_WRITE(&tca9548a, tca9548a_iic_write);
  DRIVER_TCA9548A_LINK_DEBUG_PRINT(&tca9548a, debug_print);

  if (tca9548a_set_addr_pin(&tca9548a, TCA9548A_ADDRESS_A0))
  {
    DPRINTF("Unable to set muxer pin\n");
  }

  while (tca9548a_init(&tca9548a))
  {
    DPRINTF("unable to init muxer\n");
    if (tca9548a_reset(&tca9548a))
    {
      DPRINTF("mux reset failed\n");
    }
    Delay_Ms(500);
  }

  Delay_Ms(250);

  /* prepare packet */
  hc12_wire_t *hdr = (hc12_wire_t*) packet_pool;
  /* CRC8 field will be overwritten, ignore in init */
  hdr->hdr.version_zero = 0;
  hdr->hdr.version = 0x02;
  hdr->hdr.size_zero = 0;
  hdr->payload.base.device_id = 0x03; /* that should be from MCU */
  hdr->payload.base.sensors_count = 1;
  hdr->payload.base.sync_byte = 'W';
  hdr->payload.base.version = 0x02;
  hdr->payload.compoint_sensor.sensor_id = 0x13;


  while (1)
  {
    loop();
  }

  return 1;
}

static void bank_fetch(uint8_t bank_id)
{
  bank_t *bank = &banks[bank_id];

  start:
  DPRINTF("Check bank: %d\n", bank_id);

  if (bank_check_new(bank))
  {
    DPRINTF("bank %d check failed\n", bank_id);
    tca9548a_reset(&tca9548a);
    // RCC->APB1PCENR &= ~RCC_APB1Periph_I2C1;
    // I2C1->CTLR1 &= ~I2C_CTLR1_PE;
    // i2c_init(&i2c);
    DPRINTF("try again\n");
    goto start;

    return;
  }

  temperature_t temp;
  pressure_t pres;
  humidity_t hum;

  if (bank_active_count(bank) == 0)
  {
    DPRINTF("bank %d without sensors\n", bank_id);

    return;
  }

  uint8_t active = 0;
  for (size_t i = 0; i < BANK_MAX_SENSORS; i++)
  {
    active_sensor_t *sensor = &bank->sensors[i];
    if (sensor->type == SENSOR_NONE)
    {
      break;
    }

    const char* sensor_name = sensor_to_str(sensor->type);

    // fetch data and put to storage
    obtain_t result = sensor->sensor->obtain(sensor->sensor, &temp, &pres, &hum);
    if (result == OBTAIN_ERROR)
    {
      DPRINTF("bank %d sensor: %s: error reading data\n", bank_id, sensor_name);

      const any_sensor_factory_t *factory = find_sensor(sensor->type)->factory;
      if (factory->destroy)
      {
        factory->destroy(sensor->sensor, bank->arena);
      }

      sensor->type = SENSOR_NONE;
      sensor->sensor = NULL;

      /* move to next sensor */
      continue;
    }

    if (result & OBTAIN_HUMIDITY)
    {
      DPRINTF("bank %d sensor: %s: store humidity: %s (fpt)\n", bank_id, sensor_name, fpt_cstr(hum, -1));

      packet_put_reading(&packet, bank_id, i, OBTAIN_HUMIDITY, hum);
    }

    if (result & OBTAIN_PRESSURE)
    {
      // DPRINTF("bank %d sensor: %s: store pressure: %s (fpt)\n", bank_id, sensor_name, fpt_cstr(pres, -1));
      DPRINTF("bank %d sensor: %s: store pressure: " PR_FPT " (fpt)\n", bank_id, sensor_name,  F2PRINTF(fpt2fl_q17(pres)));

      packet_put_reading(&packet, bank_id, i, OBTAIN_PRESSURE, pres);
    }

    if (result & OBTAIN_TEMPERATURE)
    {
      DPRINTF("bank %d sensor: %s: store temperature: %s (fpt)\n", bank_id, sensor_name, fpt_cstr(temp, -1));

      packet_put_reading(&packet, bank_id, i, OBTAIN_TEMPERATURE, temp);
    }

    ++active;
  }
}

void acquire_pt100()
{
  if (max31865_single_read(&max31865, &tmp_raw_temperature16, NULL) == 0)
  {
    packet_put_reading(&packet, PACKET_BANK_PT100, 0, OBTAIN_TEMPERATURE, tmp_raw_temperature16);
    DPRINTF("pt100 reading ok: %d (raw)\n", tmp_raw_temperature16);
  }
  else
  {
    if (max31865.inited)
    {
      DPRINTF("pt100 reading fault, deinit\n");
      max31865_deinit(&max31865);
    }
    else
    {
      DPRINTF("no pt100 sensor\n");
    }
  }
}

void loop()
{
  DPRINTF("LOOP\n");

  packet_clear_readings(&packet);

  acquire_pt100();

  for (size_t b = 0; b < BANKS_COUNT; ++b)
  {
    bank_fetch(b);
  }

  if (!packet_sensor_readings(&packet))
  {
    DPRINTF("no sensors stored in packet\n");
    goto end;
  }

  /* set to max allowed, it will be changed to real size after compression */
  uint8_t packet_size = packet_payload_max_size;
  uint8_t *packet_data = packet_payload;
  if (packet_to_bytes(&packet, packet_data, &packet_size))
  {
    DPRINTF("unable to store packet in bytes");

    goto end;
  }

  // set payload size: dynamic payload + static header
  ((hc12_wire_t*)packet_pool)->hdr.size = packet_size + sizeof(weather_compound_packet_t);

  // calculate crc8 on packet
  // HDR + payload
  uint8_t total_size_without_crc8 = (sizeof(hc12_wire_t) - 1) + packet_size;
  // Put CRC8 at the end
  packet_payload[packet_size] = calculate_crc8(packet_pool, total_size_without_crc8);

  DPRINTF("waking up hc12\n");

  hc12_wake_up();

  DPRINTF("packet size: %d\n", packet_size);

  /* send it over uart and include crc8 at the end */
  hc12_send(packet_pool, total_size_without_crc8 + 1);

  DPRINTF("puting to sleep hc12\n");
  hc12_sleep();

  end:
#if MCU_SLEEP
  sleep_for(LOOP_WAIT);
#else
  Delay_Ms(LOOP_WAIT);
#endif
}
