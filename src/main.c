#include "ahtxx.h"
#include "bank.h"
#include "bmp280.h"
#include "common_driver.h"
#include "debug.h"
#include "hdc1080.h"
#include "packet.h"
#include "shtxx.h"
#include "spi.h"
#include "si7021.h"
#include "htuxx.h"
#include "mcp9808.h"
#include "wtocol.h"

#include <alloca.h>
#include <stdio.h>
#include <ch32fun.h>

#include <lib_i2c.h>
#include <driver_tca9548a.h>
#include <driver_max31865.h>

/* for debug */
#include <driver_bme280.h>


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
#ifdef WITH_HTU21D
  // broken chip sample? unable to read temp
  {SENSOR_HTU21D, &sensor_factory_HTU21D},
#endif
  {SENSOR_SI7021, &sensor_factory_SI7021},
  #ifdef CH32V006
  {SENSOR_SHT3X, &sensor_factory_SHT3X},
  {SENSOR_SHT4X, &sensor_factory_SHT4X},
  {SENSOR_SHTC3, &sensor_factory_SHTC3},
  {SENSOR_MCP9808, &sensor_factory_MCP9808},
  #endif
};

#define supported_sensors_length (sizeof(supported_sensors) / sizeof(supported_sensors[0]))

static uint8_t packet_pool[sizeof(hc12_wire_t) + sizeof(packet_t)];
static arena_t packet_arena = ARENA_INIT(packet_pool, sizeof(packet_pool));

static uint8_t arena_pool[128];
static arena_t sensor_arena = ARENA_INIT(arena_pool, sizeof(arena_pool));

static packet_t packet;

/* 
 * single SPI connected sensor
 * just output "raw" reading and do convoluted conversion on received side
 */
static max31865_handle_t max31865 = {0};

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

/* better name for this, not "init" per se, more like create? */
static uint8_t init_pt100_driver()
{
  DRIVER_MAX31865_LINK_INIT(&max31865, max31865_handle_t);
  DRIVER_MAX31865_LINK_DEBUG_PRINT(&max31865, debug_print);
  DRIVER_MAX31865_LINK_DELAY_MS(&max31865, libdriver_delay_ms);
  DRIVER_MAX31865_LINK_SPI_DEINIT(&max31865, libdriver_nop_void);
  DRIVER_MAX31865_LINK_SPI_INIT(&max31865, libdriver_nop_void);
  DRIVER_MAX31865_LINK_SPI_READ(&max31865, libdriver_spi_read);
  DRIVER_MAX31865_LINK_SPI_WRITE(&max31865, &libdriver_spi_write);

  return 0;
}

static uint8_t init_pt100()
{
  init_pt100_driver();

  DPRINTF("SPI init\n");

  spi_init_t cfg = {
    .byte_order = SPI_MSB_FIRST,
    .frame_size = SPI_FRAME_8BITS,
    .mode = SPI_MODE1, // 1 or 3
    .nss_pin = PC0,
  };



  if (spi_init(&spi, &cfg) != SPI_ERR_OK)
  {
    DPRINTF("SPI init failed\n");

    return 1;
  }

  uint8_t err;
  if ((err = max31865_init(&max31865)))
  {
    DPRINTF("PT100 init failed: %d\n", err);
    return err;
  }

  if ((err = max31865_set_fault_detection_cycle_control(&max31865, MAX31865_FAULT_DETECTION_CYCLE_CONTROL_NO_ACTION)))
  {
    DPRINTF("PT100 sfdcc: %d\n", err);
    return err;
  }
  /* we are using manual fault check, */
  #if 0
  max31865_set_high_fault_threshold
  max31865_set_low_fault_threshold
  #endif

  /* should not matter at all, it's battery powered device */
  #if 0
  max31865_set_filter_select(&max31865, MAX31865_FILTER_SELECT_50HZ);
  #endif


  /* required when fetching temperature directly, debug only */
  if ((err = max31865_set_reference_resistor(&max31865, 430.0f)))
  {
    DPRINTF("reference fail\n");
    return err;
  }
  if ((err = max31865_set_resistor(&max31865, MAX31865_RESISTOR_100PT)))
  {
    DPRINTF("resistor failed\n");
    return err;
  }

  /* ensure voltage is not enabled by default, it might be on when only MCU has gone
   * through power cycle
   */
  if ((err = max31865_set_vbias(&max31865, MAX31865_BOOL_FALSE)))
  {
    DPRINTF("vbias failed\n");
    return err;
  }

  if ((err = max31865_set_wire(&max31865, MAX31865_WIRE_3)))
  {
    DPRINTF("wire failed\n");
  }

  return err;
}

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
  printf("XX_1 = %d\n", spi.is_hw);
  spi_init(&spi, &cfg);
  printf("XX_2 = %d\n", spi.is_hw);

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

void spi_fun(void)
{

  // TODO(m)
  // Bitbank mode 1 and 3?

  // in spi config put PIN->HIGH before configuration

  // PIN -> LOW
  // delay (4x nop)
  // SPIN_SEND
  // delay_us(9)
  // PIN->HIGH
  // delay (4x nop)

  DRIVER_MAX31865_LINK_INIT(&max31865, max31865_handle_t);
  DRIVER_MAX31865_LINK_DEBUG_PRINT(&max31865, debug_print);
  DRIVER_MAX31865_LINK_DELAY_MS(&max31865, libdriver_delay_ms);
  DRIVER_MAX31865_LINK_SPI_DEINIT(&max31865, libdriver_nop_void);
  DRIVER_MAX31865_LINK_SPI_INIT(&max31865, libdriver_nop_void);
  DRIVER_MAX31865_LINK_SPI_READ(&max31865, libdriver_spi_read);
  DRIVER_MAX31865_LINK_SPI_WRITE(&max31865, libdriver_spi_write);

  #if 1
  spi_init_t cfg = {
    .byte_order = SPI_MSB_FIRST,
    .frame_size = SPI_FRAME_8BITS,
    .mode = SPI_MODE1, // 1 or 3
    .nss_pin = PC0,
  };

  DPRINTF("SPI init\n");


  if (spi_init(&spi, &cfg) != SPI_ERR_OK)
  {
    DPRINTF("SPI init failed\n");
  }

  // init_pt100();

  spi_err_t spi_err;
  while(1) {
    DPRINTF("check init\n");
    if (!max31865.inited)
    {
      if ((spi_err = max31865_init(&max31865)))
      {
        DPRINTF("F: %d\n", (int)spi_err);
      }
      else
      {
        DPRINTF("I\n");
        if (max31865_set_wire(&max31865, MAX31865_WIRE_3)) {
          printf("F1\n");
        }
        if (max31865_set_fault_detection_cycle_control(&max31865, MAX31865_FAULT_DETECTION_CYCLE_CONTROL_NO_ACTION))
        {
          printf("F2\n");
        }
        if (max31865_set_filter_select(&max31865, MAX31865_FILTER_SELECT_50HZ)) {
          printf("F3\n");
        }
        if (max31865_set_resistor(&max31865, MAX31865_RESISTOR_100PT)) {
          printf("F4\n");
        }
        if (max31865_set_vbias(&max31865, MAX31865_BOOL_FALSE)) {
          printf("F5\n");
        }
        if (max31865_set_reference_resistor(&max31865, 430)) {
          printf("F6\n");
        }
      }
    }
    else
    {
      printf("Already done\n");
      static max31865_wire_t wire;
      max31865_get_wire(&max31865, &wire);
      printf("wire: %d\n", wire);
      // max31865_set_wire(&max31865, MAX31865_WIRE_3);

      if (max31865_single_read(&max31865, &tmp_raw_temperature16, &tmp_temperature)) {
        printf("T: F\n");
      } else {
      printf("T: %d (RAW: %d)\n", (int)tmp_temperature, tmp_raw_temperature16);
      }
    }

    Delay_Ms(2000);
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


void i2c_scan_callback(const uint8_t addr)
{
  printf("Address: 0x%02X Responded.\n", addr);
}

static void dma_uart_tx(const void *data, size_t len);
static void dma_uart_tx_wait(void);

#define TCA9548A_RESET_PIN  (PC4)
#define HC12_SET_PIN        (PD6)

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
  Delay_Ms(80); /* test delay, value from manual */
}

static void hc12_enter_cmd()
{
  GPIO_digitalWrite(HC12_SET_PIN, 0);
  Delay_Ms(40); /* test: value from manual */
}

static void hc12_wake_up()
{
  hc12_enter_cmd();
  Delay_Ms(1); /* test */
  hc12_enter_passthrough();
}

static void hc12_sleep()
{
  hc12_enter_cmd();

  static const uint8_t sleep_cmd[] = "AT+SLEEP\n"; /* not sure it '\n' is required */
  dma_uart_tx(sleep_cmd, sizeof(sleep_cmd) - 1); /* ignore '\0' */
  dma_uart_tx_wait();

  Delay_Ms(10); /* test: how long is enough */
  hc12_enter_passthrough_fast();
}

static void hc12_send(uint8_t *data, size_t len)
{
  /* there is small problem with HC-12 where it's capable of sending 17 bytes
    * at once and truncating everything above
    */
  size_t packet_chunks = len >> 4;
  if ((packet_chunks >> 4) != len) {
    packet_chunks++;
  }

  uint8_t *offset = packet_pool;
  for (size_t i = 0; i < packet_chunks; ++i) {
    dma_uart_tx(offset, MIN(16, len - (i << 4)));
    DPRINTF("%06ld: packet dma request done\n", SysTick->CNT);
    dma_uart_tx_wait();
    DPRINTF("%06ld: wait done\n", SysTick->CNT);
    offset += 16;
    Delay_Ms(40);
  }
}

static void initializeGPIO()
{
  GPIO_port_enable(GPIO_port_C);

  /* 2MHz is NOT working on 006 (no change on pin) */
  GPIO_pinMode(TCA9548A_RESET_PIN, GPIO_pinMode_O_openDrain, GPIO_Speed_10MHz);
  /* Default state for reset pin is HIGH or HI-Z */
  GPIO_digitalWrite(TCA9548A_RESET_PIN, 1);

  GPIO_port_enable(GPIO_port_C);
  /* it can also be driven by openDrain */
  // GPIO_pinMode(HC12_SET_PIN, GPIO_pinMode_O_pushPull, GPIO_Speed_10MHz);
  GPIO_pinMode(HC12_SET_PIN, GPIO_pinMode_O_openDrain, GPIO_Speed_10MHz);
  /* PULL-UP to ensure we are in pass-through mode */
  hc12_enter_passthrough();
}

static uint8_t tca9548a_reset(tca9548a_handle_t *handle)
{
  // pull down, and return to default mode for reset cycle
  GPIO_digitalWrite(TCA9548A_RESET_PIN, 0);
  // Delay_Us(10);
  Delay_Ms(1);
  GPIO_digitalWrite(TCA9548A_RESET_PIN, 1);
  
  printf("reset channels after reset: ");
  while(tca9548a_channel_set(handle, TCA9548A_CHANNEL_NONE))
  {
    printf(".");
    // i2c_send_stop(&i2c, true);

    Delay_Ms(100);
    RCC->APB1PCENR &= ~RCC_APB1Periph_I2C1;
    I2C1->CTLR1 &= ~I2C_CTLR1_PE;
    i2c_init(&i2c);
  }
  printf("\n");

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
    printf("check remaining sensors\n");

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



#define RX_BUF_LEN 16 // size of receive circular buffer

u8 rx_buf[RX_BUF_LEN] = {0}; // DMA receive buffer for incoming data
u8 cmd_buf[RX_BUF_LEN] = {0}; // buffer for complete command strings

void uart_fun()
{
  /*
   |USART pins| Configuration                | GPIO configuration
   +==========+==============================+=================================+
   |USARTx_TX | Full-duplex mode             | Push-pull alternate outputs     |
   |          +------------------------------|---------------------------------+
   |          | Half-duplex synchronous mode | Open-drain alternate outputs    |
   +----------+------------------------------+---------------------------------+
   |USARTx_RX | Full-duplex mode             | Floating input or pull-up input |
   |          +------------------------------+---------------------------------+
   |          | Half-duplex synchronous mode | Not used                        |
   +----------+------------------------------+---------------------------------+
  */

  /*
  Role/Mapping  0000 Default  0001 0010 0011 0100 0101 0110 0111 1000 1001
  USART1_TX     PD5           PD6  PD0  PC0  PD1  PB3  PC5  PB5  PA0  PA0
  USART1_RX     PD6           PD5  PD1  PC1  PB3  PD1  PC6  PB6  PA1  PC4
  USART1_CTS    PD3           PC6  PC3  PC6  PD7  PD7  PC7  PC7  PD2  PD5
  USART1_RTS    PC2           PC7  PC2  PC7  PA5  PA5  PB4  PB4  PD3  PD4
*/
/*
  1   PD4     PD3   20
  2   PD5     PD2   19
  3   PD6     PD1   18
  4   PD7     PC7   17
  5   PA1     PC6   16
  6   PA2     PC5   15
  7   VSS     PC4   14
  8   PD0     PC3   13
  9   VDD     PC2   12
  10  PC0     PC1   11
*/
  // Tx: PD5 (2)
  // Rx: PD6 (3)
  funGpioInitD();
	funPinMode( PD5, GPIO_CFGLR_OUT_2Mhz_PP);
	
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
    
    // if (head) {
    //   printf("HEAD: %lu\n", head);
    // }
		
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


#define CALCULATED_HPRE_DIV ((((HSI_VALUE) / (FUNCONF_SYSTEM_CORE_CLOCK)) - 1) << 4)

#if HSI_VALUE * FUNCONF_PLL_MULTIPLIER != FUNCONF_SYSTEM_CORE_CLOCK
# if (CALCULATED_HPRE_DIV + 1) * HSI_VALUE == FUNCONF_SYSTEM_CORE_CLOCK
#   error Requested clock is not multiplication or division of 24MHz
# endif
# if CALCULATED_HPRE_DIV & ~0xF0
#   error Expected system core clock is too low
# endif
#endif


// Set UART baud rate here
// #define UART_BR 115200
#define UART_BR 9600

// DMA transfer completion interrupt. It will fire when the DMA transfer is
// complete. We use it just to blink the LED
// __attribute__((interrupt)) __attribute__((section(".srodata")))
// void DMA1_Channel4_IRQHandler(void)
// {
// 	// Clear flag
// 	DMA1->INTFCR |= DMA_CTCIF4;

// 	// Blink LED
// 	// GPIOD->OUTDR ^= 1<<LED_PIN
// }

// static void led_setup(void)
// {
// 	RCC->APB2PCENR = RCC_APB2Periph_GPIOD;
// 	GPIOD->CFGLR =
// 		((GPIO_CNF_IN_PUPD)<<(4*1)) | // Keep SWIO enabled.
// 		(GPIO_Speed_2MHz | GPIO_CNF_OUT_PP)<<(4*LED_PIN);

// 	// LED ON
// 	GPIOD->BSHR = 1<<LED_PIN;
// }
#define UART_SIMPLEX (1)
#define UART_DUPLEX (2)

#define USART_MODE UART_SIMPLEX

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
  printf("CMD: %s\n", cmd);
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
      printf("buf: %d '%c'\n", rx_buf[tail], rx_buf[tail]);
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

  // Debug purpose only
  int loops = 0;
  while(DMA1_Channel4->CFGR & DMA_CFGR1_EN)
  {
    ++loops;
  }
  printf("Took %d loops\n", loops);

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
  SystemInit();

#if 0
  printf("Awaiting DOOM\n");
  #define DOOM 10
  for (int i = 0; i < DOOM; ++i) {
    Delay_Ms(1000);
    printf("%d/%d\n", i + 1, DOOM);
  }
  printf("Doom\n");

  // Check if there is ANY reasons to run it clocks other than 8MHz
  // Running at native is supported by default
# if defined(FUNCONF_SYSTEM_CORE_CLOCK) && FUNCONF_SYSTEM_CORE_CLOCK != HSI_VALUE * (FUNCONF_PLL_MULTIPLIER + FUNCONF_USE_PLL)
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

  printf("Hello?\n");

#   if 0
  printf("Testing clock; each output should be printed in 1 second delay\n");
  for (int i = 0; i < DOOM; ++i) {
    Delay_Ms(1000);
    printf("TICK\n");
  }
#   endif /* 0 */
# endif /* core setup */
#endif /* 1 */

  uart_setup();
  dma_uart_setup();


// #define W_TEST_USART
#ifdef W_TEST_USART
  printf("poke\n");
  static const char message[] = "Hello World!\r\n";
	while (1)
	{
    printf("touch\n");
    // dma_uart_rx();
		dma_uart_tx(message, sizeof(message) - 1);
		Delay_Ms(1000);
  }

  char *text = "HelloWorld\n";
  while (1) {
    printf("emit to uart\n");
    char *buf = text;
    for (int i = 0; i < strlen(text);++i) {
      while( !(USART1->STATR & USART_FLAG_TC));
      Delay_Ms(2);
      USART1->DATAR = *buf++;
    }
    Delay_Ms(1000);
  }
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
  // spi_fun2();


  initializeGPIO();

  /* disable radio ASAP */
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
  init_pt100();

  DRIVER_TCA9548A_LINK_INIT(&tca9548a);
  DRIVER_TCA9548A_LINK_DELAY_MS(&tca9548a, libdriver_delay_ms);
  DRIVER_TCA9548A_LINK_IIC_READ(&tca9548a, tca9548a_iic_read);
  DRIVER_TCA9548A_LINK_IIC_WRITE(&tca9548a, tca9548a_iic_write);

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

  // const uint32_t oldtout = i2c.tout;
  // i2c.tout >> 2;
  // // while (1)
  // // {
  //   printf("Scan begin\n");
  //   i2c_scan(i2c_scan_callback);
  // //   Delay_Ms(1000);
  // // }
  // i2c.tout = oldtout;

  #if 0
  i2c.addr = 0x40;
  i2c.regb = 2;

  // Read HTU2x chipID
  // get Electronic Serial Number: SNA_3 CRC SNA_2 CRC SNA_1 CRC SNA_0 CRC
  uint8_t data[] = {0xFA, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  if (i2c_read_reg(&i2c, 0xFA0F, data, 8) == 0) {
  printf("%x %x %x %x %x %x %x %x\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  }

  // SNB_3 SNB_2 CRC SNB_1 SNB_0 CRC
  // SNB_3 - > device ID
  // 0x0D=13=Si7013
  // 0x14=20=Si7020
  // 0x15=21=Si7021
  // 0x32=50=HTU2x/SHT2x
  if (i2c_read_reg(&i2c, 0xFCC9, data, 6) == 0) {
  printf("%x %x %x %x %x %x\n", data[0], data[1], data[2], data[3], data[4], data[5]);
  }

  // not supported on htu2x
  // 0x84B8
  if (i2c_read_reg(&i2c, 0x84B8, data, 1) == 0) {
    printf("%x\n", data[0]);
  } else {
    printf("X\n");
  }

  i2c.regb = 1;
  // OK with delay
  #if 1
  i2c_clock_stretch(&i2c, true);

  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  // this is nor working as 'hold master'
  i2c.tout = 100000;
  if (i2c_read_reg_delay(&i2c, 0xE3, data, 3, 30)) {
    printf("FIAL\n");
  }
  #else
  while(1) {
    data[0] = 0xF3;
    if (i2c_write_raw(&i2c, &data[0], 1)) {
      printf("w failed\n");
    }
    Delay_Ms(80);
    if (i2c_read_raw(&i2c, data, 3) == 0) {
      printf("OK\n");
    }
  }

  data[0] = 0xF3;
  if (i2c_write_raw(&i2c, &data[0], 1)) {
    printf("w failed\n");
  }
  while(i2c_read_raw(&i2c, data, 3) != 0) {
    printf("nothing wait...\n");
    Delay_Ms(100);
  }
#endif
  printf("%x %x %x\n", data[0], data[1], data[2]);

#endif

#if 0
  // SHT3x works OK in "solo" mode
  while(1)
  {
    const uint32_t oldtout = i2c.tout;
    i2c.tout >> 2;
    // while (1)
    // {
      printf("Scan begin\n");
      i2c_scan(i2c_scan_callback);
    //   Delay_Ms(1000);
    // }
    i2c.tout = oldtout;

    for (int sidx = 0; sidx < supported_sensors_length; ++sidx) {
      supported_sensor_t *supported_sensor = supported_sensors + sidx;
      any_sensor_factory_t *ss =  supported_sensor->factory; // &sensor_factory_HTU21D;

      any_sensor_t *s = ss->construct(NULL);

      const char *sensor_name = sensor_to_str(supported_sensor->sensor);

      uint8_t result = s->probe(s);
      if (result)
      {
        printf("[%s] No sensor probed: %d\n", sensor_name, result);
        goto err;
      }

      uint32_t t;
      uint16_t h, p;
      obtain_t available = s->obtain(s, &t, &p, &h);
      if (available == OBTAIN_ERROR)
      {
        printf("[%s] obtain failed\n", sensor_name);
        goto err;
      }
      if (available & OBTAIN_TEMPERATURE) {
        printf("[%s] T: %05lu (FPT)\n", sensor_name, t);
      }
      if (available & OBTAIN_HUMIDITY) {
        printf("[%s] H: %05u (FPT)\n", sensor_name, h);
      }
      if (available & OBTAIN_PRESSURE) {
        printf("[%s] P: %05u (FPT)\n", sensor_name, p);
      }

      sidx = supported_sensors_length;

      err:

      if (ss->destroy) {
        printf("[%s] Dtroy\n", sensor_name);
        ss->destroy(s, NULL);
        printf("[%s] Dtroy done\n", sensor_name);
      }

      Delay_Ms(5000);
    }
  }
#endif
  
  
  // supported_sensor_t *ss = &supported_sensors[0];
  // i2c.addr = ss->factory->address;
  // any_sensor_t *s = ss->factory->construct(NULL);
  // s->obtain()

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

  /* prepare packet */
  hc12_wire_t *hdr = arena_alloc(&packet_arena, sizeof(hc12_wire_t));
  hdr->hdr.version_zero = 0;
  hdr->hdr.version = 2;
  hdr->payload.base.device_id = 0x03; /* that should be from MCU */
  hdr->payload.base.sensors_count = 1;
  hdr->payload.base.sync_byte = 'w';
  hdr->payload.base.version = 0x01;
  hdr->payload.compoint_sensor.sensor_id = 0xFF; /* TODO */


  /* safety, if someone will call deepsleep */
  Delay_Ms(2000);

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
    printf("try again\n");
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

  if (active == 0)
  {
    arena_clear(bank->arena);
  }
}

void acquire_pt100()
{
  if (max31865_single_read(&max31865, &tmp_raw_temperature16, NULL) == 0)
  {
    packet_put_reading(&packet, PACKET_BANK_PT100, 0, OBTAIN_TEMPERATURE, tmp_raw_temperature16);
    printf("pt100 reading ok: %d (raw)\n", tmp_raw_temperature16);
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

  uint8_t packet_size = sizeof(packet_pool);
  if (packet_to_bytes(&packet, packet_pool, &packet_size))
  {
    DPRINTF("unable to store packet in bytes");

    goto end;
  }

  // calculate crc8 on packet

  DPRINTF("waking up hc12\n");

  hc12_wake_up();

  DPRINTF("packet size: %d\n", packet_size);

  /* send it over uart */
  hc12_send(packet_pool, packet_size);

  DPRINTF("puting to sleep hc12\n");
  hc12_sleep();

  end:
  Delay_Ms(30000);
}
