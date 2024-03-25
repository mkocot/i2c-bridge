#include <SensorProtocol.hpp>

#include <Arduino.h>
#include <Wire.h>
// #include <SimpleCLI.h>
// #include <SPI.h>
// #include <SPISlave.h>
#include <driver_aht20.h>
#include <interface/driver_aht20_interface.h>
#include <driver_bmp280.h>
#include <interface/driver_bmp280_interface.h>

#ifdef ARDUINO_ARCH_ESP8266
#define A6 (D5)
#define A7 (-1)
#define D11 (-1)
#define D12 (-1)
#define D13 (-1)
#endif
// Choosen becase they fit nicely on front of board
// [                D13]
// [                D12]
// [                D11]
// [A4  A5  A6  A7  D10]
// A4 - SDA
// A5 - SCL
// A6 - SCL_BANK_1 (bit-bang)
// A7 - SCL_BANK_2 (bit-bang)
// D10..D13 as more alternate I2C, they will nicely create long line
// one after another
constexpr auto SCL_BANK_0 = SCL;
constexpr auto SCL_BANK_1 = A6;
constexpr auto SCL_BANK_2 = A7;
constexpr auto SCL_BANK_3 = D10;
constexpr auto SCL_BANK_4 = D11;
constexpr auto SCL_BANK_5 = D12;
constexpr auto SCL_BANK_6 = D13;
static void fetch_values(float *temp, float *hum, float *pressure);

// Only D2 and D3 supports INT interrupts
// we might later choose PCINT if beneficial
constexpr auto CMD_PIN = D2;

extern int obtain(uint8_t *data)
{
  auto s = Serial.readBytes(data, 1);
  Serial.printf("obtain: %d %d\n", s, *data);
  return s;
}

constexpr auto AHT20_ADDRESS = 0x38;

aht20_handle_t *aht20{nullptr};
bmp280_handle_t *bmp280{nullptr};


uint8_t aht20_interface_iic_init()
{
  // NO-OP
  return 0;
}

uint8_t aht20_interface_iic_deinit()
{
  return 0;
}

uint8_t aht20_interface_iic_read_cmd(uint8_t addr, uint8_t *buf, uint16_t len)
{
  // For some obscure reasons, addr is shifted by 1 to right
  auto r = Wire.requestFrom((int)(addr >> 1), (int)len);
  auto d = Wire.readBytes(buf, len);
  return d != len;
}

uint8_t aht20_interface_iic_write_cmd(uint8_t addr, uint8_t *buf, uint16_t len)
{
  // For some obscure reasons, addr is shifted by 1 to right
  Wire.beginTransmission((addr >> 1));
  auto w = Wire.write(buf, len);
  Wire.endTransmission();
  return w != len;
}

void aht20_interface_delay_ms(uint32_t ms)
{
  delay(ms);
}

void aht20_interface_debug_print(const char *fmt, ...)
{
  // Serial.println(__FUNCTION__);
  // va_list args;
  // va_start(args, fmt);
  // Serial.printf(fmt, args);
  // va_end(args);
}

uint8_t bmp280_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  if (bmp280_interface_iic_write(addr, reg, nullptr, 0))
  {
    return -1;
  }

  auto r = Wire.requestFrom((int)(addr >> 1), (int)len);
  auto d = Wire.readBytes(buf, len);
  return d != len;
}

uint8_t bmp280_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  Wire.beginTransmission(addr >> 1);
  auto w = Wire.write(reg);
  if (len != 0) {
    w += Wire.write(buf, len);
  }
  Wire.endTransmission();
  return w != (len + 1u);
}

// I2C
//      ESP8266 LGT
// SDA  D2      A4
// SCL  D1      A5

static uint8_t noop(uint8_t x, uint8_t *xx, uint16_t xxx)
{
  return 0;
}

static SensorProtocol prot_handler;
static int aht_bank;


void setup()
{
  prot_handler.message_callback = [](auto result, auto cmd, auto buff, auto buff_len)
  {
    if (result != ProtocolParser::message_parse_result_t::OK || cmd != ProtocolParser::GET_SENSOR_DATA)
    {
      uint8_t err_msg[] = {0x01 /* length 1*/, 0x02 /* id RET_ERROR */, 0x17 /* CRC8 */};
      Serial.write(err_msg, sizeof(err_msg));
      Serial.flush();
      return;
    }

    struct message_frame
    {
      uint8_t msg_len = sizeof(float) * 3 + 1;
      uint8_t msg_id = ProtocolParser::RET_SENSOR_DATA;
      float temp;
      float hum;
      float pres;
      uint8_t crc8;
    } __attribute__((packed));

    static_assert(sizeof(message_frame) == 15, "X");

    float temp, hum, pres;
  
    fetch_values(&temp, &hum, &pres);

    message_frame tmp = {
      .temp = temp,
      .hum = hum,
      .pres = pres,
    };
    tmp.crc8 = calc_crc8(reinterpret_cast<uint8_t*>(&tmp), sizeof(tmp) - 1);

    Serial.write(reinterpret_cast<uint8_t*>(&tmp), sizeof(tmp));
    Serial.flush();
  };
  Serial.begin(9600);
  while(!Serial) {
    delay(100);
  }
  delay(2000);

  Serial.println("Start detecting sensors");

  Serial.println("Detecting AHTx0 at BANK1 ADDR");
  aht20 = new aht20_handle_t;// Adafruit_AHTX0();
  DRIVER_AHT20_LINK_INIT(aht20, aht20_handle_t);
  DRIVER_AHT20_LINK_IIC_INIT(aht20, aht20_interface_iic_init);
  DRIVER_AHT20_LINK_IIC_DEINIT(aht20, aht20_interface_iic_deinit);
  DRIVER_AHT20_LINK_IIC_READ_CMD(aht20, aht20_interface_iic_read_cmd);
  DRIVER_AHT20_LINK_IIC_WRITE_CMD(aht20, aht20_interface_iic_write_cmd);
  DRIVER_AHT20_LINK_DELAY_MS(aht20, aht20_interface_delay_ms);
  DRIVER_AHT20_LINK_DEBUG_PRINT(aht20, aht20_interface_debug_print);

  // Detecting AHT on alternate BANK
  aht_bank = SCL_BANK_1;
  Wire.begin(SDA, aht_bank);

  bool is_ok = false;
  if (aht20_init(aht20))
  {
    aht_bank = SCL_BANK_0;
    Wire.begin(SDA, aht_bank);
    Serial.println("No AHTx0 at BANK1");
  } else {
    Serial.println("AHTx0 detected at BANK1");
    is_ok = true;
  }

  if (!is_ok && aht20_init(aht20) != 0)
  {
    aht20_deinit(aht20);
    Serial.println("AHTx0 NOT detected on BANK0");
    delete aht20;
    aht20 = nullptr;
  }
  else
  {
    Serial.println("AHTx0 detected");
  }


  Wire.begin(SDA, SCL_BANK_0);

  bmp280 = new bmp280_handle_t;
  DRIVER_BMP280_LINK_INIT(bmp280, bmp280_handle_t);

  DRIVER_BMP280_LINK_IIC_INIT(bmp280, aht20_interface_iic_init);
  DRIVER_BMP280_LINK_IIC_DEINIT(bmp280, aht20_interface_iic_deinit);
  DRIVER_BMP280_LINK_IIC_READ(bmp280, bmp280_interface_iic_read);
  DRIVER_BMP280_LINK_IIC_WRITE(bmp280, bmp280_interface_iic_write);

  DRIVER_BMP280_LINK_SPI_INIT(bmp280, aht20_interface_iic_init);
  DRIVER_BMP280_LINK_SPI_DEINIT(bmp280, aht20_interface_iic_deinit);
  DRIVER_BMP280_LINK_SPI_READ(bmp280, noop);
  DRIVER_BMP280_LINK_SPI_WRITE(bmp280, noop);

  DRIVER_BMP280_LINK_DELAY_MS(bmp280, aht20_interface_delay_ms);
  DRIVER_BMP280_LINK_DEBUG_PRINT(bmp280, aht20_interface_debug_print);
  bmp280_set_interface(bmp280, BMP280_INTERFACE_IIC);
  // confising name, its not PIN as PIN but just i2c address...
  // anyway its wrong, because someone tought shifting it to right by
  // one byte is flawless idea
  bmp280_set_addr_pin(bmp280, BMP280_ADDRESS_ADO_HIGH);
  auto fail_code = bmp280_init(bmp280);
  if (fail_code != 0)
  {
    Serial.print("BMP280 NOT detected: ");
    Serial.println(fail_code);
    bmp280_deinit(bmp280);
    delete bmp280;
  }
  else
  {
    Serial.println("Detected BMP280 at default ADDR");
    bmp280_set_mode(bmp280, BMP280_MODE_FORCED);
    bmp280_set_pressure_oversampling(bmp280, BMP280_OVERSAMPLING_x1);
    bmp280_set_temperatue_oversampling(bmp280, BMP280_OVERSAMPLING_x1);
    bmp280_set_filter(bmp280, BMP280_FILTER_OFF);
  }
}

static void fetch_values(float *temp, float *hum, float *pressure)
{
  *temp = NAN;
  *hum = NAN;
  uint32_t temp_raw, humidity_raw;
  Wire.begin(SDA, SCL_BANK_0);
  if (bmp280 != nullptr && bmp280_read_temperature_pressure(bmp280, &temp_raw, temp, &humidity_raw, pressure) == 0)
  {
  }
  else
  {
    *pressure = NAN;
  }


  Wire.begin(SDA, aht_bank);
  uint8_t throwaway;
  if (aht20 != nullptr && aht20_read_temperature_humidity(aht20, &temp_raw, temp, &humidity_raw, &throwaway) == 0)
  {
    // for unknown reason humidity is calculated with hiher precission, but then it's cramed into uint8_t
    // this is code from aht20_read_temperature_humidity
    *hum = (static_cast<float>(humidity_raw) / 1048576.0f * 100.0f);               /* convert the humidity */
  }

  Serial.print("Data: ");
  Serial.print(*temp);
  Serial.print(" ");
  Serial.print(*hum);
  Serial.print(" ");
  Serial.println(*pressure);
}

time_t last_forced;
void loop()
{
  if (Serial.available() > 0) {
    prot_handler.has_data();
  } else {
    prot_handler.tick();
  }

  if (millis() - last_forced > 10000) {
    last_forced = millis();
    float a, b, c;
    fetch_values(&a, &b, &c);
  }

  #if 0
  if (bmp280)
  {
    bmp280->takeForcedMeasurement();
    float temp = bmp280->readTemperature();
    float pressure = bmp280->readPressure();
    Serial.print("BMP280: temp=");
    Serial.print(temp);
    Serial.print(" pressure=");
    Serial.println(pressure);
  }

  if (ahtx0)
  {
    sensors_event_t hum, temp;
    ahtx0->getEvent(&hum, &temp);
    Serial.print("AHTx0: temp=");
    Serial.print(temp.temperature);
    Serial.print(" hum=");
    Serial.println(hum.relative_humidity);
  }
  #endif
}

// volatile uint8_t cmd_initialized = 0;

// static void enable_bank(uint8_t bank_id);
// static void disable_bank(uint8_t bank_id);
// static bool in_cmd_mode();

// static void scan_i2c();
// static void check_command();

// // ISR function
// static void int_exit_cmd_mode();

// STATE  INPUT
// INIT   'R'ead
// INIT   'W'rite
// INIT   'E'nable
// INIT   'D'isable

// Based on Adafruit BusIo, following methods are used:
// wire->begin(); // initialize protocol this can be used only on our side without exposing
// wire->end(); // beut is never called, keept it on our side
// wire->beginTransmission(addr)
// wire->endTransmission()
// wire->write(buffer, len)
// wire->requestFrom (without begin/end)
// wire->read()
// wire->setClock
// Prefix all messages
// B[A} - 2 bytes BEGIN + ADDRESS
// E    - 1 byte END
// W[L][DATA] - 1 byte W + 1 byte L + L bytes
// R[A + S][L] - 1 byte R, A 7 bits address + 1 bit stop, L 1 byte length
// C[CK] - 1 byte C, 2 bytes speed

// Notes: write should always be betwwen begin() and end()
// begin and end is transaction that should not be violated, and requests 'mid fly' should return error
// end is optional so begin() will always start new transaction

// enum {
// STATE_INIT = FSM_STATE_INIT,
// STATE_ACTIVE,
// STATE_READ_FIRST_LETTER,

// STATE_ACTIVE_ID,

// STATE_READ_LENGTH,

// STATE_WRITE_LENGTH,

// STATE_ENABLE_ID,

// STATE_DISABLE_ID,

// STATE_GET_BANK_ARG,
// STATE_GET_ARG,
// STATE_GET_ADDRESS_ARG,

// STATE_FAIL = FSM_STATE_FAIL,
// };

// enum {
// EVENT_RESET = 0x0100,
// EVENT_DATA_AVAILABLE,
// };

// struct state_t {
//   uint8_t scl_banks = 0; // bitmap of active ports

//   // 1) BANK
//   // 2) ADDRESS
//   // We can assume ADDRESS > 127 -> disabled
//   uint8_t scl_current = 0;
//   uint8_t scl_direction = 0; // 0 - disabled, 1 - read, 2 write
//   uint8_t scl_amount = 0;
//   uint8_t scl_address = 0;
//   uint8_t data;
//   #define cmd_initialized cmd_initialized
//   #undef cmd_initialized
// };

// static int accept_a(fsm_event_t event)
// {
//   return event == 'A';
// }

// static int accept_r(fsm_event_t event) { return event == 'R'; }
// static int accept_w(fsm_event_t event) { return event == 'W'; }
// static int accept_e(fsm_event_t event) { return event == 'E'; }
// static int accept_d(fsm_event_t event) { return event == 'D'; }
// static int accept_bank_id(fsm_event_t event) {
//   return event >= 0 && event < 7;
// }
// static int accept_any(fsm_event_t event) {
//   return 1;
// }

// static void set_active_bank(struct fsm_t *fsm) {
//   struct state_t *state = (struct state_t*)fsm_private(fsm);
//   state->scl_current = fsm_current_event(fsm);
// }

// static void set_amount_read(struct fsm_t *fsm) {
//   struct state_t *state = (struct state_t*)fsm_private(fsm);
//   state->scl_direction = 1;
//   state->scl_amount = fsm_current_event(fsm);
// }

// static void set_amount_write(struct fsm_t *fsm) {
//   struct state_t *state = (struct state_t*)fsm_private(fsm);
//   state->scl_direction = -1;
//   state->scl_amount = fsm_current_event(fsm);
// }

// static void enable_bank(struct fsm_t *fsm) {
//   enable_bank(fsm->event);
// }

// static void disable_bank(struct fsm_t *fsm) {
//   disable_bank(fsm->event);
// }

// static void reset_state() {
// }
// static int get_event(fsm_event_t *event);

// static fsm_state_t read_command(struct fsm_t *fsm) {
//   struct state_t *state = (struct state_t*)fsm_private(fsm);
//   state->data = Serial.read();
//   switch(state->data) {
//     case 'R':
//       state->scl_direction = 1;
//       return STATE_GET_ARG;
//     case 'W':
//       state->scl_direction = -1;
//       return STATE_GET_ARG;
//     case 'A':
//     case 'E':
//     case 'D':
//     return STATE_GET_BANK_ARG;
//   }

//   return STATE_INIT;
// }

// static fsm_state_t read_arg(struct fsm_t *fsm) {
//   struct state_t *state = (struct state_t*)fsm_private(fsm);
//   state->scl_amount = Serial.read();
//   return STATE_INIT;
// }

// static fsm_state_t read_address_arg(struct fsm_t *fsm) {
//   auto address = Serial.read();
//   if (address < 0 || address >= 127) {
//     return STATE_INIT;
//   }

//   struct state_t *state = (struct state_t*)fsm_private(fsm);

//   state->scl_address = address;
// }

// static fsm_state_t read_bank_arg(struct fsm_t *fsm) {
//   auto bank_id = Serial.read();
//   if (bank_id < 0 || bank_id >= 7) {
//     return STATE_INIT;
//   }

//   // execute action
//   struct state_t *state = (struct state_t*)fsm_private(fsm);
//   if (state->data == 'A') {
//     state->scl_current = bank_id;
//     state->scl_direction = 0;
//     state->scl_address = 0xFF;
//     return STATE_GET_ADDRESS_ARG;
//   } else if (state->data == 'E') {
//     enable_bank(bank_id);
//   } else if (state->data == 'D') {
//     disable_bank(bank_id);
//   } else {
//     // unknown
//   }
//   return STATE_INIT;
// }

// static void read_byte()
// {
//   struct state_t *state = (struct state_t*)fsm_private(fsm);
//   switch(state->data)
//   {

//   }

//   return STATE_INIT;
// }
// // INIT -EVENT_DATA-> READ_CMD
// // READ_CMD -> READ_ANY
// fsm_row_t states[] = {
//   {STATE_INIT, EVENT_DATA_AVAILABLE, read_command},
//   {STATE_GET_ARG, EVENT_DATA_AVAILABLE, read_arg},
//   {STATE_GET_BANK_ARG, EVENT_DATA_AVAILABLE, read_bank_arg},
//   {STATE_GET_ADDRESS_ARG, EVENT_DATA_AVAILABLE, read_address_arg},

//   // {STATE_INIT,    accept_a, NULL, STATE_ACTIVE_ID},
//   // {STATE_ACTIVE,  accept_r, NULL, STATE_READ_LENGTH},
//   // {STATE_ACTIVE,  accept_w, NULL, STATE_WRITE_LENGTH},

//   // {STATE_INIT,    accept_e, NULL, STATE_ENABLE_ID},
//   // {STATE_ACTIVE,  accept_e, NULL, STATE_ENABLE_ID},
//   // {STATE_INIT,    accept_d, NULL, STATE_DISABLE_ID},
//   // {STATE_ACTIVE,  accept_d, NULL, STATE_DISABLE_ID},

//   // {STATE_ACTIVE_ID, accept_bank_id, set_active_bank, STATE_ACTIVE},

//   // {STATE_READ_LENGTH, NULL, set_amount_read, STATE_ACTIVE},

//   // {STATE_WRITE_LENGTH, NULL, set_amount_write, STATE_ACTIVE},

//   // {STATE_ENABLE_ID, NULL, enable_bank, STATE_INIT},

//   // {STATE_DISABLE_ID, NULL, disable_bank, STATE_INIT},

//   // // Failures (added last due to catch-all)
//   // {STATE_ACTIVE, NULL, NULL, STATE_INIT},
//   // {STATE_ACTIVE_ID, NULL, NULL, STATE_INIT},
//   // {STATE_READ_LENGTH, NULL, NULL, STATE_INIT},
//   // {STATE_WRITE_LENGTH, NULL, NULL, STATE_INIT},
//   // {STATE_ENABLE_ID, NULL, NULL, STATE_INIT},
//   // {STATE_DISABLE_ID, NULL, NULL, STATE_INIT}
// };

#if 0
enum fsm_state : uint8_t
{
  STATE_B = 'B',
  STATE_E = 'E',
  STATE_W = 'W',
  STATE_R = 'R',
  STATE_C = 'C',
  STATE_INIT = '0',
  STATE_INITIALIZED = '1',
};

struct fsm_s
{
  uint8_t state;
  union
  {
    uint8_t bits;
    uint8_t on_enter : 1;
  } flags;

  // data
  struct
  {
    uint8_t address;
    uint8_t bank;
    uint8_t remaining;
    uint16_t clock;
    union
    {
      int has_address : 1;
      int bits;
    } flags;
  } data;
};

typedef struct fsm_s *fsm_t;

void fsm_init(fsm_t fsm)
{
  fsm->state = STATE_INIT;
}

void fsm_fail(fsm_t fsm)
{
  Serial.println("#KO");

  // Wire.end();
  fsm->state = STATE_INIT;
  fsm->flags.bits = 0;
  memset(&fsm->data, 0, sizeof(fsm->data));
}

void fsm_event(fsm_t fsm, uint8_t event)
{
  if (fsm->state == STATE_INITIALIZED)
  {
    switch (event)
    {
    case STATE_E:
    case STATE_W:
    case STATE_R:
    case STATE_C:
      fsm->state = event;
      break;
    default:
      fsm_fail(fsm);
      break;
    }
  }
  else if (fsm->state == STATE_INIT)
  {
    if (event == STATE_B)
    {
      fsm->state = STATE_B;
      fsm->data.flags.bits = 0;
    }
    // else go to init
  }
  else if (fsm->state == STATE_B)
  {
    if (fsm->data.flags.has_address || (!fsm->data.flags.has_address && event <= 0x7F))
    {
      // unexpected request, move to init
      fsm_fail(fsm);
    }
    else
    {
      fsm->data.flags.has_address = true;
      fsm->flags.on_enter = true;
      fsm->data.address = event;
      fsm->state = STATE_INITIALIZED;

      Wire.begin();
      Serial.println("#OK");
    }
  }
  else if (fsm->state == STATE_E)
  {
    // TODO: finish current wire communication and move to init
    fsm->state = STATE_INIT;
    Serial.println("#OK");
  }
  else if (fsm->state == STATE_W)
  {
    if (fsm->flags.on_enter)
    {
      fsm->flags.on_enter = false;
      if (event > 0)
      {
        fsm->data.remaining = event;
      }
      else
      {
        // invlid argument go back to init
        fsm_fail(fsm);
      }
    }
    else
    {
      while(fsm->data.remaining) {
        if (Wire.write(&event, 1) != 1) {
          break;
        }
      }
      // reading done,
      if (fsm->data.remaining == 0) {
        Serial.println("#OK");
        fsm->state = STATE_INITIALIZED;
      } else {
        fsm_fail(fsm);
      }
    }
  }
  else if (fsm->state == STATE_R)
  {
    if (event > 0)
    {
      fsm->data.remaining = event;
      while (fsm->data.remaining--)
      {
        // TODO: read and respond
        uint8_t raw_byte;
        if (Wire.readBytes(&raw_byte, 1) == 1) {
          if (Serial.write(&raw_byte, 1) != 1) {
            break;
          }
        } else {
          break;
        }
      }
      if (fsm->data.remaining == 0) {
        Serial.println("#OK");
        fsm->state = STATE_INITIALIZED;
      } else {
        fsm_fail(fsm);
      }
    }
    else
    {
      fsm_fail(fsm);
    }
  }
  else if (fsm->state == STATE_C)
  {
    if (fsm->flags.on_enter)
    {
      fsm->flags.on_enter = false;
      fsm->data.clock = event;
    }
    else
    {
      fsm->data.clock = fsm->data.clock << 8 | event;
      Wire.setClock(fsm->data.clock);
      Serial.println("#OK");
    }
    fsm->state = STATE_INITIALIZED;
  }
  else
  {
    // unknown code, go back to init
    fsm_fail(fsm);
  }
}


struct LLEntry {
  
};

static void handle_init_sensor(cmd *cmd)
{
  // TYPE BMP280, BPM280, SHT30, SHT40, BMPSHT,
  // BMPSHT combo with SHT30 AND BMP280
  // BANK (optional, default 0)
  // ADDRESS (optional, default whatever sensor default is)

  // Returns ID of sensor, to be used with `fetch_sensor` command
  const Command c{cmd};
  c.getArg("type");
  c.getArg("bank");
  c.getArg("address");
}

static void handle_deinit_sensor(cmd *cmd)
{
  const Command c{cmd};
  c.getArg("id");
}

static void handle_sensor_read(cmd *cmd)
{
  // returned format is always
  // 4 bytes (float)  TEMPERATURE
  // 4 bytes (uint32) PRESSURE
  // 2 bytes (uint16) HUMIDITY

  // oxFF... is INVALID and marks MISSING data
  const Command c{cmd};
  c.getArg("id");
}

static void handle_fetch_sensors(cmd *cmd)
{
  // 
}

SimpleCLI cmds{};

static void setup_cmds()
{
  auto cmd = cmds.addCmd("init_sensor", handle_init_sensor);
  cmd.addArg("type");
  cmd.addArg("bank");
  cmd.addArg("address");

  cmd = cmds.addCmd("deinit_sensor", handle_deinit_sensor);
  cmd.addArg("id");

  cmd = cmds.addSingleArgCmd("fetch_sensor", handle_sensor_read);
  cmd.addArg("id");

  cmd = cmds.addCmd("fetch_sensors");
}
#endif

#if 0
static void spi_received (uint8_t *data, size_t len)
{
  // Register 208 -> return 
  Serial.printf("SPI data: len=%d\n", len);
}

static void spi_data_transferred()
{
  Serial.println("SPI data transferred");
}

static void spi_status_update(uint32_t status)
{
  Serial.printf("SPI status: %d\n", status);
}

static void spi_status_transferred()
{
  Serial.println("SPI status transferred");
}
void setup()
{
  // NOTE: There is no option to change SDA, SCL pins for HW I2C
  Serial.begin(9600);
  while (!Serial)
  {
    // WAIT
    delay(1);
  }
  delay(2000);
  // Enable Wire
  // Wire.begin();

  Serial.println("#OPERATIONAL");

  // D5 - CLK
  // D6 - MISO
  // D7 - MOSI
  // D8 - CS
  #if 0
  SPISlave.begin();
  SPISlave.onData(spi_received);
  SPISlave.onDataSent(spi_data_transferred);
  SPISlave.onStatus(spi_status_update);
  SPISlave.onStatusSent(spi_status_transferred);
  #endif
}

void loop()
{
  // if (!Serial.available())
  // {
  //   yield();
  //   return;
  // }

  // Serial.read();
}

  #endif
#if 0
void scan_i2c()
{
  int nDevices = 0;

  Serial.println("Scanning...");

  for (byte address = 1; address < 127; ++address)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("  !");

      ++nDevices;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
      {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
  {
    Serial.println("No I2C devices found\n");
  }
  else
  {
    Serial.println("done\n");
  }
}
#endif