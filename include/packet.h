#ifndef W_PACKET_H
#define W_PACKET_H

#include <stdint.h>
#include <stdio.h>
#include <sensor.h>

#define PACKET_BANK_PT100 (-1)

#define STATIC_ASSERT(COND) typedef char static_assertion_##__LINE__[(COND)?1:-1]

typedef union {
  uint8_t raw;
  struct {
    uint8_t count:4;
    uint8_t reserved:4;
  };
} reading_config_t;

STATIC_ASSERT(sizeof(reading_config_t) == 1);

typedef union {
  reading_config_t base_config;
  struct {
    uint8_t count:4;
    uint8_t has_pt100:1;
    uint8_t reserved:3;
  };
} temperature_config_t;

STATIC_ASSERT(sizeof(reading_config_t) == 1);

typedef union {
  uint16_t raw;
  struct {
    union {
      uint8_t lower;
    struct {
    uint8_t value_0_bank:2;
    uint8_t value_1_bank:2;
    uint8_t value_2_bank:2;
    uint8_t value_3_bank:2;
    };
  };
  union {
    uint8_t upper;
    struct {
    uint8_t value_4_bank:2;
    uint8_t value_5_bank:2;
    uint8_t value_6_bank:2;
    uint8_t value_7_bank:2;
    };
  };
};
} used_banks_t;

STATIC_ASSERT(sizeof(used_banks_t) == 2);

typedef struct {
  /* temperature */
  temperature_config_t temperature_config;
  used_banks_t temperature_used_banks;
  uint16_t temperatures[8]; /* linear scale */
  uint16_t temperature_pt100; /* linear scale */

  /* pressure */
  reading_config_t pressure_config;
  used_banks_t pressure_used_banks;
  int16_t pressures[8]; /* linear scale */

  /* humidity */
  reading_config_t humidity_config;
  used_banks_t humidity_used_banks;
  int8_t humiditys[8]; /* linear scale */

} packet_t;

static void packet_put_reading(packet_t *packet, int bank, int sensor_idx, obtain_t sensor, uint16_t value);
static void packet_clear_readings(packet_t *packet);

static inline void packet_set_used_bank(packet_t *packet, obtain_t sensor, uint8_t index, uint8_t bank)
{
  // TODO: Bits may be set other way around, from end not start...
  uint16_t *ptr;
  if (sensor == OBTAIN_TEMPERATURE)
  {
    ptr = &packet->temperature_used_banks.raw;
  }
  else if (sensor == OBTAIN_PRESSURE)
  {
    ptr = &packet->pressure_used_banks.raw;
  }
  else if (sensor == OBTAIN_HUMIDITY)
  {
    ptr = &packet->humidity_used_banks.raw;
  }
  else
  {
    /* invalid value */
    return;
  }

  *ptr |= (0b11 & bank) << (2 * index);
}

void packet_put_reading(packet_t *packet, int bank, int sensor_idx, obtain_t sensor, uint16_t value)
{
  used_banks_t *used_banks;
  reading_config_t *config;

  if (sensor == OBTAIN_TEMPERATURE)
  {
    if (bank == PACKET_BANK_PT100) /* handle PT100 */
    {
      packet->temperature_config.has_pt100 = 1;
      packet->temperature_pt100 = value;

      return;
    }

    used_banks = &packet->temperature_used_banks;
    config = &packet->temperature_config.base_config;

    packet->temperatures[config->count] = value;
  }
  else if (sensor == OBTAIN_PRESSURE)
  {
    used_banks = &packet->pressure_used_banks;
    config = &packet->pressure_config;

    packet->pressures[config->count] = value;
  }
  else if (sensor == OBTAIN_HUMIDITY)
  {
    used_banks = &packet->humidity_used_banks;
    config = &packet->humidity_config;

    packet->humiditys[config->count] = (uint8_t)value;
  }
  else
  {
    // ERROR
  }

  packet_set_used_bank(packet, sensor, config->count, bank);
  ++config->count;
}

void packet_clear_readings(packet_t *packet)
{
  packet->humidity_config.count = 0;
  packet->pressure_config.count = 0;
  packet->temperature_config.count = 0; 
  packet->temperature_config.has_pt100 = 0;
}

static uint8_t packet_sensor_readings(const packet_t *packet)
{
  return packet->humidity_config.count
    + packet->pressure_config.count
    + packet->temperature_config.count
    + packet->temperature_config.has_pt100;
}

#define PACKET_TOO_SMALL 1
#define PACKET_OK 0
#define PACKET_NO_STORAGE 2

#define _banks_expected_size(wat) \
  ((wat.count != 0) * (1 + (wat.count > 4)))

#define put_data(ptr, data, size) (memcpy(ptr, data, (size)) + (size))

static int packet_to_bytes(packet_t *packet, void *bytes, uint8_t *size)
{
  // Packing is little bit convoluted
  // 0) each of "chunks" are encoded simmilary
  // 1) store header (1 byte) with number of reading
  // 2) when count is <= 4 store only "first half" of bank config (1 byte)
  //    otherwise store full bank config (2 bytes)
  // 3) Store up-to count of sensor reading
  // 4) for temperature omit pt100 if not present

  // calculate expected size
  uint8_t expected_size = 3 /* headers */
  /* temperature */
  + _banks_expected_size(packet->temperature_config) /* banks configuration */
  + packet->temperature_config.count * sizeof(packet->temperatures[0]) + packet->temperature_config.has_pt100 /* raw readings */

  // /* pressure */
  + _banks_expected_size(packet->pressure_config)
  + packet->pressure_config.count * sizeof(packet->pressures[0])

  // /* humidity */
  + _banks_expected_size(packet->humidity_config)
  + packet->humidity_config.count * sizeof(packet->humiditys[0])
  ;

  uint8_t storage_size = *size;
  *size = expected_size;

  if (bytes == NULL)
  {
    /* there is no bytes, so just calculate size*/
    return PACKET_NO_STORAGE;
  }

  if (expected_size > storage_size)
  {
    /* too small */
    return PACKET_TOO_SMALL;
  }

  void *ptr = bytes;

  ptr = put_data(ptr, &packet->temperature_config, sizeof(packet->temperature_config));
  if (packet->temperature_config.count)
  {
    ptr = put_data(ptr, &packet->temperature_used_banks.raw, _banks_expected_size(packet->temperature_config));
    ptr = put_data(ptr, packet->temperatures, sizeof(packet->temperatures[0]) * packet->temperature_config.count);
  }

  if (packet->temperature_config.has_pt100)
  {
    ptr = put_data(ptr, &packet->temperature_pt100, sizeof(packet->temperature_pt100));
  }

  ptr = put_data(ptr, &packet->pressure_config, sizeof(packet->pressure_config));
  if (packet->pressure_config.count)
  {
    ptr = put_data(ptr, &packet->pressure_used_banks.raw, _banks_expected_size(packet->pressure_config));
    ptr = put_data(ptr, packet->pressures, sizeof(packet->pressures[0]) * packet->pressure_config.count);
  }

  ptr = put_data(ptr, &packet->humidity_config, sizeof(packet->humidity_config));
  if (packet->humidity_config.count)
  {
    ptr = put_data(ptr, &packet->humidity_used_banks.raw, _banks_expected_size(packet->humidity_config));
    ptr = put_data(ptr, packet->humiditys, sizeof(packet->humiditys[0]) * packet->humidity_config.count);
  }

  return 0;
}

#undef _banks_expected_size
#undef put_data

#endif
