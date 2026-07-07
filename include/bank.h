#ifndef I2C_BRIDGE_BANK_H
#define I2C_BRIDGE_BANK_H

#include <driver_tca9548a.h>
#include <sensor.h>


typedef enum
{
  SENSOR_NONE = 0,
  SENSOR_AHT = 1 << 0,
  SENSOR_BMP280 = 1 << 1,
  SENSOR_BME280 = 1 << 2,
  SENSOR_SHT3X = 1 << 3,
  SENSOR_SHT4X = 1 << 4,
  SENSOR_SHTC3 = 1 << 5,
  SENSOR_HTU21D = 1 << 6,
  SENSOR_HTU31D = 1 << 7,
  SENSOR_SI7021 = 1 << 8,
  SENSOR_HDC1080 = 1 << 9,
  SENSOR_MCP9808 = 1 << 10,
} sensor_t;

static const char* sensor_to_str(sensor_t sensor)
{
  switch (sensor)
  {
    case SENSOR_NONE:
      return "none";
    case SENSOR_AHT:
      return "ahtxx";
    case SENSOR_BMP280:
      return "bmp280";
    case SENSOR_BME280:
      return "bme280";
    case SENSOR_SHT3X:
      return "sht3x";
    case SENSOR_SHT4X:
      return "sht4x";
    case SENSOR_SHTC3:
      return "shtc3";
    case SENSOR_HTU21D:
      return "htu21d";
    case SENSOR_HTU31D:
      return "htu31d";
    case SENSOR_SI7021:
      return "si7021";
    case SENSOR_HDC1080:
      return "hdc1080";
    case SENSOR_MCP9808:
      return "mcp9808";
    default:
      return "unknown";
  }
}

typedef struct {
  sensor_t type;
  any_sensor_t *sensor;
} active_sensor_t;

#define BANK_MAX_SENSORS (2)

typedef struct
{
  tca9548a_channel_t channel;
  // If sensor is present then it was inited before, otherwise initialization
  // is required before acquiring any data
  // NULL or real, there is at-most 2 sensors per bank
  active_sensor_t sensors[BANK_MAX_SENSORS];
  arena_t *arena;
} bank_t;

static uint8_t bank_active_count(const bank_t *bank)
{
  uint8_t active = 0;
  for (int i = 0; i < BANK_MAX_SENSORS; ++i)
  {
    const active_sensor_t *sensor = &bank->sensors[i];
    if (sensor->type == 0)
    {
      continue;
    }

    ++active;
  }

  return active;
}

/* 0xFF if not found */
static uint8_t bank_has_sensor(const bank_t *bank, sensor_t type)
{
  for (int i = 0; i < BANK_MAX_SENSORS; ++i)
  {
    const active_sensor_t *sensor = &bank->sensors[i];
    if (sensor->type == type)
    {
      return i;
    }
  }

  return 0xFF;
}

static sensor_t bank_active_sensors(const bank_t *bank)
{
  sensor_t sensors = SENSOR_NONE;
  for (int i = 0; i < BANK_MAX_SENSORS; ++i)
  {
    sensors |= bank->sensors[i].type;
  }

  return sensors;
}

#endif
