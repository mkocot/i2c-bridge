#ifndef W_SENSOR_H
#define W_SENSOR_H

#include <memory.h>
#include <stdint.h>

// union any_sensor_u {
//   aht30_handle_t aht;
//   bmp280_handle_t bmp280;
// };

typedef struct arena_s arena_t;
struct arena_s {
    const void *memory;
    void *end;
    void *now;
};

inline uint8_t arena_init(arena_t *arena, void *pool, size_t size)
{
  arena->memory = pool;
  arena->end = arena->memory + size;
  arena->now = arena->memory;
}

inline void* arena_alloc(arena_t *arena, size_t size)
{
  if (arena->now + size >= arena->end)
  {
    return NULL;
  }

  const void* ptr = arena->now;
  arena->now += size;

  return ptr;
}

inline uint8_t arena_clear(arena_t *arena)
{
  arena->now = arena->memory;
}


struct any_sensor_s;
typedef struct any_sensor_s any_sensor_t;

typedef enum
{
  OBTAIN_ERROR = 0,
  OBTAIN_TEMPERATURE = 1 << 0,
  OBTAIN_PRESSURE = 1 << 1,
  OBTAIN_HUMIDITY = 1 << 2,

  /* aliases */
  OBTAIN_TH = OBTAIN_TEMPERATURE | OBTAIN_HUMIDITY,
  OBTAIN_TP = OBTAIN_TEMPERATURE | OBTAIN_PRESSURE,
} obtain_t;

struct any_sensor_s {
  /* private sensor data */
  void *sensor;

  /* check if sensor is valid and set configuration */
  uint8_t (*probe)(any_sensor_t *ctx);

  /* Read sensor data
    0 -> error
    Returned value is flag of following values:
    1 - temperature
    2 - pressure
    4 - humidity
    8... - reserved for future

  */
  obtain_t (*obtain)(any_sensor_t *ctx,
    int32_t *temperature,
    uint16_t *pressure,
    uint16_t *humidity
  );
};

typedef struct any_sensor_factory_s {
  const uint8_t address;
  any_sensor_t* (*construct)(arena_t *arena);
  void (*destroy)(any_sensor_t *ctx, arena_t *arena);
} any_sensor_factory_t;

/* Dummy funcion for empty (De)Init functions */
static inline uint8_t sensor_noop(any_sensor_t *ctx)
{
    return 0;
}

#define SENSOR_INIT(PROBE, OBTAIN) {.sensor = NULL, .probe = PROBE, .obtain = OBTAIN }
#define SENSOR_FACTORY(MODULE, A, C, D) \
  static any_sensor_factory_t sensor_factory_##MODULE = { \
    .address = A, .construct = C, .destroy = D \
  }

#define SENSOR_MODULE(MODULE, C, D, P, O) \
  SENSOR_FACTORY(MODULE, C, D); \
  static any_sensor_t sensor_##MODULE = SENSOR_INIT(P, O)

#define DO_OR(F) if ((F)) { return 1; }; ((void)0)
#define DO_ERR(F) if ((F)) { goto err; }; ((void)0)

#define generate_generic_probe(MODULE) \
  static uint8_t sensor_##MODULE##_probe(any_sensor_t *ctx) \
  { \
    ((void)ctx); \
    uint8_t init_state = MODULE.inited; \
    MODULE##_deinit(&MODULE); \
    uint8_t err = MODULE##_init(&MODULE); \
    MODULE.inited = init_state; \
    return err; \
  }

#endif