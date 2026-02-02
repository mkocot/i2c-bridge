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
    void *memory;
    void *end;
    void *now;
};

inline void* arena_obtain(arena_t *arena, uint16_t size)
{
    if (arena->now + size > arena->end)
    {
        return NULL;
    }

    void *ptr = arena->now;
    arena->now += size;

    return ptr;
}

struct any_sensor_s;
typedef struct any_sensor_s any_sensor_t;
typedef uint8_t (*any_sensor_f_1)(any_sensor_t*);
typedef any_sensor_f_1 init_f;
typedef any_sensor_f_1 deinit_f;
typedef any_sensor_f_1 probe_f;

typedef enum obtain_e
{
  OBTAIN_ERROR = 0,
  OBTAIN_TEMPERATURE = 1 << 0,
  OBTAIN_PRESSURE = 1 << 1,
  OBTAIN_HUMIDITY = 1 << 2,
} obtain_t;

struct any_sensor_s {
  void *sensor;

  uint8_t (*init)(any_sensor_t *ctx);
  uint8_t (*deinit)(any_sensor_t *ctx);
  uint8_t (*probe)(any_sensor_t *ctx);
  /*
    0 -> error
    Returned value is flag of following values:
    1 - temperature
    2 - pressure
    4 - humidity
    8... - reserved for future

  */
  obtain_t (*obtain)(any_sensor_t *ctx, int32_t *temperature, uint16_t *pressure, uint16_t *humidity);
};

/* Dummy funcion for empty (De)Init functions */
static inline uint8_t sensor_noop(any_sensor_t *ctx)
{
    return 0;
}

#define SENSOR_INIT(INIT, PROBE, OBTAIN, DEINIT) {.sensor = NULL, .init = INIT, .deinit = DEINIT, .probe = PROBE, .obtain = OBTAIN }
#define SENSOR_INIT_ONLY(INIT, PROBE, OBTAIN) {.sensor = NULL, .init = INIT, .deinit = sensor_noop, .probe = PROBE, .obtain = OBTAIN }
#define DO_OR(F) if ((F)) { return 1; }; ((void)0)

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