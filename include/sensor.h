#ifndef W_SENSOR_H
#define W_SENSOR_H

#include <memory.h>
#include <stdint.h>
#include <fptc.h>

#define PR_FPT "%d.%02d"
#define F2PRINTF(F) ((int)F), (int)((F - (int)F) * 100)

static void print_float(float f) {
  int int_part = (int)f;
  int frac_part = (int)((f - int_part) * 100);  // 2 decimal places
  printf("%d.%02d", int_part, frac_part);
}

typedef fpt temperature_t;
typedef fpt humidity_t;
typedef fpt pressure_t;

#define PR_FPT "%d.%02d"

static void print_fpt(fpt v)
{
  int int_part = fpt2i(v);
  int frac_part = fpt2i(fpt_mul(fpt_sub(v, i2fpt(int_part)), i2fpt(100)));
  printf(PR_FPT, int_part, frac_part);
}

#define PR_TEMP PR_FPT
#define print_temperature(t) print_fpt(t)

#define PR_HUM PR_FPT
#define print_humidity(t) print_fpt(t)

#define PR_PR PR_FPT
#define print_pressure(t) print_fpt(t)

// union any_sensor_u {
//   aht30_handle_t aht;
//   bmp280_handle_t bmp280;
// };

typedef struct arena_s arena_t;
struct arena_s {
    const void *memory;
    const void *end;
    void *now;
};

#define ARENA_INIT(POOL, SIZE) {(POOL), (POOL) + (SIZE), (POOL)}

inline static uint8_t arena_init(arena_t *arena, void *pool, size_t size);

inline static void* arena_alloc(arena_t *arena, size_t size);

inline static uint8_t arena_clear(arena_t *arena);


uint8_t arena_init(arena_t *arena, void *pool, size_t size)
{
  arena->memory = pool;
  arena->end = pool + size;
  arena->now = pool;

  return 0;
}

void* arena_alloc(arena_t *arena, size_t size)
{
  if (arena->now + size >= arena->end)
  {
    // printf("%u %d %u\n", (unsigned int)arena->now, size, (unsigned int)arena->end);
    return NULL;
  }

  void* ptr = arena->now;
  arena->now += size;

  return ptr;
}

uint8_t arena_clear(arena_t *arena)
{
  arena->now = (void*) arena->memory;

  return 0;
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
    temperature_t *temperature,
    pressure_t *pressure,
    humidity_t *humidity
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

#define MIN(A, B) ((A) < (B) ? (A) : (B))
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#define MINMAX(A, B, C) \
    ((C) < (A) ? (A) : (C) > (B) ? (B) : (C))

#if !defined(FPT_WBITS) || FPT_WBITS != 16
#error FPT_WBITS should be defined to 16 bits!
#endif

#define Q_RANGE(FROM, TO) ((TO) - (FROM) + 1)

#define T_MIN -40
#define T_MAX 85
#define TQ_MIN i2fpt(T_MIN)
#define TQ_MAX i2fpt(T_MAX)

/* bmp280 accuracy is ~ 1hPa */
#define P_MIN         (88000) /* 882 hPa was record low in hurricane */
#define P_MAX         (110000) /* 1100 hPa bmp280 max pressure */

/* pressure is Q17.14 (sign, 17, 14)*/
#define P_R           (FPT_FBITS - 2) /* radix */
#define P_FBITS       (P_R)
#define P_WBITS       (FPT_WBITS + 2)
#define FPT_ONEx(r)   ((fpt)((fpt)1 << r))
#define fl2fptx(F, r) ((fpt)((F) * FPT_ONEx(r) + ((F) >= 0 ? 0.5 : -0.5)))
#define fpt2flx(T, r) ((float) ((T)*((float)(1)/(float)(1 << (r)))))

#define i2fpt_q17(v)  (i2fpt_norm((v), 2)) /* 2 bits more for integer */
#define fl2fpt_q17(F) (fl2fptx((F), P_R))
#define fpt2fl_q17(T) (fpt2flx(T, P_R))

/* finally define min and max value */
#define PQ_MIN        (i2fpt_q17(P_MIN))
#define PQ_MAX        (i2fpt_q17(P_MAX))

#define H_MIN 0
#define H_MAX 100
#define HQ_MIN i2fpt(H_MIN)
#define HQ_MAX i2fpt(H_MAX)


#define QUANTIZE_Q(FROM, TO, BYTES, STORAGE, VALUE) \
  (STORAGE)fpt_div( \
    fpt_sub(MINMAX(FROM, TO, VALUE), FROM), \
    Q_RANGE(FROM, TO) \
  )

// static uint16_t quant(fpt val) {
//     if (val < Q_MIN) {
//         val = Q_MIN;
//     } else if (val > Q_MAX) {
//         val = Q_MAX;
//     }

//     val = fpt_sub(val, Q_MIN);
//     val = fpt_div(val, Q_RANGE);

//     return val;
// }

#define DEQUANTIZE_Q(FROM, TO, BYTES, VALUE) \
  fpt_add(fpt_mul((VALUE), Q_RANGE(FROM, TO)), FROM)
// static float dequant(uint16_t val) {
//     fpt as_fpt = val;
//     as_fpt = fpt_mul(as_fpt, Q_RANGE);
//     as_fpt = fpt_add(as_fpt, Q_MIN);

//     return fpt2fl(as_fpt);
// }

/* 16bits: -40 .. 85 */
#define QUANTIZE_TEMP(V) \
  QUANTIZE_Q(TQ_MIN, TQ_MAX, 2, int16_t, V)

#define DEQUANTIZE_TEMP(V) \
  DEQUANTIZE_Q(TQ_MIN, TQ_MAX, 2, V)

/* 16bits: 88000 .. 110000 */
#define QUANTIZE_PRESSURE(V) \
  QUANTIZE_Q(PQ_MIN, PQ_MAX, 2, uint16_t, V)


#define DEQUANTIZE_PRESSURE(V) \
  DEQUANTIZE_Q(PQ_MIN, PQ_MAX, 2, V)


static inline uint8_t quant_h(fpt val) {
    if (val < HQ_MIN) {
        val = HQ_MIN;
    } else if (val > HQ_MAX) {
        val = HQ_MAX;
    }

    val = fpt_div(val, HQ_MAX);
    /* 
     * dunno why but improves maximum difference from 0.39 to 0.19
     */
    val += val & 0xFF;
    val >>= 8;


    return val;
}

/* 8bits: 0 .. 100*/
#define QUANTIZE_HUM(V) quant_h(V)

static inline float dequant_h(uint8_t val) {
    fpt as_fpt = val;
    as_fpt = fpt_mul(as_fpt, HQ_MAX);
    as_fpt <<= 8;


    return fpt2fl(as_fpt);
}

#define DEQUANTIZE_HUM(V) dequant_h(V)

/* convert raw 16bit (range: 0..100) humidity value to FPT */
#define raw_hum_to_fpt(hum) fpt_mul(hum, i2fpt(100))

#define convert_temp(t) QUANTIZE_TEMP(t)

#define convert_pressure(p) QUANTIZE_PRESSURE(p)

// static inline uint8_t convert_hum(float p)
// {
//   return QUANTIZE_HUM(p);
// }

static inline float decode_temp(int32_t t)
{
  return DEQUANTIZE_TEMP(t);
}

static inline float decode_pressure(uint16_t p)
{
  return DEQUANTIZE_PRESSURE(p);
}

#endif