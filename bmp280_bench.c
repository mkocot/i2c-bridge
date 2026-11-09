/* Host check: the driver's fpt compensation (DRIVER_BMP280_WITH_INT) against
 * the float datasheet formula, through the packet quantizer.
 *
 * The driver is compiled in from source so this exercises the shipped code,
 * not a copy of it.  The metric is the same end-to-end one conv_test.c uses:
 *
 *   DEQUANTIZE_*( (wire type) QUANTIZE_*(conv(raw)) )  vs  truth
 *
 * where truth is the exact Bosch float formula in double precision.  Raw
 * values whose truth falls outside the wire range are counted as "clamped",
 * not as errors: the converter clamps silently and keeps the reading.
 *
 * Reference: p_b64 is the Bosch integer formula as published, which needs a
 * 64-bit division (__divdi3) and therefore cannot ship.  It is here only to
 * show how much of the remaining error is the quantizer and how much is ours.
 *
 * Exit 0 if both converters stay under one wire step end to end.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#include "include/sensor.h"

#define DRIVER_BMP280_WITH_INT
#include "lib/bmp280/src/driver_bmp280.c"

/* one wire step, in the units each converter returns */
#define T_WIRE_STEP_FPT  ((double)Q_RANGE(TQ_MIN, TQ_MAX) / 65535.0)
#define P_WIRE_STEP_FPT  ((double)Q_RANGE(PQ_MIN_17, PQ_MAX_17) / 65535.0)

typedef struct {
  uint16_t t1;
  int16_t t2, t3;
  uint16_t p1;
  int16_t p2, p3, p4, p5, p6, p7, p8, p9;
} calib_t;

/* datasheet set, plus three synthetic ones covering the p2/p3 signs */
static const calib_t sets[] = {
  { 27504, 26435, -1000, 36477, -10685, 3024, 2855, 140, -7, 15500, -14600, 6000 },
  { 26000, 28000, -8000, 22000, -16000, -24000, 19000, -9000, 21000, 12000, -11000, 9000 },
  { 25000, 31000, 20000, 20000, 4000, 12000, 900, 3000, 1500, 8000, -3000, 12000 },
  { 28000, 25000, -500, 38000, -9000, 2000, 3200, 100, -5, 16000, -15000, 6500 },
};

static void load_handle(bmp280_handle_t *h, const calib_t *c)
{
  h->t1 = c->t1; h->t2 = c->t2; h->t3 = c->t3;
  h->p1 = c->p1; h->p2 = c->p2; h->p3 = c->p3; h->p4 = c->p4;
  h->p5 = c->p5; h->p6 = c->p6; h->p7 = c->p7; h->p8 = c->p8; h->p9 = c->p9;
  h->t_fine = 0;
}

static double truth_T(const calib_t *c, uint32_t raw)
{
  double var1 = ((double)raw / 16384.0 - (double)c->t1 / 1024.0) * (double)c->t2;
  double var2 = ((double)raw / 131072.0 - (double)c->t1 / 8192.0)
              * ((double)raw / 131072.0 - (double)c->t1 / 8192.0) * (double)c->t3;
  return (var1 + var2) / 5120.0;
}

static double truth_P(const calib_t *c, uint32_t raw, double t_fine)
{
  double var1 = t_fine / 2.0 - 64000.0;
  double var2 = var1 * var1 * (double)c->p6 / 32768.0;
  var2 = var2 + var1 * (double)c->p5 * 2.0;
  var2 = (var2 / 4.0) + (double)c->p4 * 65536.0;
  var1 = ((double)c->p3 * var1 * var1 / 524288.0 + (double)c->p2 * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0) * (double)c->p1;
  if (var1 == 0.0) return 0.0;
  double p = 1048576.0 - (double)raw;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  double w1 = (double)c->p9 * p * p / 2147483648.0;
  double w2 = p * (double)c->p8 / 32768.0;
  return p + (w1 + w2 + (double)c->p7) / 16.0;
}

/* Bosch integer pressure as published: one 64-bit division, host only */
static int32_t p_b64(const calib_t *c, uint32_t raw, int32_t t_fine)
{
  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)c->p6;
  var2 = var2 + ((var1 * (int64_t)c->p5) << 17);
  var2 = var2 + (((int64_t)c->p4) << 35);
  var1 = ((var1 * var1 * (int64_t)c->p3) >> 8) + ((var1 * (int64_t)c->p2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)c->p1) >> 33;
  if (var1 == 0) return 0;
  p = 1048576 - (int64_t)raw;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)c->p9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)c->p8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)c->p7) << 4);
  return (int32_t)p;
}

typedef struct {
  double max_stage, max_e2e, sum_stage;
  long n, clamped;
} stat_t;

static void note(stat_t *s, double stage, double e2e)
{
  s->sum_stage += stage;
  s->n++;
  if (stage > s->max_stage) s->max_stage = stage;
  if (e2e > s->max_e2e) s->max_e2e = e2e;
}

int main(void)
{
  static const int32_t tfs[] = { -40 * 5120, -30 * 5120, -20 * 5120, -10 * 5120, 0,
                                  10 * 5120, 20 * 5120, 25 * 5120, 35 * 5120,
                                  50 * 5120, 65 * 5120, 85 * 5120 };
  int fail = 0;
  double ref_max = 0.0;

  for (unsigned s = 0; s < sizeof(sets) / sizeof(sets[0]); s++) {
    const calib_t *c = &sets[s];
    bmp280_handle_t h;
    stat_t temp = {0}, pres = {0}, pipe = {0};
    bmp280_temperature_t t_out;
    bmp280_pressure_t p_out;

    load_handle(&h, c);

    for (uint32_t raw = 0; raw < (1u << 20); raw++) {
      double tr = truth_T(c, raw);
      if (tr < -40.0 || tr > 85.0) { temp.clamped++; continue; }
      if (a_bmp280_compensate_temperature(&h, raw, &t_out)) { printf("temp failed\n"); return 1; }

      double tq = (double)llround(tr * 65536.0);
      double stage = fabs((double)t_out - tq) / T_WIRE_STEP_FPT;
      double e2e = fabs((double)DEQUANTIZE_TEMP((uint16_t)QUANTIZE_TEMP(t_out)) - tq) / T_WIRE_STEP_FPT;
      note(&temp, stage, e2e);

      /* the same reading carried through the pressure path: t_fine comes from
       * the temperature call above, exactly as on the device */
      if (raw & 1) continue;
      double pr = truth_P(c, raw, tr * 5120.0);
      if (pr < 88000.0 || pr > 110000.0) { pipe.clamped++; continue; }
      if (a_bmp280_compensate_pressure(&h, raw, &p_out)) { printf("pressure failed\n"); return 1; }

      double pq = (double)llround(pr * 16384.0);
      double pstage = fabs((double)p_out - pq) / P_WIRE_STEP_FPT;
      double pe2e = fabs((double)DEQUANTIZE_Q(PQ_MIN_17, PQ_MAX_17, 2,
                                             (uint16_t)QUANTIZE_PRESSURE(p_out)) - pq) / P_WIRE_STEP_FPT;
      note(&pipe, pstage, pe2e);
    }

    /* pressure alone over the whole raw range, at fixed temperatures */
    for (unsigned ti = 0; ti < sizeof(tfs) / sizeof(tfs[0]); ti++) {
      for (uint32_t raw = 0; raw < (1u << 20); raw++) {
        double pr = truth_P(c, raw, (double)tfs[ti]);
        if (pr < 88000.0 || pr > 110000.0) { pres.clamped++; continue; }
        h.t_fine = (int32_t)((int64_t)tfs[ti] * 256);       /* Q8.8, as cached */
        if (a_bmp280_compensate_pressure(&h, raw, &p_out)) { printf("pressure failed\n"); return 1; }

        double pq = (double)llround(pr * 16384.0);
        double stage = fabs((double)p_out - pq) / P_WIRE_STEP_FPT;
        double e2e = fabs((double)DEQUANTIZE_Q(PQ_MIN_17, PQ_MAX_17, 2,
                                               (uint16_t)QUANTIZE_PRESSURE(p_out)) - pq) / P_WIRE_STEP_FPT;
        note(&pres, stage, e2e);

        /* Bosch's int64 formula returns Q24.8, so <<6 lands in Q18.14.  Only
         * the datasheet calibration set is checked: its magnitudes keep
         * (p << 31) * 3125 inside int64. */
        if (s == 0) {
          long long ref = (long long)p_b64(c, raw, tfs[ti]) << 6;
          double d = (double)llabs(ref - (long long)pq) / P_WIRE_STEP_FPT;
          if (d > ref_max) ref_max = d;
        }
      }
    }

    printf("set %u  T   : stage max %7.4f mean %7.5f | e2e max %7.4f  clamped %ld\n",
           s, temp.max_stage, temp.sum_stage / temp.n, temp.max_e2e, temp.clamped);
    printf("        P   : stage max %7.4f mean %7.5f | e2e max %7.4f  clamped %ld\n",
           pres.max_stage, pres.sum_stage / pres.n, pres.max_e2e, pres.clamped);
    printf("        T->P: stage max %7.4f mean %7.5f | e2e max %7.4f  clamped %ld\n",
           pipe.max_stage, pipe.n ? pipe.sum_stage / pipe.n : 0.0, pipe.max_e2e, pipe.clamped);

    if (temp.max_stage > 0.1 || pres.max_stage > 0.1 || pipe.max_stage > 0.1) fail = 1;
    if (temp.max_e2e > 1.0 || pres.max_e2e > 1.0 || pipe.max_e2e > 1.0) fail = 1;
  }

  printf("reference vs datasheet formula: max %.4f wire steps\n", ref_max);
  if (ref_max > 0.1) fail = 1;

  printf("%s\n", fail ? "FAIL" : "PASS");
  return fail;
}
