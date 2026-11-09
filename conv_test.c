/* Host check: raw sensor value -> fpt conversion -> packet quantization.
 *
 * The first-stage conversion (include/htuxx.h, shtxx.h, hdc1080.h, ahtxx.h,
 * mcp9808.h) only matters through the packet quantizer that follows it
 * (QUANTIZE_TEMP / QUANTIZE_PRESSURE / QUANTIZE_HUM in sensor.h, packed as
 * uint16/uint8 in packet.h). So the metric here is end-to-end:
 *
 *   DEQUANTIZE_*( (wire type) QUANTIZE_*(conv(raw) + bias) )  vs  truth
 *
 * where truth = A + B*raw/D is the exact datasheet formula mirrored from
 * the vendored drivers' float conversions (compiled out per ADR-0005).
 * Truth values outside the wire range are counted as "clamped", not errors.
 * "max_int" excludes the top half-step of the wire range, where clamping
 * dominates any rounding choice.
 *
 * First-stage variants compared:
 *   cur   - current code: real headers' constants (derived, see ADR-0007)
 *           through the production round-to-nearest QUANTIZE_*
 *   trunc - same converters through the old truncating quantizer,
 *           emulated by subtracting half a wire step before QUANTIZE_*
 *   opt   - optimal constant C = round(B*2^32/D) via fpt_mul (floor)
 *   rnd   - same C with a rounding shift: the 0.5 LSB stage-1 error floor
 *
 * Also documents why the disabled fpt_div branches must stay disabled on
 * 32-bit fpt: i2fpt(65535) does not fit int32.
 *
 * Exit 0 if the trunc emulation stays within one wire step and the
 * production path within half a wire step (interior, clamping excluded).
 */
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "include/sensor.h"

#define LSB     (1.0 / 65536.0)
#define TWO32   4294967296.0L
/* half of one wire step, in fpt LSB, for the trunc emulation */
#define T_HALF  ((fpt)(Q_RANGE(TQ_MIN, TQ_MAX) >> 17))       /* 125/2 = 62 */
#define H_HALF  ((fpt)(Q_RANGE(HQ_MIN, HQ_MAX2) >> 17))      /* 25600/2 = 12800 */
#define P_HALF  ((fpt)(Q_RANGE(PQ_MIN_17, PQ_MAX_17) >> 17)) /* 5500/2 = 2750 */

typedef struct {
  double max_abs, mean, mean_abs, max_int;
  uint32_t worst_raw, n, clamped;
} stat_t;

/* optimal coefficient per variant, dispatched through globals because
 * sweep wants fpt(*f)(uint16_t) */
static int64_t g_C;
static int64_t g_Aq; /* round(A*2^16), subtracted by the opt/rnd variants */
static int g_fail;

static fpt conv_cur_opt(uint16_t raw)
{
  /* A is negative for these sensors, so adding g_Aq subtracts |A| */
  return fpt_mul((fpt)raw, (fpt)g_C) + (fpt)g_Aq;
}

static fpt conv_cur_rnd(uint16_t raw)
{
  return (fpt)(((fptd)raw * g_C + 32768) >> 16) + (fpt)g_Aq;
}

/* ---- stage-1 direct error (diagnosis) ---- */

static stat_t sweep16_direct(fpt (*conv)(uint16_t), double A, double B,
                             double D)
{
  stat_t s = { 0 };
  double sum = 0, sum_abs = 0;

  for (uint32_t raw = 0; raw <= 0xFFFF; raw++)
  {
    double e = (double)conv((uint16_t)raw) / 65536.0
               - (A + B * (double)raw / D);
    double ae = e < 0 ? -e : e;

    sum += e;
    sum_abs += ae;
    if (ae > s.max_abs)
    {
      s.max_abs = ae;
      s.worst_raw = raw;
    }
  }

  s.mean = sum / 65536.0;
  s.mean_abs = sum_abs / 65536.0;
  return s;
}

/* ---- end-to-end through the packet quantizer ----
 * tmin/wire_scale/wire_max describe the wire mapping
 * x = (truth - tmin) * wire_scale, x in [0, wire_max] */

static stat_t sweep16_e2e(fpt (*conv)(uint16_t), fpt bias, double A,
                          double B, double D, double tmin,
                          double wire_scale, double wire_max)
{
  stat_t s = { 0 };
  double sum = 0, sum_abs = 0;

  for (uint32_t raw = 0; raw <= 0xFFFF; raw++)
  {
    double truth = A + B * (double)raw / D;
    double x = (truth - tmin) * wire_scale;
    double e, ae;
    fpt v;
    uint16_t q;

    if (x < 0 || x > wire_max + 0.5)
    {
      s.clamped++;
      continue;
    }

    v = conv((uint16_t)raw) + bias;
    q = (uint16_t)QUANTIZE_TEMP(v);
    e = (double)DEQUANTIZE_TEMP(q) / 65536.0 - truth;
    ae = e < 0 ? -e : e;

    s.n++;
    sum += e;
    sum_abs += ae;
    if (ae > s.max_abs)
    {
      s.max_abs = ae;
      s.worst_raw = raw;
    }
    if (x <= wire_max - 0.5 && ae > s.max_int)
    {
      s.max_int = ae;
    }
  }

  s.mean = sum / (double)s.n;
  s.mean_abs = sum_abs / (double)s.n;
  return s;
}

static void print_e2e(const char *tag, const stat_t *s, double step)
{
  printf("    %-4s e2e max %8.2e (%5.2f wire LSB)  max_int %8.2e "
         "(%5.2f)  mean %9.2e  clamped %u\n",
         tag, s->max_abs, s->max_abs / step, s->max_int,
         s->max_int / step, s->mean, s->clamped);
}

/* per-temperature-sensor report: stage-1 diagnosis + e2e choice */
static void report16(const char *title, double A, double B, double D,
                     fpt (*cur)(uint16_t), int64_t cur_C)
{
  int64_t Copt = (int64_t)(B * TWO32 / D + 0.5L);
  double step = (double)Q_RANGE(TQ_MIN, TQ_MAX) / 65536.0 / 65536.0;
  stat_t c, o, r, e_cur, e_opt, e_rnd, e_trunc, m;

  printf("  %s: T = %.2f + %.2f*raw/%.0f  Copt %lld (cur %+lld)\n",
         title, A, B, D, (long long)Copt, (long long)(cur_C - Copt));

  g_Aq = llround(A * 65536.0);
  g_C = Copt;
  c = sweep16_direct(cur, A, B, D);
  o = sweep16_direct(conv_cur_opt, A, B, D);
  r = sweep16_direct(conv_cur_rnd, A, B, D);
  printf("    stage1 cur max %8.2e (%5.2f LSB)  opt %8.2e (%5.2f)  "
         "rnd %8.2e (%5.2f)\n",
         c.max_abs, c.max_abs / LSB, o.max_abs, o.max_abs / LSB,
         r.max_abs, r.max_abs / LSB);

  for (int64_t d = -1; d <= 1; d++)
  {
    g_C = Copt + d;
    m = sweep16_direct(conv_cur_opt, A, B, D);
    printf("      C%+lld stage1 max %8.2e (%5.2f LSB)\n",
           (long long)d, m.max_abs, m.max_abs / LSB);
  }
  g_C = Copt;

  e_cur = sweep16_e2e(cur, 0, A, B, D, T_MIN, 65536.0 / 125.0, 65535.0);
  e_opt = sweep16_e2e(conv_cur_opt, 0, A, B, D, T_MIN, 65536.0 / 125.0, 65535.0);
  e_rnd = sweep16_e2e(conv_cur_rnd, 0, A, B, D, T_MIN, 65536.0 / 125.0, 65535.0);
  /* subtracting half a wire step emulates the old truncating quantizer */
  e_trunc = sweep16_e2e(cur, -T_HALF, A, B, D, T_MIN,
                        65536.0 / 125.0, 65535.0);
  print_e2e("cur", &e_cur, step);
  print_e2e("opt", &e_opt, step);
  print_e2e("rnd", &e_rnd, step);
  print_e2e("trunc", &e_trunc, step);

  if (e_trunc.max_abs > 1.05 * step)
  {
    printf("    FAIL trunc e2e %9.2e > 1.05*step\n", e_trunc.max_abs);
    g_fail = 1;
  }
  if (e_cur.max_int > 0.55 * step + 3 * LSB)
  {
    printf("    FAIL cur e2e interior %9.2e > bound\n", e_cur.max_int);
    g_fail = 1;
  }
}

/* ---- AHT30: 20-bit raw ---- */

static stat_t sweep20_e2e(fpt (*conv)(uint32_t), fpt bias, double A,
                          double B, double D, double tmin,
                          double wire_scale, double wire_max)
{
  stat_t s = { 0 };
  double sum = 0, sum_abs = 0;

  for (uint32_t raw = 0; raw < (1u << 20); raw++)
  {
    double truth = A + B * (double)raw / D;
    double x = (truth - tmin) * wire_scale;
    double e, ae;
    fpt v;
    uint16_t q;

    if (x < 0 || x > wire_max + 0.5)
    {
      s.clamped++;
      continue;
    }

    v = conv(raw) + bias;
    q = (uint16_t)QUANTIZE_TEMP(v);
    e = (double)DEQUANTIZE_TEMP(q) / 65536.0 - truth;
    ae = e < 0 ? -e : e;

    s.n++;
    sum += e;
    sum_abs += ae;
    if (ae > s.max_abs)
    {
      s.max_abs = ae;
      s.worst_raw = raw;
    }
    if (x <= wire_max - 0.5 && ae > s.max_int)
    {
      s.max_int = ae;
    }
  }

  s.mean = sum / (double)s.n;
  s.mean_abs = sum_abs / (double)s.n;
  return s;
}

/* current converters, mirroring the sensor headers (they cannot be
 * included on the host: common_driver.h pulls in ch32fun.h) */

/* include/htuxx.h htu31d_t_fpt: B*2^32/65535 = i2fpt(B) + B */
static fpt cur_htu31d(uint16_t val)
{
  fpt as_fpt = val;
  as_fpt = fpt_mul(as_fpt, i2fpt(165) + 165);
  as_fpt = fpt_sub(as_fpt, i2fpt(40));
  return as_fpt;
}

/* include/hdc1080.h hdc1080_t_fpt: B*2^32/65536 = i2fpt(B) exact */
static fpt cur_hdc1080(uint16_t val)
{
  fpt as_fpt = val;
  as_fpt = fpt_mul(as_fpt, i2fpt(165));
  as_fpt = fpt_sub(as_fpt, i2fpt(40));
  return as_fpt;
}

/* include/shtxx.h shtxx_t_fpt: B*2^32/65535 = i2fpt(B) + B */
static fpt cur_shtxx(uint16_t val)
{
  fpt as_fpt = val;
  as_fpt = fpt_mul(as_fpt, i2fpt(175) + 175);
  as_fpt = fpt_sub(as_fpt, i2fpt(45));
  return as_fpt;
}

/* include/htuxx.h htu21d_t_fpt, include/si7021.h si7021_t_fpt:
 * B*2^32/65536 = fl2fpt(B) exact */
static fpt cur_htu21d(uint16_t val)
{
  fpt as_fpt = val;
  as_fpt = fpt_mul(as_fpt, fl2fpt(175.72f));
  as_fpt = fpt_sub(as_fpt, fl2fpt(46.85f));
  return as_fpt;
}

/* include/sensor.h raw_hum_to_fpt */
static fpt cur_hum_100(uint16_t val)
{
  return fpt_mul(val, i2fpt(100));
}

/* include/htuxx.h htu21d_h_fpt (float return), i2fpt(125)/i2fpt(6) */
static fpt cur_hum_125_65536(uint16_t val)
{
  fpt as_fpt = val;
  as_fpt = fpt_mul(as_fpt, i2fpt(125));
  as_fpt = fpt_sub(as_fpt, i2fpt(6));
  return as_fpt;
}

/* include/ahtxx.h ahtxx_t_fpt */
static fpt cur_aht_t(uint32_t val)
{
  fpt as_fpt = val;
  as_fpt = fpt_mul(as_fpt, fl2fpt(12.5f));
  as_fpt = fpt_sub(as_fpt, i2fpt(50));
  return as_fpt;
}

/* include/ahtxx.h athxx_h_fpt */
static fpt cur_aht_h(uint32_t val)
{
  return fpt_mul(val, fl2fpt(6.25f));
}

/* include/mcp9808.h mcp9808_t_fpt */
static fpt cur_mcp9808(int16_t val)
{
  return fpt_mul(i2fpt(val), fl2fpt(1.0f / 16.0f));
}

/* humidity e2e: QUANTIZE_HUM/DEQUANTIZE_HUM roundtrip.
 * truth = A + B*raw/D like the temperature sweeps. */
static stat_t sweep16_hum_e2e(fpt (*conv)(uint16_t), fpt bias, double A,
                              double B, double D)
{
  stat_t s = { 0 };
  double sum = 0, sum_abs = 0;

  for (uint32_t raw = 0; raw <= 0xFFFF; raw++)
  {
    double truth = A + B * (double)raw / D;
    double x = truth * (65536.0 / 25600.0);
    double e, ae;
    fpt v;
    uint8_t q;

    if (x < 0 || x > 255.5)
    {
      s.clamped++;
      continue;
    }

    v = conv((uint16_t)raw) + bias;
    q = (uint8_t)(uint16_t)QUANTIZE_HUM(v);
    e = (double)DEQUANTIZE_HUM(q) / 65536.0 - truth;
    ae = e < 0 ? -e : e;

    s.n++;
    sum += e;
    sum_abs += ae;
    if (ae > s.max_abs)
    {
      s.max_abs = ae;
      s.worst_raw = raw;
    }
    if (x <= 254.5 && ae > s.max_int)
    {
      s.max_int = ae;
    }
  }

  s.mean = sum / (double)s.n;
  s.mean_abs = sum_abs / (double)s.n;
  return s;
}

int main(void)
{
  double t_step = (double)Q_RANGE(TQ_MIN, TQ_MAX) / 65536.0 / 65536.0;
  double h_step = (double)Q_RANGE(HQ_MIN, HQ_MAX2) / 65536.0 / 65536.0;
  double p_step = (double)(P_MAX - P_MIN) / 65536.0; /* Pa per wire LSB */

  printf("fpt Q16.16, LSB %.4e; wire steps: T %.4e degC, H %.4e %%RH, "
         "P %.4f Pa\n", LSB, t_step, h_step, p_step);

  printf("temperature (e2e vs truth, clamped = truth outside [-40, 85]):\n");
  /* lib/htu31d/src/driver_htu31d.c: raw/65535*165 - 40 */
  report16("htu31d_t_fpt", -40.0, 165.0, 65535.0, cur_htu31d,
           (int64_t)i2fpt(165) + 165);
  /* lib/hdc1080/src/driver_hdc1080.c: raw/65536*165 - 40 (denominator 2^16,
   * the +166 magic copied from htu31d does not belong here) */
  report16("hdc1080_t_fpt", -40.0, 165.0, 65536.0, cur_hdc1080,
           (int64_t)i2fpt(165));
  /* lib/sht35/src/driver_sht35.c, lib/sht4x/src/driver_sht4x.c:
     raw/65535*175 - 45; shtc3 driver uses raw/65536*175 - 45 instead */
  report16("shtxx_t_fpt", -45.0, 175.0, 65535.0, cur_shtxx,
           (int64_t)i2fpt(175) + 175);
  /* lib/htu21d/src/driver_htu21d.c, lib/si7021/src/driver_si7021.c:
     raw/65536*175.72 - 46.85 */
  report16("htu21d_t_fpt/si7021_t_fpt", -46.85, 175.72, 65536.0,
           cur_htu21d, (int64_t)fl2fpt(175.72f));

  /* shtc3 driver truth (65536) vs the shared 65535-family converter */
  g_Aq = llround(-45.0 * 65536.0);
  g_C = (int64_t)i2fpt(175) + 175;
  stat_t sc = sweep16_direct(conv_cur_opt, -45.0, 175.0, 65536.0);
  printf("    shtc3 driver truth (/65536) vs shared converter: "
         "max %8.2e (%5.2f LSB)\n", sc.max_abs, sc.max_abs / LSB);

  /* AHT30: raw/2^20*200 - 50 == raw*12.5/2^16 - 50, exact coefficient */
  {
    double tmin = T_MIN;
    stat_t e_cur = sweep20_e2e(cur_aht_t, 0, -50.0, 200.0, 1048576.0,
                               tmin, 65536.0 / 125.0, 65535.0);
    stat_t e_trunc = sweep20_e2e(cur_aht_t, -T_HALF, -50.0, 200.0, 1048576.0,
                                 tmin, 65536.0 / 125.0, 65535.0);
    printf("  ahtxx_t_fpt (Copt 819200 = fl2fpt(12.5), already optimal)\n");
    print_e2e("cur", &e_cur, t_step);
    print_e2e("trunc", &e_trunc, t_step);
    if (e_trunc.max_abs > 1.05 * t_step)
    {
      printf("    FAIL trunc e2e %9.2e\n", e_trunc.max_abs);
      g_fail = 1;
    }
    if (e_cur.max_int > 0.55 * t_step + 3 * LSB)
    {
      printf("    FAIL cur e2e interior %9.2e\n", e_cur.max_int);
      g_fail = 1;
    }
  }

  printf("humidity (e2e vs truth, clamped = truth outside [0, 100]):\n");
  {
    /* HTU21D/SI7021 truth: raw/65536*125 - 6, exact + float roundtrip */
    stat_t e1 = sweep16_hum_e2e(cur_hum_125_65536, 0, -6.0, 125.0, 65536.0);
    stat_t e1p = sweep16_hum_e2e(cur_hum_125_65536, -H_HALF,
                                 -6.0, 125.0, 65536.0);
    int lossy = 0;
    for (uint32_t raw = 0; raw <= 0xFFFF; raw++)
    {
      fpt v = cur_hum_125_65536((uint16_t)raw);
      if ((fpt)(float)v != v)
      {
        lossy++;
      }
    }
    printf("  htu21d_h_fpt/si7021_h_fpt (exact, float roundtrip lossy %d)\n",
           lossy);
    print_e2e("cur", &e1, h_step);
    print_e2e("trunc", &e1p, h_step);

    /* SHT4x truth: raw/65535*125 - 6, converter uses 65536 */
    stat_t e2 = sweep16_hum_e2e(cur_hum_125_65536, 0, -6.0, 125.0, 65535.0);
    g_Aq = llround(-6.0 * 65536.0);
    g_C = (int64_t)(125.0 * TWO32 / 65535.0 + 0.5);
    stat_t e2o = sweep16_hum_e2e(conv_cur_opt, 0, -6.0, 125.0, 65535.0);
    printf("  sht4x_h_fpt (/65536 impl vs /65535 truth)\n");
    print_e2e("cur", &e2, h_step);
    print_e2e("opt", &e2o, h_step);

    /* raw_hum_to_fpt: exact for /65536 family (HTU21D/SI7021/HDC1080),
     * /65535 family (HTU31D/SHT35/SHT4x/SHTC3) differs by <= 0.002 %%RH */
    g_Aq = 0;
    g_C = (int64_t)i2fpt(100);
    stat_t e3 = sweep16_hum_e2e(cur_hum_100, 0, 0.0, 100.0, 65536.0);
    stat_t e3p = sweep16_hum_e2e(cur_hum_100, -H_HALF, 0.0, 100.0, 65536.0);
    stat_t e4 = sweep16_hum_e2e(cur_hum_100, 0, 0.0, 100.0, 65535.0);
    printf("  raw_hum_to_fpt\n");
    printf("    vs /65536 truth: ");
    print_e2e("cur", &e3, h_step);
    printf("    vs /65535 truth: ");
    print_e2e("cur", &e4, h_step);
    printf("    trunc (vs /65536): ");
    print_e2e("trunc", &e3p, h_step);
  }

  /* MCP9808: val*2^-4 exact, e2e shows the pure quantizer behavior */
  {
    double step = t_step;
    stat_t s = { 0 }, st = { 0 };
    double sum = 0, sumt = 0;
    uint32_t n = 0;

    for (int32_t val = -2048; val <= 2047; val++)
    {
      double truth = (double)val / 16.0;
      double x = (truth - T_MIN) * (65536.0 / 125.0);
      double e, et;
      uint16_t q;

      if (x < 0 || x > 65535.5)
      {
        continue;
      }

      q = (uint16_t)QUANTIZE_TEMP(cur_mcp9808((int16_t)val));
      e = (double)DEQUANTIZE_TEMP(q) / 65536.0 - truth;
      q = (uint16_t)QUANTIZE_TEMP(cur_mcp9808((int16_t)val) - T_HALF);
      et = (double)DEQUANTIZE_TEMP(q) / 65536.0 - truth;

      n++;
      sum += e;
      sumt += et;
      e = e < 0 ? -e : e;
      et = et < 0 ? -et : et;
      if (e > s.max_abs) { s.max_abs = e; }
      if (e > s.max_int && x <= 65534.5) { s.max_int = e; }
      if (et > st.max_abs) { st.max_abs = et; }
      if (et > st.max_int && x <= 65534.5) { st.max_int = et; }
    }
    s.mean = sum / n;
    st.mean = sumt / n;
    printf("  mcp9808_t_fpt (exact: stage1 error 0)\n");
    print_e2e("cur", &s, step);
    print_e2e("trunc", &st, step);
    if (st.max_abs > 1.05 * step)
    {
      printf("    FAIL trunc e2e %9.2e\n", st.max_abs);
      g_fail = 1;
    }
    if (s.max_int > 0.55 * step + 3 * LSB)
    {
      printf("    FAIL cur e2e interior %9.2e\n", s.max_int);
      g_fail = 1;
    }
  }

  /* Pressure has no stage-1 converter here (bmp280 compensates in the
   * driver); synthetic exact input isolates QUANTIZE_PRESSURE itself */
  {
    stat_t s = { 0 }, st = { 0 };
    double sum = 0, sumt = 0;
    uint32_t n = 0;

    for (int32_t p = P_MIN; p <= P_MAX; p++)
    {
      fpt v = i2fpt_q17(p);
      double x = (double)(p - P_MIN) * 65536.0 / (P_MAX - P_MIN);
      uint16_t q = (uint16_t)QUANTIZE_PRESSURE(v);
      double e = (double)DEQUANTIZE_PRESSURE_FLOAT(q) - (double)p;
      uint16_t qt = (uint16_t)QUANTIZE_PRESSURE(v - P_HALF);
      double et = (double)DEQUANTIZE_PRESSURE_FLOAT(qt) - (double)p;

      n++;
      sum += e;
      sumt += et;
      e = e < 0 ? -e : e;
      et = et < 0 ? -et : et;
      if (e > s.max_abs) { s.max_abs = e; }
      if (e > s.max_int && x <= 65534.5) { s.max_int = e; }
      if (et > st.max_abs) { st.max_abs = et; }
      if (et > st.max_int && x <= 65534.5) { st.max_int = et; }
    }
    s.mean = sum / n;
    st.mean = sumt / n;
    printf("pressure (synthetic exact input, e2e in Pa):\n");
    print_e2e("cur", &s, p_step);
    print_e2e("trunc", &st, p_step);
    if (st.max_abs > 1.05 * p_step)
    {
      printf("    FAIL trunc e2e %9.2e\n", st.max_abs);
      g_fail = 1;
    }
    if (s.max_int > 0.55 * p_step + 3 * LSB * 4)
    {
      printf("    FAIL cur e2e interior %9.2e\n", s.max_int);
      g_fail = 1;
    }
  }

  printf("disabled fpt_div branches, on int32 fpt:\n");
  printf("  (fpt)i2fpt(65535) = %d (negative: divisor wraps, branch is broken)\n",
         (int)(fpt)i2fpt(65535));
  printf("  (fpt)i2fpt(40000) = %d (raw > 32767 overflows i2fpt)\n",
         (int)(fpt)i2fpt(40000));

  printf("%s\n", g_fail ? "FAIL" : "PASS");
  return g_fail ? 1 : 0;
}
