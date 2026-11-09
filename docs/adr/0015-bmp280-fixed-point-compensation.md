# BMP280 compensates in fixed point under DRIVER_BMP280_WITH_INT

include/bmp280.h used to convert the driver's float outputs with fl2fpt/fl2fpt_q17, and the
driver's own integer branch was an approximation that divided 64-bit values (__divdi3) and,
in its never-built #else variant, did not compile. The macro now selects the unit type of the
output: defined, a_bmp280_compensate_temperature/_pressure compute in int64 and write fpt
directly (Q16.16 degC, Q18.14 Pa) and the wrapper stores them as-is; undefined, the original
float compensation runs untouched and the wrapper converts with fl2fpt. The integer branch no
longer contains any float arithmetic, and the wrapper's stage-1 conversion disappears because
there is nothing left to convert.

## Decisions

- t_fine is cached in the existing int32_t handle field as Q8.8 of the datasheet unit
  (5120/degC, so ~0.8 u degC). Q16.16 does not fit int32, and a sweep of cached precision from
  Q0 to Q16 showed no sensitivity on the pressure path at all.
- The only non-power-of-two divisor in the whole algorithm is p1. fpt_div_const needs a
  compile-time divisor, so the 41-bit numerator n8*6250 is split into two 16-bit halves:
  (hi16 % p1) < p1 < 2^16 keeps the second numerator inside uint32, giving two 32-bit
  divisions (__udivsi3/__umodsi3, already linked for the tree) and no __divdi3.
- 1/(1+u/2^15) comes from base-256 long division instead of a reciprocal iteration: with
  dv = 2^23*(1+u/2^15), three digit rounds lift floor(k/dv) to floor(k*2^24/dv), and pq8 =
  q >> 1. k < 2^26 and every remainder stays below dv < 2^24, so all divides are 32-bit
  (__udivsi3/__umodsi3, already linked) and the reciprocal needs no __muldi3 and no __divdi3.
  A three-step Newton seed from 1-x was measured first: same accuracy, but 114 B more code
  and ~3800 more executed instructions per reading on ch32v003, whose rv32ecxw core has no
  hardware multiplier.
- The pressure output keeps its 1/256 Pa fraction (pq8 << 6 into Q18.14). Truncating to whole
  Pa first costs up to 1 Pa, which is three wire steps - the error this ADR exists to avoid.
- Out-of-range readings clamp inside the converters (-40..85 degC, 30000..110000 Pa) and the
  read still succeeds; the temperature conversion no longer returns an error that made
  sensor_bmp280_obtain drop the whole reading.

## Cost and measured accuracy

a_bmp280_compensate_pressure is 1040 B with only 32-bit division helpers left undefined.
ch32v003 builds to 14144 bytes text+data (86.3% flash), ch32v006 to 17396 (27.4%); the
float path kept for the no-macro build is what the 006 pays for (ADR-0005).

make bmp280_bench compiles the driver in from source and sweeps every raw code at 12
temperatures over 4 calibration sets: stage-1 error max 0.008 (temperature) / 0.040
(pressure) wire steps, end to end max 0.97 wire steps, so the quantizer still dominates
(ADR-0010). The published 64-bit Bosch formula, kept in the bench as a reference, agrees with
the datasheet formula to 0.026 wire steps.
