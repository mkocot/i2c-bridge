#include "sleep.h"

#include <ch32fun.h>

// systick ticks per ms/us, derived from FUNCONF_SYSTEM_CORE_CLOCK by ch32fun.h
#define SYSTICK_ONE_MILLISECOND DELAY_MS_TIME
#define SYSTICK_ONE_MICROSECOND DELAY_US_TIME

// HPRE field dividing HSI to FUNCONF_SYSTEM_CORE_CLOCK, same formula as main.c
#define CALCULATED_HPRE_DIV ((((HSI_VALUE) / (FUNCONF_SYSTEM_CORE_CLOCK)) - 1) << 4)

// PFIC->SCTLR bits, CH32V00X reference manual 6.5.2.22
#define SCTLR_SLEEPONEXIT (1 << 1)
#define SCTLR_DEEPSLEEP   (1 << 2)

// auto wakeup enable bit in PWR->AWUCSR, manual 2.4.3
// ch32x00xhw.h does not define it
#define AWUCSR_AWUEN (1 << 1)

// LSI driven auto wakeup steps, measured on HW; rescaled x1.087 on
// 2026-10-15 to this board's LSI (~114kHz, last row directly verified:
// 60 ticks = 32.33s, smaller rows extrapolated)
struct awu_step_t
{
  uint32_t elapsed_us;
  uint8_t value;
};

static const struct awu_step_t awu_steps[] = {
  // from measurement:
  // prescaler < 256 is unreliable and average sleep time is around 20ms
  {2187, PWR_AWU_Prescaler_256},     // 2ms
  {4440, PWR_AWU_Prescaler_512},     // 4ms
  {8932, PWR_AWU_Prescaler_1024},    // 8ms
  {17912, PWR_AWU_Prescaler_2048},   // 16ms
  {35880, PWR_AWU_Prescaler_4096},   // 32ms
  {89774, PWR_AWU_Prescaler_10240},  // 80ms
  {538838, PWR_AWU_Prescaler_61440}  // 480ms
};

// Closed loop AWU sleep, ported from the ch32 template.
// Verified against the CH32V00X reference manual (2.3.2 sleep, 2.3.3 standby,
// 2.3.4 AWU, 2.4.3-2.4.5 AWU registers, 3.4.9 LSI).
// Diverges from the template in:
// - systick ticks derived from FUNCONF_SYSTEM_CORE_CLOCK, template hardcoded 8MHz HSI
// - raw ch32fun registers instead of SDK PWR_*/RCC_*Cmd helpers,
//   equivalent since only AWUPSC[3:0], AWUWR[5:0] and AWUCSR[1] exist
// - |= on EXTI and PFIC->SCTLR instead of full register writes
// - dropped open control branch, debug prints and unused variables
// - STANDBY path (MCU_SLEEP_STANDBY): SysTick freezes with the HCLK domain,
//   so elapsed time is window-counted in SRAM, calibrated live against
//   SysTick in cheap SLEEP windows (manual 2.3.3-2.3.4, ADR-0012)
#if MCU_SLEEP_STANDBY
// AWUPSC dividers aligned with awu_steps[], manual 2.4.5: exact integers,
// every row is an exact multiple of the calibration row 10240
static const uint16_t awu_psc_div[] = {256, 512, 1024, 2048, 4096, 10240, 61440};
#define AWU_CAL_ROW 5
#define AWU_CAL_TICKS 2
// sanity bounds: one tick at 10240 for LSI 90..160kHz
#define AWU_CAL_TICK_MIN 64000
#define AWU_CAL_TICK_MAX 115000
// standby LDO low-power mode (1.0V vs 1.2V, manual 2.3.3) shifts the LSI;
// measured on this board, standby windows run ~10% shorter than the
// SLEEP-mode calibration predicts
#define AWU_STANDBY_TICK_NUM 9
#define AWU_STANDBY_TICK_DEN 10

// measures one LSI tick at prescaler 10240 against SysTick in SLEEP mode
// windows; min of two rejects stale-event wakes. 0 means calibration
// failed, the caller falls back to the static table
static uint32_t awu_cal_tick_us(void)
{
  uint32_t tick_us = 0;
  for (uint8_t n = 0; n < 2; n++) {
    PWR->AWUPSC = awu_steps[AWU_CAL_ROW].value;
    PWR->AWUWR = AWU_CAL_TICKS - 1;
    EXTI->INTFR = EXTI_Line9;
    PWR->AWUCSR |= AWUCSR_AWUEN;
    uint32_t t0 = funSysTick32();
    __WFE();
    if (EXTI->INTFR & EXTI_Line9) {
      uint32_t us = (funSysTick32() - t0) / SYSTICK_ONE_MICROSECOND / AWU_CAL_TICKS;
      if (us >= AWU_CAL_TICK_MIN && us <= AWU_CAL_TICK_MAX && (!tick_us || us < tick_us)) {
        tick_us = us;
      }
    }
    EXTI->INTFR = EXTI_Line9;
  }
  return tick_us;
}
#endif

void sleep_for(uint32_t ms)
{
  /* Store what IRQ is enabled */
  uint32_t usart1_irq_was = NVIC_GetStatusIRQ(USART1_IRQn);
  NVIC_DisableIRQ(USART1_IRQn);

  // power interface clock and LSI
  RCC->APB1PCENR |= RCC_APB1Periph_PWR;
  RCC->RSTSCKR |= RCC_LSION;
  while ((RCC->RSTSCKR & RCC_LSIRDY) == 0) {}

  // EXTI line 9 carries the auto wakeup event
  EXTI->EVENR |= EXTI_Line9;
  EXTI->FTENR |= EXTI_Line9;

#if MCU_SLEEP_STANDBY
  // calibration runs in SLEEP mode, DEEPSLEEP must be clear
  PFIC->SCTLR &= ~SCTLR_DEEPSLEEP;

  uint32_t t0 = funSysTick32();
  uint32_t cal = awu_cal_tick_us();
  // calibration windows are real SLEEP-mode sleep, count them
  uint32_t cal_ms = (funSysTick32() - t0) / SYSTICK_ONE_MILLISECOND;
  uint32_t remaining_ms = ms > cal_ms ? ms - cal_ms : 0;

  // per-tick in the standby regulator state; fall back to the static
  // table scaled the same way when calibration failed
  uint32_t cal_sb = (cal ? cal : awu_steps[AWU_CAL_ROW].elapsed_us)
                    * AWU_STANDBY_TICK_NUM / AWU_STANDBY_TICK_DEN;
  uint32_t tick_us[7];
  for (uint8_t i = 0; i < 6; i++) {
    tick_us[i] = cal_sb * awu_psc_div[i] / awu_psc_div[AWU_CAL_ROW];
  }
  // cal_sb * 61440 overflows 32 bits, scale by the exact ratio 6
  tick_us[6] = cal_sb * 6;

  // STANDBY - low power mode, manual 2.3.3
  PWR->CTLR |= PWR_CTLR_PDDS;
  PFIC->SCTLR |= SCTLR_DEEPSLEEP | SCTLR_SLEEPONEXIT;

  uint8_t index = sizeof(awu_steps) / sizeof(awu_steps[0]) - 1;

  for (;;) {
    uint32_t remaining_us = remaining_ms * 1000;
    // select prescaler and window
    while (index && tick_us[index] > remaining_us) {
      --index;
    }

    uint32_t window = remaining_us / tick_us[index];

    if (window > 64) {
      window = 64;
    } else if (!window) {
      if (index) {
        --index;
        continue;
      }
      // not worth waiting, just exit
      break;
    }

    // written value is window minus 1, manual: window = input + 1
    --window;

    PWR->AWUPSC = awu_steps[index].value;
    PWR->AWUWR = window;
    EXTI->INTFR = EXTI_Line9;
    PWR->AWUCSR |= AWUCSR_AWUEN;

    __WFE();

    // EXTI INTFR does not latch across standby, the peripheral domain is
    // powered down (manual 2.3.3: only IWDG and LSI keep running), so
    // every wake is treated as the window elapsing; waiting for the flag
    // would rearm the same window forever
    // the final window overshoots by up to one tick; saturate or
    // remaining_ms wraps to ~2^32 and the loop spins for weeks
    uint32_t slept_ms = (uint32_t)(window + 1) * tick_us[index] / 1000;
    remaining_ms = slept_ms < remaining_ms ? remaining_ms - slept_ms : 0;
  }
#else
  // SLEEP - core disabled, peripherals and SysTick keep running, manual 2.3.2
  PWR->CTLR &= ~PWR_CTLR_PDDS;
  PFIC->SCTLR &= ~SCTLR_DEEPSLEEP;

  // active sleeping for longer than 65s should not be ever required,
  // just use "forever" sleep and wakeup from IRQ
  uint32_t target = funSysTick32() + ms * SYSTICK_ONE_MILLISECOND;
  uint32_t dur_us = ms * 1000;
  uint8_t index = sizeof(awu_steps) / sizeof(awu_steps[0]) - 1;

  while ((int32_t)(funSysTick32() - target) < 0) {
    // select prescaler and window
    while (index && awu_steps[index].elapsed_us > dur_us) {
      --index;
    }

    // window must stay wide: dur_us/elapsed_us reaches ~137k for 5-minute
    // sleeps, a uint8_t here truncates mod 256 and defeats the 64 clamp
    uint32_t window = dur_us / awu_steps[index].elapsed_us;

    if (window > 64) {
      window = 64;
    } else if (!window) {
      if (index) {
        --index;
        continue;
      }
      // not worth waiting, just exit
      break;
    }

    // written value is window minus 1, manual: window = input + 1
    --window;

    PWR->AWUPSC = awu_steps[index].value;
    PWR->AWUWR = window;
    PWR->AWUCSR |= AWUCSR_AWUEN;

    __WFE();

    // closed control with checking of elapsed time, accuracy ~ 2ms
    uint32_t now = funSysTick32();
    if (now < target) {
      dur_us = (target - now) / SYSTICK_ONE_MICROSECOND;
    }
  }
#endif

  // after standby wake up the core clock is HSI, SystemInit restores the PLL
  SystemInit();
#if HSI_VALUE * FUNCONF_PLL_MULTIPLIER != FUNCONF_SYSTEM_CORE_CLOCK
  // SystemInit resets HPRE to DIV1
  RCC->CFGR0 = (RCC->CFGR0 & ~RCC_HPRE) | CALCULATED_HPRE_DIV;
#endif

  // release what was enabled for the sleep
  // SLEEPONEXIT would sleep the core on the next ISR return to thread mode,
  // and with DEEPSLEEP/PDDS still set from the standby branch that is a
  // standby entry mid work phase; USART1 is re-enabled right after
  PFIC->SCTLR &= ~(SCTLR_SLEEPONEXIT | SCTLR_DEEPSLEEP);
  PWR->AWUCSR &= ~AWUCSR_AWUEN;
  RCC->RSTSCKR &= ~RCC_LSION;
  RCC->APB1PCENR &= ~RCC_APB1Periph_PWR;

  if (usart1_irq_was) {
    NVIC_EnableIRQ(USART1_IRQn);
  }
}
