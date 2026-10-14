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

// LSI driven auto wakeup steps, measured on HW
struct awu_step_t
{
  uint32_t elapsed_us;
  uint8_t value;
};

static const struct awu_step_t awu_steps[] = {
  // from measurement:
  // prescaler < 256 is unreliable and average sleep time is around 20ms
  {2012, PWR_AWU_Prescaler_256},     // 2ms
  {4085, PWR_AWU_Prescaler_512},     // 4ms
  {8217, PWR_AWU_Prescaler_1024},    // 8ms
  {16478, PWR_AWU_Prescaler_2048},   // 16ms
  {33008, PWR_AWU_Prescaler_4096},   // 32ms
  {82589, PWR_AWU_Prescaler_10240},  // 80ms
  {495711, PWR_AWU_Prescaler_61440}  // 480ms
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
void sleep_for(uint32_t ms)
{
  // active sleeping for longer than 65s should not be ever required,
  // just use "forever" sleep and wakeup from IRQ
  uint32_t target = funSysTick32() + ms * SYSTICK_ONE_MILLISECOND;
  uint32_t dur_us = ms * 1000;

  NVIC_DisableIRQ(USART1_IRQn);

  // power interface clock and LSI
  RCC->APB1PCENR |= RCC_APB1Periph_PWR;
  RCC->RSTSCKR |= RCC_LSION;
  while ((RCC->RSTSCKR & RCC_LSIRDY) == 0) {}

  // EXTI line 9 carries the auto wakeup event
  EXTI->EVENR |= EXTI_Line9;
  EXTI->FTENR |= EXTI_Line9;

  // SLEEP - core disabled, manual 2.3.2
  // STANDBY - low power mode, manual 2.3.3
#if MCU_SLEEP_STANDBY
  PWR->CTLR |= PWR_CTLR_PDDS;
  PFIC->SCTLR |= SCTLR_DEEPSLEEP | SCTLR_SLEEPONEXIT;
#else
  PWR->CTLR &= ~PWR_CTLR_PDDS;
  PFIC->SCTLR &= ~SCTLR_DEEPSLEEP;
#endif

  uint8_t index = sizeof(awu_steps) / sizeof(awu_steps[0]) - 1;

  while ((int32_t)(funSysTick32() - target) < 0) {
    // select prescaler and window
    while (index && awu_steps[index].elapsed_us > dur_us) {
      --index;
    }

    uint8_t window = dur_us / awu_steps[index].elapsed_us;

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

  // after standby wake up the core clock is HSI, SystemInit restores the PLL
  SystemInit();
#if HSI_VALUE * FUNCONF_PLL_MULTIPLIER != FUNCONF_SYSTEM_CORE_CLOCK
  // SystemInit resets HPRE to DIV1
  RCC->CFGR0 = (RCC->CFGR0 & ~RCC_HPRE) | CALCULATED_HPRE_DIV;
#endif

  // release what was enabled for the sleep
  PWR->AWUCSR &= ~AWUCSR_AWUEN;
  RCC->RSTSCKR &= ~RCC_LSION;
  RCC->APB1PCENR &= ~RCC_APB1Periph_PWR;

  NVIC_EnableIRQ(USART1_IRQn);
}
