# MCU sleep (AWU) and core clock

Verified against the CH32V00X Reference Manual V1.5 (WCH, ch32v006 manual) and the
ch32v003fun framework source. See GLOSSARY.md for short definitions.

## sleep_for() - closed-loop AWU sleep (src/sleep.c)

- Enters SLEEP (default) or STANDBY (MCU_SLEEP_STANDBY=1), wakes on the LSI-driven
  auto-wakeup (AWU) timer via EXTI line 9 in event mode, then re-arms until the
  requested duration elapses. Accuracy ~2ms (closed loop on SysTick).
- AWU step table (elapsed_us, prescaler) is from HW measurement; prescaler < 256
  is unreliable and average sleep is ~20ms.
- Toggles: include/sleep.h
  - MCU_SLEEP: 0 = busy-wait Delay_Ms, 1 = sleep_for
  - MCU_SLEEP_STANDBY: 0 = SLEEP (default), 1 = standby (lower power)
- pio run does not accept -D flags; flip the #ifndef defaults in include/sleep.h
  to build a variant, or keep the default and toggle at build time.
- ch32v006 builds all three variants SUCCESS. Standby was observed on the
  template hardware to "randomly halt the MCU"; the RM defines standby cleanly
  (SRAM held, IWDG+LSI live, AWU wake), so MCU_SLEEP_STANDBY=1 still needs
  ch32v006 board verification.

## Core clock (8MHz default, floor 3MHz)

- include/funconfig.h: #define FUNCONF_SYSTEM_CORE_CLOCK (8000000)
  - ch32fun.h honors a user value (#ifndef FUNCONF_SYSTEM_CORE_CLOCK, ~line 224)
  - the funconfig.h hook sets FUNCONF_USE_HSI=1, FUNCONF_USE_PLL=0
- 8MHz = HSI (24MHz) / 3 via RCC->CFGR0.HPRE field (bits [7:4], value 0x20)
  - CALCULATED_HPRE_DIV = (((HSI_VALUE) / (FUNCONF_SYSTEM_CORE_CLOCK)) - 1) << 4
    (defined in src/main.c; duplicated in src/sleep.c)
  - RM CFGR0 table: encoding 0010 = SYSCLK/3, and the field RESET value is 0010,
    so the factory/bootloader state is already 8MHz HCLK
- SystemInit() (ch32fun.c) unconditionally writes RCC->CFGR0 = RCC_HPRE_DIV1
  in its no-PLL path, so the divider must be re-applied after every SystemInit:
  - boot: in main() right after SystemInit()
  - wake: in sleep_for() right after SystemInit()
  - both guarded by #if HSI_VALUE * FUNCONF_PLL_MULTIPLIER != FUNCONF_SYSTEM_CORE_CLOCK
    (so reverting to 48MHz is a clean no-op)
- Compile-time check in main.c: #if HSI_VALUE / ((CALCULATED_HPRE_DIV >> 4) + 1)
  != FUNCONF_SYSTEM_CORE_CLOCK -> #error. The formula is only valid for divisors
  1..8 (RM field 0..7 maps to /1../8); larger divisors hit reserved/wrong
  encodings and are rejected.
- Valid core clocks with the existing mechanism: 24, 12, 8, 6, 4.8, 4, 3 MHz
  (W_TICK_RATIO = 24M/CORE stays an integer: 1, 2, 3, 4, 5, 6, 8).
  3MHz (HPRE /8, field 0x70) is the lowest. Hardware could go to /256
  (93.75kHz) but would need new code (proper divisor-to-field mapping).
- Everything scales from FUNCONF_SYSTEM_CORE_CLOCK already:
  - framework: DELAY_MS_TIME / DELAY_US_TIME (~line 880), UART_BRR (~line 1079)
  - project: USART1->BRR in main.c (8MHz: 833 -> 9603.8 baud),
    W_TICK_RATIO in common_driver.c I2C cycle scaling (8MHz: 3),
    flash ACTLR latency (8MHz: LATENCY_0, RM recommends 00 for <=15MHz)
  - systick = HCLK/8 (FUNCONF_SYSTICK_USE_HCLK=0): 1MHz at 8MHz,
    375kHz at 3MHz; sleep.c uses DELAY_MS_TIME / DELAY_US_TIME

## State after wake - no peripheral re-init needed

- SLEEP (RM 2.3.2): "all I/O pins keep their state in Run mode and all
  peripheral clocks are normal"; core clock OFF, no effect on other clocks.
- STANDBY (RM 2.3.3): "the high frequency clock (HSE/HSI/PLL) domain turned
  off, the SRAM and register contents held, and the I/O pin state held";
  HSI is the default system clock after wake; IWDG and LSI stay live.
- Only the clock domain is lost (standby). sleep_for() restores it via
  SystemInit() + HPRE re-apply. Peripheral registers (USART1, DMA1, GPIO,
  I2C, SPI, clock gates) survive, so i2c_init / tca9548a_init / init_pt100 /
  uart_setup / dma_uart_setup from main() stay valid; loop() does not re-run
  setup after wake.
- Audited SystemInit() register writes for the no-PLL path: FLASH->ACTLR
  (latency for configured core clock), RCC->CFGR0 (full write: clears MCO,
  ADCPRE, SW -> HSI, HPRE -> DIV1), RCC->CTLR (HSI+CSS), RCC->INTR
  (clears clock-ready status flags only), SetupDebugPrintf() (SDI, idempotent).
  None touch APB1PCENR, APB2PCENR, USART1, DMA1, GPIO or EXTI.
- sleep_for is called at the end of loop() after hc12_sleep(), so no
  UART/DMA transfer is in flight when the core sleeps. The app has no
  USART1_IRQHandler (RX is polled); the NVIC_DisableIRQ/EnableIRQ of
  USART1_IRQn in sleep_for is defensive.

## Residuals (harmless, known)

- EXTI line 9: EVENR/FTENR bits are set before sleep and never cleared after
  wake; a pending EXTI9 flag may remain. No EXTI9 interrupt is enabled and
  no __WFI runs after wake, so it cannot misfire.
- PFIC->SCTLR: SLEEPDEEP is cleared in the SLEEP branch; WFITOWFE (set by
  ch32fun __WFE) and SLEEPONEXIT may remain set - harmless, app never
  __WFI after wake.

## Build notes

- pio is not on PATH: use ~/.platformio/penv/bin/pio
- ch32v003 env (genericCH32V003F4P6, 16KB flash) does not fit the current
  firmware (~20.6KB .text); pre-existing flash overflow, unrelated to the
  clock change. ch32v006 (64KB flash) is the primary target.
- ch32v006 final size at 8MHz: Flash 44.6% (28320/63488), RAM 12.3%
