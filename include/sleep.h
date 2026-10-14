#pragma once

#include <stdint.h>

// 0: MCU sleep, core off, SRAM and peripherals held
// 1: standby (PDDS), lower power; flip for ch32v006 HW verification
#ifndef MCU_SLEEP_STANDBY
#define MCU_SLEEP_STANDBY 0
#endif

// 0: keep the busy-wait delay
// 1: sleep instead of the busy-wait
#ifndef MCU_SLEEP
#define MCU_SLEEP 0
#endif

void sleep_for(uint32_t ms);
