#ifndef W_DEBUG_H
#define W_DEBUG_H

#include <stdio.h>
#include <ch32fun.h>

#if FUNCONF_USE_DEBUGPRINTF
#define DPRINTF(FMT, ...) printf((FMT), ## __VA_ARGS__)
#else
#define DPRINTF(FMT, ...) ((void)0)
#endif

#define TRACE(FMT, ARGS...) DPRINTF("[%s:%04d]" # FMT "\n", __FUNCTION__, __LINE__, ##ARGS)

/* TRACE_T: TRACE with a SysTick tick stamp (1 tick = 1us at 8MHz core) to
 * locate stalls. Kept for ad-hoc timing diagnostics. */
#define TRACE_T(FMT, ...) DPRINTF("[%s:%04d %lu]" # FMT "\n", __FUNCTION__, __LINE__, (uint32_t)funSysTick32(), ## __VA_ARGS__)

#endif
