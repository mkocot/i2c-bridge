#ifndef W_DEBUG_H
#define W_DEBUG_H

#include <stdio.h>

#if FUNCONF_USE_DEBUGPRINTF
#define DPRINTF(FMT, ...) printf((FMT), ## __VA_ARGS__)
#else
#define DPRINTF(FMT, ...) ((void)0)
#endif

#define TRACE(FMT, ARGS...) DPRINTF("[%s:%04d]" # FMT "\n", __FUNCTION__, __LINE__, ##ARGS)

#endif