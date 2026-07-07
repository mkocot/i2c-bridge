#ifndef W_DEBUG_H
#define W_DEBUG_H

#include <stdio.h>

#if FUNCONF_USE_DEBUGPRINTF
#define DPRINTF(FMT, ARGS...) printf((FMT), ## ARGS)
#else
#define DPRINTF(FMT, ARGS...) ((void)0)
#endif

#define TRACE(FMT, ARGS...) DPRINTF("[%s:%04d]" # FMT "\n", __FUNCTION__, __LINE__, ##ARGS)

#endif