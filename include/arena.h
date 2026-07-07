#ifndef I2C_BRIDGE_ARENA_H
#define I2C_BRIDGE_ARENA_H

#include <stdint.h>
#include <stddef.h>

typedef struct arena_s arena_t;
struct arena_s {
    const void *memory;
    const void *end;
    void *now;
};

#define ARENA_INIT(POOL, SIZE) {(POOL), (POOL) + (SIZE), (POOL)}

uint8_t arena_init(arena_t *arena, void *pool, size_t size);
void* arena_alloc(arena_t *arena, size_t size);
void* arena_alloc_aligned(arena_t *arena, size_t alignment, size_t size);
uint8_t arena_clear(arena_t *arena);

#endif
