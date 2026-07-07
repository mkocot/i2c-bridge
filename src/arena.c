#include "arena.h"

uint8_t arena_init(arena_t *arena, void *pool, size_t size)
{
  arena->memory = pool;
  arena->end = pool + size;
  arena->now = pool;

  return 0;
}

void* arena_alloc(arena_t *arena, size_t size)
{
  if ((uintptr_t)arena->now + size > (uintptr_t)arena->end)
  {
    return NULL;
  }

  void* ptr = arena->now;
  arena->now = (void*)((uintptr_t)arena->now + size);

  return ptr;
}

void* arena_alloc_aligned(arena_t *arena, size_t alignment, size_t size)
{
  if (!alignment || (alignment & (alignment - 1)))
  {
    return NULL;
  }

  uintptr_t now = (uintptr_t)arena->now;
  uintptr_t aligned_now = (now + alignment - 1) & ~(alignment - 1);

  if (aligned_now + size > (uintptr_t)arena->end)
  {
    return NULL;
  }

  arena->now = (void*)aligned_now;
  void* ptr = arena->now;
  arena->now = (void*)(aligned_now + size);

  return ptr;
}

uint8_t arena_clear(arena_t *arena)
{
  arena->now = (void*) arena->memory;

  return 0;
}
