#include <unity.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>

/* Pull in the arena functions directly */
#include "sensor.h"

#define ALIGNMENT_TEST 8
#define SMALL_POOL_SIZE 64

static uint8_t test_pool[SMALL_POOL_SIZE];

void setUp(void)
{
  /* each test gets a fresh pool */
}

void tearDown(void)
{
}

static void test_aligned_alloc_returns_aligned_pointer(void)
{
  arena_t arena;
  arena_init(&arena, test_pool, SMALL_POOL_SIZE);

  /* Allocate at offset 3 with alignment 8.
   * Expected: pointer aligned to 8 (offset 8). */
  void *p = arena_alloc_aligned(&arena, ALIGNMENT_TEST, 4);
  TEST_ASSERT_NOT_NULL(p);
  TEST_ASSERT_TRUE(((uintptr_t)p & (ALIGNMENT_TEST - 1)) == 0);

  /* Second alloc at offset 12 with alignment 8.
   * Expected: pointer aligned to 16 (offset 16). */
  p = arena_alloc_aligned(&arena, ALIGNMENT_TEST, 4);
  TEST_ASSERT_NOT_NULL(p);
  TEST_ASSERT_TRUE(((uintptr_t)p & (ALIGNMENT_TEST - 1)) == 0);
}

static void test_aligned_alloc_fits_exactly(void)
{
  arena_t arena;
  uint8_t pool[16];
  arena_init(&arena, pool, sizeof(pool));

  /* offset 0 -> aligned to 0, size 16 fits exactly at end */
  void *p = arena_alloc_aligned(&arena, 8, 16);
  TEST_ASSERT_NOT_NULL(p);
  TEST_ASSERT_EQUAL_PTR(pool, p);
}

static void test_aligned_alloc_returns_null_when_full(void)
{
  arena_t arena;
  uint8_t pool[16];
  arena_init(&arena, pool, sizeof(pool));

  /* Fill exactly */
  arena_alloc_aligned(&arena, 8, 16);

  /* One more aligned alloc of 8 bytes -> needs 8 bytes of padding + 8 = 16,
   * but only 0 left. Should return NULL. */
  void *p = arena_alloc_aligned(&arena, 8, 8);
  TEST_ASSERT_NULL(p);
}

static void test_aligned_alloc_padding_consumes_space(void)
{
  arena_t arena;
  uint8_t pool[32];
  arena_init(&arena, pool, sizeof(pool));

  /* Alloc size 1 at alignment 8.
   * offset 0 -> aligned 0, consumes 1 byte (offset becomes 1).
   * Next alloc size 1 at alignment 8.
   * offset 1 -> aligned to 8, consumes 8 bytes (offset becomes 9). */
  void *p1 = arena_alloc_aligned(&arena, 8, 1);
  TEST_ASSERT_NOT_NULL(p1);

  void *p2 = arena_alloc_aligned(&arena, 8, 1);
  TEST_ASSERT_NOT_NULL(p2);

  /* p2 should be at offset 8 (aligned boundary) */
  TEST_ASSERT_EQUAL_PTR(pool + 8, p2);
}

static void test_aligned_alloc_mixed_sizes(void)
{
  arena_t arena;
  uint8_t pool[128];
  arena_init(&arena, pool, sizeof(pool));

  /* 4-byte aligned alloc of 4 bytes */
  uint32_t *p1 = (uint32_t*)arena_alloc_aligned(&arena, 4, 4);
  TEST_ASSERT_NOT_NULL(p1);
  TEST_ASSERT_TRUE(((uintptr_t)p1 & 3) == 0);

  /* 8-byte aligned alloc of 8 bytes */
  double *p2 = (double*)arena_alloc_aligned(&arena, 8, 8);
  TEST_ASSERT_NOT_NULL(p2);
  TEST_ASSERT_TRUE(((uintptr_t)p2 & 7) == 0);

  /* 16-byte aligned alloc of 16 bytes */
  uint64_t *p3 = (uint64_t*)arena_alloc_aligned(&arena, 16, 16);
  TEST_ASSERT_NOT_NULL(p3);
  TEST_ASSERT_TRUE(((uintptr_t)p3 & 15) == 0);
}

static void test_aligned_alloc_clear_resets_state(void)
{
  arena_t arena;
  uint8_t pool[64];
  arena_init(&arena, pool, sizeof(pool));

  /* Fill half the pool with aligned allocs */
  arena_alloc_aligned(&arena, 8, 32);

  /* Clear */
  arena_clear(&arena);

  /* Should be able to alloc again at the beginning */
  void *p = arena_alloc_aligned(&arena, 8, 32);
  TEST_ASSERT_NOT_NULL(p);
  TEST_ASSERT_EQUAL_PTR(pool, p);
}

static void test_aligned_alloc_16_byte(void)
{
  arena_t arena;
  uint8_t pool[64];
  arena_init(&arena, pool, sizeof(pool));

  /* Alloc uint64_t with 16-byte alignment */
  uint64_t *p = (uint64_t*)arena_alloc_aligned(&arena, 16, sizeof(uint64_t));
  TEST_ASSERT_NOT_NULL(p);
  TEST_ASSERT_TRUE(((uintptr_t)p & 15) == 0);
}

static void test_aligned_alloc_4_byte_alignment(void)
{
  arena_t arena;
  uint8_t pool[64];
  arena_init(&arena, pool, sizeof(pool));

  /* 4-byte alignment */
  void *p = arena_alloc_aligned(&arena, 4, 4);
  TEST_ASSERT_NOT_NULL(p);
  TEST_ASSERT_TRUE(((uintptr_t)p & 3) == 0);
}

static void test_aligned_alloc_non_power_of_2_returns_null(void)
{
  arena_t arena;
  uint8_t pool[64];
  arena_init(&arena, pool, sizeof(pool));

  /* 6 is not power-of-2 */
  void *p = arena_alloc_aligned(&arena, 6, 4);
  TEST_ASSERT_NULL(p);

  /* 0 is not power-of-2 */
  p = arena_alloc_aligned(&arena, 0, 4);
  TEST_ASSERT_NULL(p);

  /* 5 is not power-of-2 */
  p = arena_alloc_aligned(&arena, 5, 4);
  TEST_ASSERT_NULL(p);
}

static void test_aligned_alloc_boundary_full(void)
{
  arena_t arena;
  uint8_t pool[16];
  arena_init(&arena, pool, sizeof(pool));

  /* Alloc 15 bytes at alignment 8.
   * offset 0 -> aligned 0, consumes 15 bytes (now = 15).
   * Alloc 1 byte at alignment 8.
   * offset 15 -> aligned to 16, but 16 + 1 > 16 = end -> NULL. */
  arena_alloc_aligned(&arena, 8, 15);

  void *p = arena_alloc_aligned(&arena, 8, 1);
  TEST_ASSERT_NULL(p);
}

static void test_regular_alloc_overflow(void)
{
  arena_t arena;
  uint8_t pool[4];
  arena_init(&arena, pool, sizeof(pool));

  /* Fill exactly */
  void *p1 = arena_alloc(&arena, 4);
  TEST_ASSERT_NOT_NULL(p1);

  /* Next alloc should fail */
  void *p2 = arena_alloc(&arena, 1);
  TEST_ASSERT_NULL(p2);
}

static void test_regular_alloc_mixed_sizes(void)
{
  arena_t arena;
  uint8_t pool[32];
  arena_init(&arena, pool, sizeof(pool));

  /* Allocate various sizes */
  void *p1 = arena_alloc(&arena, 5);
  TEST_ASSERT_NOT_NULL(p1);
  TEST_ASSERT_EQUAL_PTR(pool, p1);

  void *p2 = arena_alloc(&arena, 10);
  TEST_ASSERT_NOT_NULL(p2);
  TEST_ASSERT_EQUAL_PTR(pool + 5, p2);

  void *p3 = arena_alloc(&arena, 10);
  TEST_ASSERT_NOT_NULL(p3);
  TEST_ASSERT_EQUAL_PTR(pool + 15, p3);
}

int main(void)
{
  UNITY_BEGIN();

  RUN_TEST(test_aligned_alloc_returns_aligned_pointer);
  RUN_TEST(test_aligned_alloc_fits_exactly);
  RUN_TEST(test_aligned_alloc_returns_null_when_full);
  RUN_TEST(test_aligned_alloc_padding_consumes_space);
  RUN_TEST(test_aligned_alloc_mixed_sizes);
  RUN_TEST(test_aligned_alloc_clear_resets_state);
  RUN_TEST(test_aligned_alloc_16_byte);
  RUN_TEST(test_aligned_alloc_4_byte_alignment);
  RUN_TEST(test_aligned_alloc_non_power_of_2_returns_null);
  RUN_TEST(test_aligned_alloc_boundary_full);
  RUN_TEST(test_regular_alloc_overflow);
  RUN_TEST(test_regular_alloc_mixed_sizes);

  return UNITY_END();
}
