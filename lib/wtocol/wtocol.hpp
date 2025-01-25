#ifndef W_WTOCOL_H
#define W_WTOCOL_H

// #include <Arduino.h>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>

#ifdef __LGT8F__
using ssize_t = int;
using off_t = ssize_t;
#endif

template <int32_t MIN, int32_t MAX, typename T, uint8_t Q = sizeof(T) * 8>
struct Quantizer {
  using STORAGE = T;
  // Value 0 is reserved for NAN, +/-INF, etc
  constexpr static const auto FACTOR =
      ((static_cast<uint32_t>(1) << Q) - 1) / static_cast<float>(MAX - MIN);

  constexpr static inline T quantize(float val) {

    if (std::isnan(val) || std::isinf(val)) {
      return 0;
    }

    // Clamp to MIN...MAX
    if (val < MIN) {
      val = MIN;
    } else if (val > MAX) {
      val = MAX;
    }

    return 1 + static_cast<T>((val - MIN) * FACTOR + 0.5f);
  }
};

struct THPCompoundSensorData {
  using TempQuantizer = Quantizer<-40, 85, uint16_t>;
  using HumidityQuantizer = Quantizer<0, 100, uint8_t>;
  using PressureQuantizer = Quantizer<300, 110000, uint16_t>;
};

#endif
