#include <cstdint>

#include "stm32h7xx_hal.h"

extern "C" int64_t uxr_millis() { return static_cast<int64_t>(HAL_GetTick()); }

extern "C" int64_t uxr_nanos() {
  return static_cast<int64_t>(HAL_GetTick()) * 1000000LL;
}
