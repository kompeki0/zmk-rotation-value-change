#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

int32_t zmk_value_store_get(uint8_t index, int32_t fallback);
bool zmk_value_store_set(uint8_t index, int32_t value);
bool zmk_value_store_add_clamped(uint8_t index, int32_t delta, int32_t min_v, int32_t max_v,
                                 int32_t *out_new_value);

#ifdef __cplusplus
}
#endif
