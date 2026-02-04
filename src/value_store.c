/*
 * SPDX-License-Identifier: MIT
 */
#include "zmk_value_store.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#ifndef CONFIG_ZMK_VALUE_STORE_SIZE
#define CONFIG_ZMK_VALUE_STORE_SIZE 8
#endif

// 共有ストア（単純実装）
// NOTE: 今はRAMのみ。永続化(settings)は次の段階で足せる。
static int32_t values[CONFIG_ZMK_VALUE_STORE_SIZE];

// 競合は基本起きにくいが、保険で軽量ロック（割り込み禁止）で守る
static inline unsigned int lock(void) { return irq_lock(); }
static inline void unlock(unsigned int key) { irq_unlock(key); }

int32_t zmk_value_store_get(uint8_t index, int32_t fallback) {
    if (index >= CONFIG_ZMK_VALUE_STORE_SIZE) {
        return fallback;
    }
    unsigned int k = lock();
    int32_t v = values[index];
    unlock(k);
    return v;
}

bool zmk_value_store_set(uint8_t index, int32_t value) {
    if (index >= CONFIG_ZMK_VALUE_STORE_SIZE) {
        return false;
    }
    unsigned int k = lock();
    values[index] = value;
    unlock(k);
    return true;
}

static inline int32_t clamp_i32(int32_t x, int32_t lo, int32_t hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

bool zmk_value_store_add_clamped(uint8_t index, int32_t delta, int32_t min_v, int32_t max_v,
                                 int32_t *out_new_value) {
    if (index >= CONFIG_ZMK_VALUE_STORE_SIZE) {
        return false;
    }
    if (min_v > max_v) {
        // 変な設定は入れ替えてでも安全に
        int32_t tmp = min_v;
        min_v = max_v;
        max_v = tmp;
    }

    unsigned int k = lock();
    int32_t cur = values[index];
    int32_t next = clamp_i32(cur + delta, min_v, max_v);
    values[index] = next;
    unlock(k);

    if (out_new_value) {
        *out_new_value = next;
    }
    return true;
}
