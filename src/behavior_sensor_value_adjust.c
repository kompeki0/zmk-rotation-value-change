/*
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT zmk_behavior_sensor_value_adjust

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <zmk/virtual_key_position.h>
#include <zmk/events/position_state_changed.h>

#include "zmk_value_store.h"

#ifndef ZMK_KEYMAP_SENSORS_LEN
#define ZMK_KEYMAP_SENSORS_LEN 0
#endif

#ifndef ZMK_KEYMAP_LAYERS_LEN
#define ZMK_KEYMAP_LAYERS_LEN 1
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

enum dir {
    DIR_NONE = 0,
    DIR_CW = 1,
    DIR_CCW = 2,
};

struct behavior_sensor_value_adjust_config {
    uint8_t index;
    int32_t min_v;
    int32_t max_v;
    int32_t step;
    uint16_t ticks_per_step;
};

struct value_adjust_state {
    // 回転イベントの累積（同方向でも逆でも良いが、ここでは「符号付きで」持つ）
    int32_t tick_accum;
};

struct behavior_sensor_value_adjust_data {
    enum dir pending_dir[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
    struct value_adjust_state st[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
};

static int accept_data(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event,
                       const struct zmk_sensor_config *sensor_config,
                       size_t channel_data_size,
                       const struct zmk_sensor_channel_data *channel_data) {
    ARG_UNUSED(binding);
    ARG_UNUSED(sensor_config);
    ARG_UNUSED(channel_data_size);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_sensor_value_adjust_data *data = dev->data;

    const int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);

    const struct sensor_value v = channel_data[0].value;
    int delta = (v.val1 == 0) ? v.val2 : v.val1;

    if (delta > 0) {
        data->pending_dir[sensor_index][event.layer] = DIR_CW;
    } else if (delta < 0) {
        data->pending_dir[sensor_index][event.layer] = DIR_CCW;
    } else {
        data->pending_dir[sensor_index][event.layer] = DIR_NONE;
    }

    LOG_DBG("accept pos=%d layer=%d val1=%d val2=%d delta=%d dir=%d",
            event.position, event.layer, v.val1, v.val2, delta,
            data->pending_dir[sensor_index][event.layer]);

    return 0;
}

static int process(struct zmk_behavior_binding *binding,
                   struct zmk_behavior_binding_event event,
                   enum behavior_sensor_binding_process_mode mode) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_sensor_value_adjust_config *cfg = dev->config;
    struct behavior_sensor_value_adjust_data *data = dev->data;

    const int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);

    if (mode != BEHAVIOR_SENSOR_BINDING_PROCESS_MODE_TRIGGER) {
        data->pending_dir[sensor_index][event.layer] = DIR_NONE;
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    enum dir d = data->pending_dir[sensor_index][event.layer];
    data->pending_dir[sensor_index][event.layer] = DIR_NONE;

    if (d == DIR_NONE) {
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    event.source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL;
#endif

    struct value_adjust_state *st = &data->st[sensor_index][event.layer];

    // ticks-per-step を安全化
    uint16_t tps = cfg->ticks_per_step ? cfg->ticks_per_step : 1;

    // 1回のTRIGGERを「1 tick」として数える（あなたの“連打抑止”と相性が良い）
    if (d == DIR_CW) {
        st->tick_accum += 1;
    } else {
        st->tick_accum -= 1;
    }

    // 何tick溜まったら実際に値を動かすか
    while (st->tick_accum >= (int32_t)tps) {
        st->tick_accum -= tps;

        int32_t newv = 0;
        zmk_value_store_add_clamped(cfg->index, cfg->step, cfg->min_v, cfg->max_v, &newv);
        LOG_DBG("value[%d] += %d -> %d", cfg->index, (int)cfg->step, (int)newv);
    }

    while (st->tick_accum <= -(int32_t)tps) {
        st->tick_accum += tps;

        int32_t newv = 0;
        zmk_value_store_add_clamped(cfg->index, -cfg->step, cfg->min_v, cfg->max_v, &newv);
        LOG_DBG("value[%d] -= %d -> %d", cfg->index, (int)cfg->step, (int)newv);
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api api = {
    .sensor_binding_accept_data = accept_data,
    .sensor_binding_process = process,
};

#define INST(n)                                                                                      \
    static const struct behavior_sensor_value_adjust_config cfg_##n = {                              \
        .index = (uint8_t)DT_INST_PROP(n, index),                                                     \
        .min_v = DT_INST_PROP_OR(n, min, 0),                                                          \
        .max_v = DT_INST_PROP_OR(n, max, 100),                                                        \
        .step = DT_INST_PROP_OR(n, step, 1),                                                          \
        .ticks_per_step = DT_INST_PROP_OR(n, ticks_per_step, 1),                                      \
    };                                                                                               \
    static struct behavior_sensor_value_adjust_data data_##n = {};                                    \
    BEHAVIOR_DT_INST_DEFINE(                                                                          \
        n, NULL, NULL, &data_##n, &cfg_##n,                                                           \
        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                             \
        &api);

DT_INST_FOREACH_STATUS_OKAY(INST)
