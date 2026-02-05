/*
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT zmk_behavior_mouse_move_dynamic

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/behavior_queue.h>
#include <zmk/zmk_value_store.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * param1: MOVE_* (MOVE_LEFT/RIGHT/UP/DOWN or MOVE_X/MOVE_Y encoded)
 * speed magnitude is built from value_store[index]
 */

struct behavior_mouse_move_dynamic_config {
    const char *target_behavior; // DEVICE_DT_NAME(DT_INST_PHANDLE(n, binding))
    int32_t value_min;
    int32_t value_max;
    int32_t min_vel;
    int32_t max_vel;
    uint16_t poll_ms;
    uint8_t index; // value_store index
};

struct active_state {
    bool active;
    int16_t last_pos;
    uint8_t last_layer;

    uint32_t dir_param;    // original MOVE_* param1 (sign+axis)
    uint8_t value_index;

    uint32_t active_param; // currently pressed param (rebuilt)
    struct k_work_delayable poll_work;
};

struct behavior_mouse_move_dynamic_data {
    bool inited;
    const struct device *dev; // ★インスタンスごとの dev を保持
    struct active_state st;
};

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// dir_param は 16bit x/y を詰めた MOVE エンコードと同じ想定
static uint32_t build_param_from_dir_and_vel(uint32_t dir_param, int32_t vel_mag) {
    int16_t x = (int16_t)(dir_param & 0xFFFF);
    int16_t y = (int16_t)((dir_param >> 16) & 0xFFFF);

    int16_t xo = 0, yo = 0;
    if (x != 0) xo = (x > 0) ? (int16_t)vel_mag : (int16_t)(-vel_mag);
    if (y != 0) yo = (y > 0) ? (int16_t)vel_mag : (int16_t)(-vel_mag);

    return ((uint32_t)(uint16_t)yo << 16) | (uint32_t)(uint16_t)xo;
}

static int32_t value_to_velocity(const struct behavior_mouse_move_dynamic_config *cfg, int32_t value) {
    int32_t vmin = cfg->value_min;
    int32_t vmax = cfg->value_max;
    if (vmax == vmin) return cfg->max_vel;

    value = clamp_i32(value, vmin, vmax);

    // linear map: [vmin..vmax] -> [min_vel..max_vel]
    int64_t num = (int64_t)(value - vmin) * (cfg->max_vel - cfg->min_vel);
    int64_t den = (int64_t)(vmax - vmin);
    int32_t vel = cfg->min_vel + (int32_t)(num / den);

    return clamp_i32(vel, cfg->min_vel, cfg->max_vel);
}

static int enqueue_target(struct zmk_behavior_binding_event *event, const char *target,
                          bool pressed, uint32_t param) {
    struct zmk_behavior_binding b = {
        .behavior_dev = target,
        .param1 = param,
        .param2 = 0,
    };
    return zmk_behavior_queue_add(event, b, pressed, 0);
}

static void poll_tick(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);

    // ★ネストしてるのでまず active_state を取る
    struct active_state *st = CONTAINER_OF(dwork, struct active_state, poll_work);
    // ★さらに data を取る
    struct behavior_mouse_move_dynamic_data *data =
        CONTAINER_OF(st, struct behavior_mouse_move_dynamic_data, st);

    if (!st->active || data->dev == NULL) return;

    const struct behavior_mouse_move_dynamic_config *cfg = data->dev->config;

    int32_t val = zmk_value_store_get(st->value_index, cfg->value_min);
    int32_t vel = value_to_velocity(cfg, val);
    uint32_t next_param = build_param_from_dir_and_vel(st->dir_param, vel);

    if (next_param != st->active_param) {
        struct zmk_behavior_binding_event ev = {
            .position = st->last_pos,
            .layer = st->last_layer,
            .timestamp = (uint32_t)k_uptime_get(),
        };

        LOG_DBG("mmv dyn update idx=%d val=%d vel=%d", st->value_index, val, vel);

        // release old, press new
        enqueue_target(&ev, cfg->target_behavior, false, st->active_param);
        enqueue_target(&ev, cfg->target_behavior, true, next_param);

        st->active_param = next_param;
    }

    uint16_t ms = cfg->poll_ms ? cfg->poll_ms : 40;
    k_work_reschedule(&st->poll_work, K_MSEC(ms));
}

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_mouse_move_dynamic_config *cfg = dev->config;
    struct behavior_mouse_move_dynamic_data *data = dev->data;

    data->dev = dev;

    if (!data->inited) {
        k_work_init_delayable(&data->st.poll_work, poll_tick);
        data->inited = true;
    }

    data->st.active = true;
    data->st.last_pos = event.position;
    data->st.last_layer = event.layer;
    data->st.dir_param = binding->param1;
    data->st.value_index = cfg->index;

    int32_t val = zmk_value_store_get(data->st.value_index, cfg->value_min);
    int32_t vel = value_to_velocity(cfg, val);
    uint32_t param = build_param_from_dir_and_vel(data->st.dir_param, vel);

    data->st.active_param = param;

    LOG_DBG("mmv dyn press idx=%d val=%d vel=%d", data->st.value_index, val, vel);
    enqueue_target(&event, cfg->target_behavior, true, param);

    uint16_t ms = cfg->poll_ms ? cfg->poll_ms : 40;
    k_work_reschedule(&data->st.poll_work, K_MSEC(ms));

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_mouse_move_dynamic_config *cfg = dev->config;
    struct behavior_mouse_move_dynamic_data *data = dev->data;

    if (!data->st.active) return ZMK_BEHAVIOR_TRANSPARENT;

    LOG_DBG("mmv dyn release");
    k_work_cancel_delayable(&data->st.poll_work);

    enqueue_target(&event, cfg->target_behavior, false, data->st.active_param);

    data->st.active = false;
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
};

#define INST(n)                                                                 \
    static const struct behavior_mouse_move_dynamic_config cfg_##n = {          \
        .target_behavior = DEVICE_DT_NAME(DT_INST_PHANDLE(n, binding)),         \
        .value_min = DT_INST_PROP_OR(n, value_min, 0),                          \
        .value_max = DT_INST_PROP_OR(n, value_max, 100),                        \
        .min_vel = DT_INST_PROP_OR(n, min_vel, 200),                            \
        .max_vel = DT_INST_PROP_OR(n, max_vel, 1800),                           \
        .poll_ms = DT_INST_PROP_OR(n, poll_ms, 40),                             \
        .index = DT_INST_PROP_OR(n, index, 0),                                  \
    };                                                                          \
    static struct behavior_mouse_move_dynamic_data data_##n = {};               \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, &data_##n, &cfg_##n,                 \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,  \
                            &api);

DT_INST_FOREACH_STATUS_OKAY(INST)
