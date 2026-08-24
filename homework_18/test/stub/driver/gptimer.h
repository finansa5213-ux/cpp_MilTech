#pragma once
#include <cstdint>
#include "esp_err.h"
typedef struct gptimer_t* gptimer_handle_t;
typedef enum { GPTIMER_CLK_SRC_DEFAULT = 0 } gptimer_clock_source_t;
typedef enum { GPTIMER_COUNT_UP = 0, GPTIMER_COUNT_DOWN } gptimer_count_direction_t;
typedef struct { uint64_t count_value; uint64_t alarm_value; } gptimer_alarm_event_data_t;
typedef bool (*gptimer_alarm_cb_t)(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*);
typedef struct { gptimer_alarm_cb_t on_alarm; } gptimer_event_callbacks_t;
typedef struct {
    gptimer_clock_source_t clk_src;
    gptimer_count_direction_t direction;
    uint32_t resolution_hz;
    int intr_priority;
    struct { uint32_t intr_shared: 1; } flags;
} gptimer_config_t;
typedef struct {
    uint64_t alarm_count;
    uint64_t reload_count;
    struct { uint32_t auto_reload_on_alarm: 1; } flags;
} gptimer_alarm_config_t;
esp_err_t gptimer_new_timer(const gptimer_config_t*, gptimer_handle_t*);
esp_err_t gptimer_register_event_callbacks(gptimer_handle_t, const gptimer_event_callbacks_t*, void*);
esp_err_t gptimer_enable(gptimer_handle_t);
esp_err_t gptimer_start(gptimer_handle_t);
esp_err_t gptimer_set_alarm_action(gptimer_handle_t, const gptimer_alarm_config_t*);
esp_err_t gptimer_set_raw_count(gptimer_handle_t, uint64_t);
