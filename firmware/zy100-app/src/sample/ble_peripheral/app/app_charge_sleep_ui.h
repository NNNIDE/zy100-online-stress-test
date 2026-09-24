#ifndef APP_CHARGE_SLEEP_UI_H
#define APP_CHARGE_SLEEP_UI_H

#include <stdbool.h>
#include <stdint.h>

/* Presentation only: this state never releases the battery safety latch. */
#define APP_CHARGE_LOW_BLINK_ON_MS 200U
#define APP_CHARGE_LOW_BLINK_OFF_MS 200U
#define APP_CHARGE_LOW_BLINK_CYCLES 5U
#define APP_CHARGE_LOW_BLINK_TOTAL_MS \
    ((APP_CHARGE_LOW_BLINK_ON_MS + APP_CHARGE_LOW_BLINK_OFF_MS) * APP_CHARGE_LOW_BLINK_CYCLES)

typedef struct
{
    uint32_t prompt_deadline_ms;
    bool active;
    bool sample_resolved;
    bool prompt_consumed;
    bool prompt_running;
    bool suppress_chase;
    bool display_started;
} app_charge_sleep_ui_t;

static inline void app_charge_sleep_ui_begin(app_charge_sleep_ui_t *ui)
{
    ui->active = true;
    ui->sample_resolved = false;
    ui->display_started = false;
}

static inline void app_charge_sleep_ui_sample(app_charge_sleep_ui_t *ui, bool valid)
{
    ui->sample_resolved = true;
    if (!valid) ui->suppress_chase = true;
}

static inline bool app_charge_sleep_ui_prompt_due(const app_charge_sleep_ui_t *ui, bool low)
{
    return ui->active && low && !ui->prompt_consumed;
}

/* Call only after the LED request succeeds; a full queue must not consume it. */
static inline void app_charge_sleep_ui_prompt_started(app_charge_sleep_ui_t *ui, uint32_t now)
{
    ui->prompt_consumed = true;
    ui->prompt_running = true;
    ui->suppress_chase = true;
    ui->display_started = false;
    ui->prompt_deadline_ms = now + APP_CHARGE_LOW_BLINK_TOTAL_MS;
}

static inline bool app_charge_sleep_ui_prompt_finished(app_charge_sleep_ui_t *ui, uint32_t now)
{
    if (!ui->prompt_running || (int32_t)(now - ui->prompt_deadline_ms) < 0) return false;
    ui->prompt_running = false;
    ui->sample_resolved = true;
    return true;
}

static inline void app_charge_sleep_ui_leave(app_charge_sleep_ui_t *ui)
{
    ui->active = false;
    ui->prompt_running = false;
}

static inline void app_charge_sleep_ui_unplug(app_charge_sleep_ui_t *ui)
{
    const app_charge_sleep_ui_t empty = {0};
    *ui = empty;
}

#endif
