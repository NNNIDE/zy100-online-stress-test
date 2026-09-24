#ifndef APP_BUTTON_H
#define APP_BUTTON_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Application button classifier.
 *
 * This module owns gesture timing and emits semantic actions only. GPIO,
 * LED, capture, BLE, Flash and power-state transitions remain owned by their
 * existing ports and app-task orchestration.
 */
typedef enum
{
    APP_BUTTON_PROFILE_WAKE_ONLY = 0U,
    APP_BUTTON_PROFILE_POWERED_MULTI_LEVEL,
    APP_BUTTON_PROFILE_CAPTURE_SHUTDOWN_ONLY,
    APP_BUTTON_PROFILE_RESET_CONFIRM,
} app_button_profile_t;

typedef enum
{
    APP_BUTTON_TIER_NONE = 0U,
    APP_BUTTON_TIER_SHUTDOWN,
    APP_BUTTON_TIER_PAIRING_HINT,
    APP_BUTTON_TIER_FACTORY_HINT,
} app_button_tier_t;

typedef enum
{
    APP_BUTTON_ACTION_NONE = 0U,
    APP_BUTTON_ACTION_SINGLE_CLICK,
    APP_BUTTON_ACTION_DOUBLE_CLICK,
    APP_BUTTON_ACTION_SHUTDOWN,
    APP_BUTTON_ACTION_PAIRING_HINT,
    APP_BUTTON_ACTION_FACTORY_HINT,
    APP_BUTTON_ACTION_WAKE,
    APP_BUTTON_ACTION_SHUTDOWN_PREPARE,
    APP_BUTTON_ACTION_SHUTDOWN_RELEASE_CONFIRMED,
    APP_BUTTON_ACTION_INPUT_CANCELLED,
} app_button_action_t;

typedef enum
{
    APP_BUTTON_RESET_POWER_BOUNDARY = 0U,
    APP_BUTTON_RESET_EDGE_FIFO_OVERFLOW,
    APP_BUTTON_RESET_TIME_REVERSAL,
    APP_BUTTON_RESET_SPECIAL_MODE,
    APP_BUTTON_RESET_SHUTDOWN,
} app_button_reset_reason_t;

typedef struct
{
    uint32_t wake_hold_ms;
    uint32_t shutdown_hold_ms;
    uint32_t pairing_hold_ms;
    uint32_t factory_hold_ms;
    uint32_t double_click_ms;
    uint32_t debounce_ms;
} app_button_config_t;

typedef enum
{
    APP_BUTTON_CANCEL_NONE = 0U,
    APP_BUTTON_CANCEL_FUTURE_TIME,
    APP_BUTTON_CANCEL_TIME_REVERSAL,
} app_button_cancel_reason_t;

typedef struct
{
    app_button_action_t action;
    app_button_tier_t tier;
    app_button_profile_t profile;
    uint64_t press_start_ms;
    uint64_t hold_ms;
    app_button_cancel_reason_t cancel_reason;
    uint64_t input_time_ms;
    uint64_t previous_transition_ms;
    uint8_t previous_state; /* Internal FSM state before input processing. */
    bool press_started;
    bool released;
    bool tier_changed;
    bool wake_irq_seen;
} app_button_event_t;

bool app_button_init(const app_button_config_t *config);
void app_button_fsm_reset(app_button_reset_reason_t reason);
void app_button_clear_click(void);
void app_button_continue_hold_without_release_action(void);
bool app_button_release_guard_ready(uint64_t now);
void app_button_cancel_input(bool is_pressed);
/* Convert an ISR timestamp using a contemporaneous task-side snapshot.
 * No clock offset is retained between gestures. */
bool app_button_event_time(uint32_t edge_ms, uint32_t sample_tick_ms,
                           uint64_t sample_time_ms, uint64_t *event_ms);
bool app_button_fsm_submit_edge(uint64_t timestamp_ms, bool is_pressed);
bool app_button_fsm_submit_level(bool is_pressed);
/* Submit a synthetic tap observed while the MCU was in standby.  The tap is
 * still classified by app_button_fsm_tick(), just like a GPIO edge. */
bool app_button_fsm_submit_short_tap(uint64_t press_start_ms,
                                     uint64_t release_ms);
bool app_button_fsm_submit_wake_irq(void);
void app_button_on_wake_irq(void);
void app_button_clear_wake_irq(void);
bool app_button_wake_irq_pending(void);
/* The single semantic entry point. A DOUBLE_CLICK action is emitted on the
 * second press edge; its matching release is consumed without producing
 * another click or hold action. Raw edges/levels are submitted first. */
bool app_button_fsm_tick(uint64_t runtime_ms,
                         app_button_profile_t profile,
                         app_button_event_t *event);
bool app_button_blocks_sleep(void);
bool app_button_is_tracking(void);
bool app_button_is_wake_tracking(void);
bool app_button_click_pending(void);
uint64_t app_button_press_start_ms(void);

#endif
