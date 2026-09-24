#include "app_button.h"

#include <string.h>

typedef enum
{
    APP_BUTTON_FSM_IDLE = 0U,
    APP_BUTTON_FSM_PRESS_TRACKING,
    APP_BUTTON_FSM_FIRST_CLICK_PENDING,
    APP_BUTTON_FSM_DOUBLE_CLICK_CONSUMED,
    APP_BUTTON_FSM_SHUTDOWN_PREPARE,
    APP_BUTTON_FSM_SHUTDOWN_RELEASE_WAIT,
    APP_BUTTON_FSM_SPECIAL_MODE_IGNORE,
    APP_BUTTON_FSM_POWER_TRANSITION_SYNC,
} app_button_fsm_state_t;

typedef struct
{
    app_button_config_t config;
    bool initialized;
    bool level_valid;
    bool stable_pressed;
    bool suppress_until_release;
    bool tracking;
    bool wake_action_emitted;
    bool shutdown_action_emitted;
    volatile bool wake_irq_pending;
    app_button_profile_t press_profile;
    app_button_tier_t tier;
    uint64_t press_start_ms;
    uint64_t press_sample_ms;
    uint64_t last_edge_sample_ms;
    uint64_t last_transition_ms;
    bool transition_time_valid;
    bool click_pending;
    uint64_t click_release_ms;
    /* A second press is consumed immediately as a double click.  Keep the
     * matching release from being classified as another click/hold. */
    bool double_click_consumed;
    bool confirm_second;
    bool continue_hold;
    bool guard_release_seen;
    uint64_t guard_release_ms;
    bool release_suppressed;
    bool raw_pending;
    bool raw_pending_is_edge;
    bool raw_pending_short_tap;
    bool raw_pending_level;
    uint64_t raw_pending_timestamp_ms;
    uint64_t raw_pending_tap_start_ms;
    uint64_t raw_pending_tap_release_ms;
    app_button_fsm_state_t state;
} app_button_context_t;

static app_button_context_t s_button;

static bool app_button_config_valid(const app_button_config_t *config)
{
    return (config != NULL) &&
           (config->wake_hold_ms != 0U) &&
           (config->shutdown_hold_ms != 0U) &&
           (config->pairing_hold_ms > config->shutdown_hold_ms) &&
           (config->factory_hold_ms > config->pairing_hold_ms) &&
           (config->double_click_ms != 0U) &&
           (config->debounce_ms != 0U);
}

static app_button_tier_t app_button_tier_for_hold(app_button_profile_t profile,
                                                   uint64_t hold_ms)
{
    if (profile == APP_BUTTON_PROFILE_CAPTURE_SHUTDOWN_ONLY ||
        profile == APP_BUTTON_PROFILE_RESET_CONFIRM)
    {
        return (hold_ms >= (uint64_t)s_button.config.shutdown_hold_ms) ?
               APP_BUTTON_TIER_SHUTDOWN : APP_BUTTON_TIER_NONE;
    }

    if (profile != APP_BUTTON_PROFILE_POWERED_MULTI_LEVEL)
    {
        return APP_BUTTON_TIER_NONE;
    }
    if (hold_ms >= (uint64_t)s_button.config.factory_hold_ms)
        return APP_BUTTON_TIER_FACTORY_HINT;
    if (hold_ms >= (uint64_t)s_button.config.pairing_hold_ms)
    {
        return APP_BUTTON_TIER_PAIRING_HINT;
    }
    if (hold_ms >= (uint64_t)s_button.config.shutdown_hold_ms)
    {
        return APP_BUTTON_TIER_SHUTDOWN;
    }
    return APP_BUTTON_TIER_NONE;
}

static app_button_action_t app_button_action_for_tier(app_button_tier_t tier)
{
    switch (tier)
    {
    case APP_BUTTON_TIER_SHUTDOWN:
        return APP_BUTTON_ACTION_SHUTDOWN;
    case APP_BUTTON_TIER_PAIRING_HINT:
        return APP_BUTTON_ACTION_PAIRING_HINT;
    case APP_BUTTON_TIER_FACTORY_HINT:
        return APP_BUTTON_ACTION_FACTORY_HINT;
    case APP_BUTTON_TIER_NONE:
    default:
        return APP_BUTTON_ACTION_NONE;
    }
}

static void app_button_start_press(uint64_t runtime_ms,
                                   app_button_profile_t profile,
                                   app_button_event_t *event)
{
    s_button.tracking = true;
    s_button.state = APP_BUTTON_FSM_PRESS_TRACKING;
    s_button.wake_action_emitted = false;
    s_button.shutdown_action_emitted = false;
    s_button.press_profile = profile;
    s_button.tier = APP_BUTTON_TIER_NONE;
    s_button.double_click_consumed = false;
    s_button.release_suppressed = false;
    s_button.confirm_second = false;
    s_button.continue_hold = false;
    s_button.press_start_ms = runtime_ms;
    s_button.press_sample_ms = runtime_ms;
    event->press_started = true;
    event->profile = profile;
    event->press_start_ms = runtime_ms;
}

static bool app_button_consume_double_press(uint64_t runtime_ms,
                                            app_button_profile_t profile,
                                            app_button_event_t *event)
{
    if ((!s_button.click_pending) ||
        (runtime_ms < s_button.click_release_ms) ||
        ((runtime_ms - s_button.click_release_ms) >
         (uint64_t)s_button.config.double_click_ms))
    {
        return false;
    }

    app_button_clear_click();
    if (profile == APP_BUTTON_PROFILE_RESET_CONFIRM)
    {
        s_button.confirm_second = true;
        return true;
    }
    s_button.double_click_consumed = true;
    s_button.release_suppressed = true;
    s_button.state = APP_BUTTON_FSM_DOUBLE_CLICK_CONSUMED;
    event->action = APP_BUTTON_ACTION_DOUBLE_CLICK;
    event->profile = profile;
    event->press_start_ms = runtime_ms;
    return true;
}

static void app_button_finish_short_click(uint64_t runtime_ms,
                                          uint64_t press_start_ms,
                                          app_button_event_t *event)
{
    if (s_button.click_pending &&
        (press_start_ms >= s_button.click_release_ms) &&
        ((press_start_ms - s_button.click_release_ms) <=
         (uint64_t)s_button.config.double_click_ms))
    {
        s_button.click_pending = false;
        s_button.click_release_ms = 0U;
        s_button.state = APP_BUTTON_FSM_IDLE;
        event->action = APP_BUTTON_ACTION_DOUBLE_CLICK;
        return;
    }

    s_button.click_pending = true;
    s_button.click_release_ms = runtime_ms;
    s_button.state = APP_BUTTON_FSM_FIRST_CLICK_PENDING;
    event->action = APP_BUTTON_ACTION_SINGLE_CLICK;
}

bool app_button_init(const app_button_config_t *config)
{
    if (!app_button_config_valid(config))
    {
        return false;
    }

    memset(&s_button, 0, sizeof(s_button));
    s_button.config = *config;
    s_button.initialized = true;
    return true;
}

void app_button_fsm_reset(app_button_reset_reason_t reason)
{
    app_button_config_t config = s_button.config;
    bool initialized = s_button.initialized;

    (void)reason;

    memset(&s_button, 0, sizeof(s_button));
    s_button.config = config;
    s_button.initialized = initialized;
    s_button.state = APP_BUTTON_FSM_IDLE;
}

void app_button_clear_click(void)
{
    s_button.click_pending = false;
    s_button.click_release_ms = 0U;
}

void app_button_cancel_input(bool is_pressed)
{
    if (!s_button.initialized)
    {
        return;
    }

    app_button_clear_click();
    s_button.level_valid = true;
    s_button.stable_pressed = is_pressed;
    if (is_pressed) s_button.guard_release_seen = false;
    s_button.suppress_until_release = is_pressed;
    s_button.tracking = false;
    s_button.wake_action_emitted = false;
    s_button.tier = APP_BUTTON_TIER_NONE;
    s_button.double_click_consumed = false;
    s_button.release_suppressed = false;
    s_button.confirm_second = false;
    s_button.continue_hold = false;
    s_button.press_start_ms = 0U;
    s_button.press_sample_ms = 0U;
    s_button.transition_time_valid = false;
    s_button.last_transition_ms = 0U;
    s_button.last_edge_sample_ms = 0U;
    s_button.raw_pending = false;
    s_button.raw_pending_is_edge = false;
    s_button.raw_pending_short_tap = false;
    s_button.raw_pending_timestamp_ms = 0U;
    s_button.raw_pending_tap_start_ms = 0U;
    s_button.raw_pending_tap_release_ms = 0U;
    s_button.state = APP_BUTTON_FSM_POWER_TRANSITION_SYNC;
}

bool app_button_event_time(uint32_t edge_ms, uint32_t sample_tick_ms,
                           uint64_t sample_time_ms, uint64_t *event_ms)
{
    uint32_t age_ms = sample_tick_ms - edge_ms;
    if ((event_ms == NULL) || (age_ms > INT32_MAX) ||
        ((uint64_t)age_ms > sample_time_ms))
    {
        return false;
    }
    *event_ms = sample_time_ms - (uint64_t)age_ms;
    return true;
}

bool app_button_fsm_submit_edge(uint64_t timestamp_ms, bool is_pressed)
{
    if (!s_button.initialized)
    {
        return false;
    }

    s_button.raw_pending = true;
    s_button.raw_pending_is_edge = true;
    s_button.raw_pending_short_tap = false;
    s_button.raw_pending_level = is_pressed;
    s_button.raw_pending_timestamp_ms = timestamp_ms;
    return true;
}

bool app_button_fsm_submit_level(bool is_pressed)
{
    if (!s_button.initialized)
    {
        return false;
    }

    s_button.raw_pending = true;
    s_button.raw_pending_is_edge = false;
    s_button.raw_pending_short_tap = false;
    s_button.raw_pending_level = is_pressed;
    s_button.raw_pending_timestamp_ms = 0U;
    return true;
}

bool app_button_fsm_submit_short_tap(uint64_t press_start_ms,
                                     uint64_t release_ms)
{
    if ((!s_button.initialized) || (release_ms < press_start_ms))
    {
        return false;
    }

    s_button.raw_pending = true;
    s_button.raw_pending_is_edge = false;
    s_button.raw_pending_short_tap = true;
    s_button.raw_pending_level = false;
    s_button.raw_pending_timestamp_ms = 0U;
    s_button.raw_pending_tap_start_ms = press_start_ms;
    s_button.raw_pending_tap_release_ms = release_ms;
    return true;
}

bool app_button_fsm_submit_wake_irq(void)
{
    if (!s_button.initialized)
    {
        return false;
    }
    s_button.wake_irq_pending = true;
    return true;
}

void app_button_on_wake_irq(void)
{
    (void)app_button_fsm_submit_wake_irq();
}

void app_button_clear_wake_irq(void)
{
    s_button.wake_irq_pending = false;
}

bool app_button_wake_irq_pending(void)
{
    return s_button.wake_irq_pending;
}

bool app_button_fsm_tick(uint64_t runtime_ms,
                         app_button_profile_t profile,
                         app_button_event_t *event)
{
    bool wake_irq_seen;
    bool raw_pending;
    bool raw_is_edge;
    bool raw_short_tap;
    bool raw_level;
    uint64_t gesture_ms = runtime_ms;
    bool press_started = false;
    bool released = false;
    uint64_t hold_ms = 0U;
    uint64_t tap_start_ms = 0U;
    uint64_t tap_release_ms = 0U;
    app_button_tier_t tier = APP_BUTTON_TIER_NONE;

    if ((!s_button.initialized) || (event == NULL))
    {
        return false;
    }

    memset(event, 0, sizeof(*event));
    event->previous_state = (uint8_t)s_button.state;
    event->profile = profile;
    wake_irq_seen = s_button.wake_irq_pending;
    s_button.wake_irq_pending = false;
    event->wake_irq_seen = wake_irq_seen;

    raw_pending = s_button.raw_pending;
    raw_is_edge = s_button.raw_pending_is_edge;
    raw_short_tap = s_button.raw_pending_short_tap;
    raw_level = s_button.raw_pending_level;
    tap_start_ms = s_button.raw_pending_tap_start_ms;
    tap_release_ms = s_button.raw_pending_tap_release_ms;
    if (raw_pending)
    {
        s_button.raw_pending = false;
        s_button.raw_pending_short_tap = false;
        if (raw_short_tap)
        {
            if ((tap_start_ms > runtime_ms) || (tap_release_ms > runtime_ms))
            {
                event->cancel_reason = APP_BUTTON_CANCEL_FUTURE_TIME;
                event->input_time_ms = tap_release_ms;
                event->previous_transition_ms = s_button.last_transition_ms;
                app_button_cancel_input(false);
                event->action = APP_BUTTON_ACTION_INPUT_CANCELLED;
                return true;
            }
            event->profile = profile;
            event->press_start_ms = tap_start_ms;
            event->hold_ms = runtime_ms - tap_start_ms;
            event->released = true;
            if (s_button.click_pending &&
                (tap_start_ms >= s_button.click_release_ms) &&
                ((tap_start_ms - s_button.click_release_ms) <=
                 (uint64_t)s_button.config.double_click_ms))
            {
                app_button_clear_click();
                event->action = APP_BUTTON_ACTION_DOUBLE_CLICK;
            }
            else
            {
                s_button.click_pending = true;
                s_button.click_release_ms = tap_release_ms;
                event->action = APP_BUTTON_ACTION_SINGLE_CLICK;
            }
            s_button.level_valid = true;
            s_button.stable_pressed = false;
            s_button.tracking = false;
            s_button.wake_action_emitted = false;
            s_button.tier = APP_BUTTON_TIER_NONE;
            s_button.press_start_ms = 0U;
            s_button.last_transition_ms = tap_release_ms;
            s_button.last_edge_sample_ms = tap_release_ms;
            s_button.transition_time_valid = true;
            s_button.state = APP_BUTTON_FSM_FIRST_CLICK_PENDING;
            return true;
        }
        if (raw_is_edge)
        {
            gesture_ms = s_button.raw_pending_timestamp_ms;
        }
    }
    else
    {
        raw_level = s_button.level_valid ? s_button.stable_pressed : false;
    }

    /* The FSM owns the sampled level.  Tasks never pass a second, competing
     * level into the classifier. */
    {
        bool is_pressed = raw_level;

    event->input_time_ms = gesture_ms;
    event->previous_transition_ms = s_button.last_transition_ms;
    if (gesture_ms > runtime_ms)
    {
        app_button_cancel_input(is_pressed);
        event->cancel_reason = APP_BUTTON_CANCEL_FUTURE_TIME;
        event->action = APP_BUTTON_ACTION_INPUT_CANCELLED;
        return true;
    }
    /* A live sample can precede delivery of the same physical edge.  It
     * must not cancel the gesture, restart its hold, or expire its click. */
    if (raw_pending && raw_is_edge && s_button.level_valid &&
        s_button.transition_time_valid &&
        (raw_level == s_button.stable_pressed) &&
        (gesture_ms <= s_button.last_transition_ms))
    {
        return wake_irq_seen;
    }

    if ((s_button.transition_time_valid &&
         (gesture_ms < s_button.last_transition_ms)) ||
        (s_button.tracking && (gesture_ms < s_button.press_start_ms)) ||
        (s_button.click_pending && (gesture_ms < s_button.click_release_ms)))
    {
        app_button_cancel_input(is_pressed);
        event->cancel_reason = APP_BUTTON_CANCEL_TIME_REVERSAL;
        event->action = APP_BUTTON_ACTION_INPUT_CANCELLED;
        return true;
    }

    if (!s_button.level_valid)
    {
        s_button.level_valid = true;
        s_button.stable_pressed = is_pressed;
        press_started = is_pressed;
        s_button.last_transition_ms = gesture_ms;
        s_button.last_edge_sample_ms = gesture_ms;
        s_button.transition_time_valid = true;
    }
    else
    {
        press_started = (!s_button.stable_pressed) && is_pressed;
        released = s_button.stable_pressed && (!is_pressed);

        if ((press_started || released) &&
            s_button.transition_time_valid &&
            (gesture_ms >= s_button.last_transition_ms) &&
            ((gesture_ms - s_button.last_transition_ms) <
             (uint64_t)s_button.config.debounce_ms))
        {
            press_started = false;
            released = false;
        }
        else if (press_started || released)
        {
            s_button.stable_pressed = is_pressed;
            s_button.last_transition_ms = gesture_ms;
            s_button.last_edge_sample_ms = gesture_ms;
            s_button.transition_time_valid = true;
        }
    }

    if (s_button.click_pending &&
        ((gesture_ms - s_button.click_release_ms) >
         (uint64_t)s_button.config.double_click_ms))
    {
        app_button_clear_click();
    }

    if (press_started)
    {
        s_button.guard_release_seen = false;
        app_button_start_press(gesture_ms, profile, event);
        /* Confirm the gesture on the second press edge.  This deliberately
         * happens before hold-tier evaluation so a delayed task poll cannot
         * turn the second click into a shutdown. */
        (void)app_button_consume_double_press(gesture_ms, profile, event);
    }

    if (s_button.tracking &&
        (profile == APP_BUTTON_PROFILE_CAPTURE_SHUTDOWN_ONLY) &&
        (s_button.press_profile == APP_BUTTON_PROFILE_POWERED_MULTI_LEVEL))
    {
        s_button.press_profile = APP_BUTTON_PROFILE_CAPTURE_SHUTDOWN_ONLY;
    }

    if (s_button.tracking && is_pressed)
    {
        hold_ms = gesture_ms - s_button.press_start_ms;
        event->profile = s_button.press_profile;
        event->press_start_ms = s_button.press_start_ms;
        event->hold_ms = hold_ms;

        if ((!s_button.release_suppressed) &&
            (s_button.press_profile == APP_BUTTON_PROFILE_WAKE_ONLY) &&
            (!s_button.wake_action_emitted) &&
            (hold_ms >= (uint64_t)s_button.config.wake_hold_ms))
        {
            s_button.wake_action_emitted = true;
            event->action = APP_BUTTON_ACTION_WAKE;
        }

        tier = s_button.release_suppressed ? APP_BUTTON_TIER_NONE :
               app_button_tier_for_hold(s_button.press_profile, hold_ms);
        if ((!s_button.release_suppressed) && (tier != s_button.tier))
        {
            s_button.tier = tier;
            event->tier = tier;
            event->tier_changed = true;
            if (tier == APP_BUTTON_TIER_SHUTDOWN)
            {
                s_button.state = APP_BUTTON_FSM_SHUTDOWN_PREPARE;
            }
            else if ((tier == APP_BUTTON_TIER_PAIRING_HINT) ||
                     (tier == APP_BUTTON_TIER_FACTORY_HINT))
            {
                s_button.state = APP_BUTTON_FSM_SPECIAL_MODE_IGNORE;
            }
        }
    }

    /* A delayed pump may first observe the 5/10-second tier. The
     * shutdown request must still precede any visual-only hint. */
    if (s_button.tracking && !s_button.release_suppressed &&
        (s_button.press_profile != APP_BUTTON_PROFILE_WAKE_ONLY) &&
        !s_button.shutdown_action_emitted &&
        ((gesture_ms - s_button.press_start_ms) >= s_button.config.shutdown_hold_ms))
    {
        s_button.shutdown_action_emitted = true;
        event->action = APP_BUTTON_ACTION_SHUTDOWN_PREPARE;
    }

    if (released)
    {
        event->released = true;
        if (s_button.suppress_until_release)
        {
            s_button.suppress_until_release = false;
        }
        if (s_button.tracking)
        {
            hold_ms = gesture_ms - s_button.press_start_ms;
            event->profile = s_button.press_profile;
            event->press_start_ms = s_button.press_start_ms;
            event->hold_ms = hold_ms;
            tier = s_button.release_suppressed ? APP_BUTTON_TIER_NONE :
                   app_button_tier_for_hold(s_button.press_profile, hold_ms);
            event->tier = tier;

            if ((!s_button.release_suppressed) &&
                (s_button.press_profile != APP_BUTTON_PROFILE_WAKE_ONLY))
            {
                if (s_button.continue_hold)
                {
                    event->action = APP_BUTTON_ACTION_NONE;
                }
                else if (s_button.confirm_second && tier == APP_BUTTON_TIER_NONE)
                {
                    event->action = APP_BUTTON_ACTION_DOUBLE_CLICK;
                }
                else if (tier == APP_BUTTON_TIER_NONE)
                {
                    app_button_finish_short_click(gesture_ms,
                                                  s_button.press_start_ms,
                                                  event);
                }
                else
                {
                    app_button_clear_click();
                    event->action = app_button_action_for_tier(tier);
                }
                if (tier != APP_BUTTON_TIER_NONE && !s_button.continue_hold)
                {
                    event->action = APP_BUTTON_ACTION_SHUTDOWN_RELEASE_CONFIRMED;
                }
            }
        }
        s_button.tracking = false;
        s_button.wake_action_emitted = false;
        s_button.tier = APP_BUTTON_TIER_NONE;
        s_button.double_click_consumed = false;
        s_button.release_suppressed = false;
        s_button.press_start_ms = 0U;
        s_button.press_sample_ms = 0U;
        if (tier == APP_BUTTON_TIER_SHUTDOWN)
        {
            s_button.state = APP_BUTTON_FSM_SHUTDOWN_RELEASE_WAIT;
        }
        else
        {
            s_button.state = APP_BUTTON_FSM_IDLE;
        }
    }

    return event->press_started || event->released || event->tier_changed ||
           event->wake_irq_seen || (event->action != APP_BUTTON_ACTION_NONE);
    }
}

bool app_button_blocks_sleep(void)
{
    return s_button.wake_irq_pending || s_button.tracking;
}

bool app_button_is_tracking(void)
{
    return s_button.tracking;
}

bool app_button_is_wake_tracking(void)
{
    return s_button.tracking &&
           (s_button.press_profile == APP_BUTTON_PROFILE_WAKE_ONLY);
}

bool app_button_click_pending(void)
{
    return s_button.click_pending;
}

uint64_t app_button_press_start_ms(void)
{
    return s_button.tracking ? s_button.press_start_ms : 0U;
}

void app_button_continue_hold_without_release_action(void)
{
    if (s_button.tracking) s_button.continue_hold = true;
}

bool app_button_release_guard_ready(uint64_t now)
{
    if (s_button.stable_pressed)
    {
        s_button.guard_release_seen = false;
        return false;
    }
    if (!s_button.guard_release_seen || now < s_button.guard_release_ms)
    {
        s_button.guard_release_seen = true;
        s_button.guard_release_ms = now;
        return false;
    }
    return now - s_button.guard_release_ms >= s_button.config.debounce_ms;
}
