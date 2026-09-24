#include "svc_led_owner.h"

#include <string.h>

#include "os_sched.h"
#include "os_sync.h"
#include "trace.h"

#include "../app_flags.h"
#include "svc_led_pattern.h"
#include "svc_led_task.h"

#define LED_OWNER_INVALID_INDEX       ((uint8_t)LED_OWNER_COUNT)
#define LED_OWNER_BREATH_MIN_LEVEL    SVC_LED_TASK_BLUE_BREATH_MIN_LEVEL
#define LED_OWNER_BREATH_MAX_LEVEL    SVC_LED_TASK_BLUE_BREATH_MAX_LEVEL
#define LED_OWNER_CAPTURE_BREATH_CYCLE_MS 600U
#define LED_OWNER_CAPTURE_BREATH_GAP_MS   4000U
#define LED_OWNER_CAPTURE_BREATH_STEP_MS  SVC_LED_TASK_BLUE_BREATH_STEP_MS
#define LED_OWNER_CAPTURE_BREATH_MAX_LEVEL 76U
#define LED_OWNER_CHARGE_FADE_TOTAL_MS    SVC_LED_TASK_LOGO_FADE_TOTAL_MS
#define LED_OWNER_CHARGE_FADE_STEP_MS     SVC_LED_TASK_LOGO_FADE_STEP_MS
#define LED_OWNER_TRAINING_SAFE_MAX_LEVEL 76U
#define LED_OWNER_TRAINING_TARGET_NOTIFY  1U
#define LED_OWNER_TRAINING_TARGET_LOGO    2U
#define LED_OWNER_TRAINING_TARGET_ALL     3U
#define LED_OWNER_TRAINING_EFFECT_SOLID   1U
#define LED_OWNER_TRAINING_EFFECT_BLINK   2U
#define LED_OWNER_TRAINING_EFFECT_BREATH  3U
#define LED_OWNER_TRAINING_EFFECT_MARQUEE 4U
#define LED_OWNER_TRAINING_SPEED_CAPTURE  0U
#define LED_OWNER_TRAINING_SPEED_SLOW     1U
#define LED_OWNER_TRAINING_SPEED_STANDARD 2U
#define LED_OWNER_TRAINING_SPEED_FAST     3U

#if ZY100_LED_LOG_ENABLE
#define LED_OWNER_LOG(...)            DBG_DIRECT(__VA_ARGS__)
#else
#define LED_OWNER_LOG(...)            do { if (0) { DBG_DIRECT(__VA_ARGS__); } } while (0)
#endif

#if ZY100_LED_DIAG_LOG_ENABLE
#define LED_OWNER_DIAG_LOG(...)       DBG_DIRECT(__VA_ARGS__)
#else
#define LED_OWNER_DIAG_LOG(...)       do { if (0) { DBG_DIRECT(__VA_ARGS__); } } while (0)
#endif

typedef struct
{
    bool active;
    uint8_t priority;
    led_pattern_t pattern;
} led_owner_entry_t;

typedef struct
{
    led_owner_t owner;
    led_pattern_t pattern;
} led_owner_selected_t;

typedef enum
{
    LED_CHARGE_SEQUENCE_PHASE_CHASE = 0U,
    LED_CHARGE_SEQUENCE_PHASE_FADE,
    LED_CHARGE_SEQUENCE_PHASE_BREATH,
} led_charge_sequence_phase_t;

static led_owner_entry_t s_led_owner_entries[LED_OWNER_COUNT];
static led_owner_t s_led_current_owner = LED_OWNER_NONE;
static led_pattern_t s_led_current_pattern;
static bool s_led_current_valid = false;
static bool s_led_current_on = false;
static uint8_t s_led_current_cycles_done = 0U;
static uint32_t s_led_breath_elapsed_ms = 0U;
static led_charge_sequence_phase_t s_led_charge_sequence_phase =
    LED_CHARGE_SEQUENCE_PHASE_CHASE;
static uint8_t s_led_charge_chase_step = 0U;
static uint32_t s_led_charge_fade_elapsed_ms = 0U;
static bool s_led_charge_chase_consumed = false;
static bool s_led_capture_breath_gap = false;
static uint64_t s_led_current_start_ms = 0ULL;
static uint64_t s_led_current_last_ms = 0ULL;
static zy100_rgb_color_t s_led_show_frame[ZY100_RGB_LED_COUNT];
static led_owner_t s_led_show_frame_owner = LED_OWNER_NONE;
static uint16_t s_led_show_frame_token = 0U;
static uint8_t s_led_training_step = 0U;
static const uint8_t s_led_logo_order[ZY100_LED_LOGO_COUNT] =
    ZY100_LED_LOGO_ORDER_INIT;

static bool led_owner_valid(led_owner_t owner)
{
    return owner < LED_OWNER_COUNT;
}

static bool led_owner_request_valid(led_owner_t owner)
{
    return (owner > LED_OWNER_NONE) && (owner < LED_OWNER_COUNT);
}

static bool led_short_press_cancel_if_preempted_locked(led_owner_t owner,
                                                       uint8_t priority)
{
    if ((owner == LED_OWNER_SHORT_PRESS) ||
        (priority <= LED_PRIORITY_SHORT_PRESS) ||
        !s_led_owner_entries[LED_OWNER_SHORT_PRESS].active)
    {
        return false;
    }

    s_led_owner_entries[LED_OWNER_SHORT_PRESS].active = false;
    if (s_led_show_frame_owner == LED_OWNER_SHORT_PRESS)
    {
        s_led_show_frame_owner = LED_OWNER_NONE;
    }
    return true;
}

static bool led_owner_diag_interesting(led_owner_t owner)
{
    switch (owner)
    {
    case LED_OWNER_BUTTON_HOLD:
    case LED_OWNER_BOOT_SEQUENCE:
    case LED_OWNER_TRAINING_TRANSITION:
    case LED_OWNER_CHARGE_SLEEP:
    case LED_OWNER_BLE_LINK_EVENT:
    case LED_OWNER_ONLINE_FAULT:
    case LED_OWNER_ONLINE_TRANSITION:
    case LED_OWNER_OFFLINE_V2_TRANSITION:
    case LED_OWNER_OFFLINE_V2_FAULT:
    case LED_OWNER_CHARGE_IDLE:
    case LED_OWNER_ACTIVE_BLE_WAIT_TIMEOUT:
    case LED_OWNER_FIND_DEVICE:
    case LED_OWNER_STATUS_QUERY:
    case LED_OWNER_SHORT_PRESS:
        return true;
    default:
        return false;
    }
}

static const char *led_owner_name(led_owner_t owner)
{
    switch (owner)
    {
    case LED_OWNER_NONE:
        return "none";
    case LED_OWNER_BUTTON_HOLD:
        return "button_hold";
    case LED_OWNER_BOOT_SEQUENCE:
        return "boot";
    case LED_OWNER_OTA_UPLOAD_RESULT:
        return "ota_result";
    case LED_OWNER_TRAINING_TRANSITION:
        return "training";
    case LED_OWNER_BLE_EXPORT:
        return "ble_export";
    case LED_OWNER_BATTERY_CRITICAL:
        return "battery_critical";
    case LED_OWNER_FLASH_FULL_WAIT_UPLOAD:
        return "flash_full";
    case LED_OWNER_CHARGE_SLEEP:
        return "charge_sleep";
    case LED_OWNER_BLE_LINK_EVENT:
        return "ble_link";
    case LED_OWNER_CHARGE_IDLE:
        return "charge_idle";
    case LED_OWNER_ACTIVE_BLE_WAIT_TIMEOUT:
        return "active_ble_wait";
    case LED_OWNER_FIND_DEVICE:
        return "find_device";
    case LED_OWNER_STATUS_QUERY:
        return "status_query";
    case LED_OWNER_SHORT_PRESS:
        return "short_press";
    case LED_OWNER_CAPTURING:
        return "capturing";
    case LED_OWNER_MAG_CALIBRATION:
        return "mag_calibration";
    case LED_OWNER_CLEAR_FLASH:
        return "clear_flash";
    case LED_OWNER_PENDING_EXPORT:
        return "pending_export";
    case LED_OWNER_ONLINE_FAULT:
        return "online_fault";
    case LED_OWNER_ONLINE_TRANSITION:
        return "online_transition";
    case LED_OWNER_ONLINE_RUNNING:
        return "online_running";
    case LED_OWNER_OFFLINE_V2_TRANSITION:
        return "offline_v2_transition";
    case LED_OWNER_OFFLINE_V2_RUNNING:
        return "offline_v2_running";
    case LED_OWNER_OFFLINE_V2_FAULT:
        return "offline_v2_fault";
    default:
        return "invalid";
    }
}

static void led_clear_runtime_locked(void)
{
    memset(s_led_owner_entries, 0, sizeof(s_led_owner_entries));
    s_led_current_owner = LED_OWNER_NONE;
    memset(&s_led_current_pattern, 0, sizeof(s_led_current_pattern));
    s_led_current_pattern.id = LED_PATTERN_NONE;
    s_led_current_valid = false;
    s_led_current_on = false;
    s_led_current_cycles_done = 0U;
    s_led_breath_elapsed_ms = 0U;
    s_led_charge_sequence_phase = LED_CHARGE_SEQUENCE_PHASE_CHASE;
    s_led_charge_chase_step = 0U;
    s_led_charge_fade_elapsed_ms = 0U;
    s_led_charge_chase_consumed = false;
    s_led_capture_breath_gap = false;
    s_led_current_start_ms = 0ULL;
    s_led_current_last_ms = 0ULL;
    memset(s_led_show_frame, 0, sizeof(s_led_show_frame));
    s_led_show_frame_owner = LED_OWNER_NONE;
    s_led_show_frame_token = 0U;
    s_led_training_step = 0U;
}

static void led_pattern_reset(led_pattern_t *pattern, led_pattern_id_t id)
{
    if (pattern == NULL)
    {
        return;
    }

    memset(pattern, 0, sizeof(*pattern));
    pattern->id = id;
}

void led_pattern_none(led_pattern_t *pattern)
{
    led_pattern_reset(pattern, LED_PATTERN_NONE);
}

void led_pattern_notify_off(led_pattern_t *pattern, bool logo_off)
{
    led_pattern_reset(pattern, LED_PATTERN_NOTIFY_OFF);
    if (pattern != NULL)
    {
        pattern->logo_off = logo_off;
    }
}

void led_pattern_notify_solid(led_pattern_t *pattern,
                              led_pattern_color_t color,
                              bool logo_off)
{
    led_pattern_reset(pattern, LED_PATTERN_NOTIFY_SOLID);
    if (pattern != NULL)
    {
        pattern->color = color;
        pattern->logo_off = logo_off;
    }
}

void led_pattern_notify_blink(led_pattern_t *pattern,
                              led_pattern_color_t color,
                              uint32_t on_ms,
                              uint32_t off_ms,
                              uint8_t cycles,
                              bool logo_off)
{
    led_pattern_reset(pattern, LED_PATTERN_NOTIFY_BLINK);
    if (pattern != NULL)
    {
        pattern->color = color;
        pattern->on_ms = on_ms;
        pattern->off_ms = off_ms;
        pattern->cycles = cycles;
        pattern->logo_off = logo_off;
    }
}

void led_pattern_system_breath(led_pattern_t *pattern,
                               led_pattern_color_t color, uint8_t led_mask)
{
    led_pattern_reset(pattern, LED_PATTERN_SYSTEM_BREATH);
    if (pattern == NULL) return;
    pattern->color = color;
    pattern->step = led_mask;
}

void led_pattern_notify_blue_breath(led_pattern_t *pattern, bool logo_off)
{
    led_pattern_reset(pattern, LED_PATTERN_NOTIFY_BLUE_BREATH);
    if (pattern != NULL)
    {
        pattern->logo_off = logo_off;
    }
}

void led_pattern_capture_notify_blue_breath(led_pattern_t *pattern, bool logo_off)
{
    led_pattern_reset(pattern, LED_PATTERN_CAPTURE_NOTIFY_BLUE_BREATH);
    if (pattern != NULL)
    {
        pattern->logo_off = logo_off;
    }
}

void led_pattern_training_config(
    led_pattern_t *pattern,
    const led_training_pattern_config_t *config)
{
    led_pattern_reset(pattern, LED_PATTERN_TRAINING_CONFIG);
    if ((pattern != NULL) && (config != NULL))
    {
        pattern->training = *config;
    }
}

void led_pattern_notify_purple_breath(led_pattern_t *pattern, bool logo_off)
{
    led_pattern_reset(pattern, LED_PATTERN_NOTIFY_PURPLE_BREATH);
    if (pattern != NULL)
    {
        pattern->logo_off = logo_off;
    }
}

void led_pattern_logo_simple(led_pattern_t *pattern, led_pattern_id_t id)
{
    led_pattern_reset(pattern, id);
}

void led_pattern_find_device_logo_white(led_pattern_t *pattern, uint16_t token)
{
    led_pattern_reset(pattern, LED_PATTERN_FIND_DEVICE_LOGO_WHITE);
    if (pattern != NULL)
    {
        pattern->frame_token = token;
    }
}

void led_pattern_wake_logo_chase_step(led_pattern_t *pattern, uint8_t step)
{
    led_pattern_reset(pattern, LED_PATTERN_WAKE_LOGO_CHASE_STEP);
    if (pattern != NULL)
    {
        pattern->step = step;
    }
}

void led_pattern_all_off(led_pattern_t *pattern)
{
    led_pattern_reset(pattern, LED_PATTERN_ALL_OFF);
}

void led_pattern_charge_sequence(led_pattern_t *pattern)
{
    led_pattern_reset(pattern, LED_PATTERN_CHARGE_SEQUENCE);
}

void led_pattern_charge_breath(led_pattern_t *pattern)
{
    led_pattern_reset(pattern, LED_PATTERN_CHARGE_BREATH);
}

void led_pattern_charge_full_blue(led_pattern_t *pattern)
{
    led_pattern_reset(pattern, LED_PATTERN_CHARGE_FULL_BLUE);
}

void led_pattern_sleep_charge_breath(led_pattern_t *pattern)
{
    led_pattern_charge_sequence(pattern);
}

void led_pattern_sleep_charge_full(led_pattern_t *pattern)
{
    led_pattern_charge_full_blue(pattern);
}

void led_pattern_sleep_charge_off(led_pattern_t *pattern)
{
    led_pattern_reset(pattern, LED_PATTERN_SLEEP_CHARGE_OFF);
}

static bool led_pattern_equal(const led_pattern_t *left, const led_pattern_t *right)
{
    if ((left == NULL) || (right == NULL))
    {
        return false;
    }

    if ((left->id != right->id) ||
        (left->color != right->color) ||
        (left->logo_off != right->logo_off) ||
        (left->immediate_visible != right->immediate_visible) ||
        (left->on_ms != right->on_ms) ||
        (left->off_ms != right->off_ms) ||
        (left->hold_ms != right->hold_ms) ||
        (left->cycles != right->cycles) ||
        (left->step != right->step) ||
        (left->frame_count != right->frame_count) ||
        (left->frame_token != right->frame_token) ||
        (memcmp(&left->training, &right->training,
                sizeof(left->training)) != 0))
    {
        return false;
    }

    return true;
}

static bool led_pattern_log_equal(const led_pattern_t *left, const led_pattern_t *right)
{
    if ((left == NULL) || (right == NULL))
    {
        return false;
    }

    if ((left->id != right->id) ||
        (left->color != right->color) ||
        (left->logo_off != right->logo_off) ||
        (left->immediate_visible != right->immediate_visible) ||
        (left->on_ms != right->on_ms) ||
        (left->off_ms != right->off_ms) ||
        (left->hold_ms != right->hold_ms) ||
        (left->cycles != right->cycles) ||
        (left->step != right->step) ||
        (left->frame_count != right->frame_count) ||
        (memcmp(&left->training, &right->training,
                sizeof(left->training)) != 0))
    {
        return false;
    }

    return true;
}

static bool led_select_locked(led_owner_selected_t *selected)
{
    uint8_t idx;
    uint8_t best_idx = LED_OWNER_INVALID_INDEX;
    uint8_t best_priority = 0U;

    if (selected == NULL)
    {
        return false;
    }

    for (idx = (uint8_t)LED_OWNER_BUTTON_HOLD; idx < (uint8_t)LED_OWNER_COUNT; idx++)
    {
        if (!s_led_owner_entries[idx].active)
        {
            continue;
        }

        if ((best_idx == LED_OWNER_INVALID_INDEX) ||
            (s_led_owner_entries[idx].priority > best_priority) ||
            ((s_led_owner_entries[idx].priority == best_priority) &&
             ((led_owner_t)idx == s_led_current_owner)))
        {
            best_idx = idx;
            best_priority = s_led_owner_entries[idx].priority;
        }
    }

    if (best_idx == LED_OWNER_INVALID_INDEX)
    {
        selected->owner = LED_OWNER_NONE;
        led_pattern_none(&selected->pattern);
        return false;
    }

    selected->owner = (led_owner_t)best_idx;
    selected->pattern = s_led_owner_entries[best_idx].pattern;
    return true;
}

static bool led_selected_would_apply_locked(const led_owner_selected_t *selected,
                                            bool selected_valid)
{
    if (!selected_valid)
    {
        return s_led_current_valid;
    }

    if (!s_led_current_valid || (s_led_current_owner != selected->owner))
    {
        return true;
    }

    return !led_pattern_equal(&s_led_current_pattern, &selected->pattern);
}

static bool led_apply_notify_color(led_pattern_color_t color)
{
    switch (color)
    {
    case LED_PATTERN_COLOR_GREEN:
        return svc_led_pattern_notify_green_on();
    case LED_PATTERN_COLOR_BLUE:
        return svc_led_pattern_notify_blue_on();
    case LED_PATTERN_COLOR_RED:
        return svc_led_pattern_notify_red_on();
    case LED_PATTERN_COLOR_ORANGE:
        return svc_led_pattern_notify_orange_on();
    case LED_PATTERN_COLOR_WHITE:
        return svc_led_pattern_notify_rgb(ZY100_LED_NOTIFY_BRIGHTNESS,
                                          ZY100_LED_NOTIFY_BRIGHTNESS,
                                           ZY100_LED_NOTIFY_BRIGHTNESS);
    case LED_PATTERN_COLOR_PURPLE:
        return svc_led_pattern_notify_rgb(ZY100_LED_NOTIFY_BRIGHTNESS,
                                          0U,
                                          ZY100_LED_NOTIFY_BRIGHTNESS);
    case LED_PATTERN_COLOR_NONE:
    default:
        return svc_led_pattern_notify_off();
    }
}

static uint8_t led_blue_breath_level(uint32_t elapsed_ms)
{
    const uint32_t period_ms = (uint32_t)SVC_LED_TASK_BLUE_BREATH_PERIOD_MS;
    const uint32_t half_ms = period_ms / 2U;
    uint32_t phase_ms;
    uint32_t ramp_ms;
    uint32_t delta;
    uint32_t level;

    if ((period_ms == 0U) || (half_ms == 0U) ||
        (LED_OWNER_BREATH_MAX_LEVEL <= LED_OWNER_BREATH_MIN_LEVEL))
    {
        return (uint8_t)LED_OWNER_BREATH_MAX_LEVEL;
    }

    phase_ms = elapsed_ms % period_ms;
    ramp_ms = (phase_ms <= half_ms) ? phase_ms : (period_ms - phase_ms);
    delta = (uint32_t)LED_OWNER_BREATH_MAX_LEVEL -
            (uint32_t)LED_OWNER_BREATH_MIN_LEVEL;
    level = (uint32_t)LED_OWNER_BREATH_MIN_LEVEL +
            ((delta * ramp_ms) / half_ms);

    if (level > (uint32_t)LED_OWNER_BREATH_MAX_LEVEL)
    {
        level = (uint32_t)LED_OWNER_BREATH_MAX_LEVEL;
    }

    return (uint8_t)level;
}

#if !ZY100_BUILD_PRODUCTION
static bool led_apply_breath_level(uint32_t elapsed_ms)
{
    return svc_led_pattern_notify_rgb(0U, 0U, led_blue_breath_level(elapsed_ms));
}
#endif

#if !ZY100_BUILD_PRODUCTION
static bool led_apply_purple_breath_level(uint32_t elapsed_ms)
{
    uint8_t level = led_blue_breath_level(elapsed_ms);
    return svc_led_pattern_notify_rgb(level, 0U, level);
}
#endif

static uint8_t led_capture_breath_level(uint32_t elapsed_ms)
{
    const uint32_t period_ms = (uint32_t)LED_OWNER_CAPTURE_BREATH_CYCLE_MS;
    const uint32_t half_ms = period_ms / 2U;
    uint32_t phase_ms;
    uint32_t ramp_ms;
    uint32_t level;

    if ((period_ms == 0U) || (half_ms == 0U))
    {
        return (uint8_t)LED_OWNER_CAPTURE_BREATH_MAX_LEVEL;
    }

    phase_ms = elapsed_ms % period_ms;
    ramp_ms = (phase_ms <= half_ms) ? phase_ms : (period_ms - phase_ms);
    level = ((uint32_t)LED_OWNER_CAPTURE_BREATH_MAX_LEVEL * ramp_ms) / half_ms;
    if (level > (uint32_t)LED_OWNER_CAPTURE_BREATH_MAX_LEVEL)
    {
        level = (uint32_t)LED_OWNER_CAPTURE_BREATH_MAX_LEVEL;
    }

    return (uint8_t)level;
}

static bool led_apply_capture_breath_level(uint32_t elapsed_ms)
{
    return svc_led_pattern_notify_rgb(0U, 0U, led_capture_breath_level(elapsed_ms));
}

static uint32_t led_training_active_ms(
    const led_training_pattern_config_t *config)
{
    if (config->effect == LED_OWNER_TRAINING_EFFECT_BLINK)
    {
        if (config->speed == LED_OWNER_TRAINING_SPEED_SLOW) return 800U;
        if (config->speed == LED_OWNER_TRAINING_SPEED_STANDARD) return 400U;
        return 200U;
    }
    if (config->effect == LED_OWNER_TRAINING_EFFECT_BREATH)
    {
        if (config->speed == LED_OWNER_TRAINING_SPEED_SLOW) return 1200U;
        if (config->speed == LED_OWNER_TRAINING_SPEED_STANDARD) return 900U;
        return 600U;
    }
    if (config->effect == LED_OWNER_TRAINING_EFFECT_MARQUEE)
    {
        if (config->speed == LED_OWNER_TRAINING_SPEED_SLOW) return 250U;
        if (config->speed == LED_OWNER_TRAINING_SPEED_STANDARD) return 150U;
        return 80U;
    }
    return 0U;
}

static uint32_t led_training_gap_ms(
    const led_training_pattern_config_t *config)
{
    if (config->effect == LED_OWNER_TRAINING_EFFECT_BLINK)
    {
        return led_training_active_ms(config);
    }
    if (config->effect != LED_OWNER_TRAINING_EFFECT_BREATH)
    {
        return 0U;
    }
    if ((config->speed == LED_OWNER_TRAINING_SPEED_CAPTURE) ||
        (config->speed == LED_OWNER_TRAINING_SPEED_SLOW))
    {
        return 4000U;
    }
    if (config->speed == LED_OWNER_TRAINING_SPEED_STANDARD) return 2000U;
    return 1000U;
}

static uint8_t led_training_max_level(
    const led_training_pattern_config_t *config)
{
    return (uint8_t)(((uint32_t)LED_OWNER_TRAINING_SAFE_MAX_LEVEL *
                      (uint32_t)config->brightness_percent) / 100U);
}

static zy100_rgb_color_t led_training_color(
    const led_training_pattern_config_t *config,
    uint8_t level)
{
    zy100_rgb_color_t color;

    color.red = (uint8_t)(((uint32_t)config->red * level) / 255U);
    color.green = (uint8_t)(((uint32_t)config->green * level) / 255U);
    color.blue = (uint8_t)(((uint32_t)config->blue * level) / 255U);
    return color;
}

static uint8_t led_training_sequence_index(
    const led_training_pattern_config_t *config,
    uint8_t step)
{
    uint8_t count = (config->target == LED_OWNER_TRAINING_TARGET_LOGO) ?
                    ZY100_LED_LOGO_COUNT : ZY100_RGB_LED_COUNT;
    uint8_t logical = (uint8_t)(step % count);

    if (config->reverse)
    {
        logical = (uint8_t)(count - 1U - logical);
    }
    if (logical < ZY100_LED_LOGO_COUNT)
    {
        return s_led_logo_order[logical];
    }
    return logical;
}

static bool led_training_show(uint8_t level, bool marquee)
{
    const led_training_pattern_config_t *config =
        &s_led_current_pattern.training;
    zy100_rgb_color_t frame[ZY100_RGB_LED_COUNT];
    zy100_rgb_color_t color = led_training_color(config, level);
    uint8_t index;

    memset(frame, 0, sizeof(frame));
    if (marquee)
    {
        index = led_training_sequence_index(config, s_led_training_step);
        frame[index] = color;
    }
    else if (config->target == LED_OWNER_TRAINING_TARGET_NOTIFY)
    {
        for (index = ZY100_LED_LOGO_COUNT;
             index < ZY100_RGB_LED_COUNT; index++)
        {
            frame[index] = color;
        }
    }
    else if (config->target == LED_OWNER_TRAINING_TARGET_LOGO)
    {
        for (index = 0U; index < ZY100_LED_LOGO_COUNT; index++)
        {
            frame[index] = color;
        }
    }
    else
    {
        for (index = 0U; index < ZY100_RGB_LED_COUNT; index++)
        {
            frame[index] = color;
        }
    }
    return svc_led_pattern_show_frame(frame, ZY100_RGB_LED_COUNT);
}

static uint8_t led_training_breath_level(uint32_t elapsed_ms)
{
    const led_training_pattern_config_t *config =
        &s_led_current_pattern.training;
    uint32_t active_ms = led_training_active_ms(config);
    uint32_t half_ms = active_ms / 2U;
    uint32_t ramp_ms;
    uint32_t level;

    if ((active_ms == 0U) || (half_ms == 0U))
    {
        return led_training_max_level(config);
    }
    ramp_ms = (elapsed_ms <= half_ms) ? elapsed_ms :
              ((elapsed_ms < active_ms) ? (active_ms - elapsed_ms) : 0U);
#if ZY100_BUILD_PRODUCTION
    level = ((uint32_t)led_training_max_level(config) * ramp_ms + half_ms / 2U) / half_ms;
#else
    level = ((uint32_t)led_training_max_level(config) * ramp_ms) / half_ms;
#endif
    return (uint8_t)level;
}

static bool led_apply_training_pattern(void)
{
    const led_training_pattern_config_t *config =
        &s_led_current_pattern.training;

    s_led_training_step = 0U;
    if (!config->enabled)
    {
        svc_led_pattern_shutdown_for_sleep();
        return true;
    }
    if (config->effect == LED_OWNER_TRAINING_EFFECT_BREATH)
    {
        s_led_current_on = true;
        return led_training_show(0U, false);
    }
    s_led_current_on = true;
    return led_training_show(led_training_max_level(config),
                             config->effect ==
                             LED_OWNER_TRAINING_EFFECT_MARQUEE);
}

static uint32_t led_charge_fade_total_ms(void)
{
    const uint32_t total_ms = (uint32_t)LED_OWNER_CHARGE_FADE_TOTAL_MS;

    return (total_ms == 0U) ? 1U : total_ms;
}

static uint32_t led_charge_fade_step_ms(void)
{
    const uint32_t step_ms = (uint32_t)LED_OWNER_CHARGE_FADE_STEP_MS;

    return (step_ms == 0U) ? 1U : step_ms;
}

static uint8_t led_charge_fade_level(uint32_t elapsed_ms)
{
    const uint32_t total_ms = led_charge_fade_total_ms();
    const uint32_t min_level = (uint32_t)LED_OWNER_BREATH_MIN_LEVEL;
    const uint32_t max_level = (uint32_t)LED_OWNER_BREATH_MAX_LEVEL;
    uint32_t delta;
    uint32_t remaining_ms;
    uint32_t level;

    if ((elapsed_ms >= total_ms) || (max_level <= min_level))
    {
        return (uint8_t)min_level;
    }

    delta = max_level - min_level;
    remaining_ms = total_ms - elapsed_ms;
    level = min_level + ((delta * remaining_ms) / total_ms);
    if (level > max_level)
    {
        level = max_level;
    }

    return (uint8_t)level;
}

static bool led_apply_charge_breath_start(void)
{
    s_led_charge_sequence_phase = LED_CHARGE_SEQUENCE_PHASE_BREATH;
    s_led_charge_chase_step = ZY100_LED_LOGO_COUNT;
    s_led_charge_fade_elapsed_ms = 0U;
    s_led_breath_elapsed_ms = 0U;
    return svc_led_task_charge_sequence_breath_step(
               led_blue_breath_level(s_led_breath_elapsed_ms));
}

static bool led_apply_charge_sequence_or_breath(void)
{
    if (s_led_charge_chase_consumed)
    {
        return led_apply_charge_breath_start();
    }

    s_led_charge_chase_consumed = true;
    return svc_led_task_charge_sequence_start();
}

static bool led_pattern_is_notify(led_pattern_id_t id)
{
    switch (id)
    {
    case LED_PATTERN_NOTIFY_OFF:
    case LED_PATTERN_NOTIFY_SOLID:
    case LED_PATTERN_NOTIFY_BLINK:
    case LED_PATTERN_NOTIFY_BLUE_BREATH:
    case LED_PATTERN_CAPTURE_NOTIFY_BLUE_BREATH:
    case LED_PATTERN_NOTIFY_PURPLE_BREATH:
        return true;
    default:
        return false;
    }
}

#if ZY100_BUILD_PRODUCTION
static bool led_apply_system_breath(led_pattern_color_t color, uint8_t mask)
{
    uint8_t red = 0U, green = 0U, blue = 0U;
    switch (color)
    {
    case LED_PATTERN_COLOR_WHITE: red = green = blue = 255U; break;
    case LED_PATTERN_COLOR_RED: red = 255U; break;
    case LED_PATTERN_COLOR_GREEN: green = 255U; break;
    case LED_PATTERN_COLOR_BLUE: blue = 255U; break;
    case LED_PATTERN_COLOR_PURPLE: red = blue = 255U; break;
    case LED_PATTERN_COLOR_ORANGE: red = 255U; green = 127U; break;
    default: return false;
    }
    return svc_led_task_system_breath(red, green, blue, mask);
}
#endif

static bool led_apply_pattern(const led_pattern_t *pattern, uint64_t now_ms)
{
    bool ok = true;

    if (pattern == NULL)
    {
        return false;
    }

    s_led_current_start_ms = now_ms;
    s_led_current_last_ms = now_ms;
    s_led_current_on = false;
    s_led_current_cycles_done = 0U;
    s_led_breath_elapsed_ms = 0U;
    s_led_charge_sequence_phase = LED_CHARGE_SEQUENCE_PHASE_CHASE;
    s_led_charge_chase_step = 0U;
    s_led_charge_fade_elapsed_ms = 0U;
    s_led_capture_breath_gap = false;
    s_led_training_step = 0U;
    svc_led_task_cancel_sequence();

    /* Notify is a complete foreground indication.  Its D7-D8 update must
     * not retain a logo frame from a preempted owner such as charging. */
    if (pattern->logo_off || led_pattern_is_notify(pattern->id))
    {
        ok = svc_led_pattern_logo_off();
    }

    switch (pattern->id)
    {
    case LED_PATTERN_NOTIFY_OFF:
        ok = svc_led_pattern_notify_off() && ok;
        break;
    case LED_PATTERN_NOTIFY_SOLID:
        ok = led_apply_notify_color(pattern->color) && ok;
        s_led_current_on = true;
        break;
    case LED_PATTERN_NOTIFY_BLINK:
        ok = led_apply_notify_color(pattern->color) && ok;
        s_led_current_on = true;
        break;
    case LED_PATTERN_NOTIFY_BLUE_BREATH:
#if ZY100_BUILD_PRODUCTION
        ok = led_apply_system_breath(LED_PATTERN_COLOR_BLUE, LED_MASK_NOTIFY) && ok;
#else
        ok = led_apply_breath_level(0U) && ok;
#endif
        s_led_current_on = true;
        break;
    case LED_PATTERN_CAPTURE_NOTIFY_BLUE_BREATH:
        ok = led_apply_capture_breath_level(0U) && ok;
        s_led_current_on = true;
        break;
    case LED_PATTERN_TRAINING_CONFIG:
        ok = led_apply_training_pattern() && ok;
        break;
    case LED_PATTERN_NOTIFY_PURPLE_BREATH:
#if ZY100_BUILD_PRODUCTION
        ok = led_apply_system_breath(LED_PATTERN_COLOR_PURPLE, LED_MASK_NOTIFY) && ok;
#else
        ok = led_apply_purple_breath_level(0U) && ok;
#endif
        s_led_current_on = true;
        break;
    case LED_PATTERN_LOGO_OFF:
        ok = svc_led_task_logo_off() && ok;
        break;
    case LED_PATTERN_LOGO_WHITE_HINT:
        ok = svc_led_task_logo_white_hint_immediate() && ok;
        break;
    case LED_PATTERN_LOGO_WHITE:
        ok = svc_led_task_logo_white_on() && ok;
        break;
    case LED_PATTERN_FIND_DEVICE_LOGO_WHITE:
        ok = svc_led_pattern_all_off() && ok;
        ok = svc_led_task_logo_white_on() && ok;
        break;
    case LED_PATTERN_LOGO_BLUE:
        ok = svc_led_task_logo_blue_on() && ok;
        break;
    case LED_PATTERN_LOGO_FADE_OUT:
        ok = svc_led_task_logo_fade_out() && ok;
        break;
    case LED_PATTERN_WAKE_LOGO_CHASE_STEP:
        ok = svc_led_task_wake_logo_chase_step(pattern->step) && ok;
        break;
    case LED_PATTERN_ALL_OFF:
        ok = svc_led_task_all_off() && ok;
        break;
    case LED_PATTERN_SHOW_FRAME:
        ok = (s_led_show_frame_owner == s_led_current_owner) &&
             (pattern->frame_token == s_led_show_frame_token) &&
             (pattern->frame_count == ZY100_RGB_LED_COUNT) &&
             svc_led_pattern_show_frame(s_led_show_frame, pattern->frame_count) &&
             ok;
        break;
    case LED_PATTERN_CHARGE_SEQUENCE:
    case LED_PATTERN_SLEEP_CHARGE_BREATH:
        ok = led_apply_charge_sequence_or_breath() && ok;
        s_led_current_on = true;
        break;
    case LED_PATTERN_CHARGE_BREATH:
        s_led_charge_chase_consumed = true;
        ok = led_apply_charge_breath_start() && ok;
        s_led_current_on = true;
        break;
    case LED_PATTERN_CHARGE_FULL_BLUE:
    case LED_PATTERN_SLEEP_CHARGE_FULL:
        s_led_charge_chase_consumed = true;
        ok = svc_led_task_charge_full_blue_on() && ok;
        s_led_current_on = true;
        break;
#if ZY100_BUILD_PRODUCTION
    case LED_PATTERN_SYSTEM_BREATH:
        ok = led_apply_system_breath(pattern->color, pattern->step) && ok;
        break;
#endif
    case LED_PATTERN_SLEEP_CHARGE_OFF:
        ok = svc_led_task_sleep_charge_latch_off() && ok;
        break;
    case LED_PATTERN_NONE:
    default:
        ok = true;
        break;
    }

    LED_OWNER_LOG("[LED_OWNER] apply owner=%u pattern=%u ok=%u",
                  (uint32_t)s_led_current_owner,
                  (uint32_t)pattern->id,
                  ok ? 1U : 0U);
    return ok;
}

static bool led_apply_selected_if_changed(const led_owner_selected_t *selected,
                                          bool selected_valid,
                                          uint64_t now_ms)
{
    led_owner_t next_owner = selected_valid ? selected->owner : LED_OWNER_NONE;
    const led_pattern_t *next_pattern = selected_valid ? &selected->pattern : NULL;

    if (!selected_valid)
    {
        /* Failed pattern application may already have changed the output. */
        if (s_led_current_valid || (s_led_current_owner != LED_OWNER_NONE))
        {
            svc_led_task_cancel_sequence();
            if (!svc_led_task_all_off())
            {
                return false; /* Keep the transition pending for a retry. */
            }
            s_led_current_owner = LED_OWNER_NONE;
            s_led_current_valid = false;
            led_pattern_none(&s_led_current_pattern);
        }
        return true;
    }

    if (s_led_current_valid &&
        (s_led_current_owner == next_owner) &&
        led_pattern_equal(&s_led_current_pattern, next_pattern))
    {
        return true;
    }

    s_led_current_owner = next_owner;
    s_led_current_pattern = *next_pattern;
    s_led_current_valid = true;
#if ZY100_BUILD_PRODUCTION
    if (!led_apply_pattern(&s_led_current_pattern, now_ms))
    {
        s_led_current_valid = false;
        return false;
    }
#else
    (void)led_apply_pattern(&s_led_current_pattern, now_ms);
#endif
    return true;
}

static bool led_has_same_priority_conflict_locked(led_owner_t owner, uint8_t priority)
{
    uint8_t idx;

    for (idx = 0U; idx < (uint8_t)LED_OWNER_COUNT; idx++)
    {
        if (((led_owner_t)idx != owner) &&
            s_led_owner_entries[idx].active &&
            (s_led_owner_entries[idx].priority == priority))
        {
            return true;
        }
    }

    return false;
}

bool led_request_frame(led_owner_t owner,
                       uint8_t priority,
                       const zy100_rgb_color_t *frame,
                       uint16_t count)
{
    led_owner_selected_t selected;
    led_pattern_t pattern;
    bool selected_valid;
    bool request_changed_for_log;
    bool selected_will_apply;
    bool was_active;
    bool short_press_canceled;
    led_owner_t current_before;
    led_pattern_t old_pattern;
    uint8_t old_priority = LED_PRIORITY_NONE;
    uint8_t selected_priority = LED_PRIORITY_NONE;
    led_pattern_id_t selected_pattern = LED_PATTERN_NONE;
    uint32_t lock_state;

    if (!led_owner_request_valid(owner) || (frame == NULL) || (count != ZY100_RGB_LED_COUNT))
    {
        return false;
    }

    led_pattern_reset(&pattern, LED_PATTERN_SHOW_FRAME);
    pattern.frame_count = count;

    lock_state = os_lock();
    if (led_has_same_priority_conflict_locked(owner, priority))
    {
        current_before = s_led_current_owner;
        os_unlock(lock_state);
        LED_OWNER_DIAG_LOG("[LED_OWNER][REQ_REJECT] owner=%s(%u) prio=%u pattern=%u reason=same_priority_conflict current=%s(%u)",
                           led_owner_name(owner),
                           (uint32_t)owner,
                           (uint32_t)priority,
                           (uint32_t)pattern.id,
                           led_owner_name(current_before),
                           (uint32_t)current_before);
        return false;
    }

    current_before = s_led_current_owner;
    was_active = s_led_owner_entries[owner].active;
    old_priority = s_led_owner_entries[owner].priority;
    old_pattern = s_led_owner_entries[owner].pattern;
    s_led_show_frame_token++;
    if (s_led_show_frame_token == 0U)
    {
        s_led_show_frame_token++;
    }
    memcpy(s_led_show_frame, frame, sizeof(s_led_show_frame));
    s_led_show_frame_owner = owner;
    pattern.frame_token = s_led_show_frame_token;

    s_led_owner_entries[owner].active = true;
    s_led_owner_entries[owner].priority = priority;
    s_led_owner_entries[owner].pattern = pattern;
    short_press_canceled =
        led_short_press_cancel_if_preempted_locked(owner, priority);
    selected_valid = led_select_locked(&selected);
    if (selected_valid)
    {
        selected_priority = s_led_owner_entries[selected.owner].priority;
        selected_pattern = selected.pattern.id;
    }
    request_changed_for_log = !was_active ||
                              (old_priority != priority) ||
                              !led_pattern_log_equal(&old_pattern, &pattern);
    selected_will_apply = led_selected_would_apply_locked(&selected,
                                                          selected_valid);
    os_unlock(lock_state);

    if (short_press_canceled)
    {
        DBG_DIRECT("[SHORT_PRESS_LED] cancel reason=preempted owner=%s(%u) priority=%u",
                   led_owner_name(owner),
                   (uint32_t)owner,
                   (uint32_t)priority);
    }
    if (request_changed_for_log &&
        selected_will_apply &&
        (led_owner_diag_interesting(owner) ||
         (selected_valid && led_owner_diag_interesting(selected.owner))))
    {
        LED_OWNER_DIAG_LOG("[LED_OWNER][REQ] owner=%s(%u) prio=%u pattern=%u selected=%s(%u) selected_prio=%u selected_pattern=%u current_before=%s(%u) frame=1",
                           led_owner_name(owner),
                           (uint32_t)owner,
                           (uint32_t)priority,
                           (uint32_t)pattern.id,
                           led_owner_name(selected_valid ? selected.owner : LED_OWNER_NONE),
                           selected_valid ? (uint32_t)selected.owner : 0U,
                           (uint32_t)selected_priority,
                           (uint32_t)selected_pattern,
                           led_owner_name(current_before),
                           (uint32_t)current_before);
    }
#if !ZY100_LED_DIAG_LOG_ENABLE
    (void)current_before;
    (void)selected_priority;
    (void)selected_pattern;
#endif
    return led_apply_selected_if_changed(&selected,
                                          selected_valid,
                                          os_sys_time_get());
}

bool led_request(led_owner_t owner, uint8_t priority, const led_pattern_t *pattern)
{
    led_owner_selected_t selected;
    bool selected_valid;
    bool request_changed_for_log;
    bool selected_will_apply;
    bool was_active;
    bool short_press_canceled;
    led_owner_t current_before;
    led_pattern_t old_pattern;
    uint8_t old_priority = LED_PRIORITY_NONE;
    uint8_t selected_priority = LED_PRIORITY_NONE;
    led_pattern_id_t selected_pattern = LED_PATTERN_NONE;
    uint32_t lock_state;

    if (!led_owner_request_valid(owner) || (pattern == NULL))
    {
        return false;
    }

    lock_state = os_lock();
    if (led_has_same_priority_conflict_locked(owner, priority))
    {
        current_before = s_led_current_owner;
        os_unlock(lock_state);
        LED_OWNER_DIAG_LOG("[LED_OWNER][REQ_REJECT] owner=%s(%u) prio=%u pattern=%u reason=same_priority_conflict current=%s(%u)",
                           led_owner_name(owner),
                           (uint32_t)owner,
                           (uint32_t)priority,
                           (uint32_t)pattern->id,
                           led_owner_name(current_before),
                           (uint32_t)current_before);
        return false;
    }

    current_before = s_led_current_owner;
    was_active = s_led_owner_entries[owner].active;
    old_priority = s_led_owner_entries[owner].priority;
    old_pattern = s_led_owner_entries[owner].pattern;
    if (owner == s_led_show_frame_owner)
    {
        s_led_show_frame_owner = LED_OWNER_NONE;
    }

    s_led_owner_entries[owner].active = true;
    s_led_owner_entries[owner].priority = priority;
    s_led_owner_entries[owner].pattern = *pattern;
    short_press_canceled =
        led_short_press_cancel_if_preempted_locked(owner, priority);
    selected_valid = led_select_locked(&selected);
    if (selected_valid)
    {
        selected_priority = s_led_owner_entries[selected.owner].priority;
        selected_pattern = selected.pattern.id;
    }
    request_changed_for_log = !was_active ||
                              (old_priority != priority) ||
                              !led_pattern_log_equal(&old_pattern, pattern);
    selected_will_apply = led_selected_would_apply_locked(&selected,
                                                          selected_valid);
    os_unlock(lock_state);

    if (short_press_canceled)
    {
        DBG_DIRECT("[SHORT_PRESS_LED] cancel reason=preempted owner=%s(%u) priority=%u",
                   led_owner_name(owner),
                   (uint32_t)owner,
                   (uint32_t)priority);
    }
    if (request_changed_for_log &&
        selected_will_apply &&
        (led_owner_diag_interesting(owner) ||
         (selected_valid && led_owner_diag_interesting(selected.owner))))
    {
        LED_OWNER_DIAG_LOG("[LED_OWNER][REQ] owner=%s(%u) prio=%u pattern=%u selected=%s(%u) selected_prio=%u selected_pattern=%u current_before=%s(%u) frame=0",
                           led_owner_name(owner),
                           (uint32_t)owner,
                           (uint32_t)priority,
                           (uint32_t)pattern->id,
                           led_owner_name(selected_valid ? selected.owner : LED_OWNER_NONE),
                           selected_valid ? (uint32_t)selected.owner : 0U,
                           (uint32_t)selected_priority,
                           (uint32_t)selected_pattern,
                           led_owner_name(current_before),
                           (uint32_t)current_before);
    }
#if !ZY100_LED_DIAG_LOG_ENABLE
    (void)current_before;
    (void)selected_priority;
    (void)selected_pattern;
#endif
    return led_apply_selected_if_changed(&selected,
                                          selected_valid,
                                          os_sys_time_get());
}

void led_release(led_owner_t owner)
{
    led_owner_selected_t selected;
    bool selected_valid;
    bool was_active;
    led_owner_t current_before;
    uint8_t old_priority = LED_PRIORITY_NONE;
    led_pattern_id_t old_pattern = LED_PATTERN_NONE;
    uint8_t selected_priority = LED_PRIORITY_NONE;
    led_pattern_id_t selected_pattern = LED_PATTERN_NONE;
    uint32_t lock_state;

    if (!led_owner_valid(owner))
    {
        return;
    }

    lock_state = os_lock();
    current_before = s_led_current_owner;
    was_active = s_led_owner_entries[owner].active;
    old_priority = s_led_owner_entries[owner].priority;
    old_pattern = s_led_owner_entries[owner].pattern.id;
    s_led_owner_entries[owner].active = false;
    if (owner == s_led_show_frame_owner)
    {
        s_led_show_frame_owner = LED_OWNER_NONE;
    }
    selected_valid = led_select_locked(&selected);
    if (selected_valid)
    {
        selected_priority = s_led_owner_entries[selected.owner].priority;
        selected_pattern = selected.pattern.id;
    }
    os_unlock(lock_state);

    if (was_active &&
        (led_owner_diag_interesting(owner) ||
         (selected_valid && led_owner_diag_interesting(selected.owner))))
    {
        LED_OWNER_DIAG_LOG("[LED_OWNER][REL] owner=%s(%u) old_prio=%u old_pattern=%u selected=%s(%u) selected_prio=%u selected_pattern=%u current_before=%s(%u)",
                           led_owner_name(owner),
                           (uint32_t)owner,
                           (uint32_t)old_priority,
                           (uint32_t)old_pattern,
                           led_owner_name(selected_valid ? selected.owner : LED_OWNER_NONE),
                           selected_valid ? (uint32_t)selected.owner : 0U,
                           (uint32_t)selected_priority,
                           (uint32_t)selected_pattern,
                           led_owner_name(current_before),
                           (uint32_t)current_before);
    }
#if !ZY100_LED_DIAG_LOG_ENABLE
    (void)current_before;
    (void)old_priority;
    (void)old_pattern;
    (void)selected_priority;
    (void)selected_pattern;
#endif
    led_apply_selected_if_changed(&selected,
                                  selected_valid,
                                  os_sys_time_get());
}

void led_release_all(void)
{
    uint32_t lock_state;

#if !ZY100_LED_DIAG_LOG_ENABLE
    (void)led_owner_name;
#endif
    svc_led_task_cancel_sequence();
    lock_state = os_lock();
    led_clear_runtime_locked();
    os_unlock(lock_state);
}

void led_charge_chase_once_mark_consumed(void)
{
    uint32_t lock_state = os_lock();

    s_led_charge_chase_consumed = true;
    os_unlock(lock_state);
}

void led_charge_chase_once_reset(void)
{
    uint32_t lock_state = os_lock();

    s_led_charge_chase_consumed = false;
    os_unlock(lock_state);
}

bool led_force_notify_off(bool logo_off)
{
    led_pattern_t pattern;

    led_release_all();
    led_pattern_notify_off(&pattern, logo_off);
    return led_apply_pattern(&pattern, os_sys_time_get());
}

bool led_force_all_off(void)
{
    led_pattern_t pattern;

    led_release_all();
    led_pattern_all_off(&pattern);
    return led_apply_pattern(&pattern, os_sys_time_get());
}

led_owner_t led_current_owner(void)
{
    led_owner_t owner;
    uint32_t lock_state = os_lock();

    owner = s_led_current_owner;
    os_unlock(lock_state);
    return owner;
}

led_pattern_id_t led_current_pattern_id(void)
{
    led_pattern_id_t pattern_id;
    uint32_t lock_state = os_lock();

    pattern_id = s_led_current_valid ? s_led_current_pattern.id :
                 LED_PATTERN_NONE;
    os_unlock(lock_state);
    return pattern_id;
}

const char *led_owner_name_get(led_owner_t owner)
{
    return led_owner_name(owner);
}

bool led_owner_is_active(led_owner_t owner)
{
    bool active = false;
    uint32_t lock_state;

    if (!led_owner_valid(owner))
    {
        return false;
    }

    lock_state = os_lock();
    active = s_led_owner_entries[owner].active;
    os_unlock(lock_state);
    return active;
}

bool led_has_active_priority_above(uint8_t priority)
{
    uint8_t idx;
    bool found = false;
    uint32_t lock_state = os_lock();

    for (idx = (uint8_t)LED_OWNER_BUTTON_HOLD; idx < (uint8_t)LED_OWNER_COUNT; idx++)
    {
        if (s_led_owner_entries[idx].active &&
            (s_led_owner_entries[idx].priority > priority))
        {
            found = true;
            break;
        }
    }

    os_unlock(lock_state);
    return found;
}

void led_debug_dump_state(const char *tag, const char *reason)
{
#if ZY100_LED_DIAG_LOG_ENABLE
    const char *use_tag = (tag != NULL) ? tag : "unknown";
    const char *use_reason = (reason != NULL) ? reason : "unknown";
    bool current_valid;
    led_owner_t current_owner;
    uint8_t current_priority = LED_PRIORITY_NONE;
    led_pattern_id_t current_pattern;
    bool charge_idle_active;
    uint8_t charge_idle_priority;
    led_pattern_id_t charge_idle_pattern;
    bool charge_sleep_active;
    uint8_t charge_sleep_priority;
    led_pattern_id_t charge_sleep_pattern;
    bool status_active;
    uint8_t status_priority;
    led_pattern_id_t status_pattern;
    bool train_active;
    uint8_t train_priority;
    led_pattern_id_t train_pattern;
    bool ble_link_active;
    uint8_t ble_link_priority;
    led_pattern_id_t ble_link_pattern;
    bool boot_active;
    uint8_t boot_priority;
    led_pattern_id_t boot_pattern;
    uint32_t lock_state;

    lock_state = os_lock();
    current_valid = s_led_current_valid;
    current_owner = s_led_current_owner;
    current_pattern = s_led_current_pattern.id;
    if (led_owner_valid(current_owner) && s_led_owner_entries[current_owner].active)
    {
        current_priority = s_led_owner_entries[current_owner].priority;
    }

    charge_idle_active = s_led_owner_entries[LED_OWNER_CHARGE_IDLE].active;
    charge_idle_priority = s_led_owner_entries[LED_OWNER_CHARGE_IDLE].priority;
    charge_idle_pattern = s_led_owner_entries[LED_OWNER_CHARGE_IDLE].pattern.id;
    charge_sleep_active = s_led_owner_entries[LED_OWNER_CHARGE_SLEEP].active;
    charge_sleep_priority = s_led_owner_entries[LED_OWNER_CHARGE_SLEEP].priority;
    charge_sleep_pattern = s_led_owner_entries[LED_OWNER_CHARGE_SLEEP].pattern.id;
    status_active = s_led_owner_entries[LED_OWNER_STATUS_QUERY].active;
    status_priority = s_led_owner_entries[LED_OWNER_STATUS_QUERY].priority;
    status_pattern = s_led_owner_entries[LED_OWNER_STATUS_QUERY].pattern.id;
    train_active = s_led_owner_entries[LED_OWNER_TRAINING_TRANSITION].active;
    train_priority = s_led_owner_entries[LED_OWNER_TRAINING_TRANSITION].priority;
    train_pattern = s_led_owner_entries[LED_OWNER_TRAINING_TRANSITION].pattern.id;
    ble_link_active = s_led_owner_entries[LED_OWNER_BLE_LINK_EVENT].active;
    ble_link_priority = s_led_owner_entries[LED_OWNER_BLE_LINK_EVENT].priority;
    ble_link_pattern = s_led_owner_entries[LED_OWNER_BLE_LINK_EVENT].pattern.id;
    boot_active = s_led_owner_entries[LED_OWNER_BOOT_SEQUENCE].active;
    boot_priority = s_led_owner_entries[LED_OWNER_BOOT_SEQUENCE].priority;
    boot_pattern = s_led_owner_entries[LED_OWNER_BOOT_SEQUENCE].pattern.id;
    os_unlock(lock_state);

    LED_OWNER_DIAG_LOG("[LED_OWNER][STATE] tag=%s reason=%s current=%s(%u) valid=%u prio=%u pattern=%u charge_idle=%u/%u/%u charge_sleep=%u/%u/%u",
                       use_tag,
                       use_reason,
                       led_owner_name(current_owner),
                       (uint32_t)current_owner,
                       current_valid ? 1U : 0U,
                       (uint32_t)current_priority,
                       (uint32_t)current_pattern,
                       charge_idle_active ? 1U : 0U,
                       (uint32_t)charge_idle_priority,
                       (uint32_t)charge_idle_pattern,
                       charge_sleep_active ? 1U : 0U,
                       (uint32_t)charge_sleep_priority,
                       (uint32_t)charge_sleep_pattern);
    LED_OWNER_DIAG_LOG("[LED_OWNER][STATE2] tag=%s reason=%s status=%u/%u/%u train=%u/%u/%u ble_link=%u/%u/%u boot=%u/%u/%u",
                       use_tag,
                       use_reason,
                       status_active ? 1U : 0U,
                       (uint32_t)status_priority,
                       (uint32_t)status_pattern,
                       train_active ? 1U : 0U,
                       (uint32_t)train_priority,
                       (uint32_t)train_pattern,
                       ble_link_active ? 1U : 0U,
                       (uint32_t)ble_link_priority,
                       (uint32_t)ble_link_pattern,
                       boot_active ? 1U : 0U,
                       (uint32_t)boot_priority,
                       (uint32_t)boot_pattern);
#else
    (void)tag;
    (void)reason;
#endif
}

static void led_tick_blink(uint64_t now_ms)
{
    uint64_t elapsed_ms = now_ms - s_led_current_last_ms;

    if (s_led_current_on)
    {
        if (elapsed_ms < (uint64_t)s_led_current_pattern.on_ms)
        {
            return;
        }

        s_led_current_on = false;
        s_led_current_last_ms = now_ms;
        if (s_led_current_pattern.cycles != 0U)
        {
            s_led_current_cycles_done++;
        }
        (void)svc_led_pattern_notify_off();
        return;
    }

    if (elapsed_ms < (uint64_t)s_led_current_pattern.off_ms)
    {
        return;
    }

    if ((s_led_current_pattern.cycles != 0U) &&
        (s_led_current_cycles_done >= s_led_current_pattern.cycles))
    {
        led_release(s_led_current_owner);
        return;
    }

    s_led_current_on = true;
    s_led_current_last_ms = now_ms;
    (void)led_apply_notify_color(s_led_current_pattern.color);
}

#if !ZY100_BUILD_PRODUCTION
static void led_tick_blue_breath(uint64_t now_ms)
{
    uint64_t elapsed_ms = now_ms - s_led_current_last_ms;

    if (elapsed_ms < (uint64_t)SVC_LED_TASK_BLUE_BREATH_STEP_MS)
    {
        return;
    }

    s_led_current_last_ms = now_ms;
    s_led_breath_elapsed_ms += (uint32_t)SVC_LED_TASK_BLUE_BREATH_STEP_MS;
    if (s_led_breath_elapsed_ms >= (uint32_t)SVC_LED_TASK_BLUE_BREATH_PERIOD_MS)
    {
        s_led_breath_elapsed_ms %= (uint32_t)SVC_LED_TASK_BLUE_BREATH_PERIOD_MS;
    }

    (void)led_apply_breath_level(s_led_breath_elapsed_ms);
}
#endif

#if !ZY100_BUILD_PRODUCTION
static void led_tick_purple_breath(uint64_t now_ms)
{
    uint64_t elapsed_ms = now_ms - s_led_current_last_ms;

    if (elapsed_ms < (uint64_t)SVC_LED_TASK_BLUE_BREATH_STEP_MS)
    {
        return;
    }
    s_led_current_last_ms = now_ms;
    s_led_breath_elapsed_ms += (uint32_t)SVC_LED_TASK_BLUE_BREATH_STEP_MS;
    if (s_led_breath_elapsed_ms >= (uint32_t)SVC_LED_TASK_BLUE_BREATH_PERIOD_MS)
    {
        s_led_breath_elapsed_ms %= (uint32_t)SVC_LED_TASK_BLUE_BREATH_PERIOD_MS;
    }
    (void)led_apply_purple_breath_level(s_led_breath_elapsed_ms);
}
#endif

static void led_tick_capture_breath(uint64_t now_ms)
{
    uint64_t elapsed_ms = now_ms - s_led_current_last_ms;
    uint64_t cycle_elapsed_ms;

    if (s_led_capture_breath_gap)
    {
        if (elapsed_ms < (uint64_t)LED_OWNER_CAPTURE_BREATH_GAP_MS)
        {
            return;
        }

        s_led_capture_breath_gap = false;
        s_led_current_start_ms = now_ms;
        s_led_current_last_ms = now_ms;
        s_led_breath_elapsed_ms = 0U;
        s_led_current_on = true;
        (void)led_apply_capture_breath_level(0U);
        return;
    }

    if (elapsed_ms < (uint64_t)LED_OWNER_CAPTURE_BREATH_STEP_MS)
    {
        return;
    }

    cycle_elapsed_ms = now_ms - s_led_current_start_ms;
    if (cycle_elapsed_ms >= (uint64_t)LED_OWNER_CAPTURE_BREATH_CYCLE_MS)
    {
        s_led_capture_breath_gap = true;
        s_led_current_on = false;
        s_led_current_last_ms = now_ms;
        s_led_breath_elapsed_ms = 0U;
        svc_led_pattern_shutdown_for_sleep();
        return;
    }

#if ZY100_BUILD_PRODUCTION
    s_led_current_last_ms = now_ms -
        ((now_ms - s_led_current_start_ms) % LED_OWNER_CAPTURE_BREATH_STEP_MS);
#else
    s_led_current_last_ms = now_ms;
#endif
    s_led_breath_elapsed_ms = (uint32_t)cycle_elapsed_ms;
    (void)led_apply_capture_breath_level(s_led_breath_elapsed_ms);
}

static void led_tick_training(uint64_t now_ms)
{
    const led_training_pattern_config_t *config =
        &s_led_current_pattern.training;
    uint64_t elapsed_ms = now_ms - s_led_current_last_ms;
    uint32_t active_ms;

    if (!config->enabled ||
        (config->effect == LED_OWNER_TRAINING_EFFECT_SOLID))
    {
        return;
    }
    active_ms = led_training_active_ms(config);
    if (config->effect == LED_OWNER_TRAINING_EFFECT_MARQUEE)
    {
        uint8_t count =
            (config->target == LED_OWNER_TRAINING_TARGET_LOGO) ?
            ZY100_LED_LOGO_COUNT : ZY100_RGB_LED_COUNT;

        if (elapsed_ms < active_ms)
        {
            return;
        }
        s_led_current_last_ms = now_ms;
        s_led_training_step = (uint8_t)((s_led_training_step + 1U) % count);
        (void)led_training_show(led_training_max_level(config), true);
        return;
    }
    if (config->effect == LED_OWNER_TRAINING_EFFECT_BLINK)
    {
        if (s_led_current_on)
        {
            if (elapsed_ms < active_ms)
            {
                return;
            }
            s_led_current_on = false;
            s_led_current_last_ms = now_ms;
            svc_led_pattern_shutdown_for_sleep();
            return;
        }
        if (elapsed_ms < led_training_gap_ms(config))
        {
            return;
        }
        s_led_current_on = true;
        s_led_current_last_ms = now_ms;
        (void)led_training_show(led_training_max_level(config), false);
        return;
    }

    if (s_led_capture_breath_gap)
    {
        if (elapsed_ms < led_training_gap_ms(config))
        {
            return;
        }
        s_led_capture_breath_gap = false;
        s_led_current_start_ms = now_ms;
        s_led_current_last_ms = now_ms;
        s_led_current_on = true;
        (void)led_training_show(0U, false);
        return;
    }
    if ((now_ms - s_led_current_start_ms) >= active_ms)
    {
        s_led_capture_breath_gap = true;
        s_led_current_on = false;
        s_led_current_last_ms = now_ms;
        svc_led_pattern_shutdown_for_sleep();
        return;
    }
    if (elapsed_ms < LED_OWNER_CAPTURE_BREATH_STEP_MS)
    {
        return;
    }
#if ZY100_BUILD_PRODUCTION
    s_led_current_last_ms = now_ms -
        ((now_ms - s_led_current_start_ms) % LED_OWNER_CAPTURE_BREATH_STEP_MS);
#else
    s_led_current_last_ms = now_ms;
#endif
    (void)led_training_show(
        led_training_breath_level(
            (uint32_t)(now_ms - s_led_current_start_ms)),
        false);
}

static void led_tick_charge_sequence(uint64_t now_ms)
{
    uint64_t elapsed_ms = now_ms - s_led_current_last_ms;

    if (s_led_charge_sequence_phase == LED_CHARGE_SEQUENCE_PHASE_CHASE)
    {
        if (elapsed_ms < (uint64_t)SVC_LED_TASK_CHARGE_LOGO_STEP_MS)
        {
            return;
        }

        s_led_current_last_ms = now_ms;
        s_led_charge_chase_step++;
        if (s_led_charge_chase_step < ZY100_LED_LOGO_COUNT)
        {
            (void)svc_led_task_charge_sequence_chase_step(s_led_charge_chase_step);
            return;
        }

        s_led_charge_sequence_phase = LED_CHARGE_SEQUENCE_PHASE_FADE;
        s_led_charge_fade_elapsed_ms = 0U;
        s_led_breath_elapsed_ms = 0U;
        return;
    }

    if (s_led_charge_sequence_phase == LED_CHARGE_SEQUENCE_PHASE_FADE)
    {
        const uint32_t step_ms = led_charge_fade_step_ms();

        if (elapsed_ms < (uint64_t)step_ms)
        {
            return;
        }

        s_led_current_last_ms = now_ms;
        s_led_charge_fade_elapsed_ms += step_ms;
        if (s_led_charge_fade_elapsed_ms >= led_charge_fade_total_ms())
        {
            s_led_charge_sequence_phase = LED_CHARGE_SEQUENCE_PHASE_BREATH;
            s_led_charge_fade_elapsed_ms = 0U;
            s_led_breath_elapsed_ms = 0U;
            (void)svc_led_task_charge_sequence_breath_step(
                      led_blue_breath_level(s_led_breath_elapsed_ms));
            return;
        }

        (void)svc_led_task_charge_sequence_breath_step(
                  led_charge_fade_level(s_led_charge_fade_elapsed_ms));
        return;
    }

    if (elapsed_ms < (uint64_t)SVC_LED_TASK_BLUE_BREATH_STEP_MS)
    {
        return;
    }

    s_led_current_last_ms = now_ms;
    s_led_breath_elapsed_ms += (uint32_t)SVC_LED_TASK_BLUE_BREATH_STEP_MS;
    if (s_led_breath_elapsed_ms >= (uint32_t)SVC_LED_TASK_BLUE_BREATH_PERIOD_MS)
    {
        s_led_breath_elapsed_ms %= (uint32_t)SVC_LED_TASK_BLUE_BREATH_PERIOD_MS;
    }

    (void)svc_led_task_charge_sequence_breath_step(
              led_blue_breath_level(s_led_breath_elapsed_ms));
}

void led_tick(uint64_t now_ms)
{
    if (!s_led_current_valid)
    {
        return;
    }

    if (s_led_current_start_ms == 0ULL)
    {
        s_led_current_start_ms = now_ms;
    }
    if (s_led_current_last_ms == 0ULL)
    {
        s_led_current_last_ms = now_ms;
    }

    if ((s_led_current_pattern.hold_ms != 0U) &&
        ((now_ms - s_led_current_start_ms) >= (uint64_t)s_led_current_pattern.hold_ms))
    {
        led_release(s_led_current_owner);
        return;
    }

    switch (s_led_current_pattern.id)
    {
    case LED_PATTERN_NOTIFY_BLINK:
        led_tick_blink(now_ms);
        break;
    case LED_PATTERN_NOTIFY_BLUE_BREATH:
#if !ZY100_BUILD_PRODUCTION
        led_tick_blue_breath(now_ms);
#endif
        break;
    case LED_PATTERN_CAPTURE_NOTIFY_BLUE_BREATH:
        led_tick_capture_breath(now_ms);
        break;
    case LED_PATTERN_TRAINING_CONFIG:
        led_tick_training(now_ms);
        break;
    case LED_PATTERN_NOTIFY_PURPLE_BREATH:
#if !ZY100_BUILD_PRODUCTION
        led_tick_purple_breath(now_ms);
#endif
        break;
    case LED_PATTERN_CHARGE_SEQUENCE:
    case LED_PATTERN_CHARGE_BREATH:
    case LED_PATTERN_SLEEP_CHARGE_BREATH:
        led_tick_charge_sequence(now_ms);
        break;
    default:
        break;
    }
}
