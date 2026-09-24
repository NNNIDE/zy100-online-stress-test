#include "svc_led_task.h"

#include <stddef.h>
#include <string.h>

#include "os_msg.h"
#include "os_sched.h"
#include "os_task.h"
#include "trace.h"

#include "../app_flags.h"
#include "svc_led_pattern.h"
#include "zy100_online_reset_trace.h"

#define SVC_LED_TASK_QUEUE_DEPTH       8U
#define SVC_LED_TASK_STACK_SIZE        (512U * 4U)
#define SVC_LED_TASK_PRIORITY          2U

#define SVC_LED_EXPECTED_NONE          0U
#define SVC_LED_EXPECTED_GREEN         3U
#define SVC_LED_EXPECTED_BLUE          4U
#define SVC_LED_EXPECTED_RED           5U
#define SVC_LED_EXPECTED_LOGO_HINT     6U
#define SVC_LED_EXPECTED_LOGO_WHITE    7U
#define SVC_LED_EXPECTED_ORANGE        8U
#define SVC_LED_EXPECTED_LOGO_BLUE     9U
#define SVC_LED_LOG_EXPECTED_INVALID   0xFFU
#define SVC_LED_LOGO_WAKE_TEST_ENABLE  0U
#define SVC_LED_LOGO_WAKE_TEST_CYCLES  5U
#define SVC_LED_LOGO_WAKE_TEST_ON_MS   1000U
#define SVC_LED_LOGO_WAKE_TEST_OFF_MS  1000U
#define SVC_LED_TASK_BLUE_FAST_ON_MS   200U
#define SVC_LED_TASK_BLUE_FAST_OFF_MS  200U
#define SVC_LED_TASK_RED_SLOW_ON_MS    800U
#define SVC_LED_TASK_RED_SLOW_OFF_MS   800U
#define SVC_LED_TASK_ORANGE_SLOW_ON_MS 800U
#define SVC_LED_TASK_ORANGE_SLOW_OFF_MS 800U
#define SVC_LED_TASK_BLUE_BREATH_MAX   SVC_LED_TASK_BLUE_BREATH_MAX_LEVEL
#define SVC_LED_TASK_BLUE_BREATH_MIN   SVC_LED_TASK_BLUE_BREATH_MIN_LEVEL
#ifndef SVC_LED_TASK_LOGO_BOOT_BRIGHTNESS
#define SVC_LED_TASK_LOGO_BOOT_BRIGHTNESS SVC_LED_TASK_LOGO_HINT_BRIGHTNESS
#endif
#ifndef SVC_LED_TASK_ZY200_LOGO_BOOT_BLUE_BRIGHTNESS
#define SVC_LED_TASK_ZY200_LOGO_BOOT_BLUE_BRIGHTNESS 76U
#endif
#if (SVC_LED_TASK_ZY200_LOGO_BOOT_BLUE_BRIGHTNESS > 255U)
#error "SVC_LED_TASK_ZY200_LOGO_BOOT_BLUE_BRIGHTNESS must fit uint8_t"
#endif
#define SVC_LED_TASK_NOTIFY_START_INDEX ZY100_LED_LOGO_COUNT

#if ZY100_LED_LOG_ENABLE
#define SVC_LED_TASK_LOG(...)          DBG_DIRECT(__VA_ARGS__)
#else
#define SVC_LED_TASK_LOG(...)          do { if (0) { DBG_DIRECT(__VA_ARGS__); } } while (0)
#endif

typedef enum
{
    SVC_LED_TASK_CMD_NOTIFY = 0U,
    SVC_LED_TASK_CMD_LOGO_WAKE_TEST,
    SVC_LED_TASK_CMD_LOGO_WHITE_HINT,
    SVC_LED_TASK_CMD_LOGO_FADE_OUT,
    SVC_LED_TASK_CMD_NOTIFY_BLUE_FAST_BLINK,
    SVC_LED_TASK_CMD_NOTIFY_RED_SLOW_BLINK,
    SVC_LED_TASK_CMD_NOTIFY_ORANGE_SLOW_BLINK,
    SVC_LED_TASK_CMD_NOTIFY_BLUE_BREATH,
    SVC_LED_TASK_CMD_RESUME_LOGO_WHITE_HINT,
    SVC_LED_TASK_CMD_SYSTEM_BREATH,
} svc_led_task_cmd_t;

typedef struct
{
    svc_led_task_cmd_t cmd;
    svc_led_task_state_t state;
    uint32_t generation;
#if ZY100_BUILD_PRODUCTION
    uint32_t start_ms;
    uint8_t red, green, blue, led_mask;
#endif
} svc_led_task_msg_t;

static void *s_svc_led_task_handle = NULL;
static void *s_svc_led_queue_handle = NULL;
static volatile uint32_t s_svc_led_generation = 0U;
static volatile uint32_t s_logo_fade_requested = 0U;
static volatile uint32_t s_logo_fade_finished = UINT32_MAX;
static volatile uint32_t s_logo_fade_completed = UINT32_MAX;
static uint32_t s_svc_led_logo_test_generation = 0U;
static uint8_t s_svc_led_expected_out = SVC_LED_EXPECTED_NONE;
static uint8_t s_svc_led_resume_expected_out = SVC_LED_EXPECTED_NONE;
static uint8_t s_svc_led_log_expected_out = SVC_LED_LOG_EXPECTED_INVALID;
static uint8_t s_svc_led_last_apply_fail = SVC_LED_LOG_EXPECTED_INVALID;
static uint8_t s_svc_led_last_queue_full = SVC_LED_LOG_EXPECTED_INVALID;
static uint32_t s_svc_led_sleep_charge_breath_elapsed_ms = 0U;
static bool svc_led_task_apply_sleep_charge_breath_level(uint8_t blue_level);
static bool svc_led_task_apply_charge_sequence_chase_step(uint8_t step);
static bool svc_led_task_apply_charge_sequence_breath_level(uint8_t blue_level);
static bool svc_led_task_apply_charge_full_blue(void);
static bool svc_led_task_render_resume_expected(void);

static void svc_led_task_next_generation(void)
{
    if (!svc_led_pattern_lock()) return;
    s_svc_led_generation++;
    svc_led_pattern_unlock();
}

static uint32_t svc_led_task_blue_breath_next_elapsed(uint32_t elapsed_ms)
{
    if (SVC_LED_TASK_BLUE_BREATH_PERIOD_MS == 0U)
    {
        return 0U;
    }

    elapsed_ms += (uint32_t)SVC_LED_TASK_BLUE_BREATH_STEP_MS;
    if (elapsed_ms >= (uint32_t)SVC_LED_TASK_BLUE_BREATH_PERIOD_MS)
    {
        elapsed_ms %= (uint32_t)SVC_LED_TASK_BLUE_BREATH_PERIOD_MS;
    }

    return elapsed_ms;
}

static uint8_t svc_led_task_blue_breath_level(uint32_t elapsed_ms)
{
    const uint32_t period_ms = (uint32_t)SVC_LED_TASK_BLUE_BREATH_PERIOD_MS;
    const uint32_t half_ms = period_ms / 2U;
    uint32_t phase_ms;
    uint32_t ramp_ms;
    uint32_t delta;
    uint32_t level;

    if ((period_ms == 0U) || (half_ms == 0U) ||
        (SVC_LED_TASK_BLUE_BREATH_MAX <= SVC_LED_TASK_BLUE_BREATH_MIN))
    {
        return (uint8_t)SVC_LED_TASK_BLUE_BREATH_MAX;
    }

    phase_ms = elapsed_ms % period_ms;
    ramp_ms = (phase_ms <= half_ms) ?
              phase_ms :
              (period_ms - phase_ms);
    delta = (uint32_t)SVC_LED_TASK_BLUE_BREATH_MAX -
            (uint32_t)SVC_LED_TASK_BLUE_BREATH_MIN;
    level = (uint32_t)SVC_LED_TASK_BLUE_BREATH_MIN +
            ((delta * ramp_ms) / half_ms);

    if (level > (uint32_t)SVC_LED_TASK_BLUE_BREATH_MAX)
    {
        level = (uint32_t)SVC_LED_TASK_BLUE_BREATH_MAX;
    }

    return (uint8_t)level;
}

static uint32_t svc_led_task_logo_fade_total_ms(void)
{
    const uint32_t total_ms = (uint32_t)SVC_LED_TASK_LOGO_FADE_TOTAL_MS;

    return (total_ms == 0U) ? 1U : total_ms;
}

static uint32_t svc_led_task_logo_fade_step_ms(void)
{
    const uint32_t step_ms = (uint32_t)SVC_LED_TASK_LOGO_FADE_STEP_MS;

    return (step_ms == 0U) ? 1U : step_ms;
}

static uint8_t svc_led_task_logo_fade_level(uint32_t elapsed_ms)
{
    const uint32_t total_ms = svc_led_task_logo_fade_total_ms();
    uint32_t remaining_ms;
    uint32_t level;

    if (elapsed_ms >= total_ms)
    {
        return 0U;
    }

    remaining_ms = total_ms - elapsed_ms;
    level = ((uint32_t)SVC_LED_TASK_LOGO_HINT_BRIGHTNESS * remaining_ms) /
            total_ms;
    if (level > (uint32_t)SVC_LED_TASK_LOGO_HINT_BRIGHTNESS)
    {
        level = (uint32_t)SVC_LED_TASK_LOGO_HINT_BRIGHTNESS;
    }

    return (uint8_t)level;
}

static uint8_t svc_led_task_logo_boot_blue_brightness(void)
{
#if (ZY100_LED_BOARD_PROFILE == ZY100_LED_BOARD_PROFILE_ZY200)
    return (uint8_t)SVC_LED_TASK_ZY200_LOGO_BOOT_BLUE_BRIGHTNESS;
#else
    return (uint8_t)SVC_LED_TASK_LOGO_BOOT_BRIGHTNESS;
#endif
}

static const char *svc_led_task_state_name(svc_led_task_state_t state)
{
    switch (state)
    {
    case SVC_LED_TASK_STATE_GREEN:
        return "green";
    case SVC_LED_TASK_STATE_BLUE:
        return "blue";
    case SVC_LED_TASK_STATE_RED:
        return "red";
    case SVC_LED_TASK_STATE_SLEEP_OFF:
        return "sleep_off";
    case SVC_LED_TASK_STATE_LOGO_WHITE_HINT:
        return "logo_white_hint";
    case SVC_LED_TASK_STATE_LOGO_WHITE:
        return "logo_white";
    case SVC_LED_TASK_STATE_LOGO_BLUE:
        return "logo_blue";
    case SVC_LED_TASK_STATE_LOGO_CHASE:
        return "logo_chase";
    case SVC_LED_TASK_STATE_BLUE_FAST_BLINK:
        return "blue_fast_blink";
    case SVC_LED_TASK_STATE_RED_SLOW_BLINK:
        return "red_slow_blink";
    case SVC_LED_TASK_STATE_ORANGE_SLOW_BLINK:
        return "orange_slow_blink";
    case SVC_LED_TASK_STATE_BLUE_BREATH:
        return "blue_breath";
    case SVC_LED_TASK_STATE_CHARGE_SEQUENCE:
        return "charge_sequence";
    case SVC_LED_TASK_STATE_CHARGE_FULL_BLUE:
        return "charge_full_blue";
    case SVC_LED_TASK_STATE_SLEEP_CHARGE:
        return "sleep_charge";
    case SVC_LED_TASK_STATE_SLEEP_CHARGE_FULL:
        return "sleep_charge_full";
    case SVC_LED_TASK_STATE_SYSTEM_BREATH:
        return "system_breath";
    case SVC_LED_TASK_STATE_OFF:
    default:
        return "off";
    }
}

static uint8_t svc_led_task_expected_from_state(svc_led_task_state_t state)
{
    switch (state)
    {
    case SVC_LED_TASK_STATE_SYSTEM_BREATH:
        return 10U;
    case SVC_LED_TASK_STATE_GREEN:
        return SVC_LED_EXPECTED_GREEN;
    case SVC_LED_TASK_STATE_BLUE:
        return SVC_LED_EXPECTED_BLUE;
    case SVC_LED_TASK_STATE_RED:
        return SVC_LED_EXPECTED_RED;
    case SVC_LED_TASK_STATE_LOGO_WHITE_HINT:
        return SVC_LED_EXPECTED_LOGO_HINT;
    case SVC_LED_TASK_STATE_LOGO_WHITE:
        return SVC_LED_EXPECTED_LOGO_WHITE;
    case SVC_LED_TASK_STATE_LOGO_CHASE:
    case SVC_LED_TASK_STATE_LOGO_BLUE:
        return SVC_LED_EXPECTED_LOGO_BLUE;
    case SVC_LED_TASK_STATE_BLUE_FAST_BLINK:
    case SVC_LED_TASK_STATE_BLUE_BREATH:
    case SVC_LED_TASK_STATE_CHARGE_SEQUENCE:
    case SVC_LED_TASK_STATE_CHARGE_FULL_BLUE:
    case SVC_LED_TASK_STATE_SLEEP_CHARGE:
    case SVC_LED_TASK_STATE_SLEEP_CHARGE_FULL:
        return SVC_LED_EXPECTED_BLUE;
    case SVC_LED_TASK_STATE_RED_SLOW_BLINK:
        return SVC_LED_EXPECTED_RED;
    case SVC_LED_TASK_STATE_ORANGE_SLOW_BLINK:
        return SVC_LED_EXPECTED_ORANGE;
    case SVC_LED_TASK_STATE_SLEEP_OFF:
    case SVC_LED_TASK_STATE_OFF:
    default:
        return SVC_LED_EXPECTED_NONE;
    }
}

static void svc_led_task_set_expected(svc_led_task_state_t state)
{
    s_svc_led_expected_out = svc_led_task_expected_from_state(state);
}

static void svc_led_task_log_if_changed(svc_led_task_state_t state)
{
    uint8_t expected_out = svc_led_task_expected_from_state(state);

    if (s_svc_led_log_expected_out == expected_out)
    {
        return;
    }

#if (ZY100_LED_HW_ENABLE == 0U)
    SVC_LED_TASK_LOG("[LED] ideal notify=%s hw=disabled", svc_led_task_state_name(state));
#else
    SVC_LED_TASK_LOG("[LED] notify=%s hw=1", svc_led_task_state_name(state));
#endif
    s_svc_led_log_expected_out = expected_out;
}

static void svc_led_task_log_apply_fail(svc_led_task_state_t state)
{
    uint8_t expected_out = svc_led_task_expected_from_state(state);

    if (s_svc_led_last_apply_fail == expected_out)
    {
        return;
    }

    SVC_LED_TASK_LOG("[LED][ERR] stage=apply state=%s hw=%u",
                     svc_led_task_state_name(state),
                     (uint32_t)ZY100_LED_HW_ENABLE);
    s_svc_led_last_apply_fail = expected_out;
}

static void svc_led_task_log_queue_full(svc_led_task_state_t state)
{
    uint8_t expected_out = svc_led_task_expected_from_state(state);

    if (s_svc_led_last_queue_full == expected_out)
    {
        return;
    }

    SVC_LED_TASK_LOG("[LED][WARN] queue_full state=%s", svc_led_task_state_name(state));
    s_svc_led_last_queue_full = expected_out;
}

static bool svc_led_task_apply_state(svc_led_task_state_t state)
{
#if (ZY100_LED_HW_ENABLE == 0U)
    svc_led_task_log_if_changed(state);
    return true;
#else
    bool applied;

    switch (state)
    {
    case SVC_LED_TASK_STATE_GREEN:
        applied = svc_led_pattern_notify_green_on();
        break;
    case SVC_LED_TASK_STATE_BLUE:
        applied = svc_led_pattern_notify_blue_on();
        break;
    case SVC_LED_TASK_STATE_RED:
        applied = svc_led_pattern_notify_red_on();
        break;
    case SVC_LED_TASK_STATE_SLEEP_OFF:
    case SVC_LED_TASK_STATE_OFF:
    default:
        applied = svc_led_pattern_notify_off();
        break;
    }

    if (applied)
    {
        s_svc_led_last_apply_fail = SVC_LED_LOG_EXPECTED_INVALID;
        svc_led_task_log_if_changed(state);
    }
    else
    {
        svc_led_task_log_apply_fail(state);
    }

    return applied;
#endif
}

static bool svc_led_task_generation_lock(uint32_t generation)
{
    if (!svc_led_pattern_lock()) return false;
    if (generation != s_svc_led_generation)
    {
        svc_led_pattern_unlock();
        return false;
    }
    return true;
}

static void svc_led_task_apply_notify_msg(const svc_led_task_msg_t *msg)
{
    if ((msg != NULL) && svc_led_task_generation_lock(msg->generation))
    {
        (void)svc_led_task_apply_state(msg->state);
        svc_led_pattern_unlock();
    }
}

static bool svc_led_task_apply_logo_white_hint(void)
{
    if (!svc_led_pattern_notify_off())
    {
        return false;
    }

    if (!svc_led_pattern_logo_off())
    {
        return false;
    }

    return svc_led_pattern_logo_rgb(SVC_LED_TASK_LOGO_HINT_BRIGHTNESS,
                                    SVC_LED_TASK_LOGO_HINT_BRIGHTNESS,
                                    SVC_LED_TASK_LOGO_HINT_BRIGHTNESS);
}

static bool svc_led_task_apply_logo_white(bool on)
{
    if (on)
    {
        return svc_led_pattern_logo_rgb(SVC_LED_TASK_LOGO_BOOT_BRIGHTNESS,
                                        SVC_LED_TASK_LOGO_BOOT_BRIGHTNESS,
                                        SVC_LED_TASK_LOGO_BOOT_BRIGHTNESS);
    }

    return svc_led_pattern_logo_off();
}

static bool svc_led_task_apply_logo_blue(bool on)
{
    if (on)
    {
        return svc_led_pattern_logo_rgb(0U,
                                        0U,
                                        svc_led_task_logo_boot_blue_brightness());
    }

    return svc_led_pattern_logo_off();
}

static bool svc_led_task_render_resume_expected(void)
{
    switch (s_svc_led_resume_expected_out)
    {
    case SVC_LED_EXPECTED_GREEN:
        return svc_led_pattern_notify_green_on();
    case SVC_LED_EXPECTED_BLUE:
        return svc_led_pattern_notify_blue_on();
    case SVC_LED_EXPECTED_RED:
        return svc_led_pattern_notify_red_on();
    case SVC_LED_EXPECTED_ORANGE:
        return svc_led_pattern_notify_orange_on();
    case SVC_LED_EXPECTED_LOGO_HINT:
        return svc_led_task_apply_logo_white_hint();
    case SVC_LED_EXPECTED_LOGO_WHITE:
        return svc_led_task_apply_logo_white(true);
    case SVC_LED_EXPECTED_LOGO_BLUE:
        return svc_led_task_apply_logo_blue(true);
    case SVC_LED_EXPECTED_NONE:
    default:
        return svc_led_pattern_all_off();
    }
}

static bool svc_led_task_apply_wake_logo_chase_step(uint8_t step)
{
    static const uint8_t logo_order[ZY100_LED_LOGO_COUNT] = ZY100_LED_LOGO_ORDER_INIT;
    zy100_rgb_color_t frame[ZY100_RGB_LED_COUNT];
    const uint8_t blue = svc_led_task_logo_boot_blue_brightness();
    uint8_t idx;

    if (step >= ZY100_LED_LOGO_COUNT)
    {
        return false;
    }

    for (idx = 0U; idx < ZY100_RGB_LED_COUNT; idx++)
    {
        frame[idx].red = 0U;
        frame[idx].green = 0U;
        frame[idx].blue = 0U;
    }

    for (idx = 0U; idx <= step; idx++)
    {
        uint8_t logo_index = logo_order[idx];

        if (logo_index >= ZY100_LED_LOGO_COUNT)
        {
            return false;
        }
        frame[logo_index].red = 0U;
        frame[logo_index].green = 0U;
        frame[logo_index].blue = blue;
    }

    return svc_led_pattern_show_frame(frame, ZY100_RGB_LED_COUNT);
}

static void svc_led_task_frame_clear(zy100_rgb_color_t *frame)
{
    uint8_t idx;

    if (frame == NULL)
    {
        return;
    }

    for (idx = 0U; idx < ZY100_RGB_LED_COUNT; idx++)
    {
        frame[idx].red = 0U;
        frame[idx].green = 0U;
        frame[idx].blue = 0U;
    }
}

static void svc_led_task_frame_notify_red(zy100_rgb_color_t *frame)
{
    uint8_t idx;

    if (frame == NULL)
    {
        return;
    }

    for (idx = SVC_LED_TASK_NOTIFY_START_INDEX; idx < ZY100_RGB_LED_COUNT; idx++)
    {
        frame[idx].red = (uint8_t)ZY100_LED_NOTIFY_BRIGHTNESS;
        frame[idx].green = 0U;
        frame[idx].blue = 0U;
    }
}

static bool svc_led_task_apply_charge_sequence_chase_step(uint8_t step)
{
    static const uint8_t logo_order[ZY100_LED_LOGO_COUNT] = ZY100_LED_LOGO_ORDER_INIT;
    zy100_rgb_color_t frame[ZY100_RGB_LED_COUNT];
    const uint8_t blue = svc_led_task_logo_boot_blue_brightness();
    uint8_t idx;

    if (step >= ZY100_LED_LOGO_COUNT)
    {
        return false;
    }

    svc_led_task_frame_clear(frame);
    svc_led_task_frame_notify_red(frame);

    for (idx = 0U; idx <= step; idx++)
    {
        uint8_t logo_index = logo_order[idx];

        if (logo_index >= ZY100_LED_LOGO_COUNT)
        {
            return false;
        }
        frame[logo_index].red = 0U;
        frame[logo_index].green = 0U;
        frame[logo_index].blue = blue;
    }

    return svc_led_pattern_show_frame(frame, ZY100_RGB_LED_COUNT);
}

static bool svc_led_task_apply_charge_sequence_breath_level(uint8_t blue_level)
{
    zy100_rgb_color_t frame[ZY100_RGB_LED_COUNT];
    uint8_t idx;

    svc_led_task_frame_clear(frame);
    svc_led_task_frame_notify_red(frame);

    for (idx = 0U; idx < ZY100_LED_LOGO_COUNT; idx++)
    {
        frame[idx].red = 0U;
        frame[idx].green = 0U;
        frame[idx].blue = blue_level;
    }

    return svc_led_pattern_show_frame(frame, ZY100_RGB_LED_COUNT);
}

static bool svc_led_task_apply_charge_full_blue(void)
{
    zy100_rgb_color_t frame[ZY100_RGB_LED_COUNT];
    uint8_t idx;

    for (idx = 0U; idx < ZY100_RGB_LED_COUNT; idx++)
    {
        frame[idx].red = 0U;
        frame[idx].green = 0U;
        frame[idx].blue = (uint8_t)ZY100_LED_NOTIFY_BRIGHTNESS;
    }

    return svc_led_pattern_show_frame(frame, ZY100_RGB_LED_COUNT);
}

static bool svc_led_task_run_logo_fade_out(uint32_t generation)
{
    uint32_t elapsed_ms = 0U;
    const uint32_t total_ms = svc_led_task_logo_fade_total_ms();
    const uint32_t step_ms = svc_led_task_logo_fade_step_ms();

    SVC_LED_TASK_LOG("[LED] logo_fade step_ms=%lu total_ms=%lu max=%u",
                     (unsigned long)step_ms,
                     (unsigned long)total_ms,
                     (uint32_t)SVC_LED_TASK_LOGO_HINT_BRIGHTNESS);

    while (elapsed_ms < total_ms)
    {
        uint8_t level = svc_led_task_logo_fade_level(elapsed_ms);

        if (generation != s_svc_led_generation)
        {
            return false;
        }

        bool applied;
        if (!svc_led_task_generation_lock(generation)) return false;
        applied = svc_led_pattern_logo_rgb(level, level, level);
        svc_led_pattern_unlock();
        if (!applied)
        {
            SVC_LED_TASK_LOG("[LED][ERR] stage=logo_fade level=%u", level);
            return false;
        }

        os_delay(step_ms);
        if (generation != s_svc_led_generation)
        {
            return false;
        }

        if ((total_ms - elapsed_ms) <= step_ms)
        {
            elapsed_ms = total_ms;
        }
        else
        {
            elapsed_ms += step_ms;
        }
    }

    if (generation != s_svc_led_generation)
    {
        return false;
    }

    if (!svc_led_task_generation_lock(generation)) return false;
    {
        bool applied = svc_led_pattern_logo_off();
        svc_led_pattern_unlock();
        return applied;
    }
}

typedef bool (*svc_led_task_notify_on_func_t)(void);

static void svc_led_task_run_notify_blink(uint32_t generation,
                                          svc_led_task_notify_on_func_t notify_on,
                                          uint32_t on_ms,
                                          uint32_t off_ms,
                                          const char *name)
{
    uint32_t deadline = os_sys_time_get();
    bool on = true;
    (void)name;
    if (notify_on == NULL) return;
    while (generation == s_svc_led_generation)
    {
        bool ok;
        uint32_t now;
        if (!svc_led_pattern_lock()) return;
        if (generation != s_svc_led_generation)
        {
            svc_led_pattern_unlock();
            return;
        }
        if (on) (void)svc_led_pattern_logo_off();
        ok = on ? notify_on() : svc_led_pattern_notify_off();
        svc_led_pattern_unlock();
        if (!ok) return;
        deadline += on ? on_ms : off_ms;
        now = os_sys_time_get();
        if ((int32_t)(deadline - now) > 0)
        {
            os_delay(deadline - now);
        }
        else
        {
            /* A late scheduler must not burst stale phases. */
            deadline = now;
        }
        on = !on;
    }
}

static void svc_led_task_run_notify_blue_breath(uint32_t generation)
{
    uint32_t elapsed_ms = 0U;
    uint8_t level;

    SVC_LED_TASK_LOG("[LED] notify_breath=blue step_ms=%lu period_ms=%lu min=%u max=%u",
                     (unsigned long)SVC_LED_TASK_BLUE_BREATH_STEP_MS,
                     (unsigned long)SVC_LED_TASK_BLUE_BREATH_PERIOD_MS,
                     (uint32_t)SVC_LED_TASK_BLUE_BREATH_MIN,
                     (uint32_t)SVC_LED_TASK_BLUE_BREATH_MAX);

    (void)svc_led_pattern_logo_off();
    while (generation == s_svc_led_generation)
    {
        level = svc_led_task_blue_breath_level(elapsed_ms);
        bool applied;
        if (!svc_led_task_generation_lock(generation)) return;
        applied = svc_led_pattern_notify_rgb(0U, 0U, level);
        svc_led_pattern_unlock();
        if (!applied)
        {
            SVC_LED_TASK_LOG("[LED][ERR] notify_breath_blue level=%u",
                             (uint32_t)level);
            return;
        }

        os_delay(SVC_LED_TASK_BLUE_BREATH_STEP_MS);
        if (generation != s_svc_led_generation)
        {
            return;
        }

        elapsed_ms = svc_led_task_blue_breath_next_elapsed(elapsed_ms);
    }
}

static bool svc_led_task_apply_notify_blue_breath_first_frame(void)
{
#if (ZY100_LED_HW_ENABLE == 0U)
    svc_led_task_log_if_changed(SVC_LED_TASK_STATE_BLUE_BREATH);
    return true;
#else
    if (!svc_led_pattern_resume_after_wake())
    {
        DBG_DIRECT("[LED][WARN] blue_breath_first_frame_fail stage=resume");
        return false;
    }

    if (!svc_led_pattern_logo_off())
    {
        DBG_DIRECT("[LED][WARN] blue_breath_first_frame_fail stage=logo_off");
        return false;
    }

    if (!svc_led_pattern_notify_rgb(0U, 0U,
                                    svc_led_task_blue_breath_level(0U)))
    {
        DBG_DIRECT("[LED][WARN] blue_breath_first_frame_fail stage=notify");
        return false;
    }

    svc_led_task_log_if_changed(SVC_LED_TASK_STATE_BLUE_BREATH);
    return true;
#endif
}

static bool svc_led_task_apply_sleep_charge_breath_level(uint8_t blue_level)
{
#if (ZY100_LED_HW_ENABLE == 0U)
    svc_led_task_log_if_changed(SVC_LED_TASK_STATE_SLEEP_CHARGE);
    return true;
#else
    if (!svc_led_task_apply_charge_sequence_breath_level(blue_level))
    {
        DBG_DIRECT("[LED][WARN] sleep_charge_frame_fail stage=charge_frame");
        return false;
    }

    svc_led_task_log_if_changed(SVC_LED_TASK_STATE_SLEEP_CHARGE);
    return true;
#endif
}

static void svc_led_task_run_logo_wake_test(uint32_t generation)
{
#if (SVC_LED_LOGO_WAKE_TEST_ENABLE != 0U)
    uint8_t cycle;
    svc_led_task_msg_t msg;

    SVC_LED_TASK_LOG("[LED][TEST] logo_green_blink start cycles=%u on_ms=%u off_ms=%u",
                     SVC_LED_LOGO_WAKE_TEST_CYCLES,
                     SVC_LED_LOGO_WAKE_TEST_ON_MS,
                     SVC_LED_LOGO_WAKE_TEST_OFF_MS);

    for (cycle = 0U; cycle < SVC_LED_LOGO_WAKE_TEST_CYCLES; cycle++)
    {
        if (generation != s_svc_led_logo_test_generation)
        {
            return;
        }

        if (!svc_led_pattern_logo_green_on())
        {
            SVC_LED_TASK_LOG("[LED][ERR] stage=logo_test phase=green cycle=%u", (uint32_t)(cycle + 1U));
            return;
        }

        os_delay(SVC_LED_LOGO_WAKE_TEST_ON_MS);
        while (os_msg_recv(s_svc_led_queue_handle, &msg, 0U))
        {
            if (msg.cmd == SVC_LED_TASK_CMD_NOTIFY)
            {
                svc_led_task_apply_notify_msg(&msg);
            }
        }
        if (generation != s_svc_led_logo_test_generation)
        {
            return;
        }

        if (!svc_led_pattern_logo_off())
        {
            SVC_LED_TASK_LOG("[LED][ERR] stage=logo_test phase=off cycle=%u", (uint32_t)(cycle + 1U));
            return;
        }

        os_delay(SVC_LED_LOGO_WAKE_TEST_OFF_MS);
        while (os_msg_recv(s_svc_led_queue_handle, &msg, 0U))
        {
            if (msg.cmd == SVC_LED_TASK_CMD_NOTIFY)
            {
                svc_led_task_apply_notify_msg(&msg);
            }
        }
    }

    (void)svc_led_pattern_logo_off();
    SVC_LED_TASK_LOG("[LED][TEST] logo_green_blink done");
#else
    (void)generation;
#endif
}

#if ZY100_BUILD_PRODUCTION
static bool svc_led_task_system_breath_frame(const svc_led_task_msg_t *msg,
                                             uint32_t now_ms)
{
    zy100_rgb_color_t frame[ZY100_RGB_LED_COUNT] = {{0}};
    uint8_t level;
    uint8_t i;
    bool ok = false;

    if (!svc_led_pattern_lock()) return false;
    /* Check after acquiring the render lock: cancellation may have completed
     * while this worker was waiting behind the successor's first frame. */
    if (msg->generation == s_svc_led_generation)
    {
        level = svc_led_task_blue_breath_level(now_ms - msg->start_ms);
        for (i = 0U; i < ZY100_RGB_LED_COUNT; ++i)
        {
            if ((msg->led_mask & (1U << i)) != 0U)
            {
                frame[i].red = (uint8_t)(((uint32_t)msg->red * level + 127U) / 255U);
                frame[i].green = (uint8_t)(((uint32_t)msg->green * level + 127U) / 255U);
                frame[i].blue = (uint8_t)(((uint32_t)msg->blue * level + 127U) / 255U);
            }
        }
        ok = svc_led_pattern_show_frame(frame, ZY100_RGB_LED_COUNT);
    }
    svc_led_pattern_unlock();
    return ok;
}

static void svc_led_task_run_system_breath(const svc_led_task_msg_t *msg)
{
    uint32_t now_ms;
    uint32_t delay_ms;
    bool failed = false;
    while (msg->generation == s_svc_led_generation)
    {
        now_ms = (uint32_t)os_sys_time_get();
        /* Absolute 20 ms boundaries; a late wake renders only the current
         * phase, never a burst of missed frames or an extra relative delay. */
        delay_ms = SVC_LED_TASK_BLUE_BREATH_STEP_MS -
                   ((now_ms - msg->start_ms) % SVC_LED_TASK_BLUE_BREATH_STEP_MS);
        os_delay(delay_ms);
        now_ms = (uint32_t)os_sys_time_get();
        if (!svc_led_task_system_breath_frame(msg, now_ms))
        {
            if (msg->generation != s_svc_led_generation) return;
            if (!failed) SVC_LED_TASK_LOG("[LED][WARN] system_breath_frame_retry");
            failed = true;
        }
        else failed = false;
    }
}
#endif

bool svc_led_task_system_breath(uint8_t red, uint8_t green, uint8_t blue,
                                uint8_t led_mask)
{
#if ZY100_BUILD_PRODUCTION
    svc_led_task_msg_t msg;
    bool ok;
    memset(&msg, 0, sizeof(msg));
    if (led_mask == 0U || !svc_led_pattern_lock()) return false;
    svc_led_task_next_generation();
    ok = svc_led_task_init();
    if (ok)
    {
        msg.cmd = SVC_LED_TASK_CMD_SYSTEM_BREATH;
        msg.state = SVC_LED_TASK_STATE_SYSTEM_BREATH;
        msg.generation = s_svc_led_generation;
        msg.start_ms = (uint32_t)os_sys_time_get();
        msg.red = red; msg.green = green; msg.blue = blue;
        msg.led_mask = led_mask;
        ok = svc_led_task_system_breath_frame(&msg, msg.start_ms);
        if (ok) ok = os_msg_send(s_svc_led_queue_handle, &msg, 0U);
    }
    if (ok) svc_led_task_set_expected(SVC_LED_TASK_STATE_SYSTEM_BREATH);
    else SVC_LED_TASK_LOG("[LED][ERR] system_breath_start");
    svc_led_pattern_unlock();
    return ok;
#else
    (void)red; (void)green; (void)blue; (void)led_mask;
    return false;
#endif
}

static void svc_led_task_handle_msg(const svc_led_task_msg_t *msg)
{
    if (msg == NULL)
    {
        return;
    }

    if (msg->cmd == SVC_LED_TASK_CMD_LOGO_WAKE_TEST)
    {
        if (msg->generation == s_svc_led_logo_test_generation)
        {
            svc_led_task_run_logo_wake_test(msg->generation);
        }
        return;
    }

    if (msg->generation != s_svc_led_generation)
    {
        return;
    }

#if ZY100_BUILD_PRODUCTION
    if (msg->cmd == SVC_LED_TASK_CMD_SYSTEM_BREATH)
    {
        svc_led_task_run_system_breath(msg);
        return;
    }
#endif
    if (msg->cmd == SVC_LED_TASK_CMD_LOGO_WHITE_HINT)
    {
        if (!svc_led_task_generation_lock(msg->generation)) return;
        if (!svc_led_task_apply_logo_white_hint())
        {
            SVC_LED_TASK_LOG("[LED][ERR] stage=logo_white_hint");
        }
        svc_led_pattern_unlock();
        return;
    }

    if (msg->cmd == SVC_LED_TASK_CMD_LOGO_FADE_OUT)
    {
        bool cleared;
        /* Stop the previous notify hint before fading the LOGO. */
        if (!svc_led_task_generation_lock(msg->generation)) return;
        cleared = svc_led_pattern_notify_off();
        svc_led_pattern_unlock();
        if (cleared &&
            svc_led_task_run_logo_fade_out(msg->generation))
        {
            s_logo_fade_completed = msg->generation;
        }
        s_logo_fade_finished = msg->generation;
        return;
    }

    if (msg->cmd == SVC_LED_TASK_CMD_NOTIFY_BLUE_FAST_BLINK)
    {
        svc_led_task_run_notify_blink(msg->generation,
                                      svc_led_pattern_notify_blue_on,
                                      SVC_LED_TASK_BLUE_FAST_ON_MS,
                                      SVC_LED_TASK_BLUE_FAST_OFF_MS,
                                      "blue_fast");
        return;
    }

    if (msg->cmd == SVC_LED_TASK_CMD_NOTIFY_RED_SLOW_BLINK)
    {
        svc_led_task_run_notify_blink(msg->generation,
                                      svc_led_pattern_notify_red_on,
                                      SVC_LED_TASK_RED_SLOW_ON_MS,
                                      SVC_LED_TASK_RED_SLOW_OFF_MS,
                                      "red_slow");
        return;
    }

    if (msg->cmd == SVC_LED_TASK_CMD_RESUME_LOGO_WHITE_HINT)
    {
        bool resumed;
        if (!svc_led_task_generation_lock(msg->generation)) return;
        resumed = svc_led_pattern_resume_after_wake();

        if (resumed)
        {
            zy100_power_reset_trace_set_stage(ZY100_POWER_TRACE_LED_HINT);
            resumed = svc_led_task_apply_logo_white_hint();
        }
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[APP_STANDBY_LED] async_resume done=%u generation=%lu",
                   resumed ? 1U : 0U,
                   (unsigned long)msg->generation);
        svc_led_pattern_unlock();
        return;
    }

    if (msg->cmd == SVC_LED_TASK_CMD_NOTIFY_ORANGE_SLOW_BLINK)
    {
        svc_led_task_run_notify_blink(msg->generation,
                                      svc_led_pattern_notify_orange_on,
                                      SVC_LED_TASK_ORANGE_SLOW_ON_MS,
                                      SVC_LED_TASK_ORANGE_SLOW_OFF_MS,
                                      "orange_slow");
        return;
    }

    if (msg->cmd == SVC_LED_TASK_CMD_NOTIFY_BLUE_BREATH)
    {
        svc_led_task_run_notify_blue_breath(msg->generation);
        return;
    }

    svc_led_task_apply_notify_msg(msg);
}

static void svc_led_task_main(void *p_param)
{
    svc_led_task_msg_t msg;

    (void)p_param;

    while (true)
    {
        if (os_msg_recv(s_svc_led_queue_handle, &msg, 0xFFFFFFFFU))
        {
            svc_led_task_handle_msg(&msg);
        }
    }
}

bool svc_led_task_init(void)
{
    if (s_svc_led_queue_handle == NULL)
    {
        if (!os_msg_queue_create(&s_svc_led_queue_handle,
                                 SVC_LED_TASK_QUEUE_DEPTH,
                                 sizeof(svc_led_task_msg_t)))
        {
            SVC_LED_TASK_LOG("[LED][ERR] task queue create failed");
            return false;
        }
    }

    if (s_svc_led_task_handle == NULL)
    {
        if (!os_task_create(&s_svc_led_task_handle,
                            "led_svc",
                            svc_led_task_main,
                            0,
                            SVC_LED_TASK_STACK_SIZE,
                            SVC_LED_TASK_PRIORITY))
        {
            SVC_LED_TASK_LOG("[LED][ERR] task create failed");
            return false;
        }
        SVC_LED_TASK_LOG("[LED] task started prio=%u hw=%u",
                         (uint32_t)SVC_LED_TASK_PRIORITY,
                         (uint32_t)ZY100_LED_HW_ENABLE);
    }

    return true;
}

bool svc_led_task_resume_after_wake(void)
{
    bool resumed;

    if (!svc_led_task_init())
    {
        return false;
    }

    svc_led_task_next_generation();
    s_svc_led_logo_test_generation++;
#if (ZY100_LED_HW_ENABLE == 0U)
    resumed = true;
#else
    resumed = svc_led_pattern_resume_after_wake();
#endif
    if (resumed)
    {
        zy100_power_reset_trace_set_led_stage(
            ZY100_POWER_LED_STAGE_RENDER_ACTIVE_STATE);
        resumed = svc_led_task_render_resume_expected();
        if (resumed)
        {
            s_svc_led_expected_out = s_svc_led_resume_expected_out;
            s_svc_led_last_apply_fail = SVC_LED_LOG_EXPECTED_INVALID;
            ZY100_LOG_ROUTINE(ZY100_LOG_EVENT, "[EVT][LED] resume ok expected=%u",
                            (uint32_t)s_svc_led_resume_expected_out);
        }
        else
        {
            svc_led_pattern_shutdown_for_sleep();
            ZY100_LOG_WARN("[WRN][LED] resume render_failed expected=%u safe_off=1",
                           (uint32_t)s_svc_led_resume_expected_out);
        }
#if (SVC_LED_LOGO_WAKE_TEST_ENABLE != 0U)
        svc_led_task_msg_t msg;

        msg.cmd = SVC_LED_TASK_CMD_LOGO_WAKE_TEST;
        msg.state = SVC_LED_TASK_STATE_OFF;
        msg.generation = s_svc_led_logo_test_generation;
        if (!os_msg_send(s_svc_led_queue_handle, &msg, 0U))
        {
            SVC_LED_TASK_LOG("[LED][WARN] queue_full state=logo_test");
        }
#endif
    }

    return resumed;
}

void svc_led_task_mark_sleep(void)
{
    svc_led_task_next_generation();
    s_svc_led_logo_test_generation++;
    s_svc_led_resume_expected_out = s_svc_led_expected_out;
    svc_led_task_set_expected(SVC_LED_TASK_STATE_SLEEP_OFF);
    svc_led_task_log_if_changed(SVC_LED_TASK_STATE_SLEEP_OFF);
#if (ZY100_LED_HW_ENABLE != 0U)
    svc_led_pattern_shutdown_for_sleep();
#endif
}

static bool svc_led_task_post(svc_led_task_state_t state)
{
    svc_led_task_msg_t msg;

    svc_led_task_set_expected(state);
    svc_led_task_next_generation();

    if (!svc_led_task_init())
    {
        svc_led_task_log_if_changed(state);
        return false;
    }

    msg.cmd = SVC_LED_TASK_CMD_NOTIFY;
    msg.state = state;
    msg.generation = s_svc_led_generation;
    if (!os_msg_send(s_svc_led_queue_handle, &msg, 0U))
    {
        svc_led_task_log_queue_full(state);
        return false;
    }

    s_svc_led_last_queue_full = SVC_LED_LOG_EXPECTED_INVALID;
    return true;
}

static bool svc_led_task_post_cmd(svc_led_task_cmd_t cmd, svc_led_task_state_t state)
{
    svc_led_task_msg_t msg;

    svc_led_task_set_expected(state);
    svc_led_task_next_generation();
    s_svc_led_logo_test_generation++;

    if (!svc_led_task_init())
    {
        svc_led_task_log_if_changed(state);
        return false;
    }

    msg.cmd = cmd;
    msg.state = state;
    msg.generation = s_svc_led_generation;
    if (!os_msg_send(s_svc_led_queue_handle, &msg, 0U))
    {
        svc_led_task_log_queue_full(state);
        return false;
    }

    s_svc_led_last_queue_full = SVC_LED_LOG_EXPECTED_INVALID;
    return true;
}

bool svc_led_task_notify_green_on(void)
{
    return svc_led_task_post(SVC_LED_TASK_STATE_GREEN);
}

bool svc_led_task_notify_blue_on(void)
{
    return svc_led_task_post(SVC_LED_TASK_STATE_BLUE);
}

bool svc_led_task_notify_red_on(void)
{
    return svc_led_task_post(SVC_LED_TASK_STATE_RED);
}

bool svc_led_task_notify_off(void)
{
    return svc_led_task_post(SVC_LED_TASK_STATE_OFF);
}

bool svc_led_task_all_off(void)
{
    svc_led_task_next_generation();
    s_svc_led_logo_test_generation++;
    svc_led_task_set_expected(SVC_LED_TASK_STATE_OFF);
    return svc_led_pattern_all_off();
}

void svc_led_task_cancel_sequence(void)
{
    svc_led_task_next_generation();
    s_svc_led_logo_test_generation++;
}

bool svc_led_task_logo_white_hint_on(void)
{
    return svc_led_task_post_cmd(SVC_LED_TASK_CMD_LOGO_WHITE_HINT,
                                 SVC_LED_TASK_STATE_LOGO_WHITE_HINT);
}

bool svc_led_task_resume_logo_white_hint_async(void)
{
    return svc_led_task_post_cmd(
        SVC_LED_TASK_CMD_RESUME_LOGO_WHITE_HINT,
        SVC_LED_TASK_STATE_LOGO_WHITE_HINT);
}

bool svc_led_task_logo_white_hint_immediate(void)
{
    svc_led_task_next_generation();
    s_svc_led_logo_test_generation++;
    svc_led_task_set_expected(SVC_LED_TASK_STATE_LOGO_WHITE_HINT);
    return svc_led_task_apply_logo_white_hint();
}

bool svc_led_task_logo_white_on(void)
{
    svc_led_task_next_generation();
    s_svc_led_logo_test_generation++;
    svc_led_task_set_expected(SVC_LED_TASK_STATE_LOGO_WHITE);
    return svc_led_task_apply_logo_white(true);
}

bool svc_led_task_logo_blue_on(void)
{
    svc_led_task_next_generation();
    s_svc_led_logo_test_generation++;
    svc_led_task_set_expected(SVC_LED_TASK_STATE_LOGO_BLUE);
    return svc_led_task_apply_logo_blue(true);
}

bool svc_led_task_logo_fade_out(void)
{
    bool accepted = svc_led_task_post_cmd(SVC_LED_TASK_CMD_LOGO_FADE_OUT,
                                          SVC_LED_TASK_STATE_OFF);
    s_logo_fade_requested = s_svc_led_generation;
    if (!accepted)
    {
        s_logo_fade_finished = s_logo_fade_requested;
    }
    return accepted;
}

bool svc_led_task_logo_fade_complete(void)
{
    return (s_logo_fade_requested == s_svc_led_generation) &&
           (s_logo_fade_completed == s_logo_fade_requested);
}

bool svc_led_task_logo_fade_pending(void)
{
    return (s_logo_fade_requested == s_svc_led_generation) &&
           (s_logo_fade_finished != s_logo_fade_requested);
}

bool svc_led_task_logo_off(void)
{
    svc_led_task_next_generation();
    s_svc_led_logo_test_generation++;
    svc_led_task_set_expected(SVC_LED_TASK_STATE_OFF);
    return svc_led_pattern_logo_off();
}

bool svc_led_task_wake_logo_chase_step(uint8_t step)
{
    svc_led_task_next_generation();
    s_svc_led_logo_test_generation++;
    svc_led_task_set_expected(SVC_LED_TASK_STATE_LOGO_CHASE);
    return svc_led_task_apply_wake_logo_chase_step(step);
}

bool svc_led_task_notify_blue_fast_blink(void)
{
    return svc_led_task_post_cmd(SVC_LED_TASK_CMD_NOTIFY_BLUE_FAST_BLINK,
                                 SVC_LED_TASK_STATE_BLUE_FAST_BLINK);
}

bool svc_led_task_notify_red_slow_blink(void)
{
    return svc_led_task_post_cmd(SVC_LED_TASK_CMD_NOTIFY_RED_SLOW_BLINK,
                                 SVC_LED_TASK_STATE_RED_SLOW_BLINK);
}

bool svc_led_task_notify_orange_slow_blink(void)
{
    return svc_led_task_post_cmd(SVC_LED_TASK_CMD_NOTIFY_ORANGE_SLOW_BLINK,
                                 SVC_LED_TASK_STATE_ORANGE_SLOW_BLINK);
}

bool svc_led_task_notify_blue_breath_start(bool immediate_visible)
{
    svc_led_task_msg_t msg;

    svc_led_task_set_expected(SVC_LED_TASK_STATE_BLUE_BREATH);
    svc_led_task_next_generation();
    s_svc_led_logo_test_generation++;

    if (!svc_led_task_init())
    {
        svc_led_task_log_if_changed(SVC_LED_TASK_STATE_BLUE_BREATH);
        return false;
    }

    if (immediate_visible &&
        !svc_led_task_apply_notify_blue_breath_first_frame())
    {
        return false;
    }

    msg.cmd = SVC_LED_TASK_CMD_NOTIFY_BLUE_BREATH;
    msg.state = SVC_LED_TASK_STATE_BLUE_BREATH;
    msg.generation = s_svc_led_generation;
    if (!os_msg_send(s_svc_led_queue_handle, &msg, 0U))
    {
        DBG_DIRECT("[LED][WARN] blue_breath_queue_fail");
        svc_led_task_log_queue_full(SVC_LED_TASK_STATE_BLUE_BREATH);
        return false;
    }

    s_svc_led_last_queue_full = SVC_LED_LOG_EXPECTED_INVALID;
    return true;
}

bool svc_led_task_notify_blue_breath(void)
{
    return svc_led_task_notify_blue_breath_start(false);
}

bool svc_led_task_charge_sequence_chase_step(uint8_t step)
{
    bool ok;

    svc_led_task_next_generation();
    s_svc_led_logo_test_generation++;
    svc_led_task_set_expected(SVC_LED_TASK_STATE_CHARGE_SEQUENCE);

    if (!svc_led_task_init())
    {
        svc_led_task_log_if_changed(SVC_LED_TASK_STATE_CHARGE_SEQUENCE);
        return false;
    }

    ok = svc_led_task_apply_charge_sequence_chase_step(step);
    if (ok)
    {
        svc_led_task_log_if_changed(SVC_LED_TASK_STATE_CHARGE_SEQUENCE);
    }
    else
    {
        svc_led_task_log_apply_fail(SVC_LED_TASK_STATE_CHARGE_SEQUENCE);
    }

    return ok;
}

bool svc_led_task_charge_sequence_start(void)
{
    svc_led_task_sleep_charge_breath_reset();
    return svc_led_task_charge_sequence_chase_step(0U);
}

bool svc_led_task_charge_sequence_breath_step(uint8_t blue_level)
{
    bool ok;

    svc_led_task_next_generation();
    s_svc_led_logo_test_generation++;
    svc_led_task_set_expected(SVC_LED_TASK_STATE_CHARGE_SEQUENCE);

    if (!svc_led_task_init())
    {
        svc_led_task_log_if_changed(SVC_LED_TASK_STATE_CHARGE_SEQUENCE);
        return false;
    }

    ok = svc_led_task_apply_charge_sequence_breath_level(blue_level);
    if (ok)
    {
        svc_led_task_log_if_changed(SVC_LED_TASK_STATE_CHARGE_SEQUENCE);
    }
    else
    {
        svc_led_task_log_apply_fail(SVC_LED_TASK_STATE_CHARGE_SEQUENCE);
    }

    return ok;
}

bool svc_led_task_charge_full_blue_on(void)
{
    bool ok;

    svc_led_task_next_generation();
    s_svc_led_logo_test_generation++;
    svc_led_task_set_expected(SVC_LED_TASK_STATE_CHARGE_FULL_BLUE);
    svc_led_task_sleep_charge_breath_reset();

    if (!svc_led_task_init())
    {
        svc_led_task_log_if_changed(SVC_LED_TASK_STATE_CHARGE_FULL_BLUE);
        return false;
    }

    ok = svc_led_task_apply_charge_full_blue();
    if (ok)
    {
        svc_led_task_log_if_changed(SVC_LED_TASK_STATE_CHARGE_FULL_BLUE);
    }
    else
    {
        svc_led_task_log_apply_fail(SVC_LED_TASK_STATE_CHARGE_FULL_BLUE);
    }

    return ok;
}

bool svc_led_task_sleep_charge_latch_on(void)
{
    return svc_led_task_charge_sequence_start();
}

bool svc_led_task_sleep_charge_latch_off(void)
{
    bool ok;

    svc_led_task_next_generation();
    s_svc_led_logo_test_generation++;
    svc_led_task_set_expected(SVC_LED_TASK_STATE_SLEEP_OFF);

    ok = svc_led_pattern_all_off();
    svc_led_task_log_if_changed(SVC_LED_TASK_STATE_SLEEP_OFF);
    svc_led_pattern_shutdown_for_sleep();
    svc_led_task_sleep_charge_breath_reset();
    return ok;
}

void svc_led_task_sleep_charge_breath_reset(void)
{
    s_svc_led_sleep_charge_breath_elapsed_ms = 0U;
}

bool svc_led_task_sleep_charge_breath_step(void)
{
    uint32_t elapsed_ms = s_svc_led_sleep_charge_breath_elapsed_ms;
    bool ok;

    svc_led_task_next_generation();
    s_svc_led_logo_test_generation++;
    svc_led_task_set_expected(SVC_LED_TASK_STATE_SLEEP_CHARGE);

    if (!svc_led_task_init())
    {
        svc_led_task_log_if_changed(SVC_LED_TASK_STATE_SLEEP_CHARGE);
        return false;
    }

    ok = svc_led_task_apply_sleep_charge_breath_level(
             svc_led_task_blue_breath_level(elapsed_ms));
    if (ok)
    {
        s_svc_led_sleep_charge_breath_elapsed_ms =
            svc_led_task_blue_breath_next_elapsed(elapsed_ms);
    }

    return ok;
}

bool svc_led_task_sleep_charge_full_blue_on(void)
{
    return svc_led_task_charge_full_blue_on();
}

bool svc_led_task_red_is_on_expected(void)
{
    return s_svc_led_expected_out == SVC_LED_EXPECTED_RED;
}

uint8_t svc_led_task_expected_out(void)
{
    return s_svc_led_expected_out;
}
