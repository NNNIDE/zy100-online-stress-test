#include "zy100_online_ui.h"

#include <stddef.h>

#include "app_flags.h"
#include "app_ui_policy.h"
#include "zy100_clock_config.h"

#define ONLINE_UI_PREPARE_TIMEOUT_MS 1800U
#define ONLINE_UI_COMPLETE_ON_MS      800U
#define ONLINE_UI_COMPLETE_OFF_MS     800U
#define ONLINE_UI_COMPLETE_CYCLES     3U
#define ONLINE_UI_COMPLETE_PHASES     (ONLINE_UI_COMPLETE_CYCLES * 2U)
#define ONLINE_UI_COMPLETE_TOTAL_MS   \
    ((ONLINE_UI_COMPLETE_ON_MS + ONLINE_UI_COMPLETE_OFF_MS) * \
     ONLINE_UI_COMPLETE_CYCLES)
#define ONLINE_UI_COMPLETE_TIMEOUT_MS 5600U

static uint64_t s_prepare_start_ms;
static bool s_prepare_active;
static uint64_t s_complete_start_ms;
static bool s_complete_active;
static uint8_t s_complete_phase;
static bool s_complete_phase_fail_logged;

static bool zy100_online_ui_complete_apply_phase(uint8_t phase)
{
    if ((phase >= (uint8_t)ONLINE_UI_COMPLETE_PHASES) ||
        ((phase & 1U) != 0U))
    {
        return app_led_request_notify_off(LED_OWNER_ONLINE_TRANSITION,
                                          LED_PRIORITY_ONLINE_TRANSITION,
                                          false);
    }

    return app_led_request_notify_solid(LED_OWNER_ONLINE_TRANSITION,
                                        LED_PRIORITY_ONLINE_TRANSITION,
                                        LED_PATTERN_COLOR_GREEN,
                                        false);
}

static uint8_t zy100_online_ui_complete_phase_for_elapsed(
    uint64_t elapsed_ms)
{
    const uint64_t cycle_ms =
        (uint64_t)ONLINE_UI_COMPLETE_ON_MS +
        (uint64_t)ONLINE_UI_COMPLETE_OFF_MS;
    uint64_t cycle;
    uint8_t phase;

    cycle = elapsed_ms / cycle_ms;
    if (cycle >= (uint64_t)ONLINE_UI_COMPLETE_CYCLES)
    {
        return (uint8_t)ONLINE_UI_COMPLETE_PHASES;
    }

    phase = (uint8_t)(cycle * 2ULL);
    if ((elapsed_ms % cycle_ms) >= (uint64_t)ONLINE_UI_COMPLETE_ON_MS)
    {
        phase++;
    }
    return phase;
}

void zy100_online_ui_init(void)
{
    s_prepare_start_ms = 0ULL;
    s_prepare_active = false;
    s_complete_start_ms = 0ULL;
    s_complete_active = false;
    s_complete_phase = 0U;
    s_complete_phase_fail_logged = false;
}

bool zy100_online_ui_prepare_begin(uint64_t now_ms)
{
    zy100_online_ui_release();
    if (!app_led_request_notify_blink(LED_OWNER_ONLINE_TRANSITION,
                                      LED_PRIORITY_ONLINE_TRANSITION,
                                      LED_PATTERN_COLOR_GREEN,
                                      200U, 200U, 3U, false))
    {
        return false;
    }
    s_prepare_start_ms = now_ms;
    s_prepare_active = true;
    return true;
}

bool zy100_online_ui_prepare_ready(uint64_t now_ms)
{
    if (!s_prepare_active)
    {
        return false;
    }
    if (!led_owner_is_active(LED_OWNER_ONLINE_TRANSITION) ||
        ((now_ms >= s_prepare_start_ms) &&
         ((now_ms - s_prepare_start_ms) >= ONLINE_UI_PREPARE_TIMEOUT_MS)))
    {
        led_release(LED_OWNER_ONLINE_TRANSITION);
        s_prepare_active = false;
        return true;
    }
    return false;
}

void zy100_online_ui_show_running(void)
{
    led_release(LED_OWNER_ONLINE_TRANSITION);
    led_release(LED_OWNER_ONLINE_FAULT);
    s_prepare_active = false;
    (void)app_led_request_capture_breath(LED_OWNER_ONLINE_RUNNING,
                                         LED_PRIORITY_ONLINE_RUNNING,
                                         true);
}

bool zy100_online_ui_show_complete(uint64_t now_ms)
{
    zy100_online_ui_cancel_complete("restart_complete");
    led_release(LED_OWNER_ONLINE_RUNNING);
    led_release(LED_OWNER_ONLINE_FAULT);
    led_release(LED_OWNER_ONLINE_TRANSITION);
    s_prepare_start_ms = 0ULL;
    s_prepare_active = false;
    if (!zy100_online_ui_complete_apply_phase(0U))
    {
        ZY100_LOG_WARN("[ONLINE_END_LED] done result=request_failed owner=%u pattern=%u",
                       (uint32_t)led_current_owner(),
                       (uint32_t)led_current_pattern_id());
        /* A failed request is still registered with the LED arbiter. */
        led_release(LED_OWNER_ONLINE_TRANSITION);
        return false;
    }
    s_complete_start_ms = now_ms;
    s_complete_active = true;
    s_complete_phase = 0U;
    s_complete_phase_fail_logged = false;
    return true;
}

bool zy100_online_ui_poll(uint64_t now_ms)
{
    uint64_t elapsed_ms;
    uint8_t phase;
    bool owner_active;
    bool request_ok;

    if (!s_complete_active)
    {
        return false;
    }

    elapsed_ms = zy100_online_ui_complete_elapsed_ms(now_ms);
    if (elapsed_ms >= (uint64_t)ONLINE_UI_COMPLETE_TIMEOUT_MS)
    {
        led_release(LED_OWNER_ONLINE_TRANSITION);
        s_complete_active = false;
        s_complete_start_ms = 0ULL;
        s_complete_phase = 0U;
        s_complete_phase_fail_logged = false;
        ZY100_LOG_WARN("[ONLINE_END_LED] done elapsed_ms=%llu result=timeout",
                       (unsigned long long)elapsed_ms);
        return true;
    }
    if (elapsed_ms >= (uint64_t)ONLINE_UI_COMPLETE_TOTAL_MS)
    {
        led_owner_t owner = led_current_owner();
        led_pattern_id_t pattern = led_current_pattern_id();

        led_release(LED_OWNER_ONLINE_TRANSITION);
        s_complete_active = false;
        s_complete_start_ms = 0ULL;
        s_complete_phase = 0U;
        s_complete_phase_fail_logged = false;
        ZY100_LOG_EVENT("[ONLINE_END_LED] done elapsed_ms=%llu result=complete owner=%u pattern=%u",
                        (unsigned long long)elapsed_ms,
                        (uint32_t)owner,
                        (uint32_t)pattern);
        return true;
    }

    phase = zy100_online_ui_complete_phase_for_elapsed(elapsed_ms);
    owner_active = led_owner_is_active(LED_OWNER_ONLINE_TRANSITION);
    if ((phase == s_complete_phase) && owner_active)
    {
        return false;
    }

    request_ok = zy100_online_ui_complete_apply_phase(phase);
    if (!request_ok)
    {
        if (!s_complete_phase_fail_logged)
        {
            ZY100_LOG_WARN("[ONLINE_END_LED] phase_apply_failed elapsed_ms=%llu phase=%u owner=%u pattern=%u",
                           (unsigned long long)elapsed_ms,
                           (uint32_t)phase,
                           (uint32_t)led_current_owner(),
                           (uint32_t)led_current_pattern_id());
            s_complete_phase_fail_logged = true;
        }
        return false;
    }

    if (!owner_active)
    {
        ZY100_LOG_WARN("[ONLINE_END_LED] owner_recovered elapsed_ms=%llu phase=%u owner=%u pattern=%u",
                       (unsigned long long)elapsed_ms,
                       (uint32_t)phase,
                       (uint32_t)led_current_owner(),
                       (uint32_t)led_current_pattern_id());
    }
    s_complete_phase = phase;
    s_complete_phase_fail_logged = false;
    return false;
}

bool zy100_online_ui_complete_active(void)
{
    return s_complete_active;
}

uint64_t zy100_online_ui_complete_elapsed_ms(uint64_t now_ms)
{
    if (!s_complete_active || (s_complete_start_ms == 0ULL) ||
        (now_ms < s_complete_start_ms))
    {
        return 0ULL;
    }
    return now_ms - s_complete_start_ms;
}

void zy100_online_ui_cancel_complete(const char *reason)
{
    uint64_t now_ms;
    uint64_t elapsed_ms;
    led_owner_t owner;
    led_pattern_id_t pattern;

    if (!s_complete_active)
    {
        return;
    }

    now_ms = zy100_os_time_ms();
    elapsed_ms = zy100_online_ui_complete_elapsed_ms(now_ms);
    owner = led_current_owner();
    pattern = led_current_pattern_id();
    led_release(LED_OWNER_ONLINE_TRANSITION);
    s_complete_active = false;
    s_complete_start_ms = 0ULL;
    s_complete_phase = 0U;
    s_complete_phase_fail_logged = false;
    ZY100_LOG_EVENT("[ONLINE_END_LED] canceled trigger=%s elapsed_ms=%llu owner=%u pattern=%u",
                    (reason != NULL) ? reason : "new_operation",
                    (unsigned long long)elapsed_ms,
                    (uint32_t)owner,
                    (uint32_t)pattern);
}

void zy100_online_ui_show_fault(void)
{
    zy100_online_ui_cancel_complete("online_fault");
    led_release(LED_OWNER_ONLINE_RUNNING);
    led_release(LED_OWNER_ONLINE_TRANSITION);
    (void)app_led_request_notify_blink(LED_OWNER_ONLINE_FAULT,
                                      LED_PRIORITY_ONLINE_HARD_FAULT,
                                      LED_PATTERN_COLOR_RED,
                                      250U, 250U, 6U, false);
}

void zy100_online_ui_release(void)
{
    zy100_online_ui_cancel_complete("online_ui_release");
    led_release(LED_OWNER_ONLINE_TRANSITION);
    led_release(LED_OWNER_ONLINE_RUNNING);
    led_release(LED_OWNER_ONLINE_FAULT);
    s_prepare_start_ms = 0ULL;
    s_prepare_active = false;
}
