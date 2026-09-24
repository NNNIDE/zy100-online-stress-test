#include "zy100_offline_v2_ui.h"

#include "app_ui_policy.h"

#define OFFLINE_V2_UI_PREPARE_ON_MS       200U
#define OFFLINE_V2_UI_PREPARE_OFF_MS      200U
#define OFFLINE_V2_UI_PREPARE_CYCLES        3U
#define OFFLINE_V2_UI_PREPARE_TIMEOUT_MS  1800U
#define OFFLINE_V2_UI_COMPLETE_ON_MS       800U
#define OFFLINE_V2_UI_COMPLETE_OFF_MS      800U
#define OFFLINE_V2_UI_COMPLETE_CYCLES        3U
#define OFFLINE_V2_UI_FAULT_ON_MS          250U
#define OFFLINE_V2_UI_FAULT_OFF_MS         250U

static uint64_t s_prepare_start_ms;
static bool s_prepare_active;
typedef enum
{
    BUTTON_UI_IDLE = 0U,
    BUTTON_UI_ON,
    BUTTON_UI_OFF,
    BUTTON_UI_WAIT_RUNNING,
    BUTTON_UI_FAILED,
} button_ui_phase_t;
static button_ui_phase_t s_button_phase;
static uint64_t s_button_phase_ms;
static uint8_t s_button_flashes;

typedef enum
{
    BUTTON_STOP_UI_IDLE = 0U,
    BUTTON_STOP_UI_FLASHING,
    BUTTON_STOP_UI_DARK,
    BUTTON_STOP_UI_FAILED,
    BUTTON_STOP_UI_CANCELED,
    BUTTON_STOP_UI_DONE,
} button_stop_ui_state_t;
static button_stop_ui_state_t s_button_stop_state;
static uint64_t s_button_stop_start_ms;
static uint8_t s_button_stop_phase;
static bool s_button_stop_storage_done;

static void button_stop_ui_reset(void)
{
    s_button_stop_state = BUTTON_STOP_UI_IDLE;
    s_button_stop_start_ms = 0ULL;
    s_button_stop_phase = 0U;
    s_button_stop_storage_done = false;
}


void zy100_offline_v2_ui_init(void)
{
    button_stop_ui_reset();
    s_prepare_start_ms = 0ULL;
    s_prepare_active = false;
    s_button_phase = BUTTON_UI_IDLE;
    s_button_phase_ms = 0ULL;
    s_button_flashes = 0U;
}

static bool button_ui_apply(bool on, uint64_t now_ms)
{
    bool ok = on ?
        app_led_request_notify_solid(LED_OWNER_OFFLINE_V2_TRANSITION,
            LED_PRIORITY_OFFLINE_V2_TRANSITION, LED_PATTERN_COLOR_GREEN, true) :
        app_led_request_all_off(LED_OWNER_OFFLINE_V2_TRANSITION,
            LED_PRIORITY_OFFLINE_V2_TRANSITION);
    s_button_phase_ms = now_ms;
    if (!ok || led_current_owner() != LED_OWNER_OFFLINE_V2_TRANSITION)
    {
        s_button_phase = BUTTON_UI_FAILED;
        return false;
    }
    return true;
}

bool zy100_offline_v2_ui_button_prepare_begin(uint64_t now_ms)
{
    zy100_offline_v2_ui_release();
    /* The accepted second press begins the first ON phase. The arbiter
     * replaces SHORT_PRESS directly, without inserting a dark frame. */
    s_button_flashes = 0U;
    s_button_phase = BUTTON_UI_ON;
    return button_ui_apply(true, now_ms);
}

bool zy100_offline_v2_ui_button_prepare_resume(uint64_t now_ms)
{
    if (s_button_phase == BUTTON_UI_IDLE || s_button_phase == BUTTON_UI_FAILED ||
        !led_owner_is_active(LED_OWNER_OFFLINE_V2_TRANSITION) ||
        led_current_owner() != LED_OWNER_OFFLINE_V2_TRANSITION)
    {
        return false;
    }
    /* BLE recovery can finish before or after all flashes. Preserve the
     * visible phase; only the subsequent capture-preparation timeout restarts. */
    if (s_button_phase == BUTTON_UI_WAIT_RUNNING)
    {
        s_button_phase_ms = now_ms;
    }
    return true;
}

zy100_offline_v2_button_ui_result_t zy100_offline_v2_ui_button_prepare_poll(uint64_t now_ms)
{
    uint32_t duration = s_button_phase == BUTTON_UI_ON ?
        OFFLINE_V2_UI_PREPARE_ON_MS : OFFLINE_V2_UI_PREPARE_OFF_MS;
    if (s_button_phase == BUTTON_UI_IDLE || s_button_phase == BUTTON_UI_FAILED ||
        now_ms < s_button_phase_ms ||
        !led_owner_is_active(LED_OWNER_OFFLINE_V2_TRANSITION) ||
        led_current_owner() != LED_OWNER_OFFLINE_V2_TRANSITION)
    {
        s_button_phase = BUTTON_UI_FAILED;
        return ZY100_OFFLINE_V2_BUTTON_UI_FAILED;
    }
    if (s_button_phase == BUTTON_UI_WAIT_RUNNING)
        return ZY100_OFFLINE_V2_BUTTON_UI_READY;
    if (now_ms - s_button_phase_ms < duration)
        return ZY100_OFFLINE_V2_BUTTON_UI_WAIT;
    /* Advance only one visible phase per poll. A late task must not count
     * skipped flashes, release the owner, or expose a charging background. */
    if (s_button_phase == BUTTON_UI_ON)
    {
        ++s_button_flashes;
        s_button_phase = BUTTON_UI_OFF;
        if (!button_ui_apply(false, now_ms)) return ZY100_OFFLINE_V2_BUTTON_UI_FAILED;
    }
    else if (s_button_flashes == OFFLINE_V2_UI_PREPARE_CYCLES)
    {
        s_button_phase = BUTTON_UI_WAIT_RUNNING;
        s_button_phase_ms = now_ms;
        return ZY100_OFFLINE_V2_BUTTON_UI_READY;
    }
    else
    {
        s_button_phase = BUTTON_UI_ON;
        if (!button_ui_apply(true, now_ms)) return ZY100_OFFLINE_V2_BUTTON_UI_FAILED;
    }
    return ZY100_OFFLINE_V2_BUTTON_UI_WAIT;
}

bool zy100_offline_v2_ui_button_waiting(void)
{
    return s_button_phase == BUTTON_UI_WAIT_RUNNING;
}

bool zy100_offline_v2_ui_button_wait_expired(uint64_t now_ms, uint32_t timeout_ms)
{
    return s_button_phase == BUTTON_UI_WAIT_RUNNING &&
        (now_ms < s_button_phase_ms || now_ms - s_button_phase_ms >= timeout_ms ||
         led_current_owner() != LED_OWNER_OFFLINE_V2_TRANSITION);
}

bool zy100_offline_v2_ui_prepare_begin(uint64_t now_ms)
{
    zy100_offline_v2_ui_release();
    if (!app_led_request_notify_blink(LED_OWNER_OFFLINE_V2_TRANSITION,
                                      LED_PRIORITY_OFFLINE_V2_TRANSITION,
                                      LED_PATTERN_COLOR_GREEN,
                                      OFFLINE_V2_UI_PREPARE_ON_MS,
                                      OFFLINE_V2_UI_PREPARE_OFF_MS,
                                      OFFLINE_V2_UI_PREPARE_CYCLES,
                                      false))
    {
        return false;
    }
    s_prepare_start_ms = now_ms;
    s_prepare_active = true;
    return true;
}

bool zy100_offline_v2_ui_prepare_ready(uint64_t now_ms)
{
    if (!s_prepare_active)
    {
        return false;
    }
    if (!led_owner_is_active(LED_OWNER_OFFLINE_V2_TRANSITION) ||
        ((now_ms >= s_prepare_start_ms) &&
         ((now_ms - s_prepare_start_ms) >=
          OFFLINE_V2_UI_PREPARE_TIMEOUT_MS)))
    {
        led_release(LED_OWNER_OFFLINE_V2_TRANSITION);
        s_prepare_active = false;
        return true;
    }
    return false;
}

void zy100_offline_v2_ui_show_running(void)
{
    button_stop_ui_reset();
    led_release(LED_OWNER_OFFLINE_V2_TRANSITION);
    led_release(LED_OWNER_OFFLINE_V2_FAULT);
    s_prepare_active = false;
    s_button_phase = BUTTON_UI_IDLE;
    (void)app_led_request_capture_breath(LED_OWNER_OFFLINE_V2_RUNNING,
                                         LED_PRIORITY_OFFLINE_V2_RUNNING,
                                         false);
}

void zy100_offline_v2_ui_show_stopping(void)
{
    /* Keep the Offline V2 running owner until its storage END is durable. */
}

/* The transition owner stays selected during OFF phases and late storage
 * completion. Never expose the old capture pattern between foregrounds. */
static bool button_stop_ui_apply(uint8_t phase)
{
    return ((phase & 1U) != 0U) ?
        app_led_request_all_off(LED_OWNER_OFFLINE_V2_TRANSITION,
                               LED_PRIORITY_OFFLINE_V2_TRANSITION) :
        app_led_request_notify_solid(LED_OWNER_OFFLINE_V2_TRANSITION,
                                    LED_PRIORITY_OFFLINE_V2_TRANSITION,
                                    LED_PATTERN_COLOR_GREEN, false);
}

static void button_stop_ui_dark(void)
{
    if (!app_led_request_all_off(LED_OWNER_OFFLINE_V2_TRANSITION,
                                LED_PRIORITY_OFFLINE_V2_TRANSITION))
    {
        /* A failed request can still register an owner. */
        led_release(LED_OWNER_OFFLINE_V2_TRANSITION);
    }
}

static bool button_stop_ui_start(uint64_t now_ms)
{
    bool ok = button_stop_ui_apply(0U);

    /* SHORT_PRESS still covers the running owner if the request failed. */
    led_release(LED_OWNER_OFFLINE_V2_RUNNING);
    if (!ok)
    {
        led_release(LED_OWNER_OFFLINE_V2_TRANSITION);
        s_button_stop_state = BUTTON_STOP_UI_FAILED;
        button_stop_ui_dark();
        return false;
    }
    if (led_current_owner() != LED_OWNER_OFFLINE_V2_TRANSITION)
    {
        s_button_stop_state = BUTTON_STOP_UI_CANCELED;
        led_release(LED_OWNER_OFFLINE_V2_TRANSITION);
        return false;
    }
    s_button_stop_state = BUTTON_STOP_UI_FLASHING;
    s_button_stop_start_ms = now_ms;
    s_button_stop_phase = 0U;
    return true;
}

bool zy100_offline_v2_ui_button_stop_begin(uint64_t now_ms)
{
    if (s_button_stop_state != BUTTON_STOP_UI_IDLE)
    {
        return s_button_stop_state == BUTTON_STOP_UI_FLASHING;
    }
    s_button_phase = BUTTON_UI_IDLE;
    s_prepare_active = false;
    return button_stop_ui_start(now_ms);
}

void zy100_offline_v2_ui_button_stop_cancel(void)
{
    if (s_button_stop_state == BUTTON_STOP_UI_IDLE ||
        s_button_stop_state == BUTTON_STOP_UI_CANCELED ||
        s_button_stop_state == BUTTON_STOP_UI_DONE)
    {
        return;
    }
    s_button_stop_state = BUTTON_STOP_UI_CANCELED;
    led_release(LED_OWNER_OFFLINE_V2_RUNNING);
    led_release(LED_OWNER_OFFLINE_V2_TRANSITION);
}

bool zy100_offline_v2_ui_button_stop_active(void)
{
    return s_button_stop_state == BUTTON_STOP_UI_FLASHING ||
           s_button_stop_state == BUTTON_STOP_UI_DARK ||
           s_button_stop_state == BUTTON_STOP_UI_FAILED;
}

bool zy100_offline_v2_ui_button_stop_poll(uint64_t now_ms)
{
    uint64_t elapsed_ms;
    uint8_t phase;
    const uint32_t cycle_ms = OFFLINE_V2_UI_COMPLETE_ON_MS +
                              OFFLINE_V2_UI_COMPLETE_OFF_MS;

    if (s_button_stop_state != BUTTON_STOP_UI_FLASHING)
    {
        return false;
    }
    if (now_ms < s_button_stop_start_ms ||
        led_current_owner() != LED_OWNER_OFFLINE_V2_TRANSITION)
    {
        zy100_offline_v2_ui_button_stop_cancel();
        return false;
    }
    elapsed_ms = now_ms - s_button_stop_start_ms;
    if (elapsed_ms >= (uint64_t)cycle_ms * OFFLINE_V2_UI_COMPLETE_CYCLES)
    {
        if (s_button_stop_storage_done)
        {
            s_button_stop_state = BUTTON_STOP_UI_DONE;
            led_release(LED_OWNER_OFFLINE_V2_TRANSITION);
        }
        else
        {
            s_button_stop_state = BUTTON_STOP_UI_DARK;
            button_stop_ui_dark();
        }
        return s_button_stop_storage_done;
    }
    phase = (uint8_t)((elapsed_ms / cycle_ms) * 2ULL);
    if ((elapsed_ms % cycle_ms) >= OFFLINE_V2_UI_COMPLETE_ON_MS)
    {
        ++phase;
    }
    if (phase != s_button_stop_phase && button_stop_ui_apply(phase))
    {
        s_button_stop_phase = phase;
    }
    /* A transient phase failure retries on the next poll, within the same
     * finite animation deadline. It never restarts capture or the animation. */
    return false;
}

bool zy100_offline_v2_ui_button_stop_finish(uint64_t now_ms)
{
    if (s_button_stop_state == BUTTON_STOP_UI_IDLE)
    {
        return false;
    }
    s_button_stop_storage_done = true;
    if (s_button_stop_state == BUTTON_STOP_UI_FAILED)
    {
        /* Only a failed initial request can retry after the real END commit. */
        if (!button_stop_ui_start(now_ms))
        {
            s_button_stop_state = BUTTON_STOP_UI_DONE;
            led_release(LED_OWNER_OFFLINE_V2_TRANSITION);
        }
    }
    else if (s_button_stop_state == BUTTON_STOP_UI_DARK)
    {
        s_button_stop_state = BUTTON_STOP_UI_DONE;
        led_release(LED_OWNER_OFFLINE_V2_TRANSITION);
    }
    return true;
}

void zy100_offline_v2_ui_show_complete(void)
{
    s_button_phase = BUTTON_UI_IDLE;
    led_release(LED_OWNER_OFFLINE_V2_RUNNING);
    led_release(LED_OWNER_OFFLINE_V2_FAULT);
    if (!app_led_request_notify_blink(LED_OWNER_OFFLINE_V2_TRANSITION,
                                      LED_PRIORITY_OFFLINE_V2_TRANSITION,
                                      LED_PATTERN_COLOR_GREEN,
                                      OFFLINE_V2_UI_COMPLETE_ON_MS,
                                      OFFLINE_V2_UI_COMPLETE_OFF_MS,
                                      OFFLINE_V2_UI_COMPLETE_CYCLES,
                                      false))
    {
        /* Failed application still registers an owner; do not orphan it. */
        led_release(LED_OWNER_OFFLINE_V2_TRANSITION);
    }
}

void zy100_offline_v2_ui_show_capacity(void)
{
    zy100_offline_v2_ui_button_stop_cancel();
    s_button_phase = BUTTON_UI_IDLE;
    led_release(LED_OWNER_OFFLINE_V2_RUNNING);
    led_release(LED_OWNER_OFFLINE_V2_TRANSITION);
    if (!app_led_request_notify_blink(LED_OWNER_OFFLINE_V2_FAULT,
                                      LED_PRIORITY_OFFLINE_V2_FAULT,
                                      LED_PATTERN_COLOR_RED,
                                      OFFLINE_V2_UI_FAULT_ON_MS,
                                      OFFLINE_V2_UI_FAULT_OFF_MS,
                                      10U,
                                      false))
    {
        /* Failed application still registers an owner; do not orphan it. */
        led_release(LED_OWNER_OFFLINE_V2_FAULT);
    }
}

void zy100_offline_v2_ui_show_fault(void)
{
    zy100_offline_v2_ui_button_stop_cancel();
    s_button_phase = BUTTON_UI_IDLE;
    led_release(LED_OWNER_OFFLINE_V2_RUNNING);
    led_release(LED_OWNER_OFFLINE_V2_TRANSITION);
    if (!app_led_request_notify_blink(LED_OWNER_OFFLINE_V2_FAULT,
                                      LED_PRIORITY_OFFLINE_V2_FAULT,
                                      LED_PATTERN_COLOR_RED,
                                      OFFLINE_V2_UI_FAULT_ON_MS,
                                      OFFLINE_V2_UI_FAULT_OFF_MS,
                                      6U,
                                      false))
    {
        /* Failed application still registers an owner; do not orphan it. */
        led_release(LED_OWNER_OFFLINE_V2_FAULT);
    }
}

void zy100_offline_v2_ui_release(void)
{
    button_stop_ui_reset();
    led_release(LED_OWNER_OFFLINE_V2_TRANSITION);
    led_release(LED_OWNER_OFFLINE_V2_RUNNING);
    led_release(LED_OWNER_OFFLINE_V2_FAULT);
    s_prepare_start_ms = 0ULL;
    s_prepare_active = false;
    s_button_phase = BUTTON_UI_IDLE;
    s_button_phase_ms = 0ULL;
    s_button_flashes = 0U;
}
