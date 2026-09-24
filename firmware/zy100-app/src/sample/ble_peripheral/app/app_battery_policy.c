#include "app/app_battery_policy.h"

#include <stddef.h>
#include <trace.h>

#include "app_flags.h"
#include "zy100_clock_config.h"
#include "app/app_ui_policy.h"
#include "service/battery_adc.h"
#include "service/battery_adc_guard.h"
#include "service/battery_low_guard.h"
#include "service/imu_fifo_drain_test.h"
#include "service/svc_led_owner.h"
#include "service/zy100_capture_time.h"
#if ZY100_OFFLINE_FEATURE_V2_ENABLE
#include "service/zy100_offline_v2_capture.h"
#endif

typedef struct
{
    bool critical_locked;
    uint64_t critical_red_blink_last_ms;
#if F_APP_BATTERY_ADC_ENABLE
    bool adc_mode_valid;
    bool adc_attempt_pending;
    bool adc_capture_active;
    app_battery_adc_mode_t adc_mode;
    uint32_t adc_next_due_ms;
    bool bas_reference_valid;
    uint8_t bas_reference_percent;
    bool bas_notify_pending;
    uint8_t bas_notify_percent;
    uint32_t bas_next_notify_ms;
#endif
#if F_APP_BATTERY_ADC_ENABLE && F_APP_BATTERY_ADC_GUARD_ENABLE
    bool guard_bas_active;
    bool guard_bas_valid;
    uint8_t guard_bas_percent;
    uint16_t guard_bas_raw;
    uint16_t guard_bas_mv;
    uint32_t guard_bas_check_count;
    uint32_t guard_bas_set_count;
    uint32_t guard_bas_set_fail_count;
    uint32_t guard_bas_notify_count;
    uint32_t guard_bas_notify_fail_count;
#endif
#if F_APP_BATTERY_LPC_GUARD_ENABLE
    bool lpc_arm_pending;
    uint32_t lpc_arm_due_ms;
#endif
} app_battery_policy_context_t;

static app_battery_policy_context_t s_battery_policy;

#define APP_BATTERY_CRITICAL_BLINK_ON_MS  200U
#define APP_BATTERY_CRITICAL_BLINK_OFF_MS 200U

static void app_task_battery_adc_guard_arm_capture(const char *reason);
static void app_task_battery_adc_guard_disarm_capture(const char *reason);

#if F_APP_BATTERY_ADC_ENABLE
static bool app_battery_policy_time_reached(uint32_t now_ms, uint32_t due_ms)
{
    return ((int32_t)(now_ms - due_ms) >= 0);
}

static uint32_t app_battery_policy_adc_period_ms(app_battery_adc_mode_t mode)
{
    switch (mode)
    {
    case APP_BATTERY_ADC_MODE_STANDBY_FULL:
    case APP_BATTERY_ADC_MODE_STANDBY_CHARGE:
        return (uint32_t)ZY100_STANDBY_ADC_PERIOD_MS;

    case APP_BATTERY_ADC_MODE_ACTIVE:
        return (uint32_t)APP_BATTERY_ACTIVE_ADC_PERIOD_MS;

    case APP_BATTERY_ADC_MODE_CHARGE_PARTIAL_SLEEP:
        return (uint32_t)APP_BATTERY_CHARGE_PARTIAL_ADC_PERIOD_MS;

    case APP_BATTERY_ADC_MODE_FULL_SLEEP:
    case APP_BATTERY_ADC_MODE_COUNT:
    default:
        return 0U;
    }
}

bool app_battery_policy_adc_set_mode(app_battery_adc_mode_t mode,
                                     uint32_t runtime_ms)
{
    bool first_mode;

    if (mode >= APP_BATTERY_ADC_MODE_COUNT)
    {
        return false;
    }

    first_mode = !s_battery_policy.adc_mode_valid;
    if (!first_mode && (s_battery_policy.adc_mode == mode))
    {
        return true;
    }

    if (!s_battery_policy.adc_capture_active &&
        (s_battery_policy.adc_attempt_pending || battery_adc_is_active()))
    {
        battery_adc_runtime_cancel_pending_attempt(runtime_ms);
    }

    s_battery_policy.adc_mode_valid = true;
    s_battery_policy.adc_mode = mode;
    s_battery_policy.adc_attempt_pending = false;
    if (mode == APP_BATTERY_ADC_MODE_FULL_SLEEP)
    {
        s_battery_policy.adc_next_due_ms = 0U;
    }
    else if (first_mode && (mode == APP_BATTERY_ADC_MODE_ACTIVE))
    {
        s_battery_policy.adc_next_due_ms =
            runtime_ms + (uint32_t)BATTERY_ADC_FIRST_DELAY_MS;
    }
    else
    {
        s_battery_policy.adc_next_due_ms = runtime_ms;
    }
    return true;
}

bool app_battery_policy_adc_work_due(app_battery_adc_mode_t mode,
                                     uint32_t runtime_ms)
{
    if (!app_battery_policy_adc_set_mode(mode, runtime_ms) ||
        s_battery_policy.adc_capture_active ||
        (mode == APP_BATTERY_ADC_MODE_FULL_SLEEP))
    {
        return false;
    }

    return s_battery_policy.adc_attempt_pending ||
           app_battery_policy_time_reached(runtime_ms,
                                           s_battery_policy.adc_next_due_ms);
}

bool app_battery_policy_adc_runtime_poll(app_battery_adc_mode_t mode,
                                         uint32_t runtime_ms,
                                         bool sample_allowed,
                                         bool resume_before_start)
{
    uint32_t retry_ms;
    bool completed;

    if (!app_battery_policy_adc_work_due(mode, runtime_ms))
    {
        return false;
    }

    /*
     * A due sample that is currently gated is deferred, not started.  Keep
     * adc_next_due_ms expired so the full attempt is retried as soon as the
     * gate opens.  adc_attempt_pending is reserved for a real low-level ADC
     * attempt that has already started.
     */
    if (!sample_allowed)
    {
        if (s_battery_policy.adc_attempt_pending || battery_adc_is_active())
        {
            battery_adc_runtime_cancel_pending_attempt(runtime_ms);
        }
        s_battery_policy.adc_attempt_pending = false;
        return false;
    }

    if (!s_battery_policy.adc_attempt_pending)
    {
        if (resume_before_start)
        {
            battery_adc_resume_after_dlps(runtime_ms);
        }
        else
        {
            battery_adc_runtime_force_due(runtime_ms);
        }
        s_battery_policy.adc_attempt_pending = true;
    }

    completed = battery_adc_runtime_tick(runtime_ms, true);
    if (completed)
    {
        s_battery_policy.adc_attempt_pending = false;
        s_battery_policy.adc_next_due_ms =
            runtime_ms + app_battery_policy_adc_period_ms(mode);
        return true;
    }

    if (battery_adc_is_active())
    {
        return false;
    }

    retry_ms = battery_adc_runtime_wait_ms(runtime_ms);
    if (retry_ms == 0U)
    {
        retry_ms = app_battery_policy_adc_period_ms(mode);
    }
    s_battery_policy.adc_attempt_pending = false;
    s_battery_policy.adc_next_due_ms = runtime_ms + retry_ms;
    return false;
}

void app_battery_policy_adc_force_due(uint32_t runtime_ms)
{
    if (!s_battery_policy.adc_capture_active &&
        s_battery_policy.adc_mode_valid &&
        (s_battery_policy.adc_mode != APP_BATTERY_ADC_MODE_FULL_SLEEP))
    {
        s_battery_policy.adc_next_due_ms = runtime_ms;
    }
}

void app_battery_policy_adc_cancel(uint32_t runtime_ms)
{
    uint32_t period_ms;

    if (!s_battery_policy.adc_capture_active)
    {
        battery_adc_runtime_cancel_pending_attempt(runtime_ms);
    }
    s_battery_policy.adc_attempt_pending = false;
    period_ms = app_battery_policy_adc_period_ms(s_battery_policy.adc_mode);
    if (s_battery_policy.adc_mode_valid && (period_ms != 0U))
    {
        s_battery_policy.adc_next_due_ms = runtime_ms + period_ms;
    }
}

bool app_battery_policy_adc_attempt_pending(void)
{
    return s_battery_policy.adc_attempt_pending;
}

static bool app_battery_policy_bas_update(uint8_t percent,
                                          uint32_t runtime_ms)
{
    if (!app_battery_policy_port_battery_service_set(percent))
    {
        return false;
    }

    if (!s_battery_policy.bas_reference_valid)
    {
        s_battery_policy.bas_reference_valid = true;
        s_battery_policy.bas_reference_percent = percent;
        s_battery_policy.bas_notify_pending = false;
        s_battery_policy.bas_next_notify_ms =
            runtime_ms +
            (uint32_t)APP_BATTERY_BAS_NOTIFY_MIN_INTERVAL_MS;
        return true;
    }

    if (percent == s_battery_policy.bas_reference_percent)
    {
        s_battery_policy.bas_notify_pending = false;
        return true;
    }

    s_battery_policy.bas_notify_percent = percent;
    s_battery_policy.bas_notify_pending = true;
    return true;
}

static bool app_battery_policy_bas_notify_now(uint8_t percent,
                                              uint32_t runtime_ms)
{
    (void)app_battery_policy_port_battery_service_set(percent);
    if (!app_battery_policy_port_battery_service_notify_ready() ||
        !app_battery_policy_port_battery_service_notify(percent))
    {
        s_battery_policy.bas_notify_percent = percent;
        s_battery_policy.bas_notify_pending = true;
        s_battery_policy.bas_next_notify_ms = runtime_ms;
        return false;
    }

    s_battery_policy.bas_reference_valid = true;
    s_battery_policy.bas_reference_percent = percent;
    s_battery_policy.bas_notify_pending = false;
    s_battery_policy.bas_next_notify_ms =
        runtime_ms + (uint32_t)APP_BATTERY_BAS_NOTIFY_MIN_INTERVAL_MS;
    return true;
}

static void app_battery_policy_bas_poll(uint32_t runtime_ms,
                                        bool idle_window)
{
    uint8_t percent;

    if (!s_battery_policy.bas_notify_pending ||
        !idle_window ||
        !app_battery_policy_time_reached(runtime_ms,
                                         s_battery_policy.bas_next_notify_ms) ||
        !app_battery_policy_port_battery_service_notify_ready())
    {
        return;
    }

    percent = s_battery_policy.bas_notify_percent;
    if (app_battery_policy_port_battery_service_notify(percent))
    {
        s_battery_policy.bas_reference_valid = true;
        s_battery_policy.bas_reference_percent = percent;
        s_battery_policy.bas_notify_pending = false;
        s_battery_policy.bas_next_notify_ms =
            runtime_ms +
            (uint32_t)APP_BATTERY_BAS_NOTIFY_MIN_INTERVAL_MS;
#if F_APP_BATTERY_ADC_GUARD_ENABLE
        if (s_battery_policy.guard_bas_active)
        {
            s_battery_policy.guard_bas_notify_count++;
        }
#endif
        return;
    }

#if F_APP_BATTERY_ADC_GUARD_ENABLE
    if (s_battery_policy.guard_bas_active)
    {
        s_battery_policy.guard_bas_notify_fail_count++;
    }
#endif
    s_battery_policy.bas_next_notify_ms =
        runtime_ms + (uint32_t)BATTERY_BAS_RETRY_MS;
}

bool app_battery_policy_adc_sample_once(uint32_t runtime_ms,
                                        uint8_t *percent_out)
{
    uint8_t percent;
    bool sampled;

    if (percent_out == NULL)
    {
        return false;
    }

    sampled = battery_adc_sample_once();
    percent = battery_adc_get_percent();
    *percent_out = percent;
    s_battery_policy.adc_mode_valid = true;
    s_battery_policy.adc_mode = APP_BATTERY_ADC_MODE_ACTIVE;
    s_battery_policy.adc_attempt_pending = false;
    s_battery_policy.adc_next_due_ms =
        runtime_ms + (uint32_t)BATTERY_ADC_FIRST_DELAY_MS;
    if (sampled)
    {
        (void)app_battery_policy_bas_update(percent, runtime_ms);
    }
    return sampled;
}

bool app_battery_policy_adc_commit_current(uint32_t runtime_ms)
{
    uint8_t percent = battery_adc_get_percent();
    bool updated = app_battery_policy_bas_update(percent, runtime_ms);

    app_battery_policy_handle_level(percent, runtime_ms);
    return updated;
}

bool app_battery_policy_adc_commit_bas_current(uint32_t runtime_ms)
{
    return app_battery_policy_bas_update(battery_adc_get_percent(), runtime_ms);
}

#if F_APP_BATTERY_ADC_GUARD_ENABLE

static void app_battery_policy_guard_bas_arm(uint32_t now_ms)
{
    s_battery_policy.guard_bas_active = true;
    s_battery_policy.guard_bas_valid = false;
    s_battery_policy.guard_bas_percent = 0U;
    s_battery_policy.guard_bas_raw = 0U;
    s_battery_policy.guard_bas_mv = 0U;
    s_battery_policy.guard_bas_check_count = 0U;
    s_battery_policy.guard_bas_set_count = 0U;
    s_battery_policy.guard_bas_set_fail_count = 0U;
    s_battery_policy.guard_bas_notify_count = 0U;
    s_battery_policy.guard_bas_notify_fail_count = 0U;
    (void)now_ms;
}

static void app_battery_policy_guard_bas_disarm(const char *reason)
{
    if (!s_battery_policy.guard_bas_active)
    {
        return;
    }

    s_battery_policy.guard_bas_active = false;
    (void)reason;
}

static void app_battery_policy_guard_bas_poll(uint32_t runtime_ms,
                                              bool idle_window)
{
    battery_adc_guard_report_t report;

    if (battery_adc_guard_take_report(&report))
    {
        s_battery_policy.guard_bas_percent = report.percent;
        s_battery_policy.guard_bas_raw = report.raw;
        s_battery_policy.guard_bas_mv = report.battery_mv;
        s_battery_policy.guard_bas_valid = true;
        s_battery_policy.guard_bas_check_count = report.check_count;
        if (app_battery_policy_bas_update(report.percent, runtime_ms))
        {
            s_battery_policy.guard_bas_set_count++;
        }
        else
        {
            s_battery_policy.guard_bas_set_fail_count++;
        }
    }

    (void)idle_window;
}
#endif
#endif

#if F_APP_BATTERY_ADC_ENABLE
bool app_battery_policy_prepare_capture(const char *reason)
{
#if F_APP_BATTERY_ADC_GUARD_ENABLE
    uint32_t now_ms = (uint32_t)zy100_os_time_ms();

    if (s_battery_policy.critical_locked ||
        app_battery_policy_port_runtime_shutdown_active())
    {
        DBG_DIRECT("[BAT_ADC_GUARD][PREP] fail reason=%s locked=%u shutdown=%u",
                   (reason != NULL) ? reason : "capture_start",
                   s_battery_policy.critical_locked ? 1U : 0U,
                   app_battery_policy_port_runtime_shutdown_active() ? 1U : 0U);
        return false;
    }

    if (!battery_adc_guard_prepare_capture(now_ms))
    {
        DBG_DIRECT("[BAT_ADC_GUARD][PREP] fail reason=%s",
                   (reason != NULL) ? reason : "capture_start");
        return false;
    }

    s_battery_policy.adc_attempt_pending = false;
    return true;
#else
    (void)reason;
    return true;
#endif
}

static bool app_task_battery_critical_led_blocked(void)
{
    return app_battery_policy_port_critical_led_blocked();
}

void app_battery_policy_critical_led_stop(void)
{
    s_battery_policy.critical_red_blink_last_ms = 0ULL;
    led_release(LED_OWNER_BATTERY_CRITICAL);
}

static void app_battery_policy_critical_led_begin(uint64_t runtime_ms)
{
    if (!s_battery_policy.critical_locked ||
        app_battery_policy_port_runtime_shutdown_active() ||
        app_task_battery_critical_led_blocked())
    {
        return;
    }

    app_battery_policy_port_prepare_critical_led();
    s_battery_policy.critical_red_blink_last_ms = runtime_ms;
    (void)app_led_request_notify_blink(LED_OWNER_BATTERY_CRITICAL,
                                       LED_PRIORITY_BATTERY_CRITICAL,
                                       LED_PATTERN_COLOR_RED,
                                       APP_BATTERY_CRITICAL_BLINK_ON_MS,
                                       APP_BATTERY_CRITICAL_BLINK_OFF_MS,
                                       0U,
                                       true);
}

void app_battery_policy_critical_led_tick(uint64_t runtime_ms)
{
    if (!s_battery_policy.critical_locked)
    {
        return;
    }
    if (app_battery_policy_port_runtime_shutdown_active())
    {
        return;
    }
    if (app_task_battery_critical_led_blocked())
    {
        return;
    }
    if (s_battery_policy.critical_red_blink_last_ms == 0ULL)
    {
        app_battery_policy_critical_led_begin(runtime_ms);
        return;
    }

    (void)app_led_request_notify_blink(LED_OWNER_BATTERY_CRITICAL,
                                       LED_PRIORITY_BATTERY_CRITICAL,
                                       LED_PATTERN_COLOR_RED,
                                       APP_BATTERY_CRITICAL_BLINK_ON_MS,
                                       APP_BATTERY_CRITICAL_BLINK_OFF_MS,
                                       0U,
                                       true);
}

void app_battery_policy_arm_capture(const char *reason)
{
    app_task_battery_adc_guard_arm_capture(reason);
#if F_APP_BATTERY_LPC_GUARD_ENABLE
    uint32_t now_ms = (uint32_t)zy100_os_time_ms();

    if (s_battery_policy.critical_locked ||
        app_battery_policy_port_runtime_shutdown_active())
    {
        s_battery_policy.lpc_arm_pending = false;
        s_battery_policy.lpc_arm_due_ms = 0U;
        DBG_DIRECT("[BAT_LPC] arm_skip reason=%s locked=%u shutdown=%u",
                   (reason != NULL) ? reason : "capture_start",
                   s_battery_policy.critical_locked ? 1U : 0U,
                   app_battery_policy_port_runtime_shutdown_active() ? 1U : 0U);
        return;
    }

    s_battery_policy.lpc_arm_pending = true;
    s_battery_policy.lpc_arm_due_ms =
        now_ms + (uint32_t)BATTERY_LPC_GUARD_CAPTURE_ARM_DELAY_MS;
    DBG_DIRECT("[BAT_LPC] arm_schedule reason=%s now_ms=%lu due_ms=%lu delay_ms=%u",
               (reason != NULL) ? reason : "capture_start",
               (unsigned long)now_ms,
               (unsigned long)s_battery_policy.lpc_arm_due_ms,
               (unsigned int)BATTERY_LPC_GUARD_CAPTURE_ARM_DELAY_MS);
#else
    (void)reason;
#endif
}

void app_battery_policy_disarm_capture(const char *reason)
{
    app_task_battery_adc_guard_disarm_capture(reason);
#if F_APP_BATTERY_LPC_GUARD_ENABLE
    bool pending = s_battery_policy.lpc_arm_pending;

    s_battery_policy.lpc_arm_pending = false;
    s_battery_policy.lpc_arm_due_ms = 0U;
    if (pending)
    {
        DBG_DIRECT("[BAT_LPC] arm_cancel reason=%s",
                   (reason != NULL) ? reason : "disarm");
    }
    battery_low_guard_on_capture_stop((uint32_t)zy100_os_time_ms(), reason);
#else
    (void)reason;
#endif
}

static void app_battery_policy_lpc_poll(uint32_t runtime_ms)
{
#if F_APP_BATTERY_LPC_GUARD_ENABLE
    if (!s_battery_policy.lpc_arm_pending)
    {
        return;
    }

    if (s_battery_policy.critical_locked ||
        app_battery_policy_port_runtime_shutdown_active() ||
        !app_battery_policy_port_capture_state_active() ||
        !app_battery_policy_port_capture_active() ||
        imu_fifo_drain_test_is_stop_in_progress())
    {
        DBG_DIRECT("[BAT_LPC] arm_cancel runtime_ms=%lu state=%u active=%u stop_in_progress=%u locked=%u shutdown=%u",
                   (unsigned long)runtime_ms,
                   app_battery_policy_port_capture_state_active() ? 1U : 0U,
                   app_battery_policy_port_capture_active() ? 1U : 0U,
                   imu_fifo_drain_test_is_stop_in_progress() ? 1U : 0U,
                   s_battery_policy.critical_locked ? 1U : 0U,
                   app_battery_policy_port_runtime_shutdown_active() ? 1U : 0U);
        s_battery_policy.lpc_arm_pending = false;
        s_battery_policy.lpc_arm_due_ms = 0U;
        battery_low_guard_on_capture_stop(runtime_ms, "arm_cancel");
        return;
    }

    if ((int32_t)(runtime_ms - s_battery_policy.lpc_arm_due_ms) < 0)
    {
        return;
    }

    s_battery_policy.lpc_arm_pending = false;
    s_battery_policy.lpc_arm_due_ms = 0U;
    (void)battery_low_guard_on_capture_start(runtime_ms);
#else
    (void)runtime_ms;
#endif
}

static bool app_task_battery_critical_request_stop_if_needed_with_reason(
    const app_battery_critical_event_t *event,
    const char *active_reason,
    const char *stopping_reason)
{
    (void)event;
#if V0_IMU_FIFO_DRAIN_TEST && V1_IMU_FLASH_CAPTURE_ENABLE
    if (app_battery_policy_port_capture_state_active() &&
        !imu_fifo_drain_test_is_stop_in_progress())
    {
        app_battery_policy_disarm_capture(active_reason);
        app_battery_policy_port_request_critical_stop(event);
        return true;
    }
    if (app_battery_policy_port_capture_active() ||
        imu_fifo_drain_test_is_busy() ||
        imu_fifo_drain_test_is_stop_in_progress())
    {
        app_battery_policy_disarm_capture(stopping_reason);
        app_battery_policy_port_request_critical_stop(event);
        return true;
    }
#endif
    return false;
}

static bool app_task_battery_critical_request_stop_if_needed(
    const app_battery_critical_event_t *event)
{
    return app_task_battery_critical_request_stop_if_needed_with_reason(
               event,
               "battery_low_capture",
               "battery_low_capture_stopping");
}

static void app_task_battery_adc_guard_arm_capture(const char *reason)
{
#if F_APP_BATTERY_ADC_GUARD_ENABLE
    uint32_t now_ms = (uint32_t)zy100_os_time_ms();

    (void)reason;
    if (s_battery_policy.critical_locked ||
        app_battery_policy_port_runtime_shutdown_active())
    {
        battery_adc_guard_on_capture_stop(now_ms, "arm_skip");
        return;
    }

    battery_adc_guard_on_capture_start(now_ms);
    s_battery_policy.adc_capture_active = true;
    s_battery_policy.adc_attempt_pending = false;
    app_battery_policy_guard_bas_arm(now_ms);
#else
    (void)reason;
#endif
}

static void app_task_battery_adc_guard_disarm_capture(const char *reason)
{
#if F_APP_BATTERY_ADC_GUARD_ENABLE
    uint32_t now_ms = (uint32_t)zy100_os_time_ms();

    battery_adc_guard_on_capture_stop(now_ms, reason);
    s_battery_policy.adc_capture_active = false;
    s_battery_policy.adc_attempt_pending = false;
    if (s_battery_policy.adc_mode_valid &&
        (s_battery_policy.adc_mode != APP_BATTERY_ADC_MODE_FULL_SLEEP))
    {
        s_battery_policy.adc_next_due_ms =
            now_ms + app_battery_policy_adc_period_ms(
                         s_battery_policy.adc_mode);
    }
    app_battery_policy_guard_bas_disarm(reason);
#else
    (void)reason;
#endif
}

static bool app_task_battery_adc_guard_export_pressure(void)
{
    return app_battery_policy_port_export_pressure();
}

static battery_adc_guard_block_reason_t app_task_battery_adc_guard_final_edge_block(void)
{
#if V0_IMU_FIFO_DRAIN_TEST && V1_IMU_FLASH_CAPTURE_ENABLE && \
    ZY100_FINAL_EDGE_MODE_ENABLE
    zy100_fe_capture_state_t fe_state =
        imu_fifo_drain_test_final_edge_capture_state();

    switch (fe_state)
    {
    case FE_CAPTURE_STATE_RUNNING:
        return BATTERY_ADC_GUARD_BLOCK_NONE;
    case FE_CAPTURE_STATE_DRAIN_B_ACTIVE:
        return BATTERY_ADC_GUARD_BLOCK_B_CRITICAL;
    case FE_CAPTURE_STATE_DRAIN_REPLAY:
        return BATTERY_ADC_GUARD_BLOCK_REPLAY;
    case FE_CAPTURE_STATE_DRAIN_LIVE_FIFO:
        return BATTERY_ADC_GUARD_BLOCK_FIFO;
    case FE_CAPTURE_STATE_DRAIN_RAW_STORE:
        return BATTERY_ADC_GUARD_BLOCK_RAW;
    case FE_CAPTURE_STATE_DRAIN_RECORD_STORE:
        return BATTERY_ADC_GUARD_BLOCK_RECORD;
    case FE_CAPTURE_STATE_WRITE_META:
        return BATTERY_ADC_GUARD_BLOCK_FLASH;
    case FE_CAPTURE_STATE_EXPORT_READY:
    case FE_CAPTURE_STATE_EXPORTING:
    case FE_CAPTURE_STATE_EXPORT_DONE:
    case FE_CAPTURE_STATE_EXPORT_CLEANUP:
        return BATTERY_ADC_GUARD_BLOCK_BLE;
    case FE_CAPTURE_STATE_STOP_REQUESTED:
    case FE_CAPTURE_STATE_ERROR:
        return BATTERY_ADC_GUARD_BLOCK_STATE;
    case FE_CAPTURE_STATE_IDLE:
    default:
        return BATTERY_ADC_GUARD_BLOCK_STATE;
    }
#elif V0_IMU_FIFO_DRAIN_TEST && V1_IMU_FLASH_CAPTURE_ENABLE
    /* Online Direct / Offline V2 do not run the legacy Final Edge FSM. */
    return BATTERY_ADC_GUARD_BLOCK_NONE;
#else
    return BATTERY_ADC_GUARD_BLOCK_STATE;
#endif
}

static bool app_task_battery_adc_guard_capture_running(void)
{
#if ZY100_OFFLINE_FEATURE_V2_ENABLE
    if (zy100_offline_v2_capture_input_active())
    {
        return true;
    }
#endif
    return imu_fifo_drain_test_is_active();
}

static battery_adc_guard_block_reason_t app_task_battery_adc_guard_block_reason(void)
{
#if V0_IMU_FIFO_DRAIN_TEST && V1_IMU_FLASH_CAPTURE_ENABLE
    battery_adc_guard_block_reason_t fe_block;

    if (s_battery_policy.critical_locked ||
        app_battery_policy_port_runtime_shutdown_active() ||
        app_battery_policy_port_sleep_transition_active())
    {
        return BATTERY_ADC_GUARD_BLOCK_STATE;
    }

    if (!app_battery_policy_port_capture_state_active() ||
        !app_battery_policy_port_capture_active())
    {
        return BATTERY_ADC_GUARD_BLOCK_STATE;
    }

    fe_block = app_task_battery_adc_guard_final_edge_block();
    if (fe_block == BATTERY_ADC_GUARD_BLOCK_B_CRITICAL)
    {
        return fe_block;
    }

    if (app_battery_policy_port_capture_stop_in_progress())
    {
        return BATTERY_ADC_GUARD_BLOCK_STATE;
    }

    if (app_task_battery_adc_guard_export_pressure())
    {
        return BATTERY_ADC_GUARD_BLOCK_BLE;
    }

    if (fe_block != BATTERY_ADC_GUARD_BLOCK_NONE)
    {
        return fe_block;
    }

    if (imu_fifo_drain_test_is_start_pending())
    {
        return BATTERY_ADC_GUARD_BLOCK_SPI;
    }

    if (!app_task_battery_adc_guard_capture_running())
    {
        return BATTERY_ADC_GUARD_BLOCK_STATE;
    }

    /* P2_7 ADC owns no shared SPI resource. Run one preemptible shot in
     * the App task; the higher-priority IMU worker can service FIFO normally.
     * Do not depend on its transient, single-tick idle-window token. */
    return BATTERY_ADC_GUARD_BLOCK_NONE;
#else
    return BATTERY_ADC_GUARD_BLOCK_STATE;
#endif
}

static bool app_battery_policy_adc_poll(uint32_t runtime_ms)
{
#if F_APP_BATTERY_ADC_GUARD_ENABLE
    battery_adc_guard_block_reason_t block_reason =
        app_task_battery_adc_guard_block_reason();
    bool sampled = battery_adc_guard_poll(runtime_ms, block_reason);

    return (block_reason == BATTERY_ADC_GUARD_BLOCK_NONE) && !sampled;
#else
    (void)runtime_ms;
    return false;
#endif
}

void app_task_battery_adc_guard_low_event_handle(uint32_t runtime_ms,
                                                 uint16_t mv,
                                                 uint8_t percent)
{
#if F_APP_BATTERY_ADC_GUARD_ENABLE
    app_battery_critical_event_t event;
    bool newly_critical = !s_battery_policy.critical_locked;

    event.battery_mv = mv;
    event.percent = percent;
    if (newly_critical)
    {
        (void)app_battery_policy_bas_notify_now(percent, runtime_ms);
    }
    s_battery_policy.critical_locked = true;
    (void)app_task_battery_critical_request_stop_if_needed_with_reason(
        &event,
        "battery_adc_guard_capture",
        "battery_adc_guard_capture_stopping");
#else
    (void)runtime_ms;
    (void)mv;
    (void)percent;
#endif
}

void app_task_battery_adc_guard_fault_event_handle(uint32_t runtime_ms)
{
#if F_APP_BATTERY_ADC_GUARD_ENABLE
    app_battery_policy_port_request_adc_fault_stop();
    (void)runtime_ms;
#else
    (void)runtime_ms;
#endif
}

void app_battery_policy_handle_level(uint8_t percent, uint32_t runtime_ms)
{
    if (percent <= (uint8_t)APP_BATTERY_CRITICAL_PERCENT)
    {
        app_battery_critical_event_t event;
        bool newly_critical = !s_battery_policy.critical_locked;

        event.percent = percent;
        event.battery_mv = APP_BATTERY_CRITICAL_MV_UNKNOWN;
#if F_APP_BATTERY_ADC_ENABLE && F_APP_BATTERY_ADC_GUARD_ENABLE
        if (s_battery_policy.guard_bas_valid)
        {
            event.battery_mv = s_battery_policy.guard_bas_mv;
        }
#endif
        if (newly_critical)
        {
            (void)app_battery_policy_bas_notify_now(percent, runtime_ms);
        }
        s_battery_policy.critical_locked = true;
        if (app_task_battery_critical_request_stop_if_needed(&event))
        {
            return;
        }
        app_battery_policy_critical_led_tick(runtime_ms);
        return;
    }

    if (s_battery_policy.critical_locked &&
        !app_battery_policy_port_runtime_shutdown_active() &&
        (percent >= (uint8_t)APP_BATTERY_RECOVER_PERCENT))
    {
        s_battery_policy.critical_locked = false;
#if F_APP_BATTERY_ADC_ENABLE
        /* The terminal charge UI owns the finite prompt, including its end. */
        if (!s_battery_policy.adc_mode_valid ||
            s_battery_policy.adc_mode != APP_BATTERY_ADC_MODE_CHARGE_PARTIAL_SLEEP)
#endif
            app_battery_policy_critical_led_stop();
        app_battery_policy_port_recovered();
    }
}

void app_task_battery_low_lpc_event_handle(uint32_t runtime_ms)
{
#if F_APP_BATTERY_LPC_GUARD_ENABLE
    bool latched;
    bool armed_before;
    bool active_before;
    bool stopping_before;
    bool capture_state;
    bool stop_result = false;

    armed_before = battery_low_guard_is_armed();
    capture_state = app_battery_policy_port_capture_state_active();
    active_before = capture_state && app_battery_policy_port_capture_active();
    stopping_before = app_battery_policy_port_capture_stop_in_progress();
    latched = battery_low_guard_take_latched();
    if (latched && (active_before || stopping_before))
    {
        app_battery_critical_event_t event;

        event.battery_mv = APP_BATTERY_CRITICAL_MV_UNKNOWN;
        event.percent = APP_BATTERY_CRITICAL_PERCENT_UNKNOWN;
        s_battery_policy.critical_locked = true;
        app_battery_policy_port_runtime_shutdown_mark("battery_low_lpc_capture");
        stop_result =
            app_task_battery_critical_request_stop_if_needed_with_reason(
                &event,
                "battery_low_lpc_capture",
                "battery_low_lpc_capture_stopping");
    }
    else if (latched)
    {
        battery_low_guard_on_capture_stop(runtime_ms, "lpc_event_not_capture");
    }

    DBG_DIRECT("[BAT_LPC] event runtime_ms=%lu state=%u capture_state=%u active=%u stopping=%u armed_before=%u latched=%u stop=%u locked=%u",
               (unsigned long)runtime_ms,
               capture_state ? 1U : 0U,
               capture_state ? 1U : 0U,
               active_before ? 1U : 0U,
               stopping_before ? 1U : 0U,
               armed_before ? 1U : 0U,
               latched ? 1U : 0U,
               stop_result ? 1U : 0U,
               s_battery_policy.critical_locked ? 1U : 0U);
#else
    (void)runtime_ms;
#endif
}
#endif

#if !F_APP_BATTERY_ADC_ENABLE
bool app_battery_policy_adc_set_mode(app_battery_adc_mode_t mode,
                                     uint32_t runtime_ms)
{
    (void)mode;
    (void)runtime_ms;
    return false;
}

bool app_battery_policy_adc_work_due(app_battery_adc_mode_t mode,
                                     uint32_t runtime_ms)
{
    (void)mode;
    (void)runtime_ms;
    return false;
}

bool app_battery_policy_adc_runtime_poll(app_battery_adc_mode_t mode,
                                         uint32_t runtime_ms,
                                         bool sample_allowed,
                                         bool resume_before_start)
{
    (void)mode;
    (void)runtime_ms;
    (void)sample_allowed;
    (void)resume_before_start;
    return false;
}

bool app_battery_policy_adc_sample_once(uint32_t runtime_ms,
                                        uint8_t *percent_out)
{
    (void)runtime_ms;
    if (percent_out != NULL)
    {
        *percent_out = 0U;
    }
    return false;
}

bool app_battery_policy_adc_commit_current(uint32_t runtime_ms)
{
    (void)runtime_ms;
    return false;
}

bool app_battery_policy_adc_commit_bas_current(uint32_t runtime_ms)
{
    (void)runtime_ms;
    return false;
}

void app_battery_policy_adc_force_due(uint32_t runtime_ms)
{
    (void)runtime_ms;
}

void app_battery_policy_adc_cancel(uint32_t runtime_ms)
{
    (void)runtime_ms;
}

bool app_battery_policy_adc_attempt_pending(void)
{
    return false;
}

bool app_battery_policy_prepare_capture(const char *reason)
{
    (void)reason;
    return true;
}

void app_battery_policy_arm_capture(const char *reason)
{
    (void)reason;
}

void app_battery_policy_disarm_capture(const char *reason)
{
    (void)reason;
}

static void app_battery_policy_lpc_poll(uint32_t runtime_ms)
{
    (void)runtime_ms;
}

static void app_task_battery_adc_guard_arm_capture(const char *reason)
{
    (void)reason;
}

static void app_task_battery_adc_guard_disarm_capture(const char *reason)
{
    (void)reason;
}

static bool app_battery_policy_adc_poll(uint32_t runtime_ms)
{
    (void)runtime_ms;
    return false;
}
#endif

void app_battery_policy_init(void)
{
    s_battery_policy.critical_locked = false;
    s_battery_policy.critical_red_blink_last_ms = 0ULL;
#if F_APP_BATTERY_ADC_ENABLE
    s_battery_policy.adc_mode_valid = false;
    s_battery_policy.adc_attempt_pending = false;
    s_battery_policy.adc_capture_active = false;
    s_battery_policy.adc_mode = APP_BATTERY_ADC_MODE_ACTIVE;
    s_battery_policy.adc_next_due_ms = 0U;
    s_battery_policy.bas_reference_valid = false;
    s_battery_policy.bas_reference_percent = 0U;
    s_battery_policy.bas_notify_pending = false;
    s_battery_policy.bas_notify_percent = 0U;
    s_battery_policy.bas_next_notify_ms = 0U;
#endif
#if F_APP_BATTERY_LPC_GUARD_ENABLE
    s_battery_policy.lpc_arm_pending = false;
    s_battery_policy.lpc_arm_due_ms = 0U;
#endif
#if F_APP_BATTERY_ADC_ENABLE && F_APP_BATTERY_ADC_GUARD_ENABLE
    s_battery_policy.guard_bas_active = false;
    s_battery_policy.guard_bas_valid = false;
#endif
}

void app_battery_policy_disarm_adc_guard(const char *reason)
{
    app_task_battery_adc_guard_disarm_capture(reason);
}

void app_battery_policy_poll(uint32_t runtime_ms)
{
    bool guard_idle_window;
#if F_APP_BATTERY_ADC_ENABLE
    bool bas_idle_window;
#endif

    app_battery_policy_lpc_poll(runtime_ms);
    guard_idle_window = app_battery_policy_adc_poll(runtime_ms);
#if F_APP_BATTERY_ADC_ENABLE && F_APP_BATTERY_ADC_GUARD_ENABLE
    app_battery_policy_guard_bas_poll(runtime_ms, guard_idle_window);
#else
    (void)guard_idle_window;
#endif
#if F_APP_BATTERY_ADC_ENABLE
    bas_idle_window =
        !app_battery_policy_port_runtime_shutdown_active() &&
        !app_battery_policy_port_sleep_transition_active() &&
        !app_battery_policy_port_export_pressure();
#if F_APP_BATTERY_ADC_GUARD_ENABLE
    if (s_battery_policy.guard_bas_active)
    {
        bas_idle_window = bas_idle_window && guard_idle_window;
    }
#endif
    app_battery_policy_bas_poll(runtime_ms, bas_idle_window);
#endif
}

void app_battery_policy_mark_critical(const char *reason)
{
    (void)reason;
    s_battery_policy.critical_locked = true;
}

bool app_battery_policy_is_critical_locked(void)
{
    return s_battery_policy.critical_locked;
}

