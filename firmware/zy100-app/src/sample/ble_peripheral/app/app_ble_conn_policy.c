#include "app/app_ble_conn_policy.h"
#include "app/app_ble_conn_param_mgr.h"

#include <stddef.h>
#include <string.h>

#include <gap.h>
#include <gap_conn_le.h>
#include <os_sched.h>
#include <trace.h>

#include "app/app_ble_export_perf.h"
#include "app/app_ble_ci_state.h"
#include "app_flags.h"

#if ZY100_BUILD_PRODUCTION
#undef DBG_DIRECT
#define DBG_DIRECT(...) ZY100_BLE_DIAG_LOG(__VA_ARGS__)
#endif
#include "peripheral_app.h"
#include "service/zy100_online_stream.h"

#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
#define APP_BLE_EXPORT_FAST_CI_START_NONE 0U
#define APP_BLE_EXPORT_FAST_CI_START_EXACT 1U
#define APP_BLE_EXPORT_FAST_CI_START_PARTIAL 2U
#define APP_BLE_EXPORT_FAST_CI_START_FALLBACK 3U
#define APP_BLE_EXPORT_FAST_CI_START_NOWAIT 4U

typedef struct
{
    bool active;
    bool begin_started;
    bool wait_started;
    bool request_pending;
    bool wait_before_stream;
    bool high_speed_confirmed;
    bool restore_needed;
    bool work_event_pending;
    bool device_requested_fast_ci;
    bool high_request_closed;

    uint8_t conn_id;
    uint8_t start_mode;

    uint16_t normal_interval;
    uint16_t normal_latency;
    uint16_t normal_timeout;

    uint16_t last_interval;
    uint16_t last_latency;
    uint16_t last_timeout;

    uint32_t begin_os_ms;
    uint32_t wait_start_os_ms;
    uint32_t wait_end_os_ms;
    uint32_t request_os_ms;
    uint32_t deadline_os_ms;
    uint32_t next_retry_os_ms;
    uint32_t partial_high_elapsed_ms;
    uint32_t late_high_elapsed_ms;

    uint8_t retry_count;
    uint32_t request_count;
    uint32_t request_fail_count;
    uint32_t update_ok_count;
    uint32_t update_fail_count;
    uint32_t restore_request_count;
    uint32_t restore_skip_count;
    uint32_t high_cancel_count;

    uint16_t last_cause;
    uint16_t partial_high_ci;
    uint16_t late_high_ci;
    uint8_t late_high_mode;
    bool timeout;
} app_ble_export_fast_conn_t;

#if ZY100_BLE_CAPTURE_RUNTIME_CONN_PARAM_ENABLE
typedef struct
{
    bool active;
    bool request_pending;
    bool high_timeout_logged;

    uint8_t conn_id;
    uint8_t retry_count;

    uint16_t last_interval;
    uint16_t last_latency;
    uint16_t last_timeout;
    uint16_t last_cause;

    uint32_t begin_os_ms;
    uint32_t request_os_ms;
    uint32_t next_retry_os_ms;
} app_ble_runtime_conn_t;
#endif
#endif


#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
static app_ble_export_fast_conn_t s_ble_export_fast_conn;
#if ZY100_BLE_CAPTURE_RUNTIME_CONN_PARAM_ENABLE
static app_ble_runtime_conn_t s_ble_runtime_conn;
#endif
#endif

#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
#define APP_BLE_EXPORT_FAST_CONN_TIMEOUT_FALLBACK 0x03C0U
#define APP_BLE_EXPORT_FAST_CONN_RESTORE_INTERVAL_FALLBACK \
    ((uint16_t)ZY100_BLE_RUNTIME_CONN_INTERVAL_MIN)

static uint32_t app_ble_export_fast_conn_now_ms(void)
{
    return (uint32_t)os_sys_time_get();
}

static bool app_ble_export_fast_conn_due(uint32_t now_ms, uint32_t due_ms)
{
    return (due_ms == 0U) || (((int32_t)(now_ms - due_ms)) >= 0);
}

static uint16_t app_ble_conn_interval_clamp(uint16_t ci)
{
    if (ci < 0x0006U)
    {
        return 0x0006U;
    }
    if (ci > 0x0C80U)
    {
        return 0x0C80U;
    }
    return ci;
}

static uint16_t app_ble_export_target_ci_min(void)
{
    return app_ble_conn_interval_clamp(
        (uint16_t)ZY100_BLE_EXPORT_FAST_CI_MIN);
}

static uint16_t app_ble_export_target_ci_max(void)
{
    uint16_t target_min = app_ble_export_target_ci_min();
    uint16_t target_max;

    target_max = app_ble_conn_interval_clamp(
        (uint16_t)ZY100_BLE_EXPORT_FAST_CI_MAX);

    return (target_max < target_min) ? target_min : target_max;
}

static bool app_ble_export_fast_conn_reached(uint16_t interval,
                                             uint16_t latency)
{
    uint16_t target_min = app_ble_export_target_ci_min();
    uint16_t target_max = app_ble_export_target_ci_max();

    return (interval != 0U) &&
           (interval >= target_min) &&
           (interval <= target_max) &&
           (latency == (uint16_t)ZY100_BLE_EXPORT_FAST_LATENCY);
}

static uint16_t app_ble_export_partial_ci_max(void)
{
    uint16_t partial_max = app_ble_conn_interval_clamp(
                               (uint16_t)ZY100_BLE_EXPORT_FAST_PARTIAL_CI_MAX);
    uint16_t target_min = app_ble_export_target_ci_min();

    return (partial_max < target_min) ? target_min : partial_max;
}

static bool app_ble_export_fast_conn_partial_ready(uint16_t interval,
                                                   uint16_t latency)
{
    if (app_ble_export_partial_ci_max() == app_ble_export_target_ci_min())
    {
        return false;
    }

    return (interval != 0U) &&
           (interval <= app_ble_export_partial_ci_max()) &&
           (latency == (uint16_t)ZY100_BLE_EXPORT_FAST_LATENCY);
}

static const char *app_ble_export_fast_conn_start_mode_str(uint8_t mode)
{
    switch (mode)
    {
    case APP_BLE_EXPORT_FAST_CI_START_EXACT:
        return "exact";
    case APP_BLE_EXPORT_FAST_CI_START_PARTIAL:
        return "partial";
    case APP_BLE_EXPORT_FAST_CI_START_FALLBACK:
        return "fallback";
    case APP_BLE_EXPORT_FAST_CI_START_NOWAIT:
        return "nowait";
    default:
        return "none";
    }
}

static bool app_ble_export_conn_param_mgr_high_ready(uint8_t conn_id)
{
    return app_ble_conn_param_mgr_high_ready(
               conn_id,
               app_ble_export_target_ci_min(),
               app_ble_export_target_ci_max(),
               (uint16_t)ZY100_BLE_EXPORT_FAST_LATENCY);
}

static bool app_ble_export_conn_param_mgr_high_failed(uint8_t conn_id)
{
    return app_ble_conn_param_mgr_high_failed(
               conn_id,
               app_ble_export_target_ci_min(),
               app_ble_export_target_ci_max(),
               (uint16_t)ZY100_BLE_EXPORT_FAST_LATENCY);
}

#if ZY100_BLE_CAPTURE_RUNTIME_CONN_PARAM_ENABLE
static void app_ble_runtime_conn_clear(void)
{
    memset(&s_ble_runtime_conn, 0, sizeof(s_ble_runtime_conn));
}

static void app_ble_runtime_conn_snapshot(uint8_t conn_id)
{
    uint16_t interval = 0U;
    uint16_t latency = 0U;
    uint16_t timeout = 0U;

    if (le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &interval, conn_id) ==
        GAP_CAUSE_SUCCESS)
    {
        s_ble_runtime_conn.last_interval = interval;
    }
    if (le_get_conn_param(GAP_PARAM_CONN_LATENCY, &latency, conn_id) ==
        GAP_CAUSE_SUCCESS)
    {
        s_ble_runtime_conn.last_latency = latency;
    }
    if (le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &timeout, conn_id) ==
        GAP_CAUSE_SUCCESS)
    {
        s_ble_runtime_conn.last_timeout = timeout;
    }
}

static bool app_ble_runtime_conn_quiet(void)
{
    return app_ble_conn_policy_port_online_quiet();
}

static void app_ble_runtime_conn_end(const char *reason)
{
    if (!s_ble_runtime_conn.active)
    {
        return;
    }

    if (!app_ble_runtime_conn_quiet())
    {
        DBG_DIRECT("[BLE_RUN_CI] end c=%u ci=%u lat=%u tout=%u rt=%u why=%s",
                   s_ble_runtime_conn.conn_id,
                   s_ble_runtime_conn.last_interval,
                   s_ble_runtime_conn.last_latency,
                   s_ble_runtime_conn.last_timeout,
                   s_ble_runtime_conn.retry_count,
                   (reason != NULL) ? reason : "done");
    }
    app_ble_runtime_conn_clear();
}

static bool app_ble_runtime_conn_reached(uint16_t interval,
                                         uint16_t latency)
{
    return (interval != 0U) &&
           (interval >= (uint16_t)ZY100_BLE_RUNTIME_CONN_INTERVAL_MIN) &&
           (interval <= (uint16_t)ZY100_BLE_RUNTIME_CONN_INTERVAL_MAX) &&
           (latency == (uint16_t)ZY100_BLE_RUNTIME_CONN_LATENCY);
}

static bool app_ble_runtime_conn_mgr_high_ready(uint8_t conn_id)
{
    return app_ble_conn_param_mgr_high_ready(
               conn_id,
               (uint16_t)ZY100_BLE_RUNTIME_CONN_INTERVAL_MIN,
               (uint16_t)ZY100_BLE_RUNTIME_CONN_INTERVAL_MAX,
               (uint16_t)ZY100_BLE_RUNTIME_CONN_LATENCY);
}

static bool app_ble_runtime_conn_mgr_high_failed(uint8_t conn_id)
{
    return app_ble_conn_param_mgr_high_failed(
               conn_id,
               (uint16_t)ZY100_BLE_RUNTIME_CONN_INTERVAL_MIN,
               (uint16_t)ZY100_BLE_RUNTIME_CONN_INTERVAL_MAX,
               (uint16_t)ZY100_BLE_RUNTIME_CONN_LATENCY);
}

static bool app_ble_runtime_conn_request_now(const char *reason)
{
    uint8_t conn_id = s_ble_runtime_conn.conn_id;
    app_ble_conn_param_request_result_t result;
    uint32_t now_ms;

    if (!app_ble_power_conn_valid(conn_id))
    {
        app_ble_runtime_conn_end("conn_invalid");
        return false;
    }

    app_ble_runtime_conn_snapshot(conn_id);
    if (app_ble_runtime_conn_reached(s_ble_runtime_conn.last_interval,
                                     s_ble_runtime_conn.last_latency))
    {
        if (!app_ble_runtime_conn_quiet())
        {
            DBG_DIRECT("[BLE_RUN_CI] already_runtime c=%u ci=%u lat=%u why=%s",
                       conn_id,
                       s_ble_runtime_conn.last_interval,
                       s_ble_runtime_conn.last_latency,
                       (reason != NULL) ? reason : "runtime");
        }
        app_ble_runtime_conn_end("already_runtime");
        return true;
    }

    now_ms = app_ble_export_fast_conn_now_ms();
    result = (app_ble_conn_param_request_result_t)
             app_ble_ci_state_request(APP_BLE_CI_STATE_CAPTURE_RUNTIME,
                                      reason);
    if (result == APP_BLE_CONN_PARAM_REQ_ALREADY_REACHED)
    {
        app_ble_runtime_conn_snapshot(conn_id);
        app_ble_runtime_conn_end("runtime_reached");
        return true;
    }

    if (result == APP_BLE_CONN_PARAM_REQ_SUBMITTED)
    {
        if (s_ble_runtime_conn.retry_count < 0xFFU)
        {
            s_ble_runtime_conn.retry_count++;
        }
    }

    s_ble_runtime_conn.request_os_ms = now_ms;
    s_ble_runtime_conn.last_cause = (uint16_t)result;
    s_ble_runtime_conn.request_pending =
        (result == APP_BLE_CONN_PARAM_REQ_SUBMITTED) ||
        (result == APP_BLE_CONN_PARAM_REQ_WAIT_SAME_TARGET);
    s_ble_runtime_conn.next_retry_os_ms =
        now_ms + (uint32_t)ZY100_BLE_CONN_PARAM_RETRY_COOLDOWN_MS;

    if (!app_ble_runtime_conn_quiet())
    {
        DBG_DIRECT("[BLE_RUN_CI] req c=%u ci=%u/%u lat=%u tout=%u rc=%u rt=%u why=%s",
                   conn_id,
                   (uint16_t)ZY100_BLE_RUNTIME_CONN_INTERVAL_MIN,
                   (uint16_t)ZY100_BLE_RUNTIME_CONN_INTERVAL_MAX,
                   (uint16_t)ZY100_BLE_RUNTIME_CONN_LATENCY,
                   (uint16_t)ZY100_BLE_RUNTIME_CONN_TIMEOUT,
                   result,
                   s_ble_runtime_conn.retry_count,
                   (reason != NULL) ? reason : "runtime");
    }
    return (result == APP_BLE_CONN_PARAM_REQ_SUBMITTED) ||
           (result == APP_BLE_CONN_PARAM_REQ_WAIT_SAME_TARGET) ||
           (result == APP_BLE_CONN_PARAM_REQ_BUSY);
}

void app_ble_runtime_conn_begin(const char *reason)
{
    uint8_t conn_id;

#if ZY100_ONLINE_STREAM_ENABLE
    if (zy100_online_stream_active())
    {
        return;
    }
#endif
    if (!app_ble_power_is_connected())
    {
        return;
    }

    conn_id = app_ble_power_conn_id();
    if (!app_ble_power_conn_valid(conn_id))
    {
        return;
    }

    app_ble_runtime_conn_clear();
    s_ble_runtime_conn.active = true;
    s_ble_runtime_conn.conn_id = conn_id;
    s_ble_runtime_conn.begin_os_ms = app_ble_export_fast_conn_now_ms();
    app_ble_runtime_conn_snapshot(conn_id);
    if (!app_ble_runtime_conn_quiet())
    {
        DBG_DIRECT("[BLE_RUN_CI] begin c=%u ci=%u lat=%u tout=%u ci_to=%u/%u why=%s",
                   conn_id,
                   s_ble_runtime_conn.last_interval,
                   s_ble_runtime_conn.last_latency,
                   s_ble_runtime_conn.last_timeout,
                   (uint16_t)ZY100_BLE_RUNTIME_CONN_INTERVAL_MIN,
                   (uint16_t)ZY100_BLE_RUNTIME_CONN_INTERVAL_MAX,
                   (reason != NULL) ? reason : "runtime");
    }

    (void)app_ble_runtime_conn_request_now(reason);
}

void app_ble_runtime_conn_maintain(void)
{
    uint32_t now_ms;

    if (!s_ble_runtime_conn.active)
    {
        return;
    }

    now_ms = app_ble_export_fast_conn_now_ms();

    if (!app_ble_conn_policy_port_capture_active())
    {
        app_ble_runtime_conn_end("capture_inactive");
        return;
    }

    if (!app_ble_power_conn_valid(s_ble_runtime_conn.conn_id))
    {
        app_ble_runtime_conn_end("conn_invalid");
        return;
    }

    app_ble_runtime_conn_snapshot(s_ble_runtime_conn.conn_id);
    if (app_ble_runtime_conn_mgr_high_ready(s_ble_runtime_conn.conn_id) ||
        app_ble_runtime_conn_reached(s_ble_runtime_conn.last_interval,
                                     s_ble_runtime_conn.last_latency))
    {
        app_ble_runtime_conn_end("runtime_reached");
        return;
    }

    if (app_ble_runtime_conn_mgr_high_failed(s_ble_runtime_conn.conn_id) ||
        (((uint32_t)(now_ms - s_ble_runtime_conn.begin_os_ms)) >=
         (uint32_t)ZY100_BLE_CONN_PARAM_TOTAL_TIMEOUT_MS))
    {
        if (!s_ble_runtime_conn.high_timeout_logged &&
            !app_ble_runtime_conn_quiet())
        {
            DBG_DIRECT("[BLE_RUN_CI] high_timeout c=%u ci=%u lat=%u rt=%u ms=%lu req_ms=%lu now_ms=%lu fb=1",
                       s_ble_runtime_conn.conn_id,
                       s_ble_runtime_conn.last_interval,
                       s_ble_runtime_conn.last_latency,
                       s_ble_runtime_conn.retry_count,
                       (unsigned long)(now_ms - s_ble_runtime_conn.begin_os_ms),
                       (unsigned long)s_ble_runtime_conn.request_os_ms,
                       (unsigned long)now_ms);
            s_ble_runtime_conn.high_timeout_logged = true;
        }
        app_ble_runtime_conn_end("high_timeout");
        return;
    }

    if (s_ble_runtime_conn.request_pending)
    {
        return;
    }

    if ((s_ble_runtime_conn.retry_count >=
         (uint8_t)ZY100_BLE_EXPORT_FAST_RETRY_MAX) ||
        !app_ble_export_fast_conn_due(now_ms,
                                      s_ble_runtime_conn.next_retry_os_ms))
    {
        return;
    }

    (void)app_ble_runtime_conn_request_now("retry");
}

static void app_ble_runtime_conn_on_update(uint8_t conn_id,
                                           uint16_t interval,
                                           uint16_t latency,
                                           uint16_t timeout,
                                           uint16_t cause)
{
    bool success = (cause == GAP_SUCCESS) ||
                   (cause == (uint16_t)GAP_CAUSE_SUCCESS);

    if (!s_ble_runtime_conn.active ||
        (s_ble_runtime_conn.conn_id != conn_id))
    {
        return;
    }

    if (interval != 0U)
    {
        s_ble_runtime_conn.last_interval = interval;
    }
    s_ble_runtime_conn.last_latency = latency;
    if (timeout != 0U)
    {
        s_ble_runtime_conn.last_timeout = timeout;
    }
    s_ble_runtime_conn.last_cause = cause;
    s_ble_runtime_conn.request_pending = false;

    if (!app_ble_runtime_conn_quiet())
    {
        DBG_DIRECT("[BLE_RUN_CI] update c=%u success=%u ci=%u lat=%u tout=%u cause=0x%x",
                   conn_id,
                   success ? 1U : 0U,
                   s_ble_runtime_conn.last_interval,
                   s_ble_runtime_conn.last_latency,
                   s_ble_runtime_conn.last_timeout,
                   cause);
    }

    if (success &&
        app_ble_runtime_conn_reached(s_ble_runtime_conn.last_interval,
                                     s_ble_runtime_conn.last_latency))
    {
        app_ble_runtime_conn_end("runtime_reached");
        return;
    }

    s_ble_runtime_conn.next_retry_os_ms =
        app_ble_export_fast_conn_now_ms() +
        (uint32_t)ZY100_BLE_EXPORT_FAST_RETRY_COOLDOWN_MS;
}
#endif

static void app_ble_export_post_conn_param_work_event(void)
{
    if (s_ble_export_fast_conn.work_event_pending)
    {
        return;
    }

    if (app_ble_conn_policy_port_post_work_event())
    {
        s_ble_export_fast_conn.work_event_pending = true;
    }
}

static void app_ble_export_fast_conn_mark_wait_done(uint32_t now_ms)
{
    if (s_ble_export_fast_conn.wait_before_stream)
    {
        s_ble_export_fast_conn.wait_end_os_ms = now_ms;
    }
    s_ble_export_fast_conn.wait_before_stream = false;
}

static void app_ble_export_fast_conn_cancel_high(const char *reason)
{
    if (s_ble_export_fast_conn.high_request_closed)
    {
        return;
    }
    app_ble_conn_param_mgr_cancel_owner(APP_BLE_CONN_PARAM_OWNER_EXPORT_HIGH,
                                        s_ble_export_fast_conn.conn_id,
                                        reason);
    s_ble_export_fast_conn.request_pending = false;
    s_ble_export_fast_conn.high_request_closed = true;
    s_ble_export_fast_conn.next_retry_os_ms = 0U;
    s_ble_export_fast_conn.high_cancel_count++;
}

static uint32_t app_ble_export_fast_conn_wait_elapsed_ms(uint32_t now_ms)
{
    if (s_ble_export_fast_conn.wait_start_os_ms == 0U)
    {
        return 0U;
    }
    return (uint32_t)(now_ms - s_ble_export_fast_conn.wait_start_os_ms);
}

static void app_ble_export_fast_conn_start_exact(uint32_t now_ms,
                                                 const char *reason)
{
    (void)reason;
    s_ble_export_fast_conn.high_speed_confirmed = true;
    s_ble_export_fast_conn.start_mode = APP_BLE_EXPORT_FAST_CI_START_EXACT;
    app_ble_export_fast_conn_mark_wait_done(now_ms);
}

static void app_ble_export_fast_conn_start_nowait(uint32_t now_ms,
                                                  const char *reason)
{
    uint32_t elapsed_ms;

    if (!s_ble_export_fast_conn.wait_started)
    {
        s_ble_export_fast_conn.wait_started = true;
        s_ble_export_fast_conn.wait_start_os_ms = now_ms;
    }
    s_ble_export_fast_conn.start_mode = APP_BLE_EXPORT_FAST_CI_START_NOWAIT;
    s_ble_export_fast_conn.wait_end_os_ms = now_ms;
    app_ble_export_fast_conn_mark_wait_done(now_ms);
    elapsed_ms = app_ble_export_fast_conn_wait_elapsed_ms(now_ms);

    DBG_DIRECT("[BLE_EXP_CI] stream_start_nowait c=%u ci=%u lat=%u ci_to=%u/%u ms=%lu why=%s",
               s_ble_export_fast_conn.conn_id,
               s_ble_export_fast_conn.last_interval,
               s_ble_export_fast_conn.last_latency,
               app_ble_export_target_ci_min(),
               app_ble_export_target_ci_max(),
               (unsigned long)elapsed_ms,
               (reason != NULL) ? reason : "nowait");
}

static void app_ble_export_fast_conn_start_partial(uint32_t now_ms,
                                                   const char *reason)
{
    uint32_t elapsed_ms = app_ble_export_fast_conn_wait_elapsed_ms(now_ms);

    s_ble_export_fast_conn.start_mode = APP_BLE_EXPORT_FAST_CI_START_PARTIAL;
    s_ble_export_fast_conn.partial_high_ci =
        s_ble_export_fast_conn.last_interval;
    s_ble_export_fast_conn.partial_high_elapsed_ms = elapsed_ms;
    app_ble_export_fast_conn_mark_wait_done(now_ms);

    DBG_DIRECT("[BLE_EXP_CI] partial_high_start c=%u ci=%u lat=%u ci_to=%u/%u partial_max=%u ms=%lu why=%s req=%lu rt=%u",
               s_ble_export_fast_conn.conn_id,
               s_ble_export_fast_conn.last_interval,
               s_ble_export_fast_conn.last_latency,
               app_ble_export_target_ci_min(),
               app_ble_export_target_ci_max(),
               app_ble_export_partial_ci_max(),
               (unsigned long)elapsed_ms,
               (reason != NULL) ? reason : "partial",
               (unsigned long)s_ble_export_fast_conn.request_count,
               s_ble_export_fast_conn.retry_count);
}

static void app_ble_export_fast_conn_start_fallback_keep_high(uint32_t now_ms,
                                                              const char *reason)
{
    uint32_t elapsed_ms = app_ble_export_fast_conn_wait_elapsed_ms(now_ms);

    s_ble_export_fast_conn.start_mode = APP_BLE_EXPORT_FAST_CI_START_FALLBACK;
    app_ble_export_fast_conn_mark_wait_done(now_ms);

    DBG_DIRECT("[BLE_EXP_CI] fallback_stream_start_keep_high c=%u ci=%u lat=%u tout=%u ms=%lu why=%s req=%lu rt=%u",
               s_ble_export_fast_conn.conn_id,
               s_ble_export_fast_conn.last_interval,
               s_ble_export_fast_conn.last_latency,
               s_ble_export_fast_conn.last_timeout,
               (unsigned long)elapsed_ms,
               (reason != NULL) ? reason : "stream_wait",
               (unsigned long)s_ble_export_fast_conn.request_count,
               s_ble_export_fast_conn.retry_count);
}

static void app_ble_export_fast_conn_note_late_high(uint32_t now_ms,
                                                    uint8_t mode,
                                                    const char *reason)
{
    uint32_t elapsed_ms = app_ble_export_fast_conn_wait_elapsed_ms(now_ms);

    if ((mode == APP_BLE_EXPORT_FAST_CI_START_EXACT) &&
        !s_ble_export_fast_conn.high_speed_confirmed)
    {
        s_ble_export_fast_conn.high_speed_confirmed = true;
    }

    if ((s_ble_export_fast_conn.start_mode !=
         APP_BLE_EXPORT_FAST_CI_START_FALLBACK) &&
        (s_ble_export_fast_conn.start_mode !=
         APP_BLE_EXPORT_FAST_CI_START_NOWAIT))
    {
        return;
    }

    if ((s_ble_export_fast_conn.late_high_mode ==
         APP_BLE_EXPORT_FAST_CI_START_EXACT) ||
        ((s_ble_export_fast_conn.late_high_mode ==
          APP_BLE_EXPORT_FAST_CI_START_PARTIAL) &&
         (mode != APP_BLE_EXPORT_FAST_CI_START_EXACT)))
    {
        return;
    }

    if ((s_ble_export_fast_conn.late_high_mode ==
         APP_BLE_EXPORT_FAST_CI_START_NONE) ||
        (mode == APP_BLE_EXPORT_FAST_CI_START_EXACT))
    {
        s_ble_export_fast_conn.late_high_mode = mode;
        s_ble_export_fast_conn.late_high_ci = s_ble_export_fast_conn.last_interval;
        s_ble_export_fast_conn.late_high_elapsed_ms = elapsed_ms;
    }

    DBG_DIRECT("[BLE_EXP_CI] late_high_reached c=%u ci=%u lat=%u ci_to=%u/%u mode=%s ms=%lu why=%s",
               s_ble_export_fast_conn.conn_id,
               s_ble_export_fast_conn.last_interval,
               s_ble_export_fast_conn.last_latency,
               app_ble_export_target_ci_min(),
               app_ble_export_target_ci_max(),
               app_ble_export_fast_conn_start_mode_str(mode),
               (unsigned long)elapsed_ms,
               (reason != NULL) ? reason : "late_high");
}

static void app_ble_export_fast_conn_note_late_from_last(uint32_t now_ms,
                                                         const char *reason)
{
    if (s_ble_export_fast_conn.wait_before_stream ||
        ((s_ble_export_fast_conn.start_mode !=
          APP_BLE_EXPORT_FAST_CI_START_FALLBACK) &&
         (s_ble_export_fast_conn.start_mode !=
          APP_BLE_EXPORT_FAST_CI_START_NOWAIT)))
    {
        return;
    }

    if (app_ble_export_fast_conn_reached(s_ble_export_fast_conn.last_interval,
                                         s_ble_export_fast_conn.last_latency))
    {
        app_ble_export_fast_conn_note_late_high(
            now_ms,
            APP_BLE_EXPORT_FAST_CI_START_EXACT,
            reason);
        return;
    }

    if (app_ble_export_fast_conn_partial_ready(
            s_ble_export_fast_conn.last_interval,
            s_ble_export_fast_conn.last_latency))
    {
        app_ble_export_fast_conn_note_late_high(
            now_ms,
            APP_BLE_EXPORT_FAST_CI_START_PARTIAL,
            reason);
    }
}

static bool app_ble_export_fast_conn_cancel_bg_high_if_due(uint32_t now_ms,
                                                           const char *reason)
{
    uint32_t elapsed_ms;
    bool high_failed;
    bool deadline_due;
    const char *use_reason;

    if (!s_ble_export_fast_conn.active ||
        s_ble_export_fast_conn.high_request_closed ||
        s_ble_export_fast_conn.high_speed_confirmed ||
        (s_ble_export_fast_conn.start_mode ==
         APP_BLE_EXPORT_FAST_CI_START_NOWAIT))
    {
        return false;
    }

    high_failed = app_ble_export_conn_param_mgr_high_failed(
                      s_ble_export_fast_conn.conn_id);
    deadline_due = app_ble_export_fast_conn_due(
                       now_ms,
                       s_ble_export_fast_conn.deadline_os_ms);
    if (!high_failed && !deadline_due)
    {
        return false;
    }

    elapsed_ms = app_ble_export_fast_conn_wait_elapsed_ms(now_ms);
    use_reason = (reason != NULL) ? reason :
                 (high_failed ? "high_failed" : "bg_high_timeout");
    s_ble_export_fast_conn.timeout = true;
    DBG_DIRECT("[BLE_EXP_CI] bg_high_timeout c=%u ci=%u lat=%u ms=%lu why=%s req=%lu rt=%u",
               s_ble_export_fast_conn.conn_id,
               s_ble_export_fast_conn.last_interval,
               s_ble_export_fast_conn.last_latency,
               (unsigned long)elapsed_ms,
               use_reason,
               (unsigned long)s_ble_export_fast_conn.request_count,
               s_ble_export_fast_conn.retry_count);
    app_ble_export_fast_conn_cancel_high(use_reason);
    return true;
}

static bool app_ble_export_fast_conn_request(bool restore)
{
    uint32_t now_ms = app_ble_export_fast_conn_now_ms();
    uint16_t interval_min;
    uint16_t interval_max;
    uint16_t latency;
    uint16_t timeout;
    app_ble_conn_param_request_result_t result;

    if (!app_ble_power_conn_valid(s_ble_export_fast_conn.conn_id))
    {
        if (restore)
        {
            s_ble_export_fast_conn.restore_needed = false;
        }
        return false;
    }

    if (restore)
    {
        app_ble_export_fast_conn_cancel_high("restore_req");
        interval_min = (uint16_t)ZY100_BLE_ACTIVE_IDLE_CONN_INTERVAL;
        interval_max = interval_min;
        latency = (uint16_t)ZY100_BLE_ACTIVE_IDLE_CONN_LATENCY;
        timeout = (uint16_t)ZY100_BLE_ACTIVE_IDLE_CONN_TIMEOUT;
        if ((s_ble_export_fast_conn.last_interval == interval_min) &&
            (s_ble_export_fast_conn.last_latency == latency) &&
            (s_ble_export_fast_conn.last_timeout == timeout))
        {
            s_ble_export_fast_conn.restore_needed = false;
            s_ble_export_fast_conn.restore_skip_count++;
            return true;
        }
        s_ble_export_fast_conn.restore_request_count++;
    }
    else
    {
        if (s_ble_export_fast_conn.high_request_closed)
        {
            return true;
        }
        interval_min = app_ble_export_target_ci_min();
        interval_max = app_ble_export_target_ci_max();
        latency = (uint16_t)ZY100_BLE_EXPORT_FAST_LATENCY;
        timeout = (uint16_t)ZY100_BLE_RUNTIME_CONN_TIMEOUT;
        s_ble_export_fast_conn.request_count++;
    }

    result = (app_ble_conn_param_request_result_t)
             app_ble_ci_state_request(
                 restore ? APP_BLE_CI_STATE_ACTIVE_IDLE :
                 APP_BLE_CI_STATE_ONLINE_HIGH,
                 restore ? "export_restore_idle" : "export_high");
    s_ble_export_fast_conn.request_os_ms = now_ms;
    s_ble_export_fast_conn.last_cause = (uint16_t)result;
    if (!restore && (result == APP_BLE_CONN_PARAM_REQ_SUBMITTED) &&
        (s_ble_export_fast_conn.retry_count < 0xFFU))
    {
        s_ble_export_fast_conn.retry_count++;
    }
    DBG_DIRECT("[BLE_EXP_CI] %s_req c=%u ci=%u/%u lat=%u tout=%u rc=%u rt=%u",
               restore ? "restore" : "normal",
               s_ble_export_fast_conn.conn_id,
               interval_min,
               interval_max,
               latency,
               timeout,
               result,
               s_ble_export_fast_conn.retry_count);

    if (restore)
    {
        s_ble_export_fast_conn.restore_needed = false;
        return (result == APP_BLE_CONN_PARAM_REQ_SUBMITTED) ||
               (result == APP_BLE_CONN_PARAM_REQ_WAIT_SAME_TARGET) ||
               (result == APP_BLE_CONN_PARAM_REQ_ALREADY_REACHED) ||
               (result == APP_BLE_CONN_PARAM_REQ_BUSY);
    }

    if (result == APP_BLE_CONN_PARAM_REQ_ALREADY_REACHED)
    {
        s_ble_export_fast_conn.request_pending = false;
        s_ble_export_fast_conn.device_requested_fast_ci = true;
        if (s_ble_export_fast_conn.wait_before_stream)
        {
            app_ble_export_fast_conn_start_exact(now_ms, "request_already");
        }
        else
        {
            app_ble_export_fast_conn_note_late_high(
                now_ms,
                APP_BLE_EXPORT_FAST_CI_START_EXACT,
                "request_already");
        }
        return true;
    }

    if ((result == APP_BLE_CONN_PARAM_REQ_SUBMITTED) ||
        (result == APP_BLE_CONN_PARAM_REQ_WAIT_SAME_TARGET) ||
        (result == APP_BLE_CONN_PARAM_REQ_BUSY))
    {
        s_ble_export_fast_conn.request_pending = true;
        s_ble_export_fast_conn.device_requested_fast_ci = true;
        return true;
    }

    s_ble_export_fast_conn.request_fail_count++;
    s_ble_export_fast_conn.request_pending = false;
    app_ble_export_fast_conn_mark_wait_done(now_ms);
    s_ble_export_fast_conn.next_retry_os_ms =
        now_ms + (uint32_t)ZY100_BLE_EXPORT_FAST_RETRY_COOLDOWN_MS;
    app_ble_conn_policy_port_wake_ctrl();
    return false;
}

void app_ble_export_fast_conn_begin(uint8_t conn_id)
{
    uint32_t now_ms = app_ble_export_fast_conn_now_ms();
    uint16_t last_interval;
    uint16_t last_latency;
    uint16_t last_timeout;

    if (s_ble_export_fast_conn.active)
    {
        if (s_ble_export_fast_conn.conn_id != conn_id)
        {
            app_ble_export_fast_conn_end(false);
        }
        else
        {
            return;
        }
    }

    last_interval = s_ble_export_fast_conn.last_interval;
    last_latency = s_ble_export_fast_conn.last_latency;
    last_timeout = s_ble_export_fast_conn.last_timeout;
    if (app_ble_power_conn_valid(conn_id))
    {
        uint16_t actual_interval = 0U;
        uint16_t actual_latency = 0U;
        uint16_t actual_timeout = 0U;

        if (le_get_conn_param(GAP_PARAM_CONN_INTERVAL,
                              &actual_interval,
                              conn_id) == GAP_CAUSE_SUCCESS)
        {
            last_interval = actual_interval;
        }
        if (le_get_conn_param(GAP_PARAM_CONN_LATENCY,
                              &actual_latency,
                              conn_id) == GAP_CAUSE_SUCCESS)
        {
            last_latency = actual_latency;
        }
        if (le_get_conn_param(GAP_PARAM_CONN_TIMEOUT,
                              &actual_timeout,
                              conn_id) == GAP_CAUSE_SUCCESS)
        {
            last_timeout = actual_timeout;
        }
    }
    memset(&s_ble_export_fast_conn, 0, sizeof(s_ble_export_fast_conn));
    app_ble_conn_param_mgr_clear_high_cap(conn_id);
    app_ble_conn_param_mgr_cancel_owner(APP_BLE_CONN_PARAM_OWNER_EXPORT_HIGH,
                                        conn_id,
                                        "export_begin");
    s_ble_export_fast_conn.last_interval = last_interval;
    s_ble_export_fast_conn.last_latency = last_latency;
    s_ble_export_fast_conn.last_timeout = last_timeout;
    s_ble_export_fast_conn.active = true;
    s_ble_export_fast_conn.begin_started = true;
    s_ble_export_fast_conn.conn_id = conn_id;
    s_ble_export_fast_conn.begin_os_ms = now_ms;
    s_ble_export_fast_conn.normal_interval = last_interval;
    s_ble_export_fast_conn.normal_latency = last_latency;
    s_ble_export_fast_conn.normal_timeout = last_timeout;
    DBG_DIRECT("[BLE_EXP_CI] begin c=%u normal=%u lat=%u tout=%u ci_to=%u/%u mode=export_11p25_nowait",
               conn_id,
               last_interval,
               last_latency,
               last_timeout,
               app_ble_export_target_ci_min(),
               app_ble_export_target_ci_max());

    if (app_ble_export_fast_conn_reached(last_interval, last_latency))
    {
        s_ble_export_fast_conn.high_speed_confirmed = true;
        s_ble_export_fast_conn.start_mode =
            APP_BLE_EXPORT_FAST_CI_START_EXACT;
        DBG_DIRECT("[BLE_EXP_CI] already_fast c=%u ci=%u lat=%u",
                   conn_id,
                   last_interval,
                   last_latency);
    }

    s_ble_export_fast_conn.deadline_os_ms =
        now_ms + (uint32_t)ZY100_BLE_EXPORT_FAST_WAIT_TOTAL_TIMEOUT_MS;
    app_ble_export_fast_conn_start_nowait(now_ms, "begin");
    (void)app_ble_export_fast_conn_request(false);
}

bool app_ble_export_fast_conn_ready_or_timeout(void)
{
    uint32_t now_ms = app_ble_export_fast_conn_now_ms();
    uint16_t actual_ci = 0U;
    uint16_t actual_latency = 0U;
    uint16_t actual_timeout = 0U;

    if (!s_ble_export_fast_conn.active ||
        !s_ble_export_fast_conn.wait_before_stream)
    {
        return true;
    }

    if (app_ble_power_conn_valid(s_ble_export_fast_conn.conn_id))
    {
        if (le_get_conn_param(GAP_PARAM_CONN_INTERVAL,
                              &actual_ci,
                              s_ble_export_fast_conn.conn_id) ==
            GAP_CAUSE_SUCCESS)
        {
            s_ble_export_fast_conn.last_interval = actual_ci;
        }
        if (le_get_conn_param(GAP_PARAM_CONN_LATENCY,
                              &actual_latency,
                              s_ble_export_fast_conn.conn_id) ==
            GAP_CAUSE_SUCCESS)
        {
            s_ble_export_fast_conn.last_latency = actual_latency;
        }
        if (le_get_conn_param(GAP_PARAM_CONN_TIMEOUT,
                              &actual_timeout,
                              s_ble_export_fast_conn.conn_id) ==
            GAP_CAUSE_SUCCESS)
        {
            s_ble_export_fast_conn.last_timeout = actual_timeout;
        }
    }

    if (s_ble_export_fast_conn.high_speed_confirmed ||
        app_ble_export_fast_conn_reached(s_ble_export_fast_conn.last_interval,
                                         s_ble_export_fast_conn.last_latency))
    {
        app_ble_export_fast_conn_start_exact(now_ms, "poll_exact");
        return true;
    }

    if (app_ble_export_fast_conn_partial_ready(
            s_ble_export_fast_conn.last_interval,
            s_ble_export_fast_conn.last_latency))
    {
        app_ble_export_fast_conn_start_partial(now_ms, "poll_partial");
        return true;
    }

    if (!s_ble_export_fast_conn.wait_started)
    {
        s_ble_export_fast_conn.wait_started = true;
        s_ble_export_fast_conn.wait_start_os_ms = now_ms;
        s_ble_export_fast_conn.deadline_os_ms =
            now_ms + (uint32_t)ZY100_BLE_EXPORT_FAST_WAIT_TOTAL_TIMEOUT_MS;
    }

    {
        bool high_failed = app_ble_export_conn_param_mgr_high_failed(
                               s_ble_export_fast_conn.conn_id);
        bool stream_wait_due = app_ble_export_fast_conn_due(
                                   now_ms,
                                   s_ble_export_fast_conn.wait_start_os_ms +
                                   (uint32_t)ZY100_BLE_EXPORT_FAST_STREAM_WAIT_MS);
        bool total_deadline_due = app_ble_export_fast_conn_due(
                                      now_ms,
                                      s_ble_export_fast_conn.deadline_os_ms);

        if (high_failed || total_deadline_due)
        {
            uint32_t elapsed_ms =
                app_ble_export_fast_conn_wait_elapsed_ms(now_ms);

            s_ble_export_fast_conn.timeout = true;
            s_ble_export_fast_conn.start_mode =
                APP_BLE_EXPORT_FAST_CI_START_FALLBACK;
            app_ble_export_fast_conn_mark_wait_done(now_ms);
            DBG_DIRECT("[BLE_EXP_CI] fallback_low_speed c=%u ci=%u lat=%u tout=%u req=%lu rt=%u ms=%lu req_ms=%lu now_ms=%lu",
                       s_ble_export_fast_conn.conn_id,
                       s_ble_export_fast_conn.last_interval,
                       s_ble_export_fast_conn.last_latency,
                       s_ble_export_fast_conn.last_timeout,
                       (unsigned long)s_ble_export_fast_conn.request_count,
                       s_ble_export_fast_conn.retry_count,
                       (unsigned long)elapsed_ms,
                       (unsigned long)s_ble_export_fast_conn.request_os_ms,
                       (unsigned long)now_ms);
            app_ble_export_fast_conn_cancel_high(
                high_failed ? "fallback_low_speed" : "bg_high_timeout");
            return true;
        }

        if (stream_wait_due)
        {
            app_ble_export_fast_conn_start_fallback_keep_high(
                now_ms,
                "stream_wait");
            return true;
        }
    }

    if (!s_ble_export_fast_conn.request_pending &&
        app_ble_export_fast_conn_due(now_ms,
                                     s_ble_export_fast_conn.next_retry_os_ms))
    {
        app_ble_export_post_conn_param_work_event();
    }
    return false;
}

void app_ble_export_fast_conn_maintain(uint32_t now_os_ms)
{
    if (s_ble_export_fast_conn.restore_needed)
    {
        if (!s_ble_export_fast_conn.work_event_pending)
        {
            app_ble_export_post_conn_param_work_event();
        }
        return;
    }

    if (!s_ble_export_fast_conn.active ||
        !app_ble_export_is_active() ||
        (s_ble_export_fast_conn.conn_id != app_ble_export_perf_port_conn_id()) ||
        !app_ble_power_conn_valid(s_ble_export_fast_conn.conn_id) ||
        s_ble_export_fast_conn.work_event_pending)
    {
        return;
    }

    if (s_ble_export_fast_conn.wait_before_stream &&
        app_ble_export_fast_conn_ready_or_timeout())
    {
        app_ble_conn_policy_port_wake_ctrl();
        return;
    }

    if (!s_ble_export_fast_conn.wait_before_stream)
    {
        app_ble_export_fast_conn_note_late_from_last(now_os_ms,
                                                     "maintain_high");
        if (app_ble_export_fast_conn_cancel_bg_high_if_due(now_os_ms,
                                                           NULL))
        {
            return;
        }
    }

    if (s_ble_export_fast_conn.request_pending)
    {
        if (!s_ble_export_fast_conn.wait_before_stream)
        {
            return;
        }
        if ((app_ble_export_conn_param_mgr_high_ready(
                 s_ble_export_fast_conn.conn_id) ||
             app_ble_export_conn_param_mgr_high_failed(
                 s_ble_export_fast_conn.conn_id)) &&
            s_ble_export_fast_conn.wait_before_stream)
        {
            app_ble_export_post_conn_param_work_event();
        }
        return;
    }

    if (s_ble_export_fast_conn.high_request_closed)
    {
        return;
    }

    if (app_ble_export_fast_conn_reached(s_ble_export_fast_conn.last_interval,
                                         s_ble_export_fast_conn.last_latency) ||
        (s_ble_export_fast_conn.retry_count >=
         (uint8_t)ZY100_BLE_EXPORT_FAST_RETRY_MAX) ||
        !app_ble_export_fast_conn_due(now_os_ms,
                                      s_ble_export_fast_conn.next_retry_os_ms))
    {
        return;
    }

    app_ble_export_post_conn_param_work_event();
}

void app_ble_export_fast_conn_end(bool restore)
{
    bool was_active = s_ble_export_fast_conn.active;
    bool connected = false;
    bool device_requested_fast_ci = s_ble_export_fast_conn.device_requested_fast_ci;

#if ZY100_BLE_EXPORT_RESTORE_CONN_PARAM_ENABLE
    if (restore && was_active)
    {
        connected = app_ble_power_conn_valid(s_ble_export_fast_conn.conn_id);
    }
#endif

    s_ble_export_fast_conn.active = false;
    s_ble_export_fast_conn.request_pending = false;
    s_ble_export_fast_conn.wait_before_stream = false;
    s_ble_export_fast_conn.work_event_pending = false;
    if (was_active)
    {
        app_ble_export_fast_conn_cancel_high(
            restore ? "export_end_restore" : "export_end");
    }
    if (was_active)
    {
        ZY100_DIAG_LOG("[BLE_EXP_CI] end restore=%u connected=%u req=%u c=%u ci=%u lat=%u",
                   restore ? 1U : 0U,
                   connected ? 1U : 0U,
                   device_requested_fast_ci ? 1U : 0U,
                   s_ble_export_fast_conn.conn_id,
                   s_ble_export_fast_conn.last_interval,
                   s_ble_export_fast_conn.last_latency);
    }

    if (restore && was_active && connected && device_requested_fast_ci)
    {
        s_ble_export_fast_conn.restore_needed = true;
        app_ble_export_post_conn_param_work_event();
    }
    else
    {
        if (restore && was_active && connected && !device_requested_fast_ci)
        {
            s_ble_export_fast_conn.restore_skip_count++;
        }
        s_ble_export_fast_conn.restore_needed = false;
    }
}

bool app_ble_export_fast_conn_matches(uint8_t conn_id)
{
    return s_ble_export_fast_conn.conn_id == conn_id;
}

void app_ble_export_conn_param_work(void) __attribute__((unused));
void app_ble_export_conn_param_work(void)
{
    uint32_t now_ms = app_ble_export_fast_conn_now_ms();

    s_ble_export_fast_conn.work_event_pending = false;

    if (s_ble_export_fast_conn.restore_needed)
    {
        if (s_ble_export_fast_conn.active ||
            !app_ble_power_conn_valid(s_ble_export_fast_conn.conn_id))
        {
            if (!app_ble_power_conn_valid(s_ble_export_fast_conn.conn_id))
            {
                s_ble_export_fast_conn.restore_needed = false;
            }
            return;
        }
        (void)app_ble_export_fast_conn_request(true);
        return;
    }

    if (!s_ble_export_fast_conn.active ||
        !app_ble_export_is_active() ||
        (s_ble_export_fast_conn.conn_id != app_ble_export_perf_port_conn_id()) ||
        !app_ble_power_conn_valid(s_ble_export_fast_conn.conn_id))
    {
        return;
    }

    if (s_ble_export_fast_conn.request_pending)
    {
        if (!s_ble_export_fast_conn.wait_before_stream)
        {
            app_ble_export_fast_conn_note_late_from_last(now_ms,
                                                         "work_pending_high");
            (void)app_ble_export_fast_conn_cancel_bg_high_if_due(
                      now_ms,
                      NULL);
            return;
        }
        if (app_ble_export_fast_conn_ready_or_timeout())
        {
            app_ble_conn_policy_port_wake_ctrl();
        }
        return;
    }

    if (s_ble_export_fast_conn.wait_before_stream &&
        app_ble_export_fast_conn_ready_or_timeout())
    {
        app_ble_conn_policy_port_wake_ctrl();
        return;
    }

    if (!s_ble_export_fast_conn.wait_before_stream)
    {
        app_ble_export_fast_conn_note_late_from_last(now_ms,
                                                     "work_high");
        if (app_ble_export_fast_conn_cancel_bg_high_if_due(now_ms,
                                                           NULL))
        {
            return;
        }
    }

    if (s_ble_export_fast_conn.high_request_closed)
    {
        return;
    }

    if (app_ble_export_fast_conn_reached(s_ble_export_fast_conn.last_interval,
                                         s_ble_export_fast_conn.last_latency))
    {
        if (s_ble_export_fast_conn.wait_before_stream)
        {
            app_ble_export_fast_conn_start_exact(now_ms, "work_exact");
            app_ble_conn_policy_port_wake_ctrl();
        }
        return;
    }

    if ((s_ble_export_fast_conn.retry_count >=
         (uint8_t)ZY100_BLE_EXPORT_FAST_RETRY_MAX) ||
        !app_ble_export_fast_conn_due(now_ms,
                                      s_ble_export_fast_conn.next_retry_os_ms))
    {
        return;
    }

    (void)app_ble_export_fast_conn_request(false);
}

void app_ble_export_on_conn_param_snapshot(uint8_t conn_id,
                                           uint16_t interval,
                                           uint16_t latency,
                                           uint16_t timeout)
{
    uint32_t now_ms = app_ble_export_fast_conn_now_ms();

    s_ble_export_fast_conn.last_interval = interval;
    s_ble_export_fast_conn.last_latency = latency;
    s_ble_export_fast_conn.last_timeout = timeout;

    if (!s_ble_export_fast_conn.active ||
        (s_ble_export_fast_conn.conn_id != conn_id))
    {
        return;
    }

    if (app_ble_export_fast_conn_reached(interval, latency))
    {
        if (s_ble_export_fast_conn.wait_before_stream)
        {
            app_ble_export_fast_conn_start_exact(now_ms, "snapshot_exact");
            app_ble_conn_policy_port_wake_ctrl();
        }
        else
        {
            app_ble_export_fast_conn_note_late_high(
                now_ms,
                APP_BLE_EXPORT_FAST_CI_START_EXACT,
                "snapshot_exact");
        }
        return;
    }

    if (app_ble_export_fast_conn_partial_ready(interval, latency))
    {
        if (s_ble_export_fast_conn.wait_before_stream)
        {
            app_ble_export_fast_conn_start_partial(now_ms,
                                                   "snapshot_partial");
            app_ble_conn_policy_port_wake_ctrl();
        }
        else
        {
            app_ble_export_fast_conn_note_late_high(
                now_ms,
                APP_BLE_EXPORT_FAST_CI_START_PARTIAL,
                "snapshot_partial");
        }
        return;
    }

    if (!s_ble_export_fast_conn.wait_before_stream)
    {
        return;
    }

    s_ble_export_fast_conn.next_retry_os_ms =
        now_ms + (uint32_t)ZY100_BLE_EXPORT_FAST_RETRY_COOLDOWN_MS;
    app_ble_export_post_conn_param_work_event();
}

void app_ble_export_on_conn_param_update(uint8_t conn_id,
                                         uint16_t interval,
                                         uint16_t latency,
                                         uint16_t timeout,
                                         uint16_t cause)
{
    uint32_t now_ms = app_ble_export_fast_conn_now_ms();
    bool success = (cause == GAP_SUCCESS) ||
                   (cause == (uint16_t)GAP_CAUSE_SUCCESS);
    bool no_new_params = !success && (interval == 0U) && (timeout == 0U);

    if (interval == 0U)
    {
        interval = s_ble_export_fast_conn.last_interval;
    }
    if (no_new_params)
    {
        latency = s_ble_export_fast_conn.last_latency;
    }
    if (timeout == 0U)
    {
        timeout = s_ble_export_fast_conn.last_timeout;
    }

    s_ble_export_fast_conn.last_interval = interval;
    s_ble_export_fast_conn.last_latency = latency;
    s_ble_export_fast_conn.last_timeout = timeout;
    s_ble_export_fast_conn.last_cause = cause;

    if (success)
    {
        s_ble_export_fast_conn.update_ok_count++;
    }
    else
    {
        s_ble_export_fast_conn.update_fail_count++;
    }

#if ZY100_BLE_CAPTURE_RUNTIME_CONN_PARAM_ENABLE
    app_ble_runtime_conn_on_update(conn_id,
                                   interval,
                                   latency,
                                   timeout,
                                   cause);
#endif

    if (s_ble_export_fast_conn.active &&
        (s_ble_export_fast_conn.conn_id == conn_id))
    {
        DBG_DIRECT("[BLE_EXP_CI] update c=%u success=%u ci=%u lat=%u tout=%u cause=0x%x",
                   conn_id,
                   success ? 1U : 0U,
                   interval,
                   latency,
                   timeout,
                   cause);
    }

    if (!s_ble_export_fast_conn.active ||
        (s_ble_export_fast_conn.conn_id != conn_id))
    {
        return;
    }

    s_ble_export_fast_conn.request_pending = false;

    if (success &&
        app_ble_export_fast_conn_reached(interval, latency))
    {
        if (s_ble_export_fast_conn.wait_before_stream)
        {
            app_ble_export_fast_conn_start_exact(now_ms, "update_exact");
            app_ble_conn_policy_port_wake_ctrl();
        }
        else
        {
            app_ble_export_fast_conn_note_late_high(
                now_ms,
                APP_BLE_EXPORT_FAST_CI_START_EXACT,
                "update_exact");
        }
        return;
    }

    if (success &&
        app_ble_export_fast_conn_partial_ready(interval, latency))
    {
        s_ble_export_fast_conn.next_retry_os_ms =
            now_ms + (uint32_t)ZY100_BLE_EXPORT_FAST_RETRY_COOLDOWN_MS;
        if (s_ble_export_fast_conn.wait_before_stream)
        {
            app_ble_export_fast_conn_start_partial(now_ms,
                                                   "update_partial");
            app_ble_conn_policy_port_wake_ctrl();
        }
        else
        {
            app_ble_export_fast_conn_note_late_high(
                now_ms,
                APP_BLE_EXPORT_FAST_CI_START_PARTIAL,
                "update_partial");
        }
        return;
    }

    s_ble_export_fast_conn.next_retry_os_ms =
        now_ms + (uint32_t)ZY100_BLE_EXPORT_FAST_RETRY_COOLDOWN_MS;
    if (!s_ble_export_fast_conn.wait_before_stream)
    {
        return;
    }
    if (!success &&
        (app_ble_export_conn_param_mgr_high_failed(conn_id) ||
         app_ble_export_fast_conn_ready_or_timeout()))
    {
        app_ble_conn_policy_port_wake_ctrl();
        return;
    }

    if (s_ble_export_fast_conn.retry_count <
        (uint8_t)ZY100_BLE_EXPORT_FAST_RETRY_MAX)
    {
        app_ble_export_post_conn_param_work_event();
    }
}

#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
void app_ble_export_fast_conn_p0_log_summary(void)
{
    uint32_t wait_ms = 0U;
    uint16_t target_min = app_ble_export_target_ci_min();
    uint16_t target_max = app_ble_export_target_ci_max();
    uint16_t actual_ci = s_ble_export_fast_conn.last_interval;
    uint16_t actual_latency = s_ble_export_fast_conn.last_latency;
    uint8_t reached;

    if (s_ble_export_fast_conn.wait_start_os_ms != 0U)
    {
        uint32_t wait_end = (s_ble_export_fast_conn.wait_end_os_ms != 0U) ?
                            s_ble_export_fast_conn.wait_end_os_ms :
                            app_ble_export_fast_conn_now_ms();

        wait_ms = (uint32_t)(wait_end -
                             s_ble_export_fast_conn.wait_start_os_ms);
    }
    reached = app_ble_export_fast_conn_reached(actual_ci,
                                              actual_latency) ? 1U : 0U;

    DBG_DIRECT("[BLE_P0] ci_mode=export_11p25_nowait");
    DBG_DIRECT("[BLE_P0] ci_target_min=0x%04x", target_min);
    DBG_DIRECT("[BLE_P0] ci_target_max=0x%04x", target_max);
    DBG_DIRECT("[BLE_P0] ci_actual=0x%04x", actual_ci);
    DBG_DIRECT("[BLE_P0] ci_reached=%u", reached);
    DBG_DIRECT("[BLE_P0] fast_ci min=0x%04x max=0x%04x",
               target_min,
               target_max);
    DBG_DIRECT("[BLE_P0] fast_ci normal=0x%04x last=0x%04x",
               s_ble_export_fast_conn.normal_interval,
               s_ble_export_fast_conn.last_interval);
    DBG_DIRECT("[BLE_P0] fast_ci req=%lu rt=%lu",
               (unsigned long)s_ble_export_fast_conn.request_count,
               (unsigned long)s_ble_export_fast_conn.retry_count);
    DBG_DIRECT("[BLE_P0] fast_ci ok=%lu fail=%lu",
               (unsigned long)s_ble_export_fast_conn.update_ok_count,
               (unsigned long)(s_ble_export_fast_conn.request_fail_count +
                               s_ble_export_fast_conn.update_fail_count));
    DBG_DIRECT("[BLE_P0] fast_ci tout=%u high=%u",
               s_ble_export_fast_conn.timeout ? 1U : 0U,
               s_ble_export_fast_conn.high_speed_confirmed ? 1U : 0U);
    DBG_DIRECT("[BLE_P0] fast_ci start_mode=%s high_start_ci=0x%04x high_ms=%lu",
               app_ble_export_fast_conn_start_mode_str(
                   s_ble_export_fast_conn.start_mode),
               s_ble_export_fast_conn.partial_high_ci,
               (unsigned long)s_ble_export_fast_conn.partial_high_elapsed_ms);
    DBG_DIRECT("[BLE_P0] fast_ci late_mode=%s late_high_ci=0x%04x late_ms=%lu",
               app_ble_export_fast_conn_start_mode_str(
                   s_ble_export_fast_conn.late_high_mode),
               s_ble_export_fast_conn.late_high_ci,
               (unsigned long)s_ble_export_fast_conn.late_high_elapsed_ms);
    DBG_DIRECT("[BLE_P0] fast_ci restore_req=%lu cause=0x%04x",
               (unsigned long)s_ble_export_fast_conn.restore_request_count,
               s_ble_export_fast_conn.last_cause);
    DBG_DIRECT("[BLE_P0] fast_ci restore_skip=%lu",
               (unsigned long)s_ble_export_fast_conn.restore_skip_count);
    DBG_DIRECT("[BLE_P0] fast_ci high_cancel=%lu",
               (unsigned long)s_ble_export_fast_conn.high_cancel_count);
    DBG_DIRECT("[BLE_P0] fast_ci wait_ms=%lu", (unsigned long)wait_ms);
    DBG_DIRECT("[BLE_P0] fast_ci reached=%u", reached);
}
#endif
#endif
