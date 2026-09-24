#include "app_ble_conn_param_mgr.h"
#include "app_ble_conn_param_mgr_port.h"

#include <stddef.h>
#include <string.h>

#include <gap.h>
#include <gap_conn_le.h>
#include <os_sched.h>
#include <trace.h>

#include "../app_flags.h"

#if ZY100_PRODUCT_LOG_QUIET_ENABLE
#undef DBG_DIRECT
#define DBG_DIRECT(...) ZY100_BLE_DIAG_LOG(__VA_ARGS__)
#endif

#if F_LP_TEST_DISABLE_APP_LOG || !ZY100_LOG_BLE_STACK_INFO_ENABLE
#undef APP_PRINT_INFO0
#undef APP_PRINT_INFO1
#undef APP_PRINT_INFO2
#undef APP_PRINT_INFO3
#undef APP_PRINT_INFO4
#undef APP_PRINT_INFO5
#undef APP_PRINT_TRACE1
#define APP_PRINT_INFO0(...)
#define APP_PRINT_INFO1(...)
#define APP_PRINT_INFO2(...)
#define APP_PRINT_INFO3(...)
#define APP_PRINT_INFO4(...)
#define APP_PRINT_INFO5(...)
#define APP_PRINT_TRACE1(...)
#endif

#define APP_BLE_CONN_PARAM_CONN_ID_INVALID 0xFFU
static uint16_t app_ble_standby_ce_len(uint16_t interval)
{
    uint32_t ce_len;

    if (interval == 0U)
    {
        return 0U;
    }
    ce_len = 2UL * ((uint32_t)interval - 1UL);
    return (ce_len > 0xFFFFUL) ? 0xFFFFU : (uint16_t)ce_len;
}

#define APP_BLE_CONN_PARAM_OWNER_HIGH_MASK \
    ((uint16_t)APP_BLE_CONN_PARAM_OWNER_RUNTIME_HIGH | \
     (uint16_t)APP_BLE_CONN_PARAM_OWNER_EXPORT_HIGH)

typedef struct
{
    bool valid;
    app_ble_conn_param_owner_t primary_owner;
    uint16_t owner_mask;
    uint8_t conn_id;
    uint16_t ci_min;
    uint16_t ci_max;
    uint16_t latency;
    uint16_t timeout;
    uint8_t retry;
    uint32_t begin_os_ms;
    uint32_t due_os_ms;
    const char *reason;
} app_ble_conn_param_mgr_next_t;

typedef struct
{
    bool active;
    bool gap_pending_seen;
    bool long_wait_logged;
    app_ble_conn_param_owner_t primary_owner;
    uint16_t owner_mask;
    uint8_t conn_id;
    uint16_t ci_min;
    uint16_t ci_max;
    uint16_t latency;
    uint16_t timeout;
    uint8_t retry;
    uint32_t begin_os_ms;
    uint32_t request_os_ms;
    uint32_t next_wait_log_os_ms;
    uint16_t actual_ci;
    uint16_t actual_latency;
    uint16_t actual_timeout;
    const char *reason;
} app_ble_conn_param_mgr_active_t;

typedef struct
{
    app_ble_conn_param_mgr_active_t active;
    app_ble_conn_param_mgr_next_t next;
    uint8_t cap_conn_id;
    uint16_t cap_ci_min;
    uint16_t cap_ci_max;
    uint16_t cap_latency;
    bool high_supported;
    bool high_failed;
} app_ble_conn_param_mgr_t;

static app_ble_conn_param_mgr_t s_ble_conn_param_mgr = {
    {0},
    {0},
    APP_BLE_CONN_PARAM_CONN_ID_INVALID,
    0U,
    0U,
    0U,
    false,
    false
};

static uint32_t app_ble_conn_param_mgr_now_ms(void)
{
    return (uint32_t)os_sys_time_get();
}

static bool app_ble_conn_param_mgr_due(uint32_t now_ms, uint32_t due_ms)
{
    return (due_ms == 0U) || (((int32_t)(now_ms - due_ms)) >= 0);
}

static bool app_ble_conn_param_mgr_owner_is_high(uint16_t owner_mask)
{
    return ((owner_mask & APP_BLE_CONN_PARAM_OWNER_HIGH_MASK) != 0U);
}

static const char *app_ble_conn_param_mgr_owner_str(app_ble_conn_param_owner_t owner)
{
    switch (owner)
    {
    case APP_BLE_CONN_PARAM_OWNER_STANDBY_LOW_POWER:
        return "standby_low";
    case APP_BLE_CONN_PARAM_OWNER_STANDBY_RESTORE:
        return "standby_restore";
    case APP_BLE_CONN_PARAM_OWNER_RUNTIME_HIGH:
        return "runtime_high";
    case APP_BLE_CONN_PARAM_OWNER_EXPORT_HIGH:
        return "export_high";
    case APP_BLE_CONN_PARAM_OWNER_EXPORT_RESTORE:
        return "export_restore";
    case APP_BLE_CONN_PARAM_OWNER_CI_STATE:
        return "ci_state";
    default:
        return "unknown";
    }
}

static uint8_t app_ble_conn_param_mgr_owner_priority(
    app_ble_conn_param_owner_t owner)
{
    switch (owner)
    {
    case APP_BLE_CONN_PARAM_OWNER_CI_STATE:
        return 4U;
    case APP_BLE_CONN_PARAM_OWNER_RUNTIME_HIGH:
    case APP_BLE_CONN_PARAM_OWNER_EXPORT_HIGH:
        return 3U;
    case APP_BLE_CONN_PARAM_OWNER_STANDBY_RESTORE:
    case APP_BLE_CONN_PARAM_OWNER_EXPORT_RESTORE:
        return 2U;
    case APP_BLE_CONN_PARAM_OWNER_STANDBY_LOW_POWER:
        return 1U;
    default:
        return 0U;
    }
}

static app_ble_conn_param_owner_t app_ble_conn_param_mgr_first_owner(
    uint16_t owner_mask)
{
    if ((owner_mask &
         (uint16_t)APP_BLE_CONN_PARAM_OWNER_CI_STATE) != 0U)
    {
        return APP_BLE_CONN_PARAM_OWNER_CI_STATE;
    }
    if ((owner_mask &
         (uint16_t)APP_BLE_CONN_PARAM_OWNER_RUNTIME_HIGH) != 0U)
    {
        return APP_BLE_CONN_PARAM_OWNER_RUNTIME_HIGH;
    }
    if ((owner_mask &
         (uint16_t)APP_BLE_CONN_PARAM_OWNER_EXPORT_HIGH) != 0U)
    {
        return APP_BLE_CONN_PARAM_OWNER_EXPORT_HIGH;
    }
    if ((owner_mask &
         (uint16_t)APP_BLE_CONN_PARAM_OWNER_STANDBY_RESTORE) != 0U)
    {
        return APP_BLE_CONN_PARAM_OWNER_STANDBY_RESTORE;
    }
    if ((owner_mask &
         (uint16_t)APP_BLE_CONN_PARAM_OWNER_EXPORT_RESTORE) != 0U)
    {
        return APP_BLE_CONN_PARAM_OWNER_EXPORT_RESTORE;
    }
    if ((owner_mask &
         (uint16_t)APP_BLE_CONN_PARAM_OWNER_STANDBY_LOW_POWER) != 0U)
    {
        return APP_BLE_CONN_PARAM_OWNER_STANDBY_LOW_POWER;
    }
    return (app_ble_conn_param_owner_t)0;
}

static void app_ble_conn_param_mgr_read_actual(uint8_t conn_id,
                                               uint16_t *actual_ci,
                                               uint16_t *actual_latency,
                                               uint16_t *actual_timeout)
{
    uint16_t ci = 0U;
    uint16_t latency = 0U;
    uint16_t timeout = 0U;

    (void)le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &ci, conn_id);
    (void)le_get_conn_param(GAP_PARAM_CONN_LATENCY, &latency, conn_id);
    (void)le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &timeout, conn_id);

    if (actual_ci != NULL)
    {
        *actual_ci = ci;
    }
    if (actual_latency != NULL)
    {
        *actual_latency = latency;
    }
    if (actual_timeout != NULL)
    {
        *actual_timeout = timeout;
    }
}

static bool app_ble_conn_param_mgr_target_ready(uint16_t ci_min,
                                                uint16_t ci_max,
                                                uint16_t latency,
                                                uint16_t actual_ci,
                                                uint16_t actual_latency)
{
    return (actual_ci != 0U) &&
           (actual_ci >= ci_min) &&
           (actual_ci <= ci_max) &&
           (actual_latency == latency);
}

static uint16_t app_ble_conn_param_mgr_standby_latency_min(uint16_t interval)
{
    uint32_t interval_ms;
    uint32_t min_latency;

    if (interval == 0U)
    {
        return 0U;
    }

    interval_ms = (((uint32_t)interval * 125UL) + 99UL) / 100UL;
    if (interval_ms == 0UL)
    {
        return 0U;
    }

    min_latency = (((uint32_t)ZY100_BLE_STANDBY_EFFECTIVE_MIN_MS) +
                   interval_ms - 1UL) / interval_ms;
    if (min_latency > (uint32_t)ZY100_BLE_STANDBY_CONN_LATENCY)
    {
        min_latency = (uint32_t)ZY100_BLE_STANDBY_CONN_LATENCY;
    }
    return (uint16_t)min_latency;
}

static bool app_ble_conn_param_mgr_actual_standby_ready(uint16_t actual_ci,
                                                        uint16_t actual_latency)
{
    uint16_t latency_min;

    if (actual_ci != (uint16_t)ZY100_BLE_STANDBY_CONN_INTERVAL_MIN)
    {
        return false;
    }

    latency_min = app_ble_conn_param_mgr_standby_latency_min(actual_ci);
    return (actual_latency >= latency_min) &&
           (actual_latency <= (uint16_t)ZY100_BLE_STANDBY_CONN_LATENCY);
}

static bool app_ble_conn_param_mgr_target_reached(uint16_t owner_mask,
                                                  uint16_t ci_min,
                                                  uint16_t ci_max,
                                                  uint16_t latency,
                                                  uint16_t timeout,
                                                  uint16_t actual_ci,
                                                  uint16_t actual_latency,
                                                  uint16_t actual_timeout)
{
    (void)timeout;
    (void)actual_timeout;

    if ((owner_mask &
         (uint16_t)APP_BLE_CONN_PARAM_OWNER_STANDBY_LOW_POWER) != 0U)
    {
        return app_ble_conn_param_mgr_actual_standby_ready(actual_ci,
                                                          actual_latency);
    }

    return app_ble_conn_param_mgr_target_ready(ci_min,
                                               ci_max,
                                               latency,
                                               actual_ci,
                                               actual_latency);
}

static bool app_ble_conn_param_mgr_same_active(uint8_t conn_id,
                                               uint16_t ci_min,
                                               uint16_t ci_max,
                                               uint16_t latency,
                                               uint16_t timeout)
{
    return s_ble_conn_param_mgr.active.active &&
           (s_ble_conn_param_mgr.active.conn_id == conn_id) &&
           (s_ble_conn_param_mgr.active.ci_min == ci_min) &&
           (s_ble_conn_param_mgr.active.ci_max == ci_max) &&
           (s_ble_conn_param_mgr.active.latency == latency) &&
           (s_ble_conn_param_mgr.active.timeout == timeout);
}

static bool app_ble_conn_param_mgr_same_next(uint8_t conn_id,
                                             uint16_t ci_min,
                                             uint16_t ci_max,
                                             uint16_t latency,
                                             uint16_t timeout)
{
    return s_ble_conn_param_mgr.next.valid &&
           (s_ble_conn_param_mgr.next.conn_id == conn_id) &&
           (s_ble_conn_param_mgr.next.ci_min == ci_min) &&
           (s_ble_conn_param_mgr.next.ci_max == ci_max) &&
           (s_ble_conn_param_mgr.next.latency == latency) &&
           (s_ble_conn_param_mgr.next.timeout == timeout);
}

static bool app_ble_conn_param_mgr_cap_target_matches(uint8_t conn_id,
                                                      uint16_t ci_min,
                                                      uint16_t ci_max,
                                                      uint16_t latency)
{
    return (s_ble_conn_param_mgr.cap_conn_id == conn_id) &&
           (s_ble_conn_param_mgr.cap_ci_min == ci_min) &&
           (s_ble_conn_param_mgr.cap_ci_max == ci_max) &&
           (s_ble_conn_param_mgr.cap_latency == latency);
}

static void app_ble_conn_param_mgr_clear_high_cap_state(void)
{
    s_ble_conn_param_mgr.cap_conn_id = APP_BLE_CONN_PARAM_CONN_ID_INVALID;
    s_ble_conn_param_mgr.cap_ci_min = 0U;
    s_ble_conn_param_mgr.cap_ci_max = 0U;
    s_ble_conn_param_mgr.cap_latency = 0U;
    s_ble_conn_param_mgr.high_supported = false;
    s_ble_conn_param_mgr.high_failed = false;
}

static void app_ble_conn_param_mgr_note_high_supported(uint8_t conn_id,
                                                       uint16_t ci_min,
                                                       uint16_t ci_max,
                                                       uint16_t latency,
                                                       uint16_t actual_ci,
                                                       uint16_t actual_latency)
{
    s_ble_conn_param_mgr.cap_conn_id = conn_id;
    s_ble_conn_param_mgr.cap_ci_min = ci_min;
    s_ble_conn_param_mgr.cap_ci_max = ci_max;
    s_ble_conn_param_mgr.cap_latency = latency;
    s_ble_conn_param_mgr.high_supported = true;
    s_ble_conn_param_mgr.high_failed = false;
    if (!app_ble_online_quiet_logs_active())
    {
        DBG_DIRECT("[BLE_CI_CAP] high_ok=1 c=%u ci_to=%u/%u lat_to=%u ci=%u lat=%u",
                   conn_id,
                   ci_min,
                   ci_max,
                   latency,
                   actual_ci,
                   actual_latency);
    }
}

static void app_ble_conn_param_mgr_note_high_failed(uint8_t conn_id,
                                                    uint16_t ci_min,
                                                    uint16_t ci_max,
                                                    uint16_t latency,
                                                    uint16_t actual_ci,
                                                    uint16_t actual_latency)
{
    s_ble_conn_param_mgr.cap_conn_id = conn_id;
    s_ble_conn_param_mgr.cap_ci_min = ci_min;
    s_ble_conn_param_mgr.cap_ci_max = ci_max;
    s_ble_conn_param_mgr.cap_latency = latency;
    s_ble_conn_param_mgr.high_supported = false;
    s_ble_conn_param_mgr.high_failed = true;
    DBG_DIRECT("[BLE_CI_CAP] high_fail=1 c=%u ci_to=%u/%u lat_to=%u ci=%u lat=%u",
               conn_id,
               ci_min,
               ci_max,
               latency,
               actual_ci,
               actual_latency);
}

static void app_ble_conn_param_mgr_clear_active(void)
{
    memset(&s_ble_conn_param_mgr.active, 0, sizeof(s_ble_conn_param_mgr.active));
    s_ble_conn_param_mgr.active.conn_id = APP_BLE_CONN_PARAM_CONN_ID_INVALID;
}

static void app_ble_conn_param_mgr_cancel_owner_internal(
    app_ble_conn_param_owner_t owner,
    uint8_t conn_id,
    const char *reason)
{
    uint16_t owner_bit = (uint16_t)owner;
    bool active_deferred = false;
    bool next_cancelled = false;

    if (s_ble_conn_param_mgr.active.active &&
        (s_ble_conn_param_mgr.active.conn_id == conn_id) &&
        ((s_ble_conn_param_mgr.active.owner_mask & owner_bit) != 0U))
    {
        /*
         * le_update_conn_param() has no application-level cancellation in
         * this integration. Keep the accepted LLCP serialized until GAP
         * reports SUCCESS/FAIL or the link disconnects. The replacement
         * semantic target, if any, is queued by the following request.
         */
        active_deferred = true;
    }

    if (s_ble_conn_param_mgr.next.valid &&
        (s_ble_conn_param_mgr.next.conn_id == conn_id) &&
        ((s_ble_conn_param_mgr.next.owner_mask & owner_bit) != 0U))
    {
        s_ble_conn_param_mgr.next.owner_mask =
            (uint16_t)(s_ble_conn_param_mgr.next.owner_mask & ~owner_bit);
        next_cancelled = true;
        if (s_ble_conn_param_mgr.next.owner_mask == 0U)
        {
            memset(&s_ble_conn_param_mgr.next,
                   0,
                   sizeof(s_ble_conn_param_mgr.next));
            s_ble_conn_param_mgr.next.conn_id = APP_BLE_CONN_PARAM_CONN_ID_INVALID;
        }
        else if (s_ble_conn_param_mgr.next.primary_owner == owner)
        {
            s_ble_conn_param_mgr.next.primary_owner =
                app_ble_conn_param_mgr_first_owner(
                    s_ble_conn_param_mgr.next.owner_mask);
        }
    }

    if (app_ble_conn_param_mgr_owner_is_high(owner_bit))
    {
        app_ble_conn_param_mgr_clear_high_cap_state();
    }

    DBG_DIRECT("[BLE_CPM] cancel owner=%s c=%u defer=%u cancel=%u why=%s",
               app_ble_conn_param_mgr_owner_str(owner),
               conn_id,
               active_deferred ? 1U : 0U,
               next_cancelled ? 1U : 0U,
               (reason != NULL) ? reason : "none");
}

void app_ble_conn_param_mgr_reset(const char *reason)
{
    (void)reason;
    memset(&s_ble_conn_param_mgr, 0, sizeof(s_ble_conn_param_mgr));
    s_ble_conn_param_mgr.active.conn_id = APP_BLE_CONN_PARAM_CONN_ID_INVALID;
    s_ble_conn_param_mgr.next.conn_id = APP_BLE_CONN_PARAM_CONN_ID_INVALID;
    s_ble_conn_param_mgr.cap_conn_id = APP_BLE_CONN_PARAM_CONN_ID_INVALID;
}

static void app_ble_conn_param_mgr_queue_next(
    app_ble_conn_param_owner_t primary_owner,
    uint16_t owner_mask,
    uint8_t conn_id,
    uint16_t ci_min,
    uint16_t ci_max,
    uint16_t latency,
    uint16_t timeout,
    uint8_t retry,
    uint32_t begin_os_ms,
    uint32_t due_os_ms,
    const char *reason)
{
    app_ble_conn_param_mgr_next_t *next = &s_ble_conn_param_mgr.next;

    next->valid = true;
    next->primary_owner = primary_owner;
    next->owner_mask = owner_mask;
    next->conn_id = conn_id;
    next->ci_min = ci_min;
    next->ci_max = ci_max;
    next->latency = latency;
    next->timeout = timeout;
    next->retry = retry;
    next->begin_os_ms = begin_os_ms;
    next->due_os_ms = due_os_ms;
    next->reason = reason;
}

static bool app_ble_conn_param_mgr_submit(
    app_ble_conn_param_owner_t primary_owner,
    uint16_t owner_mask,
    uint8_t conn_id,
    uint16_t ci_min,
    uint16_t ci_max,
    uint16_t latency,
    uint16_t timeout,
    uint8_t retry,
    uint32_t begin_os_ms,
    const char *reason)
{
    uint16_t actual_ci = 0U;
    uint16_t actual_latency = 0U;
    uint16_t actual_timeout = 0U;
    uint32_t now_ms = app_ble_conn_param_mgr_now_ms();
    T_GAP_CAUSE cause;

    (void)reason;
    app_ble_conn_param_mgr_read_actual(conn_id,
                                       &actual_ci,
                                       &actual_latency,
                                       &actual_timeout);
    if (app_ble_conn_param_mgr_target_reached(owner_mask,
                                              ci_min,
                                              ci_max,
                                              latency,
                                              timeout,
                                              actual_ci,
                                              actual_latency,
                                              actual_timeout))
    {
        if (app_ble_conn_param_mgr_owner_is_high(owner_mask))
        {
            app_ble_conn_param_mgr_note_high_supported(conn_id,
                                                       ci_min,
                                                       ci_max,
                                                       latency,
                                                       actual_ci,
                                                       actual_latency);
        }
        return true;
    }

    cause = le_update_conn_param(conn_id,
                                 ci_min,
                                 ci_max,
                                 latency,
                                 timeout,
                                 app_ble_standby_ce_len(ci_min),
                                 app_ble_standby_ce_len(ci_max));

    if (!app_ble_online_quiet_logs_active())
    {
        DBG_DIRECT("[BLE_CPM] req owner=%s c=%u ci=%u/%u lat=%u tout=%u cause=0x%x rt=%u",
                   app_ble_conn_param_mgr_owner_str(primary_owner),
                   conn_id,
                   ci_min,
                   ci_max,
                   latency,
                   timeout,
                   cause,
                   retry);
    }

    if (cause != GAP_CAUSE_SUCCESS)
    {
        uint32_t total_wait_ms =
            (uint32_t)(now_ms - ((begin_os_ms != 0U) ? begin_os_ms : now_ms));

        DBG_DIRECT("[BLE_CPM] fail_actual owner=%s c=%u ci=%u lat=%u tout=%u cause=0x%x wait_ms=%lu rt=%u why=submit_fail",
                   app_ble_conn_param_mgr_owner_str(primary_owner),
                   conn_id,
                   actual_ci,
                   actual_latency,
                   actual_timeout,
                   cause,
                   (unsigned long)total_wait_ms,
                   retry);
        if ((total_wait_ms < (uint32_t)ZY100_BLE_CONN_PARAM_TOTAL_TIMEOUT_MS) &&
            (retry < (uint8_t)ZY100_BLE_CONN_PARAM_RETRY_MAX))
        {
            app_ble_conn_param_mgr_queue_next(
                primary_owner,
                owner_mask,
                conn_id,
                ci_min,
                ci_max,
                latency,
                timeout,
                (uint8_t)(retry + 1U),
                (begin_os_ms != 0U) ? begin_os_ms : now_ms,
                now_ms + (uint32_t)ZY100_BLE_CONN_PARAM_RETRY_COOLDOWN_MS,
                reason);
            return false;
        }

        ZY100_LOG_WARN("[BLE_CPM] total_timeout owner=%s c=%u ci=%u lat=%u wait_ms=%lu fb=1",
                   app_ble_conn_param_mgr_owner_str(primary_owner),
                   conn_id,
                   actual_ci,
                   actual_latency,
                   (unsigned long)total_wait_ms);
        if (app_ble_conn_param_mgr_owner_is_high(owner_mask))
        {
            app_ble_conn_param_mgr_note_high_failed(conn_id,
                                                    ci_min,
                                                    ci_max,
                                                    latency,
                                                    actual_ci,
                                                    actual_latency);
        }
        return false;
    }

    s_ble_conn_param_mgr.active.active = true;
    s_ble_conn_param_mgr.active.gap_pending_seen = false;
    s_ble_conn_param_mgr.active.long_wait_logged = false;
    s_ble_conn_param_mgr.active.primary_owner = primary_owner;
    s_ble_conn_param_mgr.active.owner_mask = owner_mask;
    s_ble_conn_param_mgr.active.conn_id = conn_id;
    s_ble_conn_param_mgr.active.ci_min = ci_min;
    s_ble_conn_param_mgr.active.ci_max = ci_max;
    s_ble_conn_param_mgr.active.latency = latency;
    s_ble_conn_param_mgr.active.timeout = timeout;
    s_ble_conn_param_mgr.active.retry = retry;
    s_ble_conn_param_mgr.active.begin_os_ms =
        (begin_os_ms != 0U) ? begin_os_ms : now_ms;
    s_ble_conn_param_mgr.active.request_os_ms = now_ms;
    s_ble_conn_param_mgr.active.next_wait_log_os_ms =
        now_ms + (uint32_t)ZY100_BLE_CONN_PARAM_NO_PENDING_TIMEOUT_MS;
    s_ble_conn_param_mgr.active.actual_ci = actual_ci;
    s_ble_conn_param_mgr.active.actual_latency = actual_latency;
    s_ble_conn_param_mgr.active.actual_timeout = actual_timeout;
    s_ble_conn_param_mgr.active.reason = reason;

    if (app_ble_conn_param_mgr_owner_is_high(owner_mask))
    {
        s_ble_conn_param_mgr.cap_conn_id = conn_id;
        s_ble_conn_param_mgr.cap_ci_min = ci_min;
        s_ble_conn_param_mgr.cap_ci_max = ci_max;
        s_ble_conn_param_mgr.cap_latency = latency;
        s_ble_conn_param_mgr.high_supported = false;
        s_ble_conn_param_mgr.high_failed = false;
    }
    return true;
}

static void app_ble_conn_param_mgr_submit_next_if_due(uint32_t now_ms)
{
    app_ble_conn_param_mgr_next_t next;

    if (s_ble_conn_param_mgr.active.active ||
        !s_ble_conn_param_mgr.next.valid ||
        !app_ble_conn_param_mgr_due(now_ms,
                                    s_ble_conn_param_mgr.next.due_os_ms))
    {
        return;
    }

    next = s_ble_conn_param_mgr.next;
    memset(&s_ble_conn_param_mgr.next, 0, sizeof(s_ble_conn_param_mgr.next));
    (void)app_ble_conn_param_mgr_submit(next.primary_owner,
                                        next.owner_mask,
                                        next.conn_id,
                                        next.ci_min,
                                        next.ci_max,
                                        next.latency,
                                        next.timeout,
                                        next.retry,
                                        next.begin_os_ms,
                                        next.reason);
}

static void app_ble_conn_param_mgr_finish_active(bool success,
                                                 uint16_t cause,
                                                 uint16_t actual_ci,
                                                 uint16_t actual_latency,
                                                 uint16_t actual_timeout,
                                                 const char *fail_reason)
{
    app_ble_conn_param_mgr_active_t active = s_ble_conn_param_mgr.active;
    uint32_t now_ms = app_ble_conn_param_mgr_now_ms();
    uint32_t wait_ms = (uint32_t)(now_ms - active.begin_os_ms);
    uint32_t retry_due;

    if (!active.active)
    {
        return;
    }

    if (success)
    {
        if (!app_ble_online_quiet_logs_active())
        {
            DBG_DIRECT("[BLE_CPM] success owners=0x%x c=%u ci=%u lat=%u tout=%u wait_ms=%lu rt=%u",
                       active.owner_mask,
                       active.conn_id,
                       actual_ci,
                       actual_latency,
                       actual_timeout,
                       (unsigned long)wait_ms,
                       active.retry);
        }
        if (app_ble_conn_param_mgr_owner_is_high(active.owner_mask) &&
            app_ble_conn_param_mgr_target_ready(active.ci_min,
                                                active.ci_max,
                                                active.latency,
                                                actual_ci,
                                                actual_latency))
        {
            app_ble_conn_param_mgr_note_high_supported(active.conn_id,
                                                       active.ci_min,
                                                       active.ci_max,
                                                       active.latency,
                                                       actual_ci,
                                                       actual_latency);
        }
        if ((active.owner_mask &
             (uint16_t)APP_BLE_CONN_PARAM_OWNER_STANDBY_RESTORE) != 0U)
        {
            app_ble_standby_clear_conn_param();
        }
        app_ble_conn_param_mgr_clear_active();
        app_ble_conn_param_mgr_submit_next_if_due(now_ms);
        return;
    }

    DBG_DIRECT("[BLE_CPM] fail_actual owner=%s c=%u ci=%u lat=%u tout=%u cause=0x%x wait_ms=%lu rt=%u why=%s",
               app_ble_conn_param_mgr_owner_str(active.primary_owner),
               active.conn_id,
               actual_ci,
               actual_latency,
               actual_timeout,
               cause,
               (unsigned long)wait_ms,
               active.retry,
               (fail_reason != NULL) ? fail_reason : "gap_fail");

    /*
     * GAP has now produced a terminal result, so the transport slot may be
     * released. A newer semantic target always wins over retrying the old
     * target. This prevents CI160/CI48/CI9 from racing after state changes.
     */
    if (s_ble_conn_param_mgr.next.valid)
    {
        DBG_DIRECT("[BLE_CPM] terminal_superseded owner=%s c=%u failed_ci=%u/%u next_owner=%s next_ci=%u/%u cause=0x%x",
                   app_ble_conn_param_mgr_owner_str(active.primary_owner),
                   active.conn_id,
                   active.ci_min,
                   active.ci_max,
                   app_ble_conn_param_mgr_owner_str(
                       s_ble_conn_param_mgr.next.primary_owner),
                   s_ble_conn_param_mgr.next.ci_min,
                   s_ble_conn_param_mgr.next.ci_max,
                   cause);
        app_ble_conn_param_mgr_clear_active();
        app_ble_conn_param_mgr_submit_next_if_due(now_ms);
        return;
    }

    if ((wait_ms < (uint32_t)ZY100_BLE_CONN_PARAM_TOTAL_TIMEOUT_MS) &&
        (active.retry < (uint8_t)ZY100_BLE_CONN_PARAM_RETRY_MAX))
    {
        retry_due = now_ms + (uint32_t)ZY100_BLE_CONN_PARAM_RETRY_COOLDOWN_MS;
        app_ble_conn_param_mgr_queue_next(active.primary_owner,
                                          active.owner_mask,
                                          active.conn_id,
                                          active.ci_min,
                                          active.ci_max,
                                          active.latency,
                                          active.timeout,
                                          (uint8_t)(active.retry + 1U),
                                          active.begin_os_ms,
                                          retry_due,
                                          active.reason);
        app_ble_conn_param_mgr_clear_active();
        return;
    }

    ZY100_LOG_WARN("[BLE_CPM] total_timeout owner=%s c=%u ci=%u lat=%u wait_ms=%lu fb=1",
               app_ble_conn_param_mgr_owner_str(active.primary_owner),
               active.conn_id,
               actual_ci,
               actual_latency,
               (unsigned long)wait_ms);
    if (app_ble_conn_param_mgr_owner_is_high(active.owner_mask))
    {
        app_ble_conn_param_mgr_note_high_failed(active.conn_id,
                                                active.ci_min,
                                                active.ci_max,
                                                active.latency,
                                                actual_ci,
                                                actual_latency);
    }
    if ((active.owner_mask &
         (uint16_t)APP_BLE_CONN_PARAM_OWNER_STANDBY_RESTORE) != 0U)
    {
        app_ble_standby_clear_conn_param();
    }
    app_ble_conn_param_mgr_clear_active();
    app_ble_conn_param_mgr_submit_next_if_due(now_ms);
}

app_ble_conn_param_request_result_t app_ble_conn_param_mgr_request(
    app_ble_conn_param_owner_t owner,
    uint8_t conn_id,
    uint16_t ci_min,
    uint16_t ci_max,
    uint16_t latency,
    uint16_t timeout,
    const char *reason)
{
    uint16_t actual_ci = 0U;
    uint16_t actual_latency = 0U;
    uint16_t actual_timeout = 0U;
    uint32_t now_ms = app_ble_conn_param_mgr_now_ms();

    if (!app_ble_ci_state_peripheral_llcp_allowed(ci_min,
                                                  ci_max,
                                                  latency,
                                                  timeout))
    {
        DBG_DIRECT("[BLE_CI_HOST][PERIPHERAL_LLCP_BLOCKED] owner=%s c=%u ci=%u..%u lat=%u why=%s",
                   app_ble_conn_param_mgr_owner_str(owner),
                   conn_id,
                   ci_min,
                   ci_max,
                   latency,
                   (reason != NULL) ? reason : "none");
        return APP_BLE_CONN_PARAM_REQ_FAILED;
    }

    if (!app_ble_power_conn_valid(conn_id))
    {
        return APP_BLE_CONN_PARAM_REQ_FAILED;
    }

    app_ble_conn_param_mgr_read_actual(conn_id,
                                       &actual_ci,
                                       &actual_latency,
                                       &actual_timeout);
    if (owner == APP_BLE_CONN_PARAM_OWNER_EXPORT_RESTORE)
    {
        app_ble_conn_param_mgr_cancel_owner_internal(
            APP_BLE_CONN_PARAM_OWNER_EXPORT_HIGH,
            conn_id,
            "restore_preempt");
    }

    if (app_ble_conn_param_mgr_same_active(conn_id,
                                           ci_min,
                                           ci_max,
                                           latency,
                                           timeout))
    {
        s_ble_conn_param_mgr.active.owner_mask |= (uint16_t)owner;
        DBG_DIRECT("[BLE_CPM] wait_same_target owner=%s active_owner=%s c=%u ci=%u lat=%u",
                   app_ble_conn_param_mgr_owner_str(owner),
                   app_ble_conn_param_mgr_owner_str(
                       s_ble_conn_param_mgr.active.primary_owner),
                   conn_id,
                   ci_min,
                   latency);
        return APP_BLE_CONN_PARAM_REQ_WAIT_SAME_TARGET;
    }

    if (s_ble_conn_param_mgr.active.active)
    {
        uint8_t new_prio = app_ble_conn_param_mgr_owner_priority(owner);
        uint8_t next_prio = app_ble_conn_param_mgr_owner_priority(
            s_ble_conn_param_mgr.next.primary_owner);
        bool actual_match = app_ble_conn_param_mgr_target_reached(
                                (uint16_t)owner,
                                ci_min,
                                ci_max,
                                latency,
                                timeout,
                                actual_ci,
                                actual_latency,
                                actual_timeout);

        if (!s_ble_conn_param_mgr.next.valid || (new_prio >= next_prio))
        {
            app_ble_conn_param_mgr_queue_next(owner,
                                              (uint16_t)owner,
                                              conn_id,
                                              ci_min,
                                              ci_max,
                                              latency,
                                              timeout,
                                              0U,
                                              now_ms,
                                              now_ms,
                                              reason);
        }
        DBG_DIRECT("[BLE_CPM] busy owner=%s active_owner=%s c=%u active_ci=%u/%u next_valid=%u queued_ci=%u/%u ci=%u match=%u act=serialize",
                   app_ble_conn_param_mgr_owner_str(owner),
                   app_ble_conn_param_mgr_owner_str(
                       s_ble_conn_param_mgr.active.primary_owner),
                   conn_id,
                   s_ble_conn_param_mgr.active.ci_min,
                   s_ble_conn_param_mgr.active.ci_max,
                   s_ble_conn_param_mgr.next.valid ? 1U : 0U,
                   s_ble_conn_param_mgr.next.valid ?
                   s_ble_conn_param_mgr.next.ci_min : 0U,
                   s_ble_conn_param_mgr.next.valid ?
                   s_ble_conn_param_mgr.next.ci_max : 0U,
                   actual_ci,
                   actual_match ? 1U : 0U);
        return APP_BLE_CONN_PARAM_REQ_BUSY;
    }

    if (s_ble_conn_param_mgr.next.valid)
    {
        if (app_ble_conn_param_mgr_same_next(conn_id,
                                             ci_min,
                                             ci_max,
                                             latency,
                                             timeout))
        {
            if (app_ble_conn_param_mgr_target_reached(
                    (uint16_t)owner,
                    ci_min,
                    ci_max,
                    latency,
                    timeout,
                    actual_ci,
                    actual_latency,
                    actual_timeout))
            {
                DBG_DIRECT("[BLE_CPM] queued_target_reached owner=%s c=%u ci=%u/%u lat=%u act=clear_queue_settled",
                           app_ble_conn_param_mgr_owner_str(owner),
                           conn_id,
                           ci_min,
                           ci_max,
                           latency);
                memset(&s_ble_conn_param_mgr.next,
                       0,
                       sizeof(s_ble_conn_param_mgr.next));
                s_ble_conn_param_mgr.next.conn_id =
                    APP_BLE_CONN_PARAM_CONN_ID_INVALID;
                if (app_ble_conn_param_mgr_owner_is_high((uint16_t)owner))
                {
                    app_ble_conn_param_mgr_note_high_supported(conn_id,
                                                               ci_min,
                                                               ci_max,
                                                               latency,
                                                               actual_ci,
                                                               actual_latency);
                }
                return APP_BLE_CONN_PARAM_REQ_ALREADY_REACHED;
            }
            s_ble_conn_param_mgr.next.owner_mask |= (uint16_t)owner;
            DBG_DIRECT("[BLE_CPM] wait_same_target owner=%s active_owner=%s c=%u ci=%u lat=%u",
                       app_ble_conn_param_mgr_owner_str(owner),
                       app_ble_conn_param_mgr_owner_str(
                           s_ble_conn_param_mgr.next.primary_owner),
                       conn_id,
                       ci_min,
                       latency);
            return APP_BLE_CONN_PARAM_REQ_WAIT_SAME_TARGET;
        }

        if (app_ble_conn_param_mgr_owner_priority(owner) >=
            app_ble_conn_param_mgr_owner_priority(
                s_ble_conn_param_mgr.next.primary_owner))
        {
            DBG_DIRECT("[BLE_CPM] replace_queued owner=%s old_owner=%s c=%u old_ci=%u/%u new_ci=%u/%u act=latest_target_wins",
                       app_ble_conn_param_mgr_owner_str(owner),
                       app_ble_conn_param_mgr_owner_str(
                           s_ble_conn_param_mgr.next.primary_owner),
                       conn_id,
                       s_ble_conn_param_mgr.next.ci_min,
                       s_ble_conn_param_mgr.next.ci_max,
                       ci_min,
                       ci_max);
            memset(&s_ble_conn_param_mgr.next,
                   0,
                   sizeof(s_ble_conn_param_mgr.next));
        }
        else
        {
            DBG_DIRECT("[BLE_CPM] busy owner=%s active_owner=%s c=%u active_ci=%u/%u next_valid=%u",
                       app_ble_conn_param_mgr_owner_str(owner),
                       app_ble_conn_param_mgr_owner_str(
                           s_ble_conn_param_mgr.next.primary_owner),
                       conn_id,
                       s_ble_conn_param_mgr.next.ci_min,
                       s_ble_conn_param_mgr.next.ci_max,
                       1U);
            return APP_BLE_CONN_PARAM_REQ_BUSY;
        }
    }

    if (app_ble_conn_param_mgr_target_reached((uint16_t)owner,
                                              ci_min,
                                              ci_max,
                                              latency,
                                              timeout,
                                              actual_ci,
                                              actual_latency,
                                              actual_timeout))
    {
        if (app_ble_conn_param_mgr_owner_is_high((uint16_t)owner))
        {
            app_ble_conn_param_mgr_note_high_supported(conn_id,
                                                       ci_min,
                                                       ci_max,
                                                       latency,
                                                       actual_ci,
                                                       actual_latency);
        }
        DBG_DIRECT("[BLE_CPM] settled owner=%s c=%u ci=%u/%u lat=%u ci=%u lat=%u active=0 queued=0",
                   app_ble_conn_param_mgr_owner_str(owner),
                   conn_id,
                   ci_min,
                   ci_max,
                   latency,
                   actual_ci,
                   actual_latency);
        return APP_BLE_CONN_PARAM_REQ_ALREADY_REACHED;
    }

    if (app_ble_conn_param_mgr_submit(owner,
                                      (uint16_t)owner,
                                      conn_id,
                                      ci_min,
                                      ci_max,
                                      latency,
                                      timeout,
                                      0U,
                                      now_ms,
                                      reason))
    {
        return APP_BLE_CONN_PARAM_REQ_SUBMITTED;
    }

    return APP_BLE_CONN_PARAM_REQ_FAILED;
}

void app_ble_conn_param_mgr_on_gap_update(uint8_t conn_id,
                                          uint8_t status,
                                          uint16_t cause,
                                          uint16_t actual_ci,
                                          uint16_t actual_latency,
                                          uint16_t actual_timeout)
{
    uint32_t now_ms = app_ble_conn_param_mgr_now_ms();
    uint32_t elapsed_ms;

    if (!s_ble_conn_param_mgr.active.active ||
        (s_ble_conn_param_mgr.active.conn_id != conn_id))
    {
        return;
    }

    s_ble_conn_param_mgr.active.actual_ci = actual_ci;
    s_ble_conn_param_mgr.active.actual_latency = actual_latency;
    s_ble_conn_param_mgr.active.actual_timeout = actual_timeout;

    if (status == GAP_CONN_PARAM_UPDATE_STATUS_PENDING)
    {
        s_ble_conn_param_mgr.active.gap_pending_seen = true;
        s_ble_conn_param_mgr.active.next_wait_log_os_ms =
            now_ms + (uint32_t)ZY100_BLE_CONN_PARAM_PENDING_TIMEOUT_MS;
        elapsed_ms = (uint32_t)(now_ms -
                                s_ble_conn_param_mgr.active.request_os_ms);
        if (!app_ble_online_quiet_logs_active())
        {
            DBG_DIRECT("[BLE_CPM] pending owner=%s c=%u ci=%u lat=%u tout=%u ms=%lu",
                       app_ble_conn_param_mgr_owner_str(
                           s_ble_conn_param_mgr.active.primary_owner),
                       conn_id,
                       actual_ci,
                       actual_latency,
                       actual_timeout,
                       (unsigned long)elapsed_ms);
        }
        return;
    }

    if (status == GAP_CONN_PARAM_UPDATE_STATUS_SUCCESS)
    {
        bool target_reached = app_ble_conn_param_mgr_target_reached(
                                  s_ble_conn_param_mgr.active.owner_mask,
                                  s_ble_conn_param_mgr.active.ci_min,
                                  s_ble_conn_param_mgr.active.ci_max,
                                  s_ble_conn_param_mgr.active.latency,
                                  s_ble_conn_param_mgr.active.timeout,
                                  actual_ci,
                                  actual_latency,
                                  actual_timeout);
        app_ble_conn_param_mgr_finish_active(target_reached,
                                             cause,
                                             actual_ci,
                                             actual_latency,
                                             actual_timeout,
                                             target_reached ?
                                             "gap_success" :
                                             "gap_success_actual_mismatch");
        return;
    }

    if (status == GAP_CONN_PARAM_UPDATE_STATUS_FAIL)
    {
        app_ble_conn_param_mgr_finish_active(false,
                                             cause,
                                             actual_ci,
                                             actual_latency,
                                             actual_timeout,
                                             "gap_fail");
    }
}

void app_ble_conn_param_mgr_maintain(void)
{
    app_ble_conn_param_mgr_active_t active;
    uint32_t now_ms = app_ble_conn_param_mgr_now_ms();
    uint32_t request_wait_ms;
    uint32_t total_wait_ms;
    uint32_t limit_ms;
    uint16_t actual_ci = 0U;
    uint16_t actual_latency = 0U;
    uint16_t actual_timeout = 0U;

    if (!s_ble_conn_param_mgr.active.active)
    {
        app_ble_conn_param_mgr_submit_next_if_due(now_ms);
        app_ble_ci_state_maintain_notify_and_disconnect();
        return;
    }

    active = s_ble_conn_param_mgr.active;
    app_ble_conn_param_mgr_read_actual(active.conn_id,
                                       &actual_ci,
                                       &actual_latency,
                                       &actual_timeout);
    s_ble_conn_param_mgr.active.actual_ci = actual_ci;
    s_ble_conn_param_mgr.active.actual_latency = actual_latency;
    s_ble_conn_param_mgr.active.actual_timeout = actual_timeout;

    request_wait_ms = (uint32_t)(now_ms - active.request_os_ms);
    total_wait_ms = (uint32_t)(now_ms - active.begin_os_ms);
    limit_ms = active.gap_pending_seen ?
               (uint32_t)ZY100_BLE_CONN_PARAM_PENDING_TIMEOUT_MS :
               (uint32_t)ZY100_BLE_CONN_PARAM_NO_PENDING_TIMEOUT_MS;

    if (request_wait_ms < limit_ms)
    {
        app_ble_ci_state_maintain_notify_and_disconnect();
        return;
    }

    if (app_ble_conn_param_mgr_due(
            now_ms,
            s_ble_conn_param_mgr.active.next_wait_log_os_ms))
    {
        DBG_DIRECT("[BLE_CPM] inflight_wait owner=%s c=%u ci_to=%u/%u lat_to=%u ci=%u lat=%u request_ms=%lu total_ms=%lu pend=%u keep_active=1 retry=0 next_valid=%u",
                   app_ble_conn_param_mgr_owner_str(active.primary_owner),
                   active.conn_id,
                   active.ci_min,
                   active.ci_max,
                   active.latency,
                   actual_ci,
                   actual_latency,
                   (unsigned long)request_wait_ms,
                   (unsigned long)total_wait_ms,
                   active.gap_pending_seen ? 1U : 0U,
                   s_ble_conn_param_mgr.next.valid ? 1U : 0U);
        s_ble_conn_param_mgr.active.next_wait_log_os_ms =
            now_ms +
            (uint32_t)ZY100_BLE_CONN_PARAM_INFLIGHT_LOG_PERIOD_MS;
    }

    if (!s_ble_conn_param_mgr.active.long_wait_logged &&
        (total_wait_ms >=
         (uint32_t)ZY100_BLE_CONN_PARAM_TOTAL_TIMEOUT_MS))
    {
        s_ble_conn_param_mgr.active.long_wait_logged = true;
        DBG_DIRECT("[BLE_CPM] inflight_long_wait owner=%s c=%u ci_to=%u/%u ci=%u lat=%u total_ms=%lu pend=%u act=wait_terminal",
                   app_ble_conn_param_mgr_owner_str(active.primary_owner),
                   active.conn_id,
                   active.ci_min,
                   active.ci_max,
                   actual_ci,
                   actual_latency,
                   (unsigned long)total_wait_ms,
                   active.gap_pending_seen ? 1U : 0U);
    }

    app_ble_ci_state_maintain_notify_and_disconnect();
}

bool app_ble_conn_param_mgr_is_settled(uint8_t conn_id)
{
    bool active_for_conn = s_ble_conn_param_mgr.active.active &&
                           (s_ble_conn_param_mgr.active.conn_id == conn_id);
    bool queued_for_conn = s_ble_conn_param_mgr.next.valid &&
                           (s_ble_conn_param_mgr.next.conn_id == conn_id);

    return !active_for_conn && !queued_for_conn;
}

bool app_ble_conn_param_mgr_high_ready(uint8_t conn_id,
                                       uint16_t ci_min,
                                       uint16_t ci_max,
                                       uint16_t latency)
{
    uint16_t actual_ci = 0U;
    uint16_t actual_latency = 0U;
    uint16_t actual_timeout = 0U;

    if (!app_ble_power_conn_valid(conn_id))
    {
        return false;
    }

    app_ble_conn_param_mgr_read_actual(conn_id,
                                       &actual_ci,
                                       &actual_latency,
                                       &actual_timeout);
    (void)actual_timeout;
    if (app_ble_conn_param_mgr_target_ready(ci_min,
                                            ci_max,
                                            latency,
                                            actual_ci,
                                            actual_latency))
    {
        if (!s_ble_conn_param_mgr.high_supported ||
            !app_ble_conn_param_mgr_cap_target_matches(conn_id,
                                                       ci_min,
                                                       ci_max,
                                                       latency))
        {
            app_ble_conn_param_mgr_note_high_supported(conn_id,
                                                       ci_min,
                                                       ci_max,
                                                       latency,
                                                       actual_ci,
                                                       actual_latency);
        }
        return true;
    }

    return false;
}

bool app_ble_conn_param_mgr_high_failed(uint8_t conn_id,
                                        uint16_t ci_min,
                                        uint16_t ci_max,
                                        uint16_t latency)
{
    return s_ble_conn_param_mgr.high_failed &&
           app_ble_conn_param_mgr_cap_target_matches(conn_id,
                                                     ci_min,
                                                     ci_max,
                                                     latency);
}

void app_ble_conn_param_mgr_clear_high_cap(uint8_t conn_id)
{
    (void)conn_id;
    app_ble_conn_param_mgr_clear_high_cap_state();
}

void app_ble_conn_param_mgr_cancel_owner(app_ble_conn_param_owner_t owner,
                                         uint8_t conn_id,
                                         const char *reason)
{
    app_ble_conn_param_mgr_cancel_owner_internal(owner, conn_id, reason);
}

void app_ble_conn_param_mgr_abort_all(uint8_t conn_id, const char *reason)
{
    bool active_match = s_ble_conn_param_mgr.active.active &&
                        (s_ble_conn_param_mgr.active.conn_id == conn_id);
    bool next_match = s_ble_conn_param_mgr.next.valid &&
                      (s_ble_conn_param_mgr.next.conn_id == conn_id);

    if (active_match || next_match)
    {
        DBG_DIRECT("[BLE_CPM] reset c=%u active=%u next=%u why=%s",
                   conn_id,
                   active_match ? 1U : 0U,
                   next_match ? 1U : 0U,
                   (reason != NULL) ? reason : "reset");
    }
    if (active_match)
    {
        app_ble_conn_param_mgr_clear_active();
    }
    if (next_match)
    {
        memset(&s_ble_conn_param_mgr.next,
               0,
               sizeof(s_ble_conn_param_mgr.next));
    }
    app_ble_conn_param_mgr_clear_high_cap_state();
}
