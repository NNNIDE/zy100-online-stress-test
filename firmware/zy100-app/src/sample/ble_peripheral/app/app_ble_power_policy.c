/**
 * @file app_ble_power_policy.c
 * @brief BLE advertising, connection and low-power policy ownership.
 */

#include <trace.h>
#include <os_sched.h>
#include <os_timer.h>
#include <stdint.h>
#include <string.h>
#include <gap.h>
#include <gap_adv.h>
#include <gap_conn_le.h>
#include <gap_msg.h>
#if F_BT_DLPS_EN
#include <dlps.h>
#endif

#include "app_flags.h"
#include "app/app_device_pairing.h"
#include "app_ble_power_policy.h"
#include "app_ble_ci_state.h"
#include "app_ble_link_trace.h"
#include "app_ble_conn_param_mgr.h"
#include "service/app_ble_notify_tracker.h"
#include "service/zy100_whole_unit_test.h"
#include "app_ble_sensor_stream.h"

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

static T_GAP_DEV_STATE gap_dev_state = {0, 0, 0, 0};
static T_GAP_CONN_STATE gap_conn_state = GAP_CONN_STATE_DISCONNECTED;

#define APP_BLE_SENSOR_CONN_ID_INVALID         0xFFU
#define APP_BLE_ADV_DEFER_LOG_PERIOD_MS        1000U

typedef struct
{
    bool valid;
    const char *reason;
    uint8_t gate;
    uint8_t stack_ready;
    int32_t init_state;
    int32_t adv_state;
    int32_t conn_state;
    uint8_t conn_id;
    uint8_t start_requested;
} app_ble_power_wake_log_snapshot_t;

typedef struct
{
    bool valid;
    bool restore_pending;
    uint8_t conn_id;
    uint16_t interval;
    uint16_t latency;
    uint16_t timeout;
} app_ble_standby_conn_param_snapshot_t;

static bool s_ble_adv_allowed = false;
static bool s_ble_sleep_quiescing = false;
static bool s_ble_shutdown_latched = false;
static bool s_ble_standby_policy_active = false;
static bool s_ble_sleep_prepare_logged = false;
static bool s_ble_sleep_ready_logged = false;
static bool s_ble_stack_ready = false;
static bool s_ble_boot_storage_ready = true;
static bool s_ble_adv_start_requested = false;
static bool s_ble_adv_stop_requested = false;
static bool s_ble_disconnect_requested = false;
static bool s_ble_disconnect_restore_pending = false;
static uint8_t s_ble_conn_id = APP_BLE_SENSOR_CONN_ID_INVALID;
static app_ble_standby_conn_param_snapshot_t s_ble_standby_conn_param;
static bool s_ble_adv_defer_log_valid = false;
static const char *s_ble_adv_defer_log_reason = NULL;
static uint8_t s_ble_adv_defer_log_kind = 0U;
static int32_t s_ble_adv_defer_log_detail = 0;
static uint64_t s_ble_adv_defer_log_last_ms = 0U;
static app_ble_power_wake_log_snapshot_t s_ble_wake_open_log;
static app_ble_power_wake_log_snapshot_t s_ble_wake_hold_log;

#if F_LP_BLE_ADV_STAGE_SWITCH_EN
#define LP_BLE_ADV_STAGE_TIMER_ID           0x1001
#define LP_BLE_ADV_SWITCH_RETRY_MS          200

typedef enum
{
    LP_BLE_ADV_STAGE_FAST = 0,
    LP_BLE_ADV_STAGE_SLOW = 1,
} T_LP_BLE_ADV_STAGE;

static void *s_lp_adv_stage_timer = NULL;
static bool s_lp_adv_stage_timer_ready = false;
static bool s_lp_link_connected = false;
static bool s_lp_adv_switch_pending = false;
static T_LP_BLE_ADV_STAGE s_lp_adv_stage = LP_BLE_ADV_STAGE_FAST;
static T_LP_BLE_ADV_STAGE s_lp_adv_target_stage = LP_BLE_ADV_STAGE_FAST;
#endif

#if F_LP_BLE_ADV_STAGE_SWITCH_EN
static void app_lp_adv_policy_on_disconnected(void);
#endif
static bool app_ble_power_adv_state_active(void);
static bool app_ble_power_conn_active(void);
static void app_ble_power_try_start_adv(const char *reason);
static void app_ble_standby_apply_adv_policy(const char *reason);
static void app_ble_standby_restore_adv_policy(const char *reason);
static void app_ble_standby_request_conn_param(uint8_t conn_id,
                                               const char *reason);
static void app_ble_standby_restore_conn_param(uint8_t conn_id,
                                                app_ble_ci_state_t target_state,
                                                const char *reason);

typedef enum
{
    APP_BLE_ADV_DEFER_BOOT_STORAGE = 1,
    APP_BLE_ADV_DEFER_GATE_CLOSED,
    APP_BLE_ADV_DEFER_SLEEP_QUIESCING,
    APP_BLE_ADV_DEFER_STACK_NOT_READY,
    APP_BLE_ADV_DEFER_CONNECTED,
    APP_BLE_ADV_DEFER_ADV_STATE,
    APP_BLE_ADV_DEFER_START_REQUESTED,
    APP_BLE_ADV_DEFER_DISCONNECT_RESTORE,
} app_ble_adv_defer_kind_t;

bool app_ble_power_conn_valid(uint8_t conn_id)
{
    T_GAP_CONN_INFO conn_info;

    if ((conn_id == APP_BLE_SENSOR_CONN_ID_INVALID) ||
        (gap_conn_state != GAP_CONN_STATE_CONNECTED) ||
        (le_get_active_link_num() == 0U))
    {
        return false;
    }

    memset(&conn_info, 0, sizeof(conn_info));
    if (!le_get_conn_info(conn_id, &conn_info))
    {
        return false;
    }

    return (conn_info.conn_state == GAP_CONN_STATE_CONNECTED);
}

#if F_BT_DLPS_EN
static void app_set_dlps_by_ble_state(bool enable_dlps)
{
#if F_APP_BUTTON_DLPS_CTRL_ENABLE
    (void)enable_dlps;
#else
    lps_mode_set(enable_dlps ? LPM_DLPS_MODE : LPM_ACTIVE_MODE);
#endif
}
#endif

#if F_LP_BLE_ADV_STAGE_SWITCH_EN
static bool app_lp_adv_state_active(void)
{
    return ((gap_dev_state.gap_adv_state == GAP_ADV_STATE_START) ||
            (gap_dev_state.gap_adv_state == GAP_ADV_STATE_ADVERTISING));
}

static const char *app_lp_adv_stage_str(T_LP_BLE_ADV_STAGE stage)
{
    return (stage == LP_BLE_ADV_STAGE_FAST) ? "FAST" : "SLOW";
}

static uint32_t app_lp_adv_units_to_ms(uint16_t interval_units)
{
    return ((uint32_t)interval_units * 625U) / 1000U;
}

static void app_lp_adv_log_policy_cfg(void)
{
    DBG_DIRECT("[LP_TEST] adv cfg: FAST=%d units(~%d ms), SLOW=%d units(~%d ms), FAST_DUR=%d ms",
               F_LP_BLE_ADV_FAST_INTERVAL_UNITS, app_lp_adv_units_to_ms(F_LP_BLE_ADV_FAST_INTERVAL_UNITS),
               F_LP_BLE_ADV_SLOW_INTERVAL_UNITS, app_lp_adv_units_to_ms(F_LP_BLE_ADV_SLOW_INTERVAL_UNITS),
               F_LP_BLE_ADV_FAST_DURATION_MS);
}

static bool app_lp_adv_set_interval(uint16_t interval_units)
{
    T_GAP_CAUSE cause_min;
    T_GAP_CAUSE cause_max;

    cause_min = le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MIN, sizeof(interval_units), &interval_units);
    cause_max = le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MAX, sizeof(interval_units), &interval_units);
    ZY100_BLE_DIAG_LOG("[BLE_ADV] interval units=%u min=0x%x max=0x%x",
                    (uint32_t)interval_units,
                    cause_min,
                    cause_max);

    if ((cause_min != GAP_CAUSE_SUCCESS) || (cause_max != GAP_CAUSE_SUCCESS))
    {
        DBG_DIRECT("[LP_TEST] adv interval set failed: interval=%d, min_cause=0x%x, max_cause=0x%x",
                   interval_units, cause_min, cause_max);
        return false;
    }

    return true;
}

static void app_lp_adv_stage_timer_start(uint32_t timeout_ms, const char *reason)
{
    if (!s_lp_adv_stage_timer_ready)
    {
        return;
    }

    if (os_timer_restart(&s_lp_adv_stage_timer, timeout_ms))
    {
        ZY100_DIAG_LOG("[LP_TEST] adv-stage timer restart: %d ms (%s)", timeout_ms, reason);
    }
    else
    {
        DBG_DIRECT("[LP_TEST] adv-stage timer restart failed: %d ms (%s)", timeout_ms, reason);
    }
}

static void app_lp_adv_stage_timer_stop(const char *reason)
{
    if (!s_lp_adv_stage_timer_ready)
    {
        return;
    }

    if (os_timer_stop(&s_lp_adv_stage_timer))
    {
        ZY100_DIAG_LOG("[LP_TEST] adv-stage timer stop (%s)", reason);
    }
    else
    {
        DBG_DIRECT("[LP_TEST] adv-stage timer stop failed (%s)", reason);
    }
}

static bool app_lp_adv_start_if_idle(const char *reason)
{
    T_GAP_CAUSE cause;

    if (s_ble_shutdown_latched)
    {
        DBG_DIRECT("[SHUTDOWN][BLE] adv_start_blocked reason=%s",
                   (reason != NULL) ? reason : "unknown");
        return false;
    }

    if (!s_ble_boot_storage_ready)
    {
        ZY100_BLE_DIAG_LOG("[BLE_ADV] wait reason=%s boot_storage_ready=0",
                        (reason != NULL) ? reason : "unknown");
        return false;
    }

    if (gap_dev_state.gap_adv_state != GAP_ADV_STATE_IDLE)
    {
        ZY100_BLE_DIAG_LOG("[BLE_ADV] skip reason=%s adv=%d",
                   (reason != NULL) ? reason : "unknown",
                   gap_dev_state.gap_adv_state);
        return true;
    }

    ZY100_BLE_DIAG_LOG("[BLE_ADV] request reason=%s init=%d adv=%d conn=%d",
               (reason != NULL) ? reason : "unknown",
               gap_dev_state.gap_init_state,
               gap_dev_state.gap_adv_state,
               gap_conn_state);
    cause = le_adv_start();
    ZY100_BLE_DIAG_LOG("[BLE_ADV] result reason=%s cause=0x%x",
               (reason != NULL) ? reason : "unknown",
               cause);
    if (cause != GAP_CAUSE_SUCCESS)
    {
        DBG_DIRECT("[LP_TEST] adv start failed (%s), cause=0x%x", reason, cause);
        return false;
    }

    ZY100_DIAG_LOG("[LP_TEST] adv start (%s), cause=0x%x", reason, cause);
    return true;
}

static void app_lp_adv_apply_pending_stage(void)
{
    uint16_t interval_units;

    if (!s_ble_boot_storage_ready)
    {
        s_lp_adv_switch_pending = false;
        app_lp_adv_stage_timer_stop("boot storage gate");
        return;
    }

    if (!s_lp_adv_switch_pending)
    {
        return;
    }

    if (s_lp_link_connected)
    {
        s_lp_adv_switch_pending = false;
        return;
    }

    if (app_lp_adv_state_active())
    {
        return;
    }

    interval_units = (s_lp_adv_target_stage == LP_BLE_ADV_STAGE_FAST) ?
                     F_LP_BLE_ADV_FAST_INTERVAL_UNITS : F_LP_BLE_ADV_SLOW_INTERVAL_UNITS;

    if (!app_lp_adv_set_interval(interval_units))
    {
        app_lp_adv_stage_timer_start(LP_BLE_ADV_SWITCH_RETRY_MS, "adv interval set retry");
        return;
    }

    s_lp_adv_stage = s_lp_adv_target_stage;
    s_lp_adv_switch_pending = false;
    DBG_DIRECT("[LP_TEST] ADV_STAGE=%s (interval=%d units ~%d ms)",
               app_lp_adv_stage_str(s_lp_adv_stage), interval_units,
               app_lp_adv_units_to_ms(interval_units));

    if (!app_lp_adv_start_if_idle("apply pending stage"))
    {
        app_lp_adv_stage_timer_start(LP_BLE_ADV_SWITCH_RETRY_MS, "adv start retry");
        return;
    }

    if (s_lp_adv_stage == LP_BLE_ADV_STAGE_FAST)
    {
        ZY100_DIAG_LOG("[LP_TEST] FAST duration=%d ms", F_LP_BLE_ADV_FAST_DURATION_MS);
        app_lp_adv_stage_timer_start(F_LP_BLE_ADV_FAST_DURATION_MS, "fast stage duration");
    }
    else
    {
        app_lp_adv_stage_timer_stop("slow stage no switch timer");
    }
}

static void app_lp_adv_request_stage(T_LP_BLE_ADV_STAGE target_stage, const char *reason,
                                     bool force_apply)
{
    T_GAP_CAUSE cause;

    if (s_lp_link_connected)
    {
        ZY100_DIAG_LOG("[LP_TEST] ignore stage switch request while connected (%s)", reason);
        return;
    }

    if (!force_apply && (s_lp_adv_stage == target_stage) && !s_lp_adv_switch_pending)
    {
        DBG_DIRECT("[LP_TEST] ADV_STAGE already %s, skip switch (%s)",
                   app_lp_adv_stage_str(target_stage), reason);
        return;
    }

    s_lp_adv_target_stage = target_stage;
    s_lp_adv_switch_pending = true;

    if (app_lp_adv_state_active())
    {
        cause = le_adv_stop();
        if (cause != GAP_CAUSE_SUCCESS)
        {
            DBG_DIRECT("[LP_TEST] adv stop for stage switch failed: target=%s, cause=0x%x (%s)",
                       app_lp_adv_stage_str(target_stage), cause, reason);
            app_lp_adv_stage_timer_start(LP_BLE_ADV_SWITCH_RETRY_MS, "adv stop retry");
            return;
        }

        DBG_DIRECT("[LP_TEST] adv stop for stage switch: target=%s (%s)",
                   app_lp_adv_stage_str(target_stage), reason);
        return;
    }

    app_lp_adv_apply_pending_stage();
}

static void app_lp_adv_stage_timer_cb(void *p_timer)
{
    (void)p_timer;

    if (s_lp_link_connected)
    {
        s_lp_adv_switch_pending = false;
        ZY100_DIAG_LOG("[LP_TEST] timer fired but link already connected, ignore");
        return;
    }

    if (s_lp_adv_switch_pending)
    {
        DBG_DIRECT("[LP_TEST] timer retry pending stage switch -> %s",
                   app_lp_adv_stage_str(s_lp_adv_target_stage));
        app_lp_adv_request_stage(s_lp_adv_target_stage, "pending stage retry", true);
        return;
    }

    if (s_lp_adv_stage == LP_BLE_ADV_STAGE_SLOW)
    {
        ZY100_DIAG_LOG("[LP_TEST] timer fired in SLOW stage, ignore duplicated switch");
        return;
    }

    DBG_DIRECT("[LP_TEST] fast stage timeout, switch to SLOW advertising");
    app_lp_adv_request_stage(LP_BLE_ADV_STAGE_SLOW, "fast stage timeout", false);
}

static void app_lp_adv_stage_init(void)
{
    if (s_lp_adv_stage_timer_ready)
    {
        return;
    }

    if (os_timer_create(&s_lp_adv_stage_timer, "lp_adv_stage", LP_BLE_ADV_STAGE_TIMER_ID,
                        F_LP_BLE_ADV_FAST_DURATION_MS, false, app_lp_adv_stage_timer_cb))
    {
        s_lp_adv_stage_timer_ready = true;
        ZY100_DIAG_LOG("[LP_TEST] adv-stage timer created (one-shot)");
        app_lp_adv_log_policy_cfg();
    }
    else
    {
        DBG_DIRECT("[LP_TEST] adv-stage timer create failed");
    }
}

static void app_lp_adv_policy_on_connected(void)
{
    s_lp_link_connected = true;
    s_lp_adv_switch_pending = false;
    app_lp_adv_stage_timer_stop("connected");
    ZY100_DIAG_LOG("[LP_TEST] LINK=CONNECTED, stop adv stage switch");

#if F_BT_DLPS_EN
    app_set_dlps_by_ble_state(false);
    ZY100_DIAG_LOG("[LP_TEST] DLPS allow=0 (connected)");
#endif
}

static void app_lp_adv_policy_on_disconnected(void)
{
    s_lp_link_connected = false;
    ZY100_DIAG_LOG("[LP_TEST] LINK=DISCONNECTED, re-enter FAST advertising");
    app_lp_adv_request_stage(LP_BLE_ADV_STAGE_FAST, "disconnected", true);

#if F_BT_DLPS_EN
    app_set_dlps_by_ble_state(true);
    ZY100_DIAG_LOG("[LP_TEST] DLPS allow=1 (disconnected)");
#endif
}
#endif

static bool app_ble_power_adv_state_active(void)
{
    return (s_ble_adv_start_requested ||
            (gap_dev_state.gap_adv_state == GAP_ADV_STATE_START) ||
            (gap_dev_state.gap_adv_state == GAP_ADV_STATE_ADVERTISING));
}

static bool app_ble_power_conn_active(void)
{
    return ((gap_conn_state == GAP_CONN_STATE_CONNECTED) &&
            (s_ble_conn_id != APP_BLE_SENSOR_CONN_ID_INVALID));
}

bool app_ble_power_is_connected(void)
{
    return app_ble_power_conn_active();
}

uint8_t app_ble_power_conn_id(void)
{
    return s_ble_conn_id;
}

uint16_t app_ble_power_conn_mtu(uint8_t conn_id)
{
    return app_ble_sensor_stream_refresh_mtu(conn_id);
}

static bool app_ble_set_adv_interval(uint16_t interval_units,
                                     const char *reason)
{
    T_GAP_CAUSE cause_min;
    T_GAP_CAUSE cause_max;

    cause_min = le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MIN,
                                 sizeof(interval_units),
                                 &interval_units);
    cause_max = le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MAX,
                                 sizeof(interval_units),
                                 &interval_units);
    if ((cause_min != GAP_CAUSE_SUCCESS) || (cause_max != GAP_CAUSE_SUCCESS))
    {
        DBG_DIRECT("[BLE_STANDBY] adv_interval_set_failed interval=%u min=0x%x max=0x%x reason=%s",
                   interval_units,
                   cause_min,
                   cause_max,
                   (reason != NULL) ? reason : "unknown");
        return false;
    }

    DBG_DIRECT("[BLE_STANDBY] adv_interval=%u reason=%s",
               interval_units,
               (reason != NULL) ? reason : "unknown");
    return true;
}

static void app_ble_standby_apply_adv_policy(const char *reason)
{
    T_GAP_CAUSE cause;

    if (app_ble_power_conn_active())
    {
        return;
    }

    if (app_ble_power_adv_state_active())
    {
        if (!s_ble_adv_stop_requested)
        {
            cause = le_adv_stop();
            s_ble_adv_stop_requested = (cause == GAP_CAUSE_SUCCESS);
            DBG_DIRECT("[BLE_STANDBY] adv_stop_for_interval cause=0x%x reason=%s",
                       cause,
                       (reason != NULL) ? reason : "standby");
        }
        return;
    }

    (void)app_ble_set_adv_interval((uint16_t)ZY100_BLE_STANDBY_ADV_INTERVAL_UNITS,
                                   reason);
    app_ble_power_try_start_adv((reason != NULL) ? reason : "standby_adv");
}

static void app_ble_standby_restore_adv_policy(const char *reason)
{
    T_GAP_CAUSE cause;

    if (app_ble_power_conn_active())
    {
        return;
    }

    if (app_ble_power_adv_state_active())
    {
        if (!s_ble_adv_stop_requested)
        {
            cause = le_adv_stop();
            s_ble_adv_stop_requested = (cause == GAP_CAUSE_SUCCESS);
            DBG_DIRECT("[BLE_STANDBY] adv_stop_for_active_interval cause=0x%x reason=%s",
                       cause,
                       (reason != NULL) ? reason : "active");
        }
        return;
    }

    (void)app_ble_set_adv_interval((uint16_t)ZY100_BLE_ACTIVE_ADV_INTERVAL_UNITS,
                                   reason);
    app_ble_power_try_start_adv((reason != NULL) ? reason : "standby_exit");
}

void app_ble_standby_clear_conn_param(void)
{
    memset(&s_ble_standby_conn_param, 0, sizeof(s_ble_standby_conn_param));
    s_ble_standby_conn_param.conn_id = APP_BLE_SENSOR_CONN_ID_INVALID;
}

static void app_ble_standby_request_conn_param(uint8_t conn_id,
                                               const char *reason)
{
    (void)conn_id;
    (void)app_ble_ci_state_request(
        APP_BLE_CI_STATE_STANDBY,
        (reason != NULL) ? reason : "standby");
}

static void app_ble_standby_restore_conn_param(uint8_t conn_id,
                                               app_ble_ci_state_t target_state,
                                               const char *reason)
{
    (void)conn_id;
    (void)app_ble_ci_state_request(
        target_state,
        (reason != NULL) ? reason : "standby_exit");
    app_ble_standby_clear_conn_param();
}

void app_ble_power_log_runtime_dlps_check(void)
{
    ZY100_LOG_VERBOSE("[LP_RUN] ble adv_state=%d conn_state=%d adv_allowed=%d sleep_quiescing=%d stream_ready=%d",
                      gap_dev_state.gap_adv_state,
                      gap_conn_state,
                      (uint8_t)s_ble_adv_allowed,
                      (uint8_t)s_ble_sleep_quiescing,
                      (uint8_t)app_ble_sensor_stream_ready_cached(s_ble_conn_id));
}

static bool app_ble_power_wake_log_due(app_ble_power_wake_log_snapshot_t *last,
                                       const char *reason,
                                       uint8_t gate)
{
    const char *use_reason = (reason != NULL) ? reason : "unknown";
    bool same_reason = false;

    if (last == NULL)
    {
        return true;
    }

    if ((last->reason != NULL) && (strcmp(last->reason, use_reason) == 0))
    {
        same_reason = true;
    }

    if (last->valid &&
        same_reason &&
        (last->gate == gate) &&
        (last->stack_ready == (uint8_t)s_ble_stack_ready) &&
        (last->init_state == (int32_t)gap_dev_state.gap_init_state) &&
        (last->adv_state == (int32_t)gap_dev_state.gap_adv_state) &&
        (last->conn_state == (int32_t)gap_conn_state) &&
        (last->conn_id == s_ble_conn_id) &&
        (last->start_requested == (uint8_t)s_ble_adv_start_requested))
    {
        return false;
    }

    last->valid = true;
    last->reason = use_reason;
    last->gate = gate;
    last->stack_ready = (uint8_t)s_ble_stack_ready;
    last->init_state = (int32_t)gap_dev_state.gap_init_state;
    last->adv_state = (int32_t)gap_dev_state.gap_adv_state;
    last->conn_state = (int32_t)gap_conn_state;
    last->conn_id = s_ble_conn_id;
    last->start_requested = (uint8_t)s_ble_adv_start_requested;
    return true;
}

static bool app_ble_power_defer_log_due(const char *reason,
                                        app_ble_adv_defer_kind_t kind,
                                        int32_t detail)
{
    const char *use_reason = (reason != NULL) ? reason : "unknown";
    uint64_t now_ms = os_sys_time_get();
    bool same_key = false;

    if (s_ble_adv_defer_log_valid &&
        (s_ble_adv_defer_log_kind == (uint8_t)kind) &&
        (s_ble_adv_defer_log_detail == detail) &&
        (s_ble_adv_defer_log_reason != NULL) &&
        (strcmp(s_ble_adv_defer_log_reason, use_reason) == 0))
    {
        same_key = true;
    }

    if (same_key &&
        ((now_ms - s_ble_adv_defer_log_last_ms) < APP_BLE_ADV_DEFER_LOG_PERIOD_MS))
    {
        return false;
    }

    s_ble_adv_defer_log_valid = true;
    s_ble_adv_defer_log_reason = use_reason;
    s_ble_adv_defer_log_kind = (uint8_t)kind;
    s_ble_adv_defer_log_detail = detail;
    s_ble_adv_defer_log_last_ms = now_ms;
    return true;
}

static void app_ble_power_try_start_adv(const char *reason)
{
    if (s_ble_shutdown_latched)
    {
        if (app_ble_power_defer_log_due(reason, APP_BLE_ADV_DEFER_GATE_CLOSED, 1))
        {
            DBG_DIRECT("[SHUTDOWN][BLE] adv_start_blocked reason=%s",
                       (reason != NULL) ? reason : "unknown");
        }
        return;
    }

    if (s_ble_disconnect_restore_pending)
    {
        if (app_ble_power_defer_log_due(
                reason, APP_BLE_ADV_DEFER_DISCONNECT_RESTORE, 0))
        {
            ZY100_BLE_DIAG_LOG("[BLE_ADV] wait reason=%s k=%u d=0",
                            (reason != NULL) ? reason : "unknown",
                            (uint8_t)APP_BLE_ADV_DEFER_DISCONNECT_RESTORE);
        }
        return;
    }

    if (!s_ble_boot_storage_ready)
    {
        if (app_ble_power_defer_log_due(reason,
                                        APP_BLE_ADV_DEFER_BOOT_STORAGE, 0))
        {
            ZY100_BLE_DIAG_LOG("[BLE_ADV] wait reason=%s k=%u d=%d",
                       (reason != NULL) ? reason : "unknown",
                       (uint8_t)APP_BLE_ADV_DEFER_BOOT_STORAGE, 0);
        }
        return;
    }

    if (!s_ble_adv_allowed)
    {
        if (app_ble_power_defer_log_due(reason, APP_BLE_ADV_DEFER_GATE_CLOSED, 0))
        {
            ZY100_BLE_DIAG_LOG("[BLE_ADV] wait reason=%s k=%u d=%d",
                       (reason != NULL) ? reason : "unknown",
                       (uint8_t)APP_BLE_ADV_DEFER_GATE_CLOSED, 0);
        }
        return;
    }

    if (s_ble_sleep_quiescing)
    {
        if (app_ble_power_defer_log_due(reason, APP_BLE_ADV_DEFER_SLEEP_QUIESCING, 0))
        {
            ZY100_BLE_DIAG_LOG("[BLE_ADV] wait reason=%s k=%u d=%d",
                       (reason != NULL) ? reason : "unknown",
                       (uint8_t)APP_BLE_ADV_DEFER_SLEEP_QUIESCING, 0);
        }
        return;
    }

    if ((!s_ble_stack_ready) ||
        (gap_dev_state.gap_init_state != GAP_INIT_STATE_STACK_READY))
    {
        if (app_ble_power_defer_log_due(reason, APP_BLE_ADV_DEFER_STACK_NOT_READY,
                                        (int32_t)gap_dev_state.gap_init_state))
        {
            ZY100_BLE_DIAG_LOG("[BLE_ADV] wait reason=%s k=%u d=%d",
                       (reason != NULL) ? reason : "unknown",
                       (uint8_t)APP_BLE_ADV_DEFER_STACK_NOT_READY,
                       gap_dev_state.gap_init_state);
        }
        return;
    }

    if (gap_conn_state == GAP_CONN_STATE_CONNECTED)
    {
        if (app_ble_power_defer_log_due(reason, APP_BLE_ADV_DEFER_CONNECTED,
                                        (int32_t)s_ble_conn_id))
        {
            ZY100_BLE_DIAG_LOG("[BLE_ADV] wait reason=%s k=%u d=%d",
                       (reason != NULL) ? reason : "unknown",
                       (uint8_t)APP_BLE_ADV_DEFER_CONNECTED,
                       s_ble_conn_id);
        }
        return;
    }

    if (gap_dev_state.gap_adv_state != GAP_ADV_STATE_IDLE)
    {
        if ((gap_dev_state.gap_adv_state == GAP_ADV_STATE_START) ||
            (gap_dev_state.gap_adv_state == GAP_ADV_STATE_ADVERTISING))
        {
            return;
        }

        if (app_ble_power_defer_log_due(reason, APP_BLE_ADV_DEFER_ADV_STATE,
                                        (int32_t)gap_dev_state.gap_adv_state))
        {
            ZY100_BLE_DIAG_LOG("[BLE_ADV] wait reason=%s k=%u d=%d",
                       (reason != NULL) ? reason : "unknown",
                       (uint8_t)APP_BLE_ADV_DEFER_ADV_STATE,
                       gap_dev_state.gap_adv_state);
        }
        return;
    }

    if (s_ble_adv_start_requested)
    {
        if (app_ble_power_defer_log_due(reason, APP_BLE_ADV_DEFER_START_REQUESTED, 0))
        {
            ZY100_BLE_DIAG_LOG("[BLE_ADV] wait reason=%s k=%u d=%d",
                       (reason != NULL) ? reason : "unknown",
                       (uint8_t)APP_BLE_ADV_DEFER_START_REQUESTED, 0);
        }
        return;
    }

#if F_LP_BLE_ADV_STAGE_SWITCH_EN
    DBG_DIRECT("[BLE_PWR] adv start policy (%s)", (reason != NULL) ? reason : "unknown");
    app_lp_adv_policy_on_disconnected();
#else
    {
        T_GAP_CAUSE adv_cause = le_adv_start();
        if ((reason != NULL) && (strcmp(reason, "button_wake") == 0))
        {
            DBG_DIRECT("[BLE_PWR] adv_start button_wake cause=0x%x", adv_cause);
        }
        else
        {
            DBG_DIRECT("[BLE_PWR] adv start (%s): cause=0x%x",
                       (reason != NULL) ? reason : "unknown", adv_cause);
        }
        APP_PRINT_INFO1("GAP adv start cause 0x%x", adv_cause);
        s_ble_adv_start_requested = (adv_cause == GAP_CAUSE_SUCCESS);
        if (adv_cause != GAP_CAUSE_SUCCESS)
        {
            app_device_pairing_window_adv_result(false, true);
            APP_PRINT_ERROR1("GAP adv start failed cause 0x%x", adv_cause);
        }
#if F_BT_DLPS_EN
        app_set_dlps_by_ble_state(true);
#endif
    }
#endif
}

void app_ble_power_set_boot_storage_ready(bool ready)
{
    if (s_ble_boot_storage_ready == ready)
    {
        return;
    }

    s_ble_boot_storage_ready = ready;
    s_ble_adv_defer_log_valid = false;
#if F_LP_BLE_ADV_STAGE_SWITCH_EN
    if (!ready)
    {
        s_lp_adv_switch_pending = false;
        app_lp_adv_stage_timer_stop("boot storage gate");
    }
#endif
    DBG_DIRECT("[BLE_PWR] boot_storage_ready=%u adv=%d conn=%d",
               ready ? 1U : 0U,
               gap_dev_state.gap_adv_state,
               gap_conn_state);
}

void app_ble_power_policy_restart_advertising_if_idle(const char *reason)
{
    app_ble_power_try_start_adv(reason);
}

void app_ble_enter_standby_policy(const char *reason)
{
    const char *use_reason = (reason != NULL) ? reason : "standby";

    if (app_device_pairing_window_active()) return;

    if (s_ble_shutdown_latched)
    {
        (void)app_ble_prepare_for_button_only_dlps();
        return;
    }

    s_ble_adv_allowed = true;
    s_ble_sleep_quiescing = false;
    s_ble_sleep_prepare_logged = false;
    s_ble_sleep_ready_logged = false;
    s_ble_standby_policy_active = true;
    if (gap_dev_state.gap_adv_state == GAP_ADV_STATE_IDLE)
    {
        s_ble_adv_stop_requested = false;
    }
    if (gap_conn_state == GAP_CONN_STATE_DISCONNECTED)
    {
        s_ble_disconnect_requested = false;
        app_ble_standby_apply_adv_policy(use_reason);
    }
    else if (app_ble_power_conn_active())
    {
        app_ble_standby_request_conn_param(s_ble_conn_id, use_reason);
#if F_BT_DLPS_EN
        app_set_dlps_by_ble_state(true);
#endif
    }

    ZY100_LOG_EVENT("[EVT][PWR] standby_enter reason=%s conn=%u",
                    use_reason,
                    app_ble_power_conn_active() ? 1U : 0U);
}

void app_ble_exit_standby_policy_begin(const char *reason)
{
    const char *use_reason = (reason != NULL) ? reason : "standby_exit";

    if (s_ble_shutdown_latched)
    {
        (void)app_ble_prepare_for_button_only_dlps();
        return;
    }

    s_ble_standby_policy_active = false;
    s_ble_adv_allowed = true;
    s_ble_sleep_quiescing = false;
    s_ble_sleep_prepare_logged = false;
    s_ble_sleep_ready_logged = false;
    if (gap_dev_state.gap_adv_state == GAP_ADV_STATE_IDLE)
    {
        s_ble_adv_stop_requested = false;
    }
    if (gap_conn_state == GAP_CONN_STATE_DISCONNECTED)
    {
        s_ble_disconnect_requested = false;
    }
    else if (app_ble_power_conn_active())
    {
#if F_BT_DLPS_EN
        app_set_dlps_by_ble_state(false);
#endif
    }

    ZY100_LOG_EVENT("[EVT][PWR] standby_exit reason=%s conn=%u",
                    use_reason,
                    app_ble_power_conn_active() ? 1U : 0U);
}

void app_ble_exit_standby_policy_complete(const char *reason,
                                          app_ble_ci_state_t target_state)
{
    const char *use_reason = (reason != NULL) ? reason : "standby_exit";

    if (s_ble_shutdown_latched)
    {
        (void)app_ble_prepare_for_button_only_dlps();
        return;
    }

    if ((target_state <= APP_BLE_CI_STATE_DISCONNECTED) ||
        (target_state >= APP_BLE_CI_STATE_COUNT))
    {
        target_state = APP_BLE_CI_STATE_ACTIVE_IDLE;
    }

    if (gap_conn_state == GAP_CONN_STATE_DISCONNECTED)
    {
        if (s_ble_disconnect_restore_pending)
        {
            s_ble_disconnect_restore_pending = false;
            DBG_DIRECT("[BLE_STANDBY] disconnect_restore_ready reason=%s",
                       use_reason);
        }
        app_ble_standby_restore_adv_policy(use_reason);
    }
    else if (app_ble_power_conn_active())
    {
        /*
         * Submit the semantic link target only after clock, Flash, sensor and
         * runtime-service restoration.  WINDOWS_CENTRAL direct START may
         * therefore request ONLINE_HIGH without an intermediate
         * ACTIVE_IDLE/Balanced profile.  PERIPHERAL_EXACT callers keep their
         * existing CI48 -> CI9 follow-up by passing ACTIVE_IDLE here.
         */
        app_ble_standby_restore_conn_param(s_ble_conn_id,
                                           target_state,
                                           use_reason);
    }

    ZY100_LOG_EVENT("[EVT][PWR] standby_ready reason=%s target=%s",
                    use_reason,
                    app_ble_ci_state_name(target_state));
}

void app_ble_power_on_wake(void)
{
    if (s_ble_shutdown_latched)
    {
        DBG_DIRECT("[SHUTDOWN][BLE] wake_open_blocked latch=1");
        (void)app_ble_prepare_for_button_only_dlps();
        return;
    }

    if (gap_dev_state.gap_adv_state == GAP_ADV_STATE_IDLE)
    {
        s_ble_adv_stop_requested = false;
    }
    if (gap_conn_state == GAP_CONN_STATE_DISCONNECTED)
    {
        s_ble_disconnect_requested = false;
    }
    s_ble_adv_allowed = true;
    s_ble_sleep_quiescing = false;
    s_ble_sleep_prepare_logged = false;
    s_ble_sleep_ready_logged = false;
    s_ble_standby_policy_active = false;
    app_ble_standby_clear_conn_param();
    if (app_ble_power_wake_log_due(&s_ble_wake_open_log, "button_wake",
                                   (uint8_t)s_ble_adv_allowed))
    {
        DBG_DIRECT("[BLE_SLEEP] button_wake restart_ble=1");
        DBG_DIRECT("[BLE_PWR] wake: gate open stack_ready=%u init=%d adv=%d conn=%d conn_id=%u start_req=%u",
                   (uint8_t)s_ble_stack_ready,
                   gap_dev_state.gap_init_state,
                   gap_dev_state.gap_adv_state,
                   gap_conn_state,
                   s_ble_conn_id,
                   (uint8_t)s_ble_adv_start_requested);
    }
    if (s_ble_stack_ready &&
        (gap_dev_state.gap_init_state == GAP_INIT_STATE_STACK_READY) &&
        (gap_conn_state == GAP_CONN_STATE_DISCONNECTED) &&
        (gap_dev_state.gap_adv_state == GAP_ADV_STATE_IDLE))
    {
        (void)app_ble_set_adv_interval((uint16_t)ZY100_BLE_ACTIVE_ADV_INTERVAL_UNITS,
                                       "button_wake");
    }
    app_ble_power_try_start_adv("button_wake");
}

void app_ble_power_on_wake_hold_adv(const char *reason)
{
    const char *use_reason = (reason != NULL) ? reason : "wake_hold_adv";
    bool conn_active = app_ble_power_conn_active();
    bool adv_active = false;
    bool preserve_adv_gate = false;

    if (s_ble_shutdown_latched)
    {
        DBG_DIRECT("[SHUTDOWN][BLE] wake_hold_blocked reason=%s",
                   use_reason);
        (void)app_ble_prepare_for_button_only_dlps();
        return;
    }

    if (gap_dev_state.gap_adv_state == GAP_ADV_STATE_IDLE)
    {
        s_ble_adv_stop_requested = false;
    }
    if (gap_conn_state == GAP_CONN_STATE_DISCONNECTED)
    {
        s_ble_disconnect_requested = false;
    }
    adv_active = app_ble_power_adv_state_active();
    preserve_adv_gate = conn_active || adv_active || s_ble_adv_stop_requested;
    s_ble_adv_allowed = preserve_adv_gate ? true : false;
    s_ble_sleep_quiescing = false;
    s_ble_sleep_prepare_logged = false;
    s_ble_sleep_ready_logged = false;
    if (app_ble_power_wake_log_due(&s_ble_wake_hold_log, use_reason,
                                   (uint8_t)s_ble_adv_allowed))
    {
        if (conn_active)
        {
            DBG_DIRECT("[BLE_SLEEP] hold_adv_skip_connected reason=%s", use_reason);
        }
        else if (preserve_adv_gate)
        {
            DBG_DIRECT("[BLE_SLEEP] hold_adv_preserve_active_adv reason=%s adv=%d stop_req=%u",
                       use_reason,
                       gap_dev_state.gap_adv_state,
                       (uint8_t)s_ble_adv_stop_requested);
        }
        else
        {
            DBG_DIRECT("[BLE_SLEEP] button_wake hold_adv=1 reason=%s", use_reason);
        }
        DBG_DIRECT("[BLE_PWR] wake_hold g=%u sr=%u i=%d a=%d c=%d id=%u req=%u on=%u aa=%u stop=%u",
                   (uint8_t)s_ble_adv_allowed,
                   (uint8_t)s_ble_stack_ready,
                   gap_dev_state.gap_init_state,
                   gap_dev_state.gap_adv_state,
                   gap_conn_state,
                   s_ble_conn_id,
                   (uint8_t)s_ble_adv_start_requested,
                   conn_active ? 1U : 0U,
                   adv_active ? 1U : 0U,
                   (uint8_t)s_ble_adv_stop_requested);
    }
}

bool app_ble_prepare_for_button_only_dlps(void)
{
    bool adv_active = app_ble_power_adv_state_active();
    bool conn_active = app_ble_power_conn_active();
    bool was_quiescing = s_ble_sleep_quiescing;

    s_ble_adv_allowed = false;
    s_ble_sleep_quiescing = true;
    s_ble_standby_policy_active = false;
    s_ble_disconnect_restore_pending = false;
    app_ble_standby_clear_conn_param();
    if (!was_quiescing)
    {
        s_ble_sleep_prepare_logged = false;
        s_ble_sleep_ready_logged = false;
    }
    if (!s_ble_sleep_prepare_logged)
    {
        DBG_DIRECT("[BLE_SLEEP] prepare_button_only conn=%u adv=%u closing=%u",
                   conn_active ? 1U : 0U,
                   adv_active ? 1U : 0U,
                   (uint8_t)s_ble_sleep_quiescing);
        s_ble_sleep_prepare_logged = true;
    }

    if (conn_active)
    {
        if (!s_ble_disconnect_requested)
        {
            T_GAP_CAUSE cause = le_disconnect(s_ble_conn_id);
            app_ble_ci_state_snapshot_t link_snapshot;

            DBG_DIRECT("[BLE_SLEEP] disconnect_for_sleep conn_id=%u cause=0x%x",
                       s_ble_conn_id, cause);
            s_ble_disconnect_requested = (cause == GAP_CAUSE_SUCCESS);
            app_ble_ci_state_get_snapshot(&link_snapshot);
            app_ble_link_trace_note_local_disconnect(
                &link_snapshot,
                (uint16_t)cause,
                s_ble_disconnect_requested);
        }
        return false;
    }

    if (gap_dev_state.gap_adv_state == GAP_ADV_STATE_START)
    {
        /*
         * The stack rejects le_adv_stop() while advertising is still in its
         * START transition. Keep advertising disallowed and wait for the GAP
         * state callback to report ADVERTISING before issuing one stop.
         */
        if (s_ble_shutdown_latched)
        {
            DBG_DIRECT("[SHUTDOWN][BLE] adv_start_race state=%d action=wait_then_stop",
                       gap_dev_state.gap_adv_state);
        }
        return false;
    }

    if (gap_dev_state.gap_adv_state == GAP_ADV_STATE_ADVERTISING)
    {
        if (!s_ble_adv_stop_requested)
        {
            T_GAP_CAUSE cause = le_adv_stop();
            DBG_DIRECT("[BLE_SLEEP] stop_adv_for_sleep cause=0x%x", cause);
            s_ble_adv_stop_requested = (cause == GAP_CAUSE_SUCCESS);
        }
        return false;
    }

    return app_ble_power_is_quiesced();
}

void app_ble_power_shutdown_latch(void)
{
    bool first_latch = !s_ble_shutdown_latched;

    s_ble_shutdown_latched = true;
    s_ble_adv_allowed = false;
    s_ble_sleep_quiescing = true;
    s_ble_standby_policy_active = false;
    s_ble_disconnect_restore_pending = false;
    app_ble_standby_clear_conn_param();
#if F_LP_BLE_ADV_STAGE_SWITCH_EN
    s_lp_adv_switch_pending = false;
    app_lp_adv_stage_timer_stop("shutdown_latch");
#endif
    if (first_latch)
    {
        s_ble_sleep_prepare_logged = false;
        s_ble_sleep_ready_logged = false;
        DBG_DIRECT("[SHUTDOWN][BLE] latch gate=0 conn=%u adv=%u",
                   app_ble_power_conn_active() ? 1U : 0U,
                   app_ble_power_adv_state_active() ? 1U : 0U);
    }
    (void)app_ble_prepare_for_button_only_dlps();
}

void app_ble_power_shutdown_release_on_wake(void)
{
    if (!s_ble_shutdown_latched)
    {
        return;
    }

    s_ble_shutdown_latched = false;
    s_ble_sleep_quiescing = false;
    s_ble_sleep_prepare_logged = false;
    s_ble_sleep_ready_logged = false;
    DBG_DIRECT("[SHUTDOWN][BLE] release reason=authorized_button_wake");
}

bool app_ble_power_shutdown_latched(void)
{
    return s_ble_shutdown_latched;
}

bool app_ble_prepare_for_ota_quiesce(void)
{
    bool adv_active = app_ble_power_adv_state_active();
    bool conn_active = app_ble_power_conn_active();
    bool was_quiescing = s_ble_sleep_quiescing;

    s_ble_adv_allowed = false;
    s_ble_sleep_quiescing = true;
    s_ble_disconnect_restore_pending = false;
#if F_LP_BLE_ADV_STAGE_SWITCH_EN
    s_lp_adv_switch_pending = false;
    app_lp_adv_stage_timer_stop("ota_quiesce");
#endif
    if (!was_quiescing)
    {
        s_ble_sleep_prepare_logged = false;
        s_ble_sleep_ready_logged = false;
    }
    if (!s_ble_sleep_prepare_logged)
    {
        DBG_DIRECT("[BLE_SLEEP] prepare_ota conn=%u adv=%u closing=%u",
                   conn_active ? 1U : 0U,
                   adv_active ? 1U : 0U,
                   (uint8_t)s_ble_sleep_quiescing);
        s_ble_sleep_prepare_logged = true;
    }

    if (conn_active)
    {
        if (!s_ble_disconnect_requested)
        {
            T_GAP_CAUSE cause = le_disconnect(s_ble_conn_id);
            DBG_DIRECT("[BLE_SLEEP] disconnect_for_ota conn_id=%u cause=0x%x",
                       s_ble_conn_id, cause);
            s_ble_disconnect_requested = (cause == GAP_CAUSE_SUCCESS);
        }
        return false;
    }

    if (gap_dev_state.gap_adv_state == GAP_ADV_STATE_START)
    {
        return false;
    }

    if (gap_dev_state.gap_adv_state == GAP_ADV_STATE_ADVERTISING)
    {
        if (!s_ble_adv_stop_requested)
        {
            T_GAP_CAUSE cause = le_adv_stop();
            DBG_DIRECT("[BLE_SLEEP] stop_adv_for_ota cause=0x%x", cause);
            s_ble_adv_stop_requested = (cause == GAP_CAUSE_SUCCESS);
        }
        return false;
    }

    return app_ble_power_is_quiesced();
}

void app_ble_recover_after_ota_cancel(void)
{
    if (s_ble_shutdown_latched)
    {
        DBG_DIRECT("[SHUTDOWN][BLE] ota_recover_blocked latch=1");
        (void)app_ble_prepare_for_button_only_dlps();
        return;
    }

    s_ble_sleep_quiescing = false;
    s_ble_sleep_prepare_logged = false;
    s_ble_sleep_ready_logged = false;
    s_ble_disconnect_requested = false;
    s_ble_adv_stop_requested = false;
    s_ble_adv_allowed = true;
    app_ble_power_try_start_adv("ota_cancel");
}

void app_ble_recover_after_button_only_sleep_fail(void)
{
    if (s_ble_shutdown_latched)
    {
        DBG_DIRECT("[SHUTDOWN][BLE] sleep_fail_recover_blocked latch=1");
        (void)app_ble_prepare_for_button_only_dlps();
        return;
    }

    s_ble_sleep_quiescing = false;
    s_ble_sleep_prepare_logged = false;
    s_ble_sleep_ready_logged = false;
    s_ble_adv_allowed = true;
    if (gap_dev_state.gap_adv_state == GAP_ADV_STATE_IDLE)
    {
        s_ble_adv_stop_requested = false;
    }
    if (gap_conn_state == GAP_CONN_STATE_DISCONNECTED)
    {
        s_ble_disconnect_requested = false;
    }
    DBG_DIRECT("[BLE_SLEEP] recover_adv_after_sleep_fail");
    app_ble_power_try_start_adv("sleep_fail");
}

bool app_ble_power_is_quiesced(void)
{
    bool adv_idle = (gap_dev_state.gap_adv_state == GAP_ADV_STATE_IDLE);
    bool disconnected = (gap_conn_state == GAP_CONN_STATE_DISCONNECTED);
    bool sleep_blocking_op =
        s_ble_stack_ready &&
        (gap_dev_state.gap_init_state != GAP_INIT_STATE_STACK_READY);
    bool quiesced = (adv_idle && disconnected &&
                     (!s_ble_adv_start_requested) &&
                     (!s_ble_adv_stop_requested) &&
                     (!s_ble_disconnect_requested) &&
                     (!sleep_blocking_op));

    if (quiesced && s_ble_sleep_quiescing && (!s_ble_sleep_ready_logged))
    {
        DBG_DIRECT("[BLE_SLEEP] quiesced conn=0 adv=0 adv_start_pending=0 adv_stop_pending=0 disc_pending=0");
        DBG_DIRECT("[BLE_SLEEP] ready_button_only conn=0 adv=0");
        s_ble_sleep_ready_logged = true;
    }

    return quiesced;
}

void app_ble_power_on_stack_ready(void)
{
    s_ble_stack_ready = true;
    DBG_DIRECT("[BLE_PWR] stack ready: gate=%u storage=%u sleep=%u adv=%d conn=%d",
               (uint8_t)s_ble_adv_allowed,
               s_ble_boot_storage_ready ? 1U : 0U,
               (uint8_t)s_ble_sleep_quiescing,
               gap_dev_state.gap_adv_state,
               gap_conn_state);
    app_ble_power_try_start_adv("stack ready");
}


T_GAP_CONN_STATE app_ble_power_policy_note_conn_state(
    T_GAP_CONN_STATE new_state)
{
    T_GAP_CONN_STATE old_state = gap_conn_state;

    gap_conn_state = new_state;
    return old_state;
}

T_GAP_CONN_STATE app_ble_power_policy_conn_state(void)
{
    return gap_conn_state;
}

bool app_ble_power_policy_standby_active(void)
{
    return s_ble_standby_policy_active;
}

void app_ble_power_policy_disconnect_if_required(void)
{
    T_GAP_CAUSE cause;
    app_ble_ci_state_snapshot_t link_snapshot;

    if (!app_ble_ci_state_disconnect_required() ||
        !app_ble_power_conn_active() ||
        s_ble_disconnect_requested)
    {
        return;
    }

    cause = le_disconnect(s_ble_conn_id);
    s_ble_disconnect_requested = (cause == GAP_CAUSE_SUCCESS);
    app_ble_ci_state_get_snapshot(&link_snapshot);
    app_ble_link_trace_note_local_disconnect(
        &link_snapshot,
        (uint16_t)cause,
        s_ble_disconnect_requested);
    DBG_DIRECT("[BLE_CI_HOST] disconnect_required conn=%u cause=0x%x submitted=%u",
               s_ble_conn_id,
               cause,
               s_ble_disconnect_requested ? 1U : 0U);
    if (s_ble_disconnect_requested)
    {
        app_ble_ci_state_disconnect_requested();
    }
}

void app_ble_power_policy_on_connected(uint8_t conn_id)
{
    app_device_pairing_note_connected();
#if F_LP_BLE_ADV_STAGE_SWITCH_EN
    app_lp_adv_policy_on_connected();
#else
#if F_BT_DLPS_EN
    if (s_ble_standby_policy_active)
    {
        app_set_dlps_by_ble_state(true);
    }
    else
    {
        /* Wake and stay active right after link-up for current measurement split. */
        app_set_dlps_by_ble_state(false);
    }
#endif
#endif

    s_ble_conn_id = conn_id;
    app_ble_notify_tracker_reset_conn(conn_id);
    s_ble_disconnect_requested = false;
    s_ble_disconnect_restore_pending = false;
    DBG_DIRECT("[BLE_PWR] connected: conn_id=%u", s_ble_conn_id);
    if (s_ble_shutdown_latched)
    {
        DBG_DIRECT("[SHUTDOWN][BLE] connected_after_latch conn=%u action=handoff_disconnect",
                   conn_id);
    }
}

void app_ble_power_policy_on_disconnected(
    uint8_t conn_id, app_ble_disconnect_recovery_t recovery)
{
    bool adv_gate_reopened = false;

    app_ble_notify_tracker_reset_conn(conn_id);
    if ((s_ble_conn_id == conn_id) || (APP_MAX_LINKS == 1))
    {
        s_ble_conn_id = APP_BLE_SENSOR_CONN_ID_INVALID;
    }
    s_ble_disconnect_requested = false;
    app_ble_standby_clear_conn_param();
    app_ble_conn_param_mgr_reset("disconnected");

    if ((!s_ble_shutdown_latched) &&
        (!s_ble_sleep_quiescing) && (!s_ble_adv_allowed))
    {
        s_ble_adv_allowed = true;
        adv_gate_reopened = true;
    }

    if (adv_gate_reopened)
    {
        DBG_DIRECT("[BLE_PWR] disconnect reopen_adv_gate conn_id=%u standby=%u adv=%d",
                   conn_id,
                   s_ble_standby_policy_active ? 1U : 0U,
                   gap_dev_state.gap_adv_state);
    }
    DBG_DIRECT("[BLE_PWR] disconnected: conn_id=%u gate=%u sleep=%u adv=%d",
               conn_id,
               (uint8_t)s_ble_adv_allowed,
               (uint8_t)s_ble_sleep_quiescing,
               gap_dev_state.gap_adv_state);

    if (s_ble_sleep_quiescing)
    {
        DBG_DIRECT("[BLE_SLEEP] disconnected_for_sleep no_adv_restart=1");
        return;
    }

    if (recovery == APP_BLE_DISCONNECT_RECOVERY_DEFER_ACTIVE_RESTORE)
    {
        s_ble_disconnect_restore_pending = true;
        DBG_DIRECT("[BLE_STANDBY] disconnect_restore_deferred conn_id=%u",
                   conn_id);
        return;
    }

    if (s_ble_standby_policy_active)
    {
        app_ble_standby_apply_adv_policy("standby_disconnect");
    }
    else
    {
        app_ble_power_try_start_adv("disconnect");
    }
}

void app_ble_power_policy_complete_deferred_disconnect_restore(
    const char *reason)
{
    const char *use_reason = (reason != NULL) ? reason :
                             "disconnect_restore_recovered";

    if (!s_ble_disconnect_restore_pending)
    {
        return;
    }
    if (s_ble_shutdown_latched || s_ble_sleep_quiescing)
    {
        s_ble_disconnect_restore_pending = false;
        DBG_DIRECT("[BLE_STANDBY] disconnect_restore_cancelled reason=%s shutdown=%u sleep=%u",
                   use_reason,
                   s_ble_shutdown_latched ? 1U : 0U,
                   s_ble_sleep_quiescing ? 1U : 0U);
        return;
    }

    app_ble_exit_standby_policy_begin(use_reason);
    app_ble_exit_standby_policy_complete(use_reason,
                                         APP_BLE_CI_STATE_ACTIVE_IDLE);
}

void app_handle_dev_state_evt(T_GAP_DEV_STATE new_state, uint16_t cause)
{
    T_GAP_DEV_STATE old_state = gap_dev_state;

    ZY100_BLE_DIAG_LOG("[BLE_GAP] dev old=%d/%d new=%d/%d sub=%d cause=0x%x gate=%u ready=%u req=%u",
               old_state.gap_init_state,
               old_state.gap_adv_state,
               new_state.gap_init_state,
               new_state.gap_adv_state,
               new_state.gap_adv_sub_state,
               cause,
               s_ble_adv_allowed ? 1U : 0U,
               s_ble_stack_ready ? 1U : 0U,
               s_ble_adv_start_requested ? 1U : 0U);
    ZY100_LOG_VERBOSE("[APP_GAP] dev_state: init=%d adv=%d sub=%d cause=0x%x",
                      new_state.gap_init_state, new_state.gap_adv_state,
                      new_state.gap_adv_sub_state, cause);
    APP_PRINT_INFO3("app_handle_dev_state_evt: init state %d, adv state %d, cause 0x%x",
                    new_state.gap_init_state, new_state.gap_adv_state, cause);
    gap_dev_state = new_state;

    if (old_state.gap_init_state != new_state.gap_init_state)
    {
        if (new_state.gap_init_state == GAP_INIT_STATE_STACK_READY)
        {
            ZY100_LOG_EVENT("[BLE_BOOT] stack_ready cause=0x%x adv=%d conn=%d",
                       cause,
                       new_state.gap_adv_state,
                       gap_conn_state);
            APP_PRINT_INFO0("GAP stack ready");
            /*stack ready*/
#if F_LP_BLE_ADV_STAGE_SWITCH_EN
            app_lp_adv_stage_init();
#endif
            app_ble_power_on_stack_ready();
        }
    }

    if (old_state.gap_adv_state != new_state.gap_adv_state)
    {
        app_device_pairing_window_adv_result(
            !s_ble_shutdown_latched && cause == 0U &&
            (new_state.gap_adv_state == GAP_ADV_STATE_ADVERTISING ||
             (new_state.gap_adv_state == GAP_ADV_STATE_IDLE &&
              new_state.gap_adv_sub_state == GAP_ADV_TO_IDLE_CAUSE_CONN)),
            !s_ble_shutdown_latched && cause != 0U);
        if (new_state.gap_adv_state == GAP_ADV_STATE_IDLE)
        {
            ZY100_LOG_EVENT("[BLE_ADV] stopped cause=0x%x sub=%d",
                            cause, new_state.gap_adv_sub_state);
            s_ble_adv_start_requested = false;
            s_ble_adv_stop_requested = false;
            if (s_ble_sleep_quiescing)
            {
                DBG_DIRECT("[BLE_SLEEP] adv_stopped_for_sleep");
            }
            if (new_state.gap_adv_sub_state == GAP_ADV_TO_IDLE_CAUSE_CONN)
            {
                APP_PRINT_INFO0("GAP adv stoped: because connection created");
            }
            else
            {
                APP_PRINT_INFO0("GAP adv stoped");
            }
#if F_LP_BLE_ADV_STAGE_SWITCH_EN
            app_lp_adv_apply_pending_stage();
#endif
            if (s_ble_standby_policy_active)
            {
                app_ble_standby_apply_adv_policy("standby_adv_idle");
            }
            else if (s_ble_adv_allowed && !s_ble_sleep_quiescing &&
                     (gap_conn_state == GAP_CONN_STATE_DISCONNECTED))
            {
                app_ble_standby_restore_adv_policy("adv_idle_active");
            }
        }
        else if (new_state.gap_adv_state == GAP_ADV_STATE_ADVERTISING)
        {
            s_ble_adv_start_requested = false;
            if (s_ble_shutdown_latched)
            {
                DBG_DIRECT("[SHUTDOWN][BLE] adv_start_race active cause=0x%x sub=%d action=stop",
                           cause,
                           new_state.gap_adv_sub_state);
            }
            else
            {
                if (cause == 0U)
                {
                    zy100_whole_unit_first_user_boot_note_advertising();
                }
                ZY100_LOG_EVENT("[BLE_ADV] active cause=0x%x sub=%d",
                                cause,
                                new_state.gap_adv_sub_state);
            }
            APP_PRINT_INFO0("GAP adv start");
#if F_BT_DLPS_EN
            /* Keep advertising connectable while allowing DLPS for low-power test. */
            app_set_dlps_by_ble_state(true);
#endif
        }
    }

    if (s_ble_sleep_quiescing && app_ble_power_adv_state_active() && (!s_ble_adv_stop_requested))
    {
        DBG_DIRECT("[BLE_PWR] sleep quiescing saw active adv state=%d, request stop",
                   gap_dev_state.gap_adv_state);
        (void)app_ble_prepare_for_button_only_dlps();
    }
}

bool app_ble_power_stack_ready(void)
{
    return s_ble_stack_ready && gap_dev_state.gap_init_state == GAP_INIT_STATE_STACK_READY;
}
