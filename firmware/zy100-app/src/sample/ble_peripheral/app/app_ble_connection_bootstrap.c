#include "app_ble_connection_bootstrap.h"

#include <string.h>

#include <os_sched.h>

#include "../app_task.h"
#include "app_ble_ci_state.h"
#include "app_ble_ctrl_dispatcher_port.h"
#include "app_ble_offline_v2_sync.h"
#include "../service/zy100_ble_ctrl_protocol.h"
#include "../service/zy100_ble_ctrl_service.h"
#include "../service/zy100_calibration_manager.h"
#include "../service/zy100_offline_v2_capture.h"
#include "../service/zy100_online_stream.h"
#include "../storage/zy100_offline_v2_storage.h"

#define APP_BLE_BOOTSTRAP_CONN_ID_INVALID 0xFFU
#define APP_BLE_BOOTSTRAP_HOST_DEADLINE_MS 15000UL
#define APP_BLE_BOOTSTRAP_DISCONNECT_GRACE_MS 500UL
#define APP_BLE_BOOTSTRAP_REASON_UPGRADE_HOST 1U

typedef struct
{
    bool connected;
    bool queried;
    bool secure_ready;
    bool host_ci_v3_seen;
    bool user_sync_ready;
    bool feature_config_ready;
    bool recovery_only;
    bool upgrade_snapshot_sent;
    uint8_t conn_id;
    uint8_t revision;
    uint8_t recovery_stage;
    uint8_t recovery_reason;
    uint16_t recovery_retry_ms;
    uint32_t generation;
    uint32_t user_id;
    uint32_t host_deadline_ms;
    uint32_t disconnect_after_ms;
    bool snapshot_valid;
    uint8_t snapshot_overall;
    uint8_t snapshot_device_state;
    uint8_t snapshot_action;
    uint32_t snapshot_ready_bits;
    uint32_t snapshot_detail;
} app_ble_bootstrap_state_t;

static app_ble_bootstrap_state_t s_bootstrap;
static uint32_t s_next_generation = 0U;

static bool app_ble_bootstrap_ack_ready(uint8_t conn_id)
{
    return s_bootstrap.connected && (s_bootstrap.conn_id == conn_id) &&
           zy100_ble_ctrl_service_ack_notify_enabled(conn_id);
}

static bool app_ble_bootstrap_deadline_expired(uint32_t now_ms,
                                               uint32_t deadline_ms)
{
    return (deadline_ms != 0U) && ((int32_t)(now_ms - deadline_ms) >= 0);
}

static void app_ble_bootstrap_arm_host_deadline(void)
{
    if (zy100_offline_v2_capture_active())
    {
        return;
    }
    if (app_ble_bootstrap_ack_ready(s_bootstrap.conn_id) &&
        s_bootstrap.secure_ready && (s_bootstrap.host_deadline_ms == 0U) &&
        !s_bootstrap.host_ci_v3_seen)
    {
        s_bootstrap.host_deadline_ms =
            (uint32_t)os_sys_time_get() + APP_BLE_BOOTSTRAP_HOST_DEADLINE_MS;
    }
}

static uint32_t app_ble_bootstrap_ready_bits_for_link(uint8_t conn_id, bool restoring_link)
{
    uint32_t bits = 0U;
    bool link_policy_ready;
    bool control_ready;
    bool offline_ready;
    bool online_ready;

    if (s_bootstrap.secure_ready) bits |= ZY100_BLE_CONNECTION_READY_SECURE;
    if (app_ble_bootstrap_ack_ready(conn_id))
        bits |= ZY100_BLE_CONNECTION_READY_ACK_NOTIFY;
    link_policy_ready = s_bootstrap.host_ci_v3_seen &&
        (restoring_link || app_ble_ci_state_target_reached(app_ble_ci_state_desired()));
    if (link_policy_ready) bits |= ZY100_BLE_CONNECTION_READY_LINK_POLICY;
    if (s_bootstrap.user_sync_ready)
        bits |= ZY100_BLE_CONNECTION_READY_USER_SYNC;
    if (s_bootstrap.feature_config_ready)
        bits |= ZY100_BLE_CONNECTION_READY_FEATURE_CONFIG;
    if (zy100_cal_manager_business_ready(conn_id))
        bits |= ZY100_BLE_CONNECTION_READY_CALIBRATION;
    if (zy100_ble_ctrl_service_export_notify_enabled(conn_id))
        bits |= ZY100_BLE_CONNECTION_READY_EXPORT_NOTIFY;
    if (app_ble_time_sync_ok()) bits |= ZY100_BLE_CONNECTION_READY_TIME_SYNC;
    offline_ready =
        (app_ble_offline_v2_sync_arb_state() == APP_BLE_OFFLINE_ARB_ONLINE_AVAILABLE) &&
        !app_ble_offline_v2_sync_active() &&
        (app_ble_offline_v2_sync_pending_count() == 0U);
    if (offline_ready) bits |= ZY100_BLE_CONNECTION_READY_OFFLINE_GATE;
    /* TX-ring reuse is expected to change while an online session is active.
     * Keep Bootstrap capability stable until the session enters cleanup. */
    online_ready = zy100_online_stream_active() ||
                   zy100_online_stream_ready(conn_id);
    if (online_ready) bits |= ZY100_BLE_CONNECTION_READY_ONLINE_STREAM;
    if (app_task_device_logic_ready()) bits |= ZY100_BLE_CONNECTION_READY_DEVICE_LOGIC;
    if (zy100_offline_v2_capture_active())
    {
        bits |= ZY100_BLE_CONNECTION_READY_OFFLINE_CAPTURE_OBSERVER;
    }
    else
    {
        uint32_t index;
        uint32_t count = app_ble_offline_v2_sync_pending_count();

        for (index = 0U; index < count; index++)
        {
            zy100_offline_v2_session_info_t info;

            if (zy100_offline_v2_storage_session_get_for_user(
                    s_bootstrap.user_id, index, &info) &&
                (info.health != ZY100_OFFLINE_V2_HEALTH_NORMAL))
            {
                bits |= ZY100_BLE_CONNECTION_READY_OFFLINE_SESSION_ATTENTION;
                break;
            }
        }
    }

    control_ready =
        ((bits & (ZY100_BLE_CONNECTION_READY_SECURE |
                  ZY100_BLE_CONNECTION_READY_ACK_NOTIFY |
                  ZY100_BLE_CONNECTION_READY_LINK_POLICY |
                  ZY100_BLE_CONNECTION_READY_USER_SYNC |
                  ZY100_BLE_CONNECTION_READY_CALIBRATION |
                  ZY100_BLE_CONNECTION_READY_EXPORT_NOTIFY |
                  ZY100_BLE_CONNECTION_READY_TIME_SYNC |
                  ZY100_BLE_CONNECTION_READY_DEVICE_LOGIC)) ==
         (ZY100_BLE_CONNECTION_READY_SECURE |
          ZY100_BLE_CONNECTION_READY_ACK_NOTIFY |
          ZY100_BLE_CONNECTION_READY_LINK_POLICY |
          ZY100_BLE_CONNECTION_READY_USER_SYNC |
          ZY100_BLE_CONNECTION_READY_CALIBRATION |
          ZY100_BLE_CONNECTION_READY_EXPORT_NOTIFY |
          ZY100_BLE_CONNECTION_READY_TIME_SYNC |
          ZY100_BLE_CONNECTION_READY_DEVICE_LOGIC));
    if (control_ready) bits |= ZY100_BLE_CONNECTION_READY_CONTROL;
    if (control_ready && offline_ready && online_ready &&
        s_bootstrap.feature_config_ready)
        bits |= ZY100_BLE_CONNECTION_READY_BUSINESS;
    return bits;
}

static uint32_t app_ble_bootstrap_ready_bits(uint8_t conn_id)
{
    return app_ble_bootstrap_ready_bits_for_link(conn_id, false);
}

bool app_ble_connection_bootstrap_business_ready_except_link(uint8_t conn_id)
{
    return s_bootstrap.connected && (s_bootstrap.conn_id == conn_id) &&
           !s_bootstrap.recovery_only &&
           ((app_ble_bootstrap_ready_bits_for_link(conn_id, true) &
             ZY100_BLE_CONNECTION_READY_BUSINESS) != 0U);
}

uint32_t app_ble_connection_bootstrap_generation(void)
{
    return s_bootstrap.generation;
}


static uint8_t app_ble_bootstrap_stage_from_bits(uint32_t bits)
{
    if ((bits & ZY100_BLE_CONNECTION_READY_SECURE) == 0U)
        return APP_BLE_BOOTSTRAP_STAGE_SECURITY;
    if ((bits & ZY100_BLE_CONNECTION_READY_LINK_POLICY) == 0U)
        return APP_BLE_BOOTSTRAP_STAGE_LINK_POLICY;
    if ((bits & ZY100_BLE_CONNECTION_READY_USER_SYNC) == 0U)
        return APP_BLE_BOOTSTRAP_STAGE_USER_SYNC;
    if ((bits & ZY100_BLE_CONNECTION_READY_CALIBRATION) == 0U)
        return APP_BLE_BOOTSTRAP_STAGE_CALIBRATION;
    if ((bits & ZY100_BLE_CONNECTION_READY_EXPORT_NOTIFY) == 0U)
        return APP_BLE_BOOTSTRAP_STAGE_EXPORT;
    if ((bits & ZY100_BLE_CONNECTION_READY_TIME_SYNC) == 0U)
        return APP_BLE_BOOTSTRAP_STAGE_TIME_SYNC;
    if ((bits & ZY100_BLE_CONNECTION_READY_CONTROL) == 0U)
        return APP_BLE_BOOTSTRAP_STAGE_CONTROL_READY;
    if ((bits & ZY100_BLE_CONNECTION_READY_OFFLINE_GATE) == 0U)
        return APP_BLE_BOOTSTRAP_STAGE_OFFLINE_SYNC;
    if ((bits & ZY100_BLE_CONNECTION_READY_ONLINE_STREAM) == 0U)
        return APP_BLE_BOOTSTRAP_STAGE_ONLINE_READY;
    if ((bits & ZY100_BLE_CONNECTION_READY_FEATURE_CONFIG) == 0U)
        return APP_BLE_BOOTSTRAP_STAGE_FEATURE_CONFIG;
    return APP_BLE_BOOTSTRAP_STAGE_BUSINESS_READY;
}

static bool app_ble_bootstrap_publish(uint8_t conn_id, bool force)
{
    uint32_t bits;
    uint8_t overall;
    uint8_t action;
    uint8_t stage;
    uint8_t reason = 0U;
    uint16_t retry_ms = 0U;
    uint8_t device_state;
    uint8_t candidate_revision;
    uint32_t detail;
    bool queued;

    if (!s_bootstrap.connected || (s_bootstrap.conn_id != conn_id) ||
        !app_ble_bootstrap_ack_ready(conn_id))
    {
        return false;
    }
    bits = app_ble_bootstrap_ready_bits(conn_id);
    stage = app_ble_bootstrap_stage_from_bits(bits);
    overall = ZY100_BLE_CONNECTION_BOOTSTRAPPING;
    action = ZY100_BLE_CONNECTION_ACTION_WAIT;
    device_state = app_task_ble_ctrl_device_state();
    if (zy100_offline_v2_capture_active())
    {
        overall = ZY100_BLE_CONNECTION_DEFERRED_OFFLINE_CAPTURE;
        action = ZY100_BLE_CONNECTION_ACTION_OBSERVE_AND_WAIT;
        stage = APP_BLE_BOOTSTRAP_STAGE_OFFLINE_CAPTURE_OBSERVER;
        retry_ms = 0U;
    }
    else if (s_bootstrap.recovery_only)
    {
        overall = ZY100_BLE_CONNECTION_RECOVERY_ONLY;
        action = ZY100_BLE_CONNECTION_ACTION_RETRY_STAGE;
        stage = s_bootstrap.recovery_stage;
        reason = s_bootstrap.recovery_reason;
        retry_ms = s_bootstrap.recovery_retry_ms;
    }
    else if ((bits & ZY100_BLE_CONNECTION_READY_BUSINESS) != 0U)
    {
        overall = ZY100_BLE_CONNECTION_BUSINESS_READY;
    }
    else if ((bits & ZY100_BLE_CONNECTION_READY_CONTROL) != 0U)
    {
        overall = ZY100_BLE_CONNECTION_CONTROL_READY;
    }
    if (!zy100_offline_v2_capture_active() &&
        (device_state == ZY100_BLE_DEVICE_STATE_OFFLINE_SESSION_READY))
    {
        uint32_t count = app_ble_offline_v2_sync_pending_count();
        zy100_offline_v2_session_info_t info;

        stage = APP_BLE_BOOTSTRAP_STAGE_OFFLINE_SYNC;
        /* Match the index view used by pending_count after user sync. */
        if ((count != 0U) &&
            ((s_bootstrap.user_id != 0U) ?
             zy100_offline_v2_storage_session_get_for_user(
                 s_bootstrap.user_id, count - 1U, &info) :
             zy100_offline_v2_storage_session_get(count - 1U, &info)))
        {
            reason = (uint8_t)info.stop_reason;
        }
    }
    detail = zy100_ble_connection_detail_pack(stage, reason, retry_ms);
    if (!force && s_bootstrap.snapshot_valid &&
        (s_bootstrap.snapshot_overall == overall) &&
        (s_bootstrap.snapshot_device_state == device_state) &&
        (s_bootstrap.snapshot_action == action) &&
        (s_bootstrap.snapshot_ready_bits == bits) &&
        (s_bootstrap.snapshot_detail == detail))
    {
        return true;
    }
    candidate_revision = (uint8_t)(s_bootstrap.revision + 1U);
    if (candidate_revision == 0U) candidate_revision = 1U;
    queued = zy100_ble_ctrl_service_notify_connection_state(
        candidate_revision,
        overall,
        device_state,
        action,
        s_bootstrap.generation,
        bits,
        detail);
    if (queued)
    {
        if (!s_bootstrap.snapshot_valid ||
            (s_bootstrap.snapshot_overall != overall) ||
            (s_bootstrap.snapshot_device_state != device_state))
        {
            ZY100_LOG_EVENT("[BLE_READY] conn=%u overall=%u state=%u stage=%u reason=%u",
                            conn_id, overall, device_state, stage, reason);
        }
        s_bootstrap.revision = candidate_revision;
        s_bootstrap.snapshot_valid = true;
        s_bootstrap.snapshot_overall = overall;
        s_bootstrap.snapshot_device_state = device_state;
        s_bootstrap.snapshot_action = action;
        s_bootstrap.snapshot_ready_bits = bits;
        s_bootstrap.snapshot_detail = detail;
    }
    ZY100_BLE_DIAG_LOG("[BLE_BOOTSTRAP] snapshot gen=%lu conn=%u rev=%u overall=%u bits=0x%08lX stage=%u reason=%u queued=%u",
                       (unsigned long)s_bootstrap.generation,
                       conn_id,
                       candidate_revision,
                       overall,
                       (unsigned long)bits,
                       stage,
                       reason,
                       queued ? 1U : 0U);
    return queued;
}

void app_ble_connection_bootstrap_init(void)
{
    memset(&s_bootstrap, 0, sizeof(s_bootstrap));
    s_bootstrap.conn_id = APP_BLE_BOOTSTRAP_CONN_ID_INVALID;
}

void app_ble_connection_bootstrap_on_connected(uint8_t conn_id)
{
    app_ble_connection_bootstrap_init();
    s_next_generation++;
    if (s_next_generation == 0U) s_next_generation = 1U;
    s_bootstrap.connected = true;
    s_bootstrap.conn_id = conn_id;
    s_bootstrap.generation = s_next_generation;
    ZY100_BLE_DIAG_LOG("[BLE_BOOTSTRAP] connected gen=%lu conn=%u ack_cccd=%u",
                       (unsigned long)s_bootstrap.generation,
                       conn_id,
                       app_ble_bootstrap_ack_ready(conn_id) ? 1U : 0U);
}

void app_ble_connection_bootstrap_on_disconnected(uint8_t conn_id)
{
    if (s_bootstrap.conn_id == conn_id) app_ble_connection_bootstrap_init();
}

void app_ble_connection_bootstrap_on_ack_cccd(uint8_t conn_id, bool enabled)
{
    if (!s_bootstrap.connected || (s_bootstrap.conn_id != conn_id))
    {
        ZY100_BLE_DIAG_LOG("[BLE_BOOTSTRAP] cccd_event_deferred conn=%u enabled=%u connected=%u active_conn=%u actual=%u",
                           conn_id,
                           enabled ? 1U : 0U,
                           s_bootstrap.connected ? 1U : 0U,
                           s_bootstrap.conn_id,
                           zy100_ble_ctrl_service_ack_notify_enabled(conn_id) ? 1U : 0U);
        return;
    }
    if (!enabled) s_bootstrap.host_deadline_ms = 0U;
    app_ble_bootstrap_arm_host_deadline();
    if (enabled && s_bootstrap.queried)
    {
        (void)app_ble_bootstrap_publish(conn_id, false);
    }
}

void app_ble_connection_bootstrap_on_security_ready(uint8_t conn_id)
{
    if (!s_bootstrap.connected || (s_bootstrap.conn_id != conn_id)) return;
    s_bootstrap.secure_ready = true;
    app_ble_bootstrap_arm_host_deadline();
    (void)app_ble_bootstrap_publish(conn_id, false);
}

bool app_ble_connection_bootstrap_host_ci_allowed(uint8_t conn_id)
{
    return s_bootstrap.connected && (s_bootstrap.conn_id == conn_id) &&
           s_bootstrap.queried && app_ble_bootstrap_ack_ready(conn_id) &&
           s_bootstrap.secure_ready;
}

void app_ble_connection_bootstrap_on_host_ci_enabled(uint8_t conn_id)
{
    if (!app_ble_connection_bootstrap_host_ci_allowed(conn_id)) return;
    s_bootstrap.host_ci_v3_seen = true;
    s_bootstrap.host_deadline_ms = 0U;
    (void)app_ble_bootstrap_publish(conn_id, false);
}

bool app_ble_connection_bootstrap_user_sync_allowed(uint8_t conn_id)
{
    return s_bootstrap.connected && (s_bootstrap.conn_id == conn_id) &&
           s_bootstrap.queried && app_ble_bootstrap_ack_ready(conn_id) &&
           s_bootstrap.secure_ready && s_bootstrap.host_ci_v3_seen;
}

bool app_ble_connection_bootstrap_user_sync_ready(uint8_t conn_id)
{
    return app_ble_connection_bootstrap_user_sync_allowed(conn_id) &&
           s_bootstrap.user_sync_ready && (s_bootstrap.user_id != 0U);
}

uint32_t app_ble_connection_bootstrap_user_id(uint8_t conn_id)
{
    return app_ble_connection_bootstrap_user_sync_ready(conn_id) ?
           s_bootstrap.user_id : 0U;
}

uint32_t app_ble_connection_bootstrap_current_user_id(void)
{
    return (s_bootstrap.connected && s_bootstrap.user_sync_ready) ?
           s_bootstrap.user_id : 0U;
}

bool app_ble_connection_bootstrap_on_user_sync_committed(uint8_t conn_id,
                                                         uint32_t user_id)
{
    if ((user_id == 0U) ||
        !app_ble_connection_bootstrap_user_sync_allowed(conn_id) ||
        (s_bootstrap.user_sync_ready &&
         (s_bootstrap.user_id != user_id)))
    {
        return false;
    }
    s_bootstrap.user_id = user_id;
    s_bootstrap.user_sync_ready = true;
    (void)app_ble_bootstrap_publish(conn_id, false);
    return true;
}

bool app_ble_connection_bootstrap_feature_config_sync_allowed(uint8_t conn_id)
{
    return app_ble_connection_bootstrap_user_sync_ready(conn_id) &&
           app_ble_time_sync_ok() &&
           zy100_online_stream_ready(conn_id) &&
           !app_ble_offline_v2_sync_active();
}

bool app_ble_connection_bootstrap_feature_config_ready(uint8_t conn_id)
{
    return app_ble_connection_bootstrap_feature_config_sync_allowed(conn_id) &&
           s_bootstrap.feature_config_ready;
}

bool app_ble_connection_bootstrap_business_ready(uint8_t conn_id)
{
    return s_bootstrap.connected && (s_bootstrap.conn_id == conn_id) &&
           ((app_ble_bootstrap_ready_bits(conn_id) &
             ZY100_BLE_CONNECTION_READY_BUSINESS) != 0U);
}

void app_ble_connection_bootstrap_on_feature_config_pending(uint8_t conn_id)
{
    if (!s_bootstrap.connected || (s_bootstrap.conn_id != conn_id))
    {
        return;
    }
    s_bootstrap.feature_config_ready = false;
    (void)app_ble_bootstrap_publish(conn_id, false);
}

bool app_ble_connection_bootstrap_on_feature_config_committed(
    uint8_t conn_id,
    uint32_t user_id)
{
    if (!app_ble_connection_bootstrap_feature_config_sync_allowed(conn_id) ||
        (user_id == 0U) || (user_id != s_bootstrap.user_id))
    {
        return false;
    }
    s_bootstrap.feature_config_ready = true;
    (void)app_ble_bootstrap_publish(conn_id, false);
    return true;
}

void app_ble_connection_bootstrap_handle_query(uint8_t conn_id)
{
    if (!s_bootstrap.connected || (s_bootstrap.conn_id != conn_id)) return;
    s_bootstrap.queried = true;
    ZY100_BLE_DIAG_LOG("[BLE_BOOTSTRAP] query gen=%lu conn=%u ack_cccd=%u secure=%u",
                       (unsigned long)s_bootstrap.generation,
                       conn_id,
                       app_ble_bootstrap_ack_ready(conn_id) ? 1U : 0U,
                       s_bootstrap.secure_ready ? 1U : 0U);
    (void)app_ble_bootstrap_publish(conn_id, true);
}

void app_ble_connection_bootstrap_refresh(uint8_t conn_id)
{
    if (s_bootstrap.queried)
        (void)app_ble_bootstrap_publish(conn_id, false);
}

void app_ble_connection_bootstrap_enter_recovery(uint8_t conn_id,
                                                 app_ble_bootstrap_stage_t stage,
                                                 uint8_t reason,
                                                 uint16_t retry_ms)
{
    if (!s_bootstrap.connected || (s_bootstrap.conn_id != conn_id)) return;
    s_bootstrap.recovery_only = true;
    s_bootstrap.recovery_stage = (uint8_t)stage;
    s_bootstrap.recovery_reason = reason;
    s_bootstrap.recovery_retry_ms = retry_ms;
    (void)app_ble_bootstrap_publish(conn_id, false);
}

void app_ble_connection_bootstrap_maintain(void)
{
    uint32_t now_ms;

    if (!s_bootstrap.connected) return;
    if (s_bootstrap.queried)
    {
        (void)app_ble_bootstrap_publish(s_bootstrap.conn_id, false);
    }
    now_ms = (uint32_t)os_sys_time_get();
    if (!zy100_offline_v2_capture_active() &&
        app_ble_bootstrap_deadline_expired(now_ms, s_bootstrap.host_deadline_ms) &&
        !s_bootstrap.host_ci_v3_seen && !s_bootstrap.upgrade_snapshot_sent)
    {
        uint8_t candidate_revision = (uint8_t)(s_bootstrap.revision + 1U);
        bool queued;
        if (candidate_revision == 0U) candidate_revision = 1U;
        queued = zy100_ble_ctrl_service_notify_connection_state(
            candidate_revision,
            ZY100_BLE_CONNECTION_FAILED,
            app_task_ble_ctrl_device_state(),
            ZY100_BLE_CONNECTION_ACTION_UPGRADE_HOST,
            s_bootstrap.generation,
            app_ble_bootstrap_ready_bits(s_bootstrap.conn_id),
            zy100_ble_connection_detail_pack(APP_BLE_BOOTSTRAP_STAGE_LINK_POLICY,
                                             APP_BLE_BOOTSTRAP_REASON_UPGRADE_HOST,
                                             0U));
        if (queued)
        {
            s_bootstrap.upgrade_snapshot_sent = true;
            s_bootstrap.revision = candidate_revision;
            s_bootstrap.disconnect_after_ms =
                now_ms + APP_BLE_BOOTSTRAP_DISCONNECT_GRACE_MS;
        }
    }
    if (app_ble_bootstrap_deadline_expired(now_ms,
                                           s_bootstrap.disconnect_after_ms))
    {
        s_bootstrap.disconnect_after_ms = 0U;
        app_ble_ci_state_require_disconnect("bootstrap_upgrade_host");
    }
}

void offline_capture_connection_changed(bool active)
{
    if (!s_bootstrap.connected || !s_bootstrap.queried)
    {
        return;
    }
    if (active)
    {
        s_bootstrap.host_deadline_ms = 0U;
        s_bootstrap.disconnect_after_ms = 0U;
    }
    (void)app_ble_bootstrap_publish(s_bootstrap.conn_id, true);
}
