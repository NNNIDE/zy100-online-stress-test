#include "app_ble_capture_observer.h"

#include <stddef.h>
#include <string.h>

#include <gap_conn_le.h>
#include <gap_storage_le.h>
#include <trace.h>

#include "app_ble_offline_v2_sync.h"
#include "app_ble_power_policy.h"
#include "../zy100_clock_config.h"
#include "../service/zy100_offline_v2_capture.h"

#define APP_BLE_CAPTURE_OBSERVER_CONN_INVALID 0xFFU

typedef struct
{
    bool active;
    bool known_peer;
    bool security_ready;
    bool repair_pending;
    bool repair_attempted;
    bool bond_refresh_seen;
    bool disconnect_pending;
    bool capture_seen;
    uint8_t conn_id;
    uint8_t last_command;
    uint32_t connected_ms;
    uint32_t command_count;
    uint32_t deferred_count;
} app_ble_capture_observer_runtime_t;

static app_ble_capture_observer_runtime_t s_observer =
{
    false, false, false, false, false, false, false, false,
    APP_BLE_CAPTURE_OBSERVER_CONN_INVALID, 0U, 0U, 0U, 0U
};

static bool app_ble_capture_observer_key_entry_valid(
    const T_LE_KEY_ENTRY *entry)
{
    uintptr_t value = (uintptr_t)entry;

    return (value != (uintptr_t)0U) &&
           (value != (uintptr_t)0xFFFFFFFFUL) &&
           (value != (uintptr_t)0xBDBDBDBDUL) &&
           entry->is_used;
}

static bool app_ble_capture_observer_peer_bonded(uint8_t conn_id)
{
    T_GAP_CONN_INFO info;
    T_LE_KEY_ENTRY *entry;

    memset(&info, 0, sizeof(info));
    if (!le_get_conn_info(conn_id, &info) ||
        (info.conn_state != GAP_CONN_STATE_CONNECTED))
    {
        return false;
    }
    entry = le_find_key_entry(
                info.remote_bd,
                (T_GAP_REMOTE_ADDR_TYPE)info.remote_bd_type);
    return app_ble_capture_observer_key_entry_valid(entry);
}

bool app_ble_capture_observer_on_connected(uint8_t conn_id)
{
    if (!zy100_offline_v2_capture_active())
    {
        return false;
    }
    memset(&s_observer, 0, sizeof(s_observer));
    s_observer.active = true;
    s_observer.capture_seen = true;
    s_observer.conn_id = conn_id;
    s_observer.connected_ms = zy100_os_time_ms();
    s_observer.known_peer = app_ble_capture_observer_peer_bonded(conn_id);
    s_observer.security_ready = false;
    s_observer.disconnect_pending = !s_observer.known_peer;
    DBG_DIRECT("[OFF_OBS_A] conn=%u active=1 known=%u state=%u",
               conn_id,
               s_observer.known_peer ? 1U : 0U,
               (uint32_t)zy100_offline_v2_capture_state());
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[OFF_OBS_A] conn=%u security=0 repair=0",
               conn_id);
    if (!s_observer.known_peer)
    {
        DBG_DIRECT("[OFF_OBS_D] conn=%u exit=rejected reason=unbonded",
                   conn_id);
    }
    return true;
}

void app_ble_capture_observer_on_disconnected(uint8_t conn_id)
{
    if (!s_observer.active || (s_observer.conn_id != conn_id))
    {
        return;
    }
    DBG_DIRECT("[OFF_OBS_D] conn=%u exit=disconnect commands=%lu deferred=%lu",
               conn_id,
               (unsigned long)s_observer.command_count,
               (unsigned long)s_observer.deferred_count);
    memset(&s_observer, 0, sizeof(s_observer));
    s_observer.conn_id = APP_BLE_CAPTURE_OBSERVER_CONN_INVALID;
}

bool app_ble_capture_observer_on_auth_started(uint8_t conn_id)
{
    if (!app_ble_capture_observer_active(conn_id))
    {
        return false;
    }
    s_observer.security_ready = false;
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[OFF_OBS_B] conn=%u step=auth result=started repair=%u",
               conn_id,
               s_observer.repair_pending ? 1U : 0U);
    return true;
}

bool app_ble_capture_observer_on_auth_complete(uint8_t conn_id,
                                               bool success,
                                               bool *security_ready_out)
{
    if (security_ready_out != NULL)
    {
        *security_ready_out = false;
    }
    if (!app_ble_capture_observer_active(conn_id))
    {
        return false;
    }
    if (!success || !s_observer.known_peer ||
        !app_ble_capture_observer_peer_bonded(conn_id))
    {
        s_observer.security_ready = false;
        s_observer.disconnect_pending = true;
        DBG_DIRECT("[OFF_OBS_D] conn=%u exit=auth_failed known=%u success=%u",
                   conn_id,
                   s_observer.known_peer ? 1U : 0U,
                   success ? 1U : 0U);
        return true;
    }
    s_observer.security_ready = true;
    s_observer.repair_pending = false;
    s_observer.disconnect_pending = false;
    if (security_ready_out != NULL)
    {
        *security_ready_out = true;
    }
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[OFF_OBS_B] conn=%u step=repair_pair result=auth_ok",
               conn_id);
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[OFF_OBS_A] conn=%u security=1 repair=%u refresh=%u",
               conn_id,
               s_observer.repair_attempted ? 1U : 0U,
               s_observer.bond_refresh_seen ? 1U : 0U);
    return true;
}

bool app_ble_capture_observer_on_bond_key_missing(uint8_t conn_id)
{
    if (!app_ble_capture_observer_active(conn_id) ||
        !s_observer.known_peer)
    {
        return false;
    }
    s_observer.security_ready = false;
    s_observer.repair_pending = true;
    s_observer.disconnect_pending = false;
    DBG_DIRECT("[OFF_OBS_B] conn=%u step=bond result=key_missing_deferred",
               conn_id);
    return true;
}

bool app_ble_capture_observer_on_bond_add(uint8_t conn_id)
{
    if (!app_ble_capture_observer_active(conn_id) ||
        !s_observer.known_peer)
    {
        return false;
    }
    s_observer.bond_refresh_seen = true;
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[OFF_OBS_B] conn=%u step=bond result=refresh_seen",
               conn_id);
    return true;
}

void app_ble_capture_observer_on_ack_cccd(uint8_t conn_id, bool enabled)
{
    if (!app_ble_capture_observer_active(conn_id))
    {
        return;
    }
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[OFF_OBS_B] conn=%u step=ack_cccd result=%s",
               conn_id,
               enabled ? "ready" : "disabled");
}

bool app_ble_capture_observer_active(uint8_t conn_id)
{
    return s_observer.active && (s_observer.conn_id == conn_id) &&
           (zy100_offline_v2_capture_active() || s_observer.repair_pending);
}

bool app_ble_capture_observer_authorized(uint8_t conn_id)
{
    return app_ble_capture_observer_active(conn_id) &&
           s_observer.known_peer && s_observer.security_ready;
}

app_ble_capture_pairing_decision_t
app_ble_capture_observer_on_pairing_request(uint8_t conn_id,
                                            const char *reason)
{
    const bool just_work = (reason != NULL) &&
                           (strcmp(reason, "just_work") == 0);

    if (!app_ble_capture_observer_active(conn_id))
    {
        return APP_BLE_CAPTURE_PAIRING_NOT_OBSERVER;
    }
    if (just_work && s_observer.known_peer &&
        !s_observer.repair_attempted)
    {
        s_observer.repair_pending = true;
        s_observer.repair_attempted = true;
        s_observer.security_ready = false;
        s_observer.disconnect_pending = false;
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[OFF_OBS_B] conn=%u step=repair_pair result=accept",
                   conn_id);
        return APP_BLE_CAPTURE_PAIRING_ACCEPT_KNOWN_REPAIR;
    }
    s_observer.disconnect_pending = true;
    DBG_DIRECT("[OFF_OBS_D] conn=%u exit=pairing_rejected reason=%s",
               conn_id,
               (reason != NULL) ? reason : "pairing_event");
    DBG_DIRECT("[OFF_OBS_D] conn=%u reject=%s known=%u attempted=%u",
               conn_id,
               just_work ? "unbonded_or_repeat" : "method_rejected",
               s_observer.known_peer ? 1U : 0U,
               s_observer.repair_attempted ? 1U : 0U);
    return APP_BLE_CAPTURE_PAIRING_REJECT;
}

bool app_ble_capture_observer_take_command_window(
    uint8_t conn_id,
    uint8_t command,
    uint32_t *generation_out,
    zy100_offline_v2_observer_window_result_t *result_out)
{
    bool allowed;

    if (!app_ble_capture_observer_authorized(conn_id))
    {
        if (result_out != NULL)
        {
            *result_out = ZY100_OFFLINE_V2_OBSERVER_WINDOW_NOT_RUNNING;
        }
        return false;
    }
    allowed = zy100_offline_v2_capture_observer_window_take(
                  generation_out, result_out);
    s_observer.last_command = command;
    if (allowed)
    {
        s_observer.command_count++;
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[OFF_OBS_B] conn=%u step=cmd_%u result=run gen=%lu",
                   conn_id,
                   command,
                   (unsigned long)((generation_out != NULL) ?
                                   *generation_out : 0U));
    }
    else
    {
        s_observer.deferred_count++;
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[OFF_OBS_B] conn=%u step=cmd_%u result=defer reason=%u",
                   conn_id,
                   command,
                   (uint32_t)((result_out != NULL) ? *result_out :
                              ZY100_OFFLINE_V2_OBSERVER_WINDOW_CONSUMED));
    }
    return allowed;
}

void app_ble_capture_observer_poll(uint32_t now_ms)
{
    if (!s_observer.active)
    {
        return;
    }
    if (s_observer.disconnect_pending &&
        app_ble_power_conn_valid(s_observer.conn_id))
    {
        app_ble_ci_state_require_disconnect("offline_capture_observer");
        s_observer.disconnect_pending = false;
        return;
    }
    if (!zy100_offline_v2_capture_active() && !s_observer.repair_pending)
    {
        zy100_offline_v2_capture_diag_t diag;

        memset(&diag, 0, sizeof(diag));
        (void)zy100_offline_v2_capture_get_diag(&diag);
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[OFF_OBS_C] conn=%u qmax=%lu push_max_us=%lu feat_max_us=%lu",
                   s_observer.conn_id,
                   (unsigned long)diag.event_queue_max_level,
                   (unsigned long)diag.push_max_us,
                   (unsigned long)diag.feature_max_us);
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[OFF_OBS_D] conn=%u exit=capture_done elapsed_ms=%lu",
                   s_observer.conn_id,
                   (unsigned long)(now_ms - s_observer.connected_ms));
        app_ble_offline_v2_sync_note_transfer_admitted(
            "observer_capture_done");
        s_observer.active = false;
    }
}
