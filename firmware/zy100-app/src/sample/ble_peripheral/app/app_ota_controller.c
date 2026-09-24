#include <stddef.h>
#include <string.h>

#include <os_sched.h>
#include <trace.h>

#include "app_flags.h"
#include "app/app_device_pairing.h"
#if ZY100_PRODUCT_LOG_QUIET_ENABLE
#undef DBG_DIRECT
#define DBG_DIRECT(...) ZY100_LOG_VERBOSE(__VA_ARGS__)
#endif

#include "app_task.h"
#include "app_ota_controller.h"
#include "app_ota_controller_port.h"
#include "app_ble_ci_state.h"
#include "app_ui_policy.h"
#include "peripheral_app.h"
#include "service/app_ble_notify_tracker.h"
#include "service/external_flash_capture.h"
#include "service/svc_led_owner.h"
#include "service/zy100_ble_ctrl_protocol.h"
#include "service/zy100_online_spool.h"
#include "service/zy100_rtc_clock.h"
#include "service/zy100_whole_unit_test.h"

#if F_APP_DFU_ENTRY_ENABLE && F_BLE_OTA_SERVICE_ENABLE
#define APP_OTA_CONTROLLER_ENTRY_ENABLE 1
#else
#define APP_OTA_CONTROLLER_ENTRY_ENABLE 0
#endif

#if APP_OTA_CONTROLLER_ENTRY_ENABLE
#define APP_OTA_PREPARE_LEASE_MS       10000ULL
#define APP_OTA_PREPARE_CONN_INVALID   0xFFU

#define APP_OTA_TRANSITION_IDLE            0U
#define APP_OTA_TRANSITION_ADMISSION_CHECK 1U
#define APP_OTA_TRANSITION_MODE_PENDING    2U
#define APP_OTA_TRANSITION_SWITCH_STARTED  3U

#define APP_OTA_ACK_NONE                   0U
#define APP_OTA_ACK_NOT_REQUIRED_COMPLETE  1U
#define APP_OTA_ACK_NOT_REQUIRED_WAITING   2U
#define APP_OTA_ACK_REQUIRED_NOT_SUBMITTED 3U
#define APP_OTA_ACK_REQUIRED_WAITING       4U
#define APP_OTA_ACK_REQUIRED_COMPLETE      5U

static uint8_t s_ota_transition_state = APP_OTA_TRANSITION_IDLE;
static bool s_ota_prepare_reserved = false;
static bool s_ota_silent_led_owned = false;
static uint8_t s_ota_accept_ack_state = APP_OTA_ACK_NONE;
#define s_ota_admission_lock \
    (s_ota_transition_state >= APP_OTA_TRANSITION_ADMISSION_CHECK)
#define s_ota_mode_pending \
    (s_ota_transition_state >= APP_OTA_TRANSITION_MODE_PENDING)
#define s_ota_switch_started \
    (s_ota_transition_state == APP_OTA_TRANSITION_SWITCH_STARTED)
#define s_ota_accept_ack_required \
    (s_ota_accept_ack_state >= APP_OTA_ACK_REQUIRED_NOT_SUBMITTED)
#define s_ota_accept_ack_waiting \
    ((s_ota_accept_ack_state == APP_OTA_ACK_NOT_REQUIRED_WAITING) || \
     (s_ota_accept_ack_state == APP_OTA_ACK_REQUIRED_WAITING))
#define s_ota_accept_ack_completed \
    ((s_ota_accept_ack_state == APP_OTA_ACK_NOT_REQUIRED_COMPLETE) || \
     (s_ota_accept_ack_state == APP_OTA_ACK_REQUIRED_COMPLETE))

static uint8_t s_ota_prepare_conn_id = APP_OTA_PREPARE_CONN_INVALID;
static uint64_t s_ota_prepare_deadline_ms = 0ULL;
static uint32_t s_ota_prepare_detail = 0U;

static const char *app_ota_controller_reject_reason_name(
    app_ota_reject_reason_t reason)
{
    switch (reason)
    {
    case APP_OTA_REJECT_NOT_CONNECTED:
        return "NOT_CONNECTED";
    case APP_OTA_REJECT_NOT_PAIRED:
        return "NOT_PAIRED";
    case APP_OTA_REJECT_LINK_NOT_ENCRYPTED:
        return "LINK_NOT_ENCRYPTED";
    case APP_OTA_REJECT_CAPTURE_BUSY:
        return "CAPTURE_BUSY";
    case APP_OTA_REJECT_FLASH_BUSY:
        return "FLASH_BUSY";
    case APP_OTA_REJECT_STREAM_BUSY:
        return "STREAM_BUSY";
    case APP_OTA_REJECT_SPOOL_NOT_DRAINED:
        return "SPOOL_NOT_DRAINED";
    case APP_OTA_REJECT_POWER_TRANSITION:
        return "POWER_TRANSITION";
    case APP_OTA_REJECT_CALIBRATION_BUSY:
        return "CALIBRATION_BUSY";
    case APP_OTA_REJECT_OTA_ALREADY_PENDING:
        return "OTA_ALREADY_PENDING";
    case APP_OTA_REJECT_PREPARE_REQUIRED:
        return "PREPARE_REQUIRED";
    case APP_OTA_REJECT_NONE:
    default:
        return "NONE";
    }
}

static const char *app_ota_controller_source_name(app_ota_source_t source)
{
    switch (source)
    {
    case APP_OTA_SOURCE_STANDARD_GATT:
        return "standard_gatt";
    case APP_OTA_SOURCE_PRIVATE_V2:
        return "private_v2";
    case APP_OTA_SOURCE_PRIVATE_V5:
        return "private_v5";
    case APP_OTA_SOURCE_CTRL_COMMIT:
        return "ctrl_commit";
    case APP_OTA_SOURCE_LEGACY:
    default:
        return "legacy";
    }
}

static void app_ota_controller_release_silent_led(void)
{
    if (s_ota_silent_led_owned)
    {
        led_release(LED_OWNER_OTA_UPLOAD_RESULT);
        s_ota_silent_led_owned = false;
    }
}

static void app_ota_controller_reset_prepare_reservation(void)
{
    s_ota_prepare_reserved = false;
    s_ota_prepare_conn_id = APP_OTA_PREPARE_CONN_INVALID;
    s_ota_prepare_deadline_ms = 0ULL;
}

static bool app_ota_controller_claim_silent_led(void)
{
    if (!app_led_request_all_off(LED_OWNER_OTA_UPLOAD_RESULT,
                                 LED_PRIORITY_OTA_UPLOAD_RESULT))
    {
        return false;
    }
    s_ota_silent_led_owned = true;
    return true;
}

static app_ota_reject_reason_t app_ota_controller_quiet_snapshot(
    const char *stage)
{
    uint32_t power_block_mask = 0U;
    app_ota_reject_reason_t reason =
        app_ota_controller_port_quiet_reason(&power_block_mask);

    ZY100_LOG_EVENT("[OTA_QUIET] stage=%s result=%s power_mask=0x%03lX state=%s",
                    (stage != NULL) ? stage : "unknown",
                    app_ota_controller_reject_reason_name(reason),
                    (unsigned long)power_block_mask,
                    app_ota_controller_port_power_state_name());
    return reason;
}

static void app_ota_controller_log_reject_snapshot(
    const char *stage,
    app_ota_reject_reason_t reason)
{
    uint32_t power_block_mask = 0U;

    (void)app_ota_controller_port_quiet_reason(&power_block_mask);
    ZY100_LOG_EVENT("[OTA_REJECT] stage=%s reason=%s power_mask=0x%03lX state=%s",
                    (stage != NULL) ? stage : "unknown",
                    app_ota_controller_reject_reason_name(reason),
                    (unsigned long)power_block_mask,
                    app_ota_controller_port_power_state_name());
}

static void app_ota_controller_release_prepared_rejection(
    uint8_t conn_id,
    app_ota_reject_reason_t reason,
    const char *stage)
{
    app_ble_ota_link_snapshot_t link;
    bool lease_valid;

    memset(&link, 0, sizeof(link));
    app_ble_ota_link_snapshot(conn_id, &link);
    lease_valid = app_ble_ci_state_ota_host_lease_valid(
                      conn_id, APP_BLE_CI_STATE_ONLINE_HIGH);

    s_ota_transition_state = APP_OTA_TRANSITION_IDLE;
    s_ota_accept_ack_state = APP_OTA_ACK_NONE;
    app_ota_controller_reset_prepare_reservation();
    app_ota_controller_release_silent_led();

    if (link.connected && link.paired && link.encrypted && lease_valid &&
        app_ble_ci_state_release_ota_host_lease(
            conn_id,
            APP_BLE_CI_STATE_ACTIVE_IDLE,
            (stage != NULL) ? stage : "ota_local_reject"))
    {
        DBG_DIRECT("[OTA_GATE] local_reject_release stage=%s reason=%s conn=%u disconnect=0",
                   (stage != NULL) ? stage : "unknown",
                   app_ota_controller_reject_reason_name(reason),
                   conn_id);
        return;
    }

    app_ble_ci_state_fail_ota_host_lease("ota_local_release_unavailable");
    app_ble_ci_state_require_disconnect("ota_local_release_unavailable");
    DBG_DIRECT("[OTA_GATE] reject_force_fail stage=%s reason=%s conn=%u link=%u/%u/%u lease=%u",
               (stage != NULL) ? stage : "unknown",
               app_ota_controller_reject_reason_name(reason),
               conn_id,
               link.connected ? 1U : 0U,
               link.paired ? 1U : 0U,
               link.encrypted ? 1U : 0U,
               lease_valid ? 1U : 0U);
}

void app_ota_controller_init(void)
{
    s_ota_transition_state = APP_OTA_TRANSITION_IDLE;
    s_ota_prepare_reserved = false;
    s_ota_prepare_conn_id = APP_OTA_PREPARE_CONN_INVALID;
    s_ota_prepare_deadline_ms = 0ULL;
    s_ota_prepare_detail = 0U;
    s_ota_silent_led_owned = false;
    s_ota_accept_ack_state = APP_OTA_ACK_NONE;
}

app_ble_ctrl_req_result_t app_task_ble_ctrl_request_ota_link_intent(
    uint8_t conn_id)
{
    app_ble_ota_link_snapshot_t link;
    app_ota_reject_reason_t quiet_reason;

    memset(&link, 0, sizeof(link));
    app_ble_ota_link_snapshot(conn_id, &link);
    if (!link.connected || !link.paired || !link.encrypted)
    {
        return APP_BLE_CTRL_REQ_RESULT_INVALID_STATE;
    }
    if (s_ota_admission_lock || s_ota_mode_pending ||
        s_ota_switch_started || s_ota_prepare_reserved)
    {
        return APP_BLE_CTRL_REQ_RESULT_BUSY;
    }
    quiet_reason = app_ota_controller_quiet_snapshot("link_intent");
    if (quiet_reason != APP_OTA_REJECT_NONE)
    {
        return (quiet_reason == APP_OTA_REJECT_POWER_TRANSITION) ?
               APP_BLE_CTRL_REQ_RESULT_NOT_READY :
               APP_BLE_CTRL_REQ_RESULT_BUSY;
    }
    if (!app_ble_ci_state_begin_ota_host_wait(
            conn_id,
            ZY100_BLE_OTA_LINK_INTENT_WINDOW_MS))
    {
        return APP_BLE_CTRL_REQ_RESULT_NOT_READY;
    }
    return APP_BLE_CTRL_REQ_RESULT_OK;
}

app_ble_ctrl_req_result_t app_task_ble_ctrl_request_ota_prepare(uint8_t conn_id)
{
    app_ble_ota_link_snapshot_t link;
    app_ota_reject_reason_t quiet_reason;
    bool was_standby = app_ota_controller_port_standby_active();

    s_ota_prepare_detail = 0U;

    memset(&link, 0, sizeof(link));
    app_ble_ota_link_snapshot(conn_id, &link);
    if (!link.connected || !link.paired || !link.encrypted)
    {
        DBG_DIRECT("[OTA_PREP] reject reason=link_auth conn=%u connected=%u paired=%u encrypted=%u",
                   conn_id,
                   link.connected ? 1U : 0U,
                   link.paired ? 1U : 0U,
                   link.encrypted ? 1U : 0U);
        return APP_BLE_CTRL_REQ_RESULT_INVALID_STATE;
    }

    if (s_ota_prepare_reserved)
    {
        if (s_ota_prepare_conn_id != conn_id)
        {
            return APP_BLE_CTRL_REQ_RESULT_BUSY;
        }
        if (!app_ble_ci_state_begin_ota_host_lease(
                conn_id,
                APP_BLE_CI_STATE_ONLINE_HIGH,
                (uint32_t)APP_OTA_PREPARE_LEASE_MS,
                "ble_ota_prepare_refresh"))
        {
            s_ota_prepare_detail = ZY100_BLE_OTA_DETAIL_LINK_NOT_HIGH;
            app_task_ota_cancel_admission(APP_OTA_REJECT_POWER_TRANSITION);
            return APP_BLE_CTRL_REQ_RESULT_NOT_READY;
        }
        s_ota_prepare_deadline_ms =
            os_sys_time_get() + APP_OTA_PREPARE_LEASE_MS;
        if (!app_ota_controller_claim_silent_led())
        {
            app_task_ota_cancel_admission(APP_OTA_REJECT_POWER_TRANSITION);
            return APP_BLE_CTRL_REQ_RESULT_INTERNAL_ERROR;
        }
        if (!app_ota_controller_port_active() ||
            !app_ble_ci_state_ota_host_lease_valid(
                conn_id, APP_BLE_CI_STATE_ONLINE_HIGH))
        {
            s_ota_prepare_detail = ZY100_BLE_OTA_DETAIL_LINK_NOT_HIGH;
            app_ota_controller_release_prepared_rejection(
                conn_id,
                APP_OTA_REJECT_POWER_TRANSITION,
                "prepare_refresh_state_reject");
            return APP_BLE_CTRL_REQ_RESULT_NOT_READY;
        }
        quiet_reason = app_ota_controller_quiet_snapshot("prepare_refresh");
        if (quiet_reason != APP_OTA_REJECT_NONE)
        {
            app_ota_controller_release_prepared_rejection(
                conn_id, quiet_reason, "prepare_refresh_reject");
            return (quiet_reason == APP_OTA_REJECT_POWER_TRANSITION) ?
                   APP_BLE_CTRL_REQ_RESULT_NOT_READY :
                   APP_BLE_CTRL_REQ_RESULT_BUSY;
        }
        DBG_DIRECT("[OTA_PREP] ready idempotent=1 conn=%u lease_ms=%lu",
                   conn_id,
                   (unsigned long)APP_OTA_PREPARE_LEASE_MS);
        return APP_BLE_CTRL_REQ_RESULT_OK;
    }

    if (s_ota_admission_lock || s_ota_mode_pending || s_ota_switch_started)
    {
        return APP_BLE_CTRL_REQ_RESULT_BUSY;
    }

    quiet_reason = app_ota_controller_quiet_snapshot("prepare_pre");
    if (quiet_reason != APP_OTA_REJECT_NONE)
    {
        DBG_DIRECT("[OTA_PREP] reject reason=%s",
                   app_ota_controller_reject_reason_name(quiet_reason));
        return (quiet_reason == APP_OTA_REJECT_POWER_TRANSITION) ?
               APP_BLE_CTRL_REQ_RESULT_NOT_READY :
               APP_BLE_CTRL_REQ_RESULT_BUSY;
    }

    if (!app_ble_ci_state_begin_ota_host_lease(
            conn_id,
            APP_BLE_CI_STATE_ONLINE_HIGH,
            (uint32_t)APP_OTA_PREPARE_LEASE_MS,
            "ble_ota_prepare"))
    {
        s_ota_prepare_detail = ZY100_BLE_OTA_DETAIL_LINK_NOT_HIGH;
        DBG_DIRECT("[OTA_PREP] reject reason=host_link_not_high conn=%u detail=0x%04lX",
                   conn_id,
                   (unsigned long)s_ota_prepare_detail);
        return APP_BLE_CTRL_REQ_RESULT_NOT_READY;
    }

    s_ota_prepare_reserved = true;
    s_ota_prepare_conn_id = conn_id;
    s_ota_prepare_deadline_ms =
        os_sys_time_get() + APP_OTA_PREPARE_LEASE_MS;

    if (was_standby &&
        !app_ota_controller_port_restore_standby_online_high(
            "ble_ota_prepare"))
    {
        ZY100_LOG_ERROR("[OTA_PREP] standby_restore_failed");
        app_task_ota_cancel_admission(APP_OTA_REJECT_POWER_TRANSITION);
        return APP_BLE_CTRL_REQ_RESULT_INTERNAL_ERROR;
    }

    if (!app_ota_controller_port_active())
    {
        DBG_DIRECT("[OTA_PREP] reject reason=power_state state=%s",
                   app_ota_controller_port_power_state_name());
        app_task_ota_cancel_admission(APP_OTA_REJECT_POWER_TRANSITION);
        return APP_BLE_CTRL_REQ_RESULT_NOT_READY;
    }

    if (!app_ble_ci_state_ota_host_lease_valid(
            conn_id, APP_BLE_CI_STATE_ONLINE_HIGH))
    {
        s_ota_prepare_detail = ZY100_BLE_OTA_DETAIL_LINK_NOT_HIGH;
        DBG_DIRECT("[OTA_PREP] reject reason=host_link_lost_after_wake conn=%u detail=0x%04lX",
                   conn_id,
                   (unsigned long)s_ota_prepare_detail);
        app_task_ota_cancel_admission(APP_OTA_REJECT_POWER_TRANSITION);
        return APP_BLE_CTRL_REQ_RESULT_NOT_READY;
    }

    app_ota_controller_port_cancel_wake_boot("ble_ota_prepare");
    app_ota_controller_port_cancel_online_complete("ota_prepare");
    if (!app_ota_controller_claim_silent_led())
    {
        DBG_DIRECT("[OTA_PREP] reject reason=led_all_off_failed");
        app_task_ota_cancel_admission(APP_OTA_REJECT_POWER_TRANSITION);
        return APP_BLE_CTRL_REQ_RESULT_INTERNAL_ERROR;
    }
    app_ota_controller_port_note_activity("ble_ota_prepare");
    quiet_reason = app_ota_controller_quiet_snapshot("prepare_post");
    if (quiet_reason != APP_OTA_REJECT_NONE)
    {
        app_ota_controller_release_prepared_rejection(
            conn_id, quiet_reason, "prepare_post_reject");
        return (quiet_reason == APP_OTA_REJECT_POWER_TRANSITION) ?
               APP_BLE_CTRL_REQ_RESULT_NOT_READY :
               APP_BLE_CTRL_REQ_RESULT_BUSY;
    }
    if (!app_ota_controller_port_active() ||
        !app_ble_ci_state_ota_host_lease_valid(
            conn_id, APP_BLE_CI_STATE_ONLINE_HIGH))
    {
        s_ota_prepare_detail = ZY100_BLE_OTA_DETAIL_LINK_NOT_HIGH;
        app_ota_controller_release_prepared_rejection(
            conn_id,
            APP_OTA_REJECT_POWER_TRANSITION,
            "prepare_post_state_reject");
        return APP_BLE_CTRL_REQ_RESULT_NOT_READY;
    }
    DBG_DIRECT("[OTA_PREP] ready standby_wake=%u target=ONLINE_HIGH conn=%u lease_ms=%lu silent=1 host_preapplied=1 peripheral_llcp=0",
               was_standby ? 1U : 0U,
               conn_id,
               (unsigned long)APP_OTA_PREPARE_LEASE_MS);
    return APP_BLE_CTRL_REQ_RESULT_OK;
}

uint32_t app_task_ble_ctrl_ota_prepare_detail(void)
{
    return s_ota_prepare_detail;
}

bool app_task_ota_blocks_new_business(void)
{
    return s_ota_admission_lock ||
           s_ota_mode_pending ||
           s_ota_switch_started ||
           s_ota_prepare_reserved ||
           app_ble_ci_state_ota_host_wait_active();
}

void app_task_ota_cancel_admission(app_ota_reject_reason_t reason)
{
    s_ota_transition_state = APP_OTA_TRANSITION_IDLE;
    s_ota_accept_ack_state = APP_OTA_ACK_NONE;
    app_ota_controller_reset_prepare_reservation();
    app_ota_controller_release_silent_led();
    app_ble_ci_state_fail_ota_host_lease("ota_admission_cancel");
    DBG_DIRECT("[OTA_GATE] cancel reason=%s",
               app_ota_controller_reject_reason_name(reason));
}

bool app_task_ota_try_admit(uint8_t conn_id,
                            app_ota_source_t source,
                            bool ack_required,
                            app_ota_reject_reason_t *reason_out)
{
    app_ble_ota_link_snapshot_t link;
    app_ota_reject_reason_t reason = APP_OTA_REJECT_NONE;
    bool prepared_for_conn =
        s_ota_prepare_reserved && (s_ota_prepare_conn_id == conn_id);

    if (app_device_pairing_window_active() || app_task_shutdown_blocks_new_business() ||
        zy100_whole_unit_first_user_boot_pending())
    {
        reason = APP_OTA_REJECT_POWER_TRANSITION;
        if (prepared_for_conn)
        {
            app_ota_controller_release_prepared_rejection(
                conn_id, reason, "commit_shutdown_reject");
        }
        goto reject_without_lock;
    }

    if ((source == APP_OTA_SOURCE_CTRL_COMMIT) && !prepared_for_conn)
    {
        reason = APP_OTA_REJECT_PREPARE_REQUIRED;
        goto reject_without_lock;
    }
    if ((source == APP_OTA_SOURCE_CTRL_COMMIT) &&
        !app_ble_ci_state_ota_host_lease_valid(
            conn_id, APP_BLE_CI_STATE_ONLINE_HIGH))
    {
        reason = APP_OTA_REJECT_POWER_TRANSITION;
        app_task_ota_cancel_admission(reason);
        goto reject_without_lock;
    }
    if (s_ota_admission_lock || s_ota_mode_pending || s_ota_switch_started)
    {
        reason = APP_OTA_REJECT_OTA_ALREADY_PENDING;
        goto reject_without_lock;
    }
    if (s_ota_prepare_reserved && !prepared_for_conn)
    {
        reason = APP_OTA_REJECT_OTA_ALREADY_PENDING;
        goto reject_without_lock;
    }

    /*
     * BLE/profile callbacks and all business start decisions are serialized
     * by App task. Set the one-way admission lock before taking the final
     * authentication/quiet snapshot; no volatile-only synchronization is used.
     */
    s_ota_transition_state = APP_OTA_TRANSITION_ADMISSION_CHECK;
    memset(&link, 0, sizeof(link));
    app_ble_ota_link_snapshot(conn_id, &link);
    if (!link.connected)
    {
        reason = APP_OTA_REJECT_NOT_CONNECTED;
    }
    else if (!link.paired)
    {
        reason = APP_OTA_REJECT_NOT_PAIRED;
    }
    else if (!link.encrypted)
    {
        reason = APP_OTA_REJECT_LINK_NOT_ENCRYPTED;
    }
    else
    {
        reason = app_ota_controller_quiet_snapshot(
                     (source == APP_OTA_SOURCE_CTRL_COMMIT) ?
                     "commit" : "admit");
    }

    if (reason != APP_OTA_REJECT_NONE)
    {
        s_ota_transition_state = APP_OTA_TRANSITION_IDLE;
        if (prepared_for_conn)
        {
            app_ota_controller_release_prepared_rejection(
                conn_id, reason, "commit_local_reject");
        }
        goto reject_without_lock;
    }

    app_ota_controller_reset_prepare_reservation();
    s_ota_transition_state = APP_OTA_TRANSITION_MODE_PENDING;
    s_ota_accept_ack_state = ack_required ?
                             APP_OTA_ACK_REQUIRED_NOT_SUBMITTED :
                             APP_OTA_ACK_NOT_REQUIRED_COMPLETE;
    app_ota_controller_port_button_reset_hold();
    app_ota_controller_port_cancel_charge_led("ota_admitted");
    app_ota_controller_port_cancel_online_complete("ota_admitted");
    ZY100_LOG_EVENT("[OTA_GATE] accept source=%s conn=%u ack=%u",
               app_ota_controller_source_name(source),
               conn_id,
               ack_required ? 1U : 0U);
    if (reason_out != NULL)
    {
        *reason_out = APP_OTA_REJECT_NONE;
    }
    return true;

reject_without_lock:
    if (reason_out != NULL)
    {
        *reason_out = reason;
    }
    if (source == APP_OTA_SOURCE_CTRL_COMMIT)
    {
        app_ota_controller_log_reject_snapshot("commit_reject", reason);
    }
    DBG_DIRECT("[OTA_GATE] reject reason=%s",
               app_ota_controller_reject_reason_name(reason));
    return false;
}

void app_task_ota_accept_ack_result(bool submitted)
{
    if (!s_ota_mode_pending || !s_ota_admission_lock)
    {
        return;
    }
    if (!submitted)
    {
        ZY100_LOG_ERROR("[OTA_GATE] final_fail reason=ACCEPT_ACK_NOT_SUBMITTED");
        app_task_ota_cancel_admission(APP_OTA_REJECT_STREAM_BUSY);
        return;
    }
    s_ota_accept_ack_state = s_ota_accept_ack_required ?
                             APP_OTA_ACK_REQUIRED_WAITING :
                             APP_OTA_ACK_NOT_REQUIRED_WAITING;
}

void app_task_ota_on_notify_complete(bool success)
{
    if (!s_ota_mode_pending || !s_ota_admission_lock ||
        !s_ota_accept_ack_waiting)
    {
        return;
    }
    s_ota_accept_ack_state = s_ota_accept_ack_required ?
                             APP_OTA_ACK_REQUIRED_NOT_SUBMITTED :
                             APP_OTA_ACK_NONE;
    if (!success)
    {
        ZY100_LOG_ERROR("[OTA_GATE] final_fail reason=ACCEPT_ACK_SEND_FAILED");
        app_task_ota_cancel_admission(APP_OTA_REJECT_STREAM_BUSY);
        return;
    }
    s_ota_accept_ack_state = APP_OTA_ACK_REQUIRED_COMPLETE;
    DBG_DIRECT("[OTA_GATE] accept_ack_complete");
}

void app_task_ota_on_disconnect(uint8_t conn_id)
{
    if (s_ota_prepare_reserved &&
        (s_ota_prepare_conn_id == conn_id) &&
        !s_ota_mode_pending)
    {
        DBG_DIRECT("[OTA_PREP] cancel reason=disconnect conn=%u", conn_id);
        app_task_ota_cancel_admission(APP_OTA_REJECT_NOT_CONNECTED);
        return;
    }
    if (s_ota_mode_pending && s_ota_accept_ack_required &&
        !s_ota_accept_ack_completed)
    {
        ZY100_LOG_ERROR("[OTA_GATE] final_fail reason=DISCONNECT_BEFORE_ACCEPT_ACK conn=%u",
                   conn_id);
        app_task_ota_cancel_admission(APP_OTA_REJECT_NOT_CONNECTED);
    }
}

void app_task_request_ota_mode(void)
{
    app_ota_reject_reason_t reason;
    (void)app_task_ota_try_admit(app_ble_power_conn_id(),
                                 APP_OTA_SOURCE_LEGACY,
                                 false,
                                 &reason);
}

void app_task_request_ota_mode_delayed(uint32_t delay_ms)
{
    (void)delay_ms;
    /* Strict admission never queues an OTA request for later. */
    app_task_request_ota_mode();
}

bool app_ota_controller_transition_pending(void)
{
    return s_ota_admission_lock || s_ota_mode_pending ||
           s_ota_switch_started || s_ota_prepare_reserved;
}

uint32_t app_ota_controller_standby_wait_ms(uint32_t wait_ms,
                                             uint64_t runtime_ms)
{
    (void)runtime_ms;

    /* BLE events wake the App task.  While LINK_INTENT owns the ADC gate,
     * retain the normal standby wait instead of spinning on an expired ADC
     * deadline or an artificial 5 ms OTA poll cadence. */
    if (app_ble_ci_state_ota_host_wait_active())
    {
        return wait_ms;
    }

    if (app_task_ota_blocks_new_business() &&
        (wait_ms > (uint32_t)F_APP_BUTTON_POLL_INTERVAL_MS))
    {
        return (uint32_t)F_APP_BUTTON_POLL_INTERVAL_MS;
    }

    return wait_ms;
}

void app_ota_controller_poll(void)
{
    app_ota_reject_reason_t reason;
    uint64_t runtime_ms = os_sys_time_get();

    if (s_ota_prepare_reserved && !s_ota_mode_pending)
    {
        if (runtime_ms >= s_ota_prepare_deadline_ms)
        {
            DBG_DIRECT("[OTA_PREP] timeout conn=%u lease_ms=%lu",
                       s_ota_prepare_conn_id,
                       (unsigned long)APP_OTA_PREPARE_LEASE_MS);
            app_task_ota_cancel_admission(APP_OTA_REJECT_POWER_TRANSITION);
        }
        return;
    }

    if ((!s_ota_admission_lock) ||
        (!s_ota_mode_pending) ||
        s_ota_switch_started)
    {
        return;
    }

    if (s_ota_accept_ack_required && !s_ota_accept_ack_completed)
    {
        return;
    }

    /* The accepted OTA-entry ACK must complete before disconnect. */
    if (app_ble_notify_tracker_in_flight())
    {
        return;
    }

#if F_APP_BLE_ENABLE
    if (!app_ota_controller_port_prepare_ble_quiesce())
    {
        return;
    }
#endif

    reason = app_ota_controller_quiet_snapshot("switch_final");
    if ((reason != APP_OTA_REJECT_NONE) ||
        !external_flash_capture_flash_wip_clear()
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE && \
    ZY100_ONLINE_STREAM_ENABLE
        || !zy100_online_spool_all_acked()
#endif
       )
    {
        if (reason == APP_OTA_REJECT_NONE)
        {
            reason = APP_OTA_REJECT_FLASH_BUSY;
        }
        ZY100_LOG_ERROR("[OTA_GATE] final_fail reason=%s",
                   app_ota_controller_reject_reason_name(reason));
        app_task_ota_cancel_admission(reason);
#if F_APP_BLE_ENABLE
        app_ota_controller_port_recover_ble_after_cancel();
#endif
        return;
    }

    if (!app_ota_controller_port_arm_time_checkpoint())
    {
        DBG_DIRECT("[OTA_TIME][WARN] arm_degraded continue_ota=1");
    }
    app_ota_controller_port_disable_watchdog("ota_switch");
    s_ota_transition_state = APP_OTA_TRANSITION_SWITCH_STARTED;
    ZY100_LOG_EVENT("[OTA_SWITCH] dfu_switch_to_ota_mode");
    zy100_rtc_clock_prepare_ota_handoff();
    app_ota_controller_port_switch_to_dfu();
}

#else

void app_ota_controller_init(void)
{
}

app_ble_ctrl_req_result_t app_task_ble_ctrl_request_ota_prepare(uint8_t conn_id)
{
    (void)conn_id;
    return APP_BLE_CTRL_REQ_RESULT_NOT_READY;
}

app_ble_ctrl_req_result_t app_task_ble_ctrl_request_ota_link_intent(
    uint8_t conn_id)
{
    (void)conn_id;
    return APP_BLE_CTRL_REQ_RESULT_NOT_READY;
}

uint32_t app_task_ble_ctrl_ota_prepare_detail(void)
{
    return 0U;
}

void app_task_request_ota_mode(void)
{
    DBG_DIRECT("[OTA_REQ] ignored: ota entry disabled");
}

void app_task_request_ota_mode_delayed(uint32_t delay_ms)
{
    (void)delay_ms;
    DBG_DIRECT("[OTA_REQ] delayed ignored: ota entry disabled");
}

bool app_task_ota_try_admit(uint8_t conn_id,
                            app_ota_source_t source,
                            bool ack_required,
                            app_ota_reject_reason_t *reason_out)
{
    (void)conn_id;
    (void)source;
    (void)ack_required;
    if (reason_out != NULL)
    {
        *reason_out = APP_OTA_REJECT_OTA_ALREADY_PENDING;
    }
    return false;
}

void app_task_ota_accept_ack_result(bool submitted)
{
    (void)submitted;
}

void app_task_ota_on_notify_complete(bool success)
{
    (void)success;
}

void app_task_ota_cancel_admission(app_ota_reject_reason_t reason)
{
    (void)reason;
}

void app_task_ota_on_disconnect(uint8_t conn_id)
{
    (void)conn_id;
}

bool app_task_ota_blocks_new_business(void)
{
    return false;
}

bool app_ota_controller_transition_pending(void)
{
    return false;
}

uint32_t app_ota_controller_standby_wait_ms(uint32_t wait_ms,
                                             uint64_t runtime_ms)
{
    (void)runtime_ms;
    return wait_ms;
}

void app_ota_controller_poll(void)
{
}

#endif
