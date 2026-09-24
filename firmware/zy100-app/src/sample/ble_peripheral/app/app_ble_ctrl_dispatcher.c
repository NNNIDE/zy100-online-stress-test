#include "app_ble_ctrl_dispatcher.h"
#if ZY100_ONLINE_STRESS_TEST_ENABLE
#include "zy100_online_stress.h"
#include "zy100_online_stream.h"
#endif
#include "app_ble_ctrl_dispatcher_port.h"
#include "app_ble_training_recovery.h"
#include "../common/zy100_byteorder.h"
#include "gap.h"

#include <string.h>

#include <cmsis_compiler.h>
#include <os_sched.h>
#include <trace.h>

#include "../app_flags.h"
#include "../app_task.h"
#include "app_ble_capture_observer.h"
#include "app_ble_ci_state.h"
#include "app_ble_link_trace.h"
#include "app_ble_connection_bootstrap.h"
#include "app_ble_offline_v2_sync.h"
#include "app_ble_power_policy.h"
#include "../service/zy100_ble_ctrl_protocol.h"
#include "../service/zy100_ble_ctrl_service.h"
#include "../service/zy100_calibration_manager.h"
#include "../service/zy100_whole_unit_test.h"
#include "../service/zy100_offline_v2_capture.h"
#include "../service/app_ble_notify_tracker.h"
#include "../service/zy100_rtc_clock.h"
#include "../service/zy100_system_info_store.h"
#include "../service/zy100_feature_config.h"
#include "../service/zy100_production_shipping.h"
#include "../storage/zy100_offline_v2_storage.h"

#if ZY100_PRODUCT_LOG_QUIET_ENABLE
#undef DBG_DIRECT
#define DBG_DIRECT(...) ZY100_BLE_DIAG_LOG(__VA_ARGS__)
#endif

static bool app_ble_ctrl_capture_observer_command(uint8_t command);

typedef struct
{
    bool valid;
    uint8_t conn_id;
    uint32_t user_id;
    uint32_t generation;
} app_ble_ctrl_user_sync_pending_t;

typedef struct
{
    zy100_ble_cmd_frame_t cmd;
    app_ble_ci_state_snapshot_t link_snapshot;
    app_ble_offline_v2_sync_reply_t offline_reply;
    uint32_t detail;
    uint32_t user_sync_owned_count;
    uint32_t user_sync_foreign_count;
    zy100_ble_ack_status_t status;
    zy100_ble_exec_mode_t exec_mode;
    uint8_t conn_id;
    uint8_t device_state;
    uint8_t response_flags;
    bool notify_sent;
    bool link_notify_requested;
    bool bootstrap_notify_requested;
    bool standby_settled;
    bool standby_legacy_ping;
    bool ota_commit_accepted;
    bool offline_reply_valid;
    bool user_sync_reply_valid;
    bool capture_observer_permitted;
    bool training_reply_valid;
    uint8_t training_payload[8];
} app_ble_ctrl_dispatch_ctx_t;

static app_ble_ctrl_user_sync_pending_t s_user_sync_pending;
static uint32_t s_control_receive_id;

/* Suppress identical retries, but keep a new failure or connection visible. */
static void app_ble_ctrl_log_ack_result(const app_ble_ctrl_dispatch_ctx_t *ctx,
                                        const zy100_ble_ack_frame_t *ack)
{
    static uint32_t last_generation;
    static uint32_t last_detail;
    static uint32_t last_key;
    static bool failed;
    uint32_t generation;
    uint32_t key;

    if ((ctx->status == ZY100_BLE_ACK_STATUS_OK) && ctx->notify_sent)
    {
        failed = false;
        return;
    }
    generation = app_ble_ci_state_generation();
    key = (uint32_t)ctx->cmd.cmd | ((uint32_t)ctx->status << 8) |
          ((uint32_t)ctx->notify_sent << 16) | ((uint32_t)ctx->conn_id << 24);
    if (!failed || (generation != last_generation) ||
        (key != last_key) || (ack->detail_le != last_detail) ||
        ZY100_PRODUCTION_DETAIL_LOG_ENABLE)
    {
        ZY100_LOG_WARN("[BLE_ACK] cmd=0x%02X seq=%u st=%u detail=0x%08lX notify=%u",
                       ctx->cmd.cmd, ctx->cmd.seq, (uint32_t)ctx->status,
                       (unsigned long)ack->detail_le, ctx->notify_sent ? 1U : 0U);
    }
    last_generation = generation;
    last_detail = ack->detail_le;
    last_key = key;
    failed = true;
}

__STATIC_FORCEINLINE void app_ble_ctrl_user_sync_pending_clear(void)
{
    memset(&s_user_sync_pending, 0, sizeof(s_user_sync_pending));
}

/* A reused connection slot must never inherit an earlier user's request. */
static void app_ble_ctrl_user_sync_pending_expire(void)
{
    if (s_user_sync_pending.valid &&
        (!app_ble_power_conn_valid(s_user_sync_pending.conn_id) ||
         (s_user_sync_pending.generation !=
          app_ble_connection_bootstrap_generation())))
    {
        app_ble_ctrl_user_sync_pending_clear();
    }
}

void app_ble_ctrl_dispatcher_on_disconnect(uint8_t conn_id)
{
    if (s_user_sync_pending.valid &&
        (s_user_sync_pending.conn_id == conn_id))
    {
        app_ble_ctrl_user_sync_pending_clear();
    }
}

void app_ble_ctrl_dispatcher_poll(void)
{
    uint8_t conn_id;
    uint32_t user_id;

    app_ble_ctrl_user_sync_pending_expire();
    if (!s_user_sync_pending.valid)
    {
        return;
    }
    conn_id = s_user_sync_pending.conn_id;
    user_id = s_user_sync_pending.user_id;
    if (zy100_offline_v2_capture_active() ||
        zy100_offline_v2_storage_job_busy() ||
        app_ble_offline_v2_sync_active())
    {
        return;
    }
    if (!zy100_system_info_set_latest_user_id(user_id) ||
        !app_ble_connection_bootstrap_on_user_sync_committed(conn_id,
                                                              user_id))
    {
        ZY100_LOG_ERROR("[BLE_USER][ERR] async_commit conn=%u user=%lu",
                   conn_id,
                   (unsigned long)user_id);
        app_ble_ctrl_user_sync_pending_clear();
        return;
    }
    DBG_DIRECT("[BLE_USER] async_commit conn=%u user=%lu result=ok",
               conn_id,
               (unsigned long)user_id);
    app_ble_ctrl_user_sync_pending_clear();
    app_ble_connection_bootstrap_refresh(conn_id);
}

static zy100_ble_ack_status_t app_ble_ctrl_status_from_req(app_ble_ctrl_req_result_t result)
{
    switch (result)
    {
    case APP_BLE_CTRL_REQ_RESULT_OK:
        return ZY100_BLE_ACK_STATUS_OK;
    case APP_BLE_CTRL_REQ_RESULT_BUSY:
        return ZY100_BLE_ACK_STATUS_BUSY;
    case APP_BLE_CTRL_REQ_RESULT_NOT_READY:
        return ZY100_BLE_ACK_STATUS_NOT_READY;
    case APP_BLE_CTRL_REQ_RESULT_INVALID_STATE:
    case APP_BLE_CTRL_REQ_RESULT_ID_MISMATCH:
        return ZY100_BLE_ACK_STATUS_INVALID_STATE;
    case APP_BLE_CTRL_REQ_RESULT_EXPORT_READY_HAS_DATA:
        return ZY100_BLE_ACK_STATUS_NOT_READY;
    case APP_BLE_CTRL_REQ_RESULT_INTERNAL_ERROR:
    default:
        return ZY100_BLE_ACK_STATUS_INTERNAL_ERROR;
    }
}

static zy100_ble_ack_status_t app_ble_ctrl_status_from_ota_reject(
    app_ota_reject_reason_t reason)
{
    switch (reason)
    {
    case APP_OTA_REJECT_NOT_CONNECTED:
    case APP_OTA_REJECT_NOT_PAIRED:
    case APP_OTA_REJECT_LINK_NOT_ENCRYPTED:
    case APP_OTA_REJECT_PREPARE_REQUIRED:
        return ZY100_BLE_ACK_STATUS_INVALID_STATE;
    case APP_OTA_REJECT_POWER_TRANSITION:
        return ZY100_BLE_ACK_STATUS_NOT_READY;
    case APP_OTA_REJECT_CAPTURE_BUSY:
    case APP_OTA_REJECT_FLASH_BUSY:
    case APP_OTA_REJECT_STREAM_BUSY:
    case APP_OTA_REJECT_SPOOL_NOT_DRAINED:
    case APP_OTA_REJECT_CALIBRATION_BUSY:
    case APP_OTA_REJECT_OTA_ALREADY_PENDING:
        return ZY100_BLE_ACK_STATUS_BUSY;
    case APP_OTA_REJECT_NONE:
    default:
        return ZY100_BLE_ACK_STATUS_INTERNAL_ERROR;
    }
}

static const char *app_ble_ctrl_reject_reason(uint8_t cmd,
                                              app_ble_ctrl_req_result_t result,
                                              uint8_t device_state)
{
    if (result == APP_BLE_CTRL_REQ_RESULT_BUSY)
    {
        if ((device_state == ZY100_BLE_DEVICE_STATE_BLE_EXPORTING) ||
            (device_state == ZY100_BLE_DEVICE_STATE_BLE_EXPORT_WAIT_CONFIRM))
        {
            return "ble_export_active";
        }
        if (device_state == ZY100_BLE_DEVICE_STATE_EXPORT_READY)
        {
            return "ble_export_pending";
        }
        if (device_state == ZY100_BLE_DEVICE_STATE_CLEARING_FLASH)
        {
            return (cmd == ZY100_BLE_CMD_CLEAR_FLASH) ? "already_clearing" : "clearing_flash";
        }
        if (device_state == ZY100_BLE_DEVICE_STATE_CAPTURING)
        {
            return "capturing";
        }
        if (device_state == ZY100_BLE_DEVICE_STATE_STOPPING)
        {
            return "stopping";
        }
        return "busy";
    }
    if (result == APP_BLE_CTRL_REQ_RESULT_NOT_READY)
    {
        if ((cmd == ZY100_BLE_CMD_START_CAPTURE) &&
            app_ble_power_is_connected() &&
            !app_ble_time_sync_ok())
        {
            return "time_not_synced";
        }
        return "not_ready";
    }
    if (result == APP_BLE_CTRL_REQ_RESULT_EXPORT_READY_HAS_DATA)
    {
        return "export_ready_has_data";
    }
    if (result == APP_BLE_CTRL_REQ_RESULT_ID_MISMATCH)
    {
        return "id_mismatch";
    }
    if (result == APP_BLE_CTRL_REQ_RESULT_INVALID_STATE)
    {
        return "state";
    }
    return "internal";
}

static bool app_ble_ctrl_cmd_requires_pairing(uint8_t cmd)
{
    return (cmd == ZY100_BLE_CMD_START_CAPTURE) ||
           (cmd == ZY100_BLE_CMD_PAUSE_CAPTURE) ||
           (cmd == ZY100_BLE_CMD_CLEAR_FLASH) ||
           (cmd == ZY100_BLE_CMD_FIND_DEVICE) ||
           (cmd == ZY100_BLE_CMD_HOST_CI_MODE_ENABLE) ||
           (cmd == ZY100_BLE_CMD_HOST_PROFILE_RESULT) ||
           (cmd == ZY100_BLE_CMD_GET_LINK_STATE) ||
           (cmd == ZY100_BLE_CMD_ENTER_SHIPPING) ||
           (cmd == ZY100_BLE_CMD_CONNECTION_USER_SYNC) ||
           (cmd == ZY100_BLE_CMD_FEATURE_CONFIG_SYNC) ||
           (cmd == ZY100_BLE_CMD_OTA_PREPARE) ||
           (cmd == ZY100_BLE_CMD_OTA_COMMIT) ||
           (cmd == ZY100_BLE_CMD_OTA_LINK_INTENT) ||
           (cmd == ZY100_BLE_CMD_EXPORT_CONFIRM) ||
           (cmd == ZY100_BLE_CMD_ONLINE_STREAM_READY) ||
           (cmd == ZY100_BLE_CMD_ONLINE_RECORD_ACK) ||
           (cmd == ZY100_BLE_CMD_OFFLINE_CAPTURE_START) ||
           (cmd == ZY100_BLE_CMD_OFFLINE_CAPTURE_STOP) ||
           (cmd == ZY100_BLE_CMD_TRAINING_SNAPSHOT) ||
           (cmd == ZY100_BLE_CMD_TRAINING_STOP) ||
           ((cmd >= ZY100_BLE_CMD_OFFLINE_SESSION_LIST) &&
            (cmd <= ZY100_BLE_CMD_OFFLINE_FOREIGN_PURGE));
}

static bool app_ble_ctrl_host_ci_state_allowed(uint8_t device_state)
{
    return (device_state == ZY100_BLE_DEVICE_STATE_WAIT_START) ||
           (device_state == ZY100_BLE_DEVICE_STATE_OFFLINE_CAPTURING) ||
           (device_state == ZY100_BLE_DEVICE_STATE_OFFLINE_FINALIZING) ||
           (device_state == ZY100_BLE_DEVICE_STATE_OFFLINE_SESSION_READY);
}

static bool app_ble_ctrl_cmd_is_offline_v2(uint8_t cmd)
{
    return (cmd >= ZY100_BLE_CMD_OFFLINE_SESSION_LIST) &&
           (cmd <= ZY100_BLE_CMD_OFFLINE_FOREIGN_PURGE);
}

static bool app_ble_ctrl_cmd_is_link_maintenance(uint8_t cmd)
{
    return (cmd == ZY100_BLE_CMD_HOST_CI_MODE_ENABLE) ||
           (cmd == ZY100_BLE_CMD_HOST_PROFILE_RESULT) ||
           (cmd == ZY100_BLE_CMD_GET_LINK_STATE) ||
           (cmd == ZY100_BLE_CMD_CONNECTION_USER_SYNC) ||
           (cmd == ZY100_BLE_CMD_GET_CONNECTION_STATE) ||
           (cmd == ZY100_BLE_CMD_PING);
}

static bool app_ble_ctrl_cmd_allowed_by_offline_arb(uint8_t cmd)
{
    app_ble_offline_v2_arb_state_t state =
        app_ble_offline_v2_sync_arb_state();

    if ((cmd == ZY100_BLE_CMD_OFFLINE_CAPTURE_STOP) ||
        (cmd == ZY100_BLE_CMD_TRAINING_STOP) ||
        (cmd == ZY100_BLE_CMD_TRAINING_SNAPSHOT) ||
        app_ble_ctrl_cmd_is_link_maintenance(cmd))
    {
        return true;
    }
    if (cmd == ZY100_BLE_CMD_CLEAR_FLASH)
    {
        return app_ble_offline_v2_sync_clear_admitted();
    }
    if (state == APP_BLE_OFFLINE_ARB_BOOTSTRAP)
    {
        uint32_t pending = app_ble_offline_v2_sync_pending_count();

        if (app_ble_offline_v2_sync_transfer_admitted())
        {
            if (app_ble_ctrl_cmd_is_offline_v2(cmd))
            {
                return true;
            }
            if (cmd == ZY100_BLE_CMD_TIME_SYNC)
            {
                return (pending == 0U) &&
                       !app_ble_offline_v2_sync_active() &&
                       !zy100_offline_v2_storage_job_busy();
            }
            return false;
        }
        /* The first LIST establishes transfer admission even when this user
         * owns no sessions.  The Offline handler still enforces user-sync,
         * capture/storage ownership and Export Notify readiness. */
        if (cmd == ZY100_BLE_CMD_OFFLINE_SESSION_LIST)
        {
            return true;
        }
        if (pending != 0U)
        {
            return false;
        }
        return false;
    }
    if ((state == APP_BLE_OFFLINE_ARB_REQUIRED) ||
        (state == APP_BLE_OFFLINE_ARB_TRANSFER))
    {
        return app_ble_ctrl_cmd_is_offline_v2(cmd);
    }
    return true;
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_start_capture(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    app_ble_ctrl_req_result_t req_result =
        APP_BLE_CTRL_REQ_RESULT_INTERNAL_ERROR;
    app_task_training_context_t training_ctx;

    if (!app_ble_connection_bootstrap_business_ready(ctx->conn_id))
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->status = ZY100_BLE_ACK_STATUS_NOT_READY;
        ctx->detail = ZY100_BLE_START_DETAIL_FEATURE_CONFIG_UNCONFIRMED;
        return;
    }
    if (!zy100_cal_manager_business_ready(ctx->conn_id))
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->status = ZY100_BLE_ACK_STATUS_NOT_READY;
        ctx->detail = ZY100_BLE_START_DETAIL_CAL_INFO_UNCONFIRMED;
        return;
    }
#if ZY100_LOG_VERBOSE_DEFAULT
    DBG_DIRECT("[BLE_CMD] start_capture request seq=%u user=%u train=%u time_ms=%llu",
               ctx->cmd.seq,
               ctx->cmd.user_id_le,
               ctx->cmd.training_id_le,
               (unsigned long long)ctx->cmd.device_time_ms_le);
#endif
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    if (!zy100_online_stress_armed(ctx->conn_id))
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->status = ZY100_BLE_ACK_STATUS_NOT_READY;
        ctx->detail = 0x53540001UL;
        return;
    }
#endif
    req_result = app_task_ble_ctrl_request_start(
                     ctx->cmd.seq,
                     ctx->cmd.user_id_le,
                     ctx->cmd.device_time_ms_le,
                     ctx->cmd.training_id_le);
    ctx->status = app_ble_ctrl_status_from_req(req_result);
    if (req_result == APP_BLE_CTRL_REQ_RESULT_OK)
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_ACCEPTED_ASYNC;
        ctx->detail = 0U;
#if ZY100_LOG_VERBOSE_DEFAULT
        DBG_DIRECT("[BLE_CMD] start_capture queued seq=%u", ctx->cmd.seq);
#endif
    }
    else
    {
        ctx->detail = app_task_ble_ctrl_start_detail();
        ctx->device_state = app_task_ble_ctrl_device_state();
        if (req_result == APP_BLE_CTRL_REQ_RESULT_EXPORT_READY_HAS_DATA)
        {
            ctx->device_state = ZY100_BLE_DEVICE_STATE_EXPORT_READY;
        }
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        if (req_result == APP_BLE_CTRL_REQ_RESULT_ID_MISMATCH)
        {
            app_task_training_context_get(&training_ctx);
            DBG_DIRECT("[BLE_CMD] start_capture reject seq=%u status=0x%02X reason=id_mismatch cmd_user=%lu active_user=%lu cmd_train=%lu next_train=%lu",
                       ctx->cmd.seq,
                       (uint32_t)ctx->status,
                       (unsigned long)ctx->cmd.user_id_le,
                       (unsigned long)training_ctx.active_user_id,
                       (unsigned long)ctx->cmd.training_id_le,
                       (unsigned long)training_ctx.next_training_id);
        }
        else
        {
            DBG_DIRECT("[BLE_CMD] start_capture reject seq=%u status=0x%02X reason=%s",
                       ctx->cmd.seq,
                       (uint32_t)ctx->status,
                       app_ble_ctrl_reject_reason(ctx->cmd.cmd,
                                                  req_result,
                                                  ctx->device_state));
        }
    }
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_pause_capture(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    app_ble_ctrl_req_result_t req_result =
        APP_BLE_CTRL_REQ_RESULT_INTERNAL_ERROR;
    app_task_training_context_t training_ctx;
    uint32_t active_user_id;
    uint32_t active_training_id;

    app_task_training_context_get(&training_ctx);
    active_user_id = training_ctx.current_capture_started ?
                     training_ctx.current_capture_user_id :
                     training_ctx.active_user_id;
    active_training_id = training_ctx.current_capture_started ?
                         training_ctx.current_capture_training_id :
                         training_ctx.next_training_id;
#if !ZY100_LOG_VERBOSE_DEFAULT
    (void)active_user_id;
    (void)active_training_id;
#endif
#if ZY100_LOG_VERBOSE_DEFAULT
    DBG_DIRECT("[BLE_CMD] pause_capture request seq=%u cmd_user=%lu cmd_train=%lu active_user=%lu active_train=%lu",
               ctx->cmd.seq,
               (unsigned long)ctx->cmd.user_id_le,
               (unsigned long)ctx->cmd.training_id_le,
               (unsigned long)active_user_id,
               (unsigned long)active_training_id);
#endif
    req_result = app_task_ble_ctrl_request_pause(
                     ctx->cmd.seq,
                     ctx->cmd.user_id_le,
                     ctx->cmd.device_time_ms_le,
                     ctx->cmd.training_id_le);
    ctx->status = app_ble_ctrl_status_from_req(req_result);
    if (req_result == APP_BLE_CTRL_REQ_RESULT_OK)
    {
        ctx->device_state = ZY100_BLE_DEVICE_STATE_STOPPING;
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_REAL_ACTION;
#if ZY100_LOG_VERBOSE_DEFAULT
        DBG_DIRECT("[BLE_CMD] pause_capture accepted seq=%u", ctx->cmd.seq);
#endif
    }
    else
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        DBG_DIRECT("[BLE_CMD] pause_capture reject seq=%u status=0x%02X reason=%s",
                   ctx->cmd.seq,
                   (uint32_t)ctx->status,
                   app_ble_ctrl_reject_reason(ctx->cmd.cmd,
                                              req_result,
                                              ctx->device_state));
    }
}

/* Private capture metadata requires this connection's completed USER_SYNC. */
__STATIC_FORCEINLINE void app_ble_ctrl_handle_training_snapshot(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    zy100_training_snapshot_t value;
    const zy100_training_snapshot_t *fresh = NULL;
    uint32_t generation = app_ble_connection_bootstrap_generation();
    uint32_t user = app_ble_connection_bootstrap_user_id(ctx->conn_id);
    uint32_t request = (uint32_t)ctx->cmd.device_time_ms_le;
    ctx->device_state = app_task_ble_ctrl_device_state();
    ctx->exec_mode = ZY100_BLE_EXEC_MODE_DRY_RUN_NO_ACTION;
    ctx->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
    ctx->detail = ZY100_BLE_OFFLINE_DETAIL_SNAPSHOT_ARGS;
    if (!app_ble_pairing_ready() || !app_ble_connection_bootstrap_user_sync_ready(ctx->conn_id) ||
        user == 0U || user != ctx->cmd.user_id_le || request == 0U ||
        (ctx->cmd.device_time_ms_le >> 32) != 0U ||
        ctx->cmd.training_id_le >= APP_TRAINING_SNAPSHOT_PAGE_COUNT) return;
    if (ctx->cmd.training_id_le == 0U &&
        !app_ble_training_snapshot_cached(generation, user, request))
    {
        if (!app_task_offline_training_snapshot(&value) ||
            gap_get_param(GAP_PARAM_BD_ADDR, value.device_address) != GAP_CAUSE_SUCCESS)
        {
            ctx->status = ZY100_BLE_ACK_STATUS_NOT_READY;
            ctx->detail = ZY100_BLE_OFFLINE_DETAIL_OWNER_BUSY | (100UL << 16);
            return;
        }
        if (value.owner_user_id != 0U && value.owner_user_id != user) return;
        fresh = &value;
    }
    ctx->status = app_ble_training_snapshot_page(generation, user, request,
        ctx->cmd.training_id_le, (uint32_t)os_sys_time_get(), fresh,
        ctx->training_payload, &ctx->detail);
    if (ctx->status == ZY100_BLE_ACK_STATUS_OK)
    {
        ctx->training_reply_valid = true;
        ctx->response_flags = (ZY100_TRAINING_SNAPSHOT_VERSION << 4) |
                              (uint8_t)ctx->cmd.training_id_le;
    }
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_training_stop(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    uint8_t token[ZY100_TRAINING_TOKEN_BYTES];
    uint32_t user = app_ble_connection_bootstrap_user_id(ctx->conn_id);
    app_ble_ctrl_req_result_t result;
    ctx->device_state = app_task_ble_ctrl_device_state();
    ctx->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
    ctx->detail = ZY100_BLE_OFFLINE_DETAIL_SESSION_MISMATCH;
    if (!app_ble_pairing_ready() || !app_ble_connection_bootstrap_user_sync_ready(ctx->conn_id)) return;
    zy100_put_u32_le(token, ctx->cmd.user_id_le);
    zy100_put_u64_le(token + 4, ctx->cmd.device_time_ms_le);
    zy100_put_u32_le(token + 12, ctx->cmd.training_id_le);
    if (!zy100_offline_v2_capture_token_matches(user, token)) return;
    if (!zy100_offline_v2_capture_active())
    {
        ctx->status = ZY100_BLE_ACK_STATUS_OK;
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_DRY_RUN_NO_ACTION;
        ctx->detail = ZY100_BLE_OFFLINE_DETAIL_ALREADY_ENDED;
        return;
    }
    result = app_task_ble_ctrl_request_offline_stop(ctx->cmd.seq, user, 0ULL, 0U);
    ctx->status = app_ble_ctrl_status_from_req(result);
    ctx->detail = app_task_ble_ctrl_offline_detail();
    ctx->device_state = app_task_ble_ctrl_device_state();
    if (result == APP_BLE_CTRL_REQ_RESULT_OK) ctx->exec_mode = ZY100_BLE_EXEC_MODE_REAL_ACTION;
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_offline_capture_start(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    app_ble_ctrl_req_result_t req_result =
        APP_BLE_CTRL_REQ_RESULT_INTERNAL_ERROR;

    if (!app_ble_connection_bootstrap_business_ready(ctx->conn_id))
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->status = ZY100_BLE_ACK_STATUS_NOT_READY;
        ctx->detail = ZY100_BLE_START_DETAIL_FEATURE_CONFIG_UNCONFIRMED;
        return;
    }
    if (!app_ble_connection_bootstrap_user_sync_ready(ctx->conn_id) ||
        (app_ble_connection_bootstrap_user_id(ctx->conn_id) !=
         ctx->cmd.user_id_le) ||
        (ctx->cmd.device_time_ms_le != 0ULL) ||
        (ctx->cmd.training_id_le != 0U))
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
        ctx->detail = ZY100_BLE_OFFLINE_DETAIL_INTENT;
        return;
    }
    if (!app_ble_offline_v2_sync_business_bootstrap_complete())
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->status = ZY100_BLE_ACK_STATUS_NOT_READY;
        ctx->detail = ZY100_BLE_OFFLINE_DETAIL_OWNER_BUSY;
        return;
    }
    req_result = app_task_ble_ctrl_request_offline_start(
        ctx->cmd.seq,
        ctx->cmd.user_id_le,
        ctx->cmd.device_time_ms_le,
        ctx->cmd.training_id_le);
    ctx->status = app_ble_ctrl_status_from_req(req_result);
    ctx->detail = app_task_ble_ctrl_offline_detail();
    if (req_result == APP_BLE_CTRL_REQ_RESULT_OK)
    {
        ctx->device_state = ZY100_BLE_DEVICE_STATE_OFFLINE_INTENT_READY;
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_ACCEPTED_ASYNC;
        ctx->detail = 0U;
    }
    else
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
    }
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_offline_capture_stop(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    app_ble_ctrl_req_result_t req_result =
        APP_BLE_CTRL_REQ_RESULT_INTERNAL_ERROR;

    if (!app_ble_power_conn_valid(ctx->conn_id) ||
        !app_ble_pairing_ready() ||
        (ctx->cmd.user_id_le == 0U) ||
        (ctx->cmd.device_time_ms_le != 0ULL) ||
        (ctx->cmd.training_id_le != 0U))
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
        ctx->detail = ZY100_BLE_OFFLINE_DETAIL_INTENT;
        return;
    }
    req_result = app_task_ble_ctrl_request_offline_stop(
        ctx->cmd.seq,
        ctx->cmd.user_id_le,
        ctx->cmd.device_time_ms_le,
        ctx->cmd.training_id_le);
    ctx->status = app_ble_ctrl_status_from_req(req_result);
    ctx->detail = app_task_ble_ctrl_offline_detail();
    if (req_result == APP_BLE_CTRL_REQ_RESULT_OK)
    {
        ctx->device_state =
            (ctx->detail == ZY100_BLE_OFFLINE_DETAIL_START_CANCELLED) ?
            ZY100_BLE_DEVICE_STATE_WAIT_START :
            ZY100_BLE_DEVICE_STATE_OFFLINE_FINALIZING;
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_REAL_ACTION;
    }
    else
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
    }
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_clear_flash(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    app_ble_ctrl_req_result_t req_result =
        APP_BLE_CTRL_REQ_RESULT_INTERNAL_ERROR;

#if ZY100_LOG_VERBOSE_DEFAULT
    DBG_DIRECT("[BLE_CMD] clear_flash request seq=%u user=%u train=%u time_ms=%llu",
               ctx->cmd.seq,
               ctx->cmd.user_id_le,
               ctx->cmd.training_id_le,
               (unsigned long long)ctx->cmd.device_time_ms_le);
#endif
    req_result = app_task_ble_ctrl_request_clear_flash(
                     ctx->cmd.seq,
                     ctx->cmd.user_id_le,
                     ctx->cmd.device_time_ms_le,
                     ctx->cmd.training_id_le);
    ctx->status = app_ble_ctrl_status_from_req(req_result);
    if (req_result == APP_BLE_CTRL_REQ_RESULT_OK)
    {
        ctx->device_state = ZY100_BLE_DEVICE_STATE_CLEARING_FLASH;
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_ACCEPTED_ASYNC;
    }
    else
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        DBG_DIRECT("[BLE_CMD] clear_flash reject seq=%u status=0x%02X reason=%s",
                   ctx->cmd.seq,
                   (uint32_t)ctx->status,
                   app_ble_ctrl_reject_reason(ctx->cmd.cmd,
                                              req_result,
                                              ctx->device_state));
    }
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_enter_shipping(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    app_ble_ctrl_req_result_t req_result =
        app_task_ble_ctrl_request_enter_shipping(
            ctx->conn_id,
            ctx->cmd.seq,
            ctx->cmd.user_id_le,
            ctx->cmd.training_id_le);

    ctx->status = app_ble_ctrl_status_from_req(req_result);
    ctx->device_state = app_task_ble_ctrl_device_state();
    ctx->detail = 0U;
    if (req_result == APP_BLE_CTRL_REQ_RESULT_OK)
    {
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_ACCEPTED_ASYNC;
        DBG_DIRECT("[PROD_SHIP] command accepted conn=%u seq=%u",
                   ctx->conn_id, ctx->cmd.seq);
    }
    else
    {
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        DBG_DIRECT("[PROD_SHIP] command rejected conn=%u seq=%u status=0x%02X",
                   ctx->conn_id, ctx->cmd.seq, (uint32_t)ctx->status);
    }
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_time_sync(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    uint32_t sync_user_id = ctx->cmd.user_id_le;
    uint32_t sync_training_id = ctx->cmd.training_id_le;
    zy100_rtc_debug_sync_snapshot_t rtc_sync_snapshot;

#if ZY100_LOG_VERBOSE_DEFAULT
    DBG_DIRECT("[BLE_SYNC] request seq=%u user=%lu train=%lu unix_ms=%llu",
               ctx->cmd.seq,
               (unsigned long)sync_user_id,
               (unsigned long)sync_training_id,
               (unsigned long long)ctx->cmd.device_time_ms_le);
#endif
    if (sync_user_id == 0U)
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
        ctx->detail = 0U;
        DBG_DIRECT("[BLE_SYNC] reject seq=%u reason=user_id_zero",
                   ctx->cmd.seq);
        return;
    }
    if (!app_ble_connection_bootstrap_user_sync_ready(ctx->conn_id) ||
        (app_ble_connection_bootstrap_user_id(ctx->conn_id) !=
         sync_user_id))
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
        ctx->detail = 0U;
        DBG_DIRECT("[BLE_SYNC] reject seq=%u reason=connection_user_mismatch user=%lu active=%lu",
                   ctx->cmd.seq,
                   (unsigned long)sync_user_id,
                   (unsigned long)app_ble_connection_bootstrap_user_id(
                       ctx->conn_id));
        return;
    }
    if (sync_training_id == 0U)
    {
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[TRAIN_SYNC][WARN] normalize training_id 0 -> 1");
        sync_training_id = 1U;
    }
    ctx->cmd.user_id_le = sync_user_id;
    ctx->cmd.training_id_le = sync_training_id;
    zy100_rtc_clock_debug_log_pre_sync(sync_user_id,
                                       sync_training_id,
                                       ctx->cmd.device_time_ms_le,
                                       &rtc_sync_snapshot);
    zy100_rtc_clock_sync(sync_user_id, ctx->cmd.device_time_ms_le);
    zy100_rtc_clock_debug_log_sync_done(sync_user_id,
                                        sync_training_id,
                                        &rtc_sync_snapshot);
    app_task_training_context_apply_time_sync(sync_user_id, sync_training_id);
    app_ble_ctrl_port_mark_time_sync_complete();
    ctx->device_state = app_task_ble_ctrl_device_state();
    ctx->exec_mode = ZY100_BLE_EXEC_MODE_REAL_ACTION;
    ctx->status = ZY100_BLE_ACK_STATUS_OK;
    ctx->detail = 1U;
#if ZY100_OFFLINE_FEATURE_V2_ENABLE
    app_ble_offline_v2_sync_note_business_bootstrap_complete("time_sync");
#else
    app_ble_export_try_auto_start("after_time_sync");
#endif
    ctx->device_state = app_task_ble_ctrl_device_state();
    if (app_task_training_meta_repair_needed())
    {
        DBG_DIRECT("[BLE_SYNC] repair_needed=1 ack_detail=1");
    }
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_find_device(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    app_ble_ctrl_req_result_t req_result =
        APP_BLE_CTRL_REQ_RESULT_INTERNAL_ERROR;

#if ZY100_LOG_VERBOSE_DEFAULT
    DBG_DIRECT("[BLE_CMD] find_device request seq=%u", ctx->cmd.seq);
#endif
    req_result = app_task_ble_ctrl_request_find_device(ctx->cmd.seq);
    ctx->status = app_ble_ctrl_status_from_req(req_result);
    ctx->device_state = app_task_ble_ctrl_device_state();
    if (req_result == APP_BLE_CTRL_REQ_RESULT_OK)
    {
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_REAL_ACTION;
        ctx->detail = (uint32_t)ZY100_BLE_FIND_DEVICE_HOLD_MS;
    }
    else
    {
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->detail = 0U;
        DBG_DIRECT("[BLE_CMD] find_device reject seq=%u status=0x%02X reason=%s",
                   ctx->cmd.seq,
                   (uint32_t)ctx->status,
                   app_ble_ctrl_reject_reason(ctx->cmd.cmd,
                                              req_result,
                                              ctx->device_state));
    }
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_host_ci_mode_enable(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    uint32_t host_version =
        (uint32_t)(ctx->cmd.device_time_ms_le & 0xFFFFFFFFULL);
    uint32_t profile_signature =
        (uint32_t)(ctx->cmd.device_time_ms_le >> 32);
    uint8_t protocol_version = (uint8_t)
        ((ctx->cmd.training_id_le & ZY100_BLE_HOST_PROTOCOL_MASK) >>
         ZY100_BLE_HOST_PROTOCOL_SHIFT);
    uint8_t host_platform = (uint8_t)
        ((ctx->cmd.training_id_le & ZY100_BLE_HOST_PLATFORM_MASK) >>
         ZY100_BLE_HOST_PLATFORM_SHIFT);
    uint8_t pre_enable_state = app_task_ble_ctrl_device_state();
    bool state_allowed =
        app_ble_ctrl_host_ci_state_allowed(pre_enable_state);
    bool enabled = false;

    if (state_allowed &&
        app_ble_connection_bootstrap_host_ci_allowed(ctx->conn_id))
    {
        bool notify_subscribed =
            zy100_ble_ctrl_service_ack_notify_enabled(ctx->conn_id);
        if ((protocol_version ==
             ZY100_BLE_HOST_CI_PROTOCOL_VERSION_V3) &&
            (host_platform ==
             (uint8_t)ZY100_BLE_HOST_PLATFORM_WINDOWS_LEGACY))
        {
            enabled = app_ble_ci_state_enable_windows_central(
                          ctx->conn_id,
                          ctx->cmd.user_id_le,
                          host_version,
                          ctx->cmd.training_id_le,
                          profile_signature,
                          notify_subscribed);
        }
        else if ((protocol_version ==
                  ZY100_BLE_HOST_CI_PROTOCOL_VERSION_V3) &&
                 (host_platform ==
                  (uint8_t)ZY100_BLE_HOST_PLATFORM_ANDROID))
        {
            enabled = app_ble_ci_state_enable_android_central(
                          ctx->conn_id,
                          ctx->cmd.user_id_le,
                          host_version,
                          ctx->cmd.training_id_le,
                          profile_signature,
                          notify_subscribed);
        }
    }
    ctx->device_state = app_task_ble_ctrl_device_state();
    ctx->exec_mode = enabled ?
        ZY100_BLE_EXEC_MODE_REAL_ACTION :
        ZY100_BLE_EXEC_MODE_NONE;
    ctx->status = enabled ? ZY100_BLE_ACK_STATUS_OK :
        ZY100_BLE_ACK_STATUS_NOT_READY;
    if (enabled)
    {
        ctx->detail = app_ble_ci_state_generation();
        app_ble_connection_bootstrap_on_host_ci_enabled(ctx->conn_id);
    }
    else if (!state_allowed)
    {
        ctx->detail = ZY100_BLE_HOST_CI_ENABLE_DETAIL_DEVICE_NOT_IDLE;
    }
    else
    {
        ctx->detail = app_ble_ci_state_host_enable_reject_detail();
        if (ctx->detail == 0U)
        {
            ctx->detail =
                ZY100_BLE_HOST_CI_ENABLE_DETAIL_CAPABILITY_INVALID;
        }
    }
    ctx->response_flags = enabled ? 1U : 0U;
    ctx->link_notify_requested = enabled;
    DBG_DIRECT("[HOST_CI] enable state=0x%02X allowed=%u enabled=%u detail=0x%08lX",
               pre_enable_state,
               state_allowed ? 1U : 0U,
               enabled ? 1U : 0U,
               (unsigned long)ctx->detail);
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_host_profile_result(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    uint32_t generation =
        (uint32_t)(ctx->cmd.device_time_ms_le & 0xFFFFFFFFULL);
    uint32_t transition =
        (uint32_t)(ctx->cmd.device_time_ms_le >> 32);
    app_ble_ci_host_profile_t profile =
        (app_ble_ci_host_profile_t)
        (ctx->cmd.training_id_le & 0xFFU);
    uint8_t request_status =
        (uint8_t)((ctx->cmd.training_id_le >> 8) & 0xFFU);
    app_ble_ci_host_result_t result =
        (app_ble_ci_host_result_t)
        ((ctx->cmd.training_id_le >> 16) & 0xFFU);
    bool accepted =
        app_ble_ci_state_host_profile_result(
            ctx->conn_id,
            ctx->cmd.user_id_le,
            generation,
            transition,
            profile,
            request_status,
            result);

    ctx->device_state = app_task_ble_ctrl_device_state();
    ctx->exec_mode = accepted ?
        ZY100_BLE_EXEC_MODE_REAL_ACTION :
        ZY100_BLE_EXEC_MODE_NONE;
    ctx->status = accepted ? ZY100_BLE_ACK_STATUS_OK :
        ZY100_BLE_ACK_STATUS_INVALID_STATE;
    ctx->detail = transition;
    ctx->link_notify_requested = accepted;
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_connection_user_sync(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    uint32_t persisted_user_id = 0U;
    bool persisted_valid = false;
    bool same_user;

    ctx->device_state = app_task_ble_ctrl_device_state();
    if (zy100_offline_v2_storage_foreign_purge_active())
    {
        ctx->device_state = ZY100_BLE_DEVICE_STATE_OFFLINE_RECLAIMING;
    }
    ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
    ctx->detail = 0U;
    if ((ctx->cmd.user_id_le == 0U) ||
        (ctx->cmd.device_time_ms_le != 0ULL) ||
        (ctx->cmd.training_id_le != 0U))
    {
        ctx->status = ZY100_BLE_ACK_STATUS_BAD_LENGTH;
        return;
    }
    if (!app_ble_connection_bootstrap_user_sync_allowed(ctx->conn_id))
    {
        ctx->status = ZY100_BLE_ACK_STATUS_NOT_READY;
        return;
    }
    app_ble_ctrl_user_sync_pending_expire();
    if (app_ble_connection_bootstrap_user_sync_ready(ctx->conn_id))
    {
        if (app_ble_connection_bootstrap_user_id(ctx->conn_id) !=
            ctx->cmd.user_id_le)
        {
            ctx->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
            return;
        }
        same_user = true;
    }
    else if (s_user_sync_pending.valid)
    {
        if ((s_user_sync_pending.conn_id != ctx->conn_id) ||
            (s_user_sync_pending.user_id != ctx->cmd.user_id_le))
        {
            ctx->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
            return;
        }
        ctx->user_sync_owned_count =
            zy100_offline_v2_storage_session_count_for_user(
                ctx->cmd.user_id_le);
        ctx->user_sync_foreign_count =
            zy100_offline_v2_storage_foreign_session_count(
                ctx->cmd.user_id_le);
        ctx->user_sync_reply_valid = true;
        ctx->status = ZY100_BLE_ACK_STATUS_OK;
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_ACCEPTED_ASYNC;
        ctx->detail = ctx->user_sync_foreign_count;
        ctx->response_flags = ZY100_BLE_CONNECTION_USER_SYNC_VERSION;
        return;
    }
    else
    {
        if (!zy100_system_info_get_latest_user_id(
                &persisted_user_id, &persisted_valid))
        {
            ctx->status = ZY100_BLE_ACK_STATUS_INTERNAL_ERROR;
            return;
        }
        same_user = persisted_valid &&
                    (persisted_user_id == ctx->cmd.user_id_le);
        if (!same_user && zy100_offline_v2_capture_active())
        {
            s_user_sync_pending.conn_id = ctx->conn_id;
            s_user_sync_pending.user_id = ctx->cmd.user_id_le;
            s_user_sync_pending.generation =
                app_ble_connection_bootstrap_generation();
            s_user_sync_pending.valid = true;
            ctx->user_sync_owned_count =
                zy100_offline_v2_storage_session_count_for_user(
                    ctx->cmd.user_id_le);
            ctx->user_sync_foreign_count =
                zy100_offline_v2_storage_foreign_session_count(
                    ctx->cmd.user_id_le);
            ctx->user_sync_reply_valid = true;
            ctx->status = ZY100_BLE_ACK_STATUS_OK;
            ctx->exec_mode = ZY100_BLE_EXEC_MODE_ACCEPTED_ASYNC;
            ctx->detail = ctx->user_sync_foreign_count;
            ctx->response_flags =
                ZY100_BLE_CONNECTION_USER_SYNC_VERSION;
            DBG_DIRECT("[BLE_USER] async_accept conn=%u user=%lu",
                       ctx->conn_id,
                       (unsigned long)ctx->cmd.user_id_le);
            return;
        }
        if (!zy100_system_info_set_latest_user_id(ctx->cmd.user_id_le))
        {
            ctx->status = ZY100_BLE_ACK_STATUS_INTERNAL_ERROR;
            return;
        }
        if (!app_ble_connection_bootstrap_on_user_sync_committed(
                ctx->conn_id, ctx->cmd.user_id_le))
        {
            ctx->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
            return;
        }
    }
    ctx->user_sync_owned_count =
        zy100_offline_v2_storage_session_count_for_user(
            ctx->cmd.user_id_le);
    ctx->user_sync_foreign_count =
        zy100_offline_v2_storage_foreign_session_count(
            ctx->cmd.user_id_le);
    ctx->user_sync_reply_valid = true;
    ctx->status = ZY100_BLE_ACK_STATUS_OK;
    ctx->exec_mode = same_user ?
        ZY100_BLE_EXEC_MODE_DRY_RUN_NO_ACTION :
        ZY100_BLE_EXEC_MODE_REAL_ACTION;
    ctx->detail = ctx->user_sync_foreign_count;
    ctx->response_flags = ZY100_BLE_CONNECTION_USER_SYNC_VERSION;
    DBG_DIRECT("[BLE_USER] sync conn=%u user=%lu same=%u owned=%lu foreign=%lu",
               ctx->conn_id,
               (unsigned long)ctx->cmd.user_id_le,
               same_user ? 1U : 0U,
               (unsigned long)ctx->user_sync_owned_count,
               (unsigned long)ctx->user_sync_foreign_count);
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_ota_link_intent(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    app_ble_ctrl_req_result_t req_result =
        APP_BLE_CTRL_REQ_RESULT_INTERNAL_ERROR;

    req_result = app_task_ble_ctrl_request_ota_link_intent(ctx->conn_id);
    ctx->status = app_ble_ctrl_status_from_req(req_result);
    ctx->device_state = app_task_ble_ctrl_device_state();
    ctx->exec_mode = (req_result == APP_BLE_CTRL_REQ_RESULT_OK) ?
        ZY100_BLE_EXEC_MODE_REAL_ACTION :
        ZY100_BLE_EXEC_MODE_NONE;
    ctx->detail = (req_result == APP_BLE_CTRL_REQ_RESULT_OK) ?
        ZY100_BLE_OTA_LINK_INTENT_WINDOW_MS : 0U;
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_ota_prepare(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    app_ble_ctrl_req_result_t req_result =
        APP_BLE_CTRL_REQ_RESULT_INTERNAL_ERROR;

    req_result = app_task_ble_ctrl_request_ota_prepare(ctx->conn_id);
    ctx->status = app_ble_ctrl_status_from_req(req_result);
    ctx->device_state = app_task_ble_ctrl_device_state();
    ctx->exec_mode = (req_result == APP_BLE_CTRL_REQ_RESULT_OK) ?
        ZY100_BLE_EXEC_MODE_REAL_ACTION :
        ZY100_BLE_EXEC_MODE_NONE;
    ctx->detail = app_task_ble_ctrl_ota_prepare_detail();
    if (ctx->detail == 0U)
    {
        ctx->detail = app_ble_ci_state_generation();
    }
    DBG_DIRECT("[OTA_PREP] ack status=0x%02X state=0x%02X detail=0x%08lX",
               (uint32_t)ctx->status,
               (uint32_t)ctx->device_state,
               (unsigned long)ctx->detail);
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_ota_commit(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    app_ota_reject_reason_t ota_commit_reject_reason =
        APP_OTA_REJECT_NONE;

    ctx->ota_commit_accepted = app_task_ota_try_admit(
                                   ctx->conn_id,
                                   APP_OTA_SOURCE_CTRL_COMMIT,
                                   true,
                                   &ota_commit_reject_reason);
    ctx->status = ctx->ota_commit_accepted ?
        ZY100_BLE_ACK_STATUS_OK :
        app_ble_ctrl_status_from_ota_reject(ota_commit_reject_reason);
    ctx->device_state = app_task_ble_ctrl_device_state();
    ctx->exec_mode = ctx->ota_commit_accepted ?
        ZY100_BLE_EXEC_MODE_REAL_ACTION :
        ZY100_BLE_EXEC_MODE_NONE;
    ctx->detail = ctx->ota_commit_accepted ? 0U :
        (uint32_t)ota_commit_reject_reason;
    if (ctx->ota_commit_accepted)
    {
        ZY100_LOG_EVENT("[OTA_COMMIT] accept conn=%u seq=%u",
                   ctx->conn_id,
                   ctx->cmd.seq);
    }
    else
    {
        ZY100_LOG_EVENT("[OTA_COMMIT] reject conn=%u seq=%u reason=%u status=0x%02X",
                   ctx->conn_id,
                   ctx->cmd.seq,
                   (uint32_t)ota_commit_reject_reason,
                   (uint32_t)ctx->status);
    }
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_export_confirm(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    app_ble_ctrl_req_result_t req_result =
        APP_BLE_CTRL_REQ_RESULT_INTERNAL_ERROR;

#if ZY100_LOG_VERBOSE_DEFAULT
    DBG_DIRECT("[BLE_EXPORT] confirm_cmd seq=%u export_id=%lu host_time_ms=%llu result=%lu",
               ctx->cmd.seq,
               (unsigned long)ctx->cmd.user_id_le,
               (unsigned long long)ctx->cmd.device_time_ms_le,
               (unsigned long)ctx->cmd.training_id_le);
#endif
    req_result = app_task_ble_export_request_confirm(
                     ctx->cmd.seq,
                     ctx->cmd.user_id_le,
                     ctx->cmd.device_time_ms_le,
                     ctx->cmd.training_id_le);
    ctx->status = app_ble_ctrl_status_from_req(req_result);
    ctx->device_state = app_task_ble_ctrl_device_state();
    ctx->exec_mode = ZY100_BLE_EXEC_MODE_REAL_ACTION;
    ctx->detail = ctx->cmd.training_id_le;
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_online_stream_ready(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    app_ble_ctrl_req_result_t req_result =
        APP_BLE_CTRL_REQ_RESULT_INTERNAL_ERROR;

    if (!zy100_cal_manager_business_ready(ctx->conn_id))
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->status = ZY100_BLE_ACK_STATUS_NOT_READY;
        ctx->detail = ZY100_BLE_START_DETAIL_CAL_INFO_UNCONFIRMED;
        return;
    }
    req_result = app_task_ble_ctrl_request_online_ready(
                     ctx->cmd.seq,
                     ctx->conn_id,
                     ctx->cmd.user_id_le,
                     ctx->cmd.device_time_ms_le);
    ctx->status = app_ble_ctrl_status_from_req(req_result);
    ctx->device_state = app_task_ble_ctrl_device_state();
    ctx->exec_mode = (req_result == APP_BLE_CTRL_REQ_RESULT_OK) ?
        ZY100_BLE_EXEC_MODE_REAL_ACTION :
        ZY100_BLE_EXEC_MODE_NONE;
    ctx->detail = ctx->cmd.training_id_le;
    DBG_DIRECT("[BLE_CMD] online_ready seq=%u conn=%u status=0x%02X cap=0x%08lX",
               ctx->cmd.seq,
               ctx->conn_id,
               (uint32_t)ctx->status,
               (unsigned long)ctx->cmd.user_id_le);
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_feature_config_sync(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    app_ble_ctrl_req_result_t req_result =
        APP_BLE_CTRL_REQ_RESULT_INTERNAL_ERROR;
    zy100_feature_config_t feature_config;
    uint64_t packed = ctx->cmd.device_time_ms_le;
    uint32_t current_generation = 0U;

    memset(&feature_config, 0, sizeof(feature_config));
    feature_config.schema_version = (uint8_t)(packed & 0xFFU);
    feature_config.flags = (uint8_t)((packed >> 8) & 0xFFU);
    feature_config.target = (uint8_t)((packed >> 16) & 0xFFU);
    feature_config.effect = (uint8_t)((packed >> 24) & 0xFFU);
    feature_config.red = (uint8_t)((packed >> 32) & 0xFFU);
    feature_config.green = (uint8_t)((packed >> 40) & 0xFFU);
    feature_config.blue = (uint8_t)((packed >> 48) & 0xFFU);
    feature_config.brightness_percent =
        (uint8_t)((packed >> 56) & 0xFFU);
    ctx->device_state = app_task_ble_ctrl_device_state();
    ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
    ctx->response_flags = ZY100_FEATURE_CONFIG_ACK_RESERVED;
    if ((ctx->cmd.user_id_le == 0U) ||
        !app_ble_connection_bootstrap_user_sync_ready(ctx->conn_id) ||
        (app_ble_connection_bootstrap_user_id(ctx->conn_id) !=
         ctx->cmd.user_id_le))
    {
        ctx->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
        ctx->detail = ZY100_BLE_FEATURE_DETAIL_USER;
        return;
    }
    if (ctx->cmd.training_id_le != zy100_feature_config_crc32(
            ctx->cmd.user_id_le, &feature_config))
    {
        ctx->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
        ctx->detail = ZY100_BLE_FEATURE_DETAIL_CRC;
        return;
    }
    if (!zy100_feature_config_validate(&feature_config))
    {
        ctx->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
        ctx->detail = ZY100_BLE_FEATURE_DETAIL_FIELD;
        return;
    }
    if (!app_ble_time_sync_ok())
    {
        ctx->status = ZY100_BLE_ACK_STATUS_NOT_READY;
        ctx->detail = ZY100_BLE_FEATURE_DETAIL_TIME_SYNC;
        return;
    }
    if (!app_ble_connection_bootstrap_feature_config_sync_allowed(
            ctx->conn_id))
    {
        ctx->status = ZY100_BLE_ACK_STATUS_BUSY;
        ctx->detail = ZY100_BLE_FEATURE_DETAIL_BUSY;
        return;
    }
    if (!app_task_feature_config_sync_allowed())
    {
        ctx->status = ZY100_BLE_ACK_STATUS_BUSY;
        ctx->detail = ZY100_BLE_FEATURE_DETAIL_BUSY;
        return;
    }
    (void)zy100_feature_config_get(NULL, NULL, &current_generation);
#if !ZY100_OFFLINE_V2_WOM_START_ENABLE
    if (zy100_feature_config_matches(ctx->cmd.user_id_le,
                                     &feature_config,
                                     ctx->cmd.training_id_le))
    {
        if (!app_task_feature_config_dry_run_begin(ctx->conn_id,
                                                   ctx->cmd.seq))
        {
            ctx->status = ZY100_BLE_ACK_STATUS_BUSY;
            ctx->detail = ZY100_BLE_FEATURE_DETAIL_BUSY;
            return;
        }
        if (!app_ble_connection_bootstrap_on_feature_config_committed(
                ctx->conn_id, ctx->cmd.user_id_le))
        {
            ctx->status = ZY100_BLE_ACK_STATUS_INTERNAL_ERROR;
            ctx->detail = ZY100_BLE_FEATURE_DETAIL_BUSY;
            return;
        }
        ctx->status = ZY100_BLE_ACK_STATUS_OK;
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_DRY_RUN_NO_ACTION;
        ctx->detail = current_generation;
        return;
    }
#endif
    req_result = app_task_ble_ctrl_request_feature_config(
        ctx->conn_id,
        ctx->cmd.seq,
        ctx->cmd.user_id_le,
        &feature_config,
        ctx->cmd.training_id_le);
    ctx->status = app_ble_ctrl_status_from_req(req_result);
    if (req_result == APP_BLE_CTRL_REQ_RESULT_OK)
    {
        app_ble_connection_bootstrap_on_feature_config_pending(
            ctx->conn_id);
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_ACCEPTED_ASYNC;
        ctx->detail = current_generation + 1U;
        if (ctx->detail == 0U)
        {
            ctx->detail = 1U;
        }
    }
    else
    {
        ctx->detail = (req_result == APP_BLE_CTRL_REQ_RESULT_BUSY) ?
            ZY100_BLE_FEATURE_DETAIL_BUSY :
            ZY100_BLE_FEATURE_DETAIL_FLASH;
    }
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_online_record_ack(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    app_ble_ctrl_req_result_t req_result =
        APP_BLE_CTRL_REQ_RESULT_INTERNAL_ERROR;

    req_result = app_task_ble_ctrl_request_online_record_ack(
                     ctx->cmd.seq,
                     ctx->conn_id,
                     ctx->cmd.user_id_le,
                     ctx->cmd.device_time_ms_le,
                     ctx->cmd.training_id_le);
    ctx->status = app_ble_ctrl_status_from_req(req_result);
    ctx->device_state = app_task_ble_ctrl_device_state();
    ctx->exec_mode = (req_result == APP_BLE_CTRL_REQ_RESULT_OK) ?
        ZY100_BLE_EXEC_MODE_REAL_ACTION :
        ZY100_BLE_EXEC_MODE_NONE;
    ctx->detail = ctx->cmd.training_id_le;
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_offline_v2_command(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    ctx->offline_reply_valid =
        app_ble_offline_v2_sync_handle_command(
            ctx->conn_id, &ctx->cmd, &ctx->offline_reply);
    if (!ctx->offline_reply_valid)
    {
        ctx->status = ZY100_BLE_ACK_STATUS_UNSUPPORTED_CMD;
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->detail = 0U;
    }
    else
    {
        ctx->status = ctx->offline_reply.status;
        ctx->device_state = (uint8_t)ctx->offline_reply.device_state;
        ctx->exec_mode = ctx->offline_reply.exec_mode;
        ctx->detail = ctx->offline_reply.detail;
    }
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_ping(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
#if ZY100_LOG_VERBOSE_DEFAULT
    DBG_DIRECT("[BLE_CMD] ping request seq=%u user=%u train=%u time_ms=%llu",
               ctx->cmd.seq,
               ctx->cmd.user_id_le,
               ctx->cmd.training_id_le,
               (unsigned long long)ctx->cmd.device_time_ms_le);
#endif
    ctx->device_state = app_task_ble_ctrl_device_state();
    if (!ctx->standby_legacy_ping)
    {
        app_ble_ci_state_note_host_ping(
            ctx->conn_id,
            ctx->cmd.user_id_le);
    }
    app_ble_ci_state_get_snapshot(&ctx->link_snapshot);
    app_ble_link_trace_note_ping(&ctx->link_snapshot, ctx->cmd.seq);
    ctx->exec_mode = ZY100_BLE_EXEC_MODE_DRY_RUN_NO_ACTION;
    if (app_task_training_meta_repair_needed())
    {
        ctx->detail = 2U;
    }
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_unsupported_command(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    ctx->device_state = app_task_ble_ctrl_device_state();
    ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
    ctx->status = ZY100_BLE_ACK_STATUS_UNSUPPORTED_CMD;
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_get_link_state(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    ctx->device_state = app_task_ble_ctrl_device_state();
    ctx->exec_mode = ZY100_BLE_EXEC_MODE_DRY_RUN_NO_ACTION;
    ctx->status = ZY100_BLE_ACK_STATUS_OK;
    ctx->detail = app_ble_ci_state_generation();
    ctx->link_notify_requested = true;
}

__STATIC_FORCEINLINE void app_ble_ctrl_handle_get_connection_state(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    ctx->device_state = app_task_ble_ctrl_device_state();
    ctx->exec_mode = ZY100_BLE_EXEC_MODE_DRY_RUN_NO_ACTION;
    if ((ctx->cmd.user_id_le != 0U) ||
        (ctx->cmd.device_time_ms_le != 0ULL) ||
        (ctx->cmd.training_id_le != 0U))
    {
        ctx->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
        ctx->detail = 0U;
    }
    else
    {
        ctx->status = ZY100_BLE_ACK_STATUS_OK;
        ctx->detail = ZY100_BLE_BOOTSTRAP_VERSION;
        ctx->bootstrap_notify_requested = true;
    }
}

__STATIC_FORCEINLINE bool app_ble_ctrl_parse_and_note_activity(
    app_ble_ctrl_dispatch_ctx_t *ctx,
    const uint8_t *data,
    uint16_t len)
{
    memset(&ctx->cmd, 0, sizeof(ctx->cmd));
    ctx->status = zy100_ble_ctrl_parse_command(data,
                                                len,
                                                &ctx->cmd);
    if (ctx->status != ZY100_BLE_ACK_STATUS_OK)
    {
        return false;
    }

    s_control_receive_id++;
    if (ctx->cmd.cmd == ZY100_BLE_CMD_OFFLINE_CAPTURE_START ||
        ctx->cmd.cmd == ZY100_BLE_CMD_OFFLINE_CAPTURE_STOP ||
        ctx->cmd.cmd == ZY100_BLE_CMD_TRAINING_STOP ||
        ctx->cmd.cmd == ZY100_BLE_CMD_TIME_SYNC ||
        ctx->cmd.cmd == ZY100_BLE_CMD_ONLINE_STREAM_READY ||
        ctx->cmd.cmd == ZY100_BLE_CMD_OFFLINE_FINAL_CONFIRM)
        ZY100_LOG_DETAIL("[OFF_RX] id=%lu cmd=%u seq=%u gen=%lu at=%llu state=%u",
            (unsigned long)s_control_receive_id, ctx->cmd.cmd, ctx->cmd.seq,
            (unsigned long)app_ble_connection_bootstrap_generation(),
            (unsigned long long)os_sys_time_get(), (unsigned)app_task_ble_ctrl_device_state());
    ctx->standby_settled = app_ble_ci_state_host_standby_settled();
    ctx->standby_legacy_ping = ctx->standby_settled &&
                                (ctx->cmd.cmd == ZY100_BLE_CMD_PING);
    if ((ctx->cmd.cmd != ZY100_BLE_CMD_TIMEOUT_ACTION_ACK) &&
        (ctx->cmd.cmd != ZY100_BLE_CMD_TRAINING_SNAPSHOT) &&
        !app_task_timeout_action_pending() &&
        !app_task_shutdown_blocks_new_business() &&
        !ctx->standby_legacy_ping &&
        !(ctx->standby_settled &&
          (ctx->cmd.cmd == ZY100_BLE_CMD_TIME_SYNC)))
    {
        app_task_auto_idle_note_ble_command(ctx->cmd.cmd);
    }
#if ZY100_LOG_VERBOSE_DEFAULT
    DBG_DIRECT("[BLE_WRITE] cmd=%s seq=%u",
               zy100_ble_ctrl_cmd_name(ctx->cmd.cmd),
               ctx->cmd.seq);
#endif
    return true;
}

__STATIC_FORCEINLINE bool app_ble_ctrl_apply_common_gates(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    if (app_task_timeout_action_pending() &&
        (ctx->cmd.cmd != ZY100_BLE_CMD_PING) &&
        (ctx->cmd.cmd != ZY100_BLE_CMD_GET_CONNECTION_STATE) &&
        (ctx->cmd.cmd != ZY100_BLE_CMD_GET_LINK_STATE) &&
        (ctx->cmd.cmd != ZY100_BLE_CMD_OFFLINE_CAPTURE_STOP) &&
        (ctx->cmd.cmd != ZY100_BLE_CMD_TRAINING_STOP) &&
        (ctx->cmd.cmd != ZY100_BLE_CMD_TRAINING_SNAPSHOT))
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->status = ZY100_BLE_ACK_STATUS_BUSY;
        ctx->detail = 0U;
        return false;
    }
    if (app_task_shutdown_blocks_new_business())
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->status = ZY100_BLE_ACK_STATUS_NOT_READY;
        ctx->detail = 0U;
        DBG_DIRECT("[SHUTDOWN][BLE] cmd_reject conn=%u cmd=%s seq=%u status=NOT_READY",
                   ctx->conn_id,
                   zy100_ble_ctrl_cmd_name(ctx->cmd.cmd),
                   ctx->cmd.seq);
        return false;
    }
    if (zy100_whole_unit_first_user_boot_pending())
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->status = ZY100_BLE_ACK_STATUS_NOT_READY;
        ctx->detail = 0U;
        return false;
    }
    if (ctx->standby_settled &&
        (ctx->cmd.cmd == ZY100_BLE_CMD_TIME_SYNC))
    {
        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->status = ZY100_BLE_ACK_STATUS_NOT_READY;
        ctx->detail = 0U;
        DBG_DIRECT("[BLE_STANDBY] reject_background cmd=TIME_SYNC seq=%u",
                   ctx->cmd.seq);
        return false;
    }
    if (app_ble_ctrl_cmd_requires_pairing(ctx->cmd.cmd) &&
        (!app_ble_pairing_ready() || !app_task_device_logic_ready()))
    {
        bool pairing_ok = app_ble_pairing_ready();
        bool device_ready = app_task_device_logic_ready();

        ctx->device_state = app_task_ble_ctrl_device_state();
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->status = ZY100_BLE_ACK_STATUS_NOT_READY;
        ctx->detail = 0U;
        DBG_DIRECT("[BLE_PAIR] cmd_reject conn=%u cmd=%s seq=%u reason=pairing_gate pairing_ok=%u device_ready=%u started=%u",
                   ctx->conn_id,
                   zy100_ble_ctrl_cmd_name(ctx->cmd.cmd),
                   ctx->cmd.seq,
                   pairing_ok ? 1U : 0U,
                   device_ready ? 1U : 0U,
                   app_ble_ctrl_port_pairing_started() ? 1U : 0U);
        return false;
    }

    if (zy100_offline_v2_capture_active() &&
        app_ble_ctrl_capture_observer_command(ctx->cmd.cmd))
    {
        uint32_t observer_generation = 0U;
        zy100_offline_v2_observer_window_result_t observer_result;

        ctx->capture_observer_permitted =
            app_ble_capture_observer_take_command_window(
                ctx->conn_id,
                ctx->cmd.cmd,
                &observer_generation,
                &observer_result);
        if (!ctx->capture_observer_permitted)
        {
            ctx->detail = ZY100_BLE_OFFLINE_DETAIL_OWNER_BUSY |
                           ((uint32_t)observer_result << 16);
        }
    }
    if (zy100_offline_v2_capture_active() &&
        (ctx->cmd.cmd != ZY100_BLE_CMD_GET_CONNECTION_STATE) &&
        (ctx->cmd.cmd != ZY100_BLE_CMD_OFFLINE_CAPTURE_STOP) &&
        (ctx->cmd.cmd != ZY100_BLE_CMD_TRAINING_STOP) &&
        (ctx->cmd.cmd != ZY100_BLE_CMD_TRAINING_SNAPSHOT) &&
        !ctx->capture_observer_permitted)
    {
        /* Capture and algorithm own CPU/sensor/Flash. Observer commands need
         * a one-shot service-owned safe-window token. */
        ctx->device_state = app_task_ble_ctrl_device_state();
        if (zy100_offline_v2_storage_foreign_purge_active())
        {
            ctx->device_state = ZY100_BLE_DEVICE_STATE_OFFLINE_RECLAIMING;
        }
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->status = ZY100_BLE_ACK_STATUS_BUSY;
        if (!app_ble_ctrl_capture_observer_command(ctx->cmd.cmd))
        {
            ctx->detail = ZY100_BLE_OFFLINE_DETAIL_OWNER_BUSY;
        }
        return false;
    }
    if (app_ble_offline_v2_sync_blocks_new_business() &&
        !app_ble_ctrl_cmd_allowed_by_offline_arb(ctx->cmd.cmd))
    {
        app_ble_offline_v2_arb_state_t arb =
            app_ble_offline_v2_sync_arb_state();
        uint32_t pending = app_ble_offline_v2_sync_pending_count();

        if (arb == APP_BLE_OFFLINE_ARB_TRANSFER)
        {
            ctx->device_state = ZY100_BLE_DEVICE_STATE_OFFLINE_SYNCING;
        }
        else if (pending != 0U)
        {
            ctx->device_state = ZY100_BLE_DEVICE_STATE_OFFLINE_SESSION_READY;
        }
        else
        {
            ctx->device_state = app_task_ble_ctrl_device_state();
        }
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->status = ZY100_BLE_ACK_STATUS_BUSY;
        ctx->detail = ZY100_BLE_OFFLINE_DETAIL_OWNER_BUSY;
        DBG_DIRECT("[BLE_BLOCK] cmd=%s state=%u detail=%lu",
                   zy100_ble_ctrl_cmd_name(ctx->cmd.cmd),
                   (uint32_t)arb,
                   (unsigned long)ctx->detail);
        DBG_DIRECT("[BLE_BLOCK] pending=%lu sync=%u job=%u",
                   (unsigned long)pending,
                   app_ble_offline_v2_sync_active() ? 1U : 0U,
                   zy100_offline_v2_storage_job_busy() ? 1U : 0U);
        return false;
    }
    if (zy100_cal_manager_mag_active() &&
        (ctx->cmd.cmd != ZY100_BLE_CMD_PING) &&
        (ctx->cmd.cmd != ZY100_BLE_CMD_GET_LINK_STATE) &&
        (ctx->cmd.cmd != ZY100_BLE_CMD_GET_CONNECTION_STATE) &&
        (ctx->cmd.cmd != ZY100_BLE_CMD_HOST_PROFILE_RESULT))
    {
        ctx->device_state = ZY100_BLE_DEVICE_STATE_CALIBRATING;
        ctx->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        ctx->status = ZY100_BLE_ACK_STATUS_BUSY;
        ctx->detail = 0U;
        return false;
    }
    return true;
}

__STATIC_FORCEINLINE void app_ble_ctrl_build_and_publish_ack(
    app_ble_ctrl_dispatch_ctx_t *ctx,
    zy100_ble_ack_frame_t *ack,
    uint8_t *ack_bytes)
{
    zy100_ble_ctrl_build_ack_ex(&ctx->cmd,
                                ctx->status,
                                (zy100_ble_device_state_t)ctx->device_state,
                                ctx->exec_mode,
                                ctx->detail,
                                ack);
    if (ctx->offline_reply_valid)
    {
        ack->user_id_echo_le = ctx->offline_reply.user_id_echo;
        ack->training_id_echo_le = ctx->offline_reply.training_id_echo;
    }
    if (ctx->user_sync_reply_valid)
    {
        ack->user_id_echo_le = ctx->cmd.user_id_le;
        ack->training_id_echo_le = ctx->user_sync_owned_count;
        ack->detail_le = ctx->user_sync_foreign_count;
    }
    if (ctx->training_reply_valid)
    {
        ack->user_id_echo_le = (uint32_t)ctx->cmd.device_time_ms_le;
        ack->training_id_echo_le = zy100_get_u32_le(ctx->training_payload);
        ack->detail_le = zy100_get_u32_le(ctx->training_payload + 4);
    }
    if (ctx->cmd.cmd == ZY100_BLE_CMD_TRAINING_STOP)
    {
        /* Tokens are not echoed into ordinary ACK/log fields. */
        ack->user_id_echo_le = 0U;
        ack->training_id_echo_le = 0U;
    }
    ack->reserved = ctx->response_flags;
    if (zy100_ble_ctrl_encode_ack(ack,
                                  ack_bytes,
                                  ZY100_BLE_ACK_FRAME_LEN))
    {
        ctx->notify_sent = app_ble_ctrl_port_publish_ack(
                                ctx->conn_id,
                                ack_bytes,
                                ZY100_BLE_ACK_FRAME_LEN);
    }
}

__STATIC_FORCEINLINE void app_ble_ctrl_run_post_ack_hooks(
    app_ble_ctrl_dispatch_ctx_t *ctx,
    const zy100_ble_ack_frame_t *ack)
{
    bool notify_sent = ctx->notify_sent;

    if (ctx->cmd.cmd == ZY100_BLE_CMD_PING)
    {
        app_ble_ci_state_get_snapshot(&ctx->link_snapshot);
        app_ble_link_trace_note_ping_ack(
            &ctx->link_snapshot,
            ctx->cmd.seq,
            notify_sent,
            app_ble_notify_tracker_count());
    }
    if (ctx->cmd.cmd == ZY100_BLE_CMD_FEATURE_CONFIG_SYNC)
    {
        app_task_feature_config_ack_submitted(ctx->conn_id,
                                              ctx->cmd.seq,
                                              notify_sent);
    }
    if (ctx->ota_commit_accepted)
    {
        app_task_ota_accept_ack_result(notify_sent);
    }
    app_ble_ctrl_log_ack_result(ctx, ack);
    if (!ctx->standby_legacy_ping &&
        !((ctx->status == ZY100_BLE_ACK_STATUS_OK) &&
          app_ble_online_quiet_logs_active()))
    {
        DBG_DIRECT("[BLE_ACK] conn=%u cmd=%s seq=%u status=0x%02X device_state=0x%02X exec_mode=0x%02X notify=%u",
                   ctx->conn_id,
                   zy100_ble_ctrl_cmd_name(ctx->cmd.cmd),
                   ctx->cmd.seq,
                   (uint32_t)ctx->status,
                   (uint32_t)ctx->device_state,
                   (uint32_t)ctx->exec_mode,
                   notify_sent ? 1U : 0U);
    }
    if (ctx->cmd.cmd == ZY100_BLE_CMD_TIME_SYNC)
    {
#if ZY100_LOG_VERBOSE_DEFAULT
        ZY100_DIAG_LOG("[BLE_ACK] cmd=TIME_SYNC seq=%u status=0x%02X device_state=0x%02X exec_mode=0x%02X user=%lu train=%lu detail=%lu",
                       ack->seq_echo,
                       (uint32_t)ack->status,
                       (uint32_t)ack->device_state,
                       (uint32_t)ack->exec_mode,
                       (unsigned long)ack->user_id_echo_le,
                       (unsigned long)ack->training_id_echo_le,
                       (unsigned long)ack->detail_le);
#endif
    }
}

__STATIC_FORCEINLINE void app_ble_ctrl_run_followup_notifications(
    app_ble_ctrl_dispatch_ctx_t *ctx)
{
    if (ctx->link_notify_requested)
    {
        if (!app_ble_ci_state_take_link_notify(&ctx->link_snapshot))
        {
            app_ble_ci_state_get_snapshot(&ctx->link_snapshot);
        }
        (void)zy100_ble_ctrl_service_notify_link_state(
            ctx->link_snapshot.session_id,
            ctx->link_snapshot.generation,
            ctx->link_snapshot.transition_id,
            (uint8_t)ctx->link_snapshot.desired_state,
            (uint8_t)ctx->link_snapshot.expected_profile,
            (uint8_t)ctx->link_snapshot.link_phase,
            (uint8_t)ctx->link_snapshot.initiator_mode,
            ctx->link_snapshot.actual_ci,
            ctx->link_snapshot.actual_latency);
    }
    if (ctx->bootstrap_notify_requested)
    {
        app_ble_connection_bootstrap_handle_query(ctx->conn_id);
    }
    else if (ctx->status == ZY100_BLE_ACK_STATUS_OK &&
             ctx->cmd.cmd != ZY100_BLE_CMD_TRAINING_SNAPSHOT)
    {
        app_ble_connection_bootstrap_refresh(ctx->conn_id);
    }
}

void app_ble_ctrl_dispatcher_handle_write(uint8_t conn_id,
                                          const uint8_t *data,
                                          uint16_t len)
{
    app_ble_ctrl_dispatch_ctx_t ctx;
    zy100_ble_ack_frame_t ack;
    uint8_t ack_bytes[ZY100_BLE_ACK_FRAME_LEN];

    ctx.conn_id = conn_id;
    ctx.exec_mode = ZY100_BLE_EXEC_MODE_NONE;
    ctx.detail = (uint32_t)os_sys_time_get();
    ctx.notify_sent = false;
    ctx.link_notify_requested = false;
    ctx.bootstrap_notify_requested = false;
    ctx.standby_settled = false;
    ctx.standby_legacy_ping = false;
    ctx.ota_commit_accepted = false;
    ctx.response_flags = 0U;
    ctx.offline_reply_valid = false;
    ctx.user_sync_reply_valid = false;
    ctx.user_sync_owned_count = 0U;
    ctx.user_sync_foreign_count = 0U;
    ctx.capture_observer_permitted = false;
    ctx.training_reply_valid = false;

    if (app_ble_ctrl_parse_and_note_activity(&ctx, data, len))
    {
        if (ctx.cmd.cmd == ZY100_BLE_CMD_TIMEOUT_ACTION_ACK)
        {
            ctx.device_state = app_task_ble_ctrl_device_state();
            ctx.detail = 0U;
            ctx.status = app_task_timeout_action_ack(conn_id, ctx.cmd.seq,
                ctx.cmd.user_id_le, ctx.cmd.training_id_le, ctx.cmd.device_time_ms_le) ?
                ZY100_BLE_ACK_STATUS_OK : ZY100_BLE_ACK_STATUS_INVALID_STATE;
        }
        else if (app_ble_ctrl_apply_common_gates(&ctx))
        {
            switch (ctx.cmd.cmd)
            {
#if ZY100_ONLINE_STRESS_TEST_ENABLE
            case ZY100_BLE_CMD_ONLINE_STRESS:
                ctx.device_state = app_task_ble_ctrl_device_state();
                ctx.detail = 0U;
                if (!app_ble_connection_bootstrap_business_ready(ctx.conn_id) ||
                    ctx.device_state != ZY100_BLE_DEVICE_STATE_WAIT_START ||
                    zy100_online_stream_active() || zy100_online_stream_start_prepare_in_progress())
                { ctx.status = ZY100_BLE_ACK_STATUS_NOT_READY; }
                else
                {
                    ctx.status = zy100_online_stress_command(ctx.conn_id,
                        ctx.cmd.training_id_le, ctx.cmd.device_time_ms_le, &ctx.detail) ?
                        ZY100_BLE_ACK_STATUS_OK : ZY100_BLE_ACK_STATUS_INVALID_STATE;
                }
                break;
#endif
            case ZY100_BLE_CMD_START_CAPTURE:
                app_ble_ctrl_handle_start_capture(&ctx);
                break;
            case ZY100_BLE_CMD_PAUSE_CAPTURE:
                app_ble_ctrl_handle_pause_capture(&ctx);
                break;
            case ZY100_BLE_CMD_TRAINING_SNAPSHOT:
                app_ble_ctrl_handle_training_snapshot(&ctx);
                break;
            case ZY100_BLE_CMD_TRAINING_STOP:
                app_ble_ctrl_handle_training_stop(&ctx);
                break;
            case ZY100_BLE_CMD_OFFLINE_CAPTURE_START:
                app_ble_ctrl_handle_offline_capture_start(&ctx);
                break;
            case ZY100_BLE_CMD_OFFLINE_CAPTURE_STOP:
                app_ble_ctrl_handle_offline_capture_stop(&ctx);
                break;
            case ZY100_BLE_CMD_CLEAR_FLASH:
                app_ble_ctrl_handle_clear_flash(&ctx);
                break;
            case ZY100_BLE_CMD_ENTER_SHIPPING:
                app_ble_ctrl_handle_enter_shipping(&ctx);
                break;
            case ZY100_BLE_CMD_TIME_SYNC:
                app_ble_ctrl_handle_time_sync(&ctx);
                break;
            case ZY100_BLE_CMD_FIND_DEVICE:
                app_ble_ctrl_handle_find_device(&ctx);
                break;
            case ZY100_BLE_CMD_HOST_CI_MODE_ENABLE:
                app_ble_ctrl_handle_host_ci_mode_enable(&ctx);
                break;
            case ZY100_BLE_CMD_HOST_PROFILE_RESULT:
                app_ble_ctrl_handle_host_profile_result(&ctx);
                break;
            case ZY100_BLE_CMD_GET_LINK_STATE:
                app_ble_ctrl_handle_get_link_state(&ctx);
                break;
            case ZY100_BLE_CMD_CONNECTION_USER_SYNC:
                app_ble_ctrl_handle_connection_user_sync(&ctx);
                break;
            case ZY100_BLE_CMD_GET_CONNECTION_STATE:
                app_ble_ctrl_handle_get_connection_state(&ctx);
                break;
            case ZY100_BLE_CMD_OTA_LINK_INTENT:
                app_ble_ctrl_handle_ota_link_intent(&ctx);
                break;
            case ZY100_BLE_CMD_OTA_PREPARE:
                app_ble_ctrl_handle_ota_prepare(&ctx);
                break;
            case ZY100_BLE_CMD_OTA_COMMIT:
                app_ble_ctrl_handle_ota_commit(&ctx);
                break;
            case ZY100_BLE_CMD_EXPORT_CONFIRM:
                app_ble_ctrl_handle_export_confirm(&ctx);
                break;
            case ZY100_BLE_CMD_ONLINE_STREAM_READY:
                app_ble_ctrl_handle_online_stream_ready(&ctx);
                break;
            case ZY100_BLE_CMD_FEATURE_CONFIG_SYNC:
                app_ble_ctrl_handle_feature_config_sync(&ctx);
                break;
            case ZY100_BLE_CMD_ONLINE_RECORD_ACK:
                app_ble_ctrl_handle_online_record_ack(&ctx);
                break;
            case ZY100_BLE_CMD_OFFLINE_SESSION_LIST:
            case ZY100_BLE_CMD_OFFLINE_SESSION_BEGIN:
            case ZY100_BLE_CMD_OFFLINE_CHUNK_ACK:
            case ZY100_BLE_CMD_OFFLINE_SESSION_RESUME:
            case ZY100_BLE_CMD_OFFLINE_FINAL_CONFIRM:
            case ZY100_BLE_CMD_OFFLINE_RECLAIM_STATUS:
            case ZY100_BLE_CMD_OFFLINE_FOREIGN_PURGE:
                app_ble_ctrl_handle_offline_v2_command(&ctx);
                break;
            case ZY100_BLE_CMD_PING:
                app_ble_ctrl_handle_ping(&ctx);
                break;
            default:
                app_ble_ctrl_handle_unsupported_command(&ctx);
                break;
            }
        }
    }
    else
    {
        ctx.device_state = app_task_ble_ctrl_device_state();
        ctx.exec_mode = ZY100_BLE_EXEC_MODE_NONE;
        DBG_DIRECT("[BLE_CMD] reject len=%u status=0x%02X cmd=%u seq=%u",
                   len,
                   (uint32_t)ctx.status,
                   ctx.cmd.cmd,
                   ctx.cmd.seq);
    }

    app_ble_ctrl_build_and_publish_ack(&ctx, &ack, ack_bytes);
    if (ctx.cmd.cmd == ZY100_BLE_CMD_TIMEOUT_ACTION_ACK) return;
    app_ble_ctrl_run_post_ack_hooks(&ctx, &ack);
    app_ble_ctrl_run_followup_notifications(&ctx);
}

static bool app_ble_ctrl_capture_observer_command(uint8_t command)
{
    return (command == ZY100_BLE_CMD_PING) ||
           (command == ZY100_BLE_CMD_GET_LINK_STATE) ||
           (command == ZY100_BLE_CMD_HOST_CI_MODE_ENABLE) ||
           (command == ZY100_BLE_CMD_HOST_PROFILE_RESULT) ||
           (command == ZY100_BLE_CMD_CONNECTION_USER_SYNC);
}
