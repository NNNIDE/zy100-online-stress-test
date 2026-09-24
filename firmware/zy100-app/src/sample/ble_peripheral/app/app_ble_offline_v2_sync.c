#include "app_ble_offline_v2_sync.h"

#include <stddef.h>
#include <string.h>
#include <trace.h>

#include "app_ble_ci_state.h"
#include "app_ble_connection_bootstrap.h"
#include "app_ble_conn_param_mgr.h"
#include "app_ble_export_controller.h"
#include "app_ble_power_policy.h"
#include "app_ble_profile_router.h"
#include "../common/zy100_byteorder.h"
#include "../service/zy100_capture_profile.h"
#include "../service/zy100_crc32.h"
#include "../service/zy100_offline_v2_capture.h"
#include "../service/zy100_ble_ctrl_service.h"
#include "../storage/zy100_offline_v2_storage.h"

#define OFFLINE_SYNC_MAGIC                 0xE7U
#define OFFLINE_SYNC_VERSION               ZY100_OFFLINE_V2_EXPORT_VERSION
#define OFFLINE_SYNC_OUTER_BYTES           8U
#define OFFLINE_SYNC_NOTIFY_MAX_BYTES      244U
#define OFFLINE_SYNC_CHUNK_DATA_BYTES      200U
#define OFFLINE_SYNC_CHUNK_HEADER_BYTES    28U
#define OFFLINE_SYNC_MAX_UNACKED_BYTES     ZY100_OFFLINE_V2_ACK_WINDOW_BYTES
#define OFFLINE_SYNC_LIST_ENTRY_BYTES      40U
#define OFFLINE_SYNC_LIST_HEADER_BYTES     20U
#define OFFLINE_SYNC_LIST_MAX_ENTRIES      5U
#define OFFLINE_SYNC_MANIFEST_MAX_BYTES    ZY100_OFFLINE_V2_MANIFEST_BYTES
#define OFFLINE_SYNC_LIST_HEALTH_CLEAN     0x01U
#define OFFLINE_SYNC_LIST_HEALTH_ATTENTION 0x02U
#define OFFLINE_SYNC_LIST_HEALTH_RECOVERED 0x04U
#define OFFLINE_SYNC_FLAG_LAST_FRAGMENT    0x01U
#define OFFLINE_SYNC_FLAG_RETRANSMIT       0x02U
#define OFFLINE_SYNC_FLAG_LIST_LAST        0x04U
#define OFFLINE_SYNC_CONN_INVALID          0xFFU
#define OFFLINE_SYNC_ACK_TIMEOUT_MS         5000U
#define OFFLINE_SYNC_ABORT_CONTRACT         3U
#define OFFLINE_SYNC_ABORT_ACK_TIMEOUT      6U
#define OFFLINE_SYNC_ABORT_LOCAL_CAPTURE    7U

typedef enum
{
    OFFLINE_SYNC_IDLE = 0U,
    OFFLINE_SYNC_WAIT_CI_HIGH,
    OFFLINE_SYNC_WAIT_HOST_RESUME,
    OFFLINE_SYNC_STREAMING,
    OFFLINE_SYNC_WAIT_CHUNK_ACK,
    OFFLINE_SYNC_VERIFY_RESUME,
    OFFLINE_SYNC_WAIT_RESUME,
    OFFLINE_SYNC_WAIT_FINAL_CONFIRM,
    OFFLINE_SYNC_CONFIRMING,
    OFFLINE_SYNC_WAIT_CI_BALANCED,
} offline_sync_state_t;

typedef struct
{
    offline_sync_state_t state;
    zy100_offline_v2_session_info_t session;
    uint8_t conn_id;
    uint16_t chunk_seq;
    uint32_t transfer_id;
    uint32_t manifest_crc;
    uint32_t transfer_seed;
    uint32_t sent_offset;
    uint32_t acked_offset;
    uint32_t sent_crc_state;
    uint32_t acked_crc_state;
    uint32_t acked_prefix_crc;
    uint32_t chunk_count;
    uint32_t list_generation;
    uint32_t list_revision;
    uint32_t list_expected_cursor;
    uint32_t list_pending_cursor;
    uint8_t admitted_conn;
    bool list_walk_valid;
    bool local_preempt_admitted;
    bool preempt_requested;
    bool preempt_abort_pending;
    uint32_t verify_offset;
    uint32_t verify_target;
    uint32_t verify_crc_state;
    uint32_t verify_expected_crc;
    uint32_t last_ack_ms;
    uint32_t queued_since_ms;
    uint32_t in_flight_since_ms;
    uint16_t queued_len;
    uint8_t queued_type;
    uint8_t in_flight_type;
    bool notify_in_flight;
    bool ack_progress_pending;
    bool retransmitting;
    bool host_resume_verified;
    bool clear_cancel_requested;
    uint8_t page[256];
    uint8_t frame[OFFLINE_SYNC_NOTIFY_MAX_BYTES];
} offline_sync_runtime_t;

static offline_sync_runtime_t s_sync = {
    OFFLINE_SYNC_IDLE, {0}, OFFLINE_SYNC_CONN_INVALID
};
static bool s_offline_arb_transfer_admitted;
static bool s_offline_arb_business_bootstrap_complete;
static app_ble_offline_v2_arb_state_t s_offline_arb_logged =
    APP_BLE_OFFLINE_ARB_BOOTSTRAP;

static void offline_sync_enter_wait_resume(const char *reason);

static uint16_t offline_sync_u16_saturate(uint32_t value)
{
    return (value > 0xFFFFUL) ? 0xFFFFU : (uint16_t)value;
}

static uint32_t offline_sync_foreign_purge_detail(uint32_t total_sectors,
                                                  uint32_t erased_sectors)
{
    return ((uint32_t)offline_sync_u16_saturate(total_sectors) << 16) |
           (uint32_t)offline_sync_u16_saturate(erased_sectors);
}

static uint32_t offline_sync_current_user_id(void)
{
    return app_ble_connection_bootstrap_current_user_id();
}

static uint32_t offline_sync_owned_session_count(void)
{
    uint32_t user_id = offline_sync_current_user_id();

    return (user_id != 0U) ?
           zy100_offline_v2_storage_session_count_for_user(user_id) :
           zy100_offline_v2_storage_session_count();
}

static void offline_sync_enter_wait_ci_balanced(const char *reason)
{
    uint8_t request_result;

    s_sync.state = OFFLINE_SYNC_WAIT_CI_BALANCED;
    request_result = app_ble_ci_state_request(
        APP_BLE_CI_STATE_ACTIVE_IDLE,
        (reason != NULL) ? reason : "offline_v2_restore_balanced");
    ZY100_LOG_DETAIL("[OFF_CI] phase=restore_balanced request=%u sid=%lu",
               request_result,
               (unsigned long)s_sync.session.session_id);
}

static app_ble_offline_v2_arb_state_t offline_sync_arb_compute(void)
{
    uint32_t pending = offline_sync_owned_session_count();

    if (!app_ble_power_is_connected())
    {
        return APP_BLE_OFFLINE_ARB_BOOTSTRAP;
    }
    if (app_ble_offline_v2_sync_active() ||
        (!zy100_offline_v2_capture_active() &&
         zy100_offline_v2_storage_job_busy()))
    {
        return APP_BLE_OFFLINE_ARB_TRANSFER;
    }
    if (pending != 0U)
    {
        return (s_offline_arb_transfer_admitted ||
                s_offline_arb_business_bootstrap_complete) ?
               APP_BLE_OFFLINE_ARB_REQUIRED :
               APP_BLE_OFFLINE_ARB_BOOTSTRAP;
    }
    return s_offline_arb_business_bootstrap_complete ?
           APP_BLE_OFFLINE_ARB_ONLINE_AVAILABLE :
           APP_BLE_OFFLINE_ARB_BOOTSTRAP;
}

static void offline_sync_arb_log(const char *reason, bool force)
{
    app_ble_offline_v2_arb_state_t state = offline_sync_arb_compute();

    if (!force && (state == s_offline_arb_logged))
    {
        return;
    }
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[BLE_ARB_A] old=%u new=%u transfer=%u business=%u",
               (uint32_t)s_offline_arb_logged,
               (uint32_t)state,
               s_offline_arb_transfer_admitted ? 1U : 0U,
               s_offline_arb_business_bootstrap_complete ? 1U : 0U);
    ZY100_LOG_DETAIL("[BLE_ARB_B] pending=%lu sync=%u job=%u",
               (unsigned long)offline_sync_owned_session_count(),
               app_ble_offline_v2_sync_active() ? 1U : 0U,
               zy100_offline_v2_storage_job_busy() ? 1U : 0U);
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[BLE_ARB_C] online=%u reason=%s",
               (state == APP_BLE_OFFLINE_ARB_ONLINE_AVAILABLE) ? 1U : 0U,
               (reason != NULL) ? reason : "state");
    s_offline_arb_logged = state;
}

static void offline_sync_arb_log_if_changed(const char *reason)
{
    offline_sync_arb_log(reason, false);
}

static void offline_sync_arb_reset_connection(const char *reason)
{
    app_ble_offline_v2_arb_state_t old_state = s_offline_arb_logged;
    bool changed = s_offline_arb_transfer_admitted ||
                   s_offline_arb_business_bootstrap_complete ||
                   (old_state != APP_BLE_OFFLINE_ARB_BOOTSTRAP);

    s_sync.local_preempt_admitted = false;
    s_sync.list_walk_valid = false;
    s_offline_arb_transfer_admitted = false;
    s_offline_arb_business_bootstrap_complete = false;
    s_offline_arb_logged = APP_BLE_OFFLINE_ARB_BOOTSTRAP;
    if (!changed)
    {
        return;
    }
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[BLE_ARB_A] old=%u new=%u transfer=0 business=0",
               (uint32_t)old_state,
               (uint32_t)APP_BLE_OFFLINE_ARB_BOOTSTRAP);
    ZY100_LOG_DETAIL("[BLE_ARB_B] pending=%lu sync=%u job=%u",
               (unsigned long)zy100_offline_v2_storage_session_count(),
               app_ble_offline_v2_sync_active() ? 1U : 0U,
               zy100_offline_v2_storage_job_busy() ? 1U : 0U);
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[BLE_ARB_C] online=0 reason=%s",
               (reason != NULL) ? reason : "disconnect");
}

static bool offline_sync_owner_available(void)
{
    return !zy100_offline_v2_capture_active() &&
           (zy100_capture_profile_current() == ZY100_CAPTURE_PROFILE_NONE) &&
           !zy100_offline_v2_storage_job_busy();
}

static bool offline_sync_transport_ready(uint8_t conn_id)
{
    return app_ble_power_is_connected() &&
           app_ble_power_conn_valid(conn_id) &&
           app_ble_ci_state_host_session_managed() &&
           zy100_ble_ctrl_service_export_notify_enabled(conn_id);
}

static void offline_sync_reply_default(
    const zy100_ble_cmd_frame_t *command,
    app_ble_offline_v2_sync_reply_t *reply)
{
    memset(reply, 0, sizeof(*reply));
    reply->status = ZY100_BLE_ACK_STATUS_INTERNAL_ERROR;
    reply->exec_mode = ZY100_BLE_EXEC_MODE_NONE;
    reply->device_state = ZY100_BLE_DEVICE_STATE_OFFLINE_SESSION_READY;
    reply->user_id_echo = command->user_id_le;
    reply->training_id_echo = command->training_id_le;
}

static bool offline_sync_confirm_identity_matches(
    const zy100_ble_cmd_frame_t *command,
    uint32_t generation,
    uint32_t transfer_id)
{
    return (command->user_id_le == s_sync.session.session_id) &&
           (generation == s_sync.session.generation) &&
           (transfer_id == s_sync.transfer_id) &&
           (command->training_id_le == s_sync.session.stream_crc32);
}

static void offline_sync_log_confirm(const char *path,
                                     uint8_t status,
                                     uint32_t generation,
                                     uint32_t transfer_id,
                                     uint32_t crc)
{
    DBG_DIRECT("[OFF_CONFIRM_A] path=%s status=%u state=%u",
               path, status, (uint32_t)s_sync.state);
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[OFF_CONFIRM_B] sid=%lu gen=%lu tid=%lu crc=%08lX",
               (unsigned long)s_sync.session.session_id,
               (unsigned long)generation,
               (unsigned long)transfer_id,
               (unsigned long)crc);
}

static bool offline_sync_queue_frame(uint8_t type,
                                     uint8_t flags,
                                     const uint8_t *payload,
                                     uint16_t payload_len)
{
    if ((payload_len > (OFFLINE_SYNC_NOTIFY_MAX_BYTES -
                        OFFLINE_SYNC_OUTER_BYTES)) ||
        (s_sync.queued_len != 0U) || s_sync.notify_in_flight)
    {
        return false;
    }
    s_sync.frame[0] = OFFLINE_SYNC_MAGIC;
    s_sync.frame[1] = OFFLINE_SYNC_VERSION;
    s_sync.frame[2] = type;
    s_sync.frame[3] = flags;
    zy100_put_u16_le(&s_sync.frame[4], s_sync.chunk_seq++);
    zy100_put_u16_le(&s_sync.frame[6], payload_len);
    if ((payload_len != 0U) && (payload != &s_sync.frame[8]))
    {
        memcpy(&s_sync.frame[8], payload, payload_len);
    }
    s_sync.queued_len = payload_len + OFFLINE_SYNC_OUTER_BYTES;
    s_sync.queued_type = type;
    s_sync.queued_since_ms = 0U;
    return true;
}

static bool offline_sync_type_is_list(uint8_t type)
{
    return type == ZY100_BLE_OFFLINE_FRAME_SESSION_LIST;
}

static bool offline_sync_reclaim_status_pending(void)
{
    return ((s_sync.queued_len != 0U) &&
            (s_sync.queued_type == ZY100_BLE_OFFLINE_FRAME_RECLAIM_STATUS)) ||
           (s_sync.notify_in_flight &&
            (s_sync.in_flight_type == ZY100_BLE_OFFLINE_FRAME_RECLAIM_STATUS));
}

static bool offline_sync_payload_state(void)
{
    return (s_sync.state == OFFLINE_SYNC_WAIT_CI_HIGH) ||
           (s_sync.state == OFFLINE_SYNC_WAIT_HOST_RESUME) ||
           (s_sync.state == OFFLINE_SYNC_STREAMING) ||
           (s_sync.state == OFFLINE_SYNC_WAIT_CHUNK_ACK) ||
           (s_sync.state == OFFLINE_SYNC_VERIFY_RESUME) ||
           (s_sync.state == OFFLINE_SYNC_WAIT_FINAL_CONFIRM) ||
           (s_sync.state == OFFLINE_SYNC_CONFIRMING) ||
           (s_sync.state == OFFLINE_SYNC_WAIT_CI_BALANCED);
}

static void offline_sync_reset_control_frame(void)
{
    s_sync.queued_len = 0U;
    s_sync.queued_type = 0U;
    s_sync.queued_since_ms = 0U;
    s_sync.notify_in_flight = false;
    s_sync.in_flight_type = 0U;
    s_sync.in_flight_since_ms = 0U;
}

static void offline_sync_finish_clear_cancel(void)
{
    offline_sync_reset_control_frame();
    app_ble_export_ui_cancel("offline_v2_clear_cancel");
    if (app_ble_power_is_connected() &&
        app_ble_power_conn_valid(s_sync.conn_id) &&
        app_ble_ci_state_host_session_managed())
    {
        offline_sync_enter_wait_ci_balanced(
            "offline_v2_clear_restore_balanced");
    }
    else
    {
        s_sync.state = OFFLINE_SYNC_IDLE;
        s_sync.conn_id = OFFLINE_SYNC_CONN_INVALID;
    }
}

static void offline_sync_control_frame_failed(uint8_t type,
                                              const char *reason)
{
    offline_sync_reset_control_frame();
    if ((type == ZY100_BLE_OFFLINE_FRAME_RECLAIM_STATUS) &&
        ((s_sync.state == OFFLINE_SYNC_CONFIRMING) ||
         (s_sync.state == OFFLINE_SYNC_WAIT_CI_BALANCED)))
    {
        /* A status reply failure must not undo the accepted confirmation. */
        return;
    }
    if (offline_sync_type_is_list(type) ||
        !offline_sync_payload_state())
    {
        s_sync.state = OFFLINE_SYNC_IDLE;
        s_sync.conn_id = OFFLINE_SYNC_CONN_INVALID;
        app_ble_export_ui_cancel(
            (reason != NULL) ? reason : "offline_v2_control_failed");
        return;
    }
    offline_sync_enter_wait_resume(reason);
}

static bool offline_sync_queue_abort(uint32_t reason, uint32_t detail)
{
    uint8_t *payload = &s_sync.frame[8];

    memset(payload, 0, 24U);
    zy100_put_u32_le(&payload[0], s_sync.session.session_id);
    zy100_put_u32_le(&payload[4], s_sync.transfer_id);
    zy100_put_u32_le(&payload[8], s_sync.session.generation);
    zy100_put_u32_le(&payload[12], s_sync.acked_offset);
    zy100_put_u32_le(&payload[16], reason);
    zy100_put_u32_le(&payload[20], detail);
    return offline_sync_queue_frame(
        ZY100_BLE_OFFLINE_FRAME_SYNC_ABORT,
        OFFLINE_SYNC_FLAG_LAST_FRAGMENT, payload, 24U);
}

static void offline_sync_enter_wait_resume(const char *reason)
{
    s_sync.host_resume_verified = false;
    s_sync.state = OFFLINE_SYNC_WAIT_RESUME;
    if (app_ble_power_is_connected() &&
        app_ble_power_conn_valid(s_sync.conn_id) &&
        app_ble_ci_state_host_session_managed())
    {
        (void)app_ble_ci_state_request(
            APP_BLE_CI_STATE_ACTIVE_IDLE,
            "offline_v2_error_restore_balanced");
    }
    app_ble_export_ui_cancel(
        (reason != NULL) ? reason : "offline_v2_wait_resume");
}

static bool offline_sync_queue_reclaim_status(uint32_t session_id,
                                              uint32_t generation,
                                              uint8_t state,
                                              uint32_t erased_bytes,
                                              uint32_t extent_bytes)
{
    uint8_t *payload = &s_sync.frame[8];

    /* The shared payload still belongs to the queued/in-flight frame. */
    if ((s_sync.queued_len != 0U) || s_sync.notify_in_flight)
    {
        return false;
    }
    memset(payload, 0, 24U);
    zy100_put_u32_le(&payload[0], session_id);
    zy100_put_u32_le(&payload[4], generation);
    payload[8] = state;
    zy100_put_u32_le(&payload[12], erased_bytes);
    zy100_put_u32_le(&payload[16], extent_bytes);
    zy100_put_u32_le(&payload[20], 0U);
    return offline_sync_queue_frame(
        ZY100_BLE_OFFLINE_FRAME_RECLAIM_STATUS,
        OFFLINE_SYNC_FLAG_LAST_FRAGMENT, payload, 24U);
}

static bool offline_sync_read_session_pages(
    const zy100_offline_v2_session_info_t *info,
    uint8_t begin[256],
    uint8_t end[256])
{
    bool begin_contract_valid;
    bool pages_valid;

    if ((info == NULL) || !info->finalized)
    {
        DBG_DIRECT("[OFFLINE_V2][READ_ERR] stage=info");
        return false;
    }
    if (info->end_exclusive < (info->begin_addr + 4096U))
    {
        DBG_DIRECT("[OFFLINE_V2][READ_ERR] stage=range session=%u", info->session_id);
        return false;
    }
    if (!zy100_offline_v2_storage_read(info->begin_addr, begin, 256U))
    {
        DBG_DIRECT("[OFFLINE_V2][READ_ERR] stage=begin session=%u addr=%x",
                   info->session_id, info->begin_addr);
        return false;
    }
    if (!zy100_offline_v2_storage_read(info->end_exclusive - 4096U, end, 256U))
    {
        DBG_DIRECT("[OFFLINE_V2][READ_ERR] stage=end session=%u addr=%x",
                   info->session_id, info->end_exclusive - 4096U);
        return false;
    }
    begin_contract_valid =
        ((zy100_get_u16_le(&begin[0x04]) ==
          ZY100_OFFLINE_V2_BEGIN_LEGACY_VERSION) &&
         (begin[0x44] == ZY100_OFFLINE_V2_MANIFEST_LEGACY_VERSION)) ||
        ((zy100_get_u16_le(&begin[0x04]) ==
          ZY100_OFFLINE_V2_BEGIN_VERSION) &&
         (begin[0x44] == ZY100_OFFLINE_V2_MANIFEST_VERSION) &&
         (zy100_get_u32_le(
              &begin[ZY100_OFFLINE_V2_BEGIN_OWNER_USER_ID_OFFSET]) != 0U));
    pages_valid = begin_contract_valid &&
           (((info->manifest_version ==
              ZY100_OFFLINE_V2_MANIFEST_LEGACY_VERSION) &&
             (info->owner_user_id == 0U)) ||
            ((info->manifest_version == ZY100_OFFLINE_V2_MANIFEST_VERSION) &&
             (info->owner_user_id != 0U) &&
             (zy100_get_u32_le(
                  &begin[ZY100_OFFLINE_V2_BEGIN_OWNER_USER_ID_OFFSET]) ==
              info->owner_user_id))) &&
           (zy100_get_u32_le(&begin[0x00]) == 0x3253464FUL) &&
           (zy100_get_u32_le(&begin[0x08]) == info->session_id) &&
           (zy100_get_u32_le(&begin[0x0C]) == info->generation) &&
           (zy100_get_u16_le(&begin[0x1C]) ==
            ZY100_OFFLINE_V2_EVENT_RECORD_BYTES) &&
           (zy100_get_u32_le(&begin[0x3C]) ==
            ZY100_OFFLINE_V2_CONFIG_CRC32) &&
           (begin[0x45] == ZY100_OFFLINE_V2_EVENT_VERSION) &&
           (zy100_get_u32_le(&end[0x00]) == 0x3244464FUL) &&
           (zy100_get_u16_le(&end[0x04]) == 2U) &&
           (zy100_get_u32_le(&end[0x08]) == info->session_id) &&
           (zy100_get_u32_le(&end[0x0C]) == info->generation) &&
           (zy100_get_u32_le(&end[0x4C]) ==
            zy100_get_u32_le(&begin[0x3C]));
    if (!pages_valid)
    {
        DBG_DIRECT("[OFFLINE_V2][READ_ERR] stage=contract session=%u begin=%x end=%x",
                   info->session_id, zy100_get_u32_le(&begin[0]),
                   zy100_get_u32_le(&end[0]));
    }
    return pages_valid;
}

static bool offline_sync_queue_list(uint32_t generation,
                                    uint32_t cursor,
                                    uint32_t requested,
                                    uint32_t *next_cursor_out)
{
    uint8_t *payload = &s_sync.frame[8];
    uint32_t user_id = offline_sync_current_user_id();
    uint32_t total = zy100_offline_v2_storage_session_count_for_user(user_id);
    uint32_t maximum = requested;
    uint32_t count = 0U;
    uint32_t index;
    uint32_t next_cursor;
    uint32_t crc;

    if (maximum == 0U || maximum > OFFLINE_SYNC_LIST_MAX_ENTRIES)
    {
        maximum = OFFLINE_SYNC_LIST_MAX_ENTRIES;
    }
    if (generation == 0U)
    {
        if (cursor != 0U) return false;
        s_sync.list_revision = zy100_offline_v2_storage_directory_revision();
        s_sync.list_expected_cursor = 0U;
        s_sync.list_walk_valid = true;
        s_sync.list_generation++;
        if (s_sync.list_generation == 0U) s_sync.list_generation = 1U;
        generation = s_sync.list_generation;
    }
    else if (generation != s_sync.list_generation || !s_sync.list_walk_valid ||
             s_sync.list_revision != zy100_offline_v2_storage_directory_revision())
    {
        DBG_DIRECT("[OFFLINE_V2][LIST_ERR] stage=generation got=%u expected=%u",
                   generation, s_sync.list_generation);
        return false;
    }
    if (cursor > total || cursor != s_sync.list_expected_cursor)
    {
        DBG_DIRECT("[OFFLINE_V2][LIST_ERR] stage=cursor got=%u total=%u", cursor, total);
        return false;
    }

    memset(payload, 0, OFFLINE_SYNC_LIST_HEADER_BYTES +
                       OFFLINE_SYNC_LIST_MAX_ENTRIES *
                       OFFLINE_SYNC_LIST_ENTRY_BYTES);
    for (index = cursor; (index < total) && (count < maximum); index++)
    {
        zy100_offline_v2_session_info_t info;
        uint8_t *entry = &payload[OFFLINE_SYNC_LIST_HEADER_BYTES +
                                  count * OFFLINE_SYNC_LIST_ENTRY_BYTES];

        if (!zy100_offline_v2_storage_session_get_for_user(user_id, total - index - 1U, &info))
        {
            DBG_DIRECT("[OFFLINE_V2][LIST_ERR] stage=lookup index=%u", index);
            return false;
        }
        zy100_put_u32_le(&entry[0], info.session_id);
        zy100_put_u32_le(&entry[4], info.generation);
        entry[8] = info.confirmed ? 2U : 1U;
        entry[9] = info.clean ? OFFLINE_SYNC_LIST_HEALTH_CLEAN : 0U;
        if (info.health != ZY100_OFFLINE_V2_HEALTH_NORMAL)
        {
            entry[9] |= OFFLINE_SYNC_LIST_HEALTH_ATTENTION;
        }
        if (info.health == ZY100_OFFLINE_V2_HEALTH_RECOVERED_PREFIX)
        {
            entry[9] |= OFFLINE_SYNC_LIST_HEALTH_RECOVERED;
        }
        zy100_put_u16_le(&entry[10], info.manifest_version);
        zy100_put_u32_le(&entry[12], info.event_count);
        zy100_put_u32_le(&entry[16], info.logical_bytes);
        zy100_put_u32_le(&entry[20], info.stream_crc32);
        zy100_put_u64_le(&entry[24], ((uint64_t)info.start_unix_hi << 32) | info.start_unix_lo);
        zy100_put_u32_le(&entry[32], info.duration_ms);
        zy100_put_u32_le(&entry[36], ZY100_OFFLINE_V2_CONFIG_CRC32);
        count++;
    }
    next_cursor = ((cursor + count) < total) ?
                  (cursor + count) : 0xFFFFFFFFUL;
    zy100_put_u32_le(&payload[0], generation);
    zy100_put_u32_le(&payload[4], cursor);
    zy100_put_u16_le(&payload[8], (uint16_t)total);
    zy100_put_u16_le(&payload[10], (uint16_t)count);
    zy100_put_u32_le(&payload[12], next_cursor);
    zy100_put_u32_le(&payload[16], 0U);
    crc = zy100_crc32_ieee(payload,
        OFFLINE_SYNC_LIST_HEADER_BYTES +
        count * OFFLINE_SYNC_LIST_ENTRY_BYTES);
    zy100_put_u32_le(&payload[16], crc);
    if (!offline_sync_queue_frame(
            ZY100_BLE_OFFLINE_FRAME_SESSION_LIST,
            (next_cursor == 0xFFFFFFFFUL) ?
                OFFLINE_SYNC_FLAG_LIST_LAST : 0U,
            payload,
            (uint16_t)(OFFLINE_SYNC_LIST_HEADER_BYTES +
                       count * OFFLINE_SYNC_LIST_ENTRY_BYTES)))
    {
        DBG_DIRECT("[OFFLINE_V2][LIST_ERR] stage=queue count=%u", count);
        return false;
    }
    s_sync.list_pending_cursor = next_cursor;
    if (next_cursor_out != NULL) *next_cursor_out = next_cursor;
    return true;
}

static uint16_t offline_sync_build_manifest(
    uint8_t payload[OFFLINE_SYNC_MANIFEST_MAX_BYTES],
    const uint8_t begin[256],
    const uint8_t end[256])
{
    uint32_t manifest_crc;
    uint32_t physical_span;
    uint32_t owner_user_id = 0U;
    uint16_t manifest_bytes;
    uint16_t manifest_crc_offset;
    uint16_t manifest_version;
    uint8_t flags = 0U;

    manifest_version = begin[0x44];
    if (manifest_version == ZY100_OFFLINE_V2_MANIFEST_LEGACY_VERSION)
    {
        manifest_bytes = ZY100_OFFLINE_V2_MANIFEST_LEGACY_BYTES;
        manifest_crc_offset = 116U;
    }
    else if (manifest_version == ZY100_OFFLINE_V2_MANIFEST_VERSION)
    {
        manifest_bytes = ZY100_OFFLINE_V2_MANIFEST_BYTES;
        manifest_crc_offset = ZY100_OFFLINE_V2_MANIFEST_CRC_OFFSET;
        owner_user_id = zy100_get_u32_le(
            &begin[ZY100_OFFLINE_V2_BEGIN_OWNER_USER_ID_OFFSET]);
        if (owner_user_id == 0U)
        {
            return 0U;
        }
    }
    else
    {
        return 0U;
    }
    physical_span = zy100_get_u32_le(&end[0x18]);
    if ((zy100_get_u16_le(&begin[0x22]) & 1U) != 0U) flags |= 1U;
    if (zy100_get_u16_le(&end[0x2C]) ==
        ZY100_OFFLINE_V2_STOP_POWER_LOSS_RECOVERED) flags |= 2U;
    memset(payload, 0, OFFLINE_SYNC_MANIFEST_MAX_BYTES);
    zy100_put_u32_le(&payload[0], s_sync.session.session_id);
    zy100_put_u32_le(&payload[4], s_sync.transfer_id);
    zy100_put_u32_le(&payload[8], s_sync.session.generation);
    zy100_put_u16_le(&payload[12], begin[0x44]);
    zy100_put_u16_le(&payload[14], zy100_get_u16_le(&begin[0x1C]));
    zy100_put_u16_le(&payload[16], ZY100_OFFLINE_V2_EVENT_SLOT_BYTES);
    zy100_put_u16_le(&payload[18], 128U);
    payload[20] = 12U;
    payload[21] = flags;
    zy100_put_u32_le(&payload[24], 800U);
    zy100_put_u32_le(&payload[28], s_sync.session.event_count);
    zy100_put_u32_le(&payload[32], s_sync.session.logical_bytes);
    zy100_put_u32_le(&payload[36], s_sync.session.stream_crc32);
    zy100_put_u64_le(&payload[40], zy100_get_u64_le(&begin[0x10]));
    zy100_put_u32_le(&payload[48], zy100_get_u32_le(&end[0x28]));
    zy100_put_u32_le(&payload[52], zy100_get_u32_le(&begin[0x24]));
    zy100_put_u32_le(&payload[56], zy100_get_u32_le(&begin[0x28]));
    zy100_put_u32_le(&payload[60], zy100_get_u32_le(&begin[0x2C]));
    zy100_put_u32_le(&payload[64], zy100_get_u32_le(&begin[0x30]));
    zy100_put_u32_le(&payload[68], zy100_get_u32_le(&begin[0x34]));
    zy100_put_u32_le(&payload[72], physical_span);
    zy100_put_u32_le(&payload[76],
                     s_sync.session.end_exclusive -
                     s_sync.session.begin_addr);
    zy100_put_u16_le(&payload[80], (uint16_t)s_sync.session.stop_reason);
    payload[82] = s_sync.session.clean ? 1U : 0U;
    payload[83] = (uint8_t)s_sync.session.health;
    zy100_put_u32_le(&payload[84],
                     s_sync.session.quality.fifo_overflow_count);
    zy100_put_u32_le(&payload[88],
                     s_sync.session.quality.fifo_discard_count);
    zy100_put_u32_le(&payload[92],
                     s_sync.session.quality.time_gap_count);
    zy100_put_u32_le(&payload[96],
                     s_sync.session.quality.feature_drop_count);
    zy100_put_u32_le(&payload[100],
                     s_sync.session.quality.q12_clip_event_count);
    zy100_put_u32_le(&payload[104],
                     s_sync.session.quality.flash_error_count);
    zy100_put_u32_le(&payload[4], 0U);
    zy100_put_u32_le(&payload[108], zy100_get_u32_le(&begin[0x38]));
    zy100_put_u32_le(&payload[112], zy100_get_u32_le(&begin[0x3C]));
    if (manifest_version == ZY100_OFFLINE_V2_MANIFEST_VERSION)
    {
        zy100_put_u32_le(
            &payload[ZY100_OFFLINE_V2_MANIFEST_OWNER_USER_ID_OFFSET],
            owner_user_id);
    }
    zy100_put_u32_le(&payload[manifest_crc_offset], 0U);
    manifest_crc = zy100_crc32_ieee(payload, manifest_bytes);
    zy100_put_u32_le(&payload[4], s_sync.transfer_id);
    zy100_put_u32_le(&payload[manifest_crc_offset], manifest_crc);
    s_sync.manifest_crc = manifest_crc;
    return manifest_bytes;
}

static bool offline_sync_queue_begin(void)
{
    uint8_t *payload = &s_sync.frame[8];
    uint8_t end[256];
    uint16_t manifest_bytes;

    if (!offline_sync_read_session_pages(&s_sync.session,
                                         s_sync.page, end))
    {
        return false;
    }
    manifest_bytes = offline_sync_build_manifest(payload, s_sync.page, end);
    if (manifest_bytes == 0U)
    {
        return false;
    }
    return offline_sync_queue_frame(
        ZY100_BLE_OFFLINE_FRAME_SESSION_BEGIN,
        OFFLINE_SYNC_FLAG_LAST_FRAGMENT,
        payload,
        manifest_bytes);
}

static bool offline_sync_read_logical(uint32_t offset,
                                      uint8_t *data,
                                      uint16_t length)
{
    uint16_t copied = 0U;
    uint32_t record_bytes;

    record_bytes = (s_sync.session.event_count != 0U) ?
        (s_sync.session.logical_bytes / s_sync.session.event_count) :
        ZY100_OFFLINE_V2_EVENT_RECORD_BYTES;
    if ((record_bytes != ZY100_OFFLINE_V2_EVENT_RECORD_BYTES) ||
        ((s_sync.session.event_count != 0U) &&
         ((s_sync.session.logical_bytes % s_sync.session.event_count) != 0U)))
    {
        return false;
    }

    while (copied < length)
    {
        uint32_t logical = offset + copied;
        uint32_t event_index = logical /
                               record_bytes;
        uint32_t in_record = logical %
                             record_bytes;
        uint16_t part = (uint16_t)(record_bytes -
                                   in_record);
        uint32_t address;

        if (part > (uint16_t)(length - copied))
            part = (uint16_t)(length - copied);
        address = s_sync.session.begin_addr + 256U +
                  event_index * ZY100_OFFLINE_V2_EVENT_SLOT_BYTES +
                  in_record;
        if (!zy100_offline_v2_storage_read(address, &data[copied], part))
            return false;
        copied = (uint16_t)(copied + part);
    }
    return true;
}

static bool offline_sync_queue_chunk(void)
{
    uint8_t *payload = &s_sync.frame[8];
    uint32_t remaining = s_sync.session.logical_bytes - s_sync.sent_offset;
    uint32_t unacked = s_sync.sent_offset - s_sync.acked_offset;
    uint32_t window_remaining =
        (unacked <= OFFLINE_SYNC_MAX_UNACKED_BYTES) ?
        (OFFLINE_SYNC_MAX_UNACKED_BYTES - unacked) : 0U;
    uint16_t length = (remaining > OFFLINE_SYNC_CHUNK_DATA_BYTES) ?
                      OFFLINE_SYNC_CHUNK_DATA_BYTES : (uint16_t)remaining;
    uint32_t next_crc_state;
    uint32_t prefix_crc;

    if (length > window_remaining)
    {
        length = (uint16_t)window_remaining;
    }
    if ((unacked > OFFLINE_SYNC_MAX_UNACKED_BYTES) || (length == 0U) ||
        !offline_sync_read_logical(s_sync.sent_offset, &payload[28], length))
    {
        return false;
    }
    zy100_put_u32_le(&payload[0], s_sync.session.session_id);
    zy100_put_u32_le(&payload[4], s_sync.transfer_id);
    zy100_put_u32_le(&payload[8], s_sync.sent_offset);
    zy100_put_u32_le(&payload[12], s_sync.session.logical_bytes);
    zy100_put_u16_le(&payload[16], length);
    zy100_put_u16_le(&payload[18], 0U);
    zy100_put_u32_le(&payload[20], zy100_crc32_ieee(&payload[28], length));
    next_crc_state = zy100_crc32_ieee_update(s_sync.sent_crc_state,
                                             &payload[28], length);
    prefix_crc = zy100_crc32_ieee_finish(next_crc_state);
    zy100_put_u32_le(&payload[24], prefix_crc);
    if (!offline_sync_queue_frame(
            ZY100_BLE_OFFLINE_FRAME_SESSION_CHUNK,
            (uint8_t)((((s_sync.sent_offset + length) ==
                        s_sync.session.logical_bytes) ?
                       OFFLINE_SYNC_FLAG_LAST_FRAGMENT : 0U) |
                      (s_sync.retransmitting ?
                       OFFLINE_SYNC_FLAG_RETRANSMIT : 0U)),
            payload, (uint16_t)(OFFLINE_SYNC_CHUNK_HEADER_BYTES + length)))
    {
        return false;
    }
    s_sync.sent_crc_state = next_crc_state;
    s_sync.sent_offset += length;
    s_sync.chunk_count++;
    if (((s_sync.sent_offset - s_sync.acked_offset) >=
         OFFLINE_SYNC_MAX_UNACKED_BYTES) ||
        (s_sync.sent_offset == s_sync.session.logical_bytes))
    {
        s_sync.state = OFFLINE_SYNC_WAIT_CHUNK_ACK;
        s_sync.ack_progress_pending = true;
        ZY100_LOG_DETAIL("[OFF_WINDOW] phase=wait_ack sid=%lu from=%lu to=%lu final=%u",
                   (unsigned long)s_sync.session.session_id,
                   (unsigned long)s_sync.acked_offset,
                   (unsigned long)s_sync.sent_offset,
                   (s_sync.sent_offset ==
                    s_sync.session.logical_bytes) ? 1U : 0U);
    }
    return true;
}

static bool offline_sync_queue_end(void)
{
    uint8_t *payload = &s_sync.frame[8];
    uint8_t end[256];
    uint32_t manifest_crc;

    if (!offline_sync_read_session_pages(&s_sync.session,
                                         s_sync.page, end)) return false;
    if (offline_sync_build_manifest(payload, s_sync.page, end) == 0U)
    {
        return false;
    }
    manifest_crc = s_sync.manifest_crc;
    memset(payload, 0, 44U);
    zy100_put_u32_le(&payload[0], s_sync.session.session_id);
    zy100_put_u32_le(&payload[4], s_sync.transfer_id);
    zy100_put_u32_le(&payload[8], s_sync.session.generation);
    zy100_put_u32_le(&payload[12], s_sync.session.logical_bytes);
    zy100_put_u32_le(&payload[16], s_sync.session.event_count);
    zy100_put_u32_le(&payload[20], s_sync.session.stream_crc32);
    zy100_put_u32_le(&payload[24], manifest_crc);
    zy100_put_u32_le(&payload[28], zy100_get_u32_le(&end[0x18]));
    zy100_put_u32_le(&payload[32], s_sync.session.end_exclusive -
                                   s_sync.session.begin_addr);
    zy100_put_u32_le(&payload[36], s_sync.chunk_count);
    zy100_put_u32_le(&payload[40], 0U);
    return offline_sync_queue_frame(
        ZY100_BLE_OFFLINE_FRAME_SESSION_END,
        OFFLINE_SYNC_FLAG_LAST_FRAGMENT, payload, 44U);
}

static bool offline_sync_select_session(uint32_t session_id,
                                        uint32_t generation)
{
    zy100_offline_v2_session_info_t info;
    if (zy100_offline_v2_storage_latest_for_user(offline_sync_current_user_id(), &info) &&
        info.session_id == session_id && info.generation == generation && info.finalized && !info.confirmed)
    { s_sync.session = info; return true; }
    return false;
}

static void offline_sync_handle_foreign_purge(
    const zy100_ble_cmd_frame_t *command,
    app_ble_offline_v2_sync_reply_t *reply)
{
    zy100_offline_v2_foreign_purge_result_t purge_result;
    uint32_t remaining_sessions = 0U;
    uint32_t total_sectors = 0U;
    uint32_t erased_sectors = 0U;
    uint32_t batch_token = 0U;
    uint32_t current_user_id = offline_sync_current_user_id();

    if ((command->user_id_le == 0U) ||
        (command->user_id_le != current_user_id) ||
        (command->device_time_ms_le != 0ULL) ||
        (command->training_id_le != 0U))
    {
        reply->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_BAD_PURGE_ARGS;
        return;
    }
    if (!zy100_offline_v2_storage_foreign_purge_active() &&
        !app_ble_offline_v2_sync_foreign_purge_admitted())
    {
        reply->status = ZY100_BLE_ACK_STATUS_BUSY;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_OWNER_BUSY;
        return;
    }
    purge_result = zy100_offline_v2_storage_foreign_purge_request(
        current_user_id, command->seq);
    if (purge_result == ZY100_OFFLINE_V2_FOREIGN_PURGE_NO_ACTION)
    {
        reply->status = ZY100_BLE_ACK_STATUS_OK;
        reply->exec_mode = ZY100_BLE_EXEC_MODE_DRY_RUN_NO_ACTION;
        reply->device_state = ZY100_BLE_DEVICE_STATE_OFFLINE_SESSION_READY;
        reply->training_id_echo = 0U;
        reply->detail = 0U;
        return;
    }
    if (purge_result == ZY100_OFFLINE_V2_FOREIGN_PURGE_OWNER_MISMATCH)
    {
        reply->status = ZY100_BLE_ACK_STATUS_BUSY;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_OWNER_BUSY;
        return;
    }
    if (purge_result == ZY100_OFFLINE_V2_FOREIGN_PURGE_ERROR)
    {
        reply->status = ZY100_BLE_ACK_STATUS_INTERNAL_ERROR;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_FLASH_IO;
        reply->device_state = ZY100_BLE_DEVICE_STATE_ERROR;
        return;
    }
    if ((purge_result != ZY100_OFFLINE_V2_FOREIGN_PURGE_ACTIVE) ||
        !zy100_offline_v2_storage_foreign_purge_status(
            current_user_id, &remaining_sessions, &total_sectors,
            &erased_sectors, &batch_token))
    {
        reply->status = ZY100_BLE_ACK_STATUS_BUSY;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_OWNER_BUSY;
        return;
    }
    (void)batch_token;
    reply->status = ZY100_BLE_ACK_STATUS_OK;
    reply->exec_mode = ZY100_BLE_EXEC_MODE_ACCEPTED_ASYNC;
    reply->device_state = ZY100_BLE_DEVICE_STATE_OFFLINE_RECLAIMING;
    reply->training_id_echo = remaining_sessions;
    reply->detail = offline_sync_foreign_purge_detail(
        total_sectors, erased_sectors);
}

static void offline_sync_handle_session_list(
    uint8_t conn_id,
    const zy100_ble_cmd_frame_t *command,
    uint32_t low,
    app_ble_offline_v2_sync_reply_t *reply)
{
    uint32_t next_cursor = 0U;

    if ((s_sync.state != OFFLINE_SYNC_IDLE) ||
        (s_sync.queued_len != 0U) || s_sync.notify_in_flight)
    {
        reply->status = ZY100_BLE_ACK_STATUS_BUSY;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_OWNER_BUSY;
        return;
    }
    if (!offline_sync_queue_list(command->user_id_le, low,
                                 command->training_id_le,
                                 &next_cursor))
    {
        reply->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_STALE_TRANSFER;
        return;
    }
    s_sync.conn_id = conn_id;
    reply->status = ZY100_BLE_ACK_STATUS_OK;
    reply->exec_mode = ZY100_BLE_EXEC_MODE_ACCEPTED_ASYNC;
    reply->user_id_echo = s_sync.list_generation;
    reply->training_id_echo = next_cursor;
    reply->detail = offline_sync_owned_session_count();
}

static void offline_sync_handle_session_begin(
    uint8_t conn_id,
    const zy100_ble_cmd_frame_t *command,
    uint32_t high,
    uint32_t low,
    app_ble_offline_v2_sync_reply_t *reply)
{
    uint8_t ci_request_result;

    if ((s_sync.queued_len != 0U) || s_sync.notify_in_flight)
    {
        reply->status = ZY100_BLE_ACK_STATUS_BUSY;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_OWNER_BUSY;
        return;
    }
    if ((low != 0U) || (command->training_id_le != 0U) ||
        !offline_sync_select_session(command->user_id_le, high))
    {
        reply->status = ZY100_BLE_ACK_STATUS_NOT_READY;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_NOT_FOUND;
        return;
    }
    s_sync.conn_id = conn_id;
    s_sync.transfer_id = ++s_sync.transfer_seed;
    if (s_sync.transfer_id == 0U)
        s_sync.transfer_id = ++s_sync.transfer_seed;
    s_sync.sent_offset = 0U;
    s_sync.acked_offset = 0U;
    s_sync.sent_crc_state = zy100_crc32_ieee_begin();
    s_sync.acked_crc_state = s_sync.sent_crc_state;
    s_sync.acked_prefix_crc = zy100_crc32_ieee_finish(
        s_sync.acked_crc_state);
    s_sync.chunk_count = 0U;
    s_sync.chunk_seq = 0U;
    s_sync.retransmitting = false;
    s_sync.host_resume_verified = false;
    s_sync.ack_progress_pending = true;
    s_sync.state = OFFLINE_SYNC_WAIT_CI_HIGH;
    ci_request_result = app_ble_ci_state_request(
        APP_BLE_CI_STATE_CAPTURE_RUNTIME,
        "offline_v2_transfer");
    if ((ci_request_result ==
         (uint8_t)APP_BLE_CONN_PARAM_REQ_FAILED) ||
        (ci_request_result ==
         (uint8_t)APP_BLE_CONN_PARAM_REQ_BUSY))
    {
        s_sync.state = OFFLINE_SYNC_IDLE;
        reply->status = ZY100_BLE_ACK_STATUS_NOT_READY;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_OWNER_BUSY;
        return;
    }
    ZY100_LOG_DETAIL("[OFF_CI] phase=wait_high sid=%lu tid=%lu request=%lu",
               (unsigned long)s_sync.session.session_id,
               (unsigned long)s_sync.transfer_id,
               (unsigned long)ci_request_result);
    app_ble_export_ui_begin_upload("offline_v2_sync");
    reply->status = ZY100_BLE_ACK_STATUS_OK;
    reply->exec_mode = ZY100_BLE_EXEC_MODE_ACCEPTED_ASYNC;
    reply->training_id_echo = s_sync.acked_offset;
    reply->detail = s_sync.transfer_id;
    reply->device_state = ZY100_BLE_DEVICE_STATE_OFFLINE_SYNCING;
}

static void offline_sync_handle_chunk_ack(
    uint8_t conn_id,
    const zy100_ble_cmd_frame_t *command,
    uint32_t high,
    uint32_t low,
    app_ble_offline_v2_sync_reply_t *reply)
{
    bool streaming = (s_sync.state == OFFLINE_SYNC_STREAMING) ||
                     (s_sync.state == OFFLINE_SYNC_WAIT_CHUNK_ACK);

    /* An ACK confirms progress in this transfer; it never resumes an aborted
     * transfer or bypasses the initial RESUME/CI barriers. */
    if ((conn_id != s_sync.conn_id) ||
        (s_sync.conn_id == OFFLINE_SYNC_CONN_INVALID) ||
        (s_sync.transfer_id == 0U) ||
        (command->user_id_le != s_sync.session.session_id) ||
        (high != s_sync.transfer_id) ||
        (low != s_sync.sent_offset) ||
        (low < s_sync.acked_offset) ||
        (command->training_id_le !=
         zy100_crc32_ieee_finish(s_sync.sent_crc_state)) ||
        !s_sync.host_resume_verified ||
        (!streaming &&
         !((s_sync.state == OFFLINE_SYNC_WAIT_FINAL_CONFIRM) &&
           (low == s_sync.session.logical_bytes) &&
           (low == s_sync.acked_offset))) ||
        ((low == s_sync.acked_offset) &&
         (command->training_id_le != s_sync.acked_prefix_crc)))
    {
        reply->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_OFFSET;
        return;
    }
    if (low == s_sync.acked_offset)
    {
        /* Repeated confirmation is not progress: keep the timeout, queued
         * frames and terminal phase intact, including a pending END. */
        reply->status = ZY100_BLE_ACK_STATUS_OK;
        reply->exec_mode = ZY100_BLE_EXEC_MODE_DRY_RUN_NO_ACTION;
        reply->training_id_echo = low;
        reply->detail = s_sync.transfer_id;
        return;
    }
    s_sync.acked_offset = low;
    s_sync.acked_crc_state = s_sync.sent_crc_state;
    s_sync.acked_prefix_crc = command->training_id_le;
    s_sync.ack_progress_pending = true;
    s_sync.state = OFFLINE_SYNC_STREAMING;
    ZY100_LOG_DETAIL("[OFF_WINDOW] phase=ack_ok sid=%lu offset=%lu crc=%08lX",
               (unsigned long)s_sync.session.session_id,
               (unsigned long)low,
               (unsigned long)command->training_id_le);
    reply->status = ZY100_BLE_ACK_STATUS_OK;
    reply->exec_mode = ZY100_BLE_EXEC_MODE_REAL_ACTION;
    reply->training_id_echo = low;
    reply->detail = s_sync.transfer_id;
}

static void offline_sync_handle_session_resume(
    uint8_t conn_id,
    const zy100_ble_cmd_frame_t *command,
    uint32_t high,
    uint32_t low,
    app_ble_offline_v2_sync_reply_t *reply)
{
    bool initial_resume_barrier =
        (s_sync.state == OFFLINE_SYNC_WAIT_HOST_RESUME);
    uint8_t ci_request_result;

    if ((command->user_id_le != s_sync.session.session_id) ||
        ((high != s_sync.transfer_id) &&
         !((high == 0U) && (low == 0U))) ||
        (low > s_sync.session.logical_bytes) ||
        (s_sync.session.restart_from_zero && low != 0U) ||
        (s_sync.queued_len != 0U) ||
        (!initial_resume_barrier &&
         (s_sync.state != OFFLINE_SYNC_WAIT_RESUME)))
    {
        reply->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_OFFSET;
        return;
    }
    if (!initial_resume_barrier)
    {
        ci_request_result = app_ble_ci_state_request(
            APP_BLE_CI_STATE_CAPTURE_RUNTIME,
            "offline_v2_resume_transfer");
        if ((ci_request_result ==
             (uint8_t)APP_BLE_CONN_PARAM_REQ_FAILED) ||
            (ci_request_result ==
             (uint8_t)APP_BLE_CONN_PARAM_REQ_BUSY))
        {
            reply->status = ZY100_BLE_ACK_STATUS_NOT_READY;
            reply->detail = ZY100_BLE_OFFLINE_DETAIL_OWNER_BUSY;
            return;
        }
    }
    s_sync.conn_id = conn_id;
    s_sync.verify_offset = 0U;
    s_sync.verify_target = low;
    s_sync.verify_crc_state = zy100_crc32_ieee_begin();
    s_sync.verify_expected_crc = command->training_id_le;
    s_sync.state = OFFLINE_SYNC_VERIFY_RESUME;
    app_ble_export_ui_begin_upload("offline_v2_resume");
    reply->status = ZY100_BLE_ACK_STATUS_OK;
    reply->exec_mode = ZY100_BLE_EXEC_MODE_ACCEPTED_ASYNC;
    reply->training_id_echo = low;
    reply->detail = s_sync.transfer_id;
    reply->device_state = ZY100_BLE_DEVICE_STATE_OFFLINE_SYNCING;
    DBG_DIRECT("[OFF_RESUME] phase=accepted sid=%lu tid=%lu offset=%lu initial=%u",
               (unsigned long)s_sync.session.session_id,
               (unsigned long)s_sync.transfer_id,
               (unsigned long)low,
               initial_resume_barrier ? 1U : 0U);
}

static void offline_sync_handle_final_confirm(
    const zy100_ble_cmd_frame_t *command,
    uint32_t high,
    uint32_t low,
    app_ble_offline_v2_sync_reply_t *reply)
{
    if (offline_sync_confirm_identity_matches(command, high, low) &&
        (s_sync.state == OFFLINE_SYNC_WAIT_FINAL_CONFIRM))
    {
        if (zy100_offline_v2_storage_job_busy())
        {
            reply->status = ZY100_BLE_ACK_STATUS_BUSY;
            reply->detail = ZY100_BLE_OFFLINE_DETAIL_OWNER_BUSY;
            offline_sync_log_confirm("current_busy", reply->status,
                                     high, low,
                                     command->training_id_le);
            return;
        }
        if (!zy100_offline_v2_storage_confirm_session(
                s_sync.session.session_id,
                s_sync.session.generation,
                offline_sync_current_user_id(),
                s_sync.transfer_id,
                s_sync.session.logical_bytes,
                s_sync.session.stream_crc32))
        {
            reply->status = ZY100_BLE_ACK_STATUS_INTERNAL_ERROR;
            reply->detail = ZY100_BLE_OFFLINE_DETAIL_FLASH_IO;
            offline_sync_log_confirm("current_fail", reply->status,
                                     high, low,
                                     command->training_id_le);
            return;
        }
        s_sync.state = OFFLINE_SYNC_CONFIRMING;
        reply->status = ZY100_BLE_ACK_STATUS_OK;
        reply->exec_mode = ZY100_BLE_EXEC_MODE_ACCEPTED_ASYNC;
        reply->training_id_echo = s_sync.session.generation;
        reply->detail = 3U;
        reply->device_state =
            ZY100_BLE_DEVICE_STATE_OFFLINE_RECLAIMING;
        offline_sync_log_confirm("current", reply->status, high, low,
                                 command->training_id_le);
        return;
    }
    if (offline_sync_confirm_identity_matches(command, high, low) &&
        (s_sync.state == OFFLINE_SYNC_CONFIRMING))
    {
        reply->status = ZY100_BLE_ACK_STATUS_OK;
        reply->exec_mode = ZY100_BLE_EXEC_MODE_ACCEPTED_ASYNC;
        reply->training_id_echo = s_sync.session.generation;
        reply->detail = 3U;
        reply->device_state =
            ZY100_BLE_DEVICE_STATE_OFFLINE_RECLAIMING;
        offline_sync_log_confirm("retry", reply->status, high, low,
                                 command->training_id_le);
        return;
    }
    if ((s_sync.state == OFFLINE_SYNC_WAIT_FINAL_CONFIRM) ||
        (s_sync.state == OFFLINE_SYNC_CONFIRMING))
    {
        reply->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_STALE_TRANSFER;
        offline_sync_log_confirm("stale", reply->status, high, low,
                                 command->training_id_le);
        return;
    }
    if (zy100_offline_v2_storage_confirm_replay_matches(
            command->user_id_le, high,
            offline_sync_current_user_id(), low,
            command->training_id_le))
    {
        uint8_t reclaim_state;
        uint32_t erased_bytes;
        uint32_t extent_bytes;

        if (!zy100_offline_v2_storage_reclaim_status(
                command->user_id_le, high,
                offline_sync_current_user_id(), &reclaim_state,
                &erased_bytes, &extent_bytes))
        {
            reply->status = ZY100_BLE_ACK_STATUS_INTERNAL_ERROR;
            reply->detail = ZY100_BLE_OFFLINE_DETAIL_FLASH_IO;
            return;
        }
        reply->status = ZY100_BLE_ACK_STATUS_OK;
        reply->exec_mode = ZY100_BLE_EXEC_MODE_DRY_RUN_NO_ACTION;
        reply->training_id_echo = high;
        reply->detail = reclaim_state;
        reply->device_state =
            (reclaim_state == ZY100_OFFLINE_V2_RECLAIM_RECLAIMING) ?
            ZY100_BLE_DEVICE_STATE_OFFLINE_RECLAIMING :
            ZY100_BLE_DEVICE_STATE_OFFLINE_SESSION_READY;
        offline_sync_log_confirm("replay", reply->status, high, low,
                                 command->training_id_le);
        return;
    }
    reply->status = ZY100_BLE_ACK_STATUS_INVALID_STATE;
    reply->detail = ZY100_BLE_OFFLINE_DETAIL_STALE_TRANSFER;
    offline_sync_log_confirm("stale", reply->status, high, low,
                             command->training_id_le);
}

static void offline_sync_handle_reclaim_status(
    uint8_t conn_id,
    const zy100_ble_cmd_frame_t *command,
    uint32_t high,
    uint32_t low,
    app_ble_offline_v2_sync_reply_t *reply)
{
    uint8_t reclaim_state;
    uint32_t erased_bytes;
    uint32_t extent_bytes;

    if ((low != 0U) || (command->training_id_le != 0U) ||
        !zy100_offline_v2_storage_reclaim_status(
            command->user_id_le, high,
            offline_sync_current_user_id(), &reclaim_state,
            &erased_bytes, &extent_bytes) ||
        (reclaim_state == ZY100_OFFLINE_V2_RECLAIM_UNKNOWN))
    {
        reply->status = ZY100_BLE_ACK_STATUS_NOT_READY;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_NOT_FOUND;
        return;
    }
    if (!offline_sync_queue_reclaim_status(
            command->user_id_le, high, reclaim_state,
            erased_bytes, extent_bytes))
    {
        reply->status = ZY100_BLE_ACK_STATUS_BUSY;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_OWNER_BUSY;
        return;
    }
    s_sync.conn_id = conn_id;
    reply->status = ZY100_BLE_ACK_STATUS_OK;
    reply->exec_mode = ZY100_BLE_EXEC_MODE_ACCEPTED_ASYNC;
    reply->training_id_echo = high;
    reply->detail = reclaim_state;
}

bool app_ble_offline_v2_sync_handle_command(
    uint8_t conn_id,
    const zy100_ble_cmd_frame_t *command,
    app_ble_offline_v2_sync_reply_t *reply)
{
    uint32_t high;
    uint32_t low;

    if ((command == NULL) || (reply == NULL)) return false;
    offline_sync_reply_default(command, reply);
    if ((command->cmd < ZY100_BLE_CMD_OFFLINE_SESSION_LIST) ||
        (command->cmd > ZY100_BLE_CMD_OFFLINE_FOREIGN_PURGE)) return false;
    if (!app_ble_connection_bootstrap_user_sync_ready(conn_id))
    {
        reply->status = ZY100_BLE_ACK_STATUS_NOT_READY;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_OWNER_BUSY;
        return true;
    }
    if (s_sync.clear_cancel_requested || s_sync.preempt_requested)
    {
        reply->status = ZY100_BLE_ACK_STATUS_BUSY;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_OWNER_BUSY;
        return true;
    }
    if (command->cmd == ZY100_BLE_CMD_OFFLINE_FOREIGN_PURGE)
    {
        offline_sync_handle_foreign_purge(command, reply);
        return true;
    }
    if (!offline_sync_owner_available() &&
        (command->cmd != ZY100_BLE_CMD_OFFLINE_FINAL_CONFIRM) &&
        (command->cmd != ZY100_BLE_CMD_OFFLINE_RECLAIM_STATUS) &&
        !(command->cmd == ZY100_BLE_CMD_OFFLINE_SESSION_LIST &&
          s_sync.local_preempt_admitted && !zy100_offline_v2_capture_active() &&
          zy100_capture_profile_current() == ZY100_CAPTURE_PROFILE_NONE &&
          !zy100_offline_v2_storage_foreign_purge_active()))
    {
        reply->status = ZY100_BLE_ACK_STATUS_BUSY;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_OWNER_BUSY;
        return true;
    }
    if (!offline_sync_transport_ready(conn_id))
    {
        reply->status = ZY100_BLE_ACK_STATUS_NOT_READY;
        reply->detail = ZY100_BLE_OFFLINE_DETAIL_NOT_FINAL;
        return true;
    }
    high = (uint32_t)(command->device_time_ms_le >> 32);
    low = (uint32_t)command->device_time_ms_le;
    app_ble_offline_v2_sync_note_transfer_admitted("offline_cmd");

    switch (command->cmd)
    {
    case ZY100_BLE_CMD_OFFLINE_SESSION_LIST:
        offline_sync_handle_session_list(conn_id, command, low, reply);
        break;
    case ZY100_BLE_CMD_OFFLINE_SESSION_BEGIN:
        offline_sync_handle_session_begin(conn_id, command, high, low, reply);
        break;
    case ZY100_BLE_CMD_OFFLINE_CHUNK_ACK:
        offline_sync_handle_chunk_ack(conn_id, command, high, low, reply);
        break;
    case ZY100_BLE_CMD_OFFLINE_SESSION_RESUME:
        offline_sync_handle_session_resume(conn_id, command, high, low, reply);
        break;
    case ZY100_BLE_CMD_OFFLINE_FINAL_CONFIRM:
        offline_sync_handle_final_confirm(command, high, low, reply);
        break;
    case ZY100_BLE_CMD_OFFLINE_RECLAIM_STATUS:
        offline_sync_handle_reclaim_status(conn_id, command, high, low, reply);
        break;
    default:
        reply->status = ZY100_BLE_ACK_STATUS_UNSUPPORTED_CMD;
        break;
    }
    return true;
}

void app_ble_offline_v2_sync_poll(uint64_t now_ms)
{
    bool sent;
    uint32_t now32 = (uint32_t)now_ms;
    uint32_t purge_user_id;
    uint32_t purge_batch_token;
    uint32_t purge_total_sectors;
    bool purge_failed;

    if (s_sync.preempt_requested)
    {
        if (s_sync.notify_in_flight) return;
        if (s_sync.preempt_abort_pending)
        {
            s_sync.preempt_abort_pending = false;
            if (offline_sync_transport_ready(s_sync.conn_id) &&
                offline_sync_queue_abort(OFFLINE_SYNC_ABORT_LOCAL_CAPTURE, 0U) &&
                zy100_ble_ctrl_service_send_export_notify(s_sync.conn_id, s_sync.frame, s_sync.queued_len))
            {
                s_sync.notify_in_flight = true;
                s_sync.in_flight_type = s_sync.queued_type;
                s_sync.in_flight_since_ms = now32;
            }
            s_sync.queued_len = 0U;
            s_sync.queued_type = 0U;
        }
        return;
    }
    if (s_sync.local_preempt_admitted && !zy100_offline_v2_capture_active() &&
        zy100_capture_profile_current() == ZY100_CAPTURE_PROFILE_NONE &&
        !s_sync.notify_in_flight && s_sync.queued_len == 0U)
        zy100_offline_v2_storage_resume_reclaim(offline_sync_current_user_id());
    offline_sync_arb_log_if_changed("poll");

    if (zy100_offline_v2_storage_foreign_purge_completion_peek(
            &purge_user_id, &purge_batch_token, &purge_total_sectors,
            &purge_failed))
    {
        sent = zy100_ble_ctrl_service_notify_async_result(
            ZY100_BLE_CMD_OFFLINE_FOREIGN_PURGE,
            (uint8_t)(purge_batch_token & 0xFFU),
            purge_failed ? ZY100_BLE_ACK_STATUS_INTERNAL_ERROR :
                           ZY100_BLE_ACK_STATUS_OK,
            purge_failed ? ZY100_BLE_DEVICE_STATE_ERROR :
                           ZY100_BLE_DEVICE_STATE_OFFLINE_SESSION_READY,
            ZY100_BLE_EXEC_MODE_ASYNC_DONE,
            purge_user_id,
            0U,
            purge_failed ? ZY100_BLE_OFFLINE_DETAIL_FLASH_IO :
                offline_sync_foreign_purge_detail(
                    purge_total_sectors, purge_total_sectors));
        if (sent)
        {
            zy100_offline_v2_storage_foreign_purge_completion_ack(
                purge_batch_token);
        }
    }

    if (s_sync.ack_progress_pending)
    {
        s_sync.last_ack_ms = now32;
        s_sync.ack_progress_pending = false;
    }

    if (s_sync.state == OFFLINE_SYNC_CONFIRMING)
    {
        if (!zy100_offline_v2_storage_job_busy())
        {
            app_ble_export_ui_cancel("offline_v2_confirmed");
            offline_sync_enter_wait_ci_balanced(
                "offline_v2_reclaim_restore_balanced");
        }
        if (!offline_sync_reclaim_status_pending())
        {
            return;
        }
    }
    if (s_sync.state == OFFLINE_SYNC_WAIT_CI_BALANCED)
    {
        if (app_ble_ci_state_target_reached(
                APP_BLE_CI_STATE_ACTIVE_IDLE) &&
            !offline_sync_reclaim_status_pending())
        {
            ZY100_LOG_DETAIL("[OFF_CI] phase=balanced_applied sid=%lu",
                       (unsigned long)s_sync.session.session_id);
            s_sync.state = OFFLINE_SYNC_IDLE;
            s_sync.conn_id = OFFLINE_SYNC_CONN_INVALID;
        }
        if (!offline_sync_reclaim_status_pending())
        {
            return;
        }
    }
    if (s_sync.clear_cancel_requested && !s_sync.notify_in_flight)
    {
        offline_sync_finish_clear_cancel();
        return;
    }
    if (s_sync.notify_in_flight)
    {
        if ((uint32_t)(now32 - s_sync.in_flight_since_ms) >=
            OFFLINE_SYNC_ACK_TIMEOUT_MS)
        {
            uint8_t failed_type = s_sync.in_flight_type;
            offline_sync_control_frame_failed(
                failed_type,
                offline_sync_type_is_list(failed_type) ?
                    "offline_v2_list_complete_timeout" :
                    "offline_v2_notify_complete_timeout");
        }
        return;
    }
    if (s_sync.queued_len != 0U)
    {
        if (s_sync.queued_since_ms == 0U)
        {
            s_sync.queued_since_ms = now32;
        }
        if (offline_sync_transport_ready(s_sync.conn_id))
        {
            sent = zy100_ble_ctrl_service_send_export_notify(
                s_sync.conn_id,
                s_sync.frame, s_sync.queued_len);
            if (sent)
            {
                s_sync.notify_in_flight = true;
                s_sync.in_flight_type = s_sync.queued_type;
                s_sync.in_flight_since_ms = now32;
                s_sync.queued_len = 0U;
                s_sync.queued_type = 0U;
                s_sync.queued_since_ms = 0U;
                return;
            }
        }
        if ((uint32_t)(now32 - s_sync.queued_since_ms) >=
            OFFLINE_SYNC_ACK_TIMEOUT_MS)
        {
            uint8_t failed_type = s_sync.queued_type;
            offline_sync_control_frame_failed(
                failed_type,
                offline_sync_type_is_list(failed_type) ?
                    "offline_v2_list_submit_timeout" :
                    "offline_v2_notify_submit_timeout");
        }
        return;
    }
    if ((s_sync.state == OFFLINE_SYNC_CONFIRMING) ||
        (s_sync.state == OFFLINE_SYNC_WAIT_CI_BALANCED))
    {
        return;
    }
    if (s_sync.state == OFFLINE_SYNC_VERIFY_RESUME)
    {
        if (s_sync.verify_offset < s_sync.verify_target)
        {
            uint32_t remaining = s_sync.verify_target -
                                 s_sync.verify_offset;
            uint16_t length = (remaining > sizeof(s_sync.page)) ?
                              sizeof(s_sync.page) : (uint16_t)remaining;

            if (!offline_sync_read_logical(s_sync.verify_offset,
                                           s_sync.page, length))
            {
                (void)offline_sync_queue_abort(
                    OFFLINE_SYNC_ABORT_CONTRACT,
                    ZY100_BLE_OFFLINE_DETAIL_FLASH_IO);
                offline_sync_enter_wait_resume(
                    "offline_v2_resume_read_failed");
                return;
            }
            s_sync.verify_crc_state = zy100_crc32_ieee_update(
                s_sync.verify_crc_state, s_sync.page, length);
            s_sync.verify_offset += length;
            return;
        }
        if (zy100_crc32_ieee_finish(s_sync.verify_crc_state) !=
            s_sync.verify_expected_crc)
        {
            (void)offline_sync_queue_abort(
                OFFLINE_SYNC_ABORT_CONTRACT,
                ZY100_BLE_OFFLINE_DETAIL_CRC);
            offline_sync_enter_wait_resume(
                "offline_v2_resume_crc_failed");
            return;
        }
        s_sync.session.restart_from_zero = false;
        zy100_offline_v2_storage_require_restart(s_sync.session.session_id, s_sync.session.generation, false);
        s_sync.acked_offset = s_sync.verify_target;
        s_sync.sent_offset = s_sync.verify_target;
        s_sync.acked_crc_state = s_sync.verify_crc_state;
        s_sync.sent_crc_state = s_sync.verify_crc_state;
        s_sync.acked_prefix_crc = s_sync.verify_expected_crc;
        s_sync.retransmitting = (s_sync.verify_target != 0U);
        s_sync.host_resume_verified = true;
        s_sync.chunk_count = 0U;
        s_sync.ack_progress_pending = true;
        s_sync.state = OFFLINE_SYNC_WAIT_CI_HIGH;
        return;
    }
    if (s_sync.state == OFFLINE_SYNC_WAIT_CI_HIGH)
    {
        if (app_ble_ci_state_target_reached(
                APP_BLE_CI_STATE_CAPTURE_RUNTIME))
        {
            if (!offline_sync_queue_begin())
            {
                offline_sync_enter_wait_resume(
                    "offline_v2_begin_queue_failed");
                return;
            }
            if (s_sync.host_resume_verified)
            {
                s_sync.state = OFFLINE_SYNC_STREAMING;
                DBG_DIRECT("[OFF_BEGIN] phase=resume_verified sid=%lu tid=%lu offset=%lu",
                           (unsigned long)s_sync.session.session_id,
                           (unsigned long)s_sync.transfer_id,
                           (unsigned long)s_sync.sent_offset);
            }
            else
            {
                s_sync.state = OFFLINE_SYNC_WAIT_HOST_RESUME;
                DBG_DIRECT("[OFF_BEGIN] phase=wait_host_resume sid=%lu tid=%lu",
                           (unsigned long)s_sync.session.session_id,
                           (unsigned long)s_sync.transfer_id);
            }
            ZY100_LOG_DETAIL("[OFF_CI] phase=high_applied sid=%lu tid=%lu",
                       (unsigned long)s_sync.session.session_id,
                       (unsigned long)s_sync.transfer_id);
        }
        return;
    }
    if (s_sync.state == OFFLINE_SYNC_WAIT_HOST_RESUME)
    {
        return;
    }
    if (s_sync.state == OFFLINE_SYNC_WAIT_CHUNK_ACK)
    {
        if ((uint32_t)(now32 - s_sync.last_ack_ms) >=
            OFFLINE_SYNC_ACK_TIMEOUT_MS)
        {
            if (offline_sync_queue_abort(OFFLINE_SYNC_ABORT_ACK_TIMEOUT,
                                          s_sync.sent_offset))
            {
                offline_sync_enter_wait_resume(
                    "offline_v2_ack_timeout");
            }
        }
        return;
    }
    if (s_sync.state != OFFLINE_SYNC_STREAMING) return;
    if (s_sync.sent_offset < s_sync.session.logical_bytes)
    {
        if (!offline_sync_queue_chunk())
        {
            offline_sync_enter_wait_resume(
                "offline_v2_chunk_queue_failed");
        }
        return;
    }
    if (s_sync.acked_offset == s_sync.session.logical_bytes)
    {
        if (offline_sync_queue_end())
        {
            s_sync.state = OFFLINE_SYNC_WAIT_FINAL_CONFIRM;
        }
    }
}

void app_ble_offline_v2_sync_shutdown(void)
{
    if ((s_sync.conn_id != OFFLINE_SYNC_CONN_INVALID) ||
        (s_sync.state != OFFLINE_SYNC_IDLE))
    {
        app_ble_offline_v2_sync_on_disconnect(s_sync.conn_id);
    }
}

void app_ble_offline_v2_sync_on_disconnect(uint8_t conn_id)
{
    offline_sync_state_t old_state;

    offline_sync_arb_reset_connection("disconnect");
    if (conn_id != s_sync.conn_id) return;
    old_state = s_sync.state;
    s_sync.conn_id = OFFLINE_SYNC_CONN_INVALID;
    offline_sync_reset_control_frame();
    if (s_sync.clear_cancel_requested)
    {
        offline_sync_finish_clear_cancel();
        return;
    }
    s_sync.ack_progress_pending = false;
    s_sync.retransmitting = false;
    s_sync.state = OFFLINE_SYNC_IDLE;
    app_ble_export_ui_cancel("offline_v2_disconnect");
    DBG_DIRECT("[OFF_DISC] old=%u new=0 job=%u pending=%lu",
               (uint32_t)old_state,
               zy100_offline_v2_storage_job_busy() ? 1U : 0U,
               (unsigned long)zy100_offline_v2_storage_session_count());
}

void app_ble_offline_v2_sync_on_send_complete(uint8_t conn_id,
                                              uint16_t cause)
{
    if ((conn_id == s_sync.conn_id) && s_sync.notify_in_flight)
    {
        uint8_t completed_type = s_sync.in_flight_type;
        s_sync.notify_in_flight = false;
        s_sync.in_flight_type = 0U;
        s_sync.in_flight_since_ms = 0U;
        if (s_sync.preempt_requested) return;
        if (s_sync.clear_cancel_requested)
        {
            offline_sync_finish_clear_cancel();
            return;
        }
        if (cause != 0U)
        {
            if (offline_sync_type_is_list(completed_type)) s_sync.list_walk_valid = false;
            offline_sync_control_frame_failed(
                completed_type,
                offline_sync_type_is_list(completed_type) ?
                    "offline_v2_list_notify_failed" :
                    "offline_v2_notify_failed");
        }
        else if (offline_sync_type_is_list(completed_type) &&
                 (s_sync.state == OFFLINE_SYNC_IDLE))
        {
            if (s_sync.list_walk_valid && s_sync.list_revision == zy100_offline_v2_storage_directory_revision())
            {
                s_sync.list_expected_cursor = s_sync.list_pending_cursor;
                if (s_sync.list_pending_cursor == 0xFFFFFFFFUL)
                {
                    s_sync.local_preempt_admitted = offline_sync_owned_session_count() != 0U;
                    s_sync.admitted_conn = conn_id;
                }
            }
            s_sync.conn_id = OFFLINE_SYNC_CONN_INVALID;
        }
    }
}

bool app_ble_offline_v2_sync_active(void)
{
    return (s_sync.state != OFFLINE_SYNC_IDLE) ||
           (s_sync.queued_len != 0U) || s_sync.notify_in_flight;
}

bool app_ble_offline_v2_sync_transfer_active(void)
{
    return !s_sync.preempt_requested && offline_sync_payload_state();
}

bool app_ble_offline_v2_sync_blocks_capture(void)
{
    return app_ble_offline_v2_sync_active() ||
           zy100_offline_v2_storage_foreign_purge_active();
}

bool app_ble_offline_v2_sync_blocks_online(void)
{
    return !s_offline_arb_business_bootstrap_complete ||
           (offline_sync_owned_session_count() != 0U) ||
           app_ble_offline_v2_sync_active() ||
           zy100_offline_v2_storage_job_busy();
}

bool app_ble_offline_v2_sync_blocks_new_business(void)
{
    return app_ble_power_is_connected() &&
           app_ble_offline_v2_sync_blocks_online();
}

app_ble_offline_v2_arb_state_t app_ble_offline_v2_sync_arb_state(void)
{
    return offline_sync_arb_compute();
}

uint32_t app_ble_offline_v2_sync_pending_count(void)
{
    return offline_sync_owned_session_count();
}

bool app_ble_offline_v2_sync_transfer_admitted(void)
{
    return s_offline_arb_transfer_admitted;
}

bool app_ble_offline_v2_sync_clear_admitted(void)
{
    uint32_t user_id = offline_sync_current_user_id();

    return app_ble_power_is_connected() &&
           (user_id != 0U) &&
           s_offline_arb_business_bootstrap_complete &&
           s_offline_arb_transfer_admitted &&
           (zy100_offline_v2_storage_session_count_for_user(user_id) == 0U) &&
           !zy100_offline_v2_capture_active() &&
           !zy100_offline_v2_storage_session_active() &&
           !app_ble_offline_v2_sync_active() &&
           !zy100_offline_v2_storage_job_busy();
}

bool app_ble_offline_v2_sync_foreign_purge_admitted(void)
{
    uint32_t user_id = offline_sync_current_user_id();

    return app_ble_power_is_connected() &&
           (user_id != 0U) &&
           s_offline_arb_transfer_admitted &&
           (zy100_offline_v2_storage_session_count_for_user(user_id) == 0U) &&
           !zy100_offline_v2_capture_active() &&
           !zy100_offline_v2_storage_session_active() &&
           !app_ble_offline_v2_sync_active() &&
           !zy100_offline_v2_storage_job_busy();
}

bool app_ble_offline_v2_sync_business_bootstrap_complete(void)
{
    return s_offline_arb_business_bootstrap_complete;
}

void app_ble_offline_v2_sync_begin_capture_epoch(const char *reason)
{
    bool changed = s_offline_arb_transfer_admitted ||
                   s_offline_arb_business_bootstrap_complete;

    s_offline_arb_transfer_admitted = false;
    s_offline_arb_business_bootstrap_complete = false;
    if (changed)
    {
        offline_sync_arb_log(reason, true);
    }
}

void app_ble_offline_v2_sync_note_transfer_admitted(const char *reason)
{
    if (s_offline_arb_transfer_admitted)
    {
        return;
    }
    s_offline_arb_transfer_admitted = true;
    offline_sync_arb_log(reason, true);
}

void app_ble_offline_v2_sync_note_business_bootstrap_complete(
    const char *reason)
{
    if (s_offline_arb_transfer_admitted &&
        s_offline_arb_business_bootstrap_complete)
    {
        return;
    }
    s_offline_arb_transfer_admitted = true;
    s_offline_arb_business_bootstrap_complete = true;
    offline_sync_arb_log(reason, true);
}

bool app_ble_offline_v2_sync_cancel_for_clear(void)
{
    if ((s_sync.state == OFFLINE_SYNC_CONFIRMING) ||
        zy100_offline_v2_storage_job_busy())
    {
        return false;
    }
    s_sync.clear_cancel_requested = true;
    s_sync.queued_len = 0U;
    s_sync.queued_type = 0U;
    s_sync.queued_since_ms = 0U;
    app_ble_export_ui_cancel("offline_v2_clear_requested");
    if (!s_sync.notify_in_flight)
    {
        offline_sync_finish_clear_cancel();
    }
    return true;
}

bool app_ble_offline_v2_sync_quiesced_for_clear(void)
{
    return s_sync.clear_cancel_requested &&
           !s_sync.notify_in_flight &&
           (s_sync.queued_len == 0U) &&
           (s_sync.state == OFFLINE_SYNC_IDLE);
}

void app_ble_offline_v2_sync_reset_after_clear(void)
{
    offline_sync_reset_control_frame();
    memset(&s_sync.session, 0, sizeof(s_sync.session));
    s_sync.state = OFFLINE_SYNC_IDLE;
    s_sync.conn_id = OFFLINE_SYNC_CONN_INVALID;
    s_sync.transfer_id = 0U;
    s_sync.sent_offset = 0U;
    s_sync.acked_offset = 0U;
    s_sync.clear_cancel_requested = false;
    app_ble_export_ui_cancel("offline_v2_clear_complete");
}

bool app_ble_offline_v2_sync_local_preempt_admitted(void)
{
    return s_sync.local_preempt_admitted && !s_sync.preempt_requested &&
        offline_sync_transport_ready(s_sync.admitted_conn) &&
        app_ble_connection_bootstrap_user_sync_ready(s_sync.admitted_conn) &&
        offline_sync_owned_session_count() != 0U &&
        !zy100_offline_v2_storage_foreign_purge_active() && !s_sync.clear_cancel_requested;
}

bool app_ble_offline_v2_sync_preempt_begin(void)
{
    if (!app_ble_offline_v2_sync_local_preempt_admitted() ||
        !zy100_offline_v2_storage_pause_reclaim()) return false;
    s_sync.preempt_requested = true;
    s_sync.preempt_abort_pending = s_sync.transfer_id != 0U && s_sync.state != OFFLINE_SYNC_IDLE;
    if (s_sync.preempt_abort_pending)
        zy100_offline_v2_storage_require_restart(s_sync.session.session_id, s_sync.session.generation, true);
    /* Do not touch frame bytes until the outstanding callback has returned. */
    s_sync.queued_len = 0U;
    s_sync.queued_type = 0U;
    s_sync.ack_progress_pending = false;
    s_sync.host_resume_verified = false;
    s_sync.state = OFFLINE_SYNC_IDLE;
    s_sync.list_walk_valid = false;
    app_ble_export_ui_cancel("offline_local_preempt");
    return true;
}

bool app_ble_offline_v2_sync_preempt_quiesced(void)
{
    return s_sync.preempt_requested && !s_sync.notify_in_flight && !s_sync.preempt_abort_pending &&
        !zy100_offline_v2_storage_job_busy();
}

void app_ble_offline_v2_sync_preempt_end(void)
{
    if (!s_sync.preempt_requested || s_sync.notify_in_flight || s_sync.preempt_abort_pending) return;
    s_sync.preempt_requested = false;
    s_sync.transfer_id = 0U;
    s_sync.conn_id = OFFLINE_SYNC_CONN_INVALID;
    zy100_offline_v2_storage_release_reclaim();
}
