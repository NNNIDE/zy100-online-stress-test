#include <stddef.h>
#include <string.h>

#include <gap.h>
#include <os_sched.h>
#include <trace.h>

#include "app_flags.h"
#include "app_task.h"
#include "peripheral_app.h"
#include "zy100_clock_config.h"
#include "app/app_ble_conn_policy.h"
#include "app/app_ble_export_controller.h"
#include "app/app_ble_offline_v2_sync.h"
#include "app/app_ble_export_perf.h"
#include "app/app_ble_export_port.h"
#include "app/app_training_context.h"
#include "common/zy100_byteorder.h"
#include "service/imu_fifo_drain_test.h"
#include "service/svc_led_owner.h"
#include "service/svc_led_pattern.h"
#include "service/zy100_ble_ctrl_protocol.h"
#include "service/zy100_ble_ctrl_service.h"
#include "service/zy100_crc32.h"
#include "service/zy100_feuf_export_producer.h"
#include "service/zy100_session_range_table.h"

#if (defined(ZY100_BUILD_FACTORY) && ZY100_BUILD_FACTORY) || \
    !ZY100_LEGACY_OFFLINE_ENABLE

static bool s_v2_upload_led_active;

void app_ble_export_try_auto_start(const char *reason)
{
    /* Offline V2 is host-command initiated.  Keeping this compatibility
     * hook side-effect free prevents an unsolicited LIST from racing the
     * Host-CI/calibration connection gate. */
    (void)reason;
}
void app_ble_export_poll(void) {}
void app_ble_export_on_disconnect(uint8_t conn_id) { (void)conn_id; }
bool app_ble_export_is_active(void) { return false; }
bool app_ble_export_blocks_sleep(void) { return false; }
bool app_ble_export_blocks_capture(void) { return false; }
bool app_ble_export_blocks_clear(void) { return false; }
bool app_ble_export_transport_busy(void) { return false; }
bool app_ble_export_waiting_confirm(void) { return false; }
bool app_ble_export_reclaim_active(void) { return false; }
bool app_ble_export_clear_retry_required(void) { return false; }
uint32_t app_ble_export_current_or_retry_id(void) { return 0U; }
void app_ble_export_clear_state_reset_after_finalize(void) {}
void app_ble_export_abort_keep_flash(const char *reason) { (void)reason; }
bool app_ble_export_ui_result_active(void) { return false; }
bool app_ble_export_ui_blocks_normal_flow(void) { return false; }
bool app_ble_export_ui_failed(void) { return false; }
bool app_ble_export_ui_active(void) { return s_v2_upload_led_active; }
bool app_ble_export_ui_button_override_active(void) { return false; }
void app_ble_export_ui_tick(uint64_t runtime_ms)
{
    if (s_v2_upload_led_active) led_tick(runtime_ms);
}
void app_ble_export_ui_cancel(const char *reason)
{
    (void)reason;
    s_v2_upload_led_active = false;
    led_release(LED_OWNER_BLE_EXPORT);
}
void app_ble_export_ui_begin_upload(const char *reason)
{
    led_pattern_t pattern;
    (void)reason;
    app_ble_export_port_ui_release_training_end("offline_v2_upload");
    app_ble_export_port_ui_cancel_charge("offline_v2_upload");
    app_ble_export_port_ui_stop_export_wait();
    app_ble_export_port_ui_stop_clear_red();
    led_release(LED_OWNER_FLASH_FULL_WAIT_UPLOAD);
    led_pattern_notify_blink(&pattern, LED_PATTERN_COLOR_GREEN,
                             200U, 200U, 0U, true);
    s_v2_upload_led_active = led_request(LED_OWNER_BLE_EXPORT,
                                         LED_PRIORITY_BLE_EXPORT,
                                         &pattern);
}
void app_ble_export_ui_show_blue(uint8_t owner,
                                 uint8_t priority,
                                 const char *reason)
{
    (void)owner;
    (void)priority;
    (void)reason;
}
void app_ble_export_controller_on_send_data_complete(uint8_t conn_id,
                                                     T_SERVER_ID service_id,
                                                     uint16_t attrib_idx,
                                                     uint16_t cause,
                                                     uint16_t credits)
{
    (void)conn_id;
    (void)service_id;
    (void)attrib_idx;
    (void)cause;
    (void)credits;
}
void app_ble_export_controller_tx_ready(void) {}
bool app_ble_export_controller_prepare_for_sleep_request(void) { return true; }
void app_ble_export_controller_shutdown_poll(void) {}
uint8_t app_ble_export_controller_request_confirm(uint8_t seq,
                                                  uint32_t export_id,
                                                  uint64_t host_time_ms,
                                                  uint32_t result)
{
    (void)seq;
    (void)export_id;
    (void)host_time_ms;
    (void)result;
    return 0U;
}
uint8_t app_ble_export_perf_port_conn_id(void) { return 0U; }
bool app_ble_export_perf_port_streaming(void) { return false; }
const zy100_feuf_export_producer_t *app_ble_export_perf_port_producer(void)
{
    return 0;
}

#else

#if ZY100_LED_LOG_ENABLE
#define APP_LED_LOG(...) DBG_DIRECT(__VA_ARGS__)
#else
#define APP_LED_LOG(...) do { if (0) { DBG_DIRECT(__VA_ARGS__); } } while (0)
#endif

#define APP_BLE_UPLOAD_LED_FAST_ON_MS    200ULL
#define APP_BLE_UPLOAD_LED_FAST_OFF_MS   200ULL
#define APP_BLE_UPLOAD_SUCCESS_HOLD_MS   2000ULL
#define APP_BLE_UPLOAD_FAIL_BLINK_CYCLES 3U

typedef enum
{
    APP_BLE_UPLOAD_LED_IDLE = 0U,
    APP_BLE_UPLOAD_LED_UPLOADING,
    APP_BLE_UPLOAD_LED_SUCCESS_HOLD,
    APP_BLE_UPLOAD_LED_FAIL_BLINK,
} app_ble_upload_led_state_t;

static app_ble_upload_led_state_t s_ble_upload_led_state =
    APP_BLE_UPLOAD_LED_IDLE;
static bool s_ble_export_clear_retry_required = false;
static uint32_t s_ble_export_clear_failed_export_id = 0U;
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
static bool s_ble_export_resume_reclaim_active = false;
static uint32_t s_ble_export_resume_reclaim_uid = 0U;
#define s_ble_export_storage_state g_app_ble_export_port_storage_state
#endif
typedef enum
{
    APP_BLE_EXPORT_IDLE = 0U,
    APP_BLE_EXPORT_WAIT_SUBSCRIBE,
    APP_BLE_EXPORT_PREPARE,
    APP_BLE_EXPORT_START_FRAME,
    APP_BLE_EXPORT_STREAMING,
    APP_BLE_EXPORT_END_FRAME,
    APP_BLE_EXPORT_WAIT_CONFIRM,
    APP_BLE_EXPORT_CLEAR_AFTER_CONFIRM,
    APP_BLE_EXPORT_DONE,
    APP_BLE_EXPORT_FAILED,
} app_ble_export_state_t;

#define APP_BLE_EXPORT_MAGIC               0xE7U
#define APP_BLE_EXPORT_VERSION             0x01U
#define APP_BLE_EXPORT_FLAG_LAST_FRAME     0x01U
#define APP_BLE_EXPORT_HEADER_BYTES        8U
#define APP_BLE_EXPORT_DEFAULT_ATT_MTU     23U
#define APP_BLE_EXPORT_CTRL_PAYLOAD_BYTES  16U
#define APP_BLE_EXPORT_PROGRESS_BYTES      (16UL * 1024UL)
#define APP_BLE_EXPORT_PROGRESS_MS         1000ULL
#define APP_BLE_EXPORT_SECTION_INVALID     0xFFFFFFFFUL

typedef char app_ble_export_feuf_notify_fit_check[
    ((APP_BLE_EXPORT_HEADER_BYTES + sizeof(zy100_feuf_frame_header_t) +
      ZY100_BLE_EXPORT_FEUF_PAYLOAD_MAX) <= APP_BLE_EXPORT_MAX_NOTIFY_BYTES) ? 1 : -1];

typedef enum
{
    APP_BLE_EXPORT_SEND_OK = 0U,
    APP_BLE_EXPORT_SEND_LOCAL_BLOCKED,
    APP_BLE_EXPORT_SEND_BLOCKED,
    APP_BLE_EXPORT_SEND_FATAL,
} app_ble_export_send_result_t;

typedef enum
{
    APP_BLE_EXPORT_CTRL_DONE = 0U,
    APP_BLE_EXPORT_CTRL_BLOCKED,
    APP_BLE_EXPORT_CTRL_FATAL,
} app_ble_export_ctrl_result_t;

#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE

typedef struct
{
    bool used;
    uint16_t len;
    uint16_t payload_len;
    uint16_t chunk_seq;
    uint8_t frame_type;
    uint8_t frame[APP_BLE_EXPORT_MAX_NOTIFY_BYTES];
} app_ble_export_tx_slot_t;

typedef struct
{
    uint16_t len;
    uint16_t payload_len;
    uint16_t chunk_seq;
    uint8_t frame_type;
} app_ble_export_tx_complete_info_t;
#endif

typedef struct
{
    app_ble_export_state_t state;
    uint8_t conn_id;
    uint32_t export_id;
    uint16_t notify_max;
    uint16_t chunk_payload;
    uint16_t chunk_seq;
    uint32_t chunk_count;
    uint32_t total_bytes;
    uint32_t crc_state;
    uint32_t stream_crc32;
    uint32_t current_export_id;
    uint32_t current_export_session_uid;
    uint32_t current_export_index;
    uint32_t current_export_total;
    uint32_t current_end_seq;
    uint32_t current_session_crc;
    uint32_t batch_exported_count;
    bool end_sent;
    bool clear_begin_requested;
    bool mtu_logged;
    bool progress_logged;
    uint32_t progress_section;
    uint32_t progress_last_section;
    uint32_t progress_last_sent;
    uint64_t progress_last_ms;
    uint8_t ctrl_type;
    uint16_t ctrl_seq;
    uint16_t ctrl_len;
    uint16_t ctrl_offset;
    uint8_t ctrl_payload[APP_BLE_EXPORT_CTRL_PAYLOAD_BYTES];
    uint8_t tx_frame[APP_BLE_EXPORT_MAX_NOTIFY_BYTES];
    uint8_t pending_payload[APP_BLE_EXPORT_MAX_NOTIFY_BYTES - APP_BLE_EXPORT_HEADER_BYTES];
    uint8_t prefetch_cache[ZY100_BLE_EXPORT_PREFETCH_BYTES];
    zy100_feuf_export_producer_t producer;
    zy100_feuf_export_aggregate_plan_t agg_plan;
#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
    app_ble_export_tx_slot_t tx_ring[ZY100_BLE_EXPORT_TX_RING_DEPTH];
    uint8_t tx_head;
    uint8_t tx_tail;
    uint8_t tx_in_flight;
    uint16_t tx_budget;
    bool tx_ready_event_pending;
    bool tx_probe_used;
    bool tx_fatal_complete;
    uint16_t tx_fatal_complete_cause;
    uint32_t tx_backoff_until_ms;
    uint32_t send_blocked_count;
    uint32_t send_blocked_consecutive;
    uint32_t send_blocked_first_ms;
#endif
} app_ble_export_ctx_t;

typedef struct
{
    bool valid;
    bool notify_failed_logged;
    uint8_t seq;
    uint8_t conn_id;
    uint32_t export_id;
    uint32_t user_id;
    uint32_t training_id;
    uint64_t host_time_ms;
} app_ble_export_confirm_ctx_t;

static app_ble_export_ctx_t s_ble_export;
static app_ble_export_confirm_ctx_t s_ble_export_confirm_ctx;

uint8_t app_ble_export_perf_port_conn_id(void)
{
    return s_ble_export.conn_id;
}

bool app_ble_export_perf_port_streaming(void)
{
    return s_ble_export.state == APP_BLE_EXPORT_STREAMING;
}

const zy100_feuf_export_producer_t *app_ble_export_perf_port_producer(void)
{
    return &s_ble_export.producer;
}
#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
static bool app_ble_export_tx_reset(void);
static uint8_t app_ble_export_tx_free_slots(void);
static bool app_ble_export_tx_can_submit_reason(uint8_t *reason);
static bool app_ble_export_tx_can_submit(void);
static app_ble_export_tx_slot_t *app_ble_export_tx_alloc_slot(void);
static void app_ble_export_tx_commit_slot(uint16_t len,
                                          uint8_t frame_type,
                                          uint16_t chunk_seq,
                                          uint16_t payload_len);
static void app_ble_export_tx_cancel_slot(void);
static bool app_ble_export_tx_complete_one(
    app_ble_export_tx_complete_info_t *info);
static void app_ble_export_tx_clear_backpressure(void);
static void app_ble_export_post_tx_ready_event(void);
static void app_ble_export_fast_pump(void) __attribute__((unused));
#endif
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
#define APP_BLE_EXPORT_P0_POLL_RETURN()                         \
    do                                                          \
    {                                                           \
        app_ble_export_p0_poll_exit(p0_poll_tracked,            \
                                    p0_notify_ok_before);       \
        return;                                                 \
    } while (0)
#else
#define APP_BLE_EXPORT_P0_POLL_RETURN()                         \
    do                                                          \
    {                                                           \
        return;                                                 \
    } while (0)
#endif

void app_ble_export_controller_on_send_data_complete(uint8_t conn_id,
                                                     T_SERVER_ID service_id,
                                                     uint16_t attrib_idx,
                                                     uint16_t cause,
                                                     uint16_t credits)
{
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    app_ble_export_p0_on_send_data_complete(conn_id,
                                            service_id,
                                            attrib_idx,
                                            cause,
                                            credits);
#endif

#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
    app_ble_export_tx_complete_info_t complete_info;

    if (conn_id != s_ble_export.conn_id)
    {
        return;
    }

    if (s_ble_export.tx_in_flight == 0U)
    {
        return;
    }

    memset(&complete_info, 0, sizeof(complete_info));
    if (!app_ble_export_tx_complete_one(&complete_info))
    {
        return;
    }

    if (cause == GAP_SUCCESS)
    {
        uint16_t ring_free = (uint16_t)app_ble_export_tx_free_slots();
        uint16_t free_window = 0U;
        uint16_t new_budget;

        if (s_ble_export.tx_in_flight < ZY100_BLE_EXPORT_TX_WINDOW)
        {
            free_window = (uint16_t)(ZY100_BLE_EXPORT_TX_WINDOW -
                                     s_ble_export.tx_in_flight);
        }
        if (ring_free < free_window)
        {
            free_window = ring_free;
        }
        new_budget = free_window;
        if (credits < new_budget)
        {
            new_budget = credits;
        }
        if (new_budget > s_ble_export.tx_budget)
        {
            s_ble_export.tx_budget = new_budget;
        }
        app_ble_export_tx_clear_backpressure();
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        if (complete_info.frame_type == APP_BLE_EXPORT_FRAME_DATA)
        {
            app_ble_export_p0_note_data_complete_ok(
                app_ble_export_p0_now_os_ms());
        }
        app_ble_export_p0_note_tx_budget(s_ble_export.tx_budget);
#endif
        app_ble_export_post_tx_ready_event();
    }
    else
    {
        s_ble_export.tx_budget = 0U;
        s_ble_export.tx_fatal_complete = true;
        s_ble_export.tx_fatal_complete_cause = cause;
        app_ble_export_post_tx_ready_event();
    }
#else
#if !ZY100_BLE_EXPORT_P0_PERF_ENABLE
    (void)conn_id;
    (void)cause;
    (void)credits;
#endif
#endif

}

static void app_ble_export_fail_keep_flash(uint32_t detail);
static void app_ble_export_clear_failed(uint32_t detail);

#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
static void app_ble_export_refresh_storage(const char *reason)
{
    app_ble_export_port_refresh_storage(reason);
}
#endif

static bool app_ble_export_state_streaming(app_ble_export_state_t state)
{
    return (state == APP_BLE_EXPORT_PREPARE) ||
           (state == APP_BLE_EXPORT_START_FRAME) ||
           (state == APP_BLE_EXPORT_STREAMING) ||
           (state == APP_BLE_EXPORT_END_FRAME);
}

#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
static bool app_ble_export_tx_reset(void)
{
    if (s_ble_export.tx_in_flight != 0U)
    {
        return false;
    }

    memset(s_ble_export.tx_ring, 0, sizeof(s_ble_export.tx_ring));
    s_ble_export.tx_head = 0U;
    s_ble_export.tx_tail = 0U;
    s_ble_export.tx_in_flight = 0U;
    s_ble_export.tx_budget = ZY100_BLE_EXPORT_INITIAL_CREDITS;
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    app_ble_export_p0_note_tx_budget(s_ble_export.tx_budget);
#endif
    s_ble_export.tx_ready_event_pending = false;
    s_ble_export.tx_probe_used = false;
    s_ble_export.tx_backoff_until_ms = 0U;
    s_ble_export.send_blocked_count = 0U;
    s_ble_export.send_blocked_consecutive = 0U;
    s_ble_export.send_blocked_first_ms = 0U;
    s_ble_export.tx_fatal_complete = false;
    s_ble_export.tx_fatal_complete_cause = 0U;
    return true;
}

static uint8_t app_ble_export_tx_free_slots(void)
{
    if (s_ble_export.tx_in_flight >= ZY100_BLE_EXPORT_TX_RING_DEPTH)
    {
        return 0U;
    }
    return (uint8_t)(ZY100_BLE_EXPORT_TX_RING_DEPTH -
                     s_ble_export.tx_in_flight);
}

static bool app_ble_export_tx_deadline_due(uint32_t now_ms,
                                           uint32_t due_ms)
{
    return (due_ms == 0U) || (((int32_t)(now_ms - due_ms)) >= 0);
}

static bool app_ble_export_tx_backoff_active(uint32_t now_ms)
{
    return (s_ble_export.tx_backoff_until_ms != 0U) &&
           !app_ble_export_tx_deadline_due(now_ms,
                                           s_ble_export.tx_backoff_until_ms);
}

static void app_ble_export_tx_try_probe_budget(uint32_t now_ms)
{
    if ((s_ble_export.tx_budget != 0U) ||
        (s_ble_export.tx_in_flight != 0U) ||
        s_ble_export.tx_probe_used ||
        app_ble_export_tx_backoff_active(now_ms))
    {
        return;
    }
    s_ble_export.tx_probe_used = true;
    s_ble_export.tx_budget = 1U;
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    app_ble_export_p0_note_tx_budget(s_ble_export.tx_budget);
#endif
}

static void app_ble_export_tx_set_backoff(uint32_t now_ms)
{
    s_ble_export.tx_budget = 0U;
    s_ble_export.tx_backoff_until_ms =
        now_ms + (uint32_t)ZY100_BLE_EXPORT_BACKOFF_MS;
}

static void app_ble_export_tx_clear_backpressure(void)
{
    s_ble_export.tx_probe_used = false;
    s_ble_export.tx_backoff_until_ms = 0U;
    s_ble_export.send_blocked_consecutive = 0U;
    s_ble_export.send_blocked_first_ms = 0U;
}

static void app_ble_export_tx_stop_flow_control(void)
{
    s_ble_export.tx_budget = 0U;
    s_ble_export.tx_probe_used = false;
    s_ble_export.tx_backoff_until_ms = 0U;
    s_ble_export.send_blocked_count = 0U;
    s_ble_export.send_blocked_consecutive = 0U;
    s_ble_export.send_blocked_first_ms = 0U;
}

static bool app_ble_export_tx_can_submit_reason(uint8_t *reason)
{
    uint8_t local_reason = APP_BLE_EXPORT_TX_SUBMIT_OK;
    uint32_t now_ms = (uint32_t)os_sys_time_get();

    if (app_ble_export_tx_backoff_active(now_ms))
    {
        local_reason = APP_BLE_EXPORT_TX_SUBMIT_BACKOFF;
    }
    else
    {
        app_ble_export_tx_try_probe_budget(now_ms);
    }

    if (local_reason != APP_BLE_EXPORT_TX_SUBMIT_OK)
    {
        /* Keep the backoff reason selected above. */
    }
    else if (s_ble_export.tx_budget == 0U)
    {
        local_reason = APP_BLE_EXPORT_TX_SUBMIT_NO_BUDGET;
    }
    else if (s_ble_export.tx_in_flight >= ZY100_BLE_EXPORT_TX_WINDOW)
    {
        local_reason = APP_BLE_EXPORT_TX_SUBMIT_WINDOW_FULL;
    }
    else if (app_ble_export_tx_free_slots() == 0U)
    {
        local_reason = APP_BLE_EXPORT_TX_SUBMIT_NO_SLOT;
    }
    else if (s_ble_export.tx_ring[s_ble_export.tx_head].used)
    {
        local_reason = APP_BLE_EXPORT_TX_SUBMIT_HEAD_USED;
    }

    if (reason != NULL)
    {
        *reason = local_reason;
    }
    return local_reason == APP_BLE_EXPORT_TX_SUBMIT_OK;
}

static bool app_ble_export_tx_can_submit(void)
{
    return app_ble_export_tx_can_submit_reason(NULL);
}

static app_ble_export_tx_slot_t *app_ble_export_tx_alloc_slot(void)
{
    app_ble_export_tx_slot_t *slot;
    uint8_t reason = APP_BLE_EXPORT_TX_SUBMIT_OK;

    if (!app_ble_export_tx_can_submit_reason(&reason))
    {
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        app_ble_export_p0_note_tx_block_reason(reason);
#endif
        return NULL;
    }

    slot = &s_ble_export.tx_ring[s_ble_export.tx_head];
    if (slot->used)
    {
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        app_ble_export_p0_note_tx_block_reason(
            APP_BLE_EXPORT_TX_SUBMIT_HEAD_USED);
#endif
        return NULL;
    }
    return slot;
}

static void app_ble_export_tx_commit_slot(uint16_t len,
                                          uint8_t frame_type,
                                          uint16_t chunk_seq,
                                          uint16_t payload_len)
{
    app_ble_export_tx_slot_t *slot =
        &s_ble_export.tx_ring[s_ble_export.tx_head];

    slot->used = true;
    slot->len = len;
    slot->frame_type = frame_type;
    slot->chunk_seq = chunk_seq;
    slot->payload_len = payload_len;

    s_ble_export.tx_head++;
    if (s_ble_export.tx_head >= ZY100_BLE_EXPORT_TX_RING_DEPTH)
    {
        s_ble_export.tx_head = 0U;
    }

    s_ble_export.tx_in_flight++;
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    app_ble_export_p0_note_tx_in_flight(s_ble_export.tx_in_flight);
#endif

    if (s_ble_export.tx_budget != 0U)
    {
        s_ble_export.tx_budget--;
    }
}

static void app_ble_export_tx_cancel_slot(void)
{
    app_ble_export_tx_slot_t *slot =
        &s_ble_export.tx_ring[s_ble_export.tx_head];

    slot->used = false;
    slot->len = 0U;
    slot->payload_len = 0U;
    slot->chunk_seq = 0U;
    slot->frame_type = 0U;
}

static bool app_ble_export_tx_complete_one(
    app_ble_export_tx_complete_info_t *info)
{
    app_ble_export_tx_slot_t *slot;

    if (s_ble_export.tx_in_flight == 0U)
    {
        return false;
    }

    slot = &s_ble_export.tx_ring[s_ble_export.tx_tail];
    if (!slot->used)
    {
        return false;
    }

    if (info != NULL)
    {
        info->len = slot->len;
        info->payload_len = slot->payload_len;
        info->chunk_seq = slot->chunk_seq;
        info->frame_type = slot->frame_type;
    }
    slot->used = false;
    slot->len = 0U;
    slot->payload_len = 0U;
    slot->chunk_seq = 0U;
    slot->frame_type = 0U;

    s_ble_export.tx_tail++;
    if (s_ble_export.tx_tail >= ZY100_BLE_EXPORT_TX_RING_DEPTH)
    {
        s_ble_export.tx_tail = 0U;
    }

    s_ble_export.tx_in_flight--;
    return true;
}

static void app_ble_export_post_tx_ready_event(void)
{
    if (s_ble_export.tx_ready_event_pending)
    {
        return;
    }

    if (app_ble_export_port_post_tx_ready_event())
    {
        s_ble_export.tx_ready_event_pending = true;
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        app_ble_export_p0_note_tx_ready_event();
#endif
    }
}

static bool app_ble_export_fast_state_can_pump(void)
{
    return app_ble_export_state_streaming(s_ble_export.state);
}

static void app_ble_export_fast_pump(void) __attribute__((unused));
static void app_ble_export_fast_pump(void)
{
    uint8_t steps = 0U;
    uint16_t old_seq;
    uint16_t old_ctrl_offset;
    uint32_t old_total;
    app_ble_export_state_t old_state;
    bool old_end_sent;

#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    app_ble_export_p0_note_fast_pump_call();
#endif

    if (!app_ble_export_fast_state_can_pump())
    {
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        app_ble_export_p0_note_fast_pump_steps(steps);
#endif
        return;
    }

    if ((s_ble_export.state == APP_BLE_EXPORT_PREPARE) &&
        (s_ble_export.tx_in_flight != 0U))
    {
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        app_ble_export_p0_note_fast_pump_steps(steps);
#endif
        return;
    }

    while (steps < (ZY100_BLE_EXPORT_TX_WINDOW + 2U))
    {
        if (!app_ble_export_fast_state_can_pump())
        {
            break;
        }

        if ((s_ble_export.state == APP_BLE_EXPORT_PREPARE) &&
            (s_ble_export.tx_in_flight != 0U))
        {
            break;
        }

        if ((s_ble_export.state == APP_BLE_EXPORT_STREAMING) &&
            !app_ble_export_tx_can_submit())
        {
            break;
        }

        old_state = s_ble_export.state;
        old_seq = s_ble_export.chunk_seq;
        old_total = s_ble_export.total_bytes;
        old_ctrl_offset = s_ble_export.ctrl_offset;
        old_end_sent = s_ble_export.end_sent;

        app_ble_export_poll();
        steps++;

        if ((old_state == s_ble_export.state) &&
            (old_seq == s_ble_export.chunk_seq) &&
            (old_total == s_ble_export.total_bytes) &&
            (old_ctrl_offset == s_ble_export.ctrl_offset) &&
            (old_end_sent == s_ble_export.end_sent))
        {
            break;
        }

        if ((s_ble_export.state == APP_BLE_EXPORT_STREAMING) &&
            !app_ble_export_tx_can_submit())
        {
            break;
        }
    }
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    app_ble_export_p0_note_fast_pump_steps(steps);
#endif
}
#endif

bool app_ble_export_is_active(void)
{
    return app_ble_export_state_streaming(s_ble_export.state) ||
           (s_ble_export.state == APP_BLE_EXPORT_WAIT_CONFIRM) ||
           (s_ble_export.state == APP_BLE_EXPORT_CLEAR_AFTER_CONFIRM);
}

bool app_ble_export_blocks_sleep(void)
{
    return app_ble_export_is_active() ||
           app_ble_export_ui_blocks_normal_flow();
}

bool app_ble_export_blocks_capture(void)
{
    return s_ble_export_clear_retry_required ||
           app_ble_export_is_active() ||
           app_ble_export_ui_blocks_normal_flow() ||
           app_ble_export_port_blocks_capture_extra();
}

bool app_ble_export_blocks_clear(void)
{
    if (s_ble_export_clear_retry_required)
    {
        return false;
    }
    return app_ble_export_port_blocks_clear_extra() ||
           app_ble_export_is_active() ||
#if !(ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE)
           (app_ble_power_is_connected() && app_ble_export_port_has_pending_sessions());
#else
           false;
#endif
}

bool app_ble_export_ui_button_override_active(void)
{
#if F_BT_DLPS_EN && F_APP_BUTTON_DLPS_CTRL_ENABLE
    return app_ble_export_port_ui_button_override_active();
#else
    return false;
#endif
}

static void app_ble_export_ui_reset_runtime(void)
{
    s_ble_upload_led_state = APP_BLE_UPLOAD_LED_IDLE;
}

bool app_ble_export_ui_result_active(void)
{
    return (s_ble_upload_led_state == APP_BLE_UPLOAD_LED_SUCCESS_HOLD) ||
           (s_ble_upload_led_state == APP_BLE_UPLOAD_LED_FAIL_BLINK);
}

bool app_ble_export_ui_blocks_normal_flow(void)
{
    return app_ble_export_ui_result_active() &&
           !app_ble_export_ui_button_override_active();
}

static void app_ble_export_ui_complete_and_restore(const char *reason)
{
    app_ble_export_ui_reset_runtime();
    app_ble_export_port_ui_restore_capture(reason);
}

void app_ble_export_ui_tick(uint64_t runtime_ms)
{
    if (s_ble_upload_led_state == APP_BLE_UPLOAD_LED_IDLE)
    {
        return;
    }

    if (app_ble_export_ui_button_override_active())
    {
        led_release(LED_OWNER_BLE_EXPORT);
        app_ble_export_ui_reset_runtime();
        return;
    }

    led_tick(runtime_ms);
    if (app_ble_export_ui_result_active() &&
        !led_owner_is_active(LED_OWNER_BLE_EXPORT))
    {
        if (s_ble_upload_led_state == APP_BLE_UPLOAD_LED_SUCCESS_HOLD)
        {
            APP_LED_LOG("[BLE_LED] upload_success_done restore=1");
            app_ble_export_ui_complete_and_restore("ble_upload_success_done");
        }
        else
        {
            APP_LED_LOG("[BLE_LED] upload_fail_done restore=1");
            app_ble_export_ui_complete_and_restore("ble_upload_fail_done");
        }
        return;
    }
}

void app_ble_export_ui_cancel(const char *reason)
{
    bool button_override = app_ble_export_ui_button_override_active();

    if (s_ble_upload_led_state == APP_BLE_UPLOAD_LED_IDLE)
    {
        return;
    }

    APP_LED_LOG("[BLE_LED] upload_led_cancel reason=%s",
                (reason != NULL) ? reason : "unknown");
    app_ble_export_ui_reset_runtime();
    led_release(LED_OWNER_BLE_EXPORT);
    if (!button_override)
    {
        app_ble_export_port_ui_stop_export_wait();
        app_ble_export_port_ui_stop_clear_red();
    }
}

void app_ble_export_ui_begin_upload(const char *reason)
{
    uint64_t runtime_ms = zy100_os_time_ms();
    led_pattern_t pattern;

    app_ble_export_port_ui_release_training_end(reason);
    if (app_ble_export_ui_button_override_active())
    {
        app_ble_export_ui_cancel(reason);
        return;
    }

    if (s_ble_upload_led_state == APP_BLE_UPLOAD_LED_UPLOADING)
    {
        app_ble_export_ui_tick(runtime_ms);
        return;
    }

    s_ble_upload_led_state = APP_BLE_UPLOAD_LED_UPLOADING;
    app_ble_export_port_ui_cancel_charge("ble_upload_green");
    app_ble_export_port_ui_stop_export_wait();
    app_ble_export_port_ui_stop_clear_red();
    led_release(LED_OWNER_FLASH_FULL_WAIT_UPLOAD);
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
    app_ble_export_port_ui_cancel_storage_retry();
#endif
    led_pattern_notify_blink(&pattern,
                             LED_PATTERN_COLOR_GREEN,
                             (uint32_t)APP_BLE_UPLOAD_LED_FAST_ON_MS,
                             (uint32_t)APP_BLE_UPLOAD_LED_FAST_OFF_MS,
                             0U,
                             true);
    (void)led_request(LED_OWNER_BLE_EXPORT,
                      LED_PRIORITY_BLE_EXPORT,
                      &pattern);
    APP_LED_LOG("[BLE_LED] upload_green_fast_blink reason=%s on_ms=%lu off_ms=%lu logo_off=1",
                (reason != NULL) ? reason : "ble_upload",
                (unsigned long)APP_BLE_UPLOAD_LED_FAST_ON_MS,
                (unsigned long)APP_BLE_UPLOAD_LED_FAST_OFF_MS);
}

static void app_ble_export_ui_begin_success(const char *reason)
{
    led_pattern_t pattern;

    if (app_ble_export_ui_button_override_active())
    {
        app_ble_export_ui_cancel(reason);
        return;
    }

    s_ble_upload_led_state = APP_BLE_UPLOAD_LED_SUCCESS_HOLD;
    app_ble_export_port_ui_cancel_charge("ble_upload_green");
    app_ble_export_port_ui_stop_export_wait();
    app_ble_export_port_ui_stop_clear_red();
    led_release(LED_OWNER_FLASH_FULL_WAIT_UPLOAD);
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
    app_ble_export_port_ui_cancel_storage_retry();
#endif
    app_ble_export_port_ui_release_training_end(reason);
    led_pattern_notify_solid(&pattern, LED_PATTERN_COLOR_GREEN, true);
    pattern.hold_ms = (uint32_t)APP_BLE_UPLOAD_SUCCESS_HOLD_MS;
    (void)led_request(LED_OWNER_BLE_EXPORT,
                      LED_PRIORITY_BLE_EXPORT,
                      &pattern);
    APP_LED_LOG("[BLE_LED] upload_success_green_hold reason=%s hold_ms=%lu logo_off=1",
                (reason != NULL) ? reason : "ble_upload_success",
                (unsigned long)APP_BLE_UPLOAD_SUCCESS_HOLD_MS);
}

static void app_ble_export_ui_begin_failure(const char *reason)
{
    led_pattern_t pattern;

    if (app_ble_export_ui_button_override_active())
    {
        app_ble_export_ui_cancel(reason);
        return;
    }

    s_ble_upload_led_state = APP_BLE_UPLOAD_LED_FAIL_BLINK;
    app_ble_export_port_ui_cancel_charge("ble_upload_red");
    app_ble_export_port_ui_stop_export_wait();
    app_ble_export_port_ui_stop_clear_red();
    led_release(LED_OWNER_FLASH_FULL_WAIT_UPLOAD);
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
    app_ble_export_port_ui_cancel_storage_retry();
#endif
    led_pattern_notify_blink(&pattern,
                             LED_PATTERN_COLOR_RED,
                             (uint32_t)APP_BLE_UPLOAD_LED_FAST_ON_MS,
                             (uint32_t)APP_BLE_UPLOAD_LED_FAST_OFF_MS,
                             APP_BLE_UPLOAD_FAIL_BLINK_CYCLES,
                             true);
    (void)led_request(LED_OWNER_BLE_EXPORT,
                      LED_PRIORITY_BLE_EXPORT,
                      &pattern);
    APP_LED_LOG("[BLE_LED] upload_fail_red_fast_blink reason=%s cycles=%lu on_ms=%lu off_ms=%lu logo_off=1",
                (reason != NULL) ? reason : "ble_upload_fail",
                (unsigned long)APP_BLE_UPLOAD_FAIL_BLINK_CYCLES,
                (unsigned long)APP_BLE_UPLOAD_LED_FAST_ON_MS,
                (unsigned long)APP_BLE_UPLOAD_LED_FAST_OFF_MS);
}

void app_ble_export_ui_show_blue(uint8_t owner,
                                 uint8_t priority,
                                 const char *reason)
{
    const char *use_reason = (reason != NULL) ? reason : "ble_export_processing";

    app_ble_export_port_ui_stop_export_wait();
    app_ble_export_port_ui_request_blue(owner, priority, use_reason);
    app_ble_export_port_ui_log_blue(use_reason);
}

static void app_ble_export_blue_blink(const char *reason) __attribute__((unused));
static void app_ble_export_blue_blink(const char *reason)
{
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
    app_ble_export_ui_show_blue((uint8_t)LED_OWNER_BLE_EXPORT,
                           LED_PRIORITY_BLE_EXPORT,
                           (reason != NULL) ? reason : "ble_export_processing");
#else
    app_ble_export_port_ui_start_export_wait();
    if (reason != NULL)
    {
        APP_LED_LOG("[BLE_LED] %s blue_blink=1", reason);
    }
#endif
}

static bool app_ble_export_conn_ready(uint8_t conn_id)
{
    return app_ble_power_conn_valid(conn_id) &&
           zy100_ble_ctrl_service_ack_notify_enabled(conn_id) &&
           zy100_ble_ctrl_service_export_notify_enabled(conn_id);
}

static void app_ble_export_progress_reset(void)
{
    s_ble_export.progress_logged = false;
    s_ble_export.progress_section = APP_BLE_EXPORT_SECTION_INVALID;
    s_ble_export.progress_last_section = APP_BLE_EXPORT_SECTION_INVALID;
    s_ble_export.progress_last_sent = 0U;
    s_ble_export.progress_last_ms = 0ULL;
}

static void app_ble_export_log_progress(
    const zy100_feuf_producer_progress_t *progress,
    bool force)
{
    uint64_t now_ms = os_sys_time_get();
    uint32_t section = s_ble_export.progress_section;
    bool section_changed;
    bool byte_step;
    bool time_step;

    if (progress != NULL)
    {
        section = progress->section_id;
        s_ble_export.progress_section = section;
    }
    if (section == APP_BLE_EXPORT_SECTION_INVALID)
    {
        section = 0U;
    }

    section_changed = (section != s_ble_export.progress_last_section);
    byte_step = (s_ble_export.total_bytes >= s_ble_export.progress_last_sent) &&
                ((s_ble_export.total_bytes -
                  s_ble_export.progress_last_sent) >=
                 APP_BLE_EXPORT_PROGRESS_BYTES);
    time_step = (s_ble_export.progress_last_ms == 0ULL) ||
                ((now_ms - s_ble_export.progress_last_ms) >=
                 APP_BLE_EXPORT_PROGRESS_MS);
    if (force ||
        !s_ble_export.progress_logged ||
        section_changed ||
        byte_step ||
        time_step)
    {
#if ZY100_BLE_EXPORT_MID_LOG_ENABLE
        DBG_DIRECT("[BLE_EXPORT] progress export_id=%lu section=%u sent=%lu chunk=%lu crc=0x%08lX",
                   (unsigned long)s_ble_export.export_id,
                   section,
                   (unsigned long)s_ble_export.total_bytes,
                   (unsigned long)s_ble_export.chunk_count,
                   (unsigned long)zy100_crc32_ieee_finish(s_ble_export.crc_state));
#endif
        s_ble_export.progress_logged = true;
        s_ble_export.progress_last_section = section;
        s_ble_export.progress_last_sent = s_ble_export.total_bytes;
        s_ble_export.progress_last_ms = now_ms;
    }
}

static void app_ble_export_confirm_context_clear(void)
{
    memset(&s_ble_export_confirm_ctx, 0, sizeof(s_ble_export_confirm_ctx));
}

static void app_ble_export_confirm_context_store(uint8_t seq,
                                                 uint32_t export_id,
                                                 uint64_t host_time_ms)
{
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
    memset(&s_ble_export_confirm_ctx, 0, sizeof(s_ble_export_confirm_ctx));
    s_ble_export_confirm_ctx.valid = true;
    s_ble_export_confirm_ctx.seq = seq;
    s_ble_export_confirm_ctx.conn_id = s_ble_export.conn_id;
    s_ble_export_confirm_ctx.export_id = export_id;
    s_ble_export_confirm_ctx.host_time_ms = host_time_ms;
    s_ble_export_confirm_ctx.user_id = s_ble_export.producer.session.user_id;
    s_ble_export_confirm_ctx.training_id =
        s_ble_export.producer.session.training_id;
#else
    app_task_training_context_t training_ctx;

    memset(&training_ctx, 0, sizeof(training_ctx));
    app_task_training_context_get(&training_ctx);
    memset(&s_ble_export_confirm_ctx, 0, sizeof(s_ble_export_confirm_ctx));
    s_ble_export_confirm_ctx.valid = true;
    s_ble_export_confirm_ctx.seq = seq;
    s_ble_export_confirm_ctx.conn_id = s_ble_export.conn_id;
    s_ble_export_confirm_ctx.export_id = export_id;
    s_ble_export_confirm_ctx.host_time_ms = host_time_ms;
    if (training_ctx.completed_export_valid)
    {
        s_ble_export_confirm_ctx.user_id =
            training_ctx.completed_export_user_id;
        s_ble_export_confirm_ctx.training_id =
            training_ctx.completed_export_training_id;
    }
    else if (training_ctx.current_capture_started)
    {
        s_ble_export_confirm_ctx.user_id =
            training_ctx.current_capture_user_id;
        s_ble_export_confirm_ctx.training_id =
            training_ctx.current_capture_training_id;
    }
    else
    {
        s_ble_export_confirm_ctx.user_id = training_ctx.active_user_id;
        s_ble_export_confirm_ctx.training_id = training_ctx.next_training_id;
    }
#endif
}

static void app_ble_export_clear_notify_failed(const char *reason)
{
    const char *use_reason = (reason != NULL) ? reason : "unknown";

    if (!s_ble_export_confirm_ctx.notify_failed_logged)
    {
        DBG_DIRECT("[BLE_EXPORT] clear_notify_failed export_id=%lu seq=%u reason=%s",
                   (unsigned long)s_ble_export_confirm_ctx.export_id,
                   s_ble_export_confirm_ctx.seq,
                   use_reason);
        s_ble_export_confirm_ctx.notify_failed_logged = true;
    }
}

static void app_ble_export_clear_notify_final(uint8_t status,
                                              uint8_t device_state,
                                              uint32_t detail)
{
    bool sent;

    if (!s_ble_export_confirm_ctx.valid)
    {
        app_ble_export_clear_notify_failed("no_context");
        app_ble_export_confirm_context_clear();
        return;
    }
    if (!app_ble_power_conn_valid(s_ble_export_confirm_ctx.conn_id))
    {
        app_ble_export_clear_notify_failed("link_not_ready");
        app_ble_export_confirm_context_clear();
        return;
    }
    if (!zy100_ble_ctrl_service_ack_notify_enabled(
            s_ble_export_confirm_ctx.conn_id))
    {
        app_ble_export_clear_notify_failed("ack_notify_disabled");
        app_ble_export_confirm_context_clear();
        return;
    }

    sent = zy100_ble_ctrl_service_notify_async_result(
        ZY100_BLE_CMD_EXPORT_CONFIRM,
        s_ble_export_confirm_ctx.seq,
        status,
        device_state,
        ZY100_BLE_EXEC_MODE_ASYNC_DONE,
        s_ble_export_confirm_ctx.user_id,
        s_ble_export_confirm_ctx.training_id,
        detail);
    if (sent)
    {
        if (ZY100_BLE_EXPORT_DETAIL_LOG_ENABLE)
        {
            DBG_DIRECT("[BLE_EXPORT] clear_notify export_id=%lu seq=%u status=%u state=%u",
                       (unsigned long)s_ble_export_confirm_ctx.export_id,
                       s_ble_export_confirm_ctx.seq,
                       status,
                       device_state);
        }
    }
    else
    {
        app_ble_export_clear_notify_failed("send_failed");
    }
    app_ble_export_confirm_context_clear();
}

static void app_ble_export_abort_keep_flash_common(const char *reason,
                                                   bool show_failure_led)
{
    uint32_t export_id = s_ble_export.export_id;
    app_ble_export_state_t state = s_ble_export.state;
    uint32_t bytes_sent = s_ble_export.total_bytes;

    if (state == APP_BLE_EXPORT_WAIT_CONFIRM)
    {
        DBG_DIRECT("[BLE_EXPORT] disconnect abort export_id=%lu state=wait_confirm keep_flash=1",
                   (unsigned long)export_id);
    }
    else
    {
        DBG_DIRECT("[BLE_EXPORT] disconnect abort export_id=%lu state=%u bytes_sent=%lu keep_flash=1",
                   (unsigned long)export_id,
                   (uint32_t)state,
                   (unsigned long)bytes_sent);
    }
    (void)reason;
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    app_ble_export_p0_record_fail(
        ((reason != NULL) && (strcmp(reason, "link_not_ready") == 0)) ?
        APP_BLE_P0_FAIL_STAGE_LINK : APP_BLE_P0_FAIL_STAGE_NONE,
        0U,
        s_ble_export.chunk_seq,
        0U,
        0U);
    app_ble_export_p0_log_summary(
        ((reason != NULL) && (strcmp(reason, "disconnect") == 0)) ?
        "disconnect_abort" : "abort_keep_flash");
#endif
#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
    app_ble_export_fast_conn_end(false);
#endif
    memset(&s_ble_export, 0, sizeof(s_ble_export));
    s_ble_export.state = APP_BLE_EXPORT_FAILED;
    app_ble_export_confirm_context_clear();
    app_ble_export_port_set_in_progress(false);
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
    app_ble_export_refresh_storage("ble_export_abort");
    if (!show_failure_led)
    {
        app_ble_export_ui_cancel("ble_disconnect_silent");
        APP_LED_LOG("[BLE_EXPORT] abort_keep_flash_silent reason=%s",
                    (reason != NULL) ? reason : "unknown");
        return;
    }
    if (app_ble_export_port_flash_full_wait_gate_active() &&
        !app_ble_power_is_connected())
    {
        app_ble_export_port_ui_restore_capture("flash_full_wait_ble_upload");
        return;
    }
#endif
    if (!show_failure_led)
    {
        app_ble_export_ui_cancel("ble_disconnect_silent");
        APP_LED_LOG("[BLE_EXPORT] abort_keep_flash_silent reason=%s",
                    (reason != NULL) ? reason : "unknown");
        return;
    }

    app_ble_export_ui_begin_failure(
        app_ble_power_is_connected() ? "ble_export_abort" : "ble_disconnected");
}

void app_ble_export_abort_keep_flash(const char *reason)
{
    app_ble_export_abort_keep_flash_common(reason, true);
}

static void app_ble_export_abort_keep_flash_silent(const char *reason)
{
    app_ble_export_abort_keep_flash_common(reason, false);
}

#if F_APP_BLE_ENABLE
bool app_ble_export_controller_prepare_for_sleep_request(void)
{
    if (s_ble_export.state == APP_BLE_EXPORT_CLEAR_AFTER_CONFIRM)
    {
        /* Reclaim/delete is already running; keep sleep pending until poll finishes. */
        DBG_DIRECT("[LP_REQ] defer reason=ble_export_clear_after_confirm");
        return false;
    }

    if (app_ble_export_state_streaming(s_ble_export.state) ||
        (s_ble_export.state == APP_BLE_EXPORT_WAIT_CONFIRM) ||
        (s_ble_export.state == APP_BLE_EXPORT_WAIT_SUBSCRIBE))
    {
        /* BLE transport can stop; flash data remains available for later export. */
        DBG_DIRECT("[LP_REQ] abort_ble_export_for_sleep state=%u",
                   (uint32_t)s_ble_export.state);
        app_ble_export_abort_keep_flash("sleep");
    }

    return true;
}
#endif

void app_ble_export_on_disconnect(uint8_t conn_id)
{
    if ((s_ble_export.conn_id == conn_id) &&
        ((s_ble_export.state == APP_BLE_EXPORT_PREPARE) ||
         (s_ble_export.state == APP_BLE_EXPORT_START_FRAME) ||
         (s_ble_export.state == APP_BLE_EXPORT_STREAMING) ||
         (s_ble_export.state == APP_BLE_EXPORT_END_FRAME) ||
         (s_ble_export.state == APP_BLE_EXPORT_WAIT_CONFIRM)))
    {
        app_ble_export_abort_keep_flash_silent("disconnect");
    }
    else if ((s_ble_export.conn_id == conn_id) &&
             (s_ble_export.state == APP_BLE_EXPORT_CLEAR_AFTER_CONFIRM))
    {
        DBG_DIRECT("[BLE_EXPORT] disconnect during clear_after_confirm keep_reclaim=1");
        app_ble_export_confirm_context_clear();
    }
#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
    else if (app_ble_export_fast_conn_matches(conn_id))
    {
        app_ble_export_fast_conn_end(false);
    }
#endif
}

static bool app_ble_export_gate_ready(const char *reason,
                                      uint8_t *conn_id_out,
                                      uint32_t *pending_out)
{
    uint8_t conn_id = app_ble_power_conn_id();
    bool connected = app_ble_power_is_connected();
    bool ack_ntf = connected && zy100_ble_ctrl_service_ack_notify_enabled(conn_id);
    bool export_ntf = connected && zy100_ble_ctrl_service_export_notify_enabled(conn_id);
    bool sync_ok = app_ble_time_sync_ok();
    bool pairing_ok = app_ble_pairing_ready();
    bool device_ready = app_ble_export_port_device_logic_ready();
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
    uint32_t pending;
#else
    uint32_t pending = app_ble_export_port_pending_session_count();
#endif
    bool busy_state = app_ble_export_is_active();
    bool blocked = app_ble_export_port_gate_blocks_export() ||
                   s_ble_export_clear_retry_required ||
                   app_ble_export_ui_blocks_normal_flow();

#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
    app_ble_export_refresh_storage(
        (reason != NULL) ? reason : "ble_export_gate");
    pending = s_ble_export_storage_state.pending_count;
    blocked = blocked ||
              s_ble_export_storage_state.clear_active ||
              s_ble_export_storage_state.reclaim_active ||
              s_ble_export_storage_state.expc_without_rcld ||
              s_ble_export_resume_reclaim_active;
#endif
    if (pending_out != NULL)
    {
        *pending_out = pending;
    }
    if (conn_id_out != NULL)
    {
        *conn_id_out = conn_id;
    }
    if (connected && sync_ok && (pending != 0U) &&
        (!ack_ntf || !export_ntf) &&
        !app_ble_export_port_gate_blocks_export())
    {
        s_ble_export.state = APP_BLE_EXPORT_WAIT_SUBSCRIBE;
        DBG_DIRECT("[BLE_EXPORT] wait_ready reason=notify_not_enabled pending=%lu",
                   (unsigned long)pending);
    }
    if (connected && ack_ntf && export_ntf && sync_ok && (pending != 0U) &&
        pairing_ok && device_ready && !blocked && !busy_state)
    {
        return true;
    }

    if (ZY100_BLE_EXPORT_DETAIL_LOG_ENABLE)
    {
        DBG_DIRECT("[BLE_EXPORT] auto_skip reason=%s connected=%u ack_ntf=%u export_ntf=%u sync_ok=%u pairing_ok=%u device_ready=%u pending=%u state=%u",
                   (reason != NULL) ? reason : "auto",
                   connected ? 1U : 0U,
                   ack_ntf ? 1U : 0U,
                   export_ntf ? 1U : 0U,
                   sync_ok ? 1U : 0U,
                   pairing_ok ? 1U : 0U,
                   device_ready ? 1U : 0U,
                   pending,
                   (uint32_t)s_ble_export.state);
    }
    return false;
}

void app_ble_export_try_auto_start(const char *reason)
{
    uint8_t conn_id = 0xFFU;
    uint32_t pending = 0U;
#if !(ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE)
    uint32_t bytes = 0U;
#endif

    if (app_ble_export_port_ota_blocks_new_business())
    {
        DBG_DIRECT("[OTA_GATE] block start=offline_export reason=%s",
                   (reason != NULL) ? reason : "unknown");
        return;
    }

    if (app_ble_export_port_b_critical_active())
    {
        app_ble_export_port_note_b_critical_block(
            APP_BLE_EXPORT_PORT_BLOCK_EXPORT);
        return;
    }

    if (app_ble_export_port_link_led_auto_export_defer(reason))
    {
        return;
    }

#if !(ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE)
    bytes = app_ble_export_port_pending_total_bytes();
    if (ZY100_BLE_EXPORT_DETAIL_LOG_ENABLE)
    {
        DBG_DIRECT("[BLE_EXPORT] auto_check %s pending=%u sessions=%u bytes=%lu",
                   (reason != NULL) ? reason : "auto",
                   app_ble_export_port_has_pending_sessions() ? 1U : 0U,
                   app_ble_export_port_pending_session_count(),
                   (unsigned long)bytes);
    }
    if (app_ble_export_port_pending_session_count() == 0U)
    {
        if (ZY100_BLE_EXPORT_DETAIL_LOG_ENABLE)
        {
            DBG_DIRECT("[BLE_EXPORT] auto_no_pending skip_meta_repair reason=%s",
                       (reason != NULL) ? reason : "auto");
        }
    }
#else
    app_ble_export_refresh_storage(
        (reason != NULL) ? reason : "ble_export_auto");
    if (ZY100_BLE_EXPORT_DETAIL_LOG_ENABLE)
    {
        DBG_DIRECT("[BLE_EXPORT] auto_check %s pending=%lu can_next=%u error=%s",
                   (reason != NULL) ? reason : "auto",
                   (unsigned long)s_ble_export_storage_state.pending_count,
                   s_ble_export_storage_state.can_start_next ? 1U : 0U,
                   (s_ble_export_storage_state.last_error_reason != NULL) ?
                   s_ble_export_storage_state.last_error_reason : "unknown");
    }
    if (s_ble_export_storage_state.pending_count == 0U)
    {
        if (ZY100_BLE_EXPORT_DETAIL_LOG_ENABLE)
        {
            DBG_DIRECT("[BLE_EXPORT] auto_no_pending reason=%s",
                       (reason != NULL) ? reason : "auto");
        }
    }
#endif
    if (!app_ble_export_gate_ready(reason, &conn_id, &pending))
    {
        return;
    }

#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
    if (s_ble_export.tx_in_flight != 0U)
    {
        app_ble_export_post_tx_ready_event();
        return;
    }
#endif

    memset(&s_ble_export, 0, sizeof(s_ble_export));
#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
    (void)app_ble_export_tx_reset();
#endif
    s_ble_export.state = APP_BLE_EXPORT_PREPARE;
    s_ble_export.conn_id = conn_id;
    s_ble_export.crc_state = zy100_crc32_ieee_begin();
    s_ble_export.current_export_total = pending;
    s_ble_export.current_export_index = 0U;
    s_ble_export.batch_exported_count = 0U;
    app_ble_export_progress_reset();
    app_ble_export_confirm_context_clear();
    app_ble_export_port_set_in_progress(true);
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
    app_ble_export_ui_begin_upload("ble_export_streaming");
    app_ble_export_refresh_storage("ble_export_start");
    if (ZY100_BLE_EXPORT_DETAIL_LOG_ENABLE)
    {
        DBG_DIRECT("[BLE_EXPORT] auto_start reason=%s pending=%lu",
                   (reason != NULL) ? reason : "auto",
                   (unsigned long)pending);
    }
#else
    app_ble_export_ui_begin_upload("ble_export");
    if (ZY100_BLE_EXPORT_DETAIL_LOG_ENABLE)
    {
        DBG_DIRECT("[BLE_EXPORT] auto_start reason=%s single_session=1",
                   (reason != NULL) ? reason : "auto");
        DBG_DIRECT("[BLE_EXPORT] auto_start reason=%s sessions=%u",
                   (reason != NULL) ? reason : "auto",
                   pending);
    }
#endif
    app_ble_export_port_post_ctrl_event();
}

static bool app_ble_export_prepare_mtu(void)
{
    uint16_t mtu = app_ble_power_conn_mtu(s_ble_export.conn_id);
    uint16_t notify_max;
    uint16_t chunk_payload;

    if (mtu == 0U)
    {
        mtu = APP_BLE_EXPORT_DEFAULT_ATT_MTU;
    }
    notify_max = (mtu > APP_BLE_EXPORT_ATT_HEADER_BYTES) ?
                 (uint16_t)(mtu - APP_BLE_EXPORT_ATT_HEADER_BYTES) : 0U;
    if (notify_max > APP_BLE_EXPORT_MAX_NOTIFY_BYTES)
    {
        notify_max = APP_BLE_EXPORT_MAX_NOTIFY_BYTES;
    }
    chunk_payload = (notify_max > APP_BLE_EXPORT_HEADER_BYTES) ?
                    (uint16_t)(notify_max - APP_BLE_EXPORT_HEADER_BYTES) : 0U;
    s_ble_export.notify_max = notify_max;
    s_ble_export.chunk_payload = chunk_payload;
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    app_ble_export_p0_set_notify_shape(s_ble_export.conn_id,
                                       notify_max,
                                       chunk_payload);
#endif
    if (ZY100_BLE_EXPORT_DETAIL_LOG_ENABLE && !s_ble_export.mtu_logged)
    {
        DBG_DIRECT("[BLE_EXPORT] mtu=%u notify_max=%u chunk_payload=%u",
                   mtu,
                   notify_max,
                   chunk_payload);
        s_ble_export.mtu_logged = true;
    }
    return chunk_payload > 0U;
}

static void app_ble_export_prepare_ctrl(uint8_t frame_type,
                                        uint16_t chunk_seq,
                                        const uint8_t *payload,
                                        uint16_t payload_len)
{
    s_ble_export.ctrl_type = frame_type;
    s_ble_export.ctrl_seq = chunk_seq;
    s_ble_export.ctrl_len = payload_len;
    s_ble_export.ctrl_offset = 0U;
    if ((payload != NULL) && (payload_len != 0U))
    {
        memcpy(s_ble_export.ctrl_payload, payload, payload_len);
    }
}

static bool app_ble_export_send_false_can_block(void)
{
    return app_ble_power_conn_valid(s_ble_export.conn_id) &&
           zy100_ble_ctrl_service_export_notify_enabled(s_ble_export.conn_id) &&
           app_ble_export_state_streaming(s_ble_export.state)
#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
           && !s_ble_export.tx_fatal_complete
#endif
           ;
}

static app_ble_export_send_result_t app_ble_export_note_send_blocked(
    uint8_t frame_type,
    uint16_t chunk_seq,
    uint16_t frame_len)
{
#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
    uint32_t now_ms = (uint32_t)os_sys_time_get();
    bool timeout = false;

    s_ble_export.send_blocked_count++;
    s_ble_export.send_blocked_consecutive++;
    if (s_ble_export.send_blocked_first_ms == 0U)
    {
        s_ble_export.send_blocked_first_ms = now_ms;
    }
    else if ((uint32_t)(now_ms - s_ble_export.send_blocked_first_ms) >
             (uint32_t)ZY100_BLE_EXPORT_SEND_BLOCKED_TIMEOUT_MS)
    {
        timeout = true;
    }
    if (s_ble_export.send_blocked_consecutive >
        (uint32_t)ZY100_BLE_EXPORT_SEND_BLOCKED_MAX)
    {
        timeout = true;
    }
    app_ble_export_tx_set_backoff(now_ms);
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    app_ble_export_p0_note_send_blocked(frame_type,
                                        chunk_seq,
                                        frame_len,
                                        timeout);
#endif
    return timeout ? APP_BLE_EXPORT_SEND_FATAL :
           APP_BLE_EXPORT_SEND_BLOCKED;
#else
    (void)frame_type;
    (void)chunk_seq;
    (void)frame_len;
    return APP_BLE_EXPORT_SEND_FATAL;
#endif
}

static app_ble_export_send_result_t app_ble_export_send_frame(uint8_t frame_type,
                                                              uint8_t flags,
                                                              uint16_t chunk_seq,
                                                              const uint8_t *payload,
                                                              uint16_t payload_len)
{
    uint16_t frame_len = (uint16_t)(APP_BLE_EXPORT_HEADER_BYTES + payload_len);
    uint8_t *frame;
    bool sent;
    app_ble_export_send_result_t result = APP_BLE_EXPORT_SEND_OK;
#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
    app_ble_export_tx_slot_t *slot;
#endif
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    uint64_t send_start_us;
    uint64_t send_end_us;
#endif

    if (app_ble_export_port_b_critical_active())
    {
        app_ble_export_port_note_b_critical_block(
            APP_BLE_EXPORT_PORT_BLOCK_NOTIFY);
        return APP_BLE_EXPORT_SEND_LOCAL_BLOCKED;
    }

    if ((frame_len > s_ble_export.notify_max) ||
        (frame_len > APP_BLE_EXPORT_MAX_NOTIFY_BYTES) ||
        ((payload_len != 0U) && (payload == NULL)))
    {
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        app_ble_export_p0_note_build_failure(frame_type,
                                             chunk_seq,
                                             frame_len);
#endif
        app_ble_export_fail_keep_flash(0U);
        return APP_BLE_EXPORT_SEND_FATAL;
    }

#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
    slot = app_ble_export_tx_alloc_slot();
    if (slot == NULL)
    {
        return APP_BLE_EXPORT_SEND_LOCAL_BLOCKED;
    }
    frame = slot->frame;
#else
    frame = s_ble_export.tx_frame;
#endif

    frame[0] = APP_BLE_EXPORT_MAGIC;
    frame[1] = APP_BLE_EXPORT_VERSION;
    frame[2] = frame_type;
    frame[3] = flags;
    zy100_put_u16_le(&frame[4], chunk_seq);
    zy100_put_u16_le(&frame[6], payload_len);
    if (payload_len != 0U)
    {
        memcpy(&frame[APP_BLE_EXPORT_HEADER_BYTES],
               payload,
               payload_len);
    }
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    send_start_us = app_ble_export_p0_now_us();
#endif
    sent = zy100_ble_ctrl_service_send_export_notify(s_ble_export.conn_id,
                                                     frame,
                                                     frame_len);
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    send_end_us = app_ble_export_p0_now_us();
    app_ble_export_p0_note_send_call(sent,
                                     (uint32_t)(send_end_us - send_start_us),
                                     frame_type,
                                     frame_len,
                                     payload_len);
#endif
#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
    if (sent)
    {
        app_ble_export_tx_commit_slot(frame_len,
                                      frame_type,
                                      chunk_seq,
                                      payload_len);
        app_ble_export_tx_clear_backpressure();
    }
    else
    {
        app_ble_export_tx_cancel_slot();
    }
#endif
    if (!sent)
    {
        result = app_ble_export_send_false_can_block() ?
                 app_ble_export_note_send_blocked(frame_type,
                                                  chunk_seq,
                                                  frame_len) :
                 APP_BLE_EXPORT_SEND_FATAL;
        if (result == APP_BLE_EXPORT_SEND_FATAL)
        {
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
            app_ble_export_p0_note_send_failure(frame_type,
                                                chunk_seq,
                                                frame_len);
#endif
            app_ble_export_fail_keep_flash(0U);
        }
        return result;
    }
    return APP_BLE_EXPORT_SEND_OK;
}

static app_ble_export_ctrl_result_t app_ble_export_pump_ctrl(void)
{
    uint16_t remain;
    uint16_t frag_len;
    uint8_t flags = 0U;
    app_ble_export_send_result_t send_result;

    if (s_ble_export.ctrl_offset >= s_ble_export.ctrl_len)
    {
        return APP_BLE_EXPORT_CTRL_DONE;
    }
    remain = (uint16_t)(s_ble_export.ctrl_len - s_ble_export.ctrl_offset);
    frag_len = (remain < s_ble_export.chunk_payload) ?
               remain : s_ble_export.chunk_payload;
    if (frag_len == remain)
    {
        flags = APP_BLE_EXPORT_FLAG_LAST_FRAME;
    }
    send_result = app_ble_export_send_frame(
        s_ble_export.ctrl_type,
        flags,
        s_ble_export.ctrl_seq,
        &s_ble_export.ctrl_payload[s_ble_export.ctrl_offset],
        frag_len);
    if (send_result == APP_BLE_EXPORT_SEND_FATAL)
    {
        return APP_BLE_EXPORT_CTRL_FATAL;
    }
    if (send_result != APP_BLE_EXPORT_SEND_OK)
    {
        return APP_BLE_EXPORT_CTRL_BLOCKED;
    }
    s_ble_export.ctrl_offset = (uint16_t)(s_ble_export.ctrl_offset + frag_len);
    return (s_ble_export.ctrl_offset >= s_ble_export.ctrl_len) ?
           APP_BLE_EXPORT_CTRL_DONE : APP_BLE_EXPORT_CTRL_BLOCKED;
}

static void app_ble_export_prepare_start_payload(void)
{
    uint8_t payload[APP_BLE_EXPORT_CTRL_PAYLOAD_BYTES];

    memset(payload, 0, sizeof(payload));
    zy100_put_u32_le(&payload[0], s_ble_export.export_id);
    zy100_put_u32_le(&payload[4],
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
                              (s_ble_export.current_export_total != 0U) ?
                              s_ble_export.current_export_total :
                              s_ble_export_storage_state.pending_count
#else
                              app_ble_export_port_pending_session_count()
#endif
                              );
    zy100_put_u32_le(&payload[8],
                              zy100_feuf_export_producer_estimated_payload_bytes(
                                  &s_ble_export.producer));
    app_ble_export_prepare_ctrl(APP_BLE_EXPORT_FRAME_START,
                                0U,
                                payload,
                                (uint16_t)sizeof(payload));
}

static void app_ble_export_prepare_end_payload(void)
{
    uint8_t payload[APP_BLE_EXPORT_CTRL_PAYLOAD_BYTES];

    memset(payload, 0, sizeof(payload));
    s_ble_export.stream_crc32 =
        zy100_crc32_ieee_finish(s_ble_export.crc_state);
    s_ble_export.current_session_crc = s_ble_export.stream_crc32;
    zy100_put_u32_le(&payload[0], s_ble_export.export_id);
    zy100_put_u32_le(&payload[4], s_ble_export.total_bytes);
    zy100_put_u32_le(&payload[8], s_ble_export.stream_crc32);
    zy100_put_u32_le(&payload[12], s_ble_export.chunk_count);
    app_ble_export_prepare_ctrl(APP_BLE_EXPORT_FRAME_END,
                                s_ble_export.chunk_seq,
                                payload,
                                (uint16_t)sizeof(payload));
    s_ble_export.current_end_seq = s_ble_export.chunk_seq;
}

static void app_ble_export_fail_keep_flash(uint32_t detail)
{
    ZY100_LOG_ERROR("[ERR][UPLOAD] abort detail=%lu keep_flash=1",
                    (unsigned long)detail);
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    app_ble_export_p0_note_producer_failure(
        zy100_feuf_export_producer_error(&s_ble_export.producer),
        s_ble_export.chunk_seq,
        detail);
    app_ble_export_p0_log_summary("fail_keep_flash");
#endif
#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
    app_ble_export_fast_conn_end(false);
#endif
#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
    app_ble_export_tx_stop_flow_control();
#endif
    app_ble_export_port_set_in_progress(false);
    s_ble_export.state = APP_BLE_EXPORT_FAILED;
    app_ble_export_confirm_context_clear();
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
    app_ble_export_refresh_storage("ble_export_fail");
#endif
    app_ble_export_ui_begin_failure("ble_export_fail");
}

#if !(ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE)
static void app_ble_export_clear_done(void)
{
    bool done = app_ble_export_port_clear_flash_finalize();

    if (done)
    {
        app_ble_export_clear_notify_final(ZY100_BLE_ACK_STATUS_OK,
                                          ZY100_BLE_DEVICE_STATE_WAIT_START,
                                          0U);
        app_ble_export_ui_begin_success("ble_export_clear_done");
    }
    else
    {
        app_ble_export_clear_failed(1U);
    }
}
#endif

static void app_ble_export_clear_failed(uint32_t detail)
{
    uint32_t export_id = s_ble_export.export_id;

    DBG_DIRECT("[BLE_EXPORT] clear_failed_after_confirm export_id=%lu retry_required=1",
               (unsigned long)export_id);
    DBG_DIRECT("[CAP_CLEAR] failed source=ble_export_confirm detail=%lu start_blocked=1",
               (unsigned long)detail);
#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
    app_ble_export_fast_conn_end(false);
#endif
    s_ble_export_clear_retry_required = true;
    s_ble_export_clear_failed_export_id = export_id;
    app_ble_export_port_set_in_progress(false);
    s_ble_export.state = APP_BLE_EXPORT_FAILED;
    app_ble_export_port_set_clear_failed_state();
    app_ble_export_clear_notify_final(ZY100_BLE_ACK_STATUS_INTERNAL_ERROR,
                                      ZY100_BLE_DEVICE_STATE_ERROR,
                                      detail);
    app_ble_export_ui_begin_failure("ble_export_clear_failed");
}

uint8_t app_ble_export_controller_request_confirm(uint8_t seq,
                                                              uint32_t export_id,
                                                              uint64_t host_time_ms,
                                                              uint32_t result)
{
    if ((result != 0U) || ZY100_BLE_EXPORT_DETAIL_LOG_ENABLE)
    {
        DBG_DIRECT("[BLE_EXPORT] confirm export_id=%lu result=%lu",
                   (unsigned long)export_id,
                   (unsigned long)result);
    }
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
    if ((s_ble_export.state != APP_BLE_EXPORT_WAIT_CONFIRM) ||
        (result != 0U) ||
        (export_id != s_ble_export.current_export_id) ||
        (s_ble_export.current_export_session_uid == 0U) ||
        !s_ble_export.end_sent ||
        !app_ble_power_conn_valid(s_ble_export.conn_id))
    {
        DBG_DIRECT("[BLE_EXPORT] confirm_reject reason=bad_state export_id=%lu uid=%lu state=%u",
                   (unsigned long)export_id,
                   (unsigned long)s_ble_export.current_export_session_uid,
                   (uint32_t)s_ble_export.state);
        if (result != 0U)
        {
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
            app_ble_export_p0_record_fail(APP_BLE_P0_FAIL_STAGE_CONFIRM,
                                          0U,
                                          s_ble_export.current_end_seq,
                                          0U,
                                          (uint16_t)result);
            app_ble_export_p0_log_summary("confirm_reject");
#endif
#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
            app_ble_export_fast_conn_end(false);
#endif
            s_ble_export.state = APP_BLE_EXPORT_FAILED;
            app_ble_export_port_set_in_progress(false);
            app_ble_export_refresh_storage("ble_export_fail");
            app_ble_export_ui_begin_failure("ble_export_confirm_reject");
        }
        app_ble_export_confirm_context_clear();
        return APP_BLE_CTRL_REQ_RESULT_OK;
    }
    app_ble_export_confirm_context_store(seq, export_id, host_time_ms);
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    app_ble_export_p0_mark_final();
#endif
    if (!zy100_session_range_table_confirm_export_tail(
            s_ble_export.current_export_session_uid,
            s_ble_export.stream_crc32,
            s_ble_export.total_bytes) ||
        !zy100_session_range_table_reclaim_tail_begin(
            s_ble_export.current_export_session_uid))
    {
        DBG_DIRECT("[BLE_EXPORT] confirm_reject reason=bad_state export_id=%lu uid=%lu state=%u",
                   (unsigned long)export_id,
                   (unsigned long)s_ble_export.current_export_session_uid,
                   (uint32_t)s_ble_export.state);
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        app_ble_export_p0_record_fail(APP_BLE_P0_FAIL_STAGE_CONFIRM,
                                      0U,
                                      s_ble_export.current_end_seq,
                                      0U,
                                      0U);
        app_ble_export_p0_log_summary("confirm_reject");
#endif
#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
        app_ble_export_fast_conn_end(false);
#endif
        s_ble_export.state = APP_BLE_EXPORT_FAILED;
        app_ble_export_port_set_in_progress(false);
        app_ble_export_confirm_context_clear();
        app_ble_export_refresh_storage("ble_export_confirm_fail");
        app_ble_export_ui_begin_failure("ble_export_confirm_fail");
        return APP_BLE_CTRL_REQ_RESULT_OK;
    }

    DBG_DIRECT("[BLE_EXPORT] confirmed_ok export_id=%lu uid=%lu",
               (unsigned long)export_id,
               (unsigned long)s_ble_export.current_export_session_uid);
    if (ZY100_BLE_EXPORT_DETAIL_LOG_ENABLE)
    {
        DBG_DIRECT("[BLE_EXPORT] auto_confirm_ok uid=%lu",
                   (unsigned long)s_ble_export.current_export_session_uid);
    }
    DBG_DIRECT("[SESSION_RECLAIM] begin uid=%lu",
               (unsigned long)s_ble_export.current_export_session_uid);
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    app_ble_export_p0_log_summary("confirm_ok");
#endif
#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
    app_ble_export_tx_stop_flow_control();
#endif
    s_ble_export.state = APP_BLE_EXPORT_CLEAR_AFTER_CONFIRM;
    s_ble_export.clear_begin_requested = false;
    app_ble_export_ui_begin_upload("ble_export_processing");
    app_ble_export_refresh_storage("reclaim_begin");
    imu_fifo_drain_test_final_edge_set_export_cleanup();
    app_ble_export_port_set_export_cleanup();
    app_ble_export_port_post_ctrl_event();
    return APP_BLE_CTRL_REQ_RESULT_OK;
#else
    if ((s_ble_export.state != APP_BLE_EXPORT_WAIT_CONFIRM) ||
        (result != 0U) ||
        (export_id != s_ble_export.export_id) ||
        !s_ble_export.end_sent ||
        !app_ble_power_conn_valid(s_ble_export.conn_id))
    {
        DBG_DIRECT("[BLE_EXPORT] confirm_reject export_id=%lu current=%lu result=%lu keep_flash=1",
                   (unsigned long)export_id,
                   (unsigned long)s_ble_export.export_id,
                   (unsigned long)result);
        if (result != 0U)
        {
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
            app_ble_export_p0_record_fail(APP_BLE_P0_FAIL_STAGE_CONFIRM,
                                          0U,
                                          s_ble_export.current_end_seq,
                                          0U,
                                          (uint16_t)result);
            app_ble_export_p0_log_summary("confirm_reject");
#endif
#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
            app_ble_export_fast_conn_end(false);
#endif
            s_ble_export.state = APP_BLE_EXPORT_FAILED;
            app_ble_export_port_set_in_progress(false);
            app_ble_export_ui_begin_failure("ble_export_confirm_reject");
        }
        app_ble_export_confirm_context_clear();
        return APP_BLE_CTRL_REQ_RESULT_OK;
    }

    app_ble_export_confirm_context_store(seq, export_id, host_time_ms);
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    app_ble_export_p0_mark_final();
#endif
    DBG_DIRECT("[BLE_EXPORT] confirmed_ok export_id=%lu",
               (unsigned long)export_id);
    if (ZY100_BLE_EXPORT_DETAIL_LOG_ENABLE)
    {
        DBG_DIRECT("[BLE_EXPORT] clear_after_confirm export_id=%lu",
                   (unsigned long)export_id);
    }
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    app_ble_export_p0_log_summary("confirm_ok");
#endif
#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
    app_ble_export_tx_stop_flow_control();
#endif
#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
    app_ble_export_fast_conn_end(true);
#endif
    s_ble_export.state = APP_BLE_EXPORT_CLEAR_AFTER_CONFIRM;
    s_ble_export.clear_begin_requested = true;
    app_ble_export_ui_begin_upload("ble_export_processing");
    imu_fifo_drain_test_final_edge_set_export_cleanup();
    app_ble_export_port_set_export_cleanup();
    app_ble_export_port_post_ctrl_event();
    return APP_BLE_CTRL_REQ_RESULT_OK;
#endif
}

void app_ble_export_poll(void)
{
    uint16_t payload_len = 0U;
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE || ZY100_BLE_EXPORT_AGGREGATE_PAYLOAD_ENABLE
    uint8_t aggregate_piece_count = 1U;
#endif
    zy100_feuf_producer_progress_t progress;
    zy100_flash_prepare_status_t prep_status;

    if (app_ble_export_port_b_critical_active())
    {
        app_ble_export_port_note_b_critical_block(
            APP_BLE_EXPORT_PORT_BLOCK_EXPORT);
        return;
    }

#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    bool p0_poll_tracked = false;
    uint32_t p0_notify_ok_before = 0U;
    uint64_t p0_prepare_start_us = 0ULL;

    app_ble_export_p0_poll_enter(&p0_poll_tracked,
                                 &p0_notify_ok_before);
#endif

    app_ble_export_ui_tick(zy100_os_time_ms());

#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
    if (!app_ble_export_is_active() &&
        (s_ble_export_resume_reclaim_active ||
         (s_ble_export_storage_state.expc_without_rcld &&
          (s_ble_export_storage_state.reclaim_session_uid != 0U))))
    {
        if (!s_ble_export_resume_reclaim_active)
        {
            s_ble_export_resume_reclaim_uid =
                s_ble_export_storage_state.reclaim_session_uid;
            if (!zy100_session_range_table_reclaim_tail_begin(
                    s_ble_export_resume_reclaim_uid))
            {
                app_ble_export_refresh_storage("reclaim_fail");
                app_ble_export_ui_begin_failure("reclaim_fail");
                APP_BLE_EXPORT_P0_POLL_RETURN();
            }
            s_ble_export_resume_reclaim_active = true;
            DBG_DIRECT("[SESSION_RECLAIM] begin uid=%lu",
                       (unsigned long)s_ble_export_resume_reclaim_uid);
            app_ble_export_ui_begin_upload("ble_export_processing");
            app_ble_export_refresh_storage("reclaim_begin");
        }
        prep_status = zy100_session_range_table_reclaim_tail_poll();
        if (prep_status == ZY100_FLASH_PREP_BUSY)
        {
            app_ble_export_ui_begin_upload("ble_export_processing");
            APP_BLE_EXPORT_P0_POLL_RETURN();
        }
        if (prep_status != ZY100_FLASH_PREP_DONE)
        {
            DBG_DIRECT("[SESSION_RECLAIM] fail uid=%lu status=%u",
                       (unsigned long)s_ble_export_resume_reclaim_uid,
                       (uint32_t)prep_status);
            s_ble_export_resume_reclaim_active = false;
            s_ble_export_resume_reclaim_uid = 0U;
            app_ble_export_refresh_storage("reclaim_fail");
            app_ble_export_ui_begin_failure("reclaim_fail");
            APP_BLE_EXPORT_P0_POLL_RETURN();
        }
        DBG_DIRECT("[SESSION_RECLAIM] done uid=%lu",
                   (unsigned long)s_ble_export_resume_reclaim_uid);
        s_ble_export_resume_reclaim_active = false;
        s_ble_export_resume_reclaim_uid = 0U;
        app_ble_export_refresh_storage("reclaim_done");
        app_ble_export_ui_begin_success("reclaim_done");
        APP_BLE_EXPORT_P0_POLL_RETURN();
    }
#endif

#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
    app_ble_export_fast_conn_maintain((uint32_t)os_sys_time_get());
#endif

    if ((s_ble_export.state == APP_BLE_EXPORT_IDLE) ||
        (s_ble_export.state == APP_BLE_EXPORT_DONE) ||
        (s_ble_export.state == APP_BLE_EXPORT_FAILED) ||
        (s_ble_export.state == APP_BLE_EXPORT_WAIT_SUBSCRIBE))
    {
        APP_BLE_EXPORT_P0_POLL_RETURN();
    }

    if ((s_ble_export.state != APP_BLE_EXPORT_CLEAR_AFTER_CONFIRM) &&
        !app_ble_export_conn_ready(s_ble_export.conn_id))
    {
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        app_ble_export_p0_note_poll_blocked();
#endif
        app_ble_export_abort_keep_flash("link_not_ready");
        APP_BLE_EXPORT_P0_POLL_RETURN();
    }

#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
    if (s_ble_export.tx_fatal_complete)
    {
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        app_ble_export_p0_record_fail(APP_BLE_P0_FAIL_STAGE_COMPLETE,
                                      0U,
                                      s_ble_export.chunk_seq,
                                      0U,
                                      s_ble_export.tx_fatal_complete_cause);
#endif
        app_ble_export_fail_keep_flash(
            (uint32_t)s_ble_export.tx_fatal_complete_cause);
        APP_BLE_EXPORT_P0_POLL_RETURN();
    }
#endif

    switch (s_ble_export.state)
    {
    case APP_BLE_EXPORT_PREPARE:
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        p0_prepare_start_us = app_ble_export_p0_now_us();
#endif
        if (!app_ble_export_prepare_mtu())
        {
            app_ble_export_fail_keep_flash(1U);
            APP_BLE_EXPORT_P0_POLL_RETURN();
        }
#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
        if (!app_ble_export_tx_reset())
        {
            app_ble_export_post_tx_ready_event();
            APP_BLE_EXPORT_P0_POLL_RETURN();
        }
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
        {
            zy100_session_index_entry_t session;
            uint32_t pending_count;

            app_ble_export_refresh_storage("ble_export_prepare");
            pending_count = s_ble_export_storage_state.pending_count;

            if (s_ble_export.current_export_total == 0U)
            {
                s_ble_export.current_export_total = pending_count;
            }
            if ((pending_count == 0U) ||
                !zy100_session_range_table_get_tail_pending(&session))
            {
                ZY100_LOG_EVENT("[EVT][UPLOAD] complete batch=1");
#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
                app_ble_export_fast_conn_end(true);
#endif
                app_ble_export_port_set_in_progress(false);
                s_ble_export.state = APP_BLE_EXPORT_DONE;
                imu_fifo_drain_test_final_edge_set_export_done();
                APP_BLE_EXPORT_P0_POLL_RETURN();
            }
#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
            app_ble_export_fast_conn_begin(s_ble_export.conn_id);
            if (!app_ble_export_fast_conn_ready_or_timeout())
            {
                APP_BLE_EXPORT_P0_POLL_RETURN();
            }
#endif
            s_ble_export.current_export_index =
                s_ble_export.batch_exported_count;
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
            app_ble_export_p0_reset("prepare");
            app_ble_export_p0_poll_enter(&p0_poll_tracked,
                                         &p0_notify_ok_before);
            app_ble_export_p0_set_prepare_start(p0_prepare_start_us);
            app_ble_export_p0_set_session(
                s_ble_export.conn_id,
                0U,
                session.session_uid,
                (uint16_t)s_ble_export.current_export_index,
                (uint16_t)s_ble_export.current_export_total);
            app_ble_export_p0_note_prepare_done();
            app_ble_export_p0_note_begin_start(&s_ble_export.producer);
#endif
            if (!zy100_feuf_export_producer_begin_session(
                    &s_ble_export.producer,
                    &session,
                    s_ble_export.current_export_index,
                    s_ble_export.current_export_total))
            {
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
                app_ble_export_p0_note_begin_done(&s_ble_export.producer);
#endif
                app_ble_export_fail_keep_flash(
                    zy100_feuf_export_producer_error(&s_ble_export.producer));
                APP_BLE_EXPORT_P0_POLL_RETURN();
            }
            zy100_feuf_export_producer_set_prefetch_buffer(
                &s_ble_export.producer,
                s_ble_export.prefetch_cache,
                (uint32_t)sizeof(s_ble_export.prefetch_cache));
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
            app_ble_export_p0_note_begin_done(&s_ble_export.producer);
#endif
            s_ble_export.export_id =
                zy100_feuf_export_producer_export_id(&s_ble_export.producer);
            s_ble_export.current_export_id = s_ble_export.export_id;
            s_ble_export.current_export_session_uid = session.session_uid;
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
            app_ble_export_p0_set_session(
                s_ble_export.conn_id,
                s_ble_export.export_id,
                s_ble_export.current_export_session_uid,
                (uint16_t)s_ble_export.current_export_index,
                (uint16_t)s_ble_export.current_export_total);
#endif
            if (ZY100_BLE_EXPORT_DETAIL_LOG_ENABLE)
            {
                DBG_DIRECT("[BLE_EXPORT] tail_session_start uid=%lu index_from_tail=%u pending=%u",
                           (unsigned long)session.session_uid,
                           s_ble_export.current_export_index,
                           pending_count);
            }
        }
#else
#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
        app_ble_export_fast_conn_begin(s_ble_export.conn_id);
        if (!app_ble_export_fast_conn_ready_or_timeout())
        {
            APP_BLE_EXPORT_P0_POLL_RETURN();
        }
#endif
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        app_ble_export_p0_reset("prepare");
        app_ble_export_p0_poll_enter(&p0_poll_tracked,
                                     &p0_notify_ok_before);
        app_ble_export_p0_set_prepare_start(p0_prepare_start_us);
        app_ble_export_p0_set_session(s_ble_export.conn_id,
                                      0U,
                                      0U,
                                      0U,
                                      (uint16_t)s_ble_export.current_export_total);
        app_ble_export_p0_note_prepare_done();
        app_ble_export_p0_note_begin_start(&s_ble_export.producer);
#endif
        if (!zy100_feuf_export_producer_begin(
                &s_ble_export.producer,
                (uint32_t)imu_fifo_drain_test_final_edge_stop_reason()))
        {
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
            app_ble_export_p0_note_begin_done(&s_ble_export.producer);
#endif
            app_ble_export_fail_keep_flash(
                zy100_feuf_export_producer_error(&s_ble_export.producer));
            APP_BLE_EXPORT_P0_POLL_RETURN();
        }
        zy100_feuf_export_producer_set_prefetch_buffer(
            &s_ble_export.producer,
            s_ble_export.prefetch_cache,
            (uint32_t)sizeof(s_ble_export.prefetch_cache));
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        app_ble_export_p0_note_begin_done(&s_ble_export.producer);
#endif
        s_ble_export.export_id =
            zy100_feuf_export_producer_export_id(&s_ble_export.producer);
        if (s_ble_export.export_id == 0U)
        {
            s_ble_export.export_id = (uint32_t)zy100_os_time_ms();
        }
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        app_ble_export_p0_set_session(s_ble_export.conn_id,
                                      s_ble_export.export_id,
                                      0U,
                                      0U,
                                      (uint16_t)s_ble_export.current_export_total);
#endif
#endif
        s_ble_export.chunk_seq = 1U;
        s_ble_export.chunk_count = 0U;
        s_ble_export.total_bytes = 0U;
        s_ble_export.crc_state = zy100_crc32_ieee_begin();
        s_ble_export.end_sent = false;
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        app_ble_export_p0_note_export_start_os();
#endif
        ZY100_LOG_EVENT("[EVT][UPLOAD] start id=%lu sessions=%lu",
                        (unsigned long)s_ble_export.export_id,
                        (unsigned long)s_ble_export.current_export_total);
#else
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        app_ble_export_p0_note_export_start_os();
#endif
        ZY100_LOG_EVENT("[EVT][UPLOAD] start id=%lu sessions=%u",
                        (unsigned long)s_ble_export.export_id,
                        app_ble_export_port_pending_session_count());
#endif
        app_ble_export_prepare_start_payload();
        s_ble_export.state = APP_BLE_EXPORT_START_FRAME;
        /* fall through */
    case APP_BLE_EXPORT_START_FRAME:
        {
            app_ble_export_ctrl_result_t ctrl_result =
                app_ble_export_pump_ctrl();

            if (ctrl_result == APP_BLE_EXPORT_CTRL_FATAL)
            {
                APP_BLE_EXPORT_P0_POLL_RETURN();
            }
            if (ctrl_result != APP_BLE_EXPORT_CTRL_DONE)
            {
                APP_BLE_EXPORT_P0_POLL_RETURN();
            }
        }
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        app_ble_export_p0_note_stream_start(&s_ble_export.producer);
#endif
        s_ble_export.state = APP_BLE_EXPORT_STREAMING;
        APP_BLE_EXPORT_P0_POLL_RETURN();

    case APP_BLE_EXPORT_STREAMING:
        memset(&progress, 0, sizeof(progress));
        if (zy100_feuf_export_producer_done(&s_ble_export.producer))
        {
            app_ble_export_log_progress(NULL, true);
            app_ble_export_prepare_end_payload();
            s_ble_export.state = APP_BLE_EXPORT_END_FRAME;
            APP_BLE_EXPORT_P0_POLL_RETURN();
        }
#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
        {
            uint8_t tx_submit_reason = APP_BLE_EXPORT_TX_SUBMIT_OK;

            if (!app_ble_export_tx_can_submit_reason(&tx_submit_reason))
            {
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
                app_ble_export_p0_note_tx_block_reason(tx_submit_reason);
#endif
                APP_BLE_EXPORT_P0_POLL_RETURN();
            }
        }
#endif
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        {
            uint64_t p0_peek_start_us = app_ble_export_p0_now_us();
            bool p0_peek_ok;

#if ZY100_BLE_EXPORT_AGGREGATE_PAYLOAD_ENABLE
            p0_peek_ok = zy100_feuf_export_producer_build_aggregate_plan(
                &s_ble_export.producer,
                &s_ble_export.agg_plan.after,
                s_ble_export.pending_payload,
                s_ble_export.chunk_payload,
                (uint16_t)ZY100_BLE_EXPORT_AGGREGATE_TARGET_BYTES,
                (uint8_t)ZY100_BLE_EXPORT_AGGREGATE_MAX_PIECES,
                &s_ble_export.agg_plan);
            if (p0_peek_ok)
            {
                payload_len = s_ble_export.agg_plan.len;
                aggregate_piece_count = s_ble_export.agg_plan.pieces;
                progress = s_ble_export.agg_plan.last_progress;
            }
#else
            aggregate_piece_count = 1U;
            p0_peek_ok = zy100_feuf_export_producer_peek(
                &s_ble_export.producer,
                s_ble_export.pending_payload,
                s_ble_export.chunk_payload,
                &payload_len,
                &progress);
#endif
            app_ble_export_p0_note_peek(
                (uint32_t)(app_ble_export_p0_now_us() - p0_peek_start_us),
                p0_peek_ok,
                zy100_feuf_export_producer_error(&s_ble_export.producer),
                s_ble_export.chunk_seq,
                payload_len);
            if ((!p0_peek_ok) || (payload_len == 0U))
            {
                app_ble_export_fail_keep_flash(
                    zy100_feuf_export_producer_error(&s_ble_export.producer));
                APP_BLE_EXPORT_P0_POLL_RETURN();
            }
        }
#else
#if ZY100_BLE_EXPORT_AGGREGATE_PAYLOAD_ENABLE
        if (!zy100_feuf_export_producer_build_aggregate_plan(
                &s_ble_export.producer,
                &s_ble_export.agg_plan.after,
                s_ble_export.pending_payload,
                s_ble_export.chunk_payload,
                (uint16_t)ZY100_BLE_EXPORT_AGGREGATE_TARGET_BYTES,
                (uint8_t)ZY100_BLE_EXPORT_AGGREGATE_MAX_PIECES,
                &s_ble_export.agg_plan) ||
            (s_ble_export.agg_plan.len == 0U))
#else
        if (!zy100_feuf_export_producer_peek(&s_ble_export.producer,
                                             s_ble_export.pending_payload,
                                             s_ble_export.chunk_payload,
                                             &payload_len,
                                             &progress) ||
            (payload_len == 0U))
#endif
        {
            app_ble_export_fail_keep_flash(
                zy100_feuf_export_producer_error(&s_ble_export.producer));
            APP_BLE_EXPORT_P0_POLL_RETURN();
        }
#if ZY100_BLE_EXPORT_AGGREGATE_PAYLOAD_ENABLE
        payload_len = s_ble_export.agg_plan.len;
        aggregate_piece_count = s_ble_export.agg_plan.pieces;
        progress = s_ble_export.agg_plan.last_progress;
#endif
#endif
#if ZY100_BLE_EXPORT_AGGREGATE_PAYLOAD_ENABLE && !ZY100_BLE_EXPORT_P0_PERF_ENABLE
        (void)aggregate_piece_count;
#endif
        {
            app_ble_export_send_result_t send_result =
                app_ble_export_send_frame(APP_BLE_EXPORT_FRAME_DATA,
                                          0U,
                                          s_ble_export.chunk_seq,
                                          s_ble_export.pending_payload,
                                          payload_len);

            if (send_result != APP_BLE_EXPORT_SEND_OK)
            {
#if ZY100_BLE_EXPORT_AGGREGATE_PAYLOAD_ENABLE
                if ((send_result == APP_BLE_EXPORT_SEND_BLOCKED) ||
                    (send_result == APP_BLE_EXPORT_SEND_LOCAL_BLOCKED))
                {
                    zy100_feuf_export_producer_adopt_prefetch_cache(
                        &s_ble_export.producer,
                        &s_ble_export.agg_plan.after);
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
                    app_ble_export_p0_note_aggregate_discard(
                        &s_ble_export.agg_plan);
#endif
                }
#endif
                APP_BLE_EXPORT_P0_POLL_RETURN();
            }
        }
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        app_ble_export_p0_note_data_payload_success(payload_len,
                                                    aggregate_piece_count);
#endif
        s_ble_export.crc_state =
            zy100_crc32_ieee_update(s_ble_export.crc_state,
                                    s_ble_export.pending_payload,
                                    payload_len);
        s_ble_export.total_bytes += payload_len;
        s_ble_export.chunk_count++;
#if ZY100_LOG_EXPORT_PUMP_VERBOSE && ZY100_BLE_EXPORT_MID_LOG_ENABLE
        ZY100_DIAG_LOG("[BLE_EXPORT] pump state=%u section=%u offset=%lu chunk=%u total_sent=%lu",
                   (uint32_t)s_ble_export.state,
                   progress.section_id,
                   (unsigned long)progress.stream_offset,
                   payload_len,
                   (unsigned long)s_ble_export.total_bytes);
#endif
        app_ble_export_log_progress(&progress, false);
#if ZY100_BLE_EXPORT_AGGREGATE_PAYLOAD_ENABLE
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        {
            uint64_t p0_copy_start_us = app_ble_export_p0_now_us();

            s_ble_export.producer = s_ble_export.agg_plan.after;
            app_ble_export_p0_note_producer_state_copy(
                (uint32_t)sizeof(s_ble_export.producer),
                (uint32_t)(app_ble_export_p0_now_us() -
                           p0_copy_start_us));
        }
#else
        s_ble_export.producer = s_ble_export.agg_plan.after;
#endif
#else
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        {
            uint64_t p0_commit_start_us = app_ble_export_p0_now_us();
            bool p0_commit_ok;

            p0_commit_ok =
                zy100_feuf_export_producer_commit(&s_ble_export.producer,
                                                  payload_len);
            app_ble_export_p0_note_commit(
                (uint32_t)(app_ble_export_p0_now_us() -
                           p0_commit_start_us),
                p0_commit_ok,
                zy100_feuf_export_producer_error(&s_ble_export.producer),
                s_ble_export.chunk_seq,
                payload_len);
            if (!p0_commit_ok)
            {
                app_ble_export_fail_keep_flash(
                    zy100_feuf_export_producer_error(&s_ble_export.producer));
                APP_BLE_EXPORT_P0_POLL_RETURN();
            }
        }
#else
        if (!zy100_feuf_export_producer_commit(&s_ble_export.producer,
                                               payload_len))
        {
            app_ble_export_fail_keep_flash(
                zy100_feuf_export_producer_error(&s_ble_export.producer));
            APP_BLE_EXPORT_P0_POLL_RETURN();
        }
#endif
#endif
        s_ble_export.chunk_seq++;
        APP_BLE_EXPORT_P0_POLL_RETURN();

    case APP_BLE_EXPORT_END_FRAME:
        {
            app_ble_export_ctrl_result_t ctrl_result =
                app_ble_export_pump_ctrl();

            if (ctrl_result == APP_BLE_EXPORT_CTRL_FATAL)
            {
                APP_BLE_EXPORT_P0_POLL_RETURN();
            }
            if (ctrl_result != APP_BLE_EXPORT_CTRL_DONE)
            {
                APP_BLE_EXPORT_P0_POLL_RETURN();
            }
        }
        s_ble_export.end_sent = true;
        s_ble_export.state = APP_BLE_EXPORT_WAIT_CONFIRM;
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        app_ble_export_p0_note_wait_confirm();
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
        app_ble_export_ui_begin_upload("ble_export_processing");
#else
        app_ble_export_ui_begin_upload("export_wait_confirm");
#endif
        DBG_DIRECT("[BLE_EXPORT] end export_id=%lu",
                   (unsigned long)s_ble_export.export_id);
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
        if (ZY100_BLE_EXPORT_DETAIL_LOG_ENABLE)
        {
            DBG_DIRECT("[BLE_EXPORT] tail_session_done uid=%lu bytes=%lu crc=0x%08lX",
                       (unsigned long)s_ble_export.current_export_session_uid,
                       (unsigned long)s_ble_export.total_bytes,
                       (unsigned long)s_ble_export.stream_crc32);
            DBG_DIRECT("[BLE_EXPORT] waiting_auto_confirm uid=%lu",
                       (unsigned long)s_ble_export.current_export_session_uid);
        }
#endif
        APP_BLE_EXPORT_P0_POLL_RETURN();

    case APP_BLE_EXPORT_CLEAR_AFTER_CONFIRM:
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
        prep_status = zy100_session_range_table_reclaim_tail_poll();
        if (prep_status == ZY100_FLASH_PREP_BUSY)
        {
            app_ble_export_ui_begin_upload("ble_export_processing");
            APP_BLE_EXPORT_P0_POLL_RETURN();
        }
        if (prep_status != ZY100_FLASH_PREP_DONE)
        {
            app_ble_export_clear_failed((uint32_t)prep_status);
            APP_BLE_EXPORT_P0_POLL_RETURN();
        }
        {
            uint32_t reclaimed_session_uid = s_ble_export.current_export_session_uid;
            uint8_t final_device_state = ZY100_BLE_DEVICE_STATE_WAIT_START;

            s_ble_export.batch_exported_count++;
            DBG_DIRECT("[SESSION_RECLAIM] done uid=%lu",
                       (unsigned long)reclaimed_session_uid);
            app_ble_export_refresh_storage("reclaim_done");
            s_ble_export.current_export_id = 0U;
            s_ble_export.current_export_session_uid = 0U;
            s_ble_export.current_end_seq = 0U;
            s_ble_export.current_session_crc = 0U;
            s_ble_export.end_sent = false;
            app_ble_export_progress_reset();
            if (s_ble_export_storage_state.pending_count != 0U)
            {
                if (app_ble_export_conn_ready(s_ble_export.conn_id))
                {
                    s_ble_export.state = APP_BLE_EXPORT_PREPARE;
                    final_device_state = ZY100_BLE_DEVICE_STATE_BLE_EXPORTING;
                    app_ble_export_clear_notify_final(ZY100_BLE_ACK_STATUS_OK,
                                                      final_device_state,
                                                      reclaimed_session_uid);
                    app_ble_export_ui_begin_upload("ble_export_streaming");
                    app_ble_export_port_post_ctrl_event();
                    APP_BLE_EXPORT_P0_POLL_RETURN();
                }
                if (app_ble_power_is_connected())
                {
                    DBG_DIRECT("[BLE_EXPORT] wait_ready reason=notify_not_enabled pending=%lu",
                               (unsigned long)s_ble_export_storage_state.pending_count);
                    app_ble_export_port_set_in_progress(false);
                    s_ble_export.state = APP_BLE_EXPORT_WAIT_SUBSCRIBE;
                    final_device_state = app_ble_export_port_device_state();
                    app_ble_export_clear_notify_final(ZY100_BLE_ACK_STATUS_OK,
                                                      final_device_state,
                                                      reclaimed_session_uid);
                    app_ble_export_port_ui_restore_capture("ble_pending_export_not_ready");
                    APP_BLE_EXPORT_P0_POLL_RETURN();
                }
                DBG_DIRECT("[BLE_EXPORT] batch_interrupted reason=ble_disconnected pending=%lu",
                           (unsigned long)s_ble_export_storage_state.pending_count);
#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
                app_ble_export_fast_conn_end(false);
#endif
                app_ble_export_port_set_in_progress(false);
                s_ble_export.state = APP_BLE_EXPORT_FAILED;
                if (app_ble_export_port_flash_full_wait_gate_active())
                {
                    app_ble_export_port_ui_restore_capture("flash_full_wait_ble_upload");
                    APP_BLE_EXPORT_P0_POLL_RETURN();
                }
                app_ble_export_ui_begin_failure("ble_export_batch_disconnected");
                APP_BLE_EXPORT_P0_POLL_RETURN();
            }
            DBG_DIRECT("[BLE_EXPORT] end state=batch_done");
#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
            app_ble_export_fast_conn_end(true);
#endif
            app_ble_export_port_set_in_progress(false);
            s_ble_export.state = APP_BLE_EXPORT_DONE;
            imu_fifo_drain_test_final_edge_set_export_done();
            if (s_ble_export_storage_state.can_start_next)
            {
                final_device_state = ZY100_BLE_DEVICE_STATE_WAIT_START;
            }
            else
            {
                final_device_state = app_ble_export_port_device_state();
            }
            app_ble_export_clear_notify_final(ZY100_BLE_ACK_STATUS_OK,
                                              final_device_state,
                                              reclaimed_session_uid);
            app_ble_export_ui_begin_success("ble_export_done");
            APP_BLE_EXPORT_P0_POLL_RETURN();
        }
#else
        if (s_ble_export.clear_begin_requested)
        {
            s_ble_export.clear_begin_requested = false;
            if (!imu_fifo_drain_test_prepare_flash_begin())
            {
                app_ble_export_clear_failed(
                    (uint32_t)ZY100_FLASH_PREP_ERROR);
                APP_BLE_EXPORT_P0_POLL_RETURN();
            }
        }
        prep_status = imu_fifo_drain_test_prepare_flash_poll();
        if (prep_status == ZY100_FLASH_PREP_BUSY)
        {
            APP_BLE_EXPORT_P0_POLL_RETURN();
        }
        if (prep_status != ZY100_FLASH_PREP_DONE)
        {
            app_ble_export_clear_failed((uint32_t)prep_status);
            APP_BLE_EXPORT_P0_POLL_RETURN();
        }
        app_ble_export_clear_done();
        APP_BLE_EXPORT_P0_POLL_RETURN();
#endif

    default:
        APP_BLE_EXPORT_P0_POLL_RETURN();
    }
}

void app_ble_export_controller_shutdown_poll(void)
{
    (void)app_ble_export_controller_prepare_for_sleep_request();
    if (app_ble_export_reclaim_active())
    {
        app_ble_export_poll();
    }
    /* Completion can release resources, but cannot own shutdown visuals. */
    app_ble_export_ui_cancel("shutdown_drain");
}

bool app_ble_export_transport_busy(void)
{
#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
    return app_ble_export_state_streaming(s_ble_export.state) ||
           (s_ble_export.tx_in_flight != 0U);
#else
    return app_ble_export_state_streaming(s_ble_export.state);
#endif
}

bool app_ble_export_waiting_confirm(void)
{
    return s_ble_export.state == APP_BLE_EXPORT_WAIT_CONFIRM;
}

bool app_ble_export_reclaim_active(void)
{
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
    return s_ble_export_resume_reclaim_active ||
           (s_ble_export.state == APP_BLE_EXPORT_CLEAR_AFTER_CONFIRM) ||
           s_ble_export_storage_state.reclaim_active;
#else
    return s_ble_export.state == APP_BLE_EXPORT_CLEAR_AFTER_CONFIRM;
#endif
}

bool app_ble_export_clear_retry_required(void)
{
    return s_ble_export_clear_retry_required;
}

uint32_t app_ble_export_current_or_retry_id(void)
{
    return (s_ble_export.export_id != 0U) ? s_ble_export.export_id :
           s_ble_export_clear_failed_export_id;
}

bool app_ble_export_ui_failed(void)
{
    return s_ble_upload_led_state == APP_BLE_UPLOAD_LED_FAIL_BLINK;
}

bool app_ble_export_ui_active(void)
{
    return s_ble_upload_led_state != APP_BLE_UPLOAD_LED_IDLE;
}

void app_ble_export_clear_state_reset_after_finalize(void)
{
    s_ble_export_clear_retry_required = false;
    s_ble_export_clear_failed_export_id = 0U;
    s_ble_export.state = APP_BLE_EXPORT_DONE;
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    app_ble_export_p0_log_summary("terminated");
#endif
    memset(&s_ble_export, 0, sizeof(s_ble_export));
    s_ble_export.state = APP_BLE_EXPORT_IDLE;
}

void app_ble_export_controller_tx_ready(void)
{
#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
    s_ble_export.tx_ready_event_pending = false;
    app_ble_export_fast_pump();
#endif
}

#undef APP_BLE_EXPORT_P0_POLL_RETURN

#endif
