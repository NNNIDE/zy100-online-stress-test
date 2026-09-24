#include "zy100_online_stream.h"

#include <os_sync.h>
#include <os_mem.h>
#include <gap_conn_le.h>
#include <string.h>

#include "trace.h"

#include "../bsp/imu_bsp.h"
#include "../common/zy100_byteorder.h"
#include "../zy100_clock_config.h"
#include "zy100_ble_ctrl_protocol.h"
#include "zy100_online_reset_trace.h"
#include "zy100_online_spool.h"
#include "zy100_online_raw_capture.h"
#include "zy100_mode_workspace.h"
#if ZY100_ONLINE_STRESS_TEST_ENABLE
#include "zy100_online_stress.h"
#include "FreeRTOS_API.h"
#endif

#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_DIRECT_SPOOL_ENABLE

#define ZY100_ONLINE_V2_FRAME_FLAG_LAST 0x01U
#define ZY100_ONLINE_V2_RECORD_PAYLOAD_HEADER_BYTES 28U
#define ZY100_ONLINE_V2_U32_MAX 0xFFFFFFFFUL
#define ZY100_ONLINE_V2_NOTIFY_BYTES 244U
#define ZY100_ONLINE_V2_INVALID_SLOT 0xFFU

typedef char zy100_online_v3_end_layout_check[
    ((ZY100_ONLINE_END_BASE_BYTES +
      ZY100_ONLINE_END_FIXED40_META_BYTES_V3) ==
     ZY100_ONLINE_END_FIXED40_TOTAL_BYTES_V3) ? 1 : -1];

typedef enum
{
    ZY100_ONLINE_V2_TX_SLOT_FREE = 0U,
    ZY100_ONLINE_V2_TX_SLOT_BUILDING,
    ZY100_ONLINE_V2_TX_SLOT_READY,
    ZY100_ONLINE_V2_TX_SLOT_IN_FLIGHT,
    ZY100_ONLINE_V2_TX_SLOT_QUARANTINED,
} zy100_online_v2_tx_slot_state_t;

typedef enum
{
    ZY100_ONLINE_TX_STALL_APP_POLL = 1U,
    ZY100_ONLINE_TX_STALL_READY_NOT_SUBMITTED = 2U,
    ZY100_ONLINE_TX_STALL_NOTIFY_REJECTED = 3U,
    ZY100_ONLINE_TX_STALL_SEND_COMPLETE_MISSING = 4U,
    ZY100_ONLINE_TX_STALL_ACK_PROGRESS_MISSING = 5U,
    ZY100_ONLINE_TX_STALL_STATE_INCONSISTENT = 6U
} zy100_online_v2_tx_stall_class_t;

#define ZY100_ONLINE_V2_TX_STAT_INTERVAL_MS 5000U

typedef struct
{
    zy100_online_v2_tx_slot_state_t state;
    bool completion_pending;
    bool last_fragment;
    uint8_t conn_id;
    uint8_t frame_type;
    uint8_t record_type;
    uint16_t len;
    uint16_t fragment_len;
    uint16_t payload_len;
    uint32_t session_generation;
    uint32_t ready_order;
    uint32_t record_id;
    uint32_t record_bytes;
    uint32_t fragment_offset;
#if ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
    uint8_t *data;
#else
    uint8_t data[ZY100_ONLINE_V2_NOTIFY_BYTES];
#endif
} zy100_online_v2_tx_slot_t;

typedef struct
{
    uint32_t ms;
    uint32_t detail;
    uint16_t bscan_id;
    uint8_t type;
    uint8_t reason;
    uint8_t reserve_result;
    uint8_t fe_state;
    uint8_t owner;
    uint8_t level;
} zy100_online_v2_diag_event_t;

#define ZY100_ONLINE_V2_DIAG_DEPTH 32U
typedef char zy100_online_v2_diag_event_size_check[
    (sizeof(zy100_online_v2_diag_event_t) == 16U) ? 1 : -1];

typedef struct
{
    bool waiting;
    uint8_t type;
    uint32_t record_id;
    uint32_t bytes;
    uint32_t start_ms;
} zy100_online_v2_ack_wait_t;

typedef struct
{
    uint32_t last_stage_ms;
    uint32_t last_notify_ms;
    uint32_t last_send_complete_ms;
    uint32_t last_record_ack_ms;
    uint32_t last_poll_ms;
    uint32_t next_stat_ms;
    uint32_t poll_gap_max_ms;
    uint32_t notify_ok;
    uint32_t notify_fail;
    uint32_t send_complete;
    uint32_t send_complete_credits;
    uint32_t record_ack;
    uint8_t ready_max;
    uint8_t in_flight_max;
    bool stall_logged;
} zy100_online_v2_tx_diag_t;

typedef struct
{
    bool valid;
    uint8_t origin;
    uint32_t detail;
    uint32_t at_ms;
    uint32_t stop_reason;
    uint32_t session_generation;
    uint32_t last_error;
} zy100_online_v2_abort_cause_t;

typedef struct
{
    bool ready_latched;
    bool start_preparing;
    bool start_prepare_started;
    bool start_prepare_ready;
    bool active;
    bool start_sent;
    bool end_requested;
    bool end_wait_ack;
    bool end_ack_received;
    bool abort_requested;
    bool abort_cleanup_ready;
    bool abort_notify_accepted;
    bool link_disconnected;
    bool discard_tail_on_cleanup;
    bool capture_stop_requested;
    bool stop_complete_pending;
    bool stop_complete_timeout;
    bool summary_logged;
    bool summary_pending;
    bool error_log_pending;
    uint8_t conn_id;
    uint8_t start_seq;
    uint32_t capability_mask;
    uint32_t session_id;
    uint32_t user_id;
    uint32_t training_id;
    uint32_t stop_reason;
    uint32_t stop_complete_reason;
    uint32_t abort_start_ms;
    uint32_t last_tx_progress_ms;
    uint32_t session_generation;
    uint32_t ready_order_next;
    uint16_t chunk_seq;
    bool tx_active;
    zy100_online_spool_record_t tx;
    uint32_t tx_offset;
    uint32_t tx_stage_offset;
    zy100_online_v2_ack_wait_t ack_wait[ZY100_ONLINE_RECORD_IN_FLIGHT_MAX];
    uint8_t ack_wait_head;
    uint8_t ack_wait_count;
    uint32_t gate[ZY100_ONLINE_GATE_COUNT];
    uint32_t notify_attempts;
    uint32_t notify_ok;
    uint32_t notify_fail;
    uint32_t notify_bytes;
    uint8_t tx_in_flight;
    uint8_t tx_window_limit;
    uint8_t tx_in_flight_max;
    uint32_t send_complete_count;
    uint32_t send_complete_fail;
    uint32_t send_complete_credits;
    uint32_t stage_call;
    uint32_t stage_ok;
    uint32_t stage_gate;
    uint32_t stage_no_slot;
    uint32_t stage_source_wait;
    uint32_t stage_flash_busy;
    uint32_t send_call;
    uint32_t send_ready;
    uint32_t send_window;
    uint32_t send_no_ready;
    uint8_t tx_selected_slot;
    uint8_t tx_complete_head;
    uint8_t tx_complete_tail;
    uint8_t tx_complete_count;
#if ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
    uint32_t workspace_token;
    bool tx_view_pending;
#endif
    uint8_t tx_complete_slots[ZY100_ONLINE_V2_TX_WINDOW_SLOTS];
    zy100_online_v2_tx_slot_t tx_ring[ZY100_ONLINE_V2_TX_WINDOW_SLOTS];
    uint32_t ack_timeout_count;
    uint32_t ack_latency_min_ms;
    uint32_t ack_latency_max_ms;
    uint32_t ack_latency_sum_ms;
    uint32_t ack_latency_count;
    uint32_t blind_count;
    uint32_t blind_total_ms;
    uint32_t blind_max_ms;
    uint32_t session_start_ms;
    uint32_t capture_segment_id;
    uint8_t summary_stage;
    uint32_t last_error;
    uint16_t mtu;
    uint16_t conn_interval;
    uint8_t phy_tx;
    uint8_t phy_rx;
    uint8_t diag_next;
    uint8_t diag_count;
    uint32_t dropped[ZY100_ONLINE_RECORD_TYPE_COUNT];
    uint32_t issue_count;
    uint32_t raw_accepted;
    uint32_t raw_saved;
    uint32_t raw_last_accepted_bscan;
    uint32_t raw_last_saved_bscan;
    zy100_online_clock_meta_t clock_meta;
    zy100_online_v2_diag_event_t diag[ZY100_ONLINE_V2_DIAG_DEPTH];
    zy100_online_v2_tx_diag_t tx_diag;
    zy100_online_v2_abort_cause_t abort_cause;
} zy100_online_v2_state_t;

static zy100_online_v2_state_t s_online_v2;

/* App-task owned; no per-packet logging or kernel scheduler instrumentation.
 * Accepted ATT value bytes include our framing, NOT proof of peer receipt. */
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
typedef struct
{
    uint64_t accepted_bytes;
    uint32_t window_bytes;
    uint32_t start_ms;
    uint32_t last_poll_ms;
    uint32_t window_ms;
    uint32_t elapsed_ms;
    uint32_t peak_Bps;
    uint32_t peak_window_ms;
    uint32_t poll_gap_ms;
    uint32_t heap_data_min;
    uint32_t heap_buffer_min;
    uint32_t samples;
    uint32_t fail_start;
    uint32_t ack_timeout_start;
    uint32_t ack_max_ms;
    uint32_t pending_hi;
    uint32_t pending_end;
    uint32_t capacity_blocked;
    uint32_t erase_max_ms;
    uint32_t erased_min;
    uint32_t read_errors;
    uint32_t write_errors;
    uint32_t erase_errors;
} online_v2_resource_t;
static online_v2_resource_t s_online_resource;

static uint32_t online_v2_resource_rate(uint64_t bytes, uint32_t ms)
{
    uint64_t rate = (ms != 0U) ? (bytes * 1000U / ms) : 0U;
    return (rate > 0xFFFFFFFFUL) ? 0xFFFFFFFFUL : (uint32_t)rate;
}

static void online_v2_resource_sample(void)
{
    uint32_t data_free = (uint32_t)os_mem_peek(RAM_TYPE_DATA_ON);
    uint32_t buffer_free = (uint32_t)os_mem_peek(RAM_TYPE_BUFFER_ON);
    if ((s_online_resource.samples == 0U) ||
        (data_free < s_online_resource.heap_data_min))
    {
        s_online_resource.heap_data_min = data_free;
    }
    if ((s_online_resource.samples == 0U) ||
        (buffer_free < s_online_resource.heap_buffer_min))
    {
        s_online_resource.heap_buffer_min = buffer_free;
    }
    s_online_resource.samples++;
#if ZY100_ONLINE_STRESS_TEST_ENABLE && (INCLUDE_uxTaskGetStackHighWaterMark == 1)
    zy100_online_stress_note_stacks(
        (uint32_t)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t),
        0xFFFFFFFFUL, 0xFFFFFFFFUL);
#endif
#if F_BT_LE_5_0_SET_PHYS_SUPPORT
    (void)le_get_conn_param(GAP_PARAM_CONN_TX_PHY_TYPE,
                            &s_online_v2.phy_tx, s_online_v2.conn_id);
    (void)le_get_conn_param(GAP_PARAM_CONN_RX_PHY_TYPE,
                            &s_online_v2.phy_rx, s_online_v2.conn_id);
#endif
}

static void online_v2_resource_begin(uint32_t now_ms)
{
    memset(&s_online_resource, 0, sizeof(s_online_resource));
    s_online_resource.fail_start = s_online_v2.notify_fail;
    s_online_resource.ack_timeout_start = s_online_v2.ack_timeout_count;
    s_online_resource.start_ms = now_ms;
    s_online_resource.last_poll_ms = now_ms;
    s_online_resource.window_ms = now_ms;
    s_online_v2.phy_tx = 0U;
    s_online_v2.phy_rx = 0U;
    online_v2_resource_sample();
}

static void online_v2_resource_poll(uint32_t now_ms)
{
    uint32_t gap = now_ms - s_online_resource.last_poll_ms;
    uint32_t window = now_ms - s_online_resource.window_ms;
    s_online_resource.elapsed_ms = now_ms - s_online_resource.start_ms;
    s_online_resource.last_poll_ms = now_ms;
    if (gap > s_online_resource.poll_gap_ms)
    {
        s_online_resource.poll_gap_ms = gap;
    }
    if (window >= 1000U)
    {
        uint32_t rate = online_v2_resource_rate(s_online_resource.window_bytes, window);
        if (rate > s_online_resource.peak_Bps)
        {
            s_online_resource.peak_Bps = rate;
            s_online_resource.peak_window_ms = window;
        }
        s_online_resource.window_bytes = 0U;
        s_online_resource.window_ms = now_ms;
        online_v2_resource_sample();
    }
}


/* Freeze before cleanup mutates pending bytes or performs more erases. */
static void online_v2_resource_finish(void)
{
    zy100_online_spool_stats_t stats;
    if (!s_online_v2.active) { return; }
    s_online_resource.elapsed_ms = zy100_os_time_ms() - s_online_resource.start_ms;
    zy100_online_spool_get_stats(&stats);
    s_online_resource.pending_hi = stats.pending_high_water;
    s_online_resource.pending_end = stats.pending_bytes;
    s_online_resource.capacity_blocked = stats.capacity_blocked;
    s_online_resource.erase_max_ms = stats.erase_max_ms;
    s_online_resource.erased_min = stats.erased_ahead_min_bytes;
    s_online_resource.read_errors = stats.read_errors;
    s_online_resource.write_errors = stats.write_errors;
    s_online_resource.erase_errors = stats.erase_errors;
    s_online_resource.fail_start = s_online_v2.notify_fail - s_online_resource.fail_start;
    s_online_resource.ack_timeout_start = s_online_v2.ack_timeout_count - s_online_resource.ack_timeout_start;
}
#endif

static void online_v2_latch_abort_cause(
    zy100_online_stop_reason_t reason,
    zy100_online_abort_origin_t origin,
    uint32_t detail,
    uint32_t now_ms)
{
    uint32_t lock_state = os_lock();

    if (s_online_v2.active && !s_online_v2.abort_cause.valid)
    {
        s_online_v2.abort_cause.valid = true;
#if ZY100_ONLINE_STRESS_TEST_ENABLE
        zy100_online_stress_failure((uint32_t)reason);
#endif
        s_online_v2.abort_cause.origin = (uint8_t)origin;
        s_online_v2.abort_cause.detail = detail;
        s_online_v2.abort_cause.at_ms = now_ms;
        s_online_v2.abort_cause.stop_reason = (uint32_t)reason;
        s_online_v2.abort_cause.session_generation =
            s_online_v2.session_generation;
        s_online_v2.abort_cause.last_error = s_online_v2.last_error;
    }
    os_unlock(lock_state);
}

static void online_v2_build_outer(uint8_t type,
                                  uint8_t flags,
                                  uint16_t sequence,
                                  uint8_t *frame,
                                  uint16_t payload_len)
{
    frame[0] = ZY100_ONLINE_STREAM_EXPORT_MAGIC;
    frame[1] = ZY100_ONLINE_STREAM_EXPORT_VERSION;
    frame[2] = type;
    frame[3] = flags;
    zy100_put_u16_le(&frame[4], sequence);
    zy100_put_u16_le(&frame[6], payload_len);
}

static uint16_t online_v2_take_sequence(void)
{
    uint16_t sequence;
    uint32_t lock_state = os_lock();

    sequence = s_online_v2.chunk_seq++;
    os_unlock(lock_state);
    return sequence;
}

static void online_v2_tx_slot_reset_locked(zy100_online_v2_tx_slot_t *slot)
{
    if (slot == NULL)
    {
        return;
    }
    slot->state = ZY100_ONLINE_V2_TX_SLOT_FREE;
    slot->completion_pending = false;
    slot->last_fragment = false;
    slot->conn_id = 0xFFU;
    slot->frame_type = 0U;
    slot->record_type = 0U;
    slot->len = 0U;
    slot->fragment_len = 0U;
    slot->payload_len = 0U;
    slot->session_generation = 0U;
    slot->ready_order = 0U;
    slot->record_id = 0U;
    slot->record_bytes = 0U;
    slot->fragment_offset = 0U;
}

static bool online_v2_tx_reuse_safe_locked(void)
{
    uint8_t i;

    if (
#if ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
        s_online_v2.tx_view_pending ||
#endif
        (s_online_v2.tx_selected_slot != ZY100_ONLINE_V2_INVALID_SLOT) ||
        (s_online_v2.tx_complete_count != 0U) ||
        (s_online_v2.tx_in_flight != 0U))
    {
        return false;
    }
    for (i = 0U; i < ZY100_ONLINE_V2_TX_WINDOW_SLOTS; i++)
    {
        if (s_online_v2.tx_ring[i].state != ZY100_ONLINE_V2_TX_SLOT_FREE)
        {
            return false;
        }
    }
    return true;
}

static bool online_v2_tx_reuse_safe(void)
{
    bool safe;
    uint32_t lock_state = os_lock();

    safe = online_v2_tx_reuse_safe_locked();
    os_unlock(lock_state);
    return safe;
}

static uint8_t online_v2_tx_find_free_locked(void)
{
    uint8_t i;

    for (i = 0U; i < ZY100_ONLINE_V2_TX_WINDOW_SLOTS; i++)
    {
        if (s_online_v2.tx_ring[i].state == ZY100_ONLINE_V2_TX_SLOT_FREE)
        {
            return i;
        }
    }
    return ZY100_ONLINE_V2_INVALID_SLOT;
}

static uint8_t online_v2_tx_find_ready_locked(void)
{
    uint8_t i;
    uint8_t found = ZY100_ONLINE_V2_INVALID_SLOT;
    uint32_t found_order = ZY100_ONLINE_V2_U32_MAX;

    for (i = 0U; i < ZY100_ONLINE_V2_TX_WINDOW_SLOTS; i++)
    {
        const zy100_online_v2_tx_slot_t *slot = &s_online_v2.tx_ring[i];

        if ((slot->state == ZY100_ONLINE_V2_TX_SLOT_READY) &&
            (slot->ready_order < found_order))
        {
            found = i;
            found_order = slot->ready_order;
        }
    }
    return found;
}

static uint8_t online_v2_tx_claim_slot(uint8_t frame_type,
                                        uint8_t record_type,
                                        uint32_t record_id,
                                        uint32_t record_bytes,
                                        bool last_fragment,
                                        uint16_t fragment_len,
                                        uint16_t payload_len)
{
    uint8_t slot_index;
    zy100_online_v2_tx_slot_t *slot;
    uint32_t lock_state = os_lock();

    slot_index = online_v2_tx_find_free_locked();
    if (slot_index == ZY100_ONLINE_V2_INVALID_SLOT)
    {
        os_unlock(lock_state);
        return slot_index;
    }
    slot = &s_online_v2.tx_ring[slot_index];
    online_v2_tx_slot_reset_locked(slot);
    slot->conn_id = s_online_v2.conn_id;
    slot->frame_type = frame_type;
    slot->record_type = record_type;
    slot->record_id = record_id;
    slot->record_bytes = record_bytes;
    slot->last_fragment = last_fragment;
    slot->fragment_len = fragment_len;
    slot->payload_len = payload_len;
    slot->session_generation = s_online_v2.session_generation;
    slot->state = ZY100_ONLINE_V2_TX_SLOT_BUILDING;
    os_unlock(lock_state);
    return slot_index;
}

static bool online_v2_tx_publish_slot(uint8_t slot_index, uint16_t frame_len)
{
    bool published = false;
    zy100_online_v2_tx_slot_t *slot;
    uint32_t lock_state;

    if (slot_index >= ZY100_ONLINE_V2_TX_WINDOW_SLOTS)
    {
        return false;
    }
    lock_state = os_lock();
    slot = &s_online_v2.tx_ring[slot_index];
    if ((slot->state == ZY100_ONLINE_V2_TX_SLOT_BUILDING) &&
        s_online_v2.active &&
        (slot->session_generation == s_online_v2.session_generation) &&
        (slot->conn_id == s_online_v2.conn_id))
    {
        slot->len = frame_len;
        s_online_v2.ready_order_next++;
        if (s_online_v2.ready_order_next == 0U)
        {
            s_online_v2.ready_order_next++;
        }
        slot->ready_order = s_online_v2.ready_order_next;
        slot->state = ZY100_ONLINE_V2_TX_SLOT_READY;
        published = true;
    }
    else if ((slot->state == ZY100_ONLINE_V2_TX_SLOT_BUILDING) ||
             ((slot->state == ZY100_ONLINE_V2_TX_SLOT_QUARANTINED) &&
              !slot->completion_pending &&
              (s_online_v2.tx_selected_slot != slot_index)))
    {
        online_v2_tx_slot_reset_locked(slot);
    }
    os_unlock(lock_state);
    return published;
}

static void online_v2_tx_release_builder(uint8_t slot_index)
{
    zy100_online_v2_tx_slot_t *slot;
    uint32_t lock_state;

    if (slot_index >= ZY100_ONLINE_V2_TX_WINDOW_SLOTS)
    {
        return;
    }
    lock_state = os_lock();
    slot = &s_online_v2.tx_ring[slot_index];
    if (((slot->state == ZY100_ONLINE_V2_TX_SLOT_BUILDING) ||
         (slot->state == ZY100_ONLINE_V2_TX_SLOT_QUARANTINED)) &&
        !slot->completion_pending &&
        (s_online_v2.tx_selected_slot != slot_index))
    {
        online_v2_tx_slot_reset_locked(slot);
    }
    os_unlock(lock_state);
}

#if !ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
static bool online_v2_stage_frame(const uint8_t *frame,
                                  uint16_t frame_len,
                                  uint8_t frame_type,
                                  uint8_t record_type,
                                  uint16_t payload_len)
{
    uint8_t slot_index;
    zy100_online_v2_tx_slot_t *slot;

    if ((frame == NULL) || (frame_len == 0U) ||
        (frame_len > ZY100_ONLINE_V2_NOTIFY_BYTES))
    {
        return false;
    }
    slot_index = online_v2_tx_claim_slot(frame_type,
                                         record_type,
                                         0U,
                                         0U,
                                         true,
                                         0U,
                                         payload_len);
    if (slot_index == ZY100_ONLINE_V2_INVALID_SLOT)
    {
        return false;
    }
    slot = &s_online_v2.tx_ring[slot_index];
    memcpy(slot->data, frame, frame_len);
    return online_v2_tx_publish_slot(slot_index, frame_len);
}

static bool online_v2_tx_select_ready(uint16_t notify_max,
                                      uint8_t *frame,
                                      uint16_t *frame_len)
{
    uint8_t slot_index;
    const zy100_online_v2_tx_slot_t *slot;
    bool selected = false;
    uint32_t lock_state = os_lock();

    if (s_online_v2.tx_selected_slot == ZY100_ONLINE_V2_INVALID_SLOT)
    {
        slot_index = online_v2_tx_find_ready_locked();
        if (slot_index != ZY100_ONLINE_V2_INVALID_SLOT)
        {
            slot = &s_online_v2.tx_ring[slot_index];
            if ((slot->len != 0U) && (slot->len <= notify_max))
            {
                memcpy(frame, slot->data, slot->len);
                *frame_len = slot->len;
                s_online_v2.tx_selected_slot = slot_index;
                selected = true;
            }
        }
    }
    os_unlock(lock_state);
    return selected;
}

#else
static bool online_v2_tx_select_view(uint16_t notify_max,
                                     zy100_online_notify_view_t *view)
{
    uint8_t index;
    bool selected = false;
    uint32_t lock_state = os_lock();
    if (!s_online_v2.tx_view_pending &&
        (s_online_v2.tx_selected_slot == ZY100_ONLINE_V2_INVALID_SLOT))
    {
        index = online_v2_tx_find_ready_locked();
        if (index != ZY100_ONLINE_V2_INVALID_SLOT)
        {
            const zy100_online_v2_tx_slot_t *slot = &s_online_v2.tx_ring[index];
            if ((slot->data != NULL) && (slot->len != 0U) && (slot->len <= notify_max))
            {
                view->data = slot->data;
                view->len = slot->len;
                view->frame_type = slot->frame_type;
                view->payload_len = slot->payload_len;
                s_online_v2.tx_selected_slot = index;
                s_online_v2.tx_view_pending = true;
                selected = true;
            }
        }
    }
    os_unlock(lock_state);
    return selected;
}

static void online_v2_release_workspace(void)
{
    uint32_t token;
    uint8_t i;
    uint32_t lock_state = os_lock();
    token = s_online_v2.workspace_token;
    if ((token != 0U) && online_v2_tx_reuse_safe_locked())
    {
        s_online_v2.workspace_token = 0U;
        for (i = 0U; i < ZY100_ONLINE_V2_TX_WINDOW_SLOTS; i++)
        {
            s_online_v2.tx_ring[i].data = NULL;
        }
        (void)zy100_mode_workspace_online_release(token, ZY100_ONLINE_WORKSPACE_TX);
    }
    os_unlock(lock_state);
}

typedef char online_v2_workspace_tx_fit_check[
    ((ZY100_ONLINE_V2_TX_WINDOW_SLOTS <= ZY100_ONLINE_WORKSPACE_TX_SLOTS) &&
     (ZY100_ONLINE_V2_NOTIFY_BYTES == ZY100_ONLINE_WORKSPACE_TX_BYTES)) ? 1 : -1];
#endif

static void online_v2_tx_cancel_unsent(void)
{
    uint8_t i;
    uint32_t lock_state = os_lock();

    for (i = 0U; i < ZY100_ONLINE_V2_TX_WINDOW_SLOTS; i++)
    {
        zy100_online_v2_tx_slot_t *slot = &s_online_v2.tx_ring[i];

        switch (slot->state)
        {
        case ZY100_ONLINE_V2_TX_SLOT_BUILDING:
        case ZY100_ONLINE_V2_TX_SLOT_IN_FLIGHT:
            slot->state = ZY100_ONLINE_V2_TX_SLOT_QUARANTINED;
            break;
        case ZY100_ONLINE_V2_TX_SLOT_READY:
            if (s_online_v2.tx_selected_slot == i)
            {
                slot->state = ZY100_ONLINE_V2_TX_SLOT_QUARANTINED;
            }
            else
            {
                online_v2_tx_slot_reset_locked(slot);
            }
            break;
        default:
            break;
        }
    }
    os_unlock(lock_state);
}

static void online_v2_tx_drop_unsent_terminal(void)
{
    uint8_t i;
    uint32_t lock_state = os_lock();

    for (i = 0U; i < ZY100_ONLINE_V2_TX_WINDOW_SLOTS; i++)
    {
        zy100_online_v2_tx_slot_t *slot = &s_online_v2.tx_ring[i];

        if ((slot->state == ZY100_ONLINE_V2_TX_SLOT_READY) &&
            (slot->frame_type == ZY100_BLE_ONLINE_FRAME_ABORT) &&
            (s_online_v2.tx_selected_slot != i))
        {
            online_v2_tx_slot_reset_locked(slot);
        }
    }
    os_unlock(lock_state);
}

static void online_v2_tx_release_disconnected(uint8_t conn_id)
{
    uint8_t i;
    uint32_t lock_state = os_lock();

    for (i = 0U; i < ZY100_ONLINE_V2_TX_WINDOW_SLOTS; i++)
    {
        zy100_online_v2_tx_slot_t *slot = &s_online_v2.tx_ring[i];

        if (slot->conn_id != conn_id)
        {
            continue;
        }
#if ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
        if ((slot->state == ZY100_ONLINE_V2_TX_SLOT_BUILDING) ||
            ((slot->state == ZY100_ONLINE_V2_TX_SLOT_QUARANTINED) &&
             !slot->completion_pending && (s_online_v2.tx_selected_slot != i)))
        {
            slot->state = ZY100_ONLINE_V2_TX_SLOT_QUARANTINED;
            continue; /* The builder's publish/release retires this slot. */
        }
#endif
        /* GAP teardown is the only event that proves the stack has released
         * every buffer for this connection, including a narrowly selected
         * slot that had not yet returned from server_send_data(). */
        if (s_online_v2.tx_selected_slot == i)
        {
            s_online_v2.tx_selected_slot = ZY100_ONLINE_V2_INVALID_SLOT;
        }
        online_v2_tx_slot_reset_locked(slot);
    }
    s_online_v2.tx_complete_head = 0U;
    s_online_v2.tx_complete_tail = 0U;
    s_online_v2.tx_complete_count = 0U;
    s_online_v2.tx_in_flight = 0U;
    os_unlock(lock_state);
}

static void online_v2_latch_error(uint32_t error)
{
    if (s_online_v2.last_error == error)
    {
        return;
    }
    s_online_v2.last_error = error;
    s_online_v2.error_log_pending = true;
}

static bool online_v2_elapsed(uint32_t now_ms,
                              uint32_t start_ms,
                              uint32_t *elapsed_ms)
{
    if (elapsed_ms == NULL)
    {
        return false;
    }
    if (now_ms < start_ms)
    {
        *elapsed_ms = 0U;
        return false;
    }
    *elapsed_ms = now_ms - start_ms;
    return true;
}

static void online_v2_ack_queue_reset(void)
{
    memset(s_online_v2.ack_wait, 0, sizeof(s_online_v2.ack_wait));
    s_online_v2.ack_wait_head = 0U;
    s_online_v2.ack_wait_count = 0U;
}

static zy100_online_v2_ack_wait_t *online_v2_ack_queue_oldest(void)
{
    if (s_online_v2.ack_wait_count == 0U)
    {
        return NULL;
    }
    return &s_online_v2.ack_wait[s_online_v2.ack_wait_head];
}

static bool online_v2_ack_queue_push(uint8_t type,
                                     uint32_t record_id,
                                     uint32_t bytes,
                                     uint32_t start_ms)
{
    uint8_t tail;
    zy100_online_v2_ack_wait_t *wait;

    if (s_online_v2.ack_wait_count >= ZY100_ONLINE_RECORD_IN_FLIGHT_MAX)
    {
        return false;
    }
    tail = (uint8_t)((s_online_v2.ack_wait_head +
                      s_online_v2.ack_wait_count) %
                     ZY100_ONLINE_RECORD_IN_FLIGHT_MAX);
    wait = &s_online_v2.ack_wait[tail];
    wait->waiting = true;
    wait->type = type;
    wait->record_id = record_id;
    wait->bytes = bytes;
    wait->start_ms = start_ms;
    s_online_v2.ack_wait_count++;
    return true;
}

static void online_v2_ack_queue_pop(void)
{
    zy100_online_v2_ack_wait_t *wait = online_v2_ack_queue_oldest();

    if (wait == NULL)
    {
        return;
    }
    memset(wait, 0, sizeof(*wait));
    s_online_v2.ack_wait_head =
        (uint8_t)((s_online_v2.ack_wait_head + 1U) %
                  ZY100_ONLINE_RECORD_IN_FLIGHT_MAX);
    s_online_v2.ack_wait_count--;
}

static void online_v2_advance_generation(void)
{
    s_online_v2.session_generation++;
    if (s_online_v2.session_generation == 0U)
    {
        s_online_v2.session_generation++;
    }
}

static void online_v2_request_failure(zy100_online_stop_reason_t reason)
{
    bool entering_abort;
    bool cleanup_ready;
    uint32_t now_ms;

    if (!s_online_v2.active)
    {
        return;
    }
    now_ms = zy100_os_time_ms();
    online_v2_latch_abort_cause(reason,
                                ZY100_ONLINE_ABORT_ORIGIN_STREAM_SELF,
                                (uint32_t)reason,
                                now_ms);
    entering_abort = !s_online_v2.abort_requested;
    cleanup_ready = s_online_v2.abort_cleanup_ready;
    if ((s_online_v2.stop_reason == ZY100_ONLINE_STOP_REASON_NONE) ||
        ((s_online_v2.stop_reason == ZY100_ONLINE_STOP_REASON_HOST_STOP) &&
         (reason != ZY100_ONLINE_STOP_REASON_HOST_STOP)))
    {
        s_online_v2.stop_reason = (uint32_t)reason;
    }
    if (entering_abort)
    {
        online_v2_advance_generation();
    }
    s_online_v2.capture_stop_requested = true;
    s_online_v2.ready_latched = false;
    s_online_v2.end_requested = false;
    s_online_v2.end_wait_ack = false;
    s_online_v2.end_ack_received = false;
    s_online_v2.abort_requested = true;
    s_online_v2.abort_cleanup_ready = cleanup_ready;
    s_online_v2.abort_notify_accepted = false;
    s_online_v2.discard_tail_on_cleanup = true;
    if (entering_abort)
    {
        s_online_v2.abort_start_ms = now_ms;
    }
    s_online_v2.tx_active = false;
    online_v2_ack_queue_reset();
    online_v2_tx_cancel_unsent();
}

static void online_v2_finish_abort(bool timeout)
{
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
    online_v2_resource_finish();
#endif
    s_online_v2.active = false;
#if ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
    online_v2_release_workspace();
#endif
    s_online_v2.abort_requested = false;
    s_online_v2.abort_cleanup_ready = false;
    s_online_v2.abort_notify_accepted = false;
    s_online_v2.discard_tail_on_cleanup = false;
    s_online_v2.end_requested = false;
    s_online_v2.end_wait_ack = false;
    s_online_v2.end_ack_received = false;
    s_online_v2.tx_active = false;
    online_v2_ack_queue_reset();
    s_online_v2.stop_complete_reason = s_online_v2.stop_reason;
    s_online_v2.stop_complete_timeout = timeout;
    s_online_v2.stop_complete_pending = true;
    zy100_online_spool_abort_session();
}

static void online_v2_finish_end(void)
{
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
    online_v2_resource_finish();
#endif
    s_online_v2.active = false;
#if ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
    online_v2_release_workspace();
#endif
    s_online_v2.end_requested = false;
    s_online_v2.end_wait_ack = false;
    s_online_v2.end_ack_received = false;
    s_online_v2.abort_cleanup_ready = false;
    s_online_v2.discard_tail_on_cleanup = false;
    s_online_v2.tx_active = false;
    online_v2_ack_queue_reset();
    s_online_v2.stop_complete_reason = s_online_v2.stop_reason;
    s_online_v2.stop_complete_timeout = false;
    s_online_v2.stop_complete_pending = true;
    zy100_online_stream_log_summary("end_ack");
    zy100_online_spool_abort_session();
}

static void online_v2_check_ack_timeout(uint32_t now_ms)
{
    zy100_online_v2_ack_wait_t *wait;
    uint32_t elapsed_ms;

    wait = online_v2_ack_queue_oldest();
    if (wait == NULL)
    {
        return;
    }
    if (!online_v2_elapsed(now_ms,
                           wait->start_ms,
                           &elapsed_ms))
    {
        wait->start_ms = now_ms;
        return;
    }
    if (elapsed_ms < ZY100_ONLINE_STREAM_ACK_TIMEOUT_MS)
    {
        return;
    }
    s_online_v2.ack_timeout_count++;
    if ((wait->type == ZY100_ONLINE_RECORD_END) &&
        s_online_v2.active && s_online_v2.end_requested &&
        s_online_v2.end_wait_ack && !s_online_v2.link_disconnected)
    {
        if (!online_v2_tx_reuse_safe())
        {
            wait->start_ms = now_ms;
            return;
        }
        s_online_v2.end_wait_ack = false;
        online_v2_ack_queue_reset();
        /* END is the final frame, so restoring the sequence makes every retry
         * byte-identical while the frozen session/RTC metadata is rebuilt. */
        s_online_v2.chunk_seq--;
        DBG_DIRECT("[ONLINE_V2] end_ack_timeout retry session=%lu",
                   (unsigned long)s_online_v2.session_id);
        return;
    }
    online_v2_latch_error(1U);
    online_v2_request_failure(ZY100_ONLINE_STOP_REASON_ACK_TIMEOUT);
}

static void online_v2_note_tx_progress(uint32_t now_ms)
{
    uint32_t lock_state = os_lock();

    s_online_v2.last_tx_progress_ms = now_ms;
    os_unlock(lock_state);
}

static uint32_t online_v2_tx_diag_age(uint32_t now_ms, uint32_t then_ms)
{
    uint32_t age_ms = 0U;

    if (!online_v2_elapsed(now_ms, then_ms, &age_ms))
    {
        return 0U;
    }
    return age_ms;
}

static void online_v2_tx_diag_pressure_locked(uint8_t *ready,
                                              uint32_t *ring_map,
                                              uint8_t *completion_map)
{
    uint8_t i;
    uint8_t ready_count = 0U;
    uint32_t states = 0U;
    uint8_t completions = 0U;

    for (i = 0U; i < ZY100_ONLINE_V2_TX_WINDOW_SLOTS; i++)
    {
        const zy100_online_v2_tx_slot_t *slot = &s_online_v2.tx_ring[i];

        if (slot->state == ZY100_ONLINE_V2_TX_SLOT_READY)
        {
            ready_count++;
        }
        states |= ((uint32_t)slot->state & 0x07U) << (i * 3U);
        if (slot->completion_pending)
        {
            completions |= (uint8_t)(1U << i);
        }
    }
    if (ready != NULL)
    {
        *ready = ready_count;
    }
    if (ring_map != NULL)
    {
        *ring_map = states;
    }
    if (completion_map != NULL)
    {
        *completion_map = completions;
    }
}

static void online_v2_tx_diag_note_pressure(void)
{
    uint8_t ready = 0U;
    uint32_t lock_state = os_lock();

    online_v2_tx_diag_pressure_locked(&ready, NULL, NULL);
    if (ready > s_online_v2.tx_diag.ready_max)
    {
        s_online_v2.tx_diag.ready_max = ready;
    }
    if (s_online_v2.tx_in_flight > s_online_v2.tx_diag.in_flight_max)
    {
        s_online_v2.tx_diag.in_flight_max = s_online_v2.tx_in_flight;
    }
    os_unlock(lock_state);
}

static void online_v2_tx_diag_reset(uint32_t now_ms)
{
    memset(&s_online_v2.tx_diag, 0, sizeof(s_online_v2.tx_diag));
    s_online_v2.tx_diag.last_stage_ms = now_ms;
    s_online_v2.tx_diag.last_notify_ms = now_ms;
    s_online_v2.tx_diag.last_send_complete_ms = now_ms;
    s_online_v2.tx_diag.last_record_ack_ms = now_ms;
    s_online_v2.tx_diag.last_poll_ms = now_ms;
    s_online_v2.tx_diag.next_stat_ms =
        now_ms + ZY100_ONLINE_V2_TX_STAT_INTERVAL_MS;
}

static void online_v2_tx_diag_log_periodic(uint32_t now_ms)
{
    uint8_t ready = 0U;
    uint8_t in_flight;
    uint32_t lock_state;

    if (!s_online_v2.active ||
        ((int32_t)(now_ms - s_online_v2.tx_diag.next_stat_ms) < 0))
    {
        return;
    }
    lock_state = os_lock();
    online_v2_tx_diag_pressure_locked(&ready, NULL, NULL);
    in_flight = s_online_v2.tx_in_flight;
    os_unlock(lock_state);
    ZY100_LOG_DETAIL("[ONL_TX_STAT] sid=%lu n=%lu/%lu sc=%lu ack=%lu cr=%lu if=%u/%u rd=%u/%u pg=%lu age=%lu/%lu/%lu/%lu",
               (unsigned long)s_online_v2.session_id,
               (unsigned long)s_online_v2.tx_diag.notify_ok,
               (unsigned long)s_online_v2.tx_diag.notify_fail,
               (unsigned long)s_online_v2.tx_diag.send_complete,
               (unsigned long)s_online_v2.tx_diag.record_ack,
               (unsigned long)s_online_v2.tx_diag.send_complete_credits,
               (uint32_t)in_flight,
               (uint32_t)s_online_v2.tx_diag.in_flight_max,
               (uint32_t)ready,
               (uint32_t)s_online_v2.tx_diag.ready_max,
               (unsigned long)s_online_v2.tx_diag.poll_gap_max_ms,
               (unsigned long)online_v2_tx_diag_age(
                   now_ms, s_online_v2.tx_diag.last_stage_ms),
               (unsigned long)online_v2_tx_diag_age(
                   now_ms, s_online_v2.tx_diag.last_notify_ms),
               (unsigned long)online_v2_tx_diag_age(
                   now_ms, s_online_v2.tx_diag.last_send_complete_ms),
               (unsigned long)online_v2_tx_diag_age(
                   now_ms, s_online_v2.tx_diag.last_record_ack_ms));
    s_online_v2.tx_diag.next_stat_ms =
        now_ms + ZY100_ONLINE_V2_TX_STAT_INTERVAL_MS;
    s_online_v2.tx_diag.poll_gap_max_ms = 0U;
    s_online_v2.tx_diag.notify_ok = 0U;
    s_online_v2.tx_diag.notify_fail = 0U;
    s_online_v2.tx_diag.send_complete = 0U;
    s_online_v2.tx_diag.send_complete_credits = 0U;
    s_online_v2.tx_diag.record_ack = 0U;
    s_online_v2.tx_diag.ready_max = ready;
    s_online_v2.tx_diag.in_flight_max = in_flight;
}

static zy100_online_v2_tx_stall_class_t
online_v2_tx_diag_classify(uint32_t now_ms, uint8_t ready, uint8_t in_flight)
{
    if (s_online_v2.tx_diag.poll_gap_max_ms >=
        ZY100_ONLINE_STREAM_ACK_TIMEOUT_MS)
    {
        return ZY100_ONLINE_TX_STALL_APP_POLL;
    }
    if ((s_online_v2.tx_diag.notify_fail != 0U) && (ready != 0U))
    {
        return ZY100_ONLINE_TX_STALL_NOTIFY_REJECTED;
    }
    if ((in_flight != 0U) &&
        (online_v2_tx_diag_age(
             now_ms, s_online_v2.tx_diag.last_send_complete_ms) >=
         ZY100_ONLINE_STREAM_ACK_TIMEOUT_MS))
    {
        return ZY100_ONLINE_TX_STALL_SEND_COMPLETE_MISSING;
    }
    if ((ready != 0U) && (in_flight < s_online_v2.tx_window_limit))
    {
        return ZY100_ONLINE_TX_STALL_READY_NOT_SUBMITTED;
    }
    if ((s_online_v2.ack_wait_count != 0U) &&
        (online_v2_tx_diag_age(
             now_ms, s_online_v2.tx_diag.last_record_ack_ms) >=
         ZY100_ONLINE_STREAM_ACK_TIMEOUT_MS))
    {
        return ZY100_ONLINE_TX_STALL_ACK_PROGRESS_MISSING;
    }
    return ZY100_ONLINE_TX_STALL_STATE_INCONSISTENT;
}

static void online_v2_tx_diag_log_stall(uint32_t now_ms, uint32_t age_ms)
{
    uint8_t ready = 0U;
    uint8_t in_flight;
    uint8_t completion_map = 0U;
    uint8_t oldest_slot = ZY100_ONLINE_V2_INVALID_SLOT;
    uint8_t oldest_ack_type = 0U;
    uint8_t completion_head;
    uint8_t completion_count;
    uint32_t ring_map = 0U;
    uint32_t oldest_ack_record = 0U;
    uint32_t lock_state;
    zy100_online_v2_tx_stall_class_t stall_class;

    if (s_online_v2.tx_diag.stall_logged)
    {
        return;
    }
    lock_state = os_lock();
    online_v2_tx_diag_pressure_locked(&ready, &ring_map, &completion_map);
    in_flight = s_online_v2.tx_in_flight;
    completion_head = s_online_v2.tx_complete_head;
    completion_count = s_online_v2.tx_complete_count;
    if (s_online_v2.tx_complete_count != 0U)
    {
        oldest_slot = s_online_v2.tx_complete_slots[
            s_online_v2.tx_complete_head];
    }
    if (s_online_v2.ack_wait_count != 0U)
    {
        const zy100_online_v2_ack_wait_t *wait =
            &s_online_v2.ack_wait[s_online_v2.ack_wait_head];

        oldest_ack_type = wait->type;
        oldest_ack_record = wait->record_id;
    }
    os_unlock(lock_state);
    stall_class = online_v2_tx_diag_classify(now_ms, ready, in_flight);
    DBG_DIRECT("[ONL_TX_STALL] sid=%lu cls=%u age=%lu tx=%u if=%u rd=%u ring=%lx cp=%x cq=%u/%u old=%u/%lu/%u a=%lu/%lu/%lu/%lu",
               (unsigned long)s_online_v2.session_id,
               (uint32_t)stall_class,
               (unsigned long)age_ms,
               s_online_v2.tx_active ? 1U : 0U,
               (uint32_t)in_flight,
               (uint32_t)ready,
               (unsigned long)ring_map,
               (uint32_t)completion_map,
               (uint32_t)completion_head,
               (uint32_t)completion_count,
               (uint32_t)oldest_slot,
               (unsigned long)oldest_ack_record,
               (uint32_t)oldest_ack_type,
               (unsigned long)online_v2_tx_diag_age(
                   now_ms, s_online_v2.tx_diag.last_stage_ms),
               (unsigned long)online_v2_tx_diag_age(
                   now_ms, s_online_v2.tx_diag.last_notify_ms),
               (unsigned long)online_v2_tx_diag_age(
                   now_ms, s_online_v2.tx_diag.last_send_complete_ms),
               (unsigned long)online_v2_tx_diag_age(
                   now_ms, s_online_v2.tx_diag.last_record_ack_ms));
    s_online_v2.tx_diag.stall_logged = true;
}

static bool online_v2_tx_progress_pending(void)
{
    bool pending;
    uint8_t i;
    uint32_t lock_state = os_lock();

    pending = s_online_v2.tx_active || (s_online_v2.tx_in_flight != 0U);
    for (i = 0U; !pending && (i < ZY100_ONLINE_V2_TX_WINDOW_SLOTS); i++)
    {
        pending = s_online_v2.tx_ring[i].state ==
                  ZY100_ONLINE_V2_TX_SLOT_READY;
    }
    os_unlock(lock_state);
    return pending;
}

static void online_v2_check_tx_progress_timeout(uint32_t now_ms)
{
    uint32_t elapsed_ms;

    if (!s_online_v2.active || s_online_v2.abort_requested)
    {
        return;
    }
    if (!online_v2_tx_progress_pending())
    {
        online_v2_note_tx_progress(now_ms);
        return;
    }
    if (!online_v2_elapsed(now_ms,
                           s_online_v2.last_tx_progress_ms,
                           &elapsed_ms))
    {
        online_v2_note_tx_progress(now_ms);
        return;
    }
    if (elapsed_ms < ZY100_ONLINE_STREAM_ACK_TIMEOUT_MS)
    {
        return;
    }
    online_v2_tx_diag_log_stall(now_ms, elapsed_ms);
    online_v2_latch_error(16U);
    online_v2_request_failure(ZY100_ONLINE_STOP_REASON_TX_STALL);
}

static bool online_v2_build_start(uint16_t notify_max,
                                  uint8_t *frame,
                                  uint16_t *frame_len)
{
    uint8_t *payload;
    const uint16_t payload_len = 28U;

    if (notify_max < (ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES + payload_len))
    {
        return false;
    }
    payload = &frame[ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES];
    zy100_put_u32_le(&payload[0], s_online_v2.session_id);
    zy100_put_u32_le(&payload[4], s_online_v2.user_id);
    zy100_put_u32_le(&payload[8], s_online_v2.training_id);
    zy100_put_u32_le(&payload[12], ZY100_ONLINE_STREAM_ACK_TIMEOUT_MS);
    zy100_put_u32_le(&payload[16], 1U);
    zy100_put_u32_le(&payload[20], ZY100_ONLINE_SPOOL_REGION_BYTES);
    zy100_put_u32_le(&payload[24],
                      zy100_online_spool_erased_ahead_bytes());
    online_v2_build_outer(ZY100_BLE_ONLINE_FRAME_START,
                          ZY100_ONLINE_V2_FRAME_FLAG_LAST,
                          online_v2_take_sequence(),
                          frame,
                          payload_len);
    *frame_len = ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES + payload_len;
    return true;
}

#if ZY100_ONLINE_STRESS_TEST_ENABLE
static void online_v2_stress_snapshot(uint8_t *payload,
                                       const zy100_online_spool_stats_t *stats)
{
    zy100_online_stress_transport(s_online_v2.stop_reason, stats->pending_bytes,
        stats->pending_high_water, s_online_resource.ack_max_ms,
        stats->erase_count, stats->wrap_count,
        s_online_resource.heap_data_min, s_online_resource.heap_buffer_min,
        s_online_v2.mtu, s_online_v2.conn_interval,
        (uint32_t)s_online_v2.phy_tx | ((uint32_t)s_online_v2.phy_rx << 8U));
    zy100_online_stress_encode(payload);
}

static bool online_v2_build_stress(uint16_t notify_max, uint8_t *frame,
                                   uint16_t *frame_len)
{
    zy100_online_spool_stats_t stats;
    if (notify_max < ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES + ZY100_STRESS_STATUS_BYTES)
    { return false; }
    zy100_online_spool_get_stats(&stats);
    online_v2_stress_snapshot(frame + ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES, &stats);
    online_v2_build_outer(ZY100_BLE_ONLINE_FRAME_STRESS, 0U,
        online_v2_take_sequence(), frame, ZY100_STRESS_STATUS_BYTES);
    *frame_len = ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES + ZY100_STRESS_STATUS_BYTES;
    return true;
}
#endif

static bool online_v2_build_end(uint16_t notify_max,
                                 uint8_t *frame,
                                 uint16_t *frame_len)
{
    uint8_t *payload;
    uint16_t payload_len = ZY100_ONLINE_END_BASE_BYTES;
    uint16_t metadata_version = 0U;
    uint16_t metadata_bytes = 0U;
    zy100_online_spool_stats_t stats;

    if ((s_online_v2.capability_mask &
         ZY100_ONLINE_CAPABILITY_FIXED40_UNIX_ENDPOINT_V3) != 0U)
    {
        payload_len = ZY100_ONLINE_END_FIXED40_TOTAL_BYTES_V3;
        metadata_version = ZY100_ONLINE_END_FIXED40_META_VERSION_V3;
        metadata_bytes = ZY100_ONLINE_END_FIXED40_META_BYTES_V3;
    }
    else if ((s_online_v2.capability_mask &
         ZY100_ONLINE_CAPABILITY_UNIX_ENDPOINT_V2) != 0U)
    {
        payload_len = ZY100_ONLINE_END_RTC_TOTAL_BYTES_V2;
        metadata_version = ZY100_ONLINE_END_RTC_META_VERSION_V2;
        metadata_bytes = ZY100_ONLINE_END_RTC_META_BYTES_V2;
    }
    else if ((s_online_v2.capability_mask &
              ZY100_ONLINE_CAPABILITY_RTC_ENDPOINT_V1) != 0U)
    {
        payload_len = ZY100_ONLINE_END_RTC_TOTAL_BYTES_V1;
        metadata_version = ZY100_ONLINE_END_RTC_META_VERSION_V1;
        metadata_bytes = ZY100_ONLINE_END_RTC_META_BYTES_V1;
    }

#if ZY100_ONLINE_STRESS_TEST_ENABLE
    payload_len += ZY100_STRESS_STATUS_BYTES;
#endif
    if (notify_max < (ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES + payload_len))
    {
        return false;
    }
    zy100_online_spool_get_stats(&stats);
    payload = &frame[ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES];
    zy100_put_u32_le(&payload[0], s_online_v2.session_id);
    zy100_put_u32_le(&payload[4], s_online_v2.stop_reason);
    zy100_put_u32_le(&payload[8], stats.produced[ZY100_ONLINE_RECORD_RAW]);
    zy100_put_u32_le(&payload[12], stats.produced[ZY100_ONLINE_RECORD_SUMMARY]);
    zy100_put_u32_le(&payload[16], stats.produced[ZY100_ONLINE_RECORD_EVENT]);
    zy100_put_u32_le(&payload[20], stats.acked[ZY100_ONLINE_RECORD_RAW]);
    zy100_put_u32_le(&payload[24], stats.acked[ZY100_ONLINE_RECORD_SUMMARY]);
    zy100_put_u32_le(&payload[28], stats.acked[ZY100_ONLINE_RECORD_EVENT]);
    zy100_put_u32_le(&payload[32], stats.pending_high_water);
    if (metadata_version != 0U)
    {
        zy100_put_u16_le(&payload[36], metadata_version);
        zy100_put_u16_le(&payload[38], metadata_bytes);
        zy100_put_u32_le(&payload[40], s_online_v2.clock_meta.status);
        if (metadata_version ==
            ZY100_ONLINE_END_FIXED40_META_VERSION_V3)
        {
            zy100_put_u32_le(&payload[44],
                              s_online_v2.clock_meta.accepted_packet_count);
            zy100_put_u16_le(&payload[48],
                              s_online_v2.clock_meta.first_imu_timestamp_raw);
            zy100_put_u16_le(&payload[50],
                              s_online_v2.clock_meta.last_imu_timestamp_raw);
            zy100_put_u64_le(&payload[52],
                              s_online_v2.clock_meta.first_unix_time_us);
            zy100_put_u64_le(&payload[60],
                              s_online_v2.clock_meta.last_unix_time_us);
        }
        else
        {
            zy100_put_u32_le(&payload[44],
                              s_online_v2.clock_meta.rtc_nominal_tick_hz);
            zy100_put_u32_le(&payload[48],
                              s_online_v2.clock_meta.accepted_packet_count);
            zy100_put_u16_le(&payload[52],
                              s_online_v2.clock_meta.first_imu_timestamp_raw);
            zy100_put_u16_le(&payload[54],
                              s_online_v2.clock_meta.last_imu_timestamp_raw);
            zy100_put_u64_le(&payload[56],
                              s_online_v2.clock_meta.first_rtc_tick);
            zy100_put_u64_le(&payload[64],
                              s_online_v2.clock_meta.last_rtc_tick);
            zy100_put_u64_le(&payload[72],
                              s_online_v2.clock_meta.rtc_wrap_ticks);
            if (metadata_version == ZY100_ONLINE_END_RTC_META_VERSION_V2)
            {
                zy100_put_u64_le(&payload[80],
                                  s_online_v2.clock_meta.first_unix_time_ms);
                zy100_put_u64_le(&payload[88],
                                  s_online_v2.clock_meta.last_unix_time_ms);
            }
        }
    }
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    online_v2_stress_snapshot(payload + ZY100_ONLINE_END_FIXED40_TOTAL_BYTES_V3, &stats);
    zy100_online_stress_terminal_freeze();
#endif
    online_v2_build_outer(ZY100_BLE_ONLINE_FRAME_END,
                          ZY100_ONLINE_V2_FRAME_FLAG_LAST,
                          online_v2_take_sequence(),
                          frame,
                          payload_len);
    *frame_len = ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES + payload_len;
    return true;
}

static bool online_v2_end_delivery_closed(void)
{
    zy100_online_spool_stats_t stats;
    uint8_t type;

    if (s_online_v2.discard_tail_on_cleanup ||
        (s_online_v2.raw_accepted != s_online_v2.raw_saved) ||
        !zy100_online_spool_all_acked())
    {
        return false;
    }
    zy100_online_spool_get_stats(&stats);
    for (type = ZY100_ONLINE_RECORD_RAW;
         type < ZY100_ONLINE_RECORD_TYPE_COUNT;
         type++)
    {
        if ((stats.produced[type] != stats.committed[type]) ||
            (stats.committed[type] != stats.acked[type]) ||
            (s_online_v2.dropped[type] != 0U))
        {
            return false;
        }
    }
    return true;
}

static bool online_v2_build_abort(uint16_t notify_max,
                                  uint8_t *frame,
                                  uint16_t *frame_len)
{
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    zy100_online_spool_stats_t stats;
    if (notify_max < ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES + ZY100_STRESS_STATUS_BYTES)
    { return false; }
    zy100_online_spool_get_stats(&stats);
    online_v2_stress_snapshot(frame + ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES, &stats);
    zy100_online_stress_terminal_freeze();
#else
    if (notify_max < ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES) { return false; }
#endif
    online_v2_build_outer(ZY100_BLE_ONLINE_FRAME_ABORT,
                          ZY100_ONLINE_V2_FRAME_FLAG_LAST,
                          online_v2_take_sequence(),
                          frame,
#if ZY100_ONLINE_STRESS_TEST_ENABLE
                          ZY100_STRESS_STATUS_BYTES);
    *frame_len = ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES + ZY100_STRESS_STATUS_BYTES;
#else
                          0U);
    *frame_len = ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES;
#endif
    return true;
}

static bool online_v2_record_staging_allowed_locked(void)
{
    if (!s_online_v2.active || !s_online_v2.start_sent ||
        s_online_v2.end_wait_ack || s_online_v2.abort_requested ||
        (s_online_v2.ack_wait_count >= ZY100_ONLINE_RECORD_IN_FLIGHT_MAX))
    {
        return false;
    }
    if (!s_online_v2.end_requested)
    {
        return true;
    }
    return s_online_v2.abort_cleanup_ready &&
           !s_online_v2.discard_tail_on_cleanup;
}

bool zy100_online_stream_stage_record_fragment(uint32_t now_ms)
{
    zy100_online_spool_io_result_t io_result;
    zy100_online_spool_record_t tx;
    uint32_t tx_offset;
    uint32_t session_generation;
    uint16_t notify_max;
    uint16_t sequence;
    uint8_t slot_index;
    zy100_online_v2_tx_slot_t *slot;
    uint8_t *payload;
    uint8_t *fragment;
    uint16_t max_fragment;
    uint16_t fragment_len;
    uint8_t flags = 0U;
    bool last_fragment;
    bool need_record;
    bool published;
    uint8_t i;
    uint32_t lock_state;

    s_online_v2.stage_call++;
    lock_state = os_lock();
    if (!online_v2_record_staging_allowed_locked())
    {
        os_unlock(lock_state);
        s_online_v2.stage_gate++;
        return false;
    }
    /* Flash staging is serialized by the capture task, but READY fragments
     * may accumulate so the lower-priority App task can fill one BLE window
     * whenever it gets scheduled. */
    for (i = 0U; i < ZY100_ONLINE_V2_TX_WINDOW_SLOTS; i++)
    {
        if (s_online_v2.tx_ring[i].state ==
            ZY100_ONLINE_V2_TX_SLOT_BUILDING)
        {
            os_unlock(lock_state);
            s_online_v2.stage_no_slot++;
            return false;
        }
    }
    if (online_v2_tx_find_free_locked() == ZY100_ONLINE_V2_INVALID_SLOT)
    {
        os_unlock(lock_state);
        s_online_v2.stage_no_slot++;
        return false;
    }
    need_record = !s_online_v2.tx_active;
    os_unlock(lock_state);

    if (need_record)
    {
        io_result = zy100_online_spool_peek_tx(&tx);
        if ((io_result == ZY100_ONLINE_SPOOL_IO_EMPTY) ||
            (io_result == ZY100_ONLINE_SPOOL_IO_BUSY))
        {
            s_online_v2.stage_source_wait++;
            return false;
        }
        if (io_result != ZY100_ONLINE_SPOOL_IO_OK)
        {
            online_v2_latch_error(7U);
            online_v2_request_failure(ZY100_ONLINE_STOP_REASON_FLASH_IO);
            return false;
        }
    }

    lock_state = os_lock();
    if (!online_v2_record_staging_allowed_locked())
    {
        os_unlock(lock_state);
        return false;
    }
    if (!s_online_v2.tx_active)
    {
        if (!need_record)
        {
            os_unlock(lock_state);
            return false;
        }
        s_online_v2.tx = tx;
        s_online_v2.tx_active = true;
        s_online_v2.tx_offset = 0U;
        s_online_v2.tx_stage_offset = 0U;
        s_online_v2.last_tx_progress_ms = now_ms;
    }
    tx = s_online_v2.tx;
    tx_offset = s_online_v2.tx_stage_offset;
    session_generation = s_online_v2.session_generation;
    notify_max = ZY100_ONLINE_V2_NOTIFY_BYTES;
    if ((s_online_v2.mtu > 3U) &&
        ((uint16_t)(s_online_v2.mtu - 3U) < notify_max))
    {
        notify_max = (uint16_t)(s_online_v2.mtu - 3U);
    }
    os_unlock(lock_state);

    /* The final fragment may already be READY while App has not submitted it
     * yet. That is normal window backpressure, not a malformed record. */
    if ((tx.record_bytes != 0U) && (tx_offset == tx.record_bytes))
    {
        return false;
    }
    if ((tx.record_bytes == 0U) || (tx_offset > tx.record_bytes) ||
        (notify_max <= (ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES +
                        ZY100_ONLINE_V2_RECORD_PAYLOAD_HEADER_BYTES)))
    {
        online_v2_latch_error(10U);
        online_v2_request_failure(ZY100_ONLINE_STOP_REASON_INTERNAL);
        return false;
    }
    max_fragment = (uint16_t)(notify_max -
        ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES -
        ZY100_ONLINE_V2_RECORD_PAYLOAD_HEADER_BYTES);
    fragment_len = (uint16_t)(
        ((tx.record_bytes - tx_offset) < max_fragment) ?
        (tx.record_bytes - tx_offset) : max_fragment);
    last_fragment = (tx_offset + fragment_len) >= tx.record_bytes;
    if (last_fragment)
    {
        flags = ZY100_ONLINE_V2_FRAME_FLAG_LAST;
    }
    slot_index = online_v2_tx_claim_slot(
        ZY100_BLE_ONLINE_FRAME_RECORD,
        tx.type,
        tx.record_id,
        tx.record_bytes,
        last_fragment,
        fragment_len,
        (uint16_t)(ZY100_ONLINE_V2_RECORD_PAYLOAD_HEADER_BYTES + fragment_len));
    if (slot_index == ZY100_ONLINE_V2_INVALID_SLOT)
    {
        return false;
    }
    slot = &s_online_v2.tx_ring[slot_index];
    payload = &slot->data[ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES];
    zy100_put_u32_le(&payload[0], s_online_v2.session_id);
    payload[4] = tx.type;
    payload[5] = 0U;
    zy100_put_u16_le(&payload[6], flags);
    zy100_put_u32_le(&payload[8], tx.record_id);
    zy100_put_u32_le(&payload[12], tx_offset);
    zy100_put_u32_le(&payload[16], tx.record_bytes);
    zy100_put_u16_le(&payload[20], fragment_len);
    zy100_put_u16_le(&payload[22], 0U);
    zy100_put_u32_le(&payload[24], tx.crc32);
    fragment = &payload[ZY100_ONLINE_V2_RECORD_PAYLOAD_HEADER_BYTES];
    io_result = zy100_online_spool_read(tx.header_addr + tx_offset,
                                        fragment,
                                        fragment_len);
    if (io_result == ZY100_ONLINE_SPOOL_IO_BUSY)
    {
        online_v2_tx_release_builder(slot_index);
        s_online_v2.stage_flash_busy++;
        return false;
    }
    if (io_result != ZY100_ONLINE_SPOOL_IO_OK)
    {
        online_v2_tx_release_builder(slot_index);
        online_v2_latch_error(2U);
        online_v2_request_failure(ZY100_ONLINE_STOP_REASON_FLASH_IO);
        return false;
    }
    lock_state = os_lock();
    if (!s_online_v2.active ||
        (s_online_v2.session_generation != session_generation) ||
        !s_online_v2.tx_active ||
        (s_online_v2.tx.record_id != tx.record_id) ||
        (s_online_v2.tx_stage_offset != tx_offset))
    {
        os_unlock(lock_state);
        online_v2_tx_release_builder(slot_index);
        return false;
    }
    sequence = s_online_v2.chunk_seq++;
    os_unlock(lock_state);
    online_v2_build_outer(ZY100_BLE_ONLINE_FRAME_RECORD,
                          flags,
                          sequence,
                          slot->data,
                          (uint16_t)(ZY100_ONLINE_V2_RECORD_PAYLOAD_HEADER_BYTES +
                                     fragment_len));
    slot->fragment_offset = tx_offset;
    published = online_v2_tx_publish_slot(
        slot_index,
        (uint16_t)(ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES +
                   ZY100_ONLINE_V2_RECORD_PAYLOAD_HEADER_BYTES +
                   fragment_len));
    if (published)
    {
        lock_state = os_lock();
        if (s_online_v2.active &&
            (s_online_v2.session_generation == session_generation) &&
            s_online_v2.tx_active &&
            (s_online_v2.tx.record_id == tx.record_id) &&
            (s_online_v2.tx_stage_offset == tx_offset))
        {
            s_online_v2.tx_stage_offset += fragment_len;
        }
        os_unlock(lock_state);
        online_v2_note_tx_progress(now_ms);
        s_online_v2.tx_diag.last_stage_ms = now_ms;
        s_online_v2.stage_ok++;
    }
    return published;
}

void zy100_online_stream_init(void)
{
    uint8_t i;

    memset(&s_online_v2, 0, sizeof(s_online_v2));
    s_online_v2.conn_id = 0xFFU;
    s_online_v2.tx_selected_slot = ZY100_ONLINE_V2_INVALID_SLOT;
    s_online_v2.ack_latency_min_ms = ZY100_ONLINE_V2_U32_MAX;
#if ZY100_ONLINE_V2_TX_WINDOW_ENABLE
    s_online_v2.tx_window_limit = ZY100_ONLINE_V2_TX_WINDOW_SLOTS;
#else
    s_online_v2.tx_window_limit = 1U;
#endif
    for (i = 0U; i < ZY100_ONLINE_V2_TX_WINDOW_SLOTS; i++)
    {
        online_v2_tx_slot_reset_locked(&s_online_v2.tx_ring[i]);
    }
    zy100_online_spool_init();
}

void zy100_online_stream_handle_ready(uint8_t conn_id,
                                      uint32_t capability_mask,
                                      uint64_t host_time_ms,
                                      uint32_t now_ms)
{
    (void)host_time_ms;
    (void)now_ms;
    if (s_online_v2.active || s_online_v2.end_wait_ack ||
        s_online_v2.summary_pending || !online_v2_tx_reuse_safe())
    {
        return;
    }
    if ((capability_mask & ZY100_ONLINE_CAPABILITY_REQUIRED_V3) !=
        ZY100_ONLINE_CAPABILITY_REQUIRED_V3)
    {
        s_online_v2.ready_latched = false;
        return;
    }
    s_online_v2.ready_latched = true;
    s_online_v2.conn_id = conn_id;
    s_online_v2.capability_mask = capability_mask;
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    zy100_online_stress_negotiate(conn_id, capability_mask);
#endif
    zy100_online_spool_ready_begin();
}

void zy100_online_stream_invalidate_ready(uint8_t conn_id,
                                          const char *reason)
{
    if (!s_online_v2.ready_latched || (s_online_v2.conn_id != conn_id))
    {
        return;
    }
    s_online_v2.ready_latched = false;
    DBG_DIRECT("[ONLINE_STATE] ready_invalidated conn=%u reason=%s",
               conn_id,
               (reason != NULL) ? reason : "offline_capture");
}

bool zy100_online_stream_ready(uint8_t conn_id)
{
    return s_online_v2.ready_latched &&
           (s_online_v2.conn_id == conn_id) &&
           online_v2_tx_reuse_safe();
}

bool zy100_online_stream_can_start_online(uint8_t conn_id,
                                          uint32_t *detail_out)
{
    uint32_t detail = (zy100_online_stream_ready(conn_id) &&
                       !s_online_v2.summary_pending) ?
        ZY100_BLE_START_DETAIL_ONLINE_READY :
        ZY100_BLE_START_DETAIL_ONLINE_FALLBACK_NOT_READY;
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    /* TX statistics are populated only after START. Query this connection
     * directly so first start and reconnect do not depend on a prior TX pump. */
    uint16_t mtu = 0U;
    T_GAP_CAUSE cause = le_get_conn_param(GAP_PARAM_CONN_MTU_SIZE, &mtu, conn_id);
    if ((cause != GAP_CAUSE_SUCCESS) ||
        (mtu < (3U + ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES +
         ZY100_ONLINE_END_FIXED40_TOTAL_BYTES_V3 + ZY100_STRESS_STATUS_BYTES)))
    {
        DBG_DIRECT("[STRESS_START] mtu_gate conn=%u cause=%u mtu=%u need=%u",
                   conn_id, (uint32_t)cause, mtu,
                   (uint32_t)(3U + ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES +
                   ZY100_ONLINE_END_FIXED40_TOTAL_BYTES_V3 + ZY100_STRESS_STATUS_BYTES));
        detail = ZY100_BLE_START_DETAIL_ONLINE_FALLBACK_NOT_READY;
    }
#endif
    if (detail_out != NULL)
    {
        *detail_out = detail;
    }
    return detail == ZY100_BLE_START_DETAIL_ONLINE_READY;
}

bool zy100_online_stream_start_prepare_accept(uint8_t conn_id,
                                               uint8_t seq,
                                               uint32_t now_ms)
{
    (void)now_ms;
    if (!zy100_online_stream_ready(conn_id) || s_online_v2.active ||
        s_online_v2.start_preparing)
    {
        return false;
    }
    s_online_v2.start_preparing = true;
    s_online_v2.start_prepare_started = false;
    s_online_v2.start_prepare_ready = false;
    s_online_v2.start_seq = seq;
    return true;
}

bool zy100_online_stream_start_prepare_begin(uint8_t conn_id,
                                              uint8_t seq,
                                              uint32_t now_ms,
                                              uint32_t *detail_out)
{
    uint32_t spool_detail = 0U;

    (void)now_ms;
    if (!s_online_v2.start_preparing ||
        (s_online_v2.conn_id != conn_id) ||
        (s_online_v2.start_seq != seq) ||
        s_online_v2.start_prepare_started)
    {
        if (detail_out != NULL)
        {
            *detail_out = ZY100_BLE_START_DETAIL_ONLINE_FALLBACK_NOT_READY;
        }
        return false;
    }
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    {
        zy100_online_spool_stats_t stats;
        zy100_online_spool_get_stats(&stats);
        zy100_stress_diag_prepare(stats.erase_count);
    }
#endif
    if (!zy100_online_spool_start_prepare_begin(
            ZY100_ONLINE_START_ERASE_BYTES, &spool_detail))
    {
        if (detail_out != NULL)
        {
            *detail_out = ZY100_BLE_START_DETAIL_ONLINE_FALLBACK_PREERASE |
                          (spool_detail << 16);
        }
        return false;
    }
    s_online_v2.start_prepare_started = true;
    if (detail_out != NULL)
    {
        *detail_out = ZY100_BLE_START_DETAIL_ONLINE_READY;
    }
    return true;
}

zy100_online_start_prep_status_t
zy100_online_stream_start_prepare_poll(uint32_t now_ms,
                                        uint32_t *detail_out)
{
    zy100_online_spool_start_prep_status_t status;
    uint32_t spool_detail = 0U;

    (void)now_ms;
    if (!s_online_v2.start_preparing ||
        !s_online_v2.start_prepare_started)
    {
        return ZY100_ONLINE_START_PREP_IDLE;
    }
    if (s_online_v2.start_prepare_ready)
    {
        if (detail_out != NULL)
        {
            *detail_out = ZY100_BLE_START_DETAIL_ONLINE_READY;
        }
        return ZY100_ONLINE_START_PREP_DONE;
    }
    status = zy100_online_spool_start_prepare_poll(&spool_detail);
    if (status == ZY100_ONLINE_SPOOL_START_PREP_BUSY)
    {
        return ZY100_ONLINE_START_PREP_BUSY;
    }
    if (status != ZY100_ONLINE_SPOOL_START_PREP_DONE)
    {
        if (detail_out != NULL)
        {
            *detail_out = ZY100_BLE_START_DETAIL_ONLINE_FALLBACK_PREERASE |
                          (spool_detail << 16);
        }
        return ZY100_ONLINE_START_PREP_ERROR;
    }
    s_online_v2.start_prepare_ready = true;
    if (detail_out != NULL)
    {
        *detail_out = ZY100_BLE_START_DETAIL_ONLINE_READY;
    }
    return ZY100_ONLINE_START_PREP_DONE;
}

bool zy100_online_stream_start_prepare_ready(uint8_t conn_id, uint8_t seq)
{
    return s_online_v2.start_preparing &&
           s_online_v2.start_prepare_ready &&
           (s_online_v2.conn_id == conn_id) &&
           (s_online_v2.start_seq == seq);
}

bool zy100_online_stream_start_prepare_in_progress(void)
{
    return s_online_v2.start_preparing;
}

void zy100_online_stream_start_prepare_abort(uint32_t detail,
                                              uint32_t now_ms)
{
    (void)now_ms;
    zy100_online_spool_start_prepare_abort();
    s_online_v2.start_preparing = false;
    s_online_v2.start_prepare_started = false;
    s_online_v2.start_prepare_ready = false;
    s_online_v2.ready_latched = false;
    online_v2_latch_error(detail);
}

bool zy100_online_stream_prepare_start_blocking(uint8_t conn_id,
                                                uint32_t now_ms,
                                                uint32_t *detail_out)
{
    uint32_t spool_detail = 0U;
    uint32_t detail = ZY100_BLE_START_DETAIL_ONLINE_READY;

    (void)now_ms;
    if (!zy100_online_stream_ready(conn_id) || s_online_v2.active ||
        !online_v2_tx_reuse_safe())
    {
        detail = ZY100_BLE_START_DETAIL_ONLINE_FALLBACK_NOT_READY;
    }
    else if (!zy100_online_spool_prepare_start_blocking(
                 ZY100_ONLINE_START_ERASE_BYTES, &spool_detail))
    {
        detail = ZY100_BLE_START_DETAIL_ONLINE_FALLBACK_PREERASE |
                 (spool_detail << 16);
    }
    if (detail_out != NULL)
    {
        *detail_out = detail;
    }
    return detail == ZY100_BLE_START_DETAIL_ONLINE_READY;
}

void zy100_online_stream_handle_start_fallback(uint8_t conn_id,
                                               uint32_t detail,
                                               uint32_t now_ms)
{
    (void)now_ms;
    if (s_online_v2.conn_id == conn_id)
    {
        s_online_v2.ready_latched = false;
        s_online_v2.start_preparing = false;
        s_online_v2.start_prepare_started = false;
        s_online_v2.start_prepare_ready = false;
        zy100_online_spool_start_prepare_abort();
    }
    online_v2_latch_error(detail);
}

void zy100_online_stream_begin(uint8_t conn_id,
                               uint32_t session_id,
                               uint32_t user_id,
                               uint32_t training_id,
                               uint32_t now_ms)
{
    s_online_v2.start_preparing = false;
    s_online_v2.start_prepare_started = false;
    s_online_v2.start_prepare_ready = false;
    uint8_t i;
#if ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
    uint8_t *workspace;
#endif

    if (s_online_v2.active || !zy100_online_stream_ready(conn_id) ||
        !online_v2_tx_reuse_safe())
    {
        online_v2_latch_error(11U);
        return;
    }
#if ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
    if (!zy100_mode_workspace_online_open(&s_online_v2.workspace_token, &workspace))
    {
        online_v2_latch_error(11U);
        return;
    }
    for (i = 0U; i < ZY100_ONLINE_V2_TX_WINDOW_SLOTS; i++)
    {
        s_online_v2.tx_ring[i].data = workspace + ZY100_ONLINE_WORKSPACE_TX_OFFSET +
                                     i * ZY100_ONLINE_WORKSPACE_TX_BYTES;
    }
    s_online_v2.tx_view_pending = false;
#endif
    memset(&s_online_v2.gate, 0, sizeof(s_online_v2.gate));
    online_v2_advance_generation();
    s_online_v2.active = true;
    s_online_v2.start_sent = false;
    s_online_v2.end_requested = false;
    s_online_v2.end_wait_ack = false;
    s_online_v2.end_ack_received = false;
    s_online_v2.abort_requested = false;
    s_online_v2.abort_cleanup_ready = false;
    s_online_v2.abort_notify_accepted = false;
    s_online_v2.link_disconnected = false;
    s_online_v2.discard_tail_on_cleanup = false;
    s_online_v2.capture_stop_requested = false;
    s_online_v2.stop_complete_pending = false;
    s_online_v2.stop_complete_timeout = false;
    s_online_v2.summary_logged = false;
    s_online_v2.summary_pending = false;
    s_online_v2.summary_stage = 0U;
    memset(&s_online_v2.abort_cause, 0, sizeof(s_online_v2.abort_cause));
    s_online_v2.conn_id = conn_id;
    s_online_v2.session_id = session_id;
    s_online_v2.user_id = user_id;
    s_online_v2.training_id = training_id;
    s_online_v2.stop_reason = ZY100_ONLINE_STOP_REASON_NONE;
    s_online_v2.chunk_seq = 0U;
    s_online_v2.tx_active = false;
    s_online_v2.tx_offset = 0U;
    s_online_v2.tx_stage_offset = 0U;
    s_online_v2.tx_in_flight = 0U;
#if ZY100_ONLINE_V2_TX_WINDOW_ENABLE
    s_online_v2.tx_window_limit = ZY100_ONLINE_V2_TX_WINDOW_SLOTS;
#else
    s_online_v2.tx_window_limit = 1U;
#endif
    s_online_v2.tx_selected_slot = ZY100_ONLINE_V2_INVALID_SLOT;
    s_online_v2.tx_complete_head = 0U;
    s_online_v2.tx_complete_tail = 0U;
    s_online_v2.tx_complete_count = 0U;
    s_online_v2.ready_order_next = 0U;
    for (i = 0U; i < ZY100_ONLINE_V2_TX_WINDOW_SLOTS; i++)
    {
        online_v2_tx_slot_reset_locked(&s_online_v2.tx_ring[i]);
    }
    online_v2_ack_queue_reset();
    s_online_v2.session_start_ms = now_ms;
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
    online_v2_resource_begin(now_ms);
#endif
    s_online_v2.last_tx_progress_ms = now_ms;
    online_v2_tx_diag_reset(now_ms);
    s_online_v2.capture_segment_id = 0U;
    memset(&s_online_v2.clock_meta, 0, sizeof(s_online_v2.clock_meta));
    s_online_v2.clock_meta.status =
        ((s_online_v2.capability_mask &
          ZY100_ONLINE_CAPABILITY_FIXED40_UNIX_ENDPOINT_V3) != 0U) ?
        ZY100_ONLINE_CLOCK_META_TIMEBASE_UNAVAILABLE :
        ZY100_ONLINE_CLOCK_META_RTC_UNAVAILABLE;
    zy100_online_spool_begin(session_id);
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    zy100_online_stress_session(session_id);
#endif
    zy100_online_reset_trace_arm();
}

bool zy100_online_stream_active(void) { return s_online_v2.active; }
bool zy100_online_stream_fixed40_endpoint_enabled(void)
{
    return (s_online_v2.capability_mask &
            ZY100_ONLINE_CAPABILITY_FIXED40_UNIX_ENDPOINT_V3) != 0U;
}
bool zy100_online_stream_end_wait_ack(void) { return s_online_v2.end_wait_ack; }
bool zy100_online_stream_start_sent(void) { return s_online_v2.start_sent; }
bool zy100_online_stream_end_requested(void)
{
    return s_online_v2.end_requested || s_online_v2.end_wait_ack ||
           s_online_v2.abort_requested;
}

bool zy100_online_stream_should_pause_for_upload(uint32_t now_ms)
{
    (void)now_ms;
    /* Continuous mode never creates a capture pause for upload/maintenance. */
    return false;
}

bool zy100_online_stream_upload_drained(void)
{
    if (!s_online_v2.active || !s_online_v2.start_sent ||
        s_online_v2.end_requested || s_online_v2.end_wait_ack ||
        (s_online_v2.ack_wait_count != 0U) ||
        !zy100_online_spool_all_acked())
    {
        return false;
    }
    if (!zy100_online_spool_low_water())
    {
        return true;
    }
    return zy100_online_spool_try_wrap_empty(
        ZY100_ONLINE_SPOOL_RECOVER_TARGET_BYTES);
}

uint32_t zy100_online_stream_pending_bytes(void)
{
    return zy100_online_spool_pending_bytes();
}

bool zy100_online_stream_clear_spool_begin(void)
{
    return zy100_online_spool_clear_begin();
}

zy100_flash_prepare_status_t zy100_online_stream_clear_spool_poll(void)
{
    return zy100_online_spool_clear_poll();
}

bool zy100_online_stream_post_session_cleanup_begin(void)
{
    return zy100_online_spool_post_session_cleanup_begin();
}

bool zy100_online_stream_abort_cleanup_begin(void)
{
    return zy100_online_spool_abort_cleanup_begin();
}

zy100_online_flash_step_result_t
zy100_online_stream_post_session_cleanup_poll(uint32_t target_bytes)
{
    zy100_online_spool_pump_result_t result =
        zy100_online_spool_post_session_cleanup_poll(target_bytes);

    if (result == ZY100_ONLINE_SPOOL_PUMP_ERROR)
    {
        return ZY100_ONLINE_FLASH_STEP_ERROR;
    }
    if (result == ZY100_ONLINE_SPOOL_PUMP_PROGRESS)
    {
        return ZY100_ONLINE_FLASH_STEP_ERASE_PROGRESS;
    }
    return ZY100_ONLINE_FLASH_STEP_IDLE;
}

bool zy100_online_stream_post_session_ready(uint32_t minimum_bytes)
{
    return zy100_online_spool_post_session_ready(minimum_bytes);
}

bool zy100_online_stream_flash_idle(void)
{
    /* A completed erase can leave the software WIP latch set across the
     * button-controlled sleep/wake path because the app task is retained.
     * Reconcile the latch with Flash before applying the START gate. */
    return zy100_online_spool_flash_idle_poll();
}

void zy100_online_stream_request_stop(zy100_online_stop_reason_t reason,
                                      uint32_t now_ms)
{
    (void)now_ms;
    if (!s_online_v2.active || s_online_v2.abort_requested)
    {
        return;
    }
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    if (reason != ZY100_ONLINE_STOP_REASON_NONE &&
        reason != ZY100_ONLINE_STOP_REASON_HOST_STOP)
    { zy100_online_stress_failure((uint32_t)reason); }
#endif
    if ((reason == ZY100_ONLINE_STOP_REASON_LOW_BATTERY) &&
        (s_online_v2.stop_reason == ZY100_ONLINE_STOP_REASON_HOST_STOP) &&
        !s_online_v2.end_wait_ack)
    {
        s_online_v2.stop_reason = ZY100_ONLINE_STOP_REASON_LOW_BATTERY;
    }
    if (s_online_v2.end_requested || s_online_v2.end_wait_ack)
    {
        return;
    }
    if (s_online_v2.stop_reason == ZY100_ONLINE_STOP_REASON_NONE)
    {
        s_online_v2.stop_reason = (uint32_t)reason;
    }
    s_online_v2.capture_stop_requested = true;
    s_online_v2.end_requested = true;
    s_online_v2.end_ack_received = false;
    s_online_v2.abort_cleanup_ready = false;
    s_online_v2.discard_tail_on_cleanup =
        false;
}

void zy100_online_stream_request_abort_ex(
    zy100_online_stop_reason_t reason,
    zy100_online_abort_origin_t origin,
    uint32_t detail,
    uint32_t now_ms)
{
    bool entering_abort;
    bool cleanup_ready;

    if (!s_online_v2.active)
    {
        return;
    }
    online_v2_latch_abort_cause(reason, origin, detail, now_ms);
    entering_abort = !s_online_v2.abort_requested;
    cleanup_ready = s_online_v2.abort_cleanup_ready;
    if ((s_online_v2.stop_reason == ZY100_ONLINE_STOP_REASON_NONE) ||
        ((s_online_v2.stop_reason == ZY100_ONLINE_STOP_REASON_HOST_STOP) &&
         (reason != ZY100_ONLINE_STOP_REASON_HOST_STOP)))
    {
        s_online_v2.stop_reason = (uint32_t)reason;
    }
    if (entering_abort)
    {
        online_v2_advance_generation();
    }
    s_online_v2.capture_stop_requested = true;
    s_online_v2.end_requested = false;
    s_online_v2.end_wait_ack = false;
    s_online_v2.end_ack_received = false;
    s_online_v2.abort_requested = true;
    s_online_v2.abort_cleanup_ready = cleanup_ready;
    s_online_v2.abort_notify_accepted = false;
    s_online_v2.discard_tail_on_cleanup = true;
    if (entering_abort)
    {
        s_online_v2.abort_start_ms = now_ms;
    }
    s_online_v2.tx_active = false;
    online_v2_ack_queue_reset();
    online_v2_tx_cancel_unsent();
}

void zy100_online_stream_request_abort(zy100_online_stop_reason_t reason,
                                       uint32_t now_ms)
{
    zy100_online_stream_request_abort_ex(
        reason,
        ZY100_ONLINE_ABORT_ORIGIN_UNKNOWN,
        (uint32_t)reason,
        now_ms);
}

bool zy100_online_stream_abort_cause_valid(void)
{
    return s_online_v2.abort_cause.valid;
}

bool zy100_online_stream_abort_terminal_pending(void)
{
    return s_online_v2.active && s_online_v2.abort_requested;
}

void zy100_online_stream_abort_cleanup_ready(uint32_t now_ms)
{
    if ((!s_online_v2.abort_requested && !s_online_v2.end_requested) ||
        s_online_v2.abort_cleanup_ready)
    {
        return;
    }
    s_online_v2.abort_cleanup_ready = true;
    s_online_v2.abort_start_ms = now_ms;
    if (s_online_v2.end_requested &&
        s_online_v2.discard_tail_on_cleanup)
    {
        s_online_v2.tx_active = false;
        online_v2_ack_queue_reset();
        online_v2_tx_cancel_unsent();
    }
}

void zy100_online_stream_request_stop_discard(zy100_online_stop_reason_t reason,
                                              uint32_t now_ms)
{
    zy100_online_stream_request_stop(reason, now_ms);
    if (s_online_v2.active && s_online_v2.end_requested)
    {
        s_online_v2.discard_tail_on_cleanup = true;
    }
}

bool zy100_online_stream_consume_stop_completed(zy100_online_stop_reason_t *reason,
                                                bool *timeout)
{
    if (!s_online_v2.stop_complete_pending)
    {
        return false;
    }
    if (reason != NULL)
    {
        *reason = (zy100_online_stop_reason_t)s_online_v2.stop_complete_reason;
    }
    if (timeout != NULL)
    {
        *timeout = s_online_v2.stop_complete_timeout;
    }
    s_online_v2.stop_complete_pending = false;
    return true;
}

bool zy100_online_stream_quiet_logs_active(void)
{
    return s_online_v2.active || s_online_v2.end_wait_ack;
}
bool zy100_online_stream_capture_stop_requested(void)
{
    return s_online_v2.capture_stop_requested;
}
zy100_online_stop_reason_t zy100_online_stream_capture_stop_reason(void)
{
    return (zy100_online_stop_reason_t)s_online_v2.stop_reason;
}
void zy100_online_stream_capture_stop_issued(void)
{
    s_online_v2.capture_stop_requested = false;
}

void zy100_online_stream_handle_disconnect(uint8_t conn_id, uint32_t now_ms)
{
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    zy100_online_stress_invalidate();
#endif
    (void)now_ms;
    if (s_online_v2.start_preparing && (s_online_v2.conn_id == conn_id))
    {
        zy100_online_spool_start_prepare_abort();
        s_online_v2.start_preparing = false;
        s_online_v2.start_prepare_started = false;
        s_online_v2.start_prepare_ready = false;
    }
    if (s_online_v2.ready_latched && (s_online_v2.conn_id == conn_id))
    {
        s_online_v2.ready_latched = false;
    }
    if (s_online_v2.active && (s_online_v2.conn_id == conn_id))
    {
        online_v2_latch_error(3U);
        online_v2_request_failure(ZY100_ONLINE_STOP_REASON_DISCONNECT);
        s_online_v2.link_disconnected = true;
        online_v2_tx_release_disconnected(conn_id);
    }
}

void zy100_online_stream_handle_transport_unavailable(uint8_t conn_id,
                                                       uint32_t now_ms)
{
    if (s_online_v2.start_preparing && (s_online_v2.conn_id == conn_id))
    {
        zy100_online_spool_start_prepare_abort();
        s_online_v2.start_preparing = false;
        s_online_v2.start_prepare_started = false;
        s_online_v2.start_prepare_ready = false;
    }
    if (s_online_v2.ready_latched && (s_online_v2.conn_id == conn_id))
    {
        s_online_v2.ready_latched = false;
    }
    if (s_online_v2.active && (s_online_v2.conn_id == conn_id))
    {
        online_v2_latch_error(3U);
        online_v2_request_failure(
            ZY100_ONLINE_STOP_REASON_TRANSPORT_UNAVAILABLE);
        s_online_v2.link_disconnected = false;
        s_online_v2.abort_start_ms = now_ms;
    }
}

#if ZY100_ONLINE_RECOVERY_FAULT_INJECT_ENABLE
void zy100_online_stream_test_inject_failure(zy100_online_stop_reason_t reason)
{
    online_v2_request_failure(reason);
}

void zy100_online_stream_test_force_cleanup_error(bool enable)
{
    zy100_online_spool_test_force_cleanup_error(enable);
}
#endif

void zy100_online_stream_handle_record_ack(uint8_t conn_id,
                                           uint32_t session_id,
                                           zy100_online_record_type_t type,
                                           uint32_t record_id,
                                           uint8_t ack_count,
                                           uint8_t status,
                                           uint32_t now_ms)
{
    zy100_online_v2_ack_wait_t *wait;
    uint32_t latency;
    uint32_t lock_state;

    if (!s_online_v2.active || s_online_v2.abort_requested ||
        (conn_id != s_online_v2.conn_id) ||
        (session_id != s_online_v2.session_id) ||
        (status != 0U) || (ack_count != 1U))
    {
        return;
    }
    if (type == ZY100_ONLINE_RECORD_END)
    {
        if (s_online_v2.end_wait_ack)
        {
            s_online_v2.end_wait_ack = false;
            s_online_v2.end_ack_received = true;
            online_v2_note_tx_progress(now_ms);
            s_online_v2.tx_diag.last_record_ack_ms = now_ms;
            s_online_v2.tx_diag.record_ack++;
            online_v2_ack_queue_reset();
            if (online_v2_tx_reuse_safe())
            {
                online_v2_finish_end();
            }
        }
        return;
    }
    wait = online_v2_ack_queue_oldest();
    if ((wait == NULL) ||
        (wait->type != (uint8_t)type) ||
        (wait->record_id != record_id))
    {
        return;
    }
    lock_state = os_lock();
    if (!zy100_online_spool_ack((uint8_t)type, record_id))
    {
        os_unlock(lock_state);
        online_v2_request_failure(ZY100_ONLINE_STOP_REASON_FLASH_IO);
        return;
    }
    os_unlock(lock_state);
    if (!online_v2_elapsed(now_ms, wait->start_ms, &latency))
    {
        latency = 0U;
    }
    if (latency < s_online_v2.ack_latency_min_ms)
    {
        s_online_v2.ack_latency_min_ms = latency;
    }
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
    if (latency > s_online_resource.ack_max_ms)
    {
        s_online_resource.ack_max_ms = latency;
    }
#endif
    if (latency > s_online_v2.ack_latency_max_ms)
    {
        s_online_v2.ack_latency_max_ms = latency;
    }
    s_online_v2.ack_latency_sum_ms += latency;
    s_online_v2.ack_latency_count++;
    online_v2_note_tx_progress(now_ms);
    s_online_v2.tx_diag.last_record_ack_ms = now_ms;
    s_online_v2.tx_diag.record_ack++;
    online_v2_ack_queue_pop();
}

void zy100_online_stream_note_record_produced(zy100_online_record_type_t type,
                                              uint32_t payload_bytes)
{
    (void)type;
    (void)payload_bytes;
}

void zy100_online_stream_note_raw_accepted(uint32_t bscan_id)
{
    s_online_v2.raw_accepted++;
    s_online_v2.raw_last_accepted_bscan = bscan_id;
}

void zy100_online_stream_note_raw_saved(uint32_t bscan_id)
{
    s_online_v2.raw_saved++;
    s_online_v2.raw_last_saved_bscan = bscan_id;
}

bool zy100_online_stream_queue_source_record(zy100_online_record_type_t type,
                                             uint32_t record_id,
                                             uint32_t source_addr,
                                             uint32_t record_bytes,
                                             uint32_t payload_bytes)
{
    (void)type;
    (void)record_id;
    (void)source_addr;
    (void)record_bytes;
    (void)payload_bytes;
    online_v2_latch_error(4U);
    return false;
}

void zy100_online_stream_note_record_dropped(zy100_online_record_type_t type,
                                             const char *reason,
                                             uint32_t detail)
{
    (void)reason;
    zy100_online_stream_note_record_dropped_ex(
        type,
        ZY100_ONLINE_DROP_UNKNOWN,
        detail,
        ZY100_FE_STORE_RESERVE_ERROR,
        0U,
        0U);
}

void zy100_online_stream_note_record_dropped_ex(
    zy100_online_record_type_t type,
    zy100_online_drop_reason_t reason,
    uint32_t detail,
    zy100_fe_store_reserve_result_t reserve_result,
    uint8_t fe_state,
    uint32_t now_ms)
{
    zy100_online_v2_diag_event_t *event;
    zy100_online_stop_reason_t stop_reason;
    uint32_t error;
    uint32_t idx = (uint32_t)type;

    if (idx < ZY100_ONLINE_RECORD_TYPE_COUNT)
    {
        s_online_v2.dropped[idx]++;
    }
    s_online_v2.issue_count++;
    event = &s_online_v2.diag[s_online_v2.diag_next];
    memset(event, 0, sizeof(*event));
    event->ms = now_ms;
    event->detail = detail;
    event->bscan_id = (uint16_t)(detail & 0xFFFFU);
    event->type = (uint8_t)type;
    event->reason = (uint8_t)reason;
    event->reserve_result = (uint8_t)reserve_result;
    event->fe_state = fe_state;
    event->owner = zy100_online_spool_active_reservation_kind();
    event->level = 1U;
    s_online_v2.diag_next = (uint8_t)((s_online_v2.diag_next + 1U) %
                                      ZY100_ONLINE_V2_DIAG_DEPTH);
    if (s_online_v2.diag_count < ZY100_ONLINE_V2_DIAG_DEPTH)
    {
        s_online_v2.diag_count++;
    }
    error = 0x10000UL |
            ((uint32_t)reason << 8) |
            (uint32_t)reserve_result;
    online_v2_latch_error(error);
    if (s_online_v2.active)
    {
        stop_reason = (reason == ZY100_ONLINE_DROP_TARGET_TIMEOUT) ?
                      ZY100_ONLINE_STOP_REASON_SPOOL_FULL :
                      ZY100_ONLINE_STOP_REASON_INTERNAL;
        online_v2_request_failure(stop_reason);
    }
}

void zy100_online_stream_note_gate(zy100_online_gate_reason_t reason)
{
    if (reason < ZY100_ONLINE_GATE_COUNT)
    {
        s_online_v2.gate[reason]++;
    }
}

void zy100_online_stream_note_blind_begin(uint32_t now_ms)
{
    (void)now_ms;
    s_online_v2.blind_count++;
}

void zy100_online_stream_note_blind_end(uint32_t now_ms, uint32_t duration_ms)
{
    (void)now_ms;
    s_online_v2.blind_total_ms += duration_ms;
    if (duration_ms > s_online_v2.blind_max_ms)
    {
        s_online_v2.blind_max_ms = duration_ms;
    }
}

void zy100_online_stream_note_ble_link(uint16_t mtu,
                                       uint16_t conn_interval,
                                       uint8_t phy_tx,
                                       uint8_t phy_rx)
{
    s_online_v2.mtu = mtu;
    s_online_v2.conn_interval = conn_interval;
    /* App's 0 means unknown, not a negotiated PHY update. */
    if (phy_tx != 0U) { s_online_v2.phy_tx = phy_tx; }
    if (phy_rx != 0U) { s_online_v2.phy_rx = phy_rx; }
}

void zy100_online_stream_note_capture_segment(uint32_t now_ms)
{
    uint32_t offset_ms;

    if (!s_online_v2.active || (s_online_v2.session_start_ms > now_ms))
    {
        return;
    }
    s_online_v2.capture_segment_id++;
    if (s_online_v2.capture_segment_id == 0U)
    {
        online_v2_latch_error(9U);
        online_v2_request_failure(ZY100_ONLINE_STOP_REASON_INTERNAL);
        return;
    }
    offset_ms = now_ms - s_online_v2.session_start_ms;
    zy100_online_spool_note_capture_segment(s_online_v2.capture_segment_id,
                                             offset_ms);
}

void zy100_online_stream_set_clock_meta(
    const zy100_online_clock_meta_t *meta)
{
    if (meta == NULL)
    {
        return;
    }
    s_online_v2.clock_meta = *meta;
}

zy100_online_flash_step_result_t zy100_online_stream_commit_step(
    bool safe_window,
    uint32_t now_ms)
{
    zy100_online_spool_pump_result_t result;

    (void)now_ms;
    if (!safe_window)
    {
        return ZY100_ONLINE_FLASH_STEP_BUSY;
    }
#if ZY100_ONLINE_V2_TX_WINDOW_ENABLE
    s_online_v2.tx_window_limit = ZY100_ONLINE_V2_TX_WINDOW_SLOTS;
#else
    s_online_v2.tx_window_limit = 1U;
#endif
    result = zy100_online_spool_commit_pump_once(true);
    if (result == ZY100_ONLINE_SPOOL_PUMP_ERROR)
    {
        online_v2_request_failure(ZY100_ONLINE_STOP_REASON_FLASH_IO);
        return ZY100_ONLINE_FLASH_STEP_ERROR;
    }
    if (result != ZY100_ONLINE_SPOOL_PUMP_IDLE)
    {
        return ZY100_ONLINE_FLASH_STEP_COMMIT_PROGRESS;
    }
    return ZY100_ONLINE_FLASH_STEP_IDLE;
}

zy100_online_flash_step_result_t zy100_online_stream_maintenance_step(
    bool safe_window,
    uint32_t now_ms)
{
    zy100_online_spool_pump_result_t result;

    (void)now_ms;
    if (!safe_window)
    {
        return ZY100_ONLINE_FLASH_STEP_BUSY;
    }
    if (zy100_online_spool_commit_pending() || s_online_v2.tx_active ||
        s_online_v2.end_wait_ack ||
        (s_online_v2.tx_in_flight != 0U))
    {
        return ZY100_ONLINE_FLASH_STEP_BUSY;
    }
    result = zy100_online_spool_erase_reclaimable_step();
    if (result == ZY100_ONLINE_SPOOL_PUMP_ERROR)
    {
        online_v2_request_failure(ZY100_ONLINE_STOP_REASON_FLASH_IO);
        return ZY100_ONLINE_FLASH_STEP_ERROR;
    }
    return (result == ZY100_ONLINE_SPOOL_PUMP_IDLE) ?
           ZY100_ONLINE_FLASH_STEP_IDLE :
           ZY100_ONLINE_FLASH_STEP_ERASE_PROGRESS;
}

void zy100_online_stream_pump_flash(bool safe_window, uint32_t now_ms)
{
    (void)zy100_online_stream_commit_step(safe_window, now_ms);
}

bool zy100_online_stream_pump_a1(uint32_t now_ms)
{
    zy100_online_spool_pump_result_t result;

    (void)now_ms;
#if ZY100_ONLINE_V2_TX_WINDOW_ENABLE
    s_online_v2.tx_window_limit = ZY100_ONLINE_V2_TX_WINDOW_SLOTS;
#else
    s_online_v2.tx_window_limit = 1U;
#endif
    result = zy100_online_spool_commit_pump_once(true);
    if (result == ZY100_ONLINE_SPOOL_PUMP_ERROR)
    {
        online_v2_request_failure(ZY100_ONLINE_STOP_REASON_FLASH_IO);
        return true;
    }
    return result != ZY100_ONLINE_SPOOL_PUMP_IDLE;
}

bool zy100_online_stream_micro_blind_needed(void)
{
    return zy100_online_spool_low_water() &&
           zy100_online_spool_has_reclaimable() &&
           zy100_online_spool_all_acked() &&
           !s_online_v2.tx_active &&
           (s_online_v2.ack_wait_count == 0U) &&
           !s_online_v2.end_wait_ack &&
           (s_online_v2.tx_in_flight == 0U);
}

bool zy100_online_stream_micro_blind_done(uint32_t elapsed_ms)
{
    uint32_t erased = zy100_online_spool_erased_ahead_bytes();

    if (zy100_online_spool_erase_wip())
    {
        return false;
    }
    if (erased >= ZY100_ONLINE_SPOOL_RECOVER_TARGET_BYTES)
    {
        return true;
    }
    if (elapsed_ms < ZY100_ONLINE_MICRO_BLIND_BUDGET_MS)
    {
        return false;
    }
    if (zy100_online_spool_try_wrap_empty(
            ZY100_ONLINE_SPOOL_PREERASE_MIN_BYTES))
    {
        erased = zy100_online_spool_erased_ahead_bytes();
    }
    if ((erased < ZY100_ONLINE_SPOOL_PREERASE_MIN_BYTES) &&
        !zy100_online_spool_has_reclaimable())
    {
        online_v2_latch_error(8U);
        online_v2_request_failure(ZY100_ONLINE_STOP_REASON_SPOOL_FULL);
        return false;
    }
    return erased >= ZY100_ONLINE_SPOOL_PREERASE_MIN_BYTES;
}

#if !ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
bool zy100_online_stream_build_notify(bool safe_window,
                                      uint32_t now_ms,
                                      uint16_t notify_max,
                                      uint8_t *frame,
                                      uint16_t *frame_len)
{
    bool built = false;
    uint8_t frame_type = 0U;
    uint8_t record_type = 0U;
    uint16_t payload_len = 0U;

    s_online_v2.send_call++;
    if ((frame == NULL) || (frame_len == NULL) ||
        !s_online_v2.active || !safe_window)
    {
        return false;
    }
    if (s_online_v2.tx_in_flight >= s_online_v2.tx_window_limit)
    {
        s_online_v2.send_window++;
        return false;
    }
    *frame_len = 0U;
    online_v2_check_ack_timeout(now_ms);
    if (!s_online_v2.active)
    {
        return false;
    }
    if (online_v2_tx_select_ready(notify_max, frame, frame_len))
    {
        s_online_v2.send_ready++;
        return true;
    }
    s_online_v2.send_no_ready++;
    if (s_online_v2.end_wait_ack)
    {
        return false;
    }
    if (s_online_v2.abort_requested &&
        !s_online_v2.abort_cleanup_ready)
    {
        return false;
    }
    if (s_online_v2.abort_requested)
    {
        if (!s_online_v2.link_disconnected && online_v2_tx_reuse_safe())
        {
            built = online_v2_build_abort(notify_max, frame, frame_len);
            frame_type = ZY100_BLE_ONLINE_FRAME_ABORT;
        }
    }
    else if (!s_online_v2.start_sent)
    {
        built = online_v2_build_start(notify_max, frame, frame_len);
        frame_type = ZY100_BLE_ONLINE_FRAME_START;
        payload_len = 28U;
    }
    else if (s_online_v2.end_requested &&
             s_online_v2.abort_cleanup_ready &&
             online_v2_tx_reuse_safe())
    {
        if (online_v2_end_delivery_closed())
        {
            built = online_v2_build_end(notify_max, frame, frame_len);
            frame_type = ZY100_BLE_ONLINE_FRAME_END;
            record_type = ZY100_ONLINE_RECORD_END;
            if (built)
            {
                payload_len = (uint16_t)(*frame_len -
                                          ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES);
            }
        }
        else if (s_online_v2.discard_tail_on_cleanup ||
                 zy100_online_spool_all_acked())
        {
            online_v2_latch_error(16U);
            online_v2_request_failure(ZY100_ONLINE_STOP_REASON_INTERNAL);
        }
    }
    if (built && !online_v2_stage_frame(frame,
                                        *frame_len,
                                        frame_type,
                                        record_type,
                                        payload_len))
    {
        online_v2_latch_error(6U);
        return false;
    }
    if (!built)
    {
        return false;
    }
    online_v2_note_tx_progress(now_ms);
    return online_v2_tx_select_ready(notify_max, frame, frame_len);
}

uint8_t *zy100_online_stream_notify_data(uint8_t *fallback)
{
    uint8_t *data = NULL;
    uint8_t slot_index;
    uint32_t lock_state;

    (void)fallback;
    lock_state = os_lock();
    slot_index = s_online_v2.tx_selected_slot;
    if (slot_index < ZY100_ONLINE_V2_TX_WINDOW_SLOTS)
    {
        zy100_online_v2_tx_slot_t *slot = &s_online_v2.tx_ring[slot_index];

        if ((slot->state == ZY100_ONLINE_V2_TX_SLOT_READY) ||
            (slot->state == ZY100_ONLINE_V2_TX_SLOT_QUARANTINED))
        {
            data = slot->data;
        }
    }
    os_unlock(lock_state);
    return data;
}

#else
bool zy100_online_stream_select_notify(bool safe_window, uint32_t now_ms,
                                       uint16_t notify_max,
                                       zy100_online_notify_view_t *view)
{
    uint8_t index;
    uint8_t frame_type;
    uint8_t record_type = 0U;
    uint16_t len = 0U;
    bool built;
    zy100_online_v2_tx_slot_t *slot;

    s_online_v2.send_call++;
    if ((view == NULL) || !s_online_v2.active || !safe_window) { return false; }
    memset(view, 0, sizeof(*view));
    if (s_online_v2.tx_in_flight >= s_online_v2.tx_window_limit)
    {
        s_online_v2.send_window++;
        return false;
    }
    online_v2_check_ack_timeout(now_ms);
    if (!s_online_v2.active) { return false; }
    if (online_v2_tx_select_view(notify_max, view))
    {
        s_online_v2.send_ready++;
        return true;
    }
    s_online_v2.send_no_ready++;
    if (s_online_v2.end_wait_ack ||
        (s_online_v2.abort_requested && !s_online_v2.abort_cleanup_ready)) { return false; }
    if (s_online_v2.abort_requested)
    {
        if (s_online_v2.link_disconnected || !online_v2_tx_reuse_safe()) { return false; }
        frame_type = ZY100_BLE_ONLINE_FRAME_ABORT;
    }
    else if (!s_online_v2.start_sent) { frame_type = ZY100_BLE_ONLINE_FRAME_START; }
    else if (s_online_v2.end_requested && s_online_v2.abort_cleanup_ready &&
             online_v2_tx_reuse_safe())
    {
        if (!online_v2_end_delivery_closed())
        {
            if (s_online_v2.discard_tail_on_cleanup || zy100_online_spool_all_acked())
            {
                online_v2_latch_error(16U);
                online_v2_request_failure(ZY100_ONLINE_STOP_REASON_INTERNAL);
            }
            return false;
        }
        frame_type = ZY100_BLE_ONLINE_FRAME_END;
        record_type = ZY100_ONLINE_RECORD_END;
    }
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    else if (!s_online_v2.end_requested && zy100_online_stress_status_due())
    { frame_type = ZY100_BLE_ONLINE_FRAME_STRESS; }
#endif
    else { return false; }
    index = online_v2_tx_claim_slot(frame_type, record_type, 0U, 0U, true, 0U, 0U);
    if (index == ZY100_ONLINE_V2_INVALID_SLOT) { online_v2_latch_error(6U); return false; }
    slot = &s_online_v2.tx_ring[index];
    if (frame_type == ZY100_BLE_ONLINE_FRAME_START)
    { built = online_v2_build_start(notify_max, slot->data, &len); }
    else if (frame_type == ZY100_BLE_ONLINE_FRAME_END)
    { built = online_v2_build_end(notify_max, slot->data, &len); }
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    else if (frame_type == ZY100_BLE_ONLINE_FRAME_STRESS)
    { built = online_v2_build_stress(notify_max, slot->data, &len); }
#endif
    else { built = online_v2_build_abort(notify_max, slot->data, &len); }
    if (!built) { online_v2_tx_release_builder(index); return false; }
    slot->payload_len = (frame_type == ZY100_BLE_ONLINE_FRAME_ABORT) ? 0U :
                       (uint16_t)(len - ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES);
    if (!online_v2_tx_publish_slot(index, len))
    { online_v2_latch_error(6U); return false; }
    online_v2_note_tx_progress(now_ms);
    return online_v2_tx_select_view(notify_max, view);
}
#endif

void zy100_online_stream_note_notify_result(uint8_t frame_type,
                                            bool ok,
                                            uint16_t frame_len,
                                            uint16_t payload_len,
                                            uint32_t now_ms)
{
    uint8_t slot_index;
    uint8_t actual_frame_type = 0U;
    uint16_t actual_frame_len = 0U;
    bool logical_valid = false;
    bool queue_error = false;
    bool frame_mismatch = false;
    uint32_t lock_state;

    (void)payload_len;
    s_online_v2.notify_attempts++;
    s_online_v2.tx_diag.last_notify_ms = now_ms;
    lock_state = os_lock();
#if ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
    s_online_v2.tx_view_pending = false;
#endif
    slot_index = s_online_v2.tx_selected_slot;
    s_online_v2.tx_selected_slot = ZY100_ONLINE_V2_INVALID_SLOT;
    if (slot_index >= ZY100_ONLINE_V2_TX_WINDOW_SLOTS)
    {
        os_unlock(lock_state);
        s_online_v2.notify_fail++;
        s_online_v2.tx_diag.notify_fail++;
        return;
    }
    {
        zy100_online_v2_tx_slot_t *slot = &s_online_v2.tx_ring[slot_index];

        actual_frame_type = slot->frame_type;
        actual_frame_len = slot->len;
        frame_mismatch = frame_type != actual_frame_type;
        if ((slot->state != ZY100_ONLINE_V2_TX_SLOT_READY) &&
            (slot->state != ZY100_ONLINE_V2_TX_SLOT_QUARANTINED))
        {
            os_unlock(lock_state);
            s_online_v2.notify_fail++;
            s_online_v2.tx_diag.notify_fail++;
            online_v2_latch_error(12U);
            return;
        }
        if (!ok)
        {
            if (slot->state == ZY100_ONLINE_V2_TX_SLOT_QUARANTINED
#if ZY100_ONLINE_STRESS_TEST_ENABLE
                || actual_frame_type == ZY100_BLE_ONLINE_FRAME_STRESS
#endif
               )
            {
                /* Diagnostic snapshots are rebuilt from the latest sample. */
                online_v2_tx_slot_reset_locked(slot);
            }
            os_unlock(lock_state);
            s_online_v2.notify_fail++;
            s_online_v2.tx_diag.notify_fail++;
            return;
        }
        logical_valid =
            (slot->state == ZY100_ONLINE_V2_TX_SLOT_READY) &&
            s_online_v2.active &&
            (slot->session_generation == s_online_v2.session_generation) &&
            (slot->conn_id == s_online_v2.conn_id);
#if ZY100_ONLINE_STRESS_TEST_ENABLE
        if (logical_valid && actual_frame_type == ZY100_BLE_ONLINE_FRAME_STRESS)
        {
            zy100_online_stress_status_sent(zy100_get_u32_le(slot->data + 20U));
        }
#endif
        slot->completion_pending = true;
        if (logical_valid)
        {
            slot->state = ZY100_ONLINE_V2_TX_SLOT_IN_FLIGHT;
        }
        else
        {
            slot->state = ZY100_ONLINE_V2_TX_SLOT_QUARANTINED;
        }
        if (s_online_v2.tx_complete_count <
            ZY100_ONLINE_V2_TX_WINDOW_SLOTS)
        {
            s_online_v2.tx_complete_slots[s_online_v2.tx_complete_tail] =
                slot_index;
            s_online_v2.tx_complete_tail =
                (uint8_t)((s_online_v2.tx_complete_tail + 1U) %
                          ZY100_ONLINE_V2_TX_WINDOW_SLOTS);
            s_online_v2.tx_complete_count++;
            if (s_online_v2.tx_in_flight < 0xFFU)
            {
                s_online_v2.tx_in_flight++;
            }
        }
        else
        {
            queue_error = true;
        }
        if (logical_valid)
        {
            if (actual_frame_type == ZY100_BLE_ONLINE_FRAME_START)
            {
                s_online_v2.start_sent = true;
            }
            else if (actual_frame_type == ZY100_BLE_ONLINE_FRAME_ABORT)
            {
                s_online_v2.abort_notify_accepted = true;
            }
            else if (actual_frame_type == ZY100_BLE_ONLINE_FRAME_END)
            {
                s_online_v2.end_wait_ack = true;
                if (!online_v2_ack_queue_push(ZY100_ONLINE_RECORD_END,
                                              0U,
                                              0U,
                                              now_ms))
                {
                    queue_error = true;
                }
            }
            else if (actual_frame_type == ZY100_BLE_ONLINE_FRAME_RECORD)
            {
                if (!s_online_v2.tx_active ||
                    (s_online_v2.tx.record_id != slot->record_id) ||
                    (s_online_v2.tx.record_bytes != slot->record_bytes) ||
                    (slot->fragment_offset != s_online_v2.tx_offset) ||
                    (s_online_v2.tx_offset > s_online_v2.tx.record_bytes) ||
                    (slot->fragment_len >
                     (s_online_v2.tx.record_bytes - s_online_v2.tx_offset)))
                {
                    queue_error = true;
                }
                else
                {
                    s_online_v2.tx_offset += slot->fragment_len;
                    if (slot->last_fragment)
                    {
                        if (!zy100_online_spool_mark_tx_sent(
                                slot->record_type,
                                slot->record_id) ||
                            !online_v2_ack_queue_push(slot->record_type,
                                                      slot->record_id,
                                                      slot->record_bytes,
                                                      now_ms))
                        {
                            queue_error = true;
                        }
                        s_online_v2.tx_active = false;
                        s_online_v2.tx_offset = 0U;
                        s_online_v2.tx_stage_offset = 0U;
                    }
                }
            }
        }
    }
    os_unlock(lock_state);
    online_v2_note_tx_progress(now_ms);
    s_online_v2.notify_ok++;
    s_online_v2.tx_diag.notify_ok++;
    online_v2_tx_diag_note_pressure();
    s_online_v2.notify_bytes += actual_frame_len;
    if (logical_valid)
    {
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
        s_online_resource.accepted_bytes += actual_frame_len;
        s_online_resource.window_bytes += actual_frame_len;
#endif
    }
    if (s_online_v2.tx_in_flight > s_online_v2.tx_in_flight_max)
    {
        s_online_v2.tx_in_flight_max = s_online_v2.tx_in_flight;
    }
    if (frame_mismatch)
    {
        online_v2_latch_error(13U);
    }
    if (queue_error)
    {
        online_v2_latch_error(14U);
        online_v2_request_failure(ZY100_ONLINE_STOP_REASON_INTERNAL);
    }
    (void)frame_len;
}

void zy100_online_stream_get_tx_pressure(uint8_t *ready, uint8_t *in_flight)
{
    uint8_t i;
    uint8_t count = 0U;
    uint32_t lock_state = os_lock();

    for (i = 0U; i < ZY100_ONLINE_V2_TX_WINDOW_SLOTS; i++)
    {
        if (s_online_v2.tx_ring[i].state == ZY100_ONLINE_V2_TX_SLOT_READY)
        {
            count++;
        }
    }
    if (ready != NULL)
    {
        *ready = count;
    }
    if (in_flight != NULL)
    {
        *in_flight = s_online_v2.tx_in_flight;
    }
    os_unlock(lock_state);
}

void zy100_online_stream_note_send_complete(uint8_t conn_id,
                                            uint16_t cause,
                                            uint16_t credits)
{
    uint8_t slot_index;
    uint8_t frame_type;
    uint32_t slot_generation;
    uint32_t now_ms;
    bool finish_abort = false;
    bool finish_end = false;
    uint32_t lock_state = os_lock();

    if (s_online_v2.tx_complete_count == 0U)
    {
        os_unlock(lock_state);
        return;
    }
    slot_index = s_online_v2.tx_complete_slots[s_online_v2.tx_complete_head];
    if (slot_index >= ZY100_ONLINE_V2_TX_WINDOW_SLOTS)
    {
        os_unlock(lock_state);
        online_v2_latch_error(15U);
        return;
    }
    {
        zy100_online_v2_tx_slot_t *slot = &s_online_v2.tx_ring[slot_index];

        if (!slot->completion_pending || (slot->conn_id != conn_id))
        {
            os_unlock(lock_state);
            return;
        }
        frame_type = slot->frame_type;
        slot_generation = slot->session_generation;
        s_online_v2.tx_complete_head =
            (uint8_t)((s_online_v2.tx_complete_head + 1U) %
                      ZY100_ONLINE_V2_TX_WINDOW_SLOTS);
        s_online_v2.tx_complete_count--;
        if (s_online_v2.tx_in_flight != 0U)
        {
            s_online_v2.tx_in_flight--;
        }
        online_v2_tx_slot_reset_locked(slot);
    }
    os_unlock(lock_state);
    now_ms = (uint32_t)zy100_os_time_ms();
    online_v2_note_tx_progress(now_ms);
    s_online_v2.tx_diag.last_send_complete_ms = now_ms;
    s_online_v2.tx_diag.send_complete++;
    s_online_v2.tx_diag.send_complete_credits += credits;
    s_online_v2.send_complete_count++;
    s_online_v2.send_complete_credits += credits;
    if (cause != 0U)
    {
        s_online_v2.send_complete_fail++;
        online_v2_latch_error(5U);
        online_v2_request_failure(
            ZY100_ONLINE_STOP_REASON_TX_COMPLETE_FAILED);
        return;
    }
    if (slot_generation == s_online_v2.session_generation)
    {
        finish_abort = (frame_type == ZY100_BLE_ONLINE_FRAME_ABORT) &&
                       s_online_v2.abort_requested &&
                       s_online_v2.abort_notify_accepted &&
                       online_v2_tx_reuse_safe();
        finish_end = s_online_v2.end_ack_received &&
                     online_v2_tx_reuse_safe();
    }
    if (finish_abort)
    {
        online_v2_finish_abort(false);
    }
    else if (finish_end)
    {
        online_v2_finish_end();
    }
}

void zy100_online_stream_poll(uint32_t now_ms)
{
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    if (s_online_v2.active)
    {
        zy100_online_stress_note_wait(s_online_v2.tx_active ||
            s_online_v2.tx_in_flight != 0U, s_online_v2.ack_wait_count != 0U);
    }
#endif
    uint32_t abort_elapsed_ms;
    uint32_t poll_gap_ms;

    if (s_online_v2.active)
    {
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
        online_v2_resource_poll(now_ms);
#endif
    }
    if (s_online_v2.active &&
        online_v2_elapsed(now_ms,
                          s_online_v2.tx_diag.last_poll_ms,
                          &poll_gap_ms) &&
        (poll_gap_ms > s_online_v2.tx_diag.poll_gap_max_ms))
    {
        s_online_v2.tx_diag.poll_gap_max_ms = poll_gap_ms;
    }
    if (s_online_v2.active)
    {
        s_online_v2.tx_diag.last_poll_ms = now_ms;
    }

    online_v2_check_ack_timeout(now_ms);
    online_v2_check_tx_progress_timeout(now_ms);
    if (s_online_v2.active)
    {
        online_v2_tx_diag_note_pressure();
        online_v2_tx_diag_log_periodic(now_ms);
    }
    if (s_online_v2.abort_requested && s_online_v2.abort_cleanup_ready)
    {
        if (s_online_v2.link_disconnected && online_v2_tx_reuse_safe())
        {
            online_v2_finish_abort(false);
        }
        else if (online_v2_elapsed(now_ms,
                                   s_online_v2.abort_start_ms,
                                   &abort_elapsed_ms) &&
                 (abort_elapsed_ms >= ZY100_ONLINE_STREAM_ACK_TIMEOUT_MS))
        {
            online_v2_tx_drop_unsent_terminal();
            if (online_v2_tx_reuse_safe())
            {
                online_v2_finish_abort(true);
            }
        }
    }
    if (s_online_v2.end_ack_received && online_v2_tx_reuse_safe())
    {
        online_v2_finish_end();
    }
#if ZY100_ONLINE_V2_ERROR_LOG_ENABLE
    if (s_online_v2.error_log_pending)
    {
        ZY100_LOG_ERROR("[ONLINE_V2_ERR] stream=%lu",
                   (unsigned long)s_online_v2.last_error);
        s_online_v2.error_log_pending = false;
    }
#endif
    if (!s_online_v2.active && s_online_v2.stop_complete_pending &&
        !s_online_v2.summary_logged)
    {
        zy100_online_stream_log_summary("abort");
    }
}

void zy100_online_stream_log_summary(const char *reason)
{
    (void)reason;
    if (!s_online_v2.summary_logged)
    {
        s_online_v2.summary_pending = true;
    }
}

bool zy100_online_stream_log_summary_step(const char *reason)
{
    zy100_online_spool_stats_t stats;
    uint32_t ack_avg = 0U;
    uint32_t ack_min = 0U;
    uint32_t raw_wait;

    if (s_online_v2.summary_logged)
    {
        return true;
    }
    s_online_v2.summary_pending = true;
    zy100_online_spool_get_stats(&stats);
    if (s_online_v2.ack_latency_count != 0U)
    {
        ack_avg = s_online_v2.ack_latency_sum_ms /
                  s_online_v2.ack_latency_count;
        ack_min = s_online_v2.ack_latency_min_ms;
    }
    raw_wait = stats.reserve_busy[ZY100_FE_STORE_TARGET_RAW] +
               stats.reserve_full[ZY100_FE_STORE_TARGET_RAW];
    switch (s_online_v2.summary_stage)
    {
    case 0U:
        ZY100_LOG_DETAIL("[ONL_D_FLOW] hit=%lu acc=%lu wait=%lu saved=%lu drop=%lu",
               (unsigned long)(s_online_v2.raw_accepted +
                               s_online_v2.dropped[ZY100_ONLINE_RECORD_RAW]),
               (unsigned long)s_online_v2.raw_accepted,
               (unsigned long)raw_wait,
               (unsigned long)s_online_v2.raw_saved,
               (unsigned long)s_online_v2.dropped[ZY100_ONLINE_RECORD_RAW]);
        break;
    case 1U:
        ZY100_LOG_DETAIL("[ONL_D_STORE] p=%lu/%lu/%lu c=%lu/%lu/%lu a=%lu/%lu/%lu",
               (unsigned long)stats.produced[1],
               (unsigned long)stats.produced[2],
               (unsigned long)stats.produced[3],
               (unsigned long)stats.committed[1],
               (unsigned long)stats.committed[2],
               (unsigned long)stats.committed[3],
               (unsigned long)stats.acked[1],
               (unsigned long)stats.acked[2],
               (unsigned long)stats.acked[3]);
        break;
    case 2U:
        ZY100_LOG_DETAIL("[ONL_D_RSV] raw=%lu/%lu/%lu/%lu own=%lu/%lu/%lu",
               (unsigned long)stats.reserve_ok[ZY100_FE_STORE_TARGET_RAW],
               (unsigned long)stats.reserve_busy[ZY100_FE_STORE_TARGET_RAW],
               (unsigned long)stats.reserve_full[ZY100_FE_STORE_TARGET_RAW],
               (unsigned long)stats.reserve_error[ZY100_FE_STORE_TARGET_RAW],
               (unsigned long)stats.reserve_busy_owner[ZY100_FE_STORE_TARGET_RAW],
               (unsigned long)stats.reserve_busy_owner[ZY100_FE_STORE_TARGET_SUMMARY],
               (unsigned long)stats.reserve_busy_owner[ZY100_FE_STORE_TARGET_EVENT]);
        break;
    case 3U:
        ZY100_LOG_DETAIL("[ONL_D_FLASH] crc=%lu/%lu/%lu erase=%lu/%lu/%lu wrap=%lu",
               (unsigned long)stats.crc_pages,
               (unsigned long)stats.crc_total_us,
               (unsigned long)stats.crc_max_us,
               (unsigned long)stats.erase_count,
               (unsigned long)stats.erase_total_ms,
               (unsigned long)stats.erase_max_ms,
               (unsigned long)stats.wrap_count);
        break;
    case 4U:
        ZY100_LOG_DETAIL("[ONL_D_XIO] wip=%lu spi=%lu retry=%lu guard=%lu er=%lu/%lu/%lu",
               (unsigned long)stats.tx_read_wip_blocked,
               (unsigned long)stats.tx_read_spi_busy,
               (unsigned long)stats.tx_read_busy_retry,
               (unsigned long)stats.erase_tx_conflict_prevented,
               (unsigned long)stats.erase_issue_count,
               (unsigned long)stats.erase_complete_count,
               (unsigned long)stats.erase_timeout_count);
        break;
    case 5U:
        if (stats.read_errors || stats.write_errors || stats.erase_errors ||
            stats.state_errors || stats.ack_errors || stats.last_error || stats.erase_timeout_count)
        ZY100_LOG_ERROR("[ONL_D_FERR] r=%lu w=%lu e=%lu st=%lu ack=%lu last=%lu",
               (unsigned long)stats.read_errors,
               (unsigned long)stats.write_errors,
               (unsigned long)stats.erase_errors,
               (unsigned long)stats.state_errors,
               (unsigned long)stats.ack_errors,
               (unsigned long)stats.last_error);
        break;
    case 6U:
        if (stats.last_error || stats.last_error_detail)
        ZY100_LOG_ERROR("[ONL_D_FDET] code=%lu detail=%lu",
                   (unsigned long)stats.last_error,
                   (unsigned long)stats.last_error_detail);
        break;
    case 7U:
        ZY100_LOG_DETAIL("[ONL_D_BLE] bytes=%lu n=%lu/%lu cr=%lu ack=%lu/%lu/%lu to=%lu",
               (unsigned long)s_online_v2.notify_bytes,
               (unsigned long)s_online_v2.notify_ok,
               (unsigned long)s_online_v2.notify_fail,
               (unsigned long)s_online_v2.send_complete_credits,
               (unsigned long)ack_min,
               (unsigned long)ack_avg,
               (unsigned long)s_online_v2.ack_latency_max_ms,
               (unsigned long)s_online_v2.ack_timeout_count);
        break;
    case 8U:
        ZY100_LOG_DETAIL("[ONL_D_TXS] %lu/%lu g=%lu ns=%lu src=%lu fb=%lu",
               (unsigned long)s_online_v2.stage_ok,
               (unsigned long)s_online_v2.stage_call,
               (unsigned long)s_online_v2.stage_gate,
               (unsigned long)s_online_v2.stage_no_slot,
               (unsigned long)s_online_v2.stage_source_wait,
               (unsigned long)s_online_v2.stage_flash_busy);
        break;
    case 9U:
        ZY100_LOG_DETAIL("[ONL_D_TXP] %lu/%lu win=%lu nr=%lu if=%u",
               (unsigned long)s_online_v2.send_ready,
               (unsigned long)s_online_v2.send_call,
               (unsigned long)s_online_v2.send_window,
               (unsigned long)s_online_v2.send_no_ready,
               (uint32_t)s_online_v2.tx_in_flight_max);
        break;
    case 10U:
        ZY100_LOG_DETAIL("[ONL_D_SPOOL] p=%lu/%lu used=%lu gen=%lu/%lu wrap=%lu",
               (unsigned long)stats.pending_records,
               (unsigned long)stats.pending_high_water,
               (unsigned long)stats.logical_used_bytes,
               (unsigned long)stats.reserve_generation,
               (unsigned long)stats.ack_generation,
               (unsigned long)stats.wrap_count);
        break;
    case 11U:
        ZY100_LOG_DETAIL("[ONL_D_RING] era=%lu/%lu rec=%lu block=%lu/%lu gm=%lu",
               (unsigned long)stats.erased_ahead_min_bytes,
               (unsigned long)stats.erased_ahead_bytes,
               (unsigned long)stats.reclaimable_sector_count,
               (unsigned long)stats.capacity_blocked,
               (unsigned long)stats.erase_blocked,
               (unsigned long)stats.generation_mismatch);
        break;
    case 12U:
        if (s_online_v2.abort_cause.valid)
        {
            ZY100_LOG_ERROR("[ONL_CAUSE_A] org=%u det=%lu ms=%lu sr=%lu gen=%lu le=%lu",
                       (uint32_t)s_online_v2.abort_cause.origin,
                       (unsigned long)s_online_v2.abort_cause.detail,
                       (unsigned long)s_online_v2.abort_cause.at_ms,
                       (unsigned long)s_online_v2.abort_cause.stop_reason,
                       (unsigned long)s_online_v2.abort_cause.session_generation,
                       (unsigned long)s_online_v2.abort_cause.last_error);
        }
        break;
    case 13U:
        ZY100_LOG_EVENT("[ONLINE_END] why=%s issue=%lu last=%lu pend=%lu/%lu bytes=%lu notify_fail=%lu ack_to=%lu",
               (reason != NULL) ? reason : "unknown",
               (unsigned long)s_online_v2.issue_count,
               (unsigned long)s_online_v2.last_error,
               (unsigned long)stats.pending_records,
               (unsigned long)stats.pending_bytes,
               (unsigned long)s_online_v2.notify_bytes,
               (unsigned long)s_online_v2.notify_fail,
               (unsigned long)s_online_v2.ack_timeout_count);
        break;
    case 14U:
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
        ZY100_LOG_EVENT("[RES_BLE] ms=%lu accept_Bps=%lu peak_Bps=%lu win_ms=%lu mtu=%u ci=%u phy=%u/%u",
            (unsigned long)s_online_resource.elapsed_ms,
            (unsigned long)online_v2_resource_rate(s_online_resource.accepted_bytes, s_online_resource.elapsed_ms),
            (unsigned long)s_online_resource.peak_Bps,
            (unsigned long)s_online_resource.peak_window_ms,
            s_online_v2.mtu, s_online_v2.conn_interval, s_online_v2.phy_tx, s_online_v2.phy_rx);
#endif
        break;
    case 15U:
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
        ZY100_LOG_EVENT("[RES_MEM] data_min=%lu buffer_min=%lu samples=%lu poll_gap_ms=%lu cpu_pct=NA ble_max=NA",
            (unsigned long)s_online_resource.heap_data_min,
            (unsigned long)s_online_resource.heap_buffer_min,
            (unsigned long)s_online_resource.samples,
            (unsigned long)s_online_resource.poll_gap_ms);
#endif
        break;
    case 16U:
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
        ZY100_LOG_EVENT("[RES_TX] pend_hi=%lu pend_end=%lu cap_block=%lu ack_max_ms=%lu ack_to=%lu fail=%lu",
            (unsigned long)s_online_resource.pending_hi, (unsigned long)s_online_resource.pending_end,
            (unsigned long)s_online_resource.capacity_blocked, (unsigned long)s_online_resource.ack_max_ms,
            (unsigned long)s_online_resource.ack_timeout_start,
            (unsigned long)s_online_resource.fail_start);
#endif
        break;
    case 17U:
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
        ZY100_LOG_EVENT("[RES_FLASH] erase_max_ms=%lu erased_min=%lu errors=%lu/%lu/%lu",
            (unsigned long)s_online_resource.erase_max_ms, (unsigned long)s_online_resource.erased_min,
            (unsigned long)s_online_resource.read_errors, (unsigned long)s_online_resource.write_errors,
            (unsigned long)s_online_resource.erase_errors);
#endif
        break;
    case 18U:
    case 19U:
    case 20U:
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
        zy100_online_raw_capture_log_resource(s_online_v2.summary_stage - 18U);
#endif
        break;
    default:
    {
        uint32_t event_index = (uint32_t)s_online_v2.summary_stage - 21U;
        uint32_t count = (s_online_v2.issue_count > 4U) ? 4U :
                         s_online_v2.issue_count;
        if (event_index < count)
        {
            uint32_t first = (s_online_v2.diag_next +
                              ZY100_ONLINE_V2_DIAG_DEPTH - count) %
                             ZY100_ONLINE_V2_DIAG_DEPTH;
            const zy100_online_v2_diag_event_t *event =
                &s_online_v2.diag[(first + event_index) %
                                  ZY100_ONLINE_V2_DIAG_DEPTH];

            ZY100_LOG_ERROR("[ONL_D_EVT] ms=%lu t=%u r=%u rv=%u fe=%u own=%u d=%lu",
                       (unsigned long)event->ms,
                       event->type,
                       event->reason,
                       event->reserve_result,
                       event->fe_state,
                       event->owner,
                       (unsigned long)event->detail);
        }
        else
        {
#if ZY100_ONLINE_STRESS_TEST_ENABLE
            if (!zy100_stress_diag_log_step(s_online_v2.session_id, event_index - count))
            { s_online_v2.summary_stage++; return false; }
#endif
            s_online_v2.summary_logged = true;
            s_online_v2.summary_pending = false;
            return true;
        }
        break;
    }
    }
    s_online_v2.summary_stage++;
    return false;
}

#endif
