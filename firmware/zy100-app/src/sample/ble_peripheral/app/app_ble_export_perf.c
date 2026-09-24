#include "app/app_ble_export_perf.h"

#include <stddef.h>
#include <string.h>

#include <gap.h>
#include <os_sched.h>
#include <trace.h>

#include "app_flags.h"
#include "bsp/imu_bsp.h"
#include "peripheral_app.h"
#include "service/zy100_ble_ctrl_service.h"

#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
#define APP_BLE_P0_U32_MAX 0xFFFFFFFFUL

typedef struct
{
    uint32_t count;
    uint64_t total;
    uint32_t min;
    uint32_t max;
} app_ble_p0_u32_stat_t;

typedef struct
{
    bool active;
    bool summary_logged;
    uint8_t conn_id;
    uint32_t export_id;
    uint32_t session_uid;
    uint16_t session_index;
    uint16_t session_total;
    uint64_t session_start_us;
    uint64_t prepare_start_us;
    uint64_t prepare_done_us;
    uint64_t producer_begin_start_us;
    uint64_t producer_begin_done_us;
    uint64_t first_notify_us;
    uint64_t first_data_notify_us;
    uint64_t last_notify_us;
    uint64_t sent_done_us;
    uint64_t wait_confirm_start_us;
    uint64_t final_us;
    uint64_t export_start_os_ms;
    uint64_t first_notify_os_ms;
    uint64_t first_data_notify_os_ms;
    uint64_t sent_done_os_ms;
    uint64_t last_data_complete_os_ms;
    uint64_t final_os_ms;
} app_ble_export_p0_time_t;

typedef struct
{
    app_ble_export_p0_time_t time;
    app_ble_export_p0_link_snapshot_t link;
    uint32_t notify_attempts;
    uint32_t notify_ok;
    uint32_t notify_fail;
    uint32_t notify_bytes;
    uint32_t app_payload_bytes;
    uint32_t start_frame_count;
    uint32_t data_frame_count;
    uint32_t end_frame_count;
    uint32_t other_frame_count;
    uint8_t last_fail_stage;
    uint8_t last_fail_type;
    uint16_t last_fail_seq;
    uint16_t last_fail_len;
    uint16_t last_fail_cause;
    uint32_t last_producer_error;
    int32_t last_send_ret;
    uint16_t last_complete_cause;
    app_ble_p0_u32_stat_t notify_len;
    app_ble_p0_u32_stat_t data_payload_len;
    uint16_t feuf_payload_max_snapshot;
    uint32_t agg_enabled;
    uint32_t agg_frame_count;
    uint32_t agg_piece_total;
    uint32_t agg_piece_max;
    uint32_t agg_single_piece_count;
    uint32_t agg_multi_piece_count;
    uint32_t agg_target_hit_count;
    uint32_t agg_short_count;
    uint32_t agg_discard_count;
    uint32_t agg_discard_bytes;
    uint32_t prefetch_discard_read_calls;
    uint32_t prefetch_discard_read_bytes;
    uint32_t producer_state_copy_bytes;
    app_ble_p0_u32_stat_t producer_state_copy_us;
    uint32_t data_payload_le_32;
    uint32_t data_payload_le_64;
    uint32_t data_payload_le_128;
    uint32_t data_payload_le_192;
    uint32_t data_payload_gt_192;
    app_ble_p0_u32_stat_t notify_gap_us;
    app_ble_p0_u32_stat_t send_call_us;
    uint64_t previous_notify_us;
    uint32_t poll_calls;
    uint32_t poll_with_send_count;
    uint32_t poll_without_send_count;
    uint32_t poll_state_not_streaming_count;
    uint32_t poll_blocked_count;
    app_ble_p0_u32_stat_t poll_gap_us;
    app_ble_p0_u32_stat_t frames_per_poll;
    uint64_t previous_poll_us;
    uint32_t peek_calls;
    uint32_t peek_fail;
    app_ble_p0_u32_stat_t peek_us;
    uint32_t commit_calls;
    uint32_t commit_fail;
    app_ble_p0_u32_stat_t commit_us;
    uint32_t send_complete_count;
    uint32_t send_complete_fail;
    uint32_t send_blocked_count;
    uint32_t send_blocked_timeout_count;
    uint16_t send_complete_last_cause;
    uint16_t send_complete_credits_last;
    uint32_t send_complete_credits_total;
    app_ble_p0_u32_stat_t send_complete_credits;
    app_ble_p0_u32_stat_t send_complete_gap_us;
    uint64_t previous_complete_us;
#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
    uint8_t tx_in_flight_max;
    uint16_t tx_budget_max;
    uint32_t tx_no_budget_count;
    uint32_t tx_window_full_count;
    uint32_t tx_no_slot_count;
    uint32_t tx_head_used_count;
    uint32_t tx_ready_event_count;
    uint32_t pump_call_count;
    uint32_t pump_steps_total;
    uint8_t pump_steps_max;
#endif
    zy100_feuf_export_flash_perf_t flash_base_before_begin;
    zy100_feuf_export_flash_perf_t flash_after_begin;
    zy100_feuf_export_flash_perf_t flash_base_stream;
    zy100_feuf_export_flash_perf_t flash_final;
    zy100_feuf_export_flash_perf_t flash_stream;
    zy100_feuf_export_prefetch_perf_t prefetch_final;
} app_ble_export_p0_ctx_t;

static app_ble_export_p0_ctx_t s_ble_export_p0;

uint64_t app_ble_export_p0_now_us(void)
{
    return imu_bsp_local_timestamp_us();
}

uint64_t app_ble_export_p0_now_os_ms(void)
{
    return os_sys_time_get();
}

static void app_ble_p0_stat_reset(app_ble_p0_u32_stat_t *s)
{
    if (s == NULL)
    {
        return;
    }
    s->count = 0U;
    s->total = 0ULL;
    s->min = (uint32_t)APP_BLE_P0_U32_MAX;
    s->max = 0U;
}

static void app_ble_p0_stat_add(app_ble_p0_u32_stat_t *s, uint32_t v)
{
    if (s == NULL)
    {
        return;
    }
    s->count++;
    s->total += v;
    if (v < s->min)
    {
        s->min = v;
    }
    if (v > s->max)
    {
        s->max = v;
    }
}

static uint32_t app_ble_p0_stat_avg(const app_ble_p0_u32_stat_t *s)
{
    if ((s == NULL) || (s->count == 0U))
    {
        return 0U;
    }
    return (uint32_t)(s->total / s->count);
}

static uint32_t app_ble_p0_stat_min(const app_ble_p0_u32_stat_t *s)
{
    if ((s == NULL) || (s->count == 0U))
    {
        return 0U;
    }
    return s->min;
}

static uint32_t app_ble_p0_stat_max(const app_ble_p0_u32_stat_t *s)
{
    if ((s == NULL) || (s->count == 0U))
    {
        return 0U;
    }
    return s->max;
}

static uint32_t app_ble_export_p0_delta_ms(uint64_t start_us,
                                           uint64_t end_us)
    __attribute__((unused));
static uint32_t app_ble_export_p0_delta_ms(uint64_t start_us,
                                           uint64_t end_us)
{
    if ((start_us == 0ULL) || (end_us < start_us))
    {
        return 0U;
    }
    return (uint32_t)((end_us - start_us) / 1000ULL);
}

static uint32_t app_ble_export_p0_delta_os_ms(uint64_t start_ms,
                                              uint64_t end_ms)
    __attribute__((unused));
static uint32_t app_ble_export_p0_delta_os_ms(uint64_t start_ms,
                                              uint64_t end_ms)
{
    uint64_t delta;

    if ((start_ms == 0ULL) || (end_ms < start_ms))
    {
        return 0U;
    }
    delta = end_ms - start_ms;
    return (delta > APP_BLE_P0_U32_MAX) ?
           (uint32_t)APP_BLE_P0_U32_MAX : (uint32_t)delta;
}

static uint32_t app_ble_export_p0_rate_bps(uint32_t bytes,
                                           uint32_t stream_ms)
    __attribute__((unused));
static uint32_t app_ble_export_p0_rate_bps(uint32_t bytes,
                                           uint32_t stream_ms)
{
    uint64_t rate;

    if (stream_ms == 0U)
    {
        return 0U;
    }
    rate = ((uint64_t)bytes * 1000ULL) / stream_ms;
    return (rate > APP_BLE_P0_U32_MAX) ?
           (uint32_t)APP_BLE_P0_U32_MAX : (uint32_t)rate;
}

static void app_ble_export_p0_flash_reset(
    zy100_feuf_export_flash_perf_t *perf)
{
    if (perf != NULL)
    {
        memset(perf, 0, sizeof(*perf));
    }
}

static void app_ble_export_p0_reset_stats(void)
{
    app_ble_p0_stat_reset(&s_ble_export_p0.notify_len);
    app_ble_p0_stat_reset(&s_ble_export_p0.data_payload_len);
    app_ble_p0_stat_reset(&s_ble_export_p0.notify_gap_us);
    app_ble_p0_stat_reset(&s_ble_export_p0.send_call_us);
    app_ble_p0_stat_reset(&s_ble_export_p0.poll_gap_us);
    app_ble_p0_stat_reset(&s_ble_export_p0.frames_per_poll);
    app_ble_p0_stat_reset(&s_ble_export_p0.peek_us);
    app_ble_p0_stat_reset(&s_ble_export_p0.commit_us);
    app_ble_p0_stat_reset(&s_ble_export_p0.producer_state_copy_us);
    app_ble_p0_stat_reset(&s_ble_export_p0.send_complete_credits);
    app_ble_p0_stat_reset(&s_ble_export_p0.send_complete_gap_us);
}

void app_ble_export_p0_reset(const char *reason)
{
    uint64_t now_us = app_ble_export_p0_now_us();

    (void)reason;
    memset(&s_ble_export_p0, 0, sizeof(s_ble_export_p0));
    s_ble_export_p0.time.active = true;
    s_ble_export_p0.time.conn_id = app_ble_export_perf_port_conn_id();
    s_ble_export_p0.time.session_start_us = now_us;
    s_ble_export_p0.time.prepare_start_us = now_us;
    s_ble_export_p0.last_send_ret = -1;
    app_ble_export_p0_reset_stats();
    app_ble_export_p0_flash_reset(&s_ble_export_p0.flash_base_before_begin);
    app_ble_export_p0_flash_reset(&s_ble_export_p0.flash_after_begin);
    app_ble_export_p0_flash_reset(&s_ble_export_p0.flash_base_stream);
    app_ble_export_p0_flash_reset(&s_ble_export_p0.flash_final);
    app_ble_export_p0_flash_reset(&s_ble_export_p0.flash_stream);
    memset(&s_ble_export_p0.prefetch_final, 0,
           sizeof(s_ble_export_p0.prefetch_final));
    s_ble_export_p0.agg_enabled =
        (uint32_t)ZY100_BLE_EXPORT_AGGREGATE_PAYLOAD_ENABLE;
}

void app_ble_export_p0_note_export_start_os(void)
{
    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    s_ble_export_p0.time.export_start_os_ms =
        app_ble_export_p0_now_os_ms();
}

#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
void app_ble_export_p0_note_tx_budget(uint16_t budget)
{
    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    if (budget > s_ble_export_p0.tx_budget_max)
    {
        s_ble_export_p0.tx_budget_max = budget;
    }
}

void app_ble_export_p0_note_tx_in_flight(uint8_t in_flight)
{
    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    if (in_flight > s_ble_export_p0.tx_in_flight_max)
    {
        s_ble_export_p0.tx_in_flight_max = in_flight;
    }
}

void app_ble_export_p0_note_tx_block_reason(uint8_t reason)
{
    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }

    switch (reason)
    {
    case APP_BLE_EXPORT_TX_SUBMIT_NO_BUDGET:
        s_ble_export_p0.tx_no_budget_count++;
        break;
    case APP_BLE_EXPORT_TX_SUBMIT_WINDOW_FULL:
        s_ble_export_p0.tx_window_full_count++;
        break;
    case APP_BLE_EXPORT_TX_SUBMIT_NO_SLOT:
        s_ble_export_p0.tx_no_slot_count++;
        break;
    case APP_BLE_EXPORT_TX_SUBMIT_HEAD_USED:
        s_ble_export_p0.tx_head_used_count++;
        break;
    case APP_BLE_EXPORT_TX_SUBMIT_BACKOFF:
        s_ble_export_p0.tx_no_budget_count++;
        break;
    default:
        break;
    }
}

void app_ble_export_p0_note_tx_ready_event(void)
{
    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    s_ble_export_p0.tx_ready_event_count++;
}

void app_ble_export_p0_note_fast_pump_call(void)
{
    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    s_ble_export_p0.pump_call_count++;
}

void app_ble_export_p0_note_fast_pump_steps(uint8_t steps)
{
    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    s_ble_export_p0.pump_steps_total += steps;
    if (steps > s_ble_export_p0.pump_steps_max)
    {
        s_ble_export_p0.pump_steps_max = steps;
    }
}
#endif

void app_ble_export_p0_set_prepare_start(uint64_t prepare_start_us)
{
    if (!s_ble_export_p0.time.active)
    {
        return;
    }
    s_ble_export_p0.time.session_start_us = prepare_start_us;
    s_ble_export_p0.time.prepare_start_us = prepare_start_us;
}

void app_ble_export_p0_set_session(uint8_t conn_id,
                                          uint32_t export_id,
                                          uint32_t session_uid,
                                          uint16_t session_index,
                                          uint16_t session_total)
{
    if (!s_ble_export_p0.time.active)
    {
        return;
    }
    s_ble_export_p0.time.conn_id = conn_id;
    s_ble_export_p0.time.export_id = export_id;
    s_ble_export_p0.time.session_uid = session_uid;
    s_ble_export_p0.time.session_index = session_index;
    s_ble_export_p0.time.session_total = session_total;
    app_ble_export_p0_get_link_snapshot(conn_id, &s_ble_export_p0.link);
}

void app_ble_export_p0_note_prepare_done(void)
{
    if (s_ble_export_p0.time.active)
    {
        s_ble_export_p0.time.prepare_done_us = app_ble_export_p0_now_us();
    }
}

void app_ble_export_p0_note_begin_start(
    zy100_feuf_export_producer_t *producer)
{
    if (s_ble_export_p0.time.active)
    {
        s_ble_export_p0.time.producer_begin_start_us =
            app_ble_export_p0_now_us();
        zy100_feuf_export_producer_get_flash_perf(
            producer,
            &s_ble_export_p0.flash_base_before_begin);
    }
}

void app_ble_export_p0_note_begin_done(
    zy100_feuf_export_producer_t *producer)
{
    if (!s_ble_export_p0.time.active)
    {
        return;
    }
    s_ble_export_p0.time.producer_begin_done_us =
        app_ble_export_p0_now_us();
    s_ble_export_p0.feuf_payload_max_snapshot =
        zy100_feuf_export_producer_payload_max(producer);
    zy100_feuf_export_producer_get_flash_perf(
        producer,
        &s_ble_export_p0.flash_after_begin);
}

void app_ble_export_p0_note_stream_start(
    zy100_feuf_export_producer_t *producer)
{
    if (!s_ble_export_p0.time.active)
    {
        return;
    }
    zy100_feuf_export_producer_get_flash_perf(
        producer,
        &s_ble_export_p0.flash_base_stream);
    zy100_feuf_export_producer_p0_mark_stream_start(producer);
}

void app_ble_export_p0_record_fail(uint8_t stage,
                                          uint8_t type,
                                          uint16_t seq,
                                          uint16_t len,
                                          uint16_t cause)
{
    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    s_ble_export_p0.last_fail_stage = stage;
    s_ble_export_p0.last_fail_type = type;
    s_ble_export_p0.last_fail_seq = seq;
    s_ble_export_p0.last_fail_len = len;
    s_ble_export_p0.last_fail_cause = cause;
}

void app_ble_export_p0_note_notify_success(uint8_t frame_type,
                                                  uint16_t frame_len,
                                                  uint16_t payload_len)
{
    uint64_t now_us = app_ble_export_p0_now_us();
    uint64_t now_os_ms = app_ble_export_p0_now_os_ms();

    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    s_ble_export_p0.notify_ok++;
    s_ble_export_p0.notify_bytes += frame_len;
    app_ble_p0_stat_add(&s_ble_export_p0.notify_len, frame_len);
    if (s_ble_export_p0.time.first_notify_us == 0ULL)
    {
        s_ble_export_p0.time.first_notify_us = now_us;
        s_ble_export_p0.time.first_notify_os_ms = now_os_ms;
    }
    if (s_ble_export_p0.previous_notify_us != 0ULL)
    {
        app_ble_p0_stat_add(
            &s_ble_export_p0.notify_gap_us,
            (uint32_t)(now_us - s_ble_export_p0.previous_notify_us));
    }
    s_ble_export_p0.previous_notify_us = now_us;
    s_ble_export_p0.time.last_notify_us = now_us;
    switch (frame_type)
    {
    case APP_BLE_EXPORT_FRAME_START:
        s_ble_export_p0.start_frame_count++;
        break;
    case APP_BLE_EXPORT_FRAME_DATA:
        s_ble_export_p0.data_frame_count++;
        s_ble_export_p0.app_payload_bytes += payload_len;
        if (s_ble_export_p0.time.first_data_notify_us == 0ULL)
        {
            s_ble_export_p0.time.first_data_notify_us = now_us;
            s_ble_export_p0.time.first_data_notify_os_ms = now_os_ms;
        }
        s_ble_export_p0.time.sent_done_us = now_us;
        s_ble_export_p0.time.sent_done_os_ms = now_os_ms;
        break;
    case APP_BLE_EXPORT_FRAME_END:
        s_ble_export_p0.end_frame_count++;
        break;
    default:
        s_ble_export_p0.other_frame_count++;
        break;
    }
}

void app_ble_export_p0_note_send_blocked(uint8_t frame_type,
                                                uint16_t chunk_seq,
                                                uint16_t frame_len,
                                                bool timeout)
{
    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    s_ble_export_p0.send_blocked_count++;
    if (timeout)
    {
        s_ble_export_p0.send_blocked_timeout_count++;
        app_ble_export_p0_record_fail(APP_BLE_P0_FAIL_STAGE_SEND,
                                      frame_type,
                                      chunk_seq,
                                      frame_len,
                                      0U);
    }
}

void app_ble_export_p0_note_aggregate_discard(
    const zy100_feuf_export_aggregate_plan_t *plan)
{
    if (!s_ble_export_p0.time.active ||
        s_ble_export_p0.time.summary_logged ||
        (plan == NULL) ||
        !plan->valid)
    {
        return;
    }
    s_ble_export_p0.agg_discard_count++;
    s_ble_export_p0.agg_discard_bytes += plan->len;
    s_ble_export_p0.prefetch_discard_read_calls +=
        plan->prefetch_read_calls_delta;
    s_ble_export_p0.prefetch_discard_read_bytes +=
        plan->prefetch_read_bytes_delta;
}

void app_ble_export_p0_note_producer_state_copy(uint32_t bytes,
                                                       uint32_t elapsed_us)
{
    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    s_ble_export_p0.producer_state_copy_bytes += bytes;
    app_ble_p0_stat_add(&s_ble_export_p0.producer_state_copy_us, elapsed_us);
}

void app_ble_export_p0_note_data_complete_ok(uint64_t now_os_ms)
{
    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    s_ble_export_p0.time.last_data_complete_os_ms = now_os_ms;
}

void app_ble_export_p0_note_data_payload_success(uint16_t payload_len,
                                                        uint8_t piece_count)
{
    uint32_t use_piece_count = (piece_count == 0U) ? 1U : piece_count;

    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    s_ble_export_p0.agg_frame_count++;
    s_ble_export_p0.agg_piece_total += use_piece_count;
    if (use_piece_count > s_ble_export_p0.agg_piece_max)
    {
        s_ble_export_p0.agg_piece_max = use_piece_count;
    }
    if (use_piece_count > 1U)
    {
        s_ble_export_p0.agg_multi_piece_count++;
    }
    else
    {
        s_ble_export_p0.agg_single_piece_count++;
    }
    if (payload_len >= (uint16_t)ZY100_BLE_EXPORT_AGGREGATE_TARGET_BYTES)
    {
        s_ble_export_p0.agg_target_hit_count++;
    }
    else
    {
        s_ble_export_p0.agg_short_count++;
    }
    app_ble_p0_stat_add(&s_ble_export_p0.data_payload_len, payload_len);
    if (payload_len <= 32U)
    {
        s_ble_export_p0.data_payload_le_32++;
    }
    else if (payload_len <= 64U)
    {
        s_ble_export_p0.data_payload_le_64++;
    }
    else if (payload_len <= 128U)
    {
        s_ble_export_p0.data_payload_le_128++;
    }
    else if (payload_len <= 192U)
    {
        s_ble_export_p0.data_payload_le_192++;
    }
    else
    {
        s_ble_export_p0.data_payload_gt_192++;
    }
}

void app_ble_export_p0_note_wait_confirm(void)
{
    uint64_t now_us = app_ble_export_p0_now_us();

    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    s_ble_export_p0.time.wait_confirm_start_us = now_us;
}

void app_ble_export_p0_mark_final(void)
{
    if (!s_ble_export_p0.time.active ||
        s_ble_export_p0.time.summary_logged ||
        (s_ble_export_p0.time.final_us != 0ULL))
    {
        return;
    }
    s_ble_export_p0.time.final_us = app_ble_export_p0_now_us();
    s_ble_export_p0.time.final_os_ms = app_ble_export_p0_now_os_ms();
}

void app_ble_export_p0_poll_enter(bool *tracked,
                                         uint32_t *notify_ok_before)
{
    uint64_t now_us;

    if (tracked != NULL)
    {
        *tracked = false;
    }
    if (notify_ok_before != NULL)
    {
        *notify_ok_before = s_ble_export_p0.notify_ok;
    }
    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    now_us = app_ble_export_p0_now_us();
    s_ble_export_p0.poll_calls++;
    if (s_ble_export_p0.previous_poll_us != 0ULL)
    {
        app_ble_p0_stat_add(
            &s_ble_export_p0.poll_gap_us,
            (uint32_t)(now_us - s_ble_export_p0.previous_poll_us));
    }
    s_ble_export_p0.previous_poll_us = now_us;
    if (!app_ble_export_perf_port_streaming())
    {
        s_ble_export_p0.poll_state_not_streaming_count++;
    }
    if (tracked != NULL)
    {
        *tracked = true;
    }
}

void app_ble_export_p0_poll_exit(bool tracked,
                                        uint32_t notify_ok_before)
{
    uint32_t frames;

    if (!tracked ||
        !s_ble_export_p0.time.active ||
        s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    frames = s_ble_export_p0.notify_ok - notify_ok_before;
    app_ble_p0_stat_add(&s_ble_export_p0.frames_per_poll, frames);
    if (frames != 0U)
    {
        s_ble_export_p0.poll_with_send_count++;
    }
    else
    {
        s_ble_export_p0.poll_without_send_count++;
    }
}

static void app_ble_export_p0_log_stat(const char *name,
                                       const app_ble_p0_u32_stat_t *stat)
    __attribute__((unused));
static void app_ble_export_p0_log_stat(const char *name,
                                       const app_ble_p0_u32_stat_t *stat)
{
    DBG_DIRECT("[BLE_P0] %s count=%lu total=%llu",
               name,
               (unsigned long)((stat != NULL) ? stat->count : 0U),
               (unsigned long long)((stat != NULL) ? stat->total : 0ULL));
    DBG_DIRECT("[BLE_P0] %s min=%lu max=%lu",
               name,
               (unsigned long)app_ble_p0_stat_min(stat),
               (unsigned long)app_ble_p0_stat_max(stat));
    DBG_DIRECT("[BLE_P0] %s avg=%lu",
               name,
               (unsigned long)app_ble_p0_stat_avg(stat));
}

static void app_ble_export_p0_log_flash(
    const char *name,
    const zy100_feuf_export_flash_perf_t *perf) __attribute__((unused));
static void app_ble_export_p0_log_flash(
    const char *name,
    const zy100_feuf_export_flash_perf_t *perf)
{
    uint32_t avg = 0U;

    if ((perf != NULL) && (perf->flash_read_calls != 0U))
    {
        avg = (uint32_t)(perf->flash_read_total_us /
                         perf->flash_read_calls);
    }
    DBG_DIRECT("[BLE_P0] %s calls=%lu fail=%lu",
               name,
               (unsigned long)((perf != NULL) ? perf->flash_read_calls : 0U),
               (unsigned long)((perf != NULL) ?
                               perf->flash_read_fail_count : 0U));
    DBG_DIRECT("[BLE_P0] %s bytes=%lu",
               name,
               (unsigned long)((perf != NULL) ? perf->flash_read_bytes : 0U));
    DBG_DIRECT("[BLE_P0] %s_us total=%llu",
               name,
               (unsigned long long)((perf != NULL) ?
                                    perf->flash_read_total_us : 0ULL));
    DBG_DIRECT("[BLE_P0] %s_us min=%lu max=%lu",
               name,
               (unsigned long)((perf != NULL) ?
                               perf->flash_read_min_us : 0U),
               (unsigned long)((perf != NULL) ?
                               perf->flash_read_max_us : 0U));
    DBG_DIRECT("[BLE_P0] %s_us avg=%lu", name, (unsigned long)avg);
}

void app_ble_export_p0_log_summary(const char *result)
{
#if !ZY100_BLE_EXPORT_DETAIL_LOG_ENABLE
    uint32_t total_ms;
    uint32_t stream_ms;
    uint32_t wait_ms;
    uint32_t app_bps;
    uint32_t flash_avg_us = 0U;
    uint64_t stream_start_us;
    const char *use_result = (result != NULL) ? result : "unknown";

    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    if (s_ble_export_p0.time.final_us == 0ULL)
    {
        s_ble_export_p0.time.final_us = app_ble_export_p0_now_us();
    }
    if (s_ble_export_p0.time.final_os_ms == 0ULL)
    {
        s_ble_export_p0.time.final_os_ms = app_ble_export_p0_now_os_ms();
    }
    s_ble_export_p0.time.summary_logged = true;
    s_ble_export_p0.time.active = false;
    zy100_feuf_export_producer_get_flash_perf(
        app_ble_export_perf_port_producer(),
        &s_ble_export_p0.flash_final);
    zy100_feuf_export_producer_get_flash_stream_perf(
        app_ble_export_perf_port_producer(),
        &s_ble_export_p0.flash_stream);
    zy100_feuf_export_producer_get_prefetch_perf(
        app_ble_export_perf_port_producer(),
        &s_ble_export_p0.prefetch_final);
    app_ble_export_p0_get_link_snapshot(s_ble_export_p0.time.conn_id,
                                        &s_ble_export_p0.link);

    total_ms = app_ble_export_p0_delta_ms(
        s_ble_export_p0.time.session_start_us,
        s_ble_export_p0.time.final_us);
    stream_start_us = (s_ble_export_p0.time.first_data_notify_us != 0ULL) ?
                      s_ble_export_p0.time.first_data_notify_us :
                      s_ble_export_p0.time.first_notify_us;
    stream_ms = app_ble_export_p0_delta_ms(
        stream_start_us,
        s_ble_export_p0.time.sent_done_us);
    wait_ms = app_ble_export_p0_delta_ms(
        s_ble_export_p0.time.wait_confirm_start_us,
        s_ble_export_p0.time.final_us);
    app_bps = app_ble_export_p0_rate_bps(
        s_ble_export_p0.app_payload_bytes,
        stream_ms);
    if (s_ble_export_p0.flash_final.flash_read_calls != 0U)
    {
        flash_avg_us = (uint32_t)(
            s_ble_export_p0.flash_final.flash_read_total_us /
            s_ble_export_p0.flash_final.flash_read_calls);
    }

    DBG_DIRECT("[BLE_P0] result=%s id=%lu uid=%lu sess=%u/%u",
               use_result,
               (unsigned long)s_ble_export_p0.time.export_id,
               (unsigned long)s_ble_export_p0.time.session_uid,
               s_ble_export_p0.time.session_index,
               s_ble_export_p0.time.session_total);
    DBG_DIRECT("[BLE_P0] time total_ms=%lu stream_ms=%lu wait_ms=%lu rate=%lu",
               (unsigned long)total_ms,
               (unsigned long)stream_ms,
               (unsigned long)wait_ms,
               (unsigned long)app_bps);
    DBG_DIRECT("[BLE_P0] link mtu=%u ci=%u lat=%u phy=%u/%u dle=%u",
               s_ble_export_p0.link.mtu_last,
               s_ble_export_p0.link.conn_interval_last,
               s_ble_export_p0.link.conn_latency_last,
               s_ble_export_p0.link.phy_tx_last,
               s_ble_export_p0.link.phy_rx_last,
               s_ble_export_p0.link.dle_tx_octets_last);
    DBG_DIRECT("[BLE_P0] tx attempts=%lu ok=%lu fail=%lu bytes=%lu blocked=%lu timeout=%lu inflight=%u",
               (unsigned long)s_ble_export_p0.notify_attempts,
               (unsigned long)s_ble_export_p0.notify_ok,
               (unsigned long)s_ble_export_p0.notify_fail,
               (unsigned long)s_ble_export_p0.notify_bytes,
               (unsigned long)s_ble_export_p0.send_blocked_count,
               (unsigned long)s_ble_export_p0.send_blocked_timeout_count,
               s_ble_export_p0.tx_in_flight_max);
    DBG_DIRECT("[BLE_P0] flash calls=%lu fail=%lu bytes=%lu avg=%lu max=%lu prefetch=%lu",
               (unsigned long)s_ble_export_p0.flash_final.flash_read_calls,
               (unsigned long)s_ble_export_p0.flash_final.flash_read_fail_count,
               (unsigned long)s_ble_export_p0.flash_final.flash_read_bytes,
               (unsigned long)flash_avg_us,
               (unsigned long)s_ble_export_p0.flash_final.flash_read_max_us,
               (unsigned long)s_ble_export_p0.prefetch_final.copied_bytes);
    DBG_DIRECT("[BLE_P0] last_fail stage=%u cause=0x%04x type=%u seq=%u producer=%lu complete=0x%04x",
               s_ble_export_p0.last_fail_stage,
               s_ble_export_p0.last_fail_cause,
               s_ble_export_p0.last_fail_type,
               s_ble_export_p0.last_fail_seq,
               (unsigned long)s_ble_export_p0.last_producer_error,
               s_ble_export_p0.last_complete_cause);
#else
    uint32_t total_ms;
    uint32_t prepare_ms;
    uint32_t begin_ms;
    uint32_t stream_ms;
    uint32_t wait_ms;
    uint64_t stream_start_us;
    uint32_t app_bps;
    uint32_t notify_bps;
    uint32_t stream_os_ms;
    uint32_t export_to_data_ms;
    uint32_t data_to_sent_ms;
    uint32_t app_bps_os;
    uint32_t notify_bps_os;
    uint32_t effective_bytes;
    uint32_t feuf_stream_bytes;
    uint32_t notify_bytes;
    uint32_t complete_os_ms;
    uint32_t experience_os_ms;
    uint32_t feuf_effective_bps_os;
    uint32_t feuf_effective_bps_complete_os;
    uint32_t experience_bps_os;
    uint32_t clock_ratio_x1000;
    uint32_t prefetch_total = 0U;
    uint32_t prefetch_hit_rate_x1000 = 0U;
    const char *use_result = (result != NULL) ? result : "unknown";

    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    if (s_ble_export_p0.time.final_us == 0ULL)
    {
        s_ble_export_p0.time.final_us = app_ble_export_p0_now_us();
    }
    if (s_ble_export_p0.time.final_os_ms == 0ULL)
    {
        s_ble_export_p0.time.final_os_ms = app_ble_export_p0_now_os_ms();
    }
    s_ble_export_p0.time.summary_logged = true;
    s_ble_export_p0.time.active = false;
    zy100_feuf_export_producer_get_flash_perf(
        &s_ble_export.producer,
        &s_ble_export_p0.flash_final);
    zy100_feuf_export_producer_get_flash_stream_perf(
        &s_ble_export.producer,
        &s_ble_export_p0.flash_stream);
    zy100_feuf_export_producer_get_prefetch_perf(
        &s_ble_export.producer,
        &s_ble_export_p0.prefetch_final);
    app_ble_export_p0_get_link_snapshot(s_ble_export_p0.time.conn_id,
                                        &s_ble_export_p0.link);

    total_ms = app_ble_export_p0_delta_ms(
        s_ble_export_p0.time.session_start_us,
        s_ble_export_p0.time.final_us);
    prepare_ms = app_ble_export_p0_delta_ms(
        s_ble_export_p0.time.prepare_start_us,
        s_ble_export_p0.time.prepare_done_us);
    begin_ms = app_ble_export_p0_delta_ms(
        s_ble_export_p0.time.producer_begin_start_us,
        s_ble_export_p0.time.producer_begin_done_us);
    stream_start_us = (s_ble_export_p0.time.first_data_notify_us != 0ULL) ?
                      s_ble_export_p0.time.first_data_notify_us :
                      s_ble_export_p0.time.first_notify_us;
    stream_ms = app_ble_export_p0_delta_ms(
        stream_start_us,
        s_ble_export_p0.time.sent_done_us);
    wait_ms = app_ble_export_p0_delta_ms(
        s_ble_export_p0.time.wait_confirm_start_us,
        s_ble_export_p0.time.final_us);
    app_bps = app_ble_export_p0_rate_bps(
        s_ble_export_p0.app_payload_bytes,
        stream_ms);
    notify_bps = app_ble_export_p0_rate_bps(
        s_ble_export_p0.notify_bytes,
        stream_ms);
    stream_os_ms = app_ble_export_p0_delta_os_ms(
        s_ble_export_p0.time.first_data_notify_os_ms,
        s_ble_export_p0.time.sent_done_os_ms);
    export_to_data_ms = app_ble_export_p0_delta_os_ms(
        s_ble_export_p0.time.export_start_os_ms,
        s_ble_export_p0.time.first_data_notify_os_ms);
    data_to_sent_ms = app_ble_export_p0_delta_os_ms(
        s_ble_export_p0.time.first_data_notify_os_ms,
        s_ble_export_p0.time.sent_done_os_ms);
    app_bps_os = app_ble_export_p0_rate_bps(
        s_ble_export_p0.app_payload_bytes,
        stream_os_ms);
    notify_bps_os = app_ble_export_p0_rate_bps(
        s_ble_export_p0.notify_bytes,
        stream_os_ms);
    effective_bytes = zy100_feuf_export_producer_payload_bytes_sent(
        &s_ble_export.producer);
    feuf_stream_bytes = s_ble_export.total_bytes;
    notify_bytes = s_ble_export_p0.notify_bytes;
    complete_os_ms = app_ble_export_p0_delta_os_ms(
        s_ble_export_p0.time.first_data_notify_os_ms,
        s_ble_export_p0.time.last_data_complete_os_ms);
    experience_os_ms = app_ble_export_p0_delta_os_ms(
        s_ble_export_p0.time.export_start_os_ms,
        s_ble_export_p0.time.final_os_ms);
    feuf_effective_bps_os = app_ble_export_p0_rate_bps(
        effective_bytes,
        stream_os_ms);
    feuf_effective_bps_complete_os = app_ble_export_p0_rate_bps(
        effective_bytes,
        complete_os_ms);
    experience_bps_os = app_ble_export_p0_rate_bps(
        effective_bytes,
        experience_os_ms);
    clock_ratio_x1000 = (stream_os_ms != 0U) ?
                        (uint32_t)(((uint64_t)stream_ms * 1000ULL) /
                                   stream_os_ms) : 0U;
    prefetch_total = s_ble_export_p0.prefetch_final.hit_count +
                     s_ble_export_p0.prefetch_final.miss_count +
                     s_ble_export_p0.prefetch_final.bypass_count;
    if (prefetch_total != 0U)
    {
        prefetch_hit_rate_x1000 =
            (uint32_t)(((uint64_t)s_ble_export_p0.prefetch_final.hit_count *
                        1000ULL) / prefetch_total);
    }

    DBG_DIRECT("[P0] r=%s id=%lu uid=%lu | [P0] bt=%s",
               use_result,
               (unsigned long)s_ble_export_p0.time.export_id,
               (unsigned long)s_ble_export_p0.time.session_uid,
               ZY100_BLE_EXPORT_STEP2_BUILD_TAG);
    DBG_DIRECT("[P0] sess i=%u t=%u conn=%u | [P0] time tm=%lu sm=%lu",
               s_ble_export_p0.time.session_index,
               s_ble_export_p0.time.session_total,
               s_ble_export_p0.time.conn_id,
               (unsigned long)total_ms,
               (unsigned long)stream_ms);
    DBG_DIRECT("[P0] time pm=%lu bm=%lu | [P0] time wm=%lu | [P0] os es=%llu sd=%llu | [P0] os fn=%llu",
               (unsigned long)prepare_ms,
               (unsigned long)begin_ms, (unsigned long)wait_ms,
               (unsigned long long)s_ble_export_p0.time.export_start_os_ms,
               (unsigned long long)s_ble_export_p0.time.sent_done_os_ms,
               (unsigned long long)s_ble_export_p0.time.first_notify_os_ms);
    DBG_DIRECT("[P0] os fd=%llu | [P0] os fo=%llu stream_os_ms=%lu | [P0] os last_data_cm=%llu cm=%lu | [P0] clock_cal p0_sm=%lu os_sm=%lu rx=%lu",
               (unsigned long long)s_ble_export_p0.time.first_data_notify_os_ms,
               (unsigned long long)s_ble_export_p0.time.final_os_ms,
               (unsigned long)stream_os_ms,
               (unsigned long long)s_ble_export_p0.time.last_data_complete_os_ms,
               (unsigned long)complete_os_ms,
               (unsigned long)stream_ms,
               (unsigned long)stream_os_ms,
               (unsigned long)clock_ratio_x1000);
    DBG_DIRECT("[P0] os ed=%lu ds=%lu | [P0] link ms=%u ml=%u | [P0] link mu=%lu | [P0] link nm=%u ch=%u",
               (unsigned long)export_to_data_ms,
               (unsigned long)data_to_sent_ms,
               s_ble_export_p0.link.mtu_at_start,
               s_ble_export_p0.link.mtu_last,
               (unsigned long)s_ble_export_p0.link.mtu_update_count,
               s_ble_export_p0.link.notify_max,
               s_ble_export_p0.link.chunk_payload);
    DBG_DIRECT("[P0] conn cs=0x%04x cl=0x%04x | [P0] conn lat=%u to=0x%04x | [P0] conn up=%lu",
               s_ble_export_p0.link.conn_interval_at_start,
               s_ble_export_p0.link.conn_interval_last,
               s_ble_export_p0.link.conn_latency_last,
               s_ble_export_p0.link.conn_timeout_last,
               (unsigned long)s_ble_export_p0.link.conn_update_count);
#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
    app_ble_export_fast_conn_p0_log_summary();
#endif
    DBG_DIRECT("[P0] dle xo=%u xt=%u | [P0] dle up=%lu",
               s_ble_export_p0.link.dle_tx_octets_last,
               s_ble_export_p0.link.dle_max_tx_time_last,
               (unsigned long)s_ble_export_p0.link.dle_update_count);
    DBG_DIRECT("[P0] phy tx=%u rx=%u up=%lu | [P0] tx a=%lu ok=%lu f=%lu",
               s_ble_export_p0.link.phy_tx_last,
               s_ble_export_p0.link.phy_rx_last,
               (unsigned long)s_ble_export_p0.link.phy_update_count,
               (unsigned long)s_ble_export_p0.notify_attempts,
               (unsigned long)s_ble_export_p0.notify_ok,
               (unsigned long)s_ble_export_p0.notify_fail);
    DBG_DIRECT("[P0] tx notify_b=%lu ab=%lu | [P0] bytes fe=%lu fs=%lu | [P0] bytes notify=%lu | [P0] tx fr start=%lu data=%lu",
               (unsigned long)s_ble_export_p0.notify_bytes,
               (unsigned long)s_ble_export_p0.app_payload_bytes,
               (unsigned long)effective_bytes,
               (unsigned long)feuf_stream_bytes,
               (unsigned long)notify_bytes,
               (unsigned long)s_ble_export_p0.start_frame_count,
               (unsigned long)s_ble_export_p0.data_frame_count);
    DBG_DIRECT("[P0] tx fr end=%lu other=%lu | [P0] feuf_payload_x=%u | [P0] feuf_fit_ble=%u | [P0] agg en=%lu frames=%lu",
               (unsigned long)s_ble_export_p0.end_frame_count,
               (unsigned long)s_ble_export_p0.other_frame_count,
               (uint32_t)s_ble_export_p0.feuf_payload_max_snapshot,
               (uint32_t)ZY100_BLE_EXPORT_FEUF_PAYLOAD_FIT_BLE_ENABLE,
               (unsigned long)s_ble_export_p0.agg_enabled,
               (unsigned long)s_ble_export_p0.agg_frame_count);
#if ZY100_BLE_EXPORT_AGGREGATE_PAYLOAD_ENABLE
    DBG_DIRECT("[P0] agg mode=stream_plan target=%u",
               (uint32_t)ZY100_BLE_EXPORT_AGGREGATE_TARGET_BYTES);
#else
    DBG_DIRECT("[P0] agg mode=disabled");
#endif
    DBG_DIRECT("[P0] agg pieces_t=%lu x=%lu | [P0] agg single=%lu multi=%lu | [P0] agg th=%lu short=%lu | [P0] agg discard_c=%lu discard_b=%lu",
               (unsigned long)s_ble_export_p0.agg_piece_total,
               (unsigned long)s_ble_export_p0.agg_piece_max,
               (unsigned long)s_ble_export_p0.agg_single_piece_count,
               (unsigned long)s_ble_export_p0.agg_multi_piece_count,
               (unsigned long)s_ble_export_p0.agg_target_hit_count,
               (unsigned long)s_ble_export_p0.agg_short_count,
               (unsigned long)s_ble_export_p0.agg_discard_count,
               (unsigned long)s_ble_export_p0.agg_discard_bytes);
    DBG_DIRECT("[P0] prefetch_discard c=%lu b=%lu",
               (unsigned long)s_ble_export_p0.prefetch_discard_read_calls,
               (unsigned long)s_ble_export_p0.prefetch_discard_read_bytes);
    app_ble_export_p0_log_stat("data_payload",
                               &s_ble_export_p0.data_payload_len);
    DBG_DIRECT("[P0] data_payload le32=%lu le64=%lu | [P0] data_payload le128=%lu le192=%lu | [P0] data_payload gt192=%lu | [P0] rate app_Bps=%lu",
               (unsigned long)s_ble_export_p0.data_payload_le_32,
               (unsigned long)s_ble_export_p0.data_payload_le_64,
               (unsigned long)s_ble_export_p0.data_payload_le_128,
               (unsigned long)s_ble_export_p0.data_payload_le_192,
               (unsigned long)s_ble_export_p0.data_payload_gt_192,
               (unsigned long)app_bps);
    DBG_DIRECT("[P0] rate notify_Bps=%lu | [P0] rate_os app_Bps_os=%lu notify_Bps_os=%lu | [P0] rate_os feuf_effective_Bps_os=%lu | [P0] rate_os feuf_effective_Bps_complete_os=%lu",
               (unsigned long)notify_bps,
               (unsigned long)app_bps_os,
               (unsigned long)notify_bps_os,
               (unsigned long)feuf_effective_bps_os,
               (unsigned long)feuf_effective_bps_complete_os);
    DBG_DIRECT("[P0] rate_os experience_Bps_os=%lu",
               (unsigned long)experience_bps_os);
#if ZY100_BLE_EXPORT_FAST_PUMP_ENABLE
    DBG_DIRECT("[P0] fast_window_cfg=%u fast_ring_cfg=%u | [P0] tx_in_flight_x=%u tx_budget_x=%u | [P0] tx_no_budget_c=%lu tx_window_full_c=%lu | [P0] tx_no_slot_c=%lu tx_head_used_c=%lu",
               (uint32_t)ZY100_BLE_EXPORT_TX_WINDOW,
               (uint32_t)ZY100_BLE_EXPORT_TX_RING_DEPTH,
               s_ble_export_p0.tx_in_flight_max,
               s_ble_export_p0.tx_budget_max,
               (unsigned long)s_ble_export_p0.tx_no_budget_count,
               (unsigned long)s_ble_export_p0.tx_window_full_count,
               (unsigned long)s_ble_export_p0.tx_no_slot_count,
               (unsigned long)s_ble_export_p0.tx_head_used_count);
    DBG_DIRECT("[P0] tx_ready_event_c=%lu pump_call_c=%lu | [P0] pump_steps_t=%lu pump_steps_x=%u",
               (unsigned long)s_ble_export_p0.tx_ready_event_count,
               (unsigned long)s_ble_export_p0.pump_call_count,
               (unsigned long)s_ble_export_p0.pump_steps_total,
               s_ble_export_p0.pump_steps_max);
#endif
    app_ble_export_p0_log_stat("notify_len", &s_ble_export_p0.notify_len);
    app_ble_export_p0_log_stat("notify_gap", &s_ble_export_p0.notify_gap_us);
    DBG_DIRECT("[P0] poll c=%lu with_send=%lu | [P0] poll no_send=%lu blocked=%lu | [P0] poll state_not_stream=%lu",
               (unsigned long)s_ble_export_p0.poll_calls,
               (unsigned long)s_ble_export_p0.poll_with_send_count,
               (unsigned long)s_ble_export_p0.poll_without_send_count,
               (unsigned long)s_ble_export_p0.poll_blocked_count,
               (unsigned long)s_ble_export_p0.poll_state_not_streaming_count);
    app_ble_export_p0_log_stat("poll_gap", &s_ble_export_p0.poll_gap_us);
    app_ble_export_p0_log_stat("frames_per_poll",
                               &s_ble_export_p0.frames_per_poll);
    app_ble_export_p0_log_stat("send_us", &s_ble_export_p0.send_call_us);
    DBG_DIRECT("[P0] peek_us c=%lu f=%lu",
               (unsigned long)s_ble_export_p0.peek_calls,
               (unsigned long)s_ble_export_p0.peek_fail);
    app_ble_export_p0_log_stat("peek_us", &s_ble_export_p0.peek_us);
    DBG_DIRECT("[P0] commit_us c=%lu f=%lu",
               (unsigned long)s_ble_export_p0.commit_calls,
               (unsigned long)s_ble_export_p0.commit_fail);
    app_ble_export_p0_log_stat("commit_us", &s_ble_export_p0.commit_us);
    DBG_DIRECT("[P0] producer_state_copy b=%lu",
               (unsigned long)s_ble_export_p0.producer_state_copy_bytes);
    app_ble_export_p0_log_stat("producer_state_copy_us",
                               &s_ble_export_p0.producer_state_copy_us);
    app_ble_export_p0_log_flash("flash_stream",
                                &s_ble_export_p0.flash_stream);
    app_ble_export_p0_log_flash("flash_prepare",
                                &s_ble_export_p0.flash_after_begin);
    app_ble_export_p0_log_flash("flash_total",
                                &s_ble_export_p0.flash_final);
    DBG_DIRECT("[P0] complete c=%lu f=%lu | [P0] send_blocked c=%lu to=%lu | [P0] complete cause=0x%04x | [P0] credits t=%lu last=%u",
               (unsigned long)s_ble_export_p0.send_complete_count,
               (unsigned long)s_ble_export_p0.send_complete_fail,
               (unsigned long)s_ble_export_p0.send_blocked_count,
               (unsigned long)s_ble_export_p0.send_blocked_timeout_count,
               s_ble_export_p0.send_complete_last_cause,
               (unsigned long)s_ble_export_p0.send_complete_credits_total,
               s_ble_export_p0.send_complete_credits_last);
    DBG_DIRECT("[P0] credits n=%lu x=%lu | [P0] credits v=%lu",
               (unsigned long)app_ble_p0_stat_min(
                   &s_ble_export_p0.send_complete_credits),
               (unsigned long)app_ble_p0_stat_max(
                   &s_ble_export_p0.send_complete_credits),
               (unsigned long)app_ble_p0_stat_avg(
                   &s_ble_export_p0.send_complete_credits));
    app_ble_export_p0_log_stat("complete_gap",
                               &s_ble_export_p0.send_complete_gap_us);
    DBG_DIRECT("[P0] prefetch en=%u b=%u | [P0] prefetch hit=%lu miss=%lu bypass=%lu",
               (uint32_t)ZY100_BLE_EXPORT_PREFETCH_ENABLE,
               (uint32_t)ZY100_BLE_EXPORT_PREFETCH_BYTES,
               (unsigned long)s_ble_export_p0.prefetch_final.hit_count,
               (unsigned long)s_ble_export_p0.prefetch_final.miss_count,
               (unsigned long)s_ble_export_p0.prefetch_final.bypass_count);
    DBG_DIRECT("[P0] prefetch read_c=%lu read_b=%lu | [P0] prefetch copied_b=%lu hit_rate_x1000=%lu",
               (unsigned long)s_ble_export_p0.prefetch_final.read_calls,
               (unsigned long)s_ble_export_p0.prefetch_final.read_bytes,
               (unsigned long)s_ble_export_p0.prefetch_final.copied_bytes,
               (unsigned long)prefetch_hit_rate_x1000);
    DBG_DIRECT("[P0] last_fail stage=%u cause=0x%04x | [P0] last_fail type=%u seq=%u | [P0] last_fail len=%u send_ret=%d | [P0] last_err producer=%lu complete=0x%04x",
               s_ble_export_p0.last_fail_stage,
               s_ble_export_p0.last_fail_cause,
               s_ble_export_p0.last_fail_type,
               s_ble_export_p0.last_fail_seq,
               s_ble_export_p0.last_fail_len,
               (int)s_ble_export_p0.last_send_ret,
               (unsigned long)s_ble_export_p0.last_producer_error,
               s_ble_export_p0.last_complete_cause);
#endif
}

void app_ble_export_p0_on_send_data_complete(uint8_t conn_id,
                                             T_SERVER_ID service_id,
                                             uint16_t attrib_idx,
                                             uint16_t cause,
                                             uint16_t credits)
{
    uint64_t now_us;

    if (!zy100_ble_ctrl_service_is_export_attrib(service_id, attrib_idx) ||
        !s_ble_export_p0.time.active ||
        s_ble_export_p0.time.summary_logged ||
        (conn_id != s_ble_export_p0.time.conn_id))
    {
        return;
    }
    now_us = app_ble_export_p0_now_us();
    s_ble_export_p0.send_complete_count++;
    s_ble_export_p0.send_complete_last_cause = cause;
    s_ble_export_p0.send_complete_credits_last = credits;
    s_ble_export_p0.send_complete_credits_total += credits;
    app_ble_p0_stat_add(&s_ble_export_p0.send_complete_credits, credits);
    if (s_ble_export_p0.previous_complete_us != 0ULL)
    {
        app_ble_p0_stat_add(
            &s_ble_export_p0.send_complete_gap_us,
            (uint32_t)(now_us - s_ble_export_p0.previous_complete_us));
    }
    s_ble_export_p0.previous_complete_us = now_us;
    if (cause != GAP_SUCCESS)
    {
        s_ble_export_p0.send_complete_fail++;
        s_ble_export_p0.notify_fail++;
        s_ble_export_p0.last_complete_cause = cause;
        app_ble_export_p0_record_fail(APP_BLE_P0_FAIL_STAGE_COMPLETE,
                                      0U,
                                      0U,
                                      0U,
                                      cause);
    }
}

void app_ble_export_p0_note_build_failure(uint8_t frame_type,
                                          uint16_t chunk_seq,
                                          uint16_t frame_len)
{
    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    s_ble_export_p0.last_send_ret = -1;
    app_ble_export_p0_record_fail(APP_BLE_P0_FAIL_STAGE_BUILD,
                                  frame_type,
                                  chunk_seq,
                                  frame_len,
                                  frame_len);
}

void app_ble_export_p0_note_send_call(bool sent,
                                      uint32_t elapsed_us,
                                      uint8_t frame_type,
                                      uint16_t frame_len,
                                      uint16_t payload_len)
{
    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    s_ble_export_p0.notify_attempts++;
    app_ble_p0_stat_add(&s_ble_export_p0.send_call_us, elapsed_us);
    s_ble_export_p0.last_send_ret = sent ? 1 : 0;
    if (sent)
    {
        app_ble_export_p0_note_notify_success(frame_type, frame_len, payload_len);
    }
}

void app_ble_export_p0_note_send_failure(uint8_t frame_type,
                                         uint16_t chunk_seq,
                                         uint16_t frame_len)
{
    if (!s_ble_export_p0.time.active || s_ble_export_p0.time.summary_logged)
    {
        return;
    }
    s_ble_export_p0.notify_fail++;
    app_ble_export_p0_record_fail(APP_BLE_P0_FAIL_STAGE_SEND,
                                  frame_type,
                                  chunk_seq,
                                  frame_len,
                                  0U);
}

void app_ble_export_p0_note_poll_blocked(void)
{
    if (s_ble_export_p0.time.active && !s_ble_export_p0.time.summary_logged)
    {
        s_ble_export_p0.poll_blocked_count++;
    }
}

void app_ble_export_p0_note_producer_failure(uint32_t error,
                                             uint16_t chunk_seq,
                                             uint32_t detail)
{
    s_ble_export_p0.last_producer_error = error;
    if (s_ble_export_p0.last_fail_stage == APP_BLE_P0_FAIL_STAGE_NONE)
    {
        app_ble_export_p0_record_fail(APP_BLE_P0_FAIL_STAGE_PEEK,
                                      0U,
                                      chunk_seq,
                                      0U,
                                      (uint16_t)detail);
    }
}

void app_ble_export_p0_note_peek(uint32_t elapsed_us,
                                 bool ok,
                                 uint32_t producer_error,
                                 uint16_t chunk_seq,
                                 uint16_t payload_len)
{
    s_ble_export_p0.peek_calls++;
    app_ble_p0_stat_add(&s_ble_export_p0.peek_us, elapsed_us);
    if (!ok || (payload_len == 0U))
    {
        s_ble_export_p0.peek_fail++;
        s_ble_export_p0.last_producer_error = producer_error;
        app_ble_export_p0_record_fail(APP_BLE_P0_FAIL_STAGE_PEEK,
                                      0U,
                                      chunk_seq,
                                      payload_len,
                                      (uint16_t)producer_error);
    }
}

void app_ble_export_p0_note_commit(uint32_t elapsed_us,
                                   bool ok,
                                   uint32_t producer_error,
                                   uint16_t chunk_seq,
                                   uint16_t payload_len)
{
    s_ble_export_p0.commit_calls++;
    app_ble_p0_stat_add(&s_ble_export_p0.commit_us, elapsed_us);
    if (!ok)
    {
        s_ble_export_p0.commit_fail++;
        s_ble_export_p0.last_producer_error = producer_error;
        app_ble_export_p0_record_fail(APP_BLE_P0_FAIL_STAGE_COMMIT,
                                      APP_BLE_EXPORT_FRAME_DATA,
                                      chunk_seq,
                                      payload_len,
                                      (uint16_t)producer_error);
    }
}
#endif
