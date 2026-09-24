#ifndef ZY100_CAPTURE_SPI_SCHEDULER_H
#define ZY100_CAPTURE_SPI_SCHEDULER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define ZY100_SPI_SCHED_START_GUARD_MS 25U

/*
 * Capture runtime SPI boundary:
 * while capture runtime is active, business modules request work here and the
 * scheduler executor is the only layer allowed to call IMU/Flash SPI paths.
 *
 * OIS is reserved for a later phase. Future rule: a 6.4kHz OIS timer is enabled
 * only while OIS_ACTIVE; OIS sample priority is above RT marker; Flash pump is
 * disabled while OIS is active. ZY100_OIS_ENABLE remains 0 in this phase.
 */
typedef enum
{
    ZY100_SPI_PRIO_FIFO_EMERGENCY = 0U,
    ZY100_SPI_PRIO_OIS_SAMPLE     = 1U,
    ZY100_SPI_PRIO_RT_MARKER      = 2U,
    ZY100_SPI_PRIO_FIFO_NORMAL    = 3U,
    ZY100_SPI_PRIO_FLASH_BG       = 4U,
} zy100_spi_prio_t;

typedef enum
{
    ZY100_SPI_OP_NONE = 0U,
    ZY100_SPI_OP_OIS_READ,
    ZY100_SPI_OP_RT_UI_READ,
    ZY100_SPI_OP_FIFO_COUNT,
    ZY100_SPI_OP_FIFO_INT_STATUS,
    ZY100_SPI_OP_FIFO_DRAIN_512,
    ZY100_SPI_OP_FIFO_POLL_DRAIN,
    ZY100_SPI_OP_FLASH_PUMP,
} zy100_spi_op_t;

typedef enum
{
    ZY100_SPI_SCHED_FLASH_RESULT_UNKNOWN = 0U,
    ZY100_SPI_SCHED_FLASH_RESULT_NO_WORK,
    ZY100_SPI_SCHED_FLASH_RESULT_WIP_BUSY,
    ZY100_SPI_SCHED_FLASH_RESULT_PAGE_ISSUED,
    ZY100_SPI_SCHED_FLASH_RESULT_ERROR,
} zy100_spi_sched_flash_result_t;

typedef enum
{
    ZY100_SPI_SCHED_DRAIN_SRC_IRQ_FIXED = 0U,
    ZY100_SPI_SCHED_DRAIN_SRC_COUNT_BASED,
    ZY100_SPI_SCHED_DRAIN_SRC_CONFIRMED,
    ZY100_SPI_SCHED_DRAIN_SRC_EMERGENCY,
    ZY100_SPI_SCHED_DRAIN_SRC_FINAL,
} zy100_spi_sched_drain_src_t;

typedef struct
{
    uint32_t seq;
    zy100_spi_sched_drain_src_t src;
    uint16_t req_len;
    uint16_t actual_len;
    bool cached_count_valid;
    uint16_t cached_count;
    bool fifo_irq_pending;
    uint32_t irq_pending_count;
    uint8_t int_level;
    uint32_t queue_level;
    uint32_t flash_capacity_packets;
    uint32_t fill_packets;
    bool fast_path;
    uint32_t time_since_last_irq_ms;
    uint32_t time_since_last_drain_ms;
    bool capped_by_flash_capacity;
} zy100_spi_sched_drain_context_t;

typedef struct
{
    bool count_valid;
    bool drained;
    bool ingress_blocked;
    bool high_pressure;
    bool emergency;
    uint16_t fifo_count;
    uint16_t drain_len;
} zy100_spi_sched_poll_drain_result_t;

typedef struct
{
    uint32_t rt_due;
    uint32_t rt_read_ok;
    uint32_t rt_late;
    uint32_t rt_skip_fifo;
    uint32_t rt_skip_spi;
    uint32_t fifo_count_op;
    uint32_t fifo_int_status_op;
    uint32_t fifo_drain_op;
    uint32_t fifo_emergency_count;
    uint32_t fifo_emergency_drain_full_len;
    uint32_t fifo_emergency_drain_staged;
    uint32_t fifo_emergency_drop;
    uint32_t fifo_emergency_saved;
    uint32_t fifo_emergency_capacity_bypass;
    uint32_t flash_pump_op;
    uint32_t flash_page_issued;
    uint32_t flash_block_done;
    uint32_t flash_wip_busy;
    uint32_t flash_no_work;
    uint32_t after_poll_pump_count;
    uint32_t burst_enter_count;
    uint32_t burst_step_count;
    uint32_t burst_page_issued_count;
    uint32_t burst_wip_busy_count;
    uint32_t stale_safe_allow_count;
    uint32_t stale_safe_deny_count;
    uint32_t flash_budget_stop_count;
    uint32_t flash_fifo_skip_count;
    uint32_t final_steps_count;
    uint32_t final_wip_busy_count;
    uint32_t flash_blocked_by_rt;
    uint32_t flash_blocked_by_fifo;
    uint32_t flash_throttle_rt_due;
    uint32_t flash_throttle_fifo_pending;
    uint32_t flash_throttle_budget;
    uint32_t max_consecutive_flash_steps;
    uint32_t flash_starved_count;
    uint32_t flash_queue_level;
    uint32_t flash_queue_near_full;
    uint32_t max_fifo_count;
    uint32_t fifo_drain_latched;
    uint32_t fifo_count_over_capacity;
    uint32_t fifo_count_to_drain_latch;
    uint32_t fifo_count_rejected;
    uint32_t fifo_irq_to_drain_latch;
    uint32_t fifo_fixed_drain_op;
    uint32_t fifo_irq_drain_rt_defer;
    uint32_t fifo_irq_drain_force_after_defer;
    uint32_t fifo_count_reason_poll;
    uint32_t fifo_count_reason_probe;
    uint32_t fifo_count_reason_cache;
    uint32_t fifo_count_reason_emergency_50ms;
    uint32_t fifo_count_reason_debug_100ms;
    uint32_t fifo_count_reason_post_drain;
    uint32_t fifo_post_drain_count_req;
    uint32_t fifo_post_drain_int_status_req;
    uint32_t post_drain_count_op;
    uint32_t post_drain_rechain;
    uint32_t post_drain_count_high;
    uint32_t post_drain_count_lt_wm;
    uint32_t post_drain_count_ge_wm;
    uint32_t post_drain_count_ge_emergency;
    uint32_t consecutive_drain_count;
    uint32_t fifo_count_skipped_rt_due;
    uint32_t fifo_int_status_deferred_rt;
    uint32_t fifo_int_status_after_drain;
    uint32_t irq_repeat_before_status;
    uint32_t fifo_drain_fail_count;
    uint32_t fifo_drain_fail_reason;
    uint32_t fifo_drain_fail_last_len;
    uint32_t fifo_drain_fail_first_header;
    uint32_t fifo_drain_len_requested;
    uint32_t fifo_drain_len_actual;
    uint32_t fifo_drain_capped_by_flash_capacity;
    uint32_t fifo_drain_no_flash_capacity;
    uint32_t fifo_drain_cross_block_append;
    uint32_t fifo_start_guard_skip;
    uint32_t fifo_first_irq_confirm;
    uint32_t fifo_first_irq_confirm_ok;
    uint32_t fifo_fixed_drain_after_warmup;
    uint32_t early_drain_bad_header;
    uint32_t first_good_drain_ms;
    uint32_t count_probe_during_guard;
    uint32_t count_confirm_to_pending_drain;
    uint32_t unguarded_drain_blocked_by_start_guard;
    uint32_t confirmed_drain_pending;
    uint32_t fifo_service_due_count;
    uint32_t fifo_service_overdue_count;
    uint32_t fifo_service_emergency_count;
    uint32_t fifo_service_count_probe;
    uint32_t fifo_service_recovered_by_count;
    uint32_t fifo_service_rt_skip;
    uint32_t fifo_service_flash_block;
    uint32_t ms_since_last_fifo_drain;
    uint32_t no_op_count;
} zy100_spi_sched_stats_t;

typedef bool (*zy100_spi_sched_read_fifo_count_cb_t)(void *ctx, uint16_t *fifo_count);
typedef bool (*zy100_spi_sched_read_fifo_int_status_cb_t)(void *ctx);
typedef bool (*zy100_spi_sched_drain_fifo_cb_t)(void *ctx,
                                                const zy100_spi_sched_drain_context_t *drain_ctx,
                                                uint16_t drain_len,
                                                bool guarded_startup,
                                                uint8_t *first_header,
                                                uint16_t *actual_read_len,
                                                bool *capped_by_flash_capacity,
                                                bool *no_flash_capacity);
typedef bool (*zy100_spi_sched_rt_marker_read_cb_t)(void *ctx, uint32_t now_ms);
typedef void (*zy100_spi_sched_rt_marker_skip_cb_t)(void *ctx, uint32_t now_ms);
typedef bool (*zy100_spi_sched_flash_pump_cb_t)(void *ctx);
typedef zy100_spi_sched_flash_result_t
(*zy100_spi_sched_flash_result_cb_t)(void *ctx);
typedef void (*zy100_spi_sched_flash_skip_cb_t)(void *ctx);
typedef bool (*zy100_spi_sched_fifo_poll_drain_cb_t)(
    void *ctx,
    bool high_water_alarm,
    zy100_spi_sched_poll_drain_result_t *result);

typedef struct
{
    uint32_t now_ms;
    bool capture_active;
    bool fifo_irq_pending;
    bool fifo_poll_due;
    bool fifo_high_water_alarm_pending;
    bool fifo_rescue_pending;
    bool rt_due_pending;
    bool fifo_service_due;
    bool fifo_service_overdue;
    bool fifo_service_emergency;
    bool flash_pump_requested;
    bool flash_has_pending_work;
    bool flash_raw_queue_busy;
    uint32_t flash_queue_level;
    uint32_t flash_backlog_blocks;
    bool flash_queue_near_full;
    uint32_t ms_since_last_fifo_drain;
    bool timer_only_test;
    bool fifo_only_test;
    void *ctx;
    zy100_spi_sched_read_fifo_count_cb_t read_fifo_count;
    zy100_spi_sched_read_fifo_int_status_cb_t read_fifo_int_status;
    zy100_spi_sched_drain_fifo_cb_t drain_fifo_512;
    zy100_spi_sched_fifo_poll_drain_cb_t fifo_poll_drain;
    zy100_spi_sched_rt_marker_read_cb_t rt_marker_read;
    zy100_spi_sched_rt_marker_skip_cb_t rt_marker_skip_fifo;
    zy100_spi_sched_rt_marker_skip_cb_t rt_marker_skip_spi;
    zy100_spi_sched_flash_pump_cb_t flash_pump;
    zy100_spi_sched_flash_result_cb_t flash_pump_result;
    zy100_spi_sched_flash_skip_cb_t flash_skip_fifo;
} zy100_spi_sched_input_t;

void zy100_spi_sched_reset(uint32_t now_ms);
void zy100_spi_sched_mark_session_start(uint32_t now_ms);
void zy100_spi_sched_invalidate_fifo_count(void);
void zy100_spi_sched_capture_stop(void);
uint32_t zy100_spi_sched_alloc_drain_seq(void);
bool zy100_spi_sched_request_rt_marker(void);
bool zy100_spi_sched_request_fifo_count(void);
bool zy100_spi_sched_request_fifo_count_emergency_probe(void);
bool zy100_spi_sched_request_fifo_count_debug_probe(void);
bool zy100_spi_sched_request_fifo_int_status(void);
bool zy100_spi_sched_request_fifo_drain(void);
bool zy100_spi_sched_request_fifo_drain_from_irq(uint32_t now_ms,
                                                 uint32_t pending_irq_count);
bool zy100_spi_sched_request_flash_pump(void);
bool zy100_spi_sched_has_pending_work(void);
bool zy100_spi_sched_has_urgent_work(void);
bool zy100_spi_sched_execute_once(const zy100_spi_sched_input_t *in);
bool zy100_spi_sched_last_op_was_flash_pump(void);
void zy100_spi_sched_note_flash_pump_step_result(bool page_issued,
                                                 bool block_done,
                                                 bool wip_busy,
                                                 bool no_work);
void zy100_spi_sched_note_final_flash_pump_step(bool wip_busy);
void zy100_spi_sched_note_fifo_emergency_drain_result(bool full_len,
                                                      bool staged,
                                                      bool dropped,
                                                      bool saved,
                                                      bool capacity_bypass);
void zy100_spi_sched_note_fifo_drain_cross_block_append(void);
void zy100_spi_sched_get_stats(zy100_spi_sched_stats_t *out);
void zy100_spi_sched_log_1s_if_due(uint32_t now_ms);
void zy100_spi_sched_log_5s_if_due(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_CAPTURE_SPI_SCHEDULER_H */
