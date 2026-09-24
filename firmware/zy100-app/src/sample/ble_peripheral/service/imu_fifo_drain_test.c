#include "imu_fifo_drain_test.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS_API.h"
#include "os_sched.h"
#include "os_sync.h"
#include "trace.h"
#include "zy100_capture_start_diag.h"
#include "rtl876x_spi.h"

#ifndef portYIELD_FROM_ISR
#include "cmsis_armcc.h"
#define APP_PORT_SY_FULL_READ_WRITE     15
#define APP_PORT_NVIC_INT_CTRL_REG      (*((volatile uint32_t *)0xe000ed04))
#define APP_PORT_NVIC_PENDSVSET_BIT     (1UL << 28UL)
#define portYIELD_FROM_ISR(xSwitchRequired)                  \
    do                                                       \
    {                                                        \
        if ((xSwitchRequired) != pdFALSE)                    \
        {                                                    \
            APP_PORT_NVIC_INT_CTRL_REG = APP_PORT_NVIC_PENDSVSET_BIT; \
            __dsb(APP_PORT_SY_FULL_READ_WRITE);              \
            __isb(APP_PORT_SY_FULL_READ_WRITE);              \
        }                                                    \
    } while (0)
#endif

#include "../app_flags.h"
#if ZY100_PRODUCT_LOG_QUIET_ENABLE
#undef DBG_DIRECT
#define DBG_DIRECT(...) ZY100_LOG_VERBOSE(__VA_ARGS__)
#endif
#if ZY100_SRAM_SUMMARY_LOG_ENABLE
#include "FreeRTOS/portable.h"
#include "mem_config.h"
#endif
#include "../bsp/imu_bsp.h"
#include "../common/imu_common.h"
#include "../driver/icm53611_driver.h"
#include "../driver/icm53611_reg.h"
#include "../driver/icm53611_spi.h"
#include "../driver/spi_bus_owner.h"
#include "../zy100_clock_config.h"
#include "imu_pre_trigger_marker.h"
#include "imu_sample_marker.h"
#include "imu_swing_detector.h"
#include "imu_wom_irq.h"
#if V1_IMU_FLASH_CAPTURE_ENABLE
#include "external_flash_capture.h"
#endif
#include "imu_rt_marker.h"
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
#include "zy100_final_edge_bscan.h"
#endif
#include "zy100_fifo_ingress_ring.h"
#include "zy100_capture_spi_scheduler.h"
#include "zy100_rt_marker_timer.h"
#include "zy100_edge_shadow.h"
#if ZY100_ONLINE_STREAM_ENABLE
#include "zy100_online_stream.h"
#include "zy100_online_reset_trace.h"
#include "zy100_online_spool.h"
#include "zy100_online_raw_capture.h"
#if ZY100_ONLINE_STRESS_TEST_ENABLE
#include "zy100_online_stress.h"
#endif
#include "zy100_mode_workspace.h"
#endif
#if ZY100_LEGACY_OFFLINE_ENABLE
#include "zy100_offline_v2_capture.h"
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE
#include "zy100_edge_lite.h"
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE
#include "zy100_final_edge_queue.h"
#include "zy100_final_edge_lf_pre_ring.h"
#include "zy100_final_edge_raw_store.h"
#include "zy100_final_edge_record_store.h"
#include "zy100_capture_time.h"
#include "zy100_training_session.h"
#include "zy100_session_directory.h"
#endif
#if V0_IMU_FIFO_DRAIN_TEST

#define V0_TASK_STACK_WORDS             2048U
#if ZY100_IMU_LIFECYCLE_LOG_ENABLE
#define V0_IMU_LIFECYCLE_LOG(...)       DBG_DIRECT(__VA_ARGS__)
#else
#define V0_IMU_LIFECYCLE_LOG(...)       do { if (0) { DBG_DIRECT(__VA_ARGS__); } } while (0)
#endif
#ifndef V0_ADC_SAFE_WINDOW_MAX_AGE_MS
#define V0_ADC_SAFE_WINDOW_MAX_AGE_MS   2U
#endif
#ifndef V0_TASK_PRIORITY_CANDIDATE
#define V0_TASK_PRIORITY_CANDIDATE      6U
#endif
#if (V0_TASK_PRIORITY_CANDIDATE < configMAX_PRIORITIES)
#define V0_TASK_PRIORITY                (tskIDLE_PRIORITY + V0_TASK_PRIORITY_CANDIDATE)
#else
#define V0_TASK_PRIORITY                (configMAX_PRIORITIES - 1U)
#endif
#define V0_CAPTURE_DURATION_MS          120000U
#define V0_LOG_PERIOD_MS                1000U
#define V0_FE_LOG_PERIOD_MS             5000U
#define V0_POLL_PERIOD_MS               5U
#define V0_PACKET_SIZE_BYTES            16U
#define V0_FIFO_WATERMARK_BYTES         ZY100_FIFO_WATERMARK_BYTES
#define V0_FIFO_DRAIN_THRESHOLD_BYTES   ZY100_FIFO_DRAIN_THRESHOLD_BYTES
#define V0_FIFO_DRAIN_MAX_RUNTIME_BYTES ZY100_FIFO_DRAIN_MAX_BYTES
#define V0_FIFO_WATERMARK_ALARM_BYTES   ZY100_FIFO_WATERMARK_ALARM_BYTES
#ifndef V0_ONLINE_COOP_YIELD_PERIOD_MS
#define V0_ONLINE_COOP_YIELD_PERIOD_MS 20U
#endif

static bool v0_online_quiet_logs(void)
{
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    return zy100_online_stream_quiet_logs_active();
#else
    return false;
#endif
}

static void v0_online_capture_coop_yield(uint32_t now_ms)
{
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    static uint32_t s_last_yield_ms = 0U;

    if (!zy100_online_stream_active() && !zy100_online_stream_end_wait_ack())
    {
        return;
    }
    if ((s_last_yield_ms != 0U) &&
        ((uint32_t)(now_ms - s_last_yield_ms) < V0_ONLINE_COOP_YIELD_PERIOD_MS))
    {
        return;
    }

    s_last_yield_ms = now_ms;
    if (zy100_online_raw_capture_active())
    {
        (void)zy100_online_raw_capture_mag_checkpoint(
            ZY100_ONLINE_RAW_MAG_CHECKPOINT_BEFORE_COOP_YIELD_FORCE);
    }
    os_delay(1U);
#else
    IMU_UNUSED(now_ms);
#endif
}

/* Final drain runs after capture has stopped. Yield only for an online
 * session so the App task can keep the AON watchdog service cadence. */
static void v0_online_final_drain_coop_yield(void)
{
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    if (zy100_online_stream_active() ||
        zy100_online_stream_end_wait_ack())
    {
        os_delay(1U);
    }
#endif
}
#define V0_FIFO_HIGH_WATER_BYTES        ZY100_FIFO_EMERGENCY_BYTES
#define V0_FIFO_READ_BUFFER_BYTES       512U
#define V0_FIFO_DRAIN_MAX_BYTES         (V0_FIFO_READ_BUFFER_BYTES * 8U)
#define V0_SAMPLE_RATE_HZ               ZY100_UI_FIFO_SAMPLE_RATE_HZ
#define V0_SAMPLE_INTERVAL_US           (1000000U / V0_SAMPLE_RATE_HZ)
#define V0_PACKET_THRESHOLD_PERCENT     99U
#define V0_WM_IRQ_MIN_DRAIN_COUNT       4000U
#define V0_FIFO_COUNT_EMERGENCY_PROBE_MS 50U
#define V0_FIFO_COUNT_DEBUG_PROBE_MS    100U
#define V0_RT_START_WAIT_LOG_PERIOD_MS  5000U
#define V0_FLASH_Q_LOG_PERIOD_MS        1000U
#define V0_FIFO_DRAIN_CTX_RING_SIZE     8U
#define V0_PHASE_A_FINAL_FLUSH_TIMEOUT_MS 1000U
#define V0_PHASE_A_FINAL_FLUSH_MAX_RETRY  4096U
#define V0_TS_WARMUP_PACKET_COUNT       8U
#define V0_TS_DELTA_MIN_US              1000U
#define V0_TS_DELTA_MAX_US              1500U
#ifndef V0_ONLINE_BAD_FIFO_CONSECUTIVE_LIMIT
#define V0_ONLINE_BAD_FIFO_CONSECUTIVE_LIMIT 8U
#endif
#define V0_HEADER_MSG                   BIT(7)
#define V0_HEADER_ACCEL                 BIT(6)
#define V0_HEADER_GYRO                  BIT(5)
#define V0_HEADER_20                    BIT(4)
#define V0_HEADER_TIMESTAMP_MASK        (BIT(3) | BIT(2))
#define V0_HEADER_TIMESTAMP_ODR         BIT(3)
#define V0_PACKET_ACCEL_X_HIGH_IDX      1U
#define V0_PACKET_ACCEL_X_LOW_IDX       2U
#define V0_PACKET_ACCEL_Y_HIGH_IDX      3U
#define V0_PACKET_ACCEL_Y_LOW_IDX       4U
#define V0_PACKET_ACCEL_Z_HIGH_IDX      5U
#define V0_PACKET_ACCEL_Z_LOW_IDX       6U
#define V0_PACKET_GYRO_X_HIGH_IDX       7U
#define V0_PACKET_GYRO_X_LOW_IDX        8U
#define V0_PACKET_GYRO_Y_HIGH_IDX       9U
#define V0_PACKET_GYRO_Y_LOW_IDX        10U
#define V0_PACKET_GYRO_Z_HIGH_IDX       11U
#define V0_PACKET_GYRO_Z_LOW_IDX        12U
#define V0_PACKET_TMST_HIGH_IDX         14U
#define V0_PACKET_TMST_LOW_IDX          15U
#define V0_NOTIFY_START                 BIT(0)
#define V0_NOTIFY_RT_TIMER              BIT(1)
#define V0_NOTIFY_FIFO_IRQ              BIT(2)
#define V0_NOTIFY_LIVE_TIMER            BIT(3)
#define V0_NOTIFY_FE_TIMER              V0_NOTIFY_LIVE_TIMER
#define V0_NOTIFY_MASK                  (V0_NOTIFY_START | V0_NOTIFY_RT_TIMER | \
                                         V0_NOTIFY_FIFO_IRQ | V0_NOTIFY_LIVE_TIMER)
#if ZY100_FINAL_EDGE_MODE_ENABLE
#define V0_FE_NORMAL_PACKET_BYTES       V0_PACKET_SIZE_BYTES
#define V0_FE_HIGH_PRESSURE_BYTES       ZY100_FINAL_EDGE_INT_HIGH_PRESSURE_WATERMARK_BYTES
#define V0_FE_EMERGENCY_BYTES           768U
#define V0_FE_CATCHUP_256_BYTES         256U
#define V0_FE_CATCHUP_512_BYTES         512U
#endif
#define V0_FLASH_RUNTIME_ENABLE         0U

#if ZY100_FIFO_MARKER_TRIGGER_ENABLE
#define V0_FIFO_MARKER_TRIGGER_ENABLED  1
#else
#define V0_FIFO_MARKER_TRIGGER_ENABLED  0
#endif

#if IMU_RUNTIME_VERBOSE_LOG_ENABLE || ZY100_FIFO_MARKER_DEBUG_ENABLE
#define V0_FIFO_INT_STATUS_RUNTIME_DEBUG 1
#else
#define V0_FIFO_INT_STATUS_RUNTIME_DEBUG 0
#endif

typedef struct
{
    uint32_t round;
    uint32_t irq_count;
    uint32_t irq_drain_count;
    uint32_t poll_count;
    uint32_t poll_below_wm_count;
    uint32_t poll_rescue_count;
    uint32_t poll_rescue_drain_count;
    uint32_t high_water_rescue_count;
    uint32_t stop_drain_count;
    uint32_t packet_count;
    uint32_t bytes_read;
    uint32_t max_fifo_count;
    uint32_t last_fifo_drain_ms;
    uint32_t fifo_drain_since_start;
    uint32_t max_loop_gap_ms;
    uint32_t loop_gap_over_10ms_count;
    uint32_t loop_gap_over_20ms_count;
    uint32_t queue_max_level;
    uint32_t ingress_max_level;
    uint32_t ingress_flush_bytes;
    uint32_t ingress_flush_blocked_by_flash_capacity;
    uint32_t ingress_final_flush_fail_count;
    uint32_t ingress_backpressure_enter;
    uint32_t ingress_backpressure_pump;
    uint32_t ingress_backpressure_flush;
    uint32_t ingress_backpressure_recovered;
    uint32_t ingress_backpressure_fatal;
    uint32_t ingress_free_min;
    uint32_t ingress_level_max;
    uint32_t flash_pump_count;
    uint32_t flash_block_done_count;
    uint32_t phase_a_alarm_irq_count;
    uint32_t phase_a_ingress_emergency_attempts;
    uint16_t phase_a_ingress_blocked_packets;
    uint16_t phase_a_ingress_blocked_fifo_count;
    uint32_t fifo_full_count;
    uint32_t bad_header_count;
    uint32_t ts_jump_count;
    uint32_t reset_done_count_during_capture;
    uint32_t read_error_count;
    uint32_t edge_hook_feed_count;
    uint16_t fifo_lost_pkt_count;
    uint16_t last_tmst_raw;
    uint8_t ts_warmup_remaining;
    bool tmst_valid;
    bool capture_active;
    bool fatal_error;
    bool reached_final_summary;
    bool ingress_backpressure_pending;
    bool phase_a_ingress_drain_blocked;
    bool phase_a_ingress_blocked_high_pressure;
    bool phase_a_ingress_blocked_emergency;
    bool phase_a_alarm_service_done;
    imu_status_t fatal_status;
    imu_fifo_drain_test_stop_reason_t stop_reason;
#if ZY100_RT_MARKER_ENABLE && ZY100_RT_MARKER_TIMER_ENABLE
    bool rt_start_good_drain;
    uint32_t rt_start_last_drain_seq;
    uint16_t rt_start_last_cached_count;
    uint32_t rt_start_last_bad_count;
    uint16_t rt_start_last_lost_count;
    uint32_t rt_start_last_full_count;
    const char *rt_start_wait_reason;
#endif
} v0_fifo_stats_t;

#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
typedef struct
{
    bool active;
    bool captured;
    bool fatal_error;
    bool bad_fifo_valid;
    uint8_t summary_stage;
    uint8_t bad_fifo_header;
    uint8_t bad_fifo_int_level;
    uint8_t bad_fifo_source;
    uint16_t bad_fifo_offset;
    uint16_t bad_fifo_req_len;
    uint16_t bad_fifo_actual_len;
    uint16_t bad_fifo_count;
    uint32_t stop_reason;
    uint32_t completion_generation;
    uint32_t fatal_status;
    uint32_t fifo_full_count;
    uint32_t fifo_lost_pkt_count;
    uint32_t bad_header_count;
    uint32_t ingress_overflow_count;
    uint32_t raw_error;
    uint32_t raw_accepted;
    uint32_t raw_committed;
    uint32_t raw_ring_high_water;
} v0_online_abort_diag_t;
#endif

typedef struct
{
    bool valid;
    zy100_spi_sched_drain_context_t ctx;
    uint16_t post_count;
    uint16_t actual_read_len;
    uint32_t read_us;
    uint8_t first_header;
    bool completed;
} v0_fifo_drain_diag_entry_t;

#if ZY100_LEGACY_OFFLINE_ENABLE
typedef enum
{
    V0_OFFLINE_FAULT_NONE = 0U,
    V0_OFFLINE_FAULT_BAD_HEADER,
    V0_OFFLINE_FAULT_FIFO_FULL,
    V0_OFFLINE_FAULT_FIFO_LOST,
    V0_OFFLINE_FAULT_SPI_READ,
    V0_OFFLINE_FAULT_INGRESS,
    V0_OFFLINE_FAULT_ALGO,
} v0_offline_fault_t;

typedef enum
{
    V0_OFFLINE_FIFO_STAGE_NORMAL = 0U,
    V0_OFFLINE_FIFO_STAGE_ALARM,
    V0_OFFLINE_FIFO_STAGE_RESCUE,
} v0_offline_fifo_stage_t;

typedef struct
{
    bool active;
    bool first_valid;
    v0_offline_fault_t first_fault;
    v0_offline_fifo_stage_t first_stage;
    uint8_t first_header;
    uint8_t final_int_status;
    uint16_t first_fifo_count;
    uint16_t first_req_len;
    uint16_t first_actual_len;
    uint16_t final_fifo_count;
    uint16_t final_lost_raw;
    uint32_t first_time_ms;
    uint32_t first_packet_seq;
    uint32_t drain_count;
    uint32_t drain_bytes;
    uint32_t irq_service_count;
    uint32_t poll_service_count;
    uint32_t alarm_enter_count;
    uint32_t rescue_enter_count;
    uint32_t pressure_recover_count;
    uint32_t yield_count;
    uint32_t max_burst_reads;
    uint32_t max_read_us;
    uint32_t max_loop_gap_ms;
    uint16_t max_fifo_count;
    uint8_t final_gpio_level;
    uint8_t final_spi_status;
} v0_offline_fifo_diag_t;
#endif

static TaskHandle_t s_v0_task_handle = NULL;
static volatile uint32_t s_v0_imu_fifo_pending_count = 0U;
static volatile uint32_t s_v0_irq_count = 0U;
static volatile bool s_v0_high_water_alarm_pending = false;
static bool s_v0_phase_a_rescue_pending = false;
static volatile uint32_t s_v0_current_sample_seq = 0U;
static volatile bool s_v0_start_pending = false;
static volatile bool s_v0_stop_requested = false;
static volatile bool s_v0_online_pause_stop_requested = false;
static volatile bool s_v0_capture_active = false;
static volatile uint32_t s_v0_last_packet_result = 0U;
static volatile bool s_v0_offline_exit_logged = false;
static volatile bool s_v0_stop_in_progress = false;
static volatile bool s_v0_worker_busy = false;
static volatile bool s_v0_capture_done = false;
static volatile uint32_t s_v0_completion_generation = 0U;
static volatile bool s_v0_spi_fifo_busy = false;
static volatile bool s_v0_hw_ready = false;
static volatile bool s_v0_adc_safe_window_open = false;
static volatile uint32_t s_v0_adc_safe_window_ms = 0U;
static volatile bool s_v0_suppress_next_stop_request_log = false;
static volatile uint32_t s_v0_next_round = 1U;
static volatile uint32_t s_v0_current_round = 0U;
static volatile imu_fifo_drain_test_stop_reason_t s_v0_stop_reason =
    IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
static volatile imu_fifo_drain_test_stop_reason_t s_v0_last_stop_reason =
    IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
#if ZY100_ONLINE_STREAM_ENABLE
static uint32_t s_v0_online_bad_fifo_consecutive = 0U;
static volatile bool s_v0_online_abort_requested = false;
static zy100_online_pause_progress_t s_v0_online_pause_progress;
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
static v0_online_abort_diag_t s_v0_online_abort_diag;
/* Producer-owned snapshot; consumed only by the post-stop App summary. */
static struct
{
    uint32_t capture_ms;
    uint32_t packets;
    uint32_t fifo_hi;
    uint32_t full;
    uint32_t lost;
    uint32_t read_errors;
    uint32_t bad_headers;
    uint32_t gap_ms;
    uint32_t stack_free;
    bool pending;
    uint8_t stage;
} s_v0_online_resource;
#if !ZY100_FINAL_EDGE_MODE_ENABLE
static volatile uint32_t s_v0_online_live_timer_due_count = 0U;
static volatile uint32_t s_v0_online_live_timer_total_due_count = 0U;
static volatile uint32_t s_v0_online_live_timer_last_us = 0U;
static volatile uint32_t s_v0_online_live_timer_gap_max_us = 0U;
static volatile uint32_t s_v0_online_live_timer_notify_fail_count = 0U;
static volatile bool s_v0_online_live_timer_running = false;
#endif
#endif
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE
static volatile zy100_fe_capture_state_t s_fe_capture_state =
    FE_CAPTURE_STATE_IDLE;
static volatile zy100_fe_stop_reason_t s_fe_stop_reason =
    ZY100_FE_STOP_REASON_NONE;
static volatile bool s_fe_export_ready = false;
static volatile bool s_fe_stop_seen = false;
static volatile bool s_fe_final_drain_done = false;
static volatile bool s_fe_export_cleared = false;
static volatile bool s_fe_export_preserved = false;
static volatile uint32_t s_fe_stop_request_ms = 0U;
static bool s_fe_final_drain_active = false;
static bool s_final_edge_no_new_data_mode = false;
static bool s_final_edge_live_timer_stopped_for_drain = false;
static bool s_final_edge_imu_live_stopped_for_drain = false;
#endif
#if V1_IMU_FLASH_CAPTURE_ENABLE
static volatile bool s_v1_flash_ready_wait_start = false;
static bool s_v1_flash_prepare_busy_logged = false;
#endif
static uint8_t s_v0_fifo_buf[V0_FIFO_READ_BUFFER_BYTES];
#if V0_FLASH_RUNTIME_ENABLE
static uint8_t s_v0_ingress_flush_buf[ZY100_FIFO_INGRESS_FLUSH_BYTES];
#endif
static v0_fifo_drain_diag_entry_t s_v0_fifo_drain_ctx_ring[V0_FIFO_DRAIN_CTX_RING_SIZE];
#if V0_FLASH_RUNTIME_ENABLE
static bool s_v0_phase_a_ingress_flush_used_this_loop = false;
#endif
static uint8_t s_v0_fifo_drain_ctx_ring_next = 0U;
static uint8_t s_v0_fifo_drain_ctx_ring_count = 0U;
#if ZY100_LEGACY_OFFLINE_ENABLE
static v0_offline_fifo_diag_t s_v0_offline_fifo_diag;
typedef char v0_offline_fifo_diag_size_check[
    ((sizeof(s_v0_offline_fifo_diag) + sizeof(s_v0_fifo_drain_ctx_ring)) <=
      768U) ? 1 : -1];
#endif
#if V0_FIFO_MARKER_TRIGGER_ENABLED
static uint64_t s_v0_detector_sample_base_ts_us = 0ULL;
static uint32_t s_v0_detector_rate_accum = 0U;
#endif

#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
static void v0_online_abort_diag_latch_bad_fifo(
    const zy100_spi_sched_drain_context_t *drain_ctx,
    uint16_t offset,
    uint8_t header,
    uint8_t int_level_after)
{
    uint32_t lock_state;

    if (!s_v0_online_abort_diag.active)
    {
        return;
    }

    lock_state = os_lock();
    if (!s_v0_online_abort_diag.bad_fifo_valid)
    {
        s_v0_online_abort_diag.bad_fifo_valid = true;
        s_v0_online_abort_diag.bad_fifo_header = header;
        s_v0_online_abort_diag.bad_fifo_int_level = int_level_after;
        s_v0_online_abort_diag.bad_fifo_offset = offset;
        if (drain_ctx != NULL)
        {
            s_v0_online_abort_diag.bad_fifo_source =
                (uint8_t)drain_ctx->src;
            s_v0_online_abort_diag.bad_fifo_req_len = drain_ctx->req_len;
            s_v0_online_abort_diag.bad_fifo_actual_len =
                drain_ctx->actual_len;
            s_v0_online_abort_diag.bad_fifo_count =
                drain_ctx->cached_count_valid ? drain_ctx->cached_count : 0U;
        }
    }
    os_unlock(lock_state);
}

static void v0_online_abort_diag_capture_end(const v0_fifo_stats_t *stats)
{
    zy100_online_raw_capture_stats_t raw_stats;

    if ((stats == NULL) || !s_v0_online_abort_diag.active ||
        s_v0_online_abort_diag.captured)
    {
        return;
    }
    memset(&raw_stats, 0, sizeof(raw_stats));
    zy100_online_raw_capture_get_stats(&raw_stats);
    s_v0_online_abort_diag.captured = true;
    s_v0_online_abort_diag.fatal_error = stats->fatal_error;
    s_v0_online_abort_diag.stop_reason = (uint32_t)stats->stop_reason;
    s_v0_online_abort_diag.completion_generation =
        s_v0_completion_generation + 1U;
    s_v0_online_abort_diag.fatal_status = (uint32_t)stats->fatal_status;
    s_v0_online_abort_diag.fifo_full_count = stats->fifo_full_count;
    s_v0_online_abort_diag.fifo_lost_pkt_count =
        (uint32_t)stats->fifo_lost_pkt_count;
    s_v0_online_abort_diag.bad_header_count = stats->bad_header_count;
    s_v0_online_abort_diag.ingress_overflow_count =
        zy100_fifo_ingress_overflow_count();
    s_v0_online_abort_diag.raw_error = (uint32_t)raw_stats.error;
    s_v0_online_abort_diag.raw_accepted = raw_stats.accepted_packets;
    s_v0_online_abort_diag.raw_committed = raw_stats.committed_packets;
    s_v0_online_abort_diag.raw_ring_high_water = raw_stats.ring_high_water;
}
#endif

#if ZY100_LEGACY_OFFLINE_ENABLE
static void v0_offline_fifo_diag_reset(bool active)
{
    memset(&s_v0_offline_fifo_diag, 0, sizeof(s_v0_offline_fifo_diag));
    s_v0_offline_fifo_diag.active = active;
}

static void v0_offline_fifo_latch_fault(v0_offline_fault_t fault,
                                        const v0_fifo_stats_t *stats,
                                        uint16_t fifo_count,
                                        uint16_t req_len,
                                        uint16_t actual_len,
                                        uint8_t header,
                                        v0_offline_fifo_stage_t stage)
{
    if (!s_v0_offline_fifo_diag.active ||
        s_v0_offline_fifo_diag.first_valid)
    {
        return;
    }
    s_v0_offline_fifo_diag.first_valid = true;
    s_v0_offline_fifo_diag.first_fault = fault;
    s_v0_offline_fifo_diag.first_stage = stage;
    s_v0_offline_fifo_diag.first_header = header;
    s_v0_offline_fifo_diag.first_fifo_count = fifo_count;
    s_v0_offline_fifo_diag.first_req_len = req_len;
    s_v0_offline_fifo_diag.first_actual_len = actual_len;
    s_v0_offline_fifo_diag.first_time_ms = zy100_os_time_ms();
    s_v0_offline_fifo_diag.first_packet_seq =
        (stats != NULL) ? stats->packet_count : 0U;
}

static void v0_offline_fifo_resolve_post_stop_fault(
    const v0_fifo_stats_t *stats)
{
    v0_offline_fault_t resolved = V0_OFFLINE_FAULT_NONE;

    if (stats == NULL)
    {
        return;
    }
    if ((stats->fifo_lost_pkt_count != 0U) ||
        (s_v0_offline_fifo_diag.final_lost_raw != 0U))
    {
        resolved = V0_OFFLINE_FAULT_FIFO_LOST;
    }
    else if ((stats->fifo_full_count != 0U) ||
             ((s_v0_offline_fifo_diag.final_int_status &
               ICM53611_INT_STATUS_FIFO_FULL_INT) != 0U))
    {
        resolved = V0_OFFLINE_FAULT_FIFO_FULL;
    }
    if (resolved != V0_OFFLINE_FAULT_NONE)
    {
        s_v0_offline_fifo_diag.first_fault = resolved;
        s_v0_offline_fifo_diag.first_valid = true;
    }
}
#endif

#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
static zy100_online_trace_imu_phase_t v0_online_pause_trace_phase(
    zy100_online_pause_phase_t phase)
{
    switch (phase)
    {
    case ZY100_ONLINE_PAUSE_PHASE_STOP_LIVE:
        return ZY100_ONLINE_TRACE_IMU_STOP_LIVE;
    case ZY100_ONLINE_PAUSE_PHASE_DRAIN_FIFO_REPLAY:
        return ZY100_ONLINE_TRACE_IMU_DRAIN;
    case ZY100_ONLINE_PAUSE_PHASE_FINISH_ACTIVE_RECORD:
        return ZY100_ONLINE_TRACE_IMU_FINISH_RECORD;
    case ZY100_ONLINE_PAUSE_PHASE_RAW_FINAL:
        return ZY100_ONLINE_TRACE_IMU_RAW_FINAL;
    case ZY100_ONLINE_PAUSE_PHASE_RECORD_FINAL:
        return ZY100_ONLINE_TRACE_IMU_RECORD_FINAL;
    case ZY100_ONLINE_PAUSE_PHASE_STOP_DONE:
        return ZY100_ONLINE_TRACE_IMU_STOP_DONE;
    case ZY100_ONLINE_PAUSE_PHASE_ABORT:
        return ZY100_ONLINE_TRACE_IMU_ABORT;
    case ZY100_ONLINE_PAUSE_PHASE_IDLE:
    default:
        return ZY100_ONLINE_TRACE_IMU_IDLE;
    }
}

static void v0_online_trace_runtime(void)
{
    zy100_online_reset_trace_set_runtime(
        (uint8_t)spi_bus_current_owner(),
        (uint8_t)imu_bsp_ois_tick_timer_get_owner(),
        imu_bsp_ois_tick_timer_is_running(),
        imu_fifo_drain_test_final_edge_b_critical_active(),
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
        zy100_final_edge_raw_store_has_pending_work(),
#else
        false,
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE
        zy100_final_edge_record_store_has_pending_work());
#else
        false);
#endif
}

static void v0_online_pause_set_phase(zy100_online_pause_phase_t phase)
{
    uint32_t now_ms = zy100_os_time_ms();
    uint32_t lock_state = os_lock();
    bool phase_changed = false;

    if (s_v0_online_pause_progress.phase != phase)
    {
        phase_changed = true;
        s_v0_online_pause_progress.phase = phase;
        s_v0_online_pause_progress.phase_start_ms = now_ms;
        s_v0_online_pause_progress.last_progress_ms = now_ms;
        s_v0_online_pause_progress.progress_seq++;
    }
    s_v0_online_pause_progress.active =
        (phase != ZY100_ONLINE_PAUSE_PHASE_IDLE) &&
        (phase != ZY100_ONLINE_PAUSE_PHASE_STOP_DONE);
    s_v0_online_pause_progress.abort_requested =
        s_v0_online_abort_requested;
    os_unlock(lock_state);
    if (phase_changed)
    {
        zy100_online_reset_trace_set_imu_phase(
            v0_online_pause_trace_phase(phase));
        v0_online_trace_runtime();
        /* Task context, after unlocking; no sample/ISR logging. */
        ZY100_LOG_DETAIL("[ONLINE_STOP] phase=%u", (uint32_t)phase);
    }
}

static void v0_online_pause_note_progress(void)
{
    uint32_t lock_state = os_lock();

    s_v0_online_pause_progress.progress_seq++;
    s_v0_online_pause_progress.last_progress_ms = zy100_os_time_ms();
    os_unlock(lock_state);
}

static void v0_online_pause_reset(void)
{
    uint32_t lock_state = os_lock();

    memset(&s_v0_online_pause_progress, 0,
           sizeof(s_v0_online_pause_progress));
    s_v0_online_abort_requested = false;
    os_unlock(lock_state);
    zy100_online_reset_trace_set_imu_phase(ZY100_ONLINE_TRACE_IMU_LIVE);
}
#else
#define v0_online_pause_set_phase(phase) do { (void)(phase); } while (0)
#define v0_online_pause_note_progress() do { } while (0)
#define v0_online_pause_reset() do { } while (0)
#define v0_online_trace_runtime() do { } while (0)
#endif

#if ZY100_FINAL_EDGE_MODE_ENABLE
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
typedef enum
{
    ZY100_FE_REPLAY_SRC_NOHIT = 0U,
    ZY100_FE_REPLAY_SRC_TRUEHIT = 1U,
} zy100_fe_replay_source_t;

typedef enum
{
    V0_FE_REPLAY_SERVICE_NO_WORK = 0U,
    V0_FE_REPLAY_SERVICE_FED_OR_DONE,
    V0_FE_REPLAY_SERVICE_PRESSURE_YIELD,
    V0_FE_REPLAY_SERVICE_ERROR,
} v0_fe_replay_service_result_t;
#endif

typedef enum
{
    V0_FE_B_CRITICAL_IDLE = 0U,
    V0_FE_B_CRITICAL_PENDING,
    V0_FE_B_CRITICAL_ACTIVE,
} v0_fe_b_critical_state_t;

typedef struct
{
    uint32_t packet_count;
    uint32_t edge_feed_count;
    uint32_t rt_feed_count;
    uint32_t rt_trigger_count;
    uint32_t rt_deferred_count;
    uint32_t rt_gated_count;
    uint32_t edge_summary_count;
    uint32_t edge_event_count;
    uint32_t edge_raw_req_count;
    uint32_t edge_raw_sup_count;
    uint32_t replay_begin_count;
    uint32_t replay_done_count;
    uint32_t replay_fail_count;
    uint32_t replay_nohit_count;
    uint32_t replay_truehit_count;
    uint32_t replay_truehit_begin_count;
    uint32_t replay_truehit_done_count;
    uint32_t replay_truehit_fail_count;
    uint32_t replay_od_warn_count;
    uint32_t replay_ds_bad_count;
    uint32_t replay_ds_overflow_count;
    uint32_t replay_samples_expected;
    uint32_t replay_samples_fed;
    uint32_t replay_edge_feed;
    uint32_t replay_edge_error;
    uint32_t replay_truehit_samples_expected;
    uint32_t replay_truehit_samples_fed;
    uint32_t replay_truehit_edge_feed;
    uint32_t replay_truehit_edge_error;
    uint32_t replay_summary_count;
    uint32_t replay_event_count;
    uint32_t replay_raw_sup_count;
    uint32_t replay_batch_max_us;
    uint32_t replay_sample_max_us;
    uint32_t replay_total_us;
    uint32_t replay_last_us;
    uint32_t replay_fifo_high_count;
    uint32_t replay_fifo_hard_count;
    uint32_t replay_fifo_max_count;
    uint32_t replay_batch_shrink_count;
    uint32_t replay_pressure_interleave_count;
    uint32_t replay_pressure_interleave_packets;
    uint32_t replay_pressure_interleave_max_fifo;
    uint32_t replay_pressure_interleave_last_bscan_id;
    uint32_t replay_long_sample_bscan_id;
    uint32_t replay_long_batch_bscan_id;
    uint32_t replay_backlog_edge_only_count;
    uint32_t replay_backlog_max_packets;
    uint32_t replay_queue_sum_drop_delta;
    uint32_t replay_queue_evt_drop_delta;
    uint32_t b_scan_start_reject_raw_replay_count;
    uint32_t b_critical_pending_count;
    uint32_t b_critical_active_count;
    uint32_t b_critical_clear_count;
    uint32_t b_critical_replay_blocked_count;
    uint32_t b_critical_raw_blocked_count;
    uint32_t b_critical_record_blocked_count;
    uint32_t b_critical_flash_blocked_count;
    uint32_t b_critical_offline_bypass_count;
    uint32_t b_critical_offline_post_quiet_bypass_count;
    uint32_t b_start_raw_busy_count;
    uint32_t b_start_replay_busy_count;
    uint32_t b_start_spi_retry_count;
    uint32_t rt_request_expired_count;
    uint32_t b_restore_fifo_flush_count;
    uint32_t b_restore_fifo_flush_fail_count;
    uint32_t b_restore_fifo_flush_last_status;
    uint32_t b_restore_fifo_flush_last_count;
    uint32_t b_pending_to_first_frame_last_us;
    uint32_t b_pending_to_first_frame_max_us;
    uint32_t b_first_frame_read_done_last_us;
    uint32_t rt_gated_raw_store_count;
    uint32_t rt_gated_replay_count;
    uint32_t rt_gated_replay_truehit_count;
    uint32_t rt_gated_replay_backlog_count;
    uint32_t rt_gated_other_count;
    uint32_t hdr_bad_count;
    uint32_t dt_bad_count;
    uint32_t dt_min_us;
    uint32_t dt_max_us;
    uint32_t timer_due_count;
    uint32_t timer_due_consumed_count;
    uint32_t timer_due_max;
    uint32_t timer_due_coalesced_count;
    uint32_t timer_due_capped_count;
    uint32_t timer_empty_count;
    uint32_t timer_int_wake_count;
    uint32_t service_count;
    uint32_t service_gap_sum_us;
    uint32_t service_gap_max_us;
    uint32_t runtime_service_gap_max_us;
    uint32_t final_service_gap_max_us;
    uint32_t service_last_us;
    uint32_t service_last_ms;
    uint32_t drain_count;
    uint32_t drain_packet_sum;
    uint32_t drain_packet_max;
    uint32_t drain_16_count;
    uint32_t drain_32_count;
    uint32_t drain_64_count;
    uint32_t drain_128_count;
    uint32_t drain_256_count;
    uint32_t drain_512_count;
    uint32_t fifo_max_count;
    uint32_t fifo_emergency_count;
    uint32_t edge_time_sum_us;
    uint32_t edge_time_max_us;
    uint32_t rt_time_sum_us;
    uint32_t rt_time_max_us;
    bool dt_seen;
    bool timing_api_ok;
} v0_final_edge_phase1_stats_t;

typedef struct
{
    uint32_t page_program;
    uint32_t erase;
    uint32_t raw_saved;
    uint32_t summary_flash;
    uint32_t event_flash;
} v0_final_edge_flash_stats_t;

typedef struct
{
    uint32_t check_count;
    uint32_t last_raw_used;
    uint32_t last_summary_used;
    uint32_t last_event_used;
    uint32_t last_meta_used;
    uint32_t max_raw_used;
    uint32_t max_summary_used;
    uint32_t max_event_used;
    uint32_t max_meta_used;
    uint32_t stop_count;
    uint32_t stop_last_region;
    uint32_t stop_last_used;
    uint32_t stop_last_total;
    uint32_t stop_last_pct;
} v0_final_edge_wm_summary_t;

typedef struct
{
    uint8_t reached_summary_ok;
    uint8_t fatal_ok;
    uint8_t fifo_full_ok;
    uint8_t fifo_lost_ok;
    uint8_t bad_header_ok;
    uint8_t dt_seen_ok;
    uint8_t dt_bad_ok;
    uint8_t dt_range_ok;
    uint8_t packet_nonzero_ok;
    uint8_t edge_feed_ok;
    uint8_t b_scan_count_ok;
    uint8_t b_scan_fail_ok;
    uint8_t b_last_err_ok;
    uint8_t restore_ok;
    uint8_t flash_zero_ok;
    uint8_t final_pass;
    uint8_t phase6_seen;
    uint8_t phase6_replay_ok;
    uint8_t phase6_done_ok;
    uint8_t phase6_nohit_ok;
    uint8_t phase6_truehit_ok;
    uint8_t phase6_err;
    uint8_t phase6_inc;
    uint8_t phase6_ds_ok;
    uint8_t phase6_edge_ok;
    uint8_t phase6_pending_ok;
    uint8_t phase6_gate_ok;
    uint8_t phase6_backlog_ok;
    uint8_t phase7_seen;
    uint8_t phase7_summary_ok;
    uint8_t phase7_event_ok;
    uint8_t phase7_meta_ok;
    uint8_t phase7_err;
    uint8_t phase7_inc;
    uint8_t b_frame_ok;
    uint8_t b_hwin_ok;
    uint8_t b_nohit1280_ok;
    uint8_t b_truehit_short;
    uint32_t b_scan_fail_count;
    uint32_t b_fail_reason_or;
    uint32_t b_fail_last_id;
    uint32_t b_fail_last_reason;
    uint32_t b_fail_hard_count;
    uint32_t b_fail_frame_count;
    uint32_t b_fail_hwin_count;
    uint32_t b_fail_trunc_count;
    uint32_t b_fail_last_hit;
    uint32_t b_fail_last_frames;
    uint32_t nohit_od_warn_count;
    uint32_t nohit_od_warn_or;
    uint32_t nohit_od_warn_last_id;
    uint32_t nohit_od_warn_last_reason;
    uint32_t nohit_od_warn_last_run;
    uint32_t truehit_od_warn_count;
    uint32_t truehit_od_warn_or;
    uint32_t truehit_od_warn_last_id;
    uint32_t truehit_od_warn_last_reason;
    uint32_t truehit_od_warn_last_run;
    uint32_t phase7_summary_saved;
    uint32_t phase7_summary_enqueued;
    uint32_t phase7_event_saved;
    uint32_t phase7_event_enqueued;
    uint32_t phase7_pending;
    uint32_t phase7_full;
    uint32_t phase7_verify;
    uint32_t phase7_write;
    uint32_t phase7_bflash;
    uint32_t phase7_runtime_erase;
    uint8_t phase8_stop;
    uint8_t phase8_drain;
    uint8_t phase8_export;
    uint8_t phase8_clear;
    uint8_t phase8_err;
    uint8_t phase8_reason_ok;
    uint8_t phase8_ready;
    uint8_t phase8_exporting;
    uint8_t phase8_done;
    uint8_t phase8_preserved;
    uint8_t phase8_cleared;
    uint8_t phase8_raw_idle;
    uint8_t phase8_rec_idle;
    uint8_t phase8_meta_ok;
    uint8_t phase8_fifo_safe;
    uint8_t phase8_b_idle;
} v0_final_edge_pass_diag_t;

#if ZY100_FINAL_EDGE_PHASE4_HIT_WINDOW_ENABLE
typedef struct
{
    uint32_t hit_window_seen;
    uint32_t hit_window_ok;
    uint32_t hit_window_fail;
    uint32_t b_hwin_ok;
    uint32_t b_hwin_fail;
    uint32_t early_hit_count;
    uint32_t early_lf_ok;
    uint32_t early_lf_fail;
    uint32_t post_trunc_count;
    uint32_t window_xor;
    uint8_t self_pass;
    uint8_t self_early;
    uint8_t self_tail;
} v0_final_edge_phase4_stats_t;
#endif

#if ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
#define V0_FINAL_EDGE_HIT20_CALIB_COUNT 7U

#define FE_BFAIL_REASON_ERR          (1UL << 0)
#define FE_BFAIL_REASON_READ_ERR     (1UL << 1)
#define FE_BFAIL_REASON_OVER         (1UL << 2)
#define FE_BFAIL_REASON_DROP         (1UL << 3)
#define FE_BFAIL_REASON_NOHIT_FRAME  (1UL << 4)
#define FE_BFAIL_REASON_HWIN_INVALID (1UL << 5)
#define FE_BFAIL_REASON_HWIN_INCOMP  (1UL << 6)
#define FE_BFAIL_REASON_POST_TRUNC   (1UL << 7)
#define FE_BFAIL_REASON_OWNER        (1UL << 8)
#define FE_BFAIL_REASON_CLEANUP      (1UL << 9)
#define FE_BFAIL_REASON_NULL_RESULT  (1UL << 10)
#define FE_BFAIL_REASON_RUN_FAIL     (1UL << 11)
#define FE_BFAIL_REASON_STALE        (1UL << 12)
#define FE_BFAIL_REASON_DT_BAD       (1UL << 13)
#define FE_BFAIL_REASON_ACQ_OVERFLOW (1UL << 14)
#define FE_BFAIL_REASON_ACQ_READ_ERR (1UL << 15)
#define FE_BFAIL_REASON_ACQ_IRQ_LATE (1UL << 16)
#define FE_BFAIL_REASON_ACQ_BACKLOG  (1UL << 17)
#define FE_BFAIL_REASON_ACQ_TMST_MISMATCH (1UL << 18)
#define FE_BFAIL_REASON_BEGIN_FAIL   (1UL << 19)

typedef struct
{
    uint32_t od_scan_count;
    uint32_t od_hit_scan_count;
    uint32_t od_nohit_scan_count;
    uint32_t od_early_events;
    uint32_t od_mid_events;
    uint32_t od_tail_events;
    uint32_t od_post_events;
    uint32_t od_max_pending;
    uint32_t tmst_delta_156_157_count;
    uint32_t tmst_delta_zero_count;
    uint32_t tmst_delta_double_count;
    uint32_t tmst_delta_bad_count;
    uint32_t tmst_delta_other_bad_count;
    uint16_t tmst_delta_min;
    uint16_t tmst_delta_max;
    bool tmst_delta_seen;
    uint32_t tmst_delta_first_bad_scan_id;
    uint32_t tmst_delta_last_bad_scan_id;
    uint16_t tmst_delta_first_bad_seq;
    uint16_t tmst_delta_last_bad_seq;
    uint32_t read_us_max;
    uint32_t read_us_max_scan_id;
    uint16_t read_us_max_seq;
} v0_final_edge_od_diag_stats_t;

#if ZY100_ONLINE_STREAM_ENABLE
typedef struct
{
    bool active;
    uint8_t summary_stage;
    bool live_tail_valid;
    bool current_b_a1_valid;
    bool current_b_last_valid;
    bool awaiting_a2_first;
    uint32_t live_tail_seq;
    uint16_t live_tail_tmst_raw;
    uint32_t current_b_id;
    uint32_t current_a1_seq;
    uint16_t current_a1_tmst_raw;
    uint16_t current_b_last_tmst_raw;
    uint32_t b_count;
    uint32_t b_fail_count;
    uint32_t a1_valid_count;
    uint32_t a2_valid_count;
    uint32_t a2_missing_count;
    uint32_t b_frames_total;
    uint32_t b_frames_max;
    uint32_t b_dt_bad_total;
    uint32_t b_stale_total;
    uint32_t b_raw_mismatch_total;
    uint32_t b_delta_156_157_total;
    uint32_t b_delta_bad_total;
    uint32_t b_delta_zero_total;
    uint32_t b_delta_double_total;
    uint16_t b_delta_min;
    uint16_t b_delta_max;
    bool b_delta_seen;
    uint32_t gap_a1_b_count;
    uint32_t gap_a1_b_last_us;
    uint32_t gap_a1_b_min_us;
    uint32_t gap_a1_b_max_us;
    uint32_t gap_b_a2_count;
    uint32_t gap_b_a2_last_us;
    uint32_t gap_b_a2_min_us;
    uint32_t gap_b_a2_max_us;
    uint32_t last_b_id;
    uint32_t last_b_first_tmst_raw;
    uint32_t last_b_last_tmst_raw;
    uint32_t last_a2_seq;
    uint32_t last_a2_tmst_raw;
} v0_online_edge_diag_t;
#endif

#if ZY100_FINAL_EDGE_HIT20_CALIB_ENABLE
typedef struct
{
    uint32_t peak20_min;
    uint32_t peak20_max;
    uint64_t peak20_sum;
    uint32_t peak20_count;
    uint32_t peak20_over_count[V0_FINAL_EDGE_HIT20_CALIB_COUNT];
    uint32_t cross_count[V0_FINAL_EDGE_HIT20_CALIB_COUNT];
    uint32_t hit_scan_count;
    uint32_t nohit_scan_count;
} v0_final_edge_hit20_calib_stats_t;

static const uint32_t s_final_edge_hit20_calib_thresholds
    [V0_FINAL_EDGE_HIT20_CALIB_COUNT] =
{
    ZY100_FINAL_EDGE_HIT20_CALIB_TH0,
    ZY100_FINAL_EDGE_HIT20_CALIB_TH1,
    ZY100_FINAL_EDGE_HIT20_CALIB_TH2,
    ZY100_FINAL_EDGE_HIT20_CALIB_TH3,
    ZY100_FINAL_EDGE_HIT20_CALIB_TH4,
    ZY100_FINAL_EDGE_HIT20_CALIB_TH5,
    ZY100_FINAL_EDGE_HIT20_CALIB_TH6,
};
#endif

static uint8_t s_final_edge_b_ois_ring
    [ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES * ZY100_FINAL_EDGE_OIS_FRAME_BYTES];
static zy100_final_edge_bscan_result_t s_final_edge_b_last_result;
static v0_final_edge_od_diag_stats_t s_final_edge_od_diag_stats;
#if ZY100_ONLINE_STREAM_ENABLE
static v0_online_edge_diag_t s_v0_online_edge_diag;
#endif
#if ZY100_FINAL_EDGE_HIT20_CALIB_ENABLE
static v0_final_edge_hit20_calib_stats_t s_final_edge_hit20_calib_stats;
#endif
static volatile v0_fe_b_critical_state_t s_final_edge_b_critical_state =
    V0_FE_B_CRITICAL_IDLE;
static uint32_t s_final_edge_b_pending_start_us = 0U;
#if ZY100_FINAL_EDGE_B_CRITICAL_GATE_ENABLE && \
    ZY100_FINAL_EDGE_OFFLINE_B_CRITICAL_BYPASS_VERIFY_ENABLE
static bool s_final_edge_b_critical_offline_bypass_logged = false;
#endif
#if (ZY100_FINAL_EDGE_POST_RESTORE_QUIET_MS != 0U)
static volatile uint32_t s_final_edge_post_restore_quiet_until_ms = 0U;
#endif
static bool s_final_edge_b_scan_active = false;
static bool s_final_edge_rt_gated_for_b = false;
static bool s_final_edge_rt_trigger_valid = false;
static bool s_final_edge_resume_ok = false;
static uint16_t s_final_edge_rt_trigger_tmst_raw = 0U;
static uint32_t s_final_edge_b_scan_count = 0U;
static uint32_t s_final_edge_b_scan_fail_count = 0U;
static uint32_t s_final_edge_b_fail_reason_or = 0U;
static uint32_t s_final_edge_b_fail_last_reason = 0U;
static uint32_t s_final_edge_b_fail_last_id = 0U;
static uint32_t s_final_edge_b_fail_last_frames = 0U;
static uint32_t s_final_edge_b_fail_last_hit = 0U;
static uint32_t s_final_edge_b_fail_last_exit_reason = 0U;
static uint32_t s_final_edge_b_fail_last_seq =
    ZY100_FINAL_EDGE_BSCAN_SEQ32_INVALID;
static uint32_t s_final_edge_b_fail_last_elapsed_ms = 0U;
static uint32_t s_final_edge_b_fail_hard_count = 0U;
static uint32_t s_final_edge_b_fail_frame_count = 0U;
static uint32_t s_final_edge_b_fail_hwin_count = 0U;
static uint32_t s_final_edge_b_fail_trunc_count = 0U;
static uint32_t s_final_edge_b_begin_fail_count = 0U;
static uint32_t s_final_edge_b_begin_fail_last_stage = 0U;
static uint32_t s_final_edge_b_begin_fail_last_status = 0U;
static uint32_t s_final_edge_b_begin_fail_last_retry = 0U;
static uint32_t s_final_edge_b_begin_fail_last_owner = 0U;
static uint32_t s_final_edge_b_begin_fail_last_cleanup = 0U;
static uint32_t s_final_edge_b_nohit_frame_min = 0U;
static uint32_t s_final_edge_b_nohit_frame_max = 0U;
static uint32_t s_final_edge_b_nohit_frame_last = 0U;
static uint32_t s_final_edge_b_nohit_frame_sum_short = 0U;
static uint32_t s_final_edge_b_nohit_frame_last_exit = 0U;
static uint32_t s_final_edge_b_mismatch_count = 0U;
static uint32_t s_final_edge_b_mismatch_hit_count = 0U;
static uint32_t s_final_edge_b_mismatch_nohit_count = 0U;
static uint32_t s_final_edge_b_mismatch_last_id = 0U;
static uint32_t s_final_edge_b_mismatch_last_seq =
    ZY100_FINAL_EDGE_BSCAN_SEQ32_INVALID;
static uint32_t s_final_edge_b_mismatch_last_probe = 0U;
static uint32_t s_final_edge_b_mismatch_last_raw = 0U;
static uint32_t s_final_edge_b_mismatch_soft_drop_count = 0U;
static uint32_t s_final_edge_b_mismatch_soft_last_id = 0U;
static uint32_t s_final_edge_b_mismatch_soft_last_probe = 0U;
static uint32_t s_final_edge_b_mismatch_soft_last_raw = 0U;
static uint32_t s_final_edge_b_mismatch_soft_limit = 0U;
static uint32_t s_final_edge_nohit_od_warn_count = 0U;
static uint32_t s_final_edge_nohit_od_warn_or = 0U;
static uint32_t s_final_edge_nohit_od_warn_last_id = 0U;
static uint32_t s_final_edge_nohit_od_warn_last_reason = 0U;
static uint32_t s_final_edge_nohit_od_warn_last_run = 0U;
static uint32_t s_final_edge_nohit_od_warn_last_frames = 0U;
static uint32_t s_final_edge_nohit_od_warn_last_err = 0U;
static uint32_t s_final_edge_nohit_od_warn_last_over = 0U;
static uint32_t s_final_edge_nohit_od_warn_last_drop = 0U;
static uint32_t s_final_edge_truehit_od_warn_count = 0U;
static uint32_t s_final_edge_truehit_od_warn_or = 0U;
static uint32_t s_final_edge_truehit_od_warn_last_id = 0U;
static uint32_t s_final_edge_truehit_od_warn_last_reason = 0U;
static uint32_t s_final_edge_truehit_od_warn_last_run = 0U;
static uint32_t s_final_edge_truehit_od_warn_last_frames = 0U;
static uint32_t s_final_edge_truehit_od_warn_last_over = 0U;
static uint32_t s_final_edge_truehit_od_warn_last_drop = 0U;
static uint32_t s_final_edge_restore_ok_count = 0U;
static uint32_t s_final_edge_restore_fail_count = 0U;
static uint32_t s_final_edge_official_hit_count = 0U;
static uint32_t s_final_edge_official_nohit_count = 0U;
static uint32_t s_final_edge_last_b_id = 0U;
static uint32_t s_final_edge_rt_trigger_sample_index = 0U;
static uint32_t s_final_edge_restore_fallback_count = 0U;
static uint32_t s_final_edge_restore_fallback_status = 0U;
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
static bool s_final_edge_replay_pending = false;
static bool s_final_edge_replay_active = false;
static bool s_final_edge_replay_backlog_edge_only_active = false;
static uint32_t s_final_edge_replay_index = 0U;
static uint32_t s_final_edge_replay_count = 0U;
static uint32_t s_final_edge_replay_bscan_id = 0U;
static uint32_t s_final_edge_replay_started_ms = 0U;
static uint32_t s_final_edge_replay_last_ms = 0U;
static uint32_t s_final_edge_replay_backlog_start_count = 0U;
static zy100_fe_replay_source_t s_final_edge_replay_source =
    ZY100_FE_REPLAY_SRC_NOHIT;
#endif
#if ZY100_FINAL_EDGE_PHASE4_HIT_WINDOW_ENABLE
static v0_final_edge_phase4_stats_t s_final_edge_phase4_stats;
static zy100_final_edge_hit_window_view_t s_final_edge_last_hit_window_view;
static zy100_final_edge_lf_pre_query_t s_final_edge_last_lf_pre_query;
#endif
#endif

static v0_final_edge_phase1_stats_t s_final_edge_phase1_stats;
static v0_final_edge_flash_stats_t s_final_edge_flash_stats;
static v0_final_edge_wm_summary_t s_final_edge_wm_summary;
static bool s_final_edge_rt_deferred_active = false;
static uint32_t s_final_edge_edge_sample_index = 0U;
static uint32_t s_final_edge_last_rt_trigger_ms = 0U;
static volatile uint32_t s_final_edge_timer_due_count = 0U;
static volatile uint32_t s_final_edge_timer_total_due_count = 0U;
static volatile uint32_t s_final_edge_timer_last_us = 0U;
static volatile uint32_t s_final_edge_timer_gap_max_us = 0U;
static volatile uint32_t s_final_edge_timer_notify_fail_count = 0U;
static volatile bool s_final_edge_timer_running = false;
static bool s_final_edge_record_prepare_started = false;
typedef enum
{
    V0_FE_PREPARE_MODE_CLEAR_ALL = 0U,
    V0_FE_PREPARE_MODE_APPEND_SESSION = 1U,
} v0_fe_prepare_mode_t;
static v0_fe_prepare_mode_t s_final_edge_prepare_mode =
    V0_FE_PREPARE_MODE_CLEAR_ALL;
static bool s_final_edge_dir_prepare_started = false;
static bool s_final_edge_raw_prepare_started = false;
static bool s_final_edge_online_spool_prepare_started = false;
#if ZY100_ONLINE_DIRECT_SPOOL_ENABLE
static bool s_final_edge_online_target_prepare = false;
#endif
static bool s_final_edge_active_session_valid = false;
static bool s_final_edge_empty_error_aborted = false;
static zy100_session_alloc_t s_final_edge_active_session_alloc;
static uint32_t s_final_edge_store_prep_last_phase = 0U;
static uint32_t s_final_edge_store_prep_last_done = 0xFFFFFFFFU;
static uint32_t s_final_edge_store_prep_last_total = 0U;
static uint32_t s_final_edge_store_prep_last_err = 0U;
static bool s_final_edge_store_prep_done_logged = false;
#endif

static void v0_imu_irq_callback(void);
static void v0_reset_fifo_drain_diag_ring(void);
static bool v0_refresh_lost_count(v0_fifo_stats_t *stats);
static bool v0_read_int_status(v0_fifo_stats_t *stats, icm53611_int_status_t *snapshot);
#if V0_FLASH_RUNTIME_ENABLE
static bool v0_drain_fifo_packets(v0_fifo_stats_t *stats);
#endif
static void v0_fill_phase_a_ui_fifo_config(icm53611_cfg_t *imu_cfg,
                                           icm53611_fifo_cfg_t *fifo_cfg);
static bool v0_verify_ui_fifo_rate_config(v0_fifo_stats_t *stats);
#if !V0_FLASH_RUNTIME_ENABLE && ZY100_IMU_STARTUP_DBG_LOG_ENABLE
static void v0_dbg_log_init_readback(void);
static void v0_dbg_fifo_count_after_start(void);
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE
static bool v0_final_edge_live_service_handler(v0_fifo_stats_t *stats,
                                               bool flush);
#if ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
static bool v0_final_edge_b_critical_active(void);
static bool v0_final_edge_consume_deferred_b_scan(v0_fifo_stats_t *stats);
#endif
static uint32_t v0_final_edge_next_edge_sample_index(void);
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
static bool v0_final_edge_replay_busy(void);
static bool v0_final_edge_replay_needs_service(void);
#endif
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
static void v0_final_edge_raw_store_pump_safe(void);
static void v0_final_edge_raw_store_log_drop(
    const zy100_fe_raw_store_stats_t *raw_stats);
#if ZY100_ONLINE_STREAM_ENABLE
static void v0_final_edge_online_abort_raw_pending(
    zy100_online_drop_reason_t reason);
#endif
static bool v0_final_edge_raw_store_final_pump(void);
#endif
static bool v0_final_edge_check_flash_watermark_stop(void);
static void v0_final_edge_set_state(zy100_fe_capture_state_t state);
static void v0_final_edge_request_stop(zy100_fe_stop_reason_t reason,
                                       const char *src);
#if ZY100_ONLINE_STREAM_ENABLE
static bool v0_final_edge_online_stop_active(void);
static void v0_final_edge_request_online_pause_stop(
    imu_fifo_drain_test_stop_reason_t reason);
#endif
static bool v0_final_edge_prepare_no_new_data_for_drain(
    v0_fifo_stats_t *stats);
static bool v0_final_edge_drain_fifo_tail_once(v0_fifo_stats_t *stats,
                                               bool *fifo_empty,
                                               uint16_t *fifo_before_out,
                                               uint16_t *fifo_after_out,
                                               uint16_t *drained_out,
                                               uint32_t *edge_out);
static void v0_final_edge_note_drain_len(uint16_t drain_len);
static void v0_final_edge_record_store_pump_safe(void);
static bool v0_final_edge_record_store_final_pump(void);
static bool v0_final_edge_write_session_meta(const v0_fifo_stats_t *stats,
                                             uint32_t capture_start_ms,
                                              uint32_t final_runtime_ms,
                                              const v0_final_edge_pass_diag_t *diag);
static void v0_final_edge_log_ois_summary(void);
static void v0_final_edge_log_bscan_end_summary(void);
static void v0_final_edge_log_raw_prepare_progress(
    zy100_flash_prepare_status_t status);
#endif

#if ZY100_SRAM_SUMMARY_LOG_ENABLE
#define ZY100_SRAM_HEAP_API_AVAILABLE 1

#if (INCLUDE_uxTaskGetStackHighWaterMark == 1)
#define ZY100_SRAM_STACK_API_AVAILABLE 1
#else
#define ZY100_SRAM_STACK_API_AVAILABLE 0
#endif

typedef struct
{
    uint32_t heap_start_free;
    uint32_t heap_end_free;
    uint32_t heap_min_free;
    uint32_t heap_max_used;
    uint32_t cap_start_free;
    uint32_t cap_end_free;
    uint32_t cap_min_free;
    uint32_t cap_max_used;
    uint32_t v0_stack_free_start;
    uint32_t v0_stack_free_now;
    uint32_t v0_stack_free_min;
    uint32_t sram_total;
    uint32_t heap_total;
    uint32_t static_used;
    uint32_t algo_workspace_reserved;
    uint32_t algo_workspace_used;
    bool heap_api_ok;
    bool stack_api_ok;
    bool sram_total_known;
} zy100_sram_stats_t;

static uint32_t v0_sram_size_to_u32(size_t value)
{
    return (value > 0xFFFFFFFFUL) ? 0xFFFFFFFFU : (uint32_t)value;
}

static uint32_t v0_sram_heap_free_now(void)
{
    return v0_sram_size_to_u32(xPortGetFreeHeapSize(RAM_TYPE_DATA_ON));
}

static uint32_t v0_sram_heap_min_ever_free(void)
{
    return v0_sram_size_to_u32(xPortGetMinimumEverFreeHeapSize(RAM_TYPE_DATA_ON));
}

static uint32_t v0_sram_stack_free_now(void)
{
#if ZY100_SRAM_STACK_API_AVAILABLE
    return ((uint32_t)uxTaskGetStackHighWaterMark(NULL)) * (uint32_t)sizeof(StackType_t);
#else
    return 0U;
#endif
}

#if (ZY100_SRAM_TOTAL_BYTES != 0U)
static uint32_t v0_sram_static_used_bytes(void)
{
#if defined(__CC_ARM)
    extern unsigned int Image$$RAM_DATA_ON$$RO$$Length;
    extern unsigned int Image$$RAM_DATA_ON$$RW$$Length;
    extern unsigned int Image$$RAM_DATA_ON$$ZI$$Length;

    return (uint32_t)&Image$$RAM_DATA_ON$$RO$$Length +
           (uint32_t)&Image$$RAM_DATA_ON$$RW$$Length +
           (uint32_t)&Image$$RAM_DATA_ON$$ZI$$Length;
#else
    return 0U;
#endif
}
#endif

static void v0_sram_fill_layout(zy100_sram_stats_t *stats)
{
    if (stats == NULL)
    {
        return;
    }

#if (ZY100_SRAM_TOTAL_BYTES != 0U)
    stats->sram_total_known = true;
    stats->sram_total = ZY100_SRAM_TOTAL_BYTES;
    stats->heap_total = HEAP_DATA_ON_SIZE;
    stats->static_used = v0_sram_static_used_bytes();
#else
    stats->sram_total_known = false;
    stats->sram_total = 0U;
    stats->heap_total = 0U;
    stats->static_used = 0U;
#endif
}

static void v0_sram_reset(zy100_sram_stats_t *stats)
{
    if (stats == NULL)
    {
        return;
    }

    memset(stats, 0, sizeof(*stats));
    stats->heap_api_ok = (ZY100_SRAM_HEAP_API_AVAILABLE != 0);
    stats->stack_api_ok = (ZY100_SRAM_STACK_API_AVAILABLE != 0);
    stats->algo_workspace_reserved = zy100_edge_shadow_workspace_reserved_bytes();
    stats->algo_workspace_used = zy100_edge_shadow_workspace_used_est_bytes();
    v0_sram_fill_layout(stats);
}

static uint32_t v0_sram_capture_heap_drop(const zy100_sram_stats_t *stats)
{
    if ((stats == NULL) ||
        (stats->cap_start_free == 0U) ||
        (stats->cap_min_free == 0U) ||
        (stats->cap_start_free < stats->cap_min_free))
    {
        return 0U;
    }

    return stats->cap_start_free - stats->cap_min_free;
}

static void v0_sram_capture_start(zy100_sram_stats_t *stats)
{
    uint32_t heap_free;
    uint32_t heap_min_free;
    uint32_t stack_free;

    v0_sram_reset(stats);
    if (stats == NULL)
    {
        return;
    }

    if (stats->heap_api_ok)
    {
        heap_free = v0_sram_heap_free_now();
        heap_min_free = v0_sram_heap_min_ever_free();
        stats->heap_start_free = heap_free;
        stats->heap_end_free = heap_free;
        stats->heap_min_free = (heap_min_free != 0U) ? heap_min_free : heap_free;
        stats->cap_start_free = heap_free;
        stats->cap_end_free = heap_free;
        stats->cap_min_free = heap_free;
    }

    if (stats->stack_api_ok)
    {
        stack_free = v0_sram_stack_free_now();
        stats->v0_stack_free_start = stack_free;
        stats->v0_stack_free_now = stack_free;
        stats->v0_stack_free_min = stack_free;
    }
}

static void v0_sram_capture_sample(zy100_sram_stats_t *stats)
{
    uint32_t heap_free;
    uint32_t stack_free;

    if (stats == NULL)
    {
        return;
    }

    if (stats->heap_api_ok)
    {
        heap_free = v0_sram_heap_free_now();
        if ((heap_free != 0U) &&
            (stats->cap_min_free != 0U) &&
            (heap_free < stats->cap_min_free))
        {
            stats->cap_min_free = heap_free;
        }
        if ((heap_free != 0U) &&
            ((stats->heap_min_free == 0U) || (heap_free < stats->heap_min_free)))
        {
            stats->heap_min_free = heap_free;
        }
    }

    if (stats->stack_api_ok)
    {
        stack_free = v0_sram_stack_free_now();
        if (stack_free < stats->v0_stack_free_min)
        {
            stats->v0_stack_free_min = stack_free;
        }
        stats->v0_stack_free_now = stack_free;
    }
}

static void v0_sram_capture_end(zy100_sram_stats_t *stats)
{
    uint32_t heap_free;
    uint32_t heap_min_free;

    if (stats == NULL)
    {
        return;
    }

    if (stats->heap_api_ok)
    {
        heap_free = v0_sram_heap_free_now();
        heap_min_free = v0_sram_heap_min_ever_free();
        stats->heap_end_free = heap_free;
        stats->cap_end_free = heap_free;
        if ((heap_free != 0U) &&
            (stats->cap_min_free != 0U) &&
            (heap_free < stats->cap_min_free))
        {
            stats->cap_min_free = heap_free;
        }
        if ((heap_min_free != 0U) &&
            ((stats->heap_min_free == 0U) || (heap_min_free < stats->heap_min_free)))
        {
            stats->heap_min_free = heap_min_free;
        }
    }

    if (stats->stack_api_ok)
    {
        stats->v0_stack_free_now = v0_sram_stack_free_now();
        if (stats->v0_stack_free_now < stats->v0_stack_free_min)
        {
            stats->v0_stack_free_min = stats->v0_stack_free_now;
        }
    }

    stats->cap_max_used = v0_sram_capture_heap_drop(stats);
    if ((stats->heap_total != 0U) &&
        (stats->heap_min_free != 0U) &&
        (stats->heap_min_free <= stats->heap_total))
    {
        stats->heap_max_used = stats->heap_total - stats->heap_min_free;
    }
    else
    {
        stats->heap_max_used = stats->cap_max_used;
    }
}

#if ZY100_LOG_FE_DIAG_VERBOSE
static uint32_t v0_sram_flash_queue_bytes(void)
{
#if V1_IMU_FLASH_CAPTURE_ENABLE
    return EXTERNAL_FLASH_CAPTURE_QUEUE_BLOCK_STATE_MAX * V1_DATA_BLOCK_BYTES;
#else
    return 0U;
#endif
}

static uint32_t v0_sram_scheduler_ctx_bytes(void)
{
    return (uint32_t)sizeof(zy100_spi_sched_input_t) +
           (uint32_t)sizeof(zy100_spi_sched_stats_t) +
           (uint32_t)sizeof(zy100_spi_sched_drain_context_t);
}

static bool v0_sram_workspace_fits(uint32_t cap_min_free, uint32_t workspace_bytes)
{
    uint64_t required;

    required = (uint64_t)workspace_bytes +
               (uint64_t)ZY100_OIS_RESERVED_SRAM_BYTES +
               (uint64_t)ZY100_SRAM_SAFETY_MARGIN_BYTES;
    return ((uint64_t)cap_min_free > required);
}

static void v0_sram_algo_budget(const zy100_sram_stats_t *stats,
                                uint32_t *algo_8k_safe,
                                uint32_t *algo_16k_safe,
                                uint32_t *suggested_workspace)
{
    if (algo_8k_safe != NULL)
    {
        *algo_8k_safe = 0U;
    }
    if (algo_16k_safe != NULL)
    {
        *algo_16k_safe = 0U;
    }
    if (suggested_workspace != NULL)
    {
        *suggested_workspace = 0U;
    }

    if ((stats == NULL) || !stats->heap_api_ok || (stats->cap_min_free == 0U))
    {
        return;
    }

    if ((algo_8k_safe != NULL) &&
        v0_sram_workspace_fits(stats->cap_min_free, ZY100_ALGO_WORKSPACE_TEST_BYTES))
    {
        *algo_8k_safe = 1U;
    }
    if ((algo_16k_safe != NULL) &&
        v0_sram_workspace_fits(stats->cap_min_free, ZY100_ALGO_WORKSPACE_STRESS_BYTES))
    {
        *algo_16k_safe = 1U;
    }
    if (suggested_workspace != NULL)
    {
        if ((algo_16k_safe != NULL) && (*algo_16k_safe != 0U))
        {
            *suggested_workspace = ZY100_ALGO_WORKSPACE_STRESS_BYTES;
        }
        else if ((algo_8k_safe != NULL) && (*algo_8k_safe != 0U))
        {
            *suggested_workspace = ZY100_ALGO_WORKSPACE_TEST_BYTES;
        }
    }
}
#endif

static void v0_sram_log_summary(const zy100_sram_stats_t *stats)
{
#if ZY100_LOG_FE_DIAG_VERBOSE
    uint32_t algo_8k_safe = 0U;
    uint32_t algo_16k_safe = 0U;
    uint32_t suggested_workspace = 0U;

    if (stats == NULL)
    {
        return;
    }

    v0_sram_algo_budget(stats,
                        &algo_8k_safe,
                        &algo_16k_safe,
                        &suggested_workspace);

    ZY100_DIAG_LOG("[SRAM_SUM_A] heap_api_ok=%u stack_api_ok=%u sram_total_known=%u total=%u heap_total=%u static_used=%u",
               stats->heap_api_ok ? 1U : 0U,
               stats->stack_api_ok ? 1U : 0U,
               stats->sram_total_known ? 1U : 0U,
               stats->sram_total,
               stats->heap_total,
               stats->static_used);
    ZY100_DIAG_LOG("[SRAM_SUM_B] heap_start=%u heap_end=%u heap_min_free=%u heap_max_used=%u",
               stats->heap_start_free,
               stats->heap_end_free,
               stats->heap_min_free,
               stats->heap_max_used);
    ZY100_DIAG_LOG("[SRAM_SUM_C] cap_start_free=%u cap_end_free=%u cap_min_free=%u cap_max_used=%u",
               stats->cap_start_free,
               stats->cap_end_free,
               stats->cap_min_free,
               stats->cap_max_used);
    ZY100_DIAG_LOG("[SRAM_SUM_D] v0_stack_free_start=%u v0_stack_free_now=%u v0_stack_free_min=%u",
               stats->v0_stack_free_start,
               stats->v0_stack_free_now,
               stats->v0_stack_free_min);
    DBG_DIRECT("[SRAM_COMPONENT_A] ingress_ring=%u v0_stack=%u fifo_read_buf=%u ingress_flush_buf=%u",
               ZY100_FIFO_INGRESS_RING_BYTES,
               V0_TASK_STACK_WORDS * (uint32_t)sizeof(StackType_t),
               (uint32_t)sizeof(s_v0_fifo_buf),
#if !V0_FLASH_RUNTIME_ENABLE
               0U);
#else
               (uint32_t)sizeof(s_v0_ingress_flush_buf));
#endif
    DBG_DIRECT("[SRAM_COMPONENT_B] marker_ctx=%u scheduler_ctx=%u flash_queue=%u algo_reserved=%u algo_used=%u",
               0U,
               v0_sram_scheduler_ctx_bytes(),
               v0_sram_flash_queue_bytes(),
               stats->algo_workspace_reserved,
               stats->algo_workspace_used);
    ZY100_DIAG_LOG("[SRAM_ALGO_BUDGET] algo_8k_safe=%u algo_16k_safe=%u suggested_workspace=%u reserve_for_ois=%u safety_margin=%u",
               algo_8k_safe,
               algo_16k_safe,
               suggested_workspace,
               ZY100_OIS_RESERVED_SRAM_BYTES,
               ZY100_SRAM_SAFETY_MARGIN_BYTES);
#else
    (void)stats;
#endif
}
#endif

static uint32_t v0_elapsed_ms(uint32_t start_ms)
{
    return zy100_os_time_ms() - start_ms;
}

#if !ZY100_FINAL_EDGE_MODE_ENABLE
static uint32_t v0_delta_u32(uint32_t now, uint32_t then)
{
    return now - then;
}
#endif

#if ZY100_RUNTIME_STATS_LOG_ENABLE
static uint32_t v0_rate_per_s(uint32_t delta, uint32_t elapsed_ms)
{
    uint64_t rate;

    if (elapsed_ms == 0U)
    {
        return 0U;
    }

    rate = ((uint64_t)delta * 1000ULL) / (uint64_t)elapsed_ms;
    return (rate > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)rate;
}
#endif

static void v0_record_loop_gap(v0_fifo_stats_t *stats,
                               uint32_t now_ms,
                               uint32_t *last_loop_ms)
{
    uint32_t gap_ms;

    if ((stats == NULL) || (last_loop_ms == NULL))
    {
        return;
    }

    if (*last_loop_ms == 0U)
    {
        *last_loop_ms = now_ms;
        return;
    }

    gap_ms = now_ms - *last_loop_ms;
    *last_loop_ms = now_ms;
    if (gap_ms > stats->max_loop_gap_ms)
    {
        stats->max_loop_gap_ms = gap_ms;
    }
    if (gap_ms > 10U)
    {
        stats->loop_gap_over_10ms_count++;
    }
    if (gap_ms > 20U)
    {
        stats->loop_gap_over_20ms_count++;
    }
}

#if ZY100_RUNTIME_STATS_LOG_ENABLE && V0_FLASH_RUNTIME_ENABLE
static int32_t v0_diff_u32_i32(uint32_t lhs, uint32_t rhs)
{
    uint32_t delta;

    if (lhs >= rhs)
    {
        delta = lhs - rhs;
        return (delta > 0x7FFFFFFFUL) ? 0x7FFFFFFF : (int32_t)delta;
    }

    delta = rhs - lhs;
    return (delta > 0x7FFFFFFFUL) ? (-2147483647 - 1) : -(int32_t)delta;
}
#endif

#if V0_FLASH_RUNTIME_ENABLE
static uint32_t v0_backlog_blocks_u32(uint32_t packet_count,
                                      uint32_t write_count)
{
    uint32_t produced_blocks = packet_count / V1_PACKETS_PER_BLOCK;

    return (produced_blocks >= write_count) ?
           (produced_blocks - write_count) : 0U;
}

static void v0_update_queue_max_level(v0_fifo_stats_t *stats,
                                      const external_flash_capture_queue_status_t *status)
{
    if ((stats == NULL) || (status == NULL))
    {
        return;
    }

    if (status->queue_level > stats->queue_max_level)
    {
        stats->queue_max_level = status->queue_level;
    }
}
#endif

#if !V0_FLASH_RUNTIME_ENABLE
static uint32_t v0_expected_packets(uint32_t duration_ms)
{
    return (uint32_t)(((uint64_t)duration_ms * V0_SAMPLE_RATE_HZ) / 1000ULL);
}
#endif

#if IMU_CAPTURE_PROGRESS_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE
static const char *v0_stop_reason_name(imu_fifo_drain_test_stop_reason_t reason)
{
    switch (reason)
    {
    case IMU_FIFO_DRAIN_TEST_STOP_REASON_KEY_SLEEP:
        return "key_sleep";
    case IMU_FIFO_DRAIN_TEST_STOP_REASON_FLASH_95_PERCENT:
        return "flash_95";
    case IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL:
        return "fatal";
    case IMU_FIFO_DRAIN_TEST_STOP_REASON_EDGE_FULL:
        return "edge_full";
    case IMU_FIFO_DRAIN_TEST_STOP_REASON_PAUSED:
        return "paused";
    case IMU_FIFO_DRAIN_TEST_STOP_REASON_USER_STOP:
        return "user_stop";
    case IMU_FIFO_DRAIN_TEST_STOP_REASON_BATTERY_LOW:
        return "battery_low";
    case IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE:
    default:
        return "none";
    }
}
#endif

static void v0_adc_safe_window_close(void);

#if ZY100_FINAL_EDGE_MODE_ENABLE
static imu_fifo_drain_test_stop_reason_t v0_final_edge_legacy_stop_reason(
    zy100_fe_stop_reason_t reason)
{
    switch (reason)
    {
    case ZY100_FE_STOP_REASON_USER_SHORT_PRESS:
        return IMU_FIFO_DRAIN_TEST_STOP_REASON_USER_STOP;
    case ZY100_FE_STOP_REASON_FLASH_RAW_95:
    case ZY100_FE_STOP_REASON_FLASH_SUMMARY_95:
    case ZY100_FE_STOP_REASON_FLASH_EVENT_95:
    case ZY100_FE_STOP_REASON_FLASH_META_95:
        return IMU_FIFO_DRAIN_TEST_STOP_REASON_FLASH_95_PERCENT;
    case ZY100_FE_STOP_REASON_ERROR:
        return IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL;
    case ZY100_FE_STOP_REASON_BATTERY_LOW:
        return IMU_FIFO_DRAIN_TEST_STOP_REASON_BATTERY_LOW;
    case ZY100_FE_STOP_REASON_NONE:
    default:
        return IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
    }
}

static void v0_final_edge_set_state(zy100_fe_capture_state_t state)
{
    s_fe_capture_state = state;
}

static void v0_final_edge_request_stop(zy100_fe_stop_reason_t reason,
                                       const char *src)
{
    imu_fifo_drain_test_stop_reason_t legacy_reason;
    uint32_t now_ms = zy100_os_time_ms();
    zy100_training_stop_reason_t training_reason =
        ZY100_TRAINING_STOP_UNKNOWN;

    if (reason == ZY100_FE_STOP_REASON_NONE)
    {
        return;
    }
    if (s_fe_stop_seen)
    {
        if (!s_v0_suppress_next_stop_request_log)
        {
            ZY100_DIAG_LOG("[FE_STOP_DUP] reason=%u state=%u",
                       (uint32_t)s_fe_stop_reason,
                       (uint32_t)s_fe_capture_state);
        }
        s_v0_suppress_next_stop_request_log = false;
        return;
    }

    legacy_reason = v0_final_edge_legacy_stop_reason(reason);
    s_fe_stop_seen = true;
    s_fe_stop_reason = reason;
    s_fe_stop_request_ms = now_ms;
    s_fe_export_ready = false;
    s_fe_export_cleared = false;
    s_fe_export_preserved = false;
    s_v0_stop_reason = legacy_reason;
    s_v0_stop_requested = true;
    s_v0_stop_in_progress = true;
    v0_adc_safe_window_close();
    v0_final_edge_set_state(FE_CAPTURE_STATE_STOP_REQUESTED);
    if ((reason == ZY100_FE_STOP_REASON_FLASH_RAW_95) ||
        (reason == ZY100_FE_STOP_REASON_FLASH_SUMMARY_95) ||
        (reason == ZY100_FE_STOP_REASON_FLASH_EVENT_95) ||
        (reason == ZY100_FE_STOP_REASON_FLASH_META_95))
    {
        training_reason = ZY100_TRAINING_STOP_FLASH_FULL;
    }
    else if (reason == ZY100_FE_STOP_REASON_ERROR)
    {
        training_reason = ZY100_TRAINING_STOP_ERROR;
    }
    else if (reason == ZY100_FE_STOP_REASON_USER_SHORT_PRESS)
    {
        training_reason = ZY100_TRAINING_STOP_BUTTON;
    }
    else if (reason == ZY100_FE_STOP_REASON_BATTERY_LOW)
    {
        training_reason = ZY100_TRAINING_STOP_BATTERY_LOW;
    }
#if ZY100_ONLINE_STREAM_ENABLE
    if (zy100_online_stream_active() &&
        !zy100_online_stream_end_requested())
    {
        training_reason = ZY100_TRAINING_STOP_UNKNOWN;
    }
#endif
    if (training_reason != ZY100_TRAINING_STOP_UNKNOWN)
    {
        zy100_training_session_note_end_if_unset(training_reason);
    }

    if (!s_v0_suppress_next_stop_request_log)
    {
        bool quiet_internal_online_stop =
            v0_online_quiet_logs() &&
            (reason == ZY100_FE_STOP_REASON_USER_SHORT_PRESS) &&
            !zy100_online_stream_end_requested();

        if (!quiet_internal_online_stop)
        {
            ZY100_DIAG_LOG("[FE_STOP_REQ] reason=%u src=%s state=%u ms=%u",
                       (uint32_t)reason,
                       (src != NULL) ? src : "unknown",
                       (uint32_t)s_fe_capture_state,
                       now_ms);
        }
    }
    s_v0_suppress_next_stop_request_log = false;
}

#if ZY100_ONLINE_STREAM_ENABLE
static bool v0_final_edge_online_stop_active(void)
{
    return zy100_online_stream_active() ||
           zy100_online_stream_end_wait_ack();
}

static void v0_final_edge_request_online_pause_stop(
    imu_fifo_drain_test_stop_reason_t reason)
{
    uint32_t now_ms = zy100_os_time_ms();
    bool battery_low =
        reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_BATTERY_LOW;
    imu_fifo_drain_test_stop_reason_t use_reason = battery_low ?
        IMU_FIFO_DRAIN_TEST_STOP_REASON_BATTERY_LOW :
        IMU_FIFO_DRAIN_TEST_STOP_REASON_PAUSED;
    zy100_fe_stop_reason_t fe_reason = battery_low ?
        ZY100_FE_STOP_REASON_BATTERY_LOW :
        ZY100_FE_STOP_REASON_USER_SHORT_PRESS;

    s_v0_online_pause_stop_requested = true;
    if (s_fe_stop_seen)
    {
        if (battery_low &&
            (s_fe_stop_reason == ZY100_FE_STOP_REASON_USER_SHORT_PRESS))
        {
            s_fe_stop_reason = ZY100_FE_STOP_REASON_BATTERY_LOW;
            s_v0_stop_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_BATTERY_LOW;
        }
        else if (!battery_low ||
                 (s_fe_stop_reason == ZY100_FE_STOP_REASON_BATTERY_LOW))
        {
            s_v0_stop_reason = use_reason;
        }
        v0_adc_safe_window_close();
        s_v0_stop_requested = true;
        if (s_v0_capture_active || s_v0_worker_busy)
        {
            s_v0_stop_in_progress = true;
        }
        return;
    }

    s_fe_stop_seen = true;
    s_fe_stop_reason = fe_reason;
    s_fe_stop_request_ms = now_ms;
    s_fe_export_ready = false;
    s_fe_export_cleared = false;
    s_fe_export_preserved = false;
    s_v0_stop_reason = use_reason;
    s_v0_stop_requested = true;
    s_v0_stop_in_progress = true;
    v0_adc_safe_window_close();
    v0_final_edge_set_state(FE_CAPTURE_STATE_STOP_REQUESTED);

    if (!v0_online_quiet_logs())
    {
        DBG_DIRECT("[FE_ONLINE_PAUSE_REQ] state=%u end=%u ms=%u",
                   (uint32_t)s_fe_capture_state,
                   zy100_online_stream_end_requested() ? 1U : 0U,
                   now_ms);
    }
}
#endif
#endif

static void v0_request_stop_from_worker(imu_fifo_drain_test_stop_reason_t reason)
{
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    if (zy100_online_raw_capture_active())
    {
        imu_fifo_drain_test_stop_reason_t abort_reason =
            (reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_BATTERY_LOW) ?
            IMU_FIFO_DRAIN_TEST_STOP_REASON_BATTERY_LOW :
            IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL;
        uint32_t now_ms = zy100_os_time_ms();

        zy100_online_stream_request_abort_ex(
            ZY100_ONLINE_STOP_REASON_INTERNAL,
            ZY100_ONLINE_ABORT_ORIGIN_IMU_WORKER_STOP,
            (uint32_t)reason,
            now_ms);
        (void)imu_fifo_drain_test_request_online_abort_with_reason(
            abort_reason);
        return;
    }
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE
    if (reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_BATTERY_LOW)
    {
        s_v0_suppress_next_stop_request_log = true;
        v0_final_edge_request_stop(ZY100_FE_STOP_REASON_BATTERY_LOW, "worker");
        return;
    }
    if ((reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL) ||
        (reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_KEY_SLEEP))
    {
        v0_final_edge_request_stop(ZY100_FE_STOP_REASON_ERROR, "worker");
        return;
    }
    if (reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_USER_STOP)
    {
        v0_final_edge_request_stop(ZY100_FE_STOP_REASON_USER_SHORT_PRESS,
                                   "button");
        return;
    }
    if (reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_FLASH_95_PERCENT)
    {
        v0_final_edge_request_stop(ZY100_FE_STOP_REASON_FLASH_RAW_95,
                                   "flash");
        return;
    }
#endif
    s_v0_stop_reason = reason;
    s_v0_stop_requested = true;
    s_v0_stop_in_progress = true;
}

static void v0_adc_safe_window_close(void)
{
    s_v0_adc_safe_window_open = false;
    s_v0_adc_safe_window_ms = 0U;
}

static bool v0_adc_safe_window_state_ok(void)
{
    if (!s_v0_capture_active ||
        s_v0_start_pending ||
        s_v0_stop_requested ||
        s_v0_stop_in_progress)
    {
        return false;
    }
#if ZY100_FINAL_EDGE_MODE_ENABLE
    if ((s_fe_capture_state != FE_CAPTURE_STATE_RUNNING) ||
        s_fe_final_drain_active ||
        s_final_edge_no_new_data_mode)
    {
        return false;
    }
#endif
    if (s_v0_high_water_alarm_pending || s_v0_phase_a_rescue_pending)
    {
        return false;
    }
    if (zy100_spi_sched_has_pending_work() ||
        zy100_spi_sched_has_urgent_work())
    {
        return false;
    }
    if (zy100_fifo_ingress_level_bytes() >= ZY100_FIFO_INGRESS_FLUSH_BYTES)
    {
        return false;
    }
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    if (zy100_final_edge_raw_store_has_pending_work())
    {
        return false;
    }
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE
    if (zy100_final_edge_record_store_has_pending_work())
    {
        return false;
    }
#endif
    return true;
}

static void v0_adc_safe_window_open(uint32_t now_ms)
{
    if (!v0_adc_safe_window_state_ok())
    {
        v0_adc_safe_window_close();
        return;
    }
    s_v0_adc_safe_window_ms = now_ms;
    s_v0_adc_safe_window_open = true;
}

static bool v0_fifo_fatal_condition_seen(const v0_fifo_stats_t *stats)
{
    if (stats == NULL)
    {
        return false;
    }
    if ((stats->fifo_full_count != 0U) ||
        (stats->fifo_lost_pkt_count != 0U) ||
        (stats->bad_header_count != 0U))
    {
        return true;
    }
    return zy100_fifo_ingress_overflow_count() != 0U;
}

static bool v0_handle_fifo_fatal_if_seen(v0_fifo_stats_t *stats)
{
    uint32_t cause_detail = 4U;

    if ((stats == NULL) || stats->fatal_error ||
        !v0_fifo_fatal_condition_seen(stats))
    {
        return false;
    }

    stats->fatal_error = true;
#if ZY100_LEGACY_OFFLINE_ENABLE
    if (s_v0_offline_fifo_diag.active)
    {
        v0_offline_fault_t fault = V0_OFFLINE_FAULT_INGRESS;
        if (stats->fifo_lost_pkt_count != 0U)
        {
            fault = V0_OFFLINE_FAULT_FIFO_LOST;
        }
        else if (stats->fifo_full_count != 0U)
        {
            fault = V0_OFFLINE_FAULT_FIFO_FULL;
        }
        else if (stats->bad_header_count != 0U)
        {
            fault = V0_OFFLINE_FAULT_BAD_HEADER;
        }
        v0_offline_fifo_latch_fault(fault,
                                    stats,
                                    stats->max_fifo_count,
                                    0U,
                                    0U,
                                    0U,
                                    V0_OFFLINE_FIFO_STAGE_NORMAL);
        stats->fatal_status = IMU_STATUS_NOT_READY;
    }
    else
#endif
    {
        stats->fatal_status = IMU_STATUS_BUS_ERROR;
    }
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    if (zy100_online_raw_capture_active())
    {
        v0_online_abort_diag_capture_end(stats);
        if (stats->fifo_lost_pkt_count != 0U)
        {
            cause_detail = 1U;
        }
        else if (stats->fifo_full_count != 0U)
        {
            cause_detail = 2U;
        }
        else if (stats->bad_header_count != 0U)
        {
            cause_detail = 3U;
        }
        zy100_online_stream_request_abort_ex(
            ZY100_ONLINE_STOP_REASON_INTERNAL,
            ZY100_ONLINE_ABORT_ORIGIN_IMU_FIFO_FATAL,
            cause_detail,
            zy100_os_time_ms());
    }
#else
    IMU_UNUSED(cause_detail);
#endif
    v0_adc_safe_window_close();
    s_v0_suppress_next_stop_request_log = true;
    v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL);
    return true;
}



#if ZY100_FINAL_EDGE_MODE_ENABLE
#if ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
#if (ZY100_FINAL_EDGE_BSCAN_OD_DIAG_ENABLE || \
     ZY100_FINAL_EDGE_BSCAN_TMST_SUMMARY_ENABLE)
static void v0_final_edge_od_diag_reset(void)
{
    memset(&s_final_edge_od_diag_stats, 0, sizeof(s_final_edge_od_diag_stats));
    s_final_edge_od_diag_stats.tmst_delta_min = 0xFFFFU;
    s_final_edge_od_diag_stats.tmst_delta_first_bad_seq =
        ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
    s_final_edge_od_diag_stats.tmst_delta_last_bad_seq =
        ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
    s_final_edge_od_diag_stats.read_us_max_seq =
        ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
}

static uint32_t v0_final_edge_od_diag_delta_min(void)
{
    return s_final_edge_od_diag_stats.tmst_delta_seen ?
           (uint32_t)s_final_edge_od_diag_stats.tmst_delta_min : 0U;
}

static uint32_t v0_final_edge_od_diag_delta_max(void)
{
    return s_final_edge_od_diag_stats.tmst_delta_seen ?
           (uint32_t)s_final_edge_od_diag_stats.tmst_delta_max : 0U;
}

static void v0_final_edge_note_od_diag_result(
    const zy100_final_edge_bscan_result_t *result,
    uint32_t bscan_id)
{
    const zy100_final_edge_bscan_diag_t *diag;

    if (result == NULL)
    {
        return;
    }

    diag = &result->diag;
    s_final_edge_od_diag_stats.od_early_events += diag->od_early_count;
    s_final_edge_od_diag_stats.od_mid_events += diag->od_mid_count;
    s_final_edge_od_diag_stats.od_tail_events += diag->od_tail_count;
    s_final_edge_od_diag_stats.od_post_events += diag->od_post_count;
    if (diag->od_max_pending > s_final_edge_od_diag_stats.od_max_pending)
    {
        s_final_edge_od_diag_stats.od_max_pending = diag->od_max_pending;
    }

    if (diag->tmst_delta_min != 0xFFFFU)
    {
        if (!s_final_edge_od_diag_stats.tmst_delta_seen)
        {
            s_final_edge_od_diag_stats.tmst_delta_min =
                diag->tmst_delta_min;
            s_final_edge_od_diag_stats.tmst_delta_max =
                diag->tmst_delta_max;
            s_final_edge_od_diag_stats.tmst_delta_seen = true;
        }
        else
        {
            if (diag->tmst_delta_min <
                s_final_edge_od_diag_stats.tmst_delta_min)
            {
                s_final_edge_od_diag_stats.tmst_delta_min =
                    diag->tmst_delta_min;
            }
            if (diag->tmst_delta_max >
                s_final_edge_od_diag_stats.tmst_delta_max)
            {
                s_final_edge_od_diag_stats.tmst_delta_max =
                    diag->tmst_delta_max;
            }
        }
    }
    s_final_edge_od_diag_stats.tmst_delta_zero_count +=
        diag->tmst_delta_zero_count;
    s_final_edge_od_diag_stats.tmst_delta_156_157_count +=
        diag->tmst_delta_156_157_count;
    s_final_edge_od_diag_stats.tmst_delta_double_count +=
        diag->tmst_delta_double_count;
    s_final_edge_od_diag_stats.tmst_delta_bad_count +=
        diag->tmst_delta_bad_count;
    s_final_edge_od_diag_stats.tmst_delta_other_bad_count +=
        diag->tmst_delta_other_bad_count;
    if (diag->tmst_delta_bad_count != 0U)
    {
        if (s_final_edge_od_diag_stats.tmst_delta_first_bad_scan_id == 0U)
        {
            s_final_edge_od_diag_stats.tmst_delta_first_bad_scan_id =
                bscan_id;
            s_final_edge_od_diag_stats.tmst_delta_first_bad_seq =
                diag->tmst_delta_first_bad_seq;
        }
        s_final_edge_od_diag_stats.tmst_delta_last_bad_scan_id =
            bscan_id;
        s_final_edge_od_diag_stats.tmst_delta_last_bad_seq =
            diag->tmst_delta_last_bad_seq;
    }

    if (diag->ois_read_us_max > s_final_edge_od_diag_stats.read_us_max)
    {
        s_final_edge_od_diag_stats.read_us_max = diag->ois_read_us_max;
        s_final_edge_od_diag_stats.read_us_max_scan_id = bscan_id;
        s_final_edge_od_diag_stats.read_us_max_seq =
            diag->ois_read_us_max_seq;
    }

    if (diag->od_total_count == 0U)
    {
        return;
    }

    s_final_edge_od_diag_stats.od_scan_count++;
    if (result->hit_found != 0U)
    {
        s_final_edge_od_diag_stats.od_hit_scan_count++;
    }
    else
    {
        s_final_edge_od_diag_stats.od_nohit_scan_count++;
    }

#if (ZY100_FINAL_EDGE_BSCAN_OD_DIAG_ENABLE && \
     (ZY100_FINAL_EDGE_CAPTURE_RUNTIME_LOG_QUIET_ENABLE == 0U))
    ZY100_DIAG_LOG("[FE_OD_DIAG] id=%u hit=%u frames=%u od=%u e=%u m=%u t=%u p=%u first=%u last=%u maxp=%u",
               bscan_id,
               result->hit_found,
               result->frames,
               diag->od_total_count,
               diag->od_early_count,
               diag->od_mid_count,
               diag->od_tail_count,
               diag->od_post_count,
               diag->od_first_seq,
               diag->od_last_seq,
               diag->od_max_pending);
    ZY100_DIAG_LOG("[FE_OD_DIAG2] id=%u read_us_max=%u seq=%u dt_bad=%u dt_first=%u dt_last=%u",
               bscan_id,
               diag->ois_read_us_max,
               diag->ois_read_us_max_seq,
               diag->dt_bad_count,
               diag->dt_bad_first_seq,
               diag->dt_bad_last_seq);
    ZY100_DIAG_LOG("[FE_OD_TMST] id=%u dmin=%u dmax=%u zero=%u dbl=%u bad=%u first=%u last=%u",
               bscan_id,
               (diag->tmst_delta_min == 0xFFFFU) ?
                   0U : (uint32_t)diag->tmst_delta_min,
               diag->tmst_delta_max,
               diag->tmst_delta_zero_count,
               diag->tmst_delta_double_count,
               diag->tmst_delta_bad_count,
               diag->tmst_delta_first_bad_seq,
               diag->tmst_delta_last_bad_seq);
#endif
}
#else
static void v0_final_edge_od_diag_reset(void)
{
    memset(&s_final_edge_od_diag_stats, 0, sizeof(s_final_edge_od_diag_stats));
}

static void v0_final_edge_note_od_diag_result(
    const zy100_final_edge_bscan_result_t *result,
    uint32_t bscan_id)
{
    IMU_UNUSED(result);
    IMU_UNUSED(bscan_id);
}
#endif

static void v0_final_edge_suspend_live_dt_baseline(v0_fifo_stats_t *stats)
{
    if (stats == NULL)
    {
        return;
    }

    stats->tmst_valid = false;
    stats->ts_warmup_remaining = V0_TS_WARMUP_PACKET_COUNT;
}

static void v0_final_edge_reset_b_last_result(void)
{
#if ZY100_FINAL_EDGE_HIT20_CALIB_ENABLE
    uint32_t idx;
#endif

    memset(&s_final_edge_b_last_result, 0, sizeof(s_final_edge_b_last_result));
    s_final_edge_b_last_result.diag.od_first_seq =
        ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
    s_final_edge_b_last_result.diag.od_last_seq =
        ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
    s_final_edge_b_last_result.diag.ois_read_us_max_seq =
        ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
    s_final_edge_b_last_result.diag.dt_bad_first_seq =
        ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
    s_final_edge_b_last_result.diag.dt_bad_last_seq =
        ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
    s_final_edge_b_last_result.diag.tmst_delta_min = 0xFFFFU;
    s_final_edge_b_last_result.diag.tmst_delta_first_bad_seq =
        ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
    s_final_edge_b_last_result.diag.tmst_delta_last_bad_seq =
        ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
    s_final_edge_b_last_result.exit_last_seq =
        ZY100_FINAL_EDGE_BSCAN_SEQ32_INVALID;
    s_final_edge_b_last_result.acq_tmst_mismatch_last_seq =
        ZY100_FINAL_EDGE_BSCAN_SEQ32_INVALID;
#if ZY100_FINAL_EDGE_HIT20_CALIB_ENABLE
    for (idx = 0U; idx < V0_FINAL_EDGE_HIT20_CALIB_COUNT; idx++)
    {
        s_final_edge_b_last_result.hit20_calib_first_cross_idx[idx] = -1;
        s_final_edge_b_last_result.hit20_calib_first_cross_score[idx] = 0U;
        s_final_edge_b_last_result.hit20_calib_first_cross_tmst[idx] = 0U;
    }
#endif
}

static void v0_final_edge_inc_stat(uint32_t *value)
{
    if ((value != NULL) && (*value < 0xFFFFFFFFU))
    {
        (*value)++;
    }
}

static void v0_final_edge_add_stat(uint32_t *value, uint32_t delta)
{
    if (value == NULL)
    {
        return;
    }
    if ((0xFFFFFFFFU - *value) < delta)
    {
        *value = 0xFFFFFFFFU;
    }
    else
    {
        *value += delta;
    }
}

static bool v0_final_edge_b_pending_critical(void)
{
#if ZY100_FINAL_EDGE_B_CRITICAL_GATE_ENABLE
    return s_final_edge_b_critical_state == V0_FE_B_CRITICAL_PENDING;
#else
    return false;
#endif
}

static bool v0_offline_v2_mode_active(void)
{
#if ZY100_LEGACY_OFFLINE_ENABLE
    return zy100_offline_v2_capture_active();
#else
    return false;
#endif
}

static uint32_t v0_fifo_poll_interval_ms(bool offline_v2_mode)
{
    return offline_v2_mode ? ZY100_OFFLINE_V2_FIFO_POLL_INTERVAL_MS :
           ZY100_FIFO_POLL_INTERVAL_MS;
}

static uint32_t v0_fifo_service_soft_ms(bool offline_v2_mode)
{
    return offline_v2_mode ? ZY100_OFFLINE_V2_FIFO_SERVICE_SOFT_MS :
           ZY100_FIFO_SERVICE_SOFT_DEADLINE_MS;
}

static uint32_t v0_fifo_service_hard_ms(bool offline_v2_mode)
{
    return offline_v2_mode ? ZY100_OFFLINE_V2_FIFO_SERVICE_HARD_MS :
           ZY100_FIFO_SERVICE_HARD_DEADLINE_MS;
}

static uint32_t v0_fifo_service_emergency_ms(bool offline_v2_mode)
{
    return offline_v2_mode ? ZY100_OFFLINE_V2_FIFO_SERVICE_HARD_MS :
           ZY100_FIFO_SERVICE_EMERGENCY_DEADLINE_MS;
}

#if !ZY100_FINAL_EDGE_MODE_ENABLE
static zy100_offline_trace_service_t v0_offline_service_state(
    bool due,
    bool overdue,
    bool emergency)
{
    if (emergency)
    {
        return ZY100_OFFLINE_TRACE_SERVICE_EMERGENCY;
    }
    if (overdue)
    {
        return ZY100_OFFLINE_TRACE_SERVICE_OVERDUE;
    }
    if (due)
    {
        return ZY100_OFFLINE_TRACE_SERVICE_DUE;
    }
    return ZY100_OFFLINE_TRACE_SERVICE_IDLE;
}

static void v0_offline_stall_diag(
    v0_fifo_stats_t *stats,
    const zy100_spi_sched_stats_t *sched,
    uint32_t now_ms,
    bool scheduler_urgent,
    bool service_due,
    bool service_overdue,
    bool service_emergency,
    uint32_t last_coop_yield_ms,
    uint32_t *last_progress_ms,
    uint32_t *last_packet_count,
    uint32_t *last_drain_count,
    uint8_t *stall_log_mask,
    uint8_t *trace_state) __attribute__((unused));
static void v0_offline_stall_diag(
    v0_fifo_stats_t *stats,
    const zy100_spi_sched_stats_t *sched,
    uint32_t now_ms,
    bool scheduler_urgent,
    bool service_due,
    bool service_overdue,
    bool service_emergency,
    uint32_t last_coop_yield_ms,
    uint32_t *last_progress_ms,
    uint32_t *last_packet_count,
    uint32_t *last_drain_count,
    uint8_t *stall_log_mask,
    uint8_t *trace_state)
{
    zy100_offline_trace_service_t service;
    zy100_offline_trace_imu_phase_t phase;
    uint32_t idle_ms;
    uint8_t packed_state;
    bool pressure;

    if ((stats == NULL) || (sched == NULL) || (last_progress_ms == NULL) ||
        (last_packet_count == NULL) || (last_drain_count == NULL) ||
        (stall_log_mask == NULL) || (trace_state == NULL))
    {
        return;
    }
    if ((stats->packet_count != *last_packet_count) ||
        (stats->fifo_drain_since_start != *last_drain_count))
    {
        *last_packet_count = stats->packet_count;
        *last_drain_count = stats->fifo_drain_since_start;
        *last_progress_ms = now_ms;
        *stall_log_mask = 0U;
    }
    idle_ms = now_ms - *last_progress_ms;
    service = v0_offline_service_state(service_due,
                                       service_overdue,
                                       service_emergency);
    pressure = scheduler_urgent || s_v0_high_water_alarm_pending ||
               s_v0_phase_a_rescue_pending ||
               (service != ZY100_OFFLINE_TRACE_SERVICE_IDLE);
    phase = (pressure &&
             (idle_ms >= ZY100_OFFLINE_V2_FIFO_SERVICE_SOFT_MS)) ?
            ZY100_OFFLINE_TRACE_IMU_STALL : ZY100_OFFLINE_TRACE_IMU_LIVE;
    if (phase == ZY100_OFFLINE_TRACE_IMU_LIVE)
    {
        service = ZY100_OFFLINE_TRACE_SERVICE_IDLE;
        scheduler_urgent = false;
    }
    packed_state = (uint8_t)((uint8_t)phase << 5);
    if (phase == ZY100_OFFLINE_TRACE_IMU_STALL)
    {
        packed_state |= (uint8_t)(((uint8_t)service << 3) |
                                  (scheduler_urgent ? 4U : 0U) |
                                  (s_v0_high_water_alarm_pending ? 2U : 0U) |
                                  (s_v0_phase_a_rescue_pending ? 1U : 0U));
    }
    if (packed_state != *trace_state)
    {
        *trace_state = packed_state;
        zy100_offline_reset_trace_set_imu_state(
            phase,
            scheduler_urgent,
            (phase == ZY100_OFFLINE_TRACE_IMU_STALL) &&
                s_v0_high_water_alarm_pending,
            (phase == ZY100_OFFLINE_TRACE_IMU_STALL) &&
                s_v0_phase_a_rescue_pending,
            service,
            sched->ms_since_last_fifo_drain);
    }
    if (!pressure)
    {
        return;
    }
    if ((idle_ms >= ZY100_OFFLINE_V2_FIFO_SERVICE_SOFT_MS) &&
        ((*stall_log_mask & 0x01U) == 0U))
    {
        *stall_log_mask |= 0x01U;
    }
    else if ((idle_ms >= ZY100_OFFLINE_V2_FIFO_SERVICE_HARD_MS) &&
             ((*stall_log_mask & 0x02U) == 0U))
    {
        *stall_log_mask |= 0x02U;
    }
    else if ((idle_ms >= ZY100_OFFLINE_V2_FIFO_SERVICE_HARD_MS) &&
             ((*stall_log_mask & 0x04U) == 0U))
    {
        *stall_log_mask |= 0x04U;
    }
    else
    {
        return;
    }
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][IMU_STALL_A] idle=%lu svc=%u urg=%u alarm=%u rescue=%u",
        (unsigned long)idle_ms,
        (uint32_t)service,
        scheduler_urgent ? 1U : 0U,
        s_v0_high_water_alarm_pending ? 1U : 0U,
        s_v0_phase_a_rescue_pending ? 1U : 0U);
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][IMU_STALL_B] pkt=%lu drain=%lu fifo=%lu age=%lu read=%lu",
        (unsigned long)stats->packet_count,
        (unsigned long)stats->fifo_drain_since_start,
        (unsigned long)sched->max_fifo_count,
        (unsigned long)sched->ms_since_last_fifo_drain,
        (unsigned long)stats->read_error_count);
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][IMU_STALL_C] latch=%lu confirm=%lu noop=%lu yield_age=%lu",
        (unsigned long)sched->fifo_drain_latched,
        (unsigned long)sched->confirmed_drain_pending,
        (unsigned long)sched->no_op_count,
        (unsigned long)(now_ms - last_coop_yield_ms));
}
#endif

static bool v0_final_edge_b_critical_offline_bypass_enabled(void)
{
#if ZY100_FINAL_EDGE_B_CRITICAL_GATE_ENABLE && \
    ZY100_FINAL_EDGE_OFFLINE_B_CRITICAL_BYPASS_VERIFY_ENABLE
#if ZY100_ONLINE_STREAM_ENABLE
    return !v0_final_edge_online_stop_active();
#else
    return true;
#endif
#else
    return false;
#endif
}

static void v0_final_edge_b_critical_note_offline_bypass(bool post_quiet)
{
#if ZY100_FINAL_EDGE_B_CRITICAL_GATE_ENABLE && \
    ZY100_FINAL_EDGE_OFFLINE_B_CRITICAL_BYPASS_VERIFY_ENABLE
    v0_final_edge_inc_stat(
        &s_final_edge_phase1_stats.b_critical_offline_bypass_count);
    if (post_quiet)
    {
        v0_final_edge_inc_stat(
            &s_final_edge_phase1_stats
                 .b_critical_offline_post_quiet_bypass_count);
    }
    if (!s_final_edge_b_critical_offline_bypass_logged)
    {
        s_final_edge_b_critical_offline_bypass_logged = true;
        ZY100_DIAG_LOG("[FE_BCRIT_AB] offline_bypass=1 state=%u post=%u",
                   (uint32_t)s_final_edge_b_critical_state,
                   post_quiet ? 1U : 0U);
    }
#else
    IMU_UNUSED(post_quiet);
#endif
}

#if (ZY100_FINAL_EDGE_POST_RESTORE_QUIET_MS != 0U)
static bool v0_final_edge_post_restore_quiet_active(void)
{
    const uint32_t deadline = s_final_edge_post_restore_quiet_until_ms;

    if (deadline == 0U)
    {
        return false;
    }
    return ((int32_t)(deadline - zy100_os_time_ms()) > 0);
}

static void v0_final_edge_post_restore_quiet_begin(void)
{
    s_final_edge_post_restore_quiet_until_ms =
        zy100_os_time_ms() + ZY100_FINAL_EDGE_POST_RESTORE_QUIET_MS;
}

static void v0_final_edge_post_restore_quiet_clear(void)
{
    s_final_edge_post_restore_quiet_until_ms = 0U;
}
#else
static bool v0_final_edge_post_restore_quiet_active(void)
{
    return false;
}

static void v0_final_edge_post_restore_quiet_begin(void)
{
}

static void v0_final_edge_post_restore_quiet_clear(void)
{
}
#endif

static bool v0_final_edge_b_critical_active(void)
{
#if ZY100_FINAL_EDGE_B_CRITICAL_GATE_ENABLE
    bool post_quiet = v0_final_edge_post_restore_quiet_active();
    bool active = (s_final_edge_b_critical_state == V0_FE_B_CRITICAL_PENDING) ||
                  (s_final_edge_b_critical_state == V0_FE_B_CRITICAL_ACTIVE) ||
                  post_quiet;

    if (active && v0_final_edge_b_critical_offline_bypass_enabled())
    {
        v0_final_edge_b_critical_note_offline_bypass(post_quiet);
        return false;
    }
    return active;
#else
    return false;
#endif
}

static void v0_final_edge_b_critical_set_pending(void)
{
#if ZY100_FINAL_EDGE_B_CRITICAL_GATE_ENABLE
    if (s_final_edge_b_critical_state == V0_FE_B_CRITICAL_IDLE)
    {
        s_final_edge_b_pending_start_us =
            (uint32_t)imu_bsp_local_timestamp_us();
        v0_final_edge_inc_stat(
            &s_final_edge_phase1_stats.b_critical_pending_count);
    }
    s_final_edge_b_critical_state = V0_FE_B_CRITICAL_PENDING;
#endif
}

static void v0_final_edge_b_critical_set_active(void)
{
#if ZY100_FINAL_EDGE_B_CRITICAL_GATE_ENABLE
    s_final_edge_b_critical_state = V0_FE_B_CRITICAL_ACTIVE;
    v0_final_edge_inc_stat(
        &s_final_edge_phase1_stats.b_critical_active_count);
#endif
}

static void v0_final_edge_b_critical_clear(void)
{
#if ZY100_FINAL_EDGE_B_CRITICAL_GATE_ENABLE
    if (s_final_edge_b_critical_state != V0_FE_B_CRITICAL_IDLE)
    {
        v0_final_edge_inc_stat(
            &s_final_edge_phase1_stats.b_critical_clear_count);
    }
    s_final_edge_b_critical_state = V0_FE_B_CRITICAL_IDLE;
    s_final_edge_b_pending_start_us = 0U;
#endif
}

static void v0_final_edge_b_critical_note_first_frame(
    const zy100_final_edge_bscan_result_t *result)
{
#if ZY100_FINAL_EDGE_B_CRITICAL_GATE_ENABLE
    uint32_t elapsed_us;

    if ((result == NULL) ||
        (result->first_frame_read_done_us == 0U) ||
        (s_final_edge_b_pending_start_us == 0U))
    {
        return;
    }

    elapsed_us = result->first_frame_read_done_us -
                 s_final_edge_b_pending_start_us;
    s_final_edge_phase1_stats.b_first_frame_read_done_last_us =
        result->first_frame_read_done_us;
    s_final_edge_phase1_stats.b_pending_to_first_frame_last_us =
        elapsed_us;
    if (elapsed_us >
        s_final_edge_phase1_stats.b_pending_to_first_frame_max_us)
    {
        s_final_edge_phase1_stats.b_pending_to_first_frame_max_us =
            elapsed_us;
    }
#else
    IMU_UNUSED(result);
#endif
}

#if ZY100_ONLINE_STREAM_ENABLE
static uint32_t v0_online_diag_delta16_us(uint16_t older, uint16_t newer)
{
    return (uint32_t)((uint16_t)(newer - older));
}

static void v0_online_diag_note_min_max(uint32_t value,
                                        uint32_t *min_value,
                                        uint32_t *max_value,
                                        uint32_t *count)
{
    if ((min_value == NULL) || (max_value == NULL) || (count == NULL))
    {
        return;
    }
    if (*count == 0U)
    {
        *min_value = value;
        *max_value = value;
    }
    else
    {
        if (value < *min_value)
        {
            *min_value = value;
        }
        if (value > *max_value)
        {
            *max_value = value;
        }
    }
    (*count)++;
}

static void v0_online_diag_note_live_sample(uint32_t seq, uint16_t tmst_raw)
{
    uint32_t gap_us;

    if (!s_v0_online_edge_diag.active)
    {
        return;
    }
    if (s_v0_online_edge_diag.awaiting_a2_first &&
        s_v0_online_edge_diag.current_b_last_valid)
    {
        gap_us = v0_online_diag_delta16_us(
                     s_v0_online_edge_diag.current_b_last_tmst_raw,
                     tmst_raw);
        s_v0_online_edge_diag.a2_valid_count++;
        s_v0_online_edge_diag.last_a2_seq = seq;
        s_v0_online_edge_diag.last_a2_tmst_raw = tmst_raw;
        s_v0_online_edge_diag.gap_b_a2_last_us = gap_us;
        v0_online_diag_note_min_max(
            gap_us,
            &s_v0_online_edge_diag.gap_b_a2_min_us,
            &s_v0_online_edge_diag.gap_b_a2_max_us,
            &s_v0_online_edge_diag.gap_b_a2_count);
        s_v0_online_edge_diag.awaiting_a2_first = false;
    }
    s_v0_online_edge_diag.live_tail_valid = true;
    s_v0_online_edge_diag.live_tail_seq = seq;
    s_v0_online_edge_diag.live_tail_tmst_raw = tmst_raw;
}

static void v0_online_diag_note_b_result(
    const zy100_final_edge_bscan_result_t *result,
    uint32_t bscan_id,
    uint32_t fail_reason)
{
    uint32_t gap_us;

    if (!s_v0_online_edge_diag.active || (result == NULL))
    {
        return;
    }
    if (s_v0_online_edge_diag.awaiting_a2_first)
    {
        s_v0_online_edge_diag.a2_missing_count++;
    }

    s_v0_online_edge_diag.b_count++;
    s_v0_online_edge_diag.last_b_id = bscan_id;
    if (fail_reason != 0U)
    {
        s_v0_online_edge_diag.b_fail_count++;
    }
    s_v0_online_edge_diag.b_frames_total += result->frames;
    if (result->frames > s_v0_online_edge_diag.b_frames_max)
    {
        s_v0_online_edge_diag.b_frames_max = result->frames;
    }
    s_v0_online_edge_diag.b_dt_bad_total += result->dt_bad;
    s_v0_online_edge_diag.b_stale_total += result->stale;
    s_v0_online_edge_diag.b_raw_mismatch_total +=
        result->acq_tmst_raw_mismatch;
    s_v0_online_edge_diag.b_delta_156_157_total +=
        result->diag.tmst_delta_156_157_count;
    s_v0_online_edge_diag.b_delta_bad_total +=
        result->diag.tmst_delta_bad_count;
    s_v0_online_edge_diag.b_delta_zero_total +=
        result->diag.tmst_delta_zero_count;
    s_v0_online_edge_diag.b_delta_double_total +=
        result->diag.tmst_delta_double_count;
    if (result->diag.tmst_delta_min != 0xFFFFU)
    {
        if (!s_v0_online_edge_diag.b_delta_seen)
        {
            s_v0_online_edge_diag.b_delta_min =
                result->diag.tmst_delta_min;
            s_v0_online_edge_diag.b_delta_max =
                result->diag.tmst_delta_max;
            s_v0_online_edge_diag.b_delta_seen = true;
        }
        else
        {
            if (result->diag.tmst_delta_min <
                s_v0_online_edge_diag.b_delta_min)
            {
                s_v0_online_edge_diag.b_delta_min =
                    result->diag.tmst_delta_min;
            }
            if (result->diag.tmst_delta_max >
                s_v0_online_edge_diag.b_delta_max)
            {
                s_v0_online_edge_diag.b_delta_max =
                    result->diag.tmst_delta_max;
            }
        }
    }

    s_v0_online_edge_diag.current_b_id = bscan_id;
    s_v0_online_edge_diag.current_b_a1_valid =
        s_v0_online_edge_diag.live_tail_valid;
    s_v0_online_edge_diag.current_a1_seq =
        s_v0_online_edge_diag.live_tail_seq;
    s_v0_online_edge_diag.current_a1_tmst_raw =
        s_v0_online_edge_diag.live_tail_tmst_raw;

    if (result->first_tmst_valid)
    {
        s_v0_online_edge_diag.last_b_first_tmst_raw =
            result->first_tmst_raw;
    }
    if (result->last_tmst_valid)
    {
        s_v0_online_edge_diag.current_b_last_valid = true;
        s_v0_online_edge_diag.current_b_last_tmst_raw =
            result->last_tmst_raw;
        s_v0_online_edge_diag.last_b_last_tmst_raw =
            result->last_tmst_raw;
    }
    else
    {
        s_v0_online_edge_diag.current_b_last_valid = false;
    }
    if (s_v0_online_edge_diag.current_b_a1_valid &&
        result->first_tmst_valid)
    {
        gap_us = v0_online_diag_delta16_us(
                     s_v0_online_edge_diag.current_a1_tmst_raw,
                     result->first_tmst_raw);
        s_v0_online_edge_diag.a1_valid_count++;
        s_v0_online_edge_diag.gap_a1_b_last_us = gap_us;
        v0_online_diag_note_min_max(
            gap_us,
            &s_v0_online_edge_diag.gap_a1_b_min_us,
            &s_v0_online_edge_diag.gap_a1_b_max_us,
            &s_v0_online_edge_diag.gap_a1_b_count);
    }
    s_v0_online_edge_diag.awaiting_a2_first =
        s_v0_online_edge_diag.current_b_last_valid;
}
#endif

static void v0_final_edge_flash_stats_reset(void)
{
    memset(&s_final_edge_flash_stats, 0, sizeof(s_final_edge_flash_stats));
}

static void v0_final_edge_wm_summary_reset(void)
{
    memset(&s_final_edge_wm_summary, 0, sizeof(s_final_edge_wm_summary));
}

static void v0_final_edge_wm_note_peak(uint32_t value, uint32_t *peak)
{
    if ((peak != NULL) && (value > *peak))
    {
        *peak = value;
    }
}

static void v0_final_edge_wm_note_check(uint32_t raw_used,
                                        uint32_t summary_used,
                                        uint32_t event_used,
                                        uint32_t meta_used)
{
    if (s_final_edge_wm_summary.check_count < 0xFFFFFFFFU)
    {
        s_final_edge_wm_summary.check_count++;
    }
    s_final_edge_wm_summary.last_raw_used = raw_used;
    s_final_edge_wm_summary.last_summary_used = summary_used;
    s_final_edge_wm_summary.last_event_used = event_used;
    s_final_edge_wm_summary.last_meta_used = meta_used;
    v0_final_edge_wm_note_peak(raw_used,
                               &s_final_edge_wm_summary.max_raw_used);
    v0_final_edge_wm_note_peak(summary_used,
                               &s_final_edge_wm_summary.max_summary_used);
    v0_final_edge_wm_note_peak(event_used,
                               &s_final_edge_wm_summary.max_event_used);
    v0_final_edge_wm_note_peak(meta_used,
                               &s_final_edge_wm_summary.max_meta_used);
}

static void v0_final_edge_wm_note_stop(uint32_t region,
                                       uint32_t used,
                                       uint32_t total,
                                       uint32_t pct)
{
    if (s_final_edge_wm_summary.stop_count < 0xFFFFFFFFU)
    {
        s_final_edge_wm_summary.stop_count++;
    }
    s_final_edge_wm_summary.stop_last_region = region;
    s_final_edge_wm_summary.stop_last_used = used;
    s_final_edge_wm_summary.stop_last_total = total;
    s_final_edge_wm_summary.stop_last_pct = pct;
}

static void v0_final_edge_log_wm_summary(void)
{
    ZY100_DIAG_LOG("[FE_WM_SUM] checks=%u raw=%u sum=%u evt=%u meta=%u pct=%u",
               s_final_edge_wm_summary.check_count,
               s_final_edge_wm_summary.last_raw_used,
               s_final_edge_wm_summary.last_summary_used,
               s_final_edge_wm_summary.last_event_used,
               s_final_edge_wm_summary.last_meta_used,
               (uint32_t)ZY100_FINAL_EDGE_FLASH_STOP_WATERMARK_PCT);
    ZY100_DIAG_LOG("[FE_WM_PEAK] raw=%u sum=%u evt=%u meta=%u stop=%u",
               s_final_edge_wm_summary.max_raw_used,
               s_final_edge_wm_summary.max_summary_used,
               s_final_edge_wm_summary.max_event_used,
               s_final_edge_wm_summary.max_meta_used,
               s_final_edge_wm_summary.stop_count);
    if (s_final_edge_wm_summary.stop_count != 0U)
    {
        ZY100_DIAG_LOG("[FE_WM_STOP] region=%u used=%u total=%u pct=%u",
                   s_final_edge_wm_summary.stop_last_region,
                   s_final_edge_wm_summary.stop_last_used,
                   s_final_edge_wm_summary.stop_last_total,
                   s_final_edge_wm_summary.stop_last_pct);
    }
}

static bool v0_final_edge_flash_stats_all_zero(void)
{
#if ZY100_FINAL_EDGE_PHASE7_EDGE_FLASH_ENABLE
    zy100_fe_raw_store_stats_t raw_stats;
    zy100_fe_record_store_stats_t record_stats;

    memset(&raw_stats, 0, sizeof(raw_stats));
    memset(&record_stats, 0, sizeof(record_stats));
    zy100_final_edge_raw_store_get_stats(&raw_stats);
    zy100_final_edge_record_store_get_stats(&record_stats);
    return (record_stats.write_error == 0U) &&
           (record_stats.verify_error == 0U) &&
           (record_stats.b_active_pump_blocked == 0U);
#else
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    zy100_fe_raw_store_stats_t raw_stats;

    memset(&raw_stats, 0, sizeof(raw_stats));
    zy100_final_edge_raw_store_get_stats(&raw_stats);
    return (s_final_edge_flash_stats.summary_flash == 0U) &&
           (s_final_edge_flash_stats.event_flash == 0U);
#else
    return (s_final_edge_flash_stats.page_program == 0U) &&
           (s_final_edge_flash_stats.erase == 0U) &&
           (s_final_edge_flash_stats.raw_saved == 0U) &&
           (s_final_edge_flash_stats.summary_flash == 0U) &&
           (s_final_edge_flash_stats.event_flash == 0U);
#endif
#endif
}

#if ZY100_FINAL_EDGE_HIT20_CALIB_ENABLE
static void v0_final_edge_hit20_calib_reset_stats(void)
{
    memset(&s_final_edge_hit20_calib_stats, 0,
           sizeof(s_final_edge_hit20_calib_stats));
    s_final_edge_hit20_calib_stats.peak20_min = UINT32_MAX;
}

static void v0_final_edge_hit20_calib_note_b_result(
    const zy100_final_edge_bscan_result_t *result)
{
    uint32_t idx;
    uint32_t peak20;

    if ((result == NULL) || (result->use20 == 0U) || (result->frames == 0U))
    {
        return;
    }

    peak20 = result->peak20_score;
    if (peak20 < s_final_edge_hit20_calib_stats.peak20_min)
    {
        s_final_edge_hit20_calib_stats.peak20_min = peak20;
    }
    if (peak20 > s_final_edge_hit20_calib_stats.peak20_max)
    {
        s_final_edge_hit20_calib_stats.peak20_max = peak20;
    }
    s_final_edge_hit20_calib_stats.peak20_sum += peak20;
    s_final_edge_hit20_calib_stats.peak20_count++;

    for (idx = 0U; idx < V0_FINAL_EDGE_HIT20_CALIB_COUNT; idx++)
    {
        if (peak20 >= s_final_edge_hit20_calib_thresholds[idx])
        {
            s_final_edge_hit20_calib_stats.peak20_over_count[idx]++;
        }
        if (result->hit20_calib_first_cross_idx[idx] >= 0)
        {
            s_final_edge_hit20_calib_stats.cross_count[idx]++;
        }
    }

    if (result->hit_found != 0U)
    {
        s_final_edge_hit20_calib_stats.hit_scan_count++;
    }
    else
    {
        s_final_edge_hit20_calib_stats.nohit_scan_count++;
    }
}

static uint32_t v0_final_edge_hit20_calib_print_min(void)
{
    return (s_final_edge_hit20_calib_stats.peak20_count == 0U) ?
           0U : s_final_edge_hit20_calib_stats.peak20_min;
}

static uint32_t v0_final_edge_hit20_calib_avg(void)
{
    if (s_final_edge_hit20_calib_stats.peak20_count == 0U)
    {
        return 0U;
    }

    return (uint32_t)(s_final_edge_hit20_calib_stats.peak20_sum /
                      s_final_edge_hit20_calib_stats.peak20_count);
}

static uint32_t v0_final_edge_hit20_calib_print_max(void)
{
    return (s_final_edge_hit20_calib_stats.peak20_count == 0U) ?
           0U : s_final_edge_hit20_calib_stats.peak20_max;
}
#endif

static void v0_final_edge_note_official_b_result(
    const zy100_final_edge_bscan_result_t *result)
{
    if ((result == NULL) || (result->use20 == 0U) || (result->frames == 0U))
    {
        return;
    }

    if (result->hit_found != 0U)
    {
        s_final_edge_official_hit_count++;
    }
    else
    {
        s_final_edge_official_nohit_count++;
    }
}

#if ZY100_FINAL_EDGE_PHASE4_HIT_WINDOW_ENABLE
static void v0_final_edge_phase4_reset(void)
{
    memset(&s_final_edge_phase4_stats, 0, sizeof(s_final_edge_phase4_stats));
    memset(&s_final_edge_last_hit_window_view, 0,
           sizeof(s_final_edge_last_hit_window_view));
    memset(&s_final_edge_last_lf_pre_query, 0,
           sizeof(s_final_edge_last_lf_pre_query));
    s_final_edge_rt_trigger_valid = false;
    s_final_edge_rt_trigger_sample_index = 0U;
    s_final_edge_rt_trigger_tmst_raw = 0U;
    zy100_final_edge_lf_pre_ring_reset();
}

static bool v0_final_edge_hwin_self_one(uint32_t hit_seq,
                                        uint32_t exp_start,
                                        uint32_t exp_end,
                                        uint32_t exp_hf,
                                        uint32_t exp_lf,
                                        uint32_t exp_pkt)
{
    zy100_final_edge_bscan_result_t result;
    zy100_final_edge_hit_window_view_t view;
    bool pass;

    memset(&result, 0, sizeof(result));
    result.hit_found = 1U;
    result.hit_index = hit_seq;
    result.hit_tmst_raw = 0U;
    result.target_end_seq = exp_end;
    result.last_saved_seq = exp_end;
    result.current_saved_seq = result.last_saved_seq;
    result.frames = exp_end + 1U;
    result.frames_seen = result.frames;
    result.ring_valid_frames =
        (result.frames > ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES) ?
        ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES : result.frames;

    (void)zy100_final_edge_bscan_extract_hit_window_view(
        s_final_edge_b_ois_ring,
        sizeof(s_final_edge_b_ois_ring),
        &result,
        0U,
        &view);

    pass = (view.valid != 0U) &&
           (view.start_seq == exp_start) &&
           (view.end_seq == exp_end) &&
           (view.hf_frames == exp_hf) &&
           (view.lf_pre_frames_needed == exp_lf) &&
           (view.lf_pre_packets_needed == exp_pkt);

    ZY100_DIAG_LOG("[FE_HWIN_SELF] hit=%u start=%u end=%u hf=%u lf=%u pkt=%u pass=%u",
               hit_seq,
               view.start_seq,
               view.end_seq,
               view.hf_frames,
               view.lf_pre_frames_needed,
               view.lf_pre_packets_needed,
               pass ? 1U : 0U);
    return pass;
}

static void v0_final_edge_run_hwin_self_test(void)
{
    bool p0;
    bool p10;
    bool p94;
    bool p95;
    bool p1279;

    p0 = v0_final_edge_hwin_self_one(0U, 0U, 224U, 225U, 95U, 12U);
    p10 = v0_final_edge_hwin_self_one(10U, 0U, 234U, 235U, 85U, 11U);
    p94 = v0_final_edge_hwin_self_one(94U, 0U, 318U, 319U, 1U, 1U);
    p95 = v0_final_edge_hwin_self_one(95U, 0U, 319U, 320U, 0U, 0U);
    p1279 = v0_final_edge_hwin_self_one(1279U, 1184U, 1503U, 320U, 0U, 0U);

    s_final_edge_phase4_stats.self_early =
        (p0 && p10 && p94 && p95) ? 1U : 0U;
    s_final_edge_phase4_stats.self_tail = p1279 ? 1U : 0U;
    s_final_edge_phase4_stats.self_pass =
        (s_final_edge_phase4_stats.self_early &&
         s_final_edge_phase4_stats.self_tail) ? 1U : 0U;
    ZY100_DIAG_LOG("[FE_HWIN_SELF] pass=%u early=%u tail=%u",
               (uint32_t)s_final_edge_phase4_stats.self_pass,
               (uint32_t)s_final_edge_phase4_stats.self_early,
               (uint32_t)s_final_edge_phase4_stats.self_tail);
}

static void v0_final_edge_note_hit_window_result(
    const zy100_final_edge_bscan_result_t *result)
{
    bool hwin_ok = false;
    bool ok = false;

    memset(&s_final_edge_last_hit_window_view, 0,
           sizeof(s_final_edge_last_hit_window_view));
    memset(&s_final_edge_last_lf_pre_query, 0,
           sizeof(s_final_edge_last_lf_pre_query));

    (void)zy100_final_edge_bscan_extract_hit_window_view(
        s_final_edge_b_ois_ring,
        sizeof(s_final_edge_b_ois_ring),
        result,
        s_final_edge_last_b_id,
        &s_final_edge_last_hit_window_view);

    if ((result == NULL) || (result->hit_found == 0U))
    {
        return;
    }

    s_final_edge_phase4_stats.hit_window_seen++;
    if (result->post_truncated != 0U)
    {
        s_final_edge_phase4_stats.post_trunc_count++;
    }
    hwin_ok = (s_final_edge_last_hit_window_view.valid != 0U) &&
              (s_final_edge_last_hit_window_view.complete != 0U) &&
              (result->post_truncated == 0U);
    if (hwin_ok)
    {
        s_final_edge_phase4_stats.b_hwin_ok++;
    }
    else
    {
        s_final_edge_phase4_stats.b_hwin_fail++;
    }

    if (s_final_edge_last_hit_window_view.valid == 0U)
    {
        s_final_edge_phase4_stats.hit_window_fail++;
        return;
    }

    if ((s_final_edge_last_hit_window_view.early_hit != 0U) &&
        (s_final_edge_last_hit_window_view.lf_pre_packets_needed != 0U))
    {
        s_final_edge_phase4_stats.early_hit_count++;
        if (s_final_edge_rt_trigger_valid)
        {
            (void)zy100_final_edge_lf_pre_ring_query_ending_at(
                s_final_edge_rt_trigger_sample_index,
                s_final_edge_last_hit_window_view.lf_pre_packets_needed,
                &s_final_edge_last_lf_pre_query);
        }
        else
        {
            (void)zy100_final_edge_lf_pre_ring_query_ending_at(
                0U,
                s_final_edge_last_hit_window_view.lf_pre_packets_needed,
                &s_final_edge_last_lf_pre_query);
        }
        s_final_edge_last_hit_window_view.lf_pre_packets_available =
            s_final_edge_last_lf_pre_query.packets_available;
        s_final_edge_last_hit_window_view.lf_pre_valid =
            s_final_edge_last_lf_pre_query.valid;
        if ((s_final_edge_last_lf_pre_query.valid != 0U) &&
            (s_final_edge_last_lf_pre_query.packets_available >=
             s_final_edge_last_hit_window_view.lf_pre_packets_needed) &&
            s_final_edge_rt_trigger_valid &&
            (s_final_edge_last_lf_pre_query.newest_tmst_raw ==
             s_final_edge_rt_trigger_tmst_raw))
        {
            s_final_edge_phase4_stats.early_lf_ok++;
        }
        else
        {
            s_final_edge_phase4_stats.early_lf_fail++;
        }
    }

    if (s_final_edge_last_hit_window_view.early_hit == 0U)
    {
        ok = (s_final_edge_last_hit_window_view.complete != 0U) &&
             (s_final_edge_last_hit_window_view.hf_frames ==
              ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES) &&
             (s_final_edge_last_hit_window_view.lf_pre_frames_needed == 0U) &&
             (result->post_truncated == 0U);
    }
    else
    {
        ok = (s_final_edge_last_hit_window_view.complete != 0U) &&
             (s_final_edge_last_hit_window_view.hf_frames <
              ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES) &&
             (s_final_edge_last_hit_window_view.lf_pre_frames_needed != 0U) &&
             (s_final_edge_last_hit_window_view.lf_pre_valid != 0U) &&
             (result->post_truncated == 0U);
    }

    if (ok)
    {
        s_final_edge_phase4_stats.hit_window_ok++;
    }
    else
    {
        s_final_edge_phase4_stats.hit_window_fail++;
    }
    s_final_edge_phase4_stats.window_xor ^=
        s_final_edge_last_hit_window_view.window_xor;
}

static bool v0_final_edge_phase4_b_scan_hard_ok(
    const zy100_final_edge_bscan_result_t *result)
{
    return (result != NULL) &&
           (result->err == 0U) &&
           (result->read_err == 0U) &&
           (result->over == 0U) &&
           (result->drop == 0U) &&
#if ZY100_FINAL_EDGE_B_HARD_RT_ACQ_ENABLE
           (result->stale == 0U) &&
           (result->dt_bad == 0U) &&
           (result->acq_overflow == 0U) &&
           (result->acq_read_err == 0U) &&
#if (!ZY100_FINAL_EDGE_B_ACQ_TMST_GATE_ENABLE && \
     !ZY100_FINAL_EDGE_B_ACQ_RAW_PHASE_LOCK_ENABLE)
           (result->acq_irq_late_count == 0U) &&
#endif
           (result->acq_tmst_raw_mismatch == 0U) &&
#if ZY100_FINAL_EDGE_B_ACQ_REUSE_HIT_RING_ENABLE
           (result->acq_high_water <
            ZY100_FINAL_EDGE_B_ACQ_BACKLOG_HARD_FRAMES) &&
#endif
#endif
           (result->owner_ok != 0U) &&
           (result->cleanup_ok != 0U);
}

static bool v0_final_edge_phase4_nohit_overdrop_warning_only(
    const zy100_final_edge_bscan_result_t *result,
    uint32_t preliminary_reason)
{
#if ZY100_FINAL_EDGE_B_HARD_RT_ACQ_ENABLE
    IMU_UNUSED(result);
    IMU_UNUSED(preliminary_reason);
    return false;
#else
    const uint32_t od_mask =
        FE_BFAIL_REASON_OVER |
        FE_BFAIL_REASON_DROP;

    return (result != NULL) &&
           (result->hit_found == 0U) &&
           (result->frames == ZY100_FINAL_EDGE_B_SCAN_BASE_FRAMES) &&
           (result->read_err == 0U) &&
           (result->owner_ok != 0U) &&
           (result->cleanup_ok != 0U) &&
           ((preliminary_reason & od_mask) != 0U) &&
           ((preliminary_reason & ~od_mask) == 0U);
#endif
}

static bool v0_final_edge_phase4_truehit_overdrop_warning_only(
    const zy100_final_edge_bscan_result_t *result,
    uint32_t preliminary_reason)
{
#if (ZY100_FINAL_EDGE_TRUEHIT_OD_WARN_ALLOW == 0U)
    IMU_UNUSED(result);
    IMU_UNUSED(preliminary_reason);
    return false;
#elif ZY100_FINAL_EDGE_B_HARD_RT_ACQ_ENABLE
    IMU_UNUSED(result);
    IMU_UNUSED(preliminary_reason);
    return false;
#else
    const uint32_t od_mask =
        FE_BFAIL_REASON_OVER |
        FE_BFAIL_REASON_DROP;

    return (result != NULL) &&
           (result->hit_found != 0U) &&
           (result->read_err == 0U) &&
           (result->owner_ok != 0U) &&
           (result->cleanup_ok != 0U) &&
           (result->post_truncated == 0U) &&
           (s_final_edge_last_hit_window_view.bscan_id ==
            s_final_edge_last_b_id) &&
           (s_final_edge_last_hit_window_view.valid != 0U) &&
           (s_final_edge_last_hit_window_view.complete != 0U) &&
           ((preliminary_reason & od_mask) != 0U) &&
           ((preliminary_reason & ~od_mask) == 0U);
#endif
}

static void v0_final_edge_note_nohit_od_warning(
    uint32_t reason,
    bool run_ok,
    const zy100_final_edge_bscan_result_t *result)
{
    if (result == NULL)
    {
        return;
    }

    s_final_edge_nohit_od_warn_count++;
    s_final_edge_nohit_od_warn_or |= reason;
    s_final_edge_nohit_od_warn_last_id = s_final_edge_last_b_id;
    s_final_edge_nohit_od_warn_last_reason = reason;
    s_final_edge_nohit_od_warn_last_run = run_ok ? 1U : 0U;
    s_final_edge_nohit_od_warn_last_frames = result->frames;
    s_final_edge_nohit_od_warn_last_err = result->err;
    s_final_edge_nohit_od_warn_last_over = result->over;
    s_final_edge_nohit_od_warn_last_drop = result->drop;
}

static void v0_final_edge_note_truehit_od_warning(
    uint32_t reason,
    bool run_ok,
    const zy100_final_edge_bscan_result_t *result)
{
    if (result == NULL)
    {
        return;
    }

    s_final_edge_truehit_od_warn_count++;
    s_final_edge_truehit_od_warn_or |= reason;
    s_final_edge_truehit_od_warn_last_id = s_final_edge_last_b_id;
    s_final_edge_truehit_od_warn_last_reason = reason;
    s_final_edge_truehit_od_warn_last_run = run_ok ? 1U : 0U;
    s_final_edge_truehit_od_warn_last_frames = result->frames;
    s_final_edge_truehit_od_warn_last_over = result->over;
    s_final_edge_truehit_od_warn_last_drop = result->drop;
}

static bool v0_final_edge_phase4_latest_hwin_ok(
    const zy100_final_edge_bscan_result_t *result)
{
    return (result != NULL) &&
           (result->hit_found != 0U) &&
           (result->post_truncated == 0U) &&
           (s_final_edge_last_hit_window_view.bscan_id ==
            s_final_edge_last_b_id) &&
           (s_final_edge_last_hit_window_view.valid != 0U) &&
           (s_final_edge_last_hit_window_view.complete != 0U);
}

static uint32_t v0_final_edge_phase4_b_scan_fail_reason(
    bool run_ok,
    const zy100_final_edge_bscan_result_t *result)
{
    const uint32_t explicit_err_mask =
        FE_BFAIL_REASON_READ_ERR |
        FE_BFAIL_REASON_OVER |
        FE_BFAIL_REASON_DROP |
#if ZY100_FINAL_EDGE_B_HARD_RT_ACQ_ENABLE
        FE_BFAIL_REASON_STALE |
        FE_BFAIL_REASON_DT_BAD |
        FE_BFAIL_REASON_ACQ_OVERFLOW |
        FE_BFAIL_REASON_ACQ_READ_ERR |
#if (!ZY100_FINAL_EDGE_B_ACQ_TMST_GATE_ENABLE && \
     !ZY100_FINAL_EDGE_B_ACQ_RAW_PHASE_LOCK_ENABLE)
        FE_BFAIL_REASON_ACQ_IRQ_LATE |
#endif
        FE_BFAIL_REASON_ACQ_TMST_MISMATCH |
#if ZY100_FINAL_EDGE_B_ACQ_REUSE_HIT_RING_ENABLE
        FE_BFAIL_REASON_ACQ_BACKLOG |
#endif
#endif
        FE_BFAIL_REASON_NOHIT_FRAME |
        FE_BFAIL_REASON_POST_TRUNC |
        FE_BFAIL_REASON_OWNER |
        FE_BFAIL_REASON_CLEANUP |
        FE_BFAIL_REASON_BEGIN_FAIL;
    uint32_t reason = 0U;

    if (result == NULL)
    {
        return FE_BFAIL_REASON_NULL_RESULT;
    }

    if (result->exit_reason == ZY100_FINAL_EDGE_BSCAN_EXIT_BEGIN_FAIL)
    {
        reason |= FE_BFAIL_REASON_BEGIN_FAIL;
    }
    if (result->owner_ok == 0U)
    {
        reason |= FE_BFAIL_REASON_OWNER;
    }
    if (result->cleanup_ok == 0U)
    {
        reason |= FE_BFAIL_REASON_CLEANUP;
    }
    if (result->read_err != 0U)
    {
        reason |= FE_BFAIL_REASON_READ_ERR;
    }
    if (result->over != 0U)
    {
        reason |= FE_BFAIL_REASON_OVER;
    }
    if (result->drop != 0U)
    {
        reason |= FE_BFAIL_REASON_DROP;
    }
#if ZY100_FINAL_EDGE_B_HARD_RT_ACQ_ENABLE
    if (result->stale != 0U)
    {
        reason |= FE_BFAIL_REASON_STALE;
    }
    if (result->dt_bad != 0U)
    {
        reason |= FE_BFAIL_REASON_DT_BAD;
    }
    if (result->acq_overflow != 0U)
    {
        reason |= FE_BFAIL_REASON_ACQ_OVERFLOW;
    }
    if (result->acq_read_err != 0U)
    {
        reason |= FE_BFAIL_REASON_ACQ_READ_ERR;
    }
#if (!ZY100_FINAL_EDGE_B_ACQ_TMST_GATE_ENABLE && \
     !ZY100_FINAL_EDGE_B_ACQ_RAW_PHASE_LOCK_ENABLE)
    if (result->acq_irq_late_count != 0U)
    {
        reason |= FE_BFAIL_REASON_ACQ_IRQ_LATE;
    }
#endif
    if (result->acq_tmst_raw_mismatch != 0U)
    {
        reason |= FE_BFAIL_REASON_ACQ_TMST_MISMATCH;
    }
#if ZY100_FINAL_EDGE_B_ACQ_REUSE_HIT_RING_ENABLE
    if (result->acq_high_water >=
        ZY100_FINAL_EDGE_B_ACQ_BACKLOG_HARD_FRAMES)
    {
        reason |= FE_BFAIL_REASON_ACQ_BACKLOG;
    }
#endif
#endif

    if (result->hit_found != 0U)
    {
        if (result->post_truncated != 0U)
        {
            reason |= FE_BFAIL_REASON_POST_TRUNC;
        }
        if (s_final_edge_last_hit_window_view.bscan_id !=
            s_final_edge_last_b_id)
        {
            reason |= FE_BFAIL_REASON_HWIN_INVALID;
        }
        else
        {
            if (s_final_edge_last_hit_window_view.valid == 0U)
            {
                reason |= FE_BFAIL_REASON_HWIN_INVALID;
            }
            if (s_final_edge_last_hit_window_view.complete == 0U)
            {
                reason |= FE_BFAIL_REASON_HWIN_INCOMP;
            }
        }
    }
    else if ((result->exit_reason != ZY100_FINAL_EDGE_BSCAN_EXIT_BEGIN_FAIL) &&
             (result->frames != ZY100_FINAL_EDGE_B_SCAN_BASE_FRAMES))
    {
        reason |= FE_BFAIL_REASON_NOHIT_FRAME;
    }

    if (v0_final_edge_phase4_nohit_overdrop_warning_only(result, reason))
    {
        v0_final_edge_note_nohit_od_warning(reason, run_ok, result);
        return 0U;
    }
    if (v0_final_edge_phase4_truehit_overdrop_warning_only(result, reason))
    {
        v0_final_edge_note_truehit_od_warning(reason, run_ok, result);
        return 0U;
    }

    if ((result->err != 0U) && ((reason & explicit_err_mask) == 0U))
    {
        reason |= FE_BFAIL_REASON_ERR;
    }
    if ((!run_ok) && (reason == 0U))
    {
        reason |= FE_BFAIL_REASON_RUN_FAIL;
    }

    return reason;
}

static uint8_t v0_final_edge_phase4_b_hwin_pass3b_ok(void)
{
    if (s_final_edge_official_hit_count == 0U)
    {
        return 1U;
    }

    return ((s_final_edge_phase4_stats.b_hwin_ok ==
             s_final_edge_official_hit_count) &&
            (s_final_edge_phase4_stats.b_hwin_fail == 0U)) ? 1U : 0U;
}

static uint8_t v0_final_edge_phase4_nohit1280_pass3b_ok(void)
{
    if (s_final_edge_b_scan_count == 0U)
    {
        return 0U;
    }
    if (s_final_edge_b_last_result.hit_found != 0U)
    {
        return 1U;
    }

    return (s_final_edge_b_last_result.frames ==
            ZY100_FINAL_EDGE_B_SCAN_BASE_FRAMES) ? 1U : 0U;
}

static uint8_t v0_final_edge_phase4_truehit_short_pass3b(void)
{
    if ((s_final_edge_b_scan_count == 0U) ||
        (s_final_edge_b_last_result.hit_found == 0U) ||
        (s_final_edge_b_last_result.frames >=
         ZY100_FINAL_EDGE_B_SCAN_BASE_FRAMES))
    {
        return 0U;
    }

    return (v0_final_edge_phase4_b_scan_hard_ok(
                &s_final_edge_b_last_result) &&
            v0_final_edge_phase4_latest_hwin_ok(
                &s_final_edge_b_last_result)) ? 1U : 0U;
}
#endif

static uint32_t v0_final_edge_b_scan_fail_reason(bool run_ok)
{
#if ZY100_FINAL_EDGE_PHASE4_HIT_WINDOW_ENABLE
    return v0_final_edge_phase4_b_scan_fail_reason(
        run_ok, &s_final_edge_b_last_result);
#else
    if (!run_ok || (s_final_edge_b_last_result.err != 0U))
    {
        return FE_BFAIL_REASON_ERR;
    }
    return 0U;
#endif
}

static void v0_final_edge_note_nohit_frame_fail(
    const zy100_final_edge_bscan_result_t *result)
{
    uint32_t frames;
    uint32_t short_frames;

    if (result == NULL)
    {
        return;
    }

    frames = result->frames;
    s_final_edge_b_nohit_frame_last = frames;
    s_final_edge_b_nohit_frame_last_exit = result->exit_reason;
    if ((s_final_edge_b_nohit_frame_min == 0U) ||
        (frames < s_final_edge_b_nohit_frame_min))
    {
        s_final_edge_b_nohit_frame_min = frames;
    }
    if (frames > s_final_edge_b_nohit_frame_max)
    {
        s_final_edge_b_nohit_frame_max = frames;
    }
    short_frames =
        (frames < ZY100_FINAL_EDGE_B_SCAN_BASE_FRAMES) ?
        (ZY100_FINAL_EDGE_B_SCAN_BASE_FRAMES - frames) : 0U;
    v0_final_edge_add_stat(&s_final_edge_b_nohit_frame_sum_short,
                           short_frames);
}

static void v0_final_edge_note_b_mismatch_result(
    const zy100_final_edge_bscan_result_t *result,
    uint32_t bscan_id)
{
    uint32_t count;
    uint32_t soft_drop;

    if (result == NULL)
    {
        return;
    }
    soft_drop = result->acq_tmst_startup_mismatch_drop;
    if (soft_drop != 0U)
    {
        v0_final_edge_add_stat(
            &s_final_edge_b_mismatch_soft_drop_count,
            soft_drop);
        s_final_edge_b_mismatch_soft_last_id = bscan_id;
        s_final_edge_b_mismatch_soft_last_probe =
            result->acq_tmst_startup_mismatch_last_probe_tmst;
        s_final_edge_b_mismatch_soft_last_raw =
            result->acq_tmst_startup_mismatch_last_raw_tmst;
        s_final_edge_b_mismatch_soft_limit =
            result->acq_tmst_startup_mismatch_hard_limit;
    }

    count = result->acq_tmst_raw_mismatch;
    if (count == 0U)
    {
        return;
    }

    v0_final_edge_add_stat(&s_final_edge_b_mismatch_count, count);
    if (result->hit_found != 0U)
    {
        v0_final_edge_add_stat(&s_final_edge_b_mismatch_hit_count, count);
    }
    else
    {
        v0_final_edge_add_stat(&s_final_edge_b_mismatch_nohit_count, count);
    }
    s_final_edge_b_mismatch_last_id = bscan_id;
    s_final_edge_b_mismatch_last_seq =
        result->acq_tmst_mismatch_last_seq;
    s_final_edge_b_mismatch_last_probe =
        result->acq_tmst_mismatch_probe_tmst;
    s_final_edge_b_mismatch_last_raw =
        result->acq_tmst_mismatch_raw_tmst;
}

static void v0_final_edge_note_b_scan_fail_reason(uint32_t reason,
                                                  bool run_ok)
{
    const uint32_t hard_mask =
        FE_BFAIL_REASON_READ_ERR |
        FE_BFAIL_REASON_OVER |
        FE_BFAIL_REASON_DROP |
#if ZY100_FINAL_EDGE_B_HARD_RT_ACQ_ENABLE
        FE_BFAIL_REASON_STALE |
        FE_BFAIL_REASON_DT_BAD |
        FE_BFAIL_REASON_ACQ_OVERFLOW |
        FE_BFAIL_REASON_ACQ_READ_ERR |
#if (!ZY100_FINAL_EDGE_B_ACQ_TMST_GATE_ENABLE && \
     !ZY100_FINAL_EDGE_B_ACQ_RAW_PHASE_LOCK_ENABLE)
        FE_BFAIL_REASON_ACQ_IRQ_LATE |
#endif
        FE_BFAIL_REASON_ACQ_TMST_MISMATCH |
#if ZY100_FINAL_EDGE_B_ACQ_REUSE_HIT_RING_ENABLE
        FE_BFAIL_REASON_ACQ_BACKLOG |
#endif
#endif
        FE_BFAIL_REASON_ERR |
        FE_BFAIL_REASON_OWNER |
        FE_BFAIL_REASON_CLEANUP |
        FE_BFAIL_REASON_RUN_FAIL |
        FE_BFAIL_REASON_BEGIN_FAIL;

    if (reason == 0U)
    {
        return;
    }

    s_final_edge_b_scan_fail_count++;
    s_final_edge_b_fail_reason_or |= reason;
    s_final_edge_b_fail_last_reason = reason;
    s_final_edge_b_fail_last_id = s_final_edge_last_b_id;
    s_final_edge_b_fail_last_frames = s_final_edge_b_last_result.frames;
    s_final_edge_b_fail_last_hit = s_final_edge_b_last_result.hit_found;
    s_final_edge_b_fail_last_exit_reason =
        s_final_edge_b_last_result.exit_reason;
    s_final_edge_b_fail_last_seq = s_final_edge_b_last_result.exit_last_seq;
    s_final_edge_b_fail_last_elapsed_ms =
        s_final_edge_b_last_result.exit_elapsed_ms;
    if ((reason & hard_mask) != 0U)
    {
        s_final_edge_b_fail_hard_count++;
    }
    if ((reason & FE_BFAIL_REASON_BEGIN_FAIL) != 0U)
    {
        s_final_edge_b_begin_fail_count++;
        s_final_edge_b_begin_fail_last_stage =
            s_final_edge_b_last_result.acq_begin_fail_stage;
        s_final_edge_b_begin_fail_last_status =
            s_final_edge_b_last_result.acq_begin_fail_status;
        s_final_edge_b_begin_fail_last_retry =
            s_final_edge_b_last_result.begin_retry;
        s_final_edge_b_begin_fail_last_owner =
            s_final_edge_b_last_result.owner_ok;
        s_final_edge_b_begin_fail_last_cleanup =
            s_final_edge_b_last_result.cleanup_ok;
    }
    if ((reason & FE_BFAIL_REASON_NOHIT_FRAME) != 0U)
    {
        s_final_edge_b_fail_frame_count++;
        v0_final_edge_note_nohit_frame_fail(&s_final_edge_b_last_result);
    }
    if ((reason & (FE_BFAIL_REASON_HWIN_INVALID |
                   FE_BFAIL_REASON_HWIN_INCOMP)) != 0U)
    {
        s_final_edge_b_fail_hwin_count++;
    }
    if ((reason & FE_BFAIL_REASON_POST_TRUNC) != 0U)
    {
        s_final_edge_b_fail_trunc_count++;
    }

#if (ZY100_FINAL_EDGE_PHASE4_HIT_WINDOW_ENABLE && \
     (ZY100_FINAL_EDGE_CAPTURE_RUNTIME_LOG_QUIET_ENABLE == 0U))
    DBG_DIRECT("[FE_BFAIL] id=%u r=%08x run=%u hit=%u frames=%u",
               s_final_edge_last_b_id,
               reason,
               run_ok ? 1U : 0U,
               s_final_edge_b_last_result.hit_found,
               s_final_edge_b_last_result.frames);
    DBG_DIRECT("[FE_BFAIL2] err=%u read=%u over=%u drop=%u trunc=%u",
               s_final_edge_b_last_result.err,
               s_final_edge_b_last_result.read_err,
               s_final_edge_b_last_result.over,
               s_final_edge_b_last_result.drop,
               s_final_edge_b_last_result.post_truncated);
    DBG_DIRECT("[FE_BFAIL4] stale=%u dt_bad=%u acq_ov=%u acq_read=%u acq_late=%u",
               s_final_edge_b_last_result.stale,
               s_final_edge_b_last_result.dt_bad,
               s_final_edge_b_last_result.acq_overflow,
               s_final_edge_b_last_result.acq_read_err,
               s_final_edge_b_last_result.acq_irq_late_count);
    DBG_DIRECT("[FE_BFAIL5] gap_min=%u gap_max=%u early=%u late_gap=%u read_us=%u",
               s_final_edge_b_last_result.acq_irq_gap_us_min,
               s_final_edge_b_last_result.acq_irq_gap_us_max,
               s_final_edge_b_last_result.acq_irq_early_count,
               s_final_edge_b_last_result.acq_irq_late_gap_count,
               s_final_edge_b_last_result.acq_read_us_max);
    DBG_DIRECT("[FE_BFAIL6] high=%u irq=%u wr=%u rd=%u skip_stop=%u",
               s_final_edge_b_last_result.acq_high_water,
               s_final_edge_b_last_result.timer_irq_total,
               s_final_edge_b_last_result.acq_wr_seq,
               s_final_edge_b_last_result.acq_rd_seq,
               s_final_edge_b_last_result.acq_skipped_after_stop);
    DBG_DIRECT("[FE_BFAIL7] gate=%u pub=%u dup=%u mismatch=%u tmst_read=%u raw_read=%u",
               s_final_edge_b_last_result.acq_tmst_gate_enabled,
               s_final_edge_b_last_result.acq_tmst_new_count,
               s_final_edge_b_last_result.acq_tmst_dup_drop,
               s_final_edge_b_last_result.acq_tmst_raw_mismatch,
               s_final_edge_b_last_result.acq_tmst_read_err,
               s_final_edge_b_last_result.acq_raw_read_err);
    DBG_DIRECT("[FE_BFAIL3] owner=%u clean=%u hwin=%u/%u last=%u target=%u",
               s_final_edge_b_last_result.owner_ok,
               s_final_edge_b_last_result.cleanup_ok,
               s_final_edge_last_hit_window_view.valid,
               s_final_edge_last_hit_window_view.complete,
               s_final_edge_b_last_result.last_saved_seq,
               s_final_edge_b_last_result.target_end_seq);
#else
    IMU_UNUSED(run_ok);
#endif
}

#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
static uint32_t v0_final_edge_online_raw_bucket_next(
    const zy100_fe_raw_store_stats_t *raw_stats)
{
    if (raw_stats == NULL)
    {
        return 0xFFFFFFFFUL;
    }
    if (raw_stats->next_raw_id >= raw_stats->bucket_count)
    {
        return 0xFFFFFFFFUL;
    }
    return raw_stats->session_raw_first_bucket + raw_stats->next_raw_id;
}

static uint32_t v0_final_edge_online_raw_bucket_last(
    const zy100_fe_raw_store_stats_t *raw_stats)
{
    if (raw_stats == NULL)
    {
        return 0xFFFFFFFFUL;
    }
    if (raw_stats->last_raw_id >= raw_stats->bucket_count)
    {
        return 0xFFFFFFFFUL;
    }
    return raw_stats->session_raw_first_bucket + raw_stats->last_raw_id;
}

static uint32_t v0_final_edge_online_raw_bucket_error(
    const zy100_fe_raw_store_stats_t *raw_stats)
{
    if (raw_stats == NULL)
    {
        return 0xFFFFFFFFUL;
    }
    if (raw_stats->last_error_raw_id >= raw_stats->bucket_count)
    {
        return 0xFFFFFFFFUL;
    }
    return raw_stats->session_raw_first_bucket + raw_stats->last_error_raw_id;
}

static void v0_final_edge_online_hit_log(
    const char *event,
    uint32_t bscan_id,
    uint32_t raw_bucket,
    const zy100_fe_raw_store_stats_t *raw_stats)
{
#if ZY100_ONLINE_STREAM_ENABLE
#if ZY100_ONLINE_STREAM_VERBOSE_TRACE_ENABLE
    zy100_fe_raw_store_stats_t raw_local;
    zy100_final_edge_queue_stats_t queue_stats;
    zy100_fe_record_store_stats_t record_stats;

    if (!zy100_online_stream_active() &&
        !zy100_online_stream_end_wait_ack())
    {
        return;
    }

    memset(&raw_local, 0, sizeof(raw_local));
    if (raw_stats == NULL)
    {
        zy100_final_edge_raw_store_get_stats(&raw_local);
        raw_stats = &raw_local;
    }
    memset(&queue_stats, 0, sizeof(queue_stats));
    memset(&record_stats, 0, sizeof(record_stats));
    zy100_final_edge_queue_get_stats(&queue_stats);
    zy100_final_edge_record_store_get_stats(&record_stats);

    ZY100_DIAG_LOG("[ONLINE_HIT] ev=%s b=%lu bucket=%lu raw_begin=%lu raw_saved=%lu raw_drop=%lu evt_enq=%lu evt_saved=%lu evt_drop=%lu",
               (event != NULL) ? event : "unknown",
               (unsigned long)bscan_id,
               (unsigned long)raw_bucket,
               (unsigned long)raw_stats->raw_begin,
               (unsigned long)raw_stats->raw_saved,
               (unsigned long)raw_stats->raw_failed,
               (unsigned long)queue_stats.event_enqueued,
               (unsigned long)record_stats.event_saved,
               (unsigned long)queue_stats.event_dropped);
#else
    (void)event;
    (void)bscan_id;
    (void)raw_bucket;
    (void)raw_stats;
#endif
#else
    (void)event;
    (void)bscan_id;
    (void)raw_bucket;
    (void)raw_stats;
#endif
}
#endif

#if (ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE && \
     ZY100_FINAL_EDGE_PHASE4_HIT_WINDOW_ENABLE)
static void v0_final_edge_phase5_begin_hit_after_b(uint32_t fail_reason)
{
    const uint32_t true_hit_hard_mask =
        FE_BFAIL_REASON_READ_ERR |
        FE_BFAIL_REASON_OVER |
        FE_BFAIL_REASON_DROP |
#if ZY100_FINAL_EDGE_B_HARD_RT_ACQ_ENABLE
        FE_BFAIL_REASON_STALE |
        FE_BFAIL_REASON_DT_BAD |
        FE_BFAIL_REASON_ACQ_OVERFLOW |
        FE_BFAIL_REASON_ACQ_READ_ERR |
#if (!ZY100_FINAL_EDGE_B_ACQ_TMST_GATE_ENABLE && \
     !ZY100_FINAL_EDGE_B_ACQ_RAW_PHASE_LOCK_ENABLE)
        FE_BFAIL_REASON_ACQ_IRQ_LATE |
#endif
        FE_BFAIL_REASON_ACQ_TMST_MISMATCH |
#if ZY100_FINAL_EDGE_B_ACQ_REUSE_HIT_RING_ENABLE
        FE_BFAIL_REASON_ACQ_BACKLOG |
#endif
#endif
        FE_BFAIL_REASON_ERR |
        FE_BFAIL_REASON_OWNER |
        FE_BFAIL_REASON_CLEANUP |
        FE_BFAIL_REASON_RUN_FAIL |
        FE_BFAIL_REASON_BEGIN_FAIL;
    zy100_fe_raw_store_stats_t raw_stats_before;
    uint32_t raw_bucket;
    bool ois_od_warn;

    if (s_final_edge_b_last_result.hit_found == 0U)
    {
        return;
    }
    memset(&raw_stats_before, 0, sizeof(raw_stats_before));
    zy100_final_edge_raw_store_get_stats(&raw_stats_before);
    raw_bucket = v0_final_edge_online_raw_bucket_next(&raw_stats_before);
    if ((fail_reason & true_hit_hard_mask) != 0U)
    {
        v0_final_edge_online_hit_log("raw_skip_hwin",
                                     s_final_edge_last_b_id,
                                     raw_bucket,
                                     &raw_stats_before);
        zy100_online_stream_note_record_dropped_ex(
            ZY100_ONLINE_RECORD_RAW,
            ZY100_ONLINE_DROP_HWIN,
            fail_reason,
            ZY100_FE_STORE_RESERVE_OK,
            (uint8_t)s_fe_capture_state,
            zy100_os_time_ms());
        return;
    }
    if (s_final_edge_b_last_result.post_truncated != 0U)
    {
        v0_final_edge_online_hit_log("raw_skip_trunc",
                                     s_final_edge_last_b_id,
                                     raw_bucket,
                                     &raw_stats_before);
        zy100_online_stream_note_record_dropped_ex(
            ZY100_ONLINE_RECORD_RAW,
            ZY100_ONLINE_DROP_TRUNCATED,
            s_final_edge_b_last_result.post_truncated,
            ZY100_FE_STORE_RESERVE_OK,
            (uint8_t)s_fe_capture_state,
            zy100_os_time_ms());
        return;
    }
    ois_od_warn =
        (s_final_edge_truehit_od_warn_last_id == s_final_edge_last_b_id) &&
        (s_final_edge_truehit_od_warn_last_reason != 0U);

    if (zy100_final_edge_raw_store_begin_hit(
            s_final_edge_b_ois_ring,
            sizeof(s_final_edge_b_ois_ring),
            &s_final_edge_b_last_result,
            &s_final_edge_last_hit_window_view,
            s_final_edge_last_b_id,
            s_final_edge_rt_trigger_sample_index,
            s_final_edge_rt_trigger_tmst_raw,
            ois_od_warn))
    {
        zy100_fe_raw_store_stats_t raw_stats;

        if (zy100_online_stream_active())
        {
            zy100_online_stream_note_raw_accepted(s_final_edge_last_b_id);
        }
        memset(&raw_stats, 0, sizeof(raw_stats));
        zy100_final_edge_raw_store_get_stats(&raw_stats);
        v0_final_edge_online_hit_log("raw_begin",
                                     raw_stats.last_bscan_id,
                                     v0_final_edge_online_raw_bucket_last(&raw_stats),
                                     &raw_stats);
#if ZY100_FINAL_EDGE_LF_PRE_DIAG_ENABLE
        if (s_final_edge_last_hit_window_view.lf_pre_packets_needed != 0U)
        {
            zy100_final_edge_lf_pre_ring_diag_note_saved_section(
                raw_stats.last_raw_id,
                s_final_edge_rt_trigger_sample_index,
                s_final_edge_last_hit_window_view.lf_pre_packets_needed);
        }
#endif
    }
    else
    {
        zy100_fe_raw_store_stats_t raw_stats;

        memset(&raw_stats, 0, sizeof(raw_stats));
        zy100_final_edge_raw_store_get_stats(&raw_stats);
        v0_final_edge_online_hit_log("raw_begin_fail",
                                     s_final_edge_last_b_id,
                                     raw_bucket,
                                     &raw_stats);
        zy100_online_stream_note_record_dropped_ex(
            ZY100_ONLINE_RECORD_RAW,
            ZY100_ONLINE_DROP_BEGIN,
            s_final_edge_last_b_id,
            (zy100_fe_store_reserve_result_t)
                raw_stats.target_last_reserve_result,
            (uint8_t)s_fe_capture_state,
            zy100_os_time_ms());
    }
}
#endif

#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
static void v0_final_edge_replay_begin_common(
    zy100_fe_replay_source_t source,
    uint32_t sample_count)
{
    s_final_edge_replay_pending = true;
    s_final_edge_replay_active = false;
    s_final_edge_replay_index = 0U;
    s_final_edge_replay_count = sample_count;
    s_final_edge_replay_bscan_id = s_final_edge_last_b_id;
    s_final_edge_replay_started_ms = 0U;
    s_final_edge_replay_last_ms = 0U;
    s_final_edge_replay_backlog_start_count = 0U;
    s_final_edge_replay_source = source;

    if (source == ZY100_FE_REPLAY_SRC_TRUEHIT)
    {
        s_final_edge_phase1_stats.replay_truehit_begin_count++;
        s_final_edge_phase1_stats.replay_truehit_samples_expected +=
            sample_count;
    }
    else
    {
        s_final_edge_phase1_stats.replay_begin_count++;
        s_final_edge_phase1_stats.replay_samples_expected +=
            ZY100_FINAL_EDGE_NOHIT_REPLAY_EXPECT_SAMPLES;
    }
}

static void v0_final_edge_phase6_fail_nohit_replay(
    const char *reason_text,
    uint32_t aux0,
    uint32_t aux1)
{
    s_final_edge_phase1_stats.replay_fail_count++;
#if (ZY100_FINAL_EDGE_CAPTURE_RUNTIME_LOG_QUIET_ENABLE == 0U)
    DBG_DIRECT("[FE_REPLAY_FAIL] id=%u reason=%s a0=%u a1=%u",
               s_final_edge_last_b_id,
               (reason_text != NULL) ? reason_text : "unknown",
               aux0,
               aux1);
#else
    IMU_UNUSED(reason_text);
    IMU_UNUSED(aux0);
    IMU_UNUSED(aux1);
#endif
}

static void v0_final_edge_phase61_fail_truehit_replay(
    const char *reason_text,
    uint32_t aux0,
    uint32_t aux1)
{
    s_final_edge_phase1_stats.replay_truehit_fail_count++;
#if (ZY100_FINAL_EDGE_CAPTURE_RUNTIME_LOG_QUIET_ENABLE == 0U)
    DBG_DIRECT("[FE_REPLAY_FAIL] id=%u src=hit reason=%s a0=%u a1=%u",
               s_final_edge_last_b_id,
               (reason_text != NULL) ? reason_text : "unknown",
               aux0,
               aux1);
#else
    IMU_UNUSED(reason_text);
    IMU_UNUSED(aux0);
    IMU_UNUSED(aux1);
#endif
}

static void v0_final_edge_phase6_begin_nohit_after_b(uint32_t fail_reason)
{
    const zy100_final_edge_bscan_result_t *result =
        &s_final_edge_b_last_result;
    uint32_t ds_count;
    uint32_t ds_overflow;
    bool od_warn;

    if (result->hit_found != 0U)
    {
        return;
    }

    s_final_edge_phase1_stats.replay_nohit_count++;
    od_warn =
        (s_final_edge_nohit_od_warn_last_id == s_final_edge_last_b_id) &&
        (s_final_edge_nohit_od_warn_last_reason != 0U);
    if (od_warn)
    {
        s_final_edge_phase1_stats.replay_od_warn_count++;
    }

    ds_count = zy100_final_edge_bscan_get_ds_count();
    ds_overflow = zy100_final_edge_bscan_get_ds_overflow();

    if (v0_final_edge_replay_busy())
    {
        v0_final_edge_phase6_fail_nohit_replay("busy",
                                               s_final_edge_replay_pending ? 1U : 0U,
                                               s_final_edge_replay_active ? 1U : 0U);
        return;
    }
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    if (!zy100_final_edge_raw_store_is_idle())
    {
#if (ZY100_FINAL_EDGE_CAPTURE_RUNTIME_LOG_QUIET_ENABLE == 0U)
        ZY100_DIAG_LOG("[FE_REPLAY_FATAL] id=%u reason=raw_store_busy",
                   s_final_edge_last_b_id);
#endif
        v0_final_edge_phase6_fail_nohit_replay("raw_busy", 0U, 0U);
        return;
    }
#endif
#if (ZY100_FINAL_EDGE_REPLAY_OD_WARN_ALLOW == 0U)
    if (od_warn)
    {
        v0_final_edge_phase6_fail_nohit_replay("od_warn",
                                               s_final_edge_nohit_od_warn_last_reason,
                                               0U);
        return;
    }
#endif
    if (fail_reason != 0U)
    {
        v0_final_edge_phase6_fail_nohit_replay("b_fail", fail_reason, 0U);
        return;
    }
    if (result->frames != ZY100_FINAL_EDGE_B_SCAN_BASE_FRAMES)
    {
        v0_final_edge_phase6_fail_nohit_replay("frames",
                                               result->frames,
                                               ZY100_FINAL_EDGE_B_SCAN_BASE_FRAMES);
        return;
    }
    if ((result->read_err != 0U) ||
#if ZY100_FINAL_EDGE_B_HARD_RT_ACQ_ENABLE
        (result->stale != 0U) ||
        (result->dt_bad != 0U) ||
        (result->acq_overflow != 0U) ||
        (result->acq_read_err != 0U) ||
#if (!ZY100_FINAL_EDGE_B_ACQ_TMST_GATE_ENABLE && \
     !ZY100_FINAL_EDGE_B_ACQ_RAW_PHASE_LOCK_ENABLE)
        (result->acq_irq_late_count != 0U) ||
#endif
        (result->acq_tmst_raw_mismatch != 0U) ||
#if ZY100_FINAL_EDGE_B_ACQ_REUSE_HIT_RING_ENABLE
        (result->acq_high_water >=
         ZY100_FINAL_EDGE_B_ACQ_BACKLOG_HARD_FRAMES) ||
#endif
#endif
        (result->owner_ok == 0U) ||
        (result->cleanup_ok == 0U))
    {
        v0_final_edge_phase6_fail_nohit_replay("hard",
                                               result->read_err,
                                               (result->owner_ok << 1) |
                                               result->cleanup_ok);
        return;
    }
    if (ds_overflow != 0U)
    {
        s_final_edge_phase1_stats.replay_ds_overflow_count++;
        v0_final_edge_phase6_fail_nohit_replay("ds_over",
                                               ds_overflow,
                                               ds_count);
        return;
    }
    if (ds_count != ZY100_FINAL_EDGE_NOHIT_REPLAY_EXPECT_SAMPLES)
    {
        s_final_edge_phase1_stats.replay_ds_bad_count++;
        v0_final_edge_phase6_fail_nohit_replay("ds_count",
                                               ds_count,
                                               ZY100_FINAL_EDGE_NOHIT_REPLAY_EXPECT_SAMPLES);
        return;
    }

    v0_final_edge_replay_begin_common(ZY100_FE_REPLAY_SRC_NOHIT,
                                      ds_count);
}

#if (ZY100_FINAL_EDGE_PHASE61_TRUEHIT_REPLAY_ENABLE && \
     ZY100_FINAL_EDGE_TRUEHIT_REPLAY_ENABLE && \
     ZY100_FINAL_EDGE_PHASE4_HIT_WINDOW_ENABLE)
static void v0_final_edge_phase61_begin_truehit_after_b(uint32_t fail_reason)
{
    const zy100_final_edge_bscan_result_t *result =
        &s_final_edge_b_last_result;
    uint32_t ds_count;
    uint32_t ds_overflow;

    if (result->hit_found == 0U)
    {
        return;
    }

    if (!v0_final_edge_phase4_latest_hwin_ok(result))
    {
        v0_final_edge_phase61_fail_truehit_replay(
            "hwin",
            s_final_edge_last_hit_window_view.valid,
            s_final_edge_last_hit_window_view.complete);
        return;
    }
    if (result->post_truncated != 0U)
    {
        v0_final_edge_phase61_fail_truehit_replay("post_trunc",
                                                  result->post_truncated,
                                                  0U);
        return;
    }
    if ((result->read_err != 0U) ||
#if ZY100_FINAL_EDGE_B_HARD_RT_ACQ_ENABLE
        (result->stale != 0U) ||
        (result->dt_bad != 0U) ||
        (result->acq_overflow != 0U) ||
        (result->acq_read_err != 0U) ||
#if (!ZY100_FINAL_EDGE_B_ACQ_TMST_GATE_ENABLE && \
     !ZY100_FINAL_EDGE_B_ACQ_RAW_PHASE_LOCK_ENABLE)
        (result->acq_irq_late_count != 0U) ||
#endif
        (result->acq_tmst_raw_mismatch != 0U) ||
#if ZY100_FINAL_EDGE_B_ACQ_REUSE_HIT_RING_ENABLE
        (result->acq_high_water >=
         ZY100_FINAL_EDGE_B_ACQ_BACKLOG_HARD_FRAMES) ||
#endif
#endif
        (result->owner_ok == 0U) ||
        (result->cleanup_ok == 0U))
    {
        v0_final_edge_phase61_fail_truehit_replay(
            "hard",
            result->read_err,
            (result->owner_ok << 1) | result->cleanup_ok);
        return;
    }
    if (fail_reason != 0U)
    {
        v0_final_edge_phase61_fail_truehit_replay("b_fail",
                                                  fail_reason,
                                                  0U);
        return;
    }

    ds_count = zy100_final_edge_bscan_get_ds_count();
    ds_overflow = zy100_final_edge_bscan_get_ds_overflow();
    if (ds_overflow != 0U)
    {
        s_final_edge_phase1_stats.replay_ds_overflow_count++;
        v0_final_edge_phase61_fail_truehit_replay("ds_over",
                                                  ds_overflow,
                                                  ds_count);
        return;
    }
    if ((ds_count == 0U) ||
        (ds_count > ZY100_FINAL_EDGE_OIS_DS_MAX_SAMPLES))
    {
        s_final_edge_phase1_stats.replay_ds_bad_count++;
        v0_final_edge_phase61_fail_truehit_replay(
            "ds_count",
            ds_count,
            ZY100_FINAL_EDGE_OIS_DS_MAX_SAMPLES);
        return;
    }

    s_final_edge_phase1_stats.replay_truehit_count++;
    if (v0_final_edge_replay_busy())
    {
        v0_final_edge_phase61_fail_truehit_replay(
            "busy",
            s_final_edge_replay_pending ? 1U : 0U,
            s_final_edge_replay_active ? 1U : 0U);
        return;
    }

    v0_final_edge_replay_begin_common(ZY100_FE_REPLAY_SRC_TRUEHIT,
                                      ds_count);
}
#endif
#endif

static void v0_final_edge_phase3_reset(void)
{
    v0_final_edge_reset_b_last_result();
    v0_final_edge_od_diag_reset();
    v0_final_edge_flash_stats_reset();
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    zy100_final_edge_raw_store_reset_runtime();
#endif
    zy100_final_edge_record_store_reset_runtime();
#if ZY100_FINAL_EDGE_PHASE4_HIT_WINDOW_ENABLE
    v0_final_edge_phase4_reset();
#endif
#if ZY100_FINAL_EDGE_HIT20_CALIB_ENABLE
    v0_final_edge_hit20_calib_reset_stats();
#endif
    s_final_edge_b_scan_active = false;
    s_final_edge_rt_gated_for_b = false;
    s_final_edge_rt_trigger_valid = false;
    s_final_edge_resume_ok = false;
    s_final_edge_rt_trigger_tmst_raw = 0U;
    s_final_edge_b_scan_count = 0U;
    s_final_edge_b_scan_fail_count = 0U;
    s_final_edge_b_fail_reason_or = 0U;
    s_final_edge_b_fail_last_reason = 0U;
    s_final_edge_b_fail_last_id = 0U;
    s_final_edge_b_fail_last_frames = 0U;
    s_final_edge_b_fail_last_hit = 0U;
    s_final_edge_b_fail_last_exit_reason = 0U;
    s_final_edge_b_fail_last_seq = ZY100_FINAL_EDGE_BSCAN_SEQ32_INVALID;
    s_final_edge_b_fail_last_elapsed_ms = 0U;
    s_final_edge_b_fail_hard_count = 0U;
    s_final_edge_b_fail_frame_count = 0U;
    s_final_edge_b_fail_hwin_count = 0U;
    s_final_edge_b_fail_trunc_count = 0U;
    s_final_edge_b_begin_fail_count = 0U;
    s_final_edge_b_begin_fail_last_stage = 0U;
    s_final_edge_b_begin_fail_last_status = 0U;
    s_final_edge_b_begin_fail_last_retry = 0U;
    s_final_edge_b_begin_fail_last_owner = 0U;
    s_final_edge_b_begin_fail_last_cleanup = 0U;
    s_final_edge_b_nohit_frame_min = 0U;
    s_final_edge_b_nohit_frame_max = 0U;
    s_final_edge_b_nohit_frame_last = 0U;
    s_final_edge_b_nohit_frame_sum_short = 0U;
    s_final_edge_b_nohit_frame_last_exit = 0U;
    s_final_edge_b_mismatch_count = 0U;
    s_final_edge_b_mismatch_hit_count = 0U;
    s_final_edge_b_mismatch_nohit_count = 0U;
    s_final_edge_b_mismatch_last_id = 0U;
    s_final_edge_b_mismatch_last_seq =
        ZY100_FINAL_EDGE_BSCAN_SEQ32_INVALID;
    s_final_edge_b_mismatch_last_probe = 0U;
    s_final_edge_b_mismatch_last_raw = 0U;
    s_final_edge_nohit_od_warn_count = 0U;
    s_final_edge_nohit_od_warn_or = 0U;
    s_final_edge_nohit_od_warn_last_id = 0U;
    s_final_edge_nohit_od_warn_last_reason = 0U;
    s_final_edge_nohit_od_warn_last_run = 0U;
    s_final_edge_nohit_od_warn_last_frames = 0U;
    s_final_edge_nohit_od_warn_last_err = 0U;
    s_final_edge_nohit_od_warn_last_over = 0U;
    s_final_edge_nohit_od_warn_last_drop = 0U;
    s_final_edge_truehit_od_warn_count = 0U;
    s_final_edge_truehit_od_warn_or = 0U;
    s_final_edge_truehit_od_warn_last_id = 0U;
    s_final_edge_truehit_od_warn_last_reason = 0U;
    s_final_edge_truehit_od_warn_last_run = 0U;
    s_final_edge_truehit_od_warn_last_frames = 0U;
    s_final_edge_truehit_od_warn_last_over = 0U;
    s_final_edge_truehit_od_warn_last_drop = 0U;
    s_final_edge_restore_ok_count = 0U;
    s_final_edge_restore_fail_count = 0U;
    s_final_edge_official_hit_count = 0U;
    s_final_edge_official_nohit_count = 0U;
    s_final_edge_last_b_id = 0U;
    s_final_edge_rt_trigger_sample_index = 0U;
    s_final_edge_restore_fallback_count = 0U;
    s_final_edge_restore_fallback_status = 0U;
    s_final_edge_b_critical_state = V0_FE_B_CRITICAL_IDLE;
    s_final_edge_b_pending_start_us = 0U;
#if ZY100_FINAL_EDGE_B_CRITICAL_GATE_ENABLE && \
    ZY100_FINAL_EDGE_OFFLINE_B_CRITICAL_BYPASS_VERIFY_ENABLE
    s_final_edge_b_critical_offline_bypass_logged = false;
#endif
    v0_final_edge_post_restore_quiet_clear();
}
#endif

#if !ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
static bool v0_final_edge_b_critical_active(void)
{
    return false;
}
#endif

static void v0_final_edge_reset(void)
{
    IMU_UNUSED(s_v0_phase_a_rescue_pending);
    memset(&s_final_edge_phase1_stats, 0, sizeof(s_final_edge_phase1_stats));
    s_final_edge_last_rt_trigger_ms = 0U;
    v0_final_edge_wm_summary_reset();
    s_fe_capture_state = FE_CAPTURE_STATE_IDLE;
    s_fe_stop_reason = ZY100_FE_STOP_REASON_NONE;
    s_fe_export_ready = false;
    s_fe_stop_seen = false;
    s_fe_final_drain_done = false;
    s_fe_export_cleared = false;
    s_fe_export_preserved = false;
    s_fe_stop_request_ms = 0U;
    s_fe_final_drain_active = false;
    s_final_edge_no_new_data_mode = false;
    s_final_edge_live_timer_stopped_for_drain = false;
    s_final_edge_imu_live_stopped_for_drain = false;
    s_v0_online_pause_stop_requested = false;
    zy100_final_edge_queue_reset();
    s_final_edge_rt_deferred_active = false;
    s_final_edge_b_critical_state = V0_FE_B_CRITICAL_IDLE;
    s_final_edge_b_pending_start_us = 0U;
#if ZY100_FINAL_EDGE_B_CRITICAL_GATE_ENABLE && \
    ZY100_FINAL_EDGE_OFFLINE_B_CRITICAL_BYPASS_VERIFY_ENABLE
    s_final_edge_b_critical_offline_bypass_logged = false;
#endif
    v0_final_edge_post_restore_quiet_clear();
    s_final_edge_edge_sample_index = 0U;
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
    s_final_edge_replay_pending = false;
    s_final_edge_replay_active = false;
    s_final_edge_replay_backlog_edge_only_active = false;
    s_final_edge_replay_index = 0U;
    s_final_edge_replay_count = 0U;
    s_final_edge_replay_bscan_id = 0U;
    s_final_edge_replay_started_ms = 0U;
    s_final_edge_replay_last_ms = 0U;
    s_final_edge_replay_backlog_start_count = 0U;
    s_final_edge_replay_source = ZY100_FE_REPLAY_SRC_NOHIT;
#endif
    s_final_edge_phase1_stats.timing_api_ok = true;
    s_final_edge_timer_due_count = 0U;
    s_final_edge_timer_total_due_count = 0U;
    s_final_edge_timer_last_us = 0U;
    s_final_edge_timer_gap_max_us = 0U;
    s_final_edge_timer_notify_fail_count = 0U;
    s_final_edge_timer_running = false;
#if ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
    v0_final_edge_phase3_reset();
#endif
}

static uint32_t v0_final_edge_rate_per_s(uint32_t count, uint32_t runtime_ms)
{
    uint64_t rate;

    if (runtime_ms == 0U)
    {
        return 0U;
    }

    rate = ((uint64_t)count * 1000ULL) / (uint64_t)runtime_ms;
    return (rate > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)rate;
}

static void v0_final_edge_timer_notify_from_isr(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t now_us = (uint32_t)imu_bsp_local_timestamp_us();
    uint32_t last_us = s_final_edge_timer_last_us;

    if (last_us != 0U)
    {
        uint32_t gap_us = now_us - last_us;

        if (gap_us > s_final_edge_timer_gap_max_us)
        {
            s_final_edge_timer_gap_max_us = gap_us;
        }
    }
    s_final_edge_timer_last_us = now_us;

    if (s_final_edge_timer_due_count < 0xFFFFFFFFU)
    {
        s_final_edge_timer_due_count++;
    }
    if (s_final_edge_timer_total_due_count < 0xFFFFFFFFU)
    {
        s_final_edge_timer_total_due_count++;
    }
    if (s_v0_task_handle != NULL)
    {
        BaseType_t notify_status;

        notify_status = xTaskNotifyFromISR(s_v0_task_handle,
                                           V0_NOTIFY_FE_TIMER,
                                           eSetBits,
                                           &xHigherPriorityTaskWoken);
        if ((notify_status != pdPASS) &&
            (s_final_edge_timer_notify_fail_count < 0xFFFFFFFFU))
        {
            s_final_edge_timer_notify_fail_count++;
        }
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static uint32_t v0_final_edge_consume_timer_due(void)
{
    uint32_t lock_state;
    uint32_t due;

    lock_state = os_lock();
    due = s_final_edge_timer_due_count;
    s_final_edge_timer_due_count = 0U;
    os_unlock(lock_state);

    return due;
}

static void v0_final_edge_clear_timer_due(void)
{
    uint32_t lock_state;

    lock_state = os_lock();
    s_final_edge_timer_due_count = 0U;
    os_unlock(lock_state);
}

static bool v0_final_edge_timer_start(void)
{
    imu_status_t status;

    status = imu_bsp_ois_tick_timer_acquire(
        IMU_BSP_OIS_TICK_TIMER_OWNER_FINAL_EDGE_LIVE);
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[FE_TIMER] acquire_fail status=%u owner=%u",
                   (uint32_t)status,
                   (uint32_t)imu_bsp_ois_tick_timer_get_owner());
        return false;
    }

    v0_final_edge_clear_timer_due();
    s_final_edge_timer_last_us = 0U;
    s_final_edge_timer_gap_max_us = 0U;
    imu_bsp_ois_tick_timer_register_irq_callback(v0_final_edge_timer_notify_from_isr);
    status = imu_bsp_ois_tick_timer_config(ZY100_FINAL_EDGE_TIMER_PERIOD_US);
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_ois_tick_timer_start();
    }
    if (status != IMU_STATUS_OK)
    {
        imu_bsp_ois_tick_timer_stop();
        imu_bsp_ois_tick_timer_register_irq_callback(NULL);
        imu_bsp_ois_tick_timer_release(
            IMU_BSP_OIS_TICK_TIMER_OWNER_FINAL_EDGE_LIVE);
        DBG_DIRECT("[FE_TIMER] start_fail status=%u", (uint32_t)status);
        return false;
    }

    s_final_edge_timer_running = true;
    return true;
}

static void v0_final_edge_timer_stop(void)
{
    if (s_final_edge_timer_running ||
        (imu_bsp_ois_tick_timer_get_owner() ==
         IMU_BSP_OIS_TICK_TIMER_OWNER_FINAL_EDGE_LIVE))
    {
        imu_bsp_ois_tick_timer_stop();
        imu_bsp_ois_tick_timer_register_irq_callback(NULL);
        imu_bsp_ois_tick_timer_release(
            IMU_BSP_OIS_TICK_TIMER_OWNER_FINAL_EDGE_LIVE);
    }
    s_final_edge_timer_running = false;
    v0_final_edge_clear_timer_due();
}

static void v0_final_edge_note_dt(uint16_t dt_us, bool bad)
{
    if (!s_final_edge_phase1_stats.dt_seen)
    {
        s_final_edge_phase1_stats.dt_seen = true;
        s_final_edge_phase1_stats.dt_min_us = dt_us;
        s_final_edge_phase1_stats.dt_max_us = dt_us;
    }
    else
    {
        if ((uint32_t)dt_us < s_final_edge_phase1_stats.dt_min_us)
        {
            s_final_edge_phase1_stats.dt_min_us = dt_us;
        }
        if ((uint32_t)dt_us > s_final_edge_phase1_stats.dt_max_us)
        {
            s_final_edge_phase1_stats.dt_max_us = dt_us;
        }
    }
    if (bad)
    {
        s_final_edge_phase1_stats.dt_bad_count++;
    }
}

static uint32_t v0_final_edge_rt_marker_count(void)
{
    imu_rt_marker_stats_t rt_stats;

    memset(&rt_stats, 0, sizeof(rt_stats));
    imu_rt_marker_get_stats(&rt_stats);
    return rt_stats.marker_total;
}

static void v0_final_edge_handle_lite_output(const zy100_edge_lite_output_t *out,
                                             uint32_t edge_sample_index,
                                             uint16_t timestamp_raw)
{
    bool raw_req;
    bool raw_start;

    if (out == NULL)
    {
        return;
    }

    if ((out->flags & ZY100_EDGE_LITE_OUT_SUMMARY) != 0U)
    {
        s_final_edge_phase1_stats.edge_summary_count++;
        (void)zy100_final_edge_queue_push_summary_lite(&out->summary,
                                                       edge_sample_index,
                                                       timestamp_raw);
    }
    if ((out->flags & ZY100_EDGE_LITE_OUT_EVENT) != 0U)
    {
        s_final_edge_phase1_stats.edge_event_count++;
        (void)zy100_final_edge_queue_push_event_lite(&out->event,
                                                     edge_sample_index,
                                                     timestamp_raw);
    }

    raw_req = ((out->flags & ZY100_EDGE_LITE_OUT_RAW_REQ) != 0U);
    raw_start = ((out->flags & ZY100_EDGE_LITE_OUT_RAW_START) != 0U);
    if (raw_req)
    {
        s_final_edge_phase1_stats.edge_raw_req_count++;
        s_final_edge_phase1_stats.edge_raw_sup_count++;
        zy100_final_edge_queue_note_raw_req_suppressed();
    }
    if (raw_start)
    {
        zy100_final_edge_queue_note_raw_start_suppressed();
    }
}

static uint32_t v0_final_edge_next_edge_sample_index(void)
{
    if (s_final_edge_edge_sample_index < 0xFFFFFFFFU)
    {
        s_final_edge_edge_sample_index++;
    }
    return s_final_edge_edge_sample_index;
}

#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
static void v0_final_edge_replay_note_queue_drop(
    const zy100_final_edge_queue_stats_t *before,
    const zy100_final_edge_queue_stats_t *after,
    uint32_t bscan_id)
{
    uint32_t sum_delta;
    uint32_t evt_delta;

    if ((before == NULL) || (after == NULL))
    {
        return;
    }

    sum_delta = after->summary_dropped - before->summary_dropped;
    evt_delta = after->event_dropped - before->event_dropped;
    if (sum_delta != 0U)
    {
        s_final_edge_phase1_stats.replay_queue_sum_drop_delta += sum_delta;
        DBG_DIRECT("[FE_REPLAY_QWARN] id=%u sum_drop_delta=%u total=%u",
                   bscan_id,
                   sum_delta,
                   after->summary_dropped);
    }
    if (evt_delta != 0U)
    {
        s_final_edge_phase1_stats.replay_queue_evt_drop_delta += evt_delta;
        DBG_DIRECT("[FE_REPLAY_QWARN] id=%u evt_drop_delta=%u total=%u",
                   bscan_id,
                   evt_delta,
                   after->event_dropped);
    }
}

static void v0_final_edge_handle_lite_output_replay(
    const zy100_edge_lite_output_t *out,
    uint32_t edge_sample_index,
    uint16_t timestamp_raw)
{
    bool raw_req;
    bool raw_start;
    uint32_t source_flags;
    zy100_final_edge_queue_stats_t before;
    zy100_final_edge_queue_stats_t after;

    if (out == NULL)
    {
        return;
    }

    source_flags =
        (s_final_edge_replay_source == ZY100_FE_REPLAY_SRC_TRUEHIT) ?
        ZY100_FINAL_EDGE_QUEUE_SOURCE_REPLAY_TRUEHIT :
        ZY100_FINAL_EDGE_QUEUE_SOURCE_REPLAY_NOHIT;

    memset(&before, 0, sizeof(before));
    memset(&after, 0, sizeof(after));
    zy100_final_edge_queue_get_stats(&before);

    if ((out->flags & ZY100_EDGE_LITE_OUT_SUMMARY) != 0U)
    {
        s_final_edge_phase1_stats.replay_summary_count++;
        (void)zy100_final_edge_queue_push_summary_lite_ex(&out->summary,
                                                          edge_sample_index,
                                                          timestamp_raw,
                                                          source_flags);
    }
    if ((out->flags & ZY100_EDGE_LITE_OUT_EVENT) != 0U)
    {
        s_final_edge_phase1_stats.replay_event_count++;
        (void)zy100_final_edge_queue_push_event_lite_ex(&out->event,
                                                        edge_sample_index,
                                                        timestamp_raw,
                                                        source_flags);
    }

    raw_req = ((out->flags & ZY100_EDGE_LITE_OUT_RAW_REQ) != 0U);
    raw_start = ((out->flags & ZY100_EDGE_LITE_OUT_RAW_START) != 0U);
    if (raw_req)
    {
        s_final_edge_phase1_stats.replay_raw_sup_count++;
        zy100_final_edge_queue_note_raw_req_suppressed();
    }
    if (raw_start)
    {
        zy100_final_edge_queue_note_raw_start_suppressed();
    }

    zy100_final_edge_queue_get_stats(&after);
    v0_final_edge_replay_note_queue_drop(&before,
                                         &after,
                                         s_final_edge_replay_bscan_id);
}

static bool v0_final_edge_replay_busy(void)
{
    return s_final_edge_replay_pending || s_final_edge_replay_active;
}

static bool v0_final_edge_replay_needs_service(void)
{
    return v0_final_edge_replay_busy() ||
           s_final_edge_replay_backlog_edge_only_active;
}

static uint32_t v0_final_edge_replay_edge_total(void)
{
    return s_final_edge_phase1_stats.replay_edge_feed +
           s_final_edge_phase1_stats.replay_truehit_edge_feed;
}

static void v0_final_edge_replay_note_edge_error(void)
{
    if (s_final_edge_replay_source == ZY100_FE_REPLAY_SRC_TRUEHIT)
    {
        s_final_edge_phase1_stats.replay_truehit_edge_error++;
    }
    else
    {
        s_final_edge_phase1_stats.replay_edge_error++;
    }
}

static void v0_final_edge_replay_note_sample_fed(void)
{
    if (s_final_edge_replay_source == ZY100_FE_REPLAY_SRC_TRUEHIT)
    {
        s_final_edge_phase1_stats.replay_truehit_edge_feed++;
        s_final_edge_phase1_stats.replay_truehit_samples_fed++;
    }
    else
    {
        s_final_edge_phase1_stats.replay_edge_feed++;
        s_final_edge_phase1_stats.replay_samples_fed++;
    }
}

static bool v0_final_edge_feed_edge_from_replay_sample(
    const zy100_final_edge_bscan_ds_sample_t *sample)
{
    zy100_edge_shadow_sample_t edge_sample;
    zy100_edge_lite_output_t edge_out;
    uint32_t sample_start_us;
    uint32_t sample_elapsed_us;

    if (sample == NULL)
    {
        v0_final_edge_replay_note_edge_error();
        return false;
    }

    memset(&edge_sample, 0, sizeof(edge_sample));
    memset(&edge_out, 0, sizeof(edge_out));
    edge_sample.sample_index = v0_final_edge_next_edge_sample_index();
    edge_sample.timestamp_raw = sample->tmst_raw;
    edge_sample.accel_x_raw = sample->ax;
    edge_sample.accel_y_raw = sample->ay;
    edge_sample.accel_z_raw = sample->az;
    edge_sample.gyro_x_raw = sample->gx;
    edge_sample.gyro_y_raw = sample->gy;
    edge_sample.gyro_z_raw = sample->gz;

    sample_start_us = (uint32_t)imu_bsp_local_timestamp_us();
    if (!zy100_edge_shadow_push_sample_ex(&edge_sample, &edge_out))
    {
        v0_final_edge_replay_note_edge_error();
        return false;
    }

    v0_final_edge_handle_lite_output_replay(&edge_out,
                                            edge_sample.sample_index,
                                            sample->tmst_raw);
    sample_elapsed_us =
        (uint32_t)((uint32_t)imu_bsp_local_timestamp_us() - sample_start_us);
    s_final_edge_phase1_stats.replay_total_us += sample_elapsed_us;
    s_final_edge_phase1_stats.replay_last_us = sample_elapsed_us;
    if (sample_elapsed_us > s_final_edge_phase1_stats.replay_sample_max_us)
    {
        s_final_edge_phase1_stats.replay_sample_max_us = sample_elapsed_us;
        s_final_edge_phase1_stats.replay_long_sample_bscan_id =
            s_final_edge_replay_bscan_id;
    }
    v0_final_edge_replay_note_sample_fed();
    return true;
}
#endif

static void v0_final_edge_feed_rt_marker(const uint8_t *packet,
                                         uint32_t sample_index,
                                         uint16_t tmst_raw,
                                         uint64_t sample_t_us)
{
    uint32_t marker_before;
    uint32_t marker_after;
    uint32_t rt_start_us;
    uint32_t rt_elapsed_us;
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    bool raw_gate_active = zy100_final_edge_raw_store_rt_gate_active();
#endif
    bool replay_gate_active = false;
    bool replay_backlog_gate_active = false;
    bool other_gate_active = false;

#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
    replay_gate_active = v0_final_edge_replay_busy();
    replay_backlog_gate_active = s_final_edge_replay_backlog_edge_only_active;
#endif
    other_gate_active = s_final_edge_rt_deferred_active
#if ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
                        || s_final_edge_rt_gated_for_b
#endif
                        || s_fe_final_drain_active
                        || s_v0_stop_requested
                        || (s_fe_capture_state != FE_CAPTURE_STATE_RUNNING)
                        ;

    if (other_gate_active ||
        replay_gate_active ||
        replay_backlog_gate_active
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
        || raw_gate_active
#endif
       )
    {
        s_final_edge_phase1_stats.rt_gated_count++;
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
        if (raw_gate_active)
        {
            s_final_edge_phase1_stats.rt_gated_raw_store_count++;
            zy100_final_edge_raw_store_note_rt_gate(zy100_os_time_ms());
        }
#endif
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
        if (replay_gate_active)
        {
            if (s_final_edge_replay_source == ZY100_FE_REPLAY_SRC_TRUEHIT)
            {
                s_final_edge_phase1_stats.rt_gated_replay_truehit_count++;
            }
            else
            {
                s_final_edge_phase1_stats.rt_gated_replay_count++;
            }
        }
        if (replay_backlog_gate_active)
        {
            s_final_edge_phase1_stats.rt_gated_replay_backlog_count++;
        }
#endif
        if (other_gate_active)
        {
            s_final_edge_phase1_stats.rt_gated_other_count++;
        }
        return;
    }

    marker_before = v0_final_edge_rt_marker_count();
    rt_start_us = (uint32_t)imu_bsp_local_timestamp_us();
    IMU_UNUSED(imu_rt_marker_feed_fifo_packet(packet, sample_index, sample_t_us));
    rt_elapsed_us = (uint32_t)((uint32_t)imu_bsp_local_timestamp_us() -
                               rt_start_us);
    s_final_edge_phase1_stats.rt_feed_count++;
    s_final_edge_phase1_stats.rt_time_sum_us += rt_elapsed_us;
    if (rt_elapsed_us > s_final_edge_phase1_stats.rt_time_max_us)
    {
        s_final_edge_phase1_stats.rt_time_max_us = rt_elapsed_us;
    }
    marker_after = v0_final_edge_rt_marker_count();

    if (marker_after != marker_before)
    {
        v0_final_edge_b_critical_set_pending();
        s_final_edge_phase1_stats.rt_trigger_count++;
        s_final_edge_last_rt_trigger_ms = zy100_os_time_ms();
        s_final_edge_phase1_stats.rt_deferred_count++;
        s_final_edge_rt_deferred_active = true;
        s_final_edge_rt_trigger_valid = true;
        s_final_edge_rt_trigger_sample_index = sample_index;
        s_final_edge_rt_trigger_tmst_raw = tmst_raw;
    }
}

static void v0_final_edge_log_config(void)
{
    ZY100_DIAG_LOG("[FE_PHASE1] flash=%u b_ois=%u rt3=0 raw=%u",
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
               1U,
#else
               0U,
#endif
#if ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
               1U,
#else
               0U,
#endif
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
               1U
#else
               0U
#endif
    );
    ZY100_DIAG_LOG("[FE_CFG] en=%u hz=%u th=%u max=%u edge=%u rt=%u",
               (uint32_t)ZY100_FINAL_EDGE_MODE_ENABLE,
               (uint32_t)ZY100_FINAL_EDGE_LIVE_UI_HZ,
               (uint32_t)ZY100_FIFO_DRAIN_THRESHOLD_BYTES,
               (uint32_t)ZY100_FIFO_DRAIN_MAX_BYTES,
               (uint32_t)ZY100_EDGE_SHADOW_ENABLE,
               (uint32_t)ZY100_RT_MARKER_ENABLE);
    ZY100_DIAG_LOG("[FE_BSCAN_MOD] owner=final_edge_bscan runner=1 bridge=aba_v0 od_diag=%u lf_diag=%u",
               (uint32_t)((ZY100_FINAL_EDGE_BSCAN_OD_DIAG_ENABLE != 0U) ?
                          1U : 0U),
               (uint32_t)((ZY100_FINAL_EDGE_LF_PRE_DIAG_ENABLE != 0U) ?
                          1U : 0U));
}

static void v0_final_edge_log_stats(const v0_fifo_stats_t *stats,
                                    uint32_t runtime_ms,
                                    bool final)
{
    zy100_final_edge_queue_stats_t queue_stats;
    uint32_t dt_min = s_final_edge_phase1_stats.dt_seen ?
                      s_final_edge_phase1_stats.dt_min_us : 0U;
    uint32_t dt_max = s_final_edge_phase1_stats.dt_seen ?
                      s_final_edge_phase1_stats.dt_max_us : 0U;
    uint32_t timer_total = s_final_edge_timer_total_due_count;
    uint32_t due_pending = s_final_edge_timer_due_count;
    uint32_t service_count = s_final_edge_phase1_stats.service_count;
    uint32_t service_gap_count = (service_count > 1U) ? (service_count - 1U) : 0U;
    uint32_t avg_gap_us = (service_gap_count != 0U) ?
                          (s_final_edge_phase1_stats.service_gap_sum_us /
                           service_gap_count) : 0U;
    uint32_t avg_pkts = (s_final_edge_phase1_stats.drain_count != 0U) ?
                        (s_final_edge_phase1_stats.drain_packet_sum /
                         s_final_edge_phase1_stats.drain_count) : 0U;
    uint32_t edge_avg_us = (s_final_edge_phase1_stats.edge_feed_count != 0U) ?
                           (s_final_edge_phase1_stats.edge_time_sum_us /
                            s_final_edge_phase1_stats.edge_feed_count) : 0U;
    uint32_t rt_avg_us = (s_final_edge_phase1_stats.rt_feed_count != 0U) ?
                         (s_final_edge_phase1_stats.rt_time_sum_us /
                          s_final_edge_phase1_stats.rt_feed_count) : 0U;
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
    uint32_t replay_ms = 0U;
    uint32_t replay_backlog_start = 0U;

    if (s_final_edge_replay_started_ms != 0U)
    {
        replay_ms = (uint32_t)(s_final_edge_replay_last_ms -
                               s_final_edge_replay_started_ms);
    }
    if (s_final_edge_replay_backlog_edge_only_active)
    {
        replay_backlog_start = s_final_edge_replay_backlog_start_count;
    }
#endif
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    zy100_fe_raw_store_stats_t raw_stats;

    memset(&raw_stats, 0, sizeof(raw_stats));
    zy100_final_edge_raw_store_get_stats(&raw_stats);
#endif
    zy100_fe_record_store_stats_t record_stats;

    memset(&record_stats, 0, sizeof(record_stats));
    zy100_final_edge_record_store_get_stats(&record_stats);

#if !ZY100_FINAL_EDGE_RUNTIME_STATS_LOG_ENABLE
    if (!final)
    {
        IMU_UNUSED(stats);
        IMU_UNUSED(runtime_ms);
        return;
    }
#endif

#if !ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
    IMU_UNUSED(final);
#endif
    memset(&queue_stats, 0, sizeof(queue_stats));
    zy100_final_edge_queue_get_stats(&queue_stats);

    ZY100_DIAG_LOG("[FE_RATE] dt_min=%u dt_max=%u bad=%u hdr_bad=%u",
               dt_min,
               dt_max,
               s_final_edge_phase1_stats.dt_bad_count,
               s_final_edge_phase1_stats.hdr_bad_count);
    ZY100_DIAG_LOG("[FE_FEED] pkt=%u edge=%u rt=%u raw_sup=%u",
               s_final_edge_phase1_stats.packet_count,
               s_final_edge_phase1_stats.edge_feed_count,
               s_final_edge_phase1_stats.rt_feed_count,
               s_final_edge_phase1_stats.edge_raw_sup_count);
    ZY100_DIAG_LOG("[FE_EDGE] summary=%u event=%u raw_req=%u raw_sup=%u",
               s_final_edge_phase1_stats.edge_summary_count,
               s_final_edge_phase1_stats.edge_event_count,
               s_final_edge_phase1_stats.edge_raw_req_count,
               s_final_edge_phase1_stats.edge_raw_sup_count);
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
    DBG_DIRECT("[FE_EDGE_TOTAL] live=%u nohit=%u hit=%u total=%u",
               s_final_edge_phase1_stats.edge_feed_count,
               s_final_edge_phase1_stats.replay_edge_feed,
               s_final_edge_phase1_stats.replay_truehit_edge_feed,
               s_final_edge_phase1_stats.edge_feed_count +
               v0_final_edge_replay_edge_total());
    ZY100_DIAG_LOG("[FE_REPLAY] nh_begin=%u h_begin=%u pending=%u active=%u",
               s_final_edge_phase1_stats.replay_begin_count,
               s_final_edge_phase1_stats.replay_truehit_begin_count,
               s_final_edge_replay_pending ? 1U : 0U,
               s_final_edge_replay_active ? 1U : 0U);
    DBG_DIRECT("[FE_REPLAY2] id=%u src=%u ds=%u idx=%u backlog=%u",
               s_final_edge_replay_bscan_id,
               (uint32_t)s_final_edge_replay_source,
               zy100_final_edge_bscan_get_ds_count(),
               s_final_edge_replay_index,
               replay_backlog_start);
    DBG_DIRECT("[FE_REPLAY3] ms=%u edge_err=%u fifo_high=%u fifo_hard=%u",
               replay_ms,
               s_final_edge_phase1_stats.replay_edge_error +
               s_final_edge_phase1_stats.replay_truehit_edge_error,
               s_final_edge_phase1_stats.replay_fifo_high_count,
               s_final_edge_phase1_stats.replay_fifo_hard_count);
    ZY100_DIAG_LOG("[FE_REPLAY4] sum=%u evt=%u raw_sup=%u backlog=%u",
               s_final_edge_phase1_stats.replay_summary_count,
               s_final_edge_phase1_stats.replay_event_count,
               s_final_edge_phase1_stats.replay_raw_sup_count,
               replay_backlog_start);
    DBG_DIRECT("[FE_REPLAY_SUM] nohit=%u nh_begin=%u nh_done=%u nh_fail=%u",
               s_final_edge_phase1_stats.replay_nohit_count,
               s_final_edge_phase1_stats.replay_begin_count,
               s_final_edge_phase1_stats.replay_done_count,
               s_final_edge_phase1_stats.replay_fail_count);
    DBG_DIRECT("[FE_REPLAY_SUM2] nh_exp=%u nh_fed=%u nh_edge=%u nh_err=%u",
               s_final_edge_phase1_stats.replay_samples_expected,
               s_final_edge_phase1_stats.replay_samples_fed,
               s_final_edge_phase1_stats.replay_edge_feed,
               s_final_edge_phase1_stats.replay_edge_error);
    DBG_DIRECT("[FE_REPLAY_HIT] hit=%u h_begin=%u h_done=%u h_fail=%u",
               s_final_edge_phase1_stats.replay_truehit_count,
               s_final_edge_phase1_stats.replay_truehit_begin_count,
               s_final_edge_phase1_stats.replay_truehit_done_count,
               s_final_edge_phase1_stats.replay_truehit_fail_count);
    DBG_DIRECT("[FE_REPLAY_HIT2] h_exp=%u h_fed=%u h_edge=%u h_err=%u",
               s_final_edge_phase1_stats.replay_truehit_samples_expected,
               s_final_edge_phase1_stats.replay_truehit_samples_fed,
               s_final_edge_phase1_stats.replay_truehit_edge_feed,
               s_final_edge_phase1_stats.replay_truehit_edge_error);
    ZY100_DIAG_LOG("[FE_REPLAY_SUM3] od_nh=%u od_hit=%u ds_bad=%u ds_over=%u",
               s_final_edge_phase1_stats.replay_od_warn_count,
               s_final_edge_truehit_od_warn_count,
               s_final_edge_phase1_stats.replay_ds_bad_count,
               s_final_edge_phase1_stats.replay_ds_overflow_count);
    ZY100_DIAG_LOG("[FE_REPLAY_TIME] batch=%u sample=%u total=%u long_id=%u",
               s_final_edge_phase1_stats.replay_batch_max_us,
               s_final_edge_phase1_stats.replay_sample_max_us,
               s_final_edge_phase1_stats.replay_total_us,
               s_final_edge_phase1_stats.replay_long_batch_bscan_id);
    ZY100_DIAG_LOG("[FE_REPLAY_FIFO] high=%u hard=%u shrink=%u max_count=%u",
               s_final_edge_phase1_stats.replay_fifo_high_count,
               s_final_edge_phase1_stats.replay_fifo_hard_count,
               s_final_edge_phase1_stats.replay_batch_shrink_count,
               s_final_edge_phase1_stats.replay_fifo_max_count);
    ZY100_DIAG_LOG("[FE_REPLAY_INTER] n=%u pkts=%u max_fifo=%u last_id=%u",
               s_final_edge_phase1_stats.replay_pressure_interleave_count,
               s_final_edge_phase1_stats.replay_pressure_interleave_packets,
               s_final_edge_phase1_stats.replay_pressure_interleave_max_fifo,
               s_final_edge_phase1_stats.replay_pressure_interleave_last_bscan_id);
#endif
#if ZY100_FINAL_EDGE_QUEUE_LOG_ENABLE
#if ZY100_FINAL_EDGE_QUEUE_ENABLE
    ZY100_DIAG_LOG("[FE_Q] sum_enq=%u sum_depth=%u sum_drop=%u sum_peak=%u",
               queue_stats.summary_enqueued,
               queue_stats.summary_current_depth,
               queue_stats.summary_dropped,
               queue_stats.summary_peak_depth);
    ZY100_DIAG_LOG("[FE_Q2] evt_enq=%u evt_depth=%u evt_drop=%u evt_peak=%u",
               queue_stats.event_enqueued,
               queue_stats.event_current_depth,
               queue_stats.event_dropped,
               queue_stats.event_peak_depth);
    ZY100_DIAG_LOG("[FE_Q3] sum_oversize=%u evt_oversize=%u raw_start_sup=%u q_bytes=%u",
               queue_stats.summary_drop_oversize,
               queue_stats.event_drop_oversize,
               queue_stats.raw_start_suppressed,
               queue_stats.queue_bytes_est);
    ZY100_DIAG_LOG("[FE_Q_MEM] sum_bytes=%u evt_bytes=%u total=%u",
               queue_stats.summary_queue_bytes,
               queue_stats.event_queue_bytes,
               queue_stats.queue_bytes_est);
#else
    ZY100_DIAG_LOG("[FE_Q] disabled=1 sum_enq=0 sum_depth=0 sum_drop=0 sum_peak=0");
    ZY100_DIAG_LOG("[FE_Q2] evt_enq=0 evt_depth=0 evt_drop=0 evt_peak=0");
    ZY100_DIAG_LOG("[FE_Q3] sum_oversize=0 evt_oversize=0 raw_start_sup=%u q_bytes=0",
               queue_stats.raw_start_suppressed);
    ZY100_DIAG_LOG("[FE_Q_MEM] sum_bytes=0 evt_bytes=0 total=0");
#endif
#endif
    DBG_DIRECT("[FE_RT] feed=%u trigger=%u deferred=%u gated=%u",
               s_final_edge_phase1_stats.rt_feed_count,
               s_final_edge_phase1_stats.rt_trigger_count,
               s_final_edge_phase1_stats.rt_deferred_count,
               s_final_edge_phase1_stats.rt_gated_count);
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
    ZY100_DIAG_LOG("[FE_RT_GATE] raw=%u replay_nohit=%u replay_hit=%u backlog=%u other=%u",
               s_final_edge_phase1_stats.rt_gated_raw_store_count,
               s_final_edge_phase1_stats.rt_gated_replay_count,
               s_final_edge_phase1_stats.rt_gated_replay_truehit_count,
               s_final_edge_phase1_stats.rt_gated_replay_backlog_count,
               s_final_edge_phase1_stats.rt_gated_other_count);
#endif
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    ZY100_DIAG_LOG("[FE_PHASE5] raw_flash=%u bucket=%u region=%u count=%u",
               (uint32_t)ZY100_FINAL_EDGE_RAW_STORE_ENABLE,
               (uint32_t)ZY100_FINAL_EDGE_RAW_BUCKET_BYTES,
               (uint32_t)ZY100_FINAL_EDGE_RAW_REGION_BYTES,
               raw_stats.bucket_count);
    DBG_DIRECT("[FE_RAW] begin=%u saved=%u fail=%u full=%u pending=%u",
               raw_stats.raw_begin,
               raw_stats.raw_saved,
               raw_stats.raw_failed,
               raw_stats.raw_full,
               raw_stats.pending);
#if ZY100_LOG_FE_DIAG_VERBOSE
    DBG_DIRECT("[FE_RAW2] pages=%u wip=%u err=%u verify=%u erase=%u",
               raw_stats.page_program,
               raw_stats.wip_busy,
               raw_stats.write_error,
               raw_stats.verify_error,
               raw_stats.prepare_erase_count);
    ZY100_DIAG_LOG("[FE_RAW3] last_id=%u b=%u sec=%u hf=%u lf=%u",
               raw_stats.last_raw_id,
               raw_stats.last_bscan_id,
               raw_stats.last_sections,
               raw_stats.last_hf_frames,
               raw_stats.last_lf_packets);
    ZY100_DIAG_LOG("[FE_RAW4] bytes=%u flags=%08x gate=%u gate_max=%u",
               raw_stats.last_payload_bytes,
               raw_stats.last_flags,
               raw_stats.rt_gate_count,
               raw_stats.rt_gate_max_ms);
    ZY100_DIAG_LOG("[FE_RAW5] prep=%u active=%u inflight=%u used=%u",
               raw_stats.prepared,
               raw_stats.active,
               raw_stats.program_in_flight,
               raw_stats.used_bytes);
    ZY100_DIAG_LOG("[FE_RAW6] rt_pump=%u fin_pump=%u all_pump=%u issue=%u",
               raw_stats.runtime_pump_max_us,
               raw_stats.final_pump_max_us,
               raw_stats.pump_max_us,
               raw_stats.page_issue_max_us);
    ZY100_DIAG_LOG("[FE_RAW7] verify=%u verify_n=%u",
               raw_stats.verify_max_us,
               raw_stats.verify_page_count);
#endif
    ZY100_DIAG_LOG("[FE_RAW_REC] n=%u avg_ms=%u max_ms=%u last_ms=%u",
               raw_stats.raw_record_save_count,
               (raw_stats.raw_record_save_count == 0U) ? 0U :
                   (raw_stats.raw_record_save_total_ms /
                    raw_stats.raw_record_save_count),
               raw_stats.raw_record_save_max_ms,
               raw_stats.raw_record_save_last_ms);
    ZY100_DIAG_LOG("[FE_STORE] sum_b=%u sum_s=%u sum_f=%u sum_full=%u",
               record_stats.summary_begin,
               record_stats.summary_saved,
               record_stats.summary_failed,
               record_stats.summary_full);
    ZY100_DIAG_LOG("[FE_STORE2] evt_b=%u evt_s=%u evt_f=%u evt_full=%u",
               record_stats.event_begin,
               record_stats.event_saved,
               record_stats.event_failed,
               record_stats.event_full);
#if ZY100_LOG_FE_DIAG_VERBOSE
    DBG_DIRECT("[FE_STORE3] pages=%u wip=%u err=%u verify=%u",
               record_stats.page_program,
               record_stats.wip_busy,
               record_stats.write_error,
               record_stats.verify_error);
    ZY100_DIAG_LOG("[FE_STORE4] sum_used=%u evt_used=%u pend=%u inflight=%u",
               record_stats.summary_used_bytes,
               record_stats.event_used_bytes,
               record_stats.pending,
               record_stats.program_in_flight);
    ZY100_DIAG_LOG("[FE_STORE5] last_t=%u seq=%u bytes=%u pages=%u",
               record_stats.last_type,
               record_stats.last_seq,
               record_stats.last_payload_bytes,
               record_stats.last_page_count);
    ZY100_DIAG_LOG("[FE_STORE6] pump=%u issue=%u verify=%u bblk=%u",
               record_stats.pump_max_us,
               record_stats.issue_max_us,
               record_stats.verify_max_us,
               record_stats.b_active_pump_blocked);
    ZY100_DIAG_LOG("[FE_STORE7] raw_blk=%u replay_blk=%u rec_wip=%u raw_wait=%u",
               record_stats.raw_busy_blocked,
               record_stats.replay_pump_blocked,
               record_stats.record_wip_busy,
               record_stats.raw_wait_record_wip_count);
    ZY100_DIAG_LOG("[FE_STORE8] fin_to=%u fin_left=%u",
               record_stats.record_final_pump_timeout,
               record_stats.record_final_pending_left);
#endif
    ZY100_DIAG_LOG("[FE_META] write=%u verify=%u bytes=%u flags=%08x",
               record_stats.meta_write_ok,
               record_stats.meta_verify_ok,
               record_stats.meta_bytes,
               record_stats.meta_flags);
    ZY100_DIAG_LOG("[FE_FLASH] page_program=%u erase=%u raw_saved=%u summary_flash=%u event_flash=%u",
               raw_stats.page_program + record_stats.page_program +
                   record_stats.meta_page_program,
               raw_stats.prepare_erase_count +
                   record_stats.prepare_erase_count,
               raw_stats.raw_saved,
               record_stats.summary_saved,
               record_stats.event_saved);
#else
    ZY100_DIAG_LOG("[FE_STORE] sum_b=%u sum_s=%u sum_f=%u sum_full=%u",
               record_stats.summary_begin,
               record_stats.summary_saved,
               record_stats.summary_failed,
               record_stats.summary_full);
    ZY100_DIAG_LOG("[FE_STORE2] evt_b=%u evt_s=%u evt_f=%u evt_full=%u",
               record_stats.event_begin,
               record_stats.event_saved,
               record_stats.event_failed,
               record_stats.event_full);
    ZY100_DIAG_LOG("[FE_META] write=%u verify=%u bytes=%u flags=%08x",
               record_stats.meta_write_ok,
               record_stats.meta_verify_ok,
               record_stats.meta_bytes,
               record_stats.meta_flags);
    ZY100_DIAG_LOG("[FE_FLASH] page_program=%u erase=%u raw_saved=%u summary_flash=%u event_flash=%u",
               s_final_edge_flash_stats.page_program +
                   record_stats.page_program + record_stats.meta_page_program,
               s_final_edge_flash_stats.erase +
                   record_stats.prepare_erase_count,
               s_final_edge_flash_stats.raw_saved,
               record_stats.summary_saved,
               record_stats.event_saved);
#endif
    if (final)
    {
        v0_final_edge_log_wm_summary();
    }
#if ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
#if ZY100_FINAL_EDGE_B_SCAN_LOG_ENABLE
    if (true)
#else
    if (final)
#endif
    {
    ZY100_DIAG_LOG("[FE_BCRIT] pending=%u active=%u clear=%u spi_retry=%u",
               s_final_edge_phase1_stats.b_critical_pending_count,
               s_final_edge_phase1_stats.b_critical_active_count,
               s_final_edge_phase1_stats.b_critical_clear_count,
               s_final_edge_phase1_stats.b_start_spi_retry_count);
    ZY100_DIAG_LOG("[FE_RT_EXP] all=%u spi=%u raw=%u replay=%u",
               s_final_edge_phase1_stats.rt_request_expired_count,
               s_final_edge_phase1_stats.b_start_spi_retry_count,
               s_final_edge_phase1_stats.b_start_raw_busy_count,
               s_final_edge_phase1_stats.b_start_replay_busy_count);
    ZY100_DIAG_LOG("[FE_BCRIT2] raw_busy=%u replay_busy=%u raw_blk=%u rec_blk=%u replay_blk=%u flash_blk=%u",
               s_final_edge_phase1_stats.b_start_raw_busy_count,
               s_final_edge_phase1_stats.b_start_replay_busy_count,
               s_final_edge_phase1_stats.b_critical_raw_blocked_count,
               s_final_edge_phase1_stats.b_critical_record_blocked_count,
               s_final_edge_phase1_stats.b_critical_replay_blocked_count,
               s_final_edge_phase1_stats.b_critical_flash_blocked_count);
    ZY100_DIAG_LOG("[FE_BCRIT_AB] offline_bypass_en=%u hit=%u post_hit=%u",
#if ZY100_FINAL_EDGE_OFFLINE_B_CRITICAL_BYPASS_VERIFY_ENABLE
               1U,
#else
               0U,
#endif
               s_final_edge_phase1_stats.b_critical_offline_bypass_count,
               s_final_edge_phase1_stats
                   .b_critical_offline_post_quiet_bypass_count);
    ZY100_DIAG_LOG("[FE_BCRIT3] b_pending_to_first_b_frame_read_done_us=%u max=%u first_b_frame_read_done_us=%u",
               s_final_edge_phase1_stats.b_pending_to_first_frame_last_us,
               s_final_edge_phase1_stats.b_pending_to_first_frame_max_us,
               s_final_edge_phase1_stats.b_first_frame_read_done_last_us);
    ZY100_DIAG_LOG("[FE_BSTART] raw_replay_busy=%u",
               s_final_edge_phase1_stats.b_scan_start_reject_raw_replay_count);
    DBG_DIRECT("[FE_BRESTORE_FIFO_SUM] en=%u flush=%u fail=%u st=%u count=%u",
#if ZY100_FINAL_EDGE_B_RESTORE_FIFO_FLUSH_ENABLE
               1U,
#else
               0U,
#endif
               s_final_edge_phase1_stats.b_restore_fifo_flush_count,
               s_final_edge_phase1_stats.b_restore_fifo_flush_fail_count,
               s_final_edge_phase1_stats.b_restore_fifo_flush_last_status,
               s_final_edge_phase1_stats.b_restore_fifo_flush_last_count);
    ZY100_DIAG_LOG("[FE_BSW] id=%u enter=%u owner_ok=%u restore=%u",
               s_final_edge_last_b_id,
               (s_final_edge_b_scan_count != 0U) ? 1U : 0U,
               s_final_edge_b_last_result.owner_ok,
               s_final_edge_resume_ok ? 1U : 0U);
    DBG_DIRECT("[FE_BCAP] frames=%u bytes=%u dur_us=%u err=%u",
               s_final_edge_b_last_result.frames,
               s_final_edge_b_last_result.bytes,
               s_final_edge_b_last_result.duration_acc_us,
               s_final_edge_b_last_result.err);
    DBG_DIRECT("[FE_BCAP2] over=%u drop=%u stale=%u read_err=%u dt_bad=%u",
               s_final_edge_b_last_result.over,
               s_final_edge_b_last_result.drop,
               s_final_edge_b_last_result.stale,
               s_final_edge_b_last_result.read_err,
               s_final_edge_b_last_result.dt_bad);
    DBG_DIRECT("[FE_BACQ] overflow=%u read_err=%u irq_bad=%u read_us_max=%u high=%u",
               s_final_edge_b_last_result.acq_overflow,
               s_final_edge_b_last_result.acq_read_err,
               s_final_edge_b_last_result.acq_irq_late_count,
               s_final_edge_b_last_result.acq_read_us_max,
               s_final_edge_b_last_result.acq_high_water);
    ZY100_DIAG_LOG("[FE_BACQ2] gap_min=%u gap_max=%u early=%u late_gap=%u",
               s_final_edge_b_last_result.acq_irq_gap_us_min,
               s_final_edge_b_last_result.acq_irq_gap_us_max,
               s_final_edge_b_last_result.acq_irq_early_count,
               s_final_edge_b_last_result.acq_irq_late_gap_count);
    ZY100_DIAG_LOG("[FE_BACQ3] first_irq=%u first_read=%u timer_irq=%u wr=%u rd=%u skip_stop=%u",
               s_final_edge_b_last_result.acq_first_irq_us,
               s_final_edge_b_last_result.acq_first_read_done_us,
               s_final_edge_b_last_result.timer_irq_total,
               s_final_edge_b_last_result.acq_wr_seq,
               s_final_edge_b_last_result.acq_rd_seq,
               s_final_edge_b_last_result.acq_skipped_after_stop);
    DBG_DIRECT("[FE_BWARN] dt_bad=%u stale=%u post_trunc=%u warn_only=0",
               s_final_edge_b_last_result.dt_bad,
               s_final_edge_b_last_result.stale,
               s_final_edge_b_last_result.post_truncated);
    if (final)
    {
        DBG_DIRECT("[FE_BWARN2_SUM] n=%u od_or=%08x id=%u r=%08x run=%u frames=%u err=%u over=%u drop=%u",
                   s_final_edge_nohit_od_warn_count,
                   s_final_edge_nohit_od_warn_or,
                   s_final_edge_nohit_od_warn_last_id,
                   s_final_edge_nohit_od_warn_last_reason,
                   s_final_edge_nohit_od_warn_last_run,
                   s_final_edge_nohit_od_warn_last_frames,
                   s_final_edge_nohit_od_warn_last_err,
                   s_final_edge_nohit_od_warn_last_over,
                   s_final_edge_nohit_od_warn_last_drop);
        DBG_DIRECT("[FE_BWARN3_SUM] n=%u od_or=%08x id=%u r=%08x run=%u frames=%u over=%u drop=%u",
                   s_final_edge_truehit_od_warn_count,
                   s_final_edge_truehit_od_warn_or,
                   s_final_edge_truehit_od_warn_last_id,
                   s_final_edge_truehit_od_warn_last_reason,
                   s_final_edge_truehit_od_warn_last_run,
                   s_final_edge_truehit_od_warn_last_frames,
                   s_final_edge_truehit_od_warn_last_over,
                   s_final_edge_truehit_od_warn_last_drop);
#if ZY100_FINAL_EDGE_BSCAN_OD_DIAG_ENABLE
        {
            uint32_t od_pct =
                (s_final_edge_b_scan_count == 0U) ? 0U :
                ((s_final_edge_od_diag_stats.od_scan_count * 100U) /
                 s_final_edge_b_scan_count);

            /* od is B-scan count with OD; e/m/t/p are OD event counts. */
            ZY100_DIAG_LOG("[FE_OD_SUM] scan=%u od=%u hit_od=%u nh_od=%u od_pct=%u",
                       s_final_edge_b_scan_count,
                       s_final_edge_od_diag_stats.od_scan_count,
                       s_final_edge_od_diag_stats.od_hit_scan_count,
                       s_final_edge_od_diag_stats.od_nohit_scan_count,
                       od_pct);
            ZY100_DIAG_LOG("[FE_OD_STAGE] e=%u m=%u t=%u p=%u maxp=%u",
                       s_final_edge_od_diag_stats.od_early_events,
                       s_final_edge_od_diag_stats.od_mid_events,
                       s_final_edge_od_diag_stats.od_tail_events,
                       s_final_edge_od_diag_stats.od_post_events,
                       s_final_edge_od_diag_stats.od_max_pending);
            DBG_DIRECT("[FE_OD_TMST_SUM] zero=%u dbl=%u bad=%u dmin=%u dmax=%u",
                       s_final_edge_od_diag_stats.tmst_delta_zero_count,
                       s_final_edge_od_diag_stats.tmst_delta_double_count,
                       s_final_edge_od_diag_stats.tmst_delta_bad_count,
                       v0_final_edge_od_diag_delta_min(),
                       v0_final_edge_od_diag_delta_max());
            ZY100_DIAG_LOG("[FE_OD_READ_SUM] read_us_max=%u scan=%u seq=%u",
                       s_final_edge_od_diag_stats.read_us_max,
                       s_final_edge_od_diag_stats.read_us_max_scan_id,
                       s_final_edge_od_diag_stats.read_us_max_seq);
        }
#endif
    }
    ZY100_DIAG_LOG("[FE_HIT] hit=%u idx=%u score=%u th=%u tmst=%u",
               s_final_edge_b_last_result.hit_found,
               s_final_edge_b_last_result.hit_index,
               s_final_edge_b_last_result.hit_score20,
               (uint32_t)ZY100_FINAL_EDGE_HIT_ACC20_DELTA_L1_THRESHOLD,
               s_final_edge_b_last_result.hit_tmst_raw);
    ZY100_DIAG_LOG("[FE_HIT2] peak_idx=%u peak_score=%u first_tmst=%u last_tmst=%u",
               s_final_edge_b_last_result.peak20_index,
               s_final_edge_b_last_result.peak20_score,
               (uint32_t)s_final_edge_b_last_result.first_tmst_raw,
               (uint32_t)s_final_edge_b_last_result.last_tmst_raw);
    ZY100_DIAG_LOG("[FE_HIT_SCALE] peak16=%u peak20=%u has20=%u th16=%u th20=%u",
               s_final_edge_b_last_result.peak16_score,
               s_final_edge_b_last_result.peak20_score,
               s_final_edge_b_last_result.use20,
               (uint32_t)ZY100_FINAL_EDGE_HIT_ACC16_DELTA_L1_THRESHOLD,
               (uint32_t)ZY100_FINAL_EDGE_HIT_ACC20_DELTA_L1_THRESHOLD);
    DBG_DIRECT("[FE_HIT_DEC] ois20_err=%u use20=%u",
               s_final_edge_b_last_result.ois20_decode_err,
               s_final_edge_b_last_result.use20);
    ZY100_DIAG_LOG("[FE_HIT_OFFICIAL] th=%u hit_n=%u nohit_n=%u",
               (uint32_t)ZY100_FINAL_EDGE_HIT_ACC20_DELTA_L1_THRESHOLD,
               s_final_edge_official_hit_count,
               s_final_edge_official_nohit_count);
#if ZY100_FINAL_EDGE_HIT20_CALIB_ENABLE
    ZY100_DIAG_LOG("[FE_HIT_TH] th0=%u th1=%u th2=%u th3=%u th4=%u",
               s_final_edge_hit20_calib_thresholds[0],
               s_final_edge_hit20_calib_thresholds[1],
               s_final_edge_hit20_calib_thresholds[2],
               s_final_edge_hit20_calib_thresholds[3],
               s_final_edge_hit20_calib_thresholds[4]);
    ZY100_DIAG_LOG("[FE_HIT_TH2] th5=%u th6=%u",
               s_final_edge_hit20_calib_thresholds[5],
               s_final_edge_hit20_calib_thresholds[6]);
    /* FE_HIT_CAL_LAST* reports the last completed B-scan only. */
    ZY100_DIAG_LOG("[FE_HIT_CAL_LAST0] th=%u idx=%d score=%u",
               s_final_edge_hit20_calib_thresholds[0],
               s_final_edge_b_last_result.hit20_calib_first_cross_idx[0],
               s_final_edge_b_last_result.hit20_calib_first_cross_score[0]);
    ZY100_DIAG_LOG("[FE_HIT_CAL_LAST1] th=%u idx=%d score=%u",
               s_final_edge_hit20_calib_thresholds[1],
               s_final_edge_b_last_result.hit20_calib_first_cross_idx[1],
               s_final_edge_b_last_result.hit20_calib_first_cross_score[1]);
    ZY100_DIAG_LOG("[FE_HIT_CAL_LAST2] th=%u idx=%d score=%u",
               s_final_edge_hit20_calib_thresholds[2],
               s_final_edge_b_last_result.hit20_calib_first_cross_idx[2],
               s_final_edge_b_last_result.hit20_calib_first_cross_score[2]);
    ZY100_DIAG_LOG("[FE_HIT_CAL_LAST3] th=%u idx=%d score=%u",
               s_final_edge_hit20_calib_thresholds[3],
               s_final_edge_b_last_result.hit20_calib_first_cross_idx[3],
               s_final_edge_b_last_result.hit20_calib_first_cross_score[3]);
    ZY100_DIAG_LOG("[FE_HIT_CAL_LAST4] th=%u idx=%d score=%u",
               s_final_edge_hit20_calib_thresholds[4],
               s_final_edge_b_last_result.hit20_calib_first_cross_idx[4],
               s_final_edge_b_last_result.hit20_calib_first_cross_score[4]);
    ZY100_DIAG_LOG("[FE_HIT_CAL_LAST5] th=%u idx=%d score=%u",
               s_final_edge_hit20_calib_thresholds[5],
               s_final_edge_b_last_result.hit20_calib_first_cross_idx[5],
               s_final_edge_b_last_result.hit20_calib_first_cross_score[5]);
    ZY100_DIAG_LOG("[FE_HIT_CAL_LAST6] th=%u idx=%d score=%u",
               s_final_edge_hit20_calib_thresholds[6],
               s_final_edge_b_last_result.hit20_calib_first_cross_idx[6],
               s_final_edge_b_last_result.hit20_calib_first_cross_score[6]);
    DBG_DIRECT("[FE_HIT_DIST] n=%u min=%u avg=%u max=%u",
               s_final_edge_hit20_calib_stats.peak20_count,
               v0_final_edge_hit20_calib_print_min(),
               v0_final_edge_hit20_calib_avg(),
               v0_final_edge_hit20_calib_print_max());
    ZY100_DIAG_LOG("[FE_HIT_DIST2] ov0=%u ov1=%u ov2=%u ov3=%u ov4=%u",
               s_final_edge_hit20_calib_stats.peak20_over_count[0],
               s_final_edge_hit20_calib_stats.peak20_over_count[1],
               s_final_edge_hit20_calib_stats.peak20_over_count[2],
               s_final_edge_hit20_calib_stats.peak20_over_count[3],
               s_final_edge_hit20_calib_stats.peak20_over_count[4]);
    ZY100_DIAG_LOG("[FE_HIT_DIST2B] ov5=%u ov6=%u",
               s_final_edge_hit20_calib_stats.peak20_over_count[5],
               s_final_edge_hit20_calib_stats.peak20_over_count[6]);
    ZY100_DIAG_LOG("[FE_HIT_CROSS] c0=%u c1=%u c2=%u c3=%u c4=%u",
               s_final_edge_hit20_calib_stats.cross_count[0],
               s_final_edge_hit20_calib_stats.cross_count[1],
               s_final_edge_hit20_calib_stats.cross_count[2],
               s_final_edge_hit20_calib_stats.cross_count[3],
               s_final_edge_hit20_calib_stats.cross_count[4]);
    ZY100_DIAG_LOG("[FE_HIT_CROSS2] c5=%u c6=%u",
               s_final_edge_hit20_calib_stats.cross_count[5],
               s_final_edge_hit20_calib_stats.cross_count[6]);
    ZY100_DIAG_LOG("[FE_HIT_DIST3] hit_n=%u nohit_n=%u",
               s_final_edge_hit20_calib_stats.hit_scan_count,
               s_final_edge_hit20_calib_stats.nohit_scan_count);
#endif
#if ZY100_FINAL_EDGE_PHASE4_HIT_WINDOW_ENABLE
    if (final)
    {
        zy100_final_edge_lf_pre_stats_t lf_pre_stats;
#if ZY100_LOG_FE_DIAG_VERBOSE
        uint32_t seen;
        uint32_t hwin;
        uint32_t lf_ok;
        uint32_t trunc_ok;
        uint32_t flash_ok;
        uint32_t inconclusive;
#endif

        v0_final_edge_run_hwin_self_test();
        memset(&lf_pre_stats, 0, sizeof(lf_pre_stats));
        zy100_final_edge_lf_pre_ring_get_stats(&lf_pre_stats);

        ZY100_DIAG_LOG("[FE_HWIN] valid=%u complete=%u hit=%u start=%u end=%u hf=%u",
                   s_final_edge_last_hit_window_view.valid,
                   s_final_edge_last_hit_window_view.complete,
                   s_final_edge_b_last_result.hit_found,
                   s_final_edge_last_hit_window_view.start_seq,
                   s_final_edge_last_hit_window_view.end_seq,
                   s_final_edge_last_hit_window_view.hf_frames);
        ZY100_DIAG_LOG("[FE_HWIN2] pre=%u post=%u ring=%u wrap=%u lf_need=%u lf_pkt=%u",
                   s_final_edge_last_hit_window_view.hf_pre_frames,
                   s_final_edge_last_hit_window_view.hf_post_frames,
                   s_final_edge_last_hit_window_view.ring_start_index,
                   s_final_edge_last_hit_window_view.wraps,
                   s_final_edge_last_hit_window_view.lf_pre_frames_needed,
                   s_final_edge_last_hit_window_view.lf_pre_packets_needed);
        ZY100_DIAG_LOG("[FE_HWIN3] first=%u hit_t=%u last=%u xor=%08x",
                   s_final_edge_last_hit_window_view.first_tmst_raw,
                   s_final_edge_last_hit_window_view.hit_tmst_raw,
                   s_final_edge_last_hit_window_view.last_tmst_raw,
                   s_final_edge_last_hit_window_view.window_xor);
        DBG_DIRECT("[FE_HWIN_SUM] hit=%u ok=%u fail=%u early=%u post_trunc=%u",
                   s_final_edge_phase4_stats.hit_window_seen,
                   s_final_edge_phase4_stats.hit_window_ok,
                   s_final_edge_phase4_stats.hit_window_fail,
                   s_final_edge_phase4_stats.early_hit_count,
                   s_final_edge_phase4_stats.post_trunc_count);
        ZY100_DIAG_LOG("[FE_LF_PRE] valid=%u need=%u have=%u miss=%u wrap=%u",
                   s_final_edge_last_lf_pre_query.valid,
                   s_final_edge_last_hit_window_view.lf_pre_packets_needed,
                   s_final_edge_last_lf_pre_query.packets_available,
                   lf_pre_stats.miss_count,
                   lf_pre_stats.wrap_count);
        ZY100_DIAG_LOG("[FE_LF_PRE2] oldest=%u newest=%u tmst0=%u tmst1=%u",
                   lf_pre_stats.oldest_sample_index,
                   lf_pre_stats.newest_sample_index,
                   lf_pre_stats.oldest_tmst_raw,
                   lf_pre_stats.newest_tmst_raw);
#if ZY100_FINAL_EDGE_LF_PRE_DIAG_ENABLE
        {
            zy100_final_edge_lf_pre_diag_t lf_pre_diag;

            memset(&lf_pre_diag, 0, sizeof(lf_pre_diag));
            zy100_final_edge_lf_pre_ring_diag_get(&lf_pre_diag);
            ZY100_DIAG_LOG("[FE_LF_PRE_DIAG] sec=%u gap_sec=%u gap=%u max_delta=%u raw=%u idx=%u",
                       lf_pre_diag.saved_sections,
                       lf_pre_diag.gap_sections,
                       lf_pre_diag.gap_count,
                       lf_pre_diag.gap_max_delta_us,
                       lf_pre_diag.first_bad_raw_id,
                       lf_pre_diag.first_bad_packet_index);
        }
#endif

#if ZY100_LOG_FE_DIAG_VERBOSE
        seen = (s_final_edge_phase4_stats.hit_window_seen != 0U) ? 1U : 0U;
        hwin = (seen &&
                (s_final_edge_phase4_stats.hit_window_ok ==
                 s_final_edge_official_hit_count) &&
                (s_final_edge_phase4_stats.hit_window_fail == 0U)) ? 1U : 0U;
        lf_ok = (s_final_edge_phase4_stats.early_lf_fail == 0U) ? 1U : 0U;
        trunc_ok =
            (s_final_edge_phase4_stats.post_trunc_count == 0U) ? 1U : 0U;
        flash_ok = v0_final_edge_flash_stats_all_zero() ? 1U : 0U;
        inconclusive = (seen == 0U) ? 1U : 0U;
        ZY100_DIAG_LOG("[FE_PASS4] seen=%u hwin=%u lf=%u trunc=%u flash=%u inc=%u self=%u",
                   seen,
                   hwin,
                   lf_ok,
                   trunc_ok,
                   flash_ok,
                   inconclusive,
                   (uint32_t)s_final_edge_phase4_stats.self_pass);
        DBG_DIRECT("[FE_PASS4B] hit=%u ok=%u fail=%u official=%u",
                   s_final_edge_phase4_stats.hit_window_seen,
                   s_final_edge_phase4_stats.hit_window_ok,
                   s_final_edge_phase4_stats.hit_window_fail,
                   s_final_edge_official_hit_count);
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
        {
            uint32_t idle =
                ((raw_stats.pending == 0U) &&
                 (raw_stats.program_in_flight == 0U)) ? 1U : 0U;
            uint32_t raw_err =
                ((raw_stats.raw_failed != 0U) ||
                 (raw_stats.raw_full != 0U) ||
                 (raw_stats.raw_lf_fail != 0U) ||
                 (raw_stats.raw_hwin_fail != 0U) ||
                 (raw_stats.write_error != 0U) ||
                 (raw_stats.verify_error != 0U) ||
                 (raw_stats.pending != 0U) ||
                 (raw_stats.program_in_flight != 0U) ||
                 (raw_stats.b_active_pump_blocked != 0U)
#if !ZY100_FINAL_EDGE_PHASE7_EDGE_FLASH_ENABLE
                 ||
                 (s_final_edge_flash_stats.summary_flash != 0U) ||
                 (s_final_edge_flash_stats.event_flash != 0U)
#endif
                 ) ? 1U : 0U;
            uint32_t raw_seen =
                (s_final_edge_official_hit_count != 0U) ? 1U : 0U;
            uint32_t raw_ok =
                (raw_seen &&
                 (raw_stats.raw_saved == s_final_edge_official_hit_count) &&
                 (raw_stats.raw_begin == s_final_edge_official_hit_count) &&
                 (raw_err == 0U)) ? 1U : 0U;
            uint32_t raw_inc = (raw_seen == 0U) ? 1U : 0U;

            DBG_DIRECT("[FE_PASS5] seen=%u raw=%u saved=%u official=%u err=%u inc=%u",
                       raw_seen,
                       raw_ok,
                       raw_stats.raw_saved,
                       s_final_edge_official_hit_count,
                       raw_err,
                       raw_inc);
            ZY100_DIAG_LOG("[FE_PASS5B] idle=%u full=%u lf=%u hwin=%u bflash=%u",
                       idle,
                       (raw_stats.raw_full == 0U) ? 1U : 0U,
                       (raw_stats.raw_lf_fail == 0U) ? 1U : 0U,
                       (raw_stats.raw_hwin_fail == 0U) ? 1U : 0U,
                       (raw_stats.b_active_pump_blocked == 0U) ? 1U : 0U);
        }
#endif
#endif
    }
#else
    ZY100_DIAG_LOG("[FE_HWIN] valid=%u target=%u hf=%u lf_need=%u lf_pkt=%u",
               s_final_edge_b_last_result.hit_window_valid,
               s_final_edge_b_last_result.target_window_frames,
               s_final_edge_b_last_result.hf_frame_count,
               s_final_edge_b_last_result.lf_pre_needed_frames,
               s_final_edge_b_last_result.lf_pre_packets);
    ZY100_DIAG_LOG("[FE_HWIN2] start=%u end=%u ring=%u ring_valid=%u",
               s_final_edge_b_last_result.hf_start_seq,
               s_final_edge_b_last_result.hf_end_seq,
               s_final_edge_b_last_result.ring_start_idx,
               s_final_edge_b_last_result.ring_valid_frames);
#endif
    ZY100_DIAG_LOG("[FE_RESUME] ok=%u hz=%u dt_min=%u dt_max=%u",
               s_final_edge_resume_ok ? 1U : 0U,
               (uint32_t)ZY100_FINAL_EDGE_LIVE_UI_HZ,
               dt_min,
               dt_max);
    ZY100_DIAG_LOG("[FE_RESTORE_FALLBACK] count=%u status=%u",
               s_final_edge_restore_fallback_count,
               s_final_edge_restore_fallback_status);
    }
#endif
    ZY100_DIAG_LOG("[FE_WAKE] mode=%u int=%u timer=%u due=%u due_max=%u",
               (uint32_t)ZY100_FINAL_EDGE_LIVE_WAKE_MODE,
               s_v0_irq_count,
               timer_total,
               due_pending,
               s_final_edge_phase1_stats.timer_due_max);
    DBG_DIRECT("[FE_WAKE2] coalesced=%u capped=%u empty=%u notify_fail=%u",
               s_final_edge_phase1_stats.timer_due_coalesced_count,
               s_final_edge_phase1_stats.timer_due_capped_count,
               s_final_edge_phase1_stats.timer_empty_count,
               s_final_edge_timer_notify_fail_count);
    DBG_DIRECT("[FE_SCHED] service_hz=%u avg_gap_us=%u max_gap_us=%u",
               v0_final_edge_rate_per_s(service_count, runtime_ms),
               avg_gap_us,
               s_final_edge_phase1_stats.runtime_service_gap_max_us);
    ZY100_DIAG_LOG("[FE_SCHED3] runtime_gap=%u final_gap=%u",
               s_final_edge_phase1_stats.runtime_service_gap_max_us,
               s_final_edge_phase1_stats.final_service_gap_max_us);
    DBG_DIRECT("[FE_SCHED2] timer_hz=%u packet_hz=%u due_consumed=%u",
               v0_final_edge_rate_per_s(timer_total, runtime_ms),
               v0_final_edge_rate_per_s(s_final_edge_phase1_stats.packet_count,
                                        runtime_ms),
               s_final_edge_phase1_stats.timer_due_consumed_count);
    ZY100_DIAG_LOG("[FE_DRAIN] n=%u avg_pkts=%u max_pkts=%u d16=%u d32=%u",
               s_final_edge_phase1_stats.drain_count,
               avg_pkts,
               s_final_edge_phase1_stats.drain_packet_max,
               s_final_edge_phase1_stats.drain_16_count,
               s_final_edge_phase1_stats.drain_32_count);
    ZY100_DIAG_LOG("[FE_DRAIN2] d64=%u d128=%u d256=%u d512=%u",
               s_final_edge_phase1_stats.drain_64_count,
               s_final_edge_phase1_stats.drain_128_count,
               s_final_edge_phase1_stats.drain_256_count,
               s_final_edge_phase1_stats.drain_512_count);
    ZY100_DIAG_LOG("[FE_FIFO] max_count=%u full=%u lost=%u emergency=%u",
               (stats != NULL && stats->max_fifo_count > s_final_edge_phase1_stats.fifo_max_count) ?
                   stats->max_fifo_count : s_final_edge_phase1_stats.fifo_max_count,
               (stats != NULL) ? stats->fifo_full_count : 0U,
               (stats != NULL) ? (uint32_t)stats->fifo_lost_pkt_count : 0U,
               s_final_edge_phase1_stats.fifo_emergency_count);
    ZY100_DIAG_LOG("[FE_TIME] edge_avg_us=%u edge_max_us=%u rt_avg_us=%u rt_max_us=%u",
               edge_avg_us,
               s_final_edge_phase1_stats.edge_time_max_us,
               rt_avg_us,
               s_final_edge_phase1_stats.rt_time_max_us);
}

static void v0_final_edge_log_ois_summary(void)
{
    zy100_fe_ois_activity_stats_t stats;

    if (v0_online_quiet_logs())
    {
        return;
    }
    if (!imu_fifo_drain_test_final_edge_get_ois_activity_stats(&stats))
    {
        return;
    }

    DBG_DIRECT("[FE_OIS_SUM] trigger=%lu scan=%lu hit=%lu nohit=%lu b_fail=%lu raw=%lu last_ms=%lu",
               (unsigned long)stats.rt_trigger_count,
               (unsigned long)stats.b_scan_count,
               (unsigned long)stats.official_hit_count,
               (unsigned long)stats.official_nohit_count,
               (unsigned long)stats.b_scan_fail_count,
               (unsigned long)stats.raw_saved_count,
               (unsigned long)stats.last_rt_trigger_ms);
}

static void v0_final_edge_end_summary_log_delay(void)
{
#if (ZY100_FINAL_EDGE_END_SUMMARY_LOG_DELAY_US > 0U)
    imu_bsp_delay_us((uint32_t)ZY100_FINAL_EDGE_END_SUMMARY_LOG_DELAY_US);
#endif
}

static void v0_final_edge_log_bscan_end_summary(void)
{
#if ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
    uint32_t raw_busy = 0U;
    uint32_t raw_saved = 0U;
    uint32_t replay_fail = 0U;
    uint32_t replay_hit_fail = 0U;
    zy100_final_edge_bscan_replay_raw_stats_t replay_raw_stats;

    if (v0_online_quiet_logs())
    {
        return;
    }
    memset(&replay_raw_stats, 0, sizeof(replay_raw_stats));
    zy100_final_edge_bscan_get_replay_raw_stats(&replay_raw_stats);

#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    {
        zy100_fe_raw_store_stats_t raw_stats;

        memset(&raw_stats, 0, sizeof(raw_stats));
        zy100_final_edge_raw_store_get_stats(&raw_stats);
        raw_busy = zy100_final_edge_raw_store_has_pending_work() ? 1U : 0U;
        raw_saved = raw_stats.raw_saved;
    }
#endif
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
    replay_fail = s_final_edge_phase1_stats.replay_fail_count;
    replay_hit_fail = s_final_edge_phase1_stats.replay_truehit_fail_count;
#endif

    DBG_DIRECT("[FE_BSUM1] scan=%u fail=%u hard=%u reason_or=%08x last_id=%u last_r=%08x",
               s_final_edge_b_scan_count,
               s_final_edge_b_scan_fail_count,
               s_final_edge_b_fail_hard_count,
               s_final_edge_b_fail_reason_or,
               s_final_edge_b_fail_last_id,
               s_final_edge_b_fail_last_reason);
    DBG_DIRECT("[FE_BSUM2] hit=%u nohit=%u frame=%u hwin=%u trunc=%u raw=%u replay_fail=%u replay_hit_fail=%u",
               s_final_edge_official_hit_count,
               s_final_edge_official_nohit_count,
               s_final_edge_b_fail_frame_count,
               s_final_edge_b_fail_hwin_count,
               s_final_edge_b_fail_trunc_count,
               raw_saved,
               replay_fail,
               replay_hit_fail);
    ZY100_DIAG_LOG("[FE_BSUM3] last_hit=%u last_frames=%u last_exit=%u last_seq=%u elapsed_ms=%u",
               s_final_edge_b_fail_last_hit,
               s_final_edge_b_fail_last_frames,
               s_final_edge_b_fail_last_exit_reason,
               s_final_edge_b_fail_last_seq,
               s_final_edge_b_fail_last_elapsed_ms);
    ZY100_DIAG_LOG("[FE_BNOHIT_SUM] count=%u min=%u max=%u last=%u sum_short=%u last_exit=%u",
               s_final_edge_b_fail_frame_count,
               s_final_edge_b_nohit_frame_min,
               s_final_edge_b_nohit_frame_max,
               s_final_edge_b_nohit_frame_last,
               s_final_edge_b_nohit_frame_sum_short,
               s_final_edge_b_nohit_frame_last_exit);
    DBG_DIRECT("[FE_BBEGIN_SUM] fail=%u stage=%u status=%u retry=%u owner=%u cleanup=%u",
               s_final_edge_b_begin_fail_count,
               s_final_edge_b_begin_fail_last_stage,
               s_final_edge_b_begin_fail_last_status,
               s_final_edge_b_begin_fail_last_retry,
               s_final_edge_b_begin_fail_last_owner,
               s_final_edge_b_begin_fail_last_cleanup);
    ZY100_DIAG_LOG("[FE_BREPLAY_RAW] en=%u decim=%u count=%u over=%u first=%u last=%u bytes=%u",
               replay_raw_stats.enabled,
               replay_raw_stats.decim,
               replay_raw_stats.count,
               replay_raw_stats.overflow,
               replay_raw_stats.first_frame_index,
               replay_raw_stats.last_frame_index,
               replay_raw_stats.bytes);
    v0_final_edge_end_summary_log_delay();

    DBG_DIRECT("[FE_BACQ_SUM] high=%u overflow=%u read_err=%u late=%u early=%u late_gap=%u gap_min=%u gap_max=%u read_us=%u skip_stop=%u",
               s_final_edge_b_last_result.acq_high_water,
               s_final_edge_b_last_result.acq_overflow,
               s_final_edge_b_last_result.acq_read_err,
               s_final_edge_b_last_result.acq_irq_late_count,
               s_final_edge_b_last_result.acq_irq_early_count,
               s_final_edge_b_last_result.acq_irq_late_gap_count,
               s_final_edge_b_last_result.acq_irq_gap_us_min,
               s_final_edge_b_last_result.acq_irq_gap_us_max,
               s_final_edge_b_last_result.acq_read_us_max,
               s_final_edge_b_last_result.acq_skipped_after_stop);
    ZY100_DIAG_LOG("[FE_BACQ_SEQ] irq=%u wr=%u rd=%u first_irq=%u first_read=%u",
               s_final_edge_b_last_result.timer_irq_total,
               s_final_edge_b_last_result.acq_wr_seq,
               s_final_edge_b_last_result.acq_rd_seq,
               s_final_edge_b_last_result.acq_first_irq_us,
               s_final_edge_b_last_result.acq_first_read_done_us);
    ZY100_DIAG_LOG("[FE_BTMST_GATE1] en=%u poll_hz=%u irq=%u pub=%u dup=%u",
               s_final_edge_b_last_result.acq_tmst_gate_enabled,
               s_final_edge_b_last_result.acq_tmst_poll_hz,
               s_final_edge_b_last_result.timer_irq_total,
               s_final_edge_b_last_result.acq_tmst_new_count,
               s_final_edge_b_last_result.acq_tmst_dup_drop);
    DBG_DIRECT("[FE_BTMST_GATE2] mismatch=%u tmst_read_err=%u raw_read_err=%u tmst_us=%u raw_us=%u",
               s_final_edge_b_last_result.acq_tmst_raw_mismatch,
               s_final_edge_b_last_result.acq_tmst_read_err,
               s_final_edge_b_last_result.acq_raw_read_err,
               s_final_edge_b_last_result.acq_tmst_read_us_max,
               s_final_edge_b_last_result.acq_raw_read_us_max);
    DBG_DIRECT("[FE_BRAW_PHASE_SUM] en=%u period=%u irq=%u pub=%u dup=%u dt_bad=%u dmin=%u dmax=%u raw_us=%u",
#if ZY100_FINAL_EDGE_B_ACQ_RAW_PHASE_LOCK_ENABLE
               1U,
#else
               0U,
#endif
               (uint32_t)ZY100_FINAL_EDGE_B_ACQ_TIMER_PERIOD_TICKS,
               s_final_edge_b_last_result.timer_irq_total,
               s_final_edge_b_last_result.acq_tmst_new_count,
               s_final_edge_b_last_result.acq_tmst_dup_drop,
               s_final_edge_b_last_result.acq_raw_phase_dt_bad,
               s_final_edge_b_last_result.acq_raw_phase_delta_min,
               s_final_edge_b_last_result.acq_raw_phase_delta_max,
               s_final_edge_b_last_result.acq_raw_read_us_max);
    ZY100_DIAG_LOG("[FE_BMISMATCH_SUM] count=%u hit=%u nohit=%u last_id=%u last_seq=%u probe=%u raw=%u",
               s_final_edge_b_mismatch_count,
               s_final_edge_b_mismatch_hit_count,
               s_final_edge_b_mismatch_nohit_count,
               s_final_edge_b_mismatch_last_id,
               s_final_edge_b_mismatch_last_seq,
               s_final_edge_b_mismatch_last_probe,
               s_final_edge_b_mismatch_last_raw);
    ZY100_DIAG_LOG("[FE_BMISMATCH_SOFT] drop=%u limit=%u last_id=%u probe=%u raw=%u",
               s_final_edge_b_mismatch_soft_drop_count,
               s_final_edge_b_mismatch_soft_limit,
               s_final_edge_b_mismatch_soft_last_id,
               s_final_edge_b_mismatch_soft_last_probe,
               s_final_edge_b_mismatch_soft_last_raw);
    ZY100_DIAG_LOG("[FE_BPHASE_SUM] status=%u probes=%u seed=%u lock=%u delta=%u sync_us=%u read_us=%u",
               s_final_edge_b_last_result.acq_phase_sync_status,
               s_final_edge_b_last_result.acq_phase_sync_probes,
               s_final_edge_b_last_result.acq_phase_sync_seed_tmst,
               s_final_edge_b_last_result.acq_phase_sync_lock_tmst,
               s_final_edge_b_last_result.acq_phase_sync_delta,
               s_final_edge_b_last_result.acq_phase_sync_us,
               s_final_edge_b_last_result.acq_phase_sync_read_us_max);
    ZY100_DIAG_LOG("[FE_BPHASE2] no_change=%u soft_start=%u status=%u probes=%u sync_us=%u",
               s_final_edge_b_last_result.acq_phase_sync_no_change_count,
               s_final_edge_b_last_result.acq_phase_sync_soft_start_count,
               s_final_edge_b_last_result.acq_phase_sync_status,
               s_final_edge_b_last_result.acq_phase_sync_probes,
               s_final_edge_b_last_result.acq_phase_sync_us);
    v0_final_edge_end_summary_log_delay();

#if (ZY100_FINAL_EDGE_BSCAN_OD_DIAG_ENABLE || \
     ZY100_FINAL_EDGE_BSCAN_TMST_SUMMARY_ENABLE)
    DBG_DIRECT("[FE_BTMST_SUM] dt_bad=%u stale=%u dmin=%u dmax=%u ok156=%u zero=%u dbl=%u bad=%u other=%u",
               s_final_edge_b_last_result.dt_bad,
               s_final_edge_b_last_result.stale,
               v0_final_edge_od_diag_delta_min(),
               v0_final_edge_od_diag_delta_max(),
               s_final_edge_od_diag_stats.tmst_delta_156_157_count,
               s_final_edge_od_diag_stats.tmst_delta_zero_count,
               s_final_edge_od_diag_stats.tmst_delta_double_count,
               s_final_edge_od_diag_stats.tmst_delta_bad_count,
               s_final_edge_od_diag_stats.tmst_delta_other_bad_count);
    ZY100_DIAG_LOG("[FE_BTMST_POS] first_id=%u first_seq=%u last_id=%u last_seq=%u read_us_max=%u read_scan=%u read_seq=%u",
               s_final_edge_od_diag_stats.tmst_delta_first_bad_scan_id,
               s_final_edge_od_diag_stats.tmst_delta_first_bad_seq,
               s_final_edge_od_diag_stats.tmst_delta_last_bad_scan_id,
               s_final_edge_od_diag_stats.tmst_delta_last_bad_seq,
               s_final_edge_od_diag_stats.read_us_max,
               s_final_edge_od_diag_stats.read_us_max_scan_id,
               s_final_edge_od_diag_stats.read_us_max_seq);
#else
    ZY100_DIAG_LOG("[FE_BTMST_SUM] disabled=1 dt_bad=%u stale=%u",
               s_final_edge_b_last_result.dt_bad,
               s_final_edge_b_last_result.stale);
    ZY100_DIAG_LOG("[FE_BTMST_POS] disabled=1");
#endif
    v0_final_edge_end_summary_log_delay();

    DBG_DIRECT("[FE_STATUS] state=%u stop=%u export_ready=%u empty_abort=%u raw_busy=%u rec_busy=%u replay_pending=%u replay_active=%u b_active=%u",
               (uint32_t)s_fe_capture_state,
               (uint32_t)s_fe_stop_reason,
               s_fe_export_ready ? 1U : 0U,
               s_final_edge_empty_error_aborted ? 1U : 0U,
               raw_busy,
               zy100_final_edge_record_store_has_pending_work() ? 1U : 0U,
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
               s_final_edge_replay_pending ? 1U : 0U,
               s_final_edge_replay_active ? 1U : 0U,
#else
               0U,
               0U,
#endif
               s_final_edge_b_scan_active ? 1U : 0U);
    v0_final_edge_end_summary_log_delay();
#endif
}
#endif

static const char *v0_spi_sched_mode_name(void)
{
#if !ZY100_RT_MARKER_ENABLE
    return "T3_FIFO_ONLY";
#elif ZY100_RT_MARKER_TIMER_ONLY_TEST
    return "T1";
#elif ZY100_RT_MARKER_FIFO_ONLY_TEST
    return "T2";
#else
    return "T3_RT";
#endif
}

static void v0_mark_read_error(v0_fifo_stats_t *stats, imu_status_t status)
{
    stats->read_error_count++;
    stats->fatal_error = true;
    stats->fatal_status = status;
#if ZY100_LEGACY_OFFLINE_ENABLE
    v0_offline_fifo_latch_fault(V0_OFFLINE_FAULT_SPI_READ,
                                stats,
                                stats->max_fifo_count,
                                0U,
                                0U,
                                0U,
                                V0_OFFLINE_FIFO_STAGE_NORMAL);
#endif
#if IMU_ERROR_LOG_ENABLE
#if ZY100_LEGACY_OFFLINE_ENABLE
    if (!s_v0_offline_fifo_diag.active)
    {
        DBG_DIRECT("[ERR][IMU] read_error count=%u status=%u",
                   stats->read_error_count,
                   (uint32_t)status);
    }
#else
    DBG_DIRECT("[ERR][IMU] read_error count=%u status=%u",
               stats->read_error_count,
               (uint32_t)status);
#endif
#endif
}

#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
static bool v0_final_edge_restore_live_full_config(v0_fifo_stats_t *stats,
                                                   imu_status_t *status_out)
{
    icm53611_cfg_t imu_cfg;
    icm53611_fifo_cfg_t fifo_cfg;
    imu_status_t status;
    uint16_t fifo_count = 0U;

    v0_fill_phase_a_ui_fifo_config(&imu_cfg, &fifo_cfg);
    status = icm53611_configure_packet3_fifo_stream(&imu_cfg, &fifo_cfg);
    if (status == IMU_STATUS_OK)
    {
        status = icm53611_fifo_flush_wait_clear(&fifo_count);
    }

    if (status_out != NULL)
    {
        *status_out = status;
    }
    if (status != IMU_STATUS_OK)
    {
        if (stats != NULL)
        {
            v0_mark_read_error(stats, status);
        }
        return false;
    }
    return true;
}

static bool v0_final_edge_restore_live_after_b(v0_fifo_stats_t *stats)
{
    imu_status_t status;
    uint32_t now_ms;
#if ZY100_FINAL_EDGE_B_RESTORE_FIFO_FLUSH_ENABLE
    uint16_t fifo_count_after_flush = 0U;
#endif

    status = icm53611_restore_ui_fifo_800hz_minimal();
    if (status != IMU_STATUS_OK)
    {
        s_final_edge_restore_fallback_count++;
        s_final_edge_restore_fallback_status = (uint32_t)status;
        if (!v0_final_edge_restore_live_full_config(stats, &status))
        {
            s_final_edge_restore_fallback_status = (uint32_t)status;
            return false;
        }
    }

#if ZY100_FINAL_EDGE_B_RESTORE_FIFO_FLUSH_ENABLE
    status = icm53611_fifo_flush_wait_clear(&fifo_count_after_flush);
    v0_final_edge_inc_stat(
        &s_final_edge_phase1_stats.b_restore_fifo_flush_count);
    s_final_edge_phase1_stats.b_restore_fifo_flush_last_status =
        (uint32_t)status;
    s_final_edge_phase1_stats.b_restore_fifo_flush_last_count =
        (uint32_t)fifo_count_after_flush;
    if ((status != IMU_STATUS_OK) || !v0_online_quiet_logs())
    {
        ZY100_DIAG_LOG("[FE_BRESTORE_FIFO] flush=1 status=%u count=%u",
                   (uint32_t)status,
                   fifo_count_after_flush);
    }
    v0_online_capture_coop_yield(zy100_os_time_ms());
    if (status != IMU_STATUS_OK)
    {
        v0_final_edge_inc_stat(
            &s_final_edge_phase1_stats.b_restore_fifo_flush_fail_count);
        if (stats != NULL)
        {
            v0_mark_read_error(stats, status);
        }
        return false;
    }
#endif

    now_ms = zy100_os_time_ms();
    zy100_spi_sched_reset(now_ms);
    zy100_spi_sched_mark_session_start(now_ms);
    zy100_spi_sched_invalidate_fifo_count();
    v0_reset_fifo_drain_diag_ring();
    s_v0_imu_fifo_pending_count = 0U;
    s_v0_high_water_alarm_pending = false;
    s_v0_phase_a_rescue_pending = false;
    imu_bsp_int_clear_pending();
    imu_bsp_int_register_irq_callback(v0_imu_irq_callback);
    status = imu_bsp_int_init();
    if (status != IMU_STATUS_OK)
    {
        if (stats != NULL)
        {
            v0_mark_read_error(stats, status);
        }
        return false;
    }
    imu_bsp_int_clear_pending();

    if (stats != NULL)
    {
        stats->tmst_valid = false;
        stats->ts_warmup_remaining = V0_TS_WARMUP_PACKET_COUNT;
        stats->last_fifo_drain_ms = now_ms;
    }
#if ZY100_RT_MARKER_ENABLE
    IMU_UNUSED(imu_rt_marker_reset(0ULL));
#endif
    s_final_edge_rt_deferred_active = false;
    s_final_edge_rt_gated_for_b = false;

    if (!v0_final_edge_timer_start())
    {
        if (stats != NULL)
        {
            v0_mark_read_error(stats, IMU_STATUS_NOT_READY);
        }
        return false;
    }
    return true;
}

static void v0_final_edge_note_b_scan_start_busy(bool replay_busy,
                                                 bool raw_busy)
{
    if (replay_busy)
    {
        v0_final_edge_inc_stat(
            &s_final_edge_phase1_stats.b_start_replay_busy_count);
    }
    if (raw_busy)
    {
        v0_final_edge_inc_stat(
            &s_final_edge_phase1_stats.b_start_raw_busy_count);
    }
    if (!replay_busy && !raw_busy)
    {
        return;
    }

    v0_final_edge_inc_stat(
        &s_final_edge_phase1_stats.b_scan_start_reject_raw_replay_count);
}

static bool v0_final_edge_consume_deferred_b_scan(v0_fifo_stats_t *stats)
{
    bool run_ok;
    bool restore_ok;
    uint32_t fail_reason;
    uint32_t post_start_us;
    uint32_t post_us;
    uint32_t restore_start_us;
    uint32_t restore_us;
    bool replay_busy = false;
    bool raw_busy = false;

    if ((stats == NULL) || stats->fatal_error)
    {
        return false;
    }
    if (!s_final_edge_rt_deferred_active || s_final_edge_b_scan_active)
    {
        return true;
    }
    if (s_v0_stop_requested ||
        (s_fe_capture_state >= FE_CAPTURE_STATE_STOP_REQUESTED))
    {
        s_final_edge_rt_deferred_active = false;
        s_final_edge_rt_trigger_valid = false;
        s_final_edge_rt_gated_for_b = false;
        v0_final_edge_b_critical_clear();
        s_final_edge_phase1_stats.rt_gated_count++;
        s_final_edge_phase1_stats.rt_gated_other_count++;
        return true;
    }
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
    replay_busy = v0_final_edge_replay_needs_service();
#endif
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    raw_busy = !zy100_final_edge_raw_store_is_idle();
#endif
    v0_final_edge_note_b_scan_start_busy(replay_busy, raw_busy);
#if ZY100_FINAL_EDGE_B_HARD_RT_ACQ_ENABLE
    if (replay_busy || raw_busy)
    {
        s_final_edge_phase1_stats.rt_request_expired_count++;
        s_final_edge_rt_deferred_active = false;
        s_final_edge_rt_trigger_valid = false;
        s_final_edge_rt_gated_for_b = false;
        v0_final_edge_b_critical_clear();
        return true;
    }
#endif

    zy100_online_reset_trace_set_imu_phase(ZY100_ONLINE_TRACE_IMU_B_BEGIN);
    v0_final_edge_b_critical_set_active();
    s_final_edge_b_scan_active = true;
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    zy100_final_edge_raw_store_set_b_active(true);
#endif
    zy100_final_edge_record_store_set_b_active(true);
    s_final_edge_rt_gated_for_b = true;
    v0_final_edge_suspend_live_dt_baseline(stats);
    v0_final_edge_timer_stop();
    v0_online_trace_runtime();

    v0_final_edge_reset_b_last_result();
    zy100_online_reset_trace_set_imu_phase(ZY100_ONLINE_TRACE_IMU_B_RUN);
    run_ok = zy100_final_edge_bscan_run(
        s_final_edge_b_ois_ring,
        sizeof(s_final_edge_b_ois_ring),
        &s_final_edge_b_last_result);
    v0_online_trace_runtime();

    if (imu_bsp_ois_tick_timer_is_running() ||
        (imu_bsp_ois_tick_timer_get_owner() ==
         IMU_BSP_OIS_TICK_TIMER_OWNER_OIS_CAPTURE) ||
        (spi_bus_current_owner() == SPI_OWNER_IMU))
    {
        s_final_edge_b_scan_active = false;
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
        zy100_final_edge_raw_store_set_b_active(false);
#endif
        zy100_final_edge_record_store_set_b_active(false);
        s_final_edge_rt_deferred_active = false;
        s_final_edge_rt_trigger_valid = false;
        s_final_edge_rt_gated_for_b = false;
        v0_final_edge_b_critical_clear();
        v0_mark_read_error(stats, IMU_STATUS_NOT_READY);
        (void)imu_fifo_drain_test_request_online_abort();
        return false;
    }

    if (s_final_edge_b_last_result.begin_retry != 0U)
    {
        v0_final_edge_inc_stat(
            &s_final_edge_phase1_stats.b_start_spi_retry_count);
        s_final_edge_b_scan_active = false;
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
        zy100_final_edge_raw_store_set_b_active(false);
#endif
        zy100_final_edge_record_store_set_b_active(false);
        if (!v0_final_edge_timer_start())
        {
            v0_mark_read_error(stats, IMU_STATUS_NOT_READY);
            s_final_edge_rt_deferred_active = false;
            s_final_edge_rt_trigger_valid = false;
            s_final_edge_rt_gated_for_b = false;
            v0_final_edge_b_critical_clear();
            return false;
        }
        s_final_edge_phase1_stats.rt_request_expired_count++;
        s_final_edge_rt_deferred_active = false;
        s_final_edge_rt_trigger_valid = false;
        s_final_edge_rt_gated_for_b = false;
        v0_final_edge_b_critical_clear();
        return true;
    }

    post_start_us = (uint32_t)imu_bsp_local_timestamp_us();
    v0_final_edge_b_critical_note_first_frame(&s_final_edge_b_last_result);
    s_final_edge_b_scan_count++;
    s_final_edge_last_b_id = s_final_edge_b_scan_count;
    v0_final_edge_note_official_b_result(&s_final_edge_b_last_result);
#if ZY100_FINAL_EDGE_PHASE4_HIT_WINDOW_ENABLE
    v0_final_edge_note_hit_window_result(&s_final_edge_b_last_result);
#endif
    fail_reason = v0_final_edge_b_scan_fail_reason(run_ok);
    v0_final_edge_note_od_diag_result(&s_final_edge_b_last_result,
                                      s_final_edge_last_b_id);
    v0_final_edge_note_b_mismatch_result(&s_final_edge_b_last_result,
                                         s_final_edge_last_b_id);
#if ZY100_ONLINE_STREAM_ENABLE
    v0_online_diag_note_b_result(&s_final_edge_b_last_result,
                                 s_final_edge_last_b_id,
                                 fail_reason);
#endif
#if (ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE && \
     ZY100_FINAL_EDGE_PHASE4_HIT_WINDOW_ENABLE)
    v0_final_edge_phase5_begin_hit_after_b(fail_reason);
#endif
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
    v0_final_edge_phase6_begin_nohit_after_b(fail_reason);
#if (ZY100_FINAL_EDGE_PHASE61_TRUEHIT_REPLAY_ENABLE && \
     ZY100_FINAL_EDGE_TRUEHIT_REPLAY_ENABLE && \
     ZY100_FINAL_EDGE_PHASE4_HIT_WINDOW_ENABLE)
    v0_final_edge_phase61_begin_truehit_after_b(fail_reason);
#endif
#endif
    v0_final_edge_note_b_scan_fail_reason(fail_reason, run_ok);
#if ZY100_FINAL_EDGE_HIT20_CALIB_ENABLE
    v0_final_edge_hit20_calib_note_b_result(&s_final_edge_b_last_result);
#endif

    post_us = (uint32_t)((uint32_t)imu_bsp_local_timestamp_us() -
                         post_start_us);
    restore_start_us = (uint32_t)imu_bsp_local_timestamp_us();
    zy100_online_reset_trace_set_imu_phase(
        ZY100_ONLINE_TRACE_IMU_B_RESTORE);
    v0_online_trace_runtime();
    restore_ok = v0_final_edge_restore_live_after_b(stats);
    restore_us = (uint32_t)((uint32_t)imu_bsp_local_timestamp_us() -
                            restore_start_us);
    s_final_edge_resume_ok = restore_ok;
    if (restore_ok)
    {
        s_final_edge_restore_ok_count++;
        v0_final_edge_post_restore_quiet_begin();
    }
    else
    {
        s_final_edge_restore_fail_count++;
        v0_final_edge_post_restore_quiet_clear();
    }

    s_final_edge_b_scan_active = false;
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    zy100_final_edge_raw_store_set_b_active(false);
#endif
    zy100_final_edge_record_store_set_b_active(false);
    if (!restore_ok)
    {
        s_final_edge_rt_deferred_active = false;
        s_final_edge_rt_trigger_valid = false;
        s_final_edge_rt_gated_for_b = false;
    }
    v0_final_edge_b_critical_clear();
    zy100_online_reset_trace_set_imu_phase(ZY100_ONLINE_TRACE_IMU_LIVE);
    v0_online_trace_runtime();
    {
        uint32_t raw_pending = 0U;
        uint32_t replay_pending = 0U;
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
        raw_pending = zy100_final_edge_raw_store_has_pending_work() ? 1U : 0U;
#endif
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
        replay_pending = v0_final_edge_replay_needs_service() ? 1U : 0U;
#endif
        if (!v0_online_quiet_logs())
        {
            DBG_DIRECT("[FE_BPOST] post_us=%u restore_us=%u ok=%u raw=%u rec=%u replay=%u",
                       post_us,
                       restore_us,
                       restore_ok ? 1U : 0U,
                       raw_pending,
                       zy100_final_edge_record_store_has_pending_work() ? 1U : 0U,
                       replay_pending);
        }
    }
    return restore_ok;
}
#endif

#if ZY100_RT_MARKER_ENABLE && ZY100_RT_MARKER_TIMER_ENABLE
static bool v0_rt_start_delay_enabled(void)
{
#if ZY100_RT_START_AFTER_FIRST_FIFO_DRAIN
    return (ZY100_RT_MARKER_TIMER_ONLY_TEST == 0);
#else
    return false;
#endif
}

static void v0_rt_marker_init_start_gate(v0_fifo_stats_t *stats)
{
    if (stats == NULL)
    {
        return;
    }

    stats->rt_start_good_drain = false;
    stats->rt_start_last_drain_seq = 0U;
    stats->rt_start_last_cached_count = 0U;
    stats->rt_start_last_bad_count = 0U;
    stats->rt_start_last_lost_count = 0U;
    stats->rt_start_last_full_count = 0U;
    stats->rt_start_wait_reason = "none_yet";
}

static void v0_rt_marker_note_drain_start_candidate(
    v0_fifo_stats_t *stats,
    const zy100_spi_sched_drain_context_t *drain_ctx,
    bool drain_success,
    uint32_t bad_before,
    uint16_t lost_before,
    uint32_t full_before,
    bool post_count_valid,
    uint16_t post_count)
{
    const char *reason = "good";
    bool good = false;

    if ((stats == NULL) || !v0_rt_start_delay_enabled() ||
        zy100_rt_marker_timer_is_running())
    {
        return;
    }

    if (drain_ctx != NULL)
    {
        stats->rt_start_last_drain_seq = drain_ctx->seq;
        stats->rt_start_last_cached_count =
            drain_ctx->cached_count_valid ? drain_ctx->cached_count : 0U;
    }
    stats->rt_start_last_bad_count = stats->bad_header_count;
    stats->rt_start_last_lost_count = stats->fifo_lost_pkt_count;
    stats->rt_start_last_full_count = stats->fifo_full_count;

    if (!drain_success)
    {
        reason = "drain_failed";
    }
    else if ((drain_ctx != NULL) &&
             (drain_ctx->src == ZY100_SPI_SCHED_DRAIN_SRC_EMERGENCY))
    {
        reason = "emergency";
    }
    else if (stats->bad_header_count != bad_before)
    {
        reason = "new_bad";
    }
    else if (stats->fifo_lost_pkt_count != lost_before)
    {
        reason = "new_lost";
    }
    else if (stats->fifo_full_count != full_before)
    {
        reason = "new_full";
    }
    else if (post_count_valid &&
             ((uint32_t)post_count >= ZY100_FIFO_EMERGENCY_BYTES))
    {
        reason = "post_count_emergency";
    }
    else
    {
        good = true;
    }

    stats->rt_start_wait_reason = reason;
    stats->rt_start_good_drain = good;
}

static void v0_rt_marker_log_start_wait_if_due(v0_fifo_stats_t *stats,
                                               uint32_t runtime_ms,
                                               uint32_t *last_log_ms)
{
    const char *reason;

    if ((stats == NULL) || (last_log_ms == NULL) ||
        !v0_rt_start_delay_enabled() || zy100_rt_marker_timer_is_running())
    {
        return;
    }

    if ((runtime_ms < V0_RT_START_WAIT_LOG_PERIOD_MS) ||
        ((runtime_ms - *last_log_ms) < V0_RT_START_WAIT_LOG_PERIOD_MS))
    {
        return;
    }

    reason = (stats->rt_start_wait_reason != NULL) ?
             stats->rt_start_wait_reason : "none_yet";
    *last_log_ms = runtime_ms;
    DBG_DIRECT("[RT_START_WAIT] reason=%s drain_seq=%u cached_count=%u bad=%u lost=%u full=%u",
               reason,
               stats->rt_start_last_drain_seq,
               stats->rt_start_last_cached_count,
               stats->rt_start_last_bad_count,
               stats->rt_start_last_lost_count,
               stats->rt_start_last_full_count);
}

static bool v0_rt_marker_start_after_good_drain(v0_fifo_stats_t *stats)
{
    imu_status_t status;

    if ((stats == NULL) || !v0_rt_start_delay_enabled() ||
        !stats->rt_start_good_drain)
    {
        return false;
    }

    IMU_UNUSED(imu_rt_marker_reset(0ULL));
    zy100_rt_marker_timer_reset_counters();
    status = zy100_rt_marker_timer_start();
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
        return false;
    }

    DBG_DIRECT("[RT_START] after_first_fifo_drain=1 drain_seq=%u",
               stats->rt_start_last_drain_seq);
    return true;
}
#endif

#if !ZY100_FINAL_EDGE_MODE_ENABLE
static void v0_log_fifo_irq_diag_if_due(uint32_t runtime_ms,
                                        uint32_t irq_count,
                                        uint32_t pending_consumed_count,
                                        uint32_t *last_log_ms,
                                        uint32_t *last_irq_count,
                                        uint32_t *last_pending_consumed_count,
                                        uint32_t *last_latch_count)
{
#if !ZY100_RUNTIME_STATS_LOG_ENABLE
    IMU_UNUSED(runtime_ms);
    IMU_UNUSED(irq_count);
    IMU_UNUSED(pending_consumed_count);
    IMU_UNUSED(last_log_ms);
    IMU_UNUSED(last_irq_count);
    IMU_UNUSED(last_pending_consumed_count);
    IMU_UNUSED(last_latch_count);
#else
    zy100_spi_sched_stats_t sched_stats;
    uint32_t elapsed_ms;
    uint32_t latch_count;
    uint32_t isr_irq_per_s;
    uint32_t pending_consumed_per_s;
    uint32_t irq_to_drain_latch_per_s;

    if ((last_log_ms == NULL) ||
        (last_irq_count == NULL) ||
        (last_pending_consumed_count == NULL) ||
        (last_latch_count == NULL))
    {
        return;
    }

    elapsed_ms = v0_delta_u32(runtime_ms, *last_log_ms);
    if (elapsed_ms < V0_LOG_PERIOD_MS)
    {
        return;
    }

    /* Counters are updated at each event; copy only when this report is due. */
    zy100_spi_sched_get_stats(&sched_stats);
    latch_count = sched_stats.fifo_irq_to_drain_latch;
    isr_irq_per_s = v0_rate_per_s(v0_delta_u32(irq_count, *last_irq_count),
                                  elapsed_ms);
    pending_consumed_per_s = v0_rate_per_s(
        v0_delta_u32(pending_consumed_count, *last_pending_consumed_count),
        elapsed_ms);
    irq_to_drain_latch_per_s = v0_rate_per_s(
        v0_delta_u32(latch_count, *last_latch_count),
        elapsed_ms);

    DBG_DIRECT("[FIFO_IRQ_1S] isr_irq_per_s=%u pending_consumed_per_s=%u irq_to_drain_latch_per_s=%u int_level=%u",
               isr_irq_per_s,
               pending_consumed_per_s,
               irq_to_drain_latch_per_s,
               (uint32_t)imu_bsp_int_level());

    *last_log_ms = runtime_ms;
    *last_irq_count = irq_count;
    *last_pending_consumed_count = pending_consumed_count;
    *last_latch_count = latch_count;
#endif
}
#endif

static const char *v0_fifo_drain_src_name(zy100_spi_sched_drain_src_t src)
{
    switch (src)
    {
    case ZY100_SPI_SCHED_DRAIN_SRC_IRQ_FIXED:
        return "irq_fixed";
    case ZY100_SPI_SCHED_DRAIN_SRC_COUNT_BASED:
        return "count_based";
    case ZY100_SPI_SCHED_DRAIN_SRC_CONFIRMED:
        return "confirmed";
    case ZY100_SPI_SCHED_DRAIN_SRC_EMERGENCY:
        return "emergency";
    case ZY100_SPI_SCHED_DRAIN_SRC_FINAL:
        return "final";
    default:
        return "unknown";
    }
}

static void v0_reset_fifo_drain_diag_ring(void)
{
    memset(s_v0_fifo_drain_ctx_ring, 0, sizeof(s_v0_fifo_drain_ctx_ring));
    s_v0_fifo_drain_ctx_ring_next = 0U;
    s_v0_fifo_drain_ctx_ring_count = 0U;
}

static void v0_log_fifo_drain_begin_ctx(const zy100_spi_sched_drain_context_t *ctx,
                                        bool saved)
{
    if (ctx == NULL)
    {
        return;
    }

    DBG_DIRECT("[FIFO_DRAIN_BEGIN] saved=%u seq=%u src=%s req_len=%u actual_len=%u cached_count_valid=%u cached_count=%u fifo_irq_pending=%u irq_pending_count=%u int_level=%u queue_level=%u flash_capacity_packets=%u fill_packets=%u fast_path=%u time_since_last_irq_ms=%u time_since_last_drain_ms=%u capped_by_flash_capacity=%u",
               saved ? 1U : 0U,
               ctx->seq,
               v0_fifo_drain_src_name(ctx->src),
               ctx->req_len,
               ctx->actual_len,
               ctx->cached_count_valid ? 1U : 0U,
               ctx->cached_count,
               ctx->fifo_irq_pending ? 1U : 0U,
               ctx->irq_pending_count,
               ctx->int_level,
               ctx->queue_level,
               ctx->flash_capacity_packets,
               ctx->fill_packets,
               ctx->fast_path ? 1U : 0U,
               ctx->time_since_last_irq_ms,
               ctx->time_since_last_drain_ms,
               ctx->capped_by_flash_capacity ? 1U : 0U);
}

static void v0_store_fifo_drain_context(const zy100_spi_sched_drain_context_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    s_v0_fifo_drain_ctx_ring[s_v0_fifo_drain_ctx_ring_next].valid = true;
    s_v0_fifo_drain_ctx_ring[s_v0_fifo_drain_ctx_ring_next].ctx = *ctx;
    s_v0_fifo_drain_ctx_ring_next =
        (uint8_t)((s_v0_fifo_drain_ctx_ring_next + 1U) %
                  V0_FIFO_DRAIN_CTX_RING_SIZE);
    if (s_v0_fifo_drain_ctx_ring_count < V0_FIFO_DRAIN_CTX_RING_SIZE)
    {
        s_v0_fifo_drain_ctx_ring_count++;
    }

#if ZY100_FIFO_DRAIN_BEGIN_VERBOSE
    v0_log_fifo_drain_begin_ctx(ctx, false);
#endif
}

static void v0_log_fifo_drain_context_ring(void)
{
    uint8_t i;
    uint8_t start;

    if (s_v0_fifo_drain_ctx_ring_count == 0U)
    {
        return;
    }

    start = (uint8_t)((s_v0_fifo_drain_ctx_ring_next +
                       V0_FIFO_DRAIN_CTX_RING_SIZE -
                       s_v0_fifo_drain_ctx_ring_count) %
                      V0_FIFO_DRAIN_CTX_RING_SIZE);
    for (i = 0U; i < s_v0_fifo_drain_ctx_ring_count; i++)
    {
        uint8_t idx = (uint8_t)((start + i) % V0_FIFO_DRAIN_CTX_RING_SIZE);
        if (s_v0_fifo_drain_ctx_ring[idx].valid)
        {
            v0_log_fifo_drain_begin_ctx(&s_v0_fifo_drain_ctx_ring[idx].ctx, true);
        }
    }
}

static void v0_imu_irq_callback(void)
{
#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE && V1_IMU_FLASH_CAPTURE_ENABLE
    imu_bsp_int_clear_pending();
    s_v0_high_water_alarm_pending = true;
    s_v0_imu_fifo_pending_count++;
    if (s_v0_irq_count < 0xFFFFFFFFU)
    {
        s_v0_irq_count++;
    }
    if (s_v0_task_handle != NULL)
    {
        (void)xTaskNotifyFromISR(s_v0_task_handle,
                                 V0_NOTIFY_FIFO_IRQ,
                                 eSetBits,
                                 NULL);
    }
#else
#if V0_FIFO_MARKER_TRIGGER_ENABLED
    uint64_t now_us = imu_bsp_local_timestamp_us();
    uint32_t sample_seq = s_v0_current_sample_seq;
#endif

    s_v0_imu_fifo_pending_count++;
    if (s_v0_irq_count < 0xFFFFFFFFU)
    {
        s_v0_irq_count++;
    }
    if (s_v0_task_handle != NULL)
    {
        (void)xTaskNotifyFromISR(s_v0_task_handle,
                                 V0_NOTIFY_FIFO_IRQ,
                                 eSetBits,
                                 NULL);
    }
#if V0_FIFO_MARKER_TRIGGER_ENABLED
    imu_wom_irq_on_gpio_isr(now_us, sample_seq);
#endif
#endif
}

#if ZY100_RT_MARKER_ENABLE && ZY100_RT_MARKER_TIMER_ENABLE
static void v0_rt_marker_timer_notify_from_isr(void)
{
    if (s_v0_task_handle != NULL)
    {
        (void)xTaskNotifyFromISR(s_v0_task_handle,
                                 V0_NOTIFY_RT_TIMER,
                                 eSetBits,
                                 NULL);
    }
}
#endif

static bool v0_header_is_valid(uint8_t header)
{
    if ((header & V0_HEADER_MSG) != 0U)
    {
        return false;
    }
    if ((header & (V0_HEADER_ACCEL | V0_HEADER_GYRO)) != (V0_HEADER_ACCEL | V0_HEADER_GYRO))
    {
        return false;
    }
    if ((header & V0_HEADER_20) != 0U)
    {
        return false;
    }
    return ((header & V0_HEADER_TIMESTAMP_MASK) == V0_HEADER_TIMESTAMP_ODR);
}

static uint16_t v0_packet_tmst_raw(const uint8_t *packet)
{
    return (uint16_t)(((uint16_t)packet[V0_PACKET_TMST_HIGH_IDX] << 8) |
                      packet[V0_PACKET_TMST_LOW_IDX]);
}

#if ZY100_EDGE_SHADOW_ENABLE
static int16_t v0_packet_be_s16(const uint8_t *packet, uint8_t high_idx, uint8_t low_idx)
{
    return (int16_t)(((uint16_t)packet[high_idx] << 8) | packet[low_idx]);
}

static bool v0_decode_fifo_packet_6axis_raw(const uint8_t *packet16,
                                            zy100_edge_shadow_sample_t *out_sample)
{
    if ((packet16 == NULL) || (out_sample == NULL) || !v0_header_is_valid(packet16[0]))
    {
        return false;
    }

    out_sample->sample_index = 0U;
    out_sample->timestamp_raw = v0_packet_tmst_raw(packet16);
    out_sample->fifo_header = packet16[0];
    out_sample->flags = 0U;
    out_sample->accel_x_raw = v0_packet_be_s16(packet16,
                                               V0_PACKET_ACCEL_X_HIGH_IDX,
                                               V0_PACKET_ACCEL_X_LOW_IDX);
    out_sample->accel_y_raw = v0_packet_be_s16(packet16,
                                               V0_PACKET_ACCEL_Y_HIGH_IDX,
                                               V0_PACKET_ACCEL_Y_LOW_IDX);
    out_sample->accel_z_raw = v0_packet_be_s16(packet16,
                                               V0_PACKET_ACCEL_Z_HIGH_IDX,
                                               V0_PACKET_ACCEL_Z_LOW_IDX);
    out_sample->gyro_x_raw = v0_packet_be_s16(packet16,
                                              V0_PACKET_GYRO_X_HIGH_IDX,
                                              V0_PACKET_GYRO_X_LOW_IDX);
    out_sample->gyro_y_raw = v0_packet_be_s16(packet16,
                                              V0_PACKET_GYRO_Y_HIGH_IDX,
                                              V0_PACKET_GYRO_Y_LOW_IDX);
    out_sample->gyro_z_raw = v0_packet_be_s16(packet16,
                                              V0_PACKET_GYRO_Z_HIGH_IDX,
                                              V0_PACKET_GYRO_Z_LOW_IDX);
    out_sample->temp_raw = 0;
    return true;
}
#endif

static uint64_t v0_fifo_sample_t_us(uint32_t sample_seq)
{
    if (sample_seq == 0U)
    {
        return 0ULL;
    }
    return (uint64_t)(sample_seq - 1U) * (uint64_t)V0_SAMPLE_INTERVAL_US;
}

#if V0_FIFO_MARKER_TRIGGER_ENABLED
static uint64_t v0_detector_sample_ts_us(uint32_t sample_seq)
{
    if (sample_seq == 0U)
    {
        return s_v0_detector_sample_base_ts_us;
    }

    return s_v0_detector_sample_base_ts_us +
           ((uint64_t)(sample_seq - 1U) * (uint64_t)V0_SAMPLE_INTERVAL_US);
}

static bool v0_detector_rate_gate_allow(void)
{
    uint16_t target_hz = imu_swing_detector_input_hz();

    if (target_hz == 0U)
    {
        return false;
    }

    if ((uint32_t)target_hz >= V0_SAMPLE_RATE_HZ)
    {
        return true;
    }

    s_v0_detector_rate_accum += (uint32_t)target_hz;
    if (s_v0_detector_rate_accum >= V0_SAMPLE_RATE_HZ)
    {
        s_v0_detector_rate_accum -= V0_SAMPLE_RATE_HZ;
        return true;
    }

    return false;
}

static void v0_swing_detector_feed_packet(uint32_t sample_seq,
                                          uint16_t tmst_raw,
                                          const uint8_t *packet)
{
    imu_swing_detector_input_sample_t sample;

    if (!v0_detector_rate_gate_allow())
    {
        return;
    }

    sample.sample_seq = sample_seq;
    sample.sample_ts_us = v0_detector_sample_ts_us(sample_seq);
    sample.accel_x_raw = v0_packet_be_s16(packet,
                                          V0_PACKET_ACCEL_X_HIGH_IDX,
                                          V0_PACKET_ACCEL_X_LOW_IDX);
    sample.accel_y_raw = v0_packet_be_s16(packet,
                                          V0_PACKET_ACCEL_Y_HIGH_IDX,
                                          V0_PACKET_ACCEL_Y_LOW_IDX);
    sample.accel_z_raw = v0_packet_be_s16(packet,
                                          V0_PACKET_ACCEL_Z_HIGH_IDX,
                                          V0_PACKET_ACCEL_Z_LOW_IDX);
    sample.gyro_x_raw = v0_packet_be_s16(packet,
                                         V0_PACKET_GYRO_X_HIGH_IDX,
                                         V0_PACKET_GYRO_X_LOW_IDX);
    sample.gyro_y_raw = v0_packet_be_s16(packet,
                                         V0_PACKET_GYRO_Y_HIGH_IDX,
                                         V0_PACKET_GYRO_Y_LOW_IDX);
    sample.gyro_z_raw = v0_packet_be_s16(packet,
                                         V0_PACKET_GYRO_Z_HIGH_IDX,
                                         V0_PACKET_GYRO_Z_LOW_IDX);
    sample.imu_fifo_ts16 = tmst_raw;
    sample.imu_fifo_ts16_valid = true;

    IMU_UNUSED(imu_pre_trigger_marker_on_sample(&sample));
    IMU_UNUSED(imu_swing_detector_on_sample(&sample));
}

static void v0_process_int_status(const icm53611_int_status_t *int_status,
                                  uint32_t status_sample_seq,
                                  bool capture_running)
{
    imu_wom_irq_candidate_t wom_candidate;

    if (imu_wom_irq_process_status(int_status,
                                   status_sample_seq,
                                   capture_running,
                                   &wom_candidate))
    {
        imu_swing_detector_wom_candidate_t detector_candidate;

        detector_candidate.candidate_ts_us = wom_candidate.candidate_ts_us;
        detector_candidate.sample_seq = wom_candidate.sample_seq;
        detector_candidate.axis_mask = wom_candidate.axis_mask;
        IMU_UNUSED(imu_swing_detector_on_wom_candidate(&detector_candidate));
    }
}
#endif

#if ZY100_FIFO_SWING_STATS_ENABLE
static void v0_log_combined_imu_swing_stats(void)
{
    imu_wom_irq_stats_t wom_stats;
    imu_swing_detector_stats_t detector_stats;

    imu_wom_irq_get_stats(&wom_stats);
    imu_swing_detector_get_stats(&detector_stats);
    DBG_DIRECT("[IMU_SWING_STATS] software_swing_count=%u wom_raw_count=%u "
               "wom_candidate_count=%u false_candidate_count=%u "
               "unconfirmed_candidate_count=%u",
               detector_stats.software_swing_count,
               wom_stats.wom_raw_count,
               detector_stats.wom_candidate_count,
               detector_stats.false_candidate_count,
               detector_stats.unconfirmed_candidate_count);
}
#endif

typedef enum
{
    V0_PACKET_PROCESS_OK = 0U,
    V0_PACKET_PROCESS_BAD_HEADER,
    V0_PACKET_PROCESS_ONLINE_RAW_ERROR,
    V0_PACKET_PROCESS_OFFLINE_V2_ERROR,
} v0_packet_process_result_t;

static v0_packet_process_result_t v0_process_packet(
    v0_fifo_stats_t *stats,
    const uint8_t *packet,
    uint32_t reserved_batch_id,
    uint16_t offset_in_read_buffer)
{
    uint16_t tmst_raw;
    uint16_t dt_us;
#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
    uint64_t sample_t_us;
#endif
    IMU_UNUSED(reserved_batch_id);
    IMU_UNUSED(offset_in_read_buffer);
#if ZY100_EDGE_SHADOW_ENABLE
    zy100_edge_shadow_sample_t edge_sample;
#if ZY100_FINAL_EDGE_MODE_ENABLE
    zy100_edge_lite_output_t edge_out;
#endif
#endif

    stats->packet_count++;
    s_v0_current_sample_seq = stats->packet_count;
    if (!v0_header_is_valid(packet[0]))
    {
        stats->bad_header_count++;
#if ZY100_FINAL_EDGE_MODE_ENABLE
        s_final_edge_phase1_stats.hdr_bad_count++;
#endif
#if IMU_ERROR_LOG_ENABLE
#if ZY100_LEGACY_OFFLINE_ENABLE
        if (!s_v0_offline_fifo_diag.active)
#endif
        {
        if ((stats->bad_header_count <= 8U) ||
            ((stats->bad_header_count % 256U) == 0U))
        {
            DBG_DIRECT("[ERR][FIFO] bad_frame count=%u header=0x%02x sample_seq=%u",
                       stats->bad_header_count,
                       packet[0],
                       s_v0_current_sample_seq);
        }
        }
#endif
        return V0_PACKET_PROCESS_BAD_HEADER;
    }
#if ZY100_ONLINE_STREAM_ENABLE
    s_v0_online_bad_fifo_consecutive = 0U;
#endif

    tmst_raw = v0_packet_tmst_raw(packet);
#if ZY100_LEGACY_OFFLINE_ENABLE
    if (zy100_offline_v2_capture_input_active())
    {
        if (!zy100_offline_v2_capture_push_imu_packet(packet, tmst_raw))
        {
            return V0_PACKET_PROCESS_OFFLINE_V2_ERROR;
        }
        stats->tmst_valid = true;
        stats->last_tmst_raw = tmst_raw;
        stats->ts_warmup_remaining = 0U;
        return V0_PACKET_PROCESS_OK;
    }
#endif
#if ZY100_ONLINE_STREAM_ENABLE && \
    ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
    if (zy100_online_raw_capture_active())
    {
        if (!zy100_online_raw_capture_push_packet(packet, tmst_raw))
        {
            return V0_PACKET_PROCESS_ONLINE_RAW_ERROR;
        }
        stats->tmst_valid = true;
        stats->last_tmst_raw = tmst_raw;
        stats->ts_warmup_remaining = 0U;
        return V0_PACKET_PROCESS_OK;
    }
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_ONLINE_STREAM_ENABLE && \
    ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
    v0_online_diag_note_live_sample(stats->packet_count, tmst_raw);
#endif
#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
    sample_t_us = v0_fifo_sample_t_us(stats->packet_count);
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_EDGE_SHADOW_ENABLE
    memset(&edge_out, 0, sizeof(edge_out));
    if (v0_decode_fifo_packet_6axis_raw(packet, &edge_sample))
    {
        uint32_t edge_start_us;
        uint32_t edge_elapsed_us;

        edge_sample.sample_index = v0_final_edge_next_edge_sample_index();
        s_final_edge_phase1_stats.packet_count++;
        stats->edge_hook_feed_count++;
        s_final_edge_phase1_stats.edge_feed_count++;
        edge_start_us = (uint32_t)imu_bsp_local_timestamp_us();
        if (zy100_edge_shadow_push_sample_ex(&edge_sample, &edge_out))
        {
            v0_final_edge_handle_lite_output(&edge_out,
                                             edge_sample.sample_index,
                                             tmst_raw);
        }
        edge_elapsed_us = (uint32_t)((uint32_t)imu_bsp_local_timestamp_us() -
                                     edge_start_us);
        s_final_edge_phase1_stats.edge_time_sum_us += edge_elapsed_us;
        if (edge_elapsed_us > s_final_edge_phase1_stats.edge_time_max_us)
        {
            s_final_edge_phase1_stats.edge_time_max_us = edge_elapsed_us;
        }
    }
    else
    {
        zy100_edge_shadow_note_decode_error();
    }
#endif
#if (ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_FINAL_EDGE_PHASE4_HIT_WINDOW_ENABLE && \
     ZY100_FINAL_EDGE_LF_PRE_RING_ENABLE)
    zy100_final_edge_lf_pre_ring_push(stats->packet_count, tmst_raw, packet);
#endif
#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
#if ZY100_FINAL_EDGE_MODE_ENABLE
    v0_final_edge_feed_rt_marker(packet,
                                 stats->packet_count,
                                 tmst_raw,
                                 sample_t_us);
#else
    IMU_UNUSED(imu_rt_marker_feed_fifo_packet(packet,
                                              stats->packet_count,
                                              sample_t_us));
#endif
#endif
#if ZY100_EDGE_SHADOW_ENABLE && !ZY100_FINAL_EDGE_MODE_ENABLE
    stats->edge_hook_feed_count++;
    if (v0_decode_fifo_packet_6axis_raw(packet, &edge_sample))
    {
        edge_sample.sample_index = stats->packet_count;
        zy100_edge_shadow_push_sample(&edge_sample);
    }
    else
    {
        zy100_edge_shadow_note_decode_error();
    }
#endif
#if V0_FIFO_MARKER_TRIGGER_ENABLED
    v0_swing_detector_feed_packet(stats->packet_count, tmst_raw, packet);
#endif
    if (!stats->tmst_valid)
    {
        stats->tmst_valid = true;
        stats->last_tmst_raw = tmst_raw;
        stats->ts_warmup_remaining = V0_TS_WARMUP_PACKET_COUNT;
        return V0_PACKET_PROCESS_OK;
    }

    dt_us = (uint16_t)(tmst_raw - stats->last_tmst_raw);
    stats->last_tmst_raw = tmst_raw;

    if (stats->ts_warmup_remaining > 0U)
    {
        stats->ts_warmup_remaining--;
        return V0_PACKET_PROCESS_OK;
    }

    {
        bool dt_bad = ((dt_us < V0_TS_DELTA_MIN_US) ||
                       (dt_us > V0_TS_DELTA_MAX_US));

#if ZY100_FINAL_EDGE_MODE_ENABLE
        v0_final_edge_note_dt(dt_us, dt_bad);
#endif
        if (dt_bad)
        {
            stats->ts_jump_count++;
        }
    }

    return V0_PACKET_PROCESS_OK;
}

#if ZY100_FINAL_EDGE_MODE_ENABLE
static bool v0_final_edge_process_fifo_tail_packet(
    v0_fifo_stats_t *stats,
    const uint8_t *packet,
    uint32_t *edge_count)
{
    uint16_t tmst_raw;
    uint16_t dt_us;
#if ZY100_EDGE_SHADOW_ENABLE
    zy100_edge_shadow_sample_t edge_sample;
    zy100_edge_lite_output_t edge_out;
#endif

    if ((stats == NULL) || (packet == NULL))
    {
        return false;
    }

    stats->packet_count++;
    s_v0_current_sample_seq = stats->packet_count;
    if (!v0_header_is_valid(packet[0]))
    {
        stats->bad_header_count++;
        s_final_edge_phase1_stats.hdr_bad_count++;
#if IMU_ERROR_LOG_ENABLE
        if ((stats->bad_header_count <= 8U) ||
            ((stats->bad_header_count % 256U) == 0U))
        {
            DBG_DIRECT("[ERR][FIFO] bad_frame count=%u header=0x%02x sample_seq=%u",
                       stats->bad_header_count,
                       packet[0],
                       s_v0_current_sample_seq);
        }
#endif
        return false;
    }
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    s_v0_online_bad_fifo_consecutive = 0U;
#endif

    tmst_raw = v0_packet_tmst_raw(packet);
#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
    if (zy100_online_raw_capture_active())
    {
        IMU_UNUSED(edge_count);
        if (!zy100_online_raw_capture_push_packet(packet, tmst_raw))
        {
            return false;
        }
        stats->tmst_valid = true;
        stats->last_tmst_raw = tmst_raw;
        stats->ts_warmup_remaining = 0U;
        return true;
    }
#endif
#if ZY100_EDGE_SHADOW_ENABLE
    memset(&edge_out, 0, sizeof(edge_out));
    if (v0_decode_fifo_packet_6axis_raw(packet, &edge_sample))
    {
        uint32_t edge_start_us;
        uint32_t edge_elapsed_us;

        edge_sample.sample_index = v0_final_edge_next_edge_sample_index();
        s_final_edge_phase1_stats.packet_count++;
        stats->edge_hook_feed_count++;
        s_final_edge_phase1_stats.edge_feed_count++;
        if (edge_count != NULL)
        {
            (*edge_count)++;
        }
        edge_start_us = (uint32_t)imu_bsp_local_timestamp_us();
        if (zy100_edge_shadow_push_sample_ex(&edge_sample, &edge_out))
        {
            v0_final_edge_handle_lite_output(&edge_out,
                                             edge_sample.sample_index,
                                             tmst_raw);
        }
        edge_elapsed_us = (uint32_t)((uint32_t)imu_bsp_local_timestamp_us() -
                                     edge_start_us);
        s_final_edge_phase1_stats.edge_time_sum_us += edge_elapsed_us;
        if (edge_elapsed_us > s_final_edge_phase1_stats.edge_time_max_us)
        {
            s_final_edge_phase1_stats.edge_time_max_us = edge_elapsed_us;
        }
    }
    else
    {
        zy100_edge_shadow_note_decode_error();
    }
#else
    IMU_UNUSED(edge_count);
#endif

#if (ZY100_FINAL_EDGE_PHASE4_HIT_WINDOW_ENABLE && \
     ZY100_FINAL_EDGE_LF_PRE_RING_ENABLE)
    zy100_final_edge_lf_pre_ring_push(stats->packet_count, tmst_raw, packet);
#endif
    if (!stats->tmst_valid)
    {
        stats->tmst_valid = true;
        stats->last_tmst_raw = tmst_raw;
        stats->ts_warmup_remaining = V0_TS_WARMUP_PACKET_COUNT;
        return true;
    }

    dt_us = (uint16_t)(tmst_raw - stats->last_tmst_raw);
    stats->last_tmst_raw = tmst_raw;

    if (stats->ts_warmup_remaining > 0U)
    {
        stats->ts_warmup_remaining--;
        return true;
    }

    {
        bool dt_bad = ((dt_us < V0_TS_DELTA_MIN_US) ||
                       (dt_us > V0_TS_DELTA_MAX_US));

        v0_final_edge_note_dt(dt_us, dt_bad);
        if (dt_bad)
        {
            stats->ts_jump_count++;
        }
    }

    return true;
}
#endif

static void v0_stop_on_online_raw_error(v0_fifo_stats_t *stats)
{
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    zy100_online_raw_capture_stats_t raw_stats;
    uint32_t now_ms = zy100_os_time_ms();

    memset(&raw_stats, 0, sizeof(raw_stats));
    zy100_online_raw_capture_get_stats(&raw_stats);
    DBG_DIRECT("[ONL_RAW_STOP] fifo_bad=0");
    if (stats != NULL)
    {
        stats->fatal_error = true;
        stats->fatal_status = IMU_STATUS_NOT_READY;
    }
    v0_online_abort_diag_capture_end(stats);
    zy100_online_stream_request_abort_ex(
        ZY100_ONLINE_STOP_REASON_INTERNAL,
        ZY100_ONLINE_ABORT_ORIGIN_IMU_RAW_PACKET,
        (uint32_t)raw_stats.error,
        now_ms);
    (void)imu_fifo_drain_test_request_online_abort_with_reason(
        IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL);
#else
    IMU_UNUSED(stats);
#endif
}

#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
static uint32_t v0_online_raw_accepted_packets(void)
{
    zy100_online_raw_capture_stats_t raw_stats;

    memset(&raw_stats, 0, sizeof(raw_stats));
    zy100_online_raw_capture_get_stats(&raw_stats);
    return raw_stats.accepted_packets;
}

static bool v0_online_raw_publish_fifo_burst(
    v0_fifo_stats_t *stats,
    uint32_t accepted_before,
    uint32_t expected_packets,
    bool time_snapshot_valid,
    const zy100_online_raw_fifo_time_snapshot_t *time_snapshot)
{
    if (!zy100_online_raw_capture_active())
    {
        return true;
    }
    if (time_snapshot_valid &&
        zy100_online_raw_capture_note_fifo_burst(accepted_before,
                                                 expected_packets,
                                                 time_snapshot))
    {
        return true;
    }
    v0_stop_on_online_raw_error(stats);
    return false;
}
#endif

static void v0_stop_on_invalid_fifo_packet(v0_fifo_stats_t *stats,
                                           uint8_t header)
{
    if (stats == NULL)
    {
        return;
    }

    stats->fatal_error = true;
    stats->fatal_status = IMU_STATUS_BUS_ERROR;
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    if (zy100_online_raw_capture_active())
    {
        v0_online_abort_diag_capture_end(stats);
        zy100_online_stream_request_abort_ex(
            ZY100_ONLINE_STOP_REASON_INTERNAL,
            ZY100_ONLINE_ABORT_ORIGIN_IMU_FIFO_FATAL,
            (uint32_t)header,
            zy100_os_time_ms());
        (void)imu_fifo_drain_test_request_online_abort_with_reason(
            IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL);
        return;
    }
#endif
    v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL);
}

#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
static void v0_online_bad_fifo_reset(void)
{
    s_v0_online_bad_fifo_consecutive = 0U;
}

static bool v0_online_tolerate_bad_fifo_packet(
    const v0_fifo_stats_t *stats,
    const zy100_spi_sched_drain_context_t *drain_ctx,
    uint16_t offset,
    uint8_t header,
    uint8_t int_level_after)
{
    uint32_t consecutive;
    uint32_t total_bad;
    uint32_t drain_seq;

    if (zy100_online_raw_capture_active() ||
        !zy100_online_stream_active() ||
        zy100_online_stream_end_requested())
    {
        return false;
    }

    s_v0_online_bad_fifo_consecutive++;
    consecutive = s_v0_online_bad_fifo_consecutive;
    total_bad = (stats != NULL) ? stats->bad_header_count : 0U;
    drain_seq = (drain_ctx != NULL) ? drain_ctx->seq : 0U;

    if (!v0_online_quiet_logs())
    {
        DBG_DIRECT("[ONLINE_FIFO_DROP] header=0x%02x sample_seq=%lu total=%lu consec=%lu limit=%lu drain_seq=%lu offset=%u int=%u",
                   header,
                   (unsigned long)s_v0_current_sample_seq,
                   (unsigned long)total_bad,
                   (unsigned long)consecutive,
                   (unsigned long)V0_ONLINE_BAD_FIFO_CONSECUTIVE_LIMIT,
                   (unsigned long)drain_seq,
                   offset,
                   int_level_after);
    }

    if (consecutive <= V0_ONLINE_BAD_FIFO_CONSECUTIVE_LIMIT)
    {
        return true;
    }

    DBG_DIRECT("[ONLINE_FIFO_DROP] fatal reason=consecutive_bad_header sample_seq=%lu total=%lu consec=%lu",
               (unsigned long)s_v0_current_sample_seq,
               (unsigned long)total_bad,
               (unsigned long)consecutive);
    return false;
}
#else
static void v0_online_bad_fifo_reset(void)
{
}

static bool v0_online_tolerate_bad_fifo_packet(
    const v0_fifo_stats_t *stats,
    const zy100_spi_sched_drain_context_t *drain_ctx,
    uint16_t offset,
    uint8_t header,
    uint8_t int_level_after)
{
    IMU_UNUSED(stats);
    IMU_UNUSED(drain_ctx);
    IMU_UNUSED(offset);
    IMU_UNUSED(header);
    IMU_UNUSED(int_level_after);
    return false;
}
#endif


#if !V0_FLASH_RUNTIME_ENABLE && ZY100_IMU_STARTUP_DBG_LOG_ENABLE
static bool v0_dbg_read_reg(uint8_t reg, uint8_t *value)
{
    imu_status_t status;

    status = imu_spi_read_reg(reg, value);
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[V0_IMU_FIFO][DBG][ERR] read reg=0x%02x status=%d", reg, status);
        return false;
    }

    return true;
}

static bool v0_dbg_read_mreg1(uint8_t maddr, uint8_t *value)
{
    imu_status_t status;

    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1, maddr, value);
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[V0_IMU_FIFO][DBG][ERR] read mreg1=0x%02x status=%d", maddr, status);
        return false;
    }

    return true;
}

static bool v0_dbg_read_fifo_count_raw(uint8_t *count_h, uint8_t *count_l, uint16_t *count)
{
    uint8_t count_l_latched = 0U;

    if ((count_h == NULL) || (count_l == NULL) || (count == NULL))
    {
        return false;
    }

    if (!v0_dbg_read_reg(ICM53611_REG_FIFO_COUNTL, &count_l_latched))
    {
        return false;
    }
    if (!v0_dbg_read_reg(ICM53611_REG_FIFO_COUNTH, count_h))
    {
        return false;
    }
    if (!v0_dbg_read_reg(ICM53611_REG_FIFO_COUNTL, count_l))
    {
        return false;
    }

    IMU_UNUSED(count_l_latched);
    *count = (uint16_t)(((uint16_t)(*count_h) << 8) | *count_l);
    return true;
}

static bool v0_dbg_read_fifo_count_value(uint16_t *count)
{
    imu_status_t status;

    if (count == NULL)
    {
        return false;
    }

    *count = 0U;
    status = icm53611_fifo_get_count(count);
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[V0_IMU_FIFO][DBG][ERR] fifo_count status=%d", status);
        return false;
    }

    return true;
}

static void v0_dbg_log_ui_raw_once(void)
{
    imu_status_t status;
    uint8_t raw[12] = {0U};

    status = imu_spi_read_regs(ICM53611_REG_ACCEL_DATA_X1, raw, sizeof(raw));
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[V0_IMU_FIFO][DBG][ERR] ui_raw status=%d", status);
        return;
    }

    DBG_DIRECT("[V0_IMU_FIFO][DBG] ui_raw accel_x=%02x%02x accel_y=%02x%02x accel_z=%02x%02x "
               "gyro_x=%02x%02x gyro_y=%02x%02x gyro_z=%02x%02x",
               raw[0], raw[1],
               raw[2], raw[3],
               raw[4], raw[5],
               raw[6], raw[7],
               raw[8], raw[9],
               raw[10], raw[11]);
}

static void v0_dbg_log_init_readback(void)
{
    uint8_t pwr_mgmt0 = 0U;
    uint8_t gyro_config0 = 0U;
    uint8_t accel_config0 = 0U;
    uint8_t fifo_config1 = 0U;
    uint8_t fifo_config2 = 0U;
    uint8_t fifo_config3 = 0U;
    uint8_t intf_config0 = 0U;
    uint8_t int_config = 0U;
    uint8_t int_source0 = 0U;
    uint8_t fifo_counth = 0U;
    uint8_t fifo_countl = 0U;
    uint16_t fifo_count = 0U;
    uint8_t int_status = 0U;
    uint8_t tmst_config1 = 0U;
    uint8_t fifo_config5 = 0U;
    uint8_t sensor_config3 = 0U;
    uint8_t fdr_config = 0U;

    if (v0_online_quiet_logs())
    {
        return;
    }

    IMU_UNUSED(v0_dbg_read_reg(ICM53611_REG_PWR_MGMT0, &pwr_mgmt0));
    IMU_UNUSED(v0_dbg_read_reg(ICM53611_REG_GYRO_CONFIG0, &gyro_config0));
    IMU_UNUSED(v0_dbg_read_reg(ICM53611_REG_ACCEL_CONFIG0, &accel_config0));
    IMU_UNUSED(v0_dbg_read_reg(ICM53611_REG_FIFO_CONFIG1, &fifo_config1));
    IMU_UNUSED(v0_dbg_read_reg(ICM53611_REG_FIFO_CONFIG2, &fifo_config2));
    IMU_UNUSED(v0_dbg_read_reg(ICM53611_REG_FIFO_CONFIG3, &fifo_config3));
    IMU_UNUSED(v0_dbg_read_reg(ICM53611_REG_INTF_CONFIG0, &intf_config0));
    IMU_UNUSED(v0_dbg_read_reg(ICM53611_REG_INT_CONFIG, &int_config));
    IMU_UNUSED(v0_dbg_read_reg(ICM53611_REG_INT_SOURCE0, &int_source0));
    IMU_UNUSED(v0_dbg_read_fifo_count_raw(&fifo_counth, &fifo_countl, &fifo_count));
    IMU_UNUSED(v0_dbg_read_reg(ICM53611_REG_INT_STATUS, &int_status));

    DBG_DIRECT("[V0_IMU_FIFO][DBG] init_rb pwr=0x%02x gyro_cfg0=0x%02x accel_cfg0=0x%02x "
               "fifo_cfg1=0x%02x fifo_cfg2=0x%02x fifo_cfg3=0x%02x intf_cfg0=0x%02x",
               pwr_mgmt0,
               gyro_config0,
               accel_config0,
               fifo_config1,
               fifo_config2,
               fifo_config3,
               intf_config0);
    DBG_DIRECT("[V0_IMU_FIFO][DBG] init_rb int_cfg=0x%02x int_src0=0x%02x "
               "fifo_counth=0x%02x fifo_countl=0x%02x fifo_count=%u int_status=0x%02x",
               int_config,
               int_source0,
               fifo_counth,
               fifo_countl,
               fifo_count,
               int_status);

    IMU_UNUSED(v0_dbg_read_mreg1(ICM53611_MREG1_REG_TMST_CONFIG1, &tmst_config1));
    IMU_UNUSED(v0_dbg_read_mreg1(ICM53611_MREG1_REG_FIFO_CONFIG5, &fifo_config5));
    IMU_UNUSED(v0_dbg_read_mreg1(ICM53611_MREG1_REG_SENSOR_CONFIG3, &sensor_config3));
    IMU_UNUSED(v0_dbg_read_mreg1(ICM53611_MREG1_REG_FDR_CONFIG, &fdr_config));
    DBG_DIRECT("[V0_IMU_FIFO][DBG] mreg1_rb tmst_cfg1=0x%02x fifo_cfg5=0x%02x "
               "sensor_cfg3=0x%02x fdr_cfg=0x%02x",
               tmst_config1,
               fifo_config5,
               sensor_config3,
               fdr_config);
}

static void v0_dbg_fifo_count_after_start(void)
{
    uint16_t c1 = 0U;
    uint16_t c2 = 0U;
    uint16_t c3 = 0U;
    bool ok1;
    bool ok2;
    bool ok3;

    if (v0_online_quiet_logs())
    {
        return;
    }

    imu_bsp_delay_us(10000U);
    ok1 = v0_dbg_read_fifo_count_value(&c1);
    imu_bsp_delay_us(10000U);
    ok2 = v0_dbg_read_fifo_count_value(&c2);
    imu_bsp_delay_us(10000U);
    ok3 = v0_dbg_read_fifo_count_value(&c3);

    ZY100_DIAG_LOG("[V0_IMU_FIFO][DBG] fifo_count_after_start c1=%u c2=%u c3=%u",
               c1,
               c2,
               c3);

    if (ok1 && ok2 && ok3 && (c1 == 0U) && (c2 == 0U) && (c3 == 0U))
    {
        v0_dbg_log_ui_raw_once();
    }
}
#endif

static bool v0_refresh_lost_count(v0_fifo_stats_t *stats)
{
    imu_status_t status;
    uint16_t lost_count = 0U;

    status = icm53611_fifo_get_lost_count(&lost_count);
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
        return false;
    }

    if ((lost_count != stats->fifo_lost_pkt_count) && (lost_count != 0U))
    {
#if IMU_ERROR_LOG_ENABLE
        DBG_DIRECT("[ERR][FIFO] fifo_lost count=%u", (uint32_t)lost_count);
#endif
    }
    stats->fifo_lost_pkt_count = lost_count;
    return true;
}

static bool v0_read_int_status(v0_fifo_stats_t *stats, icm53611_int_status_t *snapshot)
{
    imu_status_t status;
    icm53611_int_status_t local_status;
    icm53611_int_status_t *int_status = (snapshot != NULL) ? snapshot : &local_status;

    status = icm53611_read_int_status(int_status);
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
        return false;
    }

    if ((int_status->int_status & ICM53611_INT_STATUS_FIFO_FULL_INT) != 0U)
    {
        stats->fifo_full_count++;
#if IMU_ERROR_LOG_ENABLE
        DBG_DIRECT("[ERR][FIFO] fifo_overflow count=%u", stats->fifo_full_count);
#endif
    }

    if (stats->capture_active &&
        ((int_status->int_status & ICM53611_INT_STATUS_RESET_DONE_INT) != 0U))
    {
        stats->reset_done_count_during_capture++;
    }

    return true;
}

static bool v0_read_fifo_count(v0_fifo_stats_t *stats, uint16_t *fifo_count)
{
    imu_status_t status;

    if (fifo_count == NULL)
    {
        return false;
    }

    *fifo_count = 0U;
    status = icm53611_fifo_get_count(fifo_count);
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
        return false;
    }

    if (*fifo_count > stats->max_fifo_count)
    {
        stats->max_fifo_count = *fifo_count;
    }

    return true;
}

static void v0_log_fifo_bad_diag(v0_fifo_stats_t *stats)
{
    imu_status_t status;
    icm53611_int_status_t int_status = {0};
    uint16_t fifo_count = 0xFFFFU;
    uint16_t fifo_lost = 0xFFFFU;
    uint32_t fifo_full = 0U;

    if (stats == NULL)
    {
        return;
    }

    status = icm53611_fifo_get_count(&fifo_count);
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
    }

    status = icm53611_read_int_status(&int_status);
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
        int_status.int_status = 0xFFU;
    }
    else if ((int_status.int_status & ICM53611_INT_STATUS_FIFO_FULL_INT) != 0U)
    {
        fifo_full = 1U;
        stats->fifo_full_count++;
    }

    status = icm53611_fifo_get_lost_count(&fifo_lost);
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
    }
    else
    {
        stats->fifo_lost_pkt_count = fifo_lost;
    }

    ZY100_DIAG_LOG("[FIFO_BAD_DIAG] fifo_count_after_bad=%u int_status_after_bad=0x%02x fifo_lost=%u fifo_full=%u",
               fifo_count,
               int_status.int_status,
               (uint32_t)fifo_lost,
               fifo_full);
    ZY100_DIAG_LOG("[FIFO_BAD_SUM] lost=%u full=%u bad_hdr=%u",
               (uint32_t)stats->fifo_lost_pkt_count,
               stats->fifo_full_count,
               stats->bad_header_count);
}

static void v0_log_fifo_bad_context(
    const zy100_spi_sched_drain_context_t *drain_ctx,
    uint16_t offset,
    uint8_t header,
    uint8_t int_level_after)
{
    uint16_t packet_index;
    uint8_t previous_header = 0xFFU;
    uint8_t next_header = 0xFFU;
    uint8_t headers[8];
    uint8_t i;

    if (drain_ctx == NULL)
    {
        return;
    }

    packet_index = offset / V0_PACKET_SIZE_BYTES;
    if (offset >= V0_PACKET_SIZE_BYTES)
    {
        previous_header = s_v0_fifo_buf[offset - V0_PACKET_SIZE_BYTES];
    }
    if ((uint32_t)offset + V0_PACKET_SIZE_BYTES < drain_ctx->actual_len)
    {
        next_header = s_v0_fifo_buf[offset + V0_PACKET_SIZE_BYTES];
    }
    for (i = 0U; i < 8U; i++)
    {
        uint16_t header_offset = (uint16_t)i * V0_PACKET_SIZE_BYTES;
        headers[i] = (header_offset < drain_ctx->actual_len) ?
                     s_v0_fifo_buf[header_offset] : 0xFFU;
    }

    DBG_DIRECT("[FIFO_BAD_CTX] drain_seq=%u packet_index_in_drain=%u offset=%u header=%02x req_len=%u actual_len=%u drain_src=%s cached_count_valid=%u cached_count=%u int_level_before=%u int_level_after=%u queue_level=%u flash_capacity_packets=%u previous_header=%02x next_header=%02x first_8_headers=%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x",
               drain_ctx->seq,
               packet_index,
               offset,
               header,
               drain_ctx->req_len,
               drain_ctx->actual_len,
               v0_fifo_drain_src_name(drain_ctx->src),
               drain_ctx->cached_count_valid ? 1U : 0U,
               drain_ctx->cached_count,
               drain_ctx->int_level,
               int_level_after,
               drain_ctx->queue_level,
               drain_ctx->flash_capacity_packets,
               previous_header,
               next_header,
               headers[0],
               headers[1],
               headers[2],
               headers[3],
               headers[4],
               headers[5],
               headers[6],
               headers[7]);
}

static bool v0_handle_bad_fifo_packet(
    v0_fifo_stats_t *stats,
    const zy100_spi_sched_drain_context_t *drain_ctx,
    uint16_t offset,
    uint8_t header,
    uint8_t int_level_after)
{
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    if (zy100_online_raw_capture_active())
    {
        v0_online_abort_diag_latch_bad_fifo(drain_ctx,
                                            offset,
                                            header,
                                            int_level_after);
    }
#endif
    if (v0_online_tolerate_bad_fifo_packet(stats,
                                           drain_ctx,
                                           offset,
                                           header,
                                           int_level_after))
    {
        return true;
    }

#if ZY100_LEGACY_OFFLINE_ENABLE
    if (s_v0_offline_fifo_diag.active)
    {
        v0_offline_fifo_stage_t stage = V0_OFFLINE_FIFO_STAGE_NORMAL;
        if ((drain_ctx != NULL) && drain_ctx->cached_count_valid)
        {
            if ((uint32_t)drain_ctx->cached_count >= ZY100_FIFO_EMERGENCY_BYTES)
            {
                stage = V0_OFFLINE_FIFO_STAGE_RESCUE;
            }
            else if ((uint32_t)drain_ctx->cached_count >=
                     ZY100_FIFO_WATERMARK_ALARM_BYTES)
            {
                stage = V0_OFFLINE_FIFO_STAGE_ALARM;
            }
        }
        v0_offline_fifo_latch_fault(
            V0_OFFLINE_FAULT_BAD_HEADER,
            stats,
            (drain_ctx != NULL) ? drain_ctx->cached_count : 0U,
            (drain_ctx != NULL) ? drain_ctx->req_len : 0U,
            (drain_ctx != NULL) ? drain_ctx->actual_len : 0U,
            header,
            stage);
        v0_stop_on_invalid_fifo_packet(stats, header);
        return false;
    }
#endif

    v0_log_fifo_bad_context(drain_ctx, offset, header, int_level_after);
    v0_log_fifo_drain_context_ring();
    v0_log_fifo_bad_diag(stats);

#if ZY100_FIFO_BAD_FRAME_DIAG_ONLY
    return true;
#else
    v0_stop_on_invalid_fifo_packet(stats, header);
    return false;
#endif
}

static void v0_prepare_fifo_drain_context(
    zy100_spi_sched_drain_context_t *dst,
    const zy100_spi_sched_drain_context_t *base,
    zy100_spi_sched_drain_src_t src,
    uint16_t req_len,
    uint16_t actual_len,
    bool cached_count_valid,
    uint16_t cached_count,
    uint32_t flash_capacity_packets,
    bool capped_by_flash_capacity)
{
    if (dst == NULL)
    {
        return;
    }

    if (base != NULL)
    {
        *dst = *base;
    }
    else
    {
        memset(dst, 0, sizeof(*dst));
        dst->seq = zy100_spi_sched_alloc_drain_seq();
        dst->src = src;
        dst->time_since_last_irq_ms = 0xFFFFFFFFU;
        dst->time_since_last_drain_ms = 0xFFFFFFFFU;
    }

    dst->req_len = req_len;
    dst->actual_len = actual_len;
    dst->cached_count_valid = cached_count_valid;
    dst->cached_count = cached_count;
    dst->int_level = imu_bsp_int_level();
    dst->flash_capacity_packets = flash_capacity_packets;
    dst->capped_by_flash_capacity = capped_by_flash_capacity;

#if V1_IMU_FLASH_CAPTURE_ENABLE
    if (!ZY100_RT_MARKER_TIMER_ONLY_TEST && !ZY100_RT_MARKER_FIFO_ONLY_TEST)
    {
        external_flash_capture_queue_status_t queue_status;

        memset(&queue_status, 0, sizeof(queue_status));
        external_flash_capture_get_queue_status(&queue_status);
        dst->queue_level = queue_status.queue_level;
        dst->fill_packets = queue_status.fill_packet_count;
    }
#endif
}

#if V0_FLASH_RUNTIME_ENABLE && !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE && 1
static void v0_log_fifo_emergency_drop(
    const zy100_spi_sched_drain_context_t *drain_ctx,
    uint16_t read_len,
    uint32_t packet_count,
    uint32_t capacity_packets)
{
    external_flash_capture_queue_status_t queue_status;

    memset(&queue_status, 0, sizeof(queue_status));
    external_flash_capture_get_queue_status(&queue_status);
    ZY100_DIAG_LOG("[FIFO_EMG] drop seq=%u read_len=%u packets=%u capacity_packets=%u q=%u fill_packets=%u",
               (drain_ctx != NULL) ? drain_ctx->seq : 0U,
               read_len,
               packet_count,
               capacity_packets,
               queue_status.queue_level,
               queue_status.fill_packet_count);
}
#endif

#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
#if !ZY100_FINAL_EDGE_MODE_ENABLE
static void v0_phase_a_update_ingress_stats(v0_fifo_stats_t *stats)
{
    uint32_t level;
    uint32_t free_bytes;
    uint32_t max_level;

    if (stats == NULL)
    {
        return;
    }

    level = zy100_fifo_ingress_level_bytes();
    free_bytes = zy100_fifo_ingress_free_bytes();
    max_level = zy100_fifo_ingress_max_level_bytes();
    if (max_level > stats->ingress_max_level)
    {
        stats->ingress_max_level = max_level;
    }
    if (level > stats->ingress_level_max)
    {
        stats->ingress_level_max = level;
    }
    if (free_bytes < stats->ingress_free_min)
    {
        stats->ingress_free_min = free_bytes;
    }
}

#if V0_FLASH_RUNTIME_ENABLE
static uint32_t v0_phase_a_min_u32(uint32_t a, uint32_t b)
{
    return (a < b) ? a : b;
}
#endif

static void v0_phase_a_add_u32(uint32_t *value, uint32_t delta)
{
    if ((value == NULL) || (delta == 0U))
    {
        return;
    }

    if ((0xFFFFFFFFU - *value) < delta)
    {
        *value = 0xFFFFFFFFU;
    }
    else
    {
        *value += delta;
    }
}
#endif

static uint16_t v0_phase_a_aligned_fifo_count(uint16_t fifo_count)
{
    return (uint16_t)(fifo_count & (uint16_t)~(V0_PACKET_SIZE_BYTES - 1U));
}

static uint16_t v0_phase_a_planned_drain_len(uint16_t fifo_count)
{
    uint16_t drain_len = v0_phase_a_aligned_fifo_count(fifo_count);

    if (drain_len > V0_FIFO_DRAIN_MAX_RUNTIME_BYTES)
    {
        drain_len = V0_FIFO_DRAIN_MAX_RUNTIME_BYTES;
    }
    if (drain_len < V0_FIFO_DRAIN_THRESHOLD_BYTES)
    {
        return 0U;
    }
    return drain_len;
}

#if !ZY100_FINAL_EDGE_MODE_ENABLE
static bool v0_offline_v2_mode_active(void)
{
#if ZY100_LEGACY_OFFLINE_ENABLE
    return zy100_offline_v2_capture_active();
#else
    return false;
#endif
}

#if ZY100_LEGACY_OFFLINE_ENABLE
static void v0_offline_complete_last_drain(uint16_t actual_read_len,
                                           uint32_t read_us,
                                           uint8_t first_header)
{
    uint8_t index;

    if (!s_v0_offline_fifo_diag.active ||
        (s_v0_fifo_drain_ctx_ring_count == 0U))
    {
        return;
    }
    index = (uint8_t)((s_v0_fifo_drain_ctx_ring_next +
                       V0_FIFO_DRAIN_CTX_RING_SIZE - 1U) %
                      V0_FIFO_DRAIN_CTX_RING_SIZE);
    s_v0_fifo_drain_ctx_ring[index].actual_read_len = actual_read_len;
    s_v0_fifo_drain_ctx_ring[index].read_us = read_us;
    s_v0_fifo_drain_ctx_ring[index].first_header = first_header;
    s_v0_fifo_drain_ctx_ring[index].completed = true;
    s_v0_offline_fifo_diag.drain_count++;
    s_v0_offline_fifo_diag.drain_bytes += actual_read_len;
    if (read_us > s_v0_offline_fifo_diag.max_read_us)
    {
        s_v0_offline_fifo_diag.max_read_us = read_us;
    }
}

static void v0_offline_set_last_post_count(uint16_t fifo_count)
{
    uint8_t index;

    if (!s_v0_offline_fifo_diag.active ||
        (s_v0_fifo_drain_ctx_ring_count == 0U))
    {
        return;
    }
    index = (uint8_t)((s_v0_fifo_drain_ctx_ring_next +
                       V0_FIFO_DRAIN_CTX_RING_SIZE - 1U) %
                      V0_FIFO_DRAIN_CTX_RING_SIZE);
    s_v0_fifo_drain_ctx_ring[index].post_count = fifo_count;
}
#endif

static uint32_t v0_fifo_poll_interval_ms(bool offline_v2_mode)
{
    return offline_v2_mode ? ZY100_OFFLINE_V2_FIFO_POLL_INTERVAL_MS :
           ZY100_FIFO_POLL_INTERVAL_MS;
}

static uint32_t v0_fifo_service_soft_ms(bool offline_v2_mode)
{
    return offline_v2_mode ? ZY100_OFFLINE_V2_FIFO_SERVICE_SOFT_MS :
           ZY100_FIFO_SERVICE_SOFT_DEADLINE_MS;
}

static uint32_t v0_fifo_service_hard_ms(bool offline_v2_mode)
{
    return offline_v2_mode ? ZY100_OFFLINE_V2_FIFO_SERVICE_HARD_MS :
           ZY100_FIFO_SERVICE_HARD_DEADLINE_MS;
}

static uint32_t v0_fifo_service_emergency_ms(bool offline_v2_mode)
{
    return offline_v2_mode ? ZY100_OFFLINE_V2_FIFO_SERVICE_HARD_MS :
           ZY100_FIFO_SERVICE_EMERGENCY_DEADLINE_MS;
}

static zy100_offline_trace_service_t v0_offline_service_state(
    bool due,
    bool overdue,
    bool emergency)
{
    if (emergency)
    {
        return ZY100_OFFLINE_TRACE_SERVICE_EMERGENCY;
    }
    if (overdue)
    {
        return ZY100_OFFLINE_TRACE_SERVICE_OVERDUE;
    }
    if (due)
    {
        return ZY100_OFFLINE_TRACE_SERVICE_DUE;
    }
    return ZY100_OFFLINE_TRACE_SERVICE_IDLE;
}

static void v0_offline_stall_diag(
    v0_fifo_stats_t *stats,
    const zy100_spi_sched_stats_t *sched,
    uint32_t now_ms,
    bool scheduler_urgent,
    bool service_due,
    bool service_overdue,
    bool service_emergency,
    uint32_t last_coop_yield_ms,
    uint32_t *last_progress_ms,
    uint32_t *last_packet_count,
    uint32_t *last_drain_count,
    uint8_t *stall_log_mask,
    uint8_t *trace_state)
{
    zy100_offline_trace_service_t service;
    zy100_offline_trace_imu_phase_t phase;
    uint32_t idle_ms;
    uint8_t packed_state;
    bool pressure;

    if ((stats == NULL) || (sched == NULL) || (last_progress_ms == NULL) ||
        (last_packet_count == NULL) || (last_drain_count == NULL) ||
        (stall_log_mask == NULL) || (trace_state == NULL))
    {
        return;
    }
    if ((stats->packet_count != *last_packet_count) ||
        (stats->fifo_drain_since_start != *last_drain_count))
    {
        *last_packet_count = stats->packet_count;
        *last_drain_count = stats->fifo_drain_since_start;
        *last_progress_ms = now_ms;
        *stall_log_mask = 0U;
    }
    idle_ms = now_ms - *last_progress_ms;
    service = v0_offline_service_state(service_due,
                                       service_overdue,
                                       service_emergency);
    pressure = scheduler_urgent || s_v0_high_water_alarm_pending ||
               s_v0_phase_a_rescue_pending ||
               (service != ZY100_OFFLINE_TRACE_SERVICE_IDLE);
    phase = (pressure &&
             (idle_ms >= ZY100_OFFLINE_V2_FIFO_SERVICE_SOFT_MS)) ?
            ZY100_OFFLINE_TRACE_IMU_STALL : ZY100_OFFLINE_TRACE_IMU_LIVE;
    if (phase == ZY100_OFFLINE_TRACE_IMU_LIVE)
    {
        service = ZY100_OFFLINE_TRACE_SERVICE_IDLE;
        scheduler_urgent = false;
    }
    packed_state = (uint8_t)((uint8_t)phase << 5);
    if (phase == ZY100_OFFLINE_TRACE_IMU_STALL)
    {
        packed_state |= (uint8_t)(((uint8_t)service << 3) |
                                  (scheduler_urgent ? 4U : 0U) |
                                  (s_v0_high_water_alarm_pending ? 2U : 0U) |
                                  (s_v0_phase_a_rescue_pending ? 1U : 0U));
    }
    if (packed_state != *trace_state)
    {
        *trace_state = packed_state;
        zy100_offline_reset_trace_set_imu_state(
            phase,
            scheduler_urgent,
            (phase == ZY100_OFFLINE_TRACE_IMU_STALL) &&
                s_v0_high_water_alarm_pending,
            (phase == ZY100_OFFLINE_TRACE_IMU_STALL) &&
                s_v0_phase_a_rescue_pending,
            service,
            sched->ms_since_last_fifo_drain);
    }
    if (!pressure)
    {
        return;
    }
    if ((idle_ms >= ZY100_OFFLINE_V2_FIFO_SERVICE_SOFT_MS) &&
        ((*stall_log_mask & 0x01U) == 0U))
    {
        *stall_log_mask |= 0x01U;
    }
    else if ((idle_ms >= ZY100_OFFLINE_V2_FIFO_SERVICE_HARD_MS) &&
             ((*stall_log_mask & 0x02U) == 0U))
    {
        *stall_log_mask |= 0x02U;
    }
    else if ((idle_ms >= ZY100_OFFLINE_V2_FIFO_SERVICE_HARD_MS) &&
             ((*stall_log_mask & 0x04U) == 0U))
    {
        *stall_log_mask |= 0x04U;
    }
    else
    {
        return;
    }
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][IMU_STALL_A] idle=%lu svc=%u urg=%u alarm=%u rescue=%u",
        (unsigned long)idle_ms,
        (uint32_t)service,
        scheduler_urgent ? 1U : 0U,
        s_v0_high_water_alarm_pending ? 1U : 0U,
        s_v0_phase_a_rescue_pending ? 1U : 0U);
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][IMU_STALL_B] pkt=%lu drain=%lu fifo=%lu age=%lu read=%lu",
        (unsigned long)stats->packet_count,
        (unsigned long)stats->fifo_drain_since_start,
        (unsigned long)sched->max_fifo_count,
        (unsigned long)sched->ms_since_last_fifo_drain,
        (unsigned long)stats->read_error_count);
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][IMU_STALL_C] latch=%lu confirm=%lu noop=%lu yield_age=%lu",
        (unsigned long)sched->fifo_drain_latched,
        (unsigned long)sched->confirmed_drain_pending,
        (unsigned long)sched->no_op_count,
        (unsigned long)(now_ms - last_coop_yield_ms));
}
#endif

#if ZY100_LEGACY_OFFLINE_ENABLE
static void v0_capture_offline_v2_final_fifo_status(v0_fifo_stats_t *stats)
{
    icm53611_int_status_t int_status = {0};
    uint16_t lost_raw = 0U;
    uint16_t fifo_count = 0U;
    imu_status_t status;

    if ((stats == NULL) || !s_v0_offline_fifo_diag.active)
    {
        return;
    }
    s_v0_offline_fifo_diag.final_gpio_level = imu_bsp_int_level();
    status = icm53611_read_int_status(&int_status);
    if (status == IMU_STATUS_OK)
    {
        s_v0_offline_fifo_diag.final_int_status = int_status.int_status;
        if ((int_status.int_status & ICM53611_INT_STATUS_FIFO_FULL_INT) != 0U)
        {
            stats->fifo_full_count++;
        }
    }
    else
    {
        s_v0_offline_fifo_diag.final_spi_status = (uint8_t)status;
        stats->read_error_count++;
    }
    status = icm53611_fifo_get_lost_count(&lost_raw);
    if (status == IMU_STATUS_OK)
    {
        s_v0_offline_fifo_diag.final_lost_raw = lost_raw;
        stats->fifo_lost_pkt_count = lost_raw;
    }
    else
    {
        s_v0_offline_fifo_diag.final_spi_status = (uint8_t)status;
        stats->read_error_count++;
    }
    status = icm53611_fifo_get_count(&fifo_count);
    if (status == IMU_STATUS_OK)
    {
        s_v0_offline_fifo_diag.final_fifo_count = fifo_count;
        if (fifo_count > s_v0_offline_fifo_diag.max_fifo_count)
        {
            s_v0_offline_fifo_diag.max_fifo_count = fifo_count;
        }
    }
    else
    {
        s_v0_offline_fifo_diag.final_spi_status = (uint8_t)status;
        stats->read_error_count++;
    }
    v0_offline_fifo_resolve_post_stop_fault(stats);
    if ((s_v0_offline_fifo_diag.first_fault == V0_OFFLINE_FAULT_FIFO_FULL) ||
        (s_v0_offline_fifo_diag.first_fault == V0_OFFLINE_FAULT_FIFO_LOST))
    {
        stats->fatal_error = true;
        stats->fatal_status = IMU_STATUS_NOT_READY;
        s_v0_stop_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL;
    }
}

static void v0_log_offline_v2_end_diag(v0_fifo_stats_t *stats,
                                       uint32_t runtime_ms)
{
    zy100_offline_v2_capture_diag_t diag;
    uint32_t fault_age_ms = 0U;
    uint8_t i;
    uint8_t start;

    if ((stats == NULL) || s_v0_offline_exit_logged ||
        !s_v0_offline_fifo_diag.active)
    {
        return;
    }
    s_v0_offline_exit_logged = true;
    memset(&diag, 0, sizeof(diag));
    (void)zy100_offline_v2_capture_get_diag(&diag);
    if (s_v0_offline_fifo_diag.first_valid)
    {
        fault_age_ms = zy100_os_time_ms() -
                       s_v0_offline_fifo_diag.first_time_ms;
    }
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][FIFO_END_A] reason=%u first=%u pkt=%lu runtime=%lu",
        (uint32_t)s_v0_stop_reason,
        (uint32_t)s_v0_offline_fifo_diag.first_fault,
        (unsigned long)stats->packet_count,
        (unsigned long)runtime_ms);
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][FIFO_END_B] bad=%lu full=%lu lost=%u read=%lu ingress=%lu",
        (unsigned long)stats->bad_header_count,
        (unsigned long)stats->fifo_full_count,
        (uint32_t)stats->fifo_lost_pkt_count,
        (unsigned long)stats->read_error_count,
        (unsigned long)zy100_fifo_ingress_overflow_count());
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][FIFO_END_C] cur=%u max=%u alarm=%lu rescue=%lu",
        s_v0_offline_fifo_diag.final_fifo_count,
        s_v0_offline_fifo_diag.max_fifo_count,
        (unsigned long)s_v0_offline_fifo_diag.alarm_enter_count,
        (unsigned long)s_v0_offline_fifo_diag.rescue_enter_count);
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][FIFO_FLOW_A] drain=%lu bytes=%lu irq=%lu poll=%lu",
        (unsigned long)s_v0_offline_fifo_diag.drain_count,
        (unsigned long)s_v0_offline_fifo_diag.drain_bytes,
        (unsigned long)s_v0_offline_fifo_diag.irq_service_count,
        (unsigned long)s_v0_offline_fifo_diag.poll_service_count);
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][FIFO_FLOW_B] burst_max=%lu read_max_us=%lu loop_max_ms=%lu",
        (unsigned long)s_v0_offline_fifo_diag.max_burst_reads,
        (unsigned long)s_v0_offline_fifo_diag.max_read_us,
        (unsigned long)s_v0_offline_fifo_diag.max_loop_gap_ms);
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][FIFO_FLOW_C] yields=%lu recover=%lu imu=%lu mag=%lu",
        (unsigned long)s_v0_offline_fifo_diag.yield_count,
        (unsigned long)s_v0_offline_fifo_diag.pressure_recover_count,
        (unsigned long)diag.imu_sample_count,
        (unsigned long)diag.mag_sample_count);
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][FIFO_FAULT_A] seq=%lu count=%u age=%lu stage=%u",
        (unsigned long)s_v0_offline_fifo_diag.first_packet_seq,
        s_v0_offline_fifo_diag.first_fifo_count,
        (unsigned long)fault_age_ms,
        (uint32_t)s_v0_offline_fifo_diag.first_stage);
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][FIFO_FAULT_B] hdr=0x%02x req=%u actual=%u src=%u",
        s_v0_offline_fifo_diag.first_header,
        s_v0_offline_fifo_diag.first_req_len,
        s_v0_offline_fifo_diag.first_actual_len,
        (uint32_t)s_v0_offline_fifo_diag.first_fault);
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][FIFO_FAULT_C] int=0x%02x lost_raw=%u gpio=%u spi=%u",
        s_v0_offline_fifo_diag.final_int_status,
        s_v0_offline_fifo_diag.final_lost_raw,
        s_v0_offline_fifo_diag.final_gpio_level,
        s_v0_offline_fifo_diag.final_spi_status);

    start = (uint8_t)((s_v0_fifo_drain_ctx_ring_next +
                       V0_FIFO_DRAIN_CTX_RING_SIZE -
                       s_v0_fifo_drain_ctx_ring_count) %
                      V0_FIFO_DRAIN_CTX_RING_SIZE);
    for (i = 0U; i < s_v0_fifo_drain_ctx_ring_count; i++)
    {
        uint8_t index = (uint8_t)((start + i) %
                                  V0_FIFO_DRAIN_CTX_RING_SIZE);
        const v0_fifo_drain_diag_entry_t *entry =
            &s_v0_fifo_drain_ctx_ring[index];
        if (!entry->valid)
        {
            continue;
        }
        ZY100_OFFLINE_V2_LOG(
            "[OFFLINE_V2][FIFO_HIST_A] i=%u seq=%lu pre=%u post=%u age=%lu",
            (uint32_t)i,
            (unsigned long)entry->ctx.seq,
            entry->ctx.cached_count,
            entry->post_count,
            (unsigned long)entry->ctx.time_since_last_drain_ms);
        ZY100_OFFLINE_V2_LOG(
            "[OFFLINE_V2][FIFO_HIST_B] i=%u req=%u actual=%u us=%lu hdr=0x%02x",
            (uint32_t)i,
            entry->ctx.req_len,
            entry->actual_read_len,
            (unsigned long)entry->read_us,
            entry->first_header);
    }
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][IMU_EXIT_A] packet=%u algo=%u fatal=%u stop=%u",
        s_v0_last_packet_result,
        (uint32_t)diag.algo_status,
        (uint32_t)stats->fatal_status,
        (uint32_t)s_v0_stop_reason);
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][IMU_EXIT_B] q=%lu qmax=%lu drop=%lu",
        (unsigned long)diag.event_queue_level,
        (unsigned long)diag.event_queue_max_level,
        (unsigned long)diag.feature_drop_count);
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][IMU_EXIT_C] fifo=%u age=%lu stage=%u",
        s_v0_offline_fifo_diag.final_fifo_count,
        (unsigned long)fault_age_ms,
        (uint32_t)s_v0_offline_fifo_diag.first_stage);
}

static void v0_stop_on_offline_v2_error(
    v0_fifo_stats_t *stats,
    v0_packet_process_result_t packet_result)
{
    if (stats != NULL)
    {
        stats->fatal_error = true;
        stats->fatal_status = IMU_STATUS_NOT_READY;
    }
    s_v0_last_packet_result = (uint32_t)packet_result;
    v0_offline_fifo_latch_fault(V0_OFFLINE_FAULT_ALGO,
                                stats,
                                stats != NULL ? stats->max_fifo_count : 0U,
                                0U,
                                0U,
                                0U,
                                V0_OFFLINE_FIFO_STAGE_NORMAL);
    v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL);
}
#endif

static bool v0_verify_ui_fifo_rate_config(v0_fifo_stats_t *stats)
{
    icm53611_a2_rate_regs_t regs;
    imu_status_t status;
    const uint8_t expected_gyro =
        (ICM53611_GYRO_FSR_2000DPS | ICM53611_GYRO_ODR_800HZ);
    const uint8_t expected_accel =
        (ICM53611_ACCEL_FSR_16G | ICM53611_ACCEL_ODR_800HZ);
    bool valid;

    status = icm53611_read_a2_rate_regs(&regs);
    valid = (status == IMU_STATUS_OK) &&
            ((regs.gyro_config0 & ICM53611_GYRO_CONFIG0_VERIFY_MASK) ==
             (expected_gyro & ICM53611_GYRO_CONFIG0_VERIFY_MASK)) &&
            ((regs.accel_config0 & ICM53611_ACCEL_CONFIG0_VERIFY_MASK) ==
             (expected_accel & ICM53611_ACCEL_CONFIG0_VERIFY_MASK)) &&
            ((regs.fdr_config &
              ICM53611_MREG1_FDR_CONFIG_FDR_SEL_MASK) == 0U);
    ZY100_OFFLINE_V2_LOG("[IMU_RATE] target_hz=%u status=%u gyro0=0x%02x accel0=0x%02x fdr=0x%02x valid=%u",
                        (uint32_t)ZY100_UI_FIFO_SAMPLE_RATE_HZ,
                        (uint32_t)status,
                        regs.gyro_config0,
                        regs.accel_config0,
                        regs.fdr_config,
                        valid ? 1U : 0U);
    if (!valid)
    {
        v0_mark_read_error(stats,
                           (status == IMU_STATUS_OK) ?
                           IMU_STATUS_VERIFY_FAILED : status);
        return false;
    }
    return true;
}

#if !ZY100_FINAL_EDGE_MODE_ENABLE
static void v0_phase_a_update_backpressure(v0_fifo_stats_t *stats)
{
    uint32_t level;
    uint32_t free_bytes;

    if (stats == NULL)
    {
        return;
    }

    level = zy100_fifo_ingress_level_bytes();
    free_bytes = zy100_fifo_ingress_free_bytes();
    v0_phase_a_update_ingress_stats(stats);

    if (stats->ingress_backpressure_pending)
    {
        if (level <= ZY100_FIFO_INGRESS_RECOVER_BYTES)
        {
            stats->ingress_backpressure_pending = false;
            stats->phase_a_ingress_emergency_attempts = 0U;
            v0_phase_a_add_u32(&stats->ingress_backpressure_recovered, 1U);
        }
        return;
    }

    if ((level >= ZY100_FIFO_INGRESS_BACKPRESSURE_BYTES) ||
        (free_bytes < ZY100_FIFO_INGRESS_CRITICAL_FREE_BYTES))
    {
        stats->ingress_backpressure_pending = true;
        v0_phase_a_add_u32(&stats->ingress_backpressure_enter, 1U);
    }
}
#endif

#if !ZY100_FINAL_EDGE_MODE_ENABLE
static bool v0_phase_a_flash_pump_once(v0_fifo_stats_t *stats)
{
#if V0_FLASH_RUNTIME_ENABLE
    external_flash_capture_queue_status_t before;
    external_flash_capture_queue_status_t after;
    bool pump_ok;

    memset(&before, 0, sizeof(before));
    memset(&after, 0, sizeof(after));
    external_flash_capture_get_queue_status(&before);
    if (stats != NULL)
    {
        v0_phase_a_add_u32(&stats->flash_pump_count, 1U);
    }
    pump_ok = external_flash_capture_pump_once_bounded();
    external_flash_capture_get_queue_status(&after);
    if ((stats != NULL) && (after.write_count > before.write_count))
    {
        v0_phase_a_add_u32(&stats->flash_block_done_count,
                           after.write_count - before.write_count);
    }
    return pump_ok;
#else
    IMU_UNUSED(stats);
    return true;
#endif
}
#endif

#if !ZY100_FINAL_EDGE_MODE_ENABLE
static void v0_phase_a_update_pressure_latches(uint16_t fifo_count)
{
    if ((uint32_t)fifo_count >= ZY100_FIFO_EMERGENCY_BYTES)
    {
        s_v0_phase_a_rescue_pending = true;
        s_v0_high_water_alarm_pending = true;
        return;
    }

    if (s_v0_phase_a_rescue_pending &&
        ((uint32_t)fifo_count < ZY100_FIFO_EMERGENCY_RESCUE_FLOOR_BYTES))
    {
        s_v0_phase_a_rescue_pending = false;
    }

    if ((uint32_t)fifo_count >= ZY100_FIFO_WATERMARK_ALARM_BYTES)
    {
        s_v0_high_water_alarm_pending = true;
    }
    else if (!s_v0_phase_a_rescue_pending)
    {
        s_v0_high_water_alarm_pending = false;
    }
}

static bool v0_phase_a_try_ingress_flush(v0_fifo_stats_t *stats, bool final_flush)
{
#if V0_FLASH_RUNTIME_ENABLE
    uint32_t level;
    uint32_t flush_bytes;
    uint32_t flush_packets;
    uint32_t capacity_packets;
    uint32_t final_tail_packets;
    uint32_t offset;

    if (stats == NULL)
    {
        return false;
    }
    if (!final_flush && s_v0_phase_a_ingress_flush_used_this_loop)
    {
        return false;
    }

    level = zy100_fifo_ingress_level_bytes();
    if (level < ZY100_FIFO_INGRESS_PACKET_BYTES)
    {
        return false;
    }
    if (!final_flush && (level < ZY100_FIFO_INGRESS_FLUSH_BYTES))
    {
        return false;
    }

    flush_bytes = final_flush ?
                  v0_phase_a_min_u32(level, ZY100_FIFO_INGRESS_FLUSH_BYTES) :
                  ZY100_FIFO_INGRESS_FLUSH_BYTES;
    flush_bytes &= ~(uint32_t)(ZY100_FIFO_INGRESS_PACKET_BYTES - 1U);
    if (flush_bytes == 0U)
    {
        return false;
    }
    flush_packets = flush_bytes / ZY100_FIFO_INGRESS_PACKET_BYTES;

    capacity_packets = external_flash_capture_accept_capacity_packets();
    if (capacity_packets < flush_packets)
    {
        final_tail_packets =
            external_flash_capture_final_tail_capacity_packets(flush_packets);
        if (final_tail_packets == 0U)
        {
            stats->ingress_flush_blocked_by_flash_capacity++;
            return false;
        }
        flush_packets = final_tail_packets;
        flush_bytes = flush_packets * ZY100_FIFO_INGRESS_PACKET_BYTES;
    }

    if (!zy100_fifo_ingress_peek_packets(s_v0_ingress_flush_buf, flush_packets))
    {
        return false;
    }

    for (offset = 0U; offset < flush_bytes;
         offset += ZY100_FIFO_INGRESS_PACKET_BYTES)
    {
        if (!external_flash_capture_append_packet(&s_v0_ingress_flush_buf[offset]))
        {
            stats->fatal_error = true;
            stats->fatal_status = IMU_STATUS_BUS_ERROR;
            v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL);
            return false;
        }
    }

    if (!zy100_fifo_ingress_drop_packets(flush_packets))
    {
        stats->fatal_error = true;
        stats->fatal_status = IMU_STATUS_BUS_ERROR;
        v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL);
        return false;
    }

    stats->ingress_flush_bytes += flush_bytes;
    v0_phase_a_update_ingress_stats(stats);
    v0_phase_a_update_backpressure(stats);
    if (!final_flush)
    {
        s_v0_phase_a_ingress_flush_used_this_loop = true;
    }
    if (external_flash_capture_is_full())
    {
        stats->reached_final_summary = true;
        v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FLASH_95_PERCENT);
    }
    return true;
#else
    IMU_UNUSED(stats);
    IMU_UNUSED(final_flush);
    return false;
#endif
}

#if !ZY100_FINAL_EDGE_MODE_ENABLE
static bool v0_phase_a_finish_ingress_tail_if_flash_full(
    v0_fifo_stats_t *stats,
    uint32_t retry_count) __attribute__((unused));
static bool v0_phase_a_finish_ingress_tail_if_flash_full(v0_fifo_stats_t *stats,
                                                         uint32_t retry_count)
{
#if V0_FLASH_RUNTIME_ENABLE
    uint32_t level;
    uint32_t packet_count;

    level = zy100_fifo_ingress_level_bytes();
    if (level == 0U)
    {
        return true;
    }
    if ((level % ZY100_FIFO_INGRESS_PACKET_BYTES) != 0U)
    {
        return false;
    }
    if (!external_flash_capture_is_full() ||
        external_flash_capture_has_error() ||
        external_flash_capture_has_pending_work() ||
        external_flash_capture_raw_queue_busy())
    {
        return false;
    }

    packet_count = level / ZY100_FIFO_INGRESS_PACKET_BYTES;
    if (!zy100_fifo_ingress_drop_packets(packet_count))
    {
        return false;
    }

    if (stats != NULL)
    {
        stats->reached_final_summary = true;
        stats->ingress_flush_blocked_by_flash_capacity++;
        v0_phase_a_update_ingress_stats(stats);
        v0_phase_a_update_backpressure(stats);
    }
    v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FLASH_95_PERCENT);
    DBG_DIRECT("[PHASE_A] final_ingress_tail_discarded_due_full bytes=%u packets=%u retry=%u",
               level,
               packet_count,
               retry_count);
    return true;
#else
    IMU_UNUSED(stats);
    IMU_UNUSED(retry_count);
    return false;
#endif
}
#endif

static bool v0_phase_a_service_backpressure_once(v0_fifo_stats_t *stats,
                                                 bool allow_flash_pump)
{
    bool progress = false;

    if (stats == NULL)
    {
        return false;
    }

    v0_phase_a_update_backpressure(stats);
    if (v0_phase_a_try_ingress_flush(stats, false))
    {
        v0_phase_a_add_u32(&stats->ingress_backpressure_flush, 1U);
        progress = true;
    }

#if V0_FLASH_RUNTIME_ENABLE
    if (allow_flash_pump && !stats->fatal_error)
    {
        v0_phase_a_add_u32(&stats->ingress_backpressure_pump, 1U);
        if (v0_phase_a_flash_pump_once(stats))
        {
            progress = true;
        }
    }
#else
    IMU_UNUSED(allow_flash_pump);
#endif

    v0_phase_a_update_backpressure(stats);
    return progress;
}
#endif

static bool v0_phase_a_ingress_can_accept_drain(uint16_t read_len)
{
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    if (zy100_online_raw_capture_active())
    {
        return (read_len != 0U) &&
               ((read_len % ZY100_FIFO_INGRESS_PACKET_BYTES) == 0U) &&
               ((uint32_t)read_len <= V0_FIFO_READ_BUFFER_BYTES);
    }
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE
    return (read_len != 0U) &&
           ((read_len % ZY100_FIFO_INGRESS_PACKET_BYTES) == 0U) &&
           ((uint32_t)read_len <= V0_FIFO_READ_BUFFER_BYTES);
#else
    if ((read_len == 0U) ||
        ((read_len % ZY100_FIFO_INGRESS_PACKET_BYTES) != 0U) ||
        ((uint32_t)read_len > V0_FIFO_DRAIN_MAX_RUNTIME_BYTES))
    {
        return false;
    }

    return zy100_fifo_ingress_free_bytes() >= (uint32_t)read_len;
#endif
}

#if ZY100_FINAL_EDGE_MODE_ENABLE
static void v0_phase_a_note_ingress_blocked(v0_fifo_stats_t *stats,
                                            uint16_t fifo_count,
                                            uint16_t read_len,
                                            bool high_pressure,
                                            bool emergency)
{
    IMU_UNUSED(stats);
    IMU_UNUSED(fifo_count);
    IMU_UNUSED(read_len);
    IMU_UNUSED(high_pressure);
    IMU_UNUSED(emergency);
}
#endif

#if !ZY100_FINAL_EDGE_MODE_ENABLE
static void v0_phase_a_clear_ingress_blocked(v0_fifo_stats_t *stats)
{
    if (stats == NULL)
    {
        return;
    }

    stats->phase_a_ingress_drain_blocked = false;
    stats->phase_a_ingress_blocked_packets = 0U;
    stats->phase_a_ingress_blocked_fifo_count = 0U;
    stats->phase_a_ingress_blocked_high_pressure = false;
    stats->phase_a_ingress_blocked_emergency = false;
}

static void v0_phase_a_fatal_ingress_full(v0_fifo_stats_t *stats)
{
    uint32_t write_count = 0U;

#if V1_IMU_FLASH_CAPTURE_ENABLE
    external_flash_capture_queue_status_t queue_status;

    memset(&queue_status, 0, sizeof(queue_status));
    external_flash_capture_get_queue_status(&queue_status);
    write_count = queue_status.write_count;
#endif
    if (stats != NULL)
    {
        stats->fatal_error = true;
        stats->fatal_status = IMU_STATUS_BUS_ERROR;
        v0_phase_a_add_u32(&stats->ingress_backpressure_fatal, 1U);
    }
    DBG_DIRECT("[INGRESS_ERR] ring_full level=%u free=%u fifo_irq=%u write_count=%u",
               zy100_fifo_ingress_level_bytes(),
               zy100_fifo_ingress_free_bytes(),
               s_v0_irq_count,
               write_count);
    v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL);
}

static void v0_phase_a_note_ingress_blocked(v0_fifo_stats_t *stats,
                                            uint16_t fifo_count,
                                            uint16_t read_len,
                                            bool high_pressure,
                                            bool emergency)
{
#if ZY100_FINAL_EDGE_MODE_ENABLE
    IMU_UNUSED(stats);
    IMU_UNUSED(fifo_count);
    IMU_UNUSED(read_len);
    IMU_UNUSED(high_pressure);
    IMU_UNUSED(emergency);
#else
    if (stats == NULL)
    {
        return;
    }
    stats->phase_a_ingress_drain_blocked = true;
    stats->phase_a_ingress_blocked_fifo_count = fifo_count;
    stats->phase_a_ingress_blocked_packets =
        read_len / ZY100_FIFO_INGRESS_PACKET_BYTES;
    stats->phase_a_ingress_blocked_high_pressure = high_pressure;
    stats->phase_a_ingress_blocked_emergency = emergency;
    v0_phase_a_update_backpressure(stats);
#endif
}

#if V0_FLASH_RUNTIME_ENABLE
static bool v0_phase_a_push_ingress_packet(v0_fifo_stats_t *stats,
                                           const uint8_t *packet)
{
    if (!zy100_fifo_ingress_push_packet(packet))
    {
        v0_phase_a_fatal_ingress_full(stats);
        return false;
    }
    v0_phase_a_update_ingress_stats(stats);
    v0_phase_a_update_backpressure(stats);
    return true;
}
#endif

static void v0_phase_a_handle_ingress_blocked(v0_fifo_stats_t *stats,
                                              bool fifo_safe_for_flash)
{
    bool high_pressure;
    bool emergency;
    bool allow_flash_pump;
    uint16_t blocked_packets;
    uint16_t blocked_read_len;

    if ((stats == NULL) || !stats->phase_a_ingress_drain_blocked)
    {
        return;
    }

    high_pressure = stats->phase_a_ingress_blocked_high_pressure;
    emergency = stats->phase_a_ingress_blocked_emergency;
    blocked_packets = stats->phase_a_ingress_blocked_packets;
    blocked_read_len =
        (uint16_t)(blocked_packets * ZY100_FIFO_INGRESS_PACKET_BYTES);
    allow_flash_pump =
        fifo_safe_for_flash || stats->ingress_backpressure_pending ||
        high_pressure || emergency;

    v0_phase_a_update_backpressure(stats);
    if ((blocked_read_len == 0U) ||
        v0_phase_a_ingress_can_accept_drain(blocked_read_len))
    {
        stats->phase_a_ingress_emergency_attempts = 0U;
        v0_phase_a_clear_ingress_blocked(stats);
        return;
    }

    if (emergency)
    {
        uint16_t fresh_fifo_count = 0U;
        uint16_t fresh_planned_drain_len;

        if (stats->phase_a_ingress_emergency_attempts <
            ZY100_FIFO_INGRESS_EMERGENCY_PUMP_ATTEMPTS)
        {
            v0_phase_a_add_u32(&stats->phase_a_ingress_emergency_attempts, 1U);
            IMU_UNUSED(v0_phase_a_service_backpressure_once(stats, true));
        }

        if (!v0_read_fifo_count(stats, &fresh_fifo_count))
        {
            v0_phase_a_clear_ingress_blocked(stats);
            return;
        }

        v0_phase_a_update_pressure_latches(fresh_fifo_count);
        fresh_planned_drain_len = v0_phase_a_planned_drain_len(fresh_fifo_count);
        v0_phase_a_update_backpressure(stats);
        if (((uint32_t)fresh_fifo_count < ZY100_FIFO_EMERGENCY_BYTES) ||
            (fresh_planned_drain_len == 0U) ||
            v0_phase_a_ingress_can_accept_drain(fresh_planned_drain_len))
        {
            if ((uint32_t)fresh_fifo_count < ZY100_FIFO_EMERGENCY_BYTES)
            {
                stats->phase_a_ingress_emergency_attempts = 0U;
            }
            v0_phase_a_clear_ingress_blocked(stats);
            return;
        }

        if (stats->phase_a_ingress_emergency_attempts >=
            ZY100_FIFO_INGRESS_EMERGENCY_PUMP_ATTEMPTS)
        {
            v0_phase_a_fatal_ingress_full(stats);
        }
        v0_phase_a_clear_ingress_blocked(stats);
        return;
    }

    stats->phase_a_ingress_emergency_attempts = 0U;
    if (allow_flash_pump)
    {
        IMU_UNUSED(v0_phase_a_service_backpressure_once(stats, true));
    }
    v0_phase_a_clear_ingress_blocked(stats);
}
#endif


#if !ZY100_FINAL_EDGE_MODE_ENABLE
static bool v0_phase_a_flush_ingress_until_empty(v0_fifo_stats_t *stats)
    __attribute__((unused));
static bool v0_phase_a_flush_ingress_until_empty(v0_fifo_stats_t *stats)
{
    uint32_t start_ms = zy100_os_time_ms();
    uint32_t retry_count = 0U;

    while (zy100_fifo_ingress_level_bytes() != 0U)
    {
        if (v0_phase_a_try_ingress_flush(stats, true))
        {
            retry_count = 0U;
            continue;
        }

#if V0_FLASH_RUNTIME_ENABLE
        IMU_UNUSED(v0_phase_a_flash_pump_once(stats));
        if (v0_phase_a_finish_ingress_tail_if_flash_full(stats, retry_count))
        {
            return true;
        }
#endif
        retry_count++;
        if ((retry_count >= V0_PHASE_A_FINAL_FLUSH_MAX_RETRY) ||
            ((uint32_t)(zy100_os_time_ms() - start_ms) >=
             V0_PHASE_A_FINAL_FLUSH_TIMEOUT_MS))
        {
            if (stats != NULL)
            {
                stats->ingress_final_flush_fail_count++;
            }
            DBG_DIRECT("[ERR][PHASE_A] final_ingress_flush_timeout level=%u retry=%u",
                       zy100_fifo_ingress_level_bytes(),
                       retry_count);
            return false;
        }
    }
    return true;
}
#endif
#endif

#if V0_FLASH_RUNTIME_ENABLE
static bool v0_drain_fifo_packets_limited(v0_fifo_stats_t *stats, uint32_t max_bytes)
{
    imu_status_t status;
    uint32_t drained_bytes_this_round = 0U;
    bool drained = false;

    if (max_bytes < V0_PACKET_SIZE_BYTES)
    {
        return false;
    }

    while (drained_bytes_this_round < max_bytes)
    {
        uint16_t fifo_count = 0U;
        uint16_t read_len;
        uint16_t req_len;
        uint16_t offset;
        uint32_t budget_remaining;
        uint32_t capacity_packets = 0U;
        bool capped_by_flash_capacity = false;
        zy100_spi_sched_drain_context_t drain_ctx;
        uint8_t int_level_after;
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
        zy100_online_raw_fifo_time_snapshot_t raw_time_snapshot;
        uint32_t raw_accepted_before = 0U;
        bool raw_time_snapshot_valid = false;
        bool raw_burst_active = zy100_online_raw_capture_active();
#endif

#if V1_IMU_FLASH_CAPTURE_ENABLE
        if (s_v0_stop_requested &&
            (s_v0_stop_reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_KEY_SLEEP))
        {
            break;
        }
#endif

        if (!v0_read_fifo_count(stats, &fifo_count))
        {
            return drained;
        }

        if (fifo_count < V0_PACKET_SIZE_BYTES)
        {
            break;
        }

        read_len = (uint16_t)(fifo_count & (uint16_t)~(V0_PACKET_SIZE_BYTES - 1U));
        req_len = read_len;
        if (read_len > V0_FIFO_READ_BUFFER_BYTES)
        {
            read_len = V0_FIFO_READ_BUFFER_BYTES;
        }

#if V0_FLASH_RUNTIME_ENABLE && !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE && \
    1
        if (!ZY100_RT_MARKER_TIMER_ONLY_TEST && !ZY100_RT_MARKER_FIFO_ONLY_TEST)
        {
            uint32_t requested_packets;

            capacity_packets = external_flash_capture_accept_capacity_packets();
            requested_packets = read_len / V0_PACKET_SIZE_BYTES;
            if (capacity_packets < requested_packets)
            {
                if (external_flash_capture_is_full())
                {
                    stats->reached_final_summary = true;
                    v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FLASH_95_PERCENT);
                }
                break;
            }
        }
#endif

        budget_remaining = max_bytes - drained_bytes_this_round;
        if (read_len > budget_remaining)
        {
            read_len = (uint16_t)(budget_remaining & (uint32_t)~(V0_PACKET_SIZE_BYTES - 1U));
        }

#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
        read_len = v0_phase_a_planned_drain_len(read_len);
#else
        read_len = (uint16_t)(read_len & (uint16_t)~(V0_PACKET_SIZE_BYTES - 1U));
#endif
        if (read_len < V0_PACKET_SIZE_BYTES)
        {
            break;
        }

#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
        if (!v0_phase_a_ingress_can_accept_drain(read_len))
        {
            v0_phase_a_note_ingress_blocked(stats,
                                           fifo_count,
                                           read_len,
                                           true,
                                           true);
            break;
        }
#endif

        v0_prepare_fifo_drain_context(&drain_ctx,
                                      NULL,
                                      ZY100_SPI_SCHED_DRAIN_SRC_FINAL,
                                      req_len,
                                      read_len,
                                      true,
                                      fifo_count,
                                      capacity_packets,
                                      capped_by_flash_capacity);
        v0_store_fifo_drain_context(&drain_ctx);
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
        if (raw_burst_active)
        {
            raw_accepted_before = v0_online_raw_accepted_packets();
        }
#endif
        status = icm53611_fifo_read(s_v0_fifo_buf, read_len);
        if (status != IMU_STATUS_OK)
        {
            v0_mark_read_error(stats, status);
            return drained;
        }
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
        if (raw_burst_active)
        {
            raw_time_snapshot_valid =
                zy100_online_raw_capture_snapshot_fifo_read(
                    raw_accepted_before,
                    &raw_time_snapshot);
        }
#endif
        int_level_after = imu_bsp_int_level();

        stats->bytes_read += read_len;
        drained_bytes_this_round += read_len;
        drained = true;
        zy100_spi_sched_invalidate_fifo_count();
        for (offset = 0U; offset < read_len; offset = (uint16_t)(offset + V0_PACKET_SIZE_BYTES))
        {
#if V1_IMU_FLASH_CAPTURE_ENABLE
            if (s_v0_stop_requested &&
                (s_v0_stop_reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_KEY_SLEEP))
            {
                return drained;
            }
#endif
            v0_packet_process_result_t packet_result =
                v0_process_packet(stats,
                                  &s_v0_fifo_buf[offset],
                                  0U,
                                  0xFFFFU);
            if (packet_result != V0_PACKET_PROCESS_OK)
            {
#if ZY100_LEGACY_OFFLINE_ENABLE
                if (packet_result == V0_PACKET_PROCESS_OFFLINE_V2_ERROR)
                {
                    v0_stop_on_offline_v2_error(stats, packet_result);
                    return drained;
                }
#endif
                if (packet_result == V0_PACKET_PROCESS_ONLINE_RAW_ERROR)
                {
                    v0_stop_on_online_raw_error(stats);
                    return drained;
                }
                if (!v0_handle_bad_fifo_packet(stats,
                                               &drain_ctx,
                                               offset,
                                               s_v0_fifo_buf[offset],
                                               int_level_after))
                {
                    return drained;
                }
                continue;
            }
#if V0_FLASH_RUNTIME_ENABLE
#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
#if ZY100_LEGACY_OFFLINE_ENABLE
            if (!s_v0_offline_fifo_diag.active &&
                !v0_phase_a_push_ingress_packet(stats, &s_v0_fifo_buf[offset]))
#else
            if (!v0_phase_a_push_ingress_packet(stats, &s_v0_fifo_buf[offset]))
#endif
            {
                return drained;
            }
#else
            if (!ZY100_RT_MARKER_TIMER_ONLY_TEST &&
                !ZY100_RT_MARKER_FIFO_ONLY_TEST &&
                !external_flash_capture_append_packet(&s_v0_fifo_buf[offset]))
            {
                stats->fatal_error = true;
                stats->fatal_status = IMU_STATUS_BUS_ERROR;
                v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL);
                return drained;
            }
            if (!ZY100_RT_MARKER_TIMER_ONLY_TEST &&
                !ZY100_RT_MARKER_FIFO_ONLY_TEST &&
                external_flash_capture_is_full())
            {
                stats->reached_final_summary = true;
                v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FLASH_95_PERCENT);
                return drained;
            }
#endif
#endif
        }
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
        if (raw_burst_active &&
            !v0_online_raw_publish_fifo_burst(
                stats,
                raw_accepted_before,
                read_len / V0_PACKET_SIZE_BYTES,
                raw_time_snapshot_valid,
                &raw_time_snapshot))
        {
            return drained;
        }
#endif
    }

    return drained;
}
#endif

#if V0_FLASH_RUNTIME_ENABLE
static bool v0_drain_fifo_packets(v0_fifo_stats_t *stats)
{
#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
    return v0_drain_fifo_packets_limited(stats, V0_FIFO_DRAIN_MAX_RUNTIME_BYTES);
#else
    return v0_drain_fifo_packets_limited(stats, V0_FIFO_DRAIN_MAX_BYTES);
#endif
}
#endif

static bool v0_drain_fifo_packets_exact(v0_fifo_stats_t *stats,
                                        const zy100_spi_sched_drain_context_t *base_drain_ctx,
                                        uint16_t read_len,
                                        bool guarded_startup,
                                        uint8_t *first_header,
                                        uint16_t *actual_read_len,
                                        bool *capped_by_flash_capacity,
                                        bool *no_flash_capacity)
{
    imu_status_t status;
    uint16_t req_len = read_len;
    uint16_t offset;
    uint32_t capacity_packets = 0U;
    uint32_t fifo_read_start_us = 0U;
    uint32_t fifo_read_elapsed_us = 0U;
    bool local_capped_by_flash_capacity = false;
    zy100_spi_sched_drain_context_t drain_ctx;
    uint8_t int_level_after;
    bool emergency_drain =
        (base_drain_ctx != NULL) &&
        (base_drain_ctx->src == ZY100_SPI_SCHED_DRAIN_SRC_EMERGENCY);
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    zy100_online_raw_fifo_time_snapshot_t raw_time_snapshot;
    uint32_t raw_accepted_before = 0U;
    bool raw_time_snapshot_valid = false;
    bool raw_burst_active = zy100_online_raw_capture_active();
#endif
#if V0_FLASH_RUNTIME_ENABLE && !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE && 1
    uint32_t emergency_packet_count = 0U;
    uint32_t fill_packets_before = 0U;
    bool emergency_capacity_bypass = false;
    bool emergency_save_allowed = false;
    bool emergency_packets_valid = true;
    bool normal_cross_block_append = false;
#endif

    if (first_header != NULL)
    {
        *first_header = 0U;
    }
    if (actual_read_len != NULL)
    {
        *actual_read_len = 0U;
    }
    if (capped_by_flash_capacity != NULL)
    {
        *capped_by_flash_capacity = false;
    }
    if (no_flash_capacity != NULL)
    {
        *no_flash_capacity = false;
    }

    if (stats == NULL)
    {
        return false;
    }

    if (read_len > V0_FIFO_READ_BUFFER_BYTES)
    {
        read_len = V0_FIFO_READ_BUFFER_BYTES;
    }
    read_len = (uint16_t)(read_len & (uint16_t)~(V0_PACKET_SIZE_BYTES - 1U));
    if (read_len < V0_PACKET_SIZE_BYTES)
    {
        return false;
    }

#if V1_IMU_FLASH_CAPTURE_ENABLE && !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE && 1
    if (!ZY100_RT_MARKER_TIMER_ONLY_TEST && !ZY100_RT_MARKER_FIFO_ONLY_TEST)
    {
        uint32_t requested_packets = read_len / V0_PACKET_SIZE_BYTES;
        external_flash_capture_queue_status_t queue_status;

        capacity_packets = external_flash_capture_accept_capacity_packets();
        memset(&queue_status, 0, sizeof(queue_status));
        external_flash_capture_get_queue_status(&queue_status);
        fill_packets_before = queue_status.fill_packet_count;
        if (emergency_drain)
        {
            bool would_complete_block;
            bool no_fill_after_complete;

            emergency_packet_count = read_len / V0_PACKET_SIZE_BYTES;
            would_complete_block =
                ((queue_status.fill_packet_count + emergency_packet_count) >=
                 V1_PACKETS_PER_BLOCK);
            no_fill_after_complete =
                would_complete_block &&
                (queue_status.queue_level >=
                 (EXTERNAL_FLASH_CAPTURE_QUEUE_BLOCK_STATE_MAX - 1U));
            emergency_capacity_bypass =
                (capacity_packets < emergency_packet_count) ||
                no_fill_after_complete;
            emergency_save_allowed = !emergency_capacity_bypass;
        }
        else if (capacity_packets < requested_packets)
        {
            uint32_t final_tail_packets =
                external_flash_capture_final_tail_capacity_packets(requested_packets);

            if (final_tail_packets != 0U)
            {
                requested_packets = final_tail_packets;
                read_len = (uint16_t)(final_tail_packets * V0_PACKET_SIZE_BYTES);
                normal_cross_block_append =
                    ((fill_packets_before + requested_packets) >
                     V1_PACKETS_PER_BLOCK);
            }
            else
            {
                if (no_flash_capacity != NULL)
                {
                    *no_flash_capacity = true;
                }
                if (external_flash_capture_is_full())
                {
                    stats->reached_final_summary = true;
                    v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FLASH_95_PERCENT);
                }
                return false;
            }
        }
        else
        {
            normal_cross_block_append =
                ((fill_packets_before + requested_packets) >
                 V1_PACKETS_PER_BLOCK);
        }
    }
#endif

    read_len = (uint16_t)(read_len & (uint16_t)~(V0_PACKET_SIZE_BYTES - 1U));
    if (read_len < V0_PACKET_SIZE_BYTES)
    {
        if (no_flash_capacity != NULL)
        {
            *no_flash_capacity = true;
        }
        return false;
    }

#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
    if (!v0_phase_a_ingress_can_accept_drain(read_len))
    {
        bool high_pressure_drain =
            emergency_drain ||
            ((base_drain_ctx != NULL) &&
             base_drain_ctx->cached_count_valid &&
             ((uint32_t)base_drain_ctx->cached_count >=
              ZY100_FIFO_WATERMARK_ALARM_BYTES));

        v0_phase_a_note_ingress_blocked(
            stats,
            (base_drain_ctx != NULL) ? base_drain_ctx->cached_count : read_len,
            read_len,
            high_pressure_drain,
            emergency_drain);
        if (no_flash_capacity != NULL)
        {
            *no_flash_capacity = true;
        }
        return false;
    }
#endif

    v0_prepare_fifo_drain_context(&drain_ctx,
                                  base_drain_ctx,
                                  ZY100_SPI_SCHED_DRAIN_SRC_COUNT_BASED,
                                  req_len,
                                  read_len,
                                  (base_drain_ctx != NULL) ?
                                      base_drain_ctx->cached_count_valid : false,
                                  (base_drain_ctx != NULL) ?
                                      base_drain_ctx->cached_count : 0U,
                                  capacity_packets,
                                  local_capped_by_flash_capacity);
    v0_store_fifo_drain_context(&drain_ctx);
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    if (raw_burst_active)
    {
        raw_accepted_before = v0_online_raw_accepted_packets();
    }
#endif
    fifo_read_start_us = (uint32_t)imu_bsp_local_timestamp_us();
    status = icm53611_fifo_read(s_v0_fifo_buf, read_len);
    fifo_read_elapsed_us =
        (uint32_t)((uint32_t)imu_bsp_local_timestamp_us() -
                   fifo_read_start_us);
#if !ZY100_LEGACY_OFFLINE_ENABLE
    IMU_UNUSED(fifo_read_elapsed_us);
#endif
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
        return false;
    }
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    if (raw_burst_active)
    {
        raw_time_snapshot_valid =
            zy100_online_raw_capture_snapshot_fifo_read(
                raw_accepted_before,
                &raw_time_snapshot);
    }
#endif
    int_level_after = imu_bsp_int_level();
    if (first_header != NULL)
    {
        *first_header = s_v0_fifo_buf[0];
    }
#if ZY100_LEGACY_OFFLINE_ENABLE
    v0_offline_complete_last_drain(read_len,
                                   fifo_read_elapsed_us,
                                   s_v0_fifo_buf[0]);
#endif

    stats->bytes_read += read_len;
    if (actual_read_len != NULL)
    {
        *actual_read_len = read_len;
    }
#if V1_IMU_FLASH_CAPTURE_ENABLE && !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE && 1
    if (emergency_drain &&
        !ZY100_RT_MARKER_TIMER_ONLY_TEST &&
        !ZY100_RT_MARKER_FIFO_ONLY_TEST)
    {
        emergency_packet_count = read_len / V0_PACKET_SIZE_BYTES;
        zy100_spi_sched_note_fifo_emergency_drain_result(
            (read_len >= V0_FIFO_WATERMARK_BYTES),
            true,
            false,
            false,
            emergency_capacity_bypass);
    }
#endif
    if (guarded_startup)
    {
        for (offset = 0U; offset < read_len; offset = (uint16_t)(offset + V0_PACKET_SIZE_BYTES))
        {
            if (s_v0_fifo_buf[offset] == 0xFFU)
            {
                if (first_header != NULL)
                {
                    *first_header = 0xFFU;
                }
                return false;
            }
        }
    }

    for (offset = 0U; offset < read_len; offset = (uint16_t)(offset + V0_PACKET_SIZE_BYTES))
    {
#if V1_IMU_FLASH_CAPTURE_ENABLE
        if (s_v0_stop_requested &&
            (s_v0_stop_reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_KEY_SLEEP))
        {
            return true;
        }
#endif
        v0_packet_process_result_t packet_result =
            v0_process_packet(stats,
                              &s_v0_fifo_buf[offset],
                              0U,
                              offset);
        if (packet_result != V0_PACKET_PROCESS_OK)
        {
#if V1_IMU_FLASH_CAPTURE_ENABLE && !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE && 1
            emergency_packets_valid = false;
#endif
#if ZY100_LEGACY_OFFLINE_ENABLE
            if (packet_result == V0_PACKET_PROCESS_OFFLINE_V2_ERROR)
            {
                v0_stop_on_offline_v2_error(stats, packet_result);
                return true;
            }
#endif
            if (packet_result == V0_PACKET_PROCESS_ONLINE_RAW_ERROR)
            {
                v0_stop_on_online_raw_error(stats);
                return true;
            }
            if (!v0_handle_bad_fifo_packet(stats,
                                           &drain_ctx,
                                           offset,
                                           s_v0_fifo_buf[offset],
                                           int_level_after))
            {
                return true;
            }
            continue;
        }
#if V0_FLASH_RUNTIME_ENABLE
#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
#if ZY100_LEGACY_OFFLINE_ENABLE
        if (!s_v0_offline_fifo_diag.active &&
            !v0_phase_a_push_ingress_packet(stats, &s_v0_fifo_buf[offset]))
#else
        if (!v0_phase_a_push_ingress_packet(stats, &s_v0_fifo_buf[offset]))
#endif
        {
            return true;
        }
#else
        if (emergency_drain &&
            !ZY100_RT_MARKER_TIMER_ONLY_TEST &&
            !ZY100_RT_MARKER_FIFO_ONLY_TEST)
        {
            continue;
        }
        if (!ZY100_RT_MARKER_TIMER_ONLY_TEST &&
            !ZY100_RT_MARKER_FIFO_ONLY_TEST &&
            !external_flash_capture_append_packet(&s_v0_fifo_buf[offset]))
        {
            stats->fatal_error = true;
            stats->fatal_status = IMU_STATUS_BUS_ERROR;
            v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL);
            return true;
        }
        if (!ZY100_RT_MARKER_TIMER_ONLY_TEST &&
            !ZY100_RT_MARKER_FIFO_ONLY_TEST &&
            external_flash_capture_is_full())
        {
            if (normal_cross_block_append)
            {
                zy100_spi_sched_note_fifo_drain_cross_block_append();
            }
            stats->reached_final_summary = true;
            v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FLASH_95_PERCENT);
            return true;
        }
#endif
#endif
    }

#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    if (raw_burst_active &&
        !v0_online_raw_publish_fifo_burst(
            stats,
            raw_accepted_before,
            read_len / V0_PACKET_SIZE_BYTES,
            raw_time_snapshot_valid,
            &raw_time_snapshot))
    {
        return true;
    }
#endif

#if V0_FLASH_RUNTIME_ENABLE && !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE && 1
    if (emergency_drain &&
        !ZY100_RT_MARKER_TIMER_ONLY_TEST &&
        !ZY100_RT_MARKER_FIFO_ONLY_TEST)
    {
        if (!emergency_save_allowed || !emergency_packets_valid)
        {
            v0_log_fifo_emergency_drop(&drain_ctx,
                                       read_len,
                                       emergency_packet_count,
                                       capacity_packets);
            zy100_spi_sched_note_fifo_emergency_drain_result(false,
                                                             false,
                                                             true,
                                                             false,
                                                             false);
            if (external_flash_capture_is_full())
            {
                stats->reached_final_summary = true;
                v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FLASH_95_PERCENT);
            }
            return true;
        }

        for (offset = 0U; offset < read_len; offset = (uint16_t)(offset + V0_PACKET_SIZE_BYTES))
        {
            if (!external_flash_capture_append_packet(&s_v0_fifo_buf[offset]))
            {
                stats->fatal_error = true;
                stats->fatal_status = IMU_STATUS_BUS_ERROR;
                v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL);
                return true;
            }
            if (external_flash_capture_is_full())
            {
                stats->reached_final_summary = true;
                v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FLASH_95_PERCENT);
                break;
            }
        }
        zy100_spi_sched_note_fifo_emergency_drain_result(false,
                                                         false,
                                                         false,
                                                         true,
                                                         false);
    }
    else if (normal_cross_block_append)
    {
        zy100_spi_sched_note_fifo_drain_cross_block_append();
    }
#else
    IMU_UNUSED(local_capped_by_flash_capacity);
#endif

    return true;
}

#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
static bool v0_online_read_first_packet_bounded(v0_fifo_stats_t *stats)
{
    uint32_t start_ms = zy100_os_time_ms();

    while ((uint32_t)(zy100_os_time_ms() - start_ms) <
           ZY100_SPI_SCHED_START_GUARD_MS)
    {
        imu_status_t status;
        uint16_t fifo_count = 0U;

        status = icm53611_fifo_get_count(&fifo_count);
        if (status != IMU_STATUS_OK)
        {
            v0_mark_read_error(stats, status);
            return false;
        }
        if (fifo_count >= V0_PACKET_SIZE_BYTES)
        {
            uint16_t actual_read_len = 0U;
            uint8_t first_header = 0U;
            bool capped = false;
            bool no_capacity = false;
            zy100_online_raw_capture_stats_t raw_stats;
            bool drained;

            drained = v0_drain_fifo_packets_exact(
                stats,
                NULL,
                V0_PACKET_SIZE_BYTES,
                true,
                &first_header,
                &actual_read_len,
                &capped,
                &no_capacity);
            memset(&raw_stats, 0, sizeof(raw_stats));
            zy100_online_raw_capture_get_stats(&raw_stats);
            if (drained && !stats->fatal_error &&
                (actual_read_len == V0_PACKET_SIZE_BYTES) &&
                (raw_stats.accepted_packets == 1U))
            {
                return true;
            }
            IMU_UNUSED(first_header);
            IMU_UNUSED(capped);
            IMU_UNUSED(no_capacity);
            if (!stats->fatal_error)
            {
                v0_stop_on_online_raw_error(stats);
            }
            return false;
        }
        os_delay(1U);
    }

    v0_mark_read_error(stats, IMU_STATUS_TIMEOUT);
    v0_stop_on_online_raw_error(stats);
    return false;
}
#endif

#if ZY100_FINAL_EDGE_MODE_ENABLE
static bool v0_final_edge_drain_fifo_tail_once(v0_fifo_stats_t *stats,
                                               bool *fifo_empty,
                                               uint16_t *fifo_before_out,
                                               uint16_t *fifo_after_out,
                                               uint16_t *drained_out,
                                               uint32_t *edge_out)
{
    imu_status_t status;
    zy100_spi_sched_drain_context_t drain_ctx;
    uint16_t fifo_before = 0U;
    uint16_t fifo_after = 0U;
    uint16_t read_len;
    uint16_t offset;
    uint8_t int_level_after;
    uint32_t edge_count = 0U;
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    zy100_online_raw_fifo_time_snapshot_t raw_time_snapshot;
    uint32_t raw_accepted_before = 0U;
    bool raw_time_snapshot_valid = false;
    bool raw_burst_active = zy100_online_raw_capture_active();
#endif

    if (fifo_empty != NULL)
    {
        *fifo_empty = false;
    }
    if (fifo_before_out != NULL)
    {
        *fifo_before_out = 0U;
    }
    if (fifo_after_out != NULL)
    {
        *fifo_after_out = 0U;
    }
    if (drained_out != NULL)
    {
        *drained_out = 0U;
    }
    if (edge_out != NULL)
    {
        *edge_out = 0U;
    }

    if ((stats == NULL) || stats->fatal_error)
    {
        return false;
    }

    status = icm53611_fifo_get_count(&fifo_before);
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
        return false;
    }
    if (fifo_before_out != NULL)
    {
        *fifo_before_out = fifo_before;
    }
    if (fifo_before == 0U)
    {
        if (fifo_after_out != NULL)
        {
            *fifo_after_out = 0U;
        }
        if (fifo_empty != NULL)
        {
            *fifo_empty = true;
        }
        return true;
    }
    if (fifo_before < V0_PACKET_SIZE_BYTES)
    {
        if (fifo_after_out != NULL)
        {
            *fifo_after_out = fifo_before;
        }
        return true;
    }

    read_len = (uint16_t)(fifo_before &
                          (uint16_t)~(V0_PACKET_SIZE_BYTES - 1U));
    if (read_len > ZY100_FINAL_EDGE_LIVE_FIFO_DRAIN_MAX_BYTES)
    {
        read_len = ZY100_FINAL_EDGE_LIVE_FIFO_DRAIN_MAX_BYTES;
    }
    if (read_len > V0_FIFO_READ_BUFFER_BYTES)
    {
        read_len = V0_FIFO_READ_BUFFER_BYTES;
    }
    read_len = (uint16_t)(read_len &
                          (uint16_t)~(V0_PACKET_SIZE_BYTES - 1U));
    if (read_len < V0_PACKET_SIZE_BYTES)
    {
        if (fifo_after_out != NULL)
        {
            *fifo_after_out = fifo_before;
        }
        return true;
    }

    memset(&drain_ctx, 0, sizeof(drain_ctx));
    drain_ctx.seq = zy100_spi_sched_alloc_drain_seq();
    drain_ctx.src = ZY100_SPI_SCHED_DRAIN_SRC_FINAL;
    drain_ctx.req_len = read_len;
    drain_ctx.actual_len = read_len;
    drain_ctx.cached_count_valid = true;
    drain_ctx.cached_count = fifo_before;
    drain_ctx.fifo_irq_pending = false;
    drain_ctx.irq_pending_count = s_v0_imu_fifo_pending_count;
    drain_ctx.int_level = imu_bsp_int_level();
    v0_store_fifo_drain_context(&drain_ctx);

#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    if (raw_burst_active)
    {
        raw_accepted_before = v0_online_raw_accepted_packets();
    }
#endif
    status = icm53611_fifo_read(s_v0_fifo_buf, read_len);
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
        return false;
    }
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    if (raw_burst_active)
    {
        raw_time_snapshot_valid =
            zy100_online_raw_capture_snapshot_fifo_read(
                raw_accepted_before,
                &raw_time_snapshot);
    }
#endif
    int_level_after = imu_bsp_int_level();

    stats->bytes_read += read_len;
    for (offset = 0U; offset < read_len;
         offset = (uint16_t)(offset + V0_PACKET_SIZE_BYTES))
    {
        if (!v0_final_edge_process_fifo_tail_packet(
                stats, &s_v0_fifo_buf[offset], &edge_count))
        {
            if (v0_header_is_valid(s_v0_fifo_buf[offset]) &&
                zy100_online_raw_capture_active())
            {
                v0_stop_on_online_raw_error(stats);
                return false;
            }
            if (!v0_handle_bad_fifo_packet(stats,
                                           &drain_ctx,
                                           offset,
                                           s_v0_fifo_buf[offset],
                                           int_level_after))
            {
                return false;
            }
        }
    }

#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    if (raw_burst_active &&
        !v0_online_raw_publish_fifo_burst(
            stats,
            raw_accepted_before,
            read_len / V0_PACKET_SIZE_BYTES,
            raw_time_snapshot_valid,
            &raw_time_snapshot))
    {
        return false;
    }
#endif

    s_final_edge_phase1_stats.drain_count++;
    s_final_edge_phase1_stats.drain_packet_sum +=
        read_len / V0_PACKET_SIZE_BYTES;
    if ((uint32_t)(read_len / V0_PACKET_SIZE_BYTES) >
        s_final_edge_phase1_stats.drain_packet_max)
    {
        s_final_edge_phase1_stats.drain_packet_max =
            read_len / V0_PACKET_SIZE_BYTES;
    }
    v0_final_edge_note_drain_len(read_len);
    stats->irq_drain_count++;
    stats->poll_rescue_drain_count++;
    stats->last_fifo_drain_ms = zy100_os_time_ms();
    if (stats->fifo_drain_since_start < 0xFFFFFFFFU)
    {
        stats->fifo_drain_since_start++;
    }
    zy100_spi_sched_invalidate_fifo_count();

    status = icm53611_fifo_get_count(&fifo_after);
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
        return false;
    }
    if (fifo_after_out != NULL)
    {
        *fifo_after_out = fifo_after;
    }
    if (drained_out != NULL)
    {
        *drained_out = read_len;
    }
    if (edge_out != NULL)
    {
        *edge_out = edge_count;
    }
    if (fifo_empty != NULL)
    {
        *fifo_empty = (fifo_after == 0U);
    }

#if ZY100_LOG_FE_DIAG_VERBOSE
    ZY100_DIAG_LOG("[FE_DRAIN_FIFO] before=%u after=%u drained=%u edge=%u rt=0",
               (uint32_t)fifo_before,
               (uint32_t)fifo_after,
               (uint32_t)read_len,
               edge_count);
#endif
    return true;
}

static bool v0_final_edge_region_reached_watermark(uint32_t used,
                                                   uint32_t total)
{
    uint64_t threshold;

    if (total == 0U)
    {
        return false;
    }
    threshold = ((uint64_t)total *
                 (uint64_t)ZY100_FINAL_EDGE_FLASH_STOP_WATERMARK_PCT) /
                100ULL;
    return (uint64_t)used >= threshold;
}

static void v0_final_edge_watermark_stop(zy100_fe_stop_reason_t reason,
                                         uint32_t region,
                                         uint32_t used,
                                         uint32_t total)
{
    uint32_t pct = (total == 0U) ? 0U :
                   (uint32_t)(((uint64_t)used * 100ULL) /
                              (uint64_t)total);

    v0_final_edge_wm_note_stop(region, used, total, pct);
    v0_final_edge_request_stop(reason, "flash");
}

static bool v0_final_edge_check_flash_watermark_stop(void)
{
    zy100_fe_raw_store_stats_t raw_stats;
    zy100_fe_record_store_stats_t record_stats;
    uint32_t meta_used = 0U;

    if (s_fe_stop_seen || s_v0_stop_requested)
    {
        return false;
    }

    memset(&raw_stats, 0, sizeof(raw_stats));
    memset(&record_stats, 0, sizeof(record_stats));
    zy100_final_edge_raw_store_get_stats(&raw_stats);
    zy100_final_edge_record_store_get_stats(&record_stats);
    meta_used = record_stats.meta_bytes;

    v0_final_edge_wm_note_check(raw_stats.used_bytes,
                                record_stats.summary_used_bytes,
                                record_stats.event_used_bytes,
                                meta_used);

    if (v0_final_edge_region_reached_watermark(
            raw_stats.used_bytes, ZY100_FINAL_EDGE_RAW_REGION_BYTES))
    {
        v0_final_edge_watermark_stop(ZY100_FE_STOP_REASON_FLASH_RAW_95,
                                     1U,
                                     raw_stats.used_bytes,
                                     ZY100_FINAL_EDGE_RAW_REGION_BYTES);
        return true;
    }
    if (v0_final_edge_region_reached_watermark(
            record_stats.summary_used_bytes,
            ZY100_FINAL_EDGE_SUMMARY_REGION_BYTES))
    {
        v0_final_edge_watermark_stop(ZY100_FE_STOP_REASON_FLASH_SUMMARY_95,
                                     2U,
                                     record_stats.summary_used_bytes,
                                     ZY100_FINAL_EDGE_SUMMARY_REGION_BYTES);
        return true;
    }
    if (v0_final_edge_region_reached_watermark(
            record_stats.event_used_bytes,
            ZY100_FINAL_EDGE_EVENT_REGION_BYTES))
    {
        v0_final_edge_watermark_stop(ZY100_FE_STOP_REASON_FLASH_EVENT_95,
                                     3U,
                                     record_stats.event_used_bytes,
                                     ZY100_FINAL_EDGE_EVENT_REGION_BYTES);
        return true;
    }
    if ((meta_used != 0U) &&
        v0_final_edge_region_reached_watermark(
            meta_used, ZY100_FINAL_EDGE_META_REGION_BYTES))
    {
        v0_final_edge_watermark_stop(ZY100_FE_STOP_REASON_FLASH_META_95,
                                     4U,
                                     meta_used,
                                     ZY100_FINAL_EDGE_META_REGION_BYTES);
        return true;
    }

    return false;
}

static void v0_final_edge_raw_store_pump_safe(void)
{
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    if (!zy100_final_edge_raw_store_has_pending_work())
    {
        return;
    }
    if (v0_online_quiet_logs())
    {
        zy100_online_stream_note_gate(ZY100_ONLINE_GATE_RAW_BUSY);
        return;
    }
    if (v0_final_edge_b_critical_active())
    {
        v0_final_edge_inc_stat(
            &s_final_edge_phase1_stats.b_critical_raw_blocked_count);
        zy100_online_stream_note_gate(ZY100_ONLINE_GATE_B_CRITICAL);
        return;
    }
    if (!s_final_edge_rt_deferred_active)
    {
        if (zy100_final_edge_record_store_program_in_flight())
        {
            zy100_final_edge_record_store_note_raw_wait_record_wip();
            zy100_online_stream_note_gate(ZY100_ONLINE_GATE_RECORD_BUSY);
            return;
        }
        {
            zy100_fe_raw_pump_result_t pump_result;

            pump_result = zy100_final_edge_raw_store_pump_once_ex(
                ZY100_FE_RAW_PUMP_CONTEXT_RUNTIME);
            if (pump_result == ZY100_FE_RAW_PUMP_RECORD_DONE)
            {
                zy100_fe_raw_store_stats_t raw_stats;
#if !ZY100_ONLINE_DIRECT_SPOOL_ENABLE
                uint32_t raw_addr;
                uint32_t record_bytes;
#endif

                memset(&raw_stats, 0, sizeof(raw_stats));
                zy100_final_edge_raw_store_get_stats(&raw_stats);
#if !ZY100_ONLINE_DIRECT_SPOOL_ENABLE
                record_bytes = raw_stats.last_page_count * 256U;
                raw_addr = ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR +
                           ((raw_stats.session_raw_first_bucket +
                             raw_stats.last_raw_id) *
                            ZY100_FINAL_EDGE_RAW_BUCKET_BYTES);
                if (record_bytes != 0U)
                {
                    (void)zy100_online_stream_queue_source_record(
                        ZY100_ONLINE_RECORD_RAW,
                        raw_stats.last_raw_id,
                        raw_addr,
                        record_bytes,
                        raw_stats.last_payload_bytes);
                }
#endif
                v0_final_edge_online_hit_log(
                    "raw_saved",
                    raw_stats.last_bscan_id,
                    v0_final_edge_online_raw_bucket_last(&raw_stats),
                    &raw_stats);
                if (zy100_online_stream_active())
                {
                    zy100_online_stream_note_raw_saved(
                        raw_stats.last_bscan_id);
                }
                IMU_UNUSED(v0_final_edge_check_flash_watermark_stop());
            }
            else if (pump_result == ZY100_FE_RAW_PUMP_ERROR)
            {
                zy100_fe_raw_store_stats_t raw_stats;

                memset(&raw_stats, 0, sizeof(raw_stats));
                zy100_final_edge_raw_store_get_stats(&raw_stats);
                zy100_online_stream_note_record_dropped_ex(
                    ZY100_ONLINE_RECORD_RAW,
                    ZY100_ONLINE_DROP_PUMP,
                    raw_stats.last_error_stage,
                    (zy100_fe_store_reserve_result_t)
                        raw_stats.target_last_reserve_result,
                    (uint8_t)s_fe_capture_state,
                    zy100_os_time_ms());
                v0_final_edge_raw_store_log_drop(&raw_stats);
            }
        }
    }
    else
    {
        zy100_online_stream_note_gate(ZY100_ONLINE_GATE_RT_DUE);
    }
#endif
}

static void v0_final_edge_raw_store_log_drop(
    const zy100_fe_raw_store_stats_t *raw_stats)
{
    if (raw_stats == NULL)
    {
        return;
    }
    if (v0_online_quiet_logs())
    {
        return;
    }
    v0_final_edge_online_hit_log(
        "raw_drop",
        raw_stats->last_bscan_id,
        v0_final_edge_online_raw_bucket_error(raw_stats),
        raw_stats);
    DBG_DIRECT("[ONL_E_RAW] st=%lu rv=%lu id=%lu wr=%lu vf=%lu",
               (unsigned long)raw_stats->last_error_stage,
               (unsigned long)raw_stats->last_error_status,
               (unsigned long)raw_stats->last_error_raw_id,
               (unsigned long)raw_stats->write_error,
               (unsigned long)raw_stats->verify_error);
}

#if ZY100_ONLINE_STREAM_ENABLE
static void v0_final_edge_online_abort_raw_pending(
    zy100_online_drop_reason_t reason)
{
    zy100_fe_raw_store_stats_t raw_stats;

    memset(&raw_stats, 0, sizeof(raw_stats));
    zy100_final_edge_raw_store_get_stats(&raw_stats);
    v0_final_edge_online_hit_log(
        "raw_abort",
        raw_stats.last_bscan_id,
        v0_final_edge_online_raw_bucket_last(&raw_stats),
        &raw_stats);
    zy100_online_stream_note_record_dropped_ex(
        ZY100_ONLINE_RECORD_RAW,
        reason,
        raw_stats.last_raw_id,
        (zy100_fe_store_reserve_result_t)
            raw_stats.target_last_reserve_result,
        (uint8_t)s_fe_capture_state,
        zy100_os_time_ms());
    if (reason == ZY100_ONLINE_DROP_TARGET_TIMEOUT)
    {
        DBG_DIRECT("[ONL_W_RAW] timeout id=%lu rv=%lu b=%lu",
                   (unsigned long)raw_stats.last_raw_id,
                   (unsigned long)raw_stats.target_last_reserve_result,
                   (unsigned long)raw_stats.last_bscan_id);
    }
    zy100_final_edge_raw_store_abort_pending();
}
#endif

static bool v0_final_edge_raw_store_final_pump(void)
{
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    uint32_t start_ms;
    uint32_t timeout_ms = ZY100_FINAL_EDGE_RAW_STORE_FINAL_PUMP_TIMEOUT_MS;
    uint32_t final_loop_last_us = 0U;
    bool online_active = false;

    if (!zy100_final_edge_raw_store_has_pending_work())
    {
        return true;
    }

#if ZY100_ONLINE_STREAM_ENABLE
    online_active = zy100_online_stream_active() ||
                    zy100_online_stream_end_wait_ack();
    if (online_active)
    {
        timeout_ms = zy100_final_edge_raw_store_waiting_target() ?
                     ZY100_ONLINE_RAW_TARGET_WAIT_TIMEOUT_MS :
                     ZY100_ONLINE_CAPTURE_PAUSE_RAW_PUMP_TIMEOUT_MS;
    }
#if !ZY100_ONLINE_DIRECT_SPOOL_ENABLE
    if (online_active &&
        (zy100_online_stream_capture_stop_reason() ==
         ZY100_ONLINE_STOP_REASON_HOST_STOP))
    {
        v0_final_edge_online_abort_raw_pending(ZY100_ONLINE_DROP_ABORT);
        return true;
    }
#endif
#endif
    start_ms = zy100_os_time_ms();
    zy100_final_edge_raw_store_set_finalizing(true);
    if (online_active)
    {
        v0_online_pause_set_phase(ZY100_ONLINE_PAUSE_PHASE_RAW_FINAL);
    }
    while (zy100_final_edge_raw_store_has_pending_work())
    {
        zy100_fe_raw_pump_result_t pump_result;
        uint32_t loop_start_us;
        uint32_t loop_gap_us;
        uint32_t loop_elapsed_us;

#if ZY100_ONLINE_STREAM_ENABLE
        if (online_active && s_v0_online_abort_requested)
        {
            zy100_final_edge_raw_store_abort_pending();
            break;
        }
#endif

        if (v0_elapsed_ms(start_ms) >= timeout_ms)
        {
#if ZY100_ONLINE_STREAM_ENABLE
            if (online_active)
            {
                s_v0_online_abort_requested = true;
                v0_online_pause_set_phase(ZY100_ONLINE_PAUSE_PHASE_ABORT);
                v0_final_edge_online_abort_raw_pending(
                    zy100_final_edge_raw_store_waiting_target() ?
                    ZY100_ONLINE_DROP_TARGET_TIMEOUT :
                    ZY100_ONLINE_DROP_ABORT);
            }
#endif
            break;
        }
        loop_start_us = (uint32_t)imu_bsp_local_timestamp_us();
        if (final_loop_last_us != 0U)
        {
            loop_gap_us = loop_start_us - final_loop_last_us;
            if (loop_gap_us >
                s_final_edge_phase1_stats.final_service_gap_max_us)
            {
                s_final_edge_phase1_stats.final_service_gap_max_us =
                    loop_gap_us;
            }
        }
#if ZY100_ONLINE_DIRECT_SPOOL_ENABLE
        if (zy100_final_edge_raw_store_waiting_target())
        {
            if (zy100_final_edge_record_store_has_active_record())
            {
                zy100_fe_record_pump_result_t record_result =
                    zy100_final_edge_record_store_pump_active_once();

                zy100_final_edge_record_store_note_raw_wait_record_wip();
                v0_online_pause_set_phase(
                    ZY100_ONLINE_PAUSE_PHASE_FINISH_ACTIVE_RECORD);
                if (record_result == ZY100_FE_REC_PUMP_ERROR)
                {
                    v0_final_edge_online_abort_raw_pending(
                        ZY100_ONLINE_DROP_ABORT);
                    break;
                }
                if ((record_result == ZY100_FE_REC_PUMP_PAGE_ISSUED) ||
                    (record_result == ZY100_FE_REC_PUMP_RECORD_DONE))
                {
                    v0_online_pause_note_progress();
                }
                v0_online_final_drain_coop_yield();
                final_loop_last_us = loop_start_us;
                continue;
            }
            if (zy100_online_spool_active_reservation_kind() != 0U)
            {
                zy100_online_spool_pump_result_t spool_result;
                uint32_t spool_progress_before;

                if (!zy100_online_spool_active_reservation_data_done())
                {
                    v0_final_edge_online_abort_raw_pending(
                        ZY100_ONLINE_DROP_ABORT);
                    break;
                }
                spool_progress_before = zy100_online_spool_progress_seq();
                spool_result = zy100_online_spool_commit_pump_once(true);
                if (spool_result == ZY100_ONLINE_SPOOL_PUMP_ERROR)
                {
                    v0_final_edge_online_abort_raw_pending(
                        ZY100_ONLINE_DROP_ABORT);
                    break;
                }
                if (zy100_online_spool_progress_seq() !=
                    spool_progress_before)
                {
                    v0_online_pause_note_progress();
                }
                v0_online_final_drain_coop_yield();
                final_loop_last_us = loop_start_us;
                continue;
            }
            {
                zy100_fe_store_reserve_result_t reserve_result =
                    zy100_final_edge_raw_store_retry_target();

                if (reserve_result == ZY100_FE_STORE_RESERVE_ERROR)
                {
                    v0_final_edge_online_abort_raw_pending(
                        ZY100_ONLINE_DROP_BEGIN);
                    break;
                }
                if (reserve_result != ZY100_FE_STORE_RESERVE_OK)
                {
                    if (reserve_result == ZY100_FE_STORE_RESERVE_FULL)
                    {
                        os_delay(1U);
                    }
                    else
                    {
                        v0_online_final_drain_coop_yield();
                    }
                    final_loop_last_us = loop_start_us;
                    continue;
                }
                v0_online_pause_set_phase(
                    ZY100_ONLINE_PAUSE_PHASE_RAW_FINAL);
                v0_online_pause_note_progress();
            }
        }
#endif
        if (zy100_final_edge_record_store_program_in_flight())
        {
            zy100_final_edge_record_store_note_raw_wait_record_wip();
            zy100_online_stream_note_gate(ZY100_ONLINE_GATE_RECORD_BUSY);
            v0_online_final_drain_coop_yield();
            final_loop_last_us = loop_start_us;
            continue;
        }
        pump_result = zy100_final_edge_raw_store_pump_once_ex(
            ZY100_FE_RAW_PUMP_CONTEXT_FINAL);
        if ((pump_result == ZY100_FE_RAW_PUMP_PAGE_ISSUED) ||
            (pump_result == ZY100_FE_RAW_PUMP_RECORD_DONE))
        {
            v0_online_pause_note_progress();
        }
        if (pump_result == ZY100_FE_RAW_PUMP_RECORD_DONE)
        {
            zy100_fe_raw_store_stats_t raw_stats;
#if !ZY100_ONLINE_DIRECT_SPOOL_ENABLE
            uint32_t raw_addr;
            uint32_t record_bytes;
#endif

            memset(&raw_stats, 0, sizeof(raw_stats));
            zy100_final_edge_raw_store_get_stats(&raw_stats);
#if !ZY100_ONLINE_DIRECT_SPOOL_ENABLE
            record_bytes = raw_stats.last_page_count * 256U;
            raw_addr = ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR +
                       ((raw_stats.session_raw_first_bucket +
                         raw_stats.last_raw_id) *
                        ZY100_FINAL_EDGE_RAW_BUCKET_BYTES);
            if (record_bytes != 0U)
            {
                (void)zy100_online_stream_queue_source_record(
                    ZY100_ONLINE_RECORD_RAW,
                    raw_stats.last_raw_id,
                    raw_addr,
                    record_bytes,
                    raw_stats.last_payload_bytes);
            }
#endif
            v0_final_edge_online_hit_log(
                "raw_saved",
                raw_stats.last_bscan_id,
                v0_final_edge_online_raw_bucket_last(&raw_stats),
                &raw_stats);
            if (zy100_online_stream_active() ||
                zy100_online_stream_end_wait_ack())
            {
                zy100_online_stream_note_raw_saved(raw_stats.last_bscan_id);
            }
        }
        if (pump_result == ZY100_FE_RAW_PUMP_ERROR)
        {
            zy100_fe_raw_store_stats_t raw_stats;

            memset(&raw_stats, 0, sizeof(raw_stats));
            zy100_final_edge_raw_store_get_stats(&raw_stats);
            zy100_online_stream_note_record_dropped_ex(
                ZY100_ONLINE_RECORD_RAW,
                ZY100_ONLINE_DROP_PUMP,
                raw_stats.last_error_stage,
                (zy100_fe_store_reserve_result_t)
                    raw_stats.target_last_reserve_result,
                (uint8_t)s_fe_capture_state,
                zy100_os_time_ms());
            v0_final_edge_raw_store_log_drop(&raw_stats);
            break;
        }
        v0_online_final_drain_coop_yield();
        loop_elapsed_us =
            (uint32_t)((uint32_t)imu_bsp_local_timestamp_us() -
                       loop_start_us);
        if (loop_elapsed_us >
            s_final_edge_phase1_stats.final_service_gap_max_us)
        {
            s_final_edge_phase1_stats.final_service_gap_max_us =
                loop_elapsed_us;
        }
        final_loop_last_us = loop_start_us;
    }
    zy100_final_edge_raw_store_set_finalizing(false);
    return !zy100_final_edge_raw_store_has_pending_work();
#else
    return true;
#endif
}

static void v0_final_edge_record_store_pump_safe(void)
{
#if ZY100_FINAL_EDGE_PHASE7_EDGE_FLASH_ENABLE && \
    ZY100_FINAL_EDGE_EDGE_STORE_RUNTIME_PUMP_ENABLE
    if (!zy100_final_edge_record_store_has_pending_work())
    {
        return;
    }
    if (v0_final_edge_b_critical_active())
    {
        v0_final_edge_inc_stat(
            &s_final_edge_phase1_stats.b_critical_record_blocked_count);
        zy100_online_stream_note_gate(ZY100_ONLINE_GATE_B_CRITICAL);
        return;
    }
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
    if (v0_final_edge_replay_needs_service())
    {
        zy100_final_edge_record_store_note_replay_blocked();
        zy100_online_stream_note_gate(ZY100_ONLINE_GATE_REPLAY);
        return;
    }
#endif
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    if (!zy100_final_edge_raw_store_is_idle())
    {
        zy100_final_edge_record_store_note_raw_busy_blocked();
        zy100_online_stream_note_gate(ZY100_ONLINE_GATE_RAW_BUSY);
        return;
    }
#endif
    if (s_final_edge_rt_deferred_active)
    {
        zy100_online_stream_note_gate(ZY100_ONLINE_GATE_RT_DUE);
        return;
    }

    {
        zy100_fe_record_pump_result_t pump_result;

        pump_result = zy100_final_edge_record_store_pump_once();
        if (pump_result == ZY100_FE_REC_PUMP_RECORD_DONE)
        {
            zy100_fe_record_store_stats_t record_stats;
#if !ZY100_ONLINE_DIRECT_SPOOL_ENABLE
            uint32_t record_bytes;
            uint32_t record_addr = 0U;
            zy100_online_record_type_t online_type =
                ZY100_ONLINE_RECORD_EVENT;
#endif

            memset(&record_stats, 0, sizeof(record_stats));
            zy100_final_edge_record_store_get_stats(&record_stats);
#if !ZY100_ONLINE_DIRECT_SPOOL_ENABLE
            record_bytes = record_stats.last_page_count * 256U;
            if (record_stats.last_type == ZY100_FE_REC_TYPE_SUMMARY)
            {
                online_type = ZY100_ONLINE_RECORD_SUMMARY;
                record_addr = record_stats.summary_next_addr - record_bytes;
            }
            else if (record_stats.last_type == ZY100_FE_REC_TYPE_EVENT)
            {
                online_type = ZY100_ONLINE_RECORD_EVENT;
                record_addr = record_stats.event_next_addr - record_bytes;
            }
            if ((record_bytes != 0U) && (record_addr != 0U))
            {
                (void)zy100_online_stream_queue_source_record(
                    online_type,
                    record_stats.last_seq,
                    record_addr,
                    record_bytes,
                    record_stats.last_payload_bytes);
            }
#endif
            IMU_UNUSED(v0_final_edge_check_flash_watermark_stop());
        }
    }
#endif
}

static bool v0_final_edge_record_store_final_pump(void)
{
#if ZY100_FINAL_EDGE_PHASE7_EDGE_FLASH_ENABLE
    uint32_t start_ms;
    uint32_t final_loop_last_us = 0U;
    bool timeout = false;
    uint32_t pending_left;

    if (!zy100_final_edge_record_store_has_pending_work())
    {
        zy100_final_edge_record_store_note_final_pump(0U, false, 0U);
        return true;
    }

    start_ms = zy100_os_time_ms();
    if (zy100_online_stream_active() ||
        zy100_online_stream_end_wait_ack())
    {
        v0_online_pause_set_phase(ZY100_ONLINE_PAUSE_PHASE_RECORD_FINAL);
    }
    while (zy100_final_edge_record_store_has_pending_work())
    {
        zy100_fe_record_pump_result_t pump_result;
        uint32_t loop_start_us;
        uint32_t loop_gap_us;
        uint32_t loop_elapsed_us;

#if ZY100_ONLINE_STREAM_ENABLE
        if (s_v0_online_abort_requested)
        {
            zy100_final_edge_record_store_abort_active();
            break;
        }
#endif

        if (v0_elapsed_ms(start_ms) >=
            ZY100_FINAL_EDGE_EDGE_STORE_FINAL_PUMP_TIMEOUT_MS)
        {
            timeout = true;
#if ZY100_ONLINE_STREAM_ENABLE
            if (zy100_online_stream_active() ||
                zy100_online_stream_end_wait_ack())
            {
                s_v0_online_abort_requested = true;
                zy100_final_edge_record_store_abort_active();
                v0_online_pause_set_phase(
                    ZY100_ONLINE_PAUSE_PHASE_ABORT);
                zy100_online_stream_request_abort_ex(
                    ZY100_ONLINE_STOP_REASON_INTERNAL,
                    ZY100_ONLINE_ABORT_ORIGIN_IMU_RAW_FINALIZE,
                    5U,
                    zy100_os_time_ms());
            }
#endif
            break;
        }

        loop_start_us = (uint32_t)imu_bsp_local_timestamp_us();
        if (final_loop_last_us != 0U)
        {
            loop_gap_us = loop_start_us - final_loop_last_us;
            if (loop_gap_us >
                s_final_edge_phase1_stats.final_service_gap_max_us)
            {
                s_final_edge_phase1_stats.final_service_gap_max_us =
                    loop_gap_us;
            }
        }

#if ZY100_ONLINE_DIRECT_SPOOL_ENABLE
        if (zy100_online_spool_commit_pending())
        {
            uint32_t spool_progress_before =
                zy100_online_spool_progress_seq();
            zy100_online_spool_pump_result_t spool_result =
                zy100_online_spool_commit_pump_once(true);

            if (spool_result == ZY100_ONLINE_SPOOL_PUMP_ERROR)
            {
                timeout = true;
                break;
            }
            if (zy100_online_spool_progress_seq() !=
                spool_progress_before)
            {
                v0_online_pause_note_progress();
            }
            v0_online_final_drain_coop_yield();
            final_loop_last_us = loop_start_us;
            continue;
        }
#endif
        pump_result = zy100_final_edge_record_store_pump_once();
        if ((pump_result == ZY100_FE_REC_PUMP_PAGE_ISSUED) ||
            (pump_result == ZY100_FE_REC_PUMP_RECORD_DONE))
        {
            v0_online_pause_note_progress();
        }
        if (pump_result == ZY100_FE_REC_PUMP_RECORD_DONE)
        {
#if !ZY100_ONLINE_DIRECT_SPOOL_ENABLE
            zy100_fe_record_store_stats_t record_stats;
            uint32_t record_bytes;
            uint32_t record_addr = 0U;
            zy100_online_record_type_t online_type =
                ZY100_ONLINE_RECORD_EVENT;

            memset(&record_stats, 0, sizeof(record_stats));
            zy100_final_edge_record_store_get_stats(&record_stats);
            record_bytes = record_stats.last_page_count * 256U;
            if (record_stats.last_type == ZY100_FE_REC_TYPE_SUMMARY)
            {
                online_type = ZY100_ONLINE_RECORD_SUMMARY;
                record_addr = record_stats.summary_next_addr - record_bytes;
            }
            else if (record_stats.last_type == ZY100_FE_REC_TYPE_EVENT)
            {
                online_type = ZY100_ONLINE_RECORD_EVENT;
                record_addr = record_stats.event_next_addr - record_bytes;
            }
            if ((record_bytes != 0U) && (record_addr != 0U))
            {
                (void)zy100_online_stream_queue_source_record(
                    online_type,
                    record_stats.last_seq,
                    record_addr,
                    record_bytes,
                    record_stats.last_payload_bytes);
            }
#endif
        }
        if (pump_result == ZY100_FE_REC_PUMP_ERROR)
        {
            break;
        }
        v0_online_final_drain_coop_yield();
        loop_elapsed_us =
            (uint32_t)((uint32_t)imu_bsp_local_timestamp_us() -
                       loop_start_us);
        if (loop_elapsed_us >
            s_final_edge_phase1_stats.final_service_gap_max_us)
        {
            s_final_edge_phase1_stats.final_service_gap_max_us =
                loop_elapsed_us;
        }
        zy100_final_edge_record_store_note_final_pump(loop_elapsed_us,
                                                      false,
                                                      0U);
        final_loop_last_us = loop_start_us;
    }

#if ZY100_ONLINE_STREAM_ENABLE
    if (timeout && !s_v0_online_abort_requested &&
        (zy100_online_stream_active() ||
         zy100_online_stream_end_wait_ack()))
    {
        s_v0_online_abort_requested = true;
        zy100_final_edge_record_store_abort_active();
        v0_online_pause_set_phase(ZY100_ONLINE_PAUSE_PHASE_ABORT);
        zy100_online_stream_request_abort_ex(
            ZY100_ONLINE_STOP_REASON_INTERNAL,
            ZY100_ONLINE_ABORT_ORIGIN_IMU_RAW_FINALIZE,
            2U,
            zy100_os_time_ms());
    }
#endif
    pending_left =
        zy100_final_edge_record_store_has_pending_work() ? 1U : 0U;
    zy100_final_edge_record_store_note_final_pump(0U,
                                                  timeout,
                                                  pending_left);
    return pending_left == 0U;
#else
    return true;
#endif
}

static bool v0_final_edge_write_session_meta(const v0_fifo_stats_t *stats,
                                             uint32_t capture_start_ms,
                                             uint32_t final_runtime_ms,
                                             const v0_final_edge_pass_diag_t *diag)
{
#if ZY100_FINAL_EDGE_PHASE7_EDGE_FLASH_ENABLE && \
    ZY100_FINAL_EDGE_SESSION_META_ENABLE
    zy100_fe_raw_store_stats_t raw_stats;
    zy100_fe_record_store_stats_t record_stats;
    zy100_final_edge_queue_stats_t queue_stats;
    zy100_fe_session_meta_input_t input;
    bool raw_ok;
    bool replay_ok;
    bool summary_ok;
    bool event_ok;
    bool store_ok;

    IMU_UNUSED(diag);
    if (stats == NULL)
    {
        return false;
    }

    memset(&raw_stats, 0, sizeof(raw_stats));
    memset(&record_stats, 0, sizeof(record_stats));
    memset(&queue_stats, 0, sizeof(queue_stats));
    memset(&input, 0, sizeof(input));
    zy100_final_edge_raw_store_get_stats(&raw_stats);
    zy100_final_edge_record_store_get_stats(&record_stats);
    zy100_final_edge_queue_get_stats(&queue_stats);

    raw_ok =
        ((s_final_edge_official_hit_count == 0U) ||
         ((raw_stats.raw_saved == s_final_edge_official_hit_count) &&
          (raw_stats.raw_begin == s_final_edge_official_hit_count))) &&
        (raw_stats.raw_failed == 0U) &&
        (raw_stats.raw_full == 0U) &&
        (raw_stats.write_error == 0U) &&
        (raw_stats.verify_error == 0U) &&
        (raw_stats.pending == 0U) &&
        (raw_stats.program_in_flight == 0U) &&
        (raw_stats.b_active_pump_blocked == 0U);
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
    replay_ok =
        (s_final_edge_phase1_stats.replay_fail_count == 0U) &&
        (s_final_edge_phase1_stats.replay_truehit_fail_count == 0U) &&
        (s_final_edge_phase1_stats.replay_ds_bad_count == 0U) &&
        (s_final_edge_phase1_stats.replay_ds_overflow_count == 0U) &&
        (s_final_edge_phase1_stats.replay_edge_error == 0U) &&
        (s_final_edge_phase1_stats.replay_truehit_edge_error == 0U) &&
        !s_final_edge_replay_pending &&
        !s_final_edge_replay_active &&
        !s_final_edge_replay_backlog_edge_only_active;
#else
    replay_ok = true;
#endif
    summary_ok =
        (record_stats.summary_saved == queue_stats.summary_enqueued) &&
        (record_stats.summary_failed == 0U) &&
        (record_stats.summary_full == 0U) &&
        (queue_stats.summary_dropped == 0U);
    event_ok =
        (record_stats.event_saved == queue_stats.event_enqueued) &&
        (record_stats.event_failed == 0U) &&
        (record_stats.event_full == 0U) &&
        (queue_stats.event_dropped == 0U);
    store_ok =
        summary_ok &&
        event_ok &&
        (record_stats.pending == 0U) &&
        (record_stats.program_in_flight == 0U) &&
        (record_stats.write_error == 0U) &&
        (record_stats.verify_error == 0U) &&
        (record_stats.b_active_pump_blocked == 0U) &&
        (record_stats.record_final_pump_timeout == 0U);

    input.round = stats->round;
    input.start_ms = capture_start_ms;
    input.end_ms = capture_start_ms + final_runtime_ms;
    input.official_hit_count = s_final_edge_official_hit_count;
    input.nohit_count = s_final_edge_official_nohit_count;
    input.raw_count = raw_stats.raw_saved;
    input.raw_used_bytes = raw_stats.used_bytes;
    if (raw_ok)
    {
        input.pass_flags |= ZY100_FE_SESSION_PASS_RAW;
    }
    if (replay_ok)
    {
        input.pass_flags |= ZY100_FE_SESSION_PASS_REPLAY;
    }
    if (summary_ok)
    {
        input.pass_flags |= ZY100_FE_SESSION_PASS_SUMMARY;
    }
    if (event_ok)
    {
        input.pass_flags |= ZY100_FE_SESSION_PASS_EVENT;
    }
    if (store_ok)
    {
        input.pass_flags |= ZY100_FE_SESSION_PASS_META |
                            ZY100_FE_SESSION_PASS_PHASE7;
    }
    if (queue_stats.summary_enqueued == 0U)
    {
        input.warn_flags |= ZY100_FE_SESSION_WARN_EMPTY_SUMMARY;
    }
    if (queue_stats.event_enqueued == 0U)
    {
        input.warn_flags |= ZY100_FE_SESSION_WARN_EMPTY_EVENT;
    }
    if (!raw_ok)
    {
        input.error_flags |= ZY100_FE_SESSION_ERROR_RAW;
    }
    if (!replay_ok)
    {
        input.error_flags |= ZY100_FE_SESSION_ERROR_REPLAY;
    }
    if ((queue_stats.summary_dropped != 0U) ||
        (queue_stats.event_dropped != 0U))
    {
        input.error_flags |= ZY100_FE_SESSION_ERROR_QUEUE_DROP;
    }
    if (!store_ok)
    {
        input.error_flags |= ZY100_FE_SESSION_ERROR_STORE;
    }
    zy100_capture_time_build_meta(input.rtc_meta);

#if ZY100_MULTI_SESSION_STORAGE_ENABLE
    if (!s_final_edge_active_session_valid)
    {
        return false;
    }
    if (zy100_online_stream_active())
    {
        uint32_t online_session_uid =
            s_final_edge_active_session_alloc.session_uid;
        bool online_final_stop = zy100_online_stream_end_requested();

        if (online_final_stop)
        {
            zy100_training_session_clear_active();
        }
        if ((online_session_uid != 0U) &&
            !zy100_session_dir_abort_active_empty(
                online_session_uid,
                (uint32_t)s_fe_stop_reason,
                final_runtime_ms))
        {
            return false;
        }
        if (online_final_stop)
        {
            s_final_edge_active_session_valid = false;
            memset(&s_final_edge_active_session_alloc,
                   0,
                   sizeof(s_final_edge_active_session_alloc));
        }
        s_final_edge_empty_error_aborted = true;
        s_fe_export_ready = false;
        s_fe_export_preserved = false;
        s_fe_capture_state = FE_CAPTURE_STATE_IDLE;
        if (!v0_online_quiet_logs())
        {
            DBG_DIRECT("[ONLINE_STATE] source_session_%s uid=%lu final_ms=%lu raw=%lu summary=%lu event=%lu",
                       online_final_stop ? "cleared" : "paused",
                       (unsigned long)online_session_uid,
                       (unsigned long)final_runtime_ms,
                       (unsigned long)raw_stats.raw_saved,
                       (unsigned long)record_stats.summary_saved,
                       (unsigned long)record_stats.event_saved);
        }
        return true;
    }
    if ((s_fe_stop_reason == ZY100_FE_STOP_REASON_ERROR) &&
        (input.raw_count == 0U) &&
        (record_stats.summary_saved == 0U) &&
        (record_stats.event_saved == 0U))
    {
        if (!zy100_session_dir_abort_active_empty(
                s_final_edge_active_session_alloc.session_uid,
                (uint32_t)s_fe_stop_reason,
                final_runtime_ms))
        {
            return false;
        }
        s_final_edge_active_session_valid = false;
        memset(&s_final_edge_active_session_alloc,
               0,
               sizeof(s_final_edge_active_session_alloc));
        s_final_edge_empty_error_aborted = true;
        s_fe_export_ready = false;
        s_fe_export_preserved = false;
        s_fe_capture_state = FE_CAPTURE_STATE_IDLE;
        return true;
    }
    zy100_training_session_set_counts(input.raw_count,
                                      record_stats.summary_saved,
                                      record_stats.event_saved);
    if (!zy100_training_session_write_end_record())
    {
        return false;
    }
    {
        zy100_session_commit_t commit;
        uint8_t training_end[ZY100_TRAINING_RECORD_BYTES];
        uint64_t rtc_start_ms = 0ULL;
        uint64_t rtc_stop_ms = 0ULL;
        uint8_t cal_start = 0U;
        uint8_t cal_stop = 0U;
        bool rtc_time_valid;
        bool from_cache = false;

        if (!zy100_training_session_get_end_export_record(training_end,
                                                          &from_cache))
        {
            return false;
        }
        rtc_time_valid =
            zy100_capture_time_decode_meta(input.rtc_meta,
                                           &rtc_start_ms,
                                           &rtc_stop_ms,
                                           &cal_start,
                                           &cal_stop,
                                           NULL) &&
            (rtc_start_ms != 0ULL) &&
            (rtc_stop_ms >= rtc_start_ms);
        memset(&commit, 0, sizeof(commit));
        commit.session_uid =
            s_final_edge_active_session_alloc.session_uid;
        commit.user_id = s_final_edge_active_session_alloc.user_id;
        commit.training_id = s_final_edge_active_session_alloc.training_id;
        commit.session_seq =
            (s_final_edge_active_session_alloc.session_seq != 0U) ?
            s_final_edge_active_session_alloc.session_seq : stats->round;
        commit.round =
            (s_final_edge_active_session_alloc.round != 0U) ?
            s_final_edge_active_session_alloc.round : stats->round;
        if (rtc_time_valid)
        {
            commit.start_time_ms = rtc_start_ms;
            commit.end_time_ms = rtc_stop_ms;
            commit.time_calibrated =
                ((cal_start != 0U) && (cal_stop != 0U)) ? 1U : 0U;
        }
        else
        {
            commit.start_time_ms =
                (s_final_edge_active_session_alloc.start_time_ms != 0ULL) ?
                s_final_edge_active_session_alloc.start_time_ms :
                (uint64_t)input.start_ms;
            commit.end_time_ms =
                commit.start_time_ms + (uint64_t)final_runtime_ms;
            commit.time_calibrated =
                s_final_edge_active_session_alloc.time_calibrated;
        }
        commit.source = s_final_edge_active_session_alloc.source;
        commit.stop_reason = (uint32_t)s_fe_stop_reason;
        commit.raw_marker_begin_addr =
            s_final_edge_active_session_alloc.raw_marker_begin_addr;
        commit.raw_data_begin_addr =
            s_final_edge_active_session_alloc.raw_data_begin_addr;
        commit.raw_first_bucket =
            s_final_edge_active_session_alloc.raw_first_bucket;
        commit.raw_limit_bucket =
            s_final_edge_active_session_alloc.raw_limit_bucket;
        commit.raw_bucket_count = raw_stats.raw_saved;
        commit.raw_used_bytes = raw_stats.used_bytes;
        commit.summary_marker_begin_addr =
            s_final_edge_active_session_alloc.summary_marker_begin_addr;
        commit.summary_data_begin_addr =
            s_final_edge_active_session_alloc.summary_data_begin_addr;
        commit.summary_base_addr =
            s_final_edge_active_session_alloc.summary_base_addr;
        commit.summary_limit_addr =
            s_final_edge_active_session_alloc.summary_limit_addr;
        commit.summary_count = record_stats.summary_saved;
        commit.summary_used_bytes = record_stats.summary_used_bytes;
        commit.event_marker_begin_addr =
            s_final_edge_active_session_alloc.event_marker_begin_addr;
        commit.event_data_begin_addr =
            s_final_edge_active_session_alloc.event_data_begin_addr;
        commit.event_base_addr =
            s_final_edge_active_session_alloc.event_base_addr;
        commit.event_limit_addr =
            s_final_edge_active_session_alloc.event_limit_addr;
        commit.event_count = record_stats.event_saved;
        commit.event_used_bytes = record_stats.event_used_bytes;
        commit.official_hit_count = input.official_hit_count;
        commit.nohit_count = input.nohit_count;
        commit.pass_flags = input.pass_flags;
        commit.warn_flags = input.warn_flags;
        commit.error_flags = input.error_flags;
        memcpy(commit.rtc_meta, input.rtc_meta, sizeof(commit.rtc_meta));
        if (!zy100_session_dir_append_commit(&commit, training_end))
        {
            return false;
        }
        s_final_edge_active_session_valid = false;
        (void)from_cache;
    }
#else
    if (!zy100_final_edge_record_store_write_session_meta(&input))
    {
        return false;
    }
    zy100_training_session_set_counts(input.raw_count,
                                      record_stats.summary_saved,
                                      record_stats.event_saved);
    (void)zy100_training_session_write_end_record();
#endif
    return true;
#else
    IMU_UNUSED(stats);
    IMU_UNUSED(capture_start_ms);
    IMU_UNUSED(final_runtime_ms);
    IMU_UNUSED(diag);
    return true;
#endif
}

static bool v0_final_edge_prepare_no_new_data_for_drain(v0_fifo_stats_t *stats)
{
    imu_status_t status;
    uint16_t fifo_before = 0U;
    uint32_t discard_notify = 0U;

    if (s_final_edge_no_new_data_mode)
    {
        return s_final_edge_imu_live_stopped_for_drain;
    }
    if ((stats == NULL) || stats->fatal_error)
    {
        return false;
    }
#if ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
    if (s_final_edge_b_scan_active)
    {
        return true;
    }
#endif
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
    if (v0_final_edge_replay_needs_service())
    {
        return true;
    }
#endif

    status = icm53611_fifo_get_count(&fifo_before);
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
        if (!v0_online_quiet_logs())
        {
            ZY100_DIAG_LOG("[FE_STOP_LIVE] timer=%u imu=%u fifo_before=%u status=%u",
                       0U,
                       0U,
                       0U,
                       (uint32_t)status);
        }
        return false;
    }

    s_final_edge_no_new_data_mode = true;
#if ZY100_FINAL_EDGE_TIMER_ENABLE
    v0_final_edge_timer_stop();
#else
    v0_final_edge_clear_timer_due();
#endif
    s_final_edge_live_timer_stopped_for_drain = true;
#if ZY100_RT_MARKER_TIMER_ENABLE
    zy100_rt_marker_timer_stop();
    zy100_rt_marker_timer_register_notify_callback(NULL);
#endif
    v0_final_edge_clear_timer_due();
    (void)xTaskNotifyWait(V0_NOTIFY_FE_TIMER,
                          V0_NOTIFY_FE_TIMER,
                          &discard_notify,
                          0U);
    s_v0_high_water_alarm_pending = false;
    s_v0_phase_a_rescue_pending = false;
    s_v0_imu_fifo_pending_count = 0U;
#if ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
    s_final_edge_rt_deferred_active = false;
    s_final_edge_rt_trigger_valid = false;
#endif

    status = icm53611_stop_ui_sensors_preserve_fifo();
    s_final_edge_imu_live_stopped_for_drain =
        (status == IMU_STATUS_OK) ? true : false;
    zy100_spi_sched_capture_stop();

    if (!v0_online_quiet_logs())
    {
            ZY100_DIAG_LOG("[FE_STOP_LIVE] timer=%u imu=%u fifo_before=%u status=%u",
                   s_final_edge_live_timer_stopped_for_drain ? 1U : 0U,
                   s_final_edge_imu_live_stopped_for_drain ? 1U : 0U,
                   (uint32_t)fifo_before,
                   (uint32_t)status);
        DBG_DIRECT("[FE_STOP_LIVE2] odr_stop=%u fifo_stop=%u err=%u",
                   s_final_edge_imu_live_stopped_for_drain ? 1U : 0U,
                   0U,
                   (status == IMU_STATUS_OK) ? 0U : (uint32_t)status);
    }

    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
        return false;
    }
    return true;
}

static bool v0_final_edge_final_drain(v0_fifo_stats_t *stats)
{
    uint32_t start_ms;
    uint32_t timeout_ms;
    uint32_t last_step_log_ms = 0U;
    uint32_t last_stuck_log_ms = 0U;
    uint32_t stuck_same_count = 0U;
    uint16_t stuck_last_fifo = 0xFFFFU;
    uint8_t last_b = 0xFFU;
    uint8_t last_replay = 0xFFU;
    uint8_t last_fifo_state = 0xFFU;
    uint8_t last_raw = 0xFFU;
    uint8_t last_rec = 0xFFU;
    bool step_log_valid = false;
    bool ok = true;
    bool online_stop = false;
    bool online_raw_deferred = false;

    if (stats == NULL)
    {
        return false;
    }

#if ZY100_ONLINE_STREAM_ENABLE
    online_stop = v0_final_edge_online_stop_active();
#endif
    timeout_ms = online_stop ?
                 (uint32_t)ZY100_ONLINE_CAPTURE_PAUSE_DRAIN_TIMEOUT_MS :
                 (uint32_t)ZY100_FINAL_EDGE_FINAL_DRAIN_TIMEOUT_MS;
    start_ms = zy100_os_time_ms();
    s_fe_final_drain_active = true;
    if (online_stop)
    {
        v0_online_pause_set_phase(
            ZY100_ONLINE_PAUSE_PHASE_DRAIN_FIFO_REPLAY);
    }
    if (!v0_online_quiet_logs())
    {
        ZY100_DIAG_LOG("[FE_DRAIN_START] reason=%u state=%u",
                   (uint32_t)s_fe_stop_reason,
                   (uint32_t)s_fe_capture_state);
    }

    while (!stats->fatal_error)
    {
        uint32_t now_ms = zy100_os_time_ms();
        uint32_t drain_elapsed_ms = (uint32_t)(now_ms - start_ms);
        uint16_t fifo_count = 0U;
        bool fifo_has_data = false;
        bool b_busy = false;
        bool replay_busy = false;
        bool raw_busy = false;
        bool rec_busy = false;
        bool post_restore_quiet = false;
        uint8_t b_flag;
        uint8_t replay_flag;
        uint8_t fifo_flag;
        uint8_t raw_flag;
        uint8_t rec_flag;

#if ZY100_ONLINE_STREAM_ENABLE
        if (online_stop && s_v0_online_abort_requested)
        {
            ok = true;
            break;
        }
#endif

        if (drain_elapsed_ms >= timeout_ms)
        {
            if (online_stop)
            {
                if (!v0_online_quiet_logs())
                {
                    DBG_DIRECT("[ONLINE_BLIND] drain_budget ms=%u raw=%u",
                               drain_elapsed_ms,
                               online_raw_deferred ? 1U : 0U);
                }
#if ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
                /* A healthy continuous-RAW END is allowed only after the
                 * frozen FIFO is proven empty. Other online modes retain the
                 * legacy bounded blind-stop behavior. */
                ok = !zy100_online_raw_capture_active();
#else
                ok = true;
#endif
            }
            else
            {
                ok = false;
            }
            break;
        }

#if ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
        b_busy = s_final_edge_b_scan_active;
        if (b_busy)
        {
            v0_final_edge_set_state(FE_CAPTURE_STATE_DRAIN_B_ACTIVE);
        }
        else if (s_final_edge_rt_deferred_active)
        {
            IMU_UNUSED(v0_final_edge_consume_deferred_b_scan(stats));
            b_busy = s_final_edge_b_scan_active;
        }
#endif
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
        replay_busy = v0_final_edge_replay_needs_service();
        if (replay_busy)
        {
            v0_final_edge_set_state(FE_CAPTURE_STATE_DRAIN_REPLAY);
        }
#endif
        if (!b_busy && !replay_busy && !s_final_edge_no_new_data_mode)
        {
            if (!v0_final_edge_prepare_no_new_data_for_drain(stats))
            {
                ok = false;
                break;
            }
        }
        if (s_final_edge_no_new_data_mode)
        {
            imu_status_t count_status = icm53611_fifo_get_count(&fifo_count);

            if (count_status != IMU_STATUS_OK)
            {
                v0_mark_read_error(stats, count_status);
                ok = false;
                break;
            }
            fifo_has_data = (fifo_count != 0U);
        }
        else if (v0_read_fifo_count(stats, &fifo_count) &&
                 (fifo_count >= V0_PACKET_SIZE_BYTES))
        {
            fifo_has_data = true;
        }
        if (fifo_has_data)
        {
            v0_final_edge_set_state(FE_CAPTURE_STATE_DRAIN_LIVE_FIFO);
        }
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
        raw_busy = zy100_final_edge_raw_store_has_pending_work();
        if (raw_busy)
        {
            v0_final_edge_set_state(FE_CAPTURE_STATE_DRAIN_RAW_STORE);
#if ZY100_ONLINE_STREAM_ENABLE
            if (online_stop)
            {
                online_raw_deferred = true;
                zy100_online_stream_note_gate(ZY100_ONLINE_GATE_RAW_BUSY);
                raw_busy = false;
            }
#endif
        }
#endif
        rec_busy = zy100_final_edge_record_store_has_pending_work();
#if ZY100_ONLINE_STREAM_ENABLE
        if (online_stop &&
            !zy100_final_edge_record_store_has_active_record())
        {
            rec_busy = false;
        }
#endif
        if (rec_busy)
        {
            v0_final_edge_set_state(FE_CAPTURE_STATE_DRAIN_RECORD_STORE);
        }
        post_restore_quiet = v0_final_edge_post_restore_quiet_active();

        b_flag = b_busy ? 1U : 0U;
        replay_flag = replay_busy ? 1U : 0U;
        fifo_flag = fifo_has_data ? 1U : 0U;
        raw_flag = raw_busy ? 1U : 0U;
        rec_flag = rec_busy ? 1U : 0U;
        if (!step_log_valid ||
            (b_flag != last_b) ||
            (replay_flag != last_replay) ||
            (fifo_flag != last_fifo_state) ||
            (raw_flag != last_raw) ||
            (rec_flag != last_rec) ||
            ((uint32_t)(now_ms - last_step_log_ms) >=
             (uint32_t)ZY100_FINAL_EDGE_DRAIN_STEP_LOG_PERIOD_MS))
        {
            if (ZY100_LOG_FE_DIAG_VERBOSE)
            {
                ZY100_DIAG_LOG("[FE_DRAIN_STEP] b=%u replay=%u fifo=%u raw=%u rec=%u meta=%u",
                           b_flag,
                           replay_flag,
                           (uint32_t)fifo_count,
                           raw_flag,
                           rec_flag,
                           0U);
            }
            step_log_valid = true;
            last_step_log_ms = now_ms;
            last_b = b_flag;
            last_replay = replay_flag;
            last_fifo_state = fifo_flag;
            last_raw = raw_flag;
            last_rec = rec_flag;
        }

        if (!b_busy && !replay_busy && !fifo_has_data && !raw_busy && !rec_busy)
        {
            break;
        }
        if (replay_busy ||
            (!s_final_edge_no_new_data_mode && fifo_has_data))
        {
            if (v0_final_edge_live_service_handler(stats, true))
            {
                v0_online_pause_note_progress();
            }
        }
        else if (s_final_edge_no_new_data_mode && fifo_has_data)
        {
            bool fifo_empty = false;
            uint16_t fifo_before = 0U;
            uint16_t fifo_after = 0U;
            uint16_t drained_len = 0U;
            uint32_t edge_count = 0U;

            if (!v0_final_edge_drain_fifo_tail_once(stats,
                                                    &fifo_empty,
                                                    &fifo_before,
                                                    &fifo_after,
                                                    &drained_len,
                                                    &edge_count))
            {
                ok = false;
                break;
            }
            IMU_UNUSED(fifo_empty);
            IMU_UNUSED(edge_count);
            if ((drained_len != 0U) && (fifo_after < fifo_before))
            {
                v0_online_pause_note_progress();
            }
            if ((drained_len == 0U) || (fifo_after >= fifo_before))
            {
                uint16_t stuck_fifo =
                    (fifo_after != 0U) ? fifo_after : fifo_before;

                if (stuck_fifo == stuck_last_fifo)
                {
                    if (stuck_same_count < 0xFFFFFFFFU)
                    {
                        stuck_same_count++;
                    }
                }
                else
                {
                    stuck_same_count = 1U;
                    stuck_last_fifo = stuck_fifo;
                }
                if ((last_stuck_log_ms == 0U) ||
                    ((uint32_t)(now_ms - last_stuck_log_ms) >=
                     (uint32_t)ZY100_FINAL_EDGE_DRAIN_STEP_LOG_PERIOD_MS))
                {
                    ZY100_DIAG_LOG("[FE_DRAIN_STUCK] fifo=%u last=%u same=%u ms=%u",
                               (uint32_t)stuck_fifo,
                               (uint32_t)stuck_last_fifo,
                               stuck_same_count,
                               (uint32_t)(now_ms - start_ms));
                    last_stuck_log_ms = now_ms;
                }
            }
            else
            {
                stuck_same_count = 0U;
                stuck_last_fifo = fifo_after;
            }
        }
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
        if (!post_restore_quiet)
        {
            v0_final_edge_raw_store_pump_safe();
        }
#endif
        if (!post_restore_quiet)
        {
#if ZY100_ONLINE_STREAM_ENABLE
            if (online_stop)
            {
                zy100_fe_record_pump_result_t record_result;

                v0_online_pause_set_phase(
                    ZY100_ONLINE_PAUSE_PHASE_FINISH_ACTIVE_RECORD);
                record_result =
                    zy100_final_edge_record_store_pump_active_once();
                if ((record_result == ZY100_FE_REC_PUMP_PAGE_ISSUED) ||
                    (record_result == ZY100_FE_REC_PUMP_RECORD_DONE))
                {
                    v0_online_pause_note_progress();
                }
                if (record_result == ZY100_FE_REC_PUMP_ERROR)
                {
                    ok = false;
                    break;
                }
            }
            else
#endif
            {
                v0_final_edge_record_store_pump_safe();
            }
        }
        if (online_stop)
        {
            os_delay(1U);
        }
        else
        {
            imu_bsp_delay_us(1000U);
        }
    }

    s_fe_final_drain_active = false;
    s_fe_final_drain_done = ok && !stats->fatal_error;
    if (!s_fe_final_drain_done)
    {
        v0_final_edge_set_state(FE_CAPTURE_STATE_ERROR);
        v0_final_edge_request_stop(ZY100_FE_STOP_REASON_ERROR, "drain");
        return false;
    }

    if (!v0_online_quiet_logs())
    {
        DBG_DIRECT("[FE_DRAIN_DONE] ms=%u raw=%u rec=%u meta=%u",
                   v0_elapsed_ms(start_ms),
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
                   zy100_final_edge_raw_store_has_pending_work() ? 0U : 1U,
#else
                   1U,
#endif
                   zy100_final_edge_record_store_has_pending_work() ? 0U : 1U,
                   0U);
    }
    else if (online_raw_deferred)
    {
        if (!v0_online_quiet_logs())
        {
            DBG_DIRECT("[ONLINE_BLIND] drain_done raw_defer=1 ms=%u",
                       v0_elapsed_ms(start_ms));
        }
    }
    return true;
}

#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
static void v0_final_edge_replay_record_fifo_pressure(uint32_t fifo_count)
{
    if (fifo_count > s_final_edge_phase1_stats.replay_fifo_max_count)
    {
        s_final_edge_phase1_stats.replay_fifo_max_count = fifo_count;
    }
    if (fifo_count >= ZY100_FINAL_EDGE_REPLAY_FIFO_WARN_BYTES)
    {
        s_final_edge_phase1_stats.replay_fifo_high_count++;
    }
    if (fifo_count >= ZY100_FINAL_EDGE_REPLAY_FIFO_HARD_BYTES)
    {
        s_final_edge_phase1_stats.replay_fifo_hard_count++;
    }
}

static bool v0_final_edge_replay_read_fifo_pressure(v0_fifo_stats_t *stats,
                                                    uint16_t *fifo_count)
{
    uint16_t local_fifo_count = 0U;

    if ((stats == NULL) || stats->fatal_error)
    {
        return false;
    }
    if (!v0_read_fifo_count(stats, &local_fifo_count))
    {
        return false;
    }
    v0_final_edge_replay_record_fifo_pressure((uint32_t)local_fifo_count);
    if (fifo_count != NULL)
    {
        *fifo_count = local_fifo_count;
    }
    return true;
}

static bool v0_final_edge_replay_fifo_pressure_yield(v0_fifo_stats_t *stats,
                                                     uint16_t *fifo_count)
{
    uint16_t local_fifo_count = 0U;

    if (!v0_final_edge_replay_read_fifo_pressure(stats, &local_fifo_count))
    {
        return false;
    }
    if (fifo_count != NULL)
    {
        *fifo_count = local_fifo_count;
    }
    if ((uint32_t)local_fifo_count < ZY100_FINAL_EDGE_REPLAY_FIFO_WARN_BYTES)
    {
        return false;
    }

    s_final_edge_phase1_stats.replay_batch_shrink_count++;
    return true;
}

static uint32_t v0_final_edge_replay_batch_limit(void)
{
    return (ZY100_FINAL_EDGE_REPLAY_BATCH_SAMPLES != 0U) ?
        ZY100_FINAL_EDGE_REPLAY_BATCH_SAMPLES : 1U;
}

static void v0_final_edge_note_replay_pressure_interleave(uint16_t fifo_count,
                                                          uint16_t drained_bytes)
{
    uint32_t packets = drained_bytes / V0_PACKET_SIZE_BYTES;

    s_final_edge_phase1_stats.replay_pressure_interleave_count++;
    s_final_edge_phase1_stats.replay_pressure_interleave_packets += packets;
    if ((uint32_t)fifo_count >
        s_final_edge_phase1_stats.replay_pressure_interleave_max_fifo)
    {
        s_final_edge_phase1_stats.replay_pressure_interleave_max_fifo =
            fifo_count;
    }
    s_final_edge_phase1_stats.replay_pressure_interleave_last_bscan_id =
        s_final_edge_replay_bscan_id;
}

static void v0_final_edge_replay_maybe_start_backlog(v0_fifo_stats_t *stats)
{
#if ZY100_FINAL_EDGE_REPLAY_LIVE_BACKLOG_EDGE_ONLY_ENABLE
    uint16_t fifo_count = 0U;

    if ((stats == NULL) || stats->fatal_error)
    {
        return;
    }
    if (!v0_read_fifo_count(stats, &fifo_count))
    {
        return;
    }
    if ((uint32_t)fifo_count >= ZY100_FINAL_EDGE_LIVE_FIFO_DRAIN_THRESHOLD_BYTES)
    {
        s_final_edge_replay_backlog_edge_only_active = true;
        s_final_edge_replay_backlog_start_count = fifo_count;
        s_final_edge_phase1_stats.replay_backlog_edge_only_count++;
        if ((uint32_t)(fifo_count / V0_PACKET_SIZE_BYTES) >
            s_final_edge_phase1_stats.replay_backlog_max_packets)
        {
            s_final_edge_phase1_stats.replay_backlog_max_packets =
                fifo_count / V0_PACKET_SIZE_BYTES;
        }
    }
#else
    IMU_UNUSED(stats);
#endif
}

static void v0_final_edge_replay_note_done(v0_fifo_stats_t *stats)
{
    if (s_final_edge_replay_source == ZY100_FE_REPLAY_SRC_TRUEHIT)
    {
        s_final_edge_phase1_stats.replay_truehit_done_count++;
    }
    else
    {
        s_final_edge_phase1_stats.replay_done_count++;
    }
    s_final_edge_replay_pending = false;
    s_final_edge_replay_active = false;
    s_final_edge_replay_index = 0U;
    s_final_edge_replay_count = 0U;
    s_final_edge_replay_last_ms = zy100_os_time_ms();
    v0_final_edge_replay_maybe_start_backlog(stats);
    s_final_edge_replay_source = ZY100_FE_REPLAY_SRC_NOHIT;
}

static void v0_final_edge_replay_note_error(const char *reason_text)
{
    zy100_fe_replay_source_t source = s_final_edge_replay_source;

    if (source == ZY100_FE_REPLAY_SRC_TRUEHIT)
    {
        s_final_edge_phase1_stats.replay_truehit_fail_count++;
    }
    else
    {
        s_final_edge_phase1_stats.replay_fail_count++;
    }
    s_final_edge_replay_pending = false;
    s_final_edge_replay_active = false;
    s_final_edge_replay_last_ms = zy100_os_time_ms();
#if (ZY100_FINAL_EDGE_CAPTURE_RUNTIME_LOG_QUIET_ENABLE == 0U)
    DBG_DIRECT("[FE_REPLAY_FAIL] id=%u src=%u reason=%s idx=%u count=%u",
               s_final_edge_replay_bscan_id,
               (uint32_t)source,
               (reason_text != NULL) ? reason_text : "feed",
               s_final_edge_replay_index,
               s_final_edge_replay_count);
#else
    IMU_UNUSED(reason_text);
#endif
    s_final_edge_replay_source = ZY100_FE_REPLAY_SRC_NOHIT;
}

static v0_fe_replay_service_result_t v0_final_edge_replay_service_impl(
    v0_fifo_stats_t *stats)
{
    uint32_t batch_limit;
    uint32_t batch_fed = 0U;
    uint32_t batch_start_us;
    uint32_t batch_elapsed_us;

    if ((stats == NULL) || stats->fatal_error || !v0_final_edge_replay_busy())
    {
        return V0_FE_REPLAY_SERVICE_NO_WORK;
    }
    if (v0_final_edge_b_critical_active())
    {
        v0_final_edge_inc_stat(
            &s_final_edge_phase1_stats.b_critical_replay_blocked_count);
        return V0_FE_REPLAY_SERVICE_NO_WORK;
    }

    if (v0_final_edge_replay_fifo_pressure_yield(stats, NULL))
    {
        return V0_FE_REPLAY_SERVICE_PRESSURE_YIELD;
    }

    if (s_final_edge_replay_pending)
    {
        s_final_edge_replay_active = true;
        s_final_edge_replay_pending = false;
        s_final_edge_replay_started_ms = zy100_os_time_ms();
        s_final_edge_replay_last_ms = s_final_edge_replay_started_ms;
    }

    batch_limit = v0_final_edge_replay_batch_limit();
    batch_start_us = (uint32_t)imu_bsp_local_timestamp_us();
    while ((batch_fed < batch_limit) &&
           (s_final_edge_replay_index < s_final_edge_replay_count))
    {
        zy100_final_edge_bscan_ds_sample_t sample;

        if (!zy100_final_edge_bscan_get_ds_sample(
                s_final_edge_replay_index, &sample))
        {
            s_final_edge_phase1_stats.replay_ds_bad_count++;
            v0_final_edge_replay_note_edge_error();
            v0_final_edge_replay_note_error("ds_get");
            return V0_FE_REPLAY_SERVICE_ERROR;
        }
        if (!v0_final_edge_feed_edge_from_replay_sample(&sample))
        {
            v0_final_edge_replay_note_error("edge_feed");
            return V0_FE_REPLAY_SERVICE_ERROR;
        }

        s_final_edge_replay_index++;
        batch_fed++;
        if (((batch_fed % ZY100_FINAL_EDGE_REPLAY_BATCH_SAMPLES_HARD) == 0U) &&
            (s_final_edge_replay_index < s_final_edge_replay_count) &&
            v0_final_edge_replay_fifo_pressure_yield(stats, NULL))
        {
            batch_elapsed_us =
                (uint32_t)((uint32_t)imu_bsp_local_timestamp_us() -
                           batch_start_us);
            if (batch_elapsed_us >
                s_final_edge_phase1_stats.replay_batch_max_us)
            {
                s_final_edge_phase1_stats.replay_batch_max_us =
                    batch_elapsed_us;
                s_final_edge_phase1_stats.replay_long_batch_bscan_id =
                    s_final_edge_replay_bscan_id;
            }
            s_final_edge_replay_last_ms = zy100_os_time_ms();
            return V0_FE_REPLAY_SERVICE_PRESSURE_YIELD;
        }
    }

    batch_elapsed_us =
        (uint32_t)((uint32_t)imu_bsp_local_timestamp_us() - batch_start_us);
    if (batch_elapsed_us > s_final_edge_phase1_stats.replay_batch_max_us)
    {
        s_final_edge_phase1_stats.replay_batch_max_us = batch_elapsed_us;
        s_final_edge_phase1_stats.replay_long_batch_bscan_id =
            s_final_edge_replay_bscan_id;
    }
    s_final_edge_replay_last_ms = zy100_os_time_ms();

    if (s_final_edge_replay_index >= s_final_edge_replay_count)
    {
        v0_final_edge_replay_note_done(stats);
    }

    return V0_FE_REPLAY_SERVICE_FED_OR_DONE;
}

static v0_fe_replay_service_result_t v0_final_edge_replay_service(
    v0_fifo_stats_t *stats)
{
    return v0_final_edge_replay_service_impl(stats);
}

static void v0_final_edge_replay_update_backlog_after_drain(v0_fifo_stats_t *stats)
{
    uint16_t fifo_count = 0U;

    if (!s_final_edge_replay_backlog_edge_only_active ||
        (stats == NULL) ||
        stats->fatal_error)
    {
        return;
    }
    if (!v0_read_fifo_count(stats, &fifo_count))
    {
        return;
    }
    if ((uint32_t)(fifo_count / V0_PACKET_SIZE_BYTES) >
        s_final_edge_phase1_stats.replay_backlog_max_packets)
    {
        s_final_edge_phase1_stats.replay_backlog_max_packets =
            fifo_count / V0_PACKET_SIZE_BYTES;
    }
    if ((uint32_t)fifo_count < ZY100_FINAL_EDGE_LIVE_FIFO_DRAIN_THRESHOLD_BYTES)
    {
        s_final_edge_replay_backlog_edge_only_active = false;
        s_final_edge_replay_backlog_start_count = 0U;
    }
}
#endif

static uint16_t v0_final_edge_plan_drain_len(uint16_t fifo_count,
                                             uint32_t due_consumed)
{
    uint16_t aligned_count =
        (uint16_t)(fifo_count & (uint16_t)~(V0_PACKET_SIZE_BYTES - 1U));
    uint32_t due_target;
    uint16_t drain_len;

    if (aligned_count < V0_PACKET_SIZE_BYTES)
    {
        return 0U;
    }

    if (aligned_count >= V0_FE_EMERGENCY_BYTES)
    {
        return V0_FE_CATCHUP_512_BYTES;
    }
    if (aligned_count >= V0_FE_HIGH_PRESSURE_BYTES)
    {
        return V0_FE_CATCHUP_256_BYTES;
    }
    if (aligned_count >= 128U)
    {
        return 128U;
    }
    if (aligned_count >= 64U)
    {
        return 64U;
    }

    due_target = due_consumed * V0_FE_NORMAL_PACKET_BYTES;
    if (due_target < V0_FE_NORMAL_PACKET_BYTES)
    {
        due_target = V0_FE_NORMAL_PACKET_BYTES;
    }
    if (due_target > aligned_count)
    {
        due_target = aligned_count;
    }
    drain_len = (uint16_t)(due_target & (uint32_t)~(V0_PACKET_SIZE_BYTES - 1U));
    if (drain_len >= 64U)
    {
        return 64U;
    }
    if (drain_len >= 32U)
    {
        return 32U;
    }
    return V0_PACKET_SIZE_BYTES;
}

static void v0_final_edge_note_drain_len(uint16_t drain_len)
{
    switch (drain_len)
    {
    case 16U:
        s_final_edge_phase1_stats.drain_16_count++;
        break;
    case 32U:
        s_final_edge_phase1_stats.drain_32_count++;
        break;
    case 64U:
        s_final_edge_phase1_stats.drain_64_count++;
        break;
    case 128U:
        s_final_edge_phase1_stats.drain_128_count++;
        break;
    case 256U:
        s_final_edge_phase1_stats.drain_256_count++;
        break;
    default:
        if (drain_len >= 512U)
        {
            s_final_edge_phase1_stats.drain_512_count++;
        }
        break;
    }
}

static bool v0_final_edge_take_high_pressure_pending(void)
{
    uint32_t lock_state;
    bool pending;

    lock_state = os_lock();
    pending = s_v0_high_water_alarm_pending;
    s_v0_high_water_alarm_pending = false;
    os_unlock(lock_state);

    return pending;
}

static bool v0_final_edge_live_service_handler_impl(v0_fifo_stats_t *stats,
                                                    bool flush)
{
    zy100_spi_sched_drain_context_t drain_ctx;
    uint32_t raw_due;
    uint32_t due_consumed;
    uint32_t now_us;
    uint16_t fifo_count = 0U;
    uint16_t drain_len;
    uint16_t actual_read_len = 0U;
    uint8_t first_header = 0U;
    bool capped_by_flash_capacity = false;
    bool no_flash_capacity = false;
    bool high_pressure_irq;
    bool drained;
    uint32_t drained_packets;
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
    v0_fe_replay_service_result_t replay_result =
        V0_FE_REPLAY_SERVICE_NO_WORK;
    bool replay_pressure_rescue = false;
#endif

    if ((stats == NULL) || stats->fatal_error)
    {
        return false;
    }
    if (s_final_edge_no_new_data_mode)
    {
        return false;
    }

#if ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
    if (v0_final_edge_b_pending_critical())
    {
        return v0_final_edge_consume_deferred_b_scan(stats);
    }
#endif

#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
    replay_result = v0_final_edge_replay_service(stats);
    if (replay_result == V0_FE_REPLAY_SERVICE_PRESSURE_YIELD)
    {
        replay_pressure_rescue = true;
    }
    else if (replay_result != V0_FE_REPLAY_SERVICE_NO_WORK)
    {
        return true;
    }
#endif

    raw_due = v0_final_edge_consume_timer_due();
    high_pressure_irq = v0_final_edge_take_high_pressure_pending();
    if (!flush
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
        && !replay_pressure_rescue
#endif
        && (raw_due == 0U) && !high_pressure_irq
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
        && !s_final_edge_replay_backlog_edge_only_active
#endif
       )
    {
        return false;
    }
    if ((flush
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
         || replay_pressure_rescue
         || s_final_edge_replay_backlog_edge_only_active
#endif
        ) && (raw_due == 0U))
    {
        raw_due = 1U;
    }

    now_us = (uint32_t)imu_bsp_local_timestamp_us();
    if (s_final_edge_phase1_stats.service_last_us != 0U)
    {
        uint32_t gap_us = now_us - s_final_edge_phase1_stats.service_last_us;

        s_final_edge_phase1_stats.service_gap_sum_us += gap_us;
        if (gap_us > s_final_edge_phase1_stats.service_gap_max_us)
        {
            s_final_edge_phase1_stats.service_gap_max_us = gap_us;
        }
        if (gap_us > s_final_edge_phase1_stats.runtime_service_gap_max_us)
        {
            s_final_edge_phase1_stats.runtime_service_gap_max_us = gap_us;
        }
    }
    s_final_edge_phase1_stats.service_last_us = now_us;
    s_final_edge_phase1_stats.service_last_ms = zy100_os_time_ms();
    s_final_edge_phase1_stats.service_count++;

    if (raw_due > s_final_edge_phase1_stats.timer_due_max)
    {
        s_final_edge_phase1_stats.timer_due_max = raw_due;
    }
    if (raw_due > 1U)
    {
        s_final_edge_phase1_stats.timer_due_coalesced_count++;
    }

    due_consumed = raw_due;
    if (due_consumed > ZY100_FINAL_EDGE_TIMER_MAX_DUE_BURST)
    {
        s_final_edge_phase1_stats.timer_due_capped_count +=
            (due_consumed - ZY100_FINAL_EDGE_TIMER_MAX_DUE_BURST);
        due_consumed = ZY100_FINAL_EDGE_TIMER_MAX_DUE_BURST;
    }
    if (due_consumed == 0U)
    {
        due_consumed = 1U;
    }
    s_final_edge_phase1_stats.timer_due_consumed_count += due_consumed;
    if (high_pressure_irq)
    {
        s_final_edge_phase1_stats.timer_int_wake_count++;
    }

    if (!v0_read_fifo_count(stats, &fifo_count))
    {
        return false;
    }
    if ((uint32_t)fifo_count > s_final_edge_phase1_stats.fifo_max_count)
    {
        s_final_edge_phase1_stats.fifo_max_count = fifo_count;
    }
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
    if (s_final_edge_replay_backlog_edge_only_active &&
        ((uint32_t)fifo_count >= ZY100_FINAL_EDGE_REPLAY_FIFO_WARN_BYTES))
    {
        v0_final_edge_replay_record_fifo_pressure((uint32_t)fifo_count);
        replay_pressure_rescue = true;
    }
#endif

    drain_len = v0_final_edge_plan_drain_len(fifo_count, due_consumed);
    if (drain_len == 0U)
    {
        s_final_edge_phase1_stats.timer_empty_count++;
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
        if (replay_pressure_rescue)
        {
            v0_final_edge_note_replay_pressure_interleave(fifo_count, 0U);
            return true;
        }
        v0_final_edge_replay_update_backlog_after_drain(stats);
#endif
        return true;
    }
    if (fifo_count >= V0_FE_HIGH_PRESSURE_BYTES)
    {
        s_final_edge_phase1_stats.fifo_emergency_count++;
    }

    memset(&drain_ctx, 0, sizeof(drain_ctx));
    drain_ctx.seq = zy100_spi_sched_alloc_drain_seq();
    drain_ctx.src = (fifo_count >= V0_FE_HIGH_PRESSURE_BYTES) ?
                    ZY100_SPI_SCHED_DRAIN_SRC_EMERGENCY :
                    ZY100_SPI_SCHED_DRAIN_SRC_COUNT_BASED;
    drain_ctx.req_len = drain_len;
    drain_ctx.actual_len = drain_len;
    drain_ctx.cached_count_valid = true;
    drain_ctx.cached_count = fifo_count;
    drain_ctx.fifo_irq_pending = high_pressure_irq;
    drain_ctx.irq_pending_count = s_v0_imu_fifo_pending_count;
    drain_ctx.int_level = imu_bsp_int_level();

    drained = v0_drain_fifo_packets_exact(stats,
                                          &drain_ctx,
                                          drain_len,
                                          false,
                                          &first_header,
                                          &actual_read_len,
                                          &capped_by_flash_capacity,
                                          &no_flash_capacity);
    IMU_UNUSED(first_header);
    IMU_UNUSED(capped_by_flash_capacity);
    IMU_UNUSED(no_flash_capacity);
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
    if (replay_pressure_rescue)
    {
        v0_final_edge_note_replay_pressure_interleave(fifo_count,
                                                      actual_read_len);
    }
#endif
    if (!drained || (actual_read_len == 0U))
    {
        return drained;
    }

    drained_packets = actual_read_len / V0_PACKET_SIZE_BYTES;
    s_final_edge_phase1_stats.drain_count++;
    s_final_edge_phase1_stats.drain_packet_sum += drained_packets;
    if (drained_packets > s_final_edge_phase1_stats.drain_packet_max)
    {
        s_final_edge_phase1_stats.drain_packet_max = drained_packets;
    }
    v0_final_edge_note_drain_len(actual_read_len);
    stats->irq_drain_count++;
    stats->poll_rescue_drain_count++;
    stats->last_fifo_drain_ms = zy100_os_time_ms();
    if (stats->fifo_drain_since_start < 0xFFFFFFFFU)
    {
        stats->fifo_drain_since_start++;
    }

#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
    if (s_final_edge_replay_backlog_edge_only_active)
    {
        v0_final_edge_replay_update_backlog_after_drain(stats);
        return true;
    }
    if (replay_pressure_rescue)
    {
        return true;
    }
#endif
#if ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
    if (!v0_final_edge_consume_deferred_b_scan(stats))
    {
        return false;
    }
#endif

    return true;
}

static bool v0_final_edge_live_service_handler(v0_fifo_stats_t *stats,
                                               bool flush)
{
    return v0_final_edge_live_service_handler_impl(stats, flush);
}
#endif

#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE && !ZY100_FINAL_EDGE_MODE_ENABLE
static void v0_online_live_timer_notify_from_isr(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t now_us = (uint32_t)imu_bsp_local_timestamp_us();
    uint32_t last_us = s_v0_online_live_timer_last_us;

    if (last_us != 0U)
    {
        uint32_t gap_us = now_us - last_us;

        if (gap_us > s_v0_online_live_timer_gap_max_us)
        {
            s_v0_online_live_timer_gap_max_us = gap_us;
        }
    }
    s_v0_online_live_timer_last_us = now_us;
    if (s_v0_online_live_timer_due_count < 0xFFFFFFFFU)
    {
        s_v0_online_live_timer_due_count++;
    }
    if (s_v0_online_live_timer_total_due_count < 0xFFFFFFFFU)
    {
        s_v0_online_live_timer_total_due_count++;
    }
    if (s_v0_task_handle != NULL)
    {
        BaseType_t notify_status;

        notify_status = xTaskNotifyFromISR(s_v0_task_handle,
                                           V0_NOTIFY_LIVE_TIMER,
                                           eSetBits,
                                           &xHigherPriorityTaskWoken);
        if ((notify_status != pdPASS) &&
            (s_v0_online_live_timer_notify_fail_count < 0xFFFFFFFFU))
        {
            s_v0_online_live_timer_notify_fail_count++;
        }
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static uint32_t v0_online_live_consume_timer_due(void)
{
    uint32_t lock_state;
    uint32_t due;

    lock_state = os_lock();
    due = s_v0_online_live_timer_due_count;
    s_v0_online_live_timer_due_count = 0U;
    os_unlock(lock_state);
    return due;
}

static void v0_online_live_clear_timer_due(void)
{
    uint32_t lock_state;

    lock_state = os_lock();
    s_v0_online_live_timer_due_count = 0U;
    os_unlock(lock_state);
}

static bool v0_online_live_timer_start(void)
{
    imu_status_t status;

    status = imu_bsp_ois_tick_timer_acquire(
        IMU_BSP_OIS_TICK_TIMER_OWNER_LIVE_FIFO);
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[ONL_TIMER] acquire_fail status=%u owner=%u",
                   (uint32_t)status,
                   (uint32_t)imu_bsp_ois_tick_timer_get_owner());
        return false;
    }

    v0_online_live_clear_timer_due();
    s_v0_online_live_timer_total_due_count = 0U;
    s_v0_online_live_timer_last_us = 0U;
    s_v0_online_live_timer_gap_max_us = 0U;
    s_v0_online_live_timer_notify_fail_count = 0U;
    imu_bsp_ois_tick_timer_register_irq_callback(
        v0_online_live_timer_notify_from_isr);
    status = imu_bsp_ois_tick_timer_config(
        ZY100_ONLINE_LIVE_TIMER_PERIOD_US);
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_ois_tick_timer_start();
    }
    if (status != IMU_STATUS_OK)
    {
        imu_bsp_ois_tick_timer_stop();
        imu_bsp_ois_tick_timer_register_irq_callback(NULL);
        imu_bsp_ois_tick_timer_release(
            IMU_BSP_OIS_TICK_TIMER_OWNER_LIVE_FIFO);
        DBG_DIRECT("[ONL_TIMER] start_fail status=%u", (uint32_t)status);
        return false;
    }

    s_v0_online_live_timer_running = true;
    return true;
}

static void v0_online_live_timer_stop(void)
{
    if (s_v0_online_live_timer_running ||
        (imu_bsp_ois_tick_timer_get_owner() ==
         IMU_BSP_OIS_TICK_TIMER_OWNER_LIVE_FIFO))
    {
        imu_bsp_ois_tick_timer_stop();
        imu_bsp_ois_tick_timer_register_irq_callback(NULL);
        imu_bsp_ois_tick_timer_release(
            IMU_BSP_OIS_TICK_TIMER_OWNER_LIVE_FIFO);
    }
    s_v0_online_live_timer_running = false;
    v0_online_live_clear_timer_due();
}

static uint16_t v0_online_live_plan_drain_len(uint16_t fifo_count,
                                               uint32_t due_consumed)
{
    uint16_t aligned_count =
        (uint16_t)(fifo_count & (uint16_t)~(V0_PACKET_SIZE_BYTES - 1U));
    uint32_t due_target;
    uint16_t drain_len;

    if (aligned_count < V0_PACKET_SIZE_BYTES)
    {
        return 0U;
    }
    if (aligned_count >= ZY100_ONLINE_IMU_FIFO_EMERGENCY_BYTES)
    {
        return 512U;
    }
    if (aligned_count >= ZY100_ONLINE_LIVE_HIGH_PRESSURE_BYTES)
    {
        return 256U;
    }
    if (aligned_count >= 128U)
    {
        return 128U;
    }
    if (aligned_count >= 64U)
    {
        return 64U;
    }

    due_target = due_consumed * V0_PACKET_SIZE_BYTES;
    if (due_target < V0_PACKET_SIZE_BYTES)
    {
        due_target = V0_PACKET_SIZE_BYTES;
    }
    if (due_target > aligned_count)
    {
        due_target = aligned_count;
    }
    drain_len = (uint16_t)(due_target &
                           (uint32_t)~(V0_PACKET_SIZE_BYTES - 1U));
    if (drain_len >= 64U)
    {
        return 64U;
    }
    if (drain_len >= 32U)
    {
        return 32U;
    }
    return V0_PACKET_SIZE_BYTES;
}

static bool v0_online_live_take_high_pressure_pending(void)
{
    uint32_t lock_state;
    bool pending;

    lock_state = os_lock();
    pending = s_v0_high_water_alarm_pending;
    s_v0_high_water_alarm_pending = false;
    os_unlock(lock_state);
    return pending;
}

static bool v0_online_live_fifo_service(v0_fifo_stats_t *stats, bool flush)
{
    zy100_spi_sched_drain_context_t drain_ctx;
    uint32_t due_consumed;
    uint16_t fifo_count = 0U;
    uint16_t drain_len;
    uint16_t actual_read_len = 0U;
    uint8_t first_header = 0U;
    bool capped_by_flash_capacity = false;
    bool no_flash_capacity = false;
    bool high_pressure_irq;
    bool drained;

    if ((stats == NULL) || stats->fatal_error)
    {
        return false;
    }

    due_consumed = v0_online_live_consume_timer_due();
    high_pressure_irq = v0_online_live_take_high_pressure_pending();
    if (!flush && (due_consumed == 0U) && !high_pressure_irq)
    {
        return false;
    }
    if (due_consumed == 0U)
    {
        due_consumed = 1U;
    }
    if (due_consumed > ZY100_ONLINE_LIVE_TIMER_MAX_DUE_BURST)
    {
        due_consumed = ZY100_ONLINE_LIVE_TIMER_MAX_DUE_BURST;
    }
    if (!v0_read_fifo_count(stats, &fifo_count))
    {
        return false;
    }

    drain_len = v0_online_live_plan_drain_len(fifo_count, due_consumed);
    if (drain_len == 0U)
    {
        return true;
    }

    memset(&drain_ctx, 0, sizeof(drain_ctx));
    drain_ctx.seq = zy100_spi_sched_alloc_drain_seq();
    drain_ctx.src =
        (fifo_count >= ZY100_ONLINE_LIVE_HIGH_PRESSURE_BYTES) ?
        ZY100_SPI_SCHED_DRAIN_SRC_EMERGENCY :
        ZY100_SPI_SCHED_DRAIN_SRC_COUNT_BASED;
    drain_ctx.req_len = drain_len;
    drain_ctx.actual_len = drain_len;
    drain_ctx.cached_count_valid = true;
    drain_ctx.cached_count = fifo_count;
    drain_ctx.fifo_irq_pending = high_pressure_irq;
    drain_ctx.irq_pending_count = s_v0_imu_fifo_pending_count;
    drain_ctx.int_level = imu_bsp_int_level();

    drained = v0_drain_fifo_packets_exact(stats,
                                          &drain_ctx,
                                          drain_len,
                                          false,
                                          &first_header,
                                          &actual_read_len,
                                          &capped_by_flash_capacity,
                                          &no_flash_capacity);
    IMU_UNUSED(first_header);
    IMU_UNUSED(capped_by_flash_capacity);
    IMU_UNUSED(no_flash_capacity);
    if (!drained || (actual_read_len == 0U))
    {
        return drained;
    }

    stats->irq_drain_count++;
    stats->poll_rescue_drain_count++;
    stats->last_fifo_drain_ms = zy100_os_time_ms();
    if (stats->fifo_drain_since_start < 0xFFFFFFFFU)
    {
        stats->fifo_drain_since_start++;
    }
    return true;
}
#endif

#if !ZY100_FINAL_EDGE_MODE_ENABLE
static bool v0_sched_read_fifo_count(void *ctx, uint16_t *fifo_count)
{
    v0_fifo_stats_t *stats = (v0_fifo_stats_t *)ctx;

    if ((stats == NULL) || (fifo_count == NULL))
    {
        return false;
    }

    if (!v0_read_fifo_count(stats, fifo_count))
    {
        return false;
    }

    if (*fifo_count < V0_FIFO_WATERMARK_BYTES)
    {
        stats->poll_below_wm_count++;
    }
    else
    {
        stats->poll_rescue_count++;
        if (*fifo_count >= V0_FIFO_HIGH_WATER_BYTES)
        {
            stats->high_water_rescue_count++;
        }
    }
    return true;
}

static bool v0_sched_read_fifo_int_status(void *ctx)
{
    v0_fifo_stats_t *stats = (v0_fifo_stats_t *)ctx;
    icm53611_int_status_t int_status;

    if (stats == NULL)
    {
        return false;
    }

    if (!v0_read_int_status(stats, &int_status))
    {
        return false;
    }

#if V0_FIFO_MARKER_TRIGGER_ENABLED
    v0_process_int_status(&int_status, s_v0_current_sample_seq, true);
#else
    IMU_UNUSED(int_status);
#endif
    return true;
}

static bool v0_sched_drain_fifo_512(void *ctx,
                                    const zy100_spi_sched_drain_context_t *drain_ctx,
                                    uint16_t drain_len,
                                    bool guarded_startup,
                                    uint8_t *first_header,
                                    uint16_t *actual_read_len,
                                    bool *capped_by_flash_capacity,
                                    bool *no_flash_capacity)
{
    v0_fifo_stats_t *stats = (v0_fifo_stats_t *)ctx;
    uint32_t bad_before;
    uint16_t lost_before;
    uint32_t full_before;
    bool drained;
    bool clean_drain;

    if (stats == NULL)
    {
        return false;
    }

#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
    drain_len = v0_phase_a_planned_drain_len(drain_len);
    if (drain_len == 0U)
    {
        if (no_flash_capacity != NULL)
        {
            *no_flash_capacity = true;
        }
        return false;
    }
#endif

    bad_before = stats->bad_header_count;
    lost_before = stats->fifo_lost_pkt_count;
    full_before = stats->fifo_full_count;
    drained = v0_drain_fifo_packets_exact(stats,
                                          drain_ctx,
                                          drain_len,
                                          guarded_startup,
                                          first_header,
                                          actual_read_len,
                                          capped_by_flash_capacity,
                                          no_flash_capacity);
    clean_drain = drained &&
                  (stats->bad_header_count == bad_before) &&
                  (stats->fifo_lost_pkt_count == lost_before) &&
                  (stats->fifo_full_count == full_before);
    if (drained)
    {
        stats->irq_drain_count++;
        stats->poll_rescue_drain_count++;
    }
    if (clean_drain)
    {
        stats->last_fifo_drain_ms = zy100_os_time_ms();
        if (stats->fifo_drain_since_start < 0xFFFFFFFFU)
        {
            stats->fifo_drain_since_start++;
        }
    }
#if ZY100_RT_MARKER_ENABLE && ZY100_RT_MARKER_TIMER_ENABLE
    v0_rt_marker_note_drain_start_candidate(stats,
                                            drain_ctx,
                                            drained,
                                            bad_before,
                                            lost_before,
                                            full_before,
                                            false,
                                            0U);
#endif
    return drained;
}

#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
static bool v0_sched_fifo_poll_drain(void *ctx,
                                     bool high_water_alarm,
                                     zy100_spi_sched_poll_drain_result_t *result)
{
    v0_fifo_stats_t *stats = (v0_fifo_stats_t *)ctx;
    zy100_spi_sched_drain_context_t drain_ctx;
    uint16_t fifo_count = 0U;
    uint16_t drain_len;
    uint16_t actual_read_len = 0U;
    uint8_t first_header = 0U;
    bool capped_by_flash_capacity = false;
    bool no_flash_capacity = false;
    bool high_pressure;
    bool emergency;
    bool drained;
    bool pressure_clear_after_count;

    if ((stats == NULL) || (result == NULL))
    {
        return false;
    }

    stats->poll_count++;
    if (!v0_read_fifo_count(stats, &fifo_count))
    {
        return false;
    }

    result->count_valid = true;
    result->fifo_count = fifo_count;
    v0_phase_a_update_pressure_latches(fifo_count);
    high_pressure = s_v0_phase_a_rescue_pending ||
                    s_v0_high_water_alarm_pending ||
                    high_water_alarm ||
                    ((uint32_t)fifo_count >= ZY100_FIFO_WATERMARK_ALARM_BYTES);
    emergency = ((uint32_t)fifo_count >= ZY100_FIFO_EMERGENCY_BYTES);
    result->high_pressure = high_pressure;
    result->emergency = emergency;
    v0_phase_a_update_backpressure(stats);
    pressure_clear_after_count =
        high_water_alarm &&
        !s_v0_high_water_alarm_pending &&
        !s_v0_phase_a_rescue_pending;
    if (stats->ingress_backpressure_pending &&
        ((uint32_t)fifo_count < ZY100_FIFO_WATERMARK_ALARM_BYTES))
    {
        IMU_UNUSED(v0_phase_a_service_backpressure_once(stats, true));
        if (pressure_clear_after_count)
        {
            stats->phase_a_alarm_service_done = true;
            zy100_spi_sched_request_fifo_int_status();
        }
        return true;
    }

    drain_len = v0_phase_a_planned_drain_len(fifo_count);
    if (drain_len == 0U)
    {
        stats->poll_below_wm_count++;
        if (pressure_clear_after_count)
        {
            stats->phase_a_alarm_service_done = true;
            zy100_spi_sched_request_fifo_int_status();
        }
        return true;
    }

    result->drain_len = drain_len;
    if (high_pressure)
    {
        stats->poll_rescue_count++;
        if (emergency)
        {
            stats->high_water_rescue_count++;
        }
    }

    if (!v0_phase_a_ingress_can_accept_drain(drain_len))
    {
        v0_phase_a_note_ingress_blocked(stats,
                                        fifo_count,
                                        drain_len,
                                        high_pressure,
                                        emergency);
        result->ingress_blocked = true;
        return true;
    }

    memset(&drain_ctx, 0, sizeof(drain_ctx));
    drain_ctx.seq = zy100_spi_sched_alloc_drain_seq();
    drain_ctx.src = emergency ? ZY100_SPI_SCHED_DRAIN_SRC_EMERGENCY :
                    ZY100_SPI_SCHED_DRAIN_SRC_COUNT_BASED;
    drain_ctx.req_len = drain_len;
    drain_ctx.actual_len = drain_len;
    drain_ctx.cached_count_valid = true;
    drain_ctx.cached_count = fifo_count;
    drain_ctx.fifo_irq_pending = high_water_alarm;
    drain_ctx.irq_pending_count = s_v0_imu_fifo_pending_count;
    drain_ctx.int_level = imu_bsp_int_level();

    drained = v0_sched_drain_fifo_512(ctx,
                                      &drain_ctx,
                                      drain_len,
                                      false,
                                      &first_header,
                                      &actual_read_len,
                                      &capped_by_flash_capacity,
                                      &no_flash_capacity);
    IMU_UNUSED(first_header);
    IMU_UNUSED(capped_by_flash_capacity);
    IMU_UNUSED(no_flash_capacity);

    result->drained = drained && (actual_read_len != 0U);
    result->drain_len = actual_read_len;
#if ZY100_LEGACY_OFFLINE_ENABLE
    if (s_v0_offline_fifo_diag.active && result->drained)
    {
        uint16_t post_count = fifo_count;
        imu_status_t post_status = icm53611_fifo_get_count(&post_count);

        if (post_status == IMU_STATUS_OK)
        {
            s_v0_offline_fifo_diag.final_fifo_count = post_count;
            v0_offline_set_last_post_count(post_count);
            v0_phase_a_update_pressure_latches(post_count);
            if (post_count > s_v0_offline_fifo_diag.max_fifo_count)
            {
                s_v0_offline_fifo_diag.max_fifo_count = post_count;
            }
        }
        else
        {
            v0_mark_read_error(stats, post_status);
        }
    }
#endif
    if (result->drained && pressure_clear_after_count)
    {
        stats->phase_a_alarm_service_done = true;
        zy100_spi_sched_request_fifo_int_status();
    }
    return true;
}
#endif

#if ZY100_LEGACY_OFFLINE_ENABLE && ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
static bool v0_offline_fifo_service(v0_fifo_stats_t *stats,
                                    bool irq_pending,
                                    bool *safe_to_yield)
{
    zy100_spi_sched_poll_drain_result_t result;
    uint32_t read_count = 0U;
    uint32_t read_limit = 1U;
    v0_offline_fifo_stage_t stage = V0_OFFLINE_FIFO_STAGE_NORMAL;
    bool first = true;

    if ((stats == NULL) || (safe_to_yield == NULL))
    {
        return false;
    }
    *safe_to_yield = true;
    do
    {
        memset(&result, 0, sizeof(result));
        if (!v0_sched_fifo_poll_drain(stats,
                                      irq_pending ||
                                      s_v0_high_water_alarm_pending,
                                      &result) ||
            !result.count_valid)
        {
            return false;
        }
        if (first)
        {
            first = false;
            if (irq_pending)
            {
                s_v0_offline_fifo_diag.irq_service_count++;
            }
            else
            {
                s_v0_offline_fifo_diag.poll_service_count++;
            }
            if (result.fifo_count > s_v0_offline_fifo_diag.max_fifo_count)
            {
                s_v0_offline_fifo_diag.max_fifo_count = result.fifo_count;
            }
            if ((uint32_t)result.fifo_count >= ZY100_FIFO_EMERGENCY_BYTES)
            {
                stage = V0_OFFLINE_FIFO_STAGE_RESCUE;
                read_limit = ZY100_OFFLINE_V2_FIFO_RESCUE_BURST_MAX;
                s_v0_offline_fifo_diag.rescue_enter_count++;
            }
            else if ((uint32_t)result.fifo_count >=
                     ZY100_FIFO_WATERMARK_ALARM_BYTES)
            {
                stage = V0_OFFLINE_FIFO_STAGE_ALARM;
                read_limit = ZY100_OFFLINE_V2_FIFO_ALARM_BURST_MAX;
                s_v0_offline_fifo_diag.alarm_enter_count++;
            }
            if ((uint32_t)result.fifo_count >= ZY100_FIFO_CAPACITY_BYTES)
            {
                v0_offline_fifo_latch_fault(V0_OFFLINE_FAULT_FIFO_FULL,
                                            stats,
                                            result.fifo_count,
                                            result.drain_len,
                                            result.drain_len,
                                            0U,
                                            stage);
                stats->fifo_full_count++;
                stats->fatal_error = true;
                stats->fatal_status = IMU_STATUS_NOT_READY;
                v0_request_stop_from_worker(
                    IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL);
            }
        }
        if (!result.drained)
        {
            break;
        }
        read_count++;
        if ((stage != V0_OFFLINE_FIFO_STAGE_NORMAL) &&
            (s_v0_offline_fifo_diag.final_fifo_count <=
             ZY100_FIFO_EMERGENCY_RESCUE_FLOOR_BYTES))
        {
            s_v0_offline_fifo_diag.pressure_recover_count++;
            break;
        }
        irq_pending = false;
    } while ((read_count < read_limit) && !stats->fatal_error);

    if (read_count > s_v0_offline_fifo_diag.max_burst_reads)
    {
        s_v0_offline_fifo_diag.max_burst_reads = read_count;
    }
    *safe_to_yield =
        (stage == V0_OFFLINE_FIFO_STAGE_NORMAL) ||
        (s_v0_offline_fifo_diag.final_fifo_count <=
         ZY100_FIFO_EMERGENCY_RESCUE_FLOOR_BYTES);
    return true;
}
#endif

#if ZY100_RT_MARKER_ENABLE
static bool v0_sched_rt_marker_read(void *ctx, uint32_t now_ms)
{
    IMU_UNUSED(ctx);
    return imu_rt_marker_handle_due_once(now_ms);
}

static void v0_sched_rt_marker_skip_fifo(void *ctx, uint32_t now_ms)
{
    IMU_UNUSED(ctx);
    imu_rt_marker_skip_fifo_emergency(now_ms);
}

static void v0_sched_rt_marker_skip_spi(void *ctx, uint32_t now_ms)
{
    IMU_UNUSED(ctx);
    imu_rt_marker_skip_spi_busy(now_ms);
}
#endif
#endif

#if !ZY100_FINAL_EDGE_MODE_ENABLE
static bool v0_sched_flash_pump(void *ctx)
{
#if V1_IMU_FLASH_CAPTURE_ENABLE
#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
    return v0_phase_a_flash_pump_once((v0_fifo_stats_t *)ctx);
#else
    IMU_UNUSED(ctx);
    return external_flash_capture_pump_once_bounded();
#endif
#else
    IMU_UNUSED(ctx);
    return true;
#endif
}
#endif


#if !ZY100_FINAL_EDGE_MODE_ENABLE
static void v0_sched_flash_skip_fifo(void *ctx)
{
    IMU_UNUSED(ctx);
}
#endif

#if V0_FLASH_RUNTIME_ENABLE
static void v0_log_flash_queue_if_due(
    uint32_t runtime_ms,
    uint32_t packet_count,
    const external_flash_capture_queue_status_t *queue_status,
    const zy100_spi_sched_stats_t *sched_stats,
    bool *log_valid,
    uint32_t *last_summary_ms,
    uint32_t *last_page_issued,
    uint32_t *last_block_done,
    uint32_t *last_level,
    bool *last_near_full,
    uint32_t *last_write_count,
    uint32_t *last_write_page_index,
    external_flash_capture_pump_result_t *last_pump_result)
{
#if !ZY100_RUNTIME_STATS_LOG_ENABLE
    IMU_UNUSED(runtime_ms);
    IMU_UNUSED(packet_count);
    IMU_UNUSED(queue_status);
    IMU_UNUSED(sched_stats);
    IMU_UNUSED(log_valid);
    IMU_UNUSED(last_summary_ms);
    IMU_UNUSED(last_page_issued);
    IMU_UNUSED(last_block_done);
    IMU_UNUSED(last_level);
    IMU_UNUSED(last_near_full);
    IMU_UNUSED(last_write_count);
    IMU_UNUSED(last_write_page_index);
    IMU_UNUSED(last_pump_result);
#else
    bool first_log;
    bool near_full_entered;
    bool block_done;
    bool event_log;
    bool summary_due;
    uint32_t produced_blocks;
    uint32_t written_blocks;
    int32_t backlog_blocks;
    uint32_t elapsed_ms;
    uint32_t page_s;
    uint32_t block_s;
#if ZY100_FLASH_Q_VERBOSE_ENABLE
    bool level_changed;
    bool page_event;
#endif

    if ((queue_status == NULL) || (sched_stats == NULL) ||
        (log_valid == NULL) || (last_summary_ms == NULL) ||
        (last_page_issued == NULL) || (last_block_done == NULL) ||
        (last_level == NULL) || (last_near_full == NULL) ||
        (last_write_count == NULL) || (last_write_page_index == NULL) ||
        (last_pump_result == NULL))
    {
        return;
    }

    first_log = !(*log_valid);
    near_full_entered = queue_status->queue_near_full && !(*last_near_full);
    block_done =
        (queue_status->last_pump_result == EXTERNAL_FLASH_CAPTURE_PUMP_BLOCK_DONE) &&
        ((queue_status->write_count != *last_write_count) ||
         (*last_pump_result != EXTERNAL_FLASH_CAPTURE_PUMP_BLOCK_DONE));

#if ZY100_FLASH_Q_VERBOSE_ENABLE
    level_changed = (queue_status->queue_level != *last_level);
    page_event = (queue_status->write_page_index != *last_write_page_index) ||
                 (queue_status->last_pump_result != *last_pump_result);
    event_log = first_log || level_changed || near_full_entered ||
                block_done || page_event;
#else
    event_log = first_log || near_full_entered || block_done;
#endif
    elapsed_ms = (uint32_t)(runtime_ms - *last_summary_ms);
    summary_due = (elapsed_ms >= V0_FLASH_Q_LOG_PERIOD_MS);

    if (event_log)
    {
        DBG_DIRECT("[FLASH_Q] level=%u near_full=%u write_count=%u write_page_index=%u pump_result=%u raw_busy=%u wip=%u",
                   queue_status->queue_level,
                   queue_status->queue_near_full ? 1U : 0U,
                   queue_status->write_count,
                   queue_status->write_page_index,
                   (uint32_t)queue_status->last_pump_result,
                   queue_status->raw_queue_busy ? 1U : 0U,
                   queue_status->normal_flash_wip ? 1U : 0U);
    }

    if (summary_due)
    {
        DBG_DIRECT("[FLASH_Q_1S] level=%u near_full=%u write_count=%u write_page_index=%u pump_op=%u page_issued=%u block_done=%u throttle_rt=%u throttle_fifo=%u throttle_budget=%u max_consecutive_flash_steps=%u",
                   queue_status->queue_level,
                   queue_status->queue_near_full ? 1U : 0U,
                   queue_status->write_count,
                   queue_status->write_page_index,
                   sched_stats->flash_pump_op,
                   sched_stats->flash_page_issued,
                   sched_stats->flash_block_done,
                   sched_stats->flash_throttle_rt_due,
                   sched_stats->flash_throttle_fifo_pending,
                   sched_stats->flash_throttle_budget,
                   sched_stats->max_consecutive_flash_steps);
        produced_blocks = packet_count / V1_PACKETS_PER_BLOCK;
        written_blocks = queue_status->write_count;
        backlog_blocks = v0_diff_u32_i32(produced_blocks, written_blocks);
        page_s = v0_rate_per_s(v0_delta_u32(sched_stats->flash_page_issued,
                                            *last_page_issued),
                               elapsed_ms);
        block_s = v0_rate_per_s(v0_delta_u32(sched_stats->flash_block_done,
                                             *last_block_done),
                                elapsed_ms);
        ZY100_DIAG_LOG("[THROUGHPUT_1S] produced_blk=%u written_blk=%u backlog=%d page_s=%u block_s=%u q=%u",
                   produced_blocks,
                   written_blocks,
                   backlog_blocks,
                   page_s,
                   block_s,
                   queue_status->queue_level);
        *last_page_issued = sched_stats->flash_page_issued;
        *last_block_done = sched_stats->flash_block_done;
        *last_summary_ms = runtime_ms;
    }

    *log_valid = true;
    *last_level = queue_status->queue_level;
    *last_near_full = queue_status->queue_near_full;
    *last_write_count = queue_status->write_count;
    *last_write_page_index = queue_status->write_page_index;
    *last_pump_result = queue_status->last_pump_result;
#endif
}
#endif

#if !V0_FLASH_RUNTIME_ENABLE
static void v0_log_stats(const v0_fifo_stats_t *stats, uint32_t runtime_ms, bool final)
{
    uint32_t expected_packets = v0_expected_packets(runtime_ms);

    if (!final && !ZY100_RUNTIME_STATS_LOG_ENABLE)
    {
        return;
    }
    if (final && !IMU_CAPTURE_SUMMARY_LOG_ENABLE)
    {
        return;
    }

    if (final)
    {
        DBG_DIRECT("[V0_IMU_FIFO][SUMMARY] round=%u t=%us irq=%u irq_drain=%u poll=%u poll_below_wm=%u "
                   "rescue=%u poll_rescue_drain=%u high_water_rescue_count=%u stop_drain=%u "
                   "pkt=%u exp=%u bytes=%u fifo_max=%u lost=%u full=%u bad_hdr=%u ts_bad=%u read_err=%u",
                   stats->round,
                   runtime_ms / 1000U,
                   stats->irq_count,
                   stats->irq_drain_count,
                   stats->poll_count,
                   stats->poll_below_wm_count,
                   stats->poll_rescue_count,
                   stats->poll_rescue_drain_count,
                   stats->high_water_rescue_count,
                   stats->stop_drain_count,
                   stats->packet_count,
                   expected_packets,
                   stats->bytes_read,
                   stats->max_fifo_count,
                   (uint32_t)stats->fifo_lost_pkt_count,
                   stats->fifo_full_count,
                   stats->bad_header_count,
                   stats->ts_jump_count,
                   stats->read_error_count);
        return;
    }

    DBG_DIRECT("[V0_IMU_FIFO][1S] round=%u t=%us irq=%u irq_drain=%u poll=%u rescue=%u "
               "pkt=%u bytes=%u fifo_max=%u lost=%u full=%u bad_hdr=%u ts_bad=%u read_err=%u",
               stats->round,
               runtime_ms / 1000U,
               stats->irq_count,
               stats->irq_drain_count,
               stats->poll_count,
               stats->poll_rescue_count,
               stats->packet_count,
               stats->bytes_read,
               stats->max_fifo_count,
               (uint32_t)stats->fifo_lost_pkt_count,
               stats->fifo_full_count,
               stats->bad_header_count,
               stats->ts_jump_count,
               stats->read_error_count);
}

#if ZY100_FINAL_EDGE_MODE_ENABLE
static bool v0_final_edge_phase3_data_passed(
    const v0_fifo_stats_t *stats,
    v0_final_edge_pass_diag_t *diag)
{
    v0_final_edge_pass_diag_t local_diag;
    v0_final_edge_pass_diag_t *out = (diag != NULL) ? diag : &local_diag;

    memset(out, 0, sizeof(*out));
#if ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
    if (stats == NULL)
    {
        return false;
    }

    out->reached_summary_ok =
        (s_fe_final_drain_done && s_fe_export_ready) ? 1U : 0U;
    out->fatal_ok = stats->fatal_error ? 0U : 1U;
    out->fifo_full_ok = (stats->fifo_full_count == 0U) ? 1U : 0U;
    out->fifo_lost_ok =
        (stats->fifo_lost_pkt_count == 0U) ? 1U : 0U;
    out->bad_header_ok =
        ((stats->bad_header_count == 0U) &&
         (s_final_edge_phase1_stats.hdr_bad_count == 0U)) ? 1U : 0U;
    out->dt_seen_ok = s_final_edge_phase1_stats.dt_seen ? 1U : 0U;
    out->dt_bad_ok =
        (s_final_edge_phase1_stats.dt_bad_count == 0U) ? 1U : 0U;
    out->dt_range_ok =
        (s_final_edge_phase1_stats.dt_seen &&
         (s_final_edge_phase1_stats.dt_min_us >= V0_TS_DELTA_MIN_US) &&
         (s_final_edge_phase1_stats.dt_max_us <= V0_TS_DELTA_MAX_US)) ?
            1U : 0U;
    out->packet_nonzero_ok =
        (s_final_edge_phase1_stats.packet_count != 0U) ? 1U : 0U;
    out->edge_feed_ok =
        (s_final_edge_phase1_stats.edge_feed_count ==
         s_final_edge_phase1_stats.packet_count) ? 1U : 0U;
    out->b_scan_count_ok = (s_final_edge_b_scan_count != 0U) ? 1U : 0U;
    out->b_scan_fail_ok =
        (s_final_edge_b_scan_fail_count == 0U) ? 1U : 0U;
    out->b_last_err_ok =
        ((s_final_edge_b_last_result.err == 0U) ||
#if ZY100_FINAL_EDGE_PHASE4_HIT_WINDOW_ENABLE
         ((s_final_edge_nohit_od_warn_last_reason != 0U) &&
          (s_final_edge_nohit_od_warn_last_id == s_final_edge_last_b_id)) ||
         ((s_final_edge_truehit_od_warn_last_reason != 0U) &&
          (s_final_edge_truehit_od_warn_last_id == s_final_edge_last_b_id))
#else
         false
#endif
        ) ? 1U : 0U;
#if ZY100_FINAL_EDGE_PHASE4_HIT_WINDOW_ENABLE
    out->b_frame_ok = out->b_scan_fail_ok;
    out->b_hwin_ok = v0_final_edge_phase4_b_hwin_pass3b_ok();
    out->b_nohit1280_ok = v0_final_edge_phase4_nohit1280_pass3b_ok();
    out->b_truehit_short = v0_final_edge_phase4_truehit_short_pass3b();
    out->b_scan_fail_count = s_final_edge_b_scan_fail_count;
    out->b_fail_reason_or = s_final_edge_b_fail_reason_or;
    out->b_fail_last_id = s_final_edge_b_fail_last_id;
    out->b_fail_last_reason = s_final_edge_b_fail_last_reason;
    out->b_fail_hard_count = s_final_edge_b_fail_hard_count;
    out->b_fail_frame_count = s_final_edge_b_fail_frame_count;
    out->b_fail_hwin_count = s_final_edge_b_fail_hwin_count;
    out->b_fail_trunc_count = s_final_edge_b_fail_trunc_count;
    out->b_fail_last_hit = s_final_edge_b_fail_last_hit;
    out->b_fail_last_frames = s_final_edge_b_fail_last_frames;
    out->nohit_od_warn_count = s_final_edge_nohit_od_warn_count;
    out->nohit_od_warn_or = s_final_edge_nohit_od_warn_or;
    out->nohit_od_warn_last_id = s_final_edge_nohit_od_warn_last_id;
    out->nohit_od_warn_last_reason =
        s_final_edge_nohit_od_warn_last_reason;
    out->nohit_od_warn_last_run = s_final_edge_nohit_od_warn_last_run;
    out->truehit_od_warn_count = s_final_edge_truehit_od_warn_count;
    out->truehit_od_warn_or = s_final_edge_truehit_od_warn_or;
    out->truehit_od_warn_last_id = s_final_edge_truehit_od_warn_last_id;
    out->truehit_od_warn_last_reason =
        s_final_edge_truehit_od_warn_last_reason;
    out->truehit_od_warn_last_run = s_final_edge_truehit_od_warn_last_run;
#else
    out->b_frame_ok = out->b_scan_fail_ok;
    out->b_hwin_ok = 1U;
    out->b_nohit1280_ok =
        ((s_final_edge_b_scan_count != 0U) &&
         ((s_final_edge_b_last_result.hit_found != 0U) ||
          (s_final_edge_b_last_result.frames ==
           ZY100_FINAL_EDGE_B_SCAN_BASE_FRAMES))) ? 1U : 0U;
    out->b_truehit_short = 0U;
    out->b_scan_fail_count = s_final_edge_b_scan_fail_count;
    out->b_fail_reason_or = 0U;
    out->b_fail_last_id = 0U;
    out->b_fail_last_reason = 0U;
    out->b_fail_hard_count = 0U;
    out->b_fail_frame_count = 0U;
    out->b_fail_hwin_count = 0U;
    out->b_fail_trunc_count = 0U;
    out->b_fail_last_hit = 0U;
    out->b_fail_last_frames = 0U;
    out->nohit_od_warn_count = 0U;
    out->nohit_od_warn_or = 0U;
    out->nohit_od_warn_last_id = 0U;
    out->nohit_od_warn_last_reason = 0U;
    out->nohit_od_warn_last_run = 0U;
    out->truehit_od_warn_count = 0U;
    out->truehit_od_warn_or = 0U;
    out->truehit_od_warn_last_id = 0U;
    out->truehit_od_warn_last_reason = 0U;
    out->truehit_od_warn_last_run = 0U;
#endif
    out->restore_ok =
        ((s_final_edge_restore_fail_count == 0U) &&
         (s_final_edge_restore_ok_count == s_final_edge_b_scan_count)) ?
            1U : 0U;
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    {
        zy100_fe_raw_store_stats_t raw_stats;
        uint8_t raw_ok = 1U;

        memset(&raw_stats, 0, sizeof(raw_stats));
        zy100_final_edge_raw_store_get_stats(&raw_stats);
        if (s_final_edge_official_hit_count != 0U)
        {
            raw_ok =
                ((raw_stats.raw_saved == s_final_edge_official_hit_count) &&
                 (raw_stats.raw_begin == s_final_edge_official_hit_count) &&
                 (raw_stats.raw_failed == 0U) &&
                 (raw_stats.raw_full == 0U) &&
                 (raw_stats.raw_lf_fail == 0U) &&
                 (raw_stats.raw_hwin_fail == 0U) &&
                 (raw_stats.write_error == 0U) &&
                 (raw_stats.verify_error == 0U) &&
                 (raw_stats.pending == 0U) &&
                 (raw_stats.program_in_flight == 0U) &&
                 (raw_stats.b_active_pump_blocked == 0U)
#if !ZY100_FINAL_EDGE_PHASE7_EDGE_FLASH_ENABLE
                 &&
                 (s_final_edge_flash_stats.summary_flash == 0U) &&
                 (s_final_edge_flash_stats.event_flash == 0U)
#endif
                ) ? 1U : 0U;
        }
        out->flash_zero_ok =
            (v0_final_edge_flash_stats_all_zero() && raw_ok) ? 1U : 0U;
    }
#else
    out->flash_zero_ok = v0_final_edge_flash_stats_all_zero() ? 1U : 0U;
#endif
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
    {
        uint32_t nohit_count = s_final_edge_phase1_stats.replay_nohit_count;
        uint32_t nohit_expected =
            nohit_count * ZY100_FINAL_EDGE_NOHIT_REPLAY_EXPECT_SAMPLES;
        uint32_t truehit_count =
            s_final_edge_phase1_stats.replay_truehit_count;
        uint32_t edge_total =
            s_final_edge_phase1_stats.edge_feed_count +
            v0_final_edge_replay_edge_total();
        uint8_t nohit_done_ok;
        uint8_t truehit_done_ok;
        uint8_t fail_ok;
        uint8_t flash_no_summary_event =
#if ZY100_FINAL_EDGE_PHASE7_EDGE_FLASH_ENABLE
            1U;
#else
            ((s_final_edge_flash_stats.summary_flash == 0U) &&
             (s_final_edge_flash_stats.event_flash == 0U)) ? 1U : 0U;
#endif
        uint8_t fifo_ok =
            ((stats->fifo_full_count == 0U) &&
             (stats->fifo_lost_pkt_count == 0U) &&
             (s_final_edge_phase1_stats.fifo_emergency_count == 0U)) ?
                1U : 0U;

        out->phase6_seen =
            ((nohit_count != 0U) ||
             (s_final_edge_official_hit_count != 0U)) ? 1U : 0U;
        out->phase6_inc = (out->phase6_seen == 0U) ? 1U : 0U;
        if (out->phase6_seen == 0U)
        {
            out->phase6_replay_ok = 0U;
            out->phase6_done_ok = 0U;
            out->phase6_nohit_ok = 0U;
            out->phase6_ds_ok = 0U;
            out->phase6_edge_ok = 0U;
            out->phase6_pending_ok = 0U;
            out->phase6_gate_ok = 0U;
            out->phase6_backlog_ok = 0U;
            out->phase6_err = 0U;
        }
        else
        {
            if (nohit_count == 0U)
            {
                out->phase6_nohit_ok = 1U;
                nohit_done_ok = 1U;
            }
            else
            {
                out->phase6_nohit_ok =
                    ((s_final_edge_phase1_stats.replay_begin_count ==
                      nohit_count) &&
                     (s_final_edge_phase1_stats.replay_samples_expected ==
                      nohit_expected) &&
                     (s_final_edge_phase1_stats.replay_samples_fed ==
                      nohit_expected) &&
                     (s_final_edge_phase1_stats.replay_edge_feed ==
                      nohit_expected)) ? 1U : 0U;
                nohit_done_ok =
                    (s_final_edge_phase1_stats.replay_done_count ==
                     nohit_count) ? 1U : 0U;
            }
#if (ZY100_FINAL_EDGE_PHASE61_TRUEHIT_REPLAY_ENABLE && \
     ZY100_FINAL_EDGE_TRUEHIT_REPLAY_ENABLE)
            if (s_final_edge_official_hit_count == 0U)
            {
                out->phase6_truehit_ok = 1U;
                truehit_done_ok = 1U;
            }
            else
            {
                out->phase6_truehit_ok =
                    ((truehit_count != 0U) &&
                     (s_final_edge_phase1_stats.replay_truehit_begin_count ==
                      truehit_count) &&
                     (s_final_edge_phase1_stats.
                      replay_truehit_samples_fed ==
                      s_final_edge_phase1_stats.
                      replay_truehit_samples_expected) &&
                     (s_final_edge_phase1_stats.
                      replay_truehit_edge_feed ==
                      s_final_edge_phase1_stats.
                      replay_truehit_samples_expected)) ? 1U : 0U;
                truehit_done_ok =
                    (s_final_edge_phase1_stats.replay_truehit_done_count ==
                     s_final_edge_phase1_stats.replay_truehit_begin_count) ?
                        1U : 0U;
            }
#else
            out->phase6_truehit_ok = 1U;
            truehit_done_ok = 1U;
#endif
            out->phase6_replay_ok =
                ((out->phase6_nohit_ok != 0U) &&
                 (out->phase6_truehit_ok != 0U)) ? 1U : 0U;
            out->phase6_done_ok =
                ((nohit_done_ok != 0U) &&
                 (truehit_done_ok != 0U)) ? 1U : 0U;
            out->phase6_ds_ok =
                ((s_final_edge_phase1_stats.replay_ds_bad_count == 0U) &&
                 (s_final_edge_phase1_stats.replay_ds_overflow_count == 0U)) ?
                    1U : 0U;
            out->phase6_edge_ok =
                ((s_final_edge_phase1_stats.edge_feed_count ==
                  s_final_edge_phase1_stats.packet_count) &&
                 (s_final_edge_phase1_stats.replay_edge_error == 0U) &&
                 (s_final_edge_phase1_stats.replay_truehit_edge_error == 0U) &&
                 (zy100_edge_shadow_accepted_count() == edge_total)) ?
                    1U : 0U;
            out->phase6_pending_ok =
                (!s_final_edge_replay_pending &&
                 !s_final_edge_replay_active) ? 1U : 0U;
            out->phase6_gate_ok = flash_no_summary_event;
            out->phase6_backlog_ok =
                s_final_edge_replay_backlog_edge_only_active ? 0U : 1U;
            fail_ok =
                ((s_final_edge_phase1_stats.replay_fail_count == 0U) &&
                 (s_final_edge_phase1_stats.replay_truehit_fail_count == 0U)) ?
                    1U : 0U;
            out->phase6_err =
                ((fail_ok != 0U) &&
                 (out->phase6_replay_ok != 0U) &&
                 (out->phase6_done_ok != 0U) &&
                 (out->phase6_ds_ok != 0U) &&
                 (out->phase6_edge_ok != 0U) &&
                 (out->phase6_pending_ok != 0U) &&
                 (out->phase6_gate_ok != 0U) &&
                 (out->phase6_backlog_ok != 0U) &&
                 (flash_no_summary_event != 0U) &&
                 (fifo_ok != 0U)) ? 0U : 1U;
        }
    }
#else
    out->phase6_replay_ok = 1U;
    out->phase6_done_ok = 1U;
    out->phase6_nohit_ok = 1U;
    out->phase6_truehit_ok = 1U;
    out->phase6_ds_ok = 1U;
    out->phase6_edge_ok = 1U;
    out->phase6_pending_ok = 1U;
    out->phase6_gate_ok = 1U;
    out->phase6_backlog_ok = 1U;
    out->phase6_err = 0U;
#endif
#if ZY100_FINAL_EDGE_PHASE7_EDGE_FLASH_ENABLE
    {
        zy100_fe_record_store_stats_t record_stats;
        zy100_final_edge_queue_stats_t queue_stats;

        memset(&record_stats, 0, sizeof(record_stats));
        memset(&queue_stats, 0, sizeof(queue_stats));
        zy100_final_edge_record_store_get_stats(&record_stats);
        zy100_final_edge_queue_get_stats(&queue_stats);

        out->phase7_summary_saved = record_stats.summary_saved;
        out->phase7_summary_enqueued = queue_stats.summary_enqueued;
        out->phase7_event_saved = record_stats.event_saved;
        out->phase7_event_enqueued = queue_stats.event_enqueued;
        out->phase7_pending =
            record_stats.pending + record_stats.program_in_flight;
        out->phase7_full =
            record_stats.summary_full + record_stats.event_full;
        out->phase7_verify =
            record_stats.verify_error + record_stats.meta_verify_error;
        out->phase7_write =
            record_stats.write_error + record_stats.meta_write_error;
        out->phase7_bflash = record_stats.b_active_pump_blocked;
        out->phase7_runtime_erase = record_stats.runtime_erase_count;

        out->phase7_seen =
            ((queue_stats.summary_enqueued != 0U) ||
             (queue_stats.event_enqueued != 0U)) ? 1U : 0U;
        out->phase7_inc = (out->phase7_seen == 0U) ? 1U : 0U;
        out->phase7_summary_ok =
            ((record_stats.summary_saved == queue_stats.summary_enqueued) &&
             (record_stats.summary_failed == 0U) &&
             (record_stats.summary_full == 0U) &&
             (queue_stats.summary_dropped == 0U)) ? 1U : 0U;
        out->phase7_event_ok =
            ((record_stats.event_saved == queue_stats.event_enqueued) &&
             (record_stats.event_failed == 0U) &&
             (record_stats.event_full == 0U) &&
             (queue_stats.event_dropped == 0U)) ? 1U : 0U;
        out->phase7_meta_ok =
            ((record_stats.meta_write_ok != 0U) &&
             (record_stats.meta_verify_ok != 0U)) ? 1U : 0U;
        out->phase7_err =
            ((out->phase7_summary_ok != 0U) &&
             (out->phase7_event_ok != 0U) &&
             (out->phase7_meta_ok != 0U) &&
             (out->phase7_pending == 0U) &&
             (out->phase7_full == 0U) &&
             (out->phase7_verify == 0U) &&
             (out->phase7_write == 0U) &&
             (out->phase7_bflash == 0U) &&
             (out->phase7_runtime_erase == 0U) &&
             (record_stats.record_final_pump_timeout == 0U) &&
             (record_stats.record_final_pending_left == 0U)) ? 0U : 1U;
    }
#else
    out->phase7_seen = 1U;
    out->phase7_summary_ok = 1U;
    out->phase7_event_ok = 1U;
    out->phase7_meta_ok = 1U;
    out->phase7_err = 0U;
    out->phase7_inc = 0U;
#endif
    {
        zy100_fe_raw_store_stats_t raw_stats;
        zy100_fe_record_store_stats_t record_stats;

        memset(&raw_stats, 0, sizeof(raw_stats));
        memset(&record_stats, 0, sizeof(record_stats));
        zy100_final_edge_raw_store_get_stats(&raw_stats);
        zy100_final_edge_record_store_get_stats(&record_stats);

        out->phase8_stop = s_fe_stop_seen ? 1U : 0U;
        out->phase8_drain = s_fe_final_drain_done ? 1U : 0U;
        out->phase8_export = s_fe_export_ready ? 1U : 0U;
        out->phase8_clear = s_fe_export_cleared ? 1U : 0U;
        out->phase8_err =
            (s_fe_capture_state == FE_CAPTURE_STATE_ERROR) ? 1U : 0U;
        out->phase8_reason_ok =
            ((s_fe_stop_reason == ZY100_FE_STOP_REASON_USER_SHORT_PRESS) ||
             (s_fe_stop_reason == ZY100_FE_STOP_REASON_FLASH_RAW_95) ||
             (s_fe_stop_reason == ZY100_FE_STOP_REASON_FLASH_SUMMARY_95) ||
             (s_fe_stop_reason == ZY100_FE_STOP_REASON_FLASH_EVENT_95) ||
             (s_fe_stop_reason == ZY100_FE_STOP_REASON_FLASH_META_95) ||
             (s_fe_stop_reason == ZY100_FE_STOP_REASON_BATTERY_LOW)) ?
                1U : 0U;
        out->phase8_ready =
            (s_fe_capture_state == FE_CAPTURE_STATE_EXPORT_READY) ? 1U : 0U;
        out->phase8_exporting =
            (s_fe_capture_state == FE_CAPTURE_STATE_EXPORTING) ? 1U : 0U;
        out->phase8_done =
            (s_fe_capture_state == FE_CAPTURE_STATE_EXPORT_DONE) ? 1U : 0U;
        out->phase8_preserved = s_fe_export_preserved ? 1U : 0U;
        out->phase8_cleared = s_fe_export_cleared ? 1U : 0U;
        out->phase8_raw_idle =
            ((raw_stats.pending == 0U) &&
             (raw_stats.program_in_flight == 0U) &&
             (raw_stats.finalizing == 0U)) ? 1U : 0U;
        out->phase8_rec_idle =
            ((record_stats.pending == 0U) &&
             (record_stats.program_in_flight == 0U)) ? 1U : 0U;
        out->phase8_meta_ok =
            ((record_stats.meta_write_ok != 0U) &&
             (record_stats.meta_verify_ok != 0U)) ? 1U : 0U;
        out->phase8_fifo_safe =
            ((stats->fifo_full_count == 0U) &&
             (stats->fifo_lost_pkt_count == 0U) &&
             (s_final_edge_phase1_stats.fifo_emergency_count == 0U)) ?
                1U : 0U;
        out->phase8_b_idle =
#if ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
            s_final_edge_b_scan_active ? 0U : 1U;
#else
            1U;
#endif
    }

    out->final_pass =
        (out->reached_summary_ok &&
         out->fatal_ok &&
         out->fifo_full_ok &&
         out->fifo_lost_ok &&
         out->bad_header_ok &&
         out->dt_seen_ok &&
         out->dt_bad_ok &&
         out->dt_range_ok &&
         out->packet_nonzero_ok &&
         out->edge_feed_ok &&
         out->b_scan_count_ok &&
         out->b_scan_fail_ok &&
         out->b_last_err_ok &&
         out->restore_ok &&
         out->flash_zero_ok &&
         (out->phase6_err == 0U) &&
         (out->phase7_seen != 0U) &&
         (out->phase7_summary_ok != 0U) &&
         (out->phase7_event_ok != 0U) &&
         (out->phase7_meta_ok != 0U) &&
         (out->phase7_err == 0U) &&
         (out->phase7_inc == 0U) &&
         (out->phase8_stop != 0U) &&
         (out->phase8_drain != 0U) &&
         (out->phase8_export != 0U) &&
         (out->phase8_err == 0U) &&
         (out->phase8_reason_ok != 0U) &&
         (out->phase8_raw_idle != 0U) &&
         (out->phase8_rec_idle != 0U) &&
         (out->phase8_meta_ok != 0U) &&
         (out->phase8_fifo_safe != 0U) &&
         (out->phase8_b_idle != 0U)) ? 1U : 0U;

    return out->final_pass != 0U;
#else
    IMU_UNUSED(stats);
    return false;
#endif
}

static void v0_final_edge_log_pass_diag(
    const v0_final_edge_pass_diag_t *diag)
{
    if (diag == NULL)
    {
        return;
    }

#if ZY100_LOG_FE_DIAG_VERBOSE
    ZY100_DIAG_LOG("[FE_PASS] pass=%u sum=%u fatal=%u full=%u lost=%u hdr=%u",
               (uint32_t)diag->final_pass,
               (uint32_t)diag->reached_summary_ok,
               (uint32_t)diag->fatal_ok,
               (uint32_t)diag->fifo_full_ok,
               (uint32_t)diag->fifo_lost_ok,
               (uint32_t)diag->bad_header_ok);
    ZY100_DIAG_LOG("[FE_PASS2] dt_seen=%u dt_bad=%u dt_rng=%u pkt=%u edge=%u",
               (uint32_t)diag->dt_seen_ok,
               (uint32_t)diag->dt_bad_ok,
               (uint32_t)diag->dt_range_ok,
               (uint32_t)diag->packet_nonzero_ok,
               (uint32_t)diag->edge_feed_ok);
    DBG_DIRECT("[FE_PASS3] b_cnt=%u b_fail=%u b_err=%u restore=%u flash=%u",
               (uint32_t)diag->b_scan_count_ok,
               (uint32_t)diag->b_scan_fail_ok,
               (uint32_t)diag->b_last_err_ok,
               (uint32_t)diag->restore_ok,
               (uint32_t)diag->flash_zero_ok);
#if ZY100_FINAL_EDGE_PHASE4_HIT_WINDOW_ENABLE
    DBG_DIRECT("[FE_PASS3B] b_frame=%u b_hwin=%u nohit1280=%u short=%u fail=%u",
               (uint32_t)diag->b_frame_ok,
               (uint32_t)diag->b_hwin_ok,
               (uint32_t)diag->b_nohit1280_ok,
               (uint32_t)diag->b_truehit_short,
               diag->b_scan_fail_count);
    ZY100_DIAG_LOG("[FE_PASS3C] r_or=%08x last_id=%u last_r=%08x hard=%u",
               diag->b_fail_reason_or,
               diag->b_fail_last_id,
               diag->b_fail_last_reason,
               diag->b_fail_hard_count);
    ZY100_DIAG_LOG("[FE_PASS3D] frame=%u hwin=%u trunc=%u last_hit=%u last_frames=%u",
               diag->b_fail_frame_count,
               diag->b_fail_hwin_count,
               diag->b_fail_trunc_count,
               diag->b_fail_last_hit,
               diag->b_fail_last_frames);
    DBG_DIRECT("[FE_PASS3E] nohit_od_warn=%u od_or=%08x last_id=%u last_r=%08x last_run=%u",
               diag->nohit_od_warn_count,
               diag->nohit_od_warn_or,
               diag->nohit_od_warn_last_id,
               diag->nohit_od_warn_last_reason,
               diag->nohit_od_warn_last_run);
    DBG_DIRECT("[FE_PASS3F] truehit_od_warn=%u od_or=%08x last_id=%u last_r=%08x last_run=%u",
               diag->truehit_od_warn_count,
               diag->truehit_od_warn_or,
               diag->truehit_od_warn_last_id,
               diag->truehit_od_warn_last_reason,
               diag->truehit_od_warn_last_run);
#endif
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
    DBG_DIRECT("[FE_PASS6] seen=%u replay=%u done=%u err=%u inc=%u",
               (uint32_t)diag->phase6_seen,
               (uint32_t)diag->phase6_replay_ok,
               (uint32_t)diag->phase6_done_ok,
               (uint32_t)diag->phase6_err,
               (uint32_t)diag->phase6_inc);
    ZY100_DIAG_LOG("[FE_PASS6B] nh=%u hit=%u ds=%u edge=%u pending=%u backlog=%u",
               (uint32_t)diag->phase6_nohit_ok,
               (uint32_t)diag->phase6_truehit_ok,
               (uint32_t)diag->phase6_ds_ok,
               (uint32_t)diag->phase6_edge_ok,
               (uint32_t)diag->phase6_pending_ok,
               (uint32_t)diag->phase6_backlog_ok);
#endif
#if ZY100_FINAL_EDGE_PHASE7_EDGE_FLASH_ENABLE
    DBG_DIRECT("[FE_PASS7] seen=%u sum=%u evt=%u meta=%u err=%u inc=%u",
               (uint32_t)diag->phase7_seen,
               (uint32_t)diag->phase7_summary_ok,
               (uint32_t)diag->phase7_event_ok,
               (uint32_t)diag->phase7_meta_ok,
               (uint32_t)diag->phase7_err,
               (uint32_t)diag->phase7_inc);
    ZY100_DIAG_LOG("[FE_PASS7B] sum_saved=%u sum_enq=%u evt_saved=%u evt_enq=%u pending=%u",
               diag->phase7_summary_saved,
               diag->phase7_summary_enqueued,
               diag->phase7_event_saved,
               diag->phase7_event_enqueued,
               diag->phase7_pending);
    ZY100_DIAG_LOG("[FE_PASS7C] full=%u verify=%u write=%u bflash=%u rterase=%u",
               diag->phase7_full,
               diag->phase7_verify,
               diag->phase7_write,
               diag->phase7_bflash,
               diag->phase7_runtime_erase);
#endif
    DBG_DIRECT("[FE_PASS8] stop=%u drain=%u export=%u clear=%u err=%u reason=%u",
               (uint32_t)diag->phase8_stop,
               (uint32_t)diag->phase8_drain,
               (uint32_t)diag->phase8_export,
               (uint32_t)diag->phase8_clear,
               (uint32_t)diag->phase8_err,
               (uint32_t)diag->phase8_reason_ok);
    ZY100_DIAG_LOG("[FE_PASS8B] ready=%u exporting=%u done=%u preserved=%u cleared=%u",
               (uint32_t)diag->phase8_ready,
               (uint32_t)diag->phase8_exporting,
               (uint32_t)diag->phase8_done,
               (uint32_t)diag->phase8_preserved,
               (uint32_t)diag->phase8_cleared);
    ZY100_DIAG_LOG("[FE_PASS8C] raw_idle=%u rec_idle=%u meta=%u fifo=%u b=%u",
               (uint32_t)diag->phase8_raw_idle,
               (uint32_t)diag->phase8_rec_idle,
               (uint32_t)diag->phase8_meta_ok,
               (uint32_t)diag->phase8_fifo_safe,
               (uint32_t)diag->phase8_b_idle);
#endif
}
#endif

#if ZY100_FINAL_EDGE_MODE_ENABLE
static bool v0_data_passed(const v0_fifo_stats_t *stats,
                           uint32_t runtime_ms,
                           v0_final_edge_pass_diag_t *diag)
{
    IMU_UNUSED(runtime_ms);
    return v0_final_edge_phase3_data_passed(stats, diag);
}
#else
static bool v0_data_passed(const v0_fifo_stats_t *stats, uint32_t runtime_ms)
{
    uint32_t expected_packets = v0_expected_packets(runtime_ms);
    uint32_t min_packets = (expected_packets * V0_PACKET_THRESHOLD_PERCENT) / 100U;

    return stats->reached_final_summary &&
           (stats->fifo_lost_pkt_count == 0U) &&
           (stats->fifo_full_count == 0U) &&
           (stats->bad_header_count == 0U) &&
           (stats->ts_jump_count == 0U) &&
           (stats->read_error_count == 0U) &&
           (stats->packet_count >= min_packets);
}
#endif

static bool v0_wm_irq_passed(const v0_fifo_stats_t *stats)
{
    return stats->reached_final_summary &&
           (stats->irq_count > 0U) &&
           (stats->irq_drain_count >= V0_WM_IRQ_MIN_DRAIN_COUNT) &&
           (stats->poll_rescue_count == 0U);
}
#endif

#if V0_FLASH_RUNTIME_ENABLE
static void v1_copy_imu_stats(const v0_fifo_stats_t *stats,
                              external_flash_capture_imu_stats_t *out)
{
    if ((stats == NULL) || (out == NULL))
    {
        return;
    }

    out->packet_count = stats->packet_count;
    out->bytes_read = stats->bytes_read;
    out->fifo_full_count = stats->fifo_full_count;
    out->bad_header_count = stats->bad_header_count;
    out->ts_jump_count = stats->ts_jump_count;
    out->read_error_count = stats->read_error_count;
    out->max_loop_gap_ms = stats->max_loop_gap_ms;
    out->loop_gap_over_10ms_count = stats->loop_gap_over_10ms_count;
    out->loop_gap_over_20ms_count = stats->loop_gap_over_20ms_count;
    out->queue_max_level = stats->queue_max_level;
    out->max_fifo_count = stats->max_fifo_count;
    out->fifo_lost_pkt_count = stats->fifo_lost_pkt_count;
}
#endif

static imu_status_t v0_imu_platform_resume_for_capture(uint32_t round)
{
    imu_status_t status;
    uint8_t who_am_i = 0U;
    bool probe_logged = false;

    s_v0_hw_ready = false;
    V0_IMU_LIFECYCLE_LOG("[V0_PWR] v0 hw resume begin round=%u", round);

    /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
    status = imu_bsp_power_ctrl(true);
    if (status != IMU_STATUS_OK)
    {
        goto fail;
    }

    imu_bsp_delay_us(5000U);
    V0_IMU_LIFECYCLE_LOG("[V0_PWR] sensor power on");

    imu_bsp_cs_high();
    status = imu_bsp_flash_cs_hold_high();
    if (status != IMU_STATUS_OK)
    {
        goto fail;
    }

    status = imu_bsp_resume_after_dlps();
    if (status != IMU_STATUS_OK)
    {
        goto fail;
    }
    V0_IMU_LIFECYCLE_LOG("[V0_PWR] spi/pad restored");

    imu_bsp_cs_high();
    status = imu_bsp_flash_cs_hold_high();
    if (status != IMU_STATUS_OK)
    {
        goto fail;
    }

    status = imu_bsp_int_init();
    if (status != IMU_STATUS_OK)
    {
        goto fail;
    }
    imu_bsp_int_clear_pending();
    imu_bsp_int_register_irq_callback(v0_imu_irq_callback);

    status = imu_bsp_spi_set_mode(SPI_CPOL_Low, SPI_CPHA_1Edge);
    if (status != IMU_STATUS_OK)
    {
        goto fail;
    }

    status = icm53611_read_whoami(&who_am_i);
    V0_IMU_LIFECYCLE_LOG("[V0_PWR] imu resume probe who=0x%02x status=%u",
                         who_am_i,
                         (uint32_t)status);
    probe_logged = true;
    if ((status != IMU_STATUS_OK) || (who_am_i != ICM53611_WHO_AM_I_VALUE))
    {
        status = (status != IMU_STATUS_OK) ? status : IMU_STATUS_NOT_FOUND;
        goto fail;
    }

    s_v0_hw_ready = true;
    return IMU_STATUS_OK;

fail:
    if (!probe_logged)
    {
        V0_IMU_LIFECYCLE_LOG("[V0_PWR] imu resume probe who=0x%02x status=%u",
                             who_am_i,
                             (uint32_t)status);
    }
    DBG_DIRECT("[V0_PWR][ERR] imu resume probe failed round=%u status=%u",
               round,
               (uint32_t)status);
    imu_bsp_int_register_irq_callback(NULL);
    s_v0_hw_ready = false;
    return status;
}

static void v0_fill_phase_a_ui_fifo_config(icm53611_cfg_t *imu_cfg,
                                           icm53611_fifo_cfg_t *fifo_cfg)
{
    if (imu_cfg != NULL)
    {
        memset(imu_cfg, 0, sizeof(*imu_cfg));
        imu_cfg->pwr_mgmt0 = (ICM53611_PWR_MGMT0_ACCEL_MODE_LN |
                              ICM53611_PWR_MGMT0_GYRO_MODE_LN);
        imu_cfg->gyro_config0 = (ICM53611_GYRO_FSR_2000DPS |
                                 ICM53611_GYRO_ODR_800HZ);
        imu_cfg->accel_config0 = (ICM53611_ACCEL_FSR_16G |
                                  ICM53611_ACCEL_ODR_800HZ);
        imu_cfg->gyro_config1 = ICM53611_GYRO_CONFIG1_DEFAULT;
        imu_cfg->accel_config1 = ICM53611_ACCEL_CONFIG1_DEFAULT;
    }
    if (fifo_cfg != NULL)
    {
        memset(fifo_cfg, 0, sizeof(*fifo_cfg));
        fifo_cfg->enable = true;
        fifo_cfg->timestamp_enable = true;
        fifo_cfg->accel_enable = true;
        fifo_cfg->gyro_enable = true;
        fifo_cfg->hires_enable = false;
        fifo_cfg->fsync_timestamp_enable = false;
#if ZY100_FINAL_EDGE_MODE_ENABLE
        fifo_cfg->watermark = ZY100_FINAL_EDGE_INT_HIGH_PRESSURE_WATERMARK_BYTES;
#else
        fifo_cfg->watermark = V0_FIFO_WATERMARK_BYTES;
#endif
    }
}

#if IMU_WOM_FIFO_ORDER_DIAG_ENABLE
static const char *v0_diag_pass_fail(bool pass)
{
    return pass ? "PASS" : "FAIL";
}

static bool v0_wom_threshold_codes_match(uint8_t x,
                                         uint8_t y,
                                         uint8_t z,
                                         uint8_t expected)
{
    return ((x == expected) && (y == expected) && (z == expected));
}

static void v0_wom_fifo_order_diag(const icm53611_cfg_t *imu_cfg,
                                   const icm53611_fifo_cfg_t *fifo_cfg,
                                   const imu_wom_irq_config_t *wom_cfg)
{
    imu_status_t status;
    imu_status_t read_status;
    uint8_t expected;
    uint8_t x = 0U;
    uint8_t y = 0U;
    uint8_t z = 0U;
    bool before_pass;
    bool after_read_pass;
    const char *summary;

    ZY100_DIAG_LOG("[IMU_WOM_ORDER_DIAG] begin");
    status = imu_wom_irq_configure_sensor(wom_cfg);
    expected = imu_wom_irq_get_threshold_code_for_diag();
    read_status = icm53611_wom_read_threshold_codes(&x, &y, &z);
    before_pass = ((status == IMU_STATUS_OK) &&
                   (read_status == IMU_STATUS_OK) &&
                   v0_wom_threshold_codes_match(x, y, z, expected));
    DBG_DIRECT("[IMU_WOM_ORDER_DIAG] phase=before_fifo write_verify=%s status=%u read_status=%u",
               v0_diag_pass_fail(before_pass),
               (uint32_t)status,
               (uint32_t)read_status);
    ZY100_DIAG_LOG("[IMU_WOM_ORDER_DIAG] phase=before_fifo read_x=0x%02x read_y=0x%02x read_z=0x%02x expected=0x%02x",
               x,
               y,
               z,
               expected);

    status = icm53611_configure_packet3_fifo_stream(imu_cfg, fifo_cfg);
    ZY100_DIAG_LOG("[IMU_WOM_ORDER_DIAG] phase=fifo_stream_config status=%u", (uint32_t)status);
    read_status = icm53611_wom_read_threshold_codes(&x, &y, &z);
    after_read_pass = ((status == IMU_STATUS_OK) &&
                       (read_status == IMU_STATUS_OK) &&
                       v0_wom_threshold_codes_match(x, y, z, expected));
    DBG_DIRECT("[IMU_WOM_ORDER_DIAG] phase=after_fifo_read read=%s status=%u",
               v0_diag_pass_fail(after_read_pass),
               (uint32_t)read_status);
    ZY100_DIAG_LOG("[IMU_WOM_ORDER_DIAG] phase=after_fifo_read read_x=0x%02x read_y=0x%02x read_z=0x%02x expected=0x%02x",
               x,
               y,
               z,
               expected);

    if (before_pass && !after_read_pass)
    {
        summary = "BEFORE_PASS_AFTER_FAIL";
    }
    else if (before_pass && after_read_pass)
    {
        summary = "PASS_PERSISTED_AFTER_FIFO";
    }
    else
    {
        summary = "THRESHOLD_VERIFY_FAIL";
    }
    ZY100_DIAG_LOG("[IMU_WOM_ORDER_DIAG] result=%s", summary);
    ZY100_DIAG_LOG("[IMU_WOM_ORDER_DIAG] end");
}
#endif


static bool v0_start_capture(v0_fifo_stats_t *stats)
{
    imu_status_t status;
    icm53611_capture_result_t prepare_result = {0};
    prepare_result.io.acquire_status = prepare_result.io.release_status = 0xffU;
    prepare_result.stage = ICM53611_CAPTURE_PLATFORM;
    uint8_t who_am_i = 0U;
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    const bool continuous_raw = zy100_online_raw_capture_active();
#else
    const bool continuous_raw = false;
#endif
#if V0_FIFO_MARKER_TRIGGER_ENABLED
    icm53611_int_status_t clear_status;
    const imu_wom_irq_config_t *wom_cfg = imu_wom_irq_default_config();
#endif
    icm53611_cfg_t imu_cfg;
    icm53611_fifo_cfg_t fifo_cfg;

#if !ZY100_IMU_LIFECYCLE_LOG_ENABLE
    (void)v0_spi_sched_mode_name;
#endif
    v0_fill_phase_a_ui_fifo_config(&imu_cfg, &fifo_cfg);

    /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
    status = imu_bsp_power_ctrl(true);
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
        goto start_failed;
    }
    imu_bsp_delay_us(5000U);

    status = imu_bsp_flash_cs_hold_high();
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
        goto start_failed;
    }

    status = icm53611_init_bus();
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
        goto start_failed;
    }

    status = imu_bsp_flash_cs_hold_high();
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
        goto start_failed;
    }

    imu_bsp_int_register_irq_callback(NULL);
    prepare_result.stage = ICM53611_CAPTURE_IRQ;
    if (!imu_bsp_int_disable_for_dlps())
    {
        v0_mark_read_error(stats, IMU_STATUS_NOT_READY);
        goto start_failed;
    }
    prepare_result.stage = ICM53611_CAPTURE_PLATFORM;
    imu_bsp_int_clear_pending();
#if V0_FIFO_MARKER_TRIGGER_ENABLED
    if (!continuous_raw)
    {
        imu_wom_irq_reset(wom_cfg);
        imu_sample_marker_reset();
        IMU_UNUSED(imu_pre_trigger_marker_reset(
            imu_pre_trigger_marker_default_config()));
        IMU_UNUSED(imu_swing_detector_reset(
            imu_swing_detector_default_config()));
    }
#endif
    s_v0_current_sample_seq = 0U;
#if V0_FIFO_MARKER_TRIGGER_ENABLED
    s_v0_detector_rate_accum = 0U;
    s_v0_detector_sample_base_ts_us = imu_bsp_local_timestamp_us();
#endif
    imu_bsp_int_clear_pending();

    status = imu_bsp_spi_set_mode(SPI_CPOL_Low, SPI_CPHA_1Edge);
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
        goto start_failed;
    }

    status = icm53611_read_whoami(&who_am_i);
    if ((status != IMU_STATUS_OK) || (who_am_i != ICM53611_WHO_AM_I_VALUE))
    {
        v0_mark_read_error(stats, (status != IMU_STATUS_OK) ? status : IMU_STATUS_NOT_FOUND);
        goto start_failed;
    }

    /* Legacy diagnostic/marker paths retain their own reset and setup order. */
    if (!continuous_raw || ZY100_RT_MARKER_TIMER_ONLY_TEST)
    {
        status = icm53611_soft_reset();
        if (status != IMU_STATUS_OK)
        {
            v0_mark_read_error(stats, status);
            goto start_failed;
        }
    }

#if V0_FIFO_MARKER_TRIGGER_ENABLED
    if (!continuous_raw)
    {
        imu_wom_irq_run_pre_fifo_mreg_diag();
#if IMU_WOM_FIFO_ORDER_DIAG_ENABLE
        v0_wom_fifo_order_diag(&imu_cfg, &fifo_cfg, wom_cfg);
#endif

        status = imu_wom_irq_configure_sensor(wom_cfg);
        if (status != IMU_STATUS_OK)
        {
#if IMU_WOM_STRICT_CONFIG_VERIFY
            v0_mark_read_error(stats, status);
            goto start_failed;
#else
            imu_wom_irq_log_configure_failed(status);
            imu_wom_irq_run_mreg_diag();
            IMU_UNUSED(imu_wom_irq_disable_sensor());
            DBG_DIRECT("[WARN][IMU] wom_configure_failed continue_capture=1");
#endif
        }
        else
        {
            status = imu_wom_irq_enable_route();
            if (status != IMU_STATUS_OK)
            {
#if IMU_WOM_STRICT_CONFIG_VERIFY
                v0_mark_read_error(stats, status);
                goto start_failed;
#else
                imu_wom_irq_log_configure_failed(status);
                IMU_UNUSED(imu_wom_irq_disable_sensor());
                DBG_DIRECT("[WARN][IMU] wom_route_failed status=%u continue_capture=1",
                           (uint32_t)status);
#endif
            }
            else
            {
                status = icm53611_read_int_status(&clear_status);
                if (status != IMU_STATUS_OK)
                {
#if IMU_WOM_STRICT_CONFIG_VERIFY
                    v0_mark_read_error(stats, status);
                    goto start_failed;
#else
                    imu_wom_irq_log_configure_failed(status);
                    IMU_UNUSED(imu_wom_irq_disable_sensor());
                    DBG_DIRECT("[WARN][IMU] wom_clear_pending_failed status=%u continue_capture=1",
                               (uint32_t)status);
#endif
                }
                else
                {
                    IMU_UNUSED(clear_status);
                    /* WOM is routed and old pending status is clear before FIFO starts. */
                }
            }
        }
    }
#endif

#if V0_FIFO_MARKER_TRIGGER_ENABLED
    s_v0_detector_sample_base_ts_us = imu_bsp_local_timestamp_us();
#endif
    zy100_spi_sched_reset(zy100_os_time_ms());
    zy100_spi_sched_invalidate_fifo_count();

    if (ZY100_RT_MARKER_TIMER_ONLY_TEST)
    {
        status = icm53611_apply_basic_config(&imu_cfg);
    }
    else if (continuous_raw)
    {
        status = icm53611_prepare_capture(&imu_cfg, &fifo_cfg, &prepare_result);
    }
    else
    {
        status = icm53611_configure_packet3_fifo_stream(&imu_cfg, &fifo_cfg);
    }
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
        goto start_failed;
    }

    prepare_result.stage = ICM53611_CAPTURE_CONFIG;
    if (!ZY100_RT_MARKER_TIMER_ONLY_TEST &&
        !v0_verify_ui_fifo_rate_config(stats))
    {
        goto start_failed;
    }

    prepare_result.stage = ICM53611_CAPTURE_LOST_COUNT;
    if (!ZY100_RT_MARKER_TIMER_ONLY_TEST && !v0_refresh_lost_count(stats))
    {
        goto start_failed;
    }

#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    prepare_result.stage = ICM53611_CAPTURE_FIRST_PACKET;
    if (continuous_raw && !v0_online_read_first_packet_bounded(stats))
    {
        goto start_failed;
    }
#endif

    prepare_result.stage = ICM53611_CAPTURE_IRQ;
    zy100_spi_sched_mark_session_start(zy100_os_time_ms());
    zy100_spi_sched_invalidate_fifo_count();

    imu_bsp_int_clear_pending();
    if (ZY100_RT_MARKER_TIMER_ONLY_TEST)
    {
        imu_bsp_int_register_irq_callback(NULL);
    }
    else
    {
        imu_bsp_int_register_irq_callback(v0_imu_irq_callback);
        status = imu_bsp_int_init();
        if (status != IMU_STATUS_OK)
        {
            v0_mark_read_error(stats, status);
            goto start_failed;
        }
    }
#if V0_FIFO_MARKER_TRIGGER_ENABLED
    if (!continuous_raw)
    {
        const imu_pre_trigger_marker_config_t *marker_cfg =
            imu_pre_trigger_marker_default_config();

        imu_wom_irq_log_config();
        DBG_DIRECT("[V0_DETECTOR_CFG] fifo_sample_hz=%u detector_input_hz=%u "
                   "marker_confirm_window_ms=%u gyro_confirm_peak_dps_min=%u "
                   "acc_step_window_ms=%u",
                   V0_SAMPLE_RATE_HZ,
                   imu_swing_detector_input_hz(),
                   marker_cfg->confirm_window_ms,
                   marker_cfg->gyro_confirm_peak_dps_min,
                   marker_cfg->acc_step_window_ms);
    }
#endif

#if !V0_FLASH_RUNTIME_ENABLE && ZY100_IMU_STARTUP_DBG_LOG_ENABLE
    v0_dbg_log_init_readback();
    v0_dbg_fifo_count_after_start();
#endif

    prepare_result.stage = ICM53611_CAPTURE_TIMER;
    stats->capture_active = true;
    s_v0_hw_ready = true;
#if ZY100_RT_MARKER_ENABLE
    if (!continuous_raw)
    {
        IMU_UNUSED(imu_rt_marker_reset(0ULL));
#if ZY100_RT_MARKER_TIMER_ENABLE
        zy100_rt_marker_timer_register_notify_callback(
            v0_rt_marker_timer_notify_from_isr);
        status = zy100_rt_marker_timer_config(ZY100_RT_MARKER_TARGET_HZ);
        if (status != IMU_STATUS_OK)
        {
            v0_mark_read_error(stats, status);
            goto start_failed;
        }
        zy100_rt_marker_timer_reset_counters();
        if (v0_rt_start_delay_enabled())
        {
            DBG_DIRECT("[RT_START_DELAY] reason=wait_first_fifo_drain");
        }
        else
        {
            status = zy100_rt_marker_timer_start();
            if (status != IMU_STATUS_OK)
            {
                v0_mark_read_error(stats, status);
                goto start_failed;
            }
        }
#endif
    }
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_FINAL_EDGE_TIMER_ENABLE
    if (!v0_final_edge_timer_start())
    {
        v0_mark_read_error(stats, IMU_STATUS_NOT_READY);
        goto start_failed;
    }
#endif
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE && !ZY100_FINAL_EDGE_MODE_ENABLE
    if (continuous_raw && !v0_online_live_timer_start())
    {
        v0_mark_read_error(stats, IMU_STATUS_NOT_READY);
        goto start_failed;
    }
#endif
    V0_IMU_LIFECYCLE_LOG("[SPI_SCHED_CFG] mode=%s rt=%u tim6=%u fifo=%u flash=%u ois=%u fifo_capacity=%u wm=%u flash_safe=%u emergency=%u",
                         v0_spi_sched_mode_name(),
                         (uint32_t)(ZY100_RT_MARKER_ENABLE != 0),
                         (uint32_t)((ZY100_RT_MARKER_ENABLE != 0) &&
                                    (ZY100_RT_MARKER_TIMER_ENABLE != 0)),
                         V0_SAMPLE_RATE_HZ,
                         (uint32_t)(V0_FLASH_RUNTIME_ENABLE != 0),
                         (uint32_t)ZY100_OIS_ENABLE,
                         ZY100_FIFO_CAPACITY_BYTES,
                         ZY100_FIFO_WATERMARK_BYTES,
                         ZY100_FIFO_FLASH_SAFE_BYTES,
                         ZY100_FIFO_EMERGENCY_BYTES);
#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
    V0_IMU_LIFECYCLE_LOG("[PHASE_A_CFG] poll_ms=%u drain_th=%u drain_max=%u wm_alarm=%u emergency=%u rescue_floor=%u ingress_bytes=%u flush_bytes=%u rt_source=fifo tim6=0 ois=0",
                         ZY100_FIFO_POLL_INTERVAL_MS,
                         ZY100_FIFO_DRAIN_THRESHOLD_BYTES,
                         ZY100_FIFO_DRAIN_MAX_BYTES,
                         ZY100_FIFO_WATERMARK_ALARM_BYTES,
                         ZY100_FIFO_EMERGENCY_BYTES,
                         ZY100_FIFO_EMERGENCY_RESCUE_FLOOR_BYTES,
                         ZY100_FIFO_INGRESS_RING_BYTES,
                         ZY100_FIFO_INGRESS_FLUSH_BYTES);
#endif
#if !V0_FLASH_RUNTIME_ENABLE
    V0_IMU_LIFECYCLE_LOG("[V0_IMU_FIFO] capture_start round=%u who=0x%02x wm=%u packet=%u odr_hz=%u",
                         stats->round,
                         who_am_i,
                         fifo_cfg.watermark,
                         V0_PACKET_SIZE_BYTES,
                         V0_SAMPLE_RATE_HZ);
#else
    IMU_UNUSED(who_am_i);
#endif
    ZY100_LOG_EVENT("[CAP_READY] mode=1 id=%lu", (unsigned long)stats->round);
    if (continuous_raw) ZY100_LOG_EVENT("[CAP_DATA] mode=1 id=%lu", (unsigned long)stats->round);
    return true;
start_failed:
    prepare_result.status = (uint8_t)stats->fatal_status;
    zy100_capture_start_log(1U, stats->round, &prepare_result);
    return false;
}

static void v0_stop_capture(void)
{
    icm53611_fifo_cfg_t fifo_cfg =
    {
        .enable = false,
        .timestamp_enable = false,
        .accel_enable = false,
        .gyro_enable = false,
        .hires_enable = false,
        .fsync_timestamp_enable = false,
        .watermark = 0U,
    };

#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_FINAL_EDGE_TIMER_ENABLE
    v0_final_edge_timer_stop();
#endif
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE && !ZY100_FINAL_EDGE_MODE_ENABLE
    v0_online_live_timer_stop();
#endif
#if ZY100_RT_MARKER_TIMER_ENABLE
    zy100_rt_marker_timer_stop();
    zy100_rt_marker_timer_register_notify_callback(NULL);
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_FINAL_EDGE_TIMER_ENABLE
    v0_final_edge_timer_stop();
#endif
    zy100_spi_sched_capture_stop();
    /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
    imu_status_t cleanup_status = icm53611_fifo_config(&fifo_cfg);
    if (cleanup_status != IMU_STATUS_OK)
        ZY100_LOG_ERROR("[CAP_CLEAN] mode=1 step=1 st=%u", (uint32_t)cleanup_status);
    imu_bsp_int_register_irq_callback(NULL);
}

static void v0_imu_platform_prepare_for_sleep(void)
{
    bool pending_cleared;
    bool hw_ready = s_v0_hw_ready;
    imu_status_t sleep_status = IMU_STATUS_OK;

    DBG_DIRECT("[V0_LP] sleep_prepare begin active=%u busy=%u hw_ready=%u",
               s_v0_capture_active ? 1U : 0U,
               s_v0_worker_busy ? 1U : 0U,
               s_v0_hw_ready ? 1U : 0U);

#if V1_IMU_FLASH_CAPTURE_ENABLE
    external_flash_capture_led_mark_sleep();
#endif

#if ZY100_RT_MARKER_TIMER_ENABLE
    zy100_rt_marker_timer_stop();
    zy100_rt_marker_timer_register_notify_callback(NULL);
#endif
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE && !ZY100_FINAL_EDGE_MODE_ENABLE
    v0_online_live_timer_stop();
#endif
    zy100_spi_sched_capture_stop();
    imu_bsp_int_register_irq_callback(NULL);
    pending_cleared = imu_bsp_int_disable_for_dlps();
    imu_bsp_int_clear_pending();
    DBG_DIRECT("[V0_LP] imu_irq masked pending_cleared=%u", pending_cleared ? 1U : 0U);

    imu_bsp_cs_high();
    (void)imu_bsp_flash_cs_hold_high();

    if (hw_ready)
    {
        /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
        sleep_status = icm53611_prepare_for_sleep();
        if (sleep_status != IMU_STATUS_OK)
        {
            DBG_DIRECT("[V0_LP][WARN] imu sleep prepare status=%d", sleep_status);
        }
    }

    s_v0_imu_fifo_pending_count = 0U;
    s_v0_high_water_alarm_pending = false;
    s_v0_phase_a_rescue_pending = false;
    s_v0_irq_count = 0U;
    s_v0_start_pending = false;
    s_v0_stop_requested = false;
    s_v0_stop_in_progress = false;
    s_v0_capture_active = false;
    s_v0_worker_busy = false;
    s_v0_spi_fifo_busy = false;
    s_v0_stop_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
    s_v0_last_stop_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
    v0_adc_safe_window_close();
    s_v0_current_round = 0U;
    s_v0_hw_ready = false;
    imu_bsp_mark_lost_after_dlps_prepare();
}

#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE && !ZY100_FINAL_EDGE_MODE_ENABLE
static bool v0_online_direct_final_fifo_drain(v0_fifo_stats_t *stats)
{
    uint32_t start_ms;
    imu_status_t status;

    if ((stats == NULL) || stats->fatal_error)
    {
        return false;
    }

    v0_online_pause_set_phase(ZY100_ONLINE_PAUSE_PHASE_STOP_LIVE);
    status = icm53611_stop_ui_sensors_preserve_fifo();
    if (status != IMU_STATUS_OK)
    {
        v0_mark_read_error(stats, status);
        return false;
    }

    v0_online_pause_set_phase(ZY100_ONLINE_PAUSE_PHASE_DRAIN_FIFO_REPLAY);
    start_ms = zy100_os_time_ms();
    while (!stats->fatal_error)
    {
        uint16_t fifo_count = 0U;
        uint16_t read_len;
        uint16_t actual_read_len = 0U;

        status = icm53611_fifo_get_count(&fifo_count);
        if (status != IMU_STATUS_OK)
        {
            v0_mark_read_error(stats, status);
            return false;
        }
        if (fifo_count == 0U)
        {
            return true;
        }
        if (fifo_count < V0_PACKET_SIZE_BYTES)
        {
            stats->fatal_error = true;
            stats->fatal_status = IMU_STATUS_BUS_ERROR;
            return false;
        }
        if ((uint32_t)(zy100_os_time_ms() - start_ms) >=
            ZY100_ONLINE_CAPTURE_PAUSE_DRAIN_TIMEOUT_MS)
        {
            stats->fatal_error = true;
            stats->fatal_status = IMU_STATUS_TIMEOUT;
            return false;
        }

        read_len = (uint16_t)(fifo_count &
                              (uint16_t)~(V0_PACKET_SIZE_BYTES - 1U));
        if (read_len > V0_FIFO_READ_BUFFER_BYTES)
        {
            read_len = V0_FIFO_READ_BUFFER_BYTES;
        }
        if (!v0_drain_fifo_packets_exact(stats, NULL, read_len, false,
                                         NULL, &actual_read_len, NULL, NULL) ||
            (actual_read_len == 0U) ||
            !zy100_online_raw_capture_pump(zy100_os_time_ms()))
        {
            if (!stats->fatal_error)
            {
                stats->fatal_error = true;
                stats->fatal_status = IMU_STATUS_BUS_ERROR;
            }
            return false;
        }
        v0_online_pause_note_progress();
        v0_online_final_drain_coop_yield();
    }
    return false;
}
#endif

static void v0_run_capture_session(uint32_t round)
{
    v0_fifo_stats_t stats = {0};
#if ZY100_SRAM_SUMMARY_LOG_ENABLE
    zy100_sram_stats_t sram_stats = {0};
#endif
    uint32_t capture_start_ms = 0U;
    uint32_t last_log_ms = 0U;
    bool online_raw_mode = false;
#if ZY100_FINAL_EDGE_MODE_ENABLE
    uint32_t last_fe_log_ms = 0U;
#endif
    uint32_t last_poll_ms = 0U;
#if !ZY100_FINAL_EDGE_MODE_ENABLE
    bool offline_v2_mode = v0_offline_v2_mode_active();
    uint32_t fifo_poll_interval_ms =
        v0_fifo_poll_interval_ms(offline_v2_mode);
    uint32_t fifo_service_soft_ms =
        v0_fifo_service_soft_ms(offline_v2_mode);
    uint32_t fifo_service_hard_ms =
        v0_fifo_service_hard_ms(offline_v2_mode);
    uint32_t fifo_service_emergency_ms =
        v0_fifo_service_emergency_ms(offline_v2_mode);
#if ZY100_LEGACY_OFFLINE_ENABLE
    uint32_t offline_last_coop_yield_ms = 0U;
#endif
#else
    bool offline_v2_mode = false;
#endif
#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
    uint32_t last_fifo_irq_ms = 0U;
    uint32_t last_fifo_count_emergency_probe_ms = 0U;
    uint32_t last_fifo_count_debug_probe_ms = 0U;
#endif
    uint32_t last_fifo_int_status_bad_header_count = 0U;
    uint32_t last_fifo_int_status_full_count = 0U;
    uint32_t fifo_irq_pending_seen_count = 0U;
    uint32_t fifo_irq_pending_consumed_count = 0U;
    uint32_t last_fifo_irq_diag_ms = 0U;
    uint32_t last_fifo_irq_diag_irq_count = 0U;
    uint32_t last_fifo_irq_diag_pending_count = 0U;
    uint32_t last_fifo_irq_diag_latch_count = 0U;
    uint32_t last_loop_ms = 0U;
    uint16_t last_fifo_int_status_lost_count = 0U;
#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
    bool startup_fifo_count_probe_pending = true;
#endif
    bool started = false;
    imu_status_t resume_status;
#if ZY100_RT_MARKER_ENABLE && ZY100_RT_MARKER_TIMER_ENABLE
    bool rt_marker_started = false;
    uint32_t rt_start_wait_last_ms = 0U;
#endif
#if V0_FLASH_RUNTIME_ENABLE
    bool v1_flash_prepared = false;
    bool v1_flash_session_started = false;
    bool old_begin_ok = false;
    bool edge_start_ok = false;
    bool flash_q_log_valid = false;
    uint32_t last_flash_q_summary_ms = 0U;
    uint32_t last_throughput_page_issued = 0U;
    uint32_t last_throughput_block_done = 0U;
    uint32_t last_flash_q_level = 0U;
    bool last_flash_q_near_full = false;
    uint32_t last_flash_q_write_count = 0U;
    uint32_t last_flash_q_write_page_index = 0U;
    external_flash_capture_pump_result_t last_flash_q_pump_result =
        EXTERNAL_FLASH_CAPTURE_PUMP_NO_WORK;
#endif


#if ZY100_FINAL_EDGE_MODE_ENABLE
    IMU_UNUSED(offline_v2_mode);
    IMU_UNUSED(last_poll_ms);
    IMU_UNUSED(last_fifo_int_status_bad_header_count);
    IMU_UNUSED(last_fifo_int_status_full_count);
    IMU_UNUSED(fifo_irq_pending_seen_count);
    IMU_UNUSED(fifo_irq_pending_consumed_count);
    IMU_UNUSED(last_fifo_irq_diag_ms);
    IMU_UNUSED(last_fifo_irq_diag_irq_count);
    IMU_UNUSED(last_fifo_irq_diag_pending_count);
    IMU_UNUSED(last_fifo_irq_diag_latch_count);
    IMU_UNUSED(last_fifo_int_status_lost_count);
#endif


    stats.round = round;
    stats.fatal_status = IMU_STATUS_OK;
    stats.stop_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
    stats.ingress_free_min = ZY100_FIFO_INGRESS_RING_BYTES;
    v0_online_bad_fifo_reset();
#if ZY100_SRAM_SUMMARY_LOG_ENABLE
    v0_sram_reset(&sram_stats);
#endif
#if ZY100_RT_MARKER_ENABLE && ZY100_RT_MARKER_TIMER_ENABLE
    v0_rt_marker_init_start_gate(&stats);
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE
    v0_final_edge_reset();
#endif
    v0_adc_safe_window_close();
    v0_reset_fifo_drain_diag_ring();
#if ZY100_LEGACY_OFFLINE_ENABLE && !ZY100_FINAL_EDGE_MODE_ENABLE
    v0_offline_fifo_diag_reset(offline_v2_mode);
#endif
    zy100_fifo_ingress_reset();
    s_v0_imu_fifo_pending_count = 0U;
    s_v0_high_water_alarm_pending = false;
    s_v0_phase_a_rescue_pending = false;
    s_v0_irq_count = 0U;
    s_v0_capture_done = false;
    s_v0_worker_busy = true;
    s_v0_spi_fifo_busy = true;
    s_v0_hw_ready = false;
#if V0_FLASH_RUNTIME_ENABLE
    IMU_UNUSED(last_log_ms);
#else
    V0_IMU_LIFECYCLE_LOG("[V0_IMU_FIFO] reset counters round=%u", stats.round);
    V0_IMU_LIFECYCLE_LOG("[V0_IMU_FIFO] run round=%u mode=%s duration_ms=%u wm=%u poll_ms=%u count_emg_probe_ms=%u count_dbg_probe_ms=%u",
                         stats.round,
                         F_APP_FW_LOG_VERSION,
                         V0_CAPTURE_DURATION_MS,
                         V0_FIFO_WATERMARK_BYTES,
                         V0_POLL_PERIOD_MS,
                         V0_FIFO_COUNT_EMERGENCY_PROBE_MS,
                         V0_FIFO_COUNT_DEBUG_PROBE_MS);
#endif

#if V0_FLASH_RUNTIME_ENABLE
    IMU_UNUSED(resume_status);
#if ZY100_LEGACY_OFFLINE_ENABLE && !ZY100_FINAL_EDGE_MODE_ENABLE
    if (offline_v2_mode)
    {
        /* Offline V2 owns event persistence in its App state machine. */
        s_v1_flash_ready_wait_start = false;
        started = v0_start_capture(&stats);
    }
    else
#endif
    if (ZY100_RT_MARKER_TIMER_ONLY_TEST || ZY100_RT_MARKER_FIFO_ONLY_TEST)
    {
        started = v0_start_capture(&stats);
    }
    else if (s_v1_flash_ready_wait_start)
    {
        v1_flash_prepared = true;
        old_begin_ok = external_flash_capture_begin();
        if (old_begin_ok)
        {
            v1_flash_session_started = true;
            s_v1_flash_ready_wait_start = false;
            started = v0_start_capture(&stats);
        }
        else
        {
            v0_mark_read_error(&stats, IMU_STATUS_BUS_ERROR);
        }
        DBG_DIRECT("[CAP_START_GATE] edge=%u old_begin_ok=%u edge_start_ok=%u hw_ready=%u worker_busy=%u capture_started=%u",
                   0U,
                   old_begin_ok ? 1U : 0U,
                   edge_start_ok ? 1U : 0U,
                   s_v0_hw_ready ? 1U : 0U,
                   s_v0_worker_busy ? 1U : 0U,
                   started ? 1U : 0U);
    }
    else
    {
        v0_mark_read_error(&stats, IMU_STATUS_NOT_READY);
    }
#else
    resume_status = v0_imu_platform_resume_for_capture(stats.round);
    if (resume_status == IMU_STATUS_OK)
    {
#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
        if (zy100_online_stream_active())
        {
            online_raw_mode = zy100_online_raw_capture_begin(0U);
            if (online_raw_mode)
            {
                started = v0_start_capture(&stats);
            }
            else
            {
                v0_mark_read_error(&stats, IMU_STATUS_NOT_READY);
            }
        }
        else
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
        if (zy100_final_edge_raw_store_start(stats.round) &&
            zy100_final_edge_record_store_start(stats.round))
        {
            started = v0_start_capture(&stats);
        }
        else
        {
            v0_mark_read_error(&stats, IMU_STATUS_NOT_READY);
        }
#else
        started = v0_start_capture(&stats);
#endif
    }
    else
    {
        v0_mark_read_error(&stats, resume_status);
    }
#endif
    s_v0_capture_active = started;
#if ZY100_ONLINE_STREAM_ENABLE && \
    ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
    if (!started && zy100_online_stream_active())
    {
        v0_online_abort_diag_capture_end(&stats);
        zy100_online_stream_request_abort_ex(
            ZY100_ONLINE_STOP_REASON_INTERNAL,
            ZY100_ONLINE_ABORT_ORIGIN_IMU_START,
            (uint32_t)stats.fatal_status,
            zy100_os_time_ms());
    }
#endif
    capture_start_ms = zy100_os_time_ms();
#if ZY100_LEGACY_OFFLINE_ENABLE && !ZY100_FINAL_EDGE_MODE_ENABLE
    offline_last_coop_yield_ms = capture_start_ms;
    if (offline_v2_mode)
    {
        zy100_offline_reset_trace_set_imu_state(
            ZY100_OFFLINE_TRACE_IMU_LIVE,
            false,
            false,
            false,
            ZY100_OFFLINE_TRACE_SERVICE_IDLE,
            0U);
    }
#endif
    if (started)
    {
#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
        if (!online_raw_mode)
#endif
        {
            zy100_edge_shadow_capture_start();
        }
#if ZY100_FINAL_EDGE_MODE_ENABLE
        v0_final_edge_set_state(FE_CAPTURE_STATE_RUNNING);
        if (ZY100_LOG_FE_DIAG_VERBOSE)
        {
            v0_final_edge_log_config();
        }
#endif
    }
#if ZY100_SRAM_SUMMARY_LOG_ENABLE
    if (started)
    {
        v0_sram_capture_start(&sram_stats);
    }
#endif
#if ZY100_RT_MARKER_ENABLE && ZY100_RT_MARKER_TIMER_ENABLE
    rt_marker_started = zy100_rt_marker_timer_is_running();
#if ZY100_FINAL_EDGE_MODE_ENABLE
    IMU_UNUSED(rt_marker_started);
    IMU_UNUSED(rt_start_wait_last_ms);
#endif
#endif

    while (started && !stats.fatal_error && !s_v0_stop_requested)
    {
#if ZY100_FINAL_EDGE_MODE_ENABLE
        uint32_t now_ms = zy100_os_time_ms();
        uint32_t runtime_ms = v0_elapsed_ms(capture_start_ms);
#if ZY100_ONLINE_BOUNDED_PUMP_ENABLE
        zy100_online_service_result_t online_service = ZY100_ONLINE_SERVICE_WAIT;
#endif

#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
        if (online_raw_mode)
        {
            (void)zy100_online_raw_capture_mag_checkpoint(
                ZY100_ONLINE_RAW_MAG_CHECKPOINT_LOOP_ENTRY);
        }
#endif
        v0_adc_safe_window_close();
        v0_record_loop_gap(&stats, now_ms, &last_loop_ms);
#if ZY100_SRAM_SUMMARY_LOG_ENABLE
        v0_sram_capture_sample(&sram_stats);
#endif
        stats.irq_count = s_v0_irq_count;
        IMU_UNUSED(v0_final_edge_live_service_handler(&stats, false));
#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
        if (online_raw_mode)
        {
            uint32_t pump_now_ms;

            (void)zy100_online_raw_capture_mag_checkpoint(
                ZY100_ONLINE_RAW_MAG_CHECKPOINT_AFTER_LIVE_SERVICE);
            v0_online_capture_coop_yield(now_ms);
            (void)zy100_online_raw_capture_mag_checkpoint(
                ZY100_ONLINE_RAW_MAG_CHECKPOINT_AFTER_COOP_YIELD);
            pump_now_ms = zy100_os_time_ms();
#if ZY100_ONLINE_BOUNDED_PUMP_ENABLE
            online_service = zy100_online_raw_capture_service(pump_now_ms);
            if (online_service == ZY100_ONLINE_SERVICE_ERROR)
#else
            if (!zy100_online_raw_capture_pump(pump_now_ms))
#endif
            {
                stats.fatal_error = true;
                stats.fatal_status = IMU_STATUS_BUS_ERROR;
                v0_online_abort_diag_capture_end(&stats);
                zy100_online_stream_request_abort_ex(
                    ZY100_ONLINE_STOP_REASON_INTERNAL,
                    ZY100_ONLINE_ABORT_ORIGIN_IMU_RAW_PUMP,
                    0U,
                    pump_now_ms);
                (void)imu_fifo_drain_test_request_online_abort_with_reason(
                    IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL);
                stats.fatal_error = true;
                stats.fatal_status = IMU_STATUS_BUS_ERROR;
            }
        }
        else
#endif
        {
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
            v0_final_edge_raw_store_pump_safe();
#endif
            v0_final_edge_record_store_pump_safe();
            v0_online_capture_coop_yield(now_ms);
        }
        if (v0_handle_fifo_fatal_if_seen(&stats))
        {
            continue;
        }

        if ((uint32_t)(runtime_ms - last_log_ms) >= V0_LOG_PERIOD_MS)
        {
            last_log_ms = runtime_ms;
            v0_log_stats(&stats, runtime_ms, false);
            IMU_UNUSED(v0_final_edge_check_flash_watermark_stop());
        }
        if (ZY100_LOG_FE_DIAG_VERBOSE &&
            ((uint32_t)(runtime_ms - last_fe_log_ms) >= V0_FE_LOG_PERIOD_MS))
        {
            last_fe_log_ms = runtime_ms;
            v0_final_edge_log_stats(&stats, runtime_ms, false);
        }
        if (!stats.fatal_error && !s_v0_stop_requested)
        {
            uint32_t idle_notify = 0U;

#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
            if (online_raw_mode)
            {
                (void)zy100_online_raw_capture_mag_checkpoint(
                    ZY100_ONLINE_RAW_MAG_CHECKPOINT_LOOP_ENTRY);
            }
#endif
            v0_adc_safe_window_open(now_ms);
            (void)xTaskNotifyWait(0U,
                                  V0_NOTIFY_MASK,
                                  &idle_notify,
#if ZY100_ONLINE_BOUNDED_PUMP_ENABLE
                                  (online_raw_mode && online_service == ZY100_ONLINE_SERVICE_READY) ? 0U :
#endif
                                  1U);
            v0_adc_safe_window_close();
#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
            if (online_raw_mode)
            {
                (void)zy100_online_raw_capture_mag_checkpoint(
                    ZY100_ONLINE_RAW_MAG_CHECKPOINT_AFTER_NOTIFY_WAIT);
            }
#endif
            if (((idle_notify & V0_NOTIFY_FE_TIMER) != 0U) &&
                !stats.fatal_error &&
                !s_v0_stop_requested)
            {
                IMU_UNUSED(v0_final_edge_live_service_handler(&stats, false));
#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
                if (online_raw_mode)
                {
                    uint32_t pump_now_ms;

                    (void)zy100_online_raw_capture_mag_checkpoint(
                        ZY100_ONLINE_RAW_MAG_CHECKPOINT_AFTER_LIVE_SERVICE);
                    v0_online_capture_coop_yield(zy100_os_time_ms());
                    (void)zy100_online_raw_capture_mag_checkpoint(
                        ZY100_ONLINE_RAW_MAG_CHECKPOINT_AFTER_COOP_YIELD);
                    pump_now_ms = zy100_os_time_ms();
                    if (!zy100_online_raw_capture_pump(pump_now_ms))
                    {
                        stats.fatal_error = true;
                        stats.fatal_status = IMU_STATUS_BUS_ERROR;
                        v0_online_abort_diag_capture_end(&stats);
                        zy100_online_stream_request_abort_ex(
                            ZY100_ONLINE_STOP_REASON_INTERNAL,
                            ZY100_ONLINE_ABORT_ORIGIN_IMU_RAW_PUMP,
                            0U,
                            pump_now_ms);
                        (void)imu_fifo_drain_test_request_online_abort_with_reason(
                            IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL);
                    }
                }
                else
#endif
                {
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
                    v0_final_edge_raw_store_pump_safe();
#endif
                    v0_final_edge_record_store_pump_safe();
                    v0_online_capture_coop_yield(zy100_os_time_ms());
                }
                if (v0_handle_fifo_fatal_if_seen(&stats))
                {
                    continue;
                }
            }
        }
#else
        uint32_t now_ms = zy100_os_time_ms();
        uint32_t runtime_ms = v0_elapsed_ms(capture_start_ms);
#if ZY100_ONLINE_BOUNDED_PUMP_ENABLE
        zy100_online_service_result_t online_service = ZY100_ONLINE_SERVICE_WAIT;
#endif
        uint32_t pending_count_snapshot;
        uint32_t pending_count_delta;
        uint32_t ms_since_last_fifo_drain = 0U;
        bool fifo_irq_pending = false;
#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
        bool fifo_irq_consumed = false;
#endif
        bool fifo_service_due = false;
        bool fifo_service_overdue = false;
        bool fifo_service_emergency = false;
        bool phase_a_fifo_poll_due = false;
        bool phase_a_high_alarm_pending = false;
        bool phase_a_rescue_pending = false;
        bool phase_a_fifo_service_pending = false;
        bool phase_a_ingress_flush_pending = false;
        bool phase_a_fifo_safe_for_flash = true;
        bool scheduler_urgent;
#if ZY100_LEGACY_OFFLINE_ENABLE
        bool offline_safe_to_yield = true;
#endif
#if V0_FLASH_RUNTIME_ENABLE
        zy100_spi_sched_stats_t sched_stats;
#endif
        IMU_UNUSED(phase_a_ingress_flush_pending);
#if ZY100_FINAL_EDGE_MODE_ENABLE
        IMU_UNUSED(phase_a_fifo_service_pending);
        IMU_UNUSED(phase_a_ingress_flush_pending);
#endif
#if !ZY100_LEGACY_OFFLINE_ENABLE
        IMU_UNUSED(phase_a_fifo_service_pending);
#endif
#if V0_FLASH_RUNTIME_ENABLE
        bool flash_has_pending_work = false;
        external_flash_capture_queue_status_t flash_q_status;
        external_flash_capture_queue_status_t flash_q_status_after;
#endif

        v0_adc_safe_window_close();
        v0_record_loop_gap(&stats, now_ms, &last_loop_ms);
#if ZY100_LEGACY_OFFLINE_ENABLE
        if (offline_v2_mode &&
            (stats.max_loop_gap_ms > s_v0_offline_fifo_diag.max_loop_gap_ms))
        {
            s_v0_offline_fifo_diag.max_loop_gap_ms = stats.max_loop_gap_ms;
        }
#endif
#if ZY100_SRAM_SUMMARY_LOG_ENABLE
        v0_sram_capture_sample(&sram_stats);
#endif
#if V0_FLASH_RUNTIME_ENABLE
        s_v0_phase_a_ingress_flush_used_this_loop = false;
#endif
        if ((stats.fifo_drain_since_start != 0U) &&
            !ZY100_RT_MARKER_TIMER_ONLY_TEST)
        {
            ms_since_last_fifo_drain = now_ms - stats.last_fifo_drain_ms;
            fifo_service_due =
                (ms_since_last_fifo_drain >= fifo_service_soft_ms);
            fifo_service_overdue =
                (ms_since_last_fifo_drain >= fifo_service_hard_ms);
            fifo_service_emergency =
                (ms_since_last_fifo_drain >= fifo_service_emergency_ms);
        }
        pending_count_snapshot = s_v0_imu_fifo_pending_count;
        pending_count_delta = v0_delta_u32(pending_count_snapshot,
                                           fifo_irq_pending_seen_count);
        stats.irq_count = s_v0_irq_count;


#if !V0_FLASH_RUNTIME_ENABLE
        if (!online_raw_mode && (runtime_ms >= V0_CAPTURE_DURATION_MS))
        {
            stats.reached_final_summary = true;
            break;
        }
#endif

        if (pending_count_delta != 0U)
        {
            fifo_irq_pending_seen_count = pending_count_snapshot;
            fifo_irq_pending_consumed_count += pending_count_delta;
            fifo_irq_pending = true;
#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
            fifo_irq_consumed = true;
            last_fifo_irq_ms = runtime_ms;
#endif
#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
            s_v0_high_water_alarm_pending = true;
            stats.phase_a_alarm_irq_count += pending_count_delta;
#else
            zy100_spi_sched_request_fifo_drain_from_irq(now_ms,
                                                        pending_count_delta);
#if V0_FIFO_INT_STATUS_RUNTIME_DEBUG
            zy100_spi_sched_request_fifo_int_status();
#endif
#endif
        }

#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
        if ((uint32_t)(runtime_ms - last_poll_ms) >= fifo_poll_interval_ms)
        {
            last_poll_ms = runtime_ms;
            phase_a_fifo_poll_due = true;
        }
#else
        if ((uint32_t)(runtime_ms - last_poll_ms) >= V0_POLL_PERIOD_MS)
        {
            last_poll_ms = runtime_ms;
            stats.poll_count++;
            if (!ZY100_RT_MARKER_TIMER_ONLY_TEST && !fifo_irq_consumed)
            {
                if (startup_fifo_count_probe_pending)
                {
                    startup_fifo_count_probe_pending = false;
                    last_fifo_count_debug_probe_ms = runtime_ms;
                    zy100_spi_sched_request_fifo_count_debug_probe();
                }
                else if (((uint32_t)(runtime_ms - last_fifo_irq_ms) >=
                          V0_FIFO_COUNT_EMERGENCY_PROBE_MS) &&
                         ((uint32_t)(runtime_ms - last_fifo_count_emergency_probe_ms) >=
                          V0_FIFO_COUNT_EMERGENCY_PROBE_MS))
                {
                    last_fifo_count_emergency_probe_ms = runtime_ms;
                    zy100_spi_sched_request_fifo_count_emergency_probe();
                }
                else if ((uint32_t)(runtime_ms - last_fifo_count_debug_probe_ms) >=
                         V0_FIFO_COUNT_DEBUG_PROBE_MS)
                {
                    last_fifo_count_debug_probe_ms = runtime_ms;
                    zy100_spi_sched_request_fifo_count_debug_probe();
                }
            }
        }
#endif

#if ZY100_RT_MARKER_ENABLE && ZY100_RT_MARKER_TIMER_ENABLE
        if (!offline_v2_mode && rt_marker_started && imu_rt_marker_has_due())
        {
            zy100_spi_sched_request_rt_marker();
        }
#elif ZY100_RT_MARKER_ENABLE
        if (!offline_v2_mode && imu_rt_marker_has_due())
        {
            zy100_spi_sched_request_rt_marker();
        }
#endif

#if V0_FLASH_RUNTIME_ENABLE
        memset(&flash_q_status, 0, sizeof(flash_q_status));
        memset(&flash_q_status_after, 0, sizeof(flash_q_status_after));
        if (!offline_v2_mode &&
            !ZY100_RT_MARKER_TIMER_ONLY_TEST &&
            !ZY100_RT_MARKER_FIFO_ONLY_TEST)
        {
            external_flash_capture_get_queue_status(&flash_q_status);
            v0_update_queue_max_level(&stats, &flash_q_status);
            flash_has_pending_work = external_flash_capture_has_pending_work() ||
                                     flash_q_status.raw_queue_busy;
        }
#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
        v0_phase_a_update_backpressure(&stats);
        if (stats.ingress_backpressure_pending)
        {
            phase_a_fifo_poll_due = true;
        }
        phase_a_high_alarm_pending = s_v0_high_water_alarm_pending;
        phase_a_rescue_pending = s_v0_phase_a_rescue_pending;
        phase_a_fifo_service_pending =
            phase_a_high_alarm_pending ||
            phase_a_rescue_pending ||
            fifo_service_due ||
            fifo_service_overdue ||
            fifo_service_emergency;
        phase_a_ingress_flush_pending =
            !stats.ingress_backpressure_pending &&
            (zy100_fifo_ingress_level_bytes() >= ZY100_FIFO_INGRESS_FLUSH_BYTES);
        phase_a_fifo_safe_for_flash =
            !phase_a_fifo_service_pending &&
            !phase_a_fifo_poll_due &&
            !stats.phase_a_ingress_drain_blocked;
#endif

        if (!offline_v2_mode &&
            !ZY100_RT_MARKER_TIMER_ONLY_TEST &&
            !ZY100_RT_MARKER_FIFO_ONLY_TEST &&
            external_flash_capture_has_error())
        {
            stats.fatal_error = true;
            stats.fatal_status = IMU_STATUS_BUS_ERROR;
            v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL);
        }
        else if (!offline_v2_mode &&
                 !ZY100_RT_MARKER_TIMER_ONLY_TEST &&
                 !ZY100_RT_MARKER_FIFO_ONLY_TEST &&
                 external_flash_capture_is_full())
        {
            stats.reached_final_summary = true;
            v0_request_stop_from_worker(IMU_FIFO_DRAIN_TEST_STOP_REASON_FLASH_95_PERCENT);
        }
        else if (!offline_v2_mode &&
                 !ZY100_RT_MARKER_TIMER_ONLY_TEST &&
                 !ZY100_RT_MARKER_FIFO_ONLY_TEST)
        {
            if (flash_has_pending_work)
            {
                zy100_spi_sched_request_flash_pump();
            }
        }
#else
        if ((uint32_t)(runtime_ms - last_log_ms) >= V0_LOG_PERIOD_MS)
        {
            last_log_ms = runtime_ms;
            v0_log_stats(&stats, runtime_ms, false);
        }
#if ZY100_FINAL_EDGE_MODE_ENABLE
        if (ZY100_LOG_FE_DIAG_VERBOSE &&
            ((uint32_t)(runtime_ms - last_fe_log_ms) >= V0_FE_LOG_PERIOD_MS))
        {
            last_fe_log_ms = runtime_ms;
            v0_final_edge_log_stats(&stats, runtime_ms, false);
        }
#endif
#endif

#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE && V0_FLASH_RUNTIME_ENABLE
        if (!offline_v2_mode &&
            !phase_a_fifo_poll_due &&
            !phase_a_fifo_service_pending &&
            phase_a_ingress_flush_pending)
        {
            bool flushed = v0_phase_a_try_ingress_flush(&stats, false);
            phase_a_ingress_flush_pending =
                (zy100_fifo_ingress_level_bytes() >= ZY100_FIFO_INGRESS_FLUSH_BYTES);
            if (!flushed && phase_a_fifo_safe_for_flash)
            {
                zy100_spi_sched_request_flash_pump();
            }
            flash_has_pending_work = external_flash_capture_has_pending_work() ||
                                     external_flash_capture_raw_queue_busy();
        }
#endif

        if (online_raw_mode)
        {
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
            IMU_UNUSED(v0_online_live_fifo_service(&stats, false));
#endif
        }
        else if (offline_v2_mode)
        {
#if ZY100_LEGACY_OFFLINE_ENABLE
            if (phase_a_fifo_poll_due || fifo_irq_pending ||
                phase_a_fifo_service_pending)
            {
                IMU_UNUSED(v0_offline_fifo_service(&stats,
                                                   fifo_irq_pending,
                                                   &offline_safe_to_yield));
            }
#endif
        }
        else
        {
            zy100_spi_sched_input_t sched_input;
            memset(&sched_input, 0, sizeof(sched_input));
            sched_input.now_ms = now_ms;
            sched_input.capture_active = true;
            sched_input.fifo_irq_pending = fifo_irq_pending;
            sched_input.fifo_poll_due = phase_a_fifo_poll_due;
            sched_input.fifo_high_water_alarm_pending = phase_a_high_alarm_pending;
            sched_input.fifo_rescue_pending = phase_a_rescue_pending;
#if ZY100_RT_MARKER_ENABLE && ZY100_RT_MARKER_TIMER_ENABLE
            sched_input.rt_due_pending = rt_marker_started ?
                                         imu_rt_marker_has_due() : false;
#elif ZY100_RT_MARKER_ENABLE
            sched_input.rt_due_pending = imu_rt_marker_has_due();
#else
            sched_input.rt_due_pending = false;
#endif
            sched_input.fifo_service_due = fifo_service_due;
            sched_input.fifo_service_overdue = fifo_service_overdue;
            sched_input.fifo_service_emergency = fifo_service_emergency;
            sched_input.flash_pump_requested = false;
#if V0_FLASH_RUNTIME_ENABLE
            sched_input.flash_has_pending_work = flash_has_pending_work;
            sched_input.flash_raw_queue_busy = flash_q_status.raw_queue_busy;
            sched_input.flash_queue_level = flash_q_status.queue_level;
            sched_input.flash_backlog_blocks =
                v0_backlog_blocks_u32(stats.packet_count, flash_q_status.write_count);
            sched_input.flash_queue_near_full = flash_q_status.queue_near_full;
#endif
            sched_input.ms_since_last_fifo_drain = ms_since_last_fifo_drain;
            sched_input.timer_only_test = (ZY100_RT_MARKER_TIMER_ONLY_TEST != 0);
            sched_input.fifo_only_test = (ZY100_RT_MARKER_FIFO_ONLY_TEST != 0);
            sched_input.ctx = &stats;
            sched_input.read_fifo_count = v0_sched_read_fifo_count;
            sched_input.read_fifo_int_status = v0_sched_read_fifo_int_status;
            sched_input.drain_fifo_512 = v0_sched_drain_fifo_512;
#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
            sched_input.fifo_poll_drain = v0_sched_fifo_poll_drain;
#endif
#if ZY100_RT_MARKER_ENABLE && ZY100_RT_MARKER_TIMER_ENABLE
            if (rt_marker_started)
            {
                sched_input.rt_marker_read = v0_sched_rt_marker_read;
                sched_input.rt_marker_skip_fifo = v0_sched_rt_marker_skip_fifo;
                sched_input.rt_marker_skip_spi = v0_sched_rt_marker_skip_spi;
            }
            else
            {
                sched_input.rt_marker_read = NULL;
                sched_input.rt_marker_skip_fifo = NULL;
                sched_input.rt_marker_skip_spi = NULL;
            }
#elif ZY100_RT_MARKER_ENABLE
            sched_input.rt_marker_read = v0_sched_rt_marker_read;
            sched_input.rt_marker_skip_fifo = v0_sched_rt_marker_skip_fifo;
            sched_input.rt_marker_skip_spi = v0_sched_rt_marker_skip_spi;
#else
            sched_input.rt_marker_read = NULL;
            sched_input.rt_marker_skip_fifo = NULL;
            sched_input.rt_marker_skip_spi = NULL;
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE
            sched_input.flash_pump = NULL;
#else
            sched_input.flash_pump = v0_sched_flash_pump;
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE
            sched_input.flash_skip_fifo = NULL;
#else
            sched_input.flash_skip_fifo = v0_sched_flash_skip_fifo;
#endif
            IMU_UNUSED(zy100_spi_sched_execute_once(&sched_input));
        }
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
        if (online_raw_mode)
        {
            uint32_t pump_now_ms;

            (void)zy100_online_raw_capture_mag_checkpoint(
                ZY100_ONLINE_RAW_MAG_CHECKPOINT_AFTER_LIVE_SERVICE);
            v0_online_capture_coop_yield(now_ms);
            (void)zy100_online_raw_capture_mag_checkpoint(
                ZY100_ONLINE_RAW_MAG_CHECKPOINT_AFTER_COOP_YIELD);
            pump_now_ms = zy100_os_time_ms();
#if ZY100_ONLINE_BOUNDED_PUMP_ENABLE
            online_service = zy100_online_raw_capture_service(pump_now_ms);
            if (online_service == ZY100_ONLINE_SERVICE_ERROR)
#else
            if (!zy100_online_raw_capture_pump(pump_now_ms))
#endif
            {
                stats.fatal_error = true;
                stats.fatal_status = IMU_STATUS_BUS_ERROR;
                v0_online_abort_diag_capture_end(&stats);
                zy100_online_stream_request_abort_ex(
                    ZY100_ONLINE_STOP_REASON_INTERNAL,
                    ZY100_ONLINE_ABORT_ORIGIN_IMU_RAW_PUMP,
                    0U,
                    pump_now_ms);
                (void)imu_fifo_drain_test_request_online_abort_with_reason(
                    IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL);
            }
        }
#endif
        if (v0_handle_fifo_fatal_if_seen(&stats))
        {
            continue;
        }
#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
        if (!online_raw_mode)
        {
            v0_phase_a_handle_ingress_blocked(&stats,
                                              phase_a_fifo_safe_for_flash);
        }
        if (v0_handle_fifo_fatal_if_seen(&stats))
        {
            continue;
        }
#endif
#if ZY100_RT_MARKER_ENABLE && ZY100_RT_MARKER_TIMER_ENABLE
        if (
#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
            !online_raw_mode &&
#endif
            !offline_v2_mode &&
            !rt_marker_started && v0_rt_start_delay_enabled() &&
            stats.rt_start_good_drain && !stats.fatal_error &&
            !s_v0_stop_requested)
        {
            rt_marker_started = v0_rt_marker_start_after_good_drain(&stats);
        }
#endif
#if V0_FLASH_RUNTIME_ENABLE
        if (!offline_v2_mode &&
            !ZY100_RT_MARKER_TIMER_ONLY_TEST &&
            !ZY100_RT_MARKER_FIFO_ONLY_TEST)
        {
            external_flash_capture_pump_result_t pump_result;

            external_flash_capture_get_queue_status(&flash_q_status_after);
            v0_update_queue_max_level(&stats, &flash_q_status_after);
            if (zy100_spi_sched_last_op_was_flash_pump())
            {
                pump_result = flash_q_status_after.last_pump_result;
                zy100_spi_sched_note_flash_pump_step_result(
                    pump_result == EXTERNAL_FLASH_CAPTURE_PUMP_PAGE_ISSUED,
                    pump_result == EXTERNAL_FLASH_CAPTURE_PUMP_BLOCK_DONE,
                    pump_result == EXTERNAL_FLASH_CAPTURE_PUMP_WIP_BUSY,
                    pump_result == EXTERNAL_FLASH_CAPTURE_PUMP_NO_WORK);
            }
        }
#endif
#if V0_FLASH_RUNTIME_ENABLE
        /* Retain the snapshot for the legacy Flash-queue diagnostics. */
        zy100_spi_sched_get_stats(&sched_stats);
        if (!offline_v2_mode &&
            !ZY100_RT_MARKER_TIMER_ONLY_TEST &&
            !ZY100_RT_MARKER_FIFO_ONLY_TEST)
        {
            v0_log_flash_queue_if_due(runtime_ms,
                                      stats.packet_count,
                                      &flash_q_status_after,
                                      &sched_stats,
                                      &flash_q_log_valid,
                                      &last_flash_q_summary_ms,
                                      &last_throughput_page_issued,
                                      &last_throughput_block_done,
                                      &last_flash_q_level,
                                      &last_flash_q_near_full,
                                      &last_flash_q_write_count,
                                      &last_flash_q_write_page_index,
                                      &last_flash_q_pump_result);
        }
#endif
        if (!offline_v2_mode)
        {
            v0_log_fifo_irq_diag_if_due(runtime_ms,
                                        stats.irq_count,
                                        fifo_irq_pending_consumed_count,
                                        &last_fifo_irq_diag_ms,
                                        &last_fifo_irq_diag_irq_count,
                                        &last_fifo_irq_diag_pending_count,
                                        &last_fifo_irq_diag_latch_count);
        }
        if ((stats.bad_header_count != last_fifo_int_status_bad_header_count) ||
            (stats.fifo_full_count != last_fifo_int_status_full_count) ||
            (stats.fifo_lost_pkt_count != last_fifo_int_status_lost_count))
        {
            zy100_spi_sched_request_fifo_int_status();
            last_fifo_int_status_bad_header_count = stats.bad_header_count;
            last_fifo_int_status_full_count = stats.fifo_full_count;
            last_fifo_int_status_lost_count = stats.fifo_lost_pkt_count;
            if (v0_handle_fifo_fatal_if_seen(&stats))
            {
                continue;
            }
        }
        scheduler_urgent = zy100_spi_sched_has_urgent_work();
#if ZY100_LEGACY_OFFLINE_ENABLE
        if (offline_v2_mode)
        {
            if ((uint32_t)(now_ms - offline_last_coop_yield_ms) >=
                ZY100_OFFLINE_V2_COOP_YIELD_PERIOD_MS &&
                offline_safe_to_yield)
            {
                offline_last_coop_yield_ms = now_ms;
#if ZY100_LEGACY_OFFLINE_ENABLE
                s_v0_offline_fifo_diag.yield_count++;
#endif
                os_delay(1U);
            }
        }
#endif
        if (!offline_v2_mode)
        {
            zy100_spi_sched_log_1s_if_due(now_ms);
            zy100_spi_sched_log_5s_if_due(now_ms);
        }
#if ZY100_RT_MARKER_ENABLE && ZY100_RT_MARKER_TIMER_ENABLE
        v0_rt_marker_log_start_wait_if_due(&stats,
                                           runtime_ms,
                                           &rt_start_wait_last_ms);
#endif

#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
#if V0_FLASH_RUNTIME_ENABLE
        if (!scheduler_urgent &&
            !phase_a_fifo_poll_due &&
            !s_v0_high_water_alarm_pending &&
            !s_v0_phase_a_rescue_pending &&
            !fifo_service_due &&
            !fifo_service_overdue &&
            !fifo_service_emergency &&
            (zy100_fifo_ingress_level_bytes() < ZY100_FIFO_INGRESS_FLUSH_BYTES) &&
            !flash_has_pending_work &&
            !external_flash_capture_has_pending_work() &&
            !external_flash_capture_raw_queue_busy())
        {
            uint32_t idle_notify = 0U;
            v0_adc_safe_window_open(now_ms);
            (void)xTaskNotifyWait(0U,
                                  V0_NOTIFY_MASK,
                                  &idle_notify,
#if ZY100_ONLINE_BOUNDED_PUMP_ENABLE
                                  (online_raw_mode && online_service == ZY100_ONLINE_SERVICE_READY) ? 0U :
#endif
                                  1U);
            v0_adc_safe_window_close();
        }
#else
        if (!scheduler_urgent &&
            !phase_a_fifo_poll_due &&
            !s_v0_high_water_alarm_pending &&
            !s_v0_phase_a_rescue_pending &&
            !fifo_service_due &&
            !fifo_service_overdue &&
            !fifo_service_emergency)
        {
            uint32_t idle_notify = 0U;
            v0_adc_safe_window_open(now_ms);
            (void)xTaskNotifyWait(0U,
                                  V0_NOTIFY_MASK,
                                  &idle_notify,
#if ZY100_ONLINE_BOUNDED_PUMP_ENABLE
                                  (online_raw_mode && online_service == ZY100_ONLINE_SERVICE_READY) ? 0U :
#endif
                                  1U);
            v0_adc_safe_window_close();
        }
#endif
#else
#if ZY100_RT_MARKER_ENABLE && ZY100_RT_MARKER_TIMER_ENABLE
        if (!scheduler_urgent &&
            (!rt_marker_started || !imu_rt_marker_has_due()))
#elif ZY100_RT_MARKER_ENABLE
        if (!scheduler_urgent && !imu_rt_marker_has_due())
#else
        if (!scheduler_urgent)
#endif
        {
            uint32_t idle_notify = 0U;
            v0_adc_safe_window_open(now_ms);
            (void)xTaskNotifyWait(0U,
                                  V0_NOTIFY_MASK,
                                  &idle_notify,
#if ZY100_ONLINE_BOUNDED_PUMP_ENABLE
                                  (online_raw_mode && online_service == ZY100_ONLINE_SERVICE_READY) ? 0U :
#endif
                                  1U);
            v0_adc_safe_window_close();
        }
#endif
#endif
    }

    v0_adc_safe_window_close();
    s_v0_stop_in_progress = true;
#if !ZY100_FINAL_EDGE_MODE_ENABLE
    if (offline_v2_mode && !stats.fatal_error)
    {
        zy100_offline_reset_trace_set_imu_state(
            ZY100_OFFLINE_TRACE_IMU_STOP,
            false,
            false,
            false,
            ZY100_OFFLINE_TRACE_SERVICE_IDLE,
            0U);
    }
#endif
    if (zy100_online_stream_active() || zy100_online_stream_end_wait_ack())
    {
        v0_online_pause_set_phase(ZY100_ONLINE_PAUSE_PHASE_STOP_LIVE);
    }
    stats.irq_count = s_v0_irq_count;
    if (started)
    {
        bool final_status_ok = false;
        bool capture_stopped = false;

#if ZY100_RT_MARKER_TIMER_ENABLE
        zy100_rt_marker_timer_stop();
        zy100_rt_marker_timer_register_notify_callback(NULL);
#endif
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE && !ZY100_FINAL_EDGE_MODE_ENABLE
        if (online_raw_mode &&
            !s_v0_online_abort_requested &&
            zy100_online_stream_end_requested() &&
            (zy100_online_stream_capture_stop_reason() ==
             ZY100_ONLINE_STOP_REASON_HOST_STOP) &&
            (s_v0_stop_reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_PAUSED))
        {
            v0_online_live_timer_stop();
            if (!v0_online_direct_final_fifo_drain(&stats))
            {
                v0_online_abort_diag_capture_end(&stats);
                zy100_online_stream_request_abort_ex(
                    ZY100_ONLINE_STOP_REASON_INTERNAL,
                    ZY100_ONLINE_ABORT_ORIGIN_IMU_RAW_FINALIZE,
                    1U,
                    zy100_os_time_ms());
            }
            capture_stopped = true;
        }
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_FINAL_EDGE_TIMER_ENABLE
        v0_final_edge_timer_stop();
        if (!v0_final_edge_final_drain(&stats))
        {
            stats.fatal_error = true;
            stats.fatal_status = IMU_STATUS_TIMEOUT;
        }
#endif
        zy100_spi_sched_capture_stop();
        stats.capture_active = false;
        s_v0_capture_active = false;

#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
        if (online_raw_mode &&
            (s_v0_online_abort_requested ||
             zy100_online_stream_end_requested() ||
             (s_v0_stop_reason != IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE)))
        {
            /* Terminal online stop does not drain/resume. Disable the FIFO
             * producer before any post-stop diagnostics or Flash cleanup. */
            v0_stop_capture();
            capture_stopped = true;
        }
#endif

#if !ZY100_FINAL_EDGE_MODE_ENABLE
        if (!offline_v2_mode && !stats.fatal_error)
#else
        if (!stats.fatal_error)
#endif
        {
            icm53611_int_status_t int_status;

            /*
             * non-runtime SPI: no RT marker/FIFO scheduler concurrency.
             * Read clear-on-read INT_STATUS registers once for final status.
             */
            final_status_ok = v0_read_int_status(&stats, &int_status);
            if (final_status_ok)
            {
#if V0_FIFO_MARKER_TRIGGER_ENABLED
#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
                if (!online_raw_mode)
#endif
                {
                    v0_process_int_status(&int_status,
                                          s_v0_current_sample_seq,
                                          true);
                }
#endif
            }
        }

#if V0_FIFO_MARKER_TRIGGER_ENABLED
#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
        if (!online_raw_mode)
#endif
        {
            IMU_UNUSED(imu_wom_irq_disable_sensor());
        }
#endif
        if (!capture_stopped)
        {
            v0_stop_capture();
        }
        imu_bsp_int_clear_pending();
        /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
#if ZY100_LEGACY_OFFLINE_ENABLE && !ZY100_FINAL_EDGE_MODE_ENABLE
        if (offline_v2_mode)
        {
            v0_capture_offline_v2_final_fifo_status(&stats);
        }
        else
#endif
        {
            (void)v0_refresh_lost_count(&stats);
        }
    }
    else
    {
        stats.capture_active = false;
        s_v0_capture_active = false;
        imu_bsp_int_register_irq_callback(NULL);
        imu_bsp_int_clear_pending();
        s_v0_hw_ready = false;
    }
#if ZY100_LEGACY_OFFLINE_ENABLE && !ZY100_FINAL_EDGE_MODE_ENABLE
    if (offline_v2_mode)
    {
        v0_log_offline_v2_end_diag(&stats,
                                   v0_elapsed_ms(capture_start_ms));
    }
#endif
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    if (online_raw_mode)
    {
        bool graceful_host_pause =
            !stats.fatal_error &&
            !s_v0_online_abort_requested &&
            zy100_online_stream_end_requested() &&
            (zy100_online_stream_capture_stop_reason() ==
             ZY100_ONLINE_STOP_REASON_HOST_STOP) &&
            (s_v0_stop_reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_PAUSED);
        bool discard_tail = !graceful_host_pause;
        bool finish_ok;

#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
        s_v0_online_resource.capture_ms = v0_elapsed_ms(capture_start_ms);
#endif
        s_v0_online_resource.packets = stats.packet_count;
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
        s_v0_online_resource.fifo_hi = stats.max_fifo_count;
#endif
        s_v0_online_resource.full = stats.fifo_full_count;
        s_v0_online_resource.lost = stats.fifo_lost_pkt_count;
        s_v0_online_resource.read_errors = stats.read_error_count;
        s_v0_online_resource.bad_headers = stats.bad_header_count;
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
        s_v0_online_resource.gap_ms = stats.max_loop_gap_ms;
#endif
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE && (INCLUDE_uxTaskGetStackHighWaterMark == 1)
        s_v0_online_resource.stack_free =
            (uint32_t)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);
#endif
        s_v0_online_resource.pending = true;
        v0_online_pause_set_phase(ZY100_ONLINE_PAUSE_PHASE_RAW_FINAL);
        v0_online_pause_note_progress();
        finish_ok = zy100_online_raw_capture_finish(
            discard_tail,
            zy100_os_time_ms(),
            v0_online_pause_note_progress);
#if ZY100_ONLINE_STRESS_TEST_ENABLE && (INCLUDE_uxTaskGetStackHighWaterMark == 1)
        zy100_online_stress_note_stacks(0xFFFFFFFFUL,
            (uint32_t)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t),
            stats.max_loop_gap_ms);
#endif
        if (finish_ok && graceful_host_pause)
        {
            zy100_online_raw_capture_stats_t raw_stats;
            zy100_online_raw_clock_meta_t raw_clock;
            zy100_online_clock_meta_t stream_clock;

            memset(&raw_stats, 0, sizeof(raw_stats));
            memset(&raw_clock, 0, sizeof(raw_clock));
            memset(&stream_clock, 0, sizeof(stream_clock));
            zy100_online_raw_capture_get_stats(&raw_stats);
            zy100_online_raw_capture_get_clock_meta(&raw_clock);
            if ((raw_stats.accepted_packets != raw_stats.committed_packets) ||
                (raw_stats.discarded_packets != 0U) ||
                (raw_stats.accepted_mag_samples !=
                 raw_stats.committed_mag_samples) ||
                (raw_stats.discarded_mag_samples != 0U) ||
                (raw_clock.accepted_packets != raw_stats.accepted_packets))
            {
                finish_ok = false;
            }
            else
            {
                stream_clock.accepted_packet_count =
                    raw_stats.accepted_packets;
                stream_clock.first_imu_timestamp_raw =
                    raw_clock.first_timestamp_raw;
                stream_clock.last_imu_timestamp_raw =
                    raw_clock.last_timestamp_raw;
                stream_clock.rtc_nominal_tick_hz =
                    raw_clock.nominal_tick_hz;
                stream_clock.first_rtc_tick = raw_clock.first_rtc_tick;
                stream_clock.last_rtc_tick = raw_clock.last_rtc_tick;
                stream_clock.rtc_wrap_ticks = raw_clock.rtc_wrap_ticks;
                stream_clock.first_unix_time_ms =
                    raw_clock.first_unix_time_ms;
                stream_clock.last_unix_time_ms =
                    raw_clock.last_unix_time_ms;
                stream_clock.first_unix_time_us =
                    raw_clock.first_unix_time_us;
                stream_clock.last_unix_time_us =
                    raw_clock.last_unix_time_us;

                if (raw_clock.fixed40_endpoint != 0U)
                {
                    if ((raw_clock.first_fixed40_valid == 0U) ||
                        (raw_clock.last_fixed40_valid == 0U))
                    {
                        stream_clock.status =
                            ZY100_ONLINE_CLOCK_META_TIMEBASE_UNAVAILABLE;
                    }
                    else if (raw_stats.accepted_packets < 2U)
                    {
                        stream_clock.status =
                            ZY100_ONLINE_CLOCK_META_LESS_THAN_TWO_PACKETS;
                    }
                    else if (raw_clock.fixed40_range_unsupported != 0U)
                    {
                        stream_clock.status =
                            ZY100_ONLINE_CLOCK_META_TIMEBASE_RANGE_UNSUPPORTED;
                    }
                    else if ((raw_clock.first_unix_valid == 0U) ||
                             (raw_clock.last_unix_valid == 0U) ||
                             (raw_clock.first_unix_time_us == 0ULL))
                    {
                        stream_clock.status =
                            ZY100_ONLINE_CLOCK_META_UNIX_UNAVAILABLE;
                    }
                    else if (raw_clock.last_unix_time_us <
                             raw_clock.first_unix_time_us)
                    {
                        stream_clock.status =
                            ZY100_ONLINE_CLOCK_META_TIMEBASE_RANGE_UNSUPPORTED;
                    }
                    else
                    {
                        stream_clock.status = ZY100_ONLINE_CLOCK_META_VALID;
                    }
                }
                else if ((raw_clock.first_rtc_valid == 0U) ||
                         (raw_clock.last_rtc_valid == 0U) ||
                         (raw_clock.nominal_tick_hz == 0U) ||
                         (raw_clock.rtc_wrap_ticks == 0ULL) ||
                         (raw_clock.first_rtc_tick >=
                          raw_clock.rtc_wrap_ticks) ||
                         (raw_clock.last_rtc_tick >=
                          raw_clock.rtc_wrap_ticks))
                {
                    stream_clock.status =
                        ZY100_ONLINE_CLOCK_META_RTC_UNAVAILABLE;
                }
                else if (raw_stats.accepted_packets < 2U)
                {
                    stream_clock.status =
                        ZY100_ONLINE_CLOCK_META_LESS_THAN_TWO_PACKETS;
                }
                else if ((raw_clock.first_unix_valid == 0U) ||
                         (raw_clock.last_unix_valid == 0U) ||
                         (raw_clock.first_unix_time_ms == 0ULL) ||
                         (raw_clock.last_unix_time_ms <
                          raw_clock.first_unix_time_ms))
                {
                    stream_clock.status =
                        ZY100_ONLINE_CLOCK_META_UNIX_UNAVAILABLE;
                }
                else
                {
                    /* RTC ticks remain authoritative for duration/rate. The
                     * Unix endpoints anchor that interval to wall-clock time. */
                    stream_clock.status = ZY100_ONLINE_CLOCK_META_VALID;
                }
                zy100_online_stream_set_clock_meta(&stream_clock);
            }
        }
        if (!finish_ok)
        {
            stats.fatal_error = true;
            stats.fatal_status = IMU_STATUS_BUS_ERROR;
            v0_online_abort_diag_capture_end(&stats);
            zy100_online_stream_request_abort_ex(
                ZY100_ONLINE_STOP_REASON_INTERNAL,
                ZY100_ONLINE_ABORT_ORIGIN_IMU_RAW_FINALIZE,
                4U,
                zy100_os_time_ms());
        }
    }
#endif
    if (started)
    {
#if ZY100_FINAL_EDGE_MODE_ENABLE
        zy100_edge_lite_output_t final_edge_out;

        memset(&final_edge_out, 0, sizeof(final_edge_out));
#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
        if (online_raw_mode)
        {
            /* No Edge/RT/ABA partial state exists in continuous RAW mode. */
        }
        else
#endif
        if (s_v0_stop_reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_PAUSED)
        {
            (void)zy100_edge_shadow_capture_end_discard_partial(&final_edge_out);
        }
        else if (zy100_edge_shadow_capture_end_ex(&final_edge_out))
        {
            v0_final_edge_handle_lite_output(&final_edge_out,
                                             s_final_edge_edge_sample_index,
                                             0U);
        }
#elif ZY100_EDGE_SHADOW_ENABLE
        zy100_edge_shadow_capture_end();
#endif
    }
    {
        uint32_t final_runtime_ms = started ? v0_elapsed_ms(capture_start_ms) : 0U;
#if ZY100_FINAL_EDGE_MODE_ENABLE
        if (started
#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
            && !online_raw_mode
#endif
           )
        {
#if ZY100_ONLINE_STREAM_ENABLE
            if (s_v0_online_abort_requested)
            {
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
                zy100_final_edge_raw_store_abort_pending();
#endif
                zy100_final_edge_record_store_abort_active();
            }
            else
#endif
            {
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
                v0_final_edge_set_state(FE_CAPTURE_STATE_DRAIN_RAW_STORE);
                if (!v0_final_edge_raw_store_final_pump())
                {
                    ZY100_DIAG_LOG("[FE_RAW_FINAL] idle=0 timeout_ms=%u",
                               (uint32_t)ZY100_FINAL_EDGE_RAW_STORE_FINAL_PUMP_TIMEOUT_MS);
                }
#endif
                v0_final_edge_set_state(FE_CAPTURE_STATE_DRAIN_RECORD_STORE);
                if (!v0_final_edge_record_store_final_pump())
                {
                    ZY100_DIAG_LOG("[FE_STORE_FINAL] idle=0 timeout_ms=%u",
                               (uint32_t)ZY100_FINAL_EDGE_EDGE_STORE_FINAL_PUMP_TIMEOUT_MS);
                }
            }
            if ((s_v0_stop_reason ==
                 IMU_FIFO_DRAIN_TEST_STOP_REASON_PAUSED) &&
                v0_final_edge_online_stop_active())
            {
                v0_final_edge_set_state(FE_CAPTURE_STATE_IDLE);
            }
            else
            {
                v0_final_edge_set_state(FE_CAPTURE_STATE_WRITE_META);
                zy100_capture_time_note_stop();
                if (!v0_final_edge_write_session_meta(&stats,
                                                      capture_start_ms,
                                                      final_runtime_ms,
                                                      NULL))
                {
                    ZY100_DIAG_LOG("[FE_META] write=0 verify=0 bytes=0 flags=00000000");
                    v0_final_edge_set_state(FE_CAPTURE_STATE_ERROR);
                    v0_final_edge_request_stop(ZY100_FE_STOP_REASON_ERROR,
                                               "meta");
                }
                else if (!stats.fatal_error &&
                         !s_final_edge_empty_error_aborted)
                {
                    stats.reached_final_summary = true;
                    s_fe_export_ready = true;
                    s_fe_export_preserved = true;
                    s_fe_export_cleared = false;
                    v0_final_edge_set_state(FE_CAPTURE_STATE_EXPORT_READY);
                    ZY100_DIAG_LOG("[FE_EXPORT_READY] reason=%u",
                               (uint32_t)s_fe_stop_reason);
                }
            }
        }
#endif
#if V0_FLASH_RUNTIME_ENABLE
        external_flash_capture_imu_stats_t imu_summary;
        imu_fifo_drain_test_stop_reason_t final_reason = s_v0_stop_reason;

        if ((final_reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE) &&
            (external_flash_capture_stop_reason() != 0U))
        {
            final_reason = (imu_fifo_drain_test_stop_reason_t)external_flash_capture_stop_reason();
        }
        if ((final_reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE) && !started)
        {
            final_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL;
        }
        if (stats.fatal_error ||
            (!offline_v2_mode && external_flash_capture_has_error()))
        {
            final_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL;
        }
        stats.stop_reason = final_reason;
        s_v0_last_stop_reason = final_reason;
        v1_copy_imu_stats(&stats, &imu_summary);
        if (started)
        {
#if V0_FIFO_MARKER_TRIGGER_ENABLED
            IMU_UNUSED(imu_pre_trigger_marker_finish(imu_bsp_local_timestamp_us()));
            IMU_UNUSED(imu_swing_detector_finish(imu_bsp_local_timestamp_us()));
#if ZY100_FIFO_SWING_STATS_ENABLE
            imu_wom_irq_log_config();
            imu_wom_irq_log_stats();
            imu_wom_irq_log_events();
            imu_wom_irq_log_swing_stats();
            imu_wom_irq_log_swing_events();
            imu_swing_detector_log_config();
            imu_swing_detector_log_stats();
            v0_log_combined_imu_swing_stats();
            imu_swing_detector_log_events();
#endif
#endif
#if ZY100_RT_MARKER_ENABLE
            if (!offline_v2_mode)
            {
                imu_rt_marker_finish();
            }
#endif
        }
#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE && 1
        if (!offline_v2_mode &&
            !v0_phase_a_flush_ingress_until_empty(&stats))
        {
            final_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL;
            stats.stop_reason = final_reason;
            s_v0_last_stop_reason = final_reason;
            v1_copy_imu_stats(&stats, &imu_summary);
        }
        {
            imu_rt_marker_stats_t rt_stats;

            imu_rt_marker_get_stats(&rt_stats);
            if (!offline_v2_mode && !v0_online_quiet_logs())
            {
                DBG_DIRECT("[PHASE_A_SUM] fifo_poll=%u fifo_drain=%u ingress_max=%u ingress_overflow=%u ingress_flush_bytes=%u marker_count=%u",
                           stats.poll_count,
                           stats.irq_drain_count,
                           stats.ingress_max_level,
                           zy100_fifo_ingress_overflow_count(),
                           stats.ingress_flush_bytes,
                           rt_stats.marker_total);
                ZY100_DIAG_LOG("[PHASE_A_SUM] bp_enter=%u bp_pump=%u bp_flush=%u bp_recovered=%u bp_fatal=%u ingress_free_min=%u ingress_level_max=%u flash_pump_count=%u flash_block_done_count=%u",
                           stats.ingress_backpressure_enter,
                           stats.ingress_backpressure_pump,
                           stats.ingress_backpressure_flush,
                           stats.ingress_backpressure_recovered,
                           stats.ingress_backpressure_fatal,
                           stats.ingress_free_min,
                           stats.ingress_level_max,
                           stats.flash_pump_count,
                           stats.flash_block_done_count);
            }
        }
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE
        if (started)
        {
            v0_final_edge_log_ois_summary();
            v0_final_edge_end_summary_log_delay();
            v0_final_edge_log_bscan_end_summary();
        }
#endif
        if (v1_flash_prepared || v1_flash_session_started)
        {
            external_flash_capture_finalize((uint32_t)final_reason, &imu_summary);
        }
#if ZY100_SRAM_SUMMARY_LOG_ENABLE
        if (started)
        {
            v0_sram_capture_end(&sram_stats);
        }
        v0_sram_log_summary(&sram_stats);
#endif
        if (started && !offline_v2_mode && !v0_online_quiet_logs())
        {
            zy100_edge_shadow_print_summary();
#if ZY100_FINAL_EDGE_MODE_ENABLE
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
            zy100_edge_shadow_check_sample_mismatch(
                stats.edge_hook_feed_count +
                v0_final_edge_replay_edge_total());
#else
            zy100_edge_shadow_check_sample_mismatch(stats.edge_hook_feed_count);
#endif
#else
            zy100_edge_shadow_check_sample_mismatch(stats.edge_hook_feed_count);
#endif
        }
        IMU_UNUSED(final_runtime_ms);
#else
#if ZY100_FINAL_EDGE_MODE_ENABLE
        v0_final_edge_pass_diag_t fe_pass_diag;
        bool data_pass = v0_data_passed(&stats, final_runtime_ms,
                                        &fe_pass_diag);
#else
        bool data_pass = v0_data_passed(&stats, final_runtime_ms);
#endif
        bool wm_irq_pass = v0_wm_irq_passed(&stats);
        imu_fifo_drain_test_stop_reason_t final_reason = s_v0_stop_reason;

        if (stats.fatal_error)
        {
            final_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL;
        }
        stats.stop_reason = final_reason;
        s_v0_last_stop_reason = final_reason;

        if (started)
        {
#if V0_FIFO_MARKER_TRIGGER_ENABLED
            IMU_UNUSED(imu_pre_trigger_marker_finish(imu_bsp_local_timestamp_us()));
            IMU_UNUSED(imu_swing_detector_finish(imu_bsp_local_timestamp_us()));
#if ZY100_FIFO_SWING_STATS_ENABLE
            imu_wom_irq_log_config();
            imu_wom_irq_log_stats();
            imu_wom_irq_log_events();
            imu_wom_irq_log_swing_stats();
            imu_wom_irq_log_swing_events();
            imu_swing_detector_log_config();
            imu_swing_detector_log_stats();
            v0_log_combined_imu_swing_stats();
            imu_swing_detector_log_events();
#endif
#if ZY100_FIFO_MARKER_SUMMARY_ENABLE
            imu_pre_trigger_marker_log_config();
            imu_pre_trigger_marker_log_stats();
            imu_sample_marker_log_config();
            imu_sample_marker_log_stats();
            imu_sample_marker_log_markers();
#endif
#endif
#if ZY100_RT_MARKER_ENABLE
            imu_rt_marker_finish();
#endif
        }
        v0_log_stats(&stats, final_runtime_ms, true);
#if ZY100_FINAL_EDGE_MODE_ENABLE
        if (started)
        {
            v0_final_edge_log_ois_summary();
            v0_final_edge_end_summary_log_delay();
            v0_final_edge_log_bscan_end_summary();
        }
        if (ZY100_LOG_FE_DIAG_VERBOSE)
        {
            v0_final_edge_log_stats(&stats, final_runtime_ms, true);
            v0_final_edge_log_pass_diag(&fe_pass_diag);
        }
#if ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE && \
    ZY100_FINAL_EDGE_OFFLINE_B_CRITICAL_BYPASS_VERIFY_ENABLE
        if (started)
        {
            ZY100_DIAG_LOG("[FE_BCRIT_AB_SUM] en=1 hit=%u post_hit=%u fifo_max=%u full=%u lost=%u emg=%u fatal=%u",
                       s_final_edge_phase1_stats
                           .b_critical_offline_bypass_count,
                       s_final_edge_phase1_stats
                           .b_critical_offline_post_quiet_bypass_count,
                       stats.max_fifo_count,
                       stats.fifo_full_count,
                       (uint32_t)stats.fifo_lost_pkt_count,
                       s_final_edge_phase1_stats.fifo_emergency_count,
                       stats.fatal_error ? 1U : 0U);
        }
#endif
#endif
        if (IMU_CAPTURE_SUMMARY_LOG_ENABLE || stats.fatal_error)
        {
            DBG_DIRECT("[V0_IMU_FIFO][RESULT] round=%u data_pass=%u wm_irq_pass=%u fatal=%u status=%u reason=%u",
                       stats.round,
                       data_pass ? 1U : 0U,
                       wm_irq_pass ? 1U : 0U,
                       stats.fatal_error ? 1U : 0U,
                       (uint32_t)stats.fatal_status,
                       (uint32_t)stats.stop_reason);
        }
#if ZY100_SRAM_SUMMARY_LOG_ENABLE
        if (started)
        {
            v0_sram_capture_end(&sram_stats);
        }
        v0_sram_log_summary(&sram_stats);
#endif
        if (started && !v0_online_quiet_logs())
        {
            zy100_edge_shadow_print_summary();
#if ZY100_FINAL_EDGE_MODE_ENABLE
#if ZY100_FINAL_EDGE_PHASE6_NOHIT_REPLAY_ENABLE
            zy100_edge_shadow_check_sample_mismatch(
                stats.edge_hook_feed_count +
                v0_final_edge_replay_edge_total());
#else
            zy100_edge_shadow_check_sample_mismatch(stats.edge_hook_feed_count);
#endif
#else
            zy100_edge_shadow_check_sample_mismatch(stats.edge_hook_feed_count);
#endif
        }
#endif
    }

#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    if (online_raw_mode || zy100_online_stream_active() ||
        zy100_online_stream_end_wait_ack())
    {
        v0_online_abort_diag_capture_end(&stats);
    }
#endif

    s_v0_start_pending = false;
    s_v0_stop_requested = false;
    s_v0_stop_in_progress = false;
    s_v0_online_pause_stop_requested = false;
    s_v0_stop_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
#if V1_IMU_FLASH_CAPTURE_ENABLE
    s_v1_flash_ready_wait_start = false;
#endif
    s_v0_imu_fifo_pending_count = 0U;
    s_v0_high_water_alarm_pending = false;
    s_v0_phase_a_rescue_pending = false;
    v0_adc_safe_window_close();
    s_v0_spi_fifo_busy = false;
    s_v0_worker_busy = false;
    s_v0_current_round = 0U;
    s_v0_capture_done = true;
    s_v0_completion_generation++;
    v0_online_pause_set_phase(ZY100_ONLINE_PAUSE_PHASE_STOP_DONE);
    v0_online_pause_note_progress();
}

static void v0_fifo_task(void *param)
{
    IMU_UNUSED(param);

    while (true)
    {
        uint32_t notify_value = 0U;

        if (xTaskNotifyWait(0U, V0_NOTIFY_MASK, &notify_value, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        if ((notify_value & V0_NOTIFY_START) == 0U)
        {
            continue;
        }

        if (!s_v0_start_pending)
        {
            continue;
        }

        s_v0_worker_busy = true;
        s_v0_start_pending = false;
        v0_run_capture_session(s_v0_current_round);
    }
}

static uint32_t v0_task_init_heap_free_now(void)
{
#if ZY100_SRAM_SUMMARY_LOG_ENABLE
    return v0_sram_heap_free_now();
#else
    return 0U;
#endif
}

static uint32_t v0_task_init_heap_min_free(void)
{
#if ZY100_SRAM_SUMMARY_LOG_ENABLE
    return v0_sram_heap_min_ever_free();
#else
    return 0U;
#endif
}

bool imu_fifo_drain_test_task_init(void)
{
    IMU_UNUSED(v0_offline_stall_diag);
    V0_IMU_LIFECYCLE_LOG("[V0_IMU_FIFO] task_init enter handle=%u heap=%u min=%u",
                         (s_v0_task_handle != NULL) ? 1U : 0U,
                         v0_task_init_heap_free_now(),
                         v0_task_init_heap_min_free());
    V0_IMU_LIFECYCLE_LOG("[V0_IMU_FIFO] shadow_init_begin");
    zy100_edge_shadow_init();
    V0_IMU_LIFECYCLE_LOG("[V0_IMU_FIFO] shadow_init_done");

    if (s_v0_task_handle != NULL)
    {
        V0_IMU_LIFECYCLE_LOG("[V0_IMU_FIFO] task_exists");
        return true;
    }

    V0_IMU_LIFECYCLE_LOG("[V0_IMU_FIFO] create_begin stack_words=%u prio=%u heap=%u min=%u",
                         (uint32_t)V0_TASK_STACK_WORDS,
                         (uint32_t)V0_TASK_PRIORITY,
                         v0_task_init_heap_free_now(),
                         v0_task_init_heap_min_free());
    if (xTaskCreate(v0_fifo_task,
                    "v0_imu_fifo",
                    V0_TASK_STACK_WORDS,
                    NULL,
                    V0_TASK_PRIORITY,
                    &s_v0_task_handle) != pdPASS)
    {
        DBG_DIRECT("[V0_IMU_FIFO][ERR] create task failed heap=%u min=%u",
                   v0_task_init_heap_free_now(),
                   v0_task_init_heap_min_free());
        return false;
    }

    V0_IMU_LIFECYCLE_LOG("[V0_IMU_FIFO] create_ok heap=%u min=%u",
                         v0_task_init_heap_free_now(),
                         v0_task_init_heap_min_free());
    return true;
}

bool imu_fifo_drain_test_task_release_idle(void)
{
    TaskHandle_t task_to_delete;
    uint32_t lock_state = os_lock();

    if (s_v0_capture_active || s_v0_worker_busy || s_v0_spi_fifo_busy ||
        s_v0_stop_in_progress || s_v0_start_pending)
    {
        os_unlock(lock_state);
        return false;
    }
    task_to_delete = s_v0_task_handle;
    s_v0_task_handle = NULL;
    os_unlock(lock_state);

    if (task_to_delete != NULL)
    {
        vTaskDelete(task_to_delete);
    }
    return true;
}

#if V1_IMU_FLASH_CAPTURE_ENABLE
static void v1_prepare_flash_reset_runtime(uint32_t round)
{
    s_v0_imu_fifo_pending_count = 0U;
    s_v0_high_water_alarm_pending = false;
    s_v0_phase_a_rescue_pending = false;
    s_v0_irq_count = 0U;
    s_v0_capture_done = false;
    s_v0_stop_requested = false;
    s_v0_stop_in_progress = false;
    s_v0_start_pending = false;
    s_v0_capture_active = false;
    s_v0_worker_busy = true;
    s_v0_spi_fifo_busy = true;
    s_v0_stop_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
    s_v0_last_stop_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
    s_v0_current_round = round;
    s_v1_flash_ready_wait_start = false;
#if ZY100_FINAL_EDGE_MODE_ENABLE
    s_final_edge_record_prepare_started = false;
    s_final_edge_dir_prepare_started = false;
    s_final_edge_raw_prepare_started = false;
    s_final_edge_online_spool_prepare_started = false;
#if ZY100_ONLINE_DIRECT_SPOOL_ENABLE
    s_final_edge_online_target_prepare = false;
#endif
    s_final_edge_empty_error_aborted = false;
    s_final_edge_store_prep_last_phase = 0U;
    s_final_edge_store_prep_last_done = 0xFFFFFFFFU;
    s_final_edge_store_prep_last_total = 0U;
    s_final_edge_store_prep_last_err = 0U;
    s_final_edge_store_prep_done_logged = false;
#endif
#if V0_FLASH_RUNTIME_ENABLE
    s_v1_flash_prepare_busy_logged = false;
#endif
}

#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
static void v0_final_edge_log_raw_prepare_progress(
    zy100_flash_prepare_status_t status)
{
    zy100_fe_raw_prepare_progress_t progress;
    bool phase_changed;
    bool err_log;
    bool done_changed;
    bool done_step;
    bool done_complete;
    bool done_complete_first;
    bool total_changed;

    memset(&progress, 0, sizeof(progress));
    zy100_final_edge_raw_store_get_prepare_progress(&progress);
    phase_changed = (progress.phase != s_final_edge_store_prep_last_phase);
    err_log = (status == ZY100_FLASH_PREP_ERROR) ||
              ((progress.err != 0U) &&
               (progress.err != s_final_edge_store_prep_last_err));
    done_changed = (progress.done != s_final_edge_store_prep_last_done);
    done_step =
        (s_final_edge_store_prep_last_done == 0xFFFFFFFFU) ||
        ((progress.done >= s_final_edge_store_prep_last_done) &&
         ((progress.done - s_final_edge_store_prep_last_done) >= 32U));
    done_complete = (progress.total != 0U) && (progress.done >= progress.total);
    done_complete_first = done_complete && !s_final_edge_store_prep_done_logged;
    total_changed = (progress.total != s_final_edge_store_prep_last_total);

    if (phase_changed)
    {
        s_final_edge_store_prep_done_logged = false;
    }

    if (phase_changed ||
        total_changed ||
        err_log ||
        (done_changed && done_step) ||
        done_complete_first)
    {
        if (err_log || !v0_online_quiet_logs())
        {
            DBG_DIRECT("[FE_STORE_PREP] phase=%u erase=%u total=%u done=%u err=%u",
                       progress.phase,
                       progress.erase,
                       progress.total,
                       progress.done,
                       progress.err);
        }
        s_final_edge_store_prep_last_phase = progress.phase;
        s_final_edge_store_prep_last_done = progress.done;
        s_final_edge_store_prep_last_total = progress.total;
        s_final_edge_store_prep_last_err = progress.err;
        if (done_complete)
        {
            s_final_edge_store_prep_done_logged = true;
        }
    }
}
#else
static void v0_final_edge_log_raw_prepare_progress(
    zy100_flash_prepare_status_t status) __attribute__((unused));
static void v0_final_edge_log_raw_prepare_progress(
    zy100_flash_prepare_status_t status)
{
    IMU_UNUSED(status);
}
#endif

bool imu_fifo_drain_test_prepare_flash_begin(void)
{
    uint32_t round;
    imu_status_t resume_status;
    bool started = false;

    IMU_UNUSED(resume_status);
    IMU_UNUSED(started);

    if (s_v0_capture_active || s_v0_worker_busy || s_v0_stop_in_progress || s_v0_start_pending)
    {
        return false;
    }

    if (!imu_fifo_drain_test_task_init())
    {
        return false;
    }

    round = s_v0_next_round;
    v1_prepare_flash_reset_runtime(round);
#if ZY100_FINAL_EDGE_MODE_ENABLE
    s_final_edge_prepare_mode = V0_FE_PREPARE_MODE_CLEAR_ALL;
    s_final_edge_active_session_valid = false;
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    resume_status = v0_imu_platform_resume_for_capture(round);
    if (resume_status == IMU_STATUS_OK)
    {
#if ZY100_MULTI_SESSION_STORAGE_ENABLE
        started = zy100_session_dir_prepare_clear_begin();
        s_final_edge_dir_prepare_started = started;
        s_final_edge_raw_prepare_started = false;
#else
        started = zy100_final_edge_raw_store_prepare_erase_begin(round);
        s_final_edge_raw_prepare_started = started;
#endif
    }
    else
    {
        DBG_DIRECT("[ERR][IMU] prepare_resume_failed status=%u",
                   (uint32_t)resume_status);
    }

    s_v0_worker_busy = false;
    s_v0_spi_fifo_busy = false;

    if (!started)
    {
        DBG_DIRECT("[FE_RAW_PREP] begin_failed round=%u", round);
        s_v0_current_round = 0U;
        s_v0_hw_ready = false;
        return false;
    }
    return true;
#else
    s_v0_worker_busy = false;
    s_v0_spi_fifo_busy = false;
    s_v1_flash_ready_wait_start = true;
    return true;
#endif
#else
    DBG_DIRECT("[EDGE_BOOT] active=%u old_raw=%u",
               0U,
               1U);
    DBG_DIRECT("[FLASH_PREPARE] begin round=%u edge=%u",
               round,
               0U);

    resume_status = v0_imu_platform_resume_for_capture(round);
    if (resume_status == IMU_STATUS_OK)
    {
        started = external_flash_capture_prepare_begin(round);
    }
    else
    {
        DBG_DIRECT("[ERR][IMU] prepare_resume_failed status=%u", (uint32_t)resume_status);
    }

    s_v0_worker_busy = false;
    s_v0_spi_fifo_busy = false;

    if (!started)
    {
        DBG_DIRECT("[FLASH_PREPARE] begin_failed round=%u edge=%u",
                   round,
                   0U);
        s_v0_current_round = 0U;
        s_v0_hw_ready = false;
        return false;
    }

    return true;
#endif
}

bool imu_fifo_drain_test_prepare_flash_append_begin(
    const zy100_session_alloc_t *alloc)
{
    uint32_t round;
    imu_status_t resume_status;
    bool started = false;

    IMU_UNUSED(resume_status);
    IMU_UNUSED(started);

    if (alloc == NULL)
    {
        return false;
    }
    if (s_v0_capture_active || s_v0_worker_busy || s_v0_stop_in_progress ||
        s_v0_start_pending)
    {
        return false;
    }
    if (!imu_fifo_drain_test_task_init())
    {
        return false;
    }

    round = (alloc->round != 0U) ? alloc->round : s_v0_next_round;
    v1_prepare_flash_reset_runtime(round);
#if ZY100_FINAL_EDGE_MODE_ENABLE
    s_final_edge_prepare_mode = V0_FE_PREPARE_MODE_APPEND_SESSION;
    s_final_edge_active_session_alloc = *alloc;
    s_final_edge_active_session_valid = true;
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    resume_status = v0_imu_platform_resume_for_capture(round);
    if (resume_status == IMU_STATUS_OK)
    {
        started = zy100_final_edge_raw_store_prepare_append_begin(
            round,
            alloc->raw_data_begin_addr,
            ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR +
            ZY100_FINAL_EDGE_RAW_REGION_BYTES);
        s_final_edge_raw_prepare_started = started;
    }
    else
    {
        DBG_DIRECT("[ERR][IMU] prepare_resume_failed status=%u",
                   (uint32_t)resume_status);
    }

    s_v0_worker_busy = false;
    s_v0_spi_fifo_busy = false;

    if (!started)
    {
        DBG_DIRECT("[FE_RAW_PREP] append_begin_failed round=%u uid=%lu",
                   round,
                   (unsigned long)alloc->session_uid);
        s_v0_current_round = 0U;
        s_v0_hw_ready = false;
        s_final_edge_active_session_valid = false;
        return false;
    }
    return true;
#else
    s_v0_worker_busy = false;
    s_v0_spi_fifo_busy = false;
    s_v1_flash_ready_wait_start = true;
    return true;
#endif
#else
    (void)round;
    return false;
#endif
}

bool imu_fifo_drain_test_prepare_online_begin(uint32_t round)
{
#if ZY100_FINAL_EDGE_MODE_ENABLE && \
    ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE && \
    ZY100_ONLINE_DIRECT_SPOOL_ENABLE
    imu_status_t resume_status;
    bool started = false;
    const zy100_fe_store_target_provider_t *provider;

    if (s_v0_capture_active || s_v0_worker_busy || s_v0_stop_in_progress ||
        s_v0_start_pending || (round == 0U))
    {
        return false;
    }
    if (!imu_fifo_drain_test_task_init())
    {
        return false;
    }
    provider = zy100_online_spool_target_provider();
    if (provider == NULL)
    {
        return false;
    }
    v1_prepare_flash_reset_runtime(round);
    s_final_edge_prepare_mode = V0_FE_PREPARE_MODE_APPEND_SESSION;
    s_final_edge_online_target_prepare = true;
    s_final_edge_active_session_valid = false;
    memset(&s_final_edge_active_session_alloc,
           0,
           sizeof(s_final_edge_active_session_alloc));

    resume_status = v0_imu_platform_resume_for_capture(round);
    if (resume_status == IMU_STATUS_OK)
    {
        started = zy100_final_edge_raw_store_prepare_target_begin(
            round, provider);
        s_final_edge_raw_prepare_started = started;
    }
    s_v0_worker_busy = false;
    s_v0_spi_fifo_busy = false;
    if (!started)
    {
        s_v0_current_round = 0U;
        s_v0_hw_ready = false;
        s_final_edge_online_target_prepare = false;
        return false;
    }
    return true;
#else
    (void)round;
    return false;
#endif
}

#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_DIRECT_SPOOL_ENABLE && \
    ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
static imu_online_preflight_status_t v0_online_preflight_result(
    imu_online_preflight_status_t status,
    uint32_t round)
{
    if (status != IMU_ONLINE_PREFLIGHT_OK)
    {
        DBG_DIRECT("[ONLINE_PREFLIGHT] st=%u round=%lu active=%u worker=%u spi=%u stop=%u start=%u provider=%u workspace=%u flash_wip=%u",
                   (uint32_t)status,
                   (unsigned long)round,
                   s_v0_capture_active ? 1U : 0U,
                   s_v0_worker_busy ? 1U : 0U,
                   s_v0_spi_fifo_busy ? 1U : 0U,
                   s_v0_stop_in_progress ? 1U : 0U,
                   s_v0_start_pending ? 1U : 0U,
                   (zy100_online_spool_target_provider() != NULL) ? 1U : 0U,
                   zy100_mode_workspace_available() ? 1U : 0U,
                   zy100_online_spool_erase_wip() ? 1U : 0U);
    }
    return status;
}
#endif

imu_online_preflight_status_t
imu_fifo_drain_test_prepare_continuous_online(uint32_t round)
{
#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_DIRECT_SPOOL_ENABLE && \
    ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
    imu_status_t resume_status;

    if ((round == 0U) || s_v0_capture_active || s_v0_stop_in_progress ||
        s_v0_start_pending)
    {
        return v0_online_preflight_result(IMU_ONLINE_PREFLIGHT_FATAL, round);
    }
    if (s_v0_worker_busy || s_v0_spi_fifo_busy)
    {
        return v0_online_preflight_result(IMU_ONLINE_PREFLIGHT_RETRY_BUSY,
                                          round);
    }
    if (!imu_fifo_drain_test_task_init())
    {
        return v0_online_preflight_result(IMU_ONLINE_PREFLIGHT_RETRY_BUSY,
                                          round);
    }
    if (zy100_online_spool_target_provider() == NULL)
    {
        return v0_online_preflight_result(
                   IMU_ONLINE_PREFLIGHT_PROVIDER_INVALID,
                   round);
    }
    if (!zy100_mode_workspace_available())
    {
        return v0_online_preflight_result(IMU_ONLINE_PREFLIGHT_WORKSPACE_BUSY,
                                          round);
    }

    v1_prepare_flash_reset_runtime(round);
    resume_status = v0_imu_platform_resume_for_capture(round);
    s_v0_worker_busy = false;
    s_v0_spi_fifo_busy = false;
    if (resume_status != IMU_STATUS_OK)
    {
        s_v0_current_round = 0U;
        s_v0_hw_ready = false;
        return v0_online_preflight_result(
                   ((resume_status == IMU_STATUS_NOT_READY) ||
                    (resume_status == IMU_STATUS_BUS_ERROR) ||
                    (resume_status == IMU_STATUS_TIMEOUT)) ?
                   IMU_ONLINE_PREFLIGHT_RETRY_RESUME :
                   IMU_ONLINE_PREFLIGHT_FATAL,
                   round);
    }
    s_v1_flash_ready_wait_start = true;
    s_v0_next_round = (s_v0_next_round == 0xFFFFFFFFU) ?
                      1U : (s_v0_next_round + 1U);
    return IMU_ONLINE_PREFLIGHT_OK;
#else
    (void)round;
    return IMU_ONLINE_PREFLIGHT_FATAL;
#endif
}

void imu_fifo_drain_test_prepare_continuous_online_abort(void)
{
#if V1_IMU_FLASH_CAPTURE_ENABLE
    if (!s_v0_capture_active && !s_v0_start_pending)
    {
        s_v0_worker_busy = false;
        s_v0_spi_fifo_busy = false;
        s_v0_current_round = 0U;
        s_v1_flash_ready_wait_start = false;
        s_v0_hw_ready = false;
    }
#endif
}

void imu_fifo_drain_test_clear_active_session_alloc(void)
{
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
    s_final_edge_active_session_valid = false;
    s_final_edge_empty_error_aborted = false;
    memset(&s_final_edge_active_session_alloc,
           0,
           sizeof(s_final_edge_active_session_alloc));
#endif
}

zy100_flash_prepare_status_t imu_fifo_drain_test_prepare_flash_poll(void)
{
    zy100_flash_prepare_status_t status;

#if ZY100_FINAL_EDGE_MODE_ENABLE
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    if (!s_final_edge_raw_prepare_started)
    {
#if ZY100_MULTI_SESSION_STORAGE_ENABLE
        if ((s_final_edge_prepare_mode == V0_FE_PREPARE_MODE_CLEAR_ALL) &&
            s_final_edge_dir_prepare_started)
        {
            status = zy100_session_dir_prepare_clear_poll();
            if (status == ZY100_FLASH_PREP_BUSY)
            {
                return status;
            }
            if (status != ZY100_FLASH_PREP_DONE)
            {
                s_v1_flash_ready_wait_start = false;
                s_v0_current_round = 0U;
                s_v0_hw_ready = false;
                return status;
            }
            if (!zy100_final_edge_raw_store_prepare_erase_begin(
                    s_v0_current_round))
            {
                s_v1_flash_ready_wait_start = false;
                s_v0_current_round = 0U;
                s_v0_hw_ready = false;
                return ZY100_FLASH_PREP_ERROR;
            }
            s_final_edge_raw_prepare_started = true;
            return ZY100_FLASH_PREP_BUSY;
        }
#endif
        return ZY100_FLASH_PREP_ERROR;
    }
    status = zy100_final_edge_raw_store_prepare_erase_poll();
    v0_final_edge_log_raw_prepare_progress(status);
    if (status == ZY100_FLASH_PREP_BUSY)
    {
        return status;
    }
    if (status == ZY100_FLASH_PREP_DONE)
    {
#if ZY100_MULTI_SESSION_STORAGE_ENABLE && ZY100_ONLINE_STREAM_ENABLE
        if (s_final_edge_prepare_mode == V0_FE_PREPARE_MODE_CLEAR_ALL)
        {
            if (!s_final_edge_online_spool_prepare_started)
            {
                if (!zy100_online_stream_clear_spool_begin())
                {
                    s_v1_flash_ready_wait_start = false;
                    s_v0_current_round = 0U;
                    s_v0_hw_ready = false;
                    return ZY100_FLASH_PREP_ERROR;
                }
                s_final_edge_online_spool_prepare_started = true;
                return ZY100_FLASH_PREP_BUSY;
            }
            status = zy100_online_stream_clear_spool_poll();
            if (status == ZY100_FLASH_PREP_BUSY)
            {
                return status;
            }
            if (status != ZY100_FLASH_PREP_DONE)
            {
                s_v1_flash_ready_wait_start = false;
                s_v0_current_round = 0U;
                s_v0_hw_ready = false;
                return status;
            }
        }
#endif
#if ZY100_FINAL_EDGE_PHASE7_EDGE_FLASH_ENABLE
        if (!s_final_edge_record_prepare_started)
        {
            bool record_started;
#if ZY100_ONLINE_DIRECT_SPOOL_ENABLE
            if (s_final_edge_online_target_prepare)
            {
                record_started =
                    zy100_final_edge_record_store_prepare_target_begin(
                        s_v0_current_round,
                        zy100_online_spool_target_provider());
            }
            else
#endif
#if ZY100_MULTI_SESSION_STORAGE_ENABLE
            if ((s_final_edge_prepare_mode ==
                 V0_FE_PREPARE_MODE_APPEND_SESSION) &&
                s_final_edge_active_session_valid)
            {
                record_started =
                    zy100_final_edge_record_store_prepare_append_begin(
                        s_v0_current_round,
                        s_final_edge_active_session_alloc.summary_data_begin_addr,
                        ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR +
                        ZY100_FINAL_EDGE_SUMMARY_REGION_BYTES,
                        s_final_edge_active_session_alloc.event_data_begin_addr,
                        ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR +
                        ZY100_FINAL_EDGE_EVENT_REGION_BYTES);
            }
            else
#endif
            {
                record_started =
                    zy100_final_edge_record_store_prepare_erase_begin(
                        s_v0_current_round);
            }
            if (!record_started)
            {
                s_v1_flash_ready_wait_start = false;
                s_v0_current_round = 0U;
                s_v0_hw_ready = false;
                return ZY100_FLASH_PREP_ERROR;
            }
            s_final_edge_record_prepare_started = true;
        }
        status = zy100_final_edge_record_store_prepare_erase_poll();
        if (status == ZY100_FLASH_PREP_BUSY)
        {
            return status;
        }
        if (status != ZY100_FLASH_PREP_DONE)
        {
            s_v1_flash_ready_wait_start = false;
            s_v0_current_round = 0U;
            s_v0_hw_ready = false;
            return status;
        }
#endif
        if (!s_v1_flash_ready_wait_start)
        {
            s_v1_flash_ready_wait_start = true;
            s_v0_next_round = (s_v0_next_round == 0xFFFFFFFFU) ?
                              1U : (s_v0_next_round + 1U);
        }
        return status;
    }
    s_v1_flash_ready_wait_start = false;
    s_v0_current_round = 0U;
    s_v0_hw_ready = false;
    return status;
#else
    s_v1_flash_ready_wait_start = true;
    return ZY100_FLASH_PREP_DONE;
#endif
#else

    status = external_flash_capture_prepare_poll();
    if (status == ZY100_FLASH_PREP_BUSY)
    {
        if (!s_v1_flash_prepare_busy_logged)
        {
            DBG_DIRECT("[FLASH_PREPARE] poll busy=1 done=0 ok=1");
            s_v1_flash_prepare_busy_logged = true;
        }
        return status;
    }

    if (status == ZY100_FLASH_PREP_DONE)
    {
        DBG_DIRECT("[FLASH_PREPARE] poll busy=0 done=1 ok=1");
        s_v1_flash_ready_wait_start = true;
        s_v0_next_round = (s_v0_next_round == 0xFFFFFFFFU) ? 1U : (s_v0_next_round + 1U);
        return status;
    }

    DBG_DIRECT("[FLASH_PREPARE] poll busy=0 done=0 ok=0 status=%u",
               (uint32_t)status);
    s_v1_flash_ready_wait_start = false;
    s_v0_current_round = 0U;
    s_v0_hw_ready = false;
    return status;
#endif
}

bool imu_fifo_drain_test_prepare_flash(void)
{
    zy100_flash_prepare_status_t status;

    if (!imu_fifo_drain_test_prepare_flash_begin())
    {
        return false;
    }

    do
    {
        status = imu_fifo_drain_test_prepare_flash_poll();
        if (status == ZY100_FLASH_PREP_BUSY)
        {
            imu_bsp_delay_us(50000U);
        }
    } while (status == ZY100_FLASH_PREP_BUSY);

    return status == ZY100_FLASH_PREP_DONE;
}

bool imu_fifo_drain_test_prepare_erased_flash(void)
{
#if !ZY100_FINAL_EDGE_MODE_ENABLE
    uint32_t round;
    imu_status_t resume_status;
    bool prepared = false;
#endif

    if (s_v0_capture_active || s_v0_worker_busy || s_v0_stop_in_progress || s_v0_start_pending)
    {
        return false;
    }

    if (!imu_fifo_drain_test_task_init())
    {
        return false;
    }

#if ZY100_FINAL_EDGE_MODE_ENABLE
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    return imu_fifo_drain_test_prepare_flash();
#else
    s_v1_flash_ready_wait_start = true;
    return true;
#endif
#else

    round = s_v0_next_round;
    s_v0_imu_fifo_pending_count = 0U;
    s_v0_high_water_alarm_pending = false;
    s_v0_phase_a_rescue_pending = false;
    s_v0_irq_count = 0U;
    s_v0_capture_done = false;
    s_v0_stop_requested = false;
    s_v0_stop_in_progress = false;
    s_v0_start_pending = false;
    s_v0_capture_active = false;
    s_v0_worker_busy = true;
    s_v0_spi_fifo_busy = true;
    s_v0_stop_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
    s_v0_last_stop_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
    s_v0_current_round = round;
    s_v1_flash_ready_wait_start = false;
    s_v1_flash_prepare_busy_logged = false;

    resume_status = v0_imu_platform_resume_for_capture(round);
    if (resume_status == IMU_STATUS_OK)
    {
        prepared = external_flash_capture_prepare_erased(round);
    }
    else
    {
        DBG_DIRECT("[ERR][IMU] prepare_resume_failed status=%u", (uint32_t)resume_status);
    }

    s_v0_worker_busy = false;
    s_v0_spi_fifo_busy = false;

    if (!prepared)
    {
        DBG_DIRECT("[FLASH_PREPARE] erased round=%u ok=0", round);
        s_v0_current_round = 0U;
        s_v0_hw_ready = false;
        return false;
    }

    DBG_DIRECT("[FLASH_PREPARE] erased round=%u ok=1", round);
    s_v1_flash_ready_wait_start = true;
    s_v0_next_round = (round == 0xFFFFFFFFU) ? 1U : (round + 1U);
    return true;
#endif
}

#if ZY100_LEGACY_OFFLINE_ENABLE
bool imu_fifo_drain_test_prepare_offline_v2(void)
{
#if ZY100_LEGACY_OFFLINE_ENABLE
    uint32_t round;
    imu_status_t resume_status;

    if (s_v0_capture_active || s_v0_worker_busy || s_v0_stop_in_progress ||
        s_v0_start_pending)
    {
        return false;
    }
    if (!imu_fifo_drain_test_task_init())
    {
        return false;
    }
    round = s_v0_next_round;
    if (round == 0U)
    {
        round = 1U;
    }
    s_v0_imu_fifo_pending_count = 0U;
    s_v0_high_water_alarm_pending = false;
    s_v0_phase_a_rescue_pending = false;
    s_v0_irq_count = 0U;
    s_v0_capture_done = false;
    s_v0_last_packet_result = (uint32_t)V0_PACKET_PROCESS_OK;
    s_v0_offline_exit_logged = false;
    s_v0_stop_requested = false;
    s_v0_stop_in_progress = false;
    s_v0_start_pending = false;
    s_v0_capture_active = false;
    s_v0_worker_busy = true;
    s_v0_spi_fifo_busy = true;
    s_v0_stop_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
    s_v0_last_stop_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
    s_v0_current_round = round;
    s_v1_flash_ready_wait_start = false;
    resume_status = v0_imu_platform_resume_for_capture(round);
    s_v0_worker_busy = false;
    s_v0_spi_fifo_busy = false;
    if (resume_status != IMU_STATUS_OK)
    {
        s_v0_current_round = 0U;
        s_v0_hw_ready = false;
        return false;
    }
    s_v1_flash_ready_wait_start = true;
    s_v0_next_round = (round == 0xFFFFFFFFU) ? 1U : (round + 1U);
    return true;
#else
    return false;
#endif
}
#endif
#else
bool imu_fifo_drain_test_prepare_flash(void)
{
    return true;
}

bool imu_fifo_drain_test_prepare_flash_begin(void)
{
    return true;
}

bool imu_fifo_drain_test_prepare_flash_append_begin(
    const zy100_session_alloc_t *alloc)
{
    (void)alloc;
    return true;
}

zy100_flash_prepare_status_t imu_fifo_drain_test_prepare_flash_poll(void)
{
    return ZY100_FLASH_PREP_DONE;
}

bool imu_fifo_drain_test_prepare_erased_flash(void)
{
    return true;
}

#if ZY100_LEGACY_OFFLINE_ENABLE
bool imu_fifo_drain_test_prepare_offline_v2(void)
{
    return false;
}
#endif

#endif

#if V1_IMU_FLASH_CAPTURE_ENABLE
static bool imu_fifo_drain_test_arm_flash_no_erase_for_start(void)
{
    uint32_t round;

    if (s_v1_flash_ready_wait_start && (s_v0_current_round != 0U))
    {
        return true;
    }

    round = s_v0_next_round;
    if (round == 0U)
    {
        round = 1U;
    }

#if ZY100_FINAL_EDGE_MODE_ENABLE
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    zy100_final_edge_raw_store_reset_after_clear(round);
#if ZY100_FINAL_EDGE_PHASE7_EDGE_FLASH_ENABLE
    zy100_final_edge_record_store_reset_after_clear(round);
#endif
#endif
#endif

    s_v0_current_round = round;
    s_v1_flash_ready_wait_start = true;
    s_v0_next_round = (round == 0xFFFFFFFFU) ? 1U : (round + 1U);
    ZY100_DIAG_LOG("[FE_STORE_ARM] no_erase round=%u", round);
    return true;
}
#endif

bool imu_fifo_drain_test_start(void)
{
    uint32_t round;

    if (s_v0_capture_active || s_v0_worker_busy || s_v0_stop_in_progress || s_v0_start_pending)
    {
        return false;
    }

    if (!imu_fifo_drain_test_task_init())
    {
        return false;
    }

#if V1_IMU_FLASH_CAPTURE_ENABLE
#if ZY100_FINAL_EDGE_MODE_ENABLE
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    if (!s_v1_flash_ready_wait_start || (s_v0_current_round == 0U))
    {
        if (!imu_fifo_drain_test_arm_flash_no_erase_for_start())
        {
            return false;
        }
    }
    round = s_v0_current_round;
#else
    round = s_v0_next_round;
#endif
#else
    if (!s_v1_flash_ready_wait_start || (s_v0_current_round == 0U))
    {
        if (!imu_fifo_drain_test_arm_flash_no_erase_for_start())
        {
            return false;
        }
    }
    round = s_v0_current_round;
#endif
#else
    round = s_v0_next_round;
#endif
    s_v0_imu_fifo_pending_count = 0U;
    s_v0_high_water_alarm_pending = false;
    s_v0_phase_a_rescue_pending = false;
    s_v0_irq_count = 0U;
    s_v0_capture_done = false;
    s_v0_stop_requested = false;
    s_v0_stop_in_progress = false;
    s_v0_start_pending = false;
    s_v0_capture_active = false;
    s_v0_worker_busy = false;
    s_v0_spi_fifo_busy = false;
    s_v0_hw_ready = false;
    v0_adc_safe_window_close();
    s_v0_stop_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
    s_v0_last_stop_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
    s_v0_current_round = round;
    v0_online_pause_reset();
    s_v0_start_pending = true;

    if (xTaskNotify(s_v0_task_handle, V0_NOTIFY_START, eSetBits) != pdPASS)
    {
        s_v0_start_pending = false;
        s_v0_current_round = 0U;
#if V1_IMU_FLASH_CAPTURE_ENABLE
        s_v1_flash_ready_wait_start = false;
#endif
        DBG_DIRECT("[V0_IMU_FIFO][ERR] start notify failed");
        return false;
    }

#if V1_IMU_FLASH_CAPTURE_ENABLE && ZY100_FINAL_EDGE_MODE_ENABLE && \
    ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    s_v1_flash_ready_wait_start = false;
#endif
#if !V1_IMU_FLASH_CAPTURE_ENABLE || \
    (ZY100_FINAL_EDGE_MODE_ENABLE && !ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE)
    s_v0_next_round = (round == 0xFFFFFFFFU) ? 1U : (round + 1U);
#endif
    return true;
}

void imu_fifo_drain_test_request_stop(imu_fifo_drain_test_stop_reason_t reason)
{
    bool log_request;

    if (s_v0_start_pending && !s_v0_worker_busy && !s_v0_capture_active)
    {
        v0_adc_safe_window_close();
        s_v0_start_pending = false;
        s_v0_stop_requested = false;
        s_v0_stop_in_progress = false;
        s_v0_stop_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
        s_v0_current_round = 0U;
#if V1_IMU_FLASH_CAPTURE_ENABLE
        s_v1_flash_ready_wait_start = false;
#endif
        return;
    }

    if (s_v0_capture_active || s_v0_worker_busy || s_v0_start_pending)
    {
#if ZY100_FINAL_EDGE_MODE_ENABLE
        if (reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_USER_STOP)
        {
            v0_final_edge_request_stop(
                ZY100_FE_STOP_REASON_USER_SHORT_PRESS, "button");
            return;
        }
        if (reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL)
        {
            s_v0_suppress_next_stop_request_log = true;
            v0_final_edge_request_stop(ZY100_FE_STOP_REASON_ERROR, "button");
            return;
        }
        if (reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_BATTERY_LOW)
        {
            s_v0_suppress_next_stop_request_log = true;
            v0_final_edge_request_stop(ZY100_FE_STOP_REASON_BATTERY_LOW,
                                       "battery");
            return;
        }
#endif
        log_request = (!s_v0_stop_requested) || (s_v0_stop_reason != reason);
        v0_adc_safe_window_close();
        s_v0_stop_reason = reason;
        s_v0_stop_requested = true;
        if (s_v0_capture_active || s_v0_worker_busy)
        {
            s_v0_stop_in_progress = true;
        }
        if (log_request)
        {
#if V1_IMU_FLASH_CAPTURE_ENABLE
#if IMU_CAPTURE_PROGRESS_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE
            {
                DBG_DIRECT("[CAP] stop_req reason=%s", v0_stop_reason_name(reason));
            }
#endif
#else
            DBG_DIRECT("[V0_IMU_FIFO] stop requested reason=%s", v0_stop_reason_name(reason));
#endif
        }
    }
}

bool imu_fifo_drain_test_request_online_pause(void)
{
    return imu_fifo_drain_test_request_online_pause_with_reason(
               IMU_FIFO_DRAIN_TEST_STOP_REASON_PAUSED);
}

bool imu_fifo_drain_test_request_online_pause_with_reason(
    imu_fifo_drain_test_stop_reason_t reason)
{
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    if ((reason != IMU_FIFO_DRAIN_TEST_STOP_REASON_PAUSED) &&
        (reason != IMU_FIFO_DRAIN_TEST_STOP_REASON_BATTERY_LOW))
    {
        return false;
    }
    if (!zy100_online_stream_active() && !zy100_online_stream_end_wait_ack())
    {
        return false;
    }
    if (s_v0_start_pending && !s_v0_worker_busy && !s_v0_capture_active)
    {
        return false;
    }
    if (!(s_v0_capture_active || s_v0_worker_busy || s_v0_start_pending))
    {
        return false;
    }

#if ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
    if (zy100_online_raw_capture_active())
    {
        uint32_t lock_state = os_lock();

        /* A normal host PAUSE is graceful: freeze the UI producer, preserve
         * FIFO, drain every already-sampled packet, then commit the short RAW
         * tail. Transport/fatal paths use request_online_abort() instead. */
        s_v0_online_abort_requested = false;
        s_v0_online_pause_progress.abort_requested = false;
#if ZY100_FINAL_EDGE_MODE_ENABLE
        s_final_edge_rt_deferred_active = false;
        s_final_edge_rt_trigger_valid = false;
        s_final_edge_rt_gated_for_b = false;
#else
        s_v0_stop_reason = reason;
        s_v0_stop_requested = true;
        s_v0_stop_in_progress = true;
#endif
        os_unlock(lock_state);
#if ZY100_FINAL_EDGE_MODE_ENABLE
        v0_final_edge_b_critical_clear();
#endif
        v0_online_pause_set_phase(ZY100_ONLINE_PAUSE_PHASE_STOP_LIVE);
#if ZY100_FINAL_EDGE_MODE_ENABLE
        v0_final_edge_request_online_pause_stop(reason);
#endif
        return true;
    }
#endif
    v0_online_pause_set_phase(ZY100_ONLINE_PAUSE_PHASE_STOP_LIVE);
#if ZY100_FINAL_EDGE_MODE_ENABLE
    v0_final_edge_request_online_pause_stop(reason);
#else
    imu_fifo_drain_test_request_stop(reason);
#endif
    return true;
#else
    IMU_UNUSED(reason);
    return false;
#endif
}

bool imu_fifo_drain_test_request_online_abort(void)
{
    return imu_fifo_drain_test_request_online_abort_with_reason(
               IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL);
}

bool imu_fifo_drain_test_request_online_abort_with_reason(
    imu_fifo_drain_test_stop_reason_t reason)
{
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    uint32_t lock_state;

    if ((reason != IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL) &&
        (reason != IMU_FIFO_DRAIN_TEST_STOP_REASON_BATTERY_LOW))
    {
        return false;
    }

    if (!(s_v0_capture_active || s_v0_worker_busy || s_v0_stop_in_progress))
    {
        return false;
    }
    lock_state = os_lock();
    s_v0_online_abort_requested = true;
    s_v0_online_pause_progress.abort_requested = true;
    s_v0_stop_requested = true;
    s_v0_stop_in_progress = true;
#if ZY100_FINAL_EDGE_MODE_ENABLE
    if ((reason == IMU_FIFO_DRAIN_TEST_STOP_REASON_BATTERY_LOW) &&
        (s_fe_stop_reason == ZY100_FE_STOP_REASON_USER_SHORT_PRESS))
    {
        s_fe_stop_reason = ZY100_FE_STOP_REASON_BATTERY_LOW;
        s_v0_stop_reason = reason;
    }
    else if ((reason != IMU_FIFO_DRAIN_TEST_STOP_REASON_BATTERY_LOW) ||
             !s_fe_stop_seen ||
             (s_fe_stop_reason == ZY100_FE_STOP_REASON_BATTERY_LOW))
    {
        s_v0_stop_reason = reason;
    }
    s_final_edge_rt_deferred_active = false;
    s_final_edge_rt_trigger_valid = false;
    s_final_edge_rt_gated_for_b = false;
#else
    s_v0_stop_reason = reason;
#endif
    os_unlock(lock_state);
#if ZY100_FINAL_EDGE_MODE_ENABLE
    v0_final_edge_b_critical_clear();
#endif
    v0_online_pause_set_phase(ZY100_ONLINE_PAUSE_PHASE_ABORT);
    return true;
#else
    IMU_UNUSED(reason);
    return false;
#endif
}

bool imu_fifo_drain_test_is_active(void)
{
    return s_v0_capture_active;
}

bool imu_fifo_drain_test_is_busy(void)
{
    return s_v0_worker_busy || s_v0_spi_fifo_busy;
}

bool imu_fifo_drain_test_is_stop_in_progress(void)
{
    return s_v0_stop_in_progress;
}

bool imu_fifo_drain_test_is_start_pending(void)
{
    return s_v0_start_pending;
}

bool imu_fifo_drain_test_is_done(void)
{
    return s_v0_capture_done &&
           !s_v0_capture_active &&
           !s_v0_worker_busy &&
           !s_v0_stop_in_progress &&
           !s_v0_start_pending;
}

uint32_t imu_fifo_drain_test_completion_generation(void)
{
    return s_v0_completion_generation;
}

bool imu_fifo_drain_test_adc_safe_window(uint32_t now_ms)
{
    if (!s_v0_adc_safe_window_open)
    {
        return false;
    }
    if ((uint32_t)(now_ms - s_v0_adc_safe_window_ms) >
        V0_ADC_SAFE_WINDOW_MAX_AGE_MS)
    {
        v0_adc_safe_window_close();
        return false;
    }
    if (!v0_adc_safe_window_state_ok())
    {
        v0_adc_safe_window_close();
        return false;
    }
    v0_adc_safe_window_close();
    return true;
}

uint32_t imu_fifo_drain_test_peek_next_round(void)
{
    return s_v0_next_round;
}

uint32_t imu_fifo_drain_test_peek_start_round(void)
{
    uint32_t round;

#if V1_IMU_FLASH_CAPTURE_ENABLE && ZY100_FINAL_EDGE_MODE_ENABLE && \
    ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    if (s_v1_flash_ready_wait_start && (s_v0_current_round != 0U))
    {
        return s_v0_current_round;
    }
#endif
    round = s_v0_next_round;
    return (round == 0U) ? 1U : round;
}

uint32_t imu_fifo_drain_test_current_round(void)
{
    return s_v0_current_round;
}

imu_fifo_drain_test_stop_reason_t imu_fifo_drain_test_last_stop_reason(void)
{
    return s_v0_last_stop_reason;
}

zy100_fe_capture_state_t imu_fifo_drain_test_final_edge_capture_state(void)
{
#if ZY100_FINAL_EDGE_MODE_ENABLE
    return s_fe_capture_state;
#else
    return FE_CAPTURE_STATE_IDLE;
#endif
}

void imu_fifo_drain_test_get_online_rt_snapshot(
    zy100_online_rt_snapshot_t *out)
{
    uint8_t index;
    const zy100_spi_sched_drain_context_t *ctx;

    if (out == NULL)
    {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->fe_state = imu_fifo_drain_test_final_edge_capture_state();
    out->capture_active = s_v0_capture_active;
    out->start_pending = s_v0_start_pending;
    out->stop_in_progress = s_v0_stop_in_progress;
    out->spi_idle = spi_bus_current_owner() == SPI_OWNER_NONE;
#if ZY100_FINAL_EDGE_MODE_ENABLE
    out->b_pending = v0_final_edge_b_pending_critical();
    out->b_critical = v0_final_edge_b_critical_active();
    out->rt_due = s_final_edge_rt_deferred_active;
    out->replay_busy = v0_final_edge_replay_needs_service();
    out->raw_store_wip =
        zy100_final_edge_raw_store_has_pending_work();
    out->record_store_wip =
        !zy100_final_edge_record_store_is_idle();
#endif
    if (s_v0_fifo_drain_ctx_ring_count != 0U)
    {
        index = (uint8_t)((s_v0_fifo_drain_ctx_ring_next +
                          V0_FIFO_DRAIN_CTX_RING_SIZE - 1U) %
                         V0_FIFO_DRAIN_CTX_RING_SIZE);
        if (s_v0_fifo_drain_ctx_ring[index].valid)
        {
            ctx = &s_v0_fifo_drain_ctx_ring[index].ctx;
            if (ctx->cached_count_valid)
            {
                out->fifo_count = (ctx->cached_count > ctx->actual_len) ?
                                  (uint16_t)(ctx->cached_count -
                                             ctx->actual_len) : 0U;
                out->fifo_count_valid = true;
            }
        }
    }
    if (s_v0_imu_fifo_pending_count != 0U)
    {
        out->fifo_count = ZY100_FINAL_EDGE_LIVE_FIFO_DRAIN_THRESHOLD_BYTES;
        out->fifo_count_valid = true;
    }
}

void imu_fifo_drain_test_get_online_pause_progress(
    zy100_online_pause_progress_t *out)
{
    uint32_t lock_state;

    if (out == NULL)
    {
        return;
    }
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    lock_state = os_lock();
    *out = s_v0_online_pause_progress;
    out->abort_requested = s_v0_online_abort_requested;
    os_unlock(lock_state);
#else
    memset(out, 0, sizeof(*out));
    lock_state = 0U;
    (void)lock_state;
#endif
}

zy100_fe_stop_reason_t imu_fifo_drain_test_final_edge_stop_reason(void)
{
#if ZY100_FINAL_EDGE_MODE_ENABLE
    return s_fe_stop_reason;
#else
    return ZY100_FE_STOP_REASON_NONE;
#endif
}

bool imu_fifo_drain_test_final_edge_get_ois_activity_stats(
    zy100_fe_ois_activity_stats_t *out)
{
    if (out == NULL)
    {
        return false;
    }

    memset(out, 0, sizeof(*out));

#if ZY100_FINAL_EDGE_MODE_ENABLE
    out->rt_trigger_count = s_final_edge_phase1_stats.rt_trigger_count;
    out->last_rt_trigger_ms = s_final_edge_last_rt_trigger_ms;
#if ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
    out->b_scan_count = s_final_edge_b_scan_count;
    out->official_hit_count = s_final_edge_official_hit_count;
    out->official_nohit_count = s_final_edge_official_nohit_count;
    out->b_scan_fail_count = s_final_edge_b_scan_fail_count;
#endif
#if ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE
    {
        zy100_fe_raw_store_stats_t raw_stats;

        memset(&raw_stats, 0, sizeof(raw_stats));
        zy100_final_edge_raw_store_get_stats(&raw_stats);
        out->raw_saved_count = raw_stats.raw_saved;
    }
#endif
    return true;
#else
    return false;
#endif
}

bool imu_fifo_drain_test_final_edge_b_critical_active(void)
{
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
    return v0_final_edge_b_critical_active();
#else
    return false;
#endif
}

void imu_fifo_drain_test_online_diag_reset(void)
{
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    memset(&s_v0_online_abort_diag, 0, sizeof(s_v0_online_abort_diag));
    memset(&s_v0_online_resource, 0, sizeof(s_v0_online_resource));
    s_v0_online_abort_diag.active = true;
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_ONLINE_STREAM_ENABLE && \
    ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
    memset(&s_v0_online_edge_diag, 0, sizeof(s_v0_online_edge_diag));
    s_v0_online_edge_diag.active = true;
    zy100_final_edge_raw_store_online_diag_reset();
    zy100_final_edge_record_store_online_diag_reset();
#endif
}

bool imu_fifo_drain_test_online_diag_log_summary_step(void)
{
#if ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
    if (s_v0_online_resource.pending)
    {
        if (s_v0_online_resource.stage == 0U)
        {
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
            ZY100_LOG_EVENT("[RES_IMU] capture_ms=%lu packets=%lu fifo_hi=%lu fifo_cap=%u full=%lu lost=%lu",
                (unsigned long)s_v0_online_resource.capture_ms,
                (unsigned long)s_v0_online_resource.packets,
                (unsigned long)s_v0_online_resource.fifo_hi, (uint32_t)ZY100_FIFO_CAPACITY_BYTES,
                (unsigned long)s_v0_online_resource.full, (unsigned long)s_v0_online_resource.lost);
#else
            ZY100_LOG_EVENT("[CAP_IMU] packets=%lu full=%lu lost=%lu",
                (unsigned long)s_v0_online_resource.packets,
                (unsigned long)s_v0_online_resource.full,
                (unsigned long)s_v0_online_resource.lost);
#endif
            s_v0_online_resource.stage++;
        }
        else
        {
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
            ZY100_LOG_EVENT("[RES_WORKER] gap_ms=%lu stack_lifetime_free_B=%lu read_err=%lu bad_hdr=%lu",
                (unsigned long)s_v0_online_resource.gap_ms,
                (unsigned long)s_v0_online_resource.stack_free,
                (unsigned long)s_v0_online_resource.read_errors,
                (unsigned long)s_v0_online_resource.bad_headers);
#else
            ZY100_LOG_EVENT("[CAP_ERRORS] read_err=%lu bad_hdr=%lu",
                (unsigned long)s_v0_online_resource.read_errors,
                (unsigned long)s_v0_online_resource.bad_headers);
#endif
            s_v0_online_resource.pending = false;
        }
        return false;
    }
    if (zy100_online_stream_abort_cause_valid() &&
        s_v0_online_abort_diag.active)
    {
        switch (s_v0_online_abort_diag.summary_stage)
        {
        case 0U:
            DBG_DIRECT("[ONL_CAUSE_B] c=%u sr=%lu g=%lu ft=%lu/%lu h=%02x o=%u io=%u/%u fc=%u i=%u s=%u",
                       s_v0_online_abort_diag.captured ? 1U : 0U,
                       (unsigned long)s_v0_online_abort_diag.stop_reason,
                       (unsigned long)s_v0_online_abort_diag.completion_generation,
                       s_v0_online_abort_diag.fatal_error ? 1UL : 0UL,
                       (unsigned long)s_v0_online_abort_diag.fatal_status,
                       s_v0_online_abort_diag.bad_fifo_header,
                       s_v0_online_abort_diag.bad_fifo_offset,
                       s_v0_online_abort_diag.bad_fifo_req_len,
                       s_v0_online_abort_diag.bad_fifo_actual_len,
                       s_v0_online_abort_diag.bad_fifo_count,
                       s_v0_online_abort_diag.bad_fifo_int_level,
                       s_v0_online_abort_diag.bad_fifo_source);
            break;
        case 1U:
            DBG_DIRECT("[ONL_CAUSE_C] f=%lu/%lu/%lu/%lu r=%lu/%lu/%lu/%lu",
                       (unsigned long)s_v0_online_abort_diag.fifo_full_count,
                       (unsigned long)s_v0_online_abort_diag.fifo_lost_pkt_count,
                       (unsigned long)s_v0_online_abort_diag.bad_header_count,
                       (unsigned long)s_v0_online_abort_diag.ingress_overflow_count,
                       (unsigned long)s_v0_online_abort_diag.raw_error,
                       (unsigned long)s_v0_online_abort_diag.raw_accepted,
                       (unsigned long)s_v0_online_abort_diag.raw_committed,
                       (unsigned long)s_v0_online_abort_diag.raw_ring_high_water);
            break;
        default:
            s_v0_online_abort_diag.active = false;
            return true;
        }
        s_v0_online_abort_diag.summary_stage++;
        return false;
    }
#endif
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_ONLINE_STREAM_ENABLE && \
    ZY100_FINAL_EDGE_PHASE3_B_SCAN_ENABLE
    uint32_t b_frames_avg = 0U;
    uint32_t b_delta_min = 0U;
    uint32_t b_delta_max = 0U;
    uint32_t raw_pages = 0U;
    uint32_t record_pages = 0U;
    uint32_t raw_violations = 0U;
    uint32_t record_violations = 0U;

    if (!s_v0_online_edge_diag.active)
    {
        return true;
    }
    if (s_v0_online_edge_diag.awaiting_a2_first)
    {
        s_v0_online_edge_diag.a2_missing_count++;
        s_v0_online_edge_diag.awaiting_a2_first = false;
    }
    if (s_v0_online_edge_diag.b_count != 0U)
    {
        b_frames_avg = s_v0_online_edge_diag.b_frames_total /
                       s_v0_online_edge_diag.b_count;
    }
    if (s_v0_online_edge_diag.b_delta_seen)
    {
        b_delta_min = s_v0_online_edge_diag.b_delta_min;
        b_delta_max = s_v0_online_edge_diag.b_delta_max;
    }
    zy100_final_edge_raw_store_get_online_diag(&raw_pages, &raw_violations);
    zy100_final_edge_record_store_get_online_diag(&record_pages,
                                                   &record_violations);

    switch (s_v0_online_edge_diag.summary_stage)
    {
    case 0U:
        DBG_DIRECT("[ONL_D_QUAL] b=%lu fail=%lu frame=%lu/%lu dt=%lu stale=%lu mis=%lu",
               (unsigned long)s_v0_online_edge_diag.b_count,
               (unsigned long)s_v0_online_edge_diag.b_fail_count,
               (unsigned long)b_frames_avg,
               (unsigned long)s_v0_online_edge_diag.b_frames_max,
               (unsigned long)s_v0_online_edge_diag.b_dt_bad_total,
               (unsigned long)s_v0_online_edge_diag.b_stale_total,
               (unsigned long)s_v0_online_edge_diag.b_raw_mismatch_total);
        break;
    case 1U:
        DBG_DIRECT("[ONL_D_RT] exp=%lu spi=%lu raw=%lu replay=%lu",
               (unsigned long)s_final_edge_phase1_stats.rt_request_expired_count,
               (unsigned long)s_final_edge_phase1_stats.b_start_spi_retry_count,
               (unsigned long)s_final_edge_phase1_stats.b_start_raw_busy_count,
               (unsigned long)s_final_edge_phase1_stats.b_start_replay_busy_count);
        break;
    case 2U:
        DBG_DIRECT("[ONL_D_DT] mm=%lu/%lu ok=%lu bad=%lu z=%lu d=%lu",
               (unsigned long)b_delta_min,
               (unsigned long)b_delta_max,
               (unsigned long)s_v0_online_edge_diag.b_delta_156_157_total,
               (unsigned long)s_v0_online_edge_diag.b_delta_bad_total,
               (unsigned long)s_v0_online_edge_diag.b_delta_zero_total,
               (unsigned long)s_v0_online_edge_diag.b_delta_double_total);
        break;
    case 3U:
        DBG_DIRECT("[ONL_D_GAP] a1b=%lu/%lu/%lu b2a=%lu/%lu/%lu miss=%lu",
               (unsigned long)s_v0_online_edge_diag.gap_a1_b_last_us,
               (unsigned long)s_v0_online_edge_diag.gap_a1_b_min_us,
               (unsigned long)s_v0_online_edge_diag.gap_a1_b_max_us,
               (unsigned long)s_v0_online_edge_diag.gap_b_a2_last_us,
               (unsigned long)s_v0_online_edge_diag.gap_b_a2_min_us,
               (unsigned long)s_v0_online_edge_diag.gap_b_a2_max_us,
               (unsigned long)s_v0_online_edge_diag.a2_missing_count);
        break;
    case 4U:
        DBG_DIRECT("[ONL_D_REPLAY] beg=%lu done=%lu fail=%lu exp=%lu/%lu fed=%lu/%lu",
               (unsigned long)s_final_edge_phase1_stats.replay_begin_count,
               (unsigned long)s_final_edge_phase1_stats.replay_done_count,
               (unsigned long)(s_final_edge_phase1_stats.replay_fail_count +
                                s_final_edge_phase1_stats.replay_truehit_fail_count),
               (unsigned long)s_final_edge_phase1_stats.replay_samples_expected,
               (unsigned long)s_final_edge_phase1_stats.replay_truehit_samples_expected,
               (unsigned long)s_final_edge_phase1_stats.replay_samples_fed,
               (unsigned long)s_final_edge_phase1_stats.replay_truehit_samples_fed);
        break;
    case 5U:
        DBG_DIRECT("[ONL_D_ADDR] rawpg=%lu recpg=%lu vio=%lu/%lu fifo=%lu over=%lu",
               (unsigned long)raw_pages,
               (unsigned long)record_pages,
               (unsigned long)raw_violations,
               (unsigned long)record_violations,
               (unsigned long)s_final_edge_phase1_stats.fifo_max_count,
               (unsigned long)zy100_fifo_ingress_overflow_count());
        break;
    default:
        s_v0_online_edge_diag.active = false;
        return true;
    }
    s_v0_online_edge_diag.summary_stage++;
    return false;
#else
    return true;
#endif
}

void imu_fifo_drain_test_online_diag_log_summary(void)
{
    (void)imu_fifo_drain_test_online_diag_log_summary_step();
}

bool imu_fifo_drain_test_final_edge_export_ready(void)
{
#if ZY100_FINAL_EDGE_MODE_ENABLE
    return s_fe_export_ready &&
           (s_fe_capture_state == FE_CAPTURE_STATE_EXPORT_READY);
#else
    return false;
#endif
}

bool imu_fifo_drain_test_final_edge_empty_error_aborted(void)
{
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE
    return s_final_edge_empty_error_aborted;
#else
    return false;
#endif
}

void imu_fifo_drain_test_final_edge_set_exporting(void)
{
#if ZY100_FINAL_EDGE_MODE_ENABLE
    if (s_fe_export_ready)
    {
        s_fe_capture_state = FE_CAPTURE_STATE_EXPORTING;
    }
#endif
}

void imu_fifo_drain_test_final_edge_set_export_ready_retry(void)
{
#if ZY100_FINAL_EDGE_MODE_ENABLE
    s_fe_export_preserved = true;
    if (s_fe_final_drain_done)
    {
        s_fe_export_ready = true;
        s_fe_capture_state = FE_CAPTURE_STATE_EXPORT_READY;
    }
#endif
}

void imu_fifo_drain_test_final_edge_set_export_done(void)
{
#if ZY100_FINAL_EDGE_MODE_ENABLE
    s_fe_capture_state = FE_CAPTURE_STATE_EXPORT_DONE;
#endif
}

void imu_fifo_drain_test_final_edge_set_export_cleanup(void)
{
#if ZY100_FINAL_EDGE_MODE_ENABLE
    s_fe_capture_state = FE_CAPTURE_STATE_EXPORT_CLEANUP;
#endif
}

void imu_fifo_drain_test_final_edge_mark_export_cleared(void)
{
#if ZY100_FINAL_EDGE_MODE_ENABLE
    uint32_t round = s_v0_next_round;

    if (round == 0U)
    {
        round = 1U;
    }
    zy100_final_edge_raw_store_reset_after_clear(round);
    zy100_final_edge_record_store_reset_after_clear(round);
    (void)zy100_session_dir_reset_all_after_bulk_clear();
    zy100_final_edge_queue_reset();
    s_final_edge_active_session_valid = false;
    s_final_edge_empty_error_aborted = false;
    s_fe_export_ready = false;
    s_fe_export_cleared = true;
    s_fe_export_preserved = false;
    s_fe_stop_seen = false;
    s_fe_stop_reason = ZY100_FE_STOP_REASON_NONE;
    s_fe_final_drain_done = false;
    s_fe_final_drain_active = false;
    s_final_edge_no_new_data_mode = false;
    s_final_edge_live_timer_stopped_for_drain = false;
    s_final_edge_imu_live_stopped_for_drain = false;
    v0_adc_safe_window_close();
    s_fe_capture_state = FE_CAPTURE_STATE_IDLE;
    s_v0_stop_requested = false;
    s_v0_stop_in_progress = false;
    s_v0_stop_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
    s_v0_last_stop_reason = IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
    s_v0_capture_done = false;
    s_v0_current_round = round;
    s_v1_flash_ready_wait_start = true;
    s_v0_next_round = (round == 0xFFFFFFFFU) ? 1U : (round + 1U);
#endif
}

bool imu_fifo_drain_test_prepare_for_sleep(void)
{
    v0_adc_safe_window_close();
    if (s_v0_capture_active || s_v0_worker_busy || s_v0_stop_in_progress || s_v0_start_pending)
    {
        imu_fifo_drain_test_request_stop(IMU_FIFO_DRAIN_TEST_STOP_REASON_KEY_SLEEP);
        return false;
    }

    imu_bsp_set_sleep_prepare_cs_diag(true);
    v0_imu_platform_prepare_for_sleep();
#if V1_IMU_FLASH_CAPTURE_ENABLE
    s_v1_flash_ready_wait_start = false;
#endif
    return true;
}

#endif

#if !V0_IMU_FIFO_DRAIN_TEST

bool imu_fifo_drain_test_task_init(void)
{
    return false;
}

bool imu_fifo_drain_test_task_release_idle(void)
{
    return true;
}

bool imu_fifo_drain_test_prepare_flash(void)
{
    return false;
}

bool imu_fifo_drain_test_prepare_erased_flash(void)
{
    return false;
}

bool imu_fifo_drain_test_start(void)
{
    return false;
}

void imu_fifo_drain_test_request_stop(imu_fifo_drain_test_stop_reason_t reason)
{
    IMU_UNUSED(reason);
}

bool imu_fifo_drain_test_request_online_pause(void)
{
    return false;
}

bool imu_fifo_drain_test_request_online_pause_with_reason(
    imu_fifo_drain_test_stop_reason_t reason)
{
    IMU_UNUSED(reason);
    return false;
}

bool imu_fifo_drain_test_request_online_abort(void)
{
    return false;
}

bool imu_fifo_drain_test_request_online_abort_with_reason(
    imu_fifo_drain_test_stop_reason_t reason)
{
    IMU_UNUSED(reason);
    return false;
}

bool imu_fifo_drain_test_is_active(void)
{
    return false;
}

bool imu_fifo_drain_test_is_busy(void)
{
    return false;
}

bool imu_fifo_drain_test_is_stop_in_progress(void)
{
    return false;
}

bool imu_fifo_drain_test_is_start_pending(void)
{
    return false;
}

bool imu_fifo_drain_test_is_done(void)
{
    return false;
}

uint32_t imu_fifo_drain_test_completion_generation(void)
{
    return 0U;
}

uint32_t imu_fifo_drain_test_peek_next_round(void)
{
    return 0U;
}

uint32_t imu_fifo_drain_test_peek_start_round(void)
{
    return 1U;
}

uint32_t imu_fifo_drain_test_current_round(void)
{
    return 0U;
}

imu_fifo_drain_test_stop_reason_t imu_fifo_drain_test_last_stop_reason(void)
{
    return IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
}

void imu_fifo_drain_test_get_online_rt_snapshot(
    zy100_online_rt_snapshot_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
        out->fe_state = FE_CAPTURE_STATE_IDLE;
        out->spi_idle = true;
    }
}

void imu_fifo_drain_test_get_online_pause_progress(
    zy100_online_pause_progress_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

bool imu_fifo_drain_test_final_edge_get_ois_activity_stats(
    zy100_fe_ois_activity_stats_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
    return false;
}

bool imu_fifo_drain_test_final_edge_b_critical_active(void)
{
    return false;
}

void imu_fifo_drain_test_online_diag_reset(void)
{
}

void imu_fifo_drain_test_online_diag_log_summary(void)
{
}

bool imu_fifo_drain_test_prepare_for_sleep(void)
{
    return true;
}

#endif
