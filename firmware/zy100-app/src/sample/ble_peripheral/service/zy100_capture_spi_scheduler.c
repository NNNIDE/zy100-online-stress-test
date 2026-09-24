#include "zy100_capture_spi_scheduler.h"

#include <string.h>

#include "os_sync.h"
#include "trace.h"

#include "../app_flags.h"
#include "../common/imu_common.h"
#include "../driver/spi_bus_owner.h"
#include "imu_rt_marker.h"

#define ZY100_SPI_SCHED_LOG_PERIOD_MS 5000U
#define ZY100_SPI_SCHED_1S_LOG_PERIOD_MS 1000U
#define ZY100_SPI_SCHED_FIFO_PACKET_BYTES 16U
#define ZY100_SPI_SCHED_FIFO_DRAIN_MAX_BYTES 512U
#define ZY100_SPI_SCHED_FIFO_COUNT_VALID_MARGIN_BYTES 64U
#define ZY100_SPI_SCHED_FIXED_DRAIN_MAX_RETRY 2U
#define ZY100_SPI_SCHED_POST_DRAIN_RECHAIN_MAX 3U
#define ZY100_SPI_SCHED_FLASH_NEAR_FULL_LEVEL 2U

#define ZY100_SPI_SCHED_DRAIN_FAIL_NONE 0U
#define ZY100_SPI_SCHED_DRAIN_FAIL_CALLBACK 1U
#define ZY100_SPI_SCHED_DRAIN_FAIL_SHORT_COUNT 2U

#if (ZY100_FIFO_WATERMARK_BYTES != 512U)
#error "ZY100 fixed IRQ drain requires ZY100_FIFO_WATERMARK_BYTES == 512"
#endif

#if (ZY100_SPI_SCHED_FIFO_PACKET_BYTES != 16U)
#error "ZY100 fixed IRQ drain requires 16-byte FIFO packets"
#endif

#if ((ZY100_SPI_SCHED_FIFO_DRAIN_MAX_BYTES % ZY100_SPI_SCHED_FIFO_PACKET_BYTES) != 0U)
#error "ZY100 fixed IRQ drain length must align to FIFO packet size"
#endif

#if (ZY100_FLASH_PUMP_STEPS_PER_MS == 0U)
#error "ZY100_FLASH_PUMP_STEPS_PER_MS must be at least 1"
#endif

#if (!ZY100_RT_MARKER_ENABLE) && V1_IMU_FLASH_CAPTURE_ENABLE && (!ZY100_OIS_ENABLE)
#define ZY100_SPI_SCHED_FIFO_ONLY_POST_DRAIN_COUNT 1
#define ZY100_SPI_SCHED_FIFO_ONLY_DYNAMIC_FLASH_BUDGET 1
#else
#define ZY100_SPI_SCHED_FIFO_ONLY_POST_DRAIN_COUNT 0
#define ZY100_SPI_SCHED_FIFO_ONLY_DYNAMIC_FLASH_BUDGET 0
#endif

#define ZY100_SPI_SCHED_FLASH_BUDGET_QUEUE_LOW_STEPS 4U
#define ZY100_SPI_SCHED_FLASH_BUDGET_QUEUE_MID_STEPS 5U
#define ZY100_SPI_SCHED_FLASH_BUDGET_QUEUE_HIGH_STEPS 6U
#define ZY100_SPI_SCHED_FLASH_BUDGET_QUEUE_MID_LEVEL 2U
#define ZY100_SPI_SCHED_FLASH_BUDGET_QUEUE_HIGH_LEVEL 3U
#define ZY100_SPI_SCHED_FLASH_BUDGET_BACKLOG_HIGH_BLOCKS 3U

typedef enum
{
    ZY100_SPI_SCHED_FIFO_COUNT_REASON_NONE = 0U,
    ZY100_SPI_SCHED_FIFO_COUNT_REASON_POLL,
    ZY100_SPI_SCHED_FIFO_COUNT_REASON_PROBE,
    ZY100_SPI_SCHED_FIFO_COUNT_REASON_CACHE,
    ZY100_SPI_SCHED_FIFO_COUNT_REASON_EMERGENCY_50MS,
    ZY100_SPI_SCHED_FIFO_COUNT_REASON_DEBUG_100MS,
    ZY100_SPI_SCHED_FIFO_COUNT_REASON_POST_DRAIN,
    ZY100_SPI_SCHED_FIFO_COUNT_REASON_SERVICE_DEADLINE,
} zy100_spi_sched_fifo_count_reason_t;

typedef struct
{
    zy100_spi_sched_stats_t stats;
    uint32_t last_log_ms;
    uint32_t last_1s_log_ms;
    zy100_spi_sched_stats_t last_1s_stats;
    uint32_t session_start_ms;
    uint32_t fifo_count_ms;
    uint32_t fifo_irq_drain_latch_ms;
    uint32_t fifo_last_irq_ms;
    uint32_t fifo_last_drain_ms;
    uint32_t fifo_irq_pending_count;
    uint32_t fifo_drain_seq;
    uint32_t flash_backlog_blocks;
    uint16_t fifo_count;
    uint16_t fifo_irq_drain_len;
    uint8_t fifo_irq_drain_rt_defer_count;
    uint8_t fifo_fixed_drain_retry_count;
    uint8_t post_drain_rechain_streak;
    uint32_t flash_starved_streak;
    uint32_t flash_budget_ms;
    uint32_t flash_consecutive_steps;
    uint8_t flash_steps_this_ms;
    bool fifo_count_valid;
    bool phase_a_flash_count_fresh;
    bool req_rt_marker;
    bool req_fifo_count;
    bool req_fifo_int_status;
    bool req_fifo_drain;
    bool req_flash_pump;
    bool fifo_drain_latched;
    bool fifo_irq_drain_latched;
    bool fifo_irq_drain_suspended_for_diag;
    bool fifo_irq_drain_diag_count_pending;
    bool fifo_int_status_pending_after_drain;
    bool fifo_int_status_post_drain_resched;
    bool fifo_fast_path_enabled;
    bool fifo_first_irq_confirm_pending;
    bool first_irq_confirm_done;
    bool first_irq_confirm_ok;
    bool confirmed_drain_pending;
    bool flash_urgent_pending;
    bool flash_budget_exhausted;
    bool last_op_was_flash_pump;
    bool safe_flash_burst_budget_active;
    zy100_spi_sched_flash_result_t last_flash_pump_result;
    zy100_spi_sched_fifo_count_reason_t fifo_count_reason;
} zy100_spi_sched_state_t;

static zy100_spi_sched_state_t s_spi_sched;

#if ZY100_RUNTIME_STATS_LOG_ENABLE
static const char *zy100_spi_sched_mode_name(void)
{
#if !ZY100_RT_MARKER_ENABLE
    return "T3_FIFO_ONLY";
#elif ZY100_RT_MARKER_TIMER_ONLY_TEST
    return "T1";
#elif ZY100_RT_MARKER_FIFO_ONLY_TEST
    return "T2";
#else
    return "T3";
#endif
}

static uint32_t zy100_spi_sched_rt_enabled(void)
{
    return (uint32_t)(ZY100_RT_MARKER_ENABLE != 0);
}

static uint32_t zy100_spi_sched_tim6_enabled(void)
{
    return (uint32_t)((ZY100_RT_MARKER_ENABLE != 0) &&
                      (ZY100_RT_MARKER_TIMER_ENABLE != 0));
}
#endif

#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE || ZY100_RUNTIME_STATS_LOG_ENABLE
static uint32_t zy100_spi_sched_elapsed_ms(uint32_t now_ms, uint32_t then_ms)
{
    return now_ms - then_ms;
}
#endif

#if ZY100_RUNTIME_STATS_LOG_ENABLE
static uint32_t zy100_spi_sched_delta_u32(uint32_t now, uint32_t then)
{
    return now - then;
}

static uint32_t zy100_spi_sched_rate_per_s(uint32_t delta, uint32_t elapsed_ms)
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

#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
static uint32_t zy100_spi_sched_session_age_ms(uint32_t now_ms)
{
    return zy100_spi_sched_elapsed_ms(now_ms, s_spi_sched.session_start_ms);
}

static bool zy100_spi_sched_start_guard_active(uint32_t now_ms)
{
    return zy100_spi_sched_session_age_ms(now_ms) < ZY100_SPI_SCHED_START_GUARD_MS;
}

static bool zy100_spi_sched_startup_guard_needed(uint32_t now_ms)
{
    return zy100_spi_sched_start_guard_active(now_ms) ||
           !s_spi_sched.fifo_fast_path_enabled;
}
#endif

#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
static bool zy100_spi_sched_fifo_cache_expired(uint32_t now_ms)
{
    if (!s_spi_sched.fifo_count_valid)
    {
        return true;
    }

    return zy100_spi_sched_elapsed_ms(now_ms, s_spi_sched.fifo_count_ms) >
           ZY100_FIFO_COUNT_CACHE_MAX_AGE_MS;
}

static bool zy100_spi_sched_fifo_cache_usable(uint32_t now_ms)
{
    return s_spi_sched.fifo_count_valid && !zy100_spi_sched_fifo_cache_expired(now_ms);
}

static uint16_t zy100_spi_sched_align_down_packet(uint16_t value)
{
    return (uint16_t)(value & (uint16_t)~(ZY100_SPI_SCHED_FIFO_PACKET_BYTES - 1U));
}

static uint32_t zy100_spi_sched_fifo_count_valid_limit(void)
{
    return ZY100_FIFO_CAPACITY_BYTES +
           ZY100_SPI_SCHED_FIFO_COUNT_VALID_MARGIN_BYTES;
}
#endif

static bool zy100_spi_sched_any_drain_latched(void)
{
    return s_spi_sched.fifo_drain_latched || s_spi_sched.fifo_irq_drain_latched;
}

static bool zy100_spi_sched_any_drain_intent(void)
{
    return zy100_spi_sched_any_drain_latched() ||
           s_spi_sched.confirmed_drain_pending;
}

static bool zy100_spi_sched_flash_urgent_from_input(const zy100_spi_sched_input_t *in)
{
    if (in == NULL)
    {
        return false;
    }

    return in->flash_queue_near_full ||
           (in->flash_queue_level >= ZY100_SPI_SCHED_FLASH_NEAR_FULL_LEVEL);
}

static void zy100_spi_sched_inc_u32(uint32_t *value)
{
    if ((value != NULL) && (*value < 0xFFFFFFFFU))
    {
        (*value)++;
    }
}

uint32_t zy100_spi_sched_alloc_drain_seq(void)
{
    if (s_spi_sched.fifo_drain_seq < 0xFFFFFFFFU)
    {
        s_spi_sched.fifo_drain_seq++;
    }
    return s_spi_sched.fifo_drain_seq;
}

#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
static uint32_t zy100_spi_sched_elapsed_or_unknown(uint32_t now_ms,
                                                   uint32_t then_ms)
{
    return (then_ms == 0U) ? 0xFFFFFFFFU :
           zy100_spi_sched_elapsed_ms(now_ms, then_ms);
}
#endif

static void zy100_spi_sched_add_irq_pending_count(uint32_t pending_irq_count)
{
    if (pending_irq_count == 0U)
    {
        return;
    }

    if ((0xFFFFFFFFU - s_spi_sched.fifo_irq_pending_count) <
        pending_irq_count)
    {
        s_spi_sched.fifo_irq_pending_count = 0xFFFFFFFFU;
    }
    else
    {
        s_spi_sched.fifo_irq_pending_count += pending_irq_count;
    }
}

static void zy100_spi_sched_refresh_flash_budget(uint32_t now_ms)
{
    if (s_spi_sched.flash_budget_ms != now_ms)
    {
        s_spi_sched.flash_budget_ms = now_ms;
        s_spi_sched.flash_steps_this_ms = 0U;
        s_spi_sched.flash_budget_exhausted = false;
    }
}

static uint32_t zy100_spi_sched_flash_budget_limit(void)
{
    uint32_t limit;

#if ZY100_SPI_SCHED_FIFO_ONLY_DYNAMIC_FLASH_BUDGET
    if ((s_spi_sched.stats.flash_queue_level >=
         ZY100_SPI_SCHED_FLASH_BUDGET_QUEUE_HIGH_LEVEL) ||
        (s_spi_sched.flash_backlog_blocks >=
         ZY100_SPI_SCHED_FLASH_BUDGET_BACKLOG_HIGH_BLOCKS))
    {
        limit = ZY100_SPI_SCHED_FLASH_BUDGET_QUEUE_HIGH_STEPS;
    }
    else if (s_spi_sched.stats.flash_queue_level >=
             ZY100_SPI_SCHED_FLASH_BUDGET_QUEUE_MID_LEVEL)
    {
        limit = ZY100_SPI_SCHED_FLASH_BUDGET_QUEUE_MID_STEPS;
    }
    else
    {
        limit = ZY100_SPI_SCHED_FLASH_BUDGET_QUEUE_LOW_STEPS;
    }
#else
    limit = ZY100_FLASH_PUMP_STEPS_PER_MS;
#endif

    return limit;
}

static void zy100_spi_sched_update_flash_budget_exhausted(void)
{
    s_spi_sched.flash_budget_exhausted =
        ((uint32_t)s_spi_sched.flash_steps_this_ms >=
         zy100_spi_sched_flash_budget_limit());
}

static bool zy100_spi_sched_flash_budget_available(uint32_t now_ms)
{
    zy100_spi_sched_refresh_flash_budget(now_ms);
    return ((uint32_t)s_spi_sched.flash_steps_this_ms <
            zy100_spi_sched_flash_budget_limit());
}

static void zy100_spi_sched_note_flash_budget_used(uint32_t now_ms)
{
    zy100_spi_sched_refresh_flash_budget(now_ms);
    if (s_spi_sched.flash_steps_this_ms < 0xFFU)
    {
        s_spi_sched.flash_steps_this_ms++;
    }
    zy100_spi_sched_update_flash_budget_exhausted();
}

static void zy100_spi_sched_note_flash_starved(void)
{
    if (!s_spi_sched.flash_urgent_pending)
    {
        return;
    }

    if (s_spi_sched.stats.flash_starved_count < 0xFFFFFFFFU)
    {
        s_spi_sched.stats.flash_starved_count++;
    }
    if (s_spi_sched.flash_starved_streak < 0xFFFFFFFFU)
    {
        s_spi_sched.flash_starved_streak++;
    }
}

static void zy100_spi_sched_note_flash_progress(void)
{
    s_spi_sched.flash_starved_streak = 0U;
    if (s_spi_sched.flash_consecutive_steps < 0xFFFFFFFFU)
    {
        s_spi_sched.flash_consecutive_steps++;
    }
    if (s_spi_sched.flash_consecutive_steps >
        s_spi_sched.stats.max_consecutive_flash_steps)
    {
        s_spi_sched.stats.max_consecutive_flash_steps =
            s_spi_sched.flash_consecutive_steps;
    }
}

static void zy100_spi_sched_note_nonflash_progress(void)
{
    s_spi_sched.flash_consecutive_steps = 0U;
    s_spi_sched.last_op_was_flash_pump = false;
}

#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
static bool zy100_spi_sched_cached_fifo_emergency(bool fifo_cache_valid,
                                                  uint16_t fifo_count)
{
    return fifo_cache_valid &&
           ((uint32_t)fifo_count >= ZY100_FIFO_EMERGENCY_BYTES);
}

static bool zy100_spi_sched_service_guard_active(const zy100_spi_sched_input_t *in)
{
    return (in != NULL) &&
           !in->timer_only_test &&
           (in->fifo_service_due ||
            in->fifo_service_overdue ||
            in->fifo_service_emergency);
}

#if ZY100_RT_MARKER_ENABLE
static void zy100_spi_sched_note_flash_throttle_rt(void)
{
    zy100_spi_sched_inc_u32(&s_spi_sched.stats.flash_blocked_by_rt);
    zy100_spi_sched_inc_u32(&s_spi_sched.stats.flash_throttle_rt_due);
    zy100_spi_sched_note_flash_starved();
}
#endif
#endif

static void zy100_spi_sched_note_flash_throttle_fifo(void)
{
    zy100_spi_sched_inc_u32(&s_spi_sched.stats.flash_blocked_by_fifo);
    zy100_spi_sched_inc_u32(&s_spi_sched.stats.flash_throttle_fifo_pending);
    zy100_spi_sched_note_flash_starved();
}

static void zy100_spi_sched_note_flash_skip_fifo(
    const zy100_spi_sched_input_t *in)
{
    zy100_spi_sched_inc_u32(&s_spi_sched.stats.flash_fifo_skip_count);
    zy100_spi_sched_note_flash_throttle_fifo();
    if ((in != NULL) && (in->flash_skip_fifo != NULL))
    {
        in->flash_skip_fifo(in->ctx);
    }
}

static void zy100_spi_sched_note_flash_throttle_budget(void)
{
    zy100_spi_sched_inc_u32(&s_spi_sched.stats.flash_budget_stop_count);
    zy100_spi_sched_inc_u32(&s_spi_sched.stats.flash_throttle_budget);
    s_spi_sched.flash_budget_exhausted = true;
    zy100_spi_sched_note_flash_starved();
}

#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
static void zy100_spi_sched_note_service_guard_input(
    const zy100_spi_sched_input_t *in,
    bool fifo_service_active)
{
    if ((in == NULL) || !fifo_service_active)
    {
        return;
    }

    s_spi_sched.stats.ms_since_last_fifo_drain =
        in->ms_since_last_fifo_drain;
    if (in->fifo_service_due)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_service_due_count);
    }
    if (in->fifo_service_overdue)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_service_overdue_count);
    }
    if (in->fifo_service_emergency)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_service_emergency_count);
    }
}
#endif

static bool zy100_spi_sched_irq_drain_ready(void)
{
    return s_spi_sched.fifo_irq_drain_latched &&
           !s_spi_sched.fifo_irq_drain_suspended_for_diag;
}

static uint8_t zy100_spi_sched_fifo_count_reason_priority(
    zy100_spi_sched_fifo_count_reason_t reason)
{
    switch (reason)
    {
    case ZY100_SPI_SCHED_FIFO_COUNT_REASON_POST_DRAIN:
        return 7U;
    case ZY100_SPI_SCHED_FIFO_COUNT_REASON_SERVICE_DEADLINE:
        return 6U;
    case ZY100_SPI_SCHED_FIFO_COUNT_REASON_EMERGENCY_50MS:
        return 5U;
    case ZY100_SPI_SCHED_FIFO_COUNT_REASON_DEBUG_100MS:
        return 4U;
    case ZY100_SPI_SCHED_FIFO_COUNT_REASON_PROBE:
        return 3U;
    case ZY100_SPI_SCHED_FIFO_COUNT_REASON_CACHE:
        return 2U;
    case ZY100_SPI_SCHED_FIFO_COUNT_REASON_POLL:
        return 1U;
    case ZY100_SPI_SCHED_FIFO_COUNT_REASON_NONE:
    default:
        return 0U;
    }
}

#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
static void zy100_spi_sched_record_fifo_count_reason(
    zy100_spi_sched_fifo_count_reason_t reason)
{
    switch (reason)
    {
    case ZY100_SPI_SCHED_FIFO_COUNT_REASON_POLL:
        s_spi_sched.stats.fifo_count_reason_poll++;
        break;
    case ZY100_SPI_SCHED_FIFO_COUNT_REASON_PROBE:
        s_spi_sched.stats.fifo_count_reason_probe++;
        break;
    case ZY100_SPI_SCHED_FIFO_COUNT_REASON_CACHE:
        s_spi_sched.stats.fifo_count_reason_cache++;
        break;
    case ZY100_SPI_SCHED_FIFO_COUNT_REASON_EMERGENCY_50MS:
        s_spi_sched.stats.fifo_count_reason_probe++;
        s_spi_sched.stats.fifo_count_reason_emergency_50ms++;
        break;
    case ZY100_SPI_SCHED_FIFO_COUNT_REASON_DEBUG_100MS:
        s_spi_sched.stats.fifo_count_reason_probe++;
        s_spi_sched.stats.fifo_count_reason_debug_100ms++;
        break;
    case ZY100_SPI_SCHED_FIFO_COUNT_REASON_POST_DRAIN:
        s_spi_sched.stats.fifo_count_reason_post_drain++;
        break;
    case ZY100_SPI_SCHED_FIFO_COUNT_REASON_SERVICE_DEADLINE:
        s_spi_sched.stats.fifo_service_count_probe++;
        break;
    case ZY100_SPI_SCHED_FIFO_COUNT_REASON_NONE:
    default:
        s_spi_sched.stats.fifo_count_reason_cache++;
        break;
    }
}
#endif

static void zy100_spi_sched_request_fifo_count_internal(
    zy100_spi_sched_fifo_count_reason_t reason,
    bool allow_with_diag_suspended_drain);

#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
static void zy100_spi_sched_request_post_drain_diag(void)
{
    s_spi_sched.req_fifo_int_status = true;
    s_spi_sched.fifo_int_status_pending_after_drain = true;
    s_spi_sched.fifo_int_status_post_drain_resched = true;
    zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_post_drain_int_status_req);

#if ZY100_SPI_SCHED_FIFO_ONLY_POST_DRAIN_COUNT
    zy100_spi_sched_request_fifo_count_internal(
        ZY100_SPI_SCHED_FIFO_COUNT_REASON_POST_DRAIN,
        true);
    zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_post_drain_count_req);
#endif
}
#endif

static void zy100_spi_sched_request_fifo_count_internal(
    zy100_spi_sched_fifo_count_reason_t reason,
    bool allow_with_diag_suspended_drain)
{
    if (zy100_spi_sched_any_drain_intent() &&
        !(allow_with_diag_suspended_drain &&
          s_spi_sched.fifo_irq_drain_suspended_for_diag))
    {
        return;
    }

    if (!s_spi_sched.req_fifo_count ||
        (zy100_spi_sched_fifo_count_reason_priority(reason) >
         zy100_spi_sched_fifo_count_reason_priority(s_spi_sched.fifo_count_reason)))
    {
        s_spi_sched.fifo_count_reason = reason;
    }
    s_spi_sched.req_fifo_count = true;
}

#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
static void zy100_spi_sched_set_drain_latch(void)
{
    if (!s_spi_sched.fifo_drain_latched)
    {
        s_spi_sched.stats.fifo_count_to_drain_latch++;
    }
    s_spi_sched.fifo_drain_latched = true;
}
#endif

static void zy100_spi_sched_reset_post_drain_rechain(void)
{
    s_spi_sched.post_drain_rechain_streak = 0U;
}

#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
static void zy100_spi_sched_record_post_drain_rechain_streak(void)
{
    if (s_spi_sched.post_drain_rechain_streak >
        s_spi_sched.stats.consecutive_drain_count)
    {
        s_spi_sched.stats.consecutive_drain_count =
            s_spi_sched.post_drain_rechain_streak;
    }
}
#endif

static void zy100_spi_sched_clear_drain_latches(void)
{
    s_spi_sched.fifo_drain_latched = false;
    s_spi_sched.fifo_irq_drain_latched = false;
    s_spi_sched.fifo_irq_drain_len = 0U;
    s_spi_sched.fifo_irq_drain_latch_ms = 0U;
    s_spi_sched.fifo_irq_pending_count = 0U;
    s_spi_sched.fifo_irq_drain_rt_defer_count = 0U;
    s_spi_sched.fifo_fixed_drain_retry_count = 0U;
    s_spi_sched.fifo_irq_drain_suspended_for_diag = false;
    s_spi_sched.fifo_irq_drain_diag_count_pending = false;
    s_spi_sched.fifo_first_irq_confirm_pending = false;
    s_spi_sched.confirmed_drain_pending = false;
}

#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
static void zy100_spi_sched_request_first_irq_confirm(void)
{
    if (s_spi_sched.confirmed_drain_pending ||
        s_spi_sched.first_irq_confirm_ok ||
        s_spi_sched.fifo_first_irq_confirm_pending)
    {
        return;
    }

    s_spi_sched.fifo_first_irq_confirm_pending = true;
    s_spi_sched.req_fifo_count = true;
    s_spi_sched.fifo_count_reason = ZY100_SPI_SCHED_FIFO_COUNT_REASON_PROBE;
    s_spi_sched.stats.fifo_first_irq_confirm++;
}

static void zy100_spi_sched_clear_irq_drain_intent(void)
{
    s_spi_sched.req_fifo_drain = false;
    s_spi_sched.fifo_drain_latched = false;
    s_spi_sched.fifo_irq_drain_latched = false;
    s_spi_sched.fifo_irq_drain_len = 0U;
    s_spi_sched.fifo_irq_drain_latch_ms = 0U;
    s_spi_sched.fifo_irq_pending_count = 0U;
    s_spi_sched.fifo_irq_drain_rt_defer_count = 0U;
    s_spi_sched.fifo_fixed_drain_retry_count = 0U;
    s_spi_sched.fifo_irq_drain_suspended_for_diag = false;
    s_spi_sched.fifo_irq_drain_diag_count_pending = false;
    s_spi_sched.confirmed_drain_pending = false;
}

static void zy100_spi_sched_set_confirmed_drain_pending(uint32_t now_ms,
                                                        bool count_confirm)
{
    if (count_confirm)
    {
        s_spi_sched.stats.count_confirm_to_pending_drain++;
    }
    s_spi_sched.first_irq_confirm_done = true;
    s_spi_sched.first_irq_confirm_ok = true;
    s_spi_sched.confirmed_drain_pending = true;
    s_spi_sched.fifo_irq_drain_latch_ms = now_ms;
    s_spi_sched.fifo_irq_drain_len = ZY100_SPI_SCHED_FIFO_DRAIN_MAX_BYTES;
    s_spi_sched.fifo_fixed_drain_retry_count = 0U;
    s_spi_sched.req_fifo_drain = false;
    s_spi_sched.fifo_drain_latched = false;
    s_spi_sched.fifo_irq_drain_latched = false;
    s_spi_sched.fifo_irq_drain_suspended_for_diag = false;
    s_spi_sched.fifo_irq_drain_diag_count_pending = false;
}

static void zy100_spi_sched_update_first_irq_confirm_count(uint32_t now_ms,
                                                           uint16_t fifo_count)
{
    s_spi_sched.fifo_count = fifo_count;
    s_spi_sched.fifo_count_ms = now_ms;
    s_spi_sched.fifo_count_valid = true;
    if (zy100_spi_sched_startup_guard_needed(now_ms))
    {
        s_spi_sched.stats.count_probe_during_guard++;
    }

    if ((uint32_t)fifo_count >= ZY100_FIFO_WATERMARK_BYTES)
    {
        if (!s_spi_sched.first_irq_confirm_ok)
        {
            s_spi_sched.stats.fifo_first_irq_confirm_ok++;
        }
        zy100_spi_sched_set_confirmed_drain_pending(now_ms, true);
        return;
    }

    s_spi_sched.first_irq_confirm_done = false;
    s_spi_sched.first_irq_confirm_ok = false;
    zy100_spi_sched_clear_irq_drain_intent();
}

static void zy100_spi_sched_update_fifo_count(uint32_t now_ms, uint16_t fifo_count)
{
    s_spi_sched.fifo_count = fifo_count;
    s_spi_sched.fifo_count_ms = now_ms;
    s_spi_sched.fifo_count_valid = true;
    if (zy100_spi_sched_startup_guard_needed(now_ms))
    {
        s_spi_sched.stats.count_probe_during_guard++;
        if ((uint32_t)fifo_count >= ZY100_FIFO_WATERMARK_BYTES)
        {
            zy100_spi_sched_set_confirmed_drain_pending(now_ms, true);
        }
        else
        {
            zy100_spi_sched_reset_post_drain_rechain();
            zy100_spi_sched_clear_irq_drain_intent();
        }
        if ((uint32_t)fifo_count > s_spi_sched.stats.max_fifo_count)
        {
            s_spi_sched.stats.max_fifo_count = fifo_count;
        }
        return;
    }

    if ((uint32_t)fifo_count >= ZY100_FIFO_WATERMARK_BYTES)
    {
        zy100_spi_sched_set_drain_latch();
    }
    else
    {
        zy100_spi_sched_reset_post_drain_rechain();
        s_spi_sched.req_fifo_drain = false;
        s_spi_sched.fifo_drain_latched = false;
        if (s_spi_sched.fifo_irq_drain_suspended_for_diag)
        {
            zy100_spi_sched_clear_drain_latches();
        }
    }
    if ((uint32_t)fifo_count > s_spi_sched.stats.max_fifo_count)
    {
        s_spi_sched.stats.max_fifo_count = fifo_count;
    }
}

static void zy100_spi_sched_update_service_deadline_count(uint32_t now_ms,
                                                          uint16_t fifo_count)
{
    s_spi_sched.fifo_count = fifo_count;
    s_spi_sched.fifo_count_ms = now_ms;
    s_spi_sched.fifo_count_valid = true;

    if (zy100_spi_sched_startup_guard_needed(now_ms))
    {
        s_spi_sched.stats.count_probe_during_guard++;
        if ((uint32_t)fifo_count >= ZY100_FIFO_WATERMARK_BYTES)
        {
            zy100_spi_sched_set_confirmed_drain_pending(now_ms, true);
        }
        return;
    }

    if ((uint32_t)fifo_count >= ZY100_FIFO_WATERMARK_BYTES)
    {
        zy100_spi_sched_set_drain_latch();
        return;
    }

    zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_service_recovered_by_count);
    zy100_spi_sched_reset_post_drain_rechain();
    s_spi_sched.req_fifo_drain = false;
    s_spi_sched.fifo_drain_latched = false;
    if (s_spi_sched.fifo_irq_drain_suspended_for_diag)
    {
        zy100_spi_sched_clear_drain_latches();
    }
}

static void zy100_spi_sched_update_post_drain_count(uint32_t now_ms,
                                                    uint16_t fifo_count)
{
    uint32_t fifo_count_u32 = (uint32_t)fifo_count;

    s_spi_sched.fifo_count = fifo_count;
    s_spi_sched.fifo_count_ms = now_ms;
    s_spi_sched.fifo_count_valid = true;

    if (fifo_count_u32 < ZY100_FIFO_WATERMARK_BYTES)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.post_drain_count_lt_wm);
        zy100_spi_sched_reset_post_drain_rechain();
        s_spi_sched.req_fifo_drain = false;
        s_spi_sched.fifo_drain_latched = false;
        return;
    }

    zy100_spi_sched_inc_u32(&s_spi_sched.stats.post_drain_count_ge_wm);
    if (fifo_count_u32 >= ZY100_FIFO_EMERGENCY_BYTES)
    {
        zy100_spi_sched_inc_u32(
            &s_spi_sched.stats.post_drain_count_ge_emergency);
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.post_drain_count_high);
    }

    if (s_spi_sched.post_drain_rechain_streak >=
        ZY100_SPI_SCHED_POST_DRAIN_RECHAIN_MAX)
    {
        DBG_DIRECT("[FIFO_RECHAIN_WARN] count=%u streak=%u",
                   fifo_count,
                   s_spi_sched.post_drain_rechain_streak);
        s_spi_sched.fifo_count_valid = false;
        s_spi_sched.req_fifo_drain = false;
        s_spi_sched.fifo_drain_latched = false;
        s_spi_sched.fifo_irq_drain_latched = false;
        zy100_spi_sched_reset_post_drain_rechain();
        return;
    }

    s_spi_sched.post_drain_rechain_streak++;
    zy100_spi_sched_record_post_drain_rechain_streak();
    zy100_spi_sched_inc_u32(&s_spi_sched.stats.post_drain_rechain);
    zy100_spi_sched_set_drain_latch();
}
#endif

#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
static bool zy100_spi_sched_execute_fifo_count(const zy100_spi_sched_input_t *in)
{
    zy100_spi_sched_fifo_count_reason_t reason = s_spi_sched.fifo_count_reason;
    bool irq_diag_count = s_spi_sched.fifo_irq_drain_diag_count_pending;
    bool first_irq_confirm = s_spi_sched.fifo_first_irq_confirm_pending;
    uint16_t fifo_count = 0U;
    uint32_t fifo_count_u32;
    bool count_ok;

    if ((in == NULL) || (in->read_fifo_count == NULL))
    {
        return false;
    }

    zy100_spi_sched_note_nonflash_progress();
    s_spi_sched.req_fifo_count = false;
    s_spi_sched.fifo_count_reason = ZY100_SPI_SCHED_FIFO_COUNT_REASON_NONE;
    s_spi_sched.fifo_first_irq_confirm_pending = false;
    s_spi_sched.stats.fifo_count_op++;
    zy100_spi_sched_record_fifo_count_reason(reason);
    if (reason == ZY100_SPI_SCHED_FIFO_COUNT_REASON_POST_DRAIN)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.post_drain_count_op);
    }

    count_ok = in->read_fifo_count(in->ctx, &fifo_count);
    if (!count_ok)
    {
        if (first_irq_confirm)
        {
            s_spi_sched.first_irq_confirm_done = false;
            s_spi_sched.first_irq_confirm_ok = false;
            zy100_spi_sched_clear_irq_drain_intent();
        }
        if (!zy100_spi_sched_any_drain_latched())
        {
            s_spi_sched.fifo_count_valid = false;
        }
        if (irq_diag_count)
        {
            s_spi_sched.fifo_irq_drain_diag_count_pending = false;
            s_spi_sched.fifo_irq_drain_suspended_for_diag = false;
            s_spi_sched.fifo_fixed_drain_retry_count = 0U;
        }
        return true;
    }

    fifo_count_u32 = (uint32_t)fifo_count;
    if (fifo_count_u32 > s_spi_sched.stats.max_fifo_count)
    {
        s_spi_sched.stats.max_fifo_count = fifo_count;
    }
    if (fifo_count_u32 > ZY100_FIFO_CAPACITY_BYTES)
    {
        s_spi_sched.stats.fifo_count_over_capacity++;
    }
    if (fifo_count_u32 > zy100_spi_sched_fifo_count_valid_limit())
    {
        s_spi_sched.stats.fifo_count_rejected++;
        if (first_irq_confirm)
        {
            s_spi_sched.first_irq_confirm_done = false;
            s_spi_sched.first_irq_confirm_ok = false;
            zy100_spi_sched_clear_irq_drain_intent();
        }
        if (!zy100_spi_sched_any_drain_latched())
        {
            s_spi_sched.fifo_count_valid = false;
        }
        if (irq_diag_count)
        {
            s_spi_sched.fifo_irq_drain_diag_count_pending = false;
            s_spi_sched.fifo_irq_drain_suspended_for_diag = false;
            s_spi_sched.fifo_fixed_drain_retry_count = 0U;
        }
        return true;
    }

    if (first_irq_confirm)
    {
        zy100_spi_sched_update_first_irq_confirm_count(in->now_ms, fifo_count);
    }
    else if (reason == ZY100_SPI_SCHED_FIFO_COUNT_REASON_POST_DRAIN)
    {
        zy100_spi_sched_update_post_drain_count(in->now_ms, fifo_count);
    }
    else if (reason == ZY100_SPI_SCHED_FIFO_COUNT_REASON_SERVICE_DEADLINE)
    {
        zy100_spi_sched_update_service_deadline_count(in->now_ms, fifo_count);
    }
    else
    {
        zy100_spi_sched_update_fifo_count(in->now_ms, fifo_count);
    }
    if (irq_diag_count)
    {
        s_spi_sched.fifo_irq_drain_diag_count_pending = false;
        s_spi_sched.fifo_irq_drain_suspended_for_diag = false;
        s_spi_sched.fifo_fixed_drain_retry_count = 0U;
    }
    return true;
}
#endif

static bool zy100_spi_sched_execute_fifo_int_status(const zy100_spi_sched_input_t *in)
{
    bool after_drain = s_spi_sched.fifo_int_status_pending_after_drain;

    if ((in == NULL) || (in->read_fifo_int_status == NULL))
    {
        return false;
    }

    zy100_spi_sched_note_nonflash_progress();
    s_spi_sched.req_fifo_int_status = false;
    s_spi_sched.fifo_int_status_pending_after_drain = false;
    s_spi_sched.fifo_int_status_post_drain_resched = false;
    s_spi_sched.stats.fifo_int_status_op++;
    if (after_drain)
    {
        s_spi_sched.stats.fifo_int_status_after_drain++;
    }
    return in->read_fifo_int_status(in->ctx);
}

#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
static uint16_t zy100_spi_sched_fixed_drain_len(void)
{
    uint16_t drain_len = s_spi_sched.fifo_irq_drain_len;

    if (drain_len == 0U)
    {
        drain_len = ZY100_SPI_SCHED_FIFO_DRAIN_MAX_BYTES;
    }

    if (s_spi_sched.fifo_count_valid &&
        ((uint32_t)s_spi_sched.fifo_count >= ZY100_FIFO_WATERMARK_BYTES))
    {
        drain_len = zy100_spi_sched_align_down_packet(s_spi_sched.fifo_count);
        if (drain_len > ZY100_SPI_SCHED_FIFO_DRAIN_MAX_BYTES)
        {
            drain_len = ZY100_SPI_SCHED_FIFO_DRAIN_MAX_BYTES;
        }
    }

    return drain_len;
}

static bool zy100_spi_sched_fixed_drain_precheck(uint32_t now_ms)
{
    if (zy100_spi_sched_start_guard_active(now_ms))
    {
        return false;
    }

    if (!s_spi_sched.fifo_fast_path_enabled &&
        !s_spi_sched.confirmed_drain_pending)
    {
        zy100_spi_sched_request_first_irq_confirm();
        return false;
    }

    if (zy100_spi_sched_fifo_cache_usable(now_ms) &&
        ((uint32_t)s_spi_sched.fifo_count < ZY100_FIFO_WATERMARK_BYTES))
    {
        s_spi_sched.first_irq_confirm_done = false;
        s_spi_sched.first_irq_confirm_ok = false;
        zy100_spi_sched_clear_irq_drain_intent();
        return false;
    }

    return true;
}

static bool zy100_spi_sched_execute_fifo_drain(const zy100_spi_sched_input_t *in,
                                               bool emergency,
                                               bool fixed_len)
{
    uint16_t drain_len;
    uint16_t actual_read_len = 0U;
    uint8_t first_header = 0U;
    bool capped_by_flash_capacity = false;
    bool no_flash_capacity = false;
    bool guarded_startup = false;
    bool confirmed_at_entry = s_spi_sched.confirmed_drain_pending;
    zy100_spi_sched_drain_context_t drain_ctx;
    bool drained;

    if ((in == NULL) || (in->drain_fifo_512 == NULL))
    {
        return false;
    }

    zy100_spi_sched_note_nonflash_progress();
    if (fixed_len)
    {
        if (!zy100_spi_sched_fixed_drain_precheck(in->now_ms))
        {
            return false;
        }
        guarded_startup = !s_spi_sched.fifo_fast_path_enabled;
        drain_len = zy100_spi_sched_fixed_drain_len();
    }
    else
    {
        if (zy100_spi_sched_startup_guard_needed(in->now_ms))
        {
            s_spi_sched.stats.unguarded_drain_blocked_by_start_guard++;
            if (s_spi_sched.fifo_count_valid &&
                ((uint32_t)s_spi_sched.fifo_count >= ZY100_FIFO_WATERMARK_BYTES))
            {
                zy100_spi_sched_set_confirmed_drain_pending(in->now_ms, false);
            }
            return false;
        }

        if (!s_spi_sched.fifo_count_valid)
        {
            return false;
        }

        drain_len = zy100_spi_sched_align_down_packet(s_spi_sched.fifo_count);
        if (drain_len < ZY100_SPI_SCHED_FIFO_PACKET_BYTES)
        {
            s_spi_sched.stats.fifo_drain_fail_count++;
            s_spi_sched.stats.fifo_drain_fail_reason =
                ZY100_SPI_SCHED_DRAIN_FAIL_SHORT_COUNT;
            s_spi_sched.stats.fifo_drain_fail_last_len = drain_len;
            s_spi_sched.fifo_count_valid = false;
            zy100_spi_sched_clear_drain_latches();
            s_spi_sched.req_fifo_drain = false;
            return true;
        }
        if (drain_len > ZY100_SPI_SCHED_FIFO_DRAIN_MAX_BYTES)
        {
            drain_len = ZY100_SPI_SCHED_FIFO_DRAIN_MAX_BYTES;
        }
    }

    s_spi_sched.stats.fifo_drain_op++;
    if (fixed_len)
    {
        s_spi_sched.stats.fifo_fixed_drain_op++;
    }
    if (emergency)
    {
        s_spi_sched.stats.fifo_emergency_count++;
    }
    s_spi_sched.stats.fifo_drain_len_requested = drain_len;
    s_spi_sched.stats.fifo_drain_len_actual = 0U;

    memset(&drain_ctx, 0, sizeof(drain_ctx));
    drain_ctx.seq = zy100_spi_sched_alloc_drain_seq();
    if (emergency)
    {
        drain_ctx.src = ZY100_SPI_SCHED_DRAIN_SRC_EMERGENCY;
    }
    else if (fixed_len && confirmed_at_entry)
    {
        drain_ctx.src = ZY100_SPI_SCHED_DRAIN_SRC_CONFIRMED;
    }
    else if (fixed_len)
    {
        drain_ctx.src = ZY100_SPI_SCHED_DRAIN_SRC_IRQ_FIXED;
    }
    else
    {
        drain_ctx.src = ZY100_SPI_SCHED_DRAIN_SRC_COUNT_BASED;
    }
    drain_ctx.req_len = drain_len;
    drain_ctx.cached_count_valid = s_spi_sched.fifo_count_valid;
    drain_ctx.cached_count = s_spi_sched.fifo_count;
    drain_ctx.fifo_irq_pending = in->fifo_irq_pending;
    drain_ctx.irq_pending_count = s_spi_sched.fifo_irq_pending_count;
    drain_ctx.queue_level = s_spi_sched.stats.flash_queue_level;
    drain_ctx.fast_path = s_spi_sched.fifo_fast_path_enabled;
    drain_ctx.time_since_last_irq_ms =
        zy100_spi_sched_elapsed_or_unknown(in->now_ms,
                                           s_spi_sched.fifo_last_irq_ms);
    drain_ctx.time_since_last_drain_ms =
        zy100_spi_sched_elapsed_or_unknown(in->now_ms,
                                           s_spi_sched.fifo_last_drain_ms);

    drained = in->drain_fifo_512(in->ctx,
                                  &drain_ctx,
                                  drain_len,
                                  guarded_startup,
                                  &first_header,
                                  &actual_read_len,
                                  &capped_by_flash_capacity,
                                  &no_flash_capacity);
    s_spi_sched.fifo_last_drain_ms = in->now_ms;
    s_spi_sched.stats.fifo_drain_len_actual = actual_read_len;
    if (capped_by_flash_capacity)
    {
        s_spi_sched.stats.fifo_drain_capped_by_flash_capacity++;
    }
    if (no_flash_capacity)
    {
        s_spi_sched.stats.fifo_drain_no_flash_capacity++;
    }
    if (drained)
    {
        if (fixed_len)
        {
            if (s_spi_sched.stats.first_good_drain_ms == 0U)
            {
                s_spi_sched.stats.first_good_drain_ms =
                    zy100_spi_sched_session_age_ms(in->now_ms);
            }
            s_spi_sched.fifo_fast_path_enabled = true;
            s_spi_sched.confirmed_drain_pending = false;
            if (!zy100_spi_sched_start_guard_active(in->now_ms))
            {
                s_spi_sched.stats.fifo_fixed_drain_after_warmup++;
            }
        }
        s_spi_sched.fifo_count_valid = false;
        s_spi_sched.req_fifo_drain = false;
        zy100_spi_sched_clear_drain_latches();
        s_spi_sched.stats.fifo_drain_fail_reason = ZY100_SPI_SCHED_DRAIN_FAIL_NONE;
        zy100_spi_sched_request_post_drain_diag();
    }
    else
    {
        if (no_flash_capacity)
        {
            s_spi_sched.fifo_count_valid = false;
            s_spi_sched.req_fifo_drain = false;
            s_spi_sched.req_flash_pump = true;
            zy100_spi_sched_clear_drain_latches();
            s_spi_sched.stats.fifo_drain_fail_reason =
                ZY100_SPI_SCHED_DRAIN_FAIL_NONE;
            s_spi_sched.stats.fifo_drain_fail_last_len = drain_len;
            s_spi_sched.stats.fifo_drain_fail_first_header = first_header;
            return false;
        }
        s_spi_sched.stats.fifo_drain_fail_count++;
        s_spi_sched.stats.fifo_drain_fail_reason =
            ZY100_SPI_SCHED_DRAIN_FAIL_CALLBACK;
        s_spi_sched.stats.fifo_drain_fail_last_len = drain_len;
        s_spi_sched.stats.fifo_drain_fail_first_header = first_header;
        if (fixed_len && guarded_startup && (first_header == 0xFFU))
        {
            s_spi_sched.stats.early_drain_bad_header++;
            s_spi_sched.first_irq_confirm_done = false;
            s_spi_sched.first_irq_confirm_ok = false;
            s_spi_sched.fifo_count_valid = false;
            zy100_spi_sched_clear_irq_drain_intent();
            zy100_spi_sched_request_first_irq_confirm();
        }
        else if (fixed_len)
        {
            if (s_spi_sched.fifo_fixed_drain_retry_count < 0xFFU)
            {
                s_spi_sched.fifo_fixed_drain_retry_count++;
            }
            if (s_spi_sched.fifo_fixed_drain_retry_count >=
                ZY100_SPI_SCHED_FIXED_DRAIN_MAX_RETRY)
            {
                s_spi_sched.fifo_irq_drain_suspended_for_diag = true;
                s_spi_sched.fifo_irq_drain_diag_count_pending = true;
                s_spi_sched.req_fifo_count = true;
                s_spi_sched.fifo_count_reason =
                    ZY100_SPI_SCHED_FIFO_COUNT_REASON_PROBE;
            }
        }
    }
    return drained;
}

#if ZY100_RT_MARKER_ENABLE
static bool zy100_spi_sched_execute_rt_marker(const zy100_spi_sched_input_t *in)
{
    if ((in == NULL) || (in->rt_marker_read == NULL))
    {
        return false;
    }

    zy100_spi_sched_note_nonflash_progress();
    if (spi_bus_current_owner() != SPI_OWNER_NONE)
    {
        s_spi_sched.stats.rt_skip_spi++;
        if (in->rt_marker_skip_spi != NULL)
        {
            in->rt_marker_skip_spi(in->ctx, in->now_ms);
        }
        return true;
    }

    if (in->rt_marker_read(in->ctx, in->now_ms))
    {
        s_spi_sched.stats.rt_read_ok++;
    }
    return true;
}

static void zy100_spi_sched_skip_rt_marker_for_fifo(const zy100_spi_sched_input_t *in)
{
    s_spi_sched.stats.rt_skip_fifo++;
    if ((in != NULL) && (in->rt_marker_skip_fifo != NULL))
    {
        in->rt_marker_skip_fifo(in->ctx, in->now_ms);
    }
    s_spi_sched.req_rt_marker = false;
}
#endif
#endif

static void zy100_spi_sched_note_flash_pump_result_enum(
    zy100_spi_sched_flash_result_t result)
{
    switch (result)
    {
    case ZY100_SPI_SCHED_FLASH_RESULT_PAGE_ISSUED:
        zy100_spi_sched_note_flash_pump_step_result(true, false, false, false);
        break;
    case ZY100_SPI_SCHED_FLASH_RESULT_WIP_BUSY:
        zy100_spi_sched_note_flash_pump_step_result(false, false, true, false);
        break;
    case ZY100_SPI_SCHED_FLASH_RESULT_NO_WORK:
        zy100_spi_sched_note_flash_pump_step_result(false, false, false, true);
        break;
    case ZY100_SPI_SCHED_FLASH_RESULT_ERROR:
    case ZY100_SPI_SCHED_FLASH_RESULT_UNKNOWN:
    default:
        break;
    }
}

static bool zy100_spi_sched_flash_requested_from_input(
    const zy100_spi_sched_input_t *in)
{
    if (in == NULL)
    {
        return false;
    }
    return s_spi_sched.req_flash_pump ||
           in->flash_pump_requested ||
           in->flash_has_pending_work ||
           in->flash_raw_queue_busy;
}

#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
static bool zy100_spi_sched_fifo_count_safe_for_flash_margin(uint32_t now_ms)
{
    uint32_t age_ms;
#if (ZY100_PHASE_A_FLASH_FIFO_STALE_SAFE_ENABLE != 0U)
    uint64_t estimated;
#endif

    if (!s_spi_sched.fifo_count_valid)
    {
        return false;
    }
    if ((uint32_t)s_spi_sched.fifo_count >= ZY100_FIFO_WATERMARK_ALARM_BYTES)
    {
        return false;
    }

    age_ms = now_ms - s_spi_sched.fifo_count_ms;
    if (age_ms <= ZY100_FIFO_COUNT_CACHE_MAX_AGE_MS)
    {
        return true;
    }
#if (ZY100_PHASE_A_FLASH_FIFO_STALE_SAFE_ENABLE == 0U)
    zy100_spi_sched_inc_u32(&s_spi_sched.stats.stale_safe_deny_count);
    return false;
#else
    if (age_ms > ZY100_PHASE_A_FLASH_FIFO_STALE_MAX_MS)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.stale_safe_deny_count);
        return false;
    }
    estimated = (uint64_t)s_spi_sched.fifo_count +
                ((uint64_t)age_ms *
                 (uint64_t)ZY100_PHASE_A_FLASH_FIFO_GROWTH_BYTES_PER_MS) +
                (uint64_t)ZY100_PHASE_A_FLASH_FIFO_SAFE_MARGIN_BYTES;
    if (estimated < (uint64_t)ZY100_FIFO_WATERMARK_ALARM_BYTES)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.stale_safe_allow_count);
        return true;
    }
    zy100_spi_sched_inc_u32(&s_spi_sched.stats.stale_safe_deny_count);
    return false;
#endif
}

static bool zy100_spi_sched_phase_a_no_fifo_blockers(
    const zy100_spi_sched_input_t *in)
{
    if (in == NULL)
    {
        return false;
    }
    return !in->fifo_irq_pending &&
           !in->fifo_high_water_alarm_pending &&
           !in->fifo_rescue_pending &&
           !in->fifo_service_due &&
           !in->fifo_service_overdue &&
           !in->fifo_service_emergency &&
           !zy100_spi_sched_any_drain_intent();
}
#endif

static bool zy100_spi_sched_execute_flash_pump(const zy100_spi_sched_input_t *in)
{
    bool pump_ok;

    if ((in == NULL) || (in->flash_pump == NULL))
    {
        return false;
    }

    if (!zy100_spi_sched_flash_budget_available(in->now_ms))
    {
        s_spi_sched.req_flash_pump = true;
        zy100_spi_sched_note_flash_throttle_budget();
        return false;
    }

    s_spi_sched.req_flash_pump = false;
    zy100_spi_sched_note_flash_budget_used(in->now_ms);
    zy100_spi_sched_inc_u32(&s_spi_sched.stats.flash_pump_op);
    zy100_spi_sched_note_flash_progress();
    s_spi_sched.last_op_was_flash_pump = true;
    s_spi_sched.last_flash_pump_result = ZY100_SPI_SCHED_FLASH_RESULT_UNKNOWN;
    pump_ok = in->flash_pump(in->ctx);
    if (in->flash_pump_result != NULL)
    {
        s_spi_sched.last_flash_pump_result = in->flash_pump_result(in->ctx);
        zy100_spi_sched_note_flash_pump_result_enum(
            s_spi_sched.last_flash_pump_result);
    }
    if (pump_ok && (in->flash_has_pending_work || in->flash_raw_queue_busy))
    {
        s_spi_sched.req_flash_pump = true;
    }
    return pump_ok;
}

#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
static bool zy100_spi_sched_execute_fifo_poll_drain_with_result(
    const zy100_spi_sched_input_t *in,
    zy100_spi_sched_poll_drain_result_t *result_out)
{
    zy100_spi_sched_poll_drain_result_t result;
    bool ok;

    if ((in == NULL) || (in->fifo_poll_drain == NULL))
    {
        return false;
    }

    memset(&result, 0, sizeof(result));
    s_spi_sched.phase_a_flash_count_fresh = false;
    zy100_spi_sched_note_nonflash_progress();
    zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_count_op);
    ok = in->fifo_poll_drain(in->ctx,
                             in->fifo_high_water_alarm_pending,
                             &result);
    if (result.count_valid)
    {
        s_spi_sched.fifo_count = result.fifo_count;
        s_spi_sched.fifo_count_ms = in->now_ms;
        s_spi_sched.fifo_count_valid = true;
        if ((uint32_t)result.fifo_count > s_spi_sched.stats.max_fifo_count)
        {
            s_spi_sched.stats.max_fifo_count = result.fifo_count;
        }
    }
    if (result.drained)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_drain_op);
        s_spi_sched.stats.fifo_drain_len_requested = result.drain_len;
        s_spi_sched.stats.fifo_drain_len_actual = result.drain_len;
        s_spi_sched.fifo_last_drain_ms = in->now_ms;
        s_spi_sched.fifo_count_valid = false;
        if (result.high_pressure)
        {
            zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_fixed_drain_op);
        }
        if (result.emergency)
        {
            zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_emergency_count);
        }
    }
    else if (result.count_valid)
    {
        s_spi_sched.phase_a_flash_count_fresh = true;
    }
    if (result_out != NULL)
    {
        *result_out = result;
    }
    return ok;
}

static bool zy100_spi_sched_phase_a_can_flash_after_poll(
    const zy100_spi_sched_input_t *in,
    const zy100_spi_sched_poll_drain_result_t *result)
{
    if ((in == NULL) || (result == NULL))
    {
        return false;
    }
    if (!zy100_spi_sched_flash_requested_from_input(in))
    {
        return false;
    }
    if (!result->count_valid ||
        result->drained ||
        result->ingress_blocked ||
        result->high_pressure ||
        result->emergency)
    {
        return false;
    }
    if (!s_spi_sched.fifo_count_valid ||
        ((uint32_t)s_spi_sched.fifo_count >= ZY100_FIFO_WATERMARK_ALARM_BYTES))
    {
        return false;
    }
    if (!zy100_spi_sched_phase_a_no_fifo_blockers(in))
    {
        return false;
    }
    if (!zy100_spi_sched_flash_budget_available(in->now_ms))
    {
        zy100_spi_sched_note_flash_throttle_budget();
        return false;
    }
    return true;
}

static bool zy100_spi_sched_phase_a_safe_burst_allowed(
    const zy100_spi_sched_input_t *in)
{
    if (!zy100_spi_sched_phase_a_no_fifo_blockers(in))
    {
        return false;
    }
    return zy100_spi_sched_fifo_count_safe_for_flash_margin(in->now_ms);
}

static bool zy100_spi_sched_execute_phase_a_flash_burst(
    const zy100_spi_sched_input_t *in)
{
    bool old_safe_budget;
    bool progress = false;
    uint32_t step;

    if ((in == NULL) || !zy100_spi_sched_phase_a_safe_burst_allowed(in))
    {
        return false;
    }

    old_safe_budget = s_spi_sched.safe_flash_burst_budget_active;
    s_spi_sched.safe_flash_burst_budget_active = true;
    zy100_spi_sched_inc_u32(&s_spi_sched.stats.burst_enter_count);
    for (step = 0U; step < ZY100_PHASE_A_FLASH_BURST_STEPS_PER_PASS; step++)
    {
        if (!zy100_spi_sched_phase_a_safe_burst_allowed(in))
        {
            break;
        }
        if (!zy100_spi_sched_flash_budget_available(in->now_ms))
        {
            zy100_spi_sched_note_flash_throttle_budget();
            break;
        }
        if (!zy100_spi_sched_execute_flash_pump(in))
        {
            break;
        }
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.burst_step_count);
        progress = true;
        if (in->flash_pump_result == NULL)
        {
            break;
        }
        if (s_spi_sched.last_flash_pump_result ==
            ZY100_SPI_SCHED_FLASH_RESULT_PAGE_ISSUED)
        {
            zy100_spi_sched_inc_u32(
                &s_spi_sched.stats.burst_page_issued_count);
        }
        else if (s_spi_sched.last_flash_pump_result ==
                 ZY100_SPI_SCHED_FLASH_RESULT_WIP_BUSY)
        {
            zy100_spi_sched_inc_u32(
                &s_spi_sched.stats.burst_wip_busy_count);
        }
        if ((s_spi_sched.last_flash_pump_result ==
             ZY100_SPI_SCHED_FLASH_RESULT_PAGE_ISSUED) ||
            (s_spi_sched.last_flash_pump_result ==
             ZY100_SPI_SCHED_FLASH_RESULT_WIP_BUSY) ||
            (s_spi_sched.last_flash_pump_result ==
             ZY100_SPI_SCHED_FLASH_RESULT_ERROR) ||
            (s_spi_sched.last_flash_pump_result ==
             ZY100_SPI_SCHED_FLASH_RESULT_NO_WORK) ||
            (s_spi_sched.last_flash_pump_result ==
             ZY100_SPI_SCHED_FLASH_RESULT_UNKNOWN))
        {
            break;
        }
    }
    s_spi_sched.safe_flash_burst_budget_active = old_safe_budget;
    zy100_spi_sched_update_flash_budget_exhausted();
    return progress;
}

static bool zy100_spi_sched_execute_phase_a(const zy100_spi_sched_input_t *in)
{
    bool flash_requested;
    bool flash_urgent;
    bool fifo_service_pending;
    bool fifo_pressure_active;
    bool fifo_count_safe_for_flash;
    zy100_spi_sched_poll_drain_result_t poll_result;
    bool poll_ok;

    if ((in == NULL) || !in->capture_active)
    {
        return false;
    }

    zy100_spi_sched_refresh_flash_budget(in->now_ms);
    s_spi_sched.last_op_was_flash_pump = false;
    s_spi_sched.stats.ms_since_last_fifo_drain = in->ms_since_last_fifo_drain;
    s_spi_sched.stats.flash_queue_level = in->flash_queue_level;
    s_spi_sched.stats.flash_queue_near_full =
        zy100_spi_sched_flash_urgent_from_input(in) ? 1U : 0U;
    s_spi_sched.flash_backlog_blocks = in->flash_backlog_blocks;
    s_spi_sched.req_rt_marker = false;
    zy100_spi_sched_update_flash_budget_exhausted();

    if (in->fifo_service_due)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_service_due_count);
    }
    if (in->fifo_service_overdue)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_service_overdue_count);
    }
    if (in->fifo_service_emergency)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_service_emergency_count);
    }
    if (in->fifo_irq_pending ||
        in->fifo_high_water_alarm_pending ||
        in->fifo_rescue_pending ||
        in->fifo_poll_due ||
        in->fifo_service_due ||
        in->fifo_service_overdue ||
        in->fifo_service_emergency ||
        zy100_spi_sched_any_drain_intent())
    {
        s_spi_sched.phase_a_flash_count_fresh = false;
    }

    fifo_service_pending =
        in->fifo_high_water_alarm_pending ||
        in->fifo_poll_due ||
        in->fifo_rescue_pending ||
        in->fifo_service_due ||
        in->fifo_service_overdue ||
        in->fifo_service_emergency ||
        s_spi_sched.req_fifo_count;
    fifo_pressure_active =
        in->fifo_high_water_alarm_pending ||
        in->fifo_rescue_pending ||
        in->fifo_service_emergency ||
        (s_spi_sched.fifo_count_valid &&
         ((uint32_t)s_spi_sched.fifo_count >= ZY100_FIFO_WATERMARK_ALARM_BYTES));
    flash_requested = zy100_spi_sched_flash_requested_from_input(in);
    flash_urgent = zy100_spi_sched_flash_urgent_from_input(in);
    s_spi_sched.flash_urgent_pending = flash_urgent;

    if (fifo_service_pending)
    {
        s_spi_sched.req_fifo_count = false;
        memset(&poll_result, 0, sizeof(poll_result));
        poll_ok = zy100_spi_sched_execute_fifo_poll_drain_with_result(
            in,
            &poll_result);
        if (poll_ok &&
            zy100_spi_sched_phase_a_can_flash_after_poll(in, &poll_result))
        {
            zy100_spi_sched_inc_u32(&s_spi_sched.stats.after_poll_pump_count);
            return zy100_spi_sched_execute_flash_pump(in);
        }
        return poll_ok;
    }

    if (flash_requested)
    {
        fifo_count_safe_for_flash =
            zy100_spi_sched_fifo_count_safe_for_flash_margin(in->now_ms);
        if (!fifo_count_safe_for_flash ||
            fifo_pressure_active ||
            zy100_spi_sched_any_drain_intent())
        {
            s_spi_sched.req_fifo_count = true;
            s_spi_sched.phase_a_flash_count_fresh = false;
            zy100_spi_sched_note_flash_skip_fifo(in);
            return false;
        }
        return zy100_spi_sched_execute_phase_a_flash_burst(in);
    }

    if (s_spi_sched.req_fifo_int_status && !fifo_pressure_active)
    {
        return zy100_spi_sched_execute_fifo_int_status(in);
    }

    zy100_spi_sched_note_nonflash_progress();
    zy100_spi_sched_inc_u32(&s_spi_sched.stats.no_op_count);
    return false;
}
#endif

static void zy100_spi_sched_sync_rt_marker_stats(void)
{
#if ZY100_RT_MARKER_ENABLE
    imu_rt_marker_stats_t rt_stats;

    imu_rt_marker_get_stats(&rt_stats);
    s_spi_sched.stats.rt_due = rt_stats.due_total;
    s_spi_sched.stats.rt_read_ok = rt_stats.read_ok_total;
    s_spi_sched.stats.rt_late = rt_stats.skipped_late_slots_total;
    s_spi_sched.stats.rt_skip_fifo = rt_stats.skipped_high_pressure_total;
    s_spi_sched.stats.rt_skip_spi = rt_stats.skipped_spi_busy_total;
#else
    s_spi_sched.req_rt_marker = false;
    s_spi_sched.stats.rt_due = 0U;
    s_spi_sched.stats.rt_read_ok = 0U;
    s_spi_sched.stats.rt_late = 0U;
    s_spi_sched.stats.rt_skip_fifo = 0U;
    s_spi_sched.stats.rt_skip_spi = 0U;
    s_spi_sched.stats.flash_blocked_by_rt = 0U;
    s_spi_sched.stats.flash_throttle_rt_due = 0U;
    s_spi_sched.stats.fifo_irq_drain_rt_defer = 0U;
    s_spi_sched.stats.fifo_irq_drain_force_after_defer = 0U;
    s_spi_sched.stats.fifo_count_skipped_rt_due = 0U;
    s_spi_sched.stats.fifo_int_status_deferred_rt = 0U;
#endif
    s_spi_sched.stats.fifo_drain_latched =
        zy100_spi_sched_any_drain_intent() ? 1U : 0U;
    s_spi_sched.stats.confirmed_drain_pending =
        s_spi_sched.confirmed_drain_pending ? 1U : 0U;
}

void zy100_spi_sched_reset(uint32_t now_ms)
{
    memset(&s_spi_sched, 0, sizeof(s_spi_sched));
    s_spi_sched.last_log_ms = now_ms;
    s_spi_sched.last_1s_log_ms = now_ms;
    s_spi_sched.session_start_ms = now_ms;
}

void zy100_spi_sched_mark_session_start(uint32_t now_ms)
{
    uint32_t lock_state = os_lock();

    s_spi_sched.last_log_ms = now_ms;
    s_spi_sched.last_1s_log_ms = now_ms;
    s_spi_sched.last_1s_stats = s_spi_sched.stats;
    s_spi_sched.session_start_ms = now_ms;
    s_spi_sched.flash_budget_ms = now_ms;
    s_spi_sched.flash_steps_this_ms = 0U;
    s_spi_sched.flash_budget_exhausted = false;
    s_spi_sched.flash_consecutive_steps = 0U;
    s_spi_sched.last_op_was_flash_pump = false;
    s_spi_sched.safe_flash_burst_budget_active = false;
    s_spi_sched.last_flash_pump_result = ZY100_SPI_SCHED_FLASH_RESULT_UNKNOWN;
    zy100_spi_sched_reset_post_drain_rechain();
    os_unlock(lock_state);
}

void zy100_spi_sched_invalidate_fifo_count(void)
{
    uint32_t lock_state = os_lock();

    s_spi_sched.fifo_count_valid = false;
    s_spi_sched.phase_a_flash_count_fresh = false;
    os_unlock(lock_state);
}

void zy100_spi_sched_capture_stop(void)
{
    uint32_t lock_state = os_lock();

    s_spi_sched.fifo_count_valid = false;
    s_spi_sched.phase_a_flash_count_fresh = false;
    s_spi_sched.fifo_count = 0U;
    s_spi_sched.req_rt_marker = false;
    s_spi_sched.req_fifo_count = false;
    s_spi_sched.req_fifo_int_status = false;
    s_spi_sched.req_fifo_drain = false;
    s_spi_sched.req_flash_pump = false;
    s_spi_sched.flash_urgent_pending = false;
    s_spi_sched.flash_starved_streak = 0U;
    s_spi_sched.flash_consecutive_steps = 0U;
    s_spi_sched.flash_steps_this_ms = 0U;
    s_spi_sched.flash_budget_exhausted = false;
    s_spi_sched.last_op_was_flash_pump = false;
    s_spi_sched.safe_flash_burst_budget_active = false;
    s_spi_sched.last_flash_pump_result = ZY100_SPI_SCHED_FLASH_RESULT_UNKNOWN;
    s_spi_sched.fifo_count_reason = ZY100_SPI_SCHED_FIFO_COUNT_REASON_NONE;
    s_spi_sched.fifo_int_status_pending_after_drain = false;
    s_spi_sched.fifo_int_status_post_drain_resched = false;
    s_spi_sched.fifo_fast_path_enabled = false;
    s_spi_sched.first_irq_confirm_done = false;
    s_spi_sched.first_irq_confirm_ok = false;
    zy100_spi_sched_reset_post_drain_rechain();
    zy100_spi_sched_clear_drain_latches();
    os_unlock(lock_state);
}

bool zy100_spi_sched_request_rt_marker(void)
{
#if ZY100_RT_MARKER_ENABLE
    s_spi_sched.req_rt_marker = true;
    return true;
#else
    s_spi_sched.req_rt_marker = false;
    return false;
#endif
}

bool zy100_spi_sched_request_fifo_count(void)
{
    zy100_spi_sched_request_fifo_count_internal(
        ZY100_SPI_SCHED_FIFO_COUNT_REASON_CACHE,
        false);
    return true;
}

bool zy100_spi_sched_request_fifo_count_emergency_probe(void)
{
    zy100_spi_sched_request_fifo_count_internal(
        ZY100_SPI_SCHED_FIFO_COUNT_REASON_EMERGENCY_50MS,
        false);
    return true;
}

bool zy100_spi_sched_request_fifo_count_debug_probe(void)
{
    zy100_spi_sched_request_fifo_count_internal(
        ZY100_SPI_SCHED_FIFO_COUNT_REASON_DEBUG_100MS,
        false);
    return true;
}

bool zy100_spi_sched_request_fifo_int_status(void)
{
    s_spi_sched.req_fifo_int_status = true;
    return true;
}

bool zy100_spi_sched_request_fifo_drain(void)
{
#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
    return false;
#else
    s_spi_sched.req_fifo_drain = true;
    if (s_spi_sched.fifo_count_valid &&
        ((uint32_t)s_spi_sched.fifo_count >= ZY100_FIFO_WATERMARK_BYTES))
    {
        if (s_spi_sched.fifo_fast_path_enabled)
        {
            zy100_spi_sched_set_drain_latch();
        }
        else
        {
            zy100_spi_sched_set_confirmed_drain_pending(s_spi_sched.fifo_count_ms,
                                                        false);
        }
    }
    return true;
#endif
}

bool zy100_spi_sched_request_fifo_drain_from_irq(uint32_t now_ms,
                                                 uint32_t pending_irq_count)
{
#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
    IMU_UNUSED(now_ms);
    zy100_spi_sched_add_irq_pending_count(pending_irq_count);
    return true;
#else
    s_spi_sched.fifo_last_irq_ms = now_ms;
    zy100_spi_sched_add_irq_pending_count(pending_irq_count);

    if (s_spi_sched.req_fifo_int_status ||
        s_spi_sched.fifo_int_status_pending_after_drain)
    {
        s_spi_sched.stats.irq_repeat_before_status++;
    }

    if (zy100_spi_sched_start_guard_active(now_ms) ||
        !s_spi_sched.fifo_fast_path_enabled)
    {
        if (zy100_spi_sched_start_guard_active(now_ms))
        {
            s_spi_sched.stats.fifo_start_guard_skip++;
        }
        zy100_spi_sched_request_first_irq_confirm();
        return true;
    }

    if (!s_spi_sched.fifo_irq_drain_latched)
    {
        s_spi_sched.fifo_irq_drain_latched = true;
        s_spi_sched.fifo_irq_drain_len = ZY100_SPI_SCHED_FIFO_DRAIN_MAX_BYTES;
        s_spi_sched.fifo_irq_drain_latch_ms = now_ms;
        s_spi_sched.fifo_irq_drain_rt_defer_count = 0U;
        s_spi_sched.fifo_fixed_drain_retry_count = 0U;
        s_spi_sched.fifo_irq_drain_suspended_for_diag = false;
        s_spi_sched.fifo_irq_drain_diag_count_pending = false;
        s_spi_sched.stats.fifo_irq_to_drain_latch++;
    }
    return true;
#endif
}

bool zy100_spi_sched_request_flash_pump(void)
{
    s_spi_sched.req_flash_pump = true;
    return true;
}

bool zy100_spi_sched_has_pending_work(void)
{
    bool rt_pending = false;

#if ZY100_RT_MARKER_ENABLE
    rt_pending = s_spi_sched.req_rt_marker;
#else
    s_spi_sched.req_rt_marker = false;
#endif

    return rt_pending ||
           s_spi_sched.req_fifo_count ||
           s_spi_sched.req_fifo_int_status ||
           s_spi_sched.req_fifo_drain ||
           s_spi_sched.req_flash_pump ||
           s_spi_sched.fifo_drain_latched ||
           zy100_spi_sched_irq_drain_ready() ||
           s_spi_sched.confirmed_drain_pending ||
           s_spi_sched.fifo_int_status_post_drain_resched;
}

bool zy100_spi_sched_has_urgent_work(void)
{
    bool rt_pending = false;

#if ZY100_RT_MARKER_ENABLE
    rt_pending = s_spi_sched.req_rt_marker;
#else
    s_spi_sched.req_rt_marker = false;
#endif

    return rt_pending ||
           s_spi_sched.req_fifo_drain ||
           s_spi_sched.fifo_drain_latched ||
           zy100_spi_sched_irq_drain_ready() ||
           s_spi_sched.fifo_int_status_post_drain_resched ||
           (s_spi_sched.req_fifo_count &&
            (s_spi_sched.fifo_count_reason ==
             ZY100_SPI_SCHED_FIFO_COUNT_REASON_POST_DRAIN ||
             s_spi_sched.fifo_count_reason ==
             ZY100_SPI_SCHED_FIFO_COUNT_REASON_SERVICE_DEADLINE)) ||
           (s_spi_sched.req_flash_pump && s_spi_sched.flash_urgent_pending &&
            !s_spi_sched.flash_budget_exhausted) ||
           s_spi_sched.confirmed_drain_pending;
}

bool zy100_spi_sched_execute_once(const zy100_spi_sched_input_t *in)
{
#if ZY100_SPI_SCHEDULER_ENABLE
#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
    return zy100_spi_sched_execute_phase_a(in);
#else
    bool rt_due;
    bool flash_requested;
    bool fifo_cache_valid;
    bool fifo_cache_ok;
    bool fifo_emergency = false;
    bool fifo_normal = false;
    bool irq_drain_ready;
    bool confirmed_drain_ready;
    bool fixed_drain_ready;
    bool startup_guard_needed;
    bool flash_urgent;
    bool flash_auto_requested;
    bool fifo_service_active;
    uint16_t fifo_count = 0U;

    if ((in == NULL) || !in->capture_active)
    {
        return false;
    }

    zy100_spi_sched_refresh_flash_budget(in->now_ms);
    s_spi_sched.last_op_was_flash_pump = false;
    fifo_service_active = zy100_spi_sched_service_guard_active(in);
    s_spi_sched.stats.ms_since_last_fifo_drain =
        in->ms_since_last_fifo_drain;
    zy100_spi_sched_note_service_guard_input(in, fifo_service_active);

    flash_auto_requested = in->flash_has_pending_work || in->flash_raw_queue_busy;
    if (flash_auto_requested)
    {
        s_spi_sched.req_flash_pump = true;
    }
    flash_urgent = zy100_spi_sched_flash_urgent_from_input(in);
    s_spi_sched.flash_urgent_pending = flash_urgent;
    s_spi_sched.stats.flash_queue_level = in->flash_queue_level;
    s_spi_sched.stats.flash_queue_near_full = flash_urgent ? 1U : 0U;
    s_spi_sched.flash_backlog_blocks = in->flash_backlog_blocks;
    zy100_spi_sched_update_flash_budget_exhausted();

#if ZY100_RT_MARKER_ENABLE
    rt_due = s_spi_sched.req_rt_marker || in->rt_due_pending;
#else
    s_spi_sched.req_rt_marker = false;
    rt_due = false;
    IMU_UNUSED(rt_due);
#endif
    flash_requested = s_spi_sched.req_flash_pump ||
                      in->flash_pump_requested ||
                      flash_auto_requested;
    startup_guard_needed = zy100_spi_sched_startup_guard_needed(in->now_ms);

    if (in->timer_only_test)
    {
#if ZY100_RT_MARKER_ENABLE
        if (rt_due)
        {
            s_spi_sched.req_rt_marker = false;
            return zy100_spi_sched_execute_rt_marker(in);
        }
#endif
        zy100_spi_sched_note_nonflash_progress();
        s_spi_sched.stats.no_op_count++;
        return false;
    }

    if (s_spi_sched.fifo_count_valid &&
        ((uint32_t)s_spi_sched.fifo_count >= ZY100_FIFO_WATERMARK_BYTES))
    {
        if (startup_guard_needed)
        {
            zy100_spi_sched_set_confirmed_drain_pending(in->now_ms, false);
        }
        else
        {
            zy100_spi_sched_set_drain_latch();
        }
    }

    fifo_cache_valid = s_spi_sched.fifo_count_valid;
    fifo_cache_ok = zy100_spi_sched_fifo_cache_usable(in->now_ms);
    if (fifo_cache_valid)
    {
        fifo_count = s_spi_sched.fifo_count;
        fifo_emergency = ((uint32_t)fifo_count >= ZY100_FIFO_EMERGENCY_BYTES);
        fifo_normal = ((uint32_t)fifo_count >= ZY100_FIFO_WATERMARK_BYTES);
        if (fifo_normal)
        {
            if (startup_guard_needed)
            {
                zy100_spi_sched_set_confirmed_drain_pending(in->now_ms, false);
            }
            else
            {
                zy100_spi_sched_set_drain_latch();
            }
        }
        else if (s_spi_sched.req_fifo_drain && fifo_cache_ok)
        {
            s_spi_sched.req_fifo_drain = false;
        }
    }

    if (fifo_emergency && fifo_cache_valid)
    {
        if (startup_guard_needed)
        {
            if (flash_requested)
            {
                zy100_spi_sched_note_flash_throttle_fifo();
            }
            s_spi_sched.stats.unguarded_drain_blocked_by_start_guard++;
            zy100_spi_sched_set_confirmed_drain_pending(in->now_ms, false);
        }
        else
        {
            if (flash_requested)
            {
                zy100_spi_sched_note_flash_throttle_fifo();
            }
#if ZY100_RT_MARKER_ENABLE
            if (rt_due)
            {
                if (fifo_service_active)
                {
                    zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_service_rt_skip);
                }
                zy100_spi_sched_skip_rt_marker_for_fifo(in);
                rt_due = false;
            }
#endif
            s_spi_sched.req_fifo_count = false;
            return zy100_spi_sched_execute_fifo_drain(in, true, false);
        }
    }

    irq_drain_ready = zy100_spi_sched_irq_drain_ready();
    confirmed_drain_ready = s_spi_sched.confirmed_drain_pending &&
                             !zy100_spi_sched_start_guard_active(in->now_ms);
    fixed_drain_ready = irq_drain_ready || confirmed_drain_ready;

    if (fifo_service_active && flash_requested)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_service_flash_block);
    }

    if (fifo_service_active &&
        !zy100_spi_sched_any_drain_intent() &&
        !(s_spi_sched.fifo_int_status_post_drain_resched &&
          s_spi_sched.req_fifo_int_status) &&
        !(s_spi_sched.req_fifo_count &&
          (s_spi_sched.fifo_count_reason ==
           ZY100_SPI_SCHED_FIFO_COUNT_REASON_POST_DRAIN)) &&
        !s_spi_sched.fifo_first_irq_confirm_pending)
    {
        zy100_spi_sched_request_fifo_count_internal(
            ZY100_SPI_SCHED_FIFO_COUNT_REASON_SERVICE_DEADLINE,
            false);
    }
#if ZY100_RT_MARKER_ENABLE
    if (rt_due)
    {
        bool fifo_priority_pending =
            fixed_drain_ready ||
            fifo_service_active ||
            s_spi_sched.fifo_drain_latched ||
            s_spi_sched.confirmed_drain_pending ||
            (s_spi_sched.fifo_int_status_post_drain_resched &&
             s_spi_sched.req_fifo_int_status) ||
            (s_spi_sched.req_fifo_count &&
             (s_spi_sched.fifo_count_reason ==
              ZY100_SPI_SCHED_FIFO_COUNT_REASON_POST_DRAIN ||
              s_spi_sched.fifo_count_reason ==
              ZY100_SPI_SCHED_FIFO_COUNT_REASON_SERVICE_DEADLINE));

        if (fifo_priority_pending)
        {
            if (fifo_service_active)
            {
                zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_service_rt_skip);
            }
            if (s_spi_sched.req_fifo_count)
            {
                s_spi_sched.stats.fifo_count_skipped_rt_due++;
            }
            if (s_spi_sched.req_fifo_int_status)
            {
                s_spi_sched.stats.fifo_int_status_deferred_rt++;
            }
            zy100_spi_sched_skip_rt_marker_for_fifo(in);
            rt_due = false;
        }
        else
        {
            if (s_spi_sched.req_fifo_count)
            {
                s_spi_sched.stats.fifo_count_skipped_rt_due++;
            }
            if (s_spi_sched.req_fifo_int_status)
            {
                s_spi_sched.stats.fifo_int_status_deferred_rt++;
            }
            if (flash_requested)
            {
                zy100_spi_sched_note_flash_throttle_rt();
            }
            s_spi_sched.req_rt_marker = false;
            return zy100_spi_sched_execute_rt_marker(in);
        }
    }
#endif

    if (fixed_drain_ready)
    {
        if (flash_requested)
        {
            zy100_spi_sched_note_flash_throttle_fifo();
        }
        return zy100_spi_sched_execute_fifo_drain(in, false, true);
    }

    if (s_spi_sched.confirmed_drain_pending)
    {
        if (flash_requested)
        {
            zy100_spi_sched_note_flash_throttle_fifo();
        }
        return false;
    }

    if (s_spi_sched.req_fifo_count &&
        (s_spi_sched.fifo_count_reason ==
         ZY100_SPI_SCHED_FIFO_COUNT_REASON_POST_DRAIN))
    {
        return zy100_spi_sched_execute_fifo_count(in);
    }

    if (s_spi_sched.fifo_first_irq_confirm_pending && s_spi_sched.req_fifo_count)
    {
        return zy100_spi_sched_execute_fifo_count(in);
    }

    if (s_spi_sched.req_fifo_count &&
        (s_spi_sched.fifo_count_reason ==
         ZY100_SPI_SCHED_FIFO_COUNT_REASON_SERVICE_DEADLINE))
    {
        return zy100_spi_sched_execute_fifo_count(in);
    }

    if (s_spi_sched.fifo_drain_latched && fifo_cache_valid)
    {
        if (flash_requested)
        {
            zy100_spi_sched_note_flash_throttle_fifo();
        }
        if (startup_guard_needed)
        {
            s_spi_sched.stats.unguarded_drain_blocked_by_start_guard++;
            zy100_spi_sched_set_confirmed_drain_pending(in->now_ms, false);
            return false;
        }
        s_spi_sched.req_fifo_count = false;
        return zy100_spi_sched_execute_fifo_drain(in, false, false);
    }

    if (s_spi_sched.fifo_drain_latched && !fifo_cache_valid)
    {
        s_spi_sched.req_fifo_count = true;
        s_spi_sched.fifo_count_reason = ZY100_SPI_SCHED_FIFO_COUNT_REASON_CACHE;
    }

    if (s_spi_sched.fifo_int_status_post_drain_resched &&
        s_spi_sched.req_fifo_int_status)
    {
        return zy100_spi_sched_execute_fifo_int_status(in);
    }

    if (flash_requested && flash_urgent)
    {
#if ZY100_OIS_ENABLE
        zy100_spi_sched_note_flash_starved();
        return false;
#else
        bool fifo_blocks_flash = fifo_service_active ||
                                 zy100_spi_sched_any_drain_intent() ||
                                 in->fifo_irq_pending ||
                                 zy100_spi_sched_cached_fifo_emergency(fifo_cache_valid,
                                                                       fifo_count);

        if (fifo_blocks_flash)
        {
            zy100_spi_sched_note_flash_throttle_fifo();
            return false;
        }
        if (in->timer_only_test || in->fifo_only_test)
        {
            zy100_spi_sched_note_flash_starved();
            return false;
        }
        return zy100_spi_sched_execute_flash_pump(in);
#endif
    }

    if (s_spi_sched.req_fifo_int_status)
    {
        return zy100_spi_sched_execute_fifo_int_status(in);
    }

    if (s_spi_sched.req_fifo_count)
    {
        return zy100_spi_sched_execute_fifo_count(in);
    }

    if (flash_requested)
    {
        bool fifo_blocks_flash = fifo_service_active ||
                                 zy100_spi_sched_any_drain_intent() ||
                                 in->fifo_irq_pending;

#if ZY100_RT_MARKER_ENABLE
        if (rt_due)
        {
            zy100_spi_sched_note_flash_throttle_rt();
            return false;
        }
#endif
#if ZY100_OIS_ENABLE
        zy100_spi_sched_note_flash_starved();
        return false;
#endif
        if (zy100_spi_sched_cached_fifo_emergency(fifo_cache_valid, fifo_count))
        {
            fifo_blocks_flash = true;
        }
        if (fifo_blocks_flash)
        {
            zy100_spi_sched_note_flash_throttle_fifo();
            return false;
        }
        if (in->timer_only_test || in->fifo_only_test)
        {
            zy100_spi_sched_note_flash_starved();
            return false;
        }
        return zy100_spi_sched_execute_flash_pump(in);
    }

    zy100_spi_sched_note_nonflash_progress();
    s_spi_sched.stats.no_op_count++;
    return false;
#endif
#else
    IMU_UNUSED(in);
    return false;
#endif
}

bool zy100_spi_sched_last_op_was_flash_pump(void)
{
    return s_spi_sched.last_op_was_flash_pump;
}

void zy100_spi_sched_note_flash_pump_step_result(bool page_issued,
                                                 bool block_done,
                                                 bool wip_busy,
                                                 bool no_work)
{
    if (page_issued)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.flash_page_issued);
    }
    if (block_done)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.flash_block_done);
    }
    if (wip_busy)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.flash_wip_busy);
    }
    if (no_work)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.flash_no_work);
    }
}

void zy100_spi_sched_note_final_flash_pump_step(bool wip_busy)
{
    zy100_spi_sched_inc_u32(&s_spi_sched.stats.final_steps_count);
    if (wip_busy)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.final_wip_busy_count);
    }
}

void zy100_spi_sched_note_fifo_emergency_drain_result(bool full_len,
                                                      bool staged,
                                                      bool dropped,
                                                      bool saved,
                                                      bool capacity_bypass)
{
    if (full_len)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_emergency_drain_full_len);
    }
    if (staged)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_emergency_drain_staged);
    }
    if (dropped)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_emergency_drop);
    }
    if (saved)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_emergency_saved);
    }
    if (capacity_bypass)
    {
        zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_emergency_capacity_bypass);
    }
}

void zy100_spi_sched_note_fifo_drain_cross_block_append(void)
{
    zy100_spi_sched_inc_u32(&s_spi_sched.stats.fifo_drain_cross_block_append);
}

void zy100_spi_sched_get_stats(zy100_spi_sched_stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    zy100_spi_sched_sync_rt_marker_stats();
    *out = s_spi_sched.stats;
}

void zy100_spi_sched_log_1s_if_due(uint32_t now_ms)
{
#if !ZY100_RUNTIME_STATS_LOG_ENABLE
    IMU_UNUSED(now_ms);
#else
    uint32_t elapsed_ms = zy100_spi_sched_elapsed_ms(now_ms,
                                                     s_spi_sched.last_1s_log_ms);
    zy100_spi_sched_stats_t now_stats;
    zy100_spi_sched_stats_t *last = &s_spi_sched.last_1s_stats;
    uint32_t irq_latch_per_s;
    uint32_t drain_per_s;
    uint32_t no_cap_per_s;
    uint32_t cap_per_s;
    uint32_t cross_per_s;
    uint32_t pump_per_s;
    uint32_t page_per_s;
    uint32_t block_per_s;
    uint32_t th_fifo_per_s;
    uint32_t th_budget_per_s;
    uint32_t wip_busy_per_s;
    uint32_t no_work_per_s;
    uint32_t emg_full_delta;
    uint32_t emg_staged_delta;
    uint32_t emg_drop_delta;
    uint32_t emg_saved_delta;
    uint32_t emg_bypass_delta;

    if (elapsed_ms < ZY100_SPI_SCHED_1S_LOG_PERIOD_MS)
    {
        return;
    }

    zy100_spi_sched_sync_rt_marker_stats();
    now_stats = s_spi_sched.stats;

    irq_latch_per_s = zy100_spi_sched_rate_per_s(
        zy100_spi_sched_delta_u32(now_stats.fifo_irq_to_drain_latch,
                                  last->fifo_irq_to_drain_latch),
        elapsed_ms);
    drain_per_s = zy100_spi_sched_rate_per_s(
        zy100_spi_sched_delta_u32(now_stats.fifo_drain_op,
                                  last->fifo_drain_op),
        elapsed_ms);
    no_cap_per_s = zy100_spi_sched_rate_per_s(
        zy100_spi_sched_delta_u32(now_stats.fifo_drain_no_flash_capacity,
                                  last->fifo_drain_no_flash_capacity),
        elapsed_ms);
    cap_per_s = zy100_spi_sched_rate_per_s(
        zy100_spi_sched_delta_u32(now_stats.fifo_drain_capped_by_flash_capacity,
                                  last->fifo_drain_capped_by_flash_capacity),
        elapsed_ms);
    cross_per_s = zy100_spi_sched_rate_per_s(
        zy100_spi_sched_delta_u32(now_stats.fifo_drain_cross_block_append,
                                  last->fifo_drain_cross_block_append),
        elapsed_ms);
    pump_per_s = zy100_spi_sched_rate_per_s(
        zy100_spi_sched_delta_u32(now_stats.flash_pump_op,
                                  last->flash_pump_op),
        elapsed_ms);
    page_per_s = zy100_spi_sched_rate_per_s(
        zy100_spi_sched_delta_u32(now_stats.flash_page_issued,
                                  last->flash_page_issued),
        elapsed_ms);
    block_per_s = zy100_spi_sched_rate_per_s(
        zy100_spi_sched_delta_u32(now_stats.flash_block_done,
                                  last->flash_block_done),
        elapsed_ms);
    th_fifo_per_s = zy100_spi_sched_rate_per_s(
        zy100_spi_sched_delta_u32(now_stats.flash_throttle_fifo_pending,
                                  last->flash_throttle_fifo_pending),
        elapsed_ms);
    th_budget_per_s = zy100_spi_sched_rate_per_s(
        zy100_spi_sched_delta_u32(now_stats.flash_throttle_budget,
                                  last->flash_throttle_budget),
        elapsed_ms);
    wip_busy_per_s = zy100_spi_sched_rate_per_s(
        zy100_spi_sched_delta_u32(now_stats.flash_wip_busy,
                                  last->flash_wip_busy),
        elapsed_ms);
    no_work_per_s = zy100_spi_sched_rate_per_s(
        zy100_spi_sched_delta_u32(now_stats.flash_no_work,
                                  last->flash_no_work),
        elapsed_ms);
    emg_full_delta = zy100_spi_sched_delta_u32(
        now_stats.fifo_emergency_drain_full_len,
        last->fifo_emergency_drain_full_len);
    emg_staged_delta = zy100_spi_sched_delta_u32(
        now_stats.fifo_emergency_drain_staged,
        last->fifo_emergency_drain_staged);
    emg_drop_delta = zy100_spi_sched_delta_u32(
        now_stats.fifo_emergency_drop,
        last->fifo_emergency_drop);
    emg_saved_delta = zy100_spi_sched_delta_u32(
        now_stats.fifo_emergency_saved,
        last->fifo_emergency_saved);
    emg_bypass_delta = zy100_spi_sched_delta_u32(
        now_stats.fifo_emergency_capacity_bypass,
        last->fifo_emergency_capacity_bypass);

    DBG_DIRECT("[SPI_SCHED_1S] fifo irq_latch/s=%u drain/s=%u no_cap/s=%u cap/s=%u cross/s=%u",
               irq_latch_per_s,
               drain_per_s,
               no_cap_per_s,
               cap_per_s,
               cross_per_s);
    DBG_DIRECT("[SPI_SCHED_1S] flash pump/s=%u page/s=%u block/s=%u th_fifo/s=%u th_budget/s=%u wip_busy/s=%u no_work/s=%u",
               pump_per_s,
               page_per_s,
               block_per_s,
               th_fifo_per_s,
               th_budget_per_s,
               wip_busy_per_s,
               no_work_per_s);
    if ((emg_full_delta != 0U) ||
        (emg_staged_delta != 0U) ||
        (emg_drop_delta != 0U) ||
        (emg_saved_delta != 0U) ||
        (emg_bypass_delta != 0U))
    {
        DBG_DIRECT("[FIFO_EMG_1S] full/s=%u staged/s=%u drop/s=%u saved/s=%u bypass/s=%u",
                   zy100_spi_sched_rate_per_s(emg_full_delta, elapsed_ms),
                   zy100_spi_sched_rate_per_s(emg_staged_delta, elapsed_ms),
                   zy100_spi_sched_rate_per_s(emg_drop_delta, elapsed_ms),
                   zy100_spi_sched_rate_per_s(emg_saved_delta, elapsed_ms),
                   zy100_spi_sched_rate_per_s(emg_bypass_delta, elapsed_ms));
    }

    s_spi_sched.last_1s_stats = now_stats;
    s_spi_sched.last_1s_log_ms = now_ms;
#endif
}

void zy100_spi_sched_log_5s_if_due(uint32_t now_ms)
{
#if !ZY100_RUNTIME_STATS_LOG_ENABLE
    IMU_UNUSED(now_ms);
#else
    if (zy100_spi_sched_elapsed_ms(now_ms, s_spi_sched.last_log_ms) <
        ZY100_SPI_SCHED_LOG_PERIOD_MS)
    {
        return;
    }

    zy100_spi_sched_sync_rt_marker_stats();
    DBG_DIRECT("[SPI_SCHED_5S] mode=%s rt=%u tim6=%u max_fifo_count=%u rt_read_ok=%u rt_skip_fifo=%u rt_skip_spi=%u fifo_count_op=%u fifo_drain_op=%u flash_pump_op=%u flash_blocked_by_rt=%u flash_blocked_by_fifo=%u flash_starved_count=%u flash_q_level=%u flash_q_near_full=%u fifo_int_status_op=%u fifo_emg=%u due=%u late=%u fifo_drain_latched=%u fifo_count_over_capacity=%u fifo_count_to_drain_latch=%u fifo_count_rejected=%u no_op_count=%u",
               zy100_spi_sched_mode_name(),
               zy100_spi_sched_rt_enabled(),
               zy100_spi_sched_tim6_enabled(),
               s_spi_sched.stats.max_fifo_count,
               s_spi_sched.stats.rt_read_ok,
               s_spi_sched.stats.rt_skip_fifo,
               s_spi_sched.stats.rt_skip_spi,
               s_spi_sched.stats.fifo_count_op,
               s_spi_sched.stats.fifo_drain_op,
               s_spi_sched.stats.flash_pump_op,
               s_spi_sched.stats.flash_blocked_by_rt,
               s_spi_sched.stats.flash_blocked_by_fifo,
               s_spi_sched.stats.flash_starved_count,
               s_spi_sched.stats.flash_queue_level,
               s_spi_sched.stats.flash_queue_near_full,
               s_spi_sched.stats.fifo_int_status_op,
               s_spi_sched.stats.fifo_emergency_count,
               s_spi_sched.stats.rt_due,
               s_spi_sched.stats.rt_late,
               s_spi_sched.stats.fifo_drain_latched,
               s_spi_sched.stats.fifo_count_over_capacity,
               s_spi_sched.stats.fifo_count_to_drain_latch,
               s_spi_sched.stats.fifo_count_rejected,
               s_spi_sched.stats.no_op_count);
    DBG_DIRECT("[SPI_SCHED_5S] flash_page_issued=%u flash_block_done=%u flash_throttle_rt_due=%u flash_throttle_fifo_pending=%u flash_throttle_budget=%u max_consecutive_flash_steps=%u",
               s_spi_sched.stats.flash_page_issued,
               s_spi_sched.stats.flash_block_done,
               s_spi_sched.stats.flash_throttle_rt_due,
               s_spi_sched.stats.flash_throttle_fifo_pending,
               s_spi_sched.stats.flash_throttle_budget,
               s_spi_sched.stats.max_consecutive_flash_steps);
    DBG_DIRECT("[SPI_SCHED_5S] fifo_irq_to_drain_latch=%u fifo_fixed_drain_op=%u fifo_irq_drain_rt_defer=%u fifo_irq_drain_force_after_defer=%u fifo_count_reason_poll=%u fifo_count_reason_probe=%u fifo_count_reason_cache=%u fifo_count_reason_emergency_50ms=%u fifo_count_reason_debug_100ms=%u fifo_count_reason_post_drain=%u fifo_count_skipped_rt_due=%u",
               s_spi_sched.stats.fifo_irq_to_drain_latch,
               s_spi_sched.stats.fifo_fixed_drain_op,
               s_spi_sched.stats.fifo_irq_drain_rt_defer,
               s_spi_sched.stats.fifo_irq_drain_force_after_defer,
               s_spi_sched.stats.fifo_count_reason_poll,
               s_spi_sched.stats.fifo_count_reason_probe,
               s_spi_sched.stats.fifo_count_reason_cache,
               s_spi_sched.stats.fifo_count_reason_emergency_50ms,
               s_spi_sched.stats.fifo_count_reason_debug_100ms,
               s_spi_sched.stats.fifo_count_reason_post_drain,
               s_spi_sched.stats.fifo_count_skipped_rt_due);
    DBG_DIRECT("[SPI_SCHED_5S] fifo_int_status_deferred_rt=%u fifo_int_status_after_drain=%u post_drain_status_req=%u post_drain_count_req=%u irq_repeat_before_status=%u fifo_drain_fail_count=%u fifo_drain_fail_reason=%u fifo_drain_fail_last_len=%u fifo_drain_fail_first_header=0x%02x",
               s_spi_sched.stats.fifo_int_status_deferred_rt,
               s_spi_sched.stats.fifo_int_status_after_drain,
               s_spi_sched.stats.fifo_post_drain_int_status_req,
               s_spi_sched.stats.fifo_post_drain_count_req,
               s_spi_sched.stats.irq_repeat_before_status,
               s_spi_sched.stats.fifo_drain_fail_count,
               s_spi_sched.stats.fifo_drain_fail_reason,
               s_spi_sched.stats.fifo_drain_fail_last_len,
               s_spi_sched.stats.fifo_drain_fail_first_header);
    DBG_DIRECT("[SPI_SCHED_5S] post_drain count_op=%u lt_wm=%u ge_wm=%u ge_emg=%u",
               s_spi_sched.stats.post_drain_count_op,
               s_spi_sched.stats.post_drain_count_lt_wm,
               s_spi_sched.stats.post_drain_count_ge_wm,
               s_spi_sched.stats.post_drain_count_ge_emergency);
    DBG_DIRECT("[SPI_SCHED_5S] post_drain rechain=%u high=%u max_streak=%u",
               s_spi_sched.stats.post_drain_rechain,
               s_spi_sched.stats.post_drain_count_high,
               s_spi_sched.stats.consecutive_drain_count);
    DBG_DIRECT("[SPI_SCHED_5S] fifo_drain_len_requested=%u fifo_drain_len_actual=%u fifo_drain_capped_by_flash_capacity=%u fifo_drain_no_flash_capacity=%u fifo_drain_cross_block_append=%u",
               s_spi_sched.stats.fifo_drain_len_requested,
               s_spi_sched.stats.fifo_drain_len_actual,
               s_spi_sched.stats.fifo_drain_capped_by_flash_capacity,
               s_spi_sched.stats.fifo_drain_no_flash_capacity,
               s_spi_sched.stats.fifo_drain_cross_block_append);
    DBG_DIRECT("[SPI_SCHED_5S] fifo_start_guard_skip=%u fifo_first_irq_confirm=%u fifo_first_irq_confirm_ok=%u fifo_fixed_drain_after_warmup=%u early_drain_bad_header=%u first_good_drain_ms=%u",
               s_spi_sched.stats.fifo_start_guard_skip,
               s_spi_sched.stats.fifo_first_irq_confirm,
               s_spi_sched.stats.fifo_first_irq_confirm_ok,
               s_spi_sched.stats.fifo_fixed_drain_after_warmup,
               s_spi_sched.stats.early_drain_bad_header,
               s_spi_sched.stats.first_good_drain_ms);
    DBG_DIRECT("[SPI_SCHED_5S] count_probe_during_guard=%u count_confirm_to_pending_drain=%u unguarded_drain_blocked_by_start_guard=%u confirmed_drain_pending=%u",
               s_spi_sched.stats.count_probe_during_guard,
               s_spi_sched.stats.count_confirm_to_pending_drain,
               s_spi_sched.stats.unguarded_drain_blocked_by_start_guard,
               s_spi_sched.stats.confirmed_drain_pending);
    DBG_DIRECT("[SPI_SCHED_5S] fifo_service due=%u overdue=%u emergency=%u count_probe=%u recovered=%u rt_skip=%u flash_block=%u ms_since_last_drain=%u",
               s_spi_sched.stats.fifo_service_due_count,
               s_spi_sched.stats.fifo_service_overdue_count,
               s_spi_sched.stats.fifo_service_emergency_count,
               s_spi_sched.stats.fifo_service_count_probe,
               s_spi_sched.stats.fifo_service_recovered_by_count,
               s_spi_sched.stats.fifo_service_rt_skip,
               s_spi_sched.stats.fifo_service_flash_block,
               s_spi_sched.stats.ms_since_last_fifo_drain);
    s_spi_sched.last_log_ms = now_ms;
#endif
}
