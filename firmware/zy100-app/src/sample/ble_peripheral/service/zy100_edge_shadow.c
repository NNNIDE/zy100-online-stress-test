#include "zy100_edge_shadow.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "../app_flags.h"
#include "trace.h"
#if ZY100_EDGE_SHADOW_USE_REAL_ALGO
#include "zy100_edge_lite.h"
#endif
#if ZY100_EDGE_SHADOW_TIMING_ENABLE
#include "../zy100_clock_config.h"
#endif

typedef struct
{
    uint32_t input_sample_count;
    uint32_t accepted_sample_count;
    uint32_t decode_error_count;
    uint32_t error_count;
    uint32_t event_count;
    uint32_t summary_count;
    uint32_t raw_window_request_count;
    uint32_t dropped_count;
    uint32_t max_push_us;
    uint8_t timing_api_ok;
    bool capture_active;
} zy100_edge_shadow_capture_stats_t;

#if ZY100_EDGE_SHADOW_ENABLE
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
__align(4) static uint8_t s_edge_shadow_workspace[ZY100_EDGE_SHADOW_WORKSPACE_BYTES];
#elif defined(__GNUC__)
static uint8_t s_edge_shadow_workspace[ZY100_EDGE_SHADOW_WORKSPACE_BYTES] __attribute__((aligned(4)));
#else
static uint8_t s_edge_shadow_workspace[ZY100_EDGE_SHADOW_WORKSPACE_BYTES];
#endif
#endif

static bool s_edge_shadow_initialized = false;
#if ZY100_EDGE_SHADOW_USE_REAL_ALGO
static bool s_edge_shadow_algo_ready __attribute__((unused)) = false;
#endif
static zy100_edge_shadow_capture_stats_t s_edge_shadow_stats;

#if ZY100_EDGE_SHADOW_ENABLE
static void zy100_edge_shadow_inc_u32(uint32_t *value)
{
    if ((value != NULL) && (*value < 0xFFFFFFFFU))
    {
        (*value)++;
    }
}
#endif

#if ZY100_EDGE_SHADOW_ENABLE && IMU_CAPTURE_SUMMARY_LOG_ENABLE
static uint32_t zy100_edge_shadow_abs_diff_u32(uint32_t a, uint32_t b)
{
    return (a >= b) ? (a - b) : (b - a);
}
#endif

#if ZY100_EDGE_SHADOW_USE_REAL_ALGO
static uint32_t zy100_edge_shadow_saturating_add_u32(uint32_t a, uint32_t b)
{
    if ((0xFFFFFFFFU - a) < b)
    {
        return 0xFFFFFFFFU;
    }
    return a + b;
}
#endif

#if ZY100_EDGE_SHADOW_ENABLE && IMU_CAPTURE_SUMMARY_LOG_ENABLE
static uint32_t zy100_edge_shadow_mismatch_threshold(uint32_t feed_count)
{
    uint32_t threshold = feed_count / 100U;

    if (threshold < 32U)
    {
        threshold = 32U;
    }
    return threshold;
}
#endif

#if ZY100_EDGE_SHADOW_USE_REAL_ALGO
static void zy100_edge_shadow_sync_lite_stats(void)
    __attribute__((unused));
static void zy100_edge_shadow_sync_lite_stats(void)
{
    zy100_edge_lite_stats_t lite_stats;

    zy100_edge_lite_get_stats(&lite_stats);
    s_edge_shadow_stats.summary_count = lite_stats.summary_count;
    s_edge_shadow_stats.event_count = lite_stats.event_count;
    s_edge_shadow_stats.raw_window_request_count = lite_stats.raw_req_count;
    s_edge_shadow_stats.dropped_count =
        zy100_edge_shadow_saturating_add_u32(lite_stats.rejected_count,
                                             lite_stats.cooldown_drop_count);
}
#endif

uint32_t zy100_edge_shadow_workspace_reserved_bytes(void)
{
#if ZY100_EDGE_SHADOW_ENABLE
    return ZY100_EDGE_SHADOW_WORKSPACE_BYTES;
#else
    return 0U;
#endif
}

uint32_t zy100_edge_shadow_workspace_used_est_bytes(void)
{
#if ZY100_EDGE_SHADOW_ENABLE && ZY100_EDGE_SHADOW_USE_REAL_ALGO
    return zy100_edge_lite_context_bytes();
#else
    return 0U;
#endif
}

uint32_t zy100_edge_shadow_accepted_count(void)
{
    return s_edge_shadow_stats.accepted_sample_count;
}

void zy100_edge_shadow_init(void)
{
    if (s_edge_shadow_initialized)
    {
        return;
    }

    memset(&s_edge_shadow_stats, 0, sizeof(s_edge_shadow_stats));
#if ZY100_EDGE_SHADOW_ENABLE && ZY100_EDGE_SHADOW_USE_REAL_ALGO
    s_edge_shadow_algo_ready =
        zy100_edge_lite_init(s_edge_shadow_workspace, sizeof(s_edge_shadow_workspace));
#endif
    s_edge_shadow_initialized = true;

#if ZY100_EDGE_SHADOW_ENABLE && (IMU_CAPTURE_SUMMARY_LOG_ENABLE || ZY100_RUNTIME_STATS_LOG_ENABLE)
    DBG_DIRECT("[EDGE_SHADOW_CFG] enable=%u workspace=%u real_algo=%u timing=%u",
               (uint32_t)ZY100_EDGE_SHADOW_ENABLE,
               (uint32_t)ZY100_EDGE_SHADOW_WORKSPACE_BYTES,
               (uint32_t)ZY100_EDGE_SHADOW_USE_REAL_ALGO,
               (uint32_t)ZY100_EDGE_SHADOW_TIMING_ENABLE);
#endif
}

void zy100_edge_shadow_capture_start(void)
{
    if (!s_edge_shadow_initialized)
    {
        zy100_edge_shadow_init();
    }

    memset(&s_edge_shadow_stats, 0, sizeof(s_edge_shadow_stats));
    s_edge_shadow_stats.capture_active = true;
#if ZY100_EDGE_SHADOW_TIMING_ENABLE
    s_edge_shadow_stats.timing_api_ok = 1U;
#else
    s_edge_shadow_stats.timing_api_ok = 0U;
#endif

#if ZY100_EDGE_SHADOW_ENABLE && ZY100_EDGE_SHADOW_USE_REAL_ALGO
    if (!s_edge_shadow_algo_ready)
    {
        s_edge_shadow_algo_ready =
            zy100_edge_lite_init(s_edge_shadow_workspace, sizeof(s_edge_shadow_workspace));
    }
    if (s_edge_shadow_algo_ready)
    {
        zy100_edge_lite_capture_start();
    }
    else
    {
        zy100_edge_shadow_inc_u32(&s_edge_shadow_stats.error_count);
    }
#endif
}

bool zy100_edge_shadow_push_sample_ex(const zy100_edge_shadow_sample_t *sample,
                                      zy100_edge_lite_output_t *out)
{
#if ZY100_EDGE_SHADOW_ENABLE
#if ZY100_EDGE_SHADOW_TIMING_ENABLE
    uint32_t start_ms = zy100_os_time_ms();
    uint32_t end_ms;
    uint32_t elapsed_us;
#endif
#if ZY100_EDGE_SHADOW_USE_REAL_ALGO
    zy100_edge_lite_output_t lite_out;
    zy100_edge_lite_output_t *lite_out_ptr = (out != NULL) ? out : &lite_out;
#endif

    if ((sample == NULL) || !s_edge_shadow_stats.capture_active)
    {
        zy100_edge_shadow_inc_u32(&s_edge_shadow_stats.error_count);
        return false;
    }

    zy100_edge_shadow_inc_u32(&s_edge_shadow_stats.input_sample_count);

#if ZY100_EDGE_SHADOW_USE_REAL_ALGO
    if (!s_edge_shadow_algo_ready)
    {
        zy100_edge_shadow_inc_u32(&s_edge_shadow_stats.error_count);
        return false;
    }
    if (zy100_edge_lite_push_sample(sample, lite_out_ptr))
    {
        zy100_edge_shadow_inc_u32(&s_edge_shadow_stats.accepted_sample_count);
        if ((lite_out_ptr->flags & ZY100_EDGE_LITE_OUT_SUMMARY) != 0U)
        {
            zy100_edge_shadow_inc_u32(&s_edge_shadow_stats.summary_count);
        }
        if ((lite_out_ptr->flags & ZY100_EDGE_LITE_OUT_EVENT) != 0U)
        {
            zy100_edge_shadow_inc_u32(&s_edge_shadow_stats.event_count);
        }
        if ((lite_out_ptr->flags & ZY100_EDGE_LITE_OUT_RAW_REQ) != 0U)
        {
            zy100_edge_shadow_inc_u32(&s_edge_shadow_stats.raw_window_request_count);
        }
    }
    else
    {
        zy100_edge_shadow_inc_u32(&s_edge_shadow_stats.error_count);
        return false;
    }
#else
    (void)out;
    zy100_edge_shadow_inc_u32(&s_edge_shadow_stats.accepted_sample_count);
#endif

#if ZY100_EDGE_SHADOW_TIMING_ENABLE
    end_ms = zy100_os_time_ms();
    elapsed_us = (end_ms - start_ms) * 1000U;
    if (elapsed_us > s_edge_shadow_stats.max_push_us)
    {
        s_edge_shadow_stats.max_push_us = elapsed_us;
    }
#endif
    return true;
#else
    (void)sample;
    (void)out;
    return false;
#endif
}

void zy100_edge_shadow_push_sample(const zy100_edge_shadow_sample_t *sample)
{
    (void)zy100_edge_shadow_push_sample_ex(sample, NULL);
}

void zy100_edge_shadow_note_decode_error(void)
{
#if ZY100_EDGE_SHADOW_ENABLE
    zy100_edge_shadow_inc_u32(&s_edge_shadow_stats.decode_error_count);
#endif
}

bool zy100_edge_shadow_capture_end_ex(zy100_edge_lite_output_t *out)
{
#if ZY100_EDGE_SHADOW_ENABLE && ZY100_EDGE_SHADOW_USE_REAL_ALGO
    bool emitted = false;

    if (s_edge_shadow_algo_ready)
    {
        emitted = zy100_edge_lite_capture_end(out);
        zy100_edge_shadow_sync_lite_stats();
    }
    s_edge_shadow_stats.capture_active = false;
    return emitted;
#else
    (void)out;
    s_edge_shadow_stats.capture_active = false;
    return false;
#endif
}

bool zy100_edge_shadow_capture_end_discard_partial(zy100_edge_lite_output_t *out)
{
#if ZY100_EDGE_SHADOW_ENABLE && ZY100_EDGE_SHADOW_USE_REAL_ALGO
    if (s_edge_shadow_algo_ready)
    {
        (void)zy100_edge_lite_capture_stop(ZY100_EDGE_LITE_STOP_DISCARD_PARTIAL, out);
        zy100_edge_shadow_sync_lite_stats();
    }
    s_edge_shadow_stats.capture_active = false;
    return false;
#else
    (void)out;
    s_edge_shadow_stats.capture_active = false;
    return false;
#endif
}

void zy100_edge_shadow_capture_end(void)
{
    (void)zy100_edge_shadow_capture_end_ex(NULL);
}

void zy100_edge_shadow_print_summary(void)
{
#if ZY100_EDGE_SHADOW_ENABLE && IMU_CAPTURE_SUMMARY_LOG_ENABLE
#if ZY100_EDGE_SHADOW_USE_REAL_ALGO
    zy100_edge_lite_stats_t lite_stats;

    if (s_edge_shadow_algo_ready)
    {
        zy100_edge_shadow_sync_lite_stats();
    }
    zy100_edge_lite_get_stats(&lite_stats);
#endif

    DBG_DIRECT("[EDGE_SHADOW_SUM_A] enable=%u workspace=%u real_algo=%u samples=%u accepted=%u errors=%u decode_errors=%u",
               (uint32_t)ZY100_EDGE_SHADOW_ENABLE,
               (uint32_t)ZY100_EDGE_SHADOW_WORKSPACE_BYTES,
               (uint32_t)ZY100_EDGE_SHADOW_USE_REAL_ALGO,
               s_edge_shadow_stats.input_sample_count,
               s_edge_shadow_stats.accepted_sample_count,
               s_edge_shadow_stats.error_count,
               s_edge_shadow_stats.decode_error_count);
    DBG_DIRECT("[EDGE_SHADOW_SUM_B] summary_out=%u event_out=%u raw_req=%u dropped=%u",
               s_edge_shadow_stats.summary_count,
               s_edge_shadow_stats.event_count,
               s_edge_shadow_stats.raw_window_request_count,
               s_edge_shadow_stats.dropped_count);
    DBG_DIRECT("[EDGE_SHADOW_SUM_C] max_push_us=%u timing_api_ok=%u",
               s_edge_shadow_stats.max_push_us,
               (uint32_t)s_edge_shadow_stats.timing_api_ok);
    DBG_DIRECT("[EDGE_SHADOW_SUM_D] workspace_used_est=%u algo_reserved=%u algo_used=%u heap_alloc=%u",
               zy100_edge_shadow_workspace_used_est_bytes(),
               zy100_edge_shadow_workspace_reserved_bytes(),
               zy100_edge_shadow_workspace_used_est_bytes(),
               0U);
#if ZY100_EDGE_SHADOW_USE_REAL_ALGO
    DBG_DIRECT("[EDGE_LITE_SUM_A] profile=high_recall samples=%u summaries=%u events=%u raw_req=%u",
               lite_stats.sample_count,
               lite_stats.summary_count,
               lite_stats.event_count,
               lite_stats.raw_req_count);
    DBG_DIRECT("[EDGE_LITE_SUM_B] trigger=%u quiet=%u chunk=%u summary_period=%u cooldown=%u",
               lite_stats.trigger_threshold,
               lite_stats.quiet_threshold,
               lite_stats.chunk_samples,
               lite_stats.summary_period_samples,
               lite_stats.cooldown_samples);
    DBG_DIRECT("[EDGE_LITE_SUM_C] clip=%u sentinel=%u rejected=%u cooldown_drop=%u max_event_forced=%u",
               lite_stats.clip_count,
               lite_stats.sentinel_count,
               lite_stats.rejected_count,
               lite_stats.cooldown_drop_count,
               lite_stats.max_event_forced_count);
    DBG_DIRECT("[EDGE_LITE_SUM_D] ctx_bytes=%u workspace=%u workspace_ok=%u",
               lite_stats.ctx_bytes,
               lite_stats.workspace_bytes,
               lite_stats.workspace_ok);
#endif
#endif
}

void zy100_edge_shadow_check_sample_mismatch(uint32_t edge_feed_count)
{
#if ZY100_EDGE_SHADOW_ENABLE && IMU_CAPTURE_SUMMARY_LOG_ENABLE
    uint32_t accepted = s_edge_shadow_stats.accepted_sample_count;
    uint32_t delta = zy100_edge_shadow_abs_diff_u32(edge_feed_count, accepted);
    uint32_t threshold = zy100_edge_shadow_mismatch_threshold(edge_feed_count);
    int32_t diff = (int32_t)accepted - (int32_t)edge_feed_count;

    if (delta > threshold)
    {
        DBG_DIRECT("[WARN][EDGE_SHADOW] edge_sample_mismatch edge=%u feed=%u diff=%d",
                   accepted,
                   edge_feed_count,
                   diff);
    }
#else
    (void)edge_feed_count;
#endif
}
