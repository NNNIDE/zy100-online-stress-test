#include "zy100_capture_time.h"

#include <stdbool.h>
#include <string.h>

#include "trace.h"
#include "../common/zy100_byteorder.h"
#include "zy100_online_stream.h"
#include "zy100_rtc_clock.h"

#define ZY100_CAPTURE_TIME_META_MAGIC0   ((uint8_t)'R')
#define ZY100_CAPTURE_TIME_META_MAGIC1   ((uint8_t)'T')
#define ZY100_CAPTURE_TIME_META_MAGIC2   ((uint8_t)'M')
#define ZY100_CAPTURE_TIME_META_MAGIC3   ((uint8_t)'1')
#define ZY100_CAPTURE_TIME_META_VERSION  1U
#define ZY100_CAPTURE_TIME_DRIFT_WARN_MS 2000

typedef struct
{
    bool started;
    uint8_t start_source;
    uint8_t stop_source;
    uint8_t time_source;
    uint8_t calibrated_start;
    uint8_t calibrated_stop;
    uint32_t user_id;
    uint32_t training_id;
    uint64_t start_ms;
    uint64_t stop_ms;
    uint64_t cmd_start_ms;
    uint64_t cmd_stop_ms;
} zy100_capture_time_ctx_t;

static zy100_capture_time_ctx_t s_capture_time;

static bool zy100_capture_time_online_quiet(uint8_t source)
{
#if ZY100_ONLINE_STREAM_ENABLE
#if ZY100_ONLINE_STREAM_VERBOSE_TRACE_ENABLE
    (void)source;
    return false;
#else
    return (source == ZY100_CAPTURE_TIME_SRC_BLE) &&
           zy100_online_stream_quiet_logs_active();
#endif
#else
    (void)source;
    return false;
#endif
}

static const char *zy100_capture_time_source_name(uint8_t source)
{
    if (source == ZY100_CAPTURE_TIME_SRC_BLE)
    {
        return "ble";
    }
    if (source == ZY100_CAPTURE_TIME_SRC_BUTTON)
    {
        return "button";
    }
    return "auto";
}

static int32_t zy100_capture_time_diff_ms(uint64_t a_ms, uint64_t b_ms)
{
    uint64_t delta;

    if (a_ms >= b_ms)
    {
        delta = a_ms - b_ms;
        return (delta > 0x7FFFFFFFULL) ? 0x7FFFFFFF : (int32_t)delta;
    }

    delta = b_ms - a_ms;
    return (delta > 0x80000000ULL) ? (-2147483647 - 1) : -(int32_t)delta;
}

static void zy100_capture_time_warn_drift(uint8_t source,
                                          uint64_t rtc_ms,
                                          uint64_t cmd_ms,
                                          const char *cmd_name)
{
    int32_t delta_ms;

    if ((source != ZY100_CAPTURE_TIME_SRC_BLE) || (cmd_ms == 0ULL))
    {
        return;
    }

    delta_ms = zy100_capture_time_diff_ms(cmd_ms, rtc_ms);
    if ((delta_ms > ZY100_CAPTURE_TIME_DRIFT_WARN_MS) ||
        (delta_ms < -ZY100_CAPTURE_TIME_DRIFT_WARN_MS))
    {
        DBG_DIRECT("[RTC_WARN] cmd_time_drift_ms=%d threshold=%d cmd=%s",
                   (int)delta_ms,
                   ZY100_CAPTURE_TIME_DRIFT_WARN_MS,
                   cmd_name);
    }
}

bool zy100_capture_time_decode_meta(
    const uint8_t meta[ZY100_CAPTURE_TIME_META_BYTES],
    uint64_t *start_ms,
    uint64_t *stop_ms,
    uint8_t *calibrated_start,
    uint8_t *calibrated_stop,
    uint8_t *time_source)
{
    if (meta == NULL)
    {
        return false;
    }
    if ((meta[0] != ZY100_CAPTURE_TIME_META_MAGIC0) ||
        (meta[1] != ZY100_CAPTURE_TIME_META_MAGIC1) ||
        (meta[2] != ZY100_CAPTURE_TIME_META_MAGIC2) ||
        (meta[3] != ZY100_CAPTURE_TIME_META_MAGIC3) ||
        (meta[4] != ZY100_CAPTURE_TIME_META_VERSION))
    {
        return false;
    }

    if (time_source != NULL)
    {
        *time_source = meta[5];
    }
    if (calibrated_start != NULL)
    {
        *calibrated_start = meta[6];
    }
    if (calibrated_stop != NULL)
    {
        *calibrated_stop = meta[7];
    }
    if (start_ms != NULL)
    {
        *start_ms = zy100_get_u64_le(&meta[16]);
    }
    if (stop_ms != NULL)
    {
        *stop_ms = zy100_get_u64_le(&meta[24]);
    }
    return true;
}

void zy100_capture_time_reset(void)
{
    memset(&s_capture_time, 0, sizeof(s_capture_time));
}

void zy100_capture_time_note_start(zy100_capture_time_src_t source,
                                   uint32_t user_id,
                                   uint32_t training_id,
                                   uint64_t cmd_time_ms)
{
    uint64_t now_ms = zy100_rtc_clock_now_ms();
    bool calibrated = zy100_rtc_clock_is_calibrated();
    uint8_t time_source = zy100_rtc_clock_time_source();

    memset(&s_capture_time, 0, sizeof(s_capture_time));
    s_capture_time.started = true;
    s_capture_time.start_source = (uint8_t)source;
    s_capture_time.stop_source = (uint8_t)source;
    s_capture_time.time_source = time_source;
    s_capture_time.calibrated_start = calibrated ? 1U : 0U;
    s_capture_time.user_id = user_id;
    s_capture_time.training_id = training_id;
    s_capture_time.start_ms = now_ms;
    s_capture_time.stop_ms = now_ms;
    s_capture_time.cmd_start_ms = cmd_time_ms;

    if (source == ZY100_CAPTURE_TIME_SRC_BLE)
    {
        if (!zy100_capture_time_online_quiet((uint8_t)source))
        {
            ZY100_LOG_ROUTINE(DBG_DIRECT, "[CAP_TIME] start source=ble calibrated=%u source_id=%u rtc_ms=%llu cmd_ms=%llu user=%u train=%u",
                       s_capture_time.calibrated_start,
                       time_source,
                       (unsigned long long)now_ms,
                       (unsigned long long)cmd_time_ms,
                       s_capture_time.user_id,
                       s_capture_time.training_id);
            zy100_capture_time_warn_drift((uint8_t)source,
                                          now_ms,
                                          cmd_time_ms,
                                          "START_CAPTURE");
        }
    }
    else
    {
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[CAP_TIME] start source=button calibrated=%u source_id=%u rtc_ms=%llu user=%u train=%u",
                   s_capture_time.calibrated_start,
                   time_source,
                   (unsigned long long)now_ms,
                   s_capture_time.user_id,
                   s_capture_time.training_id);
    }
}

void zy100_capture_time_set_stop_source(zy100_capture_time_src_t source,
                                        uint64_t cmd_time_ms)
{
    s_capture_time.stop_source = (uint8_t)source;
    s_capture_time.cmd_stop_ms = cmd_time_ms;
}

void zy100_capture_time_note_stop(void)
{
    uint64_t now_ms;
    uint64_t duration_ms;

    if (!s_capture_time.started)
    {
        return;
    }

    now_ms = zy100_rtc_clock_now_ms();
    s_capture_time.stop_ms = now_ms;
    s_capture_time.calibrated_stop =
        zy100_rtc_clock_is_calibrated() ? 1U : 0U;
    duration_ms = (now_ms >= s_capture_time.start_ms) ?
                  (now_ms - s_capture_time.start_ms) : 0ULL;

    if (!zy100_capture_time_online_quiet(s_capture_time.stop_source))
    {
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[CAP_TIME] stop source=%s calibrated=%u source_id=%u rtc_ms=%llu duration_ms=%llu",
                   zy100_capture_time_source_name(s_capture_time.stop_source),
                   s_capture_time.calibrated_stop,
                   zy100_rtc_clock_time_source(),
                   (unsigned long long)now_ms,
                   (unsigned long long)duration_ms);
        zy100_capture_time_warn_drift(s_capture_time.stop_source,
                                      now_ms,
                                      s_capture_time.cmd_stop_ms,
                                      "PAUSE_CAPTURE");
    }
}

void zy100_capture_time_build_meta(uint8_t out[ZY100_CAPTURE_TIME_META_BYTES])
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, ZY100_CAPTURE_TIME_META_BYTES);
    out[0] = ZY100_CAPTURE_TIME_META_MAGIC0;
    out[1] = ZY100_CAPTURE_TIME_META_MAGIC1;
    out[2] = ZY100_CAPTURE_TIME_META_MAGIC2;
    out[3] = ZY100_CAPTURE_TIME_META_MAGIC3;
    out[4] = ZY100_CAPTURE_TIME_META_VERSION;
    out[5] = s_capture_time.time_source;
    out[6] = s_capture_time.calibrated_start;
    out[7] = s_capture_time.calibrated_stop;
    zy100_put_u32_le(&out[8], s_capture_time.user_id);
    zy100_put_u32_le(&out[12], s_capture_time.training_id);
    zy100_put_u64_le(&out[16], s_capture_time.start_ms);
    zy100_put_u64_le(&out[24], s_capture_time.stop_ms);
}

void zy100_capture_time_log_export_from_reserved(const uint32_t reserved[32])
{
    const uint8_t *meta = (const uint8_t *)reserved;
    uint32_t user_id;
    uint32_t training_id;
    uint64_t start_ms;
    uint64_t stop_ms;
    uint64_t duration_ms;

    if (meta == NULL)
    {
        return;
    }
    if ((meta[0] != ZY100_CAPTURE_TIME_META_MAGIC0) ||
        (meta[1] != ZY100_CAPTURE_TIME_META_MAGIC1) ||
        (meta[2] != ZY100_CAPTURE_TIME_META_MAGIC2) ||
        (meta[3] != ZY100_CAPTURE_TIME_META_MAGIC3) ||
        (meta[4] != ZY100_CAPTURE_TIME_META_VERSION))
    {
        return;
    }

    user_id = zy100_get_u32_le(&meta[8]);
    training_id = zy100_get_u32_le(&meta[12]);
    start_ms = zy100_get_u64_le(&meta[16]);
    stop_ms = zy100_get_u64_le(&meta[24]);
    duration_ms = (stop_ms >= start_ms) ? (stop_ms - start_ms) : 0ULL;

    ZY100_LOG_ROUTINE(DBG_DIRECT, "[CAP_TIME_EXPORT] time_source=%u calibrated_start=%u calibrated_stop=%u user=%u train=%u start_ms=%llu stop_ms=%llu duration_ms=%llu",
               meta[5],
               meta[6],
               meta[7],
               user_id,
               training_id,
               (unsigned long long)start_ms,
               (unsigned long long)stop_ms,
               (unsigned long long)duration_ms);
}
