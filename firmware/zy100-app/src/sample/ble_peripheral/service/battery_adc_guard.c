#include "battery_adc_guard.h"

#if F_APP_BATTERY_ADC_ENABLE && F_APP_BATTERY_ADC_GUARD_ENABLE

#include <string.h>

#include "../app_flags.h"
#include "../app_task.h"
#include "../bsp/imu_bsp.h"
#include "battery_adc.h"
#include "battery_policy.h"

#if (BATTERY_ADC_GUARD_SAMPLE_COUNT == 0U)
#error "BATTERY_ADC_GUARD_SAMPLE_COUNT must be non-zero"
#endif

#if (BATTERY_ADC_GUARD_CONSECUTIVE_LOW_COUNT == 0U)
#error "BATTERY_ADC_GUARD_CONSECUTIVE_LOW_COUNT must be non-zero"
#endif

#if (BATTERY_ADC_GUARD_CONSECUTIVE_FAIL_COUNT == 0U)
#error "BATTERY_ADC_GUARD_CONSECUTIVE_FAIL_COUNT must be non-zero"
#endif

#if (BATTERY_ADC_GUARD_MIN_SAMPLES <= (2U * BATTERY_ADC_GUARD_TRIM_COUNT)) || \
    (BATTERY_ADC_GUARD_MIN_SAMPLES > BATTERY_ADC_GUARD_SAMPLE_COUNT) || \
    (BATTERY_ADC_GUARD_COLLECT_OFFSET_MS >= BATTERY_ADC_GUARD_PERIOD_MS)
#error "Invalid capture ADC window configuration"
#endif

typedef struct
{
    bool armed;
    bool session_prepared;
    bool collecting;
    bool low_latched;
    bool block_counted;
    bool report_pending;
    uint8_t valid_count;
    uint8_t low_count;
    uint8_t consecutive_fail_count;
    uint16_t window_raw[BATTERY_ADC_GUARD_SAMPLE_COUNT];
    uint32_t window_end_ms;
    uint8_t invalid_window_streak;
    uint64_t read_total_us;
    uint32_t next_due_ms;
    uint32_t last_due_ms;
    uint32_t last_sample_ms;
    uint32_t last_attempt_ms;
    uint32_t last_check_ms;
    uint32_t poll_count;
    uint32_t due_count;
    uint32_t blocker_b_critical_count;
    uint32_t blocker_spi_count;
    uint32_t blocker_fifo_count;
    uint32_t blocker_raw_count;
    uint32_t blocker_record_count;
    uint32_t blocker_replay_count;
    uint32_t blocker_flash_count;
    uint32_t blocker_ble_count;
    uint32_t blocker_adc_count;
    uint32_t blocker_state_count;
    uint32_t blocker_budget_count;
    uint32_t invalid_count;
    uint32_t one_shot_fail_count;
    uint32_t slow_one_shot_count;
    uint32_t warmup_count;
    uint32_t sample_count;
    uint32_t valid_sample_count;
    uint64_t capture_raw_sum;
    uint16_t capture_raw_min;
    uint16_t capture_raw_max;
    bool capture_raw_valid;
    bool capture_stats_pending;
    uint32_t eval_count;
    uint32_t low_eval_count;
    uint16_t last_eval_raw;
    uint32_t last_one_shot_us;
    uint32_t max_one_shot_us;
    uint16_t last_mv;
    uint16_t threshold_mv;
    uint8_t last_percent;
    battery_adc_guard_report_t report;
    battery_adc_guard_capture_stats_t capture_stats;
    battery_adc_guard_block_reason_t last_block_reason;
} battery_adc_guard_state_t;

static battery_adc_guard_state_t s_battery_adc_guard;

static bool battery_adc_guard_time_reached(uint32_t now_ms, uint32_t due_ms)
{
    return ((int32_t)(now_ms - due_ms) >= 0);
}

static uint16_t battery_adc_guard_threshold_mv(void)
{
    uint32_t threshold_mv =
        (uint32_t)battery_adc_percent_to_voltage_mv(APP_BATTERY_CRITICAL_PERCENT) +
        (uint32_t)BATTERY_ADC_GUARD_LOW_MARGIN_MV;

    return (threshold_mv > 0xFFFFUL) ? 0xFFFFU : (uint16_t)threshold_mv;
}

static void battery_adc_guard_reset_collection(void)
{
    s_battery_adc_guard.collecting = false;
    s_battery_adc_guard.valid_count = 0U;
}

static void battery_adc_guard_schedule_next(uint32_t now_ms)
{
    s_battery_adc_guard.last_sample_ms = now_ms;
    s_battery_adc_guard.next_due_ms = now_ms +
        (uint32_t)BATTERY_ADC_GUARD_SAMPLE_INTERVAL_MS;
}

static void battery_adc_guard_clear_counters(void)
{
    s_battery_adc_guard.poll_count = 0U;
    s_battery_adc_guard.due_count = 0U;
    s_battery_adc_guard.blocker_b_critical_count = 0U;
    s_battery_adc_guard.blocker_spi_count = 0U;
    s_battery_adc_guard.blocker_fifo_count = 0U;
    s_battery_adc_guard.blocker_raw_count = 0U;
    s_battery_adc_guard.blocker_record_count = 0U;
    s_battery_adc_guard.blocker_replay_count = 0U;
    s_battery_adc_guard.blocker_flash_count = 0U;
    s_battery_adc_guard.blocker_ble_count = 0U;
    s_battery_adc_guard.blocker_adc_count = 0U;
    s_battery_adc_guard.blocker_state_count = 0U;
    s_battery_adc_guard.blocker_budget_count = 0U;
    s_battery_adc_guard.invalid_count = 0U;
    s_battery_adc_guard.one_shot_fail_count = 0U;
    s_battery_adc_guard.slow_one_shot_count = 0U;
    s_battery_adc_guard.warmup_count = 0U;
    s_battery_adc_guard.sample_count = 0U;
    s_battery_adc_guard.valid_sample_count = 0U;
    s_battery_adc_guard.capture_raw_sum = 0ULL;
    s_battery_adc_guard.capture_raw_min = 0U;
    s_battery_adc_guard.capture_raw_max = 0U;
    s_battery_adc_guard.capture_raw_valid = false;
    s_battery_adc_guard.eval_count = 0U;
    s_battery_adc_guard.low_eval_count = 0U;
    s_battery_adc_guard.last_one_shot_us = 0U;
    s_battery_adc_guard.max_one_shot_us = 0U;
    s_battery_adc_guard.read_total_us = 0ULL;
    s_battery_adc_guard.invalid_window_streak = 0U;
}

static void battery_adc_guard_start_collection(void)
{
    s_battery_adc_guard.collecting = true;
    s_battery_adc_guard.valid_count = 0U;
}

static void battery_adc_guard_note_block(
    battery_adc_guard_block_reason_t block_reason)
{
    if (block_reason == BATTERY_ADC_GUARD_BLOCK_NONE)
    {
        return;
    }

    if (s_battery_adc_guard.block_counted &&
        (s_battery_adc_guard.last_block_reason == block_reason))
    {
        return;
    }

    s_battery_adc_guard.block_counted = true;
    s_battery_adc_guard.last_block_reason = block_reason;

    switch (block_reason)
    {
    case BATTERY_ADC_GUARD_BLOCK_CAPTURE:
    case BATTERY_ADC_GUARD_BLOCK_STOP:
    case BATTERY_ADC_GUARD_BLOCK_STATE:
        s_battery_adc_guard.blocker_state_count++;
        break;
    case BATTERY_ADC_GUARD_BLOCK_B_CRITICAL:
        s_battery_adc_guard.blocker_b_critical_count++;
        break;
    case BATTERY_ADC_GUARD_BLOCK_SPI:
        s_battery_adc_guard.blocker_spi_count++;
        break;
    case BATTERY_ADC_GUARD_BLOCK_FIFO:
        s_battery_adc_guard.blocker_fifo_count++;
        break;
    case BATTERY_ADC_GUARD_BLOCK_RAW:
        s_battery_adc_guard.blocker_raw_count++;
        break;
    case BATTERY_ADC_GUARD_BLOCK_RECORD:
        s_battery_adc_guard.blocker_record_count++;
        break;
    case BATTERY_ADC_GUARD_BLOCK_REPLAY:
        s_battery_adc_guard.blocker_replay_count++;
        break;
    case BATTERY_ADC_GUARD_BLOCK_STORE:
    case BATTERY_ADC_GUARD_BLOCK_FLASH:
        s_battery_adc_guard.blocker_flash_count++;
        break;
    case BATTERY_ADC_GUARD_BLOCK_EXPORT:
    case BATTERY_ADC_GUARD_BLOCK_BLE:
        s_battery_adc_guard.blocker_ble_count++;
        break;
    case BATTERY_ADC_GUARD_BLOCK_ADC:
        s_battery_adc_guard.blocker_adc_count++;
        break;
    case BATTERY_ADC_GUARD_BLOCK_BUDGET:
        s_battery_adc_guard.blocker_budget_count++;
        break;
    default:
        break;
    }
}

static void battery_adc_guard_note_invalid(uint32_t now_ms, const char *reason)
{
    s_battery_adc_guard.invalid_count++;
    s_battery_adc_guard.invalid_window_streak++;
    s_battery_adc_guard.low_count = 0U;
    battery_adc_guard_reset_collection();
    (void)reason;
    if (s_battery_adc_guard.invalid_window_streak >=
        (uint8_t)BATTERY_ADC_GUARD_INVALID_WINDOWS)
    {
        battery_adc_guard_on_capture_stop(now_ms, "adc_invalid_windows");
        app_task_battery_adc_guard_fault_event_handle(now_ms);
    }
}

static uint16_t battery_adc_guard_trimmed_raw(void)
{
    uint8_t i;
    uint8_t j;
    uint16_t value;
    uint32_t sum = 0U;
    uint8_t count = s_battery_adc_guard.valid_count;
    uint8_t kept = count - (2U * BATTERY_ADC_GUARD_TRIM_COUNT);

    /* At most nine elements; raw lifetime statistics are not modified. */
    for (i = 1U; i < count; i++)
    {
        value = s_battery_adc_guard.window_raw[i];
        j = i;
        while ((j > 0U) && (s_battery_adc_guard.window_raw[j - 1U] > value))
        {
            s_battery_adc_guard.window_raw[j] = s_battery_adc_guard.window_raw[j - 1U];
            j--;
        }
        s_battery_adc_guard.window_raw[j] = value;
    }
    for (i = BATTERY_ADC_GUARD_TRIM_COUNT;
         i < count - BATTERY_ADC_GUARD_TRIM_COUNT; i++)
    {
        sum += s_battery_adc_guard.window_raw[i];
    }
    return (uint16_t)((sum + (kept / 2U)) / kept);
}

static void battery_adc_guard_eval_complete(uint32_t now_ms)
{
    battery_adc_guard_one_shot_t avg;
    uint16_t avg_raw;
    bool low;

    if (s_battery_adc_guard.valid_count < BATTERY_ADC_GUARD_MIN_SAMPLES)
    {
        battery_adc_guard_note_invalid(now_ms, "insufficient_samples");
        return;
    }
    avg_raw = battery_adc_guard_trimmed_raw();
    if (!battery_adc_guard_convert_raw(avg_raw, &avg))
    {
        battery_adc_guard_note_invalid(now_ms, "avg_raw");
        return;
    }

    s_battery_adc_guard.last_mv = avg.battery_mv;
    s_battery_adc_guard.last_percent = avg.percent;
    s_battery_adc_guard.last_eval_raw = avg_raw;
    s_battery_adc_guard.eval_count++;
    s_battery_adc_guard.invalid_window_streak = 0U;
    s_battery_adc_guard.report.check_count = s_battery_adc_guard.eval_count;
    s_battery_adc_guard.report.runtime_ms = now_ms;
    s_battery_adc_guard.report.raw = avg_raw;
    s_battery_adc_guard.report.battery_mv = avg.battery_mv;
    s_battery_adc_guard.report.percent = avg.percent;
    s_battery_adc_guard.report_pending = true;
    low = (avg.battery_mv <= s_battery_adc_guard.threshold_mv) ||
          (avg.percent <= (uint8_t)APP_BATTERY_CRITICAL_PERCENT);

    if (low)
    {
        s_battery_adc_guard.low_count++;
        s_battery_adc_guard.low_eval_count++;
    }
    else
    {
        s_battery_adc_guard.low_count = 0U;
    }

    battery_adc_guard_reset_collection();

    if (s_battery_adc_guard.low_count >=
        (uint8_t)BATTERY_ADC_GUARD_CONSECUTIVE_LOW_COUNT)
    {
        s_battery_adc_guard.low_latched = true;
        s_battery_adc_guard.armed = false;
        app_task_battery_adc_guard_low_event_handle(now_ms,
                                                    avg.battery_mv,
                                                    avg.percent);
    }
}

static bool battery_adc_guard_sample_one(uint32_t now_ms)
{
    battery_adc_guard_one_shot_t sample;
    uint32_t start_us;
    uint32_t elapsed_us;
    bool valid;

    if ((s_battery_adc_guard.last_attempt_ms != 0U) &&
        (s_battery_adc_guard.last_attempt_ms == now_ms))
    {
        return false;
    }

    s_battery_adc_guard.last_attempt_ms = now_ms;
    s_battery_adc_guard.sample_count++;
    start_us = (uint32_t)imu_bsp_local_timestamp_us();
    valid = battery_adc_guard_session_read(&sample) && sample.valid;
    elapsed_us = (uint32_t)((uint32_t)imu_bsp_local_timestamp_us() - start_us);
    s_battery_adc_guard.read_total_us += (uint64_t)elapsed_us;
    s_battery_adc_guard.last_one_shot_us = elapsed_us;
    if (elapsed_us > s_battery_adc_guard.max_one_shot_us)
    {
        s_battery_adc_guard.max_one_shot_us = elapsed_us;
    }
    battery_adc_guard_schedule_next(now_ms);
    if (!valid)
    {
        s_battery_adc_guard.one_shot_fail_count++;
        s_battery_adc_guard.consecutive_fail_count++;
        if (s_battery_adc_guard.consecutive_fail_count >=
            (uint8_t)BATTERY_ADC_GUARD_CONSECUTIVE_FAIL_COUNT)
        {
            battery_adc_guard_on_capture_stop(now_ms,
                                              "adc_one_shot_fail_3");
            app_task_battery_adc_guard_fault_event_handle(now_ms);
        }
        return true;
    }

    s_battery_adc_guard.consecutive_fail_count = 0U;
    if (elapsed_us > (uint32_t)BATTERY_ADC_GUARD_SLOW_ONE_SHOT_US)
    {
        s_battery_adc_guard.slow_one_shot_count++;
    }

    s_battery_adc_guard.last_sample_ms = now_ms;
    s_battery_adc_guard.window_raw[s_battery_adc_guard.valid_count] = sample.raw;
    s_battery_adc_guard.valid_count++;
    s_battery_adc_guard.valid_sample_count++;
    s_battery_adc_guard.capture_raw_sum += (uint64_t)sample.raw;
    if (!s_battery_adc_guard.capture_raw_valid)
    {
        s_battery_adc_guard.capture_raw_min = sample.raw;
        s_battery_adc_guard.capture_raw_max = sample.raw;
        s_battery_adc_guard.capture_raw_valid = true;
    }
    else
    {
        if (sample.raw < s_battery_adc_guard.capture_raw_min)
        {
            s_battery_adc_guard.capture_raw_min = sample.raw;
        }
        if (sample.raw > s_battery_adc_guard.capture_raw_max)
        {
            s_battery_adc_guard.capture_raw_max = sample.raw;
        }
    }

    return true;
}

void battery_adc_guard_init(void)
{
    battery_adc_guard_session_end();
    memset(&s_battery_adc_guard, 0, sizeof(s_battery_adc_guard));
    s_battery_adc_guard.threshold_mv = battery_adc_guard_threshold_mv();
}

bool battery_adc_guard_prepare_capture(uint32_t now_ms)
{
    battery_adc_guard_one_shot_t warmup;
    uint8_t i;

    battery_adc_guard_session_end();
    memset(&s_battery_adc_guard, 0, sizeof(s_battery_adc_guard));
    s_battery_adc_guard.threshold_mv = battery_adc_guard_threshold_mv();
    s_battery_adc_guard.last_sample_ms = now_ms;

    battery_adc_runtime_cancel_pending_attempt(now_ms);
    if (!battery_adc_guard_session_begin())
    {
        return false;
    }

    for (i = 0U; i < (uint8_t)BATTERY_ADC_GUARD_WARMUP_COUNT; i++)
    {
        if (!battery_adc_guard_session_read(&warmup) || !warmup.valid)
        {
            s_battery_adc_guard.one_shot_fail_count++;
            battery_adc_guard_session_end();
            return false;
        }
        s_battery_adc_guard.warmup_count++;
    }

    s_battery_adc_guard.session_prepared = true;
    return true;
}

void battery_adc_guard_on_capture_start(uint32_t now_ms)
{
    if (!s_battery_adc_guard.session_prepared)
    {
        return;
    }

    s_battery_adc_guard.armed = true;
    s_battery_adc_guard.last_sample_ms = now_ms;
    s_battery_adc_guard.next_due_ms =
        now_ms + (uint32_t)BATTERY_ADC_GUARD_COLLECT_OFFSET_MS;
    s_battery_adc_guard.window_end_ms =
        now_ms + (uint32_t)BATTERY_ADC_GUARD_PERIOD_MS;
}

void battery_adc_guard_on_capture_stop(uint32_t now_ms, const char *reason)
{
    bool active =
        s_battery_adc_guard.armed ||
        s_battery_adc_guard.session_prepared ||
        s_battery_adc_guard.collecting ||
        (s_battery_adc_guard.poll_count != 0U) ||
        (s_battery_adc_guard.due_count != 0U) ||
        (s_battery_adc_guard.sample_count != 0U) ||
        (s_battery_adc_guard.invalid_count != 0U) ||
        (s_battery_adc_guard.blocker_b_critical_count != 0U) ||
        (s_battery_adc_guard.blocker_spi_count != 0U) ||
        (s_battery_adc_guard.blocker_fifo_count != 0U) ||
        (s_battery_adc_guard.blocker_raw_count != 0U) ||
        (s_battery_adc_guard.blocker_record_count != 0U) ||
        (s_battery_adc_guard.blocker_replay_count != 0U) ||
        (s_battery_adc_guard.blocker_flash_count != 0U) ||
        (s_battery_adc_guard.blocker_ble_count != 0U) ||
        (s_battery_adc_guard.blocker_adc_count != 0U) ||
        (s_battery_adc_guard.blocker_state_count != 0U) ||
        (s_battery_adc_guard.blocker_budget_count != 0U);

    battery_adc_guard_session_end();
    if (active && !s_battery_adc_guard.capture_stats_pending)
    {
        battery_adc_guard_capture_stats_t *stats =
            &s_battery_adc_guard.capture_stats;

        memset(stats, 0, sizeof(*stats));
        stats->attempt_count = s_battery_adc_guard.sample_count;
        stats->valid_count = s_battery_adc_guard.valid_sample_count;
        stats->fail_count = s_battery_adc_guard.one_shot_fail_count;
        stats->valid_windows = s_battery_adc_guard.eval_count;
        stats->invalid_windows = s_battery_adc_guard.invalid_count;
        stats->final_valid = (s_battery_adc_guard.eval_count != 0U);
        stats->final_raw = s_battery_adc_guard.last_eval_raw;
        stats->final_mv = s_battery_adc_guard.last_mv;
        stats->final_percent = s_battery_adc_guard.last_percent;
        stats->read_total_us = s_battery_adc_guard.read_total_us;
        stats->read_max_us = s_battery_adc_guard.max_one_shot_us;
        stats->raw_valid = s_battery_adc_guard.capture_raw_valid;
        if (stats->raw_valid)
        {
            stats->raw_min = s_battery_adc_guard.capture_raw_min;
            stats->raw_max = s_battery_adc_guard.capture_raw_max;
            stats->raw_avg = (uint16_t)(
                (s_battery_adc_guard.capture_raw_sum +
                 ((uint64_t)stats->valid_count / 2ULL)) /
                (uint64_t)stats->valid_count);
        }
        s_battery_adc_guard.capture_stats_pending = true;
    }

    s_battery_adc_guard.armed = false;
    s_battery_adc_guard.session_prepared = false;
    s_battery_adc_guard.report_pending = false;
    battery_adc_guard_reset_collection();
    s_battery_adc_guard.block_counted = false;
    s_battery_adc_guard.last_block_reason = BATTERY_ADC_GUARD_BLOCK_NONE;
    battery_adc_guard_clear_counters();
    (void)now_ms;
    (void)reason;
}

bool battery_adc_guard_poll(uint32_t now_ms,
                            battery_adc_guard_block_reason_t block_reason)
{
    bool due;

    s_battery_adc_guard.last_check_ms = now_ms;
    s_battery_adc_guard.poll_count++;

    if (!s_battery_adc_guard.armed || s_battery_adc_guard.low_latched)
    {
        return false;
    }
    if ((block_reason == BATTERY_ADC_GUARD_BLOCK_STOP) ||
        (block_reason == BATTERY_ADC_GUARD_BLOCK_CAPTURE) ||
        (block_reason == BATTERY_ADC_GUARD_BLOCK_STATE))
    {
        battery_adc_guard_note_block(block_reason);
        return false;
    }

    /* Close expired windows even under export pressure. Never sample a missed
     * window or invent a healthy result. At most two invalid windows can stop
     * capture, so this loop is bounded even after a long scheduling gap. */
    while (battery_adc_guard_time_reached(now_ms, s_battery_adc_guard.window_end_ms))
    {
        battery_adc_guard_eval_complete(now_ms);
        if (!s_battery_adc_guard.armed)
        {
            return false;
        }
        s_battery_adc_guard.window_end_ms += (uint32_t)BATTERY_ADC_GUARD_PERIOD_MS;
        s_battery_adc_guard.next_due_ms = s_battery_adc_guard.window_end_ms -
            ((uint32_t)BATTERY_ADC_GUARD_PERIOD_MS -
             (uint32_t)BATTERY_ADC_GUARD_COLLECT_OFFSET_MS);
    }
    due = battery_adc_guard_time_reached(now_ms, s_battery_adc_guard.next_due_ms);
    if (!due || (s_battery_adc_guard.valid_count >= BATTERY_ADC_GUARD_SAMPLE_COUNT))
    {
        s_battery_adc_guard.block_counted = false;
        s_battery_adc_guard.last_block_reason = BATTERY_ADC_GUARD_BLOCK_NONE;
        return false;
    }

    if (!s_battery_adc_guard.collecting && due)
    {
        s_battery_adc_guard.due_count++;
        s_battery_adc_guard.last_due_ms = now_ms;
    }

    if (block_reason != BATTERY_ADC_GUARD_BLOCK_NONE)
    {
        battery_adc_guard_note_block(block_reason);
        return false;
    }

    s_battery_adc_guard.block_counted = false;
    s_battery_adc_guard.last_block_reason = BATTERY_ADC_GUARD_BLOCK_NONE;

    if (!s_battery_adc_guard.collecting)
    {
        battery_adc_guard_start_collection();
    }

    return battery_adc_guard_sample_one(now_ms);
}

bool battery_adc_guard_is_latched(void)
{
    return s_battery_adc_guard.low_latched;
}

bool battery_adc_guard_take_report(battery_adc_guard_report_t *report)
{
    if ((report == NULL) || !s_battery_adc_guard.report_pending)
    {
        return false;
    }

    *report = s_battery_adc_guard.report;
    s_battery_adc_guard.report_pending = false;
    return true;
}

bool battery_adc_guard_take_capture_stats(
    battery_adc_guard_capture_stats_t *stats)
{
    if ((stats == NULL) || !s_battery_adc_guard.capture_stats_pending)
    {
        return false;
    }

    *stats = s_battery_adc_guard.capture_stats;
    s_battery_adc_guard.capture_stats_pending = false;
    return true;
}

void battery_adc_guard_reset_latch(void)
{
    s_battery_adc_guard.low_latched = false;
    s_battery_adc_guard.low_count = 0U;
}

#endif
