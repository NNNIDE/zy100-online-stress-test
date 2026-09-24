#include "zy100_edge_lite.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../app_flags.h"

#define ZY100_EDGE_LITE_SAMPLE_RATE_HZ        ZY100_EDGE_SAMPLE_RATE_HZ
#define ZY100_EDGE_LITE_TRIGGER_THRESHOLD     7433U
#define ZY100_EDGE_LITE_QUIET_THRESHOLD       3122U
#define ZY100_EDGE_LITE_CHUNK_SAMPLES         16U
#define ZY100_EDGE_LITE_SUMMARY_PERIOD        800U
#define ZY100_EDGE_LITE_PRE_WINDOW_SAMPLES    ((ZY100_EDGE_SAMPLE_RATE_HZ * ZY100_EDGE_RAW_PRE_MS) / 1000U)
#define ZY100_EDGE_LITE_POST_MIN_SAMPLES      ((ZY100_EDGE_SAMPLE_RATE_HZ * ZY100_EDGE_RAW_POST_MS) / 1000U)
#define ZY100_EDGE_LITE_MAX_EVENT_SAMPLES     4000U
#define ZY100_EDGE_LITE_COOLDOWN_SAMPLES      800U
#define ZY100_EDGE_LITE_QUIET_CHUNKS          8U
#define ZY100_EDGE_LITE_SHAPE_BINS            ZY100_EDGE_LITE_SHAPE_BIN_COUNT
#define ZY100_EDGE_LITE_CLIP_ABS_THRESHOLD    32760U
#define ZY100_EDGE_LITE_RAW_FIRST_EVENTS      ZY100_EDGE_RAW_SAVE_FIRST_N
#define ZY100_EDGE_LITE_RAW_EVERY_EVENT       ZY100_EDGE_RAW_SAVE_EVERY_N
#define ZY100_EDGE_LITE_ALIGN_BYTES           4U

typedef enum
{
    ZY100_EDGE_LITE_STATE_IDLE = 0,
    ZY100_EDGE_LITE_STATE_RECORDING,
    ZY100_EDGE_LITE_STATE_COOLDOWN
} zy100_edge_lite_state_t;

typedef struct
{
    uint32_t energy;
    uint32_t gyro_l1;
    uint32_t acc_delta;
    uint32_t acc_l1;
    uint32_t clip_count;
    uint32_t sentinel_count;
    uint8_t saturated_axis_bitmap;
} zy100_edge_lite_sample_metrics_t;

typedef struct
{
    zy100_edge_lite_state_t state;
    bool initialized;
    bool workspace_ok;
    bool have_prev_acc;

    int16_t prev_ax;
    int16_t prev_ay;
    int16_t prev_az;

    uint32_t next_sample_index;
    uint32_t sample_count;
    uint32_t summary_count;
    uint32_t event_count;
    uint32_t raw_req_count;
    uint32_t rejected_count;
    uint32_t cooldown_drop_count;
    uint32_t max_event_forced_count;
    uint32_t clip_count;
    uint32_t sentinel_count;
    uint32_t partial_summary_discarded;
    uint32_t partial_event_discarded;

    uint32_t summary_sample_count;
    uint32_t summary_start_sample;
    uint32_t summary_start_t_us;
    uint32_t summary_energy_sum;
    uint32_t summary_peak_energy;
    uint32_t summary_gyro_peak;
    uint32_t summary_acc_delta_peak;
    uint32_t summary_clip_count;
    uint32_t summary_sentinel_count;
    uint32_t summary_candidate_count;

    uint32_t chunk_count;
    uint32_t chunk_start_sample;
    uint32_t chunk_peak_energy;

    uint32_t quiet_chunk_count;
    uint32_t cooldown_remaining;
    uint32_t event_id_next;
    uint32_t current_event_id;
    uint32_t event_start_sample;
    uint32_t event_trigger_sample;
    uint32_t event_end_sample;
    uint32_t event_sample_count;
    uint32_t event_peak_energy;
    uint32_t event_trigger_energy;
    uint32_t event_raw_requested;
    uint32_t event_raw_window_id;
    uint32_t event_clip_count;
    uint32_t event_sentinel_count;
    uint32_t event_quality_flags;
    uint32_t event_gyro_peak[3];
    uint32_t event_accel_peak[3];
    uint32_t event_gyro_axis_energy[3];
    uint32_t event_accel_axis_energy[3];
    uint32_t event_phase_energy[4];
    uint32_t shape_acc_sum[ZY100_EDGE_LITE_SHAPE_BINS];
    uint32_t shape_gyro_sum[ZY100_EDGE_LITE_SHAPE_BINS];
    uint16_t shape_count[ZY100_EDGE_LITE_SHAPE_BINS];

    zy100_edge_lite_output_t latest_output;
} zy100_edge_lite_ctx_t;

typedef char zy100_edge_lite_ctx_size_check[
    (sizeof(zy100_edge_lite_ctx_t) <= ZY100_EDGE_SHADOW_WORKSPACE_BYTES) ? 1 : -1
];

static zy100_edge_lite_ctx_t *s_edge_lite_ctx = NULL;
static uint32_t s_edge_lite_workspace_bytes = 0U;
static bool s_edge_lite_workspace_ok = false;

static void zy100_edge_lite_zero_bins(zy100_edge_lite_ctx_t *ctx)
{
    uint8_t i;

    for (i = 0U; i < ZY100_EDGE_LITE_SHAPE_BINS; i++)
    {
        ctx->shape_acc_sum[i] = 0U;
        ctx->shape_gyro_sum[i] = 0U;
        ctx->shape_count[i] = 0U;
    }
}

static uint32_t zy100_edge_lite_abs16_u32(int16_t value)
{
    int32_t wide = (int32_t)value;

    if (wide < 0)
    {
        wide = -wide;
    }
    return (uint32_t)wide;
}

static uint32_t zy100_edge_lite_absdiff16_u32(int16_t a, int16_t b)
{
    int32_t diff = (int32_t)a - (int32_t)b;

    if (diff < 0)
    {
        diff = -diff;
    }
    return (uint32_t)diff;
}

static uint32_t zy100_edge_lite_gyro_abs_for_energy(int16_t value, uint32_t *sentinel_count)
{
    if (value == (int16_t)-32768)
    {
        if (sentinel_count != NULL)
        {
            (*sentinel_count)++;
        }
        return 0U;
    }
    return zy100_edge_lite_abs16_u32(value);
}

static uint8_t zy100_edge_lite_clip_bit(bool is_gyro, int16_t value, uint8_t bit)
{
    if (is_gyro && (value == (int16_t)-32768))
    {
        return 0U;
    }
    return (zy100_edge_lite_abs16_u32(value) >= ZY100_EDGE_LITE_CLIP_ABS_THRESHOLD) ?
           (uint8_t)(1U << bit) : 0U;
}

static void zy100_edge_lite_reset_chunk(zy100_edge_lite_ctx_t *ctx)
{
    ctx->chunk_count = 0U;
    ctx->chunk_start_sample = ctx->next_sample_index;
    ctx->chunk_peak_energy = 0U;
}

static void zy100_edge_lite_reset_event(zy100_edge_lite_ctx_t *ctx)
{
    uint8_t i;

    ctx->quiet_chunk_count = 0U;
    ctx->current_event_id = 0U;
    ctx->event_start_sample = 0U;
    ctx->event_trigger_sample = 0U;
    ctx->event_end_sample = 0U;
    ctx->event_sample_count = 0U;
    ctx->event_peak_energy = 0U;
    ctx->event_trigger_energy = 0U;
    ctx->event_raw_requested = 0U;
    ctx->event_raw_window_id = 0U;
    ctx->event_clip_count = 0U;
    ctx->event_sentinel_count = 0U;
    ctx->event_quality_flags = 0U;
    for (i = 0U; i < 3U; i++)
    {
        ctx->event_gyro_peak[i] = 0U;
        ctx->event_accel_peak[i] = 0U;
        ctx->event_gyro_axis_energy[i] = 0U;
        ctx->event_accel_axis_energy[i] = 0U;
    }
    for (i = 0U; i < 4U; i++)
    {
        ctx->event_phase_energy[i] = 0U;
    }
    zy100_edge_lite_zero_bins(ctx);
}

static void zy100_edge_lite_reset_summary(zy100_edge_lite_ctx_t *ctx)
{
    ctx->summary_sample_count = 0U;
    ctx->summary_start_sample = 0U;
    ctx->summary_start_t_us = 0U;
    ctx->summary_energy_sum = 0U;
    ctx->summary_peak_energy = 0U;
    ctx->summary_gyro_peak = 0U;
    ctx->summary_acc_delta_peak = 0U;
    ctx->summary_clip_count = 0U;
    ctx->summary_sentinel_count = 0U;
    ctx->summary_candidate_count = 0U;
}

static void zy100_edge_lite_reset_runtime(zy100_edge_lite_ctx_t *ctx)
{
    ctx->state = ZY100_EDGE_LITE_STATE_IDLE;
    ctx->have_prev_acc = false;
    ctx->prev_ax = 0;
    ctx->prev_ay = 0;
    ctx->prev_az = 0;
    ctx->next_sample_index = 0U;
    ctx->sample_count = 0U;
    ctx->summary_count = 0U;
    ctx->event_count = 0U;
    ctx->raw_req_count = 0U;
    ctx->rejected_count = 0U;
    ctx->cooldown_drop_count = 0U;
    ctx->max_event_forced_count = 0U;
    ctx->clip_count = 0U;
    ctx->sentinel_count = 0U;
    ctx->partial_summary_discarded = 0U;
    ctx->partial_event_discarded = 0U;
    zy100_edge_lite_reset_summary(ctx);
    ctx->cooldown_remaining = 0U;
    ctx->event_id_next = 1U;
    memset(&ctx->latest_output, 0, sizeof(ctx->latest_output));
    zy100_edge_lite_reset_chunk(ctx);
    zy100_edge_lite_reset_event(ctx);
}

static void zy100_edge_lite_build_metrics(zy100_edge_lite_ctx_t *ctx,
                                          const zy100_edge_shadow_sample_t *sample,
                                          zy100_edge_lite_sample_metrics_t *metrics)
{
    uint32_t gx_abs;
    uint32_t gy_abs;
    uint32_t gz_abs;
    uint32_t ax_abs;
    uint32_t ay_abs;
    uint32_t az_abs;
    uint8_t clip_bitmap;

    memset(metrics, 0, sizeof(*metrics));
    gx_abs = zy100_edge_lite_gyro_abs_for_energy(sample->gyro_x_raw,
                                                 &metrics->sentinel_count);
    gy_abs = zy100_edge_lite_gyro_abs_for_energy(sample->gyro_y_raw,
                                                 &metrics->sentinel_count);
    gz_abs = zy100_edge_lite_gyro_abs_for_energy(sample->gyro_z_raw,
                                                 &metrics->sentinel_count);
    ax_abs = zy100_edge_lite_abs16_u32(sample->accel_x_raw);
    ay_abs = zy100_edge_lite_abs16_u32(sample->accel_y_raw);
    az_abs = zy100_edge_lite_abs16_u32(sample->accel_z_raw);

    if (ctx->have_prev_acc)
    {
        metrics->acc_delta =
            zy100_edge_lite_absdiff16_u32(sample->accel_x_raw, ctx->prev_ax) +
            zy100_edge_lite_absdiff16_u32(sample->accel_y_raw, ctx->prev_ay) +
            zy100_edge_lite_absdiff16_u32(sample->accel_z_raw, ctx->prev_az);
    }

    clip_bitmap = 0U;
    clip_bitmap |= zy100_edge_lite_clip_bit(false, sample->accel_x_raw, 0U);
    clip_bitmap |= zy100_edge_lite_clip_bit(false, sample->accel_y_raw, 1U);
    clip_bitmap |= zy100_edge_lite_clip_bit(false, sample->accel_z_raw, 2U);
    clip_bitmap |= zy100_edge_lite_clip_bit(true, sample->gyro_x_raw, 3U);
    clip_bitmap |= zy100_edge_lite_clip_bit(true, sample->gyro_y_raw, 4U);
    clip_bitmap |= zy100_edge_lite_clip_bit(true, sample->gyro_z_raw, 5U);

    metrics->gyro_l1 = gx_abs + gy_abs + gz_abs;
    metrics->acc_l1 = ax_abs + ay_abs + az_abs;
    metrics->energy = metrics->gyro_l1 + metrics->acc_delta;
    metrics->saturated_axis_bitmap = clip_bitmap;
    metrics->clip_count = ((clip_bitmap & (1U << 0)) != 0U ? 1U : 0U) +
                          ((clip_bitmap & (1U << 1)) != 0U ? 1U : 0U) +
                          ((clip_bitmap & (1U << 2)) != 0U ? 1U : 0U) +
                          ((clip_bitmap & (1U << 3)) != 0U ? 1U : 0U) +
                          ((clip_bitmap & (1U << 4)) != 0U ? 1U : 0U) +
                          ((clip_bitmap & (1U << 5)) != 0U ? 1U : 0U);
}

static bool zy100_edge_lite_raw_request_policy(const zy100_edge_lite_ctx_t *ctx);

static uint32_t zy100_edge_lite_sample_t_us(uint32_t sample_index)
{
    if (sample_index == 0U)
    {
        return 0U;
    }
    return (uint32_t)(((uint64_t)(sample_index - 1U) * 1000000ULL) /
                      (uint64_t)ZY100_EDGE_LITE_SAMPLE_RATE_HZ);
}

static uint16_t zy100_edge_lite_sat_u16(uint32_t value)
{
    return (value > 0xFFFFU) ? 0xFFFFU : (uint16_t)value;
}

static void zy100_edge_lite_emit_summary(zy100_edge_lite_ctx_t *ctx,
                                         uint32_t end_sample,
                                         zy100_edge_lite_output_t *out)
{
    zy100_edge_lite_summary_t *summary;

    if ((ctx == NULL) || (out == NULL) || (ctx->summary_sample_count == 0U))
    {
        return;
    }

    ctx->summary_count++;
    out->flags |= ZY100_EDGE_LITE_OUT_SUMMARY;
    out->summary_out = 1U;
    summary = &out->summary;
    memset(summary, 0, sizeof(*summary));
    summary->summary_id = ctx->summary_count;
    summary->start_sample = ctx->summary_start_sample;
    summary->end_sample = end_sample;
    summary->start_t_us = ctx->summary_start_t_us;
    summary->duration_samples = ctx->summary_sample_count;
    summary->energy_sum = ctx->summary_energy_sum;
    summary->energy_peak = ctx->summary_peak_energy;
    summary->gyro_peak = ctx->summary_gyro_peak;
    summary->acc_delta_peak = ctx->summary_acc_delta_peak;
    summary->clip_count = ctx->summary_clip_count;
    summary->sentinel_count = ctx->summary_sentinel_count;
    summary->candidate_count = ctx->summary_candidate_count;

    ctx->summary_sample_count = 0U;
    ctx->summary_start_sample = 0U;
    ctx->summary_start_t_us = 0U;
    ctx->summary_energy_sum = 0U;
    ctx->summary_peak_energy = 0U;
    ctx->summary_gyro_peak = 0U;
    ctx->summary_acc_delta_peak = 0U;
    ctx->summary_clip_count = 0U;
    ctx->summary_sentinel_count = 0U;
    ctx->summary_candidate_count = 0U;
}

static void zy100_edge_lite_fill_event(const zy100_edge_lite_ctx_t *ctx,
                                       zy100_edge_lite_event_t *event)
{
    uint8_t i;

    if ((ctx == NULL) || (event == NULL))
    {
        return;
    }

    memset(event, 0, sizeof(*event));
    event->event_id = ctx->current_event_id;
    event->start_sample = ctx->event_start_sample;
    event->trigger_sample = ctx->event_trigger_sample;
    event->end_sample = ctx->event_end_sample;
    event->duration_samples =
        (ctx->event_end_sample >= ctx->event_start_sample) ?
        (ctx->event_end_sample - ctx->event_start_sample + 1U) : 0U;
    event->trigger_energy = ctx->event_trigger_energy;
    event->peak_energy = ctx->event_peak_energy;
    event->clip_count = ctx->event_clip_count;
    event->sentinel_count = ctx->event_sentinel_count;
    event->quality_flags = ctx->event_quality_flags;
    event->raw_requested = ctx->event_raw_requested;
    event->raw_window_id = ctx->event_raw_window_id;
    for (i = 0U; i < 3U; i++)
    {
        event->gyro_peak[i] = ctx->event_gyro_peak[i];
        event->acc_peak[i] = ctx->event_accel_peak[i];
        event->gyro_axis_energy[i] = ctx->event_gyro_axis_energy[i];
        event->acc_axis_energy[i] = ctx->event_accel_axis_energy[i];
    }
    for (i = 0U; i < 4U; i++)
    {
        event->phase_energy[i] = ctx->event_phase_energy[i];
    }
    for (i = 0U; i < ZY100_EDGE_LITE_SHAPE_BINS; i++)
    {
        uint32_t acc = ctx->shape_acc_sum[i];
        uint32_t gyro = ctx->shape_gyro_sum[i];

        if (ctx->shape_count[i] != 0U)
        {
            acc /= ctx->shape_count[i];
            gyro /= ctx->shape_count[i];
        }
        event->shape_acc[i] = zy100_edge_lite_sat_u16(acc);
        event->shape_gyro[i] = zy100_edge_lite_sat_u16(gyro);
    }
}

static void zy100_edge_lite_update_summary(zy100_edge_lite_ctx_t *ctx,
                                           uint32_t current_sample,
                                           const zy100_edge_lite_sample_metrics_t *metrics,
                                           zy100_edge_lite_output_t *out)
{
    if (ctx->summary_sample_count == 0U)
    {
        ctx->summary_start_sample = current_sample;
        ctx->summary_start_t_us = zy100_edge_lite_sample_t_us(current_sample);
    }
    ctx->summary_sample_count++;
    ctx->summary_energy_sum += metrics->energy;
    if (metrics->energy > ctx->summary_peak_energy)
    {
        ctx->summary_peak_energy = metrics->energy;
    }
    if (metrics->gyro_l1 > ctx->summary_gyro_peak)
    {
        ctx->summary_gyro_peak = metrics->gyro_l1;
    }
    if (metrics->acc_delta > ctx->summary_acc_delta_peak)
    {
        ctx->summary_acc_delta_peak = metrics->acc_delta;
    }
    ctx->summary_clip_count += metrics->clip_count;
    ctx->summary_sentinel_count += metrics->sentinel_count;

    if (ctx->summary_sample_count >= ZY100_EDGE_LITE_SUMMARY_PERIOD)
    {
        zy100_edge_lite_emit_summary(ctx, current_sample, out);
    }
}

static void zy100_edge_lite_start_event(zy100_edge_lite_ctx_t *ctx,
                                        uint32_t trigger_sample,
                                        uint32_t current_sample,
                                        uint32_t trigger_energy,
                                        zy100_edge_lite_output_t *out)
{
    bool raw_req;

    zy100_edge_lite_reset_event(ctx);
    ctx->current_event_id = ctx->event_id_next++;
    ctx->event_trigger_sample = trigger_sample;
    ctx->event_start_sample =
        (trigger_sample > ZY100_EDGE_LITE_PRE_WINDOW_SAMPLES) ?
        (trigger_sample - ZY100_EDGE_LITE_PRE_WINDOW_SAMPLES) : 1U;
    ctx->event_end_sample = current_sample;
    ctx->event_trigger_energy = trigger_energy;
    ctx->event_peak_energy = trigger_energy;
    ctx->event_quality_flags = 0U;
    ctx->summary_candidate_count++;
    ctx->state = ZY100_EDGE_LITE_STATE_RECORDING;

    raw_req = zy100_edge_lite_raw_request_policy(ctx);
    if (raw_req)
    {
        ctx->raw_req_count++;
        ctx->event_raw_requested = 1U;
        ctx->event_raw_window_id = ctx->current_event_id;
        if (out != NULL)
        {
            out->flags |= (ZY100_EDGE_LITE_OUT_RAW_REQ | ZY100_EDGE_LITE_OUT_RAW_START);
            out->raw_req_out = 1U;
            out->raw_start_out = 1U;
            out->event_id = ctx->current_event_id;
            out->start_sample = ctx->event_start_sample;
            out->trigger_sample = ctx->event_trigger_sample;
            out->end_sample = ctx->event_end_sample;
            out->peak_energy = ctx->event_peak_energy;
            out->trigger_energy = ctx->event_trigger_energy;
            out->quality_flags = ctx->event_quality_flags;
            zy100_edge_lite_fill_event(ctx, &out->event);
        }
    }
}

static void zy100_edge_lite_fold_event_sample(zy100_edge_lite_ctx_t *ctx,
                                              uint32_t current_sample,
                                              const zy100_edge_shadow_sample_t *sample,
                                              const zy100_edge_lite_sample_metrics_t *metrics)
{
    uint32_t bin;
    uint32_t phase;
    uint32_t sample_pos;
    uint32_t ax_abs;
    uint32_t ay_abs;
    uint32_t az_abs;
    uint32_t gx_abs;
    uint32_t gy_abs;
    uint32_t gz_abs;
    uint32_t ignored_sentinel_count = 0U;

    sample_pos = ctx->event_sample_count;
    ctx->event_sample_count++;
    ctx->event_end_sample = current_sample;
    if (metrics->energy > ctx->event_peak_energy)
    {
        ctx->event_peak_energy = metrics->energy;
    }
    ctx->event_clip_count += metrics->clip_count;
    ctx->event_sentinel_count += metrics->sentinel_count;
    if (metrics->clip_count != 0U)
    {
        ctx->event_quality_flags |= ZY100_EDGE_LITE_QUALITY_HAS_CLIP;
    }
    if (metrics->sentinel_count != 0U)
    {
        ctx->event_quality_flags |= ZY100_EDGE_LITE_QUALITY_HAS_SENTINEL;
    }

    ax_abs = zy100_edge_lite_abs16_u32(sample->accel_x_raw);
    ay_abs = zy100_edge_lite_abs16_u32(sample->accel_y_raw);
    az_abs = zy100_edge_lite_abs16_u32(sample->accel_z_raw);
    gx_abs = zy100_edge_lite_gyro_abs_for_energy(sample->gyro_x_raw, &ignored_sentinel_count);
    gy_abs = zy100_edge_lite_gyro_abs_for_energy(sample->gyro_y_raw, &ignored_sentinel_count);
    gz_abs = zy100_edge_lite_gyro_abs_for_energy(sample->gyro_z_raw, &ignored_sentinel_count);

    if (ax_abs > ctx->event_accel_peak[0])
    {
        ctx->event_accel_peak[0] = ax_abs;
    }
    if (ay_abs > ctx->event_accel_peak[1])
    {
        ctx->event_accel_peak[1] = ay_abs;
    }
    if (az_abs > ctx->event_accel_peak[2])
    {
        ctx->event_accel_peak[2] = az_abs;
    }
    if (gx_abs > ctx->event_gyro_peak[0])
    {
        ctx->event_gyro_peak[0] = gx_abs;
    }
    if (gy_abs > ctx->event_gyro_peak[1])
    {
        ctx->event_gyro_peak[1] = gy_abs;
    }
    if (gz_abs > ctx->event_gyro_peak[2])
    {
        ctx->event_gyro_peak[2] = gz_abs;
    }

    ctx->event_accel_axis_energy[0] += ax_abs;
    ctx->event_accel_axis_energy[1] += ay_abs;
    ctx->event_accel_axis_energy[2] += az_abs;
    ctx->event_gyro_axis_energy[0] += gx_abs;
    ctx->event_gyro_axis_energy[1] += gy_abs;
    ctx->event_gyro_axis_energy[2] += gz_abs;

    bin = (sample_pos * ZY100_EDGE_LITE_SHAPE_BINS) / ZY100_EDGE_LITE_MAX_EVENT_SAMPLES;
    if (bin >= ZY100_EDGE_LITE_SHAPE_BINS)
    {
        bin = ZY100_EDGE_LITE_SHAPE_BINS - 1U;
    }
    ctx->shape_acc_sum[bin] += metrics->acc_l1;
    ctx->shape_gyro_sum[bin] += metrics->gyro_l1;
    if (ctx->shape_count[bin] < 0xFFFFU)
    {
        ctx->shape_count[bin]++;
    }

    phase = (sample_pos * 4U) / ZY100_EDGE_LITE_MAX_EVENT_SAMPLES;
    if (phase > 3U)
    {
        phase = 3U;
    }
    ctx->event_phase_energy[phase] += metrics->energy;
}

static bool zy100_edge_lite_raw_request_policy(const zy100_edge_lite_ctx_t *ctx)
{
    uint32_t abnormal_threshold;

    abnormal_threshold = (ZY100_EDGE_LITE_TRIGGER_THRESHOLD * 5U) / 2U;
    if (ctx->current_event_id <= ZY100_EDGE_LITE_RAW_FIRST_EVENTS)
    {
        return true;
    }
    if ((ZY100_EDGE_LITE_RAW_EVERY_EVENT != 0U) &&
        ((ctx->current_event_id % ZY100_EDGE_LITE_RAW_EVERY_EVENT) == 0U))
    {
        return true;
    }
    if ((ctx->event_quality_flags & (ZY100_EDGE_LITE_QUALITY_HAS_CLIP |
                                     ZY100_EDGE_LITE_QUALITY_HAS_SENTINEL |
                                     ZY100_EDGE_LITE_QUALITY_MAX_EVENT_FORCED)) != 0U)
    {
        return true;
    }
    return (ctx->event_peak_energy >= abnormal_threshold);
}

static void zy100_edge_lite_close_event(zy100_edge_lite_ctx_t *ctx,
                                        zy100_edge_lite_output_t *out,
                                        bool max_event_forced)
{
    if (max_event_forced)
    {
        ctx->event_quality_flags |= ZY100_EDGE_LITE_QUALITY_MAX_EVENT_FORCED;
        ctx->max_event_forced_count++;
    }
    if (ctx->event_peak_energy >= ((ZY100_EDGE_LITE_TRIGGER_THRESHOLD * 5U) / 2U))
    {
        ctx->event_quality_flags |= ZY100_EDGE_LITE_QUALITY_ABNORMAL;
    }

    ctx->event_count++;
    out->flags |= ZY100_EDGE_LITE_OUT_EVENT;
    out->event_out = 1U;
    out->event_id = ctx->current_event_id;
    out->start_sample = ctx->event_start_sample;
    out->trigger_sample = ctx->event_trigger_sample;
    out->end_sample = ctx->event_end_sample;
    out->peak_energy = ctx->event_peak_energy;
    out->trigger_energy = ctx->event_trigger_energy;
    out->quality_flags = ctx->event_quality_flags;
    zy100_edge_lite_fill_event(ctx, &out->event);
    ctx->latest_output = *out;

    ctx->state = ZY100_EDGE_LITE_STATE_COOLDOWN;
    ctx->cooldown_remaining = ZY100_EDGE_LITE_COOLDOWN_SAMPLES;
    zy100_edge_lite_reset_event(ctx);
}

static void zy100_edge_lite_process_completed_chunk(zy100_edge_lite_ctx_t *ctx,
                                                   uint32_t current_sample,
                                                   zy100_edge_lite_output_t *out)
{
    if ((ctx->state == ZY100_EDGE_LITE_STATE_IDLE) &&
        (ctx->chunk_peak_energy >= ZY100_EDGE_LITE_TRIGGER_THRESHOLD))
    {
        zy100_edge_lite_start_event(ctx,
                                    ctx->chunk_start_sample,
                                    current_sample,
                                    ctx->chunk_peak_energy,
                                    out);
    }

    if (ctx->state == ZY100_EDGE_LITE_STATE_RECORDING)
    {
        uint32_t since_trigger = current_sample - ctx->event_trigger_sample;
        bool should_close = false;
        bool max_event_forced = false;

        if ((since_trigger >= ZY100_EDGE_LITE_POST_MIN_SAMPLES) &&
            (ctx->chunk_peak_energy < ZY100_EDGE_LITE_QUIET_THRESHOLD))
        {
            if (ctx->quiet_chunk_count < 0xFFFFFFFFU)
            {
                ctx->quiet_chunk_count++;
            }
            if (ctx->quiet_chunk_count >= ZY100_EDGE_LITE_QUIET_CHUNKS)
            {
                should_close = true;
            }
        }
        else if (ctx->chunk_peak_energy >= ZY100_EDGE_LITE_QUIET_THRESHOLD)
        {
            ctx->quiet_chunk_count = 0U;
        }

        if (since_trigger >= ZY100_EDGE_LITE_MAX_EVENT_SAMPLES)
        {
            should_close = true;
            max_event_forced = true;
        }

        if (should_close)
        {
            zy100_edge_lite_close_event(ctx, out, max_event_forced);
        }
    }
    zy100_edge_lite_reset_chunk(ctx);
}

uint32_t zy100_edge_lite_context_bytes(void)
{
    return (uint32_t)sizeof(zy100_edge_lite_ctx_t);
}

bool zy100_edge_lite_workspace_ok(void)
{
    return s_edge_lite_workspace_ok;
}

bool zy100_edge_lite_init(void *workspace, uint32_t workspace_bytes)
{
    uint8_t *base;
    uintptr_t addr;
    uintptr_t aligned_addr;
    uint32_t offset;

    s_edge_lite_workspace_bytes = workspace_bytes;
    s_edge_lite_workspace_ok = false;
    s_edge_lite_ctx = NULL;

    if ((workspace == NULL) ||
        (workspace_bytes < (uint32_t)sizeof(zy100_edge_lite_ctx_t)))
    {
        return false;
    }

    base = (uint8_t *)workspace;
    addr = (uintptr_t)base;
    aligned_addr = (addr + (uintptr_t)(ZY100_EDGE_LITE_ALIGN_BYTES - 1U)) &
                   ~((uintptr_t)(ZY100_EDGE_LITE_ALIGN_BYTES - 1U));
    offset = (uint32_t)(aligned_addr - addr);
    if ((workspace_bytes < offset) ||
        ((workspace_bytes - offset) < (uint32_t)sizeof(zy100_edge_lite_ctx_t)))
    {
        return false;
    }

    s_edge_lite_ctx = (zy100_edge_lite_ctx_t *)(base + offset);
    memset(s_edge_lite_ctx, 0, sizeof(*s_edge_lite_ctx));
    s_edge_lite_ctx->workspace_ok = true;
    s_edge_lite_ctx->initialized = true;
    s_edge_lite_workspace_ok = true;
    zy100_edge_lite_reset_runtime(s_edge_lite_ctx);
    return true;
}

void zy100_edge_lite_capture_start(void)
{
    if ((s_edge_lite_ctx == NULL) || !s_edge_lite_ctx->initialized)
    {
        return;
    }
    zy100_edge_lite_reset_runtime(s_edge_lite_ctx);
    s_edge_lite_ctx->initialized = true;
    s_edge_lite_ctx->workspace_ok = s_edge_lite_workspace_ok;
}

bool zy100_edge_lite_push_sample(const zy100_edge_shadow_sample_t *sample,
                                 zy100_edge_lite_output_t *out)
{
    zy100_edge_lite_ctx_t *ctx = s_edge_lite_ctx;
    zy100_edge_lite_sample_metrics_t metrics;
    uint32_t current_sample;

    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
    if ((ctx == NULL) || !ctx->initialized || !ctx->workspace_ok ||
        (sample == NULL) || (out == NULL))
    {
        if (ctx != NULL)
        {
            ctx->rejected_count++;
        }
        return false;
    }

    current_sample = (sample->sample_index != 0U) ?
                     sample->sample_index : (ctx->next_sample_index + 1U);
    zy100_edge_lite_build_metrics(ctx, sample, &metrics);
    ctx->sample_count++;
    ctx->clip_count += metrics.clip_count;
    ctx->sentinel_count += metrics.sentinel_count;
    zy100_edge_lite_update_summary(ctx, current_sample, &metrics, out);

    if (ctx->state == ZY100_EDGE_LITE_STATE_COOLDOWN)
    {
        ctx->cooldown_drop_count++;
        if (ctx->cooldown_remaining > 0U)
        {
            ctx->cooldown_remaining--;
        }
        if (ctx->cooldown_remaining == 0U)
        {
            ctx->state = ZY100_EDGE_LITE_STATE_IDLE;
        }
    }

    if (ctx->state == ZY100_EDGE_LITE_STATE_RECORDING)
    {
        zy100_edge_lite_fold_event_sample(ctx, current_sample, sample, &metrics);
    }

    if (ctx->chunk_count == 0U)
    {
        ctx->chunk_start_sample = current_sample;
    }
    ctx->chunk_count++;
    if (metrics.energy > ctx->chunk_peak_energy)
    {
        ctx->chunk_peak_energy = metrics.energy;
    }
    if (ctx->chunk_count >= ZY100_EDGE_LITE_CHUNK_SAMPLES)
    {
        zy100_edge_lite_process_completed_chunk(ctx, current_sample, out);
    }

    ctx->prev_ax = sample->accel_x_raw;
    ctx->prev_ay = sample->accel_y_raw;
    ctx->prev_az = sample->accel_z_raw;
    ctx->have_prev_acc = true;
    ctx->next_sample_index = current_sample;
    if (out->flags != ZY100_EDGE_LITE_OUT_NONE)
    {
        ctx->latest_output = *out;
    }
    return true;
}

bool zy100_edge_lite_capture_stop(zy100_edge_lite_stop_mode_t mode,
                                  zy100_edge_lite_output_t *out)
{
    zy100_edge_lite_ctx_t *ctx = s_edge_lite_ctx;
    zy100_edge_lite_output_t local_out;
    uint32_t end_sample;

    if ((ctx == NULL) || !ctx->initialized)
    {
        return false;
    }
    memset(&local_out, 0, sizeof(local_out));
    if (mode == ZY100_EDGE_LITE_STOP_DISCARD_PARTIAL)
    {
        if (ctx->summary_sample_count != 0U)
        {
            ctx->partial_summary_discarded++;
            zy100_edge_lite_reset_summary(ctx);
        }
        if ((ctx->state == ZY100_EDGE_LITE_STATE_RECORDING) &&
            (ctx->current_event_id != 0U))
        {
            ctx->partial_event_discarded++;
            zy100_edge_lite_reset_event(ctx);
        }
        ctx->state = ZY100_EDGE_LITE_STATE_IDLE;
        ctx->cooldown_remaining = 0U;
        zy100_edge_lite_reset_chunk(ctx);
        if (out != NULL)
        {
            *out = local_out;
        }
        return false;
    }

    if ((ctx->state == ZY100_EDGE_LITE_STATE_RECORDING) &&
        (ctx->current_event_id != 0U))
    {
        zy100_edge_lite_close_event(ctx, &local_out, true);
    }
    if (ctx->summary_sample_count != 0U)
    {
        end_sample = (ctx->next_sample_index != 0U) ?
                     ctx->next_sample_index : ctx->summary_start_sample;
        zy100_edge_lite_emit_summary(ctx, end_sample, &local_out);
    }
    if (local_out.flags != ZY100_EDGE_LITE_OUT_NONE)
    {
        ctx->latest_output = local_out;
    }
    if (out != NULL)
    {
        *out = local_out;
    }
    return local_out.flags != ZY100_EDGE_LITE_OUT_NONE;
}

bool zy100_edge_lite_capture_end(zy100_edge_lite_output_t *out)
{
    return zy100_edge_lite_capture_stop(ZY100_EDGE_LITE_STOP_NORMAL_END, out);
}

void zy100_edge_lite_get_stats(zy100_edge_lite_stats_t *out)
{
    zy100_edge_lite_ctx_t *ctx = s_edge_lite_ctx;

    if (out == NULL)
    {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->trigger_threshold = ZY100_EDGE_LITE_TRIGGER_THRESHOLD;
    out->quiet_threshold = ZY100_EDGE_LITE_QUIET_THRESHOLD;
    out->chunk_samples = ZY100_EDGE_LITE_CHUNK_SAMPLES;
    out->summary_period_samples = ZY100_EDGE_LITE_SUMMARY_PERIOD;
    out->cooldown_samples = ZY100_EDGE_LITE_COOLDOWN_SAMPLES;
    out->ctx_bytes = (uint32_t)sizeof(zy100_edge_lite_ctx_t);
    out->workspace_bytes = s_edge_lite_workspace_bytes;
    out->workspace_ok = s_edge_lite_workspace_ok ? 1U : 0U;

    if (ctx == NULL)
    {
        return;
    }
    out->sample_count = ctx->sample_count;
    out->summary_count = ctx->summary_count;
    out->event_count = ctx->event_count;
    out->raw_req_count = ctx->raw_req_count;
    out->clip_count = ctx->clip_count;
    out->sentinel_count = ctx->sentinel_count;
    out->rejected_count = ctx->rejected_count;
    out->cooldown_drop_count = ctx->cooldown_drop_count;
    out->max_event_forced_count = ctx->max_event_forced_count;
    out->partial_summary_discarded = ctx->partial_summary_discarded;
    out->partial_event_discarded = ctx->partial_event_discarded;
}
