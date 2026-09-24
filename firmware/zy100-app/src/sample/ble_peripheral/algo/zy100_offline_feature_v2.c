#include "zy100_offline_feature_v2.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define OFFLINE_V2_WARMUP_SAMPLES              640U
#define OFFLINE_V2_WINDOW_PRE_SAMPLES          320U
#define OFFLINE_V2_WINDOW_POST_SAMPLES         480U
#define OFFLINE_V2_RAW_WAIT_CAPACITY           241U
#define OFFLINE_V2_NMS_PENDING_CAPACITY        201U
#define OFFLINE_V2_NMS_DEQUE_CAPACITY          201U
#define OFFLINE_V2_NMS_DECISION_DELAY_SAMPLES  \
    (ZY100_OFFLINE_V2_NMS_DISTANCE_SAMPLES - 1U + \
     OFFLINE_V2_WINDOW_POST_SAMPLES)
#define OFFLINE_V2_SIGNAL_COUNT                8U
#define OFFLINE_V2_SHAPE_BINS                  16U
#define OFFLINE_V2_SAMPLES_PER_BIN             50U
#define OFFLINE_V2_GYRO_SENTINEL               ((int16_t)-32768)
#define OFFLINE_V2_MIN_SCALE                   1.0e-12f
#define OFFLINE_V2_Q12_SCALE                   4096.0f
#define OFFLINE_V2_ACCEL_LSB_PER_G             2048.0
#define OFFLINE_V2_SHOCK_RATIO_SCALE           10000.0

typedef struct
{
    uint32_t sample_index;
    uint32_t mag2;
} offline_v2_peak_t;

struct zy100_offline_v2_algo
{
    zy100_offline_v2_imu_sample_t ring[ZY100_OFFLINE_V2_RING_SAMPLES];
    uint8_t gap_before[ZY100_OFFLINE_V2_RING_SAMPLES];
    offline_v2_peak_t raw_wait[OFFLINE_V2_RAW_WAIT_CAPACITY];
    offline_v2_peak_t nms_pending[OFFLINE_V2_NMS_PENDING_CAPACITY];
    offline_v2_peak_t nms_maximum[OFFLINE_V2_NMS_DEQUE_CAPACITY];
    uint32_t shock_mag2[ZY100_OFFLINE_V2_WINDOW_SAMPLES];
    float feature[ZY100_OFFLINE_V2_FEATURE_DIMENSIONS];
    zy100_offline_v2_event_sink_t sink;
    void *sink_context;
    zy100_offline_v2_time_us_t time_us;
    uint32_t sample_count;
    uint32_t raw_head;
    uint32_t raw_count;
    uint32_t nms_head;
    uint32_t nms_count;
    uint32_t maximum_head;
    uint32_t maximum_count;
    uint32_t local_peak_count;
    uint32_t legal_peak_count;
    uint32_t nms_selected_count;
    uint32_t shock_rejected_count;
    uint32_t rejected_window_count;
    uint32_t event_count;
    uint32_t detector_last_us;
    uint32_t detector_max_us;
    uint32_t shock_last_us;
    uint32_t shock_max_us;
    uint32_t feature_last_us;
    uint32_t feature_max_us;
    zy100_offline_v2_algo_status_t status;
};

typedef char offline_v2_raw_sample_size_check[
    (sizeof(zy100_offline_v2_imu_sample_t) == 12U) ? 1 : -1];
typedef char offline_v2_workspace_budget_check[
    (sizeof(struct zy100_offline_v2_algo) <=
     ZY100_OFFLINE_V2_ALGO_WORKSPACE_MAX) ? 1 : -1];

static uint32_t offline_v2_ring_position(uint32_t sample_index)
{
    return sample_index % ZY100_OFFLINE_V2_RING_SAMPLES;
}

static const zy100_offline_v2_imu_sample_t *offline_v2_ring_sample(
    const zy100_offline_v2_algo_t *algo,
    uint32_t sample_index)
{
    if ((algo == NULL) || (sample_index >= algo->sample_count) ||
        ((algo->sample_count - sample_index) > ZY100_OFFLINE_V2_RING_SAMPLES))
    {
        return NULL;
    }
    return &algo->ring[offline_v2_ring_position(sample_index)];
}

static bool offline_v2_is_sentinel(
    const zy100_offline_v2_imu_sample_t *sample)
{
    return (sample == NULL) ||
           (sample->axis[3] == OFFLINE_V2_GYRO_SENTINEL) ||
           (sample->axis[4] == OFFLINE_V2_GYRO_SENTINEL) ||
           (sample->axis[5] == OFFLINE_V2_GYRO_SENTINEL);
}

static uint32_t offline_v2_axis_mag2(const int16_t axis[3])
{
    int64_t x = (int64_t)axis[0];
    int64_t y = (int64_t)axis[1];
    int64_t z = (int64_t)axis[2];
    uint64_t sum = (uint64_t)(x * x);

    sum += (uint64_t)(y * y);
    sum += (uint64_t)(z * z);
    return (uint32_t)sum;
}

static uint32_t offline_v2_gyro_mag2(
    const zy100_offline_v2_imu_sample_t *sample)
{
    return offline_v2_axis_mag2(&sample->axis[3]);
}

static uint32_t offline_v2_raw_position(
    const zy100_offline_v2_algo_t *algo,
    uint32_t logical)
{
    return (algo->raw_head + logical) % OFFLINE_V2_RAW_WAIT_CAPACITY;
}

static uint32_t offline_v2_nms_position(
    const zy100_offline_v2_algo_t *algo,
    uint32_t logical)
{
    return (algo->nms_head + logical) % OFFLINE_V2_NMS_PENDING_CAPACITY;
}

static uint32_t offline_v2_maximum_position(
    const zy100_offline_v2_algo_t *algo,
    uint32_t logical)
{
    return (algo->maximum_head + logical) % OFFLINE_V2_NMS_DEQUE_CAPACITY;
}

static bool offline_v2_raw_enqueue(zy100_offline_v2_algo_t *algo,
                                   uint32_t sample_index,
                                   uint32_t mag2)
{
    uint32_t position;

    if (algo->raw_count >= OFFLINE_V2_RAW_WAIT_CAPACITY)
    {
        algo->status = ZY100_OFFLINE_V2_ALGO_QUEUE_OVERFLOW;
        return false;
    }
    position = offline_v2_raw_position(algo, algo->raw_count);
    algo->raw_wait[position].sample_index = sample_index;
    algo->raw_wait[position].mag2 = mag2;
    algo->raw_count++;
    algo->local_peak_count++;
    return true;
}

static bool offline_v2_nms_enqueue(zy100_offline_v2_algo_t *algo,
                                   const offline_v2_peak_t *peak)
{
    uint32_t position;
    offline_v2_peak_t *back;

    if ((algo->nms_count >= OFFLINE_V2_NMS_PENDING_CAPACITY) ||
        (algo->maximum_count >= OFFLINE_V2_NMS_DEQUE_CAPACITY))
    {
        algo->status = ZY100_OFFLINE_V2_ALGO_QUEUE_OVERFLOW;
        return false;
    }
    position = offline_v2_nms_position(algo, algo->nms_count);
    algo->nms_pending[position] = *peak;
    algo->nms_count++;

    while (algo->maximum_count != 0U)
    {
        position = offline_v2_maximum_position(
            algo, algo->maximum_count - 1U);
        back = &algo->nms_maximum[position];
        /* Equal magnitude stays: the earlier sample wins the tie. */
        if (back->mag2 >= peak->mag2)
        {
            break;
        }
        algo->maximum_count--;
    }
    position = offline_v2_maximum_position(algo, algo->maximum_count);
    algo->nms_maximum[position] = *peak;
    algo->maximum_count++;
    algo->legal_peak_count++;
    return true;
}

static bool offline_v2_gap_in_window(const zy100_offline_v2_algo_t *algo,
                                     uint32_t start,
                                     uint32_t end)
{
    uint32_t index = start + 1U;
    uint32_t position;

    if (index >= end)
    {
        return false;
    }
    position = offline_v2_ring_position(index);
    for (; index < end; index++)
    {
        if (algo->gap_before[position] != 0U)
        {
            return true;
        }
        position++;
        if (position == ZY100_OFFLINE_V2_RING_SAMPLES)
        {
            position = 0U;
        }
    }
    return false;
}

static bool offline_v2_window_valid(const zy100_offline_v2_algo_t *algo,
                                    uint32_t center)
{
    uint32_t start;
    uint32_t end;
    uint32_t index;

    if (center < OFFLINE_V2_WINDOW_PRE_SAMPLES)
    {
        return false;
    }
    start = center - OFFLINE_V2_WINDOW_PRE_SAMPLES;
    end = center + OFFLINE_V2_WINDOW_POST_SAMPLES;
    if ((end > algo->sample_count) ||
        offline_v2_gap_in_window(algo, start, end))
    {
        return false;
    }
    for (index = start; index < end; index++)
    {
        if (offline_v2_is_sentinel(offline_v2_ring_sample(algo, index)))
        {
            return false;
        }
    }
    return true;
}

static float offline_v2_signal_value(
    const zy100_offline_v2_imu_sample_t *sample,
    uint32_t signal)
{
    float x;
    float y;
    float z;
    float x2;
    float y2;
    float z2;
    float xy;

    if (signal < 6U)
    {
        return (float)sample->axis[signal];
    }
    if (signal == 6U)
    {
        x = (float)sample->axis[0];
        y = (float)sample->axis[1];
        z = (float)sample->axis[2];
    }
    else
    {
        x = (float)sample->axis[3];
        y = (float)sample->axis[4];
        z = (float)sample->axis[5];
    }
    x2 = x * x;
    y2 = y * y;
    z2 = z * z;
    xy = x2 + y2;
    return sqrtf(xy + z2);
}

static bool offline_v2_extract_feature(zy100_offline_v2_algo_t *algo,
                                       uint32_t center,
                                       bool *saturated_out)
{
    float sum[OFFLINE_V2_SIGNAL_COUNT] = {0.0f};
    float mean[OFFLINE_V2_SIGNAL_COUNT];
    float sqdev_sum[OFFLINE_V2_SIGNAL_COUNT] = {0.0f};
    float bin_sum[OFFLINE_V2_SIGNAL_COUNT][OFFLINE_V2_SHAPE_BINS];
    float scale;
    float value;
    float delta;
    float variance;
    float bin_mean;
    uint32_t start = center - OFFLINE_V2_WINDOW_PRE_SAMPLES;
    uint32_t sample_index;
    uint32_t signal;
    uint32_t bin;
    const zy100_offline_v2_imu_sample_t *sample;

    memset(bin_sum, 0, sizeof(bin_sum));
    for (sample_index = 0U;
         sample_index < ZY100_OFFLINE_V2_WINDOW_SAMPLES;
         sample_index++)
    {
        sample = offline_v2_ring_sample(algo, start + sample_index);
        if (sample == NULL)
        {
            algo->status = ZY100_OFFLINE_V2_ALGO_RING_OVERRUN;
            return false;
        }
        for (signal = 0U; signal < OFFLINE_V2_SIGNAL_COUNT; signal++)
        {
            value = offline_v2_signal_value(sample, signal);
            sum[signal] = sum[signal] + value;
        }
    }
    for (signal = 0U; signal < OFFLINE_V2_SIGNAL_COUNT; signal++)
    {
        mean[signal] = sum[signal] / 800.0f;
    }

    for (sample_index = 0U;
         sample_index < ZY100_OFFLINE_V2_WINDOW_SAMPLES;
         sample_index++)
    {
        sample = offline_v2_ring_sample(algo, start + sample_index);
        if (sample == NULL)
        {
            algo->status = ZY100_OFFLINE_V2_ALGO_RING_OVERRUN;
            return false;
        }
        bin = sample_index / OFFLINE_V2_SAMPLES_PER_BIN;
        for (signal = 0U; signal < OFFLINE_V2_SIGNAL_COUNT; signal++)
        {
            value = offline_v2_signal_value(sample, signal);
            delta = value - mean[signal];
            sqdev_sum[signal] = sqdev_sum[signal] + (delta * delta);
            bin_sum[signal][bin] = bin_sum[signal][bin] + value;
        }
    }

    for (signal = 0U; signal < OFFLINE_V2_SIGNAL_COUNT; signal++)
    {
        variance = sqdev_sum[signal] / 800.0f;
        scale = sqrtf(variance);
        if (!isfinite(scale) || (scale < OFFLINE_V2_MIN_SCALE))
        {
            scale = 1.0f;
        }
        for (bin = 0U; bin < OFFLINE_V2_SHAPE_BINS; bin++)
        {
            bin_mean = bin_sum[signal][bin] / 50.0f;
            algo->feature[(signal * OFFLINE_V2_SHAPE_BINS) + bin] =
                (bin_mean - mean[signal]) / scale;
            if (!isfinite(algo->feature[
                    (signal * OFFLINE_V2_SHAPE_BINS) + bin]))
            {
                algo->status = ZY100_OFFLINE_V2_ALGO_NON_FINITE;
                return false;
            }
        }
    }
    *saturated_out = false;
    return true;
}

int16_t zy100_offline_v2_q12_encode(float value, bool *saturated)
{
    float scaled = value * OFFLINE_V2_Q12_SCALE;
    int32_t truncated;
    float fraction;
    int32_t rounded;
    bool clipped = false;

    if (scaled >= 32767.5f)
    {
        rounded = 32767;
        clipped = true;
    }
    else if (scaled <= -32768.5f)
    {
        rounded = -32768;
        clipped = true;
    }
    else
    {
        truncated = (int32_t)scaled;
        fraction = scaled - (float)truncated;
        rounded = truncated;
        if ((fraction > 0.5f) ||
            ((fraction == 0.5f) && ((truncated & 1) != 0)))
        {
            rounded++;
        }
        else if ((fraction < -0.5f) ||
                 ((fraction == -0.5f) && ((truncated & 1) != 0)))
        {
            rounded--;
        }
        if (rounded > 32767)
        {
            rounded = 32767;
            clipped = true;
        }
        else if (rounded < -32768)
        {
            rounded = -32768;
            clipped = true;
        }
    }
    if (saturated != NULL)
    {
        *saturated = clipped;
    }
    return (int16_t)rounded;
}

static void offline_v2_heap_sift_down(uint32_t *values,
                                      uint32_t count,
                                      uint32_t root)
{
    uint32_t child;
    uint32_t swap_index;
    uint32_t temporary;

    while (root < (count / 2U))
    {
        child = root * 2U + 1U;
        swap_index = root;
        if (values[swap_index] < values[child])
        {
            swap_index = child;
        }
        if (((child + 1U) < count) &&
            (values[swap_index] < values[child + 1U]))
        {
            swap_index = child + 1U;
        }
        if (swap_index == root)
        {
            return;
        }
        temporary = values[root];
        values[root] = values[swap_index];
        values[swap_index] = temporary;
        root = swap_index;
    }
}

static void offline_v2_heap_sort(uint32_t *values, uint32_t count)
{
    uint32_t index;
    uint32_t temporary;

    if ((values == NULL) || (count < 2U))
    {
        return;
    }
    for (index = count / 2U; index != 0U; index--)
    {
        offline_v2_heap_sift_down(values, count, index - 1U);
    }
    for (index = count - 1U; index != 0U; index--)
    {
        temporary = values[0];
        values[0] = values[index];
        values[index] = temporary;
        offline_v2_heap_sift_down(values, index, 0U);
    }
}

static bool offline_v2_round_ratio(double ratio, double *rounded_out)
{
    double scaled;
    double fraction;
    uint64_t integer;

    if ((rounded_out == NULL) || !isfinite(ratio) || (ratio < 0.0))
    {
        return false;
    }
    scaled = ratio * OFFLINE_V2_SHOCK_RATIO_SCALE;
    if (!isfinite(scaled) || (scaled > 4294967295.0))
    {
        return false;
    }
    integer = (uint64_t)scaled;
    fraction = scaled - (double)integer;
    if ((fraction > 0.5) ||
        ((fraction == 0.5) && ((integer & 1ULL) != 0ULL)))
    {
        integer++;
    }
    *rounded_out = (double)integer / OFFLINE_V2_SHOCK_RATIO_SCALE;
    return true;
}

static bool offline_v2_compute_shock(zy100_offline_v2_algo_t *algo,
                                     uint32_t center,
                                     float *numerator_out,
                                     float *baseline_out,
                                     float *ratio_out,
                                     uint16_t *quality_flags_out)
{
    uint32_t start = center - OFFLINE_V2_WINDOW_PRE_SAMPLES;
    uint32_t index;
    uint32_t axis;
    uint32_t peak_index = 0U;
    uint32_t peak_mag2 = 0U;
    uint32_t delta_start;
    uint32_t delta_end;
    const zy100_offline_v2_imu_sample_t *sample;
    double baseline;
    double numerator = 0.0;
    double left;
    double right;
    double delta;
    double rounded_ratio;
    uint16_t flags = 0U;
    uint32_t shock_start_us = 0U;
    uint32_t shock_elapsed_us;

    if ((algo == NULL) || (numerator_out == NULL) ||
        (baseline_out == NULL) || (ratio_out == NULL) ||
        (quality_flags_out == NULL))
    {
        return false;
    }
    if (algo->time_us != NULL)
    {
        shock_start_us = algo->time_us();
    }
    for (index = 0U; index < ZY100_OFFLINE_V2_WINDOW_SAMPLES; index++)
    {
        sample = offline_v2_ring_sample(algo, start + index);
        if ((sample == NULL) || offline_v2_is_sentinel(sample))
        {
            return false;
        }
        for (axis = 0U; axis < 3U; axis++)
        {
            if ((sample->axis[axis] == INT16_MIN) ||
                (sample->axis[axis] == INT16_MAX))
            {
                flags |= ZY100_OFFLINE_V2_EVENT_FLAG_ACCEL_CLIPPING;
            }
        }
        for (axis = 3U; axis < 6U; axis++)
        {
            if (sample->axis[axis] == INT16_MAX)
            {
                flags |= ZY100_OFFLINE_V2_EVENT_FLAG_GYRO_CLIPPING;
            }
        }
        algo->shock_mag2[index] = offline_v2_axis_mag2(sample->axis);
        if ((index == 0U) || (algo->shock_mag2[index] > peak_mag2))
        {
            peak_mag2 = algo->shock_mag2[index];
            peak_index = index;
        }
    }

    delta_start = (peak_index > ZY100_OFFLINE_V2_SHOCK_HALF_WINDOW_SAMPLES) ?
        (peak_index - ZY100_OFFLINE_V2_SHOCK_HALF_WINDOW_SAMPLES) : 0U;
    delta_end = peak_index + ZY100_OFFLINE_V2_SHOCK_HALF_WINDOW_SAMPLES;
    if (delta_end >= ZY100_OFFLINE_V2_WINDOW_SAMPLES)
    {
        delta_end = ZY100_OFFLINE_V2_WINDOW_SAMPLES - 1U;
    }
    if (delta_start < delta_end)
    {
        left = sqrt((double)algo->shock_mag2[delta_start]);
        for (index = delta_start; index < delta_end; index++)
        {
            right = sqrt((double)algo->shock_mag2[index + 1U]);
            delta = fabs(right - left) / OFFLINE_V2_ACCEL_LSB_PER_G;
            if (delta > numerator)
            {
                numerator = delta;
            }
            left = right;
        }
    }

    offline_v2_heap_sort(algo->shock_mag2,
                         ZY100_OFFLINE_V2_WINDOW_SAMPLES);
    baseline = (sqrt((double)algo->shock_mag2[399]) +
                sqrt((double)algo->shock_mag2[400])) /
               (2.0 * OFFLINE_V2_ACCEL_LSB_PER_G);
    if (!isfinite(baseline) || (baseline <= 0.0) ||
        !offline_v2_round_ratio(numerator / baseline, &rounded_ratio))
    {
        return false;
    }
    flags |= ZY100_OFFLINE_V2_EVENT_FLAG_SHOCK_AVAILABLE;
    if (rounded_ratio >=
        ((double)ZY100_OFFLINE_V2_SHOCK_RATIO_X10000 /
         OFFLINE_V2_SHOCK_RATIO_SCALE))
    {
        flags |= ZY100_OFFLINE_V2_EVENT_FLAG_VALID_HIT;
    }
    *numerator_out = (float)numerator;
    *baseline_out = (float)baseline;
    *ratio_out = (float)rounded_ratio;
    *quality_flags_out = flags;
    if (algo->time_us != NULL)
    {
        shock_elapsed_us = algo->time_us() - shock_start_us;
        algo->shock_last_us = shock_elapsed_us;
        if (shock_elapsed_us > algo->shock_max_us)
        {
            algo->shock_max_us = shock_elapsed_us;
        }
    }
    return true;
}

static bool offline_v2_emit_event(zy100_offline_v2_algo_t *algo,
                                  const offline_v2_peak_t *peak)
{
    zy100_offline_v2_event_t event;
    uint32_t index;
    bool saturated;
    bool any_saturated = false;
    uint16_t quality_flags;
    uint32_t feature_start_us = 0U;
    uint32_t feature_elapsed_us;

    memset(&event, 0, sizeof(event));
    if (!offline_v2_compute_shock(algo, peak->sample_index,
                                  &event.shock_numerator,
                                  &event.shock_baseline,
                                  &event.shock_ratio,
                                  &quality_flags))
    {
        algo->shock_rejected_count++;
        return true;
    }
    if ((quality_flags & ZY100_OFFLINE_V2_EVENT_FLAG_VALID_HIT) == 0U)
    {
        algo->shock_rejected_count++;
        return true;
    }

    if (algo->time_us != NULL)
    {
        feature_start_us = algo->time_us();
    }
    if (!offline_v2_extract_feature(algo, peak->sample_index, &saturated))
    {
        return false;
    }
    if (algo->time_us != NULL)
    {
        feature_elapsed_us = algo->time_us() - feature_start_us;
        algo->feature_last_us = feature_elapsed_us;
        if (feature_elapsed_us > algo->feature_max_us)
        {
            algo->feature_max_us = feature_elapsed_us;
        }
    }

    event.center_sample_index = peak->sample_index;
    event.peak_mag2_raw = peak->mag2;
    event.sample_rate_hz = ZY100_OFFLINE_V2_SAMPLE_RATE_HZ;
    event.quality_flags = quality_flags;
    event.detector_version = ZY100_OFFLINE_V2_DETECTOR_VERSION;
    event.shock_version = ZY100_OFFLINE_V2_SHOCK_VERSION;
    event.feature_version = ZY100_OFFLINE_V2_FEATURE_VERSION;
    event.codec_version = ZY100_OFFLINE_V2_EVENT_CODEC_VERSION;
    for (index = 0U; index < ZY100_OFFLINE_V2_FEATURE_DIMENSIONS; index++)
    {
        event.feature_q12[index] =
            zy100_offline_v2_q12_encode(algo->feature[index], &saturated);
        any_saturated = any_saturated || saturated;
    }
    if (any_saturated)
    {
        event.quality_flags |=
            ZY100_OFFLINE_V2_EVENT_FLAG_Q12_SATURATION;
    }
    if ((algo->sink != NULL) && !algo->sink(&event, algo->sink_context))
    {
        algo->status = ZY100_OFFLINE_V2_ALGO_EVENT_SINK_FULL;
        return false;
    }
    algo->event_count++;
    return true;
}

static bool offline_v2_finalize_one(zy100_offline_v2_algo_t *algo)
{
    offline_v2_peak_t candidate;
    offline_v2_peak_t *front;
    uint32_t lower;

    if (algo->nms_count == 0U)
    {
        return true;
    }
    candidate = algo->nms_pending[algo->nms_head];
    lower = (candidate.sample_index >=
             (ZY100_OFFLINE_V2_NMS_DISTANCE_SAMPLES - 1U)) ?
            (candidate.sample_index -
             (ZY100_OFFLINE_V2_NMS_DISTANCE_SAMPLES - 1U)) : 0U;
    while (algo->maximum_count != 0U)
    {
        front = &algo->nms_maximum[algo->maximum_head];
        if (front->sample_index >= lower)
        {
            break;
        }
        algo->maximum_head =
            (algo->maximum_head + 1U) % OFFLINE_V2_NMS_DEQUE_CAPACITY;
        algo->maximum_count--;
    }
    if ((algo->maximum_count != 0U) &&
        (algo->nms_maximum[algo->maximum_head].sample_index ==
         candidate.sample_index))
    {
        algo->nms_selected_count++;
        if (!offline_v2_emit_event(algo, &candidate))
        {
            return false;
        }
    }
    algo->nms_head =
        (algo->nms_head + 1U) % OFFLINE_V2_NMS_PENDING_CAPACITY;
    algo->nms_count--;
    return true;
}

static bool offline_v2_process_legal_windows(zy100_offline_v2_algo_t *algo)
{
    offline_v2_peak_t peak;

    while (algo->raw_count != 0U)
    {
        peak = algo->raw_wait[algo->raw_head];
        if ((algo->sample_count - peak.sample_index) <
            OFFLINE_V2_WINDOW_POST_SAMPLES)
        {
            break;
        }
        algo->raw_head =
            (algo->raw_head + 1U) % OFFLINE_V2_RAW_WAIT_CAPACITY;
        algo->raw_count--;
        if (offline_v2_window_valid(algo, peak.sample_index))
        {
            if (!offline_v2_nms_enqueue(algo, &peak))
            {
                return false;
            }
        }
        else
        {
            algo->rejected_window_count++;
        }
    }
    return true;
}

static bool offline_v2_finalize_mature(zy100_offline_v2_algo_t *algo)
{
    offline_v2_peak_t oldest;

    while (algo->nms_count != 0U)
    {
        oldest = algo->nms_pending[algo->nms_head];
        if ((algo->sample_count - oldest.sample_index) <
            OFFLINE_V2_NMS_DECISION_DELAY_SAMPLES)
        {
            break;
        }
        if (!offline_v2_finalize_one(algo))
        {
            return false;
        }
    }
    return true;
}

static bool offline_v2_detect_latest_peak(zy100_offline_v2_algo_t *algo)
{
    uint32_t center;
    uint32_t left_mag2;
    uint32_t center_mag2;
    uint32_t right_mag2;
    const zy100_offline_v2_imu_sample_t *left;
    const zy100_offline_v2_imu_sample_t *middle;
    const zy100_offline_v2_imu_sample_t *right;

    if (algo->sample_count < 3U)
    {
        return true;
    }
    center = algo->sample_count - 2U;
    if (center < (OFFLINE_V2_WARMUP_SAMPLES + 1U))
    {
        return true;
    }
    left = offline_v2_ring_sample(algo, center - 1U);
    middle = offline_v2_ring_sample(algo, center);
    right = offline_v2_ring_sample(algo, center + 1U);
    if (offline_v2_is_sentinel(left) || offline_v2_is_sentinel(middle) ||
        offline_v2_is_sentinel(right) ||
        (algo->gap_before[offline_v2_ring_position(center)] != 0U) ||
        (algo->gap_before[offline_v2_ring_position(center + 1U)] != 0U))
    {
        return true;
    }
    center_mag2 = offline_v2_gyro_mag2(middle);
    if (center_mag2 >= ZY100_OFFLINE_V2_GYRO_MAG2_THRESHOLD)
    {
        left_mag2 = offline_v2_gyro_mag2(left);
        if (center_mag2 > left_mag2)
        {
            right_mag2 = offline_v2_gyro_mag2(right);
            if (center_mag2 >= right_mag2)
            {
                return offline_v2_raw_enqueue(algo, center, center_mag2);
            }
        }
    }
    return true;
}

uint32_t zy100_offline_v2_algo_workspace_bytes(void)
{
    return (uint32_t)sizeof(struct zy100_offline_v2_algo);
}

zy100_offline_v2_algo_t *zy100_offline_v2_algo_init(
    void *workspace,
    uint32_t workspace_bytes,
    zy100_offline_v2_event_sink_t sink,
    void *sink_context)
{
    zy100_offline_v2_algo_t *algo;

    if ((workspace == NULL) ||
        (workspace_bytes < sizeof(struct zy100_offline_v2_algo)))
    {
        return NULL;
    }
    algo = (zy100_offline_v2_algo_t *)workspace;
    memset(algo, 0, sizeof(*algo));
    algo->sink = sink;
    algo->sink_context = sink_context;
    algo->status = ZY100_OFFLINE_V2_ALGO_OK;
    return algo;
}

zy100_offline_v2_algo_status_t zy100_offline_v2_algo_push(
    zy100_offline_v2_algo_t *algo,
    const zy100_offline_v2_imu_sample_t *sample,
    bool discontinuity_before)
{
    uint32_t position;
    uint32_t detector_start_us = 0U;
    uint32_t detector_elapsed_us;

    if ((algo == NULL) || (sample == NULL))
    {
        return ZY100_OFFLINE_V2_ALGO_BAD_ARG;
    }
    if (algo->status != ZY100_OFFLINE_V2_ALGO_OK)
    {
        return algo->status;
    }
    if (algo->time_us != NULL)
    {
        detector_start_us = algo->time_us();
    }
    position = offline_v2_ring_position(algo->sample_count);
    algo->ring[position] = *sample;
    algo->gap_before[position] = discontinuity_before ? 1U : 0U;
    algo->sample_count++;
    if (!offline_v2_detect_latest_peak(algo) ||
        !offline_v2_process_legal_windows(algo) ||
        !offline_v2_finalize_mature(algo))
    {
        return algo->status;
    }
    if (algo->time_us != NULL)
    {
        detector_elapsed_us = algo->time_us() - detector_start_us;
        algo->detector_last_us = detector_elapsed_us;
        if (detector_elapsed_us > algo->detector_max_us)
        {
            algo->detector_max_us = detector_elapsed_us;
        }
    }
    return algo->status;
}

zy100_offline_v2_algo_status_t zy100_offline_v2_algo_finish(
    zy100_offline_v2_algo_t *algo,
    bool clean_end)
{
    if (algo == NULL)
    {
        return ZY100_OFFLINE_V2_ALGO_BAD_ARG;
    }
    if (algo->status != ZY100_OFFLINE_V2_ALGO_OK)
    {
        return algo->status;
    }
    if (!clean_end)
    {
        algo->raw_count = 0U;
        algo->nms_count = 0U;
        algo->maximum_count = 0U;
        return algo->status;
    }
    if (!offline_v2_process_legal_windows(algo))
    {
        return algo->status;
    }
    algo->rejected_window_count += algo->raw_count;
    algo->raw_count = 0U;
    while (algo->nms_count != 0U)
    {
        if (!offline_v2_finalize_one(algo))
        {
            break;
        }
    }
    return algo->status;
}

zy100_offline_v2_algo_status_t zy100_offline_v2_algo_status(
    const zy100_offline_v2_algo_t *algo)
{
    return (algo != NULL) ? algo->status : ZY100_OFFLINE_V2_ALGO_BAD_ARG;
}

uint32_t zy100_offline_v2_algo_event_count(
    const zy100_offline_v2_algo_t *algo)
{
    return (algo != NULL) ? algo->event_count : 0U;
}

void zy100_offline_v2_algo_set_time_source(
    zy100_offline_v2_algo_t *algo,
    zy100_offline_v2_time_us_t time_us)
{
    if (algo != NULL)
    {
        algo->time_us = time_us;
    }
}

bool zy100_offline_v2_algo_get_timing(
    const zy100_offline_v2_algo_t *algo,
    zy100_offline_v2_algo_timing_t *timing_out)
{
    if ((algo == NULL) || (timing_out == NULL))
    {
        return false;
    }
    timing_out->detector_last_us = algo->detector_last_us;
    timing_out->detector_max_us = algo->detector_max_us;
    timing_out->shock_last_us = algo->shock_last_us;
    timing_out->shock_max_us = algo->shock_max_us;
    timing_out->feature_last_us = algo->feature_last_us;
    timing_out->feature_max_us = algo->feature_max_us;
    timing_out->local_peak_count = algo->local_peak_count;
    timing_out->legal_peak_count = algo->legal_peak_count;
    timing_out->nms_selected_count = algo->nms_selected_count;
    timing_out->shock_rejected_count = algo->shock_rejected_count;
    timing_out->rejected_window_count = algo->rejected_window_count;
    return true;
}

#if defined(ZY100_OFFLINE_V2_HOST_TEST)
bool zy100_offline_v2_host_extract_window(
    void *workspace,
    uint32_t workspace_bytes,
    const zy100_offline_v2_imu_sample_t window[ZY100_OFFLINE_V2_WINDOW_SAMPLES],
    float feature_out[ZY100_OFFLINE_V2_FEATURE_DIMENSIONS],
    int16_t q12_out[ZY100_OFFLINE_V2_FEATURE_DIMENSIONS])
{
    zy100_offline_v2_algo_t *algo;
    uint32_t index;
    bool saturated = false;

    if ((window == NULL) || (feature_out == NULL) || (q12_out == NULL))
    {
        return false;
    }
    algo = zy100_offline_v2_algo_init(workspace, workspace_bytes,
                                      NULL, NULL);
    if (algo == NULL)
    {
        return false;
    }
    for (index = 0U; index < ZY100_OFFLINE_V2_WINDOW_SAMPLES; index++)
    {
        algo->ring[index] = window[index];
        algo->gap_before[index] = 0U;
    }
    algo->sample_count = ZY100_OFFLINE_V2_WINDOW_SAMPLES;
    if (!offline_v2_extract_feature(algo,
                                    OFFLINE_V2_WINDOW_PRE_SAMPLES,
                                    &saturated))
    {
        return false;
    }
    memcpy(feature_out, algo->feature, sizeof(algo->feature));
    for (index = 0U; index < ZY100_OFFLINE_V2_FEATURE_DIMENSIONS; index++)
    {
        q12_out[index] = zy100_offline_v2_q12_encode(
            algo->feature[index], &saturated);
    }
    return true;
}

bool zy100_offline_v2_host_shock_window(
    const zy100_offline_v2_imu_sample_t window[ZY100_OFFLINE_V2_WINDOW_SAMPLES],
    float *numerator_out,
    float *baseline_out,
    float *ratio_out,
    uint16_t *quality_flags_out)
{
    static uint8_t workspace[ZY100_OFFLINE_V2_ALGO_WORKSPACE_MAX];
    zy100_offline_v2_algo_t *algo;
    uint32_t index;

    if ((window == NULL) || (numerator_out == NULL) ||
        (baseline_out == NULL) || (ratio_out == NULL) ||
        (quality_flags_out == NULL))
    {
        return false;
    }
    algo = zy100_offline_v2_algo_init(workspace, sizeof(workspace),
                                      NULL, NULL);
    if (algo == NULL)
    {
        return false;
    }
    for (index = 0U; index < ZY100_OFFLINE_V2_WINDOW_SAMPLES; index++)
    {
        algo->ring[index] = window[index];
        algo->gap_before[index] = 0U;
    }
    algo->sample_count = ZY100_OFFLINE_V2_WINDOW_SAMPLES;
    return offline_v2_compute_shock(algo,
                                    OFFLINE_V2_WINDOW_PRE_SAMPLES,
                                    numerator_out,
                                    baseline_out,
                                    ratio_out,
                                    quality_flags_out);
}
#endif
