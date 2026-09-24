#include "imu_marker_detector.h"

#include <stddef.h>
#include <string.h>

#include "imu_motion_metrics.h"

#define IMU_MARKER_DETECTOR_US_PER_MS       1000UL
#define IMU_MARKER_DETECTOR_SCORE_SCALE     100U

typedef struct
{
    bool active;
    uint32_t start_t_us;
    uint32_t sample_seq;
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    uint32_t gyro_peak_dps;
    uint32_t acc_step_peak_mg;
    uint32_t confirm_count;
    uint16_t flags;
} imu_marker_detector_candidate_t;

static imu_marker_detector_stats_t s_stats;
static imu_marker_detector_candidate_t s_candidate;
static imu_motion_metrics_acc_history_t s_acc_history;
static bool s_prev_valid = false;
static uint32_t s_prev_gyro_norm_dps = 0U;
static uint32_t s_prev_acc_step_mg = 0U;
static bool s_cooldown_valid = false;
static uint32_t s_cooldown_start_t_us = 0U;

static void imu_marker_detector_inc_u32(uint32_t *value)
{
    if (*value < 0xFFFFFFFFU)
    {
        (*value)++;
    }
}

static uint32_t imu_marker_detector_window_us(uint32_t window_ms)
{
    return window_ms * IMU_MARKER_DETECTOR_US_PER_MS;
}

static bool imu_marker_detector_elapsed(uint32_t now_us,
                                        uint32_t start_us,
                                        uint32_t duration_us)
{
    return ((uint32_t)(now_us - start_us) >= duration_us);
}

static uint16_t imu_marker_detector_score(uint32_t gyro_peak_dps,
                                          uint32_t acc_step_peak_mg)
{
    uint32_t gyro_score = 0U;
    uint32_t acc_score = 0U;
    uint32_t score;

    if (ZY100_RT_MARKER_GYRO_RISING_DPS_MIN != 0U)
    {
        gyro_score = (gyro_peak_dps * IMU_MARKER_DETECTOR_SCORE_SCALE) /
                     ZY100_RT_MARKER_GYRO_RISING_DPS_MIN;
    }
    if (ZY100_RT_MARKER_ACC_STEP_MG_MIN != 0U)
    {
        acc_score = (acc_step_peak_mg * IMU_MARKER_DETECTOR_SCORE_SCALE) /
                    ZY100_RT_MARKER_ACC_STEP_MG_MIN;
    }

    score = (gyro_score > acc_score) ? gyro_score : acc_score;
    return (score > 0xFFFFU) ? 0xFFFFU : (uint16_t)score;
}

static void imu_marker_detector_clear_candidate(void)
{
    memset(&s_candidate, 0, sizeof(s_candidate));
}

static void imu_marker_detector_clear_runtime_state(void)
{
    imu_marker_detector_clear_candidate();
    imu_motion_metrics_acc_history_reset(&s_acc_history);
    s_prev_valid = false;
    s_prev_gyro_norm_dps = 0U;
    s_prev_acc_step_mg = 0U;
    s_cooldown_valid = false;
    s_cooldown_start_t_us = 0U;
}

static bool imu_marker_detector_in_cooldown(uint32_t now_us)
{
    if (!s_cooldown_valid)
    {
        return false;
    }

    return !imu_marker_detector_elapsed(
                now_us,
                s_cooldown_start_t_us,
                imu_marker_detector_window_us(ZY100_RT_MARKER_COOLDOWN_MS));
}

static void imu_marker_detector_start_candidate(const imu_marker_detector_sample_t *sample,
                                                uint32_t gyro_norm_dps,
                                                uint32_t acc_step_mg,
                                                uint16_t flags)
{
    s_candidate.active = true;
    s_candidate.start_t_us = sample->t_us;
    s_candidate.sample_seq = sample->seq;
    s_candidate.accel_x_raw = sample->accel_x_raw;
    s_candidate.accel_y_raw = sample->accel_y_raw;
    s_candidate.accel_z_raw = sample->accel_z_raw;
    s_candidate.gyro_x_raw = sample->gyro_x_raw;
    s_candidate.gyro_y_raw = sample->gyro_y_raw;
    s_candidate.gyro_z_raw = sample->gyro_z_raw;
    s_candidate.gyro_peak_dps = gyro_norm_dps;
    s_candidate.acc_step_peak_mg = acc_step_mg;
    s_candidate.confirm_count = 1U;
    s_candidate.flags = flags;
    imu_marker_detector_inc_u32(&s_stats.candidate_count);
}

static void imu_marker_detector_update_candidate(uint32_t gyro_norm_dps,
                                                 uint32_t acc_step_mg,
                                                 uint16_t flags)
{
    if (gyro_norm_dps > s_candidate.gyro_peak_dps)
    {
        s_candidate.gyro_peak_dps = gyro_norm_dps;
    }
    if (acc_step_mg > s_candidate.acc_step_peak_mg)
    {
        s_candidate.acc_step_peak_mg = acc_step_mg;
    }
    if (flags != 0U)
    {
        s_candidate.flags |= flags;
        imu_marker_detector_inc_u32(&s_candidate.confirm_count);
    }
}

static void imu_marker_detector_fill_event(imu_marker_detector_event_t *event)
{
    memset(event, 0, sizeof(*event));
    event->t_us = s_candidate.start_t_us;
    event->seq = s_candidate.sample_seq;
    event->accel_x_raw = s_candidate.accel_x_raw;
    event->accel_y_raw = s_candidate.accel_y_raw;
    event->accel_z_raw = s_candidate.accel_z_raw;
    event->gyro_x_raw = s_candidate.gyro_x_raw;
    event->gyro_y_raw = s_candidate.gyro_y_raw;
    event->gyro_z_raw = s_candidate.gyro_z_raw;
    event->score = imu_marker_detector_score(s_candidate.gyro_peak_dps,
                                             s_candidate.acc_step_peak_mg);
    event->flags = (uint16_t)(s_candidate.flags |
                              IMU_MARKER_DETECTOR_EVENT_FLAG_CONFIRMED);
}

imu_status_t imu_marker_detector_reset(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
    imu_marker_detector_clear_runtime_state();
    return IMU_STATUS_OK;
}

imu_status_t imu_marker_detector_reset_runtime_state_preserve_stats(void)
{
    imu_marker_detector_clear_runtime_state();
    return IMU_STATUS_OK;
}

imu_status_t imu_marker_detector_process_sample(const imu_marker_detector_sample_t *sample,
                                                bool *event_valid,
                                                imu_marker_detector_event_t *event)
{
    int32_t acc_x_mg;
    int32_t acc_y_mg;
    int32_t acc_z_mg;
    uint32_t gyro_norm_dps;
    uint32_t acc_step_mg;
    uint16_t above_flags = 0U;
    uint16_t rising_flags = 0U;

    if ((sample == NULL) || (event_valid == NULL) || (event == NULL))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    *event_valid = false;
    memset(event, 0, sizeof(*event));

    acc_x_mg = imu_motion_metrics_raw_to_mg(sample->accel_x_raw);
    acc_y_mg = imu_motion_metrics_raw_to_mg(sample->accel_y_raw);
    acc_z_mg = imu_motion_metrics_raw_to_mg(sample->accel_z_raw);
    gyro_norm_dps = imu_motion_metrics_calc_gyro_norm_dps(sample->gyro_x_raw,
                                                          sample->gyro_y_raw,
                                                          sample->gyro_z_raw);
    acc_step_mg = imu_motion_metrics_acc_history_update(&s_acc_history,
                                                        sample->t_us,
                                                        acc_x_mg,
                                                        acc_y_mg,
                                                        acc_z_mg,
                                                        ZY100_RT_MARKER_ACC_STEP_WINDOW_MS);
    imu_marker_detector_inc_u32(&s_stats.sample_count);

    if (gyro_norm_dps >= ZY100_RT_MARKER_GYRO_RISING_DPS_MIN)
    {
        above_flags |= IMU_MARKER_DETECTOR_EVENT_FLAG_GYRO_RISE;
    }
    if (acc_step_mg >= ZY100_RT_MARKER_ACC_STEP_MG_MIN)
    {
        above_flags |= IMU_MARKER_DETECTOR_EVENT_FLAG_ACC_RISE;
    }

    if (s_prev_valid)
    {
        if ((s_prev_gyro_norm_dps < ZY100_RT_MARKER_GYRO_RISING_DPS_MIN) &&
            ((above_flags & IMU_MARKER_DETECTOR_EVENT_FLAG_GYRO_RISE) != 0U))
        {
            rising_flags |= IMU_MARKER_DETECTOR_EVENT_FLAG_GYRO_RISE;
        }
        if ((s_prev_acc_step_mg < ZY100_RT_MARKER_ACC_STEP_MG_MIN) &&
            ((above_flags & IMU_MARKER_DETECTOR_EVENT_FLAG_ACC_RISE) != 0U))
        {
            rising_flags |= IMU_MARKER_DETECTOR_EVENT_FLAG_ACC_RISE;
        }
    }

    if (imu_marker_detector_in_cooldown(sample->t_us))
    {
        if (above_flags != 0U)
        {
            imu_marker_detector_inc_u32(&s_stats.dropped_cooldown_count);
        }
        s_prev_valid = true;
        s_prev_gyro_norm_dps = gyro_norm_dps;
        s_prev_acc_step_mg = acc_step_mg;
        return IMU_STATUS_OK;
    }

    if (s_candidate.active)
    {
        bool candidate_timed_out;

        imu_marker_detector_update_candidate(gyro_norm_dps,
                                             acc_step_mg,
                                             above_flags);
        candidate_timed_out = imu_marker_detector_elapsed(
                                  sample->t_us,
                                  s_candidate.start_t_us,
                                  imu_marker_detector_window_us(
                                      ZY100_RT_MARKER_CONFIRM_WINDOW_MS));

        if (!candidate_timed_out &&
            (s_candidate.confirm_count >= ZY100_RT_MARKER_CONFIRM_MIN_SAMPLES))
        {
            imu_marker_detector_fill_event(event);
            *event_valid = true;
            imu_marker_detector_inc_u32(&s_stats.marker_count);
            s_cooldown_valid = true;
            s_cooldown_start_t_us = sample->t_us;
            imu_marker_detector_clear_candidate();
        }
        else if (candidate_timed_out)
        {
            imu_marker_detector_inc_u32(&s_stats.rejected_timeout_count);
            imu_marker_detector_clear_candidate();
        }
    }
    else if ((rising_flags != 0U) && s_prev_valid)
    {
        imu_marker_detector_start_candidate(sample,
                                            gyro_norm_dps,
                                            acc_step_mg,
                                            rising_flags);
    }

    s_prev_valid = true;
    s_prev_gyro_norm_dps = gyro_norm_dps;
    s_prev_acc_step_mg = acc_step_mg;
    return IMU_STATUS_OK;
}

void imu_marker_detector_get_stats(imu_marker_detector_stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    *out = s_stats;
}
