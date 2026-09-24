#include "imu_swing_detector.h"

#include <stddef.h>

#include "imu_motion_metrics.h"

#define IMU_SWING_DETECTOR_U64_DEC_BUF_LEN 21U
#define IMU_SWING_DETECTOR_FIFO_SAMPLE_HZ  1600U

typedef enum
{
    IMU_SWING_DETECTOR_STATE_IDLE = 0,
    IMU_SWING_DETECTOR_STATE_CANDIDATE = 1,
    IMU_SWING_DETECTOR_STATE_ACTIVE_SWING = 2,
} imu_swing_detector_state_t;

static const imu_swing_detector_config_t g_default_config =
{
    .enable = true,
    .detector_input_hz = IMU_DETECTOR_TARGET_INPUT_HZ,
    .decimation_factor = 0U,
    .acc_step_window_ms = 5U,
    .gyro_candidate_dps_min = 500U,				//角速度多大才认为出现挥拍上升沿
    .gyro_peak_dps_min = 600U,
    .confirm_window_ms = 250U,
    .quiet_gyro_dps = 80U,
    .quiet_acc_step_mg = 300U,
    .quiet_gap_ms = 250U,
    .min_swing_gap_ms = 400U,
};

static imu_swing_detector_config_t s_cfg = {0};
static bool s_cfg_valid = false;
static imu_swing_detector_state_t s_state = IMU_SWING_DETECTOR_STATE_IDLE;
static imu_swing_detector_stats_t s_stats = {0};
static imu_swing_detector_event_t s_events[IMU_SWING_DETECTOR_EVENT_LOG_MAX];
static uint32_t s_event_count = 0U;
static uint32_t s_event_dropped_count = 0U;

static uint32_t s_pending_wom_count = 0U;
static uint8_t s_pending_axis_mask = 0U;
static uint64_t s_candidate_start_ts_us = 0ULL;
static uint32_t s_candidate_start_sample_seq = 0U;
static uint32_t s_candidate_gyro_peak_dps = 0U;
static uint64_t s_candidate_peak_ts_us = 0ULL;
static uint32_t s_candidate_acc_delta_peak_mg = 0U;
static uint32_t s_candidate_acc_step_peak_mg = 0U;
static uint32_t s_candidate_jerk_peak_mg = 0U;
static bool s_candidate_acc_ref_valid = false;
static int32_t s_candidate_acc_ref_x_mg = 0;
static int32_t s_candidate_acc_ref_y_mg = 0;
static int32_t s_candidate_acc_ref_z_mg = 0;

static imu_motion_metrics_acc_history_t s_acc_history;

static imu_swing_detector_event_t s_active_event = {0};
static bool s_active_event_counted = false;
static bool s_active_event_logged = false;
static uint32_t s_active_event_log_slot = 0U;
static bool s_active_acc_ref_valid = false;
static int32_t s_active_acc_ref_x_mg = 0;
static int32_t s_active_acc_ref_y_mg = 0;
static int32_t s_active_acc_ref_z_mg = 0;
static uint32_t s_quiet_sample_count = 0U;
static uint64_t s_last_confirmed_peak_ts_us = 0ULL;

static void imu_swing_detector_inc_u32(uint32_t *value)
{
    if (*value < 0xFFFFFFFFU)
    {
        (*value)++;
    }
}

static uint32_t imu_swing_detector_duration_ms(uint64_t start_us, uint64_t end_us)
{
    uint64_t duration_ms;

    if (end_us < start_us)
    {
        return 0U;
    }

    duration_ms = (end_us - start_us) / 1000ULL;
    if (duration_ms > 0xFFFFFFFFULL)
    {
        return 0xFFFFFFFFU;
    }

    return (uint32_t)duration_ms;
}

static void imu_swing_detector_u64_to_dec(uint64_t value, char *buf, uint8_t len)
{
    char tmp[IMU_SWING_DETECTOR_U64_DEC_BUF_LEN];
    uint8_t tmp_len = 0U;
    uint8_t out_len = 0U;

    if ((buf == NULL) || (len == 0U))
    {
        return;
    }

    if (value == 0ULL)
    {
        buf[0] = '0';
        if (len > 1U)
        {
            buf[1] = '\0';
        }
        return;
    }

    while ((value > 0ULL) && (tmp_len < (uint8_t)sizeof(tmp)))
    {
        tmp[tmp_len] = (char)('0' + (value % 10ULL));
        value /= 10ULL;
        tmp_len++;
    }

    while ((tmp_len > 0U) && (out_len < (uint8_t)(len - 1U)))
    {
        tmp_len--;
        buf[out_len] = tmp[tmp_len];
        out_len++;
    }
    buf[out_len] = '\0';
}

static uint32_t imu_swing_detector_quiet_required_samples(void)
{
    uint32_t samples;

    if (s_cfg.detector_input_hz == 0U)
    {
        return 1U;
    }

    samples = (uint32_t)(((uint32_t)s_cfg.detector_input_hz *
                          (uint32_t)s_cfg.quiet_gap_ms + 999U) / 1000U);
    return (samples == 0U) ? 1U : samples;
}

static void imu_swing_detector_clear_pending_wom(void)
{
    s_pending_wom_count = 0U;
    s_pending_axis_mask = 0U;
}

static void imu_swing_detector_clear_candidate(void)
{
    imu_swing_detector_clear_pending_wom();
    s_candidate_start_ts_us = 0ULL;
    s_candidate_start_sample_seq = 0U;
    s_candidate_gyro_peak_dps = 0U;
    s_candidate_peak_ts_us = 0ULL;
    s_candidate_acc_delta_peak_mg = 0U;
    s_candidate_acc_step_peak_mg = 0U;
    s_candidate_jerk_peak_mg = 0U;
    s_candidate_acc_ref_valid = false;
    s_candidate_acc_ref_x_mg = 0;
    s_candidate_acc_ref_y_mg = 0;
    s_candidate_acc_ref_z_mg = 0;
}

static void imu_swing_detector_sync_active_event_log(void)
{
    if (s_active_event_counted && s_active_event_logged &&
        (s_active_event_log_slot < s_event_count))
    {
        s_events[s_active_event_log_slot] = s_active_event;
    }
}

static void imu_swing_detector_record_active_event(void)
{
    if (s_event_count < IMU_SWING_DETECTOR_EVENT_LOG_MAX)
    {
        s_events[s_event_count] = s_active_event;
        s_active_event_log_slot = s_event_count;
        s_active_event_logged = true;
        s_event_count++;
    }
    else
    {
        imu_swing_detector_inc_u32(&s_event_dropped_count);
        s_active_event_logged = false;
        s_active_event_log_slot = 0U;
    }

    s_stats.software_event_log_count = s_event_count;
    s_stats.software_event_log_dropped_count = s_event_dropped_count;
}

static void imu_swing_detector_start_candidate(uint64_t ts_us,
                                               uint32_t sample_seq,
                                               bool have_acc_ref,
                                               int32_t acc_x_mg,
                                               int32_t acc_y_mg,
                                               int32_t acc_z_mg)
{
    s_state = IMU_SWING_DETECTOR_STATE_CANDIDATE;
    s_candidate_start_ts_us = ts_us;
    s_candidate_start_sample_seq = sample_seq;
    s_candidate_gyro_peak_dps = 0U;
    s_candidate_peak_ts_us = ts_us;
    s_candidate_acc_delta_peak_mg = 0U;
    s_candidate_acc_step_peak_mg = 0U;
    s_candidate_jerk_peak_mg = 0U;
    s_candidate_acc_ref_valid = have_acc_ref;
    if (have_acc_ref)
    {
        s_candidate_acc_ref_x_mg = acc_x_mg;
        s_candidate_acc_ref_y_mg = acc_y_mg;
        s_candidate_acc_ref_z_mg = acc_z_mg;
    }
}

static void imu_swing_detector_fail_candidate(void)
{
    imu_swing_detector_inc_u32(&s_stats.false_candidate_count);
    imu_swing_detector_inc_u32(&s_stats.unconfirmed_candidate_count);
    imu_swing_detector_clear_candidate();
    s_state = IMU_SWING_DETECTOR_STATE_IDLE;
}

static bool imu_swing_detector_in_min_gap(void)
{
    uint64_t min_gap_us;

    if (s_last_confirmed_peak_ts_us == 0ULL)
    {
        return false;
    }

    min_gap_us = ((uint64_t)s_cfg.min_swing_gap_ms * 1000ULL);
    if (s_candidate_peak_ts_us <= s_last_confirmed_peak_ts_us)
    {
        return true;
    }

    return ((s_candidate_peak_ts_us - s_last_confirmed_peak_ts_us) < min_gap_us);
}

static void imu_swing_detector_enter_active_suppressed(void)
{
    imu_swing_detector_inc_u32(&s_stats.suppressed_by_min_gap_count);
    IMU_UNUSED(s_candidate_start_sample_seq);
    s_active_event_counted = false;
    s_active_event_logged = false;
    s_active_event_log_slot = 0U;
    s_active_acc_ref_valid = false;
    s_quiet_sample_count = 0U;
    imu_swing_detector_clear_candidate();
    s_state = IMU_SWING_DETECTOR_STATE_ACTIVE_SWING;
}

static void imu_swing_detector_enter_active_counted(void)
{
    imu_swing_detector_inc_u32(&s_stats.software_swing_count);
    imu_swing_detector_inc_u32(&s_stats.gyro_confirmed_count);
    s_last_confirmed_peak_ts_us = s_candidate_peak_ts_us;
    IMU_UNUSED(s_candidate_start_sample_seq);

    s_active_event.idx = s_stats.software_swing_count;
    s_active_event.start_ts_us = s_candidate_start_ts_us;
    s_active_event.peak_ts_us = s_candidate_peak_ts_us;
    s_active_event.end_ts_us = 0ULL;
    s_active_event.duration_ms = 0U;
    s_active_event.wom_count_inside_event = s_pending_wom_count;
    s_active_event.merged_axis_mask = s_pending_axis_mask;
    s_active_event.gyro_peak_dps = s_candidate_gyro_peak_dps;
    s_active_event.acc_delta_peak_mg = s_candidate_acc_delta_peak_mg;
    s_active_event.acc_step_peak_mg = s_candidate_acc_step_peak_mg;
    s_active_event.jerk_peak_mg = s_candidate_jerk_peak_mg;

    s_active_event_counted = true;
    s_active_event_logged = false;
    s_active_event_log_slot = 0U;
    s_active_acc_ref_valid = s_candidate_acc_ref_valid;
    s_active_acc_ref_x_mg = s_candidate_acc_ref_x_mg;
    s_active_acc_ref_y_mg = s_candidate_acc_ref_y_mg;
    s_active_acc_ref_z_mg = s_candidate_acc_ref_z_mg;
    s_quiet_sample_count = 0U;
    imu_swing_detector_record_active_event();
    imu_swing_detector_clear_candidate();
    s_state = IMU_SWING_DETECTOR_STATE_ACTIVE_SWING;
}

static void imu_swing_detector_try_confirm_candidate(void)
{
    if (s_candidate_gyro_peak_dps < (uint32_t)s_cfg.gyro_peak_dps_min)
    {
        return;
    }

    if (imu_swing_detector_in_min_gap())
    {
        imu_swing_detector_enter_active_suppressed();
    }
    else
    {
        imu_swing_detector_enter_active_counted();
    }
}

static void imu_swing_detector_update_candidate_metrics(uint64_t ts_us,
                                                        uint32_t gyro_norm_dps,
                                                        int32_t acc_x_mg,
                                                        int32_t acc_y_mg,
                                                        int32_t acc_z_mg,
                                                        uint32_t acc_step_mg)
{
    uint32_t acc_delta_mg = 0U;

    if (ts_us < s_candidate_start_ts_us)
    {
        ts_us = s_candidate_start_ts_us;
    }

    if (!s_candidate_acc_ref_valid)
    {
        s_candidate_acc_ref_x_mg = acc_x_mg;
        s_candidate_acc_ref_y_mg = acc_y_mg;
        s_candidate_acc_ref_z_mg = acc_z_mg;
        s_candidate_acc_ref_valid = true;
    }
    else
    {
        acc_delta_mg =
            imu_motion_metrics_vector_norm_i32(acc_x_mg - s_candidate_acc_ref_x_mg,
                                               acc_y_mg - s_candidate_acc_ref_y_mg,
                                               acc_z_mg - s_candidate_acc_ref_z_mg);
    }

    if (gyro_norm_dps > s_candidate_gyro_peak_dps)
    {
        s_candidate_gyro_peak_dps = gyro_norm_dps;
        s_candidate_peak_ts_us = ts_us;
    }
    if (acc_delta_mg > s_candidate_acc_delta_peak_mg)
    {
        s_candidate_acc_delta_peak_mg = acc_delta_mg;
    }
    if (acc_step_mg > s_candidate_acc_step_peak_mg)
    {
        s_candidate_acc_step_peak_mg = acc_step_mg;
    }
    if (acc_step_mg > s_candidate_jerk_peak_mg)
    {
        s_candidate_jerk_peak_mg = acc_step_mg;
    }
}

static void imu_swing_detector_update_active_metrics(uint64_t ts_us,
                                                     uint32_t gyro_norm_dps,
                                                     int32_t acc_x_mg,
                                                     int32_t acc_y_mg,
                                                     int32_t acc_z_mg,
                                                     uint32_t acc_step_mg)
{
    if (s_active_event_counted)
    {
        uint32_t acc_delta_mg = 0U;

        if (ts_us < s_active_event.start_ts_us)
        {
            ts_us = s_active_event.start_ts_us;
        }
        if (!s_active_acc_ref_valid)
        {
            s_active_acc_ref_x_mg = acc_x_mg;
            s_active_acc_ref_y_mg = acc_y_mg;
            s_active_acc_ref_z_mg = acc_z_mg;
            s_active_acc_ref_valid = true;
        }
        else
        {
            acc_delta_mg =
                imu_motion_metrics_vector_norm_i32(acc_x_mg - s_active_acc_ref_x_mg,
                                                   acc_y_mg - s_active_acc_ref_y_mg,
                                                   acc_z_mg - s_active_acc_ref_z_mg);
        }
        if (gyro_norm_dps > s_active_event.gyro_peak_dps)
        {
            s_active_event.gyro_peak_dps = gyro_norm_dps;
            s_active_event.peak_ts_us = ts_us;
        }
        if (acc_delta_mg > s_active_event.acc_delta_peak_mg)
        {
            s_active_event.acc_delta_peak_mg = acc_delta_mg;
        }
        if (acc_step_mg > s_active_event.acc_step_peak_mg)
        {
            s_active_event.acc_step_peak_mg = acc_step_mg;
        }
        if (acc_step_mg > s_active_event.jerk_peak_mg)
        {
            s_active_event.jerk_peak_mg = acc_step_mg;
        }
        imu_swing_detector_sync_active_event_log();
    }
}

static void imu_swing_detector_end_active(uint64_t end_ts_us)
{
    if (s_active_event_counted)
    {
        if (end_ts_us < s_active_event.start_ts_us)
        {
            end_ts_us = s_active_event.start_ts_us;
        }
        s_active_event.end_ts_us = end_ts_us;
        s_active_event.duration_ms =
            imu_swing_detector_duration_ms(s_active_event.start_ts_us, end_ts_us);
        imu_swing_detector_sync_active_event_log();
    }

    s_active_event_counted = false;
    s_active_event_logged = false;
    s_active_event_log_slot = 0U;
    s_active_acc_ref_valid = false;
    s_quiet_sample_count = 0U;
    s_state = IMU_SWING_DETECTOR_STATE_IDLE;
}

static void imu_swing_detector_process_active_sample(uint64_t ts_us,
                                                     uint32_t gyro_norm_dps,
                                                     int32_t acc_x_mg,
                                                     int32_t acc_y_mg,
                                                     int32_t acc_z_mg,
                                                     uint32_t acc_step_mg)
{
    imu_swing_detector_update_active_metrics(ts_us,
                                             gyro_norm_dps,
                                             acc_x_mg,
                                             acc_y_mg,
                                             acc_z_mg,
                                             acc_step_mg);

    if ((gyro_norm_dps < (uint32_t)s_cfg.quiet_gyro_dps) &&
        (acc_step_mg < (uint32_t)s_cfg.quiet_acc_step_mg))
    {
        imu_swing_detector_inc_u32(&s_quiet_sample_count);
    }
    else
    {
        s_quiet_sample_count = 0U;
    }

    if (s_quiet_sample_count >= imu_swing_detector_quiet_required_samples())
    {
        imu_swing_detector_end_active(ts_us);
    }
}

static void imu_swing_detector_process_idle_sample(uint64_t ts_us,
                                                   uint32_t sample_seq,
                                                   uint32_t gyro_norm_dps,
                                                   int32_t acc_x_mg,
                                                   int32_t acc_y_mg,
                                                   int32_t acc_z_mg,
                                                   uint32_t acc_step_mg)
{
    if (gyro_norm_dps < (uint32_t)s_cfg.gyro_candidate_dps_min)
    {
        return;
    }

    imu_swing_detector_inc_u32(&s_stats.gyro_candidate_count);
    imu_swing_detector_start_candidate(ts_us,
                                       sample_seq,
                                       true,
                                       acc_x_mg,
                                       acc_y_mg,
                                       acc_z_mg);
    imu_swing_detector_update_candidate_metrics(ts_us,
                                                gyro_norm_dps,
                                                acc_x_mg,
                                                acc_y_mg,
                                                acc_z_mg,
                                                acc_step_mg);
    imu_swing_detector_try_confirm_candidate();
}

const imu_swing_detector_config_t *imu_swing_detector_default_config(void)
{
    return &g_default_config;
}

uint16_t imu_swing_detector_input_hz(void)
{
    if (!s_cfg_valid)
    {
        return g_default_config.detector_input_hz;
    }
    return s_cfg.detector_input_hz;
}

uint16_t imu_swing_detector_decimation_factor(void)
{
    if (!s_cfg_valid)
    {
        return g_default_config.decimation_factor;
    }
    return s_cfg.decimation_factor;
}

imu_status_t imu_swing_detector_reset(const imu_swing_detector_config_t *cfg)
{
    const imu_swing_detector_config_t *use_cfg =
        (cfg != NULL) ? cfg : &g_default_config;
    uint32_t i;

    s_cfg = *use_cfg;
    if (s_cfg.detector_input_hz == 0U)
    {
        s_cfg.detector_input_hz = g_default_config.detector_input_hz;
    }
    if (s_cfg.acc_step_window_ms == 0U)
    {
        s_cfg.acc_step_window_ms = g_default_config.acc_step_window_ms;
    }

    s_cfg_valid = true;
    s_state = IMU_SWING_DETECTOR_STATE_IDLE;
    s_stats.detector_input_sample_count = 0U;
    s_stats.wom_candidate_count = 0U;
    s_stats.gyro_candidate_count = 0U;
    s_stats.software_swing_count = 0U;
    s_stats.gyro_confirmed_count = 0U;
    s_stats.false_candidate_count = 0U;
    s_stats.unconfirmed_candidate_count = 0U;
    s_stats.suppressed_by_min_gap_count = 0U;
    s_stats.software_event_log_count = 0U;
    s_stats.software_event_log_dropped_count = 0U;

    for (i = 0U; i < IMU_SWING_DETECTOR_EVENT_LOG_MAX; i++)
    {
        s_events[i].idx = 0U;
        s_events[i].start_ts_us = 0ULL;
        s_events[i].peak_ts_us = 0ULL;
        s_events[i].end_ts_us = 0ULL;
        s_events[i].duration_ms = 0U;
        s_events[i].wom_count_inside_event = 0U;
        s_events[i].merged_axis_mask = 0U;
        s_events[i].gyro_peak_dps = 0U;
        s_events[i].acc_delta_peak_mg = 0U;
        s_events[i].acc_step_peak_mg = 0U;
        s_events[i].jerk_peak_mg = 0U;
    }

    s_event_count = 0U;
    s_event_dropped_count = 0U;
    imu_swing_detector_clear_candidate();

    imu_motion_metrics_acc_history_reset(&s_acc_history);

    s_active_event.idx = 0U;
    s_active_event.start_ts_us = 0ULL;
    s_active_event.peak_ts_us = 0ULL;
    s_active_event.end_ts_us = 0ULL;
    s_active_event.duration_ms = 0U;
    s_active_event.wom_count_inside_event = 0U;
    s_active_event.merged_axis_mask = 0U;
    s_active_event.gyro_peak_dps = 0U;
    s_active_event.acc_delta_peak_mg = 0U;
    s_active_event.acc_step_peak_mg = 0U;
    s_active_event.jerk_peak_mg = 0U;
    s_active_event_counted = false;
    s_active_event_logged = false;
    s_active_event_log_slot = 0U;
    s_active_acc_ref_valid = false;
    s_active_acc_ref_x_mg = 0;
    s_active_acc_ref_y_mg = 0;
    s_active_acc_ref_z_mg = 0;
    s_quiet_sample_count = 0U;
    s_last_confirmed_peak_ts_us = 0ULL;

    return IMU_STATUS_OK;
}

imu_status_t imu_swing_detector_on_wom_candidate(
    const imu_swing_detector_wom_candidate_t *candidate)
{
    if (candidate == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }
    if (!s_cfg_valid)
    {
        (void)imu_swing_detector_reset(NULL);
    }
    if (!s_cfg.enable)
    {
        return IMU_STATUS_OK;
    }

    imu_swing_detector_inc_u32(&s_stats.wom_candidate_count);

    if (s_state == IMU_SWING_DETECTOR_STATE_CANDIDATE)
    {
        imu_swing_detector_inc_u32(&s_pending_wom_count);
        s_pending_axis_mask |= candidate->axis_mask;
    }
    else
    {
        if (s_active_event_counted)
        {
            imu_swing_detector_inc_u32(&s_active_event.wom_count_inside_event);
            s_active_event.merged_axis_mask |= candidate->axis_mask;
            imu_swing_detector_sync_active_event_log();
        }
    }

    return IMU_STATUS_OK;
}

imu_status_t imu_swing_detector_on_sample(
    const imu_swing_detector_input_sample_t *sample)
{
    uint64_t sample_ts_us;
    int32_t gyro_x_dps;
    int32_t gyro_y_dps;
    int32_t gyro_z_dps;
    int32_t acc_x_mg;
    int32_t acc_y_mg;
    int32_t acc_z_mg;
    uint32_t gyro_norm_dps;
    uint32_t acc_step_mg = 0U;
    bool candidate_timed_out = false;

    if (sample == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }
    if (!s_cfg_valid)
    {
        (void)imu_swing_detector_reset(NULL);
    }
    if (!s_cfg.enable)
    {
        return IMU_STATUS_OK;
    }

    imu_swing_detector_inc_u32(&s_stats.detector_input_sample_count);

    sample_ts_us = sample->sample_ts_us;
    gyro_x_dps = imu_motion_metrics_raw_to_dps(sample->gyro_x_raw);
    gyro_y_dps = imu_motion_metrics_raw_to_dps(sample->gyro_y_raw);
    gyro_z_dps = imu_motion_metrics_raw_to_dps(sample->gyro_z_raw);
    acc_x_mg = imu_motion_metrics_raw_to_mg(sample->accel_x_raw);
    acc_y_mg = imu_motion_metrics_raw_to_mg(sample->accel_y_raw);
    acc_z_mg = imu_motion_metrics_raw_to_mg(sample->accel_z_raw);

    gyro_norm_dps = imu_motion_metrics_vector_norm_i32(gyro_x_dps,
                                                       gyro_y_dps,
                                                       gyro_z_dps);
    acc_step_mg = imu_motion_metrics_acc_history_update(&s_acc_history,
                                                         sample_ts_us,
                                                         acc_x_mg,
                                                         acc_y_mg,
                                                         acc_z_mg,
                                                         s_cfg.acc_step_window_ms);

    if (s_state == IMU_SWING_DETECTOR_STATE_IDLE)
    {
        imu_swing_detector_process_idle_sample(sample_ts_us,
                                               sample->sample_seq,
                                               gyro_norm_dps,
                                               acc_x_mg,
                                               acc_y_mg,
                                               acc_z_mg,
                                               acc_step_mg);
        return IMU_STATUS_OK;
    }

    if (s_state == IMU_SWING_DETECTOR_STATE_CANDIDATE)
    {
        if (sample_ts_us >= s_candidate_start_ts_us)
        {
            candidate_timed_out =
                ((sample_ts_us - s_candidate_start_ts_us) >
                 ((uint64_t)s_cfg.confirm_window_ms * 1000ULL));
        }

        if (candidate_timed_out)
        {
            imu_swing_detector_fail_candidate();
            imu_swing_detector_process_idle_sample(sample_ts_us,
                                                   sample->sample_seq,
                                                   gyro_norm_dps,
                                                   acc_x_mg,
                                                   acc_y_mg,
                                                   acc_z_mg,
                                                   acc_step_mg);
            return IMU_STATUS_OK;
        }

        imu_swing_detector_update_candidate_metrics(sample_ts_us,
                                                    gyro_norm_dps,
                                                    acc_x_mg,
                                                    acc_y_mg,
                                                    acc_z_mg,
                                                    acc_step_mg);
        imu_swing_detector_try_confirm_candidate();
        return IMU_STATUS_OK;
    }

    imu_swing_detector_process_active_sample(sample_ts_us,
                                             gyro_norm_dps,
                                             acc_x_mg,
                                             acc_y_mg,
                                             acc_z_mg,
                                             acc_step_mg);
    return IMU_STATUS_OK;
}

imu_status_t imu_swing_detector_finish(uint64_t end_ts_us)
{
    if (!s_cfg_valid)
    {
        (void)imu_swing_detector_reset(NULL);
    }
    if (!s_cfg.enable)
    {
        return IMU_STATUS_OK;
    }

    if (s_state == IMU_SWING_DETECTOR_STATE_CANDIDATE)
    {
        imu_swing_detector_fail_candidate();
    }
    else if (s_state == IMU_SWING_DETECTOR_STATE_ACTIVE_SWING)
    {
        imu_swing_detector_end_active(end_ts_us);
    }

    s_stats.software_event_log_count = s_event_count;
    s_stats.software_event_log_dropped_count = s_event_dropped_count;
    return IMU_STATUS_OK;
}

void imu_swing_detector_get_stats(imu_swing_detector_stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    *out = s_stats;
    out->software_event_log_count = s_event_count;
    out->software_event_log_dropped_count = s_event_dropped_count;
}

void imu_swing_detector_log_config(void)
{
    if (!s_cfg_valid)
    {
        (void)imu_swing_detector_reset(NULL);
    }

    DBG_DIRECT("[IMU_SWING_DETECTOR_CFG] input_source=fifo_rate_gate "
               "fifo_sample_hz=%u detector_input_hz=%u decimation_factor=%u "
               "acc_step_window_ms=%u "
               "gyro_candidate_dps_min=%u gyro_peak_dps_min=%u confirm_window_ms=%u "
               "quiet_gyro_dps=%u quiet_acc_step_mg=%u quiet_gap_ms=%u "
               "min_swing_gap_ms=%u",
               IMU_SWING_DETECTOR_FIFO_SAMPLE_HZ,
               s_cfg.detector_input_hz,
               s_cfg.decimation_factor,
               s_cfg.acc_step_window_ms,
               s_cfg.gyro_candidate_dps_min,
               s_cfg.gyro_peak_dps_min,
               s_cfg.confirm_window_ms,
               s_cfg.quiet_gyro_dps,
               s_cfg.quiet_acc_step_mg,
               s_cfg.quiet_gap_ms,
               s_cfg.min_swing_gap_ms);
}

void imu_swing_detector_log_stats(void)
{
    imu_swing_detector_stats_t stats;

    imu_swing_detector_get_stats(&stats);
    DBG_DIRECT("[IMU_SWING_DETECTOR_STATS] detector_input_sample_count=%u "
               "wom_candidate_count=%u gyro_candidate_count=%u "
               "software_swing_count=%u gyro_confirmed_count=%u "
               "false_candidate_count=%u unconfirmed_candidate_count=%u "
               "suppressed_by_min_gap_count=%u software_event_log_count=%u "
               "software_event_log_dropped_count=%u",
               stats.detector_input_sample_count,
               stats.wom_candidate_count,
               stats.gyro_candidate_count,
               stats.software_swing_count,
               stats.gyro_confirmed_count,
               stats.false_candidate_count,
               stats.unconfirmed_candidate_count,
               stats.suppressed_by_min_gap_count,
               stats.software_event_log_count,
               stats.software_event_log_dropped_count);
}

void imu_swing_detector_log_events(void)
{
    uint32_t i;

    if (s_event_count == 0U)
    {
        DBG_DIRECT("[IMU_SWING_EVENT] count=0");
        return;
    }

    for (i = 0U; i < s_event_count; i++)
    {
        char start_ts[IMU_SWING_DETECTOR_U64_DEC_BUF_LEN];
        char peak_ts[IMU_SWING_DETECTOR_U64_DEC_BUF_LEN];
        char end_ts[IMU_SWING_DETECTOR_U64_DEC_BUF_LEN];
        const imu_swing_detector_event_t *event = &s_events[i];

        imu_swing_detector_u64_to_dec(event->start_ts_us, start_ts, (uint8_t)sizeof(start_ts));
        imu_swing_detector_u64_to_dec(event->peak_ts_us, peak_ts, (uint8_t)sizeof(peak_ts));
        imu_swing_detector_u64_to_dec(event->end_ts_us, end_ts, (uint8_t)sizeof(end_ts));
        DBG_DIRECT("[IMU_SWING_EVENT] idx=%u start_ts_us=%s peak_ts_us=%s "
                   "end_ts_us=%s duration_ms=%u wom_count_inside_event=%u "
                   "merged_axis_mask=0x%02x gyro_peak_dps=%u "
                   "acc_delta_peak_mg=%u acc_step_peak_mg=%u jerk_peak_mg=%u "
                   "confirm_reason=gyro_peak",
                   event->idx,
                   start_ts,
                   peak_ts,
                   end_ts,
                   event->duration_ms,
                   event->wom_count_inside_event,
                   event->merged_axis_mask,
                   event->gyro_peak_dps,
                   event->acc_delta_peak_mg,
                   event->acc_step_peak_mg,
                   event->jerk_peak_mg);
    }
}
