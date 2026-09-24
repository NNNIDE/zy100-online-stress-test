#ifndef IMU_PRE_TRIGGER_MARKER_H
#define IMU_PRE_TRIGGER_MARKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../common/imu_common.h"
#include "imu_swing_detector.h"

typedef enum
{
    IMU_PRE_TRIGGER_TWIST_AXIS_GZ = 0,
} imu_pre_trigger_twist_axis_t;

typedef struct
{
    bool enable;

    uint16_t gyro_candidate_dps_min;
    uint16_t acc_step_candidate_mg_min;
    uint16_t confirm_window_ms;
    uint16_t gyro_confirm_peak_dps_min;
    uint16_t gyro_above_candidate_min_count;
    uint16_t acc_support_step_mg_min;
    uint16_t acc_only_confirm_gyro_dps_min;
    uint16_t acc_step_window_ms;
    bool acc_only_marker_enable;

    uint16_t gyro_rearm_dps;
    uint16_t acc_step_rearm_mg;
    uint16_t quiet_rearm_ms;

    uint16_t pre_trigger_min_interval_ms;
    uint16_t hard_rearm_ms;

    bool twist_suppress_enable;
    imu_pre_trigger_twist_axis_t twist_axis;
    uint16_t twist_axis_dps_min;
    uint16_t twist_plane_dps_max;
} imu_pre_trigger_marker_config_t;

typedef struct
{
    uint32_t marker_count;
    uint32_t pre_swing_trigger_count;
    uint32_t gyro_rising_marker_count;
    uint32_t acc_step_rising_marker_count;
    uint32_t both_rising_count;
    uint32_t suppressed_by_min_interval_count;
    uint32_t pure_twist_suppressed_count;
    uint32_t pending_candidate_count;
    uint32_t pending_confirmed_count;
    uint32_t pending_rejected_count;
    uint32_t pending_timeout_count;
    uint32_t rejected_acc_only_count;
    uint32_t confirmed_by_gyro_peak_count;
    uint32_t confirmed_by_gyro_acc_support_count;
    uint32_t pending_suppressed_twist_count;
    uint32_t pending_suppressed_interval_count;
    uint32_t rearm_by_quiet_count;
    uint32_t rearm_by_timeout_count;
    uint64_t last_marker_ts_us;
    uint32_t last_marker_sample_seq;
} imu_pre_trigger_marker_stats_t;

const imu_pre_trigger_marker_config_t *imu_pre_trigger_marker_default_config(void);
imu_status_t imu_pre_trigger_marker_reset(const imu_pre_trigger_marker_config_t *cfg);
imu_status_t imu_pre_trigger_marker_on_sample(
    const imu_swing_detector_input_sample_t *sample);
imu_status_t imu_pre_trigger_marker_finish(uint64_t end_ts_us);
void imu_pre_trigger_marker_get_stats(imu_pre_trigger_marker_stats_t *out);
void imu_pre_trigger_marker_log_capture_summary(void);
void imu_pre_trigger_marker_log_config(void);
void imu_pre_trigger_marker_log_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_PRE_TRIGGER_MARKER_H */
