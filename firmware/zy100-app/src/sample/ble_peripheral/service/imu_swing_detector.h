#ifndef IMU_SWING_DETECTOR_H
#define IMU_SWING_DETECTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../common/imu_common.h"

#define IMU_SWING_DETECTOR_EVENT_LOG_MAX 16U

#ifndef IMU_DETECTOR_TARGET_INPUT_HZ
#define IMU_DETECTOR_TARGET_INPUT_HZ 500U
#endif

typedef struct
{
    bool enable;
    uint16_t detector_input_hz;
    uint16_t decimation_factor;
    uint16_t acc_step_window_ms;
    uint16_t gyro_candidate_dps_min;
    uint16_t gyro_peak_dps_min;
    uint16_t confirm_window_ms;
    uint16_t quiet_gyro_dps;
    uint16_t quiet_acc_step_mg;
    uint16_t quiet_gap_ms;
    uint16_t min_swing_gap_ms;
} imu_swing_detector_config_t;

typedef struct
{
    uint64_t candidate_ts_us;
    uint32_t sample_seq;
    uint8_t axis_mask;
} imu_swing_detector_wom_candidate_t;

typedef struct
{
    uint32_t sample_seq;
    uint64_t sample_ts_us;

    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    uint16_t imu_fifo_ts16;
    bool imu_fifo_ts16_valid;
} imu_swing_detector_input_sample_t;

typedef struct
{
    uint32_t idx;
    uint64_t start_ts_us;
    uint64_t peak_ts_us;
    uint64_t end_ts_us;
    uint32_t duration_ms;

    uint32_t wom_count_inside_event;
    uint8_t merged_axis_mask;

    uint32_t gyro_peak_dps;
    uint32_t acc_delta_peak_mg;
    uint32_t acc_step_peak_mg;
    uint32_t jerk_peak_mg;
} imu_swing_detector_event_t;

typedef struct
{
    uint32_t detector_input_sample_count;
    uint32_t wom_candidate_count;
    uint32_t gyro_candidate_count;
    uint32_t software_swing_count;
    uint32_t gyro_confirmed_count;
    uint32_t false_candidate_count;
    uint32_t unconfirmed_candidate_count;
    uint32_t suppressed_by_min_gap_count;
    uint32_t software_event_log_count;
    uint32_t software_event_log_dropped_count;
} imu_swing_detector_stats_t;

const imu_swing_detector_config_t *imu_swing_detector_default_config(void);
uint16_t imu_swing_detector_input_hz(void);
uint16_t imu_swing_detector_decimation_factor(void);

imu_status_t imu_swing_detector_reset(const imu_swing_detector_config_t *cfg);
imu_status_t imu_swing_detector_on_wom_candidate(
    const imu_swing_detector_wom_candidate_t *candidate);
imu_status_t imu_swing_detector_on_sample(
    const imu_swing_detector_input_sample_t *sample);
imu_status_t imu_swing_detector_finish(uint64_t end_ts_us);

void imu_swing_detector_get_stats(imu_swing_detector_stats_t *out);
void imu_swing_detector_log_config(void);
void imu_swing_detector_log_stats(void);
void imu_swing_detector_log_events(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_SWING_DETECTOR_H */
