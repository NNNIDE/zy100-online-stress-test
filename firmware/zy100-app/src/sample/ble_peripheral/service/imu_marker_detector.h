#ifndef IMU_MARKER_DETECTOR_H
#define IMU_MARKER_DETECTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../common/imu_common.h"

#define IMU_MARKER_DETECTOR_EVENT_FLAG_GYRO_RISE 0x0001U
#define IMU_MARKER_DETECTOR_EVENT_FLAG_ACC_RISE  0x0002U
#define IMU_MARKER_DETECTOR_EVENT_FLAG_CONFIRMED 0x0100U

typedef struct
{
    uint32_t seq;
    uint32_t t_us;
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
} imu_marker_detector_sample_t;

typedef struct
{
    uint32_t t_us;
    uint32_t seq;
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    uint16_t score;
    uint16_t flags;
} imu_marker_detector_event_t;

typedef struct
{
    uint32_t sample_count;
    uint32_t candidate_count;
    uint32_t marker_count;
    uint32_t dropped_cooldown_count;
    uint32_t rejected_timeout_count;
} imu_marker_detector_stats_t;

imu_status_t imu_marker_detector_reset(void);
imu_status_t imu_marker_detector_reset_runtime_state_preserve_stats(void);
imu_status_t imu_marker_detector_process_sample(const imu_marker_detector_sample_t *sample,
                                                bool *event_valid,
                                                imu_marker_detector_event_t *event);
void imu_marker_detector_get_stats(imu_marker_detector_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* IMU_MARKER_DETECTOR_H */
