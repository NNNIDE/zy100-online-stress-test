#ifndef IMU_MOTION_METRICS_H
#define IMU_MOTION_METRICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define IMU_MOTION_METRICS_ACC_HISTORY_SIZE 32U

typedef struct
{
    uint64_t sample_ts_us;
    int32_t acc_x_mg;
    int32_t acc_y_mg;
    int32_t acc_z_mg;
} imu_motion_metrics_acc_history_sample_t;

typedef struct
{
    imu_motion_metrics_acc_history_sample_t samples[IMU_MOTION_METRICS_ACC_HISTORY_SIZE];
    uint32_t next;
    uint32_t count;
} imu_motion_metrics_acc_history_t;

int32_t imu_motion_metrics_raw_to_dps(int16_t raw);
int32_t imu_motion_metrics_raw_to_mg(int16_t raw);
uint32_t imu_motion_metrics_vector_norm_i32(int32_t x, int32_t y, int32_t z);
uint32_t imu_motion_metrics_calc_gyro_norm_dps(int16_t gyro_x_raw,
                                               int16_t gyro_y_raw,
                                               int16_t gyro_z_raw);
uint32_t imu_motion_metrics_calc_acc_step_mg(int16_t accel_x_raw,
                                             int16_t accel_y_raw,
                                             int16_t accel_z_raw,
                                             int16_t prev_accel_x_raw,
                                             int16_t prev_accel_y_raw,
                                             int16_t prev_accel_z_raw);
void imu_motion_metrics_acc_history_reset(imu_motion_metrics_acc_history_t *history);
uint32_t imu_motion_metrics_acc_history_update(
    imu_motion_metrics_acc_history_t *history,
    uint64_t sample_ts_us,
    int32_t acc_x_mg,
    int32_t acc_y_mg,
    int32_t acc_z_mg,
    uint16_t window_ms);

#ifdef __cplusplus
}
#endif

#endif /* IMU_MOTION_METRICS_H */
