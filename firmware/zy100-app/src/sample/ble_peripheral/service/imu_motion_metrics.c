#include "imu_motion_metrics.h"

#include <stddef.h>
#include <string.h>

#define IMU_MOTION_METRICS_GYRO_LSB_X10_PER_DPS 164
#define IMU_MOTION_METRICS_ACCEL_LSB_PER_G      2048
#define IMU_MOTION_METRICS_MG_PER_G             1000

static uint32_t imu_motion_metrics_isqrt_u64(uint64_t value)
{
    uint64_t op = value;
    uint64_t res = 0ULL;
    uint64_t one = 1ULL << 62;

    while (one > op)
    {
        one >>= 2;
    }

    while (one != 0ULL)
    {
        if (op >= (res + one))
        {
            op -= (res + one);
            res = (res >> 1) + one;
        }
        else
        {
            res >>= 1;
        }
        one >>= 2;
    }

    if (res > 0xFFFFFFFFULL)
    {
        return 0xFFFFFFFFU;
    }
    return (uint32_t)res;
}

static uint64_t imu_motion_metrics_square_i32(int32_t value)
{
    int64_t wide = (int64_t)value;

    return (uint64_t)(wide * wide);
}

int32_t imu_motion_metrics_raw_to_dps(int16_t raw)
{
    int32_t scaled = ((int32_t)raw * 10);

    if (scaled >= 0)
    {
        scaled += (IMU_MOTION_METRICS_GYRO_LSB_X10_PER_DPS / 2);
    }
    else
    {
        scaled -= (IMU_MOTION_METRICS_GYRO_LSB_X10_PER_DPS / 2);
    }

    return (scaled / IMU_MOTION_METRICS_GYRO_LSB_X10_PER_DPS);
}

int32_t imu_motion_metrics_raw_to_mg(int16_t raw)
{
    int32_t scaled = ((int32_t)raw * IMU_MOTION_METRICS_MG_PER_G);

    if (scaled >= 0)
    {
        scaled += (IMU_MOTION_METRICS_ACCEL_LSB_PER_G / 2);
    }
    else
    {
        scaled -= (IMU_MOTION_METRICS_ACCEL_LSB_PER_G / 2);
    }

    return (scaled / IMU_MOTION_METRICS_ACCEL_LSB_PER_G);
}

uint32_t imu_motion_metrics_vector_norm_i32(int32_t x, int32_t y, int32_t z)
{
    uint64_t sum = imu_motion_metrics_square_i32(x);

    sum += imu_motion_metrics_square_i32(y);
    sum += imu_motion_metrics_square_i32(z);
    return imu_motion_metrics_isqrt_u64(sum);
}

uint32_t imu_motion_metrics_calc_gyro_norm_dps(int16_t gyro_x_raw,
                                               int16_t gyro_y_raw,
                                               int16_t gyro_z_raw)
{
    int32_t gyro_x_dps = imu_motion_metrics_raw_to_dps(gyro_x_raw);
    int32_t gyro_y_dps = imu_motion_metrics_raw_to_dps(gyro_y_raw);
    int32_t gyro_z_dps = imu_motion_metrics_raw_to_dps(gyro_z_raw);

    return imu_motion_metrics_vector_norm_i32(gyro_x_dps, gyro_y_dps, gyro_z_dps);
}

uint32_t imu_motion_metrics_calc_acc_step_mg(int16_t accel_x_raw,
                                             int16_t accel_y_raw,
                                             int16_t accel_z_raw,
                                             int16_t prev_accel_x_raw,
                                             int16_t prev_accel_y_raw,
                                             int16_t prev_accel_z_raw)
{
    int32_t acc_x_mg = imu_motion_metrics_raw_to_mg(accel_x_raw);
    int32_t acc_y_mg = imu_motion_metrics_raw_to_mg(accel_y_raw);
    int32_t acc_z_mg = imu_motion_metrics_raw_to_mg(accel_z_raw);
    int32_t prev_acc_x_mg = imu_motion_metrics_raw_to_mg(prev_accel_x_raw);
    int32_t prev_acc_y_mg = imu_motion_metrics_raw_to_mg(prev_accel_y_raw);
    int32_t prev_acc_z_mg = imu_motion_metrics_raw_to_mg(prev_accel_z_raw);

    return imu_motion_metrics_vector_norm_i32(acc_x_mg - prev_acc_x_mg,
                                              acc_y_mg - prev_acc_y_mg,
                                              acc_z_mg - prev_acc_z_mg);
}

void imu_motion_metrics_acc_history_reset(imu_motion_metrics_acc_history_t *history)
{
    if (history == NULL)
    {
        return;
    }

    memset(history, 0, sizeof(*history));
}

uint32_t imu_motion_metrics_acc_history_update(
    imu_motion_metrics_acc_history_t *history,
    uint64_t sample_ts_us,
    int32_t acc_x_mg,
    int32_t acc_y_mg,
    int32_t acc_z_mg,
    uint16_t window_ms)
{
    uint64_t window_us = (uint64_t)window_ms * 1000ULL;
    uint64_t best_age_us = 0ULL;
    const imu_motion_metrics_acc_history_sample_t *best = NULL;
    uint32_t acc_step_mg = 0U;
    uint32_t i;
    uint32_t slot;

    if (history == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < history->count; i++)
    {
        const imu_motion_metrics_acc_history_sample_t *candidate =
            &history->samples[i];
        uint64_t age_us;

        if (sample_ts_us < candidate->sample_ts_us)
        {
            continue;
        }

        age_us = sample_ts_us - candidate->sample_ts_us;
        if (age_us < window_us)
        {
            continue;
        }

        if ((best == NULL) || (age_us < best_age_us))
        {
            best = candidate;
            best_age_us = age_us;
        }
    }

    if (best != NULL)
    {
        acc_step_mg =
            imu_motion_metrics_vector_norm_i32(acc_x_mg - best->acc_x_mg,
                                               acc_y_mg - best->acc_y_mg,
                                               acc_z_mg - best->acc_z_mg);
    }

    slot = history->next;
    if (slot >= IMU_MOTION_METRICS_ACC_HISTORY_SIZE)
    {
        slot = 0U;
    }

    history->samples[slot].sample_ts_us = sample_ts_us;
    history->samples[slot].acc_x_mg = acc_x_mg;
    history->samples[slot].acc_y_mg = acc_y_mg;
    history->samples[slot].acc_z_mg = acc_z_mg;

    slot++;
    if (slot >= IMU_MOTION_METRICS_ACC_HISTORY_SIZE)
    {
        slot = 0U;
    }
    history->next = slot;

    if (history->count < IMU_MOTION_METRICS_ACC_HISTORY_SIZE)
    {
        history->count++;
    }

    return acc_step_mg;
}
