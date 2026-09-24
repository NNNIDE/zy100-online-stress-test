#ifndef ZY100_MAG_CALIBRATOR_H
#define ZY100_MAG_CALIBRATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define ZY100_MAG_CAL_PARAMETER_COUNT 9U

typedef struct
{
    float normal[ZY100_MAG_CAL_PARAMETER_COUNT]
                [ZY100_MAG_CAL_PARAMETER_COUNT + 1U];
    float min_value[3];
    float max_value[3];
    uint32_t last_accepted_raw[3];
    uint32_t sample_count;
    uint32_t movement_count;
    uint8_t coverage_mask;
    uint8_t direction_mask;
    bool has_baseline;
} zy100_mag_calibrator_t;

typedef enum
{
    ZY100_MAG_CAL_MODEL_NONE = 0U,
    ZY100_MAG_CAL_MODEL_HARD_IRON_ONLY = 1U,
    ZY100_MAG_CAL_MODEL_AXIS_ALIGNED = 2U,
    ZY100_MAG_CAL_MODEL_FULL = 3U,
} zy100_mag_cal_model_t;

typedef struct
{
    float bias[3];
    float matrix[9];
    float algebraic_rms;
    uint16_t quality;
    uint32_t sample_count;
    uint8_t coverage_mask;
    uint8_t direction_mask;
    zy100_mag_cal_model_t model;
} zy100_mag_cal_result_t;

typedef enum
{
    ZY100_MAG_CAL_SOLVE_OK = 0U,
    ZY100_MAG_CAL_SOLVE_INVALID_ARGUMENT,
    ZY100_MAG_CAL_SOLVE_NOT_READY,
    ZY100_MAG_CAL_SOLVE_LINEAR_SYSTEM,
    ZY100_MAG_CAL_SOLVE_SHAPE_INVERSE,
    ZY100_MAG_CAL_SOLVE_NON_POSITIVE_SCALE,
    ZY100_MAG_CAL_SOLVE_NON_POSITIVE_ELLIPSOID,
    ZY100_MAG_CAL_SOLVE_INVALID_NORMALIZATION,
    ZY100_MAG_CAL_SOLVE_NON_FINITE_RESULT,
} zy100_mag_cal_solve_status_t;

typedef enum
{
    ZY100_MAG_CAL_READY = 0U,
    ZY100_MAG_CAL_NOT_READY_ARGUMENT,
    ZY100_MAG_CAL_NOT_READY_SAMPLES,
    ZY100_MAG_CAL_NOT_READY_COVERAGE,
    ZY100_MAG_CAL_NOT_READY_AXIS,
} zy100_mag_cal_ready_status_t;

void zy100_mag_calibrator_reset(zy100_mag_calibrator_t *calibrator);
bool zy100_mag_calibrator_add(zy100_mag_calibrator_t *calibrator,
                              uint32_t raw_x,
                              uint32_t raw_y,
                              uint32_t raw_z);
uint8_t zy100_mag_calibrator_progress(const zy100_mag_calibrator_t *calibrator);
bool zy100_mag_calibrator_ready(const zy100_mag_calibrator_t *calibrator);
zy100_mag_cal_ready_status_t zy100_mag_calibrator_ready_status(
    const zy100_mag_calibrator_t *calibrator);
bool zy100_mag_calibrator_solve(const zy100_mag_calibrator_t *calibrator,
                                zy100_mag_cal_result_t *result);
zy100_mag_cal_solve_status_t zy100_mag_calibrator_solve_ex(
    const zy100_mag_calibrator_t *calibrator,
    zy100_mag_cal_result_t *result);
zy100_mag_cal_solve_status_t zy100_mag_calibrator_solve_axis_aligned_ex(
    const zy100_mag_calibrator_t *calibrator,
    zy100_mag_cal_result_t *result);
zy100_mag_cal_solve_status_t zy100_mag_calibrator_solve_hard_iron_ex(
    const zy100_mag_calibrator_t *calibrator,
    zy100_mag_cal_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_MAG_CALIBRATOR_H */
