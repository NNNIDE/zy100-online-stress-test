#include "zy100_mag_calibrator.h"

#include <math.h>
#include <string.h>

/* Engineering profile.  These dimensionless gates must be frozen against
 * real assembled-product datasets before production release. */
#define ZY100_MAG_CAL_RAW_LIMIT          1048576UL
#define ZY100_MAG_CAL_RAW_ZERO           524288.0f
#define ZY100_MAG_CAL_COUNTS_PER_GAUSS   16384.0f
#define ZY100_MAG_CAL_MIN_SAMPLES         65UL
#define ZY100_MAG_CAL_REQUIRED_MOVEMENTS  64UL
#define ZY100_MAG_CAL_MOTION_COUNTS       256L
#define ZY100_MAG_CAL_AXIS_SPAN_COUNTS   8192UL
#define ZY100_MAG_CAL_PIVOT_EPSILON      1.0e-8f
#define ZY100_MAG_CAL_MAX_PIVOT_RATIO    1.0e6f
#define ZY100_MAG_CAL_JACOBI_ITERATIONS  12U

static bool zy100_mag_cal_bias_valid(const float bias[3]);

static float zy100_mag_cal_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool zy100_mag_cal_finite(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7F800000UL) != 0x7F800000UL;
}

static float zy100_mag_cal_sqrt(float value)
{
    float estimate;
    uint8_t iteration;

    if (!(value > 0.0f))
    {
        return 0.0f;
    }
    estimate = (value > 1.0f) ? value : 1.0f;
    for (iteration = 0U; iteration < 10U; iteration++)
    {
        estimate = 0.5f * (estimate + value / estimate);
    }
    return estimate;
}

static float zy100_mag_cal_cbrt(float value)
{
    float estimate = 1.0f;
    uint8_t iteration;

    if (!(value > 0.0f))
    {
        return 0.0f;
    }
    if (value > 1.0f)
    {
        estimate = value;
    }
    for (iteration = 0U; iteration < 12U; iteration++)
    {
        estimate = (2.0f * estimate + value / (estimate * estimate)) / 3.0f;
    }
    return estimate;
}

void zy100_mag_calibrator_reset(zy100_mag_calibrator_t *calibrator)
{
    uint8_t axis;

    if (calibrator == NULL)
    {
        return;
    }
    memset(calibrator, 0, sizeof(*calibrator));
    for (axis = 0U; axis < 3U; axis++)
    {
        calibrator->min_value[axis] = (float)ZY100_MAG_CAL_RAW_LIMIT;
        calibrator->max_value[axis] = 0.0f;
    }
}

bool zy100_mag_calibrator_add(zy100_mag_calibrator_t *calibrator,
                              uint32_t raw_x,
                              uint32_t raw_y,
                              uint32_t raw_z)
{
    float value[3];
    float feature[ZY100_MAG_CAL_PARAMETER_COUNT];
    float midpoint[3];
    uint8_t row;
    uint8_t column;
    uint8_t octant = 0U;
    int32_t raw_delta[3];
    uint32_t dominant = 0UL;
    uint8_t dominant_axis = 0U;
    uint32_t next_count;

    if ((calibrator == NULL) ||
        (raw_x >= ZY100_MAG_CAL_RAW_LIMIT) ||
        (raw_y >= ZY100_MAG_CAL_RAW_LIMIT) ||
        (raw_z >= ZY100_MAG_CAL_RAW_LIMIT))
    {
        return false;
    }
    value[0] = (float)raw_x;
    value[1] = (float)raw_y;
    value[2] = (float)raw_z;
    if (!calibrator->has_baseline)
    {
        calibrator->last_accepted_raw[0] = raw_x;
        calibrator->last_accepted_raw[1] = raw_y;
        calibrator->last_accepted_raw[2] = raw_z;
        calibrator->has_baseline = true;
    }
    else
    {
        const uint32_t raw[3] = {raw_x, raw_y, raw_z};
        for (row = 0U; row < 3U; row++)
        {
            uint32_t magnitude;
            raw_delta[row] = (int32_t)raw[row] -
                             (int32_t)calibrator->last_accepted_raw[row];
            magnitude = (raw_delta[row] < 0) ?
                        (uint32_t)(-raw_delta[row]) :
                        (uint32_t)raw_delta[row];
            if (magnitude > dominant)
            {
                dominant = magnitude;
                dominant_axis = row;
            }
        }
        if (dominant < (uint32_t)ZY100_MAG_CAL_MOTION_COUNTS)
        {
            return true;
        }
        calibrator->last_accepted_raw[0] = raw_x;
        calibrator->last_accepted_raw[1] = raw_y;
        calibrator->last_accepted_raw[2] = raw_z;
        calibrator->movement_count++;
        {
            uint8_t direction_bit = (uint8_t)(dominant_axis * 2U);
            if (raw_delta[dominant_axis] >= 0)
            {
                direction_bit++;
            }
            calibrator->direction_mask |=
                (uint8_t)(1U << direction_bit);
        }
    }
    for (row = 0U; row < 3U; row++)
    {
        if (value[row] < calibrator->min_value[row])
        {
            calibrator->min_value[row] = value[row];
        }
        if (value[row] > calibrator->max_value[row])
        {
            calibrator->max_value[row] = value[row];
        }
        midpoint[row] = 0.5f * (calibrator->min_value[row] +
                                calibrator->max_value[row]);
        if (value[row] >= midpoint[row])
        {
            octant |= (uint8_t)(1U << row);
        }
        /* MMC5603 20-bit output: zero field is 524288 counts and
         * sensitivity is 16384 counts/G.  Centering before forming the
         * normal equations keeps the Float32 solve well conditioned. */
        value[row] = (value[row] - ZY100_MAG_CAL_RAW_ZERO) /
                     ZY100_MAG_CAL_COUNTS_PER_GAUSS;
    }
    calibrator->coverage_mask |= (uint8_t)(1U << octant);

    next_count = calibrator->sample_count + 1UL;
    feature[0] = value[0] * value[0];
    feature[1] = value[1] * value[1];
    feature[2] = value[2] * value[2];
    feature[3] = 2.0f * value[0] * value[1];
    feature[4] = 2.0f * value[0] * value[2];
    feature[5] = 2.0f * value[1] * value[2];
    feature[6] = 2.0f * value[0];
    feature[7] = 2.0f * value[1];
    feature[8] = 2.0f * value[2];
    for (row = 0U; row < ZY100_MAG_CAL_PARAMETER_COUNT; row++)
    {
        for (column = 0U; column < ZY100_MAG_CAL_PARAMETER_COUNT; column++)
        {
            calibrator->normal[row][column] += feature[row] * feature[column];
        }
        calibrator->normal[row][ZY100_MAG_CAL_PARAMETER_COUNT] += feature[row];
    }
    calibrator->sample_count = next_count;
    return true;
}

uint8_t zy100_mag_calibrator_progress(const zy100_mag_calibrator_t *calibrator)
{
    uint8_t directions = 0U;
    uint8_t bit;
    uint8_t axis;
    uint32_t movement_progress;
    uint32_t direction_progress;
    uint32_t span_total = 0UL;
    uint32_t progress;

    if (calibrator == NULL)
    {
        return 0U;
    }
    if (zy100_mag_calibrator_ready_status(calibrator) ==
        ZY100_MAG_CAL_READY)
    {
        return 100U;
    }
    for (bit = 0U; bit < 6U; bit++)
    {
        if ((calibrator->direction_mask & (uint8_t)(1U << bit)) != 0U)
        {
            directions++;
        }
    }
    for (axis = 0U; axis < 3U; axis++)
    {
        uint32_t span = (calibrator->max_value[axis] >
                         calibrator->min_value[axis]) ?
            (uint32_t)(calibrator->max_value[axis] -
                       calibrator->min_value[axis]) : 0UL;
        span_total += (span > ZY100_MAG_CAL_AXIS_SPAN_COUNTS) ?
                      ZY100_MAG_CAL_AXIS_SPAN_COUNTS : span;
    }
    movement_progress =
        ((calibrator->movement_count > ZY100_MAG_CAL_REQUIRED_MOVEMENTS) ?
         ZY100_MAG_CAL_REQUIRED_MOVEMENTS : calibrator->movement_count) *
        25UL / ZY100_MAG_CAL_REQUIRED_MOVEMENTS;
    direction_progress = (uint32_t)directions * 50UL / 6UL;
    progress = movement_progress + direction_progress +
        (span_total * 25UL /
         (3UL * ZY100_MAG_CAL_AXIS_SPAN_COUNTS));
    return (uint8_t)((progress > 99UL) ? 99UL : progress);
}

zy100_mag_cal_ready_status_t zy100_mag_calibrator_ready_status(
    const zy100_mag_calibrator_t *calibrator)
{
    uint8_t axis;

    if (calibrator == NULL)
    {
        return ZY100_MAG_CAL_NOT_READY_ARGUMENT;
    }
    if (calibrator->movement_count < ZY100_MAG_CAL_REQUIRED_MOVEMENTS)
    {
        return ZY100_MAG_CAL_NOT_READY_SAMPLES;
    }
    if ((calibrator->direction_mask & 0x3FU) != 0x3FU)
    {
        return ZY100_MAG_CAL_NOT_READY_COVERAGE;
    }
    for (axis = 0U; axis < 3U; axis++)
    {
        if (!(calibrator->max_value[axis] > calibrator->min_value[axis]) ||
            ((calibrator->max_value[axis] -
              calibrator->min_value[axis]) <
             (float)ZY100_MAG_CAL_AXIS_SPAN_COUNTS))
        {
            return ZY100_MAG_CAL_NOT_READY_AXIS;
        }
    }
    return ZY100_MAG_CAL_READY;
}

bool zy100_mag_calibrator_ready(const zy100_mag_calibrator_t *calibrator)
{
    return zy100_mag_calibrator_ready_status(calibrator) ==
           ZY100_MAG_CAL_READY;
}

static bool zy100_mag_cal_solve_linear_n(float matrix[9][10],
                                         uint8_t count,
                                         float solution[9],
                                         float *pivot_ratio)
{
    uint8_t column;
    uint8_t row;
    uint8_t pivot_row;
    uint8_t index;
    float min_pivot = 1.0e30f;
    float max_pivot = 0.0f;

    if ((count == 0U) || (count > 9U))
    {
        return false;
    }
    for (column = 0U; column < count; column++)
    {
        float pivot_abs = 0.0f;
        pivot_row = column;
        for (row = column; row < count; row++)
        {
            float candidate = zy100_mag_cal_abs(matrix[row][column]);
            if (candidate > pivot_abs)
            {
                pivot_abs = candidate;
                pivot_row = row;
            }
        }
        if (!(pivot_abs > ZY100_MAG_CAL_PIVOT_EPSILON))
        {
            return false;
        }
        if (pivot_row != column)
        {
            for (index = column; index <= count; index++)
            {
                float temporary = matrix[column][index];
                matrix[column][index] = matrix[pivot_row][index];
                matrix[pivot_row][index] = temporary;
            }
        }
        if (pivot_abs < min_pivot)
        {
            min_pivot = pivot_abs;
        }
        if (pivot_abs > max_pivot)
        {
            max_pivot = pivot_abs;
        }
        {
            float pivot = matrix[column][column];
            for (index = column; index <= count; index++)
            {
                matrix[column][index] /= pivot;
            }
        }
        for (row = 0U; row < count; row++)
        {
            float factor;
            if (row == column)
            {
                continue;
            }
            factor = matrix[row][column];
            for (index = column; index <= count; index++)
            {
                matrix[row][index] -= factor * matrix[column][index];
            }
        }
    }
    for (row = 0U; row < count; row++)
    {
        solution[row] = matrix[row][count];
        if (!zy100_mag_cal_finite(solution[row]))
        {
            return false;
        }
    }
    *pivot_ratio = max_pivot / min_pivot;
    return zy100_mag_cal_finite(*pivot_ratio) &&
           (*pivot_ratio <= ZY100_MAG_CAL_MAX_PIVOT_RATIO);
}

static bool zy100_mag_cal_inverse3(const float input[9], float output[9])
{
    float determinant =
        input[0] * (input[4] * input[8] - input[5] * input[7]) -
        input[1] * (input[3] * input[8] - input[5] * input[6]) +
        input[2] * (input[3] * input[7] - input[4] * input[6]);
    if (!(zy100_mag_cal_abs(determinant) > ZY100_MAG_CAL_PIVOT_EPSILON))
    {
        return false;
    }
    output[0] = (input[4] * input[8] - input[5] * input[7]) / determinant;
    output[1] = (input[2] * input[7] - input[1] * input[8]) / determinant;
    output[2] = (input[1] * input[5] - input[2] * input[4]) / determinant;
    output[3] = (input[5] * input[6] - input[3] * input[8]) / determinant;
    output[4] = (input[0] * input[8] - input[2] * input[6]) / determinant;
    output[5] = (input[2] * input[3] - input[0] * input[5]) / determinant;
    output[6] = (input[3] * input[7] - input[4] * input[6]) / determinant;
    output[7] = (input[1] * input[6] - input[0] * input[7]) / determinant;
    output[8] = (input[0] * input[4] - input[1] * input[3]) / determinant;
    return true;
}

static bool zy100_mag_cal_symmetric_sqrt(const float input[9], float output[9])
{
    float a[9];
    float vectors[9] = {1.0f, 0.0f, 0.0f,
                        0.0f, 1.0f, 0.0f,
                        0.0f, 0.0f, 1.0f};
    uint8_t iteration;
    uint8_t i;
    uint8_t j;
    uint8_t k;

    memcpy(a, input, sizeof(a));
    for (iteration = 0U; iteration < ZY100_MAG_CAL_JACOBI_ITERATIONS; iteration++)
    {
        uint8_t p = 0U;
        uint8_t q = 1U;
        float largest = zy100_mag_cal_abs(a[1]);
        if (zy100_mag_cal_abs(a[2]) > largest)
        {
            p = 0U; q = 2U; largest = zy100_mag_cal_abs(a[2]);
        }
        if (zy100_mag_cal_abs(a[5]) > largest)
        {
            p = 1U; q = 2U; largest = zy100_mag_cal_abs(a[5]);
        }
        if (largest <= ZY100_MAG_CAL_PIVOT_EPSILON)
        {
            break;
        }
        {
            float tau = (a[q * 3U + q] - a[p * 3U + p]) /
                        (2.0f * a[p * 3U + q]);
            float t = ((tau >= 0.0f) ? 1.0f : -1.0f) /
                      (zy100_mag_cal_abs(tau) +
                       zy100_mag_cal_sqrt(1.0f + tau * tau));
            float cosine = 1.0f / zy100_mag_cal_sqrt(1.0f + t * t);
            float sine = t * cosine;
            float app = a[p * 3U + p];
            float aqq = a[q * 3U + q];
            float apq = a[p * 3U + q];
            for (k = 0U; k < 3U; k++)
            {
                if ((k != p) && (k != q))
                {
                    float akp = a[k * 3U + p];
                    float akq = a[k * 3U + q];
                    a[k * 3U + p] = cosine * akp - sine * akq;
                    a[p * 3U + k] = a[k * 3U + p];
                    a[k * 3U + q] = sine * akp + cosine * akq;
                    a[q * 3U + k] = a[k * 3U + q];
                }
                {
                    float vkp = vectors[k * 3U + p];
                    float vkq = vectors[k * 3U + q];
                    vectors[k * 3U + p] = cosine * vkp - sine * vkq;
                    vectors[k * 3U + q] = sine * vkp + cosine * vkq;
                }
            }
            a[p * 3U + p] = app - t * apq;
            a[q * 3U + q] = aqq + t * apq;
            a[p * 3U + q] = 0.0f;
            a[q * 3U + p] = 0.0f;
        }
    }
    if (!(a[0] > 0.0f) || !(a[4] > 0.0f) || !(a[8] > 0.0f))
    {
        return false;
    }
    memset(output, 0, 9U * sizeof(float));
    for (i = 0U; i < 3U; i++)
    {
        float root = zy100_mag_cal_sqrt(a[i * 3U + i]);
        for (j = 0U; j < 3U; j++)
        {
            for (k = 0U; k < 3U; k++)
            {
                output[j * 3U + k] += vectors[j * 3U + i] * root *
                                      vectors[k * 3U + i];
            }
        }
    }
    return true;
}

zy100_mag_cal_solve_status_t zy100_mag_calibrator_solve_ex(
    const zy100_mag_calibrator_t *calibrator,
    zy100_mag_cal_result_t *result)
{
    float work[9][10];
    float parameter[9];
    float shape[9];
    float inverse[9];
    float center[3];
    float linear[3];
    float pivot_ratio;
    float scale;
    float determinant;
    float normalization;
    float sse;
    uint8_t row;
    uint8_t column;

    if ((calibrator == NULL) || (result == NULL))
    {
        return ZY100_MAG_CAL_SOLVE_INVALID_ARGUMENT;
    }
    if (!zy100_mag_calibrator_ready(calibrator))
    {
        return ZY100_MAG_CAL_SOLVE_NOT_READY;
    }
    memcpy(work, calibrator->normal, sizeof(work));
    if (!zy100_mag_cal_solve_linear_n(work, 9U, parameter, &pivot_ratio))
    {
        return ZY100_MAG_CAL_SOLVE_LINEAR_SYSTEM;
    }
    shape[0] = parameter[0]; shape[1] = parameter[3]; shape[2] = parameter[4];
    shape[3] = parameter[3]; shape[4] = parameter[1]; shape[5] = parameter[5];
    shape[6] = parameter[4]; shape[7] = parameter[5]; shape[8] = parameter[2];
    linear[0] = parameter[6];
    linear[1] = parameter[7];
    linear[2] = parameter[8];
    if (!zy100_mag_cal_inverse3(shape, inverse))
    {
        return ZY100_MAG_CAL_SOLVE_SHAPE_INVERSE;
    }
    for (row = 0U; row < 3U; row++)
    {
        center[row] = 0.0f;
        for (column = 0U; column < 3U; column++)
        {
            center[row] -= inverse[row * 3U + column] * linear[column];
        }
    }
    scale = 1.0f;
    for (row = 0U; row < 3U; row++)
    {
        for (column = 0U; column < 3U; column++)
        {
            scale += center[row] * shape[row * 3U + column] * center[column];
        }
    }
    if (!(scale > ZY100_MAG_CAL_PIVOT_EPSILON))
    {
        return ZY100_MAG_CAL_SOLVE_NON_POSITIVE_SCALE;
    }
    for (row = 0U; row < 9U; row++)
    {
        shape[row] /= scale;
    }
    if (!zy100_mag_cal_symmetric_sqrt(shape, result->matrix))
    {
        return ZY100_MAG_CAL_SOLVE_NON_POSITIVE_ELLIPSOID;
    }
    determinant =
        result->matrix[0] * (result->matrix[4] * result->matrix[8] -
                             result->matrix[5] * result->matrix[7]) -
        result->matrix[1] * (result->matrix[3] * result->matrix[8] -
                             result->matrix[5] * result->matrix[6]) +
        result->matrix[2] * (result->matrix[3] * result->matrix[7] -
                             result->matrix[4] * result->matrix[6]);
    normalization = zy100_mag_cal_cbrt(determinant);
    if (!(normalization > ZY100_MAG_CAL_PIVOT_EPSILON))
    {
        return ZY100_MAG_CAL_SOLVE_INVALID_NORMALIZATION;
    }
    for (row = 0U; row < 9U; row++)
    {
        result->matrix[row] /= normalization;
        if (!zy100_mag_cal_finite(result->matrix[row]))
        {
            return ZY100_MAG_CAL_SOLVE_NON_FINITE_RESULT;
        }
    }
    for (row = 0U; row < 3U; row++)
    {
        result->bias[row] = center[row] * ZY100_MAG_CAL_COUNTS_PER_GAUSS +
                            ZY100_MAG_CAL_RAW_ZERO;
    }
    if (!zy100_mag_cal_bias_valid(result->bias))
    {
        return ZY100_MAG_CAL_SOLVE_NON_FINITE_RESULT;
    }

    sse = (float)calibrator->sample_count;
    for (row = 0U; row < 9U; row++)
    {
        sse -= 2.0f * parameter[row] * calibrator->normal[row][9];
        for (column = 0U; column < 9U; column++)
        {
            sse += parameter[row] * calibrator->normal[row][column] *
                   parameter[column];
        }
    }
    if (sse < 0.0f)
    {
        sse = 0.0f;
    }
    result->algebraic_rms = zy100_mag_cal_sqrt(
                                sse / (float)calibrator->sample_count);
    if (!zy100_mag_cal_finite(result->algebraic_rms))
    {
        return ZY100_MAG_CAL_SOLVE_NON_FINITE_RESULT;
    }
    result->sample_count = calibrator->sample_count;
    result->coverage_mask = calibrator->coverage_mask;
    result->direction_mask = calibrator->direction_mask;
    result->model = ZY100_MAG_CAL_MODEL_FULL;
    result->quality = (uint16_t)(1000.0f /
                      (1.0f + 100.0f * result->algebraic_rms));
    memset(work, 0, sizeof(work));
    return ZY100_MAG_CAL_SOLVE_OK;
}

static bool zy100_mag_cal_bias_valid(const float bias[3])
{
    uint8_t axis;

    for (axis = 0U; axis < 3U; axis++)
    {
        if (!zy100_mag_cal_finite(bias[axis]) ||
            !(bias[axis] >= 0.0f) ||
            !(bias[axis] < (float)ZY100_MAG_CAL_RAW_LIMIT))
        {
            return false;
        }
    }
    return true;
}

static float zy100_mag_cal_subset_sse(
    const zy100_mag_calibrator_t *calibrator,
    const uint8_t *indices,
    uint8_t count,
    const float *parameter)
{
    float sse = (float)calibrator->sample_count;
    uint8_t row;
    uint8_t column;

    for (row = 0U; row < count; row++)
    {
        sse -= 2.0f * parameter[row] *
               calibrator->normal[indices[row]][9];
        for (column = 0U; column < count; column++)
        {
            sse += parameter[row] *
                   calibrator->normal[indices[row]][indices[column]] *
                   parameter[column];
        }
    }
    return (sse > 0.0f) ? sse : 0.0f;
}

zy100_mag_cal_solve_status_t zy100_mag_calibrator_solve_axis_aligned_ex(
    const zy100_mag_calibrator_t *calibrator,
    zy100_mag_cal_result_t *result)
{
    static const uint8_t indices[6] = {0U, 1U, 2U, 6U, 7U, 8U};
    float work[9][10];
    float parameter[9] = {0.0f};
    float pivot_ratio;
    float center[3];
    float diagonal[3];
    float scale = 1.0f;
    float determinant;
    float normalization;
    float sse;
    uint8_t row;
    uint8_t column;

    if ((calibrator == NULL) || (result == NULL))
    {
        return ZY100_MAG_CAL_SOLVE_INVALID_ARGUMENT;
    }
    if (calibrator->sample_count < ZY100_MAG_CAL_MIN_SAMPLES)
    {
        return ZY100_MAG_CAL_SOLVE_NOT_READY;
    }
    memset(work, 0, sizeof(work));
    for (row = 0U; row < 6U; row++)
    {
        for (column = 0U; column < 6U; column++)
        {
            work[row][column] =
                calibrator->normal[indices[row]][indices[column]];
        }
        work[row][6] = calibrator->normal[indices[row]][9];
    }
    if (!zy100_mag_cal_solve_linear_n(work, 6U, parameter, &pivot_ratio))
    {
        return ZY100_MAG_CAL_SOLVE_LINEAR_SYSTEM;
    }
    for (row = 0U; row < 3U; row++)
    {
        if (!(zy100_mag_cal_abs(parameter[row]) >
              ZY100_MAG_CAL_PIVOT_EPSILON))
        {
            return ZY100_MAG_CAL_SOLVE_SHAPE_INVERSE;
        }
        center[row] = -parameter[row + 3U] / parameter[row];
        scale += center[row] * center[row] * parameter[row];
    }
    if (!(scale > ZY100_MAG_CAL_PIVOT_EPSILON))
    {
        return ZY100_MAG_CAL_SOLVE_NON_POSITIVE_SCALE;
    }
    for (row = 0U; row < 3U; row++)
    {
        diagonal[row] = parameter[row] / scale;
        if (!(diagonal[row] > ZY100_MAG_CAL_PIVOT_EPSILON))
        {
            return ZY100_MAG_CAL_SOLVE_NON_POSITIVE_ELLIPSOID;
        }
        diagonal[row] = zy100_mag_cal_sqrt(diagonal[row]);
    }
    determinant = diagonal[0] * diagonal[1] * diagonal[2];
    normalization = zy100_mag_cal_cbrt(determinant);
    if (!(normalization > ZY100_MAG_CAL_PIVOT_EPSILON))
    {
        return ZY100_MAG_CAL_SOLVE_INVALID_NORMALIZATION;
    }
    memset(result, 0, sizeof(*result));
    for (row = 0U; row < 3U; row++)
    {
        result->bias[row] = center[row] * ZY100_MAG_CAL_COUNTS_PER_GAUSS +
                            ZY100_MAG_CAL_RAW_ZERO;
        result->matrix[row * 3U + row] = diagonal[row] / normalization;
    }
    if (!zy100_mag_cal_bias_valid(result->bias))
    {
        return ZY100_MAG_CAL_SOLVE_NON_FINITE_RESULT;
    }
    sse = zy100_mag_cal_subset_sse(calibrator, indices, 6U, parameter);
    result->algebraic_rms = zy100_mag_cal_sqrt(
        sse / (float)calibrator->sample_count);
    if (!zy100_mag_cal_finite(result->algebraic_rms))
    {
        return ZY100_MAG_CAL_SOLVE_NON_FINITE_RESULT;
    }
    result->quality = (uint16_t)(1000.0f /
        (1.0f + 100.0f * result->algebraic_rms));
    result->sample_count = calibrator->sample_count;
    result->coverage_mask = calibrator->coverage_mask;
    result->direction_mask = calibrator->direction_mask;
    result->model = ZY100_MAG_CAL_MODEL_AXIS_ALIGNED;
    memset(work, 0, sizeof(work));
    return ZY100_MAG_CAL_SOLVE_OK;
}

zy100_mag_cal_solve_status_t zy100_mag_calibrator_solve_hard_iron_ex(
    const zy100_mag_calibrator_t *calibrator,
    zy100_mag_cal_result_t *result)
{
    static const uint8_t linear_indices[3] = {6U, 7U, 8U};
    float work[9][10];
    float parameter[9] = {0.0f};
    float pivot_ratio;
    float radius_squared;
    float btb = 0.0f;
    float atb[4];
    float sse;
    uint8_t row;
    uint8_t column;
    uint8_t qrow;
    uint8_t qcolumn;

    if ((calibrator == NULL) || (result == NULL))
    {
        return ZY100_MAG_CAL_SOLVE_INVALID_ARGUMENT;
    }
    if (calibrator->sample_count < ZY100_MAG_CAL_MIN_SAMPLES)
    {
        return ZY100_MAG_CAL_SOLVE_NOT_READY;
    }
    memset(work, 0, sizeof(work));
    for (row = 0U; row < 3U; row++)
    {
        for (column = 0U; column < 3U; column++)
        {
            work[row][column] = calibrator->normal
                [linear_indices[row]][linear_indices[column]];
        }
        work[row][3] = calibrator->normal[linear_indices[row]][9];
        work[3][row] = work[row][3];
        atb[row] = calibrator->normal[linear_indices[row]][0] +
                   calibrator->normal[linear_indices[row]][1] +
                   calibrator->normal[linear_indices[row]][2];
        work[row][4] = atb[row];
    }
    work[3][3] = (float)calibrator->sample_count;
    atb[3] = calibrator->normal[0][9] +
             calibrator->normal[1][9] +
             calibrator->normal[2][9];
    work[3][4] = atb[3];
    if (!zy100_mag_cal_solve_linear_n(work, 4U, parameter, &pivot_ratio))
    {
        return ZY100_MAG_CAL_SOLVE_LINEAR_SYSTEM;
    }
    radius_squared = parameter[3];
    for (row = 0U; row < 3U; row++)
    {
        radius_squared += parameter[row] * parameter[row];
    }
    if (!(radius_squared > ZY100_MAG_CAL_PIVOT_EPSILON))
    {
        return ZY100_MAG_CAL_SOLVE_NON_POSITIVE_SCALE;
    }
    memset(result, 0, sizeof(*result));
    for (row = 0U; row < 3U; row++)
    {
        result->bias[row] = parameter[row] *
                            ZY100_MAG_CAL_COUNTS_PER_GAUSS +
                            ZY100_MAG_CAL_RAW_ZERO;
        result->matrix[row * 3U + row] = 1.0f;
    }
    if (!zy100_mag_cal_bias_valid(result->bias))
    {
        return ZY100_MAG_CAL_SOLVE_NON_FINITE_RESULT;
    }
    for (qrow = 0U; qrow < 3U; qrow++)
    {
        for (qcolumn = 0U; qcolumn < 3U; qcolumn++)
        {
            btb += calibrator->normal[qrow][qcolumn];
        }
    }
    sse = btb;
    for (row = 0U; row < 4U; row++)
    {
        sse -= 2.0f * parameter[row] * atb[row];
        for (column = 0U; column < 4U; column++)
        {
            float normal_value;
            if ((row < 3U) && (column < 3U))
            {
                normal_value = calibrator->normal
                    [linear_indices[row]][linear_indices[column]];
            }
            else if ((row < 3U) && (column == 3U))
            {
                normal_value = calibrator->normal[linear_indices[row]][9];
            }
            else if ((row == 3U) && (column < 3U))
            {
                normal_value = calibrator->normal[linear_indices[column]][9];
            }
            else
            {
                normal_value = (float)calibrator->sample_count;
            }
            sse += parameter[row] * normal_value * parameter[column];
        }
    }
    if (sse < 0.0f)
    {
        sse = 0.0f;
    }
    result->algebraic_rms = zy100_mag_cal_sqrt(
        sse / (float)calibrator->sample_count) / radius_squared;
    if (!zy100_mag_cal_finite(result->algebraic_rms))
    {
        return ZY100_MAG_CAL_SOLVE_NON_FINITE_RESULT;
    }
    result->quality = (uint16_t)(1000.0f /
        (1.0f + 100.0f * result->algebraic_rms));
    result->sample_count = calibrator->sample_count;
    result->coverage_mask = calibrator->coverage_mask;
    result->direction_mask = calibrator->direction_mask;
    result->model = ZY100_MAG_CAL_MODEL_HARD_IRON_ONLY;
    memset(work, 0, sizeof(work));
    return ZY100_MAG_CAL_SOLVE_OK;
}

bool zy100_mag_calibrator_solve(const zy100_mag_calibrator_t *calibrator,
                                zy100_mag_cal_result_t *result)
{
    return zy100_mag_calibrator_solve_ex(calibrator, result) ==
           ZY100_MAG_CAL_SOLVE_OK;
}
