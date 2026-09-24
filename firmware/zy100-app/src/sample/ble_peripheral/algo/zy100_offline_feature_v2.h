#ifndef ZY100_OFFLINE_FEATURE_V2_H
#define ZY100_OFFLINE_FEATURE_V2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../common/zy100_offline_v2_contract.h"

#define ZY100_OFFLINE_V2_SAMPLE_RATE_HZ        800U
#define ZY100_OFFLINE_V2_RING_SAMPLES          1280U
#define ZY100_OFFLINE_V2_WINDOW_SAMPLES        800U
#define ZY100_OFFLINE_V2_FEATURE_DIMENSIONS    128U
#define ZY100_OFFLINE_V2_FEATURE_Q12_BYTES     256U
#define ZY100_OFFLINE_V2_ALGO_WORKSPACE_MAX    (32UL * 1024UL)

#define ZY100_OFFLINE_V2_EVENT_FLAG_SHOCK_AVAILABLE 0x0001U
#define ZY100_OFFLINE_V2_EVENT_FLAG_VALID_HIT       0x0002U
#define ZY100_OFFLINE_V2_EVENT_FLAG_ACCEL_CLIPPING  0x0004U
#define ZY100_OFFLINE_V2_EVENT_FLAG_GYRO_CLIPPING   0x0008U
#define ZY100_OFFLINE_V2_EVENT_FLAG_Q12_SATURATION  0x0010U

typedef struct
{
    int16_t axis[6];
} zy100_offline_v2_imu_sample_t;

typedef enum
{
    ZY100_OFFLINE_V2_ALGO_OK = 0U,
    ZY100_OFFLINE_V2_ALGO_BAD_ARG,
    ZY100_OFFLINE_V2_ALGO_WORKSPACE_SMALL,
    ZY100_OFFLINE_V2_ALGO_QUEUE_OVERFLOW,
    ZY100_OFFLINE_V2_ALGO_RING_OVERRUN,
    ZY100_OFFLINE_V2_ALGO_NON_FINITE,
    ZY100_OFFLINE_V2_ALGO_EVENT_SINK_FULL,
} zy100_offline_v2_algo_status_t;

typedef struct
{
    uint32_t center_sample_index;
    uint32_t peak_mag2_raw;
    float shock_numerator;
    float shock_baseline;
    float shock_ratio;
    uint16_t sample_rate_hz;
    uint16_t quality_flags;
    uint8_t detector_version;
    uint8_t shock_version;
    uint8_t feature_version;
    uint8_t codec_version;
    int16_t feature_q12[ZY100_OFFLINE_V2_FEATURE_DIMENSIONS];
} zy100_offline_v2_event_t;

typedef bool (*zy100_offline_v2_event_sink_t)(
    const zy100_offline_v2_event_t *event,
    void *context);

typedef uint32_t (*zy100_offline_v2_time_us_t)(void);

typedef struct
{
    uint32_t detector_last_us;
    uint32_t detector_max_us;
    uint32_t shock_last_us;
    uint32_t shock_max_us;
    uint32_t feature_last_us;
    uint32_t feature_max_us;
    uint32_t local_peak_count;
    uint32_t legal_peak_count;
    uint32_t nms_selected_count;
    uint32_t shock_rejected_count;
    uint32_t rejected_window_count;
} zy100_offline_v2_algo_timing_t;

typedef struct zy100_offline_v2_algo zy100_offline_v2_algo_t;

uint32_t zy100_offline_v2_algo_workspace_bytes(void);
zy100_offline_v2_algo_t *zy100_offline_v2_algo_init(
    void *workspace,
    uint32_t workspace_bytes,
    zy100_offline_v2_event_sink_t sink,
    void *sink_context);
zy100_offline_v2_algo_status_t zy100_offline_v2_algo_push(
    zy100_offline_v2_algo_t *algo,
    const zy100_offline_v2_imu_sample_t *sample,
    bool discontinuity_before);
zy100_offline_v2_algo_status_t zy100_offline_v2_algo_finish(
    zy100_offline_v2_algo_t *algo,
    bool clean_end);
zy100_offline_v2_algo_status_t zy100_offline_v2_algo_status(
    const zy100_offline_v2_algo_t *algo);
uint32_t zy100_offline_v2_algo_event_count(
    const zy100_offline_v2_algo_t *algo);
void zy100_offline_v2_algo_set_time_source(
    zy100_offline_v2_algo_t *algo,
    zy100_offline_v2_time_us_t time_us);
bool zy100_offline_v2_algo_get_timing(
    const zy100_offline_v2_algo_t *algo,
    zy100_offline_v2_algo_timing_t *timing_out);

/* Public so the ties-to-even codec boundary can be host/golden tested. */
int16_t zy100_offline_v2_q12_encode(float value, bool *saturated);

#if defined(ZY100_OFFLINE_V2_HOST_TEST)
bool zy100_offline_v2_host_extract_window(
    void *workspace,
    uint32_t workspace_bytes,
    const zy100_offline_v2_imu_sample_t window[ZY100_OFFLINE_V2_WINDOW_SAMPLES],
    float feature_out[ZY100_OFFLINE_V2_FEATURE_DIMENSIONS],
    int16_t q12_out[ZY100_OFFLINE_V2_FEATURE_DIMENSIONS]);
bool zy100_offline_v2_host_shock_window(
    const zy100_offline_v2_imu_sample_t window[ZY100_OFFLINE_V2_WINDOW_SAMPLES],
    float *numerator_out,
    float *baseline_out,
    float *ratio_out,
    uint16_t *quality_flags_out);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ZY100_OFFLINE_FEATURE_V2_H */
