#ifndef ZY100_EDGE_LITE_H
#define ZY100_EDGE_LITE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "zy100_edge_shadow.h"

#define ZY100_EDGE_LITE_API_AVAILABLE       1

#define ZY100_EDGE_LITE_OUT_NONE            0x00U
#define ZY100_EDGE_LITE_OUT_SUMMARY         0x01U
#define ZY100_EDGE_LITE_OUT_EVENT           0x02U
#define ZY100_EDGE_LITE_OUT_RAW_REQ         0x04U
#define ZY100_EDGE_LITE_OUT_RAW_START       0x08U

#define ZY100_EDGE_LITE_QUALITY_PRE_WINDOW_NOT_BUFFERED 0x00000001U
#define ZY100_EDGE_LITE_QUALITY_HAS_CLIP                0x00000002U
#define ZY100_EDGE_LITE_QUALITY_HAS_SENTINEL            0x00000004U
#define ZY100_EDGE_LITE_QUALITY_MAX_EVENT_FORCED        0x00000008U
#define ZY100_EDGE_LITE_QUALITY_ABNORMAL                0x00000010U

#define ZY100_EDGE_LITE_SHAPE_BIN_COUNT                  32U

typedef struct
{
    uint32_t summary_id;
    uint32_t start_sample;
    uint32_t end_sample;
    uint32_t start_t_us;
    uint32_t duration_samples;
    uint32_t energy_sum;
    uint32_t energy_peak;
    uint32_t gyro_peak;
    uint32_t acc_delta_peak;
    uint32_t clip_count;
    uint32_t sentinel_count;
    uint32_t candidate_count;
} zy100_edge_lite_summary_t;

typedef struct
{
    uint32_t event_id;
    uint32_t start_sample;
    uint32_t trigger_sample;
    uint32_t end_sample;
    uint32_t duration_samples;
    uint32_t trigger_energy;
    uint32_t peak_energy;
    uint32_t gyro_peak[3];
    uint32_t acc_peak[3];
    uint32_t gyro_axis_energy[3];
    uint32_t acc_axis_energy[3];
    uint32_t phase_energy[4];
    uint32_t clip_count;
    uint32_t sentinel_count;
    uint32_t quality_flags;
    uint32_t raw_requested;
    uint32_t raw_window_id;
    uint16_t shape_acc[ZY100_EDGE_LITE_SHAPE_BIN_COUNT];
    uint16_t shape_gyro[ZY100_EDGE_LITE_SHAPE_BIN_COUNT];
} zy100_edge_lite_event_t;

typedef struct zy100_edge_lite_output
{
    uint32_t flags;

    uint32_t summary_out;
    uint32_t event_out;
    uint32_t raw_req_out;
    uint32_t raw_start_out;

    uint32_t event_id;
    uint32_t start_sample;
    uint32_t trigger_sample;
    uint32_t end_sample;

    uint32_t peak_energy;
    uint32_t trigger_energy;
    uint32_t quality_flags;
    zy100_edge_lite_summary_t summary;
    zy100_edge_lite_event_t event;
} zy100_edge_lite_output_t;

typedef struct
{
    uint32_t sample_count;
    uint32_t summary_count;
    uint32_t event_count;
    uint32_t raw_req_count;

    uint32_t clip_count;
    uint32_t sentinel_count;
    uint32_t rejected_count;
    uint32_t cooldown_drop_count;
    uint32_t max_event_forced_count;

    uint32_t trigger_threshold;
    uint32_t quiet_threshold;
    uint32_t chunk_samples;
    uint32_t summary_period_samples;
    uint32_t cooldown_samples;

    uint32_t ctx_bytes;
    uint32_t workspace_bytes;
    uint32_t workspace_ok;
    uint32_t partial_summary_discarded;
    uint32_t partial_event_discarded;
} zy100_edge_lite_stats_t;

typedef enum
{
    ZY100_EDGE_LITE_STOP_NORMAL_END = 0U,
    ZY100_EDGE_LITE_STOP_DISCARD_PARTIAL = 1U,
} zy100_edge_lite_stop_mode_t;

bool zy100_edge_lite_init(void *workspace, uint32_t workspace_bytes);
void zy100_edge_lite_capture_start(void);
bool zy100_edge_lite_push_sample(const zy100_edge_shadow_sample_t *sample,
                                 zy100_edge_lite_output_t *out);
bool zy100_edge_lite_capture_stop(zy100_edge_lite_stop_mode_t mode,
                                  zy100_edge_lite_output_t *out);
bool zy100_edge_lite_capture_end(zy100_edge_lite_output_t *out);
void zy100_edge_lite_get_stats(zy100_edge_lite_stats_t *out);
uint32_t zy100_edge_lite_context_bytes(void);
bool zy100_edge_lite_workspace_ok(void);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_EDGE_LITE_H */
