#ifndef ZY100_EDGE_SHADOW_H
#define ZY100_EDGE_SHADOW_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t sample_index;
    uint16_t timestamp_raw;
    uint8_t fifo_header;
    uint8_t flags;

    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;

    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    int16_t temp_raw;
} zy100_edge_shadow_sample_t;

typedef struct zy100_edge_lite_output zy100_edge_lite_output_t;

void zy100_edge_shadow_init(void);
void zy100_edge_shadow_capture_start(void);
void zy100_edge_shadow_push_sample(const zy100_edge_shadow_sample_t *sample);
bool zy100_edge_shadow_push_sample_ex(const zy100_edge_shadow_sample_t *sample,
                                      zy100_edge_lite_output_t *out);
void zy100_edge_shadow_note_decode_error(void);
void zy100_edge_shadow_capture_end(void);
bool zy100_edge_shadow_capture_end_ex(zy100_edge_lite_output_t *out);
bool zy100_edge_shadow_capture_end_discard_partial(zy100_edge_lite_output_t *out);
void zy100_edge_shadow_print_summary(void);
void zy100_edge_shadow_check_sample_mismatch(uint32_t capture_packet_count);
uint32_t zy100_edge_shadow_workspace_reserved_bytes(void);
uint32_t zy100_edge_shadow_workspace_used_est_bytes(void);
uint32_t zy100_edge_shadow_accepted_count(void);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_EDGE_SHADOW_H */
