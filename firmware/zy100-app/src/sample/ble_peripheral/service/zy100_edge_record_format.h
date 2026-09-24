#ifndef ZY100_EDGE_RECORD_FORMAT_H
#define ZY100_EDGE_RECORD_FORMAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "../app_flags.h"

#define ZY100_EDGE_MAGIC_ZYE1          0x3145595AUL
#define ZY100_EDGE_MAGIC_ZER1          0x3152455AUL
#define ZY100_EDGE_FORMAT_VERSION      2U
#define ZY100_EDGE_PROFILE_HIGH_RECALL 1U

#define ZY100_EDGE_META_SLOT_PROVISIONAL_OFFSET 0U
#define ZY100_EDGE_META_SLOT_FINAL_OFFSET       512U
#define ZY100_EDGE_META_SLOT_BYTES              512U
#define ZY100_EDGE_META_REGION_BYTES            (ZY100_EDGE_META_BLOCKS * ZY100_EDGE_FLASH_BLOCK_BYTES)
#define ZY100_EDGE_RECORD_REGION_BLOCKS         456U
#define ZY100_EDGE_RAW_REGION_BLOCKS            512U
#define ZY100_EDGE_RECORD_REGION_BYTES          (ZY100_EDGE_RECORD_REGION_BLOCKS * ZY100_EDGE_FLASH_BLOCK_BYTES)
#define ZY100_EDGE_RAW_REGION_BYTES             (ZY100_EDGE_RAW_REGION_BLOCKS * ZY100_EDGE_FLASH_BLOCK_BYTES)
#define ZY100_EDGE_TOTAL_REGION_BYTES           \
    (ZY100_EDGE_META_REGION_BYTES + ZY100_EDGE_RECORD_REGION_BYTES + ZY100_EDGE_RAW_REGION_BYTES)
#define ZY100_EDGE_TOTAL_REGION_BLOCKS          \
    (ZY100_EDGE_META_BLOCKS + ZY100_EDGE_RECORD_REGION_BLOCKS + ZY100_EDGE_RAW_REGION_BLOCKS)

#define ZY100_EDGE_SESSION_STATUS_PROVISIONAL 1U
#define ZY100_EDGE_SESSION_STATUS_COMPLETE    2U
#define ZY100_EDGE_SESSION_STATUS_FAILED      3U

#define ZY100_EDGE_RECORD_TYPE_SUMMARY        1U
#define ZY100_EDGE_RECORD_TYPE_EVENT_FEATURE  2U
#define ZY100_EDGE_RECORD_TYPE_RAW_INDEX      3U
#define ZY100_EDGE_RECORD_TYPE_END            255U

#define ZY100_EDGE_RAW_SAVED_LEVEL_NOT_SAVED     0U
#define ZY100_EDGE_RAW_SAVED_LEVEL_PARTIAL_POST  1U
#define ZY100_EDGE_RAW_SAVED_LEVEL_FULL          2U
#define ZY100_EDGE_RAW_SAVED_LEVEL_PARTIAL_PREPOST 3U

#define ZY100_EDGE_QUALITY_RAW_PRE_TRUNCATED             (1UL << 16)
#define ZY100_EDGE_QUALITY_RAW_NOT_SAVED_BUDGET          (1UL << 17)
#define ZY100_EDGE_QUALITY_RAW_NOT_SAVED_POLICY          (1UL << 18)
#define ZY100_EDGE_QUALITY_RAW_NOT_SAVED_OVERFLOW        (1UL << 19)
#define ZY100_EDGE_QUALITY_RAW_SUPPRESSED_BACKPRESSURE   (1UL << 20)
#define ZY100_EDGE_QUALITY_RAW_NOT_SAVED_TOO_SHORT       (1UL << 21)
#define ZY100_EDGE_QUALITY_RAW_PARTIAL_POST              (1UL << 22)
#define ZY100_EDGE_QUALITY_RAW_STORE_MASK                \
    (ZY100_EDGE_QUALITY_RAW_PRE_TRUNCATED |              \
     ZY100_EDGE_QUALITY_RAW_NOT_SAVED_BUDGET |           \
     ZY100_EDGE_QUALITY_RAW_NOT_SAVED_POLICY |           \
     ZY100_EDGE_QUALITY_RAW_NOT_SAVED_OVERFLOW |         \
     ZY100_EDGE_QUALITY_RAW_SUPPRESSED_BACKPRESSURE |    \
     ZY100_EDGE_QUALITY_RAW_NOT_SAVED_TOO_SHORT |        \
     ZY100_EDGE_QUALITY_RAW_PARTIAL_POST)

#define ZY100_EDGE_QUALITY_PRE_TRUNCATED \
    ZY100_EDGE_QUALITY_RAW_PRE_TRUNCATED

#define ZY100_EDGE_SHAPE_BINS 32U

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint16_t version;
    uint16_t header_bytes;
    uint32_t status;
    uint32_t round;
    uint32_t sample_rate_hz;
    uint32_t packet_bytes;
    uint32_t profile;
    uint32_t total_region_bytes;
    uint32_t meta_region_bytes;
    uint32_t record_region_bytes;
    uint32_t raw_region_bytes;
    uint32_t raw_budget_bytes;
    uint32_t record_used_bytes;
    uint32_t record_free_bytes;
    uint32_t raw_used_bytes;
    uint32_t raw_free_bytes;
    uint32_t sample_count;
    uint32_t summary_count;
    uint32_t event_count;
    uint32_t raw_req_count;
    uint32_t raw_window_count;
    uint32_t raw_rejected_budget;
    uint32_t raw_rejected_policy;
    uint32_t write_count;
    uint32_t write_error_count;
    uint32_t verify_error_count;
    uint32_t start_ms;
    uint32_t end_ms;
    uint32_t flags;
    uint32_t crc32;
    uint32_t reserved[20];
} zy100_edge_session_header_t;

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint16_t type;
    uint16_t header_bytes;
    uint16_t payload_bytes;
    uint16_t reserved0;
    uint32_t seq;
    uint32_t crc32;
} zy100_edge_record_header_t;

typedef struct __attribute__((packed))
{
    zy100_edge_record_header_t header;
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
} zy100_edge_summary_record_t;

typedef struct __attribute__((packed))
{
    zy100_edge_record_header_t header;
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
    uint32_t raw_saved;
    uint32_t raw_saved_level;
    uint32_t raw_window_id;
    uint16_t shape_acc[ZY100_EDGE_SHAPE_BINS];
    uint16_t shape_gyro[ZY100_EDGE_SHAPE_BINS];
} zy100_edge_event_record_t;

typedef struct __attribute__((packed))
{
    zy100_edge_record_header_t header;
    uint32_t raw_window_id;
    uint32_t event_id;
    uint32_t start_sample;
    uint32_t end_sample;
    uint32_t packet_count;
    uint32_t raw_flash_offset;
    uint32_t raw_bytes;
    uint32_t encoded_bytes;
    uint32_t codec;
    uint32_t quality_flags;
    uint32_t requested_start_sample;
    uint32_t actual_start_sample;
    uint32_t trigger_sample;
    uint32_t pre_bytes_saved;
    uint32_t post_bytes_saved;
    uint32_t raw_saved_level;
    uint32_t raw_min_valid_bytes;
} zy100_edge_raw_index_record_t;

typedef struct __attribute__((packed))
{
    zy100_edge_record_header_t header;
    uint32_t sample_count;
    uint32_t summary_count;
    uint32_t event_count;
    uint32_t raw_req_count;
    uint32_t raw_window_count;
    uint32_t record_used_bytes;
    uint32_t raw_used_bytes;
    uint32_t flags;
} zy100_edge_end_record_t;

typedef char zy100_edge_header_size_check[
    (sizeof(zy100_edge_session_header_t) <= ZY100_EDGE_META_SLOT_BYTES) ? 1 : -1];
typedef char zy100_edge_final_meta_slot_check[
    ((ZY100_EDGE_META_SLOT_FINAL_OFFSET + ZY100_EDGE_META_SLOT_BYTES) <=
     ZY100_EDGE_META_REGION_BYTES) ? 1 : -1];
typedef char zy100_edge_summary_size_check[
    (sizeof(zy100_edge_summary_record_t) <= 96U) ? 1 : -1];
typedef char zy100_edge_event_size_check[
    (sizeof(zy100_edge_event_record_t) <= 384U) ? 1 : -1];
typedef char zy100_edge_raw_index_size_check[
    (sizeof(zy100_edge_raw_index_record_t) <= 96U) ? 1 : -1];
typedef char zy100_edge_total_region_size_check[
    (ZY100_EDGE_TOTAL_REGION_BYTES == 3969024UL) ? 1 : -1];
typedef char zy100_edge_total_region_blocks_check[
    (ZY100_EDGE_TOTAL_REGION_BLOCKS == 969U) ? 1 : -1];

#ifdef __cplusplus
}
#endif

#endif /* ZY100_EDGE_RECORD_FORMAT_H */
