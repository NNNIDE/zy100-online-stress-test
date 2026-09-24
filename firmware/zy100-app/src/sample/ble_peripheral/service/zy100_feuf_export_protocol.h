#ifndef ZY100_FEUF_EXPORT_PROTOCOL_H
#define ZY100_FEUF_EXPORT_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "../app_flags.h"

#define ZY100_FEUF_MAGIC   0x46554546UL
#define ZY100_FEUF_VERSION 1U

#ifndef ZY100_FEUF_MAX_PAYLOAD
#define ZY100_FEUF_MAX_PAYLOAD 256U
#endif

typedef enum
{
    ZY100_FEUF_FRAME_HELLO = 1U,
    ZY100_FEUF_FRAME_MANIFEST = 2U,
    ZY100_FEUF_FRAME_DATA = 3U,
    ZY100_FEUF_FRAME_SECTION_END = 4U,
    ZY100_FEUF_FRAME_EXPORT_END = 5U,
    ZY100_FEUF_FRAME_ACK = 6U,
    ZY100_FEUF_FRAME_NAK = 7U,
    ZY100_FEUF_FRAME_ABORT = 8U,
    ZY100_FEUF_FRAME_ERROR = 9U,
    ZY100_FEUF_FRAME_EXPORT_START = 10U,
    ZY100_FEUF_FRAME_EXPORT_DONE_ACK = 11U,
} zy100_feuf_frame_type_t;

typedef enum
{
    ZY100_FEUF_SECTION_MANIFEST = 0U,
    ZY100_FEUF_SECTION_SESSION_META = 1U,
    ZY100_FEUF_SECTION_RAW_RECORDS = 2U,
    ZY100_FEUF_SECTION_SUMMARY_RECORDS = 3U,
    ZY100_FEUF_SECTION_EVENT_RECORDS = 4U,
    ZY100_FEUF_SECTION_TRAINING_BEGIN = 0x10U,
    ZY100_FEUF_SECTION_TRAINING_END = 0x11U,
    ZY100_FEUF_SECTION_END = 5U,
} zy100_feuf_section_id_t;

#define ZY100_FEUF_SECTION_BIT_META           (1UL << 0)
#define ZY100_FEUF_SECTION_BIT_RAW            (1UL << 1)
#define ZY100_FEUF_SECTION_BIT_SUMMARY        (1UL << 2)
#define ZY100_FEUF_SECTION_BIT_EVENT          (1UL << 3)
#define ZY100_FEUF_SECTION_BIT_TRAINING_BEGIN (1UL << 4)
#define ZY100_FEUF_SECTION_BIT_TRAINING_END   (1UL << 5)
#define ZY100_FEUF_SECTION_BIT_EXISTING_ALL \
    (ZY100_FEUF_SECTION_BIT_META | ZY100_FEUF_SECTION_BIT_RAW | \
     ZY100_FEUF_SECTION_BIT_SUMMARY | ZY100_FEUF_SECTION_BIT_EVENT)
#define ZY100_FEUF_SECTION_BIT_ALL \
    (ZY100_FEUF_SECTION_BIT_EXISTING_ALL | \
     ZY100_FEUF_SECTION_BIT_TRAINING_BEGIN | \
     ZY100_FEUF_SECTION_BIT_TRAINING_END)

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint16_t version;
    uint16_t header_bytes;
    uint16_t frame_type;
    uint16_t flags;

    uint32_t seq;
    uint32_t section_id;
    uint32_t section_offset;

    uint16_t payload_len;
    uint16_t reserved0;

    uint32_t payload_crc32;
    uint32_t header_crc32;
} zy100_feuf_frame_header_t;

typedef struct __attribute__((packed))
{
    uint32_t session_id;
    uint32_t frames_sent;
    uint32_t payload_bytes_sent;
    uint32_t section_crc32_xor;
    uint32_t status;
} zy100_feuf_export_end_payload_t;

typedef struct __attribute__((packed))
{
    uint32_t protocol_version;
    uint32_t session_version;
    uint32_t round;
    uint32_t stop_reason;

    uint32_t live_hz;
    uint32_t ois_hz;

    uint32_t raw_count;
    uint32_t summary_count;
    uint32_t event_count;

    uint32_t meta_bytes;
    uint32_t raw_bytes;
    uint32_t summary_bytes;
    uint32_t event_bytes;

    uint32_t meta_base;
    uint32_t raw_base;
    uint32_t summary_base;
    uint32_t event_base;

    uint32_t payload_max;
    uint32_t total_sections;
    uint32_t total_frames_est;
    uint32_t total_payload_bytes;

    uint32_t manifest_crc32;

    uint32_t training_section_count;
    uint32_t training_begin_bytes;
    uint32_t training_end_bytes;
    uint32_t training_flags;
    uint32_t section_mask_all;
} zy100_feuf_export_manifest_t;

typedef struct __attribute__((packed))
{
    uint32_t protocol_version;
    uint32_t manifest_bytes;
    uint16_t header_bytes;
    uint16_t session_version;

    uint32_t session_uid;
    uint32_t export_index;
    uint32_t export_total;
    uint32_t user_id;
    uint32_t training_id;
    uint32_t session_seq;
    uint32_t round;
    uint64_t start_time_ms;
    uint64_t end_time_ms;
    uint32_t stop_reason;

    uint32_t live_hz;
    uint32_t ois_hz;
    uint32_t raw_data_begin_addr;
    uint32_t raw_data_end_addr;
    uint32_t raw_count;
    uint32_t raw_used_bytes;
    uint32_t raw_reclaim_end_addr;
    uint32_t summary_data_begin_addr;
    uint32_t summary_data_end_addr;
    uint32_t summary_count;
    uint32_t summary_used_bytes;
    uint32_t summary_reclaim_end_addr;
    uint32_t event_data_begin_addr;
    uint32_t event_data_end_addr;
    uint32_t event_count;
    uint32_t event_used_bytes;
    uint32_t event_reclaim_end_addr;

    uint32_t meta_bytes;
    uint32_t raw_bytes;
    uint32_t summary_bytes;
    uint32_t event_bytes;
    uint32_t payload_max;
    uint32_t total_sections;
    uint32_t total_frames_est;
    uint32_t total_payload_bytes;

    uint32_t training_section_count;
    uint32_t training_begin_bytes;
    uint32_t training_end_bytes;
    uint32_t training_flags;
    uint32_t section_mask_all;
    uint32_t flags;
    uint32_t stream_crc32;
    uint32_t manifest_crc32;
    uint32_t reserved[8];
} zy100_feuf_export_manifest_v3_t;

typedef char zy100_feuf_header_size_check[
    (sizeof(zy100_feuf_frame_header_t) == 36U) ? 1 : -1];
typedef char zy100_feuf_manifest_size_check[
    (sizeof(zy100_feuf_export_manifest_t) == 108U) ? 1 : -1];
typedef char zy100_feuf_manifest_v3_size_check[
    (sizeof(zy100_feuf_export_manifest_v3_t) > 108U) ? 1 : -1];

#ifdef __cplusplus
}
#endif

#endif /* ZY100_FEUF_EXPORT_PROTOCOL_H */
