#ifndef ZY100_FINAL_EDGE_RECORD_FORMAT_H
#define ZY100_FINAL_EDGE_RECORD_FORMAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ZY100_FE_REC_MAGIC_SUMMARY 0x4D535546UL
#define ZY100_FE_REC_MAGIC_EVENT   0x56534546UL
#define ZY100_FE_REC_VERSION       1U

#define ZY100_FE_SESSION_MAGIC     0x31455346UL
#define ZY100_FE_SESSION_VERSION   1U

typedef enum
{
    ZY100_FE_REC_TYPE_SUMMARY = 1U,
    ZY100_FE_REC_TYPE_EVENT = 2U,
} zy100_fe_record_type_t;

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint16_t version;
    uint16_t header_bytes;

    uint16_t record_type;
    uint16_t flags;

    uint32_t seq;
    uint32_t edge_sample_index;
    uint32_t source_flags;

    uint32_t payload_bytes;
    uint32_t record_bytes;
    uint32_t page_count;

    uint32_t payload_xor;
    uint32_t header_xor;

    uint32_t reserved[8];
} zy100_fe_record_flash_header_t;

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint16_t version;
    uint16_t header_bytes;

    uint32_t round;
    uint32_t start_ms;
    uint32_t end_ms;

    uint32_t live_hz;
    uint32_t ois_hz;

    uint32_t official_hit_count;
    uint32_t nohit_count;

    uint32_t raw_count;
    uint32_t summary_count;
    uint32_t event_count;

    uint32_t raw_region_base;
    uint32_t raw_region_bytes;
    uint32_t summary_region_base;
    uint32_t summary_region_bytes;
    uint32_t event_region_base;
    uint32_t event_region_bytes;

    uint32_t raw_used_bytes;
    uint32_t summary_used_bytes;
    uint32_t event_used_bytes;

    uint32_t pass_flags;
    uint32_t warn_flags;
    uint32_t error_flags;

    uint32_t payload_xor;
    uint32_t header_xor;

    uint32_t reserved[32];
} zy100_fe_session_meta_t;

#ifdef __cplusplus
}
#endif

#endif /* ZY100_FINAL_EDGE_RECORD_FORMAT_H */
