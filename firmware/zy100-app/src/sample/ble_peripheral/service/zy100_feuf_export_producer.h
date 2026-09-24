#ifndef ZY100_FEUF_EXPORT_PRODUCER_H
#define ZY100_FEUF_EXPORT_PRODUCER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "zy100_feuf_export_protocol.h"
#include "zy100_final_edge_record_format.h"
#include "zy100_session_directory.h"
#include "zy100_training_session.h"

typedef enum
{
    ZY100_FEUF_PRODUCER_IDLE = 0U,
    ZY100_FEUF_PRODUCER_STREAMING,
    ZY100_FEUF_PRODUCER_DONE,
    ZY100_FEUF_PRODUCER_ERROR,
} zy100_feuf_producer_state_t;

typedef enum
{
    ZY100_FEUF_PRODUCER_ERROR_NONE = 0U,
    ZY100_FEUF_PRODUCER_ERROR_FLASH_READ = 1U,
    ZY100_FEUF_PRODUCER_ERROR_BAD_RECORD = 2U,
    ZY100_FEUF_PRODUCER_ERROR_FLASH_BUSY = 3U,
    ZY100_FEUF_PRODUCER_ERROR_BAD_STATE = 4U,
} zy100_feuf_producer_error_t;

typedef struct
{
    uint32_t frame_type;
    uint32_t section_id;
    uint32_t section_offset;
    uint32_t stream_offset;
    uint16_t frame_remaining;
} zy100_feuf_producer_progress_t;

typedef struct
{
    uint32_t flash_read_calls;
    uint32_t flash_read_fail_count;
    uint32_t flash_read_bytes;
    uint64_t flash_read_total_us;
    uint32_t flash_read_min_us;
    uint32_t flash_read_max_us;
} zy100_feuf_export_flash_perf_t;

typedef struct
{
    uint32_t hit_count;
    uint32_t miss_count;
    uint32_t bypass_count;
    uint32_t read_calls;
    uint32_t read_bytes;
    uint32_t copied_bytes;
} zy100_feuf_export_prefetch_perf_t;

typedef struct
{
    bool active;
    bool done;
    bool manifest_ready;
    bool frame_ready;
    bool export_end_sent;
    zy100_feuf_producer_state_t state;
    zy100_feuf_producer_error_t error;
    zy100_feuf_export_manifest_t manifest;
    zy100_feuf_export_manifest_v3_t manifest_v3;
    zy100_session_index_entry_t session;
    bool multi_session;
    uint32_t stop_reason;
    uint32_t export_index;
    uint32_t export_total;
    uint32_t seq_next;
    uint32_t frames_sent;
    uint32_t payload_bytes_sent;
    uint32_t section_crc32_xor;
    uint32_t section_mask;
    uint32_t available_section_mask;
    uint32_t section_done_mask;
    uint32_t current_section;
    uint32_t current_section_offset;
    uint32_t current_record_index;
    uint32_t current_record_addr;
    uint32_t current_record_offset;
    uint32_t current_record_remaining;
    uint32_t summary_next_read_addr;
    uint32_t event_next_read_addr;
    uint32_t stream_offset;
    uint32_t pending_frame_type;
    uint32_t pending_section_bit;
    uint32_t pending_payload_crc32;
    uint32_t pending_payload_len;
    uint32_t pending_section_advance;
    uint32_t pending_record_advance;
    uint32_t pending_frame_stream_bytes;
    uint8_t section_step;
    bool current_record_memory;
    uint8_t current_record_mem[sizeof(zy100_fe_session_meta_t)];
    uint8_t frame_buf[sizeof(zy100_feuf_frame_header_t) + ZY100_FEUF_MAX_PAYLOAD];
    uint16_t payload_max;
    uint16_t frame_len;
    uint16_t frame_pos;
#if ZY100_BLE_EXPORT_PREFETCH_ENABLE
    bool prefetch_cache_valid;
    uint32_t prefetch_cache_base_addr;
    uint32_t prefetch_cache_len;
    uint32_t prefetch_record_base_addr;
    uint32_t prefetch_record_end_addr;
    uint8_t *prefetch_cache;
    uint32_t prefetch_cache_capacity;
#endif
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    zy100_feuf_export_flash_perf_t flash_perf;
    zy100_feuf_export_flash_perf_t flash_stream_perf;
    zy100_feuf_export_prefetch_perf_t prefetch_perf;
    bool flash_stream_perf_active;
#endif
} zy100_feuf_export_producer_t;

typedef struct
{
    bool valid;
    zy100_feuf_export_producer_t after;
    zy100_feuf_producer_progress_t first_progress;
    zy100_feuf_producer_progress_t last_progress;
    uint32_t data_payload_delta;
    uint32_t stream_offset_start;
    uint32_t stream_offset_end;
    uint32_t prefetch_read_calls_delta;
    uint32_t prefetch_read_bytes_delta;
    uint16_t len;
    uint8_t pieces;
    uint8_t frames_completed;
} zy100_feuf_export_aggregate_plan_t;

bool zy100_feuf_export_producer_begin(zy100_feuf_export_producer_t *producer,
                                      uint32_t stop_reason);
bool zy100_feuf_export_producer_begin_session(
    zy100_feuf_export_producer_t *producer,
    const zy100_session_index_entry_t *session,
    uint32_t export_index,
    uint32_t export_total);
bool zy100_feuf_export_producer_peek(zy100_feuf_export_producer_t *producer,
                                     uint8_t *out,
                                     uint16_t out_cap,
                                     uint16_t *out_len,
                                     zy100_feuf_producer_progress_t *progress);
bool zy100_feuf_export_producer_peek_aggregate(
    zy100_feuf_export_producer_t *producer,
    uint8_t *out,
    uint16_t max_len,
    uint16_t target_len,
    uint8_t max_pieces,
    uint16_t *out_len,
    uint8_t *out_piece_count,
    zy100_feuf_producer_progress_t *progress);
bool zy100_feuf_export_producer_build_aggregate_plan(
    const zy100_feuf_export_producer_t *real,
    zy100_feuf_export_producer_t *scratch,
    uint8_t *out,
    uint16_t out_cap,
    uint16_t target_len,
    uint8_t max_pieces,
    zy100_feuf_export_aggregate_plan_t *plan);
bool zy100_feuf_export_producer_commit(zy100_feuf_export_producer_t *producer,
                                       uint16_t len);
bool zy100_feuf_export_producer_done(const zy100_feuf_export_producer_t *producer);
uint32_t zy100_feuf_export_producer_error(const zy100_feuf_export_producer_t *producer);
uint32_t zy100_feuf_export_producer_export_id(const zy100_feuf_export_producer_t *producer);
uint32_t zy100_feuf_export_producer_estimated_payload_bytes(
    const zy100_feuf_export_producer_t *producer);
uint32_t zy100_feuf_export_producer_estimated_bytes(const zy100_feuf_export_producer_t *producer);
uint16_t zy100_feuf_export_producer_payload_max(
    const zy100_feuf_export_producer_t *producer);
uint32_t zy100_feuf_export_producer_payload_bytes_sent(
    const zy100_feuf_export_producer_t *producer);
void zy100_feuf_export_producer_set_prefetch_buffer(
    zy100_feuf_export_producer_t *producer,
    uint8_t *buffer,
    uint32_t buffer_bytes);
void zy100_feuf_export_producer_adopt_prefetch_cache(
    zy100_feuf_export_producer_t *producer,
    const zy100_feuf_export_producer_t *source);
void zy100_feuf_export_producer_p0_mark_stream_start(
    zy100_feuf_export_producer_t *producer);
void zy100_feuf_export_producer_get_flash_perf(
    const zy100_feuf_export_producer_t *producer,
    zy100_feuf_export_flash_perf_t *out);
void zy100_feuf_export_producer_get_flash_stream_perf(
    const zy100_feuf_export_producer_t *producer,
    zy100_feuf_export_flash_perf_t *out);
void zy100_feuf_export_producer_get_prefetch_perf(
    const zy100_feuf_export_producer_t *producer,
    zy100_feuf_export_prefetch_perf_t *out);
#ifdef __cplusplus
}
#endif

#endif /* ZY100_FEUF_EXPORT_PRODUCER_H */
