#ifndef ZY100_SESSION_RANGE_TABLE_H
#define ZY100_SESSION_RANGE_TABLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "zy100_capture_time.h"
#include "zy100_final_edge_record_format.h"
#include "zy100_flash_common.h"
#include "zy100_training_session.h"

#define ZY100_SESSION_RANGE_ENTRY_BYTES 256U
#define ZY100_SESSION_RANGE_MARKER_BYTES 256U

#define ZY100_SESSION_DIR_RECORD_BEGIN_VALID (1UL << 0)
#define ZY100_SESSION_DIR_RECORD_END_VALID   (1UL << 1)

typedef enum
{
    ZY100_SESSION_STATE_EMPTY = 0U,
    ZY100_SESSION_STATE_ALLOCATED = 1U,
    ZY100_SESSION_STATE_BEGIN_COMPLETE = 2U,
    ZY100_SESSION_STATE_COMMITTED = 3U,
    ZY100_SESSION_STATE_EXPORT_CONFIRMED = 4U,
    ZY100_SESSION_STATE_RECLAIMING = 5U,
    ZY100_SESSION_STATE_RECLAIMED = 6U,
    ZY100_SESSION_STATE_DIRTY = 7U,

    ZY100_SESSION_STATE_PROVISIONAL = ZY100_SESSION_STATE_ALLOCATED,
    ZY100_SESSION_STATE_EXPORTED = ZY100_SESSION_STATE_RECLAIMED,
    ZY100_SESSION_STATE_ABORTED = ZY100_SESSION_STATE_RECLAIMED,
} zy100_session_state_t;

typedef enum
{
    ZY100_SESSION_ALLOC_OK = 0U,
    ZY100_SESSION_ALLOC_DIR_FULL = 1U,
    ZY100_SESSION_ALLOC_RAW_FULL = 2U,
    ZY100_SESSION_ALLOC_SUMMARY_FULL = 3U,
    ZY100_SESSION_ALLOC_EVENT_FULL = 4U,
    ZY100_SESSION_ALLOC_FLASH_NOT_READY = 5U,
    ZY100_SESSION_ALLOC_BAD_STATE = 6U,
} zy100_session_alloc_status_t;

typedef enum
{
    ZY100_SESSION_SOURCE_UNKNOWN = 0U,
    ZY100_SESSION_SOURCE_BUTTON_OFFLINE = 1U,
    ZY100_SESSION_SOURCE_BLE = 2U,
    ZY100_SESSION_SOURCE_RECOVERED = 3U,
    ZY100_SESSION_SOURCE_AUTO_MOTION = 4U,
} zy100_session_source_t;

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t bytes;
    uint32_t session_uid;
    uint32_t user_id;
    uint32_t training_id;
    uint32_t session_seq;
    uint64_t time_ms;
    uint32_t time_calibrated;
    uint32_t source;
    uint32_t region_type;
    uint32_t marker_type;
    uint32_t count;
    uint32_t used_bytes;
    uint32_t crc32;
    uint8_t reserved[ZY100_SESSION_RANGE_MARKER_BYTES - 60U];
} zy100_region_session_marker_t;

typedef struct
{
    uint32_t entry_seq;
    uint32_t entry_addr;
    uint32_t session_uid;
    zy100_session_state_t state;
    uint32_t user_id;
    uint32_t training_id;
    uint32_t session_seq;
    uint32_t round;
    uint64_t start_time_ms;
    uint64_t end_time_ms;
    uint32_t time_calibrated;
    uint32_t source;
    uint32_t stop_reason;

    uint32_t raw_marker_begin_addr;
    uint32_t raw_data_begin_addr;
    uint32_t raw_data_end_addr;
    uint32_t raw_reclaim_end_addr;
    uint32_t raw_first_bucket;
    uint32_t raw_limit_bucket;
    uint32_t raw_reserved_bucket_count;
    uint32_t raw_bucket_count;
    uint32_t raw_bucket_bytes;
    uint32_t raw_used_bytes;

    uint32_t summary_marker_begin_addr;
    uint32_t summary_data_begin_addr;
    uint32_t summary_data_end_addr;
    uint32_t summary_reclaim_end_addr;
    uint32_t summary_base_addr;
    uint32_t summary_limit_addr;
    uint32_t summary_reserved_bytes;
    uint32_t summary_count;
    uint32_t summary_used_bytes;

    uint32_t event_marker_begin_addr;
    uint32_t event_data_begin_addr;
    uint32_t event_data_end_addr;
    uint32_t event_reclaim_end_addr;
    uint32_t event_base_addr;
    uint32_t event_limit_addr;
    uint32_t event_reserved_bytes;
    uint32_t event_count;
    uint32_t event_used_bytes;

    uint32_t official_hit_count;
    uint32_t nohit_count;
    uint32_t live_hz;
    uint32_t ois_hz;
    uint32_t pass_flags;
    uint32_t warn_flags;
    uint32_t error_flags;
    uint32_t flags;
    uint32_t record_flags;
    uint8_t rtc_meta[ZY100_CAPTURE_TIME_META_BYTES];
} zy100_session_range_entry_t;

typedef zy100_session_range_entry_t zy100_session_index_entry_t;

typedef struct
{
    uint32_t entry_seq;
    uint32_t entry_addr;
    uint32_t session_uid;
    uint32_t user_id;
    uint32_t training_id;
    uint32_t session_seq;
    uint32_t round;
    uint64_t start_time_ms;
    uint32_t time_calibrated;
    uint32_t source;

    uint32_t raw_marker_begin_addr;
    uint32_t raw_data_begin_addr;
    uint32_t raw_data_end_addr;
    uint32_t raw_reclaim_end_addr;
    uint32_t raw_first_bucket;
    uint32_t raw_limit_bucket;
    uint32_t raw_reserved_bucket_count;
    uint32_t raw_bucket_bytes;

    uint32_t summary_marker_begin_addr;
    uint32_t summary_data_begin_addr;
    uint32_t summary_data_end_addr;
    uint32_t summary_reclaim_end_addr;
    uint32_t summary_base_addr;
    uint32_t summary_limit_addr;
    uint32_t summary_reserved_bytes;

    uint32_t event_marker_begin_addr;
    uint32_t event_data_begin_addr;
    uint32_t event_data_end_addr;
    uint32_t event_reclaim_end_addr;
    uint32_t event_base_addr;
    uint32_t event_limit_addr;
    uint32_t event_reserved_bytes;
} zy100_session_range_alloc_t;

typedef zy100_session_range_alloc_t zy100_session_alloc_t;

typedef struct
{
    bool valid;
    bool storage_dirty_need_clear;
    bool reclaim_active;
    bool clear_active;
    bool expc_without_rcld;
    bool can_start_next;
    uint32_t pending_count;
    uint32_t append_pos;
    uint32_t raw_tail;
    uint32_t summary_tail;
    uint32_t event_tail;
    uint32_t reclaim_session_uid;
    zy100_session_alloc_status_t last_alloc_status;
    uint8_t remaining_percent;
} zy100_session_readiness_t;

typedef struct
{
    uint32_t session_uid;
    uint32_t user_id;
    uint32_t training_id;
    uint32_t session_seq;
    uint32_t round;
    uint64_t start_time_ms;
    uint64_t end_time_ms;
    uint32_t time_calibrated;
    uint32_t source;
    uint32_t stop_reason;

    uint32_t raw_marker_begin_addr;
    uint32_t raw_data_begin_addr;
    uint32_t raw_data_end_addr;
    uint32_t raw_reclaim_end_addr;
    uint32_t raw_first_bucket;
    uint32_t raw_limit_bucket;
    uint32_t raw_bucket_count;
    uint32_t raw_used_bytes;

    uint32_t summary_marker_begin_addr;
    uint32_t summary_data_begin_addr;
    uint32_t summary_data_end_addr;
    uint32_t summary_reclaim_end_addr;
    uint32_t summary_base_addr;
    uint32_t summary_limit_addr;
    uint32_t summary_count;
    uint32_t summary_used_bytes;

    uint32_t event_marker_begin_addr;
    uint32_t event_data_begin_addr;
    uint32_t event_data_end_addr;
    uint32_t event_reclaim_end_addr;
    uint32_t event_base_addr;
    uint32_t event_limit_addr;
    uint32_t event_count;
    uint32_t event_used_bytes;

    uint32_t official_hit_count;
    uint32_t nohit_count;
    uint32_t pass_flags;
    uint32_t warn_flags;
    uint32_t error_flags;
    uint32_t flags;
    uint8_t rtc_meta[ZY100_CAPTURE_TIME_META_BYTES];
} zy100_session_range_commit_t;

typedef zy100_session_range_commit_t zy100_session_commit_t;

bool zy100_session_range_table_scan(void);
bool zy100_session_range_table_scan_boot_recovery(void);
uint32_t zy100_session_range_table_pending_count(void);
uint32_t zy100_session_range_table_total_committed_count(void);
uint32_t zy100_session_range_table_reclaimed_count(void);
bool zy100_session_range_table_storage_dirty_need_clear(void);
bool zy100_session_range_table_refresh_readiness(
    zy100_session_readiness_t *out);

bool zy100_session_range_table_get_tail_pending(
    zy100_session_range_entry_t *out);
bool zy100_session_range_table_get_tail_unreclaimed(
    zy100_session_range_entry_t *out);

bool zy100_session_range_table_allocate_next(
    uint32_t user_id,
    uint32_t training_id,
    uint32_t session_seq,
    uint32_t round,
    uint64_t start_time_ms,
    uint32_t time_calibrated,
    uint32_t source,
    zy100_session_range_alloc_t *out);
zy100_session_alloc_status_t zy100_session_range_table_last_alloc_status(void);

bool zy100_session_range_table_write_begin_complete(
    const zy100_session_range_alloc_t *alloc,
    const uint8_t training_begin[ZY100_TRAINING_RECORD_BYTES]);
bool zy100_session_range_table_commit(
    const zy100_session_range_commit_t *commit,
    const uint8_t training_end[ZY100_TRAINING_RECORD_BYTES]);
bool zy100_session_range_table_abort_active_empty(uint32_t session_uid,
                                                  uint32_t stop_reason,
                                                  uint32_t duration_ms);

bool zy100_session_range_table_confirm_export_tail(
    uint32_t session_uid,
    uint32_t stream_crc32,
    uint32_t bytes_sent);
bool zy100_session_range_table_reclaim_tail_begin(uint32_t session_uid);
zy100_flash_prepare_status_t zy100_session_range_table_reclaim_tail_poll(void);
bool zy100_session_range_table_reclaim_in_progress(void);

bool zy100_session_range_table_reclaim_tail_sector_if_possible(void);
bool zy100_session_range_table_reset_all_after_bulk_clear(void);
bool zy100_session_range_table_prepare_clear_begin(void);
zy100_flash_prepare_status_t zy100_session_range_table_prepare_clear_poll(void);

bool zy100_session_range_table_build_session_meta(
    const zy100_session_range_entry_t *session,
    zy100_fe_session_meta_t *out);

void zy100_session_range_table_get_tails(uint32_t *raw_tail,
                                         uint32_t *summary_tail,
                                         uint32_t *event_tail,
                                         uint32_t *append_pos);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_SESSION_RANGE_TABLE_H */
