#ifndef ZY100_SESSION_DIRECTORY_H
#define ZY100_SESSION_DIRECTORY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "zy100_session_range_table.h"

bool zy100_session_dir_init_scan(void);

uint32_t zy100_session_dir_pending_count(void);
uint32_t zy100_session_dir_total_committed_count(void);
uint32_t zy100_session_dir_exported_count(void);
bool zy100_session_dir_refresh_readiness(zy100_session_readiness_t *out);

bool zy100_session_dir_get_pending_by_index(
    uint32_t pending_index,
    zy100_session_index_entry_t *out);
bool zy100_session_dir_get_tail_pending(zy100_session_index_entry_t *out);

bool zy100_session_dir_allocate_next_ex(
    uint32_t user_id,
    uint32_t training_id,
    uint32_t session_seq,
    uint32_t round,
    uint64_t start_time_ms,
    uint32_t time_calibrated,
    uint32_t source,
    zy100_session_alloc_t *out);
bool zy100_session_dir_allocate_next(
    uint32_t user_id,
    uint32_t training_id,
    uint64_t start_time_ms,
    zy100_session_alloc_t *out);
zy100_session_alloc_status_t zy100_session_dir_last_alloc_status(void);

bool zy100_session_dir_append_begin(
    const zy100_session_alloc_t *alloc,
    const uint8_t training_begin[ZY100_TRAINING_RECORD_BYTES]);

bool zy100_session_dir_append_commit(
    const zy100_session_commit_t *commit,
    const uint8_t training_end[ZY100_TRAINING_RECORD_BYTES]);
bool zy100_session_dir_abort_active_empty(uint32_t session_uid,
                                          uint32_t stop_reason,
                                          uint32_t duration_ms);

bool zy100_session_dir_mark_exported(uint32_t session_uid);

bool zy100_session_dir_all_pending_exported(void);

bool zy100_session_dir_reset_all_after_bulk_clear(void);
bool zy100_session_dir_prepare_clear_begin(void);
zy100_flash_prepare_status_t zy100_session_dir_prepare_clear_poll(void);

bool zy100_session_dir_build_session_meta(
    const zy100_session_index_entry_t *session,
    zy100_fe_session_meta_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_SESSION_DIRECTORY_H */
