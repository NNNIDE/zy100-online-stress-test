#include "zy100_session_directory.h"

#include "zy100_rtc_clock.h"

bool zy100_session_dir_init_scan(void)
{
    return zy100_session_range_table_scan_boot_recovery();
}

uint32_t zy100_session_dir_pending_count(void)
{
    return zy100_session_range_table_pending_count();
}

uint32_t zy100_session_dir_total_committed_count(void)
{
    return zy100_session_range_table_total_committed_count();
}

uint32_t zy100_session_dir_exported_count(void)
{
    return zy100_session_range_table_reclaimed_count();
}

bool zy100_session_dir_refresh_readiness(zy100_session_readiness_t *out)
{
    return zy100_session_range_table_refresh_readiness(out);
}

bool zy100_session_dir_get_pending_by_index(uint32_t pending_index,
                                            zy100_session_index_entry_t *out)
{
    if (pending_index != 0U)
    {
        return false;
    }
    return zy100_session_range_table_get_tail_pending(out);
}

bool zy100_session_dir_get_tail_pending(zy100_session_index_entry_t *out)
{
    return zy100_session_range_table_get_tail_pending(out);
}

bool zy100_session_dir_allocate_next_ex(uint32_t user_id,
                                        uint32_t training_id,
                                        uint32_t session_seq,
                                        uint32_t round,
                                        uint64_t start_time_ms,
                                        uint32_t time_calibrated,
                                        uint32_t source,
                                        zy100_session_alloc_t *out)
{
    return zy100_session_range_table_allocate_next(user_id,
                                                  training_id,
                                                  session_seq,
                                                  round,
                                                  start_time_ms,
                                                  time_calibrated,
                                                  source,
                                                  out);
}

bool zy100_session_dir_allocate_next(uint32_t user_id,
                                     uint32_t training_id,
                                     uint64_t start_time_ms,
                                     zy100_session_alloc_t *out)
{
    return zy100_session_range_table_allocate_next(
        user_id,
        training_id,
        0U,
        0U,
        start_time_ms,
        zy100_rtc_clock_is_calibrated() ? 1U : 0U,
        ZY100_SESSION_SOURCE_BLE,
        out);
}

zy100_session_alloc_status_t zy100_session_dir_last_alloc_status(void)
{
    return zy100_session_range_table_last_alloc_status();
}

bool zy100_session_dir_append_begin(
    const zy100_session_alloc_t *alloc,
    const uint8_t training_begin[ZY100_TRAINING_RECORD_BYTES])
{
    return zy100_session_range_table_write_begin_complete(alloc,
                                                         training_begin);
}

bool zy100_session_dir_append_commit(
    const zy100_session_commit_t *commit,
    const uint8_t training_end[ZY100_TRAINING_RECORD_BYTES])
{
    return zy100_session_range_table_commit(commit, training_end);
}

bool zy100_session_dir_abort_active_empty(uint32_t session_uid,
                                          uint32_t stop_reason,
                                          uint32_t duration_ms)
{
    return zy100_session_range_table_abort_active_empty(session_uid,
                                                       stop_reason,
                                                       duration_ms);
}

bool zy100_session_dir_mark_exported(uint32_t session_uid)
{
    return zy100_session_range_table_confirm_export_tail(session_uid, 0U, 0U) &&
           zy100_session_range_table_reclaim_tail_begin(session_uid);
}

bool zy100_session_dir_all_pending_exported(void)
{
    return zy100_session_range_table_pending_count() == 0U;
}

bool zy100_session_dir_reset_all_after_bulk_clear(void)
{
    return zy100_session_range_table_reset_all_after_bulk_clear();
}

bool zy100_session_dir_prepare_clear_begin(void)
{
    return zy100_session_range_table_prepare_clear_begin();
}

zy100_flash_prepare_status_t zy100_session_dir_prepare_clear_poll(void)
{
    return zy100_session_range_table_prepare_clear_poll();
}

bool zy100_session_dir_build_session_meta(
    const zy100_session_index_entry_t *session,
    zy100_fe_session_meta_t *out)
{
    return zy100_session_range_table_build_session_meta(session, out);
}
