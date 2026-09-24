#ifndef ZY100_ONLINE_SPOOL_H
#define ZY100_ONLINE_SPOOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../app_flags.h"
#include <stdbool.h>
#include <stdint.h>

#include "zy100_final_edge_store_target.h"
#include "zy100_flash_common.h"

#define ZY100_ONLINE_SPOOL_RECORD_HEADER_BYTES 256U
#define ZY100_ONLINE_SPOOL_HEADER_FLAG_SEGMENT_META 0x01U
#define ZY100_ONLINE_SPOOL_HEADER_FLAG_RING_GENERATION 0x02U
#define ZY100_ONLINE_SPOOL_HEADER_EXTENSION_VERSION 1U

typedef enum
{
    ZY100_ONLINE_SPOOL_PUMP_IDLE = 0U,
    ZY100_ONLINE_SPOOL_PUMP_PROGRESS,
    ZY100_ONLINE_SPOOL_PUMP_COMMITTED,
    ZY100_ONLINE_SPOOL_PUMP_ERROR,
} zy100_online_spool_pump_result_t;

typedef enum
{
    ZY100_ONLINE_SPOOL_IO_OK = 0U,
    ZY100_ONLINE_SPOOL_IO_EMPTY,
    ZY100_ONLINE_SPOOL_IO_BUSY,
    ZY100_ONLINE_SPOOL_IO_ERROR,
} zy100_online_spool_io_result_t;

typedef enum
{
    ZY100_ONLINE_SPOOL_ERASE_IDLE = 0U,
    ZY100_ONLINE_SPOOL_ERASE_PROGRESS,
    ZY100_ONLINE_SPOOL_ERASE_BLOCKED,
    ZY100_ONLINE_SPOOL_ERASE_ERROR,
} zy100_online_spool_erase_result_t;

typedef enum
{
    ZY100_ONLINE_SPOOL_START_PREP_IDLE = 0U,
    ZY100_ONLINE_SPOOL_START_PREP_BUSY,
    ZY100_ONLINE_SPOOL_START_PREP_DONE,
    ZY100_ONLINE_SPOOL_START_PREP_ERROR,
} zy100_online_spool_start_prep_status_t;

typedef enum
{
    ZY100_ONLINE_SPOOL_RESERVE_BLOCK_NONE = 0U,
    ZY100_ONLINE_SPOOL_RESERVE_BLOCK_UNACKED,
    ZY100_ONLINE_SPOOL_RESERVE_BLOCK_ERASE_PENDING,
} zy100_online_spool_reserve_block_t;

typedef struct
{
    uint8_t type;
    uint8_t flags;
    uint16_t generation;
    uint32_t record_id;
    uint32_t header_addr;
    uint32_t data_addr;
    uint32_t record_bytes;
    uint32_t source_record_bytes;
    uint32_t payload_bytes;
    uint32_t crc32;
    uint32_t capture_segment_id;
    uint32_t segment_start_offset_ms;
    uint32_t source_id;
} zy100_online_spool_record_t;

typedef struct
{
    uint32_t produced[ZY100_ONLINE_RECORD_TYPE_COUNT];
    uint32_t committed[ZY100_ONLINE_RECORD_TYPE_COUNT];
    uint32_t acked[ZY100_ONLINE_RECORD_TYPE_COUNT];
    uint32_t dropped[ZY100_ONLINE_RECORD_TYPE_COUNT];
    uint32_t reserve_attempt[ZY100_ONLINE_RECORD_TYPE_COUNT];
    uint32_t reserve_ok[ZY100_ONLINE_RECORD_TYPE_COUNT];
    uint32_t reserve_busy[ZY100_ONLINE_RECORD_TYPE_COUNT];
    uint32_t reserve_full[ZY100_ONLINE_RECORD_TYPE_COUNT];
    uint32_t reserve_error[ZY100_ONLINE_RECORD_TYPE_COUNT];
    uint32_t reserve_busy_owner[ZY100_ONLINE_RECORD_TYPE_COUNT];
    uint32_t capacity_blocked;
    uint32_t erase_blocked;
    uint32_t generation_mismatch;
    uint32_t logical_used_bytes;
    uint32_t reserve_generation;
    uint32_t ack_generation;
    uint32_t reserve_ptr;
    uint32_t commit_ptr;
    uint32_t tx_ptr;
    uint32_t pending_bytes;
    uint32_t pending_high_water;
    uint32_t pending_records;
    uint32_t erased_ahead_bytes;
    uint32_t erased_ahead_min_bytes;
    uint32_t erased_sector_count;
    uint32_t reclaimable_sector_count;
    uint32_t erase_count;
    uint32_t wrap_count;
    uint32_t crc_pages;
    uint32_t crc_total_us;
    uint32_t crc_max_us;
    uint32_t header_programs;
    uint32_t erase_total_ms;
    uint32_t erase_max_ms;
    uint32_t read_errors;
    uint32_t write_errors;
    uint32_t erase_errors;
    uint32_t tx_read_wip_blocked;
    uint32_t tx_read_spi_busy;
    uint32_t tx_read_busy_retry;
    uint32_t erase_tx_conflict_prevented;
    uint32_t erase_issue_count;
    uint32_t erase_complete_count;
    uint32_t block32_erase_issue_count;
    uint32_t block32_erase_complete_count;
    uint32_t erase_timeout_count;
    uint32_t state_errors;
    uint32_t ack_errors;
    uint32_t last_error;
    uint32_t last_error_detail;
} zy100_online_spool_stats_t;

void zy100_online_spool_init(void);
void zy100_online_spool_ready_begin(void);
bool zy100_online_spool_prepare_start_blocking(uint32_t minimum_bytes,
                                                uint32_t *detail_out);
bool zy100_online_spool_start_prepare_begin(uint32_t minimum_bytes,
                                             uint32_t *detail_out);
zy100_online_spool_start_prep_status_t
zy100_online_spool_start_prepare_poll(uint32_t *detail_out);
void zy100_online_spool_start_prepare_abort(void);
void zy100_online_spool_begin(uint32_t session_id);
void zy100_online_spool_note_capture_segment(uint32_t capture_segment_id,
                                              uint32_t segment_start_offset_ms);
void zy100_online_spool_abort_session(void);
bool zy100_online_spool_post_session_cleanup_begin(void);
bool zy100_online_spool_abort_cleanup_begin(void);
zy100_online_spool_pump_result_t
zy100_online_spool_post_session_cleanup_poll(uint32_t target_bytes);
bool zy100_online_spool_post_session_ready(uint32_t minimum_bytes);
#if ZY100_ONLINE_RECOVERY_FAULT_INJECT_ENABLE
void zy100_online_spool_test_force_cleanup_error(bool enable);
#endif
const zy100_fe_store_target_provider_t *zy100_online_spool_target_provider(void);

#if ZY100_ONLINE_STRESS_TEST_ENABLE
zy100_fe_store_reserve_result_t zy100_online_spool_reserve_stress(
    uint32_t source_id, uint32_t source_record_bytes, uint32_t payload_bytes,
    zy100_fe_store_target_t *target);
#endif

zy100_fe_store_reserve_result_t zy100_online_spool_reserve_raw(
    uint32_t source_id,
    uint32_t source_record_bytes,
    uint32_t payload_bytes,
    zy100_fe_store_target_t *target);
zy100_online_spool_io_result_t zy100_online_spool_write_reserved_page(
    uint32_t token,
    uint32_t addr,
    const uint8_t *page);
zy100_online_spool_io_result_t zy100_online_spool_read_reserved(
    uint32_t token,
    uint32_t addr,
    uint8_t *buf,
    uint16_t len);
/* source_crc32 is the finalized IEEE CRC32 of the padded source record. */
bool zy100_online_spool_commit_with_crc(uint32_t token,
                                         uint32_t source_crc32);
void zy100_online_spool_abort_reservation(uint32_t token);

zy100_online_spool_pump_result_t zy100_online_spool_commit_pump_once(
    bool allow_flash_read);
zy100_online_spool_io_result_t zy100_online_spool_peek_tx(
    zy100_online_spool_record_t *record);
bool zy100_online_spool_mark_tx_sent(uint8_t type, uint32_t record_id);
zy100_online_spool_io_result_t zy100_online_spool_read(
    uint32_t addr,
    uint8_t *buf,
    uint16_t len);
bool zy100_online_spool_ack(uint8_t type, uint32_t record_id);

bool zy100_online_spool_preerase_poll(uint32_t target_bytes);
bool zy100_online_spool_erase_reclaimable_poll(void);
zy100_online_spool_pump_result_t
zy100_online_spool_erase_reclaimable_step(void);
zy100_online_spool_pump_result_t
zy100_online_spool_erase_reclaimable_block32_step(void);
zy100_online_spool_erase_result_t
zy100_online_spool_erase_forward_block32_step(void);
/* Experimental target only; ownership/bounds are identical to the legacy helper. */
zy100_online_spool_erase_result_t zy100_online_spool_erase_forward_to(uint32_t target_bytes);
bool zy100_online_spool_erase_wip(void);
/* Stress-only hint: issued page awaiting readback, bus free; no poll here. */
bool zy100_online_spool_page_retry_pending(void);
bool zy100_online_spool_flash_idle_poll(void);
bool zy100_online_spool_commit_pending(void);
uint8_t zy100_online_spool_active_reservation_kind(void);
bool zy100_online_spool_active_reservation_data_done(void);
bool zy100_online_spool_has_committed(void);
bool zy100_online_spool_all_acked(void);
bool zy100_online_spool_low_water(void);
bool zy100_online_spool_has_reclaimable(void);
bool zy100_online_spool_try_wrap_empty(uint32_t minimum_bytes);
zy100_online_spool_reserve_block_t
zy100_online_spool_last_reserve_block(void);
uint32_t zy100_online_spool_pending_bytes(void);
uint32_t zy100_online_spool_erased_ahead_bytes(void);
uint32_t zy100_online_spool_progress_seq(void);
uint8_t zy100_online_spool_invariant_code(void);
void zy100_online_spool_get_stats(zy100_online_spool_stats_t *out);

bool zy100_online_spool_clear_begin(void);
zy100_flash_prepare_status_t zy100_online_spool_clear_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_ONLINE_SPOOL_H */
