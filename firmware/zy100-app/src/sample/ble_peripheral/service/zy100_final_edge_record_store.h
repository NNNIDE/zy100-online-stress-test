#ifndef ZY100_FINAL_EDGE_RECORD_STORE_H
#define ZY100_FINAL_EDGE_RECORD_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "zy100_flash_common.h"
#include "zy100_capture_time.h"
#include "zy100_final_edge_record_format.h"
#include "zy100_final_edge_store_target.h"

#define ZY100_FE_SESSION_PASS_RAW      (1UL << 0)
#define ZY100_FE_SESSION_PASS_REPLAY   (1UL << 1)
#define ZY100_FE_SESSION_PASS_SUMMARY  (1UL << 2)
#define ZY100_FE_SESSION_PASS_EVENT    (1UL << 3)
#define ZY100_FE_SESSION_PASS_META     (1UL << 4)
#define ZY100_FE_SESSION_PASS_PHASE7   (1UL << 5)

#define ZY100_FE_SESSION_WARN_EMPTY_SUMMARY (1UL << 0)
#define ZY100_FE_SESSION_WARN_EMPTY_EVENT   (1UL << 1)

#define ZY100_FE_SESSION_ERROR_RAW        (1UL << 0)
#define ZY100_FE_SESSION_ERROR_REPLAY     (1UL << 1)
#define ZY100_FE_SESSION_ERROR_QUEUE_DROP (1UL << 2)
#define ZY100_FE_SESSION_ERROR_STORE      (1UL << 3)
#define ZY100_FE_SESSION_ERROR_META       (1UL << 4)

typedef enum
{
    ZY100_FE_REC_PUMP_NO_WORK = 0U,
    ZY100_FE_REC_PUMP_WIP_BUSY,
    ZY100_FE_REC_PUMP_PAGE_ISSUED,
    ZY100_FE_REC_PUMP_RECORD_DONE,
    ZY100_FE_REC_PUMP_ERROR,
} zy100_fe_record_pump_result_t;

typedef struct
{
    uint32_t prepared;
    uint32_t started;
    uint32_t active;
    uint32_t pending;
    uint32_t program_in_flight;

    uint32_t summary_begin;
    uint32_t summary_saved;
    uint32_t summary_failed;
    uint32_t summary_full;

    uint32_t event_begin;
    uint32_t event_saved;
    uint32_t event_failed;
    uint32_t event_full;

    uint32_t page_program;
    uint32_t online_target_page_program;
    uint32_t online_offline_region_write_violation;
    uint32_t wip_busy;
    uint32_t record_wip_busy;
    uint32_t write_error;
    uint32_t verify_error;

    uint32_t prepare_erase_count;
    uint32_t runtime_erase_count;

    uint32_t summary_next_addr;
    uint32_t event_next_addr;
    uint32_t summary_base_addr;
    uint32_t summary_limit_addr;
    uint32_t event_base_addr;
    uint32_t event_limit_addr;
    uint32_t summary_used_bytes;
    uint32_t event_used_bytes;

    uint32_t last_type;
    uint32_t last_seq;
    uint32_t last_payload_bytes;
    uint32_t last_page_count;

    uint32_t b_active_pump_blocked;
    uint32_t replay_pump_blocked;
    uint32_t raw_busy_blocked;
    uint32_t raw_wait_record_wip_count;

    uint32_t pump_max_us;
    uint32_t issue_max_us;
    uint32_t verify_max_us;

    uint32_t record_final_pump_max_us;
    uint32_t record_final_pump_timeout;
    uint32_t record_final_pending_left;

    uint32_t meta_page_program;
    uint32_t meta_write_ok;
    uint32_t meta_verify_ok;
    uint32_t meta_write_error;
    uint32_t meta_verify_error;
    uint32_t meta_bytes;
    uint32_t meta_flags;
} zy100_fe_record_store_stats_t;

typedef struct
{
    uint32_t round;
    uint32_t start_ms;
    uint32_t end_ms;
    uint32_t official_hit_count;
    uint32_t nohit_count;
    uint32_t raw_count;
    uint32_t raw_used_bytes;
    uint32_t pass_flags;
    uint32_t warn_flags;
    uint32_t error_flags;
    uint8_t rtc_meta[ZY100_CAPTURE_TIME_META_BYTES];
} zy100_fe_session_meta_input_t;

bool zy100_final_edge_record_store_prepare_erase_begin(uint32_t round);
bool zy100_final_edge_record_store_prepare_erase_range_begin(
    uint32_t round,
    uint32_t summary_base_addr,
    uint32_t summary_limit_addr,
    uint32_t event_base_addr,
    uint32_t event_limit_addr,
    bool erase_meta_scratch);
bool zy100_final_edge_record_store_prepare_append_begin(
    uint32_t round,
    uint32_t summary_data_begin_addr,
    uint32_t summary_region_end_addr,
    uint32_t event_data_begin_addr,
    uint32_t event_region_end_addr);
bool zy100_final_edge_record_store_prepare_target_begin(
    uint32_t round,
    const zy100_fe_store_target_provider_t *provider);
zy100_flash_prepare_status_t zy100_final_edge_record_store_prepare_erase_poll(void);
bool zy100_final_edge_record_store_start(uint32_t round);
void zy100_final_edge_record_store_reset_runtime(void);
void zy100_final_edge_record_store_reset_after_clear(uint32_t round);

zy100_fe_record_pump_result_t zy100_final_edge_record_store_pump_once(void);
zy100_fe_record_pump_result_t
zy100_final_edge_record_store_pump_active_once(void);

bool zy100_final_edge_record_store_has_pending_work(void);
bool zy100_final_edge_record_store_has_active_record(void);
bool zy100_final_edge_record_store_is_idle(void);
bool zy100_final_edge_record_store_has_error(void);
bool zy100_final_edge_record_store_program_in_flight(void);
void zy100_final_edge_record_store_abort_active(void);
bool zy100_final_edge_record_store_write_session_meta(
    const zy100_fe_session_meta_input_t *input);
bool zy100_final_edge_record_store_build_session_meta(
    const zy100_fe_session_meta_input_t *input,
    zy100_fe_session_meta_t *out);
void zy100_final_edge_record_store_get_stats(
    zy100_fe_record_store_stats_t *out);
void zy100_final_edge_record_store_online_diag_reset(void);
void zy100_final_edge_record_store_get_online_diag(uint32_t *verified_pages,
                                                    uint32_t *address_violations);

void zy100_final_edge_record_store_set_b_active(bool active);
void zy100_final_edge_record_store_note_replay_blocked(void);
void zy100_final_edge_record_store_note_raw_busy_blocked(void);
void zy100_final_edge_record_store_note_raw_wait_record_wip(void);
void zy100_final_edge_record_store_note_final_pump(uint32_t elapsed_us,
                                                   bool timeout,
                                                   uint32_t pending_left);
void zy100_final_edge_record_store_set_target_provider(
    const zy100_fe_store_target_provider_t *provider);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_FINAL_EDGE_RECORD_STORE_H */
