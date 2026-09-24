#ifndef ZY100_FINAL_EDGE_RAW_STORE_H
#define ZY100_FINAL_EDGE_RAW_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "zy100_flash_common.h"
#include "zy100_final_edge_bscan.h"
#include "zy100_final_edge_raw_format.h"
#include "zy100_final_edge_store_target.h"

typedef enum
{
    ZY100_FE_RAW_PUMP_NO_WORK = 0U,
    ZY100_FE_RAW_PUMP_WIP_BUSY,
    ZY100_FE_RAW_PUMP_PAGE_ISSUED,
    ZY100_FE_RAW_PUMP_RECORD_DONE,
    ZY100_FE_RAW_PUMP_ERROR,
} zy100_fe_raw_pump_result_t;

typedef enum
{
    ZY100_FE_RAW_PUMP_CONTEXT_RUNTIME = 0U,
    ZY100_FE_RAW_PUMP_CONTEXT_FINAL,
} zy100_fe_raw_pump_context_t;

typedef struct
{
    uint32_t prepared;
    uint32_t running;
    uint32_t active;
    uint32_t pending;
    uint32_t program_in_flight;
    uint32_t finalizing;
    uint32_t raw_begin;
    uint32_t raw_saved;
    uint32_t raw_failed;
    uint32_t raw_full;
    uint32_t raw_lf_fail;
    uint32_t raw_hwin_fail;
    uint32_t page_program;
    uint32_t online_target_page_program;
    uint32_t online_offline_region_write_violation;
    uint32_t wip_busy;
    uint32_t write_error;
    uint32_t verify_error;
    uint32_t erase_count;
    uint32_t prepare_erase_count;
    uint32_t runtime_erase_count;
    uint32_t bucket_count;
    uint32_t session_raw_first_bucket;
    uint32_t session_raw_limit_bucket;
    uint32_t next_raw_id;
    uint32_t used_bytes;
    uint32_t last_raw_id;
    uint32_t last_bscan_id;
    uint32_t last_sections;
    uint32_t last_hf_frames;
    uint32_t last_lf_packets;
    uint32_t last_payload_bytes;
    uint32_t last_page_count;
    uint32_t last_flags;
    uint32_t rt_gate_count;
    uint32_t rt_gate_max_ms;
    uint32_t b_active_pump_blocked;
    uint32_t pump_max_us;
    uint32_t runtime_pump_max_us;
    uint32_t final_pump_max_us;
    uint32_t page_issue_max_us;
    uint32_t verify_max_us;
    uint32_t verify_page_count;
    uint32_t raw_record_save_count;
    uint32_t raw_record_save_total_ms;
    uint32_t raw_record_save_max_ms;
    uint32_t raw_record_save_last_ms;
    uint32_t target_wait_busy;
    uint32_t target_wait_space;
    uint32_t target_retry_count;
    uint32_t target_retry_ok;
    uint32_t target_retry_error;
    uint32_t target_wait_total_ms;
    uint32_t target_wait_max_ms;
    uint32_t target_wait_last_ms;
    uint32_t target_wait_bscan_id;
    uint32_t target_last_reserve_result;
    uint32_t last_error_stage;
    uint32_t last_error_status;
    uint32_t last_error_raw_id;
    uint32_t last_error_addr;
    uint32_t last_error_expected;
    uint32_t last_error_readback;
} zy100_fe_raw_store_stats_t;

typedef struct
{
    uint32_t phase;
    uint32_t erase;
    uint32_t total;
    uint32_t done;
    uint32_t err;
} zy100_fe_raw_prepare_progress_t;

bool zy100_final_edge_raw_store_prepare_erase_begin(uint32_t round);
bool zy100_final_edge_raw_store_prepare_erase_range_begin(
    uint32_t round,
    uint32_t raw_first_bucket,
    uint32_t raw_limit_bucket);
bool zy100_final_edge_raw_store_prepare_append_begin(
    uint32_t round,
    uint32_t raw_data_begin_addr,
    uint32_t raw_region_end_addr);
bool zy100_final_edge_raw_store_prepare_target_begin(
    uint32_t round,
    const zy100_fe_store_target_provider_t *provider);
zy100_flash_prepare_status_t zy100_final_edge_raw_store_prepare_erase_poll(void);
bool zy100_final_edge_raw_store_start(uint32_t round);
void zy100_final_edge_raw_store_reset_runtime(void);
void zy100_final_edge_raw_store_reset_after_clear(uint32_t round);
void zy100_final_edge_raw_store_abort_pending(void);

bool zy100_final_edge_raw_store_begin_hit(
    const uint8_t *raw_ring,
    uint32_t raw_ring_bytes,
    const zy100_final_edge_bscan_result_t *result,
    const zy100_final_edge_hit_window_view_t *view,
    uint32_t bscan_id,
    uint32_t rt_trigger_sample,
    uint16_t rt_trigger_tmst_raw,
    bool ois_od_warn);

zy100_fe_raw_pump_result_t zy100_final_edge_raw_store_pump_once_ex(
    zy100_fe_raw_pump_context_t context);
zy100_fe_raw_pump_result_t zy100_final_edge_raw_store_pump_once(void);
bool zy100_final_edge_raw_store_has_pending_work(void);
bool zy100_final_edge_raw_store_program_in_flight(void);
bool zy100_final_edge_raw_store_waiting_target(void);
bool zy100_final_edge_raw_store_waiting_target_space(void);
zy100_fe_store_reserve_result_t
zy100_final_edge_raw_store_retry_target(void);
bool zy100_final_edge_raw_store_rt_gate_active(void);
bool zy100_final_edge_raw_store_has_error(void);
bool zy100_final_edge_raw_store_is_idle(void);
void zy100_final_edge_raw_store_get_stats(zy100_fe_raw_store_stats_t *out);
void zy100_final_edge_raw_store_online_diag_reset(void);
void zy100_final_edge_raw_store_get_online_diag(uint32_t *verified_pages,
                                                 uint32_t *address_violations);
void zy100_final_edge_raw_store_get_prepare_progress(
    zy100_fe_raw_prepare_progress_t *out);

void zy100_final_edge_raw_store_set_b_active(bool active);
void zy100_final_edge_raw_store_note_rt_gate(uint32_t now_ms);
void zy100_final_edge_raw_store_set_finalizing(bool finalizing);
void zy100_final_edge_raw_store_set_target_provider(
    const zy100_fe_store_target_provider_t *provider);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_FINAL_EDGE_RAW_STORE_H */
