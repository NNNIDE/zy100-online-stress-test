#ifndef OIS_ABA_V0_SRAM_CAPTURE_H
#define OIS_ABA_V0_SRAM_CAPTURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../app_flags.h"
#include "zy100_final_edge_bscan.h"

typedef zy100_final_edge_bscan_ds_sample_t final_edge_ois_ds_sample_t;
typedef zy100_final_edge_bscan_result_t
    zy100_ois_aba_v0_final_edge_b_scan_result_t;

typedef enum
{
    ZY100_FE_B_ACQ_CONSUME_EMPTY = 0U,
    ZY100_FE_B_ACQ_CONSUME_OK = 1U,
    ZY100_FE_B_ACQ_CONSUME_READ_ERR = 2U,
} zy100_fe_b_acq_consume_status_t;

typedef enum
{
    ZY100_FE_B_ACQ_PHASE_SYNC_DISABLED = 0U,
    ZY100_FE_B_ACQ_PHASE_SYNC_OK = 1U,
    ZY100_FE_B_ACQ_PHASE_SYNC_READ_ERR = 2U,
    ZY100_FE_B_ACQ_PHASE_SYNC_NO_CHANGE = 3U,
} zy100_fe_b_acq_phase_sync_status_t;

typedef enum
{
    ZY100_FE_B_ACQ_BEGIN_FAIL_NONE = 0U,
    ZY100_FE_B_ACQ_BEGIN_FAIL_BUSY = 1U,
    ZY100_FE_B_ACQ_BEGIN_FAIL_A1_TO_B_PREP = 2U,
    ZY100_FE_B_ACQ_BEGIN_FAIL_OIS_ENABLE = 3U,
    ZY100_FE_B_ACQ_BEGIN_FAIL_SPI_ACQUIRE = 4U,
    ZY100_FE_B_ACQ_BEGIN_FAIL_ACQ_RESET = 5U,
    ZY100_FE_B_ACQ_BEGIN_FAIL_TIMER_ACQUIRE = 6U,
    ZY100_FE_B_ACQ_BEGIN_FAIL_TIMER_CONFIG = 7U,
    ZY100_FE_B_ACQ_BEGIN_FAIL_PHASE_SYNC_READ_ERR = 8U,
    ZY100_FE_B_ACQ_BEGIN_FAIL_TIMER_START = 9U,
} zy100_fe_b_acq_begin_fail_stage_t;

typedef struct
{
    uint32_t wr_seq;
    uint32_t rd_seq;
    uint32_t overflow;
    uint32_t read_err;
    uint32_t raw_read_err;
    uint32_t skipped_after_stop;
    uint32_t irq_total;
    uint32_t first_irq_us;
    uint32_t first_read_done_us;
    uint32_t read_us_max;
    uint32_t irq_late_count;
    uint32_t irq_early_count;
    uint32_t irq_late_gap_count;
    uint32_t irq_gap_us_min;
    uint32_t irq_gap_us_max;
    uint32_t high_water;
    uint32_t overflow_first_slot_seq;
    uint32_t overflow_last_slot_seq;
    uint32_t phase_sync_status;
    uint32_t phase_sync_probes;
    uint32_t phase_sync_seed_tmst;
    uint32_t phase_sync_lock_tmst;
    uint32_t phase_sync_delta;
    uint32_t phase_sync_us;
    uint32_t phase_sync_read_us_max;
    uint32_t begin_fail_stage;
    uint32_t begin_fail_status;
    uint32_t phase_sync_no_change_count;
    uint32_t phase_sync_soft_start_count;
    uint32_t tmst_gate_enabled;
    uint32_t tmst_poll_hz;
    uint32_t tmst_new_count;
    uint32_t tmst_dup_drop;
    uint32_t tmst_read_err;
    uint32_t tmst_raw_mismatch;
    uint32_t tmst_mismatch_last_seq;
    uint32_t tmst_mismatch_probe_tmst;
    uint32_t tmst_mismatch_raw_tmst;
    uint32_t tmst_startup_mismatch_drop;
    uint32_t tmst_startup_mismatch_last_probe_tmst;
    uint32_t tmst_startup_mismatch_last_raw_tmst;
    uint32_t tmst_startup_mismatch_hard_limit;
    uint32_t tmst_read_us_max;
    uint32_t raw_read_us_max;
    uint32_t raw_phase_dt_bad;
    uint32_t raw_phase_delta_min;
    uint32_t raw_phase_delta_max;
} zy100_fe_b_acq_stats_t;

/* legacy ABA public API */
bool zy100_ois_aba_v0_sram_capture_run_once_blocking(void);
bool zy100_ois_aba_v0_sram_capture_has_run(void);
bool zy100_ois_aba_v0_sram_capture_is_busy(void);
bool zy100_ois_aba_v0_sram_capture_consume_just_finished(void);

/* old final_edge compatibility wrappers.
 * TODO Phase 7 remove after final_edge bscan fully detached.
 */
bool zy100_ois_aba_v0_final_edge_b_scan_run(
    uint8_t *buffer,
    uint32_t buffer_bytes,
    zy100_ois_aba_v0_final_edge_b_scan_result_t *result);
bool zy100_ois_aba_v0_final_edge_extract_hit_window_view(
    const uint8_t *raw_ring,
    uint32_t raw_ring_bytes,
    const zy100_ois_aba_v0_final_edge_b_scan_result_t *result,
    uint32_t bscan_id,
    zy100_final_edge_hit_window_view_t *view);
bool zy100_ois_aba_v0_final_edge_hit_window_get_frame(
    const uint8_t *raw_ring,
    uint32_t raw_ring_bytes,
    const zy100_final_edge_hit_window_view_t *view,
    uint32_t offset,
    const uint8_t **frame_out);
uint32_t zy100_ois_aba_v0_final_edge_get_ds_count(void);
uint32_t zy100_ois_aba_v0_final_edge_get_ds_overflow(void);
bool zy100_ois_aba_v0_final_edge_get_ds_sample(
    uint32_t index,
    final_edge_ois_ds_sample_t *out);

/* final_edge temporary bridge API.
 * TODO Phase 7 remove after final_edge bscan fully detached.
 */
bool zy100_ois_aba_v0_final_edge_bridge_begin_bscan(
    zy100_final_edge_bscan_result_t *result,
    uint8_t *raw_ring,
    uint32_t raw_ring_bytes);
uint32_t zy100_ois_aba_v0_final_edge_bridge_take_pending_ticks(void);
bool zy100_ois_aba_v0_final_edge_bridge_read_ois_frame(
    uint8_t frame[ZY100_FINAL_EDGE_OIS_FRAME_BYTES]);
zy100_fe_b_acq_consume_status_t
zy100_ois_aba_v0_final_edge_bridge_consume_acq_frame(
    uint32_t *slot_seq_out,
    const uint8_t **frame_out);
void zy100_ois_aba_v0_final_edge_bridge_acq_set_stop_after(
    uint32_t stop_after_seq);
void zy100_ois_aba_v0_final_edge_bridge_get_acq_stats(
    zy100_fe_b_acq_stats_t *out);
bool zy100_ois_aba_v0_final_edge_bridge_end_bscan(
    zy100_final_edge_bscan_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* OIS_ABA_V0_SRAM_CAPTURE_H */
