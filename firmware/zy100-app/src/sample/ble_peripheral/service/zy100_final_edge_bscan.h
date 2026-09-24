#ifndef ZY100_FINAL_EDGE_BSCAN_H
#define ZY100_FINAL_EDGE_BSCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct __attribute__((packed))
{
    uint16_t tmst_raw;
    uint16_t frame_index;
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} zy100_final_edge_bscan_ds_sample_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(zy100_final_edge_bscan_ds_sample_t) == 16U,
               "zy100_final_edge_bscan_ds_sample_t must be 16 bytes");
#else
typedef char zy100_final_edge_bscan_ds_sample_must_be_16[
    (sizeof(zy100_final_edge_bscan_ds_sample_t) == 16U) ? 1 : -1];
#endif

typedef struct
{
    uint32_t enabled;
    uint32_t decim;
    uint32_t count;
    uint32_t overflow;
    uint32_t store_count;
    uint32_t first_frame_index;
    uint32_t last_frame_index;
    uint32_t first_tmst_raw;
    uint32_t last_tmst_raw;
    uint32_t bytes;
} zy100_final_edge_bscan_replay_raw_stats_t;

#define ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID 0xFFFFU
#define ZY100_FINAL_EDGE_BSCAN_SEQ32_INVALID 0xFFFFFFFFU

typedef enum
{
    ZY100_FINAL_EDGE_BSCAN_EXIT_NONE = 0U,
    ZY100_FINAL_EDGE_BSCAN_EXIT_BEGIN_FAIL = 1U,
    ZY100_FINAL_EDGE_BSCAN_EXIT_ACQ_EMPTY_TIMEOUT = 2U,
    ZY100_FINAL_EDGE_BSCAN_EXIT_ACQ_READ_ERR = 3U,
    ZY100_FINAL_EDGE_BSCAN_EXIT_NULL_FRAME = 4U,
    ZY100_FINAL_EDGE_BSCAN_EXIT_ACQ_HARD_FAULT = 5U,
    ZY100_FINAL_EDGE_BSCAN_EXIT_DT_BAD = 6U,
    ZY100_FINAL_EDGE_BSCAN_EXIT_STALE = 7U,
    ZY100_FINAL_EDGE_BSCAN_EXIT_HIT_WINDOW_DONE = 8U,
    ZY100_FINAL_EDGE_BSCAN_EXIT_NOHIT_BASE_DONE = 9U,
    ZY100_FINAL_EDGE_BSCAN_EXIT_MAX_FRAMES = 10U,
    ZY100_FINAL_EDGE_BSCAN_EXIT_CLEANUP_FAIL = 11U,
} zy100_final_edge_bscan_exit_reason_t;

typedef struct
{
    uint16_t od_early_count;
    uint16_t od_mid_count;
    uint16_t od_tail_count;
    uint16_t od_post_count;
    uint16_t od_first_seq;
    uint16_t od_last_seq;
    uint16_t od_max_pending;
    uint16_t od_total_count;

    uint32_t ois_read_us_max;
    uint16_t ois_read_us_max_seq;

    uint16_t dt_bad_first_seq;
    uint16_t dt_bad_last_seq;
    uint16_t dt_bad_count;

    uint16_t tmst_delta_min;
    uint16_t tmst_delta_max;
    uint16_t tmst_delta_156_157_count;
    uint16_t tmst_delta_bad_count;
    uint16_t tmst_delta_zero_count;
    uint16_t tmst_delta_double_count;
    uint16_t tmst_delta_other_bad_count;
    uint16_t tmst_delta_first_bad_seq;
    uint16_t tmst_delta_last_bad_seq;
} zy100_final_edge_bscan_diag_t;

typedef struct
{
    uint32_t frames;
    uint32_t frames_seen;
    uint32_t bytes;
    uint32_t duration_acc_us;
    uint32_t over;
    uint32_t drop;
    uint32_t stale;
    uint32_t read_err;
    uint32_t dt_bad;
    uint32_t err;
    uint32_t post_truncated;
    uint32_t owner_ok;
    uint32_t cleanup_ok;
    uint32_t begin_retry;
    uint32_t exit_reason;
    uint32_t exit_frames;
    uint32_t exit_last_seq;
    uint32_t exit_elapsed_ms;
    uint32_t exit_idle_spin;
    uint32_t timer_irq_total;
    uint32_t first_frame_read_done_us;
    uint32_t acq_overflow;
    uint32_t acq_read_err;
    uint32_t acq_raw_read_err;
    uint32_t acq_irq_late_count;
    uint32_t acq_irq_early_count;
    uint32_t acq_irq_late_gap_count;
    uint32_t acq_irq_gap_us_min;
    uint32_t acq_irq_gap_us_max;
    uint32_t acq_read_us_max;
    uint32_t acq_high_water;
    uint32_t acq_first_irq_us;
    uint32_t acq_first_read_done_us;
    uint32_t acq_wr_seq;
    uint32_t acq_rd_seq;
    uint32_t acq_skipped_after_stop;
    uint32_t acq_phase_sync_status;
    uint32_t acq_phase_sync_probes;
    uint32_t acq_phase_sync_seed_tmst;
    uint32_t acq_phase_sync_lock_tmst;
    uint32_t acq_phase_sync_delta;
    uint32_t acq_phase_sync_us;
    uint32_t acq_phase_sync_read_us_max;
    uint32_t acq_begin_fail_stage;
    uint32_t acq_begin_fail_status;
    uint32_t acq_phase_sync_no_change_count;
    uint32_t acq_phase_sync_soft_start_count;
    uint32_t acq_tmst_gate_enabled;
    uint32_t acq_tmst_poll_hz;
    uint32_t acq_tmst_new_count;
    uint32_t acq_tmst_dup_drop;
    uint32_t acq_tmst_read_err;
    uint32_t acq_tmst_raw_mismatch;
    uint32_t acq_tmst_mismatch_last_seq;
    uint32_t acq_tmst_mismatch_probe_tmst;
    uint32_t acq_tmst_mismatch_raw_tmst;
    uint32_t acq_tmst_startup_mismatch_drop;
    uint32_t acq_tmst_startup_mismatch_last_probe_tmst;
    uint32_t acq_tmst_startup_mismatch_last_raw_tmst;
    uint32_t acq_tmst_startup_mismatch_hard_limit;
    uint32_t acq_tmst_read_us_max;
    uint32_t acq_raw_read_us_max;
    uint32_t acq_raw_phase_dt_bad;
    uint32_t acq_raw_phase_delta_min;
    uint32_t acq_raw_phase_delta_max;
    uint32_t hit_found;
    uint32_t hit_index;
    uint32_t hit_score;
    uint32_t hit_score20;
    uint32_t hit_tmst_raw;
    uint32_t peak_index;
    uint32_t peak_score;
    uint32_t peak16_index;
    uint32_t peak16_score;
    uint32_t peak20_index;
    uint32_t peak20_score;
    uint32_t use20;
    uint32_t ois20_decode_err;
    int32_t hit20_calib_first_cross_idx[7];
    uint32_t hit20_calib_first_cross_score[7];
    uint32_t hit20_calib_first_cross_tmst[7];
    uint32_t hit_window_valid;
    uint32_t target_window_frames;
    uint32_t ring_valid_frames;
    uint32_t hf_start_seq;
    uint32_t hf_end_seq;
    uint32_t hf_frame_count;
    uint32_t lf_pre_needed_frames;
    uint32_t lf_pre_packets;
    uint32_t ring_start_idx;
    uint32_t target_end_seq;
    uint32_t last_saved_seq;
    /* Legacy alias: final_edge Phase 4 treats this as last_saved_seq. */
    uint32_t current_saved_seq;
    uint16_t first_tmst_raw;
    uint16_t last_tmst_raw;
    bool first_tmst_valid;
    bool last_tmst_valid;
    zy100_final_edge_bscan_diag_t diag;
} zy100_final_edge_bscan_result_t;

typedef struct
{
    uint8_t valid;
    uint8_t complete;
    uint8_t early_hit;
    uint8_t wraps;

    uint32_t bscan_id;
    uint32_t hit_seq;
    uint32_t start_seq;
    uint32_t end_seq;
    uint32_t ring_start_index;

    uint32_t hf_pre_frames;
    uint32_t hf_post_frames;
    uint32_t hf_frames;

    uint32_t lf_pre_frames_needed;
    uint32_t lf_pre_packets_needed;
    uint32_t lf_pre_packets_available;
    uint32_t lf_pre_valid;

    uint16_t first_tmst_raw;
    uint16_t hit_tmst_raw;
    uint16_t last_tmst_raw;

    uint32_t window_xor;
} zy100_final_edge_hit_window_view_t;

bool zy100_final_edge_bscan_run(
    uint8_t *buffer,
    uint32_t buffer_bytes,
    zy100_final_edge_bscan_result_t *result);
bool zy100_final_edge_bscan_extract_hit_window_view(
    const uint8_t *raw_ring,
    uint32_t raw_ring_bytes,
    const zy100_final_edge_bscan_result_t *result,
    uint32_t bscan_id,
    zy100_final_edge_hit_window_view_t *view);
bool zy100_final_edge_bscan_hit_window_get_frame(
    const uint8_t *raw_ring,
    uint32_t raw_ring_bytes,
    const zy100_final_edge_hit_window_view_t *view,
    uint32_t offset,
    const uint8_t **frame_out);
uint32_t zy100_final_edge_bscan_get_ds_count(void);
uint32_t zy100_final_edge_bscan_get_ds_overflow(void);
bool zy100_final_edge_bscan_get_ds_sample(
    uint32_t index,
    zy100_final_edge_bscan_ds_sample_t *out);
bool zy100_final_edge_bscan_get_replay_raw_stats(
    zy100_final_edge_bscan_replay_raw_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_FINAL_EDGE_BSCAN_H */
