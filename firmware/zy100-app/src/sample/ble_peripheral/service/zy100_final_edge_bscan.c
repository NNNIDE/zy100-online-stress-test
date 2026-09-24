#include "zy100_final_edge_bscan.h"

#include <string.h>

#include "../app_flags.h"
#include "../bsp/imu_bsp.h"
#include "../common/imu_common.h"
#include "../driver/icm53611_driver.h"
#include "../zy100_clock_config.h"
#include "ois_aba_v0_sram_capture.h"

#ifndef IMU_UNUSED
#define IMU_UNUSED(x) ((void)(x))
#endif

#define ZY100_FINAL_EDGE_PHASE3_ACTIVE \
    (ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_FINAL_EDGE_BSCAN_BUILD_ENABLE)

#define ZY100_FINAL_EDGE_BSCAN_GUARD_MS 300U
#define ZY100_FINAL_EDGE_BSCAN_HIT20_CALIB_COUNT 7U
#define ZY100_FINAL_EDGE_BSCAN_OD_EARLY_END_SEQ 320U
#define ZY100_FINAL_EDGE_BSCAN_OD_MID_END_SEQ 960U
#define ZY100_FINAL_EDGE_BSCAN_OD_TAIL_END_SEQ 1280U

typedef char zy100_final_edge_bscan_frame_size_check[
    (ZY100_FINAL_EDGE_OIS_FRAME_BYTES == ICM53611_OIS_RAW_FRAME_BYTES) ?
        1 : -1];

#if ZY100_FINAL_EDGE_PHASE3_ACTIVE

#if ZY100_FINAL_EDGE_HIT20_CALIB_ENABLE
static const uint32_t s_fe_bscan_hit20_calib_thresholds
    [ZY100_FINAL_EDGE_BSCAN_HIT20_CALIB_COUNT] =
{
    ZY100_FINAL_EDGE_HIT20_CALIB_TH0,
    ZY100_FINAL_EDGE_HIT20_CALIB_TH1,
    ZY100_FINAL_EDGE_HIT20_CALIB_TH2,
    ZY100_FINAL_EDGE_HIT20_CALIB_TH3,
    ZY100_FINAL_EDGE_HIT20_CALIB_TH4,
    ZY100_FINAL_EDGE_HIT20_CALIB_TH5,
    ZY100_FINAL_EDGE_HIT20_CALIB_TH6,
};
#endif

typedef struct __attribute__((packed))
{
    uint8_t frame[ICM53611_OIS_RAW_FRAME_BYTES];
} zy100_final_edge_bscan_replay_raw_frame_t;

typedef char zy100_final_edge_bscan_replay_raw_frame_size_check[
    (sizeof(zy100_final_edge_bscan_replay_raw_frame_t) ==
     ICM53611_OIS_RAW_FRAME_BYTES) ? 1 : -1];

typedef struct
{
    zy100_final_edge_bscan_replay_raw_frame_t raw
        [ZY100_FINAL_EDGE_OIS_DS_MAX_SAMPLES];
    uint32_t ds_count;
    uint32_t ds_overflow;
    uint32_t ds_store_count;
    uint32_t ds_last_frame_index;
    uint16_t ds_first_tmst_raw;
    uint16_t ds_last_tmst_raw;
} zy100_final_edge_bscan_ds_ctx_t;

typedef struct
{
    int16_t prev16_ax;
    int16_t prev16_ay;
    int16_t prev16_az;
    bool prev16_valid;
    int32_t prev20_ax;
    int32_t prev20_ay;
    int32_t prev20_az;
    bool prev20_valid;
} zy100_final_edge_bscan_hit_state_t;

static zy100_final_edge_bscan_ds_ctx_t s_fe_bscan_ds_ctx;

static uint32_t fe_bscan_elapsed_ms(uint32_t start_ms)
{
    return zy100_os_time_ms() - start_ms;
}

/*
 * Adjacent-sample or known-short-gap helper only. Do not use this for
 * first-to-last 200 ms B duration because the 16-bit IMU timestamp wraps.
 */
static uint16_t fe_bscan_timestamp_delta16_us(uint16_t older, uint16_t newer)
{
    uint16_t delta = (uint16_t)(newer - older);

    return delta;
}

static uint16_t fe_bscan_ois_raw_frame_tmst_raw(const uint8_t *frame)
{
    return (uint16_t)(((uint16_t)frame[14] << 8) | frame[15]);
}

static uint32_t fe_bscan_abs_i32(int32_t value)
{
    return (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
}

static void fe_bscan_diag_reset_result(
    zy100_final_edge_bscan_result_t *result)
{
    if (result == NULL)
    {
        return;
    }

    result->diag.od_first_seq = ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
    result->diag.od_last_seq = ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
    result->diag.ois_read_us_max_seq = ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
    result->diag.dt_bad_first_seq = ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
    result->diag.dt_bad_last_seq = ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
    result->diag.tmst_delta_min = 0xFFFFU;
    result->diag.tmst_delta_max = 0U;
    result->diag.tmst_delta_first_bad_seq =
        ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
    result->diag.tmst_delta_last_bad_seq =
        ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
    result->exit_last_seq = ZY100_FINAL_EDGE_BSCAN_SEQ32_INVALID;
    result->acq_tmst_mismatch_last_seq =
        ZY100_FINAL_EDGE_BSCAN_SEQ32_INVALID;
}

static void fe_bscan_note_exit(
    zy100_final_edge_bscan_result_t *result,
    zy100_final_edge_bscan_exit_reason_t reason,
    uint32_t last_seq,
    uint32_t start_ms,
    uint32_t idle_spin)
{
    if ((result == NULL) || (reason == ZY100_FINAL_EDGE_BSCAN_EXIT_NONE))
    {
        return;
    }

    result->exit_reason = (uint32_t)reason;
    result->exit_frames = result->frames;
    result->exit_last_seq = last_seq;
    result->exit_elapsed_ms =
        (start_ms == 0U) ? 0U : fe_bscan_elapsed_ms(start_ms);
    result->exit_idle_spin = idle_spin;
}

static void fe_bscan_note_exit_if_none(
    zy100_final_edge_bscan_result_t *result,
    zy100_final_edge_bscan_exit_reason_t reason,
    uint32_t last_seq,
    uint32_t start_ms,
    uint32_t idle_spin)
{
    if ((result != NULL) &&
        (result->exit_reason == ZY100_FINAL_EDGE_BSCAN_EXIT_NONE))
    {
        fe_bscan_note_exit(result, reason, last_seq, start_ms, idle_spin);
    }
}

static uint32_t fe_bscan_last_seq_or_invalid(
    const zy100_final_edge_bscan_result_t *result)
{
    if ((result == NULL) || (result->frames == 0U))
    {
        return ZY100_FINAL_EDGE_BSCAN_SEQ32_INVALID;
    }
    return result->last_saved_seq;
}

#if (ZY100_FINAL_EDGE_BSCAN_OD_DIAG_ENABLE || \
     ZY100_FINAL_EDGE_BSCAN_TMST_SUMMARY_ENABLE)
static uint16_t fe_bscan_u32_to_u16_sat(uint32_t value)
{
    return (value > 0xFFFFU) ? 0xFFFFU : (uint16_t)value;
}

static void fe_bscan_inc_u16_sat(uint16_t *value)
{
    if ((value != NULL) && (*value < 0xFFFFU))
    {
        (*value)++;
    }
}
#endif

#if ZY100_FINAL_EDGE_BSCAN_OD_DIAG_ENABLE
#if !ZY100_FINAL_EDGE_B_HARD_RT_ACQ_ENABLE
static void fe_bscan_diag_note_od_event(
    zy100_final_edge_bscan_result_t *result,
    uint32_t seq,
    uint32_t pending)
{
    zy100_final_edge_bscan_diag_t *diag;
    uint16_t seq16;
    uint16_t pending16;

    if (result == NULL)
    {
        return;
    }

    diag = &result->diag;
    seq16 = fe_bscan_u32_to_u16_sat(seq);
    pending16 = fe_bscan_u32_to_u16_sat(pending);

    fe_bscan_inc_u16_sat(&diag->od_total_count);
    if (diag->od_first_seq == ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID)
    {
        diag->od_first_seq = seq16;
    }
    diag->od_last_seq = seq16;
    if (pending16 > diag->od_max_pending)
    {
        diag->od_max_pending = pending16;
    }

    if (seq < ZY100_FINAL_EDGE_BSCAN_OD_EARLY_END_SEQ)
    {
        fe_bscan_inc_u16_sat(&diag->od_early_count);
    }
    else if (seq < ZY100_FINAL_EDGE_BSCAN_OD_MID_END_SEQ)
    {
        fe_bscan_inc_u16_sat(&diag->od_mid_count);
    }
    else if (seq < ZY100_FINAL_EDGE_BSCAN_OD_TAIL_END_SEQ)
    {
        fe_bscan_inc_u16_sat(&diag->od_tail_count);
    }
    else
    {
        fe_bscan_inc_u16_sat(&diag->od_post_count);
    }
}

static void fe_bscan_diag_note_read_us(
    zy100_final_edge_bscan_result_t *result,
    uint32_t seq,
    uint32_t read_us)
{
    if (result == NULL)
    {
        return;
    }

    if (read_us > result->diag.ois_read_us_max)
    {
        result->diag.ois_read_us_max = read_us;
        result->diag.ois_read_us_max_seq = fe_bscan_u32_to_u16_sat(seq);
    }
}
#endif
#endif

#if ZY100_FINAL_EDGE_BSCAN_TMST_SUMMARY_ENABLE
static void fe_bscan_diag_note_tmst_delta(
    zy100_final_edge_bscan_result_t *result,
    uint32_t seq,
    uint16_t delta,
    bool bad)
{
    zy100_final_edge_bscan_diag_t *diag;
    uint16_t seq16;

    if (result == NULL)
    {
        return;
    }

    diag = &result->diag;
    seq16 = fe_bscan_u32_to_u16_sat(seq);
    if (diag->tmst_delta_min == 0xFFFFU)
    {
        diag->tmst_delta_min = delta;
        diag->tmst_delta_max = delta;
    }
    else
    {
        if (delta < diag->tmst_delta_min)
        {
            diag->tmst_delta_min = delta;
        }
        if (delta > diag->tmst_delta_max)
        {
            diag->tmst_delta_max = delta;
        }
    }

    if ((delta == 156U) || (delta == 157U))
    {
        fe_bscan_inc_u16_sat(&diag->tmst_delta_156_157_count);
    }
    if (delta == 0U)
    {
        fe_bscan_inc_u16_sat(&diag->tmst_delta_zero_count);
    }
    if ((delta == 312U) || (delta == 313U))
    {
        fe_bscan_inc_u16_sat(&diag->tmst_delta_double_count);
    }
    if (bad)
    {
        fe_bscan_inc_u16_sat(&diag->tmst_delta_bad_count);
        if ((delta != 0U) && (delta != 312U) && (delta != 313U))
        {
            fe_bscan_inc_u16_sat(&diag->tmst_delta_other_bad_count);
        }
        if (diag->tmst_delta_first_bad_seq ==
            ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID)
        {
            diag->tmst_delta_first_bad_seq = seq16;
        }
        diag->tmst_delta_last_bad_seq = seq16;
        if (diag->dt_bad_first_seq == ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID)
        {
            diag->dt_bad_first_seq = seq16;
        }
        diag->dt_bad_last_seq = seq16;
        fe_bscan_inc_u16_sat(&diag->dt_bad_count);
    }
}
#else
static void fe_bscan_diag_note_tmst_delta(
    zy100_final_edge_bscan_result_t *result,
    uint32_t seq,
    uint16_t delta,
    bool bad)
{
    IMU_UNUSED(result);
    IMU_UNUSED(seq);
    IMU_UNUSED(delta);
    IMU_UNUSED(bad);
}
#endif

#if !ZY100_FINAL_EDGE_BSCAN_OD_DIAG_ENABLE
#if !ZY100_FINAL_EDGE_B_HARD_RT_ACQ_ENABLE
static void fe_bscan_diag_note_od_event(
    zy100_final_edge_bscan_result_t *result,
    uint32_t seq,
    uint32_t pending)
{
    IMU_UNUSED(result);
    IMU_UNUSED(seq);
    IMU_UNUSED(pending);
}
#endif
#endif

static int16_t fe_bscan_be_to_s16(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

static int16_t fe_bscan_sat_ois20_to_i16(int32_t value)
{
    int32_t shifted = value >> 4;

    if (shifted > 32767)
    {
        return 32767;
    }
    if (shifted < -32768)
    {
        return (int16_t)-32768;
    }
    return (int16_t)shifted;
}

static void fe_bscan_ds_reset(void)
{
    memset(&s_fe_bscan_ds_ctx, 0, sizeof(s_fe_bscan_ds_ctx));
}

static void fe_bscan_ds_store_frame(
    const uint8_t raw[ICM53611_OIS_RAW_FRAME_BYTES],
    uint32_t seq,
    uint16_t tmst_raw)
{
    zy100_final_edge_bscan_replay_raw_frame_t *dst;

    if ((raw == NULL) || ((seq % ZY100_FINAL_EDGE_OIS_DECIM) != 0U))
    {
        return;
    }

    if (s_fe_bscan_ds_ctx.ds_count >= ZY100_FINAL_EDGE_OIS_DS_MAX_SAMPLES)
    {
        s_fe_bscan_ds_ctx.ds_overflow++;
        return;
    }

    dst = &s_fe_bscan_ds_ctx.raw[s_fe_bscan_ds_ctx.ds_count];
    memcpy(dst->frame, raw, ICM53611_OIS_RAW_FRAME_BYTES);

    if (s_fe_bscan_ds_ctx.ds_count == 0U)
    {
        s_fe_bscan_ds_ctx.ds_first_tmst_raw = tmst_raw;
    }
    s_fe_bscan_ds_ctx.ds_last_tmst_raw = tmst_raw;
    s_fe_bscan_ds_ctx.ds_last_frame_index = seq;
    s_fe_bscan_ds_ctx.ds_count++;
    s_fe_bscan_ds_ctx.ds_store_count++;
}

static bool fe_bscan_replay_raw_to_ds_sample(
    const zy100_final_edge_bscan_replay_raw_frame_t *src,
    uint32_t index,
    zy100_final_edge_bscan_ds_sample_t *out)
{
    int32_t ax20 = 0;
    int32_t ay20 = 0;
    int32_t az20 = 0;
    const uint8_t *raw;

    if ((src == NULL) || (out == NULL))
    {
        return false;
    }

    raw = src->frame;
    if (!icm53611_decode_ois20_accel_raw(raw, &ax20, &ay20, &az20))
    {
        return false;
    }

    out->tmst_raw = fe_bscan_ois_raw_frame_tmst_raw(raw);
    out->frame_index = (uint16_t)(index * ZY100_FINAL_EDGE_OIS_DECIM);
    out->ax = fe_bscan_sat_ois20_to_i16(ax20);
    out->ay = fe_bscan_sat_ois20_to_i16(ay20);
    out->az = fe_bscan_sat_ois20_to_i16(az20);
    out->gx = fe_bscan_be_to_s16(raw[8], raw[9]);
    out->gy = fe_bscan_be_to_s16(raw[10], raw[11]);
    out->gz = fe_bscan_be_to_s16(raw[12], raw[13]);

    return true;
}

static void fe_bscan_hit20_calib_reset_result(
    zy100_final_edge_bscan_result_t *result)
{
    uint32_t idx;

    if (result == NULL)
    {
        return;
    }

    for (idx = 0U; idx < ZY100_FINAL_EDGE_BSCAN_HIT20_CALIB_COUNT; idx++)
    {
        result->hit20_calib_first_cross_idx[idx] = -1;
        result->hit20_calib_first_cross_score[idx] = 0U;
        result->hit20_calib_first_cross_tmst[idx] = 0U;
    }
}

#if ZY100_FINAL_EDGE_HIT20_CALIB_ENABLE
static void fe_bscan_hit20_calib_note_score(
    zy100_final_edge_bscan_result_t *result,
    uint32_t seq,
    uint16_t tmst_raw,
    uint32_t score20)
{
    uint32_t idx;

    if (result == NULL)
    {
        return;
    }

    for (idx = 0U; idx < ZY100_FINAL_EDGE_BSCAN_HIT20_CALIB_COUNT; idx++)
    {
        if ((result->hit20_calib_first_cross_idx[idx] < 0) &&
            (score20 >= s_fe_bscan_hit20_calib_thresholds[idx]))
        {
            result->hit20_calib_first_cross_idx[idx] = (int32_t)seq;
            result->hit20_calib_first_cross_score[idx] = score20;
            result->hit20_calib_first_cross_tmst[idx] = tmst_raw;
        }
    }
}
#endif

static void fe_bscan_update_hit_window(
    zy100_final_edge_bscan_result_t *result)
{
    uint32_t hit_seq;

    if ((result == NULL) || (result->hit_found == 0U))
    {
        return;
    }

    hit_seq = result->hit_index;
    result->hit_window_valid = 1U;
    result->target_window_frames = ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES;
    result->hf_end_seq = hit_seq + ZY100_FINAL_EDGE_HIT_POST_FRAMES;
    if (hit_seq >= ZY100_FINAL_EDGE_HIT_PRE_FRAMES)
    {
        result->hf_start_seq = hit_seq - ZY100_FINAL_EDGE_HIT_PRE_FRAMES;
        result->lf_pre_needed_frames = 0U;
    }
    else
    {
        result->hf_start_seq = 0U;
        result->lf_pre_needed_frames =
            ZY100_FINAL_EDGE_HIT_PRE_FRAMES - hit_seq;
    }
    result->hf_frame_count = result->hf_end_seq - result->hf_start_seq + 1U;
    result->lf_pre_packets = (result->lf_pre_needed_frames + 7U) / 8U;
    result->ring_start_idx =
        result->hf_start_seq % ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES;
}

static uint32_t fe_bscan_window_xor_update(uint32_t value,
                                           const uint8_t *frame)
{
    uint32_t idx;

    for (idx = 0U; idx < ICM53611_OIS_RAW_FRAME_BYTES; idx++)
    {
        value ^= ((uint32_t)frame[idx]) << ((idx & 3U) * 8U);
    }
    return value;
}

bool zy100_final_edge_bscan_hit_window_get_frame(
    const uint8_t *raw_ring,
    uint32_t raw_ring_bytes,
    const zy100_final_edge_hit_window_view_t *view,
    uint32_t offset,
    const uint8_t **frame_out)
{
    uint32_t ring_bytes =
        ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES * ICM53611_OIS_RAW_FRAME_BYTES;
    uint32_t ring_index;

    if ((raw_ring == NULL) || (view == NULL) || (frame_out == NULL) ||
        (raw_ring_bytes < ring_bytes))
    {
        return false;
    }
    *frame_out = NULL;
    if ((view->valid == 0U) || (view->complete == 0U) ||
        (offset >= view->hf_frames))
    {
        return false;
    }

    ring_index =
        (view->start_seq + offset) % ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES;
    *frame_out = &raw_ring[ring_index * ICM53611_OIS_RAW_FRAME_BYTES];
    return true;
}

bool zy100_final_edge_bscan_extract_hit_window_view(
    const uint8_t *raw_ring,
    uint32_t raw_ring_bytes,
    const zy100_final_edge_bscan_result_t *result,
    uint32_t bscan_id,
    zy100_final_edge_hit_window_view_t *view)
{
    const uint32_t ring_bytes =
        ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES * ICM53611_OIS_RAW_FRAME_BYTES;
    uint32_t hit_seq;
    uint32_t oldest_seq;
    uint32_t offset;

    if (view == NULL)
    {
        return false;
    }
    memset(view, 0, sizeof(*view));
    view->bscan_id = bscan_id;

    if ((raw_ring == NULL) || (raw_ring_bytes < ring_bytes) ||
        (result == NULL) || (result->hit_found == 0U))
    {
        return false;
    }

    hit_seq = result->hit_index;
    view->valid = 1U;
    view->hit_seq = hit_seq;
    view->end_seq = hit_seq + ZY100_FINAL_EDGE_HIT_POST_FRAMES;
    if (hit_seq >= ZY100_FINAL_EDGE_HIT_PRE_FRAMES)
    {
        view->start_seq = hit_seq - ZY100_FINAL_EDGE_HIT_PRE_FRAMES;
        view->hf_pre_frames = ZY100_FINAL_EDGE_HIT_PRE_FRAMES;
        view->lf_pre_frames_needed = 0U;
        view->early_hit = 0U;
    }
    else
    {
        view->start_seq = 0U;
        view->hf_pre_frames = hit_seq;
        view->lf_pre_frames_needed =
            ZY100_FINAL_EDGE_HIT_PRE_FRAMES - hit_seq;
        view->early_hit = 1U;
    }
    view->hf_post_frames = ZY100_FINAL_EDGE_HIT_POST_FRAMES;
    view->hf_frames = view->end_seq - view->start_seq + 1U;
    view->lf_pre_packets_needed =
        (view->lf_pre_frames_needed + 7U) / 8U;
    view->ring_start_index =
        view->start_seq % ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES;
    view->wraps =
        ((view->ring_start_index + view->hf_frames) >
         ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES) ? 1U : 0U;

    if ((result->ring_valid_frames == 0U) ||
        (result->last_saved_seq + 1U < result->ring_valid_frames))
    {
        return true;
    }

    oldest_seq = result->last_saved_seq - result->ring_valid_frames + 1U;
    if ((result->last_saved_seq >= view->end_seq) &&
        (result->post_truncated == 0U) &&
        (result->ring_valid_frames >= view->hf_frames) &&
        (view->start_seq >= oldest_seq))
    {
        const uint8_t *frame;

        view->complete = 1U;
        frame = &raw_ring[(view->start_seq %
                          ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES) *
                         ICM53611_OIS_RAW_FRAME_BYTES];
        view->first_tmst_raw = fe_bscan_ois_raw_frame_tmst_raw(frame);
        frame = &raw_ring[(hit_seq % ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES) *
                         ICM53611_OIS_RAW_FRAME_BYTES];
        view->hit_tmst_raw = fe_bscan_ois_raw_frame_tmst_raw(frame);
        frame = &raw_ring[(view->end_seq %
                          ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES) *
                         ICM53611_OIS_RAW_FRAME_BYTES];
        view->last_tmst_raw = fe_bscan_ois_raw_frame_tmst_raw(frame);

        for (offset = 0U; offset < view->hf_frames; offset++)
        {
            if (zy100_final_edge_bscan_hit_window_get_frame(
                    raw_ring, raw_ring_bytes, view, offset, &frame))
            {
                view->window_xor =
                    fe_bscan_window_xor_update(view->window_xor, frame);
            }
        }
    }

    return true;
}

static void fe_bscan_note_frame(
    zy100_final_edge_bscan_result_t *result,
    const uint8_t raw[ICM53611_OIS_RAW_FRAME_BYTES],
    uint32_t seq,
    zy100_final_edge_bscan_hit_state_t *hit_state,
    bool lightweight)
{
    uint16_t tmst_raw;
    int16_t ax16 = 0;
    int16_t ay16 = 0;
    int16_t az16 = 0;
    int32_t ax20 = 0;
    int32_t ay20 = 0;
    int32_t az20 = 0;

    if ((result == NULL) || (raw == NULL))
    {
        return;
    }

    tmst_raw = fe_bscan_ois_raw_frame_tmst_raw(raw);
    if (result->frames == 0U)
    {
        result->first_tmst_raw = tmst_raw;
        result->first_tmst_valid = true;
    }
    else
    {
        uint16_t dt_us = fe_bscan_timestamp_delta16_us(
            result->last_tmst_raw,
            tmst_raw);
        bool dt_bad =
            ((dt_us < ZY100_FINAL_EDGE_OIS_DT_MIN_US) ||
             (dt_us > ZY100_FINAL_EDGE_OIS_DT_MAX_US));

        result->duration_acc_us += dt_us;
        if (dt_us == 0U)
        {
            result->stale++;
        }
        if (dt_bad)
        {
            result->dt_bad++;
        }
        fe_bscan_diag_note_tmst_delta(result, seq, dt_us, dt_bad);
    }

    result->last_tmst_raw = tmst_raw;
    result->last_tmst_valid = true;
    result->last_saved_seq = seq;
    result->current_saved_seq = result->last_saved_seq;
    result->frames++;
    result->frames_seen = result->frames;
    result->ring_valid_frames =
        (result->frames > ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES) ?
        ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES : result->frames;
    /*
     * Logical captured byte count. The final_edge high raw backing store is a
     * hit-window circular ring, not a linear frames*19 SRAM buffer.
     */
    result->bytes = result->frames * ICM53611_OIS_RAW_FRAME_BYTES;

    fe_bscan_ds_store_frame(raw, seq, tmst_raw);

    if (hit_state == NULL)
    {
        return;
    }

    if (icm53611_decode_ois_raw_accel16(raw, &ax16, &ay16, &az16) ==
        IMU_STATUS_OK)
    {
        if (hit_state->prev16_valid)
        {
            int32_t dx16 = (int32_t)ax16 - (int32_t)hit_state->prev16_ax;
            int32_t dy16 = (int32_t)ay16 - (int32_t)hit_state->prev16_ay;
            int32_t dz16 = (int32_t)az16 - (int32_t)hit_state->prev16_az;
            uint32_t score16 = fe_bscan_abs_i32(dx16) +
                               fe_bscan_abs_i32(dy16) +
                               fe_bscan_abs_i32(dz16);

            if (score16 > result->peak16_score)
            {
                result->peak16_score = score16;
                result->peak16_index = seq;
            }
        }
        hit_state->prev16_ax = ax16;
        hit_state->prev16_ay = ay16;
        hit_state->prev16_az = az16;
        hit_state->prev16_valid = true;
    }

    if (!icm53611_decode_ois20_accel_raw(raw, &ax20, &ay20, &az20))
    {
        result->ois20_decode_err++;
        return;
    }

    result->use20 = 1U;
    if (hit_state->prev20_valid)
    {
        int32_t dx20 = ax20 - hit_state->prev20_ax;
        int32_t dy20 = ay20 - hit_state->prev20_ay;
        int32_t dz20 = az20 - hit_state->prev20_az;
        uint32_t score20 = fe_bscan_abs_i32(dx20) +
                           fe_bscan_abs_i32(dy20) +
                           fe_bscan_abs_i32(dz20);

#if ZY100_FINAL_EDGE_HIT20_CALIB_ENABLE
        if (!lightweight)
        {
            fe_bscan_hit20_calib_note_score(result, seq, tmst_raw, score20);
        }
#endif
        if (score20 > result->peak20_score)
        {
            result->peak20_score = score20;
            result->peak20_index = seq;
            result->peak_score = score20;
            result->peak_index = seq;
        }
        if ((result->hit_found == 0U) &&
            (score20 >= ZY100_FINAL_EDGE_HIT_ACC20_DELTA_L1_THRESHOLD))
        {
            result->hit_found = 1U;
            result->hit_index = seq;
            result->hit_score = score20;
            result->hit_score20 = score20;
            result->hit_tmst_raw = tmst_raw;
            result->target_end_seq = seq + ZY100_FINAL_EDGE_HIT_POST_FRAMES;
            fe_bscan_update_hit_window(result);
        }
    }

    hit_state->prev20_ax = ax20;
    hit_state->prev20_ay = ay20;
    hit_state->prev20_az = az20;
    hit_state->prev20_valid = true;
}

#if ZY100_FINAL_EDGE_B_HARD_RT_ACQ_ENABLE
static void fe_bscan_refresh_acq_stats(
    zy100_final_edge_bscan_result_t *result)
{
    zy100_fe_b_acq_stats_t stats;

    if (result == NULL)
    {
        return;
    }

    zy100_ois_aba_v0_final_edge_bridge_get_acq_stats(&stats);
    result->timer_irq_total = stats.irq_total;
    result->acq_overflow = stats.overflow;
    result->acq_read_err = stats.read_err;
    result->acq_raw_read_err = stats.raw_read_err;
    result->acq_irq_late_count = stats.irq_late_count;
    result->acq_irq_early_count = stats.irq_early_count;
    result->acq_irq_late_gap_count = stats.irq_late_gap_count;
    result->acq_irq_gap_us_min = stats.irq_gap_us_min;
    result->acq_irq_gap_us_max = stats.irq_gap_us_max;
    result->acq_read_us_max = stats.read_us_max;
    result->acq_high_water = stats.high_water;
    result->acq_first_irq_us = stats.first_irq_us;
    result->acq_first_read_done_us = stats.first_read_done_us;
    result->acq_wr_seq = stats.wr_seq;
    result->acq_rd_seq = stats.rd_seq;
    result->acq_skipped_after_stop = stats.skipped_after_stop;
    result->acq_phase_sync_status = stats.phase_sync_status;
    result->acq_phase_sync_probes = stats.phase_sync_probes;
    result->acq_phase_sync_seed_tmst = stats.phase_sync_seed_tmst;
    result->acq_phase_sync_lock_tmst = stats.phase_sync_lock_tmst;
    result->acq_phase_sync_delta = stats.phase_sync_delta;
    result->acq_phase_sync_us = stats.phase_sync_us;
    result->acq_phase_sync_read_us_max = stats.phase_sync_read_us_max;
    result->acq_begin_fail_stage = stats.begin_fail_stage;
    result->acq_begin_fail_status = stats.begin_fail_status;
    result->acq_phase_sync_no_change_count =
        stats.phase_sync_no_change_count;
    result->acq_phase_sync_soft_start_count =
        stats.phase_sync_soft_start_count;
    result->acq_tmst_gate_enabled = stats.tmst_gate_enabled;
    result->acq_tmst_poll_hz = stats.tmst_poll_hz;
    result->acq_tmst_new_count = stats.tmst_new_count;
    result->acq_tmst_dup_drop = stats.tmst_dup_drop;
    result->acq_tmst_read_err = stats.tmst_read_err;
    result->acq_tmst_raw_mismatch = stats.tmst_raw_mismatch;
    result->acq_tmst_mismatch_last_seq = stats.tmst_mismatch_last_seq;
    result->acq_tmst_mismatch_probe_tmst =
        stats.tmst_mismatch_probe_tmst;
    result->acq_tmst_mismatch_raw_tmst = stats.tmst_mismatch_raw_tmst;
    result->acq_tmst_startup_mismatch_drop =
        stats.tmst_startup_mismatch_drop;
    result->acq_tmst_startup_mismatch_last_probe_tmst =
        stats.tmst_startup_mismatch_last_probe_tmst;
    result->acq_tmst_startup_mismatch_last_raw_tmst =
        stats.tmst_startup_mismatch_last_raw_tmst;
    result->acq_tmst_startup_mismatch_hard_limit =
        stats.tmst_startup_mismatch_hard_limit;
    result->acq_tmst_read_us_max = stats.tmst_read_us_max;
    result->acq_raw_read_us_max = stats.raw_read_us_max;
    result->acq_raw_phase_dt_bad = stats.raw_phase_dt_bad;
    result->acq_raw_phase_delta_min = stats.raw_phase_delta_min;
    result->acq_raw_phase_delta_max = stats.raw_phase_delta_max;
    if (stats.raw_phase_dt_bad > result->dt_bad)
    {
        result->dt_bad = stats.raw_phase_dt_bad;
    }
    if (stats.read_err > result->read_err)
    {
        result->read_err = stats.read_err;
    }
    if (stats.overflow > result->over)
    {
        result->over = stats.overflow;
    }
    if (stats.overflow > result->drop)
    {
        result->drop = stats.overflow;
    }
    if (stats.read_us_max > result->diag.ois_read_us_max)
    {
        result->diag.ois_read_us_max = stats.read_us_max;
    }
    if ((result->first_frame_read_done_us == 0U) &&
        (stats.first_read_done_us != 0U))
    {
        result->first_frame_read_done_us = stats.first_read_done_us;
    }
}

#if ZY100_FINAL_EDGE_B_ACQ_REUSE_HIT_RING_ENABLE
static bool fe_bscan_acq_backlog_hard(
    const zy100_final_edge_bscan_result_t *result)
{
    return (result != NULL) &&
           (result->acq_high_water >=
            ZY100_FINAL_EDGE_B_ACQ_BACKLOG_HARD_FRAMES);
}
#endif

static bool fe_bscan_acq_stats_have_hard_fault(
    const zy100_final_edge_bscan_result_t *result)
{
    bool hard;

    if (result == NULL)
    {
        return false;
    }

    hard = ((result->acq_overflow != 0U) ||
            (result->acq_read_err != 0U) ||
            (result->acq_tmst_raw_mismatch != 0U) ||
            (result->acq_raw_phase_dt_bad != 0U)
#if (!ZY100_FINAL_EDGE_B_ACQ_TMST_GATE_ENABLE && \
     !ZY100_FINAL_EDGE_B_ACQ_RAW_PHASE_LOCK_ENABLE)
            || (result->acq_irq_late_count != 0U)
#endif
#if ZY100_FINAL_EDGE_B_ACQ_REUSE_HIT_RING_ENABLE
            || fe_bscan_acq_backlog_hard(result)
#endif
           );
    return hard;
}
#endif

static bool zy100_final_edge_bscan_run_impl(
    uint8_t *buffer,
    uint32_t buffer_bytes,
    zy100_final_edge_bscan_result_t *result)
{
    const uint32_t ring_bytes =
        ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES * ICM53611_OIS_RAW_FRAME_BYTES;
    zy100_final_edge_bscan_hit_state_t hit_state = { 0 };
    uint32_t start_ms = 0U;
    uint32_t idle_spin = 0U;
    bool cleanup_ok = false;
    bool bridge_started = false;
    bool bridge_begin_ok;
#if ZY100_FINAL_EDGE_B_ACQ_REUSE_HIT_RING_ENABLE
    bool stop_after_set = false;
#endif

    if (result != NULL)
    {
        memset(result, 0, sizeof(*result));
        fe_bscan_diag_reset_result(result);
        fe_bscan_hit20_calib_reset_result(result);
    }
    fe_bscan_ds_reset();
    if ((buffer == NULL) || (result == NULL) || (buffer_bytes < ring_bytes))
    {
        if (result != NULL)
        {
            result->err = 1U;
            fe_bscan_note_exit(result,
                               ZY100_FINAL_EDGE_BSCAN_EXIT_BEGIN_FAIL,
                               ZY100_FINAL_EDGE_BSCAN_SEQ32_INVALID,
                               0U,
                               0U);
        }
        return false;
    }

    bridge_begin_ok =
        zy100_ois_aba_v0_final_edge_bridge_begin_bscan(result,
                                                       buffer,
                                                       buffer_bytes);
    if (!bridge_begin_ok)
    {
        if (result->begin_retry != 0U)
        {
            fe_bscan_note_exit(result,
                               ZY100_FINAL_EDGE_BSCAN_EXIT_BEGIN_FAIL,
                               ZY100_FINAL_EDGE_BSCAN_SEQ32_INVALID,
                               0U,
                               0U);
            return false;
        }
        if ((result->owner_ok == 0U) && (result->cleanup_ok == 0U) &&
            (result->read_err == 0U))
        {
            fe_bscan_note_exit(result,
                               ZY100_FINAL_EDGE_BSCAN_EXIT_BEGIN_FAIL,
                               ZY100_FINAL_EDGE_BSCAN_SEQ32_INVALID,
                               0U,
                               0U);
            return false;
        }
        fe_bscan_note_exit(result,
                           ZY100_FINAL_EDGE_BSCAN_EXIT_BEGIN_FAIL,
                           ZY100_FINAL_EDGE_BSCAN_SEQ32_INVALID,
                           0U,
                           0U);
        cleanup_ok = (result->cleanup_ok != 0U);
        goto finalize;
    }
    bridge_started = true;

    start_ms = zy100_os_time_ms();
    while (result->frames < ZY100_FINAL_EDGE_B_SCAN_MAX_FRAMES)
    {
#if ZY100_FINAL_EDGE_B_HARD_RT_ACQ_ENABLE
        uint32_t slot_seq = 0U;
        const uint8_t *src = NULL;
        bool stop_now = false;
        zy100_fe_b_acq_consume_status_t acq_status;

        acq_status =
            zy100_ois_aba_v0_final_edge_bridge_consume_acq_frame(
                &slot_seq,
                &src);
        if (acq_status == ZY100_FE_B_ACQ_CONSUME_EMPTY)
        {
            fe_bscan_refresh_acq_stats(result);
            if (fe_bscan_acq_stats_have_hard_fault(result))
            {
                fe_bscan_note_exit(
                    result,
                    ZY100_FINAL_EDGE_BSCAN_EXIT_ACQ_HARD_FAULT,
                    fe_bscan_last_seq_or_invalid(result),
                    start_ms,
                    idle_spin);
                break;
            }
            idle_spin++;
            if (((idle_spin & 0xFFU) == 0U) &&
                (fe_bscan_elapsed_ms(start_ms) >=
                 (ZY100_FINAL_EDGE_B_SCAN_MS +
                  ZY100_FINAL_EDGE_BSCAN_GUARD_MS)))
            {
                result->err++;
                fe_bscan_note_exit(
                    result,
                    ZY100_FINAL_EDGE_BSCAN_EXIT_ACQ_EMPTY_TIMEOUT,
                    fe_bscan_last_seq_or_invalid(result),
                    start_ms,
                    idle_spin);
                break;
            }
            continue;
        }

        idle_spin = 0U;
        fe_bscan_refresh_acq_stats(result);
        if (acq_status == ZY100_FE_B_ACQ_CONSUME_READ_ERR)
        {
            fe_bscan_note_exit(result,
                               ZY100_FINAL_EDGE_BSCAN_EXIT_ACQ_READ_ERR,
                               fe_bscan_last_seq_or_invalid(result),
                               start_ms,
                               idle_spin);
            break;
        }
        if (src == NULL)
        {
            result->read_err++;
            fe_bscan_note_exit(result,
                               ZY100_FINAL_EDGE_BSCAN_EXIT_NULL_FRAME,
                               fe_bscan_last_seq_or_invalid(result),
                               start_ms,
                               idle_spin);
            break;
        }

#if ZY100_FINAL_EDGE_B_ACQ_RING_ENABLE
        {
            uint8_t *dst =
                &buffer[(slot_seq % ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES) *
                        ICM53611_OIS_RAW_FRAME_BYTES];
            memcpy(dst, src, ICM53611_OIS_RAW_FRAME_BYTES);
            src = dst;
        }
#endif

        fe_bscan_note_frame(result, src, slot_seq, &hit_state, true);

#if ZY100_FINAL_EDGE_B_ACQ_REUSE_HIT_RING_ENABLE
        if ((result->hit_found != 0U) && !stop_after_set)
        {
            zy100_ois_aba_v0_final_edge_bridge_acq_set_stop_after(
                result->target_end_seq);
            stop_after_set = true;
        }
#endif

        fe_bscan_refresh_acq_stats(result);
        if (fe_bscan_acq_stats_have_hard_fault(result))
        {
            fe_bscan_note_exit(
                result,
                ZY100_FINAL_EDGE_BSCAN_EXIT_ACQ_HARD_FAULT,
                fe_bscan_last_seq_or_invalid(result),
                start_ms,
                idle_spin);
            break;
        }
        if (result->dt_bad != 0U)
        {
            fe_bscan_note_exit(result,
                               ZY100_FINAL_EDGE_BSCAN_EXIT_DT_BAD,
                               fe_bscan_last_seq_or_invalid(result),
                               start_ms,
                               idle_spin);
            break;
        }
        if (result->stale != 0U)
        {
            fe_bscan_note_exit(result,
                               ZY100_FINAL_EDGE_BSCAN_EXIT_STALE,
                               fe_bscan_last_seq_or_invalid(result),
                               start_ms,
                               idle_spin);
            break;
        }
        if (result->hit_found != 0U)
        {
            if (result->last_saved_seq >= result->target_end_seq)
            {
                stop_now = true;
                fe_bscan_note_exit(
                    result,
                    ZY100_FINAL_EDGE_BSCAN_EXIT_HIT_WINDOW_DONE,
                    fe_bscan_last_seq_or_invalid(result),
                    start_ms,
                    idle_spin);
            }
        }
        else if (result->frames >= ZY100_FINAL_EDGE_B_SCAN_BASE_FRAMES)
        {
            stop_now = true;
            fe_bscan_note_exit(
                result,
                ZY100_FINAL_EDGE_BSCAN_EXIT_NOHIT_BASE_DONE,
                fe_bscan_last_seq_or_invalid(result),
                start_ms,
                idle_spin);
        }

        if (stop_now)
        {
            break;
        }
#else
        uint32_t pending =
            zy100_ois_aba_v0_final_edge_bridge_take_pending_ticks();
        uint32_t seq;
        uint8_t *dst;
        bool stop_now = false;
#if ZY100_FINAL_EDGE_BSCAN_OD_DIAG_ENABLE
        uint64_t read_start_us;
        uint32_t read_elapsed_us;
#endif

        if (pending == 0U)
        {
            idle_spin++;
            if (((idle_spin & 0xFFU) == 0U) &&
                (fe_bscan_elapsed_ms(start_ms) >=
                 (ZY100_FINAL_EDGE_B_SCAN_MS +
                  ZY100_FINAL_EDGE_BSCAN_GUARD_MS)))
            {
                result->err++;
                fe_bscan_note_exit(
                    result,
                    ZY100_FINAL_EDGE_BSCAN_EXIT_ACQ_EMPTY_TIMEOUT,
                    fe_bscan_last_seq_or_invalid(result),
                    start_ms,
                    idle_spin);
                break;
            }
            continue;
        }
        idle_spin = 0U;

        seq = result->frames;
        if (pending > 1U)
        {
            fe_bscan_diag_note_od_event(result, seq, pending);
            result->over++;
            result->drop += (pending - 1U);
        }

        dst = &buffer[(seq % ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES) *
                      ICM53611_OIS_RAW_FRAME_BYTES];
#if ZY100_FINAL_EDGE_BSCAN_OD_DIAG_ENABLE
        read_start_us = imu_bsp_local_timestamp_us();
#endif
        if (!zy100_ois_aba_v0_final_edge_bridge_read_ois_frame(dst))
        {
#if ZY100_FINAL_EDGE_BSCAN_OD_DIAG_ENABLE
            read_elapsed_us =
                (uint32_t)(imu_bsp_local_timestamp_us() - read_start_us);
            fe_bscan_diag_note_read_us(result, seq, read_elapsed_us);
#endif
            result->read_err++;
            continue;
        }
#if ZY100_FINAL_EDGE_BSCAN_OD_DIAG_ENABLE
        read_elapsed_us =
            (uint32_t)(imu_bsp_local_timestamp_us() - read_start_us);
        fe_bscan_diag_note_read_us(result, seq, read_elapsed_us);
#endif

        fe_bscan_note_frame(result, dst, seq, &hit_state, false);

        if (result->hit_found != 0U)
        {
            if (result->last_saved_seq >= result->target_end_seq)
            {
                stop_now = true;
                fe_bscan_note_exit(
                    result,
                    ZY100_FINAL_EDGE_BSCAN_EXIT_HIT_WINDOW_DONE,
                    fe_bscan_last_seq_or_invalid(result),
                    start_ms,
                    idle_spin);
            }
        }
        else if (result->frames >= ZY100_FINAL_EDGE_B_SCAN_BASE_FRAMES)
        {
            stop_now = true;
            fe_bscan_note_exit(
                result,
                ZY100_FINAL_EDGE_BSCAN_EXIT_NOHIT_BASE_DONE,
                fe_bscan_last_seq_or_invalid(result),
                start_ms,
                idle_spin);
        }

        if (stop_now)
        {
            break;
        }
#endif
    }
    if ((result->hit_found != 0U) &&
        (result->last_saved_seq < result->target_end_seq) &&
        (result->frames >= ZY100_FINAL_EDGE_B_SCAN_MAX_FRAMES))
    {
        result->post_truncated = 1U;
    }
    if (result->frames >= ZY100_FINAL_EDGE_B_SCAN_MAX_FRAMES)
    {
        fe_bscan_note_exit_if_none(
            result,
            ZY100_FINAL_EDGE_BSCAN_EXIT_MAX_FRAMES,
            fe_bscan_last_seq_or_invalid(result),
            start_ms,
            idle_spin);
    }

    cleanup_ok = zy100_ois_aba_v0_final_edge_bridge_end_bscan(result);
    if (!cleanup_ok)
    {
        fe_bscan_note_exit(result,
                           ZY100_FINAL_EDGE_BSCAN_EXIT_CLEANUP_FAIL,
                           fe_bscan_last_seq_or_invalid(result),
                           start_ms,
                           idle_spin);
    }

finalize:
    if (result->hit_found == 0U)
    {
        result->hit_window_valid = 0U;
        result->hf_frame_count = 0U;
        result->lf_pre_needed_frames = 0U;
        result->lf_pre_packets = 0U;
        if (result->frames != ZY100_FINAL_EDGE_B_SCAN_BASE_FRAMES)
        {
            result->err++;
            fe_bscan_note_exit_if_none(
                result,
                ZY100_FINAL_EDGE_BSCAN_EXIT_MAX_FRAMES,
                fe_bscan_last_seq_or_invalid(result),
                start_ms,
                idle_spin);
        }
    }
    else
    {
        fe_bscan_update_hit_window(result);
    }
    result->err += result->read_err;
    result->err += result->over;
    result->err += result->drop;
#if ZY100_FINAL_EDGE_B_HARD_RT_ACQ_ENABLE
    result->err += result->stale;
    result->err += result->dt_bad;
    if (result->acq_read_err > result->read_err)
    {
        result->err += (result->acq_read_err - result->read_err);
    }
    if (result->acq_overflow > result->over)
    {
        result->err += (result->acq_overflow - result->over);
    }
#if (!ZY100_FINAL_EDGE_B_ACQ_TMST_GATE_ENABLE && \
     !ZY100_FINAL_EDGE_B_ACQ_RAW_PHASE_LOCK_ENABLE)
    result->err += result->acq_irq_late_count;
#endif
    result->err += result->acq_tmst_raw_mismatch;
#if ZY100_FINAL_EDGE_B_ACQ_REUSE_HIT_RING_ENABLE
    if (fe_bscan_acq_backlog_hard(result))
    {
        result->err++;
    }
#endif
#endif
    if (!cleanup_ok)
    {
        result->err++;
    }

    if (!bridge_started && (result->cleanup_ok == 0U))
    {
        cleanup_ok = false;
    }
    return ((result->owner_ok != 0U) && cleanup_ok && (result->err == 0U));
}

bool zy100_final_edge_bscan_run(
    uint8_t *buffer,
    uint32_t buffer_bytes,
    zy100_final_edge_bscan_result_t *result)
{
    return zy100_final_edge_bscan_run_impl(buffer, buffer_bytes, result);
}

uint32_t zy100_final_edge_bscan_get_ds_count(void)
{
    return s_fe_bscan_ds_ctx.ds_count;
}

uint32_t zy100_final_edge_bscan_get_ds_overflow(void)
{
    return s_fe_bscan_ds_ctx.ds_overflow;
}

bool zy100_final_edge_bscan_get_ds_sample(
    uint32_t index,
    zy100_final_edge_bscan_ds_sample_t *out)
{
    if ((out == NULL) || (index >= s_fe_bscan_ds_ctx.ds_count))
    {
        return false;
    }

    return fe_bscan_replay_raw_to_ds_sample(
        &s_fe_bscan_ds_ctx.raw[index],
        index,
        out);
}

bool zy100_final_edge_bscan_get_replay_raw_stats(
    zy100_final_edge_bscan_replay_raw_stats_t *out)
{
    if (out == NULL)
    {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->enabled = 1U;
    out->decim = ZY100_FINAL_EDGE_OIS_DECIM;
    out->count = s_fe_bscan_ds_ctx.ds_count;
    out->overflow = s_fe_bscan_ds_ctx.ds_overflow;
    out->store_count = s_fe_bscan_ds_ctx.ds_store_count;
    out->first_frame_index = 0U;
    out->last_frame_index =
        (s_fe_bscan_ds_ctx.ds_count == 0U) ?
        0U : s_fe_bscan_ds_ctx.ds_last_frame_index;
    out->first_tmst_raw =
        (s_fe_bscan_ds_ctx.ds_count == 0U) ?
        0U : s_fe_bscan_ds_ctx.ds_first_tmst_raw;
    out->last_tmst_raw =
        (s_fe_bscan_ds_ctx.ds_count == 0U) ?
        0U : s_fe_bscan_ds_ctx.ds_last_tmst_raw;
    out->bytes =
        s_fe_bscan_ds_ctx.ds_count * ICM53611_OIS_RAW_FRAME_BYTES;

    return true;
}

#else

bool zy100_final_edge_bscan_run(
    uint8_t *buffer,
    uint32_t buffer_bytes,
    zy100_final_edge_bscan_result_t *result)
{
    IMU_UNUSED(buffer);
    IMU_UNUSED(buffer_bytes);
    if (result != NULL)
    {
        memset(result, 0, sizeof(*result));
        result->diag.od_first_seq = ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
        result->diag.od_last_seq = ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
        result->diag.ois_read_us_max_seq =
            ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
        result->diag.dt_bad_first_seq =
            ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
        result->diag.dt_bad_last_seq =
            ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
        result->diag.tmst_delta_min = 0xFFFFU;
        result->diag.tmst_delta_first_bad_seq =
            ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
        result->diag.tmst_delta_last_bad_seq =
            ZY100_FINAL_EDGE_BSCAN_SEQ_INVALID;
        result->exit_reason = ZY100_FINAL_EDGE_BSCAN_EXIT_BEGIN_FAIL;
        result->exit_last_seq = ZY100_FINAL_EDGE_BSCAN_SEQ32_INVALID;
        result->acq_tmst_mismatch_last_seq =
            ZY100_FINAL_EDGE_BSCAN_SEQ32_INVALID;
        result->err = 1U;
    }
    return false;
}

bool zy100_final_edge_bscan_extract_hit_window_view(
    const uint8_t *raw_ring,
    uint32_t raw_ring_bytes,
    const zy100_final_edge_bscan_result_t *result,
    uint32_t bscan_id,
    zy100_final_edge_hit_window_view_t *view)
{
    IMU_UNUSED(raw_ring);
    IMU_UNUSED(raw_ring_bytes);
    IMU_UNUSED(result);
    IMU_UNUSED(bscan_id);
    if (view != NULL)
    {
        memset(view, 0, sizeof(*view));
    }
    return false;
}

bool zy100_final_edge_bscan_hit_window_get_frame(
    const uint8_t *raw_ring,
    uint32_t raw_ring_bytes,
    const zy100_final_edge_hit_window_view_t *view,
    uint32_t offset,
    const uint8_t **frame_out)
{
    IMU_UNUSED(raw_ring);
    IMU_UNUSED(raw_ring_bytes);
    IMU_UNUSED(view);
    IMU_UNUSED(offset);
    if (frame_out != NULL)
    {
        *frame_out = NULL;
    }
    return false;
}

uint32_t zy100_final_edge_bscan_get_ds_count(void)
{
    return 0U;
}

uint32_t zy100_final_edge_bscan_get_ds_overflow(void)
{
    return 0U;
}

bool zy100_final_edge_bscan_get_ds_sample(
    uint32_t index,
    zy100_final_edge_bscan_ds_sample_t *out)
{
    IMU_UNUSED(index);
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
    return false;
}

bool zy100_final_edge_bscan_get_replay_raw_stats(
    zy100_final_edge_bscan_replay_raw_stats_t *out)
{
    if (out == NULL)
    {
        return false;
    }

    memset(out, 0, sizeof(*out));
    return true;
}

#endif
