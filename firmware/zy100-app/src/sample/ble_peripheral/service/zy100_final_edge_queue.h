#ifndef ZY100_FINAL_EDGE_QUEUE_H
#define ZY100_FINAL_EDGE_QUEUE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../app_flags.h"
#include "zy100_edge_lite.h"
#include "zy100_edge_record_format.h"

#define ZY100_FINAL_EDGE_QUEUE_SOURCE_LIVE_800      (1UL << 0)
#define ZY100_FINAL_EDGE_QUEUE_SOURCE_REPLAY_NOHIT  (1UL << 1)
#define ZY100_FINAL_EDGE_QUEUE_SOURCE_REPLAY_TRUEHIT (1UL << 2)

/*
 * Online continuous-RAW may exclusively borrow the two queue payload arrays.
 * Queue ownership metadata is deliberately excluded from this span.
 */
#define ZY100_FINAL_EDGE_QUEUE_ONLINE_WORKSPACE_BYTES 20480UL

typedef struct
{
    uint32_t summary_enqueued;
    uint32_t summary_dropped;
    uint32_t summary_drop_oldest;
    uint32_t summary_drop_oversize;
    uint32_t summary_peak_depth;
    uint32_t summary_current_depth;

    uint32_t event_enqueued;
    uint32_t event_dropped;
    uint32_t event_drop_oldest;
    uint32_t event_drop_oversize;
    uint32_t event_peak_depth;
    uint32_t event_current_depth;

    uint32_t raw_req_suppressed;
    uint32_t raw_start_suppressed;

    uint32_t queue_reset_count;
    uint32_t queue_bytes_est;
    uint32_t summary_queue_bytes;
    uint32_t event_queue_bytes;
} zy100_final_edge_queue_stats_t;

typedef struct
{
    uint32_t seq;
    uint32_t edge_sample_index;
    uint32_t source_flags;
    uint16_t timestamp_raw;
    uint16_t payload_bytes;
    zy100_edge_summary_record_t record;
} zy100_final_edge_summary_copy_t;

typedef struct
{
    uint32_t seq;
    uint32_t edge_sample_index;
    uint32_t source_flags;
    uint16_t timestamp_raw;
    uint16_t payload_bytes;
    zy100_edge_event_record_t record;
} zy100_final_edge_event_copy_t;

void zy100_final_edge_queue_reset(void);

/*
 * Claim is accepted only after the normal offline queue reset. The returned
 * static span remains valid until release; callers must serialize the online
 * and offline capture modes. While claimed, the normal queue APIs safely
 * reject work or return an empty snapshot. Release discards the borrowed
 * bytes without changing the static object's address or size.
 */
bool zy100_final_edge_queue_online_workspace_claim(
    uint8_t **out_base,
    uint32_t *out_bytes);
bool zy100_final_edge_queue_online_workspace_release(void);
bool zy100_final_edge_queue_online_workspace_borrowed(void);
bool zy100_final_edge_queue_online_workspace_available(void);

bool zy100_final_edge_queue_push_summary_lite(
    const zy100_edge_lite_summary_t *summary,
    uint32_t edge_sample_index,
    uint16_t timestamp_raw);

bool zy100_final_edge_queue_push_summary_lite_ex(
    const zy100_edge_lite_summary_t *summary,
    uint32_t edge_sample_index,
    uint16_t timestamp_raw,
    uint32_t source_flags);

bool zy100_final_edge_queue_push_event_lite(
    const zy100_edge_lite_event_t *event,
    uint32_t edge_sample_index,
    uint16_t timestamp_raw);

bool zy100_final_edge_queue_push_event_lite_ex(
    const zy100_edge_lite_event_t *event,
    uint32_t edge_sample_index,
    uint16_t timestamp_raw,
    uint32_t source_flags);

bool zy100_final_edge_queue_pop_summary_copy(
    zy100_final_edge_summary_copy_t *out);

bool zy100_final_edge_queue_pop_event_copy(
    zy100_final_edge_event_copy_t *out);

void zy100_final_edge_queue_note_raw_req_suppressed(void);
void zy100_final_edge_queue_note_raw_start_suppressed(void);

void zy100_final_edge_queue_get_stats(zy100_final_edge_queue_stats_t *out);

uint16_t zy100_final_edge_queue_summary_count(void);
uint16_t zy100_final_edge_queue_event_count(void);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_FINAL_EDGE_QUEUE_H */
