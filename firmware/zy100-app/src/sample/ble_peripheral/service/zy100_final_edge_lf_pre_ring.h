#ifndef ZY100_FINAL_EDGE_LF_PRE_RING_H
#define ZY100_FINAL_EDGE_LF_PRE_RING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../app_flags.h"

/* The LF entry array only; write/count/stats metadata is not borrowed. */
#define ZY100_FINAL_EDGE_LF_PRE_ONLINE_WORKSPACE_BYTES 9600UL

typedef struct
{
    uint8_t packet[ZY100_FINAL_EDGE_LF_PRE_PACKET_BYTES];
    uint32_t live_sample_index;
    uint16_t tmst_raw;
    uint16_t flags;
} zy100_final_edge_lf_pre_entry_t;

typedef struct
{
    uint32_t push_count;
    uint32_t wrap_count;
    uint32_t oldest_sample_index;
    uint32_t newest_sample_index;
    uint16_t oldest_tmst_raw;
    uint16_t newest_tmst_raw;
    uint32_t miss_count;
} zy100_final_edge_lf_pre_stats_t;

typedef struct
{
    uint8_t valid;
    uint32_t requested_count;
    uint32_t packets_available;
    uint32_t oldest_sample_index;
    uint32_t newest_sample_index;
    uint16_t oldest_tmst_raw;
    uint16_t newest_tmst_raw;
    uint32_t miss_count;
    uint32_t wrap_count;
} zy100_final_edge_lf_pre_query_t;

typedef struct
{
    uint32_t saved_sections;
    uint32_t gap_sections;
    uint32_t gap_count;
    uint16_t gap_max_delta_us;
    uint32_t first_bad_raw_id;
    uint16_t first_bad_packet_index;
    uint16_t first_bad_delta_us;
} zy100_final_edge_lf_pre_diag_t;

void zy100_final_edge_lf_pre_ring_reset(void);

/*
 * Claim is accepted only after the normal LF ring has been reset. The
 * returned static span remains valid until release; callers must serialize
 * online and offline capture modes. While claimed, existing LF APIs safely
 * reject work or return an empty snapshot. Release discards the span without
 * changing the static ring declaration, address, or size.
 */
bool zy100_final_edge_lf_pre_ring_online_workspace_claim(
    uint8_t **out_base,
    uint32_t *out_bytes);
bool zy100_final_edge_lf_pre_ring_online_workspace_release(void);
bool zy100_final_edge_lf_pre_ring_online_workspace_borrowed(void);
bool zy100_final_edge_lf_pre_ring_online_workspace_available(void);
void zy100_final_edge_lf_pre_ring_push(
    uint32_t sample_index,
    uint16_t tmst_raw,
    const uint8_t packet[ZY100_FINAL_EDGE_LF_PRE_PACKET_BYTES]);
bool zy100_final_edge_lf_pre_ring_query_ending_at(
    uint32_t anchor_sample_index,
    uint32_t count,
    zy100_final_edge_lf_pre_query_t *out_info);
bool zy100_final_edge_lf_pre_ring_copy_ending_at(
    uint32_t anchor_sample_index,
    uint32_t count,
    zy100_final_edge_lf_pre_entry_t *out_entries,
    uint32_t out_capacity,
    zy100_final_edge_lf_pre_query_t *out_info);
void zy100_final_edge_lf_pre_ring_get_stats(
    zy100_final_edge_lf_pre_stats_t *out_stats);
void zy100_final_edge_lf_pre_ring_diag_reset(void);
void zy100_final_edge_lf_pre_ring_diag_note_saved_section(
    uint32_t raw_id,
    uint32_t anchor_sample_index,
    uint32_t count);
void zy100_final_edge_lf_pre_ring_diag_get(
    zy100_final_edge_lf_pre_diag_t *out_diag);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_FINAL_EDGE_LF_PRE_RING_H */
