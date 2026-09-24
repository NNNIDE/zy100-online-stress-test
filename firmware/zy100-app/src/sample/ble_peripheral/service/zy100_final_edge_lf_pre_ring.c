#include "zy100_final_edge_lf_pre_ring.h"

#include <string.h>

#define ZY100_FINAL_EDGE_LF_PRE_DIAG_RAW_INVALID 0xFFFFFFFFUL
#define ZY100_FINAL_EDGE_LF_PRE_DIAG_IDX_INVALID 0xFFFFU
#define ZY100_FINAL_EDGE_LF_PRE_DIAG_DELTA_MIN_US 1000U
#define ZY100_FINAL_EDGE_LF_PRE_DIAG_DELTA_MAX_US 1500U
#define ZY100_FINAL_EDGE_LF_PRE_BORROWED_COUNT 0xFFFFFFFFUL

#if (ZY100_FINAL_EDGE_LF_PRE_RING_ENABLE != 0U)

typedef char zy100_final_edge_lf_pre_packet_size_check[
    (ZY100_FINAL_EDGE_LF_PRE_PACKET_BYTES == 16U) ? 1 : -1];

static zy100_final_edge_lf_pre_entry_t
    s_lf_pre_ring[ZY100_FINAL_EDGE_LF_PRE_RING_PACKETS];
typedef char zy100_final_edge_lf_pre_online_workspace_size_check[
    (sizeof(s_lf_pre_ring) ==
     ZY100_FINAL_EDGE_LF_PRE_ONLINE_WORKSPACE_BYTES) ? 1 : -1];
static zy100_final_edge_lf_pre_stats_t s_lf_pre_stats;
static uint32_t s_lf_pre_write_pos;
static uint32_t s_lf_pre_count;
#if ZY100_FINAL_EDGE_LF_PRE_DIAG_ENABLE
static zy100_final_edge_lf_pre_diag_t s_lf_pre_diag;
#endif

#if ZY100_FINAL_EDGE_LF_PRE_DIAG_ENABLE
static void lf_pre_diag_reset_state(void)
{
    memset(&s_lf_pre_diag, 0, sizeof(s_lf_pre_diag));
    s_lf_pre_diag.first_bad_raw_id =
        ZY100_FINAL_EDGE_LF_PRE_DIAG_RAW_INVALID;
    s_lf_pre_diag.first_bad_packet_index =
        ZY100_FINAL_EDGE_LF_PRE_DIAG_IDX_INVALID;
}
#endif

static uint32_t lf_pre_advance(uint32_t pos)
{
    pos++;
    if (pos >= ZY100_FINAL_EDGE_LF_PRE_RING_PACKETS)
    {
        pos = 0U;
    }
    return pos;
}

static uint32_t lf_pre_oldest_pos(void)
{
    return (s_lf_pre_write_pos +
            ZY100_FINAL_EDGE_LF_PRE_RING_PACKETS -
            s_lf_pre_count) % ZY100_FINAL_EDGE_LF_PRE_RING_PACKETS;
}

static const zy100_final_edge_lf_pre_entry_t *lf_pre_find_entry(
    uint32_t sample_index)
{
    uint32_t pos;
    uint32_t idx;

    if (s_lf_pre_count == 0U)
    {
        return NULL;
    }

    pos = lf_pre_oldest_pos();
    for (idx = 0U; idx < s_lf_pre_count; idx++)
    {
        const zy100_final_edge_lf_pre_entry_t *entry = &s_lf_pre_ring[pos];

        if (entry->live_sample_index == sample_index)
        {
            return entry;
        }
        pos = lf_pre_advance(pos);
    }

    return NULL;
}

static void lf_pre_refresh_bounds(void)
{
    uint32_t oldest_pos;
    uint32_t newest_pos;

    if (s_lf_pre_count == 0U)
    {
        s_lf_pre_stats.oldest_sample_index = 0U;
        s_lf_pre_stats.newest_sample_index = 0U;
        s_lf_pre_stats.oldest_tmst_raw = 0U;
        s_lf_pre_stats.newest_tmst_raw = 0U;
        return;
    }

    oldest_pos = lf_pre_oldest_pos();
    newest_pos = (s_lf_pre_write_pos == 0U) ?
                 (ZY100_FINAL_EDGE_LF_PRE_RING_PACKETS - 1U) :
                 (s_lf_pre_write_pos - 1U);
    s_lf_pre_stats.oldest_sample_index =
        s_lf_pre_ring[oldest_pos].live_sample_index;
    s_lf_pre_stats.newest_sample_index =
        s_lf_pre_ring[newest_pos].live_sample_index;
    s_lf_pre_stats.oldest_tmst_raw = s_lf_pre_ring[oldest_pos].tmst_raw;
    s_lf_pre_stats.newest_tmst_raw = s_lf_pre_ring[newest_pos].tmst_raw;
}

void zy100_final_edge_lf_pre_ring_reset(void)
{
    if (zy100_final_edge_lf_pre_ring_online_workspace_borrowed())
    {
        return;
    }

    memset(s_lf_pre_ring, 0, sizeof(s_lf_pre_ring));
    memset(&s_lf_pre_stats, 0, sizeof(s_lf_pre_stats));
#if ZY100_FINAL_EDGE_LF_PRE_DIAG_ENABLE
    lf_pre_diag_reset_state();
#endif
    s_lf_pre_write_pos = 0U;
    s_lf_pre_count = 0U;
}

bool zy100_final_edge_lf_pre_ring_online_workspace_borrowed(void)
{
    return s_lf_pre_count == ZY100_FINAL_EDGE_LF_PRE_BORROWED_COUNT;
}

bool zy100_final_edge_lf_pre_ring_online_workspace_available(void)
{
    return !zy100_final_edge_lf_pre_ring_online_workspace_borrowed() &&
           (s_lf_pre_count == 0U) && (s_lf_pre_write_pos == 0U);
}

bool zy100_final_edge_lf_pre_ring_online_workspace_claim(
    uint8_t **out_base,
    uint32_t *out_bytes)
{
    if (out_base != NULL)
    {
        *out_base = NULL;
    }
    if (out_bytes != NULL)
    {
        *out_bytes = 0U;
    }
    if ((out_base == NULL) || (out_bytes == NULL) ||
        !zy100_final_edge_lf_pre_ring_online_workspace_available())
    {
        return false;
    }

    /* Set the guard before publishing the writable span. */
    s_lf_pre_count = ZY100_FINAL_EDGE_LF_PRE_BORROWED_COUNT;
    *out_base = (uint8_t *)&s_lf_pre_ring[0];
    *out_bytes = (uint32_t)sizeof(s_lf_pre_ring);
    return true;
}

bool zy100_final_edge_lf_pre_ring_online_workspace_release(void)
{
    if (!zy100_final_edge_lf_pre_ring_online_workspace_borrowed())
    {
        return false;
    }

    /* Keep the borrowed marker asserted until the payload is discarded. */
    memset(s_lf_pre_ring, 0, sizeof(s_lf_pre_ring));
    s_lf_pre_stats.oldest_sample_index = 0U;
    s_lf_pre_stats.newest_sample_index = 0U;
    s_lf_pre_stats.oldest_tmst_raw = 0U;
    s_lf_pre_stats.newest_tmst_raw = 0U;
    s_lf_pre_write_pos = 0U;
    s_lf_pre_count = 0U;
    return true;
}

void zy100_final_edge_lf_pre_ring_push(
    uint32_t sample_index,
    uint16_t tmst_raw,
    const uint8_t packet[ZY100_FINAL_EDGE_LF_PRE_PACKET_BYTES])
{
    zy100_final_edge_lf_pre_entry_t *entry;

    if (zy100_final_edge_lf_pre_ring_online_workspace_borrowed() ||
        (packet == NULL))
    {
        return;
    }

    entry = &s_lf_pre_ring[s_lf_pre_write_pos];
    memcpy(entry->packet, packet, ZY100_FINAL_EDGE_LF_PRE_PACKET_BYTES);
    entry->live_sample_index = sample_index;
    entry->tmst_raw = tmst_raw;
    entry->flags = 0U;

    s_lf_pre_write_pos = lf_pre_advance(s_lf_pre_write_pos);
    if (s_lf_pre_count < ZY100_FINAL_EDGE_LF_PRE_RING_PACKETS)
    {
        s_lf_pre_count++;
    }
    else
    {
        s_lf_pre_stats.wrap_count++;
    }
    s_lf_pre_stats.push_count++;
    lf_pre_refresh_bounds();
}

bool zy100_final_edge_lf_pre_ring_query_ending_at(
    uint32_t anchor_sample_index,
    uint32_t count,
    zy100_final_edge_lf_pre_query_t *out_info)
{
    zy100_final_edge_lf_pre_query_t local_info;
    zy100_final_edge_lf_pre_query_t *info =
        (out_info != NULL) ? out_info : &local_info;
    uint32_t available = 0U;
    uint32_t offset;

    memset(info, 0, sizeof(*info));
    info->requested_count = count;
    if (zy100_final_edge_lf_pre_ring_online_workspace_borrowed())
    {
        return false;
    }
    info->miss_count = s_lf_pre_stats.miss_count;
    info->wrap_count = s_lf_pre_stats.wrap_count;

    if (count == 0U)
    {
        info->valid = 1U;
        return true;
    }
    if ((anchor_sample_index + 1U) < count)
    {
        s_lf_pre_stats.miss_count++;
        info->miss_count = s_lf_pre_stats.miss_count;
        return false;
    }

    for (offset = 0U; offset < count; offset++)
    {
        uint32_t sample = anchor_sample_index - offset;
        const zy100_final_edge_lf_pre_entry_t *entry =
            lf_pre_find_entry(sample);

        if (entry == NULL)
        {
            break;
        }

        if (available == 0U)
        {
            info->newest_sample_index = entry->live_sample_index;
            info->newest_tmst_raw = entry->tmst_raw;
        }
        info->oldest_sample_index = entry->live_sample_index;
        info->oldest_tmst_raw = entry->tmst_raw;
        available++;
    }

    info->packets_available = available;
    if (available == count)
    {
        info->valid = 1U;
        return true;
    }

    s_lf_pre_stats.miss_count++;
    info->miss_count = s_lf_pre_stats.miss_count;
    return false;
}

bool zy100_final_edge_lf_pre_ring_copy_ending_at(
    uint32_t anchor_sample_index,
    uint32_t count,
    zy100_final_edge_lf_pre_entry_t *out_entries,
    uint32_t out_capacity,
    zy100_final_edge_lf_pre_query_t *out_info)
{
    zy100_final_edge_lf_pre_query_t local_info;
    zy100_final_edge_lf_pre_query_t *info =
        (out_info != NULL) ? out_info : &local_info;
    uint32_t idx;
    uint32_t first_sample;

    memset(info, 0, sizeof(*info));
    info->requested_count = count;
    if (zy100_final_edge_lf_pre_ring_online_workspace_borrowed())
    {
        return false;
    }
    info->miss_count = s_lf_pre_stats.miss_count;
    info->wrap_count = s_lf_pre_stats.wrap_count;

    if (count == 0U)
    {
        info->valid = 1U;
        return true;
    }
    if ((out_entries == NULL) || (out_capacity < count) ||
        ((anchor_sample_index + 1U) < count))
    {
        s_lf_pre_stats.miss_count++;
        info->miss_count = s_lf_pre_stats.miss_count;
        return false;
    }

    first_sample = anchor_sample_index - (count - 1U);
    for (idx = 0U; idx < count; idx++)
    {
        uint32_t expected_sample = first_sample + idx;
        const zy100_final_edge_lf_pre_entry_t *entry =
            lf_pre_find_entry(expected_sample);

        if ((entry == NULL) ||
            (entry->live_sample_index != expected_sample))
        {
            s_lf_pre_stats.miss_count++;
            info->packets_available = idx;
            info->miss_count = s_lf_pre_stats.miss_count;
            return false;
        }

        out_entries[idx] = *entry;
        if (idx == 0U)
        {
            info->oldest_sample_index = entry->live_sample_index;
            info->oldest_tmst_raw = entry->tmst_raw;
        }
        info->newest_sample_index = entry->live_sample_index;
        info->newest_tmst_raw = entry->tmst_raw;
        info->packets_available = idx + 1U;
    }

    info->valid = 1U;
    return true;
}

void zy100_final_edge_lf_pre_ring_get_stats(
    zy100_final_edge_lf_pre_stats_t *out_stats)
{
    if (out_stats == NULL)
    {
        return;
    }

    if (zy100_final_edge_lf_pre_ring_online_workspace_borrowed())
    {
        *out_stats = s_lf_pre_stats;
        out_stats->oldest_sample_index = 0U;
        out_stats->newest_sample_index = 0U;
        out_stats->oldest_tmst_raw = 0U;
        out_stats->newest_tmst_raw = 0U;
        return;
    }

    lf_pre_refresh_bounds();
    *out_stats = s_lf_pre_stats;
}

void zy100_final_edge_lf_pre_ring_diag_reset(void)
{
    if (zy100_final_edge_lf_pre_ring_online_workspace_borrowed())
    {
        return;
    }
#if ZY100_FINAL_EDGE_LF_PRE_DIAG_ENABLE
    lf_pre_diag_reset_state();
#endif
}

void zy100_final_edge_lf_pre_ring_diag_note_saved_section(
    uint32_t raw_id,
    uint32_t anchor_sample_index,
    uint32_t count)
{
    if (zy100_final_edge_lf_pre_ring_online_workspace_borrowed())
    {
        return;
    }
#if ZY100_FINAL_EDGE_LF_PRE_DIAG_ENABLE
    uint32_t first_sample;
    uint32_t idx;
    uint16_t prev_tmst = 0U;
    bool prev_valid = false;
    bool section_has_gap = false;

    s_lf_pre_diag.saved_sections++;
    if (count < 2U)
    {
        return;
    }
    if ((anchor_sample_index + 1U) < count)
    {
        return;
    }

    first_sample = anchor_sample_index - (count - 1U);
    for (idx = 0U; idx < count; idx++)
    {
        const zy100_final_edge_lf_pre_entry_t *entry =
            lf_pre_find_entry(first_sample + idx);

        if (entry == NULL)
        {
            return;
        }

        if (prev_valid)
        {
            uint16_t delta =
                (uint16_t)(entry->tmst_raw - prev_tmst);

            if ((delta < ZY100_FINAL_EDGE_LF_PRE_DIAG_DELTA_MIN_US) ||
                (delta > ZY100_FINAL_EDGE_LF_PRE_DIAG_DELTA_MAX_US))
            {
                if (!section_has_gap)
                {
                    s_lf_pre_diag.gap_sections++;
                    section_has_gap = true;
                }
                s_lf_pre_diag.gap_count++;
                if (delta > s_lf_pre_diag.gap_max_delta_us)
                {
                    s_lf_pre_diag.gap_max_delta_us = delta;
                }
                if (s_lf_pre_diag.first_bad_raw_id ==
                    ZY100_FINAL_EDGE_LF_PRE_DIAG_RAW_INVALID)
                {
                    s_lf_pre_diag.first_bad_raw_id = raw_id;
                    s_lf_pre_diag.first_bad_packet_index =
                        (idx > 0xFFFFU) ? 0xFFFFU : (uint16_t)idx;
                    s_lf_pre_diag.first_bad_delta_us = delta;
                }
            }
        }

        prev_tmst = entry->tmst_raw;
        prev_valid = true;
    }
#else
    (void)raw_id;
    (void)anchor_sample_index;
    (void)count;
#endif
}

void zy100_final_edge_lf_pre_ring_diag_get(
    zy100_final_edge_lf_pre_diag_t *out_diag)
{
    if (out_diag == NULL)
    {
        return;
    }

    if (zy100_final_edge_lf_pre_ring_online_workspace_borrowed())
    {
        memset(out_diag, 0, sizeof(*out_diag));
        out_diag->first_bad_raw_id =
            ZY100_FINAL_EDGE_LF_PRE_DIAG_RAW_INVALID;
        out_diag->first_bad_packet_index =
            ZY100_FINAL_EDGE_LF_PRE_DIAG_IDX_INVALID;
        return;
    }

#if ZY100_FINAL_EDGE_LF_PRE_DIAG_ENABLE
    *out_diag = s_lf_pre_diag;
#else
    memset(out_diag, 0, sizeof(*out_diag));
    out_diag->first_bad_raw_id =
        ZY100_FINAL_EDGE_LF_PRE_DIAG_RAW_INVALID;
    out_diag->first_bad_packet_index =
        ZY100_FINAL_EDGE_LF_PRE_DIAG_IDX_INVALID;
#endif
}

#else

bool zy100_final_edge_lf_pre_ring_online_workspace_borrowed(void)
{
    return false;
}

bool zy100_final_edge_lf_pre_ring_online_workspace_available(void)
{
    return false;
}

bool zy100_final_edge_lf_pre_ring_online_workspace_claim(
    uint8_t **out_base,
    uint32_t *out_bytes)
{
    if (out_base != NULL)
    {
        *out_base = NULL;
    }
    if (out_bytes != NULL)
    {
        *out_bytes = 0U;
    }
    return false;
}

bool zy100_final_edge_lf_pre_ring_online_workspace_release(void)
{
    return false;
}

void zy100_final_edge_lf_pre_ring_reset(void)
{
}

void zy100_final_edge_lf_pre_ring_push(
    uint32_t sample_index,
    uint16_t tmst_raw,
    const uint8_t packet[ZY100_FINAL_EDGE_LF_PRE_PACKET_BYTES])
{
    (void)sample_index;
    (void)tmst_raw;
    (void)packet;
}

bool zy100_final_edge_lf_pre_ring_query_ending_at(
    uint32_t anchor_sample_index,
    uint32_t count,
    zy100_final_edge_lf_pre_query_t *out_info)
{
    if (out_info != NULL)
    {
        memset(out_info, 0, sizeof(*out_info));
        out_info->requested_count = count;
    }
    (void)anchor_sample_index;
    return false;
}

bool zy100_final_edge_lf_pre_ring_copy_ending_at(
    uint32_t anchor_sample_index,
    uint32_t count,
    zy100_final_edge_lf_pre_entry_t *out_entries,
    uint32_t out_capacity,
    zy100_final_edge_lf_pre_query_t *out_info)
{
    if (out_info != NULL)
    {
        memset(out_info, 0, sizeof(*out_info));
        out_info->requested_count = count;
    }
    (void)anchor_sample_index;
    (void)out_entries;
    (void)out_capacity;
    return false;
}

void zy100_final_edge_lf_pre_ring_get_stats(
    zy100_final_edge_lf_pre_stats_t *out_stats)
{
    if (out_stats != NULL)
    {
        memset(out_stats, 0, sizeof(*out_stats));
    }
}

void zy100_final_edge_lf_pre_ring_diag_reset(void)
{
}

void zy100_final_edge_lf_pre_ring_diag_note_saved_section(
    uint32_t raw_id,
    uint32_t anchor_sample_index,
    uint32_t count)
{
    (void)raw_id;
    (void)anchor_sample_index;
    (void)count;
}

void zy100_final_edge_lf_pre_ring_diag_get(
    zy100_final_edge_lf_pre_diag_t *out_diag)
{
    if (out_diag != NULL)
    {
        memset(out_diag, 0, sizeof(*out_diag));
        out_diag->first_bad_raw_id =
            ZY100_FINAL_EDGE_LF_PRE_DIAG_RAW_INVALID;
        out_diag->first_bad_packet_index =
            ZY100_FINAL_EDGE_LF_PRE_DIAG_IDX_INVALID;
    }
}

#endif
