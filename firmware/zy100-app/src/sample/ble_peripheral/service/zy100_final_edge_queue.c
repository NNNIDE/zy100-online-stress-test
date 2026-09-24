#include "zy100_final_edge_queue.h"

#include <stddef.h>
#include <string.h>

#include "../zy100_clock_config.h"
#include "zy100_edge_record_format.h"

#define ZY100_FINAL_EDGE_QUEUE_BORROWED_HEAD 0xFFFFU

typedef struct
{
    uint32_t seq;
    uint32_t edge_sample_index;
    uint16_t timestamp_raw;
    uint16_t payload_bytes;
    uint32_t source_flags;
    uint32_t enqueue_ms;
    zy100_edge_summary_record_t record;
} zy100_final_edge_summary_entry_t;

typedef struct
{
    uint32_t seq;
    uint32_t edge_sample_index;
    uint16_t timestamp_raw;
    uint16_t payload_bytes;
    uint32_t source_flags;
    uint32_t enqueue_ms;
    zy100_edge_event_record_t record;
} zy100_final_edge_event_entry_t;

typedef struct
{
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} zy100_final_edge_ring_t;

typedef struct
{
    zy100_final_edge_ring_t summary_ring;
    zy100_final_edge_ring_t event_ring;
    uint32_t record_seq;
    zy100_final_edge_queue_stats_t stats;
#if ZY100_FINAL_EDGE_QUEUE_ENABLE
    zy100_final_edge_summary_entry_t summary_queue[ZY100_FINAL_EDGE_SUMMARY_QUEUE_DEPTH];
    zy100_final_edge_event_entry_t event_queue[ZY100_FINAL_EDGE_EVENT_QUEUE_DEPTH];
#endif
} zy100_final_edge_queue_ctx_t;

static zy100_final_edge_queue_ctx_t s_final_edge_queue;

#if ZY100_FINAL_EDGE_QUEUE_ENABLE
typedef char zy100_final_edge_queue_online_arrays_adjacent_check[
    (offsetof(zy100_final_edge_queue_ctx_t, event_queue) ==
     (offsetof(zy100_final_edge_queue_ctx_t, summary_queue) +
      sizeof(((zy100_final_edge_queue_ctx_t *)0)->summary_queue))) ? 1 : -1];
typedef char zy100_final_edge_queue_online_workspace_size_check[
    ((sizeof(((zy100_final_edge_queue_ctx_t *)0)->summary_queue) +
      sizeof(((zy100_final_edge_queue_ctx_t *)0)->event_queue)) ==
     ZY100_FINAL_EDGE_QUEUE_ONLINE_WORKSPACE_BYTES) ? 1 : -1];
typedef char zy100_final_edge_queue_static_size_check[
    (sizeof(zy100_final_edge_queue_ctx_t) == 20568U) ? 1 : -1];
#endif

static uint16_t zy100_final_edge_queue_advance(uint16_t pos, uint16_t capacity)
    __attribute__((unused));
static uint16_t zy100_final_edge_queue_advance(uint16_t pos, uint16_t capacity)
{
    pos++;
    if (pos >= capacity)
    {
        pos = 0U;
    }
    return pos;
}

static uint32_t zy100_final_edge_queue_next_seq(void)
    __attribute__((unused));
static uint32_t zy100_final_edge_queue_next_seq(void)
{
    uint32_t seq = s_final_edge_queue.record_seq;

    s_final_edge_queue.record_seq++;
    return seq;
}

static void zy100_final_edge_fill_record_header(zy100_edge_record_header_t *header,
                                                uint16_t type,
                                                uint16_t total_bytes,
                                                uint32_t seq)
{
    memset(header, 0, sizeof(*header));
    header->magic = ZY100_EDGE_MAGIC_ZER1;
    header->type = type;
    header->header_bytes = (uint16_t)sizeof(zy100_edge_record_header_t);
    header->payload_bytes =
        (uint16_t)(total_bytes - (uint16_t)sizeof(zy100_edge_record_header_t));
    header->seq = seq;
    /*
     * Phase 2 keeps RAM-only records. CRC is intentionally a placeholder here;
     * persistent/export CRC is owned by the later Flash/export phase.
     */
    header->crc32 = 0U;
}

static void zy100_final_edge_fill_summary_record(
    const zy100_edge_lite_summary_t *src,
    zy100_edge_summary_record_t *dst,
    uint32_t seq) __attribute__((unused));
static void zy100_final_edge_fill_summary_record(
    const zy100_edge_lite_summary_t *src,
    zy100_edge_summary_record_t *dst,
    uint32_t seq)
{
    memset(dst, 0, sizeof(*dst));
    zy100_final_edge_fill_record_header(&dst->header,
                                        ZY100_EDGE_RECORD_TYPE_SUMMARY,
                                        (uint16_t)sizeof(*dst),
                                        seq);
    dst->summary_id = src->summary_id;
    dst->start_sample = src->start_sample;
    dst->end_sample = src->end_sample;
    dst->start_t_us = src->start_t_us;
    dst->duration_samples = src->duration_samples;
    dst->energy_sum = src->energy_sum;
    dst->energy_peak = src->energy_peak;
    dst->gyro_peak = src->gyro_peak;
    dst->acc_delta_peak = src->acc_delta_peak;
    dst->clip_count = src->clip_count;
    dst->sentinel_count = src->sentinel_count;
    dst->candidate_count = src->candidate_count;
}

static void zy100_final_edge_fill_event_record(
    const zy100_edge_lite_event_t *src,
    zy100_edge_event_record_t *dst,
    uint32_t seq) __attribute__((unused));
static void zy100_final_edge_fill_event_record(
    const zy100_edge_lite_event_t *src,
    zy100_edge_event_record_t *dst,
    uint32_t seq)
{
    uint8_t i;

    memset(dst, 0, sizeof(*dst));
    zy100_final_edge_fill_record_header(&dst->header,
                                        ZY100_EDGE_RECORD_TYPE_EVENT_FEATURE,
                                        (uint16_t)sizeof(*dst),
                                        seq);
    dst->event_id = src->event_id;
    dst->start_sample = src->start_sample;
    dst->trigger_sample = src->trigger_sample;
    dst->end_sample = src->end_sample;
    dst->duration_samples = src->duration_samples;
    dst->trigger_energy = src->trigger_energy;
    dst->peak_energy = src->peak_energy;
    dst->clip_count = src->clip_count;
    dst->sentinel_count = src->sentinel_count;
    dst->quality_flags = src->quality_flags;
    dst->raw_requested = src->raw_requested;
    dst->raw_saved = 0U;
    dst->raw_saved_level = ZY100_EDGE_RAW_SAVED_LEVEL_NOT_SAVED;
    dst->raw_window_id = src->raw_window_id;
    for (i = 0U; i < 3U; i++)
    {
        dst->gyro_peak[i] = src->gyro_peak[i];
        dst->acc_peak[i] = src->acc_peak[i];
        dst->gyro_axis_energy[i] = src->gyro_axis_energy[i];
        dst->acc_axis_energy[i] = src->acc_axis_energy[i];
    }
    for (i = 0U; i < 4U; i++)
    {
        dst->phase_energy[i] = src->phase_energy[i];
    }
    for (i = 0U; i < ZY100_EDGE_SHAPE_BINS; i++)
    {
        dst->shape_acc[i] = src->shape_acc[i];
        dst->shape_gyro[i] = src->shape_gyro[i];
    }
}

static void zy100_final_edge_queue_update_memory_stats(void)
{
#if ZY100_FINAL_EDGE_QUEUE_ENABLE
    s_final_edge_queue.stats.summary_queue_bytes =
        (uint32_t)sizeof(s_final_edge_queue.summary_queue);
    s_final_edge_queue.stats.event_queue_bytes =
        (uint32_t)sizeof(s_final_edge_queue.event_queue);
    s_final_edge_queue.stats.queue_bytes_est =
        s_final_edge_queue.stats.summary_queue_bytes +
        s_final_edge_queue.stats.event_queue_bytes;
#else
    s_final_edge_queue.stats.summary_queue_bytes = 0U;
    s_final_edge_queue.stats.event_queue_bytes = 0U;
    s_final_edge_queue.stats.queue_bytes_est = 0U;
#endif
}

bool zy100_final_edge_queue_online_workspace_borrowed(void)
{
#if ZY100_FINAL_EDGE_QUEUE_ENABLE
    return s_final_edge_queue.summary_ring.head ==
           ZY100_FINAL_EDGE_QUEUE_BORROWED_HEAD;
#else
    return false;
#endif
}

bool zy100_final_edge_queue_online_workspace_available(void)
{
#if ZY100_FINAL_EDGE_QUEUE_ENABLE
    return !zy100_final_edge_queue_online_workspace_borrowed() &&
           (s_final_edge_queue.summary_ring.count == 0U) &&
           (s_final_edge_queue.event_ring.count == 0U) &&
           (s_final_edge_queue.summary_ring.head == 0U) &&
           (s_final_edge_queue.summary_ring.tail == 0U) &&
           (s_final_edge_queue.event_ring.head == 0U) &&
           (s_final_edge_queue.event_ring.tail == 0U);
#else
    return false;
#endif
}

bool zy100_final_edge_queue_online_workspace_claim(
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

#if ZY100_FINAL_EDGE_QUEUE_ENABLE
    if ((out_base == NULL) || (out_bytes == NULL) ||
        !zy100_final_edge_queue_online_workspace_available())
    {
        return false;
    }

    /* Set the guard before publishing the writable span. */
    s_final_edge_queue.summary_ring.head =
        ZY100_FINAL_EDGE_QUEUE_BORROWED_HEAD;
    s_final_edge_queue.stats.summary_current_depth = 0U;
    s_final_edge_queue.stats.event_current_depth = 0U;
    *out_base = (uint8_t *)&s_final_edge_queue.summary_queue[0];
    *out_bytes = (uint32_t)(sizeof(s_final_edge_queue.summary_queue) +
                            sizeof(s_final_edge_queue.event_queue));
    return true;
#else
    return false;
#endif
}

bool zy100_final_edge_queue_online_workspace_release(void)
{
#if ZY100_FINAL_EDGE_QUEUE_ENABLE
    if (!zy100_final_edge_queue_online_workspace_borrowed())
    {
        return false;
    }

    /* Keep the borrowed marker asserted until the payload is discarded. */
    memset(s_final_edge_queue.summary_queue, 0,
           sizeof(s_final_edge_queue.summary_queue));
    memset(s_final_edge_queue.event_queue, 0,
           sizeof(s_final_edge_queue.event_queue));
    s_final_edge_queue.summary_ring.tail = 0U;
    s_final_edge_queue.summary_ring.count = 0U;
    s_final_edge_queue.event_ring.head = 0U;
    s_final_edge_queue.event_ring.tail = 0U;
    s_final_edge_queue.event_ring.count = 0U;
    s_final_edge_queue.stats.summary_current_depth = 0U;
    s_final_edge_queue.stats.event_current_depth = 0U;
    s_final_edge_queue.summary_ring.head = 0U;
    return true;
#else
    return false;
#endif
}

void zy100_final_edge_queue_reset(void)
{
    uint32_t reset_count = s_final_edge_queue.stats.queue_reset_count + 1U;

    if (zy100_final_edge_queue_online_workspace_borrowed())
    {
        return;
    }

    memset(&s_final_edge_queue, 0, sizeof(s_final_edge_queue));
    s_final_edge_queue.stats.queue_reset_count = reset_count;
    zy100_final_edge_queue_update_memory_stats();
}

bool zy100_final_edge_queue_push_summary_lite(
    const zy100_edge_lite_summary_t *summary,
    uint32_t edge_sample_index,
    uint16_t timestamp_raw)
{
    return zy100_final_edge_queue_push_summary_lite_ex(
        summary,
        edge_sample_index,
        timestamp_raw,
        ZY100_FINAL_EDGE_QUEUE_SOURCE_LIVE_800);
}

bool zy100_final_edge_queue_push_summary_lite_ex(
    const zy100_edge_lite_summary_t *summary,
    uint32_t edge_sample_index,
    uint16_t timestamp_raw,
    uint32_t source_flags)
{
#if ZY100_FINAL_EDGE_QUEUE_ENABLE
    zy100_edge_summary_record_t record;
    zy100_final_edge_summary_entry_t *entry;
    uint32_t seq;

    if (zy100_final_edge_queue_online_workspace_borrowed() ||
        (summary == NULL))
    {
        return false;
    }
    if (sizeof(record) > ZY100_FINAL_EDGE_SUMMARY_RECORD_MAX_BYTES)
    {
        s_final_edge_queue.stats.summary_dropped++;
        s_final_edge_queue.stats.summary_drop_oversize++;
        return false;
    }

    if (s_final_edge_queue.summary_ring.count >=
        ZY100_FINAL_EDGE_SUMMARY_QUEUE_DEPTH)
    {
#if ZY100_FINAL_EDGE_QUEUE_DROP_OLDEST_SUMMARY
        s_final_edge_queue.stats.summary_dropped++;
        s_final_edge_queue.stats.summary_drop_oldest++;
        s_final_edge_queue.summary_ring.head =
            zy100_final_edge_queue_advance(s_final_edge_queue.summary_ring.head,
                                           ZY100_FINAL_EDGE_SUMMARY_QUEUE_DEPTH);
        s_final_edge_queue.summary_ring.count--;
#else
        s_final_edge_queue.stats.summary_dropped++;
        return false;
#endif
    }

    seq = zy100_final_edge_queue_next_seq();
    zy100_final_edge_fill_summary_record(summary, &record, seq);

    entry = &s_final_edge_queue.summary_queue[s_final_edge_queue.summary_ring.tail];
    entry->seq = seq;
    entry->edge_sample_index = edge_sample_index;
    entry->timestamp_raw = timestamp_raw;
    entry->payload_bytes = (uint16_t)sizeof(record);
    entry->source_flags = source_flags;
    entry->enqueue_ms = zy100_os_time_ms();
    memcpy(&entry->record, &record, sizeof(record));
    s_final_edge_queue.summary_ring.tail =
        zy100_final_edge_queue_advance(s_final_edge_queue.summary_ring.tail,
                                       ZY100_FINAL_EDGE_SUMMARY_QUEUE_DEPTH);
    s_final_edge_queue.summary_ring.count++;
    s_final_edge_queue.stats.summary_enqueued++;
    s_final_edge_queue.stats.summary_current_depth =
        s_final_edge_queue.summary_ring.count;
    if (s_final_edge_queue.stats.summary_current_depth >
        s_final_edge_queue.stats.summary_peak_depth)
    {
        s_final_edge_queue.stats.summary_peak_depth =
            s_final_edge_queue.stats.summary_current_depth;
    }
    return true;
#else
    (void)summary;
    (void)edge_sample_index;
    (void)timestamp_raw;
    (void)source_flags;
    return false;
#endif
}

bool zy100_final_edge_queue_push_event_lite(
    const zy100_edge_lite_event_t *event,
    uint32_t edge_sample_index,
    uint16_t timestamp_raw)
{
    return zy100_final_edge_queue_push_event_lite_ex(
        event,
        edge_sample_index,
        timestamp_raw,
        ZY100_FINAL_EDGE_QUEUE_SOURCE_LIVE_800);
}

bool zy100_final_edge_queue_push_event_lite_ex(
    const zy100_edge_lite_event_t *event,
    uint32_t edge_sample_index,
    uint16_t timestamp_raw,
    uint32_t source_flags)
{
#if ZY100_FINAL_EDGE_QUEUE_ENABLE
    zy100_edge_event_record_t record;
    zy100_final_edge_event_entry_t *entry;
    uint32_t seq;

    if (zy100_final_edge_queue_online_workspace_borrowed() ||
        (event == NULL))
    {
        return false;
    }
    if (sizeof(record) > ZY100_FINAL_EDGE_EVENT_RECORD_MAX_BYTES)
    {
        s_final_edge_queue.stats.event_dropped++;
        s_final_edge_queue.stats.event_drop_oversize++;
        return false;
    }

    if (s_final_edge_queue.event_ring.count >=
        ZY100_FINAL_EDGE_EVENT_QUEUE_DEPTH)
    {
#if ZY100_FINAL_EDGE_QUEUE_DROP_OLDEST_EVENT
        s_final_edge_queue.stats.event_dropped++;
        s_final_edge_queue.stats.event_drop_oldest++;
        s_final_edge_queue.event_ring.head =
            zy100_final_edge_queue_advance(s_final_edge_queue.event_ring.head,
                                           ZY100_FINAL_EDGE_EVENT_QUEUE_DEPTH);
        s_final_edge_queue.event_ring.count--;
#else
        s_final_edge_queue.stats.event_dropped++;
        return false;
#endif
    }

    seq = zy100_final_edge_queue_next_seq();
    zy100_final_edge_fill_event_record(event, &record, seq);

    entry = &s_final_edge_queue.event_queue[s_final_edge_queue.event_ring.tail];
    entry->seq = seq;
    entry->edge_sample_index = edge_sample_index;
    entry->timestamp_raw = timestamp_raw;
    entry->payload_bytes = (uint16_t)sizeof(record);
    entry->source_flags = source_flags;
    entry->enqueue_ms = zy100_os_time_ms();
    memcpy(&entry->record, &record, sizeof(record));
    s_final_edge_queue.event_ring.tail =
        zy100_final_edge_queue_advance(s_final_edge_queue.event_ring.tail,
                                       ZY100_FINAL_EDGE_EVENT_QUEUE_DEPTH);
    s_final_edge_queue.event_ring.count++;
    s_final_edge_queue.stats.event_enqueued++;
    s_final_edge_queue.stats.event_current_depth =
        s_final_edge_queue.event_ring.count;
    if (s_final_edge_queue.stats.event_current_depth >
        s_final_edge_queue.stats.event_peak_depth)
    {
        s_final_edge_queue.stats.event_peak_depth =
            s_final_edge_queue.stats.event_current_depth;
    }
    return true;
#else
    (void)event;
    (void)edge_sample_index;
    (void)timestamp_raw;
    (void)source_flags;
    return false;
#endif
}

bool zy100_final_edge_queue_pop_summary_copy(
    zy100_final_edge_summary_copy_t *out)
{
#if ZY100_FINAL_EDGE_QUEUE_ENABLE
    const zy100_final_edge_summary_entry_t *entry;

    if (zy100_final_edge_queue_online_workspace_borrowed() ||
        (out == NULL) || (s_final_edge_queue.summary_ring.count == 0U))
    {
        return false;
    }

    entry = &s_final_edge_queue.summary_queue[s_final_edge_queue.summary_ring.head];
    memset(out, 0, sizeof(*out));
    out->seq = entry->seq;
    out->edge_sample_index = entry->edge_sample_index;
    out->source_flags = entry->source_flags;
    out->timestamp_raw = entry->timestamp_raw;
    out->payload_bytes = entry->payload_bytes;
    memcpy(&out->record, &entry->record, sizeof(out->record));

    s_final_edge_queue.summary_ring.head =
        zy100_final_edge_queue_advance(s_final_edge_queue.summary_ring.head,
                                       ZY100_FINAL_EDGE_SUMMARY_QUEUE_DEPTH);
    s_final_edge_queue.summary_ring.count--;
    s_final_edge_queue.stats.summary_current_depth =
        s_final_edge_queue.summary_ring.count;
    return true;
#else
    (void)out;
    return false;
#endif
}

bool zy100_final_edge_queue_pop_event_copy(
    zy100_final_edge_event_copy_t *out)
{
#if ZY100_FINAL_EDGE_QUEUE_ENABLE
    const zy100_final_edge_event_entry_t *entry;

    if (zy100_final_edge_queue_online_workspace_borrowed() ||
        (out == NULL) || (s_final_edge_queue.event_ring.count == 0U))
    {
        return false;
    }

    entry = &s_final_edge_queue.event_queue[s_final_edge_queue.event_ring.head];
    memset(out, 0, sizeof(*out));
    out->seq = entry->seq;
    out->edge_sample_index = entry->edge_sample_index;
    out->source_flags = entry->source_flags;
    out->timestamp_raw = entry->timestamp_raw;
    out->payload_bytes = entry->payload_bytes;
    memcpy(&out->record, &entry->record, sizeof(out->record));

    s_final_edge_queue.event_ring.head =
        zy100_final_edge_queue_advance(s_final_edge_queue.event_ring.head,
                                       ZY100_FINAL_EDGE_EVENT_QUEUE_DEPTH);
    s_final_edge_queue.event_ring.count--;
    s_final_edge_queue.stats.event_current_depth =
        s_final_edge_queue.event_ring.count;
    return true;
#else
    (void)out;
    return false;
#endif
}

void zy100_final_edge_queue_note_raw_req_suppressed(void)
{
    if (zy100_final_edge_queue_online_workspace_borrowed())
    {
        return;
    }
    s_final_edge_queue.stats.raw_req_suppressed++;
}

void zy100_final_edge_queue_note_raw_start_suppressed(void)
{
    if (zy100_final_edge_queue_online_workspace_borrowed())
    {
        return;
    }
    s_final_edge_queue.stats.raw_start_suppressed++;
}

void zy100_final_edge_queue_get_stats(zy100_final_edge_queue_stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    if (zy100_final_edge_queue_online_workspace_borrowed())
    {
        *out = s_final_edge_queue.stats;
        out->summary_current_depth = 0U;
        out->event_current_depth = 0U;
#if ZY100_FINAL_EDGE_QUEUE_ENABLE
        out->summary_queue_bytes =
            (uint32_t)sizeof(s_final_edge_queue.summary_queue);
        out->event_queue_bytes =
            (uint32_t)sizeof(s_final_edge_queue.event_queue);
        out->queue_bytes_est = out->summary_queue_bytes +
                               out->event_queue_bytes;
#endif
        return;
    }

    s_final_edge_queue.stats.summary_current_depth =
        s_final_edge_queue.summary_ring.count;
    s_final_edge_queue.stats.event_current_depth =
        s_final_edge_queue.event_ring.count;
    zy100_final_edge_queue_update_memory_stats();
    *out = s_final_edge_queue.stats;
}

uint16_t zy100_final_edge_queue_summary_count(void)
{
    if (zy100_final_edge_queue_online_workspace_borrowed())
    {
        return 0U;
    }
    return s_final_edge_queue.summary_ring.count;
}

uint16_t zy100_final_edge_queue_event_count(void)
{
    if (zy100_final_edge_queue_online_workspace_borrowed())
    {
        return 0U;
    }
    return s_final_edge_queue.event_ring.count;
}
