#include "zy100_online_raw_capture.h"
#if ZY100_ONLINE_STRESS_TEST_ENABLE
#include "zy100_online_stress.h"
#endif

#include <string.h>

#include "os_sched.h"

#include "../bsp/bsp_capture_timebase.h"
#include "../driver/gd25q32e_spi.h"
#include "../zy100_clock_config.h"
#include "zy100_crc32.h"
#include "zy100_final_edge_raw_format.h"
#include "mag_capture_service.h"
#include "zy100_capture_profile.h"
#include "zy100_mode_workspace.h"
#include "zy100_online_spool.h"
#include "zy100_online_stream.h"

#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_DIRECT_SPOOL_ENABLE && \
    ZY100_ONLINE_CONTINUOUS_RAW_ENABLE

#define ZY100_ONLINE_RAW_SPAN_COUNT              3U
#define ZY100_ONLINE_RAW_PACKET_BYTES            16U
#define ZY100_ONLINE_RAW_MAG_RING_SAMPLES         256U
#define ZY100_ONLINE_RAW_IMU_RING_BYTES          31072UL
#define ZY100_ONLINE_RAW_MAG_RING_BYTES \
    (ZY100_ONLINE_RAW_MAG_RING_SAMPLES * \
     (uint32_t)sizeof(zy100_fe_raw_mag_sample_t))
#define ZY100_ONLINE_RAW_WORKSPACE_REQUIRED_BYTES \
    (ZY100_ONLINE_RAW_IMU_RING_BYTES + ZY100_ONLINE_RAW_MAG_RING_BYTES)
#define ZY100_ONLINE_RAW_MAG_POLL_PERIOD_US       5000UL
#define ZY100_ONLINE_RAW_MAG_PERIOD_US          10000UL
#define ZY100_ONLINE_RAW_MAG_FRESH_TIMEOUT_US   50000UL
#define ZY100_ONLINE_RAW_MAG_CONSEC_ERROR_MAX      10U
#define ZY100_ONLINE_RAW_RECORD_TARGET_PACKETS   640U
#define ZY100_ONLINE_RAW_CATCHUP_HIGH_PACKETS   1280U
#define ZY100_ONLINE_RAW_PREERASE_MAX_PACKETS \
    (ZY100_ONLINE_RAW_RECORD_TARGET_PACKETS - 1U)
#define ZY100_ONLINE_RAW_RING_PACKETS \
    (ZY100_ONLINE_RAW_IMU_RING_BYTES / ZY100_ONLINE_RAW_PACKET_BYTES)
#define ZY100_ONLINE_RAW_FINISH_TIMEOUT_MS \
    (GD25Q32E_BLOCK32_ERASE_TIMEOUT_MS + \
     (3UL * ZY100_FE_RAW_IMU_MAG_SOURCE_RECORD_PAGES_MAX * \
      GD25Q32E_PAGE_PROGRAM_TIMEOUT_MS))
#define ZY100_ONLINE_RAW_FIXED40_RANGE_GUARD_MS \
    ((uint32_t)(BSP_CAPTURE_TIMEBASE_RANGE_US / 1000ULL))

typedef enum
{
    ZY100_ONLINE_RAW_RECORD_IDLE = 0U,
    ZY100_ONLINE_RAW_RECORD_WRITE,
    ZY100_ONLINE_RAW_RECORD_COMMIT,
} zy100_online_raw_record_state_t;

typedef enum
{
    ZY100_ONLINE_RAW_MAG_PHASE_TASK_GAP = 0U,
    ZY100_ONLINE_RAW_MAG_PHASE_RECORD_PREPARE,
    ZY100_ONLINE_RAW_MAG_PHASE_FLASH_IO,
    ZY100_ONLINE_RAW_MAG_PHASE_BLE_STAGING,
    ZY100_ONLINE_RAW_MAG_PHASE_AFTER_LIVE_SERVICE,
    ZY100_ONLINE_RAW_MAG_PHASE_BEFORE_COOP_YIELD_FORCE,
    ZY100_ONLINE_RAW_MAG_PHASE_AFTER_COOP_YIELD,
    ZY100_ONLINE_RAW_MAG_PHASE_AFTER_NOTIFY_WAIT,
    ZY100_ONLINE_RAW_MAG_PHASE_LOOP_ENTRY,
} zy100_online_raw_mag_phase_t;

typedef struct
{
    uint32_t value;
    uint32_t word;
    uint8_t word_bytes;
} zy100_online_raw_xor_acc_t;

typedef struct
{
    uint8_t *base;
    uint32_t bytes;
} zy100_online_raw_span_t;

typedef struct
{
    uint16_t ring;
    uint8_t level;
    uint8_t record_state;
    uint8_t erase_wip;
    uint8_t tx_ready;
    uint8_t tx_in_flight;
    uint8_t invariant;
    uint32_t reserve_waits;
} zy100_online_raw_pressure_t;

typedef struct
{
    volatile bool active;
    bool finishing;
#if ZY100_ONLINE_ERASE_AHEAD_ENABLE
    bool erase_refill;
#endif
    bool page_in_flight;
    bool page_prepared;
    bool timestamp_valid;
    bool reserve_urgent;
    bool use_fixed40_endpoint;
    bool fixed40_configured;
    uint8_t error;
    uint8_t record_state;
    uint8_t pressure_next;
    uint16_t first_timestamp_raw;
    uint16_t last_timestamp_raw;
    uint16_t record_first_timestamp_raw;
    uint16_t record_last_timestamp_raw;
    uint16_t reserved;
    uint32_t ring_head;
    uint32_t ring_count;
    uint32_t source_id_next;
    uint32_t source_page_next;
    uint32_t record_packet_count;
    uint32_t record_start_seq;
    uint32_t record_payload_bytes;
    uint32_t record_payload_xor;
    uint32_t source_record_bytes;
    uint32_t source_record_pages;
    uint32_t source_crc;
    uint32_t record_write_start_ms;
    uint32_t fixed40_last_os_ms;
    uint32_t fixed40_committed_counter;
    uint64_t fixed40_elapsed_us;
    bool mag_scheduling;
    bool mag_deadline_started;
    bool mag_checkpoint_valid;
    bool mag_gate_primed;
    uint8_t mag_consecutive_errors;
    uint32_t mag_ring_head;
    uint32_t mag_ring_count;
    uint32_t record_mag_count;
    uint32_t record_mag_read_errors;
    uint32_t record_mag_missed_deadlines;
    uint32_t pending_mag_read_errors;
    uint32_t pending_mag_missed_deadlines;
    uint32_t mag_next_poll_deadline_us;
    uint32_t mag_next_sample_deadline_us;
    uint32_t mag_last_elapsed_us;
    uint32_t mag_last_fresh_elapsed_us;
    uint32_t mag_checkpoint_last_elapsed_us;
    zy100_fe_store_target_t target;
    zy100_online_raw_span_t span[ZY100_ONLINE_RAW_SPAN_COUNT];
    zy100_online_raw_pressure_t pressure[4];
    zy100_online_raw_clock_meta_t clock_meta;
    uint8_t *page;
    uint8_t *verify;
    uint32_t workspace_token;
    zy100_online_raw_capture_stats_t stats;
} zy100_online_raw_state_t;

static zy100_online_raw_state_t s_online_raw;
static zy100_fe_raw_mag_sample_t *s_online_raw_mag_ring;
static bool s_online_mag_prepared;

typedef char zy100_online_raw_xor_packet_size_check[
    (ZY100_ONLINE_RAW_PACKET_BYTES == 16U) ? 1 : -1];
typedef char zy100_online_raw_xor_imu_header_check[
    ((sizeof(zy100_fe_raw_section_header_t) % 4U) == 0U) ? 1 : -1];

typedef char zy100_online_raw_workspace_packet_check[
    ((ZY100_ONLINE_RAW_RING_PACKETS * ZY100_ONLINE_RAW_PACKET_BYTES) ==
      ZY100_ONLINE_RAW_IMU_RING_BYTES) ? 1 : -1];
typedef char zy100_online_raw_workspace_capacity_check[
    (ZY100_ONLINE_RAW_RING_PACKETS == 1942U) ? 1 : -1];
typedef char zy100_online_raw_workspace_alignment_check[
    ((ZY100_ONLINE_RAW_IMU_RING_BYTES % sizeof(uint32_t)) == 0U) ? 1 : -1];
typedef char zy100_online_raw_record_geometry_check[
    ((ZY100_ONLINE_SPOOL_RECORD_HEADER_BYTES +
      ZY100_FE_RAW_IMU_MAG_SOURCE_RECORD_BYTES_MAX) ==
     ZY100_FE_RAW_IMU_MAG_ONLINE_RECORD_BYTES_MAX) ? 1 : -1];
typedef char zy100_online_raw_arena_layout_check[
    ((ZY100_ONLINE_RAW_IMU_RING_BYTES == ZY100_ONLINE_WORKSPACE_IMU_BYTES) &&
     (ZY100_ONLINE_RAW_MAG_RING_BYTES == ZY100_ONLINE_WORKSPACE_MAG_BYTES) &&
     (GD25Q32E_PAGE_BYTES == 256U)) ? 1 : -1];
typedef char zy100_online_raw_shared_workspace_fit_check[
    (ZY100_ONLINE_RAW_WORKSPACE_REQUIRED_BYTES <=
     ZY100_MODE_WORKSPACE_TRANSIENT_BYTES) ? 1 : -1];
typedef char zy100_online_raw_mag_record_slack_check[
    (ZY100_ONLINE_RAW_MAG_RING_SAMPLES >
     ZY100_FE_RAW_MAG_SAMPLES_PER_RECORD_MAX) ? 1 : -1];

static void online_raw_latch_error(zy100_online_raw_error_t error)
{
    if (s_online_raw.error == ZY100_ONLINE_RAW_ERROR_NONE)
    {
#if ZY100_ONLINE_STRESS_TEST_ENABLE
        zy100_online_stress_diag_freeze(0x200U + (uint32_t)error);
#endif
        s_online_raw.page_prepared = false;
        s_online_raw.error = (uint8_t)error;
        s_online_raw.stats.error = (uint8_t)error;
    }
}

static void online_raw_note_max_u32(uint32_t *maximum, uint32_t value)
{
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
    if ((maximum != NULL) && (value > *maximum))
    {
        *maximum = value;
    }
#else
    (void)maximum;
    (void)value;
#endif
}

static bool online_raw_profile_start(uint32_t *counter_out)
{
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
    return bsp_capture_timebase_snapshot(counter_out);
#else
    *counter_out = 0U;
    return false;
#endif
}

static void online_raw_profile_note_elapsed(uint32_t *maximum,
                                            uint32_t start_counter,
                                            bool start_valid)
{
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
    uint32_t end_counter;

    if (start_valid && bsp_capture_timebase_snapshot(&end_counter))
    {
        online_raw_note_max_u32(maximum, start_counter - end_counter);
    }
#else
    (void)maximum;
    (void)start_counter;
    (void)start_valid;
#endif
}

static void online_raw_finalize_fixed40(bool discard_tail)
{
    if (!s_online_raw.use_fixed40_endpoint)
    {
        return;
    }

    bsp_capture_timebase_stop();
    if (discard_tail ||
        (s_online_raw.error != ZY100_ONLINE_RAW_ERROR_NONE))
    {
        s_online_raw.clock_meta.last_fixed40_valid = 0U;
        s_online_raw.clock_meta.last_unix_time_us = 0ULL;
        return;
    }

    if ((s_online_raw.clock_meta.first_fixed40_valid == 0U) ||
        (s_online_raw.clock_meta.last_fixed40_valid == 0U))
    {
        return;
    }

    if (s_online_raw.clock_meta.fixed40_range_unsupported != 0U)
    {
        s_online_raw.clock_meta.last_unix_time_us = 0ULL;
        return;
    }

    if ((s_online_raw.clock_meta.first_unix_valid == 0U) ||
        (s_online_raw.clock_meta.first_unix_time_us >
         (0xFFFFFFFFFFFFFFFFULL - s_online_raw.fixed40_elapsed_us)))
    {
        s_online_raw.clock_meta.last_unix_time_us = 0ULL;
        return;
    }

    s_online_raw.clock_meta.last_unix_time_us =
        s_online_raw.clock_meta.first_unix_time_us +
        s_online_raw.fixed40_elapsed_us;
    s_online_raw.clock_meta.last_unix_valid = 1U;
}

static uint32_t online_raw_physical_packet(uint32_t logical_packet)
{
    return (s_online_raw.ring_head + logical_packet) %
           ZY100_ONLINE_RAW_RING_PACKETS;
}

static uint8_t *online_raw_packet_ptr(uint32_t physical_packet)
{
    uint32_t byte_offset = physical_packet * ZY100_ONLINE_RAW_PACKET_BYTES;

    if (byte_offset < s_online_raw.span[0].bytes)
    {
        return s_online_raw.span[0].base + byte_offset;
    }
    byte_offset -= s_online_raw.span[0].bytes;
    if (byte_offset < s_online_raw.span[1].bytes)
    {
        return s_online_raw.span[1].base + byte_offset;
    }
    byte_offset -= s_online_raw.span[1].bytes;
    if (byte_offset < s_online_raw.span[2].bytes)
    {
        return s_online_raw.span[2].base + byte_offset;
    }
    return NULL;
}

static const uint8_t *online_raw_logical_packet_ptr(uint32_t logical_packet)
{
    if (logical_packet >= s_online_raw.ring_count)
    {
        return NULL;
    }
    return online_raw_packet_ptr(online_raw_physical_packet(logical_packet));
}

static uint16_t online_raw_packet_timestamp(const uint8_t *packet)
{
    return (uint16_t)(((uint16_t)packet[14] << 8) | packet[15]);
}

static uint32_t online_raw_physical_mag_sample(uint32_t logical_sample)
{
    return (s_online_raw.mag_ring_head + logical_sample) %
           ZY100_ONLINE_RAW_MAG_RING_SAMPLES;
}

static const zy100_fe_raw_mag_sample_t *online_raw_logical_mag_sample(
    uint32_t logical_sample)
{
    if (logical_sample >= s_online_raw.mag_ring_count)
    {
        return NULL;
    }
    return &s_online_raw_mag_ring[
        online_raw_physical_mag_sample(logical_sample)];
}

static void online_raw_fill_imu_section(
    zy100_fe_raw_section_header_t *section)
{
    memset(section, 0, sizeof(*section));
    section->section_type =
        (uint16_t)ZY100_FE_RAW_SECTION_CONTINUOUS_800HZ;
    section->header_bytes = (uint16_t)sizeof(*section);
    section->sample_hz = ZY100_FE_RAW_CONTINUOUS_SAMPLE_HZ;
    section->packet_or_frame_bytes = ZY100_ONLINE_RAW_PACKET_BYTES;
    section->count = s_online_raw.record_packet_count;
    section->first_tmst_raw = s_online_raw.record_first_timestamp_raw;
    section->last_tmst_raw = s_online_raw.record_last_timestamp_raw;
    section->payload_bytes = s_online_raw.record_packet_count *
                             ZY100_ONLINE_RAW_PACKET_BYTES;
}

static void online_raw_fill_mag_section(
    zy100_fe_raw_mag_section_header_t *section)
{
    memset(section, 0, sizeof(*section));
    section->base.section_type =
        (uint16_t)ZY100_FE_RAW_SECTION_MAG_RAW_100HZ;
    section->base.header_bytes = (uint16_t)sizeof(*section);
    section->base.sample_hz = ZY100_FE_RAW_MAG_SAMPLE_HZ;
    section->base.packet_or_frame_bytes = ZY100_FE_RAW_MAG_SAMPLE_BYTES;
    section->base.count = s_online_raw.record_mag_count;
    section->base.payload_bytes = s_online_raw.record_mag_count *
                                  ZY100_FE_RAW_MAG_SAMPLE_BYTES;
    section->base.flags = ZY100_FE_RAW_MAG_SECTION_FLAGS_CURRENT;
    section->read_error_count = s_online_raw.record_mag_read_errors;
    section->missed_deadline_count =
        s_online_raw.record_mag_missed_deadlines;
}

static bool online_raw_record_has_imu(void)
{
    return s_online_raw.record_packet_count != 0U;
}

static bool online_raw_record_has_mag(void)
{
    return s_online_raw.record_mag_count != 0U;
}


static bool online_raw_xor_update(zy100_online_raw_xor_acc_t *acc,
                                  const void *data,
                                  uint32_t bytes)
{
    const uint8_t *src = (const uint8_t *)data;
    uint32_t index;

    if ((acc == NULL) || ((src == NULL) && (bytes != 0U)))
    {
        return false;
    }
    for (index = 0U; index < bytes; index++)
    {
        acc->word = (acc->word << 8) | (uint32_t)src[index];
        acc->word_bytes++;
        if (acc->word_bytes == 4U)
        {
            acc->value ^= acc->word;
            acc->word = 0U;
            acc->word_bytes = 0U;
        }
    }
    return true;
}

static uint32_t online_raw_imu_packet_xor(const uint8_t *packet)
{
    uint32_t value = 0U;
    uint32_t offset;

    for (offset = 0U; offset < ZY100_ONLINE_RAW_PACKET_BYTES; offset += 4U)
    {
        value ^= ((uint32_t)packet[offset] << 24) |
                 ((uint32_t)packet[offset + 1U] << 16) |
                 ((uint32_t)packet[offset + 2U] << 8) |
                 (uint32_t)packet[offset + 3U];
    }
    return value;
}

static bool online_raw_payload_xor(uint32_t *value_out)
{
    zy100_online_raw_xor_acc_t acc;
    zy100_fe_raw_section_header_t imu_section;
    zy100_fe_raw_mag_section_header_t mag_section;
    const uint8_t *packet;
    const zy100_fe_raw_mag_sample_t *sample;
    uint32_t index;

    if (value_out == NULL)
    {
        return false;
    }
    memset(&acc, 0, sizeof(acc));

    if (online_raw_record_has_imu())
    {
        online_raw_fill_imu_section(&imu_section);
        if (!online_raw_xor_update(&acc, &imu_section, sizeof(imu_section)))
        {
            return false;
        }
        for (index = 0U; index < s_online_raw.record_packet_count; index++)
        {
            packet = online_raw_logical_packet_ptr(index);
            if (packet == NULL)
            {
                return false;
            }
            /* The IMU header and every packet end on a complete XOR word. */
            acc.value ^= online_raw_imu_packet_xor(packet);
        }
    }

    if (online_raw_record_has_mag())
    {
        online_raw_fill_mag_section(&mag_section);
        if (!online_raw_xor_update(&acc, &mag_section, sizeof(mag_section)))
        {
            return false;
        }
        for (index = 0U; index < s_online_raw.record_mag_count; index++)
        {
            sample = online_raw_logical_mag_sample(index);
            if ((sample == NULL) ||
                !online_raw_xor_update(&acc,
                                       sample,
                                       ZY100_FE_RAW_MAG_SAMPLE_BYTES))
            {
                return false;
            }
        }
    }

    if (acc.word_bytes != 0U)
    {
        acc.value ^= acc.word << (8U * (4U - acc.word_bytes));
    }
    *value_out = acc.value;
    return true;
}

static void online_raw_build_header_page(void)
{
    zy100_fe_high_raw_header_t header;

    memset(&header, 0, sizeof(header));
    header.magic = ZY100_FE_RAW_MAGIC;
    header.version = ZY100_FE_RAW_IMU_MAG_VERSION;
    header.header_bytes = (uint16_t)sizeof(header);
    header.raw_id = s_online_raw.source_id_next;
    header.start_seq = s_online_raw.record_start_seq;
    header.end_seq = online_raw_record_has_imu() ?
                     (s_online_raw.record_start_seq +
                      s_online_raw.record_packet_count - 1U) : 0U;
    header.lf_packets = s_online_raw.record_packet_count;
    header.first_tmst_raw = online_raw_record_has_imu() ?
        s_online_raw.record_first_timestamp_raw : 0U;
    header.last_tmst_raw = online_raw_record_has_imu() ?
        s_online_raw.record_last_timestamp_raw : 0U;
    header.section_count =
        (online_raw_record_has_imu() ? 1U : 0U) +
        (online_raw_record_has_mag() ? 1U : 0U);
    header.payload_offset = ZY100_FE_RAW_HEADER_PAGE_BYTES;
    header.payload_bytes = s_online_raw.record_payload_bytes;
    header.bucket_bytes = ZY100_ONLINE_SPOOL_RECORD_HEADER_BYTES +
                          s_online_raw.source_record_bytes;
    header.flags =
        (online_raw_record_has_imu() ?
         ZY100_FE_RAW_FLAG_CONTINUOUS_800HZ : 0U) |
        (online_raw_record_has_mag() ?
         ZY100_FE_RAW_FLAG_MAG_RAW_100HZ : 0U);
    header.payload_xor = s_online_raw.record_payload_xor;
    header.page_count = s_online_raw.source_record_pages;
    header.status = ZY100_FE_RAW_STATUS_COMMITTED;

    memset(s_online_raw.page, 0xFF, GD25Q32E_PAGE_BYTES);
    memcpy(s_online_raw.page, &header, sizeof(header));
}

static uint32_t online_raw_min(uint32_t a, uint32_t b)
{
    return (a < b) ? a : b;
}

static bool online_raw_copy_payload(uint32_t offset, uint8_t *dst,
                                     uint32_t limit, uint32_t *copied)
{
    uint32_t section_offset = 0U;
    uint32_t raw_offset;
    uint32_t index;
    uint32_t within;
    uint32_t bytes;
    const uint8_t *src;

    if ((dst == NULL) || (copied == NULL) || (limit == 0U) ||
        (offset >= s_online_raw.record_payload_bytes))
    {
        return false;
    }
    limit = online_raw_min(limit, s_online_raw.record_payload_bytes - offset);
    if (online_raw_record_has_imu())
    {
        uint32_t imu_bytes = s_online_raw.record_packet_count *
                             ZY100_ONLINE_RAW_PACKET_BYTES;
        if (offset < sizeof(zy100_fe_raw_section_header_t))
        {
            zy100_fe_raw_section_header_t header;
            online_raw_fill_imu_section(&header);
            bytes = online_raw_min(limit, (uint32_t)sizeof(header) - offset);
            memcpy(dst, (const uint8_t *)&header + offset, bytes);
            *copied = bytes;
            return true;
        }
        raw_offset = offset - (uint32_t)sizeof(zy100_fe_raw_section_header_t);
        if (raw_offset < imu_bytes)
        {
            uint32_t physical;
            uint32_t span;
            index = raw_offset / ZY100_ONLINE_RAW_PACKET_BYTES;
            within = raw_offset % ZY100_ONLINE_RAW_PACKET_BYTES;
            if ((index >= s_online_raw.record_packet_count) ||
                (index >= s_online_raw.ring_count)) { return false; }
            physical = online_raw_physical_packet(index) * ZY100_ONLINE_RAW_PACKET_BYTES;
            bytes = online_raw_min(limit, imu_bytes - raw_offset);
            bytes = online_raw_min(bytes,
                (s_online_raw.ring_count - index) * ZY100_ONLINE_RAW_PACKET_BYTES - within);
            bytes = online_raw_min(bytes, ZY100_ONLINE_RAW_IMU_RING_BYTES - physical - within);
            for (span = 0U; span < ZY100_ONLINE_RAW_SPAN_COUNT; span++)
            {
                if (physical < s_online_raw.span[span].bytes)
                {
                    if ((s_online_raw.span[span].base == NULL) ||
                        (within >= s_online_raw.span[span].bytes - physical)) { return false; }
                    bytes = online_raw_min(bytes, s_online_raw.span[span].bytes - physical - within);
                    src = s_online_raw.span[span].base + physical + within;
                    memcpy(dst, src, bytes);
                    *copied = bytes;
                    return true;
                }
                physical -= s_online_raw.span[span].bytes;
            }
            return false;
        }
        section_offset = (uint32_t)sizeof(zy100_fe_raw_section_header_t) + imu_bytes;
    }
    if (online_raw_record_has_mag())
    {
        raw_offset = offset - section_offset;
        if (raw_offset < sizeof(zy100_fe_raw_mag_section_header_t))
        {
            zy100_fe_raw_mag_section_header_t header;
            online_raw_fill_mag_section(&header);
            bytes = online_raw_min(limit, (uint32_t)sizeof(header) - raw_offset);
            memcpy(dst, (const uint8_t *)&header + raw_offset, bytes);
            *copied = bytes;
            return true;
        }
        raw_offset -= (uint32_t)sizeof(zy100_fe_raw_mag_section_header_t);
        index = raw_offset / ZY100_FE_RAW_MAG_SAMPLE_BYTES;
        within = raw_offset % ZY100_FE_RAW_MAG_SAMPLE_BYTES;
        if ((index >= s_online_raw.record_mag_count) ||
            (index >= s_online_raw.mag_ring_count) || (s_online_raw_mag_ring == NULL))
        {
            return false;
        }
        bytes = online_raw_min(s_online_raw.record_mag_count - index,
                               s_online_raw.mag_ring_count - index);
        bytes = online_raw_min(bytes, ZY100_ONLINE_RAW_MAG_RING_SAMPLES -
                               online_raw_physical_mag_sample(index));
        bytes = online_raw_min(limit, bytes * ZY100_FE_RAW_MAG_SAMPLE_BYTES - within);
        src = (const uint8_t *)online_raw_logical_mag_sample(index) + within;
        memcpy(dst, src, bytes);
        *copied = bytes;
        return true;
    }
    return false;
}

static bool online_raw_build_payload_page(uint32_t payload_page)
{
    uint32_t page_offset = payload_page * GD25Q32E_PAGE_BYTES;
    uint32_t index = 0U;
    memset(s_online_raw.page, 0xFF, GD25Q32E_PAGE_BYTES);
    while ((index < GD25Q32E_PAGE_BYTES) &&
           (page_offset + index < s_online_raw.record_payload_bytes))
    {
        uint32_t copied;
        if (!online_raw_copy_payload(page_offset + index, s_online_raw.page + index,
                                      GD25Q32E_PAGE_BYTES - index, &copied))
        {
            return false;
        }
        index += copied;
    }
    return true;
}

static bool online_raw_build_source_page(uint32_t source_page)
{
    if (source_page == 0U)
    {
        online_raw_build_header_page();
        return true;
    }
    if (source_page >= s_online_raw.source_record_pages)
    {
        return false;
    }
    return online_raw_build_payload_page(source_page - 1U);
}

static void online_raw_pop_record_packets(void)
{
    if (s_online_raw.record_packet_count > s_online_raw.ring_count)
    {
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_OWNER);
        return;
    }
    s_online_raw.ring_head =
        (s_online_raw.ring_head + s_online_raw.record_packet_count) %
        ZY100_ONLINE_RAW_RING_PACKETS;
    s_online_raw.ring_count -= s_online_raw.record_packet_count;
    s_online_raw.stats.ring_packets = s_online_raw.ring_count;
}

static void online_raw_pop_record_mag(void)
{
    if (s_online_raw.record_mag_count > s_online_raw.mag_ring_count)
    {
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_OWNER);
        return;
    }
    s_online_raw.mag_ring_head =
        (s_online_raw.mag_ring_head + s_online_raw.record_mag_count) %
        ZY100_ONLINE_RAW_MAG_RING_SAMPLES;
    s_online_raw.mag_ring_count -= s_online_raw.record_mag_count;
    s_online_raw.stats.mag_ring_samples = s_online_raw.mag_ring_count;

    if ((s_online_raw.record_mag_read_errors >
         s_online_raw.pending_mag_read_errors) ||
        (s_online_raw.record_mag_missed_deadlines >
         s_online_raw.pending_mag_missed_deadlines))
    {
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_OWNER);
        return;
    }
    s_online_raw.pending_mag_read_errors -=
        s_online_raw.record_mag_read_errors;
    s_online_raw.pending_mag_missed_deadlines -=
        s_online_raw.record_mag_missed_deadlines;
}

static bool online_raw_begin_record(uint32_t packet_count)
{
    const uint8_t *first;
    const uint8_t *last;
    zy100_fe_store_reserve_result_t reserve;
    uint32_t xor_start_us;
    bool xor_profile_valid;

    if ((packet_count > ZY100_FE_RAW_CONTINUOUS_PACKETS_PER_RECORD) ||
        (packet_count > s_online_raw.ring_count))
    {
        return false;
    }
    s_online_raw.record_mag_count = s_online_raw.mag_ring_count;
    if (s_online_raw.record_mag_count >
        ZY100_FE_RAW_MAG_SAMPLES_PER_RECORD_MAX)
    {
        s_online_raw.record_mag_count =
            ZY100_FE_RAW_MAG_SAMPLES_PER_RECORD_MAX;
    }
    s_online_raw.record_mag_read_errors =
        s_online_raw.pending_mag_read_errors;
    s_online_raw.record_mag_missed_deadlines =
        s_online_raw.pending_mag_missed_deadlines;
    if ((packet_count == 0U) &&
        (s_online_raw.record_mag_count == 0U))
    {
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_HEADER);
        return false;
    }
    first = (packet_count != 0U) ?
        online_raw_logical_packet_ptr(0U) : NULL;
    last = (packet_count != 0U) ?
        online_raw_logical_packet_ptr(packet_count - 1U) : NULL;
    if ((packet_count != 0U) && ((first == NULL) || (last == NULL)))
    {
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_OWNER);
        return false;
    }

    s_online_raw.source_id_next++;
    if (s_online_raw.source_id_next == 0U)
    {
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_RESERVE);
        return false;
    }
    s_online_raw.record_packet_count = packet_count;
    s_online_raw.record_start_seq =
        s_online_raw.stats.committed_packets + 1U;
    s_online_raw.record_first_timestamp_raw = (packet_count != 0U) ?
        online_raw_packet_timestamp(first) : 0U;
    s_online_raw.record_last_timestamp_raw = (packet_count != 0U) ?
        online_raw_packet_timestamp(last) : 0U;
    s_online_raw.record_payload_bytes =
        (online_raw_record_has_imu() ?
         ((uint32_t)sizeof(zy100_fe_raw_section_header_t) +
          (packet_count * ZY100_ONLINE_RAW_PACKET_BYTES)) : 0U) +
        (online_raw_record_has_mag() ?
         ((uint32_t)sizeof(zy100_fe_raw_mag_section_header_t) +
          (s_online_raw.record_mag_count *
           ZY100_FE_RAW_MAG_SAMPLE_BYTES)) : 0U);
    s_online_raw.source_record_pages = 1U +
        ((s_online_raw.record_payload_bytes + GD25Q32E_PAGE_BYTES - 1U) /
         GD25Q32E_PAGE_BYTES);
    s_online_raw.source_record_bytes =
        s_online_raw.source_record_pages * GD25Q32E_PAGE_BYTES;
    if ((s_online_raw.source_record_pages >
         ZY100_FE_RAW_IMU_MAG_SOURCE_RECORD_PAGES_MAX) ||
        (s_online_raw.source_record_bytes >
         ZY100_FE_RAW_IMU_MAG_SOURCE_RECORD_BYTES_MAX))
    {
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_HEADER);
        return false;
    }
    xor_profile_valid = online_raw_profile_start(&xor_start_us);
    if (!online_raw_payload_xor(&s_online_raw.record_payload_xor))
    {
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_OWNER);
        return false;
    }
    online_raw_profile_note_elapsed(
        &s_online_raw.stats.payload_xor_max_us,
        xor_start_us,
        xor_profile_valid);

    memset(&s_online_raw.target, 0, sizeof(s_online_raw.target));
    reserve = zy100_online_spool_reserve_raw(
        s_online_raw.source_id_next,
        s_online_raw.source_record_bytes,
        s_online_raw.record_payload_bytes,
        &s_online_raw.target);
    if (reserve != ZY100_FE_STORE_RESERVE_OK)
    {
        s_online_raw.source_id_next--;
        s_online_raw.stats.reserve_waits++;
        if (reserve == ZY100_FE_STORE_RESERVE_ERROR)
        {
            online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_RESERVE);
        }
        else if ((reserve == ZY100_FE_STORE_RESERVE_FULL) &&
                 !s_online_raw.reserve_urgent)
        {
            s_online_raw.reserve_urgent = true;
            s_online_raw.stats.forward_erase_requests++;
        }
        return false;
    }
    if (s_online_raw.reserve_urgent)
    {
        s_online_raw.reserve_urgent = false;
        s_online_raw.stats.urgent_reserve_recoveries++;
    }
    if ((s_online_raw.target.capacity_bytes !=
         s_online_raw.source_record_bytes) ||
        ((s_online_raw.target.data_addr % GD25Q32E_PAGE_BYTES) != 0U))
    {
        zy100_online_spool_abort_reservation(s_online_raw.target.token);
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_RESERVE);
        return false;
    }

    s_online_raw.source_page_next = 0U;
    s_online_raw.source_crc = zy100_crc32_ieee_begin();
    s_online_raw.page_in_flight = false;
    s_online_raw.page_prepared = false;
    s_online_raw.record_write_start_ms =
        ZY100_TARGET_RESOURCE_DIAG_ENABLE ? zy100_os_time_ms() : 0U;
    s_online_raw.record_state = ZY100_ONLINE_RAW_RECORD_WRITE;
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    zy100_stress_diag_record_begin(1U, zy100_os_time_ms(), s_online_raw.source_record_pages);
#endif
    return true;
}

static bool online_raw_begin_record_timed(uint32_t packet_count)
{
    uint32_t prepare_start_us;
    bool prepare_profile_valid =
        online_raw_profile_start(&prepare_start_us);
    bool result = online_raw_begin_record(packet_count);

    online_raw_profile_note_elapsed(
        &s_online_raw.stats.record_prepare_max_us,
        prepare_start_us,
        prepare_profile_valid);
    return result;
}

/* Four reflected IEEE bit steps per lookup; preserve the running CRC state. */
static uint32_t online_raw_crc32_page_update(uint32_t crc,
                                             const uint8_t *page)
{
    static const uint32_t table[16] =
    {
        0x00000000UL, 0x1DB71064UL, 0x3B6E20C8UL, 0x26D930ACUL,
        0x76DC4190UL, 0x6B6B51F4UL, 0x4DB26158UL, 0x5005713CUL,
        0xEDB88320UL, 0xF00F9344UL, 0xD6D6A3E8UL, 0xCB61B38CUL,
        0x9B64C2B0UL, 0x86D3D2D4UL, 0xA00AE278UL, 0xBDBDF21CUL
    };
    uint32_t index;

    if (page == NULL)
    {
        return crc;
    }
    for (index = 0U; index < GD25Q32E_PAGE_BYTES; index++)
    {
        crc ^= page[index];
        crc = (crc >> 4U) ^ table[crc & 0x0FU];
        crc = (crc >> 4U) ^ table[crc & 0x0FU];
    }
    return crc;
}

static bool online_raw_page_step(void)
{
    zy100_online_spool_io_result_t io;
    uint32_t addr;
    uint32_t step_start_us;
    bool step_profile_valid;
    uint16_t diff;

    if (s_online_raw.source_page_next >=
        s_online_raw.source_record_pages)
    {
        return true;
    }
    addr = s_online_raw.target.data_addr +
           (s_online_raw.source_page_next * GD25Q32E_PAGE_BYTES);
    if (s_online_raw.page_in_flight)
    {
        step_profile_valid = online_raw_profile_start(&step_start_us);
        io = zy100_online_spool_read_reserved(s_online_raw.target.token,
                                               addr,
                                               s_online_raw.verify,
                                               GD25Q32E_PAGE_BYTES);
        if (io == ZY100_ONLINE_SPOOL_IO_BUSY)
        {
            return true;
        }
        if (io != ZY100_ONLINE_SPOOL_IO_OK)
        {
            online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_VERIFY);
            return false;
        }
        if (memcmp(s_online_raw.page,
                   s_online_raw.verify,
                   GD25Q32E_PAGE_BYTES) != 0)
        {
            s_online_raw.stats.verify_rereads++;
            io = zy100_online_spool_read_reserved(s_online_raw.target.token,
                                                   addr,
                                                   s_online_raw.verify,
                                                   GD25Q32E_PAGE_BYTES);
            if (io == ZY100_ONLINE_SPOOL_IO_BUSY)
            {
                return true;
            }
            if ((io == ZY100_ONLINE_SPOOL_IO_OK) &&
                (memcmp(s_online_raw.page,
                        s_online_raw.verify,
                        GD25Q32E_PAGE_BYTES) == 0))
            {
                s_online_raw.stats.verify_recovered++;
            }
            else
            {
                diff = (io == ZY100_ONLINE_SPOOL_IO_OK) ?
                       0U : GD25Q32E_PAGE_BYTES;
                if (io == ZY100_ONLINE_SPOOL_IO_OK)
                {
                    while ((diff < GD25Q32E_PAGE_BYTES) &&
                           (s_online_raw.page[diff] ==
                            s_online_raw.verify[diff]))
                    {
                        diff++;
                    }
                }
                s_online_raw.stats.verify_persistent++;
                DBG_DIRECT("[RV]%lu,%lu,%lX,%u,%u,%X,%X",
                           (unsigned long)s_online_raw.source_id_next,
                           (unsigned long)s_online_raw.source_page_next,
                           (unsigned long)addr,
                           (uint32_t)io,
                           (uint32_t)diff,
                           (diff < GD25Q32E_PAGE_BYTES) ?
                               s_online_raw.page[diff] : 0U,
                           (diff < GD25Q32E_PAGE_BYTES) ?
                               s_online_raw.verify[diff] : 0U);
                online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_VERIFY);
                return false;
            }
        }
        online_raw_profile_note_elapsed(
            &s_online_raw.stats.page_verify_max_us,
            step_start_us,
            step_profile_valid);
        s_online_raw.source_crc = online_raw_crc32_page_update(
            s_online_raw.source_crc,
            s_online_raw.page);
        s_online_raw.stats.page_verifies++;
        s_online_raw.source_page_next++;
#if ZY100_ONLINE_STRESS_TEST_ENABLE
        zy100_stress_diag_record_page(s_online_raw.source_page_next);
#endif
        s_online_raw.page_in_flight = false;
        s_online_raw.page_prepared = false;
        if (s_online_raw.source_page_next >=
            s_online_raw.source_record_pages)
        {
            online_raw_pop_record_packets();
            online_raw_pop_record_mag();
            if (s_online_raw.error != ZY100_ONLINE_RAW_ERROR_NONE)
            {
                return false;
            }
            if (!zy100_online_spool_commit_with_crc(
                    s_online_raw.target.token,
                    zy100_crc32_ieee_finish(s_online_raw.source_crc)))
            {
                online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_COMMIT);
                return false;
            }
            s_online_raw.record_state = ZY100_ONLINE_RAW_RECORD_COMMIT;
        }
        return true;
    }

    /* A BUSY retry keeps this page and its source ownership unchanged. */
    if (!s_online_raw.page_prepared)
    {
        step_profile_valid = online_raw_profile_start(&step_start_us);
        if (!online_raw_build_source_page(s_online_raw.source_page_next))
        {
            online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_PROGRAM);
            return false;
        }
        online_raw_profile_note_elapsed(
            &s_online_raw.stats.page_build_max_us,
            step_start_us,
            step_profile_valid);
        s_online_raw.page_prepared = true;
    }
    step_profile_valid = online_raw_profile_start(&step_start_us);
    io = zy100_online_spool_write_reserved_page(s_online_raw.target.token,
                                                 addr,
                                                 s_online_raw.page);
    if (io == ZY100_ONLINE_SPOOL_IO_BUSY)
    {
        return true;
    }
    if (io != ZY100_ONLINE_SPOOL_IO_OK)
    {
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_PROGRAM);
        return false;
    }
    online_raw_profile_note_elapsed(
        &s_online_raw.stats.page_program_max_us,
        step_start_us,
        step_profile_valid);
    s_online_raw.page_in_flight = true;
    s_online_raw.stats.page_programs++;
    return true;
}

static bool online_raw_commit_step(void)
{
    zy100_online_spool_pump_result_t result =
        zy100_online_spool_commit_pump_once(true);

    if (result == ZY100_ONLINE_SPOOL_PUMP_ERROR)
    {
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_COMMIT);
        return false;
    }
    if (result == ZY100_ONLINE_SPOOL_PUMP_COMMITTED)
    {
        uint32_t record_write_ms = ZY100_TARGET_RESOURCE_DIAG_ENABLE ?
            (zy100_os_time_ms() - s_online_raw.record_write_start_ms) : 0U;

#if ZY100_ONLINE_STRESS_TEST_ENABLE
        zy100_stress_diag_record_end(zy100_os_time_ms());
#endif
        s_online_raw.stats.committed_packets +=
            s_online_raw.record_packet_count;
        s_online_raw.stats.committed_mag_samples +=
            s_online_raw.record_mag_count;
        s_online_raw.stats.committed_records++;
        s_online_raw.stats.record_write_total_ms += record_write_ms;
        online_raw_note_max_u32(&s_online_raw.stats.record_write_max_ms,
                                record_write_ms);
        if (s_online_raw.record_packet_count ==
            ZY100_ONLINE_RAW_RECORD_TARGET_PACKETS)
        {
            s_online_raw.stats.normal_records++;
        }
        else if (s_online_raw.record_packet_count >
                 ZY100_ONLINE_RAW_RECORD_TARGET_PACKETS)
        {
            s_online_raw.stats.catchup_records++;
        }
        else if (s_online_raw.record_packet_count != 0U)
        {
            s_online_raw.stats.partial_records++;
        }
        s_online_raw.stats.last_record_id =
            s_online_raw.target.online_record_id;
        s_online_raw.stats.last_source_id = s_online_raw.source_id_next;
        s_online_raw.record_packet_count = 0U;
        s_online_raw.record_mag_count = 0U;
        s_online_raw.record_mag_read_errors = 0U;
        s_online_raw.record_mag_missed_deadlines = 0U;
        s_online_raw.record_state = ZY100_ONLINE_RAW_RECORD_IDLE;
        memset(&s_online_raw.target, 0, sizeof(s_online_raw.target));
    }
    return true;
}

static bool online_raw_wait_flash_idle(
    uint32_t timeout_ms,
    zy100_online_raw_progress_cb_t progress_cb)
{
    uint32_t start_ms = zy100_os_time_ms();
    bool busy = false;
    imu_status_t status;

    do
    {
        status = gd25q32e_is_busy(&busy);
        if (status != IMU_STATUS_OK)
        {
            return false;
        }
        if (!busy)
        {
            return true;
        }
        if (progress_cb != NULL)
        {
            progress_cb();
        }
        if ((uint32_t)(zy100_os_time_ms() - start_ms) >= timeout_ms)
        {
            return false;
        }
        os_delay(1U);
    } while (busy);
    return true;
}

static bool online_raw_flash_wip_finish(
    zy100_online_raw_progress_cb_t progress_cb)
{
    if (zy100_online_spool_erase_wip())
    {
        if (!online_raw_wait_flash_idle(GD25Q32E_BLOCK32_ERASE_TIMEOUT_MS,
                                        progress_cb))
        {
            return false;
        }
        return zy100_online_spool_erase_reclaimable_block32_step() !=
               ZY100_ONLINE_SPOOL_PUMP_ERROR;
    }
    if (s_online_raw.page_in_flight ||
        (s_online_raw.record_state == ZY100_ONLINE_RAW_RECORD_COMMIT))
    {
        return online_raw_wait_flash_idle(GD25Q32E_PAGE_PROGRAM_TIMEOUT_MS,
                                           progress_cb);
    }
    return true;
}

static void online_raw_discard_uncommitted_tail(void)
{
    uint32_t committed = s_online_raw.stats.committed_packets;

    if (s_online_raw.target.token != 0U)
    {
        zy100_online_spool_abort_reservation(s_online_raw.target.token);
    }
    s_online_raw.stats.discarded_packets =
        (s_online_raw.stats.accepted_packets > committed) ?
        (s_online_raw.stats.accepted_packets - committed) : 0U;
    s_online_raw.ring_count = 0U;
    s_online_raw.stats.discarded_mag_samples =
        (s_online_raw.stats.accepted_mag_samples >
         s_online_raw.stats.committed_mag_samples) ?
        (s_online_raw.stats.accepted_mag_samples -
         s_online_raw.stats.committed_mag_samples) : 0U;
    s_online_raw.mag_ring_count = 0U;
    s_online_raw.pending_mag_read_errors = 0U;
    s_online_raw.pending_mag_missed_deadlines = 0U;
    s_online_raw.record_packet_count = 0U;
    s_online_raw.record_mag_count = 0U;
    s_online_raw.record_state = ZY100_ONLINE_RAW_RECORD_IDLE;
    s_online_raw.page_in_flight = false;
    s_online_raw.page_prepared = false;
    memset(&s_online_raw.target, 0, sizeof(s_online_raw.target));
}

static void online_raw_log_finish(void)
{
    uint8_t i;
    uint32_t checkpoint_miss_sum =
        s_online_raw.stats.mag_miss_after_live_service +
        s_online_raw.stats.mag_miss_after_coop_yield +
        s_online_raw.stats.mag_miss_after_notify_wait;
    uint32_t other_task_miss =
        (s_online_raw.stats.mag_miss_task_gap >= checkpoint_miss_sum) ?
        (s_online_raw.stats.mag_miss_task_gap - checkpoint_miss_sum) : 0U;

    DBG_DIRECT("[RAW] acc=%lu commit=%lu discard=%lu rec=%lu part=%lu hi=%lu err=%u",
               (unsigned long)s_online_raw.stats.accepted_packets,
               (unsigned long)s_online_raw.stats.committed_packets,
               (unsigned long)s_online_raw.stats.discarded_packets,
               (unsigned long)s_online_raw.stats.committed_records,
               (unsigned long)s_online_raw.stats.partial_records,
               (unsigned long)s_online_raw.stats.ring_high_water,
               (uint32_t)s_online_raw.stats.error);
    DBG_DIRECT("[R]%lu,%lu,%lu,%lu,%lu,%lu,%lu/%lu/%lu,%lu/%lu/%lu",
               (unsigned long)s_online_raw.stats.page_programs,
               (unsigned long)s_online_raw.stats.page_verifies,
               (unsigned long)s_online_raw.stats.block32_erase_steps,
               (unsigned long)s_online_raw.stats.reserve_waits,
               (unsigned long)s_online_raw.stats.timestamp_errors,
               (unsigned long)s_online_raw.stats.last_record_id,
               (unsigned long)s_online_raw.stats.forward_erase_requests,
               (unsigned long)s_online_raw.stats.forward_erase_blocked,
               (unsigned long)s_online_raw.stats.urgent_reserve_recoveries,
               (unsigned long)s_online_raw.stats.verify_rereads,
               (unsigned long)s_online_raw.stats.verify_recovered,
               (unsigned long)s_online_raw.stats.verify_persistent);
    for (i = 0U; i < s_online_raw.pressure_next; i++)
    {
        const zy100_online_raw_pressure_t *p = &s_online_raw.pressure[i];
        ZY100_LOG_DETAIL("[RP]%u,%u,%u,%u,%u,%u,%u,%lu",
                   p->level, p->ring, p->record_state, p->erase_wip,
                   p->tx_ready, p->tx_in_flight, p->invariant,
                   (unsigned long)p->reserve_waits);
    }
    DBG_DIRECT("[MAG_RAW] acc=%lu commit=%lu discard=%lu read_err=%lu missed=%lu hi=%lu",
               (unsigned long)s_online_raw.stats.accepted_mag_samples,
               (unsigned long)s_online_raw.stats.committed_mag_samples,
               (unsigned long)s_online_raw.stats.discarded_mag_samples,
               (unsigned long)s_online_raw.stats.mag_read_errors,
               (unsigned long)s_online_raw.stats.mag_missed_deadlines,
               (unsigned long)s_online_raw.stats.mag_ring_high_water);
    ZY100_LOG_DETAIL("[RAW_IO] build=%lu prog=%lu verify=%lu rec_avg=%lu rec_max=%lu normal=%lu catchup=%lu mag_late=%lu post=%lu",
               (unsigned long)s_online_raw.stats.page_build_max_us,
               (unsigned long)s_online_raw.stats.page_program_max_us,
               (unsigned long)s_online_raw.stats.page_verify_max_us,
               (unsigned long)((s_online_raw.stats.committed_records != 0U) ?
                   (s_online_raw.stats.record_write_total_ms /
                    s_online_raw.stats.committed_records) : 0U),
               (unsigned long)s_online_raw.stats.record_write_max_ms,
               (unsigned long)s_online_raw.stats.normal_records,
               (unsigned long)s_online_raw.stats.catchup_records,
               (unsigned long)s_online_raw.stats.mag_lateness_max_us,
               (unsigned long)s_online_raw.stats.mag_post_flash_samples);
    ZY100_LOG_DETAIL("[RAW_SCHED] xor=%lu prep=%lu miss=%lu/%lu/%lu/%lu sample=%lu/%lu",
               (unsigned long)s_online_raw.stats.payload_xor_max_us,
               (unsigned long)s_online_raw.stats.record_prepare_max_us,
               (unsigned long)s_online_raw.stats.mag_miss_task_gap,
               (unsigned long)s_online_raw.stats.mag_miss_record_prepare,
               (unsigned long)s_online_raw.stats.mag_miss_flash_io,
               (unsigned long)s_online_raw.stats.mag_miss_ble_staging,
               (unsigned long)s_online_raw.stats.mag_after_flash_samples,
               (unsigned long)s_online_raw.stats.mag_after_staging_samples);
    ZY100_LOG_DETAIL("[RAW_GAP] miss=%lu/%lu/%lu/%lu/%lu gap=%lu/%lu/%lu",
               (unsigned long)s_online_raw.stats.mag_miss_task_gap,
               (unsigned long)s_online_raw.stats.mag_miss_after_live_service,
               (unsigned long)s_online_raw.stats.mag_miss_after_coop_yield,
               (unsigned long)s_online_raw.stats.mag_miss_after_notify_wait,
               (unsigned long)other_task_miss,
               (unsigned long)s_online_raw.stats.mag_gap_after_live_max_us,
               (unsigned long)s_online_raw.stats.mag_gap_after_coop_max_us,
               (unsigned long)s_online_raw.stats.mag_gap_after_notify_max_us);
    ZY100_LOG_DETAIL("[MAG_I2C] total=%lu max=%lu interval=%lu/%lu same=%lu",
               (unsigned long)s_online_raw.stats.mag_read_total_us,
               (unsigned long)s_online_raw.stats.mag_read_max_us,
               (unsigned long)s_online_raw.stats.mag_interval_min_us,
               (unsigned long)s_online_raw.stats.mag_interval_max_us,
               (unsigned long)s_online_raw.stats.mag_adjacent_raw9_same);
    ZY100_LOG_DETAIL("[MAG_GATE] poll=%lu idle=%lu poll_miss=%lu idle_us=%lu/%lu timeout=%lu force=%lu/%lu",
               (unsigned long)s_online_raw.stats.mag_status_poll_count,
               (unsigned long)s_online_raw.stats.mag_not_ready_poll_count,
               (unsigned long)s_online_raw.stats.mag_poll_missed_count,
               (unsigned long)s_online_raw.stats.mag_status_read_total_us,
               (unsigned long)s_online_raw.stats.mag_status_read_max_us,
               (unsigned long)s_online_raw.stats.mag_fresh_timeout_count,
               (unsigned long)s_online_raw.stats.mag_forced_poll_count,
               (unsigned long)s_online_raw.stats.mag_forced_fresh_count);
}

/* Called by the deferred end summary, after the producer has stopped. */
void zy100_online_raw_capture_log_resource(uint8_t stage)
{
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
    const zy100_online_raw_capture_stats_t *p = &s_online_raw.stats;
    switch (stage)
    {
    case 0U:
        ZY100_LOG_EVENT("[RES_RING] imu_hi=%lu imu_cap=%u mag_hi=%lu mag_cap=%u miss=%lu read_err=%lu",
            (unsigned long)p->ring_high_water, (uint32_t)ZY100_ONLINE_RAW_RING_PACKETS,
            (unsigned long)p->mag_ring_high_water, (uint32_t)ZY100_ONLINE_RAW_MAG_RING_SAMPLES,
            (unsigned long)p->mag_missed_deadlines, (unsigned long)p->mag_read_errors);
        break;
    case 1U:
        ZY100_LOG_EVENT("[RES_IO] page_build_us=%lu prog_us=%lu verify_us=%lu prep_us=%lu rec_max_ms=%lu",
            (unsigned long)p->page_build_max_us, (unsigned long)p->page_program_max_us,
            (unsigned long)p->page_verify_max_us, (unsigned long)p->record_prepare_max_us,
            (unsigned long)p->record_write_max_ms);
        break;
    case 2U:
        ZY100_LOG_EVENT("[RES_MAG] read_max_us=%lu interval_max_us=%lu late_us=%lu gap_us=%lu/%lu/%lu",
            (unsigned long)p->mag_read_max_us, (unsigned long)p->mag_interval_max_us,
            (unsigned long)p->mag_lateness_max_us, (unsigned long)p->mag_gap_after_live_max_us,
            (unsigned long)p->mag_gap_after_coop_max_us, (unsigned long)p->mag_gap_after_notify_max_us);
        break;
    default:
        break;
    }
#else
    (void)stage;
#endif
}

bool zy100_online_raw_capture_prepare_mag(void)
{
    mmc5603_cfg_t cfg;

    if (s_online_raw.active)
    {
        return false;
    }
    if (s_online_mag_prepared)
    {
        if (mag_capture_service_end(MAG_CAPTURE_OWNER_ONLINE) != MAG_STATUS_OK)
        {
            return false;
        }
        s_online_mag_prepared = false;
        (void)zy100_capture_profile_release(
            ZY100_CAPTURE_PROFILE_ONLINE);
    }
    if (!zy100_capture_profile_claim(ZY100_CAPTURE_PROFILE_ONLINE))
    {
        return false;
    }

    cfg.auto_sr_enable = true;
    cfg.bw = MMC5603_BW_LEVEL_01;
    cfg.continuous_odr = 0U;
    cfg.continuous_hpower = false;
    if (mag_capture_service_begin(MAG_CAPTURE_OWNER_ONLINE,
                                  &cfg,
                                  ZY100_FE_RAW_MAG_SAMPLE_HZ,
                                  false) != MAG_STATUS_OK)
    {
        (void)zy100_capture_profile_release(
            ZY100_CAPTURE_PROFILE_ONLINE);
        return false;
    }
    s_online_mag_prepared = true;
    return true;
}

void zy100_online_raw_capture_release_mag(void)
{
    if (s_online_mag_prepared)
    {
        mag_status_t status =
            mag_capture_service_end(MAG_CAPTURE_OWNER_ONLINE);

        if (status == MAG_STATUS_OK)
        {
            s_online_mag_prepared = false;
            (void)zy100_capture_profile_release(
                ZY100_CAPTURE_PROFILE_ONLINE);
            ZY100_LOG_ROUTINE(DBG_DIRECT, "[ONLINE_MAG] power_down_ok");
        }
        else
        {
            DBG_DIRECT("[ONLINE_MAG][ERR] power_down_failed status=%u",
                       (uint32_t)status);
        }
    }
}

static void online_raw_note_checkpoint_gap(
    zy100_online_raw_mag_phase_t phase,
    uint32_t elapsed_us)
{
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
    uint32_t gap_us;

    if ((phase != ZY100_ONLINE_RAW_MAG_PHASE_LOOP_ENTRY) &&
        (phase != ZY100_ONLINE_RAW_MAG_PHASE_AFTER_LIVE_SERVICE) &&
        (phase != ZY100_ONLINE_RAW_MAG_PHASE_AFTER_COOP_YIELD) &&
        (phase != ZY100_ONLINE_RAW_MAG_PHASE_AFTER_NOTIFY_WAIT))
    {
        return;
    }
    if (s_online_raw.mag_checkpoint_valid)
    {
        gap_us = elapsed_us -
                 s_online_raw.mag_checkpoint_last_elapsed_us;
        if (phase == ZY100_ONLINE_RAW_MAG_PHASE_AFTER_LIVE_SERVICE)
        {
            online_raw_note_max_u32(
                &s_online_raw.stats.mag_gap_after_live_max_us,
                gap_us);
        }
        else if (phase == ZY100_ONLINE_RAW_MAG_PHASE_AFTER_COOP_YIELD)
        {
            online_raw_note_max_u32(
                &s_online_raw.stats.mag_gap_after_coop_max_us,
                gap_us);
        }
        else if (phase == ZY100_ONLINE_RAW_MAG_PHASE_AFTER_NOTIFY_WAIT)
        {
            online_raw_note_max_u32(
                &s_online_raw.stats.mag_gap_after_notify_max_us,
                gap_us);
        }
    }
    s_online_raw.mag_checkpoint_last_elapsed_us = elapsed_us;
    s_online_raw.mag_checkpoint_valid = true;
#else
    (void)phase;
    (void)elapsed_us;
#endif
}

static bool online_raw_mag_schedule_step(zy100_online_raw_mag_phase_t phase)
{
    mag_capture_sample_t sample;
    zy100_fe_raw_mag_sample_t *dst;
    mag_status_t mag_status;
    uint32_t counter;
    uint32_t elapsed_us;
    uint32_t lateness_us;
    uint32_t missed;
    uint32_t poll_missed;
    uint32_t tail;
    uint32_t previous_tail;
    uint32_t interval_us;
    uint32_t read_start_counter;
    uint32_t read_end_counter;
    uint32_t read_elapsed_us;
    bool read_profile_valid;
    bool fresh;
    bool force_poll =
        (phase == ZY100_ONLINE_RAW_MAG_PHASE_BEFORE_COOP_YIELD_FORCE);
    bool poll_due;

    if (!s_online_raw.mag_scheduling ||
        (s_online_raw.clock_meta.first_fixed40_valid == 0U))
    {
        return true;
    }
    if (zy100_online_stream_end_requested())
    {
        s_online_raw.mag_scheduling = false;
        return true;
    }
    if (!bsp_capture_timebase_snapshot(&counter))
    {
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_TIMESTAMP);
        return false;
    }
    elapsed_us = s_online_raw.clock_meta.first_fixed40_counter - counter;
    online_raw_note_checkpoint_gap(phase, elapsed_us);

    /* The magnetometer starts before the first IMU Fixed40 anchor.  Consume
     * one already-complete conversion without publishing it so every stored
     * sample is known to have completed after the session anchor. */
    if (!s_online_raw.mag_gate_primed)
    {
        read_profile_valid = online_raw_profile_start(&read_start_counter);
        fresh = false;
        mag_status = mag_capture_service_read_fresh(false, &sample, &fresh);
        read_elapsed_us = 0U;
        if (read_profile_valid &&
            bsp_capture_timebase_snapshot(&read_end_counter))
        {
            read_elapsed_us = read_start_counter - read_end_counter;
        }
        s_online_raw.stats.mag_status_poll_count++;
        if (force_poll)
        {
            s_online_raw.stats.mag_forced_poll_count++;
        }
        if (mag_status != MAG_STATUS_OK)
        {
            if (read_elapsed_us != 0U)
            {
                s_online_raw.stats.mag_read_total_us += read_elapsed_us;
                online_raw_note_max_u32(
                    &s_online_raw.stats.mag_read_max_us,
                    read_elapsed_us);
            }
            s_online_raw.pending_mag_read_errors++;
            s_online_raw.stats.mag_read_errors++;
            if (s_online_raw.mag_consecutive_errors < 0xFFU)
            {
                s_online_raw.mag_consecutive_errors++;
            }
            if (s_online_raw.mag_consecutive_errors >=
                ZY100_ONLINE_RAW_MAG_CONSEC_ERROR_MAX)
            {
                online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_MAG_READ);
                return false;
            }
            return true;
        }
        s_online_raw.mag_consecutive_errors = 0U;
        if (fresh)
        {
            s_online_raw.stats.mag_read_total_us += read_elapsed_us;
            online_raw_note_max_u32(&s_online_raw.stats.mag_read_max_us,
                                    read_elapsed_us);
        }
        else
        {
            s_online_raw.stats.mag_not_ready_poll_count++;
            s_online_raw.stats.mag_status_read_total_us += read_elapsed_us;
            online_raw_note_max_u32(
                &s_online_raw.stats.mag_status_read_max_us,
                read_elapsed_us);
        }
        s_online_raw.mag_gate_primed = true;
        s_online_raw.mag_last_fresh_elapsed_us = elapsed_us;
        s_online_raw.mag_next_poll_deadline_us =
            ((elapsed_us / ZY100_ONLINE_RAW_MAG_POLL_PERIOD_US) + 1U) *
            ZY100_ONLINE_RAW_MAG_POLL_PERIOD_US;
        s_online_raw.mag_next_sample_deadline_us =
            ZY100_ONLINE_RAW_MAG_PERIOD_US;
        s_online_raw.mag_deadline_started = true;
        return true;
    }
    if (!s_online_raw.mag_deadline_started)
    {
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_OWNER);
        return false;
    }
    poll_due = ((int32_t)(elapsed_us -
                          s_online_raw.mag_next_poll_deadline_us) >= 0);
    if (!poll_due && !force_poll)
    {
        return true;
    }

    missed = 0U;
    if (poll_due)
    {
        lateness_us = elapsed_us - s_online_raw.mag_next_poll_deadline_us;
        online_raw_note_max_u32(&s_online_raw.stats.mag_lateness_max_us,
                                lateness_us);
        poll_missed = lateness_us / ZY100_ONLINE_RAW_MAG_POLL_PERIOD_US;
        s_online_raw.mag_next_poll_deadline_us +=
            (poll_missed + 1U) * ZY100_ONLINE_RAW_MAG_POLL_PERIOD_US;
        s_online_raw.stats.mag_poll_missed_count += poll_missed;

        if ((int32_t)(elapsed_us -
                      s_online_raw.mag_next_sample_deadline_us) >= 0)
        {
            lateness_us = elapsed_us -
                          s_online_raw.mag_next_sample_deadline_us;
            missed = lateness_us / ZY100_ONLINE_RAW_MAG_PERIOD_US;
            s_online_raw.mag_next_sample_deadline_us +=
                (missed + 1U) * ZY100_ONLINE_RAW_MAG_PERIOD_US;
            s_online_raw.pending_mag_missed_deadlines += missed;
            s_online_raw.stats.mag_missed_deadlines += missed;
            if (phase == ZY100_ONLINE_RAW_MAG_PHASE_RECORD_PREPARE)
            {
                s_online_raw.stats.mag_miss_record_prepare += missed;
            }
            else if (phase == ZY100_ONLINE_RAW_MAG_PHASE_FLASH_IO)
            {
                s_online_raw.stats.mag_miss_flash_io += missed;
            }
            else if (phase == ZY100_ONLINE_RAW_MAG_PHASE_BLE_STAGING)
            {
                s_online_raw.stats.mag_miss_ble_staging += missed;
            }
            else
            {
                s_online_raw.stats.mag_miss_task_gap += missed;
                if (phase == ZY100_ONLINE_RAW_MAG_PHASE_AFTER_LIVE_SERVICE)
                {
                    s_online_raw.stats.mag_miss_after_live_service += missed;
                }
                else if (phase == ZY100_ONLINE_RAW_MAG_PHASE_AFTER_COOP_YIELD)
                {
                    s_online_raw.stats.mag_miss_after_coop_yield += missed;
                }
                else if (phase == ZY100_ONLINE_RAW_MAG_PHASE_AFTER_NOTIFY_WAIT)
                {
                    s_online_raw.stats.mag_miss_after_notify_wait += missed;
                }
            }
        }
    }

    read_profile_valid = online_raw_profile_start(&read_start_counter);
    fresh = false;
    mag_status = mag_capture_service_read_fresh(false, &sample, &fresh);
    read_elapsed_us = 0U;
    if (read_profile_valid &&
        bsp_capture_timebase_snapshot(&read_end_counter))
    {
        read_elapsed_us = read_start_counter - read_end_counter;
    }
    s_online_raw.stats.mag_status_poll_count++;
    if (force_poll)
    {
        s_online_raw.stats.mag_forced_poll_count++;
    }
    if (mag_status != MAG_STATUS_OK)
    {
        if (read_elapsed_us != 0U)
        {
            s_online_raw.stats.mag_read_total_us += read_elapsed_us;
            online_raw_note_max_u32(
                &s_online_raw.stats.mag_read_max_us,
                read_elapsed_us);
        }
        s_online_raw.pending_mag_read_errors++;
        s_online_raw.stats.mag_read_errors++;
        if (s_online_raw.mag_consecutive_errors < 0xFFU)
        {
            s_online_raw.mag_consecutive_errors++;
        }
        if (s_online_raw.mag_consecutive_errors >=
            ZY100_ONLINE_RAW_MAG_CONSEC_ERROR_MAX)
        {
            online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_MAG_READ);
            return false;
        }
        return true;
    }
    s_online_raw.mag_consecutive_errors = 0U;
    if (!fresh)
    {
        s_online_raw.stats.mag_not_ready_poll_count++;
        s_online_raw.stats.mag_status_read_total_us += read_elapsed_us;
        online_raw_note_max_u32(&s_online_raw.stats.mag_status_read_max_us,
                                read_elapsed_us);
        if ((elapsed_us - s_online_raw.mag_last_fresh_elapsed_us) >=
            ZY100_ONLINE_RAW_MAG_FRESH_TIMEOUT_US)
        {
            s_online_raw.stats.mag_fresh_timeout_count++;
            online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_MAG_STALE);
            return false;
        }
        return true;
    }
    s_online_raw.stats.mag_read_total_us += read_elapsed_us;
    online_raw_note_max_u32(&s_online_raw.stats.mag_read_max_us,
                            read_elapsed_us);
    if ((s_online_raw.stats.accepted_mag_samples != 0U) &&
        (elapsed_us == s_online_raw.mag_last_elapsed_us))
    {
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_TIMESTAMP);
        return false;
    }
    if (s_online_raw.mag_ring_count >= ZY100_ONLINE_RAW_MAG_RING_SAMPLES)
    {
        DBG_DIRECT("[MAG_RAW][ERR] overflow ring=%lu cap=%u state=%u imu=%lu rec=%lu/%lu page=%lu/%lu",
                   (unsigned long)s_online_raw.mag_ring_count,
                   (uint32_t)ZY100_ONLINE_RAW_MAG_RING_SAMPLES,
                   (uint32_t)s_online_raw.record_state,
                   (unsigned long)s_online_raw.ring_count,
                   (unsigned long)s_online_raw.record_packet_count,
                   (unsigned long)s_online_raw.record_mag_count,
                   (unsigned long)s_online_raw.source_page_next,
                   (unsigned long)s_online_raw.source_record_pages);
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_OVERFLOW);
        return false;
    }
    tail = (s_online_raw.mag_ring_head + s_online_raw.mag_ring_count) %
           ZY100_ONLINE_RAW_MAG_RING_SAMPLES;
    if (ZY100_TARGET_RESOURCE_DIAG_ENABLE &&
        (s_online_raw.stats.accepted_mag_samples != 0U))
    {
        interval_us = elapsed_us - s_online_raw.mag_last_elapsed_us;
        if ((s_online_raw.stats.mag_interval_min_us == 0U) ||
            (interval_us < s_online_raw.stats.mag_interval_min_us))
        {
            s_online_raw.stats.mag_interval_min_us = interval_us;
        }
        online_raw_note_max_u32(
            &s_online_raw.stats.mag_interval_max_us,
            interval_us);
        previous_tail = (tail != 0U) ?
                        (tail - 1U) :
                        (ZY100_ONLINE_RAW_MAG_RING_SAMPLES - 1U);
        if (memcmp(s_online_raw_mag_ring[previous_tail].raw9,
                   sample.raw9,
                   sizeof(sample.raw9)) == 0)
        {
            s_online_raw.stats.mag_adjacent_raw9_same++;
        }
    }
    dst = &s_online_raw_mag_ring[tail];
    dst->elapsed_us = elapsed_us;
    memcpy(dst->raw9, sample.raw9, sizeof(dst->raw9));
    s_online_raw.mag_last_elapsed_us = elapsed_us;
    s_online_raw.mag_last_fresh_elapsed_us = elapsed_us;
    s_online_raw.mag_ring_count++;
    s_online_raw.stats.accepted_mag_samples++;
    if (force_poll)
    {
        s_online_raw.stats.mag_forced_fresh_count++;
    }
    if (phase == ZY100_ONLINE_RAW_MAG_PHASE_FLASH_IO)
    {
        s_online_raw.stats.mag_post_flash_samples++;
        s_online_raw.stats.mag_after_flash_samples++;
    }
    else if (phase == ZY100_ONLINE_RAW_MAG_PHASE_BLE_STAGING)
    {
        s_online_raw.stats.mag_after_staging_samples++;
    }
    s_online_raw.stats.mag_ring_samples = s_online_raw.mag_ring_count;
    if (ZY100_TARGET_RESOURCE_DIAG_ENABLE &&
        (s_online_raw.mag_ring_count > s_online_raw.stats.mag_ring_high_water))
    {
        s_online_raw.stats.mag_ring_high_water =
            s_online_raw.mag_ring_count;
    }
    return true;
}

bool zy100_online_raw_capture_mag_checkpoint(
    zy100_online_raw_mag_checkpoint_t checkpoint)
{
    zy100_online_raw_mag_phase_t phase;
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
    uint32_t counter;
#endif
    bool result;

    if (!s_online_raw.active)
    {
        return true;
    }
    switch (checkpoint)
    {
    case ZY100_ONLINE_RAW_MAG_CHECKPOINT_LOOP_ENTRY:
        phase = ZY100_ONLINE_RAW_MAG_PHASE_LOOP_ENTRY;
        break;
    case ZY100_ONLINE_RAW_MAG_CHECKPOINT_AFTER_LIVE_SERVICE:
        phase = ZY100_ONLINE_RAW_MAG_PHASE_AFTER_LIVE_SERVICE;
        break;
    case ZY100_ONLINE_RAW_MAG_CHECKPOINT_BEFORE_COOP_YIELD_FORCE:
        phase = ZY100_ONLINE_RAW_MAG_PHASE_BEFORE_COOP_YIELD_FORCE;
        break;
    case ZY100_ONLINE_RAW_MAG_CHECKPOINT_AFTER_COOP_YIELD:
        phase = ZY100_ONLINE_RAW_MAG_PHASE_AFTER_COOP_YIELD;
        break;
    case ZY100_ONLINE_RAW_MAG_CHECKPOINT_AFTER_NOTIFY_WAIT:
        phase = ZY100_ONLINE_RAW_MAG_PHASE_AFTER_NOTIFY_WAIT;
        break;
    default:
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_OWNER);
        return false;
    }
    result = online_raw_mag_schedule_step(phase);
#if ZY100_TARGET_RESOURCE_DIAG_ENABLE
    /* End the gap at checkpoint completion so the next phase does not count
     * the I2C transaction performed by this checkpoint as its own work. */
    if ((s_online_raw.clock_meta.first_fixed40_valid != 0U) &&
        bsp_capture_timebase_snapshot(&counter))
    {
        s_online_raw.mag_checkpoint_last_elapsed_us =
            s_online_raw.clock_meta.first_fixed40_counter - counter;
        s_online_raw.mag_checkpoint_valid = true;
    }
#endif
    return result;
}

static uint32_t online_raw_select_record_packet_count(bool urgent)
{
    uint32_t limit = ZY100_ONLINE_RAW_RECORD_TARGET_PACKETS;
    uint32_t count = s_online_raw.ring_count;

    if (s_online_raw.finishing || urgent ||
        (count >= ZY100_ONLINE_RAW_CATCHUP_HIGH_PACKETS))
    {
        limit = ZY100_FE_RAW_CONTINUOUS_PACKETS_PER_RECORD;
    }
    if (count > limit)
    {
        count = limit;
    }
    return count;
}

static void online_raw_release_workspace(void)
{
    uint32_t token = s_online_raw.workspace_token;
    uint32_t i;
    s_online_raw.workspace_token = 0U;
    s_online_raw.page_prepared = false;
    s_online_raw.page = NULL;
    s_online_raw.verify = NULL;
    s_online_raw_mag_ring = NULL;
    for (i = 0U; i < ZY100_ONLINE_RAW_SPAN_COUNT; i++)
    {
        s_online_raw.span[i].base = NULL;
        s_online_raw.span[i].bytes = 0U;
    }
    if (token != 0U)
    {
        (void)zy100_mode_workspace_online_release(token,
            ZY100_ONLINE_WORKSPACE_CAPTURE);
    }
}

bool zy100_online_raw_capture_begin(uint32_t source_id_seed)
{
    uint8_t *workspace = NULL;

    if (s_online_raw.active || !s_online_mag_prepared)
    {
        return false;
    }
    memset(&s_online_raw, 0, sizeof(s_online_raw));
    if (!zy100_mode_workspace_online_join(
            &s_online_raw.workspace_token, &workspace))
    {
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_OWNER);
        zy100_online_raw_capture_release_mag();
        return false;
    }
    if (gd25q32e_init() != IMU_STATUS_OK)
    {
        online_raw_release_workspace();
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_FLASH);
        zy100_online_raw_capture_release_mag();
        return false;
    }
    s_online_raw.page = workspace + ZY100_ONLINE_WORKSPACE_PAGE_OFFSET;
    s_online_raw.verify = workspace + ZY100_ONLINE_WORKSPACE_VERIFY_OFFSET;
    s_online_raw.span[0].base = workspace;
    s_online_raw.span[0].bytes = ZY100_ONLINE_RAW_IMU_RING_BYTES;
    s_online_raw.span[1].base = NULL;
    s_online_raw.span[1].bytes = 0U;
    s_online_raw.span[2].base = NULL;
    s_online_raw.span[2].bytes = 0U;
    s_online_raw_mag_ring = (zy100_fe_raw_mag_sample_t *)(
        workspace + ZY100_ONLINE_RAW_IMU_RING_BYTES);
    s_online_raw.source_id_next = source_id_seed;
    s_online_raw.use_fixed40_endpoint =
        zy100_online_stream_fixed40_endpoint_enabled();
    s_online_raw.clock_meta.fixed40_endpoint =
        s_online_raw.use_fixed40_endpoint ? 1U : 0U;
    if (s_online_raw.use_fixed40_endpoint)
    {
        s_online_raw.fixed40_configured = bsp_capture_timebase_init();
    }
    if (s_online_raw.use_fixed40_endpoint &&
        !s_online_raw.fixed40_configured)
    {
        online_raw_release_workspace();
        s_online_raw_mag_ring = NULL;
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_TIMESTAMP);
        zy100_online_raw_capture_release_mag();
        return false;
    }
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    if (!zy100_online_stress_begin(workspace, zy100_os_time_ms()))
    {
        online_raw_release_workspace();
        zy100_online_raw_capture_release_mag();
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_STRESS);
        return false;
    }
#endif
    s_online_raw.mag_scheduling = true;
    s_online_raw.active = true;
    return true;
}

bool zy100_online_raw_capture_active(void)
{
    return s_online_raw.active;
}

bool zy100_online_raw_capture_push_packet(const uint8_t packet[16],
                                          uint16_t timestamp_raw)
{
    uint8_t *dst;
    uint32_t tail;

    if (!s_online_raw.active || s_online_raw.finishing || (packet == NULL) ||
        (s_online_raw.error != ZY100_ONLINE_RAW_ERROR_NONE))
    {
        return false;
    }
    if (s_online_raw.ring_count >= ZY100_ONLINE_RAW_RING_PACKETS)
    {
        if (zy100_online_spool_last_reserve_block() ==
            ZY100_ONLINE_SPOOL_RESERVE_BLOCK_UNACKED)
        {
            DBG_DIRECT("[RAW_BACKPRESSURE] host_ack_throughput_insufficient ring=%lu waits=%lu",
                       (unsigned long)s_online_raw.ring_count,
                       (unsigned long)s_online_raw.stats.reserve_waits);
            online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_BACKPRESSURE);
        }
        else
        {
            online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_OVERFLOW);
        }
        return false;
    }
    tail = (s_online_raw.ring_head + s_online_raw.ring_count) %
           ZY100_ONLINE_RAW_RING_PACKETS;
    dst = online_raw_packet_ptr(tail);
    if (dst == NULL)
    {
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_OWNER);
        return false;
    }
    memcpy(dst, packet, ZY100_ONLINE_RAW_PACKET_BYTES);
    s_online_raw.ring_count++;
    if (s_online_raw.stats.accepted_packets == 0U)
    {
        s_online_raw.first_timestamp_raw = timestamp_raw;
    }
    s_online_raw.stats.accepted_packets++;
    s_online_raw.stats.ring_packets = s_online_raw.ring_count;
    if (ZY100_TARGET_RESOURCE_DIAG_ENABLE &&
        (s_online_raw.ring_count > s_online_raw.stats.ring_high_water))
    {
        s_online_raw.stats.ring_high_water = s_online_raw.ring_count;
    }
    if (s_online_raw.pressure_next < 4U)
    {
        uint32_t threshold;
        uint8_t level = (uint8_t)(s_online_raw.pressure_next + 1U);

        threshold = (level == 1U) ? (ZY100_ONLINE_RAW_RING_PACKETS / 2U) :
                    (level == 2U) ? ((ZY100_ONLINE_RAW_RING_PACKETS * 3U) / 4U) :
                    (level == 3U) ? ((ZY100_ONLINE_RAW_RING_PACKETS * 9U) / 10U) :
                                    ((ZY100_ONLINE_RAW_RING_PACKETS * 95U) / 100U);
        if (s_online_raw.ring_count >= threshold)
        {
            zy100_online_raw_pressure_t *p =
                &s_online_raw.pressure[s_online_raw.pressure_next++];

            p->ring = (uint16_t)s_online_raw.ring_count;
            p->level = level;
            p->record_state = s_online_raw.record_state;
            p->erase_wip = zy100_online_spool_erase_wip() ? 1U : 0U;
            zy100_online_stream_get_tx_pressure(&p->tx_ready,
                                                 &p->tx_in_flight);
            p->invariant = zy100_online_spool_invariant_code();
            p->reserve_waits = s_online_raw.stats.reserve_waits;
        }
    }
    s_online_raw.timestamp_valid = true;
    s_online_raw.last_timestamp_raw = timestamp_raw;
    s_online_raw.stats.last_timestamp_raw = timestamp_raw;
    return true;
}

bool zy100_online_raw_capture_snapshot_fifo_read(
    uint32_t accepted_before,
    zy100_online_raw_fifo_time_snapshot_t *snapshot)
{
    if (!s_online_raw.active || (snapshot == NULL))
    {
        return false;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    if (s_online_raw.use_fixed40_endpoint)
    {
        if (accepted_before == 0U)
        {
            snapshot->rtc_valid =
                zy100_rtc_clock_raw_snapshot(&snapshot->rtc) ? 1U : 0U;
            if (s_online_raw.fixed40_configured)
            {
                snapshot->fixed40_valid =
                    bsp_capture_timebase_start(
                        &snapshot->fixed40_counter) ? 1U : 0U;
            }
        }
        else
        {
            snapshot->fixed40_valid =
                bsp_capture_timebase_snapshot(
                    &snapshot->fixed40_counter) ? 1U : 0U;
        }
        return true;
    }

    snapshot->rtc_valid =
        zy100_rtc_clock_raw_snapshot(&snapshot->rtc) ? 1U : 0U;
    return true;
}

bool zy100_online_raw_capture_note_fifo_burst(
    uint32_t accepted_before,
    uint32_t expected_packets,
    const zy100_online_raw_fifo_time_snapshot_t *snapshot)
{
    uint32_t accepted_after;
    uint32_t fixed40_delta_us;
    uint32_t fixed40_now_os_ms;
    uint64_t delta_ticks;
    const zy100_rtc_raw_snapshot_t *rtc_snapshot;

    if (!s_online_raw.active || (expected_packets == 0U) ||
        (snapshot == NULL))
    {
        return false;
    }
    accepted_after = s_online_raw.stats.accepted_packets;
    if ((accepted_after < accepted_before) ||
        ((accepted_after - accepted_before) != expected_packets))
    {
        return false;
    }

    s_online_raw.clock_meta.accepted_packets = accepted_after;
    s_online_raw.clock_meta.first_timestamp_raw =
        s_online_raw.first_timestamp_raw;
    s_online_raw.clock_meta.last_timestamp_raw =
        s_online_raw.last_timestamp_raw;

    if (s_online_raw.use_fixed40_endpoint)
    {
        if (accepted_before == 0U)
        {
            s_online_raw.clock_meta.first_fixed40_counter =
                snapshot->fixed40_counter;
            s_online_raw.clock_meta.first_fixed40_valid =
                snapshot->fixed40_valid;
            s_online_raw.fixed40_last_os_ms = zy100_os_time_ms();
            s_online_raw.fixed40_committed_counter =
                snapshot->fixed40_counter;
            s_online_raw.fixed40_elapsed_us = 0ULL;
            if ((snapshot->rtc_valid != 0U) &&
                (snapshot->rtc.unix_time_valid != 0U) &&
                (snapshot->rtc.unix_time_ms <=
                 (0xFFFFFFFFFFFFFFFFULL / 1000ULL)))
            {
                s_online_raw.clock_meta.first_unix_time_us =
                    snapshot->rtc.unix_time_ms * 1000ULL;
                s_online_raw.clock_meta.first_unix_valid = 1U;
            }
        }
        else if (snapshot->fixed40_valid != 0U)
        {
            fixed40_now_os_ms = zy100_os_time_ms();
            if ((uint32_t)(fixed40_now_os_ms -
                           s_online_raw.fixed40_last_os_ms) >=
                ZY100_ONLINE_RAW_FIXED40_RANGE_GUARD_MS)
            {
                s_online_raw.clock_meta.fixed40_range_unsupported = 1U;
            }
            fixed40_delta_us = s_online_raw.fixed40_committed_counter -
                               snapshot->fixed40_counter;
            if (s_online_raw.fixed40_elapsed_us >
                (0xFFFFFFFFFFFFFFFFULL - (uint64_t)fixed40_delta_us))
            {
                s_online_raw.clock_meta.fixed40_range_unsupported = 1U;
            }
            else
            {
                s_online_raw.fixed40_elapsed_us +=
                    (uint64_t)fixed40_delta_us;
            }
            s_online_raw.fixed40_committed_counter =
                snapshot->fixed40_counter;
            s_online_raw.fixed40_last_os_ms = fixed40_now_os_ms;
        }
        s_online_raw.clock_meta.last_fixed40_counter =
            snapshot->fixed40_counter;
        s_online_raw.clock_meta.last_fixed40_valid =
            snapshot->fixed40_valid;
        return true;
    }

    rtc_snapshot = &snapshot->rtc;
    if ((snapshot->rtc_valid == 0U) ||
        (rtc_snapshot->nominal_tick_hz == 0U) ||
        (rtc_snapshot->wrap_ticks == 0ULL))
    {
        s_online_raw.clock_meta.last_rtc_valid = 0U;
        return true;
    }

    if (accepted_before == 0U)
    {
        s_online_raw.clock_meta.first_rtc_tick = rtc_snapshot->ticks;
        s_online_raw.clock_meta.first_rtc_valid = 1U;
        if (rtc_snapshot->unix_time_valid != 0U)
        {
            s_online_raw.clock_meta.first_unix_time_ms =
                rtc_snapshot->unix_time_ms;
            s_online_raw.clock_meta.last_unix_time_ms =
                rtc_snapshot->unix_time_ms;
            s_online_raw.clock_meta.first_unix_valid = 1U;
            s_online_raw.clock_meta.last_unix_valid = 1U;
        }
    }
    s_online_raw.clock_meta.last_rtc_tick = rtc_snapshot->ticks;
    s_online_raw.clock_meta.last_rtc_valid = 1U;
    s_online_raw.clock_meta.nominal_tick_hz =
        rtc_snapshot->nominal_tick_hz;
    s_online_raw.clock_meta.rtc_wrap_ticks = rtc_snapshot->wrap_ticks;
    if ((s_online_raw.clock_meta.first_rtc_valid != 0U) &&
        (s_online_raw.clock_meta.first_unix_valid != 0U))
    {
        delta_ticks =
            (rtc_snapshot->ticks >=
             s_online_raw.clock_meta.first_rtc_tick) ?
            (rtc_snapshot->ticks -
             s_online_raw.clock_meta.first_rtc_tick) :
            ((rtc_snapshot->wrap_ticks -
              s_online_raw.clock_meta.first_rtc_tick) +
             rtc_snapshot->ticks);
        if (delta_ticks < rtc_snapshot->wrap_ticks)
        {
            s_online_raw.clock_meta.last_unix_time_ms =
                s_online_raw.clock_meta.first_unix_time_ms +
                ((delta_ticks * 1000ULL) /
                 (uint64_t)rtc_snapshot->nominal_tick_hz);
            s_online_raw.clock_meta.last_unix_valid = 1U;
        }
        else
        {
            s_online_raw.clock_meta.last_unix_valid = 0U;
        }
    }
    return true;
}

#if ZY100_ONLINE_ERASE_AHEAD_ENABLE
static bool online_raw_background_erase_due(void)
{
    uint32_t ahead = zy100_online_spool_erased_ahead_bytes();
    if (ahead < ZY100_ONLINE_SPOOL_PREERASE_MIN_BYTES) { s_online_raw.erase_refill = true; }
    if (ahead >= ZY100_ONLINE_ERASE_HIGH_BYTES) { s_online_raw.erase_refill = false; }
    return s_online_raw.erase_refill && zy100_online_stress_erase_window();
}
#endif

bool zy100_online_raw_capture_pump(uint32_t now_ms)
{
    zy100_online_spool_erase_result_t forward_result;
    zy100_online_raw_mag_phase_t phase =
        ZY100_ONLINE_RAW_MAG_PHASE_TASK_GAP;

    if (!s_online_raw.active)
    {
        return false;
    }
    if (s_online_raw.error != ZY100_ONLINE_RAW_ERROR_NONE)
    {
        return false;
    }
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    if (!zy100_online_stress_tick(now_ms))
    {
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_STRESS);
        return false;
    }
#endif
    if (!online_raw_mag_schedule_step(ZY100_ONLINE_RAW_MAG_PHASE_TASK_GAP))
    {
        return false;
    }
    if (zy100_online_spool_erase_wip())
    {
#if ZY100_ONLINE_STRESS_TEST_ENABLE
        zy100_online_stress_diag_stage(ZY100_SD_ERASE);
#endif
        phase = ZY100_ONLINE_RAW_MAG_PHASE_FLASH_IO;
        #if ZY100_ONLINE_ERASE_AHEAD_ENABLE
        forward_result = zy100_online_spool_erase_forward_to(ZY100_ONLINE_SPOOL_PREERASE_MIN_BYTES);
#else
        forward_result = zy100_online_spool_erase_forward_block32_step();
#endif
        s_online_raw.stats.block32_erase_steps +=
            (forward_result == ZY100_ONLINE_SPOOL_ERASE_PROGRESS) ? 1U : 0U;
        if (forward_result == ZY100_ONLINE_SPOOL_ERASE_ERROR)
        {
            online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_FLASH);
            return false;
        }
        goto pump_tail;
    }
    if (s_online_raw.record_state == ZY100_ONLINE_RAW_RECORD_WRITE)
    {
#if ZY100_ONLINE_STRESS_TEST_ENABLE
        zy100_online_stress_diag_stage(ZY100_SD_RAW_WRITE);
#endif
        phase = ZY100_ONLINE_RAW_MAG_PHASE_FLASH_IO;
        if (!online_raw_page_step())
        {
            return false;
        }
    }
    else if (s_online_raw.record_state == ZY100_ONLINE_RAW_RECORD_COMMIT)
    {
#if ZY100_ONLINE_STRESS_TEST_ENABLE
        zy100_online_stress_diag_stage(ZY100_SD_RAW_COMMIT);
#endif
        phase = ZY100_ONLINE_RAW_MAG_PHASE_FLASH_IO;
        if (!online_raw_commit_step())
        {
            return false;
        }
    }
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    else if (zy100_online_stress_writer_busy())
    {
        phase = ZY100_ONLINE_RAW_MAG_PHASE_FLASH_IO;
        if (!zy100_online_stress_write_step())
        {
            online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_STRESS);
            return false;
        }
    }
#endif
    else if (s_online_raw.reserve_urgent)
    {
#if ZY100_ONLINE_STRESS_TEST_ENABLE
        zy100_online_stress_diag_stage(ZY100_SD_ERASE);
#endif
        phase = ZY100_ONLINE_RAW_MAG_PHASE_FLASH_IO;
        forward_result = zy100_online_spool_erase_forward_block32_step();
#if ZY100_ONLINE_ERASE_AHEAD_ENABLE
        zy100_stress_diag_erase(true, zy100_online_spool_erase_wip(),
                                zy100_online_spool_erased_ahead_bytes());
#endif
        s_online_raw.stats.block32_erase_steps +=
            (forward_result == ZY100_ONLINE_SPOOL_ERASE_PROGRESS) ? 1U : 0U;
        if (forward_result == ZY100_ONLINE_SPOOL_ERASE_ERROR)
        {
            online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_FLASH);
            return false;
        }
        if (forward_result == ZY100_ONLINE_SPOOL_ERASE_BLOCKED)
        {
            s_online_raw.stats.forward_erase_blocked++;
        }
        else if (forward_result == ZY100_ONLINE_SPOOL_ERASE_IDLE)
        {
            uint32_t count = online_raw_select_record_packet_count(true);
            phase = ZY100_ONLINE_RAW_MAG_PHASE_RECORD_PREPARE;
            if (!online_raw_begin_record_timed(count) &&
                (s_online_raw.error != ZY100_ONLINE_RAW_ERROR_NONE))
            {
                return false;
            }
        }
    }
    else if ((s_online_raw.ring_count >=
              ZY100_ONLINE_RAW_RECORD_TARGET_PACKETS) ||
             (s_online_raw.mag_ring_count >=
              ZY100_FE_RAW_MAG_SAMPLES_PER_RECORD_MAX) ||
             (s_online_raw.finishing &&
              ((s_online_raw.ring_count != 0U) ||
               (s_online_raw.mag_ring_count != 0U) ||
               (s_online_raw.pending_mag_read_errors != 0U) ||
               (s_online_raw.pending_mag_missed_deadlines != 0U))))
    {
#if ZY100_ONLINE_STRESS_TEST_ENABLE
        zy100_online_stress_diag_stage(ZY100_SD_RESERVE);
#endif
        uint32_t count = online_raw_select_record_packet_count(false);
        phase = ZY100_ONLINE_RAW_MAG_PHASE_RECORD_PREPARE;
        if (!online_raw_begin_record_timed(count) &&
            (s_online_raw.error != ZY100_ONLINE_RAW_ERROR_NONE))
        {
            return false;
        }
    }
    else if (!s_online_raw.finishing &&
             (s_online_raw.ring_count <=
              ZY100_ONLINE_RAW_PREERASE_MAX_PACKETS) &&
#if ZY100_ONLINE_ERASE_AHEAD_ENABLE
             online_raw_background_erase_due())
#else
             (zy100_online_spool_erased_ahead_bytes() <
              ZY100_ONLINE_SPOOL_PREERASE_MIN_BYTES))
#endif
    {
#if ZY100_ONLINE_STRESS_TEST_ENABLE
        zy100_online_stress_diag_stage(ZY100_SD_ERASE);
#endif
        phase = ZY100_ONLINE_RAW_MAG_PHASE_FLASH_IO;
        #if ZY100_ONLINE_ERASE_AHEAD_ENABLE
        forward_result = zy100_online_spool_erase_forward_to(ZY100_ONLINE_ERASE_HIGH_BYTES);
        zy100_stress_diag_erase(false, zy100_online_spool_erase_wip(),
                                zy100_online_spool_erased_ahead_bytes());
#else
        forward_result = zy100_online_spool_erase_forward_block32_step();
#endif
        s_online_raw.stats.block32_erase_steps +=
            (forward_result == ZY100_ONLINE_SPOOL_ERASE_PROGRESS) ? 1U : 0U;
        if (forward_result == ZY100_ONLINE_SPOOL_ERASE_ERROR)
        {
            online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_FLASH);
            return false;
        }
    }

#if ZY100_ONLINE_STRESS_TEST_ENABLE
    else if (!zy100_online_stress_write_step())
    {
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_STRESS);
        return false;
    }
#endif

pump_tail:
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    zy100_online_stress_publish(now_ms, &s_online_raw.stats, false);
#endif
    if (!online_raw_mag_schedule_step(phase))
    {
        return false;
    }
    /* This runs in the capture task: it may read committed online Flash into
     * a persistent RAM slot, but it never calls a BLE/GATT API. */
    (void)zy100_online_stream_stage_record_fragment(now_ms);
    if (!online_raw_mag_schedule_step(
            ZY100_ONLINE_RAW_MAG_PHASE_BLE_STAGING))
    {
        return false;
    }
    return s_online_raw.error == ZY100_ONLINE_RAW_ERROR_NONE;
}

#if ZY100_ONLINE_BOUNDED_PUMP_ENABLE
zy100_online_service_result_t zy100_online_raw_capture_service(uint32_t now_ms)
{
    uint32_t start, end, elapsed = 0U, steps = 0U;
    bool timed = bsp_capture_timebase_snapshot(&start);
    zy100_online_service_result_t result = ZY100_ONLINE_SERVICE_WAIT;
    do
    {
        uint32_t seq = zy100_online_spool_progress_seq();
        uint32_t stress = zy100_online_stress_writer_cursor();
        uint32_t page = s_online_raw.source_page_next;
        uint8_t state = s_online_raw.record_state;
        bool flight = s_online_raw.page_in_flight;
        bool erasing = zy100_online_spool_erase_wip();
        if (!zy100_online_raw_capture_pump(now_ms)) { return ZY100_ONLINE_SERVICE_ERROR; }
        steps++;
        /* Spool PUMP_PROGRESS also means BUSY: compare committed state instead. */
        if (seq == zy100_online_spool_progress_seq() &&
            stress == zy100_online_stress_writer_cursor() &&
            page == s_online_raw.source_page_next && state == s_online_raw.record_state &&
            flight == s_online_raw.page_in_flight)
        { result = ZY100_ONLINE_SERVICE_WAIT; break; }
        if (erasing || zy100_online_spool_erase_wip() ||
            (zy100_online_stress_writer_busy() && zy100_online_stress_io_waiting()))
        { result = ZY100_ONLINE_SERVICE_WAIT; break; }
        result = ZY100_ONLINE_SERVICE_READY;
        if (!timed || !bsp_capture_timebase_snapshot(&end))
        { timed = false; result = ZY100_ONLINE_SERVICE_WAIT; break; }
        elapsed = start - end; /* TIM2 is a 1 MHz down counter; unsigned wrap. */
        now_ms = zy100_os_time_ms();
    } while (steps < ZY100_ONLINE_PUMP_MAX_STEPS && elapsed < ZY100_ONLINE_PUMP_BUDGET_US);
    if (timed)
    {
        if (bsp_capture_timebase_snapshot(&end)) { elapsed = start - end; }
        else { timed = false; }
    }
    zy100_stress_diag_service(steps, elapsed, result == ZY100_ONLINE_SERVICE_WAIT, !timed);
#if ZY100_ONLINE_PAGE_RETRY_ENABLE
    /* Leave this service immediately on BUSY, then revisit through the full
     * FIFO/MAG/stop loop. Its existing periodic cooperative yield is retained.
     * Erase/owner contention and missing timebase retain the tick wait. */
    if (timed && result == ZY100_ONLINE_SERVICE_WAIT &&
        zy100_online_spool_page_retry_pending())
    { result = ZY100_ONLINE_SERVICE_READY; }
#endif
    return result;
}
#endif

bool zy100_online_raw_capture_finish(
    bool discard_tail,
    uint32_t now_ms,
    zy100_online_raw_progress_cb_t progress_cb)
{
    uint32_t start_ms = now_ms;

    if (!s_online_raw.active)
    {
        zy100_online_raw_capture_release_mag();
        return true;
    }
    s_online_raw.mag_scheduling = false;
    zy100_online_raw_capture_release_mag();
    s_online_raw.finishing = true;
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    if (!zy100_online_stress_freeze(now_ms))
    { online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_STRESS); }
#endif
    if (!online_raw_flash_wip_finish(progress_cb))
    {
        online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_FINISH);
    }
    if (discard_tail ||
        (s_online_raw.error != ZY100_ONLINE_RAW_ERROR_NONE))
    {
#if ZY100_ONLINE_STRESS_TEST_ENABLE
        zy100_online_stress_diag_freeze(0x200U + s_online_raw.error);
#endif
        online_raw_discard_uncommitted_tail();
    }
    else
    {
        while (
#if ZY100_ONLINE_STRESS_TEST_ENABLE
               zy100_online_stress_pending() ||
#endif
               (s_online_raw.ring_count != 0U) ||
               (s_online_raw.mag_ring_count != 0U) ||
               (s_online_raw.pending_mag_read_errors != 0U) ||
               (s_online_raw.pending_mag_missed_deadlines != 0U) ||
               (s_online_raw.record_state != ZY100_ONLINE_RAW_RECORD_IDLE))
        {
            if (!zy100_online_raw_capture_pump(zy100_os_time_ms()))
            {
                online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_FINISH);
                break;
            }
            if (progress_cb != NULL)
            {
                progress_cb();
            }
            if ((uint32_t)(zy100_os_time_ms() - start_ms) >=
                ZY100_ONLINE_RAW_FINISH_TIMEOUT_MS)
            {
                online_raw_latch_error(ZY100_ONLINE_RAW_ERROR_FINISH);
                break;
            }
            os_delay(1U);
        }
    }
    if ((s_online_raw.error != ZY100_ONLINE_RAW_ERROR_NONE) &&
        ((s_online_raw.ring_count != 0U) ||
         (s_online_raw.target.token != 0U)))
    {
        online_raw_discard_uncommitted_tail();
    }
    online_raw_finalize_fixed40(discard_tail);
    s_online_raw.stats.ring_packets = s_online_raw.ring_count;
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    zy100_online_stress_publish(zy100_os_time_ms(), &s_online_raw.stats, true);
    zy100_online_stress_release(discard_tail || s_online_raw.error != ZY100_ONLINE_RAW_ERROR_NONE);
#endif
    online_raw_log_finish();
    online_raw_release_workspace();
    s_online_raw_mag_ring = NULL;
    s_online_raw.active = false;
    return s_online_raw.error == ZY100_ONLINE_RAW_ERROR_NONE;
}

void zy100_online_raw_capture_get_stats(
    zy100_online_raw_capture_stats_t *out)
{
    if (out != NULL)
    {
        *out = s_online_raw.stats;
    }
}

void zy100_online_raw_capture_get_clock_meta(
    zy100_online_raw_clock_meta_t *out)
{
    if (out != NULL)
    {
        *out = s_online_raw.clock_meta;
    }
}


#else

void zy100_online_raw_capture_log_resource(uint8_t stage) { (void)stage; }
bool zy100_online_raw_capture_prepare_mag(void) { return false; }
void zy100_online_raw_capture_release_mag(void) {}
bool zy100_online_raw_capture_begin(uint32_t source_id_seed)
{
    (void)source_id_seed;
    return false;
}
bool zy100_online_raw_capture_active(void) { return false; }
bool zy100_online_raw_capture_push_packet(const uint8_t packet[16],
                                          uint16_t timestamp_raw)
{
    (void)packet;
    (void)timestamp_raw;
    return false;
}
bool zy100_online_raw_capture_snapshot_fifo_read(
    uint32_t accepted_before,
    zy100_online_raw_fifo_time_snapshot_t *snapshot)
{
    (void)accepted_before;
    if (snapshot != NULL)
    {
        memset(snapshot, 0, sizeof(*snapshot));
    }
    return false;
}
bool zy100_online_raw_capture_note_fifo_burst(
    uint32_t accepted_before,
    uint32_t expected_packets,
    const zy100_online_raw_fifo_time_snapshot_t *snapshot)
{
    (void)accepted_before;
    (void)expected_packets;
    (void)snapshot;
    return false;
}
bool zy100_online_raw_capture_mag_checkpoint(
    zy100_online_raw_mag_checkpoint_t checkpoint)
{
    (void)checkpoint;
    return true;
}
bool zy100_online_raw_capture_pump(uint32_t now_ms)
{
    (void)now_ms;
    return false;
}
bool zy100_online_raw_capture_finish(
    bool discard_tail,
    uint32_t now_ms,
    zy100_online_raw_progress_cb_t progress_cb)
{
    (void)discard_tail;
    (void)now_ms;
    (void)progress_cb;
    return true;
}
void zy100_online_raw_capture_get_stats(
    zy100_online_raw_capture_stats_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}
void zy100_online_raw_capture_get_clock_meta(
    zy100_online_raw_clock_meta_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

#endif
