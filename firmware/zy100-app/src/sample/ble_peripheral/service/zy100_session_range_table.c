#include "zy100_session_range_table.h"

#include <stddef.h>
#include <string.h>

#include "trace.h"

#include "../app_flags.h"
#include "../driver/gd25q32e_spi.h"
#include "zy100_crc32.h"
#include "zy100_final_edge_record_store.h"
#include "zy100_internal_meta_store.h"

#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE

#define ZY100_SRT_U32(a, b, c, d) \
    (((uint32_t)(uint8_t)(a)) | ((uint32_t)(uint8_t)(b) << 8) | \
     ((uint32_t)(uint8_t)(c) << 16) | ((uint32_t)(uint8_t)(d) << 24))

#define ZY100_SRT_MAGIC         ZY100_SRT_U32('S', 'R', 'T', '1')
#define ZY100_SRT_VERSION       1U
#define ZY100_SRT_ALLOC_MAGIC   ZY100_SRT_U32('A', 'L', 'L', 'C')
#define ZY100_SRT_BEGIN_MAGIC   ZY100_SRT_U32('B', 'E', 'G', 'C')
#define ZY100_SRT_COMMIT_MAGIC  ZY100_SRT_U32('C', 'M', 'I', 'T')
#define ZY100_SRT_EXPC_MAGIC    ZY100_SRT_U32('E', 'X', 'P', 'C')
#define ZY100_SRT_RCLB_MAGIC    ZY100_SRT_U32('R', 'C', 'L', 'B')
#define ZY100_SRT_RCLD_MAGIC    ZY100_SRT_U32('R', 'C', 'L', 'D')

#define ZY100_REGION_MAGIC_RAW_BEGIN     ZY100_SRT_U32('R', 'A', 'W', 'B')
#define ZY100_REGION_MAGIC_RAW_END       ZY100_SRT_U32('R', 'A', 'W', 'E')
#define ZY100_REGION_MAGIC_SUMMARY_BEGIN ZY100_SRT_U32('S', 'U', 'M', 'B')
#define ZY100_REGION_MAGIC_SUMMARY_END   ZY100_SRT_U32('S', 'U', 'M', 'E')
#define ZY100_REGION_MAGIC_EVENT_BEGIN   ZY100_SRT_U32('E', 'V', 'T', 'B')
#define ZY100_REGION_MAGIC_EVENT_END     ZY100_SRT_U32('E', 'V', 'T', 'E')

#define ZY100_REGION_TYPE_RAW     1U
#define ZY100_REGION_TYPE_SUMMARY 2U
#define ZY100_REGION_TYPE_EVENT   3U
#define ZY100_MARKER_TYPE_BEGIN   1U
#define ZY100_MARKER_TYPE_END     2U

#define ZY100_SRT_ERASE_LOG_SECTORS 1U

#define ZY100_RAW_REGION_END \
    (ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR + ZY100_FINAL_EDGE_RAW_REGION_BYTES)
#define ZY100_SUMMARY_REGION_END \
    (ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR + ZY100_FINAL_EDGE_SUMMARY_REGION_BYTES)
#define ZY100_EVENT_REGION_END \
    (ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR + ZY100_FINAL_EDGE_EVENT_REGION_BYTES)
#define ZY100_SRT_TABLE_END \
    (ZY100_SESSION_DIR_REGION_BASE_ADDR + ZY100_SESSION_DIR_REGION_BYTES)

#if (ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR != 0UL)
#define ZY100_SRT_RAW_ADDR_BEFORE_BASE(addr) \
    ((addr) < ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR)
#define ZY100_SRT_RAW_OFFSET_OR_ZERO(addr) \
    (((addr) >= ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR) ? \
     ((addr) - ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR) : 0U)
#else
#define ZY100_SRT_RAW_ADDR_BEFORE_BASE(addr) 0
#define ZY100_SRT_RAW_OFFSET_OR_ZERO(addr) (addr)
#endif

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint16_t version;
    uint16_t entry_bytes;

    uint32_t entry_seq;
    uint32_t session_uid;
    uint32_t user_id;
    uint32_t training_id;
    uint32_t session_seq;
    uint32_t round;

    uint64_t start_time_ms;
    uint32_t time_calibrated;
    uint32_t source;

    uint32_t raw_marker_begin_addr;
    uint32_t raw_data_begin_addr;
    uint32_t raw_data_end_addr;
    uint32_t raw_reclaim_end_addr;

    uint32_t summary_marker_begin_addr;
    uint32_t summary_data_begin_addr;
    uint32_t summary_data_end_addr;
    uint32_t summary_reclaim_end_addr;

    uint32_t event_marker_begin_addr;
    uint32_t event_data_begin_addr;
    uint32_t event_data_end_addr;
    uint32_t event_reclaim_end_addr;

    uint32_t begin_crc32;
    uint32_t alloc_magic;
    uint32_t begin_complete_magic;

    uint64_t end_time_ms;
    uint32_t stop_reason;

    uint32_t raw_count;
    uint32_t raw_used_bytes;
    uint32_t summary_count;
    uint32_t summary_used_bytes;
    uint32_t event_count;
    uint32_t event_used_bytes;

    uint32_t official_hit_count;
    uint32_t nohit_count;
    uint32_t live_hz;
    uint32_t ois_hz;
    uint32_t pass_flags;
    uint32_t warn_flags;
    uint32_t error_flags;
    uint32_t flags;
    uint8_t rtc_meta[ZY100_CAPTURE_TIME_META_BYTES];

    uint32_t commit_crc32;
    uint32_t commit_magic;
    uint32_t export_confirmed_magic;
    uint32_t reclaim_begin_magic;
    uint32_t reclaim_crc32;
    uint32_t reclaimed_magic;
    uint32_t export_stream_crc32;
    uint32_t export_bytes_sent;

    uint8_t reserved[16];
} zy100_srt_wire_entry_t;

typedef char zy100_srt_wire_entry_size_check[
    (sizeof(zy100_srt_wire_entry_t) == ZY100_SESSION_RANGE_ENTRY_BYTES) ?
        1 : -1];
typedef char zy100_srt_public_marker_size_check[
    (sizeof(zy100_region_session_marker_t) == ZY100_SESSION_RANGE_MARKER_BYTES) ?
        1 : -1];
typedef char zy100_srt_entry_page_check[
    (ZY100_SESSION_RANGE_ENTRY_BYTES ==
     ZY100_INTERNAL_META_STORE_PAGE_BYTES) ? 1 : -1];
typedef char zy100_srt_table_sector_entries_check[
    ((ZY100_INTERNAL_META_STORE_SECTOR_BYTES /
      ZY100_SESSION_RANGE_ENTRY_BYTES) == 16U) ? 1 : -1];

typedef enum
{
    ZY100_SRT_RECLAIM_IDLE = 0U,
    ZY100_SRT_RECLAIM_RAW,
    ZY100_SRT_RECLAIM_SUMMARY,
    ZY100_SRT_RECLAIM_EVENT,
    ZY100_SRT_RECLAIM_FINISH,
} zy100_srt_reclaim_phase_t;

typedef enum
{
    ZY100_SRT_SCAN_BOOT_RECOVERY = 0U,
    ZY100_SRT_SCAN_RUNTIME_CACHE,
    ZY100_SRT_SCAN_START_ALLOC,
} zy100_srt_scan_mode_t;

typedef struct
{
    bool scanned;
    bool dirty_need_clear;
    bool reclaim_active;
    bool clear_active;
    bool clear_in_flight;
    zy100_srt_reclaim_phase_t reclaim_phase;
    zy100_session_alloc_status_t last_alloc_status;

    uint32_t append_pos;
    uint32_t pending_count;
    uint32_t committed_count;
    uint32_t reclaimed_count;
    uint32_t raw_tail;
    uint32_t summary_tail;
    uint32_t event_tail;

    int32_t tail_pending_index;
    int32_t tail_unreclaimed_index;
    int32_t reclaim_needed_index;
    int32_t incomplete_tail_index;

    bool active_valid;
    bool active_uncommitted_seen;
    uint32_t active_entry_index;
    uint32_t active_session_uid;

    uint32_t clear_next_sector;
    uint32_t clear_total_sectors;
    uint32_t clear_done_sectors;
    uint32_t reclaim_next_addr;
    uint32_t reclaim_end_addr;
    zy100_session_range_entry_t reclaim_entry;
} zy100_srt_state_t;

static zy100_srt_state_t s_srt;

static void srt_clear_active(void)
{
    s_srt.active_valid = false;
    s_srt.active_uncommitted_seen = false;
    s_srt.active_entry_index = 0U;
    s_srt.active_session_uid = 0U;
}

static void srt_set_active(uint32_t entry_index, uint32_t session_uid)
{
    s_srt.active_valid = (session_uid != 0U);
    s_srt.active_uncommitted_seen = false;
    s_srt.active_entry_index = entry_index;
    s_srt.active_session_uid = session_uid;
}

static bool srt_active_matches(uint32_t entry_index,
                               const zy100_srt_wire_entry_t *entry)
{
    return s_srt.active_valid &&
           (entry != NULL) &&
           (s_srt.active_entry_index == entry_index) &&
           (s_srt.active_session_uid == entry->session_uid);
}

static uint32_t srt_align_up(uint32_t value, uint32_t align)
{
    return (value + (align - 1U)) & ~(align - 1U);
}

static bool srt_magic_valid(uint32_t value, uint32_t magic)
{
    return value == magic;
}

static uint32_t srt_entry_count(void)
{
    return ZY100_SESSION_DIR_REGION_BYTES / ZY100_SESSION_RANGE_ENTRY_BYTES;
}

static uint32_t srt_entry_addr(uint32_t index)
{
    return ZY100_SESSION_DIR_REGION_BASE_ADDR +
           (index * ZY100_SESSION_RANGE_ENTRY_BYTES);
}

static uint8_t srt_remaining_percent(uint32_t base, uint32_t end, uint32_t cursor)
{
    uint32_t total;
    uint32_t remaining;

    if (end <= base)
    {
        return 0U;
    }

    total = end - base;
    if (cursor <= base)
    {
        remaining = total;
    }
    else if (cursor >= end)
    {
        remaining = 0U;
    }
    else
    {
        remaining = end - cursor;
    }

    return (uint8_t)((((uint64_t)remaining * 100ULL) + (uint64_t)(total / 2U)) /
                     (uint64_t)total);
}

static uint8_t srt_min_percent(uint8_t a, uint8_t b)
{
    return (a < b) ? a : b;
}

static uint8_t srt_usable_remaining_percent(uint32_t append_pos,
                                            uint32_t raw_cursor,
                                            uint32_t summary_cursor,
                                            uint32_t event_cursor)
{
    uint32_t dir_cursor;
    uint8_t percent;

    dir_cursor = ZY100_SESSION_DIR_REGION_BASE_ADDR +
                 (append_pos * ZY100_SESSION_RANGE_ENTRY_BYTES);
    percent = srt_remaining_percent(ZY100_SESSION_DIR_REGION_BASE_ADDR,
                                    ZY100_SRT_TABLE_END,
                                    dir_cursor);
    percent = srt_min_percent(
                  percent,
                  srt_remaining_percent(ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR,
                                        ZY100_RAW_REGION_END,
                                        raw_cursor));
    percent = srt_min_percent(
                  percent,
                  srt_remaining_percent(ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR,
                                        ZY100_SUMMARY_REGION_END,
                                        summary_cursor));
    percent = srt_min_percent(
                  percent,
                  srt_remaining_percent(ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR,
                                        ZY100_EVENT_REGION_END,
                                        event_cursor));
    return percent;
}

static bool srt_is_all_ff(const uint8_t *buf, uint32_t len)
{
    uint32_t idx;

    if (buf == NULL)
    {
        return false;
    }
    for (idx = 0U; idx < len; idx++)
    {
        if (buf[idx] != 0xFFU)
        {
            return false;
        }
    }
    return true;
}

static bool srt_read_entry(uint32_t index, zy100_srt_wire_entry_t *entry)
{
    if ((entry == NULL) || (index >= srt_entry_count()))
    {
        return false;
    }
    return zy100_internal_meta_store_read(srt_entry_addr(index),
                                          (uint8_t *)entry,
                                          ZY100_SESSION_RANGE_ENTRY_BYTES);
}

static bool srt_meta_read_page(
    uint32_t offset,
    uint8_t page[ZY100_INTERNAL_META_STORE_PAGE_BYTES])
{
    return zy100_internal_meta_store_read_page(offset, page);
}

static bool srt_external_read_page(uint32_t addr,
                                   uint8_t page[GD25Q32E_PAGE_BYTES])
{
    if ((page == NULL) || ((addr % GD25Q32E_PAGE_BYTES) != 0U))
    {
        return false;
    }
    return gd25q32e_read(addr, page, GD25Q32E_PAGE_BYTES) == IMU_STATUS_OK;
}

static bool srt_meta_program_page_overlay(
    uint32_t offset,
    const uint8_t page[ZY100_INTERNAL_META_STORE_PAGE_BYTES])
{
    uint8_t old_page[ZY100_INTERNAL_META_STORE_PAGE_BYTES];
    uint32_t idx;

    if ((page == NULL) ||
        ((offset % ZY100_INTERNAL_META_STORE_PAGE_BYTES) != 0U))
    {
        return false;
    }
    if (!srt_meta_read_page(offset, old_page))
    {
        return false;
    }
    for (idx = 0U; idx < ZY100_INTERNAL_META_STORE_PAGE_BYTES; idx++)
    {
        if ((old_page[idx] & page[idx]) != page[idx])
        {
            DBG_DIRECT("[SESSION_TABLE] meta_overlay_reject off=0x%08lX idx=%lu old=0x%02X new=0x%02X",
                       (unsigned long)offset,
                       (unsigned long)idx,
                       old_page[idx],
                       page[idx]);
            return false;
        }
    }
    return zy100_internal_meta_store_program_page_overlay(offset, page);
}

static bool srt_program_entry(uint32_t index,
                              const zy100_srt_wire_entry_t *entry)
{
    if ((entry == NULL) || (index >= srt_entry_count()))
    {
        return false;
    }
    return srt_meta_program_page_overlay(srt_entry_addr(index),
                                         (const uint8_t *)entry);
}

static bool srt_external_sector_erased(uint32_t sector_addr)
{
    uint8_t page[GD25Q32E_PAGE_BYTES];
    uint32_t offset;

    if ((sector_addr % GD25Q32E_SECTOR_BYTES) != 0U)
    {
        return false;
    }
    for (offset = 0U; offset < GD25Q32E_SECTOR_BYTES;
         offset += GD25Q32E_PAGE_BYTES)
    {
        if (!srt_external_read_page(sector_addr + offset, page) ||
            !srt_is_all_ff(page, sizeof(page)))
        {
            return false;
        }
    }
    return true;
}

static bool srt_external_erase_sector(uint32_t sector_addr)
{
    if ((sector_addr % GD25Q32E_SECTOR_BYTES) != 0U)
    {
        return false;
    }
    if (gd25q32e_sector_erase_4k(sector_addr) != IMU_STATUS_OK)
    {
        return false;
    }
    if (gd25q32e_wait_while_busy(GD25Q32E_SECTOR_ERASE_TIMEOUT_MS) !=
        IMU_STATUS_OK)
    {
        return false;
    }
    return srt_external_sector_erased(sector_addr);
}

static bool srt_meta_sector_erased(uint32_t sector_offset)
{
    return zy100_internal_meta_store_sector_erased(sector_offset);
}

static bool srt_meta_erase_sector(uint32_t sector_offset)
{
    return zy100_internal_meta_store_erase_sector(sector_offset);
}

static bool srt_ensure_erased_range(uint32_t begin,
                                    uint32_t end,
                                    const char *region)
{
    uint32_t addr;

    if ((begin > end) ||
        ((begin % GD25Q32E_SECTOR_BYTES) != 0U) ||
        ((end % GD25Q32E_SECTOR_BYTES) != 0U))
    {
        return false;
    }
    for (addr = begin; addr < end; addr += GD25Q32E_SECTOR_BYTES)
    {
        if (srt_external_sector_erased(addr))
        {
            continue;
        }
        ZY100_LOG_VERBOSE("[ERASE_AHEAD] region=%s addr=0x%08lX sectors=%u reason=append_tail",
                          (region != NULL) ? region : "unknown",
                          (unsigned long)addr,
                          ZY100_SRT_ERASE_LOG_SECTORS);
        if (!srt_external_erase_sector(addr))
        {
            return false;
        }
    }
    return true;
}

static bool srt_reclaim_erase_one_sector(const char *region)
{
    uint32_t addr = s_srt.reclaim_next_addr;

    if (addr >= s_srt.reclaim_end_addr)
    {
        return true;
    }
    if (!srt_external_sector_erased(addr))
    {
#if ZY100_BLE_EXPORT_MID_LOG_ENABLE
        DBG_DIRECT("[SESSION_RECLAIM] erase region=%s begin=0x%08lX end=0x%08lX",
                   (region != NULL) ? region : "unknown",
                   (unsigned long)addr,
                   (unsigned long)(addr + GD25Q32E_SECTOR_BYTES));
#endif
        if (!srt_external_erase_sector(addr))
        {
            DBG_DIRECT("[SESSION_RECLAIM] fail uid=%lu region=%s status=%u keep_rcld=0",
                       (unsigned long)s_srt.reclaim_entry.session_uid,
                       (region != NULL) ? region : "unknown",
                       (uint32_t)ZY100_FLASH_PREP_ERROR);
            return false;
        }
    }
    s_srt.reclaim_next_addr += GD25Q32E_SECTOR_BYTES;
    return true;
}

static uint32_t srt_wire_crc(const zy100_srt_wire_entry_t *entry,
                             uint32_t crc_offset)
{
    zy100_srt_wire_entry_t temp;

    if ((entry == NULL) || (crc_offset >= sizeof(temp)))
    {
        return 0U;
    }
    memcpy(&temp, entry, sizeof(temp));
    if (crc_offset == offsetof(zy100_srt_wire_entry_t, begin_crc32))
    {
        temp.begin_crc32 = 0U;
    }
    else if (crc_offset == offsetof(zy100_srt_wire_entry_t, commit_crc32))
    {
        temp.commit_crc32 = 0U;
        temp.commit_magic = 0xFFFFFFFFUL;
    }
    else if (crc_offset == offsetof(zy100_srt_wire_entry_t, reclaim_crc32))
    {
        temp.reclaim_crc32 = 0U;
        temp.reclaimed_magic = 0xFFFFFFFFUL;
    }
    return zy100_crc32_ieee((const uint8_t *)&temp, crc_offset);
}

static bool srt_wire_alloc_valid(const zy100_srt_wire_entry_t *entry)
{
    if (entry == NULL)
    {
        return false;
    }
    return (entry->magic == ZY100_SRT_MAGIC) &&
           (entry->version == ZY100_SRT_VERSION) &&
           (entry->entry_bytes == ZY100_SESSION_RANGE_ENTRY_BYTES) &&
           srt_magic_valid(entry->alloc_magic, ZY100_SRT_ALLOC_MAGIC);
}

static bool srt_wire_begin_complete(const zy100_srt_wire_entry_t *entry)
{
    return (entry != NULL) &&
           srt_magic_valid(entry->begin_complete_magic,
                           ZY100_SRT_BEGIN_MAGIC);
}

static bool srt_wire_committed(const zy100_srt_wire_entry_t *entry)
{
    return (entry != NULL) &&
           srt_magic_valid(entry->commit_magic, ZY100_SRT_COMMIT_MAGIC);
}

static bool srt_wire_export_confirmed(const zy100_srt_wire_entry_t *entry)
{
    return (entry != NULL) &&
           srt_magic_valid(entry->export_confirmed_magic,
                           ZY100_SRT_EXPC_MAGIC);
}

static bool srt_wire_reclaim_begin(const zy100_srt_wire_entry_t *entry)
{
    return (entry != NULL) &&
           srt_magic_valid(entry->reclaim_begin_magic, ZY100_SRT_RCLB_MAGIC);
}

static bool srt_wire_reclaimed(const zy100_srt_wire_entry_t *entry)
{
    return (entry != NULL) &&
           srt_magic_valid(entry->reclaimed_magic, ZY100_SRT_RCLD_MAGIC);
}

static void srt_fill_common_ranges(zy100_session_range_entry_t *out,
                                   const zy100_srt_wire_entry_t *entry)
{
    uint32_t raw_total =
        ZY100_FINAL_EDGE_RAW_REGION_BYTES / ZY100_FINAL_EDGE_RAW_BUCKET_BYTES;

    out->raw_marker_begin_addr = entry->raw_marker_begin_addr;
    out->raw_data_begin_addr = entry->raw_data_begin_addr;
    out->raw_data_end_addr = entry->raw_data_end_addr;
    out->raw_reclaim_end_addr = entry->raw_reclaim_end_addr;
    out->raw_first_bucket =
        ZY100_SRT_RAW_OFFSET_OR_ZERO(entry->raw_data_begin_addr) /
        ZY100_FINAL_EDGE_RAW_BUCKET_BYTES;
    out->raw_limit_bucket =
        (entry->raw_data_end_addr != 0xFFFFFFFFUL) ?
        ((entry->raw_data_end_addr - ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR) /
         ZY100_FINAL_EDGE_RAW_BUCKET_BYTES) : raw_total;
    out->raw_reserved_bucket_count =
        (raw_total > out->raw_first_bucket) ?
        (raw_total - out->raw_first_bucket) : 0U;
    out->raw_bucket_count = entry->raw_count;
    out->raw_bucket_bytes = ZY100_FINAL_EDGE_RAW_BUCKET_BYTES;
    out->raw_used_bytes = entry->raw_used_bytes;

    out->summary_marker_begin_addr = entry->summary_marker_begin_addr;
    out->summary_data_begin_addr = entry->summary_data_begin_addr;
    out->summary_data_end_addr = entry->summary_data_end_addr;
    out->summary_reclaim_end_addr = entry->summary_reclaim_end_addr;
    out->summary_base_addr = entry->summary_data_begin_addr;
    out->summary_limit_addr =
        (entry->summary_data_end_addr != 0xFFFFFFFFUL) ?
        entry->summary_data_end_addr : ZY100_SUMMARY_REGION_END;
    out->summary_reserved_bytes =
        (ZY100_SUMMARY_REGION_END > entry->summary_data_begin_addr) ?
        (ZY100_SUMMARY_REGION_END - entry->summary_data_begin_addr) : 0U;
    out->summary_count = entry->summary_count;
    out->summary_used_bytes = entry->summary_used_bytes;

    out->event_marker_begin_addr = entry->event_marker_begin_addr;
    out->event_data_begin_addr = entry->event_data_begin_addr;
    out->event_data_end_addr = entry->event_data_end_addr;
    out->event_reclaim_end_addr = entry->event_reclaim_end_addr;
    out->event_base_addr = entry->event_data_begin_addr;
    out->event_limit_addr =
        (entry->event_data_end_addr != 0xFFFFFFFFUL) ?
        entry->event_data_end_addr : ZY100_EVENT_REGION_END;
    out->event_reserved_bytes =
        (ZY100_EVENT_REGION_END > entry->event_data_begin_addr) ?
        (ZY100_EVENT_REGION_END - entry->event_data_begin_addr) : 0U;
    out->event_count = entry->event_count;
    out->event_used_bytes = entry->event_used_bytes;
}

static void srt_entry_from_wire(uint32_t index,
                                const zy100_srt_wire_entry_t *entry,
                                zy100_session_range_entry_t *out)
{
    if ((entry == NULL) || (out == NULL))
    {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->entry_seq = entry->entry_seq;
    out->entry_addr = srt_entry_addr(index);
    out->session_uid = entry->session_uid;
    out->user_id = entry->user_id;
    out->training_id = entry->training_id;
    out->session_seq = entry->session_seq;
    out->round = entry->round;
    out->start_time_ms = entry->start_time_ms;
    out->end_time_ms = entry->end_time_ms;
    out->time_calibrated = entry->time_calibrated;
    out->source = entry->source;
    out->stop_reason = entry->stop_reason;
    out->official_hit_count = entry->official_hit_count;
    out->nohit_count = entry->nohit_count;
    out->live_hz = entry->live_hz;
    out->ois_hz = entry->ois_hz;
    out->pass_flags = entry->pass_flags;
    out->warn_flags = entry->warn_flags;
    out->error_flags = entry->error_flags;
    out->flags = entry->flags;
    memcpy(out->rtc_meta, entry->rtc_meta, sizeof(out->rtc_meta));
    srt_fill_common_ranges(out, entry);
    if (srt_wire_reclaimed(entry))
    {
        out->state = ZY100_SESSION_STATE_RECLAIMED;
    }
    else if (srt_wire_reclaim_begin(entry))
    {
        out->state = ZY100_SESSION_STATE_RECLAIMING;
    }
    else if (srt_wire_export_confirmed(entry))
    {
        out->state = ZY100_SESSION_STATE_EXPORT_CONFIRMED;
    }
    else if (srt_wire_committed(entry))
    {
        out->state = ZY100_SESSION_STATE_COMMITTED;
    }
    else if (srt_wire_begin_complete(entry))
    {
        out->state = ZY100_SESSION_STATE_BEGIN_COMPLETE;
    }
    else
    {
        out->state = ZY100_SESSION_STATE_ALLOCATED;
    }
    if (srt_wire_begin_complete(entry))
    {
        out->record_flags |= ZY100_SESSION_DIR_RECORD_BEGIN_VALID;
    }
    if (srt_wire_committed(entry))
    {
        out->record_flags |= ZY100_SESSION_DIR_RECORD_END_VALID;
    }
}

static bool srt_read_public_entry(uint32_t index,
                                  zy100_session_range_entry_t *out)
{
    zy100_srt_wire_entry_t wire;

    if ((out == NULL) || !srt_read_entry(index, &wire) ||
        !srt_wire_alloc_valid(&wire))
    {
        return false;
    }
    srt_entry_from_wire(index, &wire, out);
    return true;
}

static void srt_update_tail_from_wire(uint32_t entry_index,
                                      const zy100_srt_wire_entry_t *entry,
                                      zy100_srt_scan_mode_t mode)
{
    uint32_t raw_end = entry->raw_reclaim_end_addr;
    uint32_t summary_end = entry->summary_reclaim_end_addr;
    uint32_t event_end = entry->event_reclaim_end_addr;
    bool runtime_active =
        ((mode == ZY100_SRT_SCAN_RUNTIME_CACHE) ||
         (mode == ZY100_SRT_SCAN_START_ALLOC)) &&
        srt_active_matches(entry_index, entry);

    if (!srt_wire_committed(entry))
    {
        raw_end = (srt_wire_begin_complete(entry) && !runtime_active) ?
                  ZY100_RAW_REGION_END : entry->raw_data_begin_addr;
        summary_end = (srt_wire_begin_complete(entry) && !runtime_active) ?
                      ZY100_SUMMARY_REGION_END :
                      entry->summary_data_begin_addr;
        event_end = (srt_wire_begin_complete(entry) && !runtime_active) ?
                    ZY100_EVENT_REGION_END : entry->event_data_begin_addr;
    }
    if ((raw_end != 0xFFFFFFFFUL) && (raw_end > s_srt.raw_tail))
    {
        s_srt.raw_tail = raw_end;
    }
    if ((summary_end != 0xFFFFFFFFUL) && (summary_end > s_srt.summary_tail))
    {
        s_srt.summary_tail = summary_end;
    }
    if ((event_end != 0xFFFFFFFFUL) && (event_end > s_srt.event_tail))
    {
        s_srt.event_tail = event_end;
    }
}

static bool srt_scan_mode(zy100_srt_scan_mode_t mode)
{
    uint32_t idx;
    uint32_t max_entries = srt_entry_count();
    int32_t tail_unreclaimed = -1;
    uint32_t reclaim_needed_count = 0U;
    bool keep_active = (mode != ZY100_SRT_SCAN_BOOT_RECOVERY) &&
                       s_srt.active_valid;
    uint32_t active_entry_index = s_srt.active_entry_index;
    uint32_t active_session_uid = s_srt.active_session_uid;

    memset(&s_srt, 0, sizeof(s_srt));
    s_srt.last_alloc_status = ZY100_SESSION_ALLOC_OK;
    s_srt.raw_tail = ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR;
    s_srt.summary_tail = ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR;
    s_srt.event_tail = ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR;
    s_srt.tail_pending_index = -1;
    s_srt.tail_unreclaimed_index = -1;
    s_srt.reclaim_needed_index = -1;
    s_srt.incomplete_tail_index = -1;
    if (keep_active)
    {
        s_srt.active_valid = true;
        s_srt.active_entry_index = active_entry_index;
        s_srt.active_session_uid = active_session_uid;
    }

    if (!zy100_internal_meta_store_ready())
    {
        s_srt.last_alloc_status = ZY100_SESSION_ALLOC_FLASH_NOT_READY;
        return false;
    }

    for (idx = 0U; idx < max_entries; idx++)
    {
        zy100_srt_wire_entry_t wire;

        if (!srt_read_entry(idx, &wire))
        {
            s_srt.dirty_need_clear = true;
            continue;
        }
        if (srt_is_all_ff((const uint8_t *)&wire, sizeof(wire)))
        {
            continue;
        }
        if (!srt_wire_alloc_valid(&wire))
        {
            s_srt.dirty_need_clear = true;
            continue;
        }
        if ((idx + 1U) > s_srt.append_pos)
        {
            s_srt.append_pos = idx + 1U;
        }
        if (srt_wire_reclaimed(&wire))
        {
            s_srt.reclaimed_count++;
            continue;
        }

        tail_unreclaimed = (int32_t)idx;
        s_srt.tail_unreclaimed_index = (int32_t)idx;
        srt_update_tail_from_wire(idx, &wire, mode);

        if (srt_wire_committed(&wire))
        {
            s_srt.committed_count++;
            if (!srt_wire_export_confirmed(&wire))
            {
                s_srt.pending_count++;
                s_srt.tail_pending_index = (int32_t)idx;
            }
            else
            {
                s_srt.reclaim_needed_index = (int32_t)idx;
                reclaim_needed_count++;
            }
        }
        else
        {
            bool runtime_active =
                (mode != ZY100_SRT_SCAN_BOOT_RECOVERY) &&
                srt_active_matches(idx, &wire);

            if (runtime_active)
            {
                s_srt.active_uncommitted_seen = true;
            }
            else
            {
                s_srt.incomplete_tail_index = (int32_t)idx;
            }
        }
    }

    if ((s_srt.reclaim_needed_index >= 0) &&
        ((s_srt.reclaim_needed_index != tail_unreclaimed) ||
         (reclaim_needed_count != 1U)))
    {
        s_srt.dirty_need_clear = true;
    }
    if ((s_srt.incomplete_tail_index >= 0) &&
        (s_srt.incomplete_tail_index != tail_unreclaimed))
    {
        s_srt.dirty_need_clear = true;
    }
    if ((mode == ZY100_SRT_SCAN_START_ALLOC) &&
        s_srt.active_uncommitted_seen)
    {
        s_srt.last_alloc_status = ZY100_SESSION_ALLOC_BAD_STATE;
        s_srt.dirty_need_clear = true;
    }

    s_srt.scanned = true;
    ZY100_LOG_VERBOSE("[SESSION_TAIL] raw=0x%08lX summary=0x%08lX event=0x%08lX table=%u",
                      (unsigned long)s_srt.raw_tail,
                      (unsigned long)s_srt.summary_tail,
                      (unsigned long)s_srt.event_tail,
                      s_srt.append_pos);
    return !s_srt.dirty_need_clear;
}

bool zy100_session_range_table_scan(void)
{
    if (s_srt.reclaim_active ||
        s_srt.clear_active ||
        s_srt.clear_in_flight)
    {
        return s_srt.scanned && !s_srt.dirty_need_clear;
    }
    return srt_scan_mode(ZY100_SRT_SCAN_RUNTIME_CACHE);
}

bool zy100_session_range_table_scan_boot_recovery(void)
{
    return srt_scan_mode(ZY100_SRT_SCAN_BOOT_RECOVERY);
}

static bool srt_scan_if_needed(void)
{
    return s_srt.scanned || zy100_session_range_table_scan();
}

static bool srt_fill_readiness_from_active_op(zy100_session_readiness_t *out)
{
    if ((out == NULL) ||
        (!s_srt.reclaim_active &&
         !s_srt.clear_active &&
         !s_srt.clear_in_flight))
    {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->valid = true;
    out->storage_dirty_need_clear = s_srt.dirty_need_clear;
    out->reclaim_active = s_srt.reclaim_active;
    out->clear_active = s_srt.clear_active || s_srt.clear_in_flight;
    out->expc_without_rcld = s_srt.reclaim_active;
    out->pending_count = s_srt.pending_count;
    out->append_pos = s_srt.append_pos;
    out->raw_tail = s_srt.raw_tail;
    out->summary_tail = s_srt.summary_tail;
    out->event_tail = s_srt.event_tail;
    out->reclaim_session_uid = s_srt.reclaim_active ?
                               s_srt.reclaim_entry.session_uid : 0U;
    out->can_start_next = false;
    out->last_alloc_status = ZY100_SESSION_ALLOC_BAD_STATE;
    out->remaining_percent = 0U;
    return true;
}

uint32_t zy100_session_range_table_pending_count(void)
{
    (void)srt_scan_if_needed();
    return s_srt.pending_count;
}

uint32_t zy100_session_range_table_total_committed_count(void)
{
    (void)srt_scan_if_needed();
    return s_srt.committed_count;
}

uint32_t zy100_session_range_table_reclaimed_count(void)
{
    (void)srt_scan_if_needed();
    return s_srt.reclaimed_count;
}

bool zy100_session_range_table_storage_dirty_need_clear(void)
{
    (void)srt_scan_if_needed();
    return s_srt.dirty_need_clear;
}

bool zy100_session_range_table_refresh_readiness(
    zy100_session_readiness_t *out)
{
    zy100_srt_wire_entry_t wire;
    uint32_t raw_marker;
    uint32_t raw_data;
    uint32_t summary_marker;
    uint32_t summary_data;
    uint32_t event_marker;
    uint32_t event_data;
    bool scan_clean;

    if (out == NULL)
    {
        return false;
    }
    memset(out, 0, sizeof(*out));
    out->last_alloc_status = ZY100_SESSION_ALLOC_FLASH_NOT_READY;

    if (srt_fill_readiness_from_active_op(out))
    {
        return true;
    }

    scan_clean = zy100_session_range_table_scan();
    if (!s_srt.scanned)
    {
        return false;
    }

    out->valid = true;
    out->storage_dirty_need_clear = s_srt.dirty_need_clear;
    out->reclaim_active = s_srt.reclaim_active ||
                          (s_srt.reclaim_needed_index >= 0);
    out->clear_active = s_srt.clear_active || s_srt.clear_in_flight;
    out->expc_without_rcld = s_srt.reclaim_active ||
                             (s_srt.reclaim_needed_index >= 0);
    out->pending_count = s_srt.pending_count;
    out->append_pos = s_srt.append_pos;
    out->raw_tail = s_srt.raw_tail;
    out->summary_tail = s_srt.summary_tail;
    out->event_tail = s_srt.event_tail;
    out->last_alloc_status = s_srt.last_alloc_status;
    if (s_srt.reclaim_active)
    {
        out->reclaim_session_uid = s_srt.reclaim_entry.session_uid;
    }
    else if (s_srt.reclaim_needed_index >= 0)
    {
        zy100_session_range_entry_t tail;

        if (srt_read_public_entry((uint32_t)s_srt.reclaim_needed_index, &tail))
        {
            out->reclaim_session_uid = tail.session_uid;
        }
    }

    if (!scan_clean || s_srt.dirty_need_clear)
    {
        out->last_alloc_status = ZY100_SESSION_ALLOC_BAD_STATE;
        return true;
    }
    if (out->clear_active || out->reclaim_active ||
        (s_srt.incomplete_tail_index >= 0))
    {
        out->last_alloc_status = ZY100_SESSION_ALLOC_BAD_STATE;
        return true;
    }
    if (s_srt.append_pos >= srt_entry_count())
    {
        out->last_alloc_status = ZY100_SESSION_ALLOC_DIR_FULL;
        return true;
    }
    if (!srt_read_entry(s_srt.append_pos, &wire) ||
        !srt_is_all_ff((const uint8_t *)&wire, sizeof(wire)))
    {
        out->last_alloc_status = ZY100_SESSION_ALLOC_DIR_FULL;
        return true;
    }

    raw_marker = srt_align_up(s_srt.raw_tail,
                              ZY100_FINAL_EDGE_RAW_BUCKET_BYTES);
    raw_data = raw_marker + ZY100_FINAL_EDGE_RAW_BUCKET_BYTES;
    summary_marker = srt_align_up(s_srt.summary_tail, GD25Q32E_SECTOR_BYTES);
    summary_data = summary_marker + GD25Q32E_SECTOR_BYTES;
    event_marker = srt_align_up(s_srt.event_tail, GD25Q32E_SECTOR_BYTES);
    event_data = event_marker + GD25Q32E_SECTOR_BYTES;
    out->remaining_percent = srt_usable_remaining_percent(s_srt.append_pos,
                                                          raw_marker,
                                                          summary_marker,
                                                          event_marker);

    if (ZY100_SRT_RAW_ADDR_BEFORE_BASE(raw_marker) ||
        ((raw_data + ZY100_FINAL_EDGE_RAW_BUCKET_BYTES) > ZY100_RAW_REGION_END))
    {
        out->last_alloc_status = ZY100_SESSION_ALLOC_RAW_FULL;
        return true;
    }
    if ((summary_marker < ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR) ||
        ((summary_data + GD25Q32E_SECTOR_BYTES) > ZY100_SUMMARY_REGION_END))
    {
        out->last_alloc_status = ZY100_SESSION_ALLOC_SUMMARY_FULL;
        return true;
    }
    if ((event_marker < ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR) ||
        ((event_data + GD25Q32E_SECTOR_BYTES) > ZY100_EVENT_REGION_END))
    {
        out->last_alloc_status = ZY100_SESSION_ALLOC_EVENT_FULL;
        return true;
    }

    out->can_start_next = true;
    out->last_alloc_status = ZY100_SESSION_ALLOC_OK;
    return true;
}

bool zy100_session_range_table_get_tail_pending(
    zy100_session_range_entry_t *out)
{
    if ((out == NULL) || !srt_scan_if_needed() ||
        (s_srt.tail_pending_index < 0) ||
        (s_srt.tail_pending_index != s_srt.tail_unreclaimed_index))
    {
        return false;
    }
    return srt_read_public_entry((uint32_t)s_srt.tail_pending_index, out);
}

bool zy100_session_range_table_get_tail_unreclaimed(
    zy100_session_range_entry_t *out)
{
    if ((out == NULL) || !srt_scan_if_needed() ||
        (s_srt.tail_unreclaimed_index < 0))
    {
        return false;
    }
    return srt_read_public_entry((uint32_t)s_srt.tail_unreclaimed_index, out);
}

static bool srt_write_marker(uint32_t addr,
                             uint32_t magic,
                             uint32_t session_uid,
                             uint32_t user_id,
                             uint32_t training_id,
                             uint32_t session_seq,
                             uint64_t time_ms,
                             uint32_t time_calibrated,
                             uint32_t source,
                             uint32_t region_type,
                             uint32_t marker_type,
                             uint32_t count,
                             uint32_t used_bytes)
{
    zy100_region_session_marker_t marker;
    uint8_t verify[ZY100_SESSION_RANGE_MARKER_BYTES];

    if ((addr % GD25Q32E_PAGE_BYTES) != 0U)
    {
        return false;
    }
    memset(&marker, 0xFF, sizeof(marker));
    marker.magic = magic;
    marker.version = ZY100_SRT_VERSION;
    marker.bytes = (uint16_t)sizeof(marker);
    marker.session_uid = session_uid;
    marker.user_id = user_id;
    marker.training_id = training_id;
    marker.session_seq = session_seq;
    marker.time_ms = time_ms;
    marker.time_calibrated = time_calibrated;
    marker.source = source;
    marker.region_type = region_type;
    marker.marker_type = marker_type;
    marker.count = count;
    marker.used_bytes = used_bytes;
    marker.crc32 = 0U;
    marker.crc32 = zy100_crc32_ieee((const uint8_t *)&marker,
                                    (uint32_t)sizeof(marker));

    if (gd25q32e_page_program(addr,
                              (const uint8_t *)&marker,
                              (uint16_t)sizeof(marker)) != IMU_STATUS_OK)
    {
        return false;
    }
    if (gd25q32e_wait_while_busy(GD25Q32E_PAGE_PROGRAM_TIMEOUT_MS) !=
        IMU_STATUS_OK)
    {
        return false;
    }
    if (gd25q32e_read(addr, verify, sizeof(verify)) != IMU_STATUS_OK)
    {
        return false;
    }
    return memcmp(verify, &marker, sizeof(marker)) == 0;
}

static uint32_t srt_new_session_uid(uint64_t start_time_ms, uint32_t seq)
{
    uint32_t uid = (uint32_t)start_time_ms ^
                   (uint32_t)(start_time_ms >> 32) ^
                   ((seq + 1U) * 2654435761UL);

    if (uid == 0U)
    {
        uid = seq + 1U;
    }
    return uid;
}

static bool srt_abort_incomplete_tail(void)
{
    zy100_session_range_entry_t entry;
    zy100_srt_wire_entry_t wire;
    uint32_t index;
    uint32_t raw_end;
    uint32_t summary_end;
    uint32_t event_end;

    if ((s_srt.incomplete_tail_index < 0) ||
        (s_srt.incomplete_tail_index != s_srt.tail_unreclaimed_index))
    {
        return false;
    }
    index = (uint32_t)s_srt.incomplete_tail_index;
    if (!srt_read_public_entry(index, &entry) ||
        !srt_read_entry(index, &wire))
    {
        return false;
    }

    if (!srt_wire_reclaim_begin(&wire))
    {
        wire.reclaim_begin_magic = 0xFFFFFFFFUL;
        if (!srt_program_entry(index, &wire))
        {
            return false;
        }
        wire.reclaim_begin_magic = ZY100_SRT_RCLB_MAGIC;
        if (!srt_program_entry(index, &wire))
        {
            return false;
        }
    }

    raw_end = srt_wire_begin_complete(&wire) ?
              ZY100_RAW_REGION_END : entry.raw_data_begin_addr;
    summary_end = srt_wire_begin_complete(&wire) ?
                  ZY100_SUMMARY_REGION_END : entry.summary_data_begin_addr;
    event_end = srt_wire_begin_complete(&wire) ?
                ZY100_EVENT_REGION_END : entry.event_data_begin_addr;

    if ((entry.raw_marker_begin_addr < raw_end) &&
        !srt_ensure_erased_range(entry.raw_marker_begin_addr,
                                 raw_end,
                                 "raw"))
    {
        return false;
    }
    if ((entry.summary_marker_begin_addr < summary_end) &&
        !srt_ensure_erased_range(entry.summary_marker_begin_addr,
                                 summary_end,
                                 "summary"))
    {
        return false;
    }
    if ((entry.event_marker_begin_addr < event_end) &&
        !srt_ensure_erased_range(entry.event_marker_begin_addr,
                                 event_end,
                                 "event"))
    {
        return false;
    }

    wire.reclaim_crc32 = 0U;
    wire.reclaim_crc32 =
        srt_wire_crc(&wire, offsetof(zy100_srt_wire_entry_t, reclaim_crc32));
    if (!srt_program_entry(index, &wire))
    {
        return false;
    }
    wire.reclaimed_magic = ZY100_SRT_RCLD_MAGIC;
    if (!srt_program_entry(index, &wire))
    {
        return false;
    }
    DBG_DIRECT("[SESSION_RECLAIM] abort_incomplete uid=%lu",
               (unsigned long)entry.session_uid);
    return srt_scan_mode(ZY100_SRT_SCAN_START_ALLOC);
}

bool zy100_session_range_table_abort_active_empty(uint32_t session_uid,
                                                  uint32_t stop_reason,
                                                  uint32_t duration_ms)
{
    zy100_session_range_entry_t entry;
    zy100_srt_wire_entry_t wire;
    uint32_t index;

    (void)stop_reason;
    if ((session_uid == 0U) ||
        !s_srt.active_valid ||
        (s_srt.active_session_uid != session_uid))
    {
        return false;
    }
    if (!srt_scan_mode(ZY100_SRT_SCAN_RUNTIME_CACHE) ||
        (s_srt.tail_unreclaimed_index < 0))
    {
        return false;
    }
    index = s_srt.active_entry_index;
    if ((s_srt.tail_unreclaimed_index != (int32_t)index) ||
        !srt_read_public_entry(index, &entry) ||
        !srt_read_entry(index, &wire) ||
        (entry.session_uid != session_uid) ||
        srt_wire_committed(&wire) ||
        srt_wire_reclaimed(&wire))
    {
        srt_clear_active();
        s_srt.dirty_need_clear = true;
        s_srt.last_alloc_status = ZY100_SESSION_ALLOC_BAD_STATE;
        DBG_DIRECT("[SESSION_ABORT] reject reason=not_tail uid=%lu",
                   (unsigned long)session_uid);
        return false;
    }

    if (!srt_wire_reclaim_begin(&wire))
    {
        wire.reclaim_begin_magic = 0xFFFFFFFFUL;
        if (!srt_program_entry(index, &wire))
        {
            return false;
        }
        wire.reclaim_begin_magic = ZY100_SRT_RCLB_MAGIC;
        if (!srt_program_entry(index, &wire))
        {
            return false;
        }
    }

    if ((entry.raw_marker_begin_addr < entry.raw_data_begin_addr) &&
        !srt_ensure_erased_range(entry.raw_marker_begin_addr,
                                 entry.raw_data_begin_addr,
                                 "raw"))
    {
        return false;
    }
    if ((entry.summary_marker_begin_addr < entry.summary_data_begin_addr) &&
        !srt_ensure_erased_range(entry.summary_marker_begin_addr,
                                 entry.summary_data_begin_addr,
                                 "summary"))
    {
        return false;
    }
    if ((entry.event_marker_begin_addr < entry.event_data_begin_addr) &&
        !srt_ensure_erased_range(entry.event_marker_begin_addr,
                                 entry.event_data_begin_addr,
                                 "event"))
    {
        return false;
    }

    wire.reclaim_crc32 = 0U;
    wire.reclaim_crc32 =
        srt_wire_crc(&wire, offsetof(zy100_srt_wire_entry_t, reclaim_crc32));
    if (!srt_program_entry(index, &wire))
    {
        return false;
    }
    wire.reclaimed_magic = ZY100_SRT_RCLD_MAGIC;
    if (!srt_program_entry(index, &wire))
    {
        return false;
    }

    DBG_DIRECT("[SESSION_ABORT] uid=%lu reason=empty_error duration_ms=%lu raw=0 sum=0 evt=0",
               (unsigned long)session_uid,
               (unsigned long)duration_ms);
    srt_clear_active();
    return srt_scan_mode(ZY100_SRT_SCAN_RUNTIME_CACHE);
}

static bool srt_make_start_ready(void)
{
    if (s_srt.reclaim_active ||
        s_srt.clear_active ||
        s_srt.clear_in_flight)
    {
        s_srt.last_alloc_status = ZY100_SESSION_ALLOC_BAD_STATE;
        return false;
    }

    if (!srt_scan_mode(ZY100_SRT_SCAN_START_ALLOC))
    {
        return false;
    }
    if (s_srt.reclaim_needed_index >= 0)
    {
        s_srt.last_alloc_status = ZY100_SESSION_ALLOC_BAD_STATE;
        return false;
    }
    if ((s_srt.incomplete_tail_index >= 0) &&
        (s_srt.incomplete_tail_index == s_srt.tail_unreclaimed_index))
    {
        if (!srt_abort_incomplete_tail())
        {
            s_srt.last_alloc_status = ZY100_SESSION_ALLOC_BAD_STATE;
            return false;
        }
    }
    if (!srt_scan_mode(ZY100_SRT_SCAN_START_ALLOC))
    {
        s_srt.last_alloc_status = ZY100_SESSION_ALLOC_BAD_STATE;
        return false;
    }
    if (s_srt.reclaim_needed_index >= 0)
    {
        s_srt.last_alloc_status = ZY100_SESSION_ALLOC_BAD_STATE;
        return false;
    }
    return !s_srt.dirty_need_clear;
}

bool zy100_session_range_table_allocate_next(
    uint32_t user_id,
    uint32_t training_id,
    uint32_t session_seq,
    uint32_t round,
    uint64_t start_time_ms,
    uint32_t time_calibrated,
    uint32_t source,
    zy100_session_range_alloc_t *out)
{
    zy100_srt_wire_entry_t wire;
    uint32_t index;
    uint32_t raw_marker;
    uint32_t raw_data;
    uint32_t summary_marker;
    uint32_t summary_data;
    uint32_t event_marker;
    uint32_t event_data;
    uint32_t raw_total =
        ZY100_FINAL_EDGE_RAW_REGION_BYTES / ZY100_FINAL_EDGE_RAW_BUCKET_BYTES;

    if (out == NULL)
    {
        s_srt.last_alloc_status = ZY100_SESSION_ALLOC_BAD_STATE;
        return false;
    }
    memset(out, 0, sizeof(*out));
    if (!srt_make_start_ready())
    {
        s_srt.last_alloc_status = ZY100_SESSION_ALLOC_BAD_STATE;
        DBG_DIRECT("[SESSION_ALLOC] reject reason=storage_dirty_need_clear");
        return false;
    }
    if (s_srt.append_pos >= srt_entry_count())
    {
        s_srt.last_alloc_status = ZY100_SESSION_ALLOC_DIR_FULL;
        DBG_DIRECT("[SESSION_ALLOC] reject reason=session_table_full");
        return false;
    }

    raw_marker = srt_align_up(s_srt.raw_tail,
                              ZY100_FINAL_EDGE_RAW_BUCKET_BYTES);
    raw_data = raw_marker + ZY100_FINAL_EDGE_RAW_BUCKET_BYTES;
    summary_marker = srt_align_up(s_srt.summary_tail, GD25Q32E_SECTOR_BYTES);
    summary_data = summary_marker + GD25Q32E_SECTOR_BYTES;
    event_marker = srt_align_up(s_srt.event_tail, GD25Q32E_SECTOR_BYTES);
    event_data = event_marker + GD25Q32E_SECTOR_BYTES;

    if (ZY100_SRT_RAW_ADDR_BEFORE_BASE(raw_marker) ||
        ((raw_data + ZY100_FINAL_EDGE_RAW_BUCKET_BYTES) > ZY100_RAW_REGION_END))
    {
        s_srt.last_alloc_status = ZY100_SESSION_ALLOC_RAW_FULL;
        DBG_DIRECT("[SESSION_ALLOC] reject reason=raw_region_full");
        return false;
    }
    if ((summary_marker < ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR) ||
        ((summary_data + GD25Q32E_SECTOR_BYTES) > ZY100_SUMMARY_REGION_END))
    {
        s_srt.last_alloc_status = ZY100_SESSION_ALLOC_SUMMARY_FULL;
        DBG_DIRECT("[SESSION_ALLOC] reject reason=summary_region_full");
        return false;
    }
    if ((event_marker < ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR) ||
        ((event_data + GD25Q32E_SECTOR_BYTES) > ZY100_EVENT_REGION_END))
    {
        s_srt.last_alloc_status = ZY100_SESSION_ALLOC_EVENT_FULL;
        DBG_DIRECT("[SESSION_ALLOC] reject reason=event_region_full");
        return false;
    }

    index = s_srt.append_pos;
    if (!srt_read_entry(index, &wire) ||
        !srt_is_all_ff((const uint8_t *)&wire, sizeof(wire)))
    {
        s_srt.last_alloc_status = ZY100_SESSION_ALLOC_DIR_FULL;
        DBG_DIRECT("[SESSION_ALLOC] reject reason=session_table_full");
        return false;
    }
    memset(&wire, 0xFF, sizeof(wire));
    wire.magic = ZY100_SRT_MAGIC;
    wire.version = ZY100_SRT_VERSION;
    wire.entry_bytes = ZY100_SESSION_RANGE_ENTRY_BYTES;
    wire.entry_seq = index + 1U;
    wire.session_uid = srt_new_session_uid(start_time_ms, index);
    wire.user_id = user_id;
    wire.training_id = training_id;
    wire.session_seq = session_seq;
    wire.round = round;
    wire.start_time_ms = start_time_ms;
    wire.time_calibrated = time_calibrated;
    wire.source = source;
    wire.raw_marker_begin_addr = raw_marker;
    wire.raw_data_begin_addr = raw_data;
    wire.summary_marker_begin_addr = summary_marker;
    wire.summary_data_begin_addr = summary_data;
    wire.event_marker_begin_addr = event_marker;
    wire.event_data_begin_addr = event_data;
    wire.begin_crc32 = 0U;
    wire.begin_crc32 =
        srt_wire_crc(&wire, offsetof(zy100_srt_wire_entry_t, begin_crc32));
    wire.alloc_magic = 0xFFFFFFFFUL;
    if (!srt_program_entry(index, &wire))
    {
        s_srt.last_alloc_status = ZY100_SESSION_ALLOC_FLASH_NOT_READY;
        return false;
    }
    wire.alloc_magic = ZY100_SRT_ALLOC_MAGIC;
    if (!srt_program_entry(index, &wire))
    {
        s_srt.last_alloc_status = ZY100_SESSION_ALLOC_FLASH_NOT_READY;
        return false;
    }

    out->entry_seq = wire.entry_seq;
    out->entry_addr = srt_entry_addr(index);
    out->session_uid = wire.session_uid;
    out->user_id = user_id;
    out->training_id = training_id;
    out->session_seq = session_seq;
    out->round = round;
    out->start_time_ms = start_time_ms;
    out->time_calibrated = time_calibrated;
    out->source = source;
    out->raw_marker_begin_addr = raw_marker;
    out->raw_data_begin_addr = raw_data;
    out->raw_data_end_addr = 0xFFFFFFFFUL;
    out->raw_reclaim_end_addr = 0xFFFFFFFFUL;
    out->raw_first_bucket =
        (raw_data - ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR) /
        ZY100_FINAL_EDGE_RAW_BUCKET_BYTES;
    out->raw_limit_bucket = raw_total;
    out->raw_reserved_bucket_count = raw_total - out->raw_first_bucket;
    out->raw_bucket_bytes = ZY100_FINAL_EDGE_RAW_BUCKET_BYTES;
    out->summary_marker_begin_addr = summary_marker;
    out->summary_data_begin_addr = summary_data;
    out->summary_data_end_addr = 0xFFFFFFFFUL;
    out->summary_reclaim_end_addr = 0xFFFFFFFFUL;
    out->summary_base_addr = summary_data;
    out->summary_limit_addr = ZY100_SUMMARY_REGION_END;
    out->summary_reserved_bytes = ZY100_SUMMARY_REGION_END - summary_data;
    out->event_marker_begin_addr = event_marker;
    out->event_data_begin_addr = event_data;
    out->event_data_end_addr = 0xFFFFFFFFUL;
    out->event_reclaim_end_addr = 0xFFFFFFFFUL;
    out->event_base_addr = event_data;
    out->event_limit_addr = ZY100_EVENT_REGION_END;
    out->event_reserved_bytes = ZY100_EVENT_REGION_END - event_data;
    s_srt.last_alloc_status = ZY100_SESSION_ALLOC_OK;
    DBG_DIRECT("[SESSION_BEGIN] uid=%lu raw=0x%08lX summary=0x%08lX event=0x%08lX",
               (unsigned long)out->session_uid,
               (unsigned long)out->raw_marker_begin_addr,
               (unsigned long)out->summary_marker_begin_addr,
               (unsigned long)out->event_marker_begin_addr);
    srt_set_active(index, out->session_uid);
    return srt_scan_mode(ZY100_SRT_SCAN_RUNTIME_CACHE);
}

zy100_session_alloc_status_t zy100_session_range_table_last_alloc_status(void)
{
    return s_srt.last_alloc_status;
}

bool zy100_session_range_table_write_begin_complete(
    const zy100_session_range_alloc_t *alloc,
    const uint8_t training_begin[ZY100_TRAINING_RECORD_BYTES])
{
    zy100_srt_wire_entry_t wire;
    uint32_t index;

    (void)training_begin;
    if ((alloc == NULL) || (alloc->entry_seq == 0U))
    {
        return false;
    }
    index = alloc->entry_seq - 1U;
    if (!srt_read_entry(index, &wire) ||
        !srt_wire_alloc_valid(&wire) ||
        (wire.session_uid != alloc->session_uid) ||
        srt_wire_begin_complete(&wire))
    {
        return false;
    }

    if (!srt_ensure_erased_range(alloc->raw_marker_begin_addr,
                                 alloc->raw_data_begin_addr,
                                 "raw") ||
        !srt_write_marker(alloc->raw_marker_begin_addr,
                          ZY100_REGION_MAGIC_RAW_BEGIN,
                          alloc->session_uid,
                          alloc->user_id,
                          alloc->training_id,
                          alloc->session_seq,
                          alloc->start_time_ms,
                          alloc->time_calibrated,
                          alloc->source,
                          ZY100_REGION_TYPE_RAW,
                          ZY100_MARKER_TYPE_BEGIN,
                          0U,
                          0U))
    {
        return false;
    }
    if (!srt_ensure_erased_range(alloc->summary_marker_begin_addr,
                                 alloc->summary_data_begin_addr,
                                 "summary") ||
        !srt_write_marker(alloc->summary_marker_begin_addr,
                          ZY100_REGION_MAGIC_SUMMARY_BEGIN,
                          alloc->session_uid,
                          alloc->user_id,
                          alloc->training_id,
                          alloc->session_seq,
                          alloc->start_time_ms,
                          alloc->time_calibrated,
                          alloc->source,
                          ZY100_REGION_TYPE_SUMMARY,
                          ZY100_MARKER_TYPE_BEGIN,
                          0U,
                          0U))
    {
        return false;
    }
    if (!srt_ensure_erased_range(alloc->event_marker_begin_addr,
                                 alloc->event_data_begin_addr,
                                 "event") ||
        !srt_write_marker(alloc->event_marker_begin_addr,
                          ZY100_REGION_MAGIC_EVENT_BEGIN,
                          alloc->session_uid,
                          alloc->user_id,
                          alloc->training_id,
                          alloc->session_seq,
                          alloc->start_time_ms,
                          alloc->time_calibrated,
                          alloc->source,
                          ZY100_REGION_TYPE_EVENT,
                          ZY100_MARKER_TYPE_BEGIN,
                          0U,
                          0U))
    {
        return false;
    }

    if (!srt_read_entry(index, &wire) ||
        !srt_wire_alloc_valid(&wire))
    {
        return false;
    }
    wire.begin_complete_magic = ZY100_SRT_BEGIN_MAGIC;
    if (!srt_program_entry(index, &wire))
    {
        return false;
    }
    srt_set_active(index, alloc->session_uid);
    s_srt.scanned = false;
    return true;
}

bool zy100_session_range_table_commit(
    const zy100_session_range_commit_t *commit,
    const uint8_t training_end[ZY100_TRAINING_RECORD_BYTES])
{
    zy100_srt_wire_entry_t wire;
    uint32_t index;
    uint32_t raw_end_marker;
    uint32_t summary_end_marker;
    uint32_t event_end_marker;

    (void)training_end;
    if ((commit == NULL) || (commit->session_uid == 0U))
    {
        return false;
    }
    if (!srt_scan_if_needed() || (s_srt.tail_unreclaimed_index < 0))
    {
        return false;
    }
    index = (uint32_t)s_srt.tail_unreclaimed_index;
    if (!srt_read_entry(index, &wire) ||
        !srt_wire_alloc_valid(&wire) ||
        !srt_wire_begin_complete(&wire) ||
        srt_wire_committed(&wire) ||
        (wire.session_uid != commit->session_uid))
    {
        return false;
    }

    wire.raw_data_end_addr =
        wire.raw_data_begin_addr +
        (commit->raw_bucket_count * ZY100_FINAL_EDGE_RAW_BUCKET_BYTES);
    raw_end_marker = srt_align_up(wire.raw_data_end_addr,
                                  ZY100_FINAL_EDGE_RAW_BUCKET_BYTES);
    wire.raw_reclaim_end_addr =
        raw_end_marker + ZY100_FINAL_EDGE_RAW_BUCKET_BYTES;

    wire.summary_data_end_addr =
        wire.summary_data_begin_addr + commit->summary_used_bytes;
    summary_end_marker = srt_align_up(wire.summary_data_end_addr,
                                      GD25Q32E_SECTOR_BYTES);
    wire.summary_reclaim_end_addr =
        summary_end_marker + GD25Q32E_SECTOR_BYTES;

    wire.event_data_end_addr =
        wire.event_data_begin_addr + commit->event_used_bytes;
    event_end_marker = srt_align_up(wire.event_data_end_addr,
                                    GD25Q32E_SECTOR_BYTES);
    wire.event_reclaim_end_addr =
        event_end_marker + GD25Q32E_SECTOR_BYTES;

    if ((wire.raw_reclaim_end_addr > ZY100_RAW_REGION_END) ||
        (wire.summary_reclaim_end_addr > ZY100_SUMMARY_REGION_END) ||
        (wire.event_reclaim_end_addr > ZY100_EVENT_REGION_END))
    {
        return false;
    }

    if (!srt_ensure_erased_range(raw_end_marker,
                                 wire.raw_reclaim_end_addr,
                                 "raw") ||
        !srt_write_marker(raw_end_marker,
                          ZY100_REGION_MAGIC_RAW_END,
                          commit->session_uid,
                          commit->user_id,
                          commit->training_id,
                          commit->session_seq,
                          commit->end_time_ms,
                          commit->time_calibrated,
                          commit->source,
                          ZY100_REGION_TYPE_RAW,
                          ZY100_MARKER_TYPE_END,
                          commit->raw_bucket_count,
                          commit->raw_used_bytes))
    {
        return false;
    }
    if (!srt_ensure_erased_range(summary_end_marker,
                                 wire.summary_reclaim_end_addr,
                                 "summary") ||
        !srt_write_marker(summary_end_marker,
                          ZY100_REGION_MAGIC_SUMMARY_END,
                          commit->session_uid,
                          commit->user_id,
                          commit->training_id,
                          commit->session_seq,
                          commit->end_time_ms,
                          commit->time_calibrated,
                          commit->source,
                          ZY100_REGION_TYPE_SUMMARY,
                          ZY100_MARKER_TYPE_END,
                          commit->summary_count,
                          commit->summary_used_bytes))
    {
        return false;
    }
    if (!srt_ensure_erased_range(event_end_marker,
                                 wire.event_reclaim_end_addr,
                                 "event") ||
        !srt_write_marker(event_end_marker,
                          ZY100_REGION_MAGIC_EVENT_END,
                          commit->session_uid,
                          commit->user_id,
                          commit->training_id,
                          commit->session_seq,
                          commit->end_time_ms,
                          commit->time_calibrated,
                          commit->source,
                          ZY100_REGION_TYPE_EVENT,
                          ZY100_MARKER_TYPE_END,
                          commit->event_count,
                          commit->event_used_bytes))
    {
        return false;
    }

    wire.end_time_ms = commit->end_time_ms;
    wire.stop_reason = commit->stop_reason;
    wire.raw_count = commit->raw_bucket_count;
    wire.raw_used_bytes = commit->raw_used_bytes;
    wire.summary_count = commit->summary_count;
    wire.summary_used_bytes = commit->summary_used_bytes;
    wire.event_count = commit->event_count;
    wire.event_used_bytes = commit->event_used_bytes;
    wire.official_hit_count = commit->official_hit_count;
    wire.nohit_count = commit->nohit_count;
    wire.live_hz = ZY100_FINAL_EDGE_LIVE_UI_HZ;
    wire.ois_hz = ZY100_FINAL_EDGE_OIS_HZ;
    wire.pass_flags = commit->pass_flags;
    wire.warn_flags = commit->warn_flags;
    wire.error_flags = commit->error_flags;
    wire.flags = commit->flags;
    memcpy(wire.rtc_meta, commit->rtc_meta, sizeof(wire.rtc_meta));
    wire.commit_crc32 = 0U;
    wire.commit_magic = 0xFFFFFFFFUL;
    wire.commit_crc32 =
        srt_wire_crc(&wire, offsetof(zy100_srt_wire_entry_t, commit_crc32));
    if (!srt_program_entry(index, &wire))
    {
        return false;
    }
    wire.commit_magic = ZY100_SRT_COMMIT_MAGIC;
    if (!srt_program_entry(index, &wire))
    {
        return false;
    }

    DBG_DIRECT("[SESSION_END] uid=%lu raw=0x%08lX..0x%08lX summary=0x%08lX..0x%08lX event=0x%08lX..0x%08lX",
               (unsigned long)commit->session_uid,
               (unsigned long)wire.raw_data_begin_addr,
               (unsigned long)wire.raw_data_end_addr,
               (unsigned long)wire.summary_data_begin_addr,
               (unsigned long)wire.summary_data_end_addr,
               (unsigned long)wire.event_data_begin_addr,
               (unsigned long)wire.event_data_end_addr);
    DBG_DIRECT("[SESSION_COMMIT] uid=%lu raw=%lu sum=%lu evt=%lu stop_reason=%u",
               (unsigned long)commit->session_uid,
               (unsigned long)commit->raw_bucket_count,
               (unsigned long)commit->summary_count,
               (unsigned long)commit->event_count,
               commit->stop_reason);
    if (srt_active_matches(index, &wire))
    {
        srt_clear_active();
    }
    return zy100_session_range_table_scan();
}

bool zy100_session_range_table_confirm_export_tail(
    uint32_t session_uid,
    uint32_t stream_crc32,
    uint32_t bytes_sent)
{
    zy100_session_range_entry_t tail;
    zy100_srt_wire_entry_t wire;
    uint32_t index;

    if ((session_uid == 0U) ||
        !zy100_session_range_table_get_tail_pending(&tail) ||
        (tail.session_uid != session_uid))
    {
        return false;
    }
    index = tail.entry_seq - 1U;
    if (!srt_read_entry(index, &wire) ||
        !srt_wire_committed(&wire) ||
        srt_wire_export_confirmed(&wire))
    {
        return false;
    }
    wire.export_stream_crc32 = stream_crc32;
    wire.export_bytes_sent = bytes_sent;
    wire.export_confirmed_magic = 0xFFFFFFFFUL;
    if (!srt_program_entry(index, &wire))
    {
        return false;
    }
    wire.export_confirmed_magic = ZY100_SRT_EXPC_MAGIC;
    if (!srt_program_entry(index, &wire))
    {
        return false;
    }
    DBG_DIRECT("[SESSION_EXPORTED] uid=%lu",
               (unsigned long)session_uid);
    return zy100_session_range_table_scan();
}

bool zy100_session_range_table_reclaim_tail_begin(uint32_t session_uid)
{
    zy100_session_range_entry_t tail;
    zy100_srt_wire_entry_t wire;
    uint32_t index;

    if ((session_uid == 0U) ||
        !zy100_session_range_table_get_tail_unreclaimed(&tail) ||
        (tail.session_uid != session_uid))
    {
        return false;
    }
    if ((tail.raw_reclaim_end_addr != s_srt.raw_tail) ||
        (tail.summary_reclaim_end_addr != s_srt.summary_tail) ||
        (tail.event_reclaim_end_addr != s_srt.event_tail))
    {
        DBG_DIRECT("[SESSION_RECLAIM] reject reason=not_tail uid=%lu raw_end=0x%08lX raw_tail=0x%08lX sum_end=0x%08lX sum_tail=0x%08lX evt_end=0x%08lX evt_tail=0x%08lX",
                   (unsigned long)session_uid,
                   (unsigned long)tail.raw_reclaim_end_addr,
                   (unsigned long)s_srt.raw_tail,
                   (unsigned long)tail.summary_reclaim_end_addr,
                   (unsigned long)s_srt.summary_tail,
                   (unsigned long)tail.event_reclaim_end_addr,
                   (unsigned long)s_srt.event_tail);
        return false;
    }
    if ((tail.state != ZY100_SESSION_STATE_EXPORT_CONFIRMED) &&
        (tail.state != ZY100_SESSION_STATE_RECLAIMING))
    {
        return false;
    }

    index = tail.entry_seq - 1U;
    if (!srt_read_entry(index, &wire) ||
        !srt_wire_export_confirmed(&wire))
    {
        return false;
    }
    if (!srt_wire_reclaim_begin(&wire))
    {
        wire.reclaim_begin_magic = 0xFFFFFFFFUL;
        if (!srt_program_entry(index, &wire))
        {
            return false;
        }
        wire.reclaim_begin_magic = ZY100_SRT_RCLB_MAGIC;
        if (!srt_program_entry(index, &wire))
        {
            return false;
        }
    }
    s_srt.reclaim_active = true;
    s_srt.reclaim_phase = ZY100_SRT_RECLAIM_RAW;
    s_srt.reclaim_entry = tail;
    s_srt.reclaim_next_addr = tail.raw_marker_begin_addr;
    s_srt.reclaim_end_addr = tail.raw_reclaim_end_addr;
    return true;
}

zy100_flash_prepare_status_t zy100_session_range_table_reclaim_tail_poll(void)
{
    zy100_srt_wire_entry_t wire;
    uint32_t index;

    if (!s_srt.reclaim_active)
    {
        if (s_srt.reclaim_needed_index >= 0)
        {
            return ZY100_FLASH_PREP_ERROR;
        }
        return ZY100_FLASH_PREP_DONE;
    }

    if (s_srt.reclaim_phase == ZY100_SRT_RECLAIM_RAW)
    {
        if (s_srt.reclaim_next_addr < s_srt.reclaim_end_addr)
        {
            return srt_reclaim_erase_one_sector("raw") ?
                   ZY100_FLASH_PREP_BUSY : ZY100_FLASH_PREP_ERROR;
        }
        s_srt.reclaim_phase = ZY100_SRT_RECLAIM_SUMMARY;
        s_srt.reclaim_next_addr = s_srt.reclaim_entry.summary_marker_begin_addr;
        s_srt.reclaim_end_addr = s_srt.reclaim_entry.summary_reclaim_end_addr;
        return ZY100_FLASH_PREP_BUSY;
    }
    if (s_srt.reclaim_phase == ZY100_SRT_RECLAIM_SUMMARY)
    {
        if (s_srt.reclaim_next_addr < s_srt.reclaim_end_addr)
        {
            return srt_reclaim_erase_one_sector("summary") ?
                   ZY100_FLASH_PREP_BUSY : ZY100_FLASH_PREP_ERROR;
        }
        s_srt.reclaim_phase = ZY100_SRT_RECLAIM_EVENT;
        s_srt.reclaim_next_addr = s_srt.reclaim_entry.event_marker_begin_addr;
        s_srt.reclaim_end_addr = s_srt.reclaim_entry.event_reclaim_end_addr;
        return ZY100_FLASH_PREP_BUSY;
    }
    if (s_srt.reclaim_phase == ZY100_SRT_RECLAIM_EVENT)
    {
        if (s_srt.reclaim_next_addr < s_srt.reclaim_end_addr)
        {
            return srt_reclaim_erase_one_sector("event") ?
                   ZY100_FLASH_PREP_BUSY : ZY100_FLASH_PREP_ERROR;
        }
        s_srt.reclaim_phase = ZY100_SRT_RECLAIM_FINISH;
        return ZY100_FLASH_PREP_BUSY;
    }

    index = s_srt.reclaim_entry.entry_seq - 1U;
    if (!srt_read_entry(index, &wire) ||
        !srt_wire_export_confirmed(&wire) ||
        !srt_wire_reclaim_begin(&wire))
    {
        s_srt.reclaim_active = false;
        return ZY100_FLASH_PREP_ERROR;
    }
    wire.reclaim_crc32 = 0U;
    wire.reclaim_crc32 =
        srt_wire_crc(&wire, offsetof(zy100_srt_wire_entry_t, reclaim_crc32));
    if (!srt_program_entry(index, &wire))
    {
        s_srt.reclaim_active = false;
        return ZY100_FLASH_PREP_ERROR;
    }
    wire.reclaimed_magic = ZY100_SRT_RCLD_MAGIC;
    if (!srt_program_entry(index, &wire))
    {
        s_srt.reclaim_active = false;
        return ZY100_FLASH_PREP_ERROR;
    }

    s_srt.reclaim_active = false;
    s_srt.reclaim_phase = ZY100_SRT_RECLAIM_IDLE;
    (void)zy100_session_range_table_scan();
    (void)zy100_session_range_table_reclaim_tail_sector_if_possible();
    return ZY100_FLASH_PREP_DONE;
}

bool zy100_session_range_table_reclaim_in_progress(void)
{
    return s_srt.reclaim_active;
}

bool zy100_session_range_table_reclaim_tail_sector_if_possible(void)
{
    uint32_t sector;
    uint32_t last_sector = 0xFFFFFFFFUL;
    uint32_t entries_per_sector =
        ZY100_INTERNAL_META_STORE_SECTOR_BYTES /
        ZY100_SESSION_RANGE_ENTRY_BYTES;
    uint32_t sector_index;
    uint32_t first_entry;
    uint32_t i;
    uint32_t non_ff = 0U;

    for (sector = ZY100_SESSION_DIR_REGION_BASE_ADDR;
         sector < ZY100_SRT_TABLE_END;
         sector += ZY100_INTERNAL_META_STORE_SECTOR_BYTES)
    {
        if (!srt_meta_sector_erased(sector))
        {
            last_sector = sector;
        }
    }
    if (last_sector == 0xFFFFFFFFUL)
    {
        return true;
    }

    sector_index =
        (last_sector - ZY100_SESSION_DIR_REGION_BASE_ADDR) /
        ZY100_INTERNAL_META_STORE_SECTOR_BYTES;
    first_entry = sector_index * entries_per_sector;
    for (i = 0U; i < entries_per_sector; i++)
    {
        zy100_srt_wire_entry_t wire;
        uint32_t entry_index = first_entry + i;

        if (entry_index >= srt_entry_count())
        {
            break;
        }
        if (!srt_read_entry(entry_index, &wire))
        {
            return false;
        }
        if (srt_is_all_ff((const uint8_t *)&wire, sizeof(wire)))
        {
            continue;
        }
        non_ff++;
        if (!srt_wire_alloc_valid(&wire) || !srt_wire_reclaimed(&wire))
        {
            ZY100_LOG_VERBOSE("[SESSION_TABLE] tail_sector_reclaim_skip reason=has_pending sector_index=%u",
                              sector_index);
            return true;
        }
    }
    if (non_ff == 0U)
    {
        return true;
    }
    ZY100_LOG_VERBOSE("[SESSION_TABLE] tail_sector_reclaim sector_index=%u entries=%u",
                      sector_index,
                      non_ff);
    if (!srt_meta_erase_sector(last_sector))
    {
        return false;
    }
    if (!zy100_session_range_table_scan())
    {
        return false;
    }
    {
        zy100_session_range_entry_t tail;
        uint32_t tail_uid = zy100_session_range_table_get_tail_pending(&tail) ?
                            tail.session_uid : 0U;
        ZY100_LOG_VERBOSE("[SESSION_TABLE] rescan_after_reclaim append_pos=%u pending=%u tail_uid=%lu",
                          s_srt.append_pos,
                          s_srt.pending_count,
                          (unsigned long)tail_uid);
    }
    return true;
}

bool zy100_session_range_table_reset_all_after_bulk_clear(void)
{
    memset(&s_srt, 0, sizeof(s_srt));
    s_srt.last_alloc_status = ZY100_SESSION_ALLOC_OK;
    s_srt.raw_tail = ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR;
    s_srt.summary_tail = ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR;
    s_srt.event_tail = ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR;
    s_srt.tail_pending_index = -1;
    s_srt.tail_unreclaimed_index = -1;
    s_srt.reclaim_needed_index = -1;
    s_srt.incomplete_tail_index = -1;
    s_srt.scanned = true;
    return true;
}

bool zy100_session_range_table_prepare_clear_begin(void)
{
    if (!zy100_internal_meta_store_ready())
    {
        return false;
    }
    srt_clear_active();
    s_srt.clear_active = true;
    s_srt.clear_in_flight = false;
    s_srt.clear_next_sector = ZY100_FINAL_EDGE_META_REGION_BASE_ADDR;
    s_srt.clear_total_sectors =
        ZY100_FINAL_EDGE_META_REGION_BYTES /
        ZY100_INTERNAL_META_STORE_SECTOR_BYTES;
    s_srt.clear_done_sectors = 0U;
    return true;
}

zy100_flash_prepare_status_t zy100_session_range_table_prepare_clear_poll(void)
{
    if (!s_srt.clear_active)
    {
        return ZY100_FLASH_PREP_DONE;
    }
    if (s_srt.clear_done_sectors >= s_srt.clear_total_sectors)
    {
        s_srt.clear_active = false;
        (void)zy100_session_range_table_reset_all_after_bulk_clear();
        return ZY100_FLASH_PREP_DONE;
    }
    if (!srt_meta_erase_sector(s_srt.clear_next_sector))
    {
        s_srt.clear_active = false;
        return ZY100_FLASH_PREP_ERROR;
    }
    s_srt.clear_next_sector += ZY100_INTERNAL_META_STORE_SECTOR_BYTES;
    s_srt.clear_done_sectors++;
    if (s_srt.clear_done_sectors >= s_srt.clear_total_sectors)
    {
        s_srt.clear_active = false;
        (void)zy100_session_range_table_reset_all_after_bulk_clear();
        return ZY100_FLASH_PREP_DONE;
    }
    return ZY100_FLASH_PREP_BUSY;
}

bool zy100_session_range_table_build_session_meta(
    const zy100_session_range_entry_t *session,
    zy100_fe_session_meta_t *out)
{
    zy100_fe_session_meta_input_t input;

    if ((session == NULL) || (out == NULL))
    {
        return false;
    }
    memset(&input, 0, sizeof(input));
    input.round = session->round;
    input.start_ms = (uint32_t)session->start_time_ms;
    input.end_ms = (uint32_t)session->end_time_ms;
    input.official_hit_count = session->official_hit_count;
    input.nohit_count = session->nohit_count;
    input.raw_count = session->raw_bucket_count;
    input.raw_used_bytes = session->raw_used_bytes;
    input.pass_flags = session->pass_flags;
    input.warn_flags = session->warn_flags;
    input.error_flags = session->error_flags;
    memcpy(input.rtc_meta, session->rtc_meta, sizeof(input.rtc_meta));
    return zy100_final_edge_record_store_build_session_meta(&input, out);
}

void zy100_session_range_table_get_tails(uint32_t *raw_tail,
                                         uint32_t *summary_tail,
                                         uint32_t *event_tail,
                                         uint32_t *append_pos)
{
    (void)srt_scan_if_needed();
    if (raw_tail != NULL)
    {
        *raw_tail = s_srt.raw_tail;
    }
    if (summary_tail != NULL)
    {
        *summary_tail = s_srt.summary_tail;
    }
    if (event_tail != NULL)
    {
        *event_tail = s_srt.event_tail;
    }
    if (append_pos != NULL)
    {
        *append_pos = s_srt.append_pos;
    }
}

#else

bool zy100_session_range_table_scan(void) { return false; }
bool zy100_session_range_table_scan_boot_recovery(void) { return false; }
uint32_t zy100_session_range_table_pending_count(void) { return 0U; }
uint32_t zy100_session_range_table_total_committed_count(void) { return 0U; }
uint32_t zy100_session_range_table_reclaimed_count(void) { return 0U; }
bool zy100_session_range_table_storage_dirty_need_clear(void) { return false; }
bool zy100_session_range_table_refresh_readiness(
    zy100_session_readiness_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
        out->last_alloc_status = ZY100_SESSION_ALLOC_BAD_STATE;
    }
    return false;
}
bool zy100_session_range_table_get_tail_pending(zy100_session_range_entry_t *out)
{
    (void)out;
    return false;
}
bool zy100_session_range_table_get_tail_unreclaimed(zy100_session_range_entry_t *out)
{
    (void)out;
    return false;
}
bool zy100_session_range_table_allocate_next(uint32_t user_id,
                                             uint32_t training_id,
                                             uint32_t session_seq,
                                             uint32_t round,
                                             uint64_t start_time_ms,
                                             uint32_t time_calibrated,
                                             uint32_t source,
                                             zy100_session_range_alloc_t *out)
{
    (void)user_id;
    (void)training_id;
    (void)session_seq;
    (void)round;
    (void)start_time_ms;
    (void)time_calibrated;
    (void)source;
    (void)out;
    return false;
}
zy100_session_alloc_status_t zy100_session_range_table_last_alloc_status(void)
{
    return ZY100_SESSION_ALLOC_BAD_STATE;
}
bool zy100_session_range_table_write_begin_complete(
    const zy100_session_range_alloc_t *alloc,
    const uint8_t training_begin[ZY100_TRAINING_RECORD_BYTES])
{
    (void)alloc;
    (void)training_begin;
    return false;
}
bool zy100_session_range_table_commit(
    const zy100_session_range_commit_t *commit,
    const uint8_t training_end[ZY100_TRAINING_RECORD_BYTES])
{
    (void)commit;
    (void)training_end;
    return false;
}
bool zy100_session_range_table_abort_active_empty(uint32_t session_uid,
                                                  uint32_t stop_reason,
                                                  uint32_t duration_ms)
{
    (void)session_uid;
    (void)stop_reason;
    (void)duration_ms;
    return false;
}
bool zy100_session_range_table_confirm_export_tail(uint32_t session_uid,
                                                   uint32_t stream_crc32,
                                                   uint32_t bytes_sent)
{
    (void)session_uid;
    (void)stream_crc32;
    (void)bytes_sent;
    return false;
}
bool zy100_session_range_table_reclaim_tail_begin(uint32_t session_uid)
{
    (void)session_uid;
    return false;
}
zy100_flash_prepare_status_t zy100_session_range_table_reclaim_tail_poll(void)
{
    return ZY100_FLASH_PREP_DONE;
}
bool zy100_session_range_table_reclaim_in_progress(void) { return false; }
bool zy100_session_range_table_reclaim_tail_sector_if_possible(void) { return true; }
bool zy100_session_range_table_reset_all_after_bulk_clear(void) { return true; }
bool zy100_session_range_table_prepare_clear_begin(void) { return false; }
zy100_flash_prepare_status_t zy100_session_range_table_prepare_clear_poll(void)
{
    return ZY100_FLASH_PREP_DONE;
}
bool zy100_session_range_table_build_session_meta(
    const zy100_session_range_entry_t *session,
    zy100_fe_session_meta_t *out)
{
    (void)session;
    (void)out;
    return false;
}
void zy100_session_range_table_get_tails(uint32_t *raw_tail,
                                         uint32_t *summary_tail,
                                         uint32_t *event_tail,
                                         uint32_t *append_pos)
{
    (void)raw_tail;
    (void)summary_tail;
    (void)event_tail;
    (void)append_pos;
}

#endif
