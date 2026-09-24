#include "zy100_final_edge_record_store.h"

#include <string.h>

#include "trace.h"

#include "../app_flags.h"
#include "../bsp/imu_bsp.h"
#include "../driver/gd25q32e_spi.h"
#include "zy100_final_edge_queue.h"
#include "zy100_final_edge_record_format.h"
#include "zy100_capture_time.h"
#include "zy100_internal_meta_store.h"

#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_FINAL_EDGE_PHASE7_EDGE_FLASH_ENABLE

#define ZY100_FE_REC_PAGE_BYTES GD25Q32E_PAGE_BYTES
#define ZY100_FE_REC_SECTOR_BYTES GD25Q32E_SECTOR_BYTES
#define ZY100_FE_REC_VERIFY_CHUNK_BYTES 16U
#define ZY100_FE_REC_PREP_LOG_STEP 32U
#define ZY100_FE_REC_META_ERASE_BYTES GD25Q32E_SECTOR_BYTES

typedef char zy100_fe_rec_page_size_check[
    (ZY100_FE_REC_PAGE_BYTES == 256U) ? 1 : -1];
typedef char zy100_fe_rec_header_page_check[
    (sizeof(zy100_fe_record_flash_header_t) < ZY100_FE_REC_PAGE_BYTES) ?
        1 : -1];
typedef char zy100_fe_session_meta_page_check[
    (sizeof(zy100_fe_session_meta_t) <= ZY100_FE_REC_PAGE_BYTES) ? 1 : -1];
typedef char zy100_fe_session_rtc_meta_reserved_check[
    (sizeof(((zy100_fe_session_meta_t *)0)->reserved) >=
     ZY100_CAPTURE_TIME_META_BYTES) ? 1 : -1];
typedef char zy100_fe_rec_summary_bound_check[
    (sizeof(zy100_edge_summary_record_t) <=
     ZY100_FINAL_EDGE_SUMMARY_RECORD_MAX_BYTES) ? 1 : -1];
typedef char zy100_fe_rec_event_bound_check[
    (sizeof(zy100_edge_event_record_t) <=
     ZY100_FINAL_EDGE_EVENT_RECORD_MAX_BYTES) ? 1 : -1];
typedef char zy100_fe_rec_event_staging_check[
    (sizeof(zy100_edge_event_record_t) <= 384U) ? 1 : -1];
typedef char zy100_fe_rec_summary_base_check[
    ((ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR %
      ZY100_FE_REC_SECTOR_BYTES) == 0UL) ? 1 : -1];
typedef char zy100_fe_rec_summary_size_check[
    ((ZY100_FINAL_EDGE_SUMMARY_REGION_BYTES %
      ZY100_FE_REC_SECTOR_BYTES) == 0UL) ? 1 : -1];
typedef char zy100_fe_rec_event_base_check[
    ((ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR %
      ZY100_FE_REC_SECTOR_BYTES) == 0UL) ? 1 : -1];
typedef char zy100_fe_rec_event_size_check[
    ((ZY100_FINAL_EDGE_EVENT_REGION_BYTES %
      ZY100_FE_REC_SECTOR_BYTES) == 0UL) ? 1 : -1];
typedef char zy100_fe_rec_meta_base_check[
    ((ZY100_FINAL_EDGE_META_REGION_BASE_ADDR %
      ZY100_FE_REC_SECTOR_BYTES) == 0UL) ? 1 : -1];
typedef char zy100_fe_rec_meta_size_check[
    ((ZY100_FINAL_EDGE_META_REGION_BYTES %
      ZY100_FE_REC_SECTOR_BYTES) == 0UL) ? 1 : -1];
typedef char zy100_fe_rec_meta_addr_check[
    ((ZY100_FINAL_EDGE_SESSION_META_ADDR %
      ZY100_FE_REC_SECTOR_BYTES) == 0UL) ? 1 : -1];
typedef char zy100_fe_rec_meta_addr_range_check[
    ((ZY100_FINAL_EDGE_SESSION_META_ADDR >=
      ZY100_FINAL_EDGE_META_REGION_BASE_ADDR) &&
     ((ZY100_FINAL_EDGE_SESSION_META_ADDR + ZY100_FE_REC_SECTOR_BYTES) <=
      (ZY100_FINAL_EDGE_META_REGION_BASE_ADDR +
       ZY100_FINAL_EDGE_META_REGION_BYTES))) ? 1 : -1];
#if !ZY100_FINAL_EDGE_INTERNAL_META_ENABLE
typedef char zy100_fe_rec_meta_raw_overlap_check[
    (((ZY100_FINAL_EDGE_SESSION_META_ADDR + ZY100_FE_REC_SECTOR_BYTES) <=
      ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR) ||
     (ZY100_FINAL_EDGE_SESSION_META_ADDR >=
      (ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR +
       ZY100_FINAL_EDGE_RAW_REGION_BYTES))) ? 1 : -1];
#endif
typedef char zy100_fe_rec_region_order_check[
    ((ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR +
      ZY100_FINAL_EDGE_SUMMARY_REGION_BYTES) <=
     ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR) ? 1 : -1];
typedef char zy100_fe_rec_flash_size_check[
    ((ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR +
      ZY100_FINAL_EDGE_EVENT_REGION_BYTES) <=
     GD25Q32E_FLASH_SIZE_BYTES) ? 1 : -1];

typedef enum
{
    ZY100_FE_REC_PREP_IDLE = 0U,
    ZY100_FE_REC_PREP_ERASE,
    ZY100_FE_REC_PREP_DONE,
    ZY100_FE_REC_PREP_ERROR,
} zy100_fe_rec_prep_state_t;

typedef enum
{
    ZY100_FE_REC_PREP_PHASE_NONE = 0U,
    ZY100_FE_REC_PREP_PHASE_SUMMARY = 2U,
    ZY100_FE_REC_PREP_PHASE_EVENT = 3U,
    ZY100_FE_REC_PREP_PHASE_META = 4U,
} zy100_fe_rec_prep_phase_t;

typedef union
{
    zy100_final_edge_summary_copy_t summary;
    zy100_final_edge_event_copy_t event;
} zy100_fe_rec_active_copy_t;

typedef struct
{
    zy100_fe_rec_prep_state_t prep_state;
    zy100_fe_rec_prep_phase_t prep_phase;

    bool prepared;
    bool running;
    bool active;
    bool pending;
    bool program_in_flight;
    bool prep_erase_in_flight;
    bool b_active;
    bool error;

    uint32_t prep_round;
    uint32_t run_round;
    uint32_t prep_phase_base;
    uint32_t prep_phase_total;
    uint32_t prep_phase_done;
    uint32_t prep_phase_last_log;
    bool prep_erase_meta_scratch;

    uint32_t summary_base_addr;
    uint32_t summary_limit_addr;
    uint32_t summary_next_addr;
    uint32_t summary_erase_tail_addr;
    uint32_t event_base_addr;
    uint32_t event_limit_addr;
    uint32_t event_next_addr;
    uint32_t event_erase_tail_addr;
    bool append_erase_dynamic;

    uint32_t active_type;
    uint32_t active_seq;
    uint32_t active_edge_sample_index;
    uint32_t active_source_flags;
    uint32_t active_payload_bytes;
    uint32_t active_addr;
    uint32_t active_record_bytes;
    uint32_t active_page_count;
    uint32_t active_next_page;
    const zy100_fe_store_target_provider_t *target_provider;
    uint32_t target_token;

    uint32_t in_flight_addr;
    uint32_t in_flight_page;
    uint8_t page[ZY100_FE_REC_PAGE_BYTES];
    zy100_fe_rec_active_copy_t active_copy;

    zy100_fe_record_store_stats_t stats;
} zy100_fe_record_store_t;

static zy100_fe_record_store_t s_fe_record_store;
static uint32_t s_fe_record_online_verified_pages;
static uint32_t s_fe_record_online_address_violations;

static uint32_t fe_rec_elapsed_us(uint32_t start_us)
{
    return (uint32_t)((uint32_t)imu_bsp_local_timestamp_us() - start_us);
}

static void fe_rec_max_u32(uint32_t *target, uint32_t value)
{
    if ((target != NULL) && (value > *target))
    {
        *target = value;
    }
}

static uint32_t fe_rec_align_page(uint32_t value)
{
    return (value + (ZY100_FE_REC_PAGE_BYTES - 1U)) &
           ~(uint32_t)(ZY100_FE_REC_PAGE_BYTES - 1U);
}

static uint32_t fe_rec_xor_bytes(uint32_t seed,
                                 const uint8_t *buf,
                                 uint32_t len)
{
    uint32_t idx;
    uint32_t word = 0U;

    if (buf == NULL)
    {
        return seed;
    }

    for (idx = 0U; idx < len; idx++)
    {
        word = (word << 8) ^ (uint32_t)buf[idx];
        if ((idx & 3U) == 3U)
        {
            seed ^= word;
            word = 0U;
        }
    }
    if ((len & 3U) != 0U)
    {
        seed ^= word;
    }
    return seed;
}

static const uint8_t *fe_rec_active_payload(void)
{
    if (s_fe_record_store.active_type == ZY100_FE_REC_TYPE_SUMMARY)
    {
        return (const uint8_t *)&s_fe_record_store.active_copy.summary.record;
    }
    if (s_fe_record_store.active_type == ZY100_FE_REC_TYPE_EVENT)
    {
        return (const uint8_t *)&s_fe_record_store.active_copy.event.record;
    }
    return NULL;
}

static uint32_t fe_rec_region_end(uint32_t type)
{
    if (type == ZY100_FE_REC_TYPE_SUMMARY)
    {
        return s_fe_record_store.summary_limit_addr;
    }
    return s_fe_record_store.event_limit_addr;
}

static uint32_t *fe_rec_next_addr_ptr(uint32_t type)
{
    if (type == ZY100_FE_REC_TYPE_SUMMARY)
    {
        return &s_fe_record_store.summary_next_addr;
    }
    return &s_fe_record_store.event_next_addr;
}

static uint32_t *fe_rec_erase_tail_ptr(uint32_t type)
{
    if (type == ZY100_FE_REC_TYPE_SUMMARY)
    {
        return &s_fe_record_store.summary_erase_tail_addr;
    }
    return &s_fe_record_store.event_erase_tail_addr;
}

static bool fe_rec_page_erased(const uint8_t page[ZY100_FE_REC_PAGE_BYTES])
{
    uint32_t idx;

    if (page == NULL)
    {
        return false;
    }
    for (idx = 0U; idx < ZY100_FE_REC_PAGE_BYTES; idx++)
    {
        if (page[idx] != 0xFFU)
        {
            return false;
        }
    }
    return true;
}

static bool fe_rec_sector_erased(uint32_t sector_addr)
{
    uint8_t page[ZY100_FE_REC_PAGE_BYTES];
    uint32_t offset;

    if ((sector_addr % ZY100_FE_REC_SECTOR_BYTES) != 0U)
    {
        return false;
    }
    for (offset = 0U; offset < ZY100_FE_REC_SECTOR_BYTES;
         offset += ZY100_FE_REC_PAGE_BYTES)
    {
        if (gd25q32e_read(sector_addr + offset,
                          page,
                          ZY100_FE_REC_PAGE_BYTES) != IMU_STATUS_OK)
        {
            return false;
        }
        if (!fe_rec_page_erased(page))
        {
            return false;
        }
    }
    return true;
}

static bool fe_rec_erase_append_until(uint32_t type, uint32_t end_addr)
{
    uint32_t *erase_tail = fe_rec_erase_tail_ptr(type);
#if ZY100_LOG_FE_DIAG_VERBOSE
    const char *region = (type == ZY100_FE_REC_TYPE_SUMMARY) ?
                         "summary" : "event";
#endif
    uint32_t region_end = fe_rec_region_end(type);

    if (!s_fe_record_store.append_erase_dynamic)
    {
        return true;
    }
    while (*erase_tail < end_addr)
    {
        uint32_t sector_addr =
            *erase_tail & ~(uint32_t)(ZY100_FE_REC_SECTOR_BYTES - 1U);

        if ((sector_addr < *erase_tail) ||
            ((sector_addr + ZY100_FE_REC_SECTOR_BYTES) > region_end))
        {
            s_fe_record_store.error = true;
            s_fe_record_store.stats.write_error++;
            return false;
        }
        if (!fe_rec_sector_erased(sector_addr))
        {
#if ZY100_LOG_FE_DIAG_VERBOSE
            DBG_DIRECT("[ERASE_AHEAD] region=%s addr=0x%08lX sectors=%u reason=append_tail",
                       region,
                       (unsigned long)sector_addr,
                       1U);
#endif
            if (gd25q32e_sector_erase_4k(sector_addr) != IMU_STATUS_OK)
            {
                s_fe_record_store.error = true;
                s_fe_record_store.stats.write_error++;
                return false;
            }
            if (gd25q32e_wait_while_busy(GD25Q32E_SECTOR_ERASE_TIMEOUT_MS) !=
                IMU_STATUS_OK)
            {
                s_fe_record_store.error = true;
                s_fe_record_store.stats.write_error++;
                return false;
            }
            if (!fe_rec_sector_erased(sector_addr))
            {
                s_fe_record_store.error = true;
                s_fe_record_store.stats.verify_error++;
                return false;
            }
            s_fe_record_store.stats.runtime_erase_count++;
        }
        *erase_tail = sector_addr + ZY100_FE_REC_SECTOR_BYTES;
    }
    return true;
}

static void fe_rec_refresh_used_bytes(void)
{
    s_fe_record_store.stats.summary_next_addr =
        s_fe_record_store.summary_next_addr;
    s_fe_record_store.stats.event_next_addr =
        s_fe_record_store.event_next_addr;
    s_fe_record_store.stats.summary_base_addr =
        s_fe_record_store.summary_base_addr;
    s_fe_record_store.stats.summary_limit_addr =
        s_fe_record_store.summary_limit_addr;
    s_fe_record_store.stats.event_base_addr =
        s_fe_record_store.event_base_addr;
    s_fe_record_store.stats.event_limit_addr =
        s_fe_record_store.event_limit_addr;
    s_fe_record_store.stats.summary_used_bytes =
        s_fe_record_store.summary_next_addr -
        s_fe_record_store.summary_base_addr;
    s_fe_record_store.stats.event_used_bytes =
        s_fe_record_store.event_next_addr -
        s_fe_record_store.event_base_addr;
}

static void fe_rec_log_prepare(uint32_t err)
{
#if ZY100_LOG_FE_DIAG_VERBOSE
    DBG_DIRECT("[FE_STORE_PREP] phase=%u erase=%u total=%u done=%u err=%u",
               (uint32_t)s_fe_record_store.prep_phase,
               s_fe_record_store.stats.prepare_erase_count,
               s_fe_record_store.prep_phase_total,
               s_fe_record_store.prep_phase_done,
               err);
#else
    (void)err;
#endif
}

static bool fe_rec_set_prepare_phase(zy100_fe_rec_prep_phase_t phase)
{
    s_fe_record_store.prep_phase = phase;
    s_fe_record_store.prep_phase_done = 0U;
    s_fe_record_store.prep_phase_last_log = 0U;

    if (phase == ZY100_FE_REC_PREP_PHASE_SUMMARY)
    {
        s_fe_record_store.prep_phase_base = s_fe_record_store.summary_base_addr;
        s_fe_record_store.prep_phase_total =
            (s_fe_record_store.summary_limit_addr -
             s_fe_record_store.summary_base_addr) /
             ZY100_FE_REC_SECTOR_BYTES;
        return ZY100_FINAL_EDGE_SUMMARY_FLASH_ENABLE != 0U;
    }
    if (phase == ZY100_FE_REC_PREP_PHASE_EVENT)
    {
        s_fe_record_store.prep_phase_base = s_fe_record_store.event_base_addr;
        s_fe_record_store.prep_phase_total =
            (s_fe_record_store.event_limit_addr -
             s_fe_record_store.event_base_addr) /
             ZY100_FE_REC_SECTOR_BYTES;
        return ZY100_FINAL_EDGE_EVENT_FLASH_ENABLE != 0U;
    }
    if (phase == ZY100_FE_REC_PREP_PHASE_META)
    {
        if (!s_fe_record_store.prep_erase_meta_scratch)
        {
            return false;
        }
        s_fe_record_store.prep_phase_base = ZY100_FINAL_EDGE_SESSION_META_ADDR;
        s_fe_record_store.prep_phase_total =
            ZY100_FE_REC_META_ERASE_BYTES / ZY100_FE_REC_SECTOR_BYTES;
        return ZY100_FINAL_EDGE_SESSION_META_ENABLE != 0U;
    }

    s_fe_record_store.prep_phase_base = 0U;
    s_fe_record_store.prep_phase_total = 0U;
    return false;
}

static bool fe_rec_advance_prepare_phase(void)
{
    if (s_fe_record_store.prep_phase == ZY100_FE_REC_PREP_PHASE_SUMMARY)
    {
        if (fe_rec_set_prepare_phase(ZY100_FE_REC_PREP_PHASE_EVENT))
        {
            fe_rec_log_prepare(0U);
            return true;
        }
    }
    if ((s_fe_record_store.prep_phase == ZY100_FE_REC_PREP_PHASE_EVENT) ||
        (s_fe_record_store.prep_phase == ZY100_FE_REC_PREP_PHASE_SUMMARY))
    {
        if (fe_rec_set_prepare_phase(ZY100_FE_REC_PREP_PHASE_META))
        {
            fe_rec_log_prepare(0U);
            return true;
        }
    }

    s_fe_record_store.prepared = true;
    s_fe_record_store.prep_state = ZY100_FE_REC_PREP_DONE;
    s_fe_record_store.stats.prepared = 1U;
    fe_rec_log_prepare(0U);
    return false;
}

static bool fe_rec_prepare_issue_next_sector(void)
{
    uint32_t addr = s_fe_record_store.prep_phase_base +
                    (s_fe_record_store.prep_phase_done *
                     ZY100_FE_REC_SECTOR_BYTES);
    imu_status_t status;

    if (s_fe_record_store.prep_phase == ZY100_FE_REC_PREP_PHASE_META)
    {
        if (!zy100_internal_meta_store_erase_sector(addr))
        {
            s_fe_record_store.prep_state = ZY100_FE_REC_PREP_ERROR;
            s_fe_record_store.error = true;
            s_fe_record_store.stats.write_error++;
            return false;
        }
        s_fe_record_store.prep_phase_done++;
        s_fe_record_store.stats.prepare_erase_count++;
        return true;
    }

    status = gd25q32e_sector_erase_4k(addr);
    if (status != IMU_STATUS_OK)
    {
        s_fe_record_store.prep_state = ZY100_FE_REC_PREP_ERROR;
        s_fe_record_store.error = true;
        s_fe_record_store.stats.write_error++;
        return false;
    }

    s_fe_record_store.prep_erase_in_flight = true;
    s_fe_record_store.stats.prepare_erase_count++;
    return true;
}

static void fe_rec_record_fail(void)
{
    if ((s_fe_record_store.target_provider != NULL) &&
        (s_fe_record_store.target_provider->abort != NULL) &&
        (s_fe_record_store.target_token != 0U))
    {
        s_fe_record_store.target_provider->abort(
            s_fe_record_store.target_provider->context,
            s_fe_record_store.target_token);
    }
    if (s_fe_record_store.active_type == ZY100_FE_REC_TYPE_SUMMARY)
    {
        s_fe_record_store.stats.summary_failed++;
    }
    else if (s_fe_record_store.active_type == ZY100_FE_REC_TYPE_EVENT)
    {
        s_fe_record_store.stats.event_failed++;
    }
    s_fe_record_store.pending = false;
    s_fe_record_store.active = false;
    s_fe_record_store.program_in_flight = false;
    s_fe_record_store.error = true;
    s_fe_record_store.active_type = 0U;
    s_fe_record_store.target_token = 0U;
}

static void fe_rec_record_finish_ok(void)
{
    if ((s_fe_record_store.target_provider != NULL) &&
        ((s_fe_record_store.target_provider->commit == NULL) ||
         !s_fe_record_store.target_provider->commit(
             s_fe_record_store.target_provider->context,
             s_fe_record_store.target_token)))
    {
        s_fe_record_store.error = true;
        if (s_fe_record_store.active_type == ZY100_FE_REC_TYPE_SUMMARY)
        {
            s_fe_record_store.stats.summary_failed++;
        }
        else
        {
            s_fe_record_store.stats.event_failed++;
        }
    }
    if (s_fe_record_store.active_type == ZY100_FE_REC_TYPE_SUMMARY)
    {
        s_fe_record_store.stats.summary_saved++;
    }
    else if (s_fe_record_store.active_type == ZY100_FE_REC_TYPE_EVENT)
    {
        s_fe_record_store.stats.event_saved++;
    }

    s_fe_record_store.stats.last_type = s_fe_record_store.active_type;
    s_fe_record_store.stats.last_seq = s_fe_record_store.active_seq;
    s_fe_record_store.stats.last_payload_bytes =
        s_fe_record_store.active_payload_bytes;
    s_fe_record_store.stats.last_page_count =
        s_fe_record_store.active_page_count;

    s_fe_record_store.pending = false;
    s_fe_record_store.active = false;
    s_fe_record_store.program_in_flight = false;
    s_fe_record_store.active_type = 0U;
    s_fe_record_store.target_token = 0U;
    fe_rec_refresh_used_bytes();
}

static void fe_rec_build_header(zy100_fe_record_flash_header_t *header)
{
    const uint8_t *payload = fe_rec_active_payload();

    memset(header, 0, sizeof(*header));
    header->magic = (s_fe_record_store.active_type ==
                     ZY100_FE_REC_TYPE_SUMMARY) ?
                    ZY100_FE_REC_MAGIC_SUMMARY : ZY100_FE_REC_MAGIC_EVENT;
    header->version = ZY100_FE_REC_VERSION;
    header->header_bytes =
        (uint16_t)sizeof(zy100_fe_record_flash_header_t);
    header->record_type = (uint16_t)s_fe_record_store.active_type;
    header->seq = s_fe_record_store.active_seq;
    header->edge_sample_index = s_fe_record_store.active_edge_sample_index;
    header->source_flags = s_fe_record_store.active_source_flags;
    header->payload_bytes = s_fe_record_store.active_payload_bytes;
    header->record_bytes = s_fe_record_store.active_record_bytes;
    header->page_count = s_fe_record_store.active_page_count;
    header->payload_xor =
        fe_rec_xor_bytes(0U, payload, s_fe_record_store.active_payload_bytes);
    header->header_xor = 0U;
    header->header_xor =
        fe_rec_xor_bytes(0U, (const uint8_t *)header, sizeof(*header));
}

static void fe_rec_build_page(uint32_t page_index)
{
    zy100_fe_record_flash_header_t header;
    const uint8_t *payload = fe_rec_active_payload();
    uint32_t payload_offset;
    uint32_t dst_offset;
    uint32_t copy_len;
    uint32_t page_record_offset;

    memset(s_fe_record_store.page, 0xFF, sizeof(s_fe_record_store.page));

    if (page_index == 0U)
    {
        fe_rec_build_header(&header);
        memcpy(s_fe_record_store.page, &header, sizeof(header));
        payload_offset = 0U;
        dst_offset = (uint32_t)sizeof(header);
    }
    else
    {
        page_record_offset = page_index * ZY100_FE_REC_PAGE_BYTES;
        payload_offset =
            page_record_offset -
            (uint32_t)sizeof(zy100_fe_record_flash_header_t);
        dst_offset = 0U;
    }

    if ((payload != NULL) &&
        (payload_offset < s_fe_record_store.active_payload_bytes) &&
        (dst_offset < ZY100_FE_REC_PAGE_BYTES))
    {
        copy_len = s_fe_record_store.active_payload_bytes - payload_offset;
        if (copy_len > (ZY100_FE_REC_PAGE_BYTES - dst_offset))
        {
            copy_len = ZY100_FE_REC_PAGE_BYTES - dst_offset;
        }
        memcpy(&s_fe_record_store.page[dst_offset],
               &payload[payload_offset],
               copy_len);
    }
}

static bool fe_rec_verify_current_page(void)
{
#if ZY100_FINAL_EDGE_EDGE_STORE_VERIFY_ENABLE
    uint8_t verify_buf[ZY100_FE_REC_VERIFY_CHUNK_BYTES];
    uint32_t verify_start_us = (uint32_t)imu_bsp_local_timestamp_us();
    uint32_t offset;

    for (offset = 0U; offset < ZY100_FE_REC_PAGE_BYTES;
         offset += ZY100_FE_REC_VERIFY_CHUNK_BYTES)
    {
        if (gd25q32e_read(s_fe_record_store.in_flight_addr + offset,
                          verify_buf,
                          ZY100_FE_REC_VERIFY_CHUNK_BYTES) != IMU_STATUS_OK)
        {
            s_fe_record_store.stats.verify_error++;
            return false;
        }
        if (memcmp(verify_buf,
                   &s_fe_record_store.page[offset],
                   ZY100_FE_REC_VERIFY_CHUNK_BYTES) != 0)
        {
            s_fe_record_store.stats.verify_error++;
            return false;
        }
    }
    fe_rec_max_u32(&s_fe_record_store.stats.verify_max_us,
                   fe_rec_elapsed_us(verify_start_us));
#endif
    return true;
}

static bool fe_rec_issue_page(uint32_t page_index)
{
    uint32_t issue_start_us = (uint32_t)imu_bsp_local_timestamp_us();
    uint32_t addr = s_fe_record_store.active_addr +
                    (page_index * ZY100_FE_REC_PAGE_BYTES);
    imu_status_t status;

    if ((s_fe_record_store.target_provider != NULL) &&
        ((addr < ZY100_ONLINE_SPOOL_REGION_BASE_ADDR) ||
         ((addr + ZY100_FE_REC_PAGE_BYTES) >
          (ZY100_ONLINE_SPOOL_REGION_BASE_ADDR +
           ZY100_ONLINE_SPOOL_REGION_BYTES)) ||
         (addr < s_fe_record_store.active_addr) ||
         ((addr + ZY100_FE_REC_PAGE_BYTES) >
          (s_fe_record_store.active_addr +
           s_fe_record_store.active_record_bytes))))
    {
        s_fe_record_store.stats.online_offline_region_write_violation++;
        s_fe_record_online_address_violations++;
        fe_rec_record_fail();
        return false;
    }
    fe_rec_build_page(page_index);
    status = gd25q32e_page_program(addr,
                                   s_fe_record_store.page,
                                   ZY100_FE_REC_PAGE_BYTES);
    fe_rec_max_u32(&s_fe_record_store.stats.issue_max_us,
                   fe_rec_elapsed_us(issue_start_us));
    if (status != IMU_STATUS_OK)
    {
        s_fe_record_store.stats.write_error++;
        fe_rec_record_fail();
        return false;
    }

    s_fe_record_store.program_in_flight = true;
    s_fe_record_store.in_flight_addr = addr;
    s_fe_record_store.in_flight_page = page_index;
    s_fe_record_store.stats.page_program++;
    if (s_fe_record_store.target_provider != NULL)
    {
        s_fe_record_store.stats.online_target_page_program++;
    }
    return true;
}

static bool fe_rec_finish_in_flight_page(void)
{
    if (!fe_rec_verify_current_page())
    {
        fe_rec_record_fail();
        return false;
    }
    if (s_fe_record_store.target_provider != NULL)
    {
        s_fe_record_online_verified_pages++;
    }

    s_fe_record_store.program_in_flight = false;
    if (s_fe_record_store.in_flight_page == 0U)
    {
        fe_rec_record_finish_ok();
        return true;
    }

    if ((s_fe_record_store.in_flight_page + 1U) <
        s_fe_record_store.active_page_count)
    {
        s_fe_record_store.active_next_page =
            s_fe_record_store.in_flight_page + 1U;
    }
    else
    {
        s_fe_record_store.active_next_page = 0U;
    }
    return true;
}

static bool fe_rec_begin_active(uint32_t type,
                                uint32_t seq,
                                uint32_t edge_sample_index,
                                uint32_t source_flags,
                                uint32_t payload_bytes)
{
    uint32_t *next_addr = fe_rec_next_addr_ptr(type);
    uint32_t record_bytes =
        fe_rec_align_page((uint32_t)sizeof(zy100_fe_record_flash_header_t) +
                          payload_bytes);
    uint32_t region_end = fe_rec_region_end(type);
    zy100_fe_store_target_t target;
    zy100_fe_store_reserve_result_t reserve_result;
    zy100_fe_store_target_kind_t target_kind;

    s_fe_record_store.active_type = type;
    s_fe_record_store.active_seq = seq;
    s_fe_record_store.active_edge_sample_index = edge_sample_index;
    s_fe_record_store.active_source_flags = source_flags;
    s_fe_record_store.active_payload_bytes = payload_bytes;

    if (type == ZY100_FE_REC_TYPE_SUMMARY)
    {
        s_fe_record_store.stats.summary_begin++;
    }
    else
    {
        s_fe_record_store.stats.event_begin++;
    }

    if ((s_fe_record_store.target_provider == NULL) &&
        ((*next_addr + record_bytes) > region_end))
    {
        if (type == ZY100_FE_REC_TYPE_SUMMARY)
        {
            s_fe_record_store.stats.summary_full++;
            s_fe_record_store.stats.summary_failed++;
        }
        else
        {
            s_fe_record_store.stats.event_full++;
            s_fe_record_store.stats.event_failed++;
        }
        s_fe_record_store.error = true;
        s_fe_record_store.active_type = 0U;
        return false;
    }

    if (s_fe_record_store.target_provider != NULL)
    {
        target_kind = (type == ZY100_FE_REC_TYPE_SUMMARY) ?
                      ZY100_FE_STORE_TARGET_SUMMARY :
                      ZY100_FE_STORE_TARGET_EVENT;
        memset(&target, 0, sizeof(target));
        reserve_result = s_fe_record_store.target_provider->reserve(
            s_fe_record_store.target_provider->context,
            target_kind,
            seq,
            record_bytes,
            payload_bytes,
            &target);
        if ((reserve_result != ZY100_FE_STORE_RESERVE_OK) ||
            (target.capacity_bytes < record_bytes))
        {
            if (type == ZY100_FE_REC_TYPE_SUMMARY)
            {
                s_fe_record_store.stats.summary_full++;
                s_fe_record_store.stats.summary_failed++;
            }
            else
            {
                s_fe_record_store.stats.event_full++;
                s_fe_record_store.stats.event_failed++;
            }
            s_fe_record_store.error = true;
            s_fe_record_store.active_type = 0U;
            return false;
        }
        s_fe_record_store.active_addr = target.data_addr;
        s_fe_record_store.target_token = target.token;
    }
    else
    {
        s_fe_record_store.active_addr = *next_addr;
    }
    s_fe_record_store.active_record_bytes = record_bytes;
    s_fe_record_store.active_page_count =
        record_bytes / ZY100_FE_REC_PAGE_BYTES;
    s_fe_record_store.active_next_page =
        (s_fe_record_store.active_page_count > 1U) ? 1U : 0U;
    if ((s_fe_record_store.target_provider == NULL) &&
        !fe_rec_erase_append_until(type,
                                   s_fe_record_store.active_addr +
                                   s_fe_record_store.active_record_bytes))
    {
        if (type == ZY100_FE_REC_TYPE_SUMMARY)
        {
            s_fe_record_store.stats.summary_failed++;
        }
        else
        {
            s_fe_record_store.stats.event_failed++;
        }
        s_fe_record_store.active_type = 0U;
        return false;
    }
    if (s_fe_record_store.target_provider == NULL)
    {
        *next_addr += record_bytes;
    }
    s_fe_record_store.active = true;
    s_fe_record_store.pending = true;
    fe_rec_refresh_used_bytes();
    return true;
}

static bool fe_rec_begin_next_record(void)
{
    uint32_t max_record_bytes = fe_rec_align_page(
        (uint32_t)sizeof(zy100_fe_record_flash_header_t) +
        ZY100_FINAL_EDGE_EVENT_RECORD_MAX_BYTES);

    if ((s_fe_record_store.target_provider != NULL) &&
        ((s_fe_record_store.target_provider->can_reserve == NULL) ||
         !s_fe_record_store.target_provider->can_reserve(
             s_fe_record_store.target_provider->context,
             max_record_bytes)))
    {
        return false;
    }
#if ZY100_FINAL_EDGE_EVENT_FLASH_ENABLE
    if (zy100_final_edge_queue_event_count() != 0U)
    {
        if (!zy100_final_edge_queue_pop_event_copy(
                &s_fe_record_store.active_copy.event))
        {
            return false;
        }
        return fe_rec_begin_active(
            ZY100_FE_REC_TYPE_EVENT,
            s_fe_record_store.active_copy.event.seq,
            s_fe_record_store.active_copy.event.edge_sample_index,
            s_fe_record_store.active_copy.event.source_flags,
            s_fe_record_store.active_copy.event.payload_bytes);
    }
#endif

#if ZY100_FINAL_EDGE_SUMMARY_FLASH_ENABLE
    if (zy100_final_edge_queue_summary_count() != 0U)
    {
        if (!zy100_final_edge_queue_pop_summary_copy(
                &s_fe_record_store.active_copy.summary))
        {
            return false;
        }
        return fe_rec_begin_active(
            ZY100_FE_REC_TYPE_SUMMARY,
            s_fe_record_store.active_copy.summary.seq,
            s_fe_record_store.active_copy.summary.edge_sample_index,
            s_fe_record_store.active_copy.summary.source_flags,
            s_fe_record_store.active_copy.summary.payload_bytes);
    }
#endif
    return false;
}

static bool fe_rec_meta_verify_page(void)
{
#if ZY100_FINAL_EDGE_EDGE_STORE_VERIFY_ENABLE
    uint8_t verify_buf[ZY100_FE_REC_VERIFY_CHUNK_BYTES];
    uint32_t offset;

    for (offset = 0U; offset < ZY100_FE_REC_PAGE_BYTES;
         offset += ZY100_FE_REC_VERIFY_CHUNK_BYTES)
    {
        if (!zy100_internal_meta_store_read(
                ZY100_FINAL_EDGE_SESSION_META_ADDR + offset,
                verify_buf,
                ZY100_FE_REC_VERIFY_CHUNK_BYTES))
        {
            s_fe_record_store.stats.meta_verify_error++;
            return false;
        }
        if (memcmp(verify_buf,
                   &s_fe_record_store.page[offset],
                   ZY100_FE_REC_VERIFY_CHUNK_BYTES) != 0)
        {
            s_fe_record_store.stats.meta_verify_error++;
            return false;
        }
    }
#endif
    return true;
}

bool zy100_final_edge_record_store_prepare_erase_range_begin(
    uint32_t round,
    uint32_t summary_base_addr,
    uint32_t summary_limit_addr,
    uint32_t event_base_addr,
    uint32_t event_limit_addr,
    bool erase_meta_scratch)
{
    if (!zy100_final_edge_record_store_is_idle())
    {
        return false;
    }
    if ((summary_base_addr >= summary_limit_addr) ||
        (event_base_addr >= event_limit_addr) ||
        ((summary_base_addr % ZY100_FE_REC_SECTOR_BYTES) != 0UL) ||
        ((summary_limit_addr % ZY100_FE_REC_SECTOR_BYTES) != 0UL) ||
        ((event_base_addr % ZY100_FE_REC_SECTOR_BYTES) != 0UL) ||
        ((event_limit_addr % ZY100_FE_REC_SECTOR_BYTES) != 0UL))
    {
        return false;
    }
    if ((summary_base_addr < ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR) ||
        (summary_limit_addr >
         (ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR +
          ZY100_FINAL_EDGE_SUMMARY_REGION_BYTES)) ||
        (event_base_addr < ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR) ||
        (event_limit_addr >
         (ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR +
          ZY100_FINAL_EDGE_EVENT_REGION_BYTES)))
    {
        return false;
    }

    memset(&s_fe_record_store, 0, sizeof(s_fe_record_store));
    s_fe_record_store.prep_round = round;
    s_fe_record_store.prep_erase_meta_scratch = erase_meta_scratch;
    s_fe_record_store.summary_base_addr = summary_base_addr;
    s_fe_record_store.summary_limit_addr = summary_limit_addr;
    s_fe_record_store.summary_next_addr = summary_base_addr;
    s_fe_record_store.summary_erase_tail_addr = summary_base_addr;
    s_fe_record_store.event_base_addr = event_base_addr;
    s_fe_record_store.event_limit_addr = event_limit_addr;
    s_fe_record_store.event_next_addr = event_base_addr;
    s_fe_record_store.event_erase_tail_addr = event_base_addr;
    fe_rec_refresh_used_bytes();

#if ZY100_FINAL_EDGE_EDGE_STORE_PREP_ERASE_ENABLE
    s_fe_record_store.prep_state = ZY100_FE_REC_PREP_ERASE;
    if (fe_rec_set_prepare_phase(ZY100_FE_REC_PREP_PHASE_SUMMARY))
    {
        fe_rec_log_prepare(0U);
        return true;
    }
    return fe_rec_advance_prepare_phase() ||
           (s_fe_record_store.prep_state == ZY100_FE_REC_PREP_DONE);
#else
    s_fe_record_store.prepared = true;
    s_fe_record_store.prep_state = ZY100_FE_REC_PREP_DONE;
    s_fe_record_store.stats.prepared = 1U;
    return true;
#endif
}

bool zy100_final_edge_record_store_prepare_append_begin(
    uint32_t round,
    uint32_t summary_data_begin_addr,
    uint32_t summary_region_end_addr,
    uint32_t event_data_begin_addr,
    uint32_t event_region_end_addr)
{
    if (!zy100_final_edge_record_store_is_idle())
    {
        return false;
    }
    if ((summary_data_begin_addr >= summary_region_end_addr) ||
        (event_data_begin_addr >= event_region_end_addr) ||
        ((summary_data_begin_addr % ZY100_FE_REC_SECTOR_BYTES) != 0UL) ||
        ((summary_region_end_addr % ZY100_FE_REC_SECTOR_BYTES) != 0UL) ||
        ((event_data_begin_addr % ZY100_FE_REC_SECTOR_BYTES) != 0UL) ||
        ((event_region_end_addr % ZY100_FE_REC_SECTOR_BYTES) != 0UL))
    {
        return false;
    }
    if ((summary_data_begin_addr < ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR) ||
        (summary_region_end_addr >
         (ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR +
          ZY100_FINAL_EDGE_SUMMARY_REGION_BYTES)) ||
        (event_data_begin_addr < ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR) ||
        (event_region_end_addr >
         (ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR +
          ZY100_FINAL_EDGE_EVENT_REGION_BYTES)))
    {
        return false;
    }

    memset(&s_fe_record_store, 0, sizeof(s_fe_record_store));
    s_fe_record_store.prep_round = round;
    s_fe_record_store.summary_base_addr = summary_data_begin_addr;
    s_fe_record_store.summary_limit_addr = summary_region_end_addr;
    s_fe_record_store.summary_next_addr = summary_data_begin_addr;
    s_fe_record_store.summary_erase_tail_addr = summary_data_begin_addr;
    s_fe_record_store.event_base_addr = event_data_begin_addr;
    s_fe_record_store.event_limit_addr = event_region_end_addr;
    s_fe_record_store.event_next_addr = event_data_begin_addr;
    s_fe_record_store.event_erase_tail_addr = event_data_begin_addr;
    s_fe_record_store.append_erase_dynamic = true;
    s_fe_record_store.prepared = true;
    s_fe_record_store.prep_state = ZY100_FE_REC_PREP_DONE;
    s_fe_record_store.stats.prepared = 1U;
    fe_rec_refresh_used_bytes();
#if ZY100_LOG_FE_DIAG_VERBOSE
    DBG_DIRECT("[FE_STORE_PREP] append_tail summary=0x%08lX..0x%08lX event=0x%08lX..0x%08lX erase=dynamic",
               (unsigned long)summary_data_begin_addr,
               (unsigned long)summary_region_end_addr,
               (unsigned long)event_data_begin_addr,
               (unsigned long)event_region_end_addr);
#endif
    return true;
}

bool zy100_final_edge_record_store_prepare_erase_begin(uint32_t round)
{
    return zy100_final_edge_record_store_prepare_erase_range_begin(
        round,
        ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR,
        ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR +
        ZY100_FINAL_EDGE_SUMMARY_REGION_BYTES,
        ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR,
        ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR +
        ZY100_FINAL_EDGE_EVENT_REGION_BYTES,
        true);
}

bool zy100_final_edge_record_store_prepare_target_begin(
    uint32_t round,
    const zy100_fe_store_target_provider_t *provider)
{
    if (!zy100_final_edge_record_store_is_idle() || (provider == NULL) ||
        (provider->can_reserve == NULL) || (provider->reserve == NULL) ||
        (provider->commit == NULL) || (provider->abort == NULL))
    {
        return false;
    }
    memset(&s_fe_record_store, 0, sizeof(s_fe_record_store));
    s_fe_record_store.prepared = true;
    s_fe_record_store.prep_state = ZY100_FE_REC_PREP_DONE;
    s_fe_record_store.prep_round = round;
    s_fe_record_store.target_provider = provider;
    s_fe_record_store.stats.prepared = 1U;
    return true;
}

zy100_flash_prepare_status_t zy100_final_edge_record_store_prepare_erase_poll(void)
{
    bool busy = false;

    if (s_fe_record_store.prep_state == ZY100_FE_REC_PREP_DONE)
    {
        return ZY100_FLASH_PREP_DONE;
    }
    if (s_fe_record_store.prep_state != ZY100_FE_REC_PREP_ERASE)
    {
        return ZY100_FLASH_PREP_ERROR;
    }

    if (s_fe_record_store.prep_phase != ZY100_FE_REC_PREP_PHASE_META)
    {
        if (gd25q32e_is_busy(&busy) != IMU_STATUS_OK)
        {
            s_fe_record_store.prep_state = ZY100_FE_REC_PREP_ERROR;
            s_fe_record_store.error = true;
            fe_rec_log_prepare(1U);
            return ZY100_FLASH_PREP_ERROR;
        }
        if (busy)
        {
            return ZY100_FLASH_PREP_BUSY;
        }

        if (s_fe_record_store.prep_erase_in_flight)
        {
            s_fe_record_store.prep_erase_in_flight = false;
            s_fe_record_store.prep_phase_done++;
            if (((s_fe_record_store.prep_phase_done %
                  ZY100_FE_REC_PREP_LOG_STEP) == 0U) &&
                (s_fe_record_store.prep_phase_done <
                 s_fe_record_store.prep_phase_total))
            {
                fe_rec_log_prepare(0U);
            }
        }
    }

    if (s_fe_record_store.prep_phase_done >=
        s_fe_record_store.prep_phase_total)
    {
        if (fe_rec_advance_prepare_phase())
        {
            return ZY100_FLASH_PREP_BUSY;
        }
        return ZY100_FLASH_PREP_DONE;
    }

    if (!fe_rec_prepare_issue_next_sector())
    {
        fe_rec_log_prepare(1U);
        return ZY100_FLASH_PREP_ERROR;
    }
    return ZY100_FLASH_PREP_BUSY;
}

bool zy100_final_edge_record_store_start(uint32_t round)
{
    if (!s_fe_record_store.prepared ||
        (s_fe_record_store.prep_state != ZY100_FE_REC_PREP_DONE) ||
        (s_fe_record_store.prep_round != round) ||
        !zy100_final_edge_record_store_is_idle())
    {
        return false;
    }

    s_fe_record_store.running = true;
    s_fe_record_store.run_round = round;
    s_fe_record_store.stats.started = 1U;
    s_fe_record_store.stats.prepared = 1U;
    return true;
}

void zy100_final_edge_record_store_reset_runtime(void)
{
    bool prepared = s_fe_record_store.prepared;
    zy100_fe_rec_prep_state_t prep_state = s_fe_record_store.prep_state;
    uint32_t prep_round = s_fe_record_store.prep_round;
    uint32_t summary_base_addr = s_fe_record_store.summary_base_addr;
    uint32_t summary_limit_addr = s_fe_record_store.summary_limit_addr;
    uint32_t summary_erase_tail_addr =
        s_fe_record_store.summary_erase_tail_addr;
    uint32_t event_base_addr = s_fe_record_store.event_base_addr;
    uint32_t event_limit_addr = s_fe_record_store.event_limit_addr;
    uint32_t event_erase_tail_addr =
        s_fe_record_store.event_erase_tail_addr;
    bool append_erase_dynamic = s_fe_record_store.append_erase_dynamic;
    uint32_t prepare_erase_count =
        s_fe_record_store.stats.prepare_erase_count;
    const zy100_fe_store_target_provider_t *target_provider =
        s_fe_record_store.target_provider;

    if (summary_base_addr == 0U)
    {
        summary_base_addr = ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR;
        summary_limit_addr = ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR +
                             ZY100_FINAL_EDGE_SUMMARY_REGION_BYTES;
    }
    if (event_base_addr == 0U)
    {
        event_base_addr = ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR;
        event_limit_addr = ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR +
                           ZY100_FINAL_EDGE_EVENT_REGION_BYTES;
    }
    memset(&s_fe_record_store, 0, sizeof(s_fe_record_store));
    s_fe_record_store.prepared = prepared;
    s_fe_record_store.prep_state = prep_state;
    s_fe_record_store.prep_round = prep_round;
    s_fe_record_store.summary_base_addr = summary_base_addr;
    s_fe_record_store.summary_limit_addr = summary_limit_addr;
    s_fe_record_store.summary_next_addr = summary_base_addr;
    s_fe_record_store.summary_erase_tail_addr =
        (summary_erase_tail_addr != 0U) ? summary_erase_tail_addr :
        summary_base_addr;
    s_fe_record_store.event_base_addr = event_base_addr;
    s_fe_record_store.event_limit_addr = event_limit_addr;
    s_fe_record_store.event_next_addr = event_base_addr;
    s_fe_record_store.event_erase_tail_addr =
        (event_erase_tail_addr != 0U) ? event_erase_tail_addr :
        event_base_addr;
    s_fe_record_store.append_erase_dynamic = append_erase_dynamic;
    s_fe_record_store.stats.prepared = prepared ? 1U : 0U;
    s_fe_record_store.stats.prepare_erase_count = prepare_erase_count;
    s_fe_record_store.target_provider = target_provider;
    fe_rec_refresh_used_bytes();
}

void zy100_final_edge_record_store_set_target_provider(
    const zy100_fe_store_target_provider_t *provider)
{
    if (zy100_final_edge_record_store_is_idle())
    {
        s_fe_record_store.target_provider = provider;
    }
}

void zy100_final_edge_record_store_reset_after_clear(uint32_t round)
{
    memset(&s_fe_record_store, 0, sizeof(s_fe_record_store));
    s_fe_record_store.prepared = true;
    s_fe_record_store.prep_state = ZY100_FE_REC_PREP_DONE;
    s_fe_record_store.prep_round = round;
    s_fe_record_store.summary_base_addr =
        ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR;
    s_fe_record_store.summary_limit_addr =
        ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR +
        ZY100_FINAL_EDGE_SUMMARY_REGION_BYTES;
    s_fe_record_store.summary_next_addr =
        s_fe_record_store.summary_base_addr;
    s_fe_record_store.summary_erase_tail_addr =
        s_fe_record_store.summary_limit_addr;
    s_fe_record_store.event_base_addr =
        ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR;
    s_fe_record_store.event_limit_addr =
        ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR +
        ZY100_FINAL_EDGE_EVENT_REGION_BYTES;
    s_fe_record_store.event_next_addr =
        s_fe_record_store.event_base_addr;
    s_fe_record_store.event_erase_tail_addr =
        s_fe_record_store.event_limit_addr;
    s_fe_record_store.stats.prepared = 1U;
    fe_rec_refresh_used_bytes();
}

static zy100_fe_record_pump_result_t fe_rec_pump_once_internal(
    bool allow_begin_record)
{
    uint32_t pump_start_us = (uint32_t)imu_bsp_local_timestamp_us();
    uint32_t issued = 0U;
    bool busy = false;

    if (s_fe_record_store.b_active)
    {
        s_fe_record_store.stats.b_active_pump_blocked++;
        s_fe_record_store.error = true;
        return ZY100_FE_REC_PUMP_ERROR;
    }
    if (!s_fe_record_store.running ||
        !zy100_final_edge_record_store_has_pending_work())
    {
        return ZY100_FE_REC_PUMP_NO_WORK;
    }

    do
    {
        if (gd25q32e_is_busy(&busy) != IMU_STATUS_OK)
        {
            s_fe_record_store.stats.write_error++;
            fe_rec_record_fail();
            fe_rec_max_u32(&s_fe_record_store.stats.pump_max_us,
                           fe_rec_elapsed_us(pump_start_us));
            return ZY100_FE_REC_PUMP_ERROR;
        }
        if (busy)
        {
            s_fe_record_store.stats.wip_busy++;
            if (s_fe_record_store.program_in_flight)
            {
                s_fe_record_store.stats.record_wip_busy++;
            }
            fe_rec_max_u32(&s_fe_record_store.stats.pump_max_us,
                           fe_rec_elapsed_us(pump_start_us));
            return ZY100_FE_REC_PUMP_WIP_BUSY;
        }

        if (s_fe_record_store.program_in_flight)
        {
            if (!fe_rec_finish_in_flight_page())
            {
                fe_rec_max_u32(&s_fe_record_store.stats.pump_max_us,
                               fe_rec_elapsed_us(pump_start_us));
                return ZY100_FE_REC_PUMP_ERROR;
            }
            if (!s_fe_record_store.pending)
            {
                fe_rec_max_u32(&s_fe_record_store.stats.pump_max_us,
                               fe_rec_elapsed_us(pump_start_us));
                return ZY100_FE_REC_PUMP_RECORD_DONE;
            }
        }

        if (!s_fe_record_store.pending && !allow_begin_record)
        {
            break;
        }
        if (!s_fe_record_store.pending &&
            !fe_rec_begin_next_record())
        {
            if (s_fe_record_store.error)
            {
                fe_rec_max_u32(&s_fe_record_store.stats.pump_max_us,
                               fe_rec_elapsed_us(pump_start_us));
                return ZY100_FE_REC_PUMP_ERROR;
            }
            break;
        }
        if (!s_fe_record_store.pending ||
            (issued >= ZY100_FINAL_EDGE_EDGE_STORE_PUMP_BUDGET))
        {
            break;
        }
        if (!fe_rec_issue_page(s_fe_record_store.active_next_page))
        {
            fe_rec_max_u32(&s_fe_record_store.stats.pump_max_us,
                           fe_rec_elapsed_us(pump_start_us));
            return ZY100_FE_REC_PUMP_ERROR;
        }
        issued++;
    } while (issued < ZY100_FINAL_EDGE_EDGE_STORE_PUMP_BUDGET);

    fe_rec_max_u32(&s_fe_record_store.stats.pump_max_us,
                   fe_rec_elapsed_us(pump_start_us));
    return (issued != 0U) ?
           ZY100_FE_REC_PUMP_PAGE_ISSUED : ZY100_FE_REC_PUMP_NO_WORK;
}

zy100_fe_record_pump_result_t zy100_final_edge_record_store_pump_once(void)
{
    return fe_rec_pump_once_internal(true);
}

zy100_fe_record_pump_result_t
zy100_final_edge_record_store_pump_active_once(void)
{
    return fe_rec_pump_once_internal(false);
}

bool zy100_final_edge_record_store_has_pending_work(void)
{
    return s_fe_record_store.pending ||
           s_fe_record_store.program_in_flight ||
#if ZY100_FINAL_EDGE_EVENT_FLASH_ENABLE
           (zy100_final_edge_queue_event_count() != 0U) ||
#endif
#if ZY100_FINAL_EDGE_SUMMARY_FLASH_ENABLE
           (zy100_final_edge_queue_summary_count() != 0U) ||
#endif
           false;
}

bool zy100_final_edge_record_store_has_active_record(void)
{
    return s_fe_record_store.active || s_fe_record_store.pending ||
           s_fe_record_store.program_in_flight;
}

bool zy100_final_edge_record_store_is_idle(void)
{
    return (!s_fe_record_store.pending) &&
           (!s_fe_record_store.program_in_flight) &&
           (!s_fe_record_store.active);
}

bool zy100_final_edge_record_store_has_error(void)
{
    return s_fe_record_store.error ||
           (s_fe_record_store.stats.write_error != 0U) ||
           (s_fe_record_store.stats.verify_error != 0U) ||
           (s_fe_record_store.stats.b_active_pump_blocked != 0U) ||
           (s_fe_record_store.stats.meta_write_error != 0U) ||
           (s_fe_record_store.stats.meta_verify_error != 0U);
}

bool zy100_final_edge_record_store_program_in_flight(void)
{
    return s_fe_record_store.program_in_flight;
}

void zy100_final_edge_record_store_abort_active(void)
{
    if (!s_fe_record_store.active && !s_fe_record_store.pending &&
        !s_fe_record_store.program_in_flight)
    {
        return;
    }
    if (s_fe_record_store.program_in_flight)
    {
        (void)gd25q32e_wait_while_busy(GD25Q32E_PAGE_PROGRAM_TIMEOUT_MS);
        s_fe_record_store.program_in_flight = false;
    }
    fe_rec_record_fail();
}

bool zy100_final_edge_record_store_build_session_meta(
    const zy100_fe_session_meta_input_t *input,
    zy100_fe_session_meta_t *out)
{
    if ((input == NULL) || (out == NULL))
    {
        return false;
    }

    fe_rec_refresh_used_bytes();
    memset(out, 0, sizeof(*out));
    out->magic = ZY100_FE_SESSION_MAGIC;
    out->version = ZY100_FE_SESSION_VERSION;
    out->header_bytes = (uint16_t)sizeof(*out);
    out->round = input->round;
    out->start_ms = input->start_ms;
    out->end_ms = input->end_ms;
    out->live_hz = ZY100_FINAL_EDGE_LIVE_UI_HZ;
    out->ois_hz = ZY100_FINAL_EDGE_OIS_HZ;
    out->official_hit_count = input->official_hit_count;
    out->nohit_count = input->nohit_count;
    out->raw_count = input->raw_count;
    out->summary_count = s_fe_record_store.stats.summary_saved;
    out->event_count = s_fe_record_store.stats.event_saved;
    out->raw_region_base = ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR;
    out->raw_region_bytes = ZY100_FINAL_EDGE_RAW_REGION_BYTES;
    out->summary_region_base = s_fe_record_store.summary_base_addr;
    out->summary_region_bytes =
        s_fe_record_store.summary_limit_addr -
        s_fe_record_store.summary_base_addr;
    out->event_region_base = s_fe_record_store.event_base_addr;
    out->event_region_bytes =
        s_fe_record_store.event_limit_addr -
        s_fe_record_store.event_base_addr;
    out->raw_used_bytes = input->raw_used_bytes;
    out->summary_used_bytes = s_fe_record_store.stats.summary_used_bytes;
    out->event_used_bytes = s_fe_record_store.stats.event_used_bytes;
    out->pass_flags = input->pass_flags;
    out->warn_flags = input->warn_flags;
    out->error_flags = input->error_flags;
    memcpy((uint8_t *)out->reserved,
           input->rtc_meta,
           ZY100_CAPTURE_TIME_META_BYTES);
    out->payload_xor = 0U;
    out->header_xor = 0U;
    out->header_xor =
        fe_rec_xor_bytes(0U, (const uint8_t *)out, sizeof(*out));
    return true;
}

bool zy100_final_edge_record_store_write_session_meta(
    const zy100_fe_session_meta_input_t *input)
{
    zy100_fe_session_meta_t meta;

    if ((input == NULL) ||
        !s_fe_record_store.running ||
        !zy100_final_edge_record_store_is_idle())
    {
        s_fe_record_store.stats.meta_write_error++;
        s_fe_record_store.error = true;
        return false;
    }

    if (!zy100_final_edge_record_store_build_session_meta(input, &meta))
    {
        s_fe_record_store.stats.meta_write_error++;
        s_fe_record_store.error = true;
        return false;
    }

    memset(s_fe_record_store.page, 0xFF, sizeof(s_fe_record_store.page));
    memcpy(s_fe_record_store.page, &meta, sizeof(meta));

    if (!zy100_internal_meta_store_program_page_overlay(
            ZY100_FINAL_EDGE_SESSION_META_ADDR,
            s_fe_record_store.page))
    {
        s_fe_record_store.stats.meta_write_error++;
        s_fe_record_store.error = true;
        return false;
    }
    s_fe_record_store.stats.meta_page_program++;
    s_fe_record_store.stats.meta_bytes = (uint32_t)sizeof(meta);
    s_fe_record_store.stats.meta_flags = meta.pass_flags;
    s_fe_record_store.stats.meta_write_ok = 1U;

    if (!fe_rec_meta_verify_page())
    {
        s_fe_record_store.error = true;
        return false;
    }
    s_fe_record_store.stats.meta_verify_ok = 1U;
    return true;
}

void zy100_final_edge_record_store_get_stats(
    zy100_fe_record_store_stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    s_fe_record_store.stats.prepared =
        s_fe_record_store.prepared ? 1U : 0U;
    s_fe_record_store.stats.started =
        s_fe_record_store.running ? 1U : 0U;
    s_fe_record_store.stats.active =
        s_fe_record_store.active ? 1U : 0U;
    s_fe_record_store.stats.pending =
        zy100_final_edge_record_store_has_pending_work() ? 1U : 0U;
    s_fe_record_store.stats.program_in_flight =
        s_fe_record_store.program_in_flight ? 1U : 0U;
    fe_rec_refresh_used_bytes();
    *out = s_fe_record_store.stats;
}

void zy100_final_edge_record_store_set_b_active(bool active)
{
    s_fe_record_store.b_active = active;
}

void zy100_final_edge_record_store_note_replay_blocked(void)
{
    s_fe_record_store.stats.replay_pump_blocked++;
}

void zy100_final_edge_record_store_note_raw_busy_blocked(void)
{
    s_fe_record_store.stats.raw_busy_blocked++;
}

void zy100_final_edge_record_store_note_raw_wait_record_wip(void)
{
    s_fe_record_store.stats.raw_wait_record_wip_count++;
}

void zy100_final_edge_record_store_note_final_pump(uint32_t elapsed_us,
                                                   bool timeout,
                                                   uint32_t pending_left)
{
    fe_rec_max_u32(&s_fe_record_store.stats.record_final_pump_max_us,
                   elapsed_us);
    if (timeout)
    {
        s_fe_record_store.stats.record_final_pump_timeout++;
    }
    s_fe_record_store.stats.record_final_pending_left = pending_left;
}

void zy100_final_edge_record_store_online_diag_reset(void)
{
    s_fe_record_online_verified_pages = 0U;
    s_fe_record_online_address_violations = 0U;
}

void zy100_final_edge_record_store_get_online_diag(uint32_t *verified_pages,
                                                    uint32_t *address_violations)
{
    if (verified_pages != NULL)
    {
        *verified_pages = s_fe_record_online_verified_pages;
    }
    if (address_violations != NULL)
    {
        *address_violations = s_fe_record_online_address_violations;
    }
}

#else

bool zy100_final_edge_record_store_prepare_erase_begin(uint32_t round)
{
    (void)round;
    return true;
}

bool zy100_final_edge_record_store_prepare_erase_range_begin(
    uint32_t round,
    uint32_t summary_base_addr,
    uint32_t summary_limit_addr,
    uint32_t event_base_addr,
    uint32_t event_limit_addr,
    bool erase_meta_scratch)
{
    (void)round;
    (void)summary_base_addr;
    (void)summary_limit_addr;
    (void)event_base_addr;
    (void)event_limit_addr;
    (void)erase_meta_scratch;
    return true;
}

bool zy100_final_edge_record_store_prepare_append_begin(
    uint32_t round,
    uint32_t summary_data_begin_addr,
    uint32_t summary_region_end_addr,
    uint32_t event_data_begin_addr,
    uint32_t event_region_end_addr)
{
    (void)round;
    (void)summary_data_begin_addr;
    (void)summary_region_end_addr;
    (void)event_data_begin_addr;
    (void)event_region_end_addr;
    return true;
}

void zy100_final_edge_record_store_online_diag_reset(void)
{
}

void zy100_final_edge_record_store_get_online_diag(uint32_t *verified_pages,
                                                    uint32_t *address_violations)
{
    if (verified_pages != NULL)
    {
        *verified_pages = 0U;
    }
    if (address_violations != NULL)
    {
        *address_violations = 0U;
    }
}

bool zy100_final_edge_record_store_prepare_target_begin(
    uint32_t round,
    const zy100_fe_store_target_provider_t *provider)
{
    (void)round;
    (void)provider;
    return true;
}

zy100_flash_prepare_status_t zy100_final_edge_record_store_prepare_erase_poll(void)
{
    return ZY100_FLASH_PREP_DONE;
}

bool zy100_final_edge_record_store_start(uint32_t round)
{
    (void)round;
    return true;
}

void zy100_final_edge_record_store_reset_runtime(void)
{
}

void zy100_final_edge_record_store_reset_after_clear(uint32_t round)
{
    (void)round;
}

zy100_fe_record_pump_result_t zy100_final_edge_record_store_pump_once(void)
{
    return ZY100_FE_REC_PUMP_NO_WORK;
}

zy100_fe_record_pump_result_t
zy100_final_edge_record_store_pump_active_once(void)
{
    return ZY100_FE_REC_PUMP_NO_WORK;
}

bool zy100_final_edge_record_store_has_pending_work(void)
{
    return false;
}

bool zy100_final_edge_record_store_has_active_record(void)
{
    return false;
}

bool zy100_final_edge_record_store_is_idle(void)
{
    return true;
}

bool zy100_final_edge_record_store_has_error(void)
{
    return false;
}

bool zy100_final_edge_record_store_program_in_flight(void)
{
    return false;
}

void zy100_final_edge_record_store_abort_active(void)
{
}

bool zy100_final_edge_record_store_write_session_meta(
    const zy100_fe_session_meta_input_t *input)
{
    (void)input;
    return true;
}

bool zy100_final_edge_record_store_build_session_meta(
    const zy100_fe_session_meta_input_t *input,
    zy100_fe_session_meta_t *out)
{
    (void)input;
    (void)out;
    return false;
}

void zy100_final_edge_record_store_get_stats(
    zy100_fe_record_store_stats_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

void zy100_final_edge_record_store_set_b_active(bool active)
{
    (void)active;
}

void zy100_final_edge_record_store_note_replay_blocked(void)
{
}

void zy100_final_edge_record_store_note_raw_busy_blocked(void)
{
}

void zy100_final_edge_record_store_note_raw_wait_record_wip(void)
{
}

void zy100_final_edge_record_store_note_final_pump(uint32_t elapsed_us,
                                                   bool timeout,
                                                   uint32_t pending_left)
{
    (void)elapsed_us;
    (void)timeout;
    (void)pending_left;
}

void zy100_final_edge_record_store_set_target_provider(
    const zy100_fe_store_target_provider_t *provider)
{
    (void)provider;
}

#endif
