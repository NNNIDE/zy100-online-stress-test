#include "zy100_training_session.h"

#include <os_sync.h>
#include <string.h>

#include "trace.h"

#include "../app_flags.h"
#include "../common/zy100_byteorder.h"
#include "zy100_crc32.h"
#include "zy100_final_edge_record_format.h"
#include "zy100_internal_meta_store.h"
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE && \
    ZY100_ONLINE_STREAM_ENABLE
#include "zy100_online_stream.h"
#endif
#include "zy100_rtc_clock.h"

#if ZY100_LEGACY_OFFLINE_ENABLE

#define ZY100_TRAINING_MAGIC_TRNB0 ((uint8_t)'T')
#define ZY100_TRAINING_MAGIC_TRNB1 ((uint8_t)'R')
#define ZY100_TRAINING_MAGIC_TRNB2 ((uint8_t)'N')
#define ZY100_TRAINING_MAGIC_TRNB3 ((uint8_t)'B')
#define ZY100_TRAINING_MAGIC_TRNE3 ((uint8_t)'E')

#define ZY100_TRAINING_RECORD_VERSION 1U
#define ZY100_TRAINING_RECORD_TYPE_BEGIN 1U
#define ZY100_TRAINING_RECORD_TYPE_END 2U

#define ZY100_TRAINING_FLAG_CAL_START (1U << 0)
#define ZY100_TRAINING_FLAG_CAL_END   (1U << 1)
#define ZY100_TRAINING_FLAG_COMPLETED (1U << 2)
#define ZY100_TRAINING_FLAG_ABORTED   (1U << 3)

#define ZY100_TRAINING_DATA_HAS_RAW     (1UL << 0)
#define ZY100_TRAINING_DATA_HAS_SUMMARY (1UL << 1)
#define ZY100_TRAINING_DATA_HAS_EVENT   (1UL << 2)

#define ZY100_TRAINING_META_SECTOR_ADDR  ZY100_FINAL_EDGE_SESSION_META_ADDR
#define ZY100_TRAINING_META_SECTOR_BYTES \
    ZY100_INTERNAL_META_STORE_SECTOR_BYTES
#define ZY100_TRAINING_META_BEGIN_OFFSET \
    ZY100_INTERNAL_META_STORE_PAGE_BYTES
#define ZY100_TRAINING_META_END_OFFSET \
    (ZY100_INTERNAL_META_STORE_PAGE_BYTES * 2U)
#define ZY100_TRAINING_META_RESERVED_END_OFFSET \
    (ZY100_INTERNAL_META_STORE_PAGE_BYTES * 3U)

#define ZY100_TRAINING_BEGIN_ADDR \
    (ZY100_TRAINING_META_SECTOR_ADDR + ZY100_TRAINING_META_BEGIN_OFFSET)
#define ZY100_TRAINING_END_ADDR \
    (ZY100_TRAINING_META_SECTOR_ADDR + ZY100_TRAINING_META_END_OFFSET)
#define ZY100_TRAINING_META_SECTOR_END \
    (ZY100_TRAINING_META_SECTOR_ADDR + ZY100_TRAINING_META_SECTOR_BYTES)
#define ZY100_TRAINING_META_RESERVED_END_ADDR \
    (ZY100_TRAINING_META_SECTOR_ADDR + ZY100_TRAINING_META_RESERVED_END_OFFSET)
typedef char zy100_training_record_size_check[
    (ZY100_TRAINING_RECORD_BYTES == 64U) ? 1 : -1];
typedef char zy100_training_begin_page_check[
    ((ZY100_TRAINING_BEGIN_ADDR %
      ZY100_INTERNAL_META_STORE_PAGE_BYTES) == 0U) ? 1 : -1];
typedef char zy100_training_end_page_check[
    ((ZY100_TRAINING_END_ADDR %
      ZY100_INTERNAL_META_STORE_PAGE_BYTES) == 0U) ? 1 : -1];
typedef char zy100_training_begin_sector_check[
    ((ZY100_TRAINING_BEGIN_ADDR >= ZY100_TRAINING_META_SECTOR_ADDR) &&
     ((ZY100_TRAINING_BEGIN_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
      ZY100_TRAINING_META_SECTOR_END)) ? 1 : -1];
typedef char zy100_training_end_sector_check[
    ((ZY100_TRAINING_END_ADDR >= ZY100_TRAINING_META_SECTOR_ADDR) &&
     ((ZY100_TRAINING_END_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
      ZY100_TRAINING_META_SECTOR_END)) ? 1 : -1];
typedef char zy100_training_begin_meta_overlap_check[
    ((ZY100_TRAINING_BEGIN_ADDR >=
      (ZY100_FINAL_EDGE_SESSION_META_ADDR + sizeof(zy100_fe_session_meta_t))) ||
     ((ZY100_TRAINING_BEGIN_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
      ZY100_FINAL_EDGE_SESSION_META_ADDR)) ? 1 : -1];
typedef char zy100_training_end_meta_overlap_check[
    ((ZY100_TRAINING_END_ADDR >=
      (ZY100_FINAL_EDGE_SESSION_META_ADDR + sizeof(zy100_fe_session_meta_t))) ||
     ((ZY100_TRAINING_END_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
      ZY100_FINAL_EDGE_SESSION_META_ADDR)) ? 1 : -1];
typedef char zy100_training_begin_end_overlap_check[
    (((ZY100_TRAINING_BEGIN_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
      ZY100_TRAINING_END_ADDR) ||
     ((ZY100_TRAINING_END_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
      ZY100_TRAINING_BEGIN_ADDR)) ? 1 : -1];
typedef char zy100_training_reserved_range_check[
    ((ZY100_TRAINING_END_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
     ZY100_TRAINING_META_RESERVED_END_ADDR) ? 1 : -1];

typedef struct
{
    zy100_training_session_ctx_t active;
    zy100_training_session_ctx_t completed;
    bool active_valid;
    bool completed_valid;
    bool begin_written;
    bool end_written;
    bool end_cached;
    bool layout_logged;
} zy100_training_session_state_t;

typedef struct
{
    const char *reason;
    bool erased;
    uint8_t flash_status;
    bool verify;
    uint32_t first_bad_addr;
    uint8_t first_bad;
} zy100_training_write_diag_t;

static zy100_training_session_state_t s_training_session;

static bool zy100_training_online_quiet_logs(void)
{
#if ZY100_FINAL_EDGE_MODE_ENABLE && ZY100_MULTI_SESSION_STORAGE_ENABLE && \
    ZY100_ONLINE_STREAM_ENABLE
#if ZY100_ONLINE_STREAM_VERBOSE_TRACE_ENABLE
    return false;
#else
    return zy100_online_stream_quiet_logs_active();
#endif
#else
    return false;
#endif
}

static const char *zy100_training_source_name(zy100_training_source_t source)
{
    if (source == ZY100_TRAINING_SRC_BUTTON)
    {
        return "button";
    }
    if (source == ZY100_TRAINING_SRC_BLE)
    {
        return "ble";
    }
    if (source == ZY100_TRAINING_SRC_AUTO)
    {
        return "auto_motion";
    }
    return "unknown";
}

static const char *zy100_training_stop_name(
    zy100_training_stop_reason_t reason)
{
    if (reason == ZY100_TRAINING_STOP_BUTTON)
    {
        return "button_pause";
    }
    if (reason == ZY100_TRAINING_STOP_BLE)
    {
        return "ble_pause";
    }
    if (reason == ZY100_TRAINING_STOP_FLASH_FULL)
    {
        return "flash_full";
    }
    if (reason == ZY100_TRAINING_STOP_ERROR)
    {
        return "error";
    }
    if (reason == ZY100_TRAINING_STOP_BATTERY_LOW)
    {
        return "battery_low";
    }
    return "unknown";
}

static uint32_t zy100_training_data_flags(
    const zy100_training_session_ctx_t *ctx)
{
    uint32_t flags = 0U;

    if (ctx == NULL)
    {
        return 0U;
    }
    if (ctx->raw_block_count != 0U)
    {
        flags |= ZY100_TRAINING_DATA_HAS_RAW;
    }
    if (ctx->summary_count != 0U)
    {
        flags |= ZY100_TRAINING_DATA_HAS_SUMMARY;
    }
    if (ctx->event_count != 0U)
    {
        flags |= ZY100_TRAINING_DATA_HAS_EVENT;
    }
    return flags;
}

static void zy100_training_build_record(
    const zy100_training_session_ctx_t *ctx,
    uint8_t record_type,
    bool aborted,
    uint8_t out[ZY100_TRAINING_RECORD_BYTES])
{
    uint8_t flags = 0U;
    uint32_t crc;

    memset(out, 0, ZY100_TRAINING_RECORD_BYTES);
    out[0] = ZY100_TRAINING_MAGIC_TRNB0;
    out[1] = ZY100_TRAINING_MAGIC_TRNB1;
    out[2] = ZY100_TRAINING_MAGIC_TRNB2;
    out[3] = (record_type == ZY100_TRAINING_RECORD_TYPE_END) ?
             ZY100_TRAINING_MAGIC_TRNE3 : ZY100_TRAINING_MAGIC_TRNB3;
    out[4] = ZY100_TRAINING_RECORD_VERSION;
    out[5] = record_type;
    out[6] = ZY100_TRAINING_RECORD_BYTES;
    if (ctx->calibrated_start != 0U)
    {
        flags |= ZY100_TRAINING_FLAG_CAL_START;
    }
    if ((record_type == ZY100_TRAINING_RECORD_TYPE_END) &&
        (ctx->calibrated_end != 0U))
    {
        flags |= ZY100_TRAINING_FLAG_CAL_END;
    }
    if (record_type == ZY100_TRAINING_RECORD_TYPE_END)
    {
        flags |= ZY100_TRAINING_FLAG_COMPLETED;
    }
    if (aborted || (ctx->stop_reason == ZY100_TRAINING_STOP_ERROR))
    {
        flags |= ZY100_TRAINING_FLAG_ABORTED;
    }
    out[7] = flags;
    zy100_put_u32_le(&out[8], ctx->user_id);
    zy100_put_u32_le(&out[12], ctx->training_id);
    zy100_put_u32_le(&out[16], ctx->session_seq);
    zy100_put_u32_le(&out[20], ctx->capture_round);
    zy100_put_u64_le(&out[24], ctx->start_time_ms);
    if (record_type == ZY100_TRAINING_RECORD_TYPE_END)
    {
        zy100_put_u64_le(&out[32], ctx->end_time_ms);
    }
    out[40] = ctx->time_source_start;
    if (record_type == ZY100_TRAINING_RECORD_TYPE_END)
    {
        out[41] = ctx->time_source_end;
        out[43] = (uint8_t)ctx->stop_reason;
        zy100_put_u32_le(&out[44], zy100_training_data_flags(ctx));
        zy100_put_u32_le(&out[48], ctx->raw_block_count);
        zy100_put_u32_le(&out[52], ctx->summary_count);
        zy100_put_u32_le(&out[56], ctx->event_count);
    }
    out[42] = (uint8_t)ctx->source;
    crc = zy100_crc32_ieee(out, 60U);
    zy100_put_u32_le(&out[60], crc);
}

static bool zy100_training_record_valid(const uint8_t record[ZY100_TRAINING_RECORD_BYTES],
                                        uint8_t record_type)
{
    uint32_t saved_crc;
    uint32_t actual_crc;
    uint8_t magic3 = (record_type == ZY100_TRAINING_RECORD_TYPE_END) ?
                     ZY100_TRAINING_MAGIC_TRNE3 : ZY100_TRAINING_MAGIC_TRNB3;

    if (record == NULL)
    {
        return false;
    }
    if ((record[0] != ZY100_TRAINING_MAGIC_TRNB0) ||
        (record[1] != ZY100_TRAINING_MAGIC_TRNB1) ||
        (record[2] != ZY100_TRAINING_MAGIC_TRNB2) ||
        (record[3] != magic3) ||
        (record[4] != ZY100_TRAINING_RECORD_VERSION) ||
        (record[5] != record_type) ||
        (record[6] != ZY100_TRAINING_RECORD_BYTES))
    {
        return false;
    }

    saved_crc = zy100_get_u32_le(&record[60]);
    actual_crc = zy100_crc32_ieee(record, 60U);
    return saved_crc == actual_crc;
}

static imu_status_t zy100_training_flash_ready_status(void)
{
    return zy100_internal_meta_store_ready() ?
           IMU_STATUS_OK : IMU_STATUS_NOT_READY;
}

static bool zy100_training_flash_ready(void)
{
    return zy100_training_flash_ready_status() == IMU_STATUS_OK;
}

#if !ZY100_MULTI_SESSION_STORAGE_ENABLE
static uint8_t zy100_training_flash_status_sr1(void)
{
    return 0xFFU;
}
#endif

static bool zy100_training_layout_valid(void)
{
    bool sector_aligned =
        ((ZY100_TRAINING_META_SECTOR_ADDR &
          (ZY100_TRAINING_META_SECTOR_BYTES - 1U)) == 0U);
    bool sector_in_meta =
        ((ZY100_TRAINING_META_SECTOR_ADDR + ZY100_TRAINING_META_SECTOR_BYTES) <=
         ZY100_INTERNAL_META_STORE_REGION_BYTES);
    bool slots_in_range =
        (ZY100_TRAINING_BEGIN_ADDR >= ZY100_TRAINING_META_SECTOR_ADDR) &&
        ((ZY100_TRAINING_BEGIN_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
         ZY100_TRAINING_META_SECTOR_END) &&
        (ZY100_TRAINING_END_ADDR >= ZY100_TRAINING_META_SECTOR_ADDR) &&
        ((ZY100_TRAINING_END_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
         ZY100_TRAINING_META_SECTOR_END);
    bool slots_overlap =
        !(((ZY100_TRAINING_BEGIN_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
           ZY100_TRAINING_END_ADDR) ||
          ((ZY100_TRAINING_END_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
           ZY100_TRAINING_BEGIN_ADDR));
    bool meta_overlap =
        !((ZY100_TRAINING_BEGIN_ADDR >=
           (ZY100_FINAL_EDGE_SESSION_META_ADDR + sizeof(zy100_fe_session_meta_t))) ||
          ((ZY100_TRAINING_BEGIN_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
           ZY100_FINAL_EDGE_SESSION_META_ADDR)) ||
        !((ZY100_TRAINING_END_ADDR >=
           (ZY100_FINAL_EDGE_SESSION_META_ADDR + sizeof(zy100_fe_session_meta_t))) ||
          ((ZY100_TRAINING_END_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
           ZY100_FINAL_EDGE_SESSION_META_ADDR));

    return sector_aligned && sector_in_meta && slots_in_range &&
           !slots_overlap && !meta_overlap;
}

static void zy100_training_meta_log_layout(void)
{
    uint32_t overlap = 0U;
    uint32_t sector_aligned =
        ((ZY100_TRAINING_META_SECTOR_ADDR &
          (ZY100_TRAINING_META_SECTOR_BYTES - 1U)) == 0U) ? 1U : 0U;
    uint32_t in_range =
        ((ZY100_TRAINING_BEGIN_ADDR >= ZY100_TRAINING_META_SECTOR_ADDR) &&
         ((ZY100_TRAINING_BEGIN_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
          ZY100_TRAINING_META_SECTOR_END) &&
         (ZY100_TRAINING_END_ADDR >= ZY100_TRAINING_META_SECTOR_ADDR) &&
         ((ZY100_TRAINING_END_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
          ZY100_TRAINING_META_SECTOR_END) &&
         ((ZY100_TRAINING_META_SECTOR_ADDR + ZY100_TRAINING_META_SECTOR_BYTES) <=
          ZY100_INTERNAL_META_STORE_REGION_BYTES)) ? 1U : 0U;

    if (!(((ZY100_TRAINING_BEGIN_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
           ZY100_TRAINING_END_ADDR) ||
          ((ZY100_TRAINING_END_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
           ZY100_TRAINING_BEGIN_ADDR)))
    {
        overlap = 1U;
    }
    if (!((ZY100_TRAINING_BEGIN_ADDR >=
           (ZY100_FINAL_EDGE_SESSION_META_ADDR + sizeof(zy100_fe_session_meta_t))) ||
          ((ZY100_TRAINING_BEGIN_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
           ZY100_FINAL_EDGE_SESSION_META_ADDR)) ||
        !((ZY100_TRAINING_END_ADDR >=
           (ZY100_FINAL_EDGE_SESSION_META_ADDR + sizeof(zy100_fe_session_meta_t))) ||
          ((ZY100_TRAINING_END_ADDR + ZY100_TRAINING_RECORD_BYTES) <=
           ZY100_FINAL_EDGE_SESSION_META_ADDR)))
    {
        overlap = 1U;
    }

    ZY100_LOG_VERBOSE("[TRAIN_META_LAYOUT] sector=0x%08lX trnb=0x%08lX trne=0x%08lX sector_aligned=%u in_range=%u overlap=%u",
                      (unsigned long)ZY100_TRAINING_META_SECTOR_ADDR,
                      (unsigned long)ZY100_TRAINING_BEGIN_ADDR,
                      (unsigned long)ZY100_TRAINING_END_ADDR,
                      sector_aligned,
                      in_range,
                      overlap);
}

static imu_status_t zy100_training_slot_erased_detail(uint32_t addr,
                                                      bool *erased,
                                                      uint32_t *first_bad_addr,
                                                      uint8_t *first_bad)
{
    uint8_t buf[ZY100_TRAINING_RECORD_BYTES];
    uint32_t idx;

    if (erased != NULL)
    {
        *erased = false;
    }
    if (!zy100_internal_meta_store_read(addr, buf, (uint32_t)sizeof(buf)))
    {
        return zy100_internal_meta_store_ready() ?
               IMU_STATUS_BUS_ERROR : IMU_STATUS_NOT_READY;
    }

    for (idx = 0U; idx < sizeof(buf); idx++)
    {
        if (buf[idx] != 0xFFU)
        {
            if (first_bad_addr != NULL)
            {
                *first_bad_addr = addr + idx;
            }
            if (first_bad != NULL)
            {
                *first_bad = buf[idx];
            }
            return IMU_STATUS_OK;
        }
    }
    if (erased != NULL)
    {
        *erased = true;
    }
    return IMU_STATUS_OK;
}

static bool zy100_training_meta_verify(zy100_training_meta_verify_result_t *out)
{
    zy100_training_meta_verify_result_t local;
    uint32_t first_bad_addr = 0xFFFFFFFFUL;
    uint8_t first_bad = 0xFFU;
    imu_status_t status;
    bool read_fail = false;

    memset(&local, 0, sizeof(local));
    local.read_ok = true;
    local.status = IMU_STATUS_OK;
    local.first_bad_addr = 0xFFFFFFFFUL;
    local.first_bad = 0xFFU;

    if (!zy100_training_layout_valid())
    {
        local.read_ok = false;
        local.status = IMU_STATUS_INVALID_PARAM;
        zy100_training_meta_log_layout();
        if (out != NULL)
        {
            *out = local;
        }
        DBG_DIRECT("[TRAIN_META_VERIFY] trnb=0x%08lX trne=0x%08lX trnb_erased=%u trne_erased=%u first_bad_addr=0x%08lX first_bad=0x%02X reason=invalid_layout",
                   (unsigned long)ZY100_TRAINING_BEGIN_ADDR,
                   (unsigned long)ZY100_TRAINING_END_ADDR,
                   0U,
                   0U,
                   (unsigned long)local.first_bad_addr,
                   local.first_bad);
        return false;
    }

    status = zy100_training_flash_ready_status();
    if (status != IMU_STATUS_OK)
    {
        local.read_ok = false;
        local.status = status;
        if (out != NULL)
        {
            *out = local;
        }
        DBG_DIRECT("[TRAIN_META_VERIFY] read_fail addr=0x%08lX ret=%u",
                   (unsigned long)ZY100_TRAINING_BEGIN_ADDR,
                   (uint32_t)local.status);
        DBG_DIRECT("[TRAIN_META_VERIFY] trnb=0x%08lX trne=0x%08lX trnb_erased=%u trne_erased=%u first_bad_addr=0x%08lX first_bad=0x%02X reason=read_fail",
                   (unsigned long)ZY100_TRAINING_BEGIN_ADDR,
                   (unsigned long)ZY100_TRAINING_END_ADDR,
                   0U,
                   0U,
                   (unsigned long)local.first_bad_addr,
                   local.first_bad);
        return false;
    }

    status = zy100_training_slot_erased_detail(ZY100_TRAINING_BEGIN_ADDR,
                                               &local.trnb_erased,
                                               &first_bad_addr,
                                               &first_bad);
    if (status != IMU_STATUS_OK)
    {
        read_fail = true;
        local.read_ok = false;
        local.status = status;
        DBG_DIRECT("[TRAIN_META_VERIFY] read_fail addr=0x%08lX ret=%u",
                   (unsigned long)ZY100_TRAINING_BEGIN_ADDR,
                   (uint32_t)status);
    }
    if (!local.trnb_erased)
    {
        local.first_bad_addr = first_bad_addr;
        local.first_bad = first_bad;
    }

    first_bad_addr = 0xFFFFFFFFUL;
    first_bad = 0xFFU;
    status = zy100_training_slot_erased_detail(ZY100_TRAINING_END_ADDR,
                                               &local.trne_erased,
                                               &first_bad_addr,
                                               &first_bad);
    if (status != IMU_STATUS_OK)
    {
        read_fail = true;
        local.read_ok = false;
        local.status = status;
        DBG_DIRECT("[TRAIN_META_VERIFY] read_fail addr=0x%08lX ret=%u",
                   (unsigned long)ZY100_TRAINING_END_ADDR,
                   (uint32_t)status);
    }
    if (local.trnb_erased && !local.trne_erased)
    {
        local.first_bad_addr = first_bad_addr;
        local.first_bad = first_bad;
    }

    if (read_fail)
    {
        local.first_bad_addr = 0xFFFFFFFFUL;
        local.first_bad = 0xFFU;
        DBG_DIRECT("[TRAIN_META_VERIFY] trnb=0x%08lX trne=0x%08lX trnb_erased=%u trne_erased=%u first_bad_addr=0x%08lX first_bad=0x%02X reason=read_fail",
                   (unsigned long)ZY100_TRAINING_BEGIN_ADDR,
                   (unsigned long)ZY100_TRAINING_END_ADDR,
                   local.trnb_erased ? 1U : 0U,
                   local.trne_erased ? 1U : 0U,
                   (unsigned long)local.first_bad_addr,
                   local.first_bad);
    }
    else if (ZY100_LOG_META_VERBOSE ||
             !local.trnb_erased ||
             !local.trne_erased)
    {
        DBG_DIRECT("[TRAIN_META_VERIFY] trnb=0x%08lX trne=0x%08lX trnb_erased=%u trne_erased=%u first_bad_addr=0x%08lX first_bad=0x%02X",
                   (unsigned long)ZY100_TRAINING_BEGIN_ADDR,
                   (unsigned long)ZY100_TRAINING_END_ADDR,
                   local.trnb_erased ? 1U : 0U,
                   local.trne_erased ? 1U : 0U,
                   (unsigned long)local.first_bad_addr,
                   local.first_bad);
    }

    if (out != NULL)
    {
        *out = local;
    }
    return local.trnb_erased && local.trne_erased;
}

#if !ZY100_MULTI_SESSION_STORAGE_ENABLE
static bool zy100_training_flash_write_record(uint32_t addr,
                                              const uint8_t record[ZY100_TRAINING_RECORD_BYTES],
                                              zy100_training_write_diag_t *diag)
{
    uint8_t page[ZY100_INTERNAL_META_STORE_PAGE_BYTES];

    if (diag != NULL)
    {
        memset(diag, 0, sizeof(*diag));
        diag->reason = "unknown";
        diag->flash_status = zy100_training_flash_status_sr1();
        diag->first_bad_addr = 0xFFFFFFFFUL;
        diag->first_bad = 0xFFU;
    }

    if ((record == NULL) || !zy100_training_layout_valid())
    {
        if (diag != NULL)
        {
            diag->reason = "invalid_layout";
        }
        return false;
    }
    if (!zy100_training_flash_ready())
    {
        if (diag != NULL)
        {
            diag->reason = "storage_not_ready";
        }
        return false;
    }
    {
        bool erased = false;
        imu_status_t erased_status =
            zy100_training_slot_erased_detail(
                addr,
                &erased,
                (diag != NULL) ? &diag->first_bad_addr : NULL,
                (diag != NULL) ? &diag->first_bad : NULL);
        if ((erased_status != IMU_STATUS_OK) || !erased)
        {
            if (diag != NULL)
            {
                diag->reason = (erased_status == IMU_STATUS_OK) ?
                               "not_erased" : "storage_not_ready";
                diag->erased = false;
                diag->flash_status = zy100_training_flash_status_sr1();
            }
            return false;
        }
    }

    if (diag != NULL)
    {
        diag->erased = true;
    }

    if (!zy100_internal_meta_store_read_page(addr, page))
    {
        if (diag != NULL)
        {
            diag->reason = "storage_not_ready";
        }
        return false;
    }
    memcpy(page, record, ZY100_TRAINING_RECORD_BYTES);
    if (!zy100_internal_meta_store_program_page_overlay(addr, page))
    {
        if (diag != NULL)
        {
            diag->reason = "flash_program_fail";
            diag->flash_status = zy100_training_flash_status_sr1();
        }
        return false;
    }
    memset(page, 0, sizeof(page));
    if (!zy100_internal_meta_store_read(addr,
                                        page,
                                        ZY100_TRAINING_RECORD_BYTES))
    {
        if (diag != NULL)
        {
            diag->reason = "flash_verify_fail";
        }
        return false;
    }
    if (memcmp(page, record, ZY100_TRAINING_RECORD_BYTES) != 0)
    {
        uint32_t idx;

        if (diag != NULL)
        {
            diag->reason = "flash_verify_fail";
            for (idx = 0U; idx < ZY100_TRAINING_RECORD_BYTES; idx++)
            {
                if (page[idx] != record[idx])
                {
                    diag->first_bad_addr = addr + idx;
                    diag->first_bad = page[idx];
                    break;
                }
            }
        }
        return false;
    }
    if (diag != NULL)
    {
        diag->verify = true;
        diag->reason = "ok";
        diag->flash_status = zy100_training_flash_status_sr1();
    }
    return true;
}
#endif

static bool zy100_training_flash_read_record(uint32_t addr,
                                             uint8_t record_type,
                                             uint8_t out[ZY100_TRAINING_RECORD_BYTES])
{
    uint8_t local[ZY100_TRAINING_RECORD_BYTES];

    if ((out == NULL) || !zy100_training_flash_ready())
    {
        return false;
    }
    if (!zy100_internal_meta_store_read(addr,
                                        local,
                                        (uint32_t)sizeof(local)))
    {
        return false;
    }
    if (!zy100_training_record_valid(local, record_type))
    {
        return false;
    }
    memcpy(out, local, sizeof(local));
    return true;
}

static void zy100_training_fill_end_time_if_needed(
    zy100_training_session_ctx_t *ctx,
    zy100_training_stop_reason_t stop_reason)
{
    if ((ctx == NULL) || (ctx->ended != 0U))
    {
        return;
    }
    ctx->ended = 1U;
    ctx->end_time_ms = zy100_rtc_clock_now_ms();
    ctx->calibrated_end = zy100_rtc_clock_is_calibrated() ? 1U : 0U;
    ctx->time_source_end = zy100_rtc_clock_time_source();
    ctx->stop_reason = stop_reason;
}

void zy100_training_session_log_layout_once(void)
{
    uint32_t lock_state;
    bool should_log = false;

    lock_state = os_lock();
    if (!s_training_session.layout_logged)
    {
        s_training_session.layout_logged = true;
        should_log = true;
    }
    os_unlock(lock_state);

    if (!should_log)
    {
        return;
    }

    zy100_training_meta_log_layout();
}

bool zy100_training_session_begin(zy100_training_source_t source,
                                  uint32_t user_id,
                                  uint32_t training_id,
                                  uint32_t session_seq,
                                  uint32_t capture_round)
{
    zy100_training_session_ctx_t ctx;
    uint8_t record[ZY100_TRAINING_RECORD_BYTES];
    uint32_t lock_state;
    zy100_training_write_diag_t diag;

    zy100_training_session_log_layout_once();
    memset(&ctx, 0, sizeof(ctx));
    ctx.active = 1U;
    ctx.started = 1U;
    ctx.user_id = user_id;
    ctx.training_id = training_id;
    ctx.session_seq = session_seq;
    ctx.capture_round = capture_round;
    ctx.time_source_start = zy100_rtc_clock_time_source();
    ctx.time_source_end = ctx.time_source_start;
    ctx.calibrated_start = zy100_rtc_clock_is_calibrated() ? 1U : 0U;
    ctx.calibrated_end = ctx.calibrated_start;
    ctx.start_time_ms = zy100_rtc_clock_now_ms();
    ctx.end_time_ms = ctx.start_time_ms;
    ctx.source = source;
    ctx.stop_reason = ZY100_TRAINING_STOP_UNKNOWN;

    zy100_training_build_record(&ctx,
                                ZY100_TRAINING_RECORD_TYPE_BEGIN,
                                false,
                                record);
#if ZY100_MULTI_SESSION_STORAGE_ENABLE
    memset(&diag, 0, sizeof(diag));
#else
    if (!zy100_training_flash_write_record(ZY100_TRAINING_BEGIN_ADDR,
                                           record,
                                           &diag))
    {
        DBG_DIRECT("[TRAIN] begin_write failed user=%lu train=%lu addr=0x%08lX reason=%s erased=%u flash_status=0x%02X verify=%u first_bad_addr=0x%08lX first_bad=0x%02X",
                   (unsigned long)user_id,
                   (unsigned long)training_id,
                   (unsigned long)ZY100_TRAINING_BEGIN_ADDR,
                   diag.reason,
                   diag.erased ? 1U : 0U,
                   diag.flash_status,
                   diag.verify ? 1U : 0U,
                   (unsigned long)diag.first_bad_addr,
                   diag.first_bad);
        lock_state = os_lock();
        memset(&s_training_session.active, 0, sizeof(s_training_session.active));
        s_training_session.active_valid = false;
        s_training_session.begin_written = false;
        os_unlock(lock_state);
        return false;
    }
#endif

    lock_state = os_lock();
    s_training_session.active = ctx;
    s_training_session.active_valid = true;
    s_training_session.begin_written = true;
    s_training_session.end_written = false;
    s_training_session.end_cached = false;
    memset(&s_training_session.completed, 0,
           sizeof(s_training_session.completed));
    s_training_session.completed_valid = false;
    os_unlock(lock_state);

#if ZY100_MULTI_SESSION_STORAGE_ENABLE
    ZY100_LOG_VERBOSE("[TRAIN] begin_cache ok user=%lu train=%lu multi_session=1",
                      (unsigned long)ctx.user_id,
                      (unsigned long)ctx.training_id);
#else
    ZY100_LOG_VERBOSE("[TRAIN] begin_write ok user=%lu train=%lu addr=0x%08lX erased=%u verify=%u",
                      (unsigned long)ctx.user_id,
                      (unsigned long)ctx.training_id,
                      (unsigned long)ZY100_TRAINING_BEGIN_ADDR,
                      diag.erased ? 1U : 0U,
                      diag.verify ? 1U : 0U);
#endif
    if (!zy100_training_online_quiet_logs())
    {
        DBG_DIRECT("[TRAIN] begin source=%s user=%lu train=%lu session=%lu round=%lu start_ms=%llu calibrated=%u source_id=%u",
                   zy100_training_source_name(source),
                   (unsigned long)ctx.user_id,
                   (unsigned long)ctx.training_id,
                   (unsigned long)ctx.session_seq,
                   (unsigned long)ctx.capture_round,
                   (unsigned long long)ctx.start_time_ms,
                   (uint32_t)ctx.calibrated_start,
                   (uint32_t)ctx.time_source_start);
    }
    return true;
}

void zy100_training_session_update_round(uint32_t session_seq,
                                         uint32_t capture_round)
{
    uint32_t lock_state;

    lock_state = os_lock();
    if (s_training_session.active_valid)
    {
        s_training_session.active.session_seq = session_seq;
        s_training_session.active.capture_round = capture_round;
    }
    os_unlock(lock_state);
}

void zy100_training_session_note_end_request(
    zy100_training_source_t request_source,
    zy100_training_stop_reason_t stop_reason)
{
    zy100_training_session_ctx_t local;
    uint32_t lock_state;
    bool logged = false;

    lock_state = os_lock();
    if (s_training_session.active_valid)
    {
        zy100_training_fill_end_time_if_needed(&s_training_session.active,
                                               stop_reason);
        local = s_training_session.active;
        logged = true;
    }
    os_unlock(lock_state);

    if (logged)
    {
        DBG_DIRECT("[TRAIN] end_request source=%s user=%lu train=%lu end_ms=%llu reason=%s",
                   zy100_training_source_name(request_source),
                   (unsigned long)local.user_id,
                   (unsigned long)local.training_id,
                   (unsigned long long)local.end_time_ms,
                   zy100_training_stop_name(stop_reason));
    }
}

void zy100_training_session_note_end_if_unset(
    zy100_training_stop_reason_t stop_reason)
{
    zy100_training_session_ctx_t local;
    uint32_t lock_state;
    bool logged = false;

    lock_state = os_lock();
    if (s_training_session.active_valid &&
        (s_training_session.active.ended == 0U))
    {
        zy100_training_fill_end_time_if_needed(&s_training_session.active,
                                               stop_reason);
        local = s_training_session.active;
        logged = true;
    }
    os_unlock(lock_state);

    if (logged)
    {
        DBG_DIRECT("[TRAIN] end_request source=auto user=%lu train=%lu end_ms=%llu reason=%s",
                   (unsigned long)local.user_id,
                   (unsigned long)local.training_id,
                   (unsigned long long)local.end_time_ms,
                   zy100_training_stop_name(stop_reason));
    }
}

void zy100_training_session_set_counts(uint32_t raw_block_count,
                                       uint32_t summary_count,
                                       uint32_t event_count)
{
    uint32_t lock_state;

    lock_state = os_lock();
    if (s_training_session.active_valid)
    {
        s_training_session.active.raw_block_count = raw_block_count;
        s_training_session.active.summary_count = summary_count;
        s_training_session.active.event_count = event_count;
    }
    os_unlock(lock_state);
}

bool zy100_training_session_write_end_record(void)
{
    zy100_training_session_ctx_t ctx;
    uint8_t record[ZY100_TRAINING_RECORD_BYTES];
    uint64_t duration_ms;
    uint32_t lock_state;
    bool ok;
    zy100_training_write_diag_t diag;

    lock_state = os_lock();
    if (!s_training_session.active_valid)
    {
        os_unlock(lock_state);
        return false;
    }
    zy100_training_fill_end_time_if_needed(&s_training_session.active,
                                           ZY100_TRAINING_STOP_UNKNOWN);
    ctx = s_training_session.active;
    os_unlock(lock_state);

    zy100_training_build_record(&ctx,
                                ZY100_TRAINING_RECORD_TYPE_END,
                                false,
                                record);
#if ZY100_MULTI_SESSION_STORAGE_ENABLE
    memset(&diag, 0, sizeof(diag));
    ok = true;
#else
    ok = zy100_training_flash_write_record(ZY100_TRAINING_END_ADDR,
                                           record,
                                           &diag);
#endif

    lock_state = os_lock();
    s_training_session.completed = ctx;
    s_training_session.completed.active = 0U;
    s_training_session.completed_valid = true;
    s_training_session.active_valid = false;
    s_training_session.active.active = 0U;
    s_training_session.end_written = ok;
#if ZY100_MULTI_SESSION_STORAGE_ENABLE
    s_training_session.end_cached = true;
#else
    s_training_session.end_cached = !ok;
#endif
    os_unlock(lock_state);

    duration_ms = (ctx.end_time_ms >= ctx.start_time_ms) ?
                  (ctx.end_time_ms - ctx.start_time_ms) : 0ULL;
#if ZY100_MULTI_SESSION_STORAGE_ENABLE
    ZY100_LOG_VERBOSE("[TRAIN] end_cache ok user=%lu train=%lu start_ms=%llu end_ms=%llu duration_ms=%llu reason=%u multi_session=1",
                      (unsigned long)ctx.user_id,
                      (unsigned long)ctx.training_id,
                      (unsigned long long)ctx.start_time_ms,
                      (unsigned long long)ctx.end_time_ms,
                      (unsigned long long)duration_ms,
                      (uint32_t)ctx.stop_reason);
#else
    if (ok)
    {
        ZY100_LOG_VERBOSE("[TRAIN] end_write ok user=%lu train=%lu start_ms=%llu end_ms=%llu duration_ms=%llu reason=%u",
                          (unsigned long)ctx.user_id,
                          (unsigned long)ctx.training_id,
                          (unsigned long long)ctx.start_time_ms,
                          (unsigned long long)ctx.end_time_ms,
                          (unsigned long long)duration_ms,
                          (uint32_t)ctx.stop_reason);
    }
    else
    {
        DBG_DIRECT("[TRAIN] end_write failed user=%lu train=%lu reason=flash_write cached=1",
                   (unsigned long)ctx.user_id,
                   (unsigned long)ctx.training_id);
    }
#endif
    ZY100_LOG_VERBOSE("[TRAIN_GROUP] user=%lu train=%lu session=%lu round=%lu raw=%lu summary=%lu event=%lu",
                      (unsigned long)ctx.user_id,
                      (unsigned long)ctx.training_id,
                      (unsigned long)ctx.session_seq,
                      (unsigned long)ctx.capture_round,
                      (unsigned long)ctx.raw_block_count,
                      (unsigned long)ctx.summary_count,
                      (unsigned long)ctx.event_count);
    return ok;
}

bool zy100_training_session_write_start_failed(void)
{
    uint32_t lock_state;
    zy100_training_session_ctx_t local;
    bool begin_written = false;
    bool had_active = false;

    lock_state = os_lock();
    if (s_training_session.active_valid)
    {
        zy100_training_fill_end_time_if_needed(&s_training_session.active,
                                               ZY100_TRAINING_STOP_ERROR);
        local = s_training_session.active;
        begin_written = s_training_session.begin_written;
        had_active = true;
    }
    os_unlock(lock_state);

    if (!had_active)
    {
        return false;
    }

    DBG_DIRECT("[TRAIN] start_failed source=%s user=%lu train=%lu reason=imu_start_false begin_written=%u",
               zy100_training_source_name(local.source),
               (unsigned long)local.user_id,
               (unsigned long)local.training_id,
               begin_written ? 1U : 0U);
    return zy100_training_session_write_end_record();
}

void zy100_training_session_clear_active(void)
{
    uint32_t lock_state;

    lock_state = os_lock();
    memset(&s_training_session.active, 0, sizeof(s_training_session.active));
    s_training_session.active_valid = false;
    os_unlock(lock_state);
}

void zy100_training_session_clear_completed(void)
{
    uint32_t lock_state;

    lock_state = os_lock();
    memset(&s_training_session.completed, 0,
           sizeof(s_training_session.completed));
    s_training_session.completed_valid = false;
    s_training_session.begin_written = false;
    s_training_session.end_written = false;
    s_training_session.end_cached = false;
    os_unlock(lock_state);
}

bool zy100_training_session_has_export_context(void)
{
    bool has_context;
    uint32_t lock_state;

    lock_state = os_lock();
    has_context = s_training_session.active_valid ||
                  s_training_session.completed_valid ||
                  s_training_session.begin_written ||
                  s_training_session.end_written ||
                  s_training_session.end_cached;
    os_unlock(lock_state);
    return has_context;
}

zy100_training_meta_state_t zy100_training_meta_get_state(
    zy100_training_meta_verify_result_t *out)
{
    zy100_training_meta_verify_result_t local;
    zy100_training_meta_state_t state;
    bool erased;

    memset(&local, 0, sizeof(local));
    erased = zy100_training_meta_verify(&local);
    if (!local.read_ok)
    {
        if (local.status == IMU_STATUS_INVALID_PARAM)
        {
            state = ZY100_TRAINING_META_STATE_INVALID_LAYOUT;
        }
        else if (local.status == IMU_STATUS_NOT_READY)
        {
            state = ZY100_TRAINING_META_STATE_FLASH_NOT_READY;
        }
        else
        {
            state = ZY100_TRAINING_META_STATE_READ_FAIL;
        }
    }
    else if (erased)
    {
        state = ZY100_TRAINING_META_STATE_ERASED;
    }
    else
    {
        state = ZY100_TRAINING_META_STATE_NOT_ERASED;
    }

    if (out != NULL)
    {
        *out = local;
    }
    if ((state != ZY100_TRAINING_META_STATE_ERASED) ||
        ZY100_LOG_META_VERBOSE)
    {
        DBG_DIRECT("[TRAIN_META_STATE] state=%u read_ok=%u status=%u trnb_erased=%u trne_erased=%u",
                   (uint32_t)state,
                   local.read_ok ? 1U : 0U,
                   (uint32_t)local.status,
                   local.trnb_erased ? 1U : 0U,
                   local.trne_erased ? 1U : 0U);
    }
    return state;
}

bool zy100_training_meta_slots_are_erased(void)
{
    return zy100_training_meta_get_state(NULL) ==
           ZY100_TRAINING_META_STATE_ERASED;
}

imu_status_t zy100_training_meta_erase_and_verify(
    zy100_training_meta_verify_result_t *out)
{
    imu_status_t status;
    zy100_training_meta_verify_result_t verify;

    memset(&verify, 0, sizeof(verify));
    verify.read_ok = true;
    verify.status = IMU_STATUS_OK;
    verify.first_bad_addr = 0xFFFFFFFFUL;
    verify.first_bad = 0xFFU;
    if (out != NULL)
    {
        *out = verify;
    }

    zy100_training_meta_log_layout();

    if (!zy100_training_layout_valid())
    {
        DBG_DIRECT("[TRAIN_META_REPAIR] abort reason=invalid_layout");
        (void)zy100_training_meta_verify(out);
        return IMU_STATUS_INVALID_PARAM;
    }
    status = zy100_training_flash_ready_status();
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[TRAIN_META_REPAIR] ready_ret=%u", (uint32_t)status);
        (void)zy100_training_meta_verify(out);
        return status;
    }

    DBG_DIRECT("[TRAIN_META_REPAIR] erase_begin sector=0x%08lX size=4096",
               (unsigned long)ZY100_TRAINING_META_SECTOR_ADDR);
    if (!zy100_internal_meta_store_erase_sector(
            ZY100_TRAINING_META_SECTOR_ADDR))
    {
        (void)zy100_training_meta_verify(out);
        return IMU_STATUS_BUS_ERROR;
    }
    if (!zy100_training_meta_verify(&verify))
    {
        if (out != NULL)
        {
            *out = verify;
        }
        DBG_DIRECT("[TRAIN_META_REPAIR] verify_failed trnb_erased=%u trne_erased=%u read_ok=%u status=%u first_bad_addr=0x%08lX first_bad=0x%02X",
                   verify.trnb_erased ? 1U : 0U,
                   verify.trne_erased ? 1U : 0U,
                   verify.read_ok ? 1U : 0U,
                   (uint32_t)verify.status,
                   (unsigned long)verify.first_bad_addr,
                   verify.first_bad);
        return verify.read_ok ? IMU_STATUS_VERIFY_FAILED : verify.status;
    }
    if (out != NULL)
    {
        *out = verify;
    }
    ZY100_LOG_VERBOSE("[TRAIN_META_REPAIR] verify_ok trnb_erased=%u trne_erased=%u",
                      verify.trnb_erased ? 1U : 0U,
                      verify.trne_erased ? 1U : 0U);
    return IMU_STATUS_OK;
}

static bool zy100_training_cached_begin(uint8_t out[ZY100_TRAINING_RECORD_BYTES])
{
    zy100_training_session_ctx_t ctx;
    uint32_t lock_state;
    bool valid = false;

    lock_state = os_lock();
    if (s_training_session.active_valid)
    {
        ctx = s_training_session.active;
        valid = true;
    }
    else if (s_training_session.completed_valid)
    {
        ctx = s_training_session.completed;
        valid = true;
    }
    os_unlock(lock_state);

    if (!valid)
    {
        return false;
    }
    zy100_training_build_record(&ctx,
                                ZY100_TRAINING_RECORD_TYPE_BEGIN,
                                false,
                                out);
    return true;
}

static bool zy100_training_cached_end(uint8_t out[ZY100_TRAINING_RECORD_BYTES])
{
    zy100_training_session_ctx_t ctx;
    uint32_t lock_state;
    bool valid = false;

    lock_state = os_lock();
    if (s_training_session.completed_valid &&
        (s_training_session.completed.ended != 0U))
    {
        ctx = s_training_session.completed;
        valid = true;
    }
    os_unlock(lock_state);

    if (!valid)
    {
        return false;
    }
    zy100_training_build_record(&ctx,
                                ZY100_TRAINING_RECORD_TYPE_END,
                                ctx.stop_reason == ZY100_TRAINING_STOP_ERROR,
                                out);
    return true;
}

void zy100_training_session_get_export_info(
    zy100_training_session_export_info_t *out)
{
    uint8_t record[ZY100_TRAINING_RECORD_BYTES];
    bool begin_cached = false;
    bool end_cached = false;

    if (out == NULL)
    {
        return;
    }
    memset(out, 0, sizeof(*out));

    if (zy100_training_flash_read_record(ZY100_TRAINING_BEGIN_ADDR,
                                         ZY100_TRAINING_RECORD_TYPE_BEGIN,
                                         record))
    {
        out->begin_available = true;
        out->begin_bytes = ZY100_TRAINING_RECORD_BYTES;
        out->flags |= ZY100_TRAINING_EXPORT_BEGIN_PRESENT;
    }
    else if (zy100_training_cached_begin(record))
    {
        begin_cached = true;
        out->begin_available = true;
        out->begin_cached = true;
        out->begin_bytes = ZY100_TRAINING_RECORD_BYTES;
        out->flags |= ZY100_TRAINING_EXPORT_BEGIN_PRESENT |
                      ZY100_TRAINING_EXPORT_BEGIN_CACHED;
    }

    if (out->begin_available &&
        zy100_training_flash_read_record(ZY100_TRAINING_END_ADDR,
                                         ZY100_TRAINING_RECORD_TYPE_END,
                                         record))
    {
        out->end_available = true;
        out->end_bytes = ZY100_TRAINING_RECORD_BYTES;
        out->flags |= ZY100_TRAINING_EXPORT_END_PRESENT;
    }
    else if (out->begin_available && zy100_training_cached_end(record))
    {
        end_cached = true;
        out->end_available = true;
        out->end_cached = true;
        out->end_bytes = ZY100_TRAINING_RECORD_BYTES;
        out->flags |= ZY100_TRAINING_EXPORT_END_PRESENT |
                      ZY100_TRAINING_EXPORT_END_CACHED;
    }

    if (out->begin_available)
    {
        out->section_count++;
    }
    if (out->end_available)
    {
        out->section_count++;
    }
    if (out->begin_available && !out->end_available)
    {
        out->flags |= ZY100_TRAINING_EXPORT_INCOMPLETE;
    }
    (void)begin_cached;
    (void)end_cached;
}

bool zy100_training_session_get_begin_export_record(
    uint8_t out[ZY100_TRAINING_RECORD_BYTES],
    bool *from_cache)
{
    if (from_cache != NULL)
    {
        *from_cache = false;
    }
    if (zy100_training_flash_read_record(ZY100_TRAINING_BEGIN_ADDR,
                                         ZY100_TRAINING_RECORD_TYPE_BEGIN,
                                         out))
    {
        DBG_DIRECT("[TRAIN_EXPORT] begin user=%lu train=%lu session=%lu round=%lu",
                   (unsigned long)zy100_get_u32_le(&out[8]),
                   (unsigned long)zy100_get_u32_le(&out[12]),
                   (unsigned long)zy100_get_u32_le(&out[16]),
                   (unsigned long)zy100_get_u32_le(&out[20]));
        return true;
    }
    if (zy100_training_cached_begin(out))
    {
        if (from_cache != NULL)
        {
            *from_cache = true;
        }
        DBG_DIRECT("[TRAIN_EXPORT] begin user=%lu train=%lu session=%lu round=%lu source=cached",
                   (unsigned long)zy100_get_u32_le(&out[8]),
                   (unsigned long)zy100_get_u32_le(&out[12]),
                   (unsigned long)zy100_get_u32_le(&out[16]),
                   (unsigned long)zy100_get_u32_le(&out[20]));
        return true;
    }
    return false;
}

bool zy100_training_session_get_end_export_record(
    uint8_t out[ZY100_TRAINING_RECORD_BYTES],
    bool *from_cache)
{
    uint64_t start_ms;
    uint64_t end_ms;
    uint64_t duration_ms;

    if (from_cache != NULL)
    {
        *from_cache = false;
    }
    if (zy100_training_flash_read_record(ZY100_TRAINING_END_ADDR,
                                         ZY100_TRAINING_RECORD_TYPE_END,
                                         out))
    {
        DBG_DIRECT("[TRAIN_EXPORT] end user=%lu train=%lu session=%lu round=%lu",
                   (unsigned long)zy100_get_u32_le(&out[8]),
                   (unsigned long)zy100_get_u32_le(&out[12]),
                   (unsigned long)zy100_get_u32_le(&out[16]),
                   (unsigned long)zy100_get_u32_le(&out[20]));
        return true;
    }
    if (!zy100_training_cached_end(out))
    {
        return false;
    }

    if (from_cache != NULL)
    {
        *from_cache = true;
    }
    start_ms = zy100_get_u64_le(&out[24]);
    end_ms = zy100_get_u64_le(&out[32]);
    duration_ms = (end_ms >= start_ms) ? (end_ms - start_ms) : 0ULL;
    DBG_DIRECT("[TRAIN_EXPORT] end source=cached warn=trne_not_persisted");
    ZY100_LOG_VERBOSE("[CAP_TIME_EXPORT] time_source=%u calibrated_start=%u calibrated_stop=%u user=%lu train=%lu start_ms=%llu stop_ms=%llu duration_ms=%llu",
                      out[40],
                      ((out[7] & ZY100_TRAINING_FLAG_CAL_START) != 0U) ? 1U : 0U,
                      ((out[7] & ZY100_TRAINING_FLAG_CAL_END) != 0U) ? 1U : 0U,
                      (unsigned long)zy100_get_u32_le(&out[8]),
                      (unsigned long)zy100_get_u32_le(&out[12]),
                      (unsigned long long)start_ms,
                      (unsigned long long)end_ms,
                      (unsigned long long)duration_ms);
    return true;
}

static void zy100_training_ctx_from_view(
    const zy100_training_session_record_view_t *view,
    zy100_training_session_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->active = 0U;
    ctx->started = 1U;
    ctx->ended = 1U;
    ctx->user_id = view->user_id;
    ctx->training_id = view->training_id;
    ctx->session_seq = view->session_seq;
    ctx->capture_round = view->capture_round;
    ctx->time_source_start = view->time_source_start;
    ctx->time_source_end = view->time_source_end;
    ctx->calibrated_start = view->calibrated_start;
    ctx->calibrated_end = view->calibrated_end;
    ctx->start_time_ms = view->start_time_ms;
    ctx->end_time_ms = view->end_time_ms;
    ctx->source = view->source;
    ctx->stop_reason = view->stop_reason;
    ctx->raw_block_count = view->raw_block_count;
    ctx->summary_count = view->summary_count;
    ctx->event_count = view->event_count;
}

bool zy100_training_session_get_begin_export_record_for_session(
    const zy100_training_session_record_view_t *view,
    uint8_t out[ZY100_TRAINING_RECORD_BYTES])
{
    zy100_training_session_ctx_t ctx;

    if ((view == NULL) || (out == NULL))
    {
        return false;
    }
    zy100_training_ctx_from_view(view, &ctx);
    zy100_training_build_record(&ctx,
                                ZY100_TRAINING_RECORD_TYPE_BEGIN,
                                false,
                                out);
    DBG_DIRECT("[TRAIN_EXPORT] begin user=%lu train=%lu session=%lu round=%lu source=range",
               (unsigned long)view->user_id,
               (unsigned long)view->training_id,
               (unsigned long)view->session_seq,
               (unsigned long)view->capture_round);
    return true;
}

bool zy100_training_session_get_end_export_record_for_session(
    const zy100_training_session_record_view_t *view,
    uint8_t out[ZY100_TRAINING_RECORD_BYTES])
{
    zy100_training_session_ctx_t ctx;

    if ((view == NULL) || (out == NULL))
    {
        return false;
    }
    zy100_training_ctx_from_view(view, &ctx);
    zy100_training_build_record(&ctx,
                                ZY100_TRAINING_RECORD_TYPE_END,
                                view->stop_reason == ZY100_TRAINING_STOP_ERROR,
                                out);
    DBG_DIRECT("[TRAIN_EXPORT] end user=%lu train=%lu session=%lu round=%lu source=range",
               (unsigned long)view->user_id,
               (unsigned long)view->training_id,
               (unsigned long)view->session_seq,
               (unsigned long)view->capture_round);
    return true;
}

#else

void zy100_training_session_log_layout_once(void) {}

bool zy100_training_session_begin(zy100_training_source_t source,
                                  uint32_t user_id,
                                  uint32_t training_id,
                                  uint32_t session_seq,
                                  uint32_t capture_round)
{
    (void)source;
    (void)user_id;
    (void)training_id;
    (void)session_seq;
    (void)capture_round;
    return true;
}

void zy100_training_session_update_round(uint32_t session_seq,
                                         uint32_t capture_round)
{
    (void)session_seq;
    (void)capture_round;
}

void zy100_training_session_note_end_request(
    zy100_training_source_t request_source,
    zy100_training_stop_reason_t stop_reason)
{
    (void)request_source;
    (void)stop_reason;
}

void zy100_training_session_note_end_if_unset(
    zy100_training_stop_reason_t stop_reason)
{
    (void)stop_reason;
}

void zy100_training_session_set_counts(uint32_t raw_block_count,
                                       uint32_t summary_count,
                                       uint32_t event_count)
{
    (void)raw_block_count;
    (void)summary_count;
    (void)event_count;
}

bool zy100_training_session_write_end_record(void) { return true; }
bool zy100_training_session_write_start_failed(void) { return true; }
void zy100_training_session_clear_active(void) {}
void zy100_training_session_clear_completed(void) {}
bool zy100_training_session_has_export_context(void) { return false; }

zy100_training_meta_state_t zy100_training_meta_get_state(
    zy100_training_meta_verify_result_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
        out->trnb_erased = true;
        out->trne_erased = true;
        out->read_ok = true;
        out->status = IMU_STATUS_OK;
    }
    return ZY100_TRAINING_META_STATE_ERASED;
}

bool zy100_training_meta_slots_are_erased(void) { return true; }

imu_status_t zy100_training_meta_erase_and_verify(
    zy100_training_meta_verify_result_t *out)
{
    (void)zy100_training_meta_get_state(out);
    return IMU_STATUS_OK;
}

void zy100_training_session_get_export_info(
    zy100_training_session_export_info_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

bool zy100_training_session_get_begin_export_record(
    uint8_t out[ZY100_TRAINING_RECORD_BYTES],
    bool *from_cache)
{
    (void)out;
    if (from_cache != NULL) *from_cache = false;
    return false;
}

bool zy100_training_session_get_end_export_record(
    uint8_t out[ZY100_TRAINING_RECORD_BYTES],
    bool *from_cache)
{
    (void)out;
    if (from_cache != NULL) *from_cache = false;
    return false;
}

bool zy100_training_session_get_begin_export_record_for_session(
    const zy100_training_session_record_view_t *view,
    uint8_t out[ZY100_TRAINING_RECORD_BYTES])
{
    (void)view;
    (void)out;
    return false;
}

bool zy100_training_session_get_end_export_record_for_session(
    const zy100_training_session_record_view_t *view,
    uint8_t out[ZY100_TRAINING_RECORD_BYTES])
{
    (void)view;
    (void)out;
    return false;
}

#endif /* ZY100_LEGACY_OFFLINE_ENABLE */
