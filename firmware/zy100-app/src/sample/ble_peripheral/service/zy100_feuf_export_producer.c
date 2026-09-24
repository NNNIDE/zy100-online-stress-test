#include "zy100_feuf_export_producer.h"

#include <stddef.h>
#include <string.h>

#include "trace.h"

#include "../app_flags.h"
#include "../common/imu_common.h"
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
#include "../bsp/imu_bsp.h"
#endif
#include "../driver/gd25q32e_spi.h"
#include "zy100_capture_time.h"
#include "zy100_crc32.h"
#include "zy100_final_edge_raw_format.h"
#include "zy100_final_edge_raw_store.h"
#include "zy100_final_edge_record_format.h"
#include "zy100_final_edge_record_store.h"
#include "zy100_internal_meta_store.h"
#include "zy100_rtc_clock.h"

#if ZY100_FINAL_EDGE_MODE_ENABLE

#define ZY100_FEUF_MANIFEST_VERSION 2U
#define ZY100_FEUF_MANIFEST_VERSION_V3 3U

typedef enum
{
    ZY100_FEUF_SECTION_STEP_MANIFEST = 0U,
    ZY100_FEUF_SECTION_STEP_TRAIN_BEGIN,
    ZY100_FEUF_SECTION_STEP_META,
    ZY100_FEUF_SECTION_STEP_RAW,
    ZY100_FEUF_SECTION_STEP_SUMMARY,
    ZY100_FEUF_SECTION_STEP_EVENT,
    ZY100_FEUF_SECTION_STEP_TRAIN_END,
    ZY100_FEUF_SECTION_STEP_DONE,
} zy100_feuf_section_step_t;

typedef struct
{
    uint64_t start_ms;
    uint64_t end_ms;
    uint8_t time_source_start;
    uint8_t time_source_end;
    uint8_t calibrated_start;
    uint8_t calibrated_end;
    bool interval_valid;
    bool trusted_calibrated;
} zy100_feuf_resolved_time_t;

static uint32_t feuf_min_u32(uint32_t a, uint32_t b)
{
    return (a < b) ? a : b;
}

static uint16_t feuf_effective_payload_max(void)
{
#if ZY100_BLE_EXPORT_FEUF_PAYLOAD_FIT_BLE_ENABLE && \
    (ZY100_BLE_EXPORT_FEUF_PAYLOAD_MAX > 0U) && \
    (ZY100_BLE_EXPORT_FEUF_PAYLOAD_MAX <= ZY100_FEUF_MAX_PAYLOAD)
    return (uint16_t)ZY100_BLE_EXPORT_FEUF_PAYLOAD_MAX;
#else
    return (uint16_t)ZY100_FEUF_MAX_PAYLOAD;
#endif
}

static void feuf_set_error(zy100_feuf_export_producer_t *producer,
                           zy100_feuf_producer_error_t error)
{
    if (producer == NULL)
    {
        return;
    }
    producer->error = error;
    producer->state = ZY100_FEUF_PRODUCER_ERROR;
}

static uint32_t feuf_xor_bytes(const uint8_t *data, uint32_t len)
{
    uint32_t idx;
    uint32_t acc = 0U;
    uint32_t word = 0U;

    if (data == NULL)
    {
        return 0U;
    }
    for (idx = 0U; idx < len; idx++)
    {
        word = (word << 8) ^ (uint32_t)data[idx];
        if ((idx & 3U) == 3U)
        {
            acc ^= word;
            word = 0U;
        }
    }
    if ((len & 3U) != 0U)
    {
        acc ^= word;
    }
    return acc;
}

static bool feuf_resolve_session_time(
    const zy100_session_index_entry_t *session,
    zy100_feuf_resolved_time_t *out)
{
    uint64_t rtc_start_ms = 0ULL;
    uint64_t rtc_stop_ms = 0ULL;
    uint8_t cal_start = 0U;
    uint8_t cal_stop = 0U;
    uint8_t time_source = ZY100_RTC_TIME_SOURCE_DEFAULT_UNCALIBRATED;
    bool meta_decode_ok;
    bool meta_interval_valid;
    bool range_valid;

    if ((session == NULL) || (out == NULL))
    {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->time_source_start = ZY100_RTC_TIME_SOURCE_DEFAULT_UNCALIBRATED;
    out->time_source_end = ZY100_RTC_TIME_SOURCE_DEFAULT_UNCALIBRATED;
    out->calibrated_start = (session->time_calibrated != 0U) ? 1U : 0U;
    out->calibrated_end = out->calibrated_start;
    out->start_ms = session->start_time_ms;
    out->end_ms = session->end_time_ms;

    meta_decode_ok =
        zy100_capture_time_decode_meta(session->rtc_meta,
                                       &rtc_start_ms,
                                       &rtc_stop_ms,
                                       &cal_start,
                                       &cal_stop,
                                       &time_source);
    meta_interval_valid = meta_decode_ok &&
                          (rtc_start_ms != 0ULL) &&
                          (rtc_stop_ms >= rtc_start_ms);
    if (meta_decode_ok)
    {
        out->time_source_start = time_source;
        out->time_source_end = time_source;
        out->calibrated_start = (cal_start != 0U) ? 1U : 0U;
        out->calibrated_end = (cal_stop != 0U) ? 1U : 0U;
    }

    range_valid = (out->start_ms != 0ULL) &&
                  (out->end_ms != 0ULL) &&
                  (out->end_ms >= out->start_ms);
    if (!range_valid && meta_interval_valid)
    {
        out->start_ms = rtc_start_ms;
        out->end_ms = rtc_stop_ms;
        range_valid = true;
    }
    else if (!range_valid)
    {
        out->start_ms = 0ULL;
        out->end_ms = 0ULL;
    }

    out->interval_valid = range_valid;
    out->trusted_calibrated =
        range_valid &&
        (out->calibrated_start != 0U) &&
        (out->calibrated_end != 0U);
    return true;
}

static bool feuf_read_flash(zy100_feuf_export_producer_t *producer,
                            uint32_t addr,
                            uint8_t *buf,
                            uint16_t len)
{
    imu_status_t status;
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    uint64_t start_us;
    uint64_t end_us;
    uint32_t elapsed_us;

    start_us = imu_bsp_local_timestamp_us();
#endif
    status = gd25q32e_read_fast(addr, buf, len);
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    end_us = imu_bsp_local_timestamp_us();
    elapsed_us = (uint32_t)(end_us - start_us);
    if (producer != NULL)
    {
        producer->flash_perf.flash_read_calls++;
        producer->flash_perf.flash_read_bytes += len;
        producer->flash_perf.flash_read_total_us += elapsed_us;
        if ((producer->flash_perf.flash_read_min_us == 0U) ||
            (elapsed_us < producer->flash_perf.flash_read_min_us))
        {
            producer->flash_perf.flash_read_min_us = elapsed_us;
        }
        if (elapsed_us > producer->flash_perf.flash_read_max_us)
        {
            producer->flash_perf.flash_read_max_us = elapsed_us;
        }
        if (producer->flash_stream_perf_active)
        {
            producer->flash_stream_perf.flash_read_calls++;
            producer->flash_stream_perf.flash_read_bytes += len;
            producer->flash_stream_perf.flash_read_total_us += elapsed_us;
            if ((producer->flash_stream_perf.flash_read_min_us == 0U) ||
                (elapsed_us < producer->flash_stream_perf.flash_read_min_us))
            {
                producer->flash_stream_perf.flash_read_min_us = elapsed_us;
            }
            if (elapsed_us > producer->flash_stream_perf.flash_read_max_us)
            {
                producer->flash_stream_perf.flash_read_max_us = elapsed_us;
            }
        }
    }
#endif
    if (status != IMU_STATUS_OK)
    {
        if (producer != NULL)
        {
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
            producer->flash_perf.flash_read_fail_count++;
            if (producer->flash_stream_perf_active)
            {
                producer->flash_stream_perf.flash_read_fail_count++;
            }
#endif
            feuf_set_error(producer, ZY100_FEUF_PRODUCER_ERROR_FLASH_READ);
        }
        return false;
    }
    return true;
}

#if ZY100_BLE_EXPORT_PREFETCH_ENABLE
static void feuf_prefetch_invalidate(zy100_feuf_export_producer_t *producer)
{
    if (producer == NULL)
    {
        return;
    }
    producer->prefetch_cache_valid = false;
    producer->prefetch_cache_base_addr = 0U;
    producer->prefetch_cache_len = 0U;
    producer->prefetch_record_base_addr = 0U;
    producer->prefetch_record_end_addr = 0U;
}

static void feuf_prefetch_set_record(zy100_feuf_export_producer_t *producer,
                                     uint32_t base_addr,
                                     uint32_t record_len)
{
    if (producer == NULL)
    {
        return;
    }
    producer->prefetch_cache_valid = false;
    producer->prefetch_cache_base_addr = 0U;
    producer->prefetch_cache_len = 0U;
    producer->prefetch_record_base_addr = base_addr;
    producer->prefetch_record_end_addr = base_addr + record_len;
}

static bool feuf_prefetch_cache_hit(const zy100_feuf_export_producer_t *producer,
                                    uint32_t addr,
                                    uint16_t len)
{
    uint32_t cache_end;
    uint32_t req_end;

    if ((producer == NULL) || !producer->prefetch_cache_valid ||
        (producer->prefetch_cache == NULL))
    {
        return false;
    }
    cache_end = producer->prefetch_cache_base_addr +
                producer->prefetch_cache_len;
    req_end = addr + len;
    return (addr >= producer->prefetch_cache_base_addr) &&
           (req_end <= cache_end);
}

static bool feuf_read_record_cached(zy100_feuf_export_producer_t *producer,
                                    uint32_t addr,
                                    uint8_t *buf,
                                    uint16_t len)
{
    uint32_t req_end;
    uint32_t record_remaining;
    uint32_t read_len;
    uint32_t cache_offset;

    if ((producer == NULL) || (buf == NULL) || (len == 0U))
    {
        return false;
    }
    req_end = addr + len;
    if ((producer->prefetch_record_end_addr <=
         producer->prefetch_record_base_addr) ||
        (addr < producer->prefetch_record_base_addr) ||
        (req_end > producer->prefetch_record_end_addr))
    {
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        producer->prefetch_perf.bypass_count++;
#endif
        return feuf_read_flash(producer, addr, buf, len);
    }

    if (feuf_prefetch_cache_hit(producer, addr, len))
    {
        cache_offset = addr - producer->prefetch_cache_base_addr;
        memcpy(buf, &producer->prefetch_cache[cache_offset], len);
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        producer->prefetch_perf.hit_count++;
        producer->prefetch_perf.copied_bytes += len;
#endif
        return true;
    }

    record_remaining = producer->prefetch_record_end_addr - addr;
    if ((producer->prefetch_cache == NULL) ||
        (producer->prefetch_cache_capacity == 0U))
    {
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        producer->prefetch_perf.bypass_count++;
#endif
        return feuf_read_flash(producer, addr, buf, len);
    }

    read_len = feuf_min_u32(producer->prefetch_cache_capacity,
                            record_remaining);
    if (read_len < len)
    {
        read_len = len;
    }
    if (read_len > producer->prefetch_cache_capacity)
    {
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
        producer->prefetch_perf.bypass_count++;
#endif
        return feuf_read_flash(producer, addr, buf, len);
    }

#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    producer->prefetch_perf.miss_count++;
    producer->prefetch_perf.read_calls++;
    producer->prefetch_perf.read_bytes += read_len;
#endif
    producer->prefetch_cache_valid = false;
    if (!feuf_read_flash(producer,
                         addr,
                         producer->prefetch_cache,
                         (uint16_t)read_len))
    {
        feuf_prefetch_invalidate(producer);
        return false;
    }
    producer->prefetch_cache_base_addr = addr;
    producer->prefetch_cache_len = read_len;
    producer->prefetch_cache_valid = true;
    memcpy(buf, producer->prefetch_cache, len);
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    producer->prefetch_perf.copied_bytes += len;
#endif
    return true;
}
#else
static void feuf_prefetch_invalidate(zy100_feuf_export_producer_t *producer)
{
    (void)producer;
}

static void feuf_prefetch_set_record(zy100_feuf_export_producer_t *producer,
                                     uint32_t base_addr,
                                     uint32_t record_len)
{
    (void)producer;
    (void)base_addr;
    (void)record_len;
}

static bool feuf_read_record_cached(zy100_feuf_export_producer_t *producer,
                                    uint32_t addr,
                                    uint8_t *buf,
                                    uint16_t len)
{
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    if (producer != NULL)
    {
        producer->prefetch_perf.bypass_count++;
    }
#endif
    return feuf_read_flash(producer, addr, buf, len);
}
#endif

static bool feuf_read_session_meta(zy100_feuf_export_producer_t *producer,
                                   zy100_fe_session_meta_t *meta)
{
    zy100_fe_session_meta_t local_meta;
    uint32_t saved_xor;
    uint32_t actual_xor;

    if ((producer == NULL) || (meta == NULL))
    {
        return false;
    }
    if (!zy100_internal_meta_store_read(ZY100_FINAL_EDGE_SESSION_META_ADDR,
                                        (uint8_t *)&local_meta,
                                        (uint32_t)sizeof(local_meta)))
    {
        feuf_set_error(producer, ZY100_FEUF_PRODUCER_ERROR_FLASH_READ);
        return false;
    }
    saved_xor = local_meta.header_xor;
    local_meta.header_xor = 0U;
    actual_xor = feuf_xor_bytes((const uint8_t *)&local_meta,
                                (uint32_t)sizeof(local_meta));
    if ((local_meta.magic != ZY100_FE_SESSION_MAGIC) ||
        (local_meta.version != ZY100_FE_SESSION_VERSION) ||
        (local_meta.header_bytes != sizeof(local_meta)) ||
        (actual_xor != saved_xor))
    {
        DBG_DIRECT("[FE_BAD_RECORD] kind=session magic=0x%08x ver=%u hdr=%u xor=0x%08x saved=0x%08x",
                   local_meta.magic,
                   (uint32_t)local_meta.version,
                   (uint32_t)local_meta.header_bytes,
                   actual_xor,
                   saved_xor);
        feuf_set_error(producer, ZY100_FEUF_PRODUCER_ERROR_BAD_RECORD);
        return false;
    }
    local_meta.header_xor = saved_xor;
    *meta = local_meta;
    return true;
}

static bool feuf_read_raw_header(zy100_feuf_export_producer_t *producer,
                                 uint32_t raw_index,
                                 zy100_fe_high_raw_header_t *header,
                                 uint32_t *record_bytes)
{
    uint32_t addr;
    uint32_t bytes;

    if ((producer == NULL) || (header == NULL) || (record_bytes == NULL))
    {
        return false;
    }
    addr = producer->multi_session ?
           (producer->session.raw_data_begin_addr +
            (raw_index * ZY100_FINAL_EDGE_RAW_BUCKET_BYTES)) :
           (ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR +
            (raw_index * ZY100_FINAL_EDGE_RAW_BUCKET_BYTES));
    if (!feuf_read_flash(producer, addr, (uint8_t *)header, (uint16_t)sizeof(*header)))
    {
        return false;
    }
    bytes = header->payload_offset + header->payload_bytes;
    if ((header->magic != ZY100_FE_RAW_MAGIC) ||
        (header->version != ZY100_FE_RAW_VERSION) ||
        (header->header_bytes != sizeof(*header)) ||
        (header->status != ZY100_FE_RAW_STATUS_COMMITTED) ||
        (header->bucket_bytes != ZY100_FINAL_EDGE_RAW_BUCKET_BYTES) ||
        (header->payload_offset < sizeof(*header)) ||
        (bytes > ZY100_FINAL_EDGE_RAW_BUCKET_BYTES) ||
        (header->page_count == 0U))
    {
        DBG_DIRECT("[FE_BAD_RECORD] kind=raw idx=%u addr=0x%06x magic=0x%08x ver=%u hdr=%u status=%u",
                   raw_index,
                   addr,
                   header->magic,
                   (uint32_t)header->version,
                   (uint32_t)header->header_bytes,
                   (uint32_t)header->status);
        feuf_set_error(producer, ZY100_FEUF_PRODUCER_ERROR_BAD_RECORD);
        return false;
    }
    *record_bytes = bytes;
    return true;
}

static bool feuf_read_record_header(zy100_feuf_export_producer_t *producer,
                                    uint32_t addr,
                                    uint32_t expected_magic,
                                    uint16_t expected_type,
                                    uint32_t region_end,
                                    zy100_fe_record_flash_header_t *header)
{
    zy100_fe_record_flash_header_t local_header;
    uint32_t saved_xor;
    uint32_t actual_xor;

    if ((producer == NULL) || (header == NULL))
    {
        return false;
    }
    if (!feuf_read_flash(producer,
                         addr,
                         (uint8_t *)&local_header,
                         (uint16_t)sizeof(local_header)))
    {
        return false;
    }
    saved_xor = local_header.header_xor;
    local_header.header_xor = 0U;
    actual_xor = feuf_xor_bytes((const uint8_t *)&local_header,
                                (uint32_t)sizeof(local_header));
    if ((local_header.magic != expected_magic) ||
        (local_header.version != ZY100_FE_REC_VERSION) ||
        (local_header.header_bytes != sizeof(local_header)) ||
        (local_header.record_type != expected_type) ||
        (local_header.record_bytes == 0U) ||
        ((local_header.record_bytes % GD25Q32E_PAGE_BYTES) != 0U) ||
        ((addr + local_header.record_bytes) > region_end) ||
        (actual_xor != saved_xor))
    {
        DBG_DIRECT("[FE_BAD_RECORD] kind=record addr=0x%06x magic=0x%08x exp=0x%08x ver=%u hdr=%u",
                   addr,
                   local_header.magic,
                   expected_magic,
                   (uint32_t)local_header.version,
                   (uint32_t)local_header.header_bytes);
        feuf_set_error(producer, ZY100_FEUF_PRODUCER_ERROR_BAD_RECORD);
        return false;
    }
    local_header.header_xor = saved_xor;
    *header = local_header;
    return true;
}

static bool feuf_manifest_scan_raw(zy100_feuf_export_producer_t *producer,
                                   uint32_t count,
                                   uint32_t *bytes_out)
{
    uint32_t i;
    uint32_t total = 0U;

    if ((producer == NULL) || (bytes_out == NULL))
    {
        return false;
    }
    for (i = 0U; i < count; i++)
    {
        zy100_fe_high_raw_header_t header;
        uint32_t record_bytes = 0U;

        if (!feuf_read_raw_header(producer, i, &header, &record_bytes))
        {
            return false;
        }
        total += record_bytes;
    }
    *bytes_out = total;
    return true;
}

static bool feuf_manifest_scan_records(zy100_feuf_export_producer_t *producer,
                                       uint32_t base,
                                       uint32_t bytes,
                                       uint32_t count,
                                       uint32_t magic,
                                       uint16_t type,
                                       uint32_t *bytes_out)
{
    uint32_t i;
    uint32_t addr = base;
    uint32_t total = 0U;

    if ((producer == NULL) || (bytes_out == NULL))
    {
        return false;
    }
    for (i = 0U; i < count; i++)
    {
        zy100_fe_record_flash_header_t header;

        if (!feuf_read_record_header(producer,
                                     addr,
                                     magic,
                                     type,
                                     base + bytes,
                                     &header))
        {
            return false;
        }
        total += header.record_bytes;
        addr += header.record_bytes;
    }
    *bytes_out = total;
    return true;
}

static bool feuf_build_manifest(zy100_feuf_export_producer_t *producer)
{
    zy100_fe_session_meta_t meta;
    zy100_feuf_export_manifest_t manifest;
    zy100_training_session_export_info_t training_info;
    uint32_t manifest_crc;
    uint32_t data_section_count;
    uint32_t payload_max;

    if (producer == NULL)
    {
        return false;
    }

    memset(&manifest, 0, sizeof(manifest));
    memset(&training_info, 0, sizeof(training_info));
    if (!feuf_read_session_meta(producer, &meta))
    {
        return false;
    }

    zy100_training_session_log_layout_once();
    zy100_training_session_get_export_info(&training_info);
    zy100_capture_time_log_export_from_reserved(meta.reserved);
    manifest.protocol_version = ZY100_FEUF_MANIFEST_VERSION;
    manifest.session_version = ZY100_FE_SESSION_VERSION;
    manifest.round = meta.round;
    manifest.stop_reason = producer->stop_reason;
    manifest.live_hz = meta.live_hz;
    manifest.ois_hz = meta.ois_hz;
    manifest.raw_count = meta.raw_count;
    manifest.summary_count = meta.summary_count;
    manifest.event_count = meta.event_count;
    manifest.meta_bytes = (uint32_t)sizeof(zy100_fe_session_meta_t);
    DBG_DIRECT("[FE_MANIFEST_SCAN] round=%u raw=%u sum=%u evt=%u",
               manifest.round,
               manifest.raw_count,
               manifest.summary_count,
               manifest.event_count);
    if (!feuf_manifest_scan_raw(producer, meta.raw_count, &manifest.raw_bytes) ||
        !feuf_manifest_scan_records(producer,
                                    ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR,
                                    ZY100_FINAL_EDGE_SUMMARY_REGION_BYTES,
                                    meta.summary_count,
                                    ZY100_FE_REC_MAGIC_SUMMARY,
                                    ZY100_FE_REC_TYPE_SUMMARY,
                                    &manifest.summary_bytes) ||
        !feuf_manifest_scan_records(producer,
                                    ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR,
                                    ZY100_FINAL_EDGE_EVENT_REGION_BYTES,
                                    meta.event_count,
                                    ZY100_FE_REC_MAGIC_EVENT,
                                    ZY100_FE_REC_TYPE_EVENT,
                                    &manifest.event_bytes))
    {
        return false;
    }
    manifest.meta_base = ZY100_FINAL_EDGE_SESSION_META_ADDR;
    manifest.raw_base = ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR;
    manifest.summary_base = ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR;
    manifest.event_base = ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR;
    payload_max = (uint32_t)zy100_feuf_export_producer_payload_max(producer);
    manifest.payload_max = payload_max;
    manifest.training_section_count = training_info.section_count;
    manifest.training_begin_bytes = training_info.begin_bytes;
    manifest.training_end_bytes = training_info.end_bytes;
    manifest.training_flags = training_info.flags;
    manifest.section_mask_all = ZY100_FEUF_SECTION_BIT_EXISTING_ALL;
    if (training_info.begin_available)
    {
        manifest.section_mask_all |= ZY100_FEUF_SECTION_BIT_TRAINING_BEGIN;
    }
    if (training_info.end_available)
    {
        manifest.section_mask_all |= ZY100_FEUF_SECTION_BIT_TRAINING_END;
    }
    producer->available_section_mask = manifest.section_mask_all;
    data_section_count = 4U + training_info.section_count;
    manifest.total_sections = 2U + data_section_count;
    manifest.total_payload_bytes = training_info.begin_bytes +
                                   manifest.meta_bytes +
                                   manifest.raw_bytes +
                                   manifest.summary_bytes +
                                   manifest.event_bytes +
                                   training_info.end_bytes;
    manifest.total_frames_est =
        2U + data_section_count +
        ((manifest.total_payload_bytes + payload_max - 1U) / payload_max);
    manifest.manifest_crc32 = 0U;
    manifest_crc = zy100_crc32_ieee((const uint8_t *)&manifest,
                                    (uint32_t)sizeof(manifest));
    manifest.manifest_crc32 = manifest_crc;

    producer->manifest = manifest;
    producer->manifest_ready = true;
    if (training_info.begin_available && !training_info.end_available)
    {
        DBG_DIRECT("[TRAIN_EXPORT] incomplete begin=1 end=0");
    }
    DBG_DIRECT("[FEUF_MANIFEST] training_sections=%u total_sections=%u payload_bytes=%lu",
               manifest.training_section_count,
               manifest.total_sections,
               (unsigned long)manifest.total_payload_bytes);
    return true;
}

static bool feuf_build_manifest_session(
    zy100_feuf_export_producer_t *producer,
    const zy100_session_index_entry_t *session,
    uint32_t export_index,
    uint32_t export_total)
{
    zy100_feuf_export_manifest_v3_t manifest;
    zy100_training_session_export_info_t training_info;
    zy100_feuf_resolved_time_t resolved_time;
    uint32_t manifest_crc;
    uint32_t data_section_count;
    uint32_t payload_max;

    if ((producer == NULL) || (session == NULL))
    {
        return false;
    }
    memset(&manifest, 0, sizeof(manifest));
    memset(&training_info, 0, sizeof(training_info));
    memset(&resolved_time, 0, sizeof(resolved_time));
    (void)feuf_resolve_session_time(session, &resolved_time);

    if ((session->record_flags & ZY100_SESSION_DIR_RECORD_BEGIN_VALID) != 0U)
    {
        training_info.begin_available = true;
        training_info.begin_bytes = ZY100_TRAINING_RECORD_BYTES;
        training_info.flags |= ZY100_TRAINING_EXPORT_BEGIN_PRESENT;
        training_info.section_count++;
    }
    if ((session->record_flags & ZY100_SESSION_DIR_RECORD_END_VALID) != 0U)
    {
        training_info.end_available = true;
        training_info.end_bytes = ZY100_TRAINING_RECORD_BYTES;
        training_info.flags |= ZY100_TRAINING_EXPORT_END_PRESENT;
        training_info.section_count++;
    }

    producer->session = *session;
    producer->export_index = export_index;
    producer->export_total = export_total;
    manifest.protocol_version = ZY100_FEUF_MANIFEST_VERSION_V3;
    manifest.manifest_bytes = (uint32_t)sizeof(manifest);
    manifest.header_bytes = (uint16_t)sizeof(manifest);
    manifest.session_version = ZY100_FE_SESSION_VERSION;
    manifest.session_uid = session->session_uid;
    manifest.export_index = export_index;
    manifest.export_total = export_total;
    manifest.user_id = session->user_id;
    manifest.training_id = session->training_id;
    manifest.session_seq = session->session_seq;
    manifest.round = session->round;
    if (resolved_time.trusted_calibrated)
    {
        manifest.start_time_ms = resolved_time.start_ms;
        manifest.end_time_ms = resolved_time.end_ms;
    }
    manifest.stop_reason = session->stop_reason;
    manifest.live_hz = (session->live_hz != 0U) ?
                       session->live_hz : ZY100_FINAL_EDGE_LIVE_UI_HZ;
    manifest.ois_hz = (session->ois_hz != 0U) ?
                      session->ois_hz : ZY100_FINAL_EDGE_OIS_HZ;
    manifest.raw_data_begin_addr = session->raw_data_begin_addr;
    manifest.raw_data_end_addr = session->raw_data_end_addr;
    manifest.raw_count = session->raw_bucket_count;
    manifest.raw_used_bytes = session->raw_used_bytes;
    manifest.raw_reclaim_end_addr = session->raw_reclaim_end_addr;
    manifest.summary_data_begin_addr = session->summary_data_begin_addr;
    manifest.summary_data_end_addr = session->summary_data_end_addr;
    manifest.summary_count = session->summary_count;
    manifest.summary_used_bytes = session->summary_used_bytes;
    manifest.summary_reclaim_end_addr = session->summary_reclaim_end_addr;
    manifest.event_data_begin_addr = session->event_data_begin_addr;
    manifest.event_data_end_addr = session->event_data_end_addr;
    manifest.event_count = session->event_count;
    manifest.event_used_bytes = session->event_used_bytes;
    manifest.event_reclaim_end_addr = session->event_reclaim_end_addr;
    manifest.meta_bytes = (uint32_t)sizeof(zy100_fe_session_meta_t);

    DBG_DIRECT("[FE_MANIFEST_SCAN] v3 uid=%lu raw=%u sum=%u evt=%u",
               (unsigned long)session->session_uid,
               manifest.raw_count,
               manifest.summary_count,
               manifest.event_count);
    if (!feuf_manifest_scan_raw(producer,
                                session->raw_bucket_count,
                                &manifest.raw_bytes) ||
        !feuf_manifest_scan_records(producer,
                                    session->summary_data_begin_addr,
                                    session->summary_data_end_addr -
                                    session->summary_data_begin_addr,
                                    session->summary_count,
                                    ZY100_FE_REC_MAGIC_SUMMARY,
                                    ZY100_FE_REC_TYPE_SUMMARY,
                                    &manifest.summary_bytes) ||
        !feuf_manifest_scan_records(producer,
                                    session->event_data_begin_addr,
                                    session->event_data_end_addr -
                                    session->event_data_begin_addr,
                                    session->event_count,
                                    ZY100_FE_REC_MAGIC_EVENT,
                                    ZY100_FE_REC_TYPE_EVENT,
                                    &manifest.event_bytes))
    {
        return false;
    }

    payload_max = (uint32_t)zy100_feuf_export_producer_payload_max(producer);
    manifest.payload_max = payload_max;
    manifest.training_section_count = training_info.section_count;
    manifest.training_begin_bytes = training_info.begin_bytes;
    manifest.training_end_bytes = training_info.end_bytes;
    manifest.training_flags = training_info.flags;
    manifest.section_mask_all = ZY100_FEUF_SECTION_BIT_EXISTING_ALL;
    if (training_info.begin_available)
    {
        manifest.section_mask_all |= ZY100_FEUF_SECTION_BIT_TRAINING_BEGIN;
    }
    if (training_info.end_available)
    {
        manifest.section_mask_all |= ZY100_FEUF_SECTION_BIT_TRAINING_END;
    }
    producer->available_section_mask = manifest.section_mask_all;
    data_section_count = 4U + training_info.section_count;
    manifest.total_sections = 2U + data_section_count;
    manifest.total_payload_bytes = training_info.begin_bytes +
                                   manifest.meta_bytes +
                                   manifest.raw_bytes +
                                   manifest.summary_bytes +
                                   manifest.event_bytes +
                                   training_info.end_bytes;
    manifest.total_frames_est =
        2U + data_section_count +
        ((manifest.total_payload_bytes + payload_max - 1U) / payload_max);
    manifest.flags = session->flags;
    manifest.stream_crc32 = 0U;
    manifest.manifest_crc32 = 0U;
    manifest_crc = zy100_crc32_ieee((const uint8_t *)&manifest,
                                    (uint32_t)sizeof(manifest));
    manifest.manifest_crc32 = manifest_crc;

    producer->manifest_v3 = manifest;
    memset(&producer->manifest, 0, sizeof(producer->manifest));
    producer->manifest.protocol_version = ZY100_FEUF_MANIFEST_VERSION_V3;
    producer->manifest.round = session->round;
    producer->manifest.raw_count = session->raw_bucket_count;
    producer->manifest.summary_count = session->summary_count;
    producer->manifest.event_count = session->event_count;
    producer->manifest.meta_bytes = (uint32_t)sizeof(zy100_fe_session_meta_t);
    producer->manifest.raw_bytes = manifest.raw_bytes;
    producer->manifest.summary_bytes = manifest.summary_bytes;
    producer->manifest.event_bytes = manifest.event_bytes;
    producer->manifest.total_payload_bytes = manifest.total_payload_bytes;
    producer->manifest_ready = true;
    DBG_DIRECT("[FEUF_MANIFEST] v3 uid=%lu idx=%lu total=%lu payload_bytes=%lu manifest_bytes=%lu",
               (unsigned long)session->session_uid,
               (unsigned long)export_index,
               (unsigned long)export_total,
               (unsigned long)manifest.total_payload_bytes,
               (unsigned long)manifest.manifest_bytes);
    return true;
}

static uint32_t feuf_section_bit(uint32_t section_id)
{
    switch (section_id)
    {
    case ZY100_FEUF_SECTION_TRAINING_BEGIN:
        return ZY100_FEUF_SECTION_BIT_TRAINING_BEGIN;
    case ZY100_FEUF_SECTION_SESSION_META:
        return ZY100_FEUF_SECTION_BIT_META;
    case ZY100_FEUF_SECTION_RAW_RECORDS:
        return ZY100_FEUF_SECTION_BIT_RAW;
    case ZY100_FEUF_SECTION_SUMMARY_RECORDS:
        return ZY100_FEUF_SECTION_BIT_SUMMARY;
    case ZY100_FEUF_SECTION_EVENT_RECORDS:
        return ZY100_FEUF_SECTION_BIT_EVENT;
    case ZY100_FEUF_SECTION_TRAINING_END:
        return ZY100_FEUF_SECTION_BIT_TRAINING_END;
    default:
        return 0U;
    }
}

static void feuf_training_view_from_session(
    const zy100_session_index_entry_t *session,
    zy100_training_session_record_view_t *view)
{
    zy100_feuf_resolved_time_t resolved_time;

    memset(view, 0, sizeof(*view));
    memset(&resolved_time, 0, sizeof(resolved_time));
    (void)feuf_resolve_session_time(session, &resolved_time);

    view->user_id = session->user_id;
    view->training_id = session->training_id;
    view->session_seq = session->session_seq;
    view->capture_round = session->round;
    view->start_time_ms = resolved_time.start_ms;
    view->end_time_ms = resolved_time.end_ms;
    view->time_source_start = resolved_time.time_source_start;
    view->time_source_end = resolved_time.time_source_end;
    view->calibrated_start = resolved_time.calibrated_start;
    view->calibrated_end = resolved_time.calibrated_end;
    if (session->source == ZY100_SESSION_SOURCE_BLE)
    {
        view->source = ZY100_TRAINING_SRC_BLE;
    }
    else if (session->source == ZY100_SESSION_SOURCE_AUTO_MOTION)
    {
        view->source = ZY100_TRAINING_SRC_AUTO;
    }
    else
    {
        view->source = ZY100_TRAINING_SRC_BUTTON;
    }
    view->stop_reason = (zy100_training_stop_reason_t)session->stop_reason;
    view->raw_block_count = session->raw_bucket_count;
    view->summary_count = session->summary_count;
    view->event_count = session->event_count;
}

static bool feuf_start_section(zy100_feuf_export_producer_t *producer,
                               uint32_t section_id)
{
    producer->current_section = section_id;
    producer->current_section_offset = 0U;
    producer->current_record_index = 0U;
    producer->current_record_addr = 0U;
    producer->current_record_offset = 0U;
    producer->current_record_remaining = 0U;
    producer->current_record_memory = false;
    feuf_prefetch_invalidate(producer);
    memset(producer->current_record_mem, 0, sizeof(producer->current_record_mem));
    producer->summary_next_read_addr = producer->multi_session ?
        producer->session.summary_data_begin_addr :
        ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR;
    producer->event_next_read_addr = producer->multi_session ?
        producer->session.event_data_begin_addr :
        ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR;
    return true;
}

static bool feuf_advance_section(zy100_feuf_export_producer_t *producer)
{
    do
    {
        if (producer->section_step == ZY100_FEUF_SECTION_STEP_TRAIN_BEGIN)
        {
            producer->section_step = ZY100_FEUF_SECTION_STEP_META;
            if ((producer->section_mask & ZY100_FEUF_SECTION_BIT_TRAINING_BEGIN) != 0U)
            {
                return feuf_start_section(producer, ZY100_FEUF_SECTION_TRAINING_BEGIN);
            }
        }
        else if (producer->section_step == ZY100_FEUF_SECTION_STEP_META)
        {
            producer->section_step = ZY100_FEUF_SECTION_STEP_RAW;
            if ((producer->section_mask & ZY100_FEUF_SECTION_BIT_META) != 0U)
            {
                return feuf_start_section(producer, ZY100_FEUF_SECTION_SESSION_META);
            }
        }
        else if (producer->section_step == ZY100_FEUF_SECTION_STEP_RAW)
        {
            producer->section_step = ZY100_FEUF_SECTION_STEP_SUMMARY;
            if ((producer->section_mask & ZY100_FEUF_SECTION_BIT_RAW) != 0U)
            {
                return feuf_start_section(producer, ZY100_FEUF_SECTION_RAW_RECORDS);
            }
        }
        else if (producer->section_step == ZY100_FEUF_SECTION_STEP_SUMMARY)
        {
            producer->section_step = ZY100_FEUF_SECTION_STEP_EVENT;
            if ((producer->section_mask & ZY100_FEUF_SECTION_BIT_SUMMARY) != 0U)
            {
                return feuf_start_section(producer, ZY100_FEUF_SECTION_SUMMARY_RECORDS);
            }
        }
        else if (producer->section_step == ZY100_FEUF_SECTION_STEP_EVENT)
        {
            producer->section_step = ZY100_FEUF_SECTION_STEP_TRAIN_END;
            if ((producer->section_mask & ZY100_FEUF_SECTION_BIT_EVENT) != 0U)
            {
                return feuf_start_section(producer, ZY100_FEUF_SECTION_EVENT_RECORDS);
            }
        }
        else if (producer->section_step == ZY100_FEUF_SECTION_STEP_TRAIN_END)
        {
            producer->section_step = ZY100_FEUF_SECTION_STEP_DONE;
            if ((producer->section_mask & ZY100_FEUF_SECTION_BIT_TRAINING_END) != 0U)
            {
                return feuf_start_section(producer, ZY100_FEUF_SECTION_TRAINING_END);
            }
        }
        else
        {
            return false;
        }
    } while (true);
}

static bool feuf_prepare_next_record(zy100_feuf_export_producer_t *producer)
{
    if (producer->current_section == ZY100_FEUF_SECTION_TRAINING_BEGIN)
    {
        bool from_cache = false;

        if (producer->current_record_index != 0U)
        {
            return false;
        }
        if (producer->multi_session)
        {
            zy100_training_session_record_view_t view;

            feuf_training_view_from_session(&producer->session, &view);
            if (!zy100_training_session_get_begin_export_record_for_session(
                    &view,
                    producer->current_record_mem))
            {
                return false;
            }
        }
        else if (!zy100_training_session_get_begin_export_record(
                     producer->current_record_mem,
                     &from_cache))
        {
            return false;
        }
        producer->current_record_memory = true;
        feuf_prefetch_invalidate(producer);
        producer->current_record_remaining = ZY100_TRAINING_RECORD_BYTES;
        producer->current_record_offset = 0U;
        producer->current_record_index++;
        (void)from_cache;
        return true;
    }
    if (producer->current_section == ZY100_FEUF_SECTION_SESSION_META)
    {
        if (producer->current_record_index != 0U)
        {
            return false;
        }
        if (producer->multi_session)
        {
            zy100_fe_session_meta_t meta;

            if (!zy100_session_dir_build_session_meta(&producer->session,
                                                      &meta))
            {
                return false;
            }
            memcpy(producer->current_record_mem, &meta, sizeof(meta));
            producer->current_record_memory = true;
            feuf_prefetch_invalidate(producer);
        }
        else
        {
            zy100_fe_session_meta_t meta;

            if (!feuf_read_session_meta(producer, &meta))
            {
                return false;
            }
            memcpy(producer->current_record_mem, &meta, sizeof(meta));
            producer->current_record_memory = true;
            feuf_prefetch_invalidate(producer);
        }
        producer->current_record_remaining = (uint32_t)sizeof(zy100_fe_session_meta_t);
        producer->current_record_offset = 0U;
        producer->current_record_index++;
        return true;
    }
    if (producer->current_section == ZY100_FEUF_SECTION_RAW_RECORDS)
    {
        zy100_fe_high_raw_header_t header;
        uint32_t record_bytes = 0U;
        uint32_t index = producer->current_record_index;

        if (index >= producer->manifest.raw_count)
        {
            return false;
        }
        if (!feuf_read_raw_header(producer, index, &header, &record_bytes))
        {
            return false;
        }
        producer->current_record_addr =
            producer->multi_session ?
            (producer->session.raw_data_begin_addr +
             (index * ZY100_FINAL_EDGE_RAW_BUCKET_BYTES)) :
            (ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR +
             (index * ZY100_FINAL_EDGE_RAW_BUCKET_BYTES));
        producer->current_record_remaining = record_bytes;
        producer->current_record_offset = 0U;
        producer->current_record_index++;
        producer->current_record_memory = false;
        feuf_prefetch_set_record(producer,
                                 producer->current_record_addr,
                                 record_bytes);
        return true;
    }
    if (producer->current_section == ZY100_FEUF_SECTION_SUMMARY_RECORDS)
    {
        zy100_fe_record_flash_header_t header;
        uint32_t addr = producer->summary_next_read_addr;

        if (producer->current_record_index >= producer->manifest.summary_count)
        {
            return false;
        }
        if (!feuf_read_record_header(producer,
                                     addr,
                                     ZY100_FE_REC_MAGIC_SUMMARY,
                                     ZY100_FE_REC_TYPE_SUMMARY,
                                     producer->multi_session ?
                                     producer->session.summary_data_end_addr :
                                     (ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR +
                                      ZY100_FINAL_EDGE_SUMMARY_REGION_BYTES),
                                     &header))
        {
            return false;
        }
        producer->current_record_addr = addr;
        producer->current_record_remaining = header.record_bytes;
        producer->current_record_offset = 0U;
        producer->current_record_index++;
        producer->summary_next_read_addr = addr + header.record_bytes;
        producer->current_record_memory = false;
        feuf_prefetch_set_record(producer, addr, header.record_bytes);
        return true;
    }
    if (producer->current_section == ZY100_FEUF_SECTION_EVENT_RECORDS)
    {
        zy100_fe_record_flash_header_t header;
        uint32_t addr = producer->event_next_read_addr;

        if (producer->current_record_index >= producer->manifest.event_count)
        {
            return false;
        }
        if (!feuf_read_record_header(producer,
                                     addr,
                                     ZY100_FE_REC_MAGIC_EVENT,
                                     ZY100_FE_REC_TYPE_EVENT,
                                     producer->multi_session ?
                                     producer->session.event_data_end_addr :
                                     (ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR +
                                      ZY100_FINAL_EDGE_EVENT_REGION_BYTES),
                                     &header))
        {
            return false;
        }
        producer->current_record_addr = addr;
        producer->current_record_remaining = header.record_bytes;
        producer->current_record_offset = 0U;
        producer->current_record_index++;
        producer->event_next_read_addr = addr + header.record_bytes;
        producer->current_record_memory = false;
        feuf_prefetch_set_record(producer, addr, header.record_bytes);
        return true;
    }
    if (producer->current_section == ZY100_FEUF_SECTION_TRAINING_END)
    {
        bool from_cache = false;

        if (producer->current_record_index != 0U)
        {
            return false;
        }
        if (producer->multi_session)
        {
            zy100_training_session_record_view_t view;

            feuf_training_view_from_session(&producer->session, &view);
            if (!zy100_training_session_get_end_export_record_for_session(
                    &view,
                    producer->current_record_mem))
            {
                return false;
            }
        }
        else if (!zy100_training_session_get_end_export_record(
                     producer->current_record_mem,
                     &from_cache))
        {
            return false;
        }
        producer->current_record_memory = true;
        feuf_prefetch_invalidate(producer);
        producer->current_record_remaining = ZY100_TRAINING_RECORD_BYTES;
        producer->current_record_offset = 0U;
        producer->current_record_index++;
        (void)from_cache;
        return true;
    }
    return false;
}

static uint32_t feuf_header_crc(const zy100_feuf_frame_header_t *header)
{
    zy100_feuf_frame_header_t temp;

    memcpy(&temp, header, sizeof(temp));
    temp.header_crc32 = 0U;
    return zy100_crc32_ieee((const uint8_t *)&temp, (uint32_t)sizeof(temp));
}

static bool feuf_build_frame(zy100_feuf_export_producer_t *producer,
                             uint16_t frame_type,
                             uint32_t section_id,
                             uint32_t section_offset,
                             const uint8_t *payload,
                             uint16_t payload_len)
{
    zy100_feuf_frame_header_t header;
    uint32_t payload_crc;

    if ((producer == NULL) ||
        (payload_len > ZY100_FEUF_MAX_PAYLOAD) ||
        ((payload_len != 0U) && (payload == NULL)))
    {
        return false;
    }

    payload_crc = (payload_len == 0U) ? 0U :
                  zy100_crc32_ieee(payload, payload_len);
    memset(&header, 0, sizeof(header));
    header.magic = ZY100_FEUF_MAGIC;
    header.version = ZY100_FEUF_VERSION;
    header.header_bytes = (uint16_t)sizeof(header);
    header.frame_type = frame_type;
    header.seq = producer->seq_next++;
    header.section_id = section_id;
    header.section_offset = section_offset;
    header.payload_len = payload_len;
    header.payload_crc32 = payload_crc;
    header.header_crc32 = feuf_header_crc(&header);

    memcpy(producer->frame_buf, &header, sizeof(header));
    if (payload_len != 0U)
    {
        memcpy(&producer->frame_buf[sizeof(header)], payload, payload_len);
    }
    producer->frame_len = (uint16_t)(sizeof(header) + payload_len);
    producer->frame_pos = 0U;
    producer->frame_ready = true;
    producer->pending_frame_type = frame_type;
    producer->pending_section_bit = feuf_section_bit(section_id);
    producer->pending_payload_crc32 = payload_crc;
    producer->pending_payload_len = payload_len;
    producer->pending_frame_stream_bytes = producer->frame_len;
    producer->pending_section_advance = 0U;
    producer->pending_record_advance = 0U;
    return true;
}

static bool feuf_build_manifest_frame(zy100_feuf_export_producer_t *producer)
{
    if (producer->multi_session)
    {
        return feuf_build_frame(producer,
                                ZY100_FEUF_FRAME_MANIFEST,
                                ZY100_FEUF_SECTION_MANIFEST,
                                0U,
                                (const uint8_t *)&producer->manifest_v3,
                                (uint16_t)sizeof(producer->manifest_v3));
    }
    return feuf_build_frame(producer,
                            ZY100_FEUF_FRAME_MANIFEST,
                            ZY100_FEUF_SECTION_MANIFEST,
                            0U,
                            (const uint8_t *)&producer->manifest,
                            (uint16_t)sizeof(producer->manifest));
}

static bool feuf_build_section_end_frame(zy100_feuf_export_producer_t *producer)
{
    return feuf_build_frame(producer,
                            ZY100_FEUF_FRAME_SECTION_END,
                            producer->current_section,
                            producer->current_section_offset,
                            NULL,
                            0U);
}

static bool feuf_build_export_end_frame(zy100_feuf_export_producer_t *producer)
{
    zy100_feuf_export_end_payload_t payload;

    memset(&payload, 0, sizeof(payload));
    payload.session_id = producer->multi_session ?
                         producer->session.session_uid :
                         producer->manifest.round;
    payload.frames_sent = producer->frames_sent + 1U;
    payload.payload_bytes_sent = producer->payload_bytes_sent;
    payload.section_crc32_xor = producer->section_crc32_xor;
    payload.status = 0U;
    return feuf_build_frame(producer,
                            ZY100_FEUF_FRAME_EXPORT_END,
                            ZY100_FEUF_SECTION_END,
                            producer->payload_bytes_sent,
                            (const uint8_t *)&payload,
                            (uint16_t)sizeof(payload));
}

static bool feuf_build_data_or_section_frame(zy100_feuf_export_producer_t *producer)
{
    uint8_t payload[ZY100_FEUF_MAX_PAYLOAD];
    uint32_t max_payload;
    uint32_t chunk_len;

    if (producer->current_section == 0U)
    {
        if (!feuf_advance_section(producer))
        {
            return feuf_build_export_end_frame(producer);
        }
    }
    if (producer->current_record_remaining == 0U)
    {
        if (!feuf_prepare_next_record(producer))
        {
            return feuf_build_section_end_frame(producer);
        }
    }

    max_payload = (uint32_t)zy100_feuf_export_producer_payload_max(producer);
    chunk_len = feuf_min_u32(producer->current_record_remaining, max_payload);
    if (producer->current_record_memory)
    {
        memcpy(payload,
               &producer->current_record_mem[producer->current_record_offset],
               chunk_len);
    }
    else if (!feuf_read_record_cached(producer,
                                      producer->current_record_addr +
                                      producer->current_record_offset,
                                      payload,
                                      (uint16_t)chunk_len))
    {
        return false;
    }
    if (!feuf_build_frame(producer,
                          ZY100_FEUF_FRAME_DATA,
                          producer->current_section,
                          producer->current_section_offset,
                          payload,
                          (uint16_t)chunk_len))
    {
        return false;
    }
    producer->pending_record_advance = chunk_len;
    producer->pending_section_advance = chunk_len;
    return true;
}

static bool feuf_prepare_frame(zy100_feuf_export_producer_t *producer)
{
    if ((producer == NULL) || producer->done ||
        (producer->state == ZY100_FEUF_PRODUCER_ERROR))
    {
        return false;
    }
    if (producer->frame_ready)
    {
        return true;
    }

    if (producer->section_step == ZY100_FEUF_SECTION_STEP_MANIFEST)
    {
        producer->section_step = ZY100_FEUF_SECTION_STEP_TRAIN_BEGIN;
        return feuf_build_manifest_frame(producer);
    }
    return feuf_build_data_or_section_frame(producer);
}

static void feuf_commit_frame_done(zy100_feuf_export_producer_t *producer)
{
    producer->frames_sent++;
    producer->stream_offset += producer->pending_frame_stream_bytes;
    if (producer->pending_frame_type == ZY100_FEUF_FRAME_DATA)
    {
        producer->payload_bytes_sent += producer->pending_payload_len;
        producer->section_crc32_xor ^= producer->pending_payload_crc32;
        producer->current_record_offset += producer->pending_record_advance;
        producer->current_record_remaining -= producer->pending_record_advance;
        producer->current_section_offset += producer->pending_section_advance;
    }
    else if (producer->pending_frame_type == ZY100_FEUF_FRAME_SECTION_END)
    {
        producer->section_done_mask |= producer->pending_section_bit;
        producer->current_section = 0U;
    }
    else if (producer->pending_frame_type == ZY100_FEUF_FRAME_EXPORT_END)
    {
        producer->export_end_sent = true;
        producer->done = true;
        producer->active = false;
        producer->state = ZY100_FEUF_PRODUCER_DONE;
    }

    producer->frame_ready = false;
    producer->frame_len = 0U;
    producer->frame_pos = 0U;
    producer->pending_frame_type = 0U;
    producer->pending_section_bit = 0U;
    producer->pending_payload_crc32 = 0U;
    producer->pending_payload_len = 0U;
    producer->pending_section_advance = 0U;
    producer->pending_record_advance = 0U;
    producer->pending_frame_stream_bytes = 0U;
}

static bool feuf_commit_current_frame(zy100_feuf_export_producer_t *producer,
                                      uint16_t len)
{
    uint16_t remain;

    if ((producer == NULL) || (!producer->frame_ready) || (len == 0U))
    {
        return false;
    }
    remain = (uint16_t)(producer->frame_len - producer->frame_pos);
    if (len > remain)
    {
        return false;
    }
    producer->frame_pos = (uint16_t)(producer->frame_pos + len);
    if (producer->frame_pos >= producer->frame_len)
    {
        feuf_commit_frame_done(producer);
    }
    return true;
}

bool zy100_feuf_export_producer_begin(zy100_feuf_export_producer_t *producer,
                                      uint32_t stop_reason)
{
    bool busy = false;

    if (producer == NULL)
    {
        return false;
    }

    memset(producer, 0, sizeof(*producer));
    producer->payload_max = feuf_effective_payload_max();
    producer->stop_reason = stop_reason;
    producer->seq_next = 1U;
    producer->section_step = ZY100_FEUF_SECTION_STEP_MANIFEST;
    producer->state = ZY100_FEUF_PRODUCER_STREAMING;
    if ((gd25q32e_init() != IMU_STATUS_OK) ||
        (gd25q32e_is_busy(&busy) != IMU_STATUS_OK))
    {
        feuf_set_error(producer, ZY100_FEUF_PRODUCER_ERROR_FLASH_BUSY);
        return false;
    }
    if (busy)
    {
        feuf_set_error(producer, ZY100_FEUF_PRODUCER_ERROR_FLASH_BUSY);
        return false;
    }
    if (!feuf_build_manifest(producer))
    {
        if (producer->error == ZY100_FEUF_PRODUCER_ERROR_NONE)
        {
            feuf_set_error(producer, ZY100_FEUF_PRODUCER_ERROR_BAD_STATE);
        }
        return false;
    }
    producer->section_mask = producer->available_section_mask;
    producer->active = true;
    ZY100_LOG_VERBOSE("[BLE_EXPORT] pending_sessions=1 export_all=1");
    return true;
}

bool zy100_feuf_export_producer_begin_session(
    zy100_feuf_export_producer_t *producer,
    const zy100_session_index_entry_t *session,
    uint32_t export_index,
    uint32_t export_total)
{
    bool busy = false;

    if ((producer == NULL) || (session == NULL) ||
        (session->state != ZY100_SESSION_STATE_COMMITTED))
    {
        return false;
    }

    memset(producer, 0, sizeof(*producer));
    producer->payload_max = feuf_effective_payload_max();
    producer->multi_session = true;
    producer->stop_reason = session->stop_reason;
    producer->seq_next = 1U;
    producer->section_step = ZY100_FEUF_SECTION_STEP_MANIFEST;
    producer->state = ZY100_FEUF_PRODUCER_STREAMING;
    if ((gd25q32e_init() != IMU_STATUS_OK) ||
        (gd25q32e_is_busy(&busy) != IMU_STATUS_OK))
    {
        feuf_set_error(producer, ZY100_FEUF_PRODUCER_ERROR_FLASH_BUSY);
        return false;
    }
    if (busy)
    {
        feuf_set_error(producer, ZY100_FEUF_PRODUCER_ERROR_FLASH_BUSY);
        return false;
    }
    if (!feuf_build_manifest_session(producer,
                                     session,
                                     export_index,
                                     export_total))
    {
        if (producer->error == ZY100_FEUF_PRODUCER_ERROR_NONE)
        {
            feuf_set_error(producer, ZY100_FEUF_PRODUCER_ERROR_BAD_STATE);
        }
        return false;
    }
    producer->section_mask = producer->available_section_mask;
    producer->active = true;
    return true;
}

bool zy100_feuf_export_producer_peek(zy100_feuf_export_producer_t *producer,
                                     uint8_t *out,
                                     uint16_t out_cap,
                                     uint16_t *out_len,
                                     zy100_feuf_producer_progress_t *progress)
{
    uint16_t remain;
    uint16_t copy_len;

    if (out_len != NULL)
    {
        *out_len = 0U;
    }
    if ((producer == NULL) || (out == NULL) || (out_len == NULL) ||
        (out_cap == 0U))
    {
        return false;
    }
    if (producer->done)
    {
        return false;
    }
    if (!feuf_prepare_frame(producer))
    {
        return false;
    }
    remain = (uint16_t)(producer->frame_len - producer->frame_pos);
    copy_len = (remain < out_cap) ? remain : out_cap;
    memcpy(out, &producer->frame_buf[producer->frame_pos], copy_len);
    *out_len = copy_len;
    if (progress != NULL)
    {
        memset(progress, 0, sizeof(*progress));
        progress->frame_type = producer->pending_frame_type;
        progress->section_id = producer->current_section;
        progress->section_offset = producer->current_section_offset;
        progress->stream_offset = producer->stream_offset + producer->frame_pos;
        progress->frame_remaining = remain;
    }
    return true;
}

bool zy100_feuf_export_producer_peek_aggregate(
    zy100_feuf_export_producer_t *producer,
    uint8_t *out,
    uint16_t max_len,
    uint16_t target_len,
    uint8_t max_pieces,
    uint16_t *out_len,
    uint8_t *out_piece_count,
    zy100_feuf_producer_progress_t *progress)
{
    bool ok;

    if (out_piece_count != NULL)
    {
        *out_piece_count = 0U;
    }
    ok = zy100_feuf_export_producer_peek(producer,
                                         out,
                                         max_len,
                                         out_len,
                                         progress);
    if (out_piece_count != NULL)
    {
        *out_piece_count = ok ? 1U : 0U;
    }
    (void)target_len;
    (void)max_pieces;
    return ok;
}

bool zy100_feuf_export_producer_build_aggregate_plan(
    const zy100_feuf_export_producer_t *real,
    zy100_feuf_export_producer_t *scratch,
    uint8_t *out,
    uint16_t out_cap,
    uint16_t target_len,
    uint8_t max_pieces,
    zy100_feuf_export_aggregate_plan_t *plan)
{
    uint16_t total_len = 0U;
    uint16_t piece_len = 0U;
    uint16_t cap_len;
    uint8_t pieces = 0U;
    uint32_t payload_before;
    uint32_t stream_before;
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    uint32_t prefetch_calls_before;
    uint32_t prefetch_bytes_before;
#endif

    if (plan != NULL)
    {
        memset(plan, 0, sizeof(*plan));
    }
    if ((real == NULL) || (scratch == NULL) || (out == NULL) ||
        (plan == NULL) || (out_cap == 0U) || (max_pieces == 0U))
    {
        return false;
    }

    *scratch = *real;
    payload_before = real->payload_bytes_sent;
    stream_before = real->stream_offset + real->frame_pos;
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    prefetch_calls_before = scratch->prefetch_perf.read_calls;
    prefetch_bytes_before = scratch->prefetch_perf.read_bytes;
#endif
    cap_len = (target_len == 0U) ? out_cap : target_len;
    if (cap_len > out_cap)
    {
        cap_len = out_cap;
    }

    while ((total_len < cap_len) && (pieces < max_pieces) && !scratch->done)
    {
        zy100_feuf_producer_progress_t piece_progress;
        uint16_t avail = (uint16_t)(out_cap - total_len);

        memset(&piece_progress, 0, sizeof(piece_progress));
        if (avail == 0U)
        {
            break;
        }
        if (!zy100_feuf_export_producer_peek(scratch,
                                             &out[total_len],
                                             avail,
                                             &piece_len,
                                             &piece_progress) ||
            (piece_len == 0U))
        {
            break;
        }
        if (pieces == 0U)
        {
            plan->first_progress = piece_progress;
            plan->stream_offset_start = piece_progress.stream_offset;
        }
        plan->last_progress = piece_progress;
        if (piece_len >= piece_progress.frame_remaining)
        {
            plan->frames_completed++;
        }
        if (!zy100_feuf_export_producer_commit(scratch, piece_len))
        {
            return false;
        }
        total_len = (uint16_t)(total_len + piece_len);
        pieces++;
    }

    if ((total_len == 0U) || (pieces == 0U))
    {
        return false;
    }

    plan->after = *scratch;
    plan->len = total_len;
    plan->pieces = pieces;
    plan->data_payload_delta = scratch->payload_bytes_sent - payload_before;
    plan->stream_offset_end = scratch->stream_offset + scratch->frame_pos;
    if (plan->stream_offset_start == 0U)
    {
        plan->stream_offset_start = stream_before;
    }
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    plan->prefetch_read_calls_delta =
        scratch->prefetch_perf.read_calls - prefetch_calls_before;
    plan->prefetch_read_bytes_delta =
        scratch->prefetch_perf.read_bytes - prefetch_bytes_before;
#endif
    plan->valid = true;
    return true;
}

bool zy100_feuf_export_producer_commit(zy100_feuf_export_producer_t *producer,
                                       uint16_t len)
{
    bool ok;

    if ((producer == NULL) || (len == 0U))
    {
        return false;
    }
    ok = feuf_commit_current_frame(producer, len);
    if (!ok)
    {
        feuf_set_error(producer, ZY100_FEUF_PRODUCER_ERROR_BAD_STATE);
    }
    return ok;
}

bool zy100_feuf_export_producer_done(const zy100_feuf_export_producer_t *producer)
{
    return (producer != NULL) && producer->done;
}

uint32_t zy100_feuf_export_producer_error(const zy100_feuf_export_producer_t *producer)
{
    return (producer != NULL) ? (uint32_t)producer->error :
           (uint32_t)ZY100_FEUF_PRODUCER_ERROR_BAD_STATE;
}

uint32_t zy100_feuf_export_producer_export_id(const zy100_feuf_export_producer_t *producer)
{
    if (producer == NULL)
    {
        return 0U;
    }
    return producer->multi_session ? producer->session.session_uid :
           producer->manifest.round;
}

uint32_t zy100_feuf_export_producer_estimated_payload_bytes(
    const zy100_feuf_export_producer_t *producer)
{
    if ((producer == NULL) || !producer->manifest_ready)
    {
        return 0U;
    }
    return producer->manifest.total_payload_bytes;
}

uint32_t zy100_feuf_export_producer_estimated_bytes(
    const zy100_feuf_export_producer_t *producer)
{
    return zy100_feuf_export_producer_estimated_payload_bytes(producer);
}

uint16_t zy100_feuf_export_producer_payload_max(
    const zy100_feuf_export_producer_t *producer)
{
    if ((producer != NULL) &&
        (producer->payload_max > 0U) &&
        (producer->payload_max <= (uint16_t)ZY100_FEUF_MAX_PAYLOAD))
    {
        return producer->payload_max;
    }
    return (uint16_t)ZY100_FEUF_MAX_PAYLOAD;
}

uint32_t zy100_feuf_export_producer_payload_bytes_sent(
    const zy100_feuf_export_producer_t *producer)
{
    return (producer != NULL) ? producer->payload_bytes_sent : 0U;
}

void zy100_feuf_export_producer_set_prefetch_buffer(
    zy100_feuf_export_producer_t *producer,
    uint8_t *buffer,
    uint32_t buffer_bytes)
{
#if ZY100_BLE_EXPORT_PREFETCH_ENABLE
    if (producer == NULL)
    {
        return;
    }
    producer->prefetch_cache = buffer;
    producer->prefetch_cache_capacity = buffer_bytes;
    producer->prefetch_cache_valid = false;
    producer->prefetch_cache_base_addr = 0U;
    producer->prefetch_cache_len = 0U;
#else
    (void)producer;
    (void)buffer;
    (void)buffer_bytes;
#endif
}

void zy100_feuf_export_producer_adopt_prefetch_cache(
    zy100_feuf_export_producer_t *producer,
    const zy100_feuf_export_producer_t *source)
{
#if ZY100_BLE_EXPORT_PREFETCH_ENABLE
    if ((producer == NULL) || (source == NULL) ||
        (producer->prefetch_cache != source->prefetch_cache))
    {
        return;
    }
    producer->prefetch_cache_valid = source->prefetch_cache_valid;
    producer->prefetch_cache_base_addr = source->prefetch_cache_base_addr;
    producer->prefetch_cache_len = source->prefetch_cache_len;
#else
    (void)producer;
    (void)source;
#endif
}

void zy100_feuf_export_producer_p0_mark_stream_start(
    zy100_feuf_export_producer_t *producer)
{
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    if (producer == NULL)
    {
        return;
    }
    memset(&producer->flash_stream_perf, 0, sizeof(producer->flash_stream_perf));
    producer->flash_stream_perf_active = true;
#else
    (void)producer;
#endif
}

void zy100_feuf_export_producer_get_flash_perf(
    const zy100_feuf_export_producer_t *producer,
    zy100_feuf_export_flash_perf_t *out)
{
    if (out == NULL)
    {
        return;
    }
    memset(out, 0, sizeof(*out));
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    if (producer != NULL)
    {
        *out = producer->flash_perf;
    }
#else
    (void)producer;
#endif
}

void zy100_feuf_export_producer_get_flash_stream_perf(
    const zy100_feuf_export_producer_t *producer,
    zy100_feuf_export_flash_perf_t *out)
{
    if (out == NULL)
    {
        return;
    }
    memset(out, 0, sizeof(*out));
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    if (producer != NULL)
    {
        *out = producer->flash_stream_perf;
    }
#else
    (void)producer;
#endif
}

void zy100_feuf_export_producer_get_prefetch_perf(
    const zy100_feuf_export_producer_t *producer,
    zy100_feuf_export_prefetch_perf_t *out)
{
    if (out == NULL)
    {
        return;
    }
    memset(out, 0, sizeof(*out));
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
    if (producer != NULL)
    {
        *out = producer->prefetch_perf;
    }
#else
    (void)producer;
#endif
}

#else

bool zy100_feuf_export_producer_begin(zy100_feuf_export_producer_t *producer,
                                      uint32_t stop_reason)
{
    (void)producer;
    (void)stop_reason;
    return false;
}

bool zy100_feuf_export_producer_begin_session(
    zy100_feuf_export_producer_t *producer,
    const zy100_session_index_entry_t *session,
    uint32_t export_index,
    uint32_t export_total)
{
    (void)producer;
    (void)session;
    (void)export_index;
    (void)export_total;
    return false;
}

bool zy100_feuf_export_producer_peek(zy100_feuf_export_producer_t *producer,
                                     uint8_t *out,
                                     uint16_t out_cap,
                                     uint16_t *out_len,
                                     zy100_feuf_producer_progress_t *progress)
{
    (void)producer;
    (void)out;
    (void)out_cap;
    (void)progress;
    if (out_len != NULL)
    {
        *out_len = 0U;
    }
    return false;
}

bool zy100_feuf_export_producer_peek_aggregate(
    zy100_feuf_export_producer_t *producer,
    uint8_t *out,
    uint16_t max_len,
    uint16_t target_len,
    uint8_t max_pieces,
    uint16_t *out_len,
    uint8_t *out_piece_count,
    zy100_feuf_producer_progress_t *progress)
{
    (void)target_len;
    (void)max_pieces;
    if (out_piece_count != NULL)
    {
        *out_piece_count = 0U;
    }
    return zy100_feuf_export_producer_peek(producer,
                                           out,
                                           max_len,
                                           out_len,
                                           progress);
}

bool zy100_feuf_export_producer_build_aggregate_plan(
    const zy100_feuf_export_producer_t *real,
    zy100_feuf_export_producer_t *scratch,
    uint8_t *out,
    uint16_t out_cap,
    uint16_t target_len,
    uint8_t max_pieces,
    zy100_feuf_export_aggregate_plan_t *plan)
{
    (void)real;
    (void)scratch;
    (void)out;
    (void)out_cap;
    (void)target_len;
    (void)max_pieces;
    if (plan != NULL)
    {
        memset(plan, 0, sizeof(*plan));
    }
    return false;
}

bool zy100_feuf_export_producer_commit(zy100_feuf_export_producer_t *producer,
                                       uint16_t len)
{
    (void)producer;
    (void)len;
    return false;
}

bool zy100_feuf_export_producer_done(const zy100_feuf_export_producer_t *producer)
{
    (void)producer;
    return true;
}

uint32_t zy100_feuf_export_producer_error(const zy100_feuf_export_producer_t *producer)
{
    (void)producer;
    return (uint32_t)ZY100_FEUF_PRODUCER_ERROR_BAD_STATE;
}

uint32_t zy100_feuf_export_producer_export_id(const zy100_feuf_export_producer_t *producer)
{
    (void)producer;
    return 0U;
}

uint32_t zy100_feuf_export_producer_estimated_payload_bytes(
    const zy100_feuf_export_producer_t *producer)
{
    (void)producer;
    return 0U;
}

uint32_t zy100_feuf_export_producer_estimated_bytes(
    const zy100_feuf_export_producer_t *producer)
{
    return zy100_feuf_export_producer_estimated_payload_bytes(producer);
}

uint16_t zy100_feuf_export_producer_payload_max(
    const zy100_feuf_export_producer_t *producer)
{
    (void)producer;
    return (uint16_t)ZY100_FEUF_MAX_PAYLOAD;
}

uint32_t zy100_feuf_export_producer_payload_bytes_sent(
    const zy100_feuf_export_producer_t *producer)
{
    (void)producer;
    return 0U;
}

void zy100_feuf_export_producer_set_prefetch_buffer(
    zy100_feuf_export_producer_t *producer,
    uint8_t *buffer,
    uint32_t buffer_bytes)
{
    (void)producer;
    (void)buffer;
    (void)buffer_bytes;
}

void zy100_feuf_export_producer_adopt_prefetch_cache(
    zy100_feuf_export_producer_t *producer,
    const zy100_feuf_export_producer_t *source)
{
    (void)producer;
    (void)source;
}

void zy100_feuf_export_producer_p0_mark_stream_start(
    zy100_feuf_export_producer_t *producer)
{
    (void)producer;
}

void zy100_feuf_export_producer_get_flash_perf(
    const zy100_feuf_export_producer_t *producer,
    zy100_feuf_export_flash_perf_t *out)
{
    (void)producer;
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

void zy100_feuf_export_producer_get_flash_stream_perf(
    const zy100_feuf_export_producer_t *producer,
    zy100_feuf_export_flash_perf_t *out)
{
    (void)producer;
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

void zy100_feuf_export_producer_get_prefetch_perf(
    const zy100_feuf_export_producer_t *producer,
    zy100_feuf_export_prefetch_perf_t *out)
{
    (void)producer;
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

#endif
