#include "zy100_final_edge_raw_store.h"

#include <string.h>

#include "trace.h"
#include "../app_flags.h"
#include "../bsp/imu_bsp.h"
#include "../common/zy100_byteorder.h"
#include "../driver/gd25q32e_spi.h"
#include "zy100_final_edge_lf_pre_ring.h"

#if (ZY100_FINAL_EDGE_PHASE5_RAW_FLASH_ENABLE != 0U) && \
    (ZY100_FINAL_EDGE_RAW_STORE_ENABLE != 0U)

#define ZY100_FE_RAW_PAGE_BYTES GD25Q32E_PAGE_BYTES
#define ZY100_FE_RAW_SECTOR_BYTES GD25Q32E_SECTOR_BYTES
#define ZY100_FE_RAW_PAYLOAD_OFFSET ZY100_FE_RAW_PAGE_BYTES
#define ZY100_FE_RAW_MAX_LF_PACKETS 40U
#define ZY100_FE_RAW_PREP_LOG_STEP 32U
#define ZY100_FE_RAW_VERIFY_CHUNK_BYTES 16U

typedef char zy100_fe_raw_bucket_check[
    (ZY100_FINAL_EDGE_RAW_BUCKET_BYTES == 16384U) ? 1 : -1];
typedef char zy100_fe_raw_bucket_sector_align_check[
    ((ZY100_FINAL_EDGE_RAW_BUCKET_BYTES % GD25Q32E_SECTOR_BYTES) == 0U) ?
        1 : -1];
typedef char zy100_fe_raw_bucket_page_align_check[
    ((ZY100_FINAL_EDGE_RAW_BUCKET_BYTES % GD25Q32E_PAGE_BYTES) == 0U) ?
        1 : -1];
typedef char zy100_fe_raw_region_base_align_check[
    ((ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR % GD25Q32E_SECTOR_BYTES) == 0UL) ?
        1 : -1];
typedef char zy100_fe_raw_region_end_align_check[
    (((ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR +
       ZY100_FINAL_EDGE_RAW_REGION_BYTES) % GD25Q32E_SECTOR_BYTES) == 0UL) ?
        1 : -1];
typedef char zy100_fe_raw_region_order_check[
    ((ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR +
      ZY100_FINAL_EDGE_RAW_REGION_BYTES) <=
     ZY100_FINAL_EDGE_SUMMARY_REGION_BASE_ADDR) ? 1 : -1];
typedef char zy100_fe_raw_flash_size_check[
    ((ZY100_FINAL_EDGE_EVENT_REGION_BASE_ADDR +
      ZY100_FINAL_EDGE_EVENT_REGION_BYTES) <= GD25Q32E_FLASH_SIZE_BYTES) ?
        1 : -1];
typedef char zy100_fe_raw_hf_ring_check[
    (ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES == 320U) ? 1 : -1];
typedef char zy100_fe_raw_ois_frame_check[
    (ZY100_FINAL_EDGE_OIS_FRAME_BYTES == 19U) ? 1 : -1];
typedef char zy100_fe_raw_lf_packet_check[
    (ZY100_FINAL_EDGE_LF_PRE_PACKET_BYTES == 16U) ? 1 : -1];
typedef char zy100_fe_raw_header_size_check[
    (sizeof(zy100_fe_high_raw_header_t) <= GD25Q32E_PAGE_BYTES) ? 1 : -1];
typedef char zy100_fe_raw_payload_fit_check[
    ((ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES *
      ZY100_FINAL_EDGE_OIS_FRAME_BYTES) +
     (ZY100_FE_RAW_MAX_LF_PACKETS * ZY100_FINAL_EDGE_LF_PRE_PACKET_BYTES) +
     (2U * sizeof(zy100_fe_raw_section_header_t)) +
     sizeof(zy100_fe_high_raw_header_t) < ZY100_FINAL_EDGE_RAW_BUCKET_BYTES) ?
        1 : -1];

#if (ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR != 0UL)
#define ZY100_FE_RAW_ADDR_BEFORE_BASE(addr) \
    ((addr) < ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR)
#else
#define ZY100_FE_RAW_ADDR_BEFORE_BASE(addr) 0
#endif

typedef enum
{
    ZY100_FE_RAW_PREP_IDLE = 0U,
    ZY100_FE_RAW_PREP_ERASE,
    ZY100_FE_RAW_PREP_DONE,
    ZY100_FE_RAW_PREP_ERROR,
} zy100_fe_raw_prep_state_t;

typedef enum
{
    ZY100_FE_RAW_STATE_IDLE = 0U,
    ZY100_FE_RAW_STATE_WAIT_TARGET_BUSY,
    ZY100_FE_RAW_STATE_WAIT_TARGET_SPACE,
    ZY100_FE_RAW_STATE_PAYLOAD,
    ZY100_FE_RAW_STATE_HEADER,
} zy100_fe_raw_state_t;

typedef struct
{
    zy100_fe_raw_prep_state_t prep_state;
    zy100_fe_raw_state_t state;

    bool prepared;
    bool running;
    bool record_active;
    bool pending;
    bool program_in_flight;
    bool prep_erase_in_flight;
    bool b_active;
    bool finalizing;
    bool error;

    uint32_t prep_round;
    uint32_t prep_base_addr;
    uint32_t run_round;
    uint32_t prep_total_sectors;
    uint32_t prep_done_sectors;
    uint32_t prep_last_log_sector;

    uint32_t bucket_count;
    uint32_t session_raw_first_bucket;
    uint32_t session_raw_limit_bucket;
    bool append_erase_dynamic;
    bool append_erased_bucket_valid;
    uint32_t append_erased_bucket_addr;
    uint32_t current_raw_id;
    uint32_t current_bucket_addr;
    uint32_t payload_bytes;
    uint32_t payload_pages;
    uint32_t payload_page_next;
    uint32_t in_flight_addr;
    uint32_t in_flight_page_index;
    bool in_flight_header;
    const zy100_fe_store_target_provider_t *target_provider;
    uint32_t target_token;

    const uint8_t *raw_ring;
    uint32_t raw_ring_bytes;
    zy100_final_edge_bscan_result_t result;
    zy100_final_edge_hit_window_view_t view;
    zy100_final_edge_lf_pre_entry_t lf_entries[ZY100_FE_RAW_MAX_LF_PACKETS];
    uint32_t lf_count;
    uint32_t section_count;
    uint32_t flags;
    uint32_t payload_xor;
    uint32_t rt_trigger_sample;
    uint16_t rt_trigger_tmst_raw;

    uint32_t gate_start_ms;
    bool record_start_valid;
    uint32_t record_start_us;
    uint32_t target_wait_start_us;
    uint8_t page[ZY100_FE_RAW_PAGE_BYTES];

    zy100_fe_raw_store_stats_t stats;
} zy100_fe_raw_store_t;

static zy100_fe_raw_store_t s_fe_raw_store;
static uint32_t s_fe_raw_online_verified_pages;
static uint32_t s_fe_raw_online_address_violations;

#define ZY100_FE_RAW_ERR_STAGE_NONE          0U
#define ZY100_FE_RAW_ERR_STAGE_BUSY_QUERY    1U
#define ZY100_FE_RAW_ERR_STAGE_VERIFY_READ   2U
#define ZY100_FE_RAW_ERR_STAGE_VERIFY_DATA   3U
#define ZY100_FE_RAW_ERR_STAGE_PAYLOAD_COPY  4U
#define ZY100_FE_RAW_ERR_STAGE_PAYLOAD_WRITE 5U
#define ZY100_FE_RAW_ERR_STAGE_HEADER_WRITE  6U
#define ZY100_FE_RAW_ERR_STAGE_TARGET_RESERVE 7U

static void fe_raw_note_error(uint32_t stage,
                              uint32_t status,
                              uint32_t addr);
static uint32_t fe_raw_elapsed_us(uint32_t start_us);
static void fe_raw_max_u32(uint32_t *target, uint32_t value);
static void fe_raw_add_u32_sat(uint32_t *target, uint32_t value);

static bool fe_raw_waiting_target(void)
{
    return (s_fe_raw_store.state == ZY100_FE_RAW_STATE_WAIT_TARGET_BUSY) ||
           (s_fe_raw_store.state == ZY100_FE_RAW_STATE_WAIT_TARGET_SPACE);
}

static uint32_t fe_raw_target_record_bytes(void)
{
    return (s_fe_raw_store.payload_pages + 1U) * ZY100_FE_RAW_PAGE_BYTES;
}

static void fe_raw_note_target_wait_done(void)
{
    uint32_t wait_ms;

    if (s_fe_raw_store.target_wait_start_us == 0U)
    {
        return;
    }
    wait_ms = fe_raw_elapsed_us(s_fe_raw_store.target_wait_start_us) / 1000U;
    fe_raw_add_u32_sat(&s_fe_raw_store.stats.target_wait_total_ms, wait_ms);
    fe_raw_max_u32(&s_fe_raw_store.stats.target_wait_max_ms, wait_ms);
    s_fe_raw_store.stats.target_wait_last_ms = wait_ms;
    s_fe_raw_store.target_wait_start_us = 0U;
}

static zy100_fe_store_reserve_result_t fe_raw_try_reserve_target(void)
{
    zy100_fe_store_target_t target;
    zy100_fe_store_reserve_result_t result;
    uint32_t record_bytes = fe_raw_target_record_bytes();

    if ((s_fe_raw_store.target_provider == NULL) ||
        (s_fe_raw_store.target_provider->reserve == NULL))
    {
        return ZY100_FE_STORE_RESERVE_ERROR;
    }
    memset(&target, 0, sizeof(target));
    result = s_fe_raw_store.target_provider->reserve(
        s_fe_raw_store.target_provider->context,
        ZY100_FE_STORE_TARGET_RAW,
        s_fe_raw_store.current_raw_id,
        record_bytes,
        s_fe_raw_store.payload_bytes,
        &target);
    s_fe_raw_store.stats.target_last_reserve_result = (uint32_t)result;
    if ((result == ZY100_FE_STORE_RESERVE_OK) &&
        (target.capacity_bytes >= record_bytes))
    {
        s_fe_raw_store.current_bucket_addr = target.data_addr;
        s_fe_raw_store.target_token = target.token;
        s_fe_raw_store.state = ZY100_FE_RAW_STATE_PAYLOAD;
        fe_raw_note_target_wait_done();
        return ZY100_FE_STORE_RESERVE_OK;
    }
    if (result == ZY100_FE_STORE_RESERVE_BUSY)
    {
        s_fe_raw_store.state = ZY100_FE_RAW_STATE_WAIT_TARGET_BUSY;
        s_fe_raw_store.stats.target_wait_busy++;
    }
    else if (result == ZY100_FE_STORE_RESERVE_FULL)
    {
        s_fe_raw_store.state = ZY100_FE_RAW_STATE_WAIT_TARGET_SPACE;
        s_fe_raw_store.stats.target_wait_space++;
    }
    else
    {
        result = ZY100_FE_STORE_RESERVE_ERROR;
        s_fe_raw_store.stats.target_retry_error++;
        fe_raw_note_error(ZY100_FE_RAW_ERR_STAGE_TARGET_RESERVE,
                          s_fe_raw_store.stats.target_last_reserve_result,
                          0U);
    }
    if ((result != ZY100_FE_STORE_RESERVE_ERROR) &&
        (s_fe_raw_store.target_wait_start_us == 0U))
    {
        s_fe_raw_store.target_wait_start_us =
            (uint32_t)imu_bsp_local_timestamp_us();
        s_fe_raw_store.stats.target_wait_bscan_id =
            s_fe_raw_store.view.bscan_id;
    }
    return result;
}

static void fe_raw_note_error(uint32_t stage,
                              uint32_t status,
                              uint32_t addr)
{
    s_fe_raw_store.stats.last_error_stage = stage;
    s_fe_raw_store.stats.last_error_status = status;
    s_fe_raw_store.stats.last_error_raw_id = s_fe_raw_store.current_raw_id;
    s_fe_raw_store.stats.last_error_addr = addr;
    s_fe_raw_store.stats.last_error_expected = 0U;
    s_fe_raw_store.stats.last_error_readback = 0U;
}

static void fe_raw_note_error_detail(uint32_t stage,
                                     uint32_t status,
                                     uint32_t addr,
                                     uint32_t expected,
                                     uint32_t readback)
{
    fe_raw_note_error(stage, status, addr);
    s_fe_raw_store.stats.last_error_expected = expected;
    s_fe_raw_store.stats.last_error_readback = readback;
}

static bool fe_raw_flash_prepare(uint32_t stage, uint32_t addr)
{
    imu_status_t status = gd25q32e_init();

    if (status == IMU_STATUS_OK)
    {
        return true;
    }
    s_fe_raw_store.stats.write_error++;
    fe_raw_note_error(stage, (uint32_t)status, addr);
    s_fe_raw_store.error = true;
    return false;
}

static uint32_t fe_raw_elapsed_us(uint32_t start_us)
{
    return (uint32_t)((uint32_t)imu_bsp_local_timestamp_us() - start_us);
}

static void fe_raw_max_u32(uint32_t *target, uint32_t value)
{
    if ((target != NULL) && (value > *target))
    {
        *target = value;
    }
}

static void fe_raw_add_u32_sat(uint32_t *target, uint32_t value)
{
    if (target == NULL)
    {
        return;
    }
    if ((0xFFFFFFFFU - *target) < value)
    {
        *target = 0xFFFFFFFFU;
    }
    else
    {
        *target += value;
    }
}

static void fe_raw_note_pump_elapsed(zy100_fe_raw_pump_context_t context,
                                     uint32_t start_us)
{
    uint32_t elapsed_us = fe_raw_elapsed_us(start_us);

    fe_raw_max_u32(&s_fe_raw_store.stats.pump_max_us, elapsed_us);
    if (context == ZY100_FE_RAW_PUMP_CONTEXT_FINAL)
    {
        fe_raw_max_u32(&s_fe_raw_store.stats.final_pump_max_us, elapsed_us);
    }
    else
    {
        fe_raw_max_u32(&s_fe_raw_store.stats.runtime_pump_max_us,
                       elapsed_us);
    }
}

static uint32_t fe_raw_xor_bytes(uint32_t seed,
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

static void fe_raw_fill_section_header(zy100_fe_raw_section_header_t *header,
                                       uint16_t section_type,
                                       uint32_t sample_hz,
                                       uint32_t unit_bytes,
                                       uint32_t count,
                                       uint16_t first_tmst_raw,
                                       uint16_t last_tmst_raw,
                                       uint32_t payload_bytes,
                                       uint32_t flags)
{
    if (header == NULL)
    {
        return;
    }

    memset(header, 0, sizeof(*header));
    header->section_type = section_type;
    header->header_bytes = (uint16_t)sizeof(*header);
    header->sample_hz = sample_hz;
    header->packet_or_frame_bytes = unit_bytes;
    header->count = count;
    header->first_tmst_raw = first_tmst_raw;
    header->last_tmst_raw = last_tmst_raw;
    header->payload_bytes = payload_bytes;
    header->flags = flags;
}

static void fe_raw_build_committed_header(zy100_fe_high_raw_header_t *header)
{
    if (header == NULL)
    {
        return;
    }

    memset(header, 0, sizeof(*header));
    header->magic = ZY100_FE_RAW_MAGIC;
    header->version = ZY100_FE_RAW_VERSION;
    header->header_bytes = (uint16_t)sizeof(*header);
    header->raw_id = s_fe_raw_store.current_raw_id;
    header->bscan_id = s_fe_raw_store.view.bscan_id;
    header->rt_trigger_sample = s_fe_raw_store.rt_trigger_sample;
    header->rt_trigger_tmst_raw = s_fe_raw_store.rt_trigger_tmst_raw;
    header->hit_tmst_raw = s_fe_raw_store.view.hit_tmst_raw;
    header->hit_seq = s_fe_raw_store.view.hit_seq;
    header->hit_score = s_fe_raw_store.result.hit_score;
    header->hit_score20 = s_fe_raw_store.result.hit_score20;
    header->start_seq = s_fe_raw_store.view.start_seq;
    header->end_seq = s_fe_raw_store.view.end_seq;
    header->hf_frames = s_fe_raw_store.view.hf_frames;
    header->lf_packets = s_fe_raw_store.lf_count;
    header->first_tmst_raw = s_fe_raw_store.view.first_tmst_raw;
    header->last_tmst_raw = s_fe_raw_store.view.last_tmst_raw;
    header->section_count = s_fe_raw_store.section_count;
    header->payload_offset = ZY100_FE_RAW_PAYLOAD_OFFSET;
    header->payload_bytes = s_fe_raw_store.payload_bytes;
    header->bucket_bytes = ZY100_FINAL_EDGE_RAW_BUCKET_BYTES;
    header->flags = s_fe_raw_store.flags;
    header->window_xor = s_fe_raw_store.view.window_xor;
    header->payload_xor = s_fe_raw_store.payload_xor;
    header->page_count = s_fe_raw_store.payload_pages + 1U;
    header->status = ZY100_FE_RAW_STATUS_COMMITTED;
}

static uint32_t fe_raw_copy_segment(uint32_t request_offset,
                                    uint8_t *dst,
                                    uint32_t len,
                                    uint32_t segment_offset,
                                    const uint8_t *segment,
                                    uint32_t segment_len)
{
    uint32_t start;
    uint32_t copy_len;

    if ((dst == NULL) || (segment == NULL) || (len == 0U) ||
        (segment_len == 0U))
    {
        return 0U;
    }
    if ((request_offset + len) <= segment_offset)
    {
        return 0U;
    }
    if (request_offset >= (segment_offset + segment_len))
    {
        return 0U;
    }

    start = (request_offset > segment_offset) ?
            (request_offset - segment_offset) : 0U;
    copy_len = segment_len - start;
    if (copy_len > len)
    {
        copy_len = len;
    }
    if ((segment_offset + start) > request_offset)
    {
        uint32_t skip = (segment_offset + start) - request_offset;

        if (skip >= len)
        {
            return 0U;
        }
        dst += skip;
        len -= skip;
        if (copy_len > len)
        {
            copy_len = len;
        }
    }

    memcpy(dst, segment + start, copy_len);
    return copy_len;
}

static bool fe_raw_payload_copy(uint32_t payload_offset,
                                uint8_t *dst,
                                uint32_t len)
{
    zy100_fe_raw_section_header_t section;
    uint32_t cursor = 0U;
    uint32_t copied_total = 0U;
    uint32_t idx;

    if ((dst == NULL) || (len == 0U) ||
        ((payload_offset + len) > s_fe_raw_store.payload_bytes))
    {
        return false;
    }

    if (s_fe_raw_store.lf_count != 0U)
    {
        fe_raw_fill_section_header(
            &section,
            (uint16_t)ZY100_FE_RAW_SECTION_LF_PRE_800HZ,
            ZY100_FINAL_EDGE_LIVE_UI_HZ,
            ZY100_FINAL_EDGE_LF_PRE_PACKET_BYTES,
            s_fe_raw_store.lf_count,
            s_fe_raw_store.lf_entries[0].tmst_raw,
            s_fe_raw_store.lf_entries[s_fe_raw_store.lf_count - 1U].tmst_raw,
            s_fe_raw_store.lf_count *
                ZY100_FINAL_EDGE_LF_PRE_PACKET_BYTES,
            0U);
        copied_total += fe_raw_copy_segment(
            payload_offset,
            dst,
            len,
            cursor,
            (const uint8_t *)&section,
            sizeof(section));
        cursor += sizeof(section);

        for (idx = 0U; idx < s_fe_raw_store.lf_count; idx++)
        {
            copied_total += fe_raw_copy_segment(
                payload_offset,
                dst,
                len,
                cursor,
                s_fe_raw_store.lf_entries[idx].packet,
                ZY100_FINAL_EDGE_LF_PRE_PACKET_BYTES);
            cursor += ZY100_FINAL_EDGE_LF_PRE_PACKET_BYTES;
        }
    }

    fe_raw_fill_section_header(
        &section,
        (uint16_t)ZY100_FE_RAW_SECTION_HF_HIT_6400HZ,
        ZY100_FINAL_EDGE_OIS_HZ,
        ZY100_FINAL_EDGE_OIS_FRAME_BYTES,
        s_fe_raw_store.view.hf_frames,
        s_fe_raw_store.view.first_tmst_raw,
        s_fe_raw_store.view.last_tmst_raw,
        s_fe_raw_store.view.hf_frames * ZY100_FINAL_EDGE_OIS_FRAME_BYTES,
        0U);
    copied_total += fe_raw_copy_segment(
        payload_offset,
        dst,
        len,
        cursor,
        (const uint8_t *)&section,
        sizeof(section));
    cursor += sizeof(section);

    for (idx = 0U; idx < s_fe_raw_store.view.hf_frames; idx++)
    {
        const uint8_t *frame = NULL;

        if (!zy100_final_edge_bscan_hit_window_get_frame(
                s_fe_raw_store.raw_ring,
                s_fe_raw_store.raw_ring_bytes,
                &s_fe_raw_store.view,
                idx,
                &frame) ||
            (frame == NULL))
        {
            return false;
        }
        copied_total += fe_raw_copy_segment(
            payload_offset,
            dst,
            len,
            cursor,
            frame,
            ZY100_FINAL_EDGE_OIS_FRAME_BYTES);
        cursor += ZY100_FINAL_EDGE_OIS_FRAME_BYTES;
    }

    return copied_total == len;
}

static bool fe_raw_build_payload_page(void)
{
    uint32_t payload_offset =
        s_fe_raw_store.payload_page_next * ZY100_FE_RAW_PAGE_BYTES;
    uint32_t remaining = s_fe_raw_store.payload_bytes - payload_offset;
    uint32_t copy_len = (remaining > ZY100_FE_RAW_PAGE_BYTES) ?
                        ZY100_FE_RAW_PAGE_BYTES : remaining;

    memset(s_fe_raw_store.page, 0xFF, sizeof(s_fe_raw_store.page));
    if ((copy_len != 0U) &&
        !fe_raw_payload_copy(payload_offset, s_fe_raw_store.page, copy_len))
    {
        s_fe_raw_store.stats.raw_hwin_fail++;
        fe_raw_note_error(ZY100_FE_RAW_ERR_STAGE_PAYLOAD_COPY,
                          0U,
                          payload_offset);
        s_fe_raw_store.error = true;
        return false;
    }
    s_fe_raw_store.payload_xor =
        fe_raw_xor_bytes(s_fe_raw_store.payload_xor,
                         s_fe_raw_store.page,
                         copy_len);
    return true;
}

static void fe_raw_build_header_page(void)
{
    zy100_fe_high_raw_header_t header;

    memset(s_fe_raw_store.page, 0xFF, sizeof(s_fe_raw_store.page));
    fe_raw_build_committed_header(&header);
    memcpy(s_fe_raw_store.page, &header, sizeof(header));
}

static bool fe_raw_verify_current_page(void)
{
#if ZY100_FINAL_EDGE_RAW_STORE_VERIFY_ENABLE
    uint8_t verify_buf[ZY100_FE_RAW_VERIFY_CHUNK_BYTES];
    uint32_t verify_start_us = (uint32_t)imu_bsp_local_timestamp_us();
    uint32_t offset;
    imu_status_t status;

    if (!fe_raw_flash_prepare(ZY100_FE_RAW_ERR_STAGE_VERIFY_READ,
                              s_fe_raw_store.in_flight_addr))
    {
        return false;
    }
    for (offset = 0U; offset < ZY100_FE_RAW_PAGE_BYTES;
         offset += ZY100_FE_RAW_VERIFY_CHUNK_BYTES)
    {
        status = gd25q32e_read(s_fe_raw_store.in_flight_addr + offset,
                               verify_buf,
                               ZY100_FE_RAW_VERIFY_CHUNK_BYTES);
        if (status != IMU_STATUS_OK)
        {
            s_fe_raw_store.stats.verify_error++;
            fe_raw_note_error(ZY100_FE_RAW_ERR_STAGE_VERIFY_READ,
                              (uint32_t)status,
                              s_fe_raw_store.in_flight_addr + offset);
            return false;
        }
        if (memcmp(verify_buf,
                   &s_fe_raw_store.page[offset],
                   ZY100_FE_RAW_VERIFY_CHUNK_BYTES) != 0)
        {
            s_fe_raw_store.stats.verify_error++;
            fe_raw_note_error_detail(
                ZY100_FE_RAW_ERR_STAGE_VERIFY_DATA,
                offset,
                s_fe_raw_store.in_flight_addr + offset,
                zy100_get_u32_le(&s_fe_raw_store.page[offset]),
                zy100_get_u32_le(verify_buf));
            return false;
        }
    }

    s_fe_raw_store.stats.verify_page_count++;
    fe_raw_max_u32(&s_fe_raw_store.stats.verify_max_us,
                   fe_raw_elapsed_us(verify_start_us));
#endif
    return true;
}

static void fe_raw_record_finish_ok(void)
{
    uint32_t save_ms = 0U;

    if ((s_fe_raw_store.target_provider != NULL) &&
        ((s_fe_raw_store.target_provider->commit == NULL) ||
         !s_fe_raw_store.target_provider->commit(
             s_fe_raw_store.target_provider->context,
             s_fe_raw_store.target_token)))
    {
        s_fe_raw_store.stats.raw_failed++;
        s_fe_raw_store.error = true;
    }

    if (s_fe_raw_store.record_start_valid)
    {
        save_ms = fe_raw_elapsed_us(s_fe_raw_store.record_start_us) / 1000U;
    }

    s_fe_raw_store.stats.raw_saved++;
    s_fe_raw_store.stats.raw_record_save_count++;
    fe_raw_add_u32_sat(&s_fe_raw_store.stats.raw_record_save_total_ms,
                       save_ms);
    fe_raw_max_u32(&s_fe_raw_store.stats.raw_record_save_max_ms, save_ms);
    s_fe_raw_store.stats.raw_record_save_last_ms = save_ms;
    s_fe_raw_store.stats.last_raw_id = s_fe_raw_store.current_raw_id;
    s_fe_raw_store.stats.last_bscan_id = s_fe_raw_store.view.bscan_id;
    s_fe_raw_store.stats.last_sections = s_fe_raw_store.section_count;
    s_fe_raw_store.stats.last_hf_frames = s_fe_raw_store.view.hf_frames;
    s_fe_raw_store.stats.last_lf_packets = s_fe_raw_store.lf_count;
    s_fe_raw_store.stats.last_payload_bytes = s_fe_raw_store.payload_bytes;
    s_fe_raw_store.stats.last_page_count = s_fe_raw_store.payload_pages + 1U;
    s_fe_raw_store.stats.last_flags = s_fe_raw_store.flags;
    if (s_fe_raw_store.target_provider != NULL)
    {
        s_fe_raw_store.stats.used_bytes +=
            (s_fe_raw_store.payload_pages + 1U) * ZY100_FE_RAW_PAGE_BYTES;
    }
    else
    {
        s_fe_raw_store.stats.used_bytes =
            s_fe_raw_store.stats.next_raw_id * ZY100_FINAL_EDGE_RAW_BUCKET_BYTES;
    }
    s_fe_raw_store.pending = false;
    s_fe_raw_store.record_active = false;
    s_fe_raw_store.program_in_flight = false;
    s_fe_raw_store.state = ZY100_FE_RAW_STATE_IDLE;
    s_fe_raw_store.gate_start_ms = 0U;
    s_fe_raw_store.record_start_valid = false;
    s_fe_raw_store.record_start_us = 0U;
    s_fe_raw_store.target_wait_start_us = 0U;
    s_fe_raw_store.target_token = 0U;
}

static void fe_raw_record_fail(void)
{
    if ((s_fe_raw_store.target_provider != NULL) &&
        (s_fe_raw_store.target_provider->abort != NULL) &&
        (s_fe_raw_store.target_token != 0U))
    {
        s_fe_raw_store.target_provider->abort(
            s_fe_raw_store.target_provider->context,
            s_fe_raw_store.target_token);
    }
    s_fe_raw_store.stats.raw_failed++;
    s_fe_raw_store.pending = false;
    s_fe_raw_store.record_active = false;
    s_fe_raw_store.program_in_flight = false;
    s_fe_raw_store.state = ZY100_FE_RAW_STATE_IDLE;
    s_fe_raw_store.error = true;
    s_fe_raw_store.gate_start_ms = 0U;
    s_fe_raw_store.record_start_valid = false;
    s_fe_raw_store.record_start_us = 0U;
    s_fe_raw_store.target_wait_start_us = 0U;
}

static bool fe_raw_issue_page(uint32_t addr, bool is_header)
{
    uint32_t issue_start_us = (uint32_t)imu_bsp_local_timestamp_us();
    imu_status_t status;

    if ((s_fe_raw_store.target_provider != NULL) &&
        ((addr < ZY100_ONLINE_SPOOL_REGION_BASE_ADDR) ||
         ((addr + ZY100_FE_RAW_PAGE_BYTES) >
          (ZY100_ONLINE_SPOOL_REGION_BASE_ADDR +
           ZY100_ONLINE_SPOOL_REGION_BYTES)) ||
         (addr < s_fe_raw_store.current_bucket_addr) ||
         ((addr + ZY100_FE_RAW_PAGE_BYTES) >
          (s_fe_raw_store.current_bucket_addr + fe_raw_target_record_bytes()))))
    {
        s_fe_raw_store.stats.online_offline_region_write_violation++;
        s_fe_raw_online_address_violations++;
        fe_raw_record_fail();
        return false;
    }
    if (!fe_raw_flash_prepare(is_header ?
                              ZY100_FE_RAW_ERR_STAGE_HEADER_WRITE :
                              ZY100_FE_RAW_ERR_STAGE_PAYLOAD_WRITE,
                              addr))
    {
        fe_raw_record_fail();
        return false;
    }
    status = gd25q32e_page_program(addr,
                                   s_fe_raw_store.page,
                                   ZY100_FE_RAW_PAGE_BYTES);
    fe_raw_max_u32(&s_fe_raw_store.stats.page_issue_max_us,
                   fe_raw_elapsed_us(issue_start_us));
    if (status != IMU_STATUS_OK)
    {
        s_fe_raw_store.stats.write_error++;
        fe_raw_note_error(is_header ?
                          ZY100_FE_RAW_ERR_STAGE_HEADER_WRITE :
                          ZY100_FE_RAW_ERR_STAGE_PAYLOAD_WRITE,
                          (uint32_t)status,
                          addr);
        fe_raw_record_fail();
        return false;
    }

    s_fe_raw_store.program_in_flight = true;
    s_fe_raw_store.in_flight_addr = addr;
    s_fe_raw_store.in_flight_header = is_header;
    s_fe_raw_store.in_flight_page_index = is_header ?
        s_fe_raw_store.payload_pages : s_fe_raw_store.payload_page_next;
    s_fe_raw_store.stats.page_program++;
    if (s_fe_raw_store.target_provider != NULL)
    {
        s_fe_raw_store.stats.online_target_page_program++;
    }
    return true;
}

static bool fe_raw_finish_in_flight_page(void)
{
    if (!fe_raw_verify_current_page())
    {
        fe_raw_record_fail();
        return false;
    }
    if (s_fe_raw_store.target_provider != NULL)
    {
        s_fe_raw_online_verified_pages++;
    }

    if (s_fe_raw_store.in_flight_header)
    {
        fe_raw_record_finish_ok();
        return true;
    }

    s_fe_raw_store.payload_page_next++;
    if (s_fe_raw_store.payload_page_next >= s_fe_raw_store.payload_pages)
    {
        s_fe_raw_store.state = ZY100_FE_RAW_STATE_HEADER;
    }
    return true;
}

static bool fe_raw_prepare_issue_next_sector(void)
{
    uint32_t addr = s_fe_raw_store.prep_base_addr +
                    (s_fe_raw_store.prep_done_sectors *
                     ZY100_FE_RAW_SECTOR_BYTES);
    imu_status_t status;

    if (!fe_raw_flash_prepare(ZY100_FE_RAW_ERR_STAGE_PAYLOAD_WRITE, addr))
    {
        s_fe_raw_store.prep_state = ZY100_FE_RAW_PREP_ERROR;
        return false;
    }
    status = gd25q32e_sector_erase_4k(addr);
    if (status != IMU_STATUS_OK)
    {
        s_fe_raw_store.prep_state = ZY100_FE_RAW_PREP_ERROR;
        s_fe_raw_store.error = true;
        return false;
    }

    s_fe_raw_store.prep_erase_in_flight = true;
    s_fe_raw_store.stats.prepare_erase_count++;
    s_fe_raw_store.stats.erase_count++;
    return true;
}

static void fe_raw_log_prepare(uint32_t err)
{
#if ZY100_LOG_FE_DIAG_VERBOSE
    DBG_DIRECT("[FE_RAW_PREP] erase=%u total=%u done=%u err=%u",
               s_fe_raw_store.stats.prepare_erase_count,
               s_fe_raw_store.prep_total_sectors,
               s_fe_raw_store.prep_done_sectors,
               err);
#else
    (void)err;
#endif
}

static bool fe_raw_is_record_idle(void)
{
    return (!s_fe_raw_store.pending) &&
           (!s_fe_raw_store.program_in_flight) &&
           (s_fe_raw_store.state == ZY100_FE_RAW_STATE_IDLE) &&
           (!s_fe_raw_store.record_active) &&
           (!s_fe_raw_store.finalizing);
}

static bool fe_raw_page_is_erased(const uint8_t page[ZY100_FE_RAW_PAGE_BYTES])
{
    uint32_t idx;

    if (page == NULL)
    {
        return false;
    }
    for (idx = 0U; idx < ZY100_FE_RAW_PAGE_BYTES; idx++)
    {
        if (page[idx] != 0xFFU)
        {
            return false;
        }
    }
    return true;
}

static bool fe_raw_sector_erased(uint32_t sector_addr)
{
    uint8_t page[ZY100_FE_RAW_PAGE_BYTES];
    uint32_t offset;

    if ((sector_addr % ZY100_FE_RAW_SECTOR_BYTES) != 0U)
    {
        return false;
    }
    if (!fe_raw_flash_prepare(ZY100_FE_RAW_ERR_STAGE_VERIFY_READ,
                              sector_addr))
    {
        return false;
    }
    for (offset = 0U; offset < ZY100_FE_RAW_SECTOR_BYTES;
         offset += ZY100_FE_RAW_PAGE_BYTES)
    {
        if (gd25q32e_read(sector_addr + offset,
                          page,
                          ZY100_FE_RAW_PAGE_BYTES) != IMU_STATUS_OK)
        {
            s_fe_raw_store.stats.verify_error++;
            fe_raw_note_error(ZY100_FE_RAW_ERR_STAGE_VERIFY_READ,
                              0U,
                              sector_addr + offset);
            s_fe_raw_store.error = true;
            return false;
        }
        if (!fe_raw_page_is_erased(page))
        {
            return false;
        }
    }
    return true;
}

static bool fe_raw_erase_append_bucket(uint32_t bucket_addr)
{
    uint32_t addr;

    if (!s_fe_raw_store.append_erase_dynamic)
    {
        return true;
    }
    if (s_fe_raw_store.append_erased_bucket_valid &&
        (s_fe_raw_store.append_erased_bucket_addr == bucket_addr))
    {
        return true;
    }
#if ZY100_LOG_FE_DIAG_VERBOSE
    DBG_DIRECT("[ERASE_AHEAD] region=raw addr=0x%08lX sectors=%u reason=append_tail",
               (unsigned long)bucket_addr,
               (uint32_t)(ZY100_FINAL_EDGE_RAW_BUCKET_BYTES /
                          ZY100_FE_RAW_SECTOR_BYTES));
#endif
    for (addr = bucket_addr;
         addr < (bucket_addr + ZY100_FINAL_EDGE_RAW_BUCKET_BYTES);
         addr += ZY100_FE_RAW_SECTOR_BYTES)
    {
        if (fe_raw_sector_erased(addr))
        {
            continue;
        }
        if (s_fe_raw_store.error)
        {
            return false;
        }
        if (!fe_raw_flash_prepare(ZY100_FE_RAW_ERR_STAGE_PAYLOAD_WRITE, addr))
        {
            return false;
        }
        if (gd25q32e_sector_erase_4k(addr) != IMU_STATUS_OK)
        {
            s_fe_raw_store.stats.write_error++;
            fe_raw_note_error(ZY100_FE_RAW_ERR_STAGE_PAYLOAD_WRITE, 0U, addr);
            s_fe_raw_store.error = true;
            return false;
        }
        if (gd25q32e_wait_while_busy(GD25Q32E_SECTOR_ERASE_TIMEOUT_MS) !=
            IMU_STATUS_OK)
        {
            s_fe_raw_store.stats.write_error++;
            fe_raw_note_error(ZY100_FE_RAW_ERR_STAGE_BUSY_QUERY, 0U, addr);
            s_fe_raw_store.error = true;
            return false;
        }
        if (!fe_raw_sector_erased(addr))
        {
            s_fe_raw_store.stats.verify_error++;
            s_fe_raw_store.error = true;
            return false;
        }
        s_fe_raw_store.stats.runtime_erase_count++;
        s_fe_raw_store.stats.erase_count++;
    }
    s_fe_raw_store.append_erased_bucket_valid = true;
    s_fe_raw_store.append_erased_bucket_addr = bucket_addr;
    return true;
}

bool zy100_final_edge_raw_store_prepare_erase_range_begin(
    uint32_t round,
    uint32_t raw_first_bucket,
    uint32_t raw_limit_bucket)
{
    gd25q32e_jedec_id_t id;
    imu_status_t status;
    uint32_t raw_total =
        ZY100_FINAL_EDGE_RAW_REGION_BYTES / ZY100_FINAL_EDGE_RAW_BUCKET_BYTES;

    if (!fe_raw_is_record_idle())
    {
        return false;
    }
    if ((raw_first_bucket >= raw_limit_bucket) ||
        (raw_limit_bucket > raw_total))
    {
        return false;
    }

    memset(&s_fe_raw_store, 0, sizeof(s_fe_raw_store));
    s_fe_raw_store.prep_round = round;
    s_fe_raw_store.prep_base_addr =
        ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR +
        (raw_first_bucket * ZY100_FINAL_EDGE_RAW_BUCKET_BYTES);
    s_fe_raw_store.prep_total_sectors =
        ((raw_limit_bucket - raw_first_bucket) *
         ZY100_FINAL_EDGE_RAW_BUCKET_BYTES) / ZY100_FE_RAW_SECTOR_BYTES;
    s_fe_raw_store.session_raw_first_bucket = raw_first_bucket;
    s_fe_raw_store.session_raw_limit_bucket = raw_limit_bucket;
    s_fe_raw_store.bucket_count = raw_limit_bucket - raw_first_bucket;
    s_fe_raw_store.stats.bucket_count = s_fe_raw_store.bucket_count;
    s_fe_raw_store.stats.session_raw_first_bucket = raw_first_bucket;
    s_fe_raw_store.stats.session_raw_limit_bucket = raw_limit_bucket;

    status = gd25q32e_init();
    if (status != IMU_STATUS_OK)
    {
        s_fe_raw_store.prep_state = ZY100_FE_RAW_PREP_ERROR;
        s_fe_raw_store.error = true;
        fe_raw_log_prepare(1U);
        return false;
    }

    memset(&id, 0, sizeof(id));
    status = gd25q32e_read_jedec_id(&id);
    if ((status != IMU_STATUS_OK) || !gd25q32e_jedec_is_4mbyte(&id))
    {
        s_fe_raw_store.prep_state = ZY100_FE_RAW_PREP_ERROR;
        s_fe_raw_store.error = true;
        fe_raw_log_prepare(1U);
        return false;
    }

    s_fe_raw_store.prep_state = ZY100_FE_RAW_PREP_ERASE;
    fe_raw_log_prepare(0U);
    return true;
}

bool zy100_final_edge_raw_store_prepare_append_begin(
    uint32_t round,
    uint32_t raw_data_begin_addr,
    uint32_t raw_region_end_addr)
{
    gd25q32e_jedec_id_t id;
    imu_status_t status;
    uint32_t raw_first_bucket;
    uint32_t raw_limit_bucket;

    if (!fe_raw_is_record_idle())
    {
        return false;
    }
    if (ZY100_FE_RAW_ADDR_BEFORE_BASE(raw_data_begin_addr) ||
        (raw_region_end_addr >
         (ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR +
          ZY100_FINAL_EDGE_RAW_REGION_BYTES)) ||
        (raw_data_begin_addr >= raw_region_end_addr) ||
        ((raw_data_begin_addr % ZY100_FINAL_EDGE_RAW_BUCKET_BYTES) != 0U) ||
        ((raw_region_end_addr % ZY100_FINAL_EDGE_RAW_BUCKET_BYTES) != 0U))
    {
        return false;
    }

    raw_first_bucket =
        (raw_data_begin_addr - ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR) /
        ZY100_FINAL_EDGE_RAW_BUCKET_BYTES;
    raw_limit_bucket =
        (raw_region_end_addr - ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR) /
        ZY100_FINAL_EDGE_RAW_BUCKET_BYTES;
    if (raw_first_bucket >= raw_limit_bucket)
    {
        return false;
    }

    memset(&s_fe_raw_store, 0, sizeof(s_fe_raw_store));
    s_fe_raw_store.prep_round = round;
    s_fe_raw_store.prep_base_addr = raw_data_begin_addr;
    s_fe_raw_store.prep_total_sectors = 0U;
    s_fe_raw_store.prep_done_sectors = 0U;
    s_fe_raw_store.session_raw_first_bucket = raw_first_bucket;
    s_fe_raw_store.session_raw_limit_bucket = raw_limit_bucket;
    s_fe_raw_store.bucket_count = raw_limit_bucket - raw_first_bucket;
    s_fe_raw_store.append_erase_dynamic = true;
    s_fe_raw_store.prepared = true;
    s_fe_raw_store.prep_state = ZY100_FE_RAW_PREP_DONE;
    s_fe_raw_store.stats.prepared = 1U;
    s_fe_raw_store.stats.bucket_count = s_fe_raw_store.bucket_count;
    s_fe_raw_store.stats.session_raw_first_bucket = raw_first_bucket;
    s_fe_raw_store.stats.session_raw_limit_bucket = raw_limit_bucket;

    status = gd25q32e_init();
    if (status != IMU_STATUS_OK)
    {
        s_fe_raw_store.prep_state = ZY100_FE_RAW_PREP_ERROR;
        s_fe_raw_store.error = true;
        return false;
    }

    memset(&id, 0, sizeof(id));
    status = gd25q32e_read_jedec_id(&id);
    if ((status != IMU_STATUS_OK) || !gd25q32e_jedec_is_4mbyte(&id))
    {
        s_fe_raw_store.prep_state = ZY100_FE_RAW_PREP_ERROR;
        s_fe_raw_store.error = true;
        return false;
    }

#if ZY100_LOG_FE_DIAG_VERBOSE
    DBG_DIRECT("[FE_RAW_PREP] append_tail raw=0x%08lX..0x%08lX erase=dynamic",
               (unsigned long)raw_data_begin_addr,
               (unsigned long)raw_region_end_addr);
#endif
    return true;
}

bool zy100_final_edge_raw_store_prepare_erase_begin(uint32_t round)
{
    return zy100_final_edge_raw_store_prepare_erase_range_begin(
        round,
        0U,
        ZY100_FINAL_EDGE_RAW_REGION_BYTES /
        ZY100_FINAL_EDGE_RAW_BUCKET_BYTES);
}

bool zy100_final_edge_raw_store_prepare_target_begin(
    uint32_t round,
    const zy100_fe_store_target_provider_t *provider)
{
    if (!fe_raw_is_record_idle() || (provider == NULL) ||
        (provider->reserve == NULL) || (provider->commit == NULL) ||
        (provider->abort == NULL))
    {
        return false;
    }
    memset(&s_fe_raw_store, 0, sizeof(s_fe_raw_store));
    s_fe_raw_store.prepared = true;
    s_fe_raw_store.prep_state = ZY100_FE_RAW_PREP_DONE;
    s_fe_raw_store.prep_round = round;
    s_fe_raw_store.bucket_count =
        ZY100_FINAL_EDGE_RAW_REGION_BYTES / ZY100_FINAL_EDGE_RAW_BUCKET_BYTES;
    s_fe_raw_store.target_provider = provider;
    s_fe_raw_store.stats.prepared = 1U;
    s_fe_raw_store.stats.bucket_count = s_fe_raw_store.bucket_count;
    return true;
}

zy100_flash_prepare_status_t zy100_final_edge_raw_store_prepare_erase_poll(void)
{
    bool busy = false;

    if (s_fe_raw_store.prep_state == ZY100_FE_RAW_PREP_DONE)
    {
        return ZY100_FLASH_PREP_DONE;
    }
    if (s_fe_raw_store.prep_state != ZY100_FE_RAW_PREP_ERASE)
    {
        return ZY100_FLASH_PREP_ERROR;
    }

    if (gd25q32e_is_busy(&busy) != IMU_STATUS_OK)
    {
        s_fe_raw_store.prep_state = ZY100_FE_RAW_PREP_ERROR;
        s_fe_raw_store.error = true;
        fe_raw_log_prepare(1U);
        return ZY100_FLASH_PREP_ERROR;
    }
    if (busy)
    {
        return ZY100_FLASH_PREP_BUSY;
    }

    if (s_fe_raw_store.prep_erase_in_flight)
    {
        s_fe_raw_store.prep_erase_in_flight = false;
        s_fe_raw_store.prep_done_sectors++;
        if (((s_fe_raw_store.prep_done_sectors %
              ZY100_FE_RAW_PREP_LOG_STEP) == 0U) &&
            (s_fe_raw_store.prep_done_sectors <
             s_fe_raw_store.prep_total_sectors))
        {
            fe_raw_log_prepare(0U);
        }
    }

    if (s_fe_raw_store.prep_done_sectors >=
        s_fe_raw_store.prep_total_sectors)
    {
        s_fe_raw_store.prepared = true;
        s_fe_raw_store.prep_state = ZY100_FE_RAW_PREP_DONE;
        s_fe_raw_store.stats.prepared = 1U;
        fe_raw_log_prepare(0U);
        return ZY100_FLASH_PREP_DONE;
    }

    if (!fe_raw_prepare_issue_next_sector())
    {
        fe_raw_log_prepare(1U);
        return ZY100_FLASH_PREP_ERROR;
    }

    return ZY100_FLASH_PREP_BUSY;
}

void zy100_final_edge_raw_store_reset_runtime(void)
{
    bool prepared = s_fe_raw_store.prepared;
    zy100_fe_raw_prep_state_t prep_state = s_fe_raw_store.prep_state;
    uint32_t prep_round = s_fe_raw_store.prep_round;
    uint32_t prep_base_addr = s_fe_raw_store.prep_base_addr;
    uint32_t prep_total = s_fe_raw_store.prep_total_sectors;
    uint32_t prep_done = s_fe_raw_store.prep_done_sectors;
    uint32_t bucket_count = s_fe_raw_store.bucket_count;
    uint32_t session_raw_first_bucket =
        s_fe_raw_store.session_raw_first_bucket;
    uint32_t session_raw_limit_bucket =
        s_fe_raw_store.session_raw_limit_bucket;
    bool append_erase_dynamic = s_fe_raw_store.append_erase_dynamic;
    bool append_erased_bucket_valid =
        s_fe_raw_store.append_erased_bucket_valid;
    uint32_t append_erased_bucket_addr =
        s_fe_raw_store.append_erased_bucket_addr;
    uint32_t prepare_erase_count = s_fe_raw_store.stats.prepare_erase_count;
    uint32_t runtime_erase_count = s_fe_raw_store.stats.runtime_erase_count;
    uint32_t erase_count = s_fe_raw_store.stats.erase_count;
    const zy100_fe_store_target_provider_t *target_provider =
        s_fe_raw_store.target_provider;

    memset(&s_fe_raw_store, 0, sizeof(s_fe_raw_store));
    s_fe_raw_store.prepared = prepared;
    s_fe_raw_store.prep_state = prep_state;
    s_fe_raw_store.prep_round = prep_round;
    s_fe_raw_store.prep_base_addr = prep_base_addr;
    s_fe_raw_store.prep_total_sectors = prep_total;
    s_fe_raw_store.prep_done_sectors = prep_done;
    s_fe_raw_store.bucket_count = bucket_count;
    s_fe_raw_store.session_raw_first_bucket = session_raw_first_bucket;
    s_fe_raw_store.session_raw_limit_bucket = session_raw_limit_bucket;
    s_fe_raw_store.append_erase_dynamic = append_erase_dynamic;
    s_fe_raw_store.append_erased_bucket_valid = append_erased_bucket_valid;
    s_fe_raw_store.append_erased_bucket_addr = append_erased_bucket_addr;
    s_fe_raw_store.stats.prepared = prepared ? 1U : 0U;
    s_fe_raw_store.stats.prepare_erase_count = prepare_erase_count;
    s_fe_raw_store.stats.runtime_erase_count = runtime_erase_count;
    s_fe_raw_store.stats.erase_count = erase_count;
    s_fe_raw_store.stats.bucket_count = bucket_count;
    s_fe_raw_store.stats.session_raw_first_bucket = session_raw_first_bucket;
    s_fe_raw_store.stats.session_raw_limit_bucket = session_raw_limit_bucket;
    s_fe_raw_store.target_provider = target_provider;
}

void zy100_final_edge_raw_store_reset_after_clear(uint32_t round)
{
    memset(&s_fe_raw_store, 0, sizeof(s_fe_raw_store));
    s_fe_raw_store.prepared = true;
    s_fe_raw_store.prep_state = ZY100_FE_RAW_PREP_DONE;
    s_fe_raw_store.prep_round = round;
    s_fe_raw_store.prep_base_addr = ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR;
    s_fe_raw_store.prep_total_sectors =
        ZY100_FINAL_EDGE_RAW_REGION_BYTES / ZY100_FE_RAW_SECTOR_BYTES;
    s_fe_raw_store.prep_done_sectors = s_fe_raw_store.prep_total_sectors;
    s_fe_raw_store.session_raw_first_bucket = 0U;
    s_fe_raw_store.session_raw_limit_bucket =
        ZY100_FINAL_EDGE_RAW_REGION_BYTES / ZY100_FINAL_EDGE_RAW_BUCKET_BYTES;
    s_fe_raw_store.bucket_count =
        ZY100_FINAL_EDGE_RAW_REGION_BYTES / ZY100_FINAL_EDGE_RAW_BUCKET_BYTES;
    s_fe_raw_store.stats.prepared = 1U;
    s_fe_raw_store.stats.bucket_count = s_fe_raw_store.bucket_count;
    s_fe_raw_store.stats.session_raw_first_bucket =
        s_fe_raw_store.session_raw_first_bucket;
    s_fe_raw_store.stats.session_raw_limit_bucket =
        s_fe_raw_store.session_raw_limit_bucket;
}

void zy100_final_edge_raw_store_abort_pending(void)
{
    if (!zy100_final_edge_raw_store_has_pending_work())
    {
        return;
    }
    if (s_fe_raw_store.program_in_flight)
    {
        (void)gd25q32e_wait_while_busy(GD25Q32E_PAGE_PROGRAM_TIMEOUT_MS);
    }
    if ((s_fe_raw_store.target_provider != NULL) &&
        (s_fe_raw_store.target_provider->abort != NULL) &&
        (s_fe_raw_store.target_token != 0U))
    {
        s_fe_raw_store.target_provider->abort(
            s_fe_raw_store.target_provider->context,
            s_fe_raw_store.target_token);
    }
    s_fe_raw_store.stats.raw_failed++;
    s_fe_raw_store.pending = false;
    s_fe_raw_store.record_active = false;
    s_fe_raw_store.program_in_flight = false;
    s_fe_raw_store.state = ZY100_FE_RAW_STATE_IDLE;
    s_fe_raw_store.gate_start_ms = 0U;
    s_fe_raw_store.record_start_valid = false;
    s_fe_raw_store.record_start_us = 0U;
    s_fe_raw_store.target_wait_start_us = 0U;
    s_fe_raw_store.target_token = 0U;
}

void zy100_final_edge_raw_store_set_target_provider(
    const zy100_fe_store_target_provider_t *provider)
{
    if (fe_raw_is_record_idle())
    {
        s_fe_raw_store.target_provider = provider;
    }
}

bool zy100_final_edge_raw_store_start(uint32_t round)
{
    if (!s_fe_raw_store.prepared ||
        (s_fe_raw_store.prep_state != ZY100_FE_RAW_PREP_DONE) ||
        (s_fe_raw_store.prep_round != round) ||
        !fe_raw_is_record_idle())
    {
        return false;
    }

    s_fe_raw_store.running = true;
    s_fe_raw_store.run_round = round;
    s_fe_raw_store.stats.running = 1U;
    s_fe_raw_store.stats.prepared = 1U;
    s_fe_raw_store.stats.bucket_count = s_fe_raw_store.bucket_count;
    s_fe_raw_store.stats.session_raw_first_bucket =
        s_fe_raw_store.session_raw_first_bucket;
    s_fe_raw_store.stats.session_raw_limit_bucket =
        s_fe_raw_store.session_raw_limit_bucket;
    return true;
}

bool zy100_final_edge_raw_store_begin_hit(
    const uint8_t *raw_ring,
    uint32_t raw_ring_bytes,
    const zy100_final_edge_bscan_result_t *result,
    const zy100_final_edge_hit_window_view_t *view,
    uint32_t bscan_id,
    uint32_t rt_trigger_sample,
    uint16_t rt_trigger_tmst_raw,
    bool ois_od_warn)
{
    zy100_final_edge_lf_pre_query_t lf_info;
    zy100_fe_store_reserve_result_t reserve_result;

    if (!s_fe_raw_store.running || !fe_raw_is_record_idle())
    {
        s_fe_raw_store.stats.raw_failed++;
        s_fe_raw_store.error = true;
        return false;
    }
    if ((raw_ring == NULL) || (result == NULL) || (view == NULL) ||
        (result->hit_found == 0U) || (raw_ring_bytes == 0U))
    {
        s_fe_raw_store.stats.raw_failed++;
        s_fe_raw_store.error = true;
        return false;
    }
    if ((view->bscan_id != bscan_id) ||
        (view->valid == 0U) ||
        (view->complete == 0U) ||
        (view->hf_frames == 0U) ||
        (view->hf_frames > ZY100_FINAL_EDGE_HIT_RAW_RING_FRAMES))
    {
        s_fe_raw_store.stats.raw_hwin_fail++;
        s_fe_raw_store.stats.raw_failed++;
        s_fe_raw_store.error = true;
        return false;
    }
    if (result->post_truncated != 0U)
    {
        s_fe_raw_store.stats.raw_hwin_fail++;
        s_fe_raw_store.stats.raw_failed++;
        s_fe_raw_store.error = true;
        return false;
    }
    if ((s_fe_raw_store.target_provider == NULL) &&
        (s_fe_raw_store.stats.next_raw_id >= s_fe_raw_store.bucket_count))
    {
        s_fe_raw_store.stats.raw_full++;
        s_fe_raw_store.stats.raw_failed++;
        s_fe_raw_store.error = true;
        return false;
    }

    memset(&lf_info, 0, sizeof(lf_info));
    s_fe_raw_store.lf_count = 0U;
    if ((view->early_hit != 0U) && (view->lf_pre_packets_needed != 0U))
    {
        if ((view->lf_pre_packets_needed > ZY100_FE_RAW_MAX_LF_PACKETS) ||
            !zy100_final_edge_lf_pre_ring_copy_ending_at(
                rt_trigger_sample,
                view->lf_pre_packets_needed,
                s_fe_raw_store.lf_entries,
                ZY100_FE_RAW_MAX_LF_PACKETS,
                &lf_info) ||
            (lf_info.newest_sample_index != rt_trigger_sample) ||
            (lf_info.newest_tmst_raw != rt_trigger_tmst_raw))
        {
            s_fe_raw_store.stats.raw_lf_fail++;
            s_fe_raw_store.stats.raw_failed++;
            s_fe_raw_store.error = true;
            return false;
        }
        s_fe_raw_store.lf_count = view->lf_pre_packets_needed;
    }

    s_fe_raw_store.raw_ring = raw_ring;
    s_fe_raw_store.raw_ring_bytes = raw_ring_bytes;
    s_fe_raw_store.result = *result;
    s_fe_raw_store.view = *view;
    s_fe_raw_store.rt_trigger_sample = rt_trigger_sample;
    s_fe_raw_store.rt_trigger_tmst_raw = rt_trigger_tmst_raw;
    s_fe_raw_store.current_raw_id = s_fe_raw_store.stats.next_raw_id;
    s_fe_raw_store.flags = ZY100_FE_RAW_FLAG_HF_COMPLETE;
    if (view->early_hit != 0U)
    {
        s_fe_raw_store.flags |= ZY100_FE_RAW_FLAG_EARLY_HIT;
    }
    if (s_fe_raw_store.lf_count != 0U)
    {
        s_fe_raw_store.flags |= ZY100_FE_RAW_FLAG_LF_PRE_PRESENT;
    }
    if (view->wraps != 0U)
    {
        s_fe_raw_store.flags |= ZY100_FE_RAW_FLAG_HWIN_WRAP;
    }
    if (result->use20 != 0U)
    {
        s_fe_raw_store.flags |= ZY100_FE_RAW_FLAG_OIS20_USED;
    }
    if (ois_od_warn)
    {
        s_fe_raw_store.flags |= ZY100_FE_RAW_FLAG_OIS_OD_WARN;
    }
    s_fe_raw_store.section_count = (s_fe_raw_store.lf_count != 0U) ? 2U : 1U;
    s_fe_raw_store.payload_bytes =
        sizeof(zy100_fe_raw_section_header_t) +
        (view->hf_frames * ZY100_FINAL_EDGE_OIS_FRAME_BYTES);
    if (s_fe_raw_store.lf_count != 0U)
    {
        s_fe_raw_store.payload_bytes +=
            sizeof(zy100_fe_raw_section_header_t) +
            (s_fe_raw_store.lf_count * ZY100_FINAL_EDGE_LF_PRE_PACKET_BYTES);
    }
    if ((ZY100_FE_RAW_PAYLOAD_OFFSET + s_fe_raw_store.payload_bytes) >
        ZY100_FINAL_EDGE_RAW_BUCKET_BYTES)
    {
        s_fe_raw_store.stats.raw_failed++;
        s_fe_raw_store.error = true;
        return false;
    }

    s_fe_raw_store.payload_pages =
        (s_fe_raw_store.payload_bytes + ZY100_FE_RAW_PAGE_BYTES - 1U) /
        ZY100_FE_RAW_PAGE_BYTES;
    if (s_fe_raw_store.target_provider != NULL)
    {
        reserve_result = fe_raw_try_reserve_target();
        if (reserve_result == ZY100_FE_STORE_RESERVE_ERROR)
        {
            s_fe_raw_store.stats.raw_failed++;
            s_fe_raw_store.error = true;
            return false;
        }
    }
    else
    {
        s_fe_raw_store.current_bucket_addr =
            ZY100_FINAL_EDGE_RAW_REGION_BASE_ADDR +
            ((s_fe_raw_store.session_raw_first_bucket +
              s_fe_raw_store.current_raw_id) *
             ZY100_FINAL_EDGE_RAW_BUCKET_BYTES);
        if (!fe_raw_erase_append_bucket(s_fe_raw_store.current_bucket_addr))
        {
            s_fe_raw_store.stats.raw_failed++;
            return false;
        }
    }
    s_fe_raw_store.payload_page_next = 0U;
    s_fe_raw_store.payload_xor = 0U;
    s_fe_raw_store.pending = true;
    s_fe_raw_store.record_active = true;
    s_fe_raw_store.record_start_valid = true;
    s_fe_raw_store.record_start_us = (uint32_t)imu_bsp_local_timestamp_us();
    if (s_fe_raw_store.target_provider == NULL)
    {
        s_fe_raw_store.state = ZY100_FE_RAW_STATE_PAYLOAD;
    }
    s_fe_raw_store.stats.raw_begin++;
    s_fe_raw_store.stats.next_raw_id++;
    if (s_fe_raw_store.target_provider == NULL)
    {
        s_fe_raw_store.stats.used_bytes =
            s_fe_raw_store.stats.next_raw_id * ZY100_FINAL_EDGE_RAW_BUCKET_BYTES;
    }
    s_fe_raw_store.stats.last_bscan_id = bscan_id;
    s_fe_raw_store.stats.last_raw_id = s_fe_raw_store.current_raw_id;
    s_fe_raw_store.stats.last_sections = s_fe_raw_store.section_count;
    s_fe_raw_store.stats.last_hf_frames = view->hf_frames;
    s_fe_raw_store.stats.last_lf_packets = s_fe_raw_store.lf_count;
    s_fe_raw_store.stats.last_payload_bytes = s_fe_raw_store.payload_bytes;
    s_fe_raw_store.stats.last_page_count = s_fe_raw_store.payload_pages + 1U;
    s_fe_raw_store.stats.last_flags = s_fe_raw_store.flags;
    return true;
}

bool zy100_final_edge_raw_store_waiting_target(void)
{
    return fe_raw_waiting_target();
}

bool zy100_final_edge_raw_store_waiting_target_space(void)
{
    return s_fe_raw_store.state == ZY100_FE_RAW_STATE_WAIT_TARGET_SPACE;
}

zy100_fe_store_reserve_result_t
zy100_final_edge_raw_store_retry_target(void)
{
    zy100_fe_store_reserve_result_t result;

    if (!fe_raw_waiting_target() || !s_fe_raw_store.pending ||
        !s_fe_raw_store.record_active)
    {
        return ZY100_FE_STORE_RESERVE_ERROR;
    }
    s_fe_raw_store.stats.target_retry_count++;
    result = fe_raw_try_reserve_target();
    if (result == ZY100_FE_STORE_RESERVE_OK)
    {
        s_fe_raw_store.stats.target_retry_ok++;
    }
    return result;
}

zy100_fe_raw_pump_result_t zy100_final_edge_raw_store_pump_once_ex(
    zy100_fe_raw_pump_context_t context)
{
    uint32_t pump_start_us = (uint32_t)imu_bsp_local_timestamp_us();
    uint32_t issued = 0U;
    bool busy = false;

    if (s_fe_raw_store.b_active)
    {
        s_fe_raw_store.stats.b_active_pump_blocked++;
        return ZY100_FE_RAW_PUMP_NO_WORK;
    }
    if (!zy100_final_edge_raw_store_has_pending_work())
    {
        return ZY100_FE_RAW_PUMP_NO_WORK;
    }
    if (fe_raw_waiting_target())
    {
        return ZY100_FE_RAW_PUMP_NO_WORK;
    }

    do
    {
        if (!s_fe_raw_store.program_in_flight &&
            !fe_raw_flash_prepare(ZY100_FE_RAW_ERR_STAGE_BUSY_QUERY,
                                  s_fe_raw_store.current_bucket_addr))
        {
            fe_raw_record_fail();
            fe_raw_note_pump_elapsed(context, pump_start_us);
            return ZY100_FE_RAW_PUMP_ERROR;
        }
        if (gd25q32e_is_busy(&busy) != IMU_STATUS_OK)
        {
            s_fe_raw_store.stats.write_error++;
            fe_raw_note_error(ZY100_FE_RAW_ERR_STAGE_BUSY_QUERY,
                              0U,
                              s_fe_raw_store.in_flight_addr);
            fe_raw_record_fail();
            fe_raw_note_pump_elapsed(context, pump_start_us);
            return ZY100_FE_RAW_PUMP_ERROR;
        }
        if (busy)
        {
            s_fe_raw_store.stats.wip_busy++;
            fe_raw_note_pump_elapsed(context, pump_start_us);
            return ZY100_FE_RAW_PUMP_WIP_BUSY;
        }

        if (s_fe_raw_store.program_in_flight)
        {
            s_fe_raw_store.program_in_flight = false;
            if (!fe_raw_finish_in_flight_page())
            {
                fe_raw_note_pump_elapsed(context, pump_start_us);
                return ZY100_FE_RAW_PUMP_ERROR;
            }
            if (!s_fe_raw_store.pending)
            {
                fe_raw_note_pump_elapsed(context, pump_start_us);
                return ZY100_FE_RAW_PUMP_RECORD_DONE;
            }
        }

        if ((issued >= ZY100_FINAL_EDGE_RAW_STORE_PUMP_BUDGET) ||
            !s_fe_raw_store.pending)
        {
            break;
        }

        if (s_fe_raw_store.state == ZY100_FE_RAW_STATE_PAYLOAD)
        {
            uint32_t addr = s_fe_raw_store.current_bucket_addr +
                            ZY100_FE_RAW_PAYLOAD_OFFSET +
                            (s_fe_raw_store.payload_page_next *
                             ZY100_FE_RAW_PAGE_BYTES);

            if (!fe_raw_build_payload_page())
            {
                fe_raw_record_fail();
                fe_raw_note_pump_elapsed(context, pump_start_us);
                return ZY100_FE_RAW_PUMP_ERROR;
            }
            if (!fe_raw_issue_page(addr, false))
            {
                fe_raw_note_pump_elapsed(context, pump_start_us);
                return ZY100_FE_RAW_PUMP_ERROR;
            }
            issued++;
        }
        else if (s_fe_raw_store.state == ZY100_FE_RAW_STATE_HEADER)
        {
            fe_raw_build_header_page();
            if (!fe_raw_issue_page(s_fe_raw_store.current_bucket_addr, true))
            {
                fe_raw_note_pump_elapsed(context, pump_start_us);
                return ZY100_FE_RAW_PUMP_ERROR;
            }
            issued++;
        }
        else
        {
            break;
        }
    } while (issued < ZY100_FINAL_EDGE_RAW_STORE_PUMP_BUDGET);

    fe_raw_note_pump_elapsed(context, pump_start_us);
    return (issued != 0U) ?
           ZY100_FE_RAW_PUMP_PAGE_ISSUED : ZY100_FE_RAW_PUMP_NO_WORK;
}

zy100_fe_raw_pump_result_t zy100_final_edge_raw_store_pump_once(void)
{
    return zy100_final_edge_raw_store_pump_once_ex(
        ZY100_FE_RAW_PUMP_CONTEXT_RUNTIME);
}

bool zy100_final_edge_raw_store_has_pending_work(void)
{
    return s_fe_raw_store.pending ||
           s_fe_raw_store.program_in_flight ||
           (s_fe_raw_store.state != ZY100_FE_RAW_STATE_IDLE);
}

bool zy100_final_edge_raw_store_program_in_flight(void)
{
    return s_fe_raw_store.program_in_flight;
}

bool zy100_final_edge_raw_store_rt_gate_active(void)
{
#if ZY100_FINAL_EDGE_RAW_STORE_RT_GATE_ENABLE
    return !fe_raw_is_record_idle();
#else
    return false;
#endif
}

bool zy100_final_edge_raw_store_has_error(void)
{
    return s_fe_raw_store.error ||
           (s_fe_raw_store.stats.write_error != 0U) ||
           (s_fe_raw_store.stats.verify_error != 0U);
}

bool zy100_final_edge_raw_store_is_idle(void)
{
    return fe_raw_is_record_idle();
}

void zy100_final_edge_raw_store_get_stats(zy100_fe_raw_store_stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    s_fe_raw_store.stats.prepared = s_fe_raw_store.prepared ? 1U : 0U;
    s_fe_raw_store.stats.running = s_fe_raw_store.running ? 1U : 0U;
    s_fe_raw_store.stats.active = s_fe_raw_store.record_active ? 1U : 0U;
    s_fe_raw_store.stats.pending = s_fe_raw_store.pending ? 1U : 0U;
    s_fe_raw_store.stats.program_in_flight =
        s_fe_raw_store.program_in_flight ? 1U : 0U;
    s_fe_raw_store.stats.finalizing = s_fe_raw_store.finalizing ? 1U : 0U;
    s_fe_raw_store.stats.bucket_count = s_fe_raw_store.bucket_count;
    s_fe_raw_store.stats.session_raw_first_bucket =
        s_fe_raw_store.session_raw_first_bucket;
    s_fe_raw_store.stats.session_raw_limit_bucket =
        s_fe_raw_store.session_raw_limit_bucket;
    *out = s_fe_raw_store.stats;
}

void zy100_final_edge_raw_store_get_prepare_progress(
    zy100_fe_raw_prepare_progress_t *out)
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->phase = 1U;
    out->erase = s_fe_raw_store.stats.prepare_erase_count;
    out->total = s_fe_raw_store.prep_total_sectors;
    out->done = s_fe_raw_store.prep_done_sectors;
    out->err = (s_fe_raw_store.prep_state == ZY100_FE_RAW_PREP_ERROR) ?
               1U : 0U;
}

void zy100_final_edge_raw_store_set_b_active(bool active)
{
    s_fe_raw_store.b_active = active;
}

void zy100_final_edge_raw_store_note_rt_gate(uint32_t now_ms)
{
    s_fe_raw_store.stats.rt_gate_count++;
    if (s_fe_raw_store.gate_start_ms == 0U)
    {
        s_fe_raw_store.gate_start_ms = now_ms;
    }
    fe_raw_max_u32(&s_fe_raw_store.stats.rt_gate_max_ms,
                   now_ms - s_fe_raw_store.gate_start_ms);
}

void zy100_final_edge_raw_store_set_finalizing(bool finalizing)
{
    s_fe_raw_store.finalizing = finalizing;
}

void zy100_final_edge_raw_store_online_diag_reset(void)
{
    s_fe_raw_online_verified_pages = 0U;
    s_fe_raw_online_address_violations = 0U;
}

void zy100_final_edge_raw_store_get_online_diag(uint32_t *verified_pages,
                                                 uint32_t *address_violations)
{
    if (verified_pages != NULL)
    {
        *verified_pages = s_fe_raw_online_verified_pages;
    }
    if (address_violations != NULL)
    {
        *address_violations = s_fe_raw_online_address_violations;
    }
}

#else

bool zy100_final_edge_raw_store_prepare_erase_begin(uint32_t round)
{
    (void)round;
    return true;
}

bool zy100_final_edge_raw_store_prepare_erase_range_begin(
    uint32_t round,
    uint32_t raw_first_bucket,
    uint32_t raw_limit_bucket)
{
    (void)round;
    (void)raw_first_bucket;
    (void)raw_limit_bucket;
    return true;
}

bool zy100_final_edge_raw_store_prepare_append_begin(
    uint32_t round,
    uint32_t raw_data_begin_addr,
    uint32_t raw_region_end_addr)
{
    (void)round;
    (void)raw_data_begin_addr;
    (void)raw_region_end_addr;
    return true;
}

void zy100_final_edge_raw_store_online_diag_reset(void)
{
}

void zy100_final_edge_raw_store_get_online_diag(uint32_t *verified_pages,
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

bool zy100_final_edge_raw_store_prepare_target_begin(
    uint32_t round,
    const zy100_fe_store_target_provider_t *provider)
{
    (void)round;
    (void)provider;
    return true;
}

zy100_flash_prepare_status_t zy100_final_edge_raw_store_prepare_erase_poll(void)
{
    return ZY100_FLASH_PREP_DONE;
}

bool zy100_final_edge_raw_store_start(uint32_t round)
{
    (void)round;
    return true;
}

void zy100_final_edge_raw_store_reset_runtime(void)
{
}

void zy100_final_edge_raw_store_reset_after_clear(uint32_t round)
{
    (void)round;
}

void zy100_final_edge_raw_store_abort_pending(void)
{
}

void zy100_final_edge_raw_store_set_target_provider(
    const zy100_fe_store_target_provider_t *provider)
{
    (void)provider;
}

bool zy100_final_edge_raw_store_begin_hit(
    const uint8_t *raw_ring,
    uint32_t raw_ring_bytes,
    const zy100_final_edge_bscan_result_t *result,
    const zy100_final_edge_hit_window_view_t *view,
    uint32_t bscan_id,
    uint32_t rt_trigger_sample,
    uint16_t rt_trigger_tmst_raw,
    bool ois_od_warn)
{
    (void)raw_ring;
    (void)raw_ring_bytes;
    (void)result;
    (void)view;
    (void)bscan_id;
    (void)rt_trigger_sample;
    (void)rt_trigger_tmst_raw;
    (void)ois_od_warn;
    return false;
}

zy100_fe_raw_pump_result_t zy100_final_edge_raw_store_pump_once_ex(
    zy100_fe_raw_pump_context_t context)
{
    (void)context;
    return ZY100_FE_RAW_PUMP_NO_WORK;
}

zy100_fe_raw_pump_result_t zy100_final_edge_raw_store_pump_once(void)
{
    return zy100_final_edge_raw_store_pump_once_ex(
        ZY100_FE_RAW_PUMP_CONTEXT_RUNTIME);
}

bool zy100_final_edge_raw_store_has_pending_work(void)
{
    return false;
}

bool zy100_final_edge_raw_store_program_in_flight(void)
{
    return false;
}

bool zy100_final_edge_raw_store_waiting_target(void)
{
    return false;
}


bool zy100_final_edge_raw_store_waiting_target_space(void)
{
    return false;
}

zy100_fe_store_reserve_result_t
zy100_final_edge_raw_store_retry_target(void)
{
    return ZY100_FE_STORE_RESERVE_ERROR;
}

bool zy100_final_edge_raw_store_rt_gate_active(void)
{
    return false;
}

bool zy100_final_edge_raw_store_has_error(void)
{
    return false;
}

bool zy100_final_edge_raw_store_is_idle(void)
{
    return true;
}

void zy100_final_edge_raw_store_get_stats(zy100_fe_raw_store_stats_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }
}

void zy100_final_edge_raw_store_get_prepare_progress(
    zy100_fe_raw_prepare_progress_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
        out->phase = 1U;
    }
}

void zy100_final_edge_raw_store_set_b_active(bool active)
{
    (void)active;
}

void zy100_final_edge_raw_store_note_rt_gate(uint32_t now_ms)
{
    (void)now_ms;
}

void zy100_final_edge_raw_store_set_finalizing(bool finalizing)
{
    (void)finalizing;
}

#endif
