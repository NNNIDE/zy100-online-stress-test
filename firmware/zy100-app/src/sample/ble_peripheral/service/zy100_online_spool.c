#include "zy100_online_spool.h"
#if ZY100_ONLINE_STRESS_TEST_ENABLE
#include "zy100_online_stress.h"
#endif

#include <string.h>

#include "trace.h"

#include "../app_flags.h"
#include "../bsp/imu_bsp.h"
#include "../common/zy100_byteorder.h"
#include "../driver/gd25q32e_spi.h"
#include "../driver/spi_bus_owner.h"
#include "svc_app_watchdog.h"
#include "zy100_crc32.h"
#if ZY100_STRESS_IO_DIAG_ENABLE
#include "zy100_stress_diag.h"
#include "../bsp/bsp_capture_timebase.h"
#endif
#include "zy100_final_edge_raw_format.h"

#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_DIRECT_SPOOL_ENABLE

#if ZY100_ONLINE_RECOVERY_FAULT_INJECT_ENABLE
static bool s_online_spool_test_cleanup_error = false;
#endif

#define ZY100_ONLINE_SPOOL_END \
    (ZY100_ONLINE_SPOOL_REGION_BASE_ADDR + ZY100_ONLINE_SPOOL_REGION_BYTES)
#define ZY100_ONLINE_SPOOL_SECTOR_COUNT \
    (ZY100_ONLINE_SPOOL_REGION_BYTES / GD25Q32E_SECTOR_BYTES)
#define ZY100_ONLINE_SPOOL_BITMAP_WORDS \
    ((ZY100_ONLINE_SPOOL_SECTOR_COUNT + 31U) / 32U)
#define ZY100_ONLINE_SPOOL_RECORD_MAGIC   0x534C4E4FUL
#define ZY100_ONLINE_SPOOL_RECORD_VERSION 1U
#define ZY100_ONLINE_SPOOL_INVALID_INDEX  0xFFFFFFFFUL
#define ZY100_ONLINE_SPOOL_BLOCK32_SECTORS \
    (GD25Q32E_BLOCK32_BYTES / GD25Q32E_SECTOR_BYTES)

typedef enum
{
    ZY100_SPOOL_COMMIT_IDLE = 0U,
    ZY100_SPOOL_COMMIT_CRC,
    ZY100_SPOOL_COMMIT_HEADER_READY,
    ZY100_SPOOL_COMMIT_HEADER_WIP,
    ZY100_SPOOL_COMMIT_HEADER_VERIFY,
} zy100_online_spool_commit_state_t;

typedef struct
{
    bool active;
    bool data_done;
    bool crc_final_valid;
    uint8_t type;
    uint32_t source_id;
    uint32_t capture_segment_id;
    uint32_t segment_start_offset_ms;
    uint32_t record_id;
    uint32_t token;
    uint32_t header_addr;
    uint32_t data_addr;
    uint32_t source_record_bytes;
    uint32_t payload_bytes;
    uint32_t total_bytes;
    uint32_t crc_offset;
    uint32_t crc;
    uint32_t crc32_final;
    zy100_online_spool_commit_state_t commit_state;
} zy100_online_spool_reservation_t;

typedef struct
{
    uint32_t start_pos;
    uint32_t gap_bytes;
    uint32_t header_addr;
    uint32_t total_bytes;
    uint16_t generation;
} zy100_online_spool_reserve_plan_t;

typedef struct
{
    bool ready;
    bool session_active;
#if ZY100_ONLINE_PAGE_RETRY_ENABLE
    bool page_retry_pending;
#endif
    bool erase_wip;
    bool start_prepare_active;
    uint32_t start_prepare_minimum_bytes;
    uint32_t start_prepare_erase_errors;
    uint32_t start_prepare_timeout_count;
    bool clear_active;
    bool tx_cached;
    uint32_t session_id;
    uint32_t capture_segment_id;
    uint32_t segment_start_offset_ms;
    uint32_t next_record_id;
    uint32_t next_token;
    uint32_t reserve_pos;
    uint32_t commit_pos;
    uint32_t ack_pos;
    uint32_t send_pos;
    uint32_t wrap_gap_start;
    uint32_t wrap_gap_bytes;
    uint32_t erase_sector;
    uint32_t erase_sector_count;
    uint32_t erase_timeout_ms;
    uint32_t erase_start_us;
    uint32_t clear_next_sector;
    uint32_t progress_seq;
    uint32_t erased_bitmap[ZY100_ONLINE_SPOOL_BITMAP_WORDS];
    uint32_t dirty_bitmap[ZY100_ONLINE_SPOOL_BITMAP_WORDS];
    uint32_t reclaimable_bitmap[ZY100_ONLINE_SPOOL_BITMAP_WORDS];
    uint32_t error_printed_bitmap;
    uint8_t page[GD25Q32E_PAGE_BYTES];
    zy100_online_spool_reservation_t reservation;
    zy100_online_spool_record_t tx_record;
    zy100_online_spool_record_t sent_records[ZY100_ONLINE_RECORD_IN_FLIGHT_MAX];
    uint8_t sent_head;
    uint8_t sent_count;
    uint8_t last_reserve_block;
    zy100_online_spool_stats_t stats;
} zy100_online_spool_state_t;

static zy100_online_spool_state_t s_spool;

typedef char zy100_online_spool_page_check[
    (GD25Q32E_PAGE_BYTES == ZY100_ONLINE_SPOOL_RECORD_HEADER_BYTES) ? 1 : -1];
typedef char zy100_online_spool_sector_check[
    ((ZY100_ONLINE_SPOOL_REGION_BYTES % GD25Q32E_SECTOR_BYTES) == 0U) ? 1 : -1];
typedef char zy100_online_spool_bitmap_check[
    (ZY100_ONLINE_SPOOL_SECTOR_COUNT <= 256U) ? 1 : -1];
typedef char zy100_online_spool_block32_base_check[
    ((ZY100_ONLINE_SPOOL_REGION_BASE_ADDR % GD25Q32E_BLOCK32_BYTES) == 0UL) ?
        1 : -1];
typedef char zy100_online_spool_block32_size_check[
    ((ZY100_ONLINE_SPOOL_REGION_BYTES % GD25Q32E_BLOCK32_BYTES) == 0UL) ?
        1 : -1];
typedef char zy100_online_spool_block32_sector_check[
    (ZY100_ONLINE_SPOOL_BLOCK32_SECTORS == 8UL) ? 1 : -1];
typedef char zy100_online_spool_power2_check[
    ((ZY100_ONLINE_SPOOL_REGION_BYTES &
      (ZY100_ONLINE_SPOOL_REGION_BYTES - 1UL)) == 0UL) ? 1 : -1];

static uint32_t spool_pos_offset(uint32_t pos)
{
    return pos & (ZY100_ONLINE_SPOOL_REGION_BYTES - 1UL);
}

static uint32_t spool_pos_addr(uint32_t pos)
{
    return ZY100_ONLINE_SPOOL_REGION_BASE_ADDR + spool_pos_offset(pos);
}

static uint16_t spool_pos_generation(uint32_t pos)
{
    return (uint16_t)(pos / ZY100_ONLINE_SPOOL_REGION_BYTES);
}

static bool spool_pos_generation_valid(uint32_t pos)
{
    return (pos / ZY100_ONLINE_SPOOL_REGION_BYTES) <= 0xFFFFUL;
}

static uint32_t spool_skip_gap_value(uint32_t pos)
{
    if ((s_spool.wrap_gap_bytes != 0U) &&
        (pos == s_spool.wrap_gap_start))
    {
        return pos + s_spool.wrap_gap_bytes;
    }
    return pos;
}

static bool spool_skip_gap(uint32_t *pos, bool release)
{
    uint32_t next;

    if (pos == NULL)
    {
        return false;
    }
    next = spool_skip_gap_value(*pos);
    if (next < *pos)
    {
        return false;
    }
    if (next != *pos)
    {
        *pos = next;
        if (release)
        {
            s_spool.wrap_gap_start = 0U;
            s_spool.wrap_gap_bytes = 0U;
        }
    }
    return true;
}

static bool spool_skip_gap_bounded(uint32_t *pos,
                                   uint32_t limit,
                                   bool release)
{
    uint32_t next;

    if (pos == NULL)
    {
        return false;
    }
    next = spool_skip_gap_value(*pos);
    if ((next < *pos) || (next > limit))
    {
        return next >= *pos;
    }
    if (next != *pos)
    {
        *pos = next;
        if (release)
        {
            s_spool.wrap_gap_start = 0U;
            s_spool.wrap_gap_bytes = 0U;
        }
    }
    return true;
}

static bool spool_used_bytes(uint32_t *used_out)
{
    uint32_t used;

    if ((used_out == NULL) || (s_spool.reserve_pos < s_spool.ack_pos))
    {
        return false;
    }
    used = s_spool.reserve_pos - s_spool.ack_pos;
    if (used > ZY100_ONLINE_SPOOL_REGION_BYTES)
    {
        return false;
    }
    *used_out = used;
    return true;
}

#if ZY100_STRESS_IO_DIAG_ENABLE
static imu_status_t spool_io_busy(bool *busy)
{
    uint32_t a = 0U, b = 0U;
    bool va = bsp_capture_timebase_snapshot(&a), vb;
    imu_status_t status = gd25q32e_is_busy(busy);
    vb = bsp_capture_timebase_snapshot(&b);
    zy100_stress_diag_io_note(3U, a - b, va && vb);
    zy100_stress_diag_io_poll(b, vb, status == IMU_STATUS_BUS_ERROR,
                              *busy, status == IMU_STATUS_OK);
    return status;
}
static imu_status_t spool_io_program(uint32_t addr, const uint8_t *data, uint16_t len)
{
    uint32_t a = 0U, b = 0U;
    bool va = bsp_capture_timebase_snapshot(&a), vb;
    imu_status_t status = gd25q32e_page_program(addr, data, len);
    vb = bsp_capture_timebase_snapshot(&b);
    zy100_stress_diag_io_note(0U, a - b, va && vb);
    if (status == IMU_STATUS_OK) { zy100_stress_diag_io_issue(a, va); }
#if ZY100_ONLINE_PAGE_RETRY_ENABLE
    if (status == IMU_STATUS_OK) { s_spool.page_retry_pending = true; }
#endif
    if (status == IMU_STATUS_BUS_ERROR) { zy100_stress_diag_io_bus(); }
    return status;
}
static imu_status_t spool_io_read(uint32_t kind, uint32_t addr, uint8_t *data, uint16_t len)
{
    uint32_t a = 0U, b = 0U;
    bool va = bsp_capture_timebase_snapshot(&a), vb;
    imu_status_t status = gd25q32e_read_fast(addr, data, len);
#if ZY100_ONLINE_PAGE_RETRY_ENABLE
    /* A TX poll/read may see WIP clear first. Keep the writer runnable until
     * its own readback succeeds; transmitting older records cannot clear it. */
    if (kind == 1U && status == IMU_STATUS_OK) { s_spool.page_retry_pending = false; }
#endif
    vb = bsp_capture_timebase_snapshot(&b);
    zy100_stress_diag_io_note(kind, a - b, va && vb);
    if (status == IMU_STATUS_BUS_ERROR) { zy100_stress_diag_io_bus(); }
    return status;
}
#else
#define spool_io_busy gd25q32e_is_busy
#define spool_io_program gd25q32e_page_program
#define spool_io_read(kind, addr, data, len) gd25q32e_read_fast(addr, data, len)
#endif

#if ZY100_ONLINE_PAGE_RETRY_ENABLE
bool zy100_online_spool_page_retry_pending(void)
{
    return s_spool.session_active && s_spool.reservation.active &&
           s_spool.page_retry_pending && !s_spool.erase_wip &&
           (spi_bus_current_owner() == SPI_OWNER_NONE);
}
#endif

static uint32_t spool_align_page(uint32_t value)
{
    return (value + GD25Q32E_PAGE_BYTES - 1U) &
           ~(uint32_t)(GD25Q32E_PAGE_BYTES - 1U);
}

static uint32_t spool_sector_index(uint32_t addr)
{
    if ((addr < ZY100_ONLINE_SPOOL_REGION_BASE_ADDR) ||
        (addr >= ZY100_ONLINE_SPOOL_END))
    {
        return ZY100_ONLINE_SPOOL_INVALID_INDEX;
    }
    return (addr - ZY100_ONLINE_SPOOL_REGION_BASE_ADDR) /
           GD25Q32E_SECTOR_BYTES;
}

static uint32_t spool_sector_addr(uint32_t sector)
{
    return ZY100_ONLINE_SPOOL_REGION_BASE_ADDR +
           (sector * GD25Q32E_SECTOR_BYTES);
}

static bool spool_bitmap_get(const uint32_t *bitmap, uint32_t index)
{
    if ((bitmap == NULL) || (index >= ZY100_ONLINE_SPOOL_SECTOR_COUNT))
    {
        return false;
    }
    return (bitmap[index / 32U] & (1UL << (index % 32U))) != 0U;
}

static void spool_bitmap_set(uint32_t *bitmap, uint32_t index)
{
    if ((bitmap != NULL) && (index < ZY100_ONLINE_SPOOL_SECTOR_COUNT))
    {
        bitmap[index / 32U] |= (1UL << (index % 32U));
    }
}

static void spool_bitmap_clear(uint32_t *bitmap, uint32_t index)
{
    if ((bitmap != NULL) && (index < ZY100_ONLINE_SPOOL_SECTOR_COUNT))
    {
        bitmap[index / 32U] &= ~(1UL << (index % 32U));
    }
}

static uint32_t spool_bitmap_count(const uint32_t *bitmap)
{
    uint32_t index;
    uint32_t count = 0U;

    if (bitmap == NULL)
    {
        return 0U;
    }

    for (index = 0U; index < ZY100_ONLINE_SPOOL_SECTOR_COUNT; index++)
    {
        if ((bitmap[index / 32U] & (1UL << (index % 32U))) != 0U)
        {
            count++;
        }
    }
    return count;
}

static void spool_latch_error(uint32_t code, uint32_t detail)
{
    uint32_t bit = (code < 32U) ? (1UL << code) : 0U;

    s_spool.stats.last_error = code;
    s_spool.stats.last_error_detail = detail;
    if ((bit != 0U) && ((s_spool.error_printed_bitmap & bit) != 0U))
    {
        return;
    }
    s_spool.error_printed_bitmap |= bit;
}

static uint32_t spool_erased_from_pos(uint32_t pos, uint32_t limit_bytes)
{
    uint32_t offset = spool_pos_offset(pos);
    uint32_t sector;
    uint32_t bytes = 0U;
    uint32_t partial;

    if ((limit_bytes == 0U) ||
        (limit_bytes > ZY100_ONLINE_SPOOL_REGION_BYTES))
    {
        return 0U;
    }
    sector = offset / GD25Q32E_SECTOR_BYTES;
    partial = offset % GD25Q32E_SECTOR_BYTES;
    if (partial != 0U)
    {
        bytes = GD25Q32E_SECTOR_BYTES - partial;
        if (bytes > limit_bytes)
        {
            bytes = limit_bytes;
        }
        sector = (sector + 1U) % ZY100_ONLINE_SPOOL_SECTOR_COUNT;
    }
    while ((bytes < limit_bytes) &&
           spool_bitmap_get(s_spool.erased_bitmap, sector))
    {
        uint32_t add = GD25Q32E_SECTOR_BYTES;

        if (add > (limit_bytes - bytes))
        {
            add = limit_bytes - bytes;
        }
        bytes += add;
        sector = (sector + 1U) % ZY100_ONLINE_SPOOL_SECTOR_COUNT;
    }
    return bytes;
}

static uint32_t spool_erased_ahead_bytes(void)
{
    uint32_t used;

    if (!spool_used_bytes(&used))
    {
        return 0U;
    }
    return spool_erased_from_pos(s_spool.reserve_pos,
                                 ZY100_ONLINE_SPOOL_REGION_BYTES - used);
}

static void spool_refresh_stats(void)
{
    uint32_t used = 0U;

    (void)spool_used_bytes(&used);
    s_spool.stats.logical_used_bytes = used;
    s_spool.stats.reserve_generation =
        (uint32_t)spool_pos_generation(s_spool.reserve_pos);
    s_spool.stats.ack_generation =
        (uint32_t)spool_pos_generation(s_spool.ack_pos);
    s_spool.stats.reserve_ptr = spool_pos_addr(s_spool.reserve_pos);
    s_spool.stats.commit_ptr = spool_pos_addr(s_spool.commit_pos);
    s_spool.stats.tx_ptr = spool_pos_addr(s_spool.ack_pos);
    s_spool.stats.erased_ahead_bytes = spool_erased_ahead_bytes();
    if (s_spool.session_active &&
        (s_spool.stats.erased_ahead_bytes <
         s_spool.stats.erased_ahead_min_bytes))
    {
        s_spool.stats.erased_ahead_min_bytes =
            s_spool.stats.erased_ahead_bytes;
    }
    s_spool.stats.erased_sector_count =
        spool_bitmap_count(s_spool.erased_bitmap);
    s_spool.stats.reclaimable_sector_count =
        spool_bitmap_count(s_spool.reclaimable_bitmap);
}

static bool spool_flash_busy(bool *busy)
{
    imu_status_t status;

    if (busy == NULL)
    {
        s_spool.stats.read_errors++;
        spool_latch_error(1U, 0U);
        return false;
    }
    status = spool_io_busy(busy);
    if (status == IMU_STATUS_BUS_ERROR)
    {
        /* Shared-SPI ownership is transient; let the async caller retry. */
        return false;
    }
    if (status != IMU_STATUS_OK)
    {
        s_spool.stats.read_errors++;
        spool_latch_error(1U, (uint32_t)status);
        return false;
    }
    return true;
}

static bool spool_ranges_overlap(uint32_t first_addr,
                                 uint32_t first_bytes,
                                 uint32_t second_addr,
                                 uint32_t second_bytes)
{
    uint32_t first_end = first_addr + first_bytes;
    uint32_t second_end = second_addr + second_bytes;

    return (first_addr < second_end) && (second_addr < first_end);
}

static bool spool_sector_contains_live_bytes(uint32_t sector)
{
    uint32_t used;
    uint32_t live_offset;
    uint32_t sector_offset;
    uint32_t chunk;

    if ((sector >= ZY100_ONLINE_SPOOL_SECTOR_COUNT) ||
        !spool_used_bytes(&used) || (used == 0U))
    {
        return false;
    }
    live_offset = spool_pos_offset(s_spool.ack_pos);
    sector_offset = sector * GD25Q32E_SECTOR_BYTES;
    while (used != 0U)
    {
        chunk = ZY100_ONLINE_SPOOL_REGION_BYTES - live_offset;
        if (chunk > used)
        {
            chunk = used;
        }
        if ((live_offset <
             (sector_offset + GD25Q32E_SECTOR_BYTES)) &&
            (sector_offset < (live_offset + chunk)))
        {
            return true;
        }
        used -= chunk;
        live_offset = 0U;
    }
    return false;
}

static bool spool_reservation_range_valid(uint32_t token,
                                          uint32_t addr,
                                          uint32_t len)
{
    uint32_t data_end;

    if (!s_spool.reservation.active ||
        (s_spool.reservation.token != token) ||
        (len == 0U) ||
        (addr < s_spool.reservation.data_addr))
    {
        return false;
    }
    data_end = s_spool.reservation.data_addr +
               s_spool.reservation.source_record_bytes;
    return (addr < data_end) && (len <= (data_end - addr));
}

static bool spool_sector_erase_protected(uint32_t sector)
{
    uint32_t sector_addr;
    uint8_t index;

    if (sector >= ZY100_ONLINE_SPOOL_SECTOR_COUNT)
    {
        return true;
    }
    if (!s_spool.session_active)
    {
        return false;
    }
    if (!spool_bitmap_get(s_spool.reclaimable_bitmap, sector))
    {
        return true;
    }
    if (spool_sector_contains_live_bytes(sector))
    {
        return true;
    }
    sector_addr = spool_sector_addr(sector);
    if (s_spool.reservation.active &&
        spool_ranges_overlap(sector_addr,
                             GD25Q32E_SECTOR_BYTES,
                             s_spool.reservation.header_addr,
                             s_spool.reservation.total_bytes))
    {
        return true;
    }
    if (s_spool.tx_cached &&
        spool_ranges_overlap(sector_addr,
                             GD25Q32E_SECTOR_BYTES,
                             s_spool.tx_record.header_addr,
                             s_spool.tx_record.record_bytes))
    {
        return true;
    }
    for (index = 0U; index < s_spool.sent_count; index++)
    {
        const zy100_online_spool_record_t *sent =
            &s_spool.sent_records[(s_spool.sent_head + index) %
                                  ZY100_ONLINE_RECORD_IN_FLIGHT_MAX];
        if (spool_ranges_overlap(sector_addr,
                                 GD25Q32E_SECTOR_BYTES,
                                 sent->header_addr,
                                 sent->record_bytes))
        {
            return true;
        }
    }
    return false;
}

static bool spool_block32_reclaimable(uint32_t first_sector)
{
    uint32_t offset;

    if ((first_sector >= ZY100_ONLINE_SPOOL_SECTOR_COUNT) ||
        ((first_sector + ZY100_ONLINE_SPOOL_BLOCK32_SECTORS) >
         ZY100_ONLINE_SPOOL_SECTOR_COUNT) ||
        ((spool_sector_addr(first_sector) % GD25Q32E_BLOCK32_BYTES) != 0UL))
    {
        return false;
    }
    for (offset = 0U; offset < ZY100_ONLINE_SPOOL_BLOCK32_SECTORS; offset++)
    {
        uint32_t sector = first_sector + offset;

        if (spool_bitmap_get(s_spool.erased_bitmap, sector))
        {
            continue;
        }
        if (!spool_bitmap_get(s_spool.reclaimable_bitmap, sector) ||
            spool_sector_erase_protected(sector))
        {
            return false;
        }
    }
    return true;
}

static bool spool_block32_erased(uint32_t first_sector)
{
    uint32_t offset;

    if ((first_sector >= ZY100_ONLINE_SPOOL_SECTOR_COUNT) ||
        ((first_sector + ZY100_ONLINE_SPOOL_BLOCK32_SECTORS) >
         ZY100_ONLINE_SPOOL_SECTOR_COUNT))
    {
        return false;
    }
    for (offset = 0U; offset < ZY100_ONLINE_SPOOL_BLOCK32_SECTORS; offset++)
    {
        if (!spool_bitmap_get(s_spool.erased_bitmap, first_sector + offset))
        {
            return false;
        }
    }
    return true;
}

static zy100_online_spool_io_result_t spool_tx_read_ready(void)
{
    bool busy = false;
    imu_status_t status;

    if (s_spool.erase_wip)
    {
        s_spool.stats.tx_read_wip_blocked++;
        s_spool.stats.tx_read_busy_retry++;
        return ZY100_ONLINE_SPOOL_IO_BUSY;
    }
    if (spi_bus_current_owner() != SPI_OWNER_NONE)
    {
#if ZY100_STRESS_IO_DIAG_ENABLE
        { uint32_t t = 0U; bool valid = bsp_capture_timebase_snapshot(&t);
          zy100_stress_diag_io_poll(t, valid, true, false, false); }
#endif
        s_spool.stats.tx_read_spi_busy++;
        s_spool.stats.tx_read_busy_retry++;
        return ZY100_ONLINE_SPOOL_IO_BUSY;
    }
    status = spool_io_busy(&busy);
    if (status == IMU_STATUS_BUS_ERROR)
    {
        s_spool.stats.tx_read_spi_busy++;
        s_spool.stats.tx_read_busy_retry++;
        return ZY100_ONLINE_SPOOL_IO_BUSY;
    }
    if (status != IMU_STATUS_OK)
    {
        s_spool.stats.read_errors++;
        spool_latch_error(16U, (uint32_t)status);
        return ZY100_ONLINE_SPOOL_IO_ERROR;
    }
    if (busy)
    {
        s_spool.stats.tx_read_wip_blocked++;
        s_spool.stats.tx_read_busy_retry++;
        return ZY100_ONLINE_SPOOL_IO_BUSY;
    }
    return ZY100_ONLINE_SPOOL_IO_OK;
}

static bool spool_poll_erase_wip(void)
{
    bool busy = false;
    uint32_t erase_ms;
    uint32_t offset;

    if (!s_spool.erase_wip)
    {
        return false;
    }
    if (!spool_flash_busy(&busy))
    {
        return true;
    }
    if (busy)
    {
        if ((((uint32_t)imu_bsp_local_timestamp_us() -
              s_spool.erase_start_us) >=
             (s_spool.erase_timeout_ms * 1000UL)) &&
            ((s_spool.stats.last_error != 17U) ||
             (s_spool.stats.last_error_detail != s_spool.erase_sector)))
        {
            s_spool.stats.erase_errors++;
            s_spool.stats.erase_timeout_count++;
            spool_latch_error(17U, s_spool.erase_sector);
        }
        return true;
    }
    for (offset = 0U; offset < s_spool.erase_sector_count; offset++)
    {
        uint32_t sector = s_spool.erase_sector + offset;

        spool_bitmap_set(s_spool.erased_bitmap, sector);
        spool_bitmap_clear(s_spool.dirty_bitmap, sector);
        spool_bitmap_clear(s_spool.reclaimable_bitmap, sector);
    }
    s_spool.erase_wip = false;
    s_spool.stats.erase_count++;
    s_spool.stats.erase_complete_count++;
    if (s_spool.erase_sector_count == ZY100_ONLINE_SPOOL_BLOCK32_SECTORS)
    {
        s_spool.stats.block32_erase_complete_count++;
    }
    erase_ms = ((uint32_t)imu_bsp_local_timestamp_us() -
                s_spool.erase_start_us) / 1000U;
    s_spool.stats.erase_total_ms += erase_ms;
    if (erase_ms > s_spool.stats.erase_max_ms)
    {
        s_spool.stats.erase_max_ms = erase_ms;
    }
    s_spool.erase_start_us = 0U;
    s_spool.erase_sector_count = 0U;
    s_spool.erase_timeout_ms = 0U;
    spool_refresh_stats();
    return true;
}

static bool spool_issue_erase(uint32_t sector)
{
    bool busy = false;
    imu_status_t status;

    if ((sector >= ZY100_ONLINE_SPOOL_SECTOR_COUNT) || s_spool.erase_wip)
    {
        return false;
    }
    if (spool_sector_erase_protected(sector))
    {
        s_spool.stats.erase_tx_conflict_prevented++;
        return false;
    }
    if (!spool_flash_busy(&busy) || busy)
    {
        return false;
    }
    status = gd25q32e_init();
    if (status == IMU_STATUS_BUS_ERROR)
    {
        return false;
    }
    if (status != IMU_STATUS_OK)
    {
        s_spool.stats.erase_errors++;
        spool_latch_error(2U, sector);
        DBG_DIRECT("[ONLINE_SPOOL][ERR] erase4k sector=%lu stage=init status=%u owner=%u",
                   (unsigned long)sector,
                   (uint32_t)status,
                   (uint32_t)spi_bus_current_owner());
        return false;
    }
    status = gd25q32e_sector_erase_4k(spool_sector_addr(sector));
    if (status == IMU_STATUS_BUS_ERROR)
    {
        /* WREN and ERASE acquire the shared bus separately. A competing
         * owner between them is retryable and must not poison START. */
        return false;
    }
    if (status != IMU_STATUS_OK)
    {
        s_spool.stats.erase_errors++;
        spool_latch_error(2U, sector);
        DBG_DIRECT("[ONLINE_SPOOL][ERR] erase4k sector=%lu stage=cmd status=%u owner=%u",
                   (unsigned long)sector,
                   (uint32_t)status,
                   (uint32_t)spi_bus_current_owner());
        return false;
    }
    s_spool.erase_wip = true;
    s_spool.stats.erase_issue_count++;
    s_spool.erase_sector = sector;
    s_spool.erase_sector_count = 1U;
    s_spool.erase_timeout_ms = GD25Q32E_SECTOR_ERASE_TIMEOUT_MS;
    s_spool.erase_start_us = (uint32_t)imu_bsp_local_timestamp_us();
    return true;
}

static bool spool_issue_erase_block32(uint32_t first_sector)
{
    bool busy = false;
    imu_status_t status;

    if (s_spool.erase_wip || !spool_block32_reclaimable(first_sector))
    {
        return false;
    }
    if (!spool_flash_busy(&busy) || busy)
    {
        return false;
    }
    status = gd25q32e_block_erase_32k(spool_sector_addr(first_sector));
    if (status == IMU_STATUS_BUS_ERROR)
    {
        return false;
    }
    if (status != IMU_STATUS_OK)
    {
        s_spool.stats.erase_errors++;
        spool_latch_error(18U, first_sector);
        DBG_DIRECT("[ONLINE_SPOOL][ERR] erase32k sector=%lu status=%u owner=%u",
                   (unsigned long)first_sector,
                   (uint32_t)status,
                   (uint32_t)spi_bus_current_owner());
        return false;
    }
    s_spool.erase_wip = true;
    s_spool.stats.erase_issue_count++;
    s_spool.stats.block32_erase_issue_count++;
    s_spool.erase_sector = first_sector;
    s_spool.erase_sector_count = ZY100_ONLINE_SPOOL_BLOCK32_SECTORS;
    s_spool.erase_timeout_ms = GD25Q32E_BLOCK32_ERASE_TIMEOUT_MS;
    s_spool.erase_start_us = (uint32_t)imu_bsp_local_timestamp_us();
    return true;
}

static bool spool_reservation_sectors_ready(uint32_t addr,
                                             uint32_t total_bytes)
{
    uint32_t first = spool_sector_index(addr);
    uint32_t last = spool_sector_index(addr + total_bytes - 1U);
    uint32_t sector;

    if ((first == ZY100_ONLINE_SPOOL_INVALID_INDEX) ||
        (last == ZY100_ONLINE_SPOOL_INVALID_INDEX))
    {
        return false;
    }
    for (sector = first; sector <= last; sector++)
    {
        if ((sector == first) &&
            ((addr % GD25Q32E_SECTOR_BYTES) != 0U) &&
            spool_bitmap_get(s_spool.dirty_bitmap, sector))
        {
            continue;
        }
        if (!spool_bitmap_get(s_spool.erased_bitmap, sector))
        {
            return false;
        }
    }
    return true;
}

static bool spool_build_reserve_plan(
    uint32_t record_bytes,
    zy100_online_spool_reserve_plan_t *plan,
    zy100_online_spool_reserve_block_t *block_out)
{
    uint32_t aligned = spool_align_page(record_bytes);
    uint32_t total = ZY100_ONLINE_SPOOL_RECORD_HEADER_BYTES + aligned;
    uint32_t used;
    uint32_t offset;
    uint32_t gap = 0U;
    uint32_t start_pos;

    if (block_out != NULL)
    {
        *block_out = ZY100_ONLINE_SPOOL_RESERVE_BLOCK_NONE;
    }
    if ((plan == NULL) || !s_spool.session_active ||
        s_spool.reservation.active || (record_bytes == 0U) ||
        (aligned != record_bytes) ||
        (total > ZY100_ONLINE_SPOOL_REGION_BYTES) ||
        !spool_used_bytes(&used))
    {
        return false;
    }
    offset = spool_pos_offset(s_spool.reserve_pos);
    if ((offset + total) > ZY100_ONLINE_SPOOL_REGION_BYTES)
    {
        gap = ZY100_ONLINE_SPOOL_REGION_BYTES - offset;
    }
    if ((gap != 0U) && (s_spool.wrap_gap_bytes != 0U))
    {
        return false;
    }
    if ((gap > (0xFFFFFFFFUL - s_spool.reserve_pos)) ||
        (total > (0xFFFFFFFFUL - s_spool.reserve_pos - gap)))
    {
        return false;
    }
    if ((gap + total) > (ZY100_ONLINE_SPOOL_REGION_BYTES - used))
    {
        if (block_out != NULL)
        {
            *block_out = ZY100_ONLINE_SPOOL_RESERVE_BLOCK_UNACKED;
        }
        return false;
    }
    start_pos = s_spool.reserve_pos + gap;
    if (!spool_pos_generation_valid(start_pos) ||
        !spool_pos_generation_valid(start_pos + total))
    {
        return false;
    }
    memset(plan, 0, sizeof(*plan));
    plan->start_pos = start_pos;
    plan->gap_bytes = gap;
    plan->header_addr = spool_pos_addr(start_pos);
    plan->total_bytes = total;
    plan->generation = spool_pos_generation(start_pos);
    if (!spool_reservation_sectors_ready(plan->header_addr, total))
    {
        if (block_out != NULL)
        {
            *block_out = ZY100_ONLINE_SPOOL_RESERVE_BLOCK_ERASE_PENDING;
        }
        return false;
    }
    return true;
}

static bool spool_provider_can_reserve(void *context, uint32_t record_bytes)
{
    zy100_online_spool_reserve_plan_t plan;

    (void)context;
    return spool_build_reserve_plan(record_bytes, &plan, NULL);
}

static zy100_fe_store_reserve_result_t spool_provider_reserve(
    void *context,
    zy100_fe_store_target_kind_t kind,
    uint32_t source_id,
    uint32_t record_bytes,
    uint32_t payload_bytes,
    zy100_fe_store_target_t *target)
{
    zy100_online_spool_reserve_plan_t plan;
    zy100_online_spool_reserve_block_t block;
    uint32_t first;
    uint32_t last;
    uint32_t sector;
    uint16_t reserve_generation_before;

    (void)context;
    s_spool.last_reserve_block = ZY100_ONLINE_SPOOL_RESERVE_BLOCK_NONE;
    if ((uint32_t)kind < ZY100_ONLINE_RECORD_TYPE_COUNT)
    {
        s_spool.stats.reserve_attempt[(uint32_t)kind]++;
    }
    if ((target == NULL) || (record_bytes == 0U) ||
        ((record_bytes % GD25Q32E_PAGE_BYTES) != 0U) ||
        ((uint32_t)kind >= ZY100_ONLINE_RECORD_TYPE_COUNT) || (kind == 0U))
    {
        s_spool.stats.state_errors++;
        if ((uint32_t)kind < ZY100_ONLINE_RECORD_TYPE_COUNT)
        {
            s_spool.stats.reserve_error[(uint32_t)kind]++;
        }
        spool_latch_error(3U, record_bytes);
        return ZY100_FE_STORE_RESERVE_ERROR;
    }
    if (s_spool.reservation.active)
    {
        s_spool.stats.reserve_busy[(uint32_t)kind]++;
        if (s_spool.reservation.type < ZY100_ONLINE_RECORD_TYPE_COUNT)
        {
            s_spool.stats.reserve_busy_owner[s_spool.reservation.type]++;
        }
        return ZY100_FE_STORE_RESERVE_BUSY;
    }
    if (s_spool.capture_segment_id == 0U)
    {
        s_spool.stats.reserve_error[(uint32_t)kind]++;
        s_spool.stats.state_errors++;
        spool_latch_error(16U, (uint32_t)kind);
        return ZY100_FE_STORE_RESERVE_ERROR;
    }
    if (!spool_build_reserve_plan(record_bytes, &plan, &block))
    {
        if (block == ZY100_ONLINE_SPOOL_RESERVE_BLOCK_NONE)
        {
            s_spool.stats.reserve_error[(uint32_t)kind]++;
            s_spool.stats.state_errors++;
            spool_latch_error(26U, s_spool.reserve_pos);
            return ZY100_FE_STORE_RESERVE_ERROR;
        }
        s_spool.last_reserve_block = (uint8_t)block;
        if (block == ZY100_ONLINE_SPOOL_RESERVE_BLOCK_UNACKED)
        {
            s_spool.stats.capacity_blocked++;
        }
        else
        {
            s_spool.stats.erase_blocked++;
        }
        s_spool.stats.reserve_full[(uint32_t)kind]++;
        return ZY100_FE_STORE_RESERVE_FULL;
    }

    reserve_generation_before = spool_pos_generation(s_spool.reserve_pos);
    memset(&s_spool.reservation, 0, sizeof(s_spool.reservation));
    s_spool.next_record_id++;
    s_spool.next_token++;
    if ((s_spool.next_record_id == 0U) || (s_spool.next_token == 0U))
    {
        s_spool.stats.state_errors++;
        spool_latch_error(4U, 0U);
        return ZY100_FE_STORE_RESERVE_ERROR;
    }
    s_spool.reservation.active = true;
    s_spool.reservation.type = (uint8_t)kind;
    s_spool.reservation.source_id = source_id;
    s_spool.reservation.capture_segment_id = s_spool.capture_segment_id;
    s_spool.reservation.segment_start_offset_ms =
        s_spool.segment_start_offset_ms;
    s_spool.reservation.record_id = s_spool.next_record_id;
    s_spool.reservation.token = s_spool.next_token;
    if (plan.gap_bytes != 0U)
    {
        s_spool.wrap_gap_start = s_spool.reserve_pos;
        s_spool.wrap_gap_bytes = plan.gap_bytes;
        s_spool.reserve_pos += plan.gap_bytes;
    }
    s_spool.reservation.header_addr = plan.header_addr;
    s_spool.reservation.data_addr = plan.header_addr +
                                     ZY100_ONLINE_SPOOL_RECORD_HEADER_BYTES;
    s_spool.reservation.source_record_bytes = record_bytes;
    s_spool.reservation.payload_bytes = payload_bytes;
    s_spool.reservation.total_bytes = plan.total_bytes;
    s_spool.reservation.crc = zy100_crc32_ieee_begin();
    s_spool.reservation.commit_state = ZY100_SPOOL_COMMIT_IDLE;

    first = spool_sector_index(plan.header_addr);
    last = spool_sector_index(plan.header_addr + plan.total_bytes - 1U);
    for (sector = first; sector <= last; sector++)
    {
        spool_bitmap_clear(s_spool.erased_bitmap, sector);
        spool_bitmap_set(s_spool.dirty_bitmap, sector);
        spool_bitmap_clear(s_spool.reclaimable_bitmap, sector);
    }
    target->data_addr = s_spool.reservation.data_addr;
    target->capacity_bytes = record_bytes;
    target->online_record_id = s_spool.reservation.record_id;
    target->token = s_spool.reservation.token;
    s_spool.reserve_pos = plan.start_pos + plan.total_bytes;
    if (spool_pos_generation(s_spool.reserve_pos) !=
        reserve_generation_before)
    {
        s_spool.stats.wrap_count++;
    }
    s_spool.last_reserve_block = ZY100_ONLINE_SPOOL_RESERVE_BLOCK_NONE;
    s_spool.stats.produced[(uint32_t)kind]++;
    s_spool.stats.reserve_ok[(uint32_t)kind]++;
    spool_refresh_stats();
    return ZY100_FE_STORE_RESERVE_OK;
}

static bool spool_provider_commit(void *context, uint32_t token)
{
    (void)context;
    if (!s_spool.reservation.active || s_spool.reservation.data_done ||
        (s_spool.reservation.token != token))
    {
        s_spool.stats.state_errors++;
        spool_latch_error(5U, token);
        return false;
    }
    s_spool.reservation.data_done = true;
    s_spool.reservation.commit_state = ZY100_SPOOL_COMMIT_CRC;
    return true;
}

static void spool_provider_abort(void *context, uint32_t token)
{
    uint32_t start_pos;

    (void)context;
    if (!s_spool.reservation.active ||
        (s_spool.reservation.token != token))
    {
        return;
    }
    if (s_spool.reserve_pos < s_spool.reservation.total_bytes)
    {
        s_spool.stats.state_errors++;
        spool_latch_error(27U, s_spool.reserve_pos);
        return;
    }
    start_pos = s_spool.reserve_pos - s_spool.reservation.total_bytes;
    if ((s_spool.wrap_gap_bytes != 0U) &&
        (start_pos == (s_spool.wrap_gap_start + s_spool.wrap_gap_bytes)))
    {
        s_spool.reserve_pos = s_spool.wrap_gap_start;
        s_spool.wrap_gap_start = 0U;
        s_spool.wrap_gap_bytes = 0U;
    }
    else
    {
        s_spool.reserve_pos = start_pos;
    }
    s_spool.stats.dropped[s_spool.reservation.type]++;
    memset(&s_spool.reservation, 0, sizeof(s_spool.reservation));
    spool_refresh_stats();
}

static const zy100_fe_store_target_provider_t s_spool_provider =
{
    NULL,
    spool_provider_can_reserve,
    spool_provider_reserve,
    spool_provider_commit,
    spool_provider_abort,
};

static void spool_fill_header(void)
{
    uint32_t source_crc32 = s_spool.reservation.crc_final_valid ?
                            s_spool.reservation.crc32_final :
                            zy100_crc32_ieee_finish(s_spool.reservation.crc);

    memset(s_spool.page, 0xFF, sizeof(s_spool.page));
    zy100_put_u32_le(&s_spool.page[0], ZY100_ONLINE_SPOOL_RECORD_MAGIC);
    zy100_put_u16_le(&s_spool.page[4], ZY100_ONLINE_SPOOL_RECORD_VERSION);
    zy100_put_u16_le(&s_spool.page[6], ZY100_ONLINE_SPOOL_RECORD_HEADER_BYTES);
    zy100_put_u32_le(&s_spool.page[8], s_spool.session_id);
    s_spool.page[12] = s_spool.reservation.type;
    s_spool.page[13] = ZY100_ONLINE_SPOOL_HEADER_FLAG_SEGMENT_META |
                       ZY100_ONLINE_SPOOL_HEADER_FLAG_RING_GENERATION;
    zy100_put_u16_le(&s_spool.page[14],
                  ZY100_ONLINE_SPOOL_HEADER_EXTENSION_VERSION);
    zy100_put_u32_le(&s_spool.page[16], s_spool.reservation.record_id);
    zy100_put_u32_le(&s_spool.page[20], s_spool.reservation.payload_bytes);
    zy100_put_u32_le(&s_spool.page[24],
                  s_spool.reservation.source_record_bytes);
    zy100_put_u32_le(&s_spool.page[28], source_crc32);
    zy100_put_u32_le(&s_spool.page[32],
                  s_spool.reservation.capture_segment_id);
    zy100_put_u32_le(&s_spool.page[36],
                  s_spool.reservation.segment_start_offset_ms);
    zy100_put_u32_le(&s_spool.page[40], s_spool.reservation.source_id);
    zy100_put_u16_le(&s_spool.page[44],
                  spool_pos_generation(s_spool.commit_pos));
    zy100_put_u16_le(&s_spool.page[46], 0U);
}

static bool spool_validate_header(const uint8_t *page,
                                   zy100_online_spool_record_t *record,
                                   uint32_t expected_addr,
                                   uint16_t expected_generation)
{
    uint32_t source_bytes;
    uint32_t total_bytes;

    if ((page == NULL) ||
        (zy100_get_u32_le(&page[0]) != ZY100_ONLINE_SPOOL_RECORD_MAGIC) ||
        (zy100_get_u16_le(&page[4]) != ZY100_ONLINE_SPOOL_RECORD_VERSION) ||
        (zy100_get_u16_le(&page[6]) != ZY100_ONLINE_SPOOL_RECORD_HEADER_BYTES) ||
        (zy100_get_u32_le(&page[8]) != s_spool.session_id) ||
        (page[12] == 0U) || (page[12] >= ZY100_ONLINE_RECORD_TYPE_COUNT) ||
        ((page[13] & ZY100_ONLINE_SPOOL_HEADER_FLAG_RING_GENERATION) == 0U) ||
        (zy100_get_u16_le(&page[44]) != expected_generation))
    {
        return false;
    }
    if (((page[13] & ZY100_ONLINE_SPOOL_HEADER_FLAG_SEGMENT_META) != 0U) &&
        (zy100_get_u16_le(&page[14]) !=
         ZY100_ONLINE_SPOOL_HEADER_EXTENSION_VERSION))
    {
        return false;
    }
    if (((page[13] & ZY100_ONLINE_SPOOL_HEADER_FLAG_SEGMENT_META) != 0U) &&
        (zy100_get_u32_le(&page[32]) == 0U))
    {
        return false;
    }
    source_bytes = zy100_get_u32_le(&page[24]);
    total_bytes = ZY100_ONLINE_SPOOL_RECORD_HEADER_BYTES + source_bytes;
    if ((source_bytes == 0U) ||
        ((source_bytes % GD25Q32E_PAGE_BYTES) != 0U) ||
        ((expected_addr + total_bytes) > ZY100_ONLINE_SPOOL_END))
    {
        return false;
    }
    if (record != NULL)
    {
        memset(record, 0, sizeof(*record));
        record->type = page[12];
        record->flags = page[13];
        record->generation = zy100_get_u16_le(&page[44]);
        record->record_id = zy100_get_u32_le(&page[16]);
        record->header_addr = expected_addr;
        record->data_addr = expected_addr +
                            ZY100_ONLINE_SPOOL_RECORD_HEADER_BYTES;
        record->record_bytes = total_bytes;
        record->source_record_bytes = source_bytes;
        record->payload_bytes = zy100_get_u32_le(&page[20]);
        record->crc32 = zy100_get_u32_le(&page[28]);
        if ((page[13] & ZY100_ONLINE_SPOOL_HEADER_FLAG_SEGMENT_META) != 0U)
        {
            record->capture_segment_id = zy100_get_u32_le(&page[32]);
            record->segment_start_offset_ms = zy100_get_u32_le(&page[36]);
            record->source_id = zy100_get_u32_le(&page[40]);
        }
    }
    return true;
}

void zy100_online_spool_init(void)
{
    memset(&s_spool, 0, sizeof(s_spool));
    spool_refresh_stats();
}

void zy100_online_spool_ready_begin(void)
{
    uint32_t index;

    zy100_online_spool_abort_session();
    memset(s_spool.erased_bitmap, 0, sizeof(s_spool.erased_bitmap));
    memset(s_spool.dirty_bitmap, 0, sizeof(s_spool.dirty_bitmap));
    memset(s_spool.reclaimable_bitmap, 0,
           sizeof(s_spool.reclaimable_bitmap));
    for (index = 0U; index < ZY100_ONLINE_SPOOL_SECTOR_COUNT; index++)
    {
        spool_bitmap_set(s_spool.reclaimable_bitmap, index);
    }
    s_spool.ready = true;
    s_spool.reserve_pos = 0U;
    s_spool.commit_pos = 0U;
    s_spool.ack_pos = 0U;
    s_spool.send_pos = 0U;
    s_spool.wrap_gap_start = 0U;
    s_spool.wrap_gap_bytes = 0U;
    spool_refresh_stats();
}

bool zy100_online_spool_prepare_start_blocking(uint32_t minimum_bytes,
                                                uint32_t *detail_out)
{
    uint32_t required = minimum_bytes / GD25Q32E_SECTOR_BYTES;
    uint32_t sector;
    uint32_t offset;

    if ((minimum_bytes == 0U) ||
        ((minimum_bytes % GD25Q32E_SECTOR_BYTES) != 0U) ||
        (minimum_bytes > ZY100_ONLINE_SPOOL_REGION_BYTES))
    {
        if (detail_out != NULL)
        {
            *detail_out = 1U;
        }
        return false;
    }
    if (gd25q32e_init() != IMU_STATUS_OK)
    {
        if (detail_out != NULL)
        {
            *detail_out = 2U;
        }
        return false;
    }
    if (s_spool.erase_wip)
    {
        if (gd25q32e_wait_while_busy(s_spool.erase_timeout_ms) !=
            IMU_STATUS_OK)
        {
            s_spool.stats.erase_errors++;
            spool_latch_error(15U, s_spool.erase_sector);
            if (detail_out != NULL)
            {
                *detail_out = 3U;
            }
            return false;
        }
        for (offset = 0U; offset < s_spool.erase_sector_count; offset++)
        {
            sector = s_spool.erase_sector + offset;
            spool_bitmap_set(s_spool.erased_bitmap, sector);
            spool_bitmap_clear(s_spool.dirty_bitmap, sector);
            spool_bitmap_clear(s_spool.reclaimable_bitmap, sector);
        }
        s_spool.erase_wip = false;
        s_spool.stats.erase_count++;
        s_spool.stats.erase_complete_count++;
        if (s_spool.erase_sector_count == ZY100_ONLINE_SPOOL_BLOCK32_SECTORS)
        {
            s_spool.stats.block32_erase_complete_count++;
        }
        s_spool.erase_sector_count = 0U;
        s_spool.erase_timeout_ms = 0U;
    }
    for (sector = 0U; sector < required; sector++)
    {
        if (spool_bitmap_get(s_spool.erased_bitmap, sector))
        {
            continue;
        }
        svc_app_watchdog_poll((uint32_t)(imu_bsp_local_timestamp_us() / 1000ULL));
        if ((gd25q32e_sector_erase_4k(spool_sector_addr(sector)) !=
             IMU_STATUS_OK) ||
            (gd25q32e_wait_while_busy(GD25Q32E_SECTOR_ERASE_TIMEOUT_MS) !=
             IMU_STATUS_OK))
        {
            s_spool.stats.erase_errors++;
            spool_latch_error(6U, sector);
            if (detail_out != NULL)
            {
                *detail_out = 0x100U + sector;
            }
            return false;
        }
        spool_bitmap_set(s_spool.erased_bitmap, sector);
        spool_bitmap_clear(s_spool.dirty_bitmap, sector);
        spool_bitmap_clear(s_spool.reclaimable_bitmap, sector);
        s_spool.stats.erase_count++;
    }
    if (detail_out != NULL)
    {
        *detail_out = 0U;
    }
    spool_refresh_stats();
    return true;
}

bool zy100_online_spool_start_prepare_begin(uint32_t minimum_bytes,
                                             uint32_t *detail_out)
{
    imu_status_t status;

    if ((minimum_bytes == 0U) ||
        ((minimum_bytes % GD25Q32E_SECTOR_BYTES) != 0U) ||
        (minimum_bytes > ZY100_ONLINE_SPOOL_REGION_BYTES))
    {
        if (detail_out != NULL)
        {
            *detail_out = 1U;
        }
        return false;
    }
    status = gd25q32e_init();
    if ((status != IMU_STATUS_OK) && (status != IMU_STATUS_BUS_ERROR))
    {
        if (detail_out != NULL)
        {
            *detail_out = 0x200U | ((uint32_t)status & 0xFFU);
        }
        DBG_DIRECT("[ONLINE_SPOOL][ERR] start_prepare_begin status=%u owner=%u",
                   (uint32_t)status,
                   (uint32_t)spi_bus_current_owner());
        return false;
    }
    /* A transient owner at admission is handled by spool_issue_erase() on
     * subsequent polls. The earlier runtime Flash check already validated
     * the device, so START must not fail only because this instant was busy. */
    s_spool.start_prepare_active = true;
    s_spool.start_prepare_minimum_bytes = minimum_bytes;
    s_spool.start_prepare_erase_errors = s_spool.stats.erase_errors;
    s_spool.start_prepare_timeout_count = s_spool.stats.erase_timeout_count;
    if (detail_out != NULL)
    {
        *detail_out = 0U;
    }
    return true;
}

zy100_online_spool_start_prep_status_t
zy100_online_spool_start_prepare_poll(uint32_t *detail_out)
{
    uint32_t erased;

    if (!s_spool.start_prepare_active)
    {
        return ZY100_ONLINE_SPOOL_START_PREP_IDLE;
    }
    (void)zy100_online_spool_preerase_poll(
        s_spool.start_prepare_minimum_bytes);
    if (s_spool.stats.erase_timeout_count !=
        s_spool.start_prepare_timeout_count)
    {
        if (detail_out != NULL)
        {
            *detail_out = 3U;
        }
        s_spool.start_prepare_active = false;
        return ZY100_ONLINE_SPOOL_START_PREP_ERROR;
    }
    if (s_spool.stats.erase_errors != s_spool.start_prepare_erase_errors)
    {
        if (detail_out != NULL)
        {
            *detail_out = ((s_spool.stats.last_error & 0xFFU) << 8) |
                          (s_spool.stats.last_error_detail & 0xFFU);
        }
        DBG_DIRECT("[ONLINE_SPOOL][ERR] start_prepare erase=%lu base=%lu last=%lu detail=%lu",
                   (unsigned long)s_spool.stats.erase_errors,
                   (unsigned long)s_spool.start_prepare_erase_errors,
                   (unsigned long)s_spool.stats.last_error,
                   (unsigned long)s_spool.stats.last_error_detail);
        s_spool.start_prepare_active = false;
        return ZY100_ONLINE_SPOOL_START_PREP_ERROR;
    }
    erased = zy100_online_spool_erased_ahead_bytes();
    if (!s_spool.erase_wip &&
        (erased >= s_spool.start_prepare_minimum_bytes))
    {
        if (detail_out != NULL)
        {
            *detail_out = 0U;
        }
        s_spool.start_prepare_active = false;
        return ZY100_ONLINE_SPOOL_START_PREP_DONE;
    }
    return ZY100_ONLINE_SPOOL_START_PREP_BUSY;
}

void zy100_online_spool_start_prepare_abort(void)
{
    /* Do not force-idle or cancel an erase already accepted by the Flash. */
    s_spool.start_prepare_active = false;
    s_spool.start_prepare_minimum_bytes = 0U;
}

void zy100_online_spool_begin(uint32_t session_id)
{
    uint32_t erased_bitmap[ZY100_ONLINE_SPOOL_BITMAP_WORDS];
    uint32_t reclaimable_bitmap[ZY100_ONLINE_SPOOL_BITMAP_WORDS];
    uint32_t erase_count = s_spool.stats.erase_count;

    memcpy(erased_bitmap, s_spool.erased_bitmap, sizeof(erased_bitmap));
    memcpy(reclaimable_bitmap,
           s_spool.reclaimable_bitmap,
           sizeof(reclaimable_bitmap));
    memset(&s_spool, 0, sizeof(s_spool));
    memcpy(s_spool.erased_bitmap, erased_bitmap, sizeof(erased_bitmap));
    memcpy(s_spool.reclaimable_bitmap,
           reclaimable_bitmap,
           sizeof(reclaimable_bitmap));
    s_spool.stats.erase_count = erase_count;
    s_spool.ready = true;
    s_spool.session_active = true;
    s_spool.session_id = session_id;
    s_spool.stats.erased_ahead_min_bytes = 0xFFFFFFFFUL;
    s_spool.reserve_pos = 0U;
    s_spool.commit_pos = 0U;
    s_spool.ack_pos = 0U;
    s_spool.send_pos = 0U;
    spool_refresh_stats();
}

void zy100_online_spool_note_capture_segment(uint32_t capture_segment_id,
                                              uint32_t segment_start_offset_ms)
{
    if (!s_spool.session_active || s_spool.reservation.active ||
        (capture_segment_id == 0U))
    {
        return;
    }
    s_spool.capture_segment_id = capture_segment_id;
    s_spool.segment_start_offset_ms = segment_start_offset_ms;
}

void zy100_online_spool_abort_session(void)
{
    if (s_spool.reservation.active)
    {
        spool_provider_abort(NULL, s_spool.reservation.token);
    }
    s_spool.session_active = false;
    s_spool.tx_cached = false;
    s_spool.sent_head = 0U;
    s_spool.sent_count = 0U;
}

bool zy100_online_spool_abort_cleanup_begin(void)
{
    uint32_t sector;

    /* A Flash command already accepted by the device must finish naturally.
     * Never clear the software WIP latch while the part may still be busy. */
    if (s_spool.erase_wip)
    {
        return false;
    }
    zy100_online_spool_abort_session();
    for (sector = 0U; sector < ZY100_ONLINE_SPOOL_SECTOR_COUNT; sector++)
    {
        spool_bitmap_set(s_spool.reclaimable_bitmap, sector);
    }
    s_spool.ready = true;
    s_spool.reserve_pos = 0U;
    s_spool.commit_pos = 0U;
    s_spool.ack_pos = 0U;
    s_spool.send_pos = 0U;
    s_spool.wrap_gap_start = 0U;
    s_spool.wrap_gap_bytes = 0U;
    s_spool.tx_cached = false;
    memset(s_spool.sent_records, 0, sizeof(s_spool.sent_records));
    s_spool.sent_head = 0U;
    s_spool.sent_count = 0U;
    s_spool.stats.pending_records = 0U;
    s_spool.stats.pending_bytes = 0U;
    spool_refresh_stats();
    return true;
}

bool zy100_online_spool_post_session_cleanup_begin(void)
{
    uint32_t sector;

    if (!zy100_online_spool_all_acked() || s_spool.erase_wip)
    {
        return false;
    }
    zy100_online_spool_abort_session();
    for (sector = 0U; sector < ZY100_ONLINE_SPOOL_SECTOR_COUNT; sector++)
    {
        spool_bitmap_set(s_spool.reclaimable_bitmap, sector);
    }
    s_spool.ready = true;
    s_spool.reserve_pos = 0U;
    s_spool.commit_pos = 0U;
    s_spool.ack_pos = 0U;
    s_spool.send_pos = 0U;
    s_spool.wrap_gap_start = 0U;
    s_spool.wrap_gap_bytes = 0U;
    s_spool.tx_cached = false;
    spool_refresh_stats();
    return true;
}

zy100_online_spool_pump_result_t
zy100_online_spool_post_session_cleanup_poll(uint32_t target_bytes)
{
    uint32_t last_error;

#if ZY100_ONLINE_RECOVERY_FAULT_INJECT_ENABLE
    if (s_online_spool_test_cleanup_error)
    {
        return ZY100_ONLINE_SPOOL_PUMP_ERROR;
    }
#endif
    if ((target_bytes == 0U) ||
        ((target_bytes % GD25Q32E_BLOCK32_BYTES) != 0U) ||
        (target_bytes > ZY100_ONLINE_SPOOL_REGION_BYTES))
    {
        return ZY100_ONLINE_SPOOL_PUMP_ERROR;
    }
    if (s_spool.erase_wip)
    {
        last_error = s_spool.stats.last_error;
        (void)spool_poll_erase_wip();
        return (s_spool.stats.last_error != last_error) ?
               ZY100_ONLINE_SPOOL_PUMP_ERROR :
               ZY100_ONLINE_SPOOL_PUMP_PROGRESS;
    }
    if (!s_spool.ready || s_spool.session_active)
    {
        return ZY100_ONLINE_SPOOL_PUMP_ERROR;
    }
    if (!s_spool.erase_wip &&
        (spool_erased_ahead_bytes() >= target_bytes))
    {
        return ZY100_ONLINE_SPOOL_PUMP_IDLE;
    }
    return zy100_online_spool_erase_reclaimable_block32_step();
}

bool zy100_online_spool_post_session_ready(uint32_t minimum_bytes)
{
    return (minimum_bytes != 0U) &&
           ((minimum_bytes % GD25Q32E_SECTOR_BYTES) == 0U) &&
           (minimum_bytes <= ZY100_ONLINE_SPOOL_REGION_BYTES) &&
           s_spool.ready && !s_spool.session_active && !s_spool.erase_wip &&
           (spool_erased_ahead_bytes() >= minimum_bytes);
}

#if ZY100_ONLINE_RECOVERY_FAULT_INJECT_ENABLE
void zy100_online_spool_test_force_cleanup_error(bool enable)
{
    s_online_spool_test_cleanup_error = enable;
}
#endif

const zy100_fe_store_target_provider_t *zy100_online_spool_target_provider(void)
{
    return &s_spool_provider;
}

#if ZY100_ONLINE_STRESS_TEST_ENABLE
zy100_fe_store_reserve_result_t zy100_online_spool_reserve_stress(
    uint32_t source_id, uint32_t source_record_bytes, uint32_t payload_bytes,
    zy100_fe_store_target_t *target)
{
    return spool_provider_reserve(NULL, (zy100_fe_store_target_kind_t)4U,
        source_id, source_record_bytes, payload_bytes, target);
}
#endif

zy100_fe_store_reserve_result_t zy100_online_spool_reserve_raw(
    uint32_t source_id,
    uint32_t source_record_bytes,
    uint32_t payload_bytes,
    zy100_fe_store_target_t *target)
{
    return spool_provider_reserve(NULL,
                                  ZY100_FE_STORE_TARGET_RAW,
                                  source_id,
                                  source_record_bytes,
                                  payload_bytes,
                                  target);
}

zy100_online_spool_io_result_t zy100_online_spool_write_reserved_page(
    uint32_t token,
    uint32_t addr,
    const uint8_t *page)
{
    bool busy = false;
    imu_status_t status;

    if ((page == NULL) || s_spool.reservation.data_done ||
        ((addr % GD25Q32E_PAGE_BYTES) != 0U) ||
        !spool_reservation_range_valid(token,
                                       addr,
                                       GD25Q32E_PAGE_BYTES))
    {
        s_spool.stats.state_errors++;
        spool_latch_error(19U, addr);
        return ZY100_ONLINE_SPOOL_IO_ERROR;
    }
    if (s_spool.erase_wip)
    {
        return ZY100_ONLINE_SPOOL_IO_BUSY;
    }
    status = spool_io_busy(&busy);
    if ((status == IMU_STATUS_BUS_ERROR) || busy)
    {
        return ZY100_ONLINE_SPOOL_IO_BUSY;
    }
    if (status != IMU_STATUS_OK)
    {
        s_spool.stats.write_errors++;
        spool_latch_error(20U, (uint32_t)status);
        return ZY100_ONLINE_SPOOL_IO_ERROR;
    }
    status = spool_io_program(addr, page, GD25Q32E_PAGE_BYTES);
    if (status == IMU_STATUS_BUS_ERROR)
    {
        return ZY100_ONLINE_SPOOL_IO_BUSY;
    }
    if (status != IMU_STATUS_OK)
    {
        s_spool.stats.write_errors++;
        spool_latch_error(21U, (uint32_t)status);
        return ZY100_ONLINE_SPOOL_IO_ERROR;
    }
    s_spool.progress_seq++;
    return ZY100_ONLINE_SPOOL_IO_OK;
}

zy100_online_spool_io_result_t zy100_online_spool_read_reserved(
    uint32_t token,
    uint32_t addr,
    uint8_t *buf,
    uint16_t len)
{
    bool busy = false;
    imu_status_t status;

    if ((buf == NULL) ||
        !spool_reservation_range_valid(token, addr, (uint32_t)len))
    {
        s_spool.stats.state_errors++;
        spool_latch_error(22U, addr);
        return ZY100_ONLINE_SPOOL_IO_ERROR;
    }
    if (s_spool.erase_wip)
    {
        return ZY100_ONLINE_SPOOL_IO_BUSY;
    }
    status = spool_io_busy(&busy);
    if ((status == IMU_STATUS_BUS_ERROR) || busy)
    {
        return ZY100_ONLINE_SPOOL_IO_BUSY;
    }
    if (status != IMU_STATUS_OK)
    {
        s_spool.stats.read_errors++;
        spool_latch_error(23U, (uint32_t)status);
        return ZY100_ONLINE_SPOOL_IO_ERROR;
    }
    status = spool_io_read(1U, addr, buf, len);
    if (status == IMU_STATUS_BUS_ERROR)
    {
        return ZY100_ONLINE_SPOOL_IO_BUSY;
    }
    if (status != IMU_STATUS_OK)
    {
        s_spool.stats.read_errors++;
        spool_latch_error(24U, (uint32_t)status);
        return ZY100_ONLINE_SPOOL_IO_ERROR;
    }
    return ZY100_ONLINE_SPOOL_IO_OK;
}

bool zy100_online_spool_commit_with_crc(uint32_t token,
                                         uint32_t source_crc32)
{
    if (!s_spool.reservation.active || s_spool.reservation.data_done ||
        (s_spool.reservation.token != token) ||
        (s_spool.reservation.commit_state != ZY100_SPOOL_COMMIT_IDLE))
    {
        s_spool.stats.state_errors++;
        spool_latch_error(25U, token);
        return false;
    }
    s_spool.reservation.data_done = true;
    s_spool.reservation.crc_final_valid = true;
    s_spool.reservation.crc32_final = source_crc32;
    s_spool.reservation.commit_state = ZY100_SPOOL_COMMIT_HEADER_READY;
    s_spool.progress_seq++;
    return true;
}

void zy100_online_spool_abort_reservation(uint32_t token)
{
    spool_provider_abort(NULL, token);
}

zy100_online_spool_pump_result_t zy100_online_spool_commit_pump_once(
    bool allow_flash_read)
{
    bool busy = false;
    imu_status_t status;
    uint32_t crc_start_us;
    uint32_t crc_elapsed_us;
    uint32_t remaining;
    uint16_t read_len;
    zy100_online_spool_record_t verify;

    if (!s_spool.reservation.active || !s_spool.reservation.data_done)
    {
        return ZY100_ONLINE_SPOOL_PUMP_IDLE;
    }
    if (spi_bus_current_owner() != SPI_OWNER_NONE)
    {
#if ZY100_STRESS_IO_DIAG_ENABLE
        { uint32_t t = 0U; bool valid = bsp_capture_timebase_snapshot(&t);
          zy100_stress_diag_io_poll(t, valid, true, false, false); }
#endif
        s_spool.stats.tx_read_spi_busy++;
        s_spool.stats.tx_read_busy_retry++;
        return ZY100_ONLINE_SPOOL_PUMP_PROGRESS;
    }
    status = spool_io_busy(&busy);
    if (status == IMU_STATUS_BUS_ERROR)
    {
        s_spool.stats.tx_read_spi_busy++;
        s_spool.stats.tx_read_busy_retry++;
        return ZY100_ONLINE_SPOOL_PUMP_PROGRESS;
    }
    if (status != IMU_STATUS_OK)
    {
        s_spool.stats.read_errors++;
        spool_latch_error(1U, (uint32_t)status);
        return ZY100_ONLINE_SPOOL_PUMP_ERROR;
    }
    if (busy)
    {
        return ZY100_ONLINE_SPOOL_PUMP_PROGRESS;
    }
    if (s_spool.reservation.commit_state == ZY100_SPOOL_COMMIT_CRC)
    {
        if (!allow_flash_read)
        {
            return ZY100_ONLINE_SPOOL_PUMP_IDLE;
        }
        remaining = s_spool.reservation.source_record_bytes -
                    s_spool.reservation.crc_offset;
        read_len = (remaining > GD25Q32E_PAGE_BYTES) ?
                   GD25Q32E_PAGE_BYTES : (uint16_t)remaining;
        crc_start_us = (uint32_t)imu_bsp_local_timestamp_us();
        status = spool_io_read(1U, s_spool.reservation.data_addr +
                               s_spool.reservation.crc_offset,
                               s_spool.page,
                               read_len);
        if (status == IMU_STATUS_BUS_ERROR)
        {
            s_spool.stats.tx_read_spi_busy++;
            s_spool.stats.tx_read_busy_retry++;
            return ZY100_ONLINE_SPOOL_PUMP_PROGRESS;
        }
        if (status != IMU_STATUS_OK)
        {
            s_spool.stats.read_errors++;
            spool_latch_error(7U, s_spool.reservation.crc_offset);
            return ZY100_ONLINE_SPOOL_PUMP_ERROR;
        }
        s_spool.reservation.crc = zy100_crc32_ieee_update(
            s_spool.reservation.crc, s_spool.page, read_len);
        crc_elapsed_us = (uint32_t)imu_bsp_local_timestamp_us() -
                         crc_start_us;
        s_spool.stats.crc_total_us += crc_elapsed_us;
        if (crc_elapsed_us > s_spool.stats.crc_max_us)
        {
            s_spool.stats.crc_max_us = crc_elapsed_us;
        }
        s_spool.reservation.crc_offset += read_len;
        s_spool.stats.crc_pages++;
        s_spool.progress_seq++;
        if (s_spool.reservation.crc_offset >=
            s_spool.reservation.source_record_bytes)
        {
            s_spool.reservation.commit_state =
                ZY100_SPOOL_COMMIT_HEADER_READY;
        }
        return ZY100_ONLINE_SPOOL_PUMP_PROGRESS;
    }
    if (s_spool.reservation.commit_state ==
        ZY100_SPOOL_COMMIT_HEADER_READY)
    {
        if (!spool_skip_gap(&s_spool.commit_pos, false) ||
            (spool_pos_addr(s_spool.commit_pos) !=
             s_spool.reservation.header_addr) ||
            !spool_pos_generation_valid(s_spool.commit_pos))
        {
            s_spool.stats.state_errors++;
            spool_latch_error(28U, s_spool.commit_pos);
            return ZY100_ONLINE_SPOOL_PUMP_ERROR;
        }
        spool_fill_header();
        status = spool_io_program(s_spool.reservation.header_addr,
                                       s_spool.page,
                                       GD25Q32E_PAGE_BYTES);
        if (status == IMU_STATUS_BUS_ERROR)
        {
            s_spool.stats.tx_read_spi_busy++;
            s_spool.stats.tx_read_busy_retry++;
            return ZY100_ONLINE_SPOOL_PUMP_PROGRESS;
        }
        if (status != IMU_STATUS_OK)
        {
            s_spool.stats.write_errors++;
            spool_latch_error(8U, s_spool.reservation.header_addr);
            return ZY100_ONLINE_SPOOL_PUMP_ERROR;
        }
        s_spool.reservation.commit_state =
            ZY100_SPOOL_COMMIT_HEADER_WIP;
        s_spool.stats.header_programs++;
        s_spool.progress_seq++;
        return ZY100_ONLINE_SPOOL_PUMP_PROGRESS;
    }
    if (s_spool.reservation.commit_state ==
        ZY100_SPOOL_COMMIT_HEADER_WIP)
    {
        s_spool.reservation.commit_state =
            ZY100_SPOOL_COMMIT_HEADER_VERIFY;
    }
    if (s_spool.reservation.commit_state ==
        ZY100_SPOOL_COMMIT_HEADER_VERIFY)
    {
        if (!allow_flash_read)
        {
            return ZY100_ONLINE_SPOOL_PUMP_IDLE;
        }
        status = spool_io_read(1U, s_spool.reservation.header_addr,
                               s_spool.page,
                               GD25Q32E_PAGE_BYTES);
        if (status == IMU_STATUS_BUS_ERROR)
        {
            s_spool.stats.tx_read_spi_busy++;
            s_spool.stats.tx_read_busy_retry++;
            return ZY100_ONLINE_SPOOL_PUMP_PROGRESS;
        }
        if ((status != IMU_STATUS_OK) ||
            !spool_validate_header(s_spool.page,
                                   &verify,
                                   s_spool.reservation.header_addr,
                                   spool_pos_generation(s_spool.commit_pos)) ||
            (verify.record_id != s_spool.reservation.record_id))
        {
            if ((status == IMU_STATUS_OK) &&
                (zy100_get_u16_le(&s_spool.page[44]) !=
                 spool_pos_generation(s_spool.commit_pos)))
            {
                s_spool.stats.generation_mismatch++;
            }
            s_spool.stats.read_errors++;
            spool_latch_error(9U, s_spool.reservation.header_addr);
            return ZY100_ONLINE_SPOOL_PUMP_ERROR;
        }
        s_spool.commit_pos += s_spool.reservation.total_bytes;
        if (s_spool.commit_pos > s_spool.reserve_pos)
        {
            s_spool.stats.state_errors++;
            spool_latch_error(29U, s_spool.commit_pos);
            return ZY100_ONLINE_SPOOL_PUMP_ERROR;
        }
        s_spool.stats.pending_bytes += s_spool.reservation.total_bytes;
        s_spool.stats.pending_records++;
        if (s_spool.stats.pending_bytes > s_spool.stats.pending_high_water)
        {
            s_spool.stats.pending_high_water = s_spool.stats.pending_bytes;
        }
        s_spool.stats.committed[s_spool.reservation.type]++;
        s_spool.progress_seq++;
        memset(&s_spool.reservation, 0, sizeof(s_spool.reservation));
        spool_refresh_stats();
        return ZY100_ONLINE_SPOOL_PUMP_COMMITTED;
    }
    s_spool.stats.state_errors++;
    spool_latch_error(10U, s_spool.reservation.commit_state);
    return ZY100_ONLINE_SPOOL_PUMP_ERROR;
}

zy100_online_spool_io_result_t zy100_online_spool_peek_tx(
    zy100_online_spool_record_t *record)
{
    zy100_online_spool_io_result_t ready;
    imu_status_t status;

    if (record == NULL)
    {
        return ZY100_ONLINE_SPOOL_IO_ERROR;
    }
    if (s_spool.tx_cached)
    {
        *record = s_spool.tx_record;
        return ZY100_ONLINE_SPOOL_IO_OK;
    }
    if (!spool_skip_gap_bounded(&s_spool.send_pos,
                                s_spool.commit_pos,
                                false))
    {
        s_spool.stats.state_errors++;
        spool_latch_error(30U, s_spool.send_pos);
        return ZY100_ONLINE_SPOOL_IO_ERROR;
    }
    if ((s_spool.sent_count >= ZY100_ONLINE_RECORD_IN_FLIGHT_MAX) ||
        (s_spool.send_pos >= s_spool.commit_pos))
    {
        return ZY100_ONLINE_SPOOL_IO_EMPTY;
    }
    ready = spool_tx_read_ready();
    if (ready != ZY100_ONLINE_SPOOL_IO_OK)
    {
        return ready;
    }
    status = spool_io_read(2U, spool_pos_addr(s_spool.send_pos),
                           s_spool.page,
                           GD25Q32E_PAGE_BYTES);
    if (status == IMU_STATUS_BUS_ERROR)
    {
        s_spool.stats.tx_read_spi_busy++;
        s_spool.stats.tx_read_busy_retry++;
        return ZY100_ONLINE_SPOOL_IO_BUSY;
    }
    if ((status != IMU_STATUS_OK) ||
        !spool_validate_header(s_spool.page,
                               &s_spool.tx_record,
                               spool_pos_addr(s_spool.send_pos),
                               spool_pos_generation(s_spool.send_pos)))
    {
        if ((status == IMU_STATUS_OK) &&
            (zy100_get_u16_le(&s_spool.page[44]) !=
             spool_pos_generation(s_spool.send_pos)))
        {
            s_spool.stats.generation_mismatch++;
        }
        s_spool.stats.read_errors++;
        spool_latch_error(11U, spool_pos_addr(s_spool.send_pos));
        return ZY100_ONLINE_SPOOL_IO_ERROR;
    }
    s_spool.tx_cached = true;
    *record = s_spool.tx_record;
    return ZY100_ONLINE_SPOOL_IO_OK;
}

bool zy100_online_spool_mark_tx_sent(uint8_t type, uint32_t record_id)
{
    uint8_t tail;
    uint32_t send_end_pos;

    if (!s_spool.tx_cached ||
        (s_spool.tx_record.type != type) ||
        (s_spool.tx_record.record_id != record_id) ||
        (s_spool.sent_count >= ZY100_ONLINE_RECORD_IN_FLIGHT_MAX) ||
        (s_spool.tx_record.header_addr !=
         spool_pos_addr(s_spool.send_pos)) ||
        (s_spool.tx_record.generation !=
         spool_pos_generation(s_spool.send_pos)))
    {
        s_spool.stats.state_errors++;
        spool_latch_error(15U, record_id);
        return false;
    }
    send_end_pos = s_spool.send_pos + s_spool.tx_record.record_bytes;
    if ((send_end_pos < s_spool.send_pos) ||
        (send_end_pos > s_spool.commit_pos))
    {
        s_spool.stats.state_errors++;
        spool_latch_error(16U, send_end_pos);
        return false;
    }
    tail = (uint8_t)((s_spool.sent_head + s_spool.sent_count) %
                     ZY100_ONLINE_RECORD_IN_FLIGHT_MAX);
    s_spool.sent_records[tail] = s_spool.tx_record;
    s_spool.sent_count++;
    s_spool.send_pos = send_end_pos;
    s_spool.tx_cached = false;
    memset(&s_spool.tx_record, 0, sizeof(s_spool.tx_record));
    return true;
}

zy100_online_spool_io_result_t zy100_online_spool_read(uint32_t addr,
                                                        uint8_t *buf,
                                                        uint16_t len)
{
    zy100_online_spool_io_result_t ready;
    imu_status_t status;

    if ((buf == NULL) || (len == 0U) ||
        (addr < ZY100_ONLINE_SPOOL_REGION_BASE_ADDR) ||
        ((addr + len) > ZY100_ONLINE_SPOOL_END))
    {
        return ZY100_ONLINE_SPOOL_IO_ERROR;
    }
    ready = spool_tx_read_ready();
    if (ready != ZY100_ONLINE_SPOOL_IO_OK)
    {
        return ready;
    }
    status = spool_io_read(2U, addr, buf, len);
    if (status == IMU_STATUS_BUS_ERROR)
    {
        s_spool.stats.tx_read_spi_busy++;
        s_spool.stats.tx_read_busy_retry++;
        return ZY100_ONLINE_SPOOL_IO_BUSY;
    }
    if (status != IMU_STATUS_OK)
    {
        s_spool.stats.read_errors++;
        spool_latch_error(12U, addr);
        return ZY100_ONLINE_SPOOL_IO_ERROR;
    }
    return ZY100_ONLINE_SPOOL_IO_OK;
}

static void spool_mark_reclaimed_range(uint32_t old_pos, uint32_t new_pos)
{
    uint32_t boundary;

    if (new_pos <= old_pos)
    {
        return;
    }
    boundary = (old_pos & ~(uint32_t)(GD25Q32E_SECTOR_BYTES - 1U)) +
               GD25Q32E_SECTOR_BYTES;
    while ((boundary != 0U) && (boundary <= new_pos))
    {
        uint32_t sector = spool_pos_offset(boundary - 1U) /
                          GD25Q32E_SECTOR_BYTES;

        if (spool_bitmap_get(s_spool.dirty_bitmap, sector))
        {
            spool_bitmap_set(s_spool.reclaimable_bitmap, sector);
        }
        if (boundary > (0xFFFFFFFFUL - GD25Q32E_SECTOR_BYTES))
        {
            break;
        }
        boundary += GD25Q32E_SECTOR_BYTES;
    }
}

bool zy100_online_spool_ack(uint8_t type, uint32_t record_id)
{
    zy100_online_spool_record_t *sent;
    uint32_t old_ack_pos = s_spool.ack_pos;
    uint32_t record_ack_pos = s_spool.ack_pos;
    uint32_t ack_end_pos;
    bool crossed_gap_before;

    if (type == 0U || type >= ZY100_ONLINE_RECORD_TYPE_COUNT) { return false; }

    if (!spool_skip_gap_bounded(&record_ack_pos,
                                s_spool.commit_pos,
                                false))
    {
        s_spool.stats.ack_errors++;
        spool_latch_error(31U, record_id);
        return false;
    }
    crossed_gap_before = record_ack_pos != old_ack_pos;

    if (s_spool.sent_count == 0U)
    {
        s_spool.stats.ack_errors++;
        spool_latch_error(13U, record_id);
        return false;
    }
    sent = &s_spool.sent_records[s_spool.sent_head];
    if ((sent->type != type) || (sent->record_id != record_id) ||
        (sent->header_addr != spool_pos_addr(record_ack_pos)) ||
        (sent->generation != spool_pos_generation(record_ack_pos)))
    {
        if ((sent->type == type) && (sent->record_id == record_id) &&
            (sent->generation != spool_pos_generation(record_ack_pos)))
        {
            s_spool.stats.generation_mismatch++;
        }
        s_spool.stats.ack_errors++;
        spool_latch_error(13U, record_id);
        return false;
    }
    ack_end_pos = record_ack_pos + sent->record_bytes;
    if ((ack_end_pos < record_ack_pos) ||
        (ack_end_pos > s_spool.commit_pos) ||
        (s_spool.stats.pending_records == 0U))
    {
        s_spool.stats.state_errors++;
        spool_latch_error(14U, ack_end_pos);
        return false;
    }
    if (s_spool.stats.pending_bytes >= sent->record_bytes)
    {
        s_spool.stats.pending_bytes -= sent->record_bytes;
    }
    else
    {
        s_spool.stats.pending_bytes = 0U;
    }
    s_spool.stats.pending_records--;
    s_spool.stats.acked[type]++;
    record_ack_pos = ack_end_pos;
    if (!spool_skip_gap_bounded(&record_ack_pos,
                                s_spool.commit_pos,
                                true))
    {
        s_spool.stats.ack_errors++;
        spool_latch_error(31U, record_id);
        return false;
    }
    if (crossed_gap_before)
    {
        s_spool.wrap_gap_start = 0U;
        s_spool.wrap_gap_bytes = 0U;
    }
    s_spool.ack_pos = record_ack_pos;
    spool_mark_reclaimed_range(old_ack_pos, s_spool.ack_pos);
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    if (type == 4U) { zy100_online_stress_ack(sent->payload_bytes); }
#endif
    memset(sent, 0, sizeof(*sent));
    s_spool.sent_head = (uint8_t)((s_spool.sent_head + 1U) %
                                  ZY100_ONLINE_RECORD_IN_FLIGHT_MAX);
    s_spool.sent_count--;

    spool_refresh_stats();
    return true;
}

bool zy100_online_spool_preerase_poll(uint32_t target_bytes)
{
    uint32_t target_sectors;
    uint32_t sector;

    if (spool_poll_erase_wip())
    {
        return true;
    }
    if ((target_bytes == 0U) ||
        ((target_bytes % GD25Q32E_SECTOR_BYTES) != 0U))
    {
        return false;
    }
    target_sectors = target_bytes / GD25Q32E_SECTOR_BYTES;
    if (target_sectors > ZY100_ONLINE_SPOOL_SECTOR_COUNT)
    {
        target_sectors = ZY100_ONLINE_SPOOL_SECTOR_COUNT;
    }
    for (sector = 0U; sector < target_sectors; sector++)
    {
        if (!spool_bitmap_get(s_spool.erased_bitmap, sector) &&
            (!s_spool.session_active ||
             spool_bitmap_get(s_spool.reclaimable_bitmap, sector)))
        {
            return spool_issue_erase(sector);
        }
    }
    return false;
}

bool zy100_online_spool_erase_reclaimable_poll(void)
{
    uint32_t sector;

    if (spool_poll_erase_wip())
    {
        return true;
    }
    for (sector = 0U; sector < ZY100_ONLINE_SPOOL_SECTOR_COUNT; sector++)
    {
        if (spool_bitmap_get(s_spool.reclaimable_bitmap, sector))
        {
            return spool_issue_erase(sector);
        }
    }
    return false;
}

zy100_online_spool_pump_result_t
zy100_online_spool_erase_reclaimable_step(void)
{
    uint32_t last_error = s_spool.stats.last_error;
    uint32_t sector;

    if (spool_poll_erase_wip())
    {
        return (s_spool.stats.last_error != last_error) ?
               ZY100_ONLINE_SPOOL_PUMP_ERROR :
               ZY100_ONLINE_SPOOL_PUMP_PROGRESS;
    }
    for (sector = 0U; sector < ZY100_ONLINE_SPOOL_SECTOR_COUNT; sector++)
    {
        if (!spool_bitmap_get(s_spool.reclaimable_bitmap, sector))
        {
            continue;
        }
        if (spool_sector_erase_protected(sector))
        {
            s_spool.stats.erase_tx_conflict_prevented++;
            continue;
        }
        if (spool_issue_erase(sector))
        {
            return ZY100_ONLINE_SPOOL_PUMP_PROGRESS;
        }
        if (s_spool.stats.last_error != last_error)
        {
            return ZY100_ONLINE_SPOOL_PUMP_ERROR;
        }
        return ZY100_ONLINE_SPOOL_PUMP_PROGRESS;
    }
    return ZY100_ONLINE_SPOOL_PUMP_IDLE;
}

zy100_online_spool_pump_result_t
zy100_online_spool_erase_reclaimable_block32_step(void)
{
    uint32_t last_error = s_spool.stats.last_error;
    uint32_t first_sector;

    if (spool_poll_erase_wip())
    {
        return (s_spool.stats.last_error != last_error) ?
               ZY100_ONLINE_SPOOL_PUMP_ERROR :
               ZY100_ONLINE_SPOOL_PUMP_PROGRESS;
    }
    for (first_sector = 0U;
         first_sector < ZY100_ONLINE_SPOOL_SECTOR_COUNT;
         first_sector += ZY100_ONLINE_SPOOL_BLOCK32_SECTORS)
    {
        if (!spool_block32_reclaimable(first_sector))
        {
            continue;
        }
        if (spool_block32_erased(first_sector))
        {
            continue;
        }
        if (spool_issue_erase_block32(first_sector))
        {
            return ZY100_ONLINE_SPOOL_PUMP_PROGRESS;
        }
        if (s_spool.stats.last_error != last_error)
        {
            return ZY100_ONLINE_SPOOL_PUMP_ERROR;
        }
        return ZY100_ONLINE_SPOOL_PUMP_PROGRESS;
    }
    return ZY100_ONLINE_SPOOL_PUMP_IDLE;
}

static zy100_online_spool_erase_result_t
spool_erase_forward_block32_step(uint32_t target_bytes, bool return_after_poll)
{
    zy100_online_spool_reserve_plan_t plan;
    zy100_online_spool_reserve_block_t block;
    uint32_t last_error = s_spool.stats.last_error;
    uint32_t used;
    uint32_t free_bytes;
    uint32_t erased_bytes;
    uint32_t target_pos;
    uint32_t target_addr;
    uint32_t first_sector;
    bool next_record_ready;

    if (return_after_poll && s_spool.erase_wip)
    {
        (void)spool_poll_erase_wip();
        return (s_spool.stats.last_error != last_error) ?
               ZY100_ONLINE_SPOOL_ERASE_ERROR :
               (s_spool.erase_wip ? ZY100_ONLINE_SPOOL_ERASE_PROGRESS :
                ZY100_ONLINE_SPOOL_ERASE_IDLE);
    }
    if (spool_poll_erase_wip())
    {
        return (s_spool.stats.last_error != last_error) ?
               ZY100_ONLINE_SPOOL_ERASE_ERROR :
               ZY100_ONLINE_SPOOL_ERASE_PROGRESS;
    }
    if (!s_spool.session_active || s_spool.reservation.active)
    {
        return ZY100_ONLINE_SPOOL_ERASE_BLOCKED;
    }
    next_record_ready = spool_build_reserve_plan(
        ZY100_FE_RAW_IMU_MAG_SOURCE_RECORD_BYTES_MAX,
        &plan,
        &block);
    if (!next_record_ready &&
        (block == ZY100_ONLINE_SPOOL_RESERVE_BLOCK_UNACKED))
    {
        s_spool.last_reserve_block = (uint8_t)block;
        s_spool.stats.capacity_blocked++;
        return ZY100_ONLINE_SPOOL_ERASE_BLOCKED;
    }
    if (!next_record_ready &&
        (block != ZY100_ONLINE_SPOOL_RESERVE_BLOCK_ERASE_PENDING))
    {
        s_spool.stats.state_errors++;
        spool_latch_error(32U, s_spool.reserve_pos);
        return ZY100_ONLINE_SPOOL_ERASE_ERROR;
    }
    if (!spool_used_bytes(&used) ||
        ((used + plan.gap_bytes) > ZY100_ONLINE_SPOOL_REGION_BYTES))
    {
        s_spool.stats.state_errors++;
        spool_latch_error(33U, s_spool.reserve_pos);
        return ZY100_ONLINE_SPOOL_ERASE_ERROR;
    }
    free_bytes = ZY100_ONLINE_SPOOL_REGION_BYTES - used - plan.gap_bytes;
    erased_bytes = spool_erased_from_pos(plan.start_pos, free_bytes);
    if ((erased_bytes >= free_bytes) ||
        (next_record_ready &&
         (erased_bytes >= target_bytes)))
    {
        s_spool.last_reserve_block = ZY100_ONLINE_SPOOL_RESERVE_BLOCK_NONE;
        return ZY100_ONLINE_SPOOL_ERASE_IDLE;
    }
    target_pos = plan.start_pos + erased_bytes;
    target_addr = spool_pos_addr(target_pos);
    first_sector = spool_sector_index(target_addr);
    if (first_sector == ZY100_ONLINE_SPOOL_INVALID_INDEX)
    {
        s_spool.stats.state_errors++;
        spool_latch_error(34U, target_addr);
        return ZY100_ONLINE_SPOOL_ERASE_ERROR;
    }
    first_sector -= first_sector % ZY100_ONLINE_SPOOL_BLOCK32_SECTORS;
    if (spool_block32_erased(first_sector))
    {
        return ZY100_ONLINE_SPOOL_ERASE_IDLE;
    }
    if (!spool_block32_reclaimable(first_sector))
    {
        s_spool.last_reserve_block =
            ZY100_ONLINE_SPOOL_RESERVE_BLOCK_ERASE_PENDING;
        s_spool.stats.erase_blocked++;
        return ZY100_ONLINE_SPOOL_ERASE_BLOCKED;
    }
    if (spool_issue_erase_block32(first_sector))
    {
        s_spool.last_reserve_block =
            ZY100_ONLINE_SPOOL_RESERVE_BLOCK_ERASE_PENDING;
        return ZY100_ONLINE_SPOOL_ERASE_PROGRESS;
    }
    if (s_spool.stats.last_error != last_error)
    {
        return ZY100_ONLINE_SPOOL_ERASE_ERROR;
    }
    return ZY100_ONLINE_SPOOL_ERASE_BLOCKED;
}

zy100_online_spool_erase_result_t
zy100_online_spool_erase_forward_block32_step(void)
{
    return spool_erase_forward_block32_step(ZY100_ONLINE_SPOOL_PREERASE_MIN_BYTES, false);
}

#if ZY100_ONLINE_ERASE_AHEAD_ENABLE
zy100_online_spool_erase_result_t
zy100_online_spool_erase_forward_to(uint32_t target_bytes)
{
    if (target_bytes == 0U || target_bytes > ZY100_ONLINE_SPOOL_REGION_BYTES ||
        target_bytes % GD25Q32E_SECTOR_BYTES != 0U)
    { return ZY100_ONLINE_SPOOL_ERASE_ERROR; }
    return spool_erase_forward_block32_step(target_bytes, true);
}
#endif

bool zy100_online_spool_erase_wip(void)
{
    return s_spool.erase_wip;
}

bool zy100_online_spool_flash_idle_poll(void)
{
    if (s_spool.erase_wip)
    {
        (void)spool_poll_erase_wip();
    }
    return !s_spool.erase_wip;
}

bool zy100_online_spool_commit_pending(void)
{
    return s_spool.reservation.active && s_spool.reservation.data_done;
}

uint8_t zy100_online_spool_active_reservation_kind(void)
{
    return s_spool.reservation.active ? s_spool.reservation.type : 0U;
}

bool zy100_online_spool_active_reservation_data_done(void)
{
    return s_spool.reservation.active && s_spool.reservation.data_done;
}

bool zy100_online_spool_has_committed(void)
{
    uint32_t send_pos = s_spool.send_pos;

    if (!spool_skip_gap_bounded(&send_pos,
                                s_spool.commit_pos,
                                false))
    {
        return false;
    }
    return send_pos < s_spool.commit_pos;
}

bool zy100_online_spool_all_acked(void)
{
    return !s_spool.reservation.active &&
           (s_spool.stats.pending_records == 0U) &&
           (s_spool.ack_pos == s_spool.commit_pos) &&
           (s_spool.send_pos == s_spool.commit_pos) &&
           (s_spool.sent_count == 0U);
}

bool zy100_online_spool_low_water(void)
{
    return spool_erased_ahead_bytes() <
           ZY100_ONLINE_SPOOL_PREERASE_MIN_BYTES;
}

bool zy100_online_spool_has_reclaimable(void)
{
    return spool_bitmap_count(s_spool.reclaimable_bitmap) != 0U;
}

bool zy100_online_spool_try_wrap_empty(uint32_t minimum_bytes)
{
    if ((minimum_bytes == 0U) ||
        ((minimum_bytes % GD25Q32E_PAGE_BYTES) != 0U))
    {
        return false;
    }
    return zy100_online_spool_all_acked() &&
           (spool_erased_ahead_bytes() >= minimum_bytes);
}

zy100_online_spool_reserve_block_t
zy100_online_spool_last_reserve_block(void)
{
    return (zy100_online_spool_reserve_block_t)s_spool.last_reserve_block;
}

uint32_t zy100_online_spool_pending_bytes(void)
{
    return s_spool.stats.pending_bytes;
}

uint32_t zy100_online_spool_erased_ahead_bytes(void)
{
    return spool_erased_ahead_bytes();
}

uint32_t zy100_online_spool_progress_seq(void)
{
    return s_spool.progress_seq;
}

uint8_t zy100_online_spool_invariant_code(void)
{
    uint32_t used;
    uint32_t effective_commit;
    uint32_t effective_ack;
    uint32_t last_sent_pos;
    uint32_t last_sent_end;
    const zy100_online_spool_record_t *last_sent;

    if ((s_spool.ack_pos > s_spool.send_pos) ||
        (s_spool.send_pos > s_spool.commit_pos) ||
        (s_spool.commit_pos > s_spool.reserve_pos) ||
        !spool_used_bytes(&used))
    {
        return 1U;
    }
    if (((s_spool.reserve_pos | s_spool.commit_pos | s_spool.ack_pos |
          s_spool.send_pos) % GD25Q32E_PAGE_BYTES) != 0U)
    {
        return 2U;
    }
    if (!spool_pos_generation_valid(s_spool.reserve_pos) ||
        !spool_pos_generation_valid(s_spool.commit_pos) ||
        !spool_pos_generation_valid(s_spool.ack_pos) ||
        !spool_pos_generation_valid(s_spool.send_pos))
    {
        return 8U;
    }
    if ((s_spool.stats.pending_records == 0U) &&
        ((s_spool.ack_pos != s_spool.commit_pos) ||
         (s_spool.send_pos != s_spool.commit_pos) ||
         (s_spool.sent_count != 0U)))
    {
        return 3U;
    }
    effective_commit = spool_skip_gap_value(s_spool.commit_pos);
    if (s_spool.reservation.active &&
        ((spool_pos_addr(effective_commit) !=
          s_spool.reservation.header_addr) ||
         ((effective_commit + s_spool.reservation.total_bytes) !=
          s_spool.reserve_pos)))
    {
        return 4U;
    }
    if ((s_spool.wrap_gap_bytes != 0U) &&
        (((s_spool.wrap_gap_start | s_spool.wrap_gap_bytes) %
          GD25Q32E_PAGE_BYTES) != 0U ||
         (s_spool.wrap_gap_start < s_spool.ack_pos) ||
         ((s_spool.wrap_gap_start + s_spool.wrap_gap_bytes) >
          s_spool.reserve_pos) ||
         (spool_pos_offset(s_spool.wrap_gap_start +
                           s_spool.wrap_gap_bytes) != 0U)))
    {
        return 9U;
    }
    if ((s_spool.wrap_gap_bytes == 0U) &&
        (s_spool.wrap_gap_start != 0U))
    {
        return 9U;
    }
    if (s_spool.erase_wip &&
        ((s_spool.erase_sector >= ZY100_ONLINE_SPOOL_SECTOR_COUNT) ||
         ((s_spool.erase_sector + s_spool.erase_sector_count) >
          ZY100_ONLINE_SPOOL_SECTOR_COUNT)))
    {
        return 5U;
    }
    if (s_spool.sent_count > ZY100_ONLINE_RECORD_IN_FLIGHT_MAX)
    {
        return 6U;
    }
    effective_ack = spool_skip_gap_value(s_spool.ack_pos);
    if (effective_ack > s_spool.commit_pos)
    {
        effective_ack = s_spool.ack_pos;
    }
    if (s_spool.sent_count == 0U)
    {
        return (effective_ack == s_spool.send_pos) ? 0U : 7U;
    }
    last_sent = &s_spool.sent_records[
        (s_spool.sent_head + s_spool.sent_count - 1U) %
        ZY100_ONLINE_RECORD_IN_FLIGHT_MAX];
    last_sent_pos = ((uint32_t)last_sent->generation *
                     ZY100_ONLINE_SPOOL_REGION_BYTES) +
                    (last_sent->header_addr -
                     ZY100_ONLINE_SPOOL_REGION_BASE_ADDR);
    last_sent_end = last_sent_pos + last_sent->record_bytes;
    if ((s_spool.sent_records[s_spool.sent_head].header_addr !=
         spool_pos_addr(effective_ack)) ||
        (s_spool.sent_records[s_spool.sent_head].generation !=
         spool_pos_generation(effective_ack)) ||
        (last_sent_end != s_spool.send_pos))
    {
        return 7U;
    }
    return 0U;
}

void zy100_online_spool_get_stats(zy100_online_spool_stats_t *out)
{
    if (out == NULL)
    {
        return;
    }
    spool_refresh_stats();
    *out = s_spool.stats;
}

bool zy100_online_spool_clear_begin(void)
{
    if (s_spool.session_active || s_spool.erase_wip)
    {
        return false;
    }
    s_spool.clear_active = true;
    s_spool.clear_next_sector = 0U;
    return true;
}

zy100_flash_prepare_status_t zy100_online_spool_clear_poll(void)
{
    if (!s_spool.clear_active)
    {
        return ZY100_FLASH_PREP_DONE;
    }
    if (spool_poll_erase_wip())
    {
        return ZY100_FLASH_PREP_BUSY;
    }
    if (s_spool.clear_next_sector >= ZY100_ONLINE_SPOOL_SECTOR_COUNT)
    {
        s_spool.clear_active = false;
        s_spool.reserve_pos = 0U;
        s_spool.commit_pos = 0U;
        s_spool.ack_pos = 0U;
        s_spool.send_pos = 0U;
        s_spool.wrap_gap_start = 0U;
        s_spool.wrap_gap_bytes = 0U;
        spool_refresh_stats();
        return ZY100_FLASH_PREP_DONE;
    }
    if (!spool_issue_erase(s_spool.clear_next_sector))
    {
        return ZY100_FLASH_PREP_ERROR;
    }
    s_spool.clear_next_sector++;
    return ZY100_FLASH_PREP_BUSY;
}

#else

void zy100_online_spool_init(void) {}
void zy100_online_spool_ready_begin(void) {}
bool zy100_online_spool_prepare_start_blocking(uint32_t minimum_bytes,
                                                uint32_t *detail_out)
{
    (void)minimum_bytes;
    if (detail_out != NULL) { *detail_out = 0U; }
    return false;
}
bool zy100_online_spool_start_prepare_begin(uint32_t minimum_bytes,
                                             uint32_t *detail_out)
{
    (void)minimum_bytes;
    if (detail_out != NULL) { *detail_out = 1U; }
    return false;
}
zy100_online_spool_start_prep_status_t
zy100_online_spool_start_prepare_poll(uint32_t *detail_out)
{
    if (detail_out != NULL) { *detail_out = 1U; }
    return ZY100_ONLINE_SPOOL_START_PREP_ERROR;
}
void zy100_online_spool_start_prepare_abort(void) {}
void zy100_online_spool_begin(uint32_t session_id) { (void)session_id; }
void zy100_online_spool_abort_session(void) {}
bool zy100_online_spool_post_session_cleanup_begin(void) { return false; }
bool zy100_online_spool_abort_cleanup_begin(void) { return false; }
zy100_online_spool_pump_result_t
zy100_online_spool_post_session_cleanup_poll(uint32_t target_bytes)
{ (void)target_bytes; return ZY100_ONLINE_SPOOL_PUMP_ERROR; }
bool zy100_online_spool_post_session_ready(uint32_t minimum_bytes)
{ (void)minimum_bytes; return false; }
const zy100_fe_store_target_provider_t *zy100_online_spool_target_provider(void)
{
    return NULL;
}
zy100_fe_store_reserve_result_t zy100_online_spool_reserve_raw(
    uint32_t source_id,
    uint32_t source_record_bytes,
    uint32_t payload_bytes,
    zy100_fe_store_target_t *target)
{
    (void)source_id;
    (void)source_record_bytes;
    (void)payload_bytes;
    (void)target;
    return ZY100_FE_STORE_RESERVE_ERROR;
}
zy100_online_spool_io_result_t zy100_online_spool_write_reserved_page(
    uint32_t token, uint32_t addr, const uint8_t *page)
{
    (void)token;
    (void)addr;
    (void)page;
    return ZY100_ONLINE_SPOOL_IO_ERROR;
}
zy100_online_spool_io_result_t zy100_online_spool_read_reserved(
    uint32_t token, uint32_t addr, uint8_t *buf, uint16_t len)
{
    (void)token;
    (void)addr;
    (void)buf;
    (void)len;
    return ZY100_ONLINE_SPOOL_IO_ERROR;
}
bool zy100_online_spool_commit_with_crc(uint32_t token,
                                         uint32_t source_crc32)
{
    (void)token;
    (void)source_crc32;
    return false;
}
void zy100_online_spool_abort_reservation(uint32_t token)
{
    (void)token;
}
zy100_online_spool_pump_result_t zy100_online_spool_commit_pump_once(bool allow)
{
    (void)allow;
    return ZY100_ONLINE_SPOOL_PUMP_IDLE;
}
zy100_online_spool_io_result_t zy100_online_spool_peek_tx(
    zy100_online_spool_record_t *record)
{ (void)record; return ZY100_ONLINE_SPOOL_IO_EMPTY; }
bool zy100_online_spool_mark_tx_sent(uint8_t type, uint32_t id)
{ (void)type; (void)id; return false; }
zy100_online_spool_io_result_t zy100_online_spool_read(
    uint32_t addr, uint8_t *buf, uint16_t len)
{ (void)addr; (void)buf; (void)len; return ZY100_ONLINE_SPOOL_IO_ERROR; }
bool zy100_online_spool_ack(uint8_t type, uint32_t id)
{ (void)type; (void)id; return false; }
bool zy100_online_spool_preerase_poll(uint32_t bytes)
{ (void)bytes; return false; }
bool zy100_online_spool_erase_reclaimable_poll(void) { return false; }
zy100_online_spool_pump_result_t
zy100_online_spool_erase_reclaimable_step(void)
{ return ZY100_ONLINE_SPOOL_PUMP_IDLE; }
zy100_online_spool_pump_result_t
zy100_online_spool_erase_reclaimable_block32_step(void)
{ return ZY100_ONLINE_SPOOL_PUMP_IDLE; }
zy100_online_spool_erase_result_t
zy100_online_spool_erase_forward_block32_step(void)
{ return ZY100_ONLINE_SPOOL_ERASE_IDLE; }
#if ZY100_ONLINE_ERASE_AHEAD_ENABLE
zy100_online_spool_erase_result_t zy100_online_spool_erase_forward_to(uint32_t target_bytes)
{ (void)target_bytes; return ZY100_ONLINE_SPOOL_ERASE_BLOCKED; }
#endif
bool zy100_online_spool_erase_wip(void) { return false; }
bool zy100_online_spool_flash_idle_poll(void) { return true; }
bool zy100_online_spool_commit_pending(void) { return false; }
uint8_t zy100_online_spool_active_reservation_kind(void) { return 0U; }
bool zy100_online_spool_active_reservation_data_done(void) { return false; }
bool zy100_online_spool_has_committed(void) { return false; }
bool zy100_online_spool_all_acked(void) { return true; }
bool zy100_online_spool_low_water(void) { return false; }
bool zy100_online_spool_has_reclaimable(void) { return false; }
bool zy100_online_spool_try_wrap_empty(uint32_t minimum_bytes)
{ (void)minimum_bytes; return false; }
zy100_online_spool_reserve_block_t
zy100_online_spool_last_reserve_block(void)
{ return ZY100_ONLINE_SPOOL_RESERVE_BLOCK_NONE; }
uint32_t zy100_online_spool_pending_bytes(void) { return 0U; }
uint32_t zy100_online_spool_erased_ahead_bytes(void) { return 0U; }
uint32_t zy100_online_spool_progress_seq(void) { return 0U; }
uint8_t zy100_online_spool_invariant_code(void) { return 0U; }
void zy100_online_spool_get_stats(zy100_online_spool_stats_t *out)
{ if (out != NULL) { memset(out, 0, sizeof(*out)); } }
bool zy100_online_spool_clear_begin(void) { return false; }
zy100_flash_prepare_status_t zy100_online_spool_clear_poll(void)
{ return ZY100_FLASH_PREP_DONE; }

#endif
