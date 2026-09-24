#include "zy100_online_stress.h"
#include "zy100_stress_source.h"
#include "zy100_online_spool.h"
#include "zy100_mode_workspace.h"
#include "zy100_crc32.h"
#include "zy100_byteorder.h"
#include "../../src/sample/ble_peripheral/bsp/bsp_capture_timebase.h"
#include "../../src/sample/ble_peripheral/zy100_clock_config.h"
#include <os_sync.h>
#include <string.h>

typedef struct
{
    zy100_stress_source_t source;
    zy100_fe_store_target_t target;
    uint8_t *page;
    uint8_t *verify;
    uint32_t session;
    uint32_t configured_rate;
    uint32_t committed;
    uint32_t acked;
    uint32_t record_bytes;
    uint32_t record_data;
    uint32_t data_left;
    uint32_t page_offset;
    uint32_t page_data;
    uint32_t crc;
    uint32_t cache_wait;
    uint32_t flash_wait;
    uint32_t send_wait;
    uint32_t ack_wait;
    uint32_t publish_ms;
    uint32_t skipped;
    uint32_t first_failure;
    uint32_t build_max_us;
    uint32_t program_max_us;
    uint32_t verify_max_us;
    uint32_t record_max_ms;
    uint32_t record_start_ms;
    uint32_t snapshot[ZY100_STRESS_STATUS_WORDS];
    uint8_t conn;
    uint8_t phase; /* 0 idle, 1 write, 2 readback, 3 commit */
    bool io_waiting;
    bool queried;
    bool negotiated;
    bool armed;
    bool configured;
    bool prepared;
    bool status_pending;
    bool terminal_frozen;
} online_stress_t;

static online_stress_t s_stress;
/* Includes type counters, alignment, and the retry hint's measured Spool growth. */
typedef char stress_control_ram_budget[(sizeof(s_stress) + 44U + 4U +
#if ZY100_ONLINE_PAGE_RETRY_ENABLE
    8U +
#endif
    ZY100_STRESS_DIAG_RAM_LIMIT <= 1024U) ? 1 : -1];
typedef char stress_workspace_budget[
    (ZY100_ONLINE_WORKSPACE_REQUIRED_BYTES + ZY100_STRESS_RING_BYTES ==
     ZY100_MODE_WORKSPACE_TRANSIENT_BYTES) ? 1 : -1];

static bool fail(uint8_t error)
{
    if (s_stress.source.error == 0U) { s_stress.source.error = error; }
    if (s_stress.first_failure == 0U) { s_stress.first_failure = s_stress.source.error; }
    s_stress.source.frozen = true;
    zy100_online_stress_diag_freeze(s_stress.first_failure);
    return false;
}

static uint32_t maximum(uint32_t a, uint32_t b) { return a > b ? a : b; }

static void profile(uint32_t *peak, uint32_t start, bool valid)
{
    uint32_t end;
    if (valid && bsp_capture_timebase_snapshot(&end))
    { *peak = maximum(*peak, start - end); }
}

bool zy100_online_stress_command(uint8_t conn, uint32_t operation,
                                  uint64_t argument, uint32_t *detail)
{
    if (detail == NULL) { return false; }
    if (operation == 0U && argument == 0ULL)
    {
        if (s_stress.conn != conn) { zy100_online_stress_invalidate(); }
        s_stress.conn = conn;
        s_stress.queried = true;
        *detail = ZY100_STRESS_QUERY_DETAIL;
        return true;
    }
    if (operation == 1U && s_stress.queried && s_stress.conn == conn &&
        (uint32_t)(argument >> 32U) == ZY100_STRESS_PROTOCOL_VERSION &&
        zy100_stress_rate_valid((uint32_t)argument))
    {
        s_stress.configured_rate = (uint32_t)argument;
        s_stress.armed = true;
        *detail = s_stress.configured_rate;
        return true;
    }
    return false;
}

void zy100_online_stress_negotiate(uint8_t conn, uint32_t capabilities)
{
    if (s_stress.conn != conn) { zy100_online_stress_invalidate(); }
    s_stress.conn = conn;
    s_stress.negotiated = (capabilities & 0x20U) != 0U;
}

bool zy100_online_stress_armed(uint8_t conn)
{
    return s_stress.conn == conn && s_stress.queried &&
           s_stress.armed && s_stress.negotiated;
}

void zy100_online_stress_invalidate(void)
{
    s_stress.queried = false;
    s_stress.armed = false;
    s_stress.negotiated = false;
}

void zy100_online_stress_session(uint32_t session)
{
    uint32_t rate = s_stress.configured_rate;
    bool configured = s_stress.armed && s_stress.queried && s_stress.negotiated;
    memset(&s_stress, 0, sizeof(s_stress));
    s_stress.session = session;
    s_stress.configured_rate = rate;
    s_stress.configured = configured;
    s_stress.snapshot[0] = session;
    s_stress.snapshot[1] = rate;
    s_stress.snapshot[34] = 0xFFFFFFFFUL;
    s_stress.snapshot[35] = 0xFFFFFFFFUL;
    s_stress.snapshot[36] = 0xFFFFFFFFUL;
}

bool zy100_online_stress_begin(uint8_t *workspace, uint32_t now_ms)
{
    if (!s_stress.configured || workspace == NULL) { return fail(ZY100_STRESS_OWNER); }
    s_stress.page = workspace + ZY100_ONLINE_WORKSPACE_PAGE_OFFSET;
    s_stress.verify = workspace + ZY100_ONLINE_WORKSPACE_VERIFY_OFFSET;
    s_stress.publish_ms = now_ms;
    {
        zy100_online_spool_stats_t stats;
        zy100_online_spool_get_stats(&stats);
        zy100_stress_diag_begin(s_stress.session, now_ms, s_stress.configured_rate, stats.erase_count);
    }
    return zy100_stress_source_begin(&s_stress.source,
        workspace + ZY100_ONLINE_WORKSPACE_REQUIRED_BYTES,
        s_stress.configured_rate, s_stress.session, now_ms);
}

bool zy100_online_stress_tick(uint32_t now_ms)
{
    zy100_stress_diag_pump(now_ms);
    if (s_stress.source.used != 0U) { s_stress.cache_wait++; }
    if (!zy100_stress_source_tick(&s_stress.source, now_ms))
    { return fail(s_stress.source.error); }
    zy100_stress_diag_water(now_ms, s_stress.source.used);
    return true;
}

bool zy100_online_stress_freeze(uint32_t now_ms)
{
    if (!zy100_stress_source_freeze(&s_stress.source, now_ms))
    { return fail(s_stress.source.error); }
    return true;
}

bool zy100_online_stress_pending(void)
{
    return s_stress.source.used != 0U || s_stress.phase != 0U;
}

bool zy100_online_stress_writer_busy(void) { return s_stress.phase != 0U; }

void zy100_online_stress_diag_stage(zy100_stress_diag_stage_t stage)
{
    zy100_stress_diag_stage(stage, zy100_os_time_ms(), s_stress.source.used);
}

void zy100_online_stress_diag_freeze(uint32_t error)
{
    zy100_stress_diag_freeze(zy100_os_time_ms(), s_stress.source.used, error);
}

uint32_t zy100_online_stress_writer_cursor(void)
{
    return s_stress.page_offset + s_stress.phase;
}

bool zy100_online_stress_io_waiting(void) { return s_stress.io_waiting; }

bool zy100_online_stress_erase_window(void)
{
    return s_stress.source.used <= s_stress.source.rate / 50U;
}

bool zy100_online_stress_write_step(void)
{
    zy100_online_spool_io_result_t io;
    zy100_fe_store_reserve_result_t reserve;
    zy100_online_spool_pump_result_t commit;
    uint32_t block_bytes, blocks, header_bytes, addr, stamp;
    bool timed;
    s_stress.io_waiting = false;
    if (s_stress.source.error != 0U) { return false; }
    if (s_stress.phase == 0U)
    {
        block_bytes = s_stress.source.rate / 50U;
        if (block_bytes == 0U) { zy100_online_stress_diag_stage(ZY100_SD_OTHER); return true; }
        blocks = s_stress.source.used / block_bytes;
        if (blocks < 2U && !s_stress.source.frozen)
        { zy100_online_stress_diag_stage(ZY100_SD_PAIR); return true; }
        if (blocks == 0U) { zy100_online_stress_diag_stage(ZY100_SD_OTHER); return true; }
        if (blocks > 2U) { blocks = 2U; }
        s_stress.record_data = blocks * block_bytes;
        s_stress.record_bytes = (ZY100_STRESS_HEADER_BYTES +
            s_stress.record_data + 255U) & ~255UL;
        zy100_online_stress_diag_stage(ZY100_SD_RESERVE);
        reserve = zy100_online_spool_reserve_stress(
            s_stress.source.consumed / block_bytes + 1U,
            s_stress.record_bytes, s_stress.record_data, &s_stress.target);
        if (reserve == ZY100_FE_STORE_RESERVE_ERROR) { return fail(ZY100_STRESS_RESERVE); }
        if (reserve != ZY100_FE_STORE_RESERVE_OK)
        {
            s_stress.io_waiting = true;
            s_stress.flash_wait++;
            if (reserve == ZY100_FE_STORE_RESERVE_FULL)
            {
                zy100_online_spool_erase_result_t er = zy100_online_spool_erase_forward_block32_step();
#if ZY100_ONLINE_ERASE_AHEAD_ENABLE
                zy100_stress_diag_erase(true, zy100_online_spool_erase_wip(),
                                        zy100_online_spool_erased_ahead_bytes());
#endif
                if (er == ZY100_ONLINE_SPOOL_ERASE_ERROR) { return fail(ZY100_STRESS_RESERVE); }
            }
            return true;
        }
        s_stress.data_left = s_stress.record_data;
        s_stress.record_start_ms = zy100_os_time_ms();
        zy100_stress_diag_record_begin(4U, s_stress.record_start_ms, s_stress.record_bytes / 256U);
        s_stress.page_offset = 0U;
        s_stress.crc = zy100_crc32_ieee_begin();
        s_stress.prepared = false;
        s_stress.phase = 1U;
    }
    if (s_stress.phase == 3U)
    {
        zy100_online_stress_diag_stage(ZY100_SD_COMMIT);
        commit = zy100_online_spool_commit_pump_once(true);
        if (commit == ZY100_ONLINE_SPOOL_PUMP_ERROR) { return fail(ZY100_STRESS_COMMIT); }
        if (commit == ZY100_ONLINE_SPOOL_PUMP_COMMITTED)
        {
            zy100_stress_diag_record_end(zy100_os_time_ms());
            s_stress.committed += s_stress.record_data;
            s_stress.record_max_ms = maximum(s_stress.record_max_ms,
                zy100_os_time_ms() - s_stress.record_start_ms);
            s_stress.phase = 0U;
            s_stress.target.token = 0U;
        }
        else { s_stress.flash_wait++; }
        return true;
    }
    addr = s_stress.target.data_addr + s_stress.page_offset;
    if (s_stress.phase == 2U)
    {
        zy100_online_stress_diag_stage(ZY100_SD_VERIFY);
        timed = bsp_capture_timebase_snapshot(&stamp);
        io = zy100_online_spool_read_reserved(s_stress.target.token,
            addr, s_stress.verify, 256U);
        if (io == ZY100_ONLINE_SPOOL_IO_BUSY) { s_stress.io_waiting = true; s_stress.flash_wait++; return true; }
        if (io != ZY100_ONLINE_SPOOL_IO_OK ||
            memcmp(s_stress.page, s_stress.verify, 256U) != 0)
        { return fail(ZY100_STRESS_VERIFY); }
        profile(&s_stress.verify_max_us, stamp, timed);
        s_stress.crc = zy100_crc32_ieee_update(s_stress.crc, s_stress.page, 256U);
        /* Only verified bytes are reusable. A later commit failure is fatal. */
        if (!zy100_stress_source_consume(&s_stress.source, s_stress.page_data))
        { return fail(ZY100_STRESS_OWNER); }
        zy100_stress_diag_release(zy100_os_time_ms(),
            s_stress.source.used + s_stress.page_data, s_stress.page_data);
        s_stress.data_left -= s_stress.page_data;
        s_stress.page_offset += 256U;
        zy100_stress_diag_record_page(s_stress.page_offset / 256U);
        s_stress.prepared = false;
        s_stress.phase = 1U;
        if (s_stress.page_offset == s_stress.record_bytes)
        {
            if (!zy100_online_spool_commit_with_crc(s_stress.target.token,
                zy100_crc32_ieee_finish(s_stress.crc))) { return fail(ZY100_STRESS_COMMIT); }
            s_stress.phase = 3U;
        }
        return true;
    }
    zy100_online_stress_diag_stage(ZY100_SD_PROGRAM);
    if (!s_stress.prepared)
    {
        timed = bsp_capture_timebase_snapshot(&stamp);
        header_bytes = s_stress.page_offset == 0U ? ZY100_STRESS_HEADER_BYTES : 0U;
        memset(s_stress.page, 0xFF, 256U);
        if (header_bytes != 0U && !zy100_stress_source_header(&s_stress.source,
            s_stress.page, s_stress.record_data / (s_stress.source.rate / 50U)))
        { return fail(ZY100_STRESS_OWNER); }
        s_stress.page_data = 256U - header_bytes;
        if (s_stress.page_data > s_stress.data_left) { s_stress.page_data = s_stress.data_left; }
        if (!zy100_stress_source_copy(&s_stress.source,
            s_stress.page + header_bytes, s_stress.page_data)) { return fail(ZY100_STRESS_OWNER); }
        s_stress.prepared = true;
        profile(&s_stress.build_max_us, stamp, timed);
    }
    timed = bsp_capture_timebase_snapshot(&stamp);
    io = zy100_online_spool_write_reserved_page(s_stress.target.token, addr, s_stress.page);
    if (io == ZY100_ONLINE_SPOOL_IO_BUSY) { s_stress.io_waiting = true; s_stress.flash_wait++; return true; }
    if (io != ZY100_ONLINE_SPOOL_IO_OK) { return fail(ZY100_STRESS_PROGRAM); }
    profile(&s_stress.program_max_us, stamp, timed);
    s_stress.phase = 2U;
    return true;
}

void zy100_online_stress_release(bool discard)
{
    zy100_online_stress_diag_freeze(s_stress.first_failure != 0U ?
        s_stress.first_failure : (discard ? 0x105U : 0U));
    if (discard && s_stress.target.token != 0U)
    { zy100_online_spool_abort_reservation(s_stress.target.token); }
    s_stress.target.token = 0U;
    s_stress.phase = 0U;
    s_stress.source.ring = NULL;
    s_stress.page = NULL;
    s_stress.verify = NULL;
    zy100_online_stress_invalidate();
}

void zy100_online_stress_publish(uint32_t now_ms,
    const zy100_online_raw_capture_stats_t *raw, bool force)
{
    uint32_t lock, periods = (uint32_t)(now_ms - s_stress.publish_ms) / 1000U;
    uint32_t *p = s_stress.snapshot;
    if (!force && periods == 0U) { return; }
    lock = os_lock();
    if (s_stress.status_pending) { s_stress.skipped++; }
    if (periods > 1U) { s_stress.skipped += periods - 1U; }
    s_stress.publish_ms += periods * 1000U;
    p[0] = s_stress.session; p[1] = s_stress.source.rate;
    p[2] = s_stress.source.elapsed_ms; p[3] = s_stress.source.expected;
    p[4] = s_stress.source.generated; p[5] = s_stress.source.rejected;
    p[6] = s_stress.committed; p[7] = s_stress.acked;
    p[8] = s_stress.source.used; p[9] = s_stress.source.peak;
    p[10] = s_stress.source.delay_max_ms; p[11] = s_stress.cache_wait;
    p[12] = s_stress.flash_wait; p[13] = s_stress.send_wait;
    p[14] = s_stress.ack_wait; p[15] = s_stress.first_failure;
    p[22] = s_stress.skipped;
    if (raw != NULL)
    {
        p[23] = raw->accepted_packets; p[24] = raw->accepted_mag_samples;
        p[25] = raw->mag_missed_deadlines; p[26] = raw->mag_lateness_max_us;
        p[27] = raw->mag_interval_max_us; p[28] = raw->page_build_max_us;
        p[29] = raw->page_program_max_us; p[30] = raw->page_verify_max_us;
        p[31] = raw->record_write_max_ms;
    }
    /* Maxima cover both the unchanged RAW writer and the diagnostic writer. */
    p[28] = maximum(p[28], s_stress.build_max_us);
    p[29] = maximum(p[29], s_stress.program_max_us);
    p[30] = maximum(p[30], s_stress.verify_max_us);
    p[31] = maximum(p[31], s_stress.record_max_ms);
    s_stress.status_pending = true;
    os_unlock(lock);
}

bool zy100_online_stress_status_due(void) { return s_stress.status_pending; }
void zy100_online_stress_status_sent(uint32_t elapsed_ms)
{
    if (elapsed_ms == s_stress.snapshot[2]) { s_stress.status_pending = false; }
}
void zy100_online_stress_ack(uint32_t bytes) { s_stress.acked += bytes; }
void zy100_online_stress_note_wait(bool send_wait, bool ack_wait)
{
    if (send_wait) { s_stress.send_wait++; }
    if (ack_wait) { s_stress.ack_wait++; }
}
void zy100_online_stress_note_stacks(uint32_t app_stack, uint32_t worker_stack,
                                     uint32_t worker_gap)
{
    if (s_stress.terminal_frozen) { return; }
    if (app_stack != 0xFFFFFFFFUL) { s_stress.snapshot[34] = app_stack; }
    if (worker_stack != 0xFFFFFFFFUL) { s_stress.snapshot[35] = worker_stack; }
    if (worker_gap != 0xFFFFFFFFUL) { s_stress.snapshot[36] = worker_gap; }
}

void zy100_online_stress_transport(uint32_t stop_reason, uint32_t pending,
    uint32_t pending_hi, uint32_t ack_max, uint32_t erase_count,
    uint32_t wrap_count, uint32_t heap_data, uint32_t heap_buffer,
    uint32_t mtu, uint32_t ci, uint32_t phy)
{
    uint32_t *p = s_stress.snapshot;
    if (s_stress.terminal_frozen) { return; }
    p[15] = s_stress.first_failure;
    p[7] = s_stress.acked; p[13] = s_stress.send_wait; p[14] = s_stress.ack_wait;
    p[16] = stop_reason; p[17] = pending; p[18] = pending_hi; p[19] = ack_max;
    p[20] = zy100_stress_diag_erases(erase_count); p[21] = wrap_count;
    p[32] = heap_data; p[33] = heap_buffer;
    p[37] = mtu; p[38] = ci; p[39] = phy;
}

void zy100_online_stress_encode(uint8_t *dst)
{
    uint32_t i, lock = os_lock();
    zy100_put_u16_le(dst, ZY100_STRESS_PROTOCOL_VERSION);
    zy100_put_u16_le(dst + 2U, ZY100_STRESS_STATUS_BYTES);
    for (i = 0U; i < ZY100_STRESS_STATUS_WORDS; ++i)
    { zy100_put_u32_le(dst + 4U + i * 4U, s_stress.snapshot[i]); }
    os_unlock(lock);
}

void zy100_online_stress_terminal_freeze(void)
{
    s_stress.terminal_frozen = true;
    s_stress.status_pending = false;
}

void zy100_online_stress_failure(uint32_t stop_reason)
{
    if (s_stress.first_failure == 0U)
    {
        s_stress.first_failure = s_stress.source.error != 0U ?
            s_stress.source.error : (0x100U + stop_reason);
    }
}
