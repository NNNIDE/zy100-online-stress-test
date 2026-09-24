#include "zy100_stress_diag.h"
#include "../../src/sample/ble_peripheral/app_flags.h"
#include "trace.h"
#include "version.h"
#include "../../src/sample/ble_peripheral/zy100_clock_config.h"
#include <string.h>
#if ZY100_STRESS_IO_DIAG_ENABLE
#include "../../src/sample/ble_peripheral/bsp/bsp_capture_timebase.h"
#endif

#if ZY100_STRESS_IO_DIAG_ENABLE
#define SD_IO_PENDING 1U
#define SD_IO_OVERFLOW 2U
#define SD_IO_CLOCK 4U
#define SD_IO_POLLED 8U
#define SD_IO_BUS 16U
#define SD_IO_WIP 32U
typedef struct
{
    uint32_t metric[4][3]; /* calls, total us, peak us */
    uint32_t bus_count, wip_count;
    uint32_t issue, last_poll, spans, span_total, span_max, poll_max;
    uint32_t open_age, flags, bus_wait, wip_wait;
} sd_io_t;
typedef char io_ram_budget[(sizeof(sd_io_t) <= 96U) ? 1 : -1];
#endif

#define SD_DEPTH 12U
typedef struct
{
    uint32_t ms;
    uint16_t water;
    uint8_t stage;
    uint8_t reserved;
} sd_event_t;
typedef struct
{
    uint32_t total[ZY100_SD_COUNT], peak[ZY100_SD_COUNT];
    uint32_t gap_parts[ZY100_SD_COUNT], best_parts[ZY100_SD_COUNT];
    sd_event_t events[SD_DEPTH];
    uint32_t session, start, rate, prepare_base, erase_base, prepare_erases;
    uint32_t last, stage_since, pump_last, pump_max;
    uint32_t gap_start, gap_water, best_start, best_end;
    uint32_t best_water_start, best_water_end, best_freed;
    uint32_t water, released, record_start, record_kind, record_page, record_pages;
    uint32_t raw_max, stress_max, end, error, transitions;
#if ZY100_ONLINE_BOUNDED_PUMP_ENABLE || ZY100_ONLINE_ERASE_AHEAD_ENABLE
    uint32_t service_calls, service_steps, service_peak_us, service_limits;
    uint32_t service_waits, service_clock_missing, erase_background, erase_urgent;
    uint32_t erase_ahead_min, prepare_ms;
#endif
#if ZY100_STRESS_IO_DIAG_ENABLE
    sd_io_t io;
#endif
    uint8_t stage, next, count;
    bool active, frozen, gap_active, pump_seen, prepared, best_open;
} sd_state_t;
static sd_state_t s_diag;
typedef char diag_ram_budget[(sizeof(s_diag) <= ZY100_STRESS_DIAG_RAM_LIMIT) ? 1 : -1];

static uint32_t sd_max(uint32_t a, uint32_t b) { return a > b ? a : b; }
#if ZY100_STRESS_IO_DIAG_ENABLE
static void sd_io_add(uint32_t *p, uint32_t n)
{
    if (n > 0xFFFFFFFFUL - *p) { *p = 0xFFFFFFFFUL; s_diag.io.flags |= SD_IO_OVERFLOW; }
    else { *p += n; }
}
void zy100_stress_diag_io_note(uint32_t kind, uint32_t elapsed, bool valid)
{
    if (!s_diag.active || s_diag.frozen || kind >= 4U) { return; }
    sd_io_add(&s_diag.io.metric[kind][0], 1U);
    if (!valid) { s_diag.io.flags |= SD_IO_CLOCK; return; }
    sd_io_add(&s_diag.io.metric[kind][1], elapsed);
    s_diag.io.metric[kind][2] = sd_max(s_diag.io.metric[kind][2], elapsed);
}
void zy100_stress_diag_io_issue(uint32_t counter, bool valid)
{
    if (!s_diag.active || s_diag.frozen) { return; }
    if (!valid) { s_diag.io.flags |= SD_IO_CLOCK; return; }
    s_diag.io.issue = s_diag.io.last_poll = counter;
    s_diag.io.flags = (s_diag.io.flags & (SD_IO_OVERFLOW | SD_IO_CLOCK)) | SD_IO_PENDING;
}
void zy100_stress_diag_io_bus(void)
{
    if (s_diag.active && !s_diag.frozen) { sd_io_add(&s_diag.io.bus_count, 1U); }
}
void zy100_stress_diag_io_poll(uint32_t counter, bool valid, bool bus, bool wip, bool ok)
{
    uint32_t delta;
    sd_io_t *io = &s_diag.io;
    if (!s_diag.active || s_diag.frozen) { return; }
    if (bus) { sd_io_add(&io->bus_count, 1U); }
    if (ok && wip) { sd_io_add(&io->wip_count, 1U); }
    if (!valid) { io->flags |= SD_IO_CLOCK; return; }
    if (!(io->flags & SD_IO_PENDING)) { return; }
    delta = io->last_poll - counter;
    io->poll_max = sd_max(io->poll_max, delta);
    if (io->flags & SD_IO_BUS) { sd_io_add(&io->bus_wait, delta); }
    if (io->flags & SD_IO_WIP) { sd_io_add(&io->wip_wait, delta); }
    io->last_poll = counter;
    io->flags &= ~(SD_IO_BUS | SD_IO_WIP);
    io->flags |= SD_IO_POLLED | (bus ? SD_IO_BUS : 0U) | (ok && wip ? SD_IO_WIP : 0U);
    if (ok && !wip)
    {
        delta = io->issue - counter;
        sd_io_add(&io->spans, 1U); sd_io_add(&io->span_total, delta);
        io->span_max = sd_max(io->span_max, delta);
        io->flags &= ~(SD_IO_PENDING | SD_IO_POLLED | SD_IO_BUS | SD_IO_WIP);
    }
}
#endif

static void sd_account(uint32_t now)
{
    uint32_t delta = now - s_diag.last;
    s_diag.total[s_diag.stage] += delta;
    s_diag.peak[s_diag.stage] = sd_max(s_diag.peak[s_diag.stage], now - s_diag.stage_since);
    if (s_diag.gap_active) { s_diag.gap_parts[s_diag.stage] += delta; }
    s_diag.last = now;
}
static void sd_event(uint32_t now)
{
    sd_event_t *e = &s_diag.events[s_diag.next];
    e->ms = now - s_diag.start;
    e->water = (uint16_t)s_diag.water;
    e->stage = s_diag.stage;
    s_diag.next = (s_diag.next + 1U) % SD_DEPTH;
    if (s_diag.count < SD_DEPTH) { s_diag.count++; }
    s_diag.transitions++;
}
static void sd_close_gap(uint32_t now, uint32_t before, uint32_t freed, bool open)
{
    if (s_diag.gap_active &&
        (now - s_diag.gap_start >= s_diag.best_end - s_diag.best_start))
    {
        s_diag.best_start = s_diag.gap_start - s_diag.start;
        s_diag.best_end = now - s_diag.start;
        s_diag.best_water_start = s_diag.gap_water;
        s_diag.best_water_end = before;
        s_diag.best_freed = freed;
        s_diag.best_open = open;
        memcpy(s_diag.best_parts, s_diag.gap_parts, sizeof(s_diag.best_parts));
    }
}
void zy100_stress_diag_prepare(uint32_t count)
{
#if ZY100_ONLINE_BOUNDED_PUMP_ENABLE || ZY100_ONLINE_ERASE_AHEAD_ENABLE
    s_diag.prepare_ms = zy100_os_time_ms();
#endif
    s_diag.prepare_base = count;
    s_diag.prepared = true;
}
void zy100_stress_diag_begin(uint32_t session, uint32_t now, uint32_t rate, uint32_t count)
{
    uint32_t prep = s_diag.prepared ? count - s_diag.prepare_base : 0xFFFFFFFFUL;
#if ZY100_ONLINE_BOUNDED_PUMP_ENABLE || ZY100_ONLINE_ERASE_AHEAD_ENABLE
    uint32_t prep_ms = s_diag.prepared ? now - s_diag.prepare_ms : 0xFFFFFFFFUL;
#endif
    memset(&s_diag, 0, sizeof(s_diag));
#if ZY100_ONLINE_BOUNDED_PUMP_ENABLE || ZY100_ONLINE_ERASE_AHEAD_ENABLE
    s_diag.prepare_ms = prep_ms; s_diag.erase_ahead_min = 0xFFFFFFFFUL;
#endif
    s_diag.session = session; s_diag.start = now; s_diag.rate = rate;
    s_diag.last = s_diag.stage_since = now;
    s_diag.pump_last = now; s_diag.pump_seen = true;
    s_diag.erase_base = count; s_diag.prepare_erases = prep;
    s_diag.active = true;
    sd_event(now);
}
void zy100_stress_diag_water(uint32_t now, uint32_t water)
{
    if (!s_diag.active || s_diag.frozen) { return; }
    sd_account(now);
    s_diag.water = water;
    if (!s_diag.gap_active && water != 0U)
    {
        s_diag.gap_active = true; s_diag.gap_start = now; s_diag.gap_water = water;
        memset(s_diag.gap_parts, 0, sizeof(s_diag.gap_parts));
    }
}
void zy100_stress_diag_stage(zy100_stress_diag_stage_t stage, uint32_t now, uint32_t water)
{
    if (!s_diag.active || s_diag.frozen || stage >= ZY100_SD_COUNT) { return; }
    zy100_stress_diag_water(now, water);
    if (stage != s_diag.stage)
    {
        s_diag.stage = (uint8_t)stage; s_diag.stage_since = now; sd_event(now);
    }
}
void zy100_stress_diag_pump(uint32_t now)
{
    if (!s_diag.active || s_diag.frozen) { return; }
    if (s_diag.pump_seen) { s_diag.pump_max = sd_max(s_diag.pump_max, now - s_diag.pump_last); }
    s_diag.pump_last = now; s_diag.pump_seen = true;
}
void zy100_stress_diag_release(uint32_t now, uint32_t before, uint32_t bytes)
{
    if (!s_diag.active || s_diag.frozen || bytes == 0U || bytes > before) { return; }
    zy100_stress_diag_water(now, before);
    sd_close_gap(now, before, bytes, false);
    s_diag.released += bytes; s_diag.water = before - bytes;
    s_diag.gap_active = s_diag.water != 0U;
    s_diag.gap_start = now; s_diag.gap_water = s_diag.water;
    memset(s_diag.gap_parts, 0, sizeof(s_diag.gap_parts));
}
void zy100_stress_diag_record_begin(uint32_t kind, uint32_t now, uint32_t pages)
{
    if (!s_diag.active || s_diag.frozen) { return; }
    s_diag.record_kind = kind; s_diag.record_start = now;
    s_diag.record_page = 0U; s_diag.record_pages = pages;
}
void zy100_stress_diag_record_page(uint32_t page)
{
    if (s_diag.active && !s_diag.frozen) { s_diag.record_page = page; }
}
void zy100_stress_diag_record_end(uint32_t now)
{
    if (!s_diag.active || s_diag.frozen) { return; }
    if (s_diag.record_kind == 1U) { s_diag.raw_max = sd_max(s_diag.raw_max, now - s_diag.record_start); }
    if (s_diag.record_kind == 4U) { s_diag.stress_max = sd_max(s_diag.stress_max, now - s_diag.record_start); }
    s_diag.record_kind = 0U; s_diag.record_page = s_diag.record_pages = 0U;
}
void zy100_stress_diag_freeze(uint32_t now, uint32_t water, uint32_t error)
{
    if (!s_diag.active || s_diag.frozen) { return; }
    zy100_stress_diag_water(now, water);
    sd_close_gap(now, water, 0U, true);
#if ZY100_STRESS_IO_DIAG_ENABLE
    if (s_diag.io.flags & SD_IO_PENDING)
    {
        uint32_t counter;
        if (bsp_capture_timebase_snapshot(&counter)) { s_diag.io.open_age = s_diag.io.issue - counter; }
        else { s_diag.io.flags |= SD_IO_CLOCK; }
    }
#endif
    s_diag.end = now; s_diag.error = error; s_diag.frozen = true;
}
uint32_t zy100_stress_diag_erases(uint32_t cumulative)
{
    return s_diag.active ? cumulative - s_diag.erase_base : 0U;
}

#if ZY100_ONLINE_BOUNDED_PUMP_ENABLE || ZY100_ONLINE_ERASE_AHEAD_ENABLE
void zy100_stress_diag_service(uint32_t steps, uint32_t elapsed_us, bool wait, bool clock_missing)
{
    if (!s_diag.active || s_diag.frozen) { return; }
    s_diag.service_calls++; s_diag.service_steps += steps;
    s_diag.service_peak_us = sd_max(s_diag.service_peak_us, elapsed_us);
    if (steps >= ZY100_ONLINE_PUMP_MAX_STEPS || elapsed_us >= ZY100_ONLINE_PUMP_BUDGET_US)
    { s_diag.service_limits++; }
    if (wait) { s_diag.service_waits++; }
    if (clock_missing) { s_diag.service_clock_missing++; }
}
void zy100_stress_diag_erase(bool urgent, bool issued, uint32_t ahead)
{
    if (!s_diag.active || s_diag.frozen) { return; }
    if (ahead < s_diag.erase_ahead_min) { s_diag.erase_ahead_min = ahead; }
    if (issued) { if (urgent) { s_diag.erase_urgent++; } else { s_diag.erase_background++; } }
}
#endif

/* The board's direct trace truncated v1 at 119 printable characters. Split v2
 * fields so every line fits even at uint32 maxima (checked with real C output).
 * Identity repeats per line; dwell is NOT hardware busy time or CPU use. */
#define SD_LOG(format, ...) ZY100_LOG_EVENT("[STRESS_DIAG] v=2 fw=%lu sid=%lu start=%lu n=%lu " format, \
    (unsigned long)VERSION_CODE, (unsigned long)s_diag.session, \
    (unsigned long)s_diag.start, (unsigned long)index, __VA_ARGS__)
#define UL(v) ((unsigned long)(v))
bool zy100_stress_diag_log_step(uint32_t session, uint32_t index)
{
    uint32_t n, first;
    const sd_event_t *e;
    if (!s_diag.frozen || session != s_diag.session) { return true; }
    if (index == 0U)
    {
        SD_LOG("kind=begin rate=%lu elapsed=%lu", UL(s_diag.rate), UL(s_diag.end-s_diag.start));
    }
    else if (index == 1U)
    { SD_LOG("kind=state error=%lu water=%lu", UL(s_diag.error), UL(s_diag.water)); }
    else if (index == 2U)
    { SD_LOG("kind=count released=%lu pump_max=%lu", UL(s_diag.released), UL(s_diag.pump_max)); }
    else if (index == 3U)
    { SD_LOG("kind=prep prep_erases=%lu events=%lu", UL(s_diag.prepare_erases), UL(s_diag.count)); }
    else if (index == 4U)
    { SD_LOG("kind=trace transitions=%lu", UL(s_diag.transitions)); }
    else if (index == 5U)
    { SD_LOG("kind=gap from=%lu to=%lu", UL(s_diag.best_start), UL(s_diag.best_end)); }
    else if (index == 6U)
    {
        SD_LOG("kind=water water_from=%lu water_to=%lu", UL(s_diag.best_water_start), UL(s_diag.best_water_end));
    }
    else if (index == 7U)
    { SD_LOG("kind=release freed=%lu open=%lu", UL(s_diag.best_freed), UL(s_diag.best_open)); }
    else if (index == 8U)
    {
        SD_LOG("kind=record type=%lu age=%lu", UL(s_diag.record_kind), UL(s_diag.record_kind ? s_diag.end-s_diag.record_start : 0U));
    }
    else if (index == 9U)
    { SD_LOG("kind=page page=%lu pages=%lu", UL(s_diag.record_page), UL(s_diag.record_pages)); }
    else if (index == 10U)
    { SD_LOG("kind=max raw=%lu stress=%lu", UL(s_diag.raw_max), UL(s_diag.stress_max)); }
    else if (index < 11U + 2U * ZY100_SD_COUNT)
    {
        n = (index - 11U) / 2U;
        if ((index - 11U) % 2U == 0U)
        { SD_LOG("kind=stage stage=%lu total=%lu peak=%lu", UL(n), UL(s_diag.total[n]), UL(s_diag.peak[n])); }
        else
        { SD_LOG("kind=part stage=%lu gap=%lu", UL(n), UL(s_diag.best_parts[n])); }
    }
    else if (index < 11U + 2U * ZY100_SD_COUNT + s_diag.count)
    {
        n = index - 11U - 2U * ZY100_SD_COUNT;
        first = (s_diag.next + SD_DEPTH - s_diag.count) % SD_DEPTH;
        e = &s_diag.events[(first+n) % SD_DEPTH];
        SD_LOG("kind=event ms=%lu stage=%lu water=%lu", UL(e->ms), UL(e->stage), UL(e->water));
    }
    else if (index == 11U + 2U * ZY100_SD_COUNT + s_diag.count)
    { SD_LOG("kind=end lines=%lu", UL(index + 1U)); }
#if ZY100_ONLINE_BOUNDED_PUMP_ENABLE || ZY100_ONLINE_ERASE_AHEAD_ENABLE
    else if (index < 11U + 2U * ZY100_SD_COUNT + s_diag.count + 7U)
    {
        /* Separate prefix: DIAG v2 and its end/line count stay unchanged. */
        n = index - (12U + 2U * ZY100_SD_COUNT + s_diag.count);
#define OPT_LOG(fmt, ...) ZY100_LOG_EVENT("[STRESS_OPT] v=1 fw=%lu sid=%lu start=%lu n=%lu " fmt, \
        UL(VERSION_CODE), UL(s_diag.session), UL(s_diag.start), UL(n), __VA_ARGS__)
        if (n == 0U) { OPT_LOG("pump=%lu erase=%lu", UL(ZY100_ONLINE_BOUNDED_PUMP_ENABLE), UL(ZY100_ONLINE_ERASE_AHEAD_ENABLE)); }
        else if (n == 1U) { OPT_LOG("calls=%lu steps=%lu", UL(s_diag.service_calls), UL(s_diag.service_steps)); }
        else if (n == 2U) { OPT_LOG("peak_us=%lu limits=%lu", UL(s_diag.service_peak_us), UL(s_diag.service_limits)); }
        else if (n == 3U) { OPT_LOG("waits=%lu no_clock=%lu", UL(s_diag.service_waits), UL(s_diag.service_clock_missing)); }
        else if (n == 4U) { OPT_LOG("bg=%lu urgent=%lu", UL(s_diag.erase_background), UL(s_diag.erase_urgent)); }
        else { OPT_LOG("prep_ms=%lu ahead=%lu", UL(s_diag.prepare_ms), UL(s_diag.erase_ahead_min)); }
#undef OPT_LOG
    }
#endif
#if ZY100_STRESS_IO_DIAG_ENABLE
    else
    {
        uint32_t base = 12U + 2U * ZY100_SD_COUNT + s_diag.count;
#if ZY100_ONLINE_BOUNDED_PUMP_ENABLE || ZY100_ONLINE_ERASE_AHEAD_ENABLE
        base += 6U;
#endif
        n = index - base;
        /* At most two u32 values per line: preserve the UART formatter limit. */
#define IO_LOG(fmt, ...) ZY100_LOG_EVENT("[STRESS_IO] v=1 fw=%lu sid=%lu start=%lu n=%lu " fmt, \
        UL(VERSION_CODE), UL(s_diag.session), UL(s_diag.start), UL(n), __VA_ARGS__)
        if (n == 0U) { IO_LOG("fifo=%lu flags=%lu", UL(ZY100_FLASH_FIFO_TRANSFER_ENABLE), UL(s_diag.io.flags & 7U)); }
        else if (n <= 8U)
        {
            const uint32_t *m = s_diag.io.metric[(n - 1U) / 2U];
            if ((n & 1U) != 0U) { IO_LOG("calls=%lu total=%lu", UL(m[0]), UL(m[1])); }
            else { IO_LOG("peak=%lu unit=%lu", UL(m[2]), 1UL); }
        }
        else if (n == 9U) { IO_LOG("bus=%lu wip=%lu", UL(s_diag.io.bus_count), UL(s_diag.io.wip_count)); }
        else if (n == 10U) { IO_LOG("spans=%lu total=%lu", UL(s_diag.io.spans), UL(s_diag.io.span_total)); }
        else if (n == 11U) { IO_LOG("span_max=%lu poll_max=%lu", UL(s_diag.io.span_max), UL(s_diag.io.poll_max)); }
        else if (n == 12U) { IO_LOG("open_us=%lu pending=%lu", UL(s_diag.io.open_age), UL(s_diag.io.flags & SD_IO_PENDING)); }
        else if (n == 13U) { IO_LOG("bus_us=%lu wip_us=%lu", UL(s_diag.io.bus_wait), UL(s_diag.io.wip_wait)); }
        else if (n == 14U) { IO_LOG("end=%lu lines=%lu", 1UL, 15UL); }
        else { return true; }
#undef IO_LOG
    }
#else
    else { return true; }
#endif
    return false;
}
