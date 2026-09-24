#include "power_diag.h"
#if ZY100_POWER_DIAG_ENABLE
#include <string.h>
#include "zy100_rtc_clock.h"
#include "version.h"

/* Cached first error only. No standby segments, register scans or IO snapshots. */
typedef struct
{
    uint32_t transaction, power_generation, stages;
    uint16_t wait_ms, valid;
    uint8_t origin, rail, flags, stage, retries, status;
    uint8_t fail_stage, fail_status, fail_bank, fail_reg, fail_read, fail_mask, fail_expected, fail_valid;
    uint8_t pre[5], extra[7];
    uint8_t restore_count;
    uint8_t difference[8];
    uint32_t restore_transaction;
    uint32_t cycle, off_ms;
} pwrd_process_t;
typedef char pwrd_process_budget[(sizeof(pwrd_process_t) <= 64U) ? 1 : -1];
static struct
{
    pwrd_process_t process;
    uint32_t transaction, boot, wake;
    uint8_t verify_depth, next_origin;
    bool initialized, imu_recording;
} s_pwrd;
typedef char pwrd_memory_budget[(sizeof(s_pwrd) <= 128U) ? 1 : -1];

static void pwrd_line(const char *kind,
                      uint32_t a, uint32_t b, uint32_t c, uint32_t d,
                      uint32_t e, uint32_t g)
{
    ZY100_LOG_ERROR("[PWRD_%s] boot=%lu wake=%lu segment=%lu a=%08lx b=%08lx c=%08lx d=%08lx e=%08lx f=%08lx",
                   kind, (unsigned long)s_pwrd.boot, (unsigned long)s_pwrd.wake,
                   0UL, (unsigned long)a, (unsigned long)b,
                   (unsigned long)c, (unsigned long)d, (unsigned long)e, (unsigned long)g);
}
void pwrd_wake(void) { s_pwrd.wake++; }
void pwrd_adc(void) { pwrd_origin(PWRD_ADC_RESTORE); }
void pwrd_poll(void)
{
    zy100_rtc_raw_snapshot_t now;
    if (s_pwrd.initialized) return;
    s_pwrd.initialized = true;
    s_pwrd.boot = zy100_rtc_clock_raw_snapshot(&now) ? (uint32_t)now.ticks : 0U;
}

void pwrd_origin(uint8_t origin) { s_pwrd.next_origin = origin; }
void pwrd_prepare(uint8_t origin, uint8_t rail, uint32_t generation, uint8_t flags)
{
    /* Keep the first pre-power state of a Feature transaction across its inner arm. */
    if (origin == PWRD_ARM && s_pwrd.next_origin >= PWRD_FEATURE_OFF &&
        s_pwrd.next_origin <= PWRD_FEATURE_ON) return;
    memset(&s_pwrd.process, 0, sizeof(s_pwrd.process));
    s_pwrd.process.transaction = ++s_pwrd.transaction;
    s_pwrd.process.status = 0xffU;
    s_pwrd.process.origin = s_pwrd.next_origin ? s_pwrd.next_origin : origin;
    s_pwrd.process.rail = rail;
    s_pwrd.process.power_generation = generation;
    s_pwrd.process.flags = flags | 0x80U; /* Cached pre-state present. */
}
void pwrd_cycle(uint8_t stage, uint8_t status, uint32_t off_ms)
{
    pwrd_process_t *p = &s_pwrd.process;
    /* First failure wins even if subsequent cleanup also fails. */
    if (!(p->cycle & 0xff00U)) p->cycle = (p->cycle & 0xffff0000UL) | stage | ((uint32_t)status << 8);
    if (off_ms != UINT32_MAX) { p->off_ms = off_ms; p->cycle |= 0x10000UL; }
    if (stage == 6U && !status) p->cycle |= 0x20000UL;
}
void pwrd_cycle_elapsed(uint32_t ms)
{
    if (ms > 8191U) ms = 8191U;
    s_pwrd.process.cycle = (s_pwrd.process.cycle & 0x3ffffUL) | 0x40000UL | (ms << 19);
}
void pwrd_wait(uint16_t ms) { s_pwrd.process.wait_ms = ms; }
uint32_t pwrd_imu_begin(uint8_t operation)
{
    uint32_t token = (s_pwrd.transaction + 1U) & 0x3fffffffU;
    if (!token) token = 1U;
    s_pwrd.transaction = token;
    /* Failed arm cleanup must not overwrite its first failure evidence. */
    if (operation == 3U &&
        s_pwrd.process.status && s_pwrd.process.status != 0xffU)
    {
        s_pwrd.verify_depth++;
        return token | 0x40000000U;
    }
    if (operation == 2U)
    {
        pwrd_process_t *p = &s_pwrd.process;
        if (p->restore_count != UINT8_MAX) p->restore_count++;
        s_pwrd.verify_depth++;
        return token | 0x80000000U;
    }
    s_pwrd.process.transaction = token;
    s_pwrd.process.stages = 0U;
    s_pwrd.process.stage = 0U;
    s_pwrd.process.valid = 0U;
    s_pwrd.process.flags &= (uint8_t)~0x40U;
    s_pwrd.process.retries = 0U;
    s_pwrd.process.fail_stage = 0U;
    s_pwrd.process.fail_valid = 0U;
    s_pwrd.process.status = 0xffU; /* Not completed. */
    s_pwrd.imu_recording = true;
    if (operation == 1U) s_pwrd.next_origin = 0U;
    return token;
}
void pwrd_step(uint32_t token, uint8_t stage, uint8_t status)
{
    pwrd_process_t *p = &s_pwrd.process;
    if (token != p->transaction || s_pwrd.verify_depth) return;
    p->stage = stage;
    if (!status && stage < 32U) p->stages |= 1UL << stage;
    if (status && !p->fail_stage) { p->fail_stage = stage; p->fail_status = status; }
}
void pwrd_stage(uint8_t stage)
{
    if (s_pwrd.imu_recording && !s_pwrd.verify_depth) s_pwrd.process.stage = stage;
}
void pwrd_retry(void)
{
    if (s_pwrd.imu_recording && !s_pwrd.verify_depth && s_pwrd.process.retries != UINT8_MAX)
        s_pwrd.process.retries++;
}
void pwrd_verify(uint32_t token, uint8_t bank, uint8_t reg, uint8_t value,
                 uint8_t mask, uint8_t expected, uint8_t status)
{
    pwrd_process_t *p;
    if (!status && (value & mask) == (expected & mask)) return;
    if (token & 0x80000000U)
    {
        p = &s_pwrd.process;
        if (p->restore_transaction) return;
        p->restore_transaction = token;
        p->difference[0] = bank; p->difference[1] = reg;
        p->difference[2] = value; p->difference[3] = mask;
        p->difference[4] = expected; p->difference[5] = status;
        p->difference[6] = !status; p->difference[7] = PWRD_WEAK_RESTORE;
        return;
    }
    if (!s_pwrd.imu_recording || s_pwrd.verify_depth) return;
    p = &s_pwrd.process;
    if (p->fail_valid) return;
    p->fail_stage = p->stage; p->fail_status = status;
    p->fail_bank = bank; p->fail_reg = reg; p->fail_read = value;
    p->fail_mask = mask; p->fail_expected = expected;
    p->fail_valid = status ? 1U : 3U; /* Record present / actual read valid. */
}
void pwrd_fault(uint8_t operation, uint8_t bank, uint8_t reg, uint8_t value,
                bool read_valid, uint8_t status)
{
    pwrd_process_t *p = &s_pwrd.process;
    if (!status || !s_pwrd.imu_recording || s_pwrd.verify_depth || p->fail_valid) return;
    pwrd_verify(0U, bank, reg, value, 0U, 0U, status);
    p->fail_valid = 1U | (read_valid ? 2U : 0U) | (operation << 2);
}
void pwrd_io_detail(uint32_t elapsed_us, uint32_t polls, uint8_t acquire_status,
                    uint8_t release_status, uint8_t last, uint8_t valid)
{
    pwrd_process_t *p = &s_pwrd.process;
    if (!s_pwrd.imu_recording || s_pwrd.verify_depth || !p->fail_valid || (p->flags & 0x40U)) return;
    memcpy(p->pre, &elapsed_us, 4U);
    p->pre[4] = acquire_status;
    memcpy(p->extra, &polls, 4U);
    p->extra[4] = release_status;
    p->extra[5] = last;
    p->extra[6] = valid;
    p->valid &= 0xf000U; /* Payload is no longer configuration readback. */
    p->flags |= 0x40U;
}
void pwrd_imu_end(uint8_t status)
{
    if (status && !s_pwrd.process.fail_stage)
    {
        s_pwrd.process.fail_stage = s_pwrd.process.stage ? s_pwrd.process.stage : 31U;
        s_pwrd.process.fail_status = status;
    }
    s_pwrd.process.status = status;
    s_pwrd.imu_recording = false;
}
void pwrd_imu_finish(uint32_t token, uint8_t status)
{
    if (token & 0x40000000U)
    {
        /* v3 valid[15:12]: 0 unavailable, 1 OK, 2..14 status+1, 15 overflow. */
        s_pwrd.process.valid = (uint16_t)((s_pwrd.process.valid & 0x0fffU) |
            ((uint16_t)(status < 14U ? status + 1U : 15U) << 12));
        if (s_pwrd.verify_depth) s_pwrd.verify_depth--;
    }
    else if (token & 0x80000000U)
    {
        if (status) pwrd_verify(token, 0xffU, 0xffU, 0U, 0U, 0U, status);
        if (s_pwrd.verify_depth) s_pwrd.verify_depth--;
    }
    else if (token == s_pwrd.process.transaction) pwrd_imu_end(status);
}
static uint32_t pwrd_word(const uint8_t *p)
{
    return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
void pwrd_report_failure(const pwrd_context_t *c, uint8_t domain,
                         uint8_t stage, uint8_t status)
{
    const pwrd_process_t *p = &s_pwrd.process;
    bool imu = domain == 2U;
    bool detail = imu && (p->flags & 0x40U);
    /* Reuse identity only. Do not create READY, read a bus, or delay shutdown. */
    pwrd_line("FAULT", p->transaction,
              domain | ((uint32_t)stage << 8) | ((uint32_t)status << 16) | (1UL << 24),
              c->state | ((uint32_t)c->rail << 8) | ((uint32_t)c->users << 16),
              c->owner | ((uint32_t)c->blockers << 8), c->generation, VERSION_CODE);
    pwrd_line("FAULTIO", p->fail_stage | ((uint32_t)p->fail_status << 8) |
              ((uint32_t)p->fail_bank << 16) | ((uint32_t)p->fail_reg << 24),
              p->fail_read | ((uint32_t)p->fail_mask << 8) |
              ((uint32_t)p->fail_expected << 16) | ((uint32_t)(imu ? p->fail_valid : 0U) << 24),
              detail ? pwrd_word(p->pre) : 0U,
              detail ? pwrd_word(p->extra) : 0U,
              detail ? p->pre[4] | ((uint32_t)p->extra[4] << 8) |
              ((uint32_t)p->extra[5] << 16) | ((uint32_t)p->extra[6] << 24) : 0U,
              p->transaction);
    pwrd_line("FAULTEND", p->transaction, p->cycle, p->valid >> 12,
              VERSION_GCID, VERSION_GCID2, 1U);
}
#endif
