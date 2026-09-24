#include "zy100_stress_source.h"
#include <string.h>
#include <limits.h>

static void put32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8U);
    p[2] = (uint8_t)(value >> 16U);
    p[3] = (uint8_t)(value >> 24U);
}

bool zy100_stress_rate_valid(uint32_t rate)
{
    return rate == 0U || rate == 16000U || rate == 32000U ||
           rate == 48000U || rate == 64000U || rate == 96000U;
}

uint8_t zy100_stress_pattern(uint32_t seed, uint32_t byte_offset)
{
    uint32_t x = byte_offset ^ seed;
    return (uint8_t)(x ^ (x >> 8U) ^ (x >> 16U) ^ (x >> 24U));
}

bool zy100_stress_source_begin(zy100_stress_source_t *s, uint8_t *ring,
                               uint32_t rate, uint32_t seed, uint32_t now_ms)
{
    if (s == NULL || ring == NULL || !zy100_stress_rate_valid(rate))
    {
        return false;
    }
    memset(s, 0, sizeof(*s));
    s->ring = ring;
    s->rate = rate;
    s->seed = seed;
    s->start_ms = now_ms;
    return true;
}

bool zy100_stress_source_tick(zy100_stress_source_t *s, uint32_t now_ms)
{
    uint32_t block_bytes, due_blocks, seq, delay, i;
    uint64_t expected;
    if (s == NULL || s->ring == NULL) { return false; }
    if (s->frozen || s->error != 0U) { return s->error == 0U; }
    s->elapsed_ms = (uint32_t)(now_ms - s->start_ms);
    block_bytes = s->rate / (1000U / ZY100_STRESS_PERIOD_MS);
    due_blocks = s->elapsed_ms / ZY100_STRESS_PERIOD_MS;
    expected = (uint64_t)due_blocks * block_bytes;
    if (expected > UINT32_MAX || s->elapsed_ms >= 0x80000000UL)
    {
        s->error = ZY100_STRESS_RANGE;
        s->frozen = true;
        return false;
    }
    s->expected = (uint32_t)expected;
    if (block_bytes == 0U) { return true; }
    while (s->generated < s->expected)
    {
        if ((uint32_t)s->used + block_bytes > ZY100_STRESS_RING_BYTES)
        {
            s->rejected = s->expected - s->generated;
            s->error = ZY100_STRESS_RING_FULL;
            s->frozen = true;
            return false;
        }
        seq = s->generated / block_bytes;
        delay = s->elapsed_ms - (seq + 1U) * ZY100_STRESS_PERIOD_MS;
        s->delays[seq % ZY100_STRESS_DELAY_SLOTS] = delay;
        if (delay > s->delay_max_ms) { s->delay_max_ms = delay; }
        for (i = 0U; i < block_bytes; ++i)
        {
            s->ring[((uint32_t)s->head + s->used + i) % ZY100_STRESS_RING_BYTES] =
                zy100_stress_pattern(s->seed, s->generated + i);
        }
        s->generated += block_bytes;
        s->used = (uint16_t)(s->used + block_bytes);
        if (s->used > s->peak) { s->peak = s->used; }
    }
    return true;
}

bool zy100_stress_source_freeze(zy100_stress_source_t *s, uint32_t now_ms)
{
    bool ok = zy100_stress_source_tick(s, now_ms);
    if (s != NULL) { s->frozen = true; }
    return ok;
}

bool zy100_stress_source_copy(const zy100_stress_source_t *s,
                              uint8_t *dst, uint32_t count)
{
    uint32_t i;
    if (s == NULL || dst == NULL || count > s->used) { return false; }
    for (i = 0U; i < count; ++i)
    {
        dst[i] = s->ring[((uint32_t)s->head + i) % ZY100_STRESS_RING_BYTES];
    }
    return true;
}

bool zy100_stress_source_consume(zy100_stress_source_t *s, uint32_t count)
{
    if (s == NULL || count > s->used) { return false; }
    s->head = (uint16_t)(((uint32_t)s->head + count) % ZY100_STRESS_RING_BYTES);
    s->used = (uint16_t)(s->used - count);
    s->consumed += count;
    return true;
}

bool zy100_stress_source_header(const zy100_stress_source_t *s,
                                uint8_t *header, uint32_t blocks)
{
    uint32_t bytes, seq;
    if (s == NULL || header == NULL || s->rate == 0U ||
        blocks == 0U || blocks > 2U) { return false; }
    bytes = s->rate / 50U;
    if ((s->consumed % bytes) != 0U || blocks * bytes > s->used) { return false; }
    seq = s->consumed / bytes;
    memset(header, 0, ZY100_STRESS_HEADER_BYTES);
    memcpy(header, "STRS", 4U);
    header[4] = ZY100_STRESS_PROTOCOL_VERSION;
    header[6] = ZY100_STRESS_HEADER_BYTES;
    put32(header + 8U, s->seed);
    put32(header + 12U, s->rate);
    put32(header + 16U, ZY100_STRESS_PERIOD_MS);
    put32(header + 20U, seq);
    put32(header + 24U, blocks);
    put32(header + 28U, s->consumed);
    put32(header + 32U, blocks * bytes);
    put32(header + 36U, (seq + 1U) * ZY100_STRESS_PERIOD_MS);
    put32(header + 40U, s->delays[seq % ZY100_STRESS_DELAY_SLOTS]);
    if (blocks == 2U)
    {
        put32(header + 44U, s->delays[(seq + 1U) % ZY100_STRESS_DELAY_SLOTS]);
    }
    put32(header + 48U, 1U); /* deterministic pattern mode */
    put32(header + 52U, s->start_ms);
    return true;
}
