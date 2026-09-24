#ifndef ZY100_STRESS_SOURCE_H
#define ZY100_STRESS_SOURCE_H

#include <stdbool.h>
#include <stdint.h>

#define ZY100_STRESS_PERIOD_MS 20U
#define ZY100_STRESS_RING_BYTES 4096U
#define ZY100_STRESS_HEADER_BYTES 64U
#define ZY100_STRESS_DELAY_SLOTS 16U
#define ZY100_STRESS_PROTOCOL_VERSION 1U
#define ZY100_STRESS_QUERY_DETAIL 0x010C143FUL /* version, log2(ring), period, rate mask */

typedef enum
{
    ZY100_STRESS_OK = 0,
    ZY100_STRESS_RING_FULL = 1,
    ZY100_STRESS_RANGE = 2,
    ZY100_STRESS_RESERVE = 3,
    ZY100_STRESS_PROGRAM = 4,
    ZY100_STRESS_VERIFY = 5,
    ZY100_STRESS_COMMIT = 6,
    ZY100_STRESS_OWNER = 7
} zy100_stress_error_t;

/* The capture worker owns this state. No allocation and no clock/platform API. */
typedef struct
{
    uint8_t *ring;
    uint32_t rate;
    uint32_t seed;
    uint32_t start_ms;
    uint32_t elapsed_ms;
    uint32_t expected;
    uint32_t generated;
    uint32_t consumed;
    uint32_t rejected;
    uint32_t peak;
    uint32_t delay_max_ms;
    uint32_t delays[ZY100_STRESS_DELAY_SLOTS];
    uint16_t head;
    uint16_t used;
    uint8_t error;
    bool frozen;
} zy100_stress_source_t;

bool zy100_stress_rate_valid(uint32_t rate);
uint8_t zy100_stress_pattern(uint32_t seed, uint32_t byte_offset);
bool zy100_stress_source_begin(zy100_stress_source_t *s, uint8_t *ring,
                               uint32_t rate, uint32_t seed, uint32_t now_ms);
bool zy100_stress_source_tick(zy100_stress_source_t *s, uint32_t now_ms);
bool zy100_stress_source_freeze(zy100_stress_source_t *s, uint32_t now_ms);
bool zy100_stress_source_copy(const zy100_stress_source_t *s,
                              uint8_t *dst, uint32_t count);
bool zy100_stress_source_consume(zy100_stress_source_t *s, uint32_t count);
bool zy100_stress_source_header(const zy100_stress_source_t *s,
                                uint8_t *header, uint32_t blocks);

#endif
