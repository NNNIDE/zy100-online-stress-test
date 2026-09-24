#ifndef BATTERY_ADC_GUARD_H
#define BATTERY_ADC_GUARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../app_flags.h"

typedef enum
{
    BATTERY_ADC_GUARD_BLOCK_NONE = 0U,
    BATTERY_ADC_GUARD_BLOCK_CAPTURE,
    BATTERY_ADC_GUARD_BLOCK_B_CRITICAL,
    BATTERY_ADC_GUARD_BLOCK_SPI,
    BATTERY_ADC_GUARD_BLOCK_FIFO,
    BATTERY_ADC_GUARD_BLOCK_STORE,
    BATTERY_ADC_GUARD_BLOCK_EXPORT,
    BATTERY_ADC_GUARD_BLOCK_STOP,
    BATTERY_ADC_GUARD_BLOCK_RAW,
    BATTERY_ADC_GUARD_BLOCK_RECORD,
    BATTERY_ADC_GUARD_BLOCK_REPLAY,
    BATTERY_ADC_GUARD_BLOCK_FLASH,
    BATTERY_ADC_GUARD_BLOCK_BLE,
    BATTERY_ADC_GUARD_BLOCK_ADC,
    BATTERY_ADC_GUARD_BLOCK_STATE,
    BATTERY_ADC_GUARD_BLOCK_BUDGET,
    BATTERY_ADC_GUARD_BLOCK_COUNT,
} battery_adc_guard_block_reason_t;

typedef struct
{
    uint32_t check_count;
    uint32_t runtime_ms;
    uint16_t raw;
    uint16_t battery_mv;
    uint8_t percent;
} battery_adc_guard_report_t;

typedef struct
{
    uint32_t attempt_count;
    uint32_t valid_count;
    uint32_t fail_count;
    uint16_t raw_min;
    uint16_t raw_max;
    uint16_t raw_avg;
    bool raw_valid;
    uint32_t valid_windows;
    uint32_t invalid_windows;
    uint16_t final_raw;
    uint16_t final_mv;
    uint8_t final_percent;
    bool final_valid;
    uint64_t read_total_us;
    uint32_t read_max_us;
} battery_adc_guard_capture_stats_t;

#if F_APP_BATTERY_ADC_ENABLE && F_APP_BATTERY_ADC_GUARD_ENABLE
void battery_adc_guard_init(void);
bool battery_adc_guard_prepare_capture(uint32_t now_ms);
void battery_adc_guard_on_capture_start(uint32_t now_ms);
void battery_adc_guard_on_capture_stop(uint32_t now_ms, const char *reason);
bool battery_adc_guard_poll(uint32_t now_ms,
                            battery_adc_guard_block_reason_t block_reason);
bool battery_adc_guard_take_report(battery_adc_guard_report_t *report);
bool battery_adc_guard_take_capture_stats(
    battery_adc_guard_capture_stats_t *stats);
bool battery_adc_guard_is_latched(void);
void battery_adc_guard_reset_latch(void);
#else
static inline void battery_adc_guard_init(void)
{
}

static inline bool battery_adc_guard_prepare_capture(uint32_t now_ms)
{
    (void)now_ms;
    return true;
}

static inline void battery_adc_guard_on_capture_start(uint32_t now_ms)
{
    (void)now_ms;
}

static inline void battery_adc_guard_on_capture_stop(uint32_t now_ms,
                                                     const char *reason)
{
    (void)now_ms;
    (void)reason;
}

static inline bool battery_adc_guard_poll(uint32_t now_ms,
                                          battery_adc_guard_block_reason_t block_reason)
{
    (void)now_ms;
    (void)block_reason;
    return false;
}

static inline bool battery_adc_guard_take_report(battery_adc_guard_report_t *report)
{
    (void)report;
    return false;
}

static inline bool battery_adc_guard_take_capture_stats(
    battery_adc_guard_capture_stats_t *stats)
{
    (void)stats;
    return false;
}

static inline bool battery_adc_guard_is_latched(void)
{
    return false;
}

static inline void battery_adc_guard_reset_latch(void)
{
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_ADC_GUARD_H */
