#ifndef ZY100_RTC_CLOCK_H
#define ZY100_RTC_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* Development default time base.
 * This is 2026-05-26 00:00:00 UTC+8 and is not a trusted real-world
 * calibration. A successful BLE TIME_SYNC replaces it with real Unix ms.
 * When time_source is DEFAULT_UNCALIBRATED, the host/algorithm must not
 * treat now_ms as trusted wall-clock time.
 */
#define ZY100_RTC_DEFAULT_UNIX_MS 1779724800000ULL

/* RTL8762D RTC configuration shared with the resident DFU image. */
#define ZY100_RTC_INPUT_HZ             32000ULL
#define ZY100_RTC_PRESCALER_VALUE      3999U
#define ZY100_RTC_PRESCALE_DIV \
    ((uint32_t)(ZY100_RTC_PRESCALER_VALUE + 1U))
#define ZY100_RTC_COUNTER_MASK         0x00FFFFFFUL
#define ZY100_RTC_PRE_COUNTER_MASK     0x00000FFFUL
#define ZY100_RTC_COUNTER_WRAP_TICKS \
    (((uint64_t)(ZY100_RTC_COUNTER_MASK + 1UL)) * \
     ((uint64_t)ZY100_RTC_PRESCALE_DIV))

typedef enum
{
    ZY100_RTC_TIME_SOURCE_DEFAULT_UNCALIBRATED = 0U,
    ZY100_RTC_TIME_SOURCE_BLE_TIME_SYNCED = 1U,
    ZY100_RTC_TIME_SOURCE_OTA_RESTORED_ESTIMATED = 2U,
} zy100_rtc_time_source_t;

/* Monotonic raw RTC domain used for short-duration endpoint measurements.
 * These ticks are independent of BLE TIME_SYNC wall-clock rebasing. */
typedef struct
{
    uint64_t ticks;
    uint64_t wrap_ticks;
    uint64_t unix_time_ms;
    uint32_t nominal_tick_hz;
    uint8_t unix_time_valid;
    uint8_t reserved[3];
} zy100_rtc_raw_snapshot_t;

typedef struct
{
    uint64_t host_ms;
    uint64_t rtc_before_ms;
    uint64_t abs_diff_ms;
    uint64_t host_elapsed_ms;
    uint64_t rtc_elapsed_ms;
    uint64_t drift_abs_ms;
    uint32_t diff_abs_s;
    uint32_t diff_abs_ms_rem;
    uint32_t drift_abs_s;
    uint32_t drift_abs_ms_rem;
    int32_t ppm;
    uint8_t old_calibrated;
    uint8_t source;
    uint8_t has_prev;
    uint8_t elapsed_valid;
    char diff_sign;
    char drift_sign;
} zy100_rtc_debug_sync_snapshot_t;

void zy100_rtc_clock_init(void);
void zy100_rtc_clock_reset_user(void);

/* is_valid is kept for compatibility. It means "calibrated by BLE TIME_SYNC".
 * false does not mean now_ms is unusable; it means the time axis is the
 * development default and is not trusted wall-clock time.
 */
bool zy100_rtc_clock_is_valid(void);
bool zy100_rtc_clock_is_calibrated(void);
uint8_t zy100_rtc_clock_time_source(void);
uint64_t zy100_rtc_clock_now_ms(void);
bool zy100_rtc_clock_raw_snapshot(zy100_rtc_raw_snapshot_t *out);
bool zy100_rtc_clock_ota_restore_requires_consume(void);
void zy100_rtc_clock_ota_restore_note_consumed(void);
/* Final, irreversible APP -> DFU handoff only; masks the APP overflow IRQ
 * without stopping/resetting the retained RTC used by the OTA checkpoint. */
void zy100_rtc_clock_prepare_ota_handoff(void);
void zy100_rtc_clock_sync(uint32_t user_id, uint64_t unix_time_ms);
void zy100_rtc_clock_debug_log_pre_sync(uint32_t user_id,
                                        uint32_t training_id,
                                        uint64_t host_ms,
                                        zy100_rtc_debug_sync_snapshot_t *snapshot);
void zy100_rtc_clock_debug_log_sync_done(uint32_t user_id,
                                         uint32_t training_id,
                                         const zy100_rtc_debug_sync_snapshot_t *snapshot);
uint32_t zy100_rtc_clock_user_id(void);
uint64_t zy100_rtc_clock_last_sync_ms(void);
int32_t zy100_rtc_clock_last_drift_ms(void);
void zy100_rtc_clock_prepare_dlps(void);
void zy100_rtc_clock_on_wakeup(void);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_RTC_CLOCK_H */
