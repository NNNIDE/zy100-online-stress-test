#ifndef ZY100_CLOCK_CONFIG_H
#define ZY100_CLOCK_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#ifndef ZY100_VENDOR_TICK_RUNTIME_ENABLE
#define ZY100_VENDOR_TICK_RUNTIME_ENABLE 0
#endif

#define ZY100_CLOCK_CHECK_PERIOD_MS 1000U

bool zy100_clock_config_apply_active(void);
bool zy100_clock_config_apply_active_no_os_check(void);
bool zy100_clock_config_poll_check(uint32_t runtime_ms);
void zy100_clock_config_reset_poll_baseline(void);
/* Central OS runtime time wrapper; returns milliseconds; the configured RTOS tick resolution is 10ms. */
uint32_t zy100_os_time_ms(void);
/* Debug-only nominal vendor tick helpers. Do not use these for runtime scheduling or health. */
uint32_t zy100_clock_config_vendor_tick_hz(void);
uint32_t zy100_clock_config_vendor_ticks_to_us(uint32_t ticks);
uint32_t zy100_clock_config_vendor_tick_delta_us(uint32_t start_tick, uint32_t end_tick);
/* Coarse OS-ms-derived compatibility timestamp. Do not use for sub-ms deadlines or profiling. */
uint64_t zy100_clock_config_monotonic_us(void);
uint32_t zy100_clock_config_monotonic_us32(void);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_CLOCK_CONFIG_H */
