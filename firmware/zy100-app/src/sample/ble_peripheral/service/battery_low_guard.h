#ifndef BATTERY_LOW_GUARD_H
#define BATTERY_LOW_GUARD_H

#include <stdbool.h>
#include <stdint.h>

#include "../app_flags.h"

#ifdef __cplusplus
extern "C" {
#endif

#if F_APP_BATTERY_ADC_ENABLE
void battery_low_guard_force_safe_state(const char *reason);
#else
static inline void battery_low_guard_force_safe_state(const char *reason)
{
    (void)reason;
}
#endif

#if F_APP_BATTERY_ADC_ENABLE && F_APP_BATTERY_LPC_GUARD_ENABLE
bool battery_low_guard_init(void);
bool battery_low_guard_on_capture_start(uint32_t runtime_ms);
void battery_low_guard_on_capture_stop(uint32_t runtime_ms, const char *reason);
bool battery_low_guard_take_latched(void);
bool battery_low_guard_is_armed(void);
bool battery_low_guard_is_latched(void);
#else
static inline bool battery_low_guard_init(void)
{
    return true;
}

static inline bool battery_low_guard_on_capture_start(uint32_t runtime_ms)
{
    (void)runtime_ms;
    return false;
}

static inline void battery_low_guard_on_capture_stop(uint32_t runtime_ms,
                                                     const char *reason)
{
    (void)runtime_ms;
    (void)reason;
}

static inline bool battery_low_guard_take_latched(void)
{
    return false;
}

static inline bool battery_low_guard_is_armed(void)
{
    return false;
}

static inline bool battery_low_guard_is_latched(void)
{
    return false;
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_LOW_GUARD_H */
