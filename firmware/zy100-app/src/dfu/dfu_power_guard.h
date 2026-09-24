#ifndef DFU_POWER_GUARD_H
#define DFU_POWER_GUARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    DFU_POWER_GUARD_OK = 0U,
    DFU_POWER_GUARD_INVALID_STACMD_PIN,
    DFU_POWER_GUARD_EXTERNAL_POWER_READ_FAILED,
    DFU_POWER_GUARD_STACMD_LOW_ON_BATTERY,
} T_DFU_POWER_GUARD_RESULT;

typedef struct
{
    T_DFU_POWER_GUARD_RESULT result;
    bool external_power_present;
    uint8_t stacmd_level;
} T_DFU_POWER_GUARD_SNAPSHOT;

T_DFU_POWER_GUARD_RESULT dfu_power_guard_init(T_DFU_POWER_GUARD_SNAPSHOT *snapshot);
const char *dfu_power_guard_result_name(T_DFU_POWER_GUARD_RESULT result);
/* Read-only diagnostics: capture before changing the pin, including at APP entry. */
void dfu_power_guard_trace_pin(const char *stage);
/* Release in the running APP/DFU only; RESET_ALL/ROM may reconfigure the pad. */
T_DFU_POWER_GUARD_RESULT dfu_power_guard_release_before_reset(void);
/* Successful OTA tail: TX/RX use the same input pull-up as STACMD; log on P0_3. */
T_DFU_POWER_GUARD_RESULT dfu_power_guard_release_uart_before_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* DFU_POWER_GUARD_H */
