#ifndef BSP_CAPTURE_TIMEBASE_H
#define BSP_CAPTURE_TIMEBASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define BSP_CAPTURE_TIMEBASE_HZ          1000000UL
#define BSP_CAPTURE_TIMEBASE_RANGE_US    0x100000000ULL

bool bsp_capture_timebase_init(void);
bool bsp_capture_timebase_start(uint32_t *counter_out);
bool bsp_capture_timebase_snapshot(uint32_t *counter_out);
void bsp_capture_timebase_stop(void);
bool bsp_capture_timebase_is_running(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAPTURE_TIMEBASE_H */
