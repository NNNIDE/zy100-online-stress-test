#ifndef ZY100_RT_MARKER_TIMER_H
#define ZY100_RT_MARKER_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../common/imu_common.h"

typedef void (*zy100_rt_marker_timer_notify_cb_t)(void);

typedef struct
{
    uint32_t timer_due_count;
    uint32_t timer_notify_count;
} zy100_rt_marker_timer_stats_t;

imu_status_t zy100_rt_marker_timer_config(uint32_t target_hz);
imu_status_t zy100_rt_marker_timer_start(void);
void zy100_rt_marker_timer_stop(void);
bool zy100_rt_marker_timer_is_running(void);
void zy100_rt_marker_timer_reset_counters(void);
void zy100_rt_marker_timer_register_notify_callback(zy100_rt_marker_timer_notify_cb_t cb);
void zy100_rt_marker_timer_get_stats(zy100_rt_marker_timer_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_RT_MARKER_TIMER_H */
