#ifndef IMU_RT_MARKER_H
#define IMU_RT_MARKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../common/imu_common.h"

typedef struct
{
    uint32_t read_ok_total;
    uint32_t read_fail_total;
    uint32_t marker_total;
    uint32_t due_total;
    uint32_t skipped_high_pressure_total;
    uint32_t skipped_spi_busy_total;
    uint32_t skipped_not_due_total;
    uint32_t poll_call_count_total;
    uint32_t skipped_late_slots_total;
    uint32_t dropped_cooldown_total;
    uint32_t rejected_timeout_total;
    uint32_t marker_dropped_total;
    uint32_t max_read_us;
    uint32_t max_gap_us;
} imu_rt_marker_stats_t;

imu_status_t imu_rt_marker_reset(uint64_t session_start_us);
void imu_rt_marker_poll(uint64_t now_ms, bool fifo_high_pressure);
void imu_rt_marker_timer_due_isr_hook(uint32_t due_seq);
bool imu_rt_marker_has_due(void);
bool imu_rt_marker_handle_due_once(uint32_t now_ms);
void imu_rt_marker_skip_fifo_emergency(uint32_t now_ms);
void imu_rt_marker_skip_spi_busy(uint32_t now_ms);
bool imu_rt_marker_feed_fifo_packet(const uint8_t packet16[16],
                                    uint32_t sample_seq,
                                    uint64_t t_us);
void imu_rt_marker_finish(void);
void imu_rt_marker_log_final(void);
void imu_rt_marker_get_stats(imu_rt_marker_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* IMU_RT_MARKER_H */
