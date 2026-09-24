#ifndef ZY100_ONLINE_IMU_SESSION_H
#define ZY100_ONLINE_IMU_SESSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ZY100_ONLINE_IMU_STOP_NONE = 0U,
    ZY100_ONLINE_IMU_STOP_HOST_PAUSE,
    ZY100_ONLINE_IMU_STOP_DISCONNECT,
    ZY100_ONLINE_IMU_STOP_LOW_BATTERY,
    ZY100_ONLINE_IMU_STOP_FATAL,
} zy100_online_imu_stop_reason_t;

typedef enum
{
    ZY100_ONLINE_IMU_TERMINAL_NONE = 0U,
    ZY100_ONLINE_IMU_TERMINAL_GRACEFUL_HOST_PAUSE,
    ZY100_ONLINE_IMU_TERMINAL_ABORT,
} zy100_online_imu_terminal_t;

typedef struct
{
    bool active;
    bool busy;
    bool start_pending;
    bool stop_in_progress;
    bool done;
    bool graceful_tail_committed;
    uint32_t completion_generation;
    zy100_online_imu_stop_reason_t stop_reason;
    zy100_online_imu_terminal_t terminal;
} zy100_online_imu_status_t;

/* This is the Online-owned boundary around the board-verified 10375 live FIFO
 * implementation.  Callers must not select Offline V2 through this API. */
bool zy100_online_imu_session_init(void);
bool zy100_online_imu_session_prepare(uint32_t start_round);
bool zy100_online_imu_session_start(void);
bool zy100_online_imu_session_request_maintenance_pause(void);
bool zy100_online_imu_session_request_host_pause(void);
bool zy100_online_imu_session_request_low_battery_stop(void);
bool zy100_online_imu_session_request_abort(void);
bool zy100_online_imu_session_get_status(zy100_online_imu_status_t *status_out);
bool zy100_online_imu_session_take_completion(
    zy100_online_imu_stop_reason_t *reason_out,
    uint32_t *generation_out);
bool zy100_online_imu_session_prepare_sleep(void);
bool zy100_online_imu_session_release_idle_worker(void);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_ONLINE_IMU_SESSION_H */
