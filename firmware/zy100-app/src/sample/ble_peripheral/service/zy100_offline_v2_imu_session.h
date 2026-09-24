#ifndef ZY100_OFFLINE_V2_IMU_SESSION_H
#define ZY100_OFFLINE_V2_IMU_SESSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ZY100_OFFLINE_V2_IMU_IDLE = 0U,
    ZY100_OFFLINE_V2_IMU_PREPARED,
    ZY100_OFFLINE_V2_IMU_START_PENDING,
    ZY100_OFFLINE_V2_IMU_RUNNING,
    ZY100_OFFLINE_V2_IMU_STOPPING,
    ZY100_OFFLINE_V2_IMU_DONE,
    ZY100_OFFLINE_V2_IMU_ERROR,
} zy100_offline_v2_imu_state_t;

typedef enum
{
    ZY100_OFFLINE_V2_IMU_STOP_NONE = 0U,
    ZY100_OFFLINE_V2_IMU_STOP_USER,
    ZY100_OFFLINE_V2_IMU_STOP_LOW_BATTERY,
    ZY100_OFFLINE_V2_IMU_STOP_FLASH_FULL,
    ZY100_OFFLINE_V2_IMU_STOP_SYSTEM_SHUTDOWN,
    ZY100_OFFLINE_V2_IMU_STOP_SENSOR_START,
    ZY100_OFFLINE_V2_IMU_STOP_FIFO_FULL,
    ZY100_OFFLINE_V2_IMU_STOP_FIFO_LOST,
    ZY100_OFFLINE_V2_IMU_STOP_FIFO_BAD_HEADER,
    ZY100_OFFLINE_V2_IMU_STOP_FIFO_READ,
    ZY100_OFFLINE_V2_IMU_STOP_ALGORITHM,
    ZY100_OFFLINE_V2_IMU_STOP_WORKER_EXIT,
} zy100_offline_v2_imu_stop_reason_t;

typedef enum
{
    ZY100_OFFLINE_V2_IMU_ORIGIN_NONE = 0U,
    ZY100_OFFLINE_V2_IMU_ORIGIN_CONTROLLER,
    ZY100_OFFLINE_V2_IMU_ORIGIN_PREPARE,
    ZY100_OFFLINE_V2_IMU_ORIGIN_FIFO_INT,
    ZY100_OFFLINE_V2_IMU_ORIGIN_FIFO_LOST,
    ZY100_OFFLINE_V2_IMU_ORIGIN_FIFO_COUNT,
    ZY100_OFFLINE_V2_IMU_ORIGIN_FIFO_READ,
    ZY100_OFFLINE_V2_IMU_ORIGIN_FIFO_PACKET,
    ZY100_OFFLINE_V2_IMU_ORIGIN_ALGORITHM,
    ZY100_OFFLINE_V2_IMU_ORIGIN_FINAL_DRAIN,
    ZY100_OFFLINE_V2_IMU_ORIGIN_WORKER,
} zy100_offline_v2_imu_origin_t;

typedef struct
{
    bool valid;
    zy100_offline_v2_imu_stop_reason_t reason;
    zy100_offline_v2_imu_origin_t origin;
    uint32_t detail;
    uint32_t time_ms;
    uint32_t accepted_packets;
    uint16_t fifo_count;
    uint16_t lost_count;
    uint16_t request_len;
    uint16_t actual_len;
    uint16_t buffer_offset;
    uint8_t header;
    uint8_t int_status;
} zy100_offline_v2_imu_cause_t;

typedef struct
{
    zy100_offline_v2_imu_state_t state;
    zy100_offline_v2_imu_stop_reason_t stop_reason;
    uint32_t completion_generation;
    uint32_t accepted_packets;
    uint32_t service_count;
    uint32_t emergency_count;
    uint32_t read_error_count;
    uint32_t bad_header_count;
    uint32_t fifo_full_count;
    uint32_t fifo_lost_count;
    uint32_t pending_due;
    uint32_t flash_busy_defer_count;
    uint32_t flash_busy_max_consecutive;
    uint32_t flash_busy_max_us;
    uint32_t last_progress_ms;
    uint16_t last_fifo_count;
    uint16_t max_fifo_count;
    bool timer_running;
    bool stop_requested;
    bool flash_busy_active;
} zy100_offline_v2_imu_status_t;

bool zy100_offline_v2_imu_session_init(void);
bool zy100_offline_v2_imu_session_prepare(void);
bool zy100_offline_v2_imu_session_start(void);
bool zy100_offline_v2_imu_session_request_stop(
    zy100_offline_v2_imu_stop_reason_t reason);
void zy100_offline_v2_imu_session_poll(uint32_t now_ms);
bool zy100_offline_v2_imu_session_get_status(
    zy100_offline_v2_imu_status_t *status_out);
bool zy100_offline_v2_imu_session_get_first_cause(
    zy100_offline_v2_imu_cause_t *cause_out);
bool zy100_offline_v2_imu_session_take_completion(
    zy100_offline_v2_imu_stop_reason_t *reason_out);
bool zy100_offline_v2_imu_session_active(void);
bool zy100_offline_v2_imu_session_quiescent(void);
bool zy100_offline_v2_imu_session_prepare_sleep(void);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_OFFLINE_V2_IMU_SESSION_H */
