#ifndef ZY100_OFFLINE_V2_CAPTURE_H
#define ZY100_OFFLINE_V2_CAPTURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../storage/zy100_offline_v2_storage.h"
#include "zy100_training_snapshot.h"

bool zy100_offline_v2_capture_snapshot(zy100_training_snapshot_t *out);
void zy100_offline_v2_capture_set_start_source(uint8_t source);
bool zy100_offline_v2_capture_token_matches(uint32_t user_id, const uint8_t token[16]);

typedef enum
{
    ZY100_OFFLINE_V2_CAPTURE_UNINITIALIZED = 0U,
    ZY100_OFFLINE_V2_CAPTURE_IDLE,
    ZY100_OFFLINE_V2_CAPTURE_STARTING,
    ZY100_OFFLINE_V2_CAPTURE_RUNNING,
    ZY100_OFFLINE_V2_CAPTURE_STOPPING,
    ZY100_OFFLINE_V2_CAPTURE_FINALIZING,
    ZY100_OFFLINE_V2_CAPTURE_LOCKED,
    ZY100_OFFLINE_V2_CAPTURE_ERROR,
} zy100_offline_v2_capture_state_t;

typedef enum
{
    ZY100_OFFLINE_V2_START_OK = 0U,
    ZY100_OFFLINE_V2_START_NOT_READY,
    ZY100_OFFLINE_V2_START_PROFILE_BUSY,
    ZY100_OFFLINE_V2_START_WORKSPACE_BUSY,
    ZY100_OFFLINE_V2_START_FLASH_LOCKED,
    ZY100_OFFLINE_V2_START_INTERNAL_ERROR,
} zy100_offline_v2_start_status_t;

typedef struct
{
    zy100_offline_v2_algo_status_t algo_status;
    uint32_t event_queue_level;
    uint32_t event_queue_max_level;
    uint32_t feature_drop_count;
    uint32_t push_last_us;
    uint32_t push_max_us;
    uint32_t detector_last_us;
    uint32_t detector_max_us;
    uint32_t shock_last_us;
    uint32_t shock_max_us;
    uint32_t feature_last_us;
    uint32_t feature_max_us;
    uint32_t local_peak_count;
    uint32_t legal_peak_count;
    uint32_t nms_selected_count;
    uint32_t shock_rejected_count;
    uint32_t rejected_window_count;
    uint32_t imu_sample_count;
    uint32_t mag_sample_count;
    uint32_t imu_fifo_max_bytes;
    uint32_t imu_fifo_full_count;
    uint32_t imu_fifo_lost_count;
    uint32_t imu_fifo_bad_header_count;
    uint32_t imu_fifo_read_error_count;
    uint32_t imu_cause_origin;
    uint32_t imu_cause_detail;
} zy100_offline_v2_capture_diag_t;

typedef enum
{
    ZY100_OFFLINE_V2_OBSERVER_WINDOW_OK = 0U,
    ZY100_OFFLINE_V2_OBSERVER_WINDOW_NOT_RUNNING,
    ZY100_OFFLINE_V2_OBSERVER_WINDOW_STOPPING,
    ZY100_OFFLINE_V2_OBSERVER_WINDOW_EVENT_QUEUE,
    ZY100_OFFLINE_V2_OBSERVER_WINDOW_STORAGE_BUSY,
    ZY100_OFFLINE_V2_OBSERVER_WINDOW_IMU_BUSY,
    ZY100_OFFLINE_V2_OBSERVER_WINDOW_FIRST_CAUSE,
    ZY100_OFFLINE_V2_OBSERVER_WINDOW_CONSUMED,
} zy100_offline_v2_observer_window_result_t;

/* Shutdown lifecycle: DONE is business-ready; ERROR may only be sleep-safe. */
void zy100_offline_v2_capture_shutdown_begin(void);
void zy100_offline_v2_capture_prepare_wake(void);
void zy100_offline_v2_capture_shutdown_poll(uint32_t now_ms);
zy100_offline_v2_drain_status_t zy100_offline_v2_capture_shutdown_status(void);
bool zy100_offline_v2_capture_fault_exit(void);
/* Software drain only; this does not authorize power-off without HW idle. */
bool zy100_offline_v2_capture_fault_drained(void);
bool zy100_offline_v2_capture_fault_sleep_safe(void);
bool zy100_offline_v2_capture_init(void);
bool zy100_offline_v2_capture_init_with_policy(
    zy100_offline_v2_boot_policy_t policy);
void zy100_offline_v2_capture_poll(uint32_t now_ms);
zy100_offline_v2_start_status_t zy100_offline_v2_capture_start(
    uint64_t start_unix_ms,
    bool timebase_synced,
    uint32_t owner_user_id);
bool zy100_offline_v2_capture_request_stop(
    zy100_offline_v2_stop_reason_t reason);
bool zy100_offline_v2_capture_push_imu_packet(const uint8_t packet[16],
                                              uint16_t timestamp_raw);
zy100_offline_v2_capture_state_t zy100_offline_v2_capture_state(void);
bool zy100_offline_v2_capture_active(void);
bool zy100_offline_v2_capture_input_active(void);
bool zy100_offline_v2_capture_locked(void);
uint8_t zy100_offline_v2_capture_remaining_percent(void);
uint32_t zy100_offline_v2_capture_session_count(void);
bool zy100_offline_v2_capture_take_completion(
    zy100_offline_v2_stop_reason_t *reason_out);
bool zy100_offline_v2_capture_get_diag(
    zy100_offline_v2_capture_diag_t *diag_out);
bool zy100_offline_v2_capture_observer_window_take(
    uint32_t *generation_out,
    zy100_offline_v2_observer_window_result_t *result_out);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_OFFLINE_V2_CAPTURE_H */
