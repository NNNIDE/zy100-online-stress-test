#ifndef IMU_FIFO_DRAIN_TEST_H
#define IMU_FIFO_DRAIN_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "zy100_flash_common.h"
#include "zy100_session_directory.h"

typedef enum
{
    IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE = 0U,
    IMU_FIFO_DRAIN_TEST_STOP_REASON_KEY_SLEEP = 1U,
    IMU_FIFO_DRAIN_TEST_STOP_REASON_FLASH_95_PERCENT = 2U,
    IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL = 3U,
    IMU_FIFO_DRAIN_TEST_STOP_REASON_EDGE_FULL = 4U,
    IMU_FIFO_DRAIN_TEST_STOP_REASON_PAUSED = 5U,
    IMU_FIFO_DRAIN_TEST_STOP_REASON_USER_STOP = 6U,
    IMU_FIFO_DRAIN_TEST_STOP_REASON_BATTERY_LOW = 7U,
} imu_fifo_drain_test_stop_reason_t;

typedef enum
{
    ZY100_FE_STOP_REASON_NONE = 0U,
    ZY100_FE_STOP_REASON_USER_SHORT_PRESS = 1U,
    ZY100_FE_STOP_REASON_FLASH_RAW_95 = 2U,
    ZY100_FE_STOP_REASON_FLASH_SUMMARY_95 = 3U,
    ZY100_FE_STOP_REASON_FLASH_EVENT_95 = 4U,
    ZY100_FE_STOP_REASON_FLASH_META_95 = 5U,
    ZY100_FE_STOP_REASON_ERROR = 6U,
    ZY100_FE_STOP_REASON_BATTERY_LOW = 7U,
} zy100_fe_stop_reason_t;

typedef enum
{
    FE_CAPTURE_STATE_IDLE = 0U,
    FE_CAPTURE_STATE_RUNNING,
    FE_CAPTURE_STATE_STOP_REQUESTED,
    FE_CAPTURE_STATE_DRAIN_B_ACTIVE,
    FE_CAPTURE_STATE_DRAIN_REPLAY,
    FE_CAPTURE_STATE_DRAIN_LIVE_FIFO,
    FE_CAPTURE_STATE_DRAIN_RAW_STORE,
    FE_CAPTURE_STATE_DRAIN_RECORD_STORE,
    FE_CAPTURE_STATE_WRITE_META,
    FE_CAPTURE_STATE_EXPORT_READY,
    FE_CAPTURE_STATE_EXPORTING,
    FE_CAPTURE_STATE_EXPORT_DONE,
    FE_CAPTURE_STATE_EXPORT_CLEANUP,
    FE_CAPTURE_STATE_ERROR,
} zy100_fe_capture_state_t;

typedef struct
{
    uint32_t rt_trigger_count;
    uint32_t last_rt_trigger_ms;
    uint32_t b_scan_count;
    uint32_t official_hit_count;
    uint32_t official_nohit_count;
    uint32_t b_scan_fail_count;
    uint32_t raw_saved_count;
} zy100_fe_ois_activity_stats_t;

typedef struct
{
    zy100_fe_capture_state_t fe_state;
    uint16_t fifo_count;
    bool fifo_count_valid;
    bool capture_active;
    bool start_pending;
    bool stop_in_progress;
    bool b_pending;
    bool b_critical;
    bool rt_due;
    bool replay_busy;
    bool raw_store_wip;
    bool record_store_wip;
    bool spi_idle;
} zy100_online_rt_snapshot_t;

typedef enum
{
    ZY100_ONLINE_PAUSE_PHASE_IDLE = 0U,
    ZY100_ONLINE_PAUSE_PHASE_STOP_LIVE,
    ZY100_ONLINE_PAUSE_PHASE_DRAIN_FIFO_REPLAY,
    ZY100_ONLINE_PAUSE_PHASE_FINISH_ACTIVE_RECORD,
    ZY100_ONLINE_PAUSE_PHASE_RAW_FINAL,
    ZY100_ONLINE_PAUSE_PHASE_RECORD_FINAL,
    ZY100_ONLINE_PAUSE_PHASE_STOP_DONE,
    ZY100_ONLINE_PAUSE_PHASE_ABORT,
} zy100_online_pause_phase_t;

typedef enum
{
    IMU_ONLINE_PREFLIGHT_OK = 0U,
    IMU_ONLINE_PREFLIGHT_RETRY_BUSY,
    IMU_ONLINE_PREFLIGHT_RETRY_RESUME,
    IMU_ONLINE_PREFLIGHT_WORKSPACE_BUSY,
    IMU_ONLINE_PREFLIGHT_PROVIDER_INVALID,
    IMU_ONLINE_PREFLIGHT_FATAL,
} imu_online_preflight_status_t;

typedef struct
{
    zy100_online_pause_phase_t phase;
    uint32_t progress_seq;
    uint32_t phase_start_ms;
    uint32_t last_progress_ms;
    bool active;
    bool abort_requested;
} zy100_online_pause_progress_t;

bool imu_fifo_drain_test_task_init(void);
bool imu_fifo_drain_test_task_release_idle(void);
bool imu_fifo_drain_test_prepare_flash(void);
bool imu_fifo_drain_test_prepare_flash_begin(void);
bool imu_fifo_drain_test_prepare_flash_append_begin(
    const zy100_session_alloc_t *alloc);
bool imu_fifo_drain_test_prepare_online_begin(uint32_t round);
imu_online_preflight_status_t
imu_fifo_drain_test_prepare_continuous_online(uint32_t round);
void imu_fifo_drain_test_prepare_continuous_online_abort(void);
zy100_flash_prepare_status_t imu_fifo_drain_test_prepare_flash_poll(void);
void imu_fifo_drain_test_clear_active_session_alloc(void);
bool imu_fifo_drain_test_prepare_erased_flash(void);
#if ZY100_LEGACY_OFFLINE_ENABLE
bool imu_fifo_drain_test_prepare_offline_v2(void);
#endif
bool imu_fifo_drain_test_start(void);
void imu_fifo_drain_test_request_stop(imu_fifo_drain_test_stop_reason_t reason);
bool imu_fifo_drain_test_request_online_pause(void);
bool imu_fifo_drain_test_request_online_pause_with_reason(
    imu_fifo_drain_test_stop_reason_t reason);
bool imu_fifo_drain_test_request_online_abort(void);
bool imu_fifo_drain_test_request_online_abort_with_reason(
    imu_fifo_drain_test_stop_reason_t reason);
bool imu_fifo_drain_test_is_active(void);
bool imu_fifo_drain_test_is_busy(void);
bool imu_fifo_drain_test_is_stop_in_progress(void);
bool imu_fifo_drain_test_is_start_pending(void);
bool imu_fifo_drain_test_is_done(void);
uint32_t imu_fifo_drain_test_completion_generation(void);
bool imu_fifo_drain_test_adc_safe_window(uint32_t now_ms);
bool imu_fifo_drain_test_prepare_for_sleep(void);
uint32_t imu_fifo_drain_test_peek_next_round(void);
uint32_t imu_fifo_drain_test_peek_start_round(void);
uint32_t imu_fifo_drain_test_current_round(void);
imu_fifo_drain_test_stop_reason_t imu_fifo_drain_test_last_stop_reason(void);
zy100_fe_capture_state_t imu_fifo_drain_test_final_edge_capture_state(void);
void imu_fifo_drain_test_get_online_rt_snapshot(
    zy100_online_rt_snapshot_t *out);
void imu_fifo_drain_test_get_online_pause_progress(
    zy100_online_pause_progress_t *out);
zy100_fe_stop_reason_t imu_fifo_drain_test_final_edge_stop_reason(void);
bool imu_fifo_drain_test_final_edge_get_ois_activity_stats(
    zy100_fe_ois_activity_stats_t *out);
bool imu_fifo_drain_test_final_edge_b_critical_active(void);
void imu_fifo_drain_test_online_diag_reset(void);
void imu_fifo_drain_test_online_diag_log_summary(void);
bool imu_fifo_drain_test_online_diag_log_summary_step(void);
bool imu_fifo_drain_test_final_edge_export_ready(void);
bool imu_fifo_drain_test_final_edge_empty_error_aborted(void);
void imu_fifo_drain_test_final_edge_set_exporting(void);
void imu_fifo_drain_test_final_edge_set_export_ready_retry(void);
void imu_fifo_drain_test_final_edge_set_export_done(void);
void imu_fifo_drain_test_final_edge_set_export_cleanup(void);
void imu_fifo_drain_test_final_edge_mark_export_cleared(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FIFO_DRAIN_TEST_H */
