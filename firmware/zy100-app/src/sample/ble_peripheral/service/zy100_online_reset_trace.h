#ifndef ZY100_ONLINE_RESET_TRACE_H
#define ZY100_ONLINE_RESET_TRACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ZY100_ONLINE_TRACE_APP_IDLE = 0U,
    ZY100_ONLINE_TRACE_APP_START,
    ZY100_ONLINE_TRACE_APP_CAPTURE,
    ZY100_ONLINE_TRACE_APP_PAUSE_REQUEST,
    ZY100_ONLINE_TRACE_APP_WAIT_IMU_STOP,
    ZY100_ONLINE_TRACE_APP_UPLOAD,
    ZY100_ONLINE_TRACE_APP_ERASE,
    ZY100_ONLINE_TRACE_APP_RESUME,
    ZY100_ONLINE_TRACE_APP_ABORT,
    ZY100_ONLINE_TRACE_APP_FINAL_STOP,
} zy100_online_trace_app_phase_t;

typedef enum
{
    ZY100_ONLINE_TRACE_IMU_IDLE = 0U,
    ZY100_ONLINE_TRACE_IMU_LIVE,
    ZY100_ONLINE_TRACE_IMU_B_BEGIN,
    ZY100_ONLINE_TRACE_IMU_B_RUN,
    ZY100_ONLINE_TRACE_IMU_B_CLEANUP,
    ZY100_ONLINE_TRACE_IMU_B_RESTORE,
    ZY100_ONLINE_TRACE_IMU_REPLAY,
    ZY100_ONLINE_TRACE_IMU_STOP_LIVE,
    ZY100_ONLINE_TRACE_IMU_DRAIN,
    ZY100_ONLINE_TRACE_IMU_FINISH_RECORD,
    ZY100_ONLINE_TRACE_IMU_RAW_FINAL,
    ZY100_ONLINE_TRACE_IMU_RECORD_FINAL,
    ZY100_ONLINE_TRACE_IMU_STOP_DONE,
    ZY100_ONLINE_TRACE_IMU_ABORT,
} zy100_online_trace_imu_phase_t;

typedef enum
{
    ZY100_OFFLINE_TRACE_APP_IDLE = 0U,
    ZY100_OFFLINE_TRACE_APP_START,
    ZY100_OFFLINE_TRACE_APP_RUNNING,
    ZY100_OFFLINE_TRACE_APP_STOPPING,
    ZY100_OFFLINE_TRACE_APP_FINALIZING,
    ZY100_OFFLINE_TRACE_APP_ERROR,
} zy100_offline_trace_app_phase_t;

typedef enum
{
    ZY100_OFFLINE_TRACE_IMU_IDLE = 0U,
    ZY100_OFFLINE_TRACE_IMU_START,
    ZY100_OFFLINE_TRACE_IMU_LIVE,
    ZY100_OFFLINE_TRACE_IMU_STALL,
    ZY100_OFFLINE_TRACE_IMU_FATAL,
    ZY100_OFFLINE_TRACE_IMU_STOP,
} zy100_offline_trace_imu_phase_t;

typedef enum
{
    ZY100_OFFLINE_TRACE_SERVICE_IDLE = 0U,
    ZY100_OFFLINE_TRACE_SERVICE_DUE,
    ZY100_OFFLINE_TRACE_SERVICE_OVERDUE,
    ZY100_OFFLINE_TRACE_SERVICE_EMERGENCY,
} zy100_offline_trace_service_t;

typedef enum
{
    ZY100_POWER_TRACE_IDLE = 0U,
    ZY100_POWER_TRACE_STANDBY_PRESS,
    ZY100_POWER_TRACE_SHUTDOWN_PREPARE,
    ZY100_POWER_TRACE_WAIT_RELEASE,
    ZY100_POWER_TRACE_BLE_QUIESCE,
    ZY100_POWER_TRACE_SAFETY_CHECK,
    ZY100_POWER_TRACE_WAKE_REARM,
    ZY100_POWER_TRACE_COMMIT,
    ZY100_POWER_TRACE_ABORT,
    ZY100_POWER_TRACE_BUTTON_TIER,
    ZY100_POWER_TRACE_LED_BACKEND,
    ZY100_POWER_TRACE_LED_POWER,
    ZY100_POWER_TRACE_LED_FRAME,
    ZY100_POWER_TRACE_LED_HINT,
    ZY100_POWER_TRACE_CORE_READY,
    ZY100_POWER_TRACE_YHM_CONVERGE,
} zy100_power_trace_phase_t;

typedef enum
{
    ZY100_POWER_LED_STAGE_NONE = 0U,
    ZY100_POWER_LED_STAGE_DATA_LOW,
    ZY100_POWER_LED_STAGE_POWER_GPIO,
    ZY100_POWER_LED_STAGE_LEVEL_READ,
    ZY100_POWER_LED_STAGE_SETTLE_DELAY,
    ZY100_POWER_LED_STAGE_RGB_INIT,
    ZY100_POWER_LED_STAGE_TIMING,
    ZY100_POWER_LED_STAGE_ALL_OFF,
    ZY100_POWER_LED_STAGE_RENDER_ACTIVE_STATE,
} zy100_power_led_stage_t;

typedef enum
{
    ZY100_POWER_YHM_CONVERGENCE_NONE = 0U,
    ZY100_POWER_YHM_CONVERGENCE_OK,
    ZY100_POWER_YHM_CONVERGENCE_DEGRADED_SETTLED,
    ZY100_POWER_YHM_CONVERGENCE_UNSETTLED,
    ZY100_POWER_YHM_CONVERGENCE_SKIPPED,
} zy100_power_yhm_convergence_t;

void zy100_online_reset_trace_boot_report(void);
void zy100_online_reset_trace_arm(void);
void zy100_online_reset_trace_clear(void);
void zy100_online_reset_trace_set_app_phase(
    zy100_online_trace_app_phase_t phase);
void zy100_online_reset_trace_set_imu_phase(
    zy100_online_trace_imu_phase_t phase);
void zy100_online_reset_trace_set_runtime(uint8_t spi_owner,
                                          uint8_t timer_owner,
                                          bool timer_running,
                                          bool b_critical,
                                          bool raw_pending,
                                          bool record_pending);
void zy100_online_reset_trace_note_feed(void);
void zy100_offline_reset_trace_arm(void);
void zy100_offline_reset_trace_set_app_phase(
    zy100_offline_trace_app_phase_t phase);
void zy100_offline_reset_trace_set_imu_state(
    zy100_offline_trace_imu_phase_t phase,
    bool urgent,
    bool alarm,
    bool rescue,
    zy100_offline_trace_service_t service,
    uint32_t fifo_age_ms);
void zy100_power_reset_trace_arm(zy100_power_trace_phase_t phase,
                                 uint8_t power_state);
void zy100_power_reset_trace_set_phase(zy100_power_trace_phase_t phase,
                                       uint8_t power_state);
void zy100_power_reset_trace_set_stage(zy100_power_trace_phase_t phase);
void zy100_power_reset_trace_set_led_stage(zy100_power_led_stage_t stage);
void zy100_power_reset_trace_set_yhm_convergence(
    zy100_power_yhm_convergence_t convergence);
void zy100_power_reset_trace_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_ONLINE_RESET_TRACE_H */
