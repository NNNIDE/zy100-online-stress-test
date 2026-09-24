/**
*****************************************************************************************
*     Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      app_task.h
   * @brief     Routines to create App task and handle events & messages
   * @author    jane
   * @date      2017-06-02
   * @version   v1.0
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2017 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */
#ifndef _APP_TASK_H_
#define _APP_TASK_H_
#include "service/zy100_training_snapshot.h"
bool app_task_offline_training_snapshot(zy100_training_snapshot_t *out);

#include <stdbool.h>
#include <stdint.h>

#include <app_msg.h>
#include <profile_server.h>
#include "app_flags.h"
#include "app/app_training_context.h"
#include "app/app_device_pairing.h"
#include "app/app_battery_policy.h"
#include "app/app_button.h"
#include "app/app_ota_controller.h"
#include "service/zy100_feature_config.h"

#ifndef F_APP_CLOSE_UART_LOG_BEFORE_DLPS
#define F_APP_CLOSE_UART_LOG_BEFORE_DLPS (!ZY100_BLE_TARGETED_LOG_ENABLE)
#endif

/** @defgroup PERIPH_APP_TASK Peripheral App Task
  * @brief Peripheral App Task
  * @{
  */

extern void driver_init(void);
/* Connection-local receipt; execution stays in the application poll. */
bool app_task_timeout_action_pending(void);
bool app_task_timeout_action_ack(uint8_t conn_id, uint8_t seq,
    uint32_t generation, uint32_t token, uint64_t reason);
typedef struct
{
    uint32_t timestamp_ms;
    uint8_t level;
} app_button_input_edge_t;

bool app_button_read_level(uint8_t *level);
bool app_button_input_take_edge(app_button_input_edge_t *edge);
bool app_button_input_edge_pending(void);
/* Fault remains latched until input flush/disable completes recovery. */
bool app_button_input_take_fault(void);
bool app_button_input_capture_active(void);
/* Application input mode survives incidental BLE/DLPS wakeups. */
bool app_task_button_input_wants_edges(void);
void app_button_input_set_capture_enabled(bool enabled);
uint8_t app_button_input_flush(void);
bool app_button_dlps_rearm_wakeup(void);
bool app_button_dlps_rearm_release_wakeup(void);
void app_button_quiesce_wakeup_irq(void);
#if F_BT_DLPS_EN && F_APP_BUTTON_DLPS_CTRL_ENABLE
bool app_task_button_sleep_release_guard_active(void);
#endif
#if F_BT_DLPS_EN && F_APP_BUTTON_DLPS_CTRL_ENABLE
bool app_task_imu_wom_sleep_is_armed(void);
bool app_task_rearm_dlps_wakeup_sources(void);
void app_imu_wom_notify_wakeup_irq(void);
void app_imu_wom_notify_app_task_from_isr(void);
void app_charge_notify_wakeup_irq(void);
void app_charge_notify_app_task_from_isr(void);
#endif
#if F_APP_BUTTON_LOG_ENABLE && F_BT_DLPS_EN
void app_button_dlps_get_counts(uint32_t *enter_count, uint32_t *exit_count);
#endif
void app_button_notify_wakeup_irq(void);
void app_button_notify_app_task_from_isr(void);
bool app_task_v0_dlps_check(void);

bool app_task_ota_try_admit(uint8_t conn_id,
                            app_ota_source_t source,
                            bool ack_required,
                            app_ota_reject_reason_t *reason_out);
void app_task_ota_accept_ack_result(bool submitted);
void app_task_ota_on_notify_complete(bool success);
void app_task_ota_cancel_admission(app_ota_reject_reason_t reason);
void app_task_ota_on_disconnect(uint8_t conn_id);
bool app_task_ota_blocks_new_business(void);
bool app_task_shutdown_blocks_new_business(void);
bool app_task_ble_disconnect_should_restore_active(void);
bool app_task_ble_disconnect_restore_active(const char *reason);
void app_task_request_ota_mode(void);
void app_task_request_ota_mode_delayed(uint32_t delay_ms);
void app_task_auto_idle_note_activity(const char *reason);
void app_task_auto_idle_note_ble_command(uint8_t cmd);
typedef enum
{
    APP_CAL_PREPARE_WAIT = 0,
    APP_CAL_PREPARE_READY,
    APP_CAL_PREPARE_FAILED,
} app_cal_prepare_result_t;
app_cal_prepare_result_t app_task_calibration_prepare_start(void);
#if F_APP_BATTERY_ADC_ENABLE && F_APP_BATTERY_LPC_GUARD_ENABLE
bool app_task_post_io_msg_from_isr(const T_IO_MSG *msg);
void app_task_battery_low_lpc_event_handle(uint32_t runtime_ms);
#endif
#if F_APP_BATTERY_ADC_ENABLE && F_APP_BATTERY_ADC_GUARD_ENABLE
void app_task_battery_adc_guard_low_event_handle(uint32_t runtime_ms,
                                                 uint16_t mv,
                                                 uint8_t percent);
void app_task_battery_adc_guard_fault_event_handle(uint32_t runtime_ms);
#endif

typedef enum
{
    APP_BLE_CTRL_REQ_RESULT_OK = 0,
    APP_BLE_CTRL_REQ_RESULT_BUSY,
    APP_BLE_CTRL_REQ_RESULT_NOT_READY,
    APP_BLE_CTRL_REQ_RESULT_INVALID_STATE,
    APP_BLE_CTRL_REQ_RESULT_ID_MISMATCH,
    APP_BLE_CTRL_REQ_RESULT_EXPORT_READY_HAS_DATA,
    APP_BLE_CTRL_REQ_RESULT_INTERNAL_ERROR,
} app_ble_ctrl_req_result_t;

app_ble_ctrl_req_result_t app_task_ble_ctrl_request_ota_prepare(uint8_t conn_id);
app_ble_ctrl_req_result_t app_task_ble_ctrl_request_ota_link_intent(
    uint8_t conn_id);
uint32_t app_task_ble_ctrl_ota_prepare_detail(void);
app_ble_ctrl_req_result_t app_task_ble_ctrl_request_enter_shipping(
    uint8_t conn_id,
    uint8_t seq,
    uint32_t user_id,
    uint32_t training_id);

typedef enum
{
    APP_TRAINING_META_REPAIR_REASON_STALE_META_NO_PENDING = 0,
    APP_TRAINING_META_REPAIR_REASON_START_STALE_META_NO_PENDING,
} app_training_meta_repair_reason_t;

app_ble_ctrl_req_result_t app_task_ble_ctrl_request_start(uint8_t seq,
                                                          uint32_t user_id,
                                                          uint64_t time_ms,
                                                          uint32_t training_id);
uint32_t app_task_ble_ctrl_start_detail(void);
app_ble_ctrl_req_result_t app_task_ble_ctrl_request_pause(uint8_t seq,
                                                          uint32_t user_id,
                                                          uint64_t time_ms,
                                                          uint32_t training_id);
app_ble_ctrl_req_result_t app_task_ble_ctrl_request_offline_start(
    uint8_t seq,
    uint32_t user_id,
    uint64_t time_ms,
    uint32_t training_id);
app_ble_ctrl_req_result_t app_task_ble_ctrl_request_offline_stop(
    uint8_t seq,
    uint32_t user_id,
    uint64_t time_ms,
    uint32_t training_id);
uint32_t app_task_ble_ctrl_offline_detail(void);
app_ble_ctrl_req_result_t app_task_ble_ctrl_request_clear_flash(uint8_t seq,
                                                                uint32_t user_id,
                                                                uint64_t time_ms,
                                                                uint32_t training_id);
app_ble_ctrl_req_result_t app_task_ble_ctrl_request_find_device(uint8_t seq);
app_ble_ctrl_req_result_t app_task_ble_ctrl_request_feature_config(
    uint8_t conn_id,
    uint8_t seq,
    uint32_t user_id,
    const zy100_feature_config_t *config,
    uint32_t config_crc32);
bool app_task_feature_config_sync_allowed(void);
bool app_task_feature_config_dry_run_begin(uint8_t conn_id, uint8_t seq);
void app_task_feature_config_ack_submitted(uint8_t conn_id,
                                           uint8_t seq,
                                           bool submitted);
app_ble_ctrl_req_result_t app_task_ble_export_request_confirm(uint8_t seq,
                                                              uint32_t export_id,
                                                              uint64_t host_time_ms,
                                                              uint32_t result);
app_ble_ctrl_req_result_t app_task_ble_ctrl_request_online_ready(uint8_t seq,
                                                                 uint8_t conn_id,
                                                                 uint32_t capability_mask,
                                                                 uint64_t host_time_ms);
app_ble_ctrl_req_result_t app_task_ble_ctrl_request_online_record_ack(uint8_t seq,
                                                                      uint8_t conn_id,
                                                                      uint32_t session_id,
                                                                      uint64_t record_id,
                                                                      uint32_t ack_info);
void app_task_ble_ctrl_online_disconnected(uint8_t conn_id);
void app_task_ble_ctrl_wake(const char *reason);
void app_task_ble_ctrl_online_transport_disabled(uint8_t conn_id);
uint8_t app_task_ble_ctrl_device_state(void);
bool app_task_calibration_start_ready(void);
void app_task_ble_device_state_notify_force(const char *reason);
void app_task_ble_device_state_notify_reset(const char *reason);
void app_task_capture_led_update_by_state(const char *reason);
void app_task_ble_link_led_notify(bool connected, const char *reason);
void app_task_training_meta_repair_request(app_training_meta_repair_reason_t reason);
bool app_task_training_meta_repair_needed(void);
bool app_task_device_logic_ready(void);
bool app_task_pairing_bonded_persisted(void);
void app_ble_export_try_auto_start(const char *reason);
void app_ble_export_poll(void);
void app_ble_export_on_disconnect(uint8_t conn_id);
bool app_ble_export_is_active(void);
bool app_ble_export_blocks_sleep(void);
void app_ble_export_on_send_data_complete(uint8_t conn_id,
                                          T_SERVER_ID service_id,
                                          uint16_t attrib_idx,
                                          uint16_t cause,
                                          uint16_t credits);
#if ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
void app_ble_export_on_conn_param_snapshot(uint8_t conn_id,
                                           uint16_t interval,
                                           uint16_t latency,
                                           uint16_t timeout);
void app_ble_export_on_conn_param_update(uint8_t conn_id,
                                         uint16_t interval,
                                         uint16_t latency,
                                         uint16_t timeout,
                                         uint16_t cause);
#endif
void app_task_multi_session_refresh_readiness(const char *reason);
uint32_t app_task_multi_session_cached_pending_count(void);
bool app_training_flash_has_pending_sessions(void);
uint32_t app_training_flash_pending_session_count(void);
uint32_t app_training_flash_pending_total_bytes_estimate(void);

/**
 * @brief  Initialize App task
 * @return void
 */
void app_task_init(void);


/** End of PERIPH_APP_TASK
* @}
*/


#endif

