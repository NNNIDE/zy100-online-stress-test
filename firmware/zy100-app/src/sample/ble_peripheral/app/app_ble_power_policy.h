#ifndef APP_BLE_POWER_POLICY_H
#define APP_BLE_POWER_POLICY_H

#include <stdbool.h>
#include <stdint.h>
#include <gap_le.h>
#include <gap_msg.h>

#include "app_ble_ci_state.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    APP_BLE_DISCONNECT_RECOVERY_DEFAULT = 0U,
    APP_BLE_DISCONNECT_RECOVERY_DEFER_ACTIVE_RESTORE,
} app_ble_disconnect_recovery_t;

void app_handle_dev_state_evt(T_GAP_DEV_STATE new_state, uint16_t cause);

void app_ble_power_set_boot_storage_ready(bool ready);
void app_ble_power_on_wake(void);
void app_ble_power_on_wake_hold_adv(const char *reason);
void app_ble_enter_standby_policy(const char *reason);
void app_ble_exit_standby_policy_begin(const char *reason);
void app_ble_exit_standby_policy_complete(const char *reason,
                                          app_ble_ci_state_t target_state);
bool app_ble_prepare_for_button_only_dlps(void);
void app_ble_power_shutdown_latch(void);
void app_ble_power_shutdown_release_on_wake(void);
bool app_ble_power_shutdown_latched(void);
bool app_ble_prepare_for_ota_quiesce(void);
void app_ble_recover_after_button_only_sleep_fail(void);
void app_ble_recover_after_ota_cancel(void);
bool app_ble_power_is_quiesced(void);
void app_ble_power_on_stack_ready(void);
bool app_ble_power_is_connected(void);
bool app_ble_power_stack_ready(void);
uint8_t app_ble_power_conn_id(void);
bool app_ble_power_conn_valid(uint8_t conn_id);
uint16_t app_ble_power_conn_mtu(uint8_t conn_id);
void app_ble_power_log_runtime_dlps_check(void);
void app_ble_standby_clear_conn_param(void);

T_GAP_CONN_STATE app_ble_power_policy_note_conn_state(
    T_GAP_CONN_STATE new_state);
T_GAP_CONN_STATE app_ble_power_policy_conn_state(void);
bool app_ble_power_policy_standby_active(void);
void app_ble_power_policy_on_connected(uint8_t conn_id);
void app_ble_power_policy_on_disconnected(
    uint8_t conn_id, app_ble_disconnect_recovery_t recovery);
void app_ble_power_policy_complete_deferred_disconnect_restore(
    const char *reason);
void app_ble_power_policy_disconnect_if_required(void);
void app_ble_power_policy_restart_advertising_if_idle(const char *reason);

#ifdef __cplusplus
}
#endif

#endif
