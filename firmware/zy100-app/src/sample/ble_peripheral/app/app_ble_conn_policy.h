#ifndef APP_BLE_CONN_POLICY_H
#define APP_BLE_CONN_POLICY_H

#include <stdbool.h>
#include <stdint.h>

void app_ble_runtime_conn_begin(const char *reason);
void app_ble_runtime_conn_maintain(void);
void app_ble_export_fast_conn_begin(uint8_t conn_id);
bool app_ble_export_fast_conn_ready_or_timeout(void);
void app_ble_export_fast_conn_maintain(uint32_t now_os_ms);
void app_ble_export_fast_conn_end(bool restore);
bool app_ble_export_fast_conn_matches(uint8_t conn_id);
void app_ble_export_conn_param_work(void);
void app_ble_export_on_conn_param_snapshot(uint8_t conn_id,
                                           uint16_t interval,
                                           uint16_t latency,
                                           uint16_t timeout);
void app_ble_export_on_conn_param_update(uint8_t conn_id,
                                         uint16_t interval,
                                         uint16_t latency,
                                         uint16_t timeout,
                                         uint16_t cause);

bool app_ble_conn_policy_port_post_work_event(void);
void app_ble_conn_policy_port_wake_ctrl(void);
bool app_ble_conn_policy_port_online_quiet(void);
bool app_ble_conn_policy_port_capture_active(void);
bool app_ble_export_is_active(void);

#endif
