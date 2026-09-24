#ifndef APP_BLE_CONN_PARAM_MGR_PORT_H
#define APP_BLE_CONN_PARAM_MGR_PORT_H

#include <stdbool.h>
#include <stdint.h>

bool app_ble_power_conn_valid(uint8_t conn_id);
bool app_ble_ci_state_peripheral_llcp_allowed(uint16_t ci_min,
                                              uint16_t ci_max,
                                              uint16_t latency,
                                              uint16_t timeout);
bool app_ble_online_quiet_logs_active(void);
void app_ble_standby_clear_conn_param(void);
void app_ble_ci_state_maintain_notify_and_disconnect(void);

#endif /* APP_BLE_CONN_PARAM_MGR_PORT_H */
