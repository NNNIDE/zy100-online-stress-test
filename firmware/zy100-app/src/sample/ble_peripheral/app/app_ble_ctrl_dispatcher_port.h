#ifndef APP_BLE_CTRL_DISPATCHER_PORT_H
#define APP_BLE_CTRL_DISPATCHER_PORT_H

#include <stdbool.h>
#include <stdint.h>

bool app_ble_power_is_connected(void);
bool app_ble_time_sync_ok(void);
bool app_ble_pairing_ready(void);
bool app_ble_online_quiet_logs_active(void);
bool app_ble_ctrl_port_pairing_started(void);
void app_ble_ctrl_port_mark_time_sync_complete(void);
bool app_ble_ctrl_port_publish_ack(uint8_t conn_id,
                                   uint8_t *ack,
                                   uint16_t len);

#endif /* APP_BLE_CTRL_DISPATCHER_PORT_H */
