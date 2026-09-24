#ifndef APP_BLE_SENSOR_STREAM_PORT_H
#define APP_BLE_SENSOR_STREAM_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "app_ble_power_policy.h"

bool app_ble_power_is_connected(void);
uint8_t app_ble_power_conn_id(void);
bool app_ble_power_conn_valid(uint8_t conn_id);
bool app_ble_sensor_stream_port_send_v3_notify(uint8_t conn_id,
                                               void *payload,
                                               uint16_t payload_len);
bool app_ble_sensor_stream_port_legacy_ota_active(void);
#define app_ble_sensor_stream_port_mark_gap_disconnected() \
    ((void)app_ble_power_policy_note_conn_state( \
        GAP_CONN_STATE_DISCONNECTED))
#define app_ble_sensor_stream_port_restart_advertising(reason) \
    app_ble_power_policy_restart_advertising_if_idle(reason)

#endif /* APP_BLE_SENSOR_STREAM_PORT_H */
