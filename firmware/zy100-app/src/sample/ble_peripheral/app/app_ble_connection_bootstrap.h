#ifndef APP_BLE_CONNECTION_BOOTSTRAP_H
#define APP_BLE_CONNECTION_BOOTSTRAP_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    APP_BLE_BOOTSTRAP_STAGE_LINK_CONNECTED = 0U,
    APP_BLE_BOOTSTRAP_STAGE_DISCOVERING_GATT = 1U,
    APP_BLE_BOOTSTRAP_STAGE_READING_DEVICE_INFO = 2U,
    APP_BLE_BOOTSTRAP_STAGE_SUBSCRIBING_ACK = 3U,
    APP_BLE_BOOTSTRAP_STAGE_QUERY = 4U,
    APP_BLE_BOOTSTRAP_STAGE_SECURITY = 5U,
    APP_BLE_BOOTSTRAP_STAGE_LINK_POLICY = 6U,
    APP_BLE_BOOTSTRAP_STAGE_CALIBRATION = 7U,
    APP_BLE_BOOTSTRAP_STAGE_EXPORT = 8U,
    APP_BLE_BOOTSTRAP_STAGE_TIME_SYNC = 9U,
    APP_BLE_BOOTSTRAP_STAGE_CONTROL_READY = 10U,
    APP_BLE_BOOTSTRAP_STAGE_OFFLINE_SYNC = 11U,
    APP_BLE_BOOTSTRAP_STAGE_ONLINE_READY = 12U,
    APP_BLE_BOOTSTRAP_STAGE_BUSINESS_READY = 13U,
    APP_BLE_BOOTSTRAP_STAGE_OFFLINE_CAPTURE_OBSERVER = 14U,
    APP_BLE_BOOTSTRAP_STAGE_USER_SYNC = 15U,
    APP_BLE_BOOTSTRAP_STAGE_FEATURE_CONFIG = 16U,
} app_ble_bootstrap_stage_t;

void app_ble_connection_bootstrap_init(void);
void app_ble_connection_bootstrap_on_connected(uint8_t conn_id);
void app_ble_connection_bootstrap_on_disconnected(uint8_t conn_id);
void app_ble_connection_bootstrap_on_ack_cccd(uint8_t conn_id, bool enabled);
void app_ble_connection_bootstrap_on_security_ready(uint8_t conn_id);
bool app_ble_connection_bootstrap_host_ci_allowed(uint8_t conn_id);
void app_ble_connection_bootstrap_on_host_ci_enabled(uint8_t conn_id);
bool app_ble_connection_bootstrap_user_sync_allowed(uint8_t conn_id);
bool app_ble_connection_bootstrap_user_sync_ready(uint8_t conn_id);
uint32_t app_ble_connection_bootstrap_user_id(uint8_t conn_id);
uint32_t app_ble_connection_bootstrap_current_user_id(void);
bool app_ble_connection_bootstrap_on_user_sync_committed(uint8_t conn_id,
                                                         uint32_t user_id);
bool app_ble_connection_bootstrap_feature_config_sync_allowed(uint8_t conn_id);
bool app_ble_connection_bootstrap_feature_config_ready(uint8_t conn_id);
bool app_ble_connection_bootstrap_business_ready(uint8_t conn_id);
/* Diagnostic admission for an already-qualified local Standby restore only.
 * This does not open normal command admission or publish synthetic ready bits. */
bool app_ble_connection_bootstrap_business_ready_except_link(uint8_t conn_id);
uint32_t app_ble_connection_bootstrap_generation(void);
void app_ble_connection_bootstrap_on_feature_config_pending(uint8_t conn_id);
bool app_ble_connection_bootstrap_on_feature_config_committed(
    uint8_t conn_id,
    uint32_t user_id);
void app_ble_connection_bootstrap_handle_query(uint8_t conn_id);
void app_ble_connection_bootstrap_refresh(uint8_t conn_id);
void app_ble_connection_bootstrap_enter_recovery(uint8_t conn_id,
                                                 app_ble_bootstrap_stage_t stage,
                                                 uint8_t reason,
                                                 uint16_t retry_ms);
void app_ble_connection_bootstrap_maintain(void);

void offline_capture_connection_changed(bool active);

#endif /* APP_BLE_CONNECTION_BOOTSTRAP_H */
