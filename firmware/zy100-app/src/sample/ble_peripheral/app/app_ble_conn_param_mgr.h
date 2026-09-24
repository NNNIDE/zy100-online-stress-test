#ifndef APP_BLE_CONN_PARAM_MGR_H
#define APP_BLE_CONN_PARAM_MGR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    APP_BLE_CONN_PARAM_OWNER_STANDBY_LOW_POWER = 0x01U,
    APP_BLE_CONN_PARAM_OWNER_STANDBY_RESTORE = 0x02U,
    APP_BLE_CONN_PARAM_OWNER_RUNTIME_HIGH = 0x04U,
    APP_BLE_CONN_PARAM_OWNER_EXPORT_HIGH = 0x08U,
    APP_BLE_CONN_PARAM_OWNER_EXPORT_RESTORE = 0x10U,
    APP_BLE_CONN_PARAM_OWNER_CI_STATE = 0x20U,
} app_ble_conn_param_owner_t;

typedef enum
{
    APP_BLE_CONN_PARAM_REQ_FAILED = 0,
    APP_BLE_CONN_PARAM_REQ_SUBMITTED,
    APP_BLE_CONN_PARAM_REQ_WAIT_SAME_TARGET,
    APP_BLE_CONN_PARAM_REQ_ALREADY_REACHED,
    APP_BLE_CONN_PARAM_REQ_BUSY,
} app_ble_conn_param_request_result_t;

app_ble_conn_param_request_result_t app_ble_conn_param_mgr_request(
    app_ble_conn_param_owner_t owner,
    uint8_t conn_id,
    uint16_t ci_min,
    uint16_t ci_max,
    uint16_t latency,
    uint16_t timeout,
    const char *reason);
void app_ble_conn_param_mgr_on_gap_update(uint8_t conn_id,
                                          uint8_t status,
                                          uint16_t cause,
                                          uint16_t actual_ci,
                                          uint16_t actual_latency,
                                          uint16_t actual_timeout);
void app_ble_conn_param_mgr_maintain(void);
bool app_ble_conn_param_mgr_is_settled(uint8_t conn_id);
bool app_ble_conn_param_mgr_high_ready(uint8_t conn_id,
                                       uint16_t ci_min,
                                       uint16_t ci_max,
                                       uint16_t latency);
bool app_ble_conn_param_mgr_high_failed(uint8_t conn_id,
                                        uint16_t ci_min,
                                        uint16_t ci_max,
                                        uint16_t latency);
void app_ble_conn_param_mgr_clear_high_cap(uint8_t conn_id);
void app_ble_conn_param_mgr_cancel_owner(app_ble_conn_param_owner_t owner,
                                         uint8_t conn_id,
                                         const char *reason);
void app_ble_conn_param_mgr_abort_all(uint8_t conn_id, const char *reason);
void app_ble_conn_param_mgr_reset(const char *reason);

#ifdef __cplusplus
}
#endif

#endif /* APP_BLE_CONN_PARAM_MGR_H */
