#ifndef APP_BLE_CAPTURE_OBSERVER_H
#define APP_BLE_CAPTURE_OBSERVER_H

#include <stdbool.h>
#include <stdint.h>

#include "../service/zy100_offline_v2_capture.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    APP_BLE_CAPTURE_PAIRING_NOT_OBSERVER = 0,
    APP_BLE_CAPTURE_PAIRING_ACCEPT_KNOWN_REPAIR,
    APP_BLE_CAPTURE_PAIRING_REJECT
} app_ble_capture_pairing_decision_t;

bool app_ble_capture_observer_on_connected(uint8_t conn_id);
void app_ble_capture_observer_on_disconnected(uint8_t conn_id);
bool app_ble_capture_observer_on_auth_started(uint8_t conn_id);
bool app_ble_capture_observer_on_auth_complete(uint8_t conn_id,
                                               bool success,
                                               bool *security_ready_out);
bool app_ble_capture_observer_on_bond_key_missing(uint8_t conn_id);
bool app_ble_capture_observer_on_bond_add(uint8_t conn_id);
void app_ble_capture_observer_on_ack_cccd(uint8_t conn_id, bool enabled);
bool app_ble_capture_observer_active(uint8_t conn_id);
bool app_ble_capture_observer_authorized(uint8_t conn_id);
app_ble_capture_pairing_decision_t
app_ble_capture_observer_on_pairing_request(uint8_t conn_id,
                                            const char *reason);
bool app_ble_capture_observer_take_command_window(
    uint8_t conn_id,
    uint8_t command,
    uint32_t *generation_out,
    zy100_offline_v2_observer_window_result_t *result_out);
void app_ble_capture_observer_poll(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_BLE_CAPTURE_OBSERVER_H */
