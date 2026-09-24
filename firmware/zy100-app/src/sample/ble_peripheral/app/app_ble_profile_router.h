#ifndef APP_BLE_PROFILE_ROUTER_H
#define APP_BLE_PROFILE_ROUTER_H

#include <stdbool.h>
#include <stdint.h>
#include <profile_server.h>
#include "app_flags.h"

#ifdef __cplusplus
extern "C" {
#endif

T_APP_RESULT app_profile_callback(T_SERVER_ID service_id, void *p_data);

void app_ble_profile_router_bind_service_ids(
    T_SERVER_ID simple_id,
    T_SERVER_ID control_id,
    T_SERVER_ID calibration_id,
    T_SERVER_ID bas_id,
    T_SERVER_ID ota_id);

void app_ble_profile_router_on_connected(uint8_t conn_id);
void app_ble_profile_router_reset_battery_notify(void);
void app_ble_profile_router_battery_notify_poll(uint32_t now_ms);

bool app_ble_time_sync_ok(void);
bool app_ble_battery_level_notify_ready(void);
bool app_ble_battery_level_notify(uint8_t percent);
void app_ble_battery_level_notify_schedule(uint8_t percent);

#ifdef __cplusplus
}
#endif

#endif
