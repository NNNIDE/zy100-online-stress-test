#ifndef APP_BLE_PROFILE_ROUTER_PORT_H
#define APP_BLE_PROFILE_ROUTER_PORT_H

#include "app_ble_sensor_stream_port.h"

#ifdef __cplusplus
extern "C" {
#endif

#define app_ble_profile_router_port_restart_advertising_if_idle(reason) \
    app_ble_sensor_stream_port_restart_advertising(reason)

#ifdef __cplusplus
}
#endif

#endif
