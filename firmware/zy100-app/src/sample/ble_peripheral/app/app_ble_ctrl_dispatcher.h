#ifndef APP_BLE_CTRL_DISPATCHER_H
#define APP_BLE_CTRL_DISPATCHER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void app_ble_ctrl_dispatcher_handle_write(uint8_t conn_id,
                                          const uint8_t *data,
                                          uint16_t len);
void app_ble_ctrl_dispatcher_poll(void);
/* Cancel only the disconnected link's uncommitted user synchronization. */
void app_ble_ctrl_dispatcher_on_disconnect(uint8_t conn_id);

#ifdef __cplusplus
}
#endif

#endif /* APP_BLE_CTRL_DISPATCHER_H */
