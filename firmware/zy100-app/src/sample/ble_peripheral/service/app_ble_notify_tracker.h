#ifndef APP_BLE_NOTIFY_TRACKER_H
#define APP_BLE_NOTIFY_TRACKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/*
 * App-task-owned accounting for notifications accepted by server_send_data().
 * Submission and completion hooks are called from the BLE/App task context.
 */
void app_ble_notify_tracker_reset_all(void);
void app_ble_notify_tracker_reset_conn(uint8_t conn_id);
void app_ble_notify_tracker_note_submit(uint8_t conn_id);
void app_ble_notify_tracker_note_complete(uint8_t conn_id);
bool app_ble_notify_tracker_in_flight(void);
uint16_t app_ble_notify_tracker_count(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BLE_NOTIFY_TRACKER_H */
