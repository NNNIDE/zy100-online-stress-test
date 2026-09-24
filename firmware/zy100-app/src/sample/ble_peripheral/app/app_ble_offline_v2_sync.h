#ifndef APP_BLE_OFFLINE_V2_SYNC_H
#define APP_BLE_OFFLINE_V2_SYNC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../service/zy100_ble_ctrl_protocol.h"

typedef struct
{
    zy100_ble_ack_status_t status;
    zy100_ble_exec_mode_t exec_mode;
    zy100_ble_device_state_t device_state;
    uint32_t user_id_echo;
    uint32_t training_id_echo;
    uint32_t detail;
} app_ble_offline_v2_sync_reply_t;

typedef enum
{
    APP_BLE_OFFLINE_ARB_BOOTSTRAP = 0U,
    APP_BLE_OFFLINE_ARB_REQUIRED,
    APP_BLE_OFFLINE_ARB_TRANSFER,
    APP_BLE_OFFLINE_ARB_ONLINE_AVAILABLE,
} app_ble_offline_v2_arb_state_t;

bool app_ble_offline_v2_sync_handle_command(
    uint8_t conn_id,
    const zy100_ble_cmd_frame_t *command,
    app_ble_offline_v2_sync_reply_t *reply);
void app_ble_offline_v2_sync_poll(uint64_t now_ms);
/* Stop volatile transport only; storage jobs continue through capture poll. */
void app_ble_offline_v2_sync_shutdown(void);
void app_ble_offline_v2_sync_on_disconnect(uint8_t conn_id);
void app_ble_offline_v2_sync_on_send_complete(uint8_t conn_id,
                                              uint16_t cause);
bool app_ble_offline_v2_sync_active(void);
bool app_ble_offline_v2_sync_local_preempt_admitted(void);
bool app_ble_offline_v2_sync_preempt_begin(void);
bool app_ble_offline_v2_sync_preempt_quiesced(void);
void app_ble_offline_v2_sync_preempt_end(void);
bool app_ble_offline_v2_sync_transfer_active(void);
bool app_ble_offline_v2_sync_blocks_capture(void);
bool app_ble_offline_v2_sync_blocks_online(void);
bool app_ble_offline_v2_sync_blocks_new_business(void);
app_ble_offline_v2_arb_state_t app_ble_offline_v2_sync_arb_state(void);
uint32_t app_ble_offline_v2_sync_pending_count(void);
bool app_ble_offline_v2_sync_transfer_admitted(void);
bool app_ble_offline_v2_sync_clear_admitted(void);
bool app_ble_offline_v2_sync_foreign_purge_admitted(void);
bool app_ble_offline_v2_sync_business_bootstrap_complete(void);
void app_ble_offline_v2_sync_begin_capture_epoch(const char *reason);
void app_ble_offline_v2_sync_note_transfer_admitted(const char *reason);
void app_ble_offline_v2_sync_note_business_bootstrap_complete(
    const char *reason);
bool app_ble_offline_v2_sync_cancel_for_clear(void);
bool app_ble_offline_v2_sync_quiesced_for_clear(void);
void app_ble_offline_v2_sync_reset_after_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BLE_OFFLINE_V2_SYNC_H */
