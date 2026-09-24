#ifndef APP_BLE_EXPORT_PORT_H
#define APP_BLE_EXPORT_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "service/zy100_session_range_table.h"

typedef enum
{
    APP_BLE_EXPORT_PORT_BLOCK_EXPORT = 0U,
    APP_BLE_EXPORT_PORT_BLOCK_NOTIFY,
} app_ble_export_port_block_t;

typedef struct
{
    bool valid;
    bool dirty;
    uint32_t generation;
    uint32_t pending_count;
    bool can_start_next;
    bool storage_dirty_need_clear;
    bool reclaim_active;
    bool clear_active;
    bool expc_without_rcld;
    bool runtime_busy;
    const char *last_refresh_reason;
    const char *last_error_reason;
    uint64_t last_update_ms;
    uint32_t reclaim_session_uid;
    zy100_session_alloc_status_t last_alloc_status;
    uint8_t remaining_percent;
} app_ble_export_port_storage_state_t;

extern app_ble_export_port_storage_state_t
    g_app_ble_export_port_storage_state;

bool app_ble_export_port_b_critical_active(void);
void app_ble_export_port_note_b_critical_block(app_ble_export_port_block_t block);
bool app_ble_export_port_blocks_capture_extra(void);
bool app_ble_export_port_blocks_clear_extra(void);
bool app_ble_export_port_gate_blocks_export(void);
bool app_ble_export_port_device_logic_ready(void);
bool app_ble_export_port_ota_blocks_new_business(void);
bool app_ble_export_port_link_led_auto_export_defer(const char *reason);
void app_ble_export_port_set_in_progress(bool in_progress);
void app_ble_export_port_post_ctrl_event(void);
bool app_ble_export_port_post_tx_ready_event(void);
uint8_t app_ble_export_port_device_state(void);
void app_ble_export_port_set_clear_failed_state(void);

void app_ble_export_port_refresh_storage(const char *reason);
uint32_t app_ble_export_port_pending_session_count(void);
bool app_ble_export_port_has_pending_sessions(void);
uint32_t app_ble_export_port_pending_total_bytes(void);
bool app_ble_export_port_flash_full_wait_gate_active(void);

bool app_ble_export_port_ui_button_override_active(void);
void app_ble_export_port_ui_cancel_charge(const char *reason);
void app_ble_export_port_ui_stop_export_wait(void);
void app_ble_export_port_ui_start_export_wait(void);
void app_ble_export_port_ui_stop_clear_red(void);
void app_ble_export_port_ui_cancel_storage_retry(void);
void app_ble_export_port_ui_release_training_end(const char *reason);
void app_ble_export_port_ui_restore_capture(const char *reason);
void app_ble_export_port_ui_request_blue(uint8_t owner,
                                         uint8_t priority,
                                         const char *reason);
void app_ble_export_port_ui_log_blue(const char *reason);
void app_ble_export_port_set_export_cleanup(void);

bool app_ble_export_port_clear_flash_finalize(void);

#endif
