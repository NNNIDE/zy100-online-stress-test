#ifndef APP_BLE_EXPORT_CONTROLLER_H
#define APP_BLE_EXPORT_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#include <profile_server.h>

#include "service/zy100_feuf_export_producer.h"

#define APP_BLE_EXPORT_ATT_HEADER_BYTES 3U
#define APP_BLE_EXPORT_MAX_NOTIFY_BYTES 244U

void app_ble_export_try_auto_start(const char *reason);
void app_ble_export_poll(void);
void app_ble_export_on_disconnect(uint8_t conn_id);
bool app_ble_export_is_active(void);
bool app_ble_export_blocks_sleep(void);
bool app_ble_export_blocks_capture(void);
bool app_ble_export_blocks_clear(void);
bool app_ble_export_transport_busy(void);
bool app_ble_export_waiting_confirm(void);
bool app_ble_export_reclaim_active(void);
bool app_ble_export_clear_retry_required(void);
uint32_t app_ble_export_current_or_retry_id(void);
void app_ble_export_clear_state_reset_after_finalize(void);
void app_ble_export_abort_keep_flash(const char *reason);

bool app_ble_export_ui_result_active(void);
bool app_ble_export_ui_blocks_normal_flow(void);
bool app_ble_export_ui_failed(void);
bool app_ble_export_ui_active(void);
bool app_ble_export_ui_button_override_active(void);
void app_ble_export_ui_tick(uint64_t runtime_ms);
void app_ble_export_ui_cancel(const char *reason);
void app_ble_export_ui_begin_upload(const char *reason);
void app_ble_export_ui_show_blue(uint8_t owner,
                                 uint8_t priority,
                                 const char *reason);

void app_ble_export_controller_on_send_data_complete(uint8_t conn_id,
                                                     T_SERVER_ID service_id,
                                                     uint16_t attrib_idx,
                                                     uint16_t cause,
                                                     uint16_t credits);
void app_ble_export_controller_tx_ready(void);
bool app_ble_export_controller_prepare_for_sleep_request(void);
/* Pump only already accepted reclaim work while ordinary transport is stopped. */
void app_ble_export_controller_shutdown_poll(void);
uint8_t app_ble_export_controller_request_confirm(uint8_t seq,
                                                  uint32_t export_id,
                                                  uint64_t host_time_ms,
                                                  uint32_t result);

uint8_t app_ble_export_perf_port_conn_id(void);
bool app_ble_export_perf_port_streaming(void);
const zy100_feuf_export_producer_t *app_ble_export_perf_port_producer(void);

#endif
