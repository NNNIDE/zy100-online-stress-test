#ifndef APP_BLE_EXPORT_PERF_H
#define APP_BLE_EXPORT_PERF_H

#include <stdbool.h>
#include <stdint.h>

#include <profile_server.h>

#include "service/zy100_feuf_export_producer.h"

#define APP_BLE_EXPORT_FRAME_START 0x01U
#define APP_BLE_EXPORT_FRAME_DATA  0x02U
#define APP_BLE_EXPORT_FRAME_END   0x03U
#define APP_BLE_EXPORT_FRAME_ABORT 0x04U

#define APP_BLE_EXPORT_TX_SUBMIT_OK          0U
#define APP_BLE_EXPORT_TX_SUBMIT_NO_BUDGET   1U
#define APP_BLE_EXPORT_TX_SUBMIT_WINDOW_FULL 2U
#define APP_BLE_EXPORT_TX_SUBMIT_NO_SLOT     3U
#define APP_BLE_EXPORT_TX_SUBMIT_HEAD_USED   4U
#define APP_BLE_EXPORT_TX_SUBMIT_BACKOFF     5U

typedef enum
{
    APP_BLE_P0_FAIL_STAGE_NONE = 0U,
    APP_BLE_P0_FAIL_STAGE_BUILD = 1U,
    APP_BLE_P0_FAIL_STAGE_SEND = 2U,
    APP_BLE_P0_FAIL_STAGE_PEEK = 3U,
    APP_BLE_P0_FAIL_STAGE_COMMIT = 4U,
    APP_BLE_P0_FAIL_STAGE_LINK = 5U,
    APP_BLE_P0_FAIL_STAGE_CONFIRM = 6U,
    APP_BLE_P0_FAIL_STAGE_COMPLETE = 7U,
} app_ble_p0_fail_stage_t;

uint64_t app_ble_export_p0_now_us(void);
uint64_t app_ble_export_p0_now_os_ms(void);
void app_ble_export_p0_reset(const char *reason);
void app_ble_export_p0_note_export_start_os(void);
void app_ble_export_p0_note_tx_budget(uint16_t budget);
void app_ble_export_p0_note_tx_in_flight(uint8_t in_flight);
void app_ble_export_p0_note_tx_block_reason(uint8_t reason);
void app_ble_export_p0_note_tx_ready_event(void);
void app_ble_export_p0_note_fast_pump_call(void);
void app_ble_export_p0_note_fast_pump_steps(uint8_t steps);
void app_ble_export_p0_set_prepare_start(uint64_t prepare_start_us);
void app_ble_export_p0_set_session(uint8_t conn_id,
                                   uint32_t export_id,
                                   uint32_t session_uid,
                                   uint16_t session_index,
                                   uint16_t session_total);
void app_ble_export_p0_note_prepare_done(void);
void app_ble_export_p0_note_begin_start(zy100_feuf_export_producer_t *producer);
void app_ble_export_p0_note_begin_done(zy100_feuf_export_producer_t *producer);
void app_ble_export_p0_note_stream_start(zy100_feuf_export_producer_t *producer);
void app_ble_export_p0_record_fail(uint8_t stage,
                                   uint8_t type,
                                   uint16_t seq,
                                   uint16_t len,
                                   uint16_t cause);
void app_ble_export_p0_note_notify_success(uint8_t frame_type,
                                           uint16_t frame_len,
                                           uint16_t payload_len);
void app_ble_export_p0_note_send_blocked(uint8_t frame_type,
                                         uint16_t chunk_seq,
                                         uint16_t frame_len,
                                         bool timeout);
void app_ble_export_p0_note_aggregate_discard(
    const zy100_feuf_export_aggregate_plan_t *plan);
void app_ble_export_p0_note_producer_state_copy(uint32_t bytes,
                                                uint32_t elapsed_us);
void app_ble_export_p0_note_data_complete_ok(uint64_t now_os_ms);
void app_ble_export_p0_note_data_payload_success(uint16_t payload_len,
                                                 uint8_t piece_count);
void app_ble_export_p0_note_wait_confirm(void);
void app_ble_export_p0_mark_final(void);
void app_ble_export_p0_poll_enter(bool *tracked, uint32_t *notify_ok_before);
void app_ble_export_p0_poll_exit(bool tracked, uint32_t notify_ok_before);
void app_ble_export_p0_log_summary(const char *result);
void app_ble_export_p0_on_send_data_complete(uint8_t conn_id,
                                             T_SERVER_ID service_id,
                                             uint16_t attrib_idx,
                                             uint16_t cause,
                                             uint16_t credits);
void app_ble_export_fast_conn_p0_log_summary(void);

uint8_t app_ble_export_perf_port_conn_id(void);
bool app_ble_export_perf_port_streaming(void);
const zy100_feuf_export_producer_t *app_ble_export_perf_port_producer(void);

void app_ble_export_p0_note_build_failure(uint8_t frame_type,
                                          uint16_t chunk_seq,
                                          uint16_t frame_len);
void app_ble_export_p0_note_send_call(bool sent,
                                      uint32_t elapsed_us,
                                      uint8_t frame_type,
                                      uint16_t frame_len,
                                      uint16_t payload_len);
void app_ble_export_p0_note_send_failure(uint8_t frame_type,
                                         uint16_t chunk_seq,
                                         uint16_t frame_len);
void app_ble_export_p0_note_poll_blocked(void);
void app_ble_export_p0_note_producer_failure(uint32_t error,
                                             uint16_t chunk_seq,
                                             uint32_t detail);
void app_ble_export_p0_note_peek(uint32_t elapsed_us,
                                 bool ok,
                                 uint32_t producer_error,
                                 uint16_t chunk_seq,
                                 uint16_t payload_len);
void app_ble_export_p0_note_commit(uint32_t elapsed_us,
                                   bool ok,
                                   uint32_t producer_error,
                                   uint16_t chunk_seq,
                                   uint16_t payload_len);

#endif
