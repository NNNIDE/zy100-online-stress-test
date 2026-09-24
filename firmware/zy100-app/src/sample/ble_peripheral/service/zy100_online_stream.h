#ifndef ZY100_ONLINE_STREAM_H
#define ZY100_ONLINE_STREAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../app_flags.h"
#include "zy100_flash_common.h"
#include "zy100_final_edge_store_target.h"

#define ZY100_ONLINE_STREAM_EXPORT_MAGIC        0xE7U
#define ZY100_ONLINE_STREAM_EXPORT_VERSION      0x01U
#define ZY100_ONLINE_STREAM_EXPORT_HEADER_BYTES 8U
#define ZY100_ONLINE_STREAM_RECORD_HEADER_BYTES 256U
#define ZY100_ONLINE_CAPABILITY_STRESS_V1        0x00000020UL
#define ZY100_ONLINE_CAPABILITY_V2_BASE          0x00000001UL
#define ZY100_ONLINE_CAPABILITY_RTC_ENDPOINT_V1  0x00000002UL
#define ZY100_ONLINE_CAPABILITY_UNIX_ENDPOINT_V2 0x00000004UL
#define ZY100_ONLINE_CAPABILITY_FIXED40_UNIX_ENDPOINT_V3 0x00000008UL
#define ZY100_ONLINE_CAPABILITY_MAG_RAW_100HZ_V1  0x00000010UL
#define ZY100_ONLINE_CAPABILITY_REQUIRED_V3 \
    (ZY100_ONLINE_CAPABILITY_V2_BASE | \
     ZY100_ONLINE_CAPABILITY_RTC_ENDPOINT_V1 | \
     ZY100_ONLINE_CAPABILITY_UNIX_ENDPOINT_V2 | \
     ZY100_ONLINE_CAPABILITY_FIXED40_UNIX_ENDPOINT_V3 | \
     ZY100_ONLINE_CAPABILITY_MAG_RAW_100HZ_V1)
#define ZY100_ONLINE_END_BASE_BYTES              36U
#define ZY100_ONLINE_END_RTC_META_VERSION_V1     1U
#define ZY100_ONLINE_END_RTC_META_BYTES_V1       44U
#define ZY100_ONLINE_END_RTC_TOTAL_BYTES_V1      80U
#define ZY100_ONLINE_END_RTC_META_VERSION_V2     2U
#define ZY100_ONLINE_END_RTC_META_BYTES_V2       60U
#define ZY100_ONLINE_END_RTC_TOTAL_BYTES_V2      96U
#define ZY100_ONLINE_END_FIXED40_META_VERSION_V3 3U
#define ZY100_ONLINE_END_FIXED40_META_BYTES_V3   32U
#define ZY100_ONLINE_END_FIXED40_TOTAL_BYTES_V3  68U

typedef enum
{
    ZY100_ONLINE_RECORD_RAW = 1U,
    ZY100_ONLINE_RECORD_SUMMARY = 2U,
    ZY100_ONLINE_RECORD_EVENT = 3U,
    #if ZY100_ONLINE_STRESS_TEST_ENABLE
    ZY100_ONLINE_RECORD_STRESS = 4U,
    #endif
    ZY100_ONLINE_RECORD_END = 0xFFU,
} zy100_online_record_type_t;

typedef enum
{
    ZY100_ONLINE_STOP_REASON_NONE = 0U,
    /* User-requested graceful stop from Host command or physical button. */
    ZY100_ONLINE_STOP_REASON_HOST_STOP = 1U,
    ZY100_ONLINE_STOP_REASON_DISCONNECT = 2U,
    ZY100_ONLINE_STOP_REASON_ACK_TIMEOUT = 3U,
    ZY100_ONLINE_STOP_REASON_SPOOL_FULL = 4U,
    ZY100_ONLINE_STOP_REASON_INTERNAL = 5U,
    ZY100_ONLINE_STOP_REASON_TRANSPORT_UNAVAILABLE = 6U,
    ZY100_ONLINE_STOP_REASON_TX_STALL = 7U,
    ZY100_ONLINE_STOP_REASON_TX_COMPLETE_FAILED = 8U,
    ZY100_ONLINE_STOP_REASON_CAPTURE_FATAL = 9U,
    ZY100_ONLINE_STOP_REASON_LOW_BATTERY = 10U,
    ZY100_ONLINE_STOP_REASON_USER_SHUTDOWN = 11U,
    ZY100_ONLINE_STOP_REASON_FLASH_IO = 12U,
    ZY100_ONLINE_STOP_REASON_RESET_RECOVERY = 13U,
} zy100_online_stop_reason_t;

typedef enum
{
    ZY100_ONLINE_ABORT_ORIGIN_NONE = 0U,
    ZY100_ONLINE_ABORT_ORIGIN_UNKNOWN,
    ZY100_ONLINE_ABORT_ORIGIN_IMU_WORKER_STOP,
    ZY100_ONLINE_ABORT_ORIGIN_IMU_FIFO_FATAL,
    ZY100_ONLINE_ABORT_ORIGIN_IMU_RAW_PACKET,
    ZY100_ONLINE_ABORT_ORIGIN_IMU_RAW_PUMP,
    ZY100_ONLINE_ABORT_ORIGIN_IMU_RAW_FINALIZE,
    ZY100_ONLINE_ABORT_ORIGIN_IMU_START,
    ZY100_ONLINE_ABORT_ORIGIN_APP_CAPTURE_DONE,
    ZY100_ONLINE_ABORT_ORIGIN_APP_PAUSE_STALL,
    ZY100_ONLINE_ABORT_ORIGIN_APP_CAPTURE_FATAL,
    ZY100_ONLINE_ABORT_ORIGIN_APP_LOW_BATTERY,
    ZY100_ONLINE_ABORT_ORIGIN_APP_SHUTDOWN,
    ZY100_ONLINE_ABORT_ORIGIN_APP_RESUME,
    ZY100_ONLINE_ABORT_ORIGIN_STREAM_SELF,
} zy100_online_abort_origin_t;

typedef enum
{
    ZY100_ONLINE_CLOCK_META_VALID = 0U,
    ZY100_ONLINE_CLOCK_META_RTC_UNAVAILABLE = 1U,
    ZY100_ONLINE_CLOCK_META_TIMEBASE_UNAVAILABLE = 1U,
    ZY100_ONLINE_CLOCK_META_LESS_THAN_TWO_PACKETS = 2U,
    ZY100_ONLINE_CLOCK_META_RTC_RANGE_UNSUPPORTED = 3U,
    ZY100_ONLINE_CLOCK_META_TIMEBASE_RANGE_UNSUPPORTED = 3U,
    ZY100_ONLINE_CLOCK_META_UNIX_UNAVAILABLE = 4U,
} zy100_online_clock_meta_status_t;

typedef struct
{
    uint32_t status;
    uint32_t rtc_nominal_tick_hz;
    uint32_t accepted_packet_count;
    uint16_t first_imu_timestamp_raw;
    uint16_t last_imu_timestamp_raw;
    uint64_t first_rtc_tick;
    uint64_t last_rtc_tick;
    uint64_t rtc_wrap_ticks;
    uint64_t first_unix_time_ms;
    uint64_t last_unix_time_ms;
    uint64_t first_unix_time_us;
    uint64_t last_unix_time_us;
} zy100_online_clock_meta_t;

typedef enum
{
    ZY100_ONLINE_GATE_SAFE_ALLOWED = 0U,
    ZY100_ONLINE_GATE_B_CRITICAL,
    ZY100_ONLINE_GATE_REPLAY,
    ZY100_ONLINE_GATE_A2,
    ZY100_ONLINE_GATE_FIFO,
    ZY100_ONLINE_GATE_CAPTURE_ACTIVE,
    ZY100_ONLINE_GATE_RT_DUE,
    ZY100_ONLINE_GATE_SPI,
    ZY100_ONLINE_GATE_FLASH_WIP,
    ZY100_ONLINE_GATE_RAW_BUSY,
    ZY100_ONLINE_GATE_RECORD_BUSY,
    ZY100_ONLINE_GATE_ACK_WAIT,
    ZY100_ONLINE_GATE_TX_WINDOW,
    ZY100_ONLINE_GATE_COUNT,
} zy100_online_gate_reason_t;

typedef enum
{
    ZY100_ONLINE_FLASH_STEP_IDLE = 0U,
    ZY100_ONLINE_FLASH_STEP_COMMIT_PROGRESS,
    ZY100_ONLINE_FLASH_STEP_ERASE_PROGRESS,
    ZY100_ONLINE_FLASH_STEP_BUSY,
    ZY100_ONLINE_FLASH_STEP_ERROR,
} zy100_online_flash_step_result_t;

typedef enum
{
    ZY100_ONLINE_DROP_UNKNOWN = 0U,
    ZY100_ONLINE_DROP_HWIN,
    ZY100_ONLINE_DROP_TRUNCATED,
    ZY100_ONLINE_DROP_BEGIN,
    ZY100_ONLINE_DROP_PUMP,
    ZY100_ONLINE_DROP_ABORT,
    ZY100_ONLINE_DROP_TARGET_TIMEOUT,
    ZY100_ONLINE_DROP_CURSOR,
    ZY100_ONLINE_DROP_QUEUE,
} zy100_online_drop_reason_t;

typedef enum
{
    ZY100_ONLINE_START_PREP_IDLE = 0U,
    ZY100_ONLINE_START_PREP_BUSY,
    ZY100_ONLINE_START_PREP_DONE,
    ZY100_ONLINE_START_PREP_ERROR,
} zy100_online_start_prep_status_t;

void zy100_online_stream_init(void);
void zy100_online_stream_handle_ready(uint8_t conn_id,
                                      uint32_t capability_mask,
                                      uint64_t host_time_ms,
                                      uint32_t now_ms);
void zy100_online_stream_invalidate_ready(uint8_t conn_id,
                                          const char *reason);
bool zy100_online_stream_ready(uint8_t conn_id);
bool zy100_online_stream_can_start_online(uint8_t conn_id,
                                          uint32_t *detail_out);
bool zy100_online_stream_start_prepare_accept(uint8_t conn_id,
                                               uint8_t seq,
                                               uint32_t now_ms);
bool zy100_online_stream_start_prepare_begin(uint8_t conn_id,
                                              uint8_t seq,
                                              uint32_t now_ms,
                                              uint32_t *detail_out);
zy100_online_start_prep_status_t
zy100_online_stream_start_prepare_poll(uint32_t now_ms,
                                        uint32_t *detail_out);
bool zy100_online_stream_start_prepare_ready(uint8_t conn_id, uint8_t seq);
bool zy100_online_stream_start_prepare_in_progress(void);
void zy100_online_stream_start_prepare_abort(uint32_t detail,
                                              uint32_t now_ms);
bool zy100_online_stream_prepare_start_blocking(uint8_t conn_id,
                                                uint32_t now_ms,
                                                uint32_t *detail_out);
void zy100_online_stream_handle_start_fallback(uint8_t conn_id,
                                               uint32_t detail,
                                               uint32_t now_ms);
void zy100_online_stream_begin(uint8_t conn_id,
                               uint32_t session_id,
                               uint32_t user_id,
                               uint32_t training_id,
                               uint32_t now_ms);
bool zy100_online_stream_active(void);
bool zy100_online_stream_fixed40_endpoint_enabled(void);
bool zy100_online_stream_end_wait_ack(void);
bool zy100_online_stream_start_sent(void);
bool zy100_online_stream_end_requested(void);
bool zy100_online_stream_should_pause_for_upload(uint32_t now_ms);
bool zy100_online_stream_upload_drained(void);
uint32_t zy100_online_stream_pending_bytes(void);
bool zy100_online_stream_clear_spool_begin(void);
zy100_flash_prepare_status_t zy100_online_stream_clear_spool_poll(void);
bool zy100_online_stream_post_session_cleanup_begin(void);
bool zy100_online_stream_abort_cleanup_begin(void);
zy100_online_flash_step_result_t
zy100_online_stream_post_session_cleanup_poll(uint32_t target_bytes);
bool zy100_online_stream_post_session_ready(uint32_t minimum_bytes);
bool zy100_online_stream_flash_idle(void);
void zy100_online_stream_request_stop(zy100_online_stop_reason_t reason,
                                      uint32_t now_ms);
void zy100_online_stream_request_abort(zy100_online_stop_reason_t reason,
                                       uint32_t now_ms);
void zy100_online_stream_request_abort_ex(
    zy100_online_stop_reason_t reason,
    zy100_online_abort_origin_t origin,
    uint32_t detail,
    uint32_t now_ms);
bool zy100_online_stream_abort_cause_valid(void);
bool zy100_online_stream_abort_terminal_pending(void);
void zy100_online_stream_abort_cleanup_ready(uint32_t now_ms);
void zy100_online_stream_request_stop_discard(zy100_online_stop_reason_t reason,
                                              uint32_t now_ms);
bool zy100_online_stream_consume_stop_completed(zy100_online_stop_reason_t *reason,
                                                bool *timeout);
bool zy100_online_stream_quiet_logs_active(void);
bool zy100_online_stream_capture_stop_requested(void);
zy100_online_stop_reason_t zy100_online_stream_capture_stop_reason(void);
void zy100_online_stream_capture_stop_issued(void);
void zy100_online_stream_handle_disconnect(uint8_t conn_id, uint32_t now_ms);
void zy100_online_stream_handle_transport_unavailable(uint8_t conn_id,
                                                       uint32_t now_ms);
#if ZY100_ONLINE_RECOVERY_FAULT_INJECT_ENABLE
void zy100_online_stream_test_inject_failure(zy100_online_stop_reason_t reason);
void zy100_online_stream_test_force_cleanup_error(bool enable);
#endif
void zy100_online_stream_handle_record_ack(uint8_t conn_id,
                                           uint32_t session_id,
                                           zy100_online_record_type_t type,
                                           uint32_t record_id,
                                           uint8_t ack_count,
                                           uint8_t status,
                                           uint32_t now_ms);

void zy100_online_stream_note_record_produced(zy100_online_record_type_t type,
                                               uint32_t payload_bytes);
void zy100_online_stream_note_raw_accepted(uint32_t bscan_id);
void zy100_online_stream_note_raw_saved(uint32_t bscan_id);
bool zy100_online_stream_queue_source_record(zy100_online_record_type_t type,
                                             uint32_t record_id,
                                             uint32_t source_addr,
                                             uint32_t record_bytes,
                                             uint32_t payload_bytes);
void zy100_online_stream_note_record_dropped(zy100_online_record_type_t type,
                                             const char *reason,
                                             uint32_t detail);
void zy100_online_stream_note_record_dropped_ex(
    zy100_online_record_type_t type,
    zy100_online_drop_reason_t reason,
    uint32_t detail,
    zy100_fe_store_reserve_result_t reserve_result,
    uint8_t fe_state,
    uint32_t now_ms);
void zy100_online_stream_note_gate(zy100_online_gate_reason_t reason);
void zy100_online_stream_note_blind_begin(uint32_t now_ms);
void zy100_online_stream_note_blind_end(uint32_t now_ms, uint32_t duration_ms);
void zy100_online_stream_note_ble_link(uint16_t mtu,
                                       uint16_t conn_interval,
                                       uint8_t phy_tx,
                                       uint8_t phy_rx);
void zy100_online_stream_note_capture_segment(uint32_t now_ms);
void zy100_online_stream_set_clock_meta(
    const zy100_online_clock_meta_t *meta);
void zy100_online_stream_pump_flash(bool safe_window, uint32_t now_ms);
zy100_online_flash_step_result_t zy100_online_stream_commit_step(
    bool safe_window,
    uint32_t now_ms);
zy100_online_flash_step_result_t zy100_online_stream_maintenance_step(
    bool safe_window,
    uint32_t now_ms);
bool zy100_online_stream_pump_a1(uint32_t now_ms);
bool zy100_online_stream_micro_blind_needed(void);
bool zy100_online_stream_micro_blind_done(uint32_t elapsed_ms);
/* Capture-task only: stage one Flash-backed RECORD fragment into TX RAM. */
bool zy100_online_stream_stage_record_fragment(uint32_t now_ms);
#if ZY100_ONLINE_STREAM_ENABLE && ZY100_ONLINE_DIRECT_SPOOL_ENABLE && ZY100_ONLINE_CONTINUOUS_RAW_ENABLE
/* Internal borrowed view. Submit once and report its result before selecting again. */
typedef struct
{
    uint8_t *data;
    uint16_t len;
    uint16_t payload_len;
    uint8_t frame_type;
} zy100_online_notify_view_t;
bool zy100_online_stream_select_notify(bool safe_window, uint32_t now_ms,
                                       uint16_t notify_max,
                                       zy100_online_notify_view_t *view);
#endif

bool zy100_online_stream_build_notify(bool safe_window,
                                      uint32_t now_ms,
                                      uint16_t notify_max,
                                      uint8_t *frame,
                                      uint16_t *frame_len);
uint8_t *zy100_online_stream_notify_data(uint8_t *fallback);
void zy100_online_stream_note_notify_result(uint8_t frame_type,
                                            bool ok,
                                            uint16_t frame_len,
                                            uint16_t payload_len,
                                            uint32_t now_ms);
void zy100_online_stream_note_send_complete(uint8_t conn_id,
                                            uint16_t cause,
                                            uint16_t credits);
void zy100_online_stream_get_tx_pressure(uint8_t *ready, uint8_t *in_flight);
void zy100_online_stream_poll(uint32_t now_ms);
void zy100_online_stream_log_summary(const char *reason);
bool zy100_online_stream_log_summary_step(const char *reason);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_ONLINE_STREAM_H */
