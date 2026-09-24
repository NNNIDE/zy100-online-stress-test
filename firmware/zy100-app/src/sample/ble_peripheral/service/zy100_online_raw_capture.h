#ifndef ZY100_ONLINE_RAW_CAPTURE_H
#define ZY100_ONLINE_RAW_CAPTURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../app_flags.h"
#include "zy100_rtc_clock.h"

typedef enum
{
    ZY100_ONLINE_RAW_ERROR_NONE = 0U,
    ZY100_ONLINE_RAW_ERROR_OWNER,
    ZY100_ONLINE_RAW_ERROR_FLASH,
    ZY100_ONLINE_RAW_ERROR_HEADER,
    ZY100_ONLINE_RAW_ERROR_TIMESTAMP,
    ZY100_ONLINE_RAW_ERROR_OVERFLOW,
    ZY100_ONLINE_RAW_ERROR_RESERVE,
    ZY100_ONLINE_RAW_ERROR_PROGRAM,
    ZY100_ONLINE_RAW_ERROR_VERIFY,
    ZY100_ONLINE_RAW_ERROR_COMMIT,
    ZY100_ONLINE_RAW_ERROR_FINISH,
    ZY100_ONLINE_RAW_ERROR_BACKPRESSURE,
    ZY100_ONLINE_RAW_ERROR_MAG_READ,
    ZY100_ONLINE_RAW_ERROR_MAG_STALE,
#if ZY100_ONLINE_STRESS_TEST_ENABLE
    ZY100_ONLINE_RAW_ERROR_STRESS,
#endif
} zy100_online_raw_error_t;

typedef enum
{
    ZY100_ONLINE_RAW_MAG_CHECKPOINT_LOOP_ENTRY = 0U,
    ZY100_ONLINE_RAW_MAG_CHECKPOINT_AFTER_LIVE_SERVICE,
    ZY100_ONLINE_RAW_MAG_CHECKPOINT_BEFORE_COOP_YIELD_FORCE,
    ZY100_ONLINE_RAW_MAG_CHECKPOINT_AFTER_COOP_YIELD,
    ZY100_ONLINE_RAW_MAG_CHECKPOINT_AFTER_NOTIFY_WAIT,
} zy100_online_raw_mag_checkpoint_t;

typedef struct
{
    uint32_t accepted_packets;
    uint32_t committed_packets;
    uint32_t discarded_packets;
    uint32_t committed_records;
    uint32_t partial_records;
    uint32_t ring_packets;
    uint32_t ring_high_water;
    uint32_t timestamp_errors;
    uint32_t reserve_waits;
    uint32_t page_programs;
    uint32_t page_verifies;
    uint32_t verify_rereads;
    uint32_t verify_recovered;
    uint32_t verify_persistent;
    uint32_t block32_erase_steps;
    uint32_t forward_erase_requests;
    uint32_t forward_erase_blocked;
    uint32_t urgent_reserve_recoveries;
    uint32_t last_record_id;
    uint32_t last_source_id;
    uint32_t accepted_mag_samples;
    uint32_t committed_mag_samples;
    uint32_t discarded_mag_samples;
    uint32_t mag_read_errors;
    uint32_t mag_missed_deadlines;
    uint32_t mag_ring_samples;
    uint32_t mag_ring_high_water;
    uint32_t page_build_max_us;
    uint32_t page_program_max_us;
    uint32_t page_verify_max_us;
    uint32_t record_write_total_ms;
    uint32_t record_write_max_ms;
    uint32_t normal_records;
    uint32_t catchup_records;
    uint32_t mag_lateness_max_us;
    uint32_t mag_post_flash_samples;
    uint32_t payload_xor_max_us;
    uint32_t record_prepare_max_us;
    uint32_t mag_miss_task_gap;
    uint32_t mag_miss_record_prepare;
    uint32_t mag_miss_flash_io;
    uint32_t mag_miss_ble_staging;
    uint32_t mag_after_flash_samples;
    uint32_t mag_after_staging_samples;
    uint32_t mag_read_total_us;
    uint32_t mag_read_max_us;
    uint32_t mag_miss_after_live_service;
    uint32_t mag_miss_after_coop_yield;
    uint32_t mag_miss_after_notify_wait;
    uint32_t mag_gap_after_live_max_us;
    uint32_t mag_gap_after_coop_max_us;
    uint32_t mag_gap_after_notify_max_us;
    uint32_t mag_interval_min_us;
    uint32_t mag_interval_max_us;
    uint32_t mag_adjacent_raw9_same;
    uint32_t mag_status_poll_count;
    uint32_t mag_not_ready_poll_count;
    uint32_t mag_poll_missed_count;
    uint32_t mag_status_read_total_us;
    uint32_t mag_status_read_max_us;
    uint32_t mag_fresh_timeout_count;
    uint32_t mag_forced_poll_count;
    uint32_t mag_forced_fresh_count;
    uint16_t last_timestamp_raw;
    uint8_t error;
} zy100_online_raw_capture_stats_t;

typedef struct
{
    uint32_t accepted_packets;
    uint16_t first_timestamp_raw;
    uint16_t last_timestamp_raw;
    uint8_t first_rtc_valid;
    uint8_t last_rtc_valid;
    uint8_t first_unix_valid;
    uint8_t last_unix_valid;
    uint32_t nominal_tick_hz;
    uint64_t first_rtc_tick;
    uint64_t last_rtc_tick;
    uint64_t rtc_wrap_ticks;
    uint64_t first_unix_time_ms;
    uint64_t last_unix_time_ms;
    uint8_t fixed40_endpoint;
    uint8_t first_fixed40_valid;
    uint8_t last_fixed40_valid;
    uint8_t fixed40_range_unsupported;
    uint32_t first_fixed40_counter;
    uint32_t last_fixed40_counter;
    uint64_t first_unix_time_us;
    uint64_t last_unix_time_us;
} zy100_online_raw_clock_meta_t;

typedef struct
{
    zy100_rtc_raw_snapshot_t rtc;
    uint32_t fixed40_counter;
    uint8_t rtc_valid;
    uint8_t fixed40_valid;
} zy100_online_raw_fifo_time_snapshot_t;

typedef void (*zy100_online_raw_progress_cb_t)(void);

/* Called only by the IMU capture task. The online stream must already own an
 * active session and the normal Final Edge queue/LF ring must have been reset. */
bool zy100_online_raw_capture_begin(uint32_t source_id_seed);
/* App-task START gate: configure MMC5603 for uncalibrated BW01/100 Hz
 * continuous acquisition before the IMU producer is allowed to start. */
bool zy100_online_raw_capture_prepare_mag(void);
/* Deferred end-only resource lines, stage 0..2. */
void zy100_online_raw_capture_log_resource(uint8_t stage);
void zy100_online_raw_capture_release_mag(void);
bool zy100_online_raw_capture_active(void);
bool zy100_online_raw_capture_push_packet(const uint8_t packet[16],
                                          uint16_t timestamp_raw);
/* Capture the endpoint candidate immediately after a FIFO read. For V3 the
 * first call freezes calibrated Unix time and starts TIM2; later calls only
 * read the current 1 MHz counter. */
bool zy100_online_raw_capture_snapshot_fifo_read(
    uint32_t accepted_before,
    zy100_online_raw_fifo_time_snapshot_t *snapshot);
/* Call once after each successful FIFO burst has been fully validated and
 * accepted. expected_packets prevents a partial burst from publishing an
 * endpoint candidate. */
bool zy100_online_raw_capture_note_fifo_burst(
    uint32_t accepted_before,
    uint32_t expected_packets,
    const zy100_online_raw_fifo_time_snapshot_t *snapshot);
/* Low-cost V0 task checkpoints. LOOP_ENTRY may also be used immediately
 * before a blocking notify wait to establish its Fixed40 gap baseline.
 * BEFORE_COOP_YIELD_FORCE always polls STATUS1, but does not advance an
 * absolute 5 ms poll deadline that has not become due. */
bool zy100_online_raw_capture_mag_checkpoint(
    zy100_online_raw_mag_checkpoint_t checkpoint);
bool zy100_online_raw_capture_pump(uint32_t now_ms);
typedef enum
{
    ZY100_ONLINE_SERVICE_ERROR = 0,
    ZY100_ONLINE_SERVICE_WAIT,
    ZY100_ONLINE_SERVICE_READY
} zy100_online_service_result_t;
/* Capture task only; READY permits a nonblocking notification check, not an
 * inner busy loop. Full FIFO/stop/service arbitration still runs each turn. */
zy100_online_service_result_t zy100_online_raw_capture_service(uint32_t now_ms);

/* Called after the IMU producer is stopped. discard_tail=true is reserved for
 * fatal/transport aborts; false persists a final short record before release. */
bool zy100_online_raw_capture_finish(bool discard_tail,
                                     uint32_t now_ms,
                                     zy100_online_raw_progress_cb_t progress_cb);
void zy100_online_raw_capture_get_stats(
    zy100_online_raw_capture_stats_t *out);
void zy100_online_raw_capture_get_clock_meta(
    zy100_online_raw_clock_meta_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_ONLINE_RAW_CAPTURE_H */
