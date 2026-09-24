#ifndef ZY100_TRAINING_SESSION_H
#define ZY100_TRAINING_SESSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../common/imu_common.h"

#define ZY100_TRAINING_RECORD_BYTES 64U

typedef enum
{
    ZY100_TRAINING_SRC_UNKNOWN = 0U,
    ZY100_TRAINING_SRC_BUTTON = 1U,
    ZY100_TRAINING_SRC_BLE = 2U,
    ZY100_TRAINING_SRC_AUTO = 3U,
} zy100_training_source_t;

typedef enum
{
    ZY100_TRAINING_STOP_UNKNOWN = 0U,
    ZY100_TRAINING_STOP_BUTTON = 1U,
    ZY100_TRAINING_STOP_BLE = 2U,
    ZY100_TRAINING_STOP_FLASH_FULL = 3U,
    ZY100_TRAINING_STOP_ERROR = 4U,
    ZY100_TRAINING_STOP_BATTERY_LOW = 5U,
} zy100_training_stop_reason_t;

typedef struct
{
    uint8_t active;
    uint8_t started;
    uint8_t ended;

    uint32_t user_id;
    uint32_t training_id;
    uint32_t session_seq;
    uint32_t capture_round;

    uint8_t time_source_start;
    uint8_t time_source_end;
    uint8_t calibrated_start;
    uint8_t calibrated_end;

    uint64_t start_time_ms;
    uint64_t end_time_ms;

    zy100_training_source_t source;
    zy100_training_stop_reason_t stop_reason;

    uint32_t raw_block_count;
    uint32_t summary_count;
    uint32_t event_count;
} zy100_training_session_ctx_t;

typedef struct
{
    bool begin_available;
    bool end_available;
    bool begin_cached;
    bool end_cached;
    uint32_t begin_bytes;
    uint32_t end_bytes;
    uint32_t section_count;
    uint32_t flags;
} zy100_training_session_export_info_t;

#define ZY100_TRAINING_EXPORT_BEGIN_PRESENT (1UL << 0)
#define ZY100_TRAINING_EXPORT_END_PRESENT   (1UL << 1)
#define ZY100_TRAINING_EXPORT_BEGIN_CACHED  (1UL << 2)
#define ZY100_TRAINING_EXPORT_END_CACHED    (1UL << 3)
#define ZY100_TRAINING_EXPORT_INCOMPLETE    (1UL << 4)

typedef struct
{
    bool trnb_erased;
    bool trne_erased;
    bool read_ok;
    imu_status_t status;
    uint32_t first_bad_addr;
    uint8_t first_bad;
} zy100_training_meta_verify_result_t;

typedef enum
{
    ZY100_TRAINING_META_STATE_ERASED = 0,
    ZY100_TRAINING_META_STATE_NOT_ERASED,
    ZY100_TRAINING_META_STATE_FLASH_NOT_READY,
    ZY100_TRAINING_META_STATE_READ_FAIL,
    ZY100_TRAINING_META_STATE_INVALID_LAYOUT,
} zy100_training_meta_state_t;

typedef struct
{
    uint32_t user_id;
    uint32_t training_id;
    uint32_t session_seq;
    uint32_t capture_round;
    uint64_t start_time_ms;
    uint64_t end_time_ms;
    uint8_t time_source_start;
    uint8_t time_source_end;
    uint8_t calibrated_start;
    uint8_t calibrated_end;
    zy100_training_source_t source;
    zy100_training_stop_reason_t stop_reason;
    uint32_t raw_block_count;
    uint32_t summary_count;
    uint32_t event_count;
} zy100_training_session_record_view_t;

void zy100_training_session_log_layout_once(void);

bool zy100_training_session_begin(zy100_training_source_t source,
                                  uint32_t user_id,
                                  uint32_t training_id,
                                  uint32_t session_seq,
                                  uint32_t capture_round);
void zy100_training_session_update_round(uint32_t session_seq,
                                         uint32_t capture_round);
void zy100_training_session_note_end_request(
    zy100_training_source_t request_source,
    zy100_training_stop_reason_t stop_reason);
void zy100_training_session_note_end_if_unset(
    zy100_training_stop_reason_t stop_reason);
void zy100_training_session_set_counts(uint32_t raw_block_count,
                                       uint32_t summary_count,
                                       uint32_t event_count);
bool zy100_training_session_write_end_record(void);
bool zy100_training_session_write_start_failed(void);

void zy100_training_session_clear_active(void);
void zy100_training_session_clear_completed(void);
bool zy100_training_session_has_export_context(void);

zy100_training_meta_state_t zy100_training_meta_get_state(
    zy100_training_meta_verify_result_t *out);
bool zy100_training_meta_slots_are_erased(void);
imu_status_t zy100_training_meta_erase_and_verify(
    zy100_training_meta_verify_result_t *out);

void zy100_training_session_get_export_info(
    zy100_training_session_export_info_t *out);
bool zy100_training_session_get_begin_export_record(
    uint8_t out[ZY100_TRAINING_RECORD_BYTES],
    bool *from_cache);
bool zy100_training_session_get_end_export_record(
    uint8_t out[ZY100_TRAINING_RECORD_BYTES],
    bool *from_cache);
bool zy100_training_session_get_begin_export_record_for_session(
    const zy100_training_session_record_view_t *view,
    uint8_t out[ZY100_TRAINING_RECORD_BYTES]);
bool zy100_training_session_get_end_export_record_for_session(
    const zy100_training_session_record_view_t *view,
    uint8_t out[ZY100_TRAINING_RECORD_BYTES]);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_TRAINING_SESSION_H */
