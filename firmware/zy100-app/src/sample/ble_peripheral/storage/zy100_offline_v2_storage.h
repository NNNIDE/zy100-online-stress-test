#ifndef ZY100_OFFLINE_V2_STORAGE_H
#define ZY100_OFFLINE_V2_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../algo/zy100_offline_feature_v2.h"

#define ZY100_OFFLINE_V2_RESULT_BEGIN          0x000000UL
#define ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE  0x2E8000UL
#define ZY100_OFFLINE_V2_JOURNAL_A_BEGIN       0x2E8000UL
#define ZY100_OFFLINE_V2_JOURNAL_B_BEGIN       0x2F0000UL
#define ZY100_OFFLINE_V2_OFFLINE_END_EXCLUSIVE 0x2F8000UL
#define ZY100_OFFLINE_V2_ONLINE_BEGIN          0x2F8000UL
#define ZY100_OFFLINE_V2_ONLINE_END_EXCLUSIVE  0x3F8000UL
#define ZY100_OFFLINE_V2_RESERVED_BEGIN        0x3F8000UL
#define ZY100_OFFLINE_V2_FLASH_END_EXCLUSIVE   0x400000UL
#define ZY100_OFFLINE_V2_MIN_ERASED_POOL_BYTES  (32UL * 1024UL)

#define ZY100_OFFLINE_V2_RECLAIM_UNKNOWN      0U
#define ZY100_OFFLINE_V2_RECLAIM_FINAL        1U
#define ZY100_OFFLINE_V2_RECLAIM_CONFIRMED    2U
#define ZY100_OFFLINE_V2_RECLAIM_RECLAIMING   3U
#define ZY100_OFFLINE_V2_RECLAIM_TOMBSTONE    4U
#define ZY100_OFFLINE_V2_RECLAIM_ERROR        5U
#define ZY100_OFFLINE_V2_RECLAIM_PAUSED       6U

typedef enum
{
    ZY100_OFFLINE_V2_BOOT_NORMAL = 0U,
    ZY100_OFFLINE_V2_BOOT_FIRST_USER,
    /* Recover an existing layout only. Never create/migrate a layout. */
    ZY100_OFFLINE_V2_BOOT_RECOVER_ONLY,
    /* App-validated manufacturing stage; ordinary user writers remain gated. */
    ZY100_OFFLINE_V2_BOOT_MANUFACTURING,
} zy100_offline_v2_boot_policy_t;

typedef enum
{
    ZY100_OFFLINE_V2_DRAIN_DONE = 0U,
    ZY100_OFFLINE_V2_DRAIN_BUSY,
    ZY100_OFFLINE_V2_DRAIN_ERROR,
} zy100_offline_v2_drain_status_t;

typedef enum
{
    ZY100_OFFLINE_V2_HW_IDLE = 0U,
    ZY100_OFFLINE_V2_HW_BUSY,
    ZY100_OFFLINE_V2_HW_UNKNOWN,
} zy100_offline_v2_hw_status_t;

zy100_offline_v2_drain_status_t zy100_offline_v2_storage_drain_status(void);
zy100_offline_v2_hw_status_t zy100_offline_v2_storage_hardware_status(void);
bool zy100_offline_v2_storage_recover_begin(void);
bool zy100_offline_v2_storage_recovery_anchored(void);
bool zy100_offline_v2_storage_recovery_retryable(void);
uint8_t zy100_offline_v2_storage_failed_job(void);

bool zy100_offline_v2_storage_init_with_policy(
    uint8_t *scratch, uint32_t scratch_bytes,
    zy100_offline_v2_boot_policy_t policy);

typedef enum
{
    ZY100_OFFLINE_V2_STORAGE_UNINITIALIZED = 0U,
    ZY100_OFFLINE_V2_STORAGE_MIGRATING,
    ZY100_OFFLINE_V2_STORAGE_CLEARING,
    ZY100_OFFLINE_V2_STORAGE_RECOVERING,
    ZY100_OFFLINE_V2_STORAGE_READY,
    ZY100_OFFLINE_V2_STORAGE_BUSY,
    ZY100_OFFLINE_V2_STORAGE_LOCKED_5_PERCENT,
    ZY100_OFFLINE_V2_STORAGE_LEGACY_LAYOUT_BLOCKED,
    ZY100_OFFLINE_V2_STORAGE_ERROR,
} zy100_offline_v2_storage_state_t;

typedef enum
{
    ZY100_OFFLINE_V2_CLEAR_IDLE = 0U,
    ZY100_OFFLINE_V2_CLEAR_BUSY,
    ZY100_OFFLINE_V2_CLEAR_DONE,
    ZY100_OFFLINE_V2_CLEAR_ERROR,
} zy100_offline_v2_clear_status_t;

typedef enum
{
    ZY100_OFFLINE_V2_FOREIGN_PURGE_REJECTED = 0U,
    ZY100_OFFLINE_V2_FOREIGN_PURGE_NO_ACTION,
    ZY100_OFFLINE_V2_FOREIGN_PURGE_ACTIVE,
    ZY100_OFFLINE_V2_FOREIGN_PURGE_OWNER_MISMATCH,
    ZY100_OFFLINE_V2_FOREIGN_PURGE_ERROR,
} zy100_offline_v2_foreign_purge_result_t;

typedef enum
{
    ZY100_OFFLINE_V2_STOP_USER = 0U,
    ZY100_OFFLINE_V2_STOP_FLASH_FULL = 1U,
    ZY100_OFFLINE_V2_STOP_LOW_BATTERY = 2U,
    ZY100_OFFLINE_V2_STOP_POWER_LOSS_RECOVERED = 3U,
    ZY100_OFFLINE_V2_STOP_FLASH_IO_ERROR = 4U,
    ZY100_OFFLINE_V2_STOP_SAMPLE_GAP_ABORT = 5U,
    ZY100_OFFLINE_V2_STOP_SENSOR_START_ERROR = 6U,
    ZY100_OFFLINE_V2_STOP_FIFO_FULL = 7U,
    ZY100_OFFLINE_V2_STOP_FIFO_LOST = 8U,
    ZY100_OFFLINE_V2_STOP_FIFO_BAD_HEADER = 9U,
    ZY100_OFFLINE_V2_STOP_FIFO_READ_ERROR = 10U,
    ZY100_OFFLINE_V2_STOP_ALGO_ERROR = 11U,
    ZY100_OFFLINE_V2_STOP_MAG_FATAL = 12U,
    ZY100_OFFLINE_V2_STOP_WORKER_EXIT = 13U,
    ZY100_OFFLINE_V2_STOP_SYSTEM_SHUTDOWN = 14U,
} zy100_offline_v2_stop_reason_t;

typedef struct
{
    uint32_t fifo_overflow_count;
    uint32_t fifo_discard_count;
    uint32_t time_gap_count;
    uint32_t feature_drop_count;
    uint32_t q12_clip_event_count;
    uint32_t flash_error_count;
} zy100_offline_v2_quality_t;

typedef enum
{
    ZY100_OFFLINE_V2_HEALTH_NORMAL = 0U,
    ZY100_OFFLINE_V2_HEALTH_CONTROLLED_STOP = 1U,
    ZY100_OFFLINE_V2_HEALTH_ABNORMAL_FINALIZED = 2U,
    ZY100_OFFLINE_V2_HEALTH_RECOVERED_PREFIX = 3U,
} zy100_offline_v2_session_health_t;

typedef struct
{
    uint32_t session_id;
    uint32_t generation;
    uint32_t begin_addr;
    uint32_t end_exclusive;
    uint32_t event_count;
    uint32_t logical_bytes;
    uint32_t stream_crc32;
    uint32_t owner_user_id;
    uint16_t begin_version;
    uint16_t manifest_version;
    zy100_offline_v2_stop_reason_t stop_reason;
    zy100_offline_v2_session_health_t health;
    zy100_offline_v2_quality_t quality;
    bool clean;
    bool finalized;
    bool confirmed;
    bool restart_from_zero;
    /* Cached before erase; addresses may be reused, identity never is. */
    uint32_t start_unix_lo;
    uint32_t start_unix_hi;
    uint32_t duration_ms;
    uint32_t reclaim_next;
    uint32_t confirm_transfer_id;
} zy100_offline_v2_session_info_t;

bool zy100_offline_v2_storage_init(uint8_t *scratch,
                                   uint32_t scratch_bytes);
void zy100_offline_v2_storage_poll(bool fifo_urgent);
zy100_offline_v2_storage_state_t zy100_offline_v2_storage_state(void);
bool zy100_offline_v2_storage_ready_for_capture(void);
bool zy100_offline_v2_storage_job_busy(void);
bool zy100_offline_v2_storage_preempt_capacity(void);
bool zy100_offline_v2_storage_pause_reclaim(void);
void zy100_offline_v2_storage_release_reclaim(void);
void zy100_offline_v2_storage_resume_reclaim(uint32_t user_id);
uint32_t zy100_offline_v2_storage_directory_revision(void);
void zy100_offline_v2_storage_require_restart(uint32_t id, uint32_t generation, bool required);
bool zy100_offline_v2_storage_latest_for_user(
    uint32_t user_id, zy100_offline_v2_session_info_t *info_out);

bool zy100_offline_v2_storage_clear_all_begin(void);
zy100_offline_v2_clear_status_t
zy100_offline_v2_storage_clear_all_poll(void);
bool zy100_offline_v2_storage_clear_all_active(void);
zy100_offline_v2_foreign_purge_result_t
zy100_offline_v2_storage_foreign_purge_request(uint32_t protected_user_id,
                                               uint8_t request_seq);
bool zy100_offline_v2_storage_foreign_purge_status(
    uint32_t protected_user_id,
    uint32_t *remaining_sessions_out,
    uint32_t *total_sectors_out,
    uint32_t *erased_sectors_out,
    uint32_t *batch_token_out);
bool zy100_offline_v2_storage_foreign_purge_completion_peek(
    uint32_t *protected_user_id_out,
    uint32_t *batch_token_out,
    uint32_t *total_sectors_out,
    bool *failed_out);
void zy100_offline_v2_storage_foreign_purge_completion_ack(
    uint32_t batch_token);
bool zy100_offline_v2_storage_foreign_purge_active(void);
uint8_t zy100_offline_v2_storage_job_code(void);
uint32_t zy100_offline_v2_storage_progress_address(void);
uint8_t zy100_offline_v2_storage_remaining_percent(void);
bool zy100_offline_v2_storage_begin_session(uint64_t start_unix_ms,
                                             bool timebase_synced,
                                             uint32_t owner_user_id);
bool zy100_offline_v2_storage_session_active(void);
uint32_t zy100_offline_v2_storage_active_session_id(void);
uint32_t zy100_offline_v2_storage_active_owner(void);
uint32_t zy100_offline_v2_storage_active_generation(void);
uint32_t zy100_offline_v2_storage_active_event_count(void);
bool zy100_offline_v2_storage_append_event(
    const zy100_offline_v2_event_t *event);
bool zy100_offline_v2_storage_checkpoint(uint32_t now_ms);
bool zy100_offline_v2_storage_finalize_session(
    zy100_offline_v2_stop_reason_t reason,
    bool clean,
    uint32_t duration_ms,
    const zy100_offline_v2_quality_t *quality);
uint32_t zy100_offline_v2_storage_session_count(void);
uint32_t zy100_offline_v2_storage_session_count_for_user(uint32_t user_id);
uint32_t zy100_offline_v2_storage_foreign_session_count(uint32_t user_id);
bool zy100_offline_v2_storage_session_get(
    uint32_t oldest_index,
    zy100_offline_v2_session_info_t *info_out);
bool zy100_offline_v2_storage_session_get_for_user(
    uint32_t user_id,
    uint32_t oldest_visible_index,
    zy100_offline_v2_session_info_t *info_out);
bool zy100_offline_v2_storage_read(uint32_t address,
                                  uint8_t *data,
                                  uint16_t length);
bool zy100_offline_v2_storage_confirm_session(uint32_t session_id,
                                              uint32_t generation,
                                              uint32_t owner_user_id,
                                              uint32_t transfer_id,
                                              uint32_t logical_bytes,
                                              uint32_t stream_crc32);
bool zy100_offline_v2_storage_reclaim_status(uint32_t session_id,
                                             uint32_t generation,
                                             uint32_t owner_user_id,
                                             uint8_t *state_out,
                                             uint32_t *erased_bytes_out,
                                             uint32_t *extent_bytes_out);
bool zy100_offline_v2_storage_confirm_replay_matches(
    uint32_t session_id,
    uint32_t generation,
    uint32_t owner_user_id,
    uint32_t transfer_id,
    uint32_t stream_crc32);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_OFFLINE_V2_STORAGE_H */
