#include "zy100_offline_v2_storage.h"

#include <stddef.h>
#include <string.h>

#include "zy100_offline_v2_marker.h"
#include "zy100_offline_v2_journal.h"
#include "app_flags.h"
#include "../common/zy100_byteorder.h"
#include "../driver/gd25q32e_spi.h"
#include "../driver/spi_bus_owner.h"
#include "../driver/drv_internal_flash.h"
#include "../service/zy100_crc32.h"
#include "../service/zy100_device_identity.h"

#define OFFLINE_V2_PAGE_BYTES                 ZY100_OFFLINE_V2_JOURNAL_HEADER_BYTES
#define OFFLINE_V2_SECTOR_BYTES               4096UL
#define OFFLINE_V2_BLOCK_BYTES                (32UL * 1024UL)
#define OFFLINE_V2_FINAL_RESERVE_BYTES        (8UL * 1024UL)
#define OFFLINE_V2_SESSION_MAX                64U
#define OFFLINE_V2_BEGIN_MAGIC                0x3253464FUL
#define OFFLINE_V2_END_MAGIC                  0x3244464FUL
#define OFFLINE_V2_JOURNAL_ENTRY_MAGIC        0x32454A4FUL
#define OFFLINE_V2_EVENT_COMMIT               0xA55A3CC3UL
#define OFFLINE_V2_BEGIN_COMMIT               0xB16B00B5UL
#define OFFLINE_V2_END_COMMIT                 0xE16D00B5UL
#define OFFLINE_V2_JOURNAL_ENTRY_COMMIT       0x4A454E31UL
#define OFFLINE_V2_JOURNAL_TYPE_OPEN          1U
#define OFFLINE_V2_JOURNAL_TYPE_CHECKPOINT    2U
#define OFFLINE_V2_JOURNAL_TYPE_FINAL         3U
#define OFFLINE_V2_JOURNAL_TYPE_CONFIRMED     4U
#define OFFLINE_V2_JOURNAL_TYPE_DISCARD_INTENT 5U
#define OFFLINE_V2_JOURNAL_TYPE_RECLAIM_DONE  6U
#define OFFLINE_V2_JOURNAL_TYPE_FOREIGN_PURGE_BEGIN 7U
#define OFFLINE_V2_JOURNAL_TYPE_FOREIGN_PURGE_DONE  8U
#define OFFLINE_V2_JOURNAL_ENTRY_VERSION       4U
#define OFFLINE_V2_JOURNAL_TYPE_RECLAIM_PROGRESS 9U
#define OFFLINE_V2_RECLAIM_DETAIL_DISCARD       0x80000000UL
#define OFFLINE_V2_RECLAIM_DETAIL_TOKEN_MASK    0x7FFFFFFFUL
#define OFFLINE_V2_SESSION_INDEX_INVALID       0xFFFFFFFFUL
#define OFFLINE_V2_PAGE_CRC_OFFSET            0xF8U
#define OFFLINE_V2_PAGE_COMMIT_OFFSET         0xFCU
#define OFFLINE_V2_JOURNAL_ENTRY_CRC_OFFSET   0x38U
#define OFFLINE_V2_JOURNAL_ENTRY_TAG_OFFSET   0x3CU
#define OFFLINE_V2_ERASED_PERCENT_LOCK        5U
#define OFFLINE_V2_JOURNAL_BANK_BYTES         (32UL * 1024UL)
#define OFFLINE_V2_MIGRATION_READ_PAGES_PER_POLL 8U
#define OFFLINE_V2_RECOVERY_READ_PAGES_PER_POLL 32U
#define OFFLINE_V2_CLEAR_READ_PAGES_PER_POLL   8U

#if (OFFLINE_V2_MIGRATION_READ_PAGES_PER_POLL == 0U)
#error "OFFLINE_V2_MIGRATION_READ_PAGES_PER_POLL must be non-zero"
#endif

typedef enum
{
    STORAGE_JOB_NONE = 0U,
    STORAGE_JOB_MIGRATION_RESULT_BLANK_SCAN,
    STORAGE_JOB_MIGRATION_DIGEST_BEFORE,
    STORAGE_JOB_MIGRATION_ERASE_WAIT,
    STORAGE_JOB_MIGRATION_BANK_REERASE_WAIT,
    STORAGE_JOB_MIGRATION_BANK_WAIT,
    STORAGE_JOB_MIGRATION_BANK_VERIFY,
    STORAGE_JOB_MIGRATION_DIGEST_AFTER,
    STORAGE_JOB_CLEAR_EXTERNAL_WAIT,
    STORAGE_JOB_CLEAR_EXTERNAL_VERIFY,
    STORAGE_JOB_CLEAR_JOURNAL_PREP,
    STORAGE_JOB_CLEAR_JOURNAL_WAIT,
    STORAGE_JOB_CLEAR_JOURNAL_VERIFY,
    STORAGE_JOB_RECOVERY_SCAN_BEGIN,
    STORAGE_JOB_RECOVERY_SCAN_EVENT,
    STORAGE_JOB_RECOVERY_SCAN_END,
    STORAGE_JOB_RECOVERY_END_WAIT,
    STORAGE_JOB_RECOVERY_END_VERIFY,
    STORAGE_JOB_BEGIN_WAIT,
    STORAGE_JOB_BEGIN_VERIFY,
    STORAGE_JOB_BEGIN_JOURNAL_WAIT,
    STORAGE_JOB_BEGIN_JOURNAL_VERIFY,
    STORAGE_JOB_EVENT_PAGE0_WAIT,
    STORAGE_JOB_EVENT_PAGE0_VERIFY,
    STORAGE_JOB_EVENT_PAGE1_WAIT,
    STORAGE_JOB_EVENT_PAGE1_VERIFY,
    STORAGE_JOB_EVENT_COMMIT_WAIT,
    STORAGE_JOB_EVENT_COMMIT_VERIFY,
    STORAGE_JOB_CHECKPOINT_JOURNAL_WAIT,
    STORAGE_JOB_CHECKPOINT_JOURNAL_VERIFY,
    STORAGE_JOB_END_WAIT,
    STORAGE_JOB_END_VERIFY,
    STORAGE_JOB_END_JOURNAL_WAIT,
    STORAGE_JOB_END_JOURNAL_VERIFY,
    STORAGE_JOB_CONFIRM_JOURNAL_WAIT,
    STORAGE_JOB_CONFIRM_JOURNAL_VERIFY,
    STORAGE_JOB_FOREIGN_PURGE_BEGIN_WAIT,
    STORAGE_JOB_FOREIGN_PURGE_BEGIN_VERIFY,
    STORAGE_JOB_FOREIGN_PURGE_DISCARD_WAIT,
    STORAGE_JOB_FOREIGN_PURGE_DISCARD_VERIFY,
    STORAGE_JOB_RECLAIM_WAIT,
    STORAGE_JOB_RECLAIM_JOURNAL_WAIT,
    STORAGE_JOB_RECLAIM_JOURNAL_VERIFY,
    STORAGE_JOB_FOREIGN_PURGE_DONE_WAIT,
    STORAGE_JOB_FOREIGN_PURGE_DONE_VERIFY,
    STORAGE_JOB_JOURNAL_ROTATE_ERASE_WAIT,
    STORAGE_JOB_JOURNAL_ROTATE_HEADER_WAIT,
    STORAGE_JOB_JOURNAL_ROTATE_HEADER_VERIFY,
    STORAGE_JOB_CLEAR_EXTERNAL_CHECK,
    STORAGE_JOB_RECOVERY_FREE_SCAN,
    STORAGE_JOB_RECOVERY_ERASE_WAIT,
    STORAGE_JOB_RECOVERY_ERASE_VERIFY,
    STORAGE_JOB_RECOVERY_GAP_SCAN,
    STORAGE_JOB_JOURNAL_SNAPSHOT_WAIT,
    STORAGE_JOB_RECLAIM_VERIFY,
    STORAGE_JOB_RECLAIM_PROGRESS_WAIT,
    STORAGE_JOB_RECLAIM_PROGRESS_VERIFY,
} offline_v2_storage_job_t;

typedef struct
{
    zy100_offline_v2_storage_state_t state;
    offline_v2_storage_job_t job;
    zy100_offline_v2_marker_t marker;
    zy100_offline_v2_session_info_t session[OFFLINE_V2_SESSION_MAX];
    uint8_t page[OFFLINE_V2_PAGE_BYTES];
    uint8_t page2[OFFLINE_V2_PAGE_BYTES];
    uint8_t verify[OFFLINE_V2_PAGE_BYTES];
    uint32_t session_count;
    uint32_t directory_revision;
    uint32_t journal_version;
    uint32_t snapshot_index;
    uint32_t snapshot_count;
    uint32_t snapshot_addr;
    uint32_t reclaim_verify_addr;
    uint32_t active_start_lo;
    uint32_t active_start_hi;
    uint32_t active_duration_ms;
    bool reclaim_paused;
    uint32_t next_session_id;
    uint32_t next_session_generation;
    uint32_t result_head;
    uint32_t active_begin;
    uint32_t active_session_id;
    uint32_t active_generation;
    uint32_t active_owner_user_id;
    uint16_t active_begin_version;
    uint16_t active_manifest_version;
    uint32_t active_event_count;
    uint32_t active_first_center;
    uint32_t active_last_center;
    uint32_t active_stream_crc;
    uint32_t active_begin_crc;
    uint32_t active_finalize_sector;
    uint32_t active_dirty_span;
    zy100_offline_v2_stop_reason_t active_stop_reason;
    zy100_offline_v2_session_health_t active_health;
    zy100_offline_v2_quality_t active_quality;
    bool active_clean;
    uint32_t last_checkpoint_event_count;
    uint32_t last_checkpoint_ms;
    uint32_t pending_checkpoint_ms;
    uint32_t journal_next_addr;
    uint32_t journal_seq;
    uint32_t journal_bank_begin;
    uint32_t journal_generation;
    uint32_t journal_rotate_begin;
    uint32_t pending_slot_addr;
    uint32_t pending_center;
    uint32_t pending_record_crc;
    uint32_t digest_addr;
    uint32_t digest_crc;
    uint32_t erase_addr;
    uint32_t clear_verify_addr;
    uint32_t clear_verify_end;
    uint32_t reclaim_addr;
    uint32_t reclaim_session_index;
    uint32_t confirm_transfer_id;
    uint32_t foreign_purge_protected_user_id;
    uint32_t foreign_purge_batch_token;
    uint32_t foreign_purge_total_sectors;
    uint32_t foreign_purge_erased_sectors;
    uint32_t foreign_purge_remaining_sessions;
    uint32_t foreign_purge_done_user_id;
    uint32_t foreign_purge_done_batch_token;
    uint32_t foreign_purge_done_total_sectors;
    zy100_offline_v2_session_info_t reclaim_info;
    zy100_offline_v2_session_info_t tombstone_info;
    uint32_t tombstone_transfer_id;
    uint32_t recovery_begin;
    uint32_t recovery_event_count;
    uint32_t recovery_stream_crc;
    uint32_t recovery_dirty_span;
    uint32_t recovery_first_center;
    uint32_t recovery_last_center;
    uint32_t recovery_high_water;
    offline_v2_storage_job_t journal_resume_job;
    bool active;
    bool begin_pending;
    bool finalize_pending;
    bool journal_pending_reclaim;
    bool reclaim_from_foreign_purge;
    bool foreign_purge_active;
    bool foreign_purge_completion_pending;
    bool foreign_purge_completion_failed;
    bool recovery_data_seen;
    bool recovery_all_erased;
    uint8_t recovery_poll_pages;
    uint32_t recovery_read_bytes;
    uint32_t recovery_polls;
    bool recovery_apply;
    bool recovery_blocked;
    bool recovery_dirty_sector;
    uint32_t recovery_write_addr;
    bool tombstone_valid;
    bool clear_requested;
    bool clear_completed;
} offline_v2_storage_runtime_t;

static offline_v2_storage_runtime_t *s_storage;
/* Retained proof survives a failed remount. Only module-owned transactions
 * may write this layout; journal/marker commits precede destructive work. */
static bool s_recovery_anchor;
static bool s_recovery_rejected;
static uint8_t s_failed_job;
static uint32_t s_scratch_bytes;
static uint8_t s_log_last_state = 0xFFU;
static uint8_t s_log_last_job = 0xFFU;
static uint32_t s_log_last_migration_bucket = 0xFFFFFFFFUL;

typedef char offline_v2_storage_scratch_budget_check[
    (sizeof(offline_v2_storage_runtime_t) <= (8UL * 1024UL)) ? 1 : -1];
typedef char offline_v2_result_boundary_check[
    (ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE ==
     ZY100_OFFLINE_V2_JOURNAL_A_BEGIN) ? 1 : -1];
typedef char offline_v2_journal_boundary_check[
    (ZY100_OFFLINE_V2_OFFLINE_END_EXCLUSIVE ==
     ZY100_OFFLINE_V2_ONLINE_BEGIN) ? 1 : -1];
typedef char offline_v2_online_boundary_check[
    (ZY100_OFFLINE_V2_ONLINE_END_EXCLUSIVE ==
     ZY100_OFFLINE_V2_RESERVED_BEGIN) ? 1 : -1];
#if ZY100_OFFLINE_FEATURE_V2_ENABLE && ZY100_ONLINE_DIRECT_CAPTURE_ENABLE
typedef char offline_v2_online_spool_begin_check[
    (ZY100_OFFLINE_V2_ONLINE_BEGIN ==
     ZY100_ONLINE_SPOOL_REGION_BASE_ADDR) ? 1 : -1];
typedef char offline_v2_online_spool_end_check[
    (ZY100_OFFLINE_V2_ONLINE_END_EXCLUSIVE ==
     (ZY100_ONLINE_SPOOL_REGION_BASE_ADDR +
      ZY100_ONLINE_SPOOL_REGION_BYTES)) ? 1 : -1];
#endif

typedef char offline_v2_float32_contract_check[(sizeof(float) == 4U) ? 1 : -1];

static void storage_put_float_le(uint8_t *destination, float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    zy100_put_u32_le(destination, bits);
}

static uint32_t storage_align_up_sector(uint32_t value)
{
    return (value + (OFFLINE_V2_SECTOR_BYTES - 1U)) &
           ~(OFFLINE_V2_SECTOR_BYTES - 1U);
}

static bool storage_range_valid(uint32_t address, uint32_t length,
                                uint32_t end_exclusive)
{
    return (length != 0U) && (address < end_exclusive) &&
           (length <= (end_exclusive - address));
}

static bool storage_page_erased(const uint8_t *page, uint32_t bytes)
{
    uint32_t index;

    for (index = 0U; index < bytes; index++)
    {
        if (page[index] != 0xFFU)
        {
            return false;
        }
    }
    return true;
}

static void storage_fail(void);

static bool storage_wip_complete(void)
{
    bool busy = true;

    if (spi_bus_current_owner() != SPI_OWNER_NONE) return false;
    if (gd25q32e_is_busy(&busy) != IMU_STATUS_OK)
    {
        storage_fail();
        return false;
    }
    return !busy;
}

static void storage_fail(void)
{
    if (s_storage != NULL)
    {
        ZY100_LOG_ERROR("[OFFLINE_V2][STORAGE][ERR] state=%u job=%u addr=0x%06lX session=%lu events=%lu",
                            (uint32_t)s_storage->state,
                            (uint32_t)s_storage->job,
                            (unsigned long)zy100_offline_v2_storage_progress_address(),
                            (unsigned long)s_storage->active_session_id,
                            (unsigned long)s_storage->active_event_count);
        if (s_storage->foreign_purge_active)
        {
            s_storage->foreign_purge_done_user_id =
                s_storage->foreign_purge_protected_user_id;
            s_storage->foreign_purge_done_batch_token =
                s_storage->foreign_purge_batch_token;
            s_storage->foreign_purge_done_total_sectors =
                s_storage->foreign_purge_total_sectors;
            s_storage->foreign_purge_completion_pending = true;
            s_storage->foreign_purge_completion_failed = true;
            s_storage->foreign_purge_active = false;
        }
        s_failed_job = (uint8_t)s_storage->job;
        s_storage->state = ZY100_OFFLINE_V2_STORAGE_ERROR;
        s_storage->job = STORAGE_JOB_NONE;
        s_storage->active = false;
    }
}

static uint32_t storage_page_crc(uint8_t page[OFFLINE_V2_PAGE_BYTES])
{
    uint32_t crc;

    memset(&page[OFFLINE_V2_PAGE_CRC_OFFSET], 0, 4U);
    crc = zy100_crc32_ieee(page, OFFLINE_V2_PAGE_BYTES);
    zy100_put_u32_le(&page[OFFLINE_V2_PAGE_CRC_OFFSET], crc);
    return crc;
}

static bool storage_page_crc_valid(const uint8_t page[OFFLINE_V2_PAGE_BYTES],
                                   uint32_t magic,
                                   uint32_t commit)
{
    uint8_t copy[OFFLINE_V2_PAGE_BYTES];
    uint32_t stored;

    if ((zy100_get_u32_le(&page[0]) != magic) ||
        (zy100_get_u32_le(&page[OFFLINE_V2_PAGE_COMMIT_OFFSET]) != commit))
    {
        return false;
    }
    memcpy(copy, page, sizeof(copy));
    stored = zy100_get_u32_le(&copy[OFFLINE_V2_PAGE_CRC_OFFSET]);
    memset(&copy[OFFLINE_V2_PAGE_CRC_OFFSET], 0, 4U);
    return stored == zy100_crc32_ieee(copy, sizeof(copy));
}

static bool storage_program_full_page(uint32_t address, const uint8_t *page)
{
    return storage_range_valid(address, OFFLINE_V2_PAGE_BYTES,
                               ZY100_OFFLINE_V2_OFFLINE_END_EXCLUSIVE) &&
           ((address & (OFFLINE_V2_PAGE_BYTES - 1U)) == 0U) &&
           (gd25q32e_page_program(address, page,
                                  OFFLINE_V2_PAGE_BYTES) == IMU_STATUS_OK);
}

static uint32_t storage_journal_bank_end(uint32_t begin)
{
    return begin + OFFLINE_V2_JOURNAL_BANK_BYTES;
}

static uint32_t storage_journal_other_bank(uint32_t begin)
{
    return (begin == ZY100_OFFLINE_V2_JOURNAL_A_BEGIN) ?
           ZY100_OFFLINE_V2_JOURNAL_B_BEGIN :
           ZY100_OFFLINE_V2_JOURNAL_A_BEGIN;
}

static void storage_build_journal_header(bool snapshot)
{
    zy100_offline_v2_journal_header_fields_t fields;

    fields.generation = s_storage->journal_generation;
    fields.next_session_id = s_storage->next_session_id;
    fields.next_session_generation = s_storage->next_session_generation;
    fields.journal_sequence = s_storage->journal_seq;
    fields.result_high_water = s_storage->result_head;
    fields.protected_user_id = s_storage->foreign_purge_active ?
        s_storage->foreign_purge_protected_user_id : 0U;
    fields.foreign_purge_batch_token = s_storage->foreign_purge_active ?
        s_storage->foreign_purge_batch_token : 0U;
    fields.foreign_purge_total_sectors = s_storage->foreign_purge_active ?
        s_storage->foreign_purge_total_sectors : 0U;
    fields.foreign_purge_erased_sectors = s_storage->foreign_purge_active ?
        s_storage->foreign_purge_erased_sectors : 0U;
    if (snapshot)
        (void)zy100_offline_v2_journal_header_encode_v5(s_storage->page, &fields, s_storage->snapshot_count);
    else
        (void)zy100_offline_v2_journal_header_encode(s_storage->page, &fields);
}

static bool storage_program_journal_entry(
    uint8_t type,
    const zy100_offline_v2_session_info_t *info,
    uint32_t detail,
    offline_v2_storage_job_t resume_wait_job);
static bool storage_program_foreign_purge_marker(
    uint8_t type,
    offline_v2_storage_job_t resume_wait_job);

static uint32_t storage_find_identity(uint32_t id, uint32_t generation)
{
    uint32_t i;
    for (i = 0U; i < s_storage->session_count; i++)
        if (s_storage->session[i].session_id == id && s_storage->session[i].generation == generation) return i;
    return OFFLINE_V2_SESSION_INDEX_INVALID;
}

static void storage_remove_session(uint32_t index)
{
    if (index >= s_storage->session_count) return;
    s_storage->session_count--;
    memmove(&s_storage->session[index], &s_storage->session[index + 1U],
        (s_storage->session_count - index) * sizeof(s_storage->session[0]));
    s_storage->directory_revision++;
}

static bool storage_build_journal_entry(uint8_t type,
                                        const zy100_offline_v2_session_info_t *info,
                                        uint32_t detail)
{
    uint32_t crc;

    if (info == NULL)
    {
        return false;
    }
    memset(s_storage->page, 0xFF, OFFLINE_V2_PAGE_BYTES);
    zy100_put_u32_le(&s_storage->page[0x00],
                     OFFLINE_V2_JOURNAL_ENTRY_MAGIC);
    s_storage->page[0x04] = OFFLINE_V2_JOURNAL_ENTRY_VERSION;
    s_storage->page[0x05] = type;
    zy100_put_u16_le(&s_storage->page[0x06], 0U);
    zy100_put_u32_le(&s_storage->page[0x08], s_storage->journal_seq);
    zy100_put_u32_le(&s_storage->page[0x0C], info->session_id);
    zy100_put_u32_le(&s_storage->page[0x10], info->generation);
    zy100_put_u32_le(&s_storage->page[0x14], info->begin_addr);
    zy100_put_u32_le(&s_storage->page[0x18], info->end_exclusive);
    zy100_put_u32_le(&s_storage->page[0x1C], info->owner_user_id);
    zy100_put_u32_le(&s_storage->page[0x20], info->event_count);
    zy100_put_u16_le(&s_storage->page[0x24],
                     ZY100_OFFLINE_V2_EVENT_RECORD_BYTES);
    s_storage->page[0x26] = ZY100_OFFLINE_V2_EVENT_VERSION;
    s_storage->page[0x27] = (uint8_t)info->manifest_version;
    zy100_put_u32_le(&s_storage->page[0x28],
                     ZY100_OFFLINE_V2_CONFIG_CRC32);
    zy100_put_u32_le(&s_storage->page[0x2C],
                     ZY100_OFFLINE_V2_CONFIG_CRC32);
    zy100_put_u32_le(&s_storage->page[0x30], info->stream_crc32);
    zy100_put_u32_le(&s_storage->page[0x34], detail);
    memset(&s_storage->page[OFFLINE_V2_JOURNAL_ENTRY_CRC_OFFSET], 0, 4U);
    zy100_put_u32_le(&s_storage->page[OFFLINE_V2_JOURNAL_ENTRY_TAG_OFFSET],
                     OFFLINE_V2_JOURNAL_ENTRY_COMMIT);
    zy100_put_u32_le(&s_storage->page[0x40], info->start_unix_lo);
    zy100_put_u32_le(&s_storage->page[0x44], info->start_unix_hi);
    zy100_put_u32_le(&s_storage->page[0x48], info->duration_ms);
    zy100_put_u32_le(&s_storage->page[0x4C], info->reclaim_next);
    zy100_put_u32_le(&s_storage->page[0x50], info->confirm_transfer_id);
    zy100_put_u32_le(&s_storage->page[0x54], (uint32_t)info->stop_reason);
    s_storage->page[0x58] = (uint8_t)info->health;
    s_storage->page[0x59] = info->clean ? 1U : 0U;
    crc = zy100_crc32_ieee(s_storage->page, OFFLINE_V2_PAGE_BYTES);
    zy100_put_u32_le(&s_storage->page[OFFLINE_V2_JOURNAL_ENTRY_CRC_OFFSET],
                     crc);
    return true;
}

static bool storage_journal_header_valid(
    const uint8_t page[OFFLINE_V2_PAGE_BYTES],
    uint32_t *generation_out)
{
    return zy100_offline_v2_journal_header_valid(page, generation_out);
}

static bool storage_journal_entry_valid(
    const uint8_t page[OFFLINE_V2_PAGE_BYTES],
    uint32_t *sequence_out)
{
    uint8_t copy[OFFLINE_V2_PAGE_BYTES];
    uint32_t bytes = page[4] == OFFLINE_V2_JOURNAL_ENTRY_VERSION ? OFFLINE_V2_PAGE_BYTES : 64U;
    uint32_t stored;

    if ((zy100_get_u32_le(&page[0x00]) !=
         OFFLINE_V2_JOURNAL_ENTRY_MAGIC) ||
        ((page[0x04] != 2U) && (page[0x04] != 3U) &&
         (page[0x04] != OFFLINE_V2_JOURNAL_ENTRY_VERSION)) ||
        (zy100_get_u32_le(&page[OFFLINE_V2_JOURNAL_ENTRY_TAG_OFFSET]) !=
         OFFLINE_V2_JOURNAL_ENTRY_COMMIT))
    {
        return false;
    }
    memcpy(copy, page, sizeof(copy));
    stored = zy100_get_u32_le(&copy[OFFLINE_V2_JOURNAL_ENTRY_CRC_OFFSET]);
    memset(&copy[OFFLINE_V2_JOURNAL_ENTRY_CRC_OFFSET], 0, 4U);
    if (stored != zy100_crc32_ieee(copy, bytes))
    {
        return false;
    }
    if (sequence_out != NULL)
    {
        *sequence_out = zy100_get_u32_le(&page[0x08]);
    }
    return true;
}

static bool storage_journal_session_info(
    const uint8_t page[OFFLINE_V2_PAGE_BYTES],
    zy100_offline_v2_session_info_t *info_out)
{
    uint32_t event_count;

    if (info_out == NULL)
    {
        return false;
    }
    event_count = zy100_get_u32_le(&page[0x20]);
    if ((event_count >
         (ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE /
          ZY100_OFFLINE_V2_EVENT_SLOT_BYTES)) ||
        (zy100_get_u32_le(&page[0x0C]) == 0U) ||
        (zy100_get_u32_le(&page[0x10]) == 0U))
    {
        return false;
    }
    memset(info_out, 0, sizeof(*info_out));
    info_out->session_id = zy100_get_u32_le(&page[0x0C]);
    info_out->generation = zy100_get_u32_le(&page[0x10]);
    info_out->begin_addr = zy100_get_u32_le(&page[0x14]);
    info_out->end_exclusive = zy100_get_u32_le(&page[0x18]);
    info_out->event_count = event_count;
    info_out->logical_bytes = event_count *
                              ZY100_OFFLINE_V2_EVENT_RECORD_BYTES;
    info_out->stream_crc32 = zy100_get_u32_le(&page[0x30]);
    info_out->owner_user_id =
        (page[0x04] >= 3U) ?
        zy100_get_u32_le(&page[0x1C]) : 0U;
    if ((zy100_get_u32_le(&page[0x28]) !=
         ZY100_OFFLINE_V2_CONFIG_CRC32) ||
        (zy100_get_u16_le(&page[0x24]) !=
         ZY100_OFFLINE_V2_EVENT_RECORD_BYTES) ||
        (page[0x26] != ZY100_OFFLINE_V2_EVENT_VERSION) ||
        ((page[0x27] != ZY100_OFFLINE_V2_MANIFEST_LEGACY_VERSION) &&
         (page[0x27] != ZY100_OFFLINE_V2_MANIFEST_VERSION)))
    {
        return false;
    }
    info_out->manifest_version = page[0x27];
    info_out->begin_version =
        (info_out->manifest_version == ZY100_OFFLINE_V2_MANIFEST_VERSION) ?
        ZY100_OFFLINE_V2_BEGIN_VERSION :
        ZY100_OFFLINE_V2_BEGIN_LEGACY_VERSION;
    info_out->reclaim_next = info_out->begin_addr;
    if (page[4] == OFFLINE_V2_JOURNAL_ENTRY_VERSION)
    {
        info_out->start_unix_lo = zy100_get_u32_le(&page[0x40]);
        info_out->start_unix_hi = zy100_get_u32_le(&page[0x44]);
        info_out->duration_ms = zy100_get_u32_le(&page[0x48]);
        info_out->reclaim_next = zy100_get_u32_le(&page[0x4C]);
        info_out->confirm_transfer_id = zy100_get_u32_le(&page[0x50]);
        info_out->stop_reason = (zy100_offline_v2_stop_reason_t)zy100_get_u32_le(&page[0x54]);
        info_out->health = (zy100_offline_v2_session_health_t)page[0x58];
        info_out->clean = page[0x59] != 0U;
        if (info_out->reclaim_next < info_out->begin_addr ||
            info_out->reclaim_next > info_out->end_exclusive ||
            (info_out->reclaim_next % OFFLINE_V2_SECTOR_BYTES) != 0U) return false;
    }
    info_out->finalized = true;
    info_out->confirmed = true;
    return (info_out->begin_addr < info_out->end_exclusive) &&
           ((info_out->begin_addr % OFFLINE_V2_SECTOR_BYTES) == 0U) &&
           ((info_out->end_exclusive % OFFLINE_V2_SECTOR_BYTES) == 0U) &&
           storage_range_valid(info_out->begin_addr,
                               info_out->end_exclusive -
                               info_out->begin_addr,
                               ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE);
}

static bool storage_recovery_read(uint32_t address, uint8_t *data, uint16_t bytes);

static bool storage_restore_reclaim(const uint8_t *page)
{
    zy100_offline_v2_session_info_t info;
    uint32_t i, j;
    if (!storage_journal_session_info(page, &info)) return false;
    i = storage_find_identity(info.session_id, info.generation);
    if (i == OFFLINE_V2_SESSION_INDEX_INVALID)
    {
        if (s_storage->session_count == OFFLINE_V2_SESSION_MAX) return false;
        i = s_storage->session_count++;
    }
    for (j = 0U; j < s_storage->session_count; j++)
        if (j != i && s_storage->session[j].begin_addr < info.end_exclusive &&
            s_storage->session[j].end_exclusive > info.begin_addr) return false;
    info.confirm_transfer_id = zy100_get_u32_le(&page[0x34]);
    if (page[5] == OFFLINE_V2_JOURNAL_TYPE_DISCARD_INTENT)
        info.confirm_transfer_id |= OFFLINE_V2_RECLAIM_DETAIL_DISCARD;
    s_storage->session[i] = info;
    return true;
}

static bool storage_journal_open_existing(void)
{
    bool valid_a;
    bool valid_b;
    uint32_t generation_a = 0U;
    uint32_t generation_b = 0U;
    uint32_t address;
    uint32_t sequence;
    uint16_t header_version;
    const uint8_t *selected_header;

    if ((!storage_recovery_read(ZY100_OFFLINE_V2_JOURNAL_A_BEGIN,
                       s_storage->page,
                       OFFLINE_V2_PAGE_BYTES)) ||
        (!storage_recovery_read(ZY100_OFFLINE_V2_JOURNAL_B_BEGIN,
                       s_storage->page2,
                       OFFLINE_V2_PAGE_BYTES)))
    {
        return false;
    }
    valid_a = storage_journal_header_valid(s_storage->page, &generation_a);
    valid_b = storage_journal_header_valid(s_storage->page2, &generation_b);
    if (valid_a && valid_b && generation_a == generation_b &&
        memcmp(s_storage->page, s_storage->page2, OFFLINE_V2_PAGE_BYTES) != 0) return false;
    if (!valid_a && !valid_b)
    {
        s_recovery_anchor = false;
        s_recovery_rejected = true;
        return false;
    }
    if (valid_b && (!valid_a || (generation_b > generation_a)))
    {
        s_storage->journal_bank_begin =
            ZY100_OFFLINE_V2_JOURNAL_B_BEGIN;
        s_storage->journal_generation = generation_b;
        s_storage->journal_seq = zy100_get_u32_le(&s_storage->page2[0x14]);
        selected_header = s_storage->page2;
    }
    else
    {
        s_storage->journal_bank_begin =
            ZY100_OFFLINE_V2_JOURNAL_A_BEGIN;
        s_storage->journal_generation = generation_a;
        s_storage->journal_seq = zy100_get_u32_le(&s_storage->page[0x14]);
        selected_header = s_storage->page;
    }
    if (zy100_get_u32_le(&selected_header[0x0C]) != 0U)
    {
        s_storage->next_session_id =
            zy100_get_u32_le(&selected_header[0x0C]);
    }
    if (zy100_get_u32_le(&selected_header[0x10]) != 0U)
    {
        s_storage->next_session_generation =
            zy100_get_u32_le(&selected_header[0x10]);
    }
    if ((zy100_get_u16_le(&selected_header[0x04]) >= 3U) &&
        (zy100_get_u16_le(&selected_header[0x04]) <=
         ZY100_OFFLINE_V2_JOURNAL_RUNTIME_VERSION) &&
        (zy100_get_u32_le(&selected_header[0x18]) <=
         ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE))
    {
        s_storage->recovery_high_water =
            zy100_get_u32_le(&selected_header[0x18]);
    }
    header_version = zy100_get_u16_le(&selected_header[0x04]);
    s_storage->journal_version = header_version;
    if (header_version >= ZY100_OFFLINE_V2_JOURNAL_HEADER_VERSION)
    {
        s_storage->foreign_purge_protected_user_id =
            zy100_get_u32_le(&selected_header[0x1C]);
        s_storage->foreign_purge_batch_token =
            zy100_get_u32_le(&selected_header[0x20]);
        s_storage->foreign_purge_total_sectors =
            zy100_get_u32_le(&selected_header[0x28]);
        s_storage->foreign_purge_erased_sectors =
            zy100_get_u32_le(&selected_header[0x2C]);
        s_storage->foreign_purge_active =
            (s_storage->foreign_purge_protected_user_id != 0U) &&
            (s_storage->foreign_purge_batch_token != 0U);
    }
    if (s_storage->journal_seq == 0U)
    {
        s_storage->journal_seq = 1U;
    }
    s_storage->snapshot_count = header_version == ZY100_OFFLINE_V2_JOURNAL_RUNTIME_VERSION ?
        zy100_get_u32_le(&selected_header[0x30]) : 0U;
    if (s_storage->snapshot_count > OFFLINE_V2_SESSION_MAX + 1U) return false;
    address = s_storage->journal_bank_begin + OFFLINE_V2_PAGE_BYTES;
    while (address < storage_journal_bank_end(
               s_storage->journal_bank_begin))
    {
        if (!storage_recovery_read(address, s_storage->verify,
                          OFFLINE_V2_PAGE_BYTES))
        {
            return false;
        }
        if (storage_page_erased(s_storage->verify,
                                OFFLINE_V2_PAGE_BYTES))
        {
            if (address < s_storage->journal_bank_begin +
                (s_storage->snapshot_count + 1U) * OFFLINE_V2_PAGE_BYTES) return false;
            s_storage->journal_next_addr = address;
            return true;
        }
        if (!storage_journal_entry_valid(s_storage->verify, &sequence))
        {
            if (address < s_storage->journal_bank_begin +
                (s_storage->snapshot_count + 1U) * OFFLINE_V2_PAGE_BYTES) return false;
            /* Never program over a torn page.  Force a bank rotation. */
            s_storage->journal_next_addr = storage_journal_bank_end(
                s_storage->journal_bank_begin);
            return true;
        }
        if (sequence >= s_storage->journal_seq)
        {
            s_storage->journal_seq = sequence + 1U;
        }
        if (s_storage->verify[0x05] <=
            OFFLINE_V2_JOURNAL_TYPE_RECLAIM_DONE)
        {
            if ((zy100_get_u32_le(&s_storage->verify[0x18]) >
                 s_storage->recovery_high_water) &&
                (zy100_get_u32_le(&s_storage->verify[0x18]) <=
                 ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE))
            {
                s_storage->recovery_high_water =
                    zy100_get_u32_le(&s_storage->verify[0x18]);
            }
            /* Old headers contain zero high-water fields.  Session records
             * remain an allocation recovery source.  Batch records reuse
             * these offsets for progress and must never affect allocation. */
            if (zy100_get_u32_le(&s_storage->verify[0x0C]) >=
                s_storage->next_session_id)
            {
                s_storage->next_session_id =
                    zy100_get_u32_le(&s_storage->verify[0x0C]) + 1U;
            }
            if (zy100_get_u32_le(&s_storage->verify[0x10]) >=
                s_storage->next_session_generation)
            {
                s_storage->next_session_generation =
                    zy100_get_u32_le(&s_storage->verify[0x10]) + 1U;
            }
        }
        if (s_storage->verify[0x05] ==
            OFFLINE_V2_JOURNAL_TYPE_FOREIGN_PURGE_BEGIN)
        {
            s_storage->foreign_purge_protected_user_id =
                zy100_get_u32_le(&s_storage->verify[0x1C]);
            s_storage->foreign_purge_batch_token =
                zy100_get_u32_le(&s_storage->verify[0x34]);
            s_storage->foreign_purge_total_sectors =
                zy100_get_u32_le(&s_storage->verify[0x14]);
            s_storage->foreign_purge_erased_sectors =
                zy100_get_u32_le(&s_storage->verify[0x18]);
            s_storage->foreign_purge_active =
                (s_storage->foreign_purge_protected_user_id != 0U) &&
                (s_storage->foreign_purge_batch_token != 0U);
            s_storage->journal_pending_reclaim = false;
            s_storage->reclaim_from_foreign_purge = false;
        }
        else if (s_storage->verify[0x05] ==
            OFFLINE_V2_JOURNAL_TYPE_CONFIRMED || s_storage->verify[0x05] ==
            OFFLINE_V2_JOURNAL_TYPE_RECLAIM_PROGRESS)
        {
            if (!storage_restore_reclaim(s_storage->verify)) return false;
            if (!storage_journal_session_info(s_storage->verify,
                                              &s_storage->reclaim_info))
            {
                return false;
            }
            s_storage->confirm_transfer_id =
                zy100_get_u32_le(&s_storage->verify[0x34]);
            s_storage->journal_pending_reclaim = true;
            s_storage->reclaim_from_foreign_purge = false;
        }
        else if (s_storage->verify[0x05] ==
                 OFFLINE_V2_JOURNAL_TYPE_DISCARD_INTENT)
        {
            if (!storage_restore_reclaim(s_storage->verify)) return false;
            if (!storage_journal_session_info(s_storage->verify,
                                              &s_storage->reclaim_info))
            {
                return false;
            }
            s_storage->confirm_transfer_id =
                OFFLINE_V2_RECLAIM_DETAIL_DISCARD |
                (zy100_get_u32_le(&s_storage->verify[0x34]) &
                 OFFLINE_V2_RECLAIM_DETAIL_TOKEN_MASK);
            s_storage->journal_pending_reclaim = true;
            s_storage->reclaim_from_foreign_purge = true;
        }
        else if (s_storage->verify[0x05] ==
                 OFFLINE_V2_JOURNAL_TYPE_RECLAIM_DONE)
        {
            if (!storage_journal_session_info(s_storage->verify,
                                              &s_storage->tombstone_info))
            {
                return false;
            }
            s_storage->tombstone_transfer_id =
                zy100_get_u32_le(&s_storage->verify[0x34]);
            s_storage->tombstone_valid = true;
            storage_remove_session(storage_find_identity(s_storage->tombstone_info.session_id,
                s_storage->tombstone_info.generation));
            if (s_storage->foreign_purge_active &&
                ((s_storage->tombstone_transfer_id &
                  OFFLINE_V2_RECLAIM_DETAIL_DISCARD) != 0U) &&
                ((s_storage->tombstone_transfer_id &
                  OFFLINE_V2_RECLAIM_DETAIL_TOKEN_MASK) ==
                 (s_storage->foreign_purge_batch_token &
                  OFFLINE_V2_RECLAIM_DETAIL_TOKEN_MASK)))
            {
                s_storage->foreign_purge_erased_sectors +=
                    (s_storage->tombstone_info.end_exclusive -
                     s_storage->tombstone_info.begin_addr) /
                    OFFLINE_V2_SECTOR_BYTES;
            }
            if (s_storage->journal_pending_reclaim &&
                (s_storage->reclaim_info.session_id ==
                 s_storage->tombstone_info.session_id) &&
                (s_storage->reclaim_info.generation ==
                 s_storage->tombstone_info.generation))
            {
                s_storage->journal_pending_reclaim = false;
                s_storage->reclaim_from_foreign_purge = false;
            }
        }
        else if (s_storage->verify[0x05] ==
                 OFFLINE_V2_JOURNAL_TYPE_FOREIGN_PURGE_DONE)
        {
            if (s_storage->foreign_purge_active &&
                (zy100_get_u32_le(&s_storage->verify[0x34]) ==
                 s_storage->foreign_purge_batch_token))
            {
                s_storage->foreign_purge_active = false;
                s_storage->foreign_purge_protected_user_id = 0U;
                s_storage->foreign_purge_batch_token = 0U;
                s_storage->foreign_purge_total_sectors = 0U;
                s_storage->foreign_purge_erased_sectors = 0U;
            }
        }
        address += OFFLINE_V2_PAGE_BYTES;
    }
    s_storage->journal_next_addr = address;
    return true;
}

static bool storage_program_journal_entry(
    uint8_t type,
    const zy100_offline_v2_session_info_t *info,
    uint32_t detail,
    offline_v2_storage_job_t resume_wait_job)
{
    uint32_t bank_end;

    if (!storage_build_journal_entry(type, info, detail))
    {
        return false;
    }
    bank_end = storage_journal_bank_end(s_storage->journal_bank_begin);
    if (s_storage->journal_version == ZY100_OFFLINE_V2_JOURNAL_RUNTIME_VERSION &&
        storage_range_valid(s_storage->journal_next_addr,
                            OFFLINE_V2_PAGE_BYTES, bank_end))
    {
        if (!storage_program_full_page(s_storage->journal_next_addr,
                                       s_storage->page))
        {
            return false;
        }
        s_storage->job = resume_wait_job;
        return true;
    }

    memcpy(s_storage->page2, s_storage->page, OFFLINE_V2_PAGE_BYTES);
    s_storage->journal_resume_job = resume_wait_job;
    s_storage->journal_rotate_begin = storage_journal_other_bank(
        s_storage->journal_bank_begin);
    if (gd25q32e_block_erase_32k(s_storage->journal_rotate_begin) !=
        IMU_STATUS_OK)
    {
        return false;
    }
    s_storage->job = STORAGE_JOB_JOURNAL_ROTATE_ERASE_WAIT;
    return true;
}

static void storage_build_begin(uint64_t start_unix_ms,
                                bool timebase_synced,
                                uint32_t owner_user_id)
{
    s_storage->active_start_lo = (uint32_t)start_unix_ms;
    s_storage->active_start_hi = (uint32_t)(start_unix_ms >> 32);
    memset(s_storage->page, 0xFF, OFFLINE_V2_PAGE_BYTES);
    zy100_put_u32_le(&s_storage->page[0x00], OFFLINE_V2_BEGIN_MAGIC);
    zy100_put_u16_le(&s_storage->page[0x04],
                     ZY100_OFFLINE_V2_BEGIN_VERSION);
    zy100_put_u16_le(&s_storage->page[0x06], OFFLINE_V2_PAGE_BYTES);
    zy100_put_u32_le(&s_storage->page[0x08], s_storage->active_session_id);
    zy100_put_u32_le(&s_storage->page[0x0C], s_storage->active_generation);
    zy100_put_u64_le(&s_storage->page[0x10], start_unix_ms);
    zy100_put_u32_le(&s_storage->page[0x18], 800U);
    zy100_put_u16_le(&s_storage->page[0x1C],
                     ZY100_OFFLINE_V2_EVENT_RECORD_BYTES);
    zy100_put_u16_le(&s_storage->page[0x1E], 128U);
    s_storage->page[0x20] = 12U;
    s_storage->page[0x21] = 6U;
    zy100_put_u16_le(&s_storage->page[0x22],
                     timebase_synced ? 1U : 0U);
    zy100_put_u32_le(&s_storage->page[0x24], ZY100_OFFLINE_V2_ALGORITHM_ID_CRC32);
    zy100_put_u32_le(&s_storage->page[0x28], ZY100_OFFLINE_V2_DETECTOR_ID_CRC32);
    zy100_put_u32_le(&s_storage->page[0x2C], ZY100_OFFLINE_V2_FEATURE_ID_CRC32);
    zy100_put_u32_le(&s_storage->page[0x30], ZY100_OFFLINE_V2_EVENT_CODEC_ID_CRC32);
    zy100_put_u32_le(&s_storage->page[0x34], ZY100_OFFLINE_V2_MODEL_INPUT_ID_CRC32);
    zy100_put_u32_le(&s_storage->page[0x38], ZY100_OFFLINE_V2_SHOCK_ID_CRC32);
    zy100_put_u32_le(&s_storage->page[0x3C], ZY100_OFFLINE_V2_CONFIG_CRC32);
    zy100_put_u32_le(&s_storage->page[0x40],
                     ZY100_OFFLINE_V2_EVENT_SLOT_BYTES);
    s_storage->page[0x44] = ZY100_OFFLINE_V2_MANIFEST_VERSION;
    s_storage->page[0x45] = ZY100_OFFLINE_V2_EVENT_VERSION;
    zy100_put_u16_le(&s_storage->page[0x46], 0U);
    zy100_put_u32_le(&s_storage->page[0x48], ZY100_OFFLINE_V2_LAYOUT_CRC32);
    zy100_put_u32_le(
        &s_storage->page[ZY100_OFFLINE_V2_BEGIN_OWNER_USER_ID_OFFSET],
        owner_user_id);
    zy100_put_u32_le(&s_storage->page[OFFLINE_V2_PAGE_COMMIT_OFFSET],
                     OFFLINE_V2_BEGIN_COMMIT);
    s_storage->active_begin_crc = storage_page_crc(s_storage->page);
}

static void storage_build_event(const zy100_offline_v2_event_t *event)
{
    uint32_t index;
    uint32_t crc;
    uint16_t flags;

    memset(s_storage->page, 0xFF, OFFLINE_V2_PAGE_BYTES);
    memset(s_storage->page2, 0xFF, OFFLINE_V2_PAGE_BYTES);
    flags = event->quality_flags & 0x001FU;
    zy100_put_u32_le(&s_storage->page[0x00], ZY100_OFFLINE_V2_EVENT_MAGIC);
    s_storage->page[0x04] = ZY100_OFFLINE_V2_EVENT_VERSION;
    s_storage->page[0x05] = ZY100_OFFLINE_V2_EVENT_HEADER_BYTES;
    zy100_put_u16_le(&s_storage->page[0x06], flags);
    zy100_put_u32_le(&s_storage->page[0x08], s_storage->active_session_id);
    zy100_put_u32_le(&s_storage->page[0x0C], s_storage->active_event_count);
    zy100_put_u32_le(&s_storage->page[0x10], event->center_sample_index);
    zy100_put_u32_le(&s_storage->page[0x14], event->peak_mag2_raw);
    zy100_put_u32_le(&s_storage->page[0x18],
                     ZY100_OFFLINE_V2_CONFIG_CRC32);
    storage_put_float_le(&s_storage->page[0x1C], event->shock_numerator);
    storage_put_float_le(&s_storage->page[0x20], event->shock_baseline);
    storage_put_float_le(&s_storage->page[0x24], event->shock_ratio);
    s_storage->page[0x28] = event->detector_version;
    s_storage->page[0x29] = event->shock_version;
    s_storage->page[0x2A] = event->feature_version;
    s_storage->page[0x2B] = event->codec_version;
    memset(&s_storage->page[ZY100_OFFLINE_V2_EVENT_CRC_OFFSET], 0, 4U);
    for (index = 0U; index < 104U; index++)
    {
        zy100_put_u16_le(&s_storage->page[0x30 + (index * 2U)],
                         (uint16_t)event->feature_q12[index]);
    }
    for (index = 104U; index < 128U; index++)
    {
        zy100_put_u16_le(&s_storage->page2[(index - 104U) * 2U],
                         (uint16_t)event->feature_q12[index]);
    }
    crc = zy100_crc32_ieee_begin();
    crc = zy100_crc32_ieee_update(crc, s_storage->page, 256U);
    crc = zy100_crc32_ieee_update(crc, s_storage->page2, 48U);
    crc = zy100_crc32_ieee_finish(crc);
    zy100_put_u32_le(&s_storage->page[ZY100_OFFLINE_V2_EVENT_CRC_OFFSET], crc);
    s_storage->pending_record_crc = crc;
    s_storage->pending_center = event->center_sample_index;
}

static void storage_build_end(zy100_offline_v2_stop_reason_t reason,
                              bool clean,
                              uint32_t duration_ms,
                              const zy100_offline_v2_quality_t *quality)
{
    uint32_t extent;
    uint32_t stream_crc =
        zy100_crc32_ieee_finish(s_storage->active_stream_crc);

    s_storage->active_duration_ms = duration_ms;
    s_storage->active_finalize_sector = storage_align_up_sector(
        s_storage->active_begin + OFFLINE_V2_PAGE_BYTES +
        (s_storage->active_event_count *
         ZY100_OFFLINE_V2_EVENT_SLOT_BYTES) + s_storage->active_dirty_span);
    extent = (s_storage->active_finalize_sector + OFFLINE_V2_SECTOR_BYTES) -
             s_storage->active_begin;
    memset(s_storage->page, 0xFF, OFFLINE_V2_PAGE_BYTES);
    zy100_put_u32_le(&s_storage->page[0x00], OFFLINE_V2_END_MAGIC);
    zy100_put_u16_le(&s_storage->page[0x04], 2U);
    zy100_put_u16_le(&s_storage->page[0x06], OFFLINE_V2_PAGE_BYTES);
    zy100_put_u32_le(&s_storage->page[0x08], s_storage->active_session_id);
    zy100_put_u32_le(&s_storage->page[0x0C], s_storage->active_generation);
    zy100_put_u32_le(&s_storage->page[0x10], s_storage->active_event_count);
    zy100_put_u32_le(&s_storage->page[0x14],
                     s_storage->active_event_count *
                     ZY100_OFFLINE_V2_EVENT_RECORD_BYTES);
    zy100_put_u32_le(&s_storage->page[0x18],
                     s_storage->active_event_count *
                     ZY100_OFFLINE_V2_EVENT_SLOT_BYTES +
                     s_storage->active_dirty_span);
    zy100_put_u32_le(&s_storage->page[0x1C], stream_crc);
    zy100_put_u32_le(&s_storage->page[0x20],
                     (s_storage->active_event_count != 0U) ?
                     s_storage->active_first_center : 0xFFFFFFFFUL);
    zy100_put_u32_le(&s_storage->page[0x24],
                     (s_storage->active_event_count != 0U) ?
                     s_storage->active_last_center : 0xFFFFFFFFUL);
    zy100_put_u32_le(&s_storage->page[0x28], duration_ms);
    zy100_put_u16_le(&s_storage->page[0x2C], (uint16_t)reason);
    zy100_put_u16_le(&s_storage->page[0x2E], clean ? 1U : 0U);
    if (quality != NULL)
    {
        zy100_put_u32_le(&s_storage->page[0x30], quality->fifo_overflow_count);
        zy100_put_u32_le(&s_storage->page[0x34], quality->fifo_discard_count);
        zy100_put_u32_le(&s_storage->page[0x38], quality->time_gap_count);
        zy100_put_u32_le(&s_storage->page[0x3C], quality->feature_drop_count);
        zy100_put_u32_le(&s_storage->page[0x40], quality->q12_clip_event_count);
        zy100_put_u32_le(&s_storage->page[0x44], quality->flash_error_count);
    }
    zy100_put_u32_le(&s_storage->page[0x48], s_storage->active_begin_crc);
    zy100_put_u32_le(&s_storage->page[0x4C],
                     ZY100_OFFLINE_V2_CONFIG_CRC32);
    zy100_put_u32_le(&s_storage->page[0x50], extent);
    zy100_put_u32_le(&s_storage->page[OFFLINE_V2_PAGE_COMMIT_OFFSET],
                     OFFLINE_V2_END_COMMIT);
    (void)storage_page_crc(s_storage->page);
}

static zy100_offline_v2_session_health_t storage_session_health(
    zy100_offline_v2_stop_reason_t reason,
    bool clean)
{
    if (reason == ZY100_OFFLINE_V2_STOP_POWER_LOSS_RECOVERED)
    {
        return ZY100_OFFLINE_V2_HEALTH_RECOVERED_PREFIX;
    }
    if (!clean)
    {
        return ZY100_OFFLINE_V2_HEALTH_ABNORMAL_FINALIZED;
    }
    if (reason == ZY100_OFFLINE_V2_STOP_USER)
    {
        return ZY100_OFFLINE_V2_HEALTH_NORMAL;
    }
    return ZY100_OFFLINE_V2_HEALTH_CONTROLLED_STOP;
}

static void storage_info_apply_end(
    zy100_offline_v2_session_info_t *info,
    const uint8_t end[OFFLINE_V2_PAGE_BYTES])
{
    if ((info == NULL) || (end == NULL))
    {
        return;
    }
    info->duration_ms = zy100_get_u32_le(&end[0x28]);
    info->stop_reason = (zy100_offline_v2_stop_reason_t)
                        zy100_get_u16_le(&end[0x2C]);
    info->clean = zy100_get_u16_le(&end[0x2E]) != 0U;
    info->health = storage_session_health(info->stop_reason, info->clean);
    info->quality.fifo_overflow_count = zy100_get_u32_le(&end[0x30]);
    info->quality.fifo_discard_count = zy100_get_u32_le(&end[0x34]);
    info->quality.time_gap_count = zy100_get_u32_le(&end[0x38]);
    info->quality.feature_drop_count = zy100_get_u32_le(&end[0x3C]);
    info->quality.q12_clip_event_count = zy100_get_u32_le(&end[0x40]);
    info->quality.flash_error_count = zy100_get_u32_le(&end[0x44]);
}

static bool storage_add_recovered_session(uint32_t end_exclusive,
                                          uint32_t event_count,
                                          uint32_t stream_crc)
{
    zy100_offline_v2_session_info_t *info;

    if (s_storage->session_count >= OFFLINE_V2_SESSION_MAX)
    {
        return false;
    }
    if (!storage_range_valid(s_storage->recovery_begin,
            end_exclusive - s_storage->recovery_begin, ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE) ||
        s_storage->active_session_id == 0U || s_storage->active_session_id == 0xFFFFFFFFUL ||
        s_storage->active_generation == 0U || s_storage->active_generation == 0xFFFFFFFFUL)
        return false;
    {
        uint32_t i;
        for (i = 0U; i < s_storage->session_count; i++)
        {
            info = &s_storage->session[i];
            if (info->session_id == s_storage->active_session_id ||
                info->generation == s_storage->active_generation ||
                (info->begin_addr < end_exclusive &&
                 info->end_exclusive > s_storage->recovery_begin)) return false;
        }
    }
    info = &s_storage->session[s_storage->session_count++];
    memset(info, 0, sizeof(*info));
    info->session_id = s_storage->active_session_id;
    info->generation = s_storage->active_generation;
    info->begin_addr = s_storage->recovery_begin;
    info->end_exclusive = end_exclusive;
    info->event_count = event_count;
    info->logical_bytes = event_count * ZY100_OFFLINE_V2_EVENT_RECORD_BYTES;
    info->stream_crc32 = stream_crc;
    info->owner_user_id = s_storage->active_owner_user_id;
    info->begin_version = s_storage->active_begin_version;
    info->manifest_version = s_storage->active_manifest_version;
    info->finalized = true;
    info->start_unix_lo = s_storage->active_start_lo;
    info->start_unix_hi = s_storage->active_start_hi;
    storage_info_apply_end(info, s_storage->page);
    if (info->session_id >= s_storage->next_session_id)
    {
        s_storage->next_session_id = info->session_id + 1U;
    }
    if (info->generation >= s_storage->next_session_generation)
    {
        s_storage->next_session_generation = info->generation + 1U;
    }
    return true;
}

static bool storage_start_reclaim(
    const zy100_offline_v2_session_info_t *info,
    uint32_t session_index)
{
    if ((info == NULL) ||
        !storage_range_valid(info->begin_addr,
                             info->end_exclusive - info->begin_addr,
                             ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE))
    {
        return false;
    }
    s_storage->reclaim_info = *info;
    s_storage->reclaim_session_index = session_index;
    s_storage->reclaim_addr = info->reclaim_next >= info->begin_addr ? info->reclaim_next : info->begin_addr;
    if (s_storage->reclaim_paused && !s_storage->reclaim_from_foreign_purge)
    {
        s_storage->journal_pending_reclaim = false;
        s_storage->state = ZY100_OFFLINE_V2_STORAGE_READY;
        s_storage->job = STORAGE_JOB_NONE;
        return true;
    }
    s_storage->journal_pending_reclaim = true;
    if (s_storage->reclaim_addr == info->end_exclusive)
        return storage_program_journal_entry(OFFLINE_V2_JOURNAL_TYPE_RECLAIM_DONE,
            info, s_storage->confirm_transfer_id, STORAGE_JOB_RECLAIM_JOURNAL_WAIT);
    s_storage->state = ZY100_OFFLINE_V2_STORAGE_BUSY;
    if (gd25q32e_sector_erase_4k(s_storage->reclaim_addr) != IMU_STATUS_OK)
    {
        return false;
    }
    s_storage->job = STORAGE_JOB_RECLAIM_WAIT;
    return true;
}

static void storage_foreign_purge_count(uint32_t protected_user_id,
                                        uint32_t *session_count_out,
                                        uint32_t *sector_count_out)
{
    uint32_t index;
    uint32_t session_count = 0U;
    uint32_t sector_count = 0U;

    for (index = 0U; index < s_storage->session_count; index++)
    {
        const zy100_offline_v2_session_info_t *info =
            &s_storage->session[index];

        if (info->owner_user_id == protected_user_id)
        {
            continue;
        }
        session_count++;
        sector_count += (info->end_exclusive - info->begin_addr) /
                        OFFLINE_V2_SECTOR_BYTES;
    }
    if (session_count_out != NULL)
    {
        *session_count_out = session_count;
    }
    if (sector_count_out != NULL)
    {
        *sector_count_out = sector_count;
    }
}

static bool storage_foreign_purge_start_next(void)
{
    uint32_t index;

    storage_foreign_purge_count(
        s_storage->foreign_purge_protected_user_id,
        &s_storage->foreign_purge_remaining_sessions, NULL);
    for (index = 0U; index < s_storage->session_count; index++)
    {
        zy100_offline_v2_session_info_t *info =
            &s_storage->session[index];

        if (info->owner_user_id ==
            s_storage->foreign_purge_protected_user_id)
        {
            continue;
        }
        s_storage->reclaim_info = *info;
        s_storage->reclaim_session_index = index;
        s_storage->confirm_transfer_id =
            OFFLINE_V2_RECLAIM_DETAIL_DISCARD |
            (s_storage->foreign_purge_batch_token &
             OFFLINE_V2_RECLAIM_DETAIL_TOKEN_MASK);
        s_storage->reclaim_from_foreign_purge = true;
        s_storage->state = ZY100_OFFLINE_V2_STORAGE_BUSY;
        if (!storage_program_journal_entry(
                OFFLINE_V2_JOURNAL_TYPE_DISCARD_INTENT, info,
                s_storage->foreign_purge_batch_token,
                STORAGE_JOB_FOREIGN_PURGE_DISCARD_WAIT))
        {
            return false;
        }
        return true;
    }

    s_storage->foreign_purge_remaining_sessions = 0U;
    s_storage->state = ZY100_OFFLINE_V2_STORAGE_BUSY;
    return storage_program_foreign_purge_marker(
        OFFLINE_V2_JOURNAL_TYPE_FOREIGN_PURGE_DONE,
        STORAGE_JOB_FOREIGN_PURGE_DONE_WAIT);
}

static bool storage_recovery_complete(void)
{
    uint32_t index;
    uint32_t matching_index = OFFLINE_V2_SESSION_INDEX_INVALID;

    for (index = 1U; index < s_storage->session_count; index++)
    {
        uint32_t j = index;
        zy100_offline_v2_session_info_t item = s_storage->session[index];
        while (j > 0U && s_storage->session[j - 1U].generation > item.generation)
        { s_storage->session[j] = s_storage->session[j - 1U]; j--; }
        s_storage->session[j] = item;
    }
    s_storage->result_head = 0U;
    for (index = 0U; index < s_storage->session_count; index++)
        if (s_storage->session[index].end_exclusive > s_storage->result_head)
            s_storage->result_head = s_storage->session[index].end_exclusive;
    if (s_storage->journal_pending_reclaim && s_storage->reclaim_from_foreign_purge)
    {
        matching_index = storage_find_identity(s_storage->reclaim_info.session_id, s_storage->reclaim_info.generation);
        return storage_start_reclaim(&s_storage->reclaim_info, matching_index);
    }
    s_storage->journal_pending_reclaim = false;
    if (s_storage->foreign_purge_active)
    {
        return storage_foreign_purge_start_next();
    }
    s_storage->state =
        (zy100_offline_v2_storage_remaining_percent() <=
         OFFLINE_V2_ERASED_PERCENT_LOCK) ?
        ZY100_OFFLINE_V2_STORAGE_LOCKED_5_PERCENT :
        ZY100_OFFLINE_V2_STORAGE_READY;
    s_storage->job = STORAGE_JOB_NONE;
    return true;
}

static bool storage_recovery_read(uint32_t address, uint8_t *data, uint16_t bytes)
{
    uint8_t *check = data == s_storage->verify ? s_storage->page : s_storage->verify;
    if (bytes > OFFLINE_V2_PAGE_BYTES ||
        gd25q32e_read_fast(address, data, bytes) != IMU_STATUS_OK ||
        gd25q32e_read_fast(address, check, bytes) != IMU_STATUS_OK ||
        memcmp(data, check, bytes) != 0)
    {
        ZY100_OFFLINE_V2_LOG("[OFFLINE_REPAIR] access_error addr=%lu", (unsigned long)address);
        return false;
    }
    if (s_storage->state == ZY100_OFFLINE_V2_STORAGE_RECOVERING &&
        address < ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE)
    {
        s_storage->recovery_poll_pages++;
        s_storage->recovery_read_bytes += (uint32_t)bytes * 2U;
        if (!storage_page_erased(data, bytes)) s_storage->recovery_all_erased = false;
    }
    return true;
}

static int storage_event_page_check(uint32_t address,
                                     uint32_t expected_session,
                                     uint32_t expected_seq,
                                     uint32_t *record_crc_out,
                                     uint32_t *center_out)
{
    uint8_t logical[ZY100_OFFLINE_V2_EVENT_RECORD_BYTES];
    uint8_t commit[4];
    uint32_t stored_crc;
    uint32_t calculated;

    if (!storage_recovery_read(address, logical, 256U) ||
        !storage_recovery_read(address + 256U, &logical[256], 48U) ||
        !storage_recovery_read(address + ZY100_OFFLINE_V2_EVENT_COMMIT_OFFSET, commit, 4U))
        return -1;
    if ((zy100_get_u32_le(&logical[0]) != ZY100_OFFLINE_V2_EVENT_MAGIC) ||
        (logical[4] != ZY100_OFFLINE_V2_EVENT_VERSION) ||
        (logical[5] != ZY100_OFFLINE_V2_EVENT_HEADER_BYTES) ||
        ((zy100_get_u16_le(&logical[6]) & 0xFFE0U) != 0U) ||
        (zy100_get_u32_le(&logical[8]) != expected_session) ||
        (zy100_get_u32_le(&logical[12]) != expected_seq) ||
        (zy100_get_u32_le(&logical[24]) != ZY100_OFFLINE_V2_CONFIG_CRC32) ||
        (logical[0x28] != ZY100_OFFLINE_V2_DETECTOR_VERSION) ||
        (logical[0x29] != ZY100_OFFLINE_V2_SHOCK_VERSION) ||
        (logical[0x2A] != ZY100_OFFLINE_V2_FEATURE_VERSION) ||
        (logical[0x2B] != ZY100_OFFLINE_V2_EVENT_CODEC_VERSION) ||
        (zy100_get_u32_le(commit) != OFFLINE_V2_EVENT_COMMIT))
    {
        return false;
    }
    stored_crc = zy100_get_u32_le(&logical[ZY100_OFFLINE_V2_EVENT_CRC_OFFSET]);
    memset(&logical[ZY100_OFFLINE_V2_EVENT_CRC_OFFSET], 0, 4U);
    calculated = zy100_crc32_ieee(logical, sizeof(logical));
    if (stored_crc != calculated)
    {
        return false;
    }
    if (record_crc_out != NULL)
    {
        *record_crc_out = stored_crc;
    }
    if (center_out != NULL)
    {
        *center_out = zy100_get_u32_le(&logical[0x10]);
    }
    return true;
}

static void storage_clear_check_external_block(void)
{
    s_storage->clear_verify_addr = s_storage->erase_addr;
    s_storage->clear_verify_end = s_storage->erase_addr + OFFLINE_V2_BLOCK_BYTES;
    s_storage->job = STORAGE_JOB_CLEAR_EXTERNAL_CHECK;
}

static bool storage_clear_resume_from_marker(void)
{
    s_recovery_anchor = true;
    s_storage->state = ZY100_OFFLINE_V2_STORAGE_CLEARING;
    s_storage->clear_requested = true;
    s_storage->clear_completed = false;
    s_storage->erase_addr = s_storage->marker.next_erase_addr;
    if ((s_storage->erase_addr & (OFFLINE_V2_BLOCK_BYTES - 1U)) != 0U ||
        (s_storage->erase_addr >
         ZY100_OFFLINE_V2_ONLINE_END_EXCLUSIVE))
    {
        s_recovery_anchor = false;
        s_recovery_rejected = true;
        return false;
    }
    if (s_storage->erase_addr < ZY100_OFFLINE_V2_ONLINE_END_EXCLUSIVE)
    {
        /* Recheck the uncommitted block after any interrupted erase/verify. */
        storage_clear_check_external_block();
        return true;
    }
    s_storage->job = STORAGE_JOB_CLEAR_JOURNAL_PREP;
    return true;
}

/* Manufacturing gates guarantee no user writers. The only test writer owns
 * sector zero; verify its entire erase unit as well as the canonical journal. */
static bool storage_empty_handoff_proof(bool *read_error)
{
    zy100_offline_v2_journal_header_fields_t fields;
    uint32_t address;
    *read_error = false;
    if (s_storage->marker.factory_handoff != ZY100_OFFLINE_V2_FACTORY_HANDOFF_NONE &&
        s_storage->marker.factory_handoff != ZY100_OFFLINE_V2_FACTORY_HANDOFF_PENDING)
        return false;
    memset(&fields, 0, sizeof(fields));
    fields.generation = 1U;
    fields.next_session_id = 1U;
    fields.next_session_generation = 1U;
    fields.journal_sequence = 1U;
    if (!zy100_offline_v2_journal_header_encode(s_storage->page2, &fields) ||
        !storage_recovery_read(ZY100_OFFLINE_V2_JOURNAL_A_BEGIN,
                               s_storage->page, OFFLINE_V2_PAGE_BYTES)) goto read_fail;
    if (memcmp(s_storage->page, s_storage->page2, OFFLINE_V2_PAGE_BYTES) != 0) return false;
    if (!storage_recovery_read(ZY100_OFFLINE_V2_JOURNAL_B_BEGIN,
                               s_storage->page, OFFLINE_V2_PAGE_BYTES)) goto read_fail;
    if (!storage_page_erased(s_storage->page, OFFLINE_V2_PAGE_BYTES)) return false;
    if (!storage_recovery_read(ZY100_OFFLINE_V2_JOURNAL_A_BEGIN + OFFLINE_V2_PAGE_BYTES,
                               s_storage->page, OFFLINE_V2_PAGE_BYTES)) goto read_fail;
    if (!storage_page_erased(s_storage->page, OFFLINE_V2_PAGE_BYTES)) return false;
    for (address = 0U; address < OFFLINE_V2_SECTOR_BYTES; address += OFFLINE_V2_PAGE_BYTES)
    {
        if (!storage_recovery_read(address, s_storage->page, OFFLINE_V2_PAGE_BYTES)) goto read_fail;
        if (!storage_page_erased(s_storage->page, OFFLINE_V2_PAGE_BYTES)) return false;
    }
    return true;
read_fail:
    *read_error = true;
    return false;
}

bool zy100_offline_v2_storage_init(uint8_t *scratch,
                                   uint32_t scratch_bytes)
{
    return zy100_offline_v2_storage_init_with_policy(
        scratch, scratch_bytes, ZY100_OFFLINE_V2_BOOT_NORMAL);
}

bool zy100_offline_v2_storage_init_with_policy(
    uint8_t *scratch, uint32_t scratch_bytes,
    zy100_offline_v2_boot_policy_t policy)
{
    gd25q32e_jedec_id_t boot_id;
    gd25q32e_status_regs_t boot_regs;
    bool factory_handoff_empty = false;
    bool read_error = false;
    zy100_offline_v2_handoff_consume_status_t handoff_status;

    if ((scratch == NULL) ||
        ((policy != ZY100_OFFLINE_V2_BOOT_NORMAL) &&
         (policy != ZY100_OFFLINE_V2_BOOT_FIRST_USER) &&
         (policy != ZY100_OFFLINE_V2_BOOT_MANUFACTURING) &&
         (policy != ZY100_OFFLINE_V2_BOOT_RECOVER_ONLY)) ||
        (scratch_bytes < sizeof(offline_v2_storage_runtime_t)))
    {
        return false;
    }
    if (policy != ZY100_OFFLINE_V2_BOOT_RECOVER_ONLY)
    {
        s_recovery_anchor = false;
        s_recovery_rejected = false;
    }
    s_scratch_bytes = scratch_bytes;
    s_storage = (offline_v2_storage_runtime_t *)scratch;
    memset(s_storage, 0, sizeof(*s_storage));
    s_storage->reclaim_session_index = OFFLINE_V2_SESSION_INDEX_INVALID;
    s_storage->state = ZY100_OFFLINE_V2_STORAGE_UNINITIALIZED;
    s_storage->next_session_id = 1U;
    s_storage->next_session_generation = 1U;
    s_storage->journal_bank_begin = ZY100_OFFLINE_V2_JOURNAL_A_BEGIN;
    s_storage->journal_generation = 1U;
    s_storage->journal_next_addr = s_storage->journal_bank_begin +
                                   OFFLINE_V2_PAGE_BYTES;
    s_storage->journal_seq = 1U;
    if ((gd25q32e_init() != IMU_STATUS_OK) ||
        (gd25q32e_resume_and_verify(&boot_id, &boot_regs) != IMU_STATUS_OK) ||
        !zy100_offline_v2_marker_load(&s_storage->marker))
    {
        storage_fail();
        return false;
    }
    if (s_storage->marker.state == ZY100_OFFLINE_V2_MARKER_READY &&
        (s_storage->marker.layout_crc32 != ZY100_OFFLINE_V2_LAYOUT_CRC32 ||
         s_storage->marker.device_id != zy100_device_internal_id()))
    {
        s_recovery_rejected = true;
        storage_fail();
        return false;
    }
    if ((s_storage->marker.state == ZY100_OFFLINE_V2_MARKER_READY) &&
        (s_storage->marker.layout_crc32 == ZY100_OFFLINE_V2_LAYOUT_CRC32) &&
        (s_storage->marker.device_id == zy100_device_internal_id()))
    {
        if (policy == ZY100_OFFLINE_V2_BOOT_FIRST_USER ||
            policy == ZY100_OFFLINE_V2_BOOT_MANUFACTURING ||
            (policy == ZY100_OFFLINE_V2_BOOT_NORMAL &&
             zy100_offline_v2_marker_factory_handoff_pending(&s_storage->marker)))
        {
            factory_handoff_empty = storage_empty_handoff_proof(&read_error);
            if (read_error) { storage_fail(); return false; }
            if (zy100_offline_v2_marker_factory_handoff_pending(&s_storage->marker))
            {
                handoff_status = zy100_offline_v2_marker_consume_factory_handoff(&s_storage->marker);
                if (handoff_status != ZY100_OFFLINE_V2_HANDOFF_CONSUMED)
                {
                    ZY100_LOG_ERROR("[OFFLINE_V2][STORAGE][ERR] handoff consume status=%u", (uint32_t)handoff_status);
                    storage_fail();
                    return false;
                }
            }
            if (factory_handoff_empty)
            {
                s_recovery_anchor = true;
                s_storage->state = ZY100_OFFLINE_V2_STORAGE_READY;
                s_storage->job = STORAGE_JOB_NONE;
                ZY100_OFFLINE_V2_LOG("[BOOT_STORAGE] fast_ready policy=%u sector0_verified=1", (uint32_t)policy);
                return true;
            }
            ZY100_OFFLINE_V2_LOG("[BOOT_STORAGE] policy=%u fallback=full_recovery", (uint32_t)policy);
        }
        if (!storage_journal_open_existing())
        {
            storage_fail();
            return false;
        }
        s_recovery_anchor = true;
        s_recovery_rejected = false;
        s_storage->state = ZY100_OFFLINE_V2_STORAGE_RECOVERING;
        s_storage->job = STORAGE_JOB_RECOVERY_SCAN_BEGIN;
        s_storage->recovery_all_erased = true;
        ZY100_OFFLINE_V2_LOG("[BOOT_STORAGE] scan start policy=%u", (uint32_t)policy);
        return true;
    }
    /* A failed first-user proof must never initiate migration/erase. */
    if (policy == ZY100_OFFLINE_V2_BOOT_FIRST_USER)
    {
        storage_fail();
        return false;
    }
    if ((s_storage->marker.state == ZY100_OFFLINE_V2_MARKER_CLEARING) &&
        (s_storage->marker.layout_crc32 == ZY100_OFFLINE_V2_LAYOUT_CRC32) &&
        (s_storage->marker.device_id == zy100_device_internal_id()))
    {
        s_recovery_anchor = true;
        if (!storage_clear_resume_from_marker())
        {
            storage_fail();
            return false;
        }
        return true;
    }
    if (policy == ZY100_OFFLINE_V2_BOOT_RECOVER_ONLY)
    {
        s_recovery_anchor = false;
        s_recovery_rejected = true;
        storage_fail();
        return false;
    }
    s_storage->marker.factory_handoff =
        ZY100_OFFLINE_V2_FACTORY_HANDOFF_NONE;
    s_storage->state = ZY100_OFFLINE_V2_STORAGE_MIGRATING;
    if ((s_storage->marker.state == ZY100_OFFLINE_V2_MARKER_MIGRATING) &&
        (s_storage->marker.layout_crc32 == ZY100_OFFLINE_V2_LAYOUT_CRC32))
    {
        s_storage->erase_addr = s_storage->marker.next_erase_addr;
        if (s_storage->erase_addr > ZY100_OFFLINE_V2_OFFLINE_END_EXCLUSIVE)
        {
            storage_fail();
            return false;
        }
        if (s_storage->erase_addr == ZY100_OFFLINE_V2_OFFLINE_END_EXCLUSIVE)
        {
            /* The marker can outlive a torn bank-header program.  Re-erase
             * only journal A before retrying; result data, online spool and
             * the top firmware reserve remain outside this operation. */
            if (gd25q32e_block_erase_32k(
                    ZY100_OFFLINE_V2_JOURNAL_A_BEGIN) != IMU_STATUS_OK)
            {
                storage_fail();
                return false;
            }
            s_storage->job = STORAGE_JOB_MIGRATION_BANK_REERASE_WAIT;
        }
        else
        {
            if (gd25q32e_block_erase_32k(s_storage->erase_addr) !=
                IMU_STATUS_OK)
            {
                storage_fail();
                return false;
            }
            s_storage->job = STORAGE_JOB_MIGRATION_ERASE_WAIT;
        }
        return true;
    }
    /* A foreign/legacy marker is never authority to erase the result region.
     * Prove that the complete result partition is blank first.  A V1 or
     * unknown record therefore fails closed and remains exportable only by
     * its matching old firmware. */
    s_storage->digest_addr = ZY100_OFFLINE_V2_RESULT_BEGIN;
    s_storage->job = STORAGE_JOB_MIGRATION_RESULT_BLANK_SCAN;
    return true;
}

static void storage_poll_migration(void)
{
    uint32_t generation;
    uint32_t page_count;

    switch (s_storage->job)
    {
    case STORAGE_JOB_MIGRATION_RESULT_BLANK_SCAN:
        for (page_count = 0U;
             (page_count < OFFLINE_V2_MIGRATION_READ_PAGES_PER_POLL) &&
             (s_storage->digest_addr <
              ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE);
             page_count++)
        {
            if (gd25q32e_read(s_storage->digest_addr,
                              s_storage->verify,
                              OFFLINE_V2_PAGE_BYTES) != IMU_STATUS_OK)
            {
                storage_fail();
                return;
            }
            if (!storage_page_erased(s_storage->verify,
                                     OFFLINE_V2_PAGE_BYTES))
            {
                s_storage->state =
                    ZY100_OFFLINE_V2_STORAGE_LEGACY_LAYOUT_BLOCKED;
                s_storage->job = STORAGE_JOB_NONE;
                ZY100_LOG_ERROR("[OFFLINE_V2][STORAGE][BLOCKED] legacy_or_unknown_layout addr=0x%08lx no_erase=1",
                                    (unsigned long)s_storage->digest_addr);
                return;
            }
            s_storage->digest_addr += OFFLINE_V2_PAGE_BYTES;
        }
        if (s_storage->digest_addr < ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE)
        {
            return;
        }
        s_storage->digest_addr = ZY100_OFFLINE_V2_ONLINE_BEGIN;
        s_storage->digest_crc = zy100_crc32_ieee_begin();
        s_storage->job = STORAGE_JOB_MIGRATION_DIGEST_BEFORE;
        return;

    case STORAGE_JOB_MIGRATION_DIGEST_BEFORE:
        for (page_count = 0U;
             (page_count < OFFLINE_V2_MIGRATION_READ_PAGES_PER_POLL) &&
             (s_storage->digest_addr <
              ZY100_OFFLINE_V2_ONLINE_END_EXCLUSIVE);
             page_count++)
        {
            if (gd25q32e_read(s_storage->digest_addr,
                              s_storage->verify,
                              OFFLINE_V2_PAGE_BYTES) != IMU_STATUS_OK)
            {
                storage_fail();
                return;
            }
            s_storage->digest_crc = zy100_crc32_ieee_update(
                s_storage->digest_crc, s_storage->verify,
                OFFLINE_V2_PAGE_BYTES);
            s_storage->digest_addr += OFFLINE_V2_PAGE_BYTES;
        }
        if (s_storage->digest_addr < ZY100_OFFLINE_V2_ONLINE_END_EXCLUSIVE)
        {
            return;
        }
        s_storage->marker.generation = 1U;
        s_storage->marker.state = ZY100_OFFLINE_V2_MARKER_MIGRATING;
        s_storage->marker.device_id = zy100_device_internal_id();
        s_storage->marker.layout_crc32 = ZY100_OFFLINE_V2_LAYOUT_CRC32;
        s_storage->marker.online_spool_crc32 =
            zy100_crc32_ieee_finish(s_storage->digest_crc);
        s_storage->marker.next_erase_addr = 0U;
        if (!zy100_offline_v2_marker_commit(&s_storage->marker) ||
            (gd25q32e_block_erase_32k(0U) != IMU_STATUS_OK))
        {
            storage_fail();
            return;
        }
        s_storage->erase_addr = 0U;
        s_storage->job = STORAGE_JOB_MIGRATION_ERASE_WAIT;
        return;

    case STORAGE_JOB_MIGRATION_ERASE_WAIT:
        if (!storage_wip_complete())
        {
            return;
        }
        s_storage->erase_addr += OFFLINE_V2_BLOCK_BYTES;
        generation = s_storage->marker.generation + 1U;
        s_storage->marker.generation = generation;
        s_storage->marker.next_erase_addr = s_storage->erase_addr;
        if (!zy100_offline_v2_marker_commit(&s_storage->marker))
        {
            storage_fail();
            return;
        }
        if (s_storage->erase_addr <
            ZY100_OFFLINE_V2_OFFLINE_END_EXCLUSIVE)
        {
            if (gd25q32e_block_erase_32k(s_storage->erase_addr) !=
                IMU_STATUS_OK)
            {
                storage_fail();
                return;
            }
            return;
        }
        storage_build_journal_header(false);
        if (!storage_program_full_page(ZY100_OFFLINE_V2_JOURNAL_A_BEGIN,
                                       s_storage->page))
        {
            storage_fail();
            return;
        }
        s_storage->job = STORAGE_JOB_MIGRATION_BANK_WAIT;
        return;

    case STORAGE_JOB_MIGRATION_BANK_REERASE_WAIT:
        if (!storage_wip_complete())
        {
            return;
        }
        storage_build_journal_header(false);
        if (!storage_program_full_page(ZY100_OFFLINE_V2_JOURNAL_A_BEGIN,
                                       s_storage->page))
        {
            storage_fail();
            return;
        }
        s_storage->job = STORAGE_JOB_MIGRATION_BANK_WAIT;
        return;

    case STORAGE_JOB_MIGRATION_BANK_WAIT:
        if (!storage_wip_complete())
        {
            return;
        }
        s_storage->job = STORAGE_JOB_MIGRATION_BANK_VERIFY;
        return;

    case STORAGE_JOB_MIGRATION_BANK_VERIFY:
        if ((gd25q32e_read(ZY100_OFFLINE_V2_JOURNAL_A_BEGIN,
                           s_storage->verify,
                           OFFLINE_V2_PAGE_BYTES) != IMU_STATUS_OK) ||
            (memcmp(s_storage->page, s_storage->verify,
                    OFFLINE_V2_PAGE_BYTES) != 0))
        {
            storage_fail();
            return;
        }
        s_storage->digest_addr = ZY100_OFFLINE_V2_ONLINE_BEGIN;
        s_storage->digest_crc = zy100_crc32_ieee_begin();
        s_storage->job = STORAGE_JOB_MIGRATION_DIGEST_AFTER;
        return;

    case STORAGE_JOB_MIGRATION_DIGEST_AFTER:
        for (page_count = 0U;
             (page_count < OFFLINE_V2_MIGRATION_READ_PAGES_PER_POLL) &&
             (s_storage->digest_addr <
              ZY100_OFFLINE_V2_ONLINE_END_EXCLUSIVE);
             page_count++)
        {
            if (gd25q32e_read(s_storage->digest_addr,
                              s_storage->verify,
                              OFFLINE_V2_PAGE_BYTES) != IMU_STATUS_OK)
            {
                storage_fail();
                return;
            }
            s_storage->digest_crc = zy100_crc32_ieee_update(
                s_storage->digest_crc, s_storage->verify,
                OFFLINE_V2_PAGE_BYTES);
            s_storage->digest_addr += OFFLINE_V2_PAGE_BYTES;
        }
        if (s_storage->digest_addr < ZY100_OFFLINE_V2_ONLINE_END_EXCLUSIVE)
        {
            return;
        }
        if (zy100_crc32_ieee_finish(s_storage->digest_crc) !=
            s_storage->marker.online_spool_crc32)
        {
            storage_fail();
            return;
        }
        s_storage->marker.generation++;
        s_storage->marker.state = ZY100_OFFLINE_V2_MARKER_READY;
        s_storage->marker.next_erase_addr =
            ZY100_OFFLINE_V2_OFFLINE_END_EXCLUSIVE;
        if (!zy100_offline_v2_marker_commit(&s_storage->marker))
        {
            storage_fail();
            return;
        }
        s_storage->state = ZY100_OFFLINE_V2_STORAGE_READY;
        s_storage->job = STORAGE_JOB_NONE;
        return;

    default:
        storage_fail();
        return;
    }
}

static void storage_clear_reset_runtime(void)
{
    memset(s_storage->session, 0, sizeof(s_storage->session));
    s_storage->session_count = 0U;
    s_storage->next_session_id = 1U;
    s_storage->next_session_generation = 1U;
    s_storage->result_head = 0U;
    s_storage->active_begin = 0U;
    s_storage->active_session_id = 0U;
    s_storage->active_generation = 0U;
    s_storage->active_owner_user_id = 0U;
    s_storage->active_begin_version = 0U;
    s_storage->active_manifest_version = 0U;
    s_storage->active_event_count = 0U;
    s_storage->recovery_high_water = 0U;
    s_storage->active = false;
    s_storage->begin_pending = false;
    s_storage->finalize_pending = false;
    s_storage->journal_pending_reclaim = false;
    s_storage->reclaim_from_foreign_purge = false;
    s_storage->reclaim_session_index = OFFLINE_V2_SESSION_INDEX_INVALID;
    s_storage->recovery_data_seen = false;
    s_storage->tombstone_valid = false;
    s_storage->journal_bank_begin = ZY100_OFFLINE_V2_JOURNAL_A_BEGIN;
    s_storage->journal_generation = 1U;
    s_storage->journal_next_addr = ZY100_OFFLINE_V2_JOURNAL_A_BEGIN +
                                   OFFLINE_V2_PAGE_BYTES;
    s_storage->journal_seq = 1U;
    s_storage->foreign_purge_protected_user_id = 0U;
    s_storage->foreign_purge_batch_token = 0U;
    s_storage->foreign_purge_total_sectors = 0U;
    s_storage->foreign_purge_erased_sectors = 0U;
    s_storage->foreign_purge_remaining_sessions = 0U;
    s_storage->foreign_purge_done_user_id = 0U;
    s_storage->foreign_purge_done_batch_token = 0U;
    s_storage->foreign_purge_done_total_sectors = 0U;
    s_storage->foreign_purge_active = false;
    s_storage->foreign_purge_completion_pending = false;
    s_storage->foreign_purge_completion_failed = false;
}

static void storage_poll_clear(void)
{
    uint32_t next_addr;
    uint8_t pages;
    bool check_before_erase;

    switch (s_storage->job)
    {
    case STORAGE_JOB_CLEAR_EXTERNAL_WAIT:
        if (!storage_wip_complete())
        {
            return;
        }
        s_storage->clear_verify_addr = s_storage->erase_addr;
        s_storage->clear_verify_end = s_storage->erase_addr +
                                      OFFLINE_V2_BLOCK_BYTES;
        s_storage->job = STORAGE_JOB_CLEAR_EXTERNAL_VERIFY;
        return;

    case STORAGE_JOB_CLEAR_EXTERNAL_CHECK:
    case STORAGE_JOB_CLEAR_EXTERNAL_VERIFY:
        check_before_erase = s_storage->job == STORAGE_JOB_CLEAR_EXTERNAL_CHECK;
        /* Flash can remain busy across an MCU reset; never inspect it mid-erase. */
        if (check_before_erase && !storage_wip_complete()) return;
        for (pages = 0U; pages < OFFLINE_V2_CLEAR_READ_PAGES_PER_POLL &&
             s_storage->clear_verify_addr < s_storage->clear_verify_end; ++pages)
        {
            if (gd25q32e_read(s_storage->clear_verify_addr,
                               s_storage->verify,
                               OFFLINE_V2_PAGE_BYTES) != IMU_STATUS_OK)
            {
                storage_fail();
                return;
            }
            if (!storage_page_erased(s_storage->verify, OFFLINE_V2_PAGE_BYTES))
            {
                if (!check_before_erase ||
                    gd25q32e_block_erase_32k(s_storage->erase_addr) != IMU_STATUS_OK)
                {
                    storage_fail();
                    return;
                }
                s_storage->job = STORAGE_JOB_CLEAR_EXTERNAL_WAIT;
                return;
            }
            s_storage->clear_verify_addr += OFFLINE_V2_PAGE_BYTES;
        }
        if (s_storage->clear_verify_addr < s_storage->clear_verify_end) return;
        /* Both skip and erase paths prove the entire block blank before commit. */
        next_addr = s_storage->erase_addr + OFFLINE_V2_BLOCK_BYTES;
        s_storage->marker.generation++;
        s_storage->marker.next_erase_addr = next_addr;
        if (!zy100_offline_v2_marker_commit(&s_storage->marker))
        {
            storage_fail();
            return;
        }
        s_storage->erase_addr = next_addr;
        if (next_addr < ZY100_OFFLINE_V2_ONLINE_END_EXCLUSIVE)
        {
            storage_clear_check_external_block();
            return;
        }
        s_storage->job = STORAGE_JOB_CLEAR_JOURNAL_PREP;
        return;

    case STORAGE_JOB_CLEAR_JOURNAL_PREP:
        /* USER_DATA1 contains only Marker, System Info and MFG. There is no
         * internal scratch area to erase. CLEARING at the external end is
         * the durable checkpoint for retrying this journal preparation. */
        storage_clear_reset_runtime();
        storage_build_journal_header(false);
        if (!storage_program_full_page(ZY100_OFFLINE_V2_JOURNAL_A_BEGIN,
                                       s_storage->page))
        {
            storage_fail();
            return;
        }
        s_storage->job = STORAGE_JOB_CLEAR_JOURNAL_WAIT;
        return;

    case STORAGE_JOB_CLEAR_JOURNAL_WAIT:
        if (!storage_wip_complete())
        {
            return;
        }
        s_storage->job = STORAGE_JOB_CLEAR_JOURNAL_VERIFY;
        return;

    case STORAGE_JOB_CLEAR_JOURNAL_VERIFY:
        if ((gd25q32e_read(ZY100_OFFLINE_V2_JOURNAL_A_BEGIN,
                           s_storage->verify,
                           OFFLINE_V2_PAGE_BYTES) != IMU_STATUS_OK) ||
            (memcmp(s_storage->page, s_storage->verify,
                    OFFLINE_V2_PAGE_BYTES) != 0))
        {
            storage_fail();
            return;
        }
        s_storage->marker.generation++;
        s_storage->marker.state = ZY100_OFFLINE_V2_MARKER_READY;
        s_storage->marker.online_spool_crc32 = 0U;
        s_storage->marker.next_erase_addr =
            ZY100_OFFLINE_V2_ONLINE_END_EXCLUSIVE;
        if (!zy100_offline_v2_marker_commit_fresh_ready(
                &s_storage->marker))
        {
            storage_fail();
            return;
        }
        s_storage->job = STORAGE_JOB_NONE;
        s_storage->state = ZY100_OFFLINE_V2_STORAGE_READY;
        s_storage->clear_requested = false;
        s_storage->clear_completed = true;
        return;

    default:
        storage_fail();
        return;
    }
}

static bool storage_recovery_begin_valid(void)
{
        if (!storage_page_crc_valid(s_storage->page,
                                    OFFLINE_V2_BEGIN_MAGIC,
                                    OFFLINE_V2_BEGIN_COMMIT) ||
            !(((zy100_get_u16_le(&s_storage->page[0x04]) ==
                ZY100_OFFLINE_V2_BEGIN_LEGACY_VERSION) &&
               (s_storage->page[0x44] ==
                ZY100_OFFLINE_V2_MANIFEST_LEGACY_VERSION)) ||
              ((zy100_get_u16_le(&s_storage->page[0x04]) ==
                ZY100_OFFLINE_V2_BEGIN_VERSION) &&
               (s_storage->page[0x44] ==
                ZY100_OFFLINE_V2_MANIFEST_VERSION) &&
               (zy100_get_u32_le(
                    &s_storage->page[
                        ZY100_OFFLINE_V2_BEGIN_OWNER_USER_ID_OFFSET]) != 0U))) ||
            (zy100_get_u16_le(&s_storage->page[0x1C]) !=
             ZY100_OFFLINE_V2_EVENT_RECORD_BYTES) ||
            (zy100_get_u32_le(&s_storage->page[0x3C]) !=
             ZY100_OFFLINE_V2_CONFIG_CRC32) ||
            (s_storage->page[0x45] != ZY100_OFFLINE_V2_EVENT_VERSION))
        return false;
    if (zy100_get_u16_le(&s_storage->page[6]) != OFFLINE_V2_PAGE_BYTES ||
        zy100_get_u32_le(&s_storage->page[0x40]) != ZY100_OFFLINE_V2_EVENT_SLOT_BYTES ||
        zy100_get_u32_le(&s_storage->page[0x48]) != ZY100_OFFLINE_V2_LAYOUT_CRC32 ||
        zy100_get_u32_le(&s_storage->page[8]) == 0U ||
        zy100_get_u32_le(&s_storage->page[8]) == 0xFFFFFFFFUL ||
        zy100_get_u32_le(&s_storage->page[12]) == 0U ||
        zy100_get_u32_le(&s_storage->page[12]) == 0xFFFFFFFFUL) return false;
    return true;
}

/* First pass proves the complete layout before any repair writes. The second
 * pass rechecks it using the same bounded page jobs and existing scratch RAM. */
static bool storage_recovery_accept(void)
{
    uint32_t end = s_storage->active_finalize_sector + OFFLINE_V2_SECTOR_BYTES;
    if (!storage_add_recovered_session(end, s_storage->recovery_event_count,
            zy100_crc32_ieee_finish(s_storage->recovery_stream_crc))) return false;
    s_storage->recovery_high_water = end;
    s_storage->result_head = end;
    s_storage->job = STORAGE_JOB_RECOVERY_SCAN_BEGIN;
    return true;
}

static const zy100_offline_v2_session_info_t *storage_reclaim_at(uint32_t address)
{
    uint32_t i;
    for (i = 0U; i < s_storage->session_count; i++)
        if (s_storage->session[i].confirmed && address >= s_storage->session[i].begin_addr &&
            address < s_storage->session[i].end_exclusive) return &s_storage->session[i];
    if (s_storage->journal_pending_reclaim && s_storage->reclaim_from_foreign_purge &&
        address >= s_storage->reclaim_info.begin_addr && address < s_storage->reclaim_info.end_exclusive)
        return &s_storage->reclaim_info;
    return NULL;
}

static bool storage_recovery_journal_safe(void)
{
    uint32_t i;
    if (!s_storage->journal_pending_reclaim) return true;
    for (i = 0U; i < s_storage->session_count; i++)
    {
        const zy100_offline_v2_session_info_t *p = &s_storage->session[i];
        const zy100_offline_v2_session_info_t *r = &s_storage->reclaim_info;
        if (p->begin_addr >= r->end_exclusive || p->end_exclusive <= r->begin_addr) continue;
        if (p->session_id != r->session_id || p->generation != r->generation ||
            p->begin_addr != r->begin_addr || p->end_exclusive != r->end_exclusive) return false;
    }
    return true;
}

static bool storage_recovery_reclaim_page(uint32_t address)
{
    uint32_t magic = zy100_get_u32_le(s_storage->page);
    const zy100_offline_v2_session_info_t *r = storage_reclaim_at(address);
    if (r == NULL) return false;
    if ((magic == OFFLINE_V2_BEGIN_MAGIC || magic == OFFLINE_V2_END_MAGIC ||
         magic == ZY100_OFFLINE_V2_EVENT_MAGIC) &&
        zy100_get_u32_le(&s_storage->page[8]) != r->session_id) return false;
    if ((magic == OFFLINE_V2_BEGIN_MAGIC || magic == OFFLINE_V2_END_MAGIC) &&
        zy100_get_u32_le(&s_storage->page[12]) != r->generation) return false;
    return true;
}

static void storage_poll_recovery_step(void)
{
    uint32_t address;
    uint32_t sector_end;
    int event_status;
    uint32_t event_crc;
    switch (s_storage->job)
    {
    case STORAGE_JOB_RECOVERY_SCAN_BEGIN:
        if (s_storage->result_head >= ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE)
        {
            if (s_storage->recovery_blocked || !storage_recovery_journal_safe()) goto fail;
            if (!s_storage->recovery_apply && s_storage->recovery_all_erased &&
                !s_storage->journal_pending_reclaim && !s_storage->foreign_purge_active)
            {
                if (!storage_recovery_complete()) goto fail;
                ZY100_OFFLINE_V2_LOG("[BOOT_STORAGE] empty_ready passes=1 bytes=%lu polls=%lu",
                    (unsigned long)s_storage->recovery_read_bytes, (unsigned long)s_storage->recovery_polls);
                return;
            }
            if (!s_storage->recovery_apply)
            {
                ZY100_OFFLINE_V2_LOG("[OFFLINE_REPAIR] scan_ok sessions=%lu apply=1",
                                     (unsigned long)s_storage->session_count);
                s_storage->recovery_apply = true;
                for (address = s_storage->session_count; address > 0U; address--)
                    if (!s_storage->session[address - 1U].confirmed) storage_remove_session(address - 1U);
                s_storage->result_head = 0U;
                s_storage->recovery_high_water = 0U;
                s_storage->recovery_data_seen = false;
                return;
            }
            ZY100_OFFLINE_V2_LOG("[OFFLINE_REPAIR] complete sessions=%lu head=%lu bytes=%lu polls=%lu",
                (unsigned long)s_storage->session_count, (unsigned long)s_storage->recovery_high_water,
                (unsigned long)s_storage->recovery_read_bytes, (unsigned long)s_storage->recovery_polls);
            if (!storage_recovery_complete()) goto fail;
            return;
        }
        if (!storage_recovery_read(s_storage->result_head, s_storage->page, OFFLINE_V2_PAGE_BYTES)) goto fail;
        if (storage_reclaim_at(s_storage->result_head) != NULL)
        {
            if (!storage_page_erased(s_storage->page, OFFLINE_V2_PAGE_BYTES) &&
                !storage_recovery_reclaim_page(s_storage->result_head)) goto fail;
            s_storage->recovery_dirty_sector = false;
            s_storage->digest_addr = s_storage->result_head + OFFLINE_V2_PAGE_BYTES;
            s_storage->job = STORAGE_JOB_RECOVERY_FREE_SCAN;
            return;
        }
        if (!storage_recovery_begin_valid())
        {
            bool erased = storage_page_erased(s_storage->page, OFFLINE_V2_PAGE_BYTES);
            /* Only a complete supported BEGIN body with an uncommitted tag,
             * followed by fifteen erased pages, proves a disposable sector. */
            uint32_t commit = zy100_get_u32_le(&s_storage->page[OFFLINE_V2_PAGE_COMMIT_OFFSET]);
            s_storage->recovery_dirty_sector = false;
            if (!erased && commit != OFFLINE_V2_BEGIN_COMMIT)
            {
                zy100_put_u32_le(&s_storage->page[OFFLINE_V2_PAGE_COMMIT_OFFSET], OFFLINE_V2_BEGIN_COMMIT);
                s_storage->recovery_dirty_sector = storage_recovery_begin_valid();
            }
            if (!erased && !s_storage->recovery_dirty_sector &&
                !storage_recovery_reclaim_page(s_storage->result_head)) s_storage->recovery_blocked = true;
            s_storage->digest_addr = s_storage->result_head + OFFLINE_V2_PAGE_BYTES;
            s_storage->job = STORAGE_JOB_RECOVERY_FREE_SCAN;
            return;
        }
        s_storage->recovery_begin = s_storage->result_head;
        s_storage->active_session_id = zy100_get_u32_le(&s_storage->page[0x08]);
        s_storage->active_generation = zy100_get_u32_le(&s_storage->page[0x0C]);
        s_storage->active_begin_version =
            zy100_get_u16_le(&s_storage->page[0x04]);
        s_storage->active_manifest_version = s_storage->page[0x44];
        s_storage->active_owner_user_id =
            (s_storage->active_begin_version == ZY100_OFFLINE_V2_BEGIN_VERSION) ?
            zy100_get_u32_le(
                &s_storage->page[
                    ZY100_OFFLINE_V2_BEGIN_OWNER_USER_ID_OFFSET]) : 0U;
        s_storage->active_start_lo = zy100_get_u32_le(&s_storage->page[0x10]);
        s_storage->active_start_hi = zy100_get_u32_le(&s_storage->page[0x14]);
        s_storage->active_begin_crc = zy100_get_u32_le(&s_storage->page[OFFLINE_V2_PAGE_CRC_OFFSET]);
        s_storage->recovery_event_count = 0U;
        s_storage->recovery_stream_crc = zy100_crc32_ieee_begin();
        s_storage->recovery_first_center = 0xFFFFFFFFUL;
        s_storage->recovery_last_center = 0xFFFFFFFFUL;
        s_storage->job = STORAGE_JOB_RECOVERY_SCAN_EVENT;
        return;



    case STORAGE_JOB_RECOVERY_FREE_SCAN:
        while (s_storage->recovery_poll_pages < OFFLINE_V2_RECOVERY_READ_PAGES_PER_POLL &&
               s_storage->digest_addr < s_storage->result_head + OFFLINE_V2_SECTOR_BYTES)
        {
            if (!storage_recovery_read(s_storage->digest_addr, s_storage->page, OFFLINE_V2_PAGE_BYTES)) goto fail;
            if (!storage_page_erased(s_storage->page, OFFLINE_V2_PAGE_BYTES) &&
                !storage_recovery_reclaim_page(s_storage->digest_addr))
            {
                s_storage->recovery_blocked = true;
                s_storage->recovery_dirty_sector = false;
            }
            s_storage->digest_addr += OFFLINE_V2_PAGE_BYTES;
        }
        if (s_storage->digest_addr < s_storage->result_head + OFFLINE_V2_SECTOR_BYTES) return;
        if (s_storage->recovery_dirty_sector && s_storage->journal_pending_reclaim &&
            s_storage->result_head < s_storage->reclaim_info.end_exclusive &&
            s_storage->result_head + OFFLINE_V2_SECTOR_BYTES > s_storage->reclaim_info.begin_addr) goto fail;
        if (s_storage->recovery_apply && s_storage->recovery_dirty_sector)
        {
            /* Do not erase an extent still named by a pending journal transaction. */
            if (s_storage->journal_pending_reclaim &&
                s_storage->result_head < s_storage->reclaim_info.end_exclusive &&
                s_storage->result_head + OFFLINE_V2_SECTOR_BYTES > s_storage->reclaim_info.begin_addr) goto fail;
            if (gd25q32e_sector_erase_4k(s_storage->result_head) != IMU_STATUS_OK) goto fail;
            ZY100_OFFLINE_V2_LOG("[OFFLINE_REPAIR] dirty_sector addr=%lu bytes=4096",
                                 (unsigned long)s_storage->result_head);
            s_storage->job = STORAGE_JOB_RECOVERY_ERASE_WAIT;
            return;
        }
        s_storage->result_head += OFFLINE_V2_SECTOR_BYTES;
        s_storage->job = STORAGE_JOB_RECOVERY_SCAN_BEGIN;
        return;

    case STORAGE_JOB_RECOVERY_ERASE_WAIT:
        if (!storage_wip_complete()) return;
        s_storage->digest_addr = s_storage->result_head;
        s_storage->job = STORAGE_JOB_RECOVERY_ERASE_VERIFY;
        return;
    case STORAGE_JOB_RECOVERY_ERASE_VERIFY:
        if (!storage_recovery_read(s_storage->digest_addr, s_storage->page, OFFLINE_V2_PAGE_BYTES) ||
            !storage_page_erased(s_storage->page, OFFLINE_V2_PAGE_BYTES)) goto fail;
        s_storage->digest_addr += OFFLINE_V2_PAGE_BYTES;
        if (s_storage->digest_addr == s_storage->result_head + OFFLINE_V2_SECTOR_BYTES)
        {
            s_storage->result_head = s_storage->digest_addr;
            s_storage->job = STORAGE_JOB_RECOVERY_SCAN_BEGIN;
        }
        return;

    case STORAGE_JOB_RECOVERY_SCAN_EVENT:
        address = s_storage->recovery_begin + OFFLINE_V2_PAGE_BYTES +
                  s_storage->recovery_event_count * ZY100_OFFLINE_V2_EVENT_SLOT_BYTES;
        if (!storage_range_valid(address, ZY100_OFFLINE_V2_EVENT_SLOT_BYTES,
                                  ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE)) goto fail;
        {
            uint32_t center;
            event_status = storage_event_page_check(address, s_storage->active_session_id,
                                                     s_storage->recovery_event_count, &event_crc, &center);
            if (event_status < 0) goto fail;
            if (!storage_recovery_read(address, s_storage->page, 256U) ||
                !storage_recovery_read(address + 256U, s_storage->page2, 256U)) goto fail;
            if (event_status == 1)
            {
                uint32_t crc;
                /* Re-read data used for the stream must match the verified record. */
                zy100_put_u32_le(&s_storage->page[ZY100_OFFLINE_V2_EVENT_CRC_OFFSET], 0U);
                crc = zy100_crc32_ieee_update(zy100_crc32_ieee_begin(), s_storage->page, 256U);
                crc = zy100_crc32_ieee_finish(zy100_crc32_ieee_update(crc, s_storage->page2, 48U));
                zy100_put_u32_le(&s_storage->page[ZY100_OFFLINE_V2_EVENT_CRC_OFFSET], event_crc);
                if (crc != event_crc) goto fail;
                if (s_storage->recovery_event_count == 0U) s_storage->recovery_first_center = center;
                s_storage->recovery_last_center = center;
                s_storage->recovery_stream_crc = zy100_crc32_ieee_update(s_storage->recovery_stream_crc, s_storage->page, 256U);
                s_storage->recovery_stream_crc = zy100_crc32_ieee_update(s_storage->recovery_stream_crc, s_storage->page2, 48U);
                s_storage->recovery_event_count++;
                return;
            }
        }
        s_storage->recovery_dirty_span = 0U;
        if (address + 256U == storage_align_up_sector(address) &&
            storage_page_erased(s_storage->page, 256U))
            memset(s_storage->page2, 0xFF, 256U); /* next sector belongs to END, scanned separately */
        if (!storage_page_erased(s_storage->page, 256U) || !storage_page_erased(s_storage->page2, 256U))
        {
            if (zy100_get_u32_le(s_storage->page) != ZY100_OFFLINE_V2_EVENT_MAGIC ||
                zy100_get_u32_le(&s_storage->page[8]) != s_storage->active_session_id ||
                zy100_get_u32_le(&s_storage->page[12]) != s_storage->recovery_event_count) goto fail;
            s_storage->recovery_dirty_span = ZY100_OFFLINE_V2_EVENT_SLOT_BYTES;
        }
        s_storage->active_finalize_sector = storage_align_up_sector(address + s_storage->recovery_dirty_span);
        if (!storage_range_valid(s_storage->active_finalize_sector, OFFLINE_V2_SECTOR_BYTES,
                                  ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE)) goto fail;
        s_storage->digest_addr = address + s_storage->recovery_dirty_span;
        s_storage->recovery_write_addr = 0xFFFFFFFFUL;
        s_storage->pending_slot_addr = 0xFFFFFFFFUL;
        s_storage->job = STORAGE_JOB_RECOVERY_GAP_SCAN;
        return;

    case STORAGE_JOB_RECOVERY_GAP_SCAN:
        if (s_storage->digest_addr < s_storage->active_finalize_sector)
        {
            if (!storage_recovery_read(s_storage->digest_addr, s_storage->page, 256U) ||
                !storage_page_erased(s_storage->page, 256U)) goto fail;
            s_storage->digest_addr += OFFLINE_V2_PAGE_BYTES;
            return;
        }
        s_storage->job = STORAGE_JOB_RECOVERY_SCAN_END;
        return;

    case STORAGE_JOB_RECOVERY_SCAN_END:
        sector_end = s_storage->active_finalize_sector + OFFLINE_V2_SECTOR_BYTES;
        if (s_storage->digest_addr == sector_end)
        {
            if (s_storage->pending_slot_addr != 0xFFFFFFFFUL)
            {
                if (!storage_recovery_read(s_storage->pending_slot_addr, s_storage->page, 256U) ||
                    !storage_recovery_accept()) goto fail;
                return;
            }
            if (s_storage->recovery_write_addr == 0xFFFFFFFFUL) goto fail;
            zy100_offline_v2_quality_t quality;

            memset(&quality, 0, sizeof(quality));
            s_storage->active_begin = s_storage->recovery_begin;
            s_storage->active_event_count =
                s_storage->recovery_event_count;
            s_storage->active_stream_crc =
                s_storage->recovery_stream_crc;
            s_storage->active_first_center =
                s_storage->recovery_first_center;
            s_storage->active_last_center =
                s_storage->recovery_last_center;
            s_storage->active_dirty_span =
                s_storage->recovery_dirty_span;
            storage_build_end(ZY100_OFFLINE_V2_STOP_POWER_LOSS_RECOVERED,
                              false, 0U, &quality);

            if (!s_storage->recovery_apply)
            {
                if (!storage_recovery_accept()) goto fail;
                return;
            }
            /* The full read-only pass succeeded; recheck the chosen page immediately. */
            if (!storage_recovery_read(s_storage->recovery_write_addr, s_storage->page2, 256U) ||
                !storage_page_erased(s_storage->page2, 256U) ||
                !storage_program_full_page(s_storage->recovery_write_addr, s_storage->page)) goto fail;
            ZY100_OFFLINE_V2_LOG("[OFFLINE_REPAIR] prefix session=%lu events=%lu end=%lu",
                (unsigned long)s_storage->active_session_id, (unsigned long)s_storage->recovery_event_count,
                (unsigned long)s_storage->recovery_write_addr);
            s_storage->job = STORAGE_JOB_RECOVERY_END_WAIT;
            return;
        }
        if (!storage_recovery_read(s_storage->digest_addr, s_storage->page, 256U)) goto fail;
        if (storage_page_erased(s_storage->page, 256U))
        {
            if (s_storage->recovery_write_addr == 0xFFFFFFFFUL)
                s_storage->recovery_write_addr = s_storage->digest_addr;
        }
        else
        {
            if (zy100_get_u32_le(s_storage->page) != OFFLINE_V2_END_MAGIC ||
                zy100_get_u32_le(&s_storage->page[8]) != s_storage->active_session_id ||
                zy100_get_u32_le(&s_storage->page[12]) != s_storage->active_generation ||
                zy100_get_u16_le(&s_storage->page[4]) != 2U ||
                zy100_get_u16_le(&s_storage->page[6]) != OFFLINE_V2_PAGE_BYTES ||
                zy100_get_u32_le(&s_storage->page[0x4C]) != ZY100_OFFLINE_V2_CONFIG_CRC32) goto fail;
            if (storage_page_crc_valid(s_storage->page, OFFLINE_V2_END_MAGIC, OFFLINE_V2_END_COMMIT))
            {
                if (zy100_get_u16_le(&s_storage->page[4]) != 2U ||
                    zy100_get_u32_le(&s_storage->page[0x10]) != s_storage->recovery_event_count ||
                    zy100_get_u32_le(&s_storage->page[0x14]) != s_storage->recovery_event_count * ZY100_OFFLINE_V2_EVENT_RECORD_BYTES ||
                    zy100_get_u32_le(&s_storage->page[0x1C]) != zy100_crc32_ieee_finish(s_storage->recovery_stream_crc) ||
                    zy100_get_u32_le(&s_storage->page[0x4C]) != ZY100_OFFLINE_V2_CONFIG_CRC32 ||
                    zy100_get_u32_le(&s_storage->page[0x50]) != sector_end - s_storage->recovery_begin) goto fail;
                if (s_storage->pending_slot_addr != 0xFFFFFFFFUL) goto fail;
                s_storage->pending_slot_addr = s_storage->digest_addr;
            }
        }
        s_storage->digest_addr += OFFLINE_V2_PAGE_BYTES;
        return;

    case STORAGE_JOB_RECOVERY_END_WAIT:
        if (!storage_wip_complete()) return;
        s_storage->job = STORAGE_JOB_RECOVERY_END_VERIFY;
        return;
    case STORAGE_JOB_RECOVERY_END_VERIFY:
        if (!storage_recovery_read(s_storage->recovery_write_addr, s_storage->page2, 256U) ||
            memcmp(s_storage->page, s_storage->page2, 256U) != 0 || !storage_recovery_accept()) goto fail;
        return;
    default:
        break;
    }
fail:
    ZY100_OFFLINE_V2_LOG("[OFFLINE_REPAIR] isolated addr=%lu phase=%u apply=%u",
        (unsigned long)s_storage->result_head, (uint32_t)s_storage->job, s_storage->recovery_apply ? 1U : 0U);
    storage_fail();
}

static zy100_offline_v2_session_info_t storage_active_info(bool finalized)
{
    zy100_offline_v2_session_info_t info;

    memset(&info, 0, sizeof(info));
    info.session_id = s_storage->active_session_id;
    info.generation = s_storage->active_generation;
    info.begin_addr = s_storage->active_begin;
    info.end_exclusive = finalized ?
        (s_storage->active_finalize_sector + OFFLINE_V2_SECTOR_BYTES) :
        (s_storage->active_begin + OFFLINE_V2_PAGE_BYTES);
    info.event_count = s_storage->active_event_count;
    info.logical_bytes = s_storage->active_event_count *
                         ZY100_OFFLINE_V2_EVENT_RECORD_BYTES;
    info.stream_crc32 = finalized ?
        zy100_crc32_ieee_finish(s_storage->active_stream_crc) : 0U;
    info.owner_user_id = s_storage->active_owner_user_id;
    info.begin_version = s_storage->active_begin_version;
    info.manifest_version = s_storage->active_manifest_version;
    info.stop_reason = s_storage->active_stop_reason;
    info.clean = s_storage->active_clean;
    info.health = s_storage->active_health;
    info.quality = s_storage->active_quality;
    info.finalized = finalized;
    info.start_unix_lo = s_storage->active_start_lo;
    info.start_unix_hi = s_storage->active_start_hi;
    info.duration_ms = s_storage->active_duration_ms;
    info.reclaim_next = info.begin_addr;
    return info;
}

/* Batch read-only empty-sector transitions. Never loop a WIP wait, repair
 * write, or event job here. Each call returns after at most 32 page pairs. */
static void storage_poll_recovery(void)
{
    if (spi_bus_current_owner() != SPI_OWNER_NONE) return;
    s_storage->recovery_poll_pages = 0U;
    s_storage->recovery_polls++;
    do
    {
        offline_v2_storage_job_t before = s_storage->job;
        uint32_t head = s_storage->result_head;
        bool apply = s_storage->recovery_apply;
        storage_poll_recovery_step();
        if (s_storage->state != ZY100_OFFLINE_V2_STORAGE_RECOVERING ||
            apply != s_storage->recovery_apply ||
            (before != STORAGE_JOB_RECOVERY_SCAN_BEGIN && before != STORAGE_JOB_RECOVERY_FREE_SCAN) ||
            (s_storage->job != STORAGE_JOB_RECOVERY_SCAN_BEGIN && s_storage->job != STORAGE_JOB_RECOVERY_FREE_SCAN) ||
            (before == s_storage->job && head == s_storage->result_head)) break;
    } while (s_storage->recovery_poll_pages < OFFLINE_V2_RECOVERY_READ_PAGES_PER_POLL);
}

/* The inactive bank is not visible until every snapshot page has passed
 * readback. The header is the final commit; the previous bank is untouched. */
static bool storage_snapshot_next(void)
{
    while (s_storage->snapshot_index < s_storage->session_count)
    {
        const zy100_offline_v2_session_info_t *info =
            &s_storage->session[s_storage->snapshot_index++];
        if (!info->confirmed) continue;
        if (!storage_build_journal_entry(
                s_storage->foreign_purge_active && (info->confirm_transfer_id & OFFLINE_V2_RECLAIM_DETAIL_DISCARD) != 0U ?
                    OFFLINE_V2_JOURNAL_TYPE_DISCARD_INTENT : OFFLINE_V2_JOURNAL_TYPE_CONFIRMED,
                info, info->confirm_transfer_id)) return false;
        goto record;
    }
    if (s_storage->snapshot_index == s_storage->session_count)
    {
        s_storage->snapshot_index++;
        if (s_storage->tombstone_valid)
        {
            if (!storage_build_journal_entry(OFFLINE_V2_JOURNAL_TYPE_RECLAIM_DONE,
                    &s_storage->tombstone_info, s_storage->tombstone_transfer_id)) return false;
            goto record;
        }
    }
    storage_build_journal_header(true);
    if (!storage_program_full_page(s_storage->journal_rotate_begin, s_storage->page)) return false;
    s_storage->job = STORAGE_JOB_JOURNAL_ROTATE_HEADER_WAIT;
    return true;
record:
    s_storage->snapshot_addr = s_storage->journal_rotate_begin +
        (s_storage->snapshot_count + 1U) * OFFLINE_V2_PAGE_BYTES;
    if (!storage_program_full_page(s_storage->snapshot_addr, s_storage->page)) return false;
    s_storage->job = STORAGE_JOB_JOURNAL_SNAPSHOT_WAIT;
    return true;
}

static void storage_poll_runtime(void)
{
    zy100_offline_v2_session_info_t info;
    uint8_t commit[4];

    switch (s_storage->job)
    {
    case STORAGE_JOB_JOURNAL_ROTATE_ERASE_WAIT:
        if (!storage_wip_complete()) return;
        if (s_storage->journal_generation == 0xFFFFFFFFUL) { storage_fail(); return; }
        s_storage->journal_generation++;
        s_storage->snapshot_index = s_storage->snapshot_count = 0U;
        if (!storage_snapshot_next()) storage_fail();
        return;
    case STORAGE_JOB_JOURNAL_SNAPSHOT_WAIT:
        if (!storage_wip_complete()) return;
        if (gd25q32e_read(s_storage->snapshot_addr, s_storage->verify, OFFLINE_V2_PAGE_BYTES) != IMU_STATUS_OK ||
            memcmp(s_storage->page, s_storage->verify, OFFLINE_V2_PAGE_BYTES) != 0)
        { storage_fail(); return; }
        s_storage->snapshot_count++;
        if (!storage_snapshot_next()) storage_fail();
        return;
    case STORAGE_JOB_JOURNAL_ROTATE_HEADER_WAIT:
        if (!storage_wip_complete()) return;
        s_storage->job = STORAGE_JOB_JOURNAL_ROTATE_HEADER_VERIFY;
        return;
    case STORAGE_JOB_JOURNAL_ROTATE_HEADER_VERIFY:
        if ((gd25q32e_read(s_storage->journal_rotate_begin,
                           s_storage->verify, OFFLINE_V2_PAGE_BYTES) !=
             IMU_STATUS_OK) ||
            (memcmp(s_storage->page, s_storage->verify,
                    OFFLINE_V2_PAGE_BYTES) != 0))
        {
            storage_fail(); return;
        }
        s_storage->journal_bank_begin = s_storage->journal_rotate_begin;
        s_storage->journal_version = ZY100_OFFLINE_V2_JOURNAL_RUNTIME_VERSION;
        s_storage->journal_next_addr = s_storage->journal_bank_begin +
            (s_storage->snapshot_count + 1U) * OFFLINE_V2_PAGE_BYTES;
        memcpy(s_storage->page, s_storage->page2, OFFLINE_V2_PAGE_BYTES);
        if (!storage_program_full_page(s_storage->journal_next_addr,
                                       s_storage->page))
        {
            storage_fail(); return;
        }
        s_storage->job = s_storage->journal_resume_job;
        return;

    case STORAGE_JOB_BEGIN_WAIT:
        if (!storage_wip_complete()) return;
        s_storage->job = STORAGE_JOB_BEGIN_VERIFY;
        return;
    case STORAGE_JOB_BEGIN_VERIFY:
        if ((gd25q32e_read(s_storage->active_begin, s_storage->verify,
                           256U) != IMU_STATUS_OK) ||
            (memcmp(s_storage->page, s_storage->verify, 256U) != 0))
        {
            storage_fail(); return;
        }
        info = storage_active_info(false);
        if (!storage_program_journal_entry(
                OFFLINE_V2_JOURNAL_TYPE_OPEN, &info, 0U,
                STORAGE_JOB_BEGIN_JOURNAL_WAIT))
        {
            storage_fail(); return;
        }
        return;
    case STORAGE_JOB_BEGIN_JOURNAL_WAIT:
        if (!storage_wip_complete()) return;
        s_storage->job = STORAGE_JOB_BEGIN_JOURNAL_VERIFY;
        return;
    case STORAGE_JOB_BEGIN_JOURNAL_VERIFY:
        if ((gd25q32e_read(s_storage->journal_next_addr, s_storage->verify,
                           256U) != IMU_STATUS_OK) ||
            (memcmp(s_storage->page, s_storage->verify, 256U) != 0))
        {
            storage_fail(); return;
        }
        s_storage->journal_next_addr += 256U;
        s_storage->journal_seq++;
        s_storage->active = true;
        s_storage->begin_pending = false;
        s_storage->last_checkpoint_event_count = 0U;
        s_storage->last_checkpoint_ms = 0U;
        s_storage->state = ZY100_OFFLINE_V2_STORAGE_READY;
        s_storage->job = STORAGE_JOB_NONE;
        return;

    case STORAGE_JOB_EVENT_PAGE0_WAIT:
        if (!storage_wip_complete()) return;
        s_storage->job = STORAGE_JOB_EVENT_PAGE0_VERIFY;
        return;
    case STORAGE_JOB_EVENT_PAGE0_VERIFY:
        if ((gd25q32e_read(s_storage->pending_slot_addr,
                           s_storage->verify, 256U) != IMU_STATUS_OK) ||
            (memcmp(s_storage->page, s_storage->verify, 256U) != 0) ||
            !storage_program_full_page(s_storage->pending_slot_addr + 256U,
                                       s_storage->page2))
        {
            storage_fail(); return;
        }
        s_storage->job = STORAGE_JOB_EVENT_PAGE1_WAIT;
        return;
    case STORAGE_JOB_EVENT_PAGE1_WAIT:
        if (!storage_wip_complete()) return;
        s_storage->job = STORAGE_JOB_EVENT_PAGE1_VERIFY;
        return;
    case STORAGE_JOB_EVENT_PAGE1_VERIFY:
        if ((gd25q32e_read(s_storage->pending_slot_addr + 256U,
                           s_storage->verify, 256U) != IMU_STATUS_OK) ||
            (memcmp(s_storage->page2, s_storage->verify, 256U) != 0))
        {
            storage_fail(); return;
        }
        zy100_put_u32_le(commit, OFFLINE_V2_EVENT_COMMIT);
        if (!storage_range_valid(s_storage->pending_slot_addr +
                                 ZY100_OFFLINE_V2_EVENT_COMMIT_OFFSET,
                                 4U,
                                 ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE) ||
            (gd25q32e_page_program_partial_erased(
                 s_storage->pending_slot_addr +
                 ZY100_OFFLINE_V2_EVENT_COMMIT_OFFSET,
                 commit, 4U) != IMU_STATUS_OK))
        {
            storage_fail(); return;
        }
        s_storage->job = STORAGE_JOB_EVENT_COMMIT_WAIT;
        return;
    case STORAGE_JOB_EVENT_COMMIT_WAIT:
        if (!storage_wip_complete()) return;
        s_storage->job = STORAGE_JOB_EVENT_COMMIT_VERIFY;
        return;
    case STORAGE_JOB_EVENT_COMMIT_VERIFY:
        if ((gd25q32e_read(s_storage->pending_slot_addr +
                           ZY100_OFFLINE_V2_EVENT_COMMIT_OFFSET,
                           s_storage->verify, 4U) != IMU_STATUS_OK) ||
            (zy100_get_u32_le(s_storage->verify) != OFFLINE_V2_EVENT_COMMIT))
        {
            storage_fail(); return;
        }
        if (s_storage->active_event_count == 0U)
        {
            s_storage->active_first_center = s_storage->pending_center;
        }
        s_storage->active_last_center = s_storage->pending_center;
        s_storage->active_stream_crc = zy100_crc32_ieee_update(
            s_storage->active_stream_crc, s_storage->page, 256U);
        s_storage->active_stream_crc = zy100_crc32_ieee_update(
            s_storage->active_stream_crc, s_storage->page2, 48U);
        s_storage->active_event_count++;
        s_storage->job = STORAGE_JOB_NONE;
        return;

    case STORAGE_JOB_CHECKPOINT_JOURNAL_WAIT:
        if (!storage_wip_complete()) return;
        s_storage->job = STORAGE_JOB_CHECKPOINT_JOURNAL_VERIFY;
        return;
    case STORAGE_JOB_CHECKPOINT_JOURNAL_VERIFY:
        if ((gd25q32e_read(s_storage->journal_next_addr,
                           s_storage->verify, 256U) != IMU_STATUS_OK) ||
            (memcmp(s_storage->page, s_storage->verify, 256U) != 0))
        {
            storage_fail(); return;
        }
        s_storage->journal_next_addr += 256U;
        s_storage->journal_seq++;
        s_storage->last_checkpoint_event_count =
            s_storage->active_event_count;
        s_storage->last_checkpoint_ms =
            s_storage->pending_checkpoint_ms;
        s_storage->job = STORAGE_JOB_NONE;
        return;

    case STORAGE_JOB_END_WAIT:
        if (!storage_wip_complete()) return;
        s_storage->job = STORAGE_JOB_END_VERIFY;
        return;
    case STORAGE_JOB_END_VERIFY:
        if ((gd25q32e_read(s_storage->active_finalize_sector,
                           s_storage->verify, 256U) != IMU_STATUS_OK) ||
            (memcmp(s_storage->page, s_storage->verify, 256U) != 0))
        {
            storage_fail(); return;
        }
        info = storage_active_info(true);
        if (!storage_program_journal_entry(
                OFFLINE_V2_JOURNAL_TYPE_FINAL, &info, 0U,
                STORAGE_JOB_END_JOURNAL_WAIT))
        {
            storage_fail(); return;
        }
        return;
    case STORAGE_JOB_END_JOURNAL_WAIT:
        if (!storage_wip_complete()) return;
        s_storage->job = STORAGE_JOB_END_JOURNAL_VERIFY;
        return;
    case STORAGE_JOB_END_JOURNAL_VERIFY:
        if ((gd25q32e_read(s_storage->journal_next_addr, s_storage->verify,
                           256U) != IMU_STATUS_OK) ||
            (memcmp(s_storage->page, s_storage->verify, 256U) != 0) ||
            (s_storage->session_count >= OFFLINE_V2_SESSION_MAX))
        {
            storage_fail(); return;
        }
        s_storage->journal_next_addr += 256U;
        s_storage->journal_seq++;
        info = storage_active_info(true);
        s_storage->session[s_storage->session_count++] = info;
        s_storage->directory_revision++;
        s_storage->result_head = info.end_exclusive;
        s_storage->active = false;
        s_storage->finalize_pending = false;
        s_storage->state =
            (zy100_offline_v2_storage_remaining_percent() <=
             OFFLINE_V2_ERASED_PERCENT_LOCK) ?
            ZY100_OFFLINE_V2_STORAGE_LOCKED_5_PERCENT :
            ZY100_OFFLINE_V2_STORAGE_READY;
        s_storage->job = STORAGE_JOB_NONE;
        return;

    case STORAGE_JOB_CONFIRM_JOURNAL_WAIT:
        if (!storage_wip_complete()) return;
        s_storage->job = STORAGE_JOB_CONFIRM_JOURNAL_VERIFY;
        return;
    case STORAGE_JOB_CONFIRM_JOURNAL_VERIFY:
        if ((gd25q32e_read(s_storage->journal_next_addr, s_storage->verify,
                           256U) != IMU_STATUS_OK) ||
            (memcmp(s_storage->page, s_storage->verify, 256U) != 0))
        {
            storage_fail(); return;
        }
        s_storage->journal_next_addr += 256U;
        s_storage->journal_seq++;
        if (s_storage->reclaim_session_index >= s_storage->session_count)
        {
            storage_fail(); return;
        }
        s_storage->session[s_storage->reclaim_session_index].confirmed = true;
        s_storage->directory_revision++;
        s_storage->journal_pending_reclaim = true;
        if (!storage_start_reclaim(
                &s_storage->session[s_storage->reclaim_session_index],
                s_storage->reclaim_session_index))
        {
            storage_fail(); return;
        }
        return;
    case STORAGE_JOB_FOREIGN_PURGE_BEGIN_WAIT:
        if (!storage_wip_complete()) return;
        s_storage->job = STORAGE_JOB_FOREIGN_PURGE_BEGIN_VERIFY;
        return;
    case STORAGE_JOB_FOREIGN_PURGE_BEGIN_VERIFY:
        if ((gd25q32e_read(s_storage->journal_next_addr, s_storage->verify,
                           OFFLINE_V2_PAGE_BYTES) != IMU_STATUS_OK) ||
            (memcmp(s_storage->page, s_storage->verify,
                    OFFLINE_V2_PAGE_BYTES) != 0))
        {
            storage_fail(); return;
        }
        s_storage->journal_next_addr += OFFLINE_V2_PAGE_BYTES;
        s_storage->journal_seq++;
        if (!storage_foreign_purge_start_next())
        {
            storage_fail();
        }
        return;
    case STORAGE_JOB_FOREIGN_PURGE_DISCARD_WAIT:
        if (!storage_wip_complete()) return;
        s_storage->job = STORAGE_JOB_FOREIGN_PURGE_DISCARD_VERIFY;
        return;
    case STORAGE_JOB_FOREIGN_PURGE_DISCARD_VERIFY:
        if ((gd25q32e_read(s_storage->journal_next_addr, s_storage->verify,
                           OFFLINE_V2_PAGE_BYTES) != IMU_STATUS_OK) ||
            (memcmp(s_storage->page, s_storage->verify,
                    OFFLINE_V2_PAGE_BYTES) != 0))
        {
            storage_fail(); return;
        }
        s_storage->journal_next_addr += OFFLINE_V2_PAGE_BYTES;
        s_storage->journal_seq++;
        if (s_storage->reclaim_session_index < s_storage->session_count)
        {
            zy100_offline_v2_session_info_t *p = &s_storage->session[s_storage->reclaim_session_index];
            p->confirmed = true;
            p->confirm_transfer_id = s_storage->confirm_transfer_id;
            p->reclaim_next = p->begin_addr;
            s_storage->reclaim_info = *p;
        }
        s_storage->journal_pending_reclaim = true;
        if (!storage_start_reclaim(&s_storage->reclaim_info,
                                   s_storage->reclaim_session_index))
        {
            storage_fail();
        }
        return;
    case STORAGE_JOB_RECLAIM_WAIT:
        if (!storage_wip_complete()) return;
        s_storage->reclaim_verify_addr = s_storage->reclaim_addr;
        s_storage->job = STORAGE_JOB_RECLAIM_VERIFY;
        return;
    case STORAGE_JOB_RECLAIM_VERIFY:
        if (gd25q32e_read(s_storage->reclaim_verify_addr, s_storage->verify, OFFLINE_V2_PAGE_BYTES) != IMU_STATUS_OK ||
            !storage_page_erased(s_storage->verify, OFFLINE_V2_PAGE_BYTES)) { storage_fail(); return; }
        s_storage->reclaim_verify_addr += OFFLINE_V2_PAGE_BYTES;
        if (s_storage->reclaim_verify_addr < s_storage->reclaim_addr + OFFLINE_V2_SECTOR_BYTES) return;
        if (s_storage->reclaim_from_foreign_purge)
        {
            s_storage->foreign_purge_erased_sectors++;
        }
        s_storage->reclaim_addr += OFFLINE_V2_SECTOR_BYTES;
        s_storage->reclaim_info.reclaim_next = s_storage->reclaim_addr;
        if (s_storage->reclaim_session_index < s_storage->session_count)
            s_storage->session[s_storage->reclaim_session_index].reclaim_next = s_storage->reclaim_addr;
        if (s_storage->reclaim_paused && !s_storage->reclaim_from_foreign_purge)
        {
            if (!storage_program_journal_entry(OFFLINE_V2_JOURNAL_TYPE_RECLAIM_PROGRESS,
                    &s_storage->reclaim_info, s_storage->confirm_transfer_id,
                    STORAGE_JOB_RECLAIM_PROGRESS_WAIT)) storage_fail();
            return;
        }
        if (s_storage->reclaim_addr < s_storage->reclaim_info.end_exclusive)
        {
            if (gd25q32e_sector_erase_4k(s_storage->reclaim_addr) !=
                IMU_STATUS_OK)
            {
                storage_fail(); return;
            }
            s_storage->job = STORAGE_JOB_RECLAIM_WAIT;
            return;
        }
        info = s_storage->reclaim_info;
        if (!storage_program_journal_entry(
                OFFLINE_V2_JOURNAL_TYPE_RECLAIM_DONE, &info,
                s_storage->confirm_transfer_id,
                STORAGE_JOB_RECLAIM_JOURNAL_WAIT))
        {
            storage_fail(); return;
        }
        return;
    case STORAGE_JOB_RECLAIM_PROGRESS_WAIT:
        if (!storage_wip_complete()) return;
        s_storage->job = STORAGE_JOB_RECLAIM_PROGRESS_VERIFY;
        return;
    case STORAGE_JOB_RECLAIM_PROGRESS_VERIFY:
        if (gd25q32e_read(s_storage->journal_next_addr, s_storage->verify, OFFLINE_V2_PAGE_BYTES) != IMU_STATUS_OK ||
            memcmp(s_storage->page, s_storage->verify, OFFLINE_V2_PAGE_BYTES) != 0) { storage_fail(); return; }
        s_storage->journal_next_addr += OFFLINE_V2_PAGE_BYTES;
        s_storage->journal_seq++;
        s_storage->journal_pending_reclaim = false;
        s_storage->state = ZY100_OFFLINE_V2_STORAGE_READY;
        s_storage->job = STORAGE_JOB_NONE;
        return;
    case STORAGE_JOB_RECLAIM_JOURNAL_WAIT:
        if (!storage_wip_complete()) return;
        s_storage->job = STORAGE_JOB_RECLAIM_JOURNAL_VERIFY;
        return;
    case STORAGE_JOB_RECLAIM_JOURNAL_VERIFY:
        if ((gd25q32e_read(s_storage->journal_next_addr, s_storage->verify,
                           256U) != IMU_STATUS_OK) ||
            (memcmp(s_storage->page, s_storage->verify, 256U) != 0))
        {
            storage_fail(); return;
        }
        s_storage->journal_next_addr += 256U;
        s_storage->journal_seq++;
        s_storage->journal_pending_reclaim = false;
        s_storage->tombstone_info = s_storage->reclaim_info;
        s_storage->tombstone_transfer_id =
            s_storage->confirm_transfer_id;
        s_storage->tombstone_valid = true;
        storage_remove_session(storage_find_identity(s_storage->reclaim_info.session_id,
            s_storage->reclaim_info.generation));
        s_storage->reclaim_session_index = OFFLINE_V2_SESSION_INDEX_INVALID;
        /* Only the contiguous tail is released. Every owner's extent counts. */
        s_storage->result_head = 0U;
        {
            uint32_t i;
            for (i = 0U; i < s_storage->session_count; i++)
                if (s_storage->session[i].end_exclusive > s_storage->result_head)
                    s_storage->result_head = s_storage->session[i].end_exclusive;
        }
        if (s_storage->reclaim_from_foreign_purge)
        {
            s_storage->reclaim_from_foreign_purge = false;
            if (!storage_foreign_purge_start_next())
            {
                storage_fail();
            }
            return;
        }
        s_storage->state = ZY100_OFFLINE_V2_STORAGE_READY;
        s_storage->job = STORAGE_JOB_NONE;
        return;

    case STORAGE_JOB_FOREIGN_PURGE_DONE_WAIT:
        if (!storage_wip_complete()) return;
        s_storage->job = STORAGE_JOB_FOREIGN_PURGE_DONE_VERIFY;
        return;
    case STORAGE_JOB_FOREIGN_PURGE_DONE_VERIFY:
        if ((gd25q32e_read(s_storage->journal_next_addr, s_storage->verify,
                           OFFLINE_V2_PAGE_BYTES) != IMU_STATUS_OK) ||
            (memcmp(s_storage->page, s_storage->verify,
                    OFFLINE_V2_PAGE_BYTES) != 0))
        {
            storage_fail(); return;
        }
        s_storage->journal_next_addr += OFFLINE_V2_PAGE_BYTES;
        s_storage->journal_seq++;
        s_storage->foreign_purge_done_user_id =
            s_storage->foreign_purge_protected_user_id;
        s_storage->foreign_purge_done_batch_token =
            s_storage->foreign_purge_batch_token;
        s_storage->foreign_purge_done_total_sectors =
            s_storage->foreign_purge_total_sectors;
        s_storage->foreign_purge_completion_pending = true;
        s_storage->foreign_purge_completion_failed = false;
        s_storage->foreign_purge_active = false;
        s_storage->foreign_purge_protected_user_id = 0U;
        s_storage->foreign_purge_batch_token = 0U;
        s_storage->foreign_purge_total_sectors = 0U;
        s_storage->foreign_purge_erased_sectors = 0U;
        s_storage->foreign_purge_remaining_sessions = 0U;
        s_storage->state =
            (zy100_offline_v2_storage_remaining_percent() <=
             OFFLINE_V2_ERASED_PERCENT_LOCK) ?
            ZY100_OFFLINE_V2_STORAGE_LOCKED_5_PERCENT :
            ZY100_OFFLINE_V2_STORAGE_READY;
        s_storage->job = STORAGE_JOB_NONE;
        return;

    default:
        storage_fail();
        return;
    }
}

void zy100_offline_v2_storage_poll(bool fifo_urgent)
{
    uint8_t state_before;
    uint8_t job_before;

    if ((s_storage == NULL) || fifo_urgent ||
        (s_storage->state == ZY100_OFFLINE_V2_STORAGE_ERROR))
    {
        return;
    }
    state_before = (uint8_t)s_storage->state;
    job_before = (uint8_t)s_storage->job;
    if ((state_before != s_log_last_state) || (job_before != s_log_last_job))
    {
        if (s_storage->state != ZY100_OFFLINE_V2_STORAGE_RECOVERING &&
            !s_storage->begin_pending && !s_storage->active &&
            !s_storage->finalize_pending)
        {
            ZY100_LOG_ROUTINE(ZY100_LOG_EVENT, "[OFFLINE_V2][STORAGE] state=%u job=%u addr=0x%06lX remain=%u",
                                state_before,
                                job_before,
                                (unsigned long)zy100_offline_v2_storage_progress_address(),
                                zy100_offline_v2_storage_remaining_percent());
        }
        s_log_last_state = state_before;
        s_log_last_job = job_before;
    }
    if (s_storage->state == ZY100_OFFLINE_V2_STORAGE_MIGRATING)
    {
        storage_poll_migration();
        {
            uint32_t address = zy100_offline_v2_storage_progress_address();
            uint32_t bucket = address >> 18;
            if (bucket != s_log_last_migration_bucket)
            {
                ZY100_OFFLINE_V2_LOG("[OFFLINE_V2][MIGRATE] progress_addr=0x%06lX job=%u",
                                    (unsigned long)address,
                                    (uint32_t)s_storage->job);
                s_log_last_migration_bucket = bucket;
            }
        }
    }
    else if (s_storage->state == ZY100_OFFLINE_V2_STORAGE_CLEARING)
    {
        storage_poll_clear();
    }
    else if (s_storage->state == ZY100_OFFLINE_V2_STORAGE_RECOVERING)
    {
        storage_poll_recovery();
    }
    else if (s_storage->job != STORAGE_JOB_NONE)
    {
        storage_poll_runtime();
    }
}

zy100_offline_v2_storage_state_t zy100_offline_v2_storage_state(void)
{
    return (s_storage != NULL) ? s_storage->state :
           ZY100_OFFLINE_V2_STORAGE_UNINITIALIZED;
}

bool zy100_offline_v2_storage_ready_for_capture(void)
{
    return (s_storage != NULL) &&
           (s_storage->state == ZY100_OFFLINE_V2_STORAGE_READY) &&
           (s_storage->job == STORAGE_JOB_NONE) && !s_storage->active &&
           (zy100_offline_v2_storage_remaining_percent() >
            OFFLINE_V2_ERASED_PERCENT_LOCK);
}

bool zy100_offline_v2_storage_job_busy(void)
{
    return (s_storage != NULL) && (s_storage->job != STORAGE_JOB_NONE);
}

bool zy100_offline_v2_storage_clear_all_begin(void)
{
    if ((s_storage == NULL) ||
        ((s_storage->state != ZY100_OFFLINE_V2_STORAGE_READY) &&
         (s_storage->state != ZY100_OFFLINE_V2_STORAGE_LOCKED_5_PERCENT)) ||
        (s_storage->job != STORAGE_JOB_NONE) || s_storage->active ||
        s_storage->begin_pending || s_storage->finalize_pending)
    {
        return false;
    }
    s_storage->marker.generation++;
    if (s_storage->marker.generation == 0U)
    {
        s_storage->marker.generation = 1U;
    }
    s_storage->marker.state = ZY100_OFFLINE_V2_MARKER_CLEARING;
    s_storage->marker.device_id = zy100_device_internal_id();
    s_storage->marker.layout_crc32 = ZY100_OFFLINE_V2_LAYOUT_CRC32;
    s_storage->marker.online_spool_crc32 = 0U;
    s_storage->marker.next_erase_addr = 0U;
    s_storage->clear_requested = true;
    s_storage->clear_completed = false;
    s_recovery_anchor = true;
    s_storage->state = ZY100_OFFLINE_V2_STORAGE_CLEARING;
    s_storage->erase_addr = 0U;
    if (!zy100_offline_v2_marker_commit(&s_storage->marker))
    {
        storage_fail();
        return false;
    }
    storage_clear_check_external_block();
    return true;
}

zy100_offline_v2_clear_status_t
zy100_offline_v2_storage_clear_all_poll(void)
{
    if (s_storage == NULL)
    {
        return ZY100_OFFLINE_V2_CLEAR_ERROR;
    }
    zy100_offline_v2_storage_poll(false);
    if (s_storage->state == ZY100_OFFLINE_V2_STORAGE_ERROR)
    {
        return ZY100_OFFLINE_V2_CLEAR_ERROR;
    }
    if (s_storage->clear_completed)
    {
        return ZY100_OFFLINE_V2_CLEAR_DONE;
    }
    if ((s_storage->state == ZY100_OFFLINE_V2_STORAGE_CLEARING) ||
        s_storage->clear_requested)
    {
        return ZY100_OFFLINE_V2_CLEAR_BUSY;
    }
    return ZY100_OFFLINE_V2_CLEAR_IDLE;
}

bool zy100_offline_v2_storage_clear_all_active(void)
{
    return (s_storage != NULL) &&
           ((s_storage->state == ZY100_OFFLINE_V2_STORAGE_CLEARING) ||
            s_storage->clear_requested);
}

zy100_offline_v2_foreign_purge_result_t
zy100_offline_v2_storage_foreign_purge_request(uint32_t protected_user_id,
                                               uint8_t request_seq)
{
    uint32_t target_sessions;
    uint32_t target_sectors;
    uint32_t batch_token;

    if ((s_storage == NULL) || (protected_user_id == 0U))
    {
        return ZY100_OFFLINE_V2_FOREIGN_PURGE_REJECTED;
    }
    if (s_storage->state == ZY100_OFFLINE_V2_STORAGE_ERROR)
    {
        return ZY100_OFFLINE_V2_FOREIGN_PURGE_ERROR;
    }
    if (s_storage->foreign_purge_active)
    {
        return (s_storage->foreign_purge_protected_user_id ==
                protected_user_id) ?
            ZY100_OFFLINE_V2_FOREIGN_PURGE_ACTIVE :
            ZY100_OFFLINE_V2_FOREIGN_PURGE_OWNER_MISMATCH;
    }
    if (((s_storage->state != ZY100_OFFLINE_V2_STORAGE_READY) &&
         (s_storage->state !=
          ZY100_OFFLINE_V2_STORAGE_LOCKED_5_PERCENT)) ||
        (s_storage->job != STORAGE_JOB_NONE) || s_storage->active ||
        s_storage->begin_pending || s_storage->finalize_pending ||
        s_storage->journal_pending_reclaim)
    {
        return ZY100_OFFLINE_V2_FOREIGN_PURGE_REJECTED;
    }

    storage_foreign_purge_count(protected_user_id, &target_sessions,
                                &target_sectors);
    if (target_sessions == 0U)
    {
        return ZY100_OFFLINE_V2_FOREIGN_PURGE_NO_ACTION;
    }
    batch_token = (((s_storage->journal_seq & 0x007FFFFFUL) << 8) |
                   (uint32_t)request_seq) &
                  OFFLINE_V2_RECLAIM_DETAIL_TOKEN_MASK;
    if (batch_token == 0U)
    {
        batch_token = 0x00000100UL | (uint32_t)request_seq;
    }
    s_storage->foreign_purge_protected_user_id = protected_user_id;
    s_storage->foreign_purge_batch_token = batch_token;
    s_storage->foreign_purge_total_sectors = target_sectors;
    s_storage->foreign_purge_erased_sectors = 0U;
    s_storage->foreign_purge_remaining_sessions = target_sessions;
    s_storage->foreign_purge_active = true;
    s_storage->foreign_purge_completion_pending = false;
    s_storage->foreign_purge_completion_failed = false;
    s_storage->state = ZY100_OFFLINE_V2_STORAGE_BUSY;
    if (!storage_program_foreign_purge_marker(
            OFFLINE_V2_JOURNAL_TYPE_FOREIGN_PURGE_BEGIN,
            STORAGE_JOB_FOREIGN_PURGE_BEGIN_WAIT))
    {
        storage_fail();
        return ZY100_OFFLINE_V2_FOREIGN_PURGE_ERROR;
    }
    return ZY100_OFFLINE_V2_FOREIGN_PURGE_ACTIVE;
}

bool zy100_offline_v2_storage_foreign_purge_status(
    uint32_t protected_user_id,
    uint32_t *remaining_sessions_out,
    uint32_t *total_sectors_out,
    uint32_t *erased_sectors_out,
    uint32_t *batch_token_out)
{
    uint32_t remaining_sessions;

    if ((s_storage == NULL) || !s_storage->foreign_purge_active ||
        (protected_user_id == 0U) ||
        (s_storage->foreign_purge_protected_user_id != protected_user_id) ||
        (remaining_sessions_out == NULL) ||
        (total_sectors_out == NULL) || (erased_sectors_out == NULL) ||
        (batch_token_out == NULL))
    {
        return false;
    }
    storage_foreign_purge_count(protected_user_id, &remaining_sessions,
                                NULL);
    if (s_storage->reclaim_from_foreign_purge &&
        (s_storage->reclaim_session_index ==
         OFFLINE_V2_SESSION_INDEX_INVALID))
    {
        remaining_sessions++;
    }
    *remaining_sessions_out = remaining_sessions;
    *total_sectors_out = s_storage->foreign_purge_total_sectors;
    *erased_sectors_out = s_storage->foreign_purge_erased_sectors;
    if (*erased_sectors_out > *total_sectors_out)
    {
        *erased_sectors_out = *total_sectors_out;
    }
    *batch_token_out = s_storage->foreign_purge_batch_token;
    return true;
}

bool zy100_offline_v2_storage_foreign_purge_completion_peek(
    uint32_t *protected_user_id_out,
    uint32_t *batch_token_out,
    uint32_t *total_sectors_out,
    bool *failed_out)
{
    if ((s_storage == NULL) ||
        !s_storage->foreign_purge_completion_pending ||
        (protected_user_id_out == NULL) || (batch_token_out == NULL) ||
        (total_sectors_out == NULL) || (failed_out == NULL))
    {
        return false;
    }
    *protected_user_id_out = s_storage->foreign_purge_done_user_id;
    *batch_token_out = s_storage->foreign_purge_done_batch_token;
    *total_sectors_out = s_storage->foreign_purge_done_total_sectors;
    *failed_out = s_storage->foreign_purge_completion_failed;
    return true;
}

void zy100_offline_v2_storage_foreign_purge_completion_ack(
    uint32_t batch_token)
{
    if ((s_storage != NULL) &&
        s_storage->foreign_purge_completion_pending &&
        (s_storage->foreign_purge_done_batch_token == batch_token))
    {
        s_storage->foreign_purge_completion_pending = false;
    }
}

bool zy100_offline_v2_storage_foreign_purge_active(void)
{
    return (s_storage != NULL) && s_storage->foreign_purge_active;
}

uint8_t zy100_offline_v2_storage_job_code(void)
{
    return (s_storage != NULL) ? (uint8_t)s_storage->job : 0U;
}

uint32_t zy100_offline_v2_storage_progress_address(void)
{
    if (s_storage == NULL)
    {
        return 0U;
    }
    if (s_storage->state == ZY100_OFFLINE_V2_STORAGE_MIGRATING)
    {
        if ((s_storage->job == STORAGE_JOB_MIGRATION_RESULT_BLANK_SCAN) ||
            (s_storage->job == STORAGE_JOB_MIGRATION_DIGEST_BEFORE) ||
            (s_storage->job == STORAGE_JOB_MIGRATION_DIGEST_AFTER))
        {
            return s_storage->digest_addr;
        }
        return s_storage->erase_addr;
    }
    if (s_storage->state == ZY100_OFFLINE_V2_STORAGE_CLEARING)
    {
        return s_storage->erase_addr;
    }
    if (s_storage->job == STORAGE_JOB_RECLAIM_WAIT)
    {
        return s_storage->reclaim_addr;
    }
    return s_storage->result_head;
}

uint8_t zy100_offline_v2_storage_remaining_percent(void)
{
    uint32_t free_bytes;
    uint32_t used_end;
    uint32_t usable = ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE -
                      ZY100_OFFLINE_V2_RESULT_BEGIN;

    if (s_storage == NULL)
    {
        return 0U;
    }
    used_end = s_storage->result_head;
    if (s_storage->active || s_storage->begin_pending ||
        s_storage->finalize_pending)
    {
        used_end = storage_align_up_sector(
                       s_storage->active_begin + OFFLINE_V2_PAGE_BYTES +
                       (s_storage->active_event_count *
                        ZY100_OFFLINE_V2_EVENT_SLOT_BYTES)) +
                   OFFLINE_V2_SECTOR_BYTES;
    }
    if (used_end >= ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE)
    {
        return 0U;
    }
    free_bytes = ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE - used_end;
    if (free_bytes <= OFFLINE_V2_FINAL_RESERVE_BYTES)
    {
        return 0U;
    }
    free_bytes -= OFFLINE_V2_FINAL_RESERVE_BYTES;
    return (uint8_t)((free_bytes * 100UL) / usable);
}

bool zy100_offline_v2_storage_begin_session(uint64_t start_unix_ms,
                                             bool timebase_synced,
                                             uint32_t owner_user_id)
{
    gd25q32e_jedec_id_t jedec_id;
    gd25q32e_status_regs_t status_regs;
    imu_status_t flash_status;

    if ((owner_user_id == 0U) ||
        s_storage == NULL || s_storage->next_session_id == 0xFFFFFFFFUL ||
        s_storage->next_session_generation == 0xFFFFFFFFUL ||
        !zy100_offline_v2_storage_ready_for_capture() ||
        (s_storage->session_count >= OFFLINE_V2_SESSION_MAX) ||
        !storage_range_valid(s_storage->result_head,
                             ZY100_OFFLINE_V2_MIN_ERASED_POOL_BYTES +
                             OFFLINE_V2_FINAL_RESERVE_BYTES,
                             ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE))
    {
        return false;
    }

    flash_status = gd25q32e_resume_and_verify(&jedec_id, &status_regs);
    if (flash_status != IMU_STATUS_OK)
    {
        ZY100_LOG_ERROR("[OFFLINE_V2][STORAGE][ERR] begin_preflight status=%u no_session_alloc=1 no_program=1",
                            (uint32_t)flash_status);
        storage_fail();
        return false;
    }

    s_storage->active_begin = storage_align_up_sector(s_storage->result_head);
    s_storage->active_session_id = s_storage->next_session_id++;
    if (s_storage->active_session_id == 0U)
    {
        s_storage->active_session_id = s_storage->next_session_id++;
    }
    s_storage->active_generation = s_storage->next_session_generation++;
    s_storage->active_owner_user_id = owner_user_id;
    s_storage->active_begin_version = ZY100_OFFLINE_V2_BEGIN_VERSION;
    s_storage->active_manifest_version = ZY100_OFFLINE_V2_MANIFEST_VERSION;
    s_storage->active_event_count = 0U;
    s_storage->active_first_center = 0xFFFFFFFFUL;
    s_storage->active_last_center = 0xFFFFFFFFUL;
    s_storage->active_stream_crc = zy100_crc32_ieee_begin();
    s_storage->active_dirty_span = 0U;
    s_storage->active_stop_reason = ZY100_OFFLINE_V2_STOP_USER;
    s_storage->active_health = ZY100_OFFLINE_V2_HEALTH_NORMAL;
    memset(&s_storage->active_quality, 0, sizeof(s_storage->active_quality));
    s_storage->active_clean = false;
    storage_build_begin(start_unix_ms, timebase_synced, owner_user_id);
    if (!storage_program_full_page(s_storage->active_begin, s_storage->page))
    {
        storage_fail();
        return false;
    }
    s_storage->begin_pending = true;
    s_storage->state = ZY100_OFFLINE_V2_STORAGE_BUSY;
    s_storage->job = STORAGE_JOB_BEGIN_WAIT;
    return true;
}

bool zy100_offline_v2_storage_session_active(void)
{
    return (s_storage != NULL) && s_storage->active;
}

uint32_t zy100_offline_v2_storage_active_owner(void)
{
    return s_storage != NULL ? s_storage->active_owner_user_id : 0U;
}

uint32_t zy100_offline_v2_storage_active_generation(void)
{
    return (s_storage != NULL && s_storage->active) ? s_storage->active_generation : 0U;
}

uint32_t zy100_offline_v2_storage_active_session_id(void)
{
    return (s_storage != NULL) ? s_storage->active_session_id : 0U;
}

uint32_t zy100_offline_v2_storage_active_event_count(void)
{
    return (s_storage != NULL) ? s_storage->active_event_count : 0U;
}

bool zy100_offline_v2_storage_append_event(
    const zy100_offline_v2_event_t *event)
{
    uint32_t slot;
    uint32_t finalize;

    if ((s_storage == NULL) || (event == NULL) || !s_storage->active ||
        (s_storage->job != STORAGE_JOB_NONE))
    {
        return false;
    }
    slot = s_storage->active_begin + OFFLINE_V2_PAGE_BYTES +
           (s_storage->active_event_count *
            ZY100_OFFLINE_V2_EVENT_SLOT_BYTES);
    finalize = storage_align_up_sector(slot +
        ZY100_OFFLINE_V2_EVENT_SLOT_BYTES);
    if (!storage_range_valid(slot, ZY100_OFFLINE_V2_EVENT_SLOT_BYTES,
                             ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE) ||
        !storage_range_valid(finalize, OFFLINE_V2_SECTOR_BYTES,
                             ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE))
    {
        s_storage->state = ZY100_OFFLINE_V2_STORAGE_LOCKED_5_PERCENT;
        return false;
    }
    storage_build_event(event);
    s_storage->pending_slot_addr = slot;
    if (!storage_program_full_page(slot, s_storage->page))
    {
        storage_fail();
        return false;
    }
    s_storage->job = STORAGE_JOB_EVENT_PAGE0_WAIT;
    return true;
}

bool zy100_offline_v2_storage_checkpoint(uint32_t now_ms)
{
    zy100_offline_v2_session_info_t info;
    bool due_by_bytes;
    bool due_by_time;

    if ((s_storage == NULL) || !s_storage->active ||
        (s_storage->job != STORAGE_JOB_NONE) ||
        (s_storage->active_event_count ==
         s_storage->last_checkpoint_event_count))
    {
        return false;
    }
    due_by_bytes =
        (s_storage->active_event_count -
         s_storage->last_checkpoint_event_count) >= 8U;
    due_by_time = (s_storage->last_checkpoint_ms == 0U) ||
                  ((uint32_t)(now_ms - s_storage->last_checkpoint_ms) >=
                   30000U);
    if (!due_by_bytes && !due_by_time)
    {
        return false;
    }
    info = storage_active_info(false);
    info.end_exclusive = s_storage->active_begin + OFFLINE_V2_PAGE_BYTES +
                         s_storage->active_event_count *
                         ZY100_OFFLINE_V2_EVENT_SLOT_BYTES;
    info.stream_crc32 = zy100_crc32_ieee_finish(
        s_storage->active_stream_crc);
    s_storage->pending_checkpoint_ms = now_ms;
    if (!storage_program_journal_entry(
            OFFLINE_V2_JOURNAL_TYPE_CHECKPOINT, &info,
            s_storage->active_event_count,
            STORAGE_JOB_CHECKPOINT_JOURNAL_WAIT))
    {
        storage_fail();
        return false;
    }
    return true;
}

bool zy100_offline_v2_storage_finalize_session(
    zy100_offline_v2_stop_reason_t reason,
    bool clean,
    uint32_t duration_ms,
    const zy100_offline_v2_quality_t *quality)
{
    if ((s_storage == NULL) || !s_storage->active ||
        (s_storage->job != STORAGE_JOB_NONE) ||
        (reason > ZY100_OFFLINE_V2_STOP_SYSTEM_SHUTDOWN))
    {
        return false;
    }
    s_storage->active_stop_reason = reason;
    s_storage->active_clean = clean;
    s_storage->active_health = storage_session_health(reason, clean);
    memset(&s_storage->active_quality, 0, sizeof(s_storage->active_quality));
    if (quality != NULL)
    {
        s_storage->active_quality = *quality;
    }
    storage_build_end(reason, clean, duration_ms, quality);
    if (!storage_program_full_page(s_storage->active_finalize_sector,
                                   s_storage->page))
    {
        storage_fail();
        return false;
    }
    s_storage->finalize_pending = true;
    s_storage->state = ZY100_OFFLINE_V2_STORAGE_BUSY;
    s_storage->job = STORAGE_JOB_END_WAIT;
    return true;
}

uint32_t zy100_offline_v2_storage_session_count(void)
{
    return (s_storage != NULL) ? s_storage->session_count : 0U;
}

uint32_t zy100_offline_v2_storage_session_count_for_user(uint32_t user_id)
{
    uint32_t index;
    uint32_t count = 0U;

    if ((s_storage == NULL) || (user_id == 0U))
    {
        return 0U;
    }
    for (index = 0U; index < s_storage->session_count; index++)
    {
        if (s_storage->session[index].owner_user_id == user_id)
        {
            count++;
        }
    }
    return count;
}

uint32_t zy100_offline_v2_storage_foreign_session_count(uint32_t user_id)
{
    uint32_t owned;

    if ((s_storage == NULL) || (user_id == 0U))
    {
        return 0U;
    }
    owned = zy100_offline_v2_storage_session_count_for_user(user_id);
    return (owned <= s_storage->session_count) ?
           (s_storage->session_count - owned) : 0U;
}

bool zy100_offline_v2_storage_session_get(
    uint32_t oldest_index,
    zy100_offline_v2_session_info_t *info_out)
{
    if ((s_storage == NULL) || (info_out == NULL) ||
        (oldest_index >= s_storage->session_count))
    {
        return false;
    }
    *info_out = s_storage->session[oldest_index];
    return true;
}

static bool storage_program_foreign_purge_marker(
    uint8_t type,
    offline_v2_storage_job_t resume_wait_job)
{
    zy100_offline_v2_session_info_t marker_info;

    memset(&marker_info, 0, sizeof(marker_info));
    marker_info.begin_addr = s_storage->foreign_purge_total_sectors;
    marker_info.end_exclusive = s_storage->foreign_purge_erased_sectors;
    marker_info.owner_user_id = s_storage->foreign_purge_protected_user_id;
    return storage_program_journal_entry(
        type, &marker_info, s_storage->foreign_purge_batch_token,
        resume_wait_job);
}

bool zy100_offline_v2_storage_session_get_for_user(
    uint32_t user_id,
    uint32_t oldest_visible_index,
    zy100_offline_v2_session_info_t *info_out)
{
    uint32_t index;
    uint32_t visible = 0U;

    if ((s_storage == NULL) || (user_id == 0U) || (info_out == NULL))
    {
        return false;
    }
    for (index = 0U; index < s_storage->session_count; index++)
    {
        if (s_storage->session[index].owner_user_id != user_id)
        {
            continue;
        }
        if (visible == oldest_visible_index)
        {
            *info_out = s_storage->session[index];
            return true;
        }
        visible++;
    }
    return false;
}

bool zy100_offline_v2_storage_read(uint32_t address,
                                  uint8_t *data,
                                  uint16_t length)
{
    return (data != NULL) &&
           storage_range_valid(address, length,
                               ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE) &&
           (gd25q32e_read(address, data, length) == IMU_STATUS_OK);
}

bool zy100_offline_v2_storage_confirm_session(uint32_t session_id,
                                              uint32_t generation,
                                              uint32_t owner_user_id,
                                              uint32_t transfer_id,
                                              uint32_t logical_bytes,
                                              uint32_t stream_crc32)
{
    zy100_offline_v2_session_info_t *info = NULL;
    uint32_t index;

    if ((s_storage == NULL) || (s_storage->job != STORAGE_JOB_NONE) ||
        s_storage->active || (s_storage->session_count == 0U) ||
        (owner_user_id == 0U))
    {
        return false;
    }
    for (index = 0U; index < s_storage->session_count; index++)
    {
        if ((s_storage->session[index].session_id == session_id) &&
            (s_storage->session[index].generation == generation))
        {
            info = &s_storage->session[index];
            break;
        }
    }
    if ((info == NULL) || (info->owner_user_id != owner_user_id) ||
        info->confirmed || (info->logical_bytes != logical_bytes) ||
        (info->stream_crc32 != stream_crc32))
    {
        return false;
    }
    info->reclaim_next = info->begin_addr;
    info->confirm_transfer_id = transfer_id;
    s_storage->confirm_transfer_id = transfer_id;
    s_storage->reclaim_from_foreign_purge = false;
    s_storage->reclaim_session_index = index;
    if (!storage_program_journal_entry(
            OFFLINE_V2_JOURNAL_TYPE_CONFIRMED, info, transfer_id,
            STORAGE_JOB_CONFIRM_JOURNAL_WAIT))
    {
        storage_fail();
        return false;
    }
    return true;
}

bool zy100_offline_v2_storage_reclaim_status(uint32_t session_id,
                                             uint32_t generation,
                                             uint32_t owner_user_id,
                                             uint8_t *state_out,
                                             uint32_t *erased_bytes_out,
                                             uint32_t *extent_bytes_out)
{
    uint32_t index;

    if ((s_storage == NULL) || (owner_user_id == 0U) ||
        (state_out == NULL) ||
        (erased_bytes_out == NULL) || (extent_bytes_out == NULL))
    {
        return false;
    }
    *state_out = ZY100_OFFLINE_V2_RECLAIM_UNKNOWN;
    *erased_bytes_out = 0U;
    *extent_bytes_out = 0U;
    if (s_storage->journal_pending_reclaim &&
        (s_storage->reclaim_info.session_id == session_id) &&
        (s_storage->reclaim_info.generation == generation) &&
        (s_storage->reclaim_info.owner_user_id == owner_user_id))
    {
        *state_out = ZY100_OFFLINE_V2_RECLAIM_RECLAIMING;
        *extent_bytes_out = s_storage->reclaim_info.end_exclusive -
                            s_storage->reclaim_info.begin_addr;
        if (s_storage->reclaim_addr > s_storage->reclaim_info.begin_addr)
        {
            *erased_bytes_out = s_storage->reclaim_addr -
                                s_storage->reclaim_info.begin_addr;
        }
        if (*erased_bytes_out > *extent_bytes_out)
        {
            *erased_bytes_out = *extent_bytes_out;
        }
        return true;
    }
    if (s_storage->tombstone_valid &&
        (s_storage->tombstone_info.session_id == session_id) &&
        (s_storage->tombstone_info.generation == generation) &&
        (s_storage->tombstone_info.owner_user_id == owner_user_id))
    {
        *state_out = ZY100_OFFLINE_V2_RECLAIM_TOMBSTONE;
        *extent_bytes_out = s_storage->tombstone_info.end_exclusive -
                            s_storage->tombstone_info.begin_addr;
        *erased_bytes_out = *extent_bytes_out;
        return true;
    }
    for (index = 0U; index < s_storage->session_count; index++)
    {
        if ((s_storage->session[index].session_id == session_id) &&
            (s_storage->session[index].generation == generation) &&
            (s_storage->session[index].owner_user_id == owner_user_id))
        {
            if (s_storage->session[index].confirmed &&
                s_storage->session[index].reclaim_next >= s_storage->session[index].begin_addr)
                *erased_bytes_out = s_storage->session[index].reclaim_next - s_storage->session[index].begin_addr;
            *state_out = s_storage->session[index].confirmed ?
                ZY100_OFFLINE_V2_RECLAIM_PAUSED :
                ZY100_OFFLINE_V2_RECLAIM_FINAL;
            *extent_bytes_out =
                s_storage->session[index].end_exclusive -
                s_storage->session[index].begin_addr;
            return true;
        }
    }
    return true;
}

bool zy100_offline_v2_storage_confirm_replay_matches(
    uint32_t session_id,
    uint32_t generation,
    uint32_t owner_user_id,
    uint32_t transfer_id,
    uint32_t stream_crc32)
{
    if ((s_storage == NULL) || (owner_user_id == 0U))
    {
        return false;
    }
    {
        uint32_t i = storage_find_identity(session_id, generation);
        if (i < s_storage->session_count && s_storage->session[i].confirmed)
            return s_storage->session[i].owner_user_id == owner_user_id &&
                s_storage->session[i].confirm_transfer_id == transfer_id &&
                s_storage->session[i].stream_crc32 == stream_crc32;
    }
    return s_storage->tombstone_valid &&
           (s_storage->tombstone_info.session_id == session_id) &&
           (s_storage->tombstone_info.generation == generation) &&
           (s_storage->tombstone_info.owner_user_id == owner_user_id) &&
           (s_storage->tombstone_transfer_id == transfer_id) &&
           (s_storage->tombstone_info.stream_crc32 == stream_crc32);
}

/* Read hardware, not a logical spool/legacy busy latch. Never clear WIP here. */
zy100_offline_v2_hw_status_t zy100_offline_v2_storage_hardware_status(void)
{
    bool busy = true;
    if (spi_bus_current_owner() != SPI_OWNER_NONE) return ZY100_OFFLINE_V2_HW_BUSY;
    if (gd25q32e_is_busy(&busy) != IMU_STATUS_OK)
        return ZY100_OFFLINE_V2_HW_UNKNOWN;
    return busy ? ZY100_OFFLINE_V2_HW_BUSY : ZY100_OFFLINE_V2_HW_IDLE;
}

zy100_offline_v2_drain_status_t zy100_offline_v2_storage_drain_status(void)
{
    if (s_storage == NULL) return ZY100_OFFLINE_V2_DRAIN_DONE;
    if (s_storage->state == ZY100_OFFLINE_V2_STORAGE_ERROR)
        return ZY100_OFFLINE_V2_DRAIN_ERROR;
    if (s_storage->job != STORAGE_JOB_NONE || s_storage->active ||
        s_storage->begin_pending || s_storage->finalize_pending ||
        s_storage->foreign_purge_active || s_storage->journal_pending_reclaim ||
        s_storage->state == ZY100_OFFLINE_V2_STORAGE_RECOVERING ||
        s_storage->state == ZY100_OFFLINE_V2_STORAGE_CLEARING ||
        s_storage->state == ZY100_OFFLINE_V2_STORAGE_MIGRATING)
        return ZY100_OFFLINE_V2_DRAIN_BUSY;
    return ZY100_OFFLINE_V2_DRAIN_DONE;
}

bool zy100_offline_v2_storage_recover_begin(void)
{
    if (s_storage == NULL || s_storage->state != ZY100_OFFLINE_V2_STORAGE_ERROR ||
        zy100_offline_v2_storage_hardware_status() != ZY100_OFFLINE_V2_HW_IDLE)
        return false;
    return zy100_offline_v2_storage_init_with_policy((uint8_t *)s_storage,
        s_scratch_bytes, ZY100_OFFLINE_V2_BOOT_RECOVER_ONLY);
}

bool zy100_offline_v2_storage_recovery_anchored(void) { return s_recovery_anchor; }
bool zy100_offline_v2_storage_recovery_retryable(void) { return !s_recovery_rejected; }
uint8_t zy100_offline_v2_storage_failed_job(void) { return s_failed_job; }

uint32_t zy100_offline_v2_storage_directory_revision(void)
{
    return s_storage != NULL ? s_storage->directory_revision : 0U;
}

bool zy100_offline_v2_storage_latest_for_user(uint32_t user_id,
    zy100_offline_v2_session_info_t *info_out)
{
    uint32_t i, best = OFFLINE_V2_SESSION_INDEX_INVALID;
    if (s_storage == NULL || user_id == 0U || info_out == NULL) return false;
    for (i = 0U; i < s_storage->session_count; i++)
        if (s_storage->session[i].owner_user_id == user_id &&
            (best == OFFLINE_V2_SESSION_INDEX_INVALID ||
             s_storage->session[i].generation > s_storage->session[best].generation)) best = i;
    if (best == OFFLINE_V2_SESSION_INDEX_INVALID) return false;
    *info_out = s_storage->session[best];
    return true;
}

bool zy100_offline_v2_storage_preempt_capacity(void)
{
    return s_storage != NULL && !s_storage->active && !s_storage->begin_pending &&
        !s_storage->finalize_pending && !s_storage->foreign_purge_active &&
        (s_storage->state == ZY100_OFFLINE_V2_STORAGE_READY ||
         s_storage->state == ZY100_OFFLINE_V2_STORAGE_BUSY) &&
        s_storage->session_count < OFFLINE_V2_SESSION_MAX &&
        s_storage->next_session_id != 0xFFFFFFFFUL && s_storage->next_session_generation != 0xFFFFFFFFUL &&
        zy100_offline_v2_storage_remaining_percent() > OFFLINE_V2_ERASED_PERCENT_LOCK &&
        storage_range_valid(s_storage->result_head, ZY100_OFFLINE_V2_MIN_ERASED_POOL_BYTES +
            OFFLINE_V2_FINAL_RESERVE_BYTES, ZY100_OFFLINE_V2_RESULT_END_EXCLUSIVE);
}

bool zy100_offline_v2_storage_pause_reclaim(void)
{
    if (!zy100_offline_v2_storage_preempt_capacity()) return false;
    s_storage->reclaim_paused = true;
    return true;
}

void zy100_offline_v2_storage_release_reclaim(void)
{
    if (s_storage != NULL) s_storage->reclaim_paused = false;
}

void zy100_offline_v2_storage_resume_reclaim(uint32_t user_id)
{
    zy100_offline_v2_session_info_t info;
    uint32_t index;
    if (s_storage == NULL || s_storage->job != STORAGE_JOB_NONE ||
        s_storage->reclaim_paused || s_storage->active || s_storage->begin_pending ||
        s_storage->finalize_pending || s_storage->foreign_purge_active ||
        (s_storage->state != ZY100_OFFLINE_V2_STORAGE_READY &&
         s_storage->state != ZY100_OFFLINE_V2_STORAGE_LOCKED_5_PERCENT) ||
        !zy100_offline_v2_storage_latest_for_user(user_id, &info) || !info.confirmed) return;
    index = storage_find_identity(info.session_id, info.generation);
    s_storage->confirm_transfer_id = info.confirm_transfer_id;
    s_storage->reclaim_from_foreign_purge = false;
    if (!storage_start_reclaim(&s_storage->session[index], index)) storage_fail();
}

void zy100_offline_v2_storage_require_restart(uint32_t id, uint32_t generation, bool required)
{
    uint32_t i;
    if (s_storage == NULL) return;
    i = storage_find_identity(id, generation);
    if (i < s_storage->session_count && !s_storage->session[i].confirmed)
        s_storage->session[i].restart_from_zero = required;
}
