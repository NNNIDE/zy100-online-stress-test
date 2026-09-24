#include "zy100_system_info_store.h"

#include <stddef.h>
#include <string.h>

#include "trace.h"
#include "version.h"

#include "../common/zy100_internal_flash_layout.h"
#include "../driver/drv_internal_flash.h"
#include "zy100_crc32.h"

#define ZY100_SYS_INFO_U32(a, b, c, d) \
    (((uint32_t)(a)) | ((uint32_t)(b) << 8) | \
     ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))

#define ZY100_SYS_INFO_MAGIC         ZY100_SYS_INFO_U32('S', 'I', 'N', 'F')
#define ZY100_SYS_INFO_VERSION_V1    1U
#define ZY100_SYS_INFO_VERSION_V2    2U
#define ZY100_SYS_INFO_VERSION_V3    3U
#define ZY100_SYS_INFO_VERSION_V4    4U
#define ZY100_SYS_INFO_VERSION_V5    5U
#define ZY100_SYS_INFO_VERSION_V6    6U
#define ZY100_SYS_INFO_VERSION       7U
#define ZY100_SYS_INFO_RECORD_BYTES  256U
#define ZY100_SYS_INFO_LEGACY_PREFIX_BYTES 40U
#define ZY100_SYS_INFO_PEER_RECORD_BYTES 8U
#define ZY100_SYS_INFO_VERSION_CODE_FIELD_BYTES 4U
#define ZY100_SYS_INFO_USER_ID_FIELD_BYTES 4U
#define ZY100_SYS_INFO_OTA_TIME_FIELD_BYTES 48U
#define ZY100_SYS_INFO_FLAGS_ERASED 0xFFU
#define ZY100_SYS_INFO_FLAG_OTA_SUCCESS_LED_PENDING_N 0x01U
#define ZY100_SYS_INFO_FACTORY_ACCEPTANCE_OFFSET       0U
#define ZY100_SYS_INFO_WHOLE_UNIT_OFFSET               1U
#define ZY100_SYS_INFO_PRODUCTION_SHIPPING_OFFSET      2U
#define ZY100_BDADDR_RAW_FMT "%02X:%02X:%02X:%02X:%02X:%02X"
#define ZY100_BDADDR_RAW_ARG(a) \
    (unsigned int)((a)[0]), \
    (unsigned int)((a)[1]), \
    (unsigned int)((a)[2]), \
    (unsigned int)((a)[3]), \
    (unsigned int)((a)[4]), \
    (unsigned int)((a)[5])
#define ZY100_SYS_INFO_PEER_RECORD_AREA_BYTES \
    (ZY100_SYSTEM_INFO_MAX_PAIRED_PEERS * ZY100_SYS_INFO_PEER_RECORD_BYTES)
#define ZY100_SYS_INFO_PEER_COUNT_FIELD_BYTES 4U
#define ZY100_SYS_INFO_RESERVED_BYTES \
    (ZY100_SYS_INFO_RECORD_BYTES - ZY100_SYS_INFO_LEGACY_PREFIX_BYTES - \
     ZY100_SYS_INFO_PEER_COUNT_FIELD_BYTES - \
     ZY100_SYS_INFO_PEER_RECORD_AREA_BYTES - \
     ZY100_SYS_INFO_VERSION_CODE_FIELD_BYTES - \
     ZY100_SYS_INFO_USER_ID_FIELD_BYTES - \
     ZY100_SYS_INFO_OTA_TIME_FIELD_BYTES)

typedef enum
{
    ZY100_SYS_INFO_SLOT_NONE = 0U,
    ZY100_SYS_INFO_SLOT_PRIMARY,
    ZY100_SYS_INFO_SLOT_BACKUP,
} zy100_sys_info_slot_t;

typedef struct
{
    uint8_t peer_addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES];
    uint8_t peer_addr_type;
    uint8_t valid;
} zy100_sys_info_peer_record_t;

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t record_size;
    uint32_t sequence;
    uint32_t crc32;

    uint8_t ble_addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES];
    uint8_t ble_addr_type;
    uint8_t ble_addr_valid;

    uint8_t pairing_summary_valid;
    uint8_t pairing_bonded;
    uint8_t last_peer_addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES];
    uint8_t last_peer_addr_type;
    uint8_t last_peer_addr_valid;

    uint8_t first_power_seen;
    uint8_t reserved_flags;

    uint8_t software_version_major;
    uint8_t software_version_minor;
    uint8_t software_version_revision;
    uint8_t software_version_buildnum;

    uint8_t paired_peer_count;
    uint8_t paired_peer_reserved[3];
    zy100_sys_info_peer_record_t paired_peers[ZY100_SYSTEM_INFO_MAX_PAIRED_PEERS];

    uint32_t software_version_code;
    uint32_t latest_user_id;
    uint8_t ota_time_state;
    uint8_t ota_time_reserved[3];
    uint64_t ota_checkpoint_unix_ms;
    uint64_t ota_checkpoint_rtc_ticks;
    uint64_t ota_checkpoint_rtc_wrap_ticks;
    uint64_t ota_resume_unix_ms;
    uint32_t ota_checkpoint_user_id;
    uint32_t ota_checkpoint_tick_hz;
    uint32_t ota_source_version_code;
    uint8_t reserved[ZY100_SYS_INFO_RESERVED_BYTES];
} zy100_sys_info_record_t;

typedef char zy100_sys_info_record_size_check[
    (sizeof(zy100_sys_info_record_t) == ZY100_SYS_INFO_RECORD_BYTES) ? 1 : -1];
typedef char zy100_sys_info_peer_record_size_check[
    (sizeof(zy100_sys_info_peer_record_t) == ZY100_SYS_INFO_PEER_RECORD_BYTES) ? 1 : -1];
typedef char zy100_sys_info_slot_size_check[
    (ZY100_SYS_INFO_RECORD_BYTES <= ZY100_SYSTEM_INFO_SLOT_BYTES) ? 1 : -1];

static void zy100_system_info_fill_defaults(zy100_system_info_t *info)
{
    if (info == NULL)
    {
        return;
    }

    memset(info, 0, sizeof(*info));
    info->software_version_major = (uint8_t)VERSION_MAJOR;
    info->software_version_minor = (uint8_t)VERSION_MINOR;
    info->software_version_revision = (uint8_t)VERSION_REVISION;
    info->software_version_buildnum = (uint8_t)VERSION_BUILDNUM;
    info->software_version_code = (uint32_t)VERSION_CODE;
    info->whole_unit_state = (uint8_t)ZY100_WHOLE_UNIT_NOT_REQUIRED;
    info->production_shipping_state =
        (uint8_t)ZY100_PRODUCTION_SHIPPING_NONE;
}

static uint8_t zy100_system_info_bool_byte(uint8_t value)
{
    return (value != 0U) ? 1U : 0U;
}

static bool zy100_system_info_acceptance_state_valid(uint8_t state)
{
    return (state == (uint8_t)ZY100_FACTORY_ACCEPTANCE_NONE) ||
           (state == (uint8_t)ZY100_FACTORY_ACCEPTANCE_PENDING) ||
           (state == (uint8_t)ZY100_FACTORY_ACCEPTANCE_SHIP_ARMED);
}

static bool zy100_system_info_whole_unit_state_valid(uint8_t state)
{
    return state <= (uint8_t)ZY100_WHOLE_UNIT_COMPLETE;
}

static bool zy100_system_info_production_shipping_state_valid(uint8_t state)
{
    return (state == (uint8_t)ZY100_PRODUCTION_SHIPPING_NONE) ||
           (state == (uint8_t)ZY100_PRODUCTION_SHIPPING_ARMED);
}

static void zy100_system_info_ota_time_clear(
    zy100_system_info_ota_time_t *ota_time)
{
    if (ota_time == NULL)
    {
        return;
    }
    memset(ota_time, 0, sizeof(*ota_time));
    ota_time->state = (uint8_t)ZY100_OTA_TIME_STATE_NONE;
}

static bool zy100_system_info_ota_time_fields_valid(
    const zy100_system_info_ota_time_t *ota_time)
{
    if (ota_time == NULL)
    {
        return false;
    }
    return (ota_time->checkpoint_unix_ms != 0ULL) &&
           (ota_time->checkpoint_rtc_wrap_ticks != 0ULL) &&
           (ota_time->checkpoint_rtc_ticks <
            ota_time->checkpoint_rtc_wrap_ticks) &&
           (ota_time->checkpoint_user_id != 0U) &&
           (ota_time->checkpoint_tick_hz != 0U) &&
           (ota_time->source_version_code != 0U);
}

static bool zy100_system_info_peer_match(
    const zy100_system_info_paired_peer_t *peer,
    const uint8_t peer_addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES],
    uint8_t peer_type)
{
    if ((peer == NULL) || (peer_addr == NULL) ||
        (zy100_system_info_bool_byte(peer->valid) == 0U))
    {
        return false;
    }
    return (peer->peer_addr_type == peer_type) &&
           (memcmp(peer->peer_addr,
                   peer_addr,
                   ZY100_SYSTEM_INFO_BD_ADDR_BYTES) == 0);
}

static bool zy100_system_info_last_peer_matches_slot(
    const zy100_system_info_t *info)
{
    uint8_t idx;

    if ((info == NULL) ||
        (zy100_system_info_bool_byte(info->last_peer_addr_valid) == 0U))
    {
        return false;
    }

    for (idx = 0U; idx < ZY100_SYSTEM_INFO_MAX_PAIRED_PEERS; idx++)
    {
        if (zy100_system_info_peer_match(&info->paired_peers[idx],
                                         info->last_peer_addr,
                                         info->last_peer_addr_type))
        {
            return true;
        }
    }
    return false;
}

static uint8_t zy100_system_info_count_paired_peers(
    zy100_system_info_t *info,
    uint8_t *first_valid_index)
{
    uint8_t idx;
    uint8_t count = 0U;

    if (first_valid_index != NULL)
    {
        *first_valid_index = ZY100_SYSTEM_INFO_MAX_PAIRED_PEERS;
    }
    if (info == NULL)
    {
        return 0U;
    }

    for (idx = 0U; idx < ZY100_SYSTEM_INFO_MAX_PAIRED_PEERS; idx++)
    {
        info->paired_peers[idx].valid =
            zy100_system_info_bool_byte(info->paired_peers[idx].valid);
        if (info->paired_peers[idx].valid != 0U)
        {
            if ((count == 0U) && (first_valid_index != NULL))
            {
                *first_valid_index = idx;
            }
            count++;
        }
        else
        {
            memset(info->paired_peers[idx].peer_addr,
                   0,
                   sizeof(info->paired_peers[idx].peer_addr));
            info->paired_peers[idx].peer_addr_type = 0U;
        }
    }

    return count;
}

static void zy100_system_info_set_last_peer_from_slot(
    zy100_system_info_t *info,
    uint8_t index)
{
    if ((info == NULL) || (index >= ZY100_SYSTEM_INFO_MAX_PAIRED_PEERS) ||
        (info->paired_peers[index].valid == 0U))
    {
        return;
    }

    memcpy(info->last_peer_addr,
           info->paired_peers[index].peer_addr,
           sizeof(info->last_peer_addr));
    info->last_peer_addr_type = info->paired_peers[index].peer_addr_type;
    info->last_peer_addr_valid = 1U;
}

static void zy100_system_info_normalize(zy100_system_info_t *info)
{
    uint8_t first_valid_index;

    if (info == NULL)
    {
        return;
    }

    info->ble_addr_valid = zy100_system_info_bool_byte(info->ble_addr_valid);
    info->pairing_summary_valid =
        zy100_system_info_bool_byte(info->pairing_summary_valid);
    info->pairing_bonded = zy100_system_info_bool_byte(info->pairing_bonded);
    info->last_peer_addr_valid =
        zy100_system_info_bool_byte(info->last_peer_addr_valid);
    info->first_power_seen = zy100_system_info_bool_byte(info->first_power_seen);
    info->ota_success_led_pending =
        zy100_system_info_bool_byte(info->ota_success_led_pending);
    if (!zy100_system_info_acceptance_state_valid(
            info->factory_acceptance_state))
    {
        info->factory_acceptance_state =
            (uint8_t)ZY100_FACTORY_ACCEPTANCE_NONE;
    }
    if (!zy100_system_info_whole_unit_state_valid(info->whole_unit_state))
    {
        info->whole_unit_state = (uint8_t)ZY100_WHOLE_UNIT_NOT_REQUIRED;
    }
    if (!zy100_system_info_production_shipping_state_valid(
            info->production_shipping_state))
    {
        info->production_shipping_state =
            (uint8_t)ZY100_PRODUCTION_SHIPPING_NONE;
    }

    if ((info->ota_time.state != (uint8_t)ZY100_OTA_TIME_STATE_ARMED) &&
        (info->ota_time.state !=
         (uint8_t)ZY100_OTA_TIME_STATE_RESUME_PENDING))
    {
        zy100_system_info_ota_time_clear(&info->ota_time);
    }
    else if (!zy100_system_info_ota_time_fields_valid(&info->ota_time) ||
             ((info->ota_time.state ==
               (uint8_t)ZY100_OTA_TIME_STATE_ARMED) &&
              (info->ota_time.resume_unix_ms != 0ULL)) ||
             ((info->ota_time.state ==
               (uint8_t)ZY100_OTA_TIME_STATE_RESUME_PENDING) &&
              (info->ota_time.resume_unix_ms == 0ULL)))
    {
        zy100_system_info_ota_time_clear(&info->ota_time);
    }

    info->paired_peer_count =
        zy100_system_info_count_paired_peers(info, &first_valid_index);
    if (info->paired_peer_count != 0U)
    {
        info->pairing_summary_valid = 1U;
        info->pairing_bonded = 1U;
        if (!zy100_system_info_last_peer_matches_slot(info))
        {
            zy100_system_info_set_last_peer_from_slot(info, first_valid_index);
        }
    }
    else
    {
        info->pairing_summary_valid = 0U;
        info->pairing_bonded = 0U;
        memset(info->last_peer_addr, 0, sizeof(info->last_peer_addr));
        info->last_peer_addr_type = 0U;
        info->last_peer_addr_valid = 0U;
    }

    info->software_version_major = (uint8_t)VERSION_MAJOR;
    info->software_version_minor = (uint8_t)VERSION_MINOR;
    info->software_version_revision = (uint8_t)VERSION_REVISION;
    info->software_version_buildnum = (uint8_t)VERSION_BUILDNUM;
    info->software_version_code = (uint32_t)VERSION_CODE;
}

static bool zy100_system_info_record_current_software_version(
    const zy100_sys_info_record_t *record)
{
    if (record == NULL)
    {
        return false;
    }

    return ((record->version == ZY100_SYS_INFO_VERSION) ||
         (record->version == ZY100_SYS_INFO_VERSION_V6)) &&
           (record->software_version_major == (uint8_t)VERSION_MAJOR) &&
           (record->software_version_minor == (uint8_t)VERSION_MINOR) &&
           (record->software_version_revision == (uint8_t)VERSION_REVISION) &&
           (record->software_version_buildnum == (uint8_t)VERSION_BUILDNUM) &&
           (record->software_version_code == (uint32_t)VERSION_CODE);
}

static uint32_t zy100_system_info_slot_offset(zy100_sys_info_slot_t slot)
{
    if (slot == ZY100_SYS_INFO_SLOT_BACKUP)
    {
        return ZY100_SYSTEM_INFO_BACKUP_OFFSET;
    }
    return ZY100_SYSTEM_INFO_PRIMARY_OFFSET;
}

static uint32_t zy100_system_info_record_crc(
    const zy100_sys_info_record_t *record)
{
    uint32_t crc;
    uint32_t zero_crc_field = 0U;
    const uint8_t *bytes = (const uint8_t *)record;
    const uint32_t crc_offset =
        (uint32_t)offsetof(zy100_sys_info_record_t, crc32);
    const uint32_t after_crc_offset =
        crc_offset + (uint32_t)sizeof(record->crc32);

    if (record == NULL)
    {
        return 0U;
    }

    crc = zy100_crc32_ieee_begin();
    crc = zy100_crc32_ieee_update(crc, bytes, crc_offset);
    crc = zy100_crc32_ieee_update(crc,
                                  (const uint8_t *)&zero_crc_field,
                                  (uint32_t)sizeof(zero_crc_field));
    crc = zy100_crc32_ieee_update(crc,
                                  &bytes[after_crc_offset],
                                  (uint32_t)sizeof(*record) - after_crc_offset);
    return zy100_crc32_ieee_finish(crc);
}

static bool zy100_system_info_record_valid(
    const zy100_sys_info_record_t *record)
{
    if (record == NULL)
    {
        return false;
    }
    if ((record->magic != ZY100_SYS_INFO_MAGIC) ||
        ((record->version != ZY100_SYS_INFO_VERSION) &&
         (record->version != ZY100_SYS_INFO_VERSION_V6) &&
         (record->version != ZY100_SYS_INFO_VERSION_V5) &&
         (record->version != ZY100_SYS_INFO_VERSION_V4) &&
         (record->version != ZY100_SYS_INFO_VERSION_V3) &&
         (record->version != ZY100_SYS_INFO_VERSION_V2) &&
         (record->version != ZY100_SYS_INFO_VERSION_V1)) ||
        (record->record_size != ZY100_SYS_INFO_RECORD_BYTES))
    {
        return false;
    }
    return record->crc32 == zy100_system_info_record_crc(record);
}

static bool zy100_system_info_read_slot(zy100_sys_info_slot_t slot,
                                        zy100_sys_info_record_t *record,
                                        bool *valid)
{
    drv_internal_flash_status_t status;
    uint32_t offset;

    if ((record == NULL) || (valid == NULL) ||
        (slot == ZY100_SYS_INFO_SLOT_NONE))
    {
        return false;
    }

    *valid = false;
    offset = zy100_system_info_slot_offset(slot);
    status = drv_internal_flash_read(offset,
                                     (uint8_t *)record,
                                     (uint32_t)sizeof(*record));
    if (status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        DBG_DIRECT("[SYS_INFO] read_fail slot=%u off=0x%08lX status=%s",
                   (unsigned int)slot,
                   (unsigned long)offset,
                   drv_internal_flash_status_name(status));
        return false;
    }

    *valid = zy100_system_info_record_valid(record);
    return true;
}

static bool zy100_system_info_record_newer(uint32_t candidate,
                                           uint32_t current)
{
    return ((int32_t)(candidate - current)) > 0;
}

static bool zy100_system_info_find_latest(zy100_sys_info_record_t *record,
                                          zy100_sys_info_slot_t *slot,
                                          bool *found)
{
    zy100_sys_info_record_t primary;
    zy100_sys_info_record_t backup;
    bool primary_valid;
    bool backup_valid;

    if ((record == NULL) || (slot == NULL) || (found == NULL))
    {
        return false;
    }

    *slot = ZY100_SYS_INFO_SLOT_NONE;
    *found = false;

    if (!zy100_system_info_read_slot(ZY100_SYS_INFO_SLOT_PRIMARY,
                                     &primary,
                                     &primary_valid) ||
        !zy100_system_info_read_slot(ZY100_SYS_INFO_SLOT_BACKUP,
                                     &backup,
                                     &backup_valid))
    {
        return false;
    }

    if (primary_valid)
    {
        *record = primary;
        *slot = ZY100_SYS_INFO_SLOT_PRIMARY;
        *found = true;
    }
    if (backup_valid &&
        (!*found ||
         zy100_system_info_record_newer(backup.sequence, record->sequence)))
    {
        *record = backup;
        *slot = ZY100_SYS_INFO_SLOT_BACKUP;
        *found = true;
    }

    return true;
}

static void zy100_system_info_from_record(
    const zy100_sys_info_record_t *record,
    zy100_system_info_t *info)
{
    uint8_t idx;

    if ((record == NULL) || (info == NULL))
    {
        return;
    }

    memcpy(info->ble_addr, record->ble_addr, sizeof(info->ble_addr));
    info->ble_addr_type = record->ble_addr_type;
    info->ble_addr_valid = zy100_system_info_bool_byte(record->ble_addr_valid);

    info->pairing_summary_valid =
        zy100_system_info_bool_byte(record->pairing_summary_valid);
    info->pairing_bonded = zy100_system_info_bool_byte(record->pairing_bonded);
    memcpy(info->last_peer_addr,
           record->last_peer_addr,
           sizeof(info->last_peer_addr));
    info->last_peer_addr_type = record->last_peer_addr_type;
    info->last_peer_addr_valid =
        zy100_system_info_bool_byte(record->last_peer_addr_valid);

    info->paired_peer_count = 0U;
    memset(info->paired_peers, 0, sizeof(info->paired_peers));
    if (((record->version == ZY100_SYS_INFO_VERSION) ||
         (record->version == ZY100_SYS_INFO_VERSION_V6)) ||
        (record->version == ZY100_SYS_INFO_VERSION_V5) ||
        (record->version == ZY100_SYS_INFO_VERSION_V4) ||
        (record->version == ZY100_SYS_INFO_VERSION_V3) ||
        (record->version == ZY100_SYS_INFO_VERSION_V2))
    {
        for (idx = 0U; idx < ZY100_SYSTEM_INFO_MAX_PAIRED_PEERS; idx++)
        {
            memcpy(info->paired_peers[idx].peer_addr,
                   record->paired_peers[idx].peer_addr,
                   sizeof(info->paired_peers[idx].peer_addr));
            info->paired_peers[idx].peer_addr_type =
                record->paired_peers[idx].peer_addr_type;
            info->paired_peers[idx].valid =
                zy100_system_info_bool_byte(record->paired_peers[idx].valid);
        }
    }
    else if ((record->version == ZY100_SYS_INFO_VERSION_V1) &&
             (info->last_peer_addr_valid != 0U))
    {
        memcpy(info->paired_peers[0].peer_addr,
               info->last_peer_addr,
               sizeof(info->paired_peers[0].peer_addr));
        info->paired_peers[0].peer_addr_type = info->last_peer_addr_type;
        info->paired_peers[0].valid = 1U;
    }

    info->user_reset_state = (record->version == ZY100_SYS_INFO_VERSION) ?
                             record->reserved[3] : 0U;
    info->first_power_seen =
        zy100_system_info_bool_byte(record->first_power_seen);
    info->ota_success_led_pending =
        ((record->reserved_flags &
          ZY100_SYS_INFO_FLAG_OTA_SUCCESS_LED_PENDING_N) == 0U) ? 1U : 0U;
    info->factory_acceptance_state =
        record->reserved[ZY100_SYS_INFO_FACTORY_ACCEPTANCE_OFFSET];
    info->whole_unit_state =
        ((record->version == ZY100_SYS_INFO_VERSION) ||
         (record->version == ZY100_SYS_INFO_VERSION_V6)) ?
        record->reserved[ZY100_SYS_INFO_WHOLE_UNIT_OFFSET] :
        (uint8_t)ZY100_WHOLE_UNIT_NOT_REQUIRED;
    info->production_shipping_state =
        ((record->version == ZY100_SYS_INFO_VERSION) ||
         (record->version == ZY100_SYS_INFO_VERSION_V6)) ?
        record->reserved[ZY100_SYS_INFO_PRODUCTION_SHIPPING_OFFSET] :
        (uint8_t)ZY100_PRODUCTION_SHIPPING_NONE;
    info->software_version_major = record->software_version_major;
    info->software_version_minor = record->software_version_minor;
    info->software_version_revision = record->software_version_revision;
    info->software_version_buildnum = record->software_version_buildnum;
    if (((record->version == ZY100_SYS_INFO_VERSION) ||
         (record->version == ZY100_SYS_INFO_VERSION_V6)) ||
        (record->version == ZY100_SYS_INFO_VERSION_V5) ||
        (record->version == ZY100_SYS_INFO_VERSION_V4) ||
        (record->version == ZY100_SYS_INFO_VERSION_V3))
    {
        info->software_version_code = record->software_version_code;
    }
    else
    {
        info->software_version_code = (uint32_t)VERSION_CODE;
    }
    info->latest_user_id =
        (((record->version == ZY100_SYS_INFO_VERSION) ||
         (record->version == ZY100_SYS_INFO_VERSION_V6)) ||
         (record->version == ZY100_SYS_INFO_VERSION_V5) ||
         (record->version == ZY100_SYS_INFO_VERSION_V4)) ?
        record->latest_user_id : 0U;
    zy100_system_info_ota_time_clear(&info->ota_time);
    if (((record->version == ZY100_SYS_INFO_VERSION) ||
         (record->version == ZY100_SYS_INFO_VERSION_V6)) ||
        (record->version == ZY100_SYS_INFO_VERSION_V5))
    {
        info->ota_time.state = record->ota_time_state;
        info->ota_time.checkpoint_unix_ms =
            record->ota_checkpoint_unix_ms;
        info->ota_time.checkpoint_rtc_ticks =
            record->ota_checkpoint_rtc_ticks;
        info->ota_time.checkpoint_rtc_wrap_ticks =
            record->ota_checkpoint_rtc_wrap_ticks;
        info->ota_time.resume_unix_ms = record->ota_resume_unix_ms;
        info->ota_time.checkpoint_user_id =
            record->ota_checkpoint_user_id;
        info->ota_time.checkpoint_tick_hz =
            record->ota_checkpoint_tick_hz;
        info->ota_time.source_version_code =
            record->ota_source_version_code;
    }
    zy100_system_info_normalize(info);
}

static void zy100_system_info_to_record(
    const zy100_system_info_t *info,
    uint32_t sequence,
    zy100_sys_info_record_t *record)
{
    uint8_t idx;

    if ((info == NULL) || (record == NULL))
    {
        return;
    }

    memset(record, 0xFF, sizeof(*record));
    record->magic = ZY100_SYS_INFO_MAGIC;
    record->version = ZY100_SYS_INFO_VERSION;
    record->record_size = ZY100_SYS_INFO_RECORD_BYTES;
    record->sequence = sequence;
    record->crc32 = 0U;

    memcpy(record->ble_addr, info->ble_addr, sizeof(record->ble_addr));
    record->ble_addr_type = info->ble_addr_type;
    record->ble_addr_valid = info->ble_addr_valid;

    record->pairing_summary_valid = info->pairing_summary_valid;
    record->pairing_bonded = info->pairing_bonded;
    memcpy(record->last_peer_addr,
           info->last_peer_addr,
           sizeof(record->last_peer_addr));
    record->last_peer_addr_type = info->last_peer_addr_type;
    record->last_peer_addr_valid = info->last_peer_addr_valid;

    record->first_power_seen = info->first_power_seen;
    record->reserved_flags = ZY100_SYS_INFO_FLAGS_ERASED;
    if (info->ota_success_led_pending != 0U)
    {
        record->reserved_flags =
            (uint8_t)(record->reserved_flags &
                      (uint8_t)(~ZY100_SYS_INFO_FLAG_OTA_SUCCESS_LED_PENDING_N));
    }
    record->software_version_major = info->software_version_major;
    record->software_version_minor = info->software_version_minor;
    record->software_version_revision = info->software_version_revision;
    record->software_version_buildnum = info->software_version_buildnum;

    record->paired_peer_count = info->paired_peer_count;
    memset(record->paired_peer_reserved,
           0xFF,
           sizeof(record->paired_peer_reserved));
    for (idx = 0U; idx < ZY100_SYSTEM_INFO_MAX_PAIRED_PEERS; idx++)
    {
        memcpy(record->paired_peers[idx].peer_addr,
               info->paired_peers[idx].peer_addr,
               sizeof(record->paired_peers[idx].peer_addr));
        record->paired_peers[idx].peer_addr_type =
            info->paired_peers[idx].peer_addr_type;
        record->paired_peers[idx].valid = info->paired_peers[idx].valid;
    }

    record->software_version_code = info->software_version_code;
    record->latest_user_id = info->latest_user_id;
    record->ota_time_state = info->ota_time.state;
    memset(record->ota_time_reserved, 0xFF, sizeof(record->ota_time_reserved));
    record->ota_checkpoint_unix_ms = info->ota_time.checkpoint_unix_ms;
    record->ota_checkpoint_rtc_ticks = info->ota_time.checkpoint_rtc_ticks;
    record->ota_checkpoint_rtc_wrap_ticks =
        info->ota_time.checkpoint_rtc_wrap_ticks;
    record->ota_resume_unix_ms = info->ota_time.resume_unix_ms;
    record->ota_checkpoint_user_id = info->ota_time.checkpoint_user_id;
    record->ota_checkpoint_tick_hz = info->ota_time.checkpoint_tick_hz;
    record->ota_source_version_code = info->ota_time.source_version_code;
    record->reserved[ZY100_SYS_INFO_FACTORY_ACCEPTANCE_OFFSET] =
        info->factory_acceptance_state;
    record->reserved[ZY100_SYS_INFO_WHOLE_UNIT_OFFSET] =
        info->whole_unit_state;
    record->reserved[ZY100_SYS_INFO_PRODUCTION_SHIPPING_OFFSET] =
        info->production_shipping_state;
    record->reserved[3] = info->user_reset_state;
    record->crc32 = zy100_system_info_record_crc(record);
}

static bool zy100_system_info_write_slot(zy100_sys_info_slot_t slot,
                                         const zy100_system_info_t *info,
                                         uint32_t sequence)
{
    zy100_sys_info_record_t record;
    zy100_sys_info_record_t verify;
    drv_internal_flash_status_t status;
    uint32_t offset;

    if ((slot == ZY100_SYS_INFO_SLOT_NONE) || (info == NULL))
    {
        return false;
    }

    offset = zy100_system_info_slot_offset(slot);
    zy100_system_info_to_record(info, sequence, &record);

    status = drv_internal_flash_erase_sector(offset);
    if (status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        DBG_DIRECT("[SYS_INFO] erase_fail slot=%u off=0x%08lX status=%s",
                   (unsigned int)slot,
                   (unsigned long)offset,
                   drv_internal_flash_status_name(status));
        return false;
    }

    status = drv_internal_flash_write(offset,
                                      (const uint8_t *)&record,
                                      (uint32_t)sizeof(record));
    if (status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        DBG_DIRECT("[SYS_INFO] write_fail slot=%u off=0x%08lX status=%s",
                   (unsigned int)slot,
                   (unsigned long)offset,
                   drv_internal_flash_status_name(status));
        return false;
    }

    status = drv_internal_flash_read(offset,
                                     (uint8_t *)&verify,
                                     (uint32_t)sizeof(verify));
    if (status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        DBG_DIRECT("[SYS_INFO] verify_read_fail slot=%u off=0x%08lX status=%s",
                   (unsigned int)slot,
                   (unsigned long)offset,
                   drv_internal_flash_status_name(status));
        return false;
    }
    if (!zy100_system_info_record_valid(&verify) ||
        (verify.sequence != sequence) ||
        (memcmp(&verify, &record, sizeof(record)) != 0))
    {
        DBG_DIRECT("[SYS_INFO] verify_fail slot=%u off=0x%08lX seq=%lu",
                   (unsigned int)slot,
                   (unsigned long)offset,
                   (unsigned long)sequence);
        return false;
    }

    return true;
}

bool zy100_system_info_store_ready(void)
{
    drv_internal_flash_region_t region;
    drv_internal_flash_status_t status;

    status = drv_internal_flash_get_region(&region);
    if (status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        DBG_DIRECT("[SYS_INFO] not_ready status=%s",
                   drv_internal_flash_status_name(status));
        return false;
    }
    if ((region.sector_bytes != ZY100_INTERNAL_FLASH_SECTOR_BYTES) ||
        (region.size_bytes < ZY100_SYSTEM_INFO_REGION_END_OFFSET))
    {
        DBG_DIRECT("[SYS_INFO] bad_region base=0x%08lX size=%lu sector=%lu",
                   (unsigned long)region.base_addr,
                   (unsigned long)region.size_bytes,
                   (unsigned long)region.sector_bytes);
        return false;
    }

    return true;
}

bool zy100_system_info_load(zy100_system_info_t *out)
{
    zy100_sys_info_record_t record;
    zy100_sys_info_slot_t slot;
    bool found;

    if (out == NULL)
    {
        return false;
    }

    zy100_system_info_fill_defaults(out);
    if (!zy100_system_info_store_ready())
    {
        return false;
    }
    if (!zy100_system_info_find_latest(&record, &slot, &found))
    {
        return false;
    }
    if (!found)
    {
        return true;
    }

    (void)slot;
    zy100_system_info_from_record(&record, out);
    return true;
}

bool zy100_system_info_save(const zy100_system_info_t *info)
{
    zy100_system_info_t normalized;
    zy100_sys_info_record_t latest;
    zy100_sys_info_slot_t latest_slot;
    zy100_sys_info_slot_t target_slot;
    uint32_t next_sequence;
    bool found;

    if ((info == NULL) ||
        !zy100_system_info_acceptance_state_valid(
            info->factory_acceptance_state) ||
        !zy100_system_info_whole_unit_state_valid(info->whole_unit_state) ||
        !zy100_system_info_production_shipping_state_valid(
            info->production_shipping_state))
    {
        return false;
    }
    if (!zy100_system_info_store_ready())
    {
        return false;
    }
    if (!zy100_system_info_find_latest(&latest, &latest_slot, &found))
    {
        return false;
    }

    normalized = *info;
    zy100_system_info_normalize(&normalized);

    if (found)
    {
        next_sequence = latest.sequence + 1U;
        if (next_sequence == 0U)
        {
            next_sequence = 1U;
        }
        target_slot = (latest_slot == ZY100_SYS_INFO_SLOT_PRIMARY) ?
                      ZY100_SYS_INFO_SLOT_BACKUP :
                      ZY100_SYS_INFO_SLOT_PRIMARY;
    }
    else
    {
        next_sequence = 1U;
        target_slot = ZY100_SYS_INFO_SLOT_PRIMARY;
    }

    return zy100_system_info_write_slot(target_slot,
                                        &normalized,
                                        next_sequence);
}

bool zy100_system_info_ensure_current_software_version(void)
{
    zy100_system_info_t info;
    zy100_sys_info_record_t latest;
    zy100_sys_info_slot_t latest_slot;
    bool found;
    bool saved;

    if (!zy100_system_info_store_ready())
    {
        return false;
    }
    if (!zy100_system_info_find_latest(&latest, &latest_slot, &found))
    {
        return false;
    }
    if (found && zy100_system_info_record_current_software_version(&latest))
    {
        return true;
    }

    if (found)
    {
        zy100_system_info_from_record(&latest, &info);
    }
    else
    {
        zy100_system_info_fill_defaults(&info);
    }

    saved = zy100_system_info_save(&info);
    DBG_DIRECT("[SYS_INFO] version_ensure action=%s old_ver=%u slot=%u code=%lu fw=%u.%u.%u.%u ok=%u",
               found ? "migrate" : "create",
               found ? latest.version : 0U,
               (unsigned int)latest_slot,
               (unsigned long)VERSION_CODE,
               (unsigned int)VERSION_MAJOR,
               (unsigned int)VERSION_MINOR,
               (unsigned int)VERSION_REVISION,
               (unsigned int)VERSION_BUILDNUM,
               saved ? 1U : 0U);
    return saved;
}

bool zy100_system_info_mark_first_power_seen(void)
{
    zy100_system_info_t info;

    if (!zy100_system_info_load(&info))
    {
        return false;
    }
    if (info.first_power_seen != 0U)
    {
        return true;
    }

    info.first_power_seen = 1U;
    return zy100_system_info_save(&info);
}

bool zy100_system_info_set_ota_success_led_pending(bool enable)
{
    zy100_system_info_t info;
    uint8_t pending;

    if (!zy100_system_info_load(&info))
    {
        return false;
    }

    pending = enable ? 1U : 0U;
    if (info.ota_success_led_pending == pending)
    {
        return true;
    }

    info.ota_success_led_pending = pending;
    return zy100_system_info_save(&info);
}

bool zy100_system_info_get_ota_success_led_pending(bool *pending_out)
{
    zy100_system_info_t info;

    if (pending_out == NULL)
    {
        return false;
    }
    *pending_out = false;

    if (!zy100_system_info_load(&info))
    {
        return false;
    }

    *pending_out = (info.ota_success_led_pending != 0U);
    return true;
}

bool zy100_system_info_set_factory_acceptance_state(
    zy100_factory_acceptance_state_t state)
{
    zy100_system_info_t info;

    if ((state != ZY100_FACTORY_ACCEPTANCE_NONE) &&
        (state != ZY100_FACTORY_ACCEPTANCE_PENDING) &&
        (state != ZY100_FACTORY_ACCEPTANCE_SHIP_ARMED))
    {
        return false;
    }
    if (!zy100_system_info_load(&info))
    {
        return false;
    }
    if (info.factory_acceptance_state == (uint8_t)state)
    {
        return true;
    }
    info.factory_acceptance_state = (uint8_t)state;
    return zy100_system_info_save(&info);
}

bool zy100_system_info_get_factory_acceptance_state(
    zy100_factory_acceptance_state_t *state_out)
{
    zy100_system_info_t info;

    if ((state_out == NULL) || !zy100_system_info_load(&info))
    {
        return false;
    }
    *state_out = (zy100_factory_acceptance_state_t)
                 info.factory_acceptance_state;
    return true;
}

bool zy100_system_info_set_manufacturing_states(
    zy100_factory_acceptance_state_t acceptance_state,
    zy100_whole_unit_state_t whole_unit_state)
{
    zy100_system_info_t info;

    if (!zy100_system_info_acceptance_state_valid((uint8_t)acceptance_state) ||
        !zy100_system_info_whole_unit_state_valid((uint8_t)whole_unit_state) ||
        !zy100_system_info_load(&info))
    {
        return false;
    }
    if ((info.factory_acceptance_state == (uint8_t)acceptance_state) &&
        (info.whole_unit_state == (uint8_t)whole_unit_state))
    {
        return true;
    }
    info.factory_acceptance_state = (uint8_t)acceptance_state;
    info.whole_unit_state = (uint8_t)whole_unit_state;
    return zy100_system_info_save(&info);
}

bool zy100_system_info_get_manufacturing_states(
    zy100_factory_acceptance_state_t *acceptance_state_out,
    zy100_whole_unit_state_t *whole_unit_state_out)
{
    zy100_system_info_t info;

    if ((acceptance_state_out == NULL) || (whole_unit_state_out == NULL) ||
        !zy100_system_info_load(&info) ||
        !zy100_system_info_acceptance_state_valid(
            info.factory_acceptance_state) ||
        !zy100_system_info_whole_unit_state_valid(info.whole_unit_state))
    {
        return false;
    }
    *acceptance_state_out =
        (zy100_factory_acceptance_state_t)info.factory_acceptance_state;
    *whole_unit_state_out =
        (zy100_whole_unit_state_t)info.whole_unit_state;
    return true;
}

bool zy100_system_info_set_whole_unit_state(zy100_whole_unit_state_t state)
{
    zy100_system_info_t info;

    if (!zy100_system_info_whole_unit_state_valid((uint8_t)state) ||
        !zy100_system_info_load(&info))
    {
        return false;
    }
    if (info.whole_unit_state == (uint8_t)state)
    {
        return true;
    }
    info.whole_unit_state = (uint8_t)state;
    return zy100_system_info_save(&info);
}

bool zy100_system_info_get_whole_unit_state(zy100_whole_unit_state_t *state_out)
{
    zy100_system_info_t info;

    if ((state_out == NULL) || !zy100_system_info_load(&info) ||
        !zy100_system_info_whole_unit_state_valid(info.whole_unit_state))
    {
        return false;
    }
    *state_out = (zy100_whole_unit_state_t)info.whole_unit_state;
    return true;
}

bool zy100_system_info_set_production_shipping_state(
    zy100_production_shipping_state_t state)
{
    zy100_system_info_t info;

    if (!zy100_system_info_production_shipping_state_valid((uint8_t)state) ||
        !zy100_system_info_load(&info))
    {
        return false;
    }
    if (info.production_shipping_state == (uint8_t)state)
    {
        return true;
    }
    info.production_shipping_state = (uint8_t)state;
    return zy100_system_info_save(&info);
}

bool zy100_system_info_get_production_shipping_state(
    zy100_production_shipping_state_t *state_out)
{
    zy100_system_info_t info;

    if ((state_out == NULL) || !zy100_system_info_load(&info) ||
        !zy100_system_info_production_shipping_state_valid(
            info.production_shipping_state))
    {
        return false;
    }
    *state_out = (zy100_production_shipping_state_t)
                 info.production_shipping_state;
    return true;
}

bool zy100_system_info_ota_time_clear_stale(void)
{
    zy100_system_info_t info;

    if (!zy100_system_info_load(&info))
    {
        return false;
    }
    if (info.ota_time.state == (uint8_t)ZY100_OTA_TIME_STATE_NONE)
    {
        return true;
    }

    DBG_DIRECT("[OTA_TIME] clear_stale state=%u source_fw=%lu",
               (uint32_t)info.ota_time.state,
               (unsigned long)info.ota_time.source_version_code);
    zy100_system_info_ota_time_clear(&info.ota_time);
    return zy100_system_info_save(&info);
}

bool zy100_system_info_ota_time_arm(
    const zy100_system_info_ota_time_t *checkpoint)
{
    zy100_system_info_t info;

    if ((checkpoint == NULL) ||
        !zy100_system_info_ota_time_fields_valid(checkpoint))
    {
        (void)zy100_system_info_ota_time_clear_stale();
        return false;
    }
    if (!zy100_system_info_load(&info))
    {
        return false;
    }

    /* Make a failed re-arm safe: a previously interrupted ARMED record must
     * not survive as the latest valid slot if the new checkpoint write fails. */
    if (info.ota_time.state != (uint8_t)ZY100_OTA_TIME_STATE_NONE)
    {
        zy100_system_info_ota_time_clear(&info.ota_time);
        if (!zy100_system_info_save(&info))
        {
            return false;
        }
    }

    info.ota_time = *checkpoint;
    info.ota_time.state = (uint8_t)ZY100_OTA_TIME_STATE_ARMED;
    info.ota_time.resume_unix_ms = 0ULL;
    if (!zy100_system_info_save(&info))
    {
        return false;
    }

    DBG_DIRECT("[OTA_TIME] arm user=%lu unix_ms=%llu ticks=%llu wrap=%llu hz=%lu source_fw=%lu",
               (unsigned long)info.ota_time.checkpoint_user_id,
               (unsigned long long)info.ota_time.checkpoint_unix_ms,
               (unsigned long long)info.ota_time.checkpoint_rtc_ticks,
               (unsigned long long)info.ota_time.checkpoint_rtc_wrap_ticks,
               (unsigned long)info.ota_time.checkpoint_tick_hz,
               (unsigned long)info.ota_time.source_version_code);
    return true;
}

static bool zy100_system_info_ota_elapsed_ms(
    const zy100_system_info_ota_time_t *checkpoint,
    uint64_t current_rtc_ticks,
    uint64_t current_rtc_wrap_ticks,
    uint32_t current_tick_hz,
    uint64_t *elapsed_ms_out)
{
    uint64_t delta_ticks;
    uint64_t whole_seconds;
    uint64_t remainder_ticks;
    uint64_t elapsed_ms;
    const uint64_t u64_max = ~(uint64_t)0;

    if ((checkpoint == NULL) || (elapsed_ms_out == NULL) ||
        !zy100_system_info_ota_time_fields_valid(checkpoint) ||
        (current_rtc_wrap_ticks == 0ULL) ||
        (current_rtc_ticks >= current_rtc_wrap_ticks) ||
        (checkpoint->checkpoint_rtc_wrap_ticks !=
         current_rtc_wrap_ticks) ||
        (checkpoint->checkpoint_tick_hz != current_tick_hz) ||
        (current_tick_hz == 0U))
    {
        return false;
    }

    if (current_rtc_ticks >= checkpoint->checkpoint_rtc_ticks)
    {
        delta_ticks = current_rtc_ticks -
                      checkpoint->checkpoint_rtc_ticks;
    }
    else
    {
        delta_ticks = (current_rtc_wrap_ticks -
                       checkpoint->checkpoint_rtc_ticks) +
                      current_rtc_ticks;
    }

    whole_seconds = delta_ticks / (uint64_t)current_tick_hz;
    remainder_ticks = delta_ticks % (uint64_t)current_tick_hz;
    if (whole_seconds > (u64_max / 1000ULL))
    {
        return false;
    }
    elapsed_ms = (whole_seconds * 1000ULL) +
                 ((remainder_ticks * 1000ULL) /
                  (uint64_t)current_tick_hz);
    *elapsed_ms_out = elapsed_ms;
    return true;
}

bool zy100_system_info_ota_success_finalize(uint64_t current_rtc_ticks,
                                            uint64_t current_rtc_wrap_ticks,
                                            uint32_t current_tick_hz,
                                            bool *resume_created_out)
{
    zy100_system_info_t info;
    uint64_t elapsed_ms = 0ULL;
    uint64_t resume_ms = 0ULL;
    const uint64_t u64_max = ~(uint64_t)0;
    bool resume_valid = false;
    bool saved;

    if (resume_created_out != NULL)
    {
        *resume_created_out = false;
    }
    if (!zy100_system_info_load(&info))
    {
        return false;
    }

    if ((info.ota_time.state == (uint8_t)ZY100_OTA_TIME_STATE_ARMED) &&
        zy100_system_info_ota_elapsed_ms(&info.ota_time,
                                         current_rtc_ticks,
                                         current_rtc_wrap_ticks,
                                         current_tick_hz,
                                         &elapsed_ms) &&
        (info.ota_time.checkpoint_unix_ms <=
         (u64_max - elapsed_ms)) &&
        ((info.ota_time.checkpoint_unix_ms + elapsed_ms) <=
         (u64_max - ZY100_OTA_IMAGE_MIGRATION_COMPENSATION_MS)))
    {
        resume_ms = info.ota_time.checkpoint_unix_ms + elapsed_ms +
                    ZY100_OTA_IMAGE_MIGRATION_COMPENSATION_MS;
        info.ota_time.state =
            (uint8_t)ZY100_OTA_TIME_STATE_RESUME_PENDING;
        info.ota_time.resume_unix_ms = resume_ms;
        resume_valid = true;
    }
    else
    {
        zy100_system_info_ota_time_clear(&info.ota_time);
    }

    /* The LED and time-resume markers describe the same successful image
     * transition and therefore share one alternating-slot transaction. */
    info.ota_success_led_pending = 1U;
    saved = zy100_system_info_save(&info);
    if (saved && resume_valid && (resume_created_out != NULL))
    {
        *resume_created_out = true;
    }

    DBG_DIRECT("[OTA_TIME] finalize saved=%u resume=%u elapsed_ms=%llu compensation_ms=%llu resume_ms=%llu",
               saved ? 1U : 0U,
               (saved && resume_valid) ? 1U : 0U,
               (unsigned long long)elapsed_ms,
               (unsigned long long)ZY100_OTA_IMAGE_MIGRATION_COMPENSATION_MS,
               (unsigned long long)resume_ms);
    return saved;
}

bool zy100_system_info_ota_boot_consume(void)
{
    zy100_system_info_t info;

    if (!zy100_system_info_load(&info))
    {
        return false;
    }
    if ((info.ota_success_led_pending == 0U) &&
        (info.ota_time.state == (uint8_t)ZY100_OTA_TIME_STATE_NONE))
    {
        return true;
    }

    info.ota_success_led_pending = 0U;
    zy100_system_info_ota_time_clear(&info.ota_time);
    return zy100_system_info_save(&info);
}

static int8_t zy100_system_info_find_peer_slot(
    const zy100_system_info_t *info,
    const uint8_t peer_addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES],
    uint8_t peer_type)
{
    uint8_t idx;

    if ((info == NULL) || (peer_addr == NULL))
    {
        return -1;
    }

    for (idx = 0U; idx < ZY100_SYSTEM_INFO_MAX_PAIRED_PEERS; idx++)
    {
        if (zy100_system_info_peer_match(&info->paired_peers[idx],
                                         peer_addr,
                                         peer_type))
        {
            return (int8_t)idx;
        }
    }
    return -1;
}

static int8_t zy100_system_info_find_empty_peer_slot(
    const zy100_system_info_t *info)
{
    uint8_t idx;

    if (info == NULL)
    {
        return -1;
    }

    for (idx = 0U; idx < ZY100_SYSTEM_INFO_MAX_PAIRED_PEERS; idx++)
    {
        if (zy100_system_info_bool_byte(info->paired_peers[idx].valid) == 0U)
        {
            return (int8_t)idx;
        }
    }
    return -1;
}

bool zy100_system_info_add_paired_peer(
    const uint8_t peer_addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES],
    uint8_t peer_type)
{
    zy100_system_info_t info;
    int8_t slot;

    if (peer_addr == NULL)
    {
        return false;
    }
    if (!zy100_system_info_load(&info))
    {
        return false;
    }

    slot = zy100_system_info_find_peer_slot(&info, peer_addr, peer_type);
    if (slot < 0)
    {
        slot = zy100_system_info_find_empty_peer_slot(&info);
    }
    if (slot < 0)
    {
        DBG_DIRECT("[SYS_INFO] paired_peer_full max=%u type=%u addr="
                   ZY100_BDADDR_RAW_FMT,
                   (uint32_t)ZY100_SYSTEM_INFO_MAX_PAIRED_PEERS,
                   peer_type,
                   ZY100_BDADDR_RAW_ARG(peer_addr));
        return false;
    }

    memcpy(info.paired_peers[(uint8_t)slot].peer_addr,
           peer_addr,
           sizeof(info.paired_peers[(uint8_t)slot].peer_addr));
    info.paired_peers[(uint8_t)slot].peer_addr_type = peer_type;
    info.paired_peers[(uint8_t)slot].valid = 1U;

    memcpy(info.last_peer_addr, peer_addr, sizeof(info.last_peer_addr));
    info.last_peer_addr_type = peer_type;
    info.last_peer_addr_valid = 1U;

    return zy100_system_info_save(&info);
}

bool zy100_system_info_set_pairing_bonded(
    const uint8_t peer_addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES],
    uint8_t peer_type)
{
    return zy100_system_info_add_paired_peer(peer_addr, peer_type);
}

bool zy100_system_info_remove_paired_peer(
    const uint8_t peer_addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES],
    uint8_t peer_type)
{
    zy100_system_info_t info;
    int8_t slot;

    if (peer_addr == NULL)
    {
        return false;
    }
    if (!zy100_system_info_load(&info))
    {
        return false;
    }

    slot = zy100_system_info_find_peer_slot(&info, peer_addr, peer_type);
    if (slot >= 0)
    {
        memset(&info.paired_peers[(uint8_t)slot],
               0,
               sizeof(info.paired_peers[(uint8_t)slot]));
    }

    return zy100_system_info_save(&info);
}

bool zy100_system_info_clear_pairing(void)
{
    zy100_system_info_t info;

    if (!zy100_system_info_load(&info))
    {
        return false;
    }

    info.pairing_summary_valid = 0U;
    info.pairing_bonded = 0U;
    memset(info.last_peer_addr, 0, sizeof(info.last_peer_addr));
    info.last_peer_addr_type = 0U;
    info.last_peer_addr_valid = 0U;
    info.paired_peer_count = 0U;
    memset(info.paired_peers, 0, sizeof(info.paired_peers));
    return zy100_system_info_save(&info);
}

bool zy100_system_info_set_ble_addr(
    const uint8_t addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES],
    uint8_t addr_type)
{
    zy100_system_info_t info;

    if (addr == NULL)
    {
        return false;
    }
    if (!zy100_system_info_load(&info))
    {
        return false;
    }

    memcpy(info.ble_addr, addr, sizeof(info.ble_addr));
    info.ble_addr_type = addr_type;
    info.ble_addr_valid = 1U;
    return zy100_system_info_save(&info);
}

bool zy100_system_info_get_latest_user_id(uint32_t *user_id_out,
                                          bool *valid_out)
{
    zy100_system_info_t info;

    if ((user_id_out == NULL) || (valid_out == NULL))
    {
        return false;
    }
    *user_id_out = 0U;
    *valid_out = false;

    if (!zy100_system_info_load(&info))
    {
        return false;
    }

    *user_id_out = info.latest_user_id;
    *valid_out = (info.latest_user_id != 0U);
    return true;
}

bool zy100_system_info_set_latest_user_id(uint32_t user_id)
{
    zy100_system_info_t info;

    if (user_id == 0U)
    {
        return false;
    }
    if (!zy100_system_info_load(&info))
    {
        return false;
    }
    if (info.latest_user_id == user_id)
    {
        return true;
    }

    info.latest_user_id = user_id;
    return zy100_system_info_save(&info);
}
