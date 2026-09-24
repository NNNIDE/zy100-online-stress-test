#include "zy100_calibration_store.h"

#include <string.h>

#include <ftl.h>

#include "trace.h"
#include "../app_mfg/zp_mfg_store.h"
#include "zy100_app_ftl_layout.h"

typedef struct
{
    uint32_t generation;
    uint32_t blob_crc32;
    uint8_t blob[ZY100_CAL_RECORD_MAX_BYTES];
} zy100_cal_ftl_slot_t;

typedef char zy100_cal_ftl_slot_size_check[
    (sizeof(zy100_cal_ftl_slot_t) == ZY100_APP_FTL_CAL_SLOT_BYTES) ? 1 : -1];

static bool zy100_cal_store_generation_newer(uint32_t candidate,
                                             uint32_t current)
{
    return ((int32_t)(candidate - current)) > 0;
}

static zy100_cal_store_status_t zy100_cal_store_read_slot(
    uint16_t offset,
    zy100_cal_ftl_slot_t *slot,
    zy100_cal_record_info_t *info,
    bool *valid)
{
    uint32_t result;
    zy100_cal_record_info_t parsed;
    uint16_t record_bytes;

    if ((slot == NULL) || (info == NULL) || (valid == NULL))
    {
        return ZY100_CAL_STORE_INVALID;
    }
    *valid = false;
    memset(slot, 0, sizeof(*slot));
    result = ftl_load(slot, offset, (uint16_t)sizeof(*slot));
    if (result == FTL_READ_ERROR_READ_NOT_FOUND)
    {
        return ZY100_CAL_STORE_NOT_FOUND;
    }
    if (result != FTL_READ_SUCCESS)
    {
        DBG_DIRECT("[CAL_STORE] read_fail off=0x%04X result=%lu",
                   offset, (unsigned long)result);
        return ZY100_CAL_STORE_READ_ERROR;
    }

    record_bytes = zy100_cal_get_u16_le(&slot->blob[6]);
    if ((slot->generation != zy100_cal_get_u32_le(&slot->blob[8])) ||
        (slot->blob_crc32 !=
         zy100_cal_get_u32_le(&slot->blob[ZY100_CAL_RECORD_CRC_OFFSET])) ||
        !zy100_cal_record_validate(slot->blob, record_bytes, &parsed))
    {
        return ZY100_CAL_STORE_INVALID;
    }
    *info = parsed;
    *valid = true;
    return ZY100_CAL_STORE_OK;
}

static zy100_cal_store_status_t zy100_cal_store_legacy_ftl_load(
    uint8_t out[ZY100_CAL_RECORD_MAX_BYTES],
    zy100_cal_record_info_t *info)
{
    zy100_cal_ftl_slot_t slot;
    zy100_cal_record_info_t primary_info;
    zy100_cal_record_info_t backup_info;
    bool primary_valid = false;
    bool backup_valid = false;
    zy100_cal_store_status_t primary_status;
    zy100_cal_store_status_t backup_status;

    if ((out == NULL) || (info == NULL))
    {
        return ZY100_CAL_STORE_INVALID;
    }
    memset(out, 0, ZY100_CAL_RECORD_MAX_BYTES);
    memset(info, 0, sizeof(*info));

    primary_status = zy100_cal_store_read_slot(
                                               ZY100_APP_FTL_CAL_PRIMARY_OFFSET,
                                               &slot, &primary_info,
                                               &primary_valid);
    if (primary_valid)
    {
        memcpy(out, slot.blob, primary_info.record_bytes);
        *info = primary_info;
    }
    backup_status = zy100_cal_store_read_slot(
                                              ZY100_APP_FTL_CAL_BACKUP_OFFSET,
                                              &slot, &backup_info,
                                              &backup_valid);
    if (backup_valid &&
        (!primary_valid ||
         zy100_cal_store_generation_newer(backup_info.generation,
                                          primary_info.generation)))
    {
        memset(out, 0, ZY100_CAL_RECORD_MAX_BYTES);
        memcpy(out, slot.blob, backup_info.record_bytes);
        *info = backup_info;
    }
    if (primary_valid || backup_valid)
    {
        return ZY100_CAL_STORE_OK;
    }
    if ((primary_status == ZY100_CAL_STORE_READ_ERROR) ||
        (backup_status == ZY100_CAL_STORE_READ_ERROR))
    {
        return ZY100_CAL_STORE_READ_ERROR;
    }
    return ZY100_CAL_STORE_NOT_FOUND;
}

zy100_cal_store_status_t zy100_cal_store_stage_load(
    uint8_t out[ZY100_CAL_RECORD_MAX_BYTES],
    zy100_cal_record_info_t *info)
{
    return zy100_cal_store_legacy_ftl_load(out, info);
}

zy100_cal_store_status_t zy100_cal_store_stage_save(
    const uint8_t *record,
    uint16_t len,
    zy100_cal_record_info_t *info)
{
    zy100_cal_record_info_t parsed;
    zy100_cal_record_info_t current_info;
    zy100_cal_ftl_slot_t slot;
    zy100_cal_ftl_slot_t verify;
    uint8_t current[ZY100_CAL_RECORD_MAX_BYTES];
    uint16_t target_offset = ZY100_APP_FTL_CAL_PRIMARY_OFFSET;
    uint32_t result;

    if ((record == NULL) || (info == NULL) ||
        !zy100_cal_record_validate(record, len, &parsed))
    {
        return ZY100_CAL_STORE_INVALID;
    }
    if (zy100_cal_store_legacy_ftl_load(current, &current_info) ==
        ZY100_CAL_STORE_OK)
    {
        zy100_cal_ftl_slot_t primary;
        zy100_cal_ftl_slot_t backup;
        zy100_cal_record_info_t primary_info;
        zy100_cal_record_info_t backup_info;
        bool primary_valid = false;
        bool backup_valid = false;

        (void)zy100_cal_store_read_slot(ZY100_APP_FTL_CAL_PRIMARY_OFFSET,
                                        &primary, &primary_info,
                                        &primary_valid);
        (void)zy100_cal_store_read_slot(ZY100_APP_FTL_CAL_BACKUP_OFFSET,
                                        &backup, &backup_info,
                                        &backup_valid);
        if (!primary_valid)
        {
            target_offset = ZY100_APP_FTL_CAL_PRIMARY_OFFSET;
        }
        else if (!backup_valid)
        {
            target_offset = ZY100_APP_FTL_CAL_BACKUP_OFFSET;
        }
        else
        {
            target_offset = zy100_cal_store_generation_newer(
                            backup_info.generation,
                            primary_info.generation) ?
                            ZY100_APP_FTL_CAL_PRIMARY_OFFSET :
                            ZY100_APP_FTL_CAL_BACKUP_OFFSET;
        }
        if (parsed.generation <= current_info.generation)
        {
            return ZY100_CAL_STORE_INVALID;
        }
    }

    memset(&slot, 0, sizeof(slot));
    slot.generation = parsed.generation;
    slot.blob_crc32 = parsed.crc32;
    memcpy(slot.blob, record, len);
    result = ftl_save(&slot, target_offset, (uint16_t)sizeof(slot));
    if (result != FTL_WRITE_SUCCESS)
    {
        DBG_DIRECT("[CAL_STORE] stage_write_fail off=0x%04X result=%lu",
                   target_offset, (unsigned long)result);
        return ZY100_CAL_STORE_WRITE_ERROR;
    }
    memset(&verify, 0, sizeof(verify));
    result = ftl_load(&verify, target_offset, (uint16_t)sizeof(verify));
    if ((result != FTL_READ_SUCCESS) ||
        (memcmp(&verify, &slot, sizeof(slot)) != 0) ||
        !zy100_cal_record_validate(verify.blob, len, NULL))
    {
        return ZY100_CAL_STORE_VERIFY_ERROR;
    }
    *info = parsed;
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[CAL_STORE] stage_ok off=0x%04X generation=%lu bytes=%u crc=0x%08lx",
               target_offset, (unsigned long)parsed.generation,
               parsed.record_bytes, (unsigned long)parsed.crc32);
    return ZY100_CAL_STORE_OK;
}

static zy100_cal_store_status_t zy100_cal_store_from_mfg_status(
    zp_mfg_store_status_t status)
{
    switch (status)
    {
    case ZP_MFG_STORE_STATUS_OK:
        return ZY100_CAL_STORE_OK;
    case ZP_MFG_STORE_STATUS_NOT_FOUND:
    case ZP_MFG_STORE_STATUS_CALIBRATION_NOT_FOUND:
        return ZY100_CAL_STORE_NOT_FOUND;
    case ZP_MFG_STORE_STATUS_INVALID_PARAM:
    case ZP_MFG_STORE_STATUS_CALIBRATION_INVALID:
        return ZY100_CAL_STORE_INVALID;
    case ZP_MFG_STORE_STATUS_VERIFY_FAILED:
        return ZY100_CAL_STORE_VERIFY_ERROR;
    case ZP_MFG_STORE_STATUS_ALREADY_LOCKED:
    case ZP_MFG_STORE_STATUS_FLASH_ERROR:
    default:
        return ZY100_CAL_STORE_WRITE_ERROR;
    }
}

zy100_cal_store_status_t zy100_cal_store_load(
    uint8_t out[ZY100_CAL_RECORD_MAX_BYTES],
    zy100_cal_record_info_t *info)
{
    zp_mfg_store_status_t mfg_status;
    zy100_cal_store_status_t legacy_status;
    uint16_t length = 0U;

    if ((out == NULL) || (info == NULL))
    {
        return ZY100_CAL_STORE_INVALID;
    }
    memset(out, 0, ZY100_CAL_RECORD_MAX_BYTES);
    memset(info, 0, sizeof(*info));
    mfg_status = zp_mfg_store_calibration_load(out, &length);
    if (mfg_status == ZP_MFG_STORE_STATUS_OK)
    {
        if (!zy100_cal_record_validate(out, length, info))
        {
            return ZY100_CAL_STORE_INVALID;
        }
        return ZY100_CAL_STORE_OK;
    }
    if (mfg_status != ZP_MFG_STORE_STATUS_CALIBRATION_NOT_FOUND)
    {
        return zy100_cal_store_from_mfg_status(mfg_status);
    }

    legacy_status = zy100_cal_store_legacy_ftl_load(out, info);
    if (legacy_status != ZY100_CAL_STORE_OK)
    {
        return legacy_status;
    }
    mfg_status = zp_mfg_store_calibration_save(out, info->record_bytes);
    if (mfg_status != ZP_MFG_STORE_STATUS_OK)
    {
        DBG_DIRECT("[CAL_STORE] legacy_migration_fail status=%s",
                   zp_mfg_store_status_name(mfg_status));
        return zy100_cal_store_from_mfg_status(mfg_status);
    }
    DBG_DIRECT("[CAL_STORE] legacy_ftl_migrated generation=%lu bytes=%u",
               (unsigned long)info->generation, info->record_bytes);
    return ZY100_CAL_STORE_OK;
}

zy100_cal_store_status_t zy100_cal_store_save(
    const uint8_t *record,
    uint16_t len,
    zy100_cal_record_info_t *info)
{
    zy100_cal_record_info_t parsed;
    zp_mfg_store_status_t status;

    if ((record == NULL) || (info == NULL) ||
        !zy100_cal_record_validate(record, len, &parsed))
    {
        return ZY100_CAL_STORE_INVALID;
    }
    status = zp_mfg_store_calibration_save(record, len);
    if (status != ZP_MFG_STORE_STATUS_OK)
    {
        DBG_DIRECT("[CAL_STORE] mfg_write_fail status=%s",
                   zp_mfg_store_status_name(status));
        return zy100_cal_store_from_mfg_status(status);
    }
    *info = parsed;
    return ZY100_CAL_STORE_OK;
}

zy100_cal_store_status_t zy100_cal_store_clear(void)
{
    uint32_t zeros[ZY100_APP_FTL_CAL_SLOT_BYTES / 4U] = {0};
    uint32_t check[ZY100_APP_FTL_CAL_SLOT_BYTES / 4U];
    uint16_t offset;
    /* Remove migration sources before publishing an empty formal record. */
    for (offset = ZY100_APP_FTL_CAL_PRIMARY_OFFSET;
         offset < ZY100_APP_FTL_CAL_END_OFFSET; offset += ZY100_APP_FTL_CAL_SLOT_BYTES)
    {
        if (ftl_save(zeros, offset, sizeof(zeros)) != 0U ||
            ftl_load(check, offset, sizeof(check)) != 0U ||
            memcmp(zeros, check, sizeof(zeros)) != 0)
            return ZY100_CAL_STORE_VERIFY_ERROR;
    }
    return zy100_cal_store_from_mfg_status(zp_mfg_store_calibration_clear());
}

const char *zy100_cal_store_status_name(zy100_cal_store_status_t status)
{
    switch (status)
    {
    case ZY100_CAL_STORE_OK: return "OK";
    case ZY100_CAL_STORE_NOT_FOUND: return "NOT_FOUND";
    case ZY100_CAL_STORE_INVALID: return "INVALID";
    case ZY100_CAL_STORE_READ_ERROR: return "READ_ERROR";
    case ZY100_CAL_STORE_WRITE_ERROR: return "WRITE_ERROR";
    case ZY100_CAL_STORE_VERIFY_ERROR: return "VERIFY_ERROR";
    default: return "UNKNOWN";
    }
}
