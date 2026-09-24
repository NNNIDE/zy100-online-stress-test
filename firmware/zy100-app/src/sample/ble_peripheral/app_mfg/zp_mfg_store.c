#include "zp_mfg_store.h"

#include <stddef.h>
#include <string.h>

#include "../common/zy100_internal_flash_layout.h"
#include "../driver/drv_internal_flash.h"
#include "../service/zy100_crc32.h"
#include "../app_build_config.h"

#if ZY100_BUILD_PRODUCTION
#include <trace.h>
#endif

#if ZY100_BUILD_FACTORY
#include <trace.h>
#define ZP_MFG_FACTORY_LOG(...) DBG_DIRECT(__VA_ARGS__)
#else
#define ZP_MFG_FACTORY_LOG(...)
#endif

typedef union
{
    zp_mfg_record_v3_t v3;
    zp_mfg_record_v4_t v4;
    zp_mfg_record_v5_t v5;
    uint8_t raw[sizeof(zp_mfg_record_v5_t)];
} zp_mfg_raw_record_t;

typedef struct
{
    zp_mfg_record_t record;
    bool valid;
    uint8_t slot;
    uint16_t raw_version;
} zp_mfg_slot_read_t;

typedef char zp_mfg_record_slot_check[
    (sizeof(zp_mfg_record_t) <= ZY100_MFG_SLOT_BYTES) ? 1 : -1];

static uint32_t zp_mfg_slot_offset(uint8_t slot)
{
    return (slot == 0U) ? ZY100_MFG_PRIMARY_OFFSET : ZY100_MFG_BACKUP_OFFSET;
}

static uint32_t zp_mfg_crc_with_zero_field(const void *object,
                                           uint32_t object_bytes,
                                           uint32_t crc_offset)
{
    uint32_t crc;
    const uint32_t zero_crc_field = 0U;
    const uint8_t *bytes = (const uint8_t *)object;
    const uint32_t after_crc_offset = crc_offset + sizeof(zero_crc_field);

    crc = zy100_crc32_ieee_begin();
    crc = zy100_crc32_ieee_update(crc, bytes, crc_offset);
    crc = zy100_crc32_ieee_update(crc,
                                  (const uint8_t *)&zero_crc_field,
                                  (uint32_t)sizeof(zero_crc_field));
    if (after_crc_offset < object_bytes)
    {
        crc = zy100_crc32_ieee_update(crc,
                                      &bytes[after_crc_offset],
                                      object_bytes - after_crc_offset);
    }
    return zy100_crc32_ieee_finish(crc);
}

static uint32_t zp_mfg_record_v3_crc(const zp_mfg_record_v3_t *record)
{
    return zp_mfg_crc_with_zero_field(
        record,
        (uint32_t)sizeof(*record),
        (uint32_t)offsetof(zp_mfg_record_v3_t, crc32));
}

static uint32_t zp_mfg_record_v4_crc(const zp_mfg_record_v4_t *record)
{
    return zp_mfg_crc_with_zero_field(
        record,
        (uint32_t)sizeof(*record),
        (uint32_t)offsetof(zp_mfg_record_v4_t, crc32));
}

static uint32_t zp_mfg_record_v5_crc(const zp_mfg_record_v5_t *record)
{
    return zp_mfg_crc_with_zero_field(
        record,
        (uint32_t)sizeof(*record),
        (uint32_t)offsetof(zp_mfg_record_v5_t, crc32));
}

static uint32_t zp_mfg_imu_cal_crc(
    const zp_mfg_imu_single_pose_cal_v1_t *calibration)
{
    return zp_mfg_crc_with_zero_field(
        calibration,
        (uint32_t)sizeof(*calibration),
        (uint32_t)offsetof(zp_mfg_imu_single_pose_cal_v1_t, crc32));
}

void zp_mfg_store_imu_cal_finalize(
    zp_mfg_imu_single_pose_cal_v1_t *calibration)
{
    if (calibration == NULL)
    {
        return;
    }
    calibration->magic = ZP_MFG_IMU_CAL_MAGIC;
    calibration->version = ZP_MFG_IMU_CAL_VERSION;
    calibration->bytes = (uint16_t)sizeof(*calibration);
    calibration->crc32 = zp_mfg_imu_cal_crc(calibration);
}

bool zp_mfg_store_imu_cal_valid(
    const zp_mfg_imu_single_pose_cal_v1_t *calibration)
{
    const uint32_t required_flags = ZP_MFG_IMU_CAL_FLAG_GYRO_VALID |
                                    ZP_MFG_IMU_CAL_FLAG_ACCEL_VALID;

    if (calibration == NULL)
    {
        return false;
    }
    return (calibration->magic == ZP_MFG_IMU_CAL_MAGIC) &&
           (calibration->version == ZP_MFG_IMU_CAL_VERSION) &&
           (calibration->bytes == (uint16_t)sizeof(*calibration)) &&
           (calibration->mode == ZP_MFG_IMU_CAL_MODE_SINGLE_POSE_V1) &&
           ((calibration->flags & required_flags) == required_flags) &&
           ((calibration->flags & ZP_MFG_IMU_CAL_FLAG_CHARACTERIZATION) == 0UL) &&
           (calibration->gravity_axis < 3U) &&
           ((calibration->gravity_sign == 1) ||
            (calibration->gravity_sign == -1)) &&
           (calibration->sample_count != 0UL) &&
           (calibration->crc32 == zp_mfg_imu_cal_crc(calibration));
}

bool zp_mfg_store_legacy_imu_cal_absent(const zp_mfg_record_t *record)
{
    const uint8_t *bytes;
    uint32_t index;

    if ((record == NULL) ||
        (record->version != ZP_MFG_RECORD_VERSION_V5))
    {
        return false;
    }
    /*
     * Compatibility recovery for VERSION_CODE 10255..10257: those builds
     * promoted a valid V3 record to V5 with an all-zero IMU sub-record but did
     * not yet set the provenance bit.  The marker never bypasses validation:
     * accept only the exact all-zero shape, so a partially populated or
     * CRC-invalid IMU record remains invalid even if the marker is damaged.
     */
    bytes = (const uint8_t *)&record->imu_cal;
    for (index = 0UL; index < (uint32_t)sizeof(record->imu_cal); index++)
    {
        if (bytes[index] != 0U)
        {
            return false;
        }
    }
    return true;
}

bool zp_mfg_store_record_locked(const zp_mfg_record_t *record)
{
    return (record != NULL) &&
           ((record->flags & ZP_MFG_RECORD_FLAG_LOCKED) != 0UL);
}

bool zp_mfg_store_record_valid(const zp_mfg_record_t *record)
{
    if (record == NULL)
    {
        return false;
    }
    return (record->magic == ZP_MFG_RECORD_MAGIC) &&
           (record->version == ZP_MFG_RECORD_VERSION_V4) &&
           (record->record_bytes == (uint16_t)sizeof(*record)) &&
           (record->crc32 == zp_mfg_record_v4_crc(record));
}

static bool zp_mfg_raw_v5_valid(const zp_mfg_record_v5_t *record)
{
    return (record != NULL) &&
           (record->magic == ZP_MFG_RECORD_MAGIC) &&
           (record->version == ZP_MFG_RECORD_VERSION_V5) &&
           (record->record_bytes == (uint16_t)sizeof(*record)) &&
           (record->calibration_blob_bytes <=
            ZP_MFG_CALIBRATION_BLOB_MAX_BYTES) &&
           (record->crc32 == zp_mfg_record_v5_crc(record));
}

static bool zp_mfg_raw_v3_valid(const zp_mfg_record_v3_t *record)
{
    return (record != NULL) &&
           (record->magic == ZP_MFG_RECORD_MAGIC) &&
           (record->version == ZP_MFG_RECORD_VERSION_V3) &&
           (record->record_bytes == (uint16_t)sizeof(*record)) &&
           (record->crc32 == zp_mfg_record_v3_crc(record));
}

static void zp_mfg_migrate_v3_to_canonical(const zp_mfg_record_v3_t *source,
                                            zp_mfg_record_t *destination)
{
    const uint32_t common_bytes =
        (uint32_t)offsetof(zp_mfg_record_v3_t, crc32);

    memset(destination, 0, sizeof(*destination));
    memcpy(destination, source, common_bytes);
    destination->version = ZP_MFG_RECORD_VERSION_V3;
    destination->record_bytes = (uint16_t)sizeof(*source);
    destination->crc32 = source->crc32;
}

static void zp_mfg_migrate_v5_to_canonical(const zp_mfg_record_v5_t *source,
                                            zp_mfg_record_t *destination)
{
    const uint32_t common_bytes =
        (uint32_t)offsetof(zp_mfg_record_v3_t, crc32);

    memset(destination, 0, sizeof(*destination));
    memcpy(destination, source, common_bytes);
    memcpy(&destination->imu_cal, &source->imu_cal,
           sizeof(destination->imu_cal));
    destination->version = ZP_MFG_RECORD_VERSION_V5;
    destination->record_bytes = (uint16_t)sizeof(*source);
    destination->crc32 = source->crc32;
}

#if ZY100_BUILD_PRODUCTION
static void zp_mfg_log_boot_slot(uint8_t slot,
                                const zp_mfg_raw_record_t *raw,
                                bool valid)
{
    uint32_t stored_crc = 0U;
    uint32_t computed_crc = 0U;
    bool crc_known = true;

    if (raw->v3.version == ZP_MFG_RECORD_VERSION_V3)
    {
        stored_crc = raw->v3.crc32;
        computed_crc = zp_mfg_record_v3_crc(&raw->v3);
    }
    else if (raw->v4.version == ZP_MFG_RECORD_VERSION_V4)
    {
        stored_crc = raw->v4.crc32;
        computed_crc = zp_mfg_record_v4_crc(&raw->v4);
    }
    else if (raw->v5.version == ZP_MFG_RECORD_VERSION_V5)
    {
        stored_crc = raw->v5.crc32;
        computed_crc = zp_mfg_record_v5_crc(&raw->v5);
    }
    else
    {
        crc_known = false;
    }
    DBG_DIRECT("[MFG_DIAG] slot=%u magic=0x%08lX ver=%u bytes=%u valid=%u",
               slot, (unsigned long)raw->v3.magic, raw->v3.version,
               raw->v3.record_bytes, valid ? 1U : 0U);
    DBG_DIRECT("[MFG_DIAG] slot=%u gen=%lu flags=0x%08lX reserved=0x%02X cleanup=%u",
               slot, (unsigned long)raw->v3.generation,
               (unsigned long)raw->v3.flags, raw->v3.reserved_flags,
               raw->v3.factory_cleanup_pass);
    DBG_DIRECT("[MFG_DIAG] slot=%u required=0x%08lX passed=0x%08lX",
               slot, (unsigned long)raw->v3.required_test_mask,
               (unsigned long)raw->v3.passed_test_mask);
    DBG_DIRECT("[MFG_DIAG] slot=%u crc_known=%u stored=0x%08lX calculated=0x%08lX",
               slot, crc_known ? 1U : 0U, (unsigned long)stored_crc,
               (unsigned long)computed_crc);
}
#endif

static zp_mfg_store_status_t zp_mfg_read_slot_internal(uint8_t slot,
                                              zp_mfg_slot_read_t *out,
                                              bool boot_diagnostic)
{
    drv_internal_flash_status_t flash_status;
    zp_mfg_raw_record_t raw_record;

    if ((slot > 1U) || (out == NULL))
    {
        return ZP_MFG_STORE_STATUS_INVALID_PARAM;
    }

    memset(out, 0, sizeof(*out));
    memset(&raw_record, 0, sizeof(raw_record));
    out->slot = slot;
    flash_status = drv_internal_flash_read(zp_mfg_slot_offset(slot),
                                           raw_record.raw,
                                           (uint32_t)sizeof(raw_record.raw));
#if ZY100_BUILD_PRODUCTION
    if (boot_diagnostic)
    {
        DBG_DIRECT("[MFG_DIAG] slot=%u offset=0x%08lX read_status=%u",
                   slot, (unsigned long)zp_mfg_slot_offset(slot),
                   (unsigned int)flash_status);
    }
#else
    (void)boot_diagnostic;
#endif
    if (flash_status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return ZP_MFG_STORE_STATUS_FLASH_ERROR;
    }

    if (raw_record.v5.version == ZP_MFG_RECORD_VERSION_V5)
    {
        if (zp_mfg_raw_v5_valid(&raw_record.v5))
        {
            zp_mfg_migrate_v5_to_canonical(&raw_record.v5, &out->record);
            out->raw_version = ZP_MFG_RECORD_VERSION_V5;
            out->valid = true;
        }
    }
    else if (raw_record.v4.version == ZP_MFG_RECORD_VERSION_V4)
    {
        if ((raw_record.v4.magic == ZP_MFG_RECORD_MAGIC) &&
            (raw_record.v4.record_bytes == sizeof(raw_record.v4)) &&
            (raw_record.v4.crc32 == zp_mfg_record_v4_crc(&raw_record.v4)))
        {
            memcpy(&out->record, &raw_record.v4, sizeof(out->record));
            out->raw_version = ZP_MFG_RECORD_VERSION_V4;
            out->valid = true;
        }
    }
    else if (raw_record.v3.version == ZP_MFG_RECORD_VERSION_V3)
    {
        if (zp_mfg_raw_v3_valid(&raw_record.v3))
        {
            zp_mfg_migrate_v3_to_canonical(&raw_record.v3, &out->record);
            out->raw_version = ZP_MFG_RECORD_VERSION_V3;
            out->valid = true;
        }
    }

#if ZY100_BUILD_PRODUCTION
    if (boot_diagnostic)
    {
        zp_mfg_log_boot_slot(slot, &raw_record, out->valid);
    }
#endif
    return out->valid ? ZP_MFG_STORE_STATUS_OK :
           ZP_MFG_STORE_STATUS_NOT_FOUND;
}

static zp_mfg_store_status_t zp_mfg_read_slot(uint8_t slot,
                                              zp_mfg_slot_read_t *out)
{
    return zp_mfg_read_slot_internal(slot, out, false);
}

static zp_mfg_store_status_t zp_mfg_store_load_internal(zp_mfg_record_t *out,
                                                       bool boot_diagnostic,
                                                       uint8_t *selected_slot)
{
    zp_mfg_slot_read_t slot;
    zp_mfg_store_status_t status;
    bool found = false;
    bool read_error = false;
    uint8_t index;
    uint8_t selected = 0U;

    if (out == NULL)
    {
        return ZP_MFG_STORE_STATUS_INVALID_PARAM;
    }
    /* Reuse one slot buffer: boot runs on the small pre-scheduler stack. */
    memset(out, 0, sizeof(*out));
    for (index = 0U; index < 2U; index++)
    {
        status = zp_mfg_read_slot_internal(index, &slot, boot_diagnostic);
        if (status == ZP_MFG_STORE_STATUS_FLASH_ERROR)
        {
            read_error = true;
        }
        if (slot.valid && (!found ||
            ((int32_t)(slot.record.generation - out->generation) > 0)))
        {
            memcpy(out, &slot.record, sizeof(*out));
            found = true;
            selected = index;
        }
    }
    if (read_error)
    {
        memset(out, 0, sizeof(*out));
        return ZP_MFG_STORE_STATUS_FLASH_ERROR;
    }
    if (!found)
    {
        return ZP_MFG_STORE_STATUS_NOT_FOUND;
    }
    if (selected_slot != NULL)
    {
        *selected_slot = selected;
    }
#if ZY100_BUILD_PRODUCTION
    if (boot_diagnostic)
    {
        DBG_DIRECT("[MFG_DIAG] selected_slot=%u gen=%lu policy=crc_then_generation",
                   selected, (unsigned long)out->generation);
    }
#endif
    return ZP_MFG_STORE_STATUS_OK;
}

zp_mfg_store_status_t zp_mfg_store_load(zp_mfg_record_t *out)
{
    return zp_mfg_store_load_internal(out, false, NULL);
}

zp_mfg_store_status_t zp_mfg_store_load_boot_diagnostic(zp_mfg_record_t *out)
{
    return zp_mfg_store_load_internal(out, true, NULL);
}

#if ZY100_PRODUCTION_MFG_TEST_COMPAT_ENABLE
static zp_mfg_store_status_t zp_mfg_write_test_masks(
    uint8_t source_slot, zp_mfg_record_t *current,
    uint32_t required_mask, uint32_t passed_mask)
{
    zp_mfg_raw_record_t raw;
    uint32_t verify_words[16];
    const uint32_t bytes = current->record_bytes;
    /* All supported raw formats have a trailing CRC, checked by load. */
    const uint32_t crc_offset = bytes - sizeof(uint32_t);
    const uint8_t target_slot = source_slot ^ 1U;
    uint32_t crc;
    uint32_t offset;
    uint32_t length;

    if (drv_internal_flash_read(zp_mfg_slot_offset(source_slot), raw.raw,
                                bytes) != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return ZP_MFG_STORE_STATUS_FLASH_ERROR;
    }
    memcpy(&crc, &raw.raw[crc_offset], sizeof(crc));
    if ((crc != current->crc32) ||
        (zp_mfg_crc_with_zero_field(raw.raw, bytes, crc_offset) != crc))
    {
        return ZP_MFG_STORE_STATUS_VERIFY_FAILED;
    }
    raw.v3.required_test_mask = required_mask;
    raw.v3.passed_test_mask = passed_mask;
    raw.v3.generation++;
    crc = zp_mfg_crc_with_zero_field(raw.raw, bytes, crc_offset);
    memcpy(&raw.raw[crc_offset], &crc, sizeof(crc));
    DBG_DIRECT("[MFG_COMPAT] source_slot=%u target_slot=%u gen=%lu->%lu",
               source_slot, target_slot, (unsigned long)current->generation,
               (unsigned long)raw.v3.generation);
    if (drv_internal_flash_erase_sector(zp_mfg_slot_offset(target_slot)) !=
        DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return ZP_MFG_STORE_STATUS_FLASH_ERROR;
    }
    if (drv_internal_flash_write(zp_mfg_slot_offset(target_slot), raw.raw,
                                 bytes) != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return ZP_MFG_STORE_STATUS_FLASH_ERROR;
    }
    /* Small aligned chunks verify every byte without another 676-byte frame. */
    for (offset = 0U; offset < bytes; offset += length)
    {
        length = bytes - offset;
        if (length > sizeof(verify_words))
        {
            length = sizeof(verify_words);
        }
        if ((drv_internal_flash_read(zp_mfg_slot_offset(target_slot) + offset,
                                      (uint8_t *)verify_words, length) !=
             DRV_INTERNAL_FLASH_STATUS_OK) ||
            (memcmp(verify_words, &raw.raw[offset], length) != 0))
        {
            return ZP_MFG_STORE_STATUS_VERIFY_FAILED;
        }
    }
    current->required_test_mask = required_mask;
    current->passed_test_mask = passed_mask;
    current->generation = raw.v3.generation;
    current->crc32 = crc;
    return ZP_MFG_STORE_STATUS_OK;
}

zp_mfg_store_status_t zp_mfg_store_update_test_masks(
    zp_mfg_record_t *record, uint32_t required_mask, uint32_t passed_mask)
{
    zp_mfg_record_t current;
    zp_mfg_store_status_t status;
    uint8_t source_slot;
    uint8_t selected_slot;

    if ((record == NULL) || !zp_mfg_store_record_locked(record))
    {
        return ZP_MFG_STORE_STATUS_INVALID_PARAM;
    }
    status = zp_mfg_store_load_internal(&current, false, &source_slot);
    if (status != ZP_MFG_STORE_STATUS_OK)
    {
        return status;
    }
    if (memcmp(&current, record, sizeof(current)) != 0)
    {
        return ZP_MFG_STORE_STATUS_VERIFY_FAILED;
    }
    if ((current.required_test_mask == required_mask) &&
        (current.passed_test_mask == passed_mask))
    {
        return ZP_MFG_STORE_STATUS_OK;
    }
    status = zp_mfg_write_test_masks(source_slot, &current,
                                     required_mask, passed_mask);
    if (status != ZP_MFG_STORE_STATUS_OK)
    {
        return status;
    }
    status = zp_mfg_store_load_internal(record, false, &selected_slot);
    if (status != ZP_MFG_STORE_STATUS_OK)
    {
        return status;
    }
    return ((selected_slot == (source_slot ^ 1U)) &&
            (memcmp(record, &current, sizeof(current)) == 0)) ?
           ZP_MFG_STORE_STATUS_OK : ZP_MFG_STORE_STATUS_VERIFY_FAILED;
}
#endif

/* Isolated test-only transaction; never changes masks or record format. */
#if ZY100_PRODUCTION_TEST_ST_TO_CG_ENABLE
static zp_mfg_store_status_t zp_mfg_write_test_channel(
    uint8_t source_slot, zp_mfg_record_t *current)
{
    zp_mfg_raw_record_t raw;
    uint32_t verify_words[16];
    const uint32_t bytes = current->record_bytes;
    /* All supported raw formats have a trailing CRC, checked by load. */
    const uint32_t crc_offset = bytes - sizeof(uint32_t);
    const uint8_t target_slot = source_slot ^ 1U;
    uint32_t crc;
    uint32_t offset;
    uint32_t length;

    if (drv_internal_flash_read(zp_mfg_slot_offset(source_slot), raw.raw,
                                bytes) != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return ZP_MFG_STORE_STATUS_FLASH_ERROR;
    }
    memcpy(&crc, &raw.raw[crc_offset], sizeof(crc));
    if ((crc != current->crc32) ||
        (zp_mfg_crc_with_zero_field(raw.raw, bytes, crc_offset) != crc))
    {
        return ZP_MFG_STORE_STATUS_VERIFY_FAILED;
    }
    memcpy(raw.v3.channel_code, "CG", sizeof(raw.v3.channel_code));
    raw.v3.generation++;
    crc = zp_mfg_crc_with_zero_field(raw.raw, bytes, crc_offset);
    memcpy(&raw.raw[crc_offset], &crc, sizeof(crc));
    DBG_DIRECT("[MFG_CHANNEL_TEST] ST->CG source_slot=%u target_slot=%u gen=%lu->%lu",
               source_slot, target_slot, (unsigned long)current->generation,
               (unsigned long)raw.v3.generation);
    if (drv_internal_flash_erase_sector(zp_mfg_slot_offset(target_slot)) !=
        DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return ZP_MFG_STORE_STATUS_FLASH_ERROR;
    }
    if (drv_internal_flash_write(zp_mfg_slot_offset(target_slot), raw.raw,
                                 bytes) != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return ZP_MFG_STORE_STATUS_FLASH_ERROR;
    }
    /* Small aligned chunks verify every byte without another 676-byte frame. */
    for (offset = 0U; offset < bytes; offset += length)
    {
        length = bytes - offset;
        if (length > sizeof(verify_words))
        {
            length = sizeof(verify_words);
        }
        if ((drv_internal_flash_read(zp_mfg_slot_offset(target_slot) + offset,
                                      (uint8_t *)verify_words, length) !=
             DRV_INTERNAL_FLASH_STATUS_OK) ||
            (memcmp(verify_words, &raw.raw[offset], length) != 0))
        {
            return ZP_MFG_STORE_STATUS_VERIFY_FAILED;
        }
    }
    memcpy(current->channel_code, "CG", sizeof(current->channel_code));
    current->generation = raw.v3.generation;
    current->crc32 = crc;
    return ZP_MFG_STORE_STATUS_OK;
}

zp_mfg_store_status_t zp_mfg_store_test_st_to_cg(
    zp_mfg_record_t *record)
{
    zp_mfg_record_t current;
    zp_mfg_store_status_t status;
    uint8_t source_slot;
    uint8_t selected_slot;

    if ((record == NULL) || !zp_mfg_store_record_locked(record) ||
        (memcmp(record->channel_code, "ST", sizeof(record->channel_code)) != 0))
    {
        return ZP_MFG_STORE_STATUS_INVALID_PARAM;
    }
    status = zp_mfg_store_load_internal(&current, false, &source_slot);
    if (status != ZP_MFG_STORE_STATUS_OK)
    {
        return status;
    }
    if (memcmp(&current, record, sizeof(current)) != 0)
    {
        return ZP_MFG_STORE_STATUS_VERIFY_FAILED;
    }
    status = zp_mfg_write_test_channel(source_slot, &current);
    if (status != ZP_MFG_STORE_STATUS_OK)
    {
        return status;
    }
    status = zp_mfg_store_load_internal(record, false, &selected_slot);
    if (status != ZP_MFG_STORE_STATUS_OK)
    {
        return status;
    }
    return ((selected_slot == (source_slot ^ 1U)) &&
            (memcmp(record, &current, sizeof(current)) == 0)) ?
           ZP_MFG_STORE_STATUS_OK : ZP_MFG_STORE_STATUS_VERIFY_FAILED;
}
#endif

static uint8_t zp_mfg_select_write_slot(void)
{
    zp_mfg_slot_read_t slot0;
    zp_mfg_slot_read_t slot1;

    (void)zp_mfg_read_slot(0U, &slot0);
    (void)zp_mfg_read_slot(1U, &slot1);
    if (!slot0.valid)
    {
        return 0U;
    }
    if (!slot1.valid)
    {
        return 1U;
    }
    return (((int32_t)(slot1.record.generation -
                       slot0.record.generation)) > 0) ? 0U : 1U;
}

zp_mfg_store_status_t zp_mfg_store_write_locked_once(
    zp_mfg_record_t *record)
{
    zp_mfg_record_t current;
    zp_mfg_record_v3_t write_record;
    zp_mfg_record_v3_t verify_record;
    zp_mfg_store_status_t load_status;
    drv_internal_flash_status_t flash_status;
    uint8_t slot;
    uint32_t generation = 1UL;
    const uint32_t common_bytes =
        (uint32_t)offsetof(zp_mfg_record_v3_t, crc32);

    if (record == NULL)
    {
        return ZP_MFG_STORE_STATUS_INVALID_PARAM;
    }
    memset(&current, 0, sizeof(current));
    load_status = zp_mfg_store_load(&current);
    if (load_status == ZP_MFG_STORE_STATUS_OK)
    {
        if (zp_mfg_store_record_locked(&current))
        {
            return ZP_MFG_STORE_STATUS_ALREADY_LOCKED;
        }
        generation = current.generation + 1UL;
    }
    else if (load_status == ZP_MFG_STORE_STATUS_FLASH_ERROR)
    {
        return load_status;
    }

    memset(&write_record, 0, sizeof(write_record));
    memcpy(&write_record, record, common_bytes);
    write_record.magic = ZP_MFG_RECORD_MAGIC;
    write_record.version = ZP_MFG_RECORD_VERSION_V3;
    write_record.record_bytes = (uint16_t)sizeof(write_record);
    write_record.generation = generation;
    write_record.flags |= ZP_MFG_RECORD_FLAG_LOCKED;
    if (write_record.factory_cleanup_pass != 0U)
    {
        write_record.flags |= ZP_MFG_RECORD_FLAG_FACTORY_CLEANUP;
    }
    if (write_record.legacy_cleanup_suppressed != 0U)
    {
        write_record.flags |= ZP_MFG_RECORD_FLAG_LEGACY_SUPPRESS;
    }
    write_record.crc32 = zp_mfg_record_v3_crc(&write_record);

    slot = zp_mfg_select_write_slot();
    flash_status = drv_internal_flash_erase_sector(zp_mfg_slot_offset(slot));
    if (flash_status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return ZP_MFG_STORE_STATUS_FLASH_ERROR;
    }
    flash_status = drv_internal_flash_write(zp_mfg_slot_offset(slot),
                                            (const uint8_t *)&write_record,
                                            (uint32_t)sizeof(write_record));
    if (flash_status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return ZP_MFG_STORE_STATUS_FLASH_ERROR;
    }

    memset(&verify_record, 0, sizeof(verify_record));
    flash_status = drv_internal_flash_read(zp_mfg_slot_offset(slot),
                                           (uint8_t *)&verify_record,
                                           (uint32_t)sizeof(verify_record));
    if ((flash_status != DRV_INTERNAL_FLASH_STATUS_OK) ||
        (memcmp(&verify_record, &write_record, sizeof(write_record)) != 0) ||
        !zp_mfg_raw_v3_valid(&verify_record) ||
        ((verify_record.flags & ZP_MFG_RECORD_FLAG_LOCKED) == 0UL))
    {
        return ZP_MFG_STORE_STATUS_VERIFY_FAILED;
    }
    return ZP_MFG_STORE_STATUS_OK;
}

static zp_mfg_store_status_t zp_mfg_store_write_locked_once_v5_common(
    zp_mfg_record_t *record,
    const uint8_t *calibration,
    uint16_t calibration_length,
    bool allow_empty_calibration)
{
    zp_mfg_record_t current;
    zp_mfg_record_v5_t write_record;
    zp_mfg_record_v5_t verify_record;
    zp_mfg_store_status_t load_status;
    drv_internal_flash_status_t flash_status;
    uint8_t slot;
    uint32_t generation = 1UL;
    const uint32_t common_bytes =
        (uint32_t)offsetof(zp_mfg_record_v3_t, crc32);

    if ((record == NULL) ||
        (!allow_empty_calibration && (calibration == NULL)) ||
        ((calibration_length != 0U) && (calibration == NULL)) ||
        ((!allow_empty_calibration && (calibration_length == 0U))) ||
        (calibration_length > ZP_MFG_CALIBRATION_BLOB_MAX_BYTES))
    {
        return ZP_MFG_STORE_STATUS_INVALID_PARAM;
    }
    memset(&current, 0, sizeof(current));
    load_status = zp_mfg_store_load(&current);
    if (load_status == ZP_MFG_STORE_STATUS_OK)
    {
        if (zp_mfg_store_record_locked(&current))
        {
            return ZP_MFG_STORE_STATUS_ALREADY_LOCKED;
        }
        generation = current.generation + 1UL;
    }
    else if (load_status == ZP_MFG_STORE_STATUS_FLASH_ERROR)
    {
        return load_status;
    }

    memset(&write_record, 0, sizeof(write_record));
    memcpy(&write_record, record, common_bytes);
    memcpy(&write_record.imu_cal, &record->imu_cal,
           sizeof(write_record.imu_cal));
    write_record.magic = ZP_MFG_RECORD_MAGIC;
    write_record.version = ZP_MFG_RECORD_VERSION_V5;
    write_record.record_bytes = (uint16_t)sizeof(write_record);
    write_record.generation = generation;
    write_record.flags |= ZP_MFG_RECORD_FLAG_LOCKED;
    if (write_record.factory_cleanup_pass != 0U)
    {
        write_record.flags |= ZP_MFG_RECORD_FLAG_FACTORY_CLEANUP;
    }
    if (write_record.legacy_cleanup_suppressed != 0U)
    {
        write_record.flags |= ZP_MFG_RECORD_FLAG_LEGACY_SUPPRESS;
    }
    write_record.reserved_flags |= ZP_MFG_RECORD_RESERVED_LEGACY_NO_IMU_CAL;
    write_record.calibration_blob_bytes = calibration_length;
    if (calibration_length != 0U)
    {
        memcpy(write_record.calibration_blob, calibration, calibration_length);
    }
    write_record.crc32 = zp_mfg_record_v5_crc(&write_record);

    slot = zp_mfg_select_write_slot();
    ZP_MFG_FACTORY_LOG("[FACTORY_MFG] mfg_slot_target slot=%u generation=%lu",
                       slot, (unsigned long)generation);
    flash_status = drv_internal_flash_erase_sector(zp_mfg_slot_offset(slot));
    if (flash_status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return ZP_MFG_STORE_STATUS_FLASH_ERROR;
    }
    flash_status = drv_internal_flash_write(zp_mfg_slot_offset(slot),
                                            (const uint8_t *)&write_record,
                                            (uint32_t)sizeof(write_record));
    if (flash_status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return ZP_MFG_STORE_STATUS_FLASH_ERROR;
    }
    ZP_MFG_FACTORY_LOG("[FACTORY_MFG] mfg_slot_write_ok slot=%u bytes=%lu",
                       slot, (unsigned long)sizeof(write_record));
    memset(&verify_record, 0, sizeof(verify_record));
    flash_status = drv_internal_flash_read(zp_mfg_slot_offset(slot),
                                           (uint8_t *)&verify_record,
                                           (uint32_t)sizeof(verify_record));
    if ((flash_status != DRV_INTERNAL_FLASH_STATUS_OK) ||
        (memcmp(&verify_record, &write_record, sizeof(write_record)) != 0) ||
        !zp_mfg_raw_v5_valid(&verify_record) ||
        ((verify_record.flags & ZP_MFG_RECORD_FLAG_LOCKED) == 0UL))
    {
        return ZP_MFG_STORE_STATUS_VERIFY_FAILED;
    }
    ZP_MFG_FACTORY_LOG("[FACTORY_MFG] mfg_slot_readback_ok slot=%u crc=0x%08lX",
                       slot, (unsigned long)verify_record.crc32);
    return ZP_MFG_STORE_STATUS_OK;
}

zp_mfg_store_status_t zp_mfg_store_write_locked_once_v5(
    zp_mfg_record_t *record,
    const uint8_t *calibration,
    uint16_t calibration_length)
{
    return zp_mfg_store_write_locked_once_v5_common(
        record, calibration, calibration_length, false);
}

zp_mfg_store_status_t zp_mfg_store_write_locked_once_v5_no_calibration(
    zp_mfg_record_t *record)
{
    return zp_mfg_store_write_locked_once_v5_common(
        record, NULL, 0U, true);
}

static const zp_mfg_slot_read_t *zp_mfg_select_current(
    const zp_mfg_slot_read_t *slot0,
    const zp_mfg_slot_read_t *slot1)
{
    if (slot0->valid && slot1->valid)
    {
        return (((int32_t)(slot1->record.generation -
                           slot0->record.generation)) > 0) ? slot1 : slot0;
    }
    if (slot0->valid)
    {
        return slot0;
    }
    return slot1->valid ? slot1 : NULL;
}

zp_mfg_store_status_t zp_mfg_store_calibration_load(
    uint8_t out[ZP_MFG_CALIBRATION_BLOB_MAX_BYTES],
    uint16_t *length_out)
{
    zp_mfg_slot_read_t slot0;
    zp_mfg_slot_read_t slot1;
    const zp_mfg_slot_read_t *selected;
    zp_mfg_raw_record_t raw;
    drv_internal_flash_status_t flash_status;

    if ((out == NULL) || (length_out == NULL))
    {
        return ZP_MFG_STORE_STATUS_INVALID_PARAM;
    }
    memset(out, 0, ZP_MFG_CALIBRATION_BLOB_MAX_BYTES);
    *length_out = 0U;
    (void)zp_mfg_read_slot(0U, &slot0);
    (void)zp_mfg_read_slot(1U, &slot1);
    selected = zp_mfg_select_current(&slot0, &slot1);
    if ((selected == NULL) ||
        (selected->raw_version != ZP_MFG_RECORD_VERSION_V5))
    {
        return ZP_MFG_STORE_STATUS_CALIBRATION_NOT_FOUND;
    }
    memset(&raw, 0, sizeof(raw));
    flash_status = drv_internal_flash_read(
                       zp_mfg_slot_offset(selected->slot), raw.raw,
                       (uint32_t)sizeof(raw.raw));
    if ((flash_status != DRV_INTERNAL_FLASH_STATUS_OK) ||
        !zp_mfg_raw_v5_valid(&raw.v5))
    {
        return ZP_MFG_STORE_STATUS_FLASH_ERROR;
    }
    if (raw.v5.calibration_blob_bytes == 0U)
    {
        return ZP_MFG_STORE_STATUS_CALIBRATION_NOT_FOUND;
    }
    memcpy(out, raw.v5.calibration_blob,
           raw.v5.calibration_blob_bytes);
    *length_out = raw.v5.calibration_blob_bytes;
    return ZP_MFG_STORE_STATUS_OK;
}

static zp_mfg_store_status_t zp_mfg_store_calibration_update(
    const uint8_t *record,
    uint16_t length)
{
    zp_mfg_record_t current;
    zp_mfg_record_v5_t write_record;
    drv_internal_flash_status_t flash_status;
    zp_mfg_store_status_t load_status;
    uint8_t target_slot;
    uint32_t expected_generation;
    uint32_t expected_crc;
    const uint32_t common_bytes =
        (uint32_t)offsetof(zp_mfg_record_v3_t, crc32);

    if (((record == NULL) && (length != 0U)) ||
        (length > ZP_MFG_CALIBRATION_BLOB_MAX_BYTES))
    {
        return ZP_MFG_STORE_STATUS_INVALID_PARAM;
    }
    memset(&current, 0, sizeof(current));
    load_status = zp_mfg_store_load(&current);
    if (load_status != ZP_MFG_STORE_STATUS_OK)
    {
        return load_status;
    }
    if (!zp_mfg_store_record_locked(&current))
    {
        return ZP_MFG_STORE_STATUS_CALIBRATION_INVALID;
    }
    target_slot = zp_mfg_select_write_slot();

    memset(&write_record, 0, sizeof(write_record));
    memcpy(&write_record, &current, common_bytes);
    memcpy(&write_record.imu_cal, &current.imu_cal,
           sizeof(write_record.imu_cal));
    if ((current.version == ZP_MFG_RECORD_VERSION_V3) ||
        zp_mfg_store_legacy_imu_cal_absent(&current))
    {
        write_record.reserved_flags |=
            ZP_MFG_RECORD_RESERVED_LEGACY_NO_IMU_CAL;
    }
    else
    {
        write_record.reserved_flags &=
            (uint8_t)~ZP_MFG_RECORD_RESERVED_LEGACY_NO_IMU_CAL;
    }
    write_record.version = ZP_MFG_RECORD_VERSION_V5;
    write_record.record_bytes = (uint16_t)sizeof(write_record);
    write_record.generation = current.generation + 1UL;
    write_record.calibration_blob_bytes = length;
    if (length != 0U) memcpy(write_record.calibration_blob, record, length);
    write_record.crc32 = zp_mfg_record_v5_crc(&write_record);
    expected_generation = write_record.generation;
    expected_crc = write_record.crc32;

    flash_status = drv_internal_flash_erase_sector(
                       zp_mfg_slot_offset(target_slot));
    if (flash_status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return ZP_MFG_STORE_STATUS_FLASH_ERROR;
    }
    flash_status = drv_internal_flash_write(
                       zp_mfg_slot_offset(target_slot),
                       (const uint8_t *)&write_record,
                       (uint32_t)sizeof(write_record));
    if (flash_status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return ZP_MFG_STORE_STATUS_FLASH_ERROR;
    }
    memset(&write_record, 0, sizeof(write_record));
    flash_status = drv_internal_flash_read(
                       zp_mfg_slot_offset(target_slot),
                       (uint8_t *)&write_record,
                       (uint32_t)sizeof(write_record));
    if ((flash_status != DRV_INTERNAL_FLASH_STATUS_OK) ||
        !zp_mfg_raw_v5_valid(&write_record) ||
        (write_record.generation != expected_generation) ||
        (write_record.crc32 != expected_crc) ||
        (write_record.calibration_blob_bytes != length) ||
        ((length != 0U) && (memcmp(write_record.calibration_blob, record, length) != 0)))
    {
        return ZP_MFG_STORE_STATUS_VERIFY_FAILED;
    }
    return ZP_MFG_STORE_STATUS_OK;
}

zp_mfg_store_status_t zp_mfg_store_calibration_save(const uint8_t *record, uint16_t length)
{
    if (record == NULL || length == 0U) return ZP_MFG_STORE_STATUS_INVALID_PARAM;
    return zp_mfg_store_calibration_update(record, length);
}

zp_mfg_store_status_t zp_mfg_store_calibration_clear(void)
{
    zp_mfg_store_status_t status = zp_mfg_store_calibration_update(NULL, 0U);
    /* Commit an empty calibration to both copies, retaining the locked identity. */
    if (status != ZP_MFG_STORE_STATUS_OK) return status;
    return zp_mfg_store_calibration_update(NULL, 0U);
}

const char *zp_mfg_store_status_name(zp_mfg_store_status_t status)
{
    switch (status)
    {
    case ZP_MFG_STORE_STATUS_OK:
        return "OK";
    case ZP_MFG_STORE_STATUS_NOT_FOUND:
        return "NOT_FOUND";
    case ZP_MFG_STORE_STATUS_ALREADY_LOCKED:
        return "ALREADY_LOCKED";
    case ZP_MFG_STORE_STATUS_INVALID_PARAM:
        return "INVALID_PARAM";
    case ZP_MFG_STORE_STATUS_CALIBRATION_INVALID:
        return "CALIBRATION_INVALID";
    case ZP_MFG_STORE_STATUS_CALIBRATION_NOT_FOUND:
        return "CALIBRATION_NOT_FOUND";
    case ZP_MFG_STORE_STATUS_FLASH_ERROR:
        return "FLASH_ERROR";
    case ZP_MFG_STORE_STATUS_VERIFY_FAILED:
        return "VERIFY_FAILED";
    default:
        return "UNKNOWN";
    }
}
