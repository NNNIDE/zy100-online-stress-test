#ifndef ZP_MFG_STORE_H
#define ZP_MFG_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "zp_mfg_record.h"

typedef enum
{
    ZP_MFG_STORE_STATUS_OK = 0U,
    ZP_MFG_STORE_STATUS_NOT_FOUND,
    ZP_MFG_STORE_STATUS_ALREADY_LOCKED,
    ZP_MFG_STORE_STATUS_INVALID_PARAM,
    ZP_MFG_STORE_STATUS_CALIBRATION_INVALID,
    ZP_MFG_STORE_STATUS_CALIBRATION_NOT_FOUND,
    ZP_MFG_STORE_STATUS_FLASH_ERROR,
    ZP_MFG_STORE_STATUS_VERIFY_FAILED,
} zp_mfg_store_status_t;

zp_mfg_store_status_t zp_mfg_store_load(zp_mfg_record_t *out);
/* Boot-only read diagnostics; identical validation/selection, never writes. */
zp_mfg_store_status_t zp_mfg_store_load_boot_diagnostic(zp_mfg_record_t *out);
/* Boot-only, single-writer transaction. The caller validates mask policy first.
 * Rechecks the selected snapshot, preserves the original raw format and data,
 * writes the other slot, verifies all bytes and reloads the selected record.
 * On failure the caller must not use record to authorize startup. */
zp_mfg_store_status_t zp_mfg_store_update_test_masks(
    zp_mfg_record_t *record, uint32_t required_mask, uint32_t passed_mask);
/* Production test only. Caller must validate boot semantics before calling.
 * Only exact ST records may be rewritten; uses the alternate slot, preserves
 * all other raw fields, and verifies readback and final slot selection. */
zp_mfg_store_status_t zp_mfg_store_test_st_to_cg(zp_mfg_record_t *record);
bool zp_mfg_store_record_locked(const zp_mfg_record_t *record);
bool zp_mfg_store_record_valid(const zp_mfg_record_t *record);
bool zp_mfg_store_imu_cal_valid(
    const zp_mfg_imu_single_pose_cal_v1_t *calibration);
bool zp_mfg_store_legacy_imu_cal_absent(const zp_mfg_record_t *record);
void zp_mfg_store_imu_cal_finalize(
    zp_mfg_imu_single_pose_cal_v1_t *calibration);
zp_mfg_store_status_t zp_mfg_store_write_locked_once(
    zp_mfg_record_t *record);
zp_mfg_store_status_t zp_mfg_store_write_locked_once_v5(
    zp_mfg_record_t *record,
    const uint8_t *calibration,
    uint16_t calibration_length);
/* Factory SDK-only ADC path: write a valid v5 record without a CAL1 blob. */
zp_mfg_store_status_t zp_mfg_store_write_locked_once_v5_no_calibration(
    zp_mfg_record_t *record);
zp_mfg_store_status_t zp_mfg_store_calibration_load(
    uint8_t out[ZP_MFG_CALIBRATION_BLOB_MAX_BYTES],
    uint16_t *length_out);
zp_mfg_store_status_t zp_mfg_store_calibration_save(
    const uint8_t *record,
    uint16_t length);
zp_mfg_store_status_t zp_mfg_store_calibration_clear(void);
const char *zp_mfg_store_status_name(zp_mfg_store_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* ZP_MFG_STORE_H */
