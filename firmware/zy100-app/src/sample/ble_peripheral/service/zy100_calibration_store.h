#ifndef ZY100_CALIBRATION_STORE_H
#define ZY100_CALIBRATION_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "zy100_calibration_protocol.h"

typedef enum
{
    ZY100_CAL_STORE_OK = 0U,
    ZY100_CAL_STORE_NOT_FOUND,
    ZY100_CAL_STORE_INVALID,
    ZY100_CAL_STORE_READ_ERROR,
    ZY100_CAL_STORE_WRITE_ERROR,
    ZY100_CAL_STORE_VERIFY_ERROR,
} zy100_cal_store_status_t;

zy100_cal_store_status_t zy100_cal_store_load(
    uint8_t out[ZY100_CAL_RECORD_MAX_BYTES],
    zy100_cal_record_info_t *info);
zy100_cal_store_status_t zy100_cal_store_save(
    const uint8_t *record,
    uint16_t len,
    zy100_cal_record_info_t *info);
zy100_cal_store_status_t zy100_cal_store_stage_load(
    uint8_t out[ZY100_CAL_RECORD_MAX_BYTES],
    zy100_cal_record_info_t *info);
zy100_cal_store_status_t zy100_cal_store_stage_save(
    const uint8_t *record,
    uint16_t len,
    zy100_cal_record_info_t *info);
zy100_cal_store_status_t zy100_cal_store_clear(void);
const char *zy100_cal_store_status_name(zy100_cal_store_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_CALIBRATION_STORE_H */
