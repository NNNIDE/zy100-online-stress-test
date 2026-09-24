#ifndef ZY100_CALIBRATION_PROTOCOL_H
#define ZY100_CALIBRATION_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define ZY100_CAL_RECORD_MAX_BYTES          256U
#define ZY100_CAL_RECORD_MAGIC              0x314C4143UL /* "CAL1" LE */
#define ZY100_CAL_RECORD_VERSION            1U
#define ZY100_CAL_RECORD_HEADER_BYTES       30U
#define ZY100_CAL_RECORD_CRC_OFFSET         26U
#define ZY100_CAL_TLV_HEADER_BYTES          2U

#define ZY100_CAL_VALID_IMU_GYRO            0x00000001UL
#define ZY100_CAL_VALID_IMU_ACCEL           0x00000002UL
#define ZY100_CAL_VALID_IMU_INSTALL         0x00000004UL
#define ZY100_CAL_VALID_MAG_MODEL           0x00000008UL
#define ZY100_CAL_VALID_MAG_INSTALL         0x00000010UL
#define ZY100_CAL_VALID_BATTERY_GAIN        0x00000020UL
#define ZY100_CAL_MAG_MODEL_AXIS_ALIGNED    0x00010000UL
#define ZY100_CAL_MAG_MODEL_HARD_IRON_ONLY  0x00020000UL
#define ZY100_CAL_MAG_MODEL_FLAG_MASK       0x00030000UL

#define ZY100_CAL_TLV_IMU_GYRO_MODEL        0x01U
#define ZY100_CAL_TLV_IMU_ACCEL_MODEL       0x02U
#define ZY100_CAL_TLV_IMU_INSTALL_MATRIX    0x03U
#define ZY100_CAL_TLV_MAG_MODEL             0x04U
#define ZY100_CAL_TLV_MAG_INSTALL_MATRIX    0x05U
#define ZY100_CAL_TLV_QUALITY_META          0x06U
#define ZY100_CAL_TLV_BATTERY_GAIN_V1       0x07U

#define ZY100_CAL_TLV_BIAS_MATRIX_BYTES     48U
#define ZY100_CAL_TLV_MATRIX3_BYTES         36U
#define ZY100_CAL_TLV_MAG_MODEL_BYTES       48U
#define ZY100_CAL_TLV_BATTERY_GAIN_V1_BYTES 28U
#define ZY100_CAL_BATTERY_GAIN_VERSION       1U

typedef struct
{
    uint8_t version;
    uint8_t sample_count;
    uint32_t workflow_token;
    uint8_t bt_address[6];
    uint16_t reference_mv;
    uint16_t uncalibrated_mv;
    uint32_t gain_q20;
    int32_t precal_delta_mv;
} zy100_cal_battery_gain_v1_t;

#define ZY100_CAL_RX_MAGIC                  0xC3U
#define ZY100_CAL_TX_MAGIC                  0xD3U
#define ZY100_CAL_PROTOCOL_VERSION          1U
#define ZY100_CAL_RX_HEADER_BYTES           14U
#define ZY100_CAL_TX_HEADER_BYTES           16U
#define ZY100_CAL_INFO_BYTES                20U
#define ZY100_CAL_STATUS_BYTES              16U
#define ZY100_CAL_CACHE_CONFIRM_BYTES       12U
#define ZY100_CAL_MAG_DIAG_MAGIC            0x3144474DUL /* "MGD1" LE */
#define ZY100_CAL_MAG_DIAG_VERSION          1U
#define ZY100_CAL_MAG_DIAG_BYTES            68U
#define ZY100_CAL_MAG_DIAG_CRC_OFFSET       64U
#define ZY100_CAL_DIAG_STATUS_NOT_ATTEMPTED 0xFFU

typedef enum
{
    ZY100_CAL_RX_BEGIN_WRITE = 1U,
    ZY100_CAL_RX_CHUNK = 2U,
    ZY100_CAL_RX_COMMIT = 3U,
    ZY100_CAL_RX_ACK = 4U,
    ZY100_CAL_RX_REQUEST = 5U,
    ZY100_CAL_RX_ABORT = 6U,
    ZY100_CAL_RX_START_MAG_CAL = 0x10U,
    ZY100_CAL_RX_REQUEST_DIAGNOSTICS = 0x11U,
    ZY100_CAL_RX_START_MAG_CAL_WITH_POINTS = 0x12U,
    ZY100_CAL_RX_CONFIRM_INFO = 0x13U,
    ZY100_CAL_RX_CONFIRM_CACHED_RECORD = 0x14U,
} zy100_cal_rx_opcode_t;

typedef enum
{
    ZY100_CAL_TX_START = 1U,
    ZY100_CAL_TX_DATA = 2U,
    ZY100_CAL_TX_END = 3U,
    ZY100_CAL_TX_DIAG_START = 4U,
    ZY100_CAL_TX_DIAG_DATA = 5U,
    ZY100_CAL_TX_DIAG_END = 6U,
} zy100_cal_tx_type_t;

typedef enum
{
    ZY100_CAL_DIAG_COMPLETION_NONE = 0U,
    ZY100_CAL_DIAG_COMPLETION_SUCCESS = 1U,
    ZY100_CAL_DIAG_COMPLETION_LIMITED = 2U,
    ZY100_CAL_DIAG_COMPLETION_FAILED = 3U,
    ZY100_CAL_DIAG_COMPLETION_NO_UPDATE = 4U,
} zy100_cal_diag_completion_t;

typedef enum
{
    ZY100_CAL_STATUS_NO_DATA = 0U,
    ZY100_CAL_STATUS_READY = 1U,
    ZY100_CAL_STATUS_SENDING = 2U,
    ZY100_CAL_STATUS_WAIT_ACK = 3U,
    ZY100_CAL_STATUS_ACKED = 4U,
    ZY100_CAL_STATUS_WRITE_RECEIVING = 5U,
    ZY100_CAL_STATUS_COMMIT_PENDING = 6U,
    ZY100_CAL_STATUS_WRITE_OK = 7U,
    ZY100_CAL_STATUS_BAD_FRAME = 8U,
    ZY100_CAL_STATUS_BAD_RECORD = 9U,
    ZY100_CAL_STATUS_BUSY = 10U,
    ZY100_CAL_STATUS_NOT_PAIRED = 11U,
    ZY100_CAL_STATUS_STORAGE_ERROR = 12U,
    ZY100_CAL_STATUS_TX_ERROR = 13U,
    ZY100_CAL_STATUS_CAL_ACCEPTED = 14U,
    ZY100_CAL_STATUS_CAL_STARTING = 15U,
    ZY100_CAL_STATUS_CAL_COLLECTING = 16U,
    ZY100_CAL_STATUS_CAL_FITTING = 17U,
    ZY100_CAL_STATUS_CAL_SAVING = 18U,
    ZY100_CAL_STATUS_CAL_SUCCESS = 19U,
    ZY100_CAL_STATUS_CAL_FAILED_TIMEOUT = 20U,
    ZY100_CAL_STATUS_CAL_FAILED_COVERAGE = 21U,
    ZY100_CAL_STATUS_CAL_FAILED_FIT = 22U,
    ZY100_CAL_STATUS_CAL_FAILED_SENSOR = 23U,
    ZY100_CAL_STATUS_CAL_SHUTDOWN_PENDING = 24U,
    ZY100_CAL_STATUS_INVALID_STATE = 25U,
    ZY100_CAL_STATUS_CAL_SUCCESS_LIMITED = 26U,
    ZY100_CAL_STATUS_CAL_COMPLETED_NO_UPDATE = 27U,
    ZY100_CAL_STATUS_CAL_INFO_CONFIRMED = 28U,
    ZY100_CAL_STATUS_CAL_INFO_MISMATCH = 29U,
    ZY100_CAL_STATUS_CAL_INFO_REQUIRED = 30U,
    ZY100_CAL_STATUS_CAL_RECORD_CACHE_CONFIRMED = 31U,
} zy100_cal_status_code_t;

typedef enum
{
    ZY100_CAL_DETAIL_NONE = 0U,
    ZY100_CAL_DETAIL_INSUFFICIENT_SAMPLES = 1U,
    ZY100_CAL_DETAIL_INSUFFICIENT_COVERAGE = 2U,
    ZY100_CAL_DETAIL_DEGENERATE_AXIS = 3U,
    ZY100_CAL_DETAIL_LINEAR_SYSTEM = 4U,
    ZY100_CAL_DETAIL_SHAPE_INVERSE = 5U,
    ZY100_CAL_DETAIL_NON_POSITIVE_SCALE = 6U,
    ZY100_CAL_DETAIL_NON_POSITIVE_ELLIPSOID = 7U,
    ZY100_CAL_DETAIL_INVALID_NORMALIZATION = 8U,
    ZY100_CAL_DETAIL_NON_FINITE_RESULT = 9U,
    ZY100_CAL_DETAIL_RECORD_ENCODE = 10U,
    ZY100_CAL_DETAIL_SENSOR_READ = 11U,
    ZY100_CAL_DETAIL_CANDIDATE_NOT_BETTER = 12U,
    ZY100_CAL_DETAIL_MODEL_HARD_IRON_ONLY = 0x0101U,
    ZY100_CAL_DETAIL_MODEL_AXIS_ALIGNED = 0x0102U,
    ZY100_CAL_DETAIL_OLD_RECORD_RETAINED = 0x0200U,
    ZY100_CAL_DETAIL_NO_USABLE_RECORD = 0x0300U,
} zy100_cal_status_detail_t;

typedef struct
{
    uint32_t generation;
    uint32_t valid_flags;
    uint16_t quality;
    uint64_t created_unix_ms;
    uint16_t record_bytes;
    uint32_t crc32;
} zy100_cal_record_info_t;

typedef struct
{
    uint8_t opcode;
    uint8_t transaction_id;
    uint16_t offset;
    uint16_t payload_len;
    uint16_t total_len;
    uint32_t crc32;
    const uint8_t *payload;
} zy100_cal_rx_frame_t;

typedef struct
{
    uint8_t transaction_id;
    uint8_t completion_status;
    uint8_t final_model;
    uint32_t sample_count;
    uint32_t rejected_sample_count;
    uint32_t sensor_read_errors;
    uint32_t elapsed_ms;
    uint8_t fit_attempts;
    uint8_t coverage_mask;
    uint8_t direction_mask;
    uint8_t final_ready_status;
    uint8_t full_solve_status;
    uint8_t axis_solve_status;
    uint8_t hard_solve_status;
    uint16_t quality;
    uint32_t rms_x1e6;
    uint32_t min_value[3];
    uint32_t max_value[3];
} zy100_cal_mag_diag_t;

uint16_t zy100_cal_get_u16_le(const uint8_t *src);
uint32_t zy100_cal_get_u32_le(const uint8_t *src);
void zy100_cal_put_u16_le(uint8_t *dst, uint16_t value);
void zy100_cal_put_u32_le(uint8_t *dst, uint32_t value);
void zy100_cal_put_u64_le(uint8_t *dst, uint64_t value);
bool zy100_cal_record_validate(const uint8_t *record,
                               uint16_t len,
                               zy100_cal_record_info_t *info);
bool zy100_cal_record_replace_mag_model(
    const uint8_t *current,
    uint16_t current_len,
    uint32_t generation,
    uint16_t quality,
    uint32_t mag_model_flags,
    uint64_t created_unix_ms,
    const float bias[3],
    const float matrix[9],
    uint8_t out[ZY100_CAL_RECORD_MAX_BYTES],
    zy100_cal_record_info_t *info);
bool zy100_cal_record_replace_battery_gain(
    const uint8_t *current,
    uint16_t current_len,
    uint32_t generation,
    uint16_t quality,
    uint64_t created_unix_ms,
    const zy100_cal_battery_gain_v1_t *battery,
    uint8_t out[ZY100_CAL_RECORD_MAX_BYTES],
    zy100_cal_record_info_t *info);
bool zy100_cal_record_get_battery_gain(
    const uint8_t *record,
    uint16_t len,
    zy100_cal_battery_gain_v1_t *battery);
bool zy100_cal_rx_parse(const uint8_t *data,
                        uint16_t len,
                        zy100_cal_rx_frame_t *frame);
uint16_t zy100_cal_build_tx_frame(uint8_t *out,
                                  uint16_t out_cap,
                                  uint8_t transaction_id,
                                  uint8_t frame_type,
                                  uint16_t offset,
                                  const uint8_t *payload,
                                  uint16_t payload_len,
                                  uint16_t total_len,
                                  uint32_t crc32);
void zy100_cal_build_info(uint8_t out[ZY100_CAL_INFO_BYTES],
                          const zy100_cal_record_info_t *info,
                          bool valid);
bool zy100_cal_confirm_info_matches(
    const zy100_cal_rx_frame_t *frame,
    const zy100_cal_record_info_t *info,
    bool valid);
bool zy100_cal_confirm_cached_record_matches(
    const zy100_cal_rx_frame_t *frame,
    const zy100_cal_record_info_t *info,
    bool valid);
void zy100_cal_build_status(uint8_t out[ZY100_CAL_STATUS_BYTES],
                            uint8_t status,
                            uint8_t transaction_id,
                            uint16_t detail,
                            const zy100_cal_record_info_t *info);
bool zy100_cal_build_mag_diagnostics(
    uint8_t out[ZY100_CAL_MAG_DIAG_BYTES],
    const zy100_cal_mag_diag_t *diagnostics);
bool zy100_cal_update_mag_diag_completion(
    uint8_t diagnostics[ZY100_CAL_MAG_DIAG_BYTES],
    uint8_t completion_status);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_CALIBRATION_PROTOCOL_H */
