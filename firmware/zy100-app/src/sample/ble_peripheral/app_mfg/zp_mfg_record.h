#ifndef ZP_MFG_RECORD_H
#define ZP_MFG_RECORD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ZP_MFG_RECORD_MAGIC                 0x5A504D46UL
#define ZP_MFG_RECORD_VERSION_V3            3U
#define ZP_MFG_RECORD_VERSION_V4            4U
#define ZP_MFG_RECORD_VERSION_V5            5U
#define ZP_MFG_RECORD_VERSION               ZP_MFG_RECORD_VERSION_V3
#define ZP_MFG_RECORD_FLAG_LOCKED           0x00000001UL
#define ZP_MFG_RECORD_FLAG_FACTORY_CLEANUP  0x00000002UL
#define ZP_MFG_RECORD_FLAG_LEGACY_SUPPRESS  0x00000004UL

/*
 * V3 production records predate the IMU calibration sub-record.  When such a
 * locked record is promoted to V5 solely to attach the runtime calibration
 * blob, preserve that provenance in the existing reserved_flags byte.  This
 * is not an anonymous padding byte: boot semantic validation uses the bit to
 * distinguish an intentionally absent legacy IMU record from a damaged V4/V5
 * IMU record.
 */
#define ZP_MFG_RECORD_RESERVED_LEGACY_NO_IMU_CAL 0x01U
#define ZP_MFG_RECORD_RESERVED_FINAL_SHIP_HOST   0x02U
#define ZP_MFG_FINAL_SHIP_REQUIRED_TEST_MASK     0x0FB7FFF6UL

#define ZP_MFG_IMU_CAL_MAGIC                0x494D5543UL
#define ZP_MFG_IMU_CAL_VERSION              1U
#define ZP_MFG_IMU_CAL_MODE_SINGLE_POSE_V1  1U
#define ZP_MFG_IMU_CAL_FLAG_GYRO_VALID      0x00000001UL
#define ZP_MFG_IMU_CAL_FLAG_ACCEL_VALID     0x00000002UL
#define ZP_MFG_IMU_CAL_FLAG_CHARACTERIZATION 0x00000004UL

#define ZP_MFG_PRODUCT_CODE_LEN             4U
#define ZP_MFG_FINAL_SN_LEN                 16U
#define ZP_MFG_DATE_YMD_LEN                 6U
#define ZP_MFG_ENCODED_SEQ_LEN              6U
#define ZP_MFG_BLE_NAME_LEN                 9U
#define ZP_MFG_EXT_FLASH_UID_BYTES          16U
#define ZP_MFG_EXT_FLASH_UID_TEXT_LEN       32U
#define ZP_MFG_BT_ADDR_BYTES                6U
#define ZP_MFG_VERSION_TEXT_LEN             16U
#define ZP_MFG_CHANNEL_CODE_LEN              2U
#define ZP_MFG_ENVIRONMENT_CODE_LEN          2U
#define ZP_MFG_CALIBRATION_BLOB_MAX_BYTES   256U

/*
 * The Factory first-lock path writes the fixed product defaults into this
 * 11-byte object in the existing common-record reserved area. Record sizes
 * and MFG A/B layout stay unchanged; this is reserved for a future firmware
 * configuration policy, not a current production-tool setting.
 */
#define ZP_MFG_TRAINING_LED_CONFIG_BYTES              11U
#define ZP_MFG_TRAINING_LED_CONFIG_STORAGE_OFFSET      0U
#define ZP_MFG_TRAINING_LED_SCHEMA_VERSION             1U

#define ZP_MFG_TRAINING_LED_TARGET_NOTIFY              1U
#define ZP_MFG_TRAINING_LED_TARGET_LOGO                2U
#define ZP_MFG_TRAINING_LED_TARGET_ALL                 3U

#define ZP_MFG_TRAINING_LED_EFFECT_SOLID               1U
#define ZP_MFG_TRAINING_LED_EFFECT_BLINK               2U
#define ZP_MFG_TRAINING_LED_EFFECT_BREATH              3U
#define ZP_MFG_TRAINING_LED_EFFECT_MARQUEE             4U

#define ZP_MFG_TRAINING_LED_SPEED_CAPTURE_DEFAULT      0U
#define ZP_MFG_TRAINING_LED_SPEED_SLOW                 1U
#define ZP_MFG_TRAINING_LED_SPEED_DEFAULT              2U
#define ZP_MFG_TRAINING_LED_SPEED_FAST                 3U

#define ZP_MFG_TRAINING_LED_FLAG_ENABLED               0x01U
#define ZP_MFG_TRAINING_LED_FLAG_MARQUEE_REVERSE       0x02U
#define ZP_MFG_TRAINING_LED_FLAG_ALLOWED               \
    (ZP_MFG_TRAINING_LED_FLAG_ENABLED |                \
     ZP_MFG_TRAINING_LED_FLAG_MARQUEE_REVERSE)

#define ZP_MFG_TEST_LED_SHOWCASE            0x00000001UL
#define ZP_MFG_TEST_FACTORY_CLEANUP         0x00000002UL
#define ZP_MFG_TEST_EXT_FLASH_ID            0x00000004UL
#define ZP_MFG_TEST_EXT_FLASH_UID           0x00000008UL /* legacy, no longer required */
#define ZP_MFG_TEST_EXT_FLASH_RW            0x00000010UL
#define ZP_MFG_TEST_IMU                     0x00000020UL
#define ZP_MFG_TEST_MAG                     0x00000040UL
#define ZP_MFG_TEST_YHM2712                 0x00000080UL
#define ZP_MFG_TEST_BATTERY_ADC             0x00000100UL
#define ZP_MFG_TEST_CHARGE_STATUS           0x00000200UL
#define ZP_MFG_TEST_KEY                     0x00000400UL
#define ZP_MFG_TEST_BLE_ADV                 0x00000800UL
#define ZP_MFG_TEST_BLE_CONNECT             0x00001000UL
#define ZP_MFG_TEST_RSSI                    0x00002000UL
#define ZP_MFG_TEST_LED_RED                 0x00004000UL
#define ZP_MFG_TEST_LED_GREEN               0x00008000UL
#define ZP_MFG_TEST_LED_BLUE                0x00010000UL
#define ZP_MFG_TEST_LED_RGB_SEQUENCE        0x00020000UL
#define ZP_MFG_TEST_CONNECTED_CURRENT       0x00040000UL
#define ZP_MFG_TEST_SHIPPING_CURRENT        0x00080000UL
#define ZP_MFG_TEST_BATTERY_CAL             0x00100000UL
#define ZP_MFG_TEST_BATTERY_VERIFY          0x00200000UL

#define ZP_MFG_REQUIRED_TEST_MASK           \
    (ZP_MFG_TEST_FACTORY_CLEANUP |          \
     ZP_MFG_TEST_EXT_FLASH_ID |             \
     ZP_MFG_TEST_EXT_FLASH_RW |             \
     ZP_MFG_TEST_IMU |                      \
     ZP_MFG_TEST_MAG |                      \
     ZP_MFG_TEST_YHM2712 |                  \
     ZP_MFG_TEST_BATTERY_ADC |              \
     ZP_MFG_TEST_CHARGE_STATUS |            \
     ZP_MFG_TEST_KEY |                      \
     ZP_MFG_TEST_BLE_ADV |                  \
     ZP_MFG_TEST_BLE_CONNECT |              \
     ZP_MFG_TEST_RSSI |                     \
     ZP_MFG_TEST_LED_RED |                  \
     ZP_MFG_TEST_LED_GREEN |                \
     ZP_MFG_TEST_LED_BLUE |                 \
     ZP_MFG_TEST_LED_RGB_SEQUENCE |        \
     ZP_MFG_TEST_CONNECTED_CURRENT |       \
     ZP_MFG_TEST_SHIPPING_CURRENT |        \
     ZP_MFG_TEST_BATTERY_CAL |             \
     ZP_MFG_TEST_BATTERY_VERIFY)

#define ZP_MFG_RECORD_COMMON_FIELDS         \
    uint32_t magic;                         \
    uint16_t version;                       \
    uint16_t record_bytes;                  \
    uint32_t generation;                    \
    uint32_t flags;                         \
    uint32_t required_test_mask;            \
    uint32_t passed_test_mask;              \
    uint8_t factory_cleanup_pass;           \
    uint8_t legacy_cleanup_suppressed;      \
    uint8_t bt_address_valid;               \
    uint8_t reserved_flags;                 \
    char product_code[ZP_MFG_PRODUCT_CODE_LEN + 1U]; \
    char final_sn[ZP_MFG_FINAL_SN_LEN + 1U]; \
    uint32_t raw_internal_seq;              \
    char encoded_seq6[ZP_MFG_ENCODED_SEQ_LEN + 1U]; \
    char date_ymd[ZP_MFG_DATE_YMD_LEN + 1U]; \
    char ble_adv_name[ZP_MFG_BLE_NAME_LEN + 1U]; \
    uint8_t external_flash_uid[ZP_MFG_EXT_FLASH_UID_BYTES]; \
    char external_flash_uid_text[ZP_MFG_EXT_FLASH_UID_TEXT_LEN + 1U]; \
    uint8_t bt_address[ZP_MFG_BT_ADDR_BYTES]; \
    char fw_version[ZP_MFG_VERSION_TEXT_LEN]; \
    char hw_version[ZP_MFG_VERSION_TEXT_LEN]; \
    char mfg_tool_version[ZP_MFG_VERSION_TEXT_LEN]; \
    uint32_t write_counter;                 \
    uint32_t reserved_u32[8];               \
    char channel_code[ZP_MFG_CHANNEL_CODE_LEN + 1U]; \
    char environment_code[ZP_MFG_ENVIRONMENT_CODE_LEN + 1U]; \
    uint8_t reserved[90]

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t bytes;
    uint32_t flags;
    uint8_t mode;
    uint8_t gravity_axis;
    int8_t gravity_sign;
    uint8_t quality_profile;
    uint16_t accel_fsr_g;
    uint16_t gyro_fsr_dps;
    uint16_t nominal_odr_hz;
    uint16_t reserved0;
    uint32_t sample_count;
    uint32_t read_error_count;
    uint32_t duration_ms;
    int32_t gyro_bias_q16[3];
    int32_t accel_bias_q16[3];
    int32_t mean_temp_raw_q16;
    uint16_t accel_p2p_raw[3];
    uint16_t gyro_p2p_raw[3];
    uint16_t accel_norm_error_mg;
    uint16_t accel_half_drift_mg;
    uint16_t gyro_half_drift_raw;
    uint16_t reserved1;
    uint32_t reserved_u32[2];
    uint32_t crc32;
} zp_mfg_imu_single_pose_cal_v1_t;

typedef struct
{
    uint8_t schema_version;
    uint8_t auto_capture_enabled;
    uint8_t target;
    uint8_t effect;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t brightness_percent;
    uint8_t speed;
    uint8_t flags;
    uint8_t reserved;
} zp_mfg_training_led_config_t;

typedef struct
{
    ZP_MFG_RECORD_COMMON_FIELDS;
    uint32_t crc32;
} zp_mfg_record_v3_t;

typedef struct
{
    ZP_MFG_RECORD_COMMON_FIELDS;
    zp_mfg_imu_single_pose_cal_v1_t imu_cal;
    uint32_t crc32;
} zp_mfg_record_v4_t;

typedef struct
{
    ZP_MFG_RECORD_COMMON_FIELDS;
    zp_mfg_imu_single_pose_cal_v1_t imu_cal;
    uint16_t calibration_blob_bytes;
    uint16_t calibration_reserved;
    uint8_t calibration_blob[ZP_MFG_CALIBRATION_BLOB_MAX_BYTES];
    uint32_t crc32;
} zp_mfg_record_v5_t;

typedef zp_mfg_record_v4_t zp_mfg_record_t;

#undef ZP_MFG_RECORD_COMMON_FIELDS

typedef char zp_mfg_record_v3_size_check[
    (sizeof(zp_mfg_record_v3_t) == 320U) ? 1 : -1];
typedef char zp_mfg_imu_cal_size_check[
    (sizeof(zp_mfg_imu_single_pose_cal_v1_t) == 96U) ? 1 : -1];
typedef char zp_mfg_training_led_config_size_check[
    (sizeof(zp_mfg_training_led_config_t) ==
     ZP_MFG_TRAINING_LED_CONFIG_BYTES) ? 1 : -1];
typedef char zp_mfg_record_v4_size_check[
    (sizeof(zp_mfg_record_v4_t) == 416U) ? 1 : -1];
typedef char zp_mfg_record_v5_size_check[
    (sizeof(zp_mfg_record_v5_t) == 676U) ? 1 : -1];

#ifdef __cplusplus
}
#endif

#endif /* ZP_MFG_RECORD_H */
