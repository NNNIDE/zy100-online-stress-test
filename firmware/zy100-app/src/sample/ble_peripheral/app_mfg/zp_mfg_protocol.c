#include "zp_mfg_protocol.h"

#include <string.h>

#include "../common/zy100_byteorder.h"
#include "../service/zy100_crc32.h"
#include "zp_mfg_rules.h"
#include "zp_mfg_store.h"

static void zp_mfg_copy_fixed_string(char *dst,
                                     uint8_t dst_size,
                                     const uint8_t *src,
                                     uint8_t src_len)
{
    uint8_t copy_len;

    if ((dst == NULL) || (dst_size == 0U))
    {
        return;
    }
    memset(dst, 0, dst_size);
    if (src == NULL)
    {
        return;
    }

    copy_len = (src_len < (uint8_t)(dst_size - 1U)) ?
               src_len : (uint8_t)(dst_size - 1U);
    memcpy(dst, src, copy_len);
}

void zp_mfg_training_led_config_default(
    zp_mfg_training_led_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->schema_version = ZP_MFG_TRAINING_LED_SCHEMA_VERSION;
    config->target = ZP_MFG_TRAINING_LED_TARGET_NOTIFY;
    config->effect = ZP_MFG_TRAINING_LED_EFFECT_BREATH;
    config->blue = 255U;
    config->brightness_percent = 100U;
    config->speed = ZP_MFG_TRAINING_LED_SPEED_CAPTURE_DEFAULT;
    config->flags = ZP_MFG_TRAINING_LED_FLAG_ENABLED;
}

bool zp_mfg_training_led_config_validate(
    const zp_mfg_training_led_config_t *config)
{
    if ((config == NULL) ||
        (config->schema_version != ZP_MFG_TRAINING_LED_SCHEMA_VERSION) ||
        (config->auto_capture_enabled > 1U) ||
        (config->target < ZP_MFG_TRAINING_LED_TARGET_NOTIFY) ||
        (config->target > ZP_MFG_TRAINING_LED_TARGET_ALL) ||
        (config->effect < ZP_MFG_TRAINING_LED_EFFECT_SOLID) ||
        (config->effect > ZP_MFG_TRAINING_LED_EFFECT_MARQUEE) ||
        (config->brightness_percent == 0U) ||
        (config->brightness_percent > 100U) ||
        (config->speed > ZP_MFG_TRAINING_LED_SPEED_FAST) ||
        ((config->flags & (uint8_t)~ZP_MFG_TRAINING_LED_FLAG_ALLOWED) != 0U) ||
        (config->reserved != 0U))
    {
        return false;
    }

    return !((config->effect == ZP_MFG_TRAINING_LED_EFFECT_MARQUEE) &&
             (config->target == ZP_MFG_TRAINING_LED_TARGET_NOTIFY));
}

bool zp_mfg_training_led_config_set_record(
    zp_mfg_record_t *record,
    const zp_mfg_training_led_config_t *config)
{
    if ((record == NULL) || !zp_mfg_training_led_config_validate(config))
    {
        return false;
    }

    memcpy(&record->reserved[ZP_MFG_TRAINING_LED_CONFIG_STORAGE_OFFSET],
           config,
           sizeof(*config));
    return true;
}

bool zp_mfg_training_led_config_get_record(
    const zp_mfg_record_t *record,
    zp_mfg_training_led_config_t *config)
{
    uint8_t index;

    if ((record == NULL) || (config == NULL))
    {
        return false;
    }

    for (index = 0U; index < ZP_MFG_TRAINING_LED_CONFIG_BYTES; index++)
    {
        if (record->reserved[ZP_MFG_TRAINING_LED_CONFIG_STORAGE_OFFSET + index] != 0U)
        {
            memcpy(config,
                   &record->reserved[ZP_MFG_TRAINING_LED_CONFIG_STORAGE_OFFSET],
                   sizeof(*config));
            return zp_mfg_training_led_config_validate(config);
        }
    }

    zp_mfg_training_led_config_default(config);
    return true;
}

zp_mfg_semantic_status_t zp_mfg_record_semantic_validate(
    const zp_mfg_record_t *record,
    uint32_t firmware_required_mask)
{
    char expected_seq[ZP_MFG_ENCODED_SEQ_LEN + 1U];

    if (record == NULL)
    {
        return ZP_MFG_SEMANTIC_STATUS_BAD_PARAM;
    }
    if ((record->magic != ZP_MFG_RECORD_MAGIC) ||
        (((record->version == ZP_MFG_RECORD_VERSION_V3) &&
          (record->record_bytes != (uint16_t)sizeof(zp_mfg_record_v3_t))) ||
         ((record->version == ZP_MFG_RECORD_VERSION_V4) &&
           (record->record_bytes != (uint16_t)sizeof(zp_mfg_record_v4_t))) ||
          ((record->version == ZP_MFG_RECORD_VERSION_V5) &&
           (record->record_bytes != (uint16_t)sizeof(zp_mfg_record_v5_t))) ||
          ((record->version != ZP_MFG_RECORD_VERSION_V3) &&
           (record->version != ZP_MFG_RECORD_VERSION_V4) &&
           (record->version != ZP_MFG_RECORD_VERSION_V5))))
    {
        return ZP_MFG_SEMANTIC_STATUS_RECORD_HEADER;
    }
    if ((record->flags & ZP_MFG_RECORD_FLAG_LOCKED) == 0UL)
    {
        return ZP_MFG_SEMANTIC_STATUS_NOT_LOCKED;
    }
    if ((record->factory_cleanup_pass == 0U) ||
        ((record->flags & ZP_MFG_RECORD_FLAG_FACTORY_CLEANUP) == 0UL))
    {
        return ZP_MFG_SEMANTIC_STATUS_CLEANUP_MISSING;
    }
    if ((record->reserved_flags & ZP_MFG_RECORD_RESERVED_FINAL_SHIP_HOST) != 0U)
    {
        if ((record->version != ZP_MFG_RECORD_VERSION_V5) ||
            (record->required_test_mask != ZP_MFG_FINAL_SHIP_REQUIRED_TEST_MASK))
        {
            return ZP_MFG_SEMANTIC_STATUS_REQUIRED_MASK_MISSING;
        }
        firmware_required_mask = ZP_MFG_FINAL_SHIP_REQUIRED_TEST_MASK;
    }
    if ((record->required_test_mask & firmware_required_mask) !=
        firmware_required_mask)
    {
        return ZP_MFG_SEMANTIC_STATUS_REQUIRED_MASK_MISSING;
    }
    if ((record->passed_test_mask & firmware_required_mask) !=
        firmware_required_mask)
    {
        return ZP_MFG_SEMANTIC_STATUS_REQUIRED_PASS_MISSING;
    }
    if (!zp_mfg_sn_validate(record->product_code,
                            record->final_sn,
                            record->raw_internal_seq,
                            record->date_ymd))
    {
        return ZP_MFG_SEMANTIC_STATUS_SN_INVALID;
    }
    if (!zp_mfg_encode_seq6(record->raw_internal_seq, expected_seq) ||
        (memcmp(record->encoded_seq6,
                expected_seq,
                ZP_MFG_ENCODED_SEQ_LEN) != 0))
    {
        return ZP_MFG_SEMANTIC_STATUS_SEQ_INVALID;
    }
    if (!zp_mfg_ble_name_validate(record->ble_adv_name))
    {
        return ZP_MFG_SEMANTIC_STATUS_BLE_NAME_INVALID;
    }
    if ((record->version == ZP_MFG_RECORD_VERSION_V4) &&
        !zp_mfg_store_imu_cal_valid(&record->imu_cal))
    {
        return ZP_MFG_SEMANTIC_STATUS_IMU_CAL_INVALID;
    }
    if ((record->version == ZP_MFG_RECORD_VERSION_V5) &&
        !zp_mfg_store_imu_cal_valid(&record->imu_cal) &&
        !zp_mfg_store_legacy_imu_cal_absent(record))
    {
        return ZP_MFG_SEMANTIC_STATUS_IMU_CAL_INVALID;
    }

    return ZP_MFG_SEMANTIC_STATUS_OK;
}

zp_mfg_proto_status_t zp_mfg_protocol_parse_chunk(
    const uint8_t *data,
    uint16_t len,
    zp_mfg_write_chunk_header_t *header,
    const uint8_t **payload)
{
    if ((data == NULL) || (header == NULL) || (payload == NULL))
    {
        return ZP_MFG_PROTO_STATUS_BAD_PARAM;
    }
    if (len < ZP_MFG_WRITE_CHUNK_HEADER_BYTES)
    {
        return ZP_MFG_PROTO_STATUS_BAD_LENGTH;
    }

    header->transaction_id = zy100_get_u32_le(&data[0]);
    header->total_len = zy100_get_u16_le(&data[4]);
    header->offset = zy100_get_u16_le(&data[6]);
    header->chunk_len = zy100_get_u16_le(&data[8]);
    header->flags = data[10];
    header->reserved = data[11];
    header->header_crc16_zero = zy100_get_u16_le(&data[12]);

    if ((uint16_t)(len - ZP_MFG_WRITE_CHUNK_HEADER_BYTES) !=
        header->chunk_len)
    {
        return ZP_MFG_PROTO_STATUS_BAD_LENGTH;
    }
    if ((header->total_len != ZP_MFG_WRITE_PAYLOAD_BYTES) ||
        (header->chunk_len == 0U) ||
        (header->offset >= header->total_len) ||
        (header->chunk_len > (uint16_t)(header->total_len - header->offset)))
    {
        return ZP_MFG_PROTO_STATUS_BAD_RANGE;
    }

    *payload = &data[ZP_MFG_WRITE_CHUNK_HEADER_BYTES];
    return ZP_MFG_PROTO_STATUS_OK;
}

static zp_mfg_proto_status_t zp_mfg_payload_to_record_with_policy(
    const uint8_t payload[ZP_MFG_WRITE_PAYLOAD_BYTES],
    uint32_t passed_test_mask,
    uint8_t factory_cleanup_pass,
    uint8_t legacy_cleanup_suppressed,
    bool final_ship_host,
    zp_mfg_record_t *record)
{
    uint16_t version;
    uint16_t flags;
    uint32_t payload_crc;
    uint32_t expected_crc;
    uint32_t required_mask;
    uint32_t raw_seq;
    uint8_t expected_bt_addr_valid;
    char expected_seq[ZP_MFG_ENCODED_SEQ_LEN + 1U];
    const uint8_t *p;
    uint8_t idx;
    zp_mfg_training_led_config_t training_config;

    if ((payload == NULL) || (record == NULL))
    {
        return ZP_MFG_PROTO_STATUS_BAD_PARAM;
    }

    version = zy100_get_u16_le(&payload[0]);
    flags = zy100_get_u16_le(&payload[2]);
    if (version != ZP_MFG_WRITE_BINARY_V2)
    {
        return ZP_MFG_PROTO_STATUS_UNSUPPORTED_VERSION;
    }

    payload_crc = zy100_get_u32_le(&payload[ZP_MFG_WRITE_PAYLOAD_CRC_OFFSET]);
    expected_crc = zy100_crc32_ieee(payload, ZP_MFG_WRITE_PAYLOAD_CRC_OFFSET);
    if (payload_crc != expected_crc)
    {
        return ZP_MFG_PROTO_STATUS_CRC_FAIL;
    }

    memset(record, 0, sizeof(*record));
    p = &payload[4];
    zp_mfg_copy_fixed_string(record->product_code,
                             (uint8_t)sizeof(record->product_code),
                             p,
                             ZP_MFG_PRODUCT_CODE_LEN);
    p += ZP_MFG_PRODUCT_CODE_LEN;
    zp_mfg_copy_fixed_string(record->final_sn,
                             (uint8_t)sizeof(record->final_sn),
                             p,
                             ZP_MFG_FINAL_SN_LEN);
    p += ZP_MFG_FINAL_SN_LEN;
    raw_seq = zy100_get_u32_le(p);
    record->raw_internal_seq = raw_seq;
    p += 4U;
    zp_mfg_copy_fixed_string(record->encoded_seq6,
                             (uint8_t)sizeof(record->encoded_seq6),
                             p,
                             ZP_MFG_ENCODED_SEQ_LEN);
    p += ZP_MFG_ENCODED_SEQ_LEN;
    zp_mfg_copy_fixed_string(record->date_ymd,
                             (uint8_t)sizeof(record->date_ymd),
                             p,
                             ZP_MFG_DATE_YMD_LEN);
    p += ZP_MFG_DATE_YMD_LEN;
    zp_mfg_copy_fixed_string(record->ble_adv_name,
                             (uint8_t)sizeof(record->ble_adv_name),
                             p,
                             ZP_MFG_BLE_NAME_LEN);
    p += ZP_MFG_BLE_NAME_LEN;
    /* Legacy binary_v1 offset: external UID is no longer production identity. */
    memset(record->external_flash_uid, 0, ZP_MFG_EXT_FLASH_UID_BYTES);
    p += ZP_MFG_EXT_FLASH_UID_BYTES;
    zp_mfg_copy_fixed_string(record->fw_version,
                             (uint8_t)sizeof(record->fw_version),
                             p,
                             ZP_MFG_VERSION_TEXT_LEN);
    p += ZP_MFG_VERSION_TEXT_LEN;
    zp_mfg_copy_fixed_string(record->hw_version,
                             (uint8_t)sizeof(record->hw_version),
                             p,
                             ZP_MFG_VERSION_TEXT_LEN);
    p += ZP_MFG_VERSION_TEXT_LEN;
    zp_mfg_copy_fixed_string(record->mfg_tool_version,
                             (uint8_t)sizeof(record->mfg_tool_version),
                             p,
                             ZP_MFG_VERSION_TEXT_LEN);
    p += ZP_MFG_VERSION_TEXT_LEN;
    memcpy(record->bt_address, p, ZP_MFG_BT_ADDR_BYTES);
    p += ZP_MFG_BT_ADDR_BYTES;
    expected_bt_addr_valid = *p;
    if (p[1] != 0U)
    {
        return ZP_MFG_PROTO_STATUS_VALIDATE_FAIL;
    }
    p += 2U;
    required_mask = zy100_get_u32_le(p);
    p += 4U;
    zp_mfg_copy_fixed_string(record->channel_code,
                             (uint8_t)sizeof(record->channel_code),
                             p,
                             ZP_MFG_CHANNEL_CODE_LEN);
    p += ZP_MFG_CHANNEL_CODE_LEN;
    zp_mfg_copy_fixed_string(record->environment_code,
                             (uint8_t)sizeof(record->environment_code),
                             p,
                             ZP_MFG_ENVIRONMENT_CODE_LEN);
    p += ZP_MFG_ENVIRONMENT_CODE_LEN;
    if (!zp_mfg_identifier_code_validate(record->channel_code,
                                         ZP_MFG_CHANNEL_CODE_LEN))
    {
        return ZP_MFG_PROTO_STATUS_CHANNEL_INVALID;
    }
    if (!zp_mfg_identifier_code_validate(record->environment_code,
                                         ZP_MFG_ENVIRONMENT_CODE_LEN))
    {
        return ZP_MFG_PROTO_STATUS_ENVIRONMENT_INVALID;
    }
    for (idx = 0U;
         idx < (ZP_MFG_WRITE_PAYLOAD_CRC_OFFSET -
                ZP_MFG_WRITE_RESERVED_OFFSET);
         idx++)
    {
        if (p[idx] != 0U)
        {
            return ZP_MFG_PROTO_STATUS_RESERVED_NOT_ZERO;
        }
    }
    zp_mfg_training_led_config_default(&training_config);

    record->required_test_mask = required_mask;
    record->passed_test_mask = passed_test_mask;
    record->factory_cleanup_pass = factory_cleanup_pass;
    record->legacy_cleanup_suppressed = legacy_cleanup_suppressed;
    record->bt_address_valid = expected_bt_addr_valid ? 1U : 0U;
    record->flags = (uint32_t)flags;
    /* Preserve the exact binary_v2 authority used by the Factory Host.  The
     * locked Factory OTA retry gate compares this value before exposing DFU;
     * Production otherwise treats reserved_u32 as opaque as before. */
    record->reserved_u32[0] = payload_crc;
    if (!zp_mfg_training_led_config_set_record(record, &training_config))
    {
        return ZP_MFG_PROTO_STATUS_TRAINING_CONFIG_INVALID;
    }
    memset(record->external_flash_uid_text,
           0,
           sizeof(record->external_flash_uid_text));

    if (final_ship_host)
    {
        if ((required_mask != ZP_MFG_FINAL_SHIP_REQUIRED_TEST_MASK) ||
            ((passed_test_mask & ZP_MFG_TEST_SHIPPING_CURRENT) != 0UL))
        {
            return ZP_MFG_PROTO_STATUS_VALIDATE_FAIL;
        }
        record->reserved_flags |= ZP_MFG_RECORD_RESERVED_FINAL_SHIP_HOST;
    }
    else if ((required_mask & ZP_MFG_REQUIRED_TEST_MASK) !=
             ZP_MFG_REQUIRED_TEST_MASK)
    {
        return ZP_MFG_PROTO_STATUS_VALIDATE_FAIL;
    }
    if ((passed_test_mask & required_mask) != required_mask)
    {
        return ZP_MFG_PROTO_STATUS_VALIDATE_FAIL;
    }
    if (factory_cleanup_pass == 0U)
    {
        return ZP_MFG_PROTO_STATUS_VALIDATE_FAIL;
    }
    if (!zp_mfg_sn_validate(record->product_code,
                            record->final_sn,
                            raw_seq,
                            record->date_ymd))
    {
        return ZP_MFG_PROTO_STATUS_VALIDATE_FAIL;
    }
    if (!zp_mfg_encode_seq6(raw_seq, expected_seq) ||
        (memcmp(record->encoded_seq6,
                expected_seq,
                ZP_MFG_ENCODED_SEQ_LEN) != 0))
    {
        return ZP_MFG_PROTO_STATUS_VALIDATE_FAIL;
    }
    if (!zp_mfg_ble_name_validate(record->ble_adv_name))
    {
        return ZP_MFG_PROTO_STATUS_VALIDATE_FAIL;
    }
    return ZP_MFG_PROTO_STATUS_OK;
}

zp_mfg_proto_status_t zp_mfg_protocol_payload_to_record(
    const uint8_t payload[ZP_MFG_WRITE_PAYLOAD_BYTES],
    uint32_t passed_test_mask, uint8_t factory_cleanup_pass,
    uint8_t legacy_cleanup_suppressed, zp_mfg_record_t *record)
{
    return zp_mfg_payload_to_record_with_policy(payload, passed_test_mask,
        factory_cleanup_pass, legacy_cleanup_suppressed, false, record);
}

zp_mfg_proto_status_t zp_mfg_protocol_final_ship_payload_to_record(
    const uint8_t payload[ZP_MFG_WRITE_PAYLOAD_BYTES],
    uint32_t passed_test_mask, uint8_t factory_cleanup_pass,
    uint8_t legacy_cleanup_suppressed, zp_mfg_record_t *record)
{
    return zp_mfg_payload_to_record_with_policy(payload, passed_test_mask,
        factory_cleanup_pass, legacy_cleanup_suppressed, true, record);
}

const char *zp_mfg_proto_status_name(zp_mfg_proto_status_t status)
{
    switch (status)
    {
    case ZP_MFG_PROTO_STATUS_OK:
        return "OK";
    case ZP_MFG_PROTO_STATUS_BAD_PARAM:
        return "BAD_PARAM";
    case ZP_MFG_PROTO_STATUS_BAD_LENGTH:
        return "BAD_LENGTH";
    case ZP_MFG_PROTO_STATUS_BAD_RANGE:
        return "BAD_RANGE";
    case ZP_MFG_PROTO_STATUS_UNSUPPORTED_VERSION:
        return "UNSUPPORTED_VERSION";
    case ZP_MFG_PROTO_STATUS_CRC_FAIL:
        return "CRC_FAIL";
    case ZP_MFG_PROTO_STATUS_CHANNEL_INVALID:
        return "CHANNEL_INVALID";
    case ZP_MFG_PROTO_STATUS_ENVIRONMENT_INVALID:
        return "ENVIRONMENT_INVALID";
    case ZP_MFG_PROTO_STATUS_RESERVED_NOT_ZERO:
        return "RESERVED_NOT_ZERO";
    case ZP_MFG_PROTO_STATUS_TRAINING_CONFIG_INVALID:
        return "TRAINING_CONFIG_INVALID";
    case ZP_MFG_PROTO_STATUS_VALIDATE_FAIL:
        return "VALIDATE_FAIL";
    default:
        return "UNKNOWN";
    }
}

const char *zp_mfg_semantic_status_name(zp_mfg_semantic_status_t status)
{
    switch (status)
    {
    case ZP_MFG_SEMANTIC_STATUS_OK:
        return "OK";
    case ZP_MFG_SEMANTIC_STATUS_BAD_PARAM:
        return "BAD_PARAM";
    case ZP_MFG_SEMANTIC_STATUS_RECORD_HEADER:
        return "RECORD_HEADER";
    case ZP_MFG_SEMANTIC_STATUS_NOT_LOCKED:
        return "NOT_LOCKED";
    case ZP_MFG_SEMANTIC_STATUS_UID_INVALID:
        return "UID_INVALID";
    case ZP_MFG_SEMANTIC_STATUS_SN_INVALID:
        return "SN_INVALID";
    case ZP_MFG_SEMANTIC_STATUS_SEQ_INVALID:
        return "SEQ_INVALID";
    case ZP_MFG_SEMANTIC_STATUS_BLE_NAME_INVALID:
        return "BLE_NAME_INVALID";
    case ZP_MFG_SEMANTIC_STATUS_CLEANUP_MISSING:
        return "CLEANUP_MISSING";
    case ZP_MFG_SEMANTIC_STATUS_REQUIRED_MASK_MISSING:
        return "REQUIRED_MASK_MISSING";
    case ZP_MFG_SEMANTIC_STATUS_REQUIRED_PASS_MISSING:
        return "REQUIRED_PASS_MISSING";
    case ZP_MFG_SEMANTIC_STATUS_IMU_CAL_INVALID:
        return "IMU_CAL_INVALID";
    default:
        return "UNKNOWN";
    }
}
