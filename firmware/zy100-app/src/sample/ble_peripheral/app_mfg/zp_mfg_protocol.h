#ifndef ZP_MFG_PROTOCOL_H
#define ZP_MFG_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "zp_mfg_record.h"

#define ZP_MFG_WRITE_BINARY_V2              2U
#define ZP_MFG_WRITE_CHUNK_HEADER_BYTES     14U
#define ZP_MFG_WRITE_PAYLOAD_BYTES          144U
#define ZP_MFG_WRITE_PAYLOAD_CRC_OFFSET     140U
#define ZP_MFG_WRITE_CHANNEL_OFFSET          125U
#define ZP_MFG_WRITE_ENVIRONMENT_OFFSET      127U
#define ZP_MFG_WRITE_RESERVED_OFFSET         129U
#define ZP_MFG_WRITE_MAX_BUFFER_BYTES       ZP_MFG_WRITE_PAYLOAD_BYTES

typedef enum
{
    ZP_MFG_PROTO_STATUS_OK = 0U,
    ZP_MFG_PROTO_STATUS_BAD_PARAM,
    ZP_MFG_PROTO_STATUS_BAD_LENGTH,
    ZP_MFG_PROTO_STATUS_BAD_RANGE,
    ZP_MFG_PROTO_STATUS_UNSUPPORTED_VERSION,
    ZP_MFG_PROTO_STATUS_CRC_FAIL,
    ZP_MFG_PROTO_STATUS_CHANNEL_INVALID,
    ZP_MFG_PROTO_STATUS_ENVIRONMENT_INVALID,
    ZP_MFG_PROTO_STATUS_RESERVED_NOT_ZERO,
    ZP_MFG_PROTO_STATUS_TRAINING_CONFIG_INVALID,
    ZP_MFG_PROTO_STATUS_VALIDATE_FAIL,
} zp_mfg_proto_status_t;

typedef enum
{
    ZP_MFG_SEMANTIC_STATUS_OK = 0U,
    ZP_MFG_SEMANTIC_STATUS_BAD_PARAM,
    ZP_MFG_SEMANTIC_STATUS_RECORD_HEADER,
    ZP_MFG_SEMANTIC_STATUS_NOT_LOCKED,
    ZP_MFG_SEMANTIC_STATUS_UID_INVALID,
    ZP_MFG_SEMANTIC_STATUS_SN_INVALID,
    ZP_MFG_SEMANTIC_STATUS_SEQ_INVALID,
    ZP_MFG_SEMANTIC_STATUS_BLE_NAME_INVALID,
    ZP_MFG_SEMANTIC_STATUS_CLEANUP_MISSING,
    ZP_MFG_SEMANTIC_STATUS_REQUIRED_MASK_MISSING,
    ZP_MFG_SEMANTIC_STATUS_REQUIRED_PASS_MISSING,
    ZP_MFG_SEMANTIC_STATUS_IMU_CAL_INVALID,
} zp_mfg_semantic_status_t;

typedef struct
{
    uint32_t transaction_id;
    uint16_t total_len;
    uint16_t offset;
    uint16_t chunk_len;
    uint8_t flags;
    uint8_t reserved;
    uint16_t header_crc16_zero;
} zp_mfg_write_chunk_header_t;

zp_mfg_proto_status_t zp_mfg_protocol_parse_chunk(
    const uint8_t *data,
    uint16_t len,
    zp_mfg_write_chunk_header_t *header,
    const uint8_t **payload);
zp_mfg_proto_status_t zp_mfg_protocol_payload_to_record(
    const uint8_t payload[ZP_MFG_WRITE_PAYLOAD_BYTES],
    uint32_t passed_test_mask,
    uint8_t factory_cleanup_pass,
    uint8_t legacy_cleanup_suppressed,
    zp_mfg_record_t *record);
/* Explicit Factory v9 contract; never infer this policy from a missing bit. */
zp_mfg_proto_status_t zp_mfg_protocol_final_ship_payload_to_record(
    const uint8_t payload[ZP_MFG_WRITE_PAYLOAD_BYTES],
    uint32_t passed_test_mask,
    uint8_t factory_cleanup_pass,
    uint8_t legacy_cleanup_suppressed,
    zp_mfg_record_t *record);
void zp_mfg_training_led_config_default(
    zp_mfg_training_led_config_t *config);
bool zp_mfg_training_led_config_validate(
    const zp_mfg_training_led_config_t *config);
bool zp_mfg_training_led_config_set_record(
    zp_mfg_record_t *record,
    const zp_mfg_training_led_config_t *config);
bool zp_mfg_training_led_config_get_record(
    const zp_mfg_record_t *record,
    zp_mfg_training_led_config_t *config);
zp_mfg_semantic_status_t zp_mfg_record_semantic_validate(
    const zp_mfg_record_t *record,
    uint32_t firmware_required_mask);
const char *zp_mfg_proto_status_name(zp_mfg_proto_status_t status);
const char *zp_mfg_semantic_status_name(zp_mfg_semantic_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* ZP_MFG_PROTOCOL_H */
