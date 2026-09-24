#include "zy100_calibration_protocol.h"

#include <string.h>

#include "zy100_crc32.h"

uint16_t zy100_cal_get_u16_le(const uint8_t *src)
{
    return (uint16_t)(((uint16_t)src[0]) | ((uint16_t)src[1] << 8));
}

uint32_t zy100_cal_get_u32_le(const uint8_t *src)
{
    return ((uint32_t)src[0]) |
           ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

static uint64_t zy100_cal_get_u64_le(const uint8_t *src)
{
    return ((uint64_t)zy100_cal_get_u32_le(src)) |
           ((uint64_t)zy100_cal_get_u32_le(src + 4U) << 32);
}

void zy100_cal_put_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
}

void zy100_cal_put_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
    dst[2] = (uint8_t)(value >> 16);
    dst[3] = (uint8_t)(value >> 24);
}

void zy100_cal_put_u64_le(uint8_t *dst, uint64_t value)
{
    zy100_cal_put_u32_le(dst, (uint32_t)value);
    zy100_cal_put_u32_le(dst + 4U, (uint32_t)(value >> 32));
}

static bool zy100_cal_float_block_finite(const uint8_t *data, uint8_t len)
{
    uint8_t offset;

    if ((data == NULL) || ((len & 3U) != 0U))
    {
        return false;
    }
    for (offset = 0U; offset < len; offset = (uint8_t)(offset + 4U))
    {
        uint32_t bits = zy100_cal_get_u32_le(&data[offset]);
        if ((bits & 0x7F800000UL) == 0x7F800000UL)
        {
            return false;
        }
    }
    return true;
}

static bool zy100_cal_known_tlv(uint8_t type,
                                uint8_t len,
                                const uint8_t *value,
                                uint32_t *seen,
                                uint32_t *present_flags)
{
    uint32_t bit = 0UL;
    uint8_t expected = len;

    switch (type)
    {
    case ZY100_CAL_TLV_IMU_GYRO_MODEL:
        bit = ZY100_CAL_VALID_IMU_GYRO;
        expected = ZY100_CAL_TLV_BIAS_MATRIX_BYTES;
        break;
    case ZY100_CAL_TLV_IMU_ACCEL_MODEL:
        bit = ZY100_CAL_VALID_IMU_ACCEL;
        expected = ZY100_CAL_TLV_BIAS_MATRIX_BYTES;
        break;
    case ZY100_CAL_TLV_IMU_INSTALL_MATRIX:
        bit = ZY100_CAL_VALID_IMU_INSTALL;
        expected = ZY100_CAL_TLV_MATRIX3_BYTES;
        break;
    case ZY100_CAL_TLV_MAG_MODEL:
        bit = ZY100_CAL_VALID_MAG_MODEL;
        expected = ZY100_CAL_TLV_MAG_MODEL_BYTES;
        break;
    case ZY100_CAL_TLV_MAG_INSTALL_MATRIX:
        bit = ZY100_CAL_VALID_MAG_INSTALL;
        expected = ZY100_CAL_TLV_MATRIX3_BYTES;
        break;
    case ZY100_CAL_TLV_QUALITY_META:
        expected = len;
        break;
    case ZY100_CAL_TLV_BATTERY_GAIN_V1:
        bit = ZY100_CAL_VALID_BATTERY_GAIN;
        expected = ZY100_CAL_TLV_BATTERY_GAIN_V1_BYTES;
        break;
    default:
        return true;
    }

    if (len != expected)
    {
        return false;
    }
    if (bit != 0UL)
    {
        if ((*seen & bit) != 0UL)
        {
            return false;
        }
        *seen |= bit;
        *present_flags |= bit;
        if (type == ZY100_CAL_TLV_BATTERY_GAIN_V1)
        {
            return (value[0] == ZY100_CAL_BATTERY_GAIN_VERSION) &&
                   (value[1] != 0U) &&
                   (zy100_cal_get_u32_le(&value[4]) != 0UL) &&
                   (zy100_cal_get_u16_le(&value[14]) != 0U) &&
                   (zy100_cal_get_u16_le(&value[16]) != 0U) &&
                   (zy100_cal_get_u32_le(&value[18]) != 0UL);
        }
        return zy100_cal_float_block_finite(value, len);
    }
    return true;
}

bool zy100_cal_record_validate(const uint8_t *record,
                               uint16_t len,
                               zy100_cal_record_info_t *info)
{
    uint16_t record_bytes;
    uint16_t offset;
    uint32_t expected_crc;
    uint32_t crc;
    uint32_t zero = 0UL;
    uint32_t seen = 0UL;
    uint32_t present_flags = 0UL;
    uint32_t valid_flags;

    if (info != NULL)
    {
        memset(info, 0, sizeof(*info));
    }
    if ((record == NULL) || (len < ZY100_CAL_RECORD_HEADER_BYTES) ||
        (len > ZY100_CAL_RECORD_MAX_BYTES) ||
        (zy100_cal_get_u32_le(record) != ZY100_CAL_RECORD_MAGIC) ||
        (record[4] != ZY100_CAL_RECORD_VERSION) ||
        (record[5] != ZY100_CAL_RECORD_HEADER_BYTES))
    {
        return false;
    }

    record_bytes = zy100_cal_get_u16_le(&record[6]);
    if ((record_bytes != len) || (record_bytes > ZY100_CAL_RECORD_MAX_BYTES))
    {
        return false;
    }
    expected_crc = zy100_cal_get_u32_le(&record[ZY100_CAL_RECORD_CRC_OFFSET]);
    crc = zy100_crc32_ieee_begin();
    crc = zy100_crc32_ieee_update(crc, record, ZY100_CAL_RECORD_CRC_OFFSET);
    crc = zy100_crc32_ieee_update(crc, (const uint8_t *)&zero, 4U);
    crc = zy100_crc32_ieee_update(crc,
                                  &record[ZY100_CAL_RECORD_CRC_OFFSET + 4U],
                                  (uint32_t)record_bytes -
                                  (ZY100_CAL_RECORD_CRC_OFFSET + 4U));
    if (zy100_crc32_ieee_finish(crc) != expected_crc)
    {
        return false;
    }

    offset = ZY100_CAL_RECORD_HEADER_BYTES;
    while (offset < record_bytes)
    {
        uint8_t type;
        uint8_t tlv_len;
        if ((uint16_t)(record_bytes - offset) < ZY100_CAL_TLV_HEADER_BYTES)
        {
            return false;
        }
        type = record[offset];
        tlv_len = record[offset + 1U];
        offset = (uint16_t)(offset + ZY100_CAL_TLV_HEADER_BYTES);
        if ((tlv_len == 0U) || ((uint16_t)(record_bytes - offset) < tlv_len) ||
            !zy100_cal_known_tlv(type, tlv_len, &record[offset],
                                 &seen, &present_flags))
        {
            return false;
        }
        offset = (uint16_t)(offset + tlv_len);
    }

    valid_flags = zy100_cal_get_u32_le(&record[12]);
    if (((valid_flags & ZY100_CAL_MAG_MODEL_FLAG_MASK) ==
         ZY100_CAL_MAG_MODEL_FLAG_MASK) ||
        (((valid_flags & ZY100_CAL_MAG_MODEL_FLAG_MASK) != 0UL) &&
         ((valid_flags & ZY100_CAL_VALID_MAG_MODEL) == 0UL)))
    {
        return false;
    }
    if ((valid_flags & (ZY100_CAL_VALID_IMU_GYRO |
                        ZY100_CAL_VALID_IMU_ACCEL |
                        ZY100_CAL_VALID_IMU_INSTALL |
                        ZY100_CAL_VALID_MAG_MODEL |
                        ZY100_CAL_VALID_MAG_INSTALL |
                        ZY100_CAL_VALID_BATTERY_GAIN)) != present_flags)
    {
        return false;
    }
    if (info != NULL)
    {
        info->generation = zy100_cal_get_u32_le(&record[8]);
        info->valid_flags = valid_flags;
        info->quality = zy100_cal_get_u16_le(&record[16]);
        info->created_unix_ms = zy100_cal_get_u64_le(&record[18]);
        info->record_bytes = record_bytes;
        info->crc32 = expected_crc;
    }
    return true;
}

static void zy100_cal_put_float(uint8_t *destination, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    zy100_cal_put_u32_le(destination, bits);
}

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
    zy100_cal_record_info_t *info)
{
    uint16_t source_offset = ZY100_CAL_RECORD_HEADER_BYTES;
    uint16_t destination_offset = ZY100_CAL_RECORD_HEADER_BYTES;
    uint32_t valid_flags = ZY100_CAL_VALID_MAG_MODEL;
    uint32_t crc;
    uint32_t zero = 0UL;
    uint8_t index;

    if ((bias == NULL) || (matrix == NULL) || (out == NULL) ||
        ((mag_model_flags & ~ZY100_CAL_MAG_MODEL_FLAG_MASK) != 0UL) ||
        (mag_model_flags == ZY100_CAL_MAG_MODEL_FLAG_MASK) ||
        ((current != NULL) &&
         !zy100_cal_record_validate(current, current_len, NULL)))
    {
        return false;
    }
    memset(out, 0, ZY100_CAL_RECORD_MAX_BYTES);
    if (current != NULL)
    {
        valid_flags = (zy100_cal_get_u32_le(&current[12]) &
                       ~ZY100_CAL_MAG_MODEL_FLAG_MASK) |
                      ZY100_CAL_VALID_MAG_MODEL;
        while (source_offset < current_len)
        {
            uint8_t type = current[source_offset];
            uint8_t tlv_len = current[source_offset + 1U];
            uint16_t tlv_bytes = (uint16_t)(ZY100_CAL_TLV_HEADER_BYTES +
                                            tlv_len);
            if ((type != ZY100_CAL_TLV_MAG_MODEL) &&
                (type != ZY100_CAL_TLV_QUALITY_META))
            {
                if ((uint16_t)(destination_offset + tlv_bytes) >
                    ZY100_CAL_RECORD_MAX_BYTES)
                {
                    return false;
                }
                memcpy(&out[destination_offset], &current[source_offset],
                       tlv_bytes);
                destination_offset = (uint16_t)(destination_offset + tlv_bytes);
            }
            source_offset = (uint16_t)(source_offset + tlv_bytes);
        }
    }
    valid_flags |= mag_model_flags;
    if ((uint16_t)(destination_offset + ZY100_CAL_TLV_HEADER_BYTES +
                   ZY100_CAL_TLV_MAG_MODEL_BYTES) >
        ZY100_CAL_RECORD_MAX_BYTES)
    {
        return false;
    }
    out[destination_offset++] = ZY100_CAL_TLV_MAG_MODEL;
    out[destination_offset++] = ZY100_CAL_TLV_MAG_MODEL_BYTES;
    for (index = 0U; index < 3U; index++)
    {
        zy100_cal_put_float(&out[destination_offset], bias[index]);
        destination_offset = (uint16_t)(destination_offset + 4U);
    }
    for (index = 0U; index < 9U; index++)
    {
        zy100_cal_put_float(&out[destination_offset], matrix[index]);
        destination_offset = (uint16_t)(destination_offset + 4U);
    }

    zy100_cal_put_u32_le(&out[0], ZY100_CAL_RECORD_MAGIC);
    out[4] = ZY100_CAL_RECORD_VERSION;
    out[5] = ZY100_CAL_RECORD_HEADER_BYTES;
    zy100_cal_put_u16_le(&out[6], destination_offset);
    zy100_cal_put_u32_le(&out[8], generation);
    zy100_cal_put_u32_le(&out[12], valid_flags);
    zy100_cal_put_u16_le(&out[16], quality);
    zy100_cal_put_u64_le(&out[18], created_unix_ms);
    crc = zy100_crc32_ieee_begin();
    crc = zy100_crc32_ieee_update(crc, out, ZY100_CAL_RECORD_CRC_OFFSET);
    crc = zy100_crc32_ieee_update(crc, (const uint8_t *)&zero, 4U);
    crc = zy100_crc32_ieee_update(
              crc, &out[ZY100_CAL_RECORD_CRC_OFFSET + 4U],
              (uint32_t)destination_offset -
              (ZY100_CAL_RECORD_CRC_OFFSET + 4U));
    zy100_cal_put_u32_le(&out[ZY100_CAL_RECORD_CRC_OFFSET],
                        zy100_crc32_ieee_finish(crc));
    return zy100_cal_record_validate(out, destination_offset, info);
}

bool zy100_cal_record_get_battery_gain(
    const uint8_t *record,
    uint16_t len,
    zy100_cal_battery_gain_v1_t *battery)
{
    uint16_t offset = ZY100_CAL_RECORD_HEADER_BYTES;

    if ((battery == NULL) ||
        !zy100_cal_record_validate(record, len, NULL))
    {
        return false;
    }
    memset(battery, 0, sizeof(*battery));
    while (offset < len)
    {
        uint8_t type = record[offset];
        uint8_t tlv_len = record[offset + 1U];
        const uint8_t *value = &record[offset + ZY100_CAL_TLV_HEADER_BYTES];

        if (type == ZY100_CAL_TLV_BATTERY_GAIN_V1)
        {
            if ((tlv_len != ZY100_CAL_TLV_BATTERY_GAIN_V1_BYTES) ||
                (value[0] != ZY100_CAL_BATTERY_GAIN_VERSION))
            {
                return false;
            }
            battery->version = value[0];
            battery->sample_count = value[1];
            battery->workflow_token = zy100_cal_get_u32_le(&value[4]);
            memcpy(battery->bt_address, &value[8], sizeof(battery->bt_address));
            battery->reference_mv = zy100_cal_get_u16_le(&value[14]);
            battery->uncalibrated_mv = zy100_cal_get_u16_le(&value[16]);
            battery->gain_q20 = zy100_cal_get_u32_le(&value[18]);
            battery->precal_delta_mv =
                (int32_t)zy100_cal_get_u32_le(&value[22]);
            return (battery->sample_count != 0U) &&
                   (battery->workflow_token != 0UL) &&
                   (battery->reference_mv != 0U) &&
                   (battery->uncalibrated_mv != 0U) &&
                   (battery->gain_q20 != 0UL);
        }
        offset = (uint16_t)(offset + ZY100_CAL_TLV_HEADER_BYTES + tlv_len);
    }
    return false;
}

bool zy100_cal_record_replace_battery_gain(
    const uint8_t *current,
    uint16_t current_len,
    uint32_t generation,
    uint16_t quality,
    uint64_t created_unix_ms,
    const zy100_cal_battery_gain_v1_t *battery,
    uint8_t out[ZY100_CAL_RECORD_MAX_BYTES],
    zy100_cal_record_info_t *info)
{
    uint16_t source_offset = ZY100_CAL_RECORD_HEADER_BYTES;
    uint16_t destination_offset = ZY100_CAL_RECORD_HEADER_BYTES;
    uint32_t valid_flags = ZY100_CAL_VALID_BATTERY_GAIN;
    uint32_t crc;
    uint32_t zero = 0UL;
    uint8_t *value;

    if ((battery == NULL) || (out == NULL) ||
        (battery->version != ZY100_CAL_BATTERY_GAIN_VERSION) ||
        (battery->sample_count == 0U) ||
        (battery->workflow_token == 0UL) ||
        (battery->reference_mv == 0U) ||
        (battery->uncalibrated_mv == 0U) ||
        (battery->gain_q20 == 0UL) ||
        ((current != NULL) &&
         !zy100_cal_record_validate(current, current_len, NULL)))
    {
        return false;
    }
    memset(out, 0, ZY100_CAL_RECORD_MAX_BYTES);
    if (current != NULL)
    {
        valid_flags = zy100_cal_get_u32_le(&current[12]) |
                      ZY100_CAL_VALID_BATTERY_GAIN;
        while (source_offset < current_len)
        {
            uint8_t type = current[source_offset];
            uint8_t tlv_len = current[source_offset + 1U];
            uint16_t tlv_bytes = (uint16_t)(ZY100_CAL_TLV_HEADER_BYTES + tlv_len);

            if (type != ZY100_CAL_TLV_BATTERY_GAIN_V1)
            {
                if ((uint16_t)(destination_offset + tlv_bytes) >
                    ZY100_CAL_RECORD_MAX_BYTES)
                {
                    return false;
                }
                memcpy(&out[destination_offset], &current[source_offset], tlv_bytes);
                destination_offset = (uint16_t)(destination_offset + tlv_bytes);
            }
            source_offset = (uint16_t)(source_offset + tlv_bytes);
        }
    }
    if ((uint16_t)(destination_offset + ZY100_CAL_TLV_HEADER_BYTES +
                   ZY100_CAL_TLV_BATTERY_GAIN_V1_BYTES) >
        ZY100_CAL_RECORD_MAX_BYTES)
    {
        return false;
    }
    out[destination_offset++] = ZY100_CAL_TLV_BATTERY_GAIN_V1;
    out[destination_offset++] = ZY100_CAL_TLV_BATTERY_GAIN_V1_BYTES;
    value = &out[destination_offset];
    value[0] = battery->version;
    value[1] = battery->sample_count;
    value[2] = 0U;
    value[3] = 0U;
    zy100_cal_put_u32_le(&value[4], battery->workflow_token);
    memcpy(&value[8], battery->bt_address, sizeof(battery->bt_address));
    zy100_cal_put_u16_le(&value[14], battery->reference_mv);
    zy100_cal_put_u16_le(&value[16], battery->uncalibrated_mv);
    zy100_cal_put_u32_le(&value[18], battery->gain_q20);
    zy100_cal_put_u32_le(&value[22], (uint32_t)battery->precal_delta_mv);
    value[26] = 0U;
    value[27] = 0U;
    destination_offset = (uint16_t)(destination_offset +
                                    ZY100_CAL_TLV_BATTERY_GAIN_V1_BYTES);

    zy100_cal_put_u32_le(&out[0], ZY100_CAL_RECORD_MAGIC);
    out[4] = ZY100_CAL_RECORD_VERSION;
    out[5] = ZY100_CAL_RECORD_HEADER_BYTES;
    zy100_cal_put_u16_le(&out[6], destination_offset);
    zy100_cal_put_u32_le(&out[8], generation);
    zy100_cal_put_u32_le(&out[12], valid_flags);
    zy100_cal_put_u16_le(&out[16], quality);
    zy100_cal_put_u64_le(&out[18], created_unix_ms);
    crc = zy100_crc32_ieee_begin();
    crc = zy100_crc32_ieee_update(crc, out, ZY100_CAL_RECORD_CRC_OFFSET);
    crc = zy100_crc32_ieee_update(crc, (const uint8_t *)&zero, 4U);
    crc = zy100_crc32_ieee_update(
              crc, &out[ZY100_CAL_RECORD_CRC_OFFSET + 4U],
              (uint32_t)destination_offset -
              (ZY100_CAL_RECORD_CRC_OFFSET + 4U));
    zy100_cal_put_u32_le(&out[ZY100_CAL_RECORD_CRC_OFFSET],
                        zy100_crc32_ieee_finish(crc));
    return zy100_cal_record_validate(out, destination_offset, info);
}

bool zy100_cal_rx_parse(const uint8_t *data,
                        uint16_t len,
                        zy100_cal_rx_frame_t *frame)
{
    uint16_t payload_len;

    if (frame != NULL)
    {
        memset(frame, 0, sizeof(*frame));
    }
    if ((data == NULL) || (frame == NULL) ||
        (len < ZY100_CAL_RX_HEADER_BYTES) ||
        (data[0] != ZY100_CAL_RX_MAGIC) ||
        (data[1] != ZY100_CAL_PROTOCOL_VERSION))
    {
        return false;
    }
    payload_len = zy100_cal_get_u16_le(&data[6]);
    if ((uint16_t)(ZY100_CAL_RX_HEADER_BYTES + payload_len) != len)
    {
        return false;
    }
    frame->opcode = data[2];
    frame->transaction_id = data[3];
    frame->offset = zy100_cal_get_u16_le(&data[4]);
    frame->payload_len = payload_len;
    frame->total_len = zy100_cal_get_u16_le(&data[8]);
    frame->crc32 = zy100_cal_get_u32_le(&data[10]);
    frame->payload = &data[ZY100_CAL_RX_HEADER_BYTES];
    return true;
}

uint16_t zy100_cal_build_tx_frame(uint8_t *out,
                                  uint16_t out_cap,
                                  uint8_t transaction_id,
                                  uint8_t frame_type,
                                  uint16_t offset,
                                  const uint8_t *payload,
                                  uint16_t payload_len,
                                  uint16_t total_len,
                                  uint32_t crc32)
{
    uint16_t bytes = (uint16_t)(ZY100_CAL_TX_HEADER_BYTES + payload_len);
    if ((out == NULL) || (out_cap < bytes) ||
        ((payload == NULL) && (payload_len != 0U)))
    {
        return 0U;
    }
    out[0] = ZY100_CAL_TX_MAGIC;
    out[1] = ZY100_CAL_PROTOCOL_VERSION;
    out[2] = frame_type;
    out[3] = transaction_id;
    out[4] = ((uint16_t)(offset + payload_len) == total_len) ? 1U : 0U;
    out[5] = 0U;
    zy100_cal_put_u16_le(&out[6], offset);
    zy100_cal_put_u16_le(&out[8], payload_len);
    zy100_cal_put_u16_le(&out[10], total_len);
    zy100_cal_put_u32_le(&out[12], crc32);
    if (payload_len != 0U)
    {
        memcpy(&out[ZY100_CAL_TX_HEADER_BYTES], payload, payload_len);
    }
    return bytes;
}

void zy100_cal_build_info(uint8_t out[ZY100_CAL_INFO_BYTES],
                          const zy100_cal_record_info_t *info,
                          bool valid)
{
    memset(out, 0, ZY100_CAL_INFO_BYTES);
    out[0] = ZY100_CAL_TX_MAGIC;
    out[1] = ZY100_CAL_PROTOCOL_VERSION;
    out[2] = valid ? ZY100_CAL_RECORD_VERSION : 0U;
    out[3] = valid ? 1U : 0U;
    if (valid && (info != NULL))
    {
        zy100_cal_put_u32_le(&out[4], info->generation);
        zy100_cal_put_u32_le(&out[8], info->valid_flags);
        zy100_cal_put_u16_le(&out[12], info->record_bytes);
        zy100_cal_put_u16_le(&out[14], info->quality);
        zy100_cal_put_u32_le(&out[16], info->crc32);
    }
}

bool zy100_cal_confirm_info_matches(
    const zy100_cal_rx_frame_t *frame,
    const zy100_cal_record_info_t *info,
    bool valid)
{
    uint8_t expected[ZY100_CAL_INFO_BYTES];
    uint32_t crc;

    if ((frame == NULL) || (frame->payload == NULL) ||
        (frame->offset != 0U) ||
        (frame->payload_len != ZY100_CAL_INFO_BYTES) ||
        (frame->total_len != ZY100_CAL_INFO_BYTES))
    {
        return false;
    }

    crc = zy100_crc32_ieee_begin();
    crc = zy100_crc32_ieee_update(crc, frame->payload,
                                  ZY100_CAL_INFO_BYTES);
    if (zy100_crc32_ieee_finish(crc) != frame->crc32)
    {
        return false;
    }

    zy100_cal_build_info(expected, info, valid);
    return memcmp(expected, frame->payload, sizeof(expected)) == 0;
}

bool zy100_cal_confirm_cached_record_matches(
    const zy100_cal_rx_frame_t *frame,
    const zy100_cal_record_info_t *info,
    bool valid)
{
    if (!valid || (frame == NULL) || (info == NULL) ||
        (frame->payload == NULL) || (frame->offset != 0U) ||
        (frame->payload_len != ZY100_CAL_CACHE_CONFIRM_BYTES) ||
        (frame->total_len != ZY100_CAL_CACHE_CONFIRM_BYTES) ||
        (frame->crc32 != info->crc32))
    {
        return false;
    }

    return (zy100_cal_get_u32_le(&frame->payload[0]) == info->generation) &&
           (zy100_cal_get_u32_le(&frame->payload[4]) == info->valid_flags) &&
           (zy100_cal_get_u16_le(&frame->payload[8]) == info->record_bytes) &&
           (frame->payload[10] == ZY100_CAL_RECORD_VERSION) &&
           (frame->payload[11] == 0U);
}

void zy100_cal_build_status(uint8_t out[ZY100_CAL_STATUS_BYTES],
                            uint8_t status,
                            uint8_t transaction_id,
                            uint16_t detail,
                            const zy100_cal_record_info_t *info)
{
    memset(out, 0, ZY100_CAL_STATUS_BYTES);
    out[0] = ZY100_CAL_TX_MAGIC;
    out[1] = ZY100_CAL_PROTOCOL_VERSION;
    out[2] = status;
    out[3] = transaction_id;
    zy100_cal_put_u16_le(&out[4], detail);
    if (info != NULL)
    {
        zy100_cal_put_u32_le(&out[8], info->generation);
        zy100_cal_put_u32_le(&out[12], info->crc32);
    }
}

static void zy100_cal_mag_diag_write_crc(
    uint8_t diagnostics[ZY100_CAL_MAG_DIAG_BYTES])
{
    uint32_t crc = zy100_crc32_ieee_begin();
    crc = zy100_crc32_ieee_update(crc, diagnostics,
                                  ZY100_CAL_MAG_DIAG_CRC_OFFSET);
    zy100_cal_put_u32_le(&diagnostics[ZY100_CAL_MAG_DIAG_CRC_OFFSET],
                        zy100_crc32_ieee_finish(crc));
}

bool zy100_cal_build_mag_diagnostics(
    uint8_t out[ZY100_CAL_MAG_DIAG_BYTES],
    const zy100_cal_mag_diag_t *diagnostics)
{
    uint8_t axis;

    if ((out == NULL) || (diagnostics == NULL))
    {
        return false;
    }
    memset(out, 0, ZY100_CAL_MAG_DIAG_BYTES);
    zy100_cal_put_u32_le(&out[0], ZY100_CAL_MAG_DIAG_MAGIC);
    out[4] = ZY100_CAL_MAG_DIAG_VERSION;
    out[5] = diagnostics->transaction_id;
    out[6] = diagnostics->completion_status;
    out[7] = diagnostics->final_model;
    zy100_cal_put_u32_le(&out[8], diagnostics->sample_count);
    zy100_cal_put_u32_le(&out[12], diagnostics->rejected_sample_count);
    zy100_cal_put_u32_le(&out[16], diagnostics->sensor_read_errors);
    zy100_cal_put_u32_le(&out[20], diagnostics->elapsed_ms);
    out[24] = diagnostics->fit_attempts;
    out[25] = diagnostics->coverage_mask;
    out[26] = diagnostics->direction_mask;
    out[27] = diagnostics->final_ready_status;
    out[28] = diagnostics->full_solve_status;
    out[29] = diagnostics->axis_solve_status;
    out[30] = diagnostics->hard_solve_status;
    zy100_cal_put_u16_le(&out[32], diagnostics->quality);
    zy100_cal_put_u32_le(&out[36], diagnostics->rms_x1e6);
    for (axis = 0U; axis < 3U; axis++)
    {
        zy100_cal_put_u32_le(&out[40U + (uint16_t)axis * 4U],
                            diagnostics->min_value[axis]);
        zy100_cal_put_u32_le(&out[52U + (uint16_t)axis * 4U],
                            diagnostics->max_value[axis]);
    }
    zy100_cal_mag_diag_write_crc(out);
    return true;
}

bool zy100_cal_update_mag_diag_completion(
    uint8_t diagnostics[ZY100_CAL_MAG_DIAG_BYTES],
    uint8_t completion_status)
{
    if ((diagnostics == NULL) ||
        (zy100_cal_get_u32_le(&diagnostics[0]) !=
         ZY100_CAL_MAG_DIAG_MAGIC) ||
        (diagnostics[4] != ZY100_CAL_MAG_DIAG_VERSION))
    {
        return false;
    }
    diagnostics[6] = completion_status;
    zy100_cal_mag_diag_write_crc(diagnostics);
    return true;
}
