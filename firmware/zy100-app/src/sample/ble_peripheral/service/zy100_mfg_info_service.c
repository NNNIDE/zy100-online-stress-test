#include "zy100_mfg_info_service.h"

#include <stdbool.h>
#include <string.h>

#include <gatt.h>
#include <trace.h>
#include <version.h>

#include "../app_build_config.h"
#include "../app_factory/factory_boot_gate.h"
#include "../app_mfg/zp_mfg_protocol.h"
#include "../app_mfg/zp_mfg_rules.h"
#include "../app_mfg/zp_mfg_store.h"
#include "zy100_production_acceptance.h"
#include "zy100_system_info_store.h"
#include "zy100_whole_unit_test.h"

#define ZY100_MFG_INFO_VALUE_INDEX       0x02U
#define ZY100_DEVICE_CHANNEL_VALUE_INDEX 0x04U
#define ZY100_MFG_INFO_MAX_BYTES         512U
#define ZY100_MFG_INFO_INVALID_CONN_ID   0xFFU
#define ZY100_ACCEPTANCE_INFO_SAFE_BYTES 240U
#define ZY100_MFG_U32_MAX_DIGITS         10U
#define ZY100_ACCEPTANCE_INFO_MAX_BYTES  \
    ((sizeof("contract=acceptance_info_v2;mode=production_acceptance;fw_code=") - 1U) + \
     ZY100_MFG_U32_MAX_DIGITS + \
     (sizeof(";mfg_record_version=") - 1U) + ZY100_MFG_U32_MAX_DIGITS + \
     (sizeof(";mfg_payload_crc=") - 1U) + ZY100_MFG_U32_MAX_DIGITS + \
     (sizeof(";si=") - 1U) + ZY100_MFG_U32_MAX_DIGITS + \
     (sizeof(";wu=") - 1U) + ZY100_MFG_U32_MAX_DIGITS + \
     (sizeof(";sn=") - 1U) + ZP_MFG_FINAL_SN_LEN + \
     (sizeof(";name=") - 1U) + ZP_MFG_BLE_NAME_LEN + \
     (sizeof(";channel=") - 1U) + ZP_MFG_CHANNEL_CODE_LEN + \
     (sizeof(";environment=") - 1U) + ZP_MFG_ENVIRONMENT_CODE_LEN + \
     (sizeof(";end=1") - 1U))

typedef char zy100_acceptance_info_must_fit_single_windows_read[
    (ZY100_ACCEPTANCE_INFO_MAX_BYTES <= ZY100_ACCEPTANCE_INFO_SAFE_BYTES) ? 1 : -1];

static const uint8_t s_mfg_info_service_uuid[16] =
{
    0x00, 0x20, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
    0x4D, 0x4F, 0x9D, 0x4A, 0x00, 0x00, 0xCA, 0x9E
};

static T_SERVER_ID s_mfg_info_service_id = 0xFFU;
static uint8_t s_mfg_info_value[ZY100_MFG_INFO_MAX_BYTES];
#if ZY100_BUILD_PRODUCTION
static uint8_t s_device_channel_value[sizeof("DEFAULT") - 1U];
#endif
static uint16_t s_mfg_info_read_len;
static uint8_t s_mfg_info_read_conn_id = ZY100_MFG_INFO_INVALID_CONN_ID;
static bool s_mfg_info_read_active;

static T_ATTRIB_APPL s_mfg_info_attr_tbl[] =
{
    {
        (ATTRIB_FLAG_VOID | ATTRIB_FLAG_LE),
        {
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
        },
        UUID_128BIT_SIZE,
        (void *)s_mfg_info_service_uuid,
        GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_INCL,
        {
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ
        },
        1,
        NULL,
        GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,
        {
            0x00, 0x20, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
            0x4D, 0x4F, 0x9D, 0x4A, 0x01, 0x00, 0xCA, 0x9E
        },
        0,
        NULL,
        GATT_PERM_READ
    },
#if ZY100_BUILD_PRODUCTION
    {
        ATTRIB_FLAG_VALUE_INCL,
        {
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ
        },
        1,
        NULL,
        GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,
        {
            0x00, 0x20, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
            0x4D, 0x4F, 0x9D, 0x4A, 0x13, 0x00, 0xCA, 0x9E
        },
        0,
        NULL,
        GATT_PERM_READ
    },
#endif
};

static uint16_t zy100_mfg_info_append(uint8_t *out,
                                      uint16_t pos,
                                      uint16_t out_len,
                                      const char *text)
{
    if (text == NULL)
    {
        return pos;
    }
    while (*text != '\0')
    {
        if ((out != NULL) && (pos < out_len))
        {
            out[pos] = (uint8_t)*text;
        }
        pos++;
        text++;
    }
    return pos;
}

static uint16_t zy100_mfg_info_append_u32(uint8_t *out,
                                          uint16_t pos,
                                          uint16_t out_len,
                                          uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do
    {
        digits[count++] = (char)('0' + (value % 10UL));
        value /= 10UL;
    } while ((value != 0UL) && (count < (uint8_t)sizeof(digits)));

    while (count != 0U)
    {
        char ch[2];
        ch[0] = digits[--count];
        ch[1] = '\0';
        pos = zy100_mfg_info_append(out, pos, out_len, ch);
    }
    return pos;
}

static uint16_t zy100_mfg_info_append_hex_byte(uint8_t *out,
                                                uint16_t pos,
                                                uint16_t out_len,
                                                uint8_t value)
{
    static const char digits[] = "0123456789ABCDEF";
    char text[3];

    text[0] = digits[(value >> 4U) & 0x0FU];
    text[1] = digits[value & 0x0FU];
    text[2] = '\0';
    return zy100_mfg_info_append(out, pos, out_len, text);
}

static const char *zy100_mfg_info_training_target_name(uint8_t target)
{
    switch (target)
    {
    case ZP_MFG_TRAINING_LED_TARGET_NOTIFY:
        return "notify";
    case ZP_MFG_TRAINING_LED_TARGET_LOGO:
        return "logo";
    case ZP_MFG_TRAINING_LED_TARGET_ALL:
        return "all";
    default:
        return "unknown";
    }
}

static const char *zy100_mfg_info_training_effect_name(uint8_t effect)
{
    switch (effect)
    {
    case ZP_MFG_TRAINING_LED_EFFECT_SOLID:
        return "solid";
    case ZP_MFG_TRAINING_LED_EFFECT_BLINK:
        return "blink";
    case ZP_MFG_TRAINING_LED_EFFECT_BREATH:
        return "breath";
    case ZP_MFG_TRAINING_LED_EFFECT_MARQUEE:
        return "marquee";
    default:
        return "unknown";
    }
}

static const char *zy100_mfg_info_training_speed_name(uint8_t speed)
{
    switch (speed)
    {
    case ZP_MFG_TRAINING_LED_SPEED_CAPTURE_DEFAULT:
        return "capture_default";
    case ZP_MFG_TRAINING_LED_SPEED_SLOW:
        return "slow";
    case ZP_MFG_TRAINING_LED_SPEED_DEFAULT:
        return "default";
    case ZP_MFG_TRAINING_LED_SPEED_FAST:
        return "fast";
    default:
        return "unknown";
    }
}

static uint16_t zy100_mfg_info_append_training_config(
    uint8_t *out,
    uint16_t pos,
    uint16_t out_len,
    const zp_mfg_record_t *record)
{
    zp_mfg_training_led_config_t config;

    if (!zp_mfg_training_led_config_get_record(record, &config))
    {
        return zy100_mfg_info_append(out,
                                     pos,
                                     out_len,
                                     ";training_config_version=unknown");
    }

    pos = zy100_mfg_info_append(out, pos, out_len, ";training_config_version=");
    pos = zy100_mfg_info_append_u32(out, pos, out_len, config.schema_version);
    pos = zy100_mfg_info_append(out, pos, out_len, ";auto_capture=");
    pos = zy100_mfg_info_append_u32(out, pos, out_len, config.auto_capture_enabled);
    pos = zy100_mfg_info_append(out, pos, out_len, ";training_led_enabled=");
    pos = zy100_mfg_info_append_u32(
        out,
        pos,
        out_len,
        ((config.flags & ZP_MFG_TRAINING_LED_FLAG_ENABLED) != 0U) ? 1U : 0U);
    pos = zy100_mfg_info_append(out, pos, out_len, ";training_led_target=");
    pos = zy100_mfg_info_append(out,
                                pos,
                                out_len,
                                zy100_mfg_info_training_target_name(config.target));
    pos = zy100_mfg_info_append(out, pos, out_len, ";training_led_effect=");
    pos = zy100_mfg_info_append(out,
                                pos,
                                out_len,
                                zy100_mfg_info_training_effect_name(config.effect));
    pos = zy100_mfg_info_append(out, pos, out_len, ";training_led_rgb=");
    pos = zy100_mfg_info_append_hex_byte(out, pos, out_len, config.red);
    pos = zy100_mfg_info_append_hex_byte(out, pos, out_len, config.green);
    pos = zy100_mfg_info_append_hex_byte(out, pos, out_len, config.blue);
    pos = zy100_mfg_info_append(out, pos, out_len, ";training_led_brightness=");
    pos = zy100_mfg_info_append_u32(out, pos, out_len, config.brightness_percent);
    pos = zy100_mfg_info_append(out, pos, out_len, ";training_led_speed=");
    pos = zy100_mfg_info_append(out,
                                pos,
                                out_len,
                                zy100_mfg_info_training_speed_name(config.speed));
    pos = zy100_mfg_info_append(out, pos, out_len, ";training_led_reverse=");
    return zy100_mfg_info_append_u32(
        out,
        pos,
        out_len,
        ((config.flags & ZP_MFG_TRAINING_LED_FLAG_MARQUEE_REVERSE) != 0U) ?
        1U : 0U);
}

static const char *zy100_mfg_info_mode(void)
{
#if ZY100_BUILD_PRODUCTION
    if (zy100_production_acceptance_active())
    {
        return "production_acceptance";
    }
    if (zy100_whole_unit_test_active())
    {
        return "whole_unit";
    }
    return factory_boot_gate_production_blocked() ?
           "production_blocked" : "locked";
#else
    if (factory_boot_gate_locked_record_invalid())
    {
        return "factory_safe_recovery";
    }
    return factory_boot_gate_locked_user_mode_active() ?
           "factory_locked_wait" : "factory";
#endif
}

static uint16_t zy100_mfg_info_build_acceptance(
    uint8_t *out,
    uint16_t out_len,
    const zp_mfg_record_t *record,
    bool channel_valid,
    bool environment_valid,
    const char *mode)
{
    uint16_t pos = 0U;
    zy100_whole_unit_state_t whole_state = ZY100_WHOLE_UNIT_NOT_REQUIRED;
    bool whole_state_valid = zy100_system_info_get_whole_unit_state(&whole_state);

    pos = zy100_mfg_info_append(
              out, pos, out_len,
              "contract=acceptance_info_v2;mode=");
    pos = zy100_mfg_info_append(out, pos, out_len, mode);
    pos = zy100_mfg_info_append(out, pos, out_len, ";fw_code=");
    pos = zy100_mfg_info_append_u32(out, pos, out_len, VERSION_CODE);
    pos = zy100_mfg_info_append(out, pos, out_len, ";mfg_record_version=");
    pos = zy100_mfg_info_append_u32(
              out, pos, out_len, (record != NULL) ? record->version : 0U);
    pos = zy100_mfg_info_append(out, pos, out_len, ";mfg_payload_crc=");
    pos = zy100_mfg_info_append_u32(
              out, pos, out_len,
              (record != NULL) ? record->reserved_u32[0] : 0UL);
    pos = zy100_mfg_info_append(out, pos, out_len, ";si=");
    pos = zy100_mfg_info_append_u32(
              out, pos, out_len, ZY100_SYSTEM_INFO_SCHEMA_VERSION);
    pos = zy100_mfg_info_append(out, pos, out_len, ";wu=");
    pos = zy100_mfg_info_append_u32(
              out, pos, out_len,
              whole_state_valid ? (uint32_t)whole_state : 255UL);
    pos = zy100_mfg_info_append(out, pos, out_len, ";sn=");
    pos = zy100_mfg_info_append(
              out, pos, out_len, (record != NULL) ? record->final_sn : "");
    pos = zy100_mfg_info_append(out, pos, out_len, ";name=");
    pos = zy100_mfg_info_append(
              out, pos, out_len, (record != NULL) ? record->ble_adv_name : "");
    pos = zy100_mfg_info_append(out, pos, out_len, ";channel=");
    pos = zy100_mfg_info_append(
              out, pos, out_len,
              ((record != NULL) && channel_valid) ?
              record->channel_code : "unknown");
    pos = zy100_mfg_info_append(out, pos, out_len, ";environment=");
    pos = zy100_mfg_info_append(
              out, pos, out_len,
              ((record != NULL) && environment_valid) ?
              record->environment_code : "unknown");
    pos = zy100_mfg_info_append(out, pos, out_len, ";end=1");
    if ((out != NULL) && (out_len != 0U))
    {
        out[(pos < out_len) ? pos : (uint16_t)(out_len - 1U)] = '\0';
    }
    return (pos < out_len) ? pos : out_len;
}

static uint16_t zy100_mfg_info_build(uint8_t *out, uint16_t out_len)
{
    const zp_mfg_record_t *record = factory_boot_gate_record();
    bool channel_valid = false;
    bool environment_valid = false;
    bool imu_cal_valid = false;
    uint16_t pos = 0U;

    if (record != NULL)
    {
        channel_valid =
            zp_mfg_identifier_code_validate(record->channel_code,
                                            ZP_MFG_CHANNEL_CODE_LEN);
        environment_valid =
            zp_mfg_identifier_code_validate(record->environment_code,
                                            ZP_MFG_ENVIRONMENT_CODE_LEN);
        imu_cal_valid =
            ((record->version == ZP_MFG_RECORD_VERSION_V4) ||
             (record->version == ZP_MFG_RECORD_VERSION_V5)) &&
            zp_mfg_store_imu_cal_valid(&record->imu_cal);
    }

    if (zy100_production_acceptance_active())
    {
        return zy100_mfg_info_build_acceptance(
                   out, out_len, record, channel_valid, environment_valid,
                   "production_acceptance");
    }
    if (zy100_whole_unit_test_active())
    {
        return zy100_mfg_info_build_acceptance(
                   out, out_len, record, channel_valid, environment_valid,
                   "whole_unit");
    }

    pos = zy100_mfg_info_append(out, pos, out_len, "mode=");
    pos = zy100_mfg_info_append(out, pos, out_len, zy100_mfg_info_mode());
    pos = zy100_mfg_info_append(out, pos, out_len, ";fw_code=");
    pos = zy100_mfg_info_append_u32(out, pos, out_len, VERSION_CODE);
    pos = zy100_mfg_info_append(out, pos, out_len, ";reason=");
    pos = zy100_mfg_info_append(out,
                                pos,
                                out_len,
                                factory_boot_gate_reason());
    pos = zy100_mfg_info_append(out, pos, out_len, ";locked_invalid=");
    pos = zy100_mfg_info_append(out,
                                pos,
                                out_len,
                                factory_boot_gate_locked_record_invalid() ?
                                "1" : "0");
    pos = zy100_mfg_info_append(out, pos, out_len, ";mfg_record_version=");
    pos = zy100_mfg_info_append_u32(out,
                                    pos,
                                    out_len,
                                    (record != NULL) ? record->version : 0U);
    pos = zy100_mfg_info_append(out, pos, out_len, ";mfg_payload_crc=");
    pos = zy100_mfg_info_append_u32(
              out, pos, out_len,
              (record != NULL) ? record->reserved_u32[0] : 0UL);
    pos = zy100_mfg_info_append(out, pos, out_len, ";imu_cal_valid=");
    pos = zy100_mfg_info_append(out,
                                pos,
                                out_len,
                                imu_cal_valid ? "1" : "0");
    pos = zy100_mfg_info_append(out, pos, out_len, ";channel=");
    pos = zy100_mfg_info_append(out,
                                pos,
                                out_len,
                                channel_valid ?
                                record->channel_code : "unknown");
    pos = zy100_mfg_info_append(out, pos, out_len, ";environment=");
    pos = zy100_mfg_info_append(out,
                                pos,
                                out_len,
                                environment_valid ?
                                record->environment_code : "unknown");
    if (record != NULL)
    {
        pos = zy100_mfg_info_append_training_config(out, pos, out_len, record);
    }
    if (record != NULL)
    {
        pos = zy100_mfg_info_append(out, pos, out_len, ";sn=");
        pos = zy100_mfg_info_append(out, pos, out_len, record->final_sn);
        pos = zy100_mfg_info_append(out, pos, out_len, ";name=");
        pos = zy100_mfg_info_append(out, pos, out_len, record->ble_adv_name);
    }
    if ((out != NULL) && (out_len != 0U))
    {
        out[(pos < out_len) ? pos : (uint16_t)(out_len - 1U)] = '\0';
    }
    return (pos < out_len) ? pos : out_len;
}

#if ZY100_BUILD_PRODUCTION
static uint16_t zy100_device_channel_build(uint8_t *out, uint16_t out_len)
{
    static const char default_channel[] = "DEFAULT";
    const char *channel = default_channel;
    const zp_mfg_record_t *record = factory_boot_gate_record();

    if ((record != NULL) &&
        zp_mfg_identifier_code_validate(record->channel_code,
                                        ZP_MFG_CHANNEL_CODE_LEN))
    {
        if (memcmp(record->channel_code, "CG", ZP_MFG_CHANNEL_CODE_LEN) == 0)
        {
            channel = "CG";
        }
        else if (memcmp(record->channel_code, "ST", ZP_MFG_CHANNEL_CODE_LEN) == 0)
        {
            channel = "ST";
        }
    }
    return zy100_mfg_info_append(out, 0U, out_len, channel);
}
#endif

static T_APP_RESULT zy100_mfg_info_read_cb(uint8_t conn_id,
                                            T_SERVER_ID service_id,
                                            uint16_t attrib_index,
                                            uint16_t offset,
                                            uint16_t *p_length,
                                            uint8_t **pp_value)
{
    uint16_t len;
    uint16_t returned_len;
    T_APP_RESULT status;

    (void)service_id;
#if ZY100_BUILD_PRODUCTION
    if (attrib_index == ZY100_DEVICE_CHANNEL_VALUE_INDEX)
    {
        if ((p_length == NULL) || (pp_value == NULL))
        {
            return APP_RESULT_ATTR_NOT_FOUND;
        }
        if (offset != 0U)
        {
            status = APP_RESULT_INVALID_OFFSET;
            ZY100_LOG_ROUTINE(DBG_DIRECT, "[DEVICE_CHANNEL_READ] conn_id=%u offset=%u returned_len=0 callback_status=%u",
                       (uint32_t)conn_id,
                       (uint32_t)offset,
                       (uint32_t)status);
            return status;
        }
        *p_length = zy100_device_channel_build(
            s_device_channel_value,
            (uint16_t)sizeof(s_device_channel_value));
        *pp_value = s_device_channel_value;
        status = APP_RESULT_SUCCESS;
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[DEVICE_CHANNEL_READ] conn_id=%u offset=0 returned_len=%u callback_status=%u",
                   (uint32_t)conn_id,
                   (uint32_t)*p_length,
                   (uint32_t)status);
        return status;
    }
#endif
    if ((attrib_index != ZY100_MFG_INFO_VALUE_INDEX) ||
        (p_length == NULL) ||
        (pp_value == NULL))
    {
        status = APP_RESULT_ATTR_NOT_FOUND;
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[MFG_INFO_READ] conn_id=%u offset=%u total_len=%u returned_len=0 callback_status=%u",
                   (uint32_t)conn_id,
                   (uint32_t)offset,
                   0UL,
                   (uint32_t)status);
        return status;
    }

    if (offset == 0U)
    {
        len = zy100_mfg_info_build(s_mfg_info_value,
                                   (uint16_t)sizeof(s_mfg_info_value));
        s_mfg_info_read_len = len;
        s_mfg_info_read_conn_id = conn_id;
        s_mfg_info_read_active = true;
        if (zy100_production_acceptance_active())
        {
            ZY100_LOG_ROUTINE(DBG_DIRECT, "[PROD_ACCEPT] info contract=acceptance_info_v2 len=%u complete=%u",
                       (uint32_t)len,
                       (len <= ZY100_ACCEPTANCE_INFO_SAFE_BYTES) ? 1U : 0U);
        }
        else
        {
            ZY100_LOG_ROUTINE(DBG_DIRECT, "[MFG_INFO] len=%u cap=%u",
                       (uint32_t)len,
                       (uint32_t)sizeof(s_mfg_info_value));
        }
    }
    else
    {
        if ((!s_mfg_info_read_active) ||
            (s_mfg_info_read_conn_id != conn_id))
        {
            status = APP_RESULT_INVALID_OFFSET;
            ZY100_LOG_ROUTINE(DBG_DIRECT, "[MFG_INFO_READ] conn_id=%u offset=%u total_len=%u returned_len=0 callback_status=%u",
                       (uint32_t)conn_id,
                       (uint32_t)offset,
                       (uint32_t)s_mfg_info_read_len,
                       (uint32_t)status);
            return status;
        }
        len = s_mfg_info_read_len;
    }
    if (offset > len)
    {
        status = APP_RESULT_INVALID_OFFSET;
        s_mfg_info_read_active = false;
        s_mfg_info_read_conn_id = ZY100_MFG_INFO_INVALID_CONN_ID;
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[MFG_INFO_READ] conn_id=%u offset=%u total_len=%u returned_len=0 callback_status=%u",
                   (uint32_t)conn_id,
                   (uint32_t)offset,
                   (uint32_t)len,
                   (uint32_t)status);
        return status;
    }
    returned_len = (uint16_t)(len - offset);
    *pp_value = s_mfg_info_value + offset;
    *p_length = returned_len;
    status = APP_RESULT_SUCCESS;
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[MFG_INFO_READ] conn_id=%u offset=%u total_len=%u returned_len=%u callback_status=%u",
               (uint32_t)conn_id,
               (uint32_t)offset,
               (uint32_t)len,
               (uint32_t)returned_len,
               (uint32_t)status);
    if (offset == len)
    {
        s_mfg_info_read_active = false;
        s_mfg_info_read_conn_id = ZY100_MFG_INFO_INVALID_CONN_ID;
    }
    return APP_RESULT_SUCCESS;
}

static const T_FUN_GATT_SERVICE_CBS s_mfg_info_cbs =
{
    zy100_mfg_info_read_cb,
    NULL,
    NULL
};

T_SERVER_ID zy100_mfg_info_service_add(void *app_profile_callback)
{
    (void)app_profile_callback;
    if (false == server_add_service(&s_mfg_info_service_id,
                                    (uint8_t *)s_mfg_info_attr_tbl,
                                    sizeof(s_mfg_info_attr_tbl),
                                    s_mfg_info_cbs))
    {
        s_mfg_info_service_id = 0xFFU;
        APP_PRINT_ERROR0("zy100_mfg_info_service_add: fail");
    }
    return s_mfg_info_service_id;
}

void zy100_mfg_info_service_reset(uint8_t conn_id)
{
    (void)conn_id;
}
