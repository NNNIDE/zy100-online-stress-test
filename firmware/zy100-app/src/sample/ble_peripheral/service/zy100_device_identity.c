#include "zy100_device_identity.h"

#include <stdbool.h>
#include <string.h>

#define ZY100_BLE_NAME_DIGITS 6U
#define ZY100_SN_DIGITS       8U
#define ZY100_BLE_NAME_MOD    1000000ULL
#define ZY100_SN_MOD          100000000ULL
#define ZY100_MFG_BLE_NAME_LEN 9U
#define ZY100_MFG_SN_MAX_LEN   16U

static char s_ble_name[sizeof("ZP-000000")];
static char s_sn[ZY100_MFG_SN_MAX_LEN + 1U];
static bool s_identity_ready = false;
static bool s_identity_mfg_applied = false;

static bool zy100_identity_copy_fixed_string(char *dst,
                                             uint8_t dst_size,
                                             const char *src,
                                             uint8_t exact_len)
{
    uint8_t idx;

    if ((dst == NULL) || (src == NULL) || (dst_size <= exact_len))
    {
        return false;
    }

    for (idx = 0U; idx < exact_len; idx++)
    {
        if (src[idx] == '\0')
        {
            return false;
        }
        dst[idx] = src[idx];
    }
    dst[exact_len] = '\0';
    return true;
}

static uint32_t zy100_identity_value(uint64_t mod)
{
    uint64_t id = (uint64_t)ZY100_DEVICE_INTERNAL_ID;
    uint64_t value = (id * (uint64_t)ZY100_IDENTITY_OFFSET) % mod;

    return (uint32_t)value;
}

static void zy100_identity_fill_fixed_decimal(char *dst, uint8_t digits, uint32_t value)
{
    uint8_t idx;

    for (idx = 0U; idx < digits; idx++)
    {
        uint8_t pos = (uint8_t)(digits - 1U - idx);
        dst[pos] = (char)('0' + (value % 10U));
        value /= 10U;
    }
}

static void zy100_identity_init_once(void)
{
    if (s_identity_ready || s_identity_mfg_applied)
    {
        return;
    }

    s_ble_name[0] = 'Z';
    s_ble_name[1] = 'P';
    s_ble_name[2] = '-';
    zy100_identity_fill_fixed_decimal(&s_ble_name[3],
                                      ZY100_BLE_NAME_DIGITS,
                                      zy100_identity_value(ZY100_BLE_NAME_MOD));
    s_ble_name[9] = '\0';

    s_sn[0] = 'Z';
    s_sn[1] = 'P';
    s_sn[2] = '-';
    zy100_identity_fill_fixed_decimal(&s_sn[3],
                                      ZY100_SN_DIGITS,
                                      zy100_identity_value(ZY100_SN_MOD));
    s_sn[11] = '\0';

    s_identity_ready = true;
}

const char *zy100_device_ble_name(void)
{
    zy100_identity_init_once();
    return s_ble_name;
}

const char *zy100_device_sn(void)
{
    zy100_identity_init_once();
    return s_sn;
}

uint32_t zy100_device_internal_id(void)
{
    return (uint32_t)ZY100_DEVICE_INTERNAL_ID;
}

bool zy100_device_identity_apply_mfg(const char *ble_name, const char *sn)
{
    char next_ble_name[sizeof(s_ble_name)];
    char next_sn[sizeof(s_sn)];
    uint8_t sn_len = 0U;

    if ((ble_name == NULL) || (sn == NULL))
    {
        return false;
    }

    if (!zy100_identity_copy_fixed_string(next_ble_name,
                                          (uint8_t)sizeof(next_ble_name),
                                          ble_name,
                                          ZY100_MFG_BLE_NAME_LEN))
    {
        return false;
    }

    while ((sn[sn_len] != '\0') && (sn_len < ZY100_MFG_SN_MAX_LEN))
    {
        next_sn[sn_len] = sn[sn_len];
        sn_len++;
    }
    if ((sn_len == 0U) || (sn_len > ZY100_MFG_SN_MAX_LEN))
    {
        return false;
    }
    next_sn[sn_len] = '\0';

    memcpy(s_ble_name, next_ble_name, sizeof(s_ble_name));
    memcpy(s_sn, next_sn, sizeof(s_sn));
    s_identity_ready = true;
    s_identity_mfg_applied = true;
    return true;
}
