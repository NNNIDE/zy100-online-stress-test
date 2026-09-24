#include "zp_mfg_rules.h"

#include <string.h>

#define ZP_MFG_SEQ_OFFSET       357172UL
#define ZP_MFG_SEQ_MOD          1000000UL

static bool zp_mfg_is_digit(char c)
{
    return (c >= '0') && (c <= '9');
}

static bool zp_mfg_fixed_len_nonzero(const char *s, uint8_t len)
{
    uint8_t idx;

    if (s == NULL)
    {
        return false;
    }
    for (idx = 0U; idx < len; idx++)
    {
        if (s[idx] == '\0')
        {
            return false;
        }
    }
    return s[len] == '\0';
}

bool zp_mfg_encode_seq6(uint32_t raw_internal_seq,
                        char encoded[ZP_MFG_ENCODED_SEQ_LEN + 1U])
{
    uint32_t value;
    uint8_t idx;

    if ((encoded == NULL) ||
        (raw_internal_seq == 0UL) ||
        (raw_internal_seq > 999999UL))
    {
        return false;
    }

    value = (raw_internal_seq + ZP_MFG_SEQ_OFFSET) % ZP_MFG_SEQ_MOD;
    for (idx = 0U; idx < ZP_MFG_ENCODED_SEQ_LEN; idx++)
    {
        uint8_t pos = (uint8_t)(ZP_MFG_ENCODED_SEQ_LEN - 1U - idx);
        encoded[pos] = (char)('0' + (value % 10UL));
        value /= 10UL;
    }
    encoded[ZP_MFG_ENCODED_SEQ_LEN] = '\0';
    return true;
}

bool zp_mfg_sn_validate(const char product_code[ZP_MFG_PRODUCT_CODE_LEN + 1U],
                        const char final_sn[ZP_MFG_FINAL_SN_LEN + 1U],
                        uint32_t raw_internal_seq,
                        const char date_ymd[ZP_MFG_DATE_YMD_LEN + 1U])
{
    char expected_seq[ZP_MFG_ENCODED_SEQ_LEN + 1U];
    const char *tail;
    uint8_t idx;
    char sn_date[ZP_MFG_DATE_YMD_LEN + 1U];
    char sn_seq[ZP_MFG_ENCODED_SEQ_LEN + 1U];

    if (!zp_mfg_fixed_len_nonzero(product_code, ZP_MFG_PRODUCT_CODE_LEN) ||
        !zp_mfg_fixed_len_nonzero(final_sn, ZP_MFG_FINAL_SN_LEN) ||
        !zp_mfg_fixed_len_nonzero(date_ymd, ZP_MFG_DATE_YMD_LEN))
    {
        return false;
    }

    if (memcmp(product_code, final_sn, ZP_MFG_PRODUCT_CODE_LEN) != 0)
    {
        return false;
    }

    tail = &final_sn[ZP_MFG_PRODUCT_CODE_LEN];
    for (idx = 0U; idx < 12U; idx++)
    {
        if (!zp_mfg_is_digit(tail[idx]))
        {
            return false;
        }
    }

    if (!zp_mfg_encode_seq6(raw_internal_seq, expected_seq))
    {
        return false;
    }

    sn_date[0] = tail[0];
    sn_date[1] = tail[6];
    sn_date[2] = tail[2];
    sn_date[3] = tail[8];
    sn_date[4] = tail[4];
    sn_date[5] = tail[10];
    sn_date[6] = '\0';

    sn_seq[0] = tail[3];
    sn_seq[1] = tail[7];
    sn_seq[2] = tail[9];
    sn_seq[3] = tail[1];
    sn_seq[4] = tail[11];
    sn_seq[5] = tail[5];
    sn_seq[6] = '\0';

    return (memcmp(sn_date, date_ymd, ZP_MFG_DATE_YMD_LEN) == 0) &&
           (memcmp(sn_seq, expected_seq, ZP_MFG_ENCODED_SEQ_LEN) == 0);
}

bool zp_mfg_ble_name_validate(
    const char ble_name[ZP_MFG_BLE_NAME_LEN + 1U])
{
    uint8_t idx;

    if (!zp_mfg_fixed_len_nonzero(ble_name, ZP_MFG_BLE_NAME_LEN))
    {
        return false;
    }
    if ((ble_name[0] != 'Z') ||
        (ble_name[1] != 'P') ||
        (ble_name[2] != '-'))
    {
        return false;
    }
    for (idx = 3U; idx < ZP_MFG_BLE_NAME_LEN; idx++)
    {
        if (!zp_mfg_is_digit(ble_name[idx]))
        {
            return false;
        }
    }
    return true;
}

bool zp_mfg_identifier_code_validate(const char *code, uint8_t code_len)
{
    uint8_t idx;

    if ((code == NULL) || (code_len == 0U))
    {
        return false;
    }
    for (idx = 0U; idx < code_len; idx++)
    {
        if ((code[idx] < 'A') || (code[idx] > 'Z'))
        {
            return false;
        }
    }
    return code[code_len] == '\0';
}
