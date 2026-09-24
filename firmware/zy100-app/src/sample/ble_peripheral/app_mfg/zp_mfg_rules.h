#ifndef ZP_MFG_RULES_H
#define ZP_MFG_RULES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "zp_mfg_record.h"

bool zp_mfg_encode_seq6(uint32_t raw_internal_seq,
                        char encoded[ZP_MFG_ENCODED_SEQ_LEN + 1U]);
bool zp_mfg_sn_validate(const char product_code[ZP_MFG_PRODUCT_CODE_LEN + 1U],
                        const char final_sn[ZP_MFG_FINAL_SN_LEN + 1U],
                        uint32_t raw_internal_seq,
                        const char date_ymd[ZP_MFG_DATE_YMD_LEN + 1U]);
bool zp_mfg_ble_name_validate(
    const char ble_name[ZP_MFG_BLE_NAME_LEN + 1U]);
bool zp_mfg_identifier_code_validate(const char *code, uint8_t code_len);

#ifdef __cplusplus
}
#endif

#endif /* ZP_MFG_RULES_H */
