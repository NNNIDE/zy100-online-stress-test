#include "factory_boot_gate.h"
#include "../app_build_config.h"

#include <string.h>

#include <trace.h>

#include "../app_mfg/zp_mfg_protocol.h"
#include "../app_mfg/zp_mfg_store.h"

#if ZY100_BUILD_PRODUCTION
#include <version.h>
#include "factory_p2_contract.h"

static const char *factory_boot_gate_test_name(uint32_t bit)
{
    switch (bit)
    {
    case ZP_MFG_TEST_LED_SHOWCASE: return "LED_SHOWCASE";
    case ZP_MFG_TEST_FACTORY_CLEANUP: return "FACTORY_CLEANUP";
    case ZP_MFG_TEST_EXT_FLASH_ID: return "EXT_FLASH_ID";
    case ZP_MFG_TEST_EXT_FLASH_UID: return "EXT_FLASH_UID";
    case ZP_MFG_TEST_EXT_FLASH_RW: return "EXT_FLASH_RW";
    case ZP_MFG_TEST_IMU: return "IMU";
    case ZP_MFG_TEST_MAG: return "MAG";
    case ZP_MFG_TEST_YHM2712: return "YHM2712";
    case ZP_MFG_TEST_BATTERY_ADC: return "BATTERY_ADC";
    case ZP_MFG_TEST_CHARGE_STATUS: return "CHARGE_STATUS";
    case ZP_MFG_TEST_KEY: return "KEY";
    case ZP_MFG_TEST_BLE_ADV: return "BLE_ADV";
    case ZP_MFG_TEST_BLE_CONNECT: return "BLE_CONNECT";
    case ZP_MFG_TEST_RSSI: return "RSSI";
    case ZP_MFG_TEST_LED_RED: return "LED_RED";
    case ZP_MFG_TEST_LED_GREEN: return "LED_GREEN";
    case ZP_MFG_TEST_LED_BLUE: return "LED_BLUE";
    case ZP_MFG_TEST_LED_RGB_SEQUENCE: return "LED_RGB_SEQUENCE";
    case ZP_MFG_TEST_CONNECTED_CURRENT: return "CONNECTED_CURRENT";
    case ZP_MFG_TEST_SHIPPING_CURRENT: return "SHIPPING_CURRENT";
    case ZP_MFG_TEST_BATTERY_CAL: return "BATTERY_CAL";
    case ZP_MFG_TEST_BATTERY_VERIFY: return "BATTERY_VERIFY";
    case FACTORY_P2_TEST_CHARGING_CURRENT: return "CHARGING_CURRENT";
    case FACTORY_P2_TEST_UNCONNECTED_CURRENT: return "UNCONNECTED_CURRENT";
    case FACTORY_P2_TEST_RAM: return "RAM";
    case FACTORY_P2_TEST_MAG_RAW: return "MAG_RAW";
    case FACTORY_P2_TEST_IMU_ACCEL_RAW: return "IMU_ACCEL_RAW";
    case FACTORY_P2_TEST_IMU_GYRO_RAW: return "IMU_GYRO_RAW";
    default: return "UNKNOWN";
    }
}

static void factory_boot_gate_log_masks(const zp_mfg_record_t *record,
                                        zp_mfg_semantic_status_t status)
{
    const bool final_ship =
        (record->reserved_flags & ZP_MFG_RECORD_RESERVED_FINAL_SHIP_HOST) != 0U;
    const uint32_t expected = final_ship ? ZP_MFG_FINAL_SHIP_REQUIRED_TEST_MASK :
                                          ZP_MFG_REQUIRED_TEST_MASK;
    const uint32_t missing_required = expected & ~record->required_test_mask;
    const uint32_t missing_passed = expected & ~record->passed_test_mask;
    const uint32_t extra_required = record->required_test_mask & ~expected;
    uint32_t bit;

    DBG_DIRECT("[MFG_DIAG] semantic=%s policy=%s expected=0x%08lX",
               zp_mfg_semantic_status_name(status),
               final_ship ? "final_ship_exact_v5" : "legacy_subset",
               (unsigned long)expected);
    DBG_DIRECT("[MFG_DIAG] missing_required=0x%08lX missing_passed=0x%08lX extra_required=0x%08lX",
               (unsigned long)missing_required, (unsigned long)missing_passed,
               (unsigned long)extra_required);
    if (final_ship && (record->version != ZP_MFG_RECORD_VERSION_V5))
    {
        DBG_DIRECT("[MFG_DIAG] version_mismatch actual=%u expected=%u",
                   record->version, ZP_MFG_RECORD_VERSION_V5);
    }
    for (bit = 1UL; bit != 0UL; bit <<= 1U)
    {
        if (((missing_required | missing_passed |
              (final_ship ? extra_required : 0UL)) & bit) != 0UL)
        {
            DBG_DIRECT("[MFG_DIAG] item=%s bit=0x%08lX missing_required=%u missing_passed=%u extra_required=%u",
                       factory_boot_gate_test_name(bit), (unsigned long)bit,
                       (missing_required & bit) != 0UL ? 1U : 0U,
                       (missing_passed & bit) != 0UL ? 1U : 0U,
                       (extra_required & bit) != 0UL ? 1U : 0U);
        }
    }
}
#endif

extern bool zy100_device_identity_apply_mfg(const char *ble_name,
                                            const char *sn);

static bool s_gate_inited = false;
static bool s_gate_record_loaded = false;
static bool s_gate_locked_record_invalid = false;
static factory_boot_gate_mode_t s_gate_mode =
    FACTORY_BOOT_GATE_MODE_FACTORY;
static zp_mfg_record_t s_gate_record;
static const char *s_gate_reason = "not_init";

#if ZY100_PRODUCTION_MFG_TEST_COMPAT_ENABLE
static zp_mfg_semantic_status_t factory_boot_gate_test_compat(
    zp_mfg_record_t *record, zp_mfg_semantic_status_t original)
{
    const uint32_t old_required = record->required_test_mask;
    const uint32_t old_passed = record->passed_test_mask;
    const bool final_ship =
        (record->reserved_flags & ZP_MFG_RECORD_RESERVED_FINAL_SHIP_HOST) != 0U;
    const uint32_t expected = final_ship ? ZP_MFG_FINAL_SHIP_REQUIRED_TEST_MASK :
                                          ZP_MFG_REQUIRED_TEST_MASK;
    const uint32_t required = final_ship ? expected : old_required | expected;
    const uint32_t passed = old_passed | expected;
    zp_mfg_semantic_status_t semantic;
    zp_mfg_store_status_t status;

    if ((original != ZP_MFG_SEMANTIC_STATUS_REQUIRED_MASK_MISSING) &&
        (original != ZP_MFG_SEMANTIC_STATUS_REQUIRED_PASS_MISSING))
    {
        return original;
    }
    /* Validate the complete candidate before any erase; restore the snapshot
     * for the storage transaction, which checks it against the selected slot. */
    record->required_test_mask = required;
    record->passed_test_mask = passed;
    semantic = zp_mfg_record_semantic_validate(record, ZP_MFG_REQUIRED_TEST_MASK);
    record->required_test_mask = old_required;
    record->passed_test_mask = old_passed;
    if (semantic != ZP_MFG_SEMANTIC_STATUS_OK)
    {
        DBG_DIRECT("[MFG_COMPAT][ERR] candidate=%s write=0 block=1",
                   zp_mfg_semantic_status_name(semantic));
        return semantic;
    }
    DBG_DIRECT("[MFG_COMPAT] test_assumed_pass reason=%s required=0x%08lX->0x%08lX passed=0x%08lX->0x%08lX",
               zp_mfg_semantic_status_name(original),
               (unsigned long)old_required, (unsigned long)required,
               (unsigned long)old_passed, (unsigned long)passed);
    status = zp_mfg_store_update_test_masks(record, required, passed);
    if (status != ZP_MFG_STORE_STATUS_OK)
    {
        DBG_DIRECT("[MFG_COMPAT][ERR] persist=%s block=1 retry=next_boot",
                   zp_mfg_store_status_name(status));
        return original;
    }
    semantic = zp_mfg_record_semantic_validate(record, ZP_MFG_REQUIRED_TEST_MASK);
    DBG_DIRECT("[MFG_COMPAT] persisted=1 readback=1 semantic=%s gen=%lu",
               zp_mfg_semantic_status_name(semantic),
               (unsigned long)record->generation);
    return semantic;
}
#endif

void factory_boot_gate_init(void)
{
    zp_mfg_store_status_t status;
    zp_mfg_record_t *record = &s_gate_record;

    if (s_gate_inited)
    {
        return;
    }

    memset(record, 0, sizeof(*record));
#if ZY100_BUILD_PRODUCTION
    DBG_DIRECT("[MFG_DIAG] boot fw_code=%lu schema=2 test_compat=%u",
               (unsigned long)VERSION_CODE,
               (unsigned int)ZY100_PRODUCTION_MFG_TEST_COMPAT_ENABLE);
    status = zp_mfg_store_load_boot_diagnostic(record);
    DBG_DIRECT("[MFG_DIAG] load=%s locked=%u",
               zp_mfg_store_status_name(status),
               zp_mfg_store_record_locked(record) ? 1U : 0U);
#else
    status = zp_mfg_store_load(record);
#endif
    if ((status == ZP_MFG_STORE_STATUS_OK) &&
        zp_mfg_store_record_locked(record))
    {
        zp_mfg_semantic_status_t semantic_status;

        s_gate_record_loaded = true;
        semantic_status = zp_mfg_record_semantic_validate(
                              record,
                              ZP_MFG_REQUIRED_TEST_MASK);
#if ZY100_BUILD_PRODUCTION
        factory_boot_gate_log_masks(record, semantic_status);
#endif
#if ZY100_PRODUCTION_MFG_TEST_COMPAT_ENABLE
        semantic_status = factory_boot_gate_test_compat(record, semantic_status);
#endif
        if (semantic_status != ZP_MFG_SEMANTIC_STATUS_OK)
        {
            s_gate_mode = FACTORY_BOOT_GATE_MODE_FACTORY;
            s_gate_locked_record_invalid = true;
            s_gate_reason = zp_mfg_semantic_status_name(semantic_status);
            DBG_DIRECT("[MFG_BOOT][ERR] locked semantic invalid status=%s",
                       s_gate_reason);
        }
#if ZY100_PRODUCTION_TEST_ST_TO_CG_ENABLE
        else if ((memcmp(record->channel_code, "ST",
                         sizeof(record->channel_code)) == 0) &&
                 ((status = zp_mfg_store_test_st_to_cg(record)) !=
                  ZP_MFG_STORE_STATUS_OK))
        {
            s_gate_mode = FACTORY_BOOT_GATE_MODE_FACTORY;
            s_gate_locked_record_invalid = true;
            s_gate_reason = "channel_test_write_failed";
            DBG_DIRECT("[MFG_CHANNEL_TEST][ERR] persist=%s block=1 retry=next_boot",
                       zp_mfg_store_status_name(status));
        }
#endif
        else if (zy100_device_identity_apply_mfg(record->ble_adv_name,
                                            record->final_sn))
        {
            s_gate_mode = FACTORY_BOOT_GATE_MODE_LOCKED_USER;
            s_gate_reason = "locked_record";
            DBG_DIRECT("[MFG_BOOT] locked user path sn=%s name=%s gen=%lu",
                       record->final_sn,
                       record->ble_adv_name,
                       (unsigned long)record->generation);
        }
        else
        {
            s_gate_mode = FACTORY_BOOT_GATE_MODE_FACTORY;
            s_gate_locked_record_invalid = true;
            s_gate_reason = "locked_identity_invalid";
            DBG_DIRECT("[MFG_BOOT][ERR] locked identity invalid");
        }
    }
    else
    {
        memset(&s_gate_record, 0, sizeof(s_gate_record));
        s_gate_record_loaded = false;
        s_gate_locked_record_invalid = false;
        s_gate_mode = FACTORY_BOOT_GATE_MODE_FACTORY;
        s_gate_reason = (status == ZP_MFG_STORE_STATUS_NOT_FOUND) ?
                        "record_missing_or_invalid" :
                        zp_mfg_store_status_name(status);
        DBG_DIRECT("[MFG_BOOT] factory path reason=%s", s_gate_reason);
    }

    s_gate_inited = true;
}

factory_boot_gate_mode_t factory_boot_gate_mode(void)
{
    if (!s_gate_inited)
    {
        factory_boot_gate_init();
    }
    return s_gate_mode;
}

bool factory_boot_gate_factory_mode_active(void)
{
#if ZY100_BUILD_PRODUCTION
    return false;
#else
    return factory_boot_gate_mode() == FACTORY_BOOT_GATE_MODE_FACTORY;
#endif
}

bool factory_boot_gate_locked_user_mode_active(void)
{
    return factory_boot_gate_mode() == FACTORY_BOOT_GATE_MODE_LOCKED_USER;
}

bool factory_boot_gate_locked_record_invalid(void)
{
    if (!s_gate_inited)
    {
        factory_boot_gate_init();
    }
    return s_gate_locked_record_invalid;
}

bool factory_boot_gate_production_blocked(void)
{
#if ZY100_BUILD_PRODUCTION
    return factory_boot_gate_mode() != FACTORY_BOOT_GATE_MODE_LOCKED_USER;
#else
    return false;
#endif
}

const zp_mfg_record_t *factory_boot_gate_record(void)
{
    if (!s_gate_inited)
    {
        factory_boot_gate_init();
    }
    return s_gate_record_loaded ? &s_gate_record : NULL;
}

const char *factory_boot_gate_reason(void)
{
    if (!s_gate_inited)
    {
        factory_boot_gate_init();
    }
    return s_gate_reason;
}
