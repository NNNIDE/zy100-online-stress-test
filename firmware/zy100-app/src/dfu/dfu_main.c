/*
 *  Routines to access hardware
 *
 *  Copyright (c) 2014 Realtek Semiconductor Corp.
 *
 *  This module is a confidential and proprietary property of RealTek and
 *  possession or use of this module requires written permission of RealTek.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "gap.h"
#include "gap_adv.h"
#include "gap_bond_le.h"
#include "profile_server.h"
#include "os_sched.h"
#include "os_timer.h"
#include "otp_config.h"  //todo: when oepn otp needn't
#include "otp.h"
#include "trace.h"
#include "dfu_api.h"
#include "dfu_flash.h"
#include "dfu_service.h"
#include "dfu_main.h"
#include "dfu_task.h"
#include "dfu_watchdog.h"
#include "dfu_power_guard.h"
#include "dfu_application.h"
#include "board.h"
#include "rtl876x_lib_platform.h"
#include "system_rtl876x.h"
#include "../sample/ble_peripheral/service/svc_led_owner.h"
#include "../sample/ble_peripheral/service/svc_led_pattern.h"

#ifndef ZY100_LOG_DFU_INFO_ENABLE
#define ZY100_LOG_DFU_INFO_ENABLE 0
#endif

#if !ZY100_LOG_DFU_INFO_ENABLE
#undef DFU_PRINT_INFO0
#undef DFU_PRINT_INFO1
#undef DFU_PRINT_INFO2
#undef DFU_PRINT_INFO3
#undef DFU_PRINT_INFO4
#undef DFU_PRINT_INFO5
#undef DFU_PRINT_INFO6
#define DFU_PRINT_INFO0(...)
#define DFU_PRINT_INFO1(...)
#define DFU_PRINT_INFO2(...)
#define DFU_PRINT_INFO3(...)
#define DFU_PRINT_INFO4(...)
#define DFU_PRINT_INFO5(...)
#define DFU_PRINT_INFO6(...)
#endif

#if (SUPPORT_NORMAL_OTA == 1)
/*============================================================================*
 *                              Macros
 *============================================================================*/
#define TIMER_ID_DFU_TOTAL              1
#define TIMER_ID_DFU_WAIT4_CONN         2
#define TIMER_ID_DFU_IMAGE_TRANSFER     3
#define TIMER_ID_DFU_CTITTV             4
#define TIMER_ID_ZY100_OTA_LED_BREATHE  0xA100U

#define DFU_OTA_ADV_WINDOW_MIN_MS       90000U

#define BD_ADDR_SIZE                    6

#define ZY100_OTA_LED_BREATHE_TIMER_MS          50U
#define ZY100_OTA_LED_BREATHE_PERIOD_MS         2000U
#define ZY100_OTA_LED_BREATHE_HALF_PERIOD_MS    1000U
#define ZY100_OTA_LED_BREATHE_MIN_LEVEL         8U
#define ZY100_OTA_LED_GREEN_FLASH_ON_MS         160U
#define ZY100_OTA_LED_GREEN_FLASH_OFF_MS        160U
#define ZY100_OTA_LED_GREEN_FLASH_CYCLES        3U

#if defined(ZY100_BUILD_FACTORY) && (ZY100_BUILD_FACTORY == 1)
#define ZY100_DFU_ADV_ROLE                "factory"
#define ZY100_DFU_ADV_NAME                "ZY100_FOTA"
#define ZY100_DFU_ADV_NAME_LEN            10U
#else
#define ZY100_DFU_ADV_ROLE                "production"
#define ZY100_DFU_ADV_NAME                "ZY100_OTA"
#define ZY100_DFU_ADV_NAME_LEN            9U
#endif

/* What is the advertising interval when device is discoverable (units of 625us, 160=100ms)*/
#define DEFAULT_ADVERTISING_INTERVAL_MIN            160 /* 100ms */
#define DEFAULT_ADVERTISING_INTERVAL_MAX            176 /* 110ms */

/*============================================================================*
 *                               Types
 *============================================================================*/


/*============================================================================*
 *                               Variables
 *============================================================================*/
void *total_timer_handle;
void *wait4_conn_timer_handle;
void *image_transfer_timer_handle;
void *ctittv_timer_handle;
uint32_t timeout_value_total;
uint32_t timeout_value_wait4_conn;
uint32_t timeout_value_image_transfer;
uint32_t timeout_value_ctittv;
T_SERVER_ID rtk_dfu_service_id;
#if !ZY100_BUILD_PRODUCTION
static void *zy100_ota_led_breathe_timer_handle;
static uint64_t zy100_ota_led_owner_now_ms;
static bool zy100_ota_led_breathing;
#endif
static bool zy100_ota_led_clock_logged;

/*============================================================================*
 *                              Local Functions
 *============================================================================*/
static bool zy100_ota_led_prepare_clock(const char *reason)
{
    const uint32_t target_hz = (uint32_t)SYSTEM_80MHZ;
    uint32_t before_hz = get_cpu_clock();
    uint32_t after_hz;
    bool set_ok = true;
    bool ready;

    if (before_hz != target_hz)
    {
        set_ok = set_system_clock(SYSTEM_80MHZ);
    }

    after_hz = get_cpu_clock();
    ready = (set_ok != false) && (after_hz == target_hz);

    if ((!zy100_ota_led_clock_logged) || (!ready))
    {
        DBG_DIRECT("[OTA_LED] clock reason=%s before=%lu target=%lu set_ok=%u after=%lu ok=%u",
                   (reason != NULL) ? reason : "unknown",
                   (unsigned long)before_hz,
                   (unsigned long)target_hz,
                   set_ok ? 1U : 0U,
                   (unsigned long)after_hz,
                   ready ? 1U : 0U);
        zy100_ota_led_clock_logged = true;
    }

    if (!ready)
    {
        DBG_DIRECT("[OTA_LED][WARN] clock_not_80m after=%lu", (unsigned long)after_hz);
    }

    return ready;
}

#if !ZY100_BUILD_PRODUCTION
static void zy100_ota_led_breathe_timer_cb(void *p_xtimer)
{
    (void)p_xtimer;

    if (!zy100_ota_led_breathing)
    {
        return;
    }

    zy100_ota_led_owner_now_ms += (uint64_t)ZY100_OTA_LED_BREATHE_TIMER_MS;
    led_tick(zy100_ota_led_owner_now_ms);
}
#endif

void zy100_ota_led_start_blue_breathing(void)
{
#if ZY100_BUILD_PRODUCTION
    led_pattern_t pattern;
    if (!zy100_ota_led_prepare_clock("blue_start") || !svc_led_pattern_init()) return;
    led_pattern_notify_blue_breath(&pattern, true);
    if (!led_request(LED_OWNER_OTA_UPLOAD_RESULT, LED_PRIORITY_OTA_UPLOAD_RESULT, &pattern))
        DBG_DIRECT("[OTA_LED][WARN] blue_breath_request_failed");
#else

    bool timer_ok = true;
    bool init_ok;
    bool initial_ok;
    led_pattern_t pattern;

    (void)zy100_ota_led_prepare_clock("blue_start");

    if (zy100_ota_led_breathe_timer_handle == NULL)
    {
        timer_ok = os_timer_create(&zy100_ota_led_breathe_timer_handle, "otaLedBreath",
                                   TIMER_ID_ZY100_OTA_LED_BREATHE,
                                   ZY100_OTA_LED_BREATHE_TIMER_MS, true,
                                   zy100_ota_led_breathe_timer_cb);
        if ((!timer_ok) || (zy100_ota_led_breathe_timer_handle == NULL))
        {
            DBG_DIRECT("[OTA_LED][WARN] timer_create_failed ok=%u handle=%u",
                       timer_ok ? 1U : 0U,
                       (zy100_ota_led_breathe_timer_handle != NULL) ? 1U : 0U);
        }
    }

    init_ok = svc_led_pattern_init();
    if (!init_ok)
    {
        zy100_ota_led_breathing = false;
        DBG_DIRECT("[OTA_LED][WARN] init_failed");
        return;
    }

    zy100_ota_led_owner_now_ms = 1ULL;
    led_pattern_notify_blue_breath(&pattern, true);
    initial_ok = led_request(LED_OWNER_OTA_UPLOAD_RESULT,
                             LED_PRIORITY_OTA_UPLOAD_RESULT,
                             &pattern);
    if (!initial_ok)
    {
        zy100_ota_led_breathing = false;
        DBG_DIRECT("[OTA_LED][WARN] initial_blue_failed");
        return;
    }

    if (zy100_ota_led_breathe_timer_handle != NULL)
    {
        timer_ok = os_timer_start(&zy100_ota_led_breathe_timer_handle);
        if (!timer_ok)
        {
            zy100_ota_led_breathing = false;
            led_release(LED_OWNER_OTA_UPLOAD_RESULT);
            DBG_DIRECT("[OTA_LED][WARN] timer_start_failed");
            return;
        }
        zy100_ota_led_breathing = true;
        DBG_DIRECT("[OTA_LED] blue_breathing_started init=%u initial=%u timer=%u",
                   init_ok ? 1U : 0U,
                   initial_ok ? 1U : 0U,
                   timer_ok ? 1U : 0U);
    }
    else
    {
        zy100_ota_led_breathing = false;
        DBG_DIRECT("[OTA_LED][WARN] timer_unavailable_static_blue");
    }
#endif
}

void zy100_ota_led_stop(void)
{
#if ZY100_BUILD_PRODUCTION
    led_release(LED_OWNER_OTA_UPLOAD_RESULT);
#else

    zy100_ota_led_breathing = false;
    if (zy100_ota_led_breathe_timer_handle != NULL)
    {
        if (!os_timer_stop(&zy100_ota_led_breathe_timer_handle))
        {
            DBG_DIRECT("[OTA_LED][WARN] timer_stop_failed");
        }
    }
    led_release(LED_OWNER_OTA_UPLOAD_RESULT);
#endif
}

void zy100_ota_led_show_success_green_flash_blocking(void)
{
    uint8_t cycle;
    led_pattern_t pattern;
    bool request_ok;

    (void)zy100_ota_led_prepare_clock("success_flash");
    zy100_ota_led_stop();
    led_pattern_notify_blink(&pattern,
                             LED_PATTERN_COLOR_GREEN,
                             ZY100_OTA_LED_GREEN_FLASH_ON_MS,
                             ZY100_OTA_LED_GREEN_FLASH_OFF_MS,
                             ZY100_OTA_LED_GREEN_FLASH_CYCLES,
                             true);
#if !ZY100_BUILD_PRODUCTION
    if (zy100_ota_led_owner_now_ms == 0ULL)
    {
        zy100_ota_led_owner_now_ms = 1ULL;
    }
#endif
    request_ok = led_request(LED_OWNER_OTA_UPLOAD_RESULT,
                             LED_PRIORITY_OTA_UPLOAD_RESULT,
                             &pattern);
    if (!request_ok)
    {
        DBG_DIRECT("[OTA_LED][WARN] success_green_request_failed");
        return;
    }

    for (cycle = 0U; cycle < ZY100_OTA_LED_GREEN_FLASH_CYCLES; cycle++)
    {
        os_delay(ZY100_OTA_LED_GREEN_FLASH_ON_MS);
#if ZY100_BUILD_PRODUCTION
        led_tick(os_sys_time_get());
#else
        zy100_ota_led_owner_now_ms += (uint64_t)ZY100_OTA_LED_GREEN_FLASH_ON_MS;
        led_tick(zy100_ota_led_owner_now_ms);
#endif
        os_delay(ZY100_OTA_LED_GREEN_FLASH_OFF_MS);
#if ZY100_BUILD_PRODUCTION
        led_tick(os_sys_time_get());
#else
        zy100_ota_led_owner_now_ms += (uint64_t)ZY100_OTA_LED_GREEN_FLASH_OFF_MS;
        led_tick(zy100_ota_led_owner_now_ms);
#endif
    }

    led_release(LED_OWNER_OTA_UPLOAD_RESULT);
}

/* Preserve the GAP name's reverse-address order and lowercase hexadecimal. */
static void dfu_format_device_name(char *device_name, const uint8_t *bd_addr)
{
    static const char hex_digits[] = "0123456789abcdef";
    uint8_t i;

    for (i = 0U; i < BD_ADDR_SIZE; i++)
    {
        uint8_t value = bd_addr[BD_ADDR_SIZE - 1U - i];
        device_name[2U * i] = hex_digits[value >> 4U];
        device_name[2U * i + 1U] = hex_digits[value & 0x0FU];
    }
    device_name[BD_ADDR_SIZE * 2U] = '\0';
}

/*
 * @fn          Initial gap parameters
 * @brief      Initialize peripheral and gap bond manager related parameters
 *
 * @return     void
 */
void dfu_le_gap_init(void)
{

    uint8_t bt_bd_addr[6];
    gap_get_param(GAP_PARAM_BD_ADDR, bt_bd_addr); //get OTP->bt_bd_addr

    //device name and device appearance
    char device_name[GAP_DEVICE_NAME_LEN];
    dfu_format_device_name(device_name, bt_bd_addr);
    uint16_t appearance = GAP_GATT_APPEARANCE_KEYBOARD;
    uint8_t  slave_init_mtu_req = true;

    //advertising parameters
    uint8_t  adv_evt_type = GAP_ADTYPE_ADV_IND;
    uint8_t  adv_direct_type = GAP_REMOTE_ADDR_LE_PUBLIC;
    uint8_t  adv_direct_addr[GAP_BD_ADDR_LEN] = {0};
    uint8_t  adv_chann_map = GAP_ADVCHAN_ALL;
    uint8_t  adv_filter_policy = GAP_ADV_FILTER_ANY;
    uint16_t adv_int_min = DEFAULT_ADVERTISING_INTERVAL_MIN;
    uint16_t adv_int_max = DEFAULT_ADVERTISING_INTERVAL_MIN;
    uint8_t local_bd_type = GAP_LOCAL_ADDR_LE_RANDOM;
    //advertising data
    uint8_t adv_data_uuid128[31] =
    {
        /* Flags */
        0x02,                           /* length     */
        GAP_ADTYPE_FLAGS,               /* type="flags" */
        GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

        /* Service */
        0x11,                           /* length     */
        GAP_ADTYPE_128BIT_COMPLETE,     /* type="Complete list of 128-bit UUIDs" */
        GATT_UUID128_DFU_SERVICE,

        9,
        0xFF,
        0x5D,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
    };
    adv_data_uuid128[25] = bt_bd_addr[5];
    adv_data_uuid128[26] = bt_bd_addr[4];
    adv_data_uuid128[27] = bt_bd_addr[3];
    adv_data_uuid128[28] = bt_bd_addr[2];
    adv_data_uuid128[29] = bt_bd_addr[1];
    adv_data_uuid128[30] = bt_bd_addr[0];

    //scan response data
    uint8_t scan_rsp_data[ZY100_DFU_ADV_NAME_LEN + 2U] = {0U};
    scan_rsp_data[0] = (uint8_t)(ZY100_DFU_ADV_NAME_LEN + 1U);
    scan_rsp_data[1] = GAP_ADTYPE_LOCAL_NAME_COMPLETE;
    memcpy(&scan_rsp_data[2], ZY100_DFU_ADV_NAME, ZY100_DFU_ADV_NAME_LEN);

    DBG_DIRECT("[DFU_ADV] role=%s name=%s otp=%02x%02x%02x%02x%02x%02x",
               ZY100_DFU_ADV_ROLE,
               ZY100_DFU_ADV_NAME,
               bt_bd_addr[5], bt_bd_addr[4], bt_bd_addr[3],
               bt_bd_addr[2], bt_bd_addr[1], bt_bd_addr[0]);

    //GAP Bond Manager parameters
    uint8_t  gap_param_cccd_storage = false;
    uint8_t  auth_pair_mode = GAP_PAIRING_MODE_PAIRABLE;
    uint16_t auth_flags = GAP_AUTHEN_BIT_BONDING_FLAG | GAP_AUTHEN_BIT_MITM_FLAG;
    uint8_t  auth_io_cap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
#ifndef SUPPORT_ALONE_UPPERSTACK_IMG
    uint8_t  auth_oob = false;
#endif
    uint8_t  auth_use_fix_passkey = false;
    uint32_t auth_fix_passkey = 0;
    uint8_t  auth_sec_req_enalbe = false;
    uint16_t auth_sec_req_flags = GAP_AUTHEN_BIT_NONE;

    //Register gap callback
    le_register_app_cb(dfu_gap_callback);

    //Set device name and device appearance
    le_set_gap_param(GAP_PARAM_DEVICE_NAME, GAP_DEVICE_NAME_LEN, device_name);
    le_set_gap_param(GAP_PARAM_APPEARANCE, sizeof(appearance), &appearance);
    le_set_gap_param(GAP_PARAM_SLAVE_INIT_GATT_MTU_REQ, sizeof(slave_init_mtu_req),
                     &slave_init_mtu_req);

    //Set advertising parameters
    le_adv_set_param(GAP_PARAM_ADV_EVENT_TYPE, sizeof(adv_evt_type), &adv_evt_type);
    le_adv_set_param(GAP_PARAM_ADV_DIRECT_ADDR_TYPE, sizeof(adv_direct_type), &adv_direct_type);
    le_adv_set_param(GAP_PARAM_ADV_DIRECT_ADDR, sizeof(adv_direct_addr), adv_direct_addr);
    le_adv_set_param(GAP_PARAM_ADV_CHANNEL_MAP, sizeof(adv_chann_map), &adv_chann_map);
    le_adv_set_param(GAP_PARAM_ADV_FILTER_POLICY, sizeof(adv_filter_policy), &adv_filter_policy);
    le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MIN, sizeof(adv_int_min), &adv_int_min);
    le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MAX, sizeof(adv_int_max), &adv_int_max);
    le_adv_set_param(GAP_PARAM_ADV_LOCAL_ADDR_TYPE, sizeof(local_bd_type), &local_bd_type);
    le_adv_set_param(GAP_PARAM_ADV_DATA, sizeof(adv_data_uuid128), adv_data_uuid128);
    le_adv_set_param(GAP_PARAM_SCAN_RSP_DATA, sizeof(scan_rsp_data), scan_rsp_data);

    // Setup the GAP Bond Manager
    le_bond_set_param(GAP_PARAM_BOND_CCCD_STORAGE, sizeof(gap_param_cccd_storage),
                      &gap_param_cccd_storage);
    gap_set_param(GAP_PARAM_BOND_PAIRING_MODE, sizeof(auth_pair_mode), &auth_pair_mode);
    gap_set_param(GAP_PARAM_BOND_AUTHEN_REQUIREMENTS_FLAGS, sizeof(auth_flags), &auth_flags);
    gap_set_param(GAP_PARAM_BOND_IO_CAPABILITIES, sizeof(auth_io_cap), &auth_io_cap);
#ifndef SUPPORT_ALONE_UPPERSTACK_IMG
    gap_set_param(GAP_PARAM_BOND_OOB_ENABLED, sizeof(auth_oob), &auth_oob);
#endif
    le_bond_set_param(GAP_PARAM_BOND_FIXED_PASSKEY, sizeof(auth_fix_passkey), &auth_fix_passkey);
    le_bond_set_param(GAP_PARAM_BOND_FIXED_PASSKEY_ENABLE, sizeof(auth_use_fix_passkey),
                      &auth_use_fix_passkey);
    le_bond_set_param(GAP_PARAM_BOND_SEC_REQ_ENABLE, sizeof(auth_sec_req_enalbe), &auth_sec_req_enalbe);
    le_bond_set_param(GAP_PARAM_BOND_SEC_REQ_REQUIREMENT, sizeof(auth_sec_req_flags),
                      &auth_sec_req_flags);

}

/******************************************************************
 * @fn          Initial profile
 * @brief      Add simple profile service and register callbacks
 *
 * @return     void
 */
void dfu_le_profile_init(void)
{
    server_init(1);
    rtk_dfu_service_id = dfu_add_service(dfu_profile_callback);
    server_register_app_cb(dfu_profile_callback);
}

void dfu_monitor_timeout_handler(void *p_xtimer)
{
    uint32_t timer_id = 0;

    os_timer_id_get(&p_xtimer, &timer_id);

    APP_PRINT_ERROR1("dfu_monitor_timeout_handler, TimerID(%u)", timer_id);

    switch (timer_id)
    {
    case TIMER_ID_DFU_TOTAL:
    case TIMER_ID_DFU_WAIT4_CONN:
    case TIMER_ID_DFU_IMAGE_TRANSFER:
    case TIMER_ID_DFU_CTITTV:
        dfu_fw_reboot(false);
        break;
    }
}

void dfu_timer_init(void)
{
    timeout_value_total = OTP->ota_timeout_total * 1000;
    timeout_value_wait4_conn = OTP->ota_timeout_wait4_conn * 1000;
    timeout_value_image_transfer = OTP->ota_timeout_wait4_image_transfer * 1000;
    timeout_value_ctittv = OTP->ota_timeout_ctittv * 1000;

    if (timeout_value_total < DFU_OTA_ADV_WINDOW_MIN_MS)
    {
        timeout_value_total = DFU_OTA_ADV_WINDOW_MIN_MS;
    }
    if (timeout_value_wait4_conn < DFU_OTA_ADV_WINDOW_MIN_MS)
    {
        timeout_value_wait4_conn = DFU_OTA_ADV_WINDOW_MIN_MS;
    }

    os_timer_create(&total_timer_handle, "dfuTotalTimer", TIMER_ID_DFU_TOTAL,
                    timeout_value_total, false, dfu_monitor_timeout_handler);

    os_timer_create(&wait4_conn_timer_handle, "dfuWait4ConTimer", TIMER_ID_DFU_WAIT4_CONN,
                    timeout_value_wait4_conn, false, dfu_monitor_timeout_handler);

    os_timer_create(&image_transfer_timer_handle, "dfuImageTransferTimer",
                    TIMER_ID_DFU_IMAGE_TRANSFER, timeout_value_image_transfer,
                    false, dfu_monitor_timeout_handler);

    os_timer_create(&ctittv_timer_handle, "dfuCtittvTimer", TIMER_ID_DFU_CTITTV,
                    timeout_value_ctittv, false, dfu_monitor_timeout_handler);

    os_timer_start(&total_timer_handle);

    os_timer_start(&wait4_conn_timer_handle);
}

void dfu_init(void)
{
    WDG_Disable();
    if (!dfu_watchdog_init())
    {
        DBG_DIRECT("[DFU_WDG][FAULT] init failed, reset to previous APP");
        dfu_fw_reboot(false);
    }

    if (unlock_flash_bp_all())
    {
        DFU_PRINT_INFO0("[==>dfu_init: Flash unlock BP all success!");
    }
    else
    {
        DBG_DIRECT("dfu init unlock BP fail!");
    }
}
/*============================================================================*
 *                              Local Functions
 *============================================================================*/
void dfu_set_rand_addr(void)
{
    T_GAP_RAND_ADDR_TYPE rand_addr_type = GAP_RAND_ADDR_NON_RESOLVABLE;
    uint8_t random_bd[BD_ADDR_SIZE] = {0};
    le_gen_rand_addr(rand_addr_type, random_bd);
    DFU_PRINT_INFO1("dfu_set_rand_addr: rand_addr %b", TRACE_BDADDR(random_bd));
    le_set_rand_addr(random_bd);
}

void dfu_main(void)
{
    T_DFU_POWER_GUARD_SNAPSHOT power_guard_snapshot;
    T_DFU_POWER_GUARD_RESULT power_guard_result;

    power_guard_result = dfu_power_guard_init(&power_guard_snapshot);
    DBG_DIRECT("[DFU_POWER_GUARD] result=%s external=%u stacmd=%u risk=%s",
               dfu_power_guard_result_name(power_guard_result),
               power_guard_snapshot.external_power_present ? 1U : 0U,
               power_guard_snapshot.stacmd_level,
               (power_guard_result == DFU_POWER_GUARD_STACMD_LOW_ON_BATTERY) ?
               "STACMD_LONG_LOW" : "none");
    if (power_guard_result != DFU_POWER_GUARD_OK)
    {
        DBG_DIRECT("[DFU_POWER_GUARD][FAULT] DFU blocked; clear OTA flag and return to APP");
        dfu_set_ota_mode_flag(false);
        dfu_fw_reboot(false);
        return;
    }

    DBG_DIRECT("Enter DFU mode");
    DBG_DIRECT("OTP->ota_with_encryption_data=%d", OTP->ota_with_encryption_data);
    (void)zy100_ota_led_prepare_clock("dfu_main");
    le_gap_init(1);
    gap_lib_init();
    dfu_le_gap_init();
    dfu_le_profile_init();
    dfu_init();
    dfu_task_init();
}

#endif //end SUPPORT_NORMAL_OTA
