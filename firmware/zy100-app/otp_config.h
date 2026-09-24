/**
*****************************************************************************************
*     Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file    otp_config.h
  * @brief   Update Configuration in APP
  * @date    2017.6.6
  * @version v1.0
  * *************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2017 Realtek Semiconductor Corporation</center></h2>
   * *************************************************************************************
  */

/*============================================================================*
 *               Define to prevent recursive inclusion
 *============================================================================*/
#ifndef OTP_CONFIG_H
#define OTP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rtl876x_wdg.h"
#include "flash_map.h"
/*============================================================================*
 *                        debug configuration
 *============================================================================*/
/** @brief just for OTA Demo test */
#define OTA_TEST                                   0
/** @brief just for OTA Demo test */
#define OTA_TEST_BANK_SWITCH                       0
#if (OTA_TEST_BANK_SWITCH == 0)
#define OTA_TEST_IMAGE_VERSION                     0
#endif

/** @brief just for debug */
#define SYSTEM_TRACE_ENABLE                        0


/*============================================================================*
 *                        flash configuration
 *============================================================================*/
/** @brief support for puran flash*/
#define FTL_APP_CALLBACK_ENABLE                    0
/** @brief enable BP, set lock level depend on flash layout and selected flash id */
#define FLASH_BLOCK_PROTECT_ENABLE                 0
/** @brief modify delay time for wakeup flash from power down mode to standby mode*/
#define AFTER_TOGGLE_CS_DELAY                      6


/*============================================================================*
 *                        platform configuration
 *============================================================================*/
/** @brief ZY100 uses P1_0/P1_1 for KEY/SENSOR-PWR, not SWD pinmux. */
#define SWD_PINMUX_ENABLE                          0
/** @brief default disable watch dog in rom */
#define ROM_WATCH_DOG_ENABLE                       0
/** @brief set wdg mode, default reset all */
#define ROM_WATCH_DOG_MODE                         RESET_ALL
/** @brief Watch Dog Timer Config, default 4s timeout
   * div_factor: 16Bit: 32.768k/(1+divfactor).
   * cnt_limit: 2^(cnt_limit+1) - 1 ; max 11~15 = 0xFFF.
   * wdg_mode:
   *            1: RESET_ALL_EXCEPT_AON
   *            3: RESET_ALL
**/
#define ROM_WATCH_DOG_CFG_DIV_FACTOR               31
#define ROM_WATCH_DOG_CFG_CNT_LIMIT                15

/*before wdg system reset, write reset reason to specific flash addr if enable*/
#define WRITE_REASON_TO_FLASH_BEFORE_RESET_ENABLE     0
#if (WRITE_REASON_TO_FLASH_BEFORE_RESET_ENABLE > 0)
/*write reset reason to specific flash address*/
#define REBOOT_REASON_RECORD_ADDRESS                  0x8cb000
/*max number of reboot record (2^n), one reset reason need 4 bytes, occupy flash size equal 2^(n+2)*/
#define REBOOT_REASON_RECORD_LIMIT_POWERT2            10  //reserve 4K
#endif


/*============================================================================*
 *                        upperstack configuration
 *============================================================================*/
//add more here
#define BT_STACK_CONFIG_ENABLE                  1

#if (BT_STACK_CONFIG_ENABLE == 1)
void bt_stack_config_init(void);
#endif


/*============================================================================*
*                        OTA configuration
*============================================================================*/
/* User data region used by FLASH_BKP_DATA1.
 * It may be written by firmware at runtime, and can also be updated
 * independently as IMAGE_USER_DATA through single-bank OTA/DFU.
 * Keep these values synchronized with flash_map.h USER_DATA1_ADDR/SIZE.
 */
#define SUPPORT_SINGLE_BANK_OTA_USER_DATA
#ifdef SUPPORT_SINGLE_BANK_OTA_USER_DATA
#define USER_DATA_START_ADDR                       USER_DATA1_ADDR
#define USER_DATA_MAX_SIZE                         USER_DATA1_SIZE
#endif

/*normal ota timeout settings*/
#define OTA_TIMEOUT_TOTAL                          240
#define OTA_TIMEOUT_WAIT4_CONN                     60
#define OTA_TIMEOUT_WAIT4_IMAGE_TRANS              200
#define OTA_TIMEOUT_CTITTV                         0xFF

/* First-stage debug plaintext OTA only. Not for production security policy. */
#define OTA_WITH_ENCRYPTION_DATA                   0


/*============================================================================*
 *                        app configuration
 *============================================================================*/
//add more here



#ifdef __cplusplus
}
#endif


/** @} */ /* End of group OTP_CONFIG */
#endif
