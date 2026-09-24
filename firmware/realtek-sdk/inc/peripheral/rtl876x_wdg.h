/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      rtl876x_wdg.h
* @brief     header file of watch dog driver.
* @details
* @author    Lory_xu
* @date      2016-06-12
* @version   v0.1
* *********************************************************************************************************
*/

#ifndef _RTL876X_WDG_H_
#define _RTL876X_WDG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "rtl876x.h"
#include "rtl876x_bitfields.h"

/**
 * \addtogroup  IO          Peripheral Drivers
 * \defgroup    WATCH_DOG   WATCH_DOG
 *
 * \brief       Watch Dog driver module.
 *
 * \ingroup     IO
 */

/** @defgroup WATCH_DOG_Exported_Types Watch Dog Exported Types
 * \{
 * \ingroup    WATCH_DOG
 */

typedef enum
{
    INTERRUPT_CPU = 0,
    RESET_ALL_EXCEPT_AON = 1,
    RESET_CORE_DOMAIN = 2,
    RESET_ALL = 3
} T_WDG_MODE;

/**
 * wdg reset reason introduction:
 * 1.If you want to get reset reason from aon 0x15, deviding three types:
 *   a) HW reset: aon reg 0x15 is cleared to 0, magic pattern on ram will change
 *   b) SW RESET_ALL: aon reg 0x15 is cleared to 0,but magic pattern on ram not change
 *   c) SW RESET_ALL_EXCEPT_AON: obtain reset reason by reading aon reg 0x15 .
 * 2. Attention: don't use 0x1 as your reset reason when using RESET_ALL_EXCEPT_AON type! Because 0x1 is default value.
 */
typedef enum
{
    RESET_REASON_HW                 = 0x0,  /* HW reset */
    RESET_REASON_WDG_TIMEOUT        = 0x1,
    RESET_REASON_DSS_WAKEUP         = 0xCF, /* DSS Wake up */

    SW_RESET_APP_START              = 0xD0,
    SWITCH_HCI_MODE                 = 0xD1,
    SWITCH_TEST_MODE                = 0xD2,
    DFU_SWITCH_TO_OTA_MODE          = 0xD3,
    DFU_ACTIVE_RESET                = 0xD4,
    DFU_FAIL_RESET                  = 0xD5,
    UPPER_CMD_RESET,
    SINGLE_TONE_TIMEOUT_RESET,
    UART_CMD_RESET,
    RESET_REASON_FACTORY_RESET,
    RESET_REASON_LPC_TRIGGER,
    SW_RESET_APP_END                = 0xFF,
} T_SW_RESET_REASON;

typedef void (*APP_CB_WDG_RESET_TYPE)(T_WDG_MODE wdg_mode, T_SW_RESET_REASON reset_reason);
typedef bool (*BOOL_WDG_CB)(T_WDG_MODE wdg_mode, T_SW_RESET_REASON reset_reason);

/**
  * @}
  */

/** @defgroup WATCH_DOG_Exported_Variables Watch Dog Exported Variables
 * \{
 * \ingroup    WATCH_DOG
 */

extern APP_CB_WDG_RESET_TYPE app_cb_wdg_reset;
extern BOOL_WDG_CB user_wdg_cb;

/**
  * @}
  */

/** @defgroup WATCH_DOG_Exported_Functions Watch Dog Exported Functions
 * \{
 * \ingroup    WATCH_DOG
 */

/**
   * @brief  Watch Dog Clock Enable.
   */
extern void WDG_ClockEnable(void);

/**
   * @brief  Watch Dog Timer Config.
   * @param  div_factor: 16Bit: 32.768k/(1+divfactor).
   * @param  cnt_limit: 2^(cnt_limit+1) - 1 ; max 11~15 = 0xFFF.
   * @param  wdg_mode: 0: interrupt CPU
   *                   1: reset all except aon
   *                   2: reset core domain
   *                   3: reset all
   * @retval none.
   */
extern void WDG_Config(uint16_t div_factor, uint8_t cnt_limit, T_WDG_MODE wdg_mode);

/**
   * @brief  Watch Dog Timer Enable.
   */
extern void WDG_Enable(void);

/**
   * @brief  Watch Dog Timer Disable.
   */
extern void WDG_Disable(void);

/**
   * @brief  Watch Dog Timer Restart.
   */
extern void WDG_Restart(void);

/**
   * @brief  Watch Dog System Reset.
   * @param  wdg_mode: 0: interrupt CPU (Generate wdg interrupt without reset, for debug only)
   *                   1: reset all except AON (Retain AON registers from offset 0x2 to 0x17, which is recommended for reset but keeps some flags/data, such as switching to HCI mode)
   *                   2: reset core domain (Reset is not complete as it keeps all AON registers, which is not recommended)
   *                   3: reset all (Clear all AON registers, which is recommended for a complete reset)
   */
extern void WDG_SystemReset(T_WDG_MODE wdg_mode, T_SW_RESET_REASON reset_reason);

/**
   * @brief  Get reset reason.
   * @param  none.
   * @retval Reset reason. Note: The reset reason is unreliable!!!
     *         There are two typical watch dog reset type: "reset all except aon" (Keep AON register) and "reset all" (Clear AON register),
     *             so the AON register is not suitable as a recording medium for the general reset reason.
     *         This API will prioritize reading data from SRAM to determine the reset reason.
     *         However, there are limitations about SRAM retention during reset:
     *            SRAM will not be cleared automatically during a watch dog reset, but there will be a momentary power outage,
     *            which the SRAM data can be retained by most ICs but not all.
     *         Therefore, this API can only be used for debugging and cannot guarantee 100% reliable reset reason.
     *         If a reliable reset reason is required, using FTL for recording is recommended.
   */
extern T_SW_RESET_REASON  reset_reason_get(void);

/** \} */ /* End of group WATCH_DOG_Exported_Functions */

#ifdef __cplusplus
}
#endif

#endif //_RTL876X_WDG_H_
