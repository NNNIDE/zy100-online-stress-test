/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      board.h
* @brief     header file of Keypad demo.
* @details
* @author    tifnan_ge
* @date      2015-06-26
* @version   v0.1
* *********************************************************************************************************
*/


#ifndef _BOARD_H_
#define _BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "rtl876x_pinmux.h"

/** @defgroup IO Driver Config
  * @note user must config it firstly!! Do not change macro names!!
  * @{
  */

/* if use user define dlps enter/dlps exit callback function */
#define USE_USER_DEFINE_DLPS_EXIT_CB      1
#define USE_USER_DEFINE_DLPS_ENTER_CB     1

/* if use any peripherals below, #define it 1 */
#ifndef ZY100_V0_BUTTON_DLPS_BASELINE_TEST_ENABLE
#define ZY100_V0_BUTTON_DLPS_BASELINE_TEST_ENABLE 0U
#endif
#ifndef ZY100_V0_BUTTON_DLPS_BASELINE_USE_RTC_DLPS_ENABLE
#define ZY100_V0_BUTTON_DLPS_BASELINE_USE_RTC_DLPS_ENABLE 0U
#endif
#define USE_I2C0_DLPS        0
#define USE_I2C1_DLPS        0
#define USE_TIM_DLPS         0
#define USE_QDECODER_DLPS    0
#define USE_IR_DLPS          0
#if ZY100_V0_BUTTON_DLPS_BASELINE_TEST_ENABLE
#define USE_RTC_DLPS         ZY100_V0_BUTTON_DLPS_BASELINE_USE_RTC_DLPS_ENABLE
#else
#define USE_RTC_DLPS         1
#endif
#define USE_UART_DLPS        0
/*
 * Production targeted diagnostics use the ROM LOG UART on P0_3.  Register
 * the SDK's dedicated LOG UART2 save/restore hooks whenever those diagnostics
 * are enabled; otherwise the first button-only DLPS cycle can leave runtime
 * logging dependent on undocumented peripheral retention.
 */
#ifndef USE_LOG_UART2_DLPS
#define USE_LOG_UART2_DLPS   ZY100_BLE_TARGETED_LOG_ENABLE
#endif
#define USE_ADC_DLPS         0
#define USE_SPI0_DLPS        0
#define USE_SPI1_DLPS        0
#define USE_SPI2W_DLPS       0
#define USE_KEYSCAN_DLPS     0
#define USE_DMIC_DLPS        0
#define USE_GPIO_DLPS        0
#define USE_PWM0_DLPS        0
#define USE_PWM1_DLPS        0
#define USE_PWM2_DLPS        0
#define USE_PWM3_DLPS        0

/* OTA Parameter*/
#ifndef F_APP_DFU_ENTRY_ENABLE
#define F_APP_DFU_ENTRY_ENABLE        1
#endif
#ifndef SUPPORT_NORMAL_OTA
#define SUPPORT_NORMAL_OTA            F_APP_DFU_ENTRY_ENABLE
#endif
#define DFU_TEMP_BUFFER_SIZE          2048
#define DFU_BUFFER_CHECK_ENABLE       0x1

/* do not modify USE_IO_DRIVER_DLPS macro */
#define USE_IO_DRIVER_DLPS   (USE_I2C0_DLPS | USE_I2C1_DLPS | USE_TIM_DLPS | USE_QDECODER_DLPS\
                              | USE_IR_DLPS | USE_RTC_DLPS | USE_UART_DLPS | USE_SPI0_DLPS\
                              | USE_SPI1_DLPS | USE_SPI2W_DLPS | USE_KEYSCAN_DLPS | USE_DMIC_DLPS\
                              | USE_GPIO_DLPS | USE_USER_DEFINE_DLPS_EXIT_CB\
                              | USE_RTC_DLPS | USE_PWM0_DLPS | USE_PWM1_DLPS | USE_PWM2_DLPS\
                              | USE_PWM3_DLPS | USE_USER_DEFINE_DLPS_ENTER_CB)
#define DLPS_EN               1


#ifdef __cplusplus
}
#endif

#endif  /* _BOARD_H_ */
