/**
*****************************************************************************************
*     Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      main.c
   * @brief     Source file for BLE peripheral project, mainly used for initialize modules
   * @author    jane
   * @date      2017-06-12
   * @version   v1.0
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2017 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */

/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include <os_sched.h>
#include <os_sync.h>
#include <string.h>
#include <stdlib.h>
#include <rtl876x.h>
#include <rtl876x_gpio.h>
#include <rtl876x_pinmux.h>
#include <rtl876x_nvic.h>
#include <rtl876x_rcc.h>
#include <rtl876x_spi.h>
#include <trace.h>
#include "flash_adv_cfg.h"
#include <gap.h>
#include <gap_adv.h>
#include <gap_bond_le.h>
#include <gap_config.h>
#include <profile_server.h>
#include <gap_storage_le.h>
#include <gap_msg.h>
#include <simple_ble_service.h>
#include <bas.h>
#include <app_task.h>
#include <peripheral_app.h>
#include "app_flags.h"
#include "lp_dlps_helper.h"
#include "common/zy100_byteorder.h"
#if ZY100_PRODUCT_LOG_QUIET_ENABLE
#undef DBG_DIRECT
#define DBG_DIRECT(...) ZY100_LOG_VERBOSE(__VA_ARGS__)
#endif
#include "zy100_clock_config.h"
#if F_APP_BUTTON_LOG_ENABLE && F_BT_DLPS_EN && F_APP_BUTTON_DLPS_CTRL_ENABLE && \
    !F_LED_EDGE_HW_TEST_ENABLE
#include "FreeRTOS_API.h"
#endif
#if F_LED_EDGE_HW_TEST_ENABLE
#include "FreeRTOS_API.h"
#include "platform_utils.h"
#include "bsp/bsp_led_power.h"
#include "driver/drv_rgb_led.h"
#endif
#if F_RGB_MARQUEE_TEST_ENABLE
#include "service/zy100_rgb_marquee_test.h"
#endif
#if F_APP_BATTERY_ADC_ENABLE
#include "service/battery_adc.h"
#include "service/battery_adc_guard.h"
#endif
#include "service/zy100_rtc_clock.h"
#include "service/zy100_device_identity.h"
#include "service/zy100_mfg_info_service.h"
#include "service/zy100_production_acceptance.h"
#include "service/zy100_production_shipping.h"
#include "service/zy100_whole_unit_test.h"
#include "service/zy100_feature_config.h"
#if ZY100_BUILD_FACTORY
#include "app_factory/factory_ble_service.h"
#endif
#include "app_factory/factory_boot_gate.h"
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
#include "service/zy100_ble_ctrl_service.h"
#include "service/zy100_calibration_ble_service.h"
#include "service/zy100_calibration_manager.h"
#endif
#if F_BLE_OTA_SERVICE_ENABLE
#include <ota_service.h>
#endif
#if F_BT_ANCS_CLIENT_SUPPORT
#include <profile_client.h>
#include <ancs.h>
#endif
#if F_BT_DLPS_EN
#include <dlps.h>
#include <rtl876x_io_dlps.h>
#include "bsp/imu_board_pinmap.h"
#if F_APP_BUTTON_DLPS_CTRL_ENABLE
#include "bsp/imu_bsp.h"
#if ZY100_CHARGE_LED_ENABLE
#include "bsp/bsp_power_status.h"
#endif
#endif
#endif


/** @defgroup  PERIPH_DEMO_MAIN Peripheral Main
    * @brief Main file to initialize hardware and BT stack and start task scheduling
    * @{
    */

/*============================================================================*
 *                              Constants
 *============================================================================*/
/** @brief  Default minimum advertising interval when device is discoverable (units of 625us, 160=100ms) */
#define DEFAULT_ADVERTISING_INTERVAL_MIN            320
/** @brief  Default maximum advertising interval */
#define DEFAULT_ADVERTISING_INTERVAL_MAX            320
/** @brief  Unified BLE local name for GAP device name and advertising payload */
#define APP_BLE_LOCAL_NAME_LEN                      9U
#define APP_ADV_LOCAL_NAME_OFFSET                   9U
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
#define APP_GATT_CTRL_SERVICE_NUM                   1U
#define APP_GATT_CAL_SERVICE_NUM                    1U
#else
#define APP_GATT_CTRL_SERVICE_NUM                   0U
#define APP_GATT_CAL_SERVICE_NUM                    0U
#endif
#if F_BLE_OTA_SERVICE_ENABLE
#define APP_GATT_OTA_SERVICE_NUM                    1U
#else
#define APP_GATT_OTA_SERVICE_NUM                    0U
#endif
#define APP_GATT_USER_BASE_SERVICE_NUM              2U
#define APP_GATT_FACTORY_SERVICE_NUM                1U
#define APP_GATT_FACTORY_BAS_SERVICE_NUM            1U
#define APP_GATT_LOCKED_INFO_SERVICE_NUM            1U
#define APP_GATT_USER_SERVICE_NUM                   (APP_GATT_USER_BASE_SERVICE_NUM + \
                                                     APP_GATT_CTRL_SERVICE_NUM + \
                                                     APP_GATT_CAL_SERVICE_NUM + \
                                                     APP_GATT_OTA_SERVICE_NUM + \
                                                     APP_GATT_LOCKED_INFO_SERVICE_NUM)
#define APP_GATT_FACTORY_MODE_SERVICE_NUM           (APP_GATT_FACTORY_BAS_SERVICE_NUM + \
                                                     APP_GATT_FACTORY_SERVICE_NUM)
#define APP_GATT_DIAGNOSTIC_SERVICE_NUM             2U
#define APP_GATT_PRODUCTION_ACCEPTANCE_SERVICE_NUM  3U
#define APP_GATT_WHOLE_UNIT_SERVICE_NUM             3U
#define APP_GD25Q32E_UNIQUE_ID_LEN                  16U

#if ((3U + 4U + 2U + APP_BLE_LOCAL_NAME_LEN) > 31U)
#error "BLE advertising payload exceeds 31 bytes"
#endif

#if F_APP_BUTTON_LOG_ENABLE
#define APP_BUTTON_PIN                              P1_0
#define APP_BUTTON_DEBOUNCE_MS                      10U
#define APP_BUTTON_GPIO_IRQN                        GPIO8_IRQn
#define APP_BUTTON_GPIO_ISR                         GPIO8_Handler
#define APP_BUTTON_EDGE_FIFO_MAX_EDGES               8U
#define APP_BUTTON_EDGE_FIFO_RING_SIZE               (APP_BUTTON_EDGE_FIFO_MAX_EDGES + 1U)
#endif

#if F_LED_EDGE_HW_TEST_ENABLE
#define LED_EDGE_HW_TEST_TASK_STACK_WORDS           512U
#define LED_EDGE_HW_TEST_TASK_PRIORITY              (tskIDLE_PRIORITY + 2U)
#define LED_EDGE_HW_TEST_POLL_MS                    5U
#define LED_EDGE_HW_TEST_POWER_SETTLE_US            1000U
#define LED_EDGE_HW_TEST_EDGE_COUNT                 8U
#endif



/*============================================================================*
 *                              Variables
 *============================================================================*/

/** @brief  GAP - Advertisement data (max size = 31 bytes, best kept short to conserve power) */
static uint8_t adv_data[] =
{
    /* Flags */
    0x02,             /* length */
    GAP_ADTYPE_FLAGS, /* type="Flags" */
    GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    /* Service */
    0x03,             /* length */
    GAP_ADTYPE_16BIT_COMPLETE,
    LO_WORD(GATT_UUID_SIMPLE_PROFILE),
    HI_WORD(GATT_UUID_SIMPLE_PROFILE),
    /* Local name */
    APP_BLE_LOCAL_NAME_LEN + 1, /* length = type + name bytes */
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'Z', 'P', '-', '0', '0', '0', '0', '0', '0',
};
static T_SERVER_ID s_factory_mfg_srv_id = 0xFFU;

void bt_stack_config_init(void)
{
    gap_config_max_le_paired_device((uint8_t)ZY100_BLE_MAX_PAIRED_CENTRALS);
}

static void app_log_gd25q32e_unique_id_once(void)
{
    uint8_t unique_id[APP_GD25Q32E_UNIQUE_ID_LEN] = {0U};
    uint32_t id0;
    uint32_t id1;
    uint32_t id2;
    uint32_t id3;

    if (!flash_read_unique_id_gd(unique_id))
    {
        ZY100_LOG_ERROR("[ERR][BOOT] flash_uid_read_failed");
        return;
    }

    id0 = zy100_get_u32_be(&unique_id[0]);
    id1 = zy100_get_u32_be(&unique_id[4]);
    id2 = zy100_get_u32_be(&unique_id[8]);
    id3 = zy100_get_u32_be(&unique_id[12]);

    DBG_DIRECT("[BOOT][FLASH_UID] gd25q32e=%08lX%08lX%08lX%08lX",
               (unsigned long)id0,
               (unsigned long)id1,
               (unsigned long)id2,
               (unsigned long)id3);
}

#if F_APP_BUTTON_LOG_ENABLE
static uint32_t s_app_button_gpio_pin = 0U;
static volatile uint32_t s_app_button_irq_total_count = 0U;
static volatile uint32_t s_app_button_irq_pending_count = 0U;
#if F_BT_DLPS_EN && F_APP_BUTTON_DLPS_CTRL_ENABLE && !F_LED_EDGE_HW_TEST_ENABLE
static volatile app_button_input_edge_t s_app_button_edge_fifo[APP_BUTTON_EDGE_FIFO_RING_SIZE];
static volatile uint8_t s_app_button_edge_head = 0U;
static volatile uint8_t s_app_button_edge_tail = 0U;
static volatile bool s_app_button_edge_fault = false;
static volatile bool s_app_button_gpio_irq_active = false;
#endif
#endif


/*============================================================================*
 *                              Functions
 *============================================================================*/
#if F_APP_BUTTON_LOG_ENABLE
static void app_button_board_init(void)
{
    Pad_Config(APP_BUTTON_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
               PAD_OUT_HIGH);
    Pinmux_Config(APP_BUTTON_PIN, DWGPIO);
    s_app_button_gpio_pin = GPIO_GetPin(APP_BUTTON_PIN);
}

#if F_BT_DLPS_EN && F_APP_BUTTON_DLPS_CTRL_ENABLE && !F_LED_EDGE_HW_TEST_ENABLE
static uint8_t app_button_edge_fifo_clear(void)
{
    uint8_t head = s_app_button_edge_head;
    uint8_t tail = s_app_button_edge_tail;
    uint8_t dropped;

    if (head >= tail)
    {
        dropped = (uint8_t)(head - tail);
    }
    else
    {
        dropped = (uint8_t)(APP_BUTTON_EDGE_FIFO_RING_SIZE - tail + head);
    }

    s_app_button_edge_tail = head;
    s_app_button_edge_fault = false;
    if (s_app_button_gpio_pin != 0U)
    {
        GPIO_ClearINTPendingBit(s_app_button_gpio_pin);
    }
    NVIC_ClearPendingIRQ(APP_BUTTON_GPIO_IRQN);
    return dropped;
}

static void app_button_gpio_irq_mask(void)
{
    if (s_app_button_gpio_pin != 0U)
    {
        GPIO_INTConfig(s_app_button_gpio_pin, DISABLE);
        GPIO_MaskINTConfig(s_app_button_gpio_pin, ENABLE);
    }
    NVIC_DisableIRQ(APP_BUTTON_GPIO_IRQN);
}

static void app_button_gpio_irq_disable(void)
{
    if (s_app_button_gpio_pin == 0U)
    {
        (void)app_button_edge_fifo_clear();
        s_app_button_gpio_irq_active = false;
        return;
    }

    app_button_gpio_irq_mask();
    (void)app_button_edge_fifo_clear();
    s_app_button_gpio_irq_active = false;
}

static void app_button_gpio_irq_enable(void)
{
    GPIO_InitTypeDef gpio_init;
    NVIC_InitTypeDef nvic_init;
    uint32_t irq_key;
    uint32_t saved_mask;

    if (s_app_button_gpio_pin == 0U)
    {
        return;
    }

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    if (s_app_button_edge_fault)
    {
        /* DLPS restore must not discard a fault before the task cancels it. */
        app_button_gpio_irq_mask();
        return;
    }
    app_button_gpio_irq_disable();
    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = s_app_button_gpio_pin;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_ITCmd = ENABLE;
    gpio_init.GPIO_ITTrigger = GPIO_INT_BOTH_EDGE;
    gpio_init.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
    gpio_init.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;
    gpio_init.GPIO_DebounceTime = APP_BUTTON_DEBOUNCE_MS;
    gpio_init.GPIO_ControlMode = GPIO_SOFTWARE_MODE;
    /* SDK GPIO_Init configures trigger/debounce only with ITCmd=ENABLE, and
     * clears the global INTMASK.  Preserve all other pins under a short lock.
     * INTEN and NVIC remain disabled until configuration is complete. */
    irq_key = os_lock();
    saved_mask = GPIO->INTMASK;
    GPIO_Init(&gpio_init);
    GPIO_INTConfig(s_app_button_gpio_pin, DISABLE);
    GPIO->INTMASK = saved_mask | s_app_button_gpio_pin;
    os_unlock(irq_key);

    nvic_init.NVIC_IRQChannel = APP_BUTTON_GPIO_IRQN;
    nvic_init.NVIC_IRQChannelPriority = 3;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    (void)app_button_edge_fifo_clear();
    if (!app_task_button_input_wants_edges()) return;
    s_app_button_gpio_irq_active = true;
    /* SDK NVIC_Init only installs priority in its ENABLE branch.  Keep GPIO
     * masked until that priority is safe for the ISR's FreeRTOS calls. */
    NVIC_Init(&nvic_init);
    GPIO_MaskINTConfig(s_app_button_gpio_pin, DISABLE);
    GPIO_INTConfig(s_app_button_gpio_pin, ENABLE);
}
#endif

static void app_button_driver_init(void)
{
#if F_BT_DLPS_EN && F_APP_BUTTON_DLPS_CTRL_ENABLE && !F_LED_EDGE_HW_TEST_ENABLE
    app_button_gpio_irq_enable();
    return;
#else
    GPIO_InitTypeDef gpio_init;

    if (s_app_button_gpio_pin == 0U)
    {
        return;
    }

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);

    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = s_app_button_gpio_pin;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
#if F_LED_EDGE_HW_TEST_ENABLE
    gpio_init.GPIO_ITCmd = ENABLE;
    gpio_init.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
#if (F_APP_BUTTON_ACTIVE_LEVEL == 0)
    gpio_init.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
#else
    gpio_init.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_HIGH;
#endif
    gpio_init.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;
    gpio_init.GPIO_DebounceTime = APP_BUTTON_DEBOUNCE_MS;
#else
    gpio_init.GPIO_ITCmd = DISABLE;
#endif
    gpio_init.GPIO_ControlMode = GPIO_SOFTWARE_MODE;
    GPIO_Init(&gpio_init);

#if F_LED_EDGE_HW_TEST_ENABLE
    {
        NVIC_InitTypeDef nvic_init;

        nvic_init.NVIC_IRQChannel = APP_BUTTON_GPIO_IRQN;
        nvic_init.NVIC_IRQChannelPriority = 3;
        nvic_init.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&nvic_init);

        s_app_button_irq_total_count = 0U;
        s_app_button_irq_pending_count = 0U;
        GPIO_ClearINTPendingBit(s_app_button_gpio_pin);
        GPIO_MaskINTConfig(s_app_button_gpio_pin, DISABLE);
        GPIO_INTConfig(s_app_button_gpio_pin, ENABLE);
    }
#endif
#endif
}
#endif

#if F_APP_BUTTON_LOG_ENABLE && F_BT_DLPS_EN
static void app_button_gpio_input_restore(void)
{
#if F_APP_BUTTON_DLPS_CTRL_ENABLE
    if (s_app_button_gpio_pin == 0U)
    {
        return;
    }

    Pad_Config(APP_BUTTON_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);
    Pinmux_Config(APP_BUTTON_PIN, DWGPIO);

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);

    app_button_gpio_irq_enable();
#else
    GPIO_InitTypeDef gpio_init;

    if (s_app_button_gpio_pin == 0U)
    {
        return;
    }

    Pad_Config(APP_BUTTON_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE,
               PAD_OUT_HIGH);
    Pinmux_Config(APP_BUTTON_PIN, DWGPIO);
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = s_app_button_gpio_pin;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_ITCmd = DISABLE;
    gpio_init.GPIO_ControlMode = GPIO_SOFTWARE_MODE;
    GPIO_Init(&gpio_init);
    GPIO_INTConfig(s_app_button_gpio_pin, DISABLE);
    GPIO_MaskINTConfig(s_app_button_gpio_pin, ENABLE);
    GPIO_ClearINTPendingBit(s_app_button_gpio_pin);
#endif
}
#endif

bool app_button_read_level(uint8_t *level)
{
#if F_APP_BUTTON_LOG_ENABLE
    if ((level == NULL) || (s_app_button_gpio_pin == 0U))
    {
        return false;
    }

    *level = GPIO_ReadInputDataBit(s_app_button_gpio_pin);
    return true;
#else
    (void)level;
    return false;
#endif
}

bool app_button_input_take_edge(app_button_input_edge_t *edge)
{
#if F_APP_BUTTON_LOG_ENABLE && F_BT_DLPS_EN && F_APP_BUTTON_DLPS_CTRL_ENABLE && \
    !F_LED_EDGE_HW_TEST_ENABLE
    uint8_t tail;

    if ((edge == NULL) || (s_app_button_edge_tail == s_app_button_edge_head))
    {
        return false;
    }

    tail = s_app_button_edge_tail;
    edge->timestamp_ms = s_app_button_edge_fifo[tail].timestamp_ms;
    edge->level = s_app_button_edge_fifo[tail].level;
    s_app_button_edge_tail = (uint8_t)((tail + 1U) % APP_BUTTON_EDGE_FIFO_RING_SIZE);
    return true;
#else
    (void)edge;
    return false;
#endif
}

bool app_button_input_edge_pending(void)
{
#if F_APP_BUTTON_LOG_ENABLE && F_BT_DLPS_EN && F_APP_BUTTON_DLPS_CTRL_ENABLE && \
    !F_LED_EDGE_HW_TEST_ENABLE
    return (s_app_button_edge_head != s_app_button_edge_tail) || s_app_button_edge_fault;
#else
    return false;
#endif
}

bool app_button_input_take_fault(void)
{
#if F_APP_BUTTON_LOG_ENABLE && F_BT_DLPS_EN && F_APP_BUTTON_DLPS_CTRL_ENABLE && \
    !F_LED_EDGE_HW_TEST_ENABLE
    if (!s_app_button_edge_fault)
    {
        return false;
    }

    /* Keep the pause latched until task-side flush/disable, even if sampling
     * the current key level temporarily fails after this query. */
    return true;
#else
    return false;
#endif
}

uint8_t app_button_input_flush(void)
{
#if F_APP_BUTTON_LOG_ENABLE && F_BT_DLPS_EN && F_APP_BUTTON_DLPS_CTRL_ENABLE && \
    !F_LED_EDGE_HW_TEST_ENABLE
    bool restore_irq = s_app_button_gpio_irq_active;
    uint8_t dropped;

    app_button_gpio_irq_mask();
    dropped = app_button_edge_fifo_clear();
    if (restore_irq && (s_app_button_gpio_pin != 0U))
    {
        app_button_gpio_irq_enable();
    }
    return dropped;
#else
    return 0U;
#endif
}

bool app_button_input_capture_active(void)
{
#if F_APP_BUTTON_LOG_ENABLE && F_BT_DLPS_EN && F_APP_BUTTON_DLPS_CTRL_ENABLE && \
    !F_LED_EDGE_HW_TEST_ENABLE
    return s_app_button_gpio_irq_active;
#else
    return false;
#endif
}

void app_button_input_set_capture_enabled(bool enabled)
{
#if F_APP_BUTTON_LOG_ENABLE && F_BT_DLPS_EN && F_APP_BUTTON_DLPS_CTRL_ENABLE && \
    !F_LED_EDGE_HW_TEST_ENABLE
    if (enabled == s_app_button_gpio_irq_active) return;
    if (enabled) app_button_gpio_input_restore();
    else app_button_gpio_irq_disable();
#else
    (void)enabled;
#endif
}

#if F_APP_BUTTON_LOG_ENABLE && F_BT_DLPS_EN
static volatile uint32_t s_app_button_dlps_enter_count = 0U;
static volatile uint32_t s_app_button_dlps_exit_count = 0U;
typedef enum
{
    APP_BUTTON_DLPS_WAKE_POL_UNKNOWN = 0U,
    APP_BUTTON_DLPS_WAKE_POL_NORMAL_PRESS,
    APP_BUTTON_DLPS_WAKE_POL_RELEASE,
} app_button_dlps_wake_polarity_t;
static volatile app_button_dlps_wake_polarity_t s_app_button_dlps_wake_polarity =
    APP_BUTTON_DLPS_WAKE_POL_UNKNOWN;
static volatile uint32_t s_app_system_irq_unknown_count = 0U;

bool app_button_dlps_rearm_wakeup(void)
{
#if F_APP_BUTTON_DLPS_CTRL_ENABLE
    app_button_gpio_irq_disable();
#endif
    Pad_Config(APP_BUTTON_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP,
               PAD_OUT_DISABLE, PAD_OUT_LOW);
    System_WakeUpDebounceTime(0x8);
    Pad_ClearWakeupINTPendingBit(APP_BUTTON_PIN);
    NVIC_ClearPendingIRQ(System_IRQn);
    System_WakeUpPinEnable(APP_BUTTON_PIN, PAD_WAKEUP_POL_LOW, PAD_WK_DEBOUNCE_ENABLE);
    s_app_button_dlps_wake_polarity = APP_BUTTON_DLPS_WAKE_POL_NORMAL_PRESS;
    NVIC_EnableIRQ(System_IRQn);
    return true;
}

bool app_button_dlps_rearm_release_wakeup(void)
{
#if F_APP_BUTTON_DLPS_CTRL_ENABLE
    app_button_gpio_irq_disable();
#endif
    Pad_Config(APP_BUTTON_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP,
               PAD_OUT_DISABLE, PAD_OUT_LOW);
    System_WakeUpDebounceTime(0x8);
    Pad_ClearWakeupINTPendingBit(APP_BUTTON_PIN);
    NVIC_ClearPendingIRQ(System_IRQn);
#if (F_APP_BUTTON_ACTIVE_LEVEL == 0)
    System_WakeUpPinEnable(APP_BUTTON_PIN, PAD_WAKEUP_POL_HIGH, PAD_WK_DEBOUNCE_ENABLE);
#else
    System_WakeUpPinEnable(APP_BUTTON_PIN, PAD_WAKEUP_POL_LOW, PAD_WK_DEBOUNCE_ENABLE);
#endif
    s_app_button_dlps_wake_polarity = APP_BUTTON_DLPS_WAKE_POL_RELEASE;
    NVIC_EnableIRQ(System_IRQn);
    return true;
}

static bool app_dlps_wake_pending_on_pin(uint8_t pin)
{
    /*
     * Realtek RTL8762D SDK exposes the DLPS wake pending latch through
     * System_WakeUpInterruptValue().  Keep that direct dependency contained
     * here; a GPIO level check would only be a last-resort fallback because it
     * cannot distinguish an armed wake latch from an ordinary pin level.
     */
    return (System_WakeUpInterruptValue(pin) == SET);
}

static bool app_button_dlps_current_wake_level_active(void)
{
    uint8_t level = 0U;
    uint8_t wake_level;
    app_button_dlps_wake_polarity_t polarity = s_app_button_dlps_wake_polarity;

    if (!app_button_read_level(&level))
    {
        return false;
    }

    switch (polarity)
    {
    case APP_BUTTON_DLPS_WAKE_POL_NORMAL_PRESS:
        wake_level = (uint8_t)F_APP_BUTTON_ACTIVE_LEVEL;
        break;
    case APP_BUTTON_DLPS_WAKE_POL_RELEASE:
#if (F_APP_BUTTON_ACTIVE_LEVEL == 0)
        wake_level = 1U;
#else
        wake_level = 0U;
#endif
        break;
    case APP_BUTTON_DLPS_WAKE_POL_UNKNOWN:
    default:
        return false;
    }

    return (level == wake_level);
}

static bool app_imu_wom_current_wake_level_active(void)
{
#if F_APP_BUTTON_DLPS_CTRL_ENABLE && (IMU_INT_PIN != IMU_PIN_UNASSIGNED)
    return app_task_imu_wom_sleep_is_armed() && (imu_bsp_int_level() == 0U);
#else
    return false;
#endif
}

void app_button_quiesce_wakeup_irq(void)
{
    NVIC_DisableIRQ(System_IRQn);
    NVIC_ClearPendingIRQ(System_IRQn);
    Pad_ClearWakeupINTPendingBit(APP_BUTTON_PIN);
}

void app_button_dlps_get_counts(uint32_t *enter_count, uint32_t *exit_count)
{
    if (enter_count != NULL)
    {
        *enter_count = s_app_button_dlps_enter_count;
    }
    if (exit_count != NULL)
    {
        *exit_count = s_app_button_dlps_exit_count;
    }
}

static void app_button_dlps_enter_cb(void)
{
#if F_APP_BUTTON_LOG_ENABLE && F_BT_DLPS_EN && F_APP_BUTTON_DLPS_CTRL_ENABLE && \
    !F_LED_EDGE_HW_TEST_ENABLE
    if (!app_task_button_input_wants_edges()) app_button_gpio_irq_disable();
#endif
#if ZY100_OFFLINE_V2_WOM_START_ENABLE
    lp_dlps_note_enter();
#endif
    s_app_button_dlps_enter_count++;
#if F_APP_BUTTON_DLPS_CTRL_ENABLE
    return;
#else
    (void)app_button_dlps_rearm_wakeup();
    return;
#endif
}

static void app_button_dlps_exit_cb(void)
{
#if ZY100_OFFLINE_V2_WOM_START_ENABLE
    lp_dlps_note_exit();
#endif
    s_app_button_dlps_exit_count++;
    app_button_gpio_input_restore();
}

void System_Handler(void)
{
    bool button_pending = false;
    bool imu_pending = false;
    bool charge_pending = false;

    NVIC_DisableIRQ(System_IRQn);
    button_pending = app_dlps_wake_pending_on_pin(APP_BUTTON_PIN);
#if F_APP_BUTTON_DLPS_CTRL_ENABLE && ZY100_CHARGE_LED_ENABLE
    charge_pending = bsp_power_status_chg_int_wakeup_pending();
#endif

#if F_APP_BUTTON_DLPS_CTRL_ENABLE && (IMU_INT_PIN != IMU_PIN_UNASSIGNED)
    if (app_task_imu_wom_sleep_is_armed())
    {
        imu_pending = app_dlps_wake_pending_on_pin(IMU_INT_PIN);
    }
#endif

    if (!button_pending)
    {
        button_pending = app_button_dlps_current_wake_level_active();
    }

#if F_APP_BUTTON_DLPS_CTRL_ENABLE && (IMU_INT_PIN != IMU_PIN_UNASSIGNED)
    if (!imu_pending)
    {
        imu_pending = app_imu_wom_current_wake_level_active();
    }
#endif

#if F_APP_BUTTON_DLPS_CTRL_ENABLE && ZY100_CHARGE_LED_ENABLE
    if (charge_pending)
    {
#if ZY100_CHG_WAKE_DIAG_LOG_ENABLE
        DBG_DIRECT("[SYS_WAKE] dispatch order=chg_first button=%u imu=%u chg=1",
                   button_pending ? 1U : 0U,
                   imu_pending ? 1U : 0U);
#endif
        bsp_power_status_chg_int_clear_wakeup();
        app_charge_notify_wakeup_irq();
        app_charge_notify_app_task_from_isr();
    }
#endif

    if (button_pending)
    {
        Pad_ClearWakeupINTPendingBit(APP_BUTTON_PIN);
        app_button_notify_wakeup_irq();
        app_button_notify_app_task_from_isr();
    }

#if F_APP_BUTTON_DLPS_CTRL_ENABLE && (IMU_INT_PIN != IMU_PIN_UNASSIGNED)
    if (imu_pending)
    {
#if F_APP_BUTTON_DLPS_CTRL_ENABLE && ZY100_CHARGE_LED_ENABLE
        if (charge_pending)
        {
            Pad_ClearWakeupINTPendingBit(IMU_INT_PIN);
#if ZY100_CHG_WAKE_DIAG_LOG_ENABLE
            DBG_DIRECT("[SYS_WAKE] imu_wom_suppressed_by_chg");
#endif
        }
        else
#endif
        {
            Pad_ClearWakeupINTPendingBit(IMU_INT_PIN);
            app_imu_wom_notify_wakeup_irq();
            app_imu_wom_notify_app_task_from_isr();
        }
    }
#endif

    if (!button_pending && !imu_pending && !charge_pending)
    {
        s_app_system_irq_unknown_count++;
        DBG_DIRECT("[SYS_WAKE][UNKNOWN] count=%lu",
                   (unsigned long)s_app_system_irq_unknown_count);
        NVIC_ClearPendingIRQ(System_IRQn);
        NVIC_EnableIRQ(System_IRQn);
        return;
    }

    NVIC_ClearPendingIRQ(System_IRQn);
}
#endif

#if F_APP_BUTTON_LOG_ENABLE && F_BT_DLPS_EN && F_APP_BUTTON_DLPS_CTRL_ENABLE && \
    !F_LED_EDGE_HW_TEST_ENABLE
void APP_BUTTON_GPIO_ISR(void)
{
    uint8_t head;
    uint8_t next_head;

    if (s_app_button_gpio_pin == 0U)
    {
        return;
    }

    if (!s_app_button_gpio_irq_active || !app_task_button_input_wants_edges())
    {
        app_button_gpio_irq_disable();
        return;
    }
    if (s_app_button_edge_fault ||
        GPIO_GetINTStatus(s_app_button_gpio_pin) != SET) return;

    GPIO_INTConfig(s_app_button_gpio_pin, DISABLE);
    GPIO_MaskINTConfig(s_app_button_gpio_pin, ENABLE);
    GPIO_ClearINTPendingBit(s_app_button_gpio_pin);

    head = s_app_button_edge_head;
    next_head = (uint8_t)((head + 1U) % APP_BUTTON_EDGE_FIFO_RING_SIZE);
    if (next_head == s_app_button_edge_tail)
    {
        s_app_button_edge_fault = true;
        /* GPIO is already masked.  Leave the fault latched and wake the task
         * once; only task-side cancellation/flush may resume edge capture. */
        NVIC_DisableIRQ(APP_BUTTON_GPIO_IRQN);
        app_button_notify_app_task_from_isr();
        return;
    }
    else
    {
        s_app_button_edge_fifo[head].timestamp_ms =
            (uint32_t)((uint32_t)xTaskGetTickCountFromISR() *
                       (uint32_t)portTICK_PERIOD_MS);
        s_app_button_edge_fifo[head].level = GPIO_ReadInputDataBit(s_app_button_gpio_pin);
        s_app_button_edge_head = next_head;
    }

    if (s_app_button_irq_total_count < 0xFFFFFFFFU)
    {
        s_app_button_irq_total_count++;
    }

    app_button_notify_app_task_from_isr();
    if (s_app_button_gpio_irq_active && app_task_button_input_wants_edges())
    {
        GPIO_MaskINTConfig(s_app_button_gpio_pin, DISABLE);
        GPIO_INTConfig(s_app_button_gpio_pin, ENABLE);
    }
}
#endif

#if F_APP_BUTTON_LOG_ENABLE && F_LED_EDGE_HW_TEST_ENABLE
void APP_BUTTON_GPIO_ISR(void)
{
    if (s_app_button_gpio_pin == 0U)
    {
        return;
    }

    GPIO_INTConfig(s_app_button_gpio_pin, DISABLE);
    GPIO_MaskINTConfig(s_app_button_gpio_pin, ENABLE);
    GPIO_ClearINTPendingBit(s_app_button_gpio_pin);

    if (s_app_button_irq_total_count < 0xFFFFFFFFU)
    {
        s_app_button_irq_total_count++;
    }
    if (s_app_button_irq_pending_count < 0xFFFFFFFFU)
    {
        s_app_button_irq_pending_count++;
    }

    GPIO_MaskINTConfig(s_app_button_gpio_pin, DISABLE);
    GPIO_INTConfig(s_app_button_gpio_pin, ENABLE);
}
#endif

#if F_LED_EDGE_HW_TEST_ENABLE
static void app_led_edge_hw_test_delay_us(uint32_t us)
{
    if ((us != 0U) && (platform_delay_us != NULL))
    {
        platform_delay_us(us);
        return;
    }

    if (us >= 1000U)
    {
        vTaskDelay(pdMS_TO_TICKS((us + 999U) / 1000U));
    }
}

static bool app_led_edge_hw_test_take_button_press(void)
{
    uint8_t level = 0U;

    if (s_app_button_irq_pending_count == 0U)
    {
        return false;
    }

    s_app_button_irq_pending_count--;
    if (!app_button_read_level(&level))
    {
        DBG_DIRECT("[LED_EDGE_TEST][BTN][ERR] read failed");
        return false;
    }

    if (level != (uint8_t)F_APP_BUTTON_ACTIVE_LEVEL)
    {
        DBG_DIRECT("[LED_EDGE_TEST][BTN] ignore level=%u", (unsigned int)level);
        return false;
    }

    return true;
}

static void app_led_edge_hw_test_wait_release(void)
{
    uint8_t level = (uint8_t)F_APP_BUTTON_ACTIVE_LEVEL;

    while (level == (uint8_t)F_APP_BUTTON_ACTIVE_LEVEL)
    {
        if (!app_button_read_level(&level))
        {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(LED_EDGE_HW_TEST_POLL_MS));
    }
}

static void app_led_edge_hw_test_send_once(void)
{
    bsp_led_power_status_t power_status;
    drv_rgb_led_status_t rgb_status;

    drv_rgb_led_data_low();
    power_status = bsp_led_power_on();
    if (power_status != BSP_LED_POWER_STATUS_OK)
    {
        drv_rgb_led_data_low();
        DBG_DIRECT("[LED_EDGE_TEST][ERR] power_on=%s(%u)",
                   bsp_led_power_status_name(power_status),
                   (unsigned int)power_status);
        return;
    }

    app_led_edge_hw_test_delay_us(LED_EDGE_HW_TEST_POWER_SETTLE_US);
    rgb_status = drv_rgb_led_send_edge_test(LED_EDGE_HW_TEST_EDGE_COUNT);
    drv_rgb_led_data_low();

    if (rgb_status == DRV_RGB_LED_STATUS_OK)
    {
        DBG_DIRECT("[LED_EDGE_TEST] sent edges=%u", (unsigned int)LED_EDGE_HW_TEST_EDGE_COUNT);
    }
    else
    {
        DBG_DIRECT("[LED_EDGE_TEST][ERR] send=%s(%u)",
                   drv_rgb_led_status_name(rgb_status),
                   (unsigned int)rgb_status);
    }
}

static void app_led_edge_hw_test_task(void *p_param)
{
    (void)p_param;

    DBG_DIRECT("[LED_EDGE_TEST] ready: press P1_0 button, edges=%u",
               (unsigned int)LED_EDGE_HW_TEST_EDGE_COUNT);

    for (;;)
    {
        if (app_led_edge_hw_test_take_button_press())
        {
            app_led_edge_hw_test_send_once();
            app_led_edge_hw_test_wait_release();
        }

        vTaskDelay(pdMS_TO_TICKS(LED_EDGE_HW_TEST_POLL_MS));
    }
}

static void app_led_edge_hw_test_task_init(void)
{
    if (xTaskCreate(app_led_edge_hw_test_task,
                    "led_edge",
                    LED_EDGE_HW_TEST_TASK_STACK_WORDS,
                    NULL,
                    LED_EDGE_HW_TEST_TASK_PRIORITY,
                    NULL) != pdPASS)
    {
        DBG_DIRECT("[LED_EDGE_TEST][ERR] task create failed");
    }
}
#endif


/**
  * @brief  Initialize peripheral and gap bond manager related parameters
  * @return void
  */
void app_le_gap_init(void)
{
    T_GAP_CAUSE cause;
    T_GAP_CAUSE cause_evt;
    T_GAP_CAUSE cause_addr_type;
    T_GAP_CAUSE cause_addr;
    T_GAP_CAUSE cause_channels;
    T_GAP_CAUSE cause_filter;
    T_GAP_CAUSE cause_int_min;
    T_GAP_CAUSE cause_int_max;
    const zp_mfg_record_t *mfg_record = factory_boot_gate_record();
    const char *ble_name = zy100_device_ble_name();
    uint8_t ble_name_len = (uint8_t)strlen(ble_name);

    /* Device name and device appearance */
    uint8_t  device_name[GAP_DEVICE_NAME_LEN] = {0};
    uint16_t appearance = GAP_GATT_APPEARANCE_UNKNOWN;
    uint8_t  slave_init_mtu_req = true;


    /* Advertising parameters */
    uint8_t  adv_evt_type = GAP_ADTYPE_ADV_IND;
    uint8_t  adv_direct_type = GAP_REMOTE_ADDR_LE_PUBLIC;
    uint8_t  adv_direct_addr[GAP_BD_ADDR_LEN] = {0};
    uint8_t  adv_chann_map = GAP_ADVCHAN_ALL;
    uint8_t  adv_filter_policy = GAP_ADV_FILTER_ANY;
    uint16_t adv_int_min = DEFAULT_ADVERTISING_INTERVAL_MIN;
    uint16_t adv_int_max = DEFAULT_ADVERTISING_INTERVAL_MAX;

    /* GAP Bond Manager parameters */
    uint8_t  auth_pair_mode = GAP_PAIRING_MODE_PAIRABLE;
    uint16_t auth_flags = GAP_AUTHEN_BIT_BONDING_FLAG;
    uint8_t  auth_io_cap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
    uint8_t  auth_oob = false;
    uint8_t  auth_use_fix_passkey = false;
    uint32_t auth_fix_passkey = 0;
    uint8_t  auth_sec_req_enable = true;
    uint16_t auth_sec_req_flags = GAP_AUTHEN_BIT_BONDING_FLAG;

    if (factory_boot_gate_locked_user_mode_active() && (mfg_record != NULL))
    {
        ble_name = mfg_record->ble_adv_name;
        ble_name_len = (uint8_t)strlen(ble_name);
    }
    if (zy100_production_acceptance_active())
    {
        ble_name = zy100_production_acceptance_ble_name();
        ble_name_len = (uint8_t)strlen(ble_name);
    }
    if (zy100_whole_unit_test_active())
    {
        ble_name = zy100_whole_unit_test_ble_name();
        ble_name_len = (uint8_t)strlen(ble_name);
    }

    if (ble_name_len > APP_BLE_LOCAL_NAME_LEN)
    {
        ble_name_len = APP_BLE_LOCAL_NAME_LEN;
    }

    /* Keep GAP device name and advertising local name exactly the same. */
    memcpy(device_name, ble_name, ble_name_len);
    memset(&adv_data[APP_ADV_LOCAL_NAME_OFFSET], 0, APP_BLE_LOCAL_NAME_LEN);
    memcpy(&adv_data[APP_ADV_LOCAL_NAME_OFFSET], ble_name, ble_name_len);
    ZY100_LOG_EVENT("[BLE_BOOT] gap name=%s len=%u adv_min=%u adv_max=%u",
                    ble_name,
                    (uint32_t)ble_name_len,
                    (uint32_t)adv_int_min,
                    (uint32_t)adv_int_max);

    /* Set device name and device appearance */
    le_set_gap_param(GAP_PARAM_DEVICE_NAME, ble_name_len, device_name);
    le_set_gap_param(GAP_PARAM_APPEARANCE, sizeof(appearance), &appearance);
    le_set_gap_param(GAP_PARAM_SLAVE_INIT_GATT_MTU_REQ, sizeof(slave_init_mtu_req),
                     &slave_init_mtu_req);

    /* Set advertising parameters */
    cause_evt = le_adv_set_param(GAP_PARAM_ADV_EVENT_TYPE,
                                 sizeof(adv_evt_type), &adv_evt_type);
    cause_addr_type = le_adv_set_param(GAP_PARAM_ADV_DIRECT_ADDR_TYPE,
                                       sizeof(adv_direct_type),
                                       &adv_direct_type);
    cause_addr = le_adv_set_param(GAP_PARAM_ADV_DIRECT_ADDR,
                                  sizeof(adv_direct_addr), adv_direct_addr);
    cause_channels = le_adv_set_param(GAP_PARAM_ADV_CHANNEL_MAP,
                                      sizeof(adv_chann_map), &adv_chann_map);
    cause_filter = le_adv_set_param(GAP_PARAM_ADV_FILTER_POLICY,
                                    sizeof(adv_filter_policy),
                                    &adv_filter_policy);
    cause_int_min = le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MIN,
                                     sizeof(adv_int_min), &adv_int_min);
    cause_int_max = le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MAX,
                                     sizeof(adv_int_max), &adv_int_max);

    cause = le_adv_set_param(GAP_PARAM_ADV_DATA, sizeof(adv_data), (void *)adv_data);
    ZY100_LOG_DETAIL("[BLE_BOOT] gap adv_cfg evt=0x%x at=0x%x addr=0x%x ch=0x%x filter=0x%x",
                    cause_evt,
                    cause_addr_type,
                    cause_addr,
                    cause_channels,
                    cause_filter);
    ZY100_LOG_DETAIL("[BLE_BOOT] gap adv_cfg interval=0x%x/0x%x data=0x%x",
                    cause_int_min,
                    cause_int_max,
                    cause);
    APP_PRINT_INFO1("app_le_gap_init: GAP_PARAM_ADV_DATA cause 0x%x", cause);
    /* Keep scan response unset to avoid pre-start reject on some stack/image combinations. */

    if (factory_boot_gate_factory_mode_active() ||
        factory_boot_gate_production_blocked()
#if ZY100_BUILD_FACTORY
        || factory_boot_gate_locked_user_mode_active()
#endif
       )
    {
        auth_pair_mode = GAP_PAIRING_MODE_NO_PAIRING;
        auth_flags = 0U;
        auth_sec_req_enable = false;
        auth_sec_req_flags = 0U;
    }
    if (zy100_production_acceptance_active())
    {
        auth_pair_mode = GAP_PAIRING_MODE_NO_PAIRING;
        auth_flags = 0U;
        auth_sec_req_enable = false;
        auth_sec_req_flags = 0U;
    }
    if (zy100_whole_unit_test_active())
    {
        auth_pair_mode = GAP_PAIRING_MODE_NO_PAIRING;
        auth_flags = 0U;
        auth_sec_req_enable = false;
        auth_sec_req_flags = 0U;
    }

    /* Setup the GAP Bond Manager */
    gap_set_param(GAP_PARAM_BOND_PAIRING_MODE, sizeof(auth_pair_mode), &auth_pair_mode);
    gap_set_param(GAP_PARAM_BOND_AUTHEN_REQUIREMENTS_FLAGS, sizeof(auth_flags), &auth_flags);
    gap_set_param(GAP_PARAM_BOND_IO_CAPABILITIES, sizeof(auth_io_cap), &auth_io_cap);
    gap_set_param(GAP_PARAM_BOND_OOB_ENABLED, sizeof(auth_oob), &auth_oob);
    le_bond_set_param(GAP_PARAM_BOND_FIXED_PASSKEY, sizeof(auth_fix_passkey), &auth_fix_passkey);
    le_bond_set_param(GAP_PARAM_BOND_FIXED_PASSKEY_ENABLE, sizeof(auth_use_fix_passkey),
                      &auth_use_fix_passkey);
    le_bond_set_param(GAP_PARAM_BOND_SEC_REQ_ENABLE, sizeof(auth_sec_req_enable), &auth_sec_req_enable);
    le_bond_set_param(GAP_PARAM_BOND_SEC_REQ_REQUIREMENT, sizeof(auth_sec_req_flags),
                      &auth_sec_req_flags);
    DBG_DIRECT("[BLE_PAIR] app_max_links=%u max_paired=%u target=%u",
               (uint32_t)APP_MAX_LINKS,
               (uint32_t)le_get_max_le_paired_device_num(),
               (uint32_t)ZY100_BLE_MAX_PAIRED_CENTRALS);

    /* register gap message callback */
    le_register_app_cb(app_gap_callback);
}

/**
 * @brief  Add GATT services and register callbacks
 * @return void
 */
#define APP_LE_PROFILE_SIMPLE_ID_SHIFT       0U
#define APP_LE_PROFILE_CONTROL_ID_SHIFT      8U
#define APP_LE_PROFILE_CALIBRATION_ID_SHIFT 16U
#define APP_LE_PROFILE_BAS_ID_SHIFT         24U
#define APP_LE_PROFILE_ID_GET(ids, shift) \
    ((T_SERVER_ID)(((ids) >> (shift)) & 0xFFUL))
#define APP_LE_PROFILE_ID_SET(ids, shift, value) \
    do \
    { \
        (ids) = ((ids) & ~(0xFFUL << (shift))) | \
                ((uint32_t)(value) << (shift)); \
    } while (0)

static __attribute__((noinline)) void app_le_profile_bind_router_service_ids(
    uint32_t simple_control_calibration_bas,
    T_SERVER_ID ota_id)
{
    app_ble_profile_router_bind_service_ids(
        APP_LE_PROFILE_ID_GET(simple_control_calibration_bas,
                              APP_LE_PROFILE_SIMPLE_ID_SHIFT),
        APP_LE_PROFILE_ID_GET(simple_control_calibration_bas,
                              APP_LE_PROFILE_CONTROL_ID_SHIFT),
        APP_LE_PROFILE_ID_GET(simple_control_calibration_bas,
                              APP_LE_PROFILE_CALIBRATION_ID_SHIFT),
        APP_LE_PROFILE_ID_GET(simple_control_calibration_bas,
                              APP_LE_PROFILE_BAS_ID_SHIFT),
        ota_id);
}

void app_le_profile_init(void)
{
    bool factory_mode = factory_boot_gate_factory_mode_active();
    bool diagnostic_mode = factory_boot_gate_production_blocked();
    bool acceptance_mode = zy100_production_acceptance_active();
    bool whole_unit_mode = zy100_whole_unit_test_active();
    uint8_t service_num;
    uint32_t registered_service_ids = 0xFFFFFFFFUL;
    T_SERVER_ID ota_id = 0xFFU;
    T_SERVER_ID special_mode_id = 0xFFU;

#if ZY100_BUILD_FACTORY
    diagnostic_mode = diagnostic_mode ||
                      factory_boot_gate_locked_user_mode_active();
#endif
    service_num = whole_unit_mode ?
                  APP_GATT_WHOLE_UNIT_SERVICE_NUM :
                  (acceptance_mode ?
                  APP_GATT_PRODUCTION_ACCEPTANCE_SERVICE_NUM :
                  (factory_mode ?
                   APP_GATT_FACTORY_MODE_SERVICE_NUM :
                   (diagnostic_mode ?
                    APP_GATT_DIAGNOSTIC_SERVICE_NUM :
                    APP_GATT_USER_SERVICE_NUM)));

    ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile begin factory=%u diag=%u accept=%u whole=%u services=%u",
               factory_mode ? 1U : 0U,
               diagnostic_mode ? 1U : 0U,
               acceptance_mode ? 1U : 0U,
               whole_unit_mode ? 1U : 0U,
               (uint32_t)service_num);

    app_le_profile_bind_router_service_ids(0xFFFFFFFFUL, 0xFFU);
    s_factory_mfg_srv_id = 0xFFU;
    ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile server_init begin services=%u",
               (uint32_t)service_num);
    server_init(service_num);
    ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile server_init done");
    if (whole_unit_mode)
    {
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=whole_unit begin");
        APP_LE_PROFILE_ID_SET(registered_service_ids,
                              APP_LE_PROFILE_BAS_ID_SHIFT,
                              bas_add_service(app_profile_callback));
        s_factory_mfg_srv_id =
            zy100_mfg_info_service_add(app_profile_callback);
        special_mode_id =
            zy100_whole_unit_test_add_service(app_profile_callback);
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=whole_unit done mfg=%u",
                        (uint32_t)s_factory_mfg_srv_id);
    }
    else if (acceptance_mode)
    {
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=acceptance begin");
        APP_LE_PROFILE_ID_SET(registered_service_ids,
                              APP_LE_PROFILE_BAS_ID_SHIFT,
                              bas_add_service(app_profile_callback));
        s_factory_mfg_srv_id =
            zy100_mfg_info_service_add(app_profile_callback);
        special_mode_id =
            zy100_production_acceptance_add_service(app_profile_callback);
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=acceptance done mfg=%u",
                        (uint32_t)s_factory_mfg_srv_id);
    }
    else if (factory_mode)
    {
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=factory_bas begin");
        APP_LE_PROFILE_ID_SET(registered_service_ids,
                              APP_LE_PROFILE_BAS_ID_SHIFT,
                              bas_add_service(app_profile_callback));
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=factory_bas done id=%u",
                   (uint32_t)APP_LE_PROFILE_ID_GET(
                       registered_service_ids,
                       APP_LE_PROFILE_BAS_ID_SHIFT));
#if ZY100_BUILD_FACTORY
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=factory_mfg begin");
        s_factory_mfg_srv_id = factory_ble_service_add_service(app_profile_callback);
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=factory_mfg done id=%u",
                   (uint32_t)s_factory_mfg_srv_id);
#endif
    }
    else if (diagnostic_mode)
    {
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=diag_bas begin");
        APP_LE_PROFILE_ID_SET(registered_service_ids,
                              APP_LE_PROFILE_BAS_ID_SHIFT,
                              bas_add_service(app_profile_callback));
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=diag_bas done id=%u",
                   (uint32_t)APP_LE_PROFILE_ID_GET(
                       registered_service_ids,
                       APP_LE_PROFILE_BAS_ID_SHIFT));
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=diag_mfg begin");
        s_factory_mfg_srv_id =
            zy100_mfg_info_service_add(app_profile_callback);
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=diag_mfg done id=%u",
                   (uint32_t)s_factory_mfg_srv_id);
    }
    else
    {
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=simple begin");
        APP_LE_PROFILE_ID_SET(
            registered_service_ids,
            APP_LE_PROFILE_SIMPLE_ID_SHIFT,
            simp_ble_service_add_service(app_profile_callback));
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=simple done id=%u",
                   (uint32_t)APP_LE_PROFILE_ID_GET(
                       registered_service_ids,
                       APP_LE_PROFILE_SIMPLE_ID_SHIFT));
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=ctrl begin");
        APP_LE_PROFILE_ID_SET(
            registered_service_ids,
            APP_LE_PROFILE_CONTROL_ID_SHIFT,
            zy100_ble_ctrl_service_add_service(app_profile_callback));
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=ctrl done id=%u",
                   (uint32_t)APP_LE_PROFILE_ID_GET(
                       registered_service_ids,
                       APP_LE_PROFILE_CONTROL_ID_SHIFT));
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile cal_manager begin");
        zy100_cal_manager_init();
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile cal_manager done");
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=cal begin");
        APP_LE_PROFILE_ID_SET(
            registered_service_ids,
            APP_LE_PROFILE_CALIBRATION_ID_SHIFT,
            zy100_cal_ble_service_add_service(app_profile_callback));
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=cal done id=%u",
                   (uint32_t)APP_LE_PROFILE_ID_GET(
                       registered_service_ids,
                       APP_LE_PROFILE_CALIBRATION_ID_SHIFT));
#endif
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=bas begin");
        APP_LE_PROFILE_ID_SET(registered_service_ids,
                              APP_LE_PROFILE_BAS_ID_SHIFT,
                              bas_add_service(app_profile_callback));
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=bas done id=%u",
                   (uint32_t)APP_LE_PROFILE_ID_GET(
                       registered_service_ids,
                       APP_LE_PROFILE_BAS_ID_SHIFT));
#if F_BLE_OTA_SERVICE_ENABLE
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=ota begin");
        ota_id = ota_add_service(app_profile_callback);
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=ota done id=%u",
                   (uint32_t)ota_id);
#endif
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=mfg begin");
        s_factory_mfg_srv_id =
            zy100_mfg_info_service_add(app_profile_callback);
        ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile add=mfg done id=%u",
                   (uint32_t)s_factory_mfg_srv_id);
    }
    app_le_profile_bind_router_service_ids(
        registered_service_ids,
        ota_id);
    APP_PRINT_INFO2(
        "app_le_profile_init: simp_srv_id %d, bas_srv_id %d",
        APP_LE_PROFILE_ID_GET(registered_service_ids,
                              APP_LE_PROFILE_SIMPLE_ID_SHIFT),
        APP_LE_PROFILE_ID_GET(registered_service_ids,
                              APP_LE_PROFILE_BAS_ID_SHIFT));
    DBG_DIRECT("[BLE_PROFILE] simple=%u ctrl=%u bas=%u ota=%u factory=%u service_num=%u",
               (uint32_t)APP_LE_PROFILE_ID_GET(
                   registered_service_ids,
                   APP_LE_PROFILE_SIMPLE_ID_SHIFT),
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
               (uint32_t)APP_LE_PROFILE_ID_GET(
                   registered_service_ids,
                   APP_LE_PROFILE_CONTROL_ID_SHIFT),
#else
               0xFFU,
#endif
               (uint32_t)APP_LE_PROFILE_ID_GET(
                   registered_service_ids,
                   APP_LE_PROFILE_BAS_ID_SHIFT),
#if F_BLE_OTA_SERVICE_ENABLE
               (uint32_t)ota_id,
#else
               0xFFU,
#endif
               (uint32_t)s_factory_mfg_srv_id,
               (uint32_t)service_num);
    if ((APP_LE_PROFILE_ID_GET(registered_service_ids,
                               APP_LE_PROFILE_BAS_ID_SHIFT) == 0xFFU) ||
        (factory_mode && (s_factory_mfg_srv_id == 0xFFU)) ||
        ((!factory_mode) && diagnostic_mode &&
         (s_factory_mfg_srv_id == 0xFFU)) ||
        ((acceptance_mode || whole_unit_mode) &&
         ((s_factory_mfg_srv_id == 0xFFU) ||
          (special_mode_id == 0xFFU))) ||
        ((!factory_mode) && (!diagnostic_mode) &&
         (!acceptance_mode) && (!whole_unit_mode) &&
         ((APP_LE_PROFILE_ID_GET(registered_service_ids,
                                 APP_LE_PROFILE_SIMPLE_ID_SHIFT) == 0xFFU) ||
          (s_factory_mfg_srv_id == 0xFFU)
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
          || (APP_LE_PROFILE_ID_GET(registered_service_ids,
                                    APP_LE_PROFILE_CONTROL_ID_SHIFT) == 0xFFU)
          || (APP_LE_PROFILE_ID_GET(
                  registered_service_ids,
                  APP_LE_PROFILE_CALIBRATION_ID_SHIFT) == 0xFFU)
#endif
#if F_BLE_OTA_SERVICE_ENABLE
          || (ota_id == 0xFFU)
#endif
          ))
       )
    {
        APP_PRINT_ERROR0("app_le_profile_init: BLE service pre-register failed");
    }
#if !F_BLE_OTA_SERVICE_ENABLE
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
    DBG_DIRECT("[BLE_OTA] service not registered in final_edge stage");
#else
    APP_PRINT_INFO0("app_le_profile_init: OTA service disabled for development target");
#endif
#endif
    ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile register_cb begin");
    server_register_app_cb(app_profile_callback);
    ZY100_BLE_DIAG_LOG("[BLE_BOOT] profile register_cb done");
#if F_BT_ANCS_CLIENT_SUPPORT
    client_init(1);
    ancs_init(APP_MAX_LINKS);
#endif
}

/**
 * @brief    Contains the initialization of pinmux settings and pad settings
 * @note     All the pinmux settings and pad settings shall be initiated in this function,
 *           but if legacy driver is used, the initialization of pinmux setting and pad setting
 *           should be peformed with the IO initializing.
 * @return   void
 */
void board_init(void)
{
#if F_APP_BUTTON_LOG_ENABLE
    app_button_board_init();
#endif
}

/**
 * @brief    Contains the initialization of peripherals
 * @note     Both new architecture driver and legacy driver initialization method can be used
 * @return   void
 */
void driver_init(void)
{
#if F_LED_EDGE_HW_TEST_ENABLE
    bsp_led_power_status_t power_status;
    drv_rgb_led_status_t rgb_status;

#if F_APP_BUTTON_LOG_ENABLE
    app_button_driver_init();
#endif

    power_status = bsp_led_power_init();
    if (power_status != BSP_LED_POWER_STATUS_OK)
    {
        DBG_DIRECT("[LED_EDGE_TEST][ERR] power_init=%s(%u)",
                   bsp_led_power_status_name(power_status),
                   (unsigned int)power_status);
        return;
    }

    rgb_status = drv_rgb_led_init();
    drv_rgb_led_data_low();
    if (rgb_status != DRV_RGB_LED_STATUS_OK)
    {
        DBG_DIRECT("[LED_EDGE_TEST][ERR] rgb_init=%s(%u)",
                   drv_rgb_led_status_name(rgb_status),
                   (unsigned int)rgb_status);
        return;
    }

    DBG_DIRECT("[LED_EDGE_TEST] driver init ok data_pin=%u pwr_pin=%u active_low=%u pull_up=%u",
               (unsigned int)ZY100_RGB_LED_DATA_PIN,
               (unsigned int)ZY100_LED_POWER_CTRL_PIN,
               (unsigned int)ZY100_RGB_LED_DATA_ACTIVE_LOW,
               (unsigned int)F_LED_EDGE_HW_TEST_DATA_PULL_UP_ENABLE);
    return;
#else
#if F_APP_BATTERY_ADC_ENABLE
    (void)battery_adc_init();
    battery_adc_guard_init();
#endif

#if F_APP_BUTTON_LOG_ENABLE
    app_button_driver_init();
#endif

#if !V0_IMU_FIFO_DRAIN_TEST
    DBG_DIRECT("[APP_TASK] driver init: button-only bootstrap mode");
#endif
#endif
}

/**
 * @brief    Contains the power mode settings
 * @return   void
 */
void pwr_mgr_init(void)
{
#if F_LED_EDGE_HW_TEST_ENABLE
#if F_APP_BUTTON_LOG_ENABLE && F_BT_DLPS_EN
    (void)app_button_dlps_enter_cb;
    (void)app_button_dlps_exit_cb;
#endif
    DBG_DIRECT("[LED_EDGE_TEST] pwr_mgr_init skipped");
#else
#if F_BT_DLPS_EN
#if F_APP_BUTTON_LOG_ENABLE && F_APP_BUTTON_DLPS_CTRL_ENABLE
    DLPS_IORegUserDlpsEnterCb(app_button_dlps_enter_cb);
    DLPS_IORegUserDlpsExitCb(app_button_dlps_exit_cb);
#endif
#if V0_IMU_FIFO_DRAIN_TEST
    if (dlps_check_cb_reg(app_task_v0_dlps_check) == false)
    {
        DBG_DIRECT("[V0_PWR][ERR] dlps check register failed");
    }
#endif
    NVIC_ClearPendingIRQ(System_IRQn);
    NVIC_SetPriority(System_IRQn, 3);
    NVIC_EnableIRQ(System_IRQn);
    DLPS_IORegister();
#if F_APP_BUTTON_LOG_ENABLE
    (void)app_button_dlps_rearm_wakeup();
#if F_APP_BUTTON_VERBOSE_LOG_ENABLE
    DBG_DIRECT("[APP_KEY] wake pin enabled on pin=%d (active=%d)",
               APP_BUTTON_PIN, (uint8_t)F_APP_BUTTON_ACTIVE_LEVEL);
#endif
#endif
    lps_mode_set(LPM_DLPS_MODE);
#endif
#endif
}

/**
 * @brief    Contains the initialization of all tasks
 * @note     There is only one task in BLE Peripheral APP, thus only one APP task is init here
 * @return   void
 */
void task_init(void)
{
#if F_LED_EDGE_HW_TEST_ENABLE
    app_led_edge_hw_test_task_init();
#else
    app_task_init();
#endif
}

/**
 * @brief    Entry of APP code
 * @return   int (To avoid compile warning)
 */
int main(void)
{
    extern uint32_t random_seed_value;
    srand(random_seed_value);
    board_init();
#if ZY100_BUILD_PRODUCTION && !ZY100_PRODUCTION_DETAIL_LOG_ENABLE
    /* Preserve the SDK trace catalog/shared App.trace; filter routine levels.
     * Direct lifecycle/errors and SDK WARN/ERROR remain enabled. */
    if (!log_module_bitmap_trace_set(0xFFFFFFFFFFFFFFFFULL, LEVEL_INFO, false) ||
        !log_module_bitmap_trace_set(0xFFFFFFFFFFFFFFFFULL, LEVEL_TRACE, false))
        ZY100_LOG_ERROR("[ERR][BOOT] log_level_filter_failed");
#endif
    if (!zy100_clock_config_apply_active_no_os_check())
    {
        ZY100_LOG_ERROR("[ERR][BOOT] early_clock_80m_failed hz=%u", get_cpu_clock());
    }
    ZY100_LOG_EVENT("[EVT][BOOT] fw=%s flavor=%s tag=%s hz=%u",
                    F_APP_FW_LOG_VERSION,
                    ZY100_BUILD_FLAVOR_NAME,
                    ZY100_BUILD_FLAVOR_TAG,
                    get_cpu_clock());
    app_log_gd25q32e_unique_id_once();
#if ZY100_RTC_CLOCK_ENABLE
    zy100_rtc_clock_init();
#endif
    factory_boot_gate_init();
    zy100_production_shipping_boot_init();
    zy100_production_acceptance_boot_init();
    zy100_whole_unit_test_boot_init();
    zy100_feature_config_init();

#if F_LED_EDGE_HW_TEST_ENABLE
    DBG_DIRECT("[LED_EDGE_TEST] hardware test mode enabled: BLE/sensor/DLPS bypassed");
    driver_init();
    task_init();
    os_sched_start();
#else
#if F_RGB_MARQUEE_TEST_ENABLE
    DBG_DIRECT("[RGB_TEST] marquee mode enabled: BLE/App/task/DLPS flow bypassed");
    zy100_rgb_marquee_test_run_forever();
#else
#if F_LP_TEST_PURE_DLPS
    /* Pure DLPS power mode:
     * keep BLE init/adv path disabled for low-power current measurement.
     * Still init DLPS framework first, then hold ACTIVE mode until countdown reaches zero.
     */
    pwr_mgr_init();
#if F_BT_DLPS_EN
    lps_mode_set(LPM_ACTIVE_MODE);
#endif
    task_init();
    os_sched_start();
#else
    /* Current capture-chain BLE main flow. */
#if F_APP_BUTTON_DLPS_CTRL_ENABLE
    pwr_mgr_init();
    task_init();
    os_sched_start();
#else
    le_gap_init(APP_MAX_LINKS);
    gap_lib_init();
    app_le_gap_init();
    /* GAP identity is available only after GAP initialization. Reload the
     * optional MFG v5 battery gain here so its bound BD_ADDR can be checked. */
    (void)battery_adc_calibration_reload();
    app_le_profile_init();
    pwr_mgr_init();
    task_init();
    os_sched_start();
#endif
#endif
#endif
#endif

    return 0;
}
/** @} */ /* End of group PERIPH_DEMO_MAIN */
