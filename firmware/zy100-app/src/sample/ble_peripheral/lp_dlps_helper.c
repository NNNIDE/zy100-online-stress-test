#include "service/power_diag.h"
/**
*****************************************************************************************
*     Copyright(c) 2026, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
* @file      lp_dlps_helper.c
* @brief     Packaged low-power DLPS helper implementation for ble_peripheral sample.
*****************************************************************************************
*/

/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include <os_sched.h>
#include <os_task.h>
#include <os_timer.h>
#include <stddef.h>
#include <trace.h>
#include <rtl876x_rcc.h>
#include <rtl876x_pinmux.h>
#include <rtl876x_spi.h>
#include <rtl876x_gpio.h>
#include <platform_utils.h>
#include "app_flags.h"
#include "bsp/bsp_battery_adc.h"
#include "bsp/bsp_led_power.h"
#include "bsp/imu_board_pinmap.h"
#include "bsp/led_board_pinmap.h"
#include "bsp/mag_board_pinmap.h"
#include "driver/gd25q32e_spi.h"
#include "lp_dlps_helper.h"
#include "bsp/imu_bsp.h"
#include "os_sync.h"
#if ZY100_OFFLINE_V2_WOM_START_ENABLE
#include "service/zy100_rtc_clock.h"
#endif

#if F_BT_DLPS_EN
#include <dlps.h>
#include <rtl876x_io_dlps.h>
#endif

#ifndef APP_BUTTON_PIN
#define APP_BUTTON_PIN P1_0
#endif

#ifndef V0_DLPS_DEBUG_KEEP_LOG_PIN
#define V0_DLPS_DEBUG_KEEP_LOG_PIN 0
#endif

#if V0_IMU_FIFO_DRAIN_TEST && ZY100_LOG_VERBOSE_DEFAULT
#define V0_LP_POLICY_STEP_LOG(msg) DBG_DIRECT(msg)
#else
#define V0_LP_POLICY_STEP_LOG(msg) do { } while (0)
#endif

/* 1: keep UART logs in first stage; then auto-close and switch to pure measurement mode. */
#define LP_TEST_KEEP_LOG_UART          1
/* BLE-targeted builds retain LOG_TX for phone integration diagnostics. */
#define LP_TEST_RUNTIME_LOG_AUTO_CLOSE_ENABLE (!ZY100_BLE_TARGETED_LOG_ENABLE)
/* RTC A/B current test guard: direct UART/GDMA clock-off needs a safe restore plan. */
#ifndef ZY100_RTC_AB_ALLOW_UART_GDMA_CLK_OFF
#define ZY100_RTC_AB_ALLOW_UART_GDMA_CLK_OFF 0
#endif
#define LP_TEST_DIAG_WINDOW_MS         3000
/* Test-only log retention:
 * keep UART alive for a while before close so flash/DLPS diagnostics can be observed.
 * no performance constraints in this phase.
 */
#define LP_TEST_UART_PRE_CLOSE_HOLD_MS 1500U
#define LP_TEST_UART_POST_CLOSE_DRAIN_MS 50U
/* 0: do not force peripheral clock-off in packaged flow (debug baseline),
 * 1: force disable selected peripheral clocks before entering DLPS.
 */
#define LP_TEST_FORCE_CLK_OFF_EN       0
/* DLPS IO policy:
 * 0: keep current/default IO state, do not reconfigure any pad before entering DLPS.
 * 1: apply product-specific IO low-power parking in lp_dlps_config_pads().
 * Note: low-power policy for LED_SDB(P2_6) is default low state; do not force high.
 */
#define LP_TEST_DLPS_IO_CUSTOM_CFG_EN  0
/* Special IO test switch:
 * 1: force all exposed IO to SW mode + input mode + pull-down in DLPS entry path.
 * 0: keep current grouped IO strategy (G1~G6/custom as configured below).
 * This switch is for temporary current-test A/B and is fully reversible.
 */
#define LP_TEST_DLPS_IO_ALL_INPUT_PULLDOWN_EN 0
/* G1 low-power group:
 * Split test switches for KEY(P1_0)/VBAT_ADC(P2_7)/CHG(P0_2).
 * CHG-INT(P0_1) is parked explicitly at the end of the policy.
 * Current validated low-current combo:
 *   KEY=0, VBAT_ADC=1, CHG=1.
 * Keep KEY wake-source tuning for a later phase.
 */
#define LP_TEST_DLPS_IO_G1_CFG_EN      1
#define LP_TEST_DLPS_IO_G1_KEY_CFG_EN  0
#define LP_TEST_DLPS_IO_G1_VBAT_CFG_EN 1
#define LP_TEST_DLPS_IO_G1_CHG_CFG_EN  1
/* G2 low-power group (validated by current measurement):
 * In DLPS, LOG_TX/P3_0/P3_1 must be parked.
 * KEY(P1_0) is the wake source and is protected separately.
 * SENSOR-PWR(P1_1) is owned by G6.
 * LED_PWR(MICBIAS/P5_0, SDK H_0) is owned by the LED_PWR group.
 * Keep this policy independent from LED_SDB(P2_6).
 */
#define LP_TEST_DLPS_IO_G2_CFG_EN      1
/* G3 low-power group:
 * I2C_SCL(P2_5)/I2C_SDA(P2_4) -> input no-pull (high-Z intent).
 * Validated result: current has no obvious rise/fall, accepted as baseline.
 */
#define LP_TEST_DLPS_IO_G3_CFG_EN      1
/* G5 low-power group:
 * IMU dedicated pins:
 * G_SPI_CS(H_1) -> input weak pull-down;
 * G_SPI_INT(H_2) -> input no-pull.
 * Validated result: current has no obvious rise/fall, accepted as baseline.
 */
#define LP_TEST_DLPS_IO_G5_CFG_EN      1
/* G4 low-power group:
 * SPI_CLK(P4_0) -> input pull-down;
 * SPI_MOSI(P4_2) -> input pull-down;
 * SPI_MISO(P4_1) -> input pull-down;
 * FLASH_CS(P4_3) -> input pull-down.
 * Marked/kept: P4_0 input pull-down is the validated baseline for current measurement.
 * Validated result: current has no obvious rise/fall, accepted as baseline.
 */
#define LP_TEST_DLPS_IO_G4_CFG_EN      1
/* G6 low-power group:
 * SENSOR-PWR(P1_1) -> output high when sensor rail hold or IMU WoM wake is armed.
 * SENSOR-PWR(P1_1) -> output low only when sensor rail hold and WoM wake are both off.
 */
#define LP_TEST_DLPS_IO_G6_CFG_EN      1
/* Single-pin step test:
 * LED_SDB(P2_6) must be explicitly driven low in DLPS.
 * Validated effective by current measurement.
 */
#define LP_TEST_DLPS_IO_LED_SDB_CFG_EN 1
/* LED power rail:
 * LED_PWR(MICBIAS/P5_0, SDK H_0) -> input weak pull-down in DLPS.
 */
#define LP_TEST_DLPS_IO_LED_PWR_CFG_EN 1
/* FLASH shares SENSOR_VDD on ZY100 hardware.  Drive FLASH_CS high only when the
 * sensor rail hold policy keeps SENSOR-PWR on; otherwise park it as input.
 */
#define LP_TEST_FLASH_IN_SENSOR_DOMAIN 1
/* External FLASH (GD25Q32E) DLPS handling:
 * first step in packaged DLPS flow sends B9h (deep power-down) and prints SPI/ID diagnostics.
 * on DLPS wakeup, send ABh (release from deep power-down).
 * LP_TEST_FLASH_SPI_FLOW_EN is the master switch for this SPI communication flow.
 * 0: disable flash SPI B9/AB flow (keep other DLPS IO configuration unchanged).
 * 1: enable flash SPI B9/AB flow.
 */
#define LP_TEST_FLASH_SPI_FLOW_EN          0
#define LP_TEST_GD25Q32E_DPD_STEP_EN       LP_TEST_FLASH_SPI_FLOW_EN
#define LP_TEST_GD25Q32E_RELEASE_ON_WAKE_EN LP_TEST_FLASH_SPI_FLOW_EN
#define LP_TEST_GD25Q32E_CMD_B9            0xB9U
#define LP_TEST_GD25Q32E_CMD_AB            0xABU
#define LP_TEST_GD25Q32E_CMD_JEDEC_ID      0x9FU
#define LP_TEST_GD25Q32E_SPI_TIMEOUT_LOOP  200000U
#define LP_TEST_GD25Q32E_T_DP_US           20U
#define LP_TEST_GD25Q32E_T_RES1_US         40U
/* Stepwise peripheral-clock test:
 * framework on by default; every per-peripheral switch is off by default.
 * Safety boundary: do not touch TIMER/GPIO/FLASH clocks in this phase.
 */
#define LP_TEST_PERIPH_CLK_STEP_TEST_EN 1
#define LP_TEST_CLK_OFF_UART0_EN        0
#define LP_TEST_CLK_OFF_UART1_EN        0
#define LP_TEST_CLK_OFF_UART2_EN        0
#define LP_TEST_CLK_OFF_GDMA_EN         0
#define LP_TEST_CLK_OFF_I2C0_EN         0
#define LP_TEST_CLK_OFF_I2C1_EN         0
#define LP_TEST_CLK_OFF_SPI0_EN         0
#define LP_TEST_CLK_OFF_SPI1_EN         0
#define LP_TEST_CLK_OFF_SPI2W_EN        0
#define LP_TEST_CLK_OFF_ADC_EN          0
#define LP_TEST_CLK_OFF_KEYSCAN_EN      0
#define LP_TEST_CLK_OFF_QDEC_EN         0
#define LP_TEST_CLK_OFF_IR_EN           0
#define LP_TEST_CLK_OFF_IF8080_EN       0
#define LP_TEST_CLK_OFF_I2S0_EN         0
#define LP_TEST_CLK_OFF_I2S1_EN         0
#define LP_TEST_CLK_OFF_CODEC_EN        0
/* Timer step test:
 * keep baseline behavior by default (TIMER clock ON).
 * turn LP_TEST_TIMER_CLK_OFF_EN to 1 only for dedicated timer-risk experiments.
 * Current setting keeps TIMER clock ON for packaged DLPS path.
 */
#define LP_TEST_TIMER_STEP_TEST_EN      1
#define LP_TEST_TIMER_CLK_OFF_EN        0
/* Task step test:
 * framework on by default.
 * APP task suspend should remain enabled as baseline for pure DLPS current measurement.
 */
#define LP_TEST_TASK_STEP_TEST_EN       1
#define LP_TEST_TASK_STATUS_DUMP_EN     1
#define LP_TEST_TASK_SUSPEND_APP_EN     1

#if ZY100_OFFLINE_V2_WOM_START_ENABLE
static uint64_t s_lp_residency_enter_ticks;
static uint64_t s_lp_residency_ticks;
static bool s_lp_residency_valid;
static uint32_t s_lp_residency_invalid;
#endif
static volatile uint32_t s_lp_dlps_enter_cnt = 0;
static volatile uint32_t s_lp_dlps_exit_cnt = 0;
static volatile uint8_t s_lp_dlps_enter_seen = 0;
static volatile uint8_t s_lp_dlps_exit_seen = 0;
static volatile uint8_t s_lp_packaged_mode_entered = 0;
#if LP_TEST_GD25Q32E_DPD_STEP_EN
static volatile uint8_t s_lp_flash_dpd_cmd_sent = 0;
static volatile uint32_t s_lp_flash_jedec_id_u24 = 0;
#endif
#if (LP_TEST_KEEP_LOG_UART && LP_TEST_RUNTIME_LOG_AUTO_CLOSE_ENABLE)
static void *s_lp_test_task_handle = NULL;
#endif
static bool s_lp_imu_wom_wake_armed = false;
static bool s_lp_sensor_power_hold = false;
static bool s_lp_sleep_charge_led_hold = false;
static bool s_lp_sleep_charge_final_hold_logged = false;

void lp_dlps_set_imu_wom_wake_armed(bool armed)
{
    s_lp_imu_wom_wake_armed = armed;
}

void lp_dlps_set_sensor_power_hold(bool hold)
{
    s_lp_sensor_power_hold = hold;
}

static bool lp_dlps_sensor_power_should_hold(void)
{
    return s_lp_sensor_power_hold || s_lp_imu_wom_wake_armed;
}

void lp_dlps_set_sleep_charge_led_hold(bool hold)
{
    if (!hold)
    {
        s_lp_sleep_charge_final_hold_logged = false;
    }
    s_lp_sleep_charge_led_hold = hold;
}

bool lp_dlps_sleep_charge_led_hold_enabled(void)
{
    return s_lp_sleep_charge_led_hold;
}

static bool lp_dlps_is_protected_wake_pin(uint8_t pin)
{
    if (pin == APP_BUTTON_PIN)
    {
        return true;
    }
#if (IMU_INT_PIN != IMU_PIN_UNASSIGNED)
    if (s_lp_imu_wom_wake_armed && (pin == IMU_INT_PIN))
    {
        return true;
    }
#endif
    return false;
}

static void lp_dlps_notify_sensor_vdd_off(void)
{
    gd25q32e_notify_power_lost();
}

#if (LP_TEST_DLPS_IO_CUSTOM_CFG_EN || LP_TEST_DLPS_IO_ALL_INPUT_PULLDOWN_EN)
static bool lp_dlps_is_wom_owned_pin(uint8_t pin)
{
    if (!lp_dlps_sensor_power_should_hold())
    {
        return false;
    }

#if (IMU_POWER_CTRL_PIN != IMU_PIN_UNASSIGNED)
    if (pin == IMU_POWER_CTRL_PIN)
    {
        return true;
    }
#endif
#if (IMU_SPI_CS_PIN != IMU_PIN_UNASSIGNED)
    if (pin == IMU_SPI_CS_PIN)
    {
        return true;
    }
#endif
#if (IMU_INT_PIN != IMU_PIN_UNASSIGNED)
    if (pin == IMU_INT_PIN)
    {
        return s_lp_imu_wom_wake_armed;
    }
#endif

    return ((pin == P4_0) || (pin == P4_1) || (pin == P4_2) || (pin == P4_3));
}
#endif

#if (LP_TEST_KEEP_LOG_UART || LP_TEST_DLPS_IO_CUSTOM_CFG_EN || LP_TEST_DLPS_IO_ALL_INPUT_PULLDOWN_EN || LP_TEST_DLPS_IO_G1_CFG_EN || LP_TEST_DLPS_IO_G2_CFG_EN || LP_TEST_DLPS_IO_G3_CFG_EN || LP_TEST_DLPS_IO_G5_CFG_EN || LP_TEST_DLPS_IO_G4_CFG_EN || LP_TEST_DLPS_IO_G6_CFG_EN || LP_TEST_DLPS_IO_LED_PWR_CFG_EN)
static void lp_dlps_cfg_input(uint8_t pin, PAD_Pull_Mode pull_mode)
{
    if (lp_dlps_is_protected_wake_pin(pin))
    {
        return;
    }

    Pinmux_Deinit(pin);
    System_WakeUpPinDisable(pin);
    Pad_Config(pin, PAD_SW_MODE, PAD_IS_PWRON, pull_mode, PAD_OUT_DISABLE, PAD_OUT_LOW);
}
#endif

#if LP_TEST_DLPS_IO_LED_PWR_CFG_EN
static void lp_dlps_config_led_power(void);
#endif

#if LP_TEST_DLPS_IO_ALL_INPUT_PULLDOWN_EN
static void lp_dlps_config_all_io_input_pulldown(void)
{
    for (uint8_t pin = P0_0; pin <= P4_3; pin++)
    {
        if (lp_dlps_is_protected_wake_pin(pin) || lp_dlps_is_wom_owned_pin(pin))
        {
            continue;
        }
        lp_dlps_cfg_input(pin, PAD_PULL_DOWN);
    }

    /* Include MICBIAS/P5_0(LED_PWR) and 32K alternate pins in the same test policy. */
#if LP_TEST_DLPS_IO_LED_PWR_CFG_EN
    lp_dlps_config_led_power();
#else
    lp_dlps_cfg_input(H_0, PAD_PULL_DOWN);
#endif
    lp_dlps_cfg_input(H_1, PAD_PULL_DOWN);
    lp_dlps_cfg_input(H_2, PAD_PULL_DOWN);
}
#endif

#if LP_TEST_DLPS_IO_G1_CFG_EN
static void lp_dlps_cfg_adc_analog_no_digital(uint8_t pin)
{
    if (lp_dlps_is_protected_wake_pin(pin))
    {
        return;
    }

    if (pin == P2_7)
    {
        bsp_battery_adc_park_low_power();
        return;
    }

    Pinmux_Deinit(pin);
    System_WakeUpPinDisable(pin);
    /* Test mode for analog pin: keep SW mode/no-pull, disable output and shut pad digital power. */
    Pad_Config(pin, PAD_SW_MODE, PAD_NOT_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_LOW);
}
#endif

#if LP_TEST_DLPS_IO_G1_CFG_EN
static void lp_dlps_config_group_g1(void)
{
    /* G1 split test policy (validated): CHG input no-pull only in current phase. */
#if LP_TEST_DLPS_IO_G1_KEY_CFG_EN
    ZY100_LOG_VERBOSE("[LP_POLICY] KEY wake pin protected");
#endif
#if LP_TEST_DLPS_IO_G1_VBAT_CFG_EN
    lp_dlps_cfg_adc_analog_no_digital(P2_7); /* VBAT_ADC: analog path test */
#endif
#if LP_TEST_DLPS_IO_G1_CHG_CFG_EN
    lp_dlps_cfg_input(P0_2, PAD_PULL_NONE); /* CHG */
#endif
}
#endif

static void lp_dlps_config_chg_int(void)
{
    /* CHG-INT(P0_1): park as USB/VBUS input; app_task rearms wake after this policy. */
    lp_dlps_cfg_input(P0_1, PAD_PULL_NONE);
}

static void lp_dlps_cfg_output(uint8_t pin, PAD_Pull_Mode pull_mode, uint8_t high_level)
{
    if (lp_dlps_is_protected_wake_pin(pin))
    {
        return;
    }

    Pinmux_Deinit(pin);
    System_WakeUpPinDisable(pin);
    Pad_Config(pin, PAD_SW_MODE, PAD_IS_PWRON, pull_mode, PAD_OUT_ENABLE,
               high_level ? PAD_OUT_HIGH : PAD_OUT_LOW);
}

#if LP_TEST_DLPS_IO_LED_SDB_CFG_EN
static void lp_dlps_config_led_sdb(void)
{
    /* Single-pin step test policy: LED_SDB(P2_6) explicit output low in DLPS. */
    lp_dlps_cfg_output(P2_6, PAD_PULL_NONE, 0U);
}
#endif

#if LP_TEST_DLPS_IO_LED_PWR_CFG_EN
static void lp_dlps_config_led_power(void)
{
#if ZY100_LED_POWER_CTRL_PIN_AVAILABLE
    if (s_lp_sleep_charge_led_hold)
    {
        if (!bsp_led_power_is_on())
        {
            /* A charge animation may own a dark phase. Ownership alone
             * must not turn the LED rail back on after the renderer cut it. */
            (void)bsp_led_power_off();
            return;
        }
        lp_dlps_cfg_output(ZY100_LED_POWER_CTRL_PIN,
                           (ZY100_LED_POWER_CTRL_ACTIVE_HIGH != 0U) ? PAD_PULL_UP : PAD_PULL_NONE,
                           (ZY100_LED_POWER_CTRL_ACTIVE_HIGH != 0U) ? 1U : 0U);
        ZY100_LOG_VERBOSE("[LP_POLICY] LED_PWR sleep_charge_hold=1");
        return;
    }

    lp_dlps_cfg_input(ZY100_LED_POWER_CTRL_PIN, PAD_PULL_DOWN); /* LED_PWR / MICBIAS-P5_0 */
#else
    V0_LP_POLICY_STEP_LOG("[V0_LP_STEP] policy LED_PWR skipped");
#endif
}
#endif

bool lp_dlps_apply_sleep_charge_led_hold_now(void)
{
#if ZY100_LED_POWER_CTRL_PIN_AVAILABLE
    if (!s_lp_sleep_charge_led_hold)
    {
        return false;
    }
    if (!bsp_led_power_is_on())
    {
        return bsp_led_power_off() == BSP_LED_POWER_STATUS_OK;
    }

#if defined(ZY100_RGB_LED_DATA_PIN)
    lp_dlps_cfg_output(ZY100_RGB_LED_DATA_PIN,
                       PAD_PULL_NONE,
                       (ZY100_RGB_LED_DATA_ACTIVE_LOW != 0U) ? 1U : 0U);
#endif
    lp_dlps_cfg_output(ZY100_LED_POWER_CTRL_PIN,
                       (ZY100_LED_POWER_CTRL_ACTIVE_HIGH != 0U) ? PAD_PULL_UP : PAD_PULL_NONE,
                       (ZY100_LED_POWER_CTRL_ACTIVE_HIGH != 0U) ? 1U : 0U);

    if (!s_lp_sleep_charge_final_hold_logged)
    {
        ZY100_LOG_VERBOSE("[LP_POLICY] sleep_charge_hold_final=1 pwr_mode=%u pwr_oe=%u pwr_out=%u pwr_pwr=%u data_out=%u",
                          (uint32_t)bsp_led_power_ctrl_pad_mode_level(),
                          (uint32_t)bsp_led_power_ctrl_pad_oe_level(),
                          (uint32_t)bsp_led_power_ctrl_pad_out_level(),
                          (uint32_t)bsp_led_power_ctrl_pad_pwr_level(),
                          (uint32_t)bsp_led_power_data_out_level());
        s_lp_sleep_charge_final_hold_logged = true;
    }
    return true;
#else
    return false;
#endif
}

#if LP_TEST_DLPS_IO_G2_CFG_EN
static void lp_dlps_config_group_g2(void)
{
    /* Fixed DLPS policy for G2: all pins below are input weak pull-down. */
#if (V0_IMU_FIFO_DRAIN_TEST && V0_DLPS_DEBUG_KEEP_LOG_PIN)
    V0_LP_POLICY_STEP_LOG("[V0_LP_STEP] policy G2 keep LOG_TX debug pin");
#else
    lp_dlps_cfg_input(P0_3, PAD_PULL_DOWN); /* LOG_TX */
#endif
    lp_dlps_cfg_input(P3_0, PAD_PULL_DOWN); /* UART_TX */
    lp_dlps_cfg_input(P3_1, PAD_PULL_DOWN); /* UART_RX */
}
#endif

#if LP_TEST_DLPS_IO_G3_CFG_EN
static void lp_dlps_config_group_g3(void)
{
    /* G3 policy: I2C pins to input no-pull for DLPS measurement. */
    lp_dlps_cfg_input(MAG_I2C_SCL_PIN, PAD_PULL_NONE); /* I2C_SCL P2_5 */
    lp_dlps_cfg_input(MAG_I2C_SDA_PIN, PAD_PULL_NONE); /* I2C_SDA P2_4 */
}
#endif

#if LP_TEST_DLPS_IO_G5_CFG_EN
static void lp_dlps_config_group_g5(void)
{
    /* G5 policy: CS pull-down, INT no-pull. Held sensor rail keeps IMU CS inactive-high. */
#if (IMU_SPI_CS_PIN != IMU_PIN_UNASSIGNED)
    if (lp_dlps_sensor_power_should_hold())
    {
        lp_dlps_cfg_output(IMU_SPI_CS_PIN, PAD_PULL_NONE, 1U); /* G_SPI_CS */
    }
    else
    {
        lp_dlps_cfg_input(IMU_SPI_CS_PIN, PAD_PULL_DOWN); /* G_SPI_CS */
    }
#endif
#if (IMU_INT_PIN != IMU_PIN_UNASSIGNED)
    lp_dlps_cfg_input(IMU_INT_PIN, PAD_PULL_NONE);    /* G_SPI_INT */
#endif
}
#endif

#if LP_TEST_DLPS_IO_G4_CFG_EN
static void lp_dlps_config_group_g4(void)
{
    if (lp_dlps_sensor_power_should_hold())
    {
        lp_dlps_cfg_output(P4_0, PAD_PULL_NONE, 0U); /* SPI_CLK */
        lp_dlps_cfg_output(P4_2, PAD_PULL_NONE, 0U); /* SPI_MOSI */
        lp_dlps_cfg_input(P4_1, PAD_PULL_NONE);      /* SPI_MISO */
        lp_dlps_cfg_output(P4_3, PAD_PULL_NONE, 1U); /* FLASH_CS */
        return;
    }

    /* G4 policy: main SPI lines parked for DLPS measurement. */
    lp_dlps_cfg_input(P4_0, PAD_PULL_DOWN);          /* SPI_CLK (validated & kept) */
    lp_dlps_cfg_input(P4_2, PAD_PULL_DOWN);          /* SPI_MOSI */
    lp_dlps_cfg_input(P4_1, PAD_PULL_DOWN);          /* SPI_MISO */
    lp_dlps_cfg_input(P4_3, PAD_PULL_DOWN);          /* FLASH_CS */
}
#endif

#if LP_TEST_DLPS_IO_G6_CFG_EN
static void lp_dlps_config_group_g6(void)
{
    /* SENSOR-PWR is controlled by sensor-rail hold policy, not as a wake pin. */
#if (IMU_POWER_CTRL_PIN != IMU_PIN_UNASSIGNED)
    if (lp_dlps_sensor_power_should_hold())
    {
        if (imu_bsp_power_mode() == IMU_BSP_POWER_WEAK_PULL_UP)
        {
            lp_dlps_cfg_input(IMU_POWER_CTRL_PIN, PAD_PULL_UP);
            return;
        }
        lp_dlps_cfg_output(IMU_POWER_CTRL_PIN, PAD_PULL_NONE, 1U); /* SENSOR-PWR */
        return;
    }

    lp_dlps_cfg_output(IMU_POWER_CTRL_PIN, PAD_PULL_NONE, 0U); /* SENSOR-PWR */
    lp_dlps_notify_sensor_vdd_off();
#endif
}
#endif

#if LP_TEST_GD25Q32E_DPD_STEP_EN
static void lp_dlps_flash_delay_us(uint32_t delay_us)
{
    if ((delay_us != 0U) && (platform_delay_us != NULL))
    {
        platform_delay_us(delay_us);
    }
}

static bool lp_dlps_flash_spi_wait_flag(uint8_t flag, FlagStatus target)
{
    uint32_t guard = LP_TEST_GD25Q32E_SPI_TIMEOUT_LOOP;

    while (SPI_GetFlagState(SPI0, flag) != target)
    {
        if (guard-- == 0U)
        {
            return false;
        }
    }
    return true;
}

static void lp_dlps_flash_spi_flush_rx(void)
{
    while (SPI_GetFlagState(SPI0, SPI_FLAG_RFNE) == SET)
    {
        (void)SPI_ReceiveData(SPI0);
    }
}

static void lp_dlps_flash_spi_init(void)
{
    SPI_InitTypeDef spi_init;

    RCC_PeriphClockCmd(APBPeriph_SPI0, APBPeriph_SPI0_CLOCK, ENABLE);

    Pad_Config(P4_0, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE, PAD_OUT_HIGH);
    Pad_Config(P4_2, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE, PAD_OUT_HIGH);
    Pad_Config(P4_1, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE, PAD_OUT_HIGH);
    Pad_Config(P4_3, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE, PAD_OUT_HIGH);

    Pinmux_Deinit(P4_0);
    Pinmux_Deinit(P4_1);
    Pinmux_Deinit(P4_2);
    Pinmux_Deinit(P4_3);
    Pinmux_Config(P4_0, SPI0_CLK_MASTER);
    Pinmux_Config(P4_2, SPI0_MO_MASTER);
    Pinmux_Config(P4_1, SPI0_MI_MASTER);
    Pinmux_Config(P4_3, SPI0_SS_N_0_MASTER);

    /* Do not call SPI_DeInit(SPI0) here:
     * rtl876x_spi.c implementation disables APB SPI0 clock in SPI_DeInit,
     * which makes subsequent flash command transfer fail in this test path.
     */
    SPI_StructInit(&spi_init);
    spi_init.SPI_Direction = SPI_Direction_FullDuplex;
    spi_init.SPI_Mode = SPI_Mode_Master;
    spi_init.SPI_DataSize = SPI_DataSize_8b;
    spi_init.SPI_CPOL = SPI_CPOL_High;
    spi_init.SPI_CPHA = SPI_CPHA_2Edge;
    spi_init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    spi_init.SPI_RxThresholdLevel = 1U;
    spi_init.SPI_NDF = 0U;
    spi_init.SPI_FrameFormat = SPI_Frame_Motorola;
    SPI_Init(SPI0, &spi_init);
    SPI_Cmd(SPI0, ENABLE);
}

static bool lp_dlps_flash_spi_send_byte(uint8_t data)
{
    if (!lp_dlps_flash_spi_wait_flag(SPI_FLAG_TFNF, SET))
    {
        return false;
    }

    SPI_SendData(SPI0, data);
    return true;
}

static bool lp_dlps_flash_spi_send_cmd(uint8_t cmd)
{
    lp_dlps_flash_spi_flush_rx();
    if (!lp_dlps_flash_spi_send_byte(cmd))
    {
        return false;
    }
    if (!lp_dlps_flash_spi_wait_flag(SPI_FLAG_BUSY, RESET))
    {
        return false;
    }
    lp_dlps_flash_spi_flush_rx();
    return true;
}

static bool lp_dlps_flash_spi_read_jedec_id(uint8_t id_out[3])
{
    uint32_t guard;
    uint8_t rx_buf[4] = {0};

    if (id_out == NULL)
    {
        return false;
    }

    lp_dlps_flash_spi_flush_rx();
    if (!lp_dlps_flash_spi_send_byte(LP_TEST_GD25Q32E_CMD_JEDEC_ID))
    {
        return false;
    }
    if (!lp_dlps_flash_spi_send_byte(0x00U))
    {
        return false;
    }
    if (!lp_dlps_flash_spi_send_byte(0x00U))
    {
        return false;
    }
    if (!lp_dlps_flash_spi_send_byte(0x00U))
    {
        return false;
    }

    if (!lp_dlps_flash_spi_wait_flag(SPI_FLAG_BUSY, RESET))
    {
        return false;
    }

    guard = LP_TEST_GD25Q32E_SPI_TIMEOUT_LOOP;
    while (SPI_GetRxFIFOLen(SPI0) < 4U)
    {
        if (guard-- == 0U)
        {
            return false;
        }
    }

    rx_buf[0] = (uint8_t)SPI_ReceiveData(SPI0);
    rx_buf[1] = (uint8_t)SPI_ReceiveData(SPI0);
    rx_buf[2] = (uint8_t)SPI_ReceiveData(SPI0);
    rx_buf[3] = (uint8_t)SPI_ReceiveData(SPI0);

    id_out[0] = rx_buf[1];
    id_out[1] = rx_buf[2];
    id_out[2] = rx_buf[3];

    return true;
}

static uint32_t lp_dlps_flash_jedec_pack_u24(const uint8_t id[3])
{
    return (((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | (uint32_t)id[2]);
}

static bool lp_dlps_flash_enter_dpd_step(void)
{
    uint8_t id_before[3] = {0};
    uint8_t id_after_b9[3] = {0};
    uint32_t id_before_u24 = 0U;
    uint32_t id_after_u24 = 0U;
    bool id_before_ok;
    bool id_after_b9_ok;
    bool cmd_ok;
    bool entered_dpd = false;

    ZY100_DIAG_LOG("[LP_TEST][FLASH] DPD step begin");
    lp_dlps_flash_spi_init();
    ZY100_DIAG_LOG("[LP_TEST][FLASH] SPI0 init done");

    id_before_ok = lp_dlps_flash_spi_read_jedec_id(id_before);
    if (id_before_ok)
    {
        id_before_u24 = lp_dlps_flash_jedec_pack_u24(id_before);
        s_lp_flash_jedec_id_u24 = id_before_u24;
        ZY100_DIAG_LOG("[LP_TEST][FLASH] JEDEC before B9: %02x %02x %02x",
                   id_before[0], id_before[1], id_before[2]);
    }
    else
    {
        DBG_DIRECT("[LP_TEST][FLASH][WARN] SPI read JEDEC before B9 failed");
    }

    cmd_ok = lp_dlps_flash_spi_send_cmd(LP_TEST_GD25Q32E_CMD_B9);
    ZY100_DIAG_LOG("[LP_TEST][FLASH] send B9 (deep power-down): %s", cmd_ok ? "OK" : "FAIL");

    lp_dlps_flash_delay_us(LP_TEST_GD25Q32E_T_DP_US);

    id_after_b9_ok = lp_dlps_flash_spi_read_jedec_id(id_after_b9);
    if (id_after_b9_ok)
    {
        id_after_u24 = lp_dlps_flash_jedec_pack_u24(id_after_b9);
        ZY100_DIAG_LOG("[LP_TEST][FLASH] JEDEC after B9: %02x %02x %02x",
                   id_after_b9[0], id_after_b9[1], id_after_b9[2]);
        if (id_before_ok && (id_after_u24 == id_before_u24))
        {
            DBG_DIRECT("[LP_TEST][FLASH][WARN] JEDEC unchanged after B9, check DPD receive state");
        }
        else
        {
            ZY100_DIAG_LOG("[LP_TEST][FLASH] B9 receive check passed (JEDEC changed/unavailable as expected)");
        }
    }
    else
    {
        DBG_DIRECT("[LP_TEST][FLASH] JEDEC read after B9 failed (treated as DPD expected behavior)");
    }

    /* Strict gate:
     * only continue DLPS flow when flash communication before B9 is valid,
     * B9 command is sent, and post-B9 behavior indicates deep power-down.
     */
    if (id_before_ok && cmd_ok)
    {
        if (!id_after_b9_ok)
        {
            entered_dpd = true;
        }
        else if (id_after_u24 != id_before_u24)
        {
            entered_dpd = true;
        }
    }

    s_lp_flash_dpd_cmd_sent = entered_dpd ? 1U : 0U;
    if (!entered_dpd)
    {
        DBG_DIRECT("[LP_TEST][FLASH][ERR] DPD verify failed, block DLPS follow-up steps");
    }

    return entered_dpd;
}
#endif

#if (LP_TEST_GD25Q32E_DPD_STEP_EN && LP_TEST_GD25Q32E_RELEASE_ON_WAKE_EN)
static void lp_dlps_flash_release_from_dpd(void)
{
    uint8_t id_after_ab[3] = {0};
    bool cmd_ok;
    bool id_ok;

    if (s_lp_flash_dpd_cmd_sent == 0U)
    {
        return;
    }

    lp_dlps_flash_spi_init();
    cmd_ok = lp_dlps_flash_spi_send_cmd(LP_TEST_GD25Q32E_CMD_AB);
    lp_dlps_flash_delay_us(LP_TEST_GD25Q32E_T_RES1_US);
    id_ok = lp_dlps_flash_spi_read_jedec_id(id_after_ab);

    if (cmd_ok && id_ok)
    {
        const uint32_t id_u24 = lp_dlps_flash_jedec_pack_u24(id_after_ab);
        if ((s_lp_flash_jedec_id_u24 != 0U) && (id_u24 != s_lp_flash_jedec_id_u24))
        {
            DBG_DIRECT("[LP_TEST][FLASH][WARN] AB sent but JEDEC mismatch: %02x %02x %02x",
                       id_after_ab[0], id_after_ab[1], id_after_ab[2]);
        }
        else
        {
            ZY100_DIAG_LOG("[LP_TEST][FLASH] AB release OK, JEDEC: %02x %02x %02x",
                       id_after_ab[0], id_after_ab[1], id_after_ab[2]);
        }
    }
    else
    {
        DBG_DIRECT("[LP_TEST][FLASH][WARN] AB release check failed, cmd_ok=%d id_ok=%d", cmd_ok, id_ok);
    }

    s_lp_flash_dpd_cmd_sent = 0U;
}
#endif

#if !LP_TEST_KEEP_LOG_UART
static void lp_dlps_delay_ms(uint32_t delay_ms)
{
    if (delay_ms == 0U)
    {
        return;
    }

    if (platform_delay_ms != NULL)
    {
        platform_delay_ms(delay_ms);
        return;
    }

    if (os_sched_is_start())
    {
        os_delay(delay_ms);
    }
}
#endif

static void lp_dlps_close_log_uart(void)
{
    (void)log_module_bitmap_trace_set(0xFFFFFFFFFFFFFFFFULL, LEVEL_ERROR, false);
    (void)log_module_bitmap_trace_set(0xFFFFFFFFFFFFFFFFULL, LEVEL_WARN, false);
    (void)log_module_bitmap_trace_set(0xFFFFFFFFFFFFFFFFULL, LEVEL_INFO, false);
    (void)log_module_bitmap_trace_set(0xFFFFFFFFFFFFFFFFULL, LEVEL_TRACE, false);
}
void lp_dlps_close_runtime_log_uart(void)
{
    lp_dlps_close_log_uart();
}

#if (LP_TEST_KEEP_LOG_UART && LP_TEST_RUNTIME_LOG_AUTO_CLOSE_ENABLE)
static void lp_dlps_enter_pure_measurement_mode(void)
{
    ZY100_DIAG_LOG("[LP_TEST] diag done: uart log will be closed, switch to pure current mode");
    lp_dlps_close_log_uart();

    /* After log close, force UART related pins to input pull-down for clean measurement. */
    lp_dlps_cfg_input(P0_3, PAD_PULL_DOWN);
    lp_dlps_cfg_input(P3_0, PAD_PULL_DOWN);
    lp_dlps_cfg_input(P3_1, PAD_PULL_DOWN);

#if ZY100_RTC_AB_ALLOW_UART_GDMA_CLK_OFF
    /* Disable log related clocks. */
    RCC_PeriphClockCmd(APBPeriph_GDMA, APBPeriph_GDMA_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_UART2, APBPeriph_UART2_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_UART1, APBPeriph_UART1_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_UART0, APBPeriph_UART0_CLOCK, DISABLE);
#endif
}
#endif

#if (LP_TEST_KEEP_LOG_UART && LP_TEST_RUNTIME_LOG_AUTO_CLOSE_ENABLE)
static void lp_dlps_diag_task(void *p_param)
{
    (void)p_param;
    uint32_t diag_elapsed_ms = 0;
    bool diag_printed = false;

    while (1)
    {
        if (!diag_printed)
        {
            diag_printed = true;
            ZY100_DIAG_LOG("[LP_TEST] diag-once: timer dump begin");
            (void)os_timer_dump();
#if F_BT_DLPS_EN
            DBG_DIRECT("[LP_TEST] diag-once: next_to=%d, enter_cnt=%d, exit_cnt=%d, wake_cnt=%d, remain_us=%d",
                       os_timer_next_timeout_value_get(), s_lp_dlps_enter_cnt, s_lp_dlps_exit_cnt,
                       lps_wakeup_count_get(), last_lps_remain_us_get());
            DBG_DIRECT("[LP_TEST] diag-once: lps=%d/%d err=0x%x",
                       lps_mode_get(), lps_mode_stack_get(), DlpsErrorCode);
#else
            ZY100_DIAG_LOG("[LP_TEST] diag-once: F_BT_DLPS_EN=0");
#endif
            ZY100_DIAG_LOG("[LP_TEST] diag-once: timer dump end");
        }
        os_delay(1000);
        diag_elapsed_ms += 1000;

#if LP_TEST_RUNTIME_LOG_AUTO_CLOSE_ENABLE
        if (diag_elapsed_ms >= LP_TEST_DIAG_WINDOW_MS)
        {
            lp_dlps_enter_pure_measurement_mode();
            os_task_suspend(s_lp_test_task_handle);
        }
#endif
    }
}

static bool lp_dlps_task_init(void)
{
    return os_task_create(&s_lp_test_task_handle, "lp_diag", lp_dlps_diag_task, 0, 256 * 4, 1);
}
#endif

#if LP_TEST_DLPS_IO_CUSTOM_CFG_EN
static void lp_dlps_config_pads(void)
{
    /* M0_0~M0_7, M1_2~M1_7, M2_0~M2_7, M3_0~M3_6, M4_0~M4_3 => input pull-down. */
    for (uint8_t pin = P0_0; pin <= P4_3; pin++)
    {
#if LP_TEST_KEEP_LOG_UART
        if ((pin == P0_3) || (pin == P3_0) || (pin == P3_1))
        {
            continue;
        }
#endif
        if (lp_dlps_is_protected_wake_pin(pin) || lp_dlps_is_wom_owned_pin(pin))
        {
            continue;
        }
        lp_dlps_cfg_input(pin, PAD_PULL_DOWN);
    }

    /* Product board overrides (ZY-A100 hardware context). */
#if (IMU_POWER_CTRL_PIN != IMU_PIN_UNASSIGNED)
    if (lp_dlps_sensor_power_should_hold())
    {
        if (imu_bsp_power_mode() == IMU_BSP_POWER_WEAK_PULL_UP)
            lp_dlps_cfg_input(IMU_POWER_CTRL_PIN, PAD_PULL_UP);
        else
            lp_dlps_cfg_output(IMU_POWER_CTRL_PIN, PAD_PULL_NONE, 1U);
    }
    else
    {
        lp_dlps_cfg_output(IMU_POWER_CTRL_PIN, PAD_PULL_NONE, 0U);
        lp_dlps_notify_sensor_vdd_off();
    }
#endif

    /* SPI shared lines follow the sensor-rail policy. */
    lp_dlps_cfg_output(P4_0, PAD_PULL_NONE, 0U);    /* SPI_CLK */
    lp_dlps_cfg_output(P4_2, PAD_PULL_NONE, 0U);    /* SPI_MOSI */
    lp_dlps_cfg_input(P4_1, lp_dlps_sensor_power_should_hold() ? PAD_PULL_NONE : PAD_PULL_DOWN);
#if LP_TEST_FLASH_IN_SENSOR_DOMAIN
    if (lp_dlps_sensor_power_should_hold())
    {
        lp_dlps_cfg_output(P4_3, PAD_PULL_NONE, 1U); /* FLASH_CS */
    }
    else
    {
        lp_dlps_cfg_input(P4_3, PAD_PULL_DOWN);      /* FLASH_CS */
    }
#else
    lp_dlps_cfg_output(P4_3, PAD_PULL_UP, 1U);
#endif

    /* I2C_SCL / I2C_SDA: input high-Z, no pull. */
    lp_dlps_cfg_input(MAG_I2C_SCL_PIN, PAD_PULL_NONE);
    lp_dlps_cfg_input(MAG_I2C_SDA_PIN, PAD_PULL_NONE);

#if (IMU_SPI_CS_PIN != IMU_PIN_UNASSIGNED)
    if (lp_dlps_sensor_power_should_hold())
    {
        lp_dlps_cfg_output(IMU_SPI_CS_PIN, PAD_PULL_NONE, 1U);
    }
    else
    {
        lp_dlps_cfg_input(IMU_SPI_CS_PIN, PAD_PULL_DOWN);
    }
#endif
#if (IMU_INT_PIN != IMU_PIN_UNASSIGNED)
    lp_dlps_cfg_input(IMU_INT_PIN, PAD_PULL_NONE);
#endif

    /* Wake source rearm is owned by app_task_rearm_dlps_wakeup_sources(). */

    /* CHG / VBAT_ADC: input, no pull. */
    lp_dlps_cfg_input(P0_2, PAD_PULL_NONE);
    bsp_battery_adc_park_low_power();

    /* LED_SDB: low-power policy is low level. */
    lp_dlps_cfg_output(P2_6, PAD_PULL_NONE, 0U);

#if !LP_TEST_KEEP_LOG_UART
    /* LOG_TX / P3_0 / P3_1: UART off, avoid floating. */
    lp_dlps_cfg_input(P0_3, PAD_PULL_DOWN);
    lp_dlps_cfg_input(P3_0, PAD_PULL_DOWN);
    lp_dlps_cfg_input(P3_1, PAD_PULL_DOWN);
#endif

    /* LED_PWR(MICBIAS/P5_0, SDK H_0): keep LED rail disabled in custom DLPS policy. */
    lp_dlps_cfg_input(H_0, PAD_PULL_DOWN);

    /* Keep RESET/TEST_1V2/32K_XI/32K_XO default function path.
     * M5_0~M5_7 are board/package-level names and should be isolated by EVB wiring.
     */
}
#endif

#if LP_TEST_FORCE_CLK_OFF_EN
static void lp_dlps_disable_peripheral_clocks(void)
{
#if (!LP_TEST_KEEP_LOG_UART && ZY100_RTC_AB_ALLOW_UART_GDMA_CLK_OFF)
    RCC_PeriphClockCmd(APBPeriph_GDMA, APBPeriph_GDMA_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_UART2, APBPeriph_UART2_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_UART1, APBPeriph_UART1_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_UART0, APBPeriph_UART0_CLOCK, DISABLE);
#endif
    /* Keep TIMER clock for OS tick/tickless low-power framework. */
    RCC_PeriphClockCmd(APBPeriph_IF8080, APBPeriph_IF8080_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_ADC, APBPeriph_ADC_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_SPI2W, APBPeriph_SPI2W_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_KEYSCAN, APBPeriph_KEYSCAN_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_QDEC, APBPeriph_QDEC_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_I2C1, APBPeriph_I2C1_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_I2C0, APBPeriph_I2C0_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_IR, APBPeriph_IR_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_SPI1, APBPeriph_SPI1_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_SPI0, APBPeriph_SPI0_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_I2S0, APBPeriph_I2S0_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_I2S1, APBPeriph_I2S1_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_CODEC, APBPeriph_CODEC_CLOCK, DISABLE);
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, DISABLE);
}
#endif

#if LP_TEST_PERIPH_CLK_STEP_TEST_EN
static void lp_dlps_disable_peripheral_clocks_stepwise(void)
{
    uint32_t disabled_count = 0U;

#if (LP_TEST_CLK_OFF_UART0_EN && ZY100_RTC_AB_ALLOW_UART_GDMA_CLK_OFF)
    RCC_PeriphClockCmd(APBPeriph_UART0, APBPeriph_UART0_CLOCK, DISABLE);
    disabled_count++;
#endif
#if (LP_TEST_CLK_OFF_UART1_EN && ZY100_RTC_AB_ALLOW_UART_GDMA_CLK_OFF)
    RCC_PeriphClockCmd(APBPeriph_UART1, APBPeriph_UART1_CLOCK, DISABLE);
    disabled_count++;
#endif
#if (LP_TEST_CLK_OFF_UART2_EN && ZY100_RTC_AB_ALLOW_UART_GDMA_CLK_OFF)
    RCC_PeriphClockCmd(APBPeriph_UART2, APBPeriph_UART2_CLOCK, DISABLE);
    disabled_count++;
#endif
#if (LP_TEST_CLK_OFF_GDMA_EN && ZY100_RTC_AB_ALLOW_UART_GDMA_CLK_OFF)
    RCC_PeriphClockCmd(APBPeriph_GDMA, APBPeriph_GDMA_CLOCK, DISABLE);
    disabled_count++;
#endif
#if LP_TEST_CLK_OFF_I2C0_EN
    RCC_PeriphClockCmd(APBPeriph_I2C0, APBPeriph_I2C0_CLOCK, DISABLE);
    disabled_count++;
#endif
#if LP_TEST_CLK_OFF_I2C1_EN
    RCC_PeriphClockCmd(APBPeriph_I2C1, APBPeriph_I2C1_CLOCK, DISABLE);
    disabled_count++;
#endif
#if LP_TEST_CLK_OFF_SPI0_EN
    RCC_PeriphClockCmd(APBPeriph_SPI0, APBPeriph_SPI0_CLOCK, DISABLE);
    disabled_count++;
#endif
#if LP_TEST_CLK_OFF_SPI1_EN
    RCC_PeriphClockCmd(APBPeriph_SPI1, APBPeriph_SPI1_CLOCK, DISABLE);
    disabled_count++;
#endif
#if LP_TEST_CLK_OFF_SPI2W_EN
    RCC_PeriphClockCmd(APBPeriph_SPI2W, APBPeriph_SPI2W_CLOCK, DISABLE);
    disabled_count++;
#endif
#if LP_TEST_CLK_OFF_ADC_EN
    RCC_PeriphClockCmd(APBPeriph_ADC, APBPeriph_ADC_CLOCK, DISABLE);
    disabled_count++;
#endif
#if LP_TEST_CLK_OFF_KEYSCAN_EN
    RCC_PeriphClockCmd(APBPeriph_KEYSCAN, APBPeriph_KEYSCAN_CLOCK, DISABLE);
    disabled_count++;
#endif
#if LP_TEST_CLK_OFF_QDEC_EN
    RCC_PeriphClockCmd(APBPeriph_QDEC, APBPeriph_QDEC_CLOCK, DISABLE);
    disabled_count++;
#endif
#if LP_TEST_CLK_OFF_IR_EN
    RCC_PeriphClockCmd(APBPeriph_IR, APBPeriph_IR_CLOCK, DISABLE);
    disabled_count++;
#endif
#if LP_TEST_CLK_OFF_IF8080_EN
    RCC_PeriphClockCmd(APBPeriph_IF8080, APBPeriph_IF8080_CLOCK, DISABLE);
    disabled_count++;
#endif
#if LP_TEST_CLK_OFF_I2S0_EN
    RCC_PeriphClockCmd(APBPeriph_I2S0, APBPeriph_I2S0_CLOCK, DISABLE);
    disabled_count++;
#endif
#if LP_TEST_CLK_OFF_I2S1_EN
    RCC_PeriphClockCmd(APBPeriph_I2S1, APBPeriph_I2S1_CLOCK, DISABLE);
    disabled_count++;
#endif
#if LP_TEST_CLK_OFF_CODEC_EN
    RCC_PeriphClockCmd(APBPeriph_CODEC, APBPeriph_CODEC_CLOCK, DISABLE);
    disabled_count++;
#endif

    ZY100_DIAG_LOG("[LP_TEST] periph step-clk-off applied=%d (TIMER/GPIO/FLASH untouched)",
               disabled_count);
}
#endif

#if LP_TEST_TIMER_STEP_TEST_EN
static void lp_dlps_apply_timer_step_test(void)
{
#if LP_TEST_TIMER_CLK_OFF_EN
    RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, DISABLE);
    DBG_DIRECT("[LP_TEST][WARN] timer step-test: APB TIMER clock forced OFF");
#else
    ZY100_DIAG_LOG("[LP_TEST] timer step-test: APB TIMER clock kept ON (baseline)");
#endif
}
#endif

#if LP_TEST_TASK_STEP_TEST_EN
bool lp_dlps_apply_task_step_test(void *app_task_handle)
{
#if LP_TEST_TASK_STATUS_DUMP_EN
    os_task_status_dump();
    ZY100_DIAG_LOG("[LP_TEST] task step-test: task status dumped");
#endif

#if LP_TEST_TASK_SUSPEND_APP_EN
    if (app_task_handle == NULL)
    {
        DBG_DIRECT("[LP_TEST][WARN] task step-test: APP task handle is NULL");
        return false;
    }

    if (os_task_suspend(app_task_handle))
    {
        ZY100_DIAG_LOG("[LP_TEST] task step-test: APP task suspended");
        return true;
    }

    DBG_DIRECT("[LP_TEST][WARN] task step-test: APP task suspend failed");
    return false;
#else
    (void)app_task_handle;
    ZY100_DIAG_LOG("[LP_TEST] task step-test: APP suspend disabled (A/B compare mode)");
    return false;
#endif
}
#else
bool lp_dlps_apply_task_step_test(void *app_task_handle)
{
    (void)app_task_handle;
    ZY100_DIAG_LOG("[LP_TEST] task step-test disabled");
    return false;
}
#endif

void lp_dlps_apply_button_sleep_low_power_policy(void)
{
    uint32_t clk_off_count = 0U;
    uint8_t timer_clk_off = 0U;

#if LP_TEST_DLPS_IO_ALL_INPUT_PULLDOWN_EN
    for (uint8_t pin = P0_0; pin <= P4_3; pin++)
    {
        if (lp_dlps_is_protected_wake_pin(pin) || lp_dlps_is_wom_owned_pin(pin))
        {
            continue;
        }
        lp_dlps_cfg_input(pin, PAD_PULL_DOWN);
    }
    /* Keep KEY(P1_0) wake pin untouched; include MICBIAS/P5_0(LED_PWR). */
    lp_dlps_cfg_input(H_0, PAD_PULL_DOWN);
    if (!lp_dlps_is_wom_owned_pin(H_1))
    {
        lp_dlps_cfg_input(H_1, PAD_PULL_DOWN);
    }
    lp_dlps_cfg_input(H_2, PAD_PULL_DOWN);
    if (lp_dlps_sensor_power_should_hold())
    {
#if LP_TEST_DLPS_IO_G4_CFG_EN
        lp_dlps_config_group_g4();
#endif
#if LP_TEST_DLPS_IO_G5_CFG_EN
        lp_dlps_config_group_g5();
#endif
#if LP_TEST_DLPS_IO_G6_CFG_EN
        lp_dlps_config_group_g6();
#endif
    }
#else
#if LP_TEST_DLPS_IO_G1_CFG_EN
#if LP_TEST_DLPS_IO_G1_VBAT_CFG_EN
    lp_dlps_cfg_adc_analog_no_digital(P2_7); /* VBAT_ADC */
#endif
#if LP_TEST_DLPS_IO_G1_CHG_CFG_EN
    lp_dlps_cfg_input(P0_2, PAD_PULL_NONE);  /* CHG */
#endif
#endif
#if LP_TEST_DLPS_IO_G2_CFG_EN
    V0_LP_POLICY_STEP_LOG("[V0_LP_STEP] policy G2 begin");
    lp_dlps_config_group_g2();
    V0_LP_POLICY_STEP_LOG("[V0_LP_STEP] policy G2 done");
#endif
#if LP_TEST_DLPS_IO_G3_CFG_EN
    lp_dlps_config_group_g3();
#endif
#if LP_TEST_DLPS_IO_G5_CFG_EN
    V0_LP_POLICY_STEP_LOG("[V0_LP_STEP] policy G5 begin");
    lp_dlps_config_group_g5();
    V0_LP_POLICY_STEP_LOG("[V0_LP_STEP] policy G5 done");
#endif
#if LP_TEST_DLPS_IO_G4_CFG_EN
    V0_LP_POLICY_STEP_LOG("[V0_LP_STEP] policy G4 begin");
    lp_dlps_config_group_g4();
    V0_LP_POLICY_STEP_LOG("[V0_LP_STEP] policy G4 done");
#endif
#if LP_TEST_DLPS_IO_G6_CFG_EN
    V0_LP_POLICY_STEP_LOG("[V0_LP_STEP] policy G6 begin");
    lp_dlps_config_group_g6();
    V0_LP_POLICY_STEP_LOG("[V0_LP_STEP] policy G6 done");
#endif
#if LP_TEST_DLPS_IO_LED_SDB_CFG_EN
    lp_dlps_config_led_sdb();
#endif
#if LP_TEST_DLPS_IO_LED_PWR_CFG_EN
    lp_dlps_config_led_power();
#endif
#if LP_TEST_DLPS_IO_CUSTOM_CFG_EN
    lp_dlps_config_pads();
#endif
#endif

    lp_dlps_config_chg_int();
#if ZY100_OFFLINE_V2_WOM_START_ENABLE
    lp_sensor_idle_io_apply();
#endif

#if LP_TEST_PERIPH_CLK_STEP_TEST_EN
#if (LP_TEST_CLK_OFF_UART0_EN && ZY100_RTC_AB_ALLOW_UART_GDMA_CLK_OFF)
    RCC_PeriphClockCmd(APBPeriph_UART0, APBPeriph_UART0_CLOCK, DISABLE);
    clk_off_count++;
#endif
#if (LP_TEST_CLK_OFF_UART1_EN && ZY100_RTC_AB_ALLOW_UART_GDMA_CLK_OFF)
    RCC_PeriphClockCmd(APBPeriph_UART1, APBPeriph_UART1_CLOCK, DISABLE);
    clk_off_count++;
#endif
#if (LP_TEST_CLK_OFF_UART2_EN && ZY100_RTC_AB_ALLOW_UART_GDMA_CLK_OFF)
    RCC_PeriphClockCmd(APBPeriph_UART2, APBPeriph_UART2_CLOCK, DISABLE);
    clk_off_count++;
#endif
#if (LP_TEST_CLK_OFF_GDMA_EN && ZY100_RTC_AB_ALLOW_UART_GDMA_CLK_OFF)
    RCC_PeriphClockCmd(APBPeriph_GDMA, APBPeriph_GDMA_CLOCK, DISABLE);
    clk_off_count++;
#endif
#if LP_TEST_CLK_OFF_I2C0_EN
    RCC_PeriphClockCmd(APBPeriph_I2C0, APBPeriph_I2C0_CLOCK, DISABLE);
    clk_off_count++;
#endif
#if LP_TEST_CLK_OFF_I2C1_EN
    RCC_PeriphClockCmd(APBPeriph_I2C1, APBPeriph_I2C1_CLOCK, DISABLE);
    clk_off_count++;
#endif
#if LP_TEST_CLK_OFF_SPI0_EN
    RCC_PeriphClockCmd(APBPeriph_SPI0, APBPeriph_SPI0_CLOCK, DISABLE);
    clk_off_count++;
#endif
#if LP_TEST_CLK_OFF_SPI1_EN
    RCC_PeriphClockCmd(APBPeriph_SPI1, APBPeriph_SPI1_CLOCK, DISABLE);
    clk_off_count++;
#endif
#if LP_TEST_CLK_OFF_SPI2W_EN
    RCC_PeriphClockCmd(APBPeriph_SPI2W, APBPeriph_SPI2W_CLOCK, DISABLE);
    clk_off_count++;
#endif
#if LP_TEST_CLK_OFF_ADC_EN
    RCC_PeriphClockCmd(APBPeriph_ADC, APBPeriph_ADC_CLOCK, DISABLE);
    clk_off_count++;
#endif
#if LP_TEST_CLK_OFF_KEYSCAN_EN
    RCC_PeriphClockCmd(APBPeriph_KEYSCAN, APBPeriph_KEYSCAN_CLOCK, DISABLE);
    clk_off_count++;
#endif
#if LP_TEST_CLK_OFF_QDEC_EN
    RCC_PeriphClockCmd(APBPeriph_QDEC, APBPeriph_QDEC_CLOCK, DISABLE);
    clk_off_count++;
#endif
#if LP_TEST_CLK_OFF_IR_EN
    RCC_PeriphClockCmd(APBPeriph_IR, APBPeriph_IR_CLOCK, DISABLE);
    clk_off_count++;
#endif
#if LP_TEST_CLK_OFF_IF8080_EN
    RCC_PeriphClockCmd(APBPeriph_IF8080, APBPeriph_IF8080_CLOCK, DISABLE);
    clk_off_count++;
#endif
#if LP_TEST_CLK_OFF_I2S0_EN
    RCC_PeriphClockCmd(APBPeriph_I2S0, APBPeriph_I2S0_CLOCK, DISABLE);
    clk_off_count++;
#endif
#if LP_TEST_CLK_OFF_I2S1_EN
    RCC_PeriphClockCmd(APBPeriph_I2S1, APBPeriph_I2S1_CLOCK, DISABLE);
    clk_off_count++;
#endif
#if LP_TEST_CLK_OFF_CODEC_EN
    RCC_PeriphClockCmd(APBPeriph_CODEC, APBPeriph_CODEC_CLOCK, DISABLE);
    clk_off_count++;
#endif
#endif

#if LP_TEST_TIMER_STEP_TEST_EN && LP_TEST_TIMER_CLK_OFF_EN
    RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, DISABLE);
    timer_clk_off = 1U;
#endif

    ZY100_LOG_ROUTINE(DBG_DIRECT, "[LP_SLEEP] policy summary io(g2=%d g3=%d g4=%d g5=%d g6=%d led_sdb=%d sleep_charge_hold=%u) clk_off=%d timer_off=%d",
               LP_TEST_DLPS_IO_G2_CFG_EN ? 1 : 0,
               LP_TEST_DLPS_IO_G3_CFG_EN ? 1 : 0,
               LP_TEST_DLPS_IO_G4_CFG_EN ? 1 : 0,
               LP_TEST_DLPS_IO_G5_CFG_EN ? 1 : 0,
               LP_TEST_DLPS_IO_G6_CFG_EN ? 1 : 0,
               LP_TEST_DLPS_IO_LED_SDB_CFG_EN ? 1 : 0,
               s_lp_sleep_charge_led_hold ? 1U : 0U,
               clk_off_count,
               timer_clk_off);
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[LP_POLICY] KEY wake pin protected");
}


#if F_BT_DLPS_EN
void lp_dlps_note_enter(void)
{
    pwrd_enter();
#if ZY100_OFFLINE_V2_WOM_START_ENABLE
    zy100_rtc_raw_snapshot_t snapshot;
    s_lp_residency_valid = zy100_rtc_clock_raw_snapshot(&snapshot);
    if (s_lp_residency_valid) s_lp_residency_enter_ticks = snapshot.ticks;
#endif
    s_lp_dlps_enter_cnt++;
    s_lp_dlps_enter_seen = 1;
}

void lp_dlps_note_exit(void)
{
#if (LP_TEST_GD25Q32E_DPD_STEP_EN && LP_TEST_GD25Q32E_RELEASE_ON_WAKE_EN)
    lp_dlps_flash_release_from_dpd();
#endif
#if ZY100_OFFLINE_V2_WOM_START_ENABLE
    zy100_rtc_raw_snapshot_t snapshot;
    if (s_lp_residency_valid && zy100_rtc_clock_raw_snapshot(&snapshot))
    {
        s_lp_residency_ticks += (snapshot.ticks >= s_lp_residency_enter_ticks) ?
            snapshot.ticks - s_lp_residency_enter_ticks :
            snapshot.wrap_ticks - s_lp_residency_enter_ticks + snapshot.ticks;
    }
    else s_lp_residency_invalid++;
    s_lp_residency_valid = false;
#endif
    s_lp_dlps_exit_cnt++;
    s_lp_dlps_exit_seen = 1;
}
#endif


void lp_dlps_get_stats(lp_dlps_stats_t *out)
{
    uint32_t key = os_lock();
    out->enters = s_lp_dlps_enter_cnt;
    out->exits = s_lp_dlps_exit_cnt;
#if ZY100_OFFLINE_V2_WOM_START_ENABLE
    out->invalid = s_lp_residency_invalid;
    out->sleep_ms = (uint32_t)(s_lp_residency_ticks * 1000ULL / ZY100_RTC_INPUT_HZ);
#else
    out->invalid = 1U;
    out->sleep_ms = 0U;
#endif
    os_unlock(key);
}

void lp_dlps_get_residency(uint32_t *enters, uint32_t *exits, uint32_t *sleep_ms)
{
    uint32_t key = os_lock();
    if (enters != NULL) *enters = s_lp_dlps_enter_cnt;
    if (exits != NULL) *exits = s_lp_dlps_exit_cnt;
    if (sleep_ms != NULL)
    {
#if ZY100_OFFLINE_V2_WOM_START_ENABLE
        *sleep_ms = (uint32_t)(s_lp_residency_ticks * 1000ULL / ZY100_RTC_INPUT_HZ);
#else
        *sleep_ms = 0U;
#endif
    }
    os_unlock(key);
}

void lp_dlps_power_manager_init(void)
{
#if F_BT_DLPS_EN
    DLPS_IORegUserDlpsEnterCb(lp_dlps_note_enter);
    DLPS_IORegUserDlpsExitCb(lp_dlps_note_exit);
    DLPS_IORegister();
    lps_mode_set(LPM_DLPS_MODE);
#endif
}

bool lp_dlps_enter_packaged_mode(void)
{
    bool scheduler_started = os_sched_is_start();

    if (s_lp_packaged_mode_entered)
    {
        ZY100_DIAG_LOG("[LP_TEST] pure DLPS mode already active");
        return true;
    }

#if LP_TEST_GD25Q32E_DPD_STEP_EN
    /* First step for DLPS flow: put external GD25Q32E into deep power-down by B9h. */
    ZY100_DIAG_LOG("[LP_TEST][FLASH] start DPD verification gate");
    if (!lp_dlps_flash_enter_dpd_step())
    {
        DBG_DIRECT("[LP_TEST][ABORT] flash DPD not confirmed, skip packaged DLPS mode");
        return false;
    }
#else
    ZY100_DIAG_LOG("[LP_TEST][FLASH] SPI DPD flow disabled by switch");
#endif
    s_lp_packaged_mode_entered = 1;

#if LP_TEST_DLPS_IO_ALL_INPUT_PULLDOWN_EN
    lp_dlps_config_all_io_input_pulldown();
    ZY100_DIAG_LOG("[LP_TEST] io cfg special-test: all IO -> SW input pull-down");
#else
#if LP_TEST_DLPS_IO_G1_CFG_EN
    lp_dlps_config_group_g1();
    ZY100_DIAG_LOG("[LP_TEST] g1 io cfg: KEY=0, VBAT=analog(PAD_NOT_PWRON), CHG=1");
#endif

#if LP_TEST_DLPS_IO_G3_CFG_EN
    lp_dlps_config_group_g3();
    ZY100_DIAG_LOG("[LP_TEST] g3 io cfg: I2C_SCL/I2C_SDA -> input no-pull");
#endif

#if LP_TEST_DLPS_IO_G5_CFG_EN
    lp_dlps_config_group_g5();
    ZY100_DIAG_LOG("[LP_TEST] g5 io cfg: imu_wom_armed=%u", s_lp_imu_wom_wake_armed ? 1U : 0U);
#endif

#if LP_TEST_DLPS_IO_G4_CFG_EN
    lp_dlps_config_group_g4();
    ZY100_DIAG_LOG("[LP_TEST] g4 io cfg: spi_wom_armed=%u", s_lp_imu_wom_wake_armed ? 1U : 0U);
#endif

#if LP_TEST_DLPS_IO_G6_CFG_EN
    lp_dlps_config_group_g6();
    ZY100_DIAG_LOG("[LP_TEST] g6 io cfg: sensor_pwr_wom_armed=%u", s_lp_imu_wom_wake_armed ? 1U : 0U);
#endif

#if LP_TEST_DLPS_IO_LED_SDB_CFG_EN
    lp_dlps_config_led_sdb();
    ZY100_DIAG_LOG("[LP_TEST] led_sdb io cfg: P2_6 -> output low");
#endif

#if LP_TEST_DLPS_IO_CUSTOM_CFG_EN
    lp_dlps_config_pads();
    ZY100_DIAG_LOG("[LP_TEST] io cfg: custom low-power parking enabled");
#else
    ZY100_DIAG_LOG("[LP_TEST] io cfg: keep default pin state (no pad reconfiguration)");
#endif
#endif
    lp_dlps_config_chg_int();
    ZY100_DIAG_LOG("[LP_TEST] chg_int io cfg: P0_1 -> input no-pull wake-disabled");
#if LP_TEST_FORCE_CLK_OFF_EN
    lp_dlps_disable_peripheral_clocks();
    ZY100_DIAG_LOG("[LP_TEST] peripheral clock force-off enabled");
#else
    ZY100_DIAG_LOG("[LP_TEST] peripheral clock force-off disabled (diagnostic baseline)");
#endif
#if LP_TEST_PERIPH_CLK_STEP_TEST_EN
    lp_dlps_disable_peripheral_clocks_stepwise();
#endif
#if LP_TEST_TIMER_STEP_TEST_EN
    lp_dlps_apply_timer_step_test();
#endif
    lp_dlps_power_manager_init();

    ZY100_DIAG_LOG("[LP_TEST] pure DLPS mode active");
    if (scheduler_started)
    {
        ZY100_DIAG_LOG("[LP_TEST] runtime switch path: BLE/task already running before pure DLPS mode");
    }
    else
    {
        ZY100_DIAG_LOG("[LP_TEST] check: ble_adv_started=0 (BLE stack path not executed)");
        ZY100_DIAG_LOG("[LP_TEST] check: scheduler will be started for DLPS entry path");
    }
#if F_BT_DLPS_EN
    DBG_DIRECT("[LP_TEST] check: lps_mode=%d, lps_pause_stack=%d", lps_mode_get(), lps_mode_stack_get());
    ZY100_DIAG_LOG("[LP_TEST] check: dlps_enter_seen=%d, dlps_exit_seen=%d, enter_cnt=%d, exit_cnt=%d",
               s_lp_dlps_enter_seen, s_lp_dlps_exit_seen, s_lp_dlps_enter_cnt, s_lp_dlps_exit_cnt);
#else
    ZY100_DIAG_LOG("[LP_TEST] check: F_BT_DLPS_EN=0, DLPS is disabled in build");
#endif
#if (LP_TEST_KEEP_LOG_UART && LP_TEST_RUNTIME_LOG_AUTO_CLOSE_ENABLE)
    ZY100_DIAG_LOG("[LP_TEST] diag log mode: keep UART for %d ms, then auto close", LP_TEST_DIAG_WINDOW_MS);
#elif LP_TEST_KEEP_LOG_UART
    ZY100_BLE_DIAG_LOG("[BLE_DIAG] runtime_log_uart_retained auto_close=0");
#else
    ZY100_DIAG_LOG("[LP_TEST] g2 io cfg: SWD/UART pins will be parked to input pull-down after log close");
    ZY100_DIAG_LOG("[LP_TEST] hold uart log %d ms before close (test mode)", LP_TEST_UART_PRE_CLOSE_HOLD_MS);
    lp_dlps_delay_ms(LP_TEST_UART_PRE_CLOSE_HOLD_MS);
    ZY100_DIAG_LOG("[LP_TEST] log uart will be closed now");
    lp_dlps_close_log_uart();
    lp_dlps_delay_ms(LP_TEST_UART_POST_CLOSE_DRAIN_MS);
#if LP_TEST_DLPS_IO_G2_CFG_EN
    /* Must run after UART log close, so pins are not driven by active log path. */
    lp_dlps_config_group_g2();
#endif
#endif

#if (LP_TEST_KEEP_LOG_UART && LP_TEST_RUNTIME_LOG_AUTO_CLOSE_ENABLE)
    if (lp_dlps_task_init())
    {
        ZY100_DIAG_LOG("[LP_TEST] check: lp_test_task_create=OK");
    }
    else
    {
        DBG_DIRECT("[LP_TEST] check: lp_test_task_create=FAIL");
    }
#endif

    if (scheduler_started)
    {
        ZY100_DIAG_LOG("[LP_TEST] check: scheduler already running, skip os_sched_start");
    }
    else if (os_sched_start())
    {
        DBG_DIRECT("[LP_TEST] check: os_sched_start returned true (unexpected)");
    }
    else
    {
        ZY100_DIAG_LOG("[LP_TEST] check: os_sched_start returned false (scheduler not started)");
    }

    return true;
}

#if ZY100_OFFLINE_V2_WOM_START_ENABLE
extern const uint16_t PINADDR_TABLE[TOTAL_PIN_NUM];
static const uint8_t s_standby_pins[LP_STANDBY_PIN_COUNT] =
{ P1_1, P4_0, P4_2, P4_1, P4_3, H_1, P2_5, P2_4,
  P2_7, P2_6, H_0, P0_3, P3_0, P3_1, H_2, P0_1 };

static uint16_t lp_standby_pad_read(uint8_t pin)
{
    uint16_t addr = PINADDR_TABLE[pin];
    return (uint16_t)btaon_fast_read_safe(addr) |
           ((uint16_t)btaon_fast_read_safe(addr + 1U) << 8);
}

void lp_standby_io_snapshot(lp_standby_io_snapshot_t *out)
{
    uint32_t i;
    for (i = 0U; i < LP_STANDBY_PIN_COUNT; i++)
    {
        uint8_t pin = s_standby_pins[i];
        out->pad[i] = lp_standby_pad_read(pin);
        out->mux[i] = (uint8_t)(PINMUX->CFG[pin >> 2] >> ((pin & 3U) * 8U));
    }
    /* GPIO/system clock registers remain accessible: no SPI/I2C/ADC MMIO. */
    out->direction = GPIO->DATADIR;
    out->input = GPIO->DATAIN;
    out->output = GPIO->DATAOUT;
    out->clock0 = SYSBLKCTRL->u_234.PERI_CLK_CTRL0;
    out->clock1 = SYSBLKCTRL->u_238.PERI_CLK_CTRL1;
}

static bool lp_standby_pad_matches(uint8_t pin, PAD_PWR_Mode power,
                                    PAD_Pull_Mode pull, bool output, bool high)
{
    uint16_t raw = lp_standby_pad_read(pin);
    uint16_t mask = Output_En | Pull_En | Pull_Direction | (SHDN << 8) | (Pin_Mode << 8);
    uint16_t value = (PAD_SW_MODE << 9) | ((uint16_t)power << 8);
    if (pull != PAD_PULL_NONE) value |= Pull_En;
    if (pull == PAD_PULL_DOWN) value |= Pull_Direction;
    if (output)
    {
        mask |= Output_Val;
        value |= Output_En;
        if (high) value |= Output_Val;
    }
    return (raw & mask) == value;
}

bool lp_sensor_idle_io_matches(bool gpio_wom)
{
    uint32_t i;
    if ((SYSBLKCTRL->u_234.PERI_CLK_CTRL0 & (SYSBLK_ACTCK_SPI0_EN_Msk | SYSBLK_ACTCK_ADC_EN_Msk)) ||
        (SYSBLKCTRL->u_238.PERI_CLK_CTRL1 & SYSBLK_ACTCK_I2C1_EN_Msk)) return false;
    if (!lp_dlps_sensor_power_should_hold())
    {
        if (imu_bsp_power_mode() != IMU_BSP_POWER_DRIVE_LOW ||
            !lp_standby_pad_matches(P1_1, PAD_IS_PWRON, PAD_PULL_NONE, true, false) ||
            (lp_standby_pad_read(H_2) & WakeUp_En) ||
            !lp_standby_pad_matches(IMU_INT_PIN, PAD_IS_PWRON, PAD_PULL_NONE, false, false)) return false;
        for (i = 1U; i <= 5U; i++)
            if (!lp_standby_pad_matches(s_standby_pins[i], PAD_IS_PWRON,
                                        PAD_PULL_DOWN, false, false)) return false;
    }
    else
    {
        for (i = 1U; i <= 5U; i++)
        {
            bool output = (i == 1U) || (i == 2U) || (i == 4U) || (i == 5U);
            if (!lp_standby_pad_matches(s_standby_pins[i], PAD_IS_PWRON,
                                        PAD_PULL_NONE, output, (i == 4U) || (i == 5U))) return false;
        }
        if (imu_bsp_power_mode() == IMU_BSP_POWER_WEAK_PULL_UP)
        {
            if (!lp_standby_pad_matches(P1_1, PAD_IS_PWRON, PAD_PULL_UP, false, false) ||
                (lp_standby_pad_read(P1_1) & Pull_Resistance)) return false;
        }
        /* BSP restore uses high + pull-up; DLPS parking uses high + no pull.
         * Both already drive the rail high. Do not rewrite either policy. */
        else if (imu_bsp_power_mode() != IMU_BSP_POWER_DRIVE_HIGH ||
                 (!lp_standby_pad_matches(P1_1, PAD_IS_PWRON, PAD_PULL_NONE, true, true) &&
                  !lp_standby_pad_matches(P1_1, PAD_IS_PWRON, PAD_PULL_UP, true, true))) return false;
    }
    for (i = 6U; i <= 7U; i++)
        if (!lp_standby_pad_matches(s_standby_pins[i], PAD_IS_PWRON,
                                    PAD_PULL_NONE, false, false)) return false;
    if (!lp_standby_pad_matches(P2_7, PAD_NOT_PWRON, PAD_PULL_NONE, false, false)) return false;
    /* Charge uses GPIO; Full uses PAD only when WOM is armed. Check both
     * directions so an old PAD wake cannot survive a policy switch. */
    if (((lp_standby_pad_read(H_2) & WakeUp_En) != 0U) !=
        (!gpio_wom && s_lp_imu_wom_wake_armed)) return false;
    return true;
}

bool lp_standby_io_matches(bool charge)
{
    uint32_t i;
    if (!lp_sensor_idle_io_matches(charge)) return false;
    if (!charge)
    {
        /* LOG_TX remains owned by the live trace transport in this build. */
        for (i = (V0_IMU_FIFO_DRAIN_TEST && V0_DLPS_DEBUG_KEEP_LOG_PIN) ? 12U : 11U; i <= 13U; i++)
            if (!lp_standby_pad_matches(s_standby_pins[i], PAD_IS_PWRON, PAD_PULL_DOWN, false, false)) return false;
        if (bsp_led_power_is_on() || !lp_standby_pad_matches(P2_6, PAD_IS_PWRON, PAD_PULL_NONE, true, false) ||
            !lp_standby_pad_matches(H_0, PAD_IS_PWRON, PAD_PULL_DOWN, false, false)) return false;
    }
    return true;
}

bool lp_standby_spi_suspend(void)
{
    /* Only the always-on clock gate may be read after SPI0 is asleep. */
    if (!(SYSBLKCTRL->u_234.PERI_CLK_CTRL0 & SYSBLK_ACTCK_SPI0_EN_Msk)) return true;
    if (SPI_GetFlagState(IMU_SPI_PORT, SPI_FLAG_BUSY) == SET) return false;
    SPI_Cmd(IMU_SPI_PORT, DISABLE);
    RCC_PeriphClockCmd(IMU_SPI_CLOCK_ID, IMU_SPI_CLOCK_MASK, DISABLE);
    return true;
}

void lp_sensor_idle_io_apply(void)
{
    /* Use the existing rail-off parking levels; no product configuration here. */
    if (lp_dlps_sensor_power_should_hold())
    {
        lp_dlps_cfg_output(P4_0, PAD_PULL_NONE, 0U);
        lp_dlps_cfg_output(P4_2, PAD_PULL_NONE, 0U);
        lp_dlps_cfg_input(P4_1, PAD_PULL_NONE);
        lp_dlps_cfg_output(P4_3, PAD_PULL_NONE, 1U);
        lp_dlps_cfg_output(IMU_SPI_CS_PIN, PAD_PULL_NONE, 1U);
    }
    else
    {
        lp_dlps_cfg_input(P4_0, PAD_PULL_DOWN);
        lp_dlps_cfg_input(P4_2, PAD_PULL_DOWN);
        lp_dlps_cfg_input(P4_1, PAD_PULL_DOWN);
        lp_dlps_cfg_input(P4_3, PAD_PULL_DOWN);
        lp_dlps_cfg_input(IMU_SPI_CS_PIN, PAD_PULL_DOWN);
        lp_dlps_cfg_output(P1_1, PAD_PULL_NONE, 0U);
        /* IRQ disable alone leaves the previous GPIO pull/mux behind. */
        lp_dlps_cfg_input(IMU_INT_PIN, PAD_PULL_NONE);
    }
    lp_dlps_config_group_g3();
    bsp_battery_adc_park_low_power();
    imu_bsp_sleep_gpio_park(lp_dlps_sensor_power_should_hold());
}

void lp_standby_io_apply(bool charge)
{
    lp_sensor_idle_io_apply();
    if (!charge)
    {
#if !(V0_IMU_FIFO_DRAIN_TEST && V0_DLPS_DEBUG_KEEP_LOG_PIN)
        lp_dlps_cfg_input(P0_3, PAD_PULL_DOWN);
#endif
        lp_dlps_cfg_input(P3_0, PAD_PULL_DOWN);
        lp_dlps_cfg_input(P3_1, PAD_PULL_DOWN);
        lp_dlps_cfg_output(P2_6, PAD_PULL_NONE, 0U);
        lp_dlps_cfg_input(ZY100_LED_POWER_CTRL_PIN, PAD_PULL_DOWN);
    }
}

void lp_standby_io_log(const lp_standby_io_snapshot_t *before,
                       const lp_standby_io_snapshot_t *after)
{
    uint32_t i;
    for (i = 0U; i < LP_STANDBY_PIN_COUNT; i++)
        ZY100_LOG_DETAIL("[STBY_PAD] pin=%u pad=%04x/%04x mux=%02x/%02x",
                       s_standby_pins[i], before->pad[i], after->pad[i],
                       before->mux[i], after->mux[i]);
    ZY100_LOG_DETAIL("[STBY_GPIO] dir=%08lx/%08lx in=%08lx/%08lx out=%08lx/%08lx",
                   (unsigned long)before->direction, (unsigned long)after->direction,
                   (unsigned long)before->input, (unsigned long)after->input,
                   (unsigned long)before->output, (unsigned long)after->output);
    ZY100_LOG_DETAIL("[STBY_CLK] c0=%08lx/%08lx c1=%08lx/%08lx",
                   (unsigned long)before->clock0, (unsigned long)after->clock0,
                   (unsigned long)before->clock1, (unsigned long)after->clock1);
}
#endif
