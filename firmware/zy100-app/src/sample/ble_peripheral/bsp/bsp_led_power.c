#include "bsp_led_power.h"

#include "rtl876x_gpio.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"

#include "led_board_pinmap.h"

extern const uint16_t PINADDR_TABLE[TOTAL_PIN_NUM];

#ifndef F_LED_EDGE_HW_TEST_DATA_PULL_UP_ENABLE
#define F_LED_EDGE_HW_TEST_DATA_PULL_UP_ENABLE 0
#endif

static bool s_bsp_led_power_inited = false;
static bool s_bsp_led_power_on = false;
static bool s_bsp_led_power_pull_up = false;

static bool bsp_led_power_is_h_pad(uint8_t pin)
{
    return (pin == H_0) || (pin == H_1) || (pin == H_2);
}

static void bsp_led_power_config_gpio_output(uint8_t pin, bool level_high)
{
    GPIO_InitTypeDef gpio_init;
    const uint32_t gpio_pin = GPIO_GetPin(pin);

    const PAD_Pull_Mode pull_mode = level_high ? PAD_PULL_UP : PAD_PULL_NONE;

    Pad_Config(pin, PAD_PINMUX_MODE, PAD_IS_PWRON, pull_mode, PAD_OUT_ENABLE,
               level_high ? PAD_OUT_HIGH : PAD_OUT_LOW);
    Pinmux_Deinit(pin);
    System_WakeUpPinDisable(pin);
    Pinmux_Config(pin, DWGPIO);

    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = gpio_pin;
    gpio_init.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init.GPIO_ITCmd = DISABLE;
    gpio_init.GPIO_ControlMode = GPIO_SOFTWARE_MODE;
    GPIO_Init(&gpio_init);

    if (level_high)
    {
        GPIO_SetBits(gpio_pin);
    }
    else
    {
        GPIO_ResetBits(gpio_pin);
    }

    if (pin == ZY100_LED_POWER_CTRL_PIN)
    {
        s_bsp_led_power_pull_up = (pull_mode == PAD_PULL_UP);
    }
}

static void bsp_led_power_config_pad_sw_output(uint8_t pin, bool level_high)
{
    const PAD_Pull_Mode pull_mode = level_high ? PAD_PULL_UP : PAD_PULL_NONE;

    System_WakeUpPinDisable(pin);
    Pinmux_Deinit(pin);

    Pad_Config(pin, PAD_SW_MODE, PAD_IS_PWRON, pull_mode, PAD_OUT_ENABLE,
               level_high ? PAD_OUT_HIGH : PAD_OUT_LOW);

    if (pin == ZY100_LED_POWER_CTRL_PIN)
    {
        s_bsp_led_power_pull_up = (pull_mode == PAD_PULL_UP);
    }
}

static void bsp_led_power_config_pad_sw_output_pull(uint8_t pin,
                                                    bool level_high,
                                                    PAD_Pull_Mode pull_mode)
{
    System_WakeUpPinDisable(pin);
    Pinmux_Deinit(pin);
    Pad_Config(pin, PAD_SW_MODE, PAD_IS_PWRON, pull_mode, PAD_OUT_ENABLE,
               level_high ? PAD_OUT_HIGH : PAD_OUT_LOW);
}

static void bsp_led_power_config_pad_sw_high_z(uint8_t pin)
{
    System_WakeUpPinDisable(pin);
    Pinmux_Deinit(pin);
    Pad_Config(pin, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
               PAD_OUT_LOW);
}

static bool bsp_led_power_pin_valid(uint8_t pin)
{
    const uint32_t gpio_pin = GPIO_GetPin(pin);

    return (gpio_pin != 0U) && (gpio_pin != 0xFFFFFFFFUL);
}

static void bsp_led_power_config_output(uint8_t pin, bool level_high)
{
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);

    bsp_led_power_config_gpio_output(pin, level_high);
}

static bool bsp_led_power_data_mcu_level_for_led_level(bool led_high)
{
    return (ZY100_RGB_LED_DATA_ACTIVE_LOW != 0U) ? !led_high : led_high;
}

static PAD_Pull_Mode bsp_led_power_data_pad_pull(void)
{
#if F_LED_EDGE_HW_TEST_DATA_PULL_UP_ENABLE
    return PAD_PULL_UP;
#else
    return PAD_PULL_NONE;
#endif
}

void bsp_led_power_data_low(void)
{
    if (!bsp_led_power_pin_valid(ZY100_RGB_LED_DATA_PIN))
    {
        return;
    }

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    bsp_led_power_config_pad_sw_output_pull(
        ZY100_RGB_LED_DATA_PIN,
        bsp_led_power_data_mcu_level_for_led_level(false),
        bsp_led_power_data_pad_pull());
}

static void bsp_led_power_data_high_z(void)
{
    if (!bsp_led_power_pin_valid(ZY100_RGB_LED_DATA_PIN))
    {
        return;
    }

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    bsp_led_power_config_pad_sw_high_z(ZY100_RGB_LED_DATA_PIN);
}

bsp_led_power_status_t bsp_led_power_debug_drive_ctrl(bool level_high,
                                                      bsp_led_power_ctrl_mode_t mode)
{
    if (!bsp_led_power_pin_valid(ZY100_LED_POWER_CTRL_PIN))
    {
        return BSP_LED_POWER_STATUS_INVALID_PIN;
    }

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    if (mode == BSP_LED_POWER_CTRL_MODE_PAD_SW)
    {
        bsp_led_power_config_pad_sw_output(ZY100_LED_POWER_CTRL_PIN, level_high);
    }
    else
    {
        bsp_led_power_config_gpio_output(ZY100_LED_POWER_CTRL_PIN, level_high);
    }

    return BSP_LED_POWER_STATUS_OK;
}

bsp_led_power_status_t bsp_led_power_init(void)
{
    if (!bsp_led_power_pin_valid(ZY100_LED_POWER_CTRL_PIN))
    {
        return BSP_LED_POWER_STATUS_INVALID_PIN;
    }

    bsp_led_power_data_low();
    bsp_led_power_config_output(ZY100_LED_POWER_CTRL_PIN,
                                (ZY100_LED_POWER_CTRL_ACTIVE_HIGH == 0U));
    bsp_led_power_data_high_z();
    s_bsp_led_power_on = false;
    s_bsp_led_power_pull_up = false;
    s_bsp_led_power_inited = true;
    return BSP_LED_POWER_STATUS_OK;
}

bsp_led_power_status_t bsp_led_power_on(void)
{
    if (!s_bsp_led_power_inited)
    {
        bsp_led_power_status_t status = bsp_led_power_init();
        if (status != BSP_LED_POWER_STATUS_OK)
        {
            return status;
        }
    }

    bsp_led_power_data_low();
    bsp_led_power_config_output(ZY100_LED_POWER_CTRL_PIN,
                                (ZY100_LED_POWER_CTRL_ACTIVE_HIGH != 0U));
    s_bsp_led_power_on = true;
    return BSP_LED_POWER_STATUS_OK;
}

bsp_led_power_status_t bsp_led_power_off(void)
{
    if (!s_bsp_led_power_inited)
    {
        bsp_led_power_status_t status = bsp_led_power_init();
        if (status != BSP_LED_POWER_STATUS_OK)
        {
            return status;
        }
    }

    bsp_led_power_data_low();
    bsp_led_power_config_output(ZY100_LED_POWER_CTRL_PIN,
                                (ZY100_LED_POWER_CTRL_ACTIVE_HIGH == 0U));
    bsp_led_power_data_high_z();
    s_bsp_led_power_on = false;
    return BSP_LED_POWER_STATUS_OK;
}

bool bsp_led_power_is_on(void)
{
    return s_bsp_led_power_on;
}

uint8_t bsp_led_power_ctrl_pull_up_enabled(void)
{
    return s_bsp_led_power_pull_up ? 1U : 0U;
}

static uint8_t bsp_led_power_read_out(uint8_t pin)
{
    uint32_t gpio_pin;

    if (!bsp_led_power_pin_valid(pin))
    {
        return 0xFFU;
    }

    gpio_pin = GPIO_GetPin(pin);
    return GPIO_ReadOutputDataBit(gpio_pin);
}

static uint8_t bsp_led_power_read_in(uint8_t pin)
{
    uint32_t gpio_pin;

    if (!bsp_led_power_pin_valid(pin))
    {
        return 0xFFU;
    }

    gpio_pin = GPIO_GetPin(pin);
    return GPIO_ReadInputDataBit(gpio_pin);
}

static uint8_t bsp_led_power_read_pad_reg0(uint8_t pin)
{
    if (pin >= TOTAL_PIN_NUM)
    {
        return 0xFFU;
    }

    return btaon_fast_read_safe(PINADDR_TABLE[pin]);
}

static uint8_t bsp_led_power_read_pad_reg1(uint8_t pin)
{
    if (pin >= TOTAL_PIN_NUM)
    {
        return 0xFFU;
    }

    return btaon_fast_read_safe((uint16_t)(PINADDR_TABLE[pin] + 1U));
}

uint8_t bsp_led_power_ctrl_out_level(void)
{
    if (bsp_led_power_is_h_pad(ZY100_LED_POWER_CTRL_PIN))
    {
        if (bsp_led_power_ctrl_pad_mode_level() == (uint8_t)PAD_SW_MODE)
        {
            return bsp_led_power_ctrl_pad_out_level();
        }
    }

    return bsp_led_power_read_out(ZY100_LED_POWER_CTRL_PIN);
}

uint8_t bsp_led_power_ctrl_in_level(void)
{
    return bsp_led_power_read_in(ZY100_LED_POWER_CTRL_PIN);
}

uint8_t bsp_led_power_ctrl_pad_out_level(void)
{
    uint8_t reg = bsp_led_power_read_pad_reg0(ZY100_LED_POWER_CTRL_PIN);

    if (reg == 0xFFU)
    {
        return 0xFFU;
    }

    return (reg & Output_Val) ? 1U : 0U;
}

uint8_t bsp_led_power_ctrl_pad_oe_level(void)
{
    uint8_t reg = bsp_led_power_read_pad_reg0(ZY100_LED_POWER_CTRL_PIN);

    if (reg == 0xFFU)
    {
        return 0xFFU;
    }

    return (reg & Output_En) ? 1U : 0U;
}

uint8_t bsp_led_power_ctrl_pad_mode_level(void)
{
    uint8_t reg = bsp_led_power_read_pad_reg1(ZY100_LED_POWER_CTRL_PIN);

    if (reg == 0xFFU)
    {
        return 0xFFU;
    }

    return (reg & Pin_Mode) ? (uint8_t)PAD_PINMUX_MODE : (uint8_t)PAD_SW_MODE;
}

uint8_t bsp_led_power_ctrl_pad_pwr_level(void)
{
    uint8_t reg = bsp_led_power_read_pad_reg1(ZY100_LED_POWER_CTRL_PIN);

    if (reg == 0xFFU)
    {
        return 0xFFU;
    }

    return (reg & SHDN) ? 1U : 0U;
}

uint8_t bsp_led_power_data_out_level(void)
{
    return bsp_led_power_read_out(ZY100_RGB_LED_DATA_PIN);
}

const char *bsp_led_power_status_name(bsp_led_power_status_t status)
{
    switch (status)
    {
    case BSP_LED_POWER_STATUS_OK:
        return "ok";
    case BSP_LED_POWER_STATUS_INVALID_PIN:
        return "invalid_pin";
    default:
        return "unknown";
    }
}
