#include "bsp_power_status.h"

#include <stddef.h>

#include "rtl876x_gpio.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"

static bool s_bsp_power_status_inited = false;

static bool bsp_power_status_pin_valid(uint8_t pin)
{
    const uint32_t gpio_pin = GPIO_GetPin(pin);

    return (gpio_pin != 0U) && (gpio_pin != 0xFFFFFFFFUL);
}

static bsp_power_status_t bsp_power_status_config_chg_int_gpio_input(void)
{
    GPIO_InitTypeDef gpio_init;
    const uint32_t gpio_pin = GPIO_GetPin(ZY100_CHG_INT_PIN);

    if (!bsp_power_status_pin_valid(ZY100_CHG_INT_PIN))
    {
        return BSP_POWER_STATUS_INVALID_PIN;
    }

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    System_WakeUpPinDisable(ZY100_CHG_INT_PIN);
    Pad_Config(ZY100_CHG_INT_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE,
               PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pinmux_Config(ZY100_CHG_INT_PIN, DWGPIO);

    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = gpio_pin;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_ITCmd = DISABLE;
    gpio_init.GPIO_ControlMode = GPIO_SOFTWARE_MODE;
    GPIO_Init(&gpio_init);

    return BSP_POWER_STATUS_OK;
}

bsp_power_status_t bsp_power_status_init(void)
{
    bsp_power_status_t status = bsp_power_status_config_chg_int_gpio_input();

    if (status == BSP_POWER_STATUS_OK)
    {
        s_bsp_power_status_inited = true;
    }

    return status;
}

bsp_power_status_t bsp_power_status_chg_int_level(uint8_t *level)
{
    bsp_power_status_t status;

    if (level == NULL)
    {
        return BSP_POWER_STATUS_INVALID_PARAM;
    }

    status = bsp_power_status_config_chg_int_gpio_input();
    if (status != BSP_POWER_STATUS_OK)
    {
        return status;
    }

    s_bsp_power_status_inited = true;
    *level = GPIO_ReadInputDataBit(GPIO_GetPin(ZY100_CHG_INT_PIN)) ? 1U : 0U;
    return BSP_POWER_STATUS_OK;
}

bsp_power_status_t bsp_power_status_chg_int_external_power_present(bool *present)
{
    bsp_power_status_t status;
    uint8_t level = 0U;

    if (present == NULL)
    {
        return BSP_POWER_STATUS_INVALID_PARAM;
    }

    status = bsp_power_status_chg_int_level(&level);
    if (status != BSP_POWER_STATUS_OK)
    {
        return status;
    }

    *present = (level == (uint8_t)(ZY100_CHG_INT_EXTERNAL_PRESENT_LEVEL != 0U));
    return BSP_POWER_STATUS_OK;
}

bsp_power_status_t bsp_power_status_chg_int_rearm_wakeup(bool *present,
                                                         uint8_t *wake_polarity)
{
    bsp_power_status_t status;
    uint8_t level = 0U;
    uint8_t polarity;

    status = bsp_power_status_chg_int_level(&level);
    if (status != BSP_POWER_STATUS_OK)
    {
        return status;
    }

    polarity = (level != 0U) ? PAD_WAKEUP_POL_LOW : PAD_WAKEUP_POL_HIGH;
    Pinmux_Deinit(ZY100_CHG_INT_PIN);
    Pad_Config(ZY100_CHG_INT_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE,
               PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_ClearWakeupINTPendingBit(ZY100_CHG_INT_PIN);
    System_WakeUpPinEnable(ZY100_CHG_INT_PIN, polarity, PAD_WK_DEBOUNCE_DISABLE);
    Pad_ClearWakeupINTPendingBit(ZY100_CHG_INT_PIN);

    if (present != NULL)
    {
        *present = (level == (uint8_t)(ZY100_CHG_INT_EXTERNAL_PRESENT_LEVEL != 0U));
    }
    if (wake_polarity != NULL)
    {
        *wake_polarity = polarity;
    }

    return BSP_POWER_STATUS_OK;
}

bool bsp_power_status_chg_int_wakeup_pending(void)
{
    if (!s_bsp_power_status_inited &&
        !bsp_power_status_pin_valid(ZY100_CHG_INT_PIN))
    {
        return false;
    }

    return System_WakeUpInterruptValue(ZY100_CHG_INT_PIN) != 0U;
}

void bsp_power_status_chg_int_clear_wakeup(void)
{
    Pad_ClearWakeupINTPendingBit(ZY100_CHG_INT_PIN);
}

const char *bsp_power_status_name(bsp_power_status_t status)
{
    switch (status)
    {
    case BSP_POWER_STATUS_OK:
        return "ok";
    case BSP_POWER_STATUS_INVALID_PARAM:
        return "invalid_param";
    case BSP_POWER_STATUS_INVALID_PIN:
        return "invalid_pin";
    default:
        return "unknown";
    }
}
