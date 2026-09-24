#include "dfu_power_guard.h"
#include "../sample/ble_peripheral/app_flags.h"

#include <stddef.h>

#include "rtl876x.h"
#include "rtl876x_gpio.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"
#include "trace.h"
#include "../sample/ble_peripheral/bsp/bsp_power_status.h"

#define DFU_STACMD_PIN P0_2
#define DFU_UART_TX_PIN P3_0
#define DFU_UART_RX_PIN P3_1

/* Exported by the linked SDK rtl876x_pinmux.c; do not duplicate AON addresses. */
extern const uint16_t PINADDR_TABLE[TOTAL_PIN_NUM];

void dfu_power_guard_trace_pin(const char *stage)
{
    const uint32_t mask = GPIO_GetPin(DFU_STACMD_PIN);
    const uint16_t addr = PINADDR_TABLE[DFU_STACMD_PIN];
    const uint8_t pad0 = btaon_fast_read_safe(addr);
    const uint8_t pad1 = btaon_fast_read_safe((uint16_t)(addr + 1U));
    const uint8_t mux = (uint8_t)(PINMUX->CFG[DFU_STACMD_PIN >> 2] >>
                                  ((DFU_STACMD_PIN & 3U) * 8U));
    const uint32_t dir = GPIO->DATADIR;
    const uint32_t out = GPIO->DATAOUT;
    const uint32_t src = GPIO->DATASRC;
    const uint32_t in = GPIO->DATAIN;
    const uint32_t clk0 = SYSBLKCTRL->u_234.PERI_CLK_CTRL0;
    const uint32_t clk1 = SYSBLKCTRL->u_238.PERI_CLK_CTRL1;

    /* Log only after all reads. DATAOUT=0 alone does not mean output-low:
     * PAD mode/output enable, mux, direction and source must also be decoded.
     * Do not enable a clock or touch the pad merely to collect this snapshot. */
    DBG_DIRECT("[DFU_PIN] stage=%s pin=P0_2 pad0=0x%02x pad1=0x%02x mux=0x%02x",
               stage, pad0, pad1, mux);
    DBG_DIRECT("[DFU_PIN] dir=%u out=%u src=%u in=%u key=%u clk0=0x%08x clk1=0x%08x",
               (dir & mask) ? 1U : 0U, (out & mask) ? 1U : 0U,
               (src & mask) ? 1U : 0U, (in & mask) ? 1U : 0U,
               (in & GPIO_GetPin(P1_0)) ? 1U : 0U, clk0, clk1);
}

static bool dfu_power_guard_stacmd_pin_valid(uint32_t gpio_pin)
{
    return (gpio_pin != 0U) && (gpio_pin != 0xFFFFFFFFUL);
}

static T_DFU_POWER_GUARD_RESULT dfu_power_guard_config_input(uint8_t pin, uint8_t *level)
{
    GPIO_InitTypeDef gpio_init;
    const uint32_t gpio_pin = GPIO_GetPin(pin);

    if ((level == NULL) || !dfu_power_guard_stacmd_pin_valid(gpio_pin))
    {
        return DFU_POWER_GUARD_INVALID_STACMD_PIN;
    }

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);

    /*
     * V1.2 connects P0_2 to YHM2712A STACMD through the CHG_KEY network.
     * Release any inherited output drive before touching PAD or pinmux state.
     * Preloading DATAOUT high prevents a later direction change from creating a
     * low pulse, while the pin remains input-only for the whole DFU session.
     */
    GPIO->DATADIR &= ~gpio_pin;
    GPIO->DATAOUT |= gpio_pin;
    GPIO->DATASRC &= ~gpio_pin;

    Pad_Config(pin, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP,
               PAD_OUT_DISABLE, PAD_OUT_HIGH);
    Pinmux_Deinit(pin);
    Pinmux_Config(pin, DWGPIO);

    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = gpio_pin;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_ITCmd = DISABLE;
    gpio_init.GPIO_ControlMode = GPIO_SOFTWARE_MODE;
    GPIO_Init(&gpio_init);

    GPIO->DATADIR &= ~gpio_pin;
    *level = GPIO_ReadInputDataBit(gpio_pin) ? 1U : 0U;
    return DFU_POWER_GUARD_OK;
}

T_DFU_POWER_GUARD_RESULT dfu_power_guard_release_before_reset(void)
{
    uint8_t level = 0U;
    T_DFU_POWER_GUARD_RESULT result;

    dfu_power_guard_trace_pin("pre_reset_raw");
    result = dfu_power_guard_config_input(DFU_STACMD_PIN, &level);
    dfu_power_guard_trace_pin("pre_reset_released");
    DBG_DIRECT("[DFU_PIN] release_result=%s level=%u boot_rom_coverage=0",
               dfu_power_guard_result_name(result), level);
    return result;
}

static void dfu_power_guard_trace_uart_pin(const char *stage, const char *name, uint8_t pin)
{
    const uint32_t mask = GPIO_GetPin(pin);
    const uint16_t addr = PINADDR_TABLE[pin];
    const uint8_t pad0 = btaon_fast_read_safe(addr);
    const uint8_t pad1 = btaon_fast_read_safe((uint16_t)(addr + 1U));
    const uint8_t mux = (uint8_t)(PINMUX->CFG[pin >> 2] >> ((pin & 3U) * 8U));
    const uint32_t dir = GPIO->DATADIR;
    const uint32_t out = GPIO->DATAOUT;
    const uint32_t src = GPIO->DATASRC;
    const uint32_t in = GPIO->DATAIN;

    ZY100_LOG_ROUTINE(DBG_DIRECT, "[DFU_UART] stage=%s pin=%s pad0=0x%02x pad1=0x%02x mux=0x%02x",
               stage, name, pad0, pad1, mux);
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[DFU_UART] pin=%s dir=%u out=%u src=%u in=%u",
               name, (dir & mask) ? 1U : 0U, (out & mask) ? 1U : 0U,
               (src & mask) ? 1U : 0U, (in & mask) ? 1U : 0U);
}

T_DFU_POWER_GUARD_RESULT dfu_power_guard_release_uart_before_reset(void)
{
    uint8_t tx_level = 0U;
    uint8_t rx_level = 0U;
    T_DFU_POWER_GUARD_RESULT tx_result;
    T_DFU_POWER_GUARD_RESULT rx_result;

    /* V1.2 TX/RX are P3_0/P3_1. ROM LOG_UART remains on the separate P0_3. */
    dfu_power_guard_trace_uart_pin("pre_reset_raw", "TX/P3_0", DFU_UART_TX_PIN);
    dfu_power_guard_trace_uart_pin("pre_reset_raw", "RX/P3_1", DFU_UART_RX_PIN);
    tx_result = dfu_power_guard_config_input(DFU_UART_TX_PIN, &tx_level);
    rx_result = dfu_power_guard_config_input(DFU_UART_RX_PIN, &rx_level);
    dfu_power_guard_trace_uart_pin("pre_reset_released", "TX/P3_0", DFU_UART_TX_PIN);
    dfu_power_guard_trace_uart_pin("pre_reset_released", "RX/P3_1", DFU_UART_RX_PIN);
    DBG_DIRECT("[DFU_UART] tx_result=%s rx_result=%s tx_in=%u rx_in=%u boot_rom_coverage=0",
               dfu_power_guard_result_name(tx_result), dfu_power_guard_result_name(rx_result),
               tx_level, rx_level);
    return (tx_result != DFU_POWER_GUARD_OK) ? tx_result : rx_result;
}

T_DFU_POWER_GUARD_RESULT dfu_power_guard_init(T_DFU_POWER_GUARD_SNAPSHOT *snapshot)
{
    T_DFU_POWER_GUARD_RESULT result;
    bsp_power_status_t power_status;
    bool external_power_present = false;
    uint8_t stacmd_level = 0U;

    result = dfu_power_guard_config_input(DFU_STACMD_PIN, &stacmd_level);
    if (result == DFU_POWER_GUARD_OK)
    {
        power_status = bsp_power_status_chg_int_external_power_present(
                           &external_power_present);
        if (power_status != BSP_POWER_STATUS_OK)
        {
            result = DFU_POWER_GUARD_EXTERNAL_POWER_READ_FAILED;
        }
        else if ((!external_power_present) && (stacmd_level == 0U))
        {
            result = DFU_POWER_GUARD_STACMD_LOW_ON_BATTERY;
        }
    }

    if (snapshot != NULL)
    {
        snapshot->result = result;
        snapshot->external_power_present = external_power_present;
        snapshot->stacmd_level = stacmd_level;
    }
    return result;
}

const char *dfu_power_guard_result_name(T_DFU_POWER_GUARD_RESULT result)
{
    switch (result)
    {
    case DFU_POWER_GUARD_OK:
        return "ok";
    case DFU_POWER_GUARD_INVALID_STACMD_PIN:
        return "invalid_stacmd_pin";
    case DFU_POWER_GUARD_EXTERNAL_POWER_READ_FAILED:
        return "external_power_read_failed";
    case DFU_POWER_GUARD_STACMD_LOW_ON_BATTERY:
        return "stacmd_low_on_battery";
    default:
        return "unknown";
    }
}
