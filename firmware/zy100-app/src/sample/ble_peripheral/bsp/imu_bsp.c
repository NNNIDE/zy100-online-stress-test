#include "imu_bsp.h"

#include <stddef.h>

#include "app_section.h"
#include "imu_board_pinmap.h"

#include "FreeRTOS_API.h"
#include "os_sched.h"
#include "os_sync.h"
#include "os_task.h"
#include "bsp_shared_spi.h"
#include "platform_utils.h"
#include "rtl876x_gdma.h"
#include "rtl876x_gpio.h"
#include "rtl876x_nvic.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"
#include "rtl876x_spi.h"
#include "rtl876x_tim.h"
#include "trace.h"
#include "../app_flags.h"

static volatile imu_bsp_power_mode_t s_sensor_power_mode = IMU_BSP_POWER_DRIVE_LOW;
#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE
static uint16_t s_sensor_power_users;
static uint32_t s_sensor_power_generation;
static bool s_sensor_power_parking;
static void *s_sensor_power_restore_task;
static bool s_sensor_power_restoring;
static imu_bsp_power_restore_cb_t s_sensor_power_restore_cb;
#endif

#define IMU_BSP_SPI_TIMEOUT_LOOP 200000U
#define IMU_VENDOR_TICK_MASK     0x03FFFFFFU
#define IMU_VENDOR_TICK_PER_US           ZY100_PLATFORM_VENDOR_TICK_PER_US
#define IMU_FIXED_TIMER_CLOCK_PER_US     ZY100_FIXED_TIMER_CLOCK_PER_US
#define IMU_BSP_SPI_FIXED_SOURCE_HZ 40000000U
/* Software pacing window for long full-duplex reads; not a FIFO-depth claim. */
#define IMU_SPI_QUIET_READ_WINDOW_BYTES 16U

#ifndef IMU_BSP_CS_TRACE_ENABLE
#define IMU_BSP_CS_TRACE_ENABLE 1
#endif

#ifndef IMU_BSP_SPI_TRACE_ENABLE
#define IMU_BSP_SPI_TRACE_ENABLE 1
#endif

#ifndef IMU_BSP_DELAY_PATH_LOG_ENABLE
#define IMU_BSP_DELAY_PATH_LOG_ENABLE ZY100_LOG_PREP_VERBOSE
#endif

#define IMU_BSP_CS_TRACE_MAX_LOG 80U
#define IMU_BSP_SPI_TRACE_MAX_LOG 160U

#ifndef IMU_OIS_TICK_TIMER_NUM
#define IMU_OIS_TICK_TIMER_NUM TIM7
#endif

#ifndef IMU_OIS_TICK_TIMER_IRQn
#define IMU_OIS_TICK_TIMER_IRQn TIMER7_IRQn
#endif

#ifndef IMU_OIS_TICK_TIMER_ISR
#define IMU_OIS_TICK_TIMER_ISR Timer7_Handler
#endif

#ifndef IMU_OIS_TICK_TIMER_NVIC_PRIORITY
#define IMU_OIS_TICK_TIMER_NVIC_PRIORITY 3U
#endif

#if (IMU_OIS_TICK_TIMER_NVIC_PRIORITY < configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)
#error "TIM7 IRQ priority is too high for FreeRTOS FromISR APIs"
#endif

#ifndef IMU_OIS_TICK_TIMER_CLK_PER_US
#define IMU_OIS_TICK_TIMER_CLK_PER_US IMU_FIXED_TIMER_CLOCK_PER_US
#endif

#if IMU_OIS_SPI_DMA_ENABLE
/*
 * Keep SPI0 TX/RX handshakes fixed.
 * Reserve 2/3 for rtl876x_hw_aes and avoid channel 5 (startup marks it as
 * default-for-log), so IMU uses channel 0/1 for minimal conflict risk.
 */
#define IMU_SPI_DMA_TX_CHANNEL_NUM      0U
#define IMU_SPI_DMA_RX_CHANNEL_NUM      1U
#define IMU_SPI_DMA_TX_CHANNEL          GDMA_Channel0
#define IMU_SPI_DMA_RX_CHANNEL          GDMA_Channel1
#define IMU_SPI_DMA_TX_IRQN             GDMA0_Channel0_IRQn
#define IMU_SPI_DMA_RX_IRQN             GDMA0_Channel1_IRQn
#define IMU_SPI_DMA_TX_HANDLER          GDMA0_Channel0_Handler
#define IMU_SPI_DMA_RX_HANDLER          GDMA0_Channel1_Handler
#define IMU_SPI_DMA_MAX_TRANSFER_BYTES  32U
#define IMU_SPI_DMA_NOTIFY_TX_DONE      0x10000000UL
#define IMU_SPI_DMA_NOTIFY_RX_DONE      0x20000000UL
#define IMU_SPI_DMA_NOTIFY_ERR          0x40000000UL
#define IMU_SPI_DMA_NOTIFY_MASK         (IMU_SPI_DMA_NOTIFY_TX_DONE | \
                                         IMU_SPI_DMA_NOTIFY_RX_DONE | \
                                         IMU_SPI_DMA_NOTIFY_ERR)
#define IMU_SPI_DMA_HW_POLL_LIMIT       IMU_BSP_SPI_TIMEOUT_LOOP
#endif

static bool s_imu_bsp_inited = false;
static uint32_t s_imu_cs_gpio_mask = 0U;
static uint32_t s_imu_flash_cs_gpio_mask = 0U;
static uint32_t s_imu_int_gpio_mask = 0U;
static imu_bsp_int_irq_cb_t s_imu_int_irq_cb = NULL;
#if (IMU_POWER_CTRL_PIN != IMU_PIN_UNASSIGNED)
static uint32_t s_imu_power_gpio_mask = 0U;
#endif
static uint16_t s_spi_cpol = IMU_SPI_CPOL;
static uint16_t s_spi_cpha = IMU_SPI_CPHA;
#if IMU_BSP_DELAY_PATH_LOG_ENABLE
static bool s_imu_bsp_delay_path_logged = false;
static uint8_t s_imu_bsp_delay_500_log_count = 0U;
#endif

static bool s_tick_inited = false;
static uint32_t s_tick_last = 0U;
static uint64_t s_tick_wrap_base = 0U;
static uint32_t s_imu_spi_trace_cnt = 0U;
static bool s_imu_ois_tick_timer_cfg = false;
static bool s_imu_ois_tick_timer_running = false;
static volatile imu_bsp_ois_tick_timer_owner_t s_imu_ois_tick_timer_owner =
    IMU_BSP_OIS_TICK_TIMER_OWNER_NONE;
static volatile uint32_t s_imu_ois_tick_timer_irq_count = 0U;
static volatile uint32_t s_imu_ois_tick_timer_last_tick = 0U;
static imu_bsp_timer_irq_cb_t s_imu_ois_tick_timer_irq_cb = NULL;
static bool s_imu_bsp_wake_log_valid = false;
static uint32_t s_imu_bsp_wake_log_last_ms = 0U;
static uint8_t s_imu_bsp_wake_log_pwr = 0U;
static uint8_t s_imu_bsp_wake_log_spi = 0U;
static uint8_t s_imu_bsp_wake_log_pins = 0U;
static uint8_t s_imu_bsp_wake_log_cs = 0U;
#if IMU_OIS_SPI_DMA_ENABLE
static bool s_imu_spi_dma_inited = false;
static uint8_t s_imu_spi_dma_tx_buf[IMU_SPI_DMA_MAX_TRANSFER_BYTES];
static volatile uint8_t s_imu_spi_dma_tx_done = 0U;
static volatile uint8_t s_imu_spi_dma_rx_done = 0U;
static volatile uint8_t s_imu_spi_dma_error = 0U;
static volatile uint32_t s_imu_spi_dma_timeout_total = 0U;
static volatile uint32_t s_imu_spi_dma_error_total = 0U;
static volatile TaskHandle_t s_imu_spi_dma_wait_task = NULL;
#endif

static bool imu_bsp_pin_to_gpio_mask(uint8_t pin, uint32_t *gpio_mask)
{
    if (gpio_mask == NULL)
    {
        return false;
    }

    if (pin <= P3_6)
    {
        *gpio_mask = BIT(pin);
        return true;
    }

    if ((pin >= P4_0) && (pin <= P4_3))
    {
        *gpio_mask = BIT(pin - 4U);
        return true;
    }

    if ((pin == H_0) || (pin == H_1) || (pin == H_2))
    {
        *gpio_mask = BIT(pin - 11U);
        return true;
    }

    return false;
}

static bool imu_bsp_pin_to_gpio_num(uint8_t pin, uint8_t *gpio_num)
{
    if (gpio_num == NULL)
    {
        return false;
    }

    if (pin <= P3_6)
    {
        *gpio_num = pin;
        return true;
    }

    if ((pin >= P4_0) && (pin <= P4_3))
    {
        *gpio_num = (uint8_t)(pin - 4U);
        return true;
    }

    if ((pin == H_0) || (pin == H_1) || (pin == H_2))
    {
        *gpio_num = (uint8_t)(pin - 11U);
        return true;
    }

    return false;
}

static imu_status_t imu_bsp_read_gpio_levels(uint32_t gpio_mask, uint8_t *out_level, uint8_t *in_level)
{
    if ((gpio_mask == 0U) || (out_level == NULL) || (in_level == NULL))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    *out_level = GPIO_ReadOutputDataBit(gpio_mask);
    *in_level = GPIO_ReadInputDataBit(gpio_mask);
    return IMU_STATUS_OK;
}

static void imu_bsp_log_wake_if_due(uint8_t pwr, uint8_t spi, uint8_t pins, uint8_t cs)
{
    uint32_t now_ms = ((uint32_t)imu_bsp_local_timestamp_us()) / 1000U;

    if (s_imu_bsp_wake_log_valid &&
        (s_imu_bsp_wake_log_pwr == pwr) &&
        (s_imu_bsp_wake_log_spi == spi) &&
        (s_imu_bsp_wake_log_pins == pins) &&
        (s_imu_bsp_wake_log_cs == cs) &&
        ((uint32_t)(now_ms - s_imu_bsp_wake_log_last_ms) < 1000U))
    {
        return;
    }

    s_imu_bsp_wake_log_valid = true;
    s_imu_bsp_wake_log_last_ms = now_ms;
    s_imu_bsp_wake_log_pwr = pwr;
    s_imu_bsp_wake_log_spi = spi;
    s_imu_bsp_wake_log_pins = pins;
    s_imu_bsp_wake_log_cs = cs;
    ZY100_LOG_VERBOSE("[IMU_WAKE] pwr=%u spi=%u pins=%u cs=%u", pwr, spi, pins, cs);
}

static uint8_t imu_bsp_get_pinmux_func(uint8_t pin)
{
    const uint8_t pinmux_reg_num = (uint8_t)(pin >> 2);
    const uint8_t reg_offset = (uint8_t)((pin & 0x03U) << 3);
    return (uint8_t)((PINMUX->CFG[pinmux_reg_num] >> reg_offset) & 0xFFU);
}

static void imu_bsp_log_cs_config_state(const char *tag)
{
#if IMU_BSP_CS_TRACE_ENABLE
    uint8_t out_level = 0U;
    uint8_t in_level = 0U;
    uint8_t gpio_num = 0xFFU;
    const uint8_t pinmux_func = imu_bsp_get_pinmux_func(IMU_SPI_CS_PIN);
    const uint8_t dir_out = ((GPIO->DATADIR & s_imu_cs_gpio_mask) != 0U) ? 1U : 0U;
    const uint8_t ctrl_hw = ((GPIO->DATASRC & s_imu_cs_gpio_mask) != 0U) ? 1U : 0U;
    const char *use_tag = (tag != NULL) ? tag : "n/a";

    IMU_UNUSED(imu_bsp_pin_to_gpio_num(IMU_SPI_CS_PIN, &gpio_num));

    if (imu_bsp_read_gpio_levels(s_imu_cs_gpio_mask, &out_level, &in_level) == IMU_STATUS_OK)
    {
        IMU_LOG_INFO("CS cfg(%s): pin=%u gpio=%u mask=0x%08x pinmux_func=%u pad_mode=pinmux dir=%u ctrl=%s drive=pushpull out/in=%u/%u",
                     use_tag,
                     IMU_SPI_CS_PIN,
                     gpio_num,
                     (unsigned int)s_imu_cs_gpio_mask,
                     pinmux_func,
                     dir_out,
                     ctrl_hw ? "hw" : "sw",
                     out_level,
                     in_level);
    }
    else
    {
        IMU_LOG_INFO("CS cfg(%s): pin=%u gpio=%u mask=0x%08x pinmux_func=%u pad_mode=pinmux dir=%u ctrl=%s drive=pushpull",
                     use_tag,
                     IMU_SPI_CS_PIN,
                     gpio_num,
                     (unsigned int)s_imu_cs_gpio_mask,
                     pinmux_func,
                     dir_out,
                     ctrl_hw ? "hw" : "sw");
    }
#else
    IMU_UNUSED(tag);
#endif
}

DATA_RAM_FUNCTION
static void imu_bsp_cs_apply_level(bool drive_high)
{
    if (s_imu_cs_gpio_mask == 0U)
    {
        return;
    }

    if (drive_high)
    {
        GPIO_SetBits(s_imu_cs_gpio_mask);
    }
    else
    {
        GPIO_ResetBits(s_imu_cs_gpio_mask);
    }
}

static imu_status_t imu_bsp_apply_output_level(uint8_t pin,
                                               uint32_t *gpio_mask,
                                               bool drive_high)
{
    GPIO_InitTypeDef gpio_init;

    if (gpio_mask == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    if (!imu_bsp_pin_to_gpio_mask(pin, gpio_mask))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    Pad_Config(pin,
               PAD_SW_MODE,
               PAD_IS_PWRON,
               drive_high ? PAD_PULL_UP : PAD_PULL_NONE,
               PAD_OUT_ENABLE,
               drive_high ? PAD_OUT_HIGH : PAD_OUT_LOW);
    Pinmux_Deinit(pin);
    Pinmux_Config(pin, DWGPIO);

    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = *gpio_mask;
    gpio_init.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init.GPIO_ITCmd = DISABLE;
    gpio_init.GPIO_ControlMode = GPIO_SOFTWARE_MODE;
    if (drive_high)
    {
        GPIO_SetBits(*gpio_mask);
    }
    else
    {
        GPIO_ResetBits(*gpio_mask);
    }
    GPIO_Init(&gpio_init);
    return IMU_STATUS_OK;
}

DATA_RAM_FUNCTION
static imu_status_t imu_bsp_wait_spi_flag(uint8_t flag, FlagStatus target)
{
    uint32_t guard = IMU_BSP_SPI_TIMEOUT_LOOP;

    while (SPI_GetFlagState(IMU_SPI_PORT, flag) != target)
    {
        if (guard-- == 0U)
        {
            return IMU_STATUS_TIMEOUT;
        }
    }

    return IMU_STATUS_OK;
}

static uint16_t imu_bsp_flush_spi_rx(void)
{
    uint16_t drained = 0U;
    uint8_t sample0 = 0U;
    uint8_t sample1 = 0U;
    uint8_t sample2 = 0U;

    while (SPI_GetFlagState(IMU_SPI_PORT, SPI_FLAG_RFNE) == SET)
    {
        const uint8_t value = (uint8_t)SPI_ReceiveData(IMU_SPI_PORT);
        if (drained == 0U)
        {
            sample0 = value;
        }
        else if (drained == 1U)
        {
            sample1 = value;
        }
        else if (drained == 2U)
        {
            sample2 = value;
        }
        drained++;
    }

#if IMU_BSP_SPI_TRACE_ENABLE
    if ((drained != 0U) && (s_imu_spi_trace_cnt < IMU_BSP_SPI_TRACE_MAX_LOG))
    {
        IMU_LOG_WARN("SPI RX flush drained=%u first=0x%02x second=0x%02x third=0x%02x",
                     (unsigned int)drained, sample0, sample1, sample2);
        s_imu_spi_trace_cnt++;
    }
#else
    IMU_UNUSED(sample0);
    IMU_UNUSED(sample1);
    IMU_UNUSED(sample2);
#endif

    return drained;
}

DATA_RAM_FUNCTION
static void imu_bsp_flush_spi_rx_quiet(void)
{
    while (SPI_GetFlagState(IMU_SPI_PORT, SPI_FLAG_RFNE) == SET)
    {
        (void)SPI_ReceiveData(IMU_SPI_PORT);
    }
}

#if IMU_OIS_SPI_DMA_ENABLE
static void imu_bsp_spi_dma_notify_from_isr(uint32_t notify_bits)
{
    if ((notify_bits != 0U) && (s_imu_spi_dma_wait_task != NULL))
    {
        (void)xTaskNotifyFromISR((TaskHandle_t)s_imu_spi_dma_wait_task,
                                 notify_bits,
                                 eSetBits,
                                 NULL);
    }
}

static void imu_bsp_spi_dma_handle_irq(uint8_t channel_num,
                                       volatile uint8_t *done_flag,
                                       uint32_t done_notify_bit)
{
    uint32_t notify_bits = 0U;
    uint32_t channel_bit = BIT(channel_num);

    if ((GDMA_BASE->STATUS_TFR & channel_bit) != 0U)
    {
        GDMA_ClearINTPendingBit(channel_num, GDMA_INT_Transfer);
        *done_flag = 1U;
        notify_bits |= done_notify_bit;
    }

    if ((GDMA_BASE->STATUS_ERR & channel_bit) != 0U)
    {
        GDMA_ClearINTPendingBit(channel_num, GDMA_INT_Error);
        s_imu_spi_dma_error = 1U;
        notify_bits |= IMU_SPI_DMA_NOTIFY_ERR;
    }

    imu_bsp_spi_dma_notify_from_isr(notify_bits);
}

static void imu_bsp_spi_dma_config_irq(void)
{
    NVIC_InitTypeDef nvic_init;

    nvic_init.NVIC_IRQChannel = IMU_SPI_DMA_TX_IRQN;
    nvic_init.NVIC_IRQChannelPriority = IMU_INT_NVIC_PRIORITY;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    nvic_init.NVIC_IRQChannel = IMU_SPI_DMA_RX_IRQN;
    nvic_init.NVIC_IRQChannelPriority = IMU_INT_NVIC_PRIORITY;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
}

static void imu_bsp_spi_dma_config_channels(void)
{
    GDMA_InitTypeDef dma_init;

    GDMA_StructInit(&dma_init);
    dma_init.GDMA_ChannelNum = IMU_SPI_DMA_TX_CHANNEL_NUM;
    dma_init.GDMA_DIR = GDMA_DIR_MemoryToPeripheral;
    dma_init.GDMA_BufferSize = 1U;
    dma_init.GDMA_SourceInc = DMA_SourceInc_Fix;
    dma_init.GDMA_DestinationInc = DMA_DestinationInc_Fix;
    dma_init.GDMA_SourceDataSize = GDMA_DataSize_Byte;
    dma_init.GDMA_DestinationDataSize = GDMA_DataSize_Byte;
    dma_init.GDMA_SourceMsize = GDMA_Msize_1;
    dma_init.GDMA_DestinationMsize = GDMA_Msize_1;
    dma_init.GDMA_SourceAddr = (uint32_t)s_imu_spi_dma_tx_buf;
    dma_init.GDMA_DestinationAddr = (uint32_t)(&(IMU_SPI_PORT->DR[0]));
    dma_init.GDMA_DestHandshake = GDMA_Handshake_SPI0_TX;
    dma_init.GDMA_ChannelPriority = 2U;
    GDMA_Init(IMU_SPI_DMA_TX_CHANNEL, &dma_init);

    GDMA_StructInit(&dma_init);
    dma_init.GDMA_ChannelNum = IMU_SPI_DMA_RX_CHANNEL_NUM;
    dma_init.GDMA_DIR = GDMA_DIR_PeripheralToMemory;
    dma_init.GDMA_BufferSize = 1U;
    dma_init.GDMA_SourceInc = DMA_SourceInc_Fix;
    dma_init.GDMA_DestinationInc = DMA_DestinationInc_Inc;
    dma_init.GDMA_SourceDataSize = GDMA_DataSize_Byte;
    dma_init.GDMA_DestinationDataSize = GDMA_DataSize_Byte;
    dma_init.GDMA_SourceMsize = GDMA_Msize_1;
    dma_init.GDMA_DestinationMsize = GDMA_Msize_1;
    dma_init.GDMA_SourceAddr = (uint32_t)(&(IMU_SPI_PORT->DR[0]));
    dma_init.GDMA_DestinationAddr = 0U;
    dma_init.GDMA_SourceHandshake = GDMA_Handshake_SPI0_RX;
    dma_init.GDMA_ChannelPriority = 2U;
    GDMA_Init(IMU_SPI_DMA_RX_CHANNEL, &dma_init);

    GDMA_INTConfig(IMU_SPI_DMA_TX_CHANNEL_NUM, GDMA_INT_Transfer | GDMA_INT_Error, ENABLE);
    GDMA_INTConfig(IMU_SPI_DMA_RX_CHANNEL_NUM, GDMA_INT_Transfer | GDMA_INT_Error, ENABLE);
}

static void imu_bsp_spi_dma_init_once(void)
{
    uint32_t i;

    if (s_imu_spi_dma_inited)
    {
        return;
    }

    RCC_PeriphClockCmd(APBPeriph_GDMA, APBPeriph_GDMA_CLOCK, ENABLE);

    for (i = 0U; i < IMU_SPI_DMA_MAX_TRANSFER_BYTES; i++)
    {
        s_imu_spi_dma_tx_buf[i] = 0xFFU;
    }

    imu_bsp_spi_dma_config_channels();
    imu_bsp_spi_dma_config_irq();
    s_imu_spi_dma_inited = true;
}

static void imu_bsp_spi_dma_stop_transfer(void)
{
    SPI_GDMACmd(IMU_SPI_PORT, SPI_GDMAReq_Tx | SPI_GDMAReq_Rx, DISABLE);
    GDMA_Cmd(IMU_SPI_DMA_TX_CHANNEL_NUM, DISABLE);
    GDMA_Cmd(IMU_SPI_DMA_RX_CHANNEL_NUM, DISABLE);
    GDMA_ClearAllTypeINT(IMU_SPI_DMA_TX_CHANNEL_NUM);
    GDMA_ClearAllTypeINT(IMU_SPI_DMA_RX_CHANNEL_NUM);
}

static void imu_bsp_spi_dma_update_status_from_hw(void)
{
    const uint32_t tx_bit = BIT(IMU_SPI_DMA_TX_CHANNEL_NUM);
    const uint32_t rx_bit = BIT(IMU_SPI_DMA_RX_CHANNEL_NUM);

    if (s_imu_spi_dma_tx_done == 0U)
    {
        if ((GDMA_BASE->STATUS_TFR & tx_bit) != 0U)
        {
            GDMA_ClearINTPendingBit(IMU_SPI_DMA_TX_CHANNEL_NUM, GDMA_INT_Transfer);
            s_imu_spi_dma_tx_done = 1U;
        }
        else if ((GDMA_GetChannelStatus(IMU_SPI_DMA_TX_CHANNEL_NUM) == RESET) &&
                 (GDMA_GetTransferLen(IMU_SPI_DMA_TX_CHANNEL) == 0U))
        {
            /* Fallback completion check when ISR notify is delayed/lost. */
            s_imu_spi_dma_tx_done = 1U;
        }
    }

    if (s_imu_spi_dma_rx_done == 0U)
    {
        if ((GDMA_BASE->STATUS_TFR & rx_bit) != 0U)
        {
            GDMA_ClearINTPendingBit(IMU_SPI_DMA_RX_CHANNEL_NUM, GDMA_INT_Transfer);
            s_imu_spi_dma_rx_done = 1U;
        }
        else if ((GDMA_GetChannelStatus(IMU_SPI_DMA_RX_CHANNEL_NUM) == RESET) &&
                 (GDMA_GetTransferLen(IMU_SPI_DMA_RX_CHANNEL) == 0U))
        {
            /* Fallback completion check when ISR notify is delayed/lost. */
            s_imu_spi_dma_rx_done = 1U;
        }
    }

    if (s_imu_spi_dma_error == 0U)
    {
        if ((GDMA_BASE->STATUS_ERR & tx_bit) != 0U)
        {
            GDMA_ClearINTPendingBit(IMU_SPI_DMA_TX_CHANNEL_NUM, GDMA_INT_Error);
            s_imu_spi_dma_error = 1U;
        }

        if ((GDMA_BASE->STATUS_ERR & rx_bit) != 0U)
        {
            GDMA_ClearINTPendingBit(IMU_SPI_DMA_RX_CHANNEL_NUM, GDMA_INT_Error);
            s_imu_spi_dma_error = 1U;
        }
    }
}

static imu_status_t imu_bsp_spi_dma_finalize(imu_status_t status_in)
{
    imu_status_t status = status_in;
    imu_status_t idle_status;
    uint32_t guard = IMU_SPI_DMA_HW_POLL_LIMIT;

    while (guard-- != 0U)
    {
        imu_bsp_spi_dma_update_status_from_hw();
        if (s_imu_spi_dma_error != 0U)
        {
            status = IMU_STATUS_BUS_ERROR;
            if (s_imu_spi_dma_error_total < 0xFFFFFFFFU)
            {
                s_imu_spi_dma_error_total++;
            }
            break;
        }
        if ((s_imu_spi_dma_tx_done != 0U) && (s_imu_spi_dma_rx_done != 0U))
        {
            break;
        }
    }

    s_imu_spi_dma_wait_task = NULL;
    imu_bsp_spi_dma_stop_transfer();
    idle_status = imu_bsp_wait_spi_flag(SPI_FLAG_BUSY, RESET);
    (void)imu_bsp_flush_spi_rx();

    if ((status == IMU_STATUS_OK) && (idle_status != IMU_STATUS_OK))
    {
        status = idle_status;
    }
    return status;
}
#endif

static void imu_bsp_config_spi_pins(void)
{
    Pad_Config(IMU_SPI_CLK_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE, PAD_OUT_HIGH);
    Pad_Config(IMU_SPI_MOSI_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE, PAD_OUT_HIGH);
    Pad_Config(IMU_SPI_MISO_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);

    Pinmux_Deinit(IMU_SPI_CLK_PIN);
    Pinmux_Deinit(IMU_SPI_MOSI_PIN);
    Pinmux_Deinit(IMU_SPI_MISO_PIN);

    Pinmux_Config(IMU_SPI_CLK_PIN, SPI0_CLK_MASTER);
    Pinmux_Config(IMU_SPI_MOSI_PIN, SPI0_MO_MASTER);
    Pinmux_Config(IMU_SPI_MISO_PIN, SPI0_MI_MASTER);
}

/* PAD owns the inactive level until GPIO has a valid high latch/direction.
 * GPIO registers are not retained by USE_GPIO_DLPS in Production. */
static imu_status_t imu_bsp_config_inactive_cs(uint8_t pin, uint32_t *mask)
{
    GPIO_InitTypeDef gpio_init;
    if (!imu_bsp_pin_to_gpio_mask(pin, mask)) return IMU_STATUS_INVALID_PARAM;
    Pad_Config(pin, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE, PAD_OUT_HIGH);
    GPIO_SetBits(*mask);
    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = *mask;
    gpio_init.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init.GPIO_ITCmd = DISABLE;
    gpio_init.GPIO_ControlMode = GPIO_SOFTWARE_MODE;
    GPIO_Init(&gpio_init);
    Pinmux_Deinit(pin);
    Pinmux_Config(pin, DWGPIO);
    Pad_Config(pin, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE, PAD_OUT_HIGH);
    return IMU_STATUS_OK;
}

static imu_status_t imu_bsp_config_cs_pin(void)
{
    imu_status_t status = imu_bsp_config_inactive_cs(IMU_SPI_CS_PIN, &s_imu_cs_gpio_mask);
    if (status != IMU_STATUS_OK) return status;
    System_WakeUpPinDisable(IMU_SPI_CS_PIN);
    imu_bsp_log_cs_config_state("init");
    return IMU_STATUS_OK;
}

imu_status_t imu_bsp_flash_cs_hold_high(void)
{
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    System_WakeUpPinDisable(IMU_FLASH_CS_PIN);
    return imu_bsp_config_inactive_cs(IMU_FLASH_CS_PIN, &s_imu_flash_cs_gpio_mask);
}

imu_status_t imu_bsp_flash_cs_high(void)
{
    imu_status_t status;

    if (s_imu_flash_cs_gpio_mask == 0U)
    {
        status = imu_bsp_flash_cs_hold_high();
        if (status != IMU_STATUS_OK)
        {
            return status;
        }
    }

    GPIO_SetBits(s_imu_flash_cs_gpio_mask);
    return IMU_STATUS_OK;
}

imu_status_t imu_bsp_flash_cs_low(void)
{
    imu_status_t status;

    if (s_imu_flash_cs_gpio_mask == 0U)
    {
        status = imu_bsp_flash_cs_hold_high();
        if (status != IMU_STATUS_OK)
        {
            return status;
        }
    }

    GPIO_ResetBits(s_imu_flash_cs_gpio_mask);
    return IMU_STATUS_OK;
}

imu_status_t imu_bsp_get_flash_cs_level(uint8_t *out_level, uint8_t *in_level)
{
    if ((out_level == NULL) || (in_level == NULL))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    if (s_imu_flash_cs_gpio_mask == 0U)
    {
        if (!imu_bsp_pin_to_gpio_mask(IMU_FLASH_CS_PIN, &s_imu_flash_cs_gpio_mask))
        {
            return IMU_STATUS_INVALID_PARAM;
        }
    }

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    return imu_bsp_read_gpio_levels(s_imu_flash_cs_gpio_mask, out_level, in_level);
}

void imu_bsp_get_spi_nominal_config(uint32_t *source_hz,
                                    uint32_t *clk_div,
                                    uint32_t *baud_prescaler,
                                    uint32_t *sclk_hz)
{
    const uint32_t source = IMU_BSP_SPI_FIXED_SOURCE_HZ;
    const uint32_t div = 1U;
    const uint32_t baud = (uint32_t)IMU_SPI_BAUD_PRESCALER;
    const uint32_t sclk = (baud != 0U) ? (source / div / baud) : 0U;

    if (source_hz != NULL)
    {
        *source_hz = source;
    }
    if (clk_div != NULL)
    {
        *clk_div = div;
    }
    if (baud_prescaler != NULL)
    {
        *baud_prescaler = baud;
    }
    if (sclk_hz != NULL)
    {
        *sclk_hz = sclk;
    }
}

static void imu_bsp_config_spi_controller(void)
{
    SPI_InitTypeDef spi_init;

    SPI_StructInit(&spi_init);
    spi_init.SPI_Direction = SPI_Direction_FullDuplex;
    spi_init.SPI_Mode = SPI_Mode_Master;
    spi_init.SPI_DataSize = SPI_DataSize_8b;
    spi_init.SPI_CPOL = s_spi_cpol;
    spi_init.SPI_CPHA = s_spi_cpha;
    spi_init.SPI_BaudRatePrescaler = IMU_SPI_BAUD_PRESCALER;
    spi_init.SPI_FrameFormat = SPI_Frame_Motorola;
    spi_init.SPI_TxThresholdLevel = 1U;
    spi_init.SPI_RxThresholdLevel = 0U;
    spi_init.SPI_NDF = 0U;
    SPI_Init(IMU_SPI_PORT, &spi_init);
    SPI_Cmd(IMU_SPI_PORT, ENABLE);
}

imu_status_t imu_bsp_spi_set_mode(uint16_t cpol, uint16_t cpha)
{
    if ((cpol != SPI_CPOL_Low) && (cpol != SPI_CPOL_High))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    if ((cpha != SPI_CPHA_1Edge) && (cpha != SPI_CPHA_2Edge))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    s_spi_cpol = cpol;
    s_spi_cpha = cpha;

    if (!s_imu_bsp_inited)
    {
        return imu_bsp_init();
    }

    SPI_Cmd(IMU_SPI_PORT, DISABLE);
    imu_bsp_config_spi_controller();
    IMU_LOG_INFO("SPI mode set cpol=%d cpha=%d", (cpol == SPI_CPOL_High) ? 1 : 0,
                 (cpha == SPI_CPHA_2Edge) ? 1 : 0);
    return IMU_STATUS_OK;
}

imu_status_t imu_bsp_init(void)
{
    imu_status_t status;

    if (s_sensor_power_mode == IMU_BSP_POWER_WEAK_PULL_UP)
    {
        status = imu_bsp_power_access_begin();
        if (status != IMU_STATUS_OK) return status;
        imu_bsp_power_access_end();
    }
    if (s_imu_bsp_inited)
    {
        return IMU_STATUS_OK;
    }

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    RCC_PeriphClockCmd(IMU_SPI_CLOCK_ID, IMU_SPI_CLOCK_MASK, ENABLE);
    /*
     * Keep IMU SPI source deterministic for throughput validation:
     * fixed 40MHz source + SPI clock divider /1 -> SPI controller source = 40MHz.
     * With BAUDR=/2, IMU SCLK becomes 20MHz.
     */
    RCC_SPIClockConfig(IMU_SPI_PORT, DISABLE);
    RCC_SPIClkDivConfig(IMU_SPI_PORT, SPI_CLOCK_DIV_1);

    imu_bsp_config_spi_pins();

    status = imu_bsp_config_cs_pin();
    if (status != IMU_STATUS_OK)
    {
        IMU_LOG_ERROR("CS pin config failed, status=%d", status);
        return status;
    }

    /* Parking invalidates PAD/PINMUX, even when the GPIO mask is cached. */
    status = imu_bsp_flash_cs_hold_high();
    if (status != IMU_STATUS_OK) return status;

    imu_bsp_config_spi_controller();
#if IMU_OIS_SPI_DMA_ENABLE
    imu_bsp_spi_dma_init_once();
#endif
    s_imu_bsp_inited = true;
    return IMU_STATUS_OK;
}

void imu_bsp_mark_lost_after_dlps_prepare(void)
{
    s_imu_bsp_inited = false;
}

void imu_bsp_set_sleep_prepare_cs_diag(bool enable)
{
    IMU_UNUSED(enable);
}

imu_status_t imu_bsp_resume_after_dlps(void)
{
    imu_status_t status;
    uint8_t power_out = 0U;
    uint8_t power_in = 0U;
    uint8_t cs_out = 0U;
    uint8_t cs_in = 0U;
    uint8_t power_ok = 0U;
    uint8_t spi_ok = 0U;
    uint8_t pins_ok = 0U;
    uint8_t cs_ok = 0U;

    imu_bsp_set_sleep_prepare_cs_diag(false);
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    RCC_PeriphClockCmd(IMU_SPI_CLOCK_ID, IMU_SPI_CLOCK_MASK, ENABLE);
    RCC_SPIClockConfig(IMU_SPI_PORT, DISABLE);
    RCC_SPIClkDivConfig(IMU_SPI_PORT, SPI_CLOCK_DIV_1);

    imu_bsp_config_spi_pins();
    pins_ok = 1U;

    status = imu_bsp_config_cs_pin();
    if (status != IMU_STATUS_OK)
    {
        imu_bsp_log_wake_if_due(power_ok, spi_ok, pins_ok, cs_ok);
        return status;
    }
    status = imu_bsp_flash_cs_hold_high();
    if (status != IMU_STATUS_OK) return status;

    if (imu_bsp_read_gpio_levels(s_imu_cs_gpio_mask, &cs_out, &cs_in) == IMU_STATUS_OK)
    {
        cs_ok = ((cs_out != 0U) && (cs_in != 0U)) ? 1U : 0U;
    }

    SPI_Cmd(IMU_SPI_PORT, DISABLE);
    imu_bsp_config_spi_controller();
    status = imu_bsp_wait_spi_flag(SPI_FLAG_BUSY, RESET);
    if (status == IMU_STATUS_OK)
    {
        spi_ok = 1U;
    }

    if (imu_bsp_get_power_level(&power_out, &power_in) == IMU_STATUS_OK)
    {
        power_ok = ((power_out != 0U) || (power_in != 0U)) ? 1U : 0U;
    }
#if (IMU_POWER_CTRL_PIN == IMU_PIN_UNASSIGNED)
    else
    {
        power_ok = 1U;
    }
#endif

    s_imu_bsp_inited = true;
    imu_bsp_log_wake_if_due(power_ok, spi_ok, pins_ok, cs_ok);
    return status;
}

DATA_RAM_FUNCTION
void imu_bsp_cs_low(void)
{
    imu_bsp_cs_apply_level(false);
}

DATA_RAM_FUNCTION
void imu_bsp_cs_high(void)
{
    imu_bsp_cs_apply_level(true);
}

imu_status_t imu_bsp_get_cs_level(uint8_t *out_level, uint8_t *in_level)
{
    imu_status_t status;

    if ((out_level == NULL) || (in_level == NULL))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = imu_bsp_init();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    return imu_bsp_read_gpio_levels(s_imu_cs_gpio_mask, out_level, in_level);
}

imu_status_t imu_bsp_get_power_level(uint8_t *out_level, uint8_t *in_level)
{
#if (IMU_POWER_CTRL_PIN == IMU_PIN_UNASSIGNED)
    IMU_UNUSED(out_level);
    IMU_UNUSED(in_level);
    return IMU_STATUS_UNSUPPORTED;
#else
    if ((out_level == NULL) || (in_level == NULL))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    if (s_imu_power_gpio_mask == 0U)
    {
        if (!imu_bsp_pin_to_gpio_mask(IMU_POWER_CTRL_PIN, &s_imu_power_gpio_mask))
        {
            return IMU_STATUS_INVALID_PARAM;
        }
    }

    return imu_bsp_read_gpio_levels(s_imu_power_gpio_mask, out_level, in_level);
#endif
}

#if ZY100_FLASH_FIFO_TRANSFER_ENABLE
imu_status_t imu_bsp_spi_transfer_fifo(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    imu_status_t status;
    uint16_t sent = 0U, received = 0U;
    uint32_t guard = IMU_BSP_SPI_TIMEOUT_LOOP;
    if ((len == 0U) || ((tx == NULL) && (rx == NULL)))
    { return IMU_STATUS_INVALID_PARAM; }
    status = imu_bsp_init();
    if (status != IMU_STATUS_OK) { return status; }
    imu_bsp_flush_spi_rx_quiet();
    while (received < len)
    {
        bool progressed = false;
        while ((sent < len) &&
               ((uint16_t)(sent - received) < IMU_SPI_QUIET_READ_WINDOW_BYTES) &&
               (SPI_GetFlagState(IMU_SPI_PORT, SPI_FLAG_TFNF) == SET))
        {
            SPI_SendData(IMU_SPI_PORT, tx != NULL ? tx[sent] : 0xFFU);
            sent++;
            progressed = true;
        }
        while ((received < sent) &&
               (SPI_GetFlagState(IMU_SPI_PORT, SPI_FLAG_RFNE) == SET))
        {
            uint8_t value = (uint8_t)SPI_ReceiveData(IMU_SPI_PORT);
            if (rx != NULL) { rx[received] = value; }
            received++;
            progressed = true;
        }
        if (progressed) { guard = IMU_BSP_SPI_TIMEOUT_LOOP; }
        else if (guard-- == 0U) { return IMU_STATUS_TIMEOUT; }
    }
    /* CS remains asserted until the final bit leaves the peripheral. */
    return imu_bsp_wait_spi_flag(SPI_FLAG_BUSY, RESET);
}
#endif

imu_status_t imu_bsp_spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    imu_status_t status;
    uint16_t flushed;
    uint16_t i;

    if ((len == 0U) || ((tx == NULL) && (rx == NULL)))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = imu_bsp_init();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    flushed = imu_bsp_flush_spi_rx();

#if IMU_BSP_SPI_TRACE_ENABLE
    if (s_imu_spi_trace_cnt < IMU_BSP_SPI_TRACE_MAX_LOG)
    {
        IMU_LOG_INFO("SPI transfer begin len=%u tx=%u rx=%u flushed=%u",
                     (unsigned int)len,
                     (tx != NULL) ? 1U : 0U,
                     (rx != NULL) ? 1U : 0U,
                     (unsigned int)flushed);
        s_imu_spi_trace_cnt++;
    }
#else
    IMU_UNUSED(flushed);
#endif

    for (i = 0U; i < len; i++)
    {
        const uint8_t tx_byte = (tx != NULL) ? tx[i] : 0xFFU;
        uint8_t rx_byte;
#if IMU_BSP_SPI_TRACE_ENABLE
        uint8_t tx_fifo_before = 0U;
        uint8_t tx_fifo_after = 0U;
        uint8_t rx_fifo_before = 0U;
        uint8_t rx_fifo_after = 0U;
#endif

#if IMU_BSP_SPI_TRACE_ENABLE
        if (s_imu_spi_trace_cnt < IMU_BSP_SPI_TRACE_MAX_LOG)
        {
            tx_fifo_before = SPI_GetTxFIFOLen(IMU_SPI_PORT);
            rx_fifo_before = SPI_GetRxFIFOLen(IMU_SPI_PORT);
        }
#endif

        status = imu_bsp_wait_spi_flag(SPI_FLAG_TFNF, SET);
        if (status != IMU_STATUS_OK)
        {
            return status;
        }

        SPI_SendData(IMU_SPI_PORT, tx_byte);

        status = imu_bsp_wait_spi_flag(SPI_FLAG_RFNE, SET);
        if (status != IMU_STATUS_OK)
        {
            return status;
        }

        rx_byte = (uint8_t)SPI_ReceiveData(IMU_SPI_PORT);
        if (rx != NULL)
        {
            rx[i] = rx_byte;
        }

#if IMU_BSP_SPI_TRACE_ENABLE
        if (s_imu_spi_trace_cnt < IMU_BSP_SPI_TRACE_MAX_LOG)
        {
            tx_fifo_after = SPI_GetTxFIFOLen(IMU_SPI_PORT);
            rx_fifo_after = SPI_GetRxFIFOLen(IMU_SPI_PORT);
            IMU_LOG_INFO("SPI raw byte[%u/%u] tx=0x%02x rx=0x%02x tx_fifo=%u->%u rx_fifo=%u->%u",
                         (unsigned int)(i + 1U),
                         (unsigned int)len,
                         tx_byte,
                         rx_byte,
                         tx_fifo_before,
                         tx_fifo_after,
                         rx_fifo_before,
                         rx_fifo_after);
            s_imu_spi_trace_cnt++;
        }
#endif
    }

    status = imu_bsp_wait_spi_flag(SPI_FLAG_BUSY, RESET);
#if IMU_BSP_SPI_TRACE_ENABLE
    if (s_imu_spi_trace_cnt < IMU_BSP_SPI_TRACE_MAX_LOG)
    {
        IMU_LOG_INFO("SPI transfer end len=%u status=%d", (unsigned int)len, status);
        s_imu_spi_trace_cnt++;
    }
#endif
    return status;
}

DATA_RAM_FUNCTION
imu_status_t imu_bsp_spi_transfer_isr_fast(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    imu_status_t status;
    uint16_t i;

    if ((len == 0U) || ((tx == NULL) && (rx == NULL)))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    if (!s_imu_bsp_inited)
    {
        return IMU_STATUS_NOT_READY;
    }

    imu_bsp_flush_spi_rx_quiet();

    for (i = 0U; i < len; i++)
    {
        const uint8_t tx_byte = (tx != NULL) ? tx[i] : 0xFFU;

        status = imu_bsp_wait_spi_flag(SPI_FLAG_TFNF, SET);
        if (status != IMU_STATUS_OK)
        {
            return status;
        }

        SPI_SendData(IMU_SPI_PORT, tx_byte);

        status = imu_bsp_wait_spi_flag(SPI_FLAG_RFNE, SET);
        if (status != IMU_STATUS_OK)
        {
            return status;
        }

        if (rx != NULL)
        {
            rx[i] = (uint8_t)SPI_ReceiveData(IMU_SPI_PORT);
        }
        else
        {
            (void)SPI_ReceiveData(IMU_SPI_PORT);
        }
    }

    return imu_bsp_wait_spi_flag(SPI_FLAG_BUSY, RESET);
}

DATA_RAM_FUNCTION
imu_status_t imu_bsp_spi_read_reg_window_isr_fast(uint8_t cmd, uint8_t *rx, uint16_t rx_len)
{
    imu_status_t status;
    imu_status_t first_error = IMU_STATUS_OK;
    imu_status_t busy_status;
    uint16_t i;

    if ((rx == NULL) || (rx_len == 0U))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    if (!s_imu_bsp_inited)
    {
        return IMU_STATUS_NOT_READY;
    }

    imu_bsp_flush_spi_rx_quiet();

    status = imu_bsp_wait_spi_flag(SPI_FLAG_TFNF, SET);
    if (status != IMU_STATUS_OK)
    {
        first_error = status;
    }
    else
    {
        SPI_SendData(IMU_SPI_PORT, cmd);

        status = imu_bsp_wait_spi_flag(SPI_FLAG_RFNE, SET);
        if (status != IMU_STATUS_OK)
        {
            first_error = status;
        }
        else
        {
            (void)SPI_ReceiveData(IMU_SPI_PORT);
        }
    }

    if (first_error == IMU_STATUS_OK)
    {
        for (i = 0U; i < rx_len; i++)
        {
            status = imu_bsp_wait_spi_flag(SPI_FLAG_TFNF, SET);
            if (status != IMU_STATUS_OK)
            {
                first_error = status;
                break;
            }

            SPI_SendData(IMU_SPI_PORT, 0xFFU);

            status = imu_bsp_wait_spi_flag(SPI_FLAG_RFNE, SET);
            if (status != IMU_STATUS_OK)
            {
                first_error = status;
                break;
            }

            rx[i] = (uint8_t)SPI_ReceiveData(IMU_SPI_PORT);
        }
    }

    busy_status = imu_bsp_wait_spi_flag(SPI_FLAG_BUSY, RESET);
    if (first_error != IMU_STATUS_OK)
    {
        return first_error;
    }

    return busy_status;
}

DATA_RAM_FUNCTION
imu_status_t imu_bsp_spi_read_reg_window_isr_fast_fifo(uint8_t cmd,
                                                       uint8_t *rx,
                                                       uint16_t rx_len)
{
    imu_status_t status;
    imu_status_t first_error = IMU_STATUS_OK;
    imu_status_t busy_status;
    uint16_t tx_count = 0U;
    uint16_t rx_count = 0U;
    uint16_t outstanding = 0U;
    uint32_t guard = IMU_BSP_SPI_TIMEOUT_LOOP;

    if ((rx == NULL) || (rx_len == 0U))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    if (!s_imu_bsp_inited)
    {
        return IMU_STATUS_NOT_READY;
    }

    imu_bsp_flush_spi_rx_quiet();

    status = imu_bsp_wait_spi_flag(SPI_FLAG_TFNF, SET);
    if (status != IMU_STATUS_OK)
    {
        first_error = status;
    }
    else
    {
        SPI_SendData(IMU_SPI_PORT, cmd);
        status = imu_bsp_wait_spi_flag(SPI_FLAG_RFNE, SET);
        if (status != IMU_STATUS_OK)
        {
            first_error = status;
        }
        else
        {
            (void)SPI_ReceiveData(IMU_SPI_PORT);
        }
    }

    while ((first_error == IMU_STATUS_OK) && (rx_count < rx_len))
    {
        bool progressed = false;

        while ((tx_count < rx_len) &&
               (outstanding < IMU_SPI_QUIET_READ_WINDOW_BYTES) &&
               (SPI_GetFlagState(IMU_SPI_PORT, SPI_FLAG_TFNF) == SET))
        {
            SPI_SendData(IMU_SPI_PORT, 0xFFU);
            tx_count++;
            outstanding++;
            progressed = true;
        }

        while ((outstanding != 0U) &&
               (SPI_GetFlagState(IMU_SPI_PORT, SPI_FLAG_RFNE) == SET))
        {
            rx[rx_count] = (uint8_t)SPI_ReceiveData(IMU_SPI_PORT);
            rx_count++;
            outstanding--;
            progressed = true;
        }

        if (progressed)
        {
            guard = IMU_BSP_SPI_TIMEOUT_LOOP;
        }
        else if (guard-- == 0U)
        {
            first_error = IMU_STATUS_TIMEOUT;
        }
    }

    busy_status = imu_bsp_wait_spi_flag(SPI_FLAG_BUSY, RESET);
    imu_bsp_flush_spi_rx_quiet();
    if (first_error != IMU_STATUS_OK)
    {
        return first_error;
    }
    return busy_status;
}

imu_status_t imu_bsp_spi_read_quiet(uint8_t *rx, uint16_t len)
{
    imu_status_t status;
    uint16_t tx_count = 0U;
    uint16_t rx_count = 0U;
    uint32_t guard = IMU_BSP_SPI_TIMEOUT_LOOP;

    if ((rx == NULL) || (len == 0U))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = imu_bsp_init();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    imu_bsp_flush_spi_rx_quiet();

    while (rx_count < len)
    {
        bool progressed = false;

        while ((tx_count < len) &&
               ((uint16_t)(tx_count - rx_count) < IMU_SPI_QUIET_READ_WINDOW_BYTES) &&
               (SPI_GetFlagState(IMU_SPI_PORT, SPI_FLAG_TFNF) == SET))
        {
            IMU_SPI_PORT->DR[0] = 0xFFU;
            tx_count++;
            progressed = true;
        }

        while ((rx_count < len) &&
               (SPI_GetFlagState(IMU_SPI_PORT, SPI_FLAG_RFNE) == SET))
        {
            rx[rx_count] = (uint8_t)SPI_ReceiveData(IMU_SPI_PORT);
            rx_count++;
            progressed = true;
        }

        if (progressed)
        {
            guard = IMU_BSP_SPI_TIMEOUT_LOOP;
        }
        else if (guard-- == 0U)
        {
            return IMU_STATUS_TIMEOUT;
        }
    }

    status = imu_bsp_wait_spi_flag(SPI_FLAG_BUSY, RESET);
    imu_bsp_flush_spi_rx_quiet();
    return status;
}

imu_status_t imu_bsp_spi_read_dma(uint8_t *rx, uint16_t len)
{
#if IMU_OIS_SPI_DMA_ENABLE
    imu_status_t status = IMU_STATUS_OK;
    uint64_t start_us;

    if ((rx == NULL) || (len == 0U) || (len > IMU_SPI_DMA_MAX_TRANSFER_BYTES))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = imu_bsp_init();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    imu_bsp_spi_dma_init_once();
    (void)imu_bsp_flush_spi_rx();

    s_imu_spi_dma_tx_done = 0U;
    s_imu_spi_dma_rx_done = 0U;
    s_imu_spi_dma_error = 0U;
    s_imu_spi_dma_wait_task = xTaskGetCurrentTaskHandle();
    (void)xTaskNotifyWait(0U, IMU_SPI_DMA_NOTIFY_MASK, NULL, 0U);

    GDMA_Cmd(IMU_SPI_DMA_TX_CHANNEL_NUM, DISABLE);
    GDMA_Cmd(IMU_SPI_DMA_RX_CHANNEL_NUM, DISABLE);
    GDMA_ClearAllTypeINT(IMU_SPI_DMA_TX_CHANNEL_NUM);
    GDMA_ClearAllTypeINT(IMU_SPI_DMA_RX_CHANNEL_NUM);
    GDMA_SetSourceAddress(IMU_SPI_DMA_TX_CHANNEL, (uint32_t)s_imu_spi_dma_tx_buf);
    GDMA_SetDestinationAddress(IMU_SPI_DMA_TX_CHANNEL, (uint32_t)(&(IMU_SPI_PORT->DR[0])));
    GDMA_SetBufferSize(IMU_SPI_DMA_TX_CHANNEL, len);
    GDMA_SetSourceAddress(IMU_SPI_DMA_RX_CHANNEL, (uint32_t)(&(IMU_SPI_PORT->DR[0])));
    GDMA_SetDestinationAddress(IMU_SPI_DMA_RX_CHANNEL, (uint32_t)rx);
    GDMA_SetBufferSize(IMU_SPI_DMA_RX_CHANNEL, len);

    SPI_GDMACmd(IMU_SPI_PORT, SPI_GDMAReq_Tx | SPI_GDMAReq_Rx, ENABLE);
    GDMA_Cmd(IMU_SPI_DMA_RX_CHANNEL_NUM, ENABLE);
    GDMA_Cmd(IMU_SPI_DMA_TX_CHANNEL_NUM, ENABLE);

    start_us = imu_bsp_local_timestamp_us();
    while (1)
    {
        uint32_t notify_value = 0U;
        imu_bsp_spi_dma_update_status_from_hw();
        (void)xTaskNotifyWait(0U, IMU_SPI_DMA_NOTIFY_MASK, &notify_value, 0U);
        imu_bsp_spi_dma_update_status_from_hw();

        if (s_imu_spi_dma_error || ((notify_value & IMU_SPI_DMA_NOTIFY_ERR) != 0U))
        {
            status = IMU_STATUS_BUS_ERROR;
            if (s_imu_spi_dma_error_total < 0xFFFFFFFFU)
            {
                s_imu_spi_dma_error_total++;
            }
            break;
        }

        if ((GDMA_BASE->STATUS_ERR & (BIT(IMU_SPI_DMA_TX_CHANNEL_NUM) | BIT(IMU_SPI_DMA_RX_CHANNEL_NUM))) != 0U)
        {
            status = IMU_STATUS_BUS_ERROR;
            if (s_imu_spi_dma_error_total < 0xFFFFFFFFU)
            {
                s_imu_spi_dma_error_total++;
            }
            break;
        }

        if ((s_imu_spi_dma_tx_done != 0U) && (s_imu_spi_dma_rx_done != 0U))
        {
            break;
        }

        if ((imu_bsp_local_timestamp_us() - start_us) >= IMU_SPI_DMA_TIMEOUT_US)
        {
            status = IMU_STATUS_TIMEOUT;
            if (s_imu_spi_dma_timeout_total < 0xFFFFFFFFU)
            {
                s_imu_spi_dma_timeout_total++;
            }
            break;
        }
    }

    return imu_bsp_spi_dma_finalize(status);
#else
    return imu_bsp_spi_transfer(NULL, rx, len);
#endif
}

void imu_bsp_spi_dma_get_stats(uint32_t *timeout_total, uint32_t *error_total)
{
#if IMU_OIS_SPI_DMA_ENABLE
    if (timeout_total != NULL)
    {
        *timeout_total = s_imu_spi_dma_timeout_total;
    }
    if (error_total != NULL)
    {
        *error_total = s_imu_spi_dma_error_total;
    }
#else
    if (timeout_total != NULL)
    {
        *timeout_total = 0U;
    }
    if (error_total != NULL)
    {
        *error_total = 0U;
    }
#endif
}

imu_status_t imu_bsp_ois_tick_timer_config(uint32_t period_us)
{
    if (period_us == 0U)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    if (period_us > (UINT32_MAX / IMU_OIS_TICK_TIMER_CLK_PER_US))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    return imu_bsp_ois_tick_timer_config_ticks(period_us * IMU_OIS_TICK_TIMER_CLK_PER_US);
}

imu_status_t imu_bsp_ois_tick_timer_config_ticks(uint32_t period_ticks)
{
    imu_status_t status;
    TIM_TimeBaseInitTypeDef tim_init;
    NVIC_InitTypeDef nvic_init;

    if (period_ticks == 0U)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = imu_bsp_init();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);
    /*
     * OIS validation path requires deterministic timer source:
     * use fixed 40MHz instead of PLL-derived clock.
     */
    RCC_TimerClockConfig(IMU_OIS_TICK_TIMER_NUM, DISABLE);

    TIM_Cmd(IMU_OIS_TICK_TIMER_NUM, DISABLE);
    TIM_INTConfig(IMU_OIS_TICK_TIMER_NUM, DISABLE);

    TIM_StructInit(&tim_init);
    tim_init.TIM_PWM_En = PWM_DISABLE;
    tim_init.TIM_Mode = TIM_Mode_UserDefine;
    tim_init.TIM_Period = period_ticks - 1U;
    TIM_TimeBaseInit(IMU_OIS_TICK_TIMER_NUM, &tim_init);

    nvic_init.NVIC_IRQChannel = IMU_OIS_TICK_TIMER_IRQn;
    nvic_init.NVIC_IRQChannelPriority = IMU_OIS_TICK_TIMER_NVIC_PRIORITY;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    TIM_ClearINT(IMU_OIS_TICK_TIMER_NUM);
    TIM_INTConfig(IMU_OIS_TICK_TIMER_NUM, ENABLE);

    s_imu_ois_tick_timer_cfg = true;
    s_imu_ois_tick_timer_running = false;
    return IMU_STATUS_OK;
}

imu_status_t imu_bsp_ois_tick_timer_acquire(imu_bsp_ois_tick_timer_owner_t owner)
{
    uint32_t lock_state;
    imu_status_t status = IMU_STATUS_OK;

    if (owner == IMU_BSP_OIS_TICK_TIMER_OWNER_NONE)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    lock_state = os_lock();
    if ((s_imu_ois_tick_timer_owner != IMU_BSP_OIS_TICK_TIMER_OWNER_NONE) &&
        (s_imu_ois_tick_timer_owner != owner))
    {
        status = IMU_STATUS_NOT_READY;
    }
    else
    {
        s_imu_ois_tick_timer_owner = owner;
    }
    os_unlock(lock_state);

    return status;
}

void imu_bsp_ois_tick_timer_release(imu_bsp_ois_tick_timer_owner_t owner)
{
    uint32_t lock_state;

    lock_state = os_lock();
    if ((owner != IMU_BSP_OIS_TICK_TIMER_OWNER_NONE) &&
        (s_imu_ois_tick_timer_owner == owner))
    {
        s_imu_ois_tick_timer_owner = IMU_BSP_OIS_TICK_TIMER_OWNER_NONE;
    }
    os_unlock(lock_state);
}

imu_bsp_ois_tick_timer_owner_t imu_bsp_ois_tick_timer_get_owner(void)
{
    uint32_t lock_state;
    imu_bsp_ois_tick_timer_owner_t owner;

    lock_state = os_lock();
    owner = s_imu_ois_tick_timer_owner;
    os_unlock(lock_state);

    return owner;
}

imu_status_t imu_bsp_ois_tick_timer_start(void)
{
    imu_status_t status;

    status = imu_bsp_init();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    if (!s_imu_ois_tick_timer_cfg)
    {
        return IMU_STATUS_NOT_READY;
    }

    if (imu_bsp_ois_tick_timer_get_owner() == IMU_BSP_OIS_TICK_TIMER_OWNER_NONE)
    {
        return IMU_STATUS_NOT_READY;
    }

    TIM_ClearINT(IMU_OIS_TICK_TIMER_NUM);
    TIM_Cmd(IMU_OIS_TICK_TIMER_NUM, ENABLE);
    s_imu_ois_tick_timer_running = true;
    return IMU_STATUS_OK;
}

void imu_bsp_ois_tick_timer_stop(void)
{
    TIM_Cmd(IMU_OIS_TICK_TIMER_NUM, DISABLE);
    TIM_ClearINT(IMU_OIS_TICK_TIMER_NUM);
    s_imu_ois_tick_timer_running = false;
}

bool imu_bsp_ois_tick_timer_is_running(void)
{
    return s_imu_ois_tick_timer_running;
}

void imu_bsp_ois_tick_timer_register_irq_callback(imu_bsp_timer_irq_cb_t cb)
{
    s_imu_ois_tick_timer_irq_cb = cb;
}

void imu_bsp_ois_tick_timer_get_stats(uint32_t *irq_count, uint32_t *last_vendor_tick)
{
    if (irq_count != NULL)
    {
        *irq_count = s_imu_ois_tick_timer_irq_count;
    }
    if (last_vendor_tick != NULL)
    {
        *last_vendor_tick = s_imu_ois_tick_timer_last_tick;
    }
}

void imu_bsp_delay_us(uint32_t us)
{
    if (us == 0U)
    {
        return;
    }

    if (platform_delay_us != NULL)
    {
#if IMU_BSP_DELAY_PATH_LOG_ENABLE
        if (!s_imu_bsp_delay_path_logged ||
            ((us == 500U) && (s_imu_bsp_delay_500_log_count < 4U)))
        {
            DBG_DIRECT("[IMU_BSP_DELAY] us=%u platform_delay_us=0x%08x path=platform",
                       us,
                       (uint32_t)platform_delay_us);
            s_imu_bsp_delay_path_logged = true;
            if (us == 500U)
            {
                s_imu_bsp_delay_500_log_count++;
            }
        }
#endif
        platform_delay_us(us);
        return;
    }

#if IMU_BSP_DELAY_PATH_LOG_ENABLE
    if (!s_imu_bsp_delay_path_logged ||
        ((us == 500U) && (s_imu_bsp_delay_500_log_count < 4U)))
    {
        DBG_DIRECT("[IMU_BSP_DELAY] us=%u platform_delay_us=0x%08x path=vendor_tick_fallback tick_per_us=%u",
                   us,
                   (uint32_t)platform_delay_us,
                   (uint32_t)IMU_VENDOR_TICK_PER_US);
        s_imu_bsp_delay_path_logged = true;
        if (us == 500U)
        {
            s_imu_bsp_delay_500_log_count++;
        }
    }
#endif

    while (us != 0U)
    {
        const uint32_t chunk_us = (us > 1000000U) ? 1000000U : us;
        const uint32_t chunk_ticks = chunk_us * IMU_VENDOR_TICK_PER_US;
        const uint32_t start = platform_vendor_tick();
        uint32_t elapsed;

        do
        {
            const uint32_t now = platform_vendor_tick();
            elapsed = (now - start) & IMU_VENDOR_TICK_MASK;
        } while (elapsed < chunk_ticks);

        us -= chunk_us;
    }
}

uint64_t imu_bsp_local_timestamp_us(void)
{
    uint32_t tick = platform_vendor_tick();
    uint64_t full_tick;

    if (!s_tick_inited)
    {
        s_tick_inited = true;
        s_tick_last = tick;
    }
    else if (tick < s_tick_last)
    {
        s_tick_wrap_base += (uint64_t)(IMU_VENDOR_TICK_MASK + 1U);
    }

    s_tick_last = tick;
    full_tick = s_tick_wrap_base + (uint64_t)tick;
    return full_tick / IMU_VENDOR_TICK_PER_US;
}

bool imu_bsp_int_pin_assigned(void)
{
    return (IMU_INT_PIN != IMU_PIN_UNASSIGNED);
}

imu_status_t imu_bsp_int_init(void)
{
    GPIO_InitTypeDef gpio_init;
    NVIC_InitTypeDef nvic_init;

    if (!imu_bsp_int_pin_assigned())
    {
        return IMU_STATUS_OK;
    }

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);

    if (!imu_bsp_pin_to_gpio_mask(IMU_INT_PIN, &s_imu_int_gpio_mask))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    Pad_Config(IMU_INT_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);
    Pinmux_Deinit(IMU_INT_PIN);
    Pinmux_Config(IMU_INT_PIN, DWGPIO);

    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = s_imu_int_gpio_mask;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_ITCmd = ENABLE;
    gpio_init.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
    gpio_init.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_HIGH;
    gpio_init.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_DISABLE;
    gpio_init.GPIO_ControlMode = GPIO_SOFTWARE_MODE;
    GPIO_Init(&gpio_init);
    GPIO_ClearINTPendingBit(s_imu_int_gpio_mask);
    GPIO_MaskINTConfig(s_imu_int_gpio_mask, DISABLE);
    GPIO_INTConfig(s_imu_int_gpio_mask, ENABLE);

    nvic_init.NVIC_IRQChannel = IMU_INT_GPIO_IRQn;
    nvic_init.NVIC_IRQChannelPriority = IMU_INT_NVIC_PRIORITY;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    return IMU_STATUS_OK;
}

imu_status_t imu_bsp_int_init_wom_active_low_masked(void)
{
    GPIO_InitTypeDef gpio_init;
    NVIC_InitTypeDef nvic_init;

    if (!imu_bsp_int_pin_assigned())
    {
        return IMU_STATUS_OK;
    }

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);

    if (!imu_bsp_pin_to_gpio_mask(IMU_INT_PIN, &s_imu_int_gpio_mask))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    Pad_Config(IMU_INT_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_HIGH);
    Pinmux_Deinit(IMU_INT_PIN);
    Pinmux_Config(IMU_INT_PIN, DWGPIO);

    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = s_imu_int_gpio_mask;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_ITCmd = ENABLE;
    gpio_init.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
    gpio_init.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
    gpio_init.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_DISABLE;
    gpio_init.GPIO_ControlMode = GPIO_SOFTWARE_MODE;
    GPIO_Init(&gpio_init);
    GPIO_MaskINTConfig(s_imu_int_gpio_mask, ENABLE);
    GPIO_INTConfig(s_imu_int_gpio_mask, DISABLE);
    GPIO_ClearINTPendingBit(s_imu_int_gpio_mask);

    nvic_init.NVIC_IRQChannel = IMU_INT_GPIO_IRQn;
    nvic_init.NVIC_IRQChannelPriority = IMU_INT_NVIC_PRIORITY;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    return IMU_STATUS_OK;
}

imu_status_t imu_bsp_int_init_wom_active_low(void)
{
    imu_status_t status = imu_bsp_int_init_wom_active_low_masked();

    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    return imu_bsp_int_enable_irq() ? IMU_STATUS_OK : IMU_STATUS_NOT_READY;
}

void imu_bsp_int_register_irq_callback(imu_bsp_int_irq_cb_t cb)
{
    s_imu_int_irq_cb = cb;
}

imu_status_t imu_bsp_int_snapshot(imu_bsp_int_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    snapshot->valid = false;
    snapshot->pin_assigned = imu_bsp_int_pin_assigned();
    snapshot->int_enabled = false;
    snapshot->int_masked = true;
    snapshot->callback = s_imu_int_irq_cb;

    if (!snapshot->pin_assigned)
    {
        snapshot->valid = true;
        return IMU_STATUS_OK;
    }

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);

    if (s_imu_int_gpio_mask == 0U)
    {
        if (!imu_bsp_pin_to_gpio_mask(IMU_INT_PIN, &s_imu_int_gpio_mask))
        {
            return IMU_STATUS_INVALID_PARAM;
        }
    }

    snapshot->int_enabled = ((GPIO->INTEN & s_imu_int_gpio_mask) != 0U);
    snapshot->int_masked = ((GPIO->INTMASK & s_imu_int_gpio_mask) != 0U);
    snapshot->valid = true;
    return IMU_STATUS_OK;
}

imu_status_t imu_bsp_int_restore(const imu_bsp_int_snapshot_t *snapshot)
{
    if ((snapshot == NULL) || !snapshot->valid)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    if (!snapshot->pin_assigned)
    {
        s_imu_int_irq_cb = snapshot->callback;
        return IMU_STATUS_OK;
    }

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);

    if (s_imu_int_gpio_mask == 0U)
    {
        if (!imu_bsp_pin_to_gpio_mask(IMU_INT_PIN, &s_imu_int_gpio_mask))
        {
            return IMU_STATUS_INVALID_PARAM;
        }
    }

    GPIO_INTConfig(s_imu_int_gpio_mask, DISABLE);
    GPIO_MaskINTConfig(s_imu_int_gpio_mask, ENABLE);
    GPIO_ClearINTPendingBit(s_imu_int_gpio_mask);
    s_imu_int_irq_cb = snapshot->callback;
    GPIO_MaskINTConfig(s_imu_int_gpio_mask,
                       snapshot->int_masked ? ENABLE : DISABLE);
    GPIO_INTConfig(s_imu_int_gpio_mask,
                   snapshot->int_enabled ? ENABLE : DISABLE);
    return IMU_STATUS_OK;
}

uint8_t imu_bsp_int_level(void)
{
    if (s_imu_int_gpio_mask == 0U)
    {
        return 0U;
    }

    return GPIO_ReadInputDataBit(s_imu_int_gpio_mask);
}

void imu_bsp_int_clear_pending(void)
{
    if (s_imu_int_gpio_mask != 0U)
    {
        GPIO_ClearINTPendingBit(s_imu_int_gpio_mask);
    }
}

bool imu_bsp_int_enable_irq(void)
{
    if (!imu_bsp_int_pin_assigned())
    {
        return false;
    }

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);

    if (s_imu_int_gpio_mask == 0U)
    {
        if (!imu_bsp_pin_to_gpio_mask(IMU_INT_PIN, &s_imu_int_gpio_mask))
        {
            return false;
        }
    }

    GPIO_MaskINTConfig(s_imu_int_gpio_mask, ENABLE);
    GPIO_ClearINTPendingBit(s_imu_int_gpio_mask);
    GPIO_INTConfig(s_imu_int_gpio_mask, ENABLE);
    GPIO_MaskINTConfig(s_imu_int_gpio_mask, DISABLE);
    return true;
}

bool imu_bsp_int_disable_irq(void)
{
    if (!imu_bsp_int_pin_assigned())
    {
        return false;
    }

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);

    if (s_imu_int_gpio_mask == 0U)
    {
        if (!imu_bsp_pin_to_gpio_mask(IMU_INT_PIN, &s_imu_int_gpio_mask))
        {
            return false;
        }
    }

    GPIO_MaskINTConfig(s_imu_int_gpio_mask, ENABLE);
    GPIO_INTConfig(s_imu_int_gpio_mask, DISABLE);
    GPIO_ClearINTPendingBit(s_imu_int_gpio_mask);
    return true;
}

bool imu_bsp_int_disable_for_dlps(void)
{
    return imu_bsp_int_disable_irq();
}

void IMU_INT_GPIO_ISR(void)
{
    if (!imu_bsp_int_pin_assigned())
    {
        return;
    }

    imu_bsp_int_clear_pending();
    if (s_imu_int_irq_cb != NULL)
    {
        s_imu_int_irq_cb();
    }
}

void IMU_OIS_TICK_TIMER_ISR(void)
{
    TIM_ClearINT(IMU_OIS_TICK_TIMER_NUM);
    if (!s_imu_ois_tick_timer_running)
    {
        return;
    }

    if (s_imu_ois_tick_timer_irq_count < 0xFFFFFFFFU)
    {
        s_imu_ois_tick_timer_irq_count++;
    }
    s_imu_ois_tick_timer_last_tick = platform_vendor_tick();

    if (s_imu_ois_tick_timer_irq_cb != NULL)
    {
        s_imu_ois_tick_timer_irq_cb();
    }
}

#if IMU_OIS_SPI_DMA_ENABLE
void IMU_SPI_DMA_TX_HANDLER(void)
{
    imu_bsp_spi_dma_handle_irq(IMU_SPI_DMA_TX_CHANNEL_NUM,
                               &s_imu_spi_dma_tx_done,
                               IMU_SPI_DMA_NOTIFY_TX_DONE);
}

void IMU_SPI_DMA_RX_HANDLER(void)
{
    imu_bsp_spi_dma_handle_irq(IMU_SPI_DMA_RX_CHANNEL_NUM,
                               &s_imu_spi_dma_rx_done,
                               IMU_SPI_DMA_NOTIFY_RX_DONE);
}
#endif

uint32_t imu_bsp_power_generation(void)
{
#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE
    return s_sensor_power_generation;
#else
    return 0U;
#endif
}

uint16_t imu_bsp_power_users(void)
{
#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE
    return s_sensor_power_users;
#else
    return 0U;
#endif
}

bool imu_bsp_power_park_begin(void)
{
#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE
    uint32_t key = os_lock();
    bool available = !s_sensor_power_parking && !s_sensor_power_restoring &&
                     (s_sensor_power_users == 0U);
    if (available) s_sensor_power_parking = true;
    os_unlock(key);
    return available;
#else
    return true;
#endif
}

void imu_bsp_power_park_end(void)
{
#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE
    uint32_t key = os_lock();
    s_sensor_power_parking = false;
    os_unlock(key);
#endif
}

imu_bsp_power_mode_t imu_bsp_power_mode(void)
{
    return s_sensor_power_mode;
}

void imu_bsp_power_set_restore_callback(imu_bsp_power_restore_cb_t callback)
{
#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE
    s_sensor_power_restore_cb = callback;
#else
    IMU_UNUSED(callback);
#endif
}

imu_status_t imu_bsp_power_enter_weak_pull(void)
{
#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE && (IMU_POWER_CTRL_PIN != IMU_PIN_UNASSIGNED)
    GPIO_InitTypeDef gpio_init;
    uint32_t key = os_lock();
    if ((s_sensor_power_users != 0U) || s_sensor_power_restoring ||
        ((s_sensor_power_mode != IMU_BSP_POWER_DRIVE_HIGH) &&
         (s_sensor_power_mode != IMU_BSP_POWER_WEAK_PULL_UP)) ||
        (s_sensor_power_restore_cb == NULL) ||
        (bsp_shared_spi_current_owner() != BSP_SHARED_SPI_OWNER_NONE))
    {
        os_unlock(key);
        return IMU_STATUS_NOT_READY;
    }
    Pinmux_Deinit(IMU_POWER_CTRL_PIN);
    Pinmux_Config(IMU_POWER_CTRL_PIN, DWGPIO);
    GPIO_SetBits(s_imu_power_gpio_mask);
    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = s_imu_power_gpio_mask;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_ITCmd = DISABLE;
    gpio_init.GPIO_ControlMode = GPIO_SOFTWARE_MODE;
    GPIO_Init(&gpio_init);
    Pad_PullConfigValue(IMU_POWER_CTRL_PIN, PAD_WEAK_PULL);
    Pad_Config(IMU_POWER_CTRL_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP,
               PAD_OUT_DISABLE, PAD_OUT_HIGH);
    s_sensor_power_mode = IMU_BSP_POWER_WEAK_PULL_UP;
    os_unlock(key);
    return IMU_STATUS_OK;
#else
    return IMU_STATUS_NOT_READY;
#endif
}

imu_status_t imu_bsp_power_access_begin(void)
{
#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE
    imu_status_t status = IMU_STATUS_OK;
    void *task = NULL;
    uint32_t key;
    bool restore;
    key = os_lock();
    if (s_sensor_power_parking)
    {
        os_unlock(key);
        return IMU_STATUS_NOT_READY;
    }
    s_sensor_power_generation++;
    /* Normal sampling, including ISR SPI ownership, never calls task APIs. */
    if (!s_sensor_power_restoring &&
        (s_sensor_power_mode == IMU_BSP_POWER_DRIVE_HIGH))
    {
        s_sensor_power_users++;
        os_unlock(key);
        return IMU_STATUS_OK;
    }
    if (__get_IPSR() != 0U)
    {
        os_unlock(key);
        return IMU_STATUS_NOT_READY;
    }
    (void)os_task_handle_get(&task);
    if (s_sensor_power_restoring && (task != s_sensor_power_restore_task))
    {
        os_unlock(key);
        return IMU_STATUS_NOT_READY;
    }
    restore = (s_sensor_power_mode != IMU_BSP_POWER_DRIVE_HIGH);
    if (restore && s_sensor_power_restore_cb == NULL)
    {
        os_unlock(key);
        return IMU_STATUS_NOT_READY;
    }
    s_sensor_power_users++;
    if (restore)
    {
        s_sensor_power_restoring = true;
        s_sensor_power_restore_task = task;
    }
    os_unlock(key);
    if (restore)
    {
        RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
        status = imu_bsp_apply_output_level(IMU_POWER_CTRL_PIN,
                                            &s_imu_power_gpio_mask, true);
        if (status == IMU_STATUS_OK)
        {
            s_sensor_power_mode = IMU_BSP_POWER_DRIVE_HIGH;
            /* Existing ADC rail-settle interval also covers the WOM restore. */
            imu_bsp_delay_us(BATTERY_ADC_NON_CAPTURE_SETTLE_MS * 1000U);
            status = (s_sensor_power_restore_cb != NULL) ?
                     s_sensor_power_restore_cb() : IMU_STATUS_NOT_READY;
        }
        key = os_lock();
        s_sensor_power_restoring = false;
        s_sensor_power_restore_task = NULL;
        os_unlock(key);
    }
    if (status != IMU_STATUS_OK) imu_bsp_power_access_end();
    return status;
#else
    return IMU_STATUS_OK;
#endif
}

void imu_bsp_power_access_end(void)
{
#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE
    uint32_t key = os_lock();
    if (s_sensor_power_users != 0U) s_sensor_power_users--;
    os_unlock(key);
#endif
}

imu_status_t imu_bsp_power_ctrl(bool enable)
{
#if (IMU_POWER_CTRL_PIN == IMU_PIN_UNASSIGNED)
    IMU_UNUSED(enable);
    return IMU_STATUS_OK;
#else
    imu_status_t status;
    bool requested_high;
    bool drive_high;

#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE
    if (enable && (s_sensor_power_mode == IMU_BSP_POWER_WEAK_PULL_UP))
    {
        status = imu_bsp_power_access_begin();
        if (status == IMU_STATUS_OK) imu_bsp_power_access_end();
        return status;
    }
#endif
#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE
    if (enable ? s_sensor_power_parking : !imu_bsp_power_park_begin())
        return IMU_STATUS_NOT_READY;
    s_sensor_power_generation++;
#endif
    requested_high = enable ? (IMU_POWER_CTRL_ACTIVE_HIGH != 0U) : (IMU_POWER_CTRL_ACTIVE_HIGH == 0U);
    drive_high = requested_high;
    if (enable)
    {
        imu_bsp_set_sleep_prepare_cs_diag(false);
        RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    }

    status = imu_bsp_apply_output_level(IMU_POWER_CTRL_PIN,
                                        &s_imu_power_gpio_mask,
                                        drive_high);
    if (status != IMU_STATUS_OK)
    {
#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE
        if (!enable) imu_bsp_power_park_end();
#endif
        return status;
    }

    s_sensor_power_mode = enable ? IMU_BSP_POWER_DRIVE_HIGH : IMU_BSP_POWER_DRIVE_LOW;
#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE
    if (!enable) imu_bsp_power_park_end();
#endif
    IMU_LOG_INFO("power ctrl pin=%d active_high=%d enable=%d req=%d final=%d mode=%s",
                 IMU_POWER_CTRL_PIN,
                 (IMU_POWER_CTRL_ACTIVE_HIGH != 0U) ? 1 : 0,
                 enable ? 1 : 0,
                 requested_high ? 1 : 0,
                 drive_high ? 1 : 0,
                 drive_high ? "gpio_out_high" : "gpio_out_low");
    return IMU_STATUS_OK;
#endif
}

void imu_bsp_sleep_gpio_park(bool hold)
{
    const uint32_t cs = GPIO_GetPin(IMU_SPI_CS_PIN) | GPIO_GetPin(IMU_FLASH_CS_PIN);
    const uint32_t power = GPIO_GetPin(IMU_POWER_CTRL_PIN);
    const uint32_t owned = cs | power;
    uint32_t outputs = hold ? cs : power;
    uint32_t key = os_lock();
    if (hold && imu_bsp_power_mode() == IMU_BSP_POWER_DRIVE_HIGH) outputs |= power;
    /* PAD parking has already established the physical levels. These masked
     * writes normalize only the GPIO backing state; no PAD/IRQ/event changes. */
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    GPIO->DATAOUT = (GPIO->DATAOUT & ~owned) | (hold ? owned : 0U);
    GPIO->DATASRC &= ~owned;
    GPIO->DATADIR = (GPIO->DATADIR & ~owned) | outputs;
    os_unlock(key);
}
