#include "drv_rgb_led.h"

#include <stdbool.h>
#include <stddef.h>

#include "app_section.h"
#include "platform_utils.h"
#include "rtl876x_gdma.h"
#include "rtl876x_gpio.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"
#include "rtl876x_tim.h"
#include "system_rtl876x.h"

#define DRV_RGB_LED_VENDOR_TICK_REG        0x4005817CUL
#define DRV_RGB_LED_VENDOR_TICK_MASK       0x03FFFFFFUL
#define DRV_RGB_LED_VENDOR_TICK_PER_US     80U
#define DRV_RGB_LED_REQUIRED_CPU_HZ        80000000U
#define DRV_RGB_LED_T0H_TICKS_DEFAULT      14U
#define DRV_RGB_LED_T0L_TICKS_DEFAULT      19U
#define DRV_RGB_LED_T1H_TICKS_DEFAULT      37U
#define DRV_RGB_LED_T1L_TICKS_DEFAULT      0U
#define DRV_RGB_LED_FRAME_GAP_US           500U
#define DRV_RGB_LED_RESET_LATCH_US         300U
#define DRV_RGB_LED_TICK_PROBE_WAIT        200U
#define DRV_RGB_LED_BITS_PER_BYTE          8U
#define DRV_RGB_LED_CHANNELS_PER_LED       3U
#define DRV_RGB_LED_BITS_PER_LED           (DRV_RGB_LED_CHANNELS_PER_LED * DRV_RGB_LED_BITS_PER_BYTE)
#define DRV_RGB_LED_DMA_SAMPLE_TICKS       8U
#define DRV_RGB_LED_DMA_SAMPLES_PER_BIT    3U
#define DRV_RGB_LED_DMA_START_LOW_SAMPLES  1U
#define DRV_RGB_LED_DMA_ZERO_HIGH_SAMPLES  1U
#define DRV_RGB_LED_DMA_ONE_HIGH_SAMPLES   2U
#define DRV_RGB_LED_DMA_MAX_WORDS          (DRV_RGB_LED_DMA_START_LOW_SAMPLES + \
                                            ZY100_RGB_LED_COUNT * DRV_RGB_LED_BITS_PER_LED * \
                                            DRV_RGB_LED_DMA_SAMPLES_PER_BIT)
#define DRV_RGB_LED_DMA_TIMEOUT_US         5000U
#define DRV_RGB_LED_EDGE_TEST_EDGES_PER_BIT 2U

#ifndef F_LED_EDGE_HW_TEST_DATA_PULL_UP_ENABLE
#define F_LED_EDGE_HW_TEST_DATA_PULL_UP_ENABLE 0
#endif

#define DRV_RGB_LED_DMA_TIMER              TIM5
#define DRV_RGB_LED_DMA_HANDSHAKE          GDMA_Handshake_TIM5
#define DRV_RGB_LED_DMA_CHANNEL_NUM        2U
#define DRV_RGB_LED_DMA_CHANNEL            GDMA_Channel2

/*
 * RTL8762D SDK TIM/GDMA+GPIO sample writes GPIO waveform words to this vendor
 * GPIO hardware-output register with timer DMA handshaking.
 */
#define DRV_RGB_LED_GPIO_DMA_DATA_REG      0x40011200UL

typedef char drv_rgb_led_count_check[
    ((ZY100_RGB_LED_COUNT == 8U) || (ZY100_RGB_LED_COUNT == 13U)) ? 1 : -1];
typedef char drv_rgb_led_dma_sample_check[
    (DRV_RGB_LED_DMA_ONE_HIGH_SAMPLES < DRV_RGB_LED_DMA_SAMPLES_PER_BIT) ? 1 : -1];
typedef char drv_rgb_led_dma_buffer_check[(DRV_RGB_LED_DMA_MAX_WORDS <= 4095U) ? 1 : -1];

#if defined(__CC_ARM)
#define DRV_RGB_LED_FORCE_INLINE           static __forceinline
#else
#define DRV_RGB_LED_FORCE_INLINE           static inline
#endif

static bool s_drv_rgb_led_ready = false;
static uint32_t s_drv_rgb_led_gpio_pin = 0U;
static uint32_t s_drv_rgb_led_dma_high_word = 0U;
static uint32_t s_drv_rgb_led_dma_low_word = 0U;
static uint32_t s_drv_rgb_led_dma_waveform[DRV_RGB_LED_DMA_MAX_WORDS];

static PAD_OUTPUT_VAL drv_rgb_led_mcu_pad_level_for_led_level(bool led_high)
{
    const bool mcu_high = (ZY100_RGB_LED_DATA_ACTIVE_LOW != 0U) ? !led_high : led_high;

    return mcu_high ? PAD_OUT_HIGH : PAD_OUT_LOW;
}

static bool drv_rgb_led_data_pin_valid(void)
{
    const uint32_t gpio_pin = GPIO_GetPin(ZY100_RGB_LED_DATA_PIN);

    return (gpio_pin != 0U) && (gpio_pin != 0xFFFFFFFFUL);
}

static bool drv_rgb_led_clock_valid(void)
{
    return (get_cpu_clock() == DRV_RGB_LED_REQUIRED_CPU_HZ);
}

static PAD_Pull_Mode drv_rgb_led_data_pad_pull(void)
{
#if F_LED_EDGE_HW_TEST_DATA_PULL_UP_ENABLE
    return PAD_PULL_UP;
#else
    return PAD_PULL_NONE;
#endif
}

DRV_RGB_LED_FORCE_INLINE uint32_t drv_rgb_led_tick_now(void)
{
    return (*(volatile uint32_t *)DRV_RGB_LED_VENDOR_TICK_REG) & DRV_RGB_LED_VENDOR_TICK_MASK;
}

DRV_RGB_LED_FORCE_INLINE uint32_t drv_rgb_led_tick_delta(uint32_t start, uint32_t now)
{
    return (now - start) & DRV_RGB_LED_VENDOR_TICK_MASK;
}

static bool drv_rgb_led_tick_running(void)
{
    uint32_t start = drv_rgb_led_tick_now();
    uint32_t probe;

    for (probe = 0U; probe < DRV_RGB_LED_TICK_PROBE_WAIT; probe++)
    {
        if (drv_rgb_led_tick_now() != start)
        {
            return true;
        }
    }

    return false;
}

DRV_RGB_LED_FORCE_INLINE bool drv_rgb_led_wait_ticks(uint32_t start_tick,
                                                     uint32_t ticks)
{
    uint32_t last_tick = start_tick;
    uint32_t stalled_reads = 0U;

    while (drv_rgb_led_tick_delta(start_tick, drv_rgb_led_tick_now()) < ticks)
    {
        uint32_t now = drv_rgb_led_tick_now();

        if (now == last_tick)
        {
            stalled_reads++;
            if (stalled_reads >= DRV_RGB_LED_TICK_PROBE_WAIT)
            {
                return false;
            }
        }
        else
        {
            last_tick = now;
            stalled_reads = 0U;
        }
    }
    return true;
}

static bool drv_rgb_led_wait_us_raw(uint32_t us)
{
    uint32_t start = drv_rgb_led_tick_now();

    return drv_rgb_led_wait_ticks(
               start, us * DRV_RGB_LED_VENDOR_TICK_PER_US);
}

drv_rgb_led_status_t drv_rgb_led_wait_us_checked(uint32_t us)
{
    if (us == 0U)
    {
        return DRV_RGB_LED_STATUS_OK;
    }
    if (!drv_rgb_led_tick_running())
    {
        return DRV_RGB_LED_STATUS_TICK_STUCK;
    }
    if (!drv_rgb_led_wait_us_raw(us))
    {
        return DRV_RGB_LED_STATUS_TICK_STUCK;
    }

    return DRV_RGB_LED_STATUS_OK;
}

static void drv_rgb_led_dma_gpio_low(void)
{
    *((volatile uint32_t *)DRV_RGB_LED_GPIO_DMA_DATA_REG) = s_drv_rgb_led_dma_low_word;
}

static void drv_rgb_led_dma_stop(void)
{
    TIM_Cmd(DRV_RGB_LED_DMA_TIMER, DISABLE);
    GDMA_Cmd(DRV_RGB_LED_DMA_CHANNEL_NUM, DISABLE);
    GDMA_ClearAllTypeINT(DRV_RGB_LED_DMA_CHANNEL_NUM);
    drv_rgb_led_dma_gpio_low();
}

static uint16_t drv_rgb_led_append_waveform_bit(bool one, uint16_t word_index)
{
    uint8_t sample;
    const uint8_t high_samples = one ? DRV_RGB_LED_DMA_ONE_HIGH_SAMPLES :
                                 DRV_RGB_LED_DMA_ZERO_HIGH_SAMPLES;

    for (sample = 0U; sample < DRV_RGB_LED_DMA_SAMPLES_PER_BIT; sample++)
    {
        s_drv_rgb_led_dma_waveform[word_index] =
            (sample < high_samples) ? s_drv_rgb_led_dma_high_word : s_drv_rgb_led_dma_low_word;
        word_index++;
    }

    return word_index;
}

static uint16_t drv_rgb_led_append_byte_waveform(uint8_t value, uint16_t word_index)
{
    uint8_t mask;

    for (mask = 0x80U; mask != 0U; mask >>= 1U)
    {
        word_index = drv_rgb_led_append_waveform_bit((value & mask) != 0U, word_index);
    }

    return word_index;
}

static uint16_t drv_rgb_led_build_dma_waveform(const zy100_rgb_color_t *colors, uint16_t count)
{
    uint16_t led;
    uint16_t word_index = 0U;
    uint8_t sample;

    for (sample = 0U; sample < DRV_RGB_LED_DMA_START_LOW_SAMPLES; sample++)
    {
        s_drv_rgb_led_dma_waveform[word_index] = s_drv_rgb_led_dma_low_word;
        word_index++;
    }

    for (led = 0U; led < count; led++)
    {
        word_index = drv_rgb_led_append_byte_waveform(colors[led].green, word_index);
        word_index = drv_rgb_led_append_byte_waveform(colors[led].red, word_index);
        word_index = drv_rgb_led_append_byte_waveform(colors[led].blue, word_index);
    }

    return word_index;
}

static uint16_t drv_rgb_led_build_edge_test_waveform(uint8_t edge_count)
{
    uint16_t bit;
    uint16_t bit_count = (uint16_t)(edge_count / DRV_RGB_LED_EDGE_TEST_EDGES_PER_BIT);
    uint16_t word_index = 0U;
    uint8_t sample;

    for (sample = 0U; sample < DRV_RGB_LED_DMA_START_LOW_SAMPLES; sample++)
    {
        s_drv_rgb_led_dma_waveform[word_index] = s_drv_rgb_led_dma_low_word;
        word_index++;
    }

    for (bit = 0U; bit < bit_count; bit++)
    {
        word_index = drv_rgb_led_append_waveform_bit(((bit & 1U) == 0U), word_index);
    }

    return word_index;
}

static void drv_rgb_led_dma_timer_init(void)
{
    TIM_TimeBaseInitTypeDef tim_init;

    RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);

    TIM_Cmd(DRV_RGB_LED_DMA_TIMER, DISABLE);
    TIM_StructInit(&tim_init);
    tim_init.TIM_Mode = TIM_Mode_UserDefine;
    tim_init.TIM_PWM_En = PWM_ENABLE;
    tim_init.TIM_PWM_High_Count = DRV_RGB_LED_DMA_SAMPLE_TICKS - 1U;
    tim_init.TIM_PWM_Low_Count = DRV_RGB_LED_DMA_SAMPLE_TICKS - 1U;
    TIM_TimeBaseInit(DRV_RGB_LED_DMA_TIMER, &tim_init);
}

static drv_rgb_led_status_t drv_rgb_led_dma_send_waveform(uint16_t word_count)
{
    GDMA_InitTypeDef gdma_init;
    uint32_t start_tick;
    uint32_t timeout_ticks;
    uint32_t last_tick;
    uint32_t stalled_reads = 0U;

    if ((word_count == 0U) || (word_count > DRV_RGB_LED_DMA_MAX_WORDS))
    {
        return DRV_RGB_LED_STATUS_INVALID_PARAM;
    }

    if (GDMA_GetChannelStatus(DRV_RGB_LED_DMA_CHANNEL_NUM) == SET)
    {
        return DRV_RGB_LED_STATUS_BUSY;
    }

    GDMA_ClearAllTypeINT(DRV_RGB_LED_DMA_CHANNEL_NUM);

    GDMA_StructInit(&gdma_init);
    gdma_init.GDMA_ChannelNum = DRV_RGB_LED_DMA_CHANNEL_NUM;
    gdma_init.GDMA_DIR = GDMA_DIR_MemoryToPeripheral;
    gdma_init.GDMA_BufferSize = word_count;
    gdma_init.GDMA_SourceInc = DMA_SourceInc_Inc;
    gdma_init.GDMA_DestinationInc = DMA_DestinationInc_Fix;
    gdma_init.GDMA_SourceDataSize = GDMA_DataSize_Word;
    gdma_init.GDMA_DestinationDataSize = GDMA_DataSize_Word;
    gdma_init.GDMA_SourceMsize = GDMA_Msize_1;
    gdma_init.GDMA_DestinationMsize = GDMA_Msize_1;
    gdma_init.GDMA_SourceAddr = (uint32_t)s_drv_rgb_led_dma_waveform;
    gdma_init.GDMA_DestinationAddr = DRV_RGB_LED_GPIO_DMA_DATA_REG;
    gdma_init.GDMA_DestHandshake = DRV_RGB_LED_DMA_HANDSHAKE;
    gdma_init.GDMA_ChannelPriority = 2U;
    GDMA_Init(DRV_RGB_LED_DMA_CHANNEL, &gdma_init);

    GDMA_Cmd(DRV_RGB_LED_DMA_CHANNEL_NUM, ENABLE);
    TIM_Cmd(DRV_RGB_LED_DMA_TIMER, ENABLE);

    start_tick = drv_rgb_led_tick_now();
    last_tick = start_tick;
    timeout_ticks = DRV_RGB_LED_DMA_TIMEOUT_US * DRV_RGB_LED_VENDOR_TICK_PER_US;

    while (GDMA_GetChannelStatus(DRV_RGB_LED_DMA_CHANNEL_NUM) == SET)
    {
        uint32_t now = drv_rgb_led_tick_now();

        if (now == last_tick)
        {
            stalled_reads++;
            if (stalled_reads >= DRV_RGB_LED_TICK_PROBE_WAIT)
            {
                drv_rgb_led_dma_stop();
                return DRV_RGB_LED_STATUS_TICK_STUCK;
            }
        }
        else
        {
            last_tick = now;
            stalled_reads = 0U;
        }
        if (drv_rgb_led_tick_delta(start_tick, now) > timeout_ticks)
        {
            drv_rgb_led_dma_stop();
            return DRV_RGB_LED_STATUS_TIMEOUT;
        }
    }

    TIM_Cmd(DRV_RGB_LED_DMA_TIMER, DISABLE);

    if ((GDMA_BASE->STATUS_ERR & BIT(DRV_RGB_LED_DMA_CHANNEL_NUM)) != 0U)
    {
        GDMA_ClearAllTypeINT(DRV_RGB_LED_DMA_CHANNEL_NUM);
        drv_rgb_led_dma_gpio_low();
        return DRV_RGB_LED_STATUS_DMA_ERROR;
    }

    GDMA_ClearAllTypeINT(DRV_RGB_LED_DMA_CHANNEL_NUM);
    drv_rgb_led_dma_gpio_low();
    return DRV_RGB_LED_STATUS_OK;
}

static void drv_rgb_led_data_pad_sw_low(void)
{
    const PAD_OUTPUT_VAL mcu_low_level = drv_rgb_led_mcu_pad_level_for_led_level(false);

    if (s_drv_rgb_led_ready)
    {
        drv_rgb_led_dma_stop();
    }
    Pad_Config(ZY100_RGB_LED_DATA_PIN, PAD_SW_MODE, PAD_IS_PWRON, drv_rgb_led_data_pad_pull(),
               PAD_OUT_ENABLE, mcu_low_level);
    Pinmux_Deinit(ZY100_RGB_LED_DATA_PIN);
    s_drv_rgb_led_ready = false;
}

drv_rgb_led_status_t drv_rgb_led_init(void)
{
    GPIO_InitTypeDef gpio_init;

    if (!drv_rgb_led_data_pin_valid())
    {
        s_drv_rgb_led_ready = false;
        return DRV_RGB_LED_STATUS_INVALID_PIN;
    }
    if (!drv_rgb_led_clock_valid())
    {
        s_drv_rgb_led_ready = false;
        drv_rgb_led_data_pad_sw_low();
        return DRV_RGB_LED_STATUS_CLOCK_ERROR;
    }

    s_drv_rgb_led_gpio_pin = GPIO_GetPin(ZY100_RGB_LED_DATA_PIN);
#if (ZY100_RGB_LED_DATA_ACTIVE_LOW != 0U)
    s_drv_rgb_led_dma_low_word = s_drv_rgb_led_gpio_pin;
    s_drv_rgb_led_dma_high_word = 0U;
#else
    s_drv_rgb_led_dma_low_word = 0U;
    s_drv_rgb_led_dma_high_word = s_drv_rgb_led_gpio_pin;
#endif

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    Pad_Config(ZY100_RGB_LED_DATA_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, drv_rgb_led_data_pad_pull(),
               PAD_OUT_ENABLE, drv_rgb_led_mcu_pad_level_for_led_level(false));
    Pinmux_Deinit(ZY100_RGB_LED_DATA_PIN);
    Pinmux_Config(ZY100_RGB_LED_DATA_PIN, DWGPIO);

    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = s_drv_rgb_led_gpio_pin;
    gpio_init.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init.GPIO_ITCmd = DISABLE;
    gpio_init.GPIO_ControlMode = GPIO_HARDWARE_MODE;
    GPIO_Init(&gpio_init);
    drv_rgb_led_dma_gpio_low();
    drv_rgb_led_dma_timer_init();
    RCC_PeriphClockCmd(APBPeriph_GDMA, APBPeriph_GDMA_CLOCK, ENABLE);
    /*
     * Sleep may preserve a stale TIM5/GDMA status even though the software
     * ready flag was cleared.  Always force the waveform backend idle before
     * accepting a new frame.
     */
    drv_rgb_led_dma_stop();

    s_drv_rgb_led_ready = true;
    (void)drv_rgb_led_set_timing_ticks(DRV_RGB_LED_T0H_TICKS_DEFAULT,
                                       DRV_RGB_LED_T0L_TICKS_DEFAULT,
                                       DRV_RGB_LED_T1H_TICKS_DEFAULT,
                                       DRV_RGB_LED_T1L_TICKS_DEFAULT);
    return DRV_RGB_LED_STATUS_OK;
}

drv_rgb_led_status_t drv_rgb_led_set_timing_ticks(uint32_t t0h_ticks,
                                                  uint32_t t0l_ticks,
                                                  uint32_t t1h_ticks,
                                                  uint32_t t1l_ticks)
{
    (void)t0l_ticks;
    (void)t1l_ticks;

    if ((t0h_ticks == 0U) || (t1h_ticks == 0U))
    {
        return DRV_RGB_LED_STATUS_INVALID_PARAM;
    }

    return DRV_RGB_LED_STATUS_OK;
}

void drv_rgb_led_data_low(void)
{
    drv_rgb_led_data_pad_sw_low();
}

DATA_RAM_FUNCTION
drv_rgb_led_status_t drv_rgb_led_show(const zy100_rgb_color_t *colors, uint16_t count)
{
    uint16_t word_count;
    drv_rgb_led_status_t status;

    if ((colors == NULL) || (count == 0U) || (count > ZY100_RGB_LED_COUNT))
    {
        return DRV_RGB_LED_STATUS_INVALID_PARAM;
    }

    /*
     * Board power/sleep helpers may leave the data pad in a software safe
     * state. Re-arm the GPIO hardware path before every frame so TIM+GDMA
     * owns the waveform output.
     */
    status = drv_rgb_led_init();
    if (status != DRV_RGB_LED_STATUS_OK)
    {
        return status;
    }

    if (!drv_rgb_led_tick_running())
    {
        return DRV_RGB_LED_STATUS_TICK_STUCK;
    }

    word_count = drv_rgb_led_build_dma_waveform(colors, count);
    drv_rgb_led_dma_gpio_low();
    if (!drv_rgb_led_wait_us_raw(DRV_RGB_LED_FRAME_GAP_US))
    {
        drv_rgb_led_data_low();
        return DRV_RGB_LED_STATUS_TICK_STUCK;
    }

    status = drv_rgb_led_dma_send_waveform(word_count);
    if (status != DRV_RGB_LED_STATUS_OK)
    {
        drv_rgb_led_data_low();
        return status;
    }

    drv_rgb_led_data_low();
    if (!drv_rgb_led_wait_us_raw(DRV_RGB_LED_RESET_LATCH_US))
    {
        return DRV_RGB_LED_STATUS_TICK_STUCK;
    }
    return DRV_RGB_LED_STATUS_OK;
}

DATA_RAM_FUNCTION
drv_rgb_led_status_t drv_rgb_led_send_edge_test(uint8_t edge_count)
{
    uint16_t word_count;
    uint16_t bit_count;
    drv_rgb_led_status_t status;

    if ((edge_count == 0U) ||
        ((edge_count % DRV_RGB_LED_EDGE_TEST_EDGES_PER_BIT) != 0U))
    {
        return DRV_RGB_LED_STATUS_INVALID_PARAM;
    }

    bit_count = (uint16_t)(edge_count / DRV_RGB_LED_EDGE_TEST_EDGES_PER_BIT);
    if ((DRV_RGB_LED_DMA_START_LOW_SAMPLES +
         (bit_count * DRV_RGB_LED_DMA_SAMPLES_PER_BIT)) > DRV_RGB_LED_DMA_MAX_WORDS)
    {
        return DRV_RGB_LED_STATUS_INVALID_PARAM;
    }

    status = drv_rgb_led_init();
    if (status != DRV_RGB_LED_STATUS_OK)
    {
        return status;
    }

    if (!drv_rgb_led_tick_running())
    {
        return DRV_RGB_LED_STATUS_TICK_STUCK;
    }

    word_count = drv_rgb_led_build_edge_test_waveform(edge_count);
    drv_rgb_led_dma_gpio_low();
    if (!drv_rgb_led_wait_us_raw(DRV_RGB_LED_FRAME_GAP_US))
    {
        drv_rgb_led_data_low();
        return DRV_RGB_LED_STATUS_TICK_STUCK;
    }

    status = drv_rgb_led_dma_send_waveform(word_count);
    if (status != DRV_RGB_LED_STATUS_OK)
    {
        drv_rgb_led_data_low();
        return status;
    }

    drv_rgb_led_data_low();
    if (!drv_rgb_led_wait_us_raw(DRV_RGB_LED_RESET_LATCH_US))
    {
        return DRV_RGB_LED_STATUS_TICK_STUCK;
    }
    return DRV_RGB_LED_STATUS_OK;
}

drv_rgb_led_status_t drv_rgb_led_all_off(uint16_t count)
{
    zy100_rgb_color_t colors[ZY100_RGB_LED_COUNT];
    uint16_t led;

    if ((count == 0U) || (count > ZY100_RGB_LED_COUNT))
    {
        return DRV_RGB_LED_STATUS_INVALID_PARAM;
    }

    for (led = 0U; led < ZY100_RGB_LED_COUNT; led++)
    {
        colors[led].red = 0U;
        colors[led].green = 0U;
        colors[led].blue = 0U;
    }

    return drv_rgb_led_show(colors, count);
}

uint32_t drv_rgb_led_gpio_mask(void)
{
    return s_drv_rgb_led_gpio_pin;
}

const char *drv_rgb_led_status_name(drv_rgb_led_status_t status)
{
    switch (status)
    {
    case DRV_RGB_LED_STATUS_OK:
        return "ok";
    case DRV_RGB_LED_STATUS_INVALID_PARAM:
        return "invalid_param";
    case DRV_RGB_LED_STATUS_INVALID_PIN:
        return "invalid_pin";
    case DRV_RGB_LED_STATUS_NOT_READY:
        return "not_ready";
    case DRV_RGB_LED_STATUS_TICK_STUCK:
        return "tick_stuck";
    case DRV_RGB_LED_STATUS_BUSY:
        return "busy";
    case DRV_RGB_LED_STATUS_TIMEOUT:
        return "timeout";
    case DRV_RGB_LED_STATUS_DMA_ERROR:
        return "dma_error";
    case DRV_RGB_LED_STATUS_CLOCK_ERROR:
        return "clock_error";
    default:
        return "unknown";
    }
}
