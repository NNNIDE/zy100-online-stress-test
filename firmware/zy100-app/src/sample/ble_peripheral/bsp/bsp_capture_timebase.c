#include "bsp_capture_timebase.h"

#include <stddef.h>

#include "rtl876x_rcc.h"
#include "rtl876x_tim.h"

#define BSP_CAPTURE_TIMEBASE_TIMER TIM2

typedef struct
{
    bool configured;
    bool running;
} bsp_capture_timebase_state_t;

static bsp_capture_timebase_state_t s_capture_timebase;

bool bsp_capture_timebase_init(void)
{
    TIM_TimeBaseInitTypeDef tim_init;

    if (s_capture_timebase.running)
    {
        return false;
    }

    RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);
    /* DISABLE selects the fixed 40 MHz timer source on RTL8762D. */
    RCC_TimerClockConfig(BSP_CAPTURE_TIMEBASE_TIMER, DISABLE);

    TIM_Cmd(BSP_CAPTURE_TIMEBASE_TIMER, DISABLE);
    TIM_INTConfig(BSP_CAPTURE_TIMEBASE_TIMER, DISABLE);
    TIM_ClearINT(BSP_CAPTURE_TIMEBASE_TIMER);

    TIM_StructInit(&tim_init);
    tim_init.TIM_SOURCE_DIV_En = ENABLE;
    tim_init.TIM_SOURCE_DIV = TIM_CLOCK_DIVIDER_40;
    tim_init.TIM_Mode = TIM_Mode_FreeRun;
    tim_init.TIM_PWM_En = PWM_DISABLE;
    tim_init.TIM_Period = 0xFFFFFFFFUL;
    tim_init.ClockDepend = DISABLE;
    TIM_TimeBaseInit(BSP_CAPTURE_TIMEBASE_TIMER, &tim_init);

    s_capture_timebase.configured = true;
    return true;
}

bool bsp_capture_timebase_start(uint32_t *counter_out)
{
    if (!s_capture_timebase.configured || s_capture_timebase.running)
    {
        return false;
    }

    TIM_ClearINT(BSP_CAPTURE_TIMEBASE_TIMER);
    TIM_Cmd(BSP_CAPTURE_TIMEBASE_TIMER, ENABLE);
    s_capture_timebase.running = true;
    if (counter_out != NULL)
    {
        *counter_out = TIM_GetCurrentValue(BSP_CAPTURE_TIMEBASE_TIMER);
    }
    return true;
}

bool bsp_capture_timebase_snapshot(uint32_t *counter_out)
{
    if (!s_capture_timebase.running || (counter_out == NULL))
    {
        return false;
    }

    *counter_out = TIM_GetCurrentValue(BSP_CAPTURE_TIMEBASE_TIMER);
    return true;
}

void bsp_capture_timebase_stop(void)
{
    TIM_Cmd(BSP_CAPTURE_TIMEBASE_TIMER, DISABLE);
    TIM_INTConfig(BSP_CAPTURE_TIMEBASE_TIMER, DISABLE);
    TIM_ClearINT(BSP_CAPTURE_TIMEBASE_TIMER);
    s_capture_timebase.running = false;
}

bool bsp_capture_timebase_is_running(void)
{
    return s_capture_timebase.running;
}
