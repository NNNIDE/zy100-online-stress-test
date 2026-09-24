#include "zy100_rt_marker_timer.h"

#include <stddef.h>

#include "os_sync.h"
#include "rtl876x_nvic.h"
#include "rtl876x_rcc.h"
#include "rtl876x_tim.h"

#include "../app_flags.h"
#include "../bsp/imu_bsp.h"
#include "imu_rt_marker.h"

#define ZY100_RT_MARKER_TIMER_NUM             TIM6
#define ZY100_RT_MARKER_TIMER_IRQn            TIMER6_IRQn
#define ZY100_RT_MARKER_TIMER_CLK_PER_US      ZY100_FIXED_TIMER_CLOCK_PER_US
#define ZY100_RT_MARKER_TIMER_NVIC_PRIORITY   3U

typedef struct
{
    bool configured;
    bool running;
    uint32_t period_ticks;
    uint32_t timer_due_count;
    uint32_t timer_notify_count;
    zy100_rt_marker_timer_notify_cb_t notify_cb;
} zy100_rt_marker_timer_state_t;

static zy100_rt_marker_timer_state_t s_rt_timer;

static imu_status_t zy100_rt_marker_timer_config_ticks(uint32_t period_ticks)
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
    RCC_TimerClockConfig(ZY100_RT_MARKER_TIMER_NUM, DISABLE);

    TIM_Cmd(ZY100_RT_MARKER_TIMER_NUM, DISABLE);
    TIM_INTConfig(ZY100_RT_MARKER_TIMER_NUM, DISABLE);

    TIM_StructInit(&tim_init);
    tim_init.TIM_PWM_En = PWM_DISABLE;
    tim_init.TIM_Mode = TIM_Mode_UserDefine;
    tim_init.TIM_Period = period_ticks - 1U;
    TIM_TimeBaseInit(ZY100_RT_MARKER_TIMER_NUM, &tim_init);

    nvic_init.NVIC_IRQChannel = ZY100_RT_MARKER_TIMER_IRQn;
    nvic_init.NVIC_IRQChannelPriority = ZY100_RT_MARKER_TIMER_NVIC_PRIORITY;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    TIM_ClearINT(ZY100_RT_MARKER_TIMER_NUM);
    TIM_INTConfig(ZY100_RT_MARKER_TIMER_NUM, ENABLE);

    s_rt_timer.configured = true;
    s_rt_timer.running = false;
    s_rt_timer.period_ticks = period_ticks;
    s_rt_timer.timer_due_count = 0U;
    s_rt_timer.timer_notify_count = 0U;
    return IMU_STATUS_OK;
}

imu_status_t zy100_rt_marker_timer_config(uint32_t target_hz)
{
    uint32_t period_us;

    if (target_hz == 0U)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    period_us = 1000000U / target_hz;
    if ((period_us == 0U) ||
        (period_us > (UINT32_MAX / ZY100_RT_MARKER_TIMER_CLK_PER_US)))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    return zy100_rt_marker_timer_config_ticks(period_us *
                                             ZY100_RT_MARKER_TIMER_CLK_PER_US);
}

imu_status_t zy100_rt_marker_timer_start(void)
{
#if ZY100_RT_MARKER_TIMER_ENABLE
    imu_status_t status;

    status = imu_bsp_init();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    if (!s_rt_timer.configured)
    {
        return IMU_STATUS_NOT_READY;
    }

    TIM_ClearINT(ZY100_RT_MARKER_TIMER_NUM);
    TIM_Cmd(ZY100_RT_MARKER_TIMER_NUM, ENABLE);
    s_rt_timer.running = true;
    return IMU_STATUS_OK;
#else
    return IMU_STATUS_NOT_READY;
#endif
}

void zy100_rt_marker_timer_stop(void)
{
    TIM_Cmd(ZY100_RT_MARKER_TIMER_NUM, DISABLE);
    TIM_ClearINT(ZY100_RT_MARKER_TIMER_NUM);
    s_rt_timer.running = false;
}

bool zy100_rt_marker_timer_is_running(void)
{
    return s_rt_timer.running;
}

void zy100_rt_marker_timer_reset_counters(void)
{
    uint32_t lock_state;

    TIM_ClearINT(ZY100_RT_MARKER_TIMER_NUM);

    lock_state = os_lock();
    s_rt_timer.timer_due_count = 0U;
    s_rt_timer.timer_notify_count = 0U;
    os_unlock(lock_state);
}

void zy100_rt_marker_timer_register_notify_callback(zy100_rt_marker_timer_notify_cb_t cb)
{
    s_rt_timer.notify_cb = cb;
}

void zy100_rt_marker_timer_get_stats(zy100_rt_marker_timer_stats_t *out)
{
    uint32_t lock_state;

    if (out == NULL)
    {
        return;
    }

    lock_state = os_lock();
    out->timer_due_count = s_rt_timer.timer_due_count;
    out->timer_notify_count = s_rt_timer.timer_notify_count;
    os_unlock(lock_state);
}

void Timer6_Handler(void)
{
    uint32_t due_seq;
    zy100_rt_marker_timer_notify_cb_t notify_cb;

    TIM_ClearINT(ZY100_RT_MARKER_TIMER_NUM);

    if (!s_rt_timer.running)
    {
        return;
    }

    if (s_rt_timer.timer_due_count < 0xFFFFFFFFU)
    {
        s_rt_timer.timer_due_count++;
    }
    due_seq = s_rt_timer.timer_due_count;
    imu_rt_marker_timer_due_isr_hook(due_seq);

    notify_cb = s_rt_timer.notify_cb;
    if (notify_cb != NULL)
    {
        if (s_rt_timer.timer_notify_count < 0xFFFFFFFFU)
        {
            s_rt_timer.timer_notify_count++;
        }
        notify_cb();
    }
}
