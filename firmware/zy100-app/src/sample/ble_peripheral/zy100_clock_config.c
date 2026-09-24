#include "zy100_clock_config.h"

#include "app_flags.h"
#include "bsp/imu_board_pinmap.h"
#include "os_sched.h"
#include "rtl876x_lib_platform.h"
#include "rtl876x_rcc.h"
#include "service/zy100_rt_marker_timer.h"
#include "system_rtl876x.h"
#include "trace.h"

#define ZY100_CLOCK_SPI_FIXED_SOURCE_HZ        40000000U
#define ZY100_CLOCK_VENDOR_TICK_MASK           0x03FFFFFFU
#define ZY100_CLOCK_OS_DELTA_MIN_MS            900U
#define ZY100_CLOCK_OS_DELTA_MAX_MS            1100U
#define ZY100_CLOCK_TIM6_DUE_TARGET            ZY100_RT_MARKER_TARGET_HZ
#define ZY100_CLOCK_TIM6_DUE_OK_MARGIN         10U
#define ZY100_CLOCK_TIM6_DUE_ERR_MARGIN        10U
#define ZY100_CLOCK_TIM6_DUE_OK_MIN            (ZY100_CLOCK_TIM6_DUE_TARGET - ZY100_CLOCK_TIM6_DUE_OK_MARGIN)
#define ZY100_CLOCK_TIM6_DUE_OK_MAX            (ZY100_CLOCK_TIM6_DUE_TARGET + ZY100_CLOCK_TIM6_DUE_OK_MARGIN)
#define ZY100_CLOCK_TIM6_DUE_CLASS_OK          0U
#define ZY100_CLOCK_TIM6_DUE_CLASS_WARN        1U
#define ZY100_CLOCK_TIM6_DUE_CLASS_ERR         2U

#if (ZY100_FORCE_SYSTEM_80MHZ != 0) && (ZY100_SYSTEM_CLOCK_HZ != 80000000U)
#error "ZY100_FORCE_SYSTEM_80MHZ requires ZY100_SYSTEM_CLOCK_HZ to be 80000000U"
#endif

typedef struct
{
    bool baseline_valid;
    bool cpu_clock_apply_ok;
    uint8_t last_tim6_due_class;
    uint32_t last_runtime_ms;
    uint32_t last_tim6_due;
} zy100_clock_check_state_t;

static zy100_clock_check_state_t s_clock_check;

uint32_t zy100_os_time_ms(void)
{
    return (uint32_t)os_sys_time_get();
}

static uint32_t zy100_clock_spi_divisor(uint16_t clock_div)
{
    switch (clock_div)
    {
    case SPI_CLOCK_DIV_1:
        return 1U;
    case SPI_CLOCK_DIV_2:
        return 2U;
    case SPI_CLOCK_DIV_4:
        return 4U;
    case SPI_CLOCK_DIV_8:
        return 8U;
    default:
        return 0U;
    }
}

static uint32_t zy100_clock_spi_target_hz(void)
{
    uint32_t clk_divisor = zy100_clock_spi_divisor(SPI_CLOCK_DIV_1);

    if ((clk_divisor == 0U) || (IMU_SPI_BAUD_PRESCALER == 0U))
    {
        return 0U;
    }

    return ZY100_CLOCK_SPI_FIXED_SOURCE_HZ / clk_divisor / IMU_SPI_BAUD_PRESCALER;
}

static uint32_t zy100_clock_vendor_tick_delta(uint32_t start_tick, uint32_t end_tick)
{
    return (end_tick - start_tick) & ZY100_CLOCK_VENDOR_TICK_MASK;
}

static bool zy100_clock_os_delta_ok(uint32_t os_delta_ms)
{
    return (os_delta_ms >= ZY100_CLOCK_OS_DELTA_MIN_MS) &&
           (os_delta_ms <= ZY100_CLOCK_OS_DELTA_MAX_MS);
}

static bool zy100_clock_tim6_due_ok(uint32_t tim6_due_delta)
{
    return (tim6_due_delta >= ZY100_CLOCK_TIM6_DUE_OK_MIN) &&
           (tim6_due_delta <= ZY100_CLOCK_TIM6_DUE_OK_MAX);
}

static bool zy100_clock_value_near(uint32_t value, uint32_t target, uint32_t margin)
{
    uint32_t min_value;
    uint32_t max_value;

    min_value = (target > margin) ? (target - margin) : 0U;
    max_value = target + margin;
    return (value >= min_value) && (value <= max_value);
}

static bool zy100_clock_tim6_due_err(uint32_t tim6_due_delta)
{
    uint32_t half_target = ZY100_CLOCK_TIM6_DUE_TARGET / 2U;
    uint32_t double_target = ZY100_CLOCK_TIM6_DUE_TARGET * 2U;

    return zy100_clock_value_near(tim6_due_delta,
                                  half_target,
                                  ZY100_CLOCK_TIM6_DUE_ERR_MARGIN) ||
           zy100_clock_value_near(tim6_due_delta,
                                  double_target,
                                  ZY100_CLOCK_TIM6_DUE_ERR_MARGIN);
}

static uint32_t zy100_clock_measure_os_1s(void)
{
    uint32_t start_ms = zy100_os_time_ms();
    uint32_t now_ms = start_ms;

    while ((uint32_t)(now_ms - start_ms) < ZY100_CLOCK_CHECK_PERIOD_MS)
    {
        os_delay(1U);
        now_ms = zy100_os_time_ms();
    }

    ZY100_LOG_VERBOSE("[CLK_CHECK] os_1s_ms=%u", (uint32_t)(now_ms - start_ms));
    return (uint32_t)(now_ms - start_ms);
}

static void zy100_clock_config_log_bus_targets(void)
{
    uint32_t spi_hz = zy100_clock_spi_target_hz();
    uint32_t spi_divisor = zy100_clock_spi_divisor(SPI_CLOCK_DIV_1);

    ZY100_LOG_KEY("[CLK_CFG] spi_hz=%u", spi_hz);
#if ZY100_CLK_CHECK_VERBOSE_ENABLE
    DBG_DIRECT("[CLK_CHECK] spi_source_hz=%u spi_clk_div=%u spi_baud_prescaler=%u",
               ZY100_CLOCK_SPI_FIXED_SOURCE_HZ,
               spi_divisor,
               (uint32_t)IMU_SPI_BAUD_PRESCALER);
#else
    (void)spi_divisor;
#endif
}

static bool zy100_clock_spi_cfg_ok(void)
{
    return zy100_clock_spi_target_hz() != 0U;
}

static void zy100_clock_config_reset_check_baseline(void)
{
    s_clock_check.baseline_valid = false;
    s_clock_check.last_tim6_due_class = ZY100_CLOCK_TIM6_DUE_CLASS_OK;
    s_clock_check.last_runtime_ms = zy100_os_time_ms();
    s_clock_check.last_tim6_due = 0U;
}

void zy100_clock_config_reset_poll_baseline(void)
{
    zy100_clock_config_reset_check_baseline();
}

bool zy100_clock_config_apply_active_no_os_check(void)
{
    bool api_set_ok = true;
    bool active_ok;
    uint32_t cpu_clock_hz;

#if ZY100_FORCE_SYSTEM_80MHZ
    api_set_ok = set_system_clock(SYSTEM_80MHZ);
#endif

    cpu_clock_hz = get_cpu_clock();
    active_ok = (api_set_ok != false) && (cpu_clock_hz == ZY100_SYSTEM_CLOCK_HZ);
    s_clock_check.cpu_clock_apply_ok = active_ok;

    DBG_DIRECT("[CLK_CFG] pre_os target=%u set_ok=%u cpu_clock_hz=%u ok=%u",
               (uint32_t)ZY100_SYSTEM_CLOCK_HZ,
               api_set_ok ? 1U : 0U,
               cpu_clock_hz,
               active_ok ? 1U : 0U);
    if (!active_ok)
    {
        DBG_DIRECT("[CLK_CFG][ERR] pre_os cpu_clock_hz=%u target=%u",
                   cpu_clock_hz,
                   (uint32_t)ZY100_SYSTEM_CLOCK_HZ);
    }

    return active_ok;
}

bool zy100_clock_config_apply_active(void)
{
    bool api_set_ok = true;
    bool active_ok;
    bool os_tick_ok = true;
    bool spi_cfg_ok;
    uint32_t cpu_clock_hz;
    uint32_t os_delta_ms = ZY100_CLOCK_CHECK_PERIOD_MS;

#if ZY100_FORCE_SYSTEM_80MHZ
    api_set_ok = set_system_clock(SYSTEM_80MHZ);
#endif

    cpu_clock_hz = get_cpu_clock();
    active_ok = (api_set_ok != false) && (cpu_clock_hz == ZY100_SYSTEM_CLOCK_HZ);
    s_clock_check.cpu_clock_apply_ok = active_ok;

    ZY100_LOG_DETAIL("[CLK_CFG] target=%u set_ok=%u cpu_clock_hz=%u ok=%u",
               (uint32_t)ZY100_SYSTEM_CLOCK_HZ,
               api_set_ok ? 1U : 0U,
               cpu_clock_hz,
               active_ok ? 1U : 0U);
    if (!active_ok)
    {
        DBG_DIRECT("[CLK_CFG][ERR] cpu_clock_hz=%u target=%u",
                   cpu_clock_hz,
                   (uint32_t)ZY100_SYSTEM_CLOCK_HZ);
    }
    zy100_clock_config_log_bus_targets();
    os_delta_ms = zy100_clock_measure_os_1s();
    os_tick_ok = zy100_clock_os_delta_ok(os_delta_ms);
    spi_cfg_ok = zy100_clock_spi_cfg_ok();
#if ZY100_CLK_CHECK_VERBOSE_ENABLE
    DBG_DIRECT("[CLK_HEALTH] cpu_clock_apply_ok=%u os_tick_ok=%u tim6_due_ok=%u spi_cfg_ok=%u",
               active_ok ? 1U : 0U,
               os_tick_ok ? 1U : 0U,
               1U,
               spi_cfg_ok ? 1U : 0U);
#endif
    if (!os_tick_ok)
    {
        DBG_DIRECT("[CLK_CHECK][WARN] os_tick_delta_bad os_1s_ms=%u", os_delta_ms);
    }
    if (!spi_cfg_ok)
    {
        DBG_DIRECT("[CLK_CHECK][ERR] spi_cfg_bad");
    }
    ZY100_LOG_DETAIL("[CLK_CHECK] apply_ret active=%u os=%u spi=%u ok=%u",
               active_ok ? 1U : 0U,
               os_tick_ok ? 1U : 0U,
               spi_cfg_ok ? 1U : 0U,
               (active_ok && os_tick_ok && spi_cfg_ok) ? 1U : 0U);
    zy100_clock_config_reset_check_baseline();
    return active_ok && os_tick_ok && spi_cfg_ok;
}

bool zy100_clock_config_poll_check(uint32_t runtime_ms)
{
    zy100_rt_marker_timer_stats_t tim6_stats;
    uint32_t os_delta_ms;
    uint32_t tim6_due_delta;
    bool os_tick_ok;
    bool tim6_due_ok = true;
    bool tim6_due_runtime_ok = true;
    bool tim6_running;
    bool spi_cfg_ok;
    uint8_t tim6_due_class = ZY100_CLOCK_TIM6_DUE_CLASS_OK;

    zy100_rt_marker_timer_get_stats(&tim6_stats);

    if (!s_clock_check.baseline_valid)
    {
        s_clock_check.baseline_valid = true;
        s_clock_check.last_runtime_ms = runtime_ms;
        s_clock_check.last_tim6_due = tim6_stats.timer_due_count;
        return true;
    }

    os_delta_ms = runtime_ms - s_clock_check.last_runtime_ms;
    if (os_delta_ms < ZY100_CLOCK_CHECK_PERIOD_MS)
    {
        return true;
    }

    tim6_due_delta = tim6_stats.timer_due_count - s_clock_check.last_tim6_due;
    os_tick_ok = zy100_clock_os_delta_ok(os_delta_ms);
    tim6_running = zy100_rt_marker_timer_is_running();
    spi_cfg_ok = zy100_clock_spi_cfg_ok();

    if (tim6_running)
    {
        tim6_due_ok = zy100_clock_tim6_due_ok(tim6_due_delta);
        if (!tim6_due_ok)
        {
            if (zy100_clock_tim6_due_err(tim6_due_delta))
            {
                tim6_due_class = ZY100_CLOCK_TIM6_DUE_CLASS_ERR;
                tim6_due_runtime_ok = false;
            }
            else
            {
                tim6_due_class = ZY100_CLOCK_TIM6_DUE_CLASS_WARN;
            }
        }
#if ZY100_CLK_CHECK_VERBOSE_ENABLE
        DBG_DIRECT("[CLK_CHECK] tim6_target_hz=%u tim6_due_1s=%u",
                   (uint32_t)ZY100_CLOCK_TIM6_DUE_TARGET,
                   tim6_due_delta);
#endif
    }
    else
    {
#if ZY100_CLK_CHECK_VERBOSE_ENABLE
        DBG_DIRECT("[CLK_CHECK] tim6=not_running");
#endif
    }
#if ZY100_CLK_CHECK_VERBOSE_ENABLE
    DBG_DIRECT("[CLK_CHECK] os_1s_ms=%u", os_delta_ms);
#endif
#if ZY100_CLK_CHECK_VERBOSE_ENABLE
    DBG_DIRECT("[CLK_HEALTH] cpu_clock_apply_ok=%u os_tick_ok=%u tim6_due_ok=%u spi_cfg_ok=%u",
               s_clock_check.cpu_clock_apply_ok ? 1U : 0U,
               os_tick_ok ? 1U : 0U,
               tim6_due_ok ? 1U : 0U,
               spi_cfg_ok ? 1U : 0U);
#endif
    if (!s_clock_check.cpu_clock_apply_ok)
    {
        DBG_DIRECT("[CLK_CHECK][ERR] cpu_clock_apply_bad");
    }
    if (!os_tick_ok)
    {
        DBG_DIRECT("[CLK_CHECK][WARN] os_tick_delta_bad os_1s_ms=%u", os_delta_ms);
    }
    if (tim6_due_class != s_clock_check.last_tim6_due_class)
    {
        if (tim6_due_class == ZY100_CLOCK_TIM6_DUE_CLASS_ERR)
        {
            DBG_DIRECT("[CLK_CHECK][ERR] tim6_due_bad due=%u expected=%u",
                       tim6_due_delta,
                       (uint32_t)ZY100_CLOCK_TIM6_DUE_TARGET);
        }
        else if (tim6_due_class == ZY100_CLOCK_TIM6_DUE_CLASS_WARN)
        {
            DBG_DIRECT("[CLK_CHECK][WARN] tim6_due_drift due=%u expected=%u",
                       tim6_due_delta,
                       (uint32_t)ZY100_CLOCK_TIM6_DUE_TARGET);
        }
    }
    s_clock_check.last_tim6_due_class = tim6_due_class;
    if (!spi_cfg_ok)
    {
        DBG_DIRECT("[CLK_CHECK][ERR] spi_cfg_bad");
    }

    s_clock_check.last_runtime_ms = runtime_ms;
    s_clock_check.last_tim6_due = tim6_stats.timer_due_count;
    return s_clock_check.cpu_clock_apply_ok &&
           os_tick_ok &&
           tim6_due_runtime_ok &&
           spi_cfg_ok;
}

uint32_t zy100_clock_config_vendor_tick_hz(void)
{
    return (uint32_t)ZY100_VENDOR_TICK_HZ;
}

uint32_t zy100_clock_config_vendor_ticks_to_us(uint32_t ticks)
{
    uint32_t tick_hz = zy100_clock_config_vendor_tick_hz();

    if (tick_hz == 0U)
    {
        tick_hz = ZY100_VENDOR_TICK_HZ;
    }
    if (tick_hz == 0U)
    {
        return 0U;
    }

    return (uint32_t)(((uint64_t)ticks * 1000000ULL) /
                      (uint64_t)tick_hz);
}

uint32_t zy100_clock_config_vendor_tick_delta_us(uint32_t start_tick, uint32_t end_tick)
{
    return zy100_clock_config_vendor_ticks_to_us(
               zy100_clock_vendor_tick_delta(start_tick, end_tick));
}

uint64_t zy100_clock_config_monotonic_us(void)
{
    return (uint64_t)zy100_os_time_ms() * 1000ULL;
}

uint32_t zy100_clock_config_monotonic_us32(void)
{
    return (uint32_t)zy100_clock_config_monotonic_us();
}
