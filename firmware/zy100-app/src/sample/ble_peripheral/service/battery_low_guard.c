#include "battery_low_guard.h"

#if F_APP_BATTERY_ADC_ENABLE

#include <stddef.h>

#include "battery_adc.h"
#include "battery_policy.h"
#include "../app_task.h"
#include "../bsp/bsp_battery_adc.h"

#include "app_msg.h"
#include "os_sync.h"
#include "rtl876x_lpc.h"
#include "rtl876x_nvic.h"
#include "rtl876x_pinmux.h"
#include "trace.h"

#define BATTERY_LPC_GUARD_PIN              P2_7
#define BATTERY_LPC_GUARD_CHANNEL          LPC_CHANNEL_P2_7
#define BATTERY_LPC_GUARD_IRQ_PRIORITY     3U

#if F_APP_BATTERY_LPC_GUARD_ENABLE
static void battery_low_guard_configure_pad(void)
{
    Pad_Config(BATTERY_LPC_GUARD_PIN,
               PAD_PINMUX_MODE,
               PAD_IS_PWRON,
               PAD_PULL_NONE,
               PAD_OUT_DISABLE,
               PAD_OUT_HIGH);
    Pinmux_Config(BATTERY_LPC_GUARD_PIN, IDLE_MODE);
}
#endif

static void battery_low_guard_park_pin_low_power(void)
{
    bsp_battery_adc_park_low_power();
}

static void battery_low_guard_disable_lpc_hw(void)
{
    LPC_INTConfig(LPC_INT_LPCOMP_AON, DISABLE);
    LPC_INTConfig(LPC_INT_LPCOMP_NV, DISABLE);
    LPC_Cmd(DISABLE);
    NVIC_DisableIRQ(LPCOMP_IRQn);
    NVIC_ClearPendingIRQ(LPCOMP_IRQn);
}

#if F_APP_BATTERY_LPC_GUARD_ENABLE
typedef struct
{
    uint16_t mv;
    uint32_t threshold;
    const char *name;
} battery_lpc_threshold_point_t;

typedef struct
{
    uint16_t target_battery_mv;
    uint16_t target_pin_mv;
    uint16_t requested_pin_mv;
    uint16_t selected_pin_mv;
    uint16_t recover_battery_mv;
    uint16_t recover_pin_mv;
    uint32_t selected_threshold;
    const char *selected_name;
} battery_lpc_guard_config_t;

static const battery_lpc_threshold_point_t s_lpc_thresholds[] =
{
    {80U, LPC_80_mV, "LPC_80_mV"},
    {160U, LPC_160_mV, "LPC_160_mV"},
    {240U, LPC_240_mV, "LPC_240_mV"},
    {320U, LPC_320_mV, "LPC_320_mV"},
    {400U, LPC_400_mV, "LPC_400_mV"},
    {480U, LPC_480_mV, "LPC_480_mV"},
    {560U, LPC_560_mV, "LPC_560_mV"},
    {640U, LPC_640_mV, "LPC_640_mV"},
    {680U, LPC_680_mV, "LPC_680_mV"},
    {720U, LPC_720_mV, "LPC_720_mV"},
    {760U, LPC_760_mV, "LPC_760_mV"},
    {800U, LPC_800_mV, "LPC_800_mV"},
    {840U, LPC_840_mV, "LPC_840_mV"},
    {880U, LPC_880_mV, "LPC_880_mV"},
    {920U, LPC_920_mV, "LPC_920_mV"},
    {960U, LPC_960_mV, "LPC_960_mV"},
    {1000U, LPC_1000_mV, "LPC_1000_mV"},
    {1040U, LPC_1040_mV, "LPC_1040_mV"},
    {1080U, LPC_1080_mV, "LPC_1080_mV"},
    {1120U, LPC_1120_mV, "LPC_1120_mV"},
    {1160U, LPC_1160_mV, "LPC_1160_mV"},
    {1200U, LPC_1200_mV, "LPC_1200_mV"},
    {1240U, LPC_1240_mV, "LPC_1240_mV"},
    {1280U, LPC_1280_mV, "LPC_1280_mV"},
    {1320U, LPC_1320_mV, "LPC_1320_mV"},
    {1360U, LPC_1360_mV, "LPC_1360_mV"},
    {1400U, LPC_1400_mV, "LPC_1400_mV"},
    {1440U, LPC_1440_mV, "LPC_1440_mV"},
    {1480U, LPC_1480_mV, "LPC_1480_mV"},
    {1520U, LPC_1520_mV, "LPC_1520_mV"},
    {1560U, LPC_1560_mV, "LPC_1560_mV"},
    {1600U, LPC_1600_mV, "LPC_1600_mV"},
    {1640U, LPC_1640_mV, "LPC_1640_mV"},
    {1680U, LPC_1680_mV, "LPC_1680_mV"},
    {1720U, LPC_1720_mV, "LPC_1720_mV"},
    {1760U, LPC_1760_mV, "LPC_1760_mV"},
    {1800U, LPC_1800_mV, "LPC_1800_mV"},
    {1840U, LPC_1840_mV, "LPC_1840_mV"},
    {1880U, LPC_1880_mV, "LPC_1880_mV"},
    {1920U, LPC_1920_mV, "LPC_1920_mV"},
    {1960U, LPC_1960_mV, "LPC_1960_mV"},
    {2000U, LPC_2000_mV, "LPC_2000_mV"},
    {2040U, LPC_2040_mV, "LPC_2040_mV"},
    {2080U, LPC_2080_mV, "LPC_2080_mV"},
    {2120U, LPC_2120_mV, "LPC_2120_mV"},
    {2160U, LPC_2160_mV, "LPC_2160_mV"},
    {2200U, LPC_2200_mV, "LPC_2200_mV"},
    {2240U, LPC_2240_mV, "LPC_2240_mV"},
    {2280U, LPC_2280_mV, "LPC_2280_mV"},
    {2320U, LPC_2320_mV, "LPC_2320_mV"},
    {2360U, LPC_2360_mV, "LPC_2360_mV"},
    {2400U, LPC_2400_mV, "LPC_2400_mV"},
    {2440U, LPC_2440_mV, "LPC_2440_mV"},
    {2480U, LPC_2480_mV, "LPC_2480_mV"},
    {2520U, LPC_2520_mV, "LPC_2520_mV"},
    {2560U, LPC_2560_mV, "LPC_2560_mV"},
    {2640U, LPC_2640_mV, "LPC_2640_mV"},
    {2720U, LPC_2720_mV, "LPC_2720_mV"},
    {2800U, LPC_2800_mV, "LPC_2800_mV"},
    {2880U, LPC_2880_mV, "LPC_2880_mV"},
    {2960U, LPC_2960_mV, "LPC_2960_mV"},
    {3040U, LPC_3040_mV, "LPC_3040_mV"},
    {3120U, LPC_3120_mV, "LPC_3120_mV"},
    {3200U, LPC_3200_mV, "LPC_3200_mV"},
};

static bool s_battery_lpc_guard_inited = false;
static volatile bool s_battery_lpc_guard_armed = false;
static volatile bool s_battery_lpc_guard_latched = false;
static battery_lpc_guard_config_t s_battery_lpc_guard_config;
#endif

void battery_low_guard_force_safe_state(const char *reason)
{
    (void)reason;

#if F_APP_BATTERY_LPC_GUARD_ENABLE
    {
        uint32_t lock_state = os_lock();
        s_battery_lpc_guard_armed = false;
        s_battery_lpc_guard_latched = false;
        os_unlock(lock_state);
    }
#endif

    battery_low_guard_disable_lpc_hw();
    battery_low_guard_park_pin_low_power();
}

#if F_APP_BATTERY_LPC_GUARD_ENABLE

static uint16_t battery_low_guard_battery_mv_to_pin_mv(uint16_t battery_mv)
{
    const uint32_t denominator =
        (uint32_t)BATTERY_LPC_GUARD_DIVIDER_R_TOP_OHM +
        (uint32_t)BATTERY_LPC_GUARD_DIVIDER_R_BOTTOM_OHM;
    uint32_t pin_mv;

    if (denominator == 0U)
    {
        return 0U;
    }

    pin_mv = ((uint32_t)battery_mv *
              (uint32_t)BATTERY_LPC_GUARD_DIVIDER_R_BOTTOM_OHM) +
             (denominator / 2U);
    pin_mv /= denominator;
    return (pin_mv > 0xFFFFU) ? 0xFFFFU : (uint16_t)pin_mv;
}

static const battery_lpc_threshold_point_t *battery_low_guard_select_threshold(
    uint16_t requested_pin_mv)
{
    const battery_lpc_threshold_point_t *selected = &s_lpc_thresholds[0];
    uint8_t idx;

#if BATTERY_LPC_GUARD_CONSERVATIVE_MARGIN_MV == 0U
    uint16_t best_delta =
        (requested_pin_mv > selected->mv) ?
        (uint16_t)(requested_pin_mv - selected->mv) :
        (uint16_t)(selected->mv - requested_pin_mv);

    for (idx = 1U;
         idx < (uint8_t)(sizeof(s_lpc_thresholds) / sizeof(s_lpc_thresholds[0]));
         idx++)
    {
        const uint16_t point_mv = s_lpc_thresholds[idx].mv;
        const uint16_t delta =
            (requested_pin_mv > point_mv) ?
            (uint16_t)(requested_pin_mv - point_mv) :
            (uint16_t)(point_mv - requested_pin_mv);

        if (delta < best_delta)
        {
            selected = &s_lpc_thresholds[idx];
            best_delta = delta;
        }
    }
#else
    for (idx = 0U;
         idx < (uint8_t)(sizeof(s_lpc_thresholds) / sizeof(s_lpc_thresholds[0]));
         idx++)
    {
        selected = &s_lpc_thresholds[idx];
        if (s_lpc_thresholds[idx].mv >= requested_pin_mv)
        {
            break;
        }
    }
#endif

    return selected;
}

static void battery_low_guard_refresh_config(void)
{
    const battery_lpc_threshold_point_t *selected;
    uint32_t requested_pin_mv;

    s_battery_lpc_guard_config.target_battery_mv =
        battery_adc_percent_to_voltage_mv((uint8_t)BATTERY_LPC_GUARD_TARGET_PERCENT);
    s_battery_lpc_guard_config.target_pin_mv =
        battery_low_guard_battery_mv_to_pin_mv(
            s_battery_lpc_guard_config.target_battery_mv);
    s_battery_lpc_guard_config.recover_battery_mv =
        battery_adc_percent_to_voltage_mv((uint8_t)APP_BATTERY_RECOVER_PERCENT);
    s_battery_lpc_guard_config.recover_pin_mv =
        battery_low_guard_battery_mv_to_pin_mv(
            s_battery_lpc_guard_config.recover_battery_mv);

    requested_pin_mv = s_battery_lpc_guard_config.target_pin_mv +
                       (uint32_t)BATTERY_LPC_GUARD_CONSERVATIVE_MARGIN_MV;
    if (requested_pin_mv > 0xFFFFU)
    {
        requested_pin_mv = 0xFFFFU;
    }
    s_battery_lpc_guard_config.requested_pin_mv = (uint16_t)requested_pin_mv;

    selected = battery_low_guard_select_threshold(
                   s_battery_lpc_guard_config.requested_pin_mv);
    s_battery_lpc_guard_config.selected_pin_mv = selected->mv;
    s_battery_lpc_guard_config.selected_threshold = selected->threshold;
    s_battery_lpc_guard_config.selected_name = selected->name;
}

static void battery_low_guard_configure_lpc(void)
{
    LPC_InitTypeDef lpc_init;
    NVIC_InitTypeDef nvic_init;

    battery_low_guard_disable_lpc_hw();

    LPC_StructInit(&lpc_init);
    lpc_init.LPC_Channel = BATTERY_LPC_GUARD_CHANNEL;
    lpc_init.LPC_Edge = LPC_Vin_Below_Vth;
    lpc_init.LPC_Threshold = s_battery_lpc_guard_config.selected_threshold;
    LPC_Init(&lpc_init);

    nvic_init.NVIC_IRQChannel = LPCOMP_IRQn;
    nvic_init.NVIC_IRQChannelPriority = BATTERY_LPC_GUARD_IRQ_PRIORITY;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
}

static void battery_low_guard_log_config(const char *phase)
{
    DBG_DIRECT("[BAT_LPC] %s target_pct=%u target_bat_mv=%u pin_mv=%u request_pin_mv=%u threshold=%s threshold_mv=%u channel=%lu pin=%u margin_mv=%u",
               (phase != NULL) ? phase : "config",
               (unsigned int)BATTERY_LPC_GUARD_TARGET_PERCENT,
               (unsigned int)s_battery_lpc_guard_config.target_battery_mv,
               (unsigned int)s_battery_lpc_guard_config.target_pin_mv,
               (unsigned int)s_battery_lpc_guard_config.requested_pin_mv,
               s_battery_lpc_guard_config.selected_name,
               (unsigned int)s_battery_lpc_guard_config.selected_pin_mv,
               (unsigned long)BATTERY_LPC_GUARD_CHANNEL,
               (unsigned int)BATTERY_LPC_GUARD_PIN,
               (unsigned int)BATTERY_LPC_GUARD_CONSERVATIVE_MARGIN_MV);

    if (s_battery_lpc_guard_config.selected_pin_mv >
        s_battery_lpc_guard_config.recover_pin_mv)
    {
        DBG_DIRECT("[BAT_LPC][WARN] threshold_above_recover selected_pin_mv=%u recover_pct=%u recover_bat_mv=%u recover_pin_mv=%u",
                   (unsigned int)s_battery_lpc_guard_config.selected_pin_mv,
                   (unsigned int)APP_BATTERY_RECOVER_PERCENT,
                   (unsigned int)s_battery_lpc_guard_config.recover_battery_mv,
                   (unsigned int)s_battery_lpc_guard_config.recover_pin_mv);
    }
}

bool battery_low_guard_init(void)
{
    if (s_battery_lpc_guard_inited)
    {
        return true;
    }

    battery_low_guard_refresh_config();
    s_battery_lpc_guard_armed = false;
    s_battery_lpc_guard_latched = false;
    s_battery_lpc_guard_inited = true;
    battery_low_guard_log_config("init_deferred");
    return true;
}

bool battery_low_guard_on_capture_start(uint32_t runtime_ms)
{
    uint32_t lock_state;

    if (!battery_low_guard_init())
    {
        return false;
    }

    lock_state = os_lock();
    if (s_battery_lpc_guard_armed || s_battery_lpc_guard_latched)
    {
        os_unlock(lock_state);
        DBG_DIRECT("[BAT_LPC] arm_skip runtime_ms=%lu armed=%u latched=%u",
                   (unsigned long)runtime_ms,
                   s_battery_lpc_guard_armed ? 1U : 0U,
                   s_battery_lpc_guard_latched ? 1U : 0U);
        return false;
    }

    s_battery_lpc_guard_armed = true;
    os_unlock(lock_state);

    battery_low_guard_refresh_config();
    battery_low_guard_configure_pad();
    battery_low_guard_configure_lpc();
    LPC_Cmd(ENABLE);
    NVIC_ClearPendingIRQ(LPCOMP_IRQn);
    LPC_INTConfig(LPC_INT_LPCOMP_NV, ENABLE);

    battery_low_guard_log_config("arm");
    DBG_DIRECT("[BAT_LPC] armed runtime_ms=%lu armed=1 latched=0",
               (unsigned long)runtime_ms);
    return true;
}

void battery_low_guard_on_capture_stop(uint32_t runtime_ms, const char *reason)
{
    bool armed_before;
    bool latched_before;
    uint32_t lock_state;

    if (!s_battery_lpc_guard_inited)
    {
        return;
    }

    lock_state = os_lock();
    armed_before = s_battery_lpc_guard_armed;
    latched_before = s_battery_lpc_guard_latched;
    battery_low_guard_disable_lpc_hw();
    s_battery_lpc_guard_armed = false;
    os_unlock(lock_state);

    battery_low_guard_park_pin_low_power();

    if (armed_before || latched_before)
    {
        DBG_DIRECT("[BAT_LPC] disarm runtime_ms=%lu reason=%s armed_before=%u latched=%u",
                   (unsigned long)runtime_ms,
                   (reason != NULL) ? reason : "unknown",
                   armed_before ? 1U : 0U,
                   latched_before ? 1U : 0U);
    }
}

bool battery_low_guard_take_latched(void)
{
    bool latched;
    uint32_t lock_state = os_lock();

    latched = s_battery_lpc_guard_latched;
    if (latched)
    {
        s_battery_lpc_guard_latched = false;
    }

    os_unlock(lock_state);
    return latched;
}

bool battery_low_guard_is_armed(void)
{
    return s_battery_lpc_guard_armed;
}

bool battery_low_guard_is_latched(void)
{
    return s_battery_lpc_guard_latched;
}

void LPCOMP_Handler(void)
{
    T_IO_MSG msg;

    battery_low_guard_disable_lpc_hw();

    s_battery_lpc_guard_armed = false;
    if (s_battery_lpc_guard_latched)
    {
        return;
    }

    s_battery_lpc_guard_latched = true;
    msg.type = IO_MSG_TYPE_BAT_LPC;
    msg.subtype = 0U;
    msg.u.param = 0U;
    (void)app_task_post_io_msg_from_isr(&msg);
}

#endif /* F_APP_BATTERY_LPC_GUARD_ENABLE */

#endif /* F_APP_BATTERY_ADC_ENABLE */
