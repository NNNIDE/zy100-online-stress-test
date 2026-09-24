#include "svc_led_pattern.h"

#include <string.h>

#include "trace.h"
#include "os_sync.h"
#include "os_sched.h"

#include "../app_flags.h"
#include "../bsp/bsp_led_power.h"
#if !ZY100_BUILD_FACTORY
#include "zy100_online_reset_trace.h"
#endif

#define SVC_LED_LOGO_START_INDEX       0U
#define SVC_LED_NOTIFY_START_INDEX     ZY100_LED_LOGO_COUNT
#define SVC_LED_POWER_SETTLE_US        1000U
#define SVC_LED_T0H_TICKS              14U
#define SVC_LED_T0L_TICKS              19U
#define SVC_LED_T1H_TICKS              37U
#define SVC_LED_T1L_TICKS              0U
#define SVC_LED_ERROR_STAGE_NONE       0xFFU
#define SVC_LED_H0_DIAG_HOLD_US        1000000U
#define SVC_LED_H0_DIAG_GAP_US         100000U
#define SVC_LED_BRIGHTNESS_MAX_LEVEL   76U
#define SVC_LED_OUTPUT_SCALE_PERCENT    49U
#define SVC_LED_OUTPUT_SCALE_DIVISOR    100U

#ifndef ZY100_LED_PWR_H0_DIAG_ENABLE
#define ZY100_LED_PWR_H0_DIAG_ENABLE   0U
#endif

#if ZY100_LED_LOG_ENABLE
#define SVC_LED_LOG(...)                  DBG_DIRECT(__VA_ARGS__)
#else
#define SVC_LED_LOG(...)                  do { if (0) { DBG_DIRECT(__VA_ARGS__); } } while (0)
#endif

typedef char svc_led_pattern_count_check[
    ((ZY100_LED_LOGO_COUNT + ZY100_LED_NOTIFY_COUNT) == ZY100_RGB_LED_COUNT) ? 1 : -1];
typedef char svc_led_pattern_brightness_max_check[
    (SVC_LED_BRIGHTNESS_MAX_LEVEL <= 255U) ? 1 : -1];
typedef char svc_led_pattern_output_scale_check[
    (SVC_LED_OUTPUT_SCALE_PERCENT <= SVC_LED_OUTPUT_SCALE_DIVISOR) ? 1 : -1];

static zy100_rgb_color_t s_svc_led_frame[ZY100_RGB_LED_COUNT];
#if (ZY100_LED_HW_ENABLE != 0U)
static bool s_svc_led_inited = false;
static bool s_svc_led_init_logged = false;
static bool s_svc_led_power_on_logged = false;
static bool s_svc_led_resume_in_progress = false;
#if (ZY100_LED_PWR_H0_DIAG_ENABLE != 0U)
static bool s_svc_led_h0_diag_done = false;
#endif
static uint8_t s_svc_led_last_error_stage = SVC_LED_ERROR_STAGE_NONE;
static uint32_t s_svc_led_last_error_status = 0xFFFFFFFFUL;
#endif

#if (ZY100_LED_HW_ENABLE != 0U)
typedef enum
{
    SVC_LED_ERROR_STAGE_POWER_INIT = 0U,
    SVC_LED_ERROR_STAGE_RGB_INIT,
    SVC_LED_ERROR_STAGE_TIMING,
    SVC_LED_ERROR_STAGE_POWER_ON,
    SVC_LED_ERROR_STAGE_POWER_LEVEL,
    SVC_LED_ERROR_STAGE_RGB_REINIT,
    SVC_LED_ERROR_STAGE_ALL_OFF,
    SVC_LED_ERROR_STAGE_SHOW,
    SVC_LED_ERROR_STAGE_SETTLE_DELAY,
} svc_led_error_stage_t;

typedef enum
{
    SVC_LED_RESUME_STAGE_NONE = 0U,
    SVC_LED_RESUME_STAGE_DATA_LOW,
    SVC_LED_RESUME_STAGE_POWER_GPIO,
    SVC_LED_RESUME_STAGE_LEVEL_READ,
    SVC_LED_RESUME_STAGE_SETTLE_DELAY,
    SVC_LED_RESUME_STAGE_RGB_INIT,
    SVC_LED_RESUME_STAGE_TIMING,
    SVC_LED_RESUME_STAGE_ALL_OFF,
} svc_led_resume_stage_t;

static const char *svc_led_pattern_error_stage_name(svc_led_error_stage_t stage)
{
    switch (stage)
    {
    case SVC_LED_ERROR_STAGE_POWER_INIT:
        return "power_init";
    case SVC_LED_ERROR_STAGE_RGB_INIT:
        return "rgb_init";
    case SVC_LED_ERROR_STAGE_TIMING:
        return "timing";
    case SVC_LED_ERROR_STAGE_POWER_ON:
        return "power_on";
    case SVC_LED_ERROR_STAGE_POWER_LEVEL:
        return "power_level";
    case SVC_LED_ERROR_STAGE_RGB_REINIT:
        return "rgb_reinit";
    case SVC_LED_ERROR_STAGE_ALL_OFF:
        return "all_off";
    case SVC_LED_ERROR_STAGE_SHOW:
        return "show";
    case SVC_LED_ERROR_STAGE_SETTLE_DELAY:
        return "settle_delay";
    default:
        return "unknown";
    }
}

static void svc_led_pattern_resume_stage_set(svc_led_resume_stage_t stage)
{
    if (!s_svc_led_resume_in_progress)
    {
        return;
    }

#if !ZY100_BUILD_FACTORY
    zy100_power_reset_trace_set_led_stage((zy100_power_led_stage_t)stage);
#endif
}

static void svc_led_pattern_safe_shutdown(void)
{
    drv_rgb_led_data_low();
    bsp_led_power_data_low();
    (void)bsp_led_power_off();
    s_svc_led_inited = false;
}

static void svc_led_pattern_log_error(svc_led_error_stage_t stage,
                                      uint32_t status,
                                      const char *status_name)
{
    if ((s_svc_led_last_error_stage == (uint8_t)stage) &&
        (s_svc_led_last_error_status == status))
    {
        return;
    }

    ZY100_LOG_ERROR("[ERR][LED] stage=%s st=%s(%lu) count=%u timing=%u/%u/%u/%u pwr_out=%u pwr_in=%u pad=%u/%u/%u/%u data_out=%u",
                    svc_led_pattern_error_stage_name(stage),
                    (status_name != NULL) ? status_name : "unknown",
                    (unsigned long)status,
                    ZY100_RGB_LED_COUNT,
                    SVC_LED_T0H_TICKS,
                    SVC_LED_T0L_TICKS,
                    SVC_LED_T1H_TICKS,
                    SVC_LED_T1L_TICKS,
                    bsp_led_power_ctrl_out_level(),
                    bsp_led_power_ctrl_in_level(),
                    bsp_led_power_ctrl_pad_out_level(),
                    bsp_led_power_ctrl_pad_oe_level(),
                    bsp_led_power_ctrl_pad_mode_level(),
                    bsp_led_power_ctrl_pad_pwr_level(),
                    bsp_led_power_data_out_level());
    s_svc_led_last_error_stage = (uint8_t)stage;
    s_svc_led_last_error_status = status;
}

static void svc_led_pattern_clear_error_latch(void)
{
    s_svc_led_last_error_stage = SVC_LED_ERROR_STAGE_NONE;
    s_svc_led_last_error_status = 0xFFFFFFFFUL;
}

static void svc_led_pattern_log_init_once(void)
{
    if (s_svc_led_init_logged)
    {
        return;
    }

    SVC_LED_LOG("[LED][INIT] hw=%u profile=%s(%u) data=%u pwr=%u timing=%u/%u/%u/%u count=%u logo=%u notify=%u max=%u",
                (uint32_t)ZY100_LED_HW_ENABLE,
                ZY100_LED_BOARD_PROFILE_NAME,
                (uint32_t)ZY100_LED_BOARD_PROFILE,
                ZY100_RGB_LED_DATA_PIN,
                ZY100_LED_POWER_CTRL_PIN,
                SVC_LED_T0H_TICKS,
                SVC_LED_T0L_TICKS,
                SVC_LED_T1H_TICKS,
                SVC_LED_T1L_TICKS,
                ZY100_RGB_LED_COUNT,
                ZY100_LED_LOGO_COUNT,
                ZY100_LED_NOTIFY_COUNT,
                SVC_LED_BRIGHTNESS_MAX_LEVEL);
    s_svc_led_init_logged = true;
}

static void svc_led_pattern_log_power_on_once(void)
{
    if (s_svc_led_power_on_logged)
    {
        return;
    }

    SVC_LED_LOG("[LED][PWR] on pwr_out=%u pwr_in=%u pad=%u/%u/%u/%u data_out=%u",
                bsp_led_power_ctrl_out_level(),
                bsp_led_power_ctrl_in_level(),
                bsp_led_power_ctrl_pad_out_level(),
                bsp_led_power_ctrl_pad_oe_level(),
                bsp_led_power_ctrl_pad_mode_level(),
                bsp_led_power_ctrl_pad_pwr_level(),
                bsp_led_power_data_out_level());
    s_svc_led_power_on_logged = true;
}

static bool svc_led_pattern_delay_us(uint32_t us)
{
    drv_rgb_led_status_t status = drv_rgb_led_wait_us_checked(us);

    if (status != DRV_RGB_LED_STATUS_OK)
    {
        svc_led_pattern_log_error(SVC_LED_ERROR_STAGE_SETTLE_DELAY,
                                  (uint32_t)status,
                                  drv_rgb_led_status_name(status));
        return false;
    }

    return true;
}

#if (ZY100_LED_PWR_H0_DIAG_ENABLE != 0U)
static void svc_led_pattern_h0_diag_log(const char *mode, bsp_led_power_status_t status);
#endif

static void svc_led_pattern_run_h0_diag_once(void)
{
#if (ZY100_LED_PWR_H0_DIAG_ENABLE != 0U)
    bsp_led_power_status_t status;

    if (s_svc_led_h0_diag_done)
    {
        return;
    }

    s_svc_led_h0_diag_done = true;
    bsp_led_power_data_low();

    status = bsp_led_power_debug_drive_ctrl(true, BSP_LED_POWER_CTRL_MODE_GPIO);
    svc_led_pattern_h0_diag_log("gpio_high", status);
    if (!svc_led_pattern_delay_us(SVC_LED_H0_DIAG_HOLD_US))
    {
        svc_led_pattern_safe_shutdown();
        return;
    }
    (void)bsp_led_power_debug_drive_ctrl(false, BSP_LED_POWER_CTRL_MODE_GPIO);
    if (!svc_led_pattern_delay_us(SVC_LED_H0_DIAG_GAP_US))
    {
        svc_led_pattern_safe_shutdown();
        return;
    }

    status = bsp_led_power_debug_drive_ctrl(true, BSP_LED_POWER_CTRL_MODE_PAD_SW);
    svc_led_pattern_h0_diag_log("pad_sw_high", status);
    if (!svc_led_pattern_delay_us(SVC_LED_H0_DIAG_HOLD_US))
    {
        svc_led_pattern_safe_shutdown();
        return;
    }
    (void)bsp_led_power_debug_drive_ctrl(false, BSP_LED_POWER_CTRL_MODE_PAD_SW);
    if (!svc_led_pattern_delay_us(SVC_LED_H0_DIAG_GAP_US))
    {
        svc_led_pattern_safe_shutdown();
    }
#endif
}

#if (ZY100_LED_PWR_H0_DIAG_ENABLE != 0U)
static void svc_led_pattern_h0_diag_log(const char *mode, bsp_led_power_status_t status)
{
    SVC_LED_LOG("[LED][H0DIAG] mode=%s st=%s(%u) pwr_out=%u pwr_in=%u pad=%u/%u/%u/%u data_out=%u hold_ms=1000",
                mode,
                bsp_led_power_status_name(status),
                (uint32_t)status,
                bsp_led_power_ctrl_out_level(),
                bsp_led_power_ctrl_in_level(),
                bsp_led_power_ctrl_pad_out_level(),
                bsp_led_power_ctrl_pad_oe_level(),
                bsp_led_power_ctrl_pad_mode_level(),
                bsp_led_power_ctrl_pad_pwr_level(),
                bsp_led_power_data_out_level());
}
#endif

static bool svc_led_pattern_backend_init(void)
{
    bsp_led_power_status_t power_status;

    power_status = bsp_led_power_init();
    if (power_status != BSP_LED_POWER_STATUS_OK)
    {
        svc_led_pattern_log_error(SVC_LED_ERROR_STAGE_POWER_INIT,
                                  (uint32_t)power_status,
                                  bsp_led_power_status_name(power_status));
        return false;
    }

    s_svc_led_inited = true;
    s_svc_led_power_on_logged = false;
    svc_led_pattern_clear_error_latch();
    svc_led_pattern_log_init_once();
    return true;
}

static bool svc_led_pattern_ensure_init(void)
{
    if (s_svc_led_inited)
    {
        return true;
    }

    return svc_led_pattern_backend_init();
}

static bool svc_led_pattern_ensure_power_on(void)
{
    bsp_led_power_status_t power_status;
    drv_rgb_led_status_t rgb_status;

    if (!svc_led_pattern_ensure_init())
    {
        return false;
    }

    if (!bsp_led_power_is_on())
    {
        svc_led_pattern_resume_stage_set(SVC_LED_RESUME_STAGE_POWER_GPIO);
        power_status = bsp_led_power_on();
        if (power_status != BSP_LED_POWER_STATUS_OK)
        {
            svc_led_pattern_log_error(SVC_LED_ERROR_STAGE_POWER_ON,
                                      (uint32_t)power_status,
                                      bsp_led_power_status_name(power_status));
            return false;
        }
        svc_led_pattern_resume_stage_set(SVC_LED_RESUME_STAGE_LEVEL_READ);
        if (bsp_led_power_ctrl_out_level() != (uint8_t)(ZY100_LED_POWER_CTRL_ACTIVE_HIGH != 0U))
        {
            svc_led_pattern_log_error(SVC_LED_ERROR_STAGE_POWER_LEVEL,
                                      1U,
                                      "bad_level");
            return false;
        }
        svc_led_pattern_resume_stage_set(SVC_LED_RESUME_STAGE_SETTLE_DELAY);
        if (!svc_led_pattern_delay_us(SVC_LED_POWER_SETTLE_US))
        {
            return false;
        }
        svc_led_pattern_resume_stage_set(SVC_LED_RESUME_STAGE_RGB_INIT);
        rgb_status = drv_rgb_led_init();
        if (rgb_status != DRV_RGB_LED_STATUS_OK)
        {
            svc_led_pattern_log_error(SVC_LED_ERROR_STAGE_RGB_REINIT,
                                      (uint32_t)rgb_status,
                                      drv_rgb_led_status_name(rgb_status));
            return false;
        }
        svc_led_pattern_resume_stage_set(SVC_LED_RESUME_STAGE_TIMING);
        rgb_status = drv_rgb_led_set_timing_ticks(SVC_LED_T0H_TICKS,
                                                  SVC_LED_T0L_TICKS,
                                                  SVC_LED_T1H_TICKS,
                                                  SVC_LED_T1L_TICKS);
        if (rgb_status != DRV_RGB_LED_STATUS_OK)
        {
            svc_led_pattern_log_error(SVC_LED_ERROR_STAGE_TIMING,
                                      (uint32_t)rgb_status,
                                      drv_rgb_led_status_name(rgb_status));
            return false;
        }
        svc_led_pattern_resume_stage_set(SVC_LED_RESUME_STAGE_ALL_OFF);
        rgb_status = drv_rgb_led_all_off(ZY100_RGB_LED_COUNT);
        if (rgb_status != DRV_RGB_LED_STATUS_OK)
        {
            svc_led_pattern_log_error(SVC_LED_ERROR_STAGE_ALL_OFF,
                                      (uint32_t)rgb_status,
                                      drv_rgb_led_status_name(rgb_status));
            return false;
        }
        svc_led_pattern_clear_error_latch();
        svc_led_pattern_log_power_on_once();
    }

    return true;
}

static uint8_t svc_led_pattern_limit_channel(uint8_t value)
{
    if (value > (uint8_t)SVC_LED_BRIGHTNESS_MAX_LEVEL)
    {
        return (uint8_t)SVC_LED_BRIGHTNESS_MAX_LEVEL;
    }

    return value;
}

static uint8_t svc_led_pattern_output_channel(uint8_t value)
{
    uint8_t limited = svc_led_pattern_limit_channel(value);

#if ZY100_BUILD_PRODUCTION
    return (uint8_t)((((uint32_t)limited * SVC_LED_OUTPUT_SCALE_PERCENT) +
                      (SVC_LED_OUTPUT_SCALE_DIVISOR / 2U)) /
                     SVC_LED_OUTPUT_SCALE_DIVISOR);
#else
    return limited;
#endif
}

static drv_rgb_led_status_t svc_led_pattern_show_limited_frame(const zy100_rgb_color_t *frame,
                                                               uint16_t count)
{
    zy100_rgb_color_t limited_frame[ZY100_RGB_LED_COUNT];
    uint16_t idx;
    bool visible = false;
    drv_rgb_led_status_t status;

    if ((frame == NULL) || (count != ZY100_RGB_LED_COUNT))
    {
        return DRV_RGB_LED_STATUS_INVALID_PARAM;
    }

    for (idx = 0U; idx < count; idx++)
    {
        limited_frame[idx].red = svc_led_pattern_output_channel(frame[idx].red);
        limited_frame[idx].green = svc_led_pattern_output_channel(frame[idx].green);
        limited_frame[idx].blue = svc_led_pattern_output_channel(frame[idx].blue);
        visible = visible || (limited_frame[idx].red != 0U) ||
                  (limited_frame[idx].green != 0U) || (limited_frame[idx].blue != 0U);
    }

    /* Decide from the entire scaled frame before any power-on operation.
     * Keep the unscaled shadow frame and animation phase across dark frames. */
    if (!visible)
    {
        if (bsp_led_power_off() != BSP_LED_POWER_STATUS_OK)
        {
            return DRV_RGB_LED_STATUS_INVALID_PIN;
        }
        return DRV_RGB_LED_STATUS_OK;
    }
    if (!svc_led_pattern_ensure_power_on())
    {
        svc_led_pattern_safe_shutdown();
        return DRV_RGB_LED_STATUS_NOT_READY;
    }
    status = drv_rgb_led_show(limited_frame, count);
    if (status != DRV_RGB_LED_STATUS_OK)
    {
        svc_led_pattern_safe_shutdown();
    }
    return status;
}

static bool svc_led_pattern_flush(void)
{
    drv_rgb_led_status_t rgb_status;

    rgb_status = svc_led_pattern_show_limited_frame(s_svc_led_frame, ZY100_RGB_LED_COUNT);
    if (rgb_status != DRV_RGB_LED_STATUS_OK)
    {
        svc_led_pattern_log_error(SVC_LED_ERROR_STAGE_SHOW,
                                  (uint32_t)rgb_status,
                                  drv_rgb_led_status_name(rgb_status));
        return false;
    }

    svc_led_pattern_clear_error_latch();
    return true;
}
#endif

static bool svc_led_pattern_group_bounds(svc_led_group_t group, uint8_t *start, uint8_t *count)
{
    if ((start == NULL) || (count == NULL))
    {
        return false;
    }

    switch (group)
    {
    case LED_LOGO:
        *start = SVC_LED_LOGO_START_INDEX;
        *count = ZY100_LED_LOGO_COUNT;
        return true;
    case LED_NOTIFY:
        *start = SVC_LED_NOTIFY_START_INDEX;
        *count = ZY100_LED_NOTIFY_COUNT;
        return true;
    default:
        return false;
    }
}


/* Task-context render transaction. The DMA waveform buffer and shadow frame
 * must remain owned until the complete synchronous frame has finished. */
#if ZY100_BUILD_PRODUCTION
static void *s_led_render_mutex;
#endif
bool svc_led_pattern_lock(void)
{
#if ZY100_BUILD_PRODUCTION
    void *candidate = NULL;
    uint32_t key;
    if (s_led_render_mutex == NULL)
    {
        if (!os_mutex_create(&candidate)) return false;
        key = os_lock();
        if (s_led_render_mutex == NULL)
        {
            s_led_render_mutex = candidate;
            candidate = NULL;
        }
        os_unlock(key);
        if (candidate != NULL) (void)os_mutex_delete(candidate);
    }
    return os_mutex_take(s_led_render_mutex, 0xFFFFFFFFU);
#else
    return true;
#endif
}

void svc_led_pattern_unlock(void)
{
#if ZY100_BUILD_PRODUCTION
    (void)os_mutex_give(s_led_render_mutex);
#endif
}

static bool svc_led_pattern_init_locked(void)
{
#if (ZY100_LED_HW_ENABLE == 0U)
    memset(s_svc_led_frame, 0, sizeof(s_svc_led_frame));
    return true;
#else
    return svc_led_pattern_ensure_init();
#endif
}

static bool svc_led_pattern_resume_after_wake_locked(void)
{
#if (ZY100_LED_HW_ENABLE == 0U)
    memset(s_svc_led_frame, 0, sizeof(s_svc_led_frame));
    return true;
#else
    s_svc_led_inited = false;
    s_svc_led_resume_in_progress = true;
    svc_led_pattern_resume_stage_set(SVC_LED_RESUME_STAGE_DATA_LOW);
    svc_led_pattern_run_h0_diag_once();
#if !ZY100_BUILD_FACTORY
    zy100_power_reset_trace_set_stage(ZY100_POWER_TRACE_LED_BACKEND);
#endif
    if (!svc_led_pattern_backend_init())
    {
        ZY100_LOG_ERROR("[ERR][LED] resume stage=backend_failed");
        svc_led_pattern_safe_shutdown();
        s_svc_led_resume_in_progress = false;
        return false;
    }

    /* Runtime availability is not a request to light the chain. */
    memset(s_svc_led_frame, 0, sizeof(s_svc_led_frame));
    s_svc_led_resume_in_progress = false;
    return true;
#endif
}

static bool svc_led_pattern_show_frame_locked(const zy100_rgb_color_t *frame, uint16_t count)
{
    if ((frame == NULL) || (count != ZY100_RGB_LED_COUNT))
    {
        return false;
    }

    memcpy(s_svc_led_frame, frame, sizeof(s_svc_led_frame));

#if (ZY100_LED_HW_ENABLE == 0U)
    return true;
#else
    return svc_led_pattern_flush();
#endif
}

static bool svc_led_pattern_set_group_color_locked(svc_led_group_t group, const zy100_rgb_color_t *color)
{
    uint8_t start;
    uint8_t count;
    uint8_t idx;

    if ((color == NULL) || !svc_led_pattern_group_bounds(group, &start, &count))
    {
        return false;
    }

    for (idx = 0U; idx < count; idx++)
    {
        s_svc_led_frame[start + idx] = *color;
    }

#if (ZY100_LED_HW_ENABLE == 0U)
    return true;
#else
    return svc_led_pattern_flush();
#endif
}

static bool svc_led_pattern_group_off_locked(svc_led_group_t group)
{
    uint8_t start;
    uint8_t count;
    uint8_t idx;
#if (ZY100_LED_HW_ENABLE != 0U)
    drv_rgb_led_status_t rgb_status;
#endif

    if (!svc_led_pattern_group_bounds(group, &start, &count))
    {
        return false;
    }

#if (ZY100_LED_HW_ENABLE != 0U)
    if (!svc_led_pattern_ensure_init())
    {
        return false;
    }
#endif

    for (idx = 0U; idx < count; idx++)
    {
        s_svc_led_frame[start + idx].red = 0U;
        s_svc_led_frame[start + idx].green = 0U;
        s_svc_led_frame[start + idx].blue = 0U;
    }

#if (ZY100_LED_HW_ENABLE == 0U)
    return true;
#else
    rgb_status = svc_led_pattern_show_limited_frame(s_svc_led_frame, ZY100_RGB_LED_COUNT);
    if (rgb_status != DRV_RGB_LED_STATUS_OK)
    {
        svc_led_pattern_log_error(SVC_LED_ERROR_STAGE_SHOW,
                                  (uint32_t)rgb_status,
                                  drv_rgb_led_status_name(rgb_status));
        return false;
    }

    svc_led_pattern_clear_error_latch();
    return true;
#endif
}

bool svc_led_pattern_notify_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    zy100_rgb_color_t color;

    color.red = red;
    color.green = green;
    color.blue = blue;
    return svc_led_pattern_set_group_color(LED_NOTIFY, &color);
}

bool svc_led_pattern_notify_green_on(void)
{
    return svc_led_pattern_notify_rgb(0U, ZY100_LED_NOTIFY_BRIGHTNESS, 0U);
}

bool svc_led_pattern_notify_blue_on(void)
{
    return svc_led_pattern_notify_rgb(0U, 0U, ZY100_LED_NOTIFY_BRIGHTNESS);
}

bool svc_led_pattern_notify_red_on(void)
{
    return svc_led_pattern_notify_rgb(ZY100_LED_NOTIFY_BRIGHTNESS, 0U, 0U);
}

bool svc_led_pattern_notify_orange_on(void)
{
    return svc_led_pattern_notify_rgb(ZY100_LED_NOTIFY_BRIGHTNESS,
                                      (uint8_t)(ZY100_LED_NOTIFY_BRIGHTNESS / 2U),
                                      0U);
}

bool svc_led_pattern_notify_off(void)
{
    return svc_led_pattern_group_off(LED_NOTIFY);
}

bool svc_led_pattern_logo_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    zy100_rgb_color_t color;

    color.red = red;
    color.green = green;
    color.blue = blue;
    return svc_led_pattern_set_group_color(LED_LOGO, &color);
}

bool svc_led_pattern_logo_green_on(void)
{
    return svc_led_pattern_logo_rgb(0U, ZY100_LED_LOGO_TEST_BRIGHTNESS, 0U);
}

bool svc_led_pattern_logo_off(void)
{
    return svc_led_pattern_group_off(LED_LOGO);
}

static bool svc_led_pattern_all_off_locked(void)
{
#if (ZY100_LED_HW_ENABLE == 0U)
    memset(s_svc_led_frame, 0, sizeof(s_svc_led_frame));
    return true;
#else
    drv_rgb_led_status_t rgb_status;

    if (!svc_led_pattern_ensure_init())
    {
        return false;
    }

    memset(s_svc_led_frame, 0, sizeof(s_svc_led_frame));
    rgb_status = svc_led_pattern_show_limited_frame(s_svc_led_frame, ZY100_RGB_LED_COUNT);
    if (rgb_status != DRV_RGB_LED_STATUS_OK)
    {
        svc_led_pattern_log_error(SVC_LED_ERROR_STAGE_SHOW,
                                  (uint32_t)rgb_status,
                                  drv_rgb_led_status_name(rgb_status));
        return false;
    }

    svc_led_pattern_clear_error_latch();
    return true;
#endif
}

static void svc_led_pattern_shutdown_for_sleep_locked(void)
{
#if (ZY100_LED_HW_ENABLE == 0U)
    memset(s_svc_led_frame, 0, sizeof(s_svc_led_frame));
#else
    if (s_svc_led_inited && bsp_led_power_is_on())
    {
        (void)svc_led_pattern_all_off();
    }

    drv_rgb_led_data_low();
    (void)bsp_led_power_off();
#endif
}

bool svc_led_pattern_init(void)
{
    bool ok;
    if (!svc_led_pattern_lock()) return false;
    ok = svc_led_pattern_init_locked();
    svc_led_pattern_unlock();
    return ok;
}

bool svc_led_pattern_resume_after_wake(void)
{
    bool ok;
    if (!svc_led_pattern_lock()) return false;
    ok = svc_led_pattern_resume_after_wake_locked();
    svc_led_pattern_unlock();
    return ok;
}

bool svc_led_pattern_show_frame(const zy100_rgb_color_t *frame, uint16_t count)
{
    bool ok;
    if (!svc_led_pattern_lock()) return false;
    ok = svc_led_pattern_show_frame_locked(frame, count);
    svc_led_pattern_unlock();
    return ok;
}

bool svc_led_pattern_set_group_color(svc_led_group_t group, const zy100_rgb_color_t *color)
{
    bool ok;
    if (!svc_led_pattern_lock()) return false;
    ok = svc_led_pattern_set_group_color_locked(group, color);
    svc_led_pattern_unlock();
    return ok;
}

bool svc_led_pattern_group_off(svc_led_group_t group)
{
    bool ok;
    if (!svc_led_pattern_lock()) return false;
    ok = svc_led_pattern_group_off_locked(group);
    svc_led_pattern_unlock();
    return ok;
}

bool svc_led_pattern_all_off(void)
{
    bool ok;
    if (!svc_led_pattern_lock()) return false;
    ok = svc_led_pattern_all_off_locked();
    svc_led_pattern_unlock();
    return ok;
}

void svc_led_pattern_shutdown_for_sleep(void)
{
    if (!svc_led_pattern_lock()) return;
    svc_led_pattern_shutdown_for_sleep_locked();
    svc_led_pattern_unlock();
}
