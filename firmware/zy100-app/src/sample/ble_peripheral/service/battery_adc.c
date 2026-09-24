#include "service/battery_adc.h"

#include <stddef.h>
#include <string.h>

#include "os_sched.h"
#include "gap.h"
#include "trace.h"
#include "../bsp/bsp_battery_adc.h"
#include "../app_flags.h"
#include "zy100_calibration_store.h"

#if ZY100_PRODUCT_LOG_QUIET_ENABLE
#define BATTERY_ADC_INFO_LOG(...)       do { if (0) { DBG_DIRECT(__VA_ARGS__); } } while (0)
#else
#define BATTERY_ADC_INFO_LOG(...)       DBG_DIRECT(__VA_ARGS__)
#endif

#define BATTERY_ADC_RAW_MAX             BSP_BATTERY_ADC_RAW_MAX
#define BATTERY_EMPTY_MV                3500U
#define BATTERY_MID_MV                  4000U
#define BATTERY_FULL_MV                 4350U

#ifndef BATTERY_ADC_FIRST_DELAY_MS
#define BATTERY_ADC_FIRST_DELAY_MS      3000U
#endif

#ifndef BATTERY_ADC_PERIOD_MS
#define BATTERY_ADC_PERIOD_MS           10000U
#endif

#ifndef BATTERY_ADC_DEFER_MS
#define BATTERY_ADC_DEFER_MS            1000U
#endif

#ifndef BATTERY_ADC_RETRY_SETTLE_MS
#define BATTERY_ADC_RETRY_SETTLE_MS     60U
#endif

#ifndef BATTERY_ADC_WARMUP_COUNT
#define BATTERY_ADC_WARMUP_COUNT        8U
#endif

#ifndef BATTERY_ADC_SAMPLE_COUNT
#define BATTERY_ADC_SAMPLE_COUNT        64U
#endif

#ifndef BATTERY_ADC_TRIM_COUNT
#define BATTERY_ADC_TRIM_COUNT          8U
#endif

#ifndef BATTERY_ADC_SAMPLE_INTERVAL_MS
#define BATTERY_ADC_SAMPLE_INTERVAL_MS  0U
#endif

#ifndef BATTERY_ADC_FAST_BURST_ENABLE
#define BATTERY_ADC_FAST_BURST_ENABLE   1U
#endif

#ifndef BATTERY_ADC_DUMP_RAW_SAMPLES
#define BATTERY_ADC_DUMP_RAW_SAMPLES    0U
#endif

#ifndef BATTERY_ADC_DLPS_TRANSITION_LOG_ENABLE
#define BATTERY_ADC_DLPS_TRANSITION_LOG_ENABLE 0U
#endif

#ifndef BATTERY_ADC_RAW_DUMP_PER_LINE
#define BATTERY_ADC_RAW_DUMP_PER_LINE   8U
#endif

#ifndef BATTERY_ADC_MAX_SPREAD_RAW
#define BATTERY_ADC_MAX_SPREAD_RAW      40U
#endif

#ifndef BATTERY_ADC_MAX_RETRY
#define BATTERY_ADC_MAX_RETRY           1U
#endif

#ifndef BATTERY_ADC_FILTER_SHIFT
#define BATTERY_ADC_FILTER_SHIFT        3U
#endif

#ifndef BATTERY_ADC_CHARGE_REPORT_MAX_PERCENT
#define BATTERY_ADC_CHARGE_REPORT_MAX_PERCENT 99U
#endif

#ifndef BATTERY_ADC_USB_CHARGE_QUARANTINE_RETRY_MS
#define BATTERY_ADC_USB_CHARGE_QUARANTINE_RETRY_MS BATTERY_ADC_PERIOD_MS
#endif

#ifndef BATTERY_ADC_USB_CHARGE_QUARANTINE_LOG_PERIOD_MS
#define BATTERY_ADC_USB_CHARGE_QUARANTINE_LOG_PERIOD_MS BATTERY_ADC_PERIOD_MS
#endif

#ifndef ZY100_USB_BAT_ADC_QUARANTINE_ENABLE
#define ZY100_USB_BAT_ADC_QUARANTINE_ENABLE 1U
#endif

#ifndef ZY100_BAT_ADC_SUSPECT_REJECT_ENABLE
#define ZY100_BAT_ADC_SUSPECT_REJECT_ENABLE 1U
#endif

#if (BATTERY_ADC_RAW_MAX == 0U)
#error "BATTERY_ADC_RAW_MAX must be non-zero"
#endif

#if (BATTERY_FULL_MV <= BATTERY_EMPTY_MV)
#error "BATTERY_FULL_MV must be greater than BATTERY_EMPTY_MV"
#endif

#if ((BATTERY_MID_MV <= BATTERY_EMPTY_MV) || (BATTERY_MID_MV >= BATTERY_FULL_MV))
#error "BATTERY_MID_MV must be between BATTERY_EMPTY_MV and BATTERY_FULL_MV"
#endif

#if ((BATTERY_ADC_DIVIDER_R_TOP_OHM == 0UL) || \
     (BATTERY_ADC_DIVIDER_R_BOTTOM_OHM == 0UL))
#error "battery ADC divider resistors must be non-zero"
#endif

#if (BATTERY_ADC_SAMPLE_COUNT <= (BATTERY_ADC_TRIM_COUNT * 2U))
#error "BATTERY_ADC_SAMPLE_COUNT must be greater than twice BATTERY_ADC_TRIM_COUNT"
#endif

#if (BATTERY_ADC_USB_CHARGE_QUARANTINE_RETRY_MS == 0U)
#error "BATTERY_ADC_USB_CHARGE_QUARANTINE_RETRY_MS must be non-zero"
#endif

#if (BATTERY_ADC_USB_CHARGE_QUARANTINE_LOG_PERIOD_MS == 0U)
#error "BATTERY_ADC_USB_CHARGE_QUARANTINE_LOG_PERIOD_MS must be non-zero"
#endif

#if (BATTERY_ADC_FILTER_SHIFT == 0U)
#error "BATTERY_ADC_FILTER_SHIFT must be non-zero"
#endif

#if BATTERY_ADC_DUMP_RAW_SAMPLES
#if (BATTERY_ADC_RAW_DUMP_PER_LINE != 8U)
#error "BATTERY_ADC_RAW_DUMP_PER_LINE must be 8 for the fixed raw dump formatter"
#endif
#if ((BATTERY_ADC_SAMPLE_COUNT % BATTERY_ADC_RAW_DUMP_PER_LINE) != 0U)
#error "BATTERY_ADC_SAMPLE_COUNT must be a multiple of BATTERY_ADC_RAW_DUMP_PER_LINE"
#endif
#endif

typedef enum
{
    BATTERY_ADC_STAGE_IDLE = 0,
    BATTERY_ADC_STAGE_PREPARE_ADC,
    BATTERY_ADC_STAGE_SETTLE_WAIT,
    BATTERY_ADC_STAGE_FAST_BURST,
    BATTERY_ADC_STAGE_GUARD_ONE_SHOT,
    BATTERY_ADC_STAGE_GUARD_SESSION,
    BATTERY_ADC_STAGE_FINALIZE,
} battery_adc_stage_t;

typedef struct
{
    uint16_t mv;
    uint8_t percent;
} battery_adc_soc_point_t;

static const battery_adc_soc_point_t s_battery_adc_soc_lut[] =
{
    {3500U, 0U},
    {3641U, 5U},
    {3660U, 10U},
    {3706U, 20U},
    {3744U, 30U},
    {3787U, 40U},
    {3837U, 50U},
    {3902U, 60U},
    {4001U, 70U},
    {4100U, 80U},
    {4205U, 90U},
    {4259U, 95U},
    {4350U, 100U},
};

typedef enum
{
    BATTERY_ADC_STATUS_OK = 0,
    BATTERY_ADC_STATUS_NOT_INIT,
    BATTERY_ADC_STATUS_BUSY,
    BATTERY_ADC_STATUS_CONFIG_FAIL,
    BATTERY_ADC_STATUS_TIMEOUT,
    BATTERY_ADC_STATUS_RAW_RANGE_FAIL,
    BATTERY_ADC_STATUS_CALIBRATION_FAIL,
    BATTERY_ADC_STATUS_SPREAD_REJECT,
    BATTERY_ADC_STATUS_SUSPECT_ENV,
    BATTERY_ADC_STATUS_USB_CHARGE_QUARANTINE,
    BATTERY_ADC_STATUS_DEFERRED,
} battery_adc_status_t;

static bool s_battery_adc_inited = false;
static bool s_battery_adc_busy = false;
static bool s_battery_adc_guard_session_active = false;
static bool s_battery_adc_schedule_valid = false;
static bool s_battery_adc_due_pending = false;
static bool s_battery_adc_filter_valid = false;
static bool s_battery_adc_log_enabled = true;
static uint32_t s_battery_adc_next_due_ms = 0U;
static uint32_t s_battery_adc_next_action_ms = 0U;
static charge_voltage_sample_t s_battery_charge_sample;
static uint16_t s_battery_adc_last_raw = 0U;
static uint16_t s_battery_adc_filtered_raw = 0U;
static uint16_t s_battery_adc_vbat_mv = 0U;
static uint16_t s_battery_adc_last_min = 0U;
static uint16_t s_battery_adc_last_max = 0U;
static uint16_t s_battery_adc_last_spread = 0U;
static uint16_t s_battery_adc_report_vbat_mv = 0U;
static uint16_t s_battery_adc_samples[BATTERY_ADC_SAMPLE_COUNT];
#if BATTERY_ADC_DUMP_RAW_SAMPLES
static uint16_t s_battery_adc_sorted_samples[BATTERY_ADC_SAMPLE_COUNT];
#endif
static uint16_t s_battery_adc_raw_min = BATTERY_ADC_RAW_MAX;
static uint16_t s_battery_adc_raw_max = 0U;
static uint16_t s_battery_adc_burst_raw_avg = 0U;
static uint16_t s_battery_adc_burst_fail_raw = 0U;
static uint8_t s_battery_adc_retry_count = 0U;
static uint8_t s_battery_adc_last_retry = 0U;
static uint8_t s_battery_adc_percent = 0U;
static uint8_t s_battery_adc_report_percent = 0U;
static bool s_battery_adc_charge_report_present = false;
static bool s_battery_adc_charge_report_full = false;
static bool s_battery_adc_charge_path_active = false;
static bool s_battery_adc_trusted_valid = false;
static bool s_battery_adc_last_quarantine = false;
static bool s_battery_adc_usb_charge_low_hold = false;
static uint16_t s_battery_adc_trusted_raw = 0U;
static uint16_t s_battery_adc_trusted_vbat_mv = 0U;
static uint16_t s_battery_adc_filter_raw_for_filter = 0U;
static uint16_t s_battery_adc_filter_before_raw = 0U;
static uint16_t s_battery_adc_filter_step_raw = 0U;
static uint8_t s_battery_adc_trusted_percent = 0U;
static uint8_t s_battery_adc_filter_action = BATTERY_ADC_FILTER_ACTION_NONE;
static uint8_t s_battery_adc_filter_direction = BATTERY_ADC_FILTER_DIR_NONE;
static battery_adc_status_t s_battery_adc_last_status = BATTERY_ADC_STATUS_NOT_INIT;
static battery_adc_status_t s_battery_adc_burst_status = BATTERY_ADC_STATUS_OK;
static battery_adc_stage_t s_battery_adc_state = BATTERY_ADC_STAGE_IDLE;
static battery_adc_stage_t s_battery_adc_last_stage = BATTERY_ADC_STAGE_IDLE;
static battery_adc_stage_t s_battery_adc_burst_stage = BATTERY_ADC_STAGE_IDLE;
static uint32_t s_battery_adc_attempt_start_ms = 0U;
static uint32_t s_battery_adc_last_burst_ms = 0U;
static uint32_t s_battery_adc_last_total_ms = 0U;
static uint32_t s_battery_adc_ok_count = 0U;
static uint32_t s_battery_adc_fail_count = 0U;
static bool s_battery_adc_quarantine_log_valid = false;
static bool s_battery_adc_quarantine_log_held = false;
static bool s_battery_adc_quarantine_log_trusted = false;
static bool s_battery_adc_quarantine_log_path = false;
static bool s_battery_adc_quarantine_log_latch = false;
static uint32_t s_battery_adc_quarantine_log_ms = 0U;
static uint16_t s_battery_adc_quarantine_log_trusted_raw = 0U;
static uint16_t s_battery_adc_quarantine_log_filtered_raw = 0U;
static battery_adc_sample_window_cb_t s_battery_adc_sample_window_cb = NULL;
static void *s_battery_adc_sample_window_ctx = NULL;
static bool s_battery_adc_calibration_valid = false;
static uint32_t s_battery_adc_gain_q20 = 0UL;
#if BATTERY_ADC_DUMP_RAW_SAMPLES
static bool s_battery_adc_raw_samples_valid = false;
#endif

static const char *battery_adc_status_str(battery_adc_status_t status)
{
    switch (status)
    {
    case BATTERY_ADC_STATUS_OK:
        return "ok";
    case BATTERY_ADC_STATUS_NOT_INIT:
        return "not_init";
    case BATTERY_ADC_STATUS_BUSY:
        return "busy";
    case BATTERY_ADC_STATUS_CONFIG_FAIL:
        return "config_fail";
    case BATTERY_ADC_STATUS_TIMEOUT:
        return "timeout";
    case BATTERY_ADC_STATUS_RAW_RANGE_FAIL:
        return "raw_range_fail";
    case BATTERY_ADC_STATUS_CALIBRATION_FAIL:
        return "calibration_fail";
    case BATTERY_ADC_STATUS_SPREAD_REJECT:
        return "spread_reject";
    case BATTERY_ADC_STATUS_SUSPECT_ENV:
        return "suspect_env";
    case BATTERY_ADC_STATUS_USB_CHARGE_QUARANTINE:
        return "usb_charge_quarantine";
    case BATTERY_ADC_STATUS_DEFERRED:
        return "deferred";
    default:
        return "unknown";
    }
}

static const char *battery_adc_stage_str(battery_adc_stage_t stage)
{
    switch (stage)
    {
    case BATTERY_ADC_STAGE_IDLE:
        return "idle";
    case BATTERY_ADC_STAGE_PREPARE_ADC:
        return "prepare_adc";
    case BATTERY_ADC_STAGE_SETTLE_WAIT:
        return "settle_wait";
    case BATTERY_ADC_STAGE_FAST_BURST:
        return "fast_burst";
    case BATTERY_ADC_STAGE_GUARD_ONE_SHOT:
        return "guard_one_shot";
    case BATTERY_ADC_STAGE_GUARD_SESSION:
        return "guard_session";
    case BATTERY_ADC_STAGE_FINALIZE:
        return "finalize";
    default:
        return "unknown";
    }
}

static const char *battery_adc_mode_str(void)
{
    bsp_battery_adc_info_t info = {0};

    bsp_battery_adc_get_info(&info);
    return info.bypass_mode ? "bypass" : "divide";
}

static void battery_adc_record_status(battery_adc_status_t status)
{
    s_battery_adc_last_status = status;
    if (status != BATTERY_ADC_STATUS_OK) s_battery_charge_sample.valid = false;
}

static bool battery_adc_time_reached(uint32_t now_ms, uint32_t due_ms)
{
    return ((int32_t)(now_ms - due_ms) >= 0);
}

static uint32_t battery_adc_now_ms(void)
{
    return (uint32_t)os_sys_time_get();
}

static uint32_t battery_adc_elapsed_ms(uint32_t start_ms, uint32_t end_ms)
{
    return (uint32_t)(end_ms - start_ms);
}

static void battery_adc_schedule_after(uint32_t runtime_ms, uint32_t delay_ms)
{
    s_battery_adc_next_due_ms = runtime_ms + delay_ms;
    s_battery_adc_schedule_valid = true;
    s_battery_adc_due_pending = false;
}

static void battery_adc_mark_due_pending(uint32_t runtime_ms)
{
    s_battery_adc_schedule_valid = true;
    if (!s_battery_adc_due_pending)
    {
        s_battery_adc_next_due_ms = runtime_ms;
        s_battery_adc_due_pending = true;
    }
}

static void battery_adc_park_pin_low_power(void)
{
    bsp_battery_adc_park_low_power();
}

static void battery_adc_prepare_pin_for_runtime(void)
{
    bsp_battery_adc_prepare_runtime_pin();
}

static bool battery_adc_raw_in_range(uint16_t raw)
{
    return (raw <= BATTERY_ADC_RAW_MAX);
}

static uint16_t battery_adc_lerp_u16(uint32_t x,
                                     uint32_t x0,
                                     uint32_t x1,
                                     uint32_t y0,
                                     uint32_t y1)
{
    const uint32_t x_range = x1 - x0;
    const uint32_t y_range = y1 - y0;

    return (uint16_t)(y0 + ((((x - x0) * y_range) + (x_range / 2U)) / x_range));
}

static bool battery_adc_convert_raw_sdk(uint16_t raw,
                                        uint16_t *vbat_mv_out,
                                        int8_t *error_out)
{
    bsp_battery_adc_status_t status;

    if (vbat_mv_out != NULL)
    {
        *vbat_mv_out = 0U;
    }
    if (error_out != NULL)
    {
        *error_out = (int8_t)BSP_BATTERY_ADC_STATUS_CONVERSION_ERROR;
    }

    status = bsp_battery_adc_raw_to_vbat_mv(raw, vbat_mv_out);
    if ((status == BSP_BATTERY_ADC_STATUS_OK) &&
        (vbat_mv_out != NULL) && s_battery_adc_calibration_valid)
    {
        uint64_t corrected = ((uint64_t)(*vbat_mv_out) *
                              (uint64_t)s_battery_adc_gain_q20) +
                             (1ULL << 19U);
        corrected >>= 20U;
        *vbat_mv_out = (corrected > 0xFFFFULL) ? 0xFFFFU :
                       (uint16_t)corrected;
    }
    if (error_out != NULL)
    {
        *error_out = (int8_t)status;
    }
    return status == BSP_BATTERY_ADC_STATUS_OK;
}

static uint16_t battery_adc_clamp_raw_for_conversion(uint16_t raw)
{
    return raw;
}

static uint8_t battery_adc_vbat_mv_to_percent(uint16_t vbat_mv)
{
    uint8_t idx;

    if (vbat_mv <= s_battery_adc_soc_lut[0].mv)
    {
        return s_battery_adc_soc_lut[0].percent;
    }

    for (idx = 1U;
         idx < (uint8_t)(sizeof(s_battery_adc_soc_lut) /
                         sizeof(s_battery_adc_soc_lut[0]));
         idx++)
    {
        const battery_adc_soc_point_t *prev = &s_battery_adc_soc_lut[idx - 1U];
        const battery_adc_soc_point_t *next = &s_battery_adc_soc_lut[idx];

        if (vbat_mv <= next->mv)
        {
            return (uint8_t)battery_adc_lerp_u16(vbat_mv,
                                                 prev->mv,
                                                 next->mv,
                                                 prev->percent,
                                                 next->percent);
        }
    }

    return 100U;
}

uint16_t battery_adc_percent_to_voltage_mv(uint8_t percent)
{
    uint8_t idx;

    if (percent <= s_battery_adc_soc_lut[0].percent)
    {
        return s_battery_adc_soc_lut[0].mv;
    }

    for (idx = 1U;
         idx < (uint8_t)(sizeof(s_battery_adc_soc_lut) /
                         sizeof(s_battery_adc_soc_lut[0]));
         idx++)
    {
        const battery_adc_soc_point_t *prev = &s_battery_adc_soc_lut[idx - 1U];
        const battery_adc_soc_point_t *next = &s_battery_adc_soc_lut[idx];

        if (percent <= next->percent)
        {
            return battery_adc_lerp_u16(percent,
                                        prev->percent,
                                        next->percent,
                                        prev->mv,
                                        next->mv);
        }
    }

    return s_battery_adc_soc_lut[(sizeof(s_battery_adc_soc_lut) /
                                  sizeof(s_battery_adc_soc_lut[0])) - 1U].mv;
}

static void battery_adc_update_report_values(void);

static void battery_adc_note_filter_diag(uint16_t raw_for_filter,
                                         uint16_t filter_before,
                                         uint16_t filter_step,
                                         uint8_t filter_action,
                                         uint8_t filter_direction)
{
    s_battery_adc_filter_raw_for_filter = raw_for_filter;
    s_battery_adc_filter_before_raw = filter_before;
    s_battery_adc_filter_step_raw = filter_step;
    s_battery_adc_filter_action = filter_action;
    s_battery_adc_filter_direction = filter_direction;
}

static uint8_t battery_adc_filter_direction_from_raw(uint16_t from_raw,
                                                     uint16_t to_raw)
{
    if (to_raw > from_raw)
    {
        return BATTERY_ADC_FILTER_DIR_UP;
    }
    if (to_raw < from_raw)
    {
        return BATTERY_ADC_FILTER_DIR_DOWN;
    }
    return BATTERY_ADC_FILTER_DIR_NONE;
}

static bool battery_adc_should_fast_accept_raw(uint16_t raw_for_filter,
                                               uint16_t filter_before)
{
    uint16_t delta = (raw_for_filter >= filter_before) ?
                     (uint16_t)(raw_for_filter - filter_before) :
                     (uint16_t)(filter_before - raw_for_filter);

    if (s_battery_adc_charge_report_present)
    {
        return false;
    }

    return delta > BATTERY_ADC_MAX_SPREAD_RAW;
}

static uint16_t battery_adc_raw_abs_delta(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static uint16_t battery_adc_filter_step_from_delta(uint16_t delta)
{
    uint16_t step;

    if (delta == 0U)
    {
        return 0U;
    }

    step = (uint16_t)(delta >> BATTERY_ADC_FILTER_SHIFT);
    if (step == 0U)
    {
        step = 1U;
    }
    return step;
}

static uint16_t battery_adc_filter_move_towards(uint16_t current_raw,
                                                uint16_t target_raw,
                                                uint16_t *step_out,
                                                uint8_t *direction_out)
{
    uint16_t delta = battery_adc_raw_abs_delta(current_raw, target_raw);
    uint16_t step = battery_adc_filter_step_from_delta(delta);

    if (step_out != NULL)
    {
        *step_out = step;
    }
    if (direction_out != NULL)
    {
        *direction_out = battery_adc_filter_direction_from_raw(current_raw,
                                                               target_raw);
    }

    if (target_raw > current_raw)
    {
        return (uint16_t)(current_raw + step);
    }
    if (target_raw < current_raw)
    {
        return (uint16_t)(current_raw - step);
    }
    return current_raw;
}

static void battery_adc_note_rejected_filter_raw(uint16_t raw_avg,
                                                 uint8_t action)
{
    uint16_t raw_for_filter = battery_adc_clamp_raw_for_conversion(raw_avg);

    s_battery_adc_last_raw = raw_avg;
    s_battery_adc_last_quarantine = false;
    battery_adc_note_filter_diag(raw_for_filter,
                                 s_battery_adc_filtered_raw,
                                 0U,
                                 action,
                                 battery_adc_filter_direction_from_raw(
                                     s_battery_adc_filtered_raw,
                                     raw_for_filter));
}

static uint16_t battery_adc_usb_charge_no_trusted_low_limit(void)
{
    /* No absolute raw threshold is valid across SDK calibration data. */
    return 0U;
}

static bool battery_adc_raw_close_to_trusted(uint16_t raw_for_filter)
{
    if (!s_battery_adc_trusted_valid)
    {
        return false;
    }

    return (battery_adc_raw_abs_delta(raw_for_filter,
                                      s_battery_adc_trusted_raw) <=
            BATTERY_ADC_MAX_SPREAD_RAW);
}

static bool battery_adc_should_quarantine_usb_charge_raw(uint16_t raw_avg)
{
#if ZY100_USB_BAT_ADC_QUARANTINE_ENABLE
    uint16_t raw_for_filter = battery_adc_clamp_raw_for_conversion(raw_avg);
    bool charge_path_risk =
        (s_battery_adc_charge_report_present &&
         s_battery_adc_charge_path_active) ||
        s_battery_adc_usb_charge_low_hold;

    if (!charge_path_risk)
    {
        return false;
    }

    if (s_battery_adc_trusted_valid)
    {
        if (raw_for_filter >= s_battery_adc_trusted_raw)
        {
            return false;
        }

        return ((uint16_t)(s_battery_adc_trusted_raw - raw_for_filter) >
                BATTERY_ADC_MAX_SPREAD_RAW);
    }

    return (raw_for_filter < battery_adc_usb_charge_no_trusted_low_limit());
#else
    (void)raw_avg;
    return false;
#endif
}

static bool battery_adc_should_update_trusted_raw(uint16_t raw_for_filter)
{
    if (!s_battery_adc_charge_report_present ||
        !s_battery_adc_charge_path_active)
    {
        return true;
    }

    if (battery_adc_raw_close_to_trusted(raw_for_filter))
    {
        return true;
    }

    return (!s_battery_adc_trusted_valid &&
            (raw_for_filter >= battery_adc_usb_charge_no_trusted_low_limit()));
}

static void battery_adc_note_trusted_from_current(void)
{
    if (!s_battery_adc_filter_valid)
    {
        return;
    }

    s_battery_adc_trusted_raw = s_battery_adc_filtered_raw;
    s_battery_adc_trusted_vbat_mv = s_battery_adc_vbat_mv;
    s_battery_adc_trusted_percent = s_battery_adc_percent;
    s_battery_adc_trusted_valid = true;
    s_battery_adc_usb_charge_low_hold = false;
}

static bool battery_adc_restore_trusted_baseline(void)
{
    if (!s_battery_adc_trusted_valid)
    {
        return false;
    }

    s_battery_adc_filtered_raw = s_battery_adc_trusted_raw;
    s_battery_adc_vbat_mv = s_battery_adc_trusted_vbat_mv;
    s_battery_adc_percent = s_battery_adc_trusted_percent;
    s_battery_adc_filter_valid = true;
    battery_adc_update_report_values();
    return true;
}

static bool battery_adc_usb_charge_quarantine_log_due(bool held)
{
    uint32_t now_ms = battery_adc_now_ms();
    bool trusted = s_battery_adc_trusted_valid;
    bool path = s_battery_adc_charge_path_active;
    bool latch = s_battery_adc_usb_charge_low_hold;
    bool changed =
        !s_battery_adc_quarantine_log_valid ||
        (s_battery_adc_quarantine_log_held != held) ||
        (s_battery_adc_quarantine_log_trusted != trusted) ||
        (s_battery_adc_quarantine_log_path != path) ||
        (s_battery_adc_quarantine_log_latch != latch) ||
        (s_battery_adc_quarantine_log_trusted_raw != s_battery_adc_trusted_raw) ||
        (s_battery_adc_quarantine_log_filtered_raw != s_battery_adc_filtered_raw);

    if (!changed &&
        ((uint32_t)(now_ms - s_battery_adc_quarantine_log_ms) <
         (uint32_t)BATTERY_ADC_USB_CHARGE_QUARANTINE_LOG_PERIOD_MS))
    {
        return false;
    }

    s_battery_adc_quarantine_log_valid = true;
    s_battery_adc_quarantine_log_held = held;
    s_battery_adc_quarantine_log_trusted = trusted;
    s_battery_adc_quarantine_log_path = path;
    s_battery_adc_quarantine_log_latch = latch;
    s_battery_adc_quarantine_log_ms = now_ms;
    s_battery_adc_quarantine_log_trusted_raw = s_battery_adc_trusted_raw;
    s_battery_adc_quarantine_log_filtered_raw = s_battery_adc_filtered_raw;
    return true;
}

static void battery_adc_log_usb_charge_quarantine(uint16_t raw_avg,
                                                  bool held)
{
    if (!s_battery_adc_log_enabled ||
        !battery_adc_usb_charge_quarantine_log_due(held))
    {
        return;
    }

    DBG_DIRECT("[BAT_ADC] quarantine usb_charge raw=%u held=%u trusted=%u traw=%u filt=%u path=%u latch=%u",
               raw_avg,
               held ? 1U : 0U,
               s_battery_adc_trusted_valid ? 1U : 0U,
               s_battery_adc_trusted_raw,
               s_battery_adc_filtered_raw,
               s_battery_adc_charge_path_active ? 1U : 0U,
               s_battery_adc_usb_charge_low_hold ? 1U : 0U);
}

static bool battery_adc_should_reject_suspect_env_raw(uint16_t raw_avg)
{
#if ZY100_BAT_ADC_SUSPECT_REJECT_ENABLE
    uint16_t raw_for_filter = battery_adc_clamp_raw_for_conversion(raw_avg);

    if (!s_battery_adc_charge_report_present ||
        !s_battery_adc_charge_report_full ||
        !s_battery_adc_filter_valid)
    {
        return false;
    }

    if (raw_for_filter >= s_battery_adc_filtered_raw)
    {
        return false;
    }

    return ((uint16_t)(s_battery_adc_filtered_raw - raw_for_filter) >
            BATTERY_ADC_MAX_SPREAD_RAW);
#else
    (void)raw_avg;
    return false;
#endif
}

static void battery_adc_log_suspect_env_reject(uint16_t raw_avg)
{
    if (s_battery_adc_log_enabled)
    {
        DBG_DIRECT("[BAT_ADC] reject suspect_env raw=%u filt=%u full=%u",
                   raw_avg,
                   s_battery_adc_filtered_raw,
                   s_battery_adc_charge_report_full ? 1U : 0U);
    }
}

static void battery_adc_update_report_values(void)
{
    uint8_t report_percent = s_battery_adc_percent;
    uint16_t report_mv = s_battery_adc_vbat_mv;

    if (!s_battery_adc_charge_report_present)
    {
        s_battery_adc_report_percent = report_percent;
        s_battery_adc_report_vbat_mv = report_mv;
        return;
    }

    if (s_battery_adc_charge_report_full)
    {
        s_battery_adc_report_percent = 100U;
#if ZY100_BUILD_PRODUCTION
        s_battery_adc_report_vbat_mv = report_mv;
#else
        s_battery_adc_report_vbat_mv =
            (report_mv < BATTERY_FULL_MV) ? BATTERY_FULL_MV : report_mv;
#endif
        return;
    }

    if (report_percent > BATTERY_ADC_CHARGE_REPORT_MAX_PERCENT)
    {
        report_percent = BATTERY_ADC_CHARGE_REPORT_MAX_PERCENT;
    }

    s_battery_adc_report_percent = report_percent;
    s_battery_adc_report_vbat_mv = report_mv;
}

static void battery_adc_sort_samples(uint16_t *samples, uint16_t count)
{
    uint16_t i;

    for (i = 1U; i < count; i++)
    {
        uint16_t value = samples[i];
        uint16_t j = i;

        while ((j > 0U) && (samples[j - 1U] > value))
        {
            samples[j] = samples[j - 1U];
            j--;
        }
        samples[j] = value;
    }
}

static uint16_t battery_adc_trimmed_average(void)
{
    const uint16_t keep_count = BATTERY_ADC_SAMPLE_COUNT - (BATTERY_ADC_TRIM_COUNT * 2U);
    uint32_t raw_sum = 0U;
    uint16_t i;
#if BATTERY_ADC_DUMP_RAW_SAMPLES
    uint16_t *sorted_samples = s_battery_adc_sorted_samples;

    for (i = 0U; i < BATTERY_ADC_SAMPLE_COUNT; i++)
    {
        sorted_samples[i] = s_battery_adc_samples[i];
    }
#else
    uint16_t *sorted_samples = s_battery_adc_samples;
#endif

    battery_adc_sort_samples(sorted_samples, BATTERY_ADC_SAMPLE_COUNT);

    for (i = BATTERY_ADC_TRIM_COUNT; i < (BATTERY_ADC_SAMPLE_COUNT - BATTERY_ADC_TRIM_COUNT); i++)
    {
        raw_sum += sorted_samples[i];
    }

    return (uint16_t)((raw_sum + (keep_count / 2U)) / keep_count);
}

static bool battery_adc_apply_valid_raw(uint16_t raw_avg)
{
    uint16_t raw_for_filter = battery_adc_clamp_raw_for_conversion(raw_avg);
    uint16_t filter_before = s_battery_adc_filtered_raw;
    uint16_t filtered_candidate = filter_before;
    uint16_t filter_step = 0U;
    uint16_t calibrated_vbat_mv = 0U;
    int8_t calibration_error =
        (int8_t)BSP_BATTERY_ADC_STATUS_CALIBRATION_UNAVAILABLE;
    uint8_t filter_action = BATTERY_ADC_FILTER_ACTION_HOLD;
    uint8_t filter_direction = BATTERY_ADC_FILTER_DIR_NONE;

    if (!s_battery_adc_filter_valid)
    {
        filtered_candidate = raw_for_filter;
        filter_action = BATTERY_ADC_FILTER_ACTION_INIT;
    }
    else if (raw_for_filter == filter_before)
    {
        filter_action = BATTERY_ADC_FILTER_ACTION_HOLD;
    }
    else if (battery_adc_should_fast_accept_raw(raw_for_filter,
                                                filter_before))
    {
        filter_step = battery_adc_raw_abs_delta(filter_before,
                                                raw_for_filter);
        filter_direction = battery_adc_filter_direction_from_raw(filter_before,
                                                                 raw_for_filter);
        filtered_candidate = raw_for_filter;
        filter_action = BATTERY_ADC_FILTER_ACTION_FAST;
    }
    else
    {
        filtered_candidate =
            battery_adc_filter_move_towards(filter_before,
                                            raw_for_filter,
                                            &filter_step,
                                            &filter_direction);
        filter_action = BATTERY_ADC_FILTER_ACTION_STEP;
    }

    if (!battery_adc_convert_raw_sdk(filtered_candidate,
                                     &calibrated_vbat_mv,
                                     &calibration_error))
    {
        if (s_battery_adc_log_enabled)
        {
            DBG_DIRECT("[BAT_ADC] fail sdk_cal raw=%u err=%d",
                       filtered_candidate,
                       (int32_t)calibration_error);
        }
        return false;
    }

    s_battery_adc_last_raw = raw_avg;
    s_battery_adc_last_quarantine = false;
    s_battery_adc_filtered_raw = filtered_candidate;
    s_battery_adc_filter_valid = true;
    battery_adc_note_filter_diag(raw_for_filter,
                                 filter_before,
                                 filter_step,
                                 filter_action,
                                 filter_direction);

    s_battery_adc_vbat_mv = calibrated_vbat_mv;
    s_battery_adc_percent = battery_adc_vbat_mv_to_percent(calibrated_vbat_mv);
    battery_adc_update_report_values();
    if (battery_adc_should_update_trusted_raw(raw_for_filter))
    {
        battery_adc_note_trusted_from_current();
    }
    return true;
}

static bool battery_adc_config_driver(void)
{
    return bsp_battery_adc_hw_enable() == BSP_BATTERY_ADC_STATUS_OK;
}

static void battery_adc_power_down_adc(void)
{
    bsp_battery_adc_hw_disable();
}

static bool battery_adc_read_one_raw(uint16_t *raw_out)
{
    bsp_battery_adc_status_t status =
        bsp_battery_adc_read_one_shot(raw_out);

    return (status == BSP_BATTERY_ADC_STATUS_OK) ||
           (status == BSP_BATTERY_ADC_STATUS_RAW_OUT_OF_RANGE);
}

bool battery_adc_guard_convert_raw(uint16_t raw,
                                   battery_adc_guard_one_shot_t *out)
{
    uint16_t calibrated_vbat_mv = 0U;

    if (out == NULL)
    {
        return false;
    }

    out->valid = false;
    out->raw = raw;
    out->battery_mv = 0U;
    out->percent = 0U;

    if (!battery_adc_raw_in_range(raw))
    {
        return false;
    }

    if (!battery_adc_convert_raw_sdk(raw,
                                     &calibrated_vbat_mv,
                                     NULL))
    {
        return false;
    }

    out->battery_mv = calibrated_vbat_mv;
    out->percent = battery_adc_vbat_mv_to_percent(calibrated_vbat_mv);
    out->valid = true;
    return true;
}

bool battery_adc_guard_sample_one_shot(battery_adc_guard_one_shot_t *out)
{
    bool ok;
    bool owns_session = false;

    if (!s_battery_adc_guard_session_active)
    {
        if (!battery_adc_guard_session_begin())
        {
            return false;
        }
        owns_session = true;
    }

    ok = battery_adc_guard_session_read(out);
    if (owns_session)
    {
        battery_adc_guard_session_end();
    }
    return ok;
}

bool battery_adc_guard_session_begin(void)
{
    if (!s_battery_adc_inited)
    {
        battery_adc_record_status(BATTERY_ADC_STATUS_NOT_INIT);
        return false;
    }

    if (s_battery_adc_guard_session_active)
    {
        return true;
    }

    if (s_battery_adc_busy || (s_battery_adc_state != BATTERY_ADC_STAGE_IDLE))
    {
        battery_adc_record_status(BATTERY_ADC_STATUS_BUSY);
        return false;
    }

    s_battery_adc_busy = true;
    s_battery_adc_state = BATTERY_ADC_STAGE_GUARD_SESSION;
    s_battery_adc_last_stage = BATTERY_ADC_STAGE_GUARD_SESSION;
    battery_adc_prepare_pin_for_runtime();

    if (!battery_adc_config_driver())
    {
        battery_adc_power_down_adc();
        battery_adc_park_pin_low_power();
        s_battery_adc_busy = false;
        s_battery_adc_state = BATTERY_ADC_STAGE_IDLE;
        battery_adc_record_status(BATTERY_ADC_STATUS_CONFIG_FAIL);
        return false;
    }

    s_battery_adc_guard_session_active = true;
    battery_adc_record_status(BATTERY_ADC_STATUS_OK);
    return true;
}

bool battery_adc_guard_session_read(battery_adc_guard_one_shot_t *out)
{
    battery_adc_status_t status = BATTERY_ADC_STATUS_OK;
    uint16_t raw = 0U;

    if (out == NULL)
    {
        return false;
    }

    out->valid = false;
    out->raw = 0U;
    out->battery_mv = 0U;
    out->percent = 0U;

    if (!s_battery_adc_inited)
    {
        battery_adc_record_status(BATTERY_ADC_STATUS_NOT_INIT);
        return false;
    }

    if (!s_battery_adc_guard_session_active ||
        !s_battery_adc_busy ||
        (s_battery_adc_state != BATTERY_ADC_STAGE_GUARD_SESSION))
    {
        battery_adc_record_status(BATTERY_ADC_STATUS_BUSY);
        return false;
    }

    s_battery_adc_last_stage = BATTERY_ADC_STAGE_GUARD_ONE_SHOT;

    if (!battery_adc_read_one_raw(&raw))
    {
        status = BATTERY_ADC_STATUS_TIMEOUT;
        battery_adc_record_status(status);
        return false;
    }

    if (!battery_adc_guard_convert_raw(raw, out))
    {
        out->raw = raw;
        status = battery_adc_raw_in_range(raw) ?
                 BATTERY_ADC_STATUS_CALIBRATION_FAIL :
                 BATTERY_ADC_STATUS_RAW_RANGE_FAIL;
        battery_adc_record_status(status);
        return false;
    }

    battery_adc_record_status(status);
    return true;
}

bool battery_adc_guard_session_read_sdk(battery_adc_guard_one_shot_t *out)
{
    battery_adc_status_t status = BATTERY_ADC_STATUS_OK;
    uint16_t raw = 0U;
    uint16_t sdk_vbat_mv = 0U;

    if (out == NULL)
    {
        return false;
    }

    out->valid = false;
    out->raw = 0U;
    out->battery_mv = 0U;
    out->percent = 0U;

    if (!s_battery_adc_inited)
    {
        battery_adc_record_status(BATTERY_ADC_STATUS_NOT_INIT);
        return false;
    }

    if (!s_battery_adc_guard_session_active ||
        !s_battery_adc_busy ||
        (s_battery_adc_state != BATTERY_ADC_STAGE_GUARD_SESSION))
    {
        battery_adc_record_status(BATTERY_ADC_STATUS_BUSY);
        return false;
    }

    s_battery_adc_last_stage = BATTERY_ADC_STAGE_GUARD_ONE_SHOT;
    if (!battery_adc_read_one_raw(&raw))
    {
        battery_adc_record_status(BATTERY_ADC_STATUS_TIMEOUT);
        return false;
    }

    out->raw = raw;
    if (!battery_adc_raw_in_range(raw) ||
        (bsp_battery_adc_raw_to_vbat_mv(raw, &sdk_vbat_mv) !=
         BSP_BATTERY_ADC_STATUS_OK))
    {
        status = battery_adc_raw_in_range(raw) ?
                 BATTERY_ADC_STATUS_CALIBRATION_FAIL :
                 BATTERY_ADC_STATUS_RAW_RANGE_FAIL;
        battery_adc_record_status(status);
        return false;
    }

    out->battery_mv = sdk_vbat_mv;
    out->percent = battery_adc_vbat_mv_to_percent(sdk_vbat_mv);
    out->valid = true;
    battery_adc_record_status(status);
    return true;
}

void battery_adc_guard_session_end(void)
{
    if (!s_battery_adc_guard_session_active)
    {
        return;
    }

    battery_adc_power_down_adc();
    battery_adc_park_pin_low_power();
    s_battery_adc_guard_session_active = false;
    s_battery_adc_busy = false;
    s_battery_adc_state = BATTERY_ADC_STAGE_IDLE;
}

static void battery_adc_set_burst_status(battery_adc_stage_t stage,
                                         battery_adc_status_t status,
                                         uint16_t raw_for_log)
{
    s_battery_adc_burst_stage = stage;
    s_battery_adc_burst_status = status;
    s_battery_adc_burst_fail_raw = raw_for_log;
}

#if BATTERY_ADC_DUMP_RAW_SAMPLES
static void battery_adc_mark_raw_samples_valid(void)
{
    s_battery_adc_raw_samples_valid = true;
}

static void battery_adc_dump_raw_samples(uint32_t total_ms)
{
    uint16_t base;

    if (!s_battery_adc_raw_samples_valid)
    {
        return;
    }
    if (!s_battery_adc_log_enabled)
    {
        return;
    }

    DBG_DIRECT("[BAT_ADC][RAW] retry=%u total_ms=%u count=%u min=%u max=%u spread=%u",
               s_battery_adc_last_retry,
               total_ms,
               BATTERY_ADC_SAMPLE_COUNT,
               s_battery_adc_last_min,
               s_battery_adc_last_max,
               s_battery_adc_last_spread);

    for (base = 0U; base < BATTERY_ADC_SAMPLE_COUNT; base += BATTERY_ADC_RAW_DUMP_PER_LINE)
    {
        DBG_DIRECT("[BAT_ADC][RAW] idx=%u: %u %u %u %u %u %u %u %u",
                   base,
                   s_battery_adc_samples[base + 0U],
                   s_battery_adc_samples[base + 1U],
                   s_battery_adc_samples[base + 2U],
                   s_battery_adc_samples[base + 3U],
                   s_battery_adc_samples[base + 4U],
                   s_battery_adc_samples[base + 5U],
                   s_battery_adc_samples[base + 6U],
                   s_battery_adc_samples[base + 7U]);
    }
}
#else
static void battery_adc_mark_raw_samples_valid(void)
{
}

static void battery_adc_dump_raw_samples(uint32_t total_ms)
{
    (void)total_ms;
}
#endif

static void battery_adc_reset_attempt(void)
{
    s_battery_adc_raw_min = BATTERY_ADC_RAW_MAX;
    s_battery_adc_raw_max = 0U;
    s_battery_adc_burst_raw_avg = 0U;
    s_battery_adc_burst_fail_raw = 0U;
    s_battery_adc_last_burst_ms = 0U;
#if BATTERY_ADC_DUMP_RAW_SAMPLES
    s_battery_adc_raw_samples_valid = false;
#endif
    battery_adc_set_burst_status(BATTERY_ADC_STAGE_IDLE, BATTERY_ADC_STATUS_OK, 0U);
}

static void battery_adc_notify_sample_window(bool active)
{
    if (s_battery_adc_sample_window_cb != NULL)
    {
        s_battery_adc_sample_window_cb(active, s_battery_adc_sample_window_ctx);
    }
}

static bool battery_adc_run_fast_burst(uint16_t *raw_avg,
                                       uint16_t *raw_min,
                                       uint16_t *raw_max,
                                       uint16_t *spread)
{
    uint32_t burst_start_ms;
    uint32_t burst_end_ms;
    uint16_t i;
    uint16_t raw = 0U;
    bool ok = false;

    if ((raw_avg == NULL) || (raw_min == NULL) || (raw_max == NULL) || (spread == NULL))
    {
        return false;
    }

    battery_adc_reset_attempt();
    s_battery_adc_last_stage = BATTERY_ADC_STAGE_FAST_BURST;
    battery_adc_set_burst_status(BATTERY_ADC_STAGE_FAST_BURST, BATTERY_ADC_STATUS_OK, 0U);
    burst_start_ms = battery_adc_now_ms();
    battery_adc_notify_sample_window(true);

    if (!battery_adc_config_driver())
    {
        battery_adc_set_burst_status(BATTERY_ADC_STAGE_FAST_BURST,
                                     BATTERY_ADC_STATUS_CONFIG_FAIL,
                                     0U);
        goto cleanup;
    }

    for (i = 0U; i < BATTERY_ADC_WARMUP_COUNT; i++)
    {
        if (!battery_adc_read_one_raw(&raw))
        {
            battery_adc_set_burst_status(BATTERY_ADC_STAGE_FAST_BURST,
                                         BATTERY_ADC_STATUS_TIMEOUT,
                                         0U);
            goto cleanup;
        }
        if (!battery_adc_raw_in_range(raw))
        {
            battery_adc_set_burst_status(BATTERY_ADC_STAGE_FAST_BURST,
                                         BATTERY_ADC_STATUS_RAW_RANGE_FAIL,
                                         raw);
            goto cleanup;
        }
    }

    for (i = 0U; i < BATTERY_ADC_SAMPLE_COUNT; i++)
    {
        if (!battery_adc_read_one_raw(&raw))
        {
            battery_adc_set_burst_status(BATTERY_ADC_STAGE_FAST_BURST,
                                         BATTERY_ADC_STATUS_TIMEOUT,
                                         0U);
            goto cleanup;
        }
        if (!battery_adc_raw_in_range(raw))
        {
            battery_adc_set_burst_status(BATTERY_ADC_STAGE_FAST_BURST,
                                         BATTERY_ADC_STATUS_RAW_RANGE_FAIL,
                                         raw);
            goto cleanup;
        }

        s_battery_adc_samples[i] = raw;
        if (raw < s_battery_adc_raw_min)
        {
            s_battery_adc_raw_min = raw;
        }
        if (raw > s_battery_adc_raw_max)
        {
            s_battery_adc_raw_max = raw;
        }
    }

    *raw_min = s_battery_adc_raw_min;
    *raw_max = s_battery_adc_raw_max;
    *spread = (uint16_t)(s_battery_adc_raw_max - s_battery_adc_raw_min);
    battery_adc_mark_raw_samples_valid();
    *raw_avg = battery_adc_trimmed_average();
    s_battery_adc_burst_raw_avg = *raw_avg;
    ok = battery_adc_raw_in_range(*raw_avg);
    if (!ok)
    {
        battery_adc_set_burst_status(BATTERY_ADC_STAGE_FAST_BURST,
                                     BATTERY_ADC_STATUS_RAW_RANGE_FAIL,
                                     *raw_avg);
    }

cleanup:
    battery_adc_power_down_adc();
    battery_adc_notify_sample_window(false);
    burst_end_ms = battery_adc_now_ms();
    s_battery_adc_last_burst_ms = battery_adc_elapsed_ms(burst_start_ms, burst_end_ms);
    return ok;
}

static void battery_adc_begin_attempt(uint32_t runtime_ms, uint32_t settle_ms)
{
    s_battery_charge_sample.valid = false;
    battery_adc_reset_attempt();
    battery_adc_prepare_pin_for_runtime();
    s_battery_adc_state = BATTERY_ADC_STAGE_PREPARE_ADC;
    s_battery_adc_last_stage = BATTERY_ADC_STAGE_PREPARE_ADC;
    if (s_battery_adc_retry_count == 0U)
    {
        s_battery_adc_attempt_start_ms = runtime_ms;
    }
    s_battery_adc_state = BATTERY_ADC_STAGE_SETTLE_WAIT;
    s_battery_adc_last_stage = BATTERY_ADC_STAGE_SETTLE_WAIT;
    s_battery_adc_next_action_ms = runtime_ms + settle_ms;
}

static void battery_adc_fail_runtime(uint32_t runtime_ms,
                                     battery_adc_stage_t stage,
                                     battery_adc_status_t status,
                                     uint16_t raw_for_log,
                                     uint32_t reschedule_ms)
{
    s_battery_adc_last_total_ms = battery_adc_elapsed_ms(s_battery_adc_attempt_start_ms,
                                                         runtime_ms);
    battery_adc_power_down_adc();
    if (status == BATTERY_ADC_STATUS_SUSPECT_ENV)
    {
        battery_adc_park_pin_low_power();
    }
    s_battery_adc_busy = false;
    s_battery_adc_state = BATTERY_ADC_STAGE_IDLE;
    s_battery_adc_last_stage = stage;
    battery_adc_record_status(status);
    s_battery_adc_fail_count++;
    battery_adc_schedule_after(runtime_ms, reschedule_ms);
    if (s_battery_adc_log_enabled)
    {
        DBG_DIRECT("[BAT_ADC] fail stage=%s status=%s raw=%u mode=%s",
                   battery_adc_stage_str(stage),
                   battery_adc_status_str(status),
                   raw_for_log,
                   battery_adc_mode_str());
    }
}

static void battery_adc_defer_pending_attempt(uint32_t runtime_ms)
{
    s_battery_adc_last_total_ms =
        battery_adc_elapsed_ms(s_battery_adc_attempt_start_ms, runtime_ms);
    battery_adc_power_down_adc();
    s_battery_adc_busy = false;
    s_battery_adc_state = BATTERY_ADC_STAGE_IDLE;
    s_battery_adc_last_stage = BATTERY_ADC_STAGE_IDLE;
    battery_adc_record_status(BATTERY_ADC_STATUS_DEFERRED);
    battery_adc_mark_due_pending(runtime_ms);
}

static void battery_adc_log_burst(uint16_t raw_avg, uint32_t total_ms)
{
    (void)total_ms;
    (void)s_battery_adc_last_burst_ms;
#if ZY100_PRODUCT_LOG_QUIET_ENABLE
    (void)s_battery_adc_report_vbat_mv;
#endif

#if !ZY100_LOG_PERIODIC_STATUS_ENABLE
    (void)raw_avg;
    return;
#else
    if (!s_battery_adc_log_enabled)
    {
        return;
    }

    BATTERY_ADC_INFO_LOG("[BAT] pct=%u mv=%u raw=%u filt=%u",
                         s_battery_adc_report_percent,
                         s_battery_adc_report_vbat_mv,
                         raw_avg,
                         s_battery_adc_filtered_raw);
#endif
}

static void battery_adc_accept_charge_sample(uint16_t raw_avg, uint32_t now_ms)
{
    uint16_t sdk_mv = 0U;
    s_battery_charge_sample.valid = false;
    if (!battery_adc_convert_raw_sdk(raw_avg, &sdk_mv, NULL)) return;
    ++s_battery_charge_sample.sequence;
    if (s_battery_charge_sample.sequence == 0U) ++s_battery_charge_sample.sequence;
    s_battery_charge_sample.sampled_ms = now_ms;
    s_battery_charge_sample.voltage_mv = sdk_mv;
    s_battery_charge_sample.valid = true;
}

static bool battery_adc_finish_success_runtime(uint32_t runtime_ms, uint16_t raw_avg)
{
    s_battery_adc_last_total_ms = battery_adc_elapsed_ms(s_battery_adc_attempt_start_ms,
                                                         runtime_ms);
    if (!battery_adc_apply_valid_raw(raw_avg))
    {
        battery_adc_fail_runtime(runtime_ms,
                                 BATTERY_ADC_STAGE_FINALIZE,
                                 BATTERY_ADC_STATUS_CALIBRATION_FAIL,
                                 raw_avg,
                                 BATTERY_ADC_PERIOD_MS);
        return false;
    }
    battery_adc_power_down_adc();
    s_battery_adc_busy = false;
    s_battery_adc_state = BATTERY_ADC_STAGE_IDLE;
    s_battery_adc_last_stage = BATTERY_ADC_STAGE_FINALIZE;
    battery_adc_record_status(BATTERY_ADC_STATUS_OK);
    battery_adc_accept_charge_sample(raw_avg, runtime_ms);
    s_battery_adc_ok_count++;
    battery_adc_schedule_after(runtime_ms, BATTERY_ADC_PERIOD_MS);
    battery_adc_log_burst(raw_avg, s_battery_adc_last_total_ms);
    battery_adc_dump_raw_samples(s_battery_adc_last_total_ms);
    return true;
}

static bool battery_adc_finish_usb_charge_quarantine_runtime(uint32_t runtime_ms,
                                                             uint16_t raw_avg)
{
    bool held;
    uint16_t raw_for_filter = battery_adc_clamp_raw_for_conversion(raw_avg);
    uint16_t filter_before = s_battery_adc_filtered_raw;

    s_battery_adc_last_total_ms = battery_adc_elapsed_ms(s_battery_adc_attempt_start_ms,
                                                         runtime_ms);
    s_battery_adc_last_raw = raw_avg;
    s_battery_adc_last_quarantine = true;
    s_battery_adc_usb_charge_low_hold = true;
    held = battery_adc_restore_trusted_baseline();
    battery_adc_note_filter_diag(
        raw_for_filter,
        filter_before,
        battery_adc_raw_abs_delta(filter_before, s_battery_adc_filtered_raw),
        BATTERY_ADC_FILTER_ACTION_QUARANTINE,
        battery_adc_filter_direction_from_raw(filter_before,
                                              s_battery_adc_filtered_raw));
    battery_adc_power_down_adc();
    if (!held)
    {
        battery_adc_park_pin_low_power();
    }
    s_battery_adc_busy = false;
    s_battery_adc_state = BATTERY_ADC_STAGE_IDLE;
    s_battery_adc_last_stage = BATTERY_ADC_STAGE_FINALIZE;
    battery_adc_record_status(BATTERY_ADC_STATUS_USB_CHARGE_QUARANTINE);
    battery_adc_log_usb_charge_quarantine(raw_avg, held);

    if (held)
    {
        s_battery_adc_ok_count++;
        battery_adc_schedule_after(runtime_ms, BATTERY_ADC_PERIOD_MS);
        battery_adc_log_burst(raw_avg, s_battery_adc_last_total_ms);
        battery_adc_dump_raw_samples(s_battery_adc_last_total_ms);
        return true;
    }

    s_battery_adc_fail_count++;
    battery_adc_schedule_after(runtime_ms,
                               BATTERY_ADC_USB_CHARGE_QUARANTINE_RETRY_MS);
    return false;
}

static bool battery_adc_finalize_runtime(uint32_t runtime_ms)
{
    uint16_t raw_avg;
    uint16_t sdk_vbat_mv;
    uint32_t finish_ms;
    uint32_t total_ms;

    raw_avg = s_battery_adc_burst_raw_avg;
    finish_ms = battery_adc_now_ms();
    if (battery_adc_time_reached(runtime_ms, finish_ms))
    {
        finish_ms = runtime_ms;
    }
    total_ms = battery_adc_elapsed_ms(s_battery_adc_attempt_start_ms, finish_ms);

    if (s_battery_adc_burst_status != BATTERY_ADC_STATUS_OK)
    {
        battery_adc_fail_runtime(finish_ms,
                                 s_battery_adc_burst_stage,
                                 s_battery_adc_burst_status,
                                 s_battery_adc_burst_fail_raw,
                                 BATTERY_ADC_PERIOD_MS);
        return false;
    }

    s_battery_adc_last_stage = BATTERY_ADC_STAGE_FINALIZE;
    s_battery_adc_last_min = s_battery_adc_raw_min;
    s_battery_adc_last_max = s_battery_adc_raw_max;
    s_battery_adc_last_spread = s_battery_adc_raw_max - s_battery_adc_raw_min;
    s_battery_adc_last_retry = s_battery_adc_retry_count;

    if (!battery_adc_raw_in_range(raw_avg))
    {
        battery_adc_fail_runtime(finish_ms,
                                 BATTERY_ADC_STAGE_FINALIZE,
                                 BATTERY_ADC_STATUS_RAW_RANGE_FAIL,
                                 raw_avg,
                                 BATTERY_ADC_PERIOD_MS);
        return false;
    }

    if (!battery_adc_convert_raw_sdk(raw_avg, &sdk_vbat_mv, NULL))
    {
        battery_adc_fail_runtime(finish_ms,
                                 BATTERY_ADC_STAGE_FINALIZE,
                                 BATTERY_ADC_STATUS_CALIBRATION_FAIL,
                                 raw_avg,
                                 BATTERY_ADC_PERIOD_MS);
        return false;
    }

    if (s_battery_adc_last_spread > BATTERY_ADC_MAX_SPREAD_RAW)
    {
        battery_adc_log_burst(raw_avg, total_ms);
        battery_adc_dump_raw_samples(total_ms);
        if (s_battery_adc_log_enabled)
        {
            DBG_DIRECT("[BAT_ADC] reject spread=%u limit=%u retry=%u min=%u max=%u",
                       s_battery_adc_last_spread,
                       BATTERY_ADC_MAX_SPREAD_RAW,
                       s_battery_adc_retry_count,
                       s_battery_adc_last_min,
                       s_battery_adc_last_max);
        }
        if (s_battery_adc_retry_count < BATTERY_ADC_MAX_RETRY)
        {
            s_battery_adc_retry_count++;
            battery_adc_begin_attempt(finish_ms, BATTERY_ADC_RETRY_SETTLE_MS);
            return false;
        }

        battery_adc_note_rejected_filter_raw(raw_avg,
                                             BATTERY_ADC_FILTER_ACTION_REJECT);
        battery_adc_fail_runtime(finish_ms,
                                 BATTERY_ADC_STAGE_FINALIZE,
                                 BATTERY_ADC_STATUS_SPREAD_REJECT,
                                 raw_avg,
                                 BATTERY_ADC_PERIOD_MS);
        return false;
    }

    if (battery_adc_should_quarantine_usb_charge_raw(raw_avg))
    {
        return battery_adc_finish_usb_charge_quarantine_runtime(finish_ms,
                                                               raw_avg);
    }

    if (battery_adc_should_reject_suspect_env_raw(raw_avg))
    {
        battery_adc_log_suspect_env_reject(raw_avg);
        battery_adc_note_rejected_filter_raw(raw_avg,
                                             BATTERY_ADC_FILTER_ACTION_REJECT);
        battery_adc_fail_runtime(finish_ms,
                                 BATTERY_ADC_STAGE_FINALIZE,
                                 BATTERY_ADC_STATUS_SUSPECT_ENV,
                                 raw_avg,
                                 BATTERY_ADC_DEFER_MS);
        return false;
    }

    return battery_adc_finish_success_runtime(finish_ms, raw_avg);
}

bool battery_adc_init(void)
{
    bsp_battery_adc_info_t bsp_info = {0};
    bsp_battery_adc_status_t bsp_status;

    s_battery_adc_inited = true;
    s_battery_adc_busy = false;
    s_battery_adc_guard_session_active = false;
    s_battery_adc_schedule_valid = false;
    s_battery_adc_due_pending = false;
    s_battery_adc_filter_valid = false;
    s_battery_adc_next_due_ms = 0U;
    s_battery_adc_next_action_ms = 0U;
    s_battery_charge_sample.valid = false;
    s_battery_adc_last_raw = 0U;
    s_battery_adc_filtered_raw = 0U;
    s_battery_adc_vbat_mv = 0U;
    s_battery_adc_last_min = 0U;
    s_battery_adc_last_max = 0U;
    s_battery_adc_last_spread = 0U;
    s_battery_adc_burst_raw_avg = 0U;
    s_battery_adc_burst_fail_raw = 0U;
    s_battery_adc_last_retry = 0U;
    s_battery_adc_percent = 0U;
    s_battery_adc_report_percent = 0U;
    s_battery_adc_report_vbat_mv = 0U;
    s_battery_adc_charge_report_present = false;
    s_battery_adc_charge_report_full = false;
    s_battery_adc_charge_path_active = false;
    s_battery_adc_last_quarantine = false;
    s_battery_adc_filter_raw_for_filter = 0U;
    s_battery_adc_filter_before_raw = 0U;
    s_battery_adc_filter_step_raw = 0U;
    s_battery_adc_filter_action = BATTERY_ADC_FILTER_ACTION_NONE;
    s_battery_adc_filter_direction = BATTERY_ADC_FILTER_DIR_NONE;
    s_battery_adc_retry_count = 0U;
    s_battery_adc_burst_status = BATTERY_ADC_STATUS_OK;
    s_battery_adc_state = BATTERY_ADC_STAGE_IDLE;
    s_battery_adc_last_stage = BATTERY_ADC_STAGE_IDLE;
    s_battery_adc_burst_stage = BATTERY_ADC_STAGE_IDLE;
    s_battery_adc_attempt_start_ms = 0U;
    s_battery_adc_last_burst_ms = 0U;
    s_battery_adc_last_total_ms = 0U;
    s_battery_adc_last_status = BATTERY_ADC_STATUS_OK;
    s_battery_adc_ok_count = 0U;
    s_battery_adc_fail_count = 0U;
    s_battery_adc_quarantine_log_valid = false;
    s_battery_adc_quarantine_log_held = false;
    s_battery_adc_quarantine_log_trusted = false;
    s_battery_adc_quarantine_log_path = false;
    s_battery_adc_quarantine_log_latch = false;
    s_battery_adc_quarantine_log_ms = 0U;
    s_battery_adc_quarantine_log_trusted_raw = 0U;
    s_battery_adc_quarantine_log_filtered_raw = 0U;
#if BATTERY_ADC_DUMP_RAW_SAMPLES
    s_battery_adc_raw_samples_valid = false;
#endif
    battery_adc_reset_attempt();
    bsp_status = bsp_battery_adc_init();
    bsp_battery_adc_get_info(&bsp_info);
    if (s_battery_adc_log_enabled)
    {
        BATTERY_ADC_INFO_LOG("[BAT_ADC] calibration_init ok=%u src=sdk",
                             bsp_info.calibration_ready ? 1U : 0U);
    }
    if (s_battery_adc_log_enabled)
    {
        BATTERY_ADC_INFO_LOG("[BAT_ADC] init pin=%u ch=%u adc=B div=715/160 src=sdk cal=%u spread=%u",
                             bsp_info.pin,
                             bsp_info.channel,
                             bsp_info.calibration_ready ? 1U : 0U,
                             BATTERY_ADC_MAX_SPREAD_RAW);
    }
    (void)battery_adc_calibration_reload();
    (void)bsp_status;
    return true;
}

bool battery_adc_calibration_reload(void)
{
    uint8_t record[ZY100_CAL_RECORD_MAX_BYTES];
    zy100_cal_record_info_t info;
    zy100_cal_battery_gain_v1_t battery;
    zy100_cal_store_status_t status;
    uint8_t actual_bt_address[6];

    memset(record, 0, sizeof(record));
    memset(&info, 0, sizeof(info));
    memset(&battery, 0, sizeof(battery));
    status = zy100_cal_store_load(record, &info);
    if (status != ZY100_CAL_STORE_OK)
    {
        status = zy100_cal_store_stage_load(record, &info);
    }
    if ((status != ZY100_CAL_STORE_OK) ||
        !zy100_cal_record_get_battery_gain(record, info.record_bytes, &battery) ||
        (gap_get_param(GAP_PARAM_BD_ADDR, actual_bt_address) != GAP_CAUSE_SUCCESS) ||
        (memcmp(actual_bt_address, battery.bt_address,
                sizeof(actual_bt_address)) != 0))
    {
        s_battery_adc_calibration_valid = false;
        s_battery_adc_gain_q20 = 0UL;
        return false;
    }
    s_battery_adc_gain_q20 = battery.gain_q20;
    s_battery_adc_calibration_valid = true;
    BATTERY_ADC_INFO_LOG("[BAT_ADC] battery_gain active=1 q20=%lu generation=%lu",
                         (unsigned long)s_battery_adc_gain_q20,
                         (unsigned long)info.generation);
    return true;
}

bool battery_adc_calibration_active(uint32_t *gain_q20_out)
{
    if (gain_q20_out != NULL)
    {
        *gain_q20_out = s_battery_adc_gain_q20;
    }
    return s_battery_adc_calibration_valid;
}

void battery_adc_calibration_clear_runtime(void)
{
    s_battery_adc_calibration_valid = false;
    s_battery_adc_gain_q20 = 0UL;
}

bool battery_adc_sample_once(void)
{
    battery_adc_status_t status = BATTERY_ADC_STATUS_OK;
    battery_adc_stage_t stage = BATTERY_ADC_STAGE_IDLE;
    uint16_t raw_avg = 0U;
    uint16_t raw_min = 0U;
    uint16_t raw_max = 0U;
    uint16_t spread = 0U;
    uint16_t sdk_vbat_mv = 0U;
    uint32_t finish_ms;
    uint32_t total_ms;
    bool ok = false;

    if (!s_battery_adc_inited)
    {
        battery_adc_record_status(BATTERY_ADC_STATUS_NOT_INIT);
        if (s_battery_adc_log_enabled)
        {
            DBG_DIRECT("[BAT_ADC] skip status=%s",
                       battery_adc_status_str(BATTERY_ADC_STATUS_NOT_INIT));
        }
        return false;
    }

    if (s_battery_adc_busy)
    {
        battery_adc_record_status(BATTERY_ADC_STATUS_BUSY);
        if (s_battery_adc_log_enabled)
        {
            DBG_DIRECT("[BAT_ADC] skip status=%s",
                       battery_adc_status_str(BATTERY_ADC_STATUS_BUSY));
        }
        return false;
    }

    s_battery_adc_busy = true;
    s_battery_adc_state = BATTERY_ADC_STAGE_PREPARE_ADC;
    s_battery_adc_retry_count = 0U;
    s_battery_charge_sample.valid = false;
    s_battery_adc_attempt_start_ms = battery_adc_now_ms();

    for (;;)
    {
        battery_adc_prepare_pin_for_runtime();
        stage = BATTERY_ADC_STAGE_PREPARE_ADC;
        s_battery_adc_state = BATTERY_ADC_STAGE_PREPARE_ADC;
        s_battery_adc_last_stage = stage;
        stage = BATTERY_ADC_STAGE_FAST_BURST;
        s_battery_adc_state = BATTERY_ADC_STAGE_FAST_BURST;
        if (!battery_adc_run_fast_burst(&raw_avg, &raw_min, &raw_max, &spread))
        {
            status = s_battery_adc_burst_status;
            stage = s_battery_adc_burst_stage;
            raw_avg = s_battery_adc_burst_fail_raw;
            break;
        }

        s_battery_adc_state = BATTERY_ADC_STAGE_FINALIZE;
        stage = BATTERY_ADC_STAGE_FINALIZE;
        s_battery_adc_last_min = raw_min;
        s_battery_adc_last_max = raw_max;
        s_battery_adc_last_spread = spread;
        s_battery_adc_last_retry = s_battery_adc_retry_count;
        finish_ms = battery_adc_now_ms();
        total_ms = battery_adc_elapsed_ms(s_battery_adc_attempt_start_ms, finish_ms);

        if (!battery_adc_convert_raw_sdk(raw_avg, &sdk_vbat_mv, NULL))
        {
            status = BATTERY_ADC_STATUS_CALIBRATION_FAIL;
            break;
        }

        if (s_battery_adc_last_spread > BATTERY_ADC_MAX_SPREAD_RAW)
        {
            battery_adc_log_burst(raw_avg, total_ms);
            if (s_battery_adc_log_enabled)
            {
                DBG_DIRECT("[BAT_ADC] reject spread=%u limit=%u retry=%u min=%u max=%u",
                           s_battery_adc_last_spread,
                           BATTERY_ADC_MAX_SPREAD_RAW,
                           s_battery_adc_retry_count,
                           s_battery_adc_last_min,
                           s_battery_adc_last_max);
            }
            if (s_battery_adc_retry_count < BATTERY_ADC_MAX_RETRY)
            {
                s_battery_adc_retry_count++;
                continue;
            }
            battery_adc_note_rejected_filter_raw(raw_avg,
                                                 BATTERY_ADC_FILTER_ACTION_REJECT);
            status = BATTERY_ADC_STATUS_SPREAD_REJECT;
            break;
        }

        if (battery_adc_should_quarantine_usb_charge_raw(raw_avg))
        {
            uint16_t raw_for_filter =
                battery_adc_clamp_raw_for_conversion(raw_avg);
            uint16_t filter_before = s_battery_adc_filtered_raw;

            s_battery_adc_last_raw = raw_avg;
            s_battery_adc_last_quarantine = true;
            s_battery_adc_usb_charge_low_hold = true;
            ok = battery_adc_restore_trusted_baseline();
            battery_adc_note_filter_diag(
                raw_for_filter,
                filter_before,
                battery_adc_raw_abs_delta(filter_before,
                                          s_battery_adc_filtered_raw),
                BATTERY_ADC_FILTER_ACTION_QUARANTINE,
                battery_adc_filter_direction_from_raw(
                    filter_before,
                    s_battery_adc_filtered_raw));
            battery_adc_log_usb_charge_quarantine(raw_avg, ok);
            status = BATTERY_ADC_STATUS_USB_CHARGE_QUARANTINE;
            break;
        }

        if (battery_adc_should_reject_suspect_env_raw(raw_avg))
        {
            battery_adc_log_suspect_env_reject(raw_avg);
            battery_adc_note_rejected_filter_raw(raw_avg,
                                                 BATTERY_ADC_FILTER_ACTION_REJECT);
            status = BATTERY_ADC_STATUS_SUSPECT_ENV;
            break;
        }

        ok = battery_adc_apply_valid_raw(raw_avg);
        if (!ok)
        {
            status = BATTERY_ADC_STATUS_CALIBRATION_FAIL;
        }
        break;
    }

    battery_adc_power_down_adc();
    if ((status == BATTERY_ADC_STATUS_SUSPECT_ENV) ||
        ((status == BATTERY_ADC_STATUS_USB_CHARGE_QUARANTINE) && !ok))
    {
        battery_adc_park_pin_low_power();
    }
    s_battery_adc_busy = false;
    s_battery_adc_state = BATTERY_ADC_STAGE_IDLE;
    s_battery_adc_last_stage = stage;
    battery_adc_record_status(status);

    if (ok)
    {
        finish_ms = battery_adc_now_ms();
        if (status == BATTERY_ADC_STATUS_OK) battery_adc_accept_charge_sample(raw_avg, finish_ms);
        s_battery_adc_last_total_ms = battery_adc_elapsed_ms(s_battery_adc_attempt_start_ms,
                                                             finish_ms);
        s_battery_adc_ok_count++;
        battery_adc_log_burst(raw_avg, s_battery_adc_last_total_ms);
    }
    else
    {
        s_battery_adc_fail_count++;
        battery_adc_record_status(status);
        if (s_battery_adc_log_enabled)
        {
            DBG_DIRECT("[BAT_ADC] fail stage=%s status=%s raw=%u mode=%s",
                       battery_adc_stage_str(s_battery_adc_last_stage),
                       battery_adc_status_str(s_battery_adc_last_status),
                       raw_avg,
                       battery_adc_mode_str());
        }
    }

    return ok;
}

bool battery_adc_sample_test_sdk_summary(battery_adc_test_summary_t *out)
{
    uint16_t raw_avg;
    uint16_t raw_min;
    uint16_t raw_max;
    uint16_t raw_spread;
    uint16_t sdk_mv = 0U;
    bool ok = false;

    if (out == NULL)
    {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->sample_count = BATTERY_ADC_SAMPLE_COUNT;
    if (!s_battery_adc_inited || s_battery_adc_busy)
    {
        return false;
    }

    /* The normal Production sampler updates the runtime filter and applies
     * CAL1.  Test paths must observe the same SDK conversion without that
     * persisted gain, so run the burst directly and convert only its trimmed
     * raw average. */
    s_battery_adc_busy = true;
    s_battery_adc_state = BATTERY_ADC_STAGE_FAST_BURST;
    s_battery_adc_last_stage = BATTERY_ADC_STAGE_FAST_BURST;
    battery_adc_prepare_pin_for_runtime();
    if (!battery_adc_run_fast_burst(&raw_avg, &raw_min, &raw_max, &raw_spread))
    {
        goto cleanup;
    }
    if (bsp_battery_adc_raw_to_vbat_mv(raw_avg, &sdk_mv) !=
        BSP_BATTERY_ADC_STATUS_OK)
    {
        goto cleanup;
    }
    out->valid = true;
    out->raw_avg = raw_avg;
    out->raw_min = raw_min;
    out->raw_max = raw_max;
    out->raw_spread = raw_spread;
    out->sdk_mv = sdk_mv;
    ok = true;

cleanup:
    battery_adc_power_down_adc();
    s_battery_adc_busy = false;
    s_battery_adc_state = BATTERY_ADC_STAGE_IDLE;
    battery_adc_record_status(ok ? BATTERY_ADC_STATUS_OK :
                              BATTERY_ADC_STATUS_CALIBRATION_FAIL);
    return ok;
}

uint16_t battery_adc_get_raw(void)
{
    return s_battery_adc_last_raw;
}

bool battery_adc_get_charge_sample(charge_voltage_sample_t *sample)
{
    if (sample == NULL) return false;
    *sample = s_battery_charge_sample;
    return sample->valid;
}

uint16_t battery_adc_get_voltage_mv(void)
{
    return s_battery_adc_vbat_mv;
}

uint16_t battery_adc_get_charge_gate_voltage_mv(void)
{
    uint16_t calibrated_vbat_mv = 0U;

    if (s_battery_adc_last_raw == 0U)
    {
        return s_battery_adc_vbat_mv;
    }

    if (battery_adc_convert_raw_sdk(s_battery_adc_last_raw,
                                    &calibrated_vbat_mv,
                                    NULL))
    {
        return calibrated_vbat_mv;
    }

    return s_battery_adc_vbat_mv;
}

uint8_t battery_adc_get_percent(void)
{
    return s_battery_adc_report_percent;
}

void battery_adc_set_charge_report_state(bool external_power_present,
                                         bool charge_full_detected)
{
    battery_adc_set_charge_env_state(external_power_present,
                                     charge_full_detected,
                                     false);
}

void battery_adc_set_charge_env_state(bool external_power_present,
                                      bool charge_full_detected,
                                      bool charge_path_active)
{
    s_battery_adc_charge_report_present = external_power_present;
    s_battery_adc_charge_report_full = external_power_present &&
                                      charge_full_detected;
    s_battery_adc_charge_path_active = external_power_present &&
                                      charge_path_active;
    battery_adc_update_report_values();
}

static void battery_adc_fill_diag(battery_adc_diag_t *diag)
{
    if (diag == NULL)
    {
        return;
    }

    diag->ok_count = s_battery_adc_ok_count;
    diag->fail_count = s_battery_adc_fail_count;
    diag->last_status = battery_adc_status_str(s_battery_adc_last_status);
    diag->last_raw = s_battery_adc_last_raw;
    diag->raw_for_filter = s_battery_adc_filter_raw_for_filter;
    diag->filter_before_raw = s_battery_adc_filter_before_raw;
    diag->filtered_raw = s_battery_adc_filtered_raw;
    diag->filter_step_raw = s_battery_adc_filter_step_raw;
    diag->filter_valid = s_battery_adc_filter_valid;
    diag->trusted_valid = s_battery_adc_trusted_valid;
    diag->last_quarantine = s_battery_adc_last_quarantine;
    diag->usb_charge_recovery_hold = s_battery_adc_usb_charge_low_hold;
    diag->last_min = s_battery_adc_last_min;
    diag->last_max = s_battery_adc_last_max;
    diag->last_spread = s_battery_adc_last_spread;
    diag->spread_limit = BATTERY_ADC_MAX_SPREAD_RAW;
    diag->raw_mid = 0U;
    diag->trusted_raw = s_battery_adc_trusted_raw;
    diag->trusted_mv = s_battery_adc_trusted_vbat_mv;
    diag->last_retry = s_battery_adc_last_retry;
    diag->last_percent = s_battery_adc_report_percent;
    diag->trusted_percent = s_battery_adc_trusted_percent;
    diag->filter_action = s_battery_adc_filter_action;
    diag->filter_direction = s_battery_adc_filter_direction;
}

void battery_adc_peek_diag(battery_adc_diag_t *diag)
{
    battery_adc_fill_diag(diag);
}

void battery_adc_take_diag(battery_adc_diag_t *diag)
{
    if (diag == NULL)
    {
        return;
    }

    battery_adc_fill_diag(diag);
    s_battery_adc_ok_count = 0U;
    s_battery_adc_fail_count = 0U;
}

void battery_adc_runtime_force_due(uint32_t runtime_ms)
{
    battery_adc_mark_due_pending(runtime_ms);
}

uint32_t battery_adc_runtime_wait_ms(uint32_t runtime_ms)
{
    if (!s_battery_adc_inited ||
        !s_battery_adc_schedule_valid ||
        s_battery_adc_due_pending ||
        battery_adc_time_reached(runtime_ms, s_battery_adc_next_due_ms))
    {
        return 0U;
    }

    return (uint32_t)(s_battery_adc_next_due_ms - runtime_ms);
}

void battery_adc_runtime_cancel_pending_attempt(uint32_t runtime_ms)
{
    if (s_battery_adc_guard_session_active)
    {
        return;
    }

    if (s_battery_adc_busy || (s_battery_adc_state != BATTERY_ADC_STAGE_IDLE))
    {
        battery_adc_defer_pending_attempt(runtime_ms);
        return;
    }

    battery_adc_mark_due_pending(runtime_ms);
    battery_adc_record_status(BATTERY_ADC_STATUS_DEFERRED);
}

void battery_adc_set_sample_window_callback(battery_adc_sample_window_cb_t cb,
                                            void *ctx)
{
    s_battery_adc_sample_window_cb = cb;
    s_battery_adc_sample_window_ctx = ctx;
}

void battery_adc_set_log_enabled(bool enabled)
{
    s_battery_adc_log_enabled = enabled;
}

bool battery_adc_runtime_tick(uint32_t runtime_ms, bool sample_allowed)
{
    uint16_t raw_avg = 0U;
    uint16_t raw_min = 0U;
    uint16_t raw_max = 0U;
    uint16_t spread = 0U;

    if (!s_battery_adc_inited)
    {
        battery_adc_record_status(BATTERY_ADC_STATUS_NOT_INIT);
        return false;
    }

    if (s_battery_adc_guard_session_active)
    {
        return false;
    }

    if (!s_battery_adc_schedule_valid)
    {
        battery_adc_schedule_after(runtime_ms, BATTERY_ADC_FIRST_DELAY_MS);
        return false;
    }

    if (s_battery_adc_state == BATTERY_ADC_STAGE_IDLE)
    {
        uint32_t settle_ms;

        if (!s_battery_adc_due_pending &&
            !battery_adc_time_reached(runtime_ms, s_battery_adc_next_due_ms))
        {
            return false;
        }

        if (!sample_allowed)
        {
            battery_adc_mark_due_pending(runtime_ms);
            return false;
        }

        s_battery_adc_busy = true;
        s_battery_adc_retry_count = 0U;
        s_battery_adc_due_pending = false;
        settle_ms = (uint32_t)BATTERY_ADC_NON_CAPTURE_SETTLE_MS;
        battery_adc_begin_attempt(runtime_ms, settle_ms);
        if (settle_ms != 0U)
        {
            return false;
        }
    }

    if (!sample_allowed)
    {
        battery_adc_defer_pending_attempt(runtime_ms);
        return false;
    }

    switch (s_battery_adc_state)
    {
    case BATTERY_ADC_STAGE_PREPARE_ADC:
        s_battery_adc_state = BATTERY_ADC_STAGE_SETTLE_WAIT;
        s_battery_adc_last_stage = BATTERY_ADC_STAGE_SETTLE_WAIT;
        return false;

    case BATTERY_ADC_STAGE_SETTLE_WAIT:
        if (!battery_adc_time_reached(runtime_ms, s_battery_adc_next_action_ms))
        {
            return false;
        }
        s_battery_adc_state = BATTERY_ADC_STAGE_FAST_BURST;
        s_battery_adc_last_stage = BATTERY_ADC_STAGE_FAST_BURST;
        (void)battery_adc_run_fast_burst(&raw_avg, &raw_min, &raw_max, &spread);
        s_battery_adc_raw_min = raw_min;
        s_battery_adc_raw_max = raw_max;
        s_battery_adc_last_min = raw_min;
        s_battery_adc_last_max = raw_max;
        s_battery_adc_last_spread = spread;
        s_battery_adc_burst_raw_avg = raw_avg;
        s_battery_adc_state = BATTERY_ADC_STAGE_FINALIZE;
        return battery_adc_finalize_runtime(runtime_ms);

    case BATTERY_ADC_STAGE_FAST_BURST:
        s_battery_adc_last_stage = BATTERY_ADC_STAGE_FAST_BURST;
        (void)battery_adc_run_fast_burst(&raw_avg, &raw_min, &raw_max, &spread);
        s_battery_adc_raw_min = raw_min;
        s_battery_adc_raw_max = raw_max;
        s_battery_adc_last_min = raw_min;
        s_battery_adc_last_max = raw_max;
        s_battery_adc_last_spread = spread;
        s_battery_adc_burst_raw_avg = raw_avg;
        s_battery_adc_state = BATTERY_ADC_STAGE_FINALIZE;
        return battery_adc_finalize_runtime(runtime_ms);

    case BATTERY_ADC_STAGE_FINALIZE:
        return battery_adc_finalize_runtime(runtime_ms);

    case BATTERY_ADC_STAGE_IDLE:
    default:
        s_battery_adc_state = BATTERY_ADC_STAGE_IDLE;
        s_battery_adc_busy = false;
        return false;
    }
}

bool battery_adc_runtime_10s_tick(uint32_t runtime_ms)
{
    return battery_adc_runtime_tick(runtime_ms, true);
}

bool battery_adc_is_active(void)
{
    return (s_battery_adc_state == BATTERY_ADC_STAGE_SETTLE_WAIT) ||
           (s_battery_adc_state == BATTERY_ADC_STAGE_FAST_BURST) ||
           s_battery_adc_guard_session_active;
}

void battery_adc_prepare_for_dlps(void)
{
    battery_adc_guard_session_end();
    battery_adc_power_down_adc();
    battery_adc_park_pin_low_power();
    s_battery_adc_busy = false;
    s_battery_adc_guard_session_active = false;
    s_battery_adc_state = BATTERY_ADC_STAGE_IDLE;
    s_battery_adc_schedule_valid = false;
    s_battery_adc_due_pending = false;
    s_battery_adc_next_due_ms = 0U;
    s_battery_adc_next_action_ms = 0U;
    s_battery_adc_retry_count = 0U;
    s_battery_adc_last_burst_ms = 0U;
    s_battery_adc_last_total_ms = 0U;
    battery_adc_reset_attempt();
#if BATTERY_ADC_DLPS_TRANSITION_LOG_ENABLE
    if (s_battery_adc_log_enabled)
    {
        DBG_DIRECT("[BAT_ADC] prepare_for_dlps");
    }
#endif
}

void battery_adc_resume_after_dlps(uint32_t runtime_ms)
{
    if (!s_battery_adc_inited)
    {
        (void)battery_adc_init();
    }

    s_battery_adc_busy = false;
    s_battery_adc_guard_session_active = false;
    s_battery_adc_state = BATTERY_ADC_STAGE_IDLE;
    s_battery_adc_schedule_valid = true;
    s_battery_adc_due_pending = true;
    s_battery_adc_next_due_ms = runtime_ms;
    s_battery_adc_next_action_ms = 0U;
    s_battery_adc_retry_count = 0U;
    battery_adc_reset_attempt();
    battery_adc_record_status(BATTERY_ADC_STATUS_DEFERRED);
#if BATTERY_ADC_DLPS_TRANSITION_LOG_ENABLE
    if (s_battery_adc_log_enabled)
    {
        DBG_DIRECT("[BAT_ADC] resume_after_dlps due=%lu",
                   (unsigned long)runtime_ms);
    }
#endif
}
