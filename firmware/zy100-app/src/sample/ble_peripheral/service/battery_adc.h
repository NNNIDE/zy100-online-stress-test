#ifndef BATTERY_ADC_H
#define BATTERY_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "../common/charge_voltage_sample.h"

#define BATTERY_ADC_FILTER_ACTION_NONE       0U
#define BATTERY_ADC_FILTER_ACTION_INIT       1U
#define BATTERY_ADC_FILTER_ACTION_HOLD       2U
#define BATTERY_ADC_FILTER_ACTION_STEP       3U
#define BATTERY_ADC_FILTER_ACTION_FAST       4U
#define BATTERY_ADC_FILTER_ACTION_QUARANTINE 5U
#define BATTERY_ADC_FILTER_ACTION_REJECT     6U

#define BATTERY_ADC_FILTER_DIR_NONE          0U
#define BATTERY_ADC_FILTER_DIR_UP            1U
#define BATTERY_ADC_FILTER_DIR_DOWN          2U

typedef struct
{
    uint32_t ok_count;
    uint32_t fail_count;
    const char *last_status;
    uint16_t last_raw;
    uint16_t raw_for_filter;
    uint16_t filter_before_raw;
    uint16_t filtered_raw;
    uint16_t filter_step_raw;
    bool filter_valid;
    bool trusted_valid;
    bool last_quarantine;
    bool usb_charge_recovery_hold;
    uint16_t last_min;
    uint16_t last_max;
    uint16_t last_spread;
    uint16_t spread_limit;
    uint16_t raw_mid;
    uint16_t trusted_raw;
    uint16_t trusted_mv;
    uint8_t last_retry;
    uint8_t last_percent;
    uint8_t trusted_percent;
    uint8_t filter_action;
    uint8_t filter_direction;
} battery_adc_diag_t;

typedef void (*battery_adc_sample_window_cb_t)(bool active, void *ctx);

typedef struct
{
    bool valid;
    uint16_t raw;
    uint16_t battery_mv;
    uint8_t percent;
} battery_adc_guard_one_shot_t;

/* Compact SDK-only result for Factory/whole-unit diagnostics.  The burst
 * remains internal to battery_adc.c; callers receive only aggregate values.
 * `sdk_mv` is produced directly by the SDK conversion and never includes the
 * persisted CAL1/gain used by the normal product battery policy. */
typedef struct
{
    bool valid;
    uint16_t raw_avg;
    uint16_t raw_min;
    uint16_t raw_max;
    uint16_t raw_spread;
    uint16_t sdk_mv;
    uint8_t sample_count;
} battery_adc_test_summary_t;

bool battery_adc_init(void);
bool battery_adc_sample_once(void);
bool battery_adc_guard_convert_raw(uint16_t raw,
                                   battery_adc_guard_one_shot_t *out);
bool battery_adc_guard_sample_one_shot(battery_adc_guard_one_shot_t *out);
bool battery_adc_guard_session_begin(void);
bool battery_adc_guard_session_read(battery_adc_guard_one_shot_t *out);
/* Test-only conversion path: raw -> SDK VBAT conversion, never CAL1/gain. */
bool battery_adc_guard_session_read_sdk(battery_adc_guard_one_shot_t *out);
void battery_adc_guard_session_end(void);
/* Test-only SDK conversion path.  It never reloads, reads or applies CAL1. */
bool battery_adc_sample_test_sdk_summary(battery_adc_test_summary_t *out);
uint16_t battery_adc_get_raw(void);
uint16_t battery_adc_get_voltage_mv(void);
uint16_t battery_adc_get_charge_gate_voltage_mv(void);
bool battery_adc_get_charge_sample(charge_voltage_sample_t *sample);
uint8_t battery_adc_get_percent(void);
uint16_t battery_adc_percent_to_voltage_mv(uint8_t percent);
void battery_adc_set_charge_report_state(bool external_power_present,
                                         bool charge_full_detected);
void battery_adc_set_charge_env_state(bool external_power_present,
                                      bool charge_full_detected,
                                      bool charge_path_active);
void battery_adc_peek_diag(battery_adc_diag_t *diag);
void battery_adc_take_diag(battery_adc_diag_t *diag);
bool battery_adc_runtime_tick(uint32_t runtime_ms, bool sample_allowed);
bool battery_adc_runtime_10s_tick(uint32_t runtime_ms);
bool battery_adc_is_active(void);
void battery_adc_runtime_force_due(uint32_t runtime_ms);
uint32_t battery_adc_runtime_wait_ms(uint32_t runtime_ms);
void battery_adc_runtime_cancel_pending_attempt(uint32_t runtime_ms);
void battery_adc_set_sample_window_callback(battery_adc_sample_window_cb_t cb,
                                            void *ctx);
void battery_adc_set_log_enabled(bool enabled);
void battery_adc_prepare_for_dlps(void);
void battery_adc_resume_after_dlps(uint32_t runtime_ms);
bool battery_adc_calibration_reload(void);
bool battery_adc_calibration_active(uint32_t *gain_q20_out);
void battery_adc_calibration_clear_runtime(void);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_ADC_H */
