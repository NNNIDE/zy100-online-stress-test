#ifndef APP_BATTERY_POLICY_H
#define APP_BATTERY_POLICY_H

#include <stdbool.h>
#include <stdint.h>

#define APP_BATTERY_CRITICAL_PERCENT_UNKNOWN 0xFFU
#define APP_BATTERY_CRITICAL_MV_UNKNOWN      0xFFFFU

typedef struct
{
    uint16_t battery_mv;
    uint8_t percent;
} app_battery_critical_event_t;

typedef enum
{
    APP_BATTERY_ADC_MODE_ACTIVE = 0U,
    APP_BATTERY_ADC_MODE_STANDBY_FULL,
    APP_BATTERY_ADC_MODE_STANDBY_CHARGE,
    APP_BATTERY_ADC_MODE_CHARGE_PARTIAL_SLEEP,
    APP_BATTERY_ADC_MODE_FULL_SLEEP,
    APP_BATTERY_ADC_MODE_COUNT,
} app_battery_adc_mode_t;

void app_battery_policy_init(void);
bool app_battery_policy_adc_set_mode(app_battery_adc_mode_t mode,
                                     uint32_t runtime_ms);
bool app_battery_policy_adc_work_due(app_battery_adc_mode_t mode,
                                     uint32_t runtime_ms);
bool app_battery_policy_adc_runtime_poll(app_battery_adc_mode_t mode,
                                         uint32_t runtime_ms,
                                         bool sample_allowed,
                                         bool resume_before_start);
bool app_battery_policy_adc_sample_once(uint32_t runtime_ms,
                                         uint8_t *percent_out);
bool app_battery_policy_adc_commit_bas_current(uint32_t runtime_ms);
bool app_battery_policy_adc_commit_current(uint32_t runtime_ms);
void app_battery_policy_adc_force_due(uint32_t runtime_ms);
void app_battery_policy_adc_cancel(uint32_t runtime_ms);
bool app_battery_policy_adc_attempt_pending(void);
bool app_battery_policy_prepare_capture(const char *reason);
void app_battery_policy_arm_capture(const char *reason);
void app_battery_policy_disarm_capture(const char *reason);
void app_battery_policy_disarm_adc_guard(const char *reason);
void app_battery_policy_poll(uint32_t runtime_ms);
void app_battery_policy_handle_level(uint8_t percent, uint32_t runtime_ms);
void app_battery_policy_critical_led_tick(uint64_t runtime_ms);
void app_battery_policy_critical_led_stop(void);
void app_battery_policy_mark_critical(const char *reason);
bool app_battery_policy_is_critical_locked(void);

/* Compatibility callbacks used by the existing battery services. */
void app_task_battery_low_lpc_event_handle(uint32_t runtime_ms);
void app_task_battery_adc_guard_low_event_handle(uint32_t runtime_ms,
                                                 uint16_t mv,
                                                 uint8_t percent);
void app_task_battery_adc_guard_fault_event_handle(uint32_t runtime_ms);

/* Ports implemented by app_task.c; they expose commands/queries, not state. */
bool app_battery_policy_port_runtime_shutdown_active(void);
void app_battery_policy_port_runtime_shutdown_mark(const char *reason);
bool app_battery_policy_port_capture_state_active(void);
bool app_battery_policy_port_capture_active(void);
bool app_battery_policy_port_capture_stop_in_progress(void);
void app_battery_policy_port_request_critical_stop(
    const app_battery_critical_event_t *event);
void app_battery_policy_port_request_adc_fault_stop(void);
bool app_battery_policy_port_sleep_transition_active(void);
bool app_battery_policy_port_export_pressure(void);
bool app_battery_policy_port_critical_led_blocked(void);
void app_battery_policy_port_prepare_critical_led(void);
void app_battery_policy_port_recovered(void);
bool app_battery_policy_port_battery_service_set(uint8_t percent);
bool app_battery_policy_port_battery_service_notify_ready(void);
bool app_battery_policy_port_battery_service_notify(uint8_t percent);

#endif
