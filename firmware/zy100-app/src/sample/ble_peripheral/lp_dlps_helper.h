/**
*****************************************************************************************
*     Copyright(c) 2026, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
* @file      lp_dlps_helper.h
* @brief     Packaged low-power DLPS helper APIs for ble_peripheral sample.
*****************************************************************************************
*/

#ifndef _LP_DLPS_HELPER_H_
#define _LP_DLPS_HELPER_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void lp_dlps_power_manager_init(void);
/* Called by the existing product callbacks; never register a second owner. */
void lp_dlps_note_enter(void);
void lp_dlps_note_exit(void);
#define LP_STANDBY_PIN_COUNT 16U
typedef struct
{
    uint16_t pad[LP_STANDBY_PIN_COUNT];
    uint8_t mux[LP_STANDBY_PIN_COUNT];
    uint32_t direction, input, output, clock0, clock1;
} lp_standby_io_snapshot_t;
typedef struct
{
    uint32_t enters, exits, invalid, sleep_ms;
} lp_dlps_stats_t;
void lp_dlps_get_stats(lp_dlps_stats_t *out);
void lp_standby_io_snapshot(lp_standby_io_snapshot_t *out);
/* Sensor-only parking never changes KEY, USB, LED or the business state. */
bool lp_sensor_idle_io_matches(bool gpio_wom);
void lp_sensor_idle_io_apply(void);
bool lp_standby_io_matches(bool charge);
bool lp_standby_spi_suspend(void);
void lp_standby_io_apply(bool charge);
void lp_standby_io_log(const lp_standby_io_snapshot_t *before,
                       const lp_standby_io_snapshot_t *after);
void lp_dlps_get_residency(uint32_t *enters, uint32_t *exits, uint32_t *sleep_ms);
/* One-shot packaged DLPS flow:
 * - configure low-power GPIO policy
 * - disable unrelated peripheral clocks
 * - enable DLPS mode and callbacks
 * - run short diagnostics, then auto-close UART logs
 * - start scheduler and enter DLPS cycling
 */
bool lp_dlps_enter_packaged_mode(void);
/* Apply low-power IO/clock policy before button-driven DLPS sleep.
 * This helper only performs low-power parking and peripheral clock gating.
 * It does NOT touch DLPS callback registration, lps_mode_set, or wake pin rearm.
 */
void lp_dlps_apply_button_sleep_low_power_policy(void);
void lp_dlps_set_imu_wom_wake_armed(bool armed);
void lp_dlps_set_sensor_power_hold(bool hold);
void lp_dlps_set_sleep_charge_led_hold(bool hold);
bool lp_dlps_sleep_charge_led_hold_enabled(void);
bool lp_dlps_apply_sleep_charge_led_hold_now(void);
/* Task step-test helper:
 * - optional task status dump
 * - optional APP task suspend for DLPS current measurement
 * Return true if suspend call succeeded; false otherwise.
 */
bool lp_dlps_apply_task_step_test(void *app_task_handle);

void lp_dlps_close_runtime_log_uart(void);

#ifdef __cplusplus
}
#endif

#endif /* _LP_DLPS_HELPER_H_ */

