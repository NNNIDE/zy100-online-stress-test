#ifndef APP_STANDBY_PORT_H
#define APP_STANDBY_PORT_H
#include "app_standby_transaction.h"

/* Synchronous, stack-owned action arguments. Never retained by a port. */
typedef struct
{
    app_power_state_t prev_state;
    app_power_state_t target_state;
    const char *reason;
    const char *use_reason;
    uint64_t now_ms;
    app_ble_ci_state_t ble_target;
    app_power_cause_t cause;
    bool charge_mode;
    bool chg_present;
    bool standby_usb_restore;
    bool preserve_button_gesture;
} app_standby_action_t;

typedef enum
{
    APP_STANDBY_STEP_CONTINUE = 0,
    APP_STANDBY_STEP_COMPLETE,
    APP_STANDBY_STEP_FAILED,
    APP_STANDBY_STEP_WAIT
} app_standby_step_result_t;

uint64_t app_standby_port_now(void);
bool app_standby_port_pairing_active(void);
bool app_standby_port_reset_active(void);
bool app_standby_port_shutdown_guard_enabled(void);
bool app_standby_port_factory_active(void);
bool app_standby_port_sample_usb(const char *reason, bool *present);
void app_standby_port_auto_factory(const char *reason);
void app_standby_port_auto_usb(const char *reason);
void app_standby_port_usb_restore_note(const char *reason);
void app_standby_port_usb_refresh(const char *reason);
void app_standby_port_usb_switch_note(const char *reason);

app_standby_step_result_t app_standby_port_enter_prepare(app_standby_action_t *action);
app_standby_step_result_t app_standby_port_enter_drain(app_standby_action_t *action);
app_standby_step_result_t app_standby_port_enter_charge(app_standby_action_t *action);
app_standby_step_result_t app_standby_port_enter_full(app_standby_action_t *action);
app_standby_step_result_t app_standby_port_enter_io(app_standby_action_t *action);
app_standby_step_result_t app_standby_port_enter_commit(app_standby_action_t *action);
app_standby_step_result_t app_standby_port_enter_finish(app_standby_action_t *action);
app_standby_step_result_t app_standby_port_restore_clock(app_standby_action_t *action);
app_standby_step_result_t app_standby_port_restore_power(app_standby_action_t *action);
app_standby_step_result_t app_standby_port_restore_flash(app_standby_action_t *action);
app_standby_step_result_t app_standby_port_restore_runtime(app_standby_action_t *action);
app_standby_step_result_t app_standby_port_restore_finish(app_standby_action_t *action);
app_standby_step_result_t app_standby_port_cancel_clock(app_standby_action_t *action);
app_standby_step_result_t app_standby_port_cancel_flash(app_standby_action_t *action);
app_standby_step_result_t app_standby_port_cancel_runtime(app_standby_action_t *action);
app_standby_step_result_t app_standby_port_cancel_finish(app_standby_action_t *action);
#endif
