#include "app_standby_transaction.h"
#include "app_standby_port.h"

static app_standby_snapshot_t s_standby;
static app_standby_action_t s_enter;
static bool s_enter_pending, s_enter_reset_timer;
bool app_standby_transaction_pending(void) { return s_enter_pending; }

const app_standby_snapshot_t *app_standby_transaction_view(void) { return &s_standby; }
void app_standby_transaction_housekeep(uint32_t now_ms) { s_standby.housekeep_ms = now_ms; }
void app_standby_transaction_adc_normalize(bool pending) { s_standby.adc_normalize_pending = pending; }
void app_standby_transaction_clear(void)
{
    s_enter_pending = false;
    s_standby.enter_ms = 0U;
    s_standby.housekeep_ms = 0U;
    s_standby.adc_normalize_pending = false;
}

bool app_standby_transaction_housekeep_due(uint32_t now_ms, uint32_t period_ms)
{
    return s_standby.housekeep_ms == 0U ||
           (uint32_t)(now_ms - s_standby.housekeep_ms) >= period_ms;
}
uint32_t app_standby_transaction_housekeep_wait(uint32_t now_ms, uint32_t period_ms)
{
    uint32_t elapsed = (uint32_t)(now_ms - s_standby.housekeep_ms);
    return s_standby.housekeep_ms == 0U || elapsed >= period_ms ? 0U : period_ms - elapsed;
}
bool app_standby_transaction_expired(uint64_t now_ms, uint32_t timeout_ms)
{
    return s_standby.enter_ms != 0U && now_ms - s_standby.enter_ms >= timeout_ms;
}
bool app_standby_cause_skips_boot(app_power_cause_t cause)
{
    switch (cause)
    {
    case APP_POWER_CAUSE_USB:
    case APP_POWER_CAUSE_CAPTURE:
    case APP_POWER_CAUSE_MOTION:
    case APP_POWER_CAUSE_BLE_CONNECTED:
    case APP_POWER_CAUSE_BLE_DISCONNECTED:
    case APP_POWER_CAUSE_BUTTON:
    case APP_POWER_CAUSE_OTA:
    case APP_POWER_CAUSE_CALIBRATION:
    case APP_POWER_CAUSE_PAIRING:
    case APP_POWER_CAUSE_USER_RESET:
        return true;
    default:
        return false;
    }
}

static bool finish(app_standby_step_result_t result)
{
    s_standby.result = result == APP_STANDBY_STEP_COMPLETE ? APP_POWER_ACCEPTED : APP_POWER_BUSY;
    if (result == APP_STANDBY_STEP_COMPLETE) s_standby.stage = APP_STANDBY_COMPLETE;
    return result == APP_STANDBY_STEP_COMPLETE;
}

bool app_standby_transaction_enter(app_power_state_t target, const char *reason,
                                  bool reset_timer, app_power_cause_t cause)
{
    app_standby_action_t action = {APP_POWER_STATE_ACTIVE};
    app_standby_step_result_t result;
    if (s_enter_pending) return false;
    action.prev_state = app_power_manager_mode();
    action.target_state = target;
    action.charge_mode = target == APP_POWER_STATE_STANDBY_CHARGE;
    action.reason = reason;
    action.now_ms = app_standby_port_now();
    (void)app_power_manager_request(APP_POWER_REQUEST_STANDBY);
    s_standby.stage = APP_STANDBY_ADMISSION;
    s_standby.cause = cause;
    if (app_standby_port_pairing_active() || app_standby_port_reset_active() ||
        !app_power_manager_is_standby(target))
    {
        s_standby.result = APP_POWER_FORBIDDEN;
        return false;
    }
    result = app_standby_port_enter_prepare(&action);
    if (result != APP_STANDBY_STEP_CONTINUE) return finish(result);
    s_standby.stage = APP_STANDBY_DRAIN;
    result = app_standby_port_enter_drain(&action);
    if (result != APP_STANDBY_STEP_CONTINUE) return finish(result);
    s_standby.stage = APP_STANDBY_POWER;
    result = app_standby_port_enter_charge(&action);
    if (result != APP_STANDBY_STEP_CONTINUE) return finish(result);
    result = app_standby_port_enter_full(&action);
    if (result != APP_STANDBY_STEP_CONTINUE) return finish(result);
    s_standby.stage = APP_STANDBY_IO;
    s_enter = action;
    /* Callers may supply temporary diagnostic text; never retain that pointer. */
    s_enter.reason = "standby_io";
    s_enter_reset_timer = reset_timer;
    s_enter_pending = true;
    return app_standby_transaction_poll();
}

bool app_standby_transaction_poll(void)
{
    app_standby_step_result_t result;
    if (!s_enter_pending) return false;
    result = app_standby_port_enter_io(&s_enter);
    if (result == APP_STANDBY_STEP_WAIT) return false;
    s_enter_pending = false;
    if (result != APP_STANDBY_STEP_CONTINUE) return finish(result);
    app_power_manager_report(s_enter.charge_mode ? APP_POWER_STANDBY_CHARGE_PREPARED :
                                                 APP_POWER_STANDBY_FULL_PREPARED);
    (void)app_standby_port_enter_commit(&s_enter);
    if (s_enter_reset_timer || s_standby.enter_ms == 0U) s_standby.enter_ms = s_enter.now_ms;
    s_standby.housekeep_ms = s_enter.charge_mode ? 0U : (uint32_t)s_enter.now_ms;
    s_standby.stage = APP_STANDBY_AUDIT;
    return finish(app_standby_port_enter_finish(&s_enter));
}

bool app_standby_transaction_auto(const char *reason)
{
    bool present = false;
    if (app_standby_port_factory_active())
    {
        app_standby_port_auto_factory(reason);
        return false;
    }
    if (!app_standby_port_sample_usb("standby_auto", &present)) return false;
    if (present)
    {
        app_standby_port_auto_usb(reason);
        return false;
    }
    return app_standby_transaction_enter(APP_POWER_STATE_STANDBY_FULL, reason, true, APP_POWER_CAUSE_IDLE);
}

bool app_standby_transaction_usb(bool present, const char *reason)
{
    uint64_t now_ms = app_standby_port_now();
    if (!app_power_manager_is_standby(app_power_manager_mode())) return false;
    if (present)
    {
        app_standby_port_usb_restore_note(reason);
        /* Button edge capture is never selected by a USB intent. */
        return app_standby_transaction_restore(reason, APP_BLE_CI_STATE_ACTIVE_IDLE, APP_POWER_CAUSE_USB);
    }
    if (app_power_manager_mode() == APP_POWER_STATE_STANDBY_FULL)
    {
        s_standby.enter_ms = now_ms;
        s_standby.housekeep_ms = (uint32_t)now_ms;
        app_standby_port_usb_refresh(reason);
        return true;
    }
    app_standby_port_usb_switch_note(reason);
    return app_standby_transaction_enter(APP_POWER_STATE_STANDBY_FULL, reason, true, APP_POWER_CAUSE_USB);
}

bool app_standby_transaction_restore(const char *reason, app_ble_ci_state_t ble_target, app_power_cause_t cause)
{
    app_standby_action_t action = {APP_POWER_STATE_ACTIVE};
    app_standby_step_result_t result;
    bool entering = s_enter_pending;
    s_enter_pending = false;
    action.use_reason = reason != 0 ? reason : "standby_wake";
    action.reason = reason;
    /* During IO wait the charger already entered the target mode although
     * PREPARED has not been published. Restore that physical transition. */
    action.prev_state = entering ? s_enter.target_state : app_power_manager_mode();
    action.preserve_button_gesture = cause == APP_POWER_CAUSE_BUTTON;
    action.now_ms = (uint32_t)app_standby_port_now();
    action.ble_target = ble_target;
    action.cause = cause;
    s_standby.cause = cause;
    s_standby.stage = APP_STANDBY_ADMISSION;
    if (!app_power_manager_is_standby(action.prev_state))
    {
        s_standby.result = APP_POWER_NO_ACTION;
        return true;
    }
    if (app_standby_port_shutdown_guard_enabled() &&
        ((entering && app_power_manager_blocks_business()) ||
         app_power_manager_request(APP_POWER_REQUEST_RESTORE) == APP_POWER_FORBIDDEN))
    {
        s_standby.result = APP_POWER_FORBIDDEN;
        return false;
    }
    s_standby.stage = APP_STANDBY_RESTORE_CLOCK;
    result = app_standby_port_restore_clock(&action);
    if (result != APP_STANDBY_STEP_CONTINUE) return finish(result);
    (void)app_standby_port_sample_usb("standby_wake_chg", &action.chg_present);
    action.standby_usb_restore = action.chg_present && cause == APP_POWER_CAUSE_USB;
    s_standby.stage = APP_STANDBY_RESTORE_POWER;
    result = app_standby_port_restore_power(&action);
    if (result != APP_STANDBY_STEP_CONTINUE) return finish(result);
    s_standby.stage = APP_STANDBY_RESTORE_FLASH;
    result = app_standby_port_restore_flash(&action);
    if (result != APP_STANDBY_STEP_CONTINUE) return finish(result);
    app_power_manager_report(APP_POWER_STANDBY_RESOURCES_RESTORED);
    s_standby.stage = APP_STANDBY_RESTORE_RUNTIME;
    result = app_standby_port_restore_runtime(&action);
    if (result != APP_STANDBY_STEP_CONTINUE) return finish(result);
    return finish(app_standby_port_restore_finish(&action));
}

bool app_standby_transaction_cancel(const char *reason, bool present, app_power_cause_t cause)
{
    app_standby_action_t action = {APP_POWER_STATE_ACTIVE};
    app_standby_step_result_t result;
    s_enter_pending = false;
    action.reason = reason;
    action.use_reason = reason != 0 ? reason : "standby_cancel";
    action.chg_present = present;
    action.standby_usb_restore = present && cause == APP_POWER_CAUSE_USB;
    action.cause = cause;
    action.now_ms = (uint32_t)app_standby_port_now();
    s_standby.cause = cause;
    s_standby.stage = APP_STANDBY_ADMISSION;
    if (app_standby_port_shutdown_guard_enabled() && app_power_manager_blocks_business())
    {
        s_standby.result = APP_POWER_FORBIDDEN;
        return false;
    }
    s_standby.stage = APP_STANDBY_RESTORE_CLOCK;
    result = app_standby_port_cancel_clock(&action);
    if (result != APP_STANDBY_STEP_CONTINUE) return finish(result);
    s_standby.stage = APP_STANDBY_RESTORE_FLASH;
    result = app_standby_port_cancel_flash(&action);
    if (result != APP_STANDBY_STEP_CONTINUE) return finish(result);
    s_standby.stage = APP_STANDBY_RESTORE_RUNTIME;
    result = app_standby_port_cancel_runtime(&action);
    if (result != APP_STANDBY_STEP_CONTINUE) return finish(result);
    return finish(app_standby_port_cancel_finish(&action));
}

