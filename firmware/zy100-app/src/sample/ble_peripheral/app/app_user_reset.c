#include "app_user_reset.h"
#include <string.h>
#include "trace.h"
#include "service/zy100_system_info_store.h"
#include "service/zy100_calibration_store.h"
#include "service/zy100_calibration_manager.h"
#include "gap_bond_le.h"
#include "app_device_pairing.h"

static app_user_reset_state_t s_reset;
static uint32_t s_since;
static uint32_t s_now;
static uint8_t s_persisted;
static bool s_resume;
static app_user_reset_state_t s_failed_step;
static uint8_t s_retry_count;

static void reset_state(app_user_reset_state_t state)
{
    if (state == USER_RESET_ERROR)
    {
        s_failed_step = s_reset;
        DBG_DIRECT("[USER_RESET] failed step=%u retry=%u", (unsigned)s_reset, s_retry_count);
    }
    s_reset = state;
    s_since = s_now;
    DBG_DIRECT("[USER_RESET] state=%u", (unsigned)state);
}
bool app_user_reset_active(void) { return s_reset != USER_RESET_OFF; }
bool app_user_reset_executing(void) { return s_reset >= USER_RESET_MARK; }
bool app_user_reset_confirming(void) { return s_reset == USER_RESET_CONFIRM; }
bool app_user_reset_needs_drain(void) { return s_reset == USER_RESET_DRAIN || s_reset == USER_RESET_CONFIRM; }
bool app_user_reset_first_pair_required(void) { return s_persisted != 0U; }

static bool reset_save(uint8_t state, bool defaults)
{
    zy100_system_info_t info;
    if (!zy100_system_info_load(&info)) return false;
    if (defaults)
    {
        info.latest_user_id = 0U;
        memset(&info.ota_time, 0, sizeof(info.ota_time));
        info.ota_success_led_pending = 0U;
        /* Hardware/manufacturing initialization has already completed. */
        info.first_power_seen = 1U;
    }
    info.user_reset_state = state;
    if (!zy100_system_info_save(&info)) return false;
    if (!zy100_system_info_load(&info) || info.user_reset_state != state ||
        (defaults && (info.latest_user_id != 0U || info.paired_peer_count != 0U))) return false;
    s_persisted = state;
    return true;
}
bool app_user_reset_require_first_pair(void)
{
    zy100_system_info_t info;
    if (app_user_reset_active() || !zy100_system_info_load(&info)) return false;
    /* Do not overwrite an unfinished reset, even if the RAM cache is stale. */
    if (info.user_reset_state != 0U && info.user_reset_state != 2U) return false;
    s_persisted = info.user_reset_state;
    if (s_persisted == 2U) return true;
    return reset_save(2U, false);
}
void app_user_reset_boot(void)
{
    zy100_system_info_t info;
    if (!zy100_system_info_load(&info)) return;
    s_persisted = info.user_reset_state;
    if (s_persisted == 1U)
    {
        s_resume = true;
        reset_state(USER_RESET_DRAIN);
        app_user_reset_port_input_lock();
        app_user_reset_port_shutdown(true);
    }
}
void app_user_reset_request(void)
{
    if (app_user_reset_active()) return;
    s_resume = false;
    s_retry_count = 0U;
    reset_state(USER_RESET_DRAIN);
    app_device_pairing_window_cancel();
    app_user_reset_port_shutdown(false);
    app_user_reset_port_input_lock();
}
static void reset_cancel(void)
{
    reset_state(USER_RESET_OFF);
    app_user_reset_port_cancel();
}
void app_user_reset_button(const app_button_event_t *event, uint32_t now)
{
    s_now = now;
    if (!app_user_reset_confirming()) return;
    if ((uint32_t)(now - s_since) >= 15000U)
    {
        reset_cancel();
        return;
    }
    if (event->action == APP_BUTTON_ACTION_SHUTDOWN_PREPARE ||
        event->hold_ms >= 1500U) { reset_cancel(); return; }
    if (event->released && event->action == APP_BUTTON_ACTION_DOUBLE_CLICK)
    {
        reset_state(USER_RESET_MARK);
        app_user_reset_port_input_lock();
    }
}
bool app_user_reset_pairing_complete(void)
{
    if (s_persisted == 0U) return true;
    if (s_persisted != 2U || app_user_reset_active()) return false;
    return reset_save(0U, false);
}
void app_user_reset_poll(uint32_t now)
{
    int result;
    s_now = now;
    /* Ordinary wake onboarding is opened by the completed boot sequence. */
    if (!app_user_reset_active()) return;
    app_user_reset_port_led(s_reset, now - s_since);
    if (s_reset >= USER_RESET_CAL && s_reset <= USER_RESET_COMMIT &&
        !app_user_reset_port_power_safe()) return;
    switch (s_reset)
    {
    case USER_RESET_DRAIN:
        if (!app_user_reset_port_quiet()) return;
        if (s_resume) { reset_state(USER_RESET_PREPARE); break; }
        if (!app_user_reset_port_pressed())
        {
            app_user_reset_port_input_lock();
            reset_state(USER_RESET_CONFIRM);
        }
        break;
    case USER_RESET_CONFIRM:
        if (now - s_since >= 15000U) reset_cancel();
        break;
    case USER_RESET_MARK:
        if (!app_user_reset_port_power_safe()) { reset_state(USER_RESET_ERROR); break; }
        if (!reset_save(1U, false)) { reset_state(USER_RESET_ERROR); break; }
        reset_state(USER_RESET_PREPARE);
        break;
    case USER_RESET_PREPARE:
        if (!app_user_reset_port_power_safe()) return;
        result = app_user_reset_port_prepare();
        if (result < 0 || now - s_since >= 30000U) reset_state(USER_RESET_ERROR);
        else if (result > 0) reset_state(USER_RESET_CAL);
        break;
    case USER_RESET_CAL:
        if (zy100_cal_store_clear() != ZY100_CAL_STORE_OK) { reset_state(USER_RESET_ERROR); break; }
        zy100_cal_manager_init();
        reset_state(USER_RESET_BONDS);
        break;
    case USER_RESET_BONDS:
        le_bond_clear_all_keys();
        if (le_get_bond_dev_num() != 0U || !zy100_system_info_clear_pairing())
        { reset_state(USER_RESET_ERROR); break; }
        app_device_pairing_shutdown();
        reset_state(USER_RESET_STORAGE);
        break;
    case USER_RESET_STORAGE:
        result = app_user_reset_port_storage();
        if (result < 0) reset_state(USER_RESET_ERROR);
        else if (result > 0) reset_state(USER_RESET_DEFAULTS);
        break;
    case USER_RESET_DEFAULTS:
        reset_state(app_user_reset_port_defaults() ? USER_RESET_COMMIT : USER_RESET_ERROR);
        break;
    case USER_RESET_COMMIT:
        reset_state(reset_save(2U, true) ? USER_RESET_FLASH : USER_RESET_ERROR);
        break;
    case USER_RESET_FLASH:
        if (now - s_since >= 1200U) reset_state(USER_RESET_CHASE);
        break;
    case USER_RESET_CHASE:
        if (now - s_since >= 1200U) reset_state(USER_RESET_PAIR);
        break;
    case USER_RESET_PAIR:
        if (!app_user_reset_port_pair()) return;
        reset_state(USER_RESET_OFF);
        break;
    case USER_RESET_ERROR:
        /* Busy work is polled in place. Retry only idempotent synchronous steps;
         * an errored storage context needs the persisted boot recovery path. */
        if (now - s_since >= 1000U && s_retry_count < 3U &&
            app_user_reset_port_power_safe() && s_failed_step != USER_RESET_STORAGE &&
            s_failed_step != USER_RESET_PREPARE)
        {
            ++s_retry_count;
            reset_state(s_failed_step);
        }
        break;
    default: break;
    }
}

void app_user_reset_cancel_pending_for_fault(void)
{
    /* Never cancel an accepted/persisted reset or its boot recovery. */
    if (!s_resume && app_user_reset_needs_drain()) reset_state(USER_RESET_OFF);
}
