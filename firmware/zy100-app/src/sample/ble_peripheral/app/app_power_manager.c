#include "app_power_manager.h"
#include "app_shutdown_transaction.h"

static app_power_snapshot_t s_power;

app_power_state_t app_power_manager_mode(void) { return s_power.mode; }
const app_power_snapshot_t *app_power_manager_view(void) { return &s_power; }

bool app_power_manager_is_standby(app_power_state_t mode)
{
    return mode == APP_POWER_STATE_STANDBY_FULL ||
           mode == APP_POWER_STATE_STANDBY_CHARGE;
}

bool app_power_manager_blocks_business(void)
{
    /* Shutdown is the owner. Never cache this permission in the power mode. */
    return app_shutdown_transaction_blocks_business();
}

app_power_result_t app_power_manager_request(app_power_request_t request)
{
    app_power_result_t result = APP_POWER_ACCEPTED;
    /* The restore no-op precedes the shutdown gate in the existing flow. */
    if (request <= APP_POWER_REQUEST_NONE || request > APP_POWER_REQUEST_PAIRING)
        result = APP_POWER_FORBIDDEN;
    else if (request == APP_POWER_REQUEST_RESTORE &&
        !app_power_manager_is_standby(s_power.mode))
        result = APP_POWER_NO_ACTION;
    else if ((request == APP_POWER_REQUEST_RESTORE ||
              request == APP_POWER_REQUEST_WAKE) &&
             app_power_manager_blocks_business())
        result = APP_POWER_FORBIDDEN;
    else if (request == APP_POWER_REQUEST_RECOVERY &&
             app_power_manager_blocks_business())
        result = APP_POWER_BUSY;
    /* Standby/pairing admission also needs lazy resource checks at the original
     * action boundaries. Acceptance here does not release any business gate. */
    s_power.last_request = request;
    s_power.last_result = result;
    return result;
}

void app_power_manager_report(app_power_report_t report)
{
    switch (report)
    {
    case APP_POWER_STANDBY_FULL_PREPARED:
        s_power.mode = APP_POWER_STATE_STANDBY_FULL; break;
    case APP_POWER_STANDBY_CHARGE_PREPARED:
        s_power.mode = APP_POWER_STATE_STANDBY_CHARGE; break;
    case APP_POWER_FULL_SLEEP_COMMITTED:
    case APP_POWER_CHARGE_REMOVED_SLEEP:
    case APP_POWER_DIAGNOSTIC_SLEEP:
        s_power.mode = APP_POWER_STATE_FULL_SLEEP_DLPS; break;
    case APP_POWER_CHARGE_SLEEP_COMMITTED:
        s_power.mode = APP_POWER_STATE_CHARGE_PARTIAL_SLEEP; break;
    case APP_POWER_FACTORY_ACTIVE:
    case APP_POWER_RECOVERY_ACTIVE:
    case APP_POWER_SLEEP_ABORTED:
    case APP_POWER_WAKE_STARTED:
    case APP_POWER_IDLE_USB_ACTIVE:
    case APP_POWER_STANDBY_CANCEL_STARTED:
    case APP_POWER_STANDBY_RESOURCES_RESTORED:
    case APP_POWER_DIAGNOSTIC_ACTIVE:
    case APP_POWER_STORAGE_FAULT_ACTIVE:
    case APP_POWER_PAIRING_INITIALIZED:
        s_power.mode = APP_POWER_STATE_ACTIVE; break;
    default:
        return;
    }
    s_power.last_report = report;
}
