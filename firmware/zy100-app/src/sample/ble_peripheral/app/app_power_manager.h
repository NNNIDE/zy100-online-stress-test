#ifndef APP_POWER_MANAGER_H
#define APP_POWER_MANAGER_H

#include <stdbool.h>

typedef enum
{
    APP_POWER_STATE_ACTIVE = 0,
    APP_POWER_STATE_STANDBY_FULL,
    APP_POWER_STATE_STANDBY_CHARGE,
    APP_POWER_STATE_FULL_SLEEP_DLPS,
    APP_POWER_STATE_CHARGE_PARTIAL_SLEEP
} app_power_state_t;

typedef enum
{
    APP_POWER_REQUEST_NONE = 0,
    APP_POWER_REQUEST_STANDBY,
    APP_POWER_REQUEST_RESTORE,
    APP_POWER_REQUEST_SHUTDOWN,
    APP_POWER_REQUEST_WAKE,
    APP_POWER_REQUEST_AUTHORIZED_WAKE,
    APP_POWER_REQUEST_RECOVERY,
    APP_POWER_REQUEST_PAIRING
} app_power_request_t;

typedef enum
{
    APP_POWER_ACCEPTED = 0,
    APP_POWER_NO_ACTION,
    APP_POWER_BUSY,
    APP_POWER_FORBIDDEN
} app_power_result_t;

/* Reports name existing application checkpoints, not desired hardware modes.
 * ACTIVE does not prove clocks/resources/business are ready. */
typedef enum
{
    APP_POWER_REPORT_NONE = 0,
    APP_POWER_FACTORY_ACTIVE,
    APP_POWER_RECOVERY_ACTIVE,
    APP_POWER_SLEEP_ABORTED,
    APP_POWER_FULL_SLEEP_COMMITTED,
    APP_POWER_CHARGE_SLEEP_COMMITTED,
    APP_POWER_WAKE_STARTED,
    APP_POWER_STANDBY_FULL_PREPARED,
    APP_POWER_STANDBY_CHARGE_PREPARED,
    APP_POWER_IDLE_USB_ACTIVE,
    APP_POWER_STANDBY_CANCEL_STARTED,
    APP_POWER_STANDBY_RESOURCES_RESTORED,
    APP_POWER_CHARGE_REMOVED_SLEEP,
    APP_POWER_DIAGNOSTIC_ACTIVE,
    APP_POWER_DIAGNOSTIC_SLEEP,
    APP_POWER_STORAGE_FAULT_ACTIVE,
    APP_POWER_PAIRING_INITIALIZED
} app_power_report_t;

typedef struct
{
    app_power_state_t mode;
    app_power_request_t last_request;
    app_power_result_t last_result;
    app_power_report_t last_report;
} app_power_snapshot_t;

app_power_state_t app_power_manager_mode(void);
const app_power_snapshot_t *app_power_manager_view(void);
bool app_power_manager_is_standby(app_power_state_t mode);
bool app_power_manager_blocks_business(void);
app_power_result_t app_power_manager_request(app_power_request_t request);
void app_power_manager_report(app_power_report_t report);

#endif
