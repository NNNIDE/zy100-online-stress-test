#ifndef APP_STANDBY_TRANSACTION_H
#define APP_STANDBY_TRANSACTION_H
#include <stdint.h>
#include "app_power_manager.h"
#include "app_ble_ci_state.h"

/* Semantic intent is independent of the diagnostic reason string. */
typedef enum
{
    APP_POWER_CAUSE_OTHER = 0,
    APP_POWER_CAUSE_USB,
    APP_POWER_CAUSE_CAPTURE,
    APP_POWER_CAUSE_MOTION,
    APP_POWER_CAUSE_BLE_CONNECTED,
    APP_POWER_CAUSE_BLE_DISCONNECTED,
    APP_POWER_CAUSE_BUTTON,
    APP_POWER_CAUSE_OTA,
    APP_POWER_CAUSE_CALIBRATION,
    APP_POWER_CAUSE_PAIRING,
    APP_POWER_CAUSE_USER_RESET,
    APP_POWER_CAUSE_IDLE
} app_power_cause_t;

typedef enum
{
    APP_STANDBY_IDLE = 0,
    APP_STANDBY_ADMISSION,
    APP_STANDBY_DRAIN,
    APP_STANDBY_POWER,
    APP_STANDBY_IO,
    APP_STANDBY_AUDIT,
    APP_STANDBY_RESTORE_CLOCK,
    APP_STANDBY_RESTORE_POWER,
    APP_STANDBY_RESTORE_FLASH,
    APP_STANDBY_RESTORE_RUNTIME,
    APP_STANDBY_COMPLETE
} app_standby_stage_t;

typedef struct
{
    uint64_t enter_ms;
    uint32_t housekeep_ms;
    bool adc_normalize_pending;
    app_standby_stage_t stage;
    app_power_cause_t cause;
    app_power_result_t result;
} app_standby_snapshot_t;

const app_standby_snapshot_t *app_standby_transaction_view(void);
/* Resume only IO admission; completed drain/charger operations are not replayed. */
bool app_standby_transaction_poll(void);
bool app_standby_transaction_pending(void);
bool app_standby_transaction_enter(app_power_state_t target, const char *reason,
                                   bool reset_timer, app_power_cause_t cause);
bool app_standby_transaction_auto(const char *reason);
bool app_standby_transaction_usb(bool present, const char *reason);
bool app_standby_transaction_restore(const char *reason, app_ble_ci_state_t ble_target,
                                     app_power_cause_t cause);
bool app_standby_transaction_cancel(const char *reason, bool present,
                                    app_power_cause_t cause);
void app_standby_transaction_clear(void);
void app_standby_transaction_housekeep(uint32_t now_ms);
void app_standby_transaction_adc_normalize(bool pending);
bool app_standby_transaction_housekeep_due(uint32_t now_ms, uint32_t period_ms);
uint32_t app_standby_transaction_housekeep_wait(uint32_t now_ms, uint32_t period_ms);
bool app_standby_transaction_expired(uint64_t now_ms, uint32_t timeout_ms);
bool app_standby_cause_skips_boot(app_power_cause_t cause);
#endif
