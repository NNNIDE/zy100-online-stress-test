#ifndef APP_OTA_CONTROLLER_PORT_H
#define APP_OTA_CONTROLLER_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "app_ota_controller.h"
#include "app_ble_power_policy.h"
#include "service/svc_app_watchdog.h"
#include "dfu_flash.h"

app_ota_reject_reason_t app_ota_controller_port_quiet_reason(
    uint32_t *power_block_mask_out);
bool app_ota_controller_port_standby_active(void);
bool app_ota_controller_port_restore_standby_online_high(const char *reason);
bool app_ota_controller_port_active(void);
const char *app_ota_controller_port_power_state_name(void);
bool app_ota_controller_port_arm_time_checkpoint(void);

void app_ota_controller_port_cancel_wake_boot(const char *reason);
void app_ota_controller_port_cancel_charge_led(const char *reason);
void app_ota_controller_port_cancel_online_complete(const char *reason);
void app_task_auto_idle_note_activity(const char *reason);

#define app_ota_controller_port_note_activity(reason) \
    app_task_auto_idle_note_activity(reason)
#define app_ota_controller_port_button_reset_hold() ((void)0)
#define app_ota_controller_port_prepare_ble_quiesce() \
    app_ble_prepare_for_ota_quiesce()
#define app_ota_controller_port_recover_ble_after_cancel() \
    app_ble_recover_after_ota_cancel()
#define app_ota_controller_port_disable_watchdog(reason) \
    svc_app_watchdog_disable_for_shutdown(reason)
#define app_ota_controller_port_switch_to_dfu() \
    dfu_switch_to_ota_mode()

#endif /* APP_OTA_CONTROLLER_PORT_H */
