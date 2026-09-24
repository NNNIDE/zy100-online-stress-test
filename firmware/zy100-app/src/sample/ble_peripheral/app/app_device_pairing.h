#ifndef APP_DEVICE_PAIRING_H
#define APP_DEVICE_PAIRING_H

#include <stdbool.h>
#include <stdint.h>

#define APP_PAIRING_EVENT_REASON_BYTES 32U
#define APP_PAIRING_PEER_ADDR_BYTES    6U
#define APP_PAIRING_EVENT_QUEUE_DEPTH  8U
#define APP_PAIRING_CONN_ID_INVALID    0xFFU

typedef enum
{
    APP_TASK_PAIRING_EVENT_AUTH_STARTED = 0,
    APP_TASK_PAIRING_EVENT_CONFIRM_PENDING,
    APP_TASK_PAIRING_EVENT_AUTH_SUCCESS,
    APP_TASK_PAIRING_EVENT_AUTH_FAILED,
    APP_TASK_PAIRING_EVENT_BOND_ADD,
    APP_TASK_PAIRING_EVENT_BOND_DELETE,
    APP_TASK_PAIRING_EVENT_BOND_CLEAR,
    APP_TASK_PAIRING_EVENT_BOND_KEY_MISSING,
    APP_TASK_PAIRING_EVENT_BOND_FULL,
    APP_TASK_PAIRING_EVENT_DISCONNECTED,
    APP_TASK_PAIRING_EVENT_PAIRING_LOST,
} app_task_pairing_event_t;

typedef struct
{
    app_task_pairing_event_t event;
    uint8_t conn_id;
    uint8_t peer_addr[APP_PAIRING_PEER_ADDR_BYTES];
    uint8_t peer_type;
    uint8_t peer_addr_valid;
    uint32_t window_generation;
    uint32_t connection_generation;
    char reason[APP_PAIRING_EVENT_REASON_BYTES];
} app_task_pairing_pending_t;

void app_task_pairing_post_event(app_task_pairing_event_t event,
                                 uint8_t conn_id,
                                 const uint8_t peer_addr[APP_PAIRING_PEER_ADDR_BYTES],
                                 uint8_t peer_type,
                                 const char *reason);
bool app_task_pairing_take_event(app_task_pairing_pending_t *out);
bool app_task_pairing_queue_has_pending(void);
void app_device_pairing_handle_pending_event(void);
/* Discard volatile pairing work; persisted bonds are untouched. */
void app_device_pairing_shutdown(void);

/* Window state is owned by the app task; GAP callbacks only enqueue facts. */
void app_device_pairing_window_request(void);
/* App task only, after boot readiness and durable onboarding admission. */
bool app_device_pairing_window_start_on_boot(void);
bool app_device_pairing_port_boot_window_ready(void);
void app_device_pairing_window_cancel(void);
bool app_device_pairing_window_active(void);
bool app_device_pairing_window_waiting(void);
void app_device_pairing_window_poll(uint64_t now_ms);
void app_device_pairing_window_adv_result(bool active, bool failed);
bool app_device_pairing_admit_new_bond(uint8_t conn_id);
void app_device_pairing_note_connected(void);
bool app_device_pairing_port_shutdown_ready(void);
bool app_device_pairing_port_window_start(void);
void app_device_pairing_port_window_led(bool active);
void app_device_pairing_port_window_success_led(const char *reason);
void app_device_pairing_port_window_expire(bool start_failed);

void app_device_pairing_port_note_activity(const char *reason);
bool app_device_pairing_port_post_event(void);
bool app_device_pairing_port_gate_load(const char *reason, bool force_reload);
bool app_device_pairing_port_bonded(void);
void app_device_pairing_port_set_bonded(bool bonded);
bool app_device_pairing_port_first_power_seen(void);
const char *app_device_pairing_port_gate_state_name(void);
void app_device_pairing_port_abort_runtime_keep_flash(const char *reason);
void app_device_pairing_port_wait_begin(const char *reason);
void app_device_pairing_port_confirm_begin(const char *reason);
void app_device_pairing_port_success_hold_begin(const char *reason);
void app_device_pairing_port_set_ready(const char *reason);
void app_device_pairing_port_set_error(const char *reason);
void app_device_pairing_port_on_reconnect_ready(const char *reason);
void app_device_pairing_port_try_auto_export(const char *reason);
bool app_device_pairing_port_prepare_confirm(const char *reason);

#endif
