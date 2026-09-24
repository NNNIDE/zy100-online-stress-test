#ifndef APP_USER_RESET_H
#define APP_USER_RESET_H
#include <stdbool.h>
#include <stdint.h>
#include "app_button.h"
typedef enum {
    USER_RESET_OFF, USER_RESET_DRAIN, USER_RESET_CONFIRM,
    USER_RESET_MARK, USER_RESET_PREPARE, USER_RESET_CAL,
    USER_RESET_BONDS, USER_RESET_STORAGE, USER_RESET_DEFAULTS,
    USER_RESET_COMMIT, USER_RESET_FLASH, USER_RESET_CHASE,
    USER_RESET_PAIR, USER_RESET_ERROR
} app_user_reset_state_t;
void app_user_reset_boot(void);
void app_user_reset_request(void);
void app_user_reset_cancel_pending_for_fault(void);
void app_user_reset_poll(uint32_t now);
bool app_user_reset_active(void);
bool app_user_reset_executing(void);
bool app_user_reset_confirming(void);
bool app_user_reset_needs_drain(void);
bool app_user_reset_first_pair_required(void);
/* Persist onboarding only; does not request reset or delete user data. */
bool app_user_reset_require_first_pair(void);
bool app_user_reset_pairing_complete(void);
void app_user_reset_button(const app_button_event_t *event, uint32_t now);
void app_user_reset_port_shutdown(bool wake);
bool app_user_reset_port_quiet(void);
int app_user_reset_port_prepare(void);
int app_user_reset_port_storage(void);
bool app_user_reset_port_defaults(void);
bool app_user_reset_port_pair(void);
void app_user_reset_port_cancel(void);
void app_user_reset_port_input_lock(void);
void app_user_reset_port_led(app_user_reset_state_t state, uint32_t elapsed);
bool app_user_reset_port_pressed(void);
bool app_user_reset_port_power_safe(void);
#endif
