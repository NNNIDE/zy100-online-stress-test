#ifndef SVC_APP_WATCHDOG_H
#define SVC_APP_WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void svc_app_watchdog_init(void);
void svc_app_watchdog_reconfigure_for_wake(const char *reason);
void svc_app_watchdog_enable_active(const char *reason);
void svc_app_watchdog_enable_standby(const char *reason);
void svc_app_watchdog_disable_for_shutdown(const char *reason);
void svc_app_watchdog_poll(uint32_t now_ms);
uint32_t svc_app_watchdog_cap_wait_ms(uint32_t wait_ms, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* SVC_APP_WATCHDOG_H */
