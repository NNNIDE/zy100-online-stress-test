#ifndef SVC_CHARGE_STATUS_H
#define SVC_CHARGE_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    SVC_CHARGE_STATUS_UNKNOWN = 0U,
    SVC_CHARGE_STATUS_NO_INPUT,
    SVC_CHARGE_STATUS_INPUT_PRESENT,
} svc_charge_status_state_t;

#ifndef SVC_CHARGE_STATUS_POLL_MS
/* UI policy poll cadence, not a YHM2712 hardware timing value. */
#define SVC_CHARGE_STATUS_POLL_MS 200U
#endif

bool svc_charge_status_init(void);
bool svc_charge_status_sample_now(void);
void svc_charge_status_update_external_power_present(bool present);
bool svc_charge_status_poll(uint32_t runtime_ms);
bool svc_charge_status_external_power_present(void);
svc_charge_status_state_t svc_charge_status_state(void);
bool svc_charge_status_take_changed(svc_charge_status_state_t *state);
const char *svc_charge_status_state_name(svc_charge_status_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* SVC_CHARGE_STATUS_H */
