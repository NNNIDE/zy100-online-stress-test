#ifndef SVC_YHM2712_CHARGE_H
#define SVC_YHM2712_CHARGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "../common/charge_voltage_sample.h"

#include "../app_build_config.h"
#include "../driver/yhm2712_acmd.h"

typedef struct
{
    uint8_t status1;
    uint8_t status2;
    uint8_t status3;
    uint8_t chg_status;
    uint8_t fsm;
    bool isns_lt_iterm;
    bool vsys_gt_vbat;
    bool in_cv;
    bool fsm_valid;
    bool valid;
} svc_yhm2712_charge_status_t;

typedef enum
{
    SVC_YHM2712_TRANSITION_ENSURE_CHARGE = 0U,
    SVC_YHM2712_TRANSITION_RESUME_ACTIVE,
} svc_yhm2712_transition_event_t;

typedef enum
{
    SVC_YHM2712_TRANSITION_OK = 0U,
    SVC_YHM2712_TRANSITION_ACMD_ERROR,
    SVC_YHM2712_TRANSITION_BAD_REQUEST,
} svc_yhm2712_transition_outcome_t;

typedef enum
{
    SVC_YHM2712_ACTION_NONE = 0U,
    SVC_YHM2712_ACTION_KEEP_CHARGE,
    SVC_YHM2712_ACTION_SLEEP_TO_CHARGE,
    SVC_YHM2712_ACTION_APPLY_CHARGE,
    SVC_YHM2712_ACTION_SLEEP_TO_DISCHARGE,
    SVC_YHM2712_ACTION_APPLY_DISCHARGE,
} svc_yhm2712_transition_action_t;

typedef struct
{
    svc_yhm2712_transition_event_t event;
    bool external_power_present;
    /* Set only when app_power verified MODE=SLEEP before this DLPS epoch. */
    bool sleep_exit_required;
    const char *reason;
} svc_yhm2712_transition_request_t;

typedef struct
{
    svc_yhm2712_transition_outcome_t outcome;
    yhm2712_acmd_status_t acmd_status;
    svc_yhm2712_transition_action_t action;
    svc_yhm2712_charge_status_t status;
    bool transport_settled;
    yhm2712_acmd_recovery_snapshot_t recovery;
} svc_yhm2712_transition_result_t;

/* Bind interface without selftest, register access or MODE changes. */
bool svc_yhm2712_charge_prepare_interface(uint8_t pin);
yhm2712_acmd_status_t svc_yhm2712_charge_init_status(uint8_t pin, bool external);
bool svc_yhm2712_charge_init(uint8_t stacmd_pin,
                             bool external_power_present);
svc_yhm2712_transition_outcome_t svc_yhm2712_charge_apply_transition(
    const svc_yhm2712_transition_request_t *request,
    svc_yhm2712_transition_result_t *result);
yhm2712_acmd_status_t svc_yhm2712_charge_configure_profile(const char *reason);
yhm2712_acmd_status_t svc_yhm2712_charge_wake_for_charge_only(void);
yhm2712_acmd_status_t svc_yhm2712_charge_wake_for_active(bool chg_present,
                                                         const char *reason);
yhm2712_acmd_status_t svc_yhm2712_charge_prepare_full_sleep(void);
#if ZY100_BUILD_FACTORY
yhm2712_acmd_status_t svc_yhm2712_charge_factory_resume_from_shipping(
    bool external_power_present);
#endif
yhm2712_acmd_status_t svc_yhm2712_charge_factory_enter_shipping(void);
yhm2712_acmd_status_t svc_yhm2712_charge_poll(uint32_t now_ms,
                                              uint16_t vbat_mv,
                                              bool chg_present);
#if ZY100_BUILD_PRODUCTION
/* Call only after existing stable USB-removal confirmation. No hardware I/O. */
void svc_yhm2712_charge_session_removed(void);
yhm2712_acmd_status_t svc_yhm2712_charge_poll_sample(
    uint32_t now_ms, const charge_voltage_sample_t *sample, bool chg_present);
#endif
bool svc_yhm2712_charge_full_detected(void);
void svc_yhm2712_charge_reset_full_detected(void);
bool svc_yhm2712_charge_status_done_latest(void);
bool svc_yhm2712_charge_status_latest(svc_yhm2712_charge_status_t *status);
bool svc_yhm2712_charge_boot_diag_latest(yhm2712_acmd_boot_diag_t *diag);

#ifdef __cplusplus
}
#endif

#endif /* SVC_YHM2712_CHARGE_H */
