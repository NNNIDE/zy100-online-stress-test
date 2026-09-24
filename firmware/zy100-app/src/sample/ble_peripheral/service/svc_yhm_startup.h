#ifndef SVC_YHM_STARTUP_H
#define SVC_YHM_STARTUP_H
#include "../driver/yhm2712_acmd.h"

typedef enum {
    YHM_STARTUP_WAIT_RELEASE, YHM_STARTUP_FAST_RETRY,
    YHM_STARTUP_SLOW_RETRY, YHM_STARTUP_READY
} svc_yhm_startup_phase_t;
typedef yhm2712_acmd_status_t (*svc_yhm_startup_attempt_t)(void *, bool);
typedef struct {
    uint32_t started, released, due, last_log, attempts, minimum_ms;
    bool tracking, external, begun, saw_pressed;
    uint8_t last_key;
    svc_yhm_startup_phase_t phase;
    yhm2712_acmd_status_t last_status;
    svc_yhm_startup_attempt_t attempt;
    void *context;
} svc_yhm_startup_t;

/* Owning task only. Poll at least every 10 ms while pending; no delay/task/ISR. */
void svc_yhm_startup_begin(svc_yhm_startup_t *state, uint32_t now,
    uint32_t minimum_ms, svc_yhm_startup_attempt_t attempt, void *context);
bool svc_yhm_startup_poll(svc_yhm_startup_t *state, uint32_t now);
bool svc_yhm_startup_ready(const svc_yhm_startup_t *state);
/* Diagnostics use only a read-only probe, never the product register profile. */
yhm2712_acmd_status_t svc_yhm_startup_read_only(void *context, bool external);
#endif
