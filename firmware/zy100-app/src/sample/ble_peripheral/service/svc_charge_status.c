#include "svc_charge_status.h"

#include <stddef.h>

#include "../bsp/bsp_power_status.h"

static bool s_svc_charge_inited = false;
static bool s_svc_charge_changed = false;
static uint32_t s_svc_charge_last_poll_ms = 0U;
static svc_charge_status_state_t s_svc_charge_state =
    SVC_CHARGE_STATUS_UNKNOWN;

static void svc_charge_status_set_state(svc_charge_status_state_t state)
{
    if (s_svc_charge_state != state)
    {
        s_svc_charge_state = state;
        s_svc_charge_changed = true;
    }
}

bool svc_charge_status_sample_now(void)
{
    bool present = false;
    bsp_power_status_t status;

    if (!s_svc_charge_inited)
    {
        status = bsp_power_status_init();
        if (status != BSP_POWER_STATUS_OK)
        {
            svc_charge_status_set_state(SVC_CHARGE_STATUS_UNKNOWN);
            return false;
        }
        s_svc_charge_inited = true;
    }

    status = bsp_power_status_chg_int_external_power_present(&present);
    if (status != BSP_POWER_STATUS_OK)
    {
        svc_charge_status_set_state(SVC_CHARGE_STATUS_UNKNOWN);
        return false;
    }

    svc_charge_status_set_state(present ?
                                SVC_CHARGE_STATUS_INPUT_PRESENT :
                                SVC_CHARGE_STATUS_NO_INPUT);
    return true;
}

bool svc_charge_status_init(void)
{
    bool ok;

    ok = svc_charge_status_sample_now();
    s_svc_charge_changed = false;
    s_svc_charge_last_poll_ms = 0U;
    return ok;
}

void svc_charge_status_update_external_power_present(bool present)
{
    s_svc_charge_inited = true;
    svc_charge_status_set_state(present ?
                                SVC_CHARGE_STATUS_INPUT_PRESENT :
                                SVC_CHARGE_STATUS_NO_INPUT);
}

bool svc_charge_status_poll(uint32_t runtime_ms)
{
    if ((s_svc_charge_last_poll_ms != 0U) &&
        ((uint32_t)(runtime_ms - s_svc_charge_last_poll_ms) <
         (uint32_t)SVC_CHARGE_STATUS_POLL_MS))
    {
        return s_svc_charge_changed;
    }

    s_svc_charge_last_poll_ms = runtime_ms;
    (void)svc_charge_status_sample_now();
    return s_svc_charge_changed;
}

bool svc_charge_status_external_power_present(void)
{
    if (s_svc_charge_state == SVC_CHARGE_STATUS_UNKNOWN)
    {
        (void)svc_charge_status_sample_now();
    }

    return s_svc_charge_state == SVC_CHARGE_STATUS_INPUT_PRESENT;
}

svc_charge_status_state_t svc_charge_status_state(void)
{
    return s_svc_charge_state;
}

bool svc_charge_status_take_changed(svc_charge_status_state_t *state)
{
    bool changed = s_svc_charge_changed;

    if (state != NULL)
    {
        *state = s_svc_charge_state;
    }

    s_svc_charge_changed = false;
    return changed;
}

const char *svc_charge_status_state_name(svc_charge_status_state_t state)
{
    switch (state)
    {
    case SVC_CHARGE_STATUS_NO_INPUT:
        return "no_input";
    case SVC_CHARGE_STATUS_INPUT_PRESENT:
        return "input_present";
    case SVC_CHARGE_STATUS_UNKNOWN:
    default:
        return "unknown";
    }
}
