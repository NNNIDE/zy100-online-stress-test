#include "svc_app_watchdog.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "rtl876x_aon_wdg.h"
#include "trace.h"

#include "../app_flags.h"
#include "zy100_online_reset_trace.h"

#if ZY100_APP_AON_WDG_ENABLE

#if (ZY100_APP_AON_WDG_TIMEOUT_SECONDS == 0U)
#error "ZY100_APP_AON_WDG_TIMEOUT_SECONDS must be non-zero"
#endif

#if (ZY100_APP_AON_WDG_TIMEOUT_SECONDS > 65U)
#error "ZY100_APP_AON_WDG_TIMEOUT_SECONDS exceeds aon_wdg_init max timeout"
#endif

#if (ZY100_APP_AON_WDG_FEED_PERIOD_MS == 0U)
#error "ZY100_APP_AON_WDG_FEED_PERIOD_MS must be non-zero"
#endif

#if (ZY100_APP_AON_WDG_FEED_PERIOD_MS >= (ZY100_APP_AON_WDG_TIMEOUT_SECONDS * 1000U))
#error "ZY100_APP_AON_WDG_FEED_PERIOD_MS must be shorter than watchdog timeout"
#endif

typedef enum
{
    SVC_APP_WATCHDOG_STATE_DISABLED = 0U,
    SVC_APP_WATCHDOG_STATE_ACTIVE,
    SVC_APP_WATCHDOG_STATE_STANDBY,
} svc_app_watchdog_state_t;

#define SVC_APP_WATCHDOG_RESET_LEVEL     1U

static bool s_svc_app_watchdog_configured = false;
static svc_app_watchdog_state_t s_svc_app_watchdog_state =
    SVC_APP_WATCHDOG_STATE_DISABLED;
static uint32_t s_svc_app_watchdog_last_feed_ms = 0U;

static const char *svc_app_watchdog_state_name(svc_app_watchdog_state_t state)
{
    switch (state)
    {
    case SVC_APP_WATCHDOG_STATE_ACTIVE:
        return "active";
    case SVC_APP_WATCHDOG_STATE_STANDBY:
        return "standby";
    case SVC_APP_WATCHDOG_STATE_DISABLED:
    default:
        return "disabled";
    }
}

static void svc_app_watchdog_configure_hw(void)
{
    aon_wdg_init((uint8_t)SVC_APP_WATCHDOG_RESET_LEVEL,
                 (uint8_t)ZY100_APP_AON_WDG_TIMEOUT_SECONDS);
    s_svc_app_watchdog_configured = true;
}

static void svc_app_watchdog_enable_state(svc_app_watchdog_state_t state,
                                          const char *reason)
{
    svc_app_watchdog_state_t prev_state = s_svc_app_watchdog_state;

    if (!s_svc_app_watchdog_configured)
    {
        svc_app_watchdog_configure_hw();
    }

    AON_WDG_Restart();
    aon_wdg_enable();
    s_svc_app_watchdog_state = state;
    s_svc_app_watchdog_last_feed_ms = 0U;

    if (prev_state != state)
    {
        DBG_DIRECT("[APP_WDG] enable state=%s timeout_s=%lu feed_ms=%lu reason=%s",
                   svc_app_watchdog_state_name(state),
                   (unsigned long)ZY100_APP_AON_WDG_TIMEOUT_SECONDS,
                   (unsigned long)ZY100_APP_AON_WDG_FEED_PERIOD_MS,
                   (reason != NULL) ? reason : "unknown");
    }
}

void svc_app_watchdog_init(void)
{
    aon_wdg_disable();
    s_svc_app_watchdog_configured = false;
    s_svc_app_watchdog_state = SVC_APP_WATCHDOG_STATE_DISABLED;
    s_svc_app_watchdog_last_feed_ms = 0U;
}

void svc_app_watchdog_reconfigure_for_wake(const char *reason)
{
    aon_wdg_disable();
    svc_app_watchdog_configure_hw();
    AON_WDG_Restart();
    aon_wdg_enable();
    s_svc_app_watchdog_state = SVC_APP_WATCHDOG_STATE_ACTIVE;
    s_svc_app_watchdog_last_feed_ms = 0U;
    DBG_DIRECT("[APP_WDG] wake_reconfigure timeout_s=%lu feed_ms=%lu reason=%s",
               (unsigned long)ZY100_APP_AON_WDG_TIMEOUT_SECONDS,
               (unsigned long)ZY100_APP_AON_WDG_FEED_PERIOD_MS,
               (reason != NULL) ? reason : "unknown");
}

void svc_app_watchdog_enable_active(const char *reason)
{
    svc_app_watchdog_enable_state(SVC_APP_WATCHDOG_STATE_ACTIVE, reason);
}

void svc_app_watchdog_enable_standby(const char *reason)
{
    svc_app_watchdog_enable_state(SVC_APP_WATCHDOG_STATE_STANDBY, reason);
}

void svc_app_watchdog_disable_for_shutdown(const char *reason)
{
    if ((s_svc_app_watchdog_state != SVC_APP_WATCHDOG_STATE_DISABLED) ||
        s_svc_app_watchdog_configured)
    {
        aon_wdg_disable();
        DBG_DIRECT("[APP_WDG] disable reason=%s prev=%s",
                   (reason != NULL) ? reason : "shutdown",
                   svc_app_watchdog_state_name(s_svc_app_watchdog_state));
    }

    s_svc_app_watchdog_state = SVC_APP_WATCHDOG_STATE_DISABLED;
    s_svc_app_watchdog_configured = false;
    s_svc_app_watchdog_last_feed_ms = 0U;
}

void svc_app_watchdog_poll(uint32_t now_ms)
{
    if (s_svc_app_watchdog_state == SVC_APP_WATCHDOG_STATE_DISABLED)
    {
        return;
    }

    if ((s_svc_app_watchdog_last_feed_ms == 0U) ||
        ((uint32_t)(now_ms - s_svc_app_watchdog_last_feed_ms) >=
         (uint32_t)ZY100_APP_AON_WDG_FEED_PERIOD_MS))
    {
        AON_WDG_Restart();
        zy100_online_reset_trace_note_feed();
        s_svc_app_watchdog_last_feed_ms = now_ms;
    }
}

uint32_t svc_app_watchdog_cap_wait_ms(uint32_t wait_ms, uint32_t now_ms)
{
    uint32_t elapsed_ms;
    uint32_t remain_ms;

    if (s_svc_app_watchdog_state == SVC_APP_WATCHDOG_STATE_DISABLED)
    {
        return wait_ms;
    }

    if (s_svc_app_watchdog_last_feed_ms == 0U)
    {
        return (wait_ms > (uint32_t)ZY100_APP_AON_WDG_FEED_PERIOD_MS) ?
               (uint32_t)ZY100_APP_AON_WDG_FEED_PERIOD_MS : wait_ms;
    }

    elapsed_ms = now_ms - s_svc_app_watchdog_last_feed_ms;
    if (elapsed_ms >= (uint32_t)ZY100_APP_AON_WDG_FEED_PERIOD_MS)
    {
        return 0U;
    }

    remain_ms = (uint32_t)ZY100_APP_AON_WDG_FEED_PERIOD_MS - elapsed_ms;
    return (wait_ms > remain_ms) ? remain_ms : wait_ms;
}

#else

void svc_app_watchdog_init(void)
{
}

void svc_app_watchdog_reconfigure_for_wake(const char *reason)
{
    (void)reason;
}

void svc_app_watchdog_enable_active(const char *reason)
{
    (void)reason;
}

void svc_app_watchdog_enable_standby(const char *reason)
{
    (void)reason;
}

void svc_app_watchdog_disable_for_shutdown(const char *reason)
{
    (void)reason;
}

void svc_app_watchdog_poll(uint32_t now_ms)
{
    (void)now_ms;
}

uint32_t svc_app_watchdog_cap_wait_ms(uint32_t wait_ms, uint32_t now_ms)
{
    (void)now_ms;
    return wait_ms;
}

#endif
