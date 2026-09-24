#include "app/app_ble_ci_state.h"
#include "app/app_ble_conn_param_mgr.h"
#include "app/app_ble_link_trace.h"

#include <stddef.h>
#include <string.h>

#include <gap.h>
#include <gap_conn_le.h>
#include <os_sched.h>
#include <trace.h>

#include "app_flags.h"

#if ZY100_BUILD_PRODUCTION
#undef DBG_DIRECT
#define DBG_DIRECT(...) ZY100_BLE_DIAG_LOG(__VA_ARGS__)
#endif
#include "peripheral_app.h"
#include "service/zy100_ble_ctrl_protocol.h"

typedef struct
{
    uint16_t ci;
    uint16_t latency;
    uint16_t timeout;
} app_ble_ci_target_t;

typedef enum
{
    APP_BLE_CI_OTA_HOST_NONE = 0,
    APP_BLE_CI_OTA_HOST_WAIT_HIGH,
    APP_BLE_CI_OTA_HOST_HIGH_LEASED,
} app_ble_ci_ota_host_phase_t;

#define APP_BLE_CI_HOST_PROFILE_MASK_REQUIRED 0x00000007UL
#define APP_BLE_CI_WINDOWS_MIN_BUILD          22000UL
#define APP_BLE_CI_ANDROID_MIN_API_LEVEL      21UL
#define APP_BLE_CI_HOST_APPLY_TIMEOUT_MS      10000UL
#define APP_BLE_CI_HOST_LEASE_MS              15000UL
#define APP_BLE_CI_HOST_ACTIVE_MIN            24U
#define APP_BLE_CI_HOST_ACTIVE_MAX            48U
#define APP_BLE_CI_HOST_HIGH                  12U
#define APP_BLE_CI_HOST_STANDBY_MIN           72U
#define APP_BLE_CI_HOST_STANDBY_MAX           144U
#define APP_BLE_CI_ANDROID_FAST_MIN           9U
#define APP_BLE_CI_ANDROID_FAST_MAX           12U
#define APP_BLE_CI_ANDROID_MID_MIN            24U
#define APP_BLE_CI_ANDROID_MID_MAX            48U
#define APP_BLE_CI_ANDROID_SLOW_MIN           72U
#define APP_BLE_CI_ANDROID_SLOW_MAX           160U
#define APP_BLE_CI_ANDROID_SLOW_LATENCY_MAX   5U

static app_ble_ci_state_snapshot_t s_ci_state;
static uint32_t s_generation_counter;
static uint32_t s_last_link_notify_revision;
static uint32_t s_link_phase_transition_id;
static uint32_t s_host_enable_reject_detail;
static app_ble_ci_ota_host_phase_t s_ota_host_phase;
static char s_transition_reason[48] = "init";

static const app_ble_ci_target_t s_ci_targets[APP_BLE_CI_STATE_COUNT] = {
    {0U, 0U, 0U},
    {
        (uint16_t)ZY100_BLE_ACTIVE_IDLE_CONN_INTERVAL,
        (uint16_t)ZY100_BLE_ACTIVE_IDLE_CONN_LATENCY,
        (uint16_t)ZY100_BLE_ACTIVE_IDLE_CONN_TIMEOUT
    },
    {
        (uint16_t)ZY100_BLE_RUNTIME_CONN_INTERVAL_MIN,
        (uint16_t)ZY100_BLE_RUNTIME_CONN_LATENCY,
        (uint16_t)ZY100_BLE_RUNTIME_CONN_TIMEOUT
    },
    {
        (uint16_t)ZY100_BLE_EXPORT_FAST_CI_MIN,
        (uint16_t)ZY100_BLE_EXPORT_FAST_LATENCY,
        (uint16_t)ZY100_BLE_RUNTIME_CONN_TIMEOUT
    },
    {
        (uint16_t)ZY100_BLE_STANDBY_CONN_INTERVAL_MIN,
        (uint16_t)ZY100_BLE_STANDBY_CONN_LATENCY,
        (uint16_t)ZY100_BLE_STANDBY_CONN_TIMEOUT
    }
};

static const app_ble_ci_target_t s_android_capture_target = {
    (uint16_t)ZY100_BLE_EXPORT_FAST_CI_MIN,
    (uint16_t)ZY100_BLE_EXPORT_FAST_LATENCY,
    (uint16_t)ZY100_BLE_RUNTIME_CONN_TIMEOUT
};

static uint32_t app_ble_ci_state_now_ms(void)
{
    return (uint32_t)os_sys_time_get();
}

static const app_ble_ci_target_t *app_ble_ci_state_target(
    app_ble_ci_state_t state)
{
    if ((state <= APP_BLE_CI_STATE_DISCONNECTED) ||
        (state >= APP_BLE_CI_STATE_COUNT))
    {
        return NULL;
    }
    return &s_ci_targets[(uint32_t)state];
}

static const app_ble_ci_target_t *app_ble_ci_state_effective_target(
    app_ble_ci_state_t state)
{
    if ((s_ci_state.initiator_mode ==
         APP_BLE_CI_INITIATOR_ANDROID_CENTRAL) &&
        (state == APP_BLE_CI_STATE_CAPTURE_RUNTIME))
    {
        return &s_android_capture_target;
    }
    return app_ble_ci_state_target(state);
}

static void app_ble_ci_state_read_actual(uint8_t conn_id)
{
    uint16_t value;

    if (le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &value, conn_id) ==
        GAP_CAUSE_SUCCESS)
    {
        s_ci_state.actual_ci = value;
    }
    if (le_get_conn_param(GAP_PARAM_CONN_LATENCY, &value, conn_id) ==
        GAP_CAUSE_SUCCESS)
    {
        s_ci_state.actual_latency = value;
    }
    if (le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &value, conn_id) ==
        GAP_CAUSE_SUCCESS)
    {
        s_ci_state.actual_timeout = value;
    }
}

static bool app_ble_ci_state_values_match(const app_ble_ci_target_t *target,
                                          uint16_t ci,
                                          uint16_t latency)
{
    return (target != NULL) &&
           (ci == target->ci) &&
           (latency == target->latency);
}

static app_ble_ci_host_profile_t app_ble_ci_state_profile(
    app_ble_ci_state_t state)
{
    if (state == APP_BLE_CI_STATE_ACTIVE_IDLE)
    {
        return APP_BLE_CI_HOST_PROFILE_BALANCED;
    }
    if ((state == APP_BLE_CI_STATE_CAPTURE_RUNTIME) ||
        (state == APP_BLE_CI_STATE_ONLINE_HIGH))
    {
        return APP_BLE_CI_HOST_PROFILE_THROUGHPUT;
    }
    if (state == APP_BLE_CI_STATE_STANDBY)
    {
        return APP_BLE_CI_HOST_PROFILE_POWER;
    }
    return APP_BLE_CI_HOST_PROFILE_NONE;
}

static bool app_ble_ci_state_windows_values_match(app_ble_ci_state_t state,
                                                   uint16_t ci,
                                                   uint16_t latency)
{
    if (latency != 0U)
    {
        return false;
    }
    if (state == APP_BLE_CI_STATE_ACTIVE_IDLE)
    {
        return (ci >= APP_BLE_CI_HOST_ACTIVE_MIN) &&
               (ci <= APP_BLE_CI_HOST_ACTIVE_MAX);
    }
    if ((state == APP_BLE_CI_STATE_CAPTURE_RUNTIME) ||
        (state == APP_BLE_CI_STATE_ONLINE_HIGH))
    {
        return ci == APP_BLE_CI_HOST_HIGH;
    }
    if (state == APP_BLE_CI_STATE_STANDBY)
    {
        return (ci >= APP_BLE_CI_HOST_STANDBY_MIN) &&
               (ci <= APP_BLE_CI_HOST_STANDBY_MAX);
    }
    return false;
}

static bool app_ble_ci_state_android_values_match(app_ble_ci_state_t state,
                                                   uint16_t ci,
                                                   uint16_t latency)
{
    if ((state == APP_BLE_CI_STATE_CAPTURE_RUNTIME) ||
        (state == APP_BLE_CI_STATE_ONLINE_HIGH))
    {
        return ((ci >= APP_BLE_CI_ANDROID_FAST_MIN) &&
                (ci <= APP_BLE_CI_ANDROID_FAST_MAX) &&
                (latency == 0U));
    }
    if (state == APP_BLE_CI_STATE_ACTIVE_IDLE)
    {
        return ((ci >= APP_BLE_CI_ANDROID_MID_MIN) &&
                (ci <= APP_BLE_CI_ANDROID_MID_MAX) &&
                (latency == 0U));
    }
    if (state == APP_BLE_CI_STATE_STANDBY)
    {
        return ((ci >= APP_BLE_CI_ANDROID_SLOW_MIN) &&
                (ci <= APP_BLE_CI_ANDROID_SLOW_MAX) &&
                (latency <= APP_BLE_CI_ANDROID_SLOW_LATENCY_MAX));
    }
    return false;
}

static bool app_ble_ci_state_host_values_match(app_ble_ci_state_t state,
                                                uint16_t ci,
                                                uint16_t latency)
{
    if (s_ci_state.initiator_mode ==
        APP_BLE_CI_INITIATOR_ANDROID_CENTRAL)
    {
        return app_ble_ci_state_android_values_match(state, ci, latency);
    }
    return app_ble_ci_state_windows_values_match(state, ci, latency);
}

static bool app_ble_ci_state_ota_host_values_match(app_ble_ci_state_t state,
                                                    uint16_t ci,
                                                    uint16_t latency)
{
    return (state == APP_BLE_CI_STATE_ONLINE_HIGH) &&
           (ci >= APP_BLE_CI_ANDROID_FAST_MIN) &&
           (ci <= APP_BLE_CI_ANDROID_FAST_MAX) &&
           (latency == 0U);
}

static bool app_ble_ci_state_host_active(void)
{
    return s_ci_state.connected &&
           ((s_ci_state.initiator_mode ==
             APP_BLE_CI_INITIATOR_WINDOWS_CENTRAL) ||
            (s_ci_state.initiator_mode ==
             APP_BLE_CI_INITIATOR_ANDROID_CENTRAL));
}

static void app_ble_ci_state_link_changed(app_ble_ci_link_phase_t phase)
{
    if ((s_ci_state.link_phase == phase) &&
        (s_link_phase_transition_id == s_ci_state.transition_id))
    {
        return;
    }
    s_ci_state.link_phase = phase;
    s_link_phase_transition_id = s_ci_state.transition_id;
    s_ci_state.notify_revision++;
    if (s_ci_state.notify_revision == 0U)
    {
        s_ci_state.notify_revision = 1U;
    }
    app_ble_link_trace_note_ci_phase(&s_ci_state);
}

static bool app_ble_ci_state_target_is_settled(
    const app_ble_ci_target_t *target)
{
    if (s_ci_state.initiator_mode == APP_BLE_CI_INITIATOR_HOST_REQUIRED)
    {
        return false;
    }
    if (s_ota_host_phase != APP_BLE_CI_OTA_HOST_NONE)
    {
        return app_ble_ci_state_ota_host_values_match(
                   s_ci_state.desired_state,
                   s_ci_state.actual_ci,
                   s_ci_state.actual_latency) &&
               app_ble_conn_param_mgr_is_settled(s_ci_state.conn_id);
    }
    if (app_ble_ci_state_host_active())
    {
        return app_ble_ci_state_host_values_match(
                   s_ci_state.desired_state,
                   s_ci_state.actual_ci,
                   s_ci_state.actual_latency) &&
               (s_ci_state.host_request_status != 0U);
    }
    return app_ble_ci_state_values_match(target,
                                         s_ci_state.actual_ci,
                                         s_ci_state.actual_latency) &&
           app_ble_conn_param_mgr_is_settled(s_ci_state.conn_id);
}

static app_ble_ci_state_t app_ble_ci_state_classify(uint16_t ci,
                                                     uint16_t latency)
{
    app_ble_ci_state_t state;

    if (s_ci_state.initiator_mode ==
        APP_BLE_CI_INITIATOR_WINDOWS_CENTRAL)
    {
        if (latency != 0U)
        {
            return APP_BLE_CI_STATE_DISCONNECTED;
        }
        if (ci == APP_BLE_CI_HOST_HIGH)
        {
            return APP_BLE_CI_STATE_ONLINE_HIGH;
        }
        if ((ci >= APP_BLE_CI_HOST_ACTIVE_MIN) &&
            (ci <= APP_BLE_CI_HOST_ACTIVE_MAX))
        {
            return APP_BLE_CI_STATE_ACTIVE_IDLE;
        }
        if ((ci >= APP_BLE_CI_HOST_STANDBY_MIN) &&
            (ci <= APP_BLE_CI_HOST_STANDBY_MAX))
        {
            return APP_BLE_CI_STATE_STANDBY;
        }
        return APP_BLE_CI_STATE_DISCONNECTED;
    }

    if (s_ci_state.initiator_mode ==
        APP_BLE_CI_INITIATOR_ANDROID_CENTRAL)
    {
        if (app_ble_ci_state_android_values_match(
                APP_BLE_CI_STATE_ACTIVE_IDLE, ci, latency))
        {
            return APP_BLE_CI_STATE_ACTIVE_IDLE;
        }
        if (app_ble_ci_state_android_values_match(
                APP_BLE_CI_STATE_STANDBY, ci, latency))
        {
            return APP_BLE_CI_STATE_STANDBY;
        }
        if (app_ble_ci_state_android_values_match(
                APP_BLE_CI_STATE_ONLINE_HIGH, ci, latency))
        {
            if (s_ci_state.desired_state ==
                APP_BLE_CI_STATE_CAPTURE_RUNTIME)
            {
                return APP_BLE_CI_STATE_CAPTURE_RUNTIME;
            }
            return APP_BLE_CI_STATE_ONLINE_HIGH;
        }
        return APP_BLE_CI_STATE_DISCONNECTED;
    }

    for (state = APP_BLE_CI_STATE_ACTIVE_IDLE;
         state < APP_BLE_CI_STATE_COUNT;
         state = (app_ble_ci_state_t)((uint32_t)state + 1U))
    {
        if (app_ble_ci_state_values_match(
                app_ble_ci_state_target(state), ci, latency))
        {
            return state;
        }
    }
    return APP_BLE_CI_STATE_DISCONNECTED;
}

static uint32_t app_ble_ci_state_effective_event_ms(uint16_t ci,
                                                    uint16_t latency)
{
    uint32_t interval_ms;
    uint32_t event_ms;

    if (ci == 0U)
    {
        return 0U;
    }
    interval_ms = (((uint32_t)ci * 125UL) + 99UL) / 100UL;
    event_ms = interval_ms * ((uint32_t)latency + 1UL);
    return event_ms;
}

static uint32_t app_ble_ci_state_compute_wait_budget(uint16_t source_ci,
                                                     uint16_t source_latency)
{
    (void)source_ci;
    (void)source_latency;
    return (uint32_t)ZY100_BLE_CI_STAGE_FAIL_TIMEOUT_MS;
}

static uint32_t app_ble_ci_state_compute_stuck_budget(
    app_ble_ci_state_t source_state,
    app_ble_ci_state_t target_state)
{
    if ((source_state == APP_BLE_CI_STATE_STANDBY) &&
        (target_state == APP_BLE_CI_STATE_ACTIVE_IDLE))
    {
        return (uint32_t)ZY100_BLE_CI_STANDBY_EXIT_STUCK_TIMEOUT_MS;
    }
    return (uint32_t)ZY100_BLE_CI_LLCP_STUCK_TIMEOUT_MS;
}

const char *app_ble_ci_state_name(app_ble_ci_state_t state)
{
    switch (state)
    {
    case APP_BLE_CI_STATE_ACTIVE_IDLE:
        return "ACTIVE_IDLE";
    case APP_BLE_CI_STATE_CAPTURE_RUNTIME:
        return "CAPTURE_RUNTIME";
    case APP_BLE_CI_STATE_ONLINE_HIGH:
        return "ONLINE_HIGH";
    case APP_BLE_CI_STATE_STANDBY:
        return "STANDBY";
    default:
        return "DISCONNECTED_OR_OTHER";
    }
}

const char *app_ble_ci_transport_name(app_ble_ci_transport_state_t state)
{
    switch (state)
    {
    case APP_BLE_CI_TRANSPORT_IDLE:
        return "IDLE";
    case APP_BLE_CI_TRANSPORT_SUBMITTED:
        return "SUBMITTED";
    case APP_BLE_CI_TRANSPORT_PENDING:
        return "PENDING";
    case APP_BLE_CI_TRANSPORT_RETRYING:
        return "RETRYING";
    case APP_BLE_CI_TRANSPORT_TERMINAL_SUCCESS:
        return "TERMINAL_SUCCESS";
    case APP_BLE_CI_TRANSPORT_TERMINAL_FAIL:
        return "TERMINAL_FAIL";
    case APP_BLE_CI_TRANSPORT_STUCK:
        return "STUCK";
    case APP_BLE_CI_TRANSPORT_WAIT_HOST:
        return "WAIT_HOST";
    default:
        return "UNKNOWN";
    }
}

void app_ble_ci_state_init(void)
{
    memset(&s_ci_state, 0, sizeof(s_ci_state));
    s_ci_state.conn_id = 0xFFU;
    s_ci_state.desired_state = APP_BLE_CI_STATE_DISCONNECTED;
    s_ci_state.actual_state = APP_BLE_CI_STATE_DISCONNECTED;
    s_ci_state.transport_state = APP_BLE_CI_TRANSPORT_IDLE;
    s_ci_state.normalized_result = APP_BLE_CI_RESULT_IDLE;
    s_ci_state.initiator_mode =
        APP_BLE_CI_INITIATOR_HOST_REQUIRED;
    s_ci_state.expected_profile = APP_BLE_CI_HOST_PROFILE_NONE;
    s_ci_state.link_phase = APP_BLE_CI_LINK_IDLE;
    s_ci_state.generation = s_generation_counter;
    s_host_enable_reject_detail = 0U;
    s_ota_host_phase = APP_BLE_CI_OTA_HOST_NONE;
    s_last_link_notify_revision = 0U;
    s_link_phase_transition_id = 0U;
    strcpy(s_transition_reason, "init");
}

void app_ble_ci_state_on_connected(uint8_t conn_id,
                                   app_ble_ci_state_t initial_state,
                                   const char *reason)
{
    app_ble_ci_state_init();
    s_generation_counter++;
    if (s_generation_counter == 0U)
    {
        s_generation_counter = 1U;
    }
    s_ci_state.generation = s_generation_counter;
    s_ci_state.connected = true;
    s_ci_state.conn_id = conn_id;
    app_ble_ci_state_read_actual(conn_id);
    s_ci_state.actual_state = app_ble_ci_state_classify(
                                  s_ci_state.actual_ci,
                                  s_ci_state.actual_latency);
    s_ci_state.source_state = s_ci_state.actual_state;
    s_ci_state.desired_state = initial_state;
    s_ci_state.expected_profile = app_ble_ci_state_profile(initial_state);
    s_ci_state.transition_id = 1U;
    s_ci_state.transition_started_ms = app_ble_ci_state_now_ms();
    s_ci_state.transport_started_ms = s_ci_state.transition_started_ms;
    s_ci_state.target_reached = false;
    s_ci_state.request_pending = true;
    s_ci_state.transport_state = APP_BLE_CI_TRANSPORT_WAIT_HOST;
    s_ci_state.normalized_result = APP_BLE_CI_RESULT_WAIT_HOST;
    {
        const app_ble_ci_target_t *target =
            app_ble_ci_state_effective_target(initial_state);
        if (target != NULL)
        {
            s_ci_state.target_ci = target->ci;
            s_ci_state.target_latency = target->latency;
            s_ci_state.target_timeout = target->timeout;
        }
    }
    strncpy(s_transition_reason,
            (reason != NULL) ? reason : "connected_wait_host",
            sizeof(s_transition_reason) - 1U);
    s_transition_reason[sizeof(s_transition_reason) - 1U] = '\0';
#if ZY100_BLE_DIAG_LOG_ENABLE
    DBG_DIRECT("[BLE_CI_SM] connected gen=%lu c=%u state=%s ci=%u lat=%u tout=%u init=%s why=%s",
               (unsigned long)s_ci_state.generation,
               conn_id,
               app_ble_ci_state_name(s_ci_state.actual_state),
               s_ci_state.actual_ci,
               s_ci_state.actual_latency,
               s_ci_state.actual_timeout,
               app_ble_ci_state_name(initial_state),
               (reason != NULL) ? reason : "connected");
#endif
}

void app_ble_ci_state_on_disconnected(uint8_t conn_id, const char *reason)
{
#if ZY100_BLE_DIAG_LOG_ENABLE
    DBG_DIRECT("[BLE_CI_SM] disconnected gen=%lu c=%u want=%s trans=%lu ci=%u lat=%u why=%s",
               (unsigned long)s_ci_state.generation,
               conn_id,
               app_ble_ci_state_name(s_ci_state.desired_state),
               (unsigned long)s_ci_state.transition_id,
               s_ci_state.actual_ci,
               s_ci_state.actual_latency,
               (reason != NULL) ? reason : "disconnected");
#endif
    app_ble_conn_param_mgr_abort_all(conn_id, "ci_state_disconnect");
    app_ble_ci_state_init();
}

static bool app_ble_ci_state_enable_host_central(
    uint8_t conn_id,
    uint32_t session_id,
    uint32_t host_version,
    uint8_t protocol_version,
    app_ble_ci_initiator_mode_t initiator_mode)
{
    const app_ble_ci_target_t *target;
    uint32_t now_ms = app_ble_ci_state_now_ms();

    (void)host_version;
    (void)protocol_version;
    s_ci_state.initiator_mode = initiator_mode;
    s_ci_state.actual_state = app_ble_ci_state_classify(
                                  s_ci_state.actual_ci,
                                  s_ci_state.actual_latency);
    s_ci_state.session_id = session_id;
    s_ci_state.lease_deadline_ms = now_ms + APP_BLE_CI_HOST_LEASE_MS;
    s_ci_state.disconnect_required = false;
    s_ci_state.expected_profile =
        app_ble_ci_state_profile(s_ci_state.desired_state);
    target = app_ble_ci_state_effective_target(s_ci_state.desired_state);
    if (target != NULL)
    {
        s_ci_state.target_ci = target->ci;
        s_ci_state.target_latency = target->latency;
        s_ci_state.target_timeout = target->timeout;
    }
    s_ci_state.transition_id++;
    if (s_ci_state.transition_id == 0U)
    {
        s_ci_state.transition_id = 1U;
    }
    s_ci_state.transition_started_ms = now_ms;
    s_ci_state.transport_started_ms = now_ms;
    s_ci_state.wait_budget_ms = APP_BLE_CI_HOST_APPLY_TIMEOUT_MS;
    s_ci_state.stuck_budget_ms = APP_BLE_CI_HOST_APPLY_TIMEOUT_MS;
    s_ci_state.host_request_status = 0U;
    s_ci_state.target_reached = false;
    s_ci_state.request_pending = true;
    s_ci_state.transport_state = APP_BLE_CI_TRANSPORT_SUBMITTED;
    s_ci_state.normalized_result = APP_BLE_CI_RESULT_SUBMITTED;
    s_ci_state.failure_reason = APP_BLE_CI_FAILURE_NONE;
    app_ble_ci_state_link_changed(APP_BLE_CI_LINK_WAIT_HOST);
#if ZY100_BLE_DIAG_LOG_ENABLE
    DBG_DIRECT("[BLE_CI_HOST] enabled gen=%lu c=%u session=%lu host=%lu proto=%u init=%u ci=%u lat=%u",
               (unsigned long)s_ci_state.generation,
               conn_id,
               (unsigned long)session_id,
               (unsigned long)host_version,
               protocol_version,
               (uint32_t)initiator_mode,
               s_ci_state.actual_ci,
               s_ci_state.actual_latency);
#endif
    return true;
}

bool app_ble_ci_state_enable_windows_central(uint8_t conn_id,
                                              uint32_t session_id,
                                              uint32_t windows_build,
                                              uint32_t profile_mask,
                                              uint32_t profile_signature,
                                              bool notify_subscribed)
{
    uint8_t protocol_version = (uint8_t)
        ((profile_mask & ZY100_BLE_HOST_PROTOCOL_MASK) >>
         ZY100_BLE_HOST_PROTOCOL_SHIFT);

    s_host_enable_reject_detail = 0U;
    if (!s_ci_state.connected || (s_ci_state.conn_id != conn_id) ||
        (session_id == 0U) || !notify_subscribed ||
        (windows_build < APP_BLE_CI_WINDOWS_MIN_BUILD) ||
        (protocol_version != ZY100_BLE_HOST_CI_PROTOCOL_VERSION_V3) ||
        (((profile_mask & ZY100_BLE_HOST_PLATFORM_MASK) >>
          ZY100_BLE_HOST_PLATFORM_SHIFT) !=
         (uint32_t)ZY100_BLE_HOST_PLATFORM_WINDOWS_LEGACY) ||
        ((profile_mask & APP_BLE_CI_HOST_PROFILE_MASK_REQUIRED) !=
         APP_BLE_CI_HOST_PROFILE_MASK_REQUIRED) ||
        (profile_signature != ZY100_BLE_HOST_PROFILE_SIGNATURE))
    {
        s_host_enable_reject_detail =
            ZY100_BLE_HOST_CI_ENABLE_DETAIL_CAPABILITY_INVALID;
        DBG_DIRECT("[BLE_CI_HOST] enable_reject c=%u session=%lu build=%lu mask=0x%08lX sig=0x%08lX notify=%u settled=%u",
                   conn_id,
                   (unsigned long)session_id,
                   (unsigned long)windows_build,
                   (unsigned long)profile_mask,
                   (unsigned long)profile_signature,
                   notify_subscribed ? 1U : 0U,
                   app_ble_conn_param_mgr_is_settled(conn_id) ? 1U : 0U);
        return false;
    }
    if (!app_ble_conn_param_mgr_is_settled(conn_id))
    {
        s_host_enable_reject_detail =
            ZY100_BLE_HOST_CI_ENABLE_DETAIL_CI_SETTLING;
        DBG_DIRECT("[BLE_CI_HOST] enable_wait_settled c=%u session=%lu ci=%u lat=%u",
                   conn_id,
                   (unsigned long)session_id,
                   s_ci_state.actual_ci,
                   s_ci_state.actual_latency);
        return false;
    }
    return app_ble_ci_state_enable_host_central(
               conn_id,
               session_id,
               windows_build,
               protocol_version,
               APP_BLE_CI_INITIATOR_WINDOWS_CENTRAL);
}

bool app_ble_ci_state_enable_android_central(uint8_t conn_id,
                                             uint32_t session_id,
                                             uint32_t android_api_level,
                                             uint32_t profile_mask,
                                             uint32_t profile_signature,
                                             bool notify_subscribed)
{
    uint8_t protocol_version = (uint8_t)
        ((profile_mask & ZY100_BLE_HOST_PROTOCOL_MASK) >>
         ZY100_BLE_HOST_PROTOCOL_SHIFT);
    uint8_t platform = (uint8_t)
        ((profile_mask & ZY100_BLE_HOST_PLATFORM_MASK) >>
         ZY100_BLE_HOST_PLATFORM_SHIFT);

    s_host_enable_reject_detail = 0U;
    if (!s_ci_state.connected || (s_ci_state.conn_id != conn_id) ||
        (session_id == 0U) || !notify_subscribed ||
        (android_api_level < APP_BLE_CI_ANDROID_MIN_API_LEVEL) ||
        (protocol_version != ZY100_BLE_HOST_CI_PROTOCOL_VERSION_V3) ||
        (platform != (uint8_t)ZY100_BLE_HOST_PLATFORM_ANDROID) ||
        ((profile_mask & APP_BLE_CI_HOST_PROFILE_MASK_REQUIRED) !=
         APP_BLE_CI_HOST_PROFILE_MASK_REQUIRED) ||
        (profile_signature != ZY100_BLE_HOST_ANDROID_SIGNATURE))
    {
        s_host_enable_reject_detail =
            ZY100_BLE_HOST_CI_ENABLE_DETAIL_CAPABILITY_INVALID;
        DBG_DIRECT("[BLE_CI_HOST] android_enable_reject c=%u session=%lu api=%lu mask=0x%08lX sig=0x%08lX notify=%u settled=%u",
                   conn_id,
                   (unsigned long)session_id,
                   (unsigned long)android_api_level,
                   (unsigned long)profile_mask,
                   (unsigned long)profile_signature,
                   notify_subscribed ? 1U : 0U,
                   app_ble_conn_param_mgr_is_settled(conn_id) ? 1U : 0U);
        return false;
    }
    if (!app_ble_conn_param_mgr_is_settled(conn_id))
    {
        s_host_enable_reject_detail =
            ZY100_BLE_HOST_CI_ENABLE_DETAIL_CI_SETTLING;
        DBG_DIRECT("[BLE_CI_HOST] android_enable_wait c=%u session=%lu ci=%u lat=%u",
                   conn_id,
                   (unsigned long)session_id,
                   s_ci_state.actual_ci,
                   s_ci_state.actual_latency);
        return false;
    }
    return app_ble_ci_state_enable_host_central(
               conn_id,
               session_id,
               android_api_level,
               protocol_version,
               APP_BLE_CI_INITIATOR_ANDROID_CENTRAL);
}

bool app_ble_ci_state_windows_central_active(void)
{
    return s_ci_state.connected &&
           (s_ci_state.initiator_mode ==
            APP_BLE_CI_INITIATOR_WINDOWS_CENTRAL);
}

bool app_ble_ci_state_android_central_active(void)
{
    return s_ci_state.connected &&
           (s_ci_state.initiator_mode ==
            APP_BLE_CI_INITIATOR_ANDROID_CENTRAL);
}

bool app_ble_ci_state_host_session_managed(void)
{
    return app_ble_ci_state_host_active();
}

bool app_ble_ci_state_host_standby_settled(void)
{
    return app_ble_ci_state_host_session_managed() &&
           (s_ci_state.desired_state == APP_BLE_CI_STATE_STANDBY) &&
           s_ci_state.target_reached &&
           (s_ci_state.transport_state ==
            APP_BLE_CI_TRANSPORT_TERMINAL_SUCCESS) &&
           (s_ci_state.link_phase == APP_BLE_CI_LINK_APPLIED);
}

bool app_ble_ci_state_peripheral_llcp_allowed(uint16_t ci_min,
                                               uint16_t ci_max,
                                               uint16_t latency,
                                               uint16_t timeout)
{
    (void)ci_min;
    (void)ci_max;
    (void)latency;
    (void)timeout;
    return false;
}

void app_ble_ci_state_note_host_transport_ready(uint8_t conn_id)
{
    if (s_ci_state.connected &&
        (s_ci_state.conn_id == conn_id) &&
        !app_ble_ci_state_host_session_managed())
    {
        s_ci_state.lease_deadline_ms =
            app_ble_ci_state_now_ms() + APP_BLE_CI_HOST_LEASE_MS;
    }
}

bool app_ble_ci_state_begin_ota_host_wait(uint8_t conn_id,
                                          uint32_t wait_ms)
{
    uint32_t now_ms;

    if (!s_ci_state.connected || (s_ci_state.conn_id != conn_id) ||
        (wait_ms == 0U) ||
        (s_ota_host_phase == APP_BLE_CI_OTA_HOST_HIGH_LEASED))
    {
        return false;
    }

    now_ms = app_ble_ci_state_now_ms();
    if (s_ota_host_phase == APP_BLE_CI_OTA_HOST_WAIT_HIGH)
    {
        s_ci_state.lease_deadline_ms = now_ms + wait_ms;
        return true;
    }

    app_ble_ci_state_read_actual(conn_id);
    s_ota_host_phase = APP_BLE_CI_OTA_HOST_WAIT_HIGH;
    s_ci_state.desired_state = APP_BLE_CI_STATE_ONLINE_HIGH;
    s_ci_state.transition_started_ms = now_ms;
    s_ci_state.lease_deadline_ms = now_ms + wait_ms;
    s_ci_state.target_reached = app_ble_ci_state_ota_host_values_match(
                                    APP_BLE_CI_STATE_ONLINE_HIGH,
                                    s_ci_state.actual_ci,
                                    s_ci_state.actual_latency);
    s_ci_state.request_pending = !s_ci_state.target_reached;
    s_ci_state.reconnect_required = false;
    s_ci_state.disconnect_required = false;
    s_ci_state.transport_state = s_ci_state.target_reached ?
        APP_BLE_CI_TRANSPORT_TERMINAL_SUCCESS :
        APP_BLE_CI_TRANSPORT_SUBMITTED;
    s_ci_state.normalized_result = s_ci_state.target_reached ?
        APP_BLE_CI_RESULT_TARGET_REACHED : APP_BLE_CI_RESULT_SUBMITTED;
    s_ci_state.failure_reason = APP_BLE_CI_FAILURE_NONE;
    DBG_DIRECT("[BLE_CI_OTA_HOST] intent_arm ci=%u l=%u llcp=0",
               s_ci_state.actual_ci,
               s_ci_state.actual_latency);
    return true;
}

bool app_ble_ci_state_begin_ota_host_lease(uint8_t conn_id,
                                           app_ble_ci_state_t target,
                                           uint32_t lease_ms,
                                           const char *reason)
{
    const app_ble_ci_target_t *target_values;
    uint32_t now_ms;

    if (!s_ci_state.connected || (s_ci_state.conn_id != conn_id) ||
        (target != APP_BLE_CI_STATE_ONLINE_HIGH) || (lease_ms == 0U))
    {
        return false;
    }

    if (s_ota_host_phase == APP_BLE_CI_OTA_HOST_HIGH_LEASED)
    {
        if (!app_ble_ci_state_ota_host_lease_valid(conn_id, target))
        {
            app_ble_ci_state_fail_ota_host_lease("ota_host_lease_refresh_invalid");
            return false;
        }
        s_ci_state.lease_deadline_ms = app_ble_ci_state_now_ms() + lease_ms;
        DBG_DIRECT("[BLE_CI_OTA_HOST] refresh c=%u ms=%lu",
                   conn_id,
                   (unsigned long)lease_ms);
        return true;
    }

    app_ble_ci_state_read_actual(conn_id);
    if (!app_ble_conn_param_mgr_is_settled(conn_id) ||
        !app_ble_ci_state_ota_host_values_match(target,
                                                s_ci_state.actual_ci,
                                                s_ci_state.actual_latency))
    {
        DBG_DIRECT("[BLE_CI_OTA_HOST] reject ci=%u l=%u s=%u",
                   s_ci_state.actual_ci,
                   s_ci_state.actual_latency,
                   app_ble_conn_param_mgr_is_settled(conn_id) ? 1U : 0U);
        return false;
    }

    now_ms = app_ble_ci_state_now_ms();
    target_values = app_ble_ci_state_target(target);
    s_ota_host_phase = APP_BLE_CI_OTA_HOST_HIGH_LEASED;
    s_ci_state.source_state = s_ci_state.actual_state;
    s_ci_state.desired_state = target;
    s_ci_state.actual_state = APP_BLE_CI_STATE_ONLINE_HIGH;
    s_ci_state.expected_profile = APP_BLE_CI_HOST_PROFILE_THROUGHPUT;
    s_ci_state.transition_id++;
    if (s_ci_state.transition_id == 0U)
    {
        s_ci_state.transition_id = 1U;
    }
    s_ci_state.transition_started_ms = now_ms;
    s_ci_state.transport_started_ms = now_ms;
    s_ci_state.lease_deadline_ms = now_ms + lease_ms;
    s_ci_state.wait_budget_ms = lease_ms;
    s_ci_state.stuck_budget_ms = lease_ms;
    s_ci_state.target_ci = (target_values != NULL) ? target_values->ci : 0U;
    s_ci_state.target_latency =
        (target_values != NULL) ? target_values->latency : 0U;
    s_ci_state.target_timeout =
        (target_values != NULL) ? target_values->timeout : 0U;
    s_ci_state.host_request_status = 1U;
    s_ci_state.target_reached = true;
    s_ci_state.request_pending = false;
    s_ci_state.gap_pending_seen = false;
    s_ci_state.reconnect_required = false;
    s_ci_state.disconnect_required = false;
    s_ci_state.timeout_logged = false;
    s_ci_state.transport_state = APP_BLE_CI_TRANSPORT_TERMINAL_SUCCESS;
    s_ci_state.normalized_result = APP_BLE_CI_RESULT_TARGET_REACHED;
    s_ci_state.failure_reason = APP_BLE_CI_FAILURE_NONE;
    strncpy(s_transition_reason,
            (reason != NULL) ? reason : "ota_host_high_speed",
            sizeof(s_transition_reason) - 1U);
    s_transition_reason[sizeof(s_transition_reason) - 1U] = '\0';
    DBG_DIRECT("[BLE_CI_OTA_HOST] leased ci=%u l=%u ms=%lu llcp=0",
               s_ci_state.actual_ci,
               s_ci_state.actual_latency,
               (unsigned long)lease_ms);
    return true;
}

bool app_ble_ci_state_ota_host_lease_valid(uint8_t conn_id,
                                           app_ble_ci_state_t target)
{
    uint32_t now_ms;

    if ((s_ota_host_phase != APP_BLE_CI_OTA_HOST_HIGH_LEASED) ||
        !s_ci_state.connected ||
        (s_ci_state.conn_id != conn_id) ||
        (s_ci_state.desired_state != target))
    {
        return false;
    }
    app_ble_ci_state_read_actual(conn_id);
    now_ms = app_ble_ci_state_now_ms();
    return ((int32_t)(now_ms - s_ci_state.lease_deadline_ms) < 0) &&
           app_ble_conn_param_mgr_is_settled(conn_id) &&
           app_ble_ci_state_ota_host_values_match(target,
                                                   s_ci_state.actual_ci,
                                                   s_ci_state.actual_latency);
}

bool app_ble_ci_state_release_ota_host_lease(uint8_t conn_id,
                                             app_ble_ci_state_t fallback_state,
                                             const char *reason)
{
    uint8_t request_result;

    if ((s_ota_host_phase == APP_BLE_CI_OTA_HOST_NONE) ||
        !s_ci_state.connected ||
        (s_ci_state.conn_id != conn_id) ||
        (app_ble_ci_state_effective_target(fallback_state) == NULL))
    {
        return false;
    }

    /*
     * A local admission rejection is not a link failure.  Drop only the OTA
     * lease, clear the disconnect latch, and publish the normal Host-CI
     * fallback target so the central can withdraw the throughput profile.
     */
    s_ota_host_phase = APP_BLE_CI_OTA_HOST_NONE;
    s_ci_state.reconnect_required = false;
    s_ci_state.disconnect_required = false;
    s_ci_state.failure_reason = APP_BLE_CI_FAILURE_NONE;
    request_result = app_ble_ci_state_request(
                         fallback_state,
                         (reason != NULL) ? reason : "ota_host_release");
    DBG_DIRECT("[BLE_CI_OTA_HOST] release c=%u fb=%s rc=%u disconnect=0 r=%s",
               conn_id,
               app_ble_ci_state_name(fallback_state),
               request_result,
               (reason != NULL) ? reason : "ota_host_release");
    return request_result != (uint8_t)APP_BLE_CONN_PARAM_REQ_FAILED;
}

void app_ble_ci_state_fail_ota_host_lease(const char *reason)
{
    if ((s_ota_host_phase == APP_BLE_CI_OTA_HOST_NONE) ||
        !s_ci_state.connected)
    {
        return;
    }
    s_ci_state.target_reached = false;
    s_ci_state.request_pending = false;
    s_ci_state.reconnect_required = true;
    s_ci_state.disconnect_required = true;
    s_ci_state.transport_state = APP_BLE_CI_TRANSPORT_TERMINAL_FAIL;
    s_ci_state.normalized_result = APP_BLE_CI_RESULT_TERMINAL_FAIL;
    s_ci_state.failure_reason = APP_BLE_CI_FAILURE_RETRY_EXHAUSTED;
    ZY100_LOG_WARN("[BLE_CI_OTA_HOST] fail r=%s disconnect=1 llcp=0",
               (reason != NULL) ? reason : "ota_host_lease_fail");
}

bool app_ble_ci_state_ota_host_wait_active(void)
{
    return s_ota_host_phase == APP_BLE_CI_OTA_HOST_WAIT_HIGH;
}

void app_ble_ci_state_cancel_ota_host_wait(void)
{
    if ((s_ota_host_phase != APP_BLE_CI_OTA_HOST_WAIT_HIGH) ||
        !s_ci_state.connected)
    {
        return;
    }
    app_ble_ci_state_fail_ota_host_lease(NULL);
    s_ota_host_phase = APP_BLE_CI_OTA_HOST_NONE;
}

uint32_t app_ble_ci_state_host_enable_reject_detail(void)
{
    return s_host_enable_reject_detail;
}
void app_ble_ci_state_note_host_ping(uint8_t conn_id, uint32_t session_id)
{
    if (app_ble_ci_state_host_session_managed() &&
        (s_ci_state.conn_id == conn_id) &&
        (s_ci_state.session_id == session_id) &&
        !app_ble_ci_state_host_standby_settled())
    {
        s_ci_state.lease_deadline_ms =
            app_ble_ci_state_now_ms() + APP_BLE_CI_HOST_LEASE_MS;
    }
}

bool app_ble_ci_state_host_profile_result(uint8_t conn_id,
                                          uint32_t session_id,
                                          uint32_t generation,
                                          uint32_t transition_id,
                                          app_ble_ci_host_profile_t profile,
                                          uint8_t request_status,
                                          app_ble_ci_host_result_t result)
{
    if (!app_ble_ci_state_host_session_managed() ||
        (s_ci_state.conn_id != conn_id) ||
        (s_ci_state.session_id != session_id) ||
        ((s_ci_state.generation & 0xFFFFUL) !=
         (generation & 0xFFFFUL)) ||
        ((s_ci_state.transition_id & 0xFFFFUL) !=
         (transition_id & 0xFFFFUL)) ||
        (s_ci_state.expected_profile != profile))
    {
#if ZY100_BLE_DIAG_LOG_ENABLE
        DBG_DIRECT("[BLE_CI_HOST] stale c=%u session=%lu gen=%lu trans=%lu prof=%u",
                   conn_id,
                   (unsigned long)session_id,
                   (unsigned long)generation,
                   (unsigned long)transition_id,
                   (uint32_t)profile);
#endif
        return false;
    }
    if (app_ble_ci_state_android_central_active() &&
        (result != APP_BLE_CI_HOST_RESULT_REQUEST_ACCEPTED) &&
        (result != APP_BLE_CI_HOST_RESULT_REQUEST_FAILED))
    {
#if ZY100_BLE_DIAG_LOG_ENABLE
        DBG_DIRECT("[BLE_CI_HOST] android_result_reject rc=%u trans=%lu",
                   (uint32_t)result,
                   (unsigned long)transition_id);
#endif
        return false;
    }
    s_ci_state.host_request_status = request_status;
    if (result == APP_BLE_CI_HOST_RESULT_REQUEST_FAILED)
    {
        s_ci_state.transport_state = APP_BLE_CI_TRANSPORT_TERMINAL_FAIL;
        s_ci_state.normalized_result = APP_BLE_CI_RESULT_TERMINAL_FAIL;
        s_ci_state.failure_reason = APP_BLE_CI_FAILURE_GAP_FAIL;
        s_ci_state.request_pending = false;
        s_ci_state.disconnect_required = true;
        app_ble_ci_state_link_changed(APP_BLE_CI_LINK_FAILED);
    }
    else if (result == APP_BLE_CI_HOST_RESULT_REQUEST_ACCEPTED)
    {
        s_ci_state.transport_state = APP_BLE_CI_TRANSPORT_PENDING;
        s_ci_state.normalized_result = APP_BLE_CI_RESULT_PENDING;
        app_ble_ci_state_link_changed(APP_BLE_CI_LINK_HOST_ACCEPTED);
    }
    else if (result == APP_BLE_CI_HOST_RESULT_ACTUAL_STABLE)
    {
        /* PC data is advisory. maintain() still reads controller actuals. */
        s_ci_state.notify_revision++;
    }
    return true;
}

uint8_t app_ble_ci_state_request(app_ble_ci_state_t target_state,
                                 const char *reason)
{
    const app_ble_ci_target_t *target =
        app_ble_ci_state_effective_target(target_state);
    uint32_t now_ms = app_ble_ci_state_now_ms();
    bool new_transition;

    if (!s_ci_state.connected || (target == NULL) ||
        !app_ble_power_conn_valid(s_ci_state.conn_id))
    {
        return (uint8_t)APP_BLE_CONN_PARAM_REQ_FAILED;
    }
    if (s_ci_state.reconnect_required)
    {
#if ZY100_BLE_DIAG_LOG_ENABLE
        DBG_DIRECT("[BLE_CI_SM] request_reject id=%lu want=%s req=%s tx=%s act=wait_disconnect",
                   (unsigned long)s_ci_state.transition_id,
                   app_ble_ci_state_name(s_ci_state.desired_state),
                   app_ble_ci_state_name(target_state),
                   app_ble_ci_transport_name(s_ci_state.transport_state));
#endif
        return (uint8_t)APP_BLE_CONN_PARAM_REQ_BUSY;
    }

    app_ble_ci_state_read_actual(s_ci_state.conn_id);
    s_ci_state.actual_state = app_ble_ci_state_classify(
                                  s_ci_state.actual_ci,
                                  s_ci_state.actual_latency);
    if (s_ota_host_phase != APP_BLE_CI_OTA_HOST_NONE)
    {
        if ((target_state != APP_BLE_CI_STATE_ONLINE_HIGH) ||
            !app_ble_conn_param_mgr_is_settled(s_ci_state.conn_id) ||
            !app_ble_ci_state_ota_host_values_match(
                target_state,
                s_ci_state.actual_ci,
                s_ci_state.actual_latency))
        {
            DBG_DIRECT("[BLE_CI_OTA_HOST] request_reject target=%s ci=%u lat=%u llcp=0",
                       app_ble_ci_state_name(target_state),
                       s_ci_state.actual_ci,
                       s_ci_state.actual_latency);
            app_ble_ci_state_fail_ota_host_lease("ota_host_target_mismatch");
            return (uint8_t)APP_BLE_CONN_PARAM_REQ_FAILED;
        }
        s_ci_state.target_reached = true;
        s_ci_state.request_pending = false;
        s_ci_state.transport_state =
            APP_BLE_CI_TRANSPORT_TERMINAL_SUCCESS;
        s_ci_state.normalized_result =
            APP_BLE_CI_RESULT_TARGET_REACHED;
        s_ci_state.failure_reason = APP_BLE_CI_FAILURE_NONE;
        return (uint8_t)APP_BLE_CONN_PARAM_REQ_ALREADY_REACHED;
    }
    new_transition = (target_state != s_ci_state.desired_state);

    if (new_transition)
    {
        s_ci_state.transition_id++;
        if (s_ci_state.transition_id == 0U)
        {
            s_ci_state.transition_id = 1U;
        }
        s_ci_state.source_state = s_ci_state.actual_state;
        s_ci_state.desired_state = target_state;
        s_ci_state.target_ci = target->ci;
        s_ci_state.target_latency = target->latency;
        s_ci_state.target_timeout = target->timeout;
        s_ci_state.transition_started_ms = now_ms;
        s_ci_state.source_effective_event_ms =
            app_ble_ci_state_effective_event_ms(s_ci_state.actual_ci,
                                                s_ci_state.actual_latency);
        s_ci_state.wait_budget_ms =
            app_ble_ci_state_compute_wait_budget(s_ci_state.actual_ci,
                                                 s_ci_state.actual_latency);
        s_ci_state.stuck_budget_ms =
            app_ble_ci_state_compute_stuck_budget(s_ci_state.source_state,
                                                  target_state);
        s_ci_state.target_reached = false;
        s_ci_state.timeout_logged = false;
        s_ci_state.gap_pending_seen = false;
        s_ci_state.reconnect_required = false;
        s_ci_state.transport_state = APP_BLE_CI_TRANSPORT_IDLE;
        s_ci_state.normalized_result = APP_BLE_CI_RESULT_IDLE;
        s_ci_state.failure_reason = APP_BLE_CI_FAILURE_NONE;
        s_ci_state.transport_started_ms = now_ms;
        s_ci_state.last_gap_cause = 0U;
        strncpy(s_transition_reason,
                (reason != NULL) ? reason : "state_request",
                sizeof(s_transition_reason) - 1U);
        s_transition_reason[sizeof(s_transition_reason) - 1U] = '\0';
#if ZY100_BLE_DIAG_LOG_ENABLE
        DBG_DIRECT("[BLE_CI_SM] transition gen=%lu id=%lu from=%s to=%s trigger=%s ref_ci=%u ref_lat=%u ref_to=%u ci=%u lat=%u src_ms=%lu budget=%lu stuck=%lu",
                   (unsigned long)s_ci_state.generation,
                   (unsigned long)s_ci_state.transition_id,
                   app_ble_ci_state_name(s_ci_state.source_state),
                   app_ble_ci_state_name(target_state),
                   s_transition_reason,
                   target->ci,
                   target->latency,
                   target->timeout,
                   s_ci_state.actual_ci,
                   s_ci_state.actual_latency,
                   (unsigned long)s_ci_state.source_effective_event_ms,
                   (unsigned long)s_ci_state.wait_budget_ms,
                   (unsigned long)s_ci_state.stuck_budget_ms);
#endif
    }

    if (app_ble_ci_state_host_session_managed())
    {
        if (new_transition)
        {
            s_ci_state.expected_profile =
                app_ble_ci_state_profile(target_state);
            s_ci_state.wait_budget_ms = APP_BLE_CI_HOST_APPLY_TIMEOUT_MS;
            s_ci_state.stuck_budget_ms = APP_BLE_CI_HOST_APPLY_TIMEOUT_MS;
            s_ci_state.transport_started_ms = now_ms;
            s_ci_state.host_request_status = 0U;
            s_ci_state.lease_deadline_ms =
                now_ms + APP_BLE_CI_HOST_LEASE_MS;
            s_ci_state.transport_state = APP_BLE_CI_TRANSPORT_SUBMITTED;
            s_ci_state.normalized_result = APP_BLE_CI_RESULT_SUBMITTED;
            s_ci_state.request_pending = true;
            app_ble_ci_state_link_changed(APP_BLE_CI_LINK_WAIT_HOST);
#if ZY100_BLE_DIAG_LOG_ENABLE
            DBG_DIRECT("[BLE_CI_HOST] wait_host gen=%lu trans=%lu session=%lu state=%s prof=%u ci=%u lat=%u budget=%lu",
                       (unsigned long)s_ci_state.generation,
                       (unsigned long)s_ci_state.transition_id,
                       (unsigned long)s_ci_state.session_id,
                       app_ble_ci_state_name(target_state),
                       (uint32_t)s_ci_state.expected_profile,
                       s_ci_state.actual_ci,
                       s_ci_state.actual_latency,
                       (unsigned long)s_ci_state.wait_budget_ms);
#endif
        }
        if ((s_ci_state.host_request_status != 0U) &&
            app_ble_ci_state_host_values_match(target_state,
                                               s_ci_state.actual_ci,
                                               s_ci_state.actual_latency))
        {
            s_ci_state.target_reached = true;
            s_ci_state.request_pending = false;
            s_ci_state.transport_state =
                APP_BLE_CI_TRANSPORT_TERMINAL_SUCCESS;
            s_ci_state.normalized_result =
                APP_BLE_CI_RESULT_TARGET_REACHED;
            app_ble_ci_state_link_changed(APP_BLE_CI_LINK_APPLIED);
            return (uint8_t)APP_BLE_CONN_PARAM_REQ_ALREADY_REACHED;
        }
        return (uint8_t)APP_BLE_CONN_PARAM_REQ_SUBMITTED;
    }

    s_ci_state.expected_profile = app_ble_ci_state_profile(target_state);
    s_ci_state.target_reached = false;
    s_ci_state.request_pending = true;
    s_ci_state.transport_state = APP_BLE_CI_TRANSPORT_WAIT_HOST;
    s_ci_state.normalized_result = APP_BLE_CI_RESULT_WAIT_HOST;
    s_ci_state.failure_reason = APP_BLE_CI_FAILURE_NONE;
#if ZY100_BLE_DIAG_LOG_ENABLE
    DBG_DIRECT("[BLE_CI_HOST] wait id=%lu state=%s prof=%u ci=%u lat=%u",
               (unsigned long)s_ci_state.transition_id,
               app_ble_ci_state_name(target_state),
               (uint32_t)s_ci_state.expected_profile,
               s_ci_state.actual_ci,
               s_ci_state.actual_latency);
#endif
    return (uint8_t)APP_BLE_CONN_PARAM_REQ_BUSY;
}

void app_ble_ci_state_on_gap_update(uint8_t conn_id,
                                    uint8_t status,
                                    uint16_t cause,
                                    uint16_t actual_ci,
                                    uint16_t actual_latency,
                                    uint16_t actual_timeout)
{
    const app_ble_ci_target_t *target;
    bool actual_matched;
    bool transport_settled;
    bool matched;
    uint32_t now_ms;

    if (!s_ci_state.connected || (s_ci_state.conn_id != conn_id))
    {
        return;
    }
    target = app_ble_ci_state_effective_target(s_ci_state.desired_state);
    s_ci_state.actual_ci = actual_ci;
    s_ci_state.actual_latency = actual_latency;
    s_ci_state.actual_timeout = actual_timeout;
    s_ci_state.actual_state =
        app_ble_ci_state_classify(actual_ci, actual_latency);
    s_ci_state.last_gap_cause = cause;
    if (s_ota_host_phase != APP_BLE_CI_OTA_HOST_NONE)
    {
        actual_matched = app_ble_ci_state_ota_host_values_match(
                             s_ci_state.desired_state,
                             actual_ci,
                             actual_latency);
        transport_settled =
            app_ble_conn_param_mgr_is_settled(s_ci_state.conn_id);
    }
    else if (app_ble_ci_state_host_session_managed())
    {
        actual_matched = (s_ci_state.host_request_status != 0U) &&
                         app_ble_ci_state_host_values_match(
                             s_ci_state.desired_state,
                             actual_ci,
                             actual_latency);
        transport_settled = true;
    }
    else
    {
        actual_matched = app_ble_ci_state_values_match(target,
                                                       actual_ci,
                                                       actual_latency);
        transport_settled =
            app_ble_conn_param_mgr_is_settled(s_ci_state.conn_id);
    }
    matched = actual_matched && transport_settled;
    now_ms = app_ble_ci_state_now_ms();
    if (s_ci_state.initiator_mode == APP_BLE_CI_INITIATOR_HOST_REQUIRED)
    {
        s_ci_state.target_reached = false;
        s_ci_state.request_pending = true;
        s_ci_state.transport_state = APP_BLE_CI_TRANSPORT_WAIT_HOST;
        s_ci_state.normalized_result = APP_BLE_CI_RESULT_WAIT_HOST;
        return;
    }
    if (s_ci_state.reconnect_required)
    {
#if ZY100_BLE_DIAG_LOG_ENABLE
        DBG_DIRECT("[BLE_CI_SM] gap_stale_after_stuck gen=%lu id=%lu status=%u cause=0x%x ci=%u lat=%u",
                   (unsigned long)s_ci_state.generation,
                   (unsigned long)s_ci_state.transition_id,
                   status,
                   cause,
                   actual_ci,
                   actual_latency);
#endif
        return;
    }
    if (matched)
    {
        s_ci_state.transport_state =
            APP_BLE_CI_TRANSPORT_TERMINAL_SUCCESS;
        s_ci_state.normalized_result = APP_BLE_CI_RESULT_TARGET_REACHED;
        s_ci_state.failure_reason = APP_BLE_CI_FAILURE_NONE;
        s_ci_state.target_reached = true;
        s_ci_state.request_pending = false;
        s_ci_state.reconnect_required = false;
        if (app_ble_ci_state_host_session_managed())
        {
            app_ble_ci_state_link_changed(APP_BLE_CI_LINK_APPLIED);
        }
    }
    else if (status == GAP_CONN_PARAM_UPDATE_STATUS_PENDING)
    {
        if ((s_ci_state.transport_state !=
             APP_BLE_CI_TRANSPORT_SUBMITTED) &&
            (s_ci_state.transport_state !=
             APP_BLE_CI_TRANSPORT_PENDING))
        {
            s_ci_state.transport_started_ms = now_ms;
        }
        s_ci_state.transport_state = APP_BLE_CI_TRANSPORT_PENDING;
        s_ci_state.normalized_result = APP_BLE_CI_RESULT_PENDING;
        s_ci_state.failure_reason = APP_BLE_CI_FAILURE_NONE;
        s_ci_state.gap_pending_seen = true;
        s_ci_state.target_reached = false;
        s_ci_state.request_pending = true;
    }
    else if ((status == GAP_CONN_PARAM_UPDATE_STATUS_SUCCESS) ||
             (status == GAP_CONN_PARAM_UPDATE_STATUS_FAIL))
    {
        if (app_ble_ci_state_host_session_managed() &&
            (status == GAP_CONN_PARAM_UPDATE_STATUS_FAIL))
        {
            s_ci_state.transport_state =
                APP_BLE_CI_TRANSPORT_TERMINAL_FAIL;
            s_ci_state.normalized_result =
                APP_BLE_CI_RESULT_TERMINAL_FAIL;
            s_ci_state.failure_reason = APP_BLE_CI_FAILURE_GAP_FAIL;
            s_ci_state.target_reached = false;
            s_ci_state.request_pending = false;
            s_ci_state.disconnect_required = true;
            app_ble_ci_state_link_changed(APP_BLE_CI_LINK_FAILED);
        }
        else
        {
        /*
         * A raw GAP terminal event is not yet a semantic terminal failure.
         * The serialized transport manager may retry an actual-parameter
         * mismatch or a transient controller rejection.  Keep the semantic
         * transition alive until a later matching update or its bounded
         * retry window expires.
         */
        s_ci_state.transport_state = APP_BLE_CI_TRANSPORT_RETRYING;
        s_ci_state.normalized_result = APP_BLE_CI_RESULT_RETRYING;
        s_ci_state.failure_reason =
            (status == GAP_CONN_PARAM_UPDATE_STATUS_SUCCESS) ?
            APP_BLE_CI_FAILURE_SUCCESS_MISMATCH :
            APP_BLE_CI_FAILURE_GAP_FAIL;
        s_ci_state.target_reached = false;
        s_ci_state.request_pending = true;
        }
    }
#if ZY100_BLE_DIAG_LOG_ENABLE
    DBG_DIRECT("[BLE_CI_SM] gap gen=%lu id=%lu want=%s tx=%s norm=%u fail=%u status=%u cause=0x%x state=%s ci=%u lat=%u tout=%u match=%u settled=%u target_match=%u transition_ms=%lu llcp_ms=%lu budget_ms=%lu",
               (unsigned long)s_ci_state.generation,
               (unsigned long)s_ci_state.transition_id,
               app_ble_ci_state_name(s_ci_state.desired_state),
               app_ble_ci_transport_name(s_ci_state.transport_state),
               (uint32_t)s_ci_state.normalized_result,
               (uint32_t)s_ci_state.failure_reason,
               status,
               cause,
               app_ble_ci_state_name(s_ci_state.actual_state),
               actual_ci,
               actual_latency,
               actual_timeout,
               actual_matched ? 1U : 0U,
               transport_settled ? 1U : 0U,
               matched ? 1U : 0U,
               (unsigned long)(now_ms -
                                s_ci_state.transition_started_ms),
               (unsigned long)(now_ms -
                                s_ci_state.transport_started_ms),
               (unsigned long)s_ci_state.wait_budget_ms);
#endif
}

void app_ble_ci_state_maintain(void)
{
    const app_ble_ci_target_t *target;
    uint32_t now_ms;
    uint32_t elapsed_ms;
    bool matched;

    if (!s_ci_state.connected ||
        !app_ble_power_conn_valid(s_ci_state.conn_id))
    {
        return;
    }
    target = app_ble_ci_state_effective_target(s_ci_state.desired_state);
    if (target == NULL)
    {
        return;
    }

    app_ble_ci_state_read_actual(s_ci_state.conn_id);
    s_ci_state.actual_state =
        app_ble_ci_state_classify(s_ci_state.actual_ci,
                                  s_ci_state.actual_latency);
    matched = app_ble_ci_state_target_is_settled(target);
    now_ms = app_ble_ci_state_now_ms();
    app_ble_link_trace_note_liveness(&s_ci_state);
    elapsed_ms = (uint32_t)(now_ms - s_ci_state.transition_started_ms);
    if (s_ci_state.initiator_mode == APP_BLE_CI_INITIATOR_HOST_REQUIRED)
    {
        s_ci_state.target_reached = false;
        s_ci_state.request_pending = true;
        s_ci_state.transport_state = APP_BLE_CI_TRANSPORT_WAIT_HOST;
        s_ci_state.normalized_result = APP_BLE_CI_RESULT_WAIT_HOST;
        if ((s_ci_state.lease_deadline_ms != 0U) &&
            ((int32_t)(now_ms - s_ci_state.lease_deadline_ms) >= 0))
        {
            s_ci_state.request_pending = false;
            s_ci_state.disconnect_required = true;
            s_ci_state.transport_state =
                APP_BLE_CI_TRANSPORT_TERMINAL_FAIL;
            s_ci_state.normalized_result =
                APP_BLE_CI_RESULT_TERMINAL_FAIL;
            s_ci_state.failure_reason =
                APP_BLE_CI_FAILURE_RETRY_EXHAUSTED;
            s_ci_state.timeout_logged = true;
        }
        return;
    }
    if (s_ota_host_phase == APP_BLE_CI_OTA_HOST_WAIT_HIGH)
    {
        if (matched)
        {
            s_ci_state.target_reached = true;
            s_ci_state.request_pending = false;
            s_ci_state.transport_state =
                APP_BLE_CI_TRANSPORT_TERMINAL_SUCCESS;
            s_ci_state.normalized_result =
                APP_BLE_CI_RESULT_TARGET_REACHED;
            s_ci_state.failure_reason = APP_BLE_CI_FAILURE_NONE;
        }
        else
        {
            s_ci_state.target_reached = false;
            s_ci_state.request_pending = true;
        }
        if ((int32_t)(now_ms - s_ci_state.lease_deadline_ms) >= 0)
        {
            app_ble_ci_state_cancel_ota_host_wait();
        }
        return;
    }
    if (s_ota_host_phase == APP_BLE_CI_OTA_HOST_HIGH_LEASED)
    {
        if (((int32_t)(now_ms - s_ci_state.lease_deadline_ms) >= 0) ||
            !matched)
        {
            app_ble_ci_state_fail_ota_host_lease(
                ((int32_t)(now_ms - s_ci_state.lease_deadline_ms) >= 0) ?
                "ota_host_lease_timeout" : "ota_host_link_lost");
        }
        else
        {
            s_ci_state.target_reached = true;
            s_ci_state.request_pending = false;
            s_ci_state.transport_state =
                APP_BLE_CI_TRANSPORT_TERMINAL_SUCCESS;
            s_ci_state.normalized_result =
                APP_BLE_CI_RESULT_TARGET_REACHED;
            s_ci_state.failure_reason = APP_BLE_CI_FAILURE_NONE;
        }
        return;
    }
        if (matched)
    {
        if (!s_ci_state.target_reached)
        {
#if ZY100_BLE_DIAG_LOG_ENABLE
            DBG_DIRECT("[BLE_CI_SM] reached id=%lu state=%s src=poll ci=%u lat=%u tout=%u ms=%lu",
                       (unsigned long)s_ci_state.transition_id,
                       app_ble_ci_state_name(s_ci_state.desired_state),
                       s_ci_state.actual_ci,
                       s_ci_state.actual_latency,
                       s_ci_state.actual_timeout,
                       (unsigned long)elapsed_ms);
#endif
        }
        s_ci_state.target_reached = true;
        s_ci_state.request_pending = false;
        s_ci_state.reconnect_required = false;
        s_ci_state.transport_state =
            APP_BLE_CI_TRANSPORT_TERMINAL_SUCCESS;
        s_ci_state.normalized_result = APP_BLE_CI_RESULT_TARGET_REACHED;
        s_ci_state.failure_reason = APP_BLE_CI_FAILURE_NONE;
        if (app_ble_ci_state_host_session_managed() &&
            (s_ci_state.link_phase != APP_BLE_CI_LINK_APPLIED))
        {
            app_ble_ci_state_link_changed(APP_BLE_CI_LINK_APPLIED);
        }
        return;
    }

    s_ci_state.target_reached = false;
    if (app_ble_ci_state_host_session_managed())
    {
                if (((int32_t)(now_ms - s_ci_state.lease_deadline_ms) >= 0) ||
            (elapsed_ms >= APP_BLE_CI_HOST_APPLY_TIMEOUT_MS))
        {
            bool lease_expired =
                ((int32_t)(now_ms - s_ci_state.lease_deadline_ms) >= 0);
            s_ci_state.transport_state =
                APP_BLE_CI_TRANSPORT_TERMINAL_FAIL;
            s_ci_state.normalized_result =
                APP_BLE_CI_RESULT_TERMINAL_FAIL;
            s_ci_state.failure_reason =
                lease_expired ? APP_BLE_CI_FAILURE_LLCP_STUCK :
                APP_BLE_CI_FAILURE_RETRY_EXHAUSTED;
            s_ci_state.request_pending = false;
            s_ci_state.disconnect_required = true;
            s_ci_state.timeout_logged = true;
            app_ble_ci_state_link_changed(APP_BLE_CI_LINK_TIMEOUT);
            ZY100_LOG_ERROR("[BLE_CI][ERR] timeout c=%u trans=%lu why=%u ci=%u lat=%u",
                            s_ci_state.conn_id,
                            (unsigned long)s_ci_state.transition_id,
                            (uint32_t)s_ci_state.failure_reason,
                            s_ci_state.actual_ci, s_ci_state.actual_latency);
        }
        return;
    }

}

bool app_ble_ci_state_target_reached(app_ble_ci_state_t target)
{
    const app_ble_ci_target_t *target_values =
        app_ble_ci_state_effective_target(target);

    if (!s_ci_state.connected ||
        (s_ci_state.desired_state != target) ||
        (target_values == NULL))
    {
        return false;
    }
    app_ble_ci_state_read_actual(s_ci_state.conn_id);
    s_ci_state.target_reached =
        app_ble_ci_state_target_is_settled(target_values);
    if (s_ci_state.target_reached)
    {
        s_ci_state.request_pending = false;
        s_ci_state.reconnect_required = false;
        s_ci_state.transport_state =
            APP_BLE_CI_TRANSPORT_TERMINAL_SUCCESS;
        s_ci_state.normalized_result = APP_BLE_CI_RESULT_TARGET_REACHED;
        s_ci_state.failure_reason = APP_BLE_CI_FAILURE_NONE;
    }
    return s_ci_state.target_reached;
}

bool app_ble_ci_state_reconnect_required(void)
{
    app_ble_ci_state_maintain();
    return s_ci_state.connected &&
           s_ci_state.reconnect_required &&
           (s_ci_state.transport_state == APP_BLE_CI_TRANSPORT_STUCK);
}

bool app_ble_ci_state_terminal_failed(void)
{
    app_ble_ci_state_maintain();
    return s_ci_state.connected &&
           (s_ci_state.transport_state ==
            APP_BLE_CI_TRANSPORT_TERMINAL_FAIL);
}

bool app_ble_ci_state_disconnect_required(void)
{
    app_ble_ci_state_maintain();
    return s_ci_state.connected && s_ci_state.disconnect_required;
}

void app_ble_ci_state_disconnect_requested(void)
{
    s_ci_state.disconnect_required = false;
}

void app_ble_ci_state_require_disconnect(const char *reason)
{
    if (!s_ci_state.connected) return;
    s_ci_state.disconnect_required = true;
    s_ci_state.link_phase = APP_BLE_CI_LINK_FAILED;
    s_ci_state.notify_revision++;
    app_ble_link_trace_note_ci_phase(&s_ci_state);
    ZY100_LOG_WARN("[BLE_CI_HOST] disconnect_required why=%s",
               (reason != NULL) ? reason : "unspecified");
}

bool app_ble_ci_state_take_link_notify(app_ble_ci_state_snapshot_t *snapshot)
{
    if (!app_ble_ci_state_host_session_managed() ||
        (s_ci_state.notify_revision == s_last_link_notify_revision))
    {
        return false;
    }
    s_last_link_notify_revision = s_ci_state.notify_revision;
    if (snapshot != NULL)
    {
        *snapshot = s_ci_state;
    }
    return true;
}

app_ble_ci_state_t app_ble_ci_state_desired(void)
{
    return s_ci_state.desired_state;
}

uint32_t app_ble_ci_state_generation(void)
{
    return s_ci_state.generation;
}

uint32_t app_ble_ci_state_wait_budget_ms(void)
{
    if (s_ci_state.wait_budget_ms == 0U)
    {
        return (uint32_t)ZY100_BLE_EXPORT_FAST_WAIT_TOTAL_TIMEOUT_MS;
    }
    return s_ci_state.wait_budget_ms;
}

void app_ble_ci_state_get_snapshot(app_ble_ci_state_snapshot_t *snapshot)
{
    if (snapshot != NULL)
    {
        *snapshot = s_ci_state;
    }
}
