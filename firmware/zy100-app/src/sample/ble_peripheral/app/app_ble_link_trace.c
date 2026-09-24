#include "app_ble_link_trace.h"

#if ZY100_BLE_LINK_TRACE_ENABLE

#include <limits.h>
#include <stddef.h>
#include <string.h>

#include <os_sched.h>

#include "app_ble_link_trace_internal.h"

#define APP_BLE_LINK_TRACE_LIVENESS_PERIOD_MS 1000UL

#define APP_BLE_LINK_TRACE_FLAG_CONNECTED          (1U << 0)
#define APP_BLE_LINK_TRACE_FLAG_TARGET_REACHED     (1U << 1)
#define APP_BLE_LINK_TRACE_FLAG_REQUEST_PENDING    (1U << 2)
#define APP_BLE_LINK_TRACE_FLAG_DISCONNECT_REQUIRED (1U << 3)
#define APP_BLE_LINK_TRACE_FLAG_STANDBY            (1U << 4)
#define APP_BLE_LINK_TRACE_FLAG_HOST_MANAGED       (1U << 5)
#define APP_BLE_LINK_TRACE_FLAG_RESULT_TRUE        (1U << 6)

typedef struct
{
    app_ble_link_trace_ring_t ring;
    uint32_t last_liveness_ms;
    uint32_t last_ping_ms;
    uint32_t max_liveness_gap_ms;
    uint32_t max_ping_gap_ms;
    uint16_t ping_count;
    uint16_t ping_ack_submit_failures;
    uint16_t local_disconnect_count;
} app_ble_link_trace_state_t;

typedef char app_ble_link_trace_state_must_fit_one_kib[
    (sizeof(app_ble_link_trace_state_t) <= 1024U) ? 1 : -1];

static app_ble_link_trace_state_t s_link_trace;

static uint8_t app_ble_link_trace_flags(
    const app_ble_ci_state_snapshot_t *snapshot)
{
    uint8_t flags = 0U;

    if (snapshot == NULL)
    {
        return flags;
    }
    if (snapshot->connected) flags |= APP_BLE_LINK_TRACE_FLAG_CONNECTED;
    if (snapshot->target_reached) flags |= APP_BLE_LINK_TRACE_FLAG_TARGET_REACHED;
    if (snapshot->request_pending) flags |= APP_BLE_LINK_TRACE_FLAG_REQUEST_PENDING;
    if (snapshot->disconnect_required) flags |= APP_BLE_LINK_TRACE_FLAG_DISCONNECT_REQUIRED;
    if ((snapshot->desired_state == APP_BLE_CI_STATE_STANDBY) ||
        (snapshot->actual_state == APP_BLE_CI_STATE_STANDBY))
    {
        flags |= APP_BLE_LINK_TRACE_FLAG_STANDBY;
    }
    if ((snapshot->initiator_mode == APP_BLE_CI_INITIATOR_WINDOWS_CENTRAL) ||
        (snapshot->initiator_mode == APP_BLE_CI_INITIATOR_ANDROID_CENTRAL))
    {
        flags |= APP_BLE_LINK_TRACE_FLAG_HOST_MANAGED;
    }
    return flags;
}

static uint32_t app_ble_link_trace_state_detail(
    const app_ble_ci_state_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return 0U;
    }
    return ((uint32_t)snapshot->failure_reason) |
           ((uint32_t)snapshot->link_phase << 8) |
           ((uint32_t)snapshot->normalized_result << 16) |
           ((uint32_t)snapshot->actual_state << 24);
}

static void app_ble_link_trace_record(
    app_ble_link_trace_event_t event,
    const app_ble_ci_state_snapshot_t *snapshot,
    uint8_t seq,
    uint32_t detail,
    uint8_t extra_flags,
    uint32_t now_ms)
{
    app_ble_link_trace_entry_t entry;

    memset(&entry, 0, sizeof(entry));
    entry.uptime_ms = now_ms;
    entry.detail = detail;
    entry.event = (uint8_t)event;
    entry.seq = seq;
    entry.flags = extra_flags;
    if (snapshot != NULL)
    {
        entry.generation = (uint16_t)(snapshot->generation & 0xFFFFU);
        entry.transition = (uint16_t)(snapshot->transition_id & 0xFFFFU);
        entry.actual_ci = snapshot->actual_ci;
        entry.actual_latency = snapshot->actual_latency;
        entry.actual_timeout = snapshot->actual_timeout;
        entry.conn_id = snapshot->conn_id;
        entry.flags |= app_ble_link_trace_flags(snapshot);
        entry.desired_state = (uint8_t)snapshot->desired_state;
        entry.transport_state = (uint8_t)snapshot->transport_state;
    }
    app_ble_link_trace_ring_push(&s_link_trace.ring, &entry);
}

void app_ble_link_trace_on_connected(
    const app_ble_ci_state_snapshot_t *snapshot)
{
    uint32_t now_ms = (uint32_t)os_sys_time_get();

    memset(&s_link_trace, 0, sizeof(s_link_trace));
    app_ble_link_trace_ring_reset(&s_link_trace.ring);
    s_link_trace.last_liveness_ms = now_ms;
    app_ble_link_trace_record(APP_BLE_LINK_TRACE_EVENT_CONNECTED,
                              snapshot,
                              0U,
                              app_ble_link_trace_state_detail(snapshot),
                              0U,
                              now_ms);
}

void app_ble_link_trace_note_liveness(
    const app_ble_ci_state_snapshot_t *snapshot)
{
    uint32_t now_ms = (uint32_t)os_sys_time_get();
    uint32_t gap_ms = now_ms - s_link_trace.last_liveness_ms;

    if ((s_link_trace.last_liveness_ms != 0U) &&
        (gap_ms < APP_BLE_LINK_TRACE_LIVENESS_PERIOD_MS))
    {
        return;
    }
    if ((s_link_trace.last_liveness_ms != 0U) &&
        (gap_ms > s_link_trace.max_liveness_gap_ms))
    {
        s_link_trace.max_liveness_gap_ms = gap_ms;
    }
    s_link_trace.last_liveness_ms = now_ms;
    app_ble_link_trace_record(APP_BLE_LINK_TRACE_EVENT_LIVENESS,
                              snapshot,
                              0U,
                              app_ble_link_trace_state_detail(snapshot),
                              0U,
                              now_ms);
}

void app_ble_link_trace_note_ping(
    const app_ble_ci_state_snapshot_t *snapshot,
    uint8_t seq)
{
    uint32_t now_ms = (uint32_t)os_sys_time_get();

    if (s_link_trace.last_ping_ms != 0U)
    {
        uint32_t gap_ms = now_ms - s_link_trace.last_ping_ms;
        if (gap_ms > s_link_trace.max_ping_gap_ms)
        {
            s_link_trace.max_ping_gap_ms = gap_ms;
        }
    }
    s_link_trace.last_ping_ms = now_ms;
    if (s_link_trace.ping_count < UINT16_MAX)
    {
        s_link_trace.ping_count++;
    }
    app_ble_link_trace_record(APP_BLE_LINK_TRACE_EVENT_PING_RX,
                              snapshot,
                              seq,
                              app_ble_link_trace_state_detail(snapshot),
                              0U,
                              now_ms);
}

void app_ble_link_trace_note_ping_ack(
    const app_ble_ci_state_snapshot_t *snapshot,
    uint8_t seq,
    bool notify_sent,
    uint16_t notify_in_flight)
{
    uint32_t detail = (uint32_t)notify_in_flight;

    if (snapshot != NULL)
    {
        detail |= ((uint32_t)snapshot->failure_reason << 16) |
                  ((uint32_t)snapshot->link_phase << 24);
    }

    if (!notify_sent && (s_link_trace.ping_ack_submit_failures < UINT16_MAX))
    {
        s_link_trace.ping_ack_submit_failures++;
    }
    app_ble_link_trace_record(APP_BLE_LINK_TRACE_EVENT_PING_ACK_SUBMIT,
                              snapshot,
                              seq,
                              detail,
                              notify_sent ? APP_BLE_LINK_TRACE_FLAG_RESULT_TRUE : 0U,
                              (uint32_t)os_sys_time_get());
}

void app_ble_link_trace_note_ci_phase(
    const app_ble_ci_state_snapshot_t *snapshot)
{
    app_ble_link_trace_record(APP_BLE_LINK_TRACE_EVENT_CI_PHASE,
                              snapshot,
                              0U,
                              app_ble_link_trace_state_detail(snapshot),
                              0U,
                              (uint32_t)os_sys_time_get());
}

void app_ble_link_trace_note_local_disconnect(
    const app_ble_ci_state_snapshot_t *snapshot,
    uint16_t submit_cause,
    bool submitted)
{
    if (s_link_trace.local_disconnect_count < UINT16_MAX)
    {
        s_link_trace.local_disconnect_count++;
    }
    app_ble_link_trace_record(APP_BLE_LINK_TRACE_EVENT_LOCAL_DISCONNECT,
                              snapshot,
                              0U,
                              (uint32_t)submit_cause,
                              submitted ? APP_BLE_LINK_TRACE_FLAG_RESULT_TRUE : 0U,
                              (uint32_t)os_sys_time_get());
}

void app_ble_link_trace_dump_disconnect(
    const app_ble_ci_state_snapshot_t *snapshot,
    uint16_t disconnect_cause)
{
    uint32_t now_ms = (uint32_t)os_sys_time_get();
    uint32_t ping_age_ms = (s_link_trace.last_ping_ms == 0U) ?
                           UINT32_MAX :
                           (now_ms - s_link_trace.last_ping_ms);
    uint32_t live_age_ms = (s_link_trace.last_liveness_ms == 0U) ?
                           UINT32_MAX :
                           (now_ms - s_link_trace.last_liveness_ms);
    uint8_t i;

    app_ble_link_trace_record(APP_BLE_LINK_TRACE_EVENT_GAP_DISCONNECT,
                              snapshot,
                              0U,
                              (uint32_t)disconnect_cause,
                              0U,
                              now_ms);
    ZY100_LOG_DIRECT("[BLE_FLIGHT] summary cause=0x%04X gen=%lu trans=%lu count=%u ping_count=%u ping_age_ms=%lu ping_gap=%lu live_age_ms=%lu live_gap=%lu ack_fail=%u local_disc=%u",
                     disconnect_cause,
                     (unsigned long)((snapshot != NULL) ? snapshot->generation : 0U),
                     (unsigned long)((snapshot != NULL) ? snapshot->transition_id : 0U),
                     s_link_trace.ring.count,
                     s_link_trace.ping_count,
                     (unsigned long)ping_age_ms,
                     (unsigned long)s_link_trace.max_ping_gap_ms,
                     (unsigned long)live_age_ms,
                     (unsigned long)s_link_trace.max_liveness_gap_ms,
                     s_link_trace.ping_ack_submit_failures,
                     s_link_trace.local_disconnect_count);

    for (i = 0U; i < s_link_trace.ring.count; i++)
    {
        const app_ble_link_trace_entry_t *entry =
            app_ble_link_trace_ring_get(&s_link_trace.ring, i);
        if (entry == NULL)
        {
            continue;
        }
        ZY100_LOG_DIRECT("[BLE_FLIGHT] i=%u t=%lu ev=%u c=%u gen=%u trans=%u seq=%u flags=0x%02X want=%u tx=%u ci=%u lat=%u tout=%u detail=0x%08lX",
                         i,
                         (unsigned long)entry->uptime_ms,
                         entry->event,
                         entry->conn_id,
                         entry->generation,
                         entry->transition,
                         entry->seq,
                         entry->flags,
                         entry->desired_state,
                         entry->transport_state,
                         entry->actual_ci,
                         entry->actual_latency,
                         entry->actual_timeout,
                         (unsigned long)entry->detail);
    }
}

#endif /* ZY100_BLE_LINK_TRACE_ENABLE */
