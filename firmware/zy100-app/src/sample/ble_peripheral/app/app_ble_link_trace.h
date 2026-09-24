#ifndef APP_BLE_LINK_TRACE_H
#define APP_BLE_LINK_TRACE_H

#include <stdbool.h>
#include <stdint.h>

#include "../app_flags.h"
#include "app_ble_ci_state.h"

typedef enum
{
    APP_BLE_LINK_TRACE_EVENT_CONNECTED = 1,
    APP_BLE_LINK_TRACE_EVENT_LIVENESS,
    APP_BLE_LINK_TRACE_EVENT_PING_RX,
    APP_BLE_LINK_TRACE_EVENT_PING_ACK_SUBMIT,
    APP_BLE_LINK_TRACE_EVENT_CI_PHASE,
    APP_BLE_LINK_TRACE_EVENT_LOCAL_DISCONNECT,
    APP_BLE_LINK_TRACE_EVENT_GAP_DISCONNECT,
} app_ble_link_trace_event_t;

#if ZY100_BLE_LINK_TRACE_ENABLE

void app_ble_link_trace_on_connected(
    const app_ble_ci_state_snapshot_t *snapshot);
void app_ble_link_trace_note_liveness(
    const app_ble_ci_state_snapshot_t *snapshot);
void app_ble_link_trace_note_ping(
    const app_ble_ci_state_snapshot_t *snapshot,
    uint8_t seq);
void app_ble_link_trace_note_ping_ack(
    const app_ble_ci_state_snapshot_t *snapshot,
    uint8_t seq,
    bool notify_sent,
    uint16_t notify_in_flight);
void app_ble_link_trace_note_ci_phase(
    const app_ble_ci_state_snapshot_t *snapshot);
void app_ble_link_trace_note_local_disconnect(
    const app_ble_ci_state_snapshot_t *snapshot,
    uint16_t submit_cause,
    bool submitted);
void app_ble_link_trace_dump_disconnect(
    const app_ble_ci_state_snapshot_t *snapshot,
    uint16_t disconnect_cause);

#else

#define app_ble_link_trace_on_connected(snapshot) ((void)(snapshot))
#define app_ble_link_trace_note_liveness(snapshot) ((void)(snapshot))
#define app_ble_link_trace_note_ping(snapshot, seq) \
    do { (void)(snapshot); (void)(seq); } while (0)
#define app_ble_link_trace_note_ping_ack(snapshot, seq, notify_sent, in_flight) \
    do { (void)(snapshot); (void)(seq); (void)(notify_sent); \
         (void)(in_flight); } while (0)
#define app_ble_link_trace_note_ci_phase(snapshot) ((void)(snapshot))
#define app_ble_link_trace_note_local_disconnect(snapshot, cause, submitted) \
    do { (void)(snapshot); (void)(cause); (void)(submitted); } while (0)
#define app_ble_link_trace_dump_disconnect(snapshot, cause) \
    do { (void)(snapshot); (void)(cause); } while (0)

#endif

#endif /* APP_BLE_LINK_TRACE_H */
