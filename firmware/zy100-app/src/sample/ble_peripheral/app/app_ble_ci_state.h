#ifndef APP_BLE_CI_STATE_H
#define APP_BLE_CI_STATE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    APP_BLE_CI_STATE_DISCONNECTED = 0,
    APP_BLE_CI_STATE_ACTIVE_IDLE,
    APP_BLE_CI_STATE_CAPTURE_RUNTIME,
    APP_BLE_CI_STATE_ONLINE_HIGH,
    APP_BLE_CI_STATE_STANDBY,
    APP_BLE_CI_STATE_COUNT
} app_ble_ci_state_t;

typedef enum
{
    APP_BLE_CI_TRANSPORT_IDLE = 0,
    APP_BLE_CI_TRANSPORT_SUBMITTED,
    APP_BLE_CI_TRANSPORT_PENDING,
    APP_BLE_CI_TRANSPORT_RETRYING,
    APP_BLE_CI_TRANSPORT_TERMINAL_SUCCESS,
    APP_BLE_CI_TRANSPORT_TERMINAL_FAIL,
    APP_BLE_CI_TRANSPORT_STUCK,
    APP_BLE_CI_TRANSPORT_WAIT_HOST,
} app_ble_ci_transport_state_t;

typedef enum
{
    APP_BLE_CI_RESULT_IDLE = 0,
    APP_BLE_CI_RESULT_SUBMITTED,
    APP_BLE_CI_RESULT_PENDING,
    APP_BLE_CI_RESULT_RETRYING,
    APP_BLE_CI_RESULT_TARGET_REACHED,
    APP_BLE_CI_RESULT_TERMINAL_FAIL,
    APP_BLE_CI_RESULT_STUCK,
    APP_BLE_CI_RESULT_WAIT_HOST,
} app_ble_ci_result_t;

typedef enum
{
    APP_BLE_CI_FAILURE_NONE = 0,
    APP_BLE_CI_FAILURE_SUBMIT,
    APP_BLE_CI_FAILURE_GAP_FAIL,
    APP_BLE_CI_FAILURE_SUCCESS_MISMATCH,
    APP_BLE_CI_FAILURE_RETRY_EXHAUSTED,
    APP_BLE_CI_FAILURE_LLCP_STUCK,
} app_ble_ci_failure_reason_t;

typedef enum
{
    APP_BLE_CI_INITIATOR_PERIPHERAL_EXACT = 0,
    APP_BLE_CI_INITIATOR_WINDOWS_CENTRAL,
    APP_BLE_CI_INITIATOR_ANDROID_CENTRAL,
    APP_BLE_CI_INITIATOR_HOST_REQUIRED,
} app_ble_ci_initiator_mode_t;

typedef enum
{
    APP_BLE_CI_HOST_PROFILE_NONE = 0,
    APP_BLE_CI_HOST_PROFILE_BALANCED = 1,
    APP_BLE_CI_HOST_PROFILE_THROUGHPUT = 2,
    APP_BLE_CI_HOST_PROFILE_POWER = 3,
} app_ble_ci_host_profile_t;

typedef enum
{
    APP_BLE_CI_LINK_IDLE = 0,
    APP_BLE_CI_LINK_INTENT,
    APP_BLE_CI_LINK_WAIT_HOST,
    APP_BLE_CI_LINK_HOST_ACCEPTED,
    APP_BLE_CI_LINK_APPLIED,
    APP_BLE_CI_LINK_FAILED,
    APP_BLE_CI_LINK_TIMEOUT,
} app_ble_ci_link_phase_t;

typedef enum
{
    APP_BLE_CI_HOST_RESULT_REQUEST_ACCEPTED = 1,
    APP_BLE_CI_HOST_RESULT_REQUEST_FAILED = 2,
    APP_BLE_CI_HOST_RESULT_ACTUAL_STABLE = 3,
} app_ble_ci_host_result_t;

typedef struct
{
    bool connected;
    bool target_reached;
    bool request_pending;
    bool gap_pending_seen;
    bool reconnect_required;
    bool disconnect_required;
    bool timeout_logged;
    uint8_t conn_id;
    uint8_t request_result;
    app_ble_ci_state_t source_state;
    app_ble_ci_state_t desired_state;
    app_ble_ci_state_t actual_state;
    app_ble_ci_transport_state_t transport_state;
    app_ble_ci_result_t normalized_result;
    app_ble_ci_failure_reason_t failure_reason;
    app_ble_ci_initiator_mode_t initiator_mode;
    app_ble_ci_host_profile_t expected_profile;
    app_ble_ci_link_phase_t link_phase;
    uint8_t host_request_status;
    uint16_t target_ci;
    uint16_t target_latency;
    uint16_t target_timeout;
    uint16_t actual_ci;
    uint16_t actual_latency;
    uint16_t actual_timeout;
    uint16_t last_gap_cause;
    uint32_t generation;
    uint32_t transition_id;
    uint32_t transition_started_ms;
    uint32_t transport_started_ms;
    uint32_t source_effective_event_ms;
    uint32_t wait_budget_ms;
    uint32_t stuck_budget_ms;
    uint32_t session_id;
    uint32_t lease_deadline_ms;
    uint32_t notify_revision;
} app_ble_ci_state_snapshot_t;

void app_ble_ci_state_init(void);
void app_ble_ci_state_on_connected(uint8_t conn_id,
                                   app_ble_ci_state_t initial_state,
                                   const char *reason);
void app_ble_ci_state_on_disconnected(uint8_t conn_id, const char *reason);
uint8_t app_ble_ci_state_request(app_ble_ci_state_t target,
                                 const char *reason);
void app_ble_ci_state_on_gap_update(uint8_t conn_id,
                                    uint8_t status,
                                    uint16_t cause,
                                    uint16_t actual_ci,
                                    uint16_t actual_latency,
                                    uint16_t actual_timeout);
void app_ble_ci_state_maintain(void);
bool app_ble_ci_state_enable_windows_central(uint8_t conn_id,
                                             uint32_t session_id,
                                             uint32_t windows_build,
                                             uint32_t profile_mask,
                                             uint32_t profile_signature,
                                             bool notify_subscribed);
bool app_ble_ci_state_enable_android_central(uint8_t conn_id,
                                             uint32_t session_id,
                                             uint32_t android_api_level,
                                             uint32_t profile_mask,
                                             uint32_t profile_signature,
                                             bool notify_subscribed);
bool app_ble_ci_state_host_profile_result(uint8_t conn_id,
                                          uint32_t session_id,
                                          uint32_t generation,
                                          uint32_t transition_id,
                                          app_ble_ci_host_profile_t profile,
                                          uint8_t request_status,
                                          app_ble_ci_host_result_t result);
void app_ble_ci_state_note_host_ping(uint8_t conn_id, uint32_t session_id);
void app_ble_ci_state_note_host_transport_ready(uint8_t conn_id);
bool app_ble_ci_state_windows_central_active(void);
bool app_ble_ci_state_android_central_active(void);
bool app_ble_ci_state_host_session_managed(void);
bool app_ble_ci_state_host_standby_settled(void);
bool app_ble_ci_state_peripheral_llcp_allowed(uint16_t ci_min,
                                              uint16_t ci_max,
                                              uint16_t latency,
                                              uint16_t timeout);
bool app_ble_ci_state_begin_ota_host_wait(uint8_t conn_id,
                                          uint32_t wait_ms);
bool app_ble_ci_state_begin_ota_host_lease(uint8_t conn_id,
                                           app_ble_ci_state_t target,
                                           uint32_t lease_ms,
                                           const char *reason);
bool app_ble_ci_state_ota_host_lease_valid(uint8_t conn_id,
                                           app_ble_ci_state_t target);
bool app_ble_ci_state_release_ota_host_lease(uint8_t conn_id,
                                             app_ble_ci_state_t fallback_state,
                                             const char *reason);
void app_ble_ci_state_fail_ota_host_lease(const char *reason);
bool app_ble_ci_state_ota_host_wait_active(void);
void app_ble_ci_state_cancel_ota_host_wait(void);
uint32_t app_ble_ci_state_host_enable_reject_detail(void);
bool app_ble_ci_state_disconnect_required(void);
void app_ble_ci_state_disconnect_requested(void);
void app_ble_ci_state_require_disconnect(const char *reason);
bool app_ble_ci_state_take_link_notify(app_ble_ci_state_snapshot_t *snapshot);

bool app_ble_ci_state_target_reached(app_ble_ci_state_t target);
bool app_ble_ci_state_reconnect_required(void);
bool app_ble_ci_state_terminal_failed(void);
app_ble_ci_state_t app_ble_ci_state_desired(void);
uint32_t app_ble_ci_state_generation(void);
uint32_t app_ble_ci_state_wait_budget_ms(void);
void app_ble_ci_state_get_snapshot(app_ble_ci_state_snapshot_t *snapshot);
const char *app_ble_ci_state_name(app_ble_ci_state_t state);
const char *app_ble_ci_transport_name(app_ble_ci_transport_state_t state);

#endif
