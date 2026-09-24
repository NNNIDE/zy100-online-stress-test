#include "app/app_device_pairing.h"
#include "app/app_user_reset.h"

#include <stddef.h>
#include <string.h>

#include <os_sync.h>
#include <trace.h>
#include <gap_bond_le.h>
#include <gap_storage_le.h>
#include <gap_conn_le.h>
#include "zy100_clock_config.h"

#include "app_flags.h"
#include "app_factory/factory_boot_gate.h"
#include "peripheral_app.h"
#include "app/app_ble_power_policy.h"
#include "service/zy100_system_info_store.h"

#define ZY100_PAIRING_BDADDR_RAW_FMT "%02X:%02X:%02X:%02X:%02X:%02X"
#define ZY100_PAIRING_BDADDR_RAW_ARG(a) \
    (unsigned int)((a)[0]), (unsigned int)((a)[1]), \
    (unsigned int)((a)[2]), (unsigned int)((a)[3]), \
    (unsigned int)((a)[4]), (unsigned int)((a)[5])

typedef struct
{
    bool active;
    bool auth_started;
    bool auth_success;
    bool identity_valid;
    bool ready_committed;
    bool had_persisted_peer_at_start;
    bool confirm_pending_seen;
    bool bond_add_seen;
    bool wait_bond_entry_logged;
    uint8_t conn_id;
    uint8_t peer_addr[APP_PAIRING_PEER_ADDR_BYTES];
    uint8_t peer_type;
} app_device_pairing_session_t;

static app_task_pairing_pending_t
    s_pairing_event_queue[APP_PAIRING_EVENT_QUEUE_DEPTH];
static uint8_t s_pairing_event_queue_head;
static uint8_t s_pairing_event_queue_tail;
static uint8_t s_pairing_event_queue_count;
static app_device_pairing_session_t s_pairing_session;

typedef enum
{
    PAIR_WINDOW_OFF = 0,
    PAIR_WINDOW_DRAIN,
    PAIR_WINDOW_START,
    PAIR_WINDOW_ACTIVE
} pairing_window_state_t;
static pairing_window_state_t s_window_state;
static uint32_t s_window_generation;
static uint32_t s_connection_generation;
static uint32_t s_window_deadline;
static bool s_window_failed;
static bool s_window_evicted;
static bool s_window_eviction_pending;
static uint8_t s_window_victim_addr[6];
static uint8_t s_window_victim_type;
static uint8_t s_window_attempt_conn = APP_PAIRING_CONN_ID_INVALID;

void app_device_pairing_note_connected(void)
{
    ++s_connection_generation;
    s_window_evicted = false;
    s_window_eviction_pending = false;
    s_window_attempt_conn = APP_PAIRING_CONN_ID_INVALID;
    memset(&s_pairing_session, 0, sizeof(s_pairing_session));
    s_pairing_session.conn_id = APP_PAIRING_CONN_ID_INVALID;
}

bool app_device_pairing_window_active(void)
{
    return s_window_state != PAIR_WINDOW_OFF;
}

bool app_device_pairing_window_waiting(void)
{
    return s_window_state == PAIR_WINDOW_DRAIN;
}

void app_device_pairing_window_request(void)
{
    if (app_device_pairing_window_active()) return;
    ++s_window_generation;
    s_window_state = PAIR_WINDOW_DRAIN;
    s_window_deadline = 0U;
    s_window_failed = false;
    s_window_evicted = false;
    s_window_attempt_conn = APP_PAIRING_CONN_ID_INVALID;
    DBG_DIRECT("[PAIR_WIN] request gen=%lu", (unsigned long)s_window_generation);
}

void app_device_pairing_window_cancel(void)
{
    if (app_device_pairing_window_active())
    {
        ++s_window_generation;
        app_device_pairing_port_window_led(false);
    }
    s_window_state = PAIR_WINDOW_OFF;
    s_window_deadline = 0U;
    s_window_failed = false;
}

bool app_device_pairing_window_start_on_boot(void)
{
    if (!app_device_pairing_port_boot_window_ready() ||
        app_ble_power_shutdown_latched()) return false;
    if (app_device_pairing_window_active())
        return s_window_state == PAIR_WINDOW_START || s_window_state == PAIR_WINDOW_ACTIVE;
    app_device_pairing_window_request();
    s_window_state = PAIR_WINDOW_START;
    s_window_deadline = zy100_os_time_ms() + 30000U;
    /* Arm before any advertising callback, without a redundant shutdown cycle. */
    app_ble_power_on_wake();
    return true;
}

void app_device_pairing_window_adv_result(bool active, bool failed)
{
    if (s_window_state == PAIR_WINDOW_START && active)
    {
        s_window_state = PAIR_WINDOW_ACTIVE;
        s_window_deadline = zy100_os_time_ms() + 30000U;
        app_device_pairing_port_window_led(true);
        DBG_DIRECT("[PAIR_WIN] advertising gen=%lu timeout_ms=30000",
                   (unsigned long)s_window_generation);
    }
    if (failed && app_device_pairing_window_active()) s_window_failed = true;
}

void app_device_pairing_window_poll(uint64_t now_ms)
{
    if (s_window_state == PAIR_WINDOW_DRAIN)
    {
        if (!app_device_pairing_port_shutdown_ready()) return;
        s_window_state = PAIR_WINDOW_START;
        /* Bound initialization/advertising startup independently of the window. */
        s_window_deadline = now_ms + 30000U;
        if (!app_device_pairing_port_window_start()) s_window_failed = true;
    }
    if (app_device_pairing_window_active() &&
        (s_window_failed || ((s_window_state != PAIR_WINDOW_DRAIN) &&
         (int32_t)((uint32_t)now_ms - s_window_deadline) >= 0)))
    {
        DBG_DIRECT("[PAIR_WIN] expire gen=%lu failed=%u",
                   (unsigned long)s_window_generation, s_window_failed ? 1U : 0U);
        const bool start_failed = s_window_failed || s_window_state == PAIR_WINDOW_START;
        app_device_pairing_window_cancel();
        app_device_pairing_port_window_expire(start_failed);
    }
}

static T_LE_KEY_ENTRY *app_pairing_connection_key(uint8_t conn_id)
{
    uint8_t addr[6];
    uint8_t type;
    T_LE_KEY_ENTRY *entry;
    if (!app_ble_power_conn_valid(conn_id) ||
        !le_get_conn_addr(conn_id, addr, &type)) return NULL;
    entry = le_find_key_entry(addr, (T_GAP_REMOTE_ADDR_TYPE)type);
#if F_BT_LE_PRIVACY_SUPPORT
    if (entry == NULL && type == GAP_REMOTE_ADDR_LE_RANDOM)
    {
        uint8_t resolved[6];
        T_GAP_IDENT_ADDR_TYPE resolved_type;
        if (le_resolve_random_address(addr, resolved, &resolved_type))
            entry = le_find_key_entry(resolved, (T_GAP_REMOTE_ADDR_TYPE)resolved_type);
    }
#endif
    return entry;
}

bool app_device_pairing_admit_new_bond(uint8_t conn_id)
{
    T_LE_KEY_ENTRY *entry;
    uint8_t victim_addr[6];
    uint8_t victim_type;
    uint8_t victim_idx;
    if (!app_device_pairing_window_active()) return !app_ble_power_shutdown_latched();
    if (!app_ble_power_conn_valid(conn_id) ||
        s_window_state != PAIR_WINDOW_ACTIVE || s_window_failed ||
        (int32_t)(zy100_os_time_ms() - s_window_deadline) >= 0) return false;
    if (app_pairing_connection_key(conn_id) != NULL) return true;
    if (le_get_bond_dev_num() < le_get_max_le_paired_device_num()) return true;
    if (s_window_evicted) return false;
    entry = le_get_low_priority_bond();
    if (entry == NULL) return false;
    victim_idx = entry->idx;
    if (entry->flags & LE_KEY_STORE_REMOTE_IRK_BIT)
    {
        memcpy(victim_addr, entry->resolved_remote_bd.addr, 6U);
        victim_type = entry->resolved_remote_bd.remote_bd_type;
    }
    else
    {
        memcpy(victim_addr, entry->remote_bd.addr, 6U);
        victim_type = entry->remote_bd.remote_bd_type;
    }
    s_window_evicted = true;
    s_window_attempt_conn = conn_id;
    memcpy(s_window_victim_addr, victim_addr, 6U);
    s_window_victim_type = victim_type;
    s_window_eviction_pending = true;
    if (le_bond_delete_by_idx(victim_idx) != GAP_CAUSE_SUCCESS ||
        !zy100_system_info_remove_paired_peer(victim_addr, victim_type))
    {
        DBG_DIRECT("[PAIR_WIN] evict_failed slot=%u", victim_idx);
        s_window_failed = true;
        return false;
    }
    DBG_DIRECT("[PAIR_WIN] evict gen=%lu slot=%u conn=%u",
               (unsigned long)s_window_generation, victim_idx, conn_id);
    return true;
}

void app_task_pairing_post_event(app_task_pairing_event_t event,
                                 uint8_t conn_id,
                                 const uint8_t peer_addr[6],
                                 uint8_t peer_type,
                                 const char *reason)
{
    app_task_pairing_pending_t pending;
    uint32_t lock_state;
    uint32_t dropped_event = 0U;
    bool enqueued = false;
    bool force_terminal_log = false;
    bool drop_newest_log = false;
    bool terminal_event;

    if (factory_boot_gate_factory_mode_active())
    {
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[PAIR_EVT] ignore_factory event=%u conn=%u reason=%s",
                   (uint32_t)event,
                   conn_id,
                   (reason != NULL) ? reason : "unknown");
        return;
    }

    if (app_ble_power_shutdown_latched())
    {
        return;
    }
    memset(&pending, 0, sizeof(pending));
    pending.event = event;
    pending.window_generation = s_window_generation;
    pending.connection_generation = s_connection_generation;
    pending.conn_id = conn_id;
    if (peer_addr != NULL)
    {
        memcpy(pending.peer_addr,
               peer_addr,
               sizeof(pending.peer_addr));
        pending.peer_addr_valid = 1U;
    }
    pending.peer_type = peer_type;
    if (reason != NULL)
    {
        strncpy(pending.reason,
                reason,
                sizeof(pending.reason) - 1U);
        pending.reason[sizeof(pending.reason) - 1U] = '\0';
    }
    app_device_pairing_port_note_activity((reason != NULL) ? reason :
                                          "pairing_event");

    terminal_event =
        (event == APP_TASK_PAIRING_EVENT_DISCONNECTED) ||
        (event == APP_TASK_PAIRING_EVENT_AUTH_FAILED) ||
        (event == APP_TASK_PAIRING_EVENT_BOND_CLEAR) ||
        (event == APP_TASK_PAIRING_EVENT_BOND_DELETE) ||
        (event == APP_TASK_PAIRING_EVENT_BOND_KEY_MISSING);

    lock_state = os_lock();
    if (s_pairing_event_queue_count >= APP_PAIRING_EVENT_QUEUE_DEPTH)
    {
        if (terminal_event)
        {
            uint8_t offset;
            uint8_t drop_offset = 0U;
            bool found_non_terminal = false;

            for (offset = 0U; offset < s_pairing_event_queue_count; offset++)
            {
                uint8_t idx = (uint8_t)((s_pairing_event_queue_head + offset) %
                                        APP_PAIRING_EVENT_QUEUE_DEPTH);
                app_task_pairing_event_t queued_event =
                    s_pairing_event_queue[idx].event;

                if ((queued_event != APP_TASK_PAIRING_EVENT_DISCONNECTED) &&
                    (queued_event != APP_TASK_PAIRING_EVENT_AUTH_FAILED) &&
                    (queued_event != APP_TASK_PAIRING_EVENT_BOND_CLEAR) &&
                    (queued_event != APP_TASK_PAIRING_EVENT_BOND_DELETE) &&
                    (queued_event != APP_TASK_PAIRING_EVENT_BOND_KEY_MISSING))
                {
                    drop_offset = offset;
                    found_non_terminal = true;
                    break;
                }
            }

            if (!found_non_terminal)
            {
                drop_offset = 0U;
            }

            {
                uint8_t idx = (uint8_t)((s_pairing_event_queue_head + drop_offset) %
                                        APP_PAIRING_EVENT_QUEUE_DEPTH);
                uint8_t move_offset;

                dropped_event = (uint32_t)s_pairing_event_queue[idx].event;
                for (move_offset = drop_offset;
                     (move_offset + 1U) < s_pairing_event_queue_count;
                     move_offset++)
                {
                    uint8_t cur_idx =
                        (uint8_t)((s_pairing_event_queue_head + move_offset) %
                                  APP_PAIRING_EVENT_QUEUE_DEPTH);
                    uint8_t next_idx =
                        (uint8_t)((s_pairing_event_queue_head + move_offset + 1U) %
                                  APP_PAIRING_EVENT_QUEUE_DEPTH);
                    s_pairing_event_queue[cur_idx] =
                        s_pairing_event_queue[next_idx];
                }
                s_pairing_event_queue_tail =
                    (uint8_t)((s_pairing_event_queue_head +
                               s_pairing_event_queue_count - 1U) %
                              APP_PAIRING_EVENT_QUEUE_DEPTH);
                s_pairing_event_queue_count--;
            }
            force_terminal_log = true;
        }
        else
        {
            dropped_event =
                (uint32_t)s_pairing_event_queue[s_pairing_event_queue_head].event;
            drop_newest_log = true;
        }
    }

    if (s_pairing_event_queue_count < APP_PAIRING_EVENT_QUEUE_DEPTH)
    {
        s_pairing_event_queue[s_pairing_event_queue_tail] = pending;
        s_pairing_event_queue_tail =
            (uint8_t)((s_pairing_event_queue_tail + 1U) %
                      APP_PAIRING_EVENT_QUEUE_DEPTH);
        s_pairing_event_queue_count++;
        enqueued = true;
    }
    os_unlock(lock_state);

    if (force_terminal_log)
    {
        DBG_DIRECT("[PAIR_EVT] queue_full force_terminal drop=%u new=%u",
                   dropped_event,
                   (uint32_t)event);
    }
    else if (drop_newest_log)
    {
        DBG_DIRECT("[PAIR_EVT] queue_full drop_newest event=%u new=%u",
                   dropped_event,
                   (uint32_t)event);
    }

    if (!enqueued)
    {
        return;
    }

    if (!app_device_pairing_port_post_event())
    {
        DBG_DIRECT("[PAIR_EVT] post_failed event=%u reason=%s",
                   (uint32_t)event,
                   (reason != NULL) ? reason : "unknown");
    }
}

bool app_task_pairing_take_event(app_task_pairing_pending_t *out)
{
    uint32_t lock_state;
    bool taken = false;

    if (out == NULL)
    {
        return false;
    }

    lock_state = os_lock();
    if (s_pairing_event_queue_count != 0U)
    {
        *out = s_pairing_event_queue[s_pairing_event_queue_head];
        s_pairing_event_queue_head =
            (uint8_t)((s_pairing_event_queue_head + 1U) %
                      APP_PAIRING_EVENT_QUEUE_DEPTH);
        s_pairing_event_queue_count--;
        if (s_pairing_event_queue_count == 0U)
        {
            s_pairing_event_queue_tail = s_pairing_event_queue_head;
        }
        taken = true;
    }
    os_unlock(lock_state);
    return taken;
}

bool app_task_pairing_queue_has_pending(void)
{
    uint32_t lock_state;
    bool has_pending;

    lock_state = os_lock();
    has_pending = (s_pairing_event_queue_count != 0U);
    os_unlock(lock_state);
    return has_pending;
}


static void app_task_pairing_session_reset(void)
{
    memset(&s_pairing_session, 0, sizeof(s_pairing_session));
    s_pairing_session.conn_id = APP_PAIRING_CONN_ID_INVALID;
}

static void app_task_pairing_session_begin(uint8_t conn_id,
                                           const char *reason)
{
    zy100_system_info_t info;

    app_task_pairing_session_reset();
    s_pairing_session.active = true;
    s_pairing_session.auth_started = true;
    s_pairing_session.conn_id = conn_id;

    memset(&info, 0, sizeof(info));
    if (zy100_system_info_load(&info))
    {
        s_pairing_session.had_persisted_peer_at_start =
            ((info.paired_peer_count != 0U) || (info.pairing_bonded != 0U));
    }
    else
    {
        DBG_DIRECT("[PAIR_EVT] auth_start_load_failed conn=%u reason=%s",
                   conn_id,
                   (reason != NULL) ? reason : "auth_started");
    }
}

static void app_task_pairing_session_ensure(uint8_t conn_id,
                                            const char *reason)
{
    if ((!s_pairing_session.active) ||
        (s_pairing_session.conn_id != conn_id))
    {
        app_task_pairing_session_begin(conn_id, reason);
    }
}

static void app_task_pairing_abort_runtime_keep_flash(const char *reason)
{
    app_device_pairing_port_abort_runtime_keep_flash(
        (reason != NULL) ? reason : "pairing_lost");
}

static void app_task_pairing_handle_bond_clear(const char *reason)
{
    const char *use_reason = (reason != NULL) ? reason : "bond_clear";

    (void)app_device_pairing_port_gate_load(use_reason, false);
    if (!zy100_system_info_clear_pairing())
    {
        DBG_DIRECT("[PAIR_EVT] clear_pairing_failed reason=%s", use_reason);
    }
    app_device_pairing_port_set_bonded(false);
    app_task_pairing_session_reset();
    app_task_pairing_abort_runtime_keep_flash(use_reason);

    if (app_device_pairing_port_first_power_seen())
    {
        app_device_pairing_port_wait_begin(use_reason);
    }
    else
    {
        DBG_DIRECT("[PAIR_EVT] lost_during_first_init keep_state=%s",
                   app_device_pairing_port_gate_state_name());
    }

    DBG_DIRECT("[PAIR_EVT] pairing_lost reason=%s first_power_seen=%u keep_flash=1",
               use_reason,
               app_device_pairing_port_first_power_seen() ? 1U : 0U);
}

static bool app_task_pairing_event_has_peer_addr(
    const app_task_pairing_pending_t *event)
{
    return (event != NULL) && (event->peer_addr_valid != 0U);
}

static bool app_task_pairing_store_bond_identity(
    const uint8_t peer_addr[APP_PAIRING_PEER_ADDR_BYTES],
    uint8_t peer_type,
    bool used_irk,
    const char *reason)
{
    const char *use_reason = (reason != NULL) ? reason : "bond_identity";
    zy100_system_info_t info;
    uint8_t count = 0U;

    if (peer_addr == NULL)
    {
        return false;
    }

    if (!zy100_system_info_add_paired_peer(peer_addr, peer_type))
    {
        DBG_DIRECT("[PAIR_EVT] bond_identity_persist_failed conn=%u type=%u reason=%s",
                   s_pairing_session.conn_id,
                   peer_type,
                   use_reason);
        return false;
    }

    s_pairing_session.identity_valid = true;
    memcpy(s_pairing_session.peer_addr,
           peer_addr,
           sizeof(s_pairing_session.peer_addr));
    s_pairing_session.peer_type = peer_type;

    memset(&info, 0, sizeof(info));
    if (zy100_system_info_load(&info))
    {
        count = info.paired_peer_count;
    }

    ZY100_LOG_DETAIL("[PAIR_EVT] bond_identity conn=%u reason=%s type=%u addr="
               ZY100_PAIRING_BDADDR_RAW_FMT " irk=%u count=%u",
               s_pairing_session.conn_id,
               use_reason,
               peer_type,
               ZY100_PAIRING_BDADDR_RAW_ARG(peer_addr),
               used_irk ? 1U : 0U,
               count);
    return true;
}

static bool app_task_pairing_try_resolve_bond_identity(const char *reason)
{
    const char *use_reason = (reason != NULL) ? reason : "bond_identity";
    T_LE_KEY_ENTRY *entry;
    const T_LE_REMOTE_BD *identity;

    if (s_pairing_session.identity_valid)
    {
        return true;
    }

    entry = app_pairing_connection_key(s_pairing_session.conn_id);
    if (entry == NULL)
    {
        ZY100_LOG_DETAIL("[PAIR_EVT] bond_identity_pending conn=%u reason=%s",
                   s_pairing_session.conn_id,
                   use_reason);
        return false;
    }

    identity = (entry->flags & LE_KEY_STORE_REMOTE_IRK_BIT) ?
        &entry->resolved_remote_bd : &entry->remote_bd;
    return app_task_pairing_store_bond_identity(identity->addr,
        identity->remote_bd_type, (entry->flags & LE_KEY_STORE_REMOTE_IRK_BIT) != 0U,
        use_reason);
}

static void app_task_pairing_handle_disconnected(
    const app_task_pairing_pending_t *event)
{
    uint8_t conn_id = (event != NULL) ? event->conn_id : APP_PAIRING_CONN_ID_INVALID;

    if (s_window_attempt_conn == conn_id)
    {
        s_window_attempt_conn = APP_PAIRING_CONN_ID_INVALID;
        s_window_evicted = false;
    }

    if ((!s_pairing_session.active) ||
        (s_pairing_session.conn_id == conn_id) ||
        (conn_id == APP_PAIRING_CONN_ID_INVALID))
    {
        app_task_pairing_session_reset();
    }
    ZY100_LOG_DETAIL("[PAIR_EVT] disconnected conn=%u session_clear=1", conn_id);
}

static void app_task_pairing_handle_auth_failed(
    const app_task_pairing_pending_t *event)
{
    const char *use_reason =
        ((event != NULL) && (event->reason[0] != '\0')) ?
        event->reason : "auth_failed";

    (void)app_device_pairing_port_gate_load(use_reason, false);
    app_task_pairing_session_reset();
    if (app_device_pairing_port_bonded())
    {
        if (app_device_pairing_port_first_power_seen())
        {
            app_device_pairing_port_set_ready(use_reason);
        }
        DBG_DIRECT("[PAIR_EVT] auth_failed keep_existing=1 reason=%s",
                   use_reason);
        return;
    }

    app_device_pairing_port_wait_begin(use_reason);
    DBG_DIRECT("[PAIR_EVT] auth_failed no_existing_peer reason=%s",
               use_reason);
}

static void app_task_pairing_try_commit_ready(const char *reason)
{
    const char *use_reason = (reason != NULL) ? reason : "pairing_ready";
    zy100_system_info_t info;
    uint8_t count = 0U;
    bool persisted_reconnect_ok;
    T_LE_KEY_ENTRY *entry;

    if (app_device_pairing_window_active() &&
        (s_window_failed || s_window_state != PAIR_WINDOW_ACTIVE ||
         (int32_t)(zy100_os_time_ms() - s_window_deadline) >= 0)) return;
    if (!app_ble_power_conn_valid(s_pairing_session.conn_id)) return;

    if ((!s_pairing_session.active) ||
        s_pairing_session.ready_committed ||
        (!s_pairing_session.auth_success))
    {
        return;
    }

    persisted_reconnect_ok =
        s_pairing_session.had_persisted_peer_at_start &&
        (!s_pairing_session.confirm_pending_seen);

    entry = app_pairing_connection_key(s_pairing_session.conn_id);
    if (entry == NULL) return;
    if (!s_pairing_session.identity_valid)
    {
        const T_LE_REMOTE_BD *identity = (entry->flags & LE_KEY_STORE_REMOTE_IRK_BIT) ?
            &entry->resolved_remote_bd : &entry->remote_bd;
        if (!app_task_pairing_store_bond_identity(identity->addr,
            identity->remote_bd_type, (entry->flags & LE_KEY_STORE_REMOTE_IRK_BIT) != 0U,
            "authenticated_identity")) return;
    }
    if (!le_set_high_priority_bond(entry->remote_bd.addr,
            (T_GAP_REMOTE_ADDR_TYPE)entry->remote_bd.remote_bd_type))
    {
        DBG_DIRECT("[PAIR_EVT] priority_failed conn=%u", s_pairing_session.conn_id);
        if (app_device_pairing_window_active()) s_window_failed = true;
        return;
    }

    if ((!s_pairing_session.identity_valid) && (!persisted_reconnect_ok))
    {
        if (!s_pairing_session.wait_bond_entry_logged)
        {
            ZY100_DIAG_LOG("[PAIR_EVT] auth_success_wait_bond_entry conn=%u persisted=%u confirm=%u",
                       s_pairing_session.conn_id,
                       s_pairing_session.had_persisted_peer_at_start ? 1U : 0U,
                       s_pairing_session.confirm_pending_seen ? 1U : 0U);
            s_pairing_session.wait_bond_entry_logged = true;
        }
        return;
    }

    if (!app_device_pairing_port_gate_load(use_reason, true))
    {
        return;
    }

    memset(&info, 0, sizeof(info));
    if (zy100_system_info_load(&info))
    {
        count = info.paired_peer_count;
    }

    /* A synchronous save may cross the window deadline. */
    if (app_device_pairing_window_active() &&
        (int32_t)(zy100_os_time_ms() - s_window_deadline) >= 0) return;
    /* Authentication and bond persistence won the deadline decision above.
     * Retire the reset onboarding marker in this same serialized commit. */
    if (!app_user_reset_pairing_complete()) return;
    if (app_device_pairing_window_active())
    {
        DBG_DIRECT("[PAIR_WIN] success gen=%lu conn=%u",
                   (unsigned long)s_window_generation, s_pairing_session.conn_id);
        /* Install the success owner before releasing the window LED. */
        app_device_pairing_port_window_success_led(use_reason);
        app_device_pairing_window_cancel();
        persisted_reconnect_ok = true;
    }
    s_pairing_session.ready_committed = true;
    app_device_pairing_port_set_bonded(true);
    if (app_device_pairing_port_first_power_seen())
    {
        if (persisted_reconnect_ok)
        {
            app_device_pairing_port_on_reconnect_ready(use_reason);
            ZY100_LOG_DETAIL("[PAIR_EVT] ready_commit_reconnect_no_hold conn=%u reason=%s",
                       s_pairing_session.conn_id,
                       use_reason);
        }
        else
        {
            app_device_pairing_port_success_hold_begin(use_reason);
        }
    }
    else
    {
        ZY100_LOG_DETAIL("[PAIR_EVT] ready_commit_wait_first_init state=%s",
                   app_device_pairing_port_gate_state_name());
    }

#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
    app_ble_pairing_mark_ready(s_pairing_session.conn_id, use_reason);
#endif
    app_device_pairing_port_try_auto_export("pairing_ready");
    ZY100_LOG_DETAIL("[PAIR_EVT] ready_commit conn=%u reason=%s identity=%u persisted_start=%u confirm_pending=%u bond_event=%u persisted_reconnect=%u count=%u",
               s_pairing_session.conn_id,
               use_reason,
               s_pairing_session.identity_valid ? 1U : 0U,
               s_pairing_session.had_persisted_peer_at_start ? 1U : 0U,
               s_pairing_session.confirm_pending_seen ? 1U : 0U,
               s_pairing_session.bond_add_seen ? 1U : 0U,
               persisted_reconnect_ok ? 1U : 0U,
               count);
}

static void app_task_pairing_handle_success(
    const app_task_pairing_pending_t *event)
{
    const char *reason;

    if (event == NULL)
    {
        return;
    }

    reason = (event->reason[0] != '\0') ? event->reason : "auth_success";
    app_task_pairing_session_ensure(event->conn_id, reason);
    s_pairing_session.auth_success = true;
    ZY100_DIAG_LOG("[PAIR_EVT] auth_success conn=%u persisted_start=%u confirm=%u",
               event->conn_id,
               s_pairing_session.had_persisted_peer_at_start ? 1U : 0U,
               s_pairing_session.confirm_pending_seen ? 1U : 0U);
    if (s_pairing_session.bond_add_seen &&
        (!s_pairing_session.identity_valid))
    {
        (void)app_task_pairing_try_resolve_bond_identity("auth_success");
    }
    app_task_pairing_try_commit_ready(reason);
}

static void app_task_pairing_handle_bond_add(
    const app_task_pairing_pending_t *event)
{
    const char *reason;

    if (event == NULL)
    {
        return;
    }

    reason = (event->reason[0] != '\0') ? event->reason : "bond_add";
    app_task_pairing_session_ensure(event->conn_id, reason);
    s_pairing_session.bond_add_seen = true;

    if (app_task_pairing_event_has_peer_addr(event))
    {
        if (!app_task_pairing_store_bond_identity(event->peer_addr,
                                                  event->peer_type,
                                                  false,
                                                  "bond_add"))
        {
            return;
        }
    }
    else if (!app_task_pairing_try_resolve_bond_identity("bond_add"))
    {
        return;
    }

    app_task_pairing_try_commit_ready(reason);
}

static void app_task_pairing_handle_bond_remove(
    const app_task_pairing_pending_t *event)
{
    const char *reason;
    zy100_system_info_t info;

    if (event == NULL)
    {
        return;
    }

    reason = (event->reason[0] != '\0') ? event->reason : "bond_remove";
    if (!app_task_pairing_event_has_peer_addr(event))
    {
        ZY100_DIAG_LOG("[PAIR_EVT] bond_remove_skip_no_identity event=%u",
                   (uint32_t)event->event);
        app_task_pairing_session_reset();
        return;
    }

    if (!zy100_system_info_remove_paired_peer(event->peer_addr,
                                              event->peer_type))
    {
        DBG_DIRECT("[PAIR_EVT] bond_remove_failed conn=%u type=%u event=%u",
                   event->conn_id,
                   event->peer_type,
                   (uint32_t)event->event);
        return;
    }
    app_task_pairing_session_reset();
    if (!zy100_system_info_load(&info))
    {
        app_device_pairing_port_set_error("bond_remove_reload_failed");
        return;
    }

    if (info.paired_peer_count == 0U)
    {
        (void)app_device_pairing_port_gate_load(reason, true);
        app_device_pairing_port_set_bonded(false);
        app_task_pairing_abort_runtime_keep_flash(reason);
        if (app_device_pairing_port_first_power_seen())
        {
            app_device_pairing_port_wait_begin(reason);
        }
        ZY100_DIAG_LOG("[PAIR_EVT] bond_remove conn=%u remain=0 type=%u event=%u",
                   event->conn_id,
                   event->peer_type,
                   (uint32_t)event->event);
        return;
    }

    (void)app_device_pairing_port_gate_load(reason, true);
    app_device_pairing_port_set_bonded(true);
    ZY100_DIAG_LOG("[PAIR_EVT] bond_remove conn=%u remain=%u type=%u event=%u",
               event->conn_id,
               info.paired_peer_count,
               event->peer_type,
               (uint32_t)event->event);
}

static void app_task_pairing_handle_bond_full(
    const app_task_pairing_pending_t *event)
{
    uint8_t conn_id = (event != NULL) ? event->conn_id : APP_PAIRING_CONN_ID_INVALID;

    DBG_DIRECT("[PAIR_EVT] bond_full conn=%u", conn_id);
    /* Do not assume the ROM retries a failed key save after LE_BOND_FULL. */
    if (app_device_pairing_window_active()) s_window_failed = true;
}

void app_device_pairing_shutdown(void)
{
    uint32_t flags = os_lock();
    s_pairing_event_queue_head = 0U;
    s_pairing_event_queue_tail = 0U;
    s_pairing_event_queue_count = 0U;
    os_unlock(flags);
    app_task_pairing_session_reset();
}

void app_device_pairing_handle_pending_event(void)
{
    app_task_pairing_pending_t event;
    const char *reason;

    if (app_ble_power_shutdown_latched())
    {
        app_device_pairing_shutdown();
        return;
    }
    memset(&event, 0, sizeof(event));
    if (!app_task_pairing_take_event(&event))
    {
        return;
    }

    if (event.window_generation != s_window_generation ||
        event.connection_generation != s_connection_generation) return;
    /* Eviction was mirrored before accepting Just Works. Its queued deletion
     * must not reset the new pairing session. */
    if (event.event == APP_TASK_PAIRING_EVENT_BOND_DELETE &&
        s_window_eviction_pending && event.peer_addr_valid &&
        event.peer_type == s_window_victim_type &&
        memcmp(event.peer_addr, s_window_victim_addr, 6U) == 0)
    {
        s_window_eviction_pending = false;
        return;
    }
    reason = (event.reason[0] != '\0') ? event.reason : "pairing_event";
    switch (event.event)
    {
    case APP_TASK_PAIRING_EVENT_AUTH_STARTED:
        (void)app_device_pairing_port_gate_load(reason, false);
        app_task_pairing_session_begin(event.conn_id, reason);
        ZY100_DIAG_LOG("[PAIR_EVT] auth_started conn=%u pairing_bonded=%u",
                   event.conn_id,
                   app_device_pairing_port_bonded() ? 1U : 0U);
        break;

    case APP_TASK_PAIRING_EVENT_CONFIRM_PENDING:
        app_task_pairing_session_ensure(event.conn_id, reason);
        s_pairing_session.confirm_pending_seen = true;
        if (!app_device_pairing_port_prepare_confirm(reason))
        {
            DBG_DIRECT("[PAIR_EVT][WARN] confirm_standby_restore_failed conn=%u reason=%s",
                       event.conn_id,
                       reason);
            break;
        }
        (void)app_device_pairing_port_gate_load(reason, false);
        if (app_device_pairing_port_bonded())
        {
            ZY100_DIAG_LOG("[PAIR_EVT] confirm_pending keep_existing=1 conn=%u",
                       event.conn_id);
        }
        app_device_pairing_port_confirm_begin(reason);
        ZY100_DIAG_LOG("[PAIR_EVT] confirm_pending conn=%u reason=%s",
                   event.conn_id,
                   reason);
        break;

    case APP_TASK_PAIRING_EVENT_AUTH_SUCCESS:
        app_task_pairing_handle_success(&event);
        break;

    case APP_TASK_PAIRING_EVENT_AUTH_FAILED:
        if (app_device_pairing_window_active())
        {
            app_task_pairing_session_reset();
            (void)le_disconnect(event.conn_id);
            break;
        }
        app_task_pairing_handle_auth_failed(&event);
        break;

    case APP_TASK_PAIRING_EVENT_BOND_ADD:
        app_task_pairing_handle_bond_add(&event);
        break;

    case APP_TASK_PAIRING_EVENT_BOND_DELETE:
    case APP_TASK_PAIRING_EVENT_BOND_KEY_MISSING:
        app_task_pairing_handle_bond_remove(&event);
        break;

    case APP_TASK_PAIRING_EVENT_BOND_CLEAR:
        app_task_pairing_handle_bond_clear(reason);
        break;

    case APP_TASK_PAIRING_EVENT_BOND_FULL:
        app_task_pairing_handle_bond_full(&event);
        break;

    case APP_TASK_PAIRING_EVENT_DISCONNECTED:
        app_task_pairing_handle_disconnected(&event);
        break;

    case APP_TASK_PAIRING_EVENT_PAIRING_LOST:
    default:
        app_task_pairing_handle_disconnected(&event);
        break;
    }

    if (app_task_pairing_queue_has_pending())
    {
        if (!app_device_pairing_port_post_event())
        {
            DBG_DIRECT("[PAIR_EVT] repost_failed");
        }
    }
}

