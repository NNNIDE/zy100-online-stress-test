#include "app_shutdown_transaction.h"
#include <string.h>

static app_shutdown_snapshot_t s_shutdown;

static void app_shutdown_transaction_clear(void)
{
    memset(&s_shutdown, 0, sizeof(s_shutdown));
}

const app_shutdown_snapshot_t *app_shutdown_transaction_view(void)
{
    return &s_shutdown;
}

bool app_shutdown_transaction_blocks_business(void)
{
    return s_shutdown.phase != APP_SHUTDOWN_PHASE_IDLE;
}

void app_shutdown_transaction_prepare_request(void)
{
    app_shutdown_transaction_clear();
}

void app_shutdown_transaction_latch(app_shutdown_source_t source, bool require_release)
{
    s_shutdown.phase = APP_SHUTDOWN_PHASE_LATCHED;
    s_shutdown.source = source;
    s_shutdown.release_required = require_release;
    s_shutdown.release_seen = !require_release;
}

void app_shutdown_transaction_require_release(void)
{
    /* A repeated request does not replace the origin or clear release_seen. */
    s_shutdown.release_required = true;
}

void app_shutdown_transaction_note_release(void)
{
    s_shutdown.release_seen = true;
}

void app_shutdown_transaction_begin_quiescing(void)
{
    /* Also used by the existing failed-commit/USB retry paths. */
    s_shutdown.phase = APP_SHUTDOWN_PHASE_QUIESCING;
}

void app_shutdown_transaction_wait_release(void)
{
    s_shutdown.phase = APP_SHUTDOWN_PHASE_WAIT_RELEASE;
}

void app_shutdown_transaction_begin_commit(bool usb_present)
{
    s_shutdown.phase = APP_SHUTDOWN_PHASE_COMMIT;
    s_shutdown.target_usb_present = usb_present;
}

void app_shutdown_transaction_note_terminal(bool usb_present)
{
    s_shutdown.phase = APP_SHUTDOWN_PHASE_TERMINAL;
    s_shutdown.target_usb_present = usb_present;
}

void app_shutdown_transaction_request_issued(uint8_t request)
{
    s_shutdown.request_mask |= request;
}

void app_shutdown_transaction_resources_invalidated(void)
{
    s_shutdown.resources_ready = false;
}

void app_shutdown_transaction_resources_ready(void)
{
    s_shutdown.resources_ready = true;
}

bool app_shutdown_transaction_update_blockers(uint8_t blockers)
{
    if (s_shutdown.blocker_mask == blockers) return false;
    s_shutdown.blocker_mask = blockers;
    return true;
}

bool app_shutdown_transaction_note_retry(uint64_t now_ms)
{
    if (!app_shutdown_transaction_blocks_business()) return false;
    s_shutdown.retry_count++;
    if (s_shutdown.last_retry_log_ms == 0U ||
        (now_ms - s_shutdown.last_retry_log_ms) >= 1000U)
    {
        s_shutdown.last_retry_log_ms = now_ms;
        return true;
    }
    return false;
}

void app_shutdown_transaction_note_storage_fault(uint32_t now_ms)
{
    s_shutdown.storage_fault_exit = true;
    s_shutdown.storage_fault_ms = now_ms;
}

void app_shutdown_transaction_release_authorized_wake(void)
{
    app_shutdown_transaction_clear();
}

void app_shutdown_transaction_release_pairing_initialized(void)
{
    app_shutdown_transaction_clear();
}
