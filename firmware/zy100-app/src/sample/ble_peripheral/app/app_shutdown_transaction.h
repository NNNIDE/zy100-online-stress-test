#ifndef APP_SHUTDOWN_TRANSACTION_H
#define APP_SHUTDOWN_TRANSACTION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    APP_SHUTDOWN_PHASE_IDLE = 0U,
    APP_SHUTDOWN_PHASE_LATCHED,
    APP_SHUTDOWN_PHASE_QUIESCING,
    APP_SHUTDOWN_PHASE_WAIT_RELEASE,
    APP_SHUTDOWN_PHASE_COMMIT,
    APP_SHUTDOWN_PHASE_TERMINAL,
} app_shutdown_phase_t;

typedef enum
{
    APP_SHUTDOWN_SOURCE_NONE = 0U,
    APP_SHUTDOWN_SOURCE_BUTTON_LONG_PRESS,
    APP_SHUTDOWN_SOURCE_BOOT_LOW_BATTERY,
    APP_SHUTDOWN_SOURCE_RUNTIME_LOW_BATTERY,
    APP_SHUTDOWN_SOURCE_ONLINE_LOW_BATTERY,
    APP_SHUTDOWN_SOURCE_STORAGE_FAULT,
    APP_SHUTDOWN_SOURCE_CONFIG_APPLY_FAULT,
    APP_SHUTDOWN_SOURCE_PAIRING_TIMEOUT,
    APP_SHUTDOWN_SOURCE_PAIRING_START_FAILED,
} app_shutdown_source_t;

#define APP_SHUTDOWN_REQUEST_BLE       0x01U
#define APP_SHUTDOWN_REQUEST_ONLINE    0x02U
#define APP_SHUTDOWN_REQUEST_OFFLINE   0x04U
#define APP_SHUTDOWN_REQUEST_EXPORT    0x08U
#define APP_SHUTDOWN_REQUEST_BLE_SILENT 0x10U
#define APP_SHUTDOWN_BLOCK_CAPTURE     0x01U
#define APP_SHUTDOWN_BLOCK_STORAGE     0x02U
#define APP_SHUTDOWN_BLOCK_FLASH       0x04U
#define APP_SHUTDOWN_BLOCK_BLE         0x08U
#define APP_SHUTDOWN_BLOCK_CAL         0x10U
#define APP_SHUTDOWN_BLOCK_ADC         0x20U
#define APP_SHUTDOWN_BLOCK_RESOURCE    0x40U

typedef struct
{
    app_shutdown_phase_t phase;
    app_shutdown_source_t source;
    uint8_t request_mask;
    uint8_t blocker_mask;
    bool release_required;
    bool release_seen;
    bool target_usb_present;
    bool resources_ready;
    uint32_t retry_count;
    uint64_t last_retry_log_ms;
    bool storage_fault_exit;
    uint32_t storage_fault_ms;
} app_shutdown_snapshot_t;

/* Synchronous API in the existing caller contexts. No hardware, clock,
 * logging or additional locks; existing execution/serialization stays intact.
 * The view is read-only and live: do not retain it across lifecycle actions.
 * Copy *view when a stable diagnostic snapshot is needed. */
const app_shutdown_snapshot_t *app_shutdown_transaction_view(void);
bool app_shutdown_transaction_blocks_business(void);

/* Only the existing first-request path prepares a fresh transaction. Keep
 * preparation separate from latch: runtime cleanup occurs between the two. */
void app_shutdown_transaction_prepare_request(void);
void app_shutdown_transaction_latch(app_shutdown_source_t source, bool require_release);
void app_shutdown_transaction_require_release(void);
void app_shutdown_transaction_note_release(void);
void app_shutdown_transaction_begin_quiescing(void);
void app_shutdown_transaction_wait_release(void);
void app_shutdown_transaction_begin_commit(bool usb_present);
void app_shutdown_transaction_note_terminal(bool usb_present);
void app_shutdown_transaction_request_issued(uint8_t request);
void app_shutdown_transaction_resources_invalidated(void);
void app_shutdown_transaction_resources_ready(void);
bool app_shutdown_transaction_update_blockers(uint8_t blockers);
/* Returns whether the existing 1000 ms retry-log window is due. */
bool app_shutdown_transaction_note_retry(uint64_t now_ms);
void app_shutdown_transaction_note_storage_fault(uint32_t now_ms);

/* Call only at the existing authorized wake / successful pairing-init sites.
 * Eligibility and hardware initialization remain with those application ports. */
void app_shutdown_transaction_release_authorized_wake(void);
void app_shutdown_transaction_release_pairing_initialized(void);

#endif
