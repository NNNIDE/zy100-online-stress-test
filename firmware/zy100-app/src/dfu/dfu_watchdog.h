#ifndef DFU_WATCHDOG_H
#define DFU_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    DFU_WATCHDOG_OPERATION_NONE = 0U,
    DFU_WATCHDOG_OPERATION_FLASH_WRITE,
    DFU_WATCHDOG_OPERATION_FLASH_ERASE,
    DFU_WATCHDOG_OPERATION_IMAGE_VALIDATE,
    DFU_WATCHDOG_OPERATION_READY_COMMIT,
    DFU_WATCHDOG_OPERATION_IMAGE_COPY,
} T_DFU_WATCHDOG_OPERATION;

typedef enum
{
    DFU_WATCHDOG_PROGRESS_NONE = 0U,
    DFU_WATCHDOG_PROGRESS_BUFFER_COMMITTED,
    DFU_WATCHDOG_PROGRESS_IMAGE_VALIDATED,
    DFU_WATCHDOG_PROGRESS_READY_COMMITTED,
    DFU_WATCHDOG_PROGRESS_IMAGE_COPIED,
} T_DFU_WATCHDOG_PROGRESS;

typedef struct
{
    bool enabled;
    bool invariant_fault;
    T_DFU_WATCHDOG_OPERATION operation;
    T_DFU_WATCHDOG_PROGRESS progress_type;
    uint32_t progress_value;
    uint32_t last_feed_ms;
    uint32_t last_progress_ms;
    uint32_t feed_count;
} T_DFU_WATCHDOG_SNAPSHOT;

bool dfu_watchdog_init(void);
void dfu_watchdog_poll(uint32_t now_ms);
uint32_t dfu_watchdog_task_wait_ms(void);
bool dfu_watchdog_operation_begin(T_DFU_WATCHDOG_OPERATION operation);
bool dfu_watchdog_operation_end(T_DFU_WATCHDOG_OPERATION operation);
bool dfu_watchdog_note_progress(T_DFU_WATCHDOG_PROGRESS progress_type,
                                uint32_t progress_value);
bool dfu_watchdog_snapshot(T_DFU_WATCHDOG_SNAPSHOT *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* DFU_WATCHDOG_H */
