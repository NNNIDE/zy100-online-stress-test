#include "dfu_watchdog.h"

#include <stddef.h>

#include "../sample/ble_peripheral/app_flags.h"
#include "board.h"

#if (SUPPORT_NORMAL_OTA == 1)

#include "os_sched.h"
#include "os_sync.h"
#include "rtl876x_aon_wdg.h"
#include "trace.h"

#if (ZY100_APP_AON_WDG_TIMEOUT_SECONDS == 0U)
#error "ZY100_APP_AON_WDG_TIMEOUT_SECONDS must be non-zero"
#endif
#if (ZY100_APP_AON_WDG_TIMEOUT_SECONDS > 65U)
#error "ZY100_APP_AON_WDG_TIMEOUT_SECONDS exceeds supported AON WDG timeout"
#endif
#if (ZY100_APP_AON_WDG_FEED_PERIOD_MS == 0U)
#error "ZY100_APP_AON_WDG_FEED_PERIOD_MS must be non-zero"
#endif
#if (ZY100_APP_AON_WDG_FEED_PERIOD_MS >= (ZY100_APP_AON_WDG_TIMEOUT_SECONDS * 1000U))
#error "ZY100_APP_AON_WDG_FEED_PERIOD_MS must be shorter than watchdog timeout"
#endif

#define DFU_WATCHDOG_RESET_LEVEL             1U
#define DFU_WATCHDOG_TASK_WAIT_MS            (ZY100_APP_AON_WDG_FEED_PERIOD_MS / 2U)

#if (DFU_WATCHDOG_TASK_WAIT_MS == 0U)
#error "DFU watchdog task wait must be non-zero"
#endif

static bool s_dfu_watchdog_enabled = false;
static uint32_t s_dfu_watchdog_operation = (uint32_t)DFU_WATCHDOG_OPERATION_NONE;
static bool s_dfu_watchdog_invariant_fault = false;
static uint32_t s_dfu_watchdog_last_feed_ms = 0U;
static uint32_t s_dfu_watchdog_last_progress_ms = 0U;
static uint32_t s_dfu_watchdog_progress_type = (uint32_t)DFU_WATCHDOG_PROGRESS_NONE;
static uint32_t s_dfu_watchdog_progress_value = 0U;
static uint32_t s_dfu_watchdog_feed_count = 0U;

static void dfu_watchdog_latch_invariant_fault_locked(void)
{
    s_dfu_watchdog_invariant_fault = true;
}

static bool dfu_watchdog_feed_locked(uint32_t now_ms)
{
    if (!s_dfu_watchdog_enabled || s_dfu_watchdog_invariant_fault ||
        !AON_WDG_IsEnable())
    {
        s_dfu_watchdog_enabled = false;
        dfu_watchdog_latch_invariant_fault_locked();
        return false;
    }

    AON_WDG_Restart();
    s_dfu_watchdog_last_feed_ms = now_ms;
    s_dfu_watchdog_feed_count++;
    return true;
}

bool dfu_watchdog_init(void)
{
    aon_wdg_disable();
    s_dfu_watchdog_enabled = false;
    s_dfu_watchdog_operation = (uint32_t)DFU_WATCHDOG_OPERATION_NONE;
    s_dfu_watchdog_invariant_fault = false;
    s_dfu_watchdog_last_feed_ms = 0U;
    s_dfu_watchdog_last_progress_ms = 0U;
    s_dfu_watchdog_progress_type = (uint32_t)DFU_WATCHDOG_PROGRESS_NONE;
    s_dfu_watchdog_progress_value = 0U;
    s_dfu_watchdog_feed_count = 0U;

    aon_wdg_init((uint8_t)DFU_WATCHDOG_RESET_LEVEL,
                 (uint8_t)ZY100_APP_AON_WDG_TIMEOUT_SECONDS);
    AON_WDG_Restart();
    aon_wdg_enable();

    if (!AON_WDG_IsEnable())
    {
        s_dfu_watchdog_invariant_fault = true;
        DBG_DIRECT("[DFU_WDG][FAULT] enable verification failed");
        return false;
    }

    s_dfu_watchdog_enabled = true;
    s_dfu_watchdog_last_feed_ms = (uint32_t)os_sys_time_get();
    s_dfu_watchdog_feed_count = 1U;
    DBG_DIRECT("[DFU_WDG] enabled reset_level=%u timeout_s=%lu feed_ms=%lu wait_ms=%lu",
               (unsigned int)DFU_WATCHDOG_RESET_LEVEL,
               (unsigned long)ZY100_APP_AON_WDG_TIMEOUT_SECONDS,
               (unsigned long)ZY100_APP_AON_WDG_FEED_PERIOD_MS,
               (unsigned long)DFU_WATCHDOG_TASK_WAIT_MS);
    return true;
}

void dfu_watchdog_poll(uint32_t now_ms)
{
    uint32_t lock_state = os_lock();
    bool feed_due = s_dfu_watchdog_enabled && !s_dfu_watchdog_invariant_fault &&
                    (s_dfu_watchdog_operation == (uint32_t)DFU_WATCHDOG_OPERATION_NONE) &&
                    ((uint32_t)(now_ms - s_dfu_watchdog_last_feed_ms) >=
                     (uint32_t)ZY100_APP_AON_WDG_FEED_PERIOD_MS);
    bool feed_ok = true;

    if (feed_due)
    {
        feed_ok = dfu_watchdog_feed_locked(now_ms);
    }
    os_unlock(lock_state);

    if (feed_due && !feed_ok)
    {
        DBG_DIRECT("[DFU_WDG][FAULT] task poll feed rejected");
    }
}

uint32_t dfu_watchdog_task_wait_ms(void)
{
    return (uint32_t)DFU_WATCHDOG_TASK_WAIT_MS;
}

bool dfu_watchdog_operation_begin(T_DFU_WATCHDOG_OPERATION operation)
{
    uint32_t now_ms = (uint32_t)os_sys_time_get();
    uint32_t lock_state = os_lock();
    uint32_t current = s_dfu_watchdog_operation;
    bool valid = s_dfu_watchdog_enabled && !s_dfu_watchdog_invariant_fault &&
                 (operation != DFU_WATCHDOG_OPERATION_NONE) &&
                 (current == (uint32_t)DFU_WATCHDOG_OPERATION_NONE);

    if (!valid || !dfu_watchdog_feed_locked(now_ms))
    {
        dfu_watchdog_latch_invariant_fault_locked();
        os_unlock(lock_state);
        DBG_DIRECT("[DFU_WDG][FAULT] operation begin requested=%u current=%u",
                   (unsigned int)operation, (unsigned int)current);
        return false;
    }

    s_dfu_watchdog_operation = (uint32_t)operation;
    os_unlock(lock_state);
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[DFU_WDG] operation_begin op=%u feed_ms=%lu",
               (unsigned int)operation, (unsigned long)now_ms);
    return true;
}

bool dfu_watchdog_operation_end(T_DFU_WATCHDOG_OPERATION operation)
{
    uint32_t now_ms = (uint32_t)os_sys_time_get();
    uint32_t lock_state = os_lock();
    uint32_t current = s_dfu_watchdog_operation;
    bool valid = s_dfu_watchdog_enabled && !s_dfu_watchdog_invariant_fault &&
                 (operation != DFU_WATCHDOG_OPERATION_NONE) &&
                 (current == (uint32_t)operation);

    if (!valid)
    {
        dfu_watchdog_latch_invariant_fault_locked();
        os_unlock(lock_state);
        DBG_DIRECT("[DFU_WDG][FAULT] operation end requested=%u current=%u",
                   (unsigned int)operation, (unsigned int)current);
        return false;
    }

    s_dfu_watchdog_operation = (uint32_t)DFU_WATCHDOG_OPERATION_NONE;
    if (!dfu_watchdog_feed_locked(now_ms))
    {
        os_unlock(lock_state);
        DBG_DIRECT("[DFU_WDG][FAULT] operation end feed rejected op=%u",
                   (unsigned int)operation);
        return false;
    }
    os_unlock(lock_state);
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[DFU_WDG] operation_end op=%u feed_ms=%lu",
               (unsigned int)operation, (unsigned long)now_ms);
    return true;
}

bool dfu_watchdog_note_progress(T_DFU_WATCHDOG_PROGRESS progress_type,
                                uint32_t progress_value)
{
    uint32_t now_ms = (uint32_t)os_sys_time_get();
    uint32_t lock_state = os_lock();
    uint32_t operation = s_dfu_watchdog_operation;
    bool valid = s_dfu_watchdog_enabled && !s_dfu_watchdog_invariant_fault &&
                 (progress_type != DFU_WATCHDOG_PROGRESS_NONE) &&
                 (operation == (uint32_t)DFU_WATCHDOG_OPERATION_NONE);

    if (!valid || !dfu_watchdog_feed_locked(now_ms))
    {
        dfu_watchdog_latch_invariant_fault_locked();
        os_unlock(lock_state);
        DBG_DIRECT("[DFU_WDG][FAULT] progress rejected type=%u value=%lu op=%u",
                   (unsigned int)progress_type,
                   (unsigned long)progress_value,
                   (unsigned int)operation);
        return false;
    }

    s_dfu_watchdog_progress_type = (uint32_t)progress_type;
    s_dfu_watchdog_progress_value = progress_value;
    s_dfu_watchdog_last_progress_ms = now_ms;
    os_unlock(lock_state);
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[DFU_WDG] progress type=%u value=%lu feed_ms=%lu",
               (unsigned int)progress_type,
               (unsigned long)progress_value,
               (unsigned long)now_ms);
    return true;
}

bool dfu_watchdog_snapshot(T_DFU_WATCHDOG_SNAPSHOT *snapshot)
{
    uint32_t lock_state;

    if (snapshot == NULL)
    {
        return false;
    }

    lock_state = os_lock();
    snapshot->enabled = s_dfu_watchdog_enabled;
    snapshot->invariant_fault = s_dfu_watchdog_invariant_fault;
    snapshot->operation = (T_DFU_WATCHDOG_OPERATION)s_dfu_watchdog_operation;
    snapshot->progress_type = (T_DFU_WATCHDOG_PROGRESS)s_dfu_watchdog_progress_type;
    snapshot->progress_value = s_dfu_watchdog_progress_value;
    snapshot->last_feed_ms = s_dfu_watchdog_last_feed_ms;
    snapshot->last_progress_ms = s_dfu_watchdog_last_progress_ms;
    snapshot->feed_count = s_dfu_watchdog_feed_count;
    os_unlock(lock_state);
    return true;
}

#else

bool dfu_watchdog_init(void) { return false; }
void dfu_watchdog_poll(uint32_t now_ms) { (void)now_ms; }
uint32_t dfu_watchdog_task_wait_ms(void) { return 0U; }
bool dfu_watchdog_operation_begin(T_DFU_WATCHDOG_OPERATION operation)
{
    (void)operation;
    return true;
}
bool dfu_watchdog_operation_end(T_DFU_WATCHDOG_OPERATION operation)
{
    (void)operation;
    return true;
}
bool dfu_watchdog_note_progress(T_DFU_WATCHDOG_PROGRESS progress_type,
                                uint32_t progress_value)
{
    (void)progress_type;
    (void)progress_value;
    return true;
}
bool dfu_watchdog_snapshot(T_DFU_WATCHDOG_SNAPSHOT *snapshot)
{
    (void)snapshot;
    return false;
}

#endif /* SUPPORT_NORMAL_OTA */
