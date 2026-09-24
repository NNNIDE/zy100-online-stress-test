#include "app/app_training_context.h"

#include <stddef.h>
#include <string.h>
#include <os_sync.h>
#include <trace.h>

#include "service/zy100_training_session.h"

typedef struct
{
    uint32_t active_user_id;
    uint32_t next_training_id;
    uint32_t last_synced_training_id;
    bool user_training_synced;
    uint32_t current_capture_user_id;
    uint32_t current_capture_training_id;
    bool current_capture_started;
    uint32_t completed_export_user_id;
    uint32_t completed_export_training_id;
    bool completed_export_valid;
} app_training_context_state_t;

static app_training_context_state_t s_training_context =
{
    .active_user_id = 1U,
    .next_training_id = 1U,
    .last_synced_training_id = 1U,
};

void app_task_training_context_reset_user(void)
{
    uint32_t lock = os_lock();
    memset(&s_training_context, 0, sizeof(s_training_context));
    os_unlock(lock);
}

void app_task_training_context_get(app_task_training_context_t *out)
{
    uint32_t lock_state;

    if (out == NULL)
    {
        return;
    }

    lock_state = os_lock();
    out->active_user_id = s_training_context.active_user_id;
    out->next_training_id = s_training_context.next_training_id;
    out->last_synced_training_id = s_training_context.last_synced_training_id;
    out->user_training_synced = s_training_context.user_training_synced;
    out->current_capture_user_id = s_training_context.current_capture_user_id;
    out->current_capture_training_id = s_training_context.current_capture_training_id;
    out->current_capture_started = s_training_context.current_capture_started;
    out->completed_export_user_id = s_training_context.completed_export_user_id;
    out->completed_export_training_id = s_training_context.completed_export_training_id;
    out->completed_export_valid = s_training_context.completed_export_valid;
    os_unlock(lock_state);
}

void app_task_training_context_apply_time_sync(uint32_t user_id,
                                               uint32_t training_id)
{
    uint32_t export_user_id;
    uint32_t export_training_id;
    bool export_valid;
    uint32_t lock_state;

    lock_state = os_lock();
    s_training_context.active_user_id = user_id;
    s_training_context.next_training_id = training_id;
    s_training_context.last_synced_training_id = training_id;
    s_training_context.user_training_synced = true;
    export_valid = s_training_context.completed_export_valid;
    export_user_id = s_training_context.completed_export_user_id;
    export_training_id = s_training_context.completed_export_training_id;
    os_unlock(lock_state);

    if (export_valid)
    {
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[TRAIN_SYNC] user=%lu next_train=%lu synced=1 keep_export_session=1 export_user=%lu export_train=%lu",
                   (unsigned long)user_id,
                   (unsigned long)training_id,
                   (unsigned long)export_user_id,
                   (unsigned long)export_training_id);
        return;
    }

    ZY100_LOG_ROUTINE(DBG_DIRECT, "[TRAIN_SYNC] user=%lu next_train=%lu synced=1",
               (unsigned long)user_id,
               (unsigned long)training_id);
}

void app_task_training_context_next_for_start(uint32_t *user_id,
                                              uint32_t *training_id,
                                              bool *synced)
{
    uint32_t lock_state = os_lock();

    if (user_id != NULL)
    {
        *user_id = s_training_context.active_user_id;
    }
    if (training_id != NULL)
    {
        *training_id = s_training_context.next_training_id;
    }
    if (synced != NULL)
    {
        *synced = s_training_context.user_training_synced;
    }
    os_unlock(lock_state);
}

void app_task_training_context_mark_start(uint32_t user_id, uint32_t training_id)
{
    uint32_t lock_state = os_lock();

    s_training_context.current_capture_user_id = user_id;
    s_training_context.current_capture_training_id = training_id;
    s_training_context.current_capture_started = true;
    os_unlock(lock_state);
}

void app_task_training_context_clear_current(void)
{
    uint32_t lock_state = os_lock();

    s_training_context.current_capture_user_id = 0U;
    s_training_context.current_capture_training_id = 0U;
    s_training_context.current_capture_started = false;
    os_unlock(lock_state);
    zy100_training_session_clear_active();
}

void app_task_training_context_clear_completed(void)
{
    uint32_t lock_state = os_lock();

    s_training_context.completed_export_user_id = 0U;
    s_training_context.completed_export_training_id = 0U;
    s_training_context.completed_export_valid = false;
    os_unlock(lock_state);
    zy100_training_session_clear_completed();
}

void app_task_training_context_mark_complete(uint32_t reason)
{
    uint32_t user_id;
    uint32_t training_id;
    uint32_t next_training_id;
    uint32_t lock_state;
    bool started;

    lock_state = os_lock();
    started = s_training_context.current_capture_started;
    user_id = s_training_context.current_capture_user_id;
    training_id = s_training_context.current_capture_training_id;
    if (started && (training_id != 0U))
    {
        next_training_id = (training_id == 0xFFFFFFFFUL) ? 1U : (training_id + 1U);
        s_training_context.next_training_id = next_training_id;
        s_training_context.completed_export_user_id = user_id;
        s_training_context.completed_export_training_id = training_id;
        s_training_context.completed_export_valid = true;
        s_training_context.current_capture_user_id = 0U;
        s_training_context.current_capture_training_id = 0U;
        s_training_context.current_capture_started = false;
    }
    else
    {
        next_training_id = s_training_context.next_training_id;
    }
    os_unlock(lock_state);

    if (started && (training_id != 0U))
    {
        DBG_DIRECT("[TRAIN_CTX] complete user=%lu train=%lu next_train=%lu reason=%u",
                   (unsigned long)user_id,
                   (unsigned long)training_id,
                   (unsigned long)next_training_id,
                   reason);
    }
}

bool app_task_training_context_has_export_context(void)
{
    bool has_context;
    uint32_t lock_state = os_lock();

    has_context = s_training_context.current_capture_started ||
                  s_training_context.completed_export_valid;
    os_unlock(lock_state);
    return has_context;
}
