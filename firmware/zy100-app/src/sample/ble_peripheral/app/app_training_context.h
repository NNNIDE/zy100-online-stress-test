#ifndef APP_TRAINING_CONTEXT_H
#define APP_TRAINING_CONTEXT_H

#include <stdbool.h>
#include <stdint.h>

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
} app_task_training_context_t;

void app_task_training_context_reset_user(void);
void app_task_training_context_get(app_task_training_context_t *out);
void app_task_training_context_apply_time_sync(uint32_t user_id,
                                               uint32_t training_id);

void app_task_training_context_next_for_start(uint32_t *user_id,
                                              uint32_t *training_id,
                                              bool *synced);
void app_task_training_context_mark_start(uint32_t user_id, uint32_t training_id);
void app_task_training_context_clear_current(void);
void app_task_training_context_clear_completed(void);
void app_task_training_context_mark_complete(uint32_t reason);
bool app_task_training_context_has_export_context(void);

#endif
