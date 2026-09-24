#ifndef ZY100_OFFLINE_V2_UI_H
#define ZY100_OFFLINE_V2_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

void zy100_offline_v2_ui_init(void);
bool zy100_offline_v2_ui_prepare_begin(uint64_t now_ms);
bool zy100_offline_v2_ui_prepare_ready(uint64_t now_ms);
typedef enum
{
    ZY100_OFFLINE_V2_BUTTON_UI_WAIT = 0U,
    ZY100_OFFLINE_V2_BUTTON_UI_READY,
    ZY100_OFFLINE_V2_BUTTON_UI_FAILED,
} zy100_offline_v2_button_ui_result_t;
/* Start the first ON phase when the second press is accepted. */
bool zy100_offline_v2_ui_button_prepare_begin(uint64_t now_ms);
/* Continue existing flashes after BLE recovery; never restart the animation. */
bool zy100_offline_v2_ui_button_prepare_resume(uint64_t now_ms);
zy100_offline_v2_button_ui_result_t zy100_offline_v2_ui_button_prepare_poll(uint64_t now_ms);
bool zy100_offline_v2_ui_button_waiting(void);
bool zy100_offline_v2_ui_button_wait_expired(uint64_t now_ms, uint32_t timeout_ms);
void zy100_offline_v2_ui_show_running(void);
void zy100_offline_v2_ui_show_stopping(void);
/* Accepted button stop: UI progresses independently of the storage drain. */
bool zy100_offline_v2_ui_button_stop_begin(uint64_t now_ms);
/* True when playback releases its owner after durable storage completion. */
bool zy100_offline_v2_ui_button_stop_poll(uint64_t now_ms);
bool zy100_offline_v2_ui_button_stop_active(void);
/* True consumes this button stop's completion, including canceled feedback. */
bool zy100_offline_v2_ui_button_stop_finish(uint64_t now_ms);
void zy100_offline_v2_ui_button_stop_cancel(void);
void zy100_offline_v2_ui_show_complete(void);
void zy100_offline_v2_ui_show_capacity(void);
void zy100_offline_v2_ui_show_fault(void);
void zy100_offline_v2_ui_release(void);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_OFFLINE_V2_UI_H */
