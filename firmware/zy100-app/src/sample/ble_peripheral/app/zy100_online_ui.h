#ifndef ZY100_ONLINE_UI_H
#define ZY100_ONLINE_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

void zy100_online_ui_init(void);
bool zy100_online_ui_prepare_begin(uint64_t now_ms);
bool zy100_online_ui_prepare_ready(uint64_t now_ms);
void zy100_online_ui_show_running(void);
bool zy100_online_ui_show_complete(uint64_t now_ms);
bool zy100_online_ui_poll(uint64_t now_ms);
bool zy100_online_ui_complete_active(void);
uint64_t zy100_online_ui_complete_elapsed_ms(uint64_t now_ms);
void zy100_online_ui_cancel_complete(const char *reason);
void zy100_online_ui_show_fault(void);
void zy100_online_ui_release(void);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_ONLINE_UI_H */
