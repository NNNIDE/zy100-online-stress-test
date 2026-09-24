#ifndef SVC_LED_PATTERN_H
#define SVC_LED_PATTERN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../driver/drv_rgb_led.h"

#ifndef ZY100_LED_NOTIFY_BRIGHTNESS
#define ZY100_LED_NOTIFY_BRIGHTNESS 255U
#endif

#ifndef ZY100_LED_LOGO_TEST_BRIGHTNESS
#define ZY100_LED_LOGO_TEST_BRIGHTNESS 255U
#endif

#ifndef ZY100_LED_HW_ENABLE
#define ZY100_LED_HW_ENABLE 1U
#endif

typedef enum
{
    LED_LOGO = 0U,
    LED_NOTIFY = 1U,
} svc_led_group_t;

/* Recursive, task-context lock shared with cancellable animation frames. */
bool svc_led_pattern_lock(void);
void svc_led_pattern_unlock(void);
bool svc_led_pattern_init(void);
bool svc_led_pattern_resume_after_wake(void);
bool svc_led_pattern_show_frame(const zy100_rgb_color_t *frame, uint16_t count);
bool svc_led_pattern_set_group_color(svc_led_group_t group, const zy100_rgb_color_t *color);
bool svc_led_pattern_group_off(svc_led_group_t group);
bool svc_led_pattern_notify_rgb(uint8_t red, uint8_t green, uint8_t blue);
bool svc_led_pattern_notify_green_on(void);
bool svc_led_pattern_notify_blue_on(void);
bool svc_led_pattern_notify_red_on(void);
bool svc_led_pattern_notify_orange_on(void);
bool svc_led_pattern_notify_off(void);
bool svc_led_pattern_logo_rgb(uint8_t red, uint8_t green, uint8_t blue);
bool svc_led_pattern_logo_green_on(void);
bool svc_led_pattern_logo_off(void);
bool svc_led_pattern_all_off(void);
void svc_led_pattern_shutdown_for_sleep(void);

#ifdef __cplusplus
}
#endif

#endif /* SVC_LED_PATTERN_H */
