#ifndef SVC_LED_TASK_H
#define SVC_LED_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    SVC_LED_TASK_STATE_OFF = 0U,
    SVC_LED_TASK_STATE_GREEN,
    SVC_LED_TASK_STATE_BLUE,
    SVC_LED_TASK_STATE_RED,
    SVC_LED_TASK_STATE_SLEEP_OFF,
    SVC_LED_TASK_STATE_LOGO_WHITE_HINT,
    SVC_LED_TASK_STATE_LOGO_WHITE,
    SVC_LED_TASK_STATE_LOGO_BLUE,
    SVC_LED_TASK_STATE_LOGO_CHASE,
    SVC_LED_TASK_STATE_BLUE_FAST_BLINK,
    SVC_LED_TASK_STATE_RED_SLOW_BLINK,
    SVC_LED_TASK_STATE_ORANGE_SLOW_BLINK,
    SVC_LED_TASK_STATE_BLUE_BREATH,
    SVC_LED_TASK_STATE_CHARGE_SEQUENCE,
    SVC_LED_TASK_STATE_CHARGE_FULL_BLUE,
    SVC_LED_TASK_STATE_SLEEP_CHARGE,
    SVC_LED_TASK_STATE_SLEEP_CHARGE_FULL,
    SVC_LED_TASK_STATE_SYSTEM_BREATH,
} svc_led_task_state_t;

#ifndef SVC_LED_TASK_LOGO_HINT_BRIGHTNESS
#define SVC_LED_TASK_LOGO_HINT_BRIGHTNESS 76U
#endif

#ifndef SVC_LED_TASK_BLUE_BREATH_PERIOD_MS
#define SVC_LED_TASK_BLUE_BREATH_PERIOD_MS 1600U
#endif
#ifndef SVC_LED_TASK_BLUE_BREATH_STEP_MS
#define SVC_LED_TASK_BLUE_BREATH_STEP_MS 20U
#endif
#ifndef SVC_LED_TASK_LOGO_FADE_TOTAL_MS
#define SVC_LED_TASK_LOGO_FADE_TOTAL_MS (SVC_LED_TASK_BLUE_BREATH_PERIOD_MS / 2U)
#endif
#ifndef SVC_LED_TASK_LOGO_FADE_STEP_MS
#define SVC_LED_TASK_LOGO_FADE_STEP_MS SVC_LED_TASK_BLUE_BREATH_STEP_MS
#endif
#ifndef SVC_LED_TASK_BLUE_BREATH_MIN_LEVEL
#define SVC_LED_TASK_BLUE_BREATH_MIN_LEVEL 4U
#endif
#ifndef SVC_LED_TASK_BLUE_BREATH_MAX_LEVEL
#define SVC_LED_TASK_BLUE_BREATH_MAX_LEVEL 76U
#endif
#ifndef SVC_LED_TASK_CHARGE_LOGO_STEP_MS
#define SVC_LED_TASK_CHARGE_LOGO_STEP_MS 200U
#endif

/* Color channels are normalized 0..255; mask selects physical chain LEDs. */
bool svc_led_task_system_breath(uint8_t red, uint8_t green, uint8_t blue,
                                uint8_t led_mask);
bool svc_led_task_init(void);
bool svc_led_task_resume_after_wake(void);
bool svc_led_task_resume_logo_white_hint_async(void);
void svc_led_task_mark_sleep(void);

bool svc_led_task_notify_green_on(void);
bool svc_led_task_notify_blue_on(void);
bool svc_led_task_notify_red_on(void);
bool svc_led_task_notify_off(void);
bool svc_led_task_all_off(void);
void svc_led_task_cancel_sequence(void);
bool svc_led_task_logo_white_hint_on(void);
bool svc_led_task_logo_white_hint_immediate(void);
bool svc_led_task_logo_white_on(void);
bool svc_led_task_logo_blue_on(void);
bool svc_led_task_logo_fade_out(void);
/* Completion belongs to the current LED command generation. */
bool svc_led_task_logo_fade_complete(void);
bool svc_led_task_logo_fade_pending(void);
bool svc_led_task_logo_off(void);
bool svc_led_task_wake_logo_chase_step(uint8_t step);
bool svc_led_task_notify_blue_fast_blink(void);
bool svc_led_task_notify_red_slow_blink(void);
bool svc_led_task_notify_orange_slow_blink(void);
bool svc_led_task_notify_blue_breath(void);
bool svc_led_task_notify_blue_breath_start(bool immediate_visible);
bool svc_led_task_charge_sequence_start(void);
bool svc_led_task_charge_sequence_chase_step(uint8_t step);
bool svc_led_task_charge_sequence_breath_step(uint8_t blue_level);
bool svc_led_task_charge_full_blue_on(void);
bool svc_led_task_sleep_charge_latch_on(void);
bool svc_led_task_sleep_charge_latch_off(void);
void svc_led_task_sleep_charge_breath_reset(void);
bool svc_led_task_sleep_charge_breath_step(void);
bool svc_led_task_sleep_charge_full_blue_on(void);

bool svc_led_task_red_is_on_expected(void);
uint8_t svc_led_task_expected_out(void);

#ifdef __cplusplus
}
#endif

#endif /* SVC_LED_TASK_H */
