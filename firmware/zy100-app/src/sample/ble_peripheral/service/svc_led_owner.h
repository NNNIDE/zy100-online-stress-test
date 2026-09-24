#ifndef SVC_LED_OWNER_H
#define SVC_LED_OWNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../bsp/led_board_pinmap.h"
#include "../driver/drv_rgb_led.h"

typedef enum
{
    LED_OWNER_NONE = 0,
    LED_OWNER_BUTTON_HOLD,
    LED_OWNER_BOOT_SEQUENCE,
    LED_OWNER_OTA_UPLOAD_RESULT,
    LED_OWNER_TRAINING_TRANSITION,
    LED_OWNER_BLE_EXPORT,
    LED_OWNER_BATTERY_CRITICAL,
    LED_OWNER_FLASH_FULL_WAIT_UPLOAD,
    LED_OWNER_CHARGE_SLEEP,
    LED_OWNER_BLE_LINK_EVENT,
    LED_OWNER_CHARGE_IDLE,
    LED_OWNER_ACTIVE_BLE_WAIT_TIMEOUT,
    LED_OWNER_FIND_DEVICE,
    LED_OWNER_STATUS_QUERY,
    LED_OWNER_SHORT_PRESS,
    LED_OWNER_CAPTURING,
    LED_OWNER_MAG_CALIBRATION,
    LED_OWNER_CLEAR_FLASH,
    LED_OWNER_PENDING_EXPORT,
    LED_OWNER_ONLINE_FAULT,
    LED_OWNER_ONLINE_TRANSITION,
    LED_OWNER_ONLINE_RUNNING,
    LED_OWNER_OFFLINE_V2_TRANSITION,
    LED_OWNER_OFFLINE_V2_RUNNING,
    LED_OWNER_OFFLINE_V2_FAULT,
    LED_OWNER_COUNT,
} led_owner_t;

#define LED_PRIORITY_NONE                   0U
#define LED_PRIORITY_PENDING_EXPORT         10U
#define LED_PRIORITY_CLEAR_FLASH            20U
#define LED_PRIORITY_CAPTURING              30U
/* Charging is a background indication only.  Every operational/product
 * indication must be able to cover it without explicitly releasing it. */
#define LED_PRIORITY_CHARGE_IDLE             1U
#define LED_PRIORITY_CHARGE_SLEEP            1U
#define LED_PRIORITY_ACTIVE_BLE_WAIT_TIMEOUT 52U
#define LED_PRIORITY_STATUS_QUERY           55U
#define LED_PRIORITY_MAG_CALIBRATION         60U
#define LED_PRIORITY_SHORT_PRESS             61U
#define LED_PRIORITY_FIND_DEVICE            65U
#define LED_PRIORITY_FLASH_FULL_WAIT_UPLOAD 70U
#define LED_PRIORITY_BATTERY_CRITICAL       116U
#define LED_PRIORITY_BLE_EXPORT             90U
#define LED_PRIORITY_TRAINING_TRANSITION    95U
#define LED_PRIORITY_BOOT_SEQUENCE          100U
#define LED_PRIORITY_BLE_LINK_EVENT         105U
#define LED_PRIORITY_OTA_UPLOAD_RESULT      110U
#define LED_PRIORITY_ONLINE_RECOVERY        102U
#define LED_PRIORITY_ONLINE_HARD_FAULT      115U
#define LED_PRIORITY_ONLINE_TRANSITION       95U
#define LED_PRIORITY_ONLINE_RUNNING          30U
#define LED_PRIORITY_OFFLINE_V2_TRANSITION   95U
#define LED_PRIORITY_OFFLINE_V2_RUNNING      30U
#define LED_PRIORITY_OFFLINE_V2_FAULT       115U
#define LED_PRIORITY_BUTTON_HOLD            120U

typedef enum
{
    LED_PATTERN_NONE = 0U,
    LED_PATTERN_NOTIFY_OFF,
    LED_PATTERN_NOTIFY_SOLID,
    LED_PATTERN_NOTIFY_BLINK,
    LED_PATTERN_NOTIFY_BLUE_BREATH,
    LED_PATTERN_CAPTURE_NOTIFY_BLUE_BREATH,
    LED_PATTERN_TRAINING_CONFIG,
    LED_PATTERN_NOTIFY_PURPLE_BREATH,
    LED_PATTERN_LOGO_OFF,
    LED_PATTERN_LOGO_WHITE_HINT,
    LED_PATTERN_LOGO_WHITE,
    LED_PATTERN_FIND_DEVICE_LOGO_WHITE,
    LED_PATTERN_LOGO_BLUE,
    LED_PATTERN_LOGO_FADE_OUT,
    LED_PATTERN_WAKE_LOGO_CHASE_STEP,
    LED_PATTERN_ALL_OFF,
    LED_PATTERN_SHOW_FRAME,
    LED_PATTERN_CHARGE_SEQUENCE,
    LED_PATTERN_CHARGE_BREATH,
    LED_PATTERN_CHARGE_FULL_BLUE,
    LED_PATTERN_SLEEP_CHARGE_BREATH,
    LED_PATTERN_SLEEP_CHARGE_FULL,
    LED_PATTERN_SLEEP_CHARGE_OFF,
    LED_PATTERN_SYSTEM_BREATH,
} led_pattern_id_t;

typedef enum
{
    LED_PATTERN_COLOR_NONE = 0U,
    LED_PATTERN_COLOR_GREEN,
    LED_PATTERN_COLOR_BLUE,
    LED_PATTERN_COLOR_RED,
    LED_PATTERN_COLOR_ORANGE,
    LED_PATTERN_COLOR_WHITE,
    LED_PATTERN_COLOR_PURPLE,
} led_pattern_color_t;

typedef struct
{
    bool enabled;
    bool reverse;
    uint8_t target;
    uint8_t effect;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t brightness_percent;
    uint8_t speed;
} led_training_pattern_config_t;

typedef struct
{
    led_pattern_id_t id;
    led_pattern_color_t color;
    bool logo_off;
    bool immediate_visible;
    uint32_t on_ms;
    uint32_t off_ms;
    uint32_t hold_ms;
    uint8_t cycles;
    uint8_t step;
    uint16_t frame_count;
    uint16_t frame_token;
    led_training_pattern_config_t training;
} led_pattern_t;

#define LED_MASK_LOGO   ((uint8_t)((1U << ZY100_LED_LOGO_COUNT) - 1U))
#define LED_MASK_ALL    ((uint8_t)((1U << ZY100_RGB_LED_COUNT) - 1U))
#define LED_MASK_NOTIFY ((uint8_t)(LED_MASK_ALL & ~LED_MASK_LOGO))
/* SYSTEM_BREATH stores the physical LED mask in step; other IDs retain their
 * existing step meaning. No wire/config structure uses this internal type. */
void led_pattern_system_breath(led_pattern_t *pattern,
                               led_pattern_color_t color, uint8_t led_mask);
void led_pattern_none(led_pattern_t *pattern);
void led_pattern_notify_off(led_pattern_t *pattern, bool logo_off);
void led_pattern_notify_solid(led_pattern_t *pattern,
                              led_pattern_color_t color,
                              bool logo_off);
void led_pattern_notify_blink(led_pattern_t *pattern,
                              led_pattern_color_t color,
                              uint32_t on_ms,
                              uint32_t off_ms,
                              uint8_t cycles,
                              bool logo_off);
void led_pattern_notify_blue_breath(led_pattern_t *pattern, bool logo_off);
void led_pattern_capture_notify_blue_breath(led_pattern_t *pattern, bool logo_off);
void led_pattern_training_config(
    led_pattern_t *pattern,
    const led_training_pattern_config_t *config);
void led_pattern_notify_purple_breath(led_pattern_t *pattern, bool logo_off);
void led_pattern_logo_simple(led_pattern_t *pattern, led_pattern_id_t id);
void led_pattern_find_device_logo_white(led_pattern_t *pattern, uint16_t token);
void led_pattern_wake_logo_chase_step(led_pattern_t *pattern, uint8_t step);
void led_pattern_all_off(led_pattern_t *pattern);
void led_pattern_charge_sequence(led_pattern_t *pattern);
void led_pattern_charge_breath(led_pattern_t *pattern);
void led_pattern_charge_full_blue(led_pattern_t *pattern);
void led_pattern_sleep_charge_breath(led_pattern_t *pattern);
void led_pattern_sleep_charge_full(led_pattern_t *pattern);
void led_pattern_sleep_charge_off(led_pattern_t *pattern);
void led_charge_chase_once_mark_consumed(void);
void led_charge_chase_once_reset(void);

bool led_request(led_owner_t owner, uint8_t priority, const led_pattern_t *pattern);
bool led_request_frame(led_owner_t owner,
                       uint8_t priority,
                       const zy100_rgb_color_t *frame,
                       uint16_t count);
void led_release(led_owner_t owner);
void led_release_all(void);
bool led_force_notify_off(bool logo_off);
bool led_force_all_off(void);
void led_tick(uint64_t now_ms);
led_owner_t led_current_owner(void);
led_pattern_id_t led_current_pattern_id(void);
const char *led_owner_name_get(led_owner_t owner);
bool led_owner_is_active(led_owner_t owner);
bool led_has_active_priority_above(uint8_t priority);
void led_debug_dump_state(const char *tag, const char *reason);

#ifdef __cplusplus
}
#endif

#endif /* SVC_LED_OWNER_H */
