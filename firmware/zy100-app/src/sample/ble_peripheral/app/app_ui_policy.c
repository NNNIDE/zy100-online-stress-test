#include "app/app_ui_policy.h"

#include <stddef.h>

#include "service/zy100_feature_config.h"

bool app_led_request_notify_off(led_owner_t owner,
                                uint8_t priority,
                                bool logo_off)
{
    led_pattern_t pattern;

    led_pattern_notify_off(&pattern, logo_off);
    return led_request(owner, priority, &pattern);
}

bool app_led_request_notify_solid(led_owner_t owner,
                                  uint8_t priority,
                                  led_pattern_color_t color,
                                  bool logo_off)
{
    led_pattern_t pattern;

    led_pattern_notify_solid(&pattern, color, logo_off);
    return led_request(owner, priority, &pattern);
}

bool app_led_request_notify_solid_hold(led_owner_t owner,
                                       uint8_t priority,
                                       led_pattern_color_t color,
                                       bool logo_off,
                                       uint32_t hold_ms)
{
    led_pattern_t pattern;

    led_pattern_notify_solid(&pattern, color, logo_off);
    pattern.hold_ms = hold_ms;
    return led_request(owner, priority, &pattern);
}

bool app_led_request_notify_blink(led_owner_t owner,
                                  uint8_t priority,
                                  led_pattern_color_t color,
                                  uint32_t on_ms,
                                  uint32_t off_ms,
                                  uint8_t cycles,
                                  bool logo_off)
{
    led_pattern_t pattern;

    led_pattern_notify_blink(&pattern, color, on_ms, off_ms, cycles, logo_off);
    return led_request(owner, priority, &pattern);
}

bool app_led_request_system_breath(led_owner_t owner, uint8_t priority,
                                    led_pattern_color_t color, uint8_t led_mask)
{
    led_pattern_t pattern;
    led_pattern_system_breath(&pattern, color, led_mask);
    return led_request(owner, priority, &pattern);
}

bool app_led_request_capture_breath(led_owner_t owner,
                                    uint8_t priority,
                                    bool logo_off)
{
    led_pattern_t pattern;
    led_training_pattern_config_t training;
    zy100_feature_config_t config;

    (void)logo_off;
    if (!zy100_feature_config_get(&config, NULL, NULL))
    {
        zy100_feature_config_default(&config);
    }
    training.enabled =
        (config.flags & ZY100_FEATURE_CONFIG_FLAG_TRAINING_LED) != 0U;
    training.reverse =
        (config.flags & ZY100_FEATURE_CONFIG_FLAG_REVERSE) != 0U;
    training.target = config.target;
    training.effect = config.effect;
    training.red = config.red;
    training.green = config.green;
    training.blue = config.blue;
    training.brightness_percent = config.brightness_percent;
    training.speed = zy100_feature_config_speed(&config);
    led_pattern_training_config(&pattern, &training);
    return led_request(owner, priority, &pattern);
}

bool app_led_request_charge_sequence(led_owner_t owner, uint8_t priority)
{
    led_pattern_t pattern;

    led_pattern_charge_sequence(&pattern);
    return led_request(owner, priority, &pattern);
}

bool app_led_request_charge_breath(led_owner_t owner, uint8_t priority)
{
    led_pattern_t pattern;

    led_pattern_charge_breath(&pattern);
    return led_request(owner, priority, &pattern);
}

bool app_led_request_charge_full_blue(led_owner_t owner, uint8_t priority)
{
    led_pattern_t pattern;

    led_pattern_charge_full_blue(&pattern);
    return led_request(owner, priority, &pattern);
}

bool app_led_request_logo(led_owner_t owner,
                          uint8_t priority,
                          led_pattern_id_t id)
{
    led_pattern_t pattern;

    led_pattern_logo_simple(&pattern, id);
    return led_request(owner, priority, &pattern);
}

bool app_led_request_wake_logo_step(led_owner_t owner,
                                    uint8_t priority,
                                    uint8_t step)
{
    led_pattern_t pattern;

    led_pattern_wake_logo_chase_step(&pattern, step);
    return led_request(owner, priority, &pattern);
}

bool app_led_request_all_off(led_owner_t owner, uint8_t priority)
{
    led_pattern_t pattern;

    led_pattern_all_off(&pattern);
    return led_request(owner, priority, &pattern);
}

bool app_led_force_notify_off(bool logo_off)
{
    return led_force_notify_off(logo_off);
}

bool app_led_force_all_off(void)
{
    return led_force_all_off();
}

void app_led_release_runtime_base_owners_except(led_owner_t keep)
{
    static const led_owner_t runtime_owners[] =
    {
        LED_OWNER_FIND_DEVICE,
        LED_OWNER_STATUS_QUERY,
        LED_OWNER_ACTIVE_BLE_WAIT_TIMEOUT,
        LED_OWNER_CAPTURING,
        LED_OWNER_ONLINE_RUNNING,
        LED_OWNER_PENDING_EXPORT,
        LED_OWNER_CLEAR_FLASH,
    };
    uint32_t index;

    for (index = 0U; index < (sizeof(runtime_owners) / sizeof(runtime_owners[0])); index++)
    {
        if (runtime_owners[index] != keep)
        {
            led_release(runtime_owners[index]);
        }
    }
}

bool app_led_request_frame(led_owner_t owner,
                           uint8_t priority,
                           const zy100_rgb_color_t *frame,
                           uint16_t count)
{
    return led_request_frame(owner, priority, frame, count);
}
