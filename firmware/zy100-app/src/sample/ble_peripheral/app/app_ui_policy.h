#ifndef APP_UI_POLICY_H
#define APP_UI_POLICY_H

#include <stdbool.h>
#include <stdint.h>

#include "service/svc_led_owner.h"

bool app_led_request_notify_off(led_owner_t owner,
                                uint8_t priority,
                                bool logo_off);
bool app_led_request_notify_solid(led_owner_t owner,
                                  uint8_t priority,
                                  led_pattern_color_t color,
                                  bool logo_off);
bool app_led_request_notify_solid_hold(led_owner_t owner,
                                       uint8_t priority,
                                       led_pattern_color_t color,
                                       bool logo_off,
                                       uint32_t hold_ms);
bool app_led_request_notify_blink(led_owner_t owner,
                                  uint8_t priority,
                                  led_pattern_color_t color,
                                  uint32_t on_ms,
                                  uint32_t off_ms,
                                  uint8_t cycles,
                                  bool logo_off);
bool app_led_request_system_breath(led_owner_t owner, uint8_t priority,
                                    led_pattern_color_t color, uint8_t led_mask);
bool app_led_request_capture_breath(led_owner_t owner,
                                    uint8_t priority,
                                    bool logo_off);
bool app_led_request_charge_sequence(led_owner_t owner, uint8_t priority);
bool app_led_request_charge_breath(led_owner_t owner, uint8_t priority);
bool app_led_request_charge_full_blue(led_owner_t owner, uint8_t priority);
bool app_led_request_logo(led_owner_t owner,
                          uint8_t priority,
                          led_pattern_id_t id);
bool app_led_request_wake_logo_step(led_owner_t owner,
                                    uint8_t priority,
                                    uint8_t step);
bool app_led_request_all_off(led_owner_t owner, uint8_t priority);
bool app_led_force_notify_off(bool logo_off);
bool app_led_force_all_off(void);
void app_led_release_runtime_base_owners_except(led_owner_t keep);
bool app_led_request_frame(led_owner_t owner,
                           uint8_t priority,
                           const zy100_rgb_color_t *frame,
                           uint16_t count);

#endif
