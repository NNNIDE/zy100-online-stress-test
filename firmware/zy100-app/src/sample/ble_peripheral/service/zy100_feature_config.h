#ifndef ZY100_FEATURE_CONFIG_H
#define ZY100_FEATURE_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define ZY100_FEATURE_CONFIG_SCHEMA_VERSION       1U
#define ZY100_FEATURE_CONFIG_WIRE_BYTES           8U
#define ZY100_FEATURE_CONFIG_ACK_RESERVED         1U

#define ZY100_FEATURE_CONFIG_FLAG_AUTO_CAPTURE    0x01U
#define ZY100_FEATURE_CONFIG_FLAG_TRAINING_LED     0x02U
#define ZY100_FEATURE_CONFIG_FLAG_REVERSE          0x04U
#define ZY100_FEATURE_CONFIG_SPEED_SHIFT           3U
#define ZY100_FEATURE_CONFIG_SPEED_MASK            0x18U
#define ZY100_FEATURE_CONFIG_FLAGS_ALLOWED         0x1FU

typedef enum
{
    ZY100_FEATURE_LED_TARGET_NOTIFY = 1U,
    ZY100_FEATURE_LED_TARGET_LOGO = 2U,
    ZY100_FEATURE_LED_TARGET_ALL = 3U,
} zy100_feature_led_target_t;

typedef enum
{
    ZY100_FEATURE_LED_EFFECT_SOLID = 1U,
    ZY100_FEATURE_LED_EFFECT_BLINK = 2U,
    ZY100_FEATURE_LED_EFFECT_BREATH = 3U,
    ZY100_FEATURE_LED_EFFECT_MARQUEE = 4U,
} zy100_feature_led_effect_t;

typedef enum
{
    ZY100_FEATURE_LED_SPEED_CAPTURE_DEFAULT = 0U,
    ZY100_FEATURE_LED_SPEED_SLOW = 1U,
    ZY100_FEATURE_LED_SPEED_STANDARD = 2U,
    ZY100_FEATURE_LED_SPEED_FAST = 3U,
} zy100_feature_led_speed_t;

typedef struct __attribute__((packed))
{
    uint8_t schema_version;
    uint8_t flags;
    uint8_t target;
    uint8_t effect;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t brightness_percent;
} zy100_feature_config_t;

typedef enum
{
    ZY100_FEATURE_CONFIG_STATUS_OK = 0U,
    ZY100_FEATURE_CONFIG_STATUS_NOT_FOUND,
    ZY100_FEATURE_CONFIG_STATUS_INVALID,
    ZY100_FEATURE_CONFIG_STATUS_READ_ERROR,
    ZY100_FEATURE_CONFIG_STATUS_WRITE_ERROR,
    ZY100_FEATURE_CONFIG_STATUS_VERIFY_ERROR,
} zy100_feature_config_status_t;

void zy100_feature_config_init(void);
void zy100_feature_config_default(zy100_feature_config_t *config);
bool zy100_feature_config_validate(const zy100_feature_config_t *config);
uint8_t zy100_feature_config_speed(const zy100_feature_config_t *config);
uint32_t zy100_feature_config_crc32(uint32_t owner_user_id,
                                    const zy100_feature_config_t *config);
bool zy100_feature_config_get(zy100_feature_config_t *config,
                              uint32_t *owner_user_id,
                              uint32_t *generation);
bool zy100_feature_config_matches(uint32_t owner_user_id,
                                  const zy100_feature_config_t *config,
                                  uint32_t config_crc32);
zy100_feature_config_status_t zy100_feature_config_commit(
    uint32_t owner_user_id,
    const zy100_feature_config_t *config,
    uint32_t config_crc32,
    uint32_t *generation_out);
zy100_feature_config_status_t zy100_feature_config_factory_invalidate(void);
bool zy100_feature_config_auto_capture_enabled(void);
const char *zy100_feature_config_status_name(
    zy100_feature_config_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_FEATURE_CONFIG_H */
