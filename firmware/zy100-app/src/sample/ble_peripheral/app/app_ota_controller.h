#ifndef APP_OTA_CONTROLLER_H
#define APP_OTA_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    APP_OTA_SOURCE_STANDARD_GATT = 0,
    APP_OTA_SOURCE_PRIVATE_V2,
    APP_OTA_SOURCE_PRIVATE_V5,
    APP_OTA_SOURCE_LEGACY,
    APP_OTA_SOURCE_CTRL_COMMIT,
} app_ota_source_t;

typedef enum
{
    APP_OTA_REJECT_NONE = 0,
    APP_OTA_REJECT_NOT_CONNECTED,
    APP_OTA_REJECT_NOT_PAIRED,
    APP_OTA_REJECT_LINK_NOT_ENCRYPTED,
    APP_OTA_REJECT_CAPTURE_BUSY,
    APP_OTA_REJECT_FLASH_BUSY,
    APP_OTA_REJECT_STREAM_BUSY,
    APP_OTA_REJECT_SPOOL_NOT_DRAINED,
    APP_OTA_REJECT_POWER_TRANSITION,
    APP_OTA_REJECT_CALIBRATION_BUSY,
    APP_OTA_REJECT_OTA_ALREADY_PENDING,
    APP_OTA_REJECT_PREPARE_REQUIRED,
} app_ota_reject_reason_t;

void app_ota_controller_init(void);
void app_ota_controller_poll(void);
bool app_ota_controller_transition_pending(void);
uint32_t app_ota_controller_standby_wait_ms(uint32_t wait_ms,
                                             uint64_t runtime_ms);

#endif /* APP_OTA_CONTROLLER_H */
