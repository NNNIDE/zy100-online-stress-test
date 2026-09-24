#ifndef APP_BLE_TRAINING_RECOVERY_H
#define APP_BLE_TRAINING_RECOVERY_H

#include "../service/zy100_training_snapshot.h"
#include "../service/zy100_ble_ctrl_protocol.h"

#define APP_TRAINING_SNAPSHOT_PAGE_COUNT 8U
#define APP_TRAINING_SNAPSHOT_TTL_MS 10000U

/* All calls run on the application task, never on the sampling worker/ISR. */
bool app_ble_training_snapshot_cached(uint32_t generation, uint32_t user_id,
                                      uint32_t request_id);
zy100_ble_ack_status_t app_ble_training_snapshot_page(
    uint32_t generation, uint32_t user_id, uint32_t request_id,
    uint32_t page, uint32_t now_ms, const zy100_training_snapshot_t *fresh,
    uint8_t payload[8], uint32_t *detail);
void app_ble_training_snapshot_reset(void);

#endif
