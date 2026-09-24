#ifndef BLE_OTA_CMD_PROTOCOL_H
#define BLE_OTA_CMD_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define BLE_OTA_CMD_ENTER_REQ_LEN 7U
#define BLE_OTA_CMD_ACK_LEN       3U

bool ble_ota_cmd_is_enter_request(const uint8_t *data, uint16_t len);
bool ble_ota_cmd_build_ack(uint8_t *out, uint16_t out_len, bool accepted);

#ifdef __cplusplus
}
#endif

#endif /* BLE_OTA_CMD_PROTOCOL_H */
