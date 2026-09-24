#include "ble_ota_cmd_protocol.h"

#include <string.h>

#define BLE_OTA_CMD_ENTER_ID       0x44U
#define BLE_OTA_CMD_ACK_ID         0x45U
#define BLE_OTA_CMD_ENTER_VALUE    0x01U
#define BLE_OTA_CMD_ACK_SUCCESS    0x01U
#define BLE_OTA_CMD_ACK_REJECTED   0x00U
#define BLE_OTA_CMD_ACK_RESERVED   0x00U

static const uint8_t s_ble_ota_enter_req[BLE_OTA_CMD_ENTER_REQ_LEN] =
{
    BLE_OTA_CMD_ENTER_ID,
    BLE_OTA_CMD_ENTER_VALUE,
    'Z',
    'Y',
    'O',
    'T',
    'A',
};

bool ble_ota_cmd_is_enter_request(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len != BLE_OTA_CMD_ENTER_REQ_LEN))
    {
        return false;
    }

    return (memcmp(data, s_ble_ota_enter_req, BLE_OTA_CMD_ENTER_REQ_LEN) == 0);
}

bool ble_ota_cmd_build_ack(uint8_t *out, uint16_t out_len, bool accepted)
{
    if ((out == NULL) || (out_len < BLE_OTA_CMD_ACK_LEN))
    {
        return false;
    }

    out[0] = BLE_OTA_CMD_ACK_ID;
    out[1] = accepted ? BLE_OTA_CMD_ACK_SUCCESS : BLE_OTA_CMD_ACK_REJECTED;
    out[2] = BLE_OTA_CMD_ACK_RESERVED;
    return true;
}
