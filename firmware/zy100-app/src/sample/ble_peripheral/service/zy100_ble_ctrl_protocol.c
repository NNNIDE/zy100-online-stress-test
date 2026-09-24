#include "zy100_ble_ctrl_protocol.h"

#include <string.h>

#include "../common/zy100_byteorder.h"

typedef char zy100_ble_cmd_frame_size_check[
    (sizeof(zy100_ble_cmd_frame_t) == ZY100_BLE_CMD_FRAME_LEN) ? 1 : -1];
typedef char zy100_ble_ack_frame_size_check[
    (sizeof(zy100_ble_ack_frame_t) == ZY100_BLE_ACK_FRAME_LEN) ? 1 : -1];

bool zy100_ble_ctrl_cmd_supported(uint8_t cmd)
{
    return
#if defined(ZY100_ONLINE_STRESS_TEST_ENABLE) && ZY100_ONLINE_STRESS_TEST_ENABLE
           (cmd == ZY100_BLE_CMD_ONLINE_STRESS) ||
#endif
           (cmd == ZY100_BLE_CMD_START_CAPTURE) ||
           (cmd == ZY100_BLE_CMD_PAUSE_CAPTURE) ||
           (cmd == ZY100_BLE_CMD_CLEAR_FLASH) ||
           (cmd == ZY100_BLE_CMD_TIME_SYNC) ||
           (cmd == ZY100_BLE_CMD_FIND_DEVICE) ||
           (cmd == ZY100_BLE_CMD_HOST_CI_MODE_ENABLE) ||
           (cmd == ZY100_BLE_CMD_HOST_PROFILE_RESULT) ||
           (cmd == ZY100_BLE_CMD_GET_LINK_STATE) ||
           (cmd == ZY100_BLE_CMD_ENTER_SHIPPING) ||
           (cmd == ZY100_BLE_CMD_OTA_PREPARE) ||
           (cmd == ZY100_BLE_CMD_OTA_COMMIT) ||
           (cmd == ZY100_BLE_CMD_OTA_LINK_INTENT) ||
           (cmd == ZY100_BLE_CMD_GET_CONNECTION_STATE) ||
           (cmd == ZY100_BLE_CMD_CONNECTION_USER_SYNC) ||
           (cmd == ZY100_BLE_CMD_TIMEOUT_ACTION_ACK) ||
           (cmd == ZY100_BLE_CMD_FEATURE_CONFIG_SYNC) ||
           (cmd == ZY100_BLE_CMD_EXPORT_CONFIRM) ||
           (cmd == ZY100_BLE_CMD_ONLINE_STREAM_READY) ||
           (cmd == ZY100_BLE_CMD_ONLINE_RECORD_ACK) ||
           (cmd == ZY100_BLE_CMD_OFFLINE_SESSION_LIST) ||
           (cmd == ZY100_BLE_CMD_OFFLINE_SESSION_BEGIN) ||
           (cmd == ZY100_BLE_CMD_OFFLINE_CHUNK_ACK) ||
           (cmd == ZY100_BLE_CMD_OFFLINE_SESSION_RESUME) ||
           (cmd == ZY100_BLE_CMD_OFFLINE_FINAL_CONFIRM) ||
           (cmd == ZY100_BLE_CMD_OFFLINE_RECLAIM_STATUS) ||
           (cmd == ZY100_BLE_CMD_OFFLINE_FOREIGN_PURGE) ||
           (cmd == ZY100_BLE_CMD_OFFLINE_CAPTURE_START) ||
           (cmd == ZY100_BLE_CMD_OFFLINE_CAPTURE_STOP) ||
           (cmd == ZY100_BLE_CMD_TRAINING_SNAPSHOT) ||
           (cmd == ZY100_BLE_CMD_TRAINING_STOP) ||
           (cmd == ZY100_BLE_CMD_PING);
}

const char *zy100_ble_ctrl_cmd_name(uint8_t cmd)
{
    switch (cmd)
    {
#if defined(ZY100_ONLINE_STRESS_TEST_ENABLE) && ZY100_ONLINE_STRESS_TEST_ENABLE
    case ZY100_BLE_CMD_ONLINE_STRESS: return "ONLINE_STRESS";
#endif
    case ZY100_BLE_CMD_START_CAPTURE:
        return "START_CAPTURE";
    case ZY100_BLE_CMD_PAUSE_CAPTURE:
        return "PAUSE_CAPTURE";
    case ZY100_BLE_CMD_CLEAR_FLASH:
        return "CLEAR_FLASH";
    case ZY100_BLE_CMD_TIME_SYNC:
        return "TIME_SYNC";
    case ZY100_BLE_CMD_FIND_DEVICE:
        return "FIND_DEVICE";
    case ZY100_BLE_CMD_HOST_CI_MODE_ENABLE:
        return "HOST_CI_MODE_ENABLE";
    case ZY100_BLE_CMD_HOST_PROFILE_RESULT:
        return "HOST_PROFILE_RESULT";
    case ZY100_BLE_CMD_GET_LINK_STATE:
        return "GET_LINK_STATE";
    case ZY100_BLE_CMD_ENTER_SHIPPING:
        return "ENTER_SHIPPING";
    case ZY100_BLE_CMD_OTA_PREPARE:
        return "OTA_PREPARE";
    case ZY100_BLE_CMD_OTA_COMMIT:
        return "OTA_COMMIT";
    case ZY100_BLE_CMD_OTA_LINK_INTENT:
        return "OTA_LINK_INTENT";
    case ZY100_BLE_CMD_GET_CONNECTION_STATE:
        return "GET_CONNECTION_STATE";
    case ZY100_BLE_CMD_CONNECTION_USER_SYNC:
        return "CONNECTION_USER_SYNC";
    case ZY100_BLE_CMD_TIMEOUT_ACTION_ACK:
        return "TIMEOUT_ACTION_ACK";
    case ZY100_BLE_CMD_TIMEOUT_ACTION_NOTIFY:
        return "TIMEOUT_ACTION_NOTIFY";
    case ZY100_BLE_CMD_FEATURE_CONFIG_SYNC:
        return "FEATURE_CONFIG_SYNC";
    case ZY100_BLE_CMD_CONNECTION_STATE_NOTIFY:
        return "CONNECTION_STATE_NOTIFY";
    case ZY100_BLE_CMD_LINK_STATE_NOTIFY:
        return "LINK_STATE_NOTIFY";
    case ZY100_BLE_CMD_EXPORT_CONFIRM:
        return "EXPORT_CONFIRM";
    case ZY100_BLE_CMD_ONLINE_STREAM_READY:
        return "ONLINE_STREAM_READY";
    case ZY100_BLE_CMD_ONLINE_RECORD_ACK:
        return "ONLINE_RECORD_ACK";
    case ZY100_BLE_CMD_OFFLINE_SESSION_LIST:
        return "OFFLINE_SESSION_LIST";
    case ZY100_BLE_CMD_OFFLINE_SESSION_BEGIN:
        return "OFFLINE_SESSION_BEGIN";
    case ZY100_BLE_CMD_OFFLINE_CHUNK_ACK:
        return "OFFLINE_CHUNK_ACK";
    case ZY100_BLE_CMD_OFFLINE_SESSION_RESUME:
        return "OFFLINE_SESSION_RESUME";
    case ZY100_BLE_CMD_OFFLINE_FINAL_CONFIRM:
        return "OFFLINE_FINAL_CONFIRM";
    case ZY100_BLE_CMD_OFFLINE_RECLAIM_STATUS:
        return "OFFLINE_RECLAIM_STATUS";
    case ZY100_BLE_CMD_OFFLINE_FOREIGN_PURGE:
        return "OFFLINE_FOREIGN_PURGE";
    case ZY100_BLE_CMD_OFFLINE_CAPTURE_START:
        return "OFFLINE_CAPTURE_START";
    case ZY100_BLE_CMD_OFFLINE_CAPTURE_STOP:
        return "OFFLINE_CAPTURE_STOP";
    case ZY100_BLE_CMD_STATE_NOTIFY:
        return "STATE_NOTIFY";
    case ZY100_BLE_CMD_PING:
        return "PING";
    case ZY100_BLE_CMD_TRAINING_SNAPSHOT: return "TRAINING_SNAPSHOT";
    case ZY100_BLE_CMD_TRAINING_STOP: return "TRAINING_STOP";
    default:
        return "UNKNOWN";
    }
}

zy100_ble_ack_status_t zy100_ble_ctrl_parse_command(const uint8_t *data,
                                                    uint16_t len,
                                                    zy100_ble_cmd_frame_t *out)
{
    if (out != NULL)
    {
        memset(out, 0, sizeof(*out));
    }

    if (data == NULL)
    {
        return ZY100_BLE_ACK_STATUS_BAD_LENGTH;
    }

    if ((out != NULL) && (len >= 4U))
    {
        out->magic = data[0];
        out->version = data[1];
        out->cmd = data[2];
        out->seq = data[3];
    }
    if ((out != NULL) && (len >= 8U))
    {
        out->user_id_le = zy100_get_u32_le(&data[4]);
    }
    if ((out != NULL) && (len >= ZY100_BLE_CMD_FRAME_LEN))
    {
        out->device_time_ms_le = zy100_get_u64_le(&data[8]);
        out->training_id_le = zy100_get_u32_le(&data[16]);
    }

    if (len != ZY100_BLE_CMD_FRAME_LEN)
    {
        return ZY100_BLE_ACK_STATUS_BAD_LENGTH;
    }

    if ((data[0] != ZY100_BLE_CMD_MAGIC) || (data[1] != ZY100_BLE_CTRL_VERSION))
    {
        return ZY100_BLE_ACK_STATUS_BAD_MAGIC_OR_VERSION;
    }

    if (!zy100_ble_ctrl_cmd_supported(data[2]))
    {
        return ZY100_BLE_ACK_STATUS_UNSUPPORTED_CMD;
    }

    return ZY100_BLE_ACK_STATUS_OK;
}

void zy100_ble_ctrl_build_ack(const zy100_ble_cmd_frame_t *cmd,
                              zy100_ble_ack_status_t status,
                              zy100_ble_device_state_t device_state,
                              uint32_t detail,
                              zy100_ble_ack_frame_t *ack)
{
    zy100_ble_ctrl_build_ack_ex(cmd,
                                status,
                                device_state,
                                ZY100_BLE_EXEC_MODE_DRY_RUN_NO_ACTION,
                                detail,
                                ack);
}

void zy100_ble_ctrl_build_ack_ex(const zy100_ble_cmd_frame_t *cmd,
                                 zy100_ble_ack_status_t status,
                                 zy100_ble_device_state_t device_state,
                                 zy100_ble_exec_mode_t exec_mode,
                                 uint32_t detail,
                                 zy100_ble_ack_frame_t *ack)
{
    if (ack == NULL)
    {
        return;
    }

    memset(ack, 0, sizeof(*ack));
    ack->magic = ZY100_BLE_ACK_MAGIC;
    ack->version = ZY100_BLE_CTRL_VERSION;
    ack->status = (uint8_t)status;
    ack->device_state = (uint8_t)device_state;
    ack->exec_mode = (uint8_t)exec_mode;
    ack->detail_le = detail;

    if (cmd != NULL)
    {
        ack->cmd_echo = cmd->cmd;
        ack->seq_echo = cmd->seq;
        ack->user_id_echo_le = cmd->user_id_le;
        ack->training_id_echo_le = cmd->training_id_le;
    }
}

bool zy100_ble_ctrl_encode_ack(const zy100_ble_ack_frame_t *ack,
                               uint8_t *out,
                               uint16_t out_len)
{
    if ((ack == NULL) || (out == NULL) || (out_len < ZY100_BLE_ACK_FRAME_LEN))
    {
        return false;
    }

    out[0] = ack->magic;
    out[1] = ack->version;
    out[2] = ack->cmd_echo;
    out[3] = ack->seq_echo;
    out[4] = ack->status;
    out[5] = ack->device_state;
    out[6] = ack->exec_mode;
    out[7] = ack->reserved;
    zy100_put_u32_le(&out[8], ack->user_id_echo_le);
    zy100_put_u32_le(&out[12], ack->training_id_echo_le);
    zy100_put_u32_le(&out[16], ack->detail_le);
    return true;
}
