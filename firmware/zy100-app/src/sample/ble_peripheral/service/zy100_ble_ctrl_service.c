#include "zy100_ble_ctrl_service.h"

#include <string.h>

#include "gatt.h"
#include "trace.h"
#include "version.h"
#include "../app_flags.h"
#include "../common/zy100_offline_v2_contract.h"
#include "zy100_ble_ctrl_protocol.h"
#include "app_ble_notify_tracker.h"
#include "zy100_device_identity.h"
#include "zy100_online_stream.h"

#define ZY100_BLE_CTRL_SERVICE_INDEX              0x00U
#define ZY100_BLE_CTRL_CMD_VALUE_INDEX           0x02U
#define ZY100_BLE_CTRL_ACK_VALUE_INDEX           0x04U
#define ZY100_BLE_CTRL_ACK_CCCD_INDEX            0x05U
#define ZY100_BLE_CTRL_DEVICE_INFO_VALUE_INDEX   0x07U
#define ZY100_BLE_CTRL_EXPORT_VALUE_INDEX        0x09U
#define ZY100_BLE_CTRL_EXPORT_CCCD_INDEX         0x0AU
#define ZY100_BLE_CTRL_DEVICE_INFO_MAX_LEN       192U
#define ZY100_BLE_CTRL_CONN_ID_INVALID           0xFFU
#define ZY100_BLE_CTRL_ACK_QUEUE_SLOTS            8U
#define ZY100_BLE_CTRL_ACK_QUEUE_STATE_LIMIT      5U

/* Attribute-table UUIDs use ATT little-endian byte order. */
static const uint8_t s_zy100_ble_ctrl_service_uuid[16] =
{
    0x00, 0x10, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
    0x4D, 0x4F, 0x9D, 0x4A, 0x00, 0x00, 0xCA, 0x9E
};
static P_FUN_SERVER_GENERAL_CB s_zy100_ble_ctrl_cb = NULL;
static T_SERVER_ID s_zy100_ble_ctrl_service_id = 0xFFU;
static uint8_t s_ack_value[ZY100_BLE_ACK_FRAME_LEN] =
{
    ZY100_BLE_ACK_MAGIC,
    ZY100_BLE_CTRL_VERSION,
    0U,
    0U,
    ZY100_BLE_ACK_STATUS_NOT_READY,
    ZY100_BLE_DEVICE_STATE_WAIT_START,
    ZY100_BLE_EXEC_MODE_DRY_RUN_NO_ACTION,
    0U,
};
static uint16_t s_ack_len = ZY100_BLE_ACK_FRAME_LEN;
static uint8_t s_ack_notify_conn_id = ZY100_BLE_CTRL_CONN_ID_INVALID;
static bool s_ack_notify_enabled = false;
static uint8_t s_device_state_notify_seq = 0U;
static uint8_t s_link_state_notify_seq = 0U;
typedef struct
{
    uint8_t frame[ZY100_BLE_ACK_FRAME_LEN];
} zy100_ble_ctrl_ack_queue_slot_t;
static zy100_ble_ctrl_ack_queue_slot_t
    s_ack_queue[ZY100_BLE_CTRL_ACK_QUEUE_SLOTS];
static uint8_t s_ack_queue_count = 0U;
static uint8_t s_ack_in_flight[ZY100_BLE_ACK_FRAME_LEN];
static bool s_ack_send_in_flight = false;
static uint8_t s_ack_send_conn_id = ZY100_BLE_CTRL_CONN_ID_INVALID;
static uint8_t s_export_notify_conn_id = ZY100_BLE_CTRL_CONN_ID_INVALID;
static bool s_export_notify_enabled = false;
static char s_device_info[ZY100_BLE_CTRL_DEVICE_INFO_MAX_LEN];
static uint16_t s_device_info_len = 0U;

static T_ATTRIB_APPL s_zy100_ble_ctrl_attr_tbl[] =
{
    {
        (ATTRIB_FLAG_VOID | ATTRIB_FLAG_LE),
        {
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),
        },
        UUID_128BIT_SIZE,
        (void *)s_zy100_ble_ctrl_service_uuid,
        GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_INCL,
        {
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            (GATT_CHAR_PROP_WRITE | GATT_CHAR_PROP_WRITE_NO_RSP)
        },
        1,
        NULL,
        GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,
        {
            0x00, 0x10, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
            0x4D, 0x4F, 0x9D, 0x4A, 0x01, 0x00, 0xCA, 0x9E
        },
        0,
        NULL,
        GATT_PERM_WRITE
    },
    {
        ATTRIB_FLAG_VALUE_INCL,
        {
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            (GATT_CHAR_PROP_READ | GATT_CHAR_PROP_NOTIFY)
        },
        1,
        NULL,
        GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,
        {
            0x00, 0x10, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
            0x4D, 0x4F, 0x9D, 0x4A, 0x02, 0x00, 0xCA, 0x9E
        },
        0,
        NULL,
        GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_CCCD_APPL,
        {
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT),
            HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
        },
        2,
        NULL,
        (GATT_PERM_READ | GATT_PERM_WRITE)
    },
    {
        ATTRIB_FLAG_VALUE_INCL,
        {
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_READ
        },
        1,
        NULL,
        GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,
        {
            0x00, 0x10, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
            0x4D, 0x4F, 0x9D, 0x4A, 0x03, 0x00, 0xCA, 0x9E
        },
        0,
        NULL,
        GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_INCL,
        {
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_NOTIFY
        },
        1,
        NULL,
        GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,
        {
            0x00, 0x10, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
            0x4D, 0x4F, 0x9D, 0x4A, 0x04, 0x00, 0xCA, 0x9E
        },
        0,
        NULL,
        GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_CCCD_APPL,
        {
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT),
            HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
        },
        2,
        NULL,
        (GATT_PERM_READ | GATT_PERM_WRITE)
    },
};

static uint16_t zy100_ble_ctrl_append(char *dst,
                                      uint16_t pos,
                                      uint16_t cap,
                                      const char *src)
{
    uint16_t idx = 0U;

    if ((dst == NULL) || (src == NULL) || (pos >= cap))
    {
        return pos;
    }

    while ((src[idx] != '\0') && ((pos + 1U) < cap))
    {
        dst[pos] = src[idx];
        pos++;
        idx++;
    }
    dst[pos] = '\0';
    return pos;
}

static uint16_t zy100_ble_ctrl_append_u32_dec(char *dst,
                                              uint16_t pos,
                                              uint16_t cap,
                                              uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    if ((dst == NULL) || (pos >= cap))
    {
        return pos;
    }

    do
    {
        digits[count] = (char)('0' + (value % 10U));
        count++;
        value /= 10U;
    } while ((value != 0U) && (count < (uint8_t)sizeof(digits)));

    while ((count != 0U) && ((pos + 1U) < cap))
    {
        count--;
        dst[pos] = digits[count];
        pos++;
    }
    dst[pos] = '\0';
    return pos;
}

static void zy100_ble_ctrl_build_device_info(void)
{
    uint16_t pos = 0U;

    pos = zy100_ble_ctrl_append(s_device_info, pos, sizeof(s_device_info), "name=");
    pos = zy100_ble_ctrl_append(s_device_info, pos, sizeof(s_device_info), zy100_device_ble_name());
    pos = zy100_ble_ctrl_append(s_device_info, pos, sizeof(s_device_info), ";sn=");
    pos = zy100_ble_ctrl_append(s_device_info, pos, sizeof(s_device_info), zy100_device_sn());
    pos = zy100_ble_ctrl_append(s_device_info, pos, sizeof(s_device_info), ";proto=1");
    pos = zy100_ble_ctrl_append(s_device_info, pos, sizeof(s_device_info), ";boot=4");
    pos = zy100_ble_ctrl_append(s_device_info, pos, sizeof(s_device_info), ";usrctx=1");
    pos = zy100_ble_ctrl_append(s_device_info, pos, sizeof(s_device_info), ";fcfg=1");
    pos = zy100_ble_ctrl_append(s_device_info, pos, sizeof(s_device_info), ";offobs=1");
    pos = zy100_ble_ctrl_append(s_device_info, pos, sizeof(s_device_info), ";offpurge=1");
    pos = zy100_ble_ctrl_append(s_device_info, pos, sizeof(s_device_info), ";offctl=1;offstop_any=1;offprio=1");
    pos = zy100_ble_ctrl_append(s_device_info, pos, sizeof(s_device_info), ";trsnap=1;trstop=1");
    pos = zy100_ble_ctrl_append(s_device_info, pos, sizeof(s_device_info), ";offmeta=");
    pos = zy100_ble_ctrl_append_u32_dec(
        s_device_info, pos, sizeof(s_device_info),
        ZY100_OFFLINE_V2_MANIFEST_VERSION);
    pos = zy100_ble_ctrl_append(s_device_info, pos, sizeof(s_device_info), ";calcache=1");
    pos = zy100_ble_ctrl_append(s_device_info, pos, sizeof(s_device_info), ";code=");
    pos = zy100_ble_ctrl_append_u32_dec(s_device_info, pos, sizeof(s_device_info), VERSION_CODE);
    pos = zy100_ble_ctrl_append(s_device_info, pos, sizeof(s_device_info), ";fw=");
    pos = zy100_ble_ctrl_append_u32_dec(s_device_info, pos, sizeof(s_device_info), VERSION_MAJOR);
    pos = zy100_ble_ctrl_append(s_device_info, pos, sizeof(s_device_info), ".");
    pos = zy100_ble_ctrl_append_u32_dec(s_device_info, pos, sizeof(s_device_info), VERSION_MINOR);
    pos = zy100_ble_ctrl_append(s_device_info, pos, sizeof(s_device_info), ".");
    pos = zy100_ble_ctrl_append_u32_dec(s_device_info, pos, sizeof(s_device_info), VERSION_REVISION);
    s_device_info_len = pos;
}

static T_APP_RESULT zy100_ble_ctrl_attr_read_cb(uint8_t conn_id,
                                                T_SERVER_ID service_id,
                                                uint16_t attrib_index,
                                                uint16_t offset,
                                                uint16_t *p_length,
                                                uint8_t **pp_value)
{
    uint16_t len;
    uint8_t *value;

    (void)conn_id;
    (void)service_id;

    if ((p_length == NULL) || (pp_value == NULL))
    {
        return APP_RESULT_APP_ERR;
    }

    switch (attrib_index)
    {
    case ZY100_BLE_CTRL_ACK_VALUE_INDEX:
        value = s_ack_value;
        len = s_ack_len;
        break;
    case ZY100_BLE_CTRL_DEVICE_INFO_VALUE_INDEX:
        if (s_device_info_len == 0U)
        {
            zy100_ble_ctrl_build_device_info();
        }
        value = (uint8_t *)s_device_info;
        len = s_device_info_len;
        break;
    default:
        return APP_RESULT_ATTR_NOT_FOUND;
    }

    if (offset > len)
    {
        return APP_RESULT_INVALID_OFFSET;
    }

    *pp_value = value + offset;
    *p_length = (uint16_t)(len - offset);
    return APP_RESULT_SUCCESS;
}

static void zy100_ble_ctrl_write_post_callback(uint8_t conn_id,
                                               T_SERVER_ID service_id,
                                               uint16_t attrib_index,
                                               uint16_t length,
                                               uint8_t *p_value)
{
    (void)conn_id;
    (void)service_id;
    (void)attrib_index;
    (void)length;
    (void)p_value;
}

static T_APP_RESULT zy100_ble_ctrl_attr_write_cb(uint8_t conn_id,
                                                 T_SERVER_ID service_id,
                                                 uint16_t attrib_index,
                                                 T_WRITE_TYPE write_type,
                                                 uint16_t length,
                                                 uint8_t *p_value,
                                                 P_FUN_WRITE_IND_POST_PROC *p_write_post_proc)
{
    zy100_ble_ctrl_callback_data_t callback_data;

    if (p_write_post_proc != NULL)
    {
        *p_write_post_proc = zy100_ble_ctrl_write_post_callback;
    }

    if (attrib_index != ZY100_BLE_CTRL_CMD_VALUE_INDEX)
    {
        return APP_RESULT_ATTR_NOT_FOUND;
    }

    if ((p_value == NULL) && (length != 0U))
    {
        return APP_RESULT_INVALID_VALUE_SIZE;
    }

    memset(&callback_data, 0, sizeof(callback_data));
    callback_data.conn_id = conn_id;
    callback_data.msg_type = SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE;
    callback_data.msg_data.write.opcode = ZY100_BLE_CTRL_WRITE_CMD;
    callback_data.msg_data.write.write_type = write_type;
    callback_data.msg_data.write.len = length;
    callback_data.msg_data.write.p_value = p_value;

    if (s_zy100_ble_ctrl_cb != NULL)
    {
        s_zy100_ble_ctrl_cb(service_id, (void *)&callback_data);
    }

    return APP_RESULT_SUCCESS;
}

static void zy100_ble_ctrl_cccd_update_cb(uint8_t conn_id,
                                          T_SERVER_ID service_id,
                                          uint16_t index,
                                          uint16_t ccc_bits)
{
    zy100_ble_ctrl_callback_data_t callback_data;

    if ((index != ZY100_BLE_CTRL_ACK_CCCD_INDEX) &&
        (index != ZY100_BLE_CTRL_EXPORT_CCCD_INDEX))
    {
        return;
    }

    memset(&callback_data, 0, sizeof(callback_data));
    callback_data.conn_id = conn_id;
    callback_data.msg_type = SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION;
    if (index == ZY100_BLE_CTRL_ACK_CCCD_INDEX)
    {
        if ((ccc_bits & GATT_CLIENT_CHAR_CONFIG_NOTIFY) != 0U)
        {
            s_ack_notify_conn_id = conn_id;
            s_ack_notify_enabled = true;
            callback_data.msg_data.notification_indification_index =
                ZY100_BLE_CTRL_NOTIFY_ACK_ENABLE;
        }
        else
        {
            if (s_ack_notify_conn_id == conn_id)
            {
                s_ack_notify_conn_id = ZY100_BLE_CTRL_CONN_ID_INVALID;
                s_ack_notify_enabled = false;
                s_ack_queue_count = 0U;
                s_ack_send_in_flight = false;
                s_ack_send_conn_id = ZY100_BLE_CTRL_CONN_ID_INVALID;
            }
            callback_data.msg_data.notification_indification_index =
                ZY100_BLE_CTRL_NOTIFY_ACK_DISABLE;
        }
    }
    else
    {
        if ((ccc_bits & GATT_CLIENT_CHAR_CONFIG_NOTIFY) != 0U)
        {
            s_export_notify_conn_id = conn_id;
            s_export_notify_enabled = true;
            callback_data.msg_data.notification_indification_index =
                ZY100_BLE_CTRL_NOTIFY_EXPORT_ENABLE;
        }
        else
        {
            if (s_export_notify_conn_id == conn_id)
            {
                s_export_notify_conn_id = ZY100_BLE_CTRL_CONN_ID_INVALID;
                s_export_notify_enabled = false;
            }
            callback_data.msg_data.notification_indification_index =
                ZY100_BLE_CTRL_NOTIFY_EXPORT_DISABLE;
        }
    }

    if (s_zy100_ble_ctrl_cb != NULL)
    {
        s_zy100_ble_ctrl_cb(service_id, (void *)&callback_data);
    }
}

static const T_FUN_GATT_SERVICE_CBS s_zy100_ble_ctrl_cbs =
{
    zy100_ble_ctrl_attr_read_cb,
    zy100_ble_ctrl_attr_write_cb,
    zy100_ble_ctrl_cccd_update_cb
};

T_SERVER_ID zy100_ble_ctrl_service_add_service(void *p_func)
{
    if (false == server_add_service(&s_zy100_ble_ctrl_service_id,
                                    (uint8_t *)s_zy100_ble_ctrl_attr_tbl,
                                    sizeof(s_zy100_ble_ctrl_attr_tbl),
                                    s_zy100_ble_ctrl_cbs))
    {
        APP_PRINT_ERROR0("zy100_ble_ctrl_service_add_service: fail");
        s_zy100_ble_ctrl_service_id = 0xFFU;
        return s_zy100_ble_ctrl_service_id;
    }

    s_zy100_ble_ctrl_cb = (P_FUN_SERVER_GENERAL_CB)p_func;
    zy100_ble_ctrl_build_device_info();
    return s_zy100_ble_ctrl_service_id;
}

bool zy100_ble_ctrl_service_set_ack_value(const uint8_t *value, uint16_t len)
{
    if ((value == NULL) || (len != ZY100_BLE_ACK_FRAME_LEN))
    {
        return false;
    }

    memcpy(s_ack_value, value, ZY100_BLE_ACK_FRAME_LEN);
    s_ack_len = ZY100_BLE_ACK_FRAME_LEN;
    return true;
}

bool zy100_ble_ctrl_service_ack_notify_enabled(uint8_t conn_id)
{
    return s_ack_notify_enabled && (s_ack_notify_conn_id == conn_id);
}

static bool zy100_ble_ctrl_ack_frame_is_coalescible(const uint8_t *frame)
{
    uint8_t cmd;

    if (frame == NULL) return false;
    cmd = frame[2];
    if (cmd == ZY100_BLE_CMD_CONNECTION_STATE_NOTIFY)
    {
        return (frame[4] == ZY100_BLE_CONNECTION_BOOTSTRAPPING) ||
               (frame[4] == ZY100_BLE_CONNECTION_CONTROL_READY);
    }
    if (cmd == ZY100_BLE_CMD_STATE_NOTIFY)
    {
        return true;
    }
    if (cmd == ZY100_BLE_CMD_LINK_STATE_NOTIFY)
    {
        return (frame[4] != ZY100_BLE_LINK_APPLIED) &&
               (frame[4] != ZY100_BLE_LINK_FAILED) &&
               (frame[4] != ZY100_BLE_LINK_TIMEOUT);
    }
    return false;
}

static void zy100_ble_ctrl_ack_queue_remove(uint8_t index)
{
    if (index >= s_ack_queue_count) return;
    if ((uint8_t)(index + 1U) < s_ack_queue_count)
    {
        memmove(&s_ack_queue[index],
                &s_ack_queue[index + 1U],
                (size_t)(s_ack_queue_count - index - 1U) *
                    sizeof(s_ack_queue[0]));
    }
    s_ack_queue_count--;
}

static bool zy100_ble_ctrl_ack_queue_make_room(bool protected_frame)
{
    uint8_t index;

    if (s_ack_queue_count < ZY100_BLE_CTRL_ACK_QUEUE_SLOTS) return true;
    if (!protected_frame) return false;
    for (index = 0U; index < s_ack_queue_count; index++)
    {
        if (zy100_ble_ctrl_ack_frame_is_coalescible(s_ack_queue[index].frame))
        {
            zy100_ble_ctrl_ack_queue_remove(index);
            return true;
        }
    }
    return false;
}

static uint8_t zy100_ble_ctrl_ack_queue_state_count(void)
{
    uint8_t index;
    uint8_t count = 0U;

    for (index = 0U; index < s_ack_queue_count; index++)
    {
        if (zy100_ble_ctrl_ack_frame_is_coalescible(s_ack_queue[index].frame))
        {
            count++;
        }
    }
    return count;
}

static bool zy100_ble_ctrl_ack_frame_is_duplicate_terminal(
    const uint8_t *queued,
    const uint8_t *candidate)
{
    uint8_t cmd;

    if ((queued == NULL) || (candidate == NULL) ||
        (queued[2] != candidate[2]))
    {
        return false;
    }
    cmd = candidate[2];
    if ((cmd != ZY100_BLE_CMD_LINK_STATE_NOTIFY) &&
        (cmd != ZY100_BLE_CMD_STATE_NOTIFY))
    {
        return false;
    }
    if (zy100_ble_ctrl_ack_frame_is_coalescible(candidate)) return false;
    return memcmp(&queued[4], &candidate[4],
                  ZY100_BLE_ACK_FRAME_LEN - 4U) == 0;
}

static bool zy100_ble_ctrl_ack_queue_kick(uint8_t conn_id)
{
    bool sent;

    if (s_ack_send_in_flight || (s_ack_queue_count == 0U) ||
        !zy100_ble_ctrl_service_ack_notify_enabled(conn_id))
    {
        return s_ack_send_in_flight || (s_ack_queue_count != 0U);
    }
    memcpy(s_ack_in_flight,
           s_ack_queue[0].frame,
           ZY100_BLE_ACK_FRAME_LEN);
    sent = server_send_data(conn_id,
                            s_zy100_ble_ctrl_service_id,
                            ZY100_BLE_CTRL_ACK_VALUE_INDEX,
                            s_ack_in_flight,
                            ZY100_BLE_ACK_FRAME_LEN,
                            GATT_PDU_TYPE_NOTIFICATION);
    if (!sent) return false;
    zy100_ble_ctrl_ack_queue_remove(0U);
    s_ack_send_in_flight = true;
    s_ack_send_conn_id = conn_id;
    app_ble_notify_tracker_note_submit(conn_id);
    return true;
}

bool zy100_ble_ctrl_service_pump_ack(uint8_t conn_id)
{
    if ((conn_id == ZY100_BLE_CTRL_CONN_ID_INVALID) ||
        !zy100_ble_ctrl_service_ack_notify_enabled(conn_id) ||
        s_ack_send_in_flight || (s_ack_queue_count == 0U))
    {
        return false;
    }
    /* Only a successful SDK submission is true; this is not business completion. */
    return zy100_ble_ctrl_ack_queue_kick(conn_id);
}

bool zy100_ble_ctrl_service_send_ack_notify(uint8_t conn_id,
                                            uint8_t *value,
                                            uint16_t len)
{
    uint8_t index;
    bool coalescible;

    if ((!zy100_ble_ctrl_service_ack_notify_enabled(conn_id)) ||
        (value == NULL) ||
        (len != ZY100_BLE_ACK_FRAME_LEN))
    {
        return false;
    }

    coalescible = zy100_ble_ctrl_ack_frame_is_coalescible(value);
    if (!coalescible)
    {
        for (index = 0U; index < s_ack_queue_count; index++)
        {
            if (zy100_ble_ctrl_ack_frame_is_duplicate_terminal(
                    s_ack_queue[index].frame, value))
            {
                memcpy(s_ack_queue[index].frame,
                       value,
                       ZY100_BLE_ACK_FRAME_LEN);
                (void)zy100_ble_ctrl_ack_queue_kick(conn_id);
                return true;
            }
        }
    }
    if (coalescible)
    {
        for (index = 0U; index < s_ack_queue_count; index++)
        {
            if ((s_ack_queue[index].frame[2] == value[2]) &&
                zy100_ble_ctrl_ack_frame_is_coalescible(
                    s_ack_queue[index].frame))
            {
                memcpy(s_ack_queue[index].frame,
                       value,
                       ZY100_BLE_ACK_FRAME_LEN);
                (void)zy100_ble_ctrl_ack_queue_kick(conn_id);
                return true;
            }
        }
        if (zy100_ble_ctrl_ack_queue_state_count() >=
            ZY100_BLE_CTRL_ACK_QUEUE_STATE_LIMIT)
        {
            return false;
        }
    }
    if (!zy100_ble_ctrl_ack_queue_make_room(!coalescible)) return false;
    memcpy(s_ack_queue[s_ack_queue_count].frame,
           value,
           ZY100_BLE_ACK_FRAME_LEN);
    s_ack_queue_count++;
    (void)zy100_ble_ctrl_ack_queue_kick(conn_id);
    return true;
}

bool zy100_ble_ctrl_service_export_notify_enabled(uint8_t conn_id)
{
    return s_export_notify_enabled && (s_export_notify_conn_id == conn_id);
}

bool zy100_ble_ctrl_service_send_export_notify(uint8_t conn_id,
                                               uint8_t *value,
                                               uint16_t len)
{
    bool sent;

    if ((!zy100_ble_ctrl_service_export_notify_enabled(conn_id)) ||
        (value == NULL) ||
        (len == 0U))
    {
        return false;
    }

    sent = server_send_data(conn_id,
                            s_zy100_ble_ctrl_service_id,
                            ZY100_BLE_CTRL_EXPORT_VALUE_INDEX,
                            value,
                            len,
                            GATT_PDU_TYPE_NOTIFICATION);
    if (sent)
    {
        app_ble_notify_tracker_note_submit(conn_id);
    }
    return sent;
}

bool zy100_ble_ctrl_service_is_export_attrib(T_SERVER_ID service_id,
                                             uint16_t attrib_idx)
{
    return (service_id == s_zy100_ble_ctrl_service_id) &&
           (attrib_idx == ZY100_BLE_CTRL_EXPORT_VALUE_INDEX);
}

bool zy100_ble_ctrl_service_is_ack_attrib(T_SERVER_ID service_id,
                                          uint16_t attrib_idx)
{
    return (service_id == s_zy100_ble_ctrl_service_id) &&
           (attrib_idx == ZY100_BLE_CTRL_ACK_VALUE_INDEX);
}

void zy100_ble_ctrl_service_on_ack_send_complete(uint8_t conn_id,
                                                 uint16_t cause)
{
    (void)cause;
    if (!s_ack_send_in_flight || (s_ack_send_conn_id != conn_id)) return;
    s_ack_send_in_flight = false;
    s_ack_send_conn_id = ZY100_BLE_CTRL_CONN_ID_INVALID;
    (void)zy100_ble_ctrl_ack_queue_kick(conn_id);
}

bool zy100_ble_ctrl_service_ack_pipeline_idle(void)
{
    return !s_ack_send_in_flight && (s_ack_queue_count == 0U);
}

bool zy100_ble_ctrl_service_notify_async_result(uint8_t cmd,
                                                uint8_t seq,
                                                uint8_t status,
                                                uint8_t device_state,
                                                uint8_t exec_mode,
                                                uint32_t user_id,
                                                uint32_t training_id,
                                                uint32_t detail)
{
    return zy100_ble_ctrl_service_notify_async_result_ex(
        s_ack_notify_conn_id,
        cmd,
        seq,
        status,
        device_state,
        exec_mode,
        0U,
        user_id,
        training_id,
        detail);
}

bool zy100_ble_ctrl_service_notify_async_result_ex(uint8_t conn_id,
                                                   uint8_t cmd,
                                                   uint8_t seq,
                                                   uint8_t status,
                                                   uint8_t device_state,
                                                   uint8_t exec_mode,
                                                   uint8_t reserved,
                                                   uint32_t user_id,
                                                   uint32_t training_id,
                                                   uint32_t detail)
{
    zy100_ble_cmd_frame_t cmd_frame;
    zy100_ble_ack_frame_t ack;
    uint8_t ack_bytes[ZY100_BLE_ACK_FRAME_LEN];
    bool notify_sent = false;

    memset(&cmd_frame, 0, sizeof(cmd_frame));
    cmd_frame.magic = ZY100_BLE_CMD_MAGIC;
    cmd_frame.version = ZY100_BLE_CTRL_VERSION;
    cmd_frame.cmd = cmd;
    cmd_frame.seq = seq;
    cmd_frame.user_id_le = user_id;
    cmd_frame.training_id_le = training_id;

    zy100_ble_ctrl_build_ack_ex(&cmd_frame,
                                (zy100_ble_ack_status_t)status,
                                (zy100_ble_device_state_t)device_state,
                                (zy100_ble_exec_mode_t)exec_mode,
                                detail,
                                &ack);
    ack.reserved = reserved;
    if (!zy100_ble_ctrl_encode_ack(&ack, ack_bytes, sizeof(ack_bytes)))
    {
        return false;
    }

    (void)zy100_ble_ctrl_service_set_ack_value(ack_bytes, sizeof(ack_bytes));
    if ((conn_id != ZY100_BLE_CTRL_CONN_ID_INVALID) &&
        (conn_id == s_ack_notify_conn_id) &&
        zy100_ble_ctrl_service_ack_notify_enabled(conn_id))
    {
        notify_sent = zy100_ble_ctrl_service_send_ack_notify(conn_id,
                                                             ack_bytes,
                                                             sizeof(ack_bytes));
    }

#if ZY100_BLE_CTRL_ACK_LOG_ENABLE
    if (!((status == ZY100_BLE_ACK_STATUS_OK) &&
          zy100_online_stream_quiet_logs_active() &&
          !ZY100_ONLINE_STREAM_VERBOSE_TRACE_ENABLE))
    {
        DBG_DIRECT("[BLE_ACK] cmd=%s seq=%u status=0x%02X device_state=0x%02X exec_mode=0x%02X notify=%u",
                   zy100_ble_ctrl_cmd_name(cmd),
                   seq,
                   status,
                   device_state,
                   exec_mode,
                   notify_sent ? 1U : 0U);
    }
#else
    if (status != ZY100_BLE_ACK_STATUS_OK)
    {
        DBG_DIRECT("[BLE_ACK][ERR] cmd=%u seq=%u status=0x%02X state=0x%02X",
                   (uint32_t)cmd,
                   (uint32_t)seq,
                   status,
                   device_state);
    }
#endif
    return notify_sent;
}

bool zy100_ble_ctrl_service_notify_timeout_action(uint8_t conn_id,
    uint8_t device_state, uint32_t generation, uint32_t token, uint8_t reason)
{
    zy100_ble_cmd_frame_t cmd;
    zy100_ble_ack_frame_t ack;
    uint8_t bytes[ZY100_BLE_ACK_FRAME_LEN];

    if ((conn_id != s_ack_notify_conn_id) ||
        !zy100_ble_ctrl_service_ack_notify_enabled(conn_id)) return false;
    memset(&cmd, 0, sizeof(cmd));
    cmd.magic = ZY100_BLE_CMD_MAGIC;
    cmd.version = ZY100_BLE_CTRL_VERSION;
    cmd.cmd = ZY100_BLE_CMD_TIMEOUT_ACTION_NOTIFY;
    cmd.seq = (uint8_t)token;
    cmd.user_id_le = generation;
    cmd.training_id_le = token;
    zy100_ble_ctrl_build_ack_ex(&cmd, ZY100_BLE_ACK_STATUS_OK,
        (zy100_ble_device_state_t)device_state,
        ZY100_BLE_EXEC_MODE_DRY_RUN_NO_ACTION, reason, &ack);
    if (!zy100_ble_ctrl_encode_ack(&ack, bytes, sizeof(bytes))) return false;
    return zy100_ble_ctrl_service_send_ack_notify(conn_id, bytes, sizeof(bytes));
}

bool zy100_ble_ctrl_service_notify_device_state(uint8_t device_state)
{
    return zy100_ble_ctrl_service_notify_device_state_ex(device_state, 0U);
}

bool zy100_ble_ctrl_service_notify_device_state_ex(uint8_t device_state,
                                                   uint32_t detail)
{
    zy100_ble_cmd_frame_t cmd_frame;
    zy100_ble_ack_frame_t ack;
    uint8_t ack_bytes[ZY100_BLE_ACK_FRAME_LEN];
    bool notify_sent;

    if (device_state == ZY100_BLE_DEVICE_STATE_UNKNOWN)
    {
        return false;
    }

    if ((s_ack_notify_conn_id == ZY100_BLE_CTRL_CONN_ID_INVALID) ||
        (!zy100_ble_ctrl_service_ack_notify_enabled(s_ack_notify_conn_id)))
    {
        return false;
    }

    memset(&cmd_frame, 0, sizeof(cmd_frame));
    cmd_frame.magic = ZY100_BLE_CMD_MAGIC;
    cmd_frame.version = ZY100_BLE_CTRL_VERSION;
    cmd_frame.cmd = ZY100_BLE_CMD_STATE_NOTIFY;
    cmd_frame.seq = s_device_state_notify_seq;

    zy100_ble_ctrl_build_ack_ex(&cmd_frame,
                                ZY100_BLE_ACK_STATUS_OK,
                                (zy100_ble_device_state_t)device_state,
                                ZY100_BLE_EXEC_MODE_DRY_RUN_NO_ACTION,
                                detail,
                                &ack);
    if (!zy100_ble_ctrl_encode_ack(&ack, ack_bytes, sizeof(ack_bytes)))
    {
        return false;
    }

    (void)zy100_ble_ctrl_service_set_ack_value(ack_bytes, sizeof(ack_bytes));
    notify_sent = zy100_ble_ctrl_service_send_ack_notify(s_ack_notify_conn_id,
                                                         ack_bytes,
                                                         sizeof(ack_bytes));
    if (notify_sent)
    {
        s_device_state_notify_seq++;
    }

#if ZY100_BLE_STATE_LOG_ENABLE
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[BLE_STATE] notify seq=%u device_state=0x%02X detail=0x%08lX sent=%u",
               cmd_frame.seq,
               device_state,
               (unsigned long)detail,
               notify_sent ? 1U : 0U);
#endif
    return notify_sent;
}

bool zy100_ble_ctrl_service_notify_link_state(uint32_t session_id,
                                              uint32_t generation,
                                              uint32_t transition_id,
                                              uint8_t business_state,
                                              uint8_t expected_profile,
                                              uint8_t link_state,
                                              uint8_t initiator_mode,
                                              uint16_t actual_ci,
                                              uint16_t actual_latency)
{
    zy100_ble_cmd_frame_t cmd_frame;
    zy100_ble_ack_frame_t ack;
    uint8_t ack_bytes[ZY100_BLE_ACK_FRAME_LEN];
    bool sent;

    if ((s_ack_notify_conn_id == ZY100_BLE_CTRL_CONN_ID_INVALID) ||
        !zy100_ble_ctrl_service_ack_notify_enabled(s_ack_notify_conn_id))
    {
        return false;
    }
    memset(&cmd_frame, 0, sizeof(cmd_frame));
    cmd_frame.magic = ZY100_BLE_CMD_MAGIC;
    cmd_frame.version = ZY100_BLE_CTRL_VERSION;
    cmd_frame.cmd = ZY100_BLE_CMD_LINK_STATE_NOTIFY;
    cmd_frame.seq = s_link_state_notify_seq;
    cmd_frame.user_id_le = session_id;
    cmd_frame.training_id_le =
        (generation & 0xFFFFUL) | ((transition_id & 0xFFFFUL) << 16);
    zy100_ble_ctrl_build_ack_ex(
        &cmd_frame,
        (zy100_ble_ack_status_t)link_state,
        (zy100_ble_device_state_t)business_state,
        (zy100_ble_exec_mode_t)expected_profile,
        ((uint32_t)actual_ci) | ((uint32_t)actual_latency << 16),
        &ack);
    ack.reserved = initiator_mode;
    if (!zy100_ble_ctrl_encode_ack(&ack, ack_bytes, sizeof(ack_bytes)))
    {
        return false;
    }
    (void)zy100_ble_ctrl_service_set_ack_value(ack_bytes, sizeof(ack_bytes));
    sent = zy100_ble_ctrl_service_send_ack_notify(s_ack_notify_conn_id,
                                                  ack_bytes,
                                                  sizeof(ack_bytes));
    if (sent)
    {
        s_link_state_notify_seq++;
    }
#if ZY100_BLE_STATE_LOG_ENABLE
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[BLE_CI_HOST] notify seq=%u gen=%lu trans=%lu session=%lu state=%u profile=%u phase=%u ci=%u lat=%u sent=%u",
               cmd_frame.seq,
               (unsigned long)generation,
               (unsigned long)transition_id,
               (unsigned long)session_id,
               business_state,
               expected_profile,
               link_state,
               actual_ci,
               actual_latency,
               sent ? 1U : 0U);
#endif
    return sent;
}

bool zy100_ble_ctrl_service_notify_connection_state(uint8_t revision,
                                                    uint8_t overall_status,
                                                    uint8_t device_state,
                                                    uint8_t suggested_action,
                                                    uint32_t generation,
                                                    uint32_t ready_bits,
                                                    uint32_t detail)
{
    zy100_ble_cmd_frame_t cmd_frame;
    zy100_ble_ack_frame_t ack;
    uint8_t ack_bytes[ZY100_BLE_ACK_FRAME_LEN];

    if ((s_ack_notify_conn_id == ZY100_BLE_CTRL_CONN_ID_INVALID) ||
        !zy100_ble_ctrl_service_ack_notify_enabled(s_ack_notify_conn_id))
    {
        return false;
    }
    memset(&cmd_frame, 0, sizeof(cmd_frame));
    cmd_frame.magic = ZY100_BLE_CMD_MAGIC;
    cmd_frame.version = ZY100_BLE_CTRL_VERSION;
    cmd_frame.cmd = ZY100_BLE_CMD_CONNECTION_STATE_NOTIFY;
    cmd_frame.seq = revision;
    cmd_frame.user_id_le = generation;
    cmd_frame.training_id_le = ready_bits;
    zy100_ble_ctrl_build_ack_ex(
        &cmd_frame,
        (zy100_ble_ack_status_t)overall_status,
        (zy100_ble_device_state_t)device_state,
        (zy100_ble_exec_mode_t)suggested_action,
        detail,
        &ack);
    ack.reserved = ZY100_BLE_BOOTSTRAP_VERSION;
    if (!zy100_ble_ctrl_encode_ack(&ack, ack_bytes, sizeof(ack_bytes)))
    {
        return false;
    }
    (void)zy100_ble_ctrl_service_set_ack_value(ack_bytes, sizeof(ack_bytes));
    return zy100_ble_ctrl_service_send_ack_notify(s_ack_notify_conn_id,
                                                  ack_bytes,
                                                  sizeof(ack_bytes));
}

void zy100_ble_ctrl_service_reset_notify_state(uint8_t conn_id)
{
    if (s_ack_notify_conn_id == conn_id)
    {
        s_ack_notify_conn_id = ZY100_BLE_CTRL_CONN_ID_INVALID;
        s_ack_notify_enabled = false;
        s_ack_queue_count = 0U;
        s_ack_send_in_flight = false;
        s_ack_send_conn_id = ZY100_BLE_CTRL_CONN_ID_INVALID;
    }
    if (s_export_notify_conn_id == conn_id)
    {
        s_export_notify_conn_id = ZY100_BLE_CTRL_CONN_ID_INVALID;
        s_export_notify_enabled = false;
    }
}
