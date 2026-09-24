#include "zy100_calibration_ble_service.h"

#include <string.h>

#include "gatt.h"
#include "trace.h"
#include "app_ble_notify_tracker.h"
#include "zy100_calibration_protocol.h"
#include "zy100_whole_unit_test.h"

#define ZY100_CAL_BLE_INFO_VALUE_INDEX        0x02U
#define ZY100_CAL_BLE_TX_VALUE_INDEX          0x04U
#define ZY100_CAL_BLE_TX_CCCD_INDEX           0x05U
#define ZY100_CAL_BLE_RX_VALUE_INDEX          0x07U
#define ZY100_CAL_BLE_STATUS_VALUE_INDEX      0x09U
#define ZY100_CAL_BLE_STATUS_CCCD_INDEX       0x0AU
#define ZY100_CAL_BLE_CONN_INVALID            0xFFU

static const uint8_t s_cal_service_uuid[16] =
{
    0x00, 0x10, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
    0x4D, 0x4F, 0x9D, 0x4A, 0x00, 0x30, 0xCA, 0x9E
};

static P_FUN_SERVER_GENERAL_CB s_cal_callback;
static T_SERVER_ID s_cal_service_id = 0xFFU;
static uint8_t s_info[ZY100_CAL_INFO_BYTES];
static uint8_t s_status[ZY100_CAL_STATUS_BYTES];
static uint8_t s_tx_conn_id = ZY100_CAL_BLE_CONN_INVALID;
static uint8_t s_status_conn_id = ZY100_CAL_BLE_CONN_INVALID;
static bool s_tx_enabled;
static bool s_status_enabled;

static T_ATTRIB_APPL s_cal_attr_tbl[] =
{
    {
        (ATTRIB_FLAG_VOID | ATTRIB_FLAG_LE),
        {LO_WORD(GATT_UUID_PRIMARY_SERVICE), HI_WORD(GATT_UUID_PRIMARY_SERVICE)},
        UUID_128BIT_SIZE, (void *)s_cal_service_uuid, GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_INCL,
        {LO_WORD(GATT_UUID_CHARACTERISTIC), HI_WORD(GATT_UUID_CHARACTERISTIC),
         GATT_CHAR_PROP_READ},
        1, NULL, GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,
        {0x00, 0x10, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
         0x4D, 0x4F, 0x9D, 0x4A, 0x01, 0x30, 0xCA, 0x9E},
        0, NULL, GATT_PERM_READ_ENCRYPTED_REQ
    },
    {
        ATTRIB_FLAG_VALUE_INCL,
        {LO_WORD(GATT_UUID_CHARACTERISTIC), HI_WORD(GATT_UUID_CHARACTERISTIC),
         GATT_CHAR_PROP_NOTIFY},
        1, NULL, GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,
        {0x00, 0x10, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
         0x4D, 0x4F, 0x9D, 0x4A, 0x02, 0x30, 0xCA, 0x9E},
        0, NULL, GATT_PERM_NOTIF_IND_ENCRYPTED_REQ
    },
    {
        ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_CCCD_APPL,
        {LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG), HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
         LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT), HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)},
        2, NULL, (GATT_PERM_READ_ENCRYPTED_REQ |
                  GATT_PERM_WRITE_ENCRYPTED_REQ)
    },
    {
        ATTRIB_FLAG_VALUE_INCL,
        {LO_WORD(GATT_UUID_CHARACTERISTIC), HI_WORD(GATT_UUID_CHARACTERISTIC),
         GATT_CHAR_PROP_WRITE},
        1, NULL, GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,
        {0x00, 0x10, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
         0x4D, 0x4F, 0x9D, 0x4A, 0x03, 0x30, 0xCA, 0x9E},
        0, NULL, GATT_PERM_WRITE_ENCRYPTED_REQ
    },
    {
        ATTRIB_FLAG_VALUE_INCL,
        {LO_WORD(GATT_UUID_CHARACTERISTIC), HI_WORD(GATT_UUID_CHARACTERISTIC),
         (GATT_CHAR_PROP_READ | GATT_CHAR_PROP_NOTIFY)},
        1, NULL, GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,
        {0x00, 0x10, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
         0x4D, 0x4F, 0x9D, 0x4A, 0x04, 0x30, 0xCA, 0x9E},
        0, NULL, (GATT_PERM_READ_ENCRYPTED_REQ |
                  GATT_PERM_NOTIF_IND_ENCRYPTED_REQ)
    },
    {
        ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_CCCD_APPL,
        {LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG), HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
         LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT), HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)},
        2, NULL, (GATT_PERM_READ_ENCRYPTED_REQ |
                  GATT_PERM_WRITE_ENCRYPTED_REQ)
    },
};

static T_APP_RESULT zy100_cal_ble_read_cb(uint8_t conn_id,
                                          T_SERVER_ID service_id,
                                          uint16_t index,
                                          uint16_t offset,
                                          uint16_t *length,
                                          uint8_t **value)
{
    uint8_t *src;
    uint16_t src_len;
    (void)conn_id;
    (void)service_id;

    if ((length == NULL) || (value == NULL))
    {
        return APP_RESULT_APP_ERR;
    }
    if (index == ZY100_CAL_BLE_INFO_VALUE_INDEX)
    {
        src = s_info;
        src_len = sizeof(s_info);
    }
    else if (index == ZY100_CAL_BLE_STATUS_VALUE_INDEX)
    {
        src = s_status;
        src_len = sizeof(s_status);
    }
    else
    {
        return APP_RESULT_ATTR_NOT_FOUND;
    }
    if (offset > src_len)
    {
        return APP_RESULT_INVALID_OFFSET;
    }
    *value = src + offset;
    *length = (uint16_t)(src_len - offset);
    return APP_RESULT_SUCCESS;
}

static T_APP_RESULT zy100_cal_ble_write_cb(uint8_t conn_id,
                                           T_SERVER_ID service_id,
                                           uint16_t index,
                                           T_WRITE_TYPE write_type,
                                           uint16_t length,
                                           uint8_t *value,
                                           P_FUN_WRITE_IND_POST_PROC *post)
{
    zy100_cal_ble_callback_data_t data;
    (void)post;

    if (zy100_whole_unit_first_user_boot_pending())
    {
        return APP_RESULT_APP_ERR;
    }
    if (index != ZY100_CAL_BLE_RX_VALUE_INDEX)
    {
        return APP_RESULT_ATTR_NOT_FOUND;
    }
    if ((value == NULL) || (length == 0U))
    {
        return APP_RESULT_INVALID_VALUE_SIZE;
    }
    memset(&data, 0, sizeof(data));
    data.conn_id = conn_id;
    data.msg_type = SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE;
    data.msg_data.write.opcode = ZY100_CAL_BLE_WRITE_RX;
    data.msg_data.write.write_type = write_type;
    data.msg_data.write.len = length;
    data.msg_data.write.p_value = value;
    if (s_cal_callback != NULL)
    {
        s_cal_callback(service_id, &data);
    }
    return APP_RESULT_SUCCESS;
}

static void zy100_cal_ble_cccd_cb(uint8_t conn_id,
                                  T_SERVER_ID service_id,
                                  uint16_t index,
                                  uint16_t bits)
{
    zy100_cal_ble_callback_data_t data;
    memset(&data, 0, sizeof(data));
    data.conn_id = conn_id;
    data.msg_type = SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION;

    if (index == ZY100_CAL_BLE_TX_CCCD_INDEX)
    {
        s_tx_enabled = (bits & GATT_CLIENT_CHAR_CONFIG_NOTIFY) != 0U;
        s_tx_conn_id = s_tx_enabled ? conn_id : ZY100_CAL_BLE_CONN_INVALID;
        data.msg_data.notification_index = s_tx_enabled ?
            ZY100_CAL_BLE_NOTIFY_TX_ENABLE : ZY100_CAL_BLE_NOTIFY_TX_DISABLE;
    }
    else if (index == ZY100_CAL_BLE_STATUS_CCCD_INDEX)
    {
        s_status_enabled = (bits & GATT_CLIENT_CHAR_CONFIG_NOTIFY) != 0U;
        s_status_conn_id = s_status_enabled ? conn_id : ZY100_CAL_BLE_CONN_INVALID;
        data.msg_data.notification_index = s_status_enabled ?
            ZY100_CAL_BLE_NOTIFY_STATUS_ENABLE : ZY100_CAL_BLE_NOTIFY_STATUS_DISABLE;
    }
    else
    {
        return;
    }
    if (s_cal_callback != NULL)
    {
        s_cal_callback(service_id, &data);
    }
}

static const T_FUN_GATT_SERVICE_CBS s_cal_cbs =
{
    zy100_cal_ble_read_cb,
    zy100_cal_ble_write_cb,
    zy100_cal_ble_cccd_cb
};

T_SERVER_ID zy100_cal_ble_service_add_service(void *callback)
{
    if (!server_add_service(&s_cal_service_id,
                            (uint8_t *)s_cal_attr_tbl,
                            sizeof(s_cal_attr_tbl), s_cal_cbs))
    {
        s_cal_service_id = 0xFFU;
        return s_cal_service_id;
    }
    s_cal_callback = (P_FUN_SERVER_GENERAL_CB)callback;
    return s_cal_service_id;
}

void zy100_cal_ble_service_set_info(const uint8_t *value, uint16_t len)
{
    if ((value != NULL) && (len == sizeof(s_info)))
    {
        memcpy(s_info, value, sizeof(s_info));
    }
}

void zy100_cal_ble_service_set_status(const uint8_t *value, uint16_t len)
{
    if ((value != NULL) && (len == sizeof(s_status)))
    {
        memcpy(s_status, value, sizeof(s_status));
    }
}

bool zy100_cal_ble_service_tx_notify_enabled(uint8_t conn_id)
{
    return s_tx_enabled && (s_tx_conn_id == conn_id);
}

bool zy100_cal_ble_service_status_notify_enabled(uint8_t conn_id)
{
    return s_status_enabled && (s_status_conn_id == conn_id);
}

bool zy100_cal_ble_service_send_tx(uint8_t conn_id,
                                   const uint8_t *value,
                                   uint16_t len)
{
    bool sent = false;

    if (zy100_cal_ble_service_tx_notify_enabled(conn_id) &&
        (value != NULL) && (len != 0U))
    {
        sent = server_send_data(conn_id, s_cal_service_id,
                                ZY100_CAL_BLE_TX_VALUE_INDEX,
                                (uint8_t *)value, len,
                                GATT_PDU_TYPE_NOTIFICATION);
        if (sent)
        {
            app_ble_notify_tracker_note_submit(conn_id);
        }
    }
    return sent;
}

bool zy100_cal_ble_service_send_status(uint8_t conn_id,
                                       const uint8_t *value,
                                       uint16_t len)
{
    bool sent = false;

    if (zy100_cal_ble_service_status_notify_enabled(conn_id) &&
        (value != NULL) && (len == sizeof(s_status)))
    {
        sent = server_send_data(conn_id, s_cal_service_id,
                                ZY100_CAL_BLE_STATUS_VALUE_INDEX,
                                (uint8_t *)value, len,
                                GATT_PDU_TYPE_NOTIFICATION);
        if (sent)
        {
            app_ble_notify_tracker_note_submit(conn_id);
        }
    }
    return sent;
}

bool zy100_cal_ble_service_is_tx_attrib(T_SERVER_ID service_id,
                                        uint16_t attrib_idx)
{
    return (service_id == s_cal_service_id) &&
           (attrib_idx == ZY100_CAL_BLE_TX_VALUE_INDEX);
}

void zy100_cal_ble_service_reset(uint8_t conn_id)
{
    if (s_tx_conn_id == conn_id)
    {
        s_tx_conn_id = ZY100_CAL_BLE_CONN_INVALID;
        s_tx_enabled = false;
    }
    if (s_status_conn_id == conn_id)
    {
        s_status_conn_id = ZY100_CAL_BLE_CONN_INVALID;
        s_status_enabled = false;
    }
}
