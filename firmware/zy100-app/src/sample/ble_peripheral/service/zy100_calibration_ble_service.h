#ifndef ZY100_CALIBRATION_BLE_SERVICE_H
#define ZY100_CALIBRATION_BLE_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include <profile_server.h>

#define ZY100_CAL_BLE_NOTIFY_TX_ENABLE       1U
#define ZY100_CAL_BLE_NOTIFY_TX_DISABLE      2U
#define ZY100_CAL_BLE_NOTIFY_STATUS_ENABLE   3U
#define ZY100_CAL_BLE_NOTIFY_STATUS_DISABLE  4U
#define ZY100_CAL_BLE_WRITE_RX               1U

typedef struct
{
    uint8_t opcode;
    T_WRITE_TYPE write_type;
    uint16_t len;
    uint8_t *p_value;
} zy100_cal_ble_write_t;

typedef union
{
    uint8_t notification_index;
    zy100_cal_ble_write_t write;
} zy100_cal_ble_msg_data_t;

typedef struct
{
    uint8_t conn_id;
    T_SERVICE_CALLBACK_TYPE msg_type;
    zy100_cal_ble_msg_data_t msg_data;
} zy100_cal_ble_callback_data_t;

T_SERVER_ID zy100_cal_ble_service_add_service(void *callback);
void zy100_cal_ble_service_set_info(const uint8_t *value, uint16_t len);
void zy100_cal_ble_service_set_status(const uint8_t *value, uint16_t len);
bool zy100_cal_ble_service_tx_notify_enabled(uint8_t conn_id);
bool zy100_cal_ble_service_status_notify_enabled(uint8_t conn_id);
bool zy100_cal_ble_service_send_tx(uint8_t conn_id,
                                   const uint8_t *value,
                                   uint16_t len);
bool zy100_cal_ble_service_send_status(uint8_t conn_id,
                                       const uint8_t *value,
                                       uint16_t len);
bool zy100_cal_ble_service_is_tx_attrib(T_SERVER_ID service_id,
                                        uint16_t attrib_idx);
void zy100_cal_ble_service_reset(uint8_t conn_id);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_CALIBRATION_BLE_SERVICE_H */
