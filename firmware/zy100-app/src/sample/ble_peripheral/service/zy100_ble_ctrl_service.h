#ifndef ZY100_BLE_CTRL_SERVICE_H
#define ZY100_BLE_CTRL_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include <profile_server.h>

#define ZY100_BLE_CTRL_NOTIFY_ACK_ENABLE  1U
#define ZY100_BLE_CTRL_NOTIFY_ACK_DISABLE 2U
#define ZY100_BLE_CTRL_NOTIFY_EXPORT_ENABLE  3U
#define ZY100_BLE_CTRL_NOTIFY_EXPORT_DISABLE 4U
#define ZY100_BLE_CTRL_WRITE_CMD          1U

typedef struct
{
    uint8_t opcode;
    T_WRITE_TYPE write_type;
    uint16_t len;
    uint8_t *p_value;
} zy100_ble_ctrl_write_msg_t;

typedef union
{
    uint8_t notification_indification_index;
    zy100_ble_ctrl_write_msg_t write;
} zy100_ble_ctrl_msg_data_t;

typedef struct
{
    uint8_t conn_id;
    T_SERVICE_CALLBACK_TYPE msg_type;
    zy100_ble_ctrl_msg_data_t msg_data;
} zy100_ble_ctrl_callback_data_t;

T_SERVER_ID zy100_ble_ctrl_service_add_service(void *p_func);
bool zy100_ble_ctrl_service_set_ack_value(const uint8_t *value, uint16_t len);
bool zy100_ble_ctrl_service_ack_notify_enabled(uint8_t conn_id);
bool zy100_ble_ctrl_service_send_ack_notify(uint8_t conn_id,
                                            uint8_t *value,
                                            uint16_t len);
bool zy100_ble_ctrl_service_export_notify_enabled(uint8_t conn_id);
bool zy100_ble_ctrl_service_send_export_notify(uint8_t conn_id,
                                               uint8_t *value,
                                               uint16_t len);
bool zy100_ble_ctrl_service_is_export_attrib(T_SERVER_ID service_id,
                                             uint16_t attrib_idx);
bool zy100_ble_ctrl_service_is_ack_attrib(T_SERVER_ID service_id,
                                          uint16_t attrib_idx);
void zy100_ble_ctrl_service_on_ack_send_complete(uint8_t conn_id,
                                                 uint16_t cause);
bool zy100_ble_ctrl_service_ack_pipeline_idle(void);
/* App-task only, with a currently connected conn_id. Attempts one queued ACK;
 * true means SDK submission, not peer receipt or business completion. */
bool zy100_ble_ctrl_service_pump_ack(uint8_t conn_id);
bool zy100_ble_ctrl_service_notify_async_result(uint8_t cmd,
                                                uint8_t seq,
                                                uint8_t status,
                                                uint8_t device_state,
                                                uint8_t exec_mode,
                                                uint32_t user_id,
                                                uint32_t training_id,
                                                uint32_t detail);
bool zy100_ble_ctrl_service_notify_async_result_ex(uint8_t conn_id,
                                                   uint8_t cmd,
                                                   uint8_t seq,
                                                   uint8_t status,
                                                   uint8_t device_state,
                                                   uint8_t exec_mode,
                                                   uint8_t reserved,
                                                   uint32_t user_id,
                                                   uint32_t training_id,
                                                   uint32_t detail);
bool zy100_ble_ctrl_service_notify_timeout_action(uint8_t conn_id,
    uint8_t device_state, uint32_t generation, uint32_t token, uint8_t reason);

bool zy100_ble_ctrl_service_notify_device_state(uint8_t device_state);
bool zy100_ble_ctrl_service_notify_device_state_ex(uint8_t device_state,
                                                   uint32_t detail);
bool zy100_ble_ctrl_service_notify_link_state(uint32_t session_id,
                                              uint32_t generation,
                                              uint32_t transition_id,
                                              uint8_t business_state,
                                              uint8_t expected_profile,
                                              uint8_t link_state,
                                              uint8_t initiator_mode,
                                              uint16_t actual_ci,
                                              uint16_t actual_latency);
bool zy100_ble_ctrl_service_notify_connection_state(uint8_t revision,
                                                    uint8_t overall_status,
                                                    uint8_t device_state,
                                                    uint8_t suggested_action,
                                                    uint32_t generation,
                                                    uint32_t ready_bits,
                                                    uint32_t detail);
void zy100_ble_ctrl_service_reset_notify_state(uint8_t conn_id);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_BLE_CTRL_SERVICE_H */
