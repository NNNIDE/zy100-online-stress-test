#ifndef APP_BLE_SENSOR_STREAM_H
#define APP_BLE_SENSOR_STREAM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_BLE_SENSOR_PAYLOAD_LEN             72U

typedef struct
{
    uint8_t status;
    uint8_t flags;
    uint16_t valid_mask;
    uint16_t seq;
    uint64_t timestamp_us;

    bool imu_valid;
    bool mag_valid;
    bool temp_valid;
    bool cfg_valid;
    bool stale_data;
    bool sensor_off;

    int32_t acc_x;
    int32_t acc_y;
    int32_t acc_z;
    int32_t gyro_x;
    int32_t gyro_y;
    int32_t gyro_z;
    int16_t temp_raw;
    uint16_t sample_period_us;

    uint8_t imu_pwr_mgmt0;
    uint8_t ois_config3;
    uint8_t imu_gyro_config0;
    uint8_t imu_accel_config0;
    uint8_t ois_status;
    uint8_t ext_data_x;
    uint8_t ext_data_y;
    uint8_t ext_data_z;

    uint32_t mag_x;
    uint32_t mag_y;
    uint32_t mag_z;

    uint32_t motion_event_label;
    bool pose_valid;
    uint32_t pose_output;
} app_ble_sensor_frame_t;

typedef struct
{
    bool ready;
    uint8_t conn_id;
    uint16_t mtu;
    uint8_t v3_notify_enabled;
    uint8_t v4_indicate_enabled;
    uint32_t null_frame;
    uint32_t pre_ready;
    uint32_t encode;
    uint32_t set_param;
    uint32_t post_ready;
    uint32_t notify;
} app_ble_sensor_stream_tx_diag_t;

bool app_ble_sensor_stream_ready(void);
void app_ble_sensor_stream_housekeep(void);
bool app_ble_sensor_frame_encode(const app_ble_sensor_frame_t *frame,
                                 uint8_t *payload,
                                 uint16_t payload_len);
bool app_ble_sensor_stream_push(const app_ble_sensor_frame_t *frame);
void app_ble_sensor_stream_take_tx_diag(app_ble_sensor_stream_tx_diag_t *diag_out);
uint32_t app_ble_sensor_stream_take_failure_count(void);

void app_ble_sensor_stream_reset(const char *reason);
uint16_t app_ble_sensor_stream_refresh_mtu(uint8_t conn_id);
void app_ble_sensor_stream_on_mtu_info(uint8_t conn_id, uint16_t mtu_size);
void app_ble_sensor_stream_on_cccd(uint8_t conn_id,
                                   bool v3_notify,
                                   bool enabled);
void app_ble_sensor_stream_prepare_read_value(void);
void app_ble_sensor_stream_note_send_failure(void);
bool app_ble_sensor_stream_v3_notify_enabled(void);
uint8_t app_ble_sensor_stream_conn_id(void);
bool app_ble_sensor_stream_ready_cached(uint8_t active_conn_id);

#ifdef __cplusplus
}
#endif

#endif /* APP_BLE_SENSOR_STREAM_H */
