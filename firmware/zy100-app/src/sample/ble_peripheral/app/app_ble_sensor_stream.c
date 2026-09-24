#include "app_ble_sensor_stream.h"
#include "app_ble_sensor_stream_port.h"

#include <string.h>

#include <gap.h>
#include <gap_conn_le.h>
#include <simple_ble_service.h>
#include <trace.h>

#include "../app_flags.h"
#include "../common/zy100_byteorder.h"
#include "../service/app_ble_notify_tracker.h"

#if ZY100_PRODUCT_LOG_QUIET_ENABLE
#undef DBG_DIRECT
#define DBG_DIRECT(...) ZY100_LOG_VERBOSE(__VA_ARGS__)
#endif

#define APP_BLE_SENSOR_CONN_ID_INVALID         0xFFU
#define APP_BLE_SENSOR_FRAME_VER               0x01U
#define APP_BLE_SENSOR_FRAME_TYPE_OIS_MAIN     0x10U
#define APP_BLE_SENSOR_VALID_GYRO              0x0001U
#define APP_BLE_SENSOR_VALID_ACCEL             0x0002U
#define APP_BLE_SENSOR_VALID_TEMP              0x0004U
#define APP_BLE_SENSOR_VALID_MAG               0x0008U
#define APP_BLE_SENSOR_VALID_TIMESTAMP         0x0010U
#define APP_BLE_SENSOR_VALID_CONFIG            0x0020U
#define APP_BLE_SENSOR_VALID_STALE             0x0040U
#define APP_BLE_SENSOR_VALID_SENSOR_OFF        0x0080U
#define APP_BLE_SENSOR_ATT_HEADER_LEN          3U
#define APP_BLE_SENSOR_REQUIRED_MTU            \
    (APP_BLE_SENSOR_PAYLOAD_LEN + APP_BLE_SENSOR_ATT_HEADER_LEN)
#define APP_BLE_SENSOR_DEFAULT_ATT_MTU         23U

static bool s_simp_v3_notify_enabled = false;
static bool s_simp_v4_indicate_enabled = false;
static uint8_t s_simp_stream_conn_id = APP_BLE_SENSOR_CONN_ID_INVALID;
static uint8_t s_simp_sensor_payload[APP_BLE_SENSOR_PAYLOAD_LEN];
static uint32_t s_simp_ble_stream_fail_count = 0U;
static uint32_t s_simp_ble_stream_null_frame_count = 0U;
static uint32_t s_simp_ble_stream_pre_ready_count = 0U;
static uint32_t s_simp_ble_stream_encode_count = 0U;
static uint32_t s_simp_ble_stream_set_param_count = 0U;
static uint32_t s_simp_ble_stream_post_ready_count = 0U;
static uint32_t s_simp_ble_stream_notify_count = 0U;
static uint16_t s_simp_stream_mtu = APP_BLE_SENSOR_DEFAULT_ATT_MTU;
static bool s_simp_stream_logged_v4_ignored = false;
static bool s_simp_stream_logged_mtu_insufficient = false;

static void app_ble_sensor_stream_counter_inc(uint32_t *counter)
{
    if ((counter != NULL) && (*counter < 0xFFFFFFFFU))
    {
        (*counter)++;
    }
}

static void app_ble_sensor_stream_note_push_drop(uint32_t *reason_counter)
{
    app_ble_sensor_stream_counter_inc(reason_counter);
}

void app_ble_sensor_stream_note_send_failure(void)
{
    app_ble_sensor_stream_counter_inc(&s_simp_ble_stream_fail_count);
}

uint32_t app_ble_sensor_stream_take_failure_count(void)
{
    uint32_t fail_count = s_simp_ble_stream_fail_count;
    s_simp_ble_stream_fail_count = 0U;
    return fail_count;
}

void app_ble_sensor_stream_take_tx_diag(app_ble_sensor_stream_tx_diag_t *diag_out)
{
    if (diag_out == NULL)
    {
        return;
    }

    memset(diag_out, 0, sizeof(*diag_out));
    diag_out->ready = (app_ble_power_is_connected() &&
                       (app_ble_power_conn_id() != APP_BLE_SENSOR_CONN_ID_INVALID) &&
                       (s_simp_stream_conn_id == app_ble_power_conn_id()) &&
                       s_simp_v3_notify_enabled &&
                       (s_simp_stream_mtu >= APP_BLE_SENSOR_REQUIRED_MTU));
    diag_out->conn_id = s_simp_stream_conn_id;
    diag_out->mtu = s_simp_stream_mtu;
    diag_out->v3_notify_enabled = s_simp_v3_notify_enabled ? 1U : 0U;
    diag_out->v4_indicate_enabled = s_simp_v4_indicate_enabled ? 1U : 0U;
    diag_out->null_frame = s_simp_ble_stream_null_frame_count;
    diag_out->pre_ready = s_simp_ble_stream_pre_ready_count;
    diag_out->encode = s_simp_ble_stream_encode_count;
    diag_out->set_param = s_simp_ble_stream_set_param_count;
    diag_out->post_ready = s_simp_ble_stream_post_ready_count;
    diag_out->notify = s_simp_ble_stream_notify_count;

    s_simp_ble_stream_null_frame_count = 0U;
    s_simp_ble_stream_pre_ready_count = 0U;
    s_simp_ble_stream_encode_count = 0U;
    s_simp_ble_stream_set_param_count = 0U;
    s_simp_ble_stream_post_ready_count = 0U;
    s_simp_ble_stream_notify_count = 0U;
}

void app_ble_sensor_stream_reset(const char *reason)
{
    bool had_state = ((s_simp_stream_conn_id != APP_BLE_SENSOR_CONN_ID_INVALID) ||
                      s_simp_v3_notify_enabled ||
                      s_simp_v4_indicate_enabled);

    s_simp_v3_notify_enabled = false;
    s_simp_v4_indicate_enabled = false;
    s_simp_stream_conn_id = APP_BLE_SENSOR_CONN_ID_INVALID;
    s_simp_stream_mtu = APP_BLE_SENSOR_DEFAULT_ATT_MTU;
    s_simp_stream_logged_v4_ignored = false;
    s_simp_stream_logged_mtu_insufficient = false;
    app_ble_sensor_stream_prepare_read_value();

    if (had_state && (reason != NULL))
    {
        DBG_DIRECT("[APP_GAP][BLE] stream reset: %s", reason);
    }
}

uint16_t app_ble_sensor_stream_refresh_mtu(uint8_t conn_id)
{
    uint16_t mtu_size = 0U;

    if (app_ble_power_conn_valid(conn_id) &&
        (le_get_conn_param(GAP_PARAM_CONN_MTU_SIZE, &mtu_size, conn_id) == GAP_CAUSE_SUCCESS) &&
        (mtu_size != 0U))
    {
        if (mtu_size != s_simp_stream_mtu)
        {
            s_simp_stream_logged_mtu_insufficient = false;
        }
        s_simp_stream_mtu = mtu_size;
    }

    return s_simp_stream_mtu;
}

bool app_ble_sensor_stream_ready(void)
{
    uint16_t mtu_size;

    if ((s_simp_stream_conn_id == APP_BLE_SENSOR_CONN_ID_INVALID) ||
        ((!s_simp_v3_notify_enabled) && (!s_simp_v4_indicate_enabled)))
    {
        s_simp_stream_logged_v4_ignored = false;
        s_simp_stream_logged_mtu_insufficient = false;
        return false;
    }

    if (!app_ble_power_conn_valid(s_simp_stream_conn_id))
    {
        app_ble_sensor_stream_reset("stale link");
        app_ble_sensor_stream_port_restart_advertising("stream stale link");
        return false;
    }

    if (!s_simp_v3_notify_enabled)
    {
        if (s_simp_v4_indicate_enabled && (!s_simp_stream_logged_v4_ignored))
        {
            ZY100_DIAG_LOG("[BLE_STREAM] V4 indicate enabled but ignored for 100Hz stream");
            s_simp_stream_logged_v4_ignored = true;
        }
        s_simp_stream_logged_mtu_insufficient = false;
        return false;
    }

    s_simp_stream_logged_v4_ignored = false;
    mtu_size = app_ble_sensor_stream_refresh_mtu(s_simp_stream_conn_id);
    if (mtu_size < APP_BLE_SENSOR_REQUIRED_MTU)
    {
        if (!s_simp_stream_logged_mtu_insufficient)
        {
            DBG_DIRECT("[BLE_STREAM] V3 notify enabled but MTU insufficient cur=%u need=%u",
                       mtu_size, (uint16_t)APP_BLE_SENSOR_REQUIRED_MTU);
            s_simp_stream_logged_mtu_insufficient = true;
        }
        return false;
    }

    s_simp_stream_logged_mtu_insufficient = false;
    return true;
}

void app_ble_sensor_stream_housekeep(void)
{
    if (le_get_active_link_num() != 0U)
    {
        return;
    }

    if (app_ble_power_is_connected() ||
        (s_simp_stream_conn_id != APP_BLE_SENSOR_CONN_ID_INVALID) ||
        s_simp_v3_notify_enabled ||
        s_simp_v4_indicate_enabled ||
        app_ble_sensor_stream_port_legacy_ota_active())
    {
        DBG_DIRECT("[APP_GAP][BLE] no active link; clear stream state and ensure advertising");
        app_ble_sensor_stream_reset(NULL);
        app_ble_sensor_stream_port_mark_gap_disconnected();
    }

    app_ble_sensor_stream_port_restart_advertising("stream housekeep");
}

bool app_ble_sensor_frame_encode(const app_ble_sensor_frame_t *frame,
                                 uint8_t *payload,
                                 uint16_t payload_len)
{
    uint8_t *p;
    uint16_t valid_mask;
    uint32_t pose_output;
    uint32_t motion_event_label;

    if ((frame == NULL) || (payload == NULL) || (payload_len != APP_BLE_SENSOR_PAYLOAD_LEN))
    {
        return false;
    }

    p = payload;
    memset(p, 0, payload_len);

    valid_mask = 0U;
    if (frame->imu_valid)
    {
        valid_mask |= (APP_BLE_SENSOR_VALID_GYRO | APP_BLE_SENSOR_VALID_ACCEL);
    }
    if (frame->temp_valid)
    {
        valid_mask |= APP_BLE_SENSOR_VALID_TEMP;
    }
    if (frame->mag_valid)
    {
        valid_mask |= APP_BLE_SENSOR_VALID_MAG;
    }
    if (frame->timestamp_us != 0ULL)
    {
        valid_mask |= APP_BLE_SENSOR_VALID_TIMESTAMP;
    }
    if (frame->cfg_valid)
    {
        valid_mask |= APP_BLE_SENSOR_VALID_CONFIG;
    }
    if (frame->stale_data)
    {
        valid_mask |= APP_BLE_SENSOR_VALID_STALE;
    }
    if (frame->sensor_off)
    {
        valid_mask |= APP_BLE_SENSOR_VALID_SENSOR_OFF;
    }
    valid_mask |= (uint16_t)(frame->valid_mask & (uint16_t)(~0x00FFU));

    p[0] = APP_BLE_SENSOR_FRAME_VER;
    p[1] = APP_BLE_SENSOR_FRAME_TYPE_OIS_MAIN;
    p[2] = frame->status;
    p[3] = frame->flags;
    zy100_put_u16_le(p + 4U, valid_mask);
    zy100_put_u16_le(p + 6U, frame->seq);
    zy100_put_u64_le(p + 8U, frame->timestamp_us);

    zy100_put_u32_le(p + 16U, (uint32_t)frame->gyro_x);
    zy100_put_u32_le(p + 20U, (uint32_t)frame->gyro_y);
    zy100_put_u32_le(p + 24U, (uint32_t)frame->gyro_z);
    zy100_put_u32_le(p + 28U, (uint32_t)frame->acc_x);
    zy100_put_u32_le(p + 32U, (uint32_t)frame->acc_y);
    zy100_put_u32_le(p + 36U, (uint32_t)frame->acc_z);
    zy100_put_u16_le(p + 40U, (uint16_t)frame->temp_raw);
    zy100_put_u16_le(p + 42U, frame->sample_period_us);

    p[44] = frame->cfg_valid ? frame->imu_pwr_mgmt0 : 0U;
    p[45] = frame->cfg_valid ? frame->ois_config3 : 0U;
    p[46] = frame->cfg_valid ? frame->imu_gyro_config0 : 0U;
    p[47] = frame->cfg_valid ? frame->imu_accel_config0 : 0U;
    p[48] = frame->imu_valid ? frame->ois_status : 0U;
    p[49] = frame->imu_valid ? frame->ext_data_x : 0U;
    p[50] = frame->imu_valid ? frame->ext_data_y : 0U;
    p[51] = frame->imu_valid ? frame->ext_data_z : 0U;

    zy100_put_u32_le(p + 52U, frame->mag_valid ? frame->mag_x : 0U);
    zy100_put_u32_le(p + 56U, frame->mag_valid ? frame->mag_y : 0U);
    zy100_put_u32_le(p + 60U, frame->mag_valid ? frame->mag_z : 0U);

    motion_event_label = frame->imu_valid ? frame->motion_event_label : 0U;
    pose_output = frame->pose_valid ? frame->pose_output : 0U;
    zy100_put_u32_le(p + 64U, motion_event_label);
    zy100_put_u32_le(p + 68U, pose_output);
    return true;
}

void app_ble_sensor_stream_prepare_read_value(void)
{
    app_ble_sensor_frame_t frame;

    if ((s_simp_sensor_payload[0] != APP_BLE_SENSOR_FRAME_VER) ||
        (s_simp_sensor_payload[1] != APP_BLE_SENSOR_FRAME_TYPE_OIS_MAIN))
    {
        memset(&frame, 0, sizeof(frame));
        frame.sensor_off = true;
        (void)app_ble_sensor_frame_encode(&frame,
                                          s_simp_sensor_payload,
                                          (uint16_t)sizeof(s_simp_sensor_payload));
    }

    (void)simp_ble_service_set_parameter(SIMPLE_BLE_SERVICE_PARAM_V1_READ_CHAR_VAL,
                                         APP_BLE_SENSOR_PAYLOAD_LEN,
                                         s_simp_sensor_payload);
}

bool app_ble_sensor_stream_push(const app_ble_sensor_frame_t *frame)
{
    bool set_param_ok;
    bool notify_ok;

    if (frame == NULL)
    {
        app_ble_sensor_stream_note_push_drop(&s_simp_ble_stream_null_frame_count);
        return false;
    }

    if (!app_ble_sensor_stream_ready())
    {
        app_ble_sensor_stream_note_push_drop(&s_simp_ble_stream_pre_ready_count);
        return false;
    }

    if (!app_ble_sensor_frame_encode(frame, s_simp_sensor_payload,
                                     (uint16_t)sizeof(s_simp_sensor_payload)))
    {
        app_ble_sensor_stream_note_push_drop(&s_simp_ble_stream_encode_count);
        return false;
    }
    set_param_ok = simp_ble_service_set_parameter(SIMPLE_BLE_SERVICE_PARAM_V1_READ_CHAR_VAL,
                                                  APP_BLE_SENSOR_PAYLOAD_LEN,
                                                  s_simp_sensor_payload);
    if (!set_param_ok)
    {
        app_ble_sensor_stream_note_push_drop(&s_simp_ble_stream_set_param_count);
    }

    if (!app_ble_sensor_stream_ready())
    {
        app_ble_sensor_stream_note_push_drop(&s_simp_ble_stream_post_ready_count);
        return false;
    }

    notify_ok = app_ble_sensor_stream_port_send_v3_notify(
                    s_simp_stream_conn_id,
                    s_simp_sensor_payload,
                    APP_BLE_SENSOR_PAYLOAD_LEN);
    if (notify_ok)
    {
        app_ble_notify_tracker_note_submit(s_simp_stream_conn_id);
    }
    if (!notify_ok)
    {
        app_ble_sensor_stream_note_push_drop(&s_simp_ble_stream_notify_count);
        app_ble_sensor_stream_note_send_failure();
    }
    return notify_ok;
}

void app_ble_sensor_stream_on_mtu_info(uint8_t conn_id, uint16_t mtu_size)
{
    if (app_ble_power_conn_valid(conn_id) &&
        (conn_id == s_simp_stream_conn_id) &&
        (mtu_size != 0U))
    {
        s_simp_stream_mtu = mtu_size;
        s_simp_stream_logged_mtu_insufficient = false;
        DBG_DIRECT("[BLE_STREAM] MTU update conn=%u cur=%u need=%u",
                   conn_id, s_simp_stream_mtu, (uint16_t)APP_BLE_SENSOR_REQUIRED_MTU);
    }
}

void app_ble_sensor_stream_on_cccd(uint8_t conn_id,
                                   bool v3_notify,
                                   bool enabled)
{
    if (v3_notify)
    {
        if (enabled)
        {
            if (app_ble_power_conn_valid(conn_id))
            {
                s_simp_stream_conn_id = conn_id;
                s_simp_v3_notify_enabled = true;
            }
            else
            {
                ZY100_DIAG_LOG("[APP_GAP][BLE] ignore stale V3 notify enable conn_id=%d",
                               conn_id);
            }
        }
        else if (conn_id == s_simp_stream_conn_id)
        {
            s_simp_v3_notify_enabled = false;
            if (!s_simp_v4_indicate_enabled)
            {
                s_simp_stream_conn_id = APP_BLE_SENSOR_CONN_ID_INVALID;
            }
        }
        else
        {
            ZY100_DIAG_LOG("[APP_GAP][BLE] ignore stale V3 notify disable conn_id=%d current=%d",
                           conn_id, s_simp_stream_conn_id);
        }
        return;
    }

    if (enabled)
    {
        if (app_ble_power_conn_valid(conn_id))
        {
            s_simp_stream_conn_id = conn_id;
            s_simp_v4_indicate_enabled = true;
        }
        else
        {
            ZY100_DIAG_LOG("[APP_GAP][BLE] ignore stale V4 indicate enable conn_id=%d",
                           conn_id);
        }
    }
    else if (conn_id == s_simp_stream_conn_id)
    {
        s_simp_v4_indicate_enabled = false;
        if (!s_simp_v3_notify_enabled)
        {
            s_simp_stream_conn_id = APP_BLE_SENSOR_CONN_ID_INVALID;
        }
    }
    else
    {
        ZY100_DIAG_LOG("[APP_GAP][BLE] ignore stale V4 indicate disable conn_id=%d current=%d",
                       conn_id, s_simp_stream_conn_id);
    }
}

bool app_ble_sensor_stream_v3_notify_enabled(void)
{
    return s_simp_v3_notify_enabled;
}

uint8_t app_ble_sensor_stream_conn_id(void)
{
    return s_simp_stream_conn_id;
}

#if ZY100_LOG_VERBOSE_DEFAULT
bool app_ble_sensor_stream_ready_cached(uint8_t active_conn_id)
{
    return (app_ble_power_is_connected() &&
            (active_conn_id != APP_BLE_SENSOR_CONN_ID_INVALID) &&
            (s_simp_stream_conn_id == active_conn_id) &&
            s_simp_v3_notify_enabled &&
            (s_simp_stream_mtu >= APP_BLE_SENSOR_REQUIRED_MTU));
}
#endif
