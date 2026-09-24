#include <stdbool.h>
#include <stdint.h>
#include <cmsis_compiler.h>
#include <trace.h>
#include <gap.h>
#include <profile_server.h>
#include <simple_ble_service.h>
#include <bas.h>
#if F_BLE_OTA_SERVICE_ENABLE
#include <ota_service.h>
#endif

#include "app_flags.h"
#include "app_task.h"
#include "app_ble_profile_router.h"
#include "app_ble_profile_router_port.h"
#include "app_ble_ctrl_dispatcher.h"
#include "app_ble_ctrl_dispatcher_port.h"
#include "app_ble_capture_observer.h"
#include "app_ble_ci_state.h"
#include "app_ble_connection_bootstrap.h"
#if ZY100_OFFLINE_FEATURE_V2_ENABLE
#include "app_ble_offline_v2_sync.h"
#endif
#include "app_ble_sensor_stream.h"
#include "app_ble_sensor_stream_port.h"
#include "peripheral_app.h"
#include "service/battery_adc.h"
#include "service/app_ble_notify_tracker.h"
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
#include "service/zy100_ble_ctrl_protocol.h"
#include "service/zy100_ble_ctrl_service.h"
#include "service/zy100_calibration_ble_service.h"
#include "service/zy100_calibration_manager.h"
#include "service/zy100_calibration_protocol.h"
#include "service/zy100_online_stream.h"
#include "service/zy100_rtc_clock.h"
#if ZY100_OFFLINE_FEATURE_V2_ENABLE
#include "storage/zy100_offline_v2_storage.h"
#endif
#endif
#if F_BLE_OTA_SERVICE_ENABLE
#include "service/ble_ota_cmd_protocol.h"
#endif

#if ZY100_PRODUCT_LOG_QUIET_ENABLE
#undef DBG_DIRECT
#define DBG_DIRECT(...) ZY100_BLE_DIAG_LOG(__VA_ARGS__)
#endif

#if F_LP_TEST_DISABLE_APP_LOG || !ZY100_LOG_BLE_STACK_INFO_ENABLE
#undef APP_PRINT_INFO0
#undef APP_PRINT_INFO1
#undef APP_PRINT_INFO2
#undef APP_PRINT_INFO3
#undef APP_PRINT_INFO4
#undef APP_PRINT_INFO5
#define APP_PRINT_INFO0(...)
#define APP_PRINT_INFO1(...)
#define APP_PRINT_INFO2(...)
#define APP_PRINT_INFO3(...)
#define APP_PRINT_INFO4(...)
#define APP_PRINT_INFO5(...)
#endif

typedef struct
{
    T_SERVER_ID simple_id;
    T_SERVER_ID control_id;
    T_SERVER_ID calibration_id;
    T_SERVER_ID bas_id;
    T_SERVER_ID ota_id;
} app_ble_profile_router_service_ids_t;

typedef struct
{
    app_ble_profile_router_service_ids_t service_ids;
    uint8_t simp_v5_conn_token;
    uint8_t bas_battery_notify_conn_token;
    bool ble_time_sync_ok;
} app_ble_profile_router_state_t;

static app_ble_profile_router_state_t s_router_state;

#define s_service_ids s_router_state.service_ids
#define s_simp_v5_conn_token s_router_state.simp_v5_conn_token
#define s_bas_battery_notify_conn_token \
    s_router_state.bas_battery_notify_conn_token
#define s_ble_time_sync_ok s_router_state.ble_time_sync_ok

static bool s_bas_battery_notify_pending;
static bool s_bas_battery_notify_wait_logged;
static uint8_t s_bas_battery_notify_percent;
static uint32_t s_bas_battery_notify_next_ms;
static uint32_t s_bas_battery_notify_retry_count;

#define APP_BLE_BAS_NOTIFY_RETRY_MS 250U
#define APP_BLE_BAS_NOTIFY_MAX_ATTEMPTS 4U

void app_ble_profile_router_bind_service_ids(
    T_SERVER_ID simple_id,
    T_SERVER_ID control_id,
    T_SERVER_ID calibration_id,
    T_SERVER_ID bas_id,
    T_SERVER_ID ota_id)
{
    s_service_ids.simple_id = simple_id;
    s_service_ids.control_id = control_id;
    s_service_ids.calibration_id = calibration_id;
    s_service_ids.bas_id = bas_id;
    s_service_ids.ota_id = ota_id;
}

bool app_ble_sensor_stream_port_send_v3_notify(uint8_t conn_id,
                                               void *payload,
                                               uint16_t payload_len)
{
    return simp_ble_service_send_v3_notify(conn_id,
                                           s_service_ids.simple_id,
                                           payload,
                                           payload_len);
}

#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
static bool app_ble_profile_calibration_record_read_allowed(uint8_t state,
                                                            bool idle)
{
    if (idle)
    {
        return true;
    }

#if ZY100_OFFLINE_FEATURE_V2_ENABLE
    return (state == ZY100_BLE_DEVICE_STATE_OFFLINE_SESSION_READY) &&
           app_task_calibration_start_ready() &&
           !app_ble_offline_v2_sync_transfer_active() &&
           !zy100_offline_v2_storage_job_busy() &&
           zy100_online_stream_flash_idle();
#else
    (void)state;
    return false;
#endif
}
#endif

#if F_BLE_OTA_SERVICE_ENABLE
static void app_ble_ota_v5_notify_reset(const char *reason)
{
    bool had_state = (s_simp_v5_conn_token != 0U);

    s_simp_v5_conn_token = 0U;

    if (had_state && (reason != NULL))
    {
        DBG_DIRECT("[OTA_ACK][V5] reset: %s", reason);
    }
}

static bool app_ble_ota_cmd_send_v3_ack(
    uint8_t conn_id,
    const uint8_t ack[BLE_OTA_CMD_ACK_LEN])
{
    bool sent;

    if (ack == NULL)
    {
        DBG_DIRECT("[OTA_ACK][V3] fail=ack_null conn=%u", conn_id);
        return false;
    }

    if (!app_ble_sensor_stream_v3_notify_enabled())
    {
        DBG_DIRECT("[OTA_ACK][V3] fail=v3_notify_disabled conn=%u", conn_id);
        return false;
    }

    if (conn_id != app_ble_sensor_stream_conn_id())
    {
        DBG_DIRECT("[OTA_ACK][V3] fail=conn_id_mismatch conn=%u current=%u",
                   conn_id, app_ble_sensor_stream_conn_id());
        return false;
    }

    if (!app_ble_power_conn_valid(conn_id))
    {
        DBG_DIRECT("[OTA_ACK][V3] fail=conn_not_current conn=%u", conn_id);
        return false;
    }

    sent = simp_ble_service_send_v3_notify(conn_id, s_service_ids.simple_id,
                                           (void *)ack,
                                           BLE_OTA_CMD_ACK_LEN);
    if (sent)
    {
        app_ble_notify_tracker_note_submit(conn_id);
    }
    if (!sent)
    {
        DBG_DIRECT("[OTA_ACK][V3] fail=send_v3_notify_failed conn=%u", conn_id);
    }
    return sent;
}

static bool app_ble_ota_cmd_send_v5_ack(
    uint8_t conn_id,
    const uint8_t ack[BLE_OTA_CMD_ACK_LEN])
{
    bool sent;

    if (ack == NULL)
    {
        DBG_DIRECT("[OTA_ACK][V5] fail=ack_null conn=%u", conn_id);
        return false;
    }

    if (s_simp_v5_conn_token == 0U)
    {
        DBG_DIRECT("[OTA_ACK][V5] fail=v5_notify_disabled conn=%u", conn_id);
        return false;
    }

    if ((uint8_t)(conn_id + 1U) != s_simp_v5_conn_token)
    {
        DBG_DIRECT("[OTA_ACK][V5] fail=conn_id_mismatch conn=%u current=%u",
                   conn_id, (uint8_t)(s_simp_v5_conn_token - 1U));
        return false;
    }

    if (!app_ble_power_conn_valid(conn_id))
    {
        DBG_DIRECT("[OTA_ACK][V5] fail=conn_not_current conn=%u", conn_id);
        return false;
    }

    sent = simp_ble_service_send_v5_notify(conn_id, s_service_ids.simple_id,
                                           (void *)ack,
                                           BLE_OTA_CMD_ACK_LEN);
    if (sent)
    {
        app_ble_notify_tracker_note_submit(conn_id);
    }
    if (!sent)
    {
        DBG_DIRECT("[OTA_ACK][V5] fail=send_v5_notify_failed conn=%u", conn_id);
    }
    return sent;
}

static bool app_ble_ota_cmd_handle_write(uint8_t conn_id,
                                         const uint8_t *data,
                                         uint16_t len,
                                         bool prefer_v5_ack)
{
    uint8_t ack[BLE_OTA_CMD_ACK_LEN];
    bool ack_sent = false;
    bool accepted;
    app_ota_reject_reason_t reject_reason = APP_OTA_REJECT_NONE;

    if (!ble_ota_cmd_is_enter_request(data, len))
    {
        return false;
    }
    accepted = app_task_ota_try_admit(
                   conn_id,
                   prefer_v5_ack ? APP_OTA_SOURCE_PRIVATE_V5 :
                                   APP_OTA_SOURCE_PRIVATE_V2,
                   true,
                   &reject_reason);
    if (ble_ota_cmd_build_ack(ack, sizeof(ack), accepted))
    {
        if (prefer_v5_ack)
        {
            ack_sent = app_ble_ota_cmd_send_v5_ack(conn_id, ack);
        }
        if (!ack_sent)
        {
            ack_sent = app_ble_ota_cmd_send_v3_ack(conn_id, ack);
        }
    }

    if (accepted)
    {
        app_task_ota_accept_ack_result(ack_sent);
    }
    else
    {
        (void)reject_reason;
    }
    return true;
}
#endif

void app_ble_profile_router_on_connected(uint8_t conn_id)
{
    (void)conn_id;
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
    s_ble_time_sync_ok = false;
#endif
}

void app_ble_profile_router_reset_battery_notify(void)
{
    s_bas_battery_notify_conn_token = 0U;
    s_bas_battery_notify_pending = false;
    s_bas_battery_notify_wait_logged = false;
    s_bas_battery_notify_percent = 0U;
    s_bas_battery_notify_next_ms = 0U;
    s_bas_battery_notify_retry_count = 0U;
}

bool app_ble_time_sync_ok(void)
{
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
    return s_ble_time_sync_ok;
#else
    return true;
#endif
}

bool app_ble_battery_level_notify_ready(void)
{
    uint8_t conn_id;

    if (s_bas_battery_notify_conn_token == 0U)
    {
        return false;
    }

    conn_id = (uint8_t)(s_bas_battery_notify_conn_token - 1U);

    return app_ble_power_is_connected() &&
           (conn_id == app_ble_power_conn_id()) &&
           app_ble_power_conn_valid(conn_id);
}

bool app_ble_battery_level_notify(uint8_t percent)
{
    bool sent;
    uint8_t conn_id;

    if (!app_ble_battery_level_notify_ready())
    {
        return false;
    }

    conn_id = (uint8_t)(s_bas_battery_notify_conn_token - 1U);
    sent = bas_battery_level_value_notify(conn_id,
                                          s_service_ids.bas_id,
                                          percent);
    if (sent)
    {
        app_ble_notify_tracker_note_submit(conn_id);
    }
    return sent;
}

void app_ble_battery_level_notify_schedule(uint8_t percent)
{
    (void)bas_set_parameter(BAS_PARAM_BATTERY_LEVEL, 1U, &percent);

    if (s_bas_battery_notify_conn_token == 0U)
    {
        return;
    }

    s_bas_battery_notify_percent = percent;
    s_bas_battery_notify_pending = true;
    s_bas_battery_notify_wait_logged = false;
    s_bas_battery_notify_next_ms = 0U;
    s_bas_battery_notify_retry_count = 0U;
}

void app_ble_profile_router_battery_notify_poll(uint32_t now_ms)
{
    uint8_t percent;

    if (!s_bas_battery_notify_pending)
    {
        return;
    }

#if ZY100_OFFLINE_FEATURE_V2_ENABLE
    if (app_ble_offline_v2_sync_active())
    {
        return;
    }
#endif

    if (!app_ble_battery_level_notify_ready())
    {
        if (!s_bas_battery_notify_wait_logged)
        {
            DBG_DIRECT("[BAS] notify_wait connected=%u token=%u current_conn=%u valid=%u",
                       app_ble_power_is_connected() ? 1U : 0U,
                       s_bas_battery_notify_conn_token,
                       app_ble_power_conn_id(),
                       app_ble_power_conn_valid(app_ble_power_conn_id()) ?
                       1U : 0U);
            s_bas_battery_notify_wait_logged = true;
        }
        return;
    }

    s_bas_battery_notify_wait_logged = false;
    if ((s_bas_battery_notify_next_ms != 0U) &&
        ((int32_t)(now_ms - s_bas_battery_notify_next_ms) < 0))
    {
        return;
    }

    percent = s_bas_battery_notify_percent;
    if (app_ble_battery_level_notify(percent))
    {
        uint32_t retry_count = s_bas_battery_notify_retry_count;

        s_bas_battery_notify_pending = false;
        s_bas_battery_notify_next_ms = 0U;
        DBG_DIRECT("[BAS] notify_submitted conn=%u percent=%u retries=%lu",
                   (uint8_t)(s_bas_battery_notify_conn_token - 1U),
                   percent,
                   (unsigned long)retry_count);
        s_bas_battery_notify_retry_count = 0U;
        return;
    }

    s_bas_battery_notify_retry_count++;
    if (s_bas_battery_notify_retry_count >=
        APP_BLE_BAS_NOTIFY_MAX_ATTEMPTS)
    {
        s_bas_battery_notify_pending = false;
        s_bas_battery_notify_next_ms = 0U;
        DBG_DIRECT("[BAS] notify_give_up conn=%u percent=%u attempts=%lu",
                   (uint8_t)(s_bas_battery_notify_conn_token - 1U),
                   percent,
                   (unsigned long)s_bas_battery_notify_retry_count);
        return;
    }
    s_bas_battery_notify_next_ms = now_ms + APP_BLE_BAS_NOTIFY_RETRY_MS;
    DBG_DIRECT("[BAS] notify_retry conn=%u percent=%u count=%lu max=%u",
               (uint8_t)(s_bas_battery_notify_conn_token - 1U),
               percent,
               (unsigned long)s_bas_battery_notify_retry_count,
               APP_BLE_BAS_NOTIFY_MAX_ATTEMPTS);
}

#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
void app_ble_ctrl_port_mark_time_sync_complete(void)
{
    s_ble_time_sync_ok = true;
}
#endif

bool app_ble_sensor_stream_port_legacy_ota_active(void)
{
#if F_BLE_OTA_SERVICE_ENABLE
    return (s_simp_v5_conn_token != 0U);
#else
    return false;
#endif
}

__STATIC_FORCEINLINE void app_ble_profile_handle_send_data_complete(
    const T_SERVER_APP_CB_DATA *p_param)
{
    app_ble_notify_tracker_note_complete(
        p_param->event_data.send_data_result.conn_id);
    app_task_ota_on_notify_complete(
        p_param->event_data.send_data_result.cause == GAP_SUCCESS);
    if (p_param->event_data.send_data_result.service_id ==
        s_service_ids.bas_id)
    {
        DBG_DIRECT("[BAS] notify_complete conn=%u cause=0x%x credits=%u",
                   p_param->event_data.send_data_result.conn_id,
                   p_param->event_data.send_data_result.cause,
                   p_param->event_data.send_data_result.credits);
    }
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
    if (zy100_ble_ctrl_service_is_ack_attrib(
            p_param->event_data.send_data_result.service_id,
            p_param->event_data.send_data_result.attrib_idx))
    {
        zy100_ble_ctrl_service_on_ack_send_complete(
            p_param->event_data.send_data_result.conn_id,
            p_param->event_data.send_data_result.cause);
    }
    if (zy100_ble_ctrl_service_is_export_attrib(
            p_param->event_data.send_data_result.service_id,
            p_param->event_data.send_data_result.attrib_idx))
    {
        app_ble_export_on_send_data_complete(
            p_param->event_data.send_data_result.conn_id,
            p_param->event_data.send_data_result.service_id,
            p_param->event_data.send_data_result.attrib_idx,
            p_param->event_data.send_data_result.cause,
            p_param->event_data.send_data_result.credits);
    }
#endif
    {
        uint8_t sensor_conn_id = app_ble_sensor_stream_conn_id();

        if ((p_param->event_data.send_data_result.conn_id ==
             sensor_conn_id) &&
            (!app_ble_power_conn_valid(sensor_conn_id)))
        {
            app_ble_sensor_stream_reset("send complete stale link");
            app_ble_profile_router_port_restart_advertising_if_idle(
                "send complete stale link");
        }
    }
#if F_BLE_OTA_SERVICE_ENABLE
    if (((uint8_t)(p_param->event_data.send_data_result.conn_id + 1U) ==
         s_simp_v5_conn_token) &&
        (!app_ble_power_conn_valid(
            (uint8_t)(s_simp_v5_conn_token - 1U))))
    {
        app_ble_ota_v5_notify_reset("send complete stale link");
        app_ble_profile_router_port_restart_advertising_if_idle(
            "send complete stale link");
    }
#endif
    if (p_param->event_data.send_data_result.cause != GAP_SUCCESS)
    {
        app_ble_sensor_stream_note_send_failure();
    }
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
    /* Preserve all completion/refill ordering; ACK completion already pumps. */
    if (!app_task_shutdown_blocks_new_business() &&
        !app_task_ota_blocks_new_business() &&
        app_ble_power_conn_valid(p_param->event_data.send_data_result.conn_id) &&
        !zy100_ble_ctrl_service_is_ack_attrib(
            p_param->event_data.send_data_result.service_id,
            p_param->event_data.send_data_result.attrib_idx))
    {
        (void)zy100_ble_ctrl_service_pump_ack(
            p_param->event_data.send_data_result.conn_id);
    }
#endif
}

__STATIC_FORCEINLINE void app_ble_profile_handle_general_event(
    T_SERVER_APP_CB_DATA *p_param)
{
    switch (p_param->eventId)
    {
    case PROFILE_EVT_SRV_REG_COMPLETE:
        APP_PRINT_INFO1("PROFILE_EVT_SRV_REG_COMPLETE: result %d",
                        p_param->event_data.service_reg_result);
        if (p_param->event_data.service_reg_result != GATT_SERVER_SUCCESS)
        {
            APP_PRINT_ERROR0("PROFILE_EVT_SRV_REG_COMPLETE: failed");
        }
        break;

    case PROFILE_EVT_SRV_REG_AFTER_INIT_COMPLETE:
        APP_PRINT_INFO3("PROFILE_EVT_SRV_REG_AFTER_INIT_COMPLETE: result %d, service_id %d, cause 0x%x",
                        p_param->event_data.server_reg_after_init_result.result,
                        p_param->event_data.server_reg_after_init_result.service_id,
                        p_param->event_data.server_reg_after_init_result.cause);
        if (p_param->event_data.server_reg_after_init_result.result != GATT_SERVER_SUCCESS)
        {
            APP_PRINT_ERROR1("PROFILE_EVT_SRV_REG_AFTER_INIT_COMPLETE failed: cause 0x%x",
                             p_param->event_data.server_reg_after_init_result.cause);
        }
        break;

    case PROFILE_EVT_SEND_DATA_COMPLETE:
        app_ble_profile_handle_send_data_complete(p_param);
        break;

    default:
        break;
    }
}

__STATIC_FORCEINLINE void app_ble_profile_handle_simple(
    TSIMP_CALLBACK_DATA *p_simp_cb_data)
{
    switch (p_simp_cb_data->msg_type)
    {
    case SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION:
        {
            switch (p_simp_cb_data->msg_data.notification_indification_index)
            {
            case SIMP_NOTIFY_INDICATE_V3_ENABLE:
                {
                    APP_PRINT_INFO0("SIMP_NOTIFY_INDICATE_V3_ENABLE");
                    app_ble_sensor_stream_on_cccd(
                        p_simp_cb_data->conn_id, true, true);
                }
                break;

            case SIMP_NOTIFY_INDICATE_V3_DISABLE:
                {
                    APP_PRINT_INFO0("SIMP_NOTIFY_INDICATE_V3_DISABLE");
                    app_ble_sensor_stream_on_cccd(
                        p_simp_cb_data->conn_id, true, false);
                }
                break;
            case SIMP_NOTIFY_INDICATE_V4_ENABLE:
                {
                    APP_PRINT_INFO0("SIMP_NOTIFY_INDICATE_V4_ENABLE");
                    app_ble_sensor_stream_on_cccd(
                        p_simp_cb_data->conn_id, false, true);
                }
                break;
            case SIMP_NOTIFY_INDICATE_V4_DISABLE:
                {
                    APP_PRINT_INFO0("SIMP_NOTIFY_INDICATE_V4_DISABLE");
                    app_ble_sensor_stream_on_cccd(
                        p_simp_cb_data->conn_id, false, false);
                }
                break;
#if F_BLE_OTA_SERVICE_ENABLE
            case SIMP_NOTIFY_INDICATE_V5_ENABLE:
                {
                    APP_PRINT_INFO0("SIMP_NOTIFY_INDICATE_V5_ENABLE");
                    if (app_ble_power_conn_valid(p_simp_cb_data->conn_id))
                    {
                        s_simp_v5_conn_token =
                            (uint8_t)(p_simp_cb_data->conn_id + 1U);
                        DBG_DIRECT("[OTA_ACK][V5] notify_enabled conn=%u",
                                   p_simp_cb_data->conn_id);
                    }
                    else
                    {
                        ZY100_DIAG_LOG("[APP_GAP][BLE] ignore stale V5 notify enable conn_id=%d",
                                       p_simp_cb_data->conn_id);
                    }
                }
                break;
            case SIMP_NOTIFY_INDICATE_V5_DISABLE:
                {
                    APP_PRINT_INFO0("SIMP_NOTIFY_INDICATE_V5_DISABLE");
                    if (((uint8_t)(p_simp_cb_data->conn_id + 1U) ==
                         s_simp_v5_conn_token))
                    {
                        app_ble_ota_v5_notify_reset("CCCD disable");
                    }
                    else
                    {
                        ZY100_DIAG_LOG("[APP_GAP][BLE] ignore stale V5 notify disable conn_id=%d current=%d",
                                       p_simp_cb_data->conn_id,
                                       (uint8_t)(s_simp_v5_conn_token - 1U));
                    }
                }
                break;
#endif
            default:
                break;
            }
        }
        break;

    case SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE:
        {
            switch (p_simp_cb_data->msg_data.read_value_index)
            {
            case SIMP_READ_V1:
                {
                    APP_PRINT_INFO0("SIMP_READ_V1");
                    app_ble_sensor_stream_prepare_read_value();
                }
                break;
            default:
                break;
            }
        }
        break;
    case SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE:
        {
            switch (p_simp_cb_data->msg_data.write.opcode)
            {
            case SIMP_WRITE_V2:
                {
#if F_BLE_OTA_SERVICE_ENABLE
                    if (app_ble_ota_cmd_handle_write(
                            p_simp_cb_data->conn_id,
                            p_simp_cb_data->msg_data.write.p_value,
                            p_simp_cb_data->msg_data.write.len,
                            false))
                    {
                        break;
                    }
#endif
                    APP_PRINT_INFO2("SIMP_WRITE_V2: write type %d, len %d",
                                    p_simp_cb_data->msg_data.write.write_type,
                                    p_simp_cb_data->msg_data.write.len);
                }
                break;
            case SIMP_WRITE_V5:
                {
#if F_BLE_OTA_SERVICE_ENABLE
                    if (app_ble_ota_cmd_handle_write(
                            p_simp_cb_data->conn_id,
                            p_simp_cb_data->msg_data.write.p_value,
                            p_simp_cb_data->msg_data.write.len,
                            true))
                    {
                        break;
                    }
#endif
                    APP_PRINT_INFO0("SIMP_WRITE_V5 ignored: no current V5 product command");
                }
                break;
            default:
                break;
            }
        }
        break;

    default:
        break;
    }
}

#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
__STATIC_FORCEINLINE void app_ble_profile_handle_control(
    zy100_ble_ctrl_callback_data_t *p_ctrl_cb_data)
{
    if (p_ctrl_cb_data == NULL)
    {
        return;
    }

    switch (p_ctrl_cb_data->msg_type)
    {
    case SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION:
        {
            switch (p_ctrl_cb_data->msg_data.notification_indification_index)
            {
            case ZY100_BLE_CTRL_NOTIFY_ACK_ENABLE:
                app_ble_connection_bootstrap_on_ack_cccd(
                    p_ctrl_cb_data->conn_id, true);
                app_ble_capture_observer_on_ack_cccd(
                    p_ctrl_cb_data->conn_id, true);
                if (app_ble_pairing_ready())
                {
                    app_ble_ci_state_note_host_transport_ready(
                        p_ctrl_cb_data->conn_id);
                }
                ZY100_DIAG_LOG("[BLE_ACK] notify_enable conn=%u",
                               p_ctrl_cb_data->conn_id);
                ZY100_LOG_VERBOSE("[BLE_SYNC] notify_ready sync_required=%u sync_ok=%u",
                                  (app_ble_power_is_connected() &&
                                   !s_ble_time_sync_ok) ? 1U : 0U,
                                  s_ble_time_sync_ok ? 1U : 0U);
                ZY100_LOG_VERBOSE("[RTC_CONN] notify_ready calibrated=%u source=%u rtc_ms=%llu sync_required=%u sync_ok=%u",
                                  zy100_rtc_clock_is_calibrated() ? 1U : 0U,
                                  (uint32_t)zy100_rtc_clock_time_source(),
                                  (unsigned long long)zy100_rtc_clock_now_ms(),
                                  (app_ble_power_is_connected() &&
                                   !s_ble_time_sync_ok) ? 1U : 0U,
                                  s_ble_time_sync_ok ? 1U : 0U);
                if (!app_ble_capture_observer_active(
                        p_ctrl_cb_data->conn_id))
                {
                    app_task_multi_session_refresh_readiness("ack_notify_enabled");
                    app_ble_export_try_auto_start("ack_notify_enabled");
                    app_task_ble_device_state_notify_force("ack_notify_enabled");
                }
                break;
            case ZY100_BLE_CTRL_NOTIFY_ACK_DISABLE:
                app_ble_connection_bootstrap_on_ack_cccd(
                    p_ctrl_cb_data->conn_id, false);
                app_ble_capture_observer_on_ack_cccd(
                    p_ctrl_cb_data->conn_id, false);
                ZY100_DIAG_LOG("[BLE_ACK] notify_disable conn=%u",
                               p_ctrl_cb_data->conn_id);
                app_task_ble_ctrl_online_transport_disabled(
                    p_ctrl_cb_data->conn_id);
                if (!app_ble_capture_observer_active(
                        p_ctrl_cb_data->conn_id))
                {
                    app_task_multi_session_refresh_readiness(
                        "ack_notify_disabled");
                    app_task_capture_led_update_by_state(
                        "ack_notify_disabled");
                }
                app_task_ble_device_state_notify_reset("ack_notify_disabled");
                break;
            case ZY100_BLE_CTRL_NOTIFY_EXPORT_ENABLE:
                app_ble_connection_bootstrap_refresh(
                    p_ctrl_cb_data->conn_id);
                app_task_multi_session_refresh_readiness("export_notify_enabled");
                ZY100_LOG_VERBOSE("[BLE_EXPORT] notify_enabled auto_check pending=%lu",
                                  (unsigned long)app_task_multi_session_cached_pending_count());
                app_ble_export_try_auto_start("export_notify_enabled");
                break;
            case ZY100_BLE_CTRL_NOTIFY_EXPORT_DISABLE:
                app_ble_connection_bootstrap_refresh(
                    p_ctrl_cb_data->conn_id);
                DBG_DIRECT("[BLE_EXPORT] notify_disable conn=%u",
                           p_ctrl_cb_data->conn_id);
                app_task_ble_ctrl_online_transport_disabled(
                    p_ctrl_cb_data->conn_id);
                app_task_multi_session_refresh_readiness("export_notify_disabled");
                app_task_capture_led_update_by_state("export_notify_disabled");
                break;
            default:
                break;
            }
        }
        break;
    case SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE:
        app_ble_ctrl_dispatcher_handle_write(
            p_ctrl_cb_data->conn_id,
            p_ctrl_cb_data->msg_data.write.p_value,
            p_ctrl_cb_data->msg_data.write.len);
        break;
    default:
        break;
    }
}

__STATIC_FORCEINLINE void app_ble_profile_handle_calibration(
    zy100_cal_ble_callback_data_t *cal)
{
    if (cal == NULL)
    {
        return;
    }
    if (cal->msg_type == SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION)
    {
        switch (cal->msg_data.notification_index)
        {
        case ZY100_CAL_BLE_NOTIFY_TX_ENABLE:
            zy100_cal_manager_on_tx_cccd(cal->conn_id, true);
            break;
        case ZY100_CAL_BLE_NOTIFY_TX_DISABLE:
            zy100_cal_manager_on_tx_cccd(cal->conn_id, false);
            break;
        case ZY100_CAL_BLE_NOTIFY_STATUS_ENABLE:
            zy100_cal_manager_on_status_cccd(cal->conn_id, true);
            break;
        case ZY100_CAL_BLE_NOTIFY_STATUS_DISABLE:
            zy100_cal_manager_on_status_cccd(cal->conn_id, false);
            break;
        default:
            break;
        }
    }
    else if ((cal->msg_type == SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE) &&
             (cal->msg_data.write.opcode == ZY100_CAL_BLE_WRITE_RX))
    {
        uint8_t state = app_task_ble_ctrl_device_state();
        bool paired = app_ble_pairing_ready();
        bool base_idle = (state == ZY100_BLE_DEVICE_STATE_WAIT_START) &&
                    app_task_calibration_start_ready() &&
                    !app_ble_export_is_active() &&
                    zy100_online_stream_flash_idle();
        bool record_read_allowed =
            app_ble_profile_calibration_record_read_allowed(state,
                                                             base_idle);
        bool idle = base_idle &&
                    !app_ble_offline_v2_sync_blocks_new_business();
        /* Admission only: accepted calibration preparation runs in the app poll. */
        zy100_cal_manager_on_write(cal->conn_id,
                                   cal->msg_data.write.write_type,
                                   cal->msg_data.write.p_value,
                                   cal->msg_data.write.len,
                                   paired, idle,
                                   record_read_allowed);
    }
}
#endif

__STATIC_FORCEINLINE void app_ble_profile_handle_bas(
    T_BAS_CALLBACK_DATA *p_bas_cb_data)
{
    switch (p_bas_cb_data->msg_type)
    {
    case SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION:
        {
            switch (p_bas_cb_data->msg_data.notification_indification_index)
            {
            case BAS_NOTIFY_BATTERY_LEVEL_ENABLE:
                {
                    APP_PRINT_INFO0("BAS_NOTIFY_BATTERY_LEVEL_ENABLE");
                    if (p_bas_cb_data->conn_id < APP_MAX_LINKS)
                    {
                        s_bas_battery_notify_conn_token =
                            (uint8_t)(p_bas_cb_data->conn_id + 1U);
                        app_ble_battery_level_notify_schedule(
                            battery_adc_get_percent());
                        DBG_DIRECT("[BAS] battery_notify_enabled conn=%u initial_pending=1 percent=%u",
                                   p_bas_cb_data->conn_id,
                                   s_bas_battery_notify_percent);
                    }
                }
                break;

            case BAS_NOTIFY_BATTERY_LEVEL_DISABLE:
                {
                    APP_PRINT_INFO0("BAS_NOTIFY_BATTERY_LEVEL_DISABLE");
                    if (((uint8_t)(p_bas_cb_data->conn_id + 1U) ==
                         s_bas_battery_notify_conn_token))
                    {
                        app_ble_profile_router_reset_battery_notify();
                        DBG_DIRECT("[BAS] battery_notify_disabled conn=%u",
                                   p_bas_cb_data->conn_id);
                    }
                }
                break;
            default:
                break;
            }
        }
        break;

    case SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE:
        {
            if (p_bas_cb_data->msg_data.read_value_index == BAS_READ_BATTERY_LEVEL)
            {
                uint8_t battery_level = battery_adc_get_percent();
                bas_set_parameter(BAS_PARAM_BATTERY_LEVEL, 1, &battery_level);
                APP_PRINT_INFO1("BAS_READ_BATTERY_LEVEL: percent=%u",
                                battery_level);
            }
        }
        break;

    default:
        break;
    }
}

#if F_BLE_OTA_SERVICE_ENABLE
__STATIC_FORCEINLINE T_APP_RESULT app_ble_profile_handle_ota(
    T_OTA_CALLBACK_DATA *p_ota_cb_data)
{
    T_APP_RESULT app_result = APP_RESULT_SUCCESS;

    if (p_ota_cb_data == NULL)
    {
        return app_result;
    }

    switch (p_ota_cb_data->msg_type)
    {
    case SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE:
        {
            if ((p_ota_cb_data->msg_data.write.opcode == OTA_WRITE_CHAR_VAL) &&
                (p_ota_cb_data->msg_data.write.u.value == OTA_VALUE_ENTER))
            {
                app_ota_reject_reason_t reject_reason =
                    APP_OTA_REJECT_NONE;
                if (!app_task_ota_try_admit(app_ble_power_conn_id(),
                                            APP_OTA_SOURCE_STANDARD_GATT,
                                            false,
                                            &reject_reason))
                {
                    app_result = APP_RESULT_APP_ERR;
                }
            }
        }
        break;

    default:
        break;
    }

    return app_result;
}
#endif

T_APP_RESULT app_profile_callback(T_SERVER_ID service_id, void *p_data)
{
    T_APP_RESULT app_result = APP_RESULT_SUCCESS;

    if (service_id == SERVICE_PROFILE_GENERAL_ID)
    {
        app_ble_profile_handle_general_event(
            (T_SERVER_APP_CB_DATA *)p_data);
    }
    else if (service_id == s_service_ids.simple_id)
    {
        app_ble_profile_handle_simple((TSIMP_CALLBACK_DATA *)p_data);
    }
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
    else if (service_id == s_service_ids.control_id)
    {
        app_ble_profile_handle_control(
            (zy100_ble_ctrl_callback_data_t *)p_data);
    }
    else if (service_id == s_service_ids.calibration_id)
    {
        app_ble_profile_handle_calibration(
            (zy100_cal_ble_callback_data_t *)p_data);
    }
#endif
    else if (service_id == s_service_ids.bas_id)
    {
        app_ble_profile_handle_bas((T_BAS_CALLBACK_DATA *)p_data);
    }
#if F_BLE_OTA_SERVICE_ENABLE
    else if (service_id == s_service_ids.ota_id)
    {
        app_result = app_ble_profile_handle_ota(
            (T_OTA_CALLBACK_DATA *)p_data);
    }
#endif

    return app_result;
}
