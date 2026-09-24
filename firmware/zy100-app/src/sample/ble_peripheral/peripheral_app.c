/**
*****************************************************************************************
*     Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      peripheral_app.c
   * @brief     This file handles BLE peripheral application routines.
   * @author    jane
   * @date      2017-06-06
   * @version   v1.0
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2017 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */

/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include <trace.h>
#include <os_sched.h>
#include <stdint.h>
#include <string.h>
#include <gap.h>
#include <gap_bond_le.h>
#include <gap_callback_le.h>
#include <gap_storage_le.h>
#include <profile_server.h>
#include <gap_msg.h>
#include <simple_ble_service.h>
#include <app_msg.h>
#include "app_flags.h"
#include "app_task.h"
#include "app/app_ble_capture_observer.h"
#include "app/app_ble_ci_state.h"
#include "app/app_ble_link_trace.h"
#include "app/app_ble_connection_bootstrap.h"
#include "app/app_ble_conn_param_mgr.h"
#include "app/app_ble_conn_param_mgr_port.h"
#include "app/app_ble_ctrl_dispatcher_port.h"
#include "app/app_ble_ctrl_dispatcher.h"
#include "app/app_ble_power_policy.h"
#include "app/app_ble_sensor_stream_port.h"
#if ZY100_BUILD_FACTORY
#include "app_factory/factory_ble_service.h"
#endif
#include "app_factory/factory_boot_gate.h"
#include "app_factory/factory_mfg.h"
#include "service/zy100_mfg_info_service.h"
#include "service/zy100_production_acceptance.h"
#include "service/zy100_production_shipping.h"
#include "service/zy100_whole_unit_test.h"
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
#include "app/app_ble_offline_v2_sync.h"
#include "service/zy100_ble_ctrl_protocol.h"
#include "service/zy100_ble_ctrl_service.h"
#include "service/zy100_calibration_ble_service.h"
#include "service/zy100_calibration_manager.h"
#include "service/zy100_calibration_protocol.h"
#include "service/zy100_online_stream.h"
#include "service/zy100_rtc_clock.h"
#include "service/zy100_system_info_store.h"
#endif
#include <peripheral_app.h>
#include <gap_conn_le.h>
#if ZY100_PRODUCT_LOG_QUIET_ENABLE
#undef DBG_DIRECT
#define DBG_DIRECT(...) ZY100_LOG_VERBOSE(__VA_ARGS__)
#endif
#if F_BT_ANCS_CLIENT_SUPPORT
#include <ancs_client.h>
#include <ancs.h>
#endif

#if F_LP_TEST_DISABLE_APP_LOG || !ZY100_LOG_BLE_STACK_INFO_ENABLE
#undef APP_PRINT_INFO0
#undef APP_PRINT_INFO1
#undef APP_PRINT_INFO2
#undef APP_PRINT_INFO3
#undef APP_PRINT_INFO4
#undef APP_PRINT_INFO5
#undef APP_PRINT_TRACE1
#define APP_PRINT_INFO0(...)
#define APP_PRINT_INFO1(...)
#define APP_PRINT_INFO2(...)
#define APP_PRINT_INFO3(...)
#define APP_PRINT_INFO4(...)
#define APP_PRINT_INFO5(...)
#define APP_PRINT_TRACE1(...)
#endif


/** @defgroup  PERIPH_APP Peripheral Application
    * @brief This file handles BLE peripheral application routines.
    * @{
    */
/*============================================================================*
 *                              Variables
 *============================================================================*/
/** @addtogroup  PERIPH_SEVER_CALLBACK Profile Server Callback Event Handler
    * @brief Handle profile server callback event
    * @{
    */
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
static app_ble_export_p0_link_snapshot_t s_ble_export_p0_link;
#endif

bool app_ble_online_quiet_logs_active(void)
{
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE && ZY100_ONLINE_STREAM_ENABLE
#if ZY100_ONLINE_STREAM_VERBOSE_TRACE_ENABLE
    return false;
#else
    return zy100_online_stream_quiet_logs_active();
#endif
#else
    return false;
#endif
}
/** @} */ /* End of group PERIPH_SEVER_CALLBACK */
/** @defgroup  PERIPH_GAP_MSG GAP Message Handler
    * @brief Handle GAP Message
    * @{
    */
#define APP_BLE_SENSOR_CONN_ID_INVALID         0xFFU
#define ZY100_BDADDR_RAW_FMT                   "%02X:%02X:%02X:%02X:%02X:%02X"
#define ZY100_BDADDR_RAW_ARG(a)                \
    (unsigned int)((a)[0]),                     \
    (unsigned int)((a)[1]),                     \
    (unsigned int)((a)[2]),                     \
    (unsigned int)((a)[3]),                     \
    (unsigned int)((a)[4]),                     \
    (unsigned int)((a)[5])
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
static uint8_t s_ble_pairing_conn_id = APP_BLE_SENSOR_CONN_ID_INVALID;
static bool s_ble_pairing_started = false;
static bool s_ble_pairing_ok = false;
#endif

/*============================================================================*
 *                              Functions
 *============================================================================*/
void app_handle_gap_msg(T_IO_MSG  *p_gap_msg);

#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE && ZY100_BLE_EXPORT_P0_PERF_ENABLE
static void app_ble_export_p0_note_conn_params(uint8_t conn_id,
                                               bool at_start,
                                               bool count_update)
{
    uint16_t conn_interval = 0U;
    uint16_t conn_latency = 0U;
    uint16_t conn_timeout = 0U;

    if (!app_ble_power_conn_valid(conn_id))
    {
        return;
    }
    (void)le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &conn_interval, conn_id);
    (void)le_get_conn_param(GAP_PARAM_CONN_LATENCY, &conn_latency, conn_id);
    (void)le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &conn_timeout, conn_id);
    if (at_start)
    {
        s_ble_export_p0_link.conn_interval_at_start = conn_interval;
        s_ble_export_p0_link.conn_latency_at_start = conn_latency;
        s_ble_export_p0_link.conn_timeout_at_start = conn_timeout;
    }
    s_ble_export_p0_link.conn_interval_last = conn_interval;
    s_ble_export_p0_link.conn_latency_last = conn_latency;
    s_ble_export_p0_link.conn_timeout_last = conn_timeout;
    if (count_update)
    {
        s_ble_export_p0_link.conn_update_count++;
    }
}

static void app_ble_export_p0_note_connected(uint8_t conn_id)
{
    uint16_t mtu_size = 0U;

    memset(&s_ble_export_p0_link, 0, sizeof(s_ble_export_p0_link));
    s_ble_export_p0_link.conn_id = conn_id;
    if (le_get_conn_param(GAP_PARAM_CONN_MTU_SIZE, &mtu_size, conn_id) ==
        GAP_CAUSE_SUCCESS)
    {
        s_ble_export_p0_link.mtu_at_start = mtu_size;
        s_ble_export_p0_link.mtu_last = mtu_size;
    }
    app_ble_export_p0_note_conn_params(conn_id, true, false);
#if F_BT_LE_5_0_SET_PHYS_SUPPORT
    (void)le_get_conn_param(GAP_PARAM_CONN_TX_PHY_TYPE,
                            &s_ble_export_p0_link.phy_tx_last,
                            conn_id);
    (void)le_get_conn_param(GAP_PARAM_CONN_RX_PHY_TYPE,
                            &s_ble_export_p0_link.phy_rx_last,
                            conn_id);
#endif
}

static void app_ble_export_p0_note_mtu(uint8_t conn_id, uint16_t mtu_size)
{
    if ((conn_id != s_ble_export_p0_link.conn_id) || (mtu_size == 0U))
    {
        return;
    }
    s_ble_export_p0_link.mtu_last = mtu_size;
    s_ble_export_p0_link.mtu_update_count++;
}

#if F_BT_LE_4_2_DATA_LEN_EXT_SUPPORT
static void app_ble_export_p0_note_dle(uint8_t conn_id,
                                       uint16_t tx_octets,
                                       uint16_t max_tx_time)
{
    if (conn_id != s_ble_export_p0_link.conn_id)
    {
        return;
    }
    s_ble_export_p0_link.dle_tx_octets_last = tx_octets;
    s_ble_export_p0_link.dle_max_tx_time_last = max_tx_time;
    s_ble_export_p0_link.dle_update_count++;
}
#endif

#if F_BT_LE_5_0_SET_PHYS_SUPPORT
static void app_ble_export_p0_note_phy(uint8_t conn_id,
                                       T_GAP_PHYS_TYPE tx_phy,
                                       T_GAP_PHYS_TYPE rx_phy)
{
    if (conn_id != s_ble_export_p0_link.conn_id)
    {
        return;
    }
    s_ble_export_p0_link.phy_tx_last = (uint8_t)tx_phy;
    s_ble_export_p0_link.phy_rx_last = (uint8_t)rx_phy;
    s_ble_export_p0_link.phy_update_count++;
}
#endif
#endif


#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
bool app_ble_ctrl_port_pairing_started(void)
{
    return s_ble_pairing_started;
}

bool app_ble_ctrl_port_publish_ack(uint8_t conn_id,
                                   uint8_t *ack,
                                   uint16_t len)
{
    bool notify_sent = false;

    (void)zy100_ble_ctrl_service_set_ack_value(ack, len);
    if (app_ble_power_conn_valid(conn_id))
    {
        notify_sent = zy100_ble_ctrl_service_send_ack_notify(conn_id,
                                                             ack,
                                                             len);
    }
    return notify_sent;
}

static void app_ble_ci_state_notify_pending(void)
{
    app_ble_ci_state_snapshot_t snapshot;
    uint8_t conn_id;

    if (!app_ble_ci_state_host_session_managed())
    {
        return;
    }
    conn_id = app_ble_power_conn_id();
    if (!zy100_ble_ctrl_service_ack_notify_enabled(conn_id) ||
        !app_ble_ci_state_take_link_notify(&snapshot))
    {
        return;
    }
    (void)zy100_ble_ctrl_service_notify_link_state(
        snapshot.session_id,
        snapshot.generation,
        snapshot.transition_id,
        (uint8_t)snapshot.desired_state,
        (uint8_t)snapshot.expected_profile,
        (uint8_t)snapshot.link_phase,
        (uint8_t)snapshot.initiator_mode,
        snapshot.actual_ci,
        snapshot.actual_latency);
}
#else
static void app_ble_ci_state_notify_pending(void)
{
}
#endif

void app_ble_ci_state_maintain_notify_and_disconnect(void)
{
    if (!app_task_shutdown_blocks_new_business())
    {
        app_ble_capture_observer_poll((uint32_t)os_sys_time_get());
        app_ble_connection_bootstrap_maintain();
        app_ble_ci_state_maintain();
        app_ble_ci_state_notify_pending();
    }
    app_ble_power_policy_disconnect_if_required();
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
    /* Retry on the existing task cadence, without opening a business gate. */
    if (!app_task_shutdown_blocks_new_business() &&
        !app_task_ota_blocks_new_business() &&
        app_ble_power_conn_valid(app_ble_power_conn_id()))
    {
        (void)zy100_ble_ctrl_service_pump_ack(app_ble_power_conn_id());
    }
#endif
}

void app_ble_calibration_poll(void)
{
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
    bool connected = app_ble_power_is_connected();
    bool paired = app_ble_pairing_ready();
    uint8_t conn_id = app_ble_power_conn_id();
    uint8_t state = app_task_ble_ctrl_device_state();
    bool idle = (state == ZY100_BLE_DEVICE_STATE_WAIT_START) &&
                 app_task_calibration_start_ready() &&
                 !app_ble_export_is_active() &&
                 zy100_online_stream_flash_idle();
    uint16_t mtu = connected ? app_ble_power_conn_mtu(conn_id) : 23U;
    zy100_cal_mag_start_gate_t mag_start_gate =
        ZY100_CAL_MAG_START_GATE_WAIT;
    app_cal_prepare_result_t prepare = APP_CAL_PREPARE_READY;

    if (!connected || !paired ||
        app_task_shutdown_blocks_new_business() ||
        app_ble_ci_state_terminal_failed() ||
        app_ble_ci_state_disconnect_required())
    {
        mag_start_gate = ZY100_CAL_MAG_START_GATE_ABORT;
    }
    else
    {
        if (zy100_cal_manager_mag_prepare_needed())
        {
            prepare = app_task_calibration_prepare_start();
            if (prepare == APP_CAL_PREPARE_READY)
                zy100_cal_manager_mag_prepare_complete();
        }
        if (prepare == APP_CAL_PREPARE_FAILED)
            mag_start_gate = ZY100_CAL_MAG_START_GATE_RESOURCE_FAILED;
        else if ((prepare == APP_CAL_PREPARE_READY) &&
                 app_ble_ci_state_target_reached(APP_BLE_CI_STATE_ACTIVE_IDLE))
            mag_start_gate = ZY100_CAL_MAG_START_GATE_READY;
    }

    zy100_cal_manager_poll(conn_id, mtu, connected,
                           paired, idle, mag_start_gate);
#endif
}

#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
void app_ble_export_p0_get_link_snapshot(
    uint8_t conn_id,
    app_ble_export_p0_link_snapshot_t *out)
{
    if (out == NULL)
    {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (conn_id == s_ble_export_p0_link.conn_id)
    {
        *out = s_ble_export_p0_link;
    }
}

void app_ble_export_p0_set_notify_shape(uint8_t conn_id,
                                        uint16_t notify_max,
                                        uint16_t chunk_payload)
{
    if (conn_id != s_ble_export_p0_link.conn_id)
    {
        return;
    }
    s_ble_export_p0_link.notify_max = notify_max;
    s_ble_export_p0_link.chunk_payload = chunk_payload;
}
#endif

bool app_ble_pairing_ready(void)
{
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
    return !app_task_shutdown_blocks_new_business() &&
           app_ble_power_is_connected() &&
           (s_ble_pairing_conn_id == app_ble_power_conn_id()) &&
           s_ble_pairing_ok;
#else
    return true;
#endif
}

void app_ble_ota_link_snapshot(uint8_t conn_id,
                               app_ble_ota_link_snapshot_t *snapshot)
{
    T_GAP_SEC_LEVEL sec_level = GAP_SEC_LEVEL_NO;

    if (snapshot == NULL)
    {
        return;
    }

    snapshot->connected = app_ble_power_conn_valid(conn_id);
    snapshot->paired = snapshot->connected && app_ble_pairing_ready();
    snapshot->encrypted =
        snapshot->connected &&
        (le_bond_get_sec_level(conn_id, &sec_level) == GAP_CAUSE_SUCCESS) &&
        (sec_level != GAP_SEC_LEVEL_NO);
}

#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
void app_ble_pairing_mark_ready(uint8_t conn_id, const char *reason)
{
    const char *use_reason = (reason != NULL) ? reason : "pairing_ready";
    const uint8_t current_conn_id = app_ble_power_conn_id();
    const T_GAP_CONN_STATE conn_state =
        app_ble_power_policy_conn_state();

    if (app_task_shutdown_blocks_new_business() ||
        (conn_id != current_conn_id) ||
        (conn_id != s_ble_pairing_conn_id) ||
        (conn_state != GAP_CONN_STATE_CONNECTED))
    {
        DBG_DIRECT("[BLE_PAIR] ready_mark_reject conn=%u cur=%u pair=%u state=%u reason=%s",
                   conn_id,
                   current_conn_id,
                   s_ble_pairing_conn_id,
                   (uint32_t)conn_state,
                   use_reason);
        return;
    }

    s_ble_pairing_ok = true;
    app_ble_connection_bootstrap_on_security_ready(conn_id);
    if (zy100_ble_ctrl_service_ack_notify_enabled(conn_id))
    {
        app_ble_ci_state_note_host_transport_ready(conn_id);
    }
    ZY100_LOG_PROCESS("[BLE_PAIR] ready conn=%u pairing_ok=1 reason=%s",
               conn_id,
               use_reason);
}
#endif


void app_ble_power_on_connected(uint8_t conn_id)
{
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
    app_task_training_context_t training_ctx;
    uint64_t rtc_ms;
#endif

    app_ble_power_policy_on_connected(conn_id);
    if (app_task_shutdown_blocks_new_business())
    {
        return;
    }
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
    app_ble_profile_router_on_connected(conn_id);
    app_task_training_context_get(&training_ctx);
    DBG_DIRECT("[BLE_SYNC] required conn=%u reason=connected active_user=%lu next_train=%lu rtc_calibrated=%u",
               conn_id,
               (unsigned long)training_ctx.active_user_id,
               (unsigned long)training_ctx.next_training_id,
               zy100_rtc_clock_is_calibrated() ? 1U : 0U);
    rtc_ms = zy100_rtc_clock_now_ms();
    ZY100_LOG_VERBOSE("[RTC_CONN] ble_connected calibrated=%u source=%u rtc_ms=%llu active_user=%lu next_train=%lu sync_required=1",
                      zy100_rtc_clock_is_calibrated() ? 1U : 0U,
                      (uint32_t)zy100_rtc_clock_time_source(),
                      (unsigned long long)rtc_ms,
                      (unsigned long)training_ctx.active_user_id,
                      (unsigned long)training_ctx.next_training_id);
    app_task_multi_session_refresh_readiness("ble_connected");
    if (app_task_multi_session_cached_pending_count() != 0U)
    {
        DBG_DIRECT("[BLE_EXPORT] wait_ready reason=notify_not_enabled pending=%lu",
                   (unsigned long)app_task_multi_session_cached_pending_count());
    }
    if (!app_ble_capture_observer_active(conn_id))
    {
        app_task_ble_link_led_notify(true, "ble_connected");
        app_ble_export_try_auto_start("ble_connected");
    }
#endif
}

void app_ble_power_on_disconnected(uint8_t conn_id,
                                   bool restore_standby_active)
{
    app_ble_power_policy_on_disconnected(
        conn_id,
        restore_standby_active ?
        APP_BLE_DISCONNECT_RECOVERY_DEFER_ACTIVE_RESTORE :
        APP_BLE_DISCONNECT_RECOVERY_DEFAULT);
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
    if (restore_standby_active &&
        !app_task_ble_disconnect_restore_active("ble_disconnected_standby"))
    {
        ZY100_LOG_ERROR("[BLE_STANDBY][ERR] disconnect_restore_handoff_failed conn=%u",
                   conn_id);
        return;
    }
    app_task_multi_session_refresh_readiness("ble_disconnected");
    if (!app_ble_capture_observer_active(conn_id))
    {
        app_task_ble_link_led_notify(false, "ble_disconnected");
    }
#endif
}

/**
 * @brief    All the application messages are pre-handled in this function
 * @note     All the IO MSGs are sent to this function, then the event handling
 *           function shall be called according to the MSG type.
 * @param[in] io_msg  IO message data
 * @return   void
 */
void app_handle_io_msg(T_IO_MSG io_msg)
{
    uint16_t msg_type = io_msg.type;

    switch (msg_type)
    {
    case IO_MSG_TYPE_BT_STATUS:
        {
            app_handle_gap_msg(&io_msg);
        }
        break;
#if F_APP_BATTERY_ADC_ENABLE && F_APP_BATTERY_LPC_GUARD_ENABLE
    case IO_MSG_TYPE_BAT_LPC:
        {
            app_task_battery_low_lpc_event_handle((uint32_t)os_sys_time_get());
        }
        break;
#endif
#if F_BT_ANCS_CLIENT_SUPPORT
    case IO_MSG_TYPE_ANCS:
        {
            ancs_handle_msg(&io_msg);
        }
        break;
#endif
    default:
        ZY100_LOG_VERBOSE("[APP_GAP] ignore io type=%d subtype=%d", io_msg.type, io_msg.subtype);
        break;
    }
}
/**
 * @brief    Handle msg GAP_MSG_LE_CONN_STATE_CHANGE
 * @note     All the gap conn state events are pre-handled in this function.
 *           Then the event handling function shall be called according to the new_state
 * @param[in] conn_id Connection ID
 * @param[in] new_state  New gap connection state
 * @param[in] disc_cause Use this cause when new_state is GAP_CONN_STATE_DISCONNECTED
 * @return   void
 */
void app_handle_conn_state_evt(uint8_t conn_id, T_GAP_CONN_STATE new_state, uint16_t disc_cause)
{
    T_GAP_CONN_STATE old_state = app_ble_power_policy_note_conn_state(new_state);

    APP_PRINT_INFO4("app_handle_conn_state_evt: conn_id %d old_state %d new_state %d, disc_cause 0x%x",
                    conn_id, old_state, new_state, disc_cause);
    (void)old_state;
    switch (new_state)
    {
    case GAP_CONN_STATE_DISCONNECTED:
        {
            app_ble_ci_state_snapshot_t link_snapshot;
            bool restore_standby_active =
                app_task_ble_disconnect_should_restore_active();
            uint32_t connection_generation = app_ble_ci_state_generation();
            app_ble_ci_state_get_snapshot(&link_snapshot);
            DBG_DIRECT("[BLE_DISCONNECT] conn=%u cause=0x%04X generation=%lu",
                       conn_id,
                       disc_cause,
                       (unsigned long)connection_generation);
            app_ble_link_trace_dump_disconnect(&link_snapshot, disc_cause);
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
            app_ble_ctrl_dispatcher_on_disconnect(conn_id);
#endif
            app_ble_connection_bootstrap_on_disconnected(conn_id);
            app_ble_ci_state_on_disconnected(conn_id, "gap_disconnected");
            app_task_auto_idle_note_activity("ble_disconnected");
            ZY100_LOG_EVENT("[EVT][BLE] disconnected conn=%u cause=0x%x",
                            conn_id, disc_cause);
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
            s_ble_pairing_conn_id = APP_BLE_SENSOR_CONN_ID_INVALID;
            s_ble_pairing_started = false;
            s_ble_pairing_ok = false;
            DBG_DIRECT("[BLE_PAIR] disconnected conn=%u pairing_ok=0", conn_id);
            app_task_pairing_post_event(APP_TASK_PAIRING_EVENT_DISCONNECTED,
                                        conn_id,
                                        NULL,
                                        0U,
                                        "disconnected");
#endif
            app_ble_sensor_stream_reset("GAP disconnected");
            app_ble_profile_router_reset_battery_notify();
            app_task_ota_on_disconnect(conn_id);
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
            app_ble_export_on_disconnect(conn_id);
            app_ble_offline_v2_sync_on_disconnect(conn_id);
            app_task_ble_ctrl_online_disconnected(conn_id);
            zy100_ble_ctrl_service_reset_notify_state(conn_id);
            zy100_cal_manager_on_disconnect(conn_id);
            app_task_ble_device_state_notify_reset("GAP disconnected");
#endif
            if (factory_boot_gate_factory_mode_active())
            {
#if ZY100_BUILD_FACTORY
                factory_ble_service_reset_notify_state(conn_id);
#endif
            }
            else
            {
                zy100_mfg_info_service_reset(conn_id);
            }
            zy100_production_acceptance_on_disconnected(conn_id);
            zy100_production_shipping_on_disconnected(conn_id);
            zy100_whole_unit_test_on_disconnected(conn_id);

            if ((disc_cause != (HCI_ERR | HCI_ERR_REMOTE_USER_TERMINATE))
                && (disc_cause != (HCI_ERR | HCI_ERR_LOCAL_HOST_TERMINATE)))
            {
                APP_PRINT_ERROR1("app_handle_conn_state_evt: connection lost cause 0x%x", disc_cause);
            }

            app_ble_power_on_disconnected(conn_id,
                                          restore_standby_active);
            app_ble_capture_observer_on_disconnected(conn_id);
        }
        break;

    case GAP_CONN_STATE_CONNECTED:
        {
            uint16_t conn_interval;
            uint16_t conn_latency;
            uint16_t conn_supervision_timeout;
            uint8_t  remote_bd[6];
            T_GAP_REMOTE_ADDR_TYPE remote_bd_type;
            bool standby_active;

            app_task_auto_idle_note_activity("ble_connected");
            ZY100_LOG_EVENT("[EVT][BLE] connected conn=%u", conn_id);
            app_ble_sensor_stream_reset(NULL);
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
            if (!app_task_shutdown_blocks_new_business() &&
                !zy100_production_acceptance_active() &&
                !zy100_whole_unit_test_active())
            {
                s_ble_pairing_conn_id = conn_id;
                s_ble_pairing_started = false;
                s_ble_pairing_ok = false;
                DBG_DIRECT("[BLE_PAIR] connected conn=%u pairing_ok=0 wait_auth=1", conn_id);
            }
#endif
            le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &conn_interval, conn_id);
            le_get_conn_param(GAP_PARAM_CONN_LATENCY, &conn_latency, conn_id);
            le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &conn_supervision_timeout, conn_id);
            le_get_conn_addr(conn_id, remote_bd, &remote_bd_type);
            APP_PRINT_INFO5("GAP_CONN_STATE_CONNECTED:remote_bd %s, remote_addr_type %d, conn_interval 0x%x, conn_latency 0x%x, conn_supervision_timeout 0x%x",
                            TRACE_BDADDR(remote_bd), remote_bd_type,
                            conn_interval, conn_latency, conn_supervision_timeout);
            if (factory_boot_gate_factory_mode_active())
            {
                factory_mfg_mark_ble_connected(conn_id);
            }
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE && ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
            app_ble_export_on_conn_param_snapshot(conn_id,
                                                  conn_interval,
                                                  conn_latency,
                                                  conn_supervision_timeout);
#endif
            app_ble_power_on_connected(conn_id);
            if (app_task_shutdown_blocks_new_business())
            {
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
                s_ble_pairing_conn_id = APP_BLE_SENSOR_CONN_ID_INVALID;
                s_ble_pairing_started = false;
                s_ble_pairing_ok = false;
#endif
                app_ble_ci_state_on_connected(
                    conn_id,
                    APP_BLE_CI_STATE_ACTIVE_IDLE,
                    "connected_shutdown_latched");
                {
                    app_ble_ci_state_snapshot_t link_snapshot;
                    app_ble_ci_state_get_snapshot(&link_snapshot);
                    app_ble_link_trace_on_connected(&link_snapshot);
                }
                DBG_DIRECT("[SHUTDOWN][BLE] connected_callback_skip conn=%u bootstrap=0 observer=0 pairing=0",
                           conn_id);
                app_ble_power_shutdown_latch();
                break;
            }
            if (zy100_production_acceptance_active())
            {
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
                s_ble_pairing_conn_id = APP_BLE_SENSOR_CONN_ID_INVALID;
                s_ble_pairing_started = false;
                s_ble_pairing_ok = false;
#endif
                app_ble_ci_state_on_connected(
                    conn_id,
                    APP_BLE_CI_STATE_ACTIVE_IDLE,
                    "production_acceptance");
                DBG_DIRECT("[PROD_ACCEPT] connected conn=%u pairing=0 bootstrap=0 business=0",
                           conn_id);
                break;
            }
            if (zy100_whole_unit_test_active())
            {
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
                s_ble_pairing_conn_id = APP_BLE_SENSOR_CONN_ID_INVALID;
                s_ble_pairing_started = false;
                s_ble_pairing_ok = false;
#endif
                app_ble_ci_state_on_connected(
                    conn_id,
                    APP_BLE_CI_STATE_ACTIVE_IDLE,
                    "whole_unit_test");
                zy100_whole_unit_test_on_connected(conn_id);
                ZY100_WHOLE_UNIT_LOG("[WHOLE] connected conn=%u pairing=0 bootstrap=0 business=0",
                           conn_id);
                break;
            }
            (void)app_ble_capture_observer_on_connected(conn_id);
            standby_active = app_ble_power_policy_standby_active();
            app_ble_ci_state_on_connected(
                conn_id,
                standby_active ?
                APP_BLE_CI_STATE_STANDBY :
                APP_BLE_CI_STATE_ACTIVE_IDLE,
                standby_active ?
                "connected_standby" :
                "connected_active");
            {
                app_ble_ci_state_snapshot_t link_snapshot;
                app_ble_ci_state_get_snapshot(&link_snapshot);
                app_ble_link_trace_on_connected(&link_snapshot);
            }
            app_ble_connection_bootstrap_on_connected(conn_id);
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE && ZY100_BLE_EXPORT_P0_PERF_ENABLE
            app_ble_export_p0_note_connected(conn_id);
#endif
        }
        break;

    default:
        break;
    }
}

/**
 * @brief    Handle msg GAP_MSG_LE_AUTHEN_STATE_CHANGE
 * @note     All the gap authentication state events are pre-handled in this function.
 *           Then the event handling function shall be called according to the new_state
 * @param[in] conn_id Connection ID
 * @param[in] new_state  New authentication state
 * @param[in] cause Use this cause when new_state is GAP_AUTHEN_STATE_COMPLETE
 * @return   void
 */
void app_handle_authen_state_evt(uint8_t conn_id, uint8_t new_state, uint16_t cause)
{
    APP_PRINT_INFO2("app_handle_authen_state_evt:conn_id %d, cause 0x%x", conn_id, cause);

    if (zy100_production_acceptance_active() ||
        zy100_whole_unit_test_active())
    {
        DBG_DIRECT("[PROD_ACCEPT] auth_event_ignored conn=%u state=%u cause=0x%x",
                   conn_id, new_state, cause);
        return;
    }

    if (app_task_shutdown_blocks_new_business())
    {
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
        s_ble_pairing_conn_id = APP_BLE_SENSOR_CONN_ID_INVALID;
        s_ble_pairing_started = false;
        s_ble_pairing_ok = false;
#endif
        DBG_DIRECT("[SHUTDOWN][BLE] auth_event_ignored conn=%u state=%u cause=0x%x",
                   conn_id,
                   new_state,
                   cause);
        return;
    }

    switch (new_state)
    {
    case GAP_AUTHEN_STATE_STARTED:
        {
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
            bool observer_handled;

            s_ble_pairing_conn_id = conn_id;
            s_ble_pairing_started = true;
            s_ble_pairing_ok = false;
            ZY100_DIAG_LOG("[BLE_PAIR] auth_started conn=%u", conn_id);
            observer_handled =
                app_ble_capture_observer_on_auth_started(conn_id);
            if (!observer_handled)
            {
                app_task_pairing_post_event(APP_TASK_PAIRING_EVENT_AUTH_STARTED,
                                            conn_id,
                                            NULL,
                                            0U,
                                            "auth_started");
            }
#endif
            APP_PRINT_INFO0("app_handle_authen_state_evt: GAP_AUTHEN_STATE_STARTED");
        }
        break;

    case GAP_AUTHEN_STATE_COMPLETE:
        {
            if (cause == GAP_SUCCESS)
            {
#if F_BT_ANCS_CLIENT_SUPPORT
                ancs_start_discovery(conn_id);
#endif
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
                {
                    bool observer_security_ready = false;
                    bool observer_handled;

                    observer_handled =
                        app_ble_capture_observer_on_auth_complete(
                            conn_id, true, &observer_security_ready);
                    s_ble_pairing_conn_id = conn_id;
                    s_ble_pairing_started = false;
                    s_ble_pairing_ok = false;
                    if (observer_handled)
                    {
                        if (observer_security_ready)
                        {
                            app_ble_pairing_mark_ready(
                                conn_id, "offline_observer_auth");
                        }
                    }
                    else
                    {
                        DBG_DIRECT("[BLE_PAIR] auth_success_defer_identity conn=%u",
                                   conn_id);
                        app_task_pairing_post_event(
                            APP_TASK_PAIRING_EVENT_AUTH_SUCCESS,
                            conn_id,
                            NULL,
                            0U,
                            "auth_success");
                    }
                }
#endif
                APP_PRINT_INFO0("app_handle_authen_state_evt: GAP_AUTHEN_STATE_COMPLETE pair success");

            }
            else
            {
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
                {
                    bool observer_security_ready = false;
                    bool observer_handled;

                    observer_handled =
                        app_ble_capture_observer_on_auth_complete(
                            conn_id, false, &observer_security_ready);
                    s_ble_pairing_conn_id = conn_id;
                    s_ble_pairing_started = false;
                    s_ble_pairing_ok = false;
                    ZY100_LOG_ERROR("[BLE_PAIR] failed conn=%u cause=0x%x pairing_ok=0",
                               conn_id,
                               cause);
                    if (!observer_handled)
                    {
                        app_task_pairing_post_event(
                            APP_TASK_PAIRING_EVENT_AUTH_FAILED,
                            conn_id,
                            NULL,
                            0U,
                            "auth_failed");
                    }
                }
#endif
                APP_PRINT_INFO0("app_handle_authen_state_evt: GAP_AUTHEN_STATE_COMPLETE pair failed");
            }
        }
        break;

    default:
        {
            APP_PRINT_ERROR1("app_handle_authen_state_evt: unknown newstate %d", new_state);
        }
        break;
    }
}

/**
 * @brief    Handle msg GAP_MSG_LE_CONN_MTU_INFO
 * @note     This msg is used to inform APP that exchange mtu procedure is completed.
 * @param[in] conn_id Connection ID
 * @param[in] mtu_size  New mtu size
 * @return   void
 */
void app_handle_conn_mtu_info_evt(uint8_t conn_id, uint16_t mtu_size)
{
    APP_PRINT_INFO2("app_handle_conn_mtu_info_evt: conn_id %d, mtu_size %d", conn_id, mtu_size);
    app_ble_sensor_stream_on_mtu_info(conn_id, mtu_size);
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE && ZY100_BLE_EXPORT_P0_PERF_ENABLE
    app_ble_export_p0_note_mtu(conn_id, mtu_size);
#endif
}

/**
 * @brief    Handle msg GAP_MSG_LE_CONN_PARAM_UPDATE
 * @note     All the connection parameter update change  events are pre-handled in this function.
 * @param[in] conn_id Connection ID
 * @param[in] status  New update state
 * @param[in] cause Use this cause when status is GAP_CONN_PARAM_UPDATE_STATUS_FAIL
 * @return   void
 */
void app_handle_conn_param_update_evt(uint8_t conn_id, uint8_t status, uint16_t cause)
{
    switch (status)
    {
    case GAP_CONN_PARAM_UPDATE_STATUS_SUCCESS:
        {
            uint16_t conn_interval = 0U;
            uint16_t conn_slave_latency = 0U;
            uint16_t conn_supervision_timeout = 0U;

            if ((le_get_conn_param(GAP_PARAM_CONN_INTERVAL,
                                   &conn_interval,
                                   conn_id) != GAP_CAUSE_SUCCESS) ||
                (le_get_conn_param(GAP_PARAM_CONN_LATENCY,
                                   &conn_slave_latency,
                                   conn_id) != GAP_CAUSE_SUCCESS) ||
                (le_get_conn_param(GAP_PARAM_CONN_TIMEOUT,
                                   &conn_supervision_timeout,
                                   conn_id) != GAP_CAUSE_SUCCESS))
            {
                break;
            }
            if (!app_ble_online_quiet_logs_active())
            {
                APP_PRINT_INFO3("app_handle_conn_param_update_evt update success:conn_interval 0x%x, conn_slave_latency 0x%x, conn_supervision_timeout 0x%x",
                                conn_interval, conn_slave_latency, conn_supervision_timeout);
                DBG_DIRECT("[BLE_CONN_PARAM] update success conn=%u ci=%u latency=%u timeout=%u",
                           conn_id,
                           conn_interval,
                           conn_slave_latency,
                           conn_supervision_timeout);
            }
            app_ble_conn_param_mgr_on_gap_update(conn_id,
                                                 status,
                                                 cause,
                                                 conn_interval,
                                                 conn_slave_latency,
                                                 conn_supervision_timeout);
            app_ble_ci_state_on_gap_update(conn_id,
                                            status,
                                            cause,
                                            conn_interval,
                                            conn_slave_latency,
                                            conn_supervision_timeout);
            app_task_ble_ctrl_wake("gap_conn_param_success");
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE && ZY100_BLE_EXPORT_P0_PERF_ENABLE
            app_ble_export_p0_note_conn_params(conn_id, false, true);
#endif
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE && ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
            app_ble_export_on_conn_param_update(conn_id,
                                                conn_interval,
                                                conn_slave_latency,
                                                conn_supervision_timeout,
                                                GAP_SUCCESS);
#endif
        }
        break;

    case GAP_CONN_PARAM_UPDATE_STATUS_FAIL:
        {
            uint16_t conn_interval = 0U;
            uint16_t conn_slave_latency = 0U;
            uint16_t conn_supervision_timeout = 0U;

            (void)le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &conn_interval, conn_id);
            (void)le_get_conn_param(GAP_PARAM_CONN_LATENCY, &conn_slave_latency, conn_id);
            (void)le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &conn_supervision_timeout, conn_id);
            APP_PRINT_ERROR1("app_handle_conn_param_update_evt update failed: cause 0x%x", cause);
            DBG_DIRECT("[BLE_CONN_PARAM] update fail conn=%u ci=%u latency=%u timeout=%u cause=0x%x",
                       conn_id,
                       conn_interval,
                       conn_slave_latency,
                       conn_supervision_timeout,
                       cause);
            app_ble_conn_param_mgr_on_gap_update(conn_id,
                                                 status,
                                                 cause,
                                                 conn_interval,
                                                 conn_slave_latency,
                                                 conn_supervision_timeout);
            app_ble_ci_state_on_gap_update(conn_id,
                                            status,
                                            cause,
                                            conn_interval,
                                            conn_slave_latency,
                                            conn_supervision_timeout);
            app_task_ble_ctrl_wake("gap_conn_param_fail");
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE && ZY100_BLE_EXPORT_FAST_CONN_PARAM_ENABLE
            app_ble_export_on_conn_param_update(conn_id,
                                                conn_interval,
                                                conn_slave_latency,
                                                conn_supervision_timeout,
                                                cause);
#endif
        }
        break;

    case GAP_CONN_PARAM_UPDATE_STATUS_PENDING:
        {
            uint16_t conn_interval = 0U;
            uint16_t conn_slave_latency = 0U;
            uint16_t conn_supervision_timeout = 0U;

            (void)le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &conn_interval, conn_id);
            (void)le_get_conn_param(GAP_PARAM_CONN_LATENCY, &conn_slave_latency, conn_id);
            (void)le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &conn_supervision_timeout, conn_id);
            if (!app_ble_online_quiet_logs_active())
            {
                APP_PRINT_INFO0("app_handle_conn_param_update_evt update pending.");
                DBG_DIRECT("[BLE_CONN_PARAM] update pending conn=%u ci=%u latency=%u timeout=%u",
                           conn_id,
                           conn_interval,
                           conn_slave_latency,
                           conn_supervision_timeout);
            }
            app_ble_conn_param_mgr_on_gap_update(conn_id,
                                                 status,
                                                 cause,
                                                 conn_interval,
                                                 conn_slave_latency,
                                                 conn_supervision_timeout);
            app_ble_ci_state_on_gap_update(conn_id,
                                            status,
                                            cause,
                                            conn_interval,
                                            conn_slave_latency,
                                            conn_supervision_timeout);
            app_task_ble_ctrl_wake("gap_conn_param_pending");
        }
        break;

    default:
        break;
    }
}

#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
static bool app_ble_pairing_addr_all_value(
    const uint8_t addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES],
    uint8_t value)
{
    uint8_t idx;

    if (addr == NULL)
    {
        return false;
    }

    for (idx = 0U; idx < ZY100_SYSTEM_INFO_BD_ADDR_BYTES; idx++)
    {
        if (addr[idx] != value)
        {
            return false;
        }
    }
    return true;
}

static bool app_ble_pairing_peer_type_valid(uint8_t peer_type)
{
    return (peer_type == (uint8_t)GAP_REMOTE_ADDR_LE_PUBLIC) ||
           (peer_type == (uint8_t)GAP_REMOTE_ADDR_LE_RANDOM);
}

static bool app_ble_pairing_key_entry_ptr_valid(const T_LE_KEY_ENTRY *entry)
{
    uintptr_t ptr = (uintptr_t)entry;

    return (ptr != (uintptr_t)0U) &&
           (ptr != (uintptr_t)0xFFFFFFFFUL) &&
           (ptr != (uintptr_t)0xBDBDBDBDUL);
}

static bool app_ble_pairing_peer_from_key_entry(
    const T_LE_KEY_ENTRY *entry,
    uint8_t peer_addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES],
    uint8_t *peer_type,
    bool *used_irk)
{
    const T_LE_REMOTE_BD *remote_bd;
    bool use_irk = false;
    uint8_t candidate_type;

    if ((entry == NULL) || (peer_addr == NULL) || (peer_type == NULL) ||
        (used_irk == NULL))
    {
        return false;
    }

    remote_bd = &entry->remote_bd;
#if F_BT_LE_PRIVACY_SUPPORT
    if ((entry->flags & LE_KEY_STORE_REMOTE_IRK_BIT) != 0U)
    {
        remote_bd = &entry->resolved_remote_bd;
        use_irk = true;
    }
#endif
    candidate_type = remote_bd->remote_bd_type;
    if (!app_ble_pairing_peer_type_valid(candidate_type) ||
        app_ble_pairing_addr_all_value(remote_bd->addr, 0x00U) ||
        app_ble_pairing_addr_all_value(remote_bd->addr, 0xFFU))
    {
        DBG_DIRECT("[BLE_PAIR] bond_identity_invalid type=%u addr="
                   ZY100_BDADDR_RAW_FMT " irk=%u",
                   candidate_type,
                   ZY100_BDADDR_RAW_ARG(remote_bd->addr),
                   use_irk ? 1U : 0U);
        return false;
    }

    memcpy(peer_addr, remote_bd->addr, ZY100_SYSTEM_INFO_BD_ADDR_BYTES);
    *peer_type = candidate_type;
    *used_irk = use_irk;
    return true;
}

bool app_ble_pairing_get_high_priority_identity(
    uint8_t peer_addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES],
    uint8_t *peer_type,
    bool *used_irk)
{
    T_LE_KEY_ENTRY *entry;

    if ((peer_addr == NULL) || (peer_type == NULL) || (used_irk == NULL))
    {
        return false;
    }

    entry = le_get_high_priority_bond();
    if (!app_ble_pairing_key_entry_ptr_valid(entry))
    {
        DBG_DIRECT("[BLE_PAIR] high_prio_identity_unavailable ptr=0x%08lX",
                   (unsigned long)(uintptr_t)entry);
        return false;
    }

    if (!app_ble_pairing_peer_from_key_entry(entry,
                                             peer_addr,
                                             peer_type,
                                             used_irk))
    {
        DBG_DIRECT("[BLE_PAIR] high_prio_identity_invalid ptr=0x%08lX",
                   (unsigned long)(uintptr_t)entry);
        return false;
    }

    DBG_DIRECT("[BLE_PAIR] high_prio_identity type=%u addr="
               ZY100_BDADDR_RAW_FMT " irk=%u",
               *peer_type,
               ZY100_BDADDR_RAW_ARG(peer_addr),
               *used_irk ? 1U : 0U);
    return true;
}

static void app_ble_pairing_post_bond_modify(
    T_LE_BOND_MODIFY_TYPE type,
    const T_LE_KEY_ENTRY *entry)
{
    app_task_pairing_event_t event = APP_TASK_PAIRING_EVENT_PAIRING_LOST;
    const char *reason = "bond_modify";
    uint8_t post_conn_id = (s_ble_pairing_conn_id != APP_BLE_SENSOR_CONN_ID_INVALID) ?
                           s_ble_pairing_conn_id : app_ble_power_conn_id();
    uint8_t peer_addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES] = {0};
    uint8_t peer_type = 0U;
    bool used_irk = false;
    bool peer_identity_valid = false;

    switch (type)
    {
    case LE_BOND_ADD:
        event = APP_TASK_PAIRING_EVENT_BOND_ADD;
        reason = "bond_add";
        break;
    case LE_BOND_DELETE:
        event = APP_TASK_PAIRING_EVENT_BOND_DELETE;
        reason = "bond_delete";
        break;
    case LE_BOND_CLEAR:
        event = APP_TASK_PAIRING_EVENT_BOND_CLEAR;
        reason = "bond_clear";
        break;
    case LE_BOND_KEY_MISSING:
        event = APP_TASK_PAIRING_EVENT_BOND_KEY_MISSING;
        reason = "key_missing";
        break;
    case LE_BOND_FULL:
        event = APP_TASK_PAIRING_EVENT_BOND_FULL;
        reason = "bond_full";
        DBG_DIRECT("[BLE_PAIR] bond_full max=%u conn=%u",
                   (uint32_t)ZY100_BLE_MAX_PAIRED_CENTRALS,
                   post_conn_id);
        break;
    default:
        ZY100_DIAG_LOG("[BLE_PAIR] bond_modify_unknown type=%u",
                   (uint32_t)type);
        return;
    }

    if (((type == LE_BOND_KEY_MISSING) &&
         app_ble_capture_observer_on_bond_key_missing(post_conn_id)) ||
        ((type == LE_BOND_ADD) &&
         app_ble_capture_observer_on_bond_add(post_conn_id)))
    {
        DBG_DIRECT("[OFF_OBS_B] conn=%u step=bond_event result=consumed type=%u",
                   post_conn_id,
                   (uint32_t)type);
        return;
    }

    if ((type == LE_BOND_ADD) ||
        (type == LE_BOND_DELETE) ||
        (type == LE_BOND_KEY_MISSING))
    {
        if (app_ble_pairing_key_entry_ptr_valid(entry))
        {
            peer_identity_valid = app_ble_pairing_peer_from_key_entry(entry,
                                                                       peer_addr,
                                                                       &peer_type,
                                                                       &used_irk);
        }
    }

    ZY100_DIAG_LOG("[BLE_PAIR] bond_modify_cb type=%u event=%u conn=%u identity=%u irk=%u",
               (uint32_t)type,
               (uint32_t)event,
               post_conn_id,
               peer_identity_valid ? 1U : 0U,
               used_irk ? 1U : 0U);
    app_task_pairing_post_event(event,
                                post_conn_id,
                                peer_identity_valid ? peer_addr : NULL,
                                peer_identity_valid ? peer_type : 0U,
                                reason);
}
#endif

/**
 * @brief    All the BT GAP MSG are pre-handled in this function.
 * @note     Then the event handling function shall be called according to the
 *           subtype of T_IO_MSG
 * @param[in] p_gap_msg Pointer to GAP msg
 * @return   void
 */
void app_handle_gap_msg(T_IO_MSG *p_gap_msg)
{
    T_LE_GAP_MSG gap_msg;
    uint8_t conn_id;
    ZY100_LOG_VERBOSE("[APP_GAP] io subtype=%d", p_gap_msg->subtype);
    memcpy(&gap_msg, &p_gap_msg->u.param, sizeof(p_gap_msg->u.param));

    APP_PRINT_TRACE1("app_handle_gap_msg: subtype %d", p_gap_msg->subtype);
    switch (p_gap_msg->subtype)
    {
    case GAP_MSG_LE_DEV_STATE_CHANGE:
        {
            app_handle_dev_state_evt(gap_msg.msg_data.gap_dev_state_change.new_state,
                                     gap_msg.msg_data.gap_dev_state_change.cause);
        }
        break;

    case GAP_MSG_LE_CONN_STATE_CHANGE:
        {
            app_handle_conn_state_evt(gap_msg.msg_data.gap_conn_state_change.conn_id,
                                      (T_GAP_CONN_STATE)gap_msg.msg_data.gap_conn_state_change.new_state,
                                      gap_msg.msg_data.gap_conn_state_change.disc_cause);
        }
        break;

    case GAP_MSG_LE_CONN_MTU_INFO:
        {
            app_handle_conn_mtu_info_evt(gap_msg.msg_data.gap_conn_mtu_info.conn_id,
                                         gap_msg.msg_data.gap_conn_mtu_info.mtu_size);
        }
        break;

    case GAP_MSG_LE_CONN_PARAM_UPDATE:
        {
            app_handle_conn_param_update_evt(gap_msg.msg_data.gap_conn_param_update.conn_id,
                                             gap_msg.msg_data.gap_conn_param_update.status,
                                             gap_msg.msg_data.gap_conn_param_update.cause);
        }
        break;

    case GAP_MSG_LE_AUTHEN_STATE_CHANGE:
        {
            app_handle_authen_state_evt(gap_msg.msg_data.gap_authen_state.conn_id,
                                        gap_msg.msg_data.gap_authen_state.new_state,
                                        gap_msg.msg_data.gap_authen_state.status);
        }
        break;

    case GAP_MSG_LE_BOND_JUST_WORK:
        {
            app_ble_capture_pairing_decision_t pairing_decision;

            conn_id = gap_msg.msg_data.gap_bond_just_work_conf.conn_id;
            pairing_decision = (app_task_shutdown_blocks_new_business() ||
                !app_device_pairing_admit_new_bond(conn_id)) ?
                APP_BLE_CAPTURE_PAIRING_REJECT :
                app_ble_capture_observer_on_pairing_request(
                    conn_id, "just_work");
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
            if (pairing_decision == APP_BLE_CAPTURE_PAIRING_NOT_OBSERVER)
            {
                app_task_pairing_post_event(
                    APP_TASK_PAIRING_EVENT_CONFIRM_PENDING,
                    conn_id,
                    NULL,
                    0U,
                    "just_work");
            }
#endif
            le_bond_just_work_confirm(
                conn_id,
                (pairing_decision == APP_BLE_CAPTURE_PAIRING_REJECT) ?
                GAP_CFM_CAUSE_REJECT : GAP_CFM_CAUSE_ACCEPT);
            APP_PRINT_INFO0("GAP_MSG_LE_BOND_JUST_WORK");
        }
        break;

    case GAP_MSG_LE_BOND_PASSKEY_DISPLAY:
        {
            uint32_t display_value = 0;
            app_ble_capture_pairing_decision_t pairing_decision;

            conn_id = gap_msg.msg_data.gap_bond_passkey_display.conn_id;
            pairing_decision = app_task_shutdown_blocks_new_business() ?
                APP_BLE_CAPTURE_PAIRING_REJECT :
                app_ble_capture_observer_on_pairing_request(
                    conn_id, "passkey_display");
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
            if (pairing_decision == APP_BLE_CAPTURE_PAIRING_NOT_OBSERVER)
            {
                app_task_pairing_post_event(
                    APP_TASK_PAIRING_EVENT_CONFIRM_PENDING,
                    conn_id,
                    NULL,
                    0U,
                    "passkey_display");
            }
#endif
            le_bond_get_display_key(conn_id, &display_value);
            APP_PRINT_INFO1("GAP_MSG_LE_BOND_PASSKEY_DISPLAY:passkey %d", display_value);
            le_bond_passkey_display_confirm(
                conn_id,
                (pairing_decision == APP_BLE_CAPTURE_PAIRING_REJECT) ?
                GAP_CFM_CAUSE_REJECT : GAP_CFM_CAUSE_ACCEPT);
        }
        break;

    case GAP_MSG_LE_BOND_USER_CONFIRMATION:
        {
            uint32_t display_value = 0;
            app_ble_capture_pairing_decision_t pairing_decision;

            conn_id = gap_msg.msg_data.gap_bond_user_conf.conn_id;
            pairing_decision = app_task_shutdown_blocks_new_business() ?
                APP_BLE_CAPTURE_PAIRING_REJECT :
                app_ble_capture_observer_on_pairing_request(
                    conn_id, "user_confirmation");
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
            if (pairing_decision == APP_BLE_CAPTURE_PAIRING_NOT_OBSERVER)
            {
                app_task_pairing_post_event(
                    APP_TASK_PAIRING_EVENT_CONFIRM_PENDING,
                    conn_id,
                    NULL,
                    0U,
                    "user_confirmation");
            }
#endif
            le_bond_get_display_key(conn_id, &display_value);
            APP_PRINT_INFO1("GAP_MSG_LE_BOND_USER_CONFIRMATION: passkey %d", display_value);
            le_bond_user_confirm(
                conn_id,
                (pairing_decision == APP_BLE_CAPTURE_PAIRING_REJECT) ?
                GAP_CFM_CAUSE_REJECT : GAP_CFM_CAUSE_ACCEPT);
        }
        break;

    case GAP_MSG_LE_BOND_PASSKEY_INPUT:
        {
            app_ble_capture_pairing_decision_t pairing_decision;

            conn_id = gap_msg.msg_data.gap_bond_passkey_input.conn_id;
            pairing_decision = app_task_shutdown_blocks_new_business() ?
                APP_BLE_CAPTURE_PAIRING_REJECT :
                app_ble_capture_observer_on_pairing_request(
                    conn_id, "passkey_input");
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
            if (pairing_decision == APP_BLE_CAPTURE_PAIRING_NOT_OBSERVER)
            {
                app_task_pairing_post_event(
                    APP_TASK_PAIRING_EVENT_CONFIRM_PENDING,
                    conn_id,
                    NULL,
                    0U,
                    "passkey_input");
            }
#endif
            APP_PRINT_INFO1("GAP_MSG_LE_BOND_PASSKEY_INPUT: conn_id %d", conn_id);
#if F_BLE_FIXED_PASSKEY_INPUT_ENABLE
            le_bond_passkey_input_confirm(
                conn_id,
                F_BLE_FIXED_PASSKEY_INPUT_VALUE,
                (pairing_decision == APP_BLE_CAPTURE_PAIRING_REJECT) ?
                GAP_CFM_CAUSE_REJECT : GAP_CFM_CAUSE_ACCEPT);
#else
            APP_PRINT_ERROR0("GAP_MSG_LE_BOND_PASSKEY_INPUT: fixed passkey disabled, reject by default");
            le_bond_passkey_input_confirm(conn_id, 0, GAP_CFM_CAUSE_REJECT);
#endif
        }
        break;

    case GAP_MSG_LE_BOND_OOB_INPUT:
        {
            uint8_t oob_data[GAP_OOB_LEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            app_ble_capture_pairing_decision_t pairing_decision;

            conn_id = gap_msg.msg_data.gap_bond_oob_input.conn_id;
            pairing_decision = app_task_shutdown_blocks_new_business() ?
                APP_BLE_CAPTURE_PAIRING_REJECT :
                app_ble_capture_observer_on_pairing_request(
                    conn_id, "oob_input");
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
            if (pairing_decision == APP_BLE_CAPTURE_PAIRING_NOT_OBSERVER)
            {
                app_task_pairing_post_event(
                    APP_TASK_PAIRING_EVENT_CONFIRM_PENDING,
                    conn_id,
                    NULL,
                    0U,
                    "oob_input");
            }
#endif
            APP_PRINT_INFO0("GAP_MSG_LE_BOND_OOB_INPUT");
            le_bond_set_param(GAP_PARAM_BOND_OOB_DATA, GAP_OOB_LEN, oob_data);
            le_bond_oob_input_confirm(
                conn_id,
                (pairing_decision == APP_BLE_CAPTURE_PAIRING_REJECT) ?
                GAP_CFM_CAUSE_REJECT : GAP_CFM_CAUSE_ACCEPT);
        }
        break;

    default:
        APP_PRINT_ERROR1("app_handle_gap_msg: unknown subtype %d", p_gap_msg->subtype);
        break;
    }
}
/** @} */ /* End of group PERIPH_GAP_MSG */

/** @defgroup  PERIPH_GAP_CALLBACK GAP Callback Event Handler
    * @brief Handle GAP callback event
    * @{
    */
/**
  * @brief Callback for gap le to notify app
  * @param[in] cb_type callback msy type @ref GAP_LE_MSG_Types.
  * @param[in] p_cb_data point to callback data @ref T_LE_CB_DATA.
  * @retval result @ref T_APP_RESULT
  */
T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data)
{
    T_APP_RESULT result = APP_RESULT_SUCCESS;
    T_LE_CB_DATA *p_data = (T_LE_CB_DATA *)p_cb_data;

    switch (cb_type)
    {
    case GAP_MSG_LE_DATA_LEN_CHANGE_INFO:
        APP_PRINT_INFO3("GAP_MSG_LE_DATA_LEN_CHANGE_INFO: conn_id %d, tx octets 0x%x, max_tx_time 0x%x",
                        p_data->p_le_data_len_change_info->conn_id,
                        p_data->p_le_data_len_change_info->max_tx_octets,
                        p_data->p_le_data_len_change_info->max_tx_time);
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE && ZY100_BLE_EXPORT_P0_PERF_ENABLE && \
    F_BT_LE_4_2_DATA_LEN_EXT_SUPPORT
        app_ble_export_p0_note_dle(
            p_data->p_le_data_len_change_info->conn_id,
            p_data->p_le_data_len_change_info->max_tx_octets,
            p_data->p_le_data_len_change_info->max_tx_time);
#endif
        break;

#if F_BT_LE_5_0_SET_PHYS_SUPPORT
    case GAP_MSG_LE_PHY_UPDATE_INFO:
        APP_PRINT_INFO4("GAP_MSG_LE_PHY_UPDATE_INFO: conn %d, cause 0x%x, rx_phy %d, tx_phy %d",
                        p_data->p_le_phy_update_info->conn_id,
                        p_data->p_le_phy_update_info->cause,
                        p_data->p_le_phy_update_info->rx_phy,
                        p_data->p_le_phy_update_info->tx_phy);
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE && ZY100_BLE_EXPORT_P0_PERF_ENABLE
        if (p_data->p_le_phy_update_info->cause == GAP_SUCCESS)
        {
            app_ble_export_p0_note_phy(
                p_data->p_le_phy_update_info->conn_id,
                p_data->p_le_phy_update_info->tx_phy,
                p_data->p_le_phy_update_info->rx_phy);
        }
#endif
        break;
#endif

#if F_BT_LE_READ_REMOTE_FEATS
    case GAP_MSG_LE_REMOTE_FEATS_INFO:
        APP_PRINT_INFO3("GAP_MSG_LE_REMOTE_FEATS_INFO: conn id %d, cause 0x%x, remote_feats %b",
                        p_data->p_le_remote_feats_info->conn_id,
                        p_data->p_le_remote_feats_info->cause,
                        TRACE_BINARY(8,
                            p_data->p_le_remote_feats_info->remote_feats));
        break;
#endif

    case GAP_MSG_LE_MODIFY_WHITE_LIST:
        APP_PRINT_INFO2("GAP_MSG_LE_MODIFY_WHITE_LIST: operation %d, cause 0x%x",
                        p_data->p_le_modify_white_list_rsp->operation,
                        p_data->p_le_modify_white_list_rsp->cause);
        break;

    case GAP_MSG_LE_BOND_MODIFY_INFO:
        {
            T_LE_BOND_MODIFY_TYPE type =
                p_data->p_le_bond_modify_info->type;

            APP_PRINT_INFO1("GAP_MSG_LE_BOND_MODIFY_INFO: type 0x%x", type);
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
            if ((type == LE_BOND_DELETE) ||
                (type == LE_BOND_CLEAR) ||
                (type == LE_BOND_KEY_MISSING))
            {
                s_ble_pairing_started = false;
                s_ble_pairing_ok = false;
            }
            app_ble_pairing_post_bond_modify(
                type,
                p_data->p_le_bond_modify_info->p_entry);
#endif
        }
        break;

    default:
        APP_PRINT_ERROR1("app_gap_callback: unhandled cb_type 0x%x", cb_type);
        break;
    }
    return result;
}
/** @} */ /* End of group PERIPH_GAP_CALLBACK */

/** @} */ /* End of group PERIPH_APP */
