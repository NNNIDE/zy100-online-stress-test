/**
*****************************************************************************************
*     Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      peripheral_app.h
   * @brief     This file handles BLE peripheral application routines.
   * @author    jane
   * @date      2017-06-06
   * @version   v1.0
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2017 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */

#ifndef _PERIPHERAL_APP__
#define _PERIPHERAL_APP__

#ifdef __cplusplus
extern "C" {
#endif
/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include <app_msg.h>
#include <gap_le.h>
#include <profile_server.h>
#include <stdbool.h>
#include <stdint.h>
#include "app_flags.h"
#include "app/app_ble_conn_param_mgr.h"
#include "app/app_ble_ci_state.h"
#include "app/app_ble_power_policy.h"
#include "app/app_ble_profile_router.h"
#include "app/app_ble_sensor_stream.h"
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
#include "service/zy100_system_info_store.h"
#endif


/** @defgroup PERIPH_APP Peripheral Application
  * @brief Peripheral Application
  * @{
  */


#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
typedef struct
{
    uint8_t conn_id;
    uint16_t mtu_at_start;
    uint16_t mtu_last;
    uint32_t mtu_update_count;
    uint16_t notify_max;
    uint16_t chunk_payload;
    uint16_t conn_interval_at_start;
    uint16_t conn_interval_last;
    uint16_t conn_latency_at_start;
    uint16_t conn_latency_last;
    uint16_t conn_timeout_at_start;
    uint16_t conn_timeout_last;
    uint32_t conn_update_count;
    uint16_t dle_tx_octets_last;
    uint16_t dle_max_tx_time_last;
    uint32_t dle_update_count;
    uint8_t phy_tx_last;
    uint8_t phy_rx_last;
    uint32_t phy_update_count;
} app_ble_export_p0_link_snapshot_t;
#endif

/*============================================================================*
 *                              Functions
 *============================================================================*/

/**
 * @brief    All the application messages are pre-handled in this function
 * @note     All the IO MSGs are sent to this function, then the event handling
 *           function shall be called according to the MSG type.
 * @param[in] io_msg  IO message data
 * @return   void
 */
void app_handle_io_msg(T_IO_MSG io_msg);

/**
  * @brief Callback for gap le to notify app
  * @param[in] cb_type callback msy type @ref GAP_LE_MSG_Types.
  * @param[in] p_cb_data point to callback data @ref T_LE_CB_DATA.
  * @retval result @ref T_APP_RESULT
  */
T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data);

void app_ble_power_on_connected(uint8_t conn_id);
void app_ble_power_on_disconnected(uint8_t conn_id,
                                   bool restore_standby_active);
void app_ble_calibration_poll(void);
#if ZY100_BLE_EXPORT_P0_PERF_ENABLE
void app_ble_export_p0_get_link_snapshot(
    uint8_t conn_id,
    app_ble_export_p0_link_snapshot_t *out);
void app_ble_export_p0_set_notify_shape(uint8_t conn_id,
                                        uint16_t notify_max,
                                        uint16_t chunk_payload);
#endif
bool app_ble_pairing_ready(void);
typedef struct
{
    bool connected;
    bool paired;
    bool encrypted;
} app_ble_ota_link_snapshot_t;
void app_ble_ota_link_snapshot(uint8_t conn_id,
                               app_ble_ota_link_snapshot_t *snapshot);
#if ZY100_FINAL_EDGE_BLE_CTRL_ENABLE
bool app_ble_pairing_get_high_priority_identity(
    uint8_t peer_addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES],
    uint8_t *peer_type,
    bool *used_irk);
void app_ble_pairing_mark_ready(uint8_t conn_id, const char *reason);
#endif
/** End of PERIPH_APP
* @}
*/


#ifdef __cplusplus
}
#endif

#endif
