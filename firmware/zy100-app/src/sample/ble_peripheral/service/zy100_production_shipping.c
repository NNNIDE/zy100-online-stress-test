#include "../app_build_config.h"

#include "zy100_production_shipping.h"

#if ZY100_BUILD_PRODUCTION

#include <trace.h>

#include "../bsp/bsp_power_status.h"
#include "svc_yhm2712_charge.h"
#include "zy100_ble_ctrl_protocol.h"
#include "zy100_ble_ctrl_service.h"
#include "zy100_system_info_store.h"

/* These are the established Production Acceptance transaction windows. */
#define PRODUCTION_SHIPPING_NOTIFY_GRACE_MS      500ULL
#define PRODUCTION_SHIPPING_POWER_CUT_TIMEOUT_MS 2000ULL

typedef enum
{
    SHIPPING_IDLE = 0U,
    SHIPPING_NOTIFY_ARMED,
    SHIPPING_EXECUTE,
    SHIPPING_WAIT_POWER_CUT,
    SHIPPING_NOTIFY_ERROR,
} shipping_state_t;

static shipping_state_t s_state = SHIPPING_IDLE;
static uint8_t s_conn_id = 0xFFU;
static uint8_t s_seq;
static uint32_t s_user_id;
static uint32_t s_training_id;
static uint64_t s_deadline_ms;
static bool s_boot_blocked;
static bool s_boot_wake_pending;

static void shipping_notify_error(uint8_t status, uint32_t detail)
{
    (void)zy100_ble_ctrl_service_notify_async_result_ex(
        s_conn_id,
        ZY100_BLE_CMD_ENTER_SHIPPING,
        s_seq,
        status,
        ZY100_BLE_DEVICE_STATE_WAIT_START,
        ZY100_BLE_EXEC_MODE_ASYNC_DONE,
        0U,
        s_user_id,
        s_training_id,
        detail);
}

void zy100_production_shipping_boot_init(void)
{
    zy100_production_shipping_state_t persisted =
        ZY100_PRODUCTION_SHIPPING_NONE;
    bool external_present = false;
    bsp_power_status_t power_status;

    s_state = SHIPPING_IDLE;
    s_boot_blocked = false;
    s_boot_wake_pending = false;
    if (!zy100_system_info_get_production_shipping_state(&persisted))
    {
        s_boot_blocked = true;
        DBG_DIRECT("[PROD_SHIP][ERR] sys_info_shipping_state_invalid block=1");
        return;
    }
    if (persisted != ZY100_PRODUCTION_SHIPPING_ARMED)
    {
        return;
    }
    power_status = bsp_power_status_chg_int_external_power_present(
                       &external_present);
    if ((power_status == BSP_POWER_STATUS_OK) && external_present)
    {
        if (zy100_system_info_set_production_shipping_state(
                ZY100_PRODUCTION_SHIPPING_NONE))
        {
            s_boot_wake_pending = true;
            DBG_DIRECT("[PROD_SHIP] vin_wake marker_consumed=1");
            return;
        }
    }
    s_boot_blocked = true;
    DBG_DIRECT("[PROD_SHIP][ERR] armed_without_vin power=%s external=%u block=1",
               bsp_power_status_name(power_status),
               external_present ? 1U : 0U);
}

bool zy100_production_shipping_boot_blocked(void)
{
    return s_boot_blocked;
}

bool zy100_production_shipping_boot_wake_pending_take(void)
{
    bool pending = s_boot_wake_pending;
    s_boot_wake_pending = false;
    return pending;
}

bool zy100_production_shipping_active(void)
{
    return s_state != SHIPPING_IDLE;
}

bool zy100_production_shipping_arm(uint8_t conn_id,
                                   uint8_t seq,
                                   uint32_t user_id,
                                   uint32_t training_id)
{
    bool external_present = true;

    if ((s_state != SHIPPING_IDLE) ||
        !zy100_ble_ctrl_service_ack_notify_enabled(conn_id) ||
        (bsp_power_status_chg_int_external_power_present(&external_present) !=
         BSP_POWER_STATUS_OK) || external_present ||
        !zy100_system_info_set_production_shipping_state(
            ZY100_PRODUCTION_SHIPPING_ARMED))
    {
        return false;
    }
    s_conn_id = conn_id;
    s_seq = seq;
    s_user_id = user_id;
    s_training_id = training_id;
    s_deadline_ms = 0ULL;
    s_state = SHIPPING_NOTIFY_ARMED;
    DBG_DIRECT("[PROD_SHIP] armed conn=%u seq=%u", conn_id, seq);
    return true;
}

void zy100_production_shipping_poll(uint64_t runtime_ms)
{
    yhm2712_acmd_status_t yhm_status;

    if ((s_state == SHIPPING_NOTIFY_ARMED) && (s_deadline_ms == 0ULL))
    {
        s_deadline_ms = runtime_ms + PRODUCTION_SHIPPING_NOTIFY_GRACE_MS;
    }
    if ((s_state == SHIPPING_NOTIFY_ARMED) &&
        (runtime_ms >= s_deadline_ms))
    {
        s_state = SHIPPING_EXECUTE;
        s_deadline_ms = 0ULL;
    }
    if (s_state == SHIPPING_EXECUTE)
    {
        yhm_status = svc_yhm2712_charge_factory_enter_shipping();
        if (yhm_status != YHM2712_ACMD_STATUS_OK)
        {
            (void)zy100_system_info_set_production_shipping_state(
                ZY100_PRODUCTION_SHIPPING_NONE);
            s_state = SHIPPING_NOTIFY_ERROR;
            shipping_notify_error(ZY100_BLE_ACK_STATUS_INTERNAL_ERROR,
                                  (uint32_t)yhm_status);
            DBG_DIRECT("[PROD_SHIP][ERR] yhm=%s", yhm2712_acmd_status_name(yhm_status));
            return;
        }
        s_state = SHIPPING_WAIT_POWER_CUT;
        s_deadline_ms = runtime_ms + PRODUCTION_SHIPPING_POWER_CUT_TIMEOUT_MS;
        return;
    }
    if ((s_state == SHIPPING_WAIT_POWER_CUT) &&
        (runtime_ms >= s_deadline_ms))
    {
        (void)zy100_system_info_set_production_shipping_state(
            ZY100_PRODUCTION_SHIPPING_NONE);
        s_state = SHIPPING_NOTIFY_ERROR;
        shipping_notify_error(ZY100_BLE_ACK_STATUS_NOT_READY, 0U);
        DBG_DIRECT("[PROD_SHIP][ERR] power_not_cut");
    }
    if ((s_state == SHIPPING_NOTIFY_ERROR) &&
        zy100_ble_ctrl_service_ack_pipeline_idle())
    {
        s_state = SHIPPING_IDLE;
        s_deadline_ms = 0ULL;
    }
}

void zy100_production_shipping_on_disconnected(uint8_t conn_id)
{
    if (s_conn_id == conn_id)
    {
        /* The marker stays armed: a successful shipping transition is
         * intentionally completed by VIN, not by a BLE disconnect. */
        s_conn_id = 0xFFU;
    }
}

#else

void zy100_production_shipping_boot_init(void) {}
bool zy100_production_shipping_boot_blocked(void) { return false; }
bool zy100_production_shipping_boot_wake_pending_take(void) { return false; }
bool zy100_production_shipping_active(void) { return false; }
bool zy100_production_shipping_arm(uint8_t conn_id, uint8_t seq,
                                   uint32_t user_id, uint32_t training_id)
{
    (void)conn_id; (void)seq; (void)user_id; (void)training_id;
    return false;
}
void zy100_production_shipping_poll(uint64_t runtime_ms) { (void)runtime_ms; }
void zy100_production_shipping_on_disconnected(uint8_t conn_id) { (void)conn_id; }

#endif
