#include "zy100_production_acceptance.h"

#include "../app_build_config.h"

#if ZY100_BUILD_PRODUCTION

#include <string.h>

#include <gatt.h>
#include <trace.h>

#include "../app_factory/factory_boot_gate.h"
#include "../bsp/bsp_power_status.h"
#include "../common/zy100_byteorder.h"
#include "svc_yhm2712_charge.h"
#include "zy100_crc32.h"
#include "zy100_system_info_store.h"

#define ACCEPTANCE_VERSION                 2U
#define ACCEPTANCE_OP_ACCEPT_AND_SHIP      0x01U
#define ACCEPTANCE_EVENT_SHIP_ARMED        0x81U
#define ACCEPTANCE_EVENT_ERROR             0xE0U
#define ACCEPTANCE_STATUS_OK               0U
#define ACCEPTANCE_STATUS_BAD_FRAME        1U
#define ACCEPTANCE_STATUS_NOT_PENDING      2U
#define ACCEPTANCE_STATUS_MFG_CRC          3U
#define ACCEPTANCE_STATUS_EXTERNAL_POWER   4U
#define ACCEPTANCE_STATUS_PERSIST          5U
#define ACCEPTANCE_STATUS_BUSY             6U
#define ACCEPTANCE_STATUS_YHM              7U
#define ACCEPTANCE_STATUS_POWER_NOT_CUT    8U
#define ACCEPTANCE_COMMAND_INDEX           0x02U
#define ACCEPTANCE_STATUS_INDEX            0x04U
#define ACCEPTANCE_STATUS_CCCD_INDEX       0x05U
#define ACCEPTANCE_CONN_INVALID            0xFFU
#define ACCEPTANCE_NOTIFY_GRACE_MS         500ULL
#define ACCEPTANCE_POWER_CUT_TIMEOUT_MS    2000ULL

typedef enum
{
    ACCEPTANCE_IDLE = 0U,
    ACCEPTANCE_NOTIFY_ARMED,
    ACCEPTANCE_EXECUTE_SHIPPING,
    ACCEPTANCE_WAIT_POWER_CUT,
    ACCEPTANCE_NOTIFY_ERROR,
} acceptance_runtime_state_t;

static const uint8_t s_service_uuid[16] =
{
    0x00, 0x20, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
    0x4D, 0x4F, 0x9D, 0x4A, 0x0B, 0x00, 0xCA, 0x9E
};
static T_SERVER_ID s_service_id = 0xFFU;
static bool s_boot_active = false;
static char s_ble_name[10] = "ZV-000000";
static acceptance_runtime_state_t s_runtime_state = ACCEPTANCE_IDLE;
static uint8_t s_conn_id = ACCEPTANCE_CONN_INVALID;
static bool s_notify_enabled = false;
static bool s_status_pending = false;
static uint32_t s_request_id = 0UL;
static uint64_t s_deadline_ms = 0ULL;
static uint8_t s_status[ZY100_PRODUCTION_ACCEPTANCE_FRAME_BYTES];

static T_ATTRIB_APPL s_attr_tbl[] =
{
    {
        (ATTRIB_FLAG_VOID | ATTRIB_FLAG_LE),
        { LO_WORD(GATT_UUID_PRIMARY_SERVICE), HI_WORD(GATT_UUID_PRIMARY_SERVICE) },
        UUID_128BIT_SIZE,
        (void *)s_service_uuid,
        GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_INCL,
        { LO_WORD(GATT_UUID_CHARACTERISTIC), HI_WORD(GATT_UUID_CHARACTERISTIC),
          GATT_CHAR_PROP_WRITE },
        1, NULL, GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,
        { 0x00, 0x20, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
          0x4D, 0x4F, 0x9D, 0x4A, 0x0C, 0x00, 0xCA, 0x9E },
        0, NULL, GATT_PERM_WRITE
    },
    {
        ATTRIB_FLAG_VALUE_INCL,
        { LO_WORD(GATT_UUID_CHARACTERISTIC), HI_WORD(GATT_UUID_CHARACTERISTIC),
          (GATT_CHAR_PROP_READ | GATT_CHAR_PROP_NOTIFY) },
        1, NULL, GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,
        { 0x00, 0x20, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
          0x4D, 0x4F, 0x9D, 0x4A, 0x0D, 0x00, 0xCA, 0x9E },
        0, NULL, GATT_PERM_READ
    },
    {
        ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_CCCD_APPL,
        { LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
          HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
          LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT),
          HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT) },
        2, NULL, (GATT_PERM_READ | GATT_PERM_WRITE)
    },
};

static void acceptance_build_status(uint8_t event,
                                    uint8_t status,
                                    uint32_t detail)
{
    memset(s_status, 0, sizeof(s_status));
    memcpy(s_status, "PVAS", 4U);
    s_status[4] = ACCEPTANCE_VERSION;
    s_status[5] = event;
    s_status[6] = status;
    s_status[7] = (uint8_t)s_runtime_state;
    zy100_put_u32_le(&s_status[8], s_request_id);
    zy100_put_u32_le(&s_status[12], detail);
    zy100_put_u32_le(&s_status[16], zy100_crc32_ieee(s_status, 16U));
    s_status_pending = true;
}

void zy100_production_acceptance_boot_init(void)
{
    zy100_factory_acceptance_state_t state = ZY100_FACTORY_ACCEPTANCE_NONE;
    zy100_whole_unit_state_t whole_state = ZY100_WHOLE_UNIT_NOT_REQUIRED;
    const zp_mfg_record_t *record = factory_boot_gate_record();
    bool external_present = false;
    bsp_power_status_t power_status;
    size_t name_len;

    s_boot_active = false;
    s_runtime_state = ACCEPTANCE_IDLE;
    if (!factory_boot_gate_locked_user_mode_active() || (record == NULL))
    {
        return;
    }
    if (!zy100_system_info_get_manufacturing_states(&state, &whole_state))
    {
        s_boot_active = true;
        DBG_DIRECT("[PROD_ACCEPT][ERR] sys_info_v6_invalid fail_closed=1");
        return;
    }

    if (state == ZY100_FACTORY_ACCEPTANCE_SHIP_ARMED)
    {
        power_status = bsp_power_status_chg_int_external_power_present(
                           &external_present);
        /* SHIP_ARMED is the durable acknowledgement that the first
         * acceptance/ship command was accepted and the YHM shipping write
         * was allowed to proceed.  A cold reboot after that point must not
         * fall back to the ZV acceptance path merely because VIN is absent:
         * the next required state is the ZW whole-unit test. */
        if (whole_state == ZY100_WHOLE_UNIT_REQUIRED)
        {
            if (zy100_system_info_set_manufacturing_states(
                    ZY100_FACTORY_ACCEPTANCE_NONE,
                    ZY100_WHOLE_UNIT_ACTIVE))
            {
                DBG_DIRECT("[PROD_ACCEPT] ship_armed_reboot_to_whole_unit "
                           "status=%s external=%u whole_unit=active",
                           bsp_power_status_name(power_status),
                           external_present ? 1U : 0U);
                return;
            }
            /* Keep the durable SHIP_ARMED marker intact.  Expose a ZV
             * recovery advertisement from the RAM state below, but never
             * release the device into normal ZP operation when the ACTIVE
             * transition cannot be persisted. */
            DBG_DIRECT("[PROD_ACCEPT][ERR] ship_armed_reboot_transition_failed "
                       "status=%s external=%u fail_closed=1 retry=ship_armed",
                       bsp_power_status_name(power_status),
                       external_present ? 1U : 0U);
            state = ZY100_FACTORY_ACCEPTANCE_PENDING;
        }
        else
        {
            DBG_DIRECT("[PROD_ACCEPT] ship_armed_without_external status=%s external=%u retry=pending",
                       bsp_power_status_name(power_status),
                       external_present ? 1U : 0U);
            if (!zy100_system_info_set_manufacturing_states(
                    ZY100_FACTORY_ACCEPTANCE_PENDING,
                    ZY100_WHOLE_UNIT_REQUIRED))
            {
                DBG_DIRECT("[PROD_ACCEPT][ERR] ship_armed_recover_pending_failed "
                           "fail_closed=1");
            }
            whole_state = ZY100_WHOLE_UNIT_REQUIRED;
            state = ZY100_FACTORY_ACCEPTANCE_PENDING;
        }
    }

    if ((state != ZY100_FACTORY_ACCEPTANCE_PENDING) ||
        (whole_state != ZY100_WHOLE_UNIT_REQUIRED))
    {
        return;
    }
    name_len = strlen(record->ble_adv_name);
    if (name_len < 6U)
    {
        DBG_DIRECT("[PROD_ACCEPT][ERR] formal_name_invalid");
        return;
    }
    memcpy(s_ble_name, "ZV-", 3U);
    memcpy(&s_ble_name[3], &record->ble_adv_name[name_len - 6U], 6U);
    s_ble_name[9] = '\0';
    s_boot_active = true;
    DBG_DIRECT("[PROD_ACCEPT] boot pending adv=%s formal=%s pairing=disabled",
               s_ble_name, record->ble_adv_name);
}

bool zy100_production_acceptance_active(void)
{
    return s_boot_active;
}

const char *zy100_production_acceptance_ble_name(void)
{
    return s_ble_name;
}

static T_APP_RESULT acceptance_read_cb(uint8_t conn_id,
                                       T_SERVER_ID service_id,
                                       uint16_t attrib_index,
                                       uint16_t offset,
                                       uint16_t *length,
                                       uint8_t **value)
{
    (void)conn_id;
    (void)service_id;
    if ((attrib_index != ACCEPTANCE_STATUS_INDEX) ||
        (length == NULL) || (value == NULL) ||
        (offset > sizeof(s_status)))
    {
        return APP_RESULT_ATTR_NOT_FOUND;
    }
    *value = s_status + offset;
    *length = (uint16_t)(sizeof(s_status) - offset);
    return APP_RESULT_SUCCESS;
}

static bool acceptance_command_valid(const uint8_t *data, uint16_t len)
{
    return (data != NULL) &&
           (len == ZY100_PRODUCTION_ACCEPTANCE_FRAME_BYTES) &&
           (memcmp(data, "PVAC", 4U) == 0) &&
           (data[4] == ACCEPTANCE_VERSION) &&
           (data[5] == ACCEPTANCE_OP_ACCEPT_AND_SHIP) &&
           (data[6] == 0U) && (data[7] == 0U) &&
           (zy100_get_u32_le(&data[16]) == zy100_crc32_ieee(data, 16U));
}

static T_APP_RESULT acceptance_write_cb(
    uint8_t conn_id,
    T_SERVER_ID service_id,
    uint16_t attrib_index,
    T_WRITE_TYPE write_type,
    uint16_t length,
    uint8_t *value,
    P_FUN_WRITE_IND_POST_PROC *post_proc)
{
    const zp_mfg_record_t *record = factory_boot_gate_record();
    zy100_factory_acceptance_state_t persisted_state;
    zy100_whole_unit_state_t whole_state;
    bool external_present = true;
    uint32_t requested_crc;

    (void)service_id;
    if (post_proc != NULL)
    {
        *post_proc = NULL;
    }
    if ((attrib_index != ACCEPTANCE_COMMAND_INDEX) ||
        (write_type != WRITE_REQUEST))
    {
        return APP_RESULT_ATTR_NOT_FOUND;
    }
    if ((value != NULL) && (length >= 12U))
    {
        s_request_id = zy100_get_u32_le(&value[8]);
    }
    s_conn_id = conn_id;
    if (!acceptance_command_valid(value, length))
    {
        acceptance_build_status(ACCEPTANCE_EVENT_ERROR,
                                ACCEPTANCE_STATUS_BAD_FRAME, length);
        return APP_RESULT_APP_ERR;
    }
    if ((s_runtime_state != ACCEPTANCE_IDLE) || !s_notify_enabled ||
        (s_conn_id != conn_id))
    {
        acceptance_build_status(ACCEPTANCE_EVENT_ERROR,
                                ACCEPTANCE_STATUS_BUSY,
                                (uint32_t)s_runtime_state);
        return APP_RESULT_APP_ERR;
    }
    if (!zy100_system_info_get_manufacturing_states(&persisted_state,
                                                     &whole_state) ||
        (persisted_state != ZY100_FACTORY_ACCEPTANCE_PENDING) ||
        (whole_state != ZY100_WHOLE_UNIT_REQUIRED))
    {
        acceptance_build_status(ACCEPTANCE_EVENT_ERROR,
                                ACCEPTANCE_STATUS_NOT_PENDING,
                                (uint32_t)persisted_state);
        return APP_RESULT_APP_ERR;
    }
    requested_crc = zy100_get_u32_le(&value[12]);
    if ((record == NULL) || (requested_crc == 0UL) ||
        (record->reserved_u32[0] != requested_crc))
    {
        acceptance_build_status(ACCEPTANCE_EVENT_ERROR,
                                ACCEPTANCE_STATUS_MFG_CRC,
                                (record != NULL) ? record->reserved_u32[0] : 0UL);
        return APP_RESULT_APP_ERR;
    }
    if ((bsp_power_status_chg_int_external_power_present(&external_present) !=
         BSP_POWER_STATUS_OK) || external_present)
    {
        acceptance_build_status(ACCEPTANCE_EVENT_ERROR,
                                ACCEPTANCE_STATUS_EXTERNAL_POWER,
                                external_present ? 1UL : 2UL);
        return APP_RESULT_APP_ERR;
    }
    if (!zy100_system_info_set_manufacturing_states(
            ZY100_FACTORY_ACCEPTANCE_SHIP_ARMED,
            ZY100_WHOLE_UNIT_REQUIRED))
    {
        acceptance_build_status(ACCEPTANCE_EVENT_ERROR,
                                ACCEPTANCE_STATUS_PERSIST, 0UL);
        return APP_RESULT_APP_ERR;
    }

    s_runtime_state = ACCEPTANCE_NOTIFY_ARMED;
    s_deadline_ms = 0ULL;
    acceptance_build_status(ACCEPTANCE_EVENT_SHIP_ARMED,
                            ACCEPTANCE_STATUS_OK, requested_crc);
    DBG_DIRECT("[PROD_ACCEPT] ship_armed conn=%u request=%lu crc=0x%08lX",
               conn_id, (unsigned long)s_request_id,
               (unsigned long)requested_crc);
    return APP_RESULT_SUCCESS;
}

static void acceptance_cccd_cb(uint8_t conn_id,
                               T_SERVER_ID service_id,
                               uint16_t index,
                               uint16_t ccc_bits)
{
    (void)service_id;
    if (index != ACCEPTANCE_STATUS_CCCD_INDEX)
    {
        return;
    }
    if ((ccc_bits & GATT_CLIENT_CHAR_CONFIG_NOTIFY) != 0U)
    {
        s_conn_id = conn_id;
        s_notify_enabled = true;
    }
    else if (s_conn_id == conn_id)
    {
        s_notify_enabled = false;
    }
}

static const T_FUN_GATT_SERVICE_CBS s_callbacks =
{
    acceptance_read_cb,
    acceptance_write_cb,
    acceptance_cccd_cb
};

T_SERVER_ID zy100_production_acceptance_add_service(void *callback)
{
    (void)callback;
    s_conn_id = ACCEPTANCE_CONN_INVALID;
    s_notify_enabled = false;
    s_status_pending = false;
    memset(s_status, 0, sizeof(s_status));
    if (!server_add_service(&s_service_id,
                            (uint8_t *)s_attr_tbl,
                            sizeof(s_attr_tbl),
                            s_callbacks))
    {
        s_service_id = 0xFFU;
        DBG_DIRECT("[PROD_ACCEPT][ERR] service_add_failed");
    }
    return s_service_id;
}

void zy100_production_acceptance_poll(uint64_t runtime_ms)
{
    yhm2712_acmd_status_t yhm_status;

    if (s_status_pending && s_notify_enabled &&
        server_send_data(s_conn_id, s_service_id,
                         ACCEPTANCE_STATUS_INDEX,
                         s_status, sizeof(s_status),
                         GATT_PDU_TYPE_NOTIFICATION))
    {
        s_status_pending = false;
        s_deadline_ms = runtime_ms + ACCEPTANCE_NOTIFY_GRACE_MS;
    }

    if ((s_runtime_state == ACCEPTANCE_NOTIFY_ARMED) &&
        !s_status_pending && (s_deadline_ms != 0ULL) &&
        (runtime_ms >= s_deadline_ms))
    {
        s_runtime_state = ACCEPTANCE_EXECUTE_SHIPPING;
        s_deadline_ms = 0ULL;
    }
    if (s_runtime_state == ACCEPTANCE_EXECUTE_SHIPPING)
    {
        DBG_DIRECT("[PROD_ACCEPT] shipping_execute request=%lu",
                   (unsigned long)s_request_id);
        yhm_status = svc_yhm2712_charge_factory_enter_shipping();
        if (yhm_status != YHM2712_ACMD_STATUS_OK)
        {
            (void)zy100_system_info_set_manufacturing_states(
                ZY100_FACTORY_ACCEPTANCE_PENDING,
                ZY100_WHOLE_UNIT_REQUIRED);
            s_runtime_state = ACCEPTANCE_NOTIFY_ERROR;
            acceptance_build_status(ACCEPTANCE_EVENT_ERROR,
                                    ACCEPTANCE_STATUS_YHM,
                                    (uint32_t)yhm_status);
            DBG_DIRECT("[PROD_ACCEPT][ERR] shipping_failed status=%s retry=pending",
                       yhm2712_acmd_status_name(yhm_status));
            return;
        }
        s_runtime_state = ACCEPTANCE_WAIT_POWER_CUT;
        s_deadline_ms = runtime_ms + ACCEPTANCE_POWER_CUT_TIMEOUT_MS;
        DBG_DIRECT("[PROD_ACCEPT] shipping_write_ok wait_power_cut_ms=%lu",
                   (unsigned long)ACCEPTANCE_POWER_CUT_TIMEOUT_MS);
    }
    if ((s_runtime_state == ACCEPTANCE_WAIT_POWER_CUT) &&
        (runtime_ms >= s_deadline_ms))
    {
        (void)zy100_system_info_set_manufacturing_states(
            ZY100_FACTORY_ACCEPTANCE_PENDING,
            ZY100_WHOLE_UNIT_REQUIRED);
        s_runtime_state = ACCEPTANCE_NOTIFY_ERROR;
        acceptance_build_status(ACCEPTANCE_EVENT_ERROR,
                                ACCEPTANCE_STATUS_POWER_NOT_CUT, 0UL);
        DBG_DIRECT("[PROD_ACCEPT][ERR] power_not_cut retry=pending");
    }
    if ((s_runtime_state == ACCEPTANCE_NOTIFY_ERROR) && !s_status_pending)
    {
        s_runtime_state = ACCEPTANCE_IDLE;
        s_deadline_ms = 0ULL;
    }
}

void zy100_production_acceptance_on_disconnected(uint8_t conn_id)
{
    if (s_conn_id == conn_id)
    {
        s_conn_id = ACCEPTANCE_CONN_INVALID;
        s_notify_enabled = false;
        s_status_pending = false;
    }
}

#else

void zy100_production_acceptance_boot_init(void) {}
bool zy100_production_acceptance_active(void) { return false; }
const char *zy100_production_acceptance_ble_name(void) { return ""; }
T_SERVER_ID zy100_production_acceptance_add_service(void *callback)
{
    (void)callback;
    return 0xFFU;
}
void zy100_production_acceptance_poll(uint64_t runtime_ms) { (void)runtime_ms; }
void zy100_production_acceptance_on_disconnected(uint8_t conn_id)
{
    (void)conn_id;
}

#endif
