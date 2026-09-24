#include <string.h>
#include "gatt.h"
#include "gap_conn_le.h"
#include "patch_header_check.h"
#include "rtl876x_wdg.h"
#include "flash_device.h"
#include "dfu_flash.h"
#include "dfu_api.h"
#include "dfu_service.h"
#include "dfu_watchdog.h"
#include "otp.h"
#include "trace.h"
#include "board.h"
#include "os_sched.h"

#ifndef ZY100_LOG_DFU_INFO_ENABLE
#define ZY100_LOG_DFU_INFO_ENABLE 0
#endif

#if !ZY100_LOG_DFU_INFO_ENABLE
#undef DFU_PRINT_INFO0
#undef DFU_PRINT_INFO1
#undef DFU_PRINT_INFO2
#undef DFU_PRINT_INFO3
#undef DFU_PRINT_INFO4
#undef DFU_PRINT_INFO5
#undef DFU_PRINT_INFO6
#undef DFU_PRINT_TRACE0
#undef DFU_PRINT_TRACE1
#undef DFU_PRINT_TRACE2
#undef DFU_PRINT_TRACE3
#define DFU_PRINT_INFO0(...)
#define DFU_PRINT_INFO1(...)
#define DFU_PRINT_INFO2(...)
#define DFU_PRINT_INFO3(...)
#define DFU_PRINT_INFO4(...)
#define DFU_PRINT_INFO5(...)
#define DFU_PRINT_INFO6(...)
#define DFU_PRINT_TRACE0(...)
#define DFU_PRINT_TRACE1(...)
#define DFU_PRINT_TRACE2(...)
#define DFU_PRINT_TRACE3(...)
#endif

/*============================================================================*
 *                              Macros
 *============================================================================*/

//#define CAL_OFFSET(type, member) ((size_t)(&((type *)0)->member))

/*============================================================================*
 *                              External Variables
 *============================================================================*/
T_DFU_PARA g_dfu_para;
uint8_t *p_ota_temp_buffer_head;
uint16_t g_ota_tmp_buf_used_size;

/*============================================================================*
 *                              Local Variables
 *============================================================================*/
uint8_t ota_temp_buffer_head[DFU_TEMP_BUFFER_SIZE];
static bool buffer_check_en = false;
static uint32_t dfu_resend_offset = 0;
static T_SERVER_ID dfu_service_id;

typedef struct
{
    bool valid;
    uint16_t image_id;
    uint32_t image_total_length;
} T_DFU_VALIDATED_IMAGE_STATE;

static T_DFU_VALIDATED_IMAGE_STATE s_dfu_validated_image = {false, 0U, 0U};

/* All transfer state is owned by dfu_main_task: GATT callbacks run from
 * gap_handle_msg(), and deferred validation/activation run from its IO queue.
 * This is an inactivity deadline, independent of the hardware watchdog. */
#if ZY100_BUILD_PRODUCTION
#define DFU_TRANSFER_IDLE_MS 10000U
#define DFU_TRANSFER_POLL_MS 250U
typedef enum
{
    DFU_TRANSFER_IDLE = 0,
    DFU_TRANSFER_RECEIVING,
    DFU_TRANSFER_WAIT_VALIDATE,
    DFU_TRANSFER_VALIDATING,
    DFU_TRANSFER_WAIT_ACTIVATE,
    DFU_TRANSFER_COMMITTING,
    DFU_TRANSFER_EXPIRED
} T_DFU_TRANSFER_STAGE;

static T_DFU_TRANSFER_STAGE s_dfu_transfer_stage;
static T_DFU_TRANSFER_STAGE s_dfu_transfer_expired_stage;
static uint32_t s_dfu_transfer_last_ms;
static bool s_dfu_transfer_connected;
static bool s_dfu_transfer_timeout_reported;

static bool dfu_transfer_waiting(void)
{
    return s_dfu_transfer_stage == DFU_TRANSFER_RECEIVING ||
           s_dfu_transfer_stage == DFU_TRANSFER_WAIT_VALIDATE ||
           s_dfu_transfer_stage == DFU_TRANSFER_WAIT_ACTIVATE;
}

static bool dfu_transfer_expired(void)
{
    if (dfu_transfer_waiting() &&
        (uint32_t)((uint32_t)os_sys_time_get() - s_dfu_transfer_last_ms) >=
        DFU_TRANSFER_IDLE_MS)
    {
        s_dfu_transfer_expired_stage = s_dfu_transfer_stage;
        s_dfu_transfer_stage = DFU_TRANSFER_EXPIRED;
    }
    return s_dfu_transfer_stage == DFU_TRANSFER_EXPIRED;
}

static void dfu_transfer_wait(T_DFU_TRANSFER_STAGE stage)
{
    s_dfu_transfer_stage = stage;
    s_dfu_transfer_last_ms = (uint32_t)os_sys_time_get();
}

static void dfu_transfer_committed(void)
{
    if (s_dfu_transfer_stage == DFU_TRANSFER_RECEIVING &&
        g_dfu_para.cur_offset == g_dfu_para.image_total_length)
    {
        dfu_transfer_wait(DFU_TRANSFER_WAIT_VALIDATE);
    }
}
#endif

/* Called only by the DFU task, outside stack callbacks and flash operations.
 * Return true also if a test/reset port returns, so late work stays fenced. */
bool dfu_service_poll_transfer_timeout(void)
{
#if ZY100_BUILD_PRODUCTION
    if (dfu_transfer_expired())
    {
        if (!s_dfu_transfer_timeout_reported)
        {
            s_dfu_transfer_timeout_reported = true;
            s_dfu_validated_image.valid = false;
            DBG_DIRECT("[DFU_STALL] stage=%u idle_ms=%lu committed=%lu buffered=%u connected=%u",
                       (unsigned int)s_dfu_transfer_expired_stage,
                       (unsigned long)((uint32_t)os_sys_time_get() - s_dfu_transfer_last_ms),
                       (unsigned long)g_dfu_para.cur_offset,
                       (unsigned int)g_ota_tmp_buf_used_size,
                       s_dfu_transfer_connected ? 1U : 0U);
            dfu_fw_reboot(false);
        }
        return true;
    }
#endif
    return false;
}

uint32_t dfu_service_transfer_wait_ms(uint32_t default_wait_ms)
{
#if ZY100_BUILD_PRODUCTION
    if (dfu_transfer_waiting() && default_wait_ms > DFU_TRANSFER_POLL_MS)
    {
        return DFU_TRANSFER_POLL_MS;
    }
#endif
    return default_wait_ms;
}

void dfu_service_transfer_link_state(uint8_t conn_id, bool connected, uint16_t cause)
{
#if ZY100_BUILD_PRODUCTION
    s_dfu_transfer_connected = connected;
    if (!connected)
    {
        DBG_DIRECT("[DFU_LINK] disconnected conn=%u cause=0x%x stage=%u offset=%lu",
                   conn_id, cause, (unsigned int)s_dfu_transfer_stage,
                   (unsigned long)g_dfu_para.cur_offset);
    }
#else
    (void)conn_id;
    (void)connected;
    (void)cause;
#endif
}

/* Preserve wire payloads; retain failures even with routine logs disabled. */
static void dfu_service_notify(uint8_t conn_id, uint8_t *data, uint16_t length)
{
    bool sent = server_send_data(conn_id, dfu_service_id,
                                 INDEX_DFU_CONTROL_POINT_CHAR_VALUE,
                                 data, length, GATT_PDU_TYPE_NOTIFICATION);
#if ZY100_BUILD_PRODUCTION
    if (!sent)
    {
        DBG_DIRECT("[DFU_NOTIFY_FAIL] conn=%u opcode=0x%x offset=%lu",
                   conn_id, data[1], (unsigned long)g_dfu_para.cur_offset);
    }
#else
    (void)sent;
#endif
}

P_FUN_SERVER_GENERAL_CB pfn_dfu_service_cb = NULL;

const uint8_t   SILENCE_GATT_UUID128_DFU_SERVICE[16] = {GATT_UUID128_DFU_SERVICE};

T_ATTRIB_APPL gatt_dfu_service_table[] =
{

    /*-------------------------- DFU Service ---------------------------*/
    /* <<Primary Service>>, .. */
    {
        (ATTRIB_FLAG_VOID | ATTRIB_FLAG_LE),           /* flags     */
        {
            LO_WORD(GATT_UUID_PRIMARY_SERVICE),
            HI_WORD(GATT_UUID_PRIMARY_SERVICE),              /* type_value */
        },
        UUID_128BIT_SIZE,                                    /* bValueLen     */
        (void *)SILENCE_GATT_UUID128_DFU_SERVICE,           /* p_value_context */
        GATT_PERM_READ                                      /* permissions  */
    },



    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* flags */
        {                                           /* type_value */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            GATT_CHAR_PROP_WRITE_NO_RSP/* characteristic properties */
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* permissions */
    },
    /*--- DFU packet characteristic value ---*/
    {
        ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,                              /* flags */
        {                                                           /* type_value */
            GATT_UUID128_DFU_DATA
        },
        0,                                                 /* bValueLen */
        NULL,
#if DFU_SERVER_REQUIRE_AUTH
        GATT_PERM_WRITE_AUTHEN_MITM_REQ                             /* permissions */
#else
        GATT_PERM_WRITE           /* permissions */
#endif
    },
    /* <<Characteristic>>, .. */
    {
        ATTRIB_FLAG_VALUE_INCL,                     /* flags */
        {                                           /* type_value */
            LO_WORD(GATT_UUID_CHARACTERISTIC),
            HI_WORD(GATT_UUID_CHARACTERISTIC),
            (GATT_CHAR_PROP_WRITE |                   /* characteristic properties */
             GATT_CHAR_PROP_NOTIFY)
            /* characteristic UUID not needed here, is UUID of next attrib. */
        },
        1,                                          /* bValueLen */
        NULL,
        GATT_PERM_READ                              /* permissions */
    },
    /*--- DFU Control Point value ---*/
    {
        ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,                              /* flags */
        {                                                           /* type_value */
            GATT_UUID128_DFU_CONTROL_POINT
        },
        0,                                                 /* bValueLen */
        NULL,
#if DFU_SERVER_REQUIRE_AUTH
        GATT_PERM_WRITE_AUTHEN_MITM_REQ                             /* permissions */
#else
        GATT_PERM_WRITE           /* permissions */
#endif
    },
    /* client characteristic configuration */
    {
        (ATTRIB_FLAG_VALUE_INCL |                   /* flags */
         ATTRIB_FLAG_CCCD_APPL),
        {                                           /* type_value */
            LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
            /* NOTE: this value has an instantiation for each client, a write to */
            /* this attribute does not modify this default value:                */
            LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT), /* client char. config. bit field */
            HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT)
        },
        2,                                          /* bValueLen */
        NULL,
#if DFU_SERVER_REQUIRE_AUTH
        (GATT_PERM_READ | GATT_PERM_WRITE_AUTHEN_MITM_REQ)          /* permissions */
#else
        (GATT_PERM_READ | GATT_PERM_WRITE)          /* permissions */
#endif
    }

};


/*============================================================================*
 *                              Local Functions
 *============================================================================*/
static void dfu_service_invalidate_validated_image(void)
{
    s_dfu_validated_image.valid = false;
    s_dfu_validated_image.image_id = 0U;
    s_dfu_validated_image.image_total_length = 0U;
}

/**
 * @brief dfu_buffer_check_process
 *
 * @param buffer_check_size     size for buffer check.
 * @param buffer_crc            calced buffer crc value.
 * @return None
*/
void dfu_buffer_check_process(uint8_t conn_id, uint16_t buffer_check_size, uint16_t buffer_crc)
{
    uint8_t notif_data[DFU_NOTIFY_LENGTH_REPORT_BUFFER_CRC] = {0};
    notif_data[0] = DFU_OPCODE_NOTIFICATION;
    notif_data[1] = DFU_OPCODE_REPORT_BUFFER_CRC;

    T_DFU_CALLBACK_DATA callback_data;
    callback_data.conn_id = conn_id;
    callback_data.msg_type = SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE;
    callback_data.msg_data.write.write_attrib_index = INDEX_DFU_CONTROL_POINT_CHAR_VALUE;

#if ZY100_BUILD_PRODUCTION
    if (dfu_transfer_expired() || s_dfu_transfer_stage != DFU_TRANSFER_RECEIVING ||
        g_ota_tmp_buf_used_size == 0U)
    {
        return;
    }
#endif
    dfu_service_invalidate_validated_image();

    if (buffer_check_size > DFU_TEMP_BUFFER_SIZE)
    {
        //invalid para
        DFU_PRINT_ERROR3("<==dfu_buffer_check_process: invalid buffer_check_size=%d(>%d), cur_offset=%d",
                         buffer_check_size, DFU_TEMP_BUFFER_SIZE, g_dfu_para.cur_offset);
        g_ota_tmp_buf_used_size = 0;
        notif_data[2] = DFU_ARV_FAIL_INVALID_PARAMETER;
        LE_UINT32_TO_ARRAY(&notif_data[3], g_dfu_para.cur_offset);
        dfu_service_notify(conn_id, notif_data, DFU_NOTIFY_LENGTH_REPORT_BUFFER_CRC);
        return;
    }

    if (g_ota_tmp_buf_used_size == buffer_check_size ||
        g_dfu_para.cur_offset + g_ota_tmp_buf_used_size == g_dfu_para.image_total_length)
    {
        if (dfu_check_buf_crc(p_ota_temp_buffer_head, g_ota_tmp_buf_used_size, buffer_crc))
        {
            //crc error
            DFU_PRINT_ERROR1("<==dfu_buffer_check_process: Buf CRC Error! cur_offset=%d",
                             g_dfu_para.cur_offset);
            g_ota_tmp_buf_used_size = 0;
            notif_data[2] = DFU_ARV_FAIL_CRC_ERROR;
            LE_UINT32_TO_ARRAY(&notif_data[3], g_dfu_para.cur_offset);
            dfu_service_notify(conn_id, notif_data, DFU_NOTIFY_LENGTH_REPORT_BUFFER_CRC);
            return;
        }

        else //crc ok
        {
            //1. devrypt data
            if (OTP->ota_with_encryption_data)
            {
                dfu_hw_aes_decrypt_image(p_ota_temp_buffer_head, p_ota_temp_buffer_head, g_ota_tmp_buf_used_size);
            }
            //2. write flash
            uint32_t result = dfu_update(g_dfu_para.ctrl_header.image_id, g_dfu_para.cur_offset,
                                         g_ota_tmp_buf_used_size, (uint32_t *)p_ota_temp_buffer_head);

            if (result == 0)
            {
                uint32_t updated_success_len = g_ota_tmp_buf_used_size;
                callback_data.msg_data.write.opcode = DFU_WRITE_DOING;
                callback_data.msg_data.write.length = 4;
                callback_data.msg_data.write.p_value = (uint8_t *)&updated_success_len;
                if (pfn_dfu_service_cb)
                {
                    T_APP_RESULT w_cause = pfn_dfu_service_cb(dfu_service_id, (void *)&callback_data);
                    if (w_cause != APP_RESULT_SUCCESS)
                    {
                        return;
                    }
                }

                g_dfu_para.cur_offset += g_ota_tmp_buf_used_size;

                if ((g_dfu_para.cur_offset - dfu_resend_offset) >= FMC_SEC_SECTION_LEN)
                {
                    dfu_resend_offset += FMC_SEC_SECTION_LEN;
                }
                g_ota_tmp_buf_used_size = 0;
                if (!dfu_watchdog_note_progress(
                        DFU_WATCHDOG_PROGRESS_BUFFER_COMMITTED,
                        g_dfu_para.cur_offset))
                {
                    DBG_DIRECT("[DFU_WDG][FAULT] committed offset progress rejected offset=%lu",
                               (unsigned long)g_dfu_para.cur_offset);
                    notif_data[2] = DFU_ARV_FAIL_OPERATION;
                    LE_UINT32_TO_ARRAY(&notif_data[3], g_dfu_para.cur_offset);
                    server_send_data(conn_id, dfu_service_id,
                                     INDEX_DFU_CONTROL_POINT_CHAR_VALUE,
                                     notif_data,
                                     DFU_NOTIFY_LENGTH_REPORT_BUFFER_CRC,
                                     GATT_PDU_TYPE_NOTIFICATION);
                    return;
                }
#if ZY100_BUILD_PRODUCTION
                dfu_transfer_committed();
#endif
                DFU_PRINT_INFO2("<==dfu_buffer_check_process: dfu_update Success! cur_offset=%d, dfu_resend_offset=%d",
                                g_dfu_para.cur_offset, dfu_resend_offset);
                notif_data[2] = DFU_ARV_SUCCESS; //valid
                LE_UINT32_TO_ARRAY(&notif_data[3], g_dfu_para.cur_offset);
                dfu_service_notify(conn_id, notif_data, DFU_NOTIFY_LENGTH_REPORT_BUFFER_CRC);
                return;
            }
            else
            {
                DFU_PRINT_ERROR1("<==dfu_buffer_check_process: dfu_update Fail result=%d", result);

                result = dfu_flash_erase_sector_with_retry(g_dfu_para.ctrl_header.image_id, dfu_resend_offset);
                if (result)
                {
                    //erase fail
                    g_ota_tmp_buf_used_size = 0;
                    g_dfu_para.cur_offset = dfu_resend_offset;
                    DFU_PRINT_ERROR1("<==dfu_buffer_check_process: erase flash fail 3 times! cur_offset=%d",
                                     g_dfu_para.cur_offset);

                    notif_data[2] = DFU_ARV_FAIL_ERASE_ERROR;
                    LE_UINT32_TO_ARRAY(&notif_data[3], g_dfu_para.cur_offset);
                    dfu_service_notify(conn_id, notif_data, DFU_NOTIFY_LENGTH_REPORT_BUFFER_CRC);
                    return;
                }

                if ((g_dfu_para.cur_offset - dfu_resend_offset) > FMC_SEC_SECTION_LEN) //need erase two sector
                {
                    DFU_PRINT_INFO0("<==dfu_buffer_check_process:Need erase two sectors");
                    result = dfu_flash_erase_sector_with_retry(g_dfu_para.ctrl_header.image_id,
                                                               dfu_resend_offset + FMC_SEC_SECTION_LEN);
                    if (result)
                    {
                        //erase fail
                        g_ota_tmp_buf_used_size = 0;
                        g_dfu_para.cur_offset =  dfu_resend_offset;
                        DFU_PRINT_ERROR1("<==dfu_buffer_check_process: erase flash fail 3 times! cur_offset=%d",
                                         g_dfu_para.cur_offset);

                        notif_data[2] = DFU_ARV_FAIL_ERASE_ERROR;
                        LE_UINT32_TO_ARRAY(&notif_data[3], g_dfu_para.cur_offset);
                        dfu_service_notify(conn_id, notif_data, DFU_NOTIFY_LENGTH_REPORT_BUFFER_CRC);
                        return;
                    }

                }
                //erase ok
                g_ota_tmp_buf_used_size = 0;
                g_dfu_para.cur_offset =  dfu_resend_offset;
                DFU_PRINT_INFO1("<==dfu_buffer_check_process: erase ok! cur_offset=%d", g_dfu_para.cur_offset);
                notif_data[2] = DFU_ARV_FAIL_PROG_ERROR;
                LE_UINT32_TO_ARRAY(&notif_data[3], g_dfu_para.cur_offset);
                dfu_service_notify(conn_id, notif_data, DFU_NOTIFY_LENGTH_REPORT_BUFFER_CRC);
                return;
            }
        }
    }
    else
    {
        DFU_PRINT_ERROR4("<==dfu_buffer_check_process: Error buffer_check_size=%d,buf_used_size=%d,cur_offset=%d,image_total_length=%d",
                         buffer_check_size, g_ota_tmp_buf_used_size,
                         g_dfu_para.cur_offset, g_dfu_para.image_total_length);
        //flush buffer.
        g_ota_tmp_buf_used_size = 0;
        notif_data[2] = DFU_ARV_FAIL_LENGTH_ERROR;
        LE_UINT32_TO_ARRAY(&notif_data[3], g_dfu_para.cur_offset);
        dfu_service_notify(conn_id, notif_data, DFU_NOTIFY_LENGTH_REPORT_BUFFER_CRC);
        return;
    }
}

void dfu_notify_conn_para_update_req(uint8_t conn_id, T_DFU_ARV_ERROR_CODE error_code)
{
    if (g_dfu_para.dfu_conn_para_update_in_progress == true)
    {
        bool notify_remote = g_dfu_para.dfu_conn_para_update_notify_remote;
        uint8_t notif_data[3] = {0};

        g_dfu_para.dfu_conn_para_update_in_progress = false;
        g_dfu_para.dfu_conn_para_update_notify_remote = false;
        if (!notify_remote)
        {
            DFU_PRINT_INFO1("<==dfu_notify_conn_para_update_req local_only error_code=0x%x", error_code);
            return;
        }

        notif_data[0] = DFU_OPCODE_NOTIFICATION;
        notif_data[1] = DFU_OPCODE_CONN_PARA_TO_UPDATE_REQ;
        notif_data[2] = error_code;

        DFU_PRINT_INFO1("<==dfu_notify_conn_para_update_req error_code=0x%x", error_code);
        /* Connection Param Update rejected, we should notify the fail result to remote device. */
        server_send_data(conn_id, dfu_service_id, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                         notif_data, DFU_NOTIFY_LENGTH_CONN_PARA_UPDATE_REQ, GATT_PDU_TYPE_NOTIFICATION);
    }
}

void dfu_service_handle_valid_fw(uint8_t conn_id)
{
    uint8_t notif_data[DFU_NOTIFY_LENGTH_VALID_FW] = {0};
    bool check_result = false;
#if ZY100_BUILD_PRODUCTION
    T_DFU_TRANSFER_STAGE previous_stage = s_dfu_transfer_stage;
    if (dfu_transfer_expired() ||
        (previous_stage != DFU_TRANSFER_WAIT_VALIDATE &&
         previous_stage != DFU_TRANSFER_WAIT_ACTIVATE))
    {
        return;
    }
    /* Paused only while actually running checksum, never while queued. */
    s_dfu_transfer_stage = DFU_TRANSFER_VALIDATING;
#endif

    dfu_service_invalidate_validated_image();
    if (g_dfu_para.cur_offset == g_dfu_para.image_total_length)
    {
        check_result = dfu_validate_checksum_only(g_dfu_para.ctrl_header.image_id);
    }
    DFU_PRINT_INFO1("DFU_OPCODE_VALID_FW: check_result=%d (1: Success, 0: Fail)", check_result);

    if (check_result)
    {
        check_result = dfu_watchdog_note_progress(
                           DFU_WATCHDOG_PROGRESS_IMAGE_VALIDATED,
                           g_dfu_para.image_total_length);
    }

    if (check_result)
    {
        s_dfu_validated_image.valid = true;
        s_dfu_validated_image.image_id = g_dfu_para.ctrl_header.image_id;
        s_dfu_validated_image.image_total_length = g_dfu_para.image_total_length;
        notif_data[2] = DFU_ARV_SUCCESS;
    }
    else
    {
        notif_data[2] = DFU_ARV_FAIL_CRC_ERROR;
    }
#if ZY100_BUILD_PRODUCTION
    s_dfu_transfer_stage = previous_stage;
    if (check_result && previous_stage == DFU_TRANSFER_WAIT_VALIDATE)
    {
        dfu_transfer_wait(DFU_TRANSFER_WAIT_ACTIVATE);
    }
    /* A failed or repeated validation retains its original deadline. */
#endif
    notif_data[0] = DFU_OPCODE_NOTIFICATION;
    notif_data[1] = DFU_OPCODE_VALID_FW;
    dfu_service_notify(conn_id, notif_data, DFU_NOTIFY_LENGTH_VALID_FW);
}

bool dfu_service_valid_fw_success(void)
{
    return s_dfu_validated_image.valid;
}

bool dfu_service_commit_validated_fw(void)
{
    uint16_t image_id;
    bool committed;
#if ZY100_BUILD_PRODUCTION
    if (dfu_transfer_expired() || s_dfu_transfer_stage != DFU_TRANSFER_WAIT_ACTIVATE)
    {
        return false;
    }
#endif

    if (!s_dfu_validated_image.valid ||
        (s_dfu_validated_image.image_id != g_dfu_para.ctrl_header.image_id) ||
        (s_dfu_validated_image.image_total_length != g_dfu_para.image_total_length) ||
        (g_dfu_para.cur_offset != g_dfu_para.image_total_length))
    {
        dfu_service_invalidate_validated_image();
        return false;
    }

    image_id = s_dfu_validated_image.image_id;
    dfu_service_invalidate_validated_image();
#if ZY100_BUILD_PRODUCTION
    /* All identity/completeness gates passed; ready commit may now begin. */
    s_dfu_transfer_stage = DFU_TRANSFER_COMMITTING;
#endif
    committed = dfu_commit_validated_image(image_id);
    if (!committed)
    {
        return false;
    }
    return dfu_watchdog_note_progress(DFU_WATCHDOG_PROGRESS_READY_COMMITTED,
                                      g_dfu_para.image_total_length);
}

/**
 * @brief dfu_service_handle_control_point_req
 *
 * @param length     control point cmd length.
 * @param p_value    control point cmd address..
 * @return None
*/
void dfu_service_handle_control_point_req(uint8_t conn_id, uint16_t length,
                                          uint8_t *p_value)
{
    T_APP_RESULT w_cause = APP_RESULT_SUCCESS;
    T_DFU_CTRL_POINT dfu_control_point;
    uint8_t *p = p_value + 1;
    uint8_t notif_data[DFU_NOTIFY_LENGTH_MAX] = {0};

    T_DFU_CALLBACK_DATA callback_data;
    callback_data.conn_id = conn_id;
    callback_data.msg_type = SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE;
    callback_data.msg_data.write.write_attrib_index = INDEX_DFU_CONTROL_POINT_CHAR_VALUE;

    dfu_control_point.opcode = *p_value;
    DFU_PRINT_TRACE2("==>dfu_service_handle_control_point_req: opcode=0x%x, length=%d",
                     dfu_control_point.opcode, length);

    if (dfu_control_point.opcode >= DFU_OPCODE_MAX || dfu_control_point.opcode <= DFU_OPCODE_MIN)
    {
        notif_data[0] = DFU_OPCODE_NOTIFICATION;
        notif_data[1] = dfu_control_point.opcode;
        notif_data[2] = 0xff;
        dfu_service_notify(conn_id, notif_data, DFU_NOTIFY_LENGTH_START_DFU);
        return;
    }

    switch (dfu_control_point.opcode)
    {
    case DFU_OPCODE_START_DFU: //0x01
        dfu_service_invalidate_validated_image();
        if (length == DFU_LENGTH_START_DFU)/*4 bytes is padding for encrypt*/
        {
            DFU_PRINT_INFO1("DFU_OPCODE_START_DFU: OTP->ota_with_encryption_data=%d",
                            OTP->ota_with_encryption_data);
            if (OTP->ota_with_encryption_data)
            {
                DFU_PRINT_INFO1("ctrl header before decryped=%b", TRACE_BINARY(16, p));
                dfu_hw_aes_decrypt_image(p, p, 16);
                DFU_PRINT_INFO1("ctrl header after decryped=%b", TRACE_BINARY(16, p));
            }

            dfu_control_point.start_dfu.ic_type = (*p);
            p += 1;
            dfu_control_point.start_dfu.secure_version = (*p);
            p += 1;
            LE_ARRAY_TO_UINT16(dfu_control_point.start_dfu.ctrl_flag.value, p);
            p += 2;
            LE_ARRAY_TO_UINT16(dfu_control_point.start_dfu.image_id, p);
            p += 2;
            LE_ARRAY_TO_UINT16(dfu_control_point.start_dfu.crc16, p);
            p += 2;
            LE_ARRAY_TO_UINT32(dfu_control_point.start_dfu.payload_len, p);

            DFU_PRINT_INFO6("DFU_OPCODE_START_DFU: ic_type=0x%x, secure_version=0x%x, ctrl_flag.value=0x%x, image_id=0x%x,crc16=0x%x, payload_len=%d",
                            dfu_control_point.start_dfu.ic_type,
                            dfu_control_point.start_dfu.secure_version,
                            dfu_control_point.start_dfu.ctrl_flag.value,
                            dfu_control_point.start_dfu.image_id,
                            dfu_control_point.start_dfu.crc16,
                            dfu_control_point.start_dfu.payload_len
                           );
            g_dfu_para.ctrl_header.ic_type = dfu_control_point.start_dfu.ic_type;
            g_dfu_para.ctrl_header.ctrl_flag.value = dfu_control_point.start_dfu.ctrl_flag.value;
            g_dfu_para.ctrl_header.image_id = dfu_control_point.start_dfu.image_id;
            g_dfu_para.ctrl_header.crc16 = dfu_control_point.start_dfu.crc16;
            g_dfu_para.ctrl_header.payload_len = dfu_control_point.start_dfu.payload_len;
            g_dfu_para.image_total_length = g_dfu_para.ctrl_header.payload_len + IMG_HEADER_SIZE;

            /*check if start dfu fileds are vaild*/
            if (g_dfu_para.ctrl_header.ic_type == DEFINED_IC_TYPE)
            {
                if (((g_dfu_para.ctrl_header.image_id >= OTA) && (g_dfu_para.ctrl_header.image_id < IMAGE_MAX))
                    || (g_dfu_para.ctrl_header.image_id == IMAGE_USER_DATA))
                {
                    callback_data.msg_data.write.opcode = DFU_WRITE_START;
                    callback_data.msg_data.write.length = DFU_HEADER_SIZE;
                    callback_data.msg_data.write.p_value = (uint8_t *)&g_dfu_para.ctrl_header;
                    if (pfn_dfu_service_cb)
                    {
                        w_cause = pfn_dfu_service_cb(dfu_service_id, (void *)&callback_data);
                        if (w_cause != APP_RESULT_SUCCESS)
                        {
                            return;
                        }
                    }

                    uint32_t result = dfu_update(g_dfu_para.ctrl_header.image_id, 0, DFU_HEADER_SIZE,
                                                 (uint32_t *)&g_dfu_para.ctrl_header);
                    if (result)
                    {
                        DBG_DIRECT("DFU_OPCODE_START_DFU: dfu_update fail=%d", result);
                        T_DFU_FAIL_REASON dfu_fail_reason = DFU_FAIL_UPDATE_FLASH;
                        callback_data.msg_data.write.opcode = DFU_WRITE_FAIL;
                        callback_data.msg_data.write.length = sizeof(T_DFU_FAIL_REASON);
                        callback_data.msg_data.write.p_value = (uint8_t *)&dfu_fail_reason;
                        if (pfn_dfu_service_cb)
                        {
                            w_cause = pfn_dfu_service_cb(dfu_service_id, (void *)&callback_data);
                            if (w_cause != APP_RESULT_SUCCESS)
                            {
                                return;
                            }
                        }

                        dfu_fw_reboot(false);
                    }
                    else
                    {
                        g_dfu_para.cur_offset += DFU_HEADER_SIZE;
                        if (!dfu_watchdog_note_progress(
                                DFU_WATCHDOG_PROGRESS_BUFFER_COMMITTED,
                                g_dfu_para.cur_offset))
                        {
                            DBG_DIRECT("[DFU_WDG][FAULT] header progress rejected offset=%lu",
                                       (unsigned long)g_dfu_para.cur_offset);
                            return;
                        }
#if ZY100_BUILD_PRODUCTION
                        dfu_transfer_wait(DFU_TRANSFER_RECEIVING);
#endif
                        DFU_PRINT_INFO0("DFU_OPCODE_START_DFU: start success!");

                        uint32_t updated_success_len = DFU_HEADER_SIZE;
                        callback_data.msg_data.write.opcode = DFU_WRITE_DOING;
                        callback_data.msg_data.write.length = 4;
                        callback_data.msg_data.write.p_value = (uint8_t *)&updated_success_len;
                        if (pfn_dfu_service_cb)
                        {
                            w_cause = pfn_dfu_service_cb(dfu_service_id, (void *)&callback_data);
                            if (w_cause != APP_RESULT_SUCCESS)
                            {
                                return;
                            }
                        }

                        notif_data[0] = DFU_OPCODE_NOTIFICATION;
                        notif_data[1] = DFU_OPCODE_START_DFU;
                        notif_data[2] = DFU_ARV_SUCCESS;
                        dfu_service_notify(conn_id, notif_data, DFU_NOTIFY_LENGTH_START_DFU);
                    }


                }
                else
                {
                    DFU_PRINT_ERROR1("DFU_OPCODE_START_DFU: image id=0x%x Error!", g_dfu_para.ctrl_header.image_id);
                    notif_data[0] = DFU_OPCODE_NOTIFICATION;
                    notif_data[1] = DFU_OPCODE_START_DFU;
                    notif_data[2] = DFU_ARV_FAIL_INVALID_PARAMETER;
                    dfu_service_notify(conn_id, notif_data, DFU_NOTIFY_LENGTH_START_DFU);
                    return;
                }
            }
            else
            {
                DFU_PRINT_ERROR1("DFU_OPCODE_START_DFU: ic type=0x%x Error!", g_dfu_para.ctrl_header.ic_type);
                notif_data[0] = DFU_OPCODE_NOTIFICATION;
                notif_data[1] = DFU_OPCODE_START_DFU;
                notif_data[2] = DFU_ARV_FAIL_INVALID_PARAMETER;
                dfu_service_notify(conn_id, notif_data, DFU_NOTIFY_LENGTH_START_DFU);
                return;
            }
        }
        break;

    case DFU_OPCODE_RECEIVE_FW_IMAGE_INFO://0x02
        dfu_service_invalidate_validated_image();
        if (length == DFU_LENGTH_RECEIVE_FW_IMAGE_INFO)
        {
            LE_ARRAY_TO_UINT16(g_dfu_para.ctrl_header.image_id, p);
            p += 2;
            LE_ARRAY_TO_UINT32(g_dfu_para.cur_offset, p);
            if ((g_dfu_para.cur_offset == 0) || (g_dfu_para.cur_offset == DFU_HEADER_SIZE))
            {
                g_ota_tmp_buf_used_size = 0;
                dfu_resend_offset = 0;
            }
            DFU_PRINT_INFO3("DFU_OPCODE_RECEIVE_FW_IMAGE_INFO: image_id=0x%x, cur_offset=%d, g_ota_tmp_buf_used_size=%d",
                            g_dfu_para.ctrl_header.image_id, g_dfu_para.cur_offset, g_ota_tmp_buf_used_size);
        }
        else
        {
            DFU_PRINT_ERROR1("DFU_OPCODE_RECEIVE_FW_IMAGE_INFO: length=%d Error!", length);
        }
        break;

    case DFU_OPCODE_VALID_FW://0x03

        if (length == DFU_LENGTH_VALID_FW)
        {
            dfu_service_invalidate_validated_image();
            LE_ARRAY_TO_UINT16(g_dfu_para.ctrl_header.image_id, p);
            DFU_PRINT_TRACE1("DFU_OPCODE_VALID_FW: image_id=0x%x", g_dfu_para.ctrl_header.image_id);

            /*It is not recommended to do things that take a long time in upperstack task cb.
                So dfu service cb need send msg to app task to handle checksum image if support silent ota.
                While app task must handle IO_MSG_TYPE_DFU_VALID_FW by calling dfu_service_handle_valid_fw.
            */
        }
        else
        {
            dfu_service_invalidate_validated_image();
            DFU_PRINT_ERROR1("DFU_OPCODE_VALID_FW: length=%d Error!", length);
        }
        break;

    case DFU_OPCODE_ACTIVE_IMAGE_RESET://0x04
        {
            if (length != DFU_LENGTH_ACTIVE_IMAGE_RESET)
            {
                dfu_service_invalidate_validated_image();
                DFU_PRINT_ERROR1("DFU_OPCODE_ACTIVE_IMAGE_RESET: length=%d Error!", length);
                break;
            }

            /*notify bootloader to reset and use new image*/
            DFU_PRINT_INFO0("DFU_OPCODE_ACTIVE_IMAGE_RESET");
            callback_data.msg_data.write.opcode = DFU_WRITE_END;
            callback_data.msg_data.write.length = DFU_HEADER_SIZE;
            callback_data.msg_data.write.p_value = (uint8_t *)&g_dfu_para.ctrl_header;
            if (pfn_dfu_service_cb)
            {
                w_cause = pfn_dfu_service_cb(dfu_service_id, (void *)&callback_data);
                if (w_cause != APP_RESULT_SUCCESS)
                {
                    return;
                }
            }

            if (!is_ota_support_bank_switch())
            {
                /*note: must unlock flash bp for ota copy before reset when not support bank switch*/
                unlock_flash_bp_all();
            }
        }
        break;

    case DFU_OPCODE_SYSTEM_RESET://0x05
        {
            /*whatever cause ota fail, clinet will send this cmd to reset device*/
            DBG_DIRECT("DFU_OPCODE_SYSTEM_RESET");

            /*if select not active image by Phone even if image transport successful. Not for single bank user data*/
            if (g_dfu_para.ctrl_header.image_id >= OTA && g_dfu_para.ctrl_header.image_id < IMAGE_MAX)
            {
#ifdef SUPPORT_ALONE_UPPERSTACK_IMG
                uint32_t temp_addr = get_temp_ota_bank_addr_by_img_id_app((T_IMG_ID)
                                                                          g_dfu_para.ctrl_header.image_id);
#else
                uint32_t temp_addr = get_temp_ota_bank_addr_by_img_id((T_IMG_ID)g_dfu_para.ctrl_header.image_id);
#endif
                T_IMG_CTRL_HEADER_FORMAT *p_temp_header = (T_IMG_CTRL_HEADER_FORMAT *)temp_addr;
                if (p_temp_header && !p_temp_header->ctrl_flag.flag_value.not_ready)
                {
                    flash_erase_locked(FLASH_ERASE_SECTOR, temp_addr);
                }
            }

            T_DFU_FAIL_REASON dfu_fail_reason = DFU_FAIL_SYSTEM_RESET_CMD;
            callback_data.msg_data.write.length = sizeof(T_DFU_FAIL_REASON);
            callback_data.msg_data.write.p_value = (uint8_t *)&dfu_fail_reason;
            callback_data.msg_data.write.opcode = DFU_WRITE_FAIL;
            if (pfn_dfu_service_cb)
            {
                w_cause = pfn_dfu_service_cb(dfu_service_id, (void *)&callback_data);
                if (w_cause != APP_RESULT_SUCCESS)
                {
                    return;
                }
            }

            dfu_fw_reboot(false);
        }
        break;

    case DFU_OPCODE_REPORT_TARGET_INFO://0x06
        if (length == DFU_LENGTH_REPORT_TARGET_INFO)
        {
            LE_ARRAY_TO_UINT16(g_dfu_para.ctrl_header.image_id, p);
            dfu_report_target_fw_info(g_dfu_para.ctrl_header.image_id, &g_dfu_para.origin_image_version,
                                      (uint32_t *)&g_dfu_para.cur_offset);
            g_dfu_para.cur_offset = 0;
            DFU_PRINT_INFO3("DFU_OPCODE_REPORT_TARGET_INFO: image_id=0x%x,origin_image_ver=0x%x, cur_offset=%d",
                            g_dfu_para.ctrl_header.image_id, g_dfu_para.origin_image_version, g_dfu_para.cur_offset);

            notif_data[0] = DFU_OPCODE_NOTIFICATION;
            notif_data[1] = DFU_OPCODE_REPORT_TARGET_INFO;
            notif_data[2] = DFU_ARV_SUCCESS;

            LE_UINT32_TO_ARRAY(&notif_data[3], g_dfu_para.origin_image_version);
            LE_UINT32_TO_ARRAY(&notif_data[7], g_dfu_para.cur_offset);
            server_send_data(conn_id, dfu_service_id, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                             notif_data, DFU_NOTIFY_LENGTH_REPORT_TARGET_INFO, GATT_PDU_TYPE_NOTIFICATION);
        }
        else
        {
            DFU_PRINT_ERROR1("DFU_OPCODE_REPORT_TARGET_INFO: length=%d Error!", length);
        }
        break;

    case DFU_OPCODE_CONN_PARA_TO_UPDATE_REQ://0x07
        {
            notif_data[0] = DFU_OPCODE_NOTIFICATION;
            notif_data[1] = DFU_OPCODE_CONN_PARA_TO_UPDATE_REQ;

            if (length  == DFU_LENGTH_CONN_PARA_TO_UPDATE_REQ)
            {
                if (g_dfu_para.dfu_conn_para_update_in_progress)
                {
                    DFU_PRINT_ERROR0("DFU_OPCODE_CONN_PARA_TO_UPDATE_REQ: OTA ConnParaUpdInProgress!");
                    notif_data[2] = DFU_ARV_FAIL_OPERATION;
                    server_send_data(conn_id, dfu_service_id, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                                     notif_data, DFU_NOTIFY_LENGTH_ARV, GATT_PDU_TYPE_NOTIFICATION);
                }
                else
                {
                    uint16_t conn_interval_min;
                    uint16_t conn_interval_max;
                    uint16_t conn_latency;
                    uint16_t superv_tout;

                    LE_ARRAY_TO_UINT16(conn_interval_min, p_value + 1);
                    LE_ARRAY_TO_UINT16(conn_interval_max, p_value + 3);
                    LE_ARRAY_TO_UINT16(conn_latency, p_value + 5);
                    LE_ARRAY_TO_UINT16(superv_tout, p_value + 7);

                    if (le_update_conn_param(conn_id, conn_interval_min, conn_interval_max, conn_latency,
                                             superv_tout, conn_interval_min * 2 - 2, conn_interval_max * 2 - 2) == GAP_CAUSE_SUCCESS)
                    {
                        /* Connection Parameter Update Request sent successfully, means this procedure is in progress. */
                        g_dfu_para.dfu_conn_para_update_in_progress = true;
                        g_dfu_para.dfu_conn_para_update_notify_remote = true;
                        DFU_PRINT_INFO4("DFU_OPCODE_CONN_PARA_TO_UPDATE_REQ: conn_min=0x%x, conn_max=0x%x, latcy=0x%x, timeout=0x%x",
                                        conn_interval_min, conn_interval_max, conn_latency, superv_tout);
                    }
                    else
                    {
                        g_dfu_para.dfu_conn_para_update_in_progress = false;
                        g_dfu_para.dfu_conn_para_update_notify_remote = false;
                        notif_data[2] = DFU_ARV_FAIL_OPERATION;
                        server_send_data(conn_id, dfu_service_id, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                                         notif_data, DFU_NOTIFY_LENGTH_ARV, GATT_PDU_TYPE_NOTIFICATION);
                    }
                }
            }
            else
            {
                /*TODO: to be masked.*/
                DFU_PRINT_ERROR1("DFU_OPCODE_CONN_PARA_TO_UPDATE_REQ: length=%d Error!", length);
                notif_data[2] = DFU_ARV_FAIL_INVALID_PARAMETER;
                server_send_data(conn_id, dfu_service_id, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                                 notif_data, DFU_NOTIFY_LENGTH_ARV, GATT_PDU_TYPE_NOTIFICATION);
            }
        }
        break;

    case DFU_OPCODE_BUFFER_CHECK_EN: //0x09
        {
            le_get_conn_param(GAP_PARAM_CONN_MTU_SIZE, &g_dfu_para.mtu_size, conn_id);
            DFU_PRINT_TRACE2("DFU_OPCODE_BUFFER_CHECK_EN: mtu_size=%d, max_bufffer_size=%d",
                             g_dfu_para.mtu_size,
                             DFU_TEMP_BUFFER_SIZE);

            notif_data[0] = DFU_OPCODE_NOTIFICATION;
            notif_data[1] = DFU_OPCODE_BUFFER_CHECK_EN;
#if (DFU_BUFFER_CHECK_ENABLE == 1)
            buffer_check_en = true;
            notif_data[2] = DFU_ARV_SUCCESS;
#else
            buffer_check_en = false;
            notif_data[2] = DFU_ARV_FAIL_OPERATION;
#endif
            LE_UINT16_TO_ARRAY(&notif_data[3], DFU_TEMP_BUFFER_SIZE);
            LE_UINT16_TO_ARRAY(&notif_data[5], g_dfu_para.mtu_size);
            server_send_data(conn_id, dfu_service_id, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                             notif_data, DFU_NOTIFY_LENGTH_BUFFER_CHECK_EN, GATT_PDU_TYPE_NOTIFICATION);
        }
        break;

    case DFU_OPCODE_REPORT_BUFFER_CRC:        //0x0a
        {
            uint16_t mBufSize;
            uint16_t mCrcVal;
            LE_ARRAY_TO_UINT16(mBufSize, p);
            p += 2;
            LE_ARRAY_TO_UINT16(mCrcVal, p);
            DFU_PRINT_INFO2("DFU_OPCODE_REPORT_BUFFER_CRC: mBufSize=0x%x, mCrcVal=0x%x", mBufSize, mCrcVal);
            dfu_buffer_check_process(conn_id, mBufSize, mCrcVal);
        }
        break;

    case DFU_OPCODE_RECEIVE_IC_TYPE://0x0b
        {
            uint8_t ic_type = 0; //0 means invalid ic type
            notif_data[0] = DFU_OPCODE_NOTIFICATION;
            notif_data[1] = DFU_OPCODE_RECEIVE_IC_TYPE;
            if (dfu_report_target_ic_type(OTA, &ic_type))
            {
                notif_data[2] = DFU_ARV_FAIL_INVALID_PARAMETER;
                notif_data[3] =  ic_type;
            }
            else
            {
                notif_data[2] = DFU_ARV_SUCCESS;
                notif_data[3] =  ic_type;
            }
            DFU_PRINT_INFO1("DFU_OPCODE_RECEIVE_IC_TYPE: ic_type=0x%x", ic_type);
            server_send_data(conn_id, dfu_service_id, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                             notif_data, DFU_NOTIFY_LENGTH_RECEIVE_IC_TYPE, GATT_PDU_TYPE_NOTIFICATION);
        }
        break;

#if (ENABLE_BANK_SWITCH_COPY_APP_DATA == 1)
    case DFU_OPCODE_COPY_IMG://0x0c
        {
            uint32_t dlAddress, dlSize;
            LE_ARRAY_TO_UINT16(g_dfu_para.ctrl_header.image_id, p);
            p += 2;
            LE_ARRAY_TO_UINT32(dlAddress, p);
            p += 4;
            LE_ARRAY_TO_UINT32(dlSize, p);
            DFU_PRINT_TRACE2("DFU_OPCODE_COPY_IMG: dlAddress=0x%x,dlSize=0x%x", dlAddress,
                             dlSize);

            notif_data[0] = DFU_OPCODE_NOTIFICATION;
            notif_data[1] = DFU_OPCODE_COPY_IMG;

            if (dfu_copy_img(g_dfu_para.ctrl_header.image_id, dlAddress, dlSize))
            {
                notif_data[2] = DFU_ARV_SUCCESS;
            }
            else
            {
                notif_data[2] = DFU_ARV_FAIL_INVALID_PARAMETER;
            }
            server_send_data(conn_id, dfu_service_id, INDEX_DFU_CONTROL_POINT_CHAR_VALUE, \
                             notif_data, DFU_NOTIFY_LENGTH_ARV, GATT_PDU_TYPE_NOTIFICATION);
        }
        break;
#endif  //end ENABLE_BANK_SWITCH_COPY_APP_DATA

    default:
        {
            DFU_PRINT_TRACE1("dfu_service_handle_control_point_req: Unknown Opcode=0x%x",
                             dfu_control_point.opcode);
        }
        break;
    }
}


/**
 * @brief dfu_service_handle_packet_req
 *
 * @param length     data reviewed length.
 * @param p_value    data receive point address.
 * @return None
*/
void dfu_service_handle_packet_req(uint8_t conn_id, uint16_t length, uint8_t *p_value)
{
    T_APP_RESULT w_cause = APP_RESULT_SUCCESS;
    T_DFU_CALLBACK_DATA callback_data;
    callback_data.conn_id = conn_id;
    callback_data.msg_type = SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE;
    callback_data.msg_data.write.write_attrib_index = INDEX_DFU_PACKET_VALUE;
    callback_data.msg_data.write.length = length;
    callback_data.msg_data.write.p_value = p_value;

#if ZY100_BUILD_PRODUCTION
    if (dfu_transfer_expired() || s_dfu_transfer_stage != DFU_TRANSFER_RECEIVING ||
        length == 0U || p_value == NULL ||
        g_dfu_para.cur_offset > g_dfu_para.image_total_length ||
        g_ota_tmp_buf_used_size > g_dfu_para.image_total_length - g_dfu_para.cur_offset ||
        length > g_dfu_para.image_total_length - g_dfu_para.cur_offset - g_ota_tmp_buf_used_size)
    {
        return;
    }
#endif
    if (length > 0U)
    {
        dfu_service_invalidate_validated_image();
    }

    DFU_PRINT_INFO4("dfu_service_handle_packet_req: length=%d, cur_offset=%d, g_ota_tmp_buf_used_size=%d, image_total_length=%d",
                    length, g_dfu_para.cur_offset,
                    g_ota_tmp_buf_used_size, g_dfu_para.image_total_length);

    if (buffer_check_en == true)
    {
        /*to avoid memory overflow*/
        if (g_ota_tmp_buf_used_size + length > DFU_TEMP_BUFFER_SIZE)
        {
            DFU_PRINT_ERROR3("<==dfu_service_handle_packet_req: Buf overflow! ota_tmp_buf_used_size=%d,length=%d, max_buffer_size=%d",
                             g_ota_tmp_buf_used_size, length, DFU_TEMP_BUFFER_SIZE);

            T_DFU_FAIL_REASON dfu_fail_reason = DFU_FAIL_EXCEED_MAX_BUFFER_SIZE;
            callback_data.msg_data.write.length = sizeof(T_DFU_FAIL_REASON);
            callback_data.msg_data.write.p_value = (uint8_t *)&dfu_fail_reason;
            callback_data.msg_data.write.opcode = DFU_WRITE_FAIL;
            if (pfn_dfu_service_cb)
            {
                w_cause = pfn_dfu_service_cb(dfu_service_id, (void *)&callback_data);
                if (w_cause != APP_RESULT_SUCCESS)
                {
                    return;
                }
            }

            dfu_fw_reboot(false);
        }
        else
        {
            memcpy(p_ota_temp_buffer_head + g_ota_tmp_buf_used_size, p_value, length);
            g_ota_tmp_buf_used_size += length;
#if ZY100_BUILD_PRODUCTION
            s_dfu_transfer_last_ms = (uint32_t)os_sys_time_get();
#endif
        }
    }
    else
    {
        /*when disable buffer check, Default client send 20bytes per packet*/
        uint32_t max_buffer_size = DFU_TEMP_BUFFER_SIZE - (DFU_TEMP_BUFFER_SIZE % 20);
        /*0.check memcpy buffer boundary*/
        if (g_ota_tmp_buf_used_size + length > max_buffer_size)
        {
            DBG_DIRECT("<==dfu_service_handle_packet_req: Buf overflow! ota_tmp_buf_used_size=%d,length=%d, max_buffer_size=%d",
                       g_ota_tmp_buf_used_size, length, max_buffer_size);

            T_DFU_FAIL_REASON dfu_fail_reason = DFU_FAIL_EXCEED_MAX_BUFFER_SIZE;
            callback_data.msg_data.write.opcode = DFU_WRITE_FAIL;
            callback_data.msg_data.write.length = sizeof(T_DFU_FAIL_REASON);
            callback_data.msg_data.write.p_value = (uint8_t *)&dfu_fail_reason;
            if (pfn_dfu_service_cb)
            {
                w_cause = pfn_dfu_service_cb(dfu_service_id, (void *)&callback_data);
                if (w_cause != APP_RESULT_SUCCESS)
                {
                    return;
                }
            }

            dfu_fw_reboot(false);
        }

        /*0.check total length*/
        if (g_dfu_para.cur_offset + g_ota_tmp_buf_used_size + length > g_dfu_para.image_total_length)
        {
            DFU_PRINT_ERROR1("<==dfu_service_handle_packet_req: received data total length beyond image_total_length(%d bytes)",
                             g_dfu_para.image_total_length);

            T_DFU_FAIL_REASON dfu_fail_reason = DFU_FAIL_EXCEED_IMG_TOTAL_LEN;
            callback_data.msg_data.write.opcode = DFU_WRITE_FAIL;
            callback_data.msg_data.write.length = sizeof(T_DFU_FAIL_REASON);
            callback_data.msg_data.write.p_value = (uint8_t *)&dfu_fail_reason;
            if (pfn_dfu_service_cb)
            {
                w_cause = pfn_dfu_service_cb(dfu_service_id, (void *)&callback_data);
                if (w_cause != APP_RESULT_SUCCESS)
                {
                    return;
                }
            }

            dfu_fw_reboot(false);
        }

        /*start handle received data*/
        //1. deceypt data
        if (OTP->ota_with_encryption_data)
        {
            dfu_hw_aes_decrypt_image(p_value, p_value, length);
        }

        //2. copy received data to buffer
        memcpy(p_ota_temp_buffer_head + g_ota_tmp_buf_used_size, p_value, length);
        g_ota_tmp_buf_used_size += length;
#if ZY100_BUILD_PRODUCTION
        s_dfu_transfer_last_ms = (uint32_t)os_sys_time_get();
#endif

        //3. write to flash
        if (g_ota_tmp_buf_used_size == max_buffer_size ||
            g_dfu_para.cur_offset + g_ota_tmp_buf_used_size == g_dfu_para.image_total_length)
        {
            uint32_t result = dfu_update(g_dfu_para.ctrl_header.image_id, g_dfu_para.cur_offset,
                                         g_ota_tmp_buf_used_size,
                                         (uint32_t *)p_ota_temp_buffer_head);
            if (result)
            {
                DBG_DIRECT("Buffer check disable: dfu_update fail=%d", result);

                /*eflash write fail, we should restart ota procedure.*/
                T_DFU_FAIL_REASON dfu_fail_reason = DFU_FAIL_UPDATE_FLASH;
                callback_data.msg_data.write.opcode = DFU_WRITE_FAIL;
                callback_data.msg_data.write.length = sizeof(T_DFU_FAIL_REASON);
                callback_data.msg_data.write.p_value = (uint8_t *)&dfu_fail_reason;
                if (pfn_dfu_service_cb)
                {
                    w_cause = pfn_dfu_service_cb(dfu_service_id, (void *)&callback_data);
                    if (w_cause != APP_RESULT_SUCCESS)
                    {
                        return;
                    }
                }

                dfu_fw_reboot(false);
            }
            else
            {
                uint32_t updated_success_len = g_ota_tmp_buf_used_size;
                callback_data.msg_data.write.opcode = DFU_WRITE_DOING;
                callback_data.msg_data.write.length = 4;
                callback_data.msg_data.write.p_value = (uint8_t *)&updated_success_len;
                if (pfn_dfu_service_cb)
                {
                    w_cause = pfn_dfu_service_cb(dfu_service_id, (void *)&callback_data);
                    if (w_cause != APP_RESULT_SUCCESS)
                    {
                        return;
                    }
                }

                /*update varibals value*/
                g_dfu_para.cur_offset += g_ota_tmp_buf_used_size;
                g_ota_tmp_buf_used_size = 0;
                if (!dfu_watchdog_note_progress(
                        DFU_WATCHDOG_PROGRESS_BUFFER_COMMITTED,
                        g_dfu_para.cur_offset))
                {
                    DBG_DIRECT("[DFU_WDG][FAULT] streaming progress rejected offset=%lu",
                               (unsigned long)g_dfu_para.cur_offset);
                    return;
                }
#if ZY100_BUILD_PRODUCTION
                dfu_transfer_committed();
#endif
            }
        }
    }

}

/**
 * @brief write characteristic data from service.
 *
 * @param ServiceID          ServiceID to be written.
 * @param iAttribIndex       Attribute index of characteristic.
 * @param wLength            length of value to be written.
 * @param pValue             value to be written.
 * @return Profile procedure result
*/
T_APP_RESULT dfu_attr_write_cb(uint8_t conn_id, uint8_t service_id, uint16_t attrib_index,
                               T_WRITE_TYPE write_type,
                               uint16_t length, uint8_t *p_value, P_FUN_WRITE_IND_POST_PROC *p_write_ind_post_proc)
{
    T_APP_RESULT w_cause = APP_RESULT_SUCCESS;
    T_DFU_CALLBACK_DATA callback_data;
#if ZY100_BUILD_PRODUCTION
    if (dfu_transfer_expired())
    {
        return APP_RESULT_APP_ERR;
    }
    if (attrib_index == INDEX_DFU_CONTROL_POINT_CHAR_VALUE)
    {
        if (length == 0U || p_value == NULL)
        {
            return APP_RESULT_INVALID_VALUE_SIZE;
        }
        switch (p_value[0])
        {
        case DFU_OPCODE_START_DFU:
            if (s_dfu_transfer_stage != DFU_TRANSFER_IDLE)
                return APP_RESULT_APP_ERR;
            break;
        case DFU_OPCODE_RECEIVE_FW_IMAGE_INFO:
        {
            uint16_t image_id;
            uint32_t offset;
            if (length != DFU_LENGTH_RECEIVE_FW_IMAGE_INFO)
                return APP_RESULT_INVALID_VALUE_SIZE;
            LE_ARRAY_TO_UINT16(image_id, &p_value[1]);
            LE_ARRAY_TO_UINT32(offset, &p_value[3]);
            if (s_dfu_transfer_stage != DFU_TRANSFER_RECEIVING ||
                image_id != g_dfu_para.ctrl_header.image_id ||
                offset < DFU_HEADER_SIZE || offset > g_dfu_para.cur_offset)
                return APP_RESULT_APP_ERR;
            break;
        }
        case DFU_OPCODE_REPORT_BUFFER_CRC:
            if (length != DFU_LENGTH_REPORT_BUFFER_CRC)
                return APP_RESULT_INVALID_VALUE_SIZE;
            if (s_dfu_transfer_stage != DFU_TRANSFER_RECEIVING)
                return APP_RESULT_APP_ERR;
            break;
        case DFU_OPCODE_REPORT_TARGET_INFO:
            /* SDK query resets cur_offset/image_id; never let it rewrite
             * the active image while the inactivity guard owns transfer. */
            if (s_dfu_transfer_stage != DFU_TRANSFER_IDLE)
                return APP_RESULT_APP_ERR;
            break;
        case DFU_OPCODE_VALID_FW:
            if (length != DFU_LENGTH_VALID_FW)
                return APP_RESULT_INVALID_VALUE_SIZE;
            if ((s_dfu_transfer_stage != DFU_TRANSFER_WAIT_VALIDATE &&
                 s_dfu_transfer_stage != DFU_TRANSFER_WAIT_ACTIVATE) ||
                p_value[1] != (uint8_t)g_dfu_para.ctrl_header.image_id ||
                p_value[2] != (uint8_t)(g_dfu_para.ctrl_header.image_id >> 8))
                return APP_RESULT_APP_ERR;
            break;
        case DFU_OPCODE_ACTIVE_IMAGE_RESET:
            if (length != DFU_LENGTH_ACTIVE_IMAGE_RESET)
                return APP_RESULT_INVALID_VALUE_SIZE;
            if (s_dfu_transfer_stage != DFU_TRANSFER_WAIT_ACTIVATE ||
                !s_dfu_validated_image.valid)
                return APP_RESULT_APP_ERR;
            break;
        default:
            break;
        }
    }
#endif
    callback_data.conn_id = conn_id;
    callback_data.msg_type = SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE;
    callback_data.msg_data.write.write_attrib_index = attrib_index;
    callback_data.msg_data.write.length = length;
    callback_data.msg_data.write.p_value = p_value;

    /* Notify Application. */
    callback_data.msg_data.write.opcode = DFU_WRITE_ATTR_ENTER;
    if (pfn_dfu_service_cb)
    {
        w_cause = pfn_dfu_service_cb(service_id, (void *)&callback_data);
        if (w_cause != APP_RESULT_SUCCESS)
        {
            return w_cause;
        }
    }

    if (attrib_index == INDEX_DFU_CONTROL_POINT_CHAR_VALUE)
    {
        dfu_service_handle_control_point_req(conn_id, length, p_value);
    }
    else if (attrib_index == INDEX_DFU_PACKET_VALUE)
    {
        dfu_service_handle_packet_req(conn_id, length, p_value);
    }
    else
    {
        DFU_PRINT_INFO1("dfu_attr_write_cb fail! attrib_index=%d", attrib_index);
        w_cause = APP_RESULT_ATTR_NOT_FOUND;
    }

    /* Notify Application. */
    callback_data.msg_data.write.opcode = DFU_WRITE_ATTR_EXIT;
    if (pfn_dfu_service_cb)
    {
        w_cause = pfn_dfu_service_cb(service_id, (void *)&callback_data);
        if (w_cause != APP_RESULT_SUCCESS)
        {
            return w_cause;
        }
    }

    return w_cause;
}

/**
 * @brief update CCCD bits from stack.
 *
 * @param ServiceId          Service ID.
 * @param Index          Attribute index of characteristic data.
 * @param wCCCBits         CCCD bits from stack.
 * @return None
*/
void dfu_cccd_update_cb(uint8_t conn_id, T_SERVER_ID service_id, uint16_t index, uint16_t ccc_bits)
{
    T_DFU_CALLBACK_DATA callback_data;
    callback_data.msg_type = SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION;
    callback_data.conn_id = conn_id;
    bool b_handle = true;
    DFU_PRINT_INFO2("dfu_cccd_update_cb: index=%d, ccc_bits=0x%x", index, ccc_bits);
    switch (index)
    {
    case INDEX_DFU_CHAR_CCCD_INDEX:
        {
            if (ccc_bits & GATT_CLIENT_CHAR_CONFIG_NOTIFY)
            {
                // Enable Notification
                callback_data.msg_type = SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION;
                callback_data.msg_data.notification_indification_index = DFU_NOTIFY_ENABLE;
            }
            else
            {
                callback_data.msg_type = SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION;
                callback_data.msg_data.notification_indification_index = DFU_NOTIFY_DISABLE;
            }
            break;
        }
    default:
        {
            b_handle = false;
            break;
        }

    }
    /* Notify Application. */
    if (pfn_dfu_service_cb && (b_handle == true))
    {
        T_APP_RESULT update_cause = pfn_dfu_service_cb(service_id, (void *)&callback_data);
        if (update_cause != APP_RESULT_SUCCESS)
        {
            return;
        }
    }

    return;
}

/**
 * @brief OTA ble Service Callbacks.
*/
const T_FUN_GATT_SERVICE_CBS DfuServiceCBs =
{
    NULL,                // Read callback function pointer
    dfu_attr_write_cb,   // Write callback function pointer
    dfu_cccd_update_cb   // CCCD update callback function pointer
};

/**
 * @brief  add OTA ble service to application.
 *
 * @param  pFunc          pointer of app callback function called by profile.
 * @return service ID auto generated by profile layer.
 * @retval ServiceId
*/
uint8_t dfu_add_service(void *pFunc)
{
    if (false == server_add_service(&dfu_service_id,
                                    (uint8_t *)gatt_dfu_service_table,
                                    sizeof(gatt_dfu_service_table),
                                    DfuServiceCBs))
    {
        DFU_PRINT_ERROR1("dfu_add_service: service_id=%d", dfu_service_id);
        dfu_service_id = 0xff;
        return dfu_service_id;
    }
    pfn_dfu_service_cb = (P_FUN_SERVER_GENERAL_CB)pFunc;
    p_ota_temp_buffer_head = ota_temp_buffer_head;

    return dfu_service_id;
}
