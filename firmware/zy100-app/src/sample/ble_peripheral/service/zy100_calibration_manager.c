#include "zy100_calibration_manager.h"

#include <string.h>

#include "os_sched.h"
#include "trace.h"
#include "mag_capture_service.h"
#include "zy100_calibration_ble_service.h"
#include "zy100_final_edge_raw_format.h"
#include "zy100_mag_calibrator.h"
#include "zy100_calibration_protocol.h"
#include "zy100_calibration_store.h"

#define ZY100_CAL_CONN_INVALID              0xFFU
#define ZY100_CAL_DEFAULT_ATT_MTU           23U
#define ZY100_CAL_MAX_NOTIFY_BYTES          244U
#define ZY100_MAG_CAL_TIMEOUT_MS             30000UL

typedef enum
{
    ZY100_CAL_TX_IDLE = 0U,
    ZY100_CAL_TX_START_PENDING,
    ZY100_CAL_TX_DATA_STREAM,
    ZY100_CAL_TX_END_PENDING,
    ZY100_CAL_TX_WAIT_ACK,
} zy100_cal_tx_state_t;

typedef enum
{
    ZY100_CAL_TX_SOURCE_NONE = 0U,
    ZY100_CAL_TX_SOURCE_RECORD,
    ZY100_CAL_TX_SOURCE_DIAGNOSTICS,
} zy100_cal_tx_source_t;

typedef struct
{
    uint8_t blob[ZY100_CAL_RECORD_MAX_BYTES];
    uint8_t mag_diag_blob[ZY100_CAL_MAG_DIAG_BYTES];
    union
    {
        struct
        {
            union
            {
                uint8_t staging[ZY100_CAL_RECORD_MAX_BYTES];
                uint8_t tx_frame[ZY100_CAL_MAX_NOTIFY_BYTES];
            } work;
            uint8_t received[ZY100_CAL_RECORD_MAX_BYTES / 8U];
        } transfer;
        zy100_mag_calibrator_t mag;
    } mode;
    zy100_cal_record_info_t info;
    zy100_cal_tx_state_t tx_state;
    zy100_cal_tx_source_t tx_source;
    uint16_t tx_offset;
    uint16_t tx_total;
    uint32_t tx_crc;
    uint16_t write_total;
    uint32_t write_crc;
    uint8_t transaction_id;
    uint8_t conn_id;
    uint8_t status;
    uint8_t status_transaction_id;
    uint16_t detail;
    bool valid;
    bool write_active;
    bool commit_pending;
    bool status_dirty;
    bool mag_start_pending;
    bool mag_prepared;
    bool mag_sensor_owned;
    bool mag_sampling_started;
    uint32_t mag_prepare_deadline_ms;
    bool mag_collecting;
    bool mag_fit_pending;
    bool mag_save_pending;
    bool mag_final_fit;
    uint64_t mag_started_ms;
    uint64_t mag_deadline_ms;
    uint32_t mag_read_errors;
    uint32_t mag_rejected_samples;
    uint8_t mag_fit_attempts;
    uint8_t mag_visual_state;
    uint8_t mag_result_model;
    uint16_t mag_result_quality;
    uint8_t mag_full_solve_status;
    uint8_t mag_axis_solve_status;
    uint8_t mag_hard_solve_status;
    uint32_t mag_result_rms_x1e6;
    bool mag_diag_valid;
    bool info_confirmed;
    bool record_confirmed;
} zy100_cal_manager_t;

static zy100_cal_manager_t s_cal;
static void zy100_cal_manager_mag_diag_reset(void);

bool zy100_cal_manager_mag_prepare_needed(void)
{
    return s_cal.mag_start_pending && !s_cal.mag_prepared &&
        ((int32_t)((uint32_t)os_sys_time_get() - s_cal.mag_prepare_deadline_ms) < 0);
}

void zy100_cal_manager_mag_prepare_complete(void)
{
    if (s_cal.mag_start_pending) s_cal.mag_prepared = true;
}

bool zy100_cal_manager_mag_active(void)
{
    return s_cal.mag_start_pending || s_cal.mag_collecting ||
           s_cal.mag_fit_pending || s_cal.mag_save_pending;
}

bool zy100_cal_manager_mag_flash_busy(void)
{
    return s_cal.mag_save_pending;
}

bool zy100_cal_manager_info_confirmed(uint8_t conn_id)
{
    return s_cal.info_confirmed &&
           (s_cal.conn_id != ZY100_CAL_CONN_INVALID) &&
           (s_cal.conn_id == conn_id);
}

bool zy100_cal_manager_business_ready(uint8_t conn_id)
{
    return zy100_cal_manager_info_confirmed(conn_id) &&
           (!s_cal.valid || s_cal.record_confirmed);
}

uint8_t zy100_cal_manager_mag_visual_state(void)
{
    return s_cal.mag_visual_state;
}

void zy100_cal_manager_mag_visual_result_consumed(void)
{
    if ((s_cal.mag_visual_state == ZY100_CAL_MAG_VISUAL_SUCCESS) ||
        (s_cal.mag_visual_state == ZY100_CAL_MAG_VISUAL_FAILURE) ||
        (s_cal.mag_visual_state == ZY100_CAL_MAG_VISUAL_CANCEL))
    {
        s_cal.mag_visual_state = ZY100_CAL_MAG_VISUAL_NONE;
    }
}

static void zy100_cal_manager_publish_info(void)
{
    uint8_t info[ZY100_CAL_INFO_BYTES];
    zy100_cal_build_info(info, &s_cal.info, s_cal.valid);
    zy100_cal_ble_service_set_info(info, sizeof(info));
}

static void zy100_cal_manager_set_status_for_transaction(
    uint8_t status, uint16_t detail, uint8_t transaction_id)
{
    uint8_t frame[ZY100_CAL_STATUS_BYTES];
    s_cal.status = status;
    s_cal.status_transaction_id = transaction_id;
    s_cal.detail = detail;
    s_cal.status_dirty = true;
    zy100_cal_build_status(frame, status, transaction_id,
                           detail, s_cal.valid ? &s_cal.info : NULL);
    zy100_cal_ble_service_set_status(frame, sizeof(frame));
}

static void zy100_cal_manager_set_status(uint8_t status, uint16_t detail)
{
    zy100_cal_manager_set_status_for_transaction(status, detail,
                                                  s_cal.transaction_id);
}

void zy100_cal_manager_init(void)
{
    zy100_cal_store_status_t status;
    memset(&s_cal, 0, sizeof(s_cal));
    s_cal.conn_id = ZY100_CAL_CONN_INVALID;
    status = zy100_cal_store_load(s_cal.blob, &s_cal.info);
    s_cal.valid = (status == ZY100_CAL_STORE_OK);
    zy100_cal_manager_publish_info();
    zy100_cal_manager_set_status(s_cal.valid ? ZY100_CAL_STATUS_READY :
                                ZY100_CAL_STATUS_NO_DATA, (uint16_t)status);
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[CAL] init valid=%u generation=%lu bytes=%u store=%s",
               s_cal.valid ? 1U : 0U,
               (unsigned long)s_cal.info.generation,
               s_cal.info.record_bytes,
               zy100_cal_store_status_name(status));
}

static void zy100_cal_manager_prepare_record_tx(uint8_t conn_id,
                                                bool increment_transaction,
                                                bool publish_sending_status)
{
    s_cal.conn_id = conn_id;
    if (increment_transaction)
    {
        s_cal.transaction_id++;
    }
    s_cal.tx_offset = 0U;
    s_cal.tx_source = s_cal.valid ? ZY100_CAL_TX_SOURCE_RECORD :
                                   ZY100_CAL_TX_SOURCE_NONE;
    s_cal.tx_total = s_cal.valid ? s_cal.info.record_bytes : 0U;
    s_cal.tx_crc = s_cal.valid ? s_cal.info.crc32 : 0UL;
    s_cal.tx_state = s_cal.valid ? ZY100_CAL_TX_START_PENDING : ZY100_CAL_TX_IDLE;
    if (publish_sending_status)
    {
        zy100_cal_manager_set_status(s_cal.valid ? ZY100_CAL_STATUS_SENDING :
                                    ZY100_CAL_STATUS_NO_DATA, 0U);
    }
}

static void zy100_cal_manager_prepare_diag_tx(uint8_t conn_id,
                                              uint8_t transaction_id)
{
    s_cal.conn_id = conn_id;
    s_cal.transaction_id = transaction_id;
    s_cal.tx_offset = 0U;
    s_cal.tx_source = ZY100_CAL_TX_SOURCE_DIAGNOSTICS;
    s_cal.tx_total = ZY100_CAL_MAG_DIAG_BYTES;
    s_cal.tx_crc = zy100_cal_get_u32_le(
        &s_cal.mag_diag_blob[ZY100_CAL_MAG_DIAG_CRC_OFFSET]);
    s_cal.tx_state = ZY100_CAL_TX_START_PENDING;
    zy100_cal_manager_set_status(ZY100_CAL_STATUS_SENDING, 0U);
}

void zy100_cal_manager_on_tx_cccd(uint8_t conn_id, bool enabled)
{
    if (enabled)
    {
        s_cal.conn_id = conn_id;
    }
    else if (s_cal.conn_id == conn_id)
    {
        s_cal.tx_state = ZY100_CAL_TX_IDLE;
        s_cal.tx_source = ZY100_CAL_TX_SOURCE_NONE;
        s_cal.conn_id = ZY100_CAL_CONN_INVALID;
    }
}

void zy100_cal_manager_on_status_cccd(uint8_t conn_id, bool enabled)
{
    if (enabled)
    {
        s_cal.conn_id = conn_id;
        s_cal.status_dirty = true;
    }
}

void zy100_cal_manager_on_disconnect(uint8_t conn_id)
{
    if (s_cal.mag_start_pending)
    {
        s_cal.mag_start_pending = false;
        s_cal.mag_visual_state = ZY100_CAL_MAG_VISUAL_CANCEL;
        zy100_cal_manager_set_status(ZY100_CAL_STATUS_INVALID_STATE, 0U);
        DBG_DIRECT("[MAG_CAL] pending_abort txn=%u reason=disconnect",
                   s_cal.transaction_id);
    }
    s_cal.info_confirmed = false;
    s_cal.record_confirmed = false;
    if (s_cal.conn_id == conn_id)
    {
        s_cal.conn_id = ZY100_CAL_CONN_INVALID;
        s_cal.tx_state = ZY100_CAL_TX_IDLE;
        s_cal.tx_source = ZY100_CAL_TX_SOURCE_NONE;
    }
    s_cal.write_active = false;
    s_cal.commit_pending = false;
    if (!zy100_cal_manager_mag_active())
    {
        memset(s_cal.mode.transfer.received, 0,
               sizeof(s_cal.mode.transfer.received));
    }
    zy100_cal_ble_service_reset(conn_id);
    zy100_cal_manager_set_status(s_cal.valid ? ZY100_CAL_STATUS_READY :
                                ZY100_CAL_STATUS_NO_DATA, 0U);
}

static bool zy100_cal_manager_all_received(void)
{
    uint16_t idx;
    for (idx = 0U; idx < s_cal.write_total; idx++)
    {
        if ((s_cal.mode.transfer.received[idx >> 3] &
             (uint8_t)(1U << (idx & 7U))) == 0U)
        {
            return false;
        }
    }
    return true;
}

static void zy100_cal_manager_begin_write(const zy100_cal_rx_frame_t *frame)
{
    if ((frame->payload_len != 0U) ||
        (frame->total_len < ZY100_CAL_RECORD_HEADER_BYTES) ||
        (frame->total_len > ZY100_CAL_RECORD_MAX_BYTES))
    {
        zy100_cal_manager_set_status(ZY100_CAL_STATUS_BAD_FRAME, 1U);
        return;
    }
    s_cal.tx_state = ZY100_CAL_TX_IDLE;
    s_cal.tx_source = ZY100_CAL_TX_SOURCE_NONE;
    memset(s_cal.mode.transfer.work.staging, 0,
           sizeof(s_cal.mode.transfer.work.staging));
    memset(s_cal.mode.transfer.received, 0,
           sizeof(s_cal.mode.transfer.received));
    s_cal.transaction_id = frame->transaction_id;
    s_cal.write_total = frame->total_len;
    s_cal.write_crc = frame->crc32;
    s_cal.write_active = true;
    s_cal.commit_pending = false;
    zy100_cal_manager_set_status(ZY100_CAL_STATUS_WRITE_RECEIVING, 0U);
}

static void zy100_cal_manager_write_chunk(const zy100_cal_rx_frame_t *frame)
{
    uint16_t idx;
    if (!s_cal.write_active ||
        (frame->transaction_id != s_cal.transaction_id) ||
        (frame->total_len != s_cal.write_total) ||
        (frame->crc32 != s_cal.write_crc) ||
        (frame->payload_len == 0U) ||
        (frame->offset > s_cal.write_total) ||
        (frame->payload_len > (uint16_t)(s_cal.write_total - frame->offset)))
    {
        zy100_cal_manager_set_status(ZY100_CAL_STATUS_BAD_FRAME, 2U);
        return;
    }
    for (idx = 0U; idx < frame->payload_len; idx++)
    {
        uint16_t pos = (uint16_t)(frame->offset + idx);
        uint8_t bit = (uint8_t)(1U << (pos & 7U));
        if (((s_cal.mode.transfer.received[pos >> 3] & bit) != 0U) &&
            (s_cal.mode.transfer.work.staging[pos] != frame->payload[idx]))
        {
            zy100_cal_manager_set_status(ZY100_CAL_STATUS_BAD_FRAME, 3U);
            return;
        }
    }
    memcpy(&s_cal.mode.transfer.work.staging[frame->offset],
           frame->payload, frame->payload_len);
    for (idx = 0U; idx < frame->payload_len; idx++)
    {
        uint16_t pos = (uint16_t)(frame->offset + idx);
        s_cal.mode.transfer.received[pos >> 3] |=
            (uint8_t)(1U << (pos & 7U));
    }
}

void zy100_cal_manager_on_write(uint8_t conn_id,
                                T_WRITE_TYPE write_type,
                                const uint8_t *data,
                                uint16_t len,
                                bool paired,
                                bool idle,
                                bool record_read_allowed)
{
    zy100_cal_rx_frame_t frame;
    if ((write_type != WRITE_REQUEST) && (write_type != WRITE_LONG))
    {
        zy100_cal_manager_set_status(ZY100_CAL_STATUS_BAD_FRAME, 4U);
        return;
    }
    if (!zy100_cal_rx_parse(data, len, &frame))
    {
        zy100_cal_manager_set_status(ZY100_CAL_STATUS_BAD_FRAME, 5U);
        return;
    }
    /* Do not let a retry/confirmation replace the accepted transaction. */
    if (s_cal.mag_start_pending)
    {
        zy100_cal_manager_set_status(ZY100_CAL_STATUS_BUSY, 0U);
        return;
    }
    /* A new confirmation must not replace an active transfer's identity. */
    if (((frame.opcode == ZY100_CAL_RX_CONFIRM_INFO) ||
         (frame.opcode == ZY100_CAL_RX_CONFIRM_CACHED_RECORD)) &&
        (s_cal.write_active || s_cal.commit_pending ||
         (s_cal.tx_state != ZY100_CAL_TX_IDLE)))
    {
        zy100_cal_manager_set_status_for_transaction(
            ZY100_CAL_STATUS_BUSY, 0U, frame.transaction_id);
        return;
    }
    s_cal.conn_id = conn_id;
    if (frame.opcode == ZY100_CAL_RX_CONFIRM_INFO)
    {
        bool mag_active = zy100_cal_manager_mag_active();
        uint8_t status_transaction_id = mag_active ? frame.transaction_id :
                                                    s_cal.transaction_id;

        s_cal.record_confirmed = false;
        if (!paired)
        {
            s_cal.info_confirmed = false;
            zy100_cal_manager_set_status_for_transaction(
                ZY100_CAL_STATUS_NOT_PAIRED, 0U, status_transaction_id);
        }
        else if (zy100_cal_confirm_info_matches(&frame, &s_cal.info,
                                               s_cal.valid))
        {
            /* Confirmation replies do not take ownership of a running MAG job. */
            if (!mag_active)
            {
                s_cal.transaction_id = frame.transaction_id;
            }
            status_transaction_id = frame.transaction_id;
            s_cal.info_confirmed = true;
            zy100_cal_manager_set_status_for_transaction(
                ZY100_CAL_STATUS_CAL_INFO_CONFIRMED, 0U, status_transaction_id);
        }
        else
        {
            s_cal.info_confirmed = false;
            zy100_cal_manager_set_status_for_transaction(
                ZY100_CAL_STATUS_CAL_INFO_MISMATCH, 0U, status_transaction_id);
        }
        return;
    }
    if (frame.opcode == ZY100_CAL_RX_CONFIRM_CACHED_RECORD)
    {
        s_cal.record_confirmed = false;
        if (!zy100_cal_manager_mag_active())
        {
            s_cal.transaction_id = frame.transaction_id;
        }
        if (!paired)
        {
            zy100_cal_manager_set_status_for_transaction(
                ZY100_CAL_STATUS_NOT_PAIRED, 0U, frame.transaction_id);
        }
        else if (!s_cal.info_confirmed)
        {
            zy100_cal_manager_set_status_for_transaction(
                ZY100_CAL_STATUS_CAL_INFO_REQUIRED, 0U, frame.transaction_id);
        }
        else if (zy100_cal_confirm_cached_record_matches(&frame, &s_cal.info,
                                                         s_cal.valid))
        {
            s_cal.record_confirmed = true;
            ZY100_LOG_ROUTINE(DBG_DIRECT, "[CAL_GATE] cached_record_confirmed conn=%u transaction=%u generation=%lu flags=0x%08lX bytes=%u crc=0x%08lX",
                       conn_id,
                       frame.transaction_id,
                       (unsigned long)s_cal.info.generation,
                       (unsigned long)s_cal.info.valid_flags,
                       s_cal.info.record_bytes,
                       (unsigned long)s_cal.info.crc32);
            zy100_cal_manager_set_status_for_transaction(
                ZY100_CAL_STATUS_CAL_RECORD_CACHE_CONFIRMED, 0U, frame.transaction_id);
        }
        else
        {
            zy100_cal_manager_set_status_for_transaction(
                ZY100_CAL_STATUS_BAD_RECORD, 0U, frame.transaction_id);
        }
        return;
    }
    if (frame.opcode == ZY100_CAL_RX_ACK)
    {
        if ((frame.transaction_id == s_cal.transaction_id) &&
            (s_cal.tx_state == ZY100_CAL_TX_WAIT_ACK) &&
            (s_cal.tx_source != ZY100_CAL_TX_SOURCE_NONE) &&
            (frame.crc32 == s_cal.tx_crc))
        {
            bool record_ack =
                (s_cal.tx_source == ZY100_CAL_TX_SOURCE_RECORD) &&
                s_cal.valid &&
                s_cal.info_confirmed &&
                (s_cal.tx_total == s_cal.info.record_bytes) &&
                (s_cal.tx_crc == s_cal.info.crc32);

            s_cal.tx_state = ZY100_CAL_TX_IDLE;
            s_cal.tx_source = ZY100_CAL_TX_SOURCE_NONE;
            if (record_ack)
            {
                s_cal.record_confirmed = true;
                ZY100_LOG_ROUTINE(DBG_DIRECT, "[CAL_GATE] record_confirmed conn=%u generation=%lu bytes=%u crc=0x%08lX",
                           conn_id,
                           (unsigned long)s_cal.info.generation,
                           s_cal.info.record_bytes,
                           (unsigned long)s_cal.info.crc32);
            }
            zy100_cal_manager_set_status(ZY100_CAL_STATUS_ACKED, 0U);
        }
        else
        {
            zy100_cal_manager_set_status(ZY100_CAL_STATUS_BAD_FRAME, 6U);
        }
        return;
    }
    if (frame.opcode == ZY100_CAL_RX_REQUEST)
    {
        if ((frame.payload_len != 0U) || (frame.offset != 0U) ||
            (frame.total_len != 0U) || (frame.crc32 != 0UL))
        {
            zy100_cal_manager_set_status(ZY100_CAL_STATUS_BAD_FRAME, 14U);
        }
        else if (!paired)
        {
            zy100_cal_manager_set_status(ZY100_CAL_STATUS_NOT_PAIRED, 0U);
        }
        else if (!s_cal.info_confirmed)
        {
            zy100_cal_manager_set_status(
                ZY100_CAL_STATUS_CAL_INFO_REQUIRED, 0U);
        }
        else if (s_cal.write_active || s_cal.commit_pending ||
                 zy100_cal_manager_mag_active() || !record_read_allowed ||
                 ((s_cal.tx_state != ZY100_CAL_TX_IDLE) &&
                  !((s_cal.tx_state == ZY100_CAL_TX_WAIT_ACK) &&
                    (s_cal.tx_source == ZY100_CAL_TX_SOURCE_RECORD))))
        {
            zy100_cal_manager_set_status(ZY100_CAL_STATUS_BUSY, 0U);
        }
        else
        {
            zy100_cal_manager_prepare_record_tx(conn_id, true, true);
        }
        return;
    }
    if (zy100_cal_manager_mag_active())
    {
        zy100_cal_manager_set_status(ZY100_CAL_STATUS_BUSY, 0U);
        return;
    }
    if (!paired)
    {
        zy100_cal_manager_set_status(ZY100_CAL_STATUS_NOT_PAIRED, 0U);
        return;
    }
    if (!idle)
    {
        zy100_cal_manager_set_status(ZY100_CAL_STATUS_BUSY, 0U);
        return;
    }
    switch (frame.opcode)
    {
    case ZY100_CAL_RX_REQUEST_DIAGNOSTICS:
        if ((frame.payload_len != 0U) || (frame.offset != 0U) ||
            (frame.total_len != 0U) || (frame.crc32 != 0UL))
        {
            zy100_cal_manager_set_status(ZY100_CAL_STATUS_BAD_FRAME, 9U);
        }
        else if (!s_cal.mag_diag_valid)
        {
            zy100_cal_manager_set_status(ZY100_CAL_STATUS_NO_DATA, 0U);
        }
        else if (s_cal.write_active || s_cal.commit_pending ||
                 (s_cal.tx_state != ZY100_CAL_TX_IDLE))
        {
            zy100_cal_manager_set_status(ZY100_CAL_STATUS_BUSY, 0U);
        }
        else
        {
            zy100_cal_manager_prepare_diag_tx(conn_id,
                                              frame.transaction_id);
        }
        break;
    case ZY100_CAL_RX_START_MAG_CAL:
    case ZY100_CAL_RX_START_MAG_CAL_WITH_POINTS:
        if (!zy100_cal_manager_business_ready(conn_id))
        {
            zy100_cal_manager_set_status(
                ZY100_CAL_STATUS_CAL_INFO_REQUIRED, 0U);
            break;
        }
        if ((frame.payload_len != 0U) || (frame.offset != 0U) ||
            (frame.total_len != 0U) || (frame.crc32 != 0UL) ||
            zy100_cal_manager_mag_active())
        {
            zy100_cal_manager_set_status(
                zy100_cal_manager_mag_active() ? ZY100_CAL_STATUS_BUSY :
                ZY100_CAL_STATUS_BAD_FRAME, 0U);
            break;
        }
        s_cal.transaction_id = frame.transaction_id;
        s_cal.tx_state = ZY100_CAL_TX_IDLE;
        s_cal.tx_source = ZY100_CAL_TX_SOURCE_NONE;
        s_cal.write_active = false;
        s_cal.commit_pending = false;
        s_cal.mag_start_pending = true;
        s_cal.mag_prepared = false;
        s_cal.mag_sensor_owned = false;
        s_cal.mag_sampling_started = false;
        s_cal.mag_prepare_deadline_ms = (uint32_t)os_sys_time_get() +
            zy100_cal_manager_port_prepare_budget_ms();
        zy100_cal_manager_mag_diag_reset();
        zy100_mag_calibrator_reset(&s_cal.mode.mag);
        s_cal.mag_read_errors = 0UL;
        s_cal.mag_fit_attempts = 0U;
        s_cal.mag_visual_state = ZY100_CAL_MAG_VISUAL_ACTIVE;
        zy100_cal_manager_set_status(ZY100_CAL_STATUS_CAL_ACCEPTED, 0U);
        break;
    case ZY100_CAL_RX_BEGIN_WRITE:
        zy100_cal_manager_begin_write(&frame);
        break;
    case ZY100_CAL_RX_CHUNK:
        zy100_cal_manager_write_chunk(&frame);
        break;
    case ZY100_CAL_RX_COMMIT:
        if (!s_cal.write_active ||
            (frame.transaction_id != s_cal.transaction_id) ||
            (frame.payload_len != 0U) || !zy100_cal_manager_all_received())
        {
            zy100_cal_manager_set_status(ZY100_CAL_STATUS_BAD_FRAME, 7U);
        }
        else
        {
            s_cal.commit_pending = true;
            zy100_cal_manager_set_status(ZY100_CAL_STATUS_COMMIT_PENDING, 0U);
        }
        break;
    case ZY100_CAL_RX_ABORT:
        s_cal.write_active = false;
        s_cal.commit_pending = false;
        zy100_cal_manager_set_status(s_cal.valid ? ZY100_CAL_STATUS_READY :
                                    ZY100_CAL_STATUS_NO_DATA, 0U);
        break;
    default:
        zy100_cal_manager_set_status(ZY100_CAL_STATUS_BAD_FRAME, 8U);
        break;
    }
}

static void zy100_cal_manager_commit(void)
{
    zy100_cal_record_info_t parsed;
    zy100_cal_store_status_t status;
    if (!zy100_cal_record_validate(s_cal.mode.transfer.work.staging,
                                   s_cal.write_total, &parsed) ||
        (parsed.crc32 != s_cal.write_crc) ||
        (s_cal.valid &&
         (((int32_t)(parsed.generation - s_cal.info.generation)) <= 0)))
    {
        s_cal.commit_pending = false;
        s_cal.write_active = false;
        zy100_cal_manager_set_status(ZY100_CAL_STATUS_BAD_RECORD, 0U);
        return;
    }
    status = zy100_cal_store_save(s_cal.mode.transfer.work.staging,
                                  s_cal.write_total, &parsed);
    s_cal.commit_pending = false;
    s_cal.write_active = false;
    if (status != ZY100_CAL_STORE_OK)
    {
        zy100_cal_manager_set_status(ZY100_CAL_STATUS_STORAGE_ERROR,
                                    (uint16_t)status);
        return;
    }
    memset(s_cal.blob, 0, sizeof(s_cal.blob));
    memcpy(s_cal.blob, s_cal.mode.transfer.work.staging, s_cal.write_total);
    s_cal.info = parsed;
    s_cal.valid = true;
    s_cal.info_confirmed = false;
    s_cal.record_confirmed = false;
    zy100_cal_manager_publish_info();
    zy100_cal_manager_set_status(ZY100_CAL_STATUS_WRITE_OK, 0U);
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[CAL] commit_ok generation=%lu bytes=%u flags=0x%08lX",
               (unsigned long)s_cal.info.generation,
               s_cal.info.record_bytes,
               (unsigned long)s_cal.info.valid_flags);
}

static void zy100_cal_manager_status_poll(uint8_t conn_id)
{
    uint8_t frame[ZY100_CAL_STATUS_BYTES];
    if (!s_cal.status_dirty ||
        !zy100_cal_ble_service_status_notify_enabled(conn_id))
    {
        return;
    }
    zy100_cal_build_status(frame, s_cal.status, s_cal.status_transaction_id,
                           s_cal.detail, s_cal.valid ? &s_cal.info : NULL);
    if (zy100_cal_ble_service_send_status(conn_id, frame, sizeof(frame)))
    {
        s_cal.status_dirty = false;
    }
}

static void zy100_cal_manager_tx_poll(uint8_t conn_id, uint16_t att_mtu)
{
    uint16_t notify_max;
    uint16_t payload_max;
    uint16_t payload_len = 0U;
    uint16_t frame_len;
    uint8_t frame_type;
    const uint8_t *payload = NULL;
    const uint8_t *source;

    if ((s_cal.tx_state == ZY100_CAL_TX_IDLE) ||
        (s_cal.tx_source == ZY100_CAL_TX_SOURCE_NONE) ||
        !zy100_cal_ble_service_tx_notify_enabled(conn_id))
    {
        return;
    }
    source = (s_cal.tx_source == ZY100_CAL_TX_SOURCE_DIAGNOSTICS) ?
             s_cal.mag_diag_blob : s_cal.blob;
    if (att_mtu < ZY100_CAL_DEFAULT_ATT_MTU)
    {
        att_mtu = ZY100_CAL_DEFAULT_ATT_MTU;
    }
    notify_max = (uint16_t)(att_mtu - 3U);
    if (notify_max > ZY100_CAL_MAX_NOTIFY_BYTES)
    {
        notify_max = ZY100_CAL_MAX_NOTIFY_BYTES;
    }
    if (notify_max < ZY100_CAL_TX_HEADER_BYTES)
    {
        return;
    }
    payload_max = (uint16_t)(notify_max - ZY100_CAL_TX_HEADER_BYTES);

    if (s_cal.tx_state == ZY100_CAL_TX_START_PENDING)
    {
        frame_type =
            (s_cal.tx_source == ZY100_CAL_TX_SOURCE_DIAGNOSTICS) ?
            ZY100_CAL_TX_DIAG_START : ZY100_CAL_TX_START;
    }
    else if (s_cal.tx_state == ZY100_CAL_TX_DATA_STREAM)
    {
        frame_type =
            (s_cal.tx_source == ZY100_CAL_TX_SOURCE_DIAGNOSTICS) ?
            ZY100_CAL_TX_DIAG_DATA : ZY100_CAL_TX_DATA;
        payload_len = (uint16_t)(s_cal.tx_total - s_cal.tx_offset);
        if (payload_len > payload_max)
        {
            payload_len = payload_max;
        }
        payload = &source[s_cal.tx_offset];
    }
    else if (s_cal.tx_state == ZY100_CAL_TX_END_PENDING)
    {
        frame_type =
            (s_cal.tx_source == ZY100_CAL_TX_SOURCE_DIAGNOSTICS) ?
            ZY100_CAL_TX_DIAG_END : ZY100_CAL_TX_END;
    }
    else
    {
        return;
    }
    frame_len = zy100_cal_build_tx_frame(s_cal.mode.transfer.work.tx_frame,
                                          sizeof(s_cal.mode.transfer.work.tx_frame),
                                         s_cal.transaction_id, frame_type,
                                         s_cal.tx_offset, payload, payload_len,
                                         s_cal.tx_total, s_cal.tx_crc);
    if ((frame_len == 0U) ||
        !zy100_cal_ble_service_send_tx(conn_id,
                                       s_cal.mode.transfer.work.tx_frame,
                                       frame_len))
    {
        return;
    }
    if (s_cal.tx_state == ZY100_CAL_TX_START_PENDING)
    {
        s_cal.tx_state = ZY100_CAL_TX_DATA_STREAM;
    }
    else if (s_cal.tx_state == ZY100_CAL_TX_DATA_STREAM)
    {
        s_cal.tx_offset = (uint16_t)(s_cal.tx_offset + payload_len);
        if (s_cal.tx_offset == s_cal.tx_total)
        {
            s_cal.tx_state = ZY100_CAL_TX_END_PENDING;
        }
    }
    else
    {
        s_cal.tx_state = ZY100_CAL_TX_WAIT_ACK;
        zy100_cal_manager_set_status(ZY100_CAL_STATUS_WAIT_ACK, 0U);
    }
}

static void zy100_cal_manager_mag_stop_sensor(void)
{
    mag_status_t status;
    if (!s_cal.mag_sensor_owned) return;
    status = mag_capture_service_end(MAG_CAPTURE_OWNER_CALIBRATION);

    if (status != MAG_STATUS_OK)
    {
        DBG_DIRECT("[MAG_CAL][ERR] power_down_failed status=%u",
                   (uint32_t)status);
    }
    else
    {
        s_cal.mag_sensor_owned = false;
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[MAG_CAL] power_down_ok txn=%u",
                   s_cal.transaction_id);
    }
}

static uint32_t zy100_cal_manager_mag_rms_x1e6(float rms)
{
    if (!(rms > 0.0f))
    {
        return 0UL;
    }
    if (rms >= 4294.0f)
    {
        return 0xFFFFFFFFUL;
    }
    return (uint32_t)(rms * 1000000.0f + 0.5f);
}

static void zy100_cal_manager_mag_diag_reset(void)
{
    memset(s_cal.mag_diag_blob, 0, sizeof(s_cal.mag_diag_blob));
    s_cal.mag_diag_valid = false;
    s_cal.mag_rejected_samples = 0UL;
    s_cal.mag_full_solve_status = ZY100_CAL_DIAG_STATUS_NOT_ATTEMPTED;
    s_cal.mag_axis_solve_status = ZY100_CAL_DIAG_STATUS_NOT_ATTEMPTED;
    s_cal.mag_hard_solve_status = ZY100_CAL_DIAG_STATUS_NOT_ATTEMPTED;
    s_cal.mag_result_rms_x1e6 = 0UL;
}

static void zy100_cal_manager_mag_diag_freeze(uint8_t completion_status,
                                               uint8_t final_model,
                                               uint16_t quality,
                                               uint32_t rms_x1e6,
                                               uint64_t now_ms)
{
    zy100_cal_mag_diag_t diagnostics;
    uint8_t axis;
    uint64_t elapsed_ms = (s_cal.mag_sampling_started && now_ms >= s_cal.mag_started_ms) ?
                          (now_ms - s_cal.mag_started_ms) : 0ULL;

    memset(&diagnostics, 0, sizeof(diagnostics));
    diagnostics.transaction_id = s_cal.transaction_id;
    diagnostics.completion_status = completion_status;
    diagnostics.final_model = final_model;
    diagnostics.sample_count = s_cal.mode.mag.sample_count;
    diagnostics.rejected_sample_count = s_cal.mag_rejected_samples;
    diagnostics.sensor_read_errors = s_cal.mag_read_errors;
    diagnostics.elapsed_ms = (elapsed_ms > 0xFFFFFFFFULL) ?
                             0xFFFFFFFFUL : (uint32_t)elapsed_ms;
    diagnostics.fit_attempts = s_cal.mag_fit_attempts;
    diagnostics.coverage_mask = s_cal.mode.mag.coverage_mask;
    diagnostics.direction_mask = s_cal.mode.mag.direction_mask;
    diagnostics.final_ready_status = (uint8_t)
        zy100_mag_calibrator_ready_status(&s_cal.mode.mag);
    diagnostics.full_solve_status = s_cal.mag_full_solve_status;
    diagnostics.axis_solve_status = s_cal.mag_axis_solve_status;
    diagnostics.hard_solve_status = s_cal.mag_hard_solve_status;
    diagnostics.quality = quality;
    diagnostics.rms_x1e6 = rms_x1e6;
    if (diagnostics.sample_count != 0UL)
    {
        for (axis = 0U; axis < 3U; axis++)
        {
            diagnostics.min_value[axis] =
                (uint32_t)s_cal.mode.mag.min_value[axis];
            diagnostics.max_value[axis] =
                (uint32_t)s_cal.mode.mag.max_value[axis];
        }
    }
    s_cal.mag_diag_valid = zy100_cal_build_mag_diagnostics(
        s_cal.mag_diag_blob, &diagnostics);
}

static void zy100_cal_manager_mag_fail(uint8_t status, uint16_t detail)
{
    if ((status == ZY100_CAL_STATUS_CAL_FAILED_TIMEOUT) ||
        (status == ZY100_CAL_STATUS_CAL_FAILED_COVERAGE) ||
        (status == ZY100_CAL_STATUS_CAL_FAILED_FIT) ||
        (status == ZY100_CAL_STATUS_CAL_FAILED_SENSOR) ||
        (status == ZY100_CAL_STATUS_STORAGE_ERROR))
    {
        if (s_cal.mag_diag_valid)
        {
            (void)zy100_cal_update_mag_diag_completion(
                s_cal.mag_diag_blob, ZY100_CAL_DIAG_COMPLETION_FAILED);
        }
        else
        {
            zy100_cal_manager_mag_diag_freeze(
                ZY100_CAL_DIAG_COMPLETION_FAILED,
                (uint8_t)ZY100_MAG_CAL_MODEL_NONE, 0U, 0UL,
                os_sys_time_get());
        }
    }
    zy100_cal_manager_mag_stop_sensor();
    s_cal.mag_start_pending = false;
    s_cal.mag_collecting = false;
    s_cal.mag_fit_pending = false;
    s_cal.mag_save_pending = false;
    s_cal.mag_final_fit = false;
    if ((status == ZY100_CAL_STATUS_CAL_FAILED_TIMEOUT) ||
        (status == ZY100_CAL_STATUS_CAL_FAILED_COVERAGE) ||
        (status == ZY100_CAL_STATUS_CAL_FAILED_FIT) ||
        (status == ZY100_CAL_STATUS_CAL_FAILED_SENSOR) ||
        (status == ZY100_CAL_STATUS_STORAGE_ERROR))
    {
        s_cal.mag_visual_state = ZY100_CAL_MAG_VISUAL_FAILURE;
    }
    else
    {
        s_cal.mag_visual_state = ZY100_CAL_MAG_VISUAL_CANCEL;
    }
    zy100_cal_manager_set_status(status, detail);
}

static uint8_t zy100_cal_manager_mag_model_rank(uint32_t valid_flags)
{
    if ((valid_flags & ZY100_CAL_VALID_MAG_MODEL) == 0UL)
    {
        return (uint8_t)ZY100_MAG_CAL_MODEL_NONE;
    }
    if ((valid_flags & ZY100_CAL_MAG_MODEL_HARD_IRON_ONLY) != 0UL)
    {
        return (uint8_t)ZY100_MAG_CAL_MODEL_HARD_IRON_ONLY;
    }
    if ((valid_flags & ZY100_CAL_MAG_MODEL_AXIS_ALIGNED) != 0UL)
    {
        return (uint8_t)ZY100_MAG_CAL_MODEL_AXIS_ALIGNED;
    }
    return (uint8_t)ZY100_MAG_CAL_MODEL_FULL;
}

static uint32_t zy100_cal_manager_mag_model_flags(
    zy100_mag_cal_model_t model)
{
    if (model == ZY100_MAG_CAL_MODEL_AXIS_ALIGNED)
    {
        return ZY100_CAL_MAG_MODEL_AXIS_ALIGNED;
    }
    if (model == ZY100_MAG_CAL_MODEL_HARD_IRON_ONLY)
    {
        return ZY100_CAL_MAG_MODEL_HARD_IRON_ONLY;
    }
    return 0UL;
}

static uint16_t zy100_cal_manager_mag_model_detail(
    zy100_mag_cal_model_t model)
{
    return (model == ZY100_MAG_CAL_MODEL_AXIS_ALIGNED) ?
           ZY100_CAL_DETAIL_MODEL_AXIS_ALIGNED :
           ZY100_CAL_DETAIL_MODEL_HARD_IRON_ONLY;
}

static bool zy100_cal_manager_mag_candidate_better(
    const zy100_mag_cal_result_t *result)
{
    uint8_t old_rank;

    if (result == NULL)
    {
        return false;
    }
    old_rank = s_cal.valid ?
               zy100_cal_manager_mag_model_rank(s_cal.info.valid_flags) :
               (uint8_t)ZY100_MAG_CAL_MODEL_NONE;
    if ((uint8_t)result->model != old_rank)
    {
        return (uint8_t)result->model > old_rank;
    }
    return (old_rank == (uint8_t)ZY100_MAG_CAL_MODEL_NONE) ||
           (result->quality >= s_cal.info.quality);
}

static void zy100_cal_manager_mag_complete_no_update(
    uint8_t conn_id,
    bool connected,
    uint16_t reason)
{
    bool has_old_mag = s_cal.valid &&
        ((s_cal.info.valid_flags & ZY100_CAL_VALID_MAG_MODEL) != 0UL);
    uint16_t detail = (uint16_t)(
        (has_old_mag ? ZY100_CAL_DETAIL_OLD_RECORD_RETAINED :
                       ZY100_CAL_DETAIL_NO_USABLE_RECORD) |
        (reason & 0x00FFU));

    zy100_cal_manager_mag_stop_sensor();
    s_cal.mag_start_pending = false;
    s_cal.mag_collecting = false;
    s_cal.mag_fit_pending = false;
    s_cal.mag_save_pending = false;
    s_cal.mag_final_fit = false;
    s_cal.mag_visual_state = ZY100_CAL_MAG_VISUAL_SUCCESS;
    zy100_cal_manager_set_status(
        ZY100_CAL_STATUS_CAL_COMPLETED_NO_UPDATE, detail);
    (void)conn_id;
    (void)connected;
    DBG_DIRECT("[MAG_CAL] no_update txn=%u reason=%u old=%u generation=%lu quality=%u",
               s_cal.transaction_id,
               reason,
               has_old_mag ? 1U : 0U,
               (unsigned long)(has_old_mag ? s_cal.info.generation : 0UL),
               has_old_mag ? s_cal.info.quality : 0U);
}

void zy100_cal_manager_abort_mag_for_shutdown(void)
{
    if (!zy100_cal_manager_mag_active())
    {
        return;
    }
    if (s_cal.mag_save_pending)
    {
        zy100_cal_manager_set_status(
            ZY100_CAL_STATUS_CAL_SHUTDOWN_PENDING, 0U);
        return;
    }
    zy100_cal_manager_mag_fail(s_cal.valid ? ZY100_CAL_STATUS_READY :
                              ZY100_CAL_STATUS_NO_DATA, 0U);
}

static void zy100_cal_manager_mag_start(void)
{
    mmc5603_cfg_t cfg;
    mag_status_t status;
    uint64_t now_ms;

    s_cal.mag_start_pending = false;
    zy100_cal_manager_mag_diag_reset();
    zy100_mag_calibrator_reset(&s_cal.mode.mag);
    s_cal.mag_read_errors = 0UL;
    s_cal.mag_fit_attempts = 0U;
    s_cal.mag_final_fit = false;
    s_cal.mag_result_model = (uint8_t)ZY100_MAG_CAL_MODEL_NONE;
    s_cal.mag_result_quality = 0U;
    now_ms = os_sys_time_get();
    s_cal.mag_started_ms = now_ms;
    zy100_cal_manager_set_status(ZY100_CAL_STATUS_CAL_STARTING, 0U);
    cfg.auto_sr_enable = true;
    cfg.bw = MMC5603_BW_LEVEL_01;
    cfg.continuous_odr = 0U;
    cfg.continuous_hpower = false;
    status = mag_capture_service_begin(MAG_CAPTURE_OWNER_CALIBRATION,
                                       &cfg, ZY100_FE_RAW_MAG_SAMPLE_HZ, false);
    if (status != MAG_STATUS_OK)
    {
        DBG_DIRECT("[MAG_CAL][ERR] start txn=%u status=%u owner=%u parked=%u",
                   s_cal.transaction_id, (uint32_t)status,
                   (uint32_t)mag_capture_service_owner(),
                   mag_capture_service_is_parked() ? 1U : 0U);
        zy100_cal_manager_mag_fail(ZY100_CAL_STATUS_CAL_FAILED_SENSOR, 0U);
        return;
    }
    now_ms = os_sys_time_get();
    s_cal.mag_started_ms = now_ms;
    s_cal.mag_sensor_owned = true;
    s_cal.mag_sampling_started = true;
    s_cal.mag_deadline_ms = now_ms + ZY100_MAG_CAL_TIMEOUT_MS;
    s_cal.mag_collecting = true;
    s_cal.mag_visual_state = ZY100_CAL_MAG_VISUAL_ACTIVE;
    zy100_cal_manager_set_status(ZY100_CAL_STATUS_CAL_COLLECTING, 0U);
    DBG_DIRECT("[MAG_CAL] collecting txn=%u timeout_ms=%lu odr=%u",
               s_cal.transaction_id,
               (unsigned long)ZY100_MAG_CAL_TIMEOUT_MS,
               (uint32_t)ZY100_FE_RAW_MAG_SAMPLE_HZ);
}

static uint16_t zy100_cal_manager_mag_solve_detail(
    zy100_mag_cal_solve_status_t status)
{
    switch (status)
    {
    case ZY100_MAG_CAL_SOLVE_LINEAR_SYSTEM:
        return ZY100_CAL_DETAIL_LINEAR_SYSTEM;
    case ZY100_MAG_CAL_SOLVE_SHAPE_INVERSE:
        return ZY100_CAL_DETAIL_SHAPE_INVERSE;
    case ZY100_MAG_CAL_SOLVE_NON_POSITIVE_SCALE:
        return ZY100_CAL_DETAIL_NON_POSITIVE_SCALE;
    case ZY100_MAG_CAL_SOLVE_NON_POSITIVE_ELLIPSOID:
        return ZY100_CAL_DETAIL_NON_POSITIVE_ELLIPSOID;
    case ZY100_MAG_CAL_SOLVE_INVALID_NORMALIZATION:
        return ZY100_CAL_DETAIL_INVALID_NORMALIZATION;
    case ZY100_MAG_CAL_SOLVE_NON_FINITE_RESULT:
        return ZY100_CAL_DETAIL_NON_FINITE_RESULT;
    case ZY100_MAG_CAL_SOLVE_INVALID_ARGUMENT:
    case ZY100_MAG_CAL_SOLVE_NOT_READY:
    default:
        return ZY100_CAL_DETAIL_LINEAR_SYSTEM;
    }
}

static void zy100_cal_manager_mag_schedule_fit(uint64_t now_ms,
                                               bool final_attempt)
{
    s_cal.mag_collecting = false;
    s_cal.mag_fit_pending = true;
    s_cal.mag_final_fit = final_attempt;
    s_cal.mag_fit_attempts++;
    zy100_cal_manager_set_status(ZY100_CAL_STATUS_CAL_FITTING, 100U);
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[MAG_CAL] fit_begin txn=%u attempt=%u final=%u elapsed_ms=%lu samples=%lu coverage=0x%02X",
               s_cal.transaction_id,
               s_cal.mag_fit_attempts,
               final_attempt ? 1U : 0U,
               (unsigned long)(now_ms - s_cal.mag_started_ms),
               (unsigned long)s_cal.mode.mag.sample_count,
               s_cal.mode.mag.coverage_mask);
}

static void zy100_cal_manager_mag_collect(void)
{
    mag_capture_sample_t sample;
    mag_status_t status;
    bool fresh = false;
    uint8_t progress;
    uint64_t now_ms;
    zy100_mag_cal_ready_status_t ready_status;

    status = mag_capture_service_read_fresh(false, &sample, &fresh);
    if (status != MAG_STATUS_OK)
    {
        s_cal.mag_read_errors++;
    }
    else if (fresh)
    {
        if (!zy100_mag_calibrator_add(&s_cal.mode.mag,
                                      sample.raw_x,
                                      sample.raw_y,
                                      sample.raw_z))
        {
            s_cal.mag_rejected_samples++;
        }
    }
    now_ms = os_sys_time_get();
    progress = zy100_mag_calibrator_progress(&s_cal.mode.mag);
    if ((s_cal.status != ZY100_CAL_STATUS_CAL_COLLECTING) ||
        (s_cal.detail != progress))
    {
        zy100_cal_manager_set_status(ZY100_CAL_STATUS_CAL_COLLECTING,
                                     progress);
    }
    ready_status = zy100_mag_calibrator_ready_status(&s_cal.mode.mag);
    if ((progress == 100U) && (ready_status == ZY100_MAG_CAL_READY))
    {
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[MAG_CAL] motion_complete txn=%u elapsed_ms=%lu samples=%lu movements=%lu directions=0x%02X",
                   s_cal.transaction_id,
                   (unsigned long)(now_ms - s_cal.mag_started_ms),
                   (unsigned long)s_cal.mode.mag.sample_count,
                   (unsigned long)s_cal.mode.mag.movement_count,
                   s_cal.mode.mag.direction_mask);
        zy100_cal_manager_mag_schedule_fit(now_ms, false);
        return;
    }
    if (now_ms >= s_cal.mag_deadline_ms)
    {
        if (s_cal.mode.mag.sample_count == 0UL)
        {
            zy100_cal_manager_mag_fail(
                ZY100_CAL_STATUS_CAL_FAILED_SENSOR,
                ZY100_CAL_DETAIL_SENSOR_READ);
        }
        else
        {
            uint16_t detail = ZY100_CAL_DETAIL_INSUFFICIENT_SAMPLES;

            if (ready_status == ZY100_MAG_CAL_NOT_READY_COVERAGE)
            {
                detail = ZY100_CAL_DETAIL_INSUFFICIENT_COVERAGE;
            }
            else if (ready_status == ZY100_MAG_CAL_NOT_READY_AXIS)
            {
                detail = ZY100_CAL_DETAIL_DEGENERATE_AXIS;
            }
            DBG_DIRECT("[MAG_CAL] motion_timeout txn=%u progress=%u samples=%lu movements=%lu directions=0x%02X ready=%u",
                       s_cal.transaction_id,
                       progress,
                       (unsigned long)s_cal.mode.mag.sample_count,
                       (unsigned long)s_cal.mode.mag.movement_count,
                       s_cal.mode.mag.direction_mask,
                       (uint32_t)ready_status);
            zy100_cal_manager_mag_fail(
                ZY100_CAL_STATUS_CAL_FAILED_TIMEOUT, detail);
        }
        return;
    }
}

static void zy100_cal_manager_mag_fit(void)
{
    zy100_mag_cal_result_t result;
    zy100_cal_record_info_t parsed;
    zy100_mag_cal_solve_status_t solve_status;
    uint64_t now_ms = os_sys_time_get();
    uint32_t next_generation = s_cal.valid ?
                               (s_cal.info.generation + 1UL) : 1UL;

    memset(&result, 0, sizeof(result));
    if (zy100_mag_calibrator_ready_status(&s_cal.mode.mag) !=
        ZY100_MAG_CAL_READY)
    {
        s_cal.mag_fit_pending = false;
        zy100_cal_manager_mag_diag_freeze(
            ZY100_CAL_DIAG_COMPLETION_FAILED,
            (uint8_t)ZY100_MAG_CAL_MODEL_NONE, 0U, 0UL, now_ms);
        zy100_cal_manager_mag_fail(
            ZY100_CAL_STATUS_CAL_FAILED_COVERAGE,
            ZY100_CAL_DETAIL_INSUFFICIENT_COVERAGE);
        return;
    }
    solve_status = zy100_mag_calibrator_solve_hard_iron_ex(
        &s_cal.mode.mag, &result);
    s_cal.mag_hard_solve_status = (uint8_t)solve_status;
    if (solve_status != ZY100_MAG_CAL_SOLVE_OK)
    {
        uint16_t detail = zy100_cal_manager_mag_solve_detail(solve_status);
        DBG_DIRECT("[MAG_CAL] fit_fail txn=%u attempt=%u final=%u elapsed_ms=%lu samples=%lu coverage=0x%02X reason=%u",
                   s_cal.transaction_id,
                   s_cal.mag_fit_attempts,
                   s_cal.mag_final_fit ? 1U : 0U,
                   (unsigned long)(now_ms - s_cal.mag_started_ms),
                   (unsigned long)s_cal.mode.mag.sample_count,
                   s_cal.mode.mag.coverage_mask,
                   detail);
        s_cal.mag_fit_pending = false;
        zy100_cal_manager_mag_diag_freeze(
            ZY100_CAL_DIAG_COMPLETION_FAILED,
            (uint8_t)ZY100_MAG_CAL_MODEL_NONE, 0U, 0UL, now_ms);
        zy100_cal_manager_mag_fail(
            ZY100_CAL_STATUS_CAL_FAILED_FIT, detail);
        return;
    }
    if (!zy100_cal_manager_mag_candidate_better(&result))
    {
        zy100_cal_manager_mag_diag_freeze(
            ZY100_CAL_DIAG_COMPLETION_NO_UPDATE,
            (uint8_t)result.model, result.quality,
            zy100_cal_manager_mag_rms_x1e6(result.algebraic_rms), now_ms);
        zy100_cal_manager_mag_complete_no_update(
            s_cal.conn_id,
            s_cal.conn_id != ZY100_CAL_CONN_INVALID,
            ZY100_CAL_DETAIL_CANDIDATE_NOT_BETTER);
        return;
    }
    s_cal.mag_result_model = (uint8_t)result.model;
    s_cal.mag_result_quality = result.quality;
    s_cal.mag_result_rms_x1e6 =
        zy100_cal_manager_mag_rms_x1e6(result.algebraic_rms);
    zy100_cal_manager_mag_diag_freeze(
        (result.model == ZY100_MAG_CAL_MODEL_FULL) ?
        ZY100_CAL_DIAG_COMPLETION_SUCCESS :
        ZY100_CAL_DIAG_COMPLETION_LIMITED,
        s_cal.mag_result_model, s_cal.mag_result_quality,
        s_cal.mag_result_rms_x1e6, now_ms);
    if (!zy100_cal_record_replace_mag_model(
             s_cal.valid ? s_cal.blob : NULL,
             s_cal.valid ? s_cal.info.record_bytes : 0U,
             next_generation, result.quality,
             zy100_cal_manager_mag_model_flags(result.model), 0ULL,
             result.bias, result.matrix,
             s_cal.mode.transfer.work.staging, &parsed))
    {
        s_cal.mag_fit_pending = false;
        zy100_cal_manager_mag_fail(ZY100_CAL_STATUS_CAL_FAILED_FIT,
                                   ZY100_CAL_DETAIL_RECORD_ENCODE);
        return;
    }
    zy100_cal_manager_mag_stop_sensor();
    s_cal.mag_fit_pending = false;
    s_cal.mag_final_fit = false;
    s_cal.mag_save_pending = true;
    s_cal.write_total = parsed.record_bytes;
    s_cal.write_crc = parsed.crc32;
    zy100_cal_manager_set_status(ZY100_CAL_STATUS_CAL_SAVING, 100U);
    ZY100_LOG_ROUTINE(DBG_DIRECT, "[MAG_CAL] fit_ok txn=%u attempt=%u elapsed_ms=%lu samples=%lu coverage=0x%02X directions=0x%02X model=%u rms=%d quality=%u",
               s_cal.transaction_id,
               s_cal.mag_fit_attempts,
               (unsigned long)(now_ms - s_cal.mag_started_ms),
               (unsigned long)result.sample_count,
               result.coverage_mask,
               result.direction_mask,
               (uint32_t)result.model,
               (int)(result.algebraic_rms * 1000000.0f),
               result.quality);
}

static void zy100_cal_manager_mag_save(void)
{
    zy100_cal_record_info_t parsed;
    zy100_cal_store_status_t status = zy100_cal_store_save(
        s_cal.mode.transfer.work.staging, s_cal.write_total, &parsed);

    s_cal.mag_save_pending = false;
    if (status != ZY100_CAL_STORE_OK)
    {
        if (s_cal.mag_diag_valid)
        {
            (void)zy100_cal_update_mag_diag_completion(
                s_cal.mag_diag_blob, ZY100_CAL_DIAG_COMPLETION_FAILED);
        }
        s_cal.mag_visual_state = ZY100_CAL_MAG_VISUAL_FAILURE;
        zy100_cal_manager_set_status(ZY100_CAL_STATUS_STORAGE_ERROR,
                                     (uint16_t)status);
        return;
    }
    memset(s_cal.blob, 0, sizeof(s_cal.blob));
    memcpy(s_cal.blob, s_cal.mode.transfer.work.staging,
           parsed.record_bytes);
    s_cal.info = parsed;
    s_cal.valid = true;
    s_cal.info_confirmed = false;
    s_cal.record_confirmed = false;
    s_cal.mag_visual_state = ZY100_CAL_MAG_VISUAL_SUCCESS;
    zy100_cal_manager_publish_info();
    if (s_cal.mag_result_model == (uint8_t)ZY100_MAG_CAL_MODEL_FULL)
    {
        zy100_cal_manager_set_status(ZY100_CAL_STATUS_CAL_SUCCESS, 100U);
    }
    else
    {
        zy100_cal_manager_set_status(
            ZY100_CAL_STATUS_CAL_SUCCESS_LIMITED,
            zy100_cal_manager_mag_model_detail(
                (zy100_mag_cal_model_t)s_cal.mag_result_model));
    }
    DBG_DIRECT("[MAG_CAL] save_ok txn=%u model=%u quality=%u generation=%lu bytes=%u crc=0x%08lX",
               s_cal.transaction_id,
               (uint32_t)s_cal.mag_result_model,
               s_cal.mag_result_quality,
               (unsigned long)parsed.generation, parsed.record_bytes,
               (unsigned long)parsed.crc32);
}

bool zy100_cal_manager_shutdown_poll(void)
{
    zy100_cal_manager_abort_mag_for_shutdown();
    if (s_cal.mag_save_pending)
    {
        /* Save is synchronous. It completes or retains the old valid record
         * before this call returns; never transmit a result during shutdown. */
        zy100_cal_manager_mag_save();
    }
    s_cal.tx_state = ZY100_CAL_TX_IDLE;
    s_cal.tx_source = ZY100_CAL_TX_SOURCE_NONE;
    s_cal.conn_id = ZY100_CAL_CONN_INVALID;
    s_cal.write_active = false;
    s_cal.commit_pending = false;
    s_cal.info_confirmed = false;
    s_cal.record_confirmed = false;
    s_cal.mag_visual_state = ZY100_CAL_MAG_VISUAL_CANCEL;
    return !zy100_cal_manager_mag_active();
}

static void zy100_cal_manager_mag_abort_pending(void)
{
    if (!s_cal.mag_start_pending)
    {
        return;
    }

    s_cal.mag_start_pending = false;
    s_cal.mag_visual_state = ZY100_CAL_MAG_VISUAL_CANCEL;
    zy100_cal_manager_set_status(ZY100_CAL_STATUS_INVALID_STATE, 0U);
    DBG_DIRECT("[MAG_CAL] pending_abort txn=%u reason=link_gate",
               s_cal.transaction_id);
}

static void zy100_cal_manager_mag_poll(
    zy100_cal_mag_start_gate_t mag_start_gate)
{
    if (s_cal.mag_start_pending)
    {
        if (mag_start_gate == ZY100_CAL_MAG_START_GATE_ABORT)
        {
            zy100_cal_manager_mag_abort_pending();
        }
        else if ((int32_t)((uint32_t)os_sys_time_get() -
                           s_cal.mag_prepare_deadline_ms) >= 0)
        {
            DBG_DIRECT("[MAG_CAL][ERR] prepare_timeout txn=%u prepared=%u",
                       s_cal.transaction_id, s_cal.mag_prepared ? 1U : 0U);
            zy100_cal_manager_mag_fail(ZY100_CAL_STATUS_CAL_FAILED_TIMEOUT, 0U);
        }
        else if (mag_start_gate == ZY100_CAL_MAG_START_GATE_RESOURCE_FAILED)
        {
            zy100_cal_manager_mag_fail(ZY100_CAL_STATUS_CAL_FAILED_SENSOR, 0U);
        }
        else if ((mag_start_gate == ZY100_CAL_MAG_START_GATE_READY) &&
                 s_cal.mag_prepared)
        {
            zy100_cal_manager_mag_start();
        }
    }
    else if (s_cal.mag_collecting)
    {
        zy100_cal_manager_mag_collect();
    }
    else if (s_cal.mag_fit_pending)
    {
        zy100_cal_manager_mag_fit();
    }
    else if (s_cal.mag_save_pending)
    {
        zy100_cal_manager_mag_save();
    }
}

void zy100_cal_manager_poll(uint8_t conn_id,
                            uint16_t att_mtu,
                            bool connected,
                            bool paired,
                            bool idle,
                            zy100_cal_mag_start_gate_t mag_start_gate)
{
    zy100_cal_manager_mag_poll(mag_start_gate);
    if (!connected || (conn_id == ZY100_CAL_CONN_INVALID))
    {
        return;
    }
    if (s_cal.commit_pending)
    {
        if (paired && idle)
        {
            zy100_cal_manager_commit();
        }
        else
        {
            s_cal.commit_pending = false;
            s_cal.write_active = false;
            zy100_cal_manager_set_status(paired ? ZY100_CAL_STATUS_BUSY :
                                        ZY100_CAL_STATUS_NOT_PAIRED, 0U);
        }
    }
    if (paired)
    {
        zy100_cal_manager_tx_poll(conn_id, att_mtu);
    }
    zy100_cal_manager_status_poll(conn_id);
}
