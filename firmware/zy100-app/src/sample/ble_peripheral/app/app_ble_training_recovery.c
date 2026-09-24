#include "app_ble_training_recovery.h"
#include "../common/zy100_byteorder.h"
#include <string.h>

static struct
{
    uint8_t bytes[ZY100_TRAINING_SNAPSHOT_BYTES];
    uint32_t generation;
    uint32_t user_id;
    uint32_t request_id;
    uint32_t started_ms;
    bool valid;
} s_snapshot;

void app_ble_training_snapshot_reset(void)
{
    memset(&s_snapshot, 0, sizeof(s_snapshot));
}

bool app_ble_training_snapshot_cached(uint32_t generation, uint32_t user_id,
                                      uint32_t request_id)
{
    return s_snapshot.valid && s_snapshot.generation == generation &&
           s_snapshot.user_id == user_id && s_snapshot.request_id == request_id;
}

static void snapshot_encode(const zy100_training_snapshot_t *value, uint32_t id)
{
    uint8_t *p = s_snapshot.bytes;
    memset(p, 0, ZY100_TRAINING_SNAPSHOT_BYTES);
    p[0] = ZY100_TRAINING_SNAPSHOT_VERSION;
    p[1] = value->phase;
    p[2] = value->flags;
    p[3] = value->source;
    memcpy(p + 4, value->device_address, 6U);
    memcpy(p + 12, value->token, ZY100_TRAINING_TOKEN_BYTES);
    zy100_put_u32_le(p + 28, value->session_id);
    zy100_put_u32_le(p + 32, value->storage_generation);
    zy100_put_u32_le(p + 36, value->owner_user_id);
    zy100_put_u64_le(p + 40, value->start_unix_ms);
    zy100_put_u64_le(p + 48, value->elapsed_ms);
    zy100_put_u32_le(p + 56, id);
    /* bytes 10..11 and 60..63 are zero, reserved for this version. */
}

zy100_ble_ack_status_t app_ble_training_snapshot_page(
    uint32_t generation, uint32_t user_id, uint32_t request_id,
    uint32_t page, uint32_t now_ms, const zy100_training_snapshot_t *fresh,
    uint8_t payload[8], uint32_t *detail)
{
    if (detail == NULL) return ZY100_BLE_ACK_STATUS_INVALID_STATE;
    *detail = ZY100_BLE_OFFLINE_DETAIL_SNAPSHOT_ARGS;
    if (payload == NULL || user_id == 0U || request_id == 0U ||
        page >= APP_TRAINING_SNAPSHOT_PAGE_COUNT) return ZY100_BLE_ACK_STATUS_INVALID_STATE;
    if (!app_ble_training_snapshot_cached(generation, user_id, request_id))
    {
        if (s_snapshot.valid && s_snapshot.generation == generation &&
            s_snapshot.user_id == user_id && request_id <= s_snapshot.request_id)
        {
            *detail = ZY100_BLE_OFFLINE_DETAIL_SNAPSHOT_EXPIRED;
            return ZY100_BLE_ACK_STATUS_NOT_READY;
        }
        if (page != 0U || fresh == NULL)
        {
            *detail = ZY100_BLE_OFFLINE_DETAIL_SNAPSHOT_EXPIRED;
            return ZY100_BLE_ACK_STATUS_NOT_READY;
        }
        if ((fresh->flags & ZY100_TRAINING_FLAG_ACTIVE) && fresh->owner_user_id != user_id)
            return ZY100_BLE_ACK_STATUS_INVALID_STATE;
        snapshot_encode(fresh, request_id);
        s_snapshot.generation = generation;
        s_snapshot.user_id = user_id;
        s_snapshot.request_id = request_id;
        s_snapshot.started_ms = now_ms;
        s_snapshot.valid = true;
    }
    if ((uint32_t)(now_ms - s_snapshot.started_ms) >= APP_TRAINING_SNAPSHOT_TTL_MS)
    {
        *detail = ZY100_BLE_OFFLINE_DETAIL_SNAPSHOT_EXPIRED;
        return ZY100_BLE_ACK_STATUS_NOT_READY;
    }
    memcpy(payload, s_snapshot.bytes + page * 8U, 8U);
    *detail = 0U;
    return ZY100_BLE_ACK_STATUS_OK;
}
