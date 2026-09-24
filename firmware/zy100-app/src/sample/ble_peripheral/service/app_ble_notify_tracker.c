#include "app_ble_notify_tracker.h"

#include "app_flags.h"

#define APP_BLE_NOTIFY_TRACKER_SATURATED 0xFFFFU

static uint16_t s_notify_in_flight[APP_MAX_LINKS];

void app_ble_notify_tracker_reset_all(void)
{
    uint8_t conn_id;

    for (conn_id = 0U; conn_id < APP_MAX_LINKS; conn_id++)
    {
        s_notify_in_flight[conn_id] = 0U;
    }
}

void app_ble_notify_tracker_reset_conn(uint8_t conn_id)
{
    if (conn_id < APP_MAX_LINKS)
    {
        s_notify_in_flight[conn_id] = 0U;
    }
}

void app_ble_notify_tracker_note_submit(uint8_t conn_id)
{
    if ((conn_id < APP_MAX_LINKS) &&
        (s_notify_in_flight[conn_id] < APP_BLE_NOTIFY_TRACKER_SATURATED))
    {
        s_notify_in_flight[conn_id]++;
    }
}

void app_ble_notify_tracker_note_complete(uint8_t conn_id)
{
    if ((conn_id < APP_MAX_LINKS) && (s_notify_in_flight[conn_id] > 0U))
    {
        s_notify_in_flight[conn_id]--;
    }
}

bool app_ble_notify_tracker_in_flight(void)
{
    return app_ble_notify_tracker_count() != 0U;
}

uint16_t app_ble_notify_tracker_count(void)
{
    uint8_t conn_id;
    uint16_t total = 0U;

    for (conn_id = 0U; conn_id < APP_MAX_LINKS; conn_id++)
    {
        uint16_t count = s_notify_in_flight[conn_id];
        if ((uint16_t)(APP_BLE_NOTIFY_TRACKER_SATURATED - total) < count)
        {
            return APP_BLE_NOTIFY_TRACKER_SATURATED;
        }
        total = (uint16_t)(total + count);
    }
    return total;
}
