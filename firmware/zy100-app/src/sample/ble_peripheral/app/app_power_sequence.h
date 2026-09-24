#ifndef APP_POWER_SEQUENCE_H
#define APP_POWER_SEQUENCE_H

#include <stdbool.h>
#include <stdint.h>

/* Application retry scheduling, not device timing. Keep every attempt bounded
 * by the existing driver timeout and return to the event loop between attempts. */
typedef struct
{
    uint32_t due_ms;
    uint8_t failures;
    bool waiting;
} app_power_retry_t;

static inline bool app_power_retry_due(const app_power_retry_t *retry, uint32_t now)
{
    return !retry->waiting || (int32_t)(now - retry->due_ms) >= 0;
}

static inline void app_power_retry_clear(app_power_retry_t *retry)
{
    retry->due_ms = 0U;
    retry->failures = 0U;
    retry->waiting = false;
}

static inline void app_power_retry_fail(app_power_retry_t *retry, uint32_t now)
{
    if (retry->failures < 255U) ++retry->failures;
    retry->due_ms = now + ((retry->failures <= 3U) ? 250U : 1000U);
    retry->waiting = true;
}

typedef enum
{
    APP_SENSOR_STOP_VERIFY = 0,
    APP_SENSOR_STOP_IMU,
    APP_SENSOR_STOP_MAG,
    APP_SENSOR_STOP_FLASH,
    APP_SENSOR_STOP_POWER,
    APP_SENSOR_STOP_IO,
    APP_SENSOR_STOP_DONE
} app_sensor_stop_step_t;

typedef enum
{
    APP_MAG_PARK_OK = 0,
    APP_MAG_PARK_BUSY,
    APP_MAG_PARK_POWER_QUERY,
    APP_MAG_PARK_UNSETTLED,
    APP_MAG_PARK_LIFECYCLE,
    APP_MAG_PARK_IO
} app_mag_park_result_t;

typedef struct
{
    app_power_retry_t retry;
    app_sensor_stop_step_t step;
} app_sensor_stop_t;

#endif
