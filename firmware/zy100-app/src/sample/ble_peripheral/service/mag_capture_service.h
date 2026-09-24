#ifndef MAG_CAPTURE_SERVICE_H
#define MAG_CAPTURE_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "../driver/mmc5603.h"
#include "sensor_sample.h"

typedef struct
{
    sensor_sample_header_t header;
    uint8_t raw9[9];
    uint32_t raw_x;
    uint32_t raw_y;
    uint32_t raw_z;
    uint8_t raw_temp;
    bool temp_valid;
    uint64_t meas_done_local_ts;
} mag_capture_sample_t;

typedef void (*mag_capture_deferred_hook_t)(const mag_capture_sample_t *sample, void *user_ctx);

typedef enum
{
    MAG_CAPTURE_LIFECYCLE_RAIL_OFF = 0,
    MAG_CAPTURE_LIFECYCLE_RAIL_ON_UNPARKED,
    MAG_CAPTURE_LIFECYCLE_PARKED,
    MAG_CAPTURE_LIFECYCLE_ACTIVE,
} mag_capture_lifecycle_t;

/* Read-only diagnostic state; callers must not use it to fabricate readiness. */
mag_capture_lifecycle_t mag_capture_service_lifecycle(void);

typedef enum
{
    MAG_CAPTURE_OWNER_NONE = 0,
    MAG_CAPTURE_OWNER_ONLINE,
    MAG_CAPTURE_OWNER_OFFLINE_V2,
    MAG_CAPTURE_OWNER_CALIBRATION,
} mag_capture_owner_t;

/* The app calls these only after the physical SENSOR-PWR transition and the
 * existing board-level rail/bus restore sequence have completed. */
mag_status_t mag_capture_service_notify_sensor_rail_on(void);
void mag_capture_service_notify_sensor_rail_off(void);

/* PARKED means MMC5603 Power Down was successfully applied while the rail is
 * still on.  RAIL_OFF is deliberately not treated as a successful park. */
mag_status_t mag_capture_service_park(void);
bool mag_capture_service_is_parked(void);
mag_capture_owner_t mag_capture_service_owner(void);

/* Only an explicit capture profile or magnetic calibration may own an active
 * MMC5603 measurement session.  begin() always starts from PARKED and end()
 * always returns the device to Power Down. */
mag_status_t mag_capture_service_begin(mag_capture_owner_t owner,
                                       const mmc5603_cfg_t *cfg,
                                       uint8_t odr,
                                       bool hpower);
mag_status_t mag_capture_service_end(mag_capture_owner_t owner);

mag_status_t mag_capture_service_single_shot(bool read_temp, mag_capture_sample_t *sample_out);
mag_status_t mag_capture_service_read_latest(bool read_temp, mag_capture_sample_t *sample_out);
mag_status_t mag_capture_service_read_fresh(bool read_temp,
                                            mag_capture_sample_t *sample_out,
                                            bool *fresh_out);

void mag_capture_service_set_deferred_hook(mag_capture_deferred_hook_t hook, void *user_ctx);
void mag_capture_service_poll_hook(void);
bool mag_capture_service_try_dequeue(mag_capture_sample_t *sample_out);

#ifdef __cplusplus
}
#endif

#endif /* MAG_CAPTURE_SERVICE_H */
