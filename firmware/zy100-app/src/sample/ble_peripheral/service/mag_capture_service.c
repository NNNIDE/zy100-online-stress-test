#include "mag_capture_service.h"

#include <string.h>

#include "trace.h"
#include "../app_flags.h"

static volatile bool s_mag_pending = false;
static mag_capture_sample_t s_mag_pending_sample;
static mag_capture_deferred_hook_t s_mag_deferred_hook = NULL;
static void *s_mag_deferred_hook_ctx = NULL;


static mag_capture_lifecycle_t s_mag_lifecycle =
    MAG_CAPTURE_LIFECYCLE_RAIL_OFF;
static mag_capture_owner_t s_mag_owner = MAG_CAPTURE_OWNER_NONE;

static bool mag_capture_owner_is_valid(mag_capture_owner_t owner)
{
    return (owner == MAG_CAPTURE_OWNER_ONLINE) ||
           (owner == MAG_CAPTURE_OWNER_OFFLINE_V2) ||
           (owner == MAG_CAPTURE_OWNER_CALIBRATION);
}

static bool mag_capture_measurement_is_active(void)
{
    return (s_mag_lifecycle == MAG_CAPTURE_LIFECYCLE_ACTIVE) &&
           (s_mag_owner != MAG_CAPTURE_OWNER_NONE);
}

static void mag_capture_clear_pending(void)
{
    s_mag_pending = false;
    memset(&s_mag_pending_sample, 0, sizeof(s_mag_pending_sample));
}

static void mag_capture_fill_header(const mmc5603_sample_t *in, mag_capture_sample_t *out)
{
    out->header.sensor_type = SENSOR_TYPE_MAG;
    out->header.source_mode = (in->source_mode == MMC5603_SOURCE_MODE_SINGLE) ?
                              SENSOR_SOURCE_SINGLE : SENSOR_SOURCE_CONTINUOUS;
    out->header.status_flags = in->status_flags;
    out->header.sample_seq = in->sample_seq;
    out->header.trig_ts = in->trig_local_ts_us;
    out->header.read_ts = in->readout_local_ts_us;

    memcpy(out->raw9, in->raw9, sizeof(out->raw9));
    out->raw_x = in->raw_x;
    out->raw_y = in->raw_y;
    out->raw_z = in->raw_z;
    out->raw_temp = in->raw_temp;
    out->temp_valid = in->temp_valid;
    out->meas_done_local_ts = in->meas_done_local_ts_us;
}

static void mag_capture_publish_sample(const mag_capture_sample_t *sample)
{
    s_mag_pending_sample = *sample;
    s_mag_pending = true;
}

mag_status_t mag_capture_service_notify_sensor_rail_on(void)
{
    if (mag_capture_measurement_is_active())
    {
        return MAG_STATUS_BUS_BUSY;
    }

    if (s_mag_lifecycle == MAG_CAPTURE_LIFECYCLE_RAIL_OFF)
    {
        s_mag_lifecycle = MAG_CAPTURE_LIFECYCLE_RAIL_ON_UNPARKED;
        s_mag_owner = MAG_CAPTURE_OWNER_NONE;
        mag_capture_clear_pending();
    }

    return MAG_STATUS_OK;
}

void mag_capture_service_notify_sensor_rail_off(void)
{
    s_mag_lifecycle = MAG_CAPTURE_LIFECYCLE_RAIL_OFF;
    s_mag_owner = MAG_CAPTURE_OWNER_NONE;
    mag_capture_clear_pending();
}

mag_status_t mag_capture_service_park(void)
{
    mag_status_t status;
    mag_capture_owner_t owner;

    if (s_mag_lifecycle == MAG_CAPTURE_LIFECYCLE_RAIL_OFF)
    {
        return MAG_STATUS_NOT_READY;
    }
    if (s_mag_lifecycle == MAG_CAPTURE_LIFECYCLE_PARKED)
    {
        ZY100_LOG_DETAIL("[MAG_LIFE] pd_cached owner=%u",
                         (uint32_t)MAG_CAPTURE_OWNER_NONE);
        return MAG_STATUS_OK;
    }

    owner = s_mag_owner;
    status = mmc5603_force_power_down();
    if (status == MAG_STATUS_OK)
    {
        s_mag_lifecycle = MAG_CAPTURE_LIFECYCLE_PARKED;
        s_mag_owner = MAG_CAPTURE_OWNER_NONE;
        mag_capture_clear_pending();
        ZY100_LOG_DETAIL("[MAG_LIFE] pd_ok owner=%u", (uint32_t)owner);
    }
    return status;
}

mag_capture_lifecycle_t mag_capture_service_lifecycle(void)
{
    return s_mag_lifecycle;
}

bool mag_capture_service_is_parked(void)
{
    return (s_mag_lifecycle == MAG_CAPTURE_LIFECYCLE_PARKED) &&
           (s_mag_owner == MAG_CAPTURE_OWNER_NONE);
}

mag_capture_owner_t mag_capture_service_owner(void)
{
    return s_mag_owner;
}

mag_status_t mag_capture_service_begin(mag_capture_owner_t owner,
                                       const mmc5603_cfg_t *cfg,
                                       uint8_t odr,
                                       bool hpower)
{
    mmc5603_cfg_t init_cfg;
    mag_status_t status;

    if (!mag_capture_owner_is_valid(owner) || (cfg == NULL) || (odr == 0U))
    {
        return MAG_STATUS_INVALID_PARAM;
    }
    if (s_mag_owner != MAG_CAPTURE_OWNER_NONE)
    {
        return MAG_STATUS_BUS_BUSY;
    }
    if (s_mag_lifecycle != MAG_CAPTURE_LIFECYCLE_PARKED)
    {
        return MAG_STATUS_NOT_READY;
    }

    /* The explicit start below is the only point that enables continuous
     * measurement for an application session. */
    init_cfg = *cfg;
    init_cfg.continuous_odr = 0U;
    init_cfg.continuous_hpower = false;
    s_mag_lifecycle = MAG_CAPTURE_LIFECYCLE_RAIL_ON_UNPARKED;
    mag_capture_clear_pending();
    s_mag_deferred_hook = NULL;
    s_mag_deferred_hook_ctx = NULL;

    status = mmc5603_init(&init_cfg);
    if (status != MAG_STATUS_OK)
    {
        mag_status_t cleanup = mag_capture_service_park();
        DBG_DIRECT("[MAG_LIFE][ERR] begin init=%u cleanup=%u", status, cleanup);
        return status;
    }

    status = mmc5603_set_continuous_mode(odr, hpower, true);
    if (status != MAG_STATUS_OK)
    {
        mag_status_t cleanup = mag_capture_service_park();
        DBG_DIRECT("[MAG_LIFE][ERR] begin continuous=%u cleanup=%u", status, cleanup);
        return status;
    }

    s_mag_owner = owner;
    s_mag_lifecycle = MAG_CAPTURE_LIFECYCLE_ACTIVE;
    return MAG_STATUS_OK;
}

mag_status_t mag_capture_service_end(mag_capture_owner_t owner)
{
    if (!mag_capture_owner_is_valid(owner))
    {
        return MAG_STATUS_INVALID_PARAM;
    }
    if (s_mag_owner == owner)
    {
        return mag_capture_service_park();
    }
    if (s_mag_owner != MAG_CAPTURE_OWNER_NONE)
    {
        return MAG_STATUS_BUS_BUSY;
    }
    if (s_mag_lifecycle == MAG_CAPTURE_LIFECYCLE_PARKED)
    {
        return MAG_STATUS_OK;
    }
    return MAG_STATUS_NOT_READY;
}

mag_status_t mag_capture_service_single_shot(bool read_temp, mag_capture_sample_t *sample_out)
{
    mmc5603_sample_t drv_sample;
    mag_status_t status;

    if (sample_out == NULL)
    {
        return MAG_STATUS_INVALID_PARAM;
    }
    if (!mag_capture_measurement_is_active())
    {
        return MAG_STATUS_NOT_READY;
    }

    status = mmc5603_take_measurement(read_temp, &drv_sample);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    mag_capture_fill_header(&drv_sample, sample_out);
    mag_capture_publish_sample(sample_out);
    return MAG_STATUS_OK;
}

mag_status_t mag_capture_service_read_latest(bool read_temp, mag_capture_sample_t *sample_out)
{
    mmc5603_sample_t drv_sample;
    mag_status_t status;

    if (sample_out == NULL)
    {
        return MAG_STATUS_INVALID_PARAM;
    }
    if (!mag_capture_measurement_is_active())
    {
        return MAG_STATUS_NOT_READY;
    }

    status = mmc5603_read_continuous_latest(read_temp, &drv_sample);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    mag_capture_fill_header(&drv_sample, sample_out);
    mag_capture_publish_sample(sample_out);
    return MAG_STATUS_OK;
}

mag_status_t mag_capture_service_read_fresh(bool read_temp,
                                            mag_capture_sample_t *sample_out,
                                            bool *fresh_out)
{
    mmc5603_sample_t drv_sample;
    mag_status_t status;

    if ((sample_out == NULL) || (fresh_out == NULL))
    {
        return MAG_STATUS_INVALID_PARAM;
    }
    *fresh_out = false;
    if (!mag_capture_measurement_is_active())
    {
        return MAG_STATUS_NOT_READY;
    }
    status = mmc5603_read_continuous_fresh(read_temp,
                                           &drv_sample,
                                           fresh_out);
    if ((status != MAG_STATUS_OK) || !*fresh_out)
    {
        return status;
    }

    mag_capture_fill_header(&drv_sample, sample_out);
    mag_capture_publish_sample(sample_out);
    return MAG_STATUS_OK;
}

void mag_capture_service_set_deferred_hook(mag_capture_deferred_hook_t hook, void *user_ctx)
{
    s_mag_deferred_hook = hook;
    s_mag_deferred_hook_ctx = user_ctx;
}

void mag_capture_service_poll_hook(void)
{
    if (s_mag_pending && (s_mag_deferred_hook != NULL))
    {
        s_mag_deferred_hook(&s_mag_pending_sample, s_mag_deferred_hook_ctx);
        s_mag_pending = false;
    }
}

bool mag_capture_service_try_dequeue(mag_capture_sample_t *sample_out)
{
    if ((sample_out == NULL) || !s_mag_pending)
    {
        return false;
    }

    *sample_out = s_mag_pending_sample;
    s_mag_pending = false;
    return true;
}
