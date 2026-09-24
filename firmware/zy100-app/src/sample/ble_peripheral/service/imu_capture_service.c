#include "imu_capture_service.h"

#include <stddef.h>

#include "../bsp/imu_bsp.h"

static volatile bool s_pending_irq = false;
static volatile uint32_t s_pending_event_id = 0U;
static volatile uint64_t s_pending_irq_ts_us = 0U;
static imu_capture_irq_notify_t s_irq_notify_cb = NULL;
static uint32_t s_imu_unified_seq = 0U;

imu_status_t imu_capture_service_init(void)
{
    s_pending_irq = false;
    s_pending_event_id = 0U;
    s_pending_irq_ts_us = 0U;
    s_irq_notify_cb = NULL;
    s_imu_unified_seq = 0U;
    imu_bsp_int_register_irq_callback(imu_capture_service_on_irq);
    return IMU_STATUS_OK;
}

void imu_capture_service_set_irq_notify(imu_capture_irq_notify_t cb)
{
    s_irq_notify_cb = cb;
}

void imu_capture_service_on_irq(void)
{
    s_pending_event_id++;
    s_pending_irq_ts_us = imu_bsp_local_timestamp_us();
    s_pending_irq = true;

    if (s_irq_notify_cb != NULL)
    {
        s_irq_notify_cb(s_pending_event_id, s_pending_irq_ts_us);
    }
}

bool imu_capture_service_has_pending_event(void)
{
    return s_pending_irq;
}

uint32_t imu_capture_service_get_pending_event_id(void)
{
    return s_pending_event_id;
}

uint64_t imu_capture_service_get_pending_irq_ts_us(void)
{
    return s_pending_irq_ts_us;
}

void imu_capture_service_clear_pending_event(void)
{
    s_pending_irq = false;
}

imu_status_t imu_capture_service_read_int_status(icm53611_int_status_t *status)
{
    if (status == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    return icm53611_read_int_status(status);
}

imu_status_t imu_capture_service_read_fifo_chunk(uint8_t *fifo_buf, uint16_t fifo_buf_len, uint16_t *read_len)
{
    imu_status_t status;
    uint16_t fifo_count = 0U;
    uint16_t bytes_to_read;

    if ((fifo_buf == NULL) || (fifo_buf_len == 0U) || (read_len == NULL))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = icm53611_fifo_get_count(&fifo_count);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    bytes_to_read = (fifo_count < fifo_buf_len) ? fifo_count : fifo_buf_len;
    if (bytes_to_read == 0U)
    {
        *read_len = 0U;
        return IMU_STATUS_OK;
    }

    status = icm53611_fifo_read(fifo_buf, bytes_to_read);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    *read_len = bytes_to_read;
    return IMU_STATUS_OK;
}

imu_status_t imu_capture_service_build_sample(const icm53611_raw_sample_t *raw,
                                              uint32_t event_id,
                                              uint32_t sample_index,
                                              imu_event_sample_t *sample_out)
{
    if ((raw == NULL) || (sample_out == NULL))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    sample_out->event_id = event_id;
    sample_out->irq_local_ts_us = s_pending_irq_ts_us;
    sample_out->fifo_sample_ts = 0U;
    sample_out->sample_index = sample_index;
    sample_out->accel_x = raw->accel_x;
    sample_out->accel_y = raw->accel_y;
    sample_out->accel_z = raw->accel_z;
    sample_out->gyro_x = raw->gyro_x;
    sample_out->gyro_y = raw->gyro_y;
    sample_out->gyro_z = raw->gyro_z;
    sample_out->fifo_header = 0U;
    sample_out->flags = 0U;

    return IMU_STATUS_OK;
}

imu_status_t imu_capture_service_build_unified_sample(const icm53611_raw_sample_t *raw,
                                                      uint32_t event_id,
                                                      imu_unified_sample_t *sample_out)
{
    if ((raw == NULL) || (sample_out == NULL))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    sample_out->header.sensor_type = SENSOR_TYPE_IMU;
    sample_out->header.source_mode = SENSOR_SOURCE_CONTINUOUS;
    sample_out->header.status_flags = 0U;
    sample_out->header.sample_seq = ++s_imu_unified_seq;
    sample_out->header.trig_ts = s_pending_irq_ts_us;
    sample_out->header.read_ts = raw->local_ts_us;

    sample_out->event_id = event_id;
    sample_out->irq_local_ts_us = s_pending_irq_ts_us;
    sample_out->accel_x = raw->accel_x;
    sample_out->accel_y = raw->accel_y;
    sample_out->accel_z = raw->accel_z;
    sample_out->gyro_x = raw->gyro_x;
    sample_out->gyro_y = raw->gyro_y;
    sample_out->gyro_z = raw->gyro_z;
    return IMU_STATUS_OK;
}
