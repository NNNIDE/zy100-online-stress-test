#ifndef IMU_CAPTURE_SERVICE_H
#define IMU_CAPTURE_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "../common/imu_common.h"
#include "../driver/icm53611_driver.h"
#include "sensor_sample.h"

typedef struct
{
    uint32_t event_id;
    uint64_t irq_local_ts_us;
    uint32_t fifo_sample_ts;
    uint32_t sample_index;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    uint8_t fifo_header;
    uint16_t flags;
} imu_event_sample_t;

typedef struct
{
    sensor_sample_header_t header;
    uint32_t event_id;
    uint64_t irq_local_ts_us;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} imu_unified_sample_t;

typedef struct
{
    uint32_t event_id;
    uint64_t irq_local_ts_us;
    uint32_t sample_count;
    imu_event_sample_t *samples;
} imu_event_window_t;

typedef void (*imu_capture_irq_notify_t)(uint32_t event_id, uint64_t irq_local_ts_us);

imu_status_t imu_capture_service_init(void);
void imu_capture_service_set_irq_notify(imu_capture_irq_notify_t cb);

/* ISR-side lightweight hook: only latch event id and timestamp. */
void imu_capture_service_on_irq(void);
bool imu_capture_service_has_pending_event(void);
uint32_t imu_capture_service_get_pending_event_id(void);
uint64_t imu_capture_service_get_pending_irq_ts_us(void);
void imu_capture_service_clear_pending_event(void);

imu_status_t imu_capture_service_read_int_status(icm53611_int_status_t *status);
imu_status_t imu_capture_service_read_fifo_chunk(uint8_t *fifo_buf, uint16_t fifo_buf_len, uint16_t *read_len);
imu_status_t imu_capture_service_build_sample(const icm53611_raw_sample_t *raw,
                                              uint32_t event_id,
                                              uint32_t sample_index,
                                              imu_event_sample_t *sample_out);
imu_status_t imu_capture_service_build_unified_sample(const icm53611_raw_sample_t *raw,
                                                      uint32_t event_id,
                                                      imu_unified_sample_t *sample_out);

#ifdef __cplusplus
}
#endif

#endif /* IMU_CAPTURE_SERVICE_H */
