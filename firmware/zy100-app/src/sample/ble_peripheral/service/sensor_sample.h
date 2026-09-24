#ifndef SENSOR_SAMPLE_H
#define SENSOR_SAMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    SENSOR_TYPE_IMU = 1,
    SENSOR_TYPE_MAG = 2,
} sensor_type_t;

typedef enum
{
    SENSOR_SOURCE_SINGLE = 0,
    SENSOR_SOURCE_CONTINUOUS = 1,
    SENSOR_SOURCE_POLLING = 2,
    SENSOR_SOURCE_INTERRUPT = 3,
    SENSOR_SOURCE_FIFO = 4,
} sensor_source_mode_t;

typedef struct
{
    uint8_t sensor_type;
    uint8_t source_mode;
    uint16_t status_flags;
    uint32_t sample_seq;
    uint64_t trig_ts;
    uint64_t read_ts;
} sensor_sample_header_t;

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAMPLE_H */
