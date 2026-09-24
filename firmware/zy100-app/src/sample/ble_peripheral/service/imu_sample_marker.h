#ifndef IMU_SAMPLE_MARKER_H
#define IMU_SAMPLE_MARKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../common/imu_common.h"

#define IMU_PRE_TRIGGER_MARKER_ENABLE                 1
#define IMU_PRE_TRIGGER_MARKER_SOURCE_GYRO_CANDIDATE  1
#define IMU_PRE_TRIGGER_MARKER_SOURCE_WOM_CANDIDATE   0

#ifndef IMU_SAMPLE_MARKER_RUNTIME_LOG_ENABLE
#define IMU_SAMPLE_MARKER_RUNTIME_LOG_ENABLE          IMU_MARKER_RUNTIME_LOG_ENABLE
#endif
#define IMU_SAMPLE_MARKER_RUNTIME_LOG_MAX_PER_CAPTURE 512U

#define IMU_SAMPLE_MARKER_MAX_COUNT                   256U
#define IMU_SAMPLE_MARKER_RECORD_BYTES                16U
#define IMU_SAMPLE_FLAG_PRE_SWING_TRIGGER             0x0001U

typedef enum
{
    IMU_MARKER_SOURCE_GYRO_CANDIDATE = 1,
    IMU_MARKER_SOURCE_WOM_CANDIDATE  = 2,
    IMU_MARKER_SOURCE_SOFTWARE       = 3,
} imu_sample_marker_source_t;

typedef struct __attribute__((packed))
{
    uint32_t sample_seq;
    uint8_t local_ts_us_le[8];
    uint16_t flags;
    uint16_t source;
} imu_sample_marker_t;

typedef char imu_sample_marker_size_check[
    (sizeof(imu_sample_marker_t) == IMU_SAMPLE_MARKER_RECORD_BYTES) ? 1 : -1];

typedef struct
{
    uint32_t sample_seq;
    uint64_t local_ts_us;
    uint16_t flags;
    uint16_t source;
} imu_sample_marker_entry_t;

typedef struct
{
    uint32_t marker_count;
    uint32_t marker_dropped_count;
    uint32_t pre_swing_trigger_count;
    uint32_t suppressed_marker_count;
    uint32_t flash_marker_write_count;
    uint32_t flash_marker_write_error_count;
    uint32_t marker_table_addr;
    uint32_t marker_table_bytes;
    uint32_t marker_table_crc;
} imu_sample_marker_stats_t;

void imu_sample_marker_reset(void);
imu_status_t imu_sample_marker_append_pre_swing_trigger(uint32_t sample_seq,
                                                        uint64_t local_ts_us,
                                                        uint16_t source);
imu_status_t imu_sample_marker_append_pre_swing_trigger_with_reason(
    uint32_t sample_seq,
    uint64_t local_ts_us,
    uint16_t source,
    const char *reason);
void imu_sample_marker_note_suppressed(void);
void imu_sample_marker_note_flash_write_success(uint32_t table_addr,
                                                uint32_t table_bytes,
                                                uint32_t table_crc);
void imu_sample_marker_note_flash_write_error(uint32_t table_addr,
                                              uint32_t table_bytes,
                                              uint32_t table_crc);

uint32_t imu_sample_marker_count(void);
bool imu_sample_marker_get_record(uint32_t index, imu_sample_marker_t *out);
bool imu_sample_marker_get_entry(uint32_t index, imu_sample_marker_entry_t *out);
void imu_sample_marker_serialize_entry(const imu_sample_marker_entry_t *entry,
                                       imu_sample_marker_t *out);
void imu_sample_marker_deserialize_record(const imu_sample_marker_t *record,
                                          imu_sample_marker_entry_t *out);

void imu_sample_marker_get_stats(imu_sample_marker_stats_t *out);
const char *imu_sample_marker_source_name(uint16_t source);
void imu_sample_marker_log_config(void);
void imu_sample_marker_log_stats(void);
void imu_sample_marker_log_markers(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_SAMPLE_MARKER_H */
