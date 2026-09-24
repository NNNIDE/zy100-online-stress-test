#ifndef ZY100_FINAL_EDGE_RAW_FORMAT_H
#define ZY100_FINAL_EDGE_RAW_FORMAT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ZY100_FE_RAW_MAGIC 0x57524546UL
#define ZY100_FE_RAW_VERSION 1U
#define ZY100_FE_RAW_CONTINUOUS_VERSION 2U
#define ZY100_FE_RAW_IMU_MAG_VERSION 3U
#define ZY100_FE_RAW_STATUS_COMMITTED 1U

#define ZY100_FE_RAW_FLAG_EARLY_HIT      (1UL << 0)
#define ZY100_FE_RAW_FLAG_LF_PRE_PRESENT (1UL << 1)
#define ZY100_FE_RAW_FLAG_HF_COMPLETE    (1UL << 2)
#define ZY100_FE_RAW_FLAG_HWIN_WRAP      (1UL << 3)
#define ZY100_FE_RAW_FLAG_OIS20_USED     (1UL << 4)
#define ZY100_FE_RAW_FLAG_OIS_OD_WARN    (1UL << 5)
#define ZY100_FE_RAW_FLAG_CONTINUOUS_800HZ (1UL << 6)
#define ZY100_FE_RAW_FLAG_MAG_RAW_100HZ     (1UL << 7)

#define ZY100_FE_RAW_HEADER_PAGE_BYTES                  256U
#define ZY100_FE_RAW_CONTINUOUS_SAMPLE_HZ               800U
#define ZY100_FE_RAW_CONTINUOUS_PACKET_BYTES            16U
#define ZY100_FE_RAW_CONTINUOUS_PACKETS_PER_RECORD      990U
#define ZY100_FE_RAW_CONTINUOUS_SECTION_HEADER_BYTES    28U
#define ZY100_FE_RAW_CONTINUOUS_PAYLOAD_BYTES           15868U
#define ZY100_FE_RAW_CONTINUOUS_PAYLOAD_STORAGE_BYTES   15872U
#define ZY100_FE_RAW_CONTINUOUS_SOURCE_RECORD_BYTES     16128U
#define ZY100_FE_RAW_CONTINUOUS_ONLINE_RECORD_BYTES     16384U
#define ZY100_FE_RAW_CONTINUOUS_PAYLOAD_PAGES           62U
#define ZY100_FE_RAW_CONTINUOUS_SOURCE_RECORD_PAGES     63U

#define ZY100_FE_RAW_MAG_SAMPLE_HZ                       100U
#define ZY100_FE_RAW_MAG_SAMPLE_BYTES                    13U
#define ZY100_FE_RAW_MAG_RAW_BYTES                       9U
#define ZY100_FE_RAW_MAG_SAMPLES_PER_RECORD_MAX          135U
#define ZY100_FE_RAW_MAG_SECTION_HEADER_BYTES             36U
#define ZY100_FE_RAW_MAG_SECTION_FLAG_FIXED40_ELAPSED_US (1UL << 0)
#define ZY100_FE_RAW_MAG_SECTION_FLAG_MMC5603_RAW9        (1UL << 1)
#define ZY100_FE_RAW_MAG_SECTION_FLAG_UNCALIBRATED        (1UL << 2)
#define ZY100_FE_RAW_MAG_SECTION_FLAG_MEAS_M_DONE_GATED    (1UL << 3)
#define ZY100_FE_RAW_MAG_SECTION_FLAGS_REQUIRED \
    (ZY100_FE_RAW_MAG_SECTION_FLAG_FIXED40_ELAPSED_US | \
     ZY100_FE_RAW_MAG_SECTION_FLAG_MMC5603_RAW9 | \
     ZY100_FE_RAW_MAG_SECTION_FLAG_UNCALIBRATED)
#define ZY100_FE_RAW_MAG_SECTION_FLAGS_CURRENT \
    (ZY100_FE_RAW_MAG_SECTION_FLAGS_REQUIRED | \
     ZY100_FE_RAW_MAG_SECTION_FLAG_MEAS_M_DONE_GATED)

#define ZY100_FE_RAW_IMU_MAG_PAYLOAD_BYTES_MAX \
    (ZY100_FE_RAW_CONTINUOUS_SECTION_HEADER_BYTES + \
     (ZY100_FE_RAW_CONTINUOUS_PACKETS_PER_RECORD * \
      ZY100_FE_RAW_CONTINUOUS_PACKET_BYTES) + \
     ZY100_FE_RAW_MAG_SECTION_HEADER_BYTES + \
     (ZY100_FE_RAW_MAG_SAMPLES_PER_RECORD_MAX * \
      ZY100_FE_RAW_MAG_SAMPLE_BYTES))
#define ZY100_FE_RAW_IMU_MAG_PAYLOAD_STORAGE_BYTES_MAX 17664U
#define ZY100_FE_RAW_IMU_MAG_SOURCE_RECORD_BYTES_MAX   17920U
#define ZY100_FE_RAW_IMU_MAG_ONLINE_RECORD_BYTES_MAX   18176U
#define ZY100_FE_RAW_IMU_MAG_PAYLOAD_PAGES_MAX            69U
#define ZY100_FE_RAW_IMU_MAG_SOURCE_RECORD_PAGES_MAX       70U

typedef enum
{
    ZY100_FE_RAW_SECTION_LF_PRE_800HZ = 1U,
    ZY100_FE_RAW_SECTION_HF_HIT_6400HZ = 2U,
    ZY100_FE_RAW_SECTION_CONTINUOUS_800HZ = 3U,
    ZY100_FE_RAW_SECTION_MAG_RAW_100HZ = 4U,
} zy100_fe_raw_section_type_t;

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint16_t version;
    uint16_t header_bytes;

    uint32_t raw_id;
    uint32_t bscan_id;

    uint32_t rt_trigger_sample;
    uint16_t rt_trigger_tmst_raw;
    uint16_t hit_tmst_raw;

    uint32_t hit_seq;
    uint32_t hit_score;
    uint32_t hit_score20;

    uint32_t start_seq;
    uint32_t end_seq;
    uint32_t hf_frames;
    uint32_t lf_packets;

    uint16_t first_tmst_raw;
    uint16_t last_tmst_raw;

    uint32_t section_count;
    uint32_t payload_offset;
    uint32_t payload_bytes;
    uint32_t bucket_bytes;

    uint32_t flags;
    uint32_t window_xor;
    uint32_t payload_xor;

    uint32_t page_count;
    uint32_t status;
    uint32_t reserved[16];
} zy100_fe_high_raw_header_t;

typedef struct __attribute__((packed))
{
    uint16_t section_type;
    uint16_t header_bytes;
    uint32_t sample_hz;
    uint32_t packet_or_frame_bytes;
    uint32_t count;
    uint16_t first_tmst_raw;
    uint16_t last_tmst_raw;
    uint32_t payload_bytes;
    uint32_t flags;
} zy100_fe_raw_section_header_t;

typedef struct __attribute__((packed))
{
    zy100_fe_raw_section_header_t base;
    uint32_t read_error_count;
    uint32_t missed_deadline_count;
} zy100_fe_raw_mag_section_header_t;

typedef struct __attribute__((packed))
{
    uint32_t elapsed_us;
    uint8_t raw9[ZY100_FE_RAW_MAG_RAW_BYTES];
} zy100_fe_raw_mag_sample_t;

typedef char zy100_fe_raw_continuous_section_header_size_check[
    (sizeof(zy100_fe_raw_section_header_t) ==
     ZY100_FE_RAW_CONTINUOUS_SECTION_HEADER_BYTES) ? 1 : -1];
typedef char zy100_fe_raw_continuous_payload_size_check[
    ((ZY100_FE_RAW_CONTINUOUS_SECTION_HEADER_BYTES +
      (ZY100_FE_RAW_CONTINUOUS_PACKETS_PER_RECORD *
       ZY100_FE_RAW_CONTINUOUS_PACKET_BYTES)) ==
     ZY100_FE_RAW_CONTINUOUS_PAYLOAD_BYTES) ? 1 : -1];
typedef char zy100_fe_raw_continuous_payload_page_check[
    ((ZY100_FE_RAW_CONTINUOUS_PAYLOAD_PAGES *
      ZY100_FE_RAW_HEADER_PAGE_BYTES) ==
     ZY100_FE_RAW_CONTINUOUS_PAYLOAD_STORAGE_BYTES) ? 1 : -1];
typedef char zy100_fe_raw_continuous_source_record_size_check[
    ((ZY100_FE_RAW_HEADER_PAGE_BYTES +
      ZY100_FE_RAW_CONTINUOUS_PAYLOAD_STORAGE_BYTES) ==
     ZY100_FE_RAW_CONTINUOUS_SOURCE_RECORD_BYTES) ? 1 : -1];
typedef char zy100_fe_raw_continuous_source_record_page_check[
    ((ZY100_FE_RAW_CONTINUOUS_SOURCE_RECORD_PAGES *
      ZY100_FE_RAW_HEADER_PAGE_BYTES) ==
     ZY100_FE_RAW_CONTINUOUS_SOURCE_RECORD_BYTES) ? 1 : -1];
typedef char zy100_fe_raw_mag_section_header_size_check[
    (sizeof(zy100_fe_raw_mag_section_header_t) ==
     ZY100_FE_RAW_MAG_SECTION_HEADER_BYTES) ? 1 : -1];
typedef char zy100_fe_raw_mag_sample_size_check[
    (sizeof(zy100_fe_raw_mag_sample_t) ==
     ZY100_FE_RAW_MAG_SAMPLE_BYTES) ? 1 : -1];
typedef char zy100_fe_raw_imu_mag_payload_size_check[
    (ZY100_FE_RAW_IMU_MAG_PAYLOAD_BYTES_MAX == 17659U) ? 1 : -1];
typedef char zy100_fe_raw_imu_mag_payload_page_check[
    ((ZY100_FE_RAW_IMU_MAG_PAYLOAD_PAGES_MAX *
      ZY100_FE_RAW_HEADER_PAGE_BYTES) ==
     ZY100_FE_RAW_IMU_MAG_PAYLOAD_STORAGE_BYTES_MAX) ? 1 : -1];
typedef char zy100_fe_raw_imu_mag_source_record_size_check[
    ((ZY100_FE_RAW_HEADER_PAGE_BYTES +
      ZY100_FE_RAW_IMU_MAG_PAYLOAD_STORAGE_BYTES_MAX) ==
     ZY100_FE_RAW_IMU_MAG_SOURCE_RECORD_BYTES_MAX) ? 1 : -1];

#ifdef __cplusplus
}
#endif

#endif /* ZY100_FINAL_EDGE_RAW_FORMAT_H */
