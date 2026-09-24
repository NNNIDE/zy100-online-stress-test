#ifndef ZY100_CAPTURE_TIME_H
#define ZY100_CAPTURE_TIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define ZY100_CAPTURE_TIME_META_BYTES 32U

typedef enum
{
    ZY100_CAPTURE_TIME_SRC_BUTTON = 0U,
    ZY100_CAPTURE_TIME_SRC_BLE = 1U,
    ZY100_CAPTURE_TIME_SRC_AUTO = 2U,
} zy100_capture_time_src_t;

typedef char zy100_capture_time_meta_size_check[
    (ZY100_CAPTURE_TIME_META_BYTES == 32U) ? 1 : -1];

void zy100_capture_time_reset(void);
void zy100_capture_time_note_start(zy100_capture_time_src_t source,
                                   uint32_t user_id,
                                   uint32_t training_id,
                                   uint64_t cmd_time_ms);
void zy100_capture_time_set_stop_source(zy100_capture_time_src_t source,
                                        uint64_t cmd_time_ms);
void zy100_capture_time_note_stop(void);
void zy100_capture_time_build_meta(uint8_t out[ZY100_CAPTURE_TIME_META_BYTES]);
bool zy100_capture_time_decode_meta(
    const uint8_t meta[ZY100_CAPTURE_TIME_META_BYTES],
    uint64_t *start_ms,
    uint64_t *stop_ms,
    uint8_t *calibrated_start,
    uint8_t *calibrated_stop,
    uint8_t *time_source);
void zy100_capture_time_log_export_from_reserved(const uint32_t reserved[32]);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_CAPTURE_TIME_H */
