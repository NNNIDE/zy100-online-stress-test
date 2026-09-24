#ifndef EXTERNAL_FLASH_CAPTURE_H
#define EXTERNAL_FLASH_CAPTURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../app_flags.h"
#include "zy100_flash_common.h"

#define V1_FLASH_SIZE_BYTES          (4UL * 1024UL * 1024UL)
#define V1_FLASH_SECTOR_BYTES        4096U
#define V1_FLASH_PAGE_BYTES          256U
#define V1_SESSION_HEADER_ADDR       0x000000UL
#define V1_DATA_START_ADDR           0x001000UL
#define V1_FLASH_LIMIT_ADDR          0x3CC000UL
#define V1_MARKER_TABLE_RESERVED_BYTES (8U * 1024U)
#define V1_MARKER_TABLE_ADDR         (V1_FLASH_SIZE_BYTES - V1_MARKER_TABLE_RESERVED_BYTES)
#define V1_RAW_SAMPLE_LIMIT_ADDR     ((V1_FLASH_LIMIT_ADDR < V1_MARKER_TABLE_ADDR) ? \
                                      V1_FLASH_LIMIT_ADDR : V1_MARKER_TABLE_ADDR)
#define V1_DATA_BLOCK_BYTES          4096U
#define V1_IMU_PACKET_BYTES          16U
#define V1_BLOCK_HEADER_BYTES        32U
#define V1_PACKETS_PER_BLOCK         254U
#define V1_RAW_SAMPLE_CAPACITY_BYTES (V1_RAW_SAMPLE_LIMIT_ADDR - V1_DATA_START_ADDR)
#define V1_MAX_DATA_BLOCKS           (V1_RAW_SAMPLE_CAPACITY_BYTES / V1_DATA_BLOCK_BYTES)
#define V1_LED_ERASE_OK_OUT          3U
#define V1_LED_EXPORT_OUT            4U
#define V1_LED_CAPTURE_DONE_OUT      5U
#define V1_SESSION_METADATA_BYTES    V1_FLASH_PAGE_BYTES
#define V1_EXPORT_STOP_FLASH_95_PERCENT 2U
#define V1_EXPORT_RESULT_IMU_PASS    0x00000001UL
#define V1_EXPORT_RESULT_FLASH_PASS  0x00000002UL

typedef enum
{
    EXTERNAL_FLASH_CAPTURE_ERASE_BUSY = 0U,
    EXTERNAL_FLASH_CAPTURE_ERASE_DONE,
    EXTERNAL_FLASH_CAPTURE_ERASE_ERROR,
    EXTERNAL_FLASH_CAPTURE_ERASE_TIMEOUT,
} external_flash_capture_erase_result_t;

typedef enum
{
    V1_FLASH_OP_IDLE = 0U,
    V1_FLASH_OP_CAPTURE_WRITE,
    V1_FLASH_OP_FLUSH,
    V1_FLASH_OP_VERIFY_WRITTEN,
    V1_FLASH_OP_ERASE_WIP,
    V1_FLASH_OP_VERIFY_EMPTY,
    V1_FLASH_OP_EXPORT_READ,
    V1_FLASH_OP_MARKER_WRITE,
    V1_FLASH_OP_POST_EXPORT_ERASE,
} v1_flash_op_t;

typedef enum
{
    V1_FLASH_PUMP_SAFE = 0U,
    V1_FLASH_PUMP_BUSY,
    V1_FLASH_PUMP_ABORTED,
    V1_FLASH_PUMP_ERROR,
    V1_FLASH_PUMP_TIMEOUT,
} v1_flash_pump_status_t;

typedef enum
{
    EXTERNAL_FLASH_CAPTURE_PUMP_NO_WORK = 0U,
    EXTERNAL_FLASH_CAPTURE_PUMP_WIP_BUSY,
    EXTERNAL_FLASH_CAPTURE_PUMP_PAGE_ISSUED,
    EXTERNAL_FLASH_CAPTURE_PUMP_BLOCK_DONE,
    EXTERNAL_FLASH_CAPTURE_PUMP_ERROR,
} external_flash_capture_pump_result_t;

#define EXTERNAL_FLASH_CAPTURE_QUEUE_BLOCK_STATE_MAX 4U

typedef struct
{
    uint32_t queue_level;
    bool queue_near_full;
    uint32_t fill_packet_count;
    uint32_t current_fill_remaining_packets;
    uint32_t total_accept_packets;
    uint32_t free_block_count;
    uint32_t full_blocks_accepted;
    uint32_t write_count;
    uint32_t write_page_index;
    uint32_t flash_op;
    bool raw_queue_busy;
    bool normal_flash_wip;
    external_flash_capture_pump_result_t last_pump_result;
    uint8_t block_state[EXTERNAL_FLASH_CAPTURE_QUEUE_BLOCK_STATE_MAX];
} external_flash_capture_queue_status_t;

typedef zy100_flash_prepare_status_t v1_flash_prepare_status_t;

#define V1_FLASH_PREP_BUSY    ZY100_FLASH_PREP_BUSY
#define V1_FLASH_PREP_DONE    ZY100_FLASH_PREP_DONE
#define V1_FLASH_PREP_ABORTED ZY100_FLASH_PREP_ABORTED
#define V1_FLASH_PREP_ERROR   ZY100_FLASH_PREP_ERROR
#define V1_FLASH_PREP_TIMEOUT ZY100_FLASH_PREP_TIMEOUT

typedef enum
{
    V1_BLOCK_CHECK_OK = 0U,
    V1_BLOCK_CHECK_FLASH_READ_ERROR,
    V1_BLOCK_CHECK_BAD_MAGIC,
    V1_BLOCK_CHECK_BAD_VERSION,
    V1_BLOCK_CHECK_BAD_HEADER_SIZE,
    V1_BLOCK_CHECK_SEQ_MISMATCH,
    V1_BLOCK_CHECK_CRC_MISMATCH,
    V1_BLOCK_CHECK_BAD_PACKET_COUNT,
    V1_BLOCK_CHECK_BAD_FLAGS,
} v1_block_check_status_t;

typedef struct
{
    uint32_t packet_count;
    uint32_t bytes_read;
    uint32_t fifo_full_count;
    uint32_t bad_header_count;
    uint32_t ts_jump_count;
    uint32_t read_error_count;
    uint32_t max_loop_gap_ms;
    uint32_t loop_gap_over_10ms_count;
    uint32_t loop_gap_over_20ms_count;
    uint32_t queue_max_level;
    uint32_t max_fifo_count;
    uint16_t fifo_lost_pkt_count;
} external_flash_capture_imu_stats_t;

typedef struct
{
    uint8_t metadata_page[V1_SESSION_METADATA_BYTES];
    uint32_t metadata_addr;
    uint32_t round;
    uint32_t data_start_addr;
    uint32_t data_bytes;
    uint32_t block_count;
    uint32_t packet_count;
    uint32_t marker_count;
    uint32_t marker_bytes;
    uint32_t flash_bytes_written;
    uint32_t stop_reason;
    uint32_t result_flags;
    bool imu_pass;
    bool flash_pass;
    bool verify_complete;
} external_flash_capture_export_session_t;

typedef struct
{
    v1_block_check_status_t status;
    uint32_t block_seq;
    uint32_t addr;
    uint32_t expected_crc;
    uint32_t actual_crc;
} v1_block_check_result_t;

bool external_flash_capture_prepare(uint32_t round);
bool external_flash_capture_prepare_begin(uint32_t round);
v1_flash_prepare_status_t external_flash_capture_prepare_poll(void);
bool external_flash_capture_prepare_erase_only_begin(uint32_t round);
external_flash_capture_erase_result_t external_flash_capture_prepare_erase_only_poll(void);
bool external_flash_capture_prepare_erased(uint32_t round);
bool external_flash_capture_begin(void);
uint32_t external_flash_capture_accept_capacity_packets(void);
uint32_t external_flash_capture_final_tail_capacity_packets(uint32_t burst_packets);
bool external_flash_capture_append_packet(const uint8_t *packet);
void external_flash_capture_pump(void);
bool external_flash_capture_pump_once_bounded(void);
bool external_flash_capture_is_full(void);
bool external_flash_capture_has_error(void);
bool external_flash_capture_has_pending_work(void);
bool external_flash_capture_raw_queue_busy(void);
bool external_flash_capture_queue_near_full(void);
void external_flash_capture_get_queue_status(external_flash_capture_queue_status_t *status);
uint32_t external_flash_capture_stop_reason(void);
bool external_flash_capture_scan_complete_session(external_flash_capture_export_session_t *session);
bool external_flash_capture_get_cached_complete_session(external_flash_capture_export_session_t *session);
bool external_flash_capture_writer_is_idle(void);
bool external_flash_capture_flash_wip_clear(void);
bool external_flash_capture_read_export_chunk(uint32_t addr, uint8_t *buf, uint16_t len);
/*
 * Checked raw block read is read-only. It never starts capture, rewrites
 * markers, or erases Flash.
 */
v1_block_check_status_t external_flash_capture_read_checked_block(uint32_t block_seq,
                                                                  uint8_t *buf,
                                                                  v1_block_check_result_t *check);
bool external_flash_capture_get_marker_export_summary(uint32_t *marker_count,
                                                      uint32_t *marker_bytes);
bool external_flash_capture_marker_export_available(uint32_t *marker_bytes);
bool external_flash_capture_read_marker_export_chunk(uint32_t offset, uint8_t *buf, uint16_t len);
v1_flash_op_t external_flash_capture_get_op(void);
const char *external_flash_capture_op_name(v1_flash_op_t op);
bool external_flash_capture_is_busy(void);
bool external_flash_capture_is_safe_for_sleep(void);
bool external_flash_capture_can_abort_for_sleep(void);
void external_flash_capture_request_sleep_abort(void);
v1_flash_pump_status_t external_flash_capture_pump_for_sleep(void);
bool external_flash_capture_post_export_erase_begin(void);
external_flash_capture_erase_result_t external_flash_capture_post_export_erase_poll(void);
bool external_flash_capture_post_export_erase_is_active(void);
void external_flash_capture_finalize(uint32_t stop_reason,
                                     const external_flash_capture_imu_stats_t *imu_stats);
void external_flash_capture_led_mark_sleep(void);
bool external_flash_capture_led_green_on(void);
bool external_flash_capture_led_blue_on(void);
bool external_flash_capture_led_red_on(void);
bool external_flash_capture_led_all_off(void);
bool external_flash_capture_led_red_is_on_expected(void);

#ifdef __cplusplus
}
#endif

#endif
