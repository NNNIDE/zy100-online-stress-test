#include "external_flash_capture.h"

#include <stddef.h>
#include <string.h>

#include "os_sched.h"
#include "trace.h"
#include "rtl876x_gpio.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"

#include "../app_flags.h"
#if ZY100_PRODUCT_LOG_QUIET_ENABLE
#undef DBG_DIRECT
#define DBG_DIRECT(...) ZY100_LOG_VERBOSE(__VA_ARGS__)
#endif
#include "../bsp/imu_bsp.h"
#include "../bsp/imu_board_pinmap.h"
#include "../bsp/mag_bsp.h"
#include "../common/imu_common.h"
#include "../common/mag_common.h"
#include "../driver/gd25q32e_spi.h"
#include "../driver/spi_bus_owner.h"
#include "imu_rt_marker.h"
#include "imu_pre_trigger_marker.h"
#include "imu_sample_marker.h"
#include "svc_led_task.h"

#ifndef IMU_UNUSED
#define IMU_UNUSED(x) ((void)(x))
#endif

#if !ZY100_LEGACY_OFFLINE_ENABLE || ZY100_FINAL_EDGE_MODE_ENABLE

#define V1_LED_EXPECTED_NONE          0U

bool external_flash_capture_prepare(uint32_t round)
{
    IMU_UNUSED(round);
    return false;
}

bool external_flash_capture_prepare_begin(uint32_t round)
{
    IMU_UNUSED(round);
    return false;
}

v1_flash_prepare_status_t external_flash_capture_prepare_poll(void)
{
    return V1_FLASH_PREP_ABORTED;
}

bool external_flash_capture_prepare_erase_only_begin(uint32_t round)
{
    IMU_UNUSED(round);
    return false;
}

external_flash_capture_erase_result_t external_flash_capture_prepare_erase_only_poll(void)
{
    return EXTERNAL_FLASH_CAPTURE_ERASE_ERROR;
}

bool external_flash_capture_prepare_erased(uint32_t round)
{
    IMU_UNUSED(round);
    return false;
}

bool external_flash_capture_begin(void)
{
    return false;
}

uint32_t external_flash_capture_accept_capacity_packets(void)
{
    return 0U;
}

uint32_t external_flash_capture_final_tail_capacity_packets(uint32_t burst_packets)
{
    IMU_UNUSED(burst_packets);
    return 0U;
}

bool external_flash_capture_append_packet(const uint8_t *packet)
{
    IMU_UNUSED(packet);
    return false;
}

void external_flash_capture_pump(void)
{
}

bool external_flash_capture_pump_once_bounded(void)
{
    return false;
}

bool external_flash_capture_is_full(void)
{
    return false;
}

bool external_flash_capture_has_error(void)
{
    return false;
}

bool external_flash_capture_has_pending_work(void)
{
    return false;
}

bool external_flash_capture_raw_queue_busy(void)
{
    return false;
}

bool external_flash_capture_queue_near_full(void)
{
    return false;
}

void external_flash_capture_get_queue_status(external_flash_capture_queue_status_t *status)
{
    if (status != NULL)
    {
        memset(status, 0, sizeof(*status));
        status->flash_op = (uint32_t)V1_FLASH_OP_IDLE;
        status->last_pump_result = EXTERNAL_FLASH_CAPTURE_PUMP_NO_WORK;
    }
}

uint32_t external_flash_capture_stop_reason(void)
{
    return 0U;
}

bool external_flash_capture_scan_complete_session(external_flash_capture_export_session_t *session)
{
    IMU_UNUSED(session);
    return false;
}

bool external_flash_capture_get_cached_complete_session(external_flash_capture_export_session_t *session)
{
    IMU_UNUSED(session);
    return false;
}

bool external_flash_capture_writer_is_idle(void)
{
    return true;
}

bool external_flash_capture_flash_wip_clear(void)
{
    return true;
}

bool external_flash_capture_read_export_chunk(uint32_t addr, uint8_t *buf, uint16_t len)
{
    IMU_UNUSED(addr);
    IMU_UNUSED(buf);
    IMU_UNUSED(len);
    return false;
}

v1_block_check_status_t external_flash_capture_read_checked_block(uint32_t block_seq,
                                                                  uint8_t *buf,
                                                                  v1_block_check_result_t *check)
{
    IMU_UNUSED(block_seq);
    IMU_UNUSED(buf);
    if (check != NULL)
    {
        memset(check, 0, sizeof(*check));
        check->status = V1_BLOCK_CHECK_FLASH_READ_ERROR;
    }
    return V1_BLOCK_CHECK_FLASH_READ_ERROR;
}

bool external_flash_capture_get_marker_export_summary(uint32_t *marker_count,
                                                      uint32_t *marker_bytes)
{
    if (marker_count != NULL)
    {
        *marker_count = 0U;
    }
    if (marker_bytes != NULL)
    {
        *marker_bytes = 0U;
    }
    return false;
}

bool external_flash_capture_marker_export_available(uint32_t *marker_bytes)
{
    if (marker_bytes != NULL)
    {
        *marker_bytes = 0U;
    }
    return false;
}

bool external_flash_capture_read_marker_export_chunk(uint32_t offset, uint8_t *buf, uint16_t len)
{
    IMU_UNUSED(offset);
    IMU_UNUSED(buf);
    IMU_UNUSED(len);
    return false;
}

v1_flash_op_t external_flash_capture_get_op(void)
{
    return V1_FLASH_OP_IDLE;
}

const char *external_flash_capture_op_name(v1_flash_op_t op)
{
    switch (op)
    {
    case V1_FLASH_OP_IDLE:
        return "idle";
    case V1_FLASH_OP_CAPTURE_WRITE:
        return "capture_write";
    case V1_FLASH_OP_FLUSH:
        return "flush";
    case V1_FLASH_OP_VERIFY_WRITTEN:
        return "verify_written";
    case V1_FLASH_OP_ERASE_WIP:
        return "erase_wip";
    case V1_FLASH_OP_VERIFY_EMPTY:
        return "verify_empty";
    case V1_FLASH_OP_EXPORT_READ:
        return "export_read";
    case V1_FLASH_OP_MARKER_WRITE:
        return "marker_write";
    case V1_FLASH_OP_POST_EXPORT_ERASE:
        return "post_export_erase";
    default:
        return "unknown";
    }
}

bool external_flash_capture_is_busy(void)
{
    return false;
}

bool external_flash_capture_is_safe_for_sleep(void)
{
    return true;
}

bool external_flash_capture_can_abort_for_sleep(void)
{
    return false;
}

void external_flash_capture_request_sleep_abort(void)
{
}

v1_flash_pump_status_t external_flash_capture_pump_for_sleep(void)
{
    return V1_FLASH_PUMP_SAFE;
}

bool external_flash_capture_post_export_erase_begin(void)
{
    return false;
}

external_flash_capture_erase_result_t external_flash_capture_post_export_erase_poll(void)
{
    return EXTERNAL_FLASH_CAPTURE_ERASE_ERROR;
}

bool external_flash_capture_post_export_erase_is_active(void)
{
    return false;
}

void external_flash_capture_finalize(uint32_t stop_reason,
                                     const external_flash_capture_imu_stats_t *imu_stats)
{
    IMU_UNUSED(stop_reason);
    IMU_UNUSED(imu_stats);
}

void external_flash_capture_led_mark_sleep(void)
{
    svc_led_task_mark_sleep();
}

bool external_flash_capture_led_green_on(void)
{
    return svc_led_task_notify_green_on();
}

bool external_flash_capture_led_blue_on(void)
{
    return svc_led_task_notify_blue_on();
}

bool external_flash_capture_led_red_on(void)
{
    return svc_led_task_notify_red_on();
}

bool external_flash_capture_led_all_off(void)
{
    return svc_led_task_notify_off();
}

bool external_flash_capture_led_red_is_on_expected(void)
{
    return svc_led_task_red_is_on_expected();
}

#else

#define V1_BLOCK_QUEUE_DEPTH          4U
#define V1_QUEUE_NEAR_FULL_LEVEL      2U
#define V1_BLOCK_MAGIC                0x42554D49UL
#define V1_SESSION_MAGIC              0x53554D49UL
#define V1_MARKER_MAGIC_0             ((uint8_t)'M')
#define V1_MARKER_MAGIC_1             ((uint8_t)'R')
#define V1_MARKER_MAGIC_2             ((uint8_t)'K')
#define V1_MARKER_MAGIC_3             ((uint8_t)'1')
#define V1_VERSION                    1U
#define V1_BLOCK_FLAG_FINAL           0x00000001UL
#define V1_BLOCK_FLAG_PARTIAL         0x00000002UL
#define V1_SESSION_PREPARED           1U
#define V1_SESSION_RUNNING            2U
#define V1_SESSION_COMPLETE           3U
#define V1_SESSION_STOPPED            4U
#define V1_SESSION_FAILED             5U
#define V1_STOP_KEY_SLEEP             1U
#define V1_STOP_FLASH_95_PERCENT      2U
#define V1_STOP_FATAL                 3U
#define V1_LED_I2C_ADDR_7BIT          0x3CU
#define V1_LED_SDB_PIN                P2_6
#define V1_LED_REG_SHUTDOWN           0x00U
#define V1_LED_REG_PWM_BASE           0x0DU
#define V1_LED_REG_PWM_UPDATE         0x25U
#define V1_LED_REG_LED_CTRL_BASE      0x32U
#define V1_LED_REG_GLOBAL_CTRL        0x4AU
#define V1_LED_REG_PWM_FREQ           0x4BU
#define V1_LED_REG_RESET              0x4FU
#define V1_LED_CHANNEL_COUNT          12U
#define V1_LED_CTRL_OUT_ON            0x01U
#define V1_LED_PWM_FULL               0xFFU
#define V1_LED_OP_TIMEOUT_MS          1500U
#define V1_LED_I2C_BUSY_TIMEOUT_MS    5U
#define V1_LED_I2C_RETRY_COUNT        3U
#define V1_LED_STATUS_OK              0U
#define V1_LED_STATUS_DISABLED        1U
#define V1_LED_STATUS_PARAM           2U
#define V1_LED_STATUS_POWER           3U
#define V1_LED_STATUS_SDB             4U
#define V1_LED_STATUS_I2C_INIT        5U
#define V1_LED_STATUS_I2C_BUSY        6U
#define V1_LED_STATUS_I2C_WRITE       7U
#define V1_LED_STATUS_TIMEOUT         8U
#define V1_LED_STATUS_NOT_READY       9U
#define V1_VERIFY_BUF_BYTES           256U
#ifndef V1_FLASH_EMPTY_VERIFY_FULL
#ifdef V1_FLASH_VERIFY_EMPTY_FULL
#define V1_FLASH_EMPTY_VERIFY_FULL    V1_FLASH_VERIFY_EMPTY_FULL
#else
#define V1_FLASH_EMPTY_VERIFY_FULL    1U
#endif
#endif
#ifndef V1_FLASH_VERIFY_DEBUG
#define V1_FLASH_VERIFY_DEBUG         0U
#endif
#define V1_EMPTY_VERIFY_BUF_BYTES     1024U
#define V1_EMPTY_VERIFY_BUF_WORDS     (V1_EMPTY_VERIFY_BUF_BYTES / sizeof(uint32_t))
typedef char v1_empty_verify_buf_nonzero_check[(V1_EMPTY_VERIFY_BUF_BYTES != 0U) ? 1 : -1];
typedef char v1_empty_verify_buf_aligned_check[
    ((V1_EMPTY_VERIFY_BUF_BYTES % sizeof(uint32_t)) == 0U) ? 1 : -1];
typedef char v1_empty_verify_buf_sector_check[
    (V1_EMPTY_VERIFY_BUF_BYTES <= V1_FLASH_SECTOR_BYTES) ? 1 : -1];
#define V1_EMPTY_VERIFY_PROGRESS_BYTES (1024UL * 1024UL)
#define V1_ERASE_WIP_POLL_MS          50U
#define V1_ERASE_SLOW_LOG_PERIOD_MS   10000U
#define V1_CHIP_ERASE_HARD_TIMEOUT_MS (GD25Q32E_CHIP_ERASE_TIMEOUT_MS * 2U)
#define V1_PROGRESS_BLOCK_STEP        64U
#define V1_LED_EXPECTED_NONE          0U
#define V1_POST_ERASE_NONE            0U
#define V1_POST_ERASE_WIP             1U
#define V1_POST_ERASE_VERIFY          2U
#define V1_ERASE_REASON_WAKE          1U
#define V1_ERASE_REASON_POST_EXPORT   2U
#define V1_EMPTY_VERIFY_NONE          0U
#define V1_EMPTY_VERIFY_ACTIVE        1U
#define V1_PREP_NONE                  0U
#define V1_PREP_ERASE                 1U
#define V1_PREP_HEADER                2U
#define V1_CAPTURE_SAMPLE_HZ          1600U
#define V1_FLASH_WAIT_OP_PAGE_PROGRAM 1U
#define V1_FLASH_WAIT_OP_CHIP_ERASE   2U
#define V1_PAGES_PER_BLOCK            (V1_DATA_BLOCK_BYTES / V1_FLASH_PAGE_BYTES)
#define V1_FIFO_DRAIN_BURST_PACKETS   (ZY100_FIFO_WATERMARK_BYTES / V1_IMU_PACKET_BYTES)
#define V1_FINALIZE_WIP_WAIT_RETRIES  V1_PAGES_PER_BLOCK

static bool v1_flash_probe_jedec_with_recovery(void);

typedef struct __attribute__((packed))
{
    uint8_t magic[4];
    uint16_t version;
    uint16_t header_size;
    uint16_t record_size;
    uint16_t reserved0;
    uint32_t marker_count;
    uint32_t payload_bytes;
    uint32_t crc32;
    uint32_t flags;
    uint32_t reserved1;
} v1_marker_table_header_t;

typedef char v1_marker_table_header_size_check[
    (sizeof(v1_marker_table_header_t) == 32U) ? 1 : -1];
typedef char v1_marker_table_addr_aligned_check[
    ((V1_MARKER_TABLE_ADDR % V1_FLASH_SECTOR_BYTES) == 0U) ? 1 : -1];
typedef char v1_marker_table_capacity_check[
    ((sizeof(v1_marker_table_header_t) +
      (IMU_SAMPLE_MARKER_MAX_COUNT * IMU_SAMPLE_MARKER_RECORD_BYTES)) <=
     V1_MARKER_TABLE_RESERVED_BYTES) ? 1 : -1];
typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint16_t version;
    uint16_t header_size;
    uint32_t block_seq;
    uint32_t first_sample_index;
    uint32_t packet_count;
    uint32_t payload_crc32;
    uint32_t flags;
    uint32_t reserved0;
    uint8_t payload[V1_PACKETS_PER_BLOCK * V1_IMU_PACKET_BYTES];
} v1_flash_imu_block_t;

typedef char v1_flash_imu_block_size_check[(sizeof(v1_flash_imu_block_t) == V1_DATA_BLOCK_BYTES) ? 1 : -1];
typedef char v1_queue_depth_public_size_check[
    (V1_BLOCK_QUEUE_DEPTH == EXTERNAL_FLASH_CAPTURE_QUEUE_BLOCK_STATE_MAX) ? 1 : -1];

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint16_t version;
    uint16_t header_size;
    uint32_t status;
    uint32_t round;
    uint32_t flash_size;
    uint32_t data_start_addr;
    uint32_t limit_addr;
    uint32_t packet_size;
    uint32_t packets_per_block;
    uint32_t imu_odr_hz;
    uint32_t stop_reason;
    uint32_t packet_count;
    uint32_t block_count;
    uint32_t flash_bytes_written;
    uint32_t imu_lost;
    uint32_t imu_full;
    uint32_t imu_bad_header;
    uint32_t imu_ts_bad;
    uint32_t imu_read_error;
    uint32_t flash_write_error;
    uint32_t flash_wip_timeout;
    uint32_t flash_verify_error;
    uint32_t flash_seq_error;
    uint32_t flash_crc_error;
    uint32_t flash_queue_overflow;
    uint32_t result_flags;
    uint32_t blocks_verified;
    uint32_t flash_packets_verified;
    uint32_t reserved[36];
} v1_session_header_t;

typedef char v1_session_header_size_check[(sizeof(v1_session_header_t) <= V1_FLASH_PAGE_BYTES) ? 1 : -1];

typedef enum
{
    V1_BLOCK_FREE = 0U,
    V1_BLOCK_FILLING = 1U,
    V1_BLOCK_PENDING = 2U,
} v1_block_state_t;

typedef struct
{
    uint32_t flash_blocks_written;
    uint32_t flash_pages_written;
    uint32_t flash_bytes_written;
    uint32_t flash_write_error_count;
    uint32_t flash_wip_timeout_count;
    uint32_t flash_verify_error_count;
    uint32_t flash_seq_error_count;
    uint32_t flash_crc_error_count;
    uint32_t flash_queue_overflow_count;
    uint32_t flash_blocks_verified;
    uint32_t flash_packets_verified;
} v1_flash_stats_t;

typedef struct
{
    bool active;
    uint8_t phase;
    uint8_t reason;
    bool slow_warned;
    uint64_t start_ms;
    uint64_t last_poll_ms;
    uint64_t last_slow_log_ms;
} v1_post_export_erase_ctx_t;

typedef struct
{
    bool active;
    uint8_t phase;
    uint32_t addr;
    uint32_t verify_bytes;
    uint64_t start_ms;
} v1_empty_verify_ctx_t;

typedef struct
{
    bool active;
    uint8_t phase;
    uint32_t round;
} v1_prepare_ctx_t;

static v1_flash_imu_block_t s_v1_blocks[V1_BLOCK_QUEUE_DEPTH];
static v1_block_state_t s_v1_block_state[V1_BLOCK_QUEUE_DEPTH];
static uint8_t s_v1_pending_q[V1_BLOCK_QUEUE_DEPTH];
static uint8_t s_v1_q_head = 0U;
static uint8_t s_v1_q_tail = 0U;
static uint8_t s_v1_q_count = 0U;
static uint8_t s_v1_fill_index = 0xFFU;
static uint8_t s_v1_write_page_index = 0U;
static uint8_t s_v1_session_page = 0U;
static uint32_t s_v1_round = 0U;
static uint32_t s_v1_next_block_seq = 0U;
static uint32_t s_v1_full_blocks_accepted = 0U;
static uint32_t s_v1_packets_stored = 0U;
static uint32_t s_v1_fill_packet_count = 0U;
static uint32_t s_v1_stop_reason = 0U;
static bool s_v1_prepared = false;
static bool s_v1_running = false;
static bool s_v1_fatal = false;
static bool s_v1_full = false;
static bool s_v1_runtime_wip_polled_clear = false;
static bool s_v1_led_ready = false;
static uint32_t s_v1_led_error_last_ms = 0U;
static bool s_v1_cached_complete_valid = false;
static external_flash_capture_export_session_t s_v1_cached_complete;
static v1_post_export_erase_ctx_t s_v1_post_erase;
static v1_empty_verify_ctx_t s_v1_empty_verify;
static v1_prepare_ctx_t s_v1_prepare;
static v1_flash_stats_t s_v1_flash_stats;
static uint64_t s_v1_capture_start_ms = 0ULL;
static uint32_t s_v1_empty_verify_buf[V1_EMPTY_VERIFY_BUF_WORDS];
static v1_flash_imu_block_t s_v1_verify_block;
static volatile v1_flash_op_t s_v1_flash_op = V1_FLASH_OP_IDLE;
static volatile bool s_v1_sleep_abort_requested = false;
static bool s_v1_last_runtime_wip_busy = false;
#if ZY100_RUNTIME_STATS_LOG_ENABLE
static bool s_v1_cap_log_valid = false;
static uint32_t s_v1_cap_log_current_rem = 0U;
static uint32_t s_v1_cap_log_total_accept = 0U;
static uint32_t s_v1_cap_log_free_count = 0U;
static uint32_t s_v1_cap_log_fill = 0U;
#endif
static external_flash_capture_pump_result_t s_v1_last_pump_result =
    EXTERNAL_FLASH_CAPTURE_PUMP_NO_WORK;

static bool v1_write_session_header(uint32_t status_code,
                                    uint32_t stop_reason,
                                    const external_flash_capture_imu_stats_t *imu_stats);
static void v1_clear_writer_state(void);
static uint64_t v1_flash_runtime_ms(void);
static void v1_flash_set_op(v1_flash_op_t op);
static const char *v1_flash_wait_op_name(uint32_t op);
static bool v1_flash_wait_for_finalize_idle(const char *phase);
static void v1_empty_verify_abort(void);
#if ZY100_FIFO_MARKER_FLASH_ENABLE
static void v1_marker_write_table(void);
#endif
static bool v1_marker_export_summary(uint32_t *marker_count, uint32_t *marker_bytes);

static uint32_t v1_crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint8_t bit;

    for (i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static uint32_t v1_crc32(const uint8_t *data, uint32_t len)
{
    return v1_crc32_update(0xFFFFFFFFUL, data, len) ^ 0xFFFFFFFFUL;
}

static void v1_flash_set_op(v1_flash_op_t op)
{
    s_v1_flash_op = op;
}

static bool v1_raw_queue_busy(void)
{
    return (s_v1_q_count != 0U) ||
           (s_v1_write_page_index != 0U) ||
           (s_v1_flash_op == V1_FLASH_OP_CAPTURE_WRITE) ||
           (s_v1_flash_op == V1_FLASH_OP_FLUSH);
}

static bool v1_queue_near_full(void)
{
    return (s_v1_q_count >= V1_QUEUE_NEAR_FULL_LEVEL);
}

const char *external_flash_capture_op_name(v1_flash_op_t op)
{
    switch (op)
    {
    case V1_FLASH_OP_IDLE:
        return "idle";
    case V1_FLASH_OP_CAPTURE_WRITE:
        return "capture_write";
    case V1_FLASH_OP_FLUSH:
        return "flush";
    case V1_FLASH_OP_VERIFY_WRITTEN:
        return "verify_written";
    case V1_FLASH_OP_ERASE_WIP:
        return "erase_wip";
    case V1_FLASH_OP_VERIFY_EMPTY:
        return "verify_empty";
    case V1_FLASH_OP_EXPORT_READ:
        return "export_read";
    case V1_FLASH_OP_MARKER_WRITE:
        return "marker_write";
    case V1_FLASH_OP_POST_EXPORT_ERASE:
        return "post_export_erase";
    default:
        return "unknown";
    }
}

static const char *v1_flash_wait_op_name(uint32_t op)
{
    switch (op)
    {
    case V1_FLASH_WAIT_OP_PAGE_PROGRAM:
        return "page_program";
    case V1_FLASH_WAIT_OP_CHIP_ERASE:
        return "chip_erase";
    default:
        return "unknown";
    }
}

static void v1_reset_state(uint32_t round)
{
    uint8_t i;

    memset(s_v1_blocks, 0xFF, sizeof(s_v1_blocks));
    memset(s_v1_block_state, 0, sizeof(s_v1_block_state));
    memset(s_v1_pending_q, 0, sizeof(s_v1_pending_q));
    memset(&s_v1_flash_stats, 0, sizeof(s_v1_flash_stats));
    s_v1_q_head = 0U;
    s_v1_q_tail = 0U;
    s_v1_q_count = 0U;
    s_v1_fill_index = 0xFFU;
    s_v1_write_page_index = 0U;
    s_v1_session_page = 0U;
    s_v1_round = round;
    s_v1_next_block_seq = 0U;
    s_v1_full_blocks_accepted = 0U;
    s_v1_packets_stored = 0U;
    s_v1_fill_packet_count = 0U;
    s_v1_stop_reason = 0U;
    s_v1_prepared = false;
    s_v1_running = false;
    s_v1_fatal = false;
    s_v1_full = false;
    s_v1_led_ready = false;
    s_v1_cached_complete_valid = false;
    memset(&s_v1_cached_complete, 0, sizeof(s_v1_cached_complete));
    memset(&s_v1_post_erase, 0, sizeof(s_v1_post_erase));
    memset(&s_v1_empty_verify, 0, sizeof(s_v1_empty_verify));
    memset(&s_v1_prepare, 0, sizeof(s_v1_prepare));
    s_v1_capture_start_ms = 0ULL;
    s_v1_sleep_abort_requested = false;
    s_v1_last_runtime_wip_busy = false;
#if ZY100_RUNTIME_STATS_LOG_ENABLE
    s_v1_cap_log_valid = false;
    s_v1_cap_log_current_rem = 0U;
    s_v1_cap_log_total_accept = 0U;
    s_v1_cap_log_free_count = 0U;
    s_v1_cap_log_fill = 0U;
#endif
    s_v1_last_pump_result = EXTERNAL_FLASH_CAPTURE_PUMP_NO_WORK;
    v1_flash_set_op(V1_FLASH_OP_IDLE);

    for (i = 0U; i < V1_BLOCK_QUEUE_DEPTH; i++)
    {
        s_v1_block_state[i] = V1_BLOCK_FREE;
    }
}

static int8_t v1_alloc_block(void)
{
    uint8_t i;

    for (i = 0U; i < V1_BLOCK_QUEUE_DEPTH; i++)
    {
        if (s_v1_block_state[i] == V1_BLOCK_FREE)
        {
            s_v1_block_state[i] = V1_BLOCK_FILLING;
            memset(&s_v1_blocks[i], 0xFF, sizeof(s_v1_blocks[i]));
            return (int8_t)i;
        }
    }

    return -1;
}

static void v1_mark_fatal_queue(void)
{
    uint8_t st0 = (uint8_t)s_v1_block_state[0];
    uint8_t st1 = (uint8_t)s_v1_block_state[1];
    uint8_t st2 = (uint8_t)s_v1_block_state[2];
    uint8_t st3 = (uint8_t)s_v1_block_state[3];

    if (s_v1_flash_stats.flash_queue_overflow_count < 0xFFFFFFFFUL)
    {
        s_v1_flash_stats.flash_queue_overflow_count++;
    }
    s_v1_fatal = true;
    s_v1_stop_reason = V1_STOP_FATAL;
    s_v1_last_pump_result = EXTERNAL_FLASH_CAPTURE_PUMP_ERROR;
    DBG_DIRECT("[ERR][FLASH] queue_overflow count=%u q_count=%u fill_packets=%u full_blocks=%u block_state=%u,%u,%u,%u write_page_index=%u flash_op=%u flash_op_name=%s raw_queue_busy=%u normal_flash_wip=%u pump_result=%u write_count=%u bytes=%u",
               s_v1_flash_stats.flash_queue_overflow_count,
               (uint32_t)s_v1_q_count,
               s_v1_fill_packet_count,
               s_v1_full_blocks_accepted,
               (uint32_t)st0,
               (uint32_t)st1,
               (uint32_t)st2,
               (uint32_t)st3,
               (uint32_t)s_v1_write_page_index,
               (uint32_t)s_v1_flash_op,
               external_flash_capture_op_name(s_v1_flash_op),
               v1_raw_queue_busy() ? 1U : 0U,
               s_v1_last_runtime_wip_busy ? 1U : 0U,
               (uint32_t)s_v1_last_pump_result,
               s_v1_flash_stats.flash_blocks_written,
               s_v1_flash_stats.flash_bytes_written);
}

static void v1_mark_write_error(uint32_t addr)
{
    if (s_v1_flash_stats.flash_write_error_count < 0xFFFFFFFFUL)
    {
        s_v1_flash_stats.flash_write_error_count++;
    }
    s_v1_fatal = true;
    s_v1_stop_reason = V1_STOP_FATAL;
    DBG_DIRECT("[ERR][FLASH] write_failed addr=0x%06x count=%u",
               addr,
               s_v1_flash_stats.flash_write_error_count);
}

static void v1_mark_timeout(uint32_t op)
{
    if (s_v1_flash_stats.flash_wip_timeout_count < 0xFFFFFFFFUL)
    {
        s_v1_flash_stats.flash_wip_timeout_count++;
    }
    s_v1_fatal = true;
    s_v1_stop_reason = V1_STOP_FATAL;
    DBG_DIRECT("[ERR][FLASH] timeout op=%u op_name=%s current_op=%u current_op_name=%s q=%u page=%u written=%u accepted=%u bytes=%u",
               op,
               v1_flash_wait_op_name(op),
               (uint32_t)s_v1_flash_op,
               external_flash_capture_op_name(s_v1_flash_op),
               (uint32_t)s_v1_q_count,
               (uint32_t)s_v1_write_page_index,
               s_v1_flash_stats.flash_blocks_written,
               s_v1_full_blocks_accepted,
               s_v1_flash_stats.flash_bytes_written);
}

static void v1_fill_block_header(v1_flash_imu_block_t *block,
                                 uint32_t block_seq,
                                 uint32_t first_sample_index,
                                 uint32_t packet_count,
                                 uint32_t flags)
{
    block->magic = V1_BLOCK_MAGIC;
    block->version = V1_VERSION;
    block->header_size = V1_BLOCK_HEADER_BYTES;
    block->block_seq = block_seq;
    block->first_sample_index = first_sample_index;
    block->packet_count = packet_count;
    block->payload_crc32 = v1_crc32(block->payload, packet_count * V1_IMU_PACKET_BYTES);
    block->flags = flags;
    block->reserved0 = 0xFFFFFFFFUL;
}

static bool v1_enqueue_block(uint8_t block_index, uint32_t packet_count, uint32_t flags)
{
    v1_flash_imu_block_t *block;
    uint32_t first_sample_index;

    if ((block_index >= V1_BLOCK_QUEUE_DEPTH) ||
        (s_v1_block_state[block_index] != V1_BLOCK_FILLING) ||
        (packet_count == 0U) ||
        (packet_count > V1_PACKETS_PER_BLOCK) ||
        (s_v1_q_count >= V1_BLOCK_QUEUE_DEPTH))
    {
        v1_mark_fatal_queue();
        return false;
    }

    block = &s_v1_blocks[block_index];
    first_sample_index = s_v1_packets_stored - packet_count;
    v1_fill_block_header(block, s_v1_next_block_seq, first_sample_index, packet_count, flags);
    s_v1_next_block_seq++;
    s_v1_block_state[block_index] = V1_BLOCK_PENDING;
    s_v1_pending_q[s_v1_q_tail] = block_index;
    s_v1_q_tail = (uint8_t)((s_v1_q_tail + 1U) % V1_BLOCK_QUEUE_DEPTH);
    s_v1_q_count++;
    return true;
}

static bool v1_prepare_new_fill_block(void)
{
    int8_t idx;

    idx = v1_alloc_block();
    if (idx < 0)
    {
        v1_mark_fatal_queue();
        return false;
    }

    s_v1_fill_index = (uint8_t)idx;
    s_v1_fill_packet_count = 0U;
    return true;
}

static bool v1_fill_block_active(void)
{
    return (s_v1_fill_index < V1_BLOCK_QUEUE_DEPTH) &&
           (s_v1_block_state[s_v1_fill_index] == V1_BLOCK_FILLING) &&
           (s_v1_fill_packet_count < V1_PACKETS_PER_BLOCK);
}

static uint32_t v1_count_free_blocks(void)
{
    uint8_t i;
    uint32_t free_count = 0U;

    for (i = 0U; i < V1_BLOCK_QUEUE_DEPTH; i++)
    {
        if (s_v1_block_state[i] == V1_BLOCK_FREE)
        {
            free_count++;
        }
    }

    return free_count;
}

static uint32_t v1_remaining_data_capacity_packets(bool active_fill)
{
    uint32_t remaining_blocks;
    uint32_t remaining_packets;

    if (s_v1_full_blocks_accepted >= V1_MAX_DATA_BLOCKS)
    {
        return 0U;
    }

    remaining_blocks = V1_MAX_DATA_BLOCKS - s_v1_full_blocks_accepted;
    remaining_packets = remaining_blocks * V1_PACKETS_PER_BLOCK;
    if (active_fill)
    {
        if (s_v1_fill_packet_count >= remaining_packets)
        {
            return 0U;
        }
        remaining_packets -= s_v1_fill_packet_count;
    }

    return remaining_packets;
}

static uint32_t v1_total_accept_capacity_packets(uint32_t *current_rem,
                                                 uint32_t *free_count)
{
    bool active_fill = v1_fill_block_active();
    uint32_t active_remaining = 0U;
    uint32_t free_blocks;
    uint32_t total_accept;
    uint32_t max_accept;

    if (active_fill)
    {
        active_remaining = V1_PACKETS_PER_BLOCK - s_v1_fill_packet_count;
    }
    free_blocks = v1_count_free_blocks();
    total_accept = active_remaining + (free_blocks * V1_PACKETS_PER_BLOCK);
    max_accept = v1_remaining_data_capacity_packets(active_fill);
    if (total_accept > max_accept)
    {
        total_accept = max_accept;
    }

    if (current_rem != NULL)
    {
        *current_rem = active_remaining;
    }
    if (free_count != NULL)
    {
        *free_count = free_blocks;
    }

    return total_accept;
}

static void v1_log_accept_capacity_if_needed(uint32_t current_rem,
                                             uint32_t total_accept,
                                             uint32_t free_count)
{
#if !ZY100_RUNTIME_STATS_LOG_ENABLE || !ZY100_LOG_FLASH_VERBOSE
    IMU_UNUSED(current_rem);
    IMU_UNUSED(total_accept);
    IMU_UNUSED(free_count);
#else
    bool event;

    event = ((current_rem < V1_FIFO_DRAIN_BURST_PACKETS) &&
             (total_accept >= V1_FIFO_DRAIN_BURST_PACKETS)) ||
            (total_accept < V1_FIFO_DRAIN_BURST_PACKETS);

#if ZY100_FLASH_Q_VERBOSE_ENABLE
    IMU_UNUSED(event);
    ZY100_LOG_FLASH_DETAIL("[FLASH_CAP] current_rem=%u total_accept=%u q_free=%u fill=%u",
                           current_rem,
                           total_accept,
                           free_count,
                           s_v1_fill_packet_count);
    s_v1_cap_log_valid = true;
    s_v1_cap_log_current_rem = current_rem;
    s_v1_cap_log_total_accept = total_accept;
    s_v1_cap_log_free_count = free_count;
    s_v1_cap_log_fill = s_v1_fill_packet_count;
#else
    if (!event)
    {
        return;
    }
    if (s_v1_cap_log_valid &&
        (s_v1_cap_log_current_rem == current_rem) &&
        (s_v1_cap_log_total_accept == total_accept) &&
        (s_v1_cap_log_free_count == free_count) &&
        (s_v1_cap_log_fill == s_v1_fill_packet_count))
    {
        return;
    }

    ZY100_LOG_FLASH_DETAIL("[FLASH_CAP] current_rem=%u total_accept=%u q_free=%u fill=%u",
                           current_rem,
                           total_accept,
                           free_count,
                           s_v1_fill_packet_count);
    s_v1_cap_log_valid = true;
    s_v1_cap_log_current_rem = current_rem;
    s_v1_cap_log_total_accept = total_accept;
    s_v1_cap_log_free_count = free_count;
    s_v1_cap_log_fill = s_v1_fill_packet_count;
#endif
#endif
}

static uint32_t v1_led_now_ms(void)
{
    return (uint32_t)(imu_bsp_local_timestamp_us() / 1000ULL);
}

static const char *v1_led_color_from_tag(const char *tag)
{
    if (tag == NULL)
    {
        return "unknown";
    }
    if (strcmp(tag, "led_green") == 0)
    {
        return "green";
    }
    if (strcmp(tag, "led_blue") == 0)
    {
        return "blue";
    }
    if (strcmp(tag, "led_red") == 0)
    {
        return "red";
    }
    return tag;
}

static void v1_led_log_error(const char *color, uint32_t status)
{
    uint32_t now_ms = v1_led_now_ms();

    if ((s_v1_led_error_last_ms != 0U) &&
        ((uint32_t)(now_ms - s_v1_led_error_last_ms) < 1000U))
    {
        return;
    }
    s_v1_led_error_last_ms = now_ms;
#if ZY100_LED_LOG_ENABLE
    DBG_DIRECT("[LED_ERR] color=%s status=%u",
               (color != NULL) ? color : "unknown",
               status);
#endif
}

static bool v1_led_deadline_expired(uint32_t start_ms)
{
    return ((uint32_t)(v1_led_now_ms() - start_ms) >= V1_LED_OP_TIMEOUT_MS);
}

static uint32_t v1_led_wait_i2c_idle(uint32_t start_ms)
{
    uint32_t busy_start_ms = v1_led_now_ms();

    while (mag_bsp_i2c_is_bus_busy())
    {
        if (v1_led_deadline_expired(start_ms))
        {
            return V1_LED_STATUS_TIMEOUT;
        }
        if ((uint32_t)(v1_led_now_ms() - busy_start_ms) >= V1_LED_I2C_BUSY_TIMEOUT_MS)
        {
            return V1_LED_STATUS_I2C_BUSY;
        }
        os_delay(1U);
    }

    return V1_LED_STATUS_OK;
}

static uint32_t v1_led_i2c_write_regs(uint8_t reg_addr,
                                      const uint8_t *data,
                                      uint16_t len,
                                      uint32_t start_ms)
{
    uint8_t payload[1U + V1_LED_CHANNEL_COUNT];
    uint16_t i;
    uint8_t retry;
    uint32_t led_status = V1_LED_STATUS_I2C_WRITE;
    mag_status_t status;

    if ((data == NULL) || (len == 0U) || (len > V1_LED_CHANNEL_COUNT))
    {
        return V1_LED_STATUS_PARAM;
    }

    payload[0] = reg_addr;
    for (i = 0U; i < len; i++)
    {
        payload[i + 1U] = data[i];
    }

    for (retry = 0U; retry < V1_LED_I2C_RETRY_COUNT; retry++)
    {
        if (v1_led_deadline_expired(start_ms))
        {
            return V1_LED_STATUS_TIMEOUT;
        }

        status = mag_bsp_i2c_init(V1_LED_I2C_ADDR_7BIT);
        if (status != MAG_STATUS_OK)
        {
            led_status = V1_LED_STATUS_I2C_INIT;
            os_delay(1U);
            continue;
        }

        led_status = v1_led_wait_i2c_idle(start_ms);
        if (led_status != V1_LED_STATUS_OK)
        {
            return led_status;
        }

        status = mag_bsp_i2c_master_write(V1_LED_I2C_ADDR_7BIT, payload, (uint16_t)(len + 1U));
        if (status == MAG_STATUS_OK)
        {
            return V1_LED_STATUS_OK;
        }
        led_status = (status == MAG_STATUS_TIMEOUT) ? V1_LED_STATUS_TIMEOUT : V1_LED_STATUS_I2C_WRITE;
        os_delay(1U);
    }

    return led_status;
}

static uint32_t v1_led_write_reg(uint8_t reg_addr, uint8_t value, uint32_t start_ms)
{
    return v1_led_i2c_write_regs(reg_addr, &value, 1U, start_ms);
}

static uint32_t v1_led_enable_sdb(void)
{
    GPIO_InitTypeDef gpio_init;
    const uint32_t gpio_pin = GPIO_GetPin(V1_LED_SDB_PIN);

    if (gpio_pin == 0U)
    {
        return V1_LED_STATUS_SDB;
    }

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    Pad_Config(V1_LED_SDB_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_HIGH);
    Pinmux_Deinit(V1_LED_SDB_PIN);
    Pinmux_Config(V1_LED_SDB_PIN, DWGPIO);

    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = gpio_pin;
    gpio_init.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init.GPIO_ITCmd = DISABLE;
    gpio_init.GPIO_ControlMode = GPIO_SOFTWARE_MODE;
    GPIO_Init(&gpio_init);
    GPIO_SetBits(gpio_pin);
    return V1_LED_STATUS_OK;
}

static uint32_t v1_led_resume_for_access(uint32_t start_ms)
{
    uint32_t led_status;
    imu_status_t status;

    s_v1_led_ready = false;

    status = imu_bsp_power_ctrl(true);
    if ((status != IMU_STATUS_OK) && (status != IMU_STATUS_UNSUPPORTED))
    {
        return V1_LED_STATUS_POWER;
    }

    led_status = v1_led_enable_sdb();
    if (led_status != V1_LED_STATUS_OK)
    {
        return led_status;
    }

    os_delay(1U);
    if (v1_led_deadline_expired(start_ms))
    {
        return V1_LED_STATUS_TIMEOUT;
    }

    if (mag_bsp_i2c_force_reinit(V1_LED_I2C_ADDR_7BIT) != MAG_STATUS_OK)
    {
        return V1_LED_STATUS_I2C_INIT;
    }

    led_status = v1_led_write_reg(V1_LED_REG_RESET, 0x00U, start_ms);
    if (led_status != V1_LED_STATUS_OK)
    {
        return led_status;
    }
    led_status = v1_led_write_reg(V1_LED_REG_SHUTDOWN, 0x01U, start_ms);
    if (led_status != V1_LED_STATUS_OK)
    {
        return led_status;
    }
    led_status = v1_led_write_reg(V1_LED_REG_GLOBAL_CTRL, 0x00U, start_ms);
    if (led_status != V1_LED_STATUS_OK)
    {
        return led_status;
    }
    led_status = v1_led_write_reg(V1_LED_REG_PWM_FREQ, 0x00U, start_ms);
    if (led_status == V1_LED_STATUS_OK)
    {
        s_v1_led_ready = true;
    }
    return led_status;
}

static uint32_t v1_led_apply_single(uint8_t out, uint8_t level, uint32_t start_ms)
{
    uint8_t pwm[V1_LED_CHANNEL_COUNT] = {0U};
    uint8_t ctrl[V1_LED_CHANNEL_COUNT] = {0U};
    uint32_t led_status;

    if ((out == 0U) || (out > V1_LED_CHANNEL_COUNT))
    {
        return V1_LED_STATUS_PARAM;
    }
    if (!s_v1_led_ready)
    {
        return V1_LED_STATUS_NOT_READY;
    }

    if (level != 0U)
    {
        pwm[out - 1U] = level;
        ctrl[out - 1U] = V1_LED_CTRL_OUT_ON;
    }

    led_status = v1_led_i2c_write_regs(V1_LED_REG_PWM_BASE, pwm, V1_LED_CHANNEL_COUNT, start_ms);
    if (led_status != V1_LED_STATUS_OK)
    {
        return led_status;
    }
    led_status = v1_led_i2c_write_regs(V1_LED_REG_LED_CTRL_BASE, ctrl, V1_LED_CHANNEL_COUNT, start_ms);
    if (led_status != V1_LED_STATUS_OK)
    {
        return led_status;
    }
    return v1_led_write_reg(V1_LED_REG_PWM_UPDATE, 0x00U, start_ms);
}

static uint32_t v1_led_apply_all_off(uint32_t start_ms)
{
    uint8_t pwm[V1_LED_CHANNEL_COUNT] = {0U};
    uint8_t ctrl[V1_LED_CHANNEL_COUNT] = {0U};
    uint32_t led_status;

    if (!s_v1_led_ready)
    {
        return V1_LED_STATUS_NOT_READY;
    }

    led_status = v1_led_i2c_write_regs(V1_LED_REG_PWM_BASE, pwm, V1_LED_CHANNEL_COUNT, start_ms);
    if (led_status != V1_LED_STATUS_OK)
    {
        return led_status;
    }
    led_status = v1_led_i2c_write_regs(V1_LED_REG_LED_CTRL_BASE, ctrl, V1_LED_CHANNEL_COUNT, start_ms);
    if (led_status != V1_LED_STATUS_OK)
    {
        return led_status;
    }

    return v1_led_write_reg(V1_LED_REG_PWM_UPDATE, 0x00U, start_ms);
}

static bool v1_led_solid(uint8_t out, const char *tag)
{
    uint32_t start_ms = v1_led_now_ms();
    uint32_t led_status;

    led_status = v1_led_resume_for_access(start_ms);
    if (led_status == V1_LED_STATUS_OK)
    {
        led_status = v1_led_apply_single(out, V1_LED_PWM_FULL, start_ms);
    }

    if (led_status != V1_LED_STATUS_OK)
    {
        v1_led_log_error(v1_led_color_from_tag(tag), led_status);
        return false;
    }

    return true;
}

bool external_flash_capture_led_green_on(void)
{
    return svc_led_task_notify_green_on();
}

bool external_flash_capture_led_blue_on(void)
{
    return svc_led_task_notify_blue_on();
}

bool external_flash_capture_led_red_on(void)
{
    return svc_led_task_notify_red_on();
}

bool external_flash_capture_led_all_off(void)
{
    return svc_led_task_notify_off();
}

bool external_flash_capture_led_red_is_on_expected(void)
{
    return svc_led_task_red_is_on_expected();
}

static bool v1_flash_wait(uint32_t timeout_ms, uint32_t op)
{
    imu_status_t status;

    status = gd25q32e_wait_while_busy(timeout_ms);
    if (status == IMU_STATUS_OK)
    {
        return true;
    }

    if (status == IMU_STATUS_TIMEOUT)
    {
        v1_mark_timeout(op);
    }
    else
    {
        v1_mark_write_error(0U);
    }
    return false;
}

static bool v1_flash_wait_for_finalize_idle(const char *phase)
{
    uint32_t retry;
    imu_status_t status = IMU_STATUS_TIMEOUT;

    for (retry = 0U; retry < V1_FINALIZE_WIP_WAIT_RETRIES; retry++)
    {
        status = gd25q32e_wait_while_busy(GD25Q32E_PAGE_PROGRAM_TIMEOUT_MS);
        if (status == IMU_STATUS_OK)
        {
            return true;
        }
        if (status != IMU_STATUS_TIMEOUT)
        {
            v1_mark_write_error(0U);
            return false;
        }

        DBG_DIRECT("[WARN][FLASH] finalize_wip_retry phase=%s retry=%u/%u current_op=%u current_op_name=%s q=%u page=%u written=%u accepted=%u bytes=%u",
                   phase,
                   retry + 1U,
                   V1_FINALIZE_WIP_WAIT_RETRIES,
                   (uint32_t)s_v1_flash_op,
                   external_flash_capture_op_name(s_v1_flash_op),
                   (uint32_t)s_v1_q_count,
                   (uint32_t)s_v1_write_page_index,
                   s_v1_flash_stats.flash_blocks_written,
                   s_v1_full_blocks_accepted,
                   s_v1_flash_stats.flash_bytes_written);
    }

    v1_mark_timeout(V1_FLASH_WAIT_OP_PAGE_PROGRAM);
    return false;
}

static uint32_t v1_flash_elapsed_ms_u32(uint64_t start_ms)
{
    const uint64_t elapsed_ms = v1_flash_runtime_ms() - start_ms;

    return (elapsed_ms > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)elapsed_ms;
}

#if IMU_CAPTURE_PROGRESS_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE
static uint32_t v1_flash_speed_kbps(uint32_t bytes, uint32_t elapsed_ms)
{
    uint64_t speed;

    if (elapsed_ms == 0U)
    {
        return 0U;
    }

    speed = (((uint64_t)bytes) * 1000ULL) / (uint64_t)elapsed_ms;
    speed /= 1024ULL;
    return (speed > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)speed;
}
#endif

static void v1_flash_verify_empty_fail(uint32_t addr, uint8_t value)
{
    s_v1_flash_stats.flash_verify_error_count++;
    DBG_DIRECT("[ERR][FLASH] verify_empty addr=0x%06x val=%02x", addr, value);
}

static bool v1_flash_verify_empty_words(uint32_t base_addr, uint16_t len)
{
    const uint8_t *bytes = (const uint8_t *)s_v1_empty_verify_buf;
    const uint16_t word_count = (uint16_t)(len / sizeof(uint32_t));
    const uint16_t tail_start = (uint16_t)(word_count * sizeof(uint32_t));
    uint16_t word_index;
    uint16_t byte_index;

    for (word_index = 0U; word_index < word_count; word_index++)
    {
        if (s_v1_empty_verify_buf[word_index] != 0xFFFFFFFFUL)
        {
            const uint32_t word_addr = base_addr + ((uint32_t)word_index * sizeof(uint32_t));
            const uint16_t word_byte = (uint16_t)(word_index * sizeof(uint32_t));

            for (byte_index = 0U; byte_index < sizeof(uint32_t); byte_index++)
            {
                if (bytes[word_byte + byte_index] != 0xFFU)
                {
                    v1_flash_verify_empty_fail(word_addr + byte_index, bytes[word_byte + byte_index]);
                    return false;
                }
            }

            v1_flash_verify_empty_fail(word_addr, bytes[word_byte]);
            return false;
        }
    }

    for (byte_index = tail_start; byte_index < len; byte_index++)
    {
        if (bytes[byte_index] != 0xFFU)
        {
            v1_flash_verify_empty_fail(base_addr + byte_index, bytes[byte_index]);
            return false;
        }
    }

    return true;
}

static void v1_empty_verify_abort(void)
{
    memset(&s_v1_empty_verify, 0, sizeof(s_v1_empty_verify));
    if ((s_v1_flash_op == V1_FLASH_OP_VERIFY_EMPTY) ||
        (s_v1_flash_op == V1_FLASH_OP_ERASE_WIP) ||
        (s_v1_flash_op == V1_FLASH_OP_POST_EXPORT_ERASE))
    {
        v1_flash_set_op(V1_FLASH_OP_IDLE);
    }
}

static void v1_flash_verify_empty_begin(void)
{
    memset(&s_v1_empty_verify, 0, sizeof(s_v1_empty_verify));
    s_v1_empty_verify.active = true;
    s_v1_empty_verify.phase = V1_EMPTY_VERIFY_ACTIVE;
    s_v1_empty_verify.start_ms = v1_flash_runtime_ms();
    s_v1_empty_verify.addr = 0U;
#if V1_FLASH_EMPTY_VERIFY_FULL
    s_v1_empty_verify.verify_bytes = V1_FLASH_SIZE_BYTES;
#else
    s_v1_empty_verify.verify_bytes = V1_FLASH_SECTOR_BYTES;
#endif
    v1_flash_set_op(V1_FLASH_OP_VERIFY_EMPTY);
#if (IMU_CAPTURE_PROGRESS_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE) && ZY100_LOG_FLASH_VERBOSE
    DBG_DIRECT("[V1_F] verify_empty_begin");
#endif
}

static v1_flash_pump_status_t v1_flash_verify_empty_poll(bool sleep_abort_allowed)
{
    uint8_t *buf = (uint8_t *)s_v1_empty_verify_buf;
    uint32_t addr;
    uint32_t elapsed_ms;
    uint16_t read_len;
    imu_status_t status;

    if (!s_v1_empty_verify.active)
    {
        return V1_FLASH_PUMP_SAFE;
    }

    if (sleep_abort_allowed && s_v1_sleep_abort_requested)
    {
        v1_empty_verify_abort();
        return V1_FLASH_PUMP_ABORTED;
    }

#if V1_FLASH_EMPTY_VERIFY_FULL
    addr = s_v1_empty_verify.addr;
    if (addr < V1_FLASH_SIZE_BYTES)
    {
        read_len = (uint16_t)(((V1_FLASH_SIZE_BYTES - addr) > V1_EMPTY_VERIFY_BUF_BYTES) ?
                              V1_EMPTY_VERIFY_BUF_BYTES :
                              (V1_FLASH_SIZE_BYTES - addr));
#if V1_FLASH_VERIFY_DEBUG && ZY100_LOG_FLASH_VERBOSE
        if ((addr != 0U) && ((addr % V1_EMPTY_VERIFY_PROGRESS_BYTES) == 0U))
        {
            DBG_DIRECT("[V1_F] verify_prog addr=0x%06x", addr);
        }
#endif

        status = gd25q32e_read_fast(addr, buf, read_len);
        if (status != IMU_STATUS_OK)
        {
            v1_flash_verify_empty_fail(addr, 0U);
            v1_empty_verify_abort();
            return V1_FLASH_PUMP_ERROR;
        }

        if (!v1_flash_verify_empty_words(addr, read_len))
        {
            v1_empty_verify_abort();
            return V1_FLASH_PUMP_ERROR;
        }
        s_v1_empty_verify.addr += read_len;
        if (s_v1_empty_verify.addr < V1_FLASH_SIZE_BYTES)
        {
            return V1_FLASH_PUMP_BUSY;
        }
    }
#else
    addr = s_v1_empty_verify.addr;
    if (addr < V1_FLASH_SECTOR_BYTES)
    {
        const uint32_t verify_addr = V1_SESSION_HEADER_ADDR + addr;

        read_len = (uint16_t)(((V1_FLASH_SECTOR_BYTES - addr) > V1_EMPTY_VERIFY_BUF_BYTES) ?
                              V1_EMPTY_VERIFY_BUF_BYTES :
                              (V1_FLASH_SECTOR_BYTES - addr));

        status = gd25q32e_read(verify_addr, buf, read_len);
        if (status != IMU_STATUS_OK)
        {
            v1_flash_verify_empty_fail(verify_addr, 0U);
            v1_empty_verify_abort();
            return V1_FLASH_PUMP_ERROR;
        }
        if (!v1_flash_verify_empty_words(verify_addr, read_len))
        {
            v1_empty_verify_abort();
            return V1_FLASH_PUMP_ERROR;
        }
        s_v1_empty_verify.addr += read_len;
        if (s_v1_empty_verify.addr < V1_FLASH_SECTOR_BYTES)
        {
            return V1_FLASH_PUMP_BUSY;
        }
    }
#endif

    elapsed_ms = v1_flash_elapsed_ms_u32(s_v1_empty_verify.start_ms);
#if (IMU_CAPTURE_PROGRESS_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE) && ZY100_LOG_FLASH_VERBOSE
    DBG_DIRECT("[V1_F] verify_empty_ok ms=%u bytes=%u speed_kbps=%u",
               elapsed_ms,
               s_v1_empty_verify.verify_bytes,
               v1_flash_speed_kbps(s_v1_empty_verify.verify_bytes, elapsed_ms));
#else
    IMU_UNUSED(elapsed_ms);
#endif
    v1_empty_verify_abort();
    return V1_FLASH_PUMP_SAFE;
}

static uint64_t v1_flash_runtime_ms(void)
{
    return os_sys_time_get();
}

static bool v1_flash_resume_for_access(void)
{
    imu_status_t status;

    status = imu_bsp_power_ctrl(true);
    if ((status != IMU_STATUS_OK) && (status != IMU_STATUS_UNSUPPORTED))
    {
        return false;
    }
    status = imu_bsp_resume_after_dlps();
    return status == IMU_STATUS_OK;
}

static bool v1_export_header_is_valid(const v1_session_header_t *header)
{
    uint32_t expected_bytes;
    const uint32_t data_capacity = V1_RAW_SAMPLE_LIMIT_ADDR - V1_DATA_START_ADDR;

    if (header == NULL)
    {
        return false;
    }
    if ((header->magic != V1_SESSION_MAGIC) ||
        (header->version != V1_VERSION) ||
        (header->header_size != sizeof(v1_session_header_t)) ||
        (header->status != V1_SESSION_COMPLETE))
    {
        return false;
    }
    if ((header->flash_size != V1_FLASH_SIZE_BYTES) ||
        (header->data_start_addr != V1_DATA_START_ADDR) ||
        (header->limit_addr != V1_RAW_SAMPLE_LIMIT_ADDR) ||
        (header->packet_size != V1_IMU_PACKET_BYTES) ||
        (header->packets_per_block != V1_PACKETS_PER_BLOCK))
    {
        return false;
    }
    if ((header->stop_reason != V1_STOP_FLASH_95_PERCENT) ||
        (header->block_count == 0U) ||
        (header->block_count > V1_MAX_DATA_BLOCKS) ||
        (header->packet_count == 0U))
    {
        return false;
    }

    expected_bytes = header->block_count * V1_DATA_BLOCK_BYTES;
    if ((expected_bytes > data_capacity) ||
        (header->flash_bytes_written != expected_bytes))
    {
        return false;
    }
    if ((header->result_flags & (V1_EXPORT_RESULT_IMU_PASS | V1_EXPORT_RESULT_FLASH_PASS)) !=
        (V1_EXPORT_RESULT_IMU_PASS | V1_EXPORT_RESULT_FLASH_PASS))
    {
        return false;
    }
    if ((header->flash_write_error != 0U) ||
        (header->flash_wip_timeout != 0U) ||
        (header->flash_verify_error != 0U) ||
        (header->flash_seq_error != 0U) ||
        (header->flash_crc_error != 0U) ||
        (header->flash_queue_overflow != 0U))
    {
        return false;
    }
    if ((header->blocks_verified != header->block_count) ||
        (header->flash_packets_verified != header->packet_count))
    {
        return false;
    }

    return true;
}

static const char *v1_export_header_invalid_reason(const v1_session_header_t *header)
{
    uint32_t expected_bytes;
    const uint32_t data_capacity = V1_RAW_SAMPLE_LIMIT_ADDR - V1_DATA_START_ADDR;

    if (header == NULL)
    {
        return "null";
    }
    if (header->magic == 0xFFFFFFFFUL)
    {
        return "erased";
    }
    if (header->magic != V1_SESSION_MAGIC)
    {
        return "bad_magic";
    }
    if ((header->version != V1_VERSION) ||
        (header->header_size != sizeof(v1_session_header_t)))
    {
        return "bad_version_or_size";
    }
    if (header->status == V1_SESSION_FAILED)
    {
        return "failed_status";
    }
    if (header->status != V1_SESSION_COMPLETE)
    {
        return "not_complete";
    }
    if ((header->flash_size != V1_FLASH_SIZE_BYTES) ||
        (header->data_start_addr != V1_DATA_START_ADDR) ||
        (header->limit_addr != V1_RAW_SAMPLE_LIMIT_ADDR) ||
        (header->packet_size != V1_IMU_PACKET_BYTES) ||
        (header->packets_per_block != V1_PACKETS_PER_BLOCK))
    {
        return "layout_mismatch";
    }
    if (header->stop_reason != V1_STOP_FLASH_95_PERCENT)
    {
        return "stop_reason";
    }
    if ((header->block_count == 0U) ||
        (header->block_count > V1_MAX_DATA_BLOCKS) ||
        (header->packet_count == 0U))
    {
        return "count";
    }

    expected_bytes = header->block_count * V1_DATA_BLOCK_BYTES;
    if ((expected_bytes > data_capacity) ||
        (header->flash_bytes_written != expected_bytes))
    {
        return "bytes";
    }
    if ((header->result_flags & (V1_EXPORT_RESULT_IMU_PASS | V1_EXPORT_RESULT_FLASH_PASS)) !=
        (V1_EXPORT_RESULT_IMU_PASS | V1_EXPORT_RESULT_FLASH_PASS))
    {
        return "result_flags";
    }
    if ((header->flash_write_error != 0U) ||
        (header->flash_wip_timeout != 0U) ||
        (header->flash_verify_error != 0U) ||
        (header->flash_seq_error != 0U) ||
        (header->flash_crc_error != 0U) ||
        (header->flash_queue_overflow != 0U))
    {
        return "flash_errors";
    }
    if ((header->blocks_verified != header->block_count) ||
        (header->flash_packets_verified != header->packet_count))
    {
        return "verify_counts";
    }

    return "unknown";
}

static void v1_export_fill_session(const uint8_t *page,
                                   uint32_t metadata_addr,
                                   external_flash_capture_export_session_t *session)
{
    const v1_session_header_t *header = (const v1_session_header_t *)page;

    memset(session, 0, sizeof(*session));
    memcpy(session->metadata_page, page, V1_SESSION_METADATA_BYTES);
    session->metadata_addr = metadata_addr;
    session->round = header->round;
    session->data_start_addr = header->data_start_addr;
    session->data_bytes = header->flash_bytes_written;
    session->block_count = header->block_count;
    session->packet_count = header->packet_count;
    session->flash_bytes_written = header->flash_bytes_written;
    session->stop_reason = header->stop_reason;
    session->result_flags = header->result_flags;
    session->imu_pass = ((header->result_flags & V1_EXPORT_RESULT_IMU_PASS) != 0U);
    session->flash_pass = ((header->result_flags & V1_EXPORT_RESULT_FLASH_PASS) != 0U);
    session->verify_complete = (header->blocks_verified == header->block_count) &&
                               (header->flash_packets_verified == header->packet_count);
}

bool external_flash_capture_scan_complete_session(external_flash_capture_export_session_t *session)
{
    /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
    uint8_t page[V1_SESSION_METADATA_BYTES];
    uint32_t page_index;
    uint32_t addr;
    bool found = false;
    imu_status_t status;
    external_flash_capture_export_session_t latest;
    v1_session_header_t last_invalid_header;
    uint32_t last_invalid_addr = V1_SESSION_HEADER_ADDR;
    const char *last_invalid_reason = "none";
    bool last_invalid_valid = false;

    if (session == NULL)
    {
        return false;
    }
    memset(session, 0, sizeof(*session));
    memset(&latest, 0, sizeof(latest));
    memset(&last_invalid_header, 0xFF, sizeof(last_invalid_header));

    if (!v1_flash_resume_for_access())
    {
        DBG_DIRECT("[ERR][EXPORT] cache_scan_resume_failed");
        s_v1_cached_complete_valid = false;
        return false;
    }
    status = gd25q32e_init();
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[ERR][EXPORT] cache_scan_init_failed status=%u", (uint32_t)status);
        s_v1_cached_complete_valid = false;
        return false;
    }

    for (page_index = 0U;
         page_index < (V1_FLASH_SECTOR_BYTES / V1_FLASH_PAGE_BYTES);
         page_index++)
    {
        addr = V1_SESSION_HEADER_ADDR + (page_index * V1_FLASH_PAGE_BYTES);
        status = gd25q32e_read(addr, page, V1_FLASH_PAGE_BYTES);
        if (status != IMU_STATUS_OK)
        {
            DBG_DIRECT("[ERR][EXPORT] cache_scan_read_failed addr=0x%06x status=%u",
                       addr,
                       (uint32_t)status);
            s_v1_cached_complete_valid = false;
            return false;
        }
        if (v1_export_header_is_valid((const v1_session_header_t *)page))
        {
            v1_export_fill_session(page, addr, &latest);
            found = true;
        }
        else
        {
            const v1_session_header_t *header = (const v1_session_header_t *)page;

            if (!last_invalid_valid || (header->magic == V1_SESSION_MAGIC))
            {
                memcpy(&last_invalid_header, header, sizeof(last_invalid_header));
                last_invalid_addr = addr;
                last_invalid_reason = v1_export_header_invalid_reason(header);
                last_invalid_valid = true;
            }
        }
    }

    if (!found)
    {
        DBG_DIRECT("[ERR][EXPORT] cache_scan_no_complete addr=0x%06x reason=%s magic=0x%08x status=%u stop=%u block=%u packet=%u bytes=%u",
                   last_invalid_addr,
                   last_invalid_reason,
                   last_invalid_header.magic,
                   last_invalid_header.status,
                   last_invalid_header.stop_reason,
                   last_invalid_header.block_count,
                   last_invalid_header.packet_count,
                   last_invalid_header.flash_bytes_written);
        DBG_DIRECT("[ERR][EXPORT] cache_scan_header_errors write=%u wip=%u verify=%u seq=%u crc=%u q_ovf=%u verified=%u/%u",
                   last_invalid_header.flash_write_error,
                   last_invalid_header.flash_wip_timeout,
                   last_invalid_header.flash_verify_error,
                   last_invalid_header.flash_seq_error,
                   last_invalid_header.flash_crc_error,
                   last_invalid_header.flash_queue_overflow,
                   last_invalid_header.blocks_verified,
                   last_invalid_header.flash_packets_verified);
        s_v1_cached_complete_valid = false;
        return false;
    }

    if (!v1_marker_export_summary(&latest.marker_count, &latest.marker_bytes))
    {
        latest.marker_count = 0U;
        latest.marker_bytes = 0U;
    }

    v1_clear_writer_state();
    s_v1_cached_complete = latest;
    s_v1_cached_complete_valid = true;
    *session = latest;
    return true;
}

bool external_flash_capture_get_cached_complete_session(external_flash_capture_export_session_t *session)
{
    if ((session == NULL) || !s_v1_cached_complete_valid)
    {
        return false;
    }
    *session = s_v1_cached_complete;
    return true;
}

bool external_flash_capture_writer_is_idle(void)
{
    return !s_v1_running &&
           (s_v1_q_count == 0U) &&
           (s_v1_fill_index == 0xFFU) &&
           (s_v1_write_page_index == 0U);
}

bool external_flash_capture_flash_wip_clear(void)
{
    /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
    bool busy = true;
    imu_status_t status;

    status = gd25q32e_init();
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[ERR][FLASH] wip_check_init_failed status=%u",
                   (uint32_t)status);
        return false;
    }
    status = gd25q32e_is_busy(&busy);
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[ERR][FLASH] wip_check_failed status=%u",
                   (uint32_t)status);
        return false;
    }
    if (busy)
    {
        DBG_DIRECT("[WARN][FLASH] wip_busy");
        return false;
    }
    return true;
}

bool external_flash_capture_read_export_chunk(uint32_t addr, uint8_t *buf, uint16_t len)
{
    /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
    const uint32_t export_limit = V1_RAW_SAMPLE_LIMIT_ADDR;
    v1_flash_op_t prev_op;
    imu_status_t status;

    if ((buf == NULL) ||
        (len == 0U) ||
        (addr < V1_DATA_START_ADDR) ||
        (addr >= export_limit) ||
        ((uint32_t)len > (export_limit - addr)))
    {
        return false;
    }

    if (s_v1_sleep_abort_requested)
    {
        return false;
    }

    prev_op = s_v1_flash_op;
    v1_flash_set_op(V1_FLASH_OP_EXPORT_READ);
    status = gd25q32e_read_fast(addr, buf, len);
    v1_flash_set_op((prev_op == V1_FLASH_OP_EXPORT_READ) ? V1_FLASH_OP_IDLE : prev_op);
    return status == IMU_STATUS_OK;
}

static void v1_block_check_init(v1_block_check_result_t *check,
                                uint32_t block_seq,
                                uint32_t addr)
{
    if (check == NULL)
    {
        return;
    }

    memset(check, 0, sizeof(*check));
    check->status = V1_BLOCK_CHECK_FLASH_READ_ERROR;
    check->block_seq = block_seq;
    check->addr = addr;
}

static v1_block_check_status_t v1_block_check_finish(v1_block_check_result_t *check,
                                                     v1_block_check_status_t status)
{
    if (check != NULL)
    {
        check->status = status;
    }
    return status;
}

v1_block_check_status_t external_flash_capture_read_checked_block(uint32_t block_seq,
                                                                  uint8_t *buf,
                                                                  v1_block_check_result_t *check)
{
    const v1_flash_imu_block_t *block = (const v1_flash_imu_block_t *)buf;
    uint32_t addr = 0U;
    uint32_t valid_bytes;

    if (block_seq < V1_MAX_DATA_BLOCKS)
    {
        addr = V1_DATA_START_ADDR + (block_seq * V1_DATA_BLOCK_BYTES);
    }
    v1_block_check_init(check, block_seq, addr);

    if ((buf == NULL) ||
        (block_seq >= V1_MAX_DATA_BLOCKS) ||
        (addr < V1_DATA_START_ADDR) ||
        ((V1_RAW_SAMPLE_LIMIT_ADDR - addr) < V1_DATA_BLOCK_BYTES))
    {
        return v1_block_check_finish(check, V1_BLOCK_CHECK_FLASH_READ_ERROR);
    }

    if (!external_flash_capture_read_export_chunk(addr, buf, V1_DATA_BLOCK_BYTES))
    {
        return v1_block_check_finish(check, V1_BLOCK_CHECK_FLASH_READ_ERROR);
    }

    if (check != NULL)
    {
        check->expected_crc = block->payload_crc32;
    }

    if (block->magic != V1_BLOCK_MAGIC)
    {
        return v1_block_check_finish(check, V1_BLOCK_CHECK_BAD_MAGIC);
    }
    if (block->version != V1_VERSION)
    {
        return v1_block_check_finish(check, V1_BLOCK_CHECK_BAD_VERSION);
    }
    if (block->header_size != V1_BLOCK_HEADER_BYTES)
    {
        return v1_block_check_finish(check, V1_BLOCK_CHECK_BAD_HEADER_SIZE);
    }
    if (block->block_seq != block_seq)
    {
        return v1_block_check_finish(check, V1_BLOCK_CHECK_SEQ_MISMATCH);
    }
    if ((block->packet_count == 0U) || (block->packet_count > V1_PACKETS_PER_BLOCK))
    {
        return v1_block_check_finish(check, V1_BLOCK_CHECK_BAD_PACKET_COUNT);
    }
    if ((block->packet_count != V1_PACKETS_PER_BLOCK) &&
        ((block->flags & (V1_BLOCK_FLAG_PARTIAL | V1_BLOCK_FLAG_FINAL)) !=
         (V1_BLOCK_FLAG_PARTIAL | V1_BLOCK_FLAG_FINAL)))
    {
        return v1_block_check_finish(check, V1_BLOCK_CHECK_BAD_FLAGS);
    }

    /*
     * CRC compatibility contract with the PC parser:
     * - The block CRC field is payload_crc32 at raw block offset 20, little-endian uint32_t.
     * - The CRC covers only payload[0 : packet_count * 16].
     * - The 32-byte block header is not included.
     * - Unused payload tail bytes after packet_count are not included.
     * - A full block covers 254 * 16 = 4064 payload bytes.
     * - v1_crc32() is IEEE CRC32, reflected poly 0xEDB88320, init 0xFFFFFFFF,
     *   final xor 0xFFFFFFFF, matching PC binascii.crc32(data) & 0xFFFFFFFF.
     */
    valid_bytes = block->packet_count * V1_IMU_PACKET_BYTES;
    if (check != NULL)
    {
        check->actual_crc = v1_crc32(block->payload, valid_bytes);
    }
    if ((check != NULL) && (check->actual_crc != block->payload_crc32))
    {
        return v1_block_check_finish(check, V1_BLOCK_CHECK_CRC_MISMATCH);
    }
    if ((check == NULL) && (v1_crc32(block->payload, valid_bytes) != block->payload_crc32))
    {
        return V1_BLOCK_CHECK_CRC_MISMATCH;
    }

    return v1_block_check_finish(check, V1_BLOCK_CHECK_OK);
}

#if ZY100_FIFO_MARKER_FLASH_ENABLE
static bool v1_marker_header_is_valid(const v1_marker_table_header_t *header)
{
    uint32_t expected_payload_bytes;

    if (header == NULL)
    {
        return false;
    }
    if ((header->magic[0] != V1_MARKER_MAGIC_0) ||
        (header->magic[1] != V1_MARKER_MAGIC_1) ||
        (header->magic[2] != V1_MARKER_MAGIC_2) ||
        (header->magic[3] != V1_MARKER_MAGIC_3))
    {
        return false;
    }
    if ((header->version != V1_VERSION) ||
        (header->header_size != sizeof(v1_marker_table_header_t)) ||
        (header->record_size != IMU_SAMPLE_MARKER_RECORD_BYTES) ||
        (header->marker_count > IMU_SAMPLE_MARKER_MAX_COUNT))
    {
        return false;
    }

    expected_payload_bytes = header->marker_count * IMU_SAMPLE_MARKER_RECORD_BYTES;
    if ((header->payload_bytes != expected_payload_bytes) ||
        ((uint32_t)header->header_size + header->payload_bytes >
         V1_MARKER_TABLE_RESERVED_BYTES))
    {
        return false;
    }

    return true;
}

static bool v1_marker_read_header(v1_marker_table_header_t *header)
{
    v1_flash_op_t prev_op;
    imu_status_t status;

    if (header == NULL)
    {
        return false;
    }

    prev_op = s_v1_flash_op;
    v1_flash_set_op(V1_FLASH_OP_EXPORT_READ);
    status = gd25q32e_read_fast(V1_MARKER_TABLE_ADDR,
                                (uint8_t *)header,
                                (uint16_t)sizeof(*header));
    v1_flash_set_op((prev_op == V1_FLASH_OP_EXPORT_READ) ? V1_FLASH_OP_IDLE : prev_op);
    return status == IMU_STATUS_OK;
}

static bool v1_marker_crc_from_flash(uint32_t payload_bytes, uint32_t *crc_out)
{
    uint8_t buf[V1_FLASH_PAGE_BYTES];
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t offset = 0U;
    v1_flash_op_t prev_op;
    imu_status_t status;

    if (crc_out == NULL)
    {
        return false;
    }

    prev_op = s_v1_flash_op;
    v1_flash_set_op(V1_FLASH_OP_EXPORT_READ);
    while (offset < payload_bytes)
    {
        uint16_t len = (uint16_t)(((payload_bytes - offset) > V1_FLASH_PAGE_BYTES) ?
                                  V1_FLASH_PAGE_BYTES :
                                  (payload_bytes - offset));

        status = gd25q32e_read_fast(V1_MARKER_TABLE_ADDR +
                                    (uint32_t)sizeof(v1_marker_table_header_t) +
                                    offset,
                                    buf,
                                    len);
        if (status != IMU_STATUS_OK)
        {
            v1_flash_set_op((prev_op == V1_FLASH_OP_EXPORT_READ) ? V1_FLASH_OP_IDLE : prev_op);
            return false;
        }
        crc = v1_crc32_update(crc, buf, len);
        offset += len;
    }
    v1_flash_set_op((prev_op == V1_FLASH_OP_EXPORT_READ) ? V1_FLASH_OP_IDLE : prev_op);

    *crc_out = crc ^ 0xFFFFFFFFUL;
    return true;
}
#endif

static bool v1_marker_export_summary(uint32_t *marker_count, uint32_t *marker_bytes)
{
#if ZY100_FIFO_MARKER_FLASH_ENABLE
    v1_marker_table_header_t header;
    uint32_t crc = 0U;

    if (marker_count != NULL)
    {
        *marker_count = 0U;
    }
    if (marker_bytes == NULL)
    {
        return false;
    }
    *marker_bytes = 0U;

    if (!v1_marker_read_header(&header) ||
        !v1_marker_header_is_valid(&header))
    {
        return false;
    }

    if (!v1_marker_crc_from_flash(header.payload_bytes, &crc) ||
        (crc != header.crc32))
    {
        return false;
    }

    if (marker_count != NULL)
    {
        *marker_count = header.marker_count;
    }
    *marker_bytes = (uint32_t)header.header_size + header.payload_bytes;
    return true;
#else
    if (marker_count != NULL)
    {
        *marker_count = 0U;
    }
    if (marker_bytes != NULL)
    {
        *marker_bytes = 0U;
    }
    return false;
#endif
}

bool external_flash_capture_get_marker_export_summary(uint32_t *marker_count,
                                                      uint32_t *marker_bytes)
{
    return v1_marker_export_summary(marker_count, marker_bytes);
}

bool external_flash_capture_marker_export_available(uint32_t *marker_bytes)
{
    uint32_t marker_count;

    return v1_marker_export_summary(&marker_count, marker_bytes);
}

bool external_flash_capture_read_marker_export_chunk(uint32_t offset, uint8_t *buf, uint16_t len)
{
    /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
#if ZY100_FIFO_MARKER_FLASH_ENABLE
    v1_flash_op_t prev_op;
    imu_status_t status;

    if ((buf == NULL) ||
        (len == 0U) ||
        (offset >= V1_MARKER_TABLE_RESERVED_BYTES) ||
        ((uint32_t)len > (V1_MARKER_TABLE_RESERVED_BYTES - offset)))
    {
        return false;
    }

    prev_op = s_v1_flash_op;
    v1_flash_set_op(V1_FLASH_OP_EXPORT_READ);
    status = gd25q32e_read_fast(V1_MARKER_TABLE_ADDR + offset, buf, len);
    v1_flash_set_op((prev_op == V1_FLASH_OP_EXPORT_READ) ? V1_FLASH_OP_IDLE : prev_op);
    return status == IMU_STATUS_OK;
#else
    IMU_UNUSED(offset);
    IMU_UNUSED(buf);
    IMU_UNUSED(len);
    return false;
#endif
}

v1_flash_op_t external_flash_capture_get_op(void)
{
    return s_v1_flash_op;
}

bool external_flash_capture_can_abort_for_sleep(void)
{
    switch (s_v1_flash_op)
    {
    case V1_FLASH_OP_VERIFY_EMPTY:
    case V1_FLASH_OP_VERIFY_WRITTEN:
    case V1_FLASH_OP_EXPORT_READ:
    case V1_FLASH_OP_CAPTURE_WRITE:
    case V1_FLASH_OP_FLUSH:
        return true;
    case V1_FLASH_OP_IDLE:
    case V1_FLASH_OP_ERASE_WIP:
    case V1_FLASH_OP_MARKER_WRITE:
    case V1_FLASH_OP_POST_EXPORT_ERASE:
    default:
        return false;
    }
}

void external_flash_capture_request_sleep_abort(void)
{
    s_v1_sleep_abort_requested = true;
}

bool external_flash_capture_prepare_erased(uint32_t round)
{
    /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
    imu_status_t status;

    v1_reset_state(round);

    status = gd25q32e_init();
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[ERR][FLASH] init_failed status=%u", (uint32_t)status);
        return false;
    }

    s_v1_prepared = true;
    if (!v1_write_session_header(V1_SESSION_PREPARED, 0U, NULL))
    {
        s_v1_prepared = false;
        return false;
    }

    return true;
}

static bool v1_flash_read_status_for_erase(gd25q32e_status_regs_t *status_regs)
{
    imu_status_t status;

    if (status_regs == NULL)
    {
        return false;
    }

    memset(status_regs, 0xFF, sizeof(*status_regs));
    status = gd25q32e_read_status(status_regs);
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[ERR][FLASH] erase_status_failed status=%u", (uint32_t)status);
        return false;
    }

    return true;
}

static bool v1_flash_status_regs_invalid(const gd25q32e_status_regs_t *status_regs)
{
    if (status_regs == NULL)
    {
        return true;
    }

    return (status_regs->sr1 == 0xFFU) &&
           (status_regs->sr2 == 0xFFU) &&
           (status_regs->sr3 == 0xFFU);
}

static void v1_flash_log_erase_timeout(uint32_t elapsed_ms)
{
    gd25q32e_status_regs_t status_regs;

    if (!v1_flash_read_status_for_erase(&status_regs))
    {
        memset(&status_regs, 0xFF, sizeof(status_regs));
    }
    DBG_DIRECT("[ERR][FLASH] erase_timeout sr1=%02x sr2=%02x sr3=%02x ms=%u hard_ms=%u",
               status_regs.sr1,
               status_regs.sr2,
               status_regs.sr3,
               elapsed_ms,
               V1_CHIP_ERASE_HARD_TIMEOUT_MS);
}

static bool v1_flash_log_erase_slow_if_due(uint32_t elapsed_ms)
{
    const uint64_t now_ms = v1_flash_runtime_ms();
    gd25q32e_status_regs_t status_regs;
    bool log_due;

    if (!s_v1_post_erase.slow_warned)
    {
        log_due = true;
    }
    else
    {
        log_due = ((uint32_t)(now_ms - s_v1_post_erase.last_slow_log_ms) >=
                   V1_ERASE_SLOW_LOG_PERIOD_MS);
    }

    if (!log_due)
    {
        return true;
    }

    if (!v1_flash_read_status_for_erase(&status_regs) ||
        v1_flash_status_regs_invalid(&status_regs))
    {
        DBG_DIRECT("[ERR][FLASH] erase_status_invalid ms=%u sr1=%02x sr2=%02x sr3=%02x",
                   elapsed_ms,
                   status_regs.sr1,
                   status_regs.sr2,
                   status_regs.sr3);
        return false;
    }

    DBG_DIRECT("[WARN][FLASH] erase_slow ms=%u warn_ms=%u hard_ms=%u sr1=%02x sr2=%02x sr3=%02x",
               elapsed_ms,
               GD25Q32E_CHIP_ERASE_TIMEOUT_MS,
               V1_CHIP_ERASE_HARD_TIMEOUT_MS,
               status_regs.sr1,
               status_regs.sr2,
               status_regs.sr3);
    s_v1_post_erase.slow_warned = true;
    s_v1_post_erase.last_slow_log_ms = now_ms;
    return true;
}

static bool v1_flash_erase_empty_begin(uint8_t reason)
{
    gd25q32e_status_regs_t status_regs;
    imu_status_t status;
    uint64_t start_ms;

    if (s_v1_post_erase.active)
    {
        DBG_DIRECT("[ERR][FLASH] erase_begin_busy reason=%u op=%u",
                   (uint32_t)reason,
                   (uint32_t)s_v1_flash_op);
        return false;
    }
    status = gd25q32e_init();
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[ERR][FLASH] init_failed status=%u", (uint32_t)status);
        return false;
    }
    if (!external_flash_capture_flash_wip_clear())
    {
        DBG_DIRECT("[ERR][FLASH] erase_begin_wip_not_clear reason=%u",
                   (uint32_t)reason);
        return false;
    }

#if ZY100_LOG_FLASH_VERBOSE
    DBG_DIRECT("[FLASH_SEQ] step=status_read");
#endif
    status = gd25q32e_read_status(&status_regs);
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[ERR][FLASH] status_read_failed status=%u", (uint32_t)status);
        return false;
    }
    if (gd25q32e_status_is_protected(&status_regs))
    {
        DBG_DIRECT("[ERR][FLASH] protected sr1=%02x sr2=%02x sr3=%02x",
                   status_regs.sr1,
                   status_regs.sr2,
                   status_regs.sr3);
        return false;
    }

#if ZY100_LOG_FLASH_VERBOSE
    DBG_DIRECT("[FLASH_SEQ] step=chip_erase_begin reason=%u", (uint32_t)reason);
#endif
    DBG_DIRECT("[V1_F] erase_begin reason=%u", (uint32_t)reason);
    start_ms = v1_flash_runtime_ms();
    status = gd25q32e_chip_erase();
    if (status != IMU_STATUS_OK)
    {
        v1_mark_write_error(0U);
        DBG_DIRECT("[ERR][FLASH] erase_failed status=%u", (uint32_t)status);
        return false;
    }

    memset(&s_v1_post_erase, 0, sizeof(s_v1_post_erase));
    s_v1_post_erase.active = true;
    s_v1_post_erase.phase = V1_POST_ERASE_WIP;
    s_v1_post_erase.reason = reason;
    s_v1_post_erase.start_ms = start_ms;
    s_v1_post_erase.last_poll_ms = 0U;
    s_v1_cached_complete_valid = false;
    v1_flash_set_op((reason == V1_ERASE_REASON_POST_EXPORT) ?
                    V1_FLASH_OP_POST_EXPORT_ERASE :
                    V1_FLASH_OP_ERASE_WIP);
    return true;
}

static v1_flash_pump_status_t v1_flash_erase_empty_pump(bool sleep_mode)
{
    bool busy = false;
    imu_status_t status;
    uint64_t now_ms;
    uint32_t elapsed_ms;

    if (!s_v1_post_erase.active)
    {
        if (!s_v1_empty_verify.active)
        {
            v1_flash_set_op(V1_FLASH_OP_IDLE);
        }
        return V1_FLASH_PUMP_SAFE;
    }

    now_ms = v1_flash_runtime_ms();
    if (s_v1_post_erase.phase == V1_POST_ERASE_WIP)
    {
        if ((s_v1_post_erase.last_poll_ms != 0U) &&
            ((now_ms - s_v1_post_erase.last_poll_ms) < V1_ERASE_WIP_POLL_MS))
        {
            return V1_FLASH_PUMP_BUSY;
        }
        s_v1_post_erase.last_poll_ms = now_ms;

        status = gd25q32e_is_busy(&busy);
        if (status != IMU_STATUS_OK)
        {
            v1_mark_write_error(0U);
            s_v1_post_erase.active = false;
            s_v1_post_erase.phase = V1_POST_ERASE_NONE;
            v1_flash_set_op(V1_FLASH_OP_IDLE);
            return V1_FLASH_PUMP_ERROR;
        }
        elapsed_ms = v1_flash_elapsed_ms_u32(s_v1_post_erase.start_ms);
        if (busy)
        {
            if (elapsed_ms >= V1_CHIP_ERASE_HARD_TIMEOUT_MS)
            {
                v1_mark_timeout(V1_FLASH_WAIT_OP_CHIP_ERASE);
                s_v1_post_erase.active = false;
                s_v1_post_erase.phase = V1_POST_ERASE_NONE;
                v1_flash_log_erase_timeout(elapsed_ms);
                v1_flash_set_op(V1_FLASH_OP_IDLE);
                return V1_FLASH_PUMP_TIMEOUT;
            }
            if (elapsed_ms >= GD25Q32E_CHIP_ERASE_TIMEOUT_MS)
            {
                if (!v1_flash_log_erase_slow_if_due(elapsed_ms))
                {
                    v1_mark_write_error(0U);
                    s_v1_post_erase.active = false;
                    s_v1_post_erase.phase = V1_POST_ERASE_NONE;
                    v1_flash_set_op(V1_FLASH_OP_IDLE);
                    return V1_FLASH_PUMP_ERROR;
                }
            }
            return V1_FLASH_PUMP_BUSY;
        }

        DBG_DIRECT("[V1_F] erase_done ms=%u", elapsed_ms);
        if (sleep_mode && s_v1_sleep_abort_requested)
        {
            s_v1_post_erase.active = false;
            s_v1_post_erase.phase = V1_POST_ERASE_NONE;
            v1_flash_set_op(V1_FLASH_OP_IDLE);
            return V1_FLASH_PUMP_SAFE;
        }

        s_v1_post_erase.phase = V1_POST_ERASE_VERIFY;
        v1_flash_verify_empty_begin();
    }

    if (s_v1_post_erase.phase == V1_POST_ERASE_VERIFY)
    {
        v1_flash_pump_status_t verify_status;

        verify_status = v1_flash_verify_empty_poll(sleep_mode);
        if (verify_status == V1_FLASH_PUMP_BUSY)
        {
            return V1_FLASH_PUMP_BUSY;
        }
        if (verify_status == V1_FLASH_PUMP_ABORTED)
        {
            s_v1_post_erase.active = false;
            s_v1_post_erase.phase = V1_POST_ERASE_NONE;
            v1_flash_set_op(V1_FLASH_OP_IDLE);
            return V1_FLASH_PUMP_ABORTED;
        }
        if (verify_status != V1_FLASH_PUMP_SAFE)
        {
            s_v1_post_erase.active = false;
            s_v1_post_erase.phase = V1_POST_ERASE_NONE;
            v1_flash_set_op(V1_FLASH_OP_IDLE);
            return verify_status;
        }

        s_v1_post_erase.active = false;
        s_v1_post_erase.phase = V1_POST_ERASE_NONE;
        v1_flash_set_op(V1_FLASH_OP_IDLE);
        return V1_FLASH_PUMP_SAFE;
    }

    s_v1_post_erase.active = false;
    s_v1_post_erase.phase = V1_POST_ERASE_NONE;
    v1_flash_set_op(V1_FLASH_OP_IDLE);
    return V1_FLASH_PUMP_ERROR;
}

static external_flash_capture_erase_result_t v1_flash_erase_empty_poll(void)
{
    v1_flash_pump_status_t status;

    status = v1_flash_erase_empty_pump(false);
    switch (status)
    {
    case V1_FLASH_PUMP_SAFE:
        return EXTERNAL_FLASH_CAPTURE_ERASE_DONE;
    case V1_FLASH_PUMP_BUSY:
        return EXTERNAL_FLASH_CAPTURE_ERASE_BUSY;
    case V1_FLASH_PUMP_TIMEOUT:
        return EXTERNAL_FLASH_CAPTURE_ERASE_TIMEOUT;
    case V1_FLASH_PUMP_ABORTED:
    case V1_FLASH_PUMP_ERROR:
    default:
        return EXTERNAL_FLASH_CAPTURE_ERASE_ERROR;
    }
}

bool external_flash_capture_post_export_erase_begin(void)
{
    /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
    return v1_flash_erase_empty_begin(V1_ERASE_REASON_POST_EXPORT);
}

external_flash_capture_erase_result_t external_flash_capture_post_export_erase_poll(void)
{
    /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
    return v1_flash_erase_empty_poll();
}

bool external_flash_capture_post_export_erase_is_active(void)
{
    return s_v1_post_erase.active;
}

bool external_flash_capture_prepare_erase_only_begin(uint32_t round)
{
    /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
    imu_status_t status;

#if ZY100_LOG_FLASH_VERBOSE
    ZY100_LOG_FLASH_DETAIL("[FLASH_SEQ] step=prepare_begin owner=%u", (uint32_t)spi_bus_current_owner());
#endif
    v1_reset_state(round);

    status = gd25q32e_init();
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[ERR][FLASH] init_failed status=%u", (uint32_t)status);
        return false;
    }

    if (!v1_flash_probe_jedec_with_recovery())
    {
        return false;
    }

    return v1_flash_erase_empty_begin(V1_ERASE_REASON_WAKE);
}

external_flash_capture_erase_result_t external_flash_capture_prepare_erase_only_poll(void)
{
    /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
    return v1_flash_erase_empty_poll();
}

static bool v1_flash_wip_clear_no_force(void)
{
    bool busy = true;
    imu_status_t status;

    status = gd25q32e_is_busy(&busy);
    return (status == IMU_STATUS_OK) && !busy;
}

bool external_flash_capture_is_busy(void)
{
    if ((s_v1_flash_op != V1_FLASH_OP_IDLE) ||
        s_v1_post_erase.active ||
        s_v1_empty_verify.active ||
        s_v1_prepare.active ||
        !external_flash_capture_writer_is_idle())
    {
        return true;
    }

    return !v1_flash_wip_clear_no_force();
}

bool external_flash_capture_is_safe_for_sleep(void)
{
    return (s_v1_flash_op == V1_FLASH_OP_IDLE) &&
           !s_v1_post_erase.active &&
           !s_v1_empty_verify.active &&
           !s_v1_prepare.active &&
           external_flash_capture_writer_is_idle() &&
           v1_flash_wip_clear_no_force();
}

static v1_flash_pump_status_t v1_flash_abort_writer_for_sleep(void)
{
    imu_status_t status;

    s_v1_running = false;
    status = gd25q32e_wait_while_busy(GD25Q32E_PAGE_PROGRAM_TIMEOUT_MS);
    if (status == IMU_STATUS_TIMEOUT)
    {
        v1_mark_timeout(V1_FLASH_WAIT_OP_PAGE_PROGRAM);
        return V1_FLASH_PUMP_TIMEOUT;
    }
    if (status != IMU_STATUS_OK)
    {
        v1_mark_write_error(0U);
        return V1_FLASH_PUMP_ERROR;
    }

    /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
    spi_bus_force_idle();
    imu_bsp_cs_high();
    (void)imu_bsp_flash_cs_hold_high();
#if IMU_RUNTIME_VERBOSE_LOG_ENABLE
    DBG_DIRECT("[V1_ABORT] key_sleep pk=%u blk=%u",
               s_v1_packets_stored,
               s_v1_flash_stats.flash_blocks_written);
#endif
    v1_clear_writer_state();
    v1_flash_set_op(V1_FLASH_OP_IDLE);
    return V1_FLASH_PUMP_ABORTED;
}

v1_flash_pump_status_t external_flash_capture_pump_for_sleep(void)
{
    /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
    v1_flash_pump_status_t status;

    switch (s_v1_flash_op)
    {
    case V1_FLASH_OP_IDLE:
        if (!external_flash_capture_writer_is_idle())
        {
            s_v1_sleep_abort_requested = true;
            return v1_flash_abort_writer_for_sleep();
        }
        return external_flash_capture_is_safe_for_sleep() ?
               V1_FLASH_PUMP_SAFE :
               V1_FLASH_PUMP_BUSY;

    case V1_FLASH_OP_VERIFY_EMPTY:
        s_v1_sleep_abort_requested = true;
        v1_empty_verify_abort();
        if (s_v1_prepare.active)
        {
            s_v1_prepare.active = false;
            s_v1_prepare.phase = V1_PREP_NONE;
        }
        if (s_v1_post_erase.active)
        {
            s_v1_post_erase.active = false;
            s_v1_post_erase.phase = V1_POST_ERASE_NONE;
        }
        return V1_FLASH_PUMP_ABORTED;

    case V1_FLASH_OP_VERIFY_WRITTEN:
        s_v1_sleep_abort_requested = true;
        return V1_FLASH_PUMP_BUSY;

    case V1_FLASH_OP_EXPORT_READ:
        s_v1_sleep_abort_requested = true;
        /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
        spi_bus_force_idle();
        v1_flash_set_op(V1_FLASH_OP_IDLE);
        return V1_FLASH_PUMP_ABORTED;

    case V1_FLASH_OP_CAPTURE_WRITE:
    case V1_FLASH_OP_FLUSH:
        s_v1_sleep_abort_requested = true;
        return v1_flash_abort_writer_for_sleep();

    case V1_FLASH_OP_ERASE_WIP:
    case V1_FLASH_OP_POST_EXPORT_ERASE:
        s_v1_sleep_abort_requested = true;
        status = v1_flash_erase_empty_pump(true);
        if ((status == V1_FLASH_PUMP_SAFE) || (status == V1_FLASH_PUMP_ABORTED))
        {
            s_v1_prepare.active = false;
            s_v1_prepare.phase = V1_PREP_NONE;
        }
        return status;

    case V1_FLASH_OP_MARKER_WRITE:
        s_v1_sleep_abort_requested = true;
        return V1_FLASH_PUMP_BUSY;

    default:
        return V1_FLASH_PUMP_ERROR;
    }
}

#if ZY100_FIFO_MARKER_FLASH_ENABLE
static bool v1_marker_reserved_is_erased(void)
{
    uint8_t page[V1_FLASH_PAGE_BYTES];
    uint32_t offset;
    uint16_t i;
    imu_status_t status;

    for (offset = 0U;
         offset < V1_MARKER_TABLE_RESERVED_BYTES;
         offset += V1_FLASH_PAGE_BYTES)
    {
        status = gd25q32e_read_fast(V1_MARKER_TABLE_ADDR + offset,
                                    page,
                                    V1_FLASH_PAGE_BYTES);
        if (status != IMU_STATUS_OK)
        {
            return false;
        }
        for (i = 0U; i < V1_FLASH_PAGE_BYTES; i++)
        {
            if (page[i] != 0xFFU)
            {
                return false;
            }
        }
    }

    return true;
}

static uint32_t v1_marker_crc_from_ram(uint32_t marker_count)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;

    for (i = 0U; i < marker_count; i++)
    {
        imu_sample_marker_t record;

        if (!imu_sample_marker_get_record(i, &record))
        {
            break;
        }
        crc = v1_crc32_update(crc,
                              (const uint8_t *)&record,
                              IMU_SAMPLE_MARKER_RECORD_BYTES);
    }

    return crc ^ 0xFFFFFFFFUL;
}

static void v1_marker_fill_header(v1_marker_table_header_t *header,
                                  uint32_t marker_count,
                                  uint32_t payload_bytes,
                                  uint32_t crc)
{
    memset(header, 0xFF, sizeof(*header));
    header->magic[0] = V1_MARKER_MAGIC_0;
    header->magic[1] = V1_MARKER_MAGIC_1;
    header->magic[2] = V1_MARKER_MAGIC_2;
    header->magic[3] = V1_MARKER_MAGIC_3;
    header->version = V1_VERSION;
    header->header_size = sizeof(v1_marker_table_header_t);
    header->record_size = IMU_SAMPLE_MARKER_RECORD_BYTES;
    header->reserved0 = 0xFFFFU;
    header->marker_count = marker_count;
    header->payload_bytes = payload_bytes;
    header->crc32 = crc;
    header->flags = 0UL;
    header->reserved1 = 0xFFFFFFFFUL;
}

static void v1_marker_build_page(uint32_t page_offset,
                                 const v1_marker_table_header_t *header,
                                 bool include_header,
                                 uint32_t marker_count,
                                 uint8_t *page)
{
    uint32_t i;

    memset(page, 0xFF, V1_FLASH_PAGE_BYTES);
    if (include_header && (page_offset == 0U) && (header != NULL))
    {
        memcpy(page, header, sizeof(*header));
    }

    for (i = 0U; i < marker_count; i++)
    {
        const uint32_t record_offset =
            (uint32_t)sizeof(v1_marker_table_header_t) +
            (i * IMU_SAMPLE_MARKER_RECORD_BYTES);

        if ((record_offset >= page_offset) &&
            ((record_offset + IMU_SAMPLE_MARKER_RECORD_BYTES) <=
             (page_offset + V1_FLASH_PAGE_BYTES)))
        {
            imu_sample_marker_t record;

            if (imu_sample_marker_get_record(i, &record))
            {
                memcpy(&page[record_offset - page_offset],
                       &record,
                       IMU_SAMPLE_MARKER_RECORD_BYTES);
            }
        }
    }
}

static bool v1_marker_program_page(uint32_t page_offset, const uint8_t *page)
{
    imu_status_t status;

    status = gd25q32e_page_program(V1_MARKER_TABLE_ADDR + page_offset,
                                   page,
                                   V1_FLASH_PAGE_BYTES);
    if (status != IMU_STATUS_OK)
    {
        return false;
    }

    return v1_flash_wait(GD25Q32E_PAGE_PROGRAM_TIMEOUT_MS, V1_FLASH_WAIT_OP_PAGE_PROGRAM);
}

static void v1_marker_write_error(uint32_t table_bytes,
                                  uint32_t crc,
                                  const char *reason)
{
    imu_sample_marker_note_flash_write_error(V1_MARKER_TABLE_ADDR, table_bytes, crc);
    DBG_DIRECT("[WARN][MARKER] marker_table_write_failed reason=%s",
               (reason != NULL) ? reason : "unknown");
}

static void v1_marker_write_table(void)
{
    uint8_t page[V1_FLASH_PAGE_BYTES];
    v1_marker_table_header_t header;
    uint32_t marker_count = imu_sample_marker_count();
    uint32_t payload_bytes = marker_count * IMU_SAMPLE_MARKER_RECORD_BYTES;
    uint32_t table_bytes = (uint32_t)sizeof(v1_marker_table_header_t) + payload_bytes;
    uint32_t crc = v1_marker_crc_from_ram(marker_count);
    uint32_t page_offset;
    uint32_t last_record_page;
    uint32_t exported_bytes = 0U;
    v1_flash_op_t prev_op = s_v1_flash_op;

    if (table_bytes > V1_MARKER_TABLE_RESERVED_BYTES)
    {
        v1_marker_write_error(table_bytes, crc, "capacity");
        return;
    }

    v1_flash_set_op(V1_FLASH_OP_MARKER_WRITE);
    if (!v1_marker_reserved_is_erased())
    {
        v1_flash_set_op(prev_op);
        v1_marker_write_error(table_bytes, crc, "not_erased");
        return;
    }

    v1_marker_fill_header(&header, marker_count, payload_bytes, crc);
    if (payload_bytes != 0U)
    {
        last_record_page = (table_bytes - 1U) & ~(uint32_t)(V1_FLASH_PAGE_BYTES - 1U);
        for (page_offset = 0U;
             page_offset <= last_record_page;
             page_offset += V1_FLASH_PAGE_BYTES)
        {
            v1_marker_build_page(page_offset,
                                 &header,
                                 false,
                                 marker_count,
                                 page);
            if (!v1_marker_program_page(page_offset, page))
            {
                v1_flash_set_op(prev_op);
                v1_marker_write_error(table_bytes, crc, "record_write");
                return;
            }
        }
    }

    v1_marker_build_page(0U, &header, true, marker_count, page);
    if (!v1_marker_program_page(0U, page))
    {
        v1_flash_set_op(prev_op);
        v1_marker_write_error(table_bytes, crc, "header_write");
        return;
    }
    v1_flash_set_op(prev_op);

    if (!external_flash_capture_marker_export_available(&exported_bytes) ||
        (exported_bytes != table_bytes))
    {
        v1_marker_write_error(table_bytes, crc, "verify");
        return;
    }

    imu_sample_marker_note_flash_write_success(V1_MARKER_TABLE_ADDR, table_bytes, crc);
}
#endif

static bool v1_write_session_header(uint32_t status_code,
                                    uint32_t stop_reason,
                                    const external_flash_capture_imu_stats_t *imu_stats)
{
    uint8_t page[V1_FLASH_PAGE_BYTES];
    v1_session_header_t *header = (v1_session_header_t *)page;
    uint32_t addr;
    uint32_t result_flags = 0U;
    bool imu_pass = false;
    bool flash_pass = false;
    v1_flash_op_t prev_op;
    imu_status_t status;

    if (s_v1_session_page >= (V1_FLASH_SECTOR_BYTES / V1_FLASH_PAGE_BYTES))
    {
        s_v1_flash_stats.flash_write_error_count++;
        return false;
    }

    if (imu_stats != NULL)
    {
        imu_pass = (imu_stats->fifo_lost_pkt_count == 0U) &&
                   (imu_stats->fifo_full_count == 0U) &&
                   (imu_stats->bad_header_count == 0U) &&
                   (imu_stats->ts_jump_count == 0U) &&
                   (imu_stats->read_error_count == 0U) &&
                   (imu_stats->packet_count > 0U);
    }

    flash_pass = (stop_reason != V1_STOP_FATAL) &&
                 (s_v1_packets_stored > 0U) &&
                 (s_v1_flash_stats.flash_write_error_count == 0U) &&
                 (s_v1_flash_stats.flash_wip_timeout_count == 0U) &&
                 (s_v1_flash_stats.flash_queue_overflow_count == 0U) &&
                 (s_v1_flash_stats.flash_verify_error_count == 0U) &&
                 (s_v1_flash_stats.flash_seq_error_count == 0U) &&
                 (s_v1_flash_stats.flash_crc_error_count == 0U) &&
                 (s_v1_flash_stats.flash_packets_verified == s_v1_packets_stored) &&
                 (s_v1_flash_stats.flash_blocks_verified == s_v1_flash_stats.flash_blocks_written);

    result_flags |= imu_pass ? 0x00000001UL : 0U;
    result_flags |= flash_pass ? 0x00000002UL : 0U;

    memset(page, 0xFF, sizeof(page));
    header->magic = V1_SESSION_MAGIC;
    header->version = V1_VERSION;
    header->header_size = sizeof(v1_session_header_t);
    header->status = status_code;
    header->round = s_v1_round;
    header->flash_size = V1_FLASH_SIZE_BYTES;
    header->data_start_addr = V1_DATA_START_ADDR;
    header->limit_addr = V1_RAW_SAMPLE_LIMIT_ADDR;
    header->packet_size = V1_IMU_PACKET_BYTES;
    header->packets_per_block = V1_PACKETS_PER_BLOCK;
    header->imu_odr_hz = V1_CAPTURE_SAMPLE_HZ;
    header->stop_reason = stop_reason;
    header->packet_count = s_v1_packets_stored;
    header->block_count = s_v1_flash_stats.flash_blocks_written;
    header->flash_bytes_written = s_v1_flash_stats.flash_bytes_written;
    header->imu_lost = (imu_stats != NULL) ? imu_stats->fifo_lost_pkt_count : 0U;
    header->imu_full = (imu_stats != NULL) ? imu_stats->fifo_full_count : 0U;
    header->imu_bad_header = (imu_stats != NULL) ? imu_stats->bad_header_count : 0U;
    header->imu_ts_bad = (imu_stats != NULL) ? imu_stats->ts_jump_count : 0U;
    header->imu_read_error = (imu_stats != NULL) ? imu_stats->read_error_count : 0U;
    header->flash_write_error = s_v1_flash_stats.flash_write_error_count;
    header->flash_wip_timeout = s_v1_flash_stats.flash_wip_timeout_count;
    header->flash_verify_error = s_v1_flash_stats.flash_verify_error_count;
    header->flash_seq_error = s_v1_flash_stats.flash_seq_error_count;
    header->flash_crc_error = s_v1_flash_stats.flash_crc_error_count;
    header->flash_queue_overflow = s_v1_flash_stats.flash_queue_overflow_count;
    header->result_flags = result_flags;
    header->blocks_verified = s_v1_flash_stats.flash_blocks_verified;
    header->flash_packets_verified = s_v1_flash_stats.flash_packets_verified;
    addr = V1_SESSION_HEADER_ADDR + ((uint32_t)s_v1_session_page * V1_FLASH_PAGE_BYTES);
    prev_op = s_v1_flash_op;
    if ((prev_op == V1_FLASH_OP_IDLE) || (prev_op == V1_FLASH_OP_VERIFY_EMPTY))
    {
        v1_flash_set_op(V1_FLASH_OP_CAPTURE_WRITE);
    }
    status = gd25q32e_page_program(addr, page, V1_FLASH_PAGE_BYTES);
    if (status != IMU_STATUS_OK)
    {
        v1_flash_set_op(prev_op);
        v1_mark_write_error(addr);
        return false;
    }
    if ((status_code == V1_SESSION_COMPLETE) ||
        (status_code == V1_SESSION_FAILED))
    {
        if (!v1_flash_wait_for_finalize_idle("session_header"))
        {
            v1_flash_set_op(prev_op);
            return false;
        }
    }
    else if (!v1_flash_wait(GD25Q32E_PAGE_PROGRAM_TIMEOUT_MS, V1_FLASH_WAIT_OP_PAGE_PROGRAM))
    {
        v1_flash_set_op(prev_op);
        return false;
    }
    v1_flash_set_op(prev_op);

    s_v1_session_page++;
    return true;
}

bool external_flash_capture_prepare(uint32_t round)
{
    /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
    v1_flash_prepare_status_t status;

    if (!external_flash_capture_prepare_begin(round))
    {
        return false;
    }

    do
    {
        status = external_flash_capture_prepare_poll();
        if (status == V1_FLASH_PREP_BUSY)
        {
            os_delay(V1_ERASE_WIP_POLL_MS);
        }
    } while (status == V1_FLASH_PREP_BUSY);

    return status == V1_FLASH_PREP_DONE;
}

static bool v1_flash_jedec_all_ff(const gd25q32e_jedec_id_t *id)
{
    if (id == NULL)
    {
        return false;
    }

    return (id->manufacturer_id == 0xFFU) &&
           (id->memory_type == 0xFFU) &&
           (id->density == 0xFFU);
}

static bool v1_flash_probe_jedec_with_recovery(void)
{
    gd25q32e_jedec_id_t id = {0U};
    gd25q32e_status_regs_t status_regs = {0U};
    imu_status_t status;
    bool first_all_ff;

    status = gd25q32e_read_jedec_id_attempt(&id, 1U);
    first_all_ff = ((status == IMU_STATUS_OK) && v1_flash_jedec_all_ff(&id));
    if ((status == IMU_STATUS_OK) && gd25q32e_jedec_is_4mbyte(&id))
    {
#if ZY100_LOG_FLASH_VERBOSE
        DBG_DIRECT("[FLASH_SEQ] step=status_read");
#endif
        status = gd25q32e_read_status(&status_regs);
        if (status != IMU_STATUS_OK)
        {
            DBG_DIRECT("[ERR][FLASH] status_read_failed status=%u", (uint32_t)status);
            return false;
        }
#if ZY100_LOG_FLASH_VERBOSE
        DBG_DIRECT("[V1_F] flash_probe_ok jedec=%02x%02x%02x sr1=%02x sr2=%02x sr3=%02x",
                   id.manufacturer_id,
                   id.memory_type,
                   id.density,
                   status_regs.sr1,
                   status_regs.sr2,
                   status_regs.sr3);
#endif
        return true;
    }

    DBG_DIRECT("[WARN][FLASH] jedec_retry status=%u id=%02x%02x%02x",
               (uint32_t)status,
               id.manufacturer_id,
               id.memory_type,
               id.density);

    status = gd25q32e_init();
    if (status == IMU_STATUS_OK)
    {
        id.manufacturer_id = 0U;
        id.memory_type = 0U;
        id.density = 0U;
        status = gd25q32e_read_jedec_id_attempt(&id, 2U);
    }

    if ((status == IMU_STATUS_OK) && gd25q32e_jedec_is_4mbyte(&id))
    {
#if ZY100_LOG_FLASH_VERBOSE
        DBG_DIRECT("[FLASH_SEQ] step=status_read");
#endif
        status = gd25q32e_read_status(&status_regs);
        if (status != IMU_STATUS_OK)
        {
            DBG_DIRECT("[ERR][FLASH] status_read_failed status=%u", (uint32_t)status);
            return false;
        }
#if ZY100_LOG_FLASH_VERBOSE
        DBG_DIRECT("[V1_F] flash_probe_ok jedec=%02x%02x%02x sr1=%02x sr2=%02x sr3=%02x",
                   id.manufacturer_id,
                   id.memory_type,
                   id.density,
                   status_regs.sr1,
                   status_regs.sr2,
                   status_regs.sr3);
#endif
        return true;
    }

    if (first_all_ff && (status == IMU_STATUS_OK) && v1_flash_jedec_all_ff(&id))
    {
        DBG_DIRECT("[FLASH_PROBE_FAIL] reason=jedec_all_ff");
    }

    DBG_DIRECT("[ERR][FLASH] jedec_id status=%u id=%02x%02x%02x",
               (uint32_t)status,
               id.manufacturer_id,
               id.memory_type,
               id.density);
    return false;
}

bool external_flash_capture_prepare_begin(uint32_t round)
{
    /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
    imu_status_t status;

#if ZY100_LOG_FLASH_VERBOSE
    ZY100_LOG_FLASH_DETAIL("[FLASH_SEQ] step=prepare_begin owner=%u", (uint32_t)spi_bus_current_owner());
#endif
    v1_reset_state(round);

    status = gd25q32e_init();
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[ERR][FLASH] init_failed status=%u", (uint32_t)status);
        return false;
    }

    if (!v1_flash_probe_jedec_with_recovery())
    {
        return false;
    }

    if (!v1_flash_erase_empty_begin(V1_ERASE_REASON_WAKE))
    {
        return false;
    }

    memset(&s_v1_prepare, 0, sizeof(s_v1_prepare));
    s_v1_prepare.active = true;
    s_v1_prepare.phase = V1_PREP_ERASE;
    s_v1_prepare.round = round;
    return true;
}

v1_flash_prepare_status_t external_flash_capture_prepare_poll(void)
{
    /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
    v1_flash_pump_status_t pump_status;

    if (!s_v1_prepare.active)
    {
        return s_v1_prepared ? V1_FLASH_PREP_DONE : V1_FLASH_PREP_ERROR;
    }

    if (s_v1_prepare.phase == V1_PREP_ERASE)
    {
        pump_status = v1_flash_erase_empty_pump(false);
        if (pump_status == V1_FLASH_PUMP_BUSY)
        {
            return V1_FLASH_PREP_BUSY;
        }
        if (pump_status == V1_FLASH_PUMP_TIMEOUT)
        {
            s_v1_prepare.active = false;
            s_v1_prepare.phase = V1_PREP_NONE;
            return V1_FLASH_PREP_TIMEOUT;
        }
        if (pump_status == V1_FLASH_PUMP_ABORTED)
        {
            s_v1_prepare.active = false;
            s_v1_prepare.phase = V1_PREP_NONE;
            return V1_FLASH_PREP_ABORTED;
        }
        if (pump_status != V1_FLASH_PUMP_SAFE)
        {
            s_v1_prepare.active = false;
            s_v1_prepare.phase = V1_PREP_NONE;
            return V1_FLASH_PREP_ERROR;
        }

#if (IMU_CAPTURE_PROGRESS_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE) && ZY100_LOG_FLASH_VERBOSE
        DBG_DIRECT("[V1_F] erase_ok");
#endif
        (void)external_flash_capture_led_green_on();
        s_v1_prepared = true;
        s_v1_prepare.phase = V1_PREP_HEADER;
    }

    if (s_v1_prepare.phase == V1_PREP_HEADER)
    {
        if (!v1_write_session_header(V1_SESSION_PREPARED, 0U, NULL))
        {
            (void)external_flash_capture_led_all_off();
            s_v1_prepared = false;
            s_v1_prepare.active = false;
            s_v1_prepare.phase = V1_PREP_NONE;
            v1_flash_set_op(V1_FLASH_OP_IDLE);
            return V1_FLASH_PREP_ERROR;
        }

        s_v1_prepare.active = false;
        s_v1_prepare.phase = V1_PREP_NONE;
        v1_flash_set_op(V1_FLASH_OP_IDLE);
        return V1_FLASH_PREP_DONE;
    }

    s_v1_prepare.active = false;
    s_v1_prepare.phase = V1_PREP_NONE;
    v1_flash_set_op(V1_FLASH_OP_IDLE);
    return V1_FLASH_PREP_ERROR;
}

bool external_flash_capture_begin(void)
{
    if (!s_v1_prepared)
    {
        return false;
    }

    if (!v1_prepare_new_fill_block())
    {
        return false;
    }

    if (!v1_write_session_header(V1_SESSION_RUNNING, 0U, NULL))
    {
        return false;
    }

    s_v1_running = true;
    s_v1_capture_start_ms = v1_flash_runtime_ms();
    DBG_DIRECT("[CAP_MODE] mode=T3 rt=%u tim6=%u fifo=%u flash=%u ois=%u",
               (uint32_t)(ZY100_RT_MARKER_ENABLE != 0),
               (uint32_t)((ZY100_RT_MARKER_ENABLE != 0) &&
                          (ZY100_RT_MARKER_TIMER_ENABLE != 0)),
               V1_CAPTURE_SAMPLE_HZ,
               (uint32_t)(V1_IMU_FLASH_CAPTURE_ENABLE != 0),
               (uint32_t)ZY100_OIS_ENABLE);
    DBG_DIRECT("[CAP] start");
    return true;
}

uint32_t external_flash_capture_accept_capacity_packets(void)
{
    uint32_t current_rem = 0U;
    uint32_t free_count = 0U;
    uint32_t total_accept;

    if (!s_v1_running || s_v1_fatal || s_v1_full ||
        (s_v1_full_blocks_accepted >= V1_MAX_DATA_BLOCKS))
    {
        return 0U;
    }

    total_accept = v1_total_accept_capacity_packets(&current_rem, &free_count);
    v1_log_accept_capacity_if_needed(current_rem, total_accept, free_count);
    return total_accept;
}

uint32_t external_flash_capture_final_tail_capacity_packets(uint32_t burst_packets)
{
    bool active_fill;
    uint32_t current_rem = 0U;
    uint32_t free_count = 0U;
    uint32_t total_accept;
    uint32_t remaining_capacity;

    if ((burst_packets == 0U) ||
        !s_v1_running ||
        s_v1_fatal ||
        s_v1_full ||
        (s_v1_full_blocks_accepted >= V1_MAX_DATA_BLOCKS))
    {
        return 0U;
    }

    active_fill = v1_fill_block_active();
    if (!active_fill)
    {
        return 0U;
    }

    total_accept = v1_total_accept_capacity_packets(&current_rem, &free_count);
    remaining_capacity = v1_remaining_data_capacity_packets(active_fill);

    if ((remaining_capacity == 0U) ||
        (remaining_capacity >= burst_packets) ||
        (total_accept != remaining_capacity) ||
        (current_rem != remaining_capacity))
    {
        return 0U;
    }

    IMU_UNUSED(free_count);
    return remaining_capacity;
}

bool external_flash_capture_append_packet(const uint8_t *packet)
{
    uint8_t *dst;

    if ((packet == NULL) ||
        !s_v1_running ||
        s_v1_fatal ||
        s_v1_full ||
        (s_v1_full_blocks_accepted >= V1_MAX_DATA_BLOCKS))
    {
        v1_mark_fatal_queue();
        return false;
    }

    if (!v1_fill_block_active() && !v1_prepare_new_fill_block())
    {
        return false;
    }

    dst = &s_v1_blocks[s_v1_fill_index].payload[s_v1_fill_packet_count * V1_IMU_PACKET_BYTES];
    memcpy(dst, packet, V1_IMU_PACKET_BYTES);
    s_v1_fill_packet_count++;
    s_v1_packets_stored++;

    if (s_v1_fill_packet_count >= V1_PACKETS_PER_BLOCK)
    {
        uint8_t full_index = s_v1_fill_index;
        if (!v1_enqueue_block(full_index, V1_PACKETS_PER_BLOCK, 0U))
        {
            return false;
        }
        s_v1_full_blocks_accepted++;
        s_v1_fill_index = 0xFFU;
        s_v1_fill_packet_count = 0U;

        if ((s_v1_full_blocks_accepted % V1_PROGRESS_BLOCK_STEP) == 0U)
        {
#if (IMU_CAPTURE_PROGRESS_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE) && ZY100_LOG_FLASH_VERBOSE
            DBG_DIRECT("[V1_P] blk=%u pk=%u", s_v1_full_blocks_accepted, s_v1_packets_stored);
#endif
        }

        if (s_v1_full_blocks_accepted >= V1_MAX_DATA_BLOCKS)
        {
            s_v1_full = true;
            s_v1_stop_reason = V1_STOP_FLASH_95_PERCENT;
            return true;
        }
    }

    return true;
}

static void v1_release_written_front_block(void)
{
    uint8_t block_index;

    if (s_v1_q_count == 0U)
    {
        return;
    }

    block_index = s_v1_pending_q[s_v1_q_head];
    if (block_index >= V1_BLOCK_QUEUE_DEPTH)
    {
        v1_mark_fatal_queue();
        return;
    }

    s_v1_block_state[block_index] = V1_BLOCK_FREE;
    s_v1_q_head = (uint8_t)((s_v1_q_head + 1U) % V1_BLOCK_QUEUE_DEPTH);
    s_v1_q_count--;
    s_v1_write_page_index = 0U;
    s_v1_flash_stats.flash_blocks_written++;
    if ((s_v1_q_count == 0U) &&
        ((s_v1_flash_op == V1_FLASH_OP_CAPTURE_WRITE) ||
         (s_v1_flash_op == V1_FLASH_OP_FLUSH)))
    {
        v1_flash_set_op(V1_FLASH_OP_IDLE);
    }
}

static bool v1_pump_one_page(bool blocking)
{
    uint8_t block_index;
    uint32_t addr;
    bool busy = false;
    imu_status_t status;

    if ((s_v1_q_count == 0U) || s_v1_fatal)
    {
        if (s_v1_flash_op == V1_FLASH_OP_CAPTURE_WRITE)
        {
            v1_flash_set_op(V1_FLASH_OP_IDLE);
        }
        return true;
    }

    if (s_v1_flash_op != V1_FLASH_OP_FLUSH)
    {
        v1_flash_set_op(V1_FLASH_OP_CAPTURE_WRITE);
    }

    status = gd25q32e_is_busy(&busy);
    if (status != IMU_STATUS_OK)
    {
        v1_mark_write_error(0U);
        return false;
    }
    if (busy)
    {
        if (blocking)
        {
            if (!v1_flash_wait_for_finalize_idle("flush_wait_busy"))
            {
                return false;
            }
        }
        else
        {
            return true;
        }
    }

    if (s_v1_write_page_index >= (V1_DATA_BLOCK_BYTES / V1_FLASH_PAGE_BYTES))
    {
        v1_release_written_front_block();
        return true;
    }

    block_index = s_v1_pending_q[s_v1_q_head];
    if ((block_index >= V1_BLOCK_QUEUE_DEPTH) ||
        (s_v1_block_state[block_index] != V1_BLOCK_PENDING))
    {
        v1_mark_fatal_queue();
        return false;
    }

    addr = V1_DATA_START_ADDR +
           (s_v1_blocks[block_index].block_seq * V1_DATA_BLOCK_BYTES) +
           ((uint32_t)s_v1_write_page_index * V1_FLASH_PAGE_BYTES);
    if (addr >= V1_RAW_SAMPLE_LIMIT_ADDR)
    {
        v1_mark_write_error(addr);
        return false;
    }

    status = gd25q32e_page_program(addr,
                                   ((const uint8_t *)&s_v1_blocks[block_index]) +
                                       ((uint32_t)s_v1_write_page_index * V1_FLASH_PAGE_BYTES),
                                   V1_FLASH_PAGE_BYTES);
    if (status != IMU_STATUS_OK)
    {
        v1_mark_write_error(addr);
        return false;
    }

    s_v1_write_page_index++;
    s_v1_flash_stats.flash_pages_written++;
    s_v1_flash_stats.flash_bytes_written += V1_FLASH_PAGE_BYTES;

    if (blocking)
    {
        return v1_flash_wait_for_finalize_idle("flush_page_program");
    }

    return true;
}

static bool v1_pump_one_page_bounded(void)
{
    uint8_t block_index;
    uint32_t addr;
    bool busy = false;
    imu_status_t status;

    if ((s_v1_q_count == 0U) || s_v1_fatal)
    {
        s_v1_runtime_wip_polled_clear = false;
        s_v1_last_runtime_wip_busy = false;
        s_v1_last_pump_result = s_v1_fatal ?
                                EXTERNAL_FLASH_CAPTURE_PUMP_ERROR :
                                EXTERNAL_FLASH_CAPTURE_PUMP_NO_WORK;
        if (s_v1_flash_op == V1_FLASH_OP_CAPTURE_WRITE)
        {
            v1_flash_set_op(V1_FLASH_OP_IDLE);
        }
        return true;
    }

    if ((s_v1_flash_op != V1_FLASH_OP_FLUSH) &&
        (s_v1_flash_op != V1_FLASH_OP_CAPTURE_WRITE))
    {
        v1_flash_set_op(V1_FLASH_OP_CAPTURE_WRITE);
        s_v1_last_pump_result = EXTERNAL_FLASH_CAPTURE_PUMP_NO_WORK;
        return true;
    }

    if (s_v1_write_page_index >= (V1_DATA_BLOCK_BYTES / V1_FLASH_PAGE_BYTES))
    {
        s_v1_runtime_wip_polled_clear = false;
        v1_release_written_front_block();
        s_v1_last_runtime_wip_busy = false;
        s_v1_last_pump_result = s_v1_fatal ?
                                EXTERNAL_FLASH_CAPTURE_PUMP_ERROR :
                                EXTERNAL_FLASH_CAPTURE_PUMP_BLOCK_DONE;
        return true;
    }

    if (!s_v1_runtime_wip_polled_clear)
    {
        status = gd25q32e_is_busy(&busy);
        if (status != IMU_STATUS_OK)
        {
            s_v1_last_runtime_wip_busy = false;
            s_v1_last_pump_result = EXTERNAL_FLASH_CAPTURE_PUMP_ERROR;
            v1_mark_write_error(0U);
            return false;
        }
        s_v1_last_runtime_wip_busy = busy;
        if (busy)
        {
            s_v1_last_pump_result = EXTERNAL_FLASH_CAPTURE_PUMP_WIP_BUSY;
            return true;
        }
        s_v1_runtime_wip_polled_clear = true;
        s_v1_last_pump_result = EXTERNAL_FLASH_CAPTURE_PUMP_NO_WORK;
        return true;
    }

    block_index = s_v1_pending_q[s_v1_q_head];
    if ((block_index >= V1_BLOCK_QUEUE_DEPTH) ||
        (s_v1_block_state[block_index] != V1_BLOCK_PENDING))
    {
        s_v1_runtime_wip_polled_clear = false;
        s_v1_last_pump_result = EXTERNAL_FLASH_CAPTURE_PUMP_ERROR;
        v1_mark_fatal_queue();
        return false;
    }

    addr = V1_DATA_START_ADDR +
           (s_v1_blocks[block_index].block_seq * V1_DATA_BLOCK_BYTES) +
           ((uint32_t)s_v1_write_page_index * V1_FLASH_PAGE_BYTES);
    if (addr >= V1_RAW_SAMPLE_LIMIT_ADDR)
    {
        s_v1_runtime_wip_polled_clear = false;
        s_v1_last_pump_result = EXTERNAL_FLASH_CAPTURE_PUMP_ERROR;
        v1_mark_write_error(addr);
        return false;
    }

    status = gd25q32e_page_program(addr,
                                   ((const uint8_t *)&s_v1_blocks[block_index]) +
                                       ((uint32_t)s_v1_write_page_index * V1_FLASH_PAGE_BYTES),
                                   V1_FLASH_PAGE_BYTES);
    if (status != IMU_STATUS_OK)
    {
        s_v1_runtime_wip_polled_clear = false;
        s_v1_last_pump_result = EXTERNAL_FLASH_CAPTURE_PUMP_ERROR;
        v1_mark_write_error(addr);
        return false;
    }

    s_v1_write_page_index++;
    s_v1_flash_stats.flash_pages_written++;
    s_v1_flash_stats.flash_bytes_written += V1_FLASH_PAGE_BYTES;
    s_v1_runtime_wip_polled_clear = false;
    s_v1_last_runtime_wip_busy = false;
    s_v1_last_pump_result = EXTERNAL_FLASH_CAPTURE_PUMP_PAGE_ISSUED;
    return true;
}

void external_flash_capture_pump(void)
{
    (void)external_flash_capture_pump_once_bounded();
}

bool external_flash_capture_pump_once_bounded(void)
{
    return v1_pump_one_page_bounded();
}

bool external_flash_capture_is_full(void)
{
    return s_v1_full;
}

bool external_flash_capture_has_error(void)
{
    return s_v1_fatal ||
           (s_v1_flash_stats.flash_write_error_count != 0U) ||
           (s_v1_flash_stats.flash_wip_timeout_count != 0U) ||
           (s_v1_flash_stats.flash_queue_overflow_count != 0U);
}

bool external_flash_capture_has_pending_work(void)
{
    return v1_raw_queue_busy();
}

bool external_flash_capture_raw_queue_busy(void)
{
    return v1_raw_queue_busy();
}

bool external_flash_capture_queue_near_full(void)
{
    return v1_queue_near_full();
}

void external_flash_capture_get_queue_status(external_flash_capture_queue_status_t *status)
{
    uint8_t i;
    uint32_t current_rem = 0U;
    uint32_t free_count = 0U;

    if (status == NULL)
    {
        return;
    }

    memset(status, 0, sizeof(*status));
    status->queue_level = s_v1_q_count;
    status->queue_near_full = v1_queue_near_full();
    status->fill_packet_count = s_v1_fill_packet_count;
    status->total_accept_packets =
        v1_total_accept_capacity_packets(&current_rem, &free_count);
    if (!s_v1_running || s_v1_fatal || s_v1_full ||
        (s_v1_full_blocks_accepted >= V1_MAX_DATA_BLOCKS))
    {
        status->total_accept_packets = 0U;
    }
    status->current_fill_remaining_packets = current_rem;
    status->free_block_count = free_count;
    status->full_blocks_accepted = s_v1_full_blocks_accepted;
    status->write_count = s_v1_flash_stats.flash_blocks_written;
    status->write_page_index = s_v1_write_page_index;
    status->flash_op = (uint32_t)s_v1_flash_op;
    status->raw_queue_busy = v1_raw_queue_busy();
    status->normal_flash_wip = s_v1_last_runtime_wip_busy;
    status->last_pump_result = s_v1_last_pump_result;
    for (i = 0U; i < EXTERNAL_FLASH_CAPTURE_QUEUE_BLOCK_STATE_MAX; i++)
    {
        status->block_state[i] = (uint8_t)s_v1_block_state[i];
    }
}

uint32_t external_flash_capture_stop_reason(void)
{
    return s_v1_stop_reason;
}

void external_flash_capture_led_mark_sleep(void)
{
    svc_led_task_mark_sleep();
}

static void v1_clear_writer_state(void)
{
    uint8_t i;

    for (i = 0U; i < V1_BLOCK_QUEUE_DEPTH; i++)
    {
        s_v1_block_state[i] = V1_BLOCK_FREE;
        s_v1_pending_q[i] = 0U;
    }
    s_v1_q_head = 0U;
    s_v1_q_tail = 0U;
    s_v1_q_count = 0U;
    s_v1_fill_index = 0xFFU;
    s_v1_write_page_index = 0U;
    s_v1_runtime_wip_polled_clear = false;
    s_v1_fill_packet_count = 0U;
#if ZY100_RUNTIME_STATS_LOG_ENABLE
    s_v1_cap_log_valid = false;
    s_v1_cap_log_current_rem = 0U;
    s_v1_cap_log_total_accept = 0U;
    s_v1_cap_log_free_count = 0U;
    s_v1_cap_log_fill = 0U;
#endif
    s_v1_running = false;
    s_v1_prepared = false;
    s_v1_full = false;
    s_v1_stop_reason = 0U;
    s_v1_capture_start_ms = 0ULL;
    if ((s_v1_flash_op == V1_FLASH_OP_CAPTURE_WRITE) ||
        (s_v1_flash_op == V1_FLASH_OP_FLUSH) ||
        (s_v1_flash_op == V1_FLASH_OP_VERIFY_WRITTEN))
    {
        v1_flash_set_op(V1_FLASH_OP_IDLE);
    }
}

static void v1_abort_key_sleep(void)
{
    imu_status_t status;

    s_v1_running = false;
    status = gd25q32e_wait_while_busy(GD25Q32E_PAGE_PROGRAM_TIMEOUT_MS);
    if (status != IMU_STATUS_OK)
    {
        if (s_v1_flash_stats.flash_wip_timeout_count < 0xFFFFFFFFUL)
        {
            s_v1_flash_stats.flash_wip_timeout_count++;
        }
        DBG_DIRECT("[ERR][FLASH] abort_wip status=%u", (uint32_t)status);
    }

    /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
    spi_bus_force_idle();
    imu_bsp_cs_high();
    (void)imu_bsp_flash_cs_hold_high();

#if IMU_RUNTIME_VERBOSE_LOG_ENABLE
    DBG_DIRECT("[V1_ABORT] key_sleep pk=%u blk=%u",
               s_v1_packets_stored,
               s_v1_flash_stats.flash_blocks_written);
#endif
}

static bool v1_flush_pending(void)
{
    bool ok = true;

    if ((s_v1_q_count != 0U) || (s_v1_write_page_index != 0U))
    {
        DBG_DIRECT("[FLASH_FINALIZE] flush_begin q=%u page=%u packets=%u bytes=%u",
                   (uint32_t)s_v1_q_count,
                   (uint32_t)s_v1_write_page_index,
                   s_v1_packets_stored,
                   s_v1_flash_stats.flash_bytes_written);
    }

    v1_flash_set_op(V1_FLASH_OP_FLUSH);
    while ((s_v1_q_count != 0U) && !s_v1_fatal)
    {
        if (!v1_pump_one_page(true))
        {
            ok = false;
            break;
        }
    }

    if (!s_v1_fatal)
    {
#if (IMU_CAPTURE_PROGRESS_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE) && ZY100_LOG_FLASH_VERBOSE
        DBG_DIRECT("[V1_F] flush_ok blk=%u", s_v1_flash_stats.flash_blocks_written);
#endif
    }
    if (s_v1_flash_op == V1_FLASH_OP_FLUSH)
    {
        v1_flash_set_op(V1_FLASH_OP_IDLE);
    }

    DBG_DIRECT("[FLASH_FINALIZE] flush_end ok=%u blocks=%u packets=%u bytes=%u q=%u page=%u",
               ok && !s_v1_fatal ? 1U : 0U,
               s_v1_flash_stats.flash_blocks_written,
               s_v1_packets_stored,
               s_v1_flash_stats.flash_bytes_written,
               (uint32_t)s_v1_q_count,
               (uint32_t)s_v1_write_page_index);
    return ok && !s_v1_fatal;
}

static bool v1_verify_written_blocks(void)
{
    uint32_t seq;
    uint32_t total_packets = 0U;
    uint32_t valid_bytes;
    uint32_t crc;
    v1_flash_imu_block_t *block = &s_v1_verify_block;
    uint16_t page;
    uint32_t addr;

    v1_flash_set_op(V1_FLASH_OP_VERIFY_WRITTEN);
    for (seq = 0U; seq < s_v1_flash_stats.flash_blocks_written; seq++)
    {
        if (s_v1_sleep_abort_requested)
        {
            v1_flash_set_op(V1_FLASH_OP_IDLE);
            return false;
        }
        addr = V1_DATA_START_ADDR + (seq * V1_DATA_BLOCK_BYTES);
        for (page = 0U; page < (V1_DATA_BLOCK_BYTES / V1_VERIFY_BUF_BYTES); page++)
        {
            if (s_v1_sleep_abort_requested)
            {
                v1_flash_set_op(V1_FLASH_OP_IDLE);
                return false;
            }
            if (gd25q32e_read(addr + ((uint32_t)page * V1_VERIFY_BUF_BYTES),
                              &((uint8_t *)block)[page * V1_VERIFY_BUF_BYTES],
                              V1_VERIFY_BUF_BYTES) != IMU_STATUS_OK)
            {
                s_v1_flash_stats.flash_verify_error_count++;
                DBG_DIRECT("[ERR][FLASH] verify_failed seq=%u", seq);
                v1_flash_set_op(V1_FLASH_OP_IDLE);
                return false;
            }
        }

        if (block->magic != V1_BLOCK_MAGIC)
        {
            s_v1_flash_stats.flash_verify_error_count++;
            DBG_DIRECT("[ERR][FLASH] verify_failed seq=%u", seq);
            v1_flash_set_op(V1_FLASH_OP_IDLE);
            return false;
        }
        if (block->block_seq != seq)
        {
            s_v1_flash_stats.flash_seq_error_count++;
            DBG_DIRECT("[ERR][FLASH] verify_failed seq=%u", seq);
            v1_flash_set_op(V1_FLASH_OP_IDLE);
            return false;
        }
        if ((block->packet_count == 0U) || (block->packet_count > V1_PACKETS_PER_BLOCK))
        {
            s_v1_flash_stats.flash_verify_error_count++;
            DBG_DIRECT("[ERR][FLASH] verify_failed seq=%u", seq);
            v1_flash_set_op(V1_FLASH_OP_IDLE);
            return false;
        }
        if ((block->packet_count != V1_PACKETS_PER_BLOCK) &&
            ((block->flags & (V1_BLOCK_FLAG_PARTIAL | V1_BLOCK_FLAG_FINAL)) !=
             (V1_BLOCK_FLAG_PARTIAL | V1_BLOCK_FLAG_FINAL)))
        {
            s_v1_flash_stats.flash_verify_error_count++;
            DBG_DIRECT("[ERR][FLASH] verify_failed seq=%u", seq);
            v1_flash_set_op(V1_FLASH_OP_IDLE);
            return false;
        }

        valid_bytes = block->packet_count * V1_IMU_PACKET_BYTES;
        crc = v1_crc32(block->payload, valid_bytes);
        if (crc != block->payload_crc32)
        {
            s_v1_flash_stats.flash_crc_error_count++;
            DBG_DIRECT("[ERR][FLASH] verify_failed seq=%u", seq);
            v1_flash_set_op(V1_FLASH_OP_IDLE);
            return false;
        }

        total_packets += block->packet_count;
        s_v1_flash_stats.flash_blocks_verified++;
    }

    s_v1_flash_stats.flash_packets_verified = total_packets;
    if (total_packets != s_v1_packets_stored)
    {
        s_v1_flash_stats.flash_verify_error_count++;
        DBG_DIRECT("[ERR][FLASH] verify_failed seq=%u", s_v1_flash_stats.flash_blocks_written);
        v1_flash_set_op(V1_FLASH_OP_IDLE);
        return false;
    }

#if (IMU_CAPTURE_PROGRESS_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE) && ZY100_LOG_FLASH_VERBOSE
    DBG_DIRECT("[V1_F] verify_ok");
#endif
    v1_flash_set_op(V1_FLASH_OP_IDLE);
    return true;
}

static const char *v1_stop_reason_name(uint32_t stop_reason)
{
    switch (stop_reason)
    {
    case V1_STOP_KEY_SLEEP:
        return "button";
    case V1_STOP_FLASH_95_PERCENT:
        return "full";
    case V1_STOP_FATAL:
        return "error";
    case 0U:
    default:
        return "normal";
    }
}

static uint32_t v1_capture_duration_s(void)
{
    uint32_t elapsed_ms;

    if (s_v1_capture_start_ms == 0ULL)
    {
        return 0U;
    }

    elapsed_ms = v1_flash_elapsed_ms_u32(s_v1_capture_start_ms);
    return elapsed_ms / 1000U;
}

static void v1_print_log_config(void)
{
#if IMU_CAPTURE_SUMMARY_LOG_ENABLE && ZY100_LOG_VERBOSE_DEFAULT
    DBG_DIRECT("[LOG_CFG] runtime=%u marker=%u axis=%u wom=%u mreg=%u progress=%u summary=%u",
               (uint32_t)IMU_RUNTIME_VERBOSE_LOG_ENABLE,
               (uint32_t)IMU_MARKER_RUNTIME_LOG_ENABLE,
               (uint32_t)IMU_AXIS_DIAG_RUNTIME_LOG_ENABLE,
               (uint32_t)IMU_WOM_RUNTIME_LOG_ENABLE,
               (uint32_t)IMU_MREG_DIAG_LOG_ENABLE,
               (uint32_t)IMU_CAPTURE_PROGRESS_LOG_ENABLE,
               (uint32_t)IMU_CAPTURE_SUMMARY_LOG_ENABLE);
#endif
}

static void v1_print_marker_summary(void)
{
#if IMU_CAPTURE_SUMMARY_LOG_ENABLE && ZY100_FIFO_MARKER_SUMMARY_ENABLE && ZY100_LOG_VERBOSE_DEFAULT
    imu_sample_marker_stats_t marker_stats;

    imu_sample_marker_get_stats(&marker_stats);
    DBG_DIRECT("[MARKER_SUMMARY_A] marker_count=%u pre_swing_trigger_count=%u",
               marker_stats.marker_count,
               marker_stats.pre_swing_trigger_count);
    DBG_DIRECT("[MARKER_SUMMARY_B] marker_dropped_count=%u marker_table_write_error_count=%u",
               marker_stats.marker_dropped_count,
               marker_stats.flash_marker_write_error_count);
#endif
}

static void v1_print_summary(uint32_t stop_reason, const external_flash_capture_imu_stats_t *imu_stats)
{
#if IMU_CAPTURE_SUMMARY_LOG_ENABLE
    const uint32_t bad_frame_count = (imu_stats != NULL) ? imu_stats->bad_header_count : 0U;
    const uint32_t fifo_lost_count = (imu_stats != NULL) ? (uint32_t)imu_stats->fifo_lost_pkt_count : 0U;
    const uint32_t read_error_count = (imu_stats != NULL) ? imu_stats->read_error_count : 0U;
    const uint32_t overflow_count = (imu_stats != NULL) ? imu_stats->fifo_full_count : 0U;
#if ZY100_LOG_VERBOSE_DEFAULT
    const uint32_t max_loop_gap_ms = (imu_stats != NULL) ? imu_stats->max_loop_gap_ms : 0U;
    const uint32_t loop_gap_over_10ms_count =
        (imu_stats != NULL) ? imu_stats->loop_gap_over_10ms_count : 0U;
    const uint32_t loop_gap_over_20ms_count =
        (imu_stats != NULL) ? imu_stats->loop_gap_over_20ms_count : 0U;
    const uint32_t queue_max_level = (imu_stats != NULL) ? imu_stats->queue_max_level : 0U;
    const uint32_t max_fifo_count = (imu_stats != NULL) ? imu_stats->max_fifo_count : 0U;
#endif

    DBG_DIRECT("[CAP_SUMMARY] reason=%s duration_s=%u packets=%u blocks=%u bytes=%u imu_err=%u flash_err=%u verify_err=%u",
               v1_stop_reason_name(stop_reason),
               v1_capture_duration_s(),
               s_v1_packets_stored,
               s_v1_flash_stats.flash_blocks_written,
               s_v1_flash_stats.flash_bytes_written,
               bad_frame_count + fifo_lost_count + read_error_count + overflow_count,
               s_v1_flash_stats.flash_write_error_count,
               s_v1_flash_stats.flash_verify_error_count);
#if ZY100_LOG_VERBOSE_DEFAULT
    v1_print_log_config();
    DBG_DIRECT("[CAP_SUMMARY_C] max_loop_gap_ms=%u gap_over_10ms=%u gap_over_20ms=%u queue_max_level=%u max_fifo_count=%u",
               max_loop_gap_ms,
               loop_gap_over_10ms_count,
               loop_gap_over_20ms_count,
               queue_max_level,
               max_fifo_count);
    DBG_DIRECT("[IMU_SUMMARY_A] bad_frame_count=%u fifo_lost_count=%u",
               bad_frame_count,
               fifo_lost_count);
    DBG_DIRECT("[IMU_SUMMARY_B] read_error_count=%u overflow_count=%u",
               read_error_count,
               overflow_count);
    /* write_count is the captured raw block write count; page writes are kept in flash_pages_written. */
    DBG_DIRECT("[FLASH_SUMMARY_A] write_count=%u write_error_count=%u",
               s_v1_flash_stats.flash_blocks_written,
               s_v1_flash_stats.flash_write_error_count);
    DBG_DIRECT("[FLASH_SUMMARY_B] verify_error_count=%u used_bytes=%u",
               s_v1_flash_stats.flash_verify_error_count,
               s_v1_flash_stats.flash_bytes_written);
    v1_print_marker_summary();
#if ZY100_FIFO_MARKER_SUMMARY_ENABLE
    imu_pre_trigger_marker_log_capture_summary();
#endif
#endif
#else
    IMU_UNUSED(stop_reason);
    IMU_UNUSED(imu_stats);
#endif
}

void external_flash_capture_finalize(uint32_t stop_reason,
                                     const external_flash_capture_imu_stats_t *imu_stats)
{
    uint32_t final_status;
    bool flush_ok;
    bool header_ok;
    imu_status_t status;

    if (s_v1_stop_reason != 0U)
    {
        stop_reason = s_v1_stop_reason;
    }
    if (s_v1_fatal)
    {
        stop_reason = V1_STOP_FATAL;
    }

    DBG_DIRECT("[CAP] stop reason=%s", v1_stop_reason_name(stop_reason));
    s_v1_running = false;

    if (stop_reason == V1_STOP_KEY_SLEEP)
    {
        v1_abort_key_sleep();
        v1_print_summary(stop_reason, imu_stats);
        v1_clear_writer_state();
        return;
    }

    if (stop_reason == V1_STOP_FATAL)
    {
        (void)gd25q32e_wait_while_busy(GD25Q32E_PAGE_PROGRAM_TIMEOUT_MS);
        /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
        spi_bus_force_idle();
        imu_bsp_cs_high();
        (void)imu_bsp_flash_cs_hold_high();
        DBG_DIRECT("[ERR][FLASH] fatal_stop");
        v1_print_summary(stop_reason, imu_stats);
#if (IMU_RUNTIME_VERBOSE_LOG_ENABLE || IMU_MARKER_RUNTIME_LOG_ENABLE) && \
    (ZY100_FIFO_MARKER_SUMMARY_ENABLE || ZY100_FIFO_MARKER_DEBUG_ENABLE)
        imu_pre_trigger_marker_log_config();
        imu_pre_trigger_marker_log_stats();
        imu_sample_marker_log_config();
        imu_sample_marker_log_stats();
        imu_sample_marker_log_markers();
#endif
        v1_clear_writer_state();
        return;
    }

    status = gd25q32e_init();
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[ERR][FLASH] finalize_init_failed status=%u", (uint32_t)status);
        v1_mark_write_error(0U);
    }

    flush_ok = v1_flush_pending();
    if (!flush_ok)
    {
        DBG_DIRECT("[ERR][FLASH] finalize_flush_failed op=%u op_name=%s q=%u page=%u written=%u accepted=%u bytes=%u",
                   (uint32_t)s_v1_flash_op,
                   external_flash_capture_op_name(s_v1_flash_op),
                   (uint32_t)s_v1_q_count,
                   (uint32_t)s_v1_write_page_index,
                   s_v1_flash_stats.flash_blocks_written,
                   s_v1_full_blocks_accepted,
                   s_v1_flash_stats.flash_bytes_written);
    }

    if (!s_v1_fatal)
    {
        (void)v1_verify_written_blocks();
    }
    if (s_v1_sleep_abort_requested)
    {
        /* non-runtime SPI: no RT marker/FIFO scheduler concurrency */
        spi_bus_force_idle();
        imu_bsp_cs_high();
        (void)imu_bsp_flash_cs_hold_high();
        v1_print_summary(V1_STOP_KEY_SLEEP, imu_stats);
#if IMU_RUNTIME_VERBOSE_LOG_ENABLE
        DBG_DIRECT("[V1_ABORT] key_sleep pk=%u blk=%u",
                   s_v1_packets_stored,
                   s_v1_flash_stats.flash_blocks_written);
#endif
        v1_clear_writer_state();
        v1_flash_set_op(V1_FLASH_OP_IDLE);
        return;
    }

#if ZY100_FIFO_MARKER_FLASH_ENABLE
    v1_marker_write_table();
#endif
    imu_rt_marker_log_final();

    if (!v1_flash_wait_for_finalize_idle("before_session_header"))
    {
        DBG_DIRECT("[ERR][FLASH] finalize_idle_before_header_failed op=%u op_name=%s",
                   (uint32_t)s_v1_flash_op,
                   external_flash_capture_op_name(s_v1_flash_op));
    }

    if ((stop_reason == V1_STOP_FATAL) ||
        s_v1_fatal ||
        (s_v1_flash_stats.flash_verify_error_count != 0U) ||
        (s_v1_flash_stats.flash_seq_error_count != 0U) ||
        (s_v1_flash_stats.flash_crc_error_count != 0U))
    {
        final_status = V1_SESSION_FAILED;
    }
    else
    {
        final_status = V1_SESSION_COMPLETE;
    }

    header_ok = v1_write_session_header(final_status, stop_reason, imu_stats);
    if (!header_ok)
    {
        DBG_DIRECT("[ERR][FLASH] finalize_header_failed status=%u stop=%u op=%u op_name=%s q=%u page=%u written=%u accepted=%u bytes=%u",
                   final_status,
                   stop_reason,
                   (uint32_t)s_v1_flash_op,
                   external_flash_capture_op_name(s_v1_flash_op),
                   (uint32_t)s_v1_q_count,
                   (uint32_t)s_v1_write_page_index,
                   s_v1_flash_stats.flash_blocks_written,
                   s_v1_full_blocks_accepted,
                   s_v1_flash_stats.flash_bytes_written);
    }
    v1_print_summary(stop_reason, imu_stats);
#if (IMU_RUNTIME_VERBOSE_LOG_ENABLE || IMU_MARKER_RUNTIME_LOG_ENABLE) && \
    (ZY100_FIFO_MARKER_SUMMARY_ENABLE || ZY100_FIFO_MARKER_DEBUG_ENABLE)
    imu_pre_trigger_marker_log_config();
    imu_pre_trigger_marker_log_stats();
    imu_sample_marker_log_config();
    imu_sample_marker_log_stats();
    imu_sample_marker_log_markers();
#endif
    (void)external_flash_capture_led_red_on();
}

#endif
