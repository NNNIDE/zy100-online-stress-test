#include "zy100_offline_v2_imu_session.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS_API.h"
#include "os_sched.h"
#include "os_sync.h"
#include "rtl876x_spi.h"
#include "trace.h"

#include "../app_flags.h"
#include "../bsp/imu_bsp.h"
#include "../bsp/bsp_shared_spi.h"
#include "../common/imu_common.h"
#include "../driver/icm53611_driver.h"
#include "../driver/icm53611_reg.h"
#include "zy100_clock_config.h"
#include "zy100_capture_start_diag.h"
#include "zy100_offline_v2_capture.h"
#include "zy100_offline_v2_config.h"
#include "zy100_offline_v2_imu_session_internal.h"

#define OFFLINE_V2_IMU_NOTIFY_START BIT(0)
#define OFFLINE_V2_IMU_NOTIFY_TICK  BIT(1)
#define OFFLINE_V2_IMU_NOTIFY_STOP  BIT(2)
#define OFFLINE_V2_IMU_NOTIFY_MASK  (OFFLINE_V2_IMU_NOTIFY_START | \
                                     OFFLINE_V2_IMU_NOTIFY_TICK | \
                                     OFFLINE_V2_IMU_NOTIFY_STOP)
#define OFFLINE_V2_IMU_HEADER_MSG            BIT(7)
#define OFFLINE_V2_IMU_HEADER_ACCEL          BIT(6)
#define OFFLINE_V2_IMU_HEADER_GYRO           BIT(5)
#define OFFLINE_V2_IMU_HEADER_20             BIT(4)
#define OFFLINE_V2_IMU_HEADER_TIMESTAMP_MASK (BIT(3) | BIT(2))
#define OFFLINE_V2_IMU_HEADER_TIMESTAMP_ODR  BIT(3)
#define OFFLINE_V2_IMU_TMST_HIGH_INDEX       14U
#define OFFLINE_V2_IMU_TMST_LOW_INDEX        15U
#define OFFLINE_V2_IMU_READ_BUFFER_BYTES \
    (ZY100_OFFLINE_V2_IMU_EMERGENCY_BURST_PACKETS * \
     ZY100_OFFLINE_V2_IMU_FIFO_PACKET_BYTES)
#define OFFLINE_V2_IMU_MAX_DUE 8U

typedef enum
{
    OFFLINE_V2_IMU_SERVICE_OK = 0U,
    OFFLINE_V2_IMU_SERVICE_DEFER_FLASH_BUSY,
    OFFLINE_V2_IMU_SERVICE_FATAL,
} offline_v2_imu_service_result_t;

#if (OFFLINE_V2_IMU_READ_BUFFER_BYTES > \
     ZY100_OFFLINE_V2_IMU_FIFO_RESCUE_FLOOR_BYTES)
#error "Offline V2 read buffer must stay within its rescue floor"
#endif

#if (ZY100_OFFLINE_V2_IMU_TASK_PRIORITY_OFFSET < configMAX_PRIORITIES)
#define OFFLINE_V2_IMU_TASK_PRIORITY \
    (tskIDLE_PRIORITY + ZY100_OFFLINE_V2_IMU_TASK_PRIORITY_OFFSET)
#else
#define OFFLINE_V2_IMU_TASK_PRIORITY (configMAX_PRIORITIES - 1U)
#endif

typedef struct
{
    TaskHandle_t task;
    zy100_offline_v2_imu_state_t state;
    zy100_offline_v2_imu_stop_reason_t stop_reason;
    zy100_offline_v2_imu_cause_t cause;
    uint32_t completion_generation;
    uint32_t accepted_packets;
    uint32_t service_count;
    uint32_t emergency_count;
    uint32_t read_error_count;
    uint32_t bad_header_count;
    uint32_t fifo_full_count;
    uint32_t fifo_lost_count;
    uint32_t last_progress_ms;
    zy100_offline_v2_spi_retry_policy_t spi_retry;
    uint32_t prepare_busy_start;
    uint16_t last_fifo_count;
    uint16_t max_fifo_count;
    volatile uint32_t timer_due;
    bool initialized;
    bool timer_running;
    bool stop_requested;
    bool completion_pending;
    bool prepared;
} offline_v2_imu_runtime_t;

static offline_v2_imu_runtime_t s_imu;
static uint8_t s_fifo_read_buffer[OFFLINE_V2_IMU_READ_BUFFER_BYTES];

static uint32_t offline_v2_imu_time_us(void)
{
    return (uint32_t)imu_bsp_local_timestamp_us();
}

static offline_v2_imu_service_result_t offline_v2_imu_status_result(
    imu_status_t status)
{
    zy100_offline_v2_spi_retry_result_t retry_result;

    if (status == IMU_STATUS_OK)
    {
        return OFFLINE_V2_IMU_SERVICE_OK;
    }
    if (status != IMU_STATUS_FLASH_BUSY)
    {
        return OFFLINE_V2_IMU_SERVICE_FATAL;
    }
    retry_result = zy100_offline_v2_spi_retry_on_bus_error(
        &s_imu.spi_retry,
        true, /* Captured by the failed acquisition, before any transfer. */
        offline_v2_imu_time_us(),
        ZY100_OFFLINE_V2_SPI_BUSY_RETRY_BUDGET_US);
    return (retry_result == ZY100_OFFLINE_V2_SPI_RETRY_DEFER) ?
        OFFLINE_V2_IMU_SERVICE_DEFER_FLASH_BUSY :
        OFFLINE_V2_IMU_SERVICE_FATAL;
}

static void offline_v2_imu_complete_service(void)
{
    zy100_offline_v2_spi_retry_complete(&s_imu.spi_retry,
                                         offline_v2_imu_time_us());
}

static void offline_v2_imu_restore_due(uint32_t due)
{
    uint32_t lock_state = os_lock();

    s_imu.timer_due = zy100_offline_v2_due_merge(s_imu.timer_due,
                                                  due,
                                                  OFFLINE_V2_IMU_MAX_DUE);
    os_unlock(lock_state);
}

static bool offline_v2_imu_reason_is_graceful(
    zy100_offline_v2_imu_stop_reason_t reason)
{
    return (reason == ZY100_OFFLINE_V2_IMU_STOP_USER) ||
           (reason == ZY100_OFFLINE_V2_IMU_STOP_LOW_BATTERY) ||
           (reason == ZY100_OFFLINE_V2_IMU_STOP_FLASH_FULL) ||
           (reason == ZY100_OFFLINE_V2_IMU_STOP_SYSTEM_SHUTDOWN);
}

static bool offline_v2_imu_header_valid(uint8_t header)
{
    if ((header & OFFLINE_V2_IMU_HEADER_MSG) != 0U)
    {
        return false;
    }
    if ((header & (OFFLINE_V2_IMU_HEADER_ACCEL |
                   OFFLINE_V2_IMU_HEADER_GYRO)) !=
        (OFFLINE_V2_IMU_HEADER_ACCEL | OFFLINE_V2_IMU_HEADER_GYRO))
    {
        return false;
    }
    if ((header & OFFLINE_V2_IMU_HEADER_20) != 0U)
    {
        return false;
    }
    return ((header & OFFLINE_V2_IMU_HEADER_TIMESTAMP_MASK) ==
            OFFLINE_V2_IMU_HEADER_TIMESTAMP_ODR);
}

static void offline_v2_imu_latch_cause(
    zy100_offline_v2_imu_stop_reason_t reason,
    zy100_offline_v2_imu_origin_t origin,
    uint32_t detail,
    uint16_t fifo_count,
    uint16_t request_len,
    uint16_t actual_len,
    uint16_t buffer_offset,
    uint8_t header,
    uint8_t int_status)
{
    bool first_cause = false;
    uint32_t lock_state = os_lock();

    if (!s_imu.cause.valid)
    {
        first_cause = true;
        s_imu.cause.valid = true;
        s_imu.cause.reason = reason;
        s_imu.cause.origin = origin;
        s_imu.cause.detail = detail;
        s_imu.cause.time_ms = zy100_os_time_ms();
        s_imu.cause.accepted_packets = s_imu.accepted_packets;
        s_imu.cause.fifo_count = fifo_count;
        s_imu.cause.lost_count = (uint16_t)s_imu.fifo_lost_count;
        s_imu.cause.request_len = request_len;
        s_imu.cause.actual_len = actual_len;
        s_imu.cause.buffer_offset = buffer_offset;
        s_imu.cause.header = header;
        s_imu.cause.int_status = int_status;
    }
    if (s_imu.stop_reason == ZY100_OFFLINE_V2_IMU_STOP_NONE)
    {
        s_imu.stop_reason = reason;
    }
    s_imu.stop_requested = true;
    if ((s_imu.state == ZY100_OFFLINE_V2_IMU_RUNNING) ||
        (s_imu.state == ZY100_OFFLINE_V2_IMU_START_PENDING))
    {
        s_imu.state = ZY100_OFFLINE_V2_IMU_STOPPING;
    }
    os_unlock(lock_state);
    if (first_cause)
    {
        if (reason == ZY100_OFFLINE_V2_IMU_STOP_USER)
        {
            ZY100_LOG_EVENT("[OFF_IMU_STOP] reason=%u origin=%u detail=%lu",
                            (uint32_t)reason, (uint32_t)origin,
                            (unsigned long)detail);
        }
        else
        {
            ZY100_LOG_ERROR("[OFF_IMU_FAIL] reason=%u origin=%u detail=%lu",
                            (uint32_t)reason, (uint32_t)origin,
                            (unsigned long)detail);
        }
    }
}

static void offline_v2_imu_timer_notify_from_isr(void)
{
    if (s_imu.timer_due < OFFLINE_V2_IMU_MAX_DUE)
    {
        s_imu.timer_due++;
    }
    if (s_imu.task != NULL)
    {
        (void)xTaskNotifyFromISR(s_imu.task,
                                OFFLINE_V2_IMU_NOTIFY_TICK,
                                eSetBits,
                                NULL);
    }
}

static uint32_t offline_v2_imu_take_due(void)
{
    uint32_t due;
    uint32_t lock_state = os_lock();

    due = s_imu.timer_due;
    s_imu.timer_due = 0U;
    os_unlock(lock_state);
    return due;
}

static void offline_v2_imu_timer_stop(void)
{
    if (s_imu.timer_running ||
        (imu_bsp_ois_tick_timer_get_owner() ==
         IMU_BSP_OIS_TICK_TIMER_OWNER_OFFLINE_V2_FIFO))
    {
        imu_bsp_ois_tick_timer_stop();
        imu_bsp_ois_tick_timer_register_irq_callback(NULL);
        imu_bsp_ois_tick_timer_release(
            IMU_BSP_OIS_TICK_TIMER_OWNER_OFFLINE_V2_FIFO);
    }
    s_imu.timer_running = false;
    s_imu.timer_due = 0U;
}

static imu_status_t offline_v2_imu_timer_start(void)
{
    imu_status_t status;

    status = imu_bsp_ois_tick_timer_acquire(
        IMU_BSP_OIS_TICK_TIMER_OWNER_OFFLINE_V2_FIFO);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    s_imu.timer_due = 0U;
    imu_bsp_ois_tick_timer_register_irq_callback(
        offline_v2_imu_timer_notify_from_isr);
    status = imu_bsp_ois_tick_timer_config(
        ZY100_OFFLINE_V2_IMU_SERVICE_PERIOD_US);
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_ois_tick_timer_start();
    }
    if (status != IMU_STATUS_OK)
    {
        imu_bsp_ois_tick_timer_register_irq_callback(NULL);
        imu_bsp_ois_tick_timer_release(
            IMU_BSP_OIS_TICK_TIMER_OWNER_OFFLINE_V2_FIFO);
        return status;
    }
    s_imu.timer_running = true;
    return IMU_STATUS_OK;
}

static void offline_v2_imu_fill_config(icm53611_cfg_t *imu_cfg,
                                       icm53611_fifo_cfg_t *fifo_cfg)
{
    memset(imu_cfg, 0, sizeof(*imu_cfg));
    imu_cfg->pwr_mgmt0 = (ICM53611_PWR_MGMT0_ACCEL_MODE_LN |
                          ICM53611_PWR_MGMT0_GYRO_MODE_LN);
    imu_cfg->gyro_config0 = (ICM53611_GYRO_FSR_2000DPS |
                             ICM53611_GYRO_ODR_800HZ);
    imu_cfg->accel_config0 = (ICM53611_ACCEL_FSR_16G |
                              ICM53611_ACCEL_ODR_800HZ);
    imu_cfg->gyro_config1 = ICM53611_GYRO_CONFIG1_DEFAULT;
    imu_cfg->accel_config1 = ICM53611_ACCEL_CONFIG1_DEFAULT;

    memset(fifo_cfg, 0, sizeof(*fifo_cfg));
    fifo_cfg->enable = true;
    fifo_cfg->timestamp_enable = true;
    fifo_cfg->accel_enable = true;
    fifo_cfg->gyro_enable = true;
    fifo_cfg->hires_enable = false;
    fifo_cfg->fsync_timestamp_enable = false;
    fifo_cfg->watermark = ZY100_OFFLINE_V2_IMU_FIFO_WATERMARK_BYTES;
}

static void offline_v2_imu_disable_fifo(void)
{
    icm53611_fifo_cfg_t fifo_cfg;

    memset(&fifo_cfg, 0, sizeof(fifo_cfg));
    offline_v2_imu_timer_stop();
    imu_status_t cleanup_status = icm53611_fifo_config(&fifo_cfg);
    if (cleanup_status != IMU_STATUS_OK)
        ZY100_LOG_ERROR("[CAP_CLEAN] mode=0 step=1 st=%u", (uint32_t)cleanup_status);
    imu_bsp_int_register_irq_callback(NULL);
    imu_bsp_int_clear_pending();
}

static bool offline_v2_imu_start_hardware(void)
{
    icm53611_cfg_t imu_cfg;
    icm53611_fifo_cfg_t fifo_cfg;
    imu_status_t status;
    uint16_t lost_count = 0U;
    icm53611_capture_result_t result;

    offline_v2_imu_fill_config(&imu_cfg, &fifo_cfg);
    status = icm53611_prepare_capture(&imu_cfg, &fifo_cfg, &result);
    if (status == IMU_STATUS_OK)
    {
        result.stage = ICM53611_CAPTURE_LOST_COUNT;
        status = icm53611_fifo_get_lost_count_ex(&lost_count);
        if (status == IMU_STATUS_OK && lost_count != 0U) status = IMU_STATUS_VERIFY_FAILED;
    }
    if (status == IMU_STATUS_OK)
    {
        result.stage = ICM53611_CAPTURE_TIMER;
        status = offline_v2_imu_timer_start();
    }
    if (status != IMU_STATUS_OK)
    {
        result.status = (uint8_t)status;
        zy100_capture_start_log(0U, s_imu.completion_generation, &result);
        offline_v2_imu_latch_cause(
            ZY100_OFFLINE_V2_IMU_STOP_SENSOR_START,
            ZY100_OFFLINE_V2_IMU_ORIGIN_PREPARE,
            (status != IMU_STATUS_OK) ? (uint32_t)imu_status_legacy(status) :
            (uint32_t)lost_count,
            0U, 0U, 0U, 0U, 0U, 0U);
        offline_v2_imu_disable_fifo();
        return false;
    }
    ZY100_LOG_EVENT("[CAP_READY] mode=0 id=%lu", (unsigned long)s_imu.completion_generation);
    s_imu.state = ZY100_OFFLINE_V2_IMU_RUNNING;
    s_imu.last_progress_ms = zy100_os_time_ms();
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][IMU] start hz=%u period_us=%u wm=%u emergency=%u",
        (uint32_t)ZY100_OFFLINE_V2_IMU_SAMPLE_RATE_HZ,
        (uint32_t)ZY100_OFFLINE_V2_IMU_SERVICE_PERIOD_US,
        (uint32_t)ZY100_OFFLINE_V2_IMU_FIFO_WATERMARK_BYTES,
        (uint32_t)ZY100_OFFLINE_V2_IMU_FIFO_EMERGENCY_BYTES);
    return true;
}

static offline_v2_imu_service_result_t offline_v2_imu_process_read(
    uint16_t fifo_count,
    uint16_t request_len,
    zy100_offline_v2_imu_origin_t origin)
{
    offline_v2_imu_service_result_t result;
    imu_status_t status;
    uint16_t offset;

    status = icm53611_fifo_read_ex(s_fifo_read_buffer, request_len);
    if (status != IMU_STATUS_OK)
    {
        result = offline_v2_imu_status_result(status);
        if (result == OFFLINE_V2_IMU_SERVICE_DEFER_FLASH_BUSY)
        {
            return result;
        }
        s_imu.read_error_count++;
        offline_v2_imu_latch_cause(
            ZY100_OFFLINE_V2_IMU_STOP_FIFO_READ,
            origin,
            (uint32_t)imu_status_legacy(status),
            fifo_count, request_len, 0U, 0U, 0U, 0U);
        return OFFLINE_V2_IMU_SERVICE_FATAL;
    }

    for (offset = 0U; offset < request_len;
         offset = (uint16_t)(offset +
                             ZY100_OFFLINE_V2_IMU_FIFO_PACKET_BYTES))
    {
        const uint8_t *packet = &s_fifo_read_buffer[offset];
        uint16_t timestamp_raw;

        if (!offline_v2_imu_header_valid(packet[0]))
        {
            s_imu.bad_header_count++;
            offline_v2_imu_latch_cause(
                ZY100_OFFLINE_V2_IMU_STOP_FIFO_BAD_HEADER,
                ZY100_OFFLINE_V2_IMU_ORIGIN_FIFO_PACKET,
                (uint32_t)packet[0], fifo_count, request_len, request_len,
                offset, packet[0], 0U);
            return OFFLINE_V2_IMU_SERVICE_FATAL;
        }
        timestamp_raw = (uint16_t)(
            ((uint16_t)packet[OFFLINE_V2_IMU_TMST_HIGH_INDEX] << 8) |
            packet[OFFLINE_V2_IMU_TMST_LOW_INDEX]);
        if (!zy100_offline_v2_capture_push_imu_packet(packet,
                                                       timestamp_raw))
        {
            offline_v2_imu_latch_cause(
                ZY100_OFFLINE_V2_IMU_STOP_ALGORITHM,
                ZY100_OFFLINE_V2_IMU_ORIGIN_ALGORITHM,
                (uint32_t)packet[0], fifo_count, request_len, request_len,
                offset, packet[0], 0U);
            return OFFLINE_V2_IMU_SERVICE_FATAL;
        }
        if (s_imu.accepted_packets == 0U)
            ZY100_LOG_EVENT("[CAP_DATA] mode=0 id=%lu", (unsigned long)s_imu.completion_generation);
        s_imu.accepted_packets++;
        s_imu.last_progress_ms = zy100_os_time_ms();
    }
    return OFFLINE_V2_IMU_SERVICE_OK;
}

static offline_v2_imu_service_result_t
offline_v2_imu_read_status_and_count(uint16_t *fifo_count_out)
{
    offline_v2_imu_service_result_t result;
    icm53611_int_status_t int_status;
    imu_status_t status;
    uint16_t lost_count = 0U;
    uint16_t fifo_count = 0U;

    memset(&int_status, 0, sizeof(int_status));
    status = icm53611_read_int_status_ex(&int_status);
    if (status != IMU_STATUS_OK)
    {
        result = offline_v2_imu_status_result(status);
        if (result == OFFLINE_V2_IMU_SERVICE_DEFER_FLASH_BUSY)
        {
            return result;
        }
        s_imu.read_error_count++;
        offline_v2_imu_latch_cause(
            ZY100_OFFLINE_V2_IMU_STOP_FIFO_READ,
            ZY100_OFFLINE_V2_IMU_ORIGIN_FIFO_INT,
            (uint32_t)imu_status_legacy(status), 0U, 0U, 0U, 0U, 0U, 0U);
        return OFFLINE_V2_IMU_SERVICE_FATAL;
    }
    if ((int_status.int_status & ICM53611_INT_STATUS_FIFO_FULL_INT) != 0U)
    {
        s_imu.fifo_full_count++;
        offline_v2_imu_latch_cause(
            ZY100_OFFLINE_V2_IMU_STOP_FIFO_FULL,
            ZY100_OFFLINE_V2_IMU_ORIGIN_FIFO_INT,
            (uint32_t)int_status.int_status, 0U, 0U, 0U, 0U, 0U,
            int_status.int_status);
        return OFFLINE_V2_IMU_SERVICE_FATAL;
    }
    status = icm53611_fifo_get_lost_count_ex(&lost_count);
    if (status != IMU_STATUS_OK)
    {
        result = offline_v2_imu_status_result(status);
        if (result == OFFLINE_V2_IMU_SERVICE_DEFER_FLASH_BUSY)
        {
            return result;
        }
        s_imu.read_error_count++;
        offline_v2_imu_latch_cause(
            ZY100_OFFLINE_V2_IMU_STOP_FIFO_READ,
            ZY100_OFFLINE_V2_IMU_ORIGIN_FIFO_LOST,
            (uint32_t)imu_status_legacy(status), 0U, 0U, 0U, 0U, 0U,
            int_status.int_status);
        return OFFLINE_V2_IMU_SERVICE_FATAL;
    }
    if (lost_count != 0U)
    {
        s_imu.fifo_lost_count = lost_count;
        offline_v2_imu_latch_cause(
            ZY100_OFFLINE_V2_IMU_STOP_FIFO_LOST,
            ZY100_OFFLINE_V2_IMU_ORIGIN_FIFO_LOST,
            (uint32_t)lost_count, 0U, 0U, 0U, 0U, 0U,
            int_status.int_status);
        return OFFLINE_V2_IMU_SERVICE_FATAL;
    }
    status = icm53611_fifo_get_count_ex(&fifo_count);
    if (status != IMU_STATUS_OK)
    {
        result = offline_v2_imu_status_result(status);
        if (result == OFFLINE_V2_IMU_SERVICE_DEFER_FLASH_BUSY)
        {
            return result;
        }
        s_imu.read_error_count++;
        offline_v2_imu_latch_cause(
            ZY100_OFFLINE_V2_IMU_STOP_FIFO_READ,
            ZY100_OFFLINE_V2_IMU_ORIGIN_FIFO_COUNT,
            (uint32_t)imu_status_legacy(status), 0U, 0U, 0U, 0U, 0U,
            int_status.int_status);
        return OFFLINE_V2_IMU_SERVICE_FATAL;
    }
    s_imu.last_fifo_count = fifo_count;
    if (fifo_count > s_imu.max_fifo_count)
    {
        s_imu.max_fifo_count = fifo_count;
    }
    *fifo_count_out = fifo_count;
    return OFFLINE_V2_IMU_SERVICE_OK;
}

static offline_v2_imu_service_result_t offline_v2_imu_service_once(void)
{
    offline_v2_imu_service_result_t result;
    uint8_t attempt = 0U;
    uint16_t fifo_count = 0U;

    s_imu.service_count++;
    result = offline_v2_imu_read_status_and_count(&fifo_count);
    if (result != OFFLINE_V2_IMU_SERVICE_OK)
    {
        return result;
    }
    if (fifo_count < ZY100_OFFLINE_V2_IMU_FIFO_PACKET_BYTES)
    {
        offline_v2_imu_complete_service();
        return OFFLINE_V2_IMU_SERVICE_OK;
    }

    do
    {
        uint16_t aligned = (uint16_t)(
            fifo_count &
            (uint16_t)~(ZY100_OFFLINE_V2_IMU_FIFO_PACKET_BYTES - 1U));
        uint16_t burst_packets =
            (fifo_count >= ZY100_OFFLINE_V2_IMU_FIFO_EMERGENCY_BYTES) ?
            ZY100_OFFLINE_V2_IMU_EMERGENCY_BURST_PACKETS :
            ZY100_OFFLINE_V2_IMU_NORMAL_BURST_PACKETS;
        uint16_t request_len = (uint16_t)(
            burst_packets * ZY100_OFFLINE_V2_IMU_FIFO_PACKET_BYTES);

        if (fifo_count >= ZY100_OFFLINE_V2_IMU_FIFO_EMERGENCY_BYTES)
        {
            s_imu.emergency_count++;
        }
        if (request_len > aligned)
        {
            request_len = aligned;
        }
        if (request_len == 0U)
        {
            offline_v2_imu_complete_service();
            return OFFLINE_V2_IMU_SERVICE_OK;
        }
        result = offline_v2_imu_process_read(
            fifo_count, request_len,
            ZY100_OFFLINE_V2_IMU_ORIGIN_FIFO_READ);
        if (result != OFFLINE_V2_IMU_SERVICE_OK)
        {
            return result;
        }
        attempt++;
        if ((fifo_count < ZY100_OFFLINE_V2_IMU_FIFO_WATERMARK_BYTES) ||
            (attempt >= ZY100_OFFLINE_V2_IMU_EMERGENCY_ATTEMPTS))
        {
            break;
        }
        result = offline_v2_imu_read_status_and_count(&fifo_count);
        if (result != OFFLINE_V2_IMU_SERVICE_OK)
        {
            return result;
        }
    } while (fifo_count >
             ZY100_OFFLINE_V2_IMU_FIFO_RESCUE_FLOOR_BYTES);
    offline_v2_imu_complete_service();
    return OFFLINE_V2_IMU_SERVICE_OK;
}

static bool offline_v2_imu_final_drain(void)
{
    uint32_t start_ms = zy100_os_time_ms();
    offline_v2_imu_service_result_t result;
    imu_status_t status;

    while (true)
    {
        status = icm53611_stop_ui_sensors_preserve_fifo_ex();
        result = offline_v2_imu_status_result(status);
        if (result == OFFLINE_V2_IMU_SERVICE_OK)
        {
            offline_v2_imu_complete_service();
            break;
        }
        if (result == OFFLINE_V2_IMU_SERVICE_DEFER_FLASH_BUSY)
        {
            os_delay(1U);
            continue;
        }
        offline_v2_imu_latch_cause(
            ZY100_OFFLINE_V2_IMU_STOP_FIFO_READ,
            ZY100_OFFLINE_V2_IMU_ORIGIN_FINAL_DRAIN,
            (uint32_t)imu_status_legacy(status), 0U, 0U, 0U, 0U, 0U, 0U);
        return false;
    }
    while (true)
    {
        uint16_t fifo_count = 0U;
        uint16_t aligned;
        uint16_t request_len;

        status = icm53611_fifo_get_count_ex(&fifo_count);
        if (status != IMU_STATUS_OK)
        {
            result = offline_v2_imu_status_result(status);
            if (result == OFFLINE_V2_IMU_SERVICE_DEFER_FLASH_BUSY)
            {
                os_delay(1U);
                continue;
            }
            offline_v2_imu_latch_cause(
                ZY100_OFFLINE_V2_IMU_STOP_FIFO_READ,
                ZY100_OFFLINE_V2_IMU_ORIGIN_FINAL_DRAIN,
                (uint32_t)imu_status_legacy(status), 0U, 0U, 0U, 0U, 0U, 0U);
            return false;
        }
        s_imu.last_fifo_count = fifo_count;
        if (fifo_count == 0U)
        {
            offline_v2_imu_complete_service();
            return true;
        }
        aligned = (uint16_t)(
            fifo_count &
            (uint16_t)~(ZY100_OFFLINE_V2_IMU_FIFO_PACKET_BYTES - 1U));
        if (aligned == 0U)
        {
            offline_v2_imu_latch_cause(
                ZY100_OFFLINE_V2_IMU_STOP_FIFO_READ,
                ZY100_OFFLINE_V2_IMU_ORIGIN_FINAL_DRAIN,
                (uint32_t)fifo_count, fifo_count, 0U, 0U, 0U, 0U, 0U);
            return false;
        }
        if ((uint32_t)(zy100_os_time_ms() - start_ms) >=
            ZY100_OFFLINE_V2_IMU_FINAL_DRAIN_TIMEOUT_MS)
        {
            offline_v2_imu_latch_cause(
                ZY100_OFFLINE_V2_IMU_STOP_FIFO_READ,
                ZY100_OFFLINE_V2_IMU_ORIGIN_FINAL_DRAIN,
                (uint32_t)IMU_STATUS_TIMEOUT, fifo_count, 0U, 0U,
                0U, 0U, 0U);
            return false;
        }
        request_len = aligned;
        if (request_len > OFFLINE_V2_IMU_READ_BUFFER_BYTES)
        {
            request_len = OFFLINE_V2_IMU_READ_BUFFER_BYTES;
        }
        result = offline_v2_imu_process_read(
            fifo_count, request_len,
            ZY100_OFFLINE_V2_IMU_ORIGIN_FINAL_DRAIN);
        if (result == OFFLINE_V2_IMU_SERVICE_DEFER_FLASH_BUSY)
        {
            os_delay(1U);
            continue;
        }
        if (result != OFFLINE_V2_IMU_SERVICE_OK)
        {
            return false;
        }
        offline_v2_imu_complete_service();
        os_delay(1U);
    }
}

static void offline_v2_imu_publish_completion(void)
{
    uint32_t lock_state;
    uint32_t prepare_busy = bsp_shared_spi_imu_prepare_busy_count() -
                            s_imu.prepare_busy_start;
    uint32_t max_busy_us = s_imu.spi_retry.max_duration_us;

    if (s_imu.spi_retry.active)
    {
        uint32_t elapsed = offline_v2_imu_time_us() - s_imu.spi_retry.start_us;
        if (elapsed > max_busy_us) max_busy_us = elapsed;
    }
    if ((prepare_busy != 0U) || (s_imu.spi_retry.defer_count != 0U))
    {
        ZY100_LOG_EVENT("[OFF_SPI_WAIT] prep=%lu defer=%lu max_us=%lu",
                        (unsigned long)prepare_busy,
                        (unsigned long)s_imu.spi_retry.defer_count,
                        (unsigned long)max_busy_us);
    }

    imu_status_t cleanup_status = icm53611_stop_ui_sensors_preserve_fifo_ex();
    if (cleanup_status != IMU_STATUS_OK)
        ZY100_LOG_ERROR("[CAP_CLEAN] mode=0 step=2 st=%u", (uint32_t)cleanup_status);
    offline_v2_imu_disable_fifo();
    lock_state = os_lock();
    s_imu.prepared = false;
    s_imu.completion_generation++;
    s_imu.completion_pending = true;
    s_imu.state = ZY100_OFFLINE_V2_IMU_DONE;
    os_unlock(lock_state);

    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][IMU_END_A] reason=%u origin=%u detail=%lu ms=%lu",
        (uint32_t)s_imu.stop_reason,
        (uint32_t)s_imu.cause.origin,
        (unsigned long)s_imu.cause.detail,
        (unsigned long)s_imu.cause.time_ms);
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][IMU_END_B] accepted=%lu service=%lu emergency=%lu max=%u",
        (unsigned long)s_imu.accepted_packets,
        (unsigned long)s_imu.service_count,
        (unsigned long)s_imu.emergency_count,
        (uint32_t)s_imu.max_fifo_count);
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][IMU_END_C] full=%lu lost=%lu bad=%lu read=%lu header=0x%02x off=%u",
        (unsigned long)s_imu.fifo_full_count,
        (unsigned long)s_imu.fifo_lost_count,
        (unsigned long)s_imu.bad_header_count,
        (unsigned long)s_imu.read_error_count,
        (uint32_t)s_imu.cause.header,
        (uint32_t)s_imu.cause.buffer_offset);
    ZY100_LOG_ROUTINE(ZY100_LOG_EVENT, 
        "[OFFLINE_V2][IMU_END_D] flash_defer=%lu max_consecutive=%lu max_us=%lu pending_due=%lu",
        (unsigned long)s_imu.spi_retry.defer_count,
        (unsigned long)s_imu.spi_retry.max_consecutive_count,
        (unsigned long)s_imu.spi_retry.max_duration_us,
        (unsigned long)s_imu.timer_due);
}

static void offline_v2_imu_run_session(void)
{
    uint32_t notify_value = 0U;

    if (s_imu.stop_requested)
    {
        offline_v2_imu_publish_completion();
        return;
    }
    if (!offline_v2_imu_start_hardware())
    {
        offline_v2_imu_publish_completion();
        return;
    }

    while (!s_imu.stop_requested)
    {
        offline_v2_imu_service_result_t result;
        uint32_t due;

        notify_value = 0U;
        (void)xTaskNotifyWait(0U,
                              OFFLINE_V2_IMU_NOTIFY_MASK,
                              &notify_value,
                              pdMS_TO_TICKS(
                                  ZY100_OFFLINE_V2_IMU_SERVICE_TIMEOUT_MS));
        if ((notify_value & OFFLINE_V2_IMU_NOTIFY_STOP) != 0U)
        {
            break;
        }
        due = offline_v2_imu_take_due();
        if (due == 0U)
        {
            due = 1U;
        }
        while ((due != 0U) && !s_imu.stop_requested)
        {
            result = offline_v2_imu_service_once();
            if (result == OFFLINE_V2_IMU_SERVICE_DEFER_FLASH_BUSY)
            {
                offline_v2_imu_restore_due(due);
                break;
            }
            if (result == OFFLINE_V2_IMU_SERVICE_FATAL)
            {
                break;
            }
            due--;
        }
    }

    offline_v2_imu_timer_stop();
    if (offline_v2_imu_reason_is_graceful(s_imu.stop_reason))
    {
        (void)offline_v2_imu_final_drain();
    }
    offline_v2_imu_publish_completion();
}

static void offline_v2_imu_task(void *param)
{
    (void)param;
    while (true)
    {
        uint32_t notify_value = 0U;

        if (xTaskNotifyWait(0U,
                            OFFLINE_V2_IMU_NOTIFY_MASK,
                            &notify_value,
                            portMAX_DELAY) != pdTRUE)
        {
            continue;
        }
        if (((notify_value & OFFLINE_V2_IMU_NOTIFY_START) == 0U) ||
            (s_imu.state != ZY100_OFFLINE_V2_IMU_START_PENDING &&
             s_imu.state != ZY100_OFFLINE_V2_IMU_STOPPING))
        {
            continue;
        }
        offline_v2_imu_run_session();
    }
}

bool zy100_offline_v2_imu_session_init(void)
{
    if (s_imu.task != NULL)
    {
        return true;
    }
    if (!s_imu.initialized)
    {
        memset(&s_imu, 0, sizeof(s_imu));
        s_imu.state = ZY100_OFFLINE_V2_IMU_IDLE;
        s_imu.initialized = true;
    }
    if (xTaskCreate(offline_v2_imu_task,
                    "offline_v2_imu",
                    ZY100_OFFLINE_V2_IMU_TASK_STACK_WORDS,
                    NULL,
                    OFFLINE_V2_IMU_TASK_PRIORITY,
                    &s_imu.task) != pdPASS)
    {
        s_imu.state = ZY100_OFFLINE_V2_IMU_ERROR;
        return false;
    }
    return true;
}

bool zy100_offline_v2_imu_session_prepare(void)
{
    imu_status_t status;
    uint8_t who_am_i = 0U;

    if (!zy100_offline_v2_imu_session_init() ||
        ((s_imu.state != ZY100_OFFLINE_V2_IMU_IDLE) &&
         (s_imu.state != ZY100_OFFLINE_V2_IMU_DONE)))
    {
        return false;
    }
    memset(&s_imu.cause, 0, sizeof(s_imu.cause));
    s_imu.stop_reason = ZY100_OFFLINE_V2_IMU_STOP_NONE;
    s_imu.stop_requested = false;
    s_imu.completion_pending = false;
    s_imu.accepted_packets = 0U;
    s_imu.service_count = 0U;
    s_imu.emergency_count = 0U;
    s_imu.read_error_count = 0U;
    s_imu.bad_header_count = 0U;
    s_imu.fifo_full_count = 0U;
    s_imu.fifo_lost_count = 0U;
    zy100_offline_v2_spi_retry_reset(&s_imu.spi_retry);
    s_imu.prepare_busy_start = bsp_shared_spi_imu_prepare_busy_count();
    s_imu.last_fifo_count = 0U;
    s_imu.max_fifo_count = 0U;
    s_imu.timer_due = 0U;

    status = imu_bsp_power_ctrl(true);
    if (status == IMU_STATUS_OK)
    {
        imu_bsp_delay_us(5000U);
        status = imu_bsp_flash_cs_hold_high();
    }
    if (status == IMU_STATUS_OK)
    {
        status = icm53611_init_bus();
    }
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_set_mode(SPI_CPOL_Low, SPI_CPHA_1Edge);
    }
    if (status == IMU_STATUS_OK)
    {
        status = icm53611_read_whoami(&who_am_i);
    }
    if ((status != IMU_STATUS_OK) ||
        (who_am_i != ICM53611_WHO_AM_I_VALUE))
    {
        offline_v2_imu_latch_cause(
            ZY100_OFFLINE_V2_IMU_STOP_SENSOR_START,
            ZY100_OFFLINE_V2_IMU_ORIGIN_PREPARE,
            (status != IMU_STATUS_OK) ? (uint32_t)imu_status_legacy(status) :
            (uint32_t)who_am_i,
            0U, 0U, 0U, 0U, 0U, 0U);
        s_imu.state = ZY100_OFFLINE_V2_IMU_ERROR;
        return false;
    }
    s_imu.prepared = true;
    s_imu.state = ZY100_OFFLINE_V2_IMU_PREPARED;
    return true;
}

bool zy100_offline_v2_imu_session_start(void)
{
    if (!s_imu.initialized || (s_imu.task == NULL) || !s_imu.prepared ||
        (s_imu.state != ZY100_OFFLINE_V2_IMU_PREPARED))
    {
        return false;
    }
    s_imu.state = ZY100_OFFLINE_V2_IMU_START_PENDING;
    if (xTaskNotify(s_imu.task,
                    OFFLINE_V2_IMU_NOTIFY_START,
                    eSetBits) != pdPASS)
    {
        s_imu.state = ZY100_OFFLINE_V2_IMU_ERROR;
        offline_v2_imu_latch_cause(
            ZY100_OFFLINE_V2_IMU_STOP_WORKER_EXIT,
            ZY100_OFFLINE_V2_IMU_ORIGIN_WORKER,
            1U, 0U, 0U, 0U, 0U, 0U, 0U);
        return false;
    }
    return true;
}

bool zy100_offline_v2_imu_session_request_stop(
    zy100_offline_v2_imu_stop_reason_t reason)
{
    if (!s_imu.initialized || (s_imu.task == NULL) ||
        ((s_imu.state != ZY100_OFFLINE_V2_IMU_PREPARED) &&
         (s_imu.state != ZY100_OFFLINE_V2_IMU_START_PENDING) &&
         (s_imu.state != ZY100_OFFLINE_V2_IMU_RUNNING) &&
         (s_imu.state != ZY100_OFFLINE_V2_IMU_STOPPING)))
    {
        return false;
    }
    offline_v2_imu_latch_cause(reason,
                               ZY100_OFFLINE_V2_IMU_ORIGIN_CONTROLLER,
                               0U, 0U, 0U, 0U, 0U, 0U, 0U);
    (void)xTaskNotify(s_imu.task,
                      OFFLINE_V2_IMU_NOTIFY_STOP,
                      eSetBits);
    return true;
}

void zy100_offline_v2_imu_session_poll(uint32_t now_ms)
{
    (void)now_ms;
}

bool zy100_offline_v2_imu_session_get_status(
    zy100_offline_v2_imu_status_t *status_out)
{
    uint32_t lock_state;

    if (status_out == NULL)
    {
        return false;
    }
    lock_state = os_lock();
    memset(status_out, 0, sizeof(*status_out));
    status_out->state = s_imu.state;
    status_out->stop_reason = s_imu.stop_reason;
    status_out->completion_generation = s_imu.completion_generation;
    status_out->accepted_packets = s_imu.accepted_packets;
    status_out->service_count = s_imu.service_count;
    status_out->emergency_count = s_imu.emergency_count;
    status_out->read_error_count = s_imu.read_error_count;
    status_out->bad_header_count = s_imu.bad_header_count;
    status_out->fifo_full_count = s_imu.fifo_full_count;
    status_out->fifo_lost_count = s_imu.fifo_lost_count;
    status_out->pending_due = s_imu.timer_due;
    status_out->flash_busy_defer_count = s_imu.spi_retry.defer_count;
    status_out->flash_busy_max_consecutive =
        s_imu.spi_retry.max_consecutive_count;
    status_out->flash_busy_max_us = s_imu.spi_retry.max_duration_us;
    status_out->last_progress_ms = s_imu.last_progress_ms;
    status_out->last_fifo_count = s_imu.last_fifo_count;
    status_out->max_fifo_count = s_imu.max_fifo_count;
    status_out->timer_running = s_imu.timer_running;
    status_out->stop_requested = s_imu.stop_requested;
    status_out->flash_busy_active = s_imu.spi_retry.active;
    os_unlock(lock_state);
    return true;
}

bool zy100_offline_v2_imu_session_get_first_cause(
    zy100_offline_v2_imu_cause_t *cause_out)
{
    uint32_t lock_state;

    if (cause_out == NULL)
    {
        return false;
    }
    lock_state = os_lock();
    *cause_out = s_imu.cause;
    os_unlock(lock_state);
    return cause_out->valid;
}

bool zy100_offline_v2_imu_session_take_completion(
    zy100_offline_v2_imu_stop_reason_t *reason_out)
{
    uint32_t lock_state;

    lock_state = os_lock();
    if (!s_imu.completion_pending)
    {
        os_unlock(lock_state);
        return false;
    }
    if (reason_out != NULL)
    {
        *reason_out = s_imu.stop_reason;
    }
    s_imu.completion_pending = false;
    s_imu.stop_requested = false;
    s_imu.state = ZY100_OFFLINE_V2_IMU_IDLE;
    os_unlock(lock_state);
    return true;
}

bool zy100_offline_v2_imu_session_active(void)
{
    return (s_imu.state == ZY100_OFFLINE_V2_IMU_START_PENDING) ||
           (s_imu.state == ZY100_OFFLINE_V2_IMU_RUNNING) ||
           (s_imu.state == ZY100_OFFLINE_V2_IMU_STOPPING);
}

bool zy100_offline_v2_imu_session_quiescent(void)
{
    return !zy100_offline_v2_imu_session_active();
}

bool zy100_offline_v2_imu_session_prepare_sleep(void)
{
    TaskHandle_t task_to_delete;
    uint32_t lock_state;

    if (zy100_offline_v2_imu_session_active())
    {
        return false;
    }
    offline_v2_imu_timer_stop();
    imu_bsp_int_register_irq_callback(NULL);
    imu_bsp_int_clear_pending();
    s_imu.prepared = false;
    s_imu.state = ZY100_OFFLINE_V2_IMU_IDLE;
    lock_state = os_lock();
    task_to_delete = s_imu.task;
    s_imu.task = NULL;
    os_unlock(lock_state);
    if (task_to_delete != NULL)
    {
        vTaskDelete(task_to_delete);
    }
    return true;
}
