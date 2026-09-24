#include "imu_rt_marker.h"

#include <stddef.h>
#include <string.h>

#include "os_sync.h"
#include "trace.h"

#include "../app_flags.h"
#include "../bsp/imu_bsp.h"
#include "../driver/icm53611_driver.h"
#include "imu_marker_detector.h"

#define IMU_RT_MARKER_US_PER_S             1000000ULL
#define IMU_RT_MARKER_US_PER_MS            1000ULL
#define IMU_RT_MARKER_FIFO_PACKET_BYTES    16U
#define IMU_RT_MARKER_FIFO_ACCEL_X_H       1U
#define IMU_RT_MARKER_FIFO_ACCEL_X_L       2U
#define IMU_RT_MARKER_FIFO_ACCEL_Y_H       3U
#define IMU_RT_MARKER_FIFO_ACCEL_Y_L       4U
#define IMU_RT_MARKER_FIFO_ACCEL_Z_H       5U
#define IMU_RT_MARKER_FIFO_ACCEL_Z_L       6U
#define IMU_RT_MARKER_FIFO_GYRO_X_H        7U
#define IMU_RT_MARKER_FIFO_GYRO_X_L        8U
#define IMU_RT_MARKER_FIFO_GYRO_Y_H        9U
#define IMU_RT_MARKER_FIFO_GYRO_Y_L        10U
#define IMU_RT_MARKER_FIFO_GYRO_Z_H        11U
#define IMU_RT_MARKER_FIFO_GYRO_Z_L        12U

typedef struct
{
    bool active;
    bool session_due_valid;
    uint32_t session_start_due_seq;
    uint32_t seq;
    uint32_t read_ok_total;
    uint32_t read_fail_total;
    uint32_t marker_total;
    uint32_t due_total;
    uint32_t skipped_high_pressure_total;
    uint32_t skipped_spi_busy_total;
    uint32_t skipped_not_due_total;
    uint32_t poll_call_count_total;
    uint32_t skipped_late_slots_total;
    uint32_t window_start_ms;
    uint32_t dropped_total_last_log;
    uint32_t read_ok_1s;
    uint32_t read_fail_1s;
    uint32_t due_1s;
    uint32_t skipped_high_pressure_1s;
    uint32_t skipped_spi_busy_1s;
    uint32_t skipped_not_due_1s;
    uint32_t poll_call_count_1s;
    uint32_t skipped_late_slots_1s;
    uint32_t max_read_us_total;
    uint32_t max_gap_us_total;
    uint32_t max_read_us_1s;
    uint32_t max_gap_us_1s;
    bool detector_state_stale;
    volatile uint32_t pending_due_count;
    volatile uint32_t pending_due_latest_seq;
} imu_rt_marker_state_t;

typedef struct
{
    uint32_t count;
    uint32_t latest_seq;
} imu_rt_marker_due_batch_t;

#if ZY100_RT_MARKER_ENABLE
static imu_rt_marker_state_t s_rt_marker;

#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE && ZY100_RT_MARKER_UI_READ_ENABLE
static uint64_t imu_rt_marker_period_us(void)
{
    uint64_t period_us;

    if (ZY100_RT_MARKER_TARGET_HZ == 0U)
    {
        return IMU_RT_MARKER_US_PER_S;
    }

    period_us = IMU_RT_MARKER_US_PER_S / (uint64_t)ZY100_RT_MARKER_TARGET_HZ;
    return (period_us == 0ULL) ? 1ULL : period_us;
}
#endif

static uint32_t imu_rt_marker_u64_to_u32_sat(uint64_t value)
{
    return (value > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)value;
}

static int16_t imu_rt_marker_packet_be_s16(const uint8_t *packet,
                                           uint8_t high_idx,
                                           uint8_t low_idx)
{
    return (int16_t)(((uint16_t)packet[high_idx] << 8) | packet[low_idx]);
}

static void imu_rt_marker_note_detector_allowed(void)
{
    s_rt_marker.detector_state_stale = false;
}

static void imu_rt_marker_note_due_slots(uint32_t slot_count)
{
    s_rt_marker.due_total += slot_count;
    s_rt_marker.due_1s += slot_count;

    if (slot_count > 1U)
    {
        uint32_t skipped_late_slots = slot_count - 1U;

        s_rt_marker.skipped_late_slots_total += skipped_late_slots;
        s_rt_marker.skipped_late_slots_1s += skipped_late_slots;
    }
}

#if !ZY100_PHASE_A_FIFO_POLL_RT_ENABLE && ZY100_RT_MARKER_UI_READ_ENABLE
static uint32_t imu_rt_marker_session_t_us(uint32_t due_seq)
{
    uint32_t due_delta;

    if (!s_rt_marker.session_due_valid)
    {
        return 0U;
    }

    due_delta = due_seq - s_rt_marker.session_start_due_seq;
    return imu_rt_marker_u64_to_u32_sat((uint64_t)due_delta *
                                        imu_rt_marker_period_us());
}
#endif

static bool imu_rt_marker_startup_ignore_active(uint32_t sample_t_us)
{
    uint32_t ignore_us = ZY100_RT_MARKER_STARTUP_IGNORE_MS *
                         (uint32_t)IMU_RT_MARKER_US_PER_MS;

    return sample_t_us < ignore_us;
}

static uint32_t imu_rt_marker_dropped_total(void)
{
    imu_marker_detector_stats_t detector_stats;

    imu_marker_detector_get_stats(&detector_stats);
    return detector_stats.dropped_cooldown_count +
           detector_stats.rejected_timeout_count;
}

static void imu_rt_marker_log_5s_if_due(uint32_t now_ms)
{
#if !ZY100_RUNTIME_STATS_LOG_ENABLE
    IMU_UNUSED(now_ms);
#else
    uint32_t elapsed_ms;
    uint32_t read_hz;
    uint32_t dropped_total;
    uint32_t dropped_window;

    if (!s_rt_marker.active)
    {
        return;
    }

    if (s_rt_marker.window_start_ms == 0U)
    {
        s_rt_marker.window_start_ms = now_ms;
        return;
    }

    elapsed_ms = now_ms - s_rt_marker.window_start_ms;
    if (elapsed_ms < ZY100_RT_MARKER_LOG_PERIOD_MS)
    {
        return;
    }

    if (elapsed_ms == 0U)
    {
        elapsed_ms = 1U;
    }
    read_hz = (uint32_t)(((uint64_t)s_rt_marker.read_ok_1s * 1000ULL) /
                         (uint64_t)elapsed_ms);
    dropped_total = imu_rt_marker_dropped_total();
    dropped_window = (dropped_total >= s_rt_marker.dropped_total_last_log) ?
                     (dropped_total - s_rt_marker.dropped_total_last_log) :
                     dropped_total;

#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
    DBG_DIRECT("[RTM_5S] source=fifo feed_packets=%u feed_hz=%u marker_count=%u dropped=%u",
               s_rt_marker.read_ok_1s,
               read_hz,
               s_rt_marker.marker_total,
               dropped_window);
#else
    DBG_DIRECT("[RTM_5S] due=%u read_ok=%u read_hz=%u late=%u skip_fifo=%u skip_spi=%u marker_count=%u dropped=%u",
               s_rt_marker.due_1s,
               s_rt_marker.read_ok_1s,
               read_hz,
               s_rt_marker.skipped_late_slots_1s,
               s_rt_marker.skipped_high_pressure_1s,
               s_rt_marker.skipped_spi_busy_1s,
               s_rt_marker.marker_total,
               dropped_window);
#endif

    s_rt_marker.read_ok_1s = 0U;
    s_rt_marker.read_fail_1s = 0U;
    s_rt_marker.due_1s = 0U;
    s_rt_marker.skipped_high_pressure_1s = 0U;
    s_rt_marker.skipped_spi_busy_1s = 0U;
    s_rt_marker.skipped_not_due_1s = 0U;
    s_rt_marker.poll_call_count_1s = 0U;
    s_rt_marker.skipped_late_slots_1s = 0U;
    s_rt_marker.max_read_us_1s = 0U;
    s_rt_marker.max_gap_us_1s = 0U;
    s_rt_marker.window_start_ms = now_ms;
    s_rt_marker.dropped_total_last_log = dropped_total;
#endif
}

static imu_rt_marker_due_batch_t imu_rt_marker_take_pending_due(void)
{
    uint32_t lock_state;
    imu_rt_marker_due_batch_t batch;

    lock_state = os_lock();
    batch.count = s_rt_marker.pending_due_count;
    batch.latest_seq = s_rt_marker.pending_due_latest_seq;
    s_rt_marker.pending_due_count = 0U;
    os_unlock(lock_state);
    return batch;
}

static bool imu_rt_marker_read_once(uint32_t due_seq, uint32_t now_ms)
{
#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE || !ZY100_RT_MARKER_UI_READ_ENABLE
    IMU_UNUSED(due_seq);
    IMU_UNUSED(now_ms);
    return false;
#else
    icm53611_raw_sample_t raw_sample;
    imu_marker_detector_sample_t detector_sample;
    imu_marker_detector_event_t event;
    imu_status_t status;
    bool event_valid = false;
    uint32_t sample_t_us;

    status = icm53611_read_ui_6axis_sample(&raw_sample);
    sample_t_us = imu_rt_marker_session_t_us(due_seq);

    if (status != IMU_STATUS_OK)
    {
        s_rt_marker.read_fail_total++;
        s_rt_marker.read_fail_1s++;
        imu_rt_marker_log_5s_if_due(now_ms);
        return false;
    }

    s_rt_marker.read_ok_total++;
    s_rt_marker.read_ok_1s++;
    s_rt_marker.seq++;
    imu_rt_marker_note_detector_allowed();

    if (imu_rt_marker_startup_ignore_active(sample_t_us))
    {
        imu_rt_marker_log_5s_if_due(now_ms);
        return true;
    }

    detector_sample.seq = s_rt_marker.seq;
    detector_sample.t_us = sample_t_us;
    detector_sample.accel_x_raw = raw_sample.accel_x;
    detector_sample.accel_y_raw = raw_sample.accel_y;
    detector_sample.accel_z_raw = raw_sample.accel_z;
    detector_sample.gyro_x_raw = raw_sample.gyro_x;
    detector_sample.gyro_y_raw = raw_sample.gyro_y;
    detector_sample.gyro_z_raw = raw_sample.gyro_z;

    if (imu_marker_detector_process_sample(&detector_sample,
                                           &event_valid,
                                           &event) != IMU_STATUS_OK)
    {
        imu_rt_marker_log_5s_if_due(now_ms);
        return true;
    }

    if (event_valid)
    {
        s_rt_marker.marker_total++;
    }
    imu_rt_marker_log_5s_if_due(now_ms);
    return true;
#endif
}

static void imu_rt_marker_skip_due(uint32_t now_ms, bool fifo_emergency)
{
    imu_rt_marker_due_batch_t batch;

    batch = imu_rt_marker_take_pending_due();
    if (batch.count == 0U)
    {
        imu_rt_marker_log_5s_if_due(now_ms);
        return;
    }

    imu_rt_marker_note_due_slots(batch.count);
    if (fifo_emergency)
    {
        s_rt_marker.skipped_high_pressure_total++;
        s_rt_marker.skipped_high_pressure_1s++;
    }
    else
    {
        s_rt_marker.skipped_spi_busy_total++;
        s_rt_marker.skipped_spi_busy_1s++;
    }
    imu_rt_marker_log_5s_if_due(now_ms);
}
#endif

imu_status_t imu_rt_marker_reset(uint64_t session_start_us)
{
#if ZY100_RT_MARKER_ENABLE
    memset(&s_rt_marker, 0, sizeof(s_rt_marker));
    s_rt_marker.active = true;
    IMU_UNUSED(session_start_us);
    IMU_UNUSED(imu_marker_detector_reset());
    return IMU_STATUS_OK;
#else
    IMU_UNUSED(session_start_us);
    return IMU_STATUS_OK;
#endif
}

void imu_rt_marker_poll(uint64_t now_ms, bool fifo_high_pressure)
{
    IMU_UNUSED(now_ms);
    IMU_UNUSED(fifo_high_pressure);
}

void imu_rt_marker_timer_due_isr_hook(uint32_t due_seq)
{
#if ZY100_RT_MARKER_ENABLE
    uint32_t lock_state;

    if (!s_rt_marker.active)
    {
        return;
    }

    lock_state = os_lock();
    if (!s_rt_marker.session_due_valid)
    {
        s_rt_marker.session_start_due_seq = due_seq;
        s_rt_marker.session_due_valid = true;
    }
    if (s_rt_marker.pending_due_count < 0xFFFFFFFFU)
    {
        s_rt_marker.pending_due_count++;
    }
    s_rt_marker.pending_due_latest_seq = due_seq;
    os_unlock(lock_state);
#else
    IMU_UNUSED(due_seq);
#endif
}

bool imu_rt_marker_has_due(void)
{
#if ZY100_RT_MARKER_ENABLE
    uint32_t lock_state;
    bool has_due;

    lock_state = os_lock();
    has_due = (s_rt_marker.pending_due_count != 0U);
    os_unlock(lock_state);
    return has_due;
#else
    return false;
#endif
}

bool imu_rt_marker_handle_due_once(uint32_t now_ms)
{
#if ZY100_RT_MARKER_ENABLE
    imu_rt_marker_due_batch_t batch;

    if (!s_rt_marker.active)
    {
        return false;
    }

    s_rt_marker.poll_call_count_total++;
    s_rt_marker.poll_call_count_1s++;

    batch = imu_rt_marker_take_pending_due();
    if (batch.count == 0U)
    {
        s_rt_marker.skipped_not_due_total++;
        s_rt_marker.skipped_not_due_1s++;
        imu_rt_marker_log_5s_if_due(now_ms);
        return false;
    }

    imu_rt_marker_note_due_slots(batch.count);
    return imu_rt_marker_read_once(batch.latest_seq, now_ms);
#else
    IMU_UNUSED(now_ms);
    return false;
#endif
}

bool imu_rt_marker_feed_fifo_packet(const uint8_t packet16[16],
                                    uint32_t sample_seq,
                                    uint64_t t_us)
{
#if ZY100_RT_MARKER_ENABLE && ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
    imu_marker_detector_sample_t detector_sample;
    imu_marker_detector_event_t event;
    bool event_valid = false;
    uint32_t sample_t_us;

    if ((packet16 == NULL) || !s_rt_marker.active)
    {
        return false;
    }

    sample_t_us = imu_rt_marker_u64_to_u32_sat(t_us);
    s_rt_marker.read_ok_total++;
    s_rt_marker.read_ok_1s++;
    s_rt_marker.seq = sample_seq;
    imu_rt_marker_note_detector_allowed();

    if (imu_rt_marker_startup_ignore_active(sample_t_us))
    {
        imu_rt_marker_log_5s_if_due((uint32_t)(sample_t_us / 1000U));
        return true;
    }

    detector_sample.seq = sample_seq;
    detector_sample.t_us = sample_t_us;
    detector_sample.accel_x_raw = imu_rt_marker_packet_be_s16(
        packet16,
        IMU_RT_MARKER_FIFO_ACCEL_X_H,
        IMU_RT_MARKER_FIFO_ACCEL_X_L);
    detector_sample.accel_y_raw = imu_rt_marker_packet_be_s16(
        packet16,
        IMU_RT_MARKER_FIFO_ACCEL_Y_H,
        IMU_RT_MARKER_FIFO_ACCEL_Y_L);
    detector_sample.accel_z_raw = imu_rt_marker_packet_be_s16(
        packet16,
        IMU_RT_MARKER_FIFO_ACCEL_Z_H,
        IMU_RT_MARKER_FIFO_ACCEL_Z_L);
    detector_sample.gyro_x_raw = imu_rt_marker_packet_be_s16(
        packet16,
        IMU_RT_MARKER_FIFO_GYRO_X_H,
        IMU_RT_MARKER_FIFO_GYRO_X_L);
    detector_sample.gyro_y_raw = imu_rt_marker_packet_be_s16(
        packet16,
        IMU_RT_MARKER_FIFO_GYRO_Y_H,
        IMU_RT_MARKER_FIFO_GYRO_Y_L);
    detector_sample.gyro_z_raw = imu_rt_marker_packet_be_s16(
        packet16,
        IMU_RT_MARKER_FIFO_GYRO_Z_H,
        IMU_RT_MARKER_FIFO_GYRO_Z_L);

    if (imu_marker_detector_process_sample(&detector_sample,
                                           &event_valid,
                                           &event) != IMU_STATUS_OK)
    {
        s_rt_marker.read_fail_total++;
        s_rt_marker.read_fail_1s++;
        imu_rt_marker_log_5s_if_due((uint32_t)(sample_t_us / 1000U));
        return false;
    }

    if (event_valid)
    {
        s_rt_marker.marker_total++;
    }
    imu_rt_marker_log_5s_if_due((uint32_t)(sample_t_us / 1000U));
    return true;
#else
    IMU_UNUSED(packet16);
    IMU_UNUSED(sample_seq);
    IMU_UNUSED(t_us);
    return true;
#endif
}

void imu_rt_marker_skip_fifo_emergency(uint32_t now_ms)
{
#if ZY100_RT_MARKER_ENABLE
    imu_rt_marker_skip_due(now_ms, true);
#else
    IMU_UNUSED(now_ms);
#endif
}

void imu_rt_marker_skip_spi_busy(uint32_t now_ms)
{
#if ZY100_RT_MARKER_ENABLE
    imu_rt_marker_skip_due(now_ms, false);
#else
    IMU_UNUSED(now_ms);
#endif
}

void imu_rt_marker_finish(void)
{
#if ZY100_RT_MARKER_ENABLE
    if (!s_rt_marker.active)
    {
        return;
    }

    s_rt_marker.active = false;
#endif
}

void imu_rt_marker_log_final(void)
{
#if ZY100_RT_MARKER_ENABLE
    uint32_t dropped_total = imu_rt_marker_dropped_total();

#if ZY100_PHASE_A_FIFO_POLL_RT_ENABLE
    DBG_DIRECT("[RTM_SUM] source=fifo feed_packets=%u marker_count=%u dropped=%u",
               s_rt_marker.read_ok_total,
               s_rt_marker.marker_total,
               dropped_total);
#else
    DBG_DIRECT("[RTM_SUM] due=%u read_ok=%u read_fail=%u late=%u skip_fifo=%u skip_spi=%u marker_count=%u dropped=%u",
               s_rt_marker.due_total,
               s_rt_marker.read_ok_total,
               s_rt_marker.read_fail_total,
               s_rt_marker.skipped_late_slots_total,
               s_rt_marker.skipped_high_pressure_total,
               s_rt_marker.skipped_spi_busy_total,
               s_rt_marker.marker_total,
               dropped_total);
#endif
#endif
}

void imu_rt_marker_get_stats(imu_rt_marker_stats_t *out)
{
#if ZY100_RT_MARKER_ENABLE
    imu_marker_detector_stats_t detector_stats;
#endif

    if (out == NULL)
    {
        return;
    }

#if ZY100_RT_MARKER_ENABLE
    imu_marker_detector_get_stats(&detector_stats);
    out->read_ok_total = s_rt_marker.read_ok_total;
    out->read_fail_total = s_rt_marker.read_fail_total;
    out->marker_total = s_rt_marker.marker_total;
    out->due_total = s_rt_marker.due_total;
    out->skipped_high_pressure_total = s_rt_marker.skipped_high_pressure_total;
    out->skipped_spi_busy_total = s_rt_marker.skipped_spi_busy_total;
    out->skipped_not_due_total = s_rt_marker.skipped_not_due_total;
    out->poll_call_count_total = s_rt_marker.poll_call_count_total;
    out->skipped_late_slots_total = s_rt_marker.skipped_late_slots_total;
    out->dropped_cooldown_total = detector_stats.dropped_cooldown_count;
    out->rejected_timeout_total = detector_stats.rejected_timeout_count;
    out->marker_dropped_total = imu_rt_marker_dropped_total();
    out->max_read_us = s_rt_marker.max_read_us_total;
    out->max_gap_us = s_rt_marker.max_gap_us_total;
#else
    memset(out, 0, sizeof(*out));
#endif
}
