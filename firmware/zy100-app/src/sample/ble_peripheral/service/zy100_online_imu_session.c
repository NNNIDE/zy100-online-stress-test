#include "zy100_online_imu_session.h"

#include <stddef.h>

#include "imu_fifo_drain_test.h"
#include "zy100_online_raw_capture.h"

static uint32_t s_completion_consumed;
static zy100_online_imu_terminal_t s_terminal;

static imu_fifo_drain_test_stop_reason_t online_imu_to_worker_reason(
    zy100_online_imu_stop_reason_t reason)
{
    switch (reason)
    {
    case ZY100_ONLINE_IMU_STOP_HOST_PAUSE:
        return IMU_FIFO_DRAIN_TEST_STOP_REASON_PAUSED;
    case ZY100_ONLINE_IMU_STOP_LOW_BATTERY:
        return IMU_FIFO_DRAIN_TEST_STOP_REASON_BATTERY_LOW;
    case ZY100_ONLINE_IMU_STOP_DISCONNECT:
    case ZY100_ONLINE_IMU_STOP_FATAL:
        return IMU_FIFO_DRAIN_TEST_STOP_REASON_FATAL;
    case ZY100_ONLINE_IMU_STOP_NONE:
    default:
        return IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE;
    }
}

static zy100_online_imu_stop_reason_t online_imu_from_worker_reason(
    imu_fifo_drain_test_stop_reason_t reason)
{
    switch (reason)
    {
    case IMU_FIFO_DRAIN_TEST_STOP_REASON_PAUSED:
        return ZY100_ONLINE_IMU_STOP_HOST_PAUSE;
    case IMU_FIFO_DRAIN_TEST_STOP_REASON_BATTERY_LOW:
        return ZY100_ONLINE_IMU_STOP_LOW_BATTERY;
    case IMU_FIFO_DRAIN_TEST_STOP_REASON_NONE:
        return ZY100_ONLINE_IMU_STOP_NONE;
    default:
        return ZY100_ONLINE_IMU_STOP_FATAL;
    }
}

bool zy100_online_imu_session_init(void)
{
    s_completion_consumed = imu_fifo_drain_test_completion_generation();
    s_terminal = ZY100_ONLINE_IMU_TERMINAL_NONE;
    /* Preserve the board-verified 10375 Online wake order.  Offline V2 keeps
     * its worker lazy, so this is the only 2048-word capture stack resident
     * before a mode has been selected. */
    return imu_fifo_drain_test_task_init();
}

bool zy100_online_imu_session_prepare(uint32_t start_round)
{
    bool prepared =
        imu_fifo_drain_test_prepare_continuous_online(start_round) ==
        IMU_ONLINE_PREFLIGHT_OK;

    if (prepared)
    {
        s_terminal = ZY100_ONLINE_IMU_TERMINAL_NONE;
    }
    return prepared;
}

bool zy100_online_imu_session_start(void)
{
    if (!imu_fifo_drain_test_start())
    {
        return false;
    }
    s_terminal = ZY100_ONLINE_IMU_TERMINAL_NONE;
    return true;
}

bool zy100_online_imu_session_request_maintenance_pause(void)
{
    if (s_terminal == ZY100_ONLINE_IMU_TERMINAL_ABORT)
    {
        return false;
    }
    return imu_fifo_drain_test_request_online_pause_with_reason(
               IMU_FIFO_DRAIN_TEST_STOP_REASON_PAUSED);
}

bool zy100_online_imu_session_request_host_pause(void)
{
    if (s_terminal == ZY100_ONLINE_IMU_TERMINAL_ABORT)
    {
        return false;
    }
    if (!imu_fifo_drain_test_request_online_pause_with_reason(
            online_imu_to_worker_reason(ZY100_ONLINE_IMU_STOP_HOST_PAUSE)))
    {
        return false;
    }
    s_terminal = ZY100_ONLINE_IMU_TERMINAL_GRACEFUL_HOST_PAUSE;
    return true;
}

bool zy100_online_imu_session_request_low_battery_stop(void)
{
    s_terminal = ZY100_ONLINE_IMU_TERMINAL_ABORT;
    return imu_fifo_drain_test_request_online_pause_with_reason(
               online_imu_to_worker_reason(
                   ZY100_ONLINE_IMU_STOP_LOW_BATTERY));
}

bool zy100_online_imu_session_request_abort(void)
{
    s_terminal = ZY100_ONLINE_IMU_TERMINAL_ABORT;
    return imu_fifo_drain_test_request_online_abort();
}

bool zy100_online_imu_session_get_status(zy100_online_imu_status_t *status_out)
{
    zy100_online_raw_capture_stats_t raw_stats;

    if (status_out == NULL)
    {
        return false;
    }
    status_out->active = imu_fifo_drain_test_is_active();
    status_out->busy = imu_fifo_drain_test_is_busy();
    status_out->start_pending = imu_fifo_drain_test_is_start_pending();
    status_out->stop_in_progress = imu_fifo_drain_test_is_stop_in_progress();
    status_out->done = imu_fifo_drain_test_is_done();
    status_out->completion_generation =
        imu_fifo_drain_test_completion_generation();
    status_out->stop_reason = online_imu_from_worker_reason(
        imu_fifo_drain_test_last_stop_reason());
    if ((status_out->stop_reason != ZY100_ONLINE_IMU_STOP_NONE) &&
        (status_out->stop_reason != ZY100_ONLINE_IMU_STOP_HOST_PAUSE))
    {
        s_terminal = ZY100_ONLINE_IMU_TERMINAL_ABORT;
    }
    status_out->terminal = s_terminal;
    status_out->graceful_tail_committed = false;
    if (status_out->done &&
        (status_out->terminal ==
         ZY100_ONLINE_IMU_TERMINAL_GRACEFUL_HOST_PAUSE) &&
        (status_out->stop_reason == ZY100_ONLINE_IMU_STOP_HOST_PAUSE))
    {
        zy100_online_raw_capture_get_stats(&raw_stats);
        status_out->graceful_tail_committed =
            (raw_stats.accepted_packets == raw_stats.committed_packets) &&
            (raw_stats.discarded_packets == 0U) &&
            (raw_stats.accepted_mag_samples ==
             raw_stats.committed_mag_samples) &&
            (raw_stats.discarded_mag_samples == 0U);
    }
    return true;
}

bool zy100_online_imu_session_take_completion(
    zy100_online_imu_stop_reason_t *reason_out,
    uint32_t *generation_out)
{
    uint32_t generation = imu_fifo_drain_test_completion_generation();

    if (!imu_fifo_drain_test_is_done() ||
        (generation == s_completion_consumed))
    {
        return false;
    }
    s_completion_consumed = generation;
    if (reason_out != NULL)
    {
        *reason_out = online_imu_from_worker_reason(
            imu_fifo_drain_test_last_stop_reason());
    }
    if (generation_out != NULL)
    {
        *generation_out = generation;
    }
    return true;
}

bool zy100_online_imu_session_prepare_sleep(void)
{
    if (imu_fifo_drain_test_is_active() ||
        imu_fifo_drain_test_is_busy() ||
        imu_fifo_drain_test_is_start_pending() ||
        imu_fifo_drain_test_is_stop_in_progress())
    {
        return false;
    }
    imu_fifo_drain_test_prepare_continuous_online_abort();
    return true;
}

bool zy100_online_imu_session_release_idle_worker(void)
{
    return imu_fifo_drain_test_task_release_idle();
}
