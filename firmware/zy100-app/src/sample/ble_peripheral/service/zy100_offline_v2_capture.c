#include "zy100_offline_v2_capture.h"

#include <stddef.h>
#include <string.h>

#include "os_sync.h"
#include "os_sched.h"
#include "rtl876x_trng.h"
#include "app_flags.h"
#include "../bsp/imu_bsp.h"
#include "mag_capture_service.h"
#include "trace.h"
#include "zy100_capture_profile.h"
#include "zy100_clock_config.h"
#include "zy100_mode_workspace.h"
#include "zy100_offline_v2_config.h"
#include "zy100_offline_v2_imu_session.h"
#include "zy100_online_reset_trace.h"

#define OFFLINE_V2_IMU_START_TIMEOUT_MS      1000U
#define OFFLINE_V2_IMU_STALL_TIMEOUT_MS      250U
#define OFFLINE_V2_EVENT_QUEUE_DEPTH            4U
#define OFFLINE_V2_WORKSPACE_ALIGN_BYTES         4U
#define OFFLINE_V2_ALIGN_UP_4(value) \
    (((value) + (OFFLINE_V2_WORKSPACE_ALIGN_BYTES - 1U)) & \
     ~(OFFLINE_V2_WORKSPACE_ALIGN_BYTES - 1U))
#define OFFLINE_V2_IMU_DT_TOLERANCE_US       250U
#define OFFLINE_V2_IMU_DT_NOMINAL_US         \
    ZY100_OFFLINE_V2_IMU_SAMPLE_INTERVAL_US
#define OFFLINE_V2_IMU_DT_MIN_US             \
    (OFFLINE_V2_IMU_DT_NOMINAL_US - OFFLINE_V2_IMU_DT_TOLERANCE_US)
#define OFFLINE_V2_IMU_DT_MAX_US             \
    (OFFLINE_V2_IMU_DT_NOMINAL_US + OFFLINE_V2_IMU_DT_TOLERANCE_US)

#if (ZY100_OFFLINE_V2_SAMPLE_RATE_HZ != \
     ZY100_OFFLINE_V2_IMU_SAMPLE_RATE_HZ)
#error "Offline V2 algorithm rate must match its private IMU session rate"
#endif

#if ((1000000U % ZY100_OFFLINE_V2_IMU_SAMPLE_RATE_HZ) != 0U)
#error "Offline V2 IMU sample rate must have an integer microsecond period"
#endif

typedef char offline_v2_event_queue_workspace_check[
    ((ZY100_OFFLINE_V2_ALGO_WORKSPACE_MAX +
      (OFFLINE_V2_EVENT_QUEUE_DEPTH * sizeof(zy100_offline_v2_event_t))) <=
     ZY100_MODE_WORKSPACE_TRANSIENT_BYTES) ? 1 : -1];

typedef struct
{
    zy100_offline_v2_capture_state_t state;
    zy100_offline_v2_algo_t *algo;
    zy100_offline_v2_quality_t quality;
    zy100_offline_v2_event_t *event_queue;
    zy100_offline_v2_stop_reason_t stop_reason;
    zy100_offline_v2_stop_reason_t completed_reason;
    uint32_t start_ms;
    uint64_t elapsed_start_ms;
    uint64_t elapsed_final_ms;
    zy100_training_snapshot_t identity;
    bool elapsed_valid;
    uint32_t last_mag_poll_ms;
    uint32_t imu_sample_count;
    uint32_t mag_sample_count;
    uint32_t imu_start_ms;
    uint32_t imu_last_progress_ms;
    uint32_t imu_start_generation;
    uint32_t event_queue_read;
    uint32_t event_queue_write;
    uint32_t event_queue_count;
    uint32_t event_queue_max_count;
    uint32_t push_last_us;
    uint32_t push_max_us;
    uint32_t observer_window_generation;
    zy100_offline_v2_capture_state_t last_logged_state;
    uint16_t last_timestamp_raw;
    bool storage_initialized;
    bool workspace_claimed;
    bool mag_started;
    bool imu_seen_active;
    bool imu_timestamp_valid;
    bool stop_requested;
    bool algo_finished;
    bool imu_completion_consumed;
    bool finalize_requested;
    bool completion_pending;
    bool last_logged_state_valid;
    bool observer_window_open;
    bool imu_time_defer_logged;
} offline_v2_capture_runtime_t;

static offline_v2_capture_runtime_t s_capture;
static bool s_shutdown_active;
static bool s_shutdown_recovering;
static bool s_shutdown_fault_exit;
static uint8_t s_shutdown_attempts;
static uint8_t s_shutdown_hw_failures;
static uint32_t s_shutdown_retry_ms;

static uint32_t offline_v2_time_us(void)
{
    return (uint32_t)imu_bsp_local_timestamp_us();
}

static bool offline_v2_elapsed_ms_valid(uint32_t now_ms,
                                        uint32_t then_ms,
                                        uint32_t *elapsed_ms_out)
{
    int32_t delta_ms;

    if (elapsed_ms_out == NULL)
    {
        return false;
    }
    delta_ms = (int32_t)(now_ms - then_ms);
    if (delta_ms < 0)
    {
        *elapsed_ms_out = 0U;
        return false;
    }
    *elapsed_ms_out = (uint32_t)delta_ms;
    return true;
}

static void offline_v2_log_time_defer_once(uint32_t now_ms,
                                           uint32_t progress_ms)
{
    if (s_capture.imu_time_defer_logged)
    {
        return;
    }
    s_capture.imu_time_defer_logged = true;
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][IMU_TIME_DEFER] now=%lu progress=%lu skew_ms=%lu",
        (unsigned long)now_ms,
        (unsigned long)progress_ms,
        (unsigned long)(progress_ms - now_ms));
}

static void offline_v2_event_queue_reset(void)
{
    uint32_t lock_state = os_lock();

    s_capture.event_queue_read = 0U;
    s_capture.event_queue_write = 0U;
    s_capture.event_queue_count = 0U;
    s_capture.event_queue_max_count = 0U;
    os_unlock(lock_state);
}

static uint32_t offline_v2_event_queue_level(void)
{
    uint32_t level;
    uint32_t lock_state = os_lock();

    level = s_capture.event_queue_count;
    os_unlock(lock_state);
    return level;
}

static const zy100_offline_v2_event_t *offline_v2_event_queue_peek(void)
{
    const zy100_offline_v2_event_t *event = NULL;
    uint32_t lock_state = os_lock();

    if ((s_capture.event_queue != NULL) &&
        (s_capture.event_queue_count != 0U))
    {
        event = &s_capture.event_queue[s_capture.event_queue_read];
    }
    os_unlock(lock_state);
    return event;
}

static void offline_v2_event_queue_pop(void)
{
    uint32_t lock_state = os_lock();

    if (s_capture.event_queue_count != 0U)
    {
        s_capture.event_queue_read =
            (s_capture.event_queue_read + 1U) % OFFLINE_V2_EVENT_QUEUE_DEPTH;
        s_capture.event_queue_count--;
    }
    os_unlock(lock_state);
}

static bool offline_v2_imu_quiescent(void)
{
    return zy100_offline_v2_imu_session_quiescent();
}

static bool offline_v2_flash_fifo_urgent(void)
{
    zy100_offline_v2_imu_status_t status;

    memset(&status, 0, sizeof(status));
    if (!zy100_offline_v2_imu_session_get_status(&status))
    {
        return true;
    }
    if (status.state == ZY100_OFFLINE_V2_IMU_STOPPING)
    {
        return true;
    }
    if (status.state != ZY100_OFFLINE_V2_IMU_RUNNING)
    {
        return false;
    }
    return (status.pending_due != 0U) || status.flash_busy_active ||
           (status.last_fifo_count >=
            ZY100_OFFLINE_V2_IMU_FIFO_WATERMARK_BYTES);
}

static int16_t offline_v2_be_s16(const uint8_t *data, uint8_t high_index)
{
    return (int16_t)(((uint16_t)data[high_index] << 8) |
                     data[(uint8_t)(high_index + 1U)]);
}

static bool offline_v2_event_sink(const zy100_offline_v2_event_t *event,
                                  void *context)
{
    offline_v2_capture_runtime_t *runtime =
        (offline_v2_capture_runtime_t *)context;

    uint32_t slot;
    uint32_t lock_state;

    if ((runtime == NULL) || (event == NULL) ||
        (runtime->event_queue == NULL))
    {
        if (runtime != NULL)
        {
            runtime->quality.feature_drop_count++;
        }
        return false;
    }

    lock_state = os_lock();
    if (runtime->event_queue_count >= OFFLINE_V2_EVENT_QUEUE_DEPTH)
    {
        runtime->quality.feature_drop_count++;
        os_unlock(lock_state);
        return false;
    }
    slot = runtime->event_queue_write;
    os_unlock(lock_state);

    runtime->event_queue[slot] = *event;

    lock_state = os_lock();
    runtime->event_queue_write =
        (runtime->event_queue_write + 1U) % OFFLINE_V2_EVENT_QUEUE_DEPTH;
    runtime->event_queue_count++;
    if ((event->quality_flags &
         ZY100_OFFLINE_V2_EVENT_FLAG_Q12_SATURATION) != 0U)
    {
        runtime->quality.q12_clip_event_count++;
    }
    if (runtime->event_queue_count > runtime->event_queue_max_count)
    {
        runtime->event_queue_max_count = runtime->event_queue_count;
    }
    os_unlock(lock_state);
    return true;
}

static void offline_v2_release_resources(void)
{
    if (s_capture.mag_started)
    {
        (void)mag_capture_service_end(MAG_CAPTURE_OWNER_OFFLINE_V2);
        s_capture.mag_started = false;
    }
    if (s_capture.workspace_claimed)
    {
        (void)zy100_mode_workspace_release(
            ZY100_MODE_WORKSPACE_OWNER_OFFLINE_V2);
        s_capture.workspace_claimed = false;
    }
    (void)zy100_capture_profile_release(
        ZY100_CAPTURE_PROFILE_OFFLINE_V2);
    zy100_offline_v2_imu_session_prepare_sleep();
    s_capture.algo = NULL;
    s_capture.event_queue = NULL;
}

static void offline_v2_log_final_summary(uint32_t now_ms)
{
#if ZY100_OFFLINE_V2_TARGETED_LOG_ENABLE
    zy100_offline_v2_capture_diag_t diag;
    zy100_offline_v2_session_info_t info;
    uint32_t runtime_ms = now_ms - s_capture.start_ms;
    uint32_t imu_hz = (runtime_ms != 0U) ?
                      (uint32_t)(((uint64_t)s_capture.imu_sample_count *
                                  1000ULL) / runtime_ms) : 0U;
    uint32_t mag_hz = (runtime_ms != 0U) ?
                      (uint32_t)(((uint64_t)s_capture.mag_sample_count *
                                  1000ULL) / runtime_ms) : 0U;
    uint32_t actions =
        zy100_offline_v2_algo_event_count(s_capture.algo);
    uint32_t saved = zy100_offline_v2_storage_active_event_count();

    memset(&diag, 0, sizeof(diag));
    (void)zy100_offline_v2_capture_get_diag(&diag);
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][FINAL_A] reason=%u sid=%lu ms=%lu actions=%lu saved=%lu remain=%u",
        (uint32_t)s_capture.completed_reason,
        (unsigned long)zy100_offline_v2_storage_active_session_id(),
        (unsigned long)runtime_ms,
        (unsigned long)actions,
        (unsigned long)saved,
        zy100_offline_v2_storage_remaining_percent());
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][FINAL_B] imu=%lu hz=%lu mag=%lu hz=%lu gap=%lu drop=%lu qmax=%lu",
        (unsigned long)s_capture.imu_sample_count,
        (unsigned long)imu_hz,
        (unsigned long)s_capture.mag_sample_count,
        (unsigned long)mag_hz,
        (unsigned long)s_capture.quality.time_gap_count,
        (unsigned long)s_capture.quality.feature_drop_count,
        (unsigned long)diag.event_queue_max_level);
    ZY100_OFFLINE_V2_LOG(
        "[OFFLINE_V2][FINAL_C] push=%lu det=%lu shock=%lu feat=%lu mag_err=%lu flash_err=%lu",
        (unsigned long)diag.push_max_us,
        (unsigned long)diag.detector_max_us,
        (unsigned long)diag.shock_max_us,
        (unsigned long)diag.feature_max_us,
        (unsigned long)s_capture.quality.fifo_discard_count,
        (unsigned long)s_capture.quality.flash_error_count);
    memset(&info, 0, sizeof(info));
    if ((zy100_offline_v2_storage_session_count() != 0U) &&
        zy100_offline_v2_storage_session_get(
            zy100_offline_v2_storage_session_count() - 1U,
            &info))
    {
        ZY100_OFFLINE_V2_LOG(
            "[OFFLINE_V2][FINAL_D] reason=%u clean=%u health=%u end_committed=1 recovery=%u",
            (uint32_t)info.stop_reason,
            info.clean ? 1U : 0U,
            (uint32_t)info.health,
            (info.health == ZY100_OFFLINE_V2_HEALTH_RECOVERED_PREFIX) ?
            1U : 0U);
    }
#else
    zy100_offline_v2_session_info_t info;
    bool committed = false;
    memset(&info, 0, sizeof(info));
    if (zy100_offline_v2_storage_session_count() != 0U)
    {
        committed = zy100_offline_v2_storage_session_get(
            zy100_offline_v2_storage_session_count() - 1U, &info);
        committed = committed && (info.session_id ==
            zy100_offline_v2_storage_active_session_id());
    }
    ZY100_LOG_EVENT("[OFFLINE_END] reason=%u ms=%lu saved=%lu committed=%u health=%u",
                    (uint32_t)s_capture.completed_reason,
                    (unsigned long)(now_ms - s_capture.start_ms),
                    (unsigned long)zy100_offline_v2_storage_active_event_count(),
                    committed ? 1U : 0U, (uint32_t)info.health);
    ZY100_LOG_EVENT("[OFFLINE_QUALITY] imu=%lu mag=%lu gap=%lu drop=%lu fifo=%lu over=%lu flash=%lu",
                    (unsigned long)s_capture.imu_sample_count,
                    (unsigned long)s_capture.mag_sample_count,
                    (unsigned long)s_capture.quality.time_gap_count,
                    (unsigned long)s_capture.quality.feature_drop_count,
                    (unsigned long)s_capture.quality.fifo_discard_count,
                    (unsigned long)s_capture.quality.fifo_overflow_count,
                    (unsigned long)s_capture.quality.flash_error_count);
#endif
}

static void offline_v2_observer_window_offer(void)
{
    zy100_offline_v2_imu_status_t status;
    zy100_offline_v2_imu_cause_t cause;
    uint32_t lock_state;

    memset(&status, 0, sizeof(status));
    memset(&cause, 0, sizeof(cause));
    if ((s_capture.state != ZY100_OFFLINE_V2_CAPTURE_RUNNING) ||
        s_capture.stop_requested ||
        (offline_v2_event_queue_level() != 0U) ||
        zy100_offline_v2_storage_job_busy() ||
        !zy100_offline_v2_imu_session_get_status(&status) ||
        (status.state != ZY100_OFFLINE_V2_IMU_RUNNING) ||
        status.stop_requested ||
        (status.timer_running == false) ||
        (zy100_offline_v2_imu_session_get_first_cause(&cause) && cause.valid))
    {
        return;
    }
    lock_state = os_lock();
    s_capture.observer_window_generation++;
    if (s_capture.observer_window_generation == 0U)
    {
        s_capture.observer_window_generation = 1U;
    }
    s_capture.observer_window_open = true;
    os_unlock(lock_state);
}

static void offline_v2_fail(const char *reason)
{
    ZY100_LOG_ERROR("[OFFLINE_V2][ERR] capture_failed reason=%s state=%u",
               (reason != NULL) ? reason : "unknown",
               (uint32_t)s_capture.state);
    if (zy100_offline_v2_imu_session_active())
    {
        (void)zy100_offline_v2_imu_session_request_stop(
            ZY100_OFFLINE_V2_IMU_STOP_WORKER_EXIT);
    }
    offline_v2_release_resources();
    s_capture.state = ZY100_OFFLINE_V2_CAPTURE_ERROR;
    s_capture.completed_reason = (zy100_offline_v2_storage_state() ==
        ZY100_OFFLINE_V2_STORAGE_ERROR) ? ZY100_OFFLINE_V2_STOP_FLASH_IO_ERROR :
        s_capture.stop_reason;
    s_capture.completion_pending = true;
    zy100_offline_reset_trace_set_app_phase(ZY100_OFFLINE_TRACE_APP_ERROR);
}

bool zy100_offline_v2_capture_init(void)
{
    return zy100_offline_v2_capture_init_with_policy(ZY100_OFFLINE_V2_BOOT_NORMAL);
}

bool zy100_offline_v2_capture_init_with_policy(zy100_offline_v2_boot_policy_t policy)
{
    uint8_t *persistent;
    uint32_t persistent_bytes = 0U;

    if (s_capture.storage_initialized)
    {
        if (zy100_offline_v2_capture_active()) return false;
        if (zy100_offline_v2_storage_state() == ZY100_OFFLINE_V2_STORAGE_ERROR)
        {
            if (!zy100_offline_v2_storage_recover_begin()) return false;
        }
        else if (zy100_offline_v2_storage_drain_status() != ZY100_OFFLINE_V2_DRAIN_DONE)
        {
            return false;
        }
        s_capture.state = ZY100_OFFLINE_V2_CAPTURE_IDLE;
        s_capture.completion_pending = false;
        s_shutdown_active = false;
        s_shutdown_fault_exit = false;
        s_shutdown_recovering = false;
        return true;
    }
    memset(&s_capture, 0, sizeof(s_capture));
    s_capture.state = ZY100_OFFLINE_V2_CAPTURE_UNINITIALIZED;
    persistent = zy100_mode_workspace_persistent_base(&persistent_bytes);
    if (persistent == NULL) return false;
    /* Initialized means a module-owned context exists, not that it is healthy. */
    s_capture.storage_initialized = true;
    if (!zy100_offline_v2_storage_init_with_policy(persistent, persistent_bytes, policy))
    {
        s_capture.state = ZY100_OFFLINE_V2_CAPTURE_ERROR;
        return false;
    }
    s_capture.storage_initialized = true;
    s_capture.state = ZY100_OFFLINE_V2_CAPTURE_IDLE;
    return true;
}

zy100_offline_v2_start_status_t zy100_offline_v2_capture_start(
    uint64_t start_unix_ms,
    bool timebase_synced,
    uint32_t owner_user_id)
{
    uint8_t *workspace = NULL;
    uint32_t workspace_bytes = 0U;
    uint32_t algo_bytes;
    uint32_t algo_aligned_bytes;
    uint32_t required_bytes;

    if ((owner_user_id == 0U) ||
        !s_capture.storage_initialized ||
        (s_capture.state == ZY100_OFFLINE_V2_CAPTURE_UNINITIALIZED) ||
        (s_capture.state == ZY100_OFFLINE_V2_CAPTURE_ERROR))
    {
        return ZY100_OFFLINE_V2_START_NOT_READY;
    }
    if (!zy100_offline_v2_storage_ready_for_capture())
    {
        return (zy100_offline_v2_storage_remaining_percent() <= 5U) ?
            ZY100_OFFLINE_V2_START_FLASH_LOCKED :
            ZY100_OFFLINE_V2_START_NOT_READY;
    }
    if (!zy100_capture_profile_claim(
            ZY100_CAPTURE_PROFILE_OFFLINE_V2))
    {
        return ZY100_OFFLINE_V2_START_PROFILE_BUSY;
    }
    algo_bytes = zy100_offline_v2_algo_workspace_bytes();
    algo_aligned_bytes = OFFLINE_V2_ALIGN_UP_4(algo_bytes);
    required_bytes = algo_aligned_bytes +
        (OFFLINE_V2_EVENT_QUEUE_DEPTH * sizeof(zy100_offline_v2_event_t));
    if (!zy100_mode_workspace_claim(
            ZY100_MODE_WORKSPACE_OWNER_OFFLINE_V2,
            required_bytes,
            &workspace,
            &workspace_bytes))
    {
        (void)zy100_capture_profile_release(
            ZY100_CAPTURE_PROFILE_OFFLINE_V2);
        return ZY100_OFFLINE_V2_START_WORKSPACE_BUSY;
    }
    memset(&s_capture.quality, 0, sizeof(s_capture.quality));
    s_capture.algo = zy100_offline_v2_algo_init(
        workspace, algo_bytes, offline_v2_event_sink, &s_capture);
    if (s_capture.algo == NULL)
    {
        (void)zy100_mode_workspace_release(
            ZY100_MODE_WORKSPACE_OWNER_OFFLINE_V2);
        (void)zy100_capture_profile_release(
            ZY100_CAPTURE_PROFILE_OFFLINE_V2);
        return ZY100_OFFLINE_V2_START_INTERNAL_ERROR;
    }
    s_capture.event_queue = (zy100_offline_v2_event_t *)
        (workspace + algo_aligned_bytes);
    zy100_offline_v2_algo_set_time_source(s_capture.algo,
                                          offline_v2_time_us);
    s_capture.workspace_claimed = true;
    offline_v2_event_queue_reset();
    s_capture.push_last_us = 0U;
    s_capture.push_max_us = 0U;
    s_capture.start_ms = 0U;
    s_capture.stop_requested = false;
    s_capture.algo_finished = false;
    s_capture.imu_completion_consumed = false;
    s_capture.finalize_requested = false;
    s_capture.completion_pending = false;
    s_capture.mag_started = false;
    s_capture.imu_seen_active = false;
    s_capture.imu_timestamp_valid = false;
    s_capture.imu_sample_count = 0U;
    s_capture.mag_sample_count = 0U;
    s_capture.last_mag_poll_ms = 0U;
    s_capture.imu_time_defer_logged = false;
    s_capture.stop_reason = ZY100_OFFLINE_V2_STOP_USER;
    if (!zy100_offline_v2_storage_begin_session(start_unix_ms,
                                                  timebase_synced,
                                                  owner_user_id))
    {
        offline_v2_release_resources();
        return ZY100_OFFLINE_V2_START_INTERNAL_ERROR;
    }
    memset(&s_capture.identity, 0, sizeof(s_capture.identity));
    s_capture.elapsed_valid = false;
    s_capture.elapsed_final_ms = 0ULL;
    s_capture.identity.owner_user_id = owner_user_id;
    s_capture.identity.session_id = zy100_offline_v2_storage_active_session_id();
    s_capture.identity.storage_generation = zy100_offline_v2_storage_active_generation();
    s_capture.identity.start_unix_ms = timebase_synced ? start_unix_ms : 0ULL;
    s_capture.identity.flags = ZY100_TRAINING_FLAG_TOKEN_VALID |
        (timebase_synced ? ZY100_TRAINING_FLAG_TIME_VALID : 0U);
    s_capture.identity.source = 255U;
    {
        uint32_t index;
        for (index = 0U; index < ZY100_TRAINING_TOKEN_BYTES; index += 4U)
        {
            uint32_t random = get_true_random_number();
            memcpy(s_capture.identity.token + index, &random, sizeof(random));
        }
    }
    s_capture.state = ZY100_OFFLINE_V2_CAPTURE_STARTING;
    zy100_offline_reset_trace_arm();
    ZY100_OFFLINE_V2_LOG("[OFFLINE_V2][CAPTURE] start_pending session=%lu storage_state=%u remain=%u",
                        (unsigned long)zy100_offline_v2_storage_active_session_id(),
                        (uint32_t)zy100_offline_v2_storage_state(),
                        zy100_offline_v2_storage_remaining_percent());
    return ZY100_OFFLINE_V2_START_OK;
}

bool zy100_offline_v2_capture_request_stop(
    zy100_offline_v2_stop_reason_t reason)
{
    zy100_offline_v2_imu_stop_reason_t imu_reason =
        ZY100_OFFLINE_V2_IMU_STOP_USER;

    if ((s_capture.state != ZY100_OFFLINE_V2_CAPTURE_RUNNING) &&
        (s_capture.state != ZY100_OFFLINE_V2_CAPTURE_STARTING))
    {
        return false;
    }
    if (reason == ZY100_OFFLINE_V2_STOP_LOW_BATTERY)
    {
        imu_reason = ZY100_OFFLINE_V2_IMU_STOP_LOW_BATTERY;
    }
    else if (reason == ZY100_OFFLINE_V2_STOP_FLASH_FULL)
    {
        imu_reason = ZY100_OFFLINE_V2_IMU_STOP_FLASH_FULL;
    }
    else if (reason == ZY100_OFFLINE_V2_STOP_SYSTEM_SHUTDOWN)
    {
        imu_reason = ZY100_OFFLINE_V2_IMU_STOP_SYSTEM_SHUTDOWN;
    }
    else if (reason == ZY100_OFFLINE_V2_STOP_FIFO_FULL)
    {
        imu_reason = ZY100_OFFLINE_V2_IMU_STOP_FIFO_FULL;
    }
    else if (reason == ZY100_OFFLINE_V2_STOP_FIFO_LOST)
    {
        imu_reason = ZY100_OFFLINE_V2_IMU_STOP_FIFO_LOST;
    }
    else if (reason == ZY100_OFFLINE_V2_STOP_FIFO_BAD_HEADER)
    {
        imu_reason = ZY100_OFFLINE_V2_IMU_STOP_FIFO_BAD_HEADER;
    }
    else if (reason == ZY100_OFFLINE_V2_STOP_ALGO_ERROR)
    {
        imu_reason = ZY100_OFFLINE_V2_IMU_STOP_ALGORITHM;
    }
    else if ((reason == ZY100_OFFLINE_V2_STOP_FLASH_IO_ERROR) ||
             (reason == ZY100_OFFLINE_V2_STOP_SAMPLE_GAP_ABORT) ||
             (reason == ZY100_OFFLINE_V2_STOP_FIFO_READ_ERROR) ||
             (reason == ZY100_OFFLINE_V2_STOP_WORKER_EXIT) ||
             (reason == ZY100_OFFLINE_V2_STOP_SENSOR_START_ERROR) ||
             (reason == ZY100_OFFLINE_V2_STOP_MAG_FATAL))
    {
        imu_reason = ZY100_OFFLINE_V2_IMU_STOP_WORKER_EXIT;
    }
    s_capture.stop_reason = reason;
    s_capture.stop_requested = true;
    s_capture.state = ZY100_OFFLINE_V2_CAPTURE_STOPPING;
    zy100_offline_reset_trace_set_app_phase(
        ZY100_OFFLINE_TRACE_APP_STOPPING);
    ZY100_OFFLINE_V2_LOG("[OFFLINE_V2][CAPTURE] stop_requested reason=%u imu=%lu mag=%lu events=%lu",
                        (uint32_t)reason,
                        (unsigned long)s_capture.imu_sample_count,
                        (unsigned long)s_capture.mag_sample_count,
                        (unsigned long)zy100_offline_v2_storage_active_event_count());
    (void)zy100_offline_v2_imu_session_request_stop(imu_reason);
    return true;
}

static zy100_offline_v2_stop_reason_t offline_v2_map_imu_stop_reason(
    zy100_offline_v2_imu_stop_reason_t reason)
{
    switch (reason)
    {
    case ZY100_OFFLINE_V2_IMU_STOP_USER:
        return ZY100_OFFLINE_V2_STOP_USER;
    case ZY100_OFFLINE_V2_IMU_STOP_LOW_BATTERY:
        return ZY100_OFFLINE_V2_STOP_LOW_BATTERY;
    case ZY100_OFFLINE_V2_IMU_STOP_FLASH_FULL:
        return ZY100_OFFLINE_V2_STOP_FLASH_FULL;
    case ZY100_OFFLINE_V2_IMU_STOP_SYSTEM_SHUTDOWN:
        return ZY100_OFFLINE_V2_STOP_SYSTEM_SHUTDOWN;
    case ZY100_OFFLINE_V2_IMU_STOP_SENSOR_START:
        return ZY100_OFFLINE_V2_STOP_SENSOR_START_ERROR;
    case ZY100_OFFLINE_V2_IMU_STOP_FIFO_FULL:
        return ZY100_OFFLINE_V2_STOP_FIFO_FULL;
    case ZY100_OFFLINE_V2_IMU_STOP_FIFO_LOST:
        return ZY100_OFFLINE_V2_STOP_FIFO_LOST;
    case ZY100_OFFLINE_V2_IMU_STOP_FIFO_BAD_HEADER:
        return ZY100_OFFLINE_V2_STOP_FIFO_BAD_HEADER;
    case ZY100_OFFLINE_V2_IMU_STOP_FIFO_READ:
        return ZY100_OFFLINE_V2_STOP_FIFO_READ_ERROR;
    case ZY100_OFFLINE_V2_IMU_STOP_ALGORITHM:
        return ZY100_OFFLINE_V2_STOP_ALGO_ERROR;
    case ZY100_OFFLINE_V2_IMU_STOP_WORKER_EXIT:
    case ZY100_OFFLINE_V2_IMU_STOP_NONE:
    default:
        return ZY100_OFFLINE_V2_STOP_WORKER_EXIT;
    }
}

static bool offline_v2_stop_is_clean(zy100_offline_v2_stop_reason_t reason)
{
    return (reason == ZY100_OFFLINE_V2_STOP_USER) ||
           (reason == ZY100_OFFLINE_V2_STOP_LOW_BATTERY) ||
           (reason == ZY100_OFFLINE_V2_STOP_FLASH_FULL) ||
           (reason == ZY100_OFFLINE_V2_STOP_SYSTEM_SHUTDOWN);
}

static void offline_v2_consume_imu_completion(void)
{
    zy100_offline_v2_imu_status_t status;
    zy100_offline_v2_imu_stop_reason_t reason;

    if (s_capture.imu_completion_consumed)
    {
        return;
    }
    memset(&status, 0, sizeof(status));
    if (zy100_offline_v2_imu_session_get_status(&status))
    {
        s_capture.quality.fifo_overflow_count +=
            status.fifo_full_count + status.fifo_lost_count;
        s_capture.quality.fifo_discard_count +=
            status.bad_header_count + status.read_error_count;
    }
    if (zy100_offline_v2_imu_session_take_completion(&reason))
    {
        s_capture.imu_completion_consumed = true;
    }
}

static void offline_v2_abort_imu_health(const char *reason,
                                        uint32_t now_ms,
                                        uint32_t last_progress_ms,
                                        uint32_t idle_ms)
{
    zy100_offline_v2_capture_diag_t diag;
    zy100_offline_v2_imu_status_t imu_status;
    zy100_offline_v2_imu_cause_t cause;

    (void)zy100_offline_v2_capture_get_diag(&diag);
    memset(&imu_status, 0, sizeof(imu_status));
    memset(&cause, 0, sizeof(cause));
    (void)zy100_offline_v2_imu_session_get_status(&imu_status);
    (void)zy100_offline_v2_imu_session_get_first_cause(&cause);
    ZY100_LOG_ERROR("[OFFLINE_V2][IMU_ABORT_A] why=%s now=%lu last=%lu idle=%lu",
                        (reason != NULL) ? reason : "unknown",
                        (unsigned long)now_ms,
                        (unsigned long)last_progress_ms,
                        (unsigned long)idle_ms);
    ZY100_OFFLINE_V2_LOG("[OFFLINE_V2][IMU_ABORT_B] state=%u active=%u stop=%u timer=%u cause=%u",
                        (uint32_t)imu_status.state,
                        zy100_offline_v2_imu_session_active() ? 1U : 0U,
                        imu_status.stop_requested ? 1U : 0U,
                        imu_status.timer_running ? 1U : 0U,
                        (uint32_t)cause.origin);
    ZY100_OFFLINE_V2_LOG("[OFFLINE_V2][IMU_ABORT_C] gen=%lu start_gen=%lu last_stop=%u algo=%u q=%lu drop=%lu",
                        (unsigned long)imu_status.completion_generation,
                        (unsigned long)s_capture.imu_start_generation,
                        (uint32_t)imu_status.stop_reason,
                        (uint32_t)diag.algo_status,
                        (unsigned long)diag.event_queue_level,
                        (unsigned long)diag.feature_drop_count);
    (void)zy100_offline_v2_capture_request_stop(
        ZY100_OFFLINE_V2_STOP_WORKER_EXIT);
}

/* Called by the first sample or the RUNNING observation, whichever wins. */
static void offline_v2_note_sampling_started(uint32_t now_ms)
{
    if (!s_capture.elapsed_valid)
    {
        s_capture.start_ms = now_ms;
        s_capture.elapsed_start_ms = os_sys_time_get();
        s_capture.elapsed_valid = true;
    }
    s_capture.imu_seen_active = true;
}

static bool offline_v2_poll_imu_health(uint32_t now_ms)
{
    zy100_offline_v2_imu_status_t status;
    uint32_t health_now_ms;
    uint32_t elapsed_ms;

    memset(&status, 0, sizeof(status));
    if (!zy100_offline_v2_imu_session_get_status(&status))
    {
        health_now_ms = zy100_os_time_ms();
        elapsed_ms = 0U;
        (void)offline_v2_elapsed_ms_valid(
            health_now_ms, s_capture.imu_last_progress_ms, &elapsed_ms);
        offline_v2_abort_imu_health("status", health_now_ms,
                                    s_capture.imu_last_progress_ms,
                                    elapsed_ms);
        return false;
    }
    health_now_ms = zy100_os_time_ms();
    if (status.state == ZY100_OFFLINE_V2_IMU_RUNNING)
    {
        offline_v2_note_sampling_started(now_ms);
    }
    if ((status.state == ZY100_OFFLINE_V2_IMU_DONE) ||
        (status.state == ZY100_OFFLINE_V2_IMU_ERROR) ||
        ((status.completion_generation != s_capture.imu_start_generation) &&
         offline_v2_imu_quiescent()))
    {
        s_capture.stop_reason = offline_v2_map_imu_stop_reason(
            status.stop_reason);
        s_capture.stop_requested = true;
        s_capture.state = ZY100_OFFLINE_V2_CAPTURE_STOPPING;
        ZY100_OFFLINE_V2_LOG(
            "[OFFLINE_V2][IMU] completion reason=%u mapped=%u gen=%lu",
            (uint32_t)status.stop_reason,
            (uint32_t)s_capture.stop_reason,
            (unsigned long)status.completion_generation);
        return false;
    }
    if (!s_capture.imu_seen_active)
    {
        if (offline_v2_elapsed_ms_valid(health_now_ms,
                                        s_capture.imu_start_ms,
                                        &elapsed_ms) &&
            (elapsed_ms >= OFFLINE_V2_IMU_START_TIMEOUT_MS))
        {
            offline_v2_abort_imu_health("start_timeout", health_now_ms,
                                        s_capture.imu_start_ms,
                                        elapsed_ms);
            return false;
        }
        return true;
    }
    if (s_capture.imu_seen_active &&
        !offline_v2_elapsed_ms_valid(health_now_ms,
                                     status.last_progress_ms,
                                     &elapsed_ms))
    {
        offline_v2_log_time_defer_once(health_now_ms,
                                       status.last_progress_ms);
        return true;
    }
    if (s_capture.imu_seen_active &&
        (elapsed_ms >= OFFLINE_V2_IMU_STALL_TIMEOUT_MS))
    {
        offline_v2_abort_imu_health("sample_stall", health_now_ms,
                                    status.last_progress_ms,
                                    elapsed_ms);
        return false;
    }
    return true;
}

bool zy100_offline_v2_capture_push_imu_packet(const uint8_t packet[16],
                                              uint16_t timestamp_raw)
{
    zy100_offline_v2_imu_sample_t sample;
    zy100_offline_v2_algo_status_t status;
    bool discontinuity = false;
    uint16_t delta;
    uint32_t push_start_us;
    uint32_t push_elapsed_us;

    if ((packet == NULL) ||
        ((s_capture.state != ZY100_OFFLINE_V2_CAPTURE_STARTING) &&
         (s_capture.state != ZY100_OFFLINE_V2_CAPTURE_RUNNING) &&
         (s_capture.state != ZY100_OFFLINE_V2_CAPTURE_STOPPING)) ||
        (s_capture.algo == NULL))
    {
        return false;
    }
    sample.axis[0] = offline_v2_be_s16(packet, 1U);
    sample.axis[1] = offline_v2_be_s16(packet, 3U);
    sample.axis[2] = offline_v2_be_s16(packet, 5U);
    sample.axis[3] = offline_v2_be_s16(packet, 7U);
    sample.axis[4] = offline_v2_be_s16(packet, 9U);
    sample.axis[5] = offline_v2_be_s16(packet, 11U);
    if (s_capture.imu_timestamp_valid)
    {
        delta = (uint16_t)(timestamp_raw - s_capture.last_timestamp_raw);
        discontinuity = (delta < OFFLINE_V2_IMU_DT_MIN_US) ||
                        (delta > OFFLINE_V2_IMU_DT_MAX_US);
        if (discontinuity)
        {
            s_capture.quality.time_gap_count++;
        }
    }
    s_capture.last_timestamp_raw = timestamp_raw;
    s_capture.imu_timestamp_valid = true;
    push_start_us = offline_v2_time_us();
    status = zy100_offline_v2_algo_push(s_capture.algo, &sample,
                                        discontinuity);
    push_elapsed_us = offline_v2_time_us() - push_start_us;
    s_capture.push_last_us = push_elapsed_us;
    if (push_elapsed_us > s_capture.push_max_us)
    {
        s_capture.push_max_us = push_elapsed_us;
    }
    if (status != ZY100_OFFLINE_V2_ALGO_OK)
    {
        if (status != ZY100_OFFLINE_V2_ALGO_EVENT_SINK_FULL)
        {
            s_capture.quality.feature_drop_count++;
        }
        s_capture.stop_reason = ZY100_OFFLINE_V2_STOP_ALGO_ERROR;
        s_capture.stop_requested = true;
        return false;
    }
    s_capture.imu_sample_count++;
    offline_v2_note_sampling_started(zy100_os_time_ms());
    s_capture.imu_last_progress_ms = zy100_os_time_ms();
    return true;
}

static bool offline_v2_prepare_sensors_and_start_imu(uint32_t now_ms)
{
    mmc5603_cfg_t cfg;
    zy100_offline_v2_imu_status_t imu_status;

    if (!zy100_offline_v2_imu_session_prepare())
    {
        return false;
    }
    cfg.auto_sr_enable = true;
    cfg.bw = MMC5603_BW_LEVEL_01;
    cfg.continuous_odr = 0U;
    cfg.continuous_hpower = false;
    if (mag_capture_service_begin(MAG_CAPTURE_OWNER_OFFLINE_V2,
                                  &cfg,
                                  ZY100_OFFLINE_V2_MAG_ODR_HZ,
                                  false) != MAG_STATUS_OK)
    {
        return false;
    }
    s_capture.mag_started = true;
    s_capture.last_mag_poll_ms = now_ms;
    /* Snapshot before notify: the higher-priority worker may finish immediately. */
    memset(&imu_status, 0, sizeof(imu_status));
    if (!zy100_offline_v2_imu_session_get_status(&imu_status))
    {
        (void)mag_capture_service_end(MAG_CAPTURE_OWNER_OFFLINE_V2);
        s_capture.mag_started = false;
        return false;
    }
    s_capture.imu_start_generation = imu_status.completion_generation;
    s_capture.start_ms = now_ms;
    s_capture.elapsed_valid = false;
    s_capture.imu_start_ms = now_ms;
    s_capture.imu_last_progress_ms = now_ms;

    s_capture.imu_seen_active = false;
    if (!zy100_offline_v2_imu_session_start())
    {
        (void)mag_capture_service_end(MAG_CAPTURE_OWNER_OFFLINE_V2);
        s_capture.mag_started = false;
        return false;
    }
    zy100_offline_reset_trace_set_app_phase(
        ZY100_OFFLINE_TRACE_APP_RUNNING);
    ZY100_LOG_EVENT("[OFFLINE_V2][SENSOR] start_requested imu_hz=%u mag_hz=%u mag_poll_ms=%u generation=%lu session=%lu",
                        (uint32_t)ZY100_OFFLINE_V2_IMU_SAMPLE_RATE_HZ,
                        (uint32_t)ZY100_OFFLINE_V2_MAG_ODR_HZ,
                        (uint32_t)ZY100_OFFLINE_V2_MAG_POLL_INTERVAL_MS,
                        (unsigned long)s_capture.imu_start_generation,
                        (unsigned long)zy100_offline_v2_storage_active_session_id());
    return true;
}

static void offline_v2_poll_mag(uint32_t now_ms)
{
    mag_capture_sample_t sample;
    bool fresh = false;
    mag_status_t status;

    if (!s_capture.mag_started ||
        ((uint32_t)(now_ms - s_capture.last_mag_poll_ms) <
         ZY100_OFFLINE_V2_MAG_POLL_INTERVAL_MS))
    {
        return;
    }
    s_capture.last_mag_poll_ms = now_ms;
    status = mag_capture_service_read_fresh(false, &sample, &fresh);
    if ((status == MAG_STATUS_OK) && fresh)
    {
        /* V2 freezes the 128-D IMU feature ABI.  The synchronized 100 Hz MAG
         * sample reaches this adapter boundary but is intentionally not an
         * input to the first feature revision and is not persisted as RAW. */
        s_capture.mag_sample_count++;
    }
    else if (status != MAG_STATUS_OK)
    {
        s_capture.quality.fifo_discard_count++;
    }
}

void zy100_offline_v2_capture_poll(uint32_t now_ms)
{
    bool clean;
    bool fifo_urgent;
    const zy100_offline_v2_event_t *queued_event;
    zy100_offline_v2_algo_status_t finish_status;

    s_capture.observer_window_open = false;
    if (!s_capture.storage_initialized)
    {
        return;
    }
    zy100_offline_v2_imu_session_poll(now_ms);
    fifo_urgent = offline_v2_flash_fifo_urgent();
    zy100_offline_v2_storage_poll(fifo_urgent);

    if (!s_capture.last_logged_state_valid ||
        (s_capture.last_logged_state != s_capture.state))
    {
        ZY100_LOG_ROUTINE(ZY100_LOG_EVENT, "[OFFLINE_V2][CAPTURE] state=%u session=%lu storage=%u job=%u",
                            (uint32_t)s_capture.state,
                            (unsigned long)zy100_offline_v2_storage_active_session_id(),
                            (uint32_t)zy100_offline_v2_storage_state(),
                            (uint32_t)zy100_offline_v2_storage_job_code());
        s_capture.last_logged_state = s_capture.state;
        s_capture.last_logged_state_valid = true;
    }
    if ((zy100_offline_v2_storage_state() ==
         ZY100_OFFLINE_V2_STORAGE_ERROR) &&
        zy100_offline_v2_capture_active())
    {
        s_capture.stop_reason = ZY100_OFFLINE_V2_STOP_FLASH_IO_ERROR;
        s_capture.stop_requested = true;
        if (!offline_v2_imu_quiescent())
        {
            (void)zy100_offline_v2_imu_session_request_stop(
                ZY100_OFFLINE_V2_IMU_STOP_WORKER_EXIT);
            return;
        }
        /* The Flash owner is faulted, so no further END write is safe.  The
         * next boot recovery closes only the verified continuous prefix. */
        offline_v2_fail("storage_io");
        return;
    }

    if ((s_capture.state == ZY100_OFFLINE_V2_CAPTURE_IDLE) &&
        (zy100_offline_v2_storage_state() ==
         ZY100_OFFLINE_V2_STORAGE_LOCKED_5_PERCENT))
    {
        s_capture.state = ZY100_OFFLINE_V2_CAPTURE_LOCKED;
    }
    else if ((s_capture.state == ZY100_OFFLINE_V2_CAPTURE_LOCKED) &&
             zy100_offline_v2_storage_ready_for_capture())
    {
        s_capture.state = ZY100_OFFLINE_V2_CAPTURE_IDLE;
    }

    if (s_capture.state == ZY100_OFFLINE_V2_CAPTURE_STARTING)
    {
        if (zy100_offline_v2_storage_session_active() &&
            !zy100_offline_v2_storage_job_busy())
        {
            if (!offline_v2_prepare_sensors_and_start_imu(now_ms))
            {
                /* BEGIN is already durable.  Close it explicitly so a sensor
                 * start failure does not leave an open session until reboot. */
                s_capture.start_ms = now_ms;
                (void)zy100_offline_v2_capture_request_stop(
                    ZY100_OFFLINE_V2_STOP_SENSOR_START_ERROR);
                return;
            }
            s_capture.state = ZY100_OFFLINE_V2_CAPTURE_RUNNING;
        }
        return;
    }

    if (s_capture.state == ZY100_OFFLINE_V2_CAPTURE_RUNNING)
    {
        if (!offline_v2_poll_imu_health(now_ms))
        {
            return;
        }
        offline_v2_poll_mag(now_ms);
        queued_event = offline_v2_event_queue_peek();
        if (!offline_v2_flash_fifo_urgent() &&
            (queued_event != NULL) &&
            !zy100_offline_v2_storage_job_busy() &&
            zy100_offline_v2_storage_append_event(queued_event))
        {
            offline_v2_event_queue_pop();
        }
        if (!offline_v2_flash_fifo_urgent() &&
            (offline_v2_event_queue_level() == 0U) &&
            !zy100_offline_v2_storage_job_busy())
        {
            (void)zy100_offline_v2_storage_checkpoint(now_ms);
        }
        if (zy100_offline_v2_storage_remaining_percent() <= 5U)
        {
            (void)zy100_offline_v2_capture_request_stop(
                ZY100_OFFLINE_V2_STOP_FLASH_FULL);
            return;
        }
        if (s_capture.stop_requested)
        {
            (void)zy100_offline_v2_capture_request_stop(s_capture.stop_reason);
            return;
        }
        offline_v2_observer_window_offer();
        return;
    }

    if (s_capture.state == ZY100_OFFLINE_V2_CAPTURE_STOPPING)
    {
        if (!offline_v2_imu_quiescent())
        {
            return;
        }
        offline_v2_consume_imu_completion();
        if (s_capture.mag_started)
        {
            (void)mag_capture_service_end(MAG_CAPTURE_OWNER_OFFLINE_V2);
            s_capture.mag_started = false;
        }
        if (!s_capture.algo_finished)
        {
            clean = offline_v2_stop_is_clean(s_capture.stop_reason);
            finish_status = zy100_offline_v2_algo_finish(s_capture.algo,
                                                         clean);
            if (finish_status != ZY100_OFFLINE_V2_ALGO_OK)
            {
                if (finish_status !=
                    ZY100_OFFLINE_V2_ALGO_EVENT_SINK_FULL)
                {
                    s_capture.quality.feature_drop_count++;
                }
                s_capture.stop_reason =
                    ZY100_OFFLINE_V2_STOP_ALGO_ERROR;
            }
            s_capture.algo_finished = true;
        }
        queued_event = offline_v2_event_queue_peek();
        if (queued_event != NULL)
        {
            if (!zy100_offline_v2_storage_job_busy() &&
                zy100_offline_v2_storage_append_event(queued_event))
            {
                offline_v2_event_queue_pop();
            }
            return;
        }
        if (zy100_offline_v2_storage_job_busy())
        {
            return;
        }
        clean = offline_v2_stop_is_clean(s_capture.stop_reason);
        s_capture.elapsed_final_ms = s_capture.elapsed_valid ?
            os_sys_time_get() - s_capture.elapsed_start_ms : 0ULL;
        if (!zy100_offline_v2_storage_finalize_session(
                s_capture.stop_reason,
                clean,
                s_capture.elapsed_final_ms > UINT32_MAX ? UINT32_MAX :
                    (uint32_t)s_capture.elapsed_final_ms,
                &s_capture.quality))
        {
            offline_v2_fail("finalize_begin");
            return;
        }
        s_capture.finalize_requested = true;
        s_capture.state = ZY100_OFFLINE_V2_CAPTURE_FINALIZING;
        zy100_offline_reset_trace_set_app_phase(
            ZY100_OFFLINE_TRACE_APP_FINALIZING);
        return;
    }

    if (s_capture.state == ZY100_OFFLINE_V2_CAPTURE_FINALIZING)
    {
        if (zy100_offline_v2_storage_job_busy() ||
            zy100_offline_v2_storage_session_active())
        {
            return;
        }
        s_capture.completed_reason = s_capture.stop_reason;
        s_capture.completion_pending = true;
        offline_v2_log_final_summary(now_ms);
        offline_v2_release_resources();
        s_capture.state =
            (zy100_offline_v2_storage_remaining_percent() <= 5U) ?
            ZY100_OFFLINE_V2_CAPTURE_LOCKED :
            ZY100_OFFLINE_V2_CAPTURE_IDLE;
        zy100_online_reset_trace_clear();
    }
}

zy100_offline_v2_capture_state_t zy100_offline_v2_capture_state(void)
{
    return s_capture.state;
}

bool zy100_offline_v2_capture_active(void)
{
    return (s_capture.state == ZY100_OFFLINE_V2_CAPTURE_STARTING) ||
           (s_capture.state == ZY100_OFFLINE_V2_CAPTURE_RUNNING) ||
           (s_capture.state == ZY100_OFFLINE_V2_CAPTURE_STOPPING) ||
           (s_capture.state == ZY100_OFFLINE_V2_CAPTURE_FINALIZING);
}

bool zy100_offline_v2_capture_input_active(void)
{
    return s_capture.state == ZY100_OFFLINE_V2_CAPTURE_RUNNING && s_capture.imu_seen_active;
}

bool zy100_offline_v2_capture_locked(void)
{
    return s_capture.state == ZY100_OFFLINE_V2_CAPTURE_LOCKED;
}

uint8_t zy100_offline_v2_capture_remaining_percent(void)
{
    return zy100_offline_v2_storage_remaining_percent();
}

uint32_t zy100_offline_v2_capture_session_count(void)
{
    return zy100_offline_v2_storage_session_count();
}

bool zy100_offline_v2_capture_take_completion(
    zy100_offline_v2_stop_reason_t *reason_out)
{
    if (!s_capture.completion_pending || (reason_out == NULL))
    {
        return false;
    }
    *reason_out = s_capture.completed_reason;
    s_capture.completion_pending = false;
    return true;
}

bool zy100_offline_v2_capture_get_diag(
    zy100_offline_v2_capture_diag_t *diag_out)
{
    zy100_offline_v2_algo_timing_t timing;
    zy100_offline_v2_imu_status_t imu_status;
    zy100_offline_v2_imu_cause_t imu_cause;
    uint32_t lock_state;

    if (diag_out == NULL)
    {
        return false;
    }
    memset(diag_out, 0, sizeof(*diag_out));
    diag_out->algo_status = zy100_offline_v2_algo_status(s_capture.algo);
    diag_out->feature_drop_count = s_capture.quality.feature_drop_count;
    diag_out->push_last_us = s_capture.push_last_us;
    diag_out->push_max_us = s_capture.push_max_us;
    diag_out->imu_sample_count = s_capture.imu_sample_count;
    diag_out->mag_sample_count = s_capture.mag_sample_count;
    memset(&imu_status, 0, sizeof(imu_status));
    memset(&imu_cause, 0, sizeof(imu_cause));
    if (zy100_offline_v2_imu_session_get_status(&imu_status))
    {
        diag_out->imu_fifo_max_bytes = imu_status.max_fifo_count;
        diag_out->imu_fifo_full_count = imu_status.fifo_full_count;
        diag_out->imu_fifo_lost_count = imu_status.fifo_lost_count;
        diag_out->imu_fifo_bad_header_count =
            imu_status.bad_header_count;
        diag_out->imu_fifo_read_error_count =
            imu_status.read_error_count;
    }
    if (zy100_offline_v2_imu_session_get_first_cause(&imu_cause))
    {
        diag_out->imu_cause_origin = (uint32_t)imu_cause.origin;
        diag_out->imu_cause_detail = imu_cause.detail;
    }
    lock_state = os_lock();
    diag_out->event_queue_level = s_capture.event_queue_count;
    diag_out->event_queue_max_level = s_capture.event_queue_max_count;
    os_unlock(lock_state);
    if (zy100_offline_v2_algo_get_timing(s_capture.algo, &timing))
    {
        diag_out->detector_last_us = timing.detector_last_us;
        diag_out->detector_max_us = timing.detector_max_us;
        diag_out->shock_last_us = timing.shock_last_us;
        diag_out->shock_max_us = timing.shock_max_us;
        diag_out->feature_last_us = timing.feature_last_us;
        diag_out->feature_max_us = timing.feature_max_us;
        diag_out->local_peak_count = timing.local_peak_count;
        diag_out->legal_peak_count = timing.legal_peak_count;
        diag_out->nms_selected_count = timing.nms_selected_count;
        diag_out->shock_rejected_count = timing.shock_rejected_count;
        diag_out->rejected_window_count = timing.rejected_window_count;
    }
    return s_capture.algo != NULL;
}

bool zy100_offline_v2_capture_observer_window_take(
    uint32_t *generation_out,
    zy100_offline_v2_observer_window_result_t *result_out)
{
    zy100_offline_v2_imu_status_t status;
    zy100_offline_v2_imu_cause_t cause;
    uint32_t lock_state;
    uint32_t queue_level;

    if (result_out != NULL)
    {
        *result_out = ZY100_OFFLINE_V2_OBSERVER_WINDOW_NOT_RUNNING;
    }
    if (s_capture.state != ZY100_OFFLINE_V2_CAPTURE_RUNNING)
    {
        if ((result_out != NULL) && zy100_offline_v2_capture_active())
        {
            *result_out = ZY100_OFFLINE_V2_OBSERVER_WINDOW_STOPPING;
        }
        return false;
    }
    if (s_capture.stop_requested)
    {
        if (result_out != NULL)
        {
            *result_out = ZY100_OFFLINE_V2_OBSERVER_WINDOW_STOPPING;
        }
        return false;
    }
    queue_level = offline_v2_event_queue_level();
    if (queue_level != 0U)
    {
        if (result_out != NULL)
        {
            *result_out = ZY100_OFFLINE_V2_OBSERVER_WINDOW_EVENT_QUEUE;
        }
        return false;
    }
    if (zy100_offline_v2_storage_job_busy())
    {
        if (result_out != NULL)
        {
            *result_out = ZY100_OFFLINE_V2_OBSERVER_WINDOW_STORAGE_BUSY;
        }
        return false;
    }
    memset(&cause, 0, sizeof(cause));
    if (zy100_offline_v2_imu_session_get_first_cause(&cause) && cause.valid)
    {
        if (result_out != NULL)
        {
            *result_out = ZY100_OFFLINE_V2_OBSERVER_WINDOW_FIRST_CAUSE;
        }
        return false;
    }
    memset(&status, 0, sizeof(status));
    if (!zy100_offline_v2_imu_session_get_status(&status) ||
        (status.state != ZY100_OFFLINE_V2_IMU_RUNNING) ||
        status.stop_requested || !status.timer_running)
    {
        if (result_out != NULL)
        {
            *result_out = ZY100_OFFLINE_V2_OBSERVER_WINDOW_IMU_BUSY;
        }
        return false;
    }
    lock_state = os_lock();
    if (!s_capture.observer_window_open)
    {
        os_unlock(lock_state);
        if (result_out != NULL)
        {
            *result_out = ZY100_OFFLINE_V2_OBSERVER_WINDOW_CONSUMED;
        }
        return false;
    }
    s_capture.observer_window_open = false;
    if (generation_out != NULL)
    {
        *generation_out = s_capture.observer_window_generation;
    }
    os_unlock(lock_state);
    if (result_out != NULL)
    {
        *result_out = ZY100_OFFLINE_V2_OBSERVER_WINDOW_OK;
    }
    return true;
}

void zy100_offline_v2_capture_shutdown_begin(void)
{
    if (s_shutdown_active) return;
    s_shutdown_active = true;
    s_shutdown_recovering = false;
    s_shutdown_fault_exit = false;
    s_shutdown_attempts = 0U;
    s_shutdown_hw_failures = 0U;
    s_shutdown_retry_ms = 0U;
}

zy100_offline_v2_drain_status_t zy100_offline_v2_capture_shutdown_status(void)
{
    zy100_offline_v2_drain_status_t storage;
    if (zy100_offline_v2_capture_active() || !offline_v2_imu_quiescent() ||
        s_capture.workspace_claimed || s_capture.mag_started)
        return ZY100_OFFLINE_V2_DRAIN_BUSY;
    storage = zy100_offline_v2_storage_drain_status();
    if (storage != ZY100_OFFLINE_V2_DRAIN_DONE) return storage;
    if (s_capture.state == ZY100_OFFLINE_V2_CAPTURE_ERROR || s_shutdown_recovering)
        return ZY100_OFFLINE_V2_DRAIN_ERROR;
    return ZY100_OFFLINE_V2_DRAIN_DONE;
}

void zy100_offline_v2_capture_shutdown_poll(uint32_t now_ms)
{
    if (!s_shutdown_active || !s_capture.storage_initialized) return;
    /* Also pump when runtime startup failed and the normal loop is inactive. */
    zy100_offline_v2_capture_poll(now_ms);
    if (zy100_offline_v2_capture_active() || !offline_v2_imu_quiescent() ||
        s_capture.workspace_claimed || s_capture.mag_started) return;
    if (s_shutdown_fault_exit) return;
    if (zy100_offline_v2_storage_drain_status() == ZY100_OFFLINE_V2_DRAIN_BUSY) return;
    if (zy100_offline_v2_storage_state() != ZY100_OFFLINE_V2_STORAGE_ERROR)
    {
        if (s_shutdown_recovering || s_capture.state == ZY100_OFFLINE_V2_CAPTURE_ERROR)
        {
            s_shutdown_recovering = false;
            s_capture.state = ZY100_OFFLINE_V2_CAPTURE_IDLE;
            ZY100_LOG_EVENT("[SHUTDOWN] offline_recovered attempts=%u", s_shutdown_attempts);
        }
        return;
    }
    if (s_shutdown_attempts >= 3U || s_shutdown_hw_failures >= 3U ||
        !zy100_offline_v2_storage_recovery_retryable())
    {
        s_shutdown_fault_exit = true;
        ZY100_LOG_ERROR("[SHUTDOWN] offline_fault_exit job=%u anchor=%u",
            zy100_offline_v2_storage_failed_job(),
            zy100_offline_v2_storage_recovery_anchored() ? 1U : 0U);
        return;
    }
    if ((s_shutdown_attempts != 0U || s_shutdown_hw_failures != 0U) &&
        (uint32_t)(now_ms - s_shutdown_retry_ms) < 1000U) return;
    {
        zy100_offline_v2_hw_status_t hw = zy100_offline_v2_storage_hardware_status();
        if (hw == ZY100_OFFLINE_V2_HW_BUSY) return;
        if (hw == ZY100_OFFLINE_V2_HW_UNKNOWN)
        {
            ++s_shutdown_hw_failures;
            s_shutdown_retry_ms = now_ms;
            return;
        }
        s_shutdown_hw_failures = 0U;
    }
    ++s_shutdown_attempts;
    s_shutdown_retry_ms = now_ms;
    s_shutdown_recovering = true;
    ZY100_LOG_EVENT("[SHUTDOWN] offline_recovery_begin attempt=%u job=%u",
        s_shutdown_attempts, zy100_offline_v2_storage_failed_job());
    (void)zy100_offline_v2_storage_recover_begin();
}

bool zy100_offline_v2_capture_fault_exit(void) { return s_shutdown_fault_exit; }
bool zy100_offline_v2_capture_fault_drained(void)
{
    /* Software drain proof only. The shutdown owner must separately verify
     * hardware idle before DPD/power-off, including recovery from unknown SR1. */
    return s_shutdown_fault_exit && !zy100_offline_v2_capture_active() &&
        offline_v2_imu_quiescent() && !s_capture.workspace_claimed && !s_capture.mag_started &&
        zy100_offline_v2_storage_recovery_anchored();
}

bool zy100_offline_v2_capture_fault_sleep_safe(void)
{
    return zy100_offline_v2_capture_fault_drained() &&
        zy100_offline_v2_storage_hardware_status() == ZY100_OFFLINE_V2_HW_IDLE;
}

void zy100_offline_v2_capture_prepare_wake(void)
{
    /* Only the authorized wake entry calls this; persistent storage health is
     * deliberately retained until strict recovery has verified it. */
    s_shutdown_active = false;
    s_shutdown_fault_exit = false;
    s_shutdown_recovering = false;
    s_shutdown_attempts = 0U;
    s_shutdown_hw_failures = 0U;
}

/* Identity/timing fields are written by the app task, not the IMU worker. */
bool zy100_offline_v2_capture_snapshot(zy100_training_snapshot_t *out)
{
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    out->source = 255U;
    if (!zy100_offline_v2_capture_active())
    {
        if (s_capture.state == ZY100_OFFLINE_V2_CAPTURE_ERROR) out->phase = 7U;
        return true;
    }
    *out = s_capture.identity;
    out->phase = (uint8_t)s_capture.state;
    out->flags |= ZY100_TRAINING_FLAG_ACTIVE;
    if (s_capture.elapsed_valid)
    {
        out->flags |= ZY100_TRAINING_FLAG_ELAPSED_VALID;
        out->elapsed_ms = s_capture.finalize_requested ? s_capture.elapsed_final_ms :
            os_sys_time_get() - s_capture.elapsed_start_ms;
    }
    return true;
}

void zy100_offline_v2_capture_set_start_source(uint8_t source)
{
    if (s_capture.state == ZY100_OFFLINE_V2_CAPTURE_STARTING)
        s_capture.identity.source = source;
}

bool zy100_offline_v2_capture_token_matches(uint32_t user_id, const uint8_t token[16])
{
    return token != NULL && user_id != 0U &&
        user_id == s_capture.identity.owner_user_id &&
        (s_capture.identity.flags & ZY100_TRAINING_FLAG_TOKEN_VALID) != 0U &&
        memcmp(token, s_capture.identity.token, ZY100_TRAINING_TOKEN_BYTES) == 0;
}
