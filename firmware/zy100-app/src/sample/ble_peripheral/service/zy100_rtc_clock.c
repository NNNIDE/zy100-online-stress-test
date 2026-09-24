#include "zy100_rtc_clock.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "app_flags.h"
#include "app_section.h"
#include "os_sync.h"
#include "rtl876x_nvic.h"
#include "rtl876x_rtc.h"
#include "trace.h"
#include "zy100_system_info_store.h"

#ifndef ZY100_RTC_CLOCK_ENABLE
#define ZY100_RTC_CLOCK_ENABLE 1
#endif

#ifndef ZY100_RTC_CLOCK_DLPS_LOG_ENABLE
#define ZY100_RTC_CLOCK_DLPS_LOG_ENABLE 0
#endif

#define ZY100_RTC_DRIFT_PPM_MIN_ELAPSED_MS   10000ULL
#define ZY100_RTC_DRIFT_WARN_MIN_ELAPSED_MS  60000ULL
#define ZY100_RTC_DRIFT_SOFT_WARN_ABS_MS     1000ULL
#define ZY100_RTC_DRIFT_WARN_ABS_MS          3000ULL

typedef struct
{
    bool initialized;
    bool calibrated;
    uint8_t time_source;
    uint8_t ota_restore_requires_consume;
    uint16_t reserved1;
    uint32_t user_id;
    uint64_t base_unix_ms;
    uint64_t base_extended_ticks;
    volatile uint64_t overflow_ticks;
    uint64_t last_sync_unix_ms;
    int32_t last_drift_ms;
} zy100_rtc_clock_state_t;

static zy100_rtc_clock_state_t s_rtc_clock RAM_DATAON_BSS_SECTION;
static uint8_t s_rtc_has_last_sync_debug RAM_DATAON_BSS_SECTION;
static uint64_t s_rtc_last_sync_host_ms RAM_DATAON_BSS_SECTION;
static uint64_t s_rtc_last_sync_rtc_ms_after_apply RAM_DATAON_BSS_SECTION;
static uint64_t s_rtc_last_dlps_prepare_ms RAM_DATAON_BSS_SECTION;
static uint8_t s_rtc_last_dlps_prepare_valid RAM_DATAON_BSS_SECTION;
#if ZY100_RTC_CLOCK_ENABLE
static uint8_t s_rtc_not_initialized_logged RAM_DATAON_BSS_SECTION;
#else
static volatile uint8_t s_rtc_disabled_dataon_anchor RAM_DATAON_BSS_SECTION;
#endif

static bool zy100_rtc_clock_ready_or_log(const char *caller)
{
#if !ZY100_RTC_CLOCK_ENABLE
    s_rtc_disabled_dataon_anchor = 0U;
    (void)caller;
    return false;
#else
    if (s_rtc_clock.initialized)
    {
        return true;
    }

    if (s_rtc_not_initialized_logged == 0U)
    {
        DBG_DIRECT("[RTC_ERR] not_initialized caller=%s",
                   (caller != NULL) ? caller : "unknown");
        s_rtc_not_initialized_logged = 1U;
    }
    return false;
#endif
}

static uint32_t zy100_rtc_clock_u64_to_u32_sat(uint64_t value)
{
    return (value > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)value;
}

static void zy100_rtc_clock_abs_parts(uint64_t abs_ms,
                                      uint32_t *abs_s,
                                      uint32_t *abs_ms_rem)
{
    if (abs_s != NULL)
    {
        *abs_s = zy100_rtc_clock_u64_to_u32_sat(abs_ms / 1000ULL);
    }
    if (abs_ms_rem != NULL)
    {
        *abs_ms_rem = (uint32_t)(abs_ms % 1000ULL);
    }
}

static void zy100_rtc_clock_signed_parts(uint64_t lhs_ms,
                                         uint64_t rhs_ms,
                                         char *sign,
                                         uint64_t *abs_ms,
                                         uint32_t *abs_s,
                                         uint32_t *abs_ms_rem)
{
    uint64_t delta_ms;
    char delta_sign;

    if (lhs_ms >= rhs_ms)
    {
        delta_ms = lhs_ms - rhs_ms;
        delta_sign = '+';
    }
    else
    {
        delta_ms = rhs_ms - lhs_ms;
        delta_sign = '-';
    }

    if (sign != NULL)
    {
        *sign = delta_sign;
    }
    if (abs_ms != NULL)
    {
        *abs_ms = delta_ms;
    }
    zy100_rtc_clock_abs_parts(delta_ms, abs_s, abs_ms_rem);
}

static int32_t zy100_rtc_clock_drift_ppm(char sign,
                                         uint64_t abs_drift_ms,
                                         uint64_t host_elapsed_ms)
{
    uint64_t abs_ppm;

    if (host_elapsed_ms < ZY100_RTC_DRIFT_PPM_MIN_ELAPSED_MS)
    {
        return 0;
    }

    abs_ppm = (abs_drift_ms * 1000000ULL) / host_elapsed_ms;
    if (abs_ppm > 0x7FFFFFFFULL)
    {
        abs_ppm = 0x7FFFFFFFULL;
    }

    return (sign == '-') ? -(int32_t)abs_ppm : (int32_t)abs_ppm;
}

static int32_t zy100_rtc_clock_saturating_diff_ms(uint64_t new_ms,
                                                  uint64_t old_ms)
{
    uint64_t delta;

    if (new_ms >= old_ms)
    {
        delta = new_ms - old_ms;
        return (delta > 0x7FFFFFFFULL) ? 0x7FFFFFFF : (int32_t)delta;
    }

    delta = old_ms - new_ms;
    return (delta > 0x80000000ULL) ? (-2147483647 - 1) : -(int32_t)delta;
}

static void zy100_rtc_clock_read_counter_snapshot(uint32_t *counter,
                                                  uint32_t *pre_counter)
{
    uint32_t counter1;
    uint32_t counter2;
    uint32_t pre1;

    /* Read the prescaler in the same main-counter interval. Callers hold
     * os_lock, so a task/ISR cannot suspend this short hardware snapshot. */
    do
    {
        counter1 = RTC_GetCounter() & ZY100_RTC_COUNTER_MASK;
        pre1 = RTC_GetPreCounter() & ZY100_RTC_PRE_COUNTER_MASK;
        counter2 = RTC_GetCounter() & ZY100_RTC_COUNTER_MASK;
    } while (counter1 != counter2);

    if (counter != NULL)
    {
        *counter = counter1;
    }
    if (pre_counter != NULL)
    {
        if (pre1 >= ZY100_RTC_PRESCALE_DIV)
        {
            pre1 = ZY100_RTC_PRESCALE_DIV - 1U;
        }
        *pre_counter = pre1;
    }
}

static uint64_t zy100_rtc_clock_raw_ticks(uint32_t counter,
                                          uint32_t pre_counter)
{
    return (((uint64_t)(counter & ZY100_RTC_COUNTER_MASK)) *
            ((uint64_t)ZY100_RTC_PRESCALE_DIV)) +
           ((uint64_t)(pre_counter % ZY100_RTC_PRESCALE_DIV));
}

/* Caller holds os_lock. A pending OVF belongs to the next epoch even before
 * RTC_Handler runs. Only the ISR consumes it, so repeated readers never add
 * the same wrap twice. OVF must be serviced within one hardware wrap period. */
static uint64_t zy100_rtc_clock_extended_ticks_locked(uint64_t *raw_ticks)
{
    uint32_t counter;
    uint32_t pre_counter;
    FlagStatus before;
    FlagStatus after;
    uint64_t raw;

    do
    {
        before = RTC_GetFlagStatus(RTC_FLAG_OVF);
        zy100_rtc_clock_read_counter_snapshot(&counter, &pre_counter);
        after = RTC_GetFlagStatus(RTC_FLAG_OVF);
    } while (before != after);

    raw = zy100_rtc_clock_raw_ticks(counter, pre_counter);
    if (raw_ticks != NULL)
    {
        *raw_ticks = raw;
    }
    return s_rtc_clock.overflow_ticks + raw +
           ((after == SET) ? ZY100_RTC_COUNTER_WRAP_TICKS : 0ULL);
}

static uint64_t zy100_rtc_clock_ticks_to_ms(uint64_t ticks)
{
    /* Preserve fractional ticks until conversion; avoid overflowing ticks*1000
     * now that elapsed time is no longer limited to one hardware period. */
#if (ZY100_RTC_INPUT_HZ % 1000ULL) == 0ULL
    return ticks / (ZY100_RTC_INPUT_HZ / 1000ULL);
#else
    return (ticks / ZY100_RTC_INPUT_HZ) * 1000ULL +
           ((ticks % ZY100_RTC_INPUT_HZ) * 1000ULL) / ZY100_RTC_INPUT_HZ;
#endif
}

static uint64_t zy100_rtc_clock_read_time_ms(uint64_t *raw_ticks,
                                             bool *calibrated)
{
    uint64_t elapsed_ticks;
    uint64_t base_unix_ms;
    uint32_t key = os_lock();

    elapsed_ticks = zy100_rtc_clock_extended_ticks_locked(raw_ticks) -
                    s_rtc_clock.base_extended_ticks;
    base_unix_ms = s_rtc_clock.base_unix_ms;
    if (calibrated != NULL)
    {
        *calibrated = s_rtc_clock.calibrated;
    }
    os_unlock(key);
    return base_unix_ms + zy100_rtc_clock_ticks_to_ms(elapsed_ticks);
}

#if ZY100_RTC_CLOCK_ENABLE
void RTC_Handler(void)
{
    uint32_t key = os_lock();
    if (RTC_GetFlagStatus(RTC_FLAG_OVF) == SET)
    {
        s_rtc_clock.overflow_ticks += ZY100_RTC_COUNTER_WRAP_TICKS;
        RTC_ClearOverFlowINT();
    }
    os_unlock(key);
}
#endif

void zy100_rtc_clock_init(void)
{
#if ZY100_RTC_CLOCK_ENABLE
    zy100_system_info_t system_info;
    uint32_t counter;
    uint32_t pre_counter;
    uint32_t key;
    NVIC_InitTypeDef nvic_init;
    uint64_t base_unix_ms = ZY100_RTC_DEFAULT_UNIX_MS;
    uint32_t restore_user_id = 0U;
    bool ota_restore = false;

    if (s_rtc_clock.initialized)
    {
        return;
    }

    if (zy100_system_info_load(&system_info) &&
        (system_info.ota_success_led_pending != 0U) &&
        (system_info.ota_time.state ==
         (uint8_t)ZY100_OTA_TIME_STATE_RESUME_PENDING) &&
        (system_info.ota_time.resume_unix_ms != 0ULL) &&
        (system_info.ota_time.checkpoint_user_id != 0U))
    {
        base_unix_ms = system_info.ota_time.resume_unix_ms;
        restore_user_id = system_info.ota_time.checkpoint_user_id;
        ota_restore = true;
    }

    NVIC_DisableIRQ(RTC_IRQn);
    RTC_DeInit();
    RTC_SetPrescaler(ZY100_RTC_PRESCALER_VALUE);
    RTC_ResetCounter();
    RTC_ResetPrescalerCounter();
    RTC_Cmd(ENABLE);

    key = os_lock();
    zy100_rtc_clock_read_counter_snapshot(&counter, &pre_counter);
    s_rtc_clock.base_unix_ms = base_unix_ms;
    s_rtc_clock.base_extended_ticks = zy100_rtc_clock_raw_ticks(counter, pre_counter);
    s_rtc_clock.overflow_ticks = 0ULL;
    s_rtc_clock.last_sync_unix_ms = 0ULL;
    s_rtc_clock.last_drift_ms = 0;
    s_rtc_clock.user_id = restore_user_id;
    s_rtc_clock.time_source = ota_restore ?
        ZY100_RTC_TIME_SOURCE_OTA_RESTORED_ESTIMATED :
        ZY100_RTC_TIME_SOURCE_DEFAULT_UNCALIBRATED;
    s_rtc_clock.ota_restore_requires_consume = ota_restore ? 1U : 0U;
    s_rtc_clock.calibrated = false;
    s_rtc_clock.initialized = true;
    s_rtc_not_initialized_logged = 0U;
    os_unlock(key);

    /* OVF has no RTC_INTConfig enable bit in this SDK. Unmask its source and
     * enable both MCU delivery and DLPS wake, without enabling tick/compare. */
    RTC_ClearOverFlowINT();
    NVIC_ClearPendingIRQ(RTC_IRQn);
    nvic_init.NVIC_IRQChannel = RTC_IRQn;
    nvic_init.NVIC_IRQChannelPriority = (1U << __NVIC_PRIO_BITS) - 1U;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
    RTC_NvCmd(ENABLE);
    RTC_MaskINTConfig(RTC_INT_OVF, DISABLE);
    RTC_SystemWakeupConfig(ENABLE);

    if (ota_restore)
    {
        DBG_DIRECT("[OTA_TIME] restore source=OTA_RESTORED_ESTIMATED base_ms=%llu user=%lu calibrated=0 consume_pending=1",
                   (unsigned long long)s_rtc_clock.base_unix_ms,
                   (unsigned long)s_rtc_clock.user_id);
    }
    else
    {
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[RTC] init source=default_uncalibrated base_ms=%llu calibrated=0",
                   (unsigned long long)s_rtc_clock.base_unix_ms);
    }
#else
    s_rtc_disabled_dataon_anchor = 0U;
    return;
#endif
}

void zy100_rtc_clock_reset_user(void)
{
    uint32_t key = os_lock();
    s_rtc_clock.calibrated = false;
    s_rtc_clock.time_source = ZY100_RTC_TIME_SOURCE_DEFAULT_UNCALIBRATED;
    s_rtc_clock.ota_restore_requires_consume = 0U;
    s_rtc_clock.last_drift_ms = 0;
    s_rtc_clock.base_unix_ms = ZY100_RTC_DEFAULT_UNIX_MS;
    s_rtc_clock.user_id = 0U;
    s_rtc_clock.last_sync_unix_ms = 0ULL;
    s_rtc_has_last_sync_debug = 0U;
    s_rtc_last_sync_host_ms = 0ULL;
    s_rtc_last_sync_rtc_ms_after_apply = 0ULL;
    os_unlock(key);
}

bool zy100_rtc_clock_is_valid(void)
{
    return zy100_rtc_clock_is_calibrated();
}

bool zy100_rtc_clock_is_calibrated(void)
{
    if (!zy100_rtc_clock_ready_or_log("is_calibrated"))
    {
        return false;
    }
    return s_rtc_clock.calibrated;
}

uint8_t zy100_rtc_clock_time_source(void)
{
    if (!zy100_rtc_clock_ready_or_log("time_source"))
    {
        return ZY100_RTC_TIME_SOURCE_DEFAULT_UNCALIBRATED;
    }
    return s_rtc_clock.time_source;
}

uint64_t zy100_rtc_clock_now_ms(void)
{
    if (!zy100_rtc_clock_ready_or_log("now_ms"))
    {
        return ZY100_RTC_DEFAULT_UNIX_MS;
    }
    return zy100_rtc_clock_read_time_ms(NULL, NULL);
}

bool zy100_rtc_clock_raw_snapshot(zy100_rtc_raw_snapshot_t *out)
{
    uint64_t unix_time_ms;
    bool calibrated;

    if (out == NULL)
    {
        return false;
    }
    memset(out, 0, sizeof(*out));
    if (!zy100_rtc_clock_ready_or_log("raw_snapshot"))
    {
        return false;
    }

    unix_time_ms = zy100_rtc_clock_read_time_ms(&out->ticks, &calibrated);
    out->wrap_ticks = ZY100_RTC_COUNTER_WRAP_TICKS;
    out->nominal_tick_hz = (uint32_t)ZY100_RTC_INPUT_HZ;
    if (calibrated)
    {
        out->unix_time_ms = unix_time_ms;
        out->unix_time_valid = 1U;
    }
    return true;
}

bool zy100_rtc_clock_ota_restore_requires_consume(void)
{
    if (!zy100_rtc_clock_ready_or_log("ota_restore_requires_consume"))
    {
        return false;
    }
    return s_rtc_clock.ota_restore_requires_consume != 0U;
}

void zy100_rtc_clock_ota_restore_note_consumed(void)
{
    if (!zy100_rtc_clock_ready_or_log("ota_restore_note_consumed"))
    {
        return;
    }
    s_rtc_clock.ota_restore_requires_consume = 0U;
    DBG_DIRECT("[OTA_TIME] consume complete capture_gate=open");
}

void zy100_rtc_clock_debug_log_pre_sync(uint32_t user_id,
                                        uint32_t training_id,
                                        uint64_t host_ms,
                                        zy100_rtc_debug_sync_snapshot_t *snapshot)
{
    zy100_rtc_debug_sync_snapshot_t local;

#if !ZY100_LOG_VERBOSE_DEFAULT
    (void)user_id;
    (void)training_id;
#endif

    if (!zy100_rtc_clock_ready_or_log("pre_sync"))
    {
        memset(&local, 0, sizeof(local));
        if (snapshot != NULL)
        {
            *snapshot = local;
        }
        return;
    }
    local.host_ms = host_ms;
    local.rtc_before_ms = zy100_rtc_clock_now_ms();
    local.old_calibrated = s_rtc_clock.calibrated ? 1U : 0U;
    local.source = s_rtc_clock.time_source;
    local.host_elapsed_ms = 0ULL;
    local.rtc_elapsed_ms = 0ULL;
    local.drift_abs_ms = 0ULL;
    local.drift_abs_s = 0U;
    local.drift_abs_ms_rem = 0U;
    local.ppm = 0;
    local.has_prev = 0U;
    local.elapsed_valid = 0U;
    local.drift_sign = '+';

    zy100_rtc_clock_signed_parts(host_ms,
                                 local.rtc_before_ms,
                                 &local.diff_sign,
                                 &local.abs_diff_ms,
                                 &local.diff_abs_s,
                                 &local.diff_abs_ms_rem);
#if ZY100_LOG_VERBOSE_DEFAULT
    DBG_DIRECT("[RTC_CHECK] pre_sync user=%lu train=%lu old_calibrated=%u source=%u host_ms=%llu rtc_ms=%llu diff=%c%lu.%03lus abs_ms=%llu",
               (unsigned long)user_id,
               (unsigned long)training_id,
               (uint32_t)local.old_calibrated,
               (uint32_t)local.source,
               (unsigned long long)local.host_ms,
               (unsigned long long)local.rtc_before_ms,
               local.diff_sign,
               (unsigned long)local.diff_abs_s,
               (unsigned long)local.diff_abs_ms_rem,
               (unsigned long long)local.abs_diff_ms);
#endif

    if ((s_rtc_has_last_sync_debug != 0U) &&
        (host_ms >= s_rtc_last_sync_host_ms) &&
        (local.rtc_before_ms >= s_rtc_last_sync_rtc_ms_after_apply))
    {
        local.has_prev = 1U;
        local.elapsed_valid = 1U;
        local.host_elapsed_ms = host_ms - s_rtc_last_sync_host_ms;
        local.rtc_elapsed_ms =
            local.rtc_before_ms - s_rtc_last_sync_rtc_ms_after_apply;
        zy100_rtc_clock_signed_parts(local.host_elapsed_ms,
                                     local.rtc_elapsed_ms,
                                     &local.drift_sign,
                                     &local.drift_abs_ms,
                                     &local.drift_abs_s,
                                     &local.drift_abs_ms_rem);
        local.ppm = zy100_rtc_clock_drift_ppm(local.drift_sign,
                                               local.drift_abs_ms,
                                               local.host_elapsed_ms);
    }

#if ZY100_LOG_VERBOSE_DEFAULT
    DBG_DIRECT("[RTC_DRIFT] since_last_sync has_prev=%u host_elapsed_ms=%llu rtc_elapsed_ms=%llu drift=%c%lu.%03lus ppm=%ld",
               (uint32_t)local.has_prev,
               (unsigned long long)local.host_elapsed_ms,
               (unsigned long long)local.rtc_elapsed_ms,
               local.drift_sign,
               (unsigned long)local.drift_abs_s,
               (unsigned long)local.drift_abs_ms_rem,
               (long)local.ppm);

    if (local.has_prev == 0U)
    {
        DBG_DIRECT("[RTC_DRIFT_JUDGE] judge=no_baseline action=sync_anyway");
    }
    else if (local.host_elapsed_ms < ZY100_RTC_DRIFT_PPM_MIN_ELAPSED_MS)
    {
        DBG_DIRECT("[RTC_DRIFT_JUDGE] judge=short_elapsed elapsed_ms=%llu drift_abs_ms=%llu ppm=0 action=sync_anyway",
                   (unsigned long long)local.host_elapsed_ms,
                   (unsigned long long)local.drift_abs_ms);
    }
    else if (local.host_elapsed_ms < ZY100_RTC_DRIFT_WARN_MIN_ELAPSED_MS)
    {
        DBG_DIRECT("[RTC_DRIFT_JUDGE] judge=observe elapsed_ms=%llu drift_abs_ms=%llu ppm=%ld action=sync_anyway",
                   (unsigned long long)local.host_elapsed_ms,
                   (unsigned long long)local.drift_abs_ms,
                   (long)local.ppm);
    }
    else if (local.drift_abs_ms <= ZY100_RTC_DRIFT_SOFT_WARN_ABS_MS)
    {
        DBG_DIRECT("[RTC_DRIFT_OK] elapsed_ms=%llu drift_abs_ms=%llu ppm=%ld",
                   (unsigned long long)local.host_elapsed_ms,
                   (unsigned long long)local.drift_abs_ms,
                   (long)local.ppm);
    }
    else if (local.drift_abs_ms <= ZY100_RTC_DRIFT_WARN_ABS_MS)
    {
        DBG_DIRECT("[RTC_DRIFT_WARN_SOFT] elapsed_ms=%llu drift_abs_ms=%llu ppm=%ld action=sync_anyway",
                   (unsigned long long)local.host_elapsed_ms,
                   (unsigned long long)local.drift_abs_ms,
                   (long)local.ppm);
    }
    else
    {
        DBG_DIRECT("[RTC_WARN] drift_large elapsed_ms=%llu drift_abs_ms=%llu ppm=%ld action=sync_anyway",
                   (unsigned long long)local.host_elapsed_ms,
                   (unsigned long long)local.drift_abs_ms,
                   (long)local.ppm);
    }
#else
    if ((local.has_prev != 0U) &&
        (local.host_elapsed_ms >= ZY100_RTC_DRIFT_WARN_MIN_ELAPSED_MS) &&
        (local.drift_abs_ms > ZY100_RTC_DRIFT_WARN_ABS_MS))
    {
        DBG_DIRECT("[RTC_WARN] drift_large elapsed_ms=%llu drift_abs_ms=%llu ppm=%ld action=sync_anyway",
                   (unsigned long long)local.host_elapsed_ms,
                   (unsigned long long)local.drift_abs_ms,
                   (long)local.ppm);
    }
#endif

    if (snapshot != NULL)
    {
        *snapshot = local;
    }
}

void zy100_rtc_clock_debug_log_sync_done(uint32_t user_id,
                                         uint32_t training_id,
                                         const zy100_rtc_debug_sync_snapshot_t *snapshot)
{
    uint64_t host_ms;
    uint64_t rtc_after_ms;
#if ZY100_LOG_VERBOSE_DEFAULT
    char applied_sign = '+';
    uint32_t applied_s = 0U;
    uint32_t applied_ms_rem = 0U;
#else
    (void)user_id;
    (void)training_id;
#endif

    if (!zy100_rtc_clock_ready_or_log("sync_done"))
    {
        return;
    }
    rtc_after_ms = zy100_rtc_clock_now_ms();
    host_ms = (snapshot != NULL) ? snapshot->host_ms : s_rtc_clock.last_sync_unix_ms;
#if ZY100_LOG_VERBOSE_DEFAULT
    if (snapshot != NULL)
    {
        applied_sign = snapshot->diff_sign;
        applied_s = snapshot->diff_abs_s;
        applied_ms_rem = snapshot->diff_abs_ms_rem;
    }
#endif

#if ZY100_LOG_VERBOSE_DEFAULT
    DBG_DIRECT("[RTC_SYNC_DONE] user=%lu train=%lu calibrated=%u source=%u now_ms=%llu applied_diff=%c%lu.%03lus",
               (unsigned long)user_id,
               (unsigned long)training_id,
               s_rtc_clock.calibrated ? 1U : 0U,
               (uint32_t)s_rtc_clock.time_source,
               (unsigned long long)rtc_after_ms,
               applied_sign,
               (unsigned long)applied_s,
               (unsigned long)applied_ms_rem);
#endif

    s_rtc_last_sync_host_ms = host_ms;
    s_rtc_last_sync_rtc_ms_after_apply = rtc_after_ms;
    s_rtc_has_last_sync_debug = 1U;
#if ZY100_LOG_VERBOSE_DEFAULT
    DBG_DIRECT("[RTC_SYNC_BASE] host_ms=%llu rtc_after_ms=%llu baseline_saved=1",
               (unsigned long long)s_rtc_last_sync_host_ms,
               (unsigned long long)s_rtc_last_sync_rtc_ms_after_apply);
#endif
}

void zy100_rtc_clock_sync(uint32_t user_id, uint64_t unix_time_ms)
{
    uint32_t key;
    uint64_t old_now_ms;
    int32_t drift_ms;
    bool old_calibrated;

    if (!zy100_rtc_clock_ready_or_log("sync"))
    {
        return;
    }
    old_now_ms = zy100_rtc_clock_now_ms();
    drift_ms = zy100_rtc_clock_saturating_diff_ms(unix_time_ms, old_now_ms);
    key = os_lock();
    old_calibrated = s_rtc_clock.calibrated;

    s_rtc_clock.base_extended_ticks = zy100_rtc_clock_extended_ticks_locked(NULL);
    s_rtc_clock.base_unix_ms = unix_time_ms;
    s_rtc_clock.last_sync_unix_ms = unix_time_ms;
    s_rtc_clock.last_drift_ms = drift_ms;
    s_rtc_clock.user_id = user_id;
    s_rtc_clock.time_source = ZY100_RTC_TIME_SOURCE_BLE_TIME_SYNCED;
    s_rtc_clock.calibrated = true;
    os_unlock(key);

    ZY100_LOG_ROUTINE(DBG_DIRECT, "[RTC] sync user=%lu unix_ms=%llu old_calibrated=%u drift_ms=%ld calibrated=1",
               (unsigned long)user_id,
               (unsigned long long)unix_time_ms,
               old_calibrated ? 1U : 0U,
               (long)drift_ms);
}

uint32_t zy100_rtc_clock_user_id(void)
{
    if (!zy100_rtc_clock_ready_or_log("user_id"))
    {
        return 0U;
    }
    return s_rtc_clock.user_id;
}

uint64_t zy100_rtc_clock_last_sync_ms(void)
{
    uint64_t last_sync_ms;
    uint32_t key;
    if (!zy100_rtc_clock_ready_or_log("last_sync_ms"))
    {
        return 0ULL;
    }
    key = os_lock();
    last_sync_ms = s_rtc_clock.last_sync_unix_ms;
    os_unlock(key);
    return last_sync_ms;
}

void zy100_rtc_clock_prepare_ota_handoff(void)
{
#if ZY100_RTC_CLOCK_ENABLE
    uint32_t key = os_lock();
    /* Called only at the irreversible DFU switch. AON RTC keeps counting for
     * the existing checkpoint; DFU must not inherit this application's IRQ. */
    RTC_MaskINTConfig(RTC_INT_OVF, ENABLE);
    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_ClearPendingIRQ(RTC_IRQn);
    os_unlock(key);
#endif
}

int32_t zy100_rtc_clock_last_drift_ms(void)
{
    if (!zy100_rtc_clock_ready_or_log("last_drift_ms"))
    {
        return 0;
    }
    return s_rtc_clock.last_drift_ms;
}

void zy100_rtc_clock_prepare_dlps(void)
{
    uint64_t now_ms;

    if (!zy100_rtc_clock_ready_or_log("prepare_dlps"))
    {
        return;
    }
    now_ms = zy100_rtc_clock_now_ms();
    s_rtc_last_dlps_prepare_ms = now_ms;
    s_rtc_last_dlps_prepare_valid = 1U;
#if ZY100_RTC_CLOCK_DLPS_LOG_ENABLE
    DBG_DIRECT("[RTC_SLEEP] keep_running=1 calibrated=%u source=%u now_ms=%llu",
               s_rtc_clock.calibrated ? 1U : 0U,
               (uint32_t)s_rtc_clock.time_source,
               (unsigned long long)now_ms);
#endif
}

void zy100_rtc_clock_on_wakeup(void)
{
    uint64_t wake_ms;
    uint64_t delta_ms = 0ULL;
#if ZY100_RTC_CLOCK_DLPS_LOG_ENABLE
    uint32_t delta_s;
    uint32_t delta_ms_rem;
#endif

    if (!zy100_rtc_clock_ready_or_log("wakeup"))
    {
        return;
    }
    wake_ms = zy100_rtc_clock_now_ms();
    if ((s_rtc_last_dlps_prepare_valid != 0U) &&
        (wake_ms >= s_rtc_last_dlps_prepare_ms))
    {
        delta_ms = wake_ms - s_rtc_last_dlps_prepare_ms;
    }
#if ZY100_RTC_CLOCK_DLPS_LOG_ENABLE
    zy100_rtc_clock_abs_parts(delta_ms, &delta_s, &delta_ms_rem);
    (void)delta_s;
    (void)delta_ms_rem;
    DBG_DIRECT("[RTC_WAKE] calibrated=%u source=%u now_ms=%llu sleep_delta_ms=%llu",
               s_rtc_clock.calibrated ? 1U : 0U,
               (uint32_t)s_rtc_clock.time_source,
               (unsigned long long)wake_ms,
               (unsigned long long)delta_ms);
    if (delta_ms == 0ULL)
    {
        DBG_DIRECT("[RTC_ERR] stopped_in_dlps delta_ms=0");
    }
#else
    (void)delta_ms;
#endif
}
