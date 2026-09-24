#include "svc_yhm2712_charge.h"

#include <stddef.h>
#include "trace.h"
#include "../app_flags.h"

#if YHM_PRODUCTION_COMPACT
#define YHM_DETAIL_LOG(...) do { if (0) { DBG_DIRECT(__VA_ARGS__); } } while (0)
#else
#define YHM_DETAIL_LOG(...) DBG_DIRECT(__VA_ARGS__)
#endif

#if YHM_PRODUCTION_COMPACT
static yhm2712_acmd_status_t svc_yhm2712_charge_init_status_impl(uint8_t stacmd_pin,
                             bool external_power_present);
static yhm2712_acmd_status_t svc_yhm2712_charge_configure_profile_impl(const char *reason);
static yhm2712_acmd_status_t svc_yhm2712_charge_wake_for_charge_only_impl(void);
static yhm2712_acmd_status_t svc_yhm2712_charge_wake_for_active_impl(bool chg_present,
                                                         const char *reason);
static yhm2712_acmd_status_t svc_yhm2712_charge_prepare_full_sleep_impl(void);
static yhm2712_acmd_status_t svc_yhm2712_charge_factory_enter_shipping_impl(void);
static yhm2712_acmd_status_t svc_yhm2712_charge_poll_impl(uint32_t now_ms,
                                              uint16_t vbat_mv,
                                              bool chg_present);
static svc_yhm2712_transition_outcome_t svc_yhm2712_charge_apply_transition_impl(
    const svc_yhm2712_transition_request_t *request,
    svc_yhm2712_transition_result_t *result);
#define svc_yhm2712_charge_init_status svc_yhm2712_charge_init_status_impl
#define svc_yhm2712_charge_configure_profile svc_yhm2712_charge_configure_profile_impl
#define svc_yhm2712_charge_wake_for_charge_only svc_yhm2712_charge_wake_for_charge_only_impl
#define svc_yhm2712_charge_wake_for_active svc_yhm2712_charge_wake_for_active_impl
#define svc_yhm2712_charge_prepare_full_sleep svc_yhm2712_charge_prepare_full_sleep_impl
#define svc_yhm2712_charge_factory_enter_shipping svc_yhm2712_charge_factory_enter_shipping_impl
#define svc_yhm2712_charge_poll svc_yhm2712_charge_poll_impl
#define svc_yhm2712_charge_apply_transition svc_yhm2712_charge_apply_transition_impl
#endif

#define SVC_YHM2712_FULL_CHECK_START_MV 4300U
#define SVC_YHM2712_FULL_ACCEPT_MIN_MV  4350U
#define SVC_YHM2712_FULL_POLL_INTERVAL_MS 30000UL
#define SVC_YHM2712_FULL_CONFIRM_COUNT  2U

/* Test-branch-only diagnostic: does not alter ACMD timing or GPIO ownership. */
#ifndef SVC_YHM2712_ACMD_ACK_DIAG_ENABLE
#define SVC_YHM2712_ACMD_ACK_DIAG_ENABLE  1U
#endif

#if ZY100_CHG_WAKE_DIAG_LOG_ENABLE
#define SVC_YHM_LOG(...)                YHM_DETAIL_LOG(__VA_ARGS__)
#else
#define SVC_YHM_LOG(...)                do { if (0) { YHM_DETAIL_LOG(__VA_ARGS__); } } while (0)
#endif

#if (SVC_YHM2712_FULL_ACCEPT_MIN_MV < SVC_YHM2712_FULL_CHECK_START_MV)
#error "SVC_YHM2712_FULL_ACCEPT_MIN_MV must be >= SVC_YHM2712_FULL_CHECK_START_MV"
#endif

static bool s_svc_yhm2712_inited = false;
static uint8_t s_svc_yhm2712_stacmd_pin = 0U;
static svc_yhm2712_charge_status_t s_svc_yhm2712_status = {0};
static uint32_t s_svc_yhm2712_last_full_poll_ms = 0U;
static uint8_t s_svc_yhm2712_full_confirm_count = 0U;
static bool s_svc_yhm2712_full_window_open = false;
static bool s_svc_yhm2712_full_poll_valid = false;
static bool s_svc_yhm2712_full_detected = false;
#if !YHM_PRODUCTION_COMPACT
static bool s_svc_yhm2712_ack_diag_reported = false;
#endif
static yhm2712_acmd_boot_diag_t s_svc_yhm2712_boot_diag;
static bool s_svc_yhm2712_boot_diag_valid = false;

#if ZY100_BUILD_FACTORY
static void svc_yhm2712_log_id_check(const char *reason)
{
    yhm2712_acmd_id_check_snapshot_t check;
    yhm_pressure_rx_snapshot_t first_bad;
    uint8_t index;

    if (!yhm2712_acmd_id_check_snapshot_get(&check))
    {
        return;
    }
    YHM_DETAIL_LOG("[FACTORY_YHM_ID] reason=%s attempts=%u max=%u consecutive=%u required=%u bad_reads=%u extra_reads=%u",
               reason, check.attempts, YHM2712_ACMD_ID_MAX_ATTEMPTS,
               check.stable_count, YHM2712_ACMD_ID_STABLE_READ_COUNT,
               check.failure_count,
               check.attempts > YHM2712_ACMD_ID_STABLE_READ_COUNT ?
               check.attempts - YHM2712_ACMD_ID_STABLE_READ_COUNT : 0U);
    if (check.failure_count == 0U)
    {
        return;
    }
    /* Only after the complete boot diagnostic has left all ACMD transactions. */
    for (index = 0U; index < check.attempts; ++index)
    {
        YHM_DETAIL_LOG("[FACTORY_YHM_ID_READ] n=%u io=%s value_valid=%u id=%02x",
                   index + 1U, yhm2712_acmd_status_name(check.status[index]),
                   check.status[index] == YHM2712_ACMD_STATUS_OK ? 1U : 0U,
                   check.values[index]);
    }
    if (yhm_pressure_rx_snapshot_get(true, &first_bad))
    {
        YHM_DETAIL_LOG("[FACTORY_YHM_ID_BAD] seq=%lu reg=%02x id=%02x status=%s capture=%s bits=%u entry=%u",
                   (unsigned long)first_bad.sequence, first_bad.reg, first_bad.value,
                   yhm2712_acmd_status_name(first_bad.status),
                   yhm2712_acmd_status_name(first_bad.capture_status),
                   first_bad.bit_count, first_bad.entry_low_bit);
        for (index = 0U; index < first_bad.bit_count && index < 8U; ++index)
        {
            YHM_DETAIL_LOG("[FACTORY_YHM_ID_EDGE] bit=%u fall=%lu rise=%lu",
                       index, (unsigned long)first_bad.fall[index],
                       (unsigned long)first_bad.rise[index]);
        }
    }
}
#endif

static void svc_yhm2712_log_ack_diag_once(const char *reason)
{
#if SVC_YHM2712_ACMD_ACK_DIAG_ENABLE && !YHM_PRODUCTION_COMPACT
    yhm2712_acmd_failure_snapshot_t failure;

    if (s_svc_yhm2712_ack_diag_reported ||
        !yhm2712_acmd_failure_snapshot_get(&failure))
    {
        return;
    }

    s_svc_yhm2712_ack_diag_reported = true;
    YHM_DETAIL_LOG("[YHM][ACK_DIAG] rs=%s r=%02x d=%c st=%s exp=%u got=%u path=%s entry=%u",
               (reason != NULL) ? reason : "online",
               (uint32_t)failure.reg,
               failure.is_write ? 'W' : 'R',
               yhm2712_acmd_stage_name(failure.stage),
               (uint32_t)failure.expected_symbol,
               (uint32_t)failure.symbol,
               yhm2712_acmd_ack_path_name(failure.ack_path),
               (uint32_t)failure.ack_entry_level);
    YHM_DETAIL_LOG("[YHM][ACK_DIAG_T] turn=%lu high=%lu fall=%lu low=%lu mf=%u bits=%u tail=%u",
               (unsigned long)failure.turnaround_tail_cycles,
               (unsigned long)failure.high_wait,
               (unsigned long)failure.fall_wait,
               (unsigned long)failure.low_cycles,
               (uint32_t)failure.missed_fall,
               (uint32_t)failure.data_bit_count,
               (uint32_t)failure.tail_drive_high);
#else
    (void)reason;
#endif
}

static const char *svc_yhm2712_chg_status_name(uint8_t status)
{
    switch (status)
    {
    case YHM2712_CHG_STATUS_DISCHARGE:
        return "discharge";
    case YHM2712_CHG_STATUS_PRE_CHARGE:
        return "pre";
    case YHM2712_CHG_STATUS_TRICKLE_CHARGE:
        return "trickle";
    case YHM2712_CHG_STATUS_CC_CHARGE:
        return "cc";
    case YHM2712_CHG_STATUS_CV_CHARGE:
        return "cv";
    case YHM2712_CHG_STATUS_CHARGE_DONE:
        return "done";
    default:
        return "reserved";
    }
}

static bool svc_yhm2712_fault_or_backoff(uint8_t chg_status, uint8_t fsm)
{
    return (fsm == YHM2712_FSM_MODE_FAULT) ||
           (chg_status == YHM2712_CHG_STATUS_CC_CHARGE) ||
           (chg_status == YHM2712_CHG_STATUS_TRICKLE_CHARGE) ||
           (chg_status == YHM2712_CHG_STATUS_PRE_CHARGE);
}

static bool svc_yhm2712_charge_done_status(uint8_t chg_status, uint8_t fsm)
{
    return (chg_status == YHM2712_CHG_STATUS_CHARGE_DONE) ||
           (fsm == YHM2712_FSM_MODE_CHARGE_DONE);
}

static void svc_yhm2712_reset_full_state(void)
{
    s_svc_yhm2712_last_full_poll_ms = 0U;
    s_svc_yhm2712_full_confirm_count = 0U;
    s_svc_yhm2712_full_window_open = false;
    s_svc_yhm2712_full_poll_valid = false;
    s_svc_yhm2712_full_detected = false;
}

static yhm2712_acmd_status_t svc_yhm2712_online_check(const char *reason);
static yhm2712_acmd_status_t svc_yhm2712_read_status(const char *reason);
static yhm2712_acmd_status_t svc_yhm2712_read_transition_fsm(void);

static bool svc_yhm2712_latest_done_status(void)
{
    return s_svc_yhm2712_status.valid &&
           svc_yhm2712_charge_done_status(s_svc_yhm2712_status.chg_status,
                                          s_svc_yhm2712_status.fsm);
}

static bool svc_yhm2712_full_vbat_accepted(uint16_t vbat_mv)
{
    return vbat_mv >= SVC_YHM2712_FULL_ACCEPT_MIN_MV;
}

static void svc_yhm2712_note_full_done_status(uint16_t vbat_mv,
                                              const char *reason)
{
    s_svc_yhm2712_full_window_open = true;

    if (!svc_yhm2712_full_vbat_accepted(vbat_mv))
    {
        s_svc_yhm2712_full_confirm_count = 0U;
        SVC_YHM_LOG("[YHM2712][SVC] full_defer reason=%s vbat=%u min=%u st1=0x%02x st2=0x%02x st3=0x%02x",
                    (reason != NULL) ? reason : "full",
                    (uint32_t)vbat_mv,
                    (uint32_t)SVC_YHM2712_FULL_ACCEPT_MIN_MV,
                    (uint32_t)s_svc_yhm2712_status.status1,
                    (uint32_t)s_svc_yhm2712_status.status2,
                    (uint32_t)s_svc_yhm2712_status.status3);
        return;
    }

    if (s_svc_yhm2712_full_confirm_count < SVC_YHM2712_FULL_CONFIRM_COUNT)
    {
        s_svc_yhm2712_full_confirm_count++;
    }

    if (s_svc_yhm2712_full_confirm_count >= SVC_YHM2712_FULL_CONFIRM_COUNT)
    {
        if (!s_svc_yhm2712_full_detected)
        {
            SVC_YHM_LOG("[YHM2712][SVC] full_accept reason=%s vbat=%u min=%u confirm=%u",
                        (reason != NULL) ? reason : "full",
                        (uint32_t)vbat_mv,
                        (uint32_t)SVC_YHM2712_FULL_ACCEPT_MIN_MV,
                        (uint32_t)s_svc_yhm2712_full_confirm_count);
            s_svc_yhm2712_full_detected = true;
        }
    }
}

static yhm2712_acmd_status_t svc_yhm2712_skip_charge_mode_if_done(const char *reason,
                                                                  bool *skip_charge_mode)
{
    yhm2712_acmd_status_t status;
    const char *use_reason = (reason != NULL) ? reason : "charge_full";

    if (skip_charge_mode != NULL)
    {
        *skip_charge_mode = false;
    }

    if (!svc_yhm2712_latest_done_status())
    {
        s_svc_yhm2712_full_confirm_count = 0U;
        return YHM2712_ACMD_STATUS_OK;
    }

    if (!s_svc_yhm2712_full_detected)
    {
        status = svc_yhm2712_read_status(use_reason);
        if (status != YHM2712_ACMD_STATUS_OK)
        {
            s_svc_yhm2712_full_confirm_count = 0U;
            return status;
        }

        if (!svc_yhm2712_latest_done_status())
        {
            s_svc_yhm2712_full_confirm_count = 0U;
            return YHM2712_ACMD_STATUS_OK;
        }

        s_svc_yhm2712_full_confirm_count = 0U;
    }

    if (s_svc_yhm2712_full_detected)
    {
        if (skip_charge_mode != NULL)
        {
            *skip_charge_mode = true;
        }

        SVC_YHM_LOG("[YHM2712][SVC] mode_skip reason=%s mode=0x%02x full=1",
                    use_reason,
                    (uint32_t)YHM2712_MODE_SET_CHARGE);
    }
    else
    {
        SVC_YHM_LOG("[YHM2712][SVC] mode_continue reason=%s mode=0x%02x raw_done=1 full=0",
                    use_reason,
                    (uint32_t)YHM2712_MODE_SET_CHARGE);
    }

    return YHM2712_ACMD_STATUS_OK;
}

static bool svc_yhm2712_skip_charge_mode_if_current_done(const char *reason)
{
    yhm2712_acmd_status_t status;
    bool skip_charge_mode = false;

    status = svc_yhm2712_online_check(reason);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return false;
    }

    status = svc_yhm2712_read_status(reason);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        s_svc_yhm2712_full_confirm_count = 0U;
        return false;
    }

    status = svc_yhm2712_skip_charge_mode_if_done(reason, &skip_charge_mode);
    return (status == YHM2712_ACMD_STATUS_OK) && skip_charge_mode;
}

static yhm2712_acmd_status_t svc_yhm2712_online_check(const char *reason)
{
    yhm2712_acmd_boot_diag_t diag;
    yhm2712_acmd_status_t status;

    if (!s_svc_yhm2712_inited)
    {
        return YHM2712_ACMD_STATUS_NOT_INIT;
    }

    if (!yhm2712_acmd_init(s_svc_yhm2712_stacmd_pin))
    {
        YHM_DETAIL_LOG("[YHM2712][SVC][ERR] reason=%s init_pin_fail pin=%u",
                   (reason != NULL) ? reason : "online",
                   s_svc_yhm2712_stacmd_pin);
        return YHM2712_ACMD_STATUS_BAD_PIN;
    }

    status = yhm2712_acmd_read_boot_diag(&diag);
#if ZY100_YHM_HAS_MOS
    yhm_mos_error_counters_log(reason);
#endif
#if ZY100_BUILD_FACTORY
    svc_yhm2712_log_id_check(reason);
#endif
    s_svc_yhm2712_boot_diag = diag;
    s_svc_yhm2712_boot_diag_valid = diag.id_valid;
    if ((status != YHM2712_ACMD_STATUS_OK) || !diag.comm_ok)
    {
        YHM_DETAIL_LOG("[YHM2712][SVC][ERR] reason=%s online_fail status=%s id=0x%02x stable=%u/%u st1=0x%02x st2=0x%02x",
                   (reason != NULL) ? reason : "online",
                   yhm2712_acmd_status_name(status),
                   diag.id,
                   diag.id_stable_count,
                   diag.id_attempts,
                   diag.status1,
                   diag.status2);
        svc_yhm2712_log_ack_diag_once(reason);
        return (status != YHM2712_ACMD_STATUS_OK) ? status : diag.result;
    }

#if SVC_YHM2712_ACMD_ACK_DIAG_ENABLE && !YHM_PRODUCTION_COMPACT
    if (!s_svc_yhm2712_ack_diag_reported)
    {
        s_svc_yhm2712_ack_diag_reported = true;
#if !ZY100_YHM_HAS_MOS
        YHM_DETAIL_LOG("[YHM][ACK_DIAG] rs=%s online_ok id=0x%02x tail_then_ack=%lu st1=0x%02x st2=0x%02x",
                   (reason != NULL) ? reason : "online",
                   diag.id,
                   (unsigned long)yhm2712_acmd_ack_turnaround_tail_count_get(),
                   diag.status1,
                   diag.status2);
#endif
    }
#endif

    return YHM2712_ACMD_STATUS_OK;
}

static yhm2712_acmd_status_t svc_yhm2712_read_status(const char *reason)
{
    yhm2712_acmd_status_t status;
    uint8_t st1 = 0U;
    uint8_t st2 = 0U;
    uint8_t st3 = 0U;

    status = yhm2712_acmd_read_reg(YHM2712_REG_STATUS1, &st1);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        s_svc_yhm2712_status.fsm_valid = false;
        s_svc_yhm2712_status.valid = false;
        return status;
    }

    status = yhm2712_acmd_read_reg(YHM2712_REG_STATUS2, &st2);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        s_svc_yhm2712_status.fsm_valid = false;
        s_svc_yhm2712_status.valid = false;
        return status;
    }

    status = yhm2712_acmd_read_reg(YHM2712_REG_STATUS3, &st3);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        s_svc_yhm2712_status.fsm_valid = false;
        s_svc_yhm2712_status.valid = false;
        return status;
    }

    s_svc_yhm2712_status.status1 = st1;
    s_svc_yhm2712_status.status2 = st2;
    s_svc_yhm2712_status.status3 = st3;
    s_svc_yhm2712_status.chg_status =
        (uint8_t)((st1 & YHM2712_STATUS1_CHG_STATUS_MASK) >>
                  YHM2712_STATUS1_CHG_STATUS_SHIFT);
    s_svc_yhm2712_status.fsm =
        (uint8_t)((st2 & YHM2712_STATUS2_FSM_MODE_MASK) >>
                  YHM2712_STATUS2_FSM_MODE_SHIFT);
    s_svc_yhm2712_status.isns_lt_iterm =
        ((st1 & YHM2712_STATUS1_ISNS_LT_ITERM_MASK) != 0U);
    s_svc_yhm2712_status.vsys_gt_vbat =
        ((st1 & YHM2712_STATUS1_VSYS_GT_VBAT_MASK) != 0U);
    s_svc_yhm2712_status.in_cv =
        (s_svc_yhm2712_status.chg_status == YHM2712_CHG_STATUS_CV_CHARGE) &&
        ((st3 & YHM2712_STATUS3_CV_BAR_MASK) == 0U);
    s_svc_yhm2712_status.fsm_valid = true;
    s_svc_yhm2712_status.valid = true;

    SVC_YHM_LOG("[YHM2712][SVC] status reason=%s st1=0x%02x st2=0x%02x st3=0x%02x chg_status=%u(%s) fsm=%u(%s)",
                (reason != NULL) ? reason : "status",
                (uint32_t)st1,
                (uint32_t)st2,
                (uint32_t)st3,
                (uint32_t)s_svc_yhm2712_status.chg_status,
                svc_yhm2712_chg_status_name(s_svc_yhm2712_status.chg_status),
                (uint32_t)s_svc_yhm2712_status.fsm,
                yhm2712_acmd_fsm_mode_name(s_svc_yhm2712_status.fsm));
    SVC_YHM_LOG("[YHM2712][SVC] status_flags reason=%s isns_lt_iterm=%u vsys_gt_vbat=%u cv=%u full=%u",
                (reason != NULL) ? reason : "status",
                (uint32_t)(s_svc_yhm2712_status.isns_lt_iterm ? 1U : 0U),
                (uint32_t)(s_svc_yhm2712_status.vsys_gt_vbat ? 1U : 0U),
                (uint32_t)(s_svc_yhm2712_status.in_cv ? 1U : 0U),
                (uint32_t)(s_svc_yhm2712_full_detected ? 1U : 0U));
    return YHM2712_ACMD_STATUS_OK;
}

static yhm2712_acmd_status_t svc_yhm2712_read_transition_fsm(void)
{
    yhm2712_acmd_status_t status;
    uint8_t status2 = 0U;
    uint8_t fsm = 0U;

    s_svc_yhm2712_status.status1 = 0U;
    s_svc_yhm2712_status.status2 = 0U;
    s_svc_yhm2712_status.status3 = 0U;
    s_svc_yhm2712_status.chg_status = 0U;
    s_svc_yhm2712_status.fsm = 0U;
    s_svc_yhm2712_status.isns_lt_iterm = false;
    s_svc_yhm2712_status.vsys_gt_vbat = false;
    s_svc_yhm2712_status.in_cv = false;
    s_svc_yhm2712_status.fsm_valid = false;
    s_svc_yhm2712_status.valid = false;

    status = yhm2712_acmd_read_fsm_mode(&status2, &fsm);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return status;
    }

    s_svc_yhm2712_status.status2 = status2;
    s_svc_yhm2712_status.fsm = fsm;
    s_svc_yhm2712_status.fsm_valid = true;
    return YHM2712_ACMD_STATUS_OK;
}

static yhm2712_acmd_status_t svc_yhm2712_write_read_reg(uint8_t reg,
                                                        uint8_t value,
                                                        uint8_t *readback)
{
    yhm2712_acmd_status_t status;
    uint8_t attempts = 0U;

    status = yhm2712_acmd_write_reg_retry_readback(reg,
                                                   value,
                                                   readback,
                                                   &attempts);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        YHM_DETAIL_LOG("[YHM2712][SVC][ERR] reg_write reg=0x%02x value=0x%02x attempts=%u status=%s",
                   reg,
                   value,
                   attempts,
                   yhm2712_acmd_status_name(status));
        return status;
    }

    return YHM2712_ACMD_STATUS_OK;
}

static yhm2712_acmd_status_t svc_yhm2712_select_physical_profile(
    bool external_power_present,
    const char *reason)
{
    yhm2712_acmd_status_t status =
        yhm2712_acmd_select_external_power_profile(external_power_present);

    if (status != YHM2712_ACMD_STATUS_OK)
    {
        YHM_DETAIL_LOG("[YHM][PHY_E] ac=%u err=%s rs=%s",
                   external_power_present ? 1U : 0U,
                   yhm2712_acmd_status_name(status),
                   (reason != NULL) ? reason : "none");
    }
    return status;
}

bool svc_yhm2712_charge_prepare_interface(uint8_t pin)
{
    s_svc_yhm2712_stacmd_pin = pin;
    s_svc_yhm2712_boot_diag_valid = false;
    s_svc_yhm2712_inited = yhm2712_acmd_init(pin);
    return s_svc_yhm2712_inited;
}



#if YHM_PRODUCTION_COMPACT
static
#endif
yhm2712_acmd_status_t svc_yhm2712_charge_init_status(uint8_t stacmd_pin,
                             bool external_power_present)
{
    yhm2712_acmd_status_t status;

#if !ZY100_CHG_WAKE_DIAG_LOG_ENABLE
    (void)svc_yhm2712_chg_status_name;
#endif
    if (!svc_yhm2712_charge_prepare_interface(stacmd_pin))
        return YHM2712_ACMD_STATUS_BAD_PIN;

    status = svc_yhm2712_select_physical_profile(external_power_present,
                                                  "init");
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return status;
    }

    return svc_yhm2712_charge_configure_profile("init");
}

#if YHM_PRODUCTION_COMPACT
static
#endif
yhm2712_acmd_status_t svc_yhm2712_charge_configure_profile(const char *reason)
{
    yhm2712_acmd_status_t status;
    uint8_t v_read = 0U;
    uint8_t i_read = 0U;
    uint8_t cfg_read = 0U;

    status = svc_yhm2712_online_check(reason);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return status;
    }

    status = svc_yhm2712_write_read_reg(YHM2712_REG_V_CTRL,
                                        YHM2712_PROFILE_V_CTRL_4350_TRICKLE_3000,
                                        &v_read);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return status;
    }

    status = svc_yhm2712_write_read_reg(YHM2712_REG_I_CTRL,
                                        YHM2712_PROFILE_I_CTRL_1X_ITERM_C20,
                                        &i_read);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return status;
    }

    status = svc_yhm2712_write_read_reg(YHM2712_REG_CONFIG,
                                        YHM2712_PROFILE_CONFIG_DEFAULT_SAFE,
                                        &cfg_read);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return status;
    }

    SVC_YHM_LOG("[YHM2712][SVC] profile reason=%s v=0x%02x i=0x%02x cfg=0x%02x target_v=0x%02x target_i=0x%02x target_cfg=0x%02x",
                (reason != NULL) ? reason : "profile",
                v_read,
                i_read,
                cfg_read,
                YHM2712_PROFILE_V_CTRL_4350_TRICKLE_3000,
                YHM2712_PROFILE_I_CTRL_1X_ITERM_C20,
                YHM2712_PROFILE_CONFIG_DEFAULT_SAFE);

    return svc_yhm2712_read_status(reason);
}

static yhm2712_acmd_status_t svc_yhm2712_write_mode(uint8_t mode,
                                                    const char *reason)
{
    yhm2712_acmd_status_t status;

    switch (mode)
    {
    case YHM2712_MODE_SET_SLEEP:
        status = yhm2712_acmd_enter_sleep_mode();
        break;
    case YHM2712_MODE_SET_DISCHARGE:
        status = yhm2712_acmd_exit_sleep_to_discharge();
        break;
    case YHM2712_MODE_SET_CHARGE:
        status = yhm2712_acmd_enter_charge_mode();
        break;
    default:
        status = YHM2712_ACMD_STATUS_BAD_PARAM;
        break;
    }

    if (status != YHM2712_ACMD_STATUS_OK)
    {
        YHM_DETAIL_LOG("[YHM2712][SVC][ERR] mode_verified reason=%s mode=0x%02x status=%s",
                   (reason != NULL) ? reason : "mode",
                   mode,
                   yhm2712_acmd_status_name(status));
        return status;
    }

    SVC_YHM_LOG("[YHM2712][SVC] mode reason=%s mode=0x%02x",
                (reason != NULL) ? reason : "mode",
                mode);
    return svc_yhm2712_read_status(reason);
}

#if YHM_PRODUCTION_COMPACT
static
#endif
yhm2712_acmd_status_t svc_yhm2712_charge_wake_for_charge_only(void)
{
    yhm2712_acmd_status_t status;
    bool skip_charge_mode = false;

    status = svc_yhm2712_select_physical_profile(true, "charge_only");
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return status;
    }

    if (svc_yhm2712_skip_charge_mode_if_current_done("charge_only_pre_full"))
    {
        return YHM2712_ACMD_STATUS_OK;
    }

    status = svc_yhm2712_charge_configure_profile("charge_only");
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return status;
    }

    status = svc_yhm2712_skip_charge_mode_if_done("charge_only_full",
                                                  &skip_charge_mode);
    if ((status != YHM2712_ACMD_STATUS_OK) || skip_charge_mode)
    {
        return status;
    }

    return svc_yhm2712_write_mode(YHM2712_MODE_SET_CHARGE, "charge_only");
}

#if YHM_PRODUCTION_COMPACT
static
#endif
yhm2712_acmd_status_t svc_yhm2712_charge_wake_for_active(bool chg_present,
                                                         const char *reason)
{
    yhm2712_acmd_status_t status;
    const char *use_reason = (reason != NULL) ? reason : "active";
    bool skip_charge_mode = false;

    status = svc_yhm2712_select_physical_profile(chg_present, use_reason);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return status;
    }

    if (chg_present &&
        svc_yhm2712_skip_charge_mode_if_current_done("active_pre_full"))
    {
        return YHM2712_ACMD_STATUS_OK;
    }

    status = svc_yhm2712_charge_configure_profile(use_reason);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return status;
    }

    if (chg_present)
    {
        status = svc_yhm2712_skip_charge_mode_if_done(use_reason,
                                                      &skip_charge_mode);
        if ((status != YHM2712_ACMD_STATUS_OK) || skip_charge_mode)
        {
            return status;
        }
    }

    return svc_yhm2712_write_mode(chg_present ? YHM2712_MODE_SET_CHARGE :
                                  YHM2712_MODE_SET_DISCHARGE,
                                  use_reason);
}

static const char *svc_yhm2712_transition_event_name(
    svc_yhm2712_transition_event_t event)
{
    switch (event)
    {
    case SVC_YHM2712_TRANSITION_ENSURE_CHARGE:
        return "ensure_charge";
    case SVC_YHM2712_TRANSITION_RESUME_ACTIVE:
        return "resume_active";
    default:
        return "invalid";
    }
}

static const char *svc_yhm2712_transition_action_name(
    svc_yhm2712_transition_action_t action)
{
    switch (action)
    {
    case SVC_YHM2712_ACTION_KEEP_CHARGE:
        return "keep_charge";
    case SVC_YHM2712_ACTION_SLEEP_TO_CHARGE:
        return "sleep_to_charge";
    case SVC_YHM2712_ACTION_APPLY_CHARGE:
        return "apply_charge";
    case SVC_YHM2712_ACTION_SLEEP_TO_DISCHARGE:
        return "sleep_to_discharge";
    case SVC_YHM2712_ACTION_APPLY_DISCHARGE:
        return "apply_discharge";
    default:
        return "none";
    }
}

static const char *svc_yhm2712_transition_outcome_name(
    svc_yhm2712_transition_outcome_t outcome)
{
    switch (outcome)
    {
    case SVC_YHM2712_TRANSITION_OK:
        return "ok";
    case SVC_YHM2712_TRANSITION_ACMD_ERROR:
        return "acmd_error";
    case SVC_YHM2712_TRANSITION_BAD_REQUEST:
        return "bad_request";
    default:
        return "invalid";
    }
}

static void svc_yhm2712_transition_result_set(
    svc_yhm2712_transition_result_t *result,
    svc_yhm2712_transition_outcome_t outcome,
    yhm2712_acmd_status_t acmd_status,
    svc_yhm2712_transition_action_t action)
{
    if (result == NULL)
    {
        return;
    }

    result->outcome = outcome;
    result->acmd_status = acmd_status;
    result->action = action;
    result->status = s_svc_yhm2712_status;
    result->transport_settled = false;
    result->recovery.valid = false;
}

static svc_yhm2712_transition_outcome_t svc_yhm2712_transition_finish(
    const svc_yhm2712_transition_request_t *request,
    svc_yhm2712_transition_result_t *result,
    svc_yhm2712_transition_outcome_t outcome,
    yhm2712_acmd_status_t acmd_status,
    svc_yhm2712_transition_action_t action)
{
#if !YHM_PRODUCTION_COMPACT
    yhm2712_acmd_failure_snapshot_t failure;
#endif
    yhm2712_acmd_recovery_snapshot_t recovery;
    const char *reason = ((request != NULL) && (request->reason != NULL)) ?
                         request->reason : "transition";
    svc_yhm2712_transition_event_t event =
        (request != NULL) ? request->event :
        SVC_YHM2712_TRANSITION_ENSURE_CHARGE;
    bool external_power_present =
        (request != NULL) && request->external_power_present;
    bool keep_charge = (action == SVC_YHM2712_ACTION_KEEP_CHARGE);

    svc_yhm2712_transition_result_set(result, outcome, acmd_status, action);
    if (yhm2712_acmd_converge_status_input(&recovery))
    {
        if (result != NULL)
        {
            result->transport_settled = true;
            result->recovery = recovery;
        }
    }
    else if (result != NULL)
    {
        result->recovery = recovery;
    }
#if !YHM_PRODUCTION_COMPACT
    if ((outcome == SVC_YHM2712_TRANSITION_ACMD_ERROR) &&
        yhm2712_acmd_failure_snapshot_get(&failure) &&
        (failure.status == acmd_status))
    {
        YHM_DETAIL_LOG("[YHM][ACMD_E] r=%02x d=%c st=%s e=%s p=%s",
                   (uint32_t)failure.reg,
                   failure.is_write ? 'W' : 'R',
                   yhm2712_acmd_stage_name(failure.stage),
                   yhm2712_acmd_status_name(failure.status),
                   yhm2712_acmd_ack_path_name(failure.ack_path));
        YHM_DETAIL_LOG("[YHM][ACMD_D] tail=%u entry=%u hw=%lu fw=%lu lo=%lu sy=%u mf=%u bit=%u",
                   (uint32_t)failure.tail_drive_high,
                   (uint32_t)failure.ack_entry_level,
                   (unsigned long)failure.high_wait,
                   (unsigned long)failure.fall_wait,
                   (unsigned long)failure.low_cycles,
                   (uint32_t)failure.symbol,
                   (uint32_t)failure.missed_fall,
                   (uint32_t)failure.data_bit_count);
    }
#endif
    YHM_DETAIL_LOG("[YHM][T] ev=%s rs=%s ac=%u fv=%u fsm=%u act=%s",
               svc_yhm2712_transition_event_name(event),
               reason,
               external_power_present ? 1U : 0U,
               s_svc_yhm2712_status.fsm_valid ? 1U : 0U,
               (uint32_t)s_svc_yhm2712_status.fsm,
               svc_yhm2712_transition_action_name(action));
    YHM_DETAIL_LOG("[YHM][R] out=%s err=%s skip=%u",
               svc_yhm2712_transition_outcome_name(outcome),
               yhm2712_acmd_status_name(acmd_status),
               keep_charge ? 1U : 0U);
    YHM_DETAIL_LOG("[YHM][SETTLE] ok=%u irq=%lu/%lu pin=%u line=%u busy=%u dir=0x%08lx src=0x%08lx",
               recovery.valid && recovery.status_input_restored &&
               recovery.primask_restored && recovery.busy_cleared ? 1U : 0U,
               (unsigned long)recovery.primask_before,
               (unsigned long)recovery.primask_after,
               recovery.status_input_restored ? 1U : 0U,
               (uint32_t)recovery.line_level,
               recovery.busy_cleared ? 0U : 1U,
               (unsigned long)recovery.gpio_datadir,
               (unsigned long)recovery.gpio_datasrc);
    return outcome;
}

#if YHM_PRODUCTION_COMPACT
static
#endif
svc_yhm2712_transition_outcome_t svc_yhm2712_charge_apply_transition(
    const svc_yhm2712_transition_request_t *request,
    svc_yhm2712_transition_result_t *result)
{
    yhm2712_acmd_status_t status;
    svc_yhm2712_transition_action_t action = SVC_YHM2712_ACTION_NONE;
    bool charge_target;

    if ((request == NULL) ||
        (request->event > SVC_YHM2712_TRANSITION_RESUME_ACTIVE) ||
        ((request->event == SVC_YHM2712_TRANSITION_ENSURE_CHARGE) &&
         !request->external_power_present) ||
        (request->sleep_exit_required &&
         ((request->event != SVC_YHM2712_TRANSITION_RESUME_ACTIVE) ||
          request->external_power_present)))
    {
        return svc_yhm2712_transition_finish(request,
                                             result,
                                             SVC_YHM2712_TRANSITION_BAD_REQUEST,
                                             YHM2712_ACMD_STATUS_BAD_PARAM,
                                             action);
    }

    if (!yhm2712_acmd_init(s_svc_yhm2712_stacmd_pin))
    {
        s_svc_yhm2712_status.fsm_valid = false;
        s_svc_yhm2712_status.valid = false;
        return svc_yhm2712_transition_finish(request,
                                             result,
                                             SVC_YHM2712_TRANSITION_ACMD_ERROR,
                                             YHM2712_ACMD_STATUS_BAD_PIN,
                                             action);
    }
    s_svc_yhm2712_inited = true;

    status = svc_yhm2712_select_physical_profile(
                 request->external_power_present,
                 request->reason);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        s_svc_yhm2712_status.fsm_valid = false;
        s_svc_yhm2712_status.valid = false;
        return svc_yhm2712_transition_finish(request,
                                             result,
                                             SVC_YHM2712_TRANSITION_ACMD_ERROR,
                                             status,
                                             action);
    }

    /*
     * YHM2712 Rev4.2 requires a system waking for large load to configure
     * the IC out of Sleep first.  app_power sets this flag only after the
     * preceding MODE=SLEEP transaction was verified, so this known-state
     * path must not put a STATUS2 query ahead of MODE=DISCHARGE.
     */
    if (request->sleep_exit_required)
    {
        action = SVC_YHM2712_ACTION_SLEEP_TO_DISCHARGE;
        status = svc_yhm2712_write_mode(YHM2712_MODE_SET_DISCHARGE,
                                        request->reason);
        if (status != YHM2712_ACMD_STATUS_OK)
        {
            return svc_yhm2712_transition_finish(
                       request,
                       result,
                       SVC_YHM2712_TRANSITION_ACMD_ERROR,
                       status,
                       action);
        }

        status = svc_yhm2712_charge_configure_profile(request->reason);
        if (status != YHM2712_ACMD_STATUS_OK)
        {
            return svc_yhm2712_transition_finish(
                       request,
                       result,
                       SVC_YHM2712_TRANSITION_ACMD_ERROR,
                       status,
                       action);
        }

        return svc_yhm2712_transition_finish(request,
                                             result,
                                             SVC_YHM2712_TRANSITION_OK,
                                             YHM2712_ACMD_STATUS_OK,
                                             action);
    }

    status = svc_yhm2712_read_transition_fsm();
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return svc_yhm2712_transition_finish(request,
                                             result,
                                             SVC_YHM2712_TRANSITION_ACMD_ERROR,
                                             status,
                                             action);
    }

    charge_target = request->external_power_present;
    if (charge_target &&
        ((s_svc_yhm2712_status.fsm == YHM2712_FSM_MODE_CHARGE) ||
         (s_svc_yhm2712_status.fsm == YHM2712_FSM_MODE_CHARGE_DONE)))
    {
        return svc_yhm2712_transition_finish(request,
                                             result,
                                             SVC_YHM2712_TRANSITION_OK,
                                             YHM2712_ACMD_STATUS_OK,
                                             SVC_YHM2712_ACTION_KEEP_CHARGE);
    }

    if (s_svc_yhm2712_status.fsm == YHM2712_FSM_MODE_SLEEP)
    {
        action = charge_target ? SVC_YHM2712_ACTION_SLEEP_TO_CHARGE :
                 SVC_YHM2712_ACTION_SLEEP_TO_DISCHARGE;
        status = svc_yhm2712_write_mode(YHM2712_MODE_SET_DISCHARGE,
                                        request->reason);
        if (status != YHM2712_ACMD_STATUS_OK)
        {
            return svc_yhm2712_transition_finish(
                       request,
                       result,
                       SVC_YHM2712_TRANSITION_ACMD_ERROR,
                       status,
                       action);
        }
    }
    else
    {
        action = charge_target ? SVC_YHM2712_ACTION_APPLY_CHARGE :
                 SVC_YHM2712_ACTION_APPLY_DISCHARGE;
    }

    status = (request->event == SVC_YHM2712_TRANSITION_ENSURE_CHARGE) ?
             svc_yhm2712_charge_wake_for_charge_only() :
             svc_yhm2712_charge_wake_for_active(
                 request->external_power_present,
                 request->reason);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return svc_yhm2712_transition_finish(request,
                                             result,
                                             SVC_YHM2712_TRANSITION_ACMD_ERROR,
                                             status,
                                             action);
    }

    return svc_yhm2712_transition_finish(request,
                                         result,
                                         SVC_YHM2712_TRANSITION_OK,
                                         YHM2712_ACMD_STATUS_OK,
                                         action);
}

#if YHM_PRODUCTION_COMPACT
static
#endif
yhm2712_acmd_status_t svc_yhm2712_charge_prepare_full_sleep(void)
{
    yhm2712_acmd_status_t status;

    status = svc_yhm2712_select_physical_profile(false, "full_sleep");
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return status;
    }

    status = svc_yhm2712_online_check("full_sleep");
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return status;
    }

    return svc_yhm2712_write_mode(YHM2712_MODE_SET_SLEEP, "full_sleep");
}

#if ZY100_BUILD_FACTORY || ZY100_BUILD_PRODUCTION || ZY100_BUILD_SHIPPING_TEST
#if YHM_PRODUCTION_COMPACT
#define YHM_SHIPPING_TRACE NULL
#else
static yhm2712_acmd_attempt_trace_t s_svc_yhm2712_shipping_trace;
#define YHM_SHIPPING_TRACE (&s_svc_yhm2712_shipping_trace)
#endif

#if !YHM_PRODUCTION_COMPACT
static const char *svc_yhm2712_factory_attempt_result_name(
    yhm2712_acmd_attempt_result_t result)
{
    switch (result)
    {
    case YHM2712_ACMD_ATTEMPT_RESULT_SUCCESS:
        return "success";
    case YHM2712_ACMD_ATTEMPT_RESULT_WRITE_FAIL:
        return "write_fail";
    case YHM2712_ACMD_ATTEMPT_RESULT_READ_FAIL:
        return "read_fail";
    case YHM2712_ACMD_ATTEMPT_RESULT_READBACK_MISMATCH:
        return "readback_mismatch";
    default:
        return "unknown";
    }
}
#endif

#if !YHM_PRODUCTION_COMPACT
static void svc_yhm2712_factory_attempt_trace_log(
    uint8_t reg,
    const yhm2712_acmd_attempt_trace_t *trace)
{
    uint8_t idx;
    uint8_t write_fail_count = 0U;
    uint8_t read_fail_count = 0U;
    uint8_t mismatch_count = 0U;
    uint8_t success_count = 0U;

    if (trace == NULL)
    {
        return;
    }

    for (idx = 0U; idx < trace->count; idx++)
    {
        const yhm2712_acmd_attempt_diag_t *entry = &trace->entry[idx];
        const char *stage_name = "none";
        const char *path_name = "none";

        if (entry->result == YHM2712_ACMD_ATTEMPT_RESULT_WRITE_FAIL)
        {
            write_fail_count++;
        }
        else if (entry->result == YHM2712_ACMD_ATTEMPT_RESULT_READ_FAIL)
        {
            read_fail_count++;
        }
        else if (entry->result ==
                 YHM2712_ACMD_ATTEMPT_RESULT_READBACK_MISMATCH)
        {
            mismatch_count++;
            stage_name = "verify";
        }
        else if (entry->result == YHM2712_ACMD_ATTEMPT_RESULT_SUCCESS)
        {
            success_count++;
        }

        if (entry->failure_valid)
        {
            stage_name = yhm2712_acmd_stage_name(entry->failure_stage);
            path_name = yhm2712_acmd_ack_path_name(entry->ack_path);
        }

        /* Keep each line below the UART diagnostic buffer limit. */
        YHM_DETAIL_LOG("[YHM2712][AT] r=%02x n=%u rs=%s w=%s rd=%s v=%u rb=%02x m=%u st=%s p=%s",
                   (uint32_t)entry->reg,
                   (uint32_t)entry->attempt,
                   svc_yhm2712_factory_attempt_result_name(entry->result),
                   entry->write_performed ?
                       yhm2712_acmd_status_name(entry->write_status) :
                       "not_run",
                   entry->read_performed ?
                       yhm2712_acmd_status_name(entry->read_status) :
                       "not_run",
                   entry->readback_valid ? 1U : 0U,
                   (uint32_t)entry->readback,
                   entry->readback_match ? 1U : 0U,
                   stage_name,
                   path_name);

        if (entry->ack_diag_valid)
        {
            YHM_DETAIL_LOG("[YHM2712][ACKP] r=%02x n=%u ok=%u st=%u e=%u a=%u l=%lu pc=%lu mg=%lu",
                       (uint32_t)entry->reg,
                       (uint32_t)entry->attempt,
                       (entry->ack_diag_status == YHM2712_ACMD_STATUS_OK) ? 1U : 0U,
                       (uint32_t)entry->ack_diag_stage,
                       (uint32_t)entry->expected_symbol,
                       (uint32_t)entry->actual_symbol,
                       (unsigned long)entry->ack_low_ticks,
                       (unsigned long)entry->ack_low_poll_count,
                       (unsigned long)entry->ack_max_tick_step);
            YHM_DETAIL_LOG("[YHM2712][ACKG] r=%02x n=%u ls=%lu rt=%lu pm=%u d=%u s=%u db=%u sy=%u",
                       (uint32_t)entry->reg,
                       (uint32_t)entry->attempt,
                       (unsigned long)entry->ack_low_start_tick,
                       (unsigned long)entry->ack_rise_tick,
                       (uint32_t)entry->ack_primask,
                       (uint32_t)entry->ack_dir_m,
                       (uint32_t)entry->ack_datasrc_m,
                       (uint32_t)entry->ack_debounce_m,
                       (uint32_t)entry->ack_lssync_m);
        }

        if (entry->failure_valid)
        {
            YHM_DETAIL_LOG("[YHM2712][ACK] r=%02x n=%u e=%u a=%u h=%lu f=%lu l=%lu t=%lu mf=%u",
                       (uint32_t)entry->reg,
                       (uint32_t)entry->attempt,
                       (uint32_t)entry->expected_symbol,
                       (uint32_t)entry->actual_symbol,
                       (unsigned long)entry->high_wait,
                       (unsigned long)entry->fall_wait,
                       (unsigned long)entry->ack_low_ticks,
                       (unsigned long)entry->turnaround_tail_ticks,
                       (uint32_t)entry->missed_fall);

        }
    }

    YHM_DETAIL_LOG("[YHM2712][ATTEMPT_SUMMARY] reg=0x%02x total=%u write_fail=%u read_fail=%u mismatch=%u success=%u",
               (uint32_t)reg,
               (uint32_t)trace->count,
               (uint32_t)write_fail_count,
               (uint32_t)read_fail_count,
               (uint32_t)mismatch_count,
               (uint32_t)success_count);
}
#else
#define svc_yhm2712_factory_attempt_trace_log(reg, trace) ((void)0)
#endif

#if !YHM_PRODUCTION_COMPACT
static void svc_yhm2712_factory_shipping_log_failure(
    const char *step,
    uint8_t reg,
    yhm2712_acmd_status_t status)
{
    yhm2712_acmd_failure_snapshot_t failure;

    if (!yhm2712_acmd_failure_snapshot_get(&failure) ||
        (failure.status != status))
    {
        YHM_DETAIL_LOG("[YHM2712][SHIP][ERR] step=%s reg=0x%02x status=%s snapshot=none",
                   step,
                   (uint32_t)reg,
                   yhm2712_acmd_status_name(status));
        return;
    }

    YHM_DETAIL_LOG("[YHM2712][SHIP][ERR] step=%s reg=0x%02x st=%s path=%s exp=%u got=%u fw=%lu lo=%lu tail=%lu",
               step,
               (uint32_t)reg,
               yhm2712_acmd_stage_name(failure.stage),
               yhm2712_acmd_ack_path_name(failure.ack_path),
               (uint32_t)failure.expected_symbol,
               (uint32_t)failure.symbol,
               (unsigned long)failure.fall_wait,
               (unsigned long)failure.low_cycles,
               (unsigned long)failure.turnaround_tail_cycles);
}
#else
#define svc_yhm2712_factory_shipping_log_failure(step, reg, status) ((void)0)
#endif

#if ZY100_YHM_HAS_MOS && !YHM_PRODUCTION_COMPACT
/* Owning task, between transactions only. Preserve all three reads so a
 * service-level mismatch is not lost just because the transport returned OK.
 * No extra ACMD, threshold change, retry, or work in the capture loop. */
static yhm_pressure_diag_t s_shipping_rx[3];
static uint32_t s_shipping_rx_previous_sequence;

static void svc_yhm2712_shipping_rx_begin(void)
{
    uint8_t i;
    s_shipping_rx_previous_sequence =
        yhm_pressure_rx_snapshot_get(false, &s_shipping_rx[0].rx) ?
        s_shipping_rx[0].rx.sequence : 0U;
    for (i = 0U; i < 3U; ++i)
        s_shipping_rx[i] = (yhm_pressure_diag_t){0};
}

static void svc_yhm2712_shipping_rx_save(uint8_t phase, uint8_t reg)
{
    yhm_pressure_diag_t *d = &s_shipping_rx[phase];
    bool ack_available = yhm_mos_ack_timing_get(d);
    d->valid = yhm_pressure_rx_snapshot_get(false, &d->rx) &&
        d->rx.reg == reg && d->rx.sequence != s_shipping_rx_previous_sequence;
    if (d->valid) s_shipping_rx_previous_sequence = d->rx.sequence;
    /* A captured DATA pulse proves this read reached its address ACK. Do not
     * attach a previous transaction's ACK to an early/entry-low failure. */
    d->ack.valid = ack_available && d->valid && d->rx.bit_count != 0U &&
        d->ack.reg == reg && d->ack.stage == YHM2712_ACMD_STAGE_ADDR_R_ACK &&
        d->ack.status == YHM2712_ACMD_STATUS_OK;
}

static void svc_yhm2712_shipping_rx_dump(uint8_t reg,
    yhm2712_acmd_status_t status, uint8_t expected, uint8_t actual)
{
    uint8_t phase;
    uint8_t bit;
    const uint32_t mask = 0x03ffffffUL; /* Existing 26-bit counter. */
    YHM_DETAIL_LOG("[SHIP_RX] reg=%02x status=%u expected=%02x actual=%02x ticks_hz=80000000",
        reg, status, expected, actual);
    for (phase = 0U; phase < 3U; ++phase) {
        const yhm_pressure_diag_t *d = &s_shipping_rx[phase];
        YHM_DETAIL_LOG("[SHIP_RX] p=%u valid=%u seq=%lu reg=%02x value=%02x bits=%u",
            phase, d->valid, (unsigned long)d->rx.sequence,
            d->rx.reg, d->rx.value, d->rx.bit_count);
        if (!d->valid) continue;
        YHM_DETAIL_LOG("[SHIP_RX] p=%u transport=%u capture=%u entry_low=%u threshold=%u",
            phase, d->rx.status, d->rx.capture_status, d->rx.entry_low_bit,
            ZY100_YHM_DATA_THRESHOLD_TICKS);
        YHM_DETAIL_LOG("[SHIP_RX] p=%u ack_valid=%u start=%08lx fall=%08lx rise=%08lx",
            phase, d->ack.valid, (unsigned long)d->ack_start,
            (unsigned long)d->ack_fall, (unsigned long)d->ack_rise);
        YHM_DETAIL_LOG("[SHIP_RX] p=%u ack_low=%lu expected=%u got=%u entry_low=%u",
            phase, (unsigned long)d->ack.low_ticks, d->ack.expected,
            d->ack.symbol, d->ack_entry_low);
        for (bit = 0U; bit < d->rx.bit_count && bit < 8U; ++bit) {
            YHM_DETAIL_LOG("[SHIP_RX] p=%u bit=%u fall=%08lx rise=%08lx width=%lu",
                phase, 7U - bit, (unsigned long)d->rx.fall[bit],
                (unsigned long)d->rx.rise[bit],
                (unsigned long)((d->rx.rise[bit] - d->rx.fall[bit]) & mask));
        }
        YHM_DETAIL_LOG("[SHIP_RX] p=%u wait=%08lx timeout=%08lx",
            phase, (unsigned long)d->rx.wait_start,
            (unsigned long)d->rx.timeout_tick);
    }
}
#else
#define svc_yhm2712_shipping_rx_begin() ((void)0)
#define svc_yhm2712_shipping_rx_save(phase, reg) ((void)0)
#define svc_yhm2712_shipping_rx_dump(reg, status, expected, actual) ((void)0)
#endif

static yhm2712_acmd_status_t svc_yhm2712_factory_shipping_probe_reg(
    uint8_t reg)
{
    yhm2712_acmd_status_t status;
    uint8_t original = 0U;
    uint8_t internal_readback = 0U;
    uint8_t final_readback = 0U;
    uint8_t attempts = 0U;
    uint8_t read_attempts = 0U;

    svc_yhm2712_shipping_rx_begin();
    status = yhm2712_acmd_read_reg_retry_pre_idle_diag(
                 reg,
                 &original,
                 &read_attempts,
                 YHM_SHIPPING_TRACE);
    svc_yhm2712_shipping_rx_save(0U, reg);
    svc_yhm2712_factory_attempt_trace_log(
        reg,
        YHM_SHIPPING_TRACE);
#if !ZY100_YHM_HAS_MOS
    YHM_DETAIL_LOG("[YHM2712][SHIP_PROBE] reg=0x%02x phase=read pre_idle_us=0 status=%s value=0x%02x attempts=%u",
               (uint32_t)reg,
               yhm2712_acmd_status_name(status),
               (uint32_t)original,
               (uint32_t)read_attempts);
#endif
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        svc_yhm2712_shipping_rx_dump(reg, status, original, original);
        svc_yhm2712_factory_shipping_log_failure("probe_read", reg, status);
        return status;
    }

    status = yhm2712_acmd_write_reg_retry_readback_pre_idle_diag(
                 reg,
                 original,
                 &internal_readback,
                 &attempts,
                 YHM_SHIPPING_TRACE);
    svc_yhm2712_shipping_rx_save(1U, reg);
    svc_yhm2712_factory_attempt_trace_log(
        reg,
        YHM_SHIPPING_TRACE);
#if !ZY100_YHM_HAS_MOS
    YHM_DETAIL_LOG("[YHM2712][SHIP_PROBE] reg=0x%02x phase=write_original pre_idle_us=0 status=%s original=0x%02x readback=0x%02x attempts=%u",
               (uint32_t)reg,
               yhm2712_acmd_status_name(status),
               (uint32_t)original,
               (uint32_t)internal_readback,
               (uint32_t)attempts);
#endif
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        svc_yhm2712_shipping_rx_dump(reg, status, original, internal_readback);
        svc_yhm2712_factory_shipping_log_failure("probe_write", reg, status);
        return status;
    }

    status = yhm2712_acmd_read_reg_retry_pre_idle_diag(
                 reg,
                 &final_readback,
                 &read_attempts,
                 YHM_SHIPPING_TRACE);
    svc_yhm2712_shipping_rx_save(2U, reg);
    svc_yhm2712_factory_attempt_trace_log(
        reg,
        YHM_SHIPPING_TRACE);
#if !ZY100_YHM_HAS_MOS
    YHM_DETAIL_LOG("[YHM2712][SHIP_PROBE] reg=0x%02x phase=final_verify pre_idle_us=0 status=%s expected=0x%02x readback=0x%02x match=%u attempts=%u",
               (uint32_t)reg,
               yhm2712_acmd_status_name(status),
               (uint32_t)original,
               (uint32_t)final_readback,
               ((status == YHM2712_ACMD_STATUS_OK) &&
                (final_readback == original)) ? 1U : 0U,
               (uint32_t)read_attempts);
#endif
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        svc_yhm2712_shipping_rx_dump(reg, status, original, final_readback);
        svc_yhm2712_factory_shipping_log_failure("probe_verify", reg, status);
        return status;
    }
    if (final_readback != original)
    {
#if YHM_PRODUCTION_COMPACT
        yhm2712_acmd_value_error(reg, original, final_readback, YHM2712_ACMD_STATUS_READBACK_MISMATCH);
#elif ZY100_YHM_HAS_MOS
        yhm_mos_report_value_mismatch(reg, original, final_readback);
#endif
        svc_yhm2712_shipping_rx_dump(reg, YHM2712_ACMD_STATUS_READBACK_MISMATCH,
            original, final_readback);
        YHM_DETAIL_LOG("[YHM2712][SHIP][ERR] step=probe_verify reg=0x%02x status=readback_mismatch expected=0x%02x got=0x%02x",
                   (uint32_t)reg,
                   (uint32_t)original,
                   (uint32_t)final_readback);
        return YHM2712_ACMD_STATUS_READBACK_MISMATCH;
    }

    return YHM2712_ACMD_STATUS_OK;
}

#if ZY100_BUILD_FACTORY
yhm2712_acmd_status_t svc_yhm2712_charge_factory_resume_from_shipping(
    bool external_power_present)
{
    yhm2712_acmd_status_t status;

    /*
     * The persisted Factory marker proves that the previous boot completed a
     * verified MODE=SHIPPING transaction.  VIN wakes YHM into a valid live
     * CHARGE/CHARGE_DONE FSM, but that fact alone does not prove BAT will take
     * over SYS after VIN removal.  Re-arm the battery handover with one
     * verified MODE=DISCHARGE command while accepting the live VIN-powered
     * FSM.  The no-VIN path retains the known-state rule that requires the
     * observed DISCHARGE state before any profile query.
     */
    if (external_power_present)
    {
        status = svc_yhm2712_select_physical_profile(
                     true,
                     "factory_shipping_resume_vin");
        if (status != YHM2712_ACMD_STATUS_OK)
        {
            YHM_DETAIL_LOG("[YHM2712][RESUME][ERR] step=vin_profile_select status=%s",
                       yhm2712_acmd_status_name(status));
            return status;
        }

        status = yhm2712_acmd_factory_arm_battery_handover_with_input();
        if (status != YHM2712_ACMD_STATUS_OK)
        {
            YHM_DETAIL_LOG("[YHM2712][RESUME][ERR] step=vin_battery_arm status=%s mode=0x%02x",
                       yhm2712_acmd_status_name(status),
                       YHM2712_MODE_SET_DISCHARGE);
            return status;
        }

        status = svc_yhm2712_charge_configure_profile(
                     "factory_shipping_resume_vin");
        if (status != YHM2712_ACMD_STATUS_OK)
        {
            YHM_DETAIL_LOG("[YHM2712][RESUME][ERR] step=vin_profile status=%s",
                       yhm2712_acmd_status_name(status));
            return status;
        }

        if (!s_svc_yhm2712_status.fsm_valid ||
            ((s_svc_yhm2712_status.fsm != YHM2712_FSM_MODE_CHARGE) &&
             (s_svc_yhm2712_status.fsm != YHM2712_FSM_MODE_CHARGE_DONE)))
        {
            YHM_DETAIL_LOG("[YHM2712][RESUME][ERR] step=vin_post_arm_fsm fsm=%u status2=0x%02x",
                       (uint32_t)s_svc_yhm2712_status.fsm,
                       (uint32_t)s_svc_yhm2712_status.status2);
            return YHM2712_ACMD_STATUS_MODE_MISMATCH;
        }

    #if !ZY100_YHM_HAS_MOS
    YHM_DETAIL_LOG("[YHM2712][RESUME] battery_path=armed command=discharge observed_fsm=%u profile=ok ac=1",
                   (uint32_t)s_svc_yhm2712_status.fsm);
#endif
        return YHM2712_ACMD_STATUS_OK;
    }

    status = svc_yhm2712_select_physical_profile(
                 false,
                 "factory_shipping_resume");
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        YHM_DETAIL_LOG("[YHM2712][RESUME][ERR] step=profile status=%s ac=%u",
                   yhm2712_acmd_status_name(status),
                   0U);
        return status;
    }

    status = svc_yhm2712_write_mode(YHM2712_MODE_SET_DISCHARGE,
                                    "factory_shipping_resume");
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        YHM_DETAIL_LOG("[YHM2712][RESUME][ERR] step=discharge status=%s mode=0x%02x",
                   yhm2712_acmd_status_name(status),
                   YHM2712_MODE_SET_DISCHARGE);
        return status;
    }

    status = svc_yhm2712_charge_configure_profile("factory_shipping_resume");
#if !ZY100_YHM_HAS_MOS
    YHM_DETAIL_LOG("[YHM2712][RESUME] battery_path=%s target=discharge mode=0x%02x profile=%s ac=0",
               (status == YHM2712_ACMD_STATUS_OK) ? "ready" : "failed",
               YHM2712_MODE_SET_DISCHARGE,
               yhm2712_acmd_status_name(status));
#endif
    return status;
}
#endif

#if YHM_PRODUCTION_COMPACT
static
#endif
yhm2712_acmd_status_t svc_yhm2712_charge_factory_enter_shipping(void)
{
    yhm2712_acmd_status_t status;
    uint8_t attempts = 0U;

    status = svc_yhm2712_select_physical_profile(false, "factory_shipping");
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return status;
    }

    status = svc_yhm2712_factory_shipping_probe_reg(YHM2712_REG_V_CTRL);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        YHM_DETAIL_LOG("[YHM2712][SHIP][ERR] abort=reg0_probe mode_write=not_sent");
        return status;
    }

    status = svc_yhm2712_factory_shipping_probe_reg(YHM2712_REG_I_CTRL);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        YHM_DETAIL_LOG("[YHM2712][SHIP][ERR] abort=reg1_probe mode_write=not_sent");
        return status;
    }

    status = svc_yhm2712_online_check("factory_shipping");
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return status;
    }

    /*
     * Shipping may remove the MCU supply immediately.  Use the frozen ACMD
     * physical profile, but deliberately do not request a readback after the
     * MODE write.  The Factory host closes this transaction with expected BLE
     * loss plus the external current measurement.
     */
    /*
     * Keep the corrected edge-based transmit widths and the explicit final
     * high handoff needed to preserve the physical 0x18 waveform, but receive
     * both ACKs through the same complete-low-width decoder used by normal
     * transactions.  Production/normal write behavior stays unchanged.
     */
#if !ZY100_YHM_HAS_MOS
    YHM_DETAIL_LOG("[YHM2712][SHIP] plan sequence=reg0_reg1_then_mode pre_idle_us=0 scope=all_transactions quick=0x84 reg=0x02 data=0x18 bits=00011000 tx=edge_timed tx_high_reassert=3 tx_low_close=1 tx_det_byte=1 boundary_diag=0 ack_poll_diag=1 ack=width ack_handoff=single_release_all_bytes");
#endif
    status = yhm2712_acmd_write_reg_edge_timed_retry_no_readback_diag(
                 YHM2712_REG_MODE,
                 YHM2712_MODE_SET_SHIPPING,
                 &attempts,
                 YHM_SHIPPING_TRACE);
    svc_yhm2712_factory_attempt_trace_log(
        YHM2712_REG_MODE,
        YHM_SHIPPING_TRACE);
#if ZY100_YHM_HAS_MOS
    YHM_DETAIL_LOG("[YHM2712][SVC] factory_shipping write=%s attempts=%u",
#else
    YHM_DETAIL_LOG("[YHM2712][SVC] factory_shipping write=%s attempts=%u handoff=vendor_high ack=width",
#endif
               yhm2712_acmd_status_name(status),
               (uint32_t)attempts);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        svc_yhm2712_factory_shipping_log_failure("mode_write",
                                                 YHM2712_REG_MODE,
                                                 status);
    }
    return status;
}
#endif

#if YHM_PRODUCTION_COMPACT
static
#endif
yhm2712_acmd_status_t svc_yhm2712_charge_poll(uint32_t now_ms,
                                              uint16_t vbat_mv,
                                              bool chg_present)
{
    yhm2712_acmd_status_t status;

    if (!chg_present)
    {
        svc_yhm2712_reset_full_state();
        return YHM2712_ACMD_STATUS_OK;
    }

    if (s_svc_yhm2712_full_detected)
    {
        return YHM2712_ACMD_STATUS_OK;
    }

    if (!s_svc_yhm2712_full_window_open)
    {
        if (vbat_mv < SVC_YHM2712_FULL_CHECK_START_MV)
        {
            return YHM2712_ACMD_STATUS_OK;
        }
        s_svc_yhm2712_full_window_open = true;
        s_svc_yhm2712_full_poll_valid = false;
    }

    if (s_svc_yhm2712_full_poll_valid &&
        ((uint32_t)(now_ms - s_svc_yhm2712_last_full_poll_ms) <
         (uint32_t)SVC_YHM2712_FULL_POLL_INTERVAL_MS))
    {
        return YHM2712_ACMD_STATUS_OK;
    }

    status = svc_yhm2712_select_physical_profile(true, "poll");
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        s_svc_yhm2712_full_confirm_count = 0U;
        s_svc_yhm2712_full_poll_valid = false;
        return status;
    }

    status = svc_yhm2712_online_check("poll");
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        s_svc_yhm2712_full_confirm_count = 0U;
        s_svc_yhm2712_full_poll_valid = false;
        YHM_DETAIL_LOG("[YHM2712][SVC][ERR] poll_online_fail status=%s",
                   yhm2712_acmd_status_name(status));
        return status;
    }

    status = svc_yhm2712_read_status("poll");
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        s_svc_yhm2712_full_confirm_count = 0U;
        s_svc_yhm2712_full_poll_valid = false;
        YHM_DETAIL_LOG("[YHM2712][SVC][ERR] poll_status_fail status=%s",
                   yhm2712_acmd_status_name(status));
        return status;
    }
    s_svc_yhm2712_last_full_poll_ms = now_ms;
    s_svc_yhm2712_full_poll_valid = true;

    if (svc_yhm2712_charge_done_status(s_svc_yhm2712_status.chg_status,
                                       s_svc_yhm2712_status.fsm))
    {
        bool full_vbat_accepted = svc_yhm2712_full_vbat_accepted(vbat_mv);

        svc_yhm2712_note_full_done_status(vbat_mv, "poll");
        if (!full_vbat_accepted)
        {
            SVC_YHM_LOG("[YHM2712][SVC] recharge_kick reason=poll_done_low_vbat vbat=%u min=%u st1=0x%02x st2=0x%02x st3=0x%02x",
                        (uint32_t)vbat_mv,
                        (uint32_t)SVC_YHM2712_FULL_ACCEPT_MIN_MV,
                        (uint32_t)s_svc_yhm2712_status.status1,
                        (uint32_t)s_svc_yhm2712_status.status2,
                        (uint32_t)s_svc_yhm2712_status.status3);
            status = svc_yhm2712_write_mode(YHM2712_MODE_SET_CHARGE,
                                            "poll_done_low_vbat");
            if (status == YHM2712_ACMD_STATUS_OK)
            {
                SVC_YHM_LOG("[YHM2712][SVC] recharge_kick_result status=%s raw_done=%u chg_status=%u fsm=%u",
                            yhm2712_acmd_status_name(status),
                            svc_yhm2712_latest_done_status() ? 1U : 0U,
                            (uint32_t)s_svc_yhm2712_status.chg_status,
                            (uint32_t)s_svc_yhm2712_status.fsm);
            }
            else
            {
                YHM_DETAIL_LOG("[YHM2712][SVC][ERR] recharge_kick_fail status=%s",
                           yhm2712_acmd_status_name(status));
            }
            return status;
        }
        return YHM2712_ACMD_STATUS_OK;
    }

    if (svc_yhm2712_fault_or_backoff(s_svc_yhm2712_status.chg_status,
                                     s_svc_yhm2712_status.fsm))
    {
        s_svc_yhm2712_full_confirm_count = 0U;
        return YHM2712_ACMD_STATUS_OK;
    }

    s_svc_yhm2712_full_confirm_count = 0U;

    return YHM2712_ACMD_STATUS_OK;
}

#if ZY100_BUILD_PRODUCTION
#define SVC_CHARGE_ACCEPT_MV 4300U
#define SVC_CHARGE_CONFIRM_MAX_MS 90000UL
typedef struct
{
    uint32_t poll_ms;
    uint32_t sequence;
    uint32_t candidate_ms;
    uint8_t candidate; /* 0: none, 1: normal done, 2: low-voltage done */
    uint8_t diagnostic;
    bool poll_valid;
    bool recovery_checked;
    bool retry_used;
    bool retry_observe;
} svc_charge_session_t;
static svc_charge_session_t s_charge_session;

static void svc_charge_log(uint8_t event, const charge_voltage_sample_t *sample)
{
    if (s_charge_session.diagnostic == event) return;
    s_charge_session.diagnostic = event;
    DBG_DIRECT("[CHGF] ev=%u mv=%u valid=%u st=%02x/%02x/%02x retry=%u",
               event, sample != NULL ? sample->voltage_mv : 0U,
               sample != NULL && sample->valid ? 1U : 0U,
               s_svc_yhm2712_status.status1, s_svc_yhm2712_status.status2,
               s_svc_yhm2712_status.status3, s_charge_session.retry_used ? 1U : 0U);
}

void svc_yhm2712_charge_session_removed(void)
{
    s_charge_session = (svc_charge_session_t){0};
    svc_yhm2712_reset_full_state();
}

static yhm2712_acmd_status_t svc_charge_verify_profile(void)
{
    static const uint8_t regs[] = {YHM2712_REG_V_CTRL, YHM2712_REG_I_CTRL,
                                  YHM2712_REG_CONFIG};
    static const uint8_t expected[] = {YHM2712_PROFILE_V_CTRL_4350_TRICKLE_3000,
                                      YHM2712_PROFILE_I_CTRL_1X_ITERM_C20,
                                      YHM2712_PROFILE_CONFIG_DEFAULT_SAFE};
    uint8_t idx;
    for (idx = 0U; idx < 3U; ++idx)
    {
        uint8_t value = 0U;
        yhm2712_acmd_status_t status = yhm2712_acmd_read_reg(regs[idx], &value);
        DBG_DIRECT("[CHGF_CFG] reg=%u got=%02x expected=%02x err=%u",
                   regs[idx], value, expected[idx], status);
        if (status != YHM2712_ACMD_STATUS_OK) return status;
        if (value != expected[idx]) return YHM2712_ACMD_STATUS_READBACK_MISMATCH;
    }
    return YHM2712_ACMD_STATUS_OK;
}

static yhm2712_acmd_status_t svc_charge_retry_once(const charge_voltage_sample_t *sample)
{
    yhm2712_acmd_status_t status;
    if (s_charge_session.recovery_checked) return YHM2712_ACMD_STATUS_OK;
    /* One recovery evaluation per USB session, including failed configuration reads. */
    s_charge_session.recovery_checked = true;
    status = svc_charge_verify_profile();
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        svc_charge_log(5U, sample); /* configuration rejected */
        return status;
    }
    /* Recheck the state after configuration reads; never kick a fault/changed state. */
    status = svc_yhm2712_read_status("full_retry_check");
    if (status != YHM2712_ACMD_STATUS_OK ||
        s_svc_yhm2712_status.fsm != YHM2712_FSM_MODE_CHARGE_DONE ||
        s_svc_yhm2712_status.chg_status != YHM2712_CHG_STATUS_CHARGE_DONE)
    {
        svc_charge_log(6U, sample);
        return status;
    }
    s_charge_session.retry_used = true;
    s_charge_session.retry_observe = true;
    svc_charge_log(7U, sample); /* restart requested, not proof of current flow */
    status = svc_yhm2712_write_mode(YHM2712_MODE_SET_CHARGE, "full_retry_once");
    if (status != YHM2712_ACMD_STATUS_OK) svc_charge_log(8U, sample);
    return status;
}

yhm2712_acmd_status_t svc_yhm2712_charge_poll_sample(
    uint32_t now_ms, const charge_voltage_sample_t *sample, bool chg_present)
{
    yhm2712_acmd_status_t status;
    uint8_t candidate;
    uint32_t elapsed;
    if (!chg_present)
    {
        /* A raw absent observation cannot reset the stable USB session or its budget. */
        s_charge_session.candidate = 0U;
        return YHM2712_ACMD_STATUS_OK;
    }
    if (s_svc_yhm2712_full_detected) return YHM2712_ACMD_STATUS_OK;
    if (sample == NULL || !sample->valid || sample->sequence == 0U ||
        (uint32_t)(now_ms - sample->sampled_ms) > SVC_CHARGE_CONFIRM_MAX_MS)
    {
        s_charge_session.candidate = 0U;
        svc_charge_log(1U, sample); /* invalid/stale observation */
        return YHM2712_ACMD_STATUS_OK;
    }
    if (sample->sequence == s_charge_session.sequence) return YHM2712_ACMD_STATUS_OK;
    candidate = sample->voltage_mv >= SVC_CHARGE_ACCEPT_MV ? 1U : 2U;
    /* A fresh voltage crossing breaks continuity even between chip status polls. */
    if (s_charge_session.candidate != candidate) s_charge_session.candidate = 0U;
    if (s_charge_session.poll_valid &&
        (uint32_t)(now_ms - s_charge_session.poll_ms) < SVC_YHM2712_FULL_POLL_INTERVAL_MS)
        return YHM2712_ACMD_STATUS_OK;
    /* Consume failed attempts too, so communication faults cannot cause a tight loop. */
    s_charge_session.poll_ms = now_ms;
    s_charge_session.poll_valid = true;
    s_charge_session.sequence = sample->sequence;
    status = svc_yhm2712_select_physical_profile(true, "full_poll");
    if (status == YHM2712_ACMD_STATUS_OK) status = svc_yhm2712_online_check("full_poll");
    if (status == YHM2712_ACMD_STATUS_OK) status = svc_yhm2712_read_status("full_poll");
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        s_charge_session.candidate = 0U;
        svc_charge_log(2U, sample);
        return status;
    }
    if (s_charge_session.retry_observe)
    {
        if (s_svc_yhm2712_status.fsm == YHM2712_FSM_MODE_CHARGE &&
            (s_svc_yhm2712_status.chg_status == YHM2712_CHG_STATUS_CC_CHARGE ||
             s_svc_yhm2712_status.in_cv))
        {
            svc_charge_log(9U, sample); /* charging state observed; current not measured */
            s_charge_session.retry_observe = false;
        }
        else if (s_svc_yhm2712_status.fsm == YHM2712_FSM_MODE_CHARGE_DONE &&
                 s_svc_yhm2712_status.chg_status == YHM2712_CHG_STATUS_CHARGE_DONE)
        {
            svc_charge_log(10U, sample); /* still done, no charging state observed */
            s_charge_session.retry_observe = false;
        }
    }
    if (s_svc_yhm2712_status.fsm != YHM2712_FSM_MODE_CHARGE_DONE ||
        s_svc_yhm2712_status.chg_status != YHM2712_CHG_STATUS_CHARGE_DONE)
    {
        s_charge_session.candidate = 0U;
        svc_charge_log(3U, sample); /* charging/fault/inconsistent, never full */
        return YHM2712_ACMD_STATUS_OK;
    }
    elapsed = (uint32_t)(sample->sampled_ms - s_charge_session.candidate_ms);
    if (s_charge_session.candidate != candidate || elapsed > SVC_CHARGE_CONFIRM_MAX_MS)
    {
        s_charge_session.candidate = candidate;
        s_charge_session.candidate_ms = sample->sampled_ms;
        svc_charge_log(candidate == 1U ? 4U : 11U, sample);
        return YHM2712_ACMD_STATUS_OK;
    }
    if (elapsed < SVC_YHM2712_FULL_POLL_INTERVAL_MS) return YHM2712_ACMD_STATUS_OK;
    s_charge_session.candidate_ms = sample->sampled_ms;
    if (candidate == 1U)
    {
        s_svc_yhm2712_full_detected = true;
        svc_charge_log(12U, sample);
        return YHM2712_ACMD_STATUS_OK;
    }
    return svc_charge_retry_once(sample);
}
#endif

bool svc_yhm2712_charge_full_detected(void)
{
    return s_svc_yhm2712_full_detected;
}

void svc_yhm2712_charge_reset_full_detected(void)
{
    svc_yhm2712_reset_full_state();
}

bool svc_yhm2712_charge_status_done_latest(void)
{
    return svc_yhm2712_latest_done_status();
}

bool svc_yhm2712_charge_status_latest(svc_yhm2712_charge_status_t *status)
{
    if (status == NULL)
    {
        return false;
    }

    *status = s_svc_yhm2712_status;
    return s_svc_yhm2712_status.valid;
}

bool svc_yhm2712_charge_boot_diag_latest(yhm2712_acmd_boot_diag_t *diag)
{
    if ((diag == NULL) || !s_svc_yhm2712_boot_diag_valid)
    {
        return false;
    }

    *diag = s_svc_yhm2712_boot_diag;
    return true;
}

#if YHM_PRODUCTION_COMPACT
#undef svc_yhm2712_charge_init_status
#undef svc_yhm2712_charge_configure_profile
#undef svc_yhm2712_charge_wake_for_charge_only
#undef svc_yhm2712_charge_wake_for_active
#undef svc_yhm2712_charge_prepare_full_sleep
#undef svc_yhm2712_charge_factory_enter_shipping
#undef svc_yhm2712_charge_poll
#undef svc_yhm2712_charge_apply_transition
static void svc_yhm_compact_summary(const char *reason, yhm2712_acmd_status_t status,
                                    const yhm2712_comm_stats_t *before, bool force)
{
    yhm2712_comm_stats_t after;
    uint32_t tx, io, values, retries, extra;
    (void)yhm2712_acmd_comm_stats_get(&after);
    tx = after.transactions - before->transactions;
    io = after.io_errors - before->io_errors;
    values = after.value_errors - before->value_errors;
    retries = after.retries - before->retries;
    extra = after.id_extra_reads - before->id_extra_reads;
    if (!force && tx == 0U && io == 0U && values == 0U && status == YHM2712_ACMD_STATUS_OK) return;
    if (status == YHM2712_ACMD_STATUS_OK && io == 0U && values == 0U)
    {
        ZY100_LOG_EVENT("[YHM] phase=%s status=ok", reason != NULL ? reason : "unknown");
        return;
    }
    ZY100_LOG_ERROR("[YHM_SUM] phase=%s status=%s tx=%lu io=%lu val=%lu retry=%lu id_extra=%lu",
               reason != NULL ? reason : "unknown", yhm2712_acmd_status_name(status),
               (unsigned long)tx, (unsigned long)io, (unsigned long)values,
               (unsigned long)retries, (unsigned long)extra);
}
yhm2712_acmd_status_t svc_yhm2712_charge_init_status(uint8_t stacmd_pin,
                             bool external_power_present)
{
    yhm2712_comm_stats_t before;
    yhm2712_acmd_status_t status;
    (void)yhm2712_acmd_comm_stats_get(&before);
    status = svc_yhm2712_charge_init_status_impl(stacmd_pin, external_power_present);
    svc_yhm_compact_summary("init", status, &before, true);
    return status;
}

yhm2712_acmd_status_t svc_yhm2712_charge_configure_profile(const char *reason)
{
    yhm2712_comm_stats_t before;
    yhm2712_acmd_status_t status;
    (void)yhm2712_acmd_comm_stats_get(&before);
    status = svc_yhm2712_charge_configure_profile_impl(reason);
    svc_yhm_compact_summary(reason, status, &before, true);
    return status;
}

yhm2712_acmd_status_t svc_yhm2712_charge_wake_for_charge_only(void)
{
    yhm2712_comm_stats_t before;
    yhm2712_acmd_status_t status;
    (void)yhm2712_acmd_comm_stats_get(&before);
    status = svc_yhm2712_charge_wake_for_charge_only_impl();
    svc_yhm_compact_summary("charge_wake", status, &before, true);
    return status;
}

yhm2712_acmd_status_t svc_yhm2712_charge_wake_for_active(bool chg_present,
                                                         const char *reason)
{
    yhm2712_comm_stats_t before;
    yhm2712_acmd_status_t status;
    (void)yhm2712_acmd_comm_stats_get(&before);
    status = svc_yhm2712_charge_wake_for_active_impl(chg_present, reason);
    svc_yhm_compact_summary(reason, status, &before, true);
    return status;
}

yhm2712_acmd_status_t svc_yhm2712_charge_prepare_full_sleep(void)
{
    yhm2712_comm_stats_t before;
    yhm2712_acmd_status_t status;
    (void)yhm2712_acmd_comm_stats_get(&before);
    status = svc_yhm2712_charge_prepare_full_sleep_impl();
    svc_yhm_compact_summary("full_sleep", status, &before, true);
    return status;
}

yhm2712_acmd_status_t svc_yhm2712_charge_factory_enter_shipping(void)
{
    yhm2712_comm_stats_t before;
    yhm2712_acmd_status_t status;
    (void)yhm2712_acmd_comm_stats_get(&before);
    status = svc_yhm2712_charge_factory_enter_shipping_impl();
    svc_yhm_compact_summary("shipping", status, &before, true);
    return status;
}

yhm2712_acmd_status_t svc_yhm2712_charge_poll(uint32_t now_ms,
                                              uint16_t vbat_mv,
                                              bool chg_present)
{
    yhm2712_comm_stats_t before;
    yhm2712_acmd_status_t status;
    (void)yhm2712_acmd_comm_stats_get(&before);
    status = svc_yhm2712_charge_poll_impl(now_ms, vbat_mv, chg_present);
    svc_yhm_compact_summary("poll", status, &before, false);
    return status;
}

svc_yhm2712_transition_outcome_t svc_yhm2712_charge_apply_transition(
    const svc_yhm2712_transition_request_t *request,
    svc_yhm2712_transition_result_t *result)
{
    yhm2712_comm_stats_t before;
    svc_yhm2712_transition_result_t local_result;
    svc_yhm2712_transition_outcome_t outcome;
    svc_yhm2712_transition_result_t *effective = result != NULL ? result : &local_result;
    (void)yhm2712_acmd_comm_stats_get(&before);
    outcome = svc_yhm2712_charge_apply_transition_impl(request, effective);
    svc_yhm_compact_summary(request != NULL ? request->reason : "transition", effective->transport_settled ? effective->acmd_status : YHM2712_ACMD_STATUS_BUSY, &before, true);
    return outcome;
}
#endif

bool svc_yhm2712_charge_init(uint8_t pin, bool external)
{
    return svc_yhm2712_charge_init_status(pin, external) == YHM2712_ACMD_STATUS_OK;
}
