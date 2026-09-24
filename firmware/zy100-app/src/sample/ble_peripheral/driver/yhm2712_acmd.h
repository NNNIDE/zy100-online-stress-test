/**
*****************************************************************************************
*     ZY100 YHM2712 Access Command driver
*****************************************************************************************
*/

#ifndef YHM2712_ACMD_H
#define YHM2712_ACMD_H

#include "yhm2712_hw_config.h"
#include <stdint.h>
#include <stdbool.h>

/* Production diagnostics are compiled out; wire configuration is independent. */
#if defined(ZY100_BUILD_PRODUCTION) && ZY100_BUILD_PRODUCTION
#define YHM_PRODUCTION_COMPACT 1
#else
#define YHM_PRODUCTION_COMPACT 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define YHM2712_ACMD_SLAVE_ADDRESS          0x04U
/* Frozen V1.1 ACMD baseline validated with logic-analyzer capture. */
#define YHM2712_ACMD_FORMAL_BASELINE_CODE   10341U

#define YHM2712_REG_V_CTRL                  0x00U
#define YHM2712_REG_I_CTRL                  0x01U
#define YHM2712_REG_MODE                    0x02U
#define YHM2712_REG_CONFIG                  0x03U
#define YHM2712_REG_STATUS1                 0x05U
#define YHM2712_REG_STATUS2                 0x06U
#define YHM2712_REG_STATUS3                 0x07U
#define YHM2712_REG_ID                      0x08U

#define YHM2712_ACMD_BOOT_REG_COUNT         3U
#define YHM2712_ACMD_ID_STABLE_READ_COUNT   5U
#if defined(ZY100_BUILD_FACTORY) && ZY100_BUILD_FACTORY
/* Factory permits a longer observation window, not fewer correct IDs. */
#define YHM2712_ACMD_ID_MAX_ATTEMPTS        20U
#else
#define YHM2712_ACMD_ID_MAX_ATTEMPTS        10U
#endif

#define YHM2712_ID_DEFAULT                  0xA0U
#define YHM2712_STATUS2_FSM_MODE_MASK       0xF0U
#define YHM2712_STATUS2_FSM_MODE_SHIFT      4U
#define YHM2712_STATUS1_CHG_STATUS_MASK     0xE0U
#define YHM2712_STATUS1_CHG_STATUS_SHIFT    5U
#define YHM2712_STATUS1_ISNS_LT_ITERM_MASK  0x02U
#define YHM2712_STATUS1_VSYS_GT_VBAT_MASK   0x01U
#define YHM2712_STATUS3_CV_BAR_MASK         0x80U
#define YHM2712_CHG_STATUS_DISCHARGE        0x00U
#define YHM2712_CHG_STATUS_PRE_CHARGE       0x01U
#define YHM2712_CHG_STATUS_TRICKLE_CHARGE   0x02U
#define YHM2712_CHG_STATUS_CC_CHARGE        0x03U
#define YHM2712_CHG_STATUS_CV_CHARGE        0x05U
#define YHM2712_CHG_STATUS_CHARGE_DONE      0x07U
#define YHM2712_FSM_MODE_SLEEP              0x02U
#define YHM2712_FSM_MODE_DISCHARGE          0x08U
#define YHM2712_FSM_MODE_FAULT              0x09U
#define YHM2712_FSM_MODE_START              0x0AU
#define YHM2712_FSM_MODE_SYS_PRE            0x0BU
#define YHM2712_FSM_MODE_CHARGE             0x0CU
#define YHM2712_FSM_MODE_CHARGE_DONE        0x0DU
#define YHM2712_FSM_MODE_STOP_CHARGE        0x0FU
#define YHM2712_PROFILE_V_CTRL_4350_TRICKLE_3000 0x64U
#define YHM2712_PROFILE_I_CTRL_1X_ITERM_C20      0x80U
#define YHM2712_PROFILE_CONFIG_DEFAULT_SAFE      0x02U
#define YHM2712_MODE_SET_SLEEP              0x28U
#define YHM2712_MODE_SET_SHIPPING           0x18U
#define YHM2712_MODE_SET_DISCHARGE          0x88U
#define YHM2712_MODE_SET_CHARGE             0xC8U

typedef enum
{
    YHM2712_ACMD_STATUS_OK = 0,
    YHM2712_ACMD_STATUS_NOT_INIT,
    YHM2712_ACMD_STATUS_BAD_PARAM,
    YHM2712_ACMD_STATUS_BAD_PIN,
    YHM2712_ACMD_STATUS_DWT_UNAVAILABLE,
    YHM2712_ACMD_STATUS_TICK_STUCK,
    YHM2712_ACMD_STATUS_LINE_BUSY_LOW,
    YHM2712_ACMD_STATUS_TIMEOUT_FALL,
    YHM2712_ACMD_STATUS_TIMEOUT_RISE,
    YHM2712_ACMD_STATUS_BAD_SYMBOL,
    YHM2712_ACMD_STATUS_ACK_MISMATCH,
    YHM2712_ACMD_STATUS_ID_MISMATCH,
    YHM2712_ACMD_STATUS_RELEASE_TIMEOUT,
    YHM2712_ACMD_STATUS_RX_ENTRY_LOW,
    YHM2712_ACMD_STATUS_BUSY,
    YHM2712_ACMD_STATUS_SELFTEST_LINE_LOW,
    YHM2712_ACMD_STATUS_SELFTEST_RELEASE_INITIAL_FAIL,
    YHM2712_ACMD_STATUS_SELFTEST_DRIVE_LOW_FAIL,
    YHM2712_ACMD_STATUS_SELFTEST_RELEASE_AFTER_FAIL,
    YHM2712_ACMD_STATUS_MODE_MISMATCH,
    YHM2712_ACMD_STATUS_READBACK_MISMATCH
} yhm2712_acmd_status_t;

typedef enum
{
    YHM2712_ACMD_STAGE_LINE_IDLE = 0,
    YHM2712_ACMD_STAGE_ADDR_W_ACK,
    YHM2712_ACMD_STAGE_REG_ACK,
    YHM2712_ACMD_STAGE_ADDR_R_ACK,
    YHM2712_ACMD_STAGE_DATA_BIT,
    YHM2712_ACMD_STAGE_DATA_ACK,
    YHM2712_ACMD_STAGE_MASTER_NACK,
    YHM2712_ACMD_STAGE_STOP
} yhm2712_acmd_stage_t;

typedef enum
{
    YHM2712_ACMD_ACK_PATH_NONE = 0,
    YHM2712_ACMD_ACK_PATH_NORMAL,
    YHM2712_ACMD_ACK_PATH_TURNAROUND_TAIL,
    YHM2712_ACMD_ACK_PATH_STUCK_LOW
} yhm2712_acmd_ack_path_t;

typedef struct
{
    bool valid;
    bool is_write;
    uint8_t reg;
    yhm2712_acmd_stage_t stage;
    yhm2712_acmd_status_t status;
    yhm2712_acmd_ack_path_t ack_path;
    uint32_t high_wait;
    uint32_t fall_wait;
    uint32_t low_cycles;
    uint32_t turnaround_tail_cycles;
    uint8_t expected_symbol;
    uint8_t symbol;
    uint8_t missed_fall;
    uint8_t data_bit_count;
    uint8_t tail_drive_high;
    uint8_t ack_entry_level;
} yhm2712_acmd_failure_snapshot_t;

#define YHM2712_ACMD_ATTEMPT_DIAG_CAPACITY  10U

typedef enum
{
    YHM2712_ACMD_ATTEMPT_RESULT_SUCCESS = 0,
    YHM2712_ACMD_ATTEMPT_RESULT_WRITE_FAIL,
    YHM2712_ACMD_ATTEMPT_RESULT_READ_FAIL,
    YHM2712_ACMD_ATTEMPT_RESULT_READBACK_MISMATCH
} yhm2712_acmd_attempt_result_t;

typedef struct
{
    uint8_t reg;
    uint8_t attempt;
    uint8_t expected;
    uint8_t readback;
    bool write_performed;
    bool read_performed;
    bool readback_valid;
    bool readback_match;
    bool failure_valid;
    bool ack_diag_valid;
    yhm2712_acmd_attempt_result_t result;
    yhm2712_acmd_status_t write_status;
    yhm2712_acmd_status_t read_status;
    yhm2712_acmd_stage_t failure_stage;
    yhm2712_acmd_ack_path_t ack_path;
    uint8_t expected_symbol;
    uint8_t actual_symbol;
    uint8_t missed_fall;
    uint32_t high_wait;
    uint32_t fall_wait;
    uint32_t ack_low_ticks;
    uint32_t turnaround_tail_ticks;
    yhm2712_acmd_stage_t ack_diag_stage;
    yhm2712_acmd_status_t ack_diag_status;
    uint32_t ack_low_poll_count;
    uint32_t ack_max_tick_step;
    uint32_t ack_low_start_tick;
    uint32_t ack_rise_tick;
    uint8_t ack_primask;
    uint8_t ack_dir_m;
    uint8_t ack_datasrc_m;
    uint8_t ack_debounce_m;
    uint8_t ack_lssync_m;
} yhm2712_acmd_attempt_diag_t;

typedef struct
{
    uint8_t count;
    yhm2712_acmd_attempt_diag_t
        entry[YHM2712_ACMD_ATTEMPT_DIAG_CAPACITY];
} yhm2712_acmd_attempt_trace_t;

typedef struct
{
    bool valid;
    bool status_input_restored;
    bool primask_restored;
    bool busy_cleared;
    uint32_t primask_before;
    uint32_t primask_after;
    uint32_t gpio_datadir;
    uint32_t gpio_datasrc;
    uint8_t line_level;
} yhm2712_acmd_recovery_snapshot_t;

typedef struct
{
    uint8_t reg;
    yhm2712_acmd_status_t status;
    uint8_t value;
    bool valid;
} yhm2712_acmd_boot_reg_diag_t;

typedef struct
{
    yhm2712_acmd_status_t result;
    yhm2712_acmd_status_t selftest_status;
    yhm2712_acmd_status_t id_status;
    yhm2712_acmd_status_t status1_status;
    yhm2712_acmd_status_t status2_status;
    yhm2712_acmd_status_t window_status;
    uint8_t stacmd_level_before;
    uint8_t stacmd_level_after_restore;
    uint8_t id;
    uint8_t id_attempts;
    uint8_t id_stable_count;
    uint8_t status1;
    uint8_t status2;
    uint8_t fsm_mode;
    uint8_t window_count;
    bool selftest_ok;
    bool id_valid;
    bool id_ok;
    bool status1_valid;
    bool status2_valid;
    bool window_all_valid;
    bool comm_ok;
    yhm2712_acmd_boot_reg_diag_t window[YHM2712_ACMD_BOOT_REG_COUNT];
} yhm2712_acmd_boot_diag_t;

typedef void (*yhm2712_acmd_delay_ms_fn_t)(uint32_t delay_ms);

#if YHM_PRODUCTION_COMPACT
typedef struct
{
    uint32_t transactions;
    uint32_t io_errors;
    uint32_t value_errors;
    uint32_t retries;
    uint32_t id_extra_reads;
} yhm2712_comm_stats_t;
/* Task-context cumulative counters; subtraction is uint32 wrap-safe. */
bool yhm2712_acmd_comm_stats_get(yhm2712_comm_stats_t *stats);
void yhm2712_acmd_value_error(uint8_t reg, uint8_t expected, uint8_t actual,
                            yhm2712_acmd_status_t status);
#endif

bool yhm2712_acmd_init(uint8_t stacmd_pin);
yhm2712_acmd_status_t yhm2712_acmd_select_external_power_profile(
    bool external_power_present);
yhm2712_acmd_status_t yhm2712_acmd_boot_toggle_probe(uint8_t stacmd_pin,
                                                     uint32_t duration_ms,
                                                     uint32_t half_period_ms,
                                                     uint8_t pad_out_enable,
                                                     yhm2712_acmd_delay_ms_fn_t delay_ms);
yhm2712_acmd_status_t yhm2712_acmd_read_reg(uint8_t reg, uint8_t *value);
yhm2712_acmd_status_t yhm2712_acmd_read_reg_retry(uint8_t reg,
                                                   uint8_t *value,
                                                   uint8_t *attempts_out);
yhm2712_acmd_status_t yhm2712_acmd_read_reg_retry_diag(
    uint8_t reg,
    uint8_t *value,
    uint8_t *attempts_out,
    yhm2712_acmd_attempt_trace_t *trace);
yhm2712_acmd_status_t yhm2712_acmd_read_reg_retry_pre_idle_diag(
    uint8_t reg,
    uint8_t *value,
    uint8_t *attempts_out,
    yhm2712_acmd_attempt_trace_t *trace);
yhm2712_acmd_status_t yhm2712_acmd_write_reg(uint8_t reg, uint8_t value);
yhm2712_acmd_status_t yhm2712_acmd_write_reg_edge_timed_retry_no_readback(
    uint8_t reg,
    uint8_t value,
    uint8_t *attempts_out);
yhm2712_acmd_status_t
yhm2712_acmd_write_reg_edge_timed_retry_no_readback_diag(
    uint8_t reg,
    uint8_t value,
    uint8_t *attempts_out,
    yhm2712_acmd_attempt_trace_t *trace);
yhm2712_acmd_status_t yhm2712_acmd_write_reg_retry_readback(uint8_t reg,
                                                            uint8_t value,
                                                            uint8_t *readback,
                                                            uint8_t *attempts_out);
yhm2712_acmd_status_t yhm2712_acmd_write_reg_retry_readback_diag(
    uint8_t reg,
    uint8_t value,
    uint8_t *readback,
    uint8_t *attempts_out,
    yhm2712_acmd_attempt_trace_t *trace);
yhm2712_acmd_status_t yhm2712_acmd_write_reg_retry_readback_pre_idle_diag(
    uint8_t reg,
    uint8_t value,
    uint8_t *readback,
    uint8_t *attempts_out,
    yhm2712_acmd_attempt_trace_t *trace);
yhm2712_acmd_status_t yhm2712_acmd_read_fsm_mode(uint8_t *status2, uint8_t *fsm_mode);
yhm2712_acmd_status_t yhm2712_acmd_enter_sleep_mode(void);
yhm2712_acmd_status_t yhm2712_acmd_exit_sleep_to_discharge(void);
yhm2712_acmd_status_t yhm2712_acmd_enter_charge_mode(void);
#if defined(ZY100_BUILD_FACTORY) && (ZY100_BUILD_FACTORY == 1)
yhm2712_acmd_status_t
yhm2712_acmd_factory_arm_battery_handover_with_input(void);
#endif
yhm2712_acmd_status_t yhm2712_acmd_read_boot_diag(yhm2712_acmd_boot_diag_t *diag);
bool yhm2712_acmd_failure_snapshot_get(yhm2712_acmd_failure_snapshot_t *snapshot);
uint32_t yhm2712_acmd_ack_turnaround_tail_count_get(void);
#if defined(YHM2712_ACMD_ACK_PREAMBLE_FILTER_ENABLE) && \
    (YHM2712_ACMD_ACK_PREAMBLE_FILTER_ENABLE == 1U)
uint32_t yhm2712_acmd_ack_preamble_filter_count_get(void);
#endif
bool yhm2712_acmd_converge_status_input(
    yhm2712_acmd_recovery_snapshot_t *snapshot);
bool yhm2712_acmd_recovery_snapshot_get(
    yhm2712_acmd_recovery_snapshot_t *snapshot);
const char *yhm2712_acmd_status_name(yhm2712_acmd_status_t status);
const char *yhm2712_acmd_stage_name(yhm2712_acmd_stage_t stage);
const char *yhm2712_acmd_ack_path_name(yhm2712_acmd_ack_path_t path);
const char *yhm2712_acmd_fsm_mode_name(uint8_t fsm_mode);

/* Read-only transport diagnostics (MOS capture only). Timestamps use the existing 26-bit
 * 80 MHz counter; only bit_count pairs are complete. No logging in capture. */
typedef struct
{
    bool valid;
    uint32_t sequence;
    uint8_t reg;
    uint8_t value;
    uint8_t bit_count;
    uint8_t entry_low_bit; /* 0..7, or 0xff when no missed entry */
    yhm2712_acmd_status_t status; /* complete register transaction */
    yhm2712_acmd_status_t capture_status;
    uint32_t fall[8];
    uint32_t rise[8];
    uint32_t wait_start;
    uint32_t timeout_tick;
} yhm_pressure_rx_snapshot_t;

/* false selects latest read; true selects first bad ID in current preflight.
 * Call from the owning test task, outside an active transaction. */
bool yhm_pressure_rx_snapshot_get(bool first_id_error,
                                  yhm_pressure_rx_snapshot_t *snapshot);

#if defined(ZY100_BUILD_YHM_PRESSURE_TEST) && \
    (ZY100_BUILD_YHM_PRESSURE_TEST == 1) && ZY100_YHM_HAS_MOS
typedef enum
{
    YHM_PRESSURE_DATA_PROFILE_FIXED = 0,
    YHM_PRESSURE_DATA_PROFILE_COUNT
} yhm_pressure_data_profile_t;

/* Select the single test-build ACK/DATA pair between transactions. */
yhm2712_acmd_status_t yhm_pressure_data_profile_select(
    yhm_pressure_data_profile_t profile);
uint32_t yhm_pressure_data_threshold_ticks_get(void);
uint32_t yhm_pressure_ack_threshold_ticks_get(void);
#define YHM_PRESSURE_DIAG_ENABLE 1
#else
#define YHM_PRESSURE_DIAG_ENABLE 0
#endif

typedef struct
{
    bool valid;
    uint8_t reg;
    yhm2712_acmd_stage_t stage;
    yhm2712_acmd_status_t status;
    uint32_t low_ticks;
    uint32_t threshold_ticks;
    uint8_t expected;
    uint8_t symbol;
} yhm_pressure_ack_sample_t;

bool yhm_pressure_ack_sample_get(yhm_pressure_ack_sample_t *sample);

#if ZY100_YHM_HAS_MOS
/* Preserve the board-validated 10016 ACK work layout in every MOS target. */
typedef struct
{
    bool valid;
    bool is_write;
    bool value_mismatch;
    uint8_t write_value;
    uint8_t tail_high;
    uint32_t data_threshold;
    yhm2712_acmd_status_t status;
    yhm_pressure_rx_snapshot_t rx;
    yhm_pressure_ack_sample_t ack;
    uint32_t ack_start;
    uint32_t ack_fall;
    uint32_t ack_rise; /* fall + measured width, not an extra timer read */
    uint8_t ack_entry_low;
} yhm_pressure_diag_t;

/* Latest ACK timestamps only; call between transactions. Other fields are not
 * exported here. This accessor also keeps shared ACK stores observable. */
bool yhm_mos_ack_timing_get(yhm_pressure_diag_t *out);
/* Task-context diagnostics; counters include errors recovered by retries. */
void yhm_mos_error_counters_log(const char *reason);
void yhm_mos_report_value_mismatch(uint8_t reg, uint8_t expected, uint8_t actual);

#endif

#if YHM_PRESSURE_DIAG_ENABLE
typedef enum
{
    YHM_PRESSURE_DIAG_LAST = 0,
    YHM_PRESSURE_DIAG_FIRST_ERROR,
    YHM_PRESSURE_DIAG_REFERENCE_0,
    YHM_PRESSURE_DIAG_REFERENCE_1,
    YHM_PRESSURE_DIAG_COUNT
} yhm_pressure_diag_slot_t;

/* All copies/reset/export run outside the transaction. Slots are per attempt. */
bool yhm_pressure_diag_reset(void);
bool yhm_pressure_diag_get(yhm_pressure_diag_slot_t slot, yhm_pressure_diag_t *out);
void yhm_pressure_diag_freeze(void);
void yhm_pressure_diag_freeze_sample(const yhm_pressure_diag_t *sample);
void yhm_pressure_diag_forget_last(void);
#endif

#if defined(ZY100_BUILD_FACTORY) && ZY100_BUILD_FACTORY
typedef struct
{
    bool valid;
    uint8_t attempts;
    uint8_t stable_count;
    uint8_t failure_count;
    uint8_t values[YHM2712_ACMD_ID_MAX_ATTEMPTS];
    yhm2712_acmd_status_t status[YHM2712_ACMD_ID_MAX_ATTEMPTS];
} yhm2712_acmd_id_check_snapshot_t;

/* Latest bounded ID check; call after communication has returned.
 * values[i] is meaningful only when status[i] is OK. */
bool yhm2712_acmd_id_check_snapshot_get(yhm2712_acmd_id_check_snapshot_t *snapshot);
#endif

#ifdef __cplusplus
}
#endif

#endif /* YHM2712_ACMD_H */
