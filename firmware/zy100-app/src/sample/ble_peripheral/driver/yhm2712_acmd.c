/**
*****************************************************************************************
*     ZY100 YHM2712 Access Command driver
*****************************************************************************************
*/

#include "driver/yhm2712_acmd.h"
#include "version.h"

#define YHM_BUILD_STRING_INNER(value) #value
#define YHM_BUILD_STRING(value) YHM_BUILD_STRING_INNER(value)

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "app_section.h"
#include "rtl876x.h"
#include "rtl876x_gpio.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"
#include "trace.h"

#if YHM_PRODUCTION_COMPACT
#define YHM_DETAIL_LOG(...) do { if (0) { DBG_DIRECT(__VA_ARGS__); } } while (0)
static void yhm_mos_preflight_error_log(uint8_t reg, yhm2712_acmd_status_t status);
#else
#define YHM_DETAIL_LOG(...) DBG_DIRECT(__VA_ARGS__)
#endif

#ifndef YHM2712_ACMD_ACK_PARITY_INVERT
#define YHM2712_ACMD_ACK_PARITY_INVERT      0U
#endif

#ifndef YHM2712_ACMD_TRACE_ENABLE
#define YHM2712_ACMD_TRACE_ENABLE           0U
#endif

#ifndef YHM2712_ACMD_LAST_BIT_TAIL_INTERVAL
#define YHM2712_ACMD_LAST_BIT_TAIL_INTERVAL 0U
#endif

#ifndef YHM2712_ACMD_PAD_OUT_ENABLE_FOR_GPIO
#define YHM2712_ACMD_PAD_OUT_ENABLE_FOR_GPIO 0U
#endif

#ifndef YHM2712_ACMD_VENDOR_COMPAT_ENABLE
#define YHM2712_ACMD_VENDOR_COMPAT_ENABLE   1U
#endif

#ifndef YHM2712_ACMD_VENDOR_DRIVE_HIGH_ENABLE
#define YHM2712_ACMD_VENDOR_DRIVE_HIGH_ENABLE 1U
#endif

#ifndef YHM2712_ACMD_VENDOR_PULLUP_ENABLE
#define YHM2712_ACMD_VENDOR_PULLUP_ENABLE   1U
#endif

#ifndef YHM2712_ACMD_VENDOR_QUICK_MODE
#define YHM2712_ACMD_VENDOR_QUICK_MODE      1U
#endif

#ifndef YHM2712_ACMD_TBIT_NS
#define YHM2712_ACMD_TBIT_NS                300U
#endif

#define YHM2712_ACMD_VENDOR_QUICK_ADDRESS   0x44U
#define YHM2712_ACMD_VENDOR_TICK_REG        0x4005817CUL
#define YHM2712_ACMD_VENDOR_TICK_MASK       0x03FFFFFFUL
#define YHM2712_ACMD_VENDOR_TICK_PER_US     80U
#define YHM2712_ACMD_TICK_PROBE_WAIT        200U
#define YHM2712_ACMD_LOGIC0_TBIT            8U
#define YHM2712_ACMD_LOGIC1_TBIT            26U
#if YHM2712_ACMD_VENDOR_COMPAT_ENABLE
#define YHM2712_ACMD_LOGICZ_TBIT            90U
#else
#define YHM2712_ACMD_LOGICZ_TBIT            60U
#endif
#define YHM2712_ACMD_SAMPLE_TBIT            24U
#define YHM2712_ACMD_VENDOR_HIGH_TBIT       8U
#define YHM2712_ACMD_INTERVAL_TBIT          1U
#define YHM2712_ACMD_EDGE_TIMEOUT_TBIT      180U
#define YHM2712_ACMD_SELFTEST_LOW_US        1U
#define YHM2712_ACMD_VENDOR_SELFTEST_IDLE_US 50U
#define YHM2712_ACMD_IDLE_AFTER_SELFTEST_US 100U
#define YHM2712_ACMD_ID_RETRY_IDLE_US       100U
#define YHM2712_ACMD_WRITE_RETRY_MAX        10U
#define YHM2712_ACMD_WRITE_RETRY_IDLE_US    100U
#define YHM2712_ACMD_READ_RETRY_MAX         YHM2712_ACMD_WRITE_RETRY_MAX
#define YHM2712_ACMD_READ_RETRY_IDLE_US     YHM2712_ACMD_WRITE_RETRY_IDLE_US
#define YHM2712_ACMD_VENDOR_WRITE_ATTEMPTS  9U
#define YHM2712_ACMD_SHIPPING_PRE_IDLE_US   0U
#ifndef YHM2712_ACMD_TX_HIGH_REASSERT_COUNT
#define YHM2712_ACMD_TX_HIGH_REASSERT_COUNT 1U
#endif
#if (YHM2712_ACMD_TX_HIGH_REASSERT_COUNT != 1U) && \
    (YHM2712_ACMD_TX_HIGH_REASSERT_COUNT != 2U) && \
    (YHM2712_ACMD_TX_HIGH_REASSERT_COUNT != 3U)
#error "YHM2712_ACMD_TX_HIGH_REASSERT_COUNT must be 1, 2 or 3"
#endif
#ifndef YHM2712_ACMD_TX_CLOSE_DATA_LOW_PULSE
#define YHM2712_ACMD_TX_CLOSE_DATA_LOW_PULSE 0U
#endif
#if (YHM2712_ACMD_TX_CLOSE_DATA_LOW_PULSE != 0U) && \
    (YHM2712_ACMD_TX_CLOSE_DATA_LOW_PULSE != 1U)
#error "YHM2712_ACMD_TX_CLOSE_DATA_LOW_PULSE must be 0 or 1"
#endif
#ifndef YHM2712_ACMD_ACK_SINGLE_RELEASE_HANDOFF
#define YHM2712_ACMD_ACK_SINGLE_RELEASE_HANDOFF 0U
#endif
#if (YHM2712_ACMD_ACK_SINGLE_RELEASE_HANDOFF != 0U) && \
    (YHM2712_ACMD_ACK_SINGLE_RELEASE_HANDOFF != 1U)
#error "YHM2712_ACMD_ACK_SINGLE_RELEASE_HANDOFF must be 0 or 1"
#endif
#if (YHM2712_ACMD_ACK_SINGLE_RELEASE_HANDOFF == 1U) && \
    ((YHM2712_ACMD_VENDOR_COMPAT_ENABLE == 0U) || \
     (YHM2712_ACMD_VENDOR_DRIVE_HIGH_ENABLE == 0U))
#error "Single-release ACK handoff requires vendor push-pull high"
#endif
#ifndef YHM2712_ACMD_FACTORY_DETERMINISTIC_BYTE_TX
#define YHM2712_ACMD_FACTORY_DETERMINISTIC_BYTE_TX 0U
#endif
#if (YHM2712_ACMD_FACTORY_DETERMINISTIC_BYTE_TX != 0U) && \
    (YHM2712_ACMD_FACTORY_DETERMINISTIC_BYTE_TX != 1U)
#error "YHM2712_ACMD_FACTORY_DETERMINISTIC_BYTE_TX must be 0 or 1"
#endif
#if (YHM2712_ACMD_FACTORY_DETERMINISTIC_BYTE_TX == 1U) && \
    ((YHM2712_ACMD_VENDOR_COMPAT_ENABLE == 0U) || \
     (YHM2712_ACMD_VENDOR_DRIVE_HIGH_ENABLE == 0U) || \
     (YHM2712_ACMD_TX_HIGH_REASSERT_COUNT < 2U) || \
     (YHM2712_ACMD_TX_CLOSE_DATA_LOW_PULSE != 1U) || \
     (YHM2712_ACMD_ACK_SINGLE_RELEASE_HANDOFF != 1U))
#error "Deterministic byte TX requires Factory push-pull, closed-pulse and single-release ACK handoff"
#endif
#ifndef YHM2712_ACMD_FACTORY_BOUNDARY_DIAG_ENABLE
#define YHM2712_ACMD_FACTORY_BOUNDARY_DIAG_ENABLE 0U
#endif
#if (YHM2712_ACMD_FACTORY_BOUNDARY_DIAG_ENABLE != 0U) && \
    (YHM2712_ACMD_FACTORY_BOUNDARY_DIAG_ENABLE != 1U)
#error "YHM2712_ACMD_FACTORY_BOUNDARY_DIAG_ENABLE must be 0 or 1"
#endif
#ifndef YHM2712_ACMD_FACTORY_ACK_POLL_DIAG_ENABLE
#define YHM2712_ACMD_FACTORY_ACK_POLL_DIAG_ENABLE 0U
#endif
#if (YHM2712_ACMD_FACTORY_ACK_POLL_DIAG_ENABLE != 0U) && \
    (YHM2712_ACMD_FACTORY_ACK_POLL_DIAG_ENABLE != 1U)
#error "YHM2712_ACMD_FACTORY_ACK_POLL_DIAG_ENABLE must be 0 or 1"
#endif
#if (YHM2712_ACMD_FACTORY_ACK_POLL_DIAG_ENABLE == 1U) && \
    (YHM2712_ACMD_ACK_SINGLE_RELEASE_HANDOFF != 1U)
#error "Factory ACK poll diagnostics require single-release ACK handoff"
#endif
#ifndef YHM2712_ACMD_FACTORY_RAM_TIMING_ENABLE
#define YHM2712_ACMD_FACTORY_RAM_TIMING_ENABLE 0U
#endif
#if (YHM2712_ACMD_FACTORY_RAM_TIMING_ENABLE != 0U) && \
    (YHM2712_ACMD_FACTORY_RAM_TIMING_ENABLE != 1U)
#error "YHM2712_ACMD_FACTORY_RAM_TIMING_ENABLE must be 0 or 1"
#endif
#if (YHM2712_ACMD_FACTORY_RAM_TIMING_ENABLE == 1U) && \
    ((YHM2712_ACMD_FACTORY_DETERMINISTIC_BYTE_TX != 1U) || \
     (YHM2712_ACMD_ACK_SINGLE_RELEASE_HANDOFF != 1U))
#error "Factory RAM timing requires deterministic byte TX and single-release ACK handoff"
#endif
#ifndef YHM2712_ACMD_TX_VENDOR_NOP_ENABLE
#define YHM2712_ACMD_TX_VENDOR_NOP_ENABLE  0U
#endif
#if (YHM2712_ACMD_TX_VENDOR_NOP_ENABLE != 0U) && \
    (YHM2712_ACMD_TX_VENDOR_NOP_ENABLE != 1U)
#error "YHM2712_ACMD_TX_VENDOR_NOP_ENABLE must be 0 or 1"
#endif
#ifndef YHM2712_ACMD_TX_VENDOR_NOP_CPU_HZ
#define YHM2712_ACMD_TX_VENDOR_NOP_CPU_HZ  0U
#endif
#ifndef YHM2712_ACMD_RX_VENDOR_SAMPLE_ENABLE
#define YHM2712_ACMD_RX_VENDOR_SAMPLE_ENABLE 0U
#endif
#if (YHM2712_ACMD_RX_VENDOR_SAMPLE_ENABLE != 0U) && \
    (YHM2712_ACMD_RX_VENDOR_SAMPLE_ENABLE != 1U)
#error "YHM2712_ACMD_RX_VENDOR_SAMPLE_ENABLE must be 0 or 1"
#endif
#if (YHM2712_ACMD_TX_VENDOR_NOP_ENABLE == 1U) && \
    (YHM2712_ACMD_TX_VENDOR_NOP_CPU_HZ != 80000000U)
#error "Vendor NOP timing is calibrated only for the gated 80 MHz diagnostic target"
#endif
#if (YHM2712_ACMD_RX_VENDOR_SAMPLE_ENABLE == 1U) && \
    (YHM2712_ACMD_TX_VENDOR_NOP_ENABLE != 1U)
#error "Vendor sampled RX requires the target-specific vendor NOP timing unit"
#endif
#ifndef YHM2712_ACMD_ACK_PREAMBLE_FILTER_ENABLE
#define YHM2712_ACMD_ACK_PREAMBLE_FILTER_ENABLE 0U
#endif
#if (YHM2712_ACMD_ACK_PREAMBLE_FILTER_ENABLE != 0U) && \
    (YHM2712_ACMD_ACK_PREAMBLE_FILTER_ENABLE != 1U)
#error "YHM2712_ACMD_ACK_PREAMBLE_FILTER_ENABLE must be 0 or 1"
#endif
#if (YHM2712_ACMD_ACK_PREAMBLE_FILTER_ENABLE == 1U) && \
    ((YHM2712_ACMD_ACK_SINGLE_RELEASE_HANDOFF != 1U) || \
     (YHM2712_ACMD_RX_VENDOR_SAMPLE_ENABLE != 0U))
#error "ACK preamble filtering requires width decoding and single-release handoff"
#endif
#if YHM2712_ACMD_TX_VENDOR_NOP_ENABLE == 1U
/*
 * Third-party YHM2710 source uses a 15-NOP inner delay loop and tunes the
 * outer count for the active MCU/compiler.  Keep the same structure, but do
 * not copy its board-specific 1/9/31 counts.  For Cortex-M4 at 80 MHz, one
 * taken loop is modelled as 15 NOP + SUBS + taken BNE = 19 core cycles; the
 * final loop is two cycles shorter.  The rounded initial counts below are a
 * scope-calibration starting point.  They are enabled only by the isolated
 * YHM pressure-test target and must be validated on P0_2 before promotion.
 */
#define YHM2712_ACMD_VENDOR_NOP_PER_LOOP       15U
#define YHM2712_ACMD_VENDOR_NOP_LOOP_CYCLES    19U
#define YHM2712_ACMD_VENDOR_NOP_ROUND_CYCLES   11U
#define YHM2712_ACMD_VENDOR_NOP_CYCLES_TBIT \
    (((YHM2712_ACMD_TX_VENDOR_NOP_CPU_HZ / 1000000U) * \
      YHM2712_ACMD_TBIT_NS + 999U) / 1000U)
#define YHM2712_ACMD_VENDOR_NOP_LOOPS(tbit_count) \
    (((YHM2712_ACMD_VENDOR_NOP_CYCLES_TBIT * (tbit_count)) + \
      YHM2712_ACMD_VENDOR_NOP_ROUND_CYCLES) / \
     YHM2712_ACMD_VENDOR_NOP_LOOP_CYCLES)
#define YHM2712_ACMD_VENDOR_NOP_B0_LOOPS \
    YHM2712_ACMD_VENDOR_NOP_LOOPS(YHM2712_ACMD_LOGIC0_TBIT)
#define YHM2712_ACMD_VENDOR_NOP_B1_LOOPS \
    YHM2712_ACMD_VENDOR_NOP_LOOPS(YHM2712_ACMD_LOGIC1_TBIT)
#define YHM2712_ACMD_VENDOR_NOP_BZ_LOOPS \
    YHM2712_ACMD_VENDOR_NOP_LOOPS(YHM2712_ACMD_LOGICZ_TBIT)
#define YHM2712_ACMD_VENDOR_SAMPLE_TBIT      16U
#define YHM2712_ACMD_VENDOR_NOP_SA_LOOPS \
    YHM2712_ACMD_VENDOR_NOP_LOOPS(YHM2712_ACMD_VENDOR_SAMPLE_TBIT)
#if (YHM2712_ACMD_VENDOR_NOP_B0_LOOPS != 10U) || \
    (YHM2712_ACMD_VENDOR_NOP_B1_LOOPS != 33U) || \
    (YHM2712_ACMD_VENDOR_NOP_BZ_LOOPS != 114U) || \
    (YHM2712_ACMD_VENDOR_NOP_SA_LOOPS != 20U)
#error "Unexpected 80 MHz vendor NOP loop calibration"
#endif
#endif
#define YHM2712_ACMD_SYMBOL_UNKNOWN         0xFFU
#define YHM2712_ACMD_DATA_BITS              8U
#define YHM2712_ACMD_READ_WIDTH_THRESHOLD_TBIT 22U
#define YHM_PRESSURE_DATA_THRESHOLD_TBIT 27U
#define YHM_PRESSURE_DATA_PROFILE_FIXED_TICKS ZY100_YHM_DATA_THRESHOLD_TICKS

#ifndef YHM2712_ACMD_FORCE_INLINE
#if defined(__CC_ARM)
#define YHM2712_ACMD_FORCE_INLINE           __forceinline static
#elif defined(__GNUC__)
#define YHM2712_ACMD_FORCE_INLINE           __attribute__((always_inline)) static inline
#else
#define YHM2712_ACMD_FORCE_INLINE           static inline
#endif
#endif

typedef enum
{
    YHM2712_ACMD_SELFTEST_PHASE_RELEASE_INITIAL = 0,
    YHM2712_ACMD_SELFTEST_PHASE_DRIVE_LOW,
    YHM2712_ACMD_SELFTEST_PHASE_RELEASE_AFTER_DRIVE,
    YHM2712_ACMD_SELFTEST_PHASE_COUNT
} yhm2712_acmd_selftest_phase_t;

typedef struct
{
    uint32_t low_cycles;
    uint8_t symbol;
    uint8_t entry_line_level;
    uint8_t missed_fall;
} yhm2712_acmd_symbol_sample_t;

typedef struct
{
    uint32_t mask;
    uint32_t datadir;
    uint32_t datain;
    uint32_t dataout;
    uint32_t datasrc;
    uint8_t dir_m;
    uint8_t in_m;
    uint8_t out_m;
} yhm2712_acmd_gpio_sample_t;

typedef struct
{
    uint32_t dataout_low;
    uint32_t dataout_high;
    uint32_t datasrc_low;
    uint32_t datadir_drive_low;
    uint32_t datadir_drive_high;
    uint32_t datadir_release_z;
    uint32_t dataout_restore;
    uint32_t datasrc_restore;
    uint32_t datadir_restore;
} yhm2712_acmd_fast_gpio_t;

typedef struct
{
    bool valid;
    uint8_t reg;
    yhm2712_acmd_stage_t stage;
    yhm2712_acmd_status_t status;
    yhm2712_acmd_symbol_sample_t symbol_sample;
} yhm2712_acmd_fail_trace_t;

#if YHM2712_ACMD_FACTORY_ACK_POLL_DIAG_ENABLE == 1U
typedef struct
{
    bool valid;
    uint32_t low_poll_count;
    uint32_t max_tick_step;
    uint32_t low_start_tick;
    uint32_t rise_tick;
    uint8_t primask;
    uint8_t dir_m;
    uint8_t datasrc_m;
    uint8_t debounce_m;
    uint8_t lssync_m;
} yhm2712_acmd_ack_poll_diag_t;
#endif

typedef struct
{
    bool valid;
    yhm2712_acmd_stage_t stage;
    uint32_t wait_cycles;
    uint8_t line_level;
    yhm2712_acmd_gpio_sample_t gpio;
} yhm2712_acmd_release_trace_t;

typedef struct
{
    bool valid;
    yhm2712_acmd_stage_t stage;
    yhm2712_acmd_ack_path_t path;
    yhm2712_acmd_status_t status;
    uint32_t high_wait;
    uint32_t fall_wait;
    uint32_t low_cycles;
    uint32_t turnaround_tail_cycles;
    uint8_t expected_symbol;
    uint8_t symbol;
    uint8_t dir_m;
    uint8_t in_m;
#if YHM2712_ACMD_FACTORY_ACK_POLL_DIAG_ENABLE == 1U
    yhm2712_acmd_ack_poll_diag_t poll_diag;
#endif
} yhm2712_acmd_ack_trace_t;

typedef struct
{
    bool valid;
    yhm2712_acmd_status_t status;
    uint8_t value;
    uint8_t bit_count;
    uint8_t symbols[YHM2712_ACMD_DATA_BITS];
    uint32_t lows[YHM2712_ACMD_DATA_BITS];
    uint32_t fall_waits[YHM2712_ACMD_DATA_BITS];
} yhm2712_acmd_data_trace_t;

typedef struct
{
    bool valid;
    yhm2712_acmd_gpio_sample_t gpio;
} yhm2712_acmd_cleanup_trace_t;

typedef struct
{
    bool valid;
    yhm2712_acmd_selftest_phase_t phase;
    yhm2712_acmd_status_t status;
    uint32_t wait_cycles;
    yhm2712_acmd_gpio_sample_t gpio;
} yhm2712_acmd_selftest_phase_trace_t;

typedef struct
{
    yhm2712_acmd_status_t result;
    uint8_t pad_out_enable;
    uint8_t level_before_prepare;
    uint8_t level_after_restore;
    bool fallback_allowed;
    yhm2712_acmd_gpio_sample_t before_prepare;
    yhm2712_acmd_gpio_sample_t after_restore;
    yhm2712_acmd_selftest_phase_trace_t phase[YHM2712_ACMD_SELFTEST_PHASE_COUNT];
} yhm2712_acmd_selftest_trace_t;

typedef struct
{
    uint8_t reg;
    uint32_t tick_hz;
    uint32_t tbit_ticks;
    uint32_t logic0_ticks;
    uint32_t logic1_ticks;
    uint32_t logicz_ticks;
    uint32_t sample_ticks;
    uint32_t interval_ticks;
    bool is_write;
    uint8_t write_value;
    uint8_t level_after_restore;
    yhm2712_acmd_fail_trace_t fail_trace;
    yhm2712_acmd_release_trace_t release_trace;
    yhm2712_acmd_ack_trace_t ack_trace;
    yhm2712_acmd_data_trace_t data_trace;
    yhm2712_acmd_cleanup_trace_t cleanup_trace;
} yhm2712_acmd_trace_snapshot_t;

typedef struct
{
    bool inited;
    bool busy;
    uint8_t stacmd_pin;
    uint32_t gpio_pin;
    uint32_t tick_hz;
    uint32_t tbit_ticks;
    uint32_t logic0_ticks;
    uint32_t logic1_ticks;
    uint32_t logicz_ticks;
    uint32_t sample_ticks;
    uint32_t vendor_high_ticks;
    uint32_t interval_ticks;
    uint32_t edge_timeout_ticks;
    uint8_t level_after_restore;
    bool phy_log_reported;
    bool external_power_profile_valid;
    bool tail_drive_high;
    uint32_t ack_turnaround_tail_count;
#if YHM2712_ACMD_ACK_PREAMBLE_FILTER_ENABLE == 1U
    uint32_t ack_preamble_filter_count;
#endif
#if YHM2712_ACMD_FACTORY_BOUNDARY_DIAG_ENABLE == 1U
    bool last_symbol_high_valid;
    uint32_t last_symbol_high_tick;
#endif
    yhm2712_acmd_fast_gpio_t fast_gpio;
    yhm2712_acmd_symbol_sample_t last_symbol;
    yhm2712_acmd_fail_trace_t fail_trace;
    yhm2712_acmd_release_trace_t release_trace;
    yhm2712_acmd_ack_trace_t ack_trace;
    yhm2712_acmd_data_trace_t data_trace;
    yhm2712_acmd_cleanup_trace_t cleanup_trace;
} yhm2712_acmd_ctx_t;

static yhm2712_acmd_ctx_t s_yhm2712_acmd;
#if ZY100_YHM_HAS_MOS
static uint32_t s_yhm_pressure_ack_threshold_ticks = ZY100_YHM_ACK_THRESHOLD_TICKS;
static yhm_pressure_ack_sample_t s_yhm_pressure_last_ack;
/* Preserve 10016 ACK bookkeeping in all MOS builds. */
static yhm_pressure_diag_t s_yhm_pressure_diag_work;
#if defined(ZY100_BUILD_YHM_PRESSURE_TEST) && \
    (ZY100_BUILD_YHM_PRESSURE_TEST == 1)
static uint32_t s_yhm_pressure_data_threshold_ticks =
    YHM_PRESSURE_DATA_PROFILE_FIXED_TICKS;
static yhm_pressure_diag_t s_yhm_pressure_diag[YHM_PRESSURE_DIAG_COUNT];
#endif
#endif
#if !YHM_PRODUCTION_COMPACT
static yhm2712_acmd_failure_snapshot_t s_yhm2712_acmd_last_failure;
#endif
static yhm2712_acmd_recovery_snapshot_t s_yhm2712_acmd_last_recovery;
#if YHM_PRODUCTION_COMPACT
/* Keep the wire context first in the compiler data anchor. */
static yhm2712_comm_stats_t s_yhm_comm_stats;
#endif

static const uint8_t s_yhm2712_acmd_boot_regs[YHM2712_ACMD_BOOT_REG_COUNT] =
{
    YHM2712_REG_ID,
    YHM2712_REG_STATUS1,
    YHM2712_REG_STATUS2,
};

const char *yhm2712_acmd_stage_name(yhm2712_acmd_stage_t stage)
{
    switch (stage)
    {
    case YHM2712_ACMD_STAGE_LINE_IDLE:
        return "line_idle";
    case YHM2712_ACMD_STAGE_ADDR_W_ACK:
        return "addr_w_ack";
    case YHM2712_ACMD_STAGE_REG_ACK:
        return "reg_ack";
    case YHM2712_ACMD_STAGE_ADDR_R_ACK:
        return "addr_r_ack";
    case YHM2712_ACMD_STAGE_DATA_BIT:
        return "data_bit";
    case YHM2712_ACMD_STAGE_DATA_ACK:
        return "data_ack";
    case YHM2712_ACMD_STAGE_MASTER_NACK:
        return "master_nack";
    case YHM2712_ACMD_STAGE_STOP:
        return "stop";
    default:
        return "unknown";
    }
}

const char *yhm2712_acmd_ack_path_name(yhm2712_acmd_ack_path_t path)
{
    switch (path)
    {
    case YHM2712_ACMD_ACK_PATH_NORMAL:
        return "normal";
    case YHM2712_ACMD_ACK_PATH_TURNAROUND_TAIL:
        return "tail_then_ack";
    case YHM2712_ACMD_ACK_PATH_STUCK_LOW:
        return "stuck_low";
    case YHM2712_ACMD_ACK_PATH_NONE:
    default:
        return "none";
    }
}

#if YHM2712_ACMD_TRACE_ENABLE
static const char *yhm2712_acmd_selftest_phase_name(yhm2712_acmd_selftest_phase_t phase)
{
    switch (phase)
    {
    case YHM2712_ACMD_SELFTEST_PHASE_RELEASE_INITIAL:
        return "release_initial";
    case YHM2712_ACMD_SELFTEST_PHASE_DRIVE_LOW:
        return "drive_low";
    case YHM2712_ACMD_SELFTEST_PHASE_RELEASE_AFTER_DRIVE:
        return "release_after_drive";
    default:
        return "unknown";
    }
}
#endif

YHM2712_ACMD_FORCE_INLINE uint32_t yhm2712_acmd_tick_now(void)
{
    return (*(volatile uint32_t *)YHM2712_ACMD_VENDOR_TICK_REG) &
           YHM2712_ACMD_VENDOR_TICK_MASK;
}

YHM2712_ACMD_FORCE_INLINE uint32_t yhm2712_acmd_tick_delta(uint32_t start,
                                                           uint32_t now)
{
    return (now - start) & YHM2712_ACMD_VENDOR_TICK_MASK;
}

static bool yhm2712_acmd_tick_running(void)
{
    uint32_t start = yhm2712_acmd_tick_now();
    uint32_t probe;

    for (probe = 0U; probe < YHM2712_ACMD_TICK_PROBE_WAIT; probe++)
    {
        if (yhm2712_acmd_tick_now() != start)
        {
            return true;
        }
    }

    return false;
}

static uint32_t yhm2712_acmd_ticks_from_tbit(uint32_t tbit_count)
{
    uint64_t ticks;

    ticks = ((uint64_t)YHM2712_ACMD_VENDOR_TICK_PER_US *
             (uint64_t)YHM2712_ACMD_TBIT_NS *
             (uint64_t)tbit_count + 999ULL) / 1000ULL;
    if (ticks == 0ULL)
    {
        ticks = 1ULL;
    }
    if (ticks > 0xFFFFFFFFULL)
    {
        ticks = 0xFFFFFFFFULL;
    }
    return (uint32_t)ticks;
}

static void yhm2712_acmd_refresh_timing(void)
{
    s_yhm2712_acmd.tick_hz = YHM2712_ACMD_VENDOR_TICK_PER_US * 1000000U;
    s_yhm2712_acmd.tbit_ticks = yhm2712_acmd_ticks_from_tbit(1U);
    s_yhm2712_acmd.logic0_ticks = yhm2712_acmd_ticks_from_tbit(YHM2712_ACMD_LOGIC0_TBIT);
    s_yhm2712_acmd.logic1_ticks = yhm2712_acmd_ticks_from_tbit(YHM2712_ACMD_LOGIC1_TBIT);
    s_yhm2712_acmd.logicz_ticks = yhm2712_acmd_ticks_from_tbit(YHM2712_ACMD_LOGICZ_TBIT);
    s_yhm2712_acmd.sample_ticks = yhm2712_acmd_ticks_from_tbit(YHM2712_ACMD_SAMPLE_TBIT);
    s_yhm2712_acmd.vendor_high_ticks =
        yhm2712_acmd_ticks_from_tbit(YHM2712_ACMD_VENDOR_HIGH_TBIT);
    s_yhm2712_acmd.interval_ticks = yhm2712_acmd_ticks_from_tbit(YHM2712_ACMD_INTERVAL_TBIT);
    s_yhm2712_acmd.edge_timeout_ticks =
        yhm2712_acmd_ticks_from_tbit(YHM2712_ACMD_EDGE_TIMEOUT_TBIT);
}

YHM2712_ACMD_FORCE_INLINE void yhm2712_acmd_delay_ticks(uint32_t ticks)
{
    const uint32_t start = yhm2712_acmd_tick_now();

    while (yhm2712_acmd_tick_delta(start, yhm2712_acmd_tick_now()) < ticks)
    {
    }
}

static void yhm2712_acmd_delay_us(uint32_t us)
{
    yhm2712_acmd_delay_ticks(us * YHM2712_ACMD_VENDOR_TICK_PER_US);
}

YHM2712_ACMD_FORCE_INLINE bool yhm2712_acmd_line_is_high(void)
{
    return ((GPIO->DATAIN & s_yhm2712_acmd.gpio_pin) != 0U);
}

static void yhm2712_acmd_capture_gpio(yhm2712_acmd_gpio_sample_t *sample)
{
    const uint32_t mask = s_yhm2712_acmd.gpio_pin;

    if (sample == NULL)
    {
        return;
    }

    sample->mask = mask;
    sample->datadir = GPIO->DATADIR;
    sample->datain = GPIO->DATAIN;
    sample->dataout = GPIO->DATAOUT;
    sample->datasrc = GPIO->DATASRC;
    sample->dir_m = ((sample->datadir & mask) != 0U) ? 1U : 0U;
    sample->in_m = ((sample->datain & mask) != 0U) ? 1U : 0U;
    sample->out_m = ((sample->dataout & mask) != 0U) ? 1U : 0U;
}

static void yhm2712_acmd_trace_clear(void)
{
#if ZY100_YHM_HAS_MOS
    s_yhm_pressure_last_ack.valid = false;
#endif
    s_yhm2712_acmd.fail_trace.valid = false;
    s_yhm2712_acmd.release_trace.valid = false;
    s_yhm2712_acmd.ack_trace.valid = false;
#if YHM2712_ACMD_FACTORY_BOUNDARY_DIAG_ENABLE == 1U
    s_yhm2712_acmd.last_symbol_high_valid = false;
#endif
    s_yhm2712_acmd.data_trace.valid = false;
    s_yhm2712_acmd.cleanup_trace.valid = false;
    s_yhm2712_acmd.level_after_restore = 0U;
    s_yhm2712_acmd.last_symbol.low_cycles = 0U;
    s_yhm2712_acmd.last_symbol.symbol = YHM2712_ACMD_SYMBOL_UNKNOWN;
    s_yhm2712_acmd.last_symbol.entry_line_level = 1U;
    s_yhm2712_acmd.last_symbol.missed_fall = 0U;
}

static void yhm2712_acmd_trace_record_fail(uint8_t reg,
                                           yhm2712_acmd_stage_t stage,
                                           yhm2712_acmd_status_t status)
{
    if (s_yhm2712_acmd.fail_trace.valid)
    {
        return;
    }

    s_yhm2712_acmd.fail_trace.valid = true;
    s_yhm2712_acmd.fail_trace.reg = reg;
    s_yhm2712_acmd.fail_trace.stage = stage;
    s_yhm2712_acmd.fail_trace.status = status;
    s_yhm2712_acmd.fail_trace.symbol_sample = s_yhm2712_acmd.last_symbol;
}

static void yhm2712_acmd_trace_record_release(yhm2712_acmd_stage_t stage,
                                              uint32_t wait_cycles)
{
    s_yhm2712_acmd.release_trace.valid = true;
    s_yhm2712_acmd.release_trace.stage = stage;
    s_yhm2712_acmd.release_trace.wait_cycles = wait_cycles;
    s_yhm2712_acmd.release_trace.line_level =
        ((GPIO->DATAIN & s_yhm2712_acmd.gpio_pin) != 0U) ? 1U : 0U;
    yhm2712_acmd_capture_gpio(&s_yhm2712_acmd.release_trace.gpio);
}

static void yhm2712_acmd_trace_record_ack(yhm2712_acmd_stage_t stage,
                                          yhm2712_acmd_ack_path_t path,
                                          yhm2712_acmd_status_t status,
                                          uint32_t high_wait,
                                          uint32_t fall_wait,
                                          uint32_t low_cycles,
                                          uint32_t turnaround_tail_cycles,
                                          uint8_t expected_symbol,
                                          uint8_t symbol
#if YHM2712_ACMD_FACTORY_ACK_POLL_DIAG_ENABLE == 1U
                                          , const yhm2712_acmd_ack_poll_diag_t *poll_diag
#endif
                                         )
{
    const uint32_t mask = s_yhm2712_acmd.gpio_pin;

    s_yhm2712_acmd.ack_trace.valid = true;
    s_yhm2712_acmd.ack_trace.stage = stage;
    s_yhm2712_acmd.ack_trace.path = path;
    s_yhm2712_acmd.ack_trace.status = status;
    s_yhm2712_acmd.ack_trace.high_wait = high_wait;
    s_yhm2712_acmd.ack_trace.fall_wait = fall_wait;
    s_yhm2712_acmd.ack_trace.low_cycles = low_cycles;
    s_yhm2712_acmd.ack_trace.turnaround_tail_cycles = turnaround_tail_cycles;
    s_yhm2712_acmd.ack_trace.expected_symbol = expected_symbol;
    s_yhm2712_acmd.ack_trace.symbol = symbol;
    s_yhm2712_acmd.ack_trace.dir_m = ((GPIO->DATADIR & mask) != 0U) ? 1U : 0U;
    s_yhm2712_acmd.ack_trace.in_m = ((GPIO->DATAIN & mask) != 0U) ? 1U : 0U;
#if YHM2712_ACMD_FACTORY_ACK_POLL_DIAG_ENABLE == 1U
    if (poll_diag != NULL)
    {
        s_yhm2712_acmd.ack_trace.poll_diag = *poll_diag;
    }
    else
    {
        s_yhm2712_acmd.ack_trace.poll_diag.valid = false;
    }
#endif
}

static void yhm2712_acmd_trace_record_cleanup(void)
{
    s_yhm2712_acmd.cleanup_trace.valid = true;
    yhm2712_acmd_capture_gpio(&s_yhm2712_acmd.cleanup_trace.gpio);
}

static void yhm2712_acmd_recovery_snapshot_clear(void)
{
    s_yhm2712_acmd_last_recovery.valid = false;
}

static void yhm2712_acmd_recovery_snapshot_record(uint32_t primask_before,
                                                   bool status_input_applied)
{
    const uint32_t mask = s_yhm2712_acmd.gpio_pin;
    yhm2712_acmd_recovery_snapshot_t *snapshot =
        &s_yhm2712_acmd_last_recovery;

    snapshot->valid = s_yhm2712_acmd.inited && (mask != 0U);
    snapshot->primask_before = primask_before;
    snapshot->primask_after = __get_PRIMASK();
    snapshot->primask_restored =
        (snapshot->primask_before == snapshot->primask_after);
    snapshot->busy_cleared = !s_yhm2712_acmd.busy;
    snapshot->gpio_datadir = GPIO->DATADIR;
    snapshot->gpio_datasrc = GPIO->DATASRC;
    snapshot->line_level = yhm2712_acmd_line_is_high() ? 1U : 0U;
    snapshot->status_input_restored = status_input_applied &&
                                      ((snapshot->gpio_datadir & mask) == 0U) &&
                                      ((snapshot->gpio_datasrc & mask) == 0U);
}

#if !YHM_PRODUCTION_COMPACT
static void yhm2712_acmd_trace_snapshot(uint8_t reg,
                                        bool is_write,
                                        uint8_t write_value,
                                        yhm2712_acmd_trace_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    snapshot->reg = reg;
    snapshot->tick_hz = s_yhm2712_acmd.tick_hz;
    snapshot->tbit_ticks = s_yhm2712_acmd.tbit_ticks;
    snapshot->logic0_ticks = s_yhm2712_acmd.logic0_ticks;
    snapshot->logic1_ticks = s_yhm2712_acmd.logic1_ticks;
    snapshot->logicz_ticks = s_yhm2712_acmd.logicz_ticks;
    snapshot->sample_ticks = s_yhm2712_acmd.sample_ticks;
    snapshot->interval_ticks = s_yhm2712_acmd.interval_ticks;
    snapshot->is_write = is_write;
    snapshot->write_value = write_value;
    snapshot->level_after_restore = s_yhm2712_acmd.level_after_restore;
    snapshot->fail_trace = s_yhm2712_acmd.fail_trace;
    snapshot->release_trace = s_yhm2712_acmd.release_trace;
    snapshot->ack_trace = s_yhm2712_acmd.ack_trace;
    snapshot->data_trace = s_yhm2712_acmd.data_trace;
    snapshot->cleanup_trace = s_yhm2712_acmd.cleanup_trace;
}
#endif

#if !YHM_PRODUCTION_COMPACT
static void yhm2712_acmd_failure_snapshot_record(
    const yhm2712_acmd_trace_snapshot_t *snapshot,
    yhm2712_acmd_status_t status)
{
    bool ack_matches;

    if ((snapshot == NULL) || (status == YHM2712_ACMD_STATUS_OK))
    {
        return;
    }

    ack_matches = snapshot->ack_trace.valid &&
                  snapshot->fail_trace.valid &&
                  (snapshot->ack_trace.stage == snapshot->fail_trace.stage);
    s_yhm2712_acmd_last_failure.valid = true;
    s_yhm2712_acmd_last_failure.is_write = snapshot->is_write;
    s_yhm2712_acmd_last_failure.reg = snapshot->reg;
    s_yhm2712_acmd_last_failure.stage = snapshot->fail_trace.valid ?
                                           snapshot->fail_trace.stage :
                                           YHM2712_ACMD_STAGE_STOP;
    s_yhm2712_acmd_last_failure.status = status;
    s_yhm2712_acmd_last_failure.ack_path = ack_matches ?
                                              snapshot->ack_trace.path :
                                              YHM2712_ACMD_ACK_PATH_NONE;
    s_yhm2712_acmd_last_failure.high_wait = ack_matches ?
                                                snapshot->ack_trace.high_wait : 0U;
    s_yhm2712_acmd_last_failure.fall_wait = ack_matches ?
                                                snapshot->ack_trace.fall_wait : 0U;
    s_yhm2712_acmd_last_failure.low_cycles = ack_matches ?
                                                 snapshot->ack_trace.low_cycles :
                                                 snapshot->fail_trace.symbol_sample.low_cycles;
    s_yhm2712_acmd_last_failure.turnaround_tail_cycles = ack_matches ?
                                                      snapshot->ack_trace.turnaround_tail_cycles :
                                                      0U;
    s_yhm2712_acmd_last_failure.expected_symbol = ack_matches ?
                                                    snapshot->ack_trace.expected_symbol :
                                                    YHM2712_ACMD_SYMBOL_UNKNOWN;
    s_yhm2712_acmd_last_failure.symbol = ack_matches ?
                                             snapshot->ack_trace.symbol :
                                             snapshot->fail_trace.symbol_sample.symbol;
    s_yhm2712_acmd_last_failure.missed_fall =
        snapshot->fail_trace.symbol_sample.missed_fall;
    s_yhm2712_acmd_last_failure.data_bit_count =
        snapshot->data_trace.valid ? snapshot->data_trace.bit_count : 0U;
    s_yhm2712_acmd_last_failure.tail_drive_high =
        s_yhm2712_acmd.tail_drive_high ? 1U : 0U;
    s_yhm2712_acmd_last_failure.ack_entry_level = ack_matches &&
                                                  (snapshot->ack_trace.turnaround_tail_cycles != 0U) ?
                                                      0U :
                                                      snapshot->fail_trace.symbol_sample.entry_line_level;
}
#endif

static void yhm2712_acmd_attempt_trace_reset(
    yhm2712_acmd_attempt_trace_t *trace)
{
    if (trace != NULL)
    {
        trace->count = 0U;
    }
}

static void yhm2712_acmd_attempt_trace_record(
    yhm2712_acmd_attempt_trace_t *trace,
    uint8_t reg,
    uint8_t attempt,
    uint8_t expected,
    yhm2712_acmd_attempt_result_t result,
    bool write_performed,
    yhm2712_acmd_status_t write_status,
    bool read_performed,
    yhm2712_acmd_status_t read_status,
    bool readback_valid,
    uint8_t readback)
{
#if YHM_PRODUCTION_COMPACT
    (void)trace; (void)reg; (void)attempt; (void)expected; (void)result;
    (void)write_performed; (void)write_status; (void)read_performed;
    (void)read_status; (void)readback_valid; (void)readback;
#else
    yhm2712_acmd_attempt_diag_t *entry;
    yhm2712_acmd_status_t failure_status = YHM2712_ACMD_STATUS_OK;
    bool failure_is_write = false;

    if ((trace == NULL) ||
        (trace->count >= YHM2712_ACMD_ATTEMPT_DIAG_CAPACITY))
    {
        return;
    }

    entry = &trace->entry[trace->count++];
    entry->reg = reg;
    entry->attempt = attempt;
    entry->expected = expected;
    entry->readback = readback;
    entry->write_performed = write_performed;
    entry->read_performed = read_performed;
    entry->readback_valid = readback_valid;
    entry->readback_match = readback_valid && (readback == expected);
    entry->failure_valid = false;
#if YHM2712_ACMD_FACTORY_ACK_POLL_DIAG_ENABLE == 1U
    entry->ack_diag_valid = false;
#endif
    entry->result = result;
    entry->write_status = write_status;
    entry->read_status = read_status;
    entry->failure_stage = YHM2712_ACMD_STAGE_STOP;
    entry->ack_path = YHM2712_ACMD_ACK_PATH_NONE;
    entry->expected_symbol = YHM2712_ACMD_SYMBOL_UNKNOWN;
    entry->actual_symbol = YHM2712_ACMD_SYMBOL_UNKNOWN;
    entry->missed_fall = 0U;
    entry->high_wait = 0U;
    entry->fall_wait = 0U;
    entry->ack_low_ticks = 0U;
    entry->turnaround_tail_ticks = 0U;
#if YHM2712_ACMD_FACTORY_ACK_POLL_DIAG_ENABLE == 1U
    entry->ack_diag_stage = YHM2712_ACMD_STAGE_STOP;
    entry->ack_diag_status = YHM2712_ACMD_STATUS_NOT_INIT;
    entry->ack_low_poll_count = 0U;
    entry->ack_max_tick_step = 0U;
    entry->ack_low_start_tick = 0U;
    entry->ack_rise_tick = 0U;
    entry->ack_primask = 0U;
    entry->ack_dir_m = 0U;
    entry->ack_datasrc_m = 0U;
    entry->ack_debounce_m = 0U;
    entry->ack_lssync_m = 0U;

    if (s_yhm2712_acmd.ack_trace.valid)
    {
        entry->ack_diag_valid = s_yhm2712_acmd.ack_trace.poll_diag.valid;
        entry->ack_diag_stage = s_yhm2712_acmd.ack_trace.stage;
        entry->ack_diag_status = s_yhm2712_acmd.ack_trace.status;
        entry->ack_path = s_yhm2712_acmd.ack_trace.path;
        entry->expected_symbol = s_yhm2712_acmd.ack_trace.expected_symbol;
        entry->actual_symbol = s_yhm2712_acmd.ack_trace.symbol;
        entry->high_wait = s_yhm2712_acmd.ack_trace.high_wait;
        entry->fall_wait = s_yhm2712_acmd.ack_trace.fall_wait;
        entry->ack_low_ticks = s_yhm2712_acmd.ack_trace.low_cycles;
        entry->turnaround_tail_ticks =
            s_yhm2712_acmd.ack_trace.turnaround_tail_cycles;
        entry->ack_low_poll_count =
            s_yhm2712_acmd.ack_trace.poll_diag.low_poll_count;
        entry->ack_max_tick_step =
            s_yhm2712_acmd.ack_trace.poll_diag.max_tick_step;
        entry->ack_low_start_tick =
            s_yhm2712_acmd.ack_trace.poll_diag.low_start_tick;
        entry->ack_rise_tick =
            s_yhm2712_acmd.ack_trace.poll_diag.rise_tick;
        entry->ack_primask = s_yhm2712_acmd.ack_trace.poll_diag.primask;
        entry->ack_dir_m = s_yhm2712_acmd.ack_trace.poll_diag.dir_m;
        entry->ack_datasrc_m =
            s_yhm2712_acmd.ack_trace.poll_diag.datasrc_m;
        entry->ack_debounce_m =
            s_yhm2712_acmd.ack_trace.poll_diag.debounce_m;
        entry->ack_lssync_m = s_yhm2712_acmd.ack_trace.poll_diag.lssync_m;
    }
#endif

    if (result == YHM2712_ACMD_ATTEMPT_RESULT_WRITE_FAIL)
    {
        failure_status = write_status;
        failure_is_write = true;
    }
    else if (result == YHM2712_ACMD_ATTEMPT_RESULT_READ_FAIL)
    {
        failure_status = read_status;
    }

    if ((failure_status == YHM2712_ACMD_STATUS_OK) ||
        !s_yhm2712_acmd_last_failure.valid ||
        (s_yhm2712_acmd_last_failure.reg != reg) ||
        (s_yhm2712_acmd_last_failure.status != failure_status) ||
        (s_yhm2712_acmd_last_failure.is_write != failure_is_write))
    {
        return;
    }

    entry->failure_valid = true;
    entry->failure_stage = s_yhm2712_acmd_last_failure.stage;
    entry->ack_path = s_yhm2712_acmd_last_failure.ack_path;
    entry->expected_symbol = s_yhm2712_acmd_last_failure.expected_symbol;
    entry->actual_symbol = s_yhm2712_acmd_last_failure.symbol;
    entry->missed_fall = s_yhm2712_acmd_last_failure.missed_fall;
    entry->high_wait = s_yhm2712_acmd_last_failure.high_wait;
    entry->fall_wait = s_yhm2712_acmd_last_failure.fall_wait;
    entry->ack_low_ticks = s_yhm2712_acmd_last_failure.low_cycles;
    entry->turnaround_tail_ticks =
        s_yhm2712_acmd_last_failure.turnaround_tail_cycles;
#endif
}

#if !YHM_PRODUCTION_COMPACT
static void yhm2712_acmd_trace_log_snapshot(const yhm2712_acmd_trace_snapshot_t *snapshot)
{
#if YHM2712_ACMD_TRACE_ENABLE
    const char *op;
    yhm2712_acmd_status_t status = YHM2712_ACMD_STATUS_OK;

    if (snapshot == NULL)
    {
        return;
    }

    if (snapshot->fail_trace.valid)
    {
        status = snapshot->fail_trace.status;
    }
    op = snapshot->is_write ? "write" : "read";
    YHM_DETAIL_LOG("[YHM2712][ACMD] %s %s reg=0x%02x value=0x%02x status=%s tick_hz=%u tbit=%u",
               (status == YHM2712_ACMD_STATUS_OK) ? "ok" : "fail",
               op,
               snapshot->reg,
               snapshot->write_value,
               yhm2712_acmd_status_name(status),
               snapshot->tick_hz,
               snapshot->tbit_ticks);
#else
    (void)snapshot;
#endif
}
#endif

YHM2712_ACMD_FORCE_INLINE void yhm2712_acmd_release_fast(void)
{
    GPIO->DATADIR = s_yhm2712_acmd.fast_gpio.datadir_release_z;
}

YHM2712_ACMD_FORCE_INLINE void yhm2712_acmd_drive_low_fast(void)
{
    GPIO->DATAOUT = s_yhm2712_acmd.fast_gpio.dataout_low;
    GPIO->DATASRC = s_yhm2712_acmd.fast_gpio.datasrc_low;
    GPIO->DATADIR = s_yhm2712_acmd.fast_gpio.datadir_drive_low;
}

YHM2712_ACMD_FORCE_INLINE void yhm2712_acmd_drive_high_fast(void)
{
#if YHM2712_ACMD_VENDOR_DRIVE_HIGH_ENABLE
    GPIO->DATAOUT = s_yhm2712_acmd.fast_gpio.dataout_high;
    GPIO->DATASRC = s_yhm2712_acmd.fast_gpio.datasrc_low;
    GPIO->DATADIR = s_yhm2712_acmd.fast_gpio.datadir_drive_high;
#else
    yhm2712_acmd_release_fast();
#endif
}

static void yhm2712_acmd_latch_fast_gpio(void)
{
    const uint32_t mask = s_yhm2712_acmd.gpio_pin;

    s_yhm2712_acmd.fast_gpio.dataout_restore = GPIO->DATAOUT;
    s_yhm2712_acmd.fast_gpio.datasrc_restore = GPIO->DATASRC;
    s_yhm2712_acmd.fast_gpio.datadir_restore = GPIO->DATADIR;
    s_yhm2712_acmd.fast_gpio.dataout_low =
        s_yhm2712_acmd.fast_gpio.dataout_restore & ~mask;
    s_yhm2712_acmd.fast_gpio.dataout_high =
        s_yhm2712_acmd.fast_gpio.dataout_restore | mask;
    s_yhm2712_acmd.fast_gpio.datasrc_low =
        s_yhm2712_acmd.fast_gpio.datasrc_restore & ~mask;
    s_yhm2712_acmd.fast_gpio.datadir_release_z =
        s_yhm2712_acmd.fast_gpio.datadir_restore & ~mask;
    s_yhm2712_acmd.fast_gpio.datadir_drive_low =
        s_yhm2712_acmd.fast_gpio.datadir_release_z | mask;
    s_yhm2712_acmd.fast_gpio.datadir_drive_high =
        s_yhm2712_acmd.fast_gpio.datadir_release_z | mask;

    GPIO->DATADIR = s_yhm2712_acmd.fast_gpio.datadir_release_z;
#if YHM2712_ACMD_VENDOR_DRIVE_HIGH_ENABLE
    GPIO->DATAOUT = s_yhm2712_acmd.fast_gpio.dataout_high;
#else
    GPIO->DATAOUT = s_yhm2712_acmd.fast_gpio.dataout_low;
#endif
    GPIO->DATASRC = s_yhm2712_acmd.fast_gpio.datasrc_low;
    GPIO->DATADIR = s_yhm2712_acmd.fast_gpio.datadir_release_z;
}

static void yhm2712_acmd_restore_fast_gpio_irq_disabled(void)
{
    GPIO->DATADIR = s_yhm2712_acmd.fast_gpio.datadir_release_z;
#if YHM2712_ACMD_VENDOR_DRIVE_HIGH_ENABLE
    GPIO->DATAOUT = s_yhm2712_acmd.fast_gpio.dataout_high;
#else
    GPIO->DATAOUT = s_yhm2712_acmd.fast_gpio.dataout_low;
#endif
    GPIO->DATASRC = s_yhm2712_acmd.fast_gpio.datasrc_low;
    GPIO->DATADIR = s_yhm2712_acmd.fast_gpio.datadir_release_z;
    yhm2712_acmd_trace_record_cleanup();
}

static yhm2712_acmd_status_t yhm2712_acmd_prepare_pin_for_acmd(uint8_t pad_out_enable)
{
    GPIO_InitTypeDef gpio_init;
    const uint32_t mask = s_yhm2712_acmd.gpio_pin;

    if (!s_yhm2712_acmd.inited)
    {
        return YHM2712_ACMD_STATUS_NOT_INIT;
    }
    if (mask == 0U)
    {
        return YHM2712_ACMD_STATUS_BAD_PIN;
    }

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    GPIO->DATADIR = GPIO->DATADIR & ~mask;
#if YHM2712_ACMD_VENDOR_DRIVE_HIGH_ENABLE
    GPIO->DATAOUT = GPIO->DATAOUT | mask;
#else
    GPIO->DATAOUT = GPIO->DATAOUT & ~mask;
#endif
    GPIO->DATASRC = GPIO->DATASRC & ~mask;
    GPIO->DATADIR = GPIO->DATADIR & ~mask;

    Pad_Config(s_yhm2712_acmd.stacmd_pin, PAD_PINMUX_MODE, PAD_IS_PWRON,
               (YHM2712_ACMD_VENDOR_PULLUP_ENABLE != 0U) ? PAD_PULL_UP : PAD_PULL_NONE,
               pad_out_enable ? PAD_OUT_ENABLE : PAD_OUT_DISABLE,
               (YHM2712_ACMD_VENDOR_DRIVE_HIGH_ENABLE != 0U) ? PAD_OUT_HIGH : PAD_OUT_LOW);
    Pinmux_Deinit(s_yhm2712_acmd.stacmd_pin);
    Pinmux_Config(s_yhm2712_acmd.stacmd_pin, DWGPIO);

    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = mask;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_ITCmd = DISABLE;
    gpio_init.GPIO_ControlMode = GPIO_SOFTWARE_MODE;
    GPIO_Init(&gpio_init);

    GPIO->DATADIR = GPIO->DATADIR & ~mask;
    return YHM2712_ACMD_STATUS_OK;
}

static void yhm2712_acmd_restore_status_input(void)
{
    GPIO_InitTypeDef gpio_init;
    const uint32_t mask = s_yhm2712_acmd.gpio_pin;

    if ((!s_yhm2712_acmd.inited) || (mask == 0U))
    {
        return;
    }

    GPIO->DATADIR = GPIO->DATADIR & ~mask;
#if YHM2712_ACMD_VENDOR_DRIVE_HIGH_ENABLE
    GPIO->DATAOUT = GPIO->DATAOUT | mask;
#else
    GPIO->DATAOUT = GPIO->DATAOUT & ~mask;
#endif
    GPIO->DATASRC = GPIO->DATASRC & ~mask;
    GPIO->DATADIR = GPIO->DATADIR & ~mask;

    Pad_Config(s_yhm2712_acmd.stacmd_pin, PAD_PINMUX_MODE, PAD_IS_PWRON,
               (YHM2712_ACMD_VENDOR_PULLUP_ENABLE != 0U) ? PAD_PULL_UP : PAD_PULL_NONE,
               PAD_OUT_DISABLE,
               (YHM2712_ACMD_VENDOR_DRIVE_HIGH_ENABLE != 0U) ? PAD_OUT_HIGH : PAD_OUT_LOW);
    Pinmux_Deinit(s_yhm2712_acmd.stacmd_pin);
    Pinmux_Config(s_yhm2712_acmd.stacmd_pin, DWGPIO);

    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = mask;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_ITCmd = DISABLE;
    gpio_init.GPIO_ControlMode = GPIO_SOFTWARE_MODE;
    GPIO_Init(&gpio_init);
}

#if YHM2712_ACMD_TX_HIGH_REASSERT_COUNT > 1U
YHM2712_ACMD_FORCE_INLINE void yhm2712_acmd_prepare_data_symbol_high_fast(void)
{
    /*
     * Target-specific ACMD handoff experiment: the first call preloads
     * DATAOUT high and changes the pin from input to output.  Following calls
     * reassert high after output ownership is established.  Keep this
     * compile-time-unrolled so no loop is added near the timed low pulse.
     */
    yhm2712_acmd_drive_high_fast();
    yhm2712_acmd_drive_high_fast();
#if YHM2712_ACMD_TX_HIGH_REASSERT_COUNT == 3U
    yhm2712_acmd_drive_high_fast();
#endif
}
#endif

#if YHM2712_ACMD_TX_VENDOR_NOP_ENABLE == 1U
YHM2712_ACMD_FORCE_INLINE void yhm2712_acmd_vendor_nop_delay(uint32_t loop_count)
{
    /*
     * Keep the loop body identical to the third-party 15-NOP delay unit.
     * Keep this as force-inlined C, matching the third-party source structure.
     * The ARMCC disassembly is a release gate: the countdown must stay in a
     * register and no timer/peripheral read may appear in the low interval.
     */
    while (loop_count > 0U)
    {
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
        __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
        loop_count--;
    }
}

YHM2712_ACMD_FORCE_INLINE uint32_t yhm2712_acmd_vendor_nop_symbol_loops(
    uint8_t symbol)
{
    if (symbol == 0U)
    {
        return YHM2712_ACMD_VENDOR_NOP_B0_LOOPS;
    }
    if (symbol == 1U)
    {
        return YHM2712_ACMD_VENDOR_NOP_B1_LOOPS;
    }
    return YHM2712_ACMD_VENDOR_NOP_BZ_LOOPS;
}

YHM2712_ACMD_FORCE_INLINE void yhm2712_acmd_drive_low_width_vendor_nop(
    uint32_t loop_count)
{
    /* DATA source and direction are already stable while the line is high. */
    GPIO->DATAOUT = s_yhm2712_acmd.fast_gpio.dataout_low;
    yhm2712_acmd_vendor_nop_delay(loop_count);
    GPIO->DATAOUT = s_yhm2712_acmd.fast_gpio.dataout_high;
}
#endif

#if YHM2712_ACMD_TX_VENDOR_NOP_ENABLE == 0U
YHM2712_ACMD_FORCE_INLINE void yhm2712_acmd_drive_low_width(
    uint32_t low_ticks,
    bool edge_timed)
{
    if (edge_timed)
    {
        /*
         * DATAOUT is the physical falling edge while the vendor-compatible
         * path is already driving high.  Start the deadline before that write
         * so GPIO register overhead is part of, rather than added after, the
         * requested on-pin low width.
         */
        const uint32_t start = yhm2712_acmd_tick_now();

        GPIO->DATAOUT = s_yhm2712_acmd.fast_gpio.dataout_low;
        GPIO->DATASRC = s_yhm2712_acmd.fast_gpio.datasrc_low;
        GPIO->DATADIR = s_yhm2712_acmd.fast_gpio.datadir_drive_low;
        while (yhm2712_acmd_tick_delta(start, yhm2712_acmd_tick_now()) < low_ticks)
        {
        }
        return;
    }

    yhm2712_acmd_drive_low_fast();
    yhm2712_acmd_delay_ticks(low_ticks);
}
#endif

#if ZY100_YHM_HAS_MOS
#include "yhm2712_transport_10013.inc"
#else
#include "yhm2712_transport_10006.inc"
#endif

#if YHM_PRESSURE_DIAG_ENABLE
bool yhm_pressure_diag_reset(void)
{
    uint32_t i;
    if (s_yhm2712_acmd.busy)
    {
        return false;
    }
    for (i = 0U; i < YHM_PRESSURE_DIAG_COUNT; i++)
    {
        s_yhm_pressure_diag[i].valid = false;
    }
    return true;
}

bool yhm_pressure_diag_get(yhm_pressure_diag_slot_t slot, yhm_pressure_diag_t *out)
{
    if ((out == NULL) || ((uint32_t)slot >= YHM_PRESSURE_DIAG_COUNT) ||
        s_yhm2712_acmd.busy || !s_yhm_pressure_diag[slot].valid)
    {
        return false;
    }
    *out = s_yhm_pressure_diag[slot];
    return true;
}

void yhm_pressure_diag_freeze_sample(const yhm_pressure_diag_t *sample)
{
    if ((sample != NULL) && sample->valid && !s_yhm2712_acmd.busy &&
        !s_yhm_pressure_diag[YHM_PRESSURE_DIAG_FIRST_ERROR].valid &&
        (sample != &s_yhm_pressure_diag[YHM_PRESSURE_DIAG_FIRST_ERROR]))
    {
        s_yhm_pressure_diag[YHM_PRESSURE_DIAG_FIRST_ERROR] = *sample;
        s_yhm_pressure_diag[YHM_PRESSURE_DIAG_FIRST_ERROR].value_mismatch =
            (sample->status == YHM2712_ACMD_STATUS_OK);
    }
}

void yhm_pressure_diag_freeze(void)
{
    yhm_pressure_diag_freeze_sample(&s_yhm_pressure_diag[YHM_PRESSURE_DIAG_LAST]);
}

void yhm_pressure_diag_forget_last(void)
{
    if (!s_yhm2712_acmd.busy)
    {
        s_yhm_pressure_diag[YHM_PRESSURE_DIAG_LAST].valid = false;
    }
}

static void yhm_pressure_diag_begin(void)
{
    s_yhm_pressure_last_ack.valid = false;
    s_yhm_pressure_diag_work.ack_start = 0U;
    s_yhm_pressure_diag_work.ack_fall = 0U;
    s_yhm_pressure_diag_work.ack_rise = 0U;
    s_yhm_pressure_diag_work.ack_entry_low = 0U;
}

/* Called only after GPIO/interrupt restoration and busy release. */
static void yhm_pressure_diag_complete(bool is_write, uint8_t value,
                                      yhm2712_acmd_status_t status)
{
    yhm_pressure_diag_t *work = &s_yhm_pressure_diag_work;
    yhm_pressure_diag_slot_t reference;
    work->valid = true;
    work->is_write = is_write;
    work->value_mismatch = false;
    work->write_value = value;
    work->tail_high = s_yhm2712_acmd.tail_drive_high ? 1U : 0U;
    work->status = status;
    work->data_threshold = s_yhm_pressure_data_threshold_ticks;
    work->rx = s_yhm_pressure_rx;
    work->rx.status = status;
    work->ack = s_yhm_pressure_last_ack;
    s_yhm_pressure_diag[YHM_PRESSURE_DIAG_LAST] = *work;
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        yhm_pressure_diag_freeze();
    }
    else if (!is_write && work->ack.valid && (work->ack.symbol <= 1U) &&
             (work->rx.bit_count == 8U))
    {
        reference = (work->ack.symbol == 0U) ?
            YHM_PRESSURE_DIAG_REFERENCE_0 : YHM_PRESSURE_DIAG_REFERENCE_1;
        if (!s_yhm_pressure_diag[reference].valid)
        {
            s_yhm_pressure_diag[reference] = *work;
        }
    }
}
#endif

#if YHM2712_ACMD_FACTORY_RAM_TIMING_ENABLE == 1U
DATA_RAM_FUNCTION
#endif
static yhm2712_acmd_status_t yhm2712_acmd_read_reg_locked(uint8_t reg, uint8_t *value)
{
#if YHM2712_ACMD_VENDOR_COMPAT_ENABLE && YHM2712_ACMD_VENDOR_QUICK_MODE
    const uint8_t quick_address_reg =
        (uint8_t)((YHM2712_ACMD_VENDOR_QUICK_ADDRESS & 0xF0U) | (reg & 0x0FU));
    const uint8_t quick_read = (uint8_t)((quick_address_reg << 1U) | 1U);
    yhm2712_acmd_status_t status = YHM2712_ACMD_STATUS_OK;

    yhm2712_acmd_vendor_reset_pulse_fast(false);
    if (!yhm2712_acmd_line_is_high())
    {
        s_yhm2712_acmd.last_symbol.low_cycles = 0U;
        s_yhm2712_acmd.last_symbol.symbol = YHM2712_ACMD_SYMBOL_UNKNOWN;
        s_yhm2712_acmd.last_symbol.entry_line_level = 0U;
        s_yhm2712_acmd.last_symbol.missed_fall = 1U;
        yhm2712_acmd_trace_record_fail(reg,
                                       YHM2712_ACMD_STAGE_LINE_IDLE,
                                       YHM2712_ACMD_STATUS_LINE_BUSY_LOW);
        return YHM2712_ACMD_STATUS_LINE_BUSY_LOW;
    }

#if YHM2712_ACMD_FACTORY_RAM_TIMING_ENABLE == 1U
    /* Prepare diagnostics before START/address; never between ACK and bit 7. */
    yhm2712_acmd_data_trace_prepare();
#endif
    yhm2712_acmd_send_symbol(2U);

    status = yhm2712_acmd_send_byte_check_ack(reg, YHM2712_ACMD_STAGE_ADDR_R_ACK, quick_read);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        yhm2712_acmd_trace_record_fail(reg, YHM2712_ACMD_STAGE_ADDR_R_ACK, status);
        goto stop;
    }

    status = yhm2712_acmd_recv_byte_send_ack(value, true);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        yhm2712_acmd_trace_record_fail(reg, YHM2712_ACMD_STAGE_DATA_BIT, status);
    }

stop:
    yhm2712_acmd_send_symbol(2U);
    return status;
#else
    const uint8_t addr_write = (uint8_t)((YHM2712_ACMD_SLAVE_ADDRESS << 1U) | 0U);
    const uint8_t addr_read = (uint8_t)((YHM2712_ACMD_SLAVE_ADDRESS << 1U) | 1U);
    yhm2712_acmd_status_t status = YHM2712_ACMD_STATUS_OK;

    if (!yhm2712_acmd_line_is_high())
    {
        s_yhm2712_acmd.last_symbol.low_cycles = 0U;
        s_yhm2712_acmd.last_symbol.symbol = YHM2712_ACMD_SYMBOL_UNKNOWN;
        s_yhm2712_acmd.last_symbol.entry_line_level = 0U;
        s_yhm2712_acmd.last_symbol.missed_fall = 1U;
        yhm2712_acmd_trace_record_fail(reg,
                                       YHM2712_ACMD_STAGE_LINE_IDLE,
                                       YHM2712_ACMD_STATUS_LINE_BUSY_LOW);
        return YHM2712_ACMD_STATUS_LINE_BUSY_LOW;
    }

    yhm2712_acmd_send_symbol(2U);

    status = yhm2712_acmd_send_byte_check_ack(reg, YHM2712_ACMD_STAGE_ADDR_W_ACK, addr_write);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        yhm2712_acmd_trace_record_fail(reg, YHM2712_ACMD_STAGE_ADDR_W_ACK, status);
        goto stop;
    }
    status = yhm2712_acmd_send_byte_check_ack(reg, YHM2712_ACMD_STAGE_REG_ACK, reg);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        yhm2712_acmd_trace_record_fail(reg, YHM2712_ACMD_STAGE_REG_ACK, status);
        goto stop;
    }

#if YHM2712_ACMD_FACTORY_RAM_TIMING_ENABLE == 1U
    /* Prepare diagnostics before START/address; never between ACK and bit 7. */
    yhm2712_acmd_data_trace_prepare();
#endif
    yhm2712_acmd_send_symbol(2U);

    status = yhm2712_acmd_send_byte_check_ack(reg, YHM2712_ACMD_STAGE_ADDR_R_ACK, addr_read);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        yhm2712_acmd_trace_record_fail(reg, YHM2712_ACMD_STAGE_ADDR_R_ACK, status);
        goto stop;
    }
    status = yhm2712_acmd_recv_byte_send_ack(value, true);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        yhm2712_acmd_trace_record_fail(reg, YHM2712_ACMD_STAGE_DATA_BIT, status);
    }

stop:
    yhm2712_acmd_send_symbol(2U);
    return status;
#endif
}

static yhm2712_acmd_status_t yhm2712_acmd_write_reg_locked_policy(
    uint8_t reg,
    uint8_t value,
    bool force_vendor_handoff_high,
    bool edge_timed,
    bool pre_idle_high)
{
#if YHM2712_ACMD_VENDOR_COMPAT_ENABLE && YHM2712_ACMD_VENDOR_QUICK_MODE
    const uint8_t quick_address_reg =
        (uint8_t)((YHM2712_ACMD_VENDOR_QUICK_ADDRESS & 0xF0U) | (reg & 0x0FU));
    const uint8_t quick_write = (uint8_t)((quick_address_reg << 1U) | 0U);
    yhm2712_acmd_status_t status = YHM2712_ACMD_STATUS_OK;

    if (edge_timed || pre_idle_high)
    {
        status = yhm2712_acmd_wait_released_high(
                     YHM2712_ACMD_SHIPPING_PRE_IDLE_US *
                     YHM2712_ACMD_VENDOR_TICK_PER_US);
        if (status != YHM2712_ACMD_STATUS_OK)
        {
            yhm2712_acmd_trace_record_fail(reg, YHM2712_ACMD_STAGE_LINE_IDLE,
                                           status);
            return status;
        }
    }

    yhm2712_acmd_vendor_reset_pulse_fast(edge_timed);
    if (!yhm2712_acmd_line_is_high())
    {
        s_yhm2712_acmd.last_symbol.low_cycles = 0U;
        s_yhm2712_acmd.last_symbol.symbol = YHM2712_ACMD_SYMBOL_UNKNOWN;
        s_yhm2712_acmd.last_symbol.entry_line_level = 0U;
        s_yhm2712_acmd.last_symbol.missed_fall = 1U;
        yhm2712_acmd_trace_record_fail(reg,
                                       YHM2712_ACMD_STAGE_LINE_IDLE,
                                       YHM2712_ACMD_STATUS_LINE_BUSY_LOW);
        return YHM2712_ACMD_STATUS_LINE_BUSY_LOW;
    }

    yhm2712_acmd_send_symbol_policy(2U, true, false, edge_timed);

    status = yhm2712_acmd_send_byte_check_ack_policy(
                 reg, YHM2712_ACMD_STAGE_ADDR_W_ACK, quick_write,
                 force_vendor_handoff_high, edge_timed);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        yhm2712_acmd_trace_record_fail(reg, YHM2712_ACMD_STAGE_ADDR_W_ACK, status);
        goto stop;
    }

    status = yhm2712_acmd_send_byte_check_ack_policy(
                 reg, YHM2712_ACMD_STAGE_DATA_ACK, value,
                 force_vendor_handoff_high, edge_timed);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        yhm2712_acmd_trace_record_fail(reg, YHM2712_ACMD_STAGE_DATA_ACK, status);
    }

stop:
    yhm2712_acmd_send_symbol_policy(2U, true, false, edge_timed);
    return status;
#else
    const uint8_t addr_write = (uint8_t)((YHM2712_ACMD_SLAVE_ADDRESS << 1U) | 0U);
    yhm2712_acmd_status_t status = YHM2712_ACMD_STATUS_OK;

    if (!yhm2712_acmd_line_is_high())
    {
        s_yhm2712_acmd.last_symbol.low_cycles = 0U;
        s_yhm2712_acmd.last_symbol.symbol = YHM2712_ACMD_SYMBOL_UNKNOWN;
        s_yhm2712_acmd.last_symbol.entry_line_level = 0U;
        s_yhm2712_acmd.last_symbol.missed_fall = 1U;
        yhm2712_acmd_trace_record_fail(reg,
                                       YHM2712_ACMD_STAGE_LINE_IDLE,
                                       YHM2712_ACMD_STATUS_LINE_BUSY_LOW);
        return YHM2712_ACMD_STATUS_LINE_BUSY_LOW;
    }

    yhm2712_acmd_send_symbol(2U);

    status = yhm2712_acmd_send_byte_check_ack_policy(
                 reg, YHM2712_ACMD_STAGE_ADDR_W_ACK, addr_write,
                 force_vendor_handoff_high, edge_timed);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        yhm2712_acmd_trace_record_fail(reg, YHM2712_ACMD_STAGE_ADDR_W_ACK, status);
        goto stop;
    }
    status = yhm2712_acmd_send_byte_check_ack_policy(
                 reg, YHM2712_ACMD_STAGE_REG_ACK, reg,
                 force_vendor_handoff_high, edge_timed);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        yhm2712_acmd_trace_record_fail(reg, YHM2712_ACMD_STAGE_REG_ACK, status);
        goto stop;
    }
    status = yhm2712_acmd_send_byte_check_ack_policy(
                 reg, YHM2712_ACMD_STAGE_DATA_ACK, value,
                 force_vendor_handoff_high, edge_timed);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        yhm2712_acmd_trace_record_fail(reg, YHM2712_ACMD_STAGE_DATA_ACK, status);
    }

stop:
    yhm2712_acmd_send_symbol_policy(2U, true, false, edge_timed);
    return status;
#endif
}

static void yhm2712_acmd_selftest_phase_init(yhm2712_acmd_selftest_trace_t *trace)
{
    uint8_t phase;

    if (trace == NULL)
    {
        return;
    }

    for (phase = 0U; phase < (uint8_t)YHM2712_ACMD_SELFTEST_PHASE_COUNT; phase++)
    {
        trace->phase[phase].valid = true;
        trace->phase[phase].phase = (yhm2712_acmd_selftest_phase_t)phase;
        trace->phase[phase].status = YHM2712_ACMD_STATUS_NOT_INIT;
        trace->phase[phase].wait_cycles = 0U;
        yhm2712_acmd_capture_gpio(&trace->phase[phase].gpio);
    }
}

static void yhm2712_acmd_selftest_record_phase(yhm2712_acmd_selftest_trace_t *trace,
                                               yhm2712_acmd_selftest_phase_t phase,
                                               yhm2712_acmd_status_t status,
                                               uint32_t wait_cycles)
{
    if ((trace == NULL) || (phase >= YHM2712_ACMD_SELFTEST_PHASE_COUNT))
    {
        return;
    }

    trace->phase[phase].valid = true;
    trace->phase[phase].phase = phase;
    trace->phase[phase].status = status;
    trace->phase[phase].wait_cycles = wait_cycles;
    yhm2712_acmd_capture_gpio(&trace->phase[phase].gpio);
}

static yhm2712_acmd_status_t yhm2712_acmd_wait_selftest_high(uint32_t *wait_ticks)
{
    const uint32_t start = yhm2712_acmd_tick_now();
    uint32_t elapsed;

    if (wait_ticks == NULL)
    {
        return YHM2712_ACMD_STATUS_BAD_PARAM;
    }

    while (!yhm2712_acmd_line_is_high())
    {
        elapsed = yhm2712_acmd_tick_delta(start, yhm2712_acmd_tick_now());
        if (elapsed > s_yhm2712_acmd.edge_timeout_ticks)
        {
            *wait_ticks = elapsed;
            return YHM2712_ACMD_STATUS_RELEASE_TIMEOUT;
        }
    }

    *wait_ticks = yhm2712_acmd_tick_delta(start, yhm2712_acmd_tick_now());
    return YHM2712_ACMD_STATUS_OK;
}

static yhm2712_acmd_status_t yhm2712_acmd_release_selftest_once(
    uint8_t pad_out_enable,
    yhm2712_acmd_selftest_trace_t *trace)
{
    yhm2712_acmd_status_t status;
    uint32_t wait_ticks = 0U;

    if (trace == NULL)
    {
        return YHM2712_ACMD_STATUS_BAD_PARAM;
    }

    trace->result = YHM2712_ACMD_STATUS_OK;
    trace->pad_out_enable = pad_out_enable;
    trace->fallback_allowed = false;
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    yhm2712_acmd_capture_gpio(&trace->before_prepare);
    trace->level_before_prepare = trace->before_prepare.in_m;
    yhm2712_acmd_selftest_phase_init(trace);

    status = yhm2712_acmd_prepare_pin_for_acmd(pad_out_enable);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        trace->result = status;
        yhm2712_acmd_restore_status_input();
        yhm2712_acmd_capture_gpio(&trace->after_restore);
        trace->level_after_restore = trace->after_restore.in_m;
        return status;
    }

    yhm2712_acmd_latch_fast_gpio();

#if YHM2712_ACMD_VENDOR_DRIVE_HIGH_ENABLE
    yhm2712_acmd_drive_high_fast();
    yhm2712_acmd_delay_us(YHM2712_ACMD_VENDOR_SELFTEST_IDLE_US);
#else
    yhm2712_acmd_release_fast();
#endif
    status = yhm2712_acmd_wait_selftest_high(&wait_ticks);
    if ((status != YHM2712_ACMD_STATUS_OK) &&
        (trace->level_before_prepare == 0U))
    {
        status = YHM2712_ACMD_STATUS_SELFTEST_LINE_LOW;
    }
    yhm2712_acmd_selftest_record_phase(trace,
                                       YHM2712_ACMD_SELFTEST_PHASE_RELEASE_INITIAL,
                                       status,
                                       wait_ticks);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        trace->result = (status == YHM2712_ACMD_STATUS_SELFTEST_LINE_LOW) ?
                        status : YHM2712_ACMD_STATUS_SELFTEST_RELEASE_INITIAL_FAIL;
        yhm2712_acmd_selftest_record_phase(trace,
                                           YHM2712_ACMD_SELFTEST_PHASE_DRIVE_LOW,
                                           trace->result,
                                           0U);
        yhm2712_acmd_selftest_record_phase(trace,
                                           YHM2712_ACMD_SELFTEST_PHASE_RELEASE_AFTER_DRIVE,
                                           trace->result,
                                           0U);
        goto restore;
    }

    yhm2712_acmd_drive_low_fast();
    yhm2712_acmd_delay_us(YHM2712_ACMD_SELFTEST_LOW_US);
    yhm2712_acmd_selftest_record_phase(trace,
                                       YHM2712_ACMD_SELFTEST_PHASE_DRIVE_LOW,
                                       YHM2712_ACMD_STATUS_OK,
                                       YHM2712_ACMD_SELFTEST_LOW_US *
                                       YHM2712_ACMD_VENDOR_TICK_PER_US);
    if ((trace->phase[YHM2712_ACMD_SELFTEST_PHASE_DRIVE_LOW].gpio.dir_m != 1U) ||
        (trace->phase[YHM2712_ACMD_SELFTEST_PHASE_DRIVE_LOW].gpio.out_m != 0U) ||
        (trace->phase[YHM2712_ACMD_SELFTEST_PHASE_DRIVE_LOW].gpio.in_m != 0U))
    {
        trace->phase[YHM2712_ACMD_SELFTEST_PHASE_DRIVE_LOW].status =
            YHM2712_ACMD_STATUS_SELFTEST_DRIVE_LOW_FAIL;
        trace->fallback_allowed =
            (trace->phase[YHM2712_ACMD_SELFTEST_PHASE_DRIVE_LOW].gpio.dir_m == 1U) &&
            (trace->phase[YHM2712_ACMD_SELFTEST_PHASE_DRIVE_LOW].gpio.out_m == 0U) &&
            (trace->phase[YHM2712_ACMD_SELFTEST_PHASE_DRIVE_LOW].gpio.in_m == 1U);
        trace->result = YHM2712_ACMD_STATUS_SELFTEST_DRIVE_LOW_FAIL;
    }

#if YHM2712_ACMD_VENDOR_DRIVE_HIGH_ENABLE
    yhm2712_acmd_drive_high_fast();
#else
    yhm2712_acmd_release_fast();
#endif
    wait_ticks = 0U;
    status = yhm2712_acmd_wait_selftest_high(&wait_ticks);
    yhm2712_acmd_selftest_record_phase(trace,
                                       YHM2712_ACMD_SELFTEST_PHASE_RELEASE_AFTER_DRIVE,
                                       status,
                                       wait_ticks);
    if ((trace->result == YHM2712_ACMD_STATUS_OK) &&
        (status != YHM2712_ACMD_STATUS_OK))
    {
        trace->result = YHM2712_ACMD_STATUS_SELFTEST_RELEASE_AFTER_FAIL;
    }

    if (trace->result == YHM2712_ACMD_STATUS_OK)
    {
        yhm2712_acmd_delay_us(YHM2712_ACMD_IDLE_AFTER_SELFTEST_US);
    }

restore:
    yhm2712_acmd_release_fast();
#if YHM2712_ACMD_VENDOR_DRIVE_HIGH_ENABLE
    GPIO->DATAOUT = s_yhm2712_acmd.fast_gpio.dataout_high;
#else
    GPIO->DATAOUT = s_yhm2712_acmd.fast_gpio.dataout_low;
#endif
    GPIO->DATASRC = s_yhm2712_acmd.fast_gpio.datasrc_low;
    GPIO->DATADIR = s_yhm2712_acmd.fast_gpio.datadir_release_z;
    yhm2712_acmd_restore_status_input();
    yhm2712_acmd_capture_gpio(&trace->after_restore);
    trace->level_after_restore = trace->after_restore.in_m;
    return trace->result;
}

static void yhm2712_acmd_log_selftest_trace(const yhm2712_acmd_selftest_trace_t *trace)
{
#if YHM2712_ACMD_TRACE_ENABLE
    if (trace == NULL)
    {
        return;
    }

    YHM_DETAIL_LOG("[YHM2712][ACMD] %s release_selftest status=%s before=%u after=%u pad_out=%u",
               (trace->result == YHM2712_ACMD_STATUS_OK) ? "ok" : "fail",
               yhm2712_acmd_status_name(trace->result),
               trace->level_before_prepare,
               trace->level_after_restore,
               trace->pad_out_enable);
#else
    (void)trace;
#endif
}

static yhm2712_acmd_status_t yhm2712_acmd_release_selftest(
    yhm2712_acmd_selftest_trace_t *final_trace)
{
    yhm2712_acmd_selftest_trace_t first_trace;
    yhm2712_acmd_selftest_trace_t retry_trace;
    yhm2712_acmd_status_t status;
    const uint8_t default_pad_out =
        (YHM2712_ACMD_PAD_OUT_ENABLE_FOR_GPIO != 0U) ? 1U : 0U;

    status = yhm2712_acmd_release_selftest_once(default_pad_out, &first_trace);
    yhm2712_acmd_log_selftest_trace(&first_trace);

    if ((status == YHM2712_ACMD_STATUS_SELFTEST_DRIVE_LOW_FAIL) &&
        (first_trace.fallback_allowed) &&
        (default_pad_out == 0U))
    {
#if YHM_PRODUCTION_COMPACT
        ++s_yhm_comm_stats.retries;
        yhm_mos_preflight_error_log(YHM2712_REG_ID, status);
#endif
        status = yhm2712_acmd_release_selftest_once(1U, &retry_trace);
        yhm2712_acmd_log_selftest_trace(&retry_trace);
        if (final_trace != NULL)
        {
            *final_trace = retry_trace;
        }
        return status;
    }

    if (final_trace != NULL)
    {
        *final_trace = first_trace;
    }
    return status;
}

bool yhm2712_acmd_init(uint8_t stacmd_pin)
{
    uint32_t gpio_pin = GPIO_GetPin(stacmd_pin);
    bool first_init = !s_yhm2712_acmd.inited;

    if (first_init)
    {
#if defined(ZY100_BUILD_FACTORY) && ZY100_BUILD_FACTORY
        YHM_DETAIL_LOG("[YHM][ID_POLICY] FACTORY_ID_MAX=" YHM_BUILD_STRING(YHM2712_ACMD_ID_MAX_ATTEMPTS)
                   ";STABLE=" YHM_BUILD_STRING(YHM2712_ACMD_ID_STABLE_READ_COUNT));
#endif
#if YHM_PRODUCTION_COMPACT
        DBG_DIRECT("[YHM] %s", ZY100_YHM_PROFILE_TAG ";code=" YHM_BUILD_STRING(VERSION_CODE));
#endif
        YHM_DETAIL_LOG("[YHM][HW] %s MOS=%u source=%u ack_ticks=%u data_ticks=%u gap_tbit=8",
                   ZY100_YHM_PROFILE_TAG ";code=" YHM_BUILD_STRING(VERSION_CODE),
                   ZY100_YHM_HAS_MOS, ZY100_YHM_TIMING_SOURCE,
                   ZY100_YHM_ACK_THRESHOLD_TICKS, ZY100_YHM_DATA_THRESHOLD_TICKS);
    }

    if ((gpio_pin == 0U) || (gpio_pin == 0xFFU))
    {
        s_yhm2712_acmd.inited = false;
        return false;
    }

    s_yhm2712_acmd.stacmd_pin = stacmd_pin;
    s_yhm2712_acmd.gpio_pin = gpio_pin;
    s_yhm2712_acmd.inited = true;
    s_yhm2712_acmd.busy = false;
    if (first_init)
    {
        s_yhm2712_acmd.external_power_profile_valid = false;
        s_yhm2712_acmd.tail_drive_high = true;
#if defined(ZY100_BUILD_YHM_PRESSURE_TEST) && \
    (ZY100_BUILD_YHM_PRESSURE_TEST == 1) && ZY100_YHM_HAS_MOS
        s_yhm_pressure_data_threshold_ticks =
            YHM_PRESSURE_DATA_PROFILE_FIXED_TICKS;
#endif
    }
    yhm2712_acmd_refresh_timing();
    if (!s_yhm2712_acmd.phy_log_reported)
    {
        YHM_DETAIL_LOG("[YHM][ACMD] base=10341 phy=vendor_pp tb=%u z=%u int=%u quick=%u tail=adaptive",
                   s_yhm2712_acmd.tbit_ticks,
                   s_yhm2712_acmd.logicz_ticks,
                   s_yhm2712_acmd.interval_ticks,
                   (uint32_t)(YHM2712_ACMD_VENDOR_QUICK_MODE != 0U));
        s_yhm2712_acmd.phy_log_reported = true;
    }
    return true;
}

#if defined(ZY100_BUILD_YHM_PRESSURE_TEST) && \
    (ZY100_BUILD_YHM_PRESSURE_TEST == 1) && ZY100_YHM_HAS_MOS
yhm2712_acmd_status_t yhm_pressure_data_profile_select(
    yhm_pressure_data_profile_t profile)
{
    uint32_t threshold_ticks;

    if (!s_yhm2712_acmd.inited)
    {
        return YHM2712_ACMD_STATUS_NOT_INIT;
    }
    if (s_yhm2712_acmd.busy)
    {
        return YHM2712_ACMD_STATUS_BUSY;
    }

    switch (profile)
    {
    case YHM_PRESSURE_DATA_PROFILE_FIXED:
        threshold_ticks = YHM_PRESSURE_DATA_PROFILE_FIXED_TICKS;
        break;
    default:
        return YHM2712_ACMD_STATUS_BAD_PARAM;
    }

    s_yhm_pressure_data_threshold_ticks = threshold_ticks;
    s_yhm_pressure_ack_threshold_ticks = ZY100_YHM_ACK_THRESHOLD_TICKS;
    return YHM2712_ACMD_STATUS_OK;
}

uint32_t yhm_pressure_data_threshold_ticks_get(void)
{
    return s_yhm_pressure_data_threshold_ticks;
}

uint32_t yhm_pressure_ack_threshold_ticks_get(void)
{
    return s_yhm_pressure_ack_threshold_ticks;
}
#endif

yhm2712_acmd_status_t yhm2712_acmd_select_external_power_profile(
    bool external_power_present)
{
    bool tail_drive_high;

    if (!s_yhm2712_acmd.inited)
    {
        return YHM2712_ACMD_STATUS_NOT_INIT;
    }
    if (s_yhm2712_acmd.busy)
    {
        return YHM2712_ACMD_STATUS_BUSY;
    }

    tail_drive_high = external_power_present;
    if (s_yhm2712_acmd.external_power_profile_valid &&
        (s_yhm2712_acmd.tail_drive_high == tail_drive_high))
    {
        return YHM2712_ACMD_STATUS_OK;
    }

    s_yhm2712_acmd.tail_drive_high = tail_drive_high;
    s_yhm2712_acmd.external_power_profile_valid = true;
    yhm2712_acmd_restore_status_input();
    YHM_DETAIL_LOG("[YHM][PHY] ac=%u tail=%s",
               external_power_present ? 1U : 0U,
               tail_drive_high ? "high" : "release");
    return YHM2712_ACMD_STATUS_OK;
}

yhm2712_acmd_status_t yhm2712_acmd_boot_toggle_probe(uint8_t stacmd_pin,
                                                     uint32_t duration_ms,
                                                     uint32_t half_period_ms,
                                                     uint8_t pad_out_enable,
                                                     yhm2712_acmd_delay_ms_fn_t delay_ms)
{
    GPIO_InitTypeDef gpio_init;
    uint32_t mask;
    uint32_t dataout_restore;
    uint32_t datasrc_restore;
    uint32_t datadir_restore;
    uint32_t dataout_low;
    uint32_t datasrc_low;
    uint32_t datadir_release_z;
    uint32_t datadir_drive_low;
    uint32_t cycle_ms;
    uint32_t cycles;
    uint32_t cycle;

    if ((duration_ms == 0U) ||
        (half_period_ms == 0U) ||
        (delay_ms == NULL))
    {
        return YHM2712_ACMD_STATUS_BAD_PARAM;
    }
    cycle_ms = half_period_ms * 2U;
    if ((cycle_ms < half_period_ms) || (duration_ms < cycle_ms))
    {
        return YHM2712_ACMD_STATUS_BAD_PARAM;
    }

    if (!yhm2712_acmd_init(stacmd_pin))
    {
        return YHM2712_ACMD_STATUS_BAD_PIN;
    }
    mask = s_yhm2712_acmd.gpio_pin;
    cycles = duration_ms / cycle_ms;
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);

    dataout_restore = GPIO->DATAOUT;
    datasrc_restore = GPIO->DATASRC;
    datadir_restore = GPIO->DATADIR;
    dataout_low = dataout_restore & ~mask;
    datasrc_low = datasrc_restore & ~mask;
    datadir_release_z = datadir_restore & ~mask;
    datadir_drive_low = datadir_release_z | mask;

#if YHM2712_ACMD_TRACE_ENABLE
    YHM_DETAIL_LOG("[YHM2712][ACMD][LA] boot_toggle begin pin=%u duration_ms=%u half_ms=%u cycles=%u pad_out=%u",
               stacmd_pin,
               duration_ms,
               half_period_ms,
               cycles,
               (pad_out_enable != 0U) ? 1U : 0U);
#endif

    GPIO->DATADIR = datadir_release_z;
    GPIO->DATAOUT = dataout_low;
    GPIO->DATASRC = datasrc_low;
    GPIO->DATADIR = datadir_release_z;

    Pad_Config(stacmd_pin, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE,
               (pad_out_enable != 0U) ? PAD_OUT_ENABLE : PAD_OUT_DISABLE,
               PAD_OUT_LOW);
    Pinmux_Deinit(stacmd_pin);
    Pinmux_Config(stacmd_pin, DWGPIO);

    GPIO_StructInit(&gpio_init);
    gpio_init.GPIO_Pin = mask;
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_ITCmd = DISABLE;
    gpio_init.GPIO_ControlMode = GPIO_SOFTWARE_MODE;
    GPIO_Init(&gpio_init);

    GPIO->DATADIR = datadir_release_z;
    GPIO->DATAOUT = dataout_low;
    GPIO->DATASRC = datasrc_low;
    GPIO->DATADIR = datadir_release_z;

    for (cycle = 0U; cycle < cycles; cycle++)
    {
        GPIO->DATADIR = datadir_drive_low;
        delay_ms(half_period_ms);
        GPIO->DATADIR = datadir_release_z;
        delay_ms(half_period_ms);
    }

    GPIO->DATADIR = datadir_release_z;
    GPIO->DATAOUT = dataout_low;
    GPIO->DATASRC = datasrc_low;
    GPIO->DATADIR = datadir_release_z;
    yhm2712_acmd_restore_status_input();

#if YHM2712_ACMD_TRACE_ENABLE
    YHM_DETAIL_LOG("[YHM2712][ACMD][LA] boot_toggle ok pin=%u status=%s",
               stacmd_pin,
               yhm2712_acmd_status_name(YHM2712_ACMD_STATUS_OK));
#endif
    return YHM2712_ACMD_STATUS_OK;
}

#if YHM_PRODUCTION_COMPACT
#define s_mos_transactions s_yhm_comm_stats.transactions
bool yhm2712_acmd_comm_stats_get(yhm2712_comm_stats_t *stats)
{
    if (stats == NULL) return false;
    *stats = s_yhm_comm_stats;
    return true;
}
void yhm_mos_error_counters_log(const char *reason) { (void)reason; }
static void yhm_mos_preflight_error_log(uint8_t reg, yhm2712_acmd_status_t status)
{
    ++s_yhm_comm_stats.io_errors;
    DBG_DIRECT("[YHM_ERR] op=CHECK reg=%02x status=%s", reg, yhm2712_acmd_status_name(status));
}
void yhm2712_acmd_value_error(uint8_t reg, uint8_t expected, uint8_t actual,
                            yhm2712_acmd_status_t status)
{
    ++s_yhm_comm_stats.value_errors;
    DBG_DIRECT("[YHM_ERR] op=VERIFY reg=%02x status=%s exp=%02x got=%02x",
               reg, yhm2712_acmd_status_name(status), expected, actual);
}
void yhm_mos_report_value_mismatch(uint8_t reg, uint8_t expected, uint8_t actual)
{
    yhm2712_acmd_value_error(reg, expected, actual,
        reg == YHM2712_REG_ID ? YHM2712_ACMD_STATUS_ID_MISMATCH : YHM2712_ACMD_STATUS_READBACK_MISMATCH);
}
static void yhm_mos_transaction_error_log(uint8_t reg, bool is_write,
                                         yhm2712_acmd_status_t status)
{
    if (status == YHM2712_ACMD_STATUS_OK) return;
    ++s_yhm_comm_stats.io_errors;
    DBG_DIRECT("[YHM_ERR] op=%c reg=%02x status=%s", is_write ? 'W' : 'R',
               reg, yhm2712_acmd_status_name(status));
}
#else
#if ZY100_YHM_HAS_MOS
/* Policy outside the wire critical section; reuse the existing recovery idle.
 * This is a board-test candidate, not a new IC timing specification. */
static uint32_t s_mos_transactions;
static uint32_t s_mos_transport_errors;
static uint32_t s_mos_value_errors;

void yhm_mos_error_counters_log(const char *reason)
{
    YHM_DETAIL_LOG("[YHM_SUM] reason=%s tx=%lu io_err=%lu value_err=%lu guard_us=%u",
               reason, s_mos_transactions, s_mos_transport_errors,
               s_mos_value_errors, YHM2712_ACMD_WRITE_RETRY_IDLE_US);
}

static void yhm_mos_preflight_error_log(uint8_t reg, yhm2712_acmd_status_t status)
{
    /* No bus capture belongs to a rejected/preflight call. */
    s_mos_transport_errors++;
    YHM_DETAIL_LOG("[YHM_ERR] tx=%lu io_err=%lu preflight=1 reg=%02x status=%s",
               s_mos_transactions, s_mos_transport_errors, reg,
               yhm2712_acmd_status_name(status));
}

static void yhm_mos_rx_error_log(uint8_t reg)
{
    uint8_t i;
    if (!s_yhm_pressure_rx.valid || s_yhm_pressure_rx.reg != reg)
    {
        return;
    }
    YHM_DETAIL_LOG("[YHM_RX] tx=%lu seq=%lu reg=%02x data=%02x bits=%u entry=%u capture=%s",
               s_mos_transactions, s_yhm_pressure_rx.sequence, reg,
               s_yhm_pressure_rx.value, s_yhm_pressure_rx.bit_count,
               s_yhm_pressure_rx.entry_low_bit,
               yhm2712_acmd_status_name(s_yhm_pressure_rx.capture_status));
    for (i = 0U; i < s_yhm_pressure_rx.bit_count && i < 8U; i++)
    {
        YHM_DETAIL_LOG("[YHM_BIT] seq=%lu bit=%u fall=%lu rise=%lu low=%lu",
                   s_yhm_pressure_rx.sequence, i, s_yhm_pressure_rx.fall[i],
                   s_yhm_pressure_rx.rise[i],
                   (s_yhm_pressure_rx.rise[i] - s_yhm_pressure_rx.fall[i]) &
                   YHM2712_ACMD_VENDOR_TICK_MASK);
    }
    YHM_DETAIL_LOG("[YHM_RX_END] seq=%lu wait=%lu timeout=%lu",
               s_yhm_pressure_rx.sequence, s_yhm_pressure_rx.wait_start,
               s_yhm_pressure_rx.timeout_tick);
}

void yhm_mos_report_value_mismatch(uint8_t reg, uint8_t expected, uint8_t actual)
{
    s_mos_value_errors++;
    YHM_DETAIL_LOG("[YHM_ERR] tx=%lu value_err=%lu reg=%02x expected=%02x actual=%02x",
               s_mos_transactions, s_mos_value_errors, reg, expected, actual);
    yhm_mos_rx_error_log(reg);
}

static void yhm_mos_transaction_error_log(uint8_t reg, bool is_write,
                                         yhm2712_acmd_status_t status)
{
    if (status == YHM2712_ACMD_STATUS_OK)
    {
        return;
    }
    s_mos_transport_errors++;
    YHM_DETAIL_LOG("[YHM_ERR] tx=%lu io_err=%lu write=%u reg=%02x status=%s irq=%lu",
               s_mos_transactions, s_mos_transport_errors, is_write, reg,
               yhm2712_acmd_status_name(status), __get_PRIMASK());
    if (s_yhm_pressure_last_ack.valid)
    {
        YHM_DETAIL_LOG("[YHM_ACK] tx=%lu stage=%u low=%lu threshold=%lu expected=%u got=%u",
                   s_mos_transactions, s_yhm_pressure_last_ack.stage,
                   s_yhm_pressure_last_ack.low_ticks,
                   s_yhm_pressure_last_ack.threshold_ticks,
                   s_yhm_pressure_last_ack.expected, s_yhm_pressure_last_ack.symbol);
        YHM_DETAIL_LOG("[YHM_ACK_TS] tx=%lu start=%lu fall=%lu rise=%lu entry=%u",
                   s_mos_transactions, s_yhm_pressure_diag_work.ack_start,
                   s_yhm_pressure_diag_work.ack_fall, s_yhm_pressure_diag_work.ack_rise,
                   s_yhm_pressure_diag_work.ack_entry_low);
    }
    if (!is_write)
    {
        yhm_mos_rx_error_log(reg);
    }
}
#endif

#endif

static yhm2712_acmd_status_t yhm2712_acmd_read_reg_policy(
    uint8_t reg,
    uint8_t *value,
    bool pre_idle_high)
{
    yhm2712_acmd_status_t status = YHM2712_ACMD_STATUS_OK;
    uint32_t primask_before = __get_PRIMASK();
    uint32_t irq_state = 0U;
    bool irq_taken = false;
    bool busy_set = false;
#if !YHM_PRODUCTION_COMPACT
    yhm2712_acmd_trace_snapshot_t snapshot;
#endif

    yhm2712_acmd_recovery_snapshot_clear();

    if (value == NULL)
    {
#if ZY100_YHM_HAS_MOS || YHM_PRODUCTION_COMPACT
        yhm_mos_preflight_error_log(reg, YHM2712_ACMD_STATUS_BAD_PARAM);
#endif
        return YHM2712_ACMD_STATUS_BAD_PARAM;
    }
    if (!s_yhm2712_acmd.inited)
    {
#if ZY100_YHM_HAS_MOS || YHM_PRODUCTION_COMPACT
        yhm_mos_preflight_error_log(reg, YHM2712_ACMD_STATUS_NOT_INIT);
#endif
        return YHM2712_ACMD_STATUS_NOT_INIT;
    }
    if (s_yhm2712_acmd.busy)
    {
#if ZY100_YHM_HAS_MOS || YHM_PRODUCTION_COMPACT
        yhm_mos_preflight_error_log(reg, YHM2712_ACMD_STATUS_BUSY);
#endif
        return YHM2712_ACMD_STATUS_BUSY;
    }

    s_yhm2712_acmd.busy = true;
    busy_set = true;
#if YHM_PRODUCTION_COMPACT && !ZY100_YHM_HAS_MOS
    ++s_yhm_comm_stats.transactions;
#endif
#if ZY100_YHM_HAS_MOS
    s_mos_transactions++;
    s_yhm_pressure_last_ack.valid = false;
#endif
    yhm2712_acmd_refresh_timing();
#if ZY100_YHM_HAS_MOS
    yhm_pressure_rx_prepare(reg);
#endif
#if YHM_PRESSURE_DIAG_ENABLE
    yhm_pressure_diag_begin();
#endif
    yhm2712_acmd_trace_clear();
    if (!yhm2712_acmd_tick_running())
    {
        status = YHM2712_ACMD_STATUS_TICK_STUCK;
        goto cleanup;
    }

    status = yhm2712_acmd_prepare_pin_for_acmd(
                 (YHM2712_ACMD_PAD_OUT_ENABLE_FOR_GPIO != 0U) ? 1U : 0U);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        yhm2712_acmd_trace_record_fail(reg, YHM2712_ACMD_STAGE_LINE_IDLE, status);
        goto cleanup;
    }

#if ZY100_YHM_HAS_MOS
    /* Pad/pinmux has just changed. Allow settling with interrupts enabled. */
    yhm2712_acmd_delay_us(YHM2712_ACMD_WRITE_RETRY_IDLE_US);
#endif
    irq_state = __get_PRIMASK();
    __disable_irq();
    irq_taken = true;
    yhm2712_acmd_latch_fast_gpio();
    if (pre_idle_high)
    {
        status = yhm2712_acmd_wait_released_high(
                     YHM2712_ACMD_SHIPPING_PRE_IDLE_US *
                     YHM2712_ACMD_VENDOR_TICK_PER_US);
        if (status != YHM2712_ACMD_STATUS_OK)
        {
            yhm2712_acmd_trace_record_fail(reg,
                                           YHM2712_ACMD_STAGE_LINE_IDLE,
                                           status);
            goto cleanup;
        }
    }
    status = yhm2712_acmd_read_reg_locked(reg, value);

cleanup:
    if (irq_taken)
    {
        yhm2712_acmd_restore_fast_gpio_irq_disabled();
        if (irq_state == 0U)
        {
            __enable_irq();
        }
        irq_taken = false;
    }

    yhm2712_acmd_restore_status_input();
    s_yhm2712_acmd.level_after_restore = yhm2712_acmd_line_is_high() ? 1U : 0U;
    if ((status != YHM2712_ACMD_STATUS_OK) && (!s_yhm2712_acmd.fail_trace.valid))
    {
        yhm2712_acmd_trace_record_fail(reg, YHM2712_ACMD_STAGE_STOP, status);
    }
#if ZY100_YHM_HAS_MOS
    s_yhm_pressure_rx.status = status;
#endif
#if ZY100_YHM_HAS_MOS || YHM_PRODUCTION_COMPACT
    if (status == YHM2712_ACMD_STATUS_OK && reg == YHM2712_REG_ID &&
        *value != YHM2712_ID_DEFAULT)
    {
        yhm_mos_report_value_mismatch(reg, YHM2712_ID_DEFAULT, *value);
    }
#endif
#if !YHM_PRODUCTION_COMPACT
    yhm2712_acmd_trace_snapshot(reg, false, 0U, &snapshot);
    yhm2712_acmd_failure_snapshot_record(&snapshot, status);
#endif
#if ZY100_YHM_HAS_MOS || YHM_PRODUCTION_COMPACT
    /* IRQ restored, but BUSY still owns the raw capture until logging ends. */
    yhm_mos_transaction_error_log(reg, false, status);
#endif
    if (busy_set)
    {
        s_yhm2712_acmd.busy = false;
    }
    yhm2712_acmd_recovery_snapshot_record(primask_before, true);
#if YHM_PRESSURE_DIAG_ENABLE
    yhm_pressure_diag_complete(false, 0U, status);
#endif
#if !YHM_PRODUCTION_COMPACT
    yhm2712_acmd_trace_log_snapshot(&snapshot);
#endif
    return status;
}

yhm2712_acmd_status_t yhm2712_acmd_read_reg(uint8_t reg, uint8_t *value)
{
    return yhm2712_acmd_read_reg_policy(reg, value, false);
}

static yhm2712_acmd_status_t yhm2712_acmd_write_reg_policy(
    uint8_t reg,
    uint8_t value,
    bool force_vendor_handoff_high,
    bool edge_timed,
    bool pre_idle_high)
{
    yhm2712_acmd_status_t status = YHM2712_ACMD_STATUS_OK;
    uint32_t primask_before = __get_PRIMASK();
    uint32_t irq_state = 0U;
    bool irq_taken = false;
    bool busy_set = false;
#if !YHM_PRODUCTION_COMPACT
    yhm2712_acmd_trace_snapshot_t snapshot;
#endif

    yhm2712_acmd_recovery_snapshot_clear();

    if (!s_yhm2712_acmd.inited)
    {
#if ZY100_YHM_HAS_MOS || YHM_PRODUCTION_COMPACT
        yhm_mos_preflight_error_log(reg, YHM2712_ACMD_STATUS_NOT_INIT);
#endif
        return YHM2712_ACMD_STATUS_NOT_INIT;
    }
    if (s_yhm2712_acmd.busy)
    {
#if ZY100_YHM_HAS_MOS || YHM_PRODUCTION_COMPACT
        yhm_mos_preflight_error_log(reg, YHM2712_ACMD_STATUS_BUSY);
#endif
        return YHM2712_ACMD_STATUS_BUSY;
    }

    s_yhm2712_acmd.busy = true;
    busy_set = true;
#if YHM_PRODUCTION_COMPACT && !ZY100_YHM_HAS_MOS
    ++s_yhm_comm_stats.transactions;
#endif
#if ZY100_YHM_HAS_MOS
    s_mos_transactions++;
    s_yhm_pressure_last_ack.valid = false;
#endif
    yhm2712_acmd_refresh_timing();
#if YHM_PRESSURE_DIAG_ENABLE
    yhm_pressure_rx_prepare(reg);
    yhm_pressure_diag_begin();
#endif
    yhm2712_acmd_trace_clear();
    if (!yhm2712_acmd_tick_running())
    {
        status = YHM2712_ACMD_STATUS_TICK_STUCK;
        goto cleanup;
    }

    status = yhm2712_acmd_prepare_pin_for_acmd(
                 (YHM2712_ACMD_PAD_OUT_ENABLE_FOR_GPIO != 0U) ? 1U : 0U);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        yhm2712_acmd_trace_record_fail(reg, YHM2712_ACMD_STAGE_LINE_IDLE, status);
        goto cleanup;
    }

#if ZY100_YHM_HAS_MOS
    /* Pad/pinmux has just changed. Allow settling with interrupts enabled. */
    yhm2712_acmd_delay_us(YHM2712_ACMD_WRITE_RETRY_IDLE_US);
#endif
    irq_state = __get_PRIMASK();
    __disable_irq();
    irq_taken = true;
    yhm2712_acmd_latch_fast_gpio();
    status = yhm2712_acmd_write_reg_locked_policy(reg, value,
                                                   force_vendor_handoff_high,
                                                   edge_timed,
                                                   pre_idle_high);

cleanup:
    if (irq_taken)
    {
        yhm2712_acmd_restore_fast_gpio_irq_disabled();
        if (irq_state == 0U)
        {
            __enable_irq();
        }
        irq_taken = false;
    }

    yhm2712_acmd_restore_status_input();
    s_yhm2712_acmd.level_after_restore = yhm2712_acmd_line_is_high() ? 1U : 0U;
    if ((status != YHM2712_ACMD_STATUS_OK) && (!s_yhm2712_acmd.fail_trace.valid))
    {
        yhm2712_acmd_trace_record_fail(reg, YHM2712_ACMD_STAGE_STOP, status);
    }
#if !YHM_PRODUCTION_COMPACT
    yhm2712_acmd_trace_snapshot(reg, true, value, &snapshot);
    yhm2712_acmd_failure_snapshot_record(&snapshot, status);
#endif
#if ZY100_YHM_HAS_MOS || YHM_PRODUCTION_COMPACT
    /* IRQ restored, but BUSY still owns the raw capture until logging ends. */
    yhm_mos_transaction_error_log(reg, true, status);
#endif
    if (busy_set)
    {
        s_yhm2712_acmd.busy = false;
    }
    yhm2712_acmd_recovery_snapshot_record(primask_before, true);
#if YHM_PRESSURE_DIAG_ENABLE
    yhm_pressure_diag_complete(true, value, status);
#endif
#if !YHM_PRODUCTION_COMPACT
    yhm2712_acmd_trace_log_snapshot(&snapshot);
#endif
    return status;
}

yhm2712_acmd_status_t yhm2712_acmd_write_reg(uint8_t reg, uint8_t value)
{
    return yhm2712_acmd_write_reg_policy(reg, value, false, false, false);
}

yhm2712_acmd_status_t yhm2712_acmd_read_reg_retry(uint8_t reg,
                                                   uint8_t *value,
                                                   uint8_t *attempts_out)
{
    return yhm2712_acmd_read_reg_retry_diag(reg,
                                             value,
                                             attempts_out,
                                             NULL);
}

static yhm2712_acmd_status_t yhm2712_acmd_read_reg_retry_diag_policy(
    uint8_t reg,
    uint8_t *value,
    uint8_t *attempts_out,
    yhm2712_acmd_attempt_trace_t *trace,
    bool pre_idle_high)
{
    yhm2712_acmd_status_t status = YHM2712_ACMD_STATUS_BAD_PARAM;
    uint8_t attempt;
    uint8_t readback = 0U;

    yhm2712_acmd_attempt_trace_reset(trace);
    if (attempts_out != NULL)
    {
        *attempts_out = 0U;
    }
    if (value == NULL)
    {
        return status;
    }
    *value = 0U;

    for (attempt = 1U; attempt <= YHM2712_ACMD_READ_RETRY_MAX; attempt++)
    {
#if YHM_PRODUCTION_COMPACT
        if (attempt > 1U) ++s_yhm_comm_stats.retries;
#endif
        if (attempts_out != NULL)
        {
            *attempts_out = attempt;
        }

        status = yhm2712_acmd_read_reg_policy(reg,
                                               &readback,
                                               pre_idle_high);
        if (status == YHM2712_ACMD_STATUS_OK)
        {
            *value = readback;
            yhm2712_acmd_attempt_trace_record(
                trace,
                reg,
                attempt,
                readback,
                YHM2712_ACMD_ATTEMPT_RESULT_SUCCESS,
                false,
                YHM2712_ACMD_STATUS_NOT_INIT,
                true,
                status,
                true,
                readback);
            return status;
        }

        yhm2712_acmd_attempt_trace_record(
            trace,
            reg,
            attempt,
            0U,
            YHM2712_ACMD_ATTEMPT_RESULT_READ_FAIL,
            false,
            YHM2712_ACMD_STATUS_NOT_INIT,
            true,
            status,
            false,
            0U);
        if (!pre_idle_high)
        {
            yhm2712_acmd_delay_us(YHM2712_ACMD_READ_RETRY_IDLE_US);
        }
    }

    return status;
}

yhm2712_acmd_status_t yhm2712_acmd_read_reg_retry_diag(
    uint8_t reg,
    uint8_t *value,
    uint8_t *attempts_out,
    yhm2712_acmd_attempt_trace_t *trace)
{
    return yhm2712_acmd_read_reg_retry_diag_policy(reg,
                                                    value,
                                                    attempts_out,
                                                    trace,
                                                    false);
}

yhm2712_acmd_status_t yhm2712_acmd_read_reg_retry_pre_idle_diag(
    uint8_t reg,
    uint8_t *value,
    uint8_t *attempts_out,
    yhm2712_acmd_attempt_trace_t *trace)
{
    return yhm2712_acmd_read_reg_retry_diag_policy(reg,
                                                    value,
                                                    attempts_out,
                                                    trace,
                                                    true);
}

yhm2712_acmd_status_t yhm2712_acmd_write_reg_edge_timed_retry_no_readback(
    uint8_t reg,
    uint8_t value,
    uint8_t *attempts_out)
{
    return yhm2712_acmd_write_reg_edge_timed_retry_no_readback_diag(
               reg,
               value,
               attempts_out,
               NULL);
}

yhm2712_acmd_status_t
yhm2712_acmd_write_reg_edge_timed_retry_no_readback_diag(
    uint8_t reg,
    uint8_t value,
    uint8_t *attempts_out,
    yhm2712_acmd_attempt_trace_t *trace)
{
    yhm2712_acmd_status_t status = YHM2712_ACMD_STATUS_NOT_INIT;
    uint8_t attempt;

    yhm2712_acmd_attempt_trace_reset(trace);
    if (attempts_out != NULL)
    {
        *attempts_out = 0U;
    }

    for (attempt = 1U; attempt <= YHM2712_ACMD_VENDOR_WRITE_ATTEMPTS; attempt++)
    {
#if YHM_PRODUCTION_COMPACT
        if (attempt > 1U) ++s_yhm_comm_stats.retries;
#endif
        if (attempts_out != NULL)
        {
            *attempts_out = attempt;
        }

        status = yhm2712_acmd_write_reg_policy(reg, value, true, true, false);
        if (status == YHM2712_ACMD_STATUS_OK)
        {
            yhm2712_acmd_attempt_trace_record(
                trace,
                reg,
                attempt,
                value,
                YHM2712_ACMD_ATTEMPT_RESULT_SUCCESS,
                true,
                status,
                false,
                YHM2712_ACMD_STATUS_NOT_INIT,
                false,
                0U);
            return status;
        }

        yhm2712_acmd_attempt_trace_record(
            trace,
            reg,
            attempt,
            value,
            YHM2712_ACMD_ATTEMPT_RESULT_WRITE_FAIL,
            true,
            status,
            false,
            YHM2712_ACMD_STATUS_NOT_INIT,
            false,
            0U);

        /* No deliberate released-idle dwell; each retry starts immediately. */
    }

    return status;
}

bool yhm2712_acmd_failure_snapshot_get(yhm2712_acmd_failure_snapshot_t *snapshot)
{
#if YHM_PRODUCTION_COMPACT
    (void)snapshot;
    return false;
#else
    if ((snapshot == NULL) || !s_yhm2712_acmd_last_failure.valid)
    {
        return false;
    }

    *snapshot = s_yhm2712_acmd_last_failure;
    return true;
#endif
}

bool yhm2712_acmd_recovery_snapshot_get(
    yhm2712_acmd_recovery_snapshot_t *snapshot)
{
    if ((snapshot == NULL) || !s_yhm2712_acmd_last_recovery.valid)
    {
        return false;
    }

    *snapshot = s_yhm2712_acmd_last_recovery;
    return true;
}

uint32_t yhm2712_acmd_ack_turnaround_tail_count_get(void)
{
    return s_yhm2712_acmd.ack_turnaround_tail_count;
}

#if YHM2712_ACMD_ACK_PREAMBLE_FILTER_ENABLE == 1U
uint32_t yhm2712_acmd_ack_preamble_filter_count_get(void)
{
    return s_yhm2712_acmd.ack_preamble_filter_count;
}
#endif

yhm2712_acmd_status_t yhm2712_acmd_write_reg_retry_readback(uint8_t reg,
                                                            uint8_t value,
                                                            uint8_t *readback,
                                                            uint8_t *attempts_out)
{
    return yhm2712_acmd_write_reg_retry_readback_diag(reg,
                                                       value,
                                                       readback,
                                                       attempts_out,
                                                       NULL);
}

static yhm2712_acmd_status_t
yhm2712_acmd_write_reg_retry_readback_diag_policy(
    uint8_t reg,
    uint8_t value,
    uint8_t *readback,
    uint8_t *attempts_out,
    yhm2712_acmd_attempt_trace_t *trace,
    bool pre_idle_high)
{
    yhm2712_acmd_status_t status;
    yhm2712_acmd_status_t last_status = YHM2712_ACMD_STATUS_OK;
    uint8_t attempt;
    uint8_t rb = 0U;

    yhm2712_acmd_attempt_trace_reset(trace);
    if (attempts_out != NULL)
    {
        *attempts_out = 0U;
    }
    if (readback != NULL)
    {
        *readback = 0U;
    }

    for (attempt = 1U; attempt <= YHM2712_ACMD_WRITE_RETRY_MAX; attempt++)
    {
#if YHM_PRODUCTION_COMPACT
        if (attempt > 1U) ++s_yhm_comm_stats.retries;
#endif
        if (attempts_out != NULL)
        {
            *attempts_out = attempt;
        }

        status = yhm2712_acmd_write_reg_policy(reg,
                                                value,
                                                false,
                                                false,
                                                pre_idle_high);
        if (status != YHM2712_ACMD_STATUS_OK)
        {
            last_status = status;
            yhm2712_acmd_attempt_trace_record(
                trace,
                reg,
                attempt,
                value,
                YHM2712_ACMD_ATTEMPT_RESULT_WRITE_FAIL,
                true,
                status,
                false,
                YHM2712_ACMD_STATUS_NOT_INIT,
                false,
                0U);
            if (!pre_idle_high)
            {
                yhm2712_acmd_delay_us(YHM2712_ACMD_WRITE_RETRY_IDLE_US);
            }
            continue;
        }

        status = yhm2712_acmd_read_reg_policy(reg, &rb, pre_idle_high);
        if (status != YHM2712_ACMD_STATUS_OK)
        {
            last_status = status;
            yhm2712_acmd_attempt_trace_record(
                trace,
                reg,
                attempt,
                value,
                YHM2712_ACMD_ATTEMPT_RESULT_READ_FAIL,
                true,
                YHM2712_ACMD_STATUS_OK,
                true,
                status,
                false,
                0U);
            if (!pre_idle_high)
            {
                yhm2712_acmd_delay_us(YHM2712_ACMD_WRITE_RETRY_IDLE_US);
            }
            continue;
        }

        if (readback != NULL)
        {
            *readback = rb;
        }

        if (rb == value)
        {
            yhm2712_acmd_attempt_trace_record(
                trace,
                reg,
                attempt,
                value,
                YHM2712_ACMD_ATTEMPT_RESULT_SUCCESS,
                true,
                YHM2712_ACMD_STATUS_OK,
                true,
                YHM2712_ACMD_STATUS_OK,
                true,
                rb);
#if YHM2712_ACMD_TRACE_ENABLE
            YHM_DETAIL_LOG("[YHM2712][REG] ok reg=0x%02x value=0x%02x read=0x%02x attempts=%u",
                       reg,
                       value,
                       rb,
                       attempt);
#endif
            return YHM2712_ACMD_STATUS_OK;
        }

        last_status = YHM2712_ACMD_STATUS_READBACK_MISMATCH;
#if ZY100_YHM_HAS_MOS || YHM_PRODUCTION_COMPACT
        yhm_mos_report_value_mismatch(reg, value, rb);
#endif
        yhm2712_acmd_attempt_trace_record(
            trace,
            reg,
            attempt,
            value,
            YHM2712_ACMD_ATTEMPT_RESULT_READBACK_MISMATCH,
            true,
            YHM2712_ACMD_STATUS_OK,
            true,
            YHM2712_ACMD_STATUS_OK,
            true,
            rb);
        if (!pre_idle_high)
        {
            yhm2712_acmd_delay_us(YHM2712_ACMD_WRITE_RETRY_IDLE_US);
        }
    }

    YHM_DETAIL_LOG("[YHM2712][REG][ERR] fail reg=0x%02x value=0x%02x read=0x%02x attempts=%u status=%s",
               reg,
               value,
               rb,
               YHM2712_ACMD_WRITE_RETRY_MAX,
               yhm2712_acmd_status_name(last_status));
    return last_status;
}

yhm2712_acmd_status_t yhm2712_acmd_write_reg_retry_readback_diag(
    uint8_t reg,
    uint8_t value,
    uint8_t *readback,
    uint8_t *attempts_out,
    yhm2712_acmd_attempt_trace_t *trace)
{
    return yhm2712_acmd_write_reg_retry_readback_diag_policy(reg,
                                                              value,
                                                              readback,
                                                              attempts_out,
                                                              trace,
                                                              false);
}

yhm2712_acmd_status_t yhm2712_acmd_write_reg_retry_readback_pre_idle_diag(
    uint8_t reg,
    uint8_t value,
    uint8_t *readback,
    uint8_t *attempts_out,
    yhm2712_acmd_attempt_trace_t *trace)
{
    return yhm2712_acmd_write_reg_retry_readback_diag_policy(reg,
                                                              value,
                                                              readback,
                                                              attempts_out,
                                                              trace,
                                                              true);
}

yhm2712_acmd_status_t yhm2712_acmd_read_fsm_mode(uint8_t *status2, uint8_t *fsm_mode)
{
    yhm2712_acmd_status_t status;
    uint8_t value = 0U;

    if ((status2 == NULL) || (fsm_mode == NULL))
    {
        return YHM2712_ACMD_STATUS_BAD_PARAM;
    }

    status = yhm2712_acmd_read_reg(YHM2712_REG_STATUS2, &value);
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return status;
    }

    *status2 = value;
    *fsm_mode = (uint8_t)((value & YHM2712_STATUS2_FSM_MODE_MASK) >>
                          YHM2712_STATUS2_FSM_MODE_SHIFT);
    return YHM2712_ACMD_STATUS_OK;
}

static bool yhm2712_acmd_fsm_matches(uint8_t fsm_mode,
                                     uint8_t expected_fsm,
                                     uint8_t alternate_fsm,
                                     bool allow_alternate)
{
    if (fsm_mode == expected_fsm)
    {
        return true;
    }
    return allow_alternate && (fsm_mode == alternate_fsm);
}

static yhm2712_acmd_status_t yhm2712_acmd_write_mode_and_verify_any(uint8_t mode_value,
                                                                    uint8_t expected_fsm,
                                                                    uint8_t alternate_fsm,
                                                                    bool allow_alternate)
{
    yhm2712_acmd_status_t status;
    yhm2712_acmd_status_t last_status = YHM2712_ACMD_STATUS_OK;
    uint8_t status2 = 0U;
    uint8_t fsm_mode = 0U;
    uint8_t attempt;

    for (attempt = 1U; attempt <= YHM2712_ACMD_WRITE_RETRY_MAX; attempt++)
    {
#if YHM_PRODUCTION_COMPACT
        if (attempt > 1U) ++s_yhm_comm_stats.retries;
#endif
        status = yhm2712_acmd_write_reg(YHM2712_REG_MODE, mode_value);
        if (status != YHM2712_ACMD_STATUS_OK)
        {
            last_status = status;
            yhm2712_acmd_delay_us(YHM2712_ACMD_WRITE_RETRY_IDLE_US);
            continue;
        }

        status = yhm2712_acmd_read_fsm_mode(&status2, &fsm_mode);
        if (status != YHM2712_ACMD_STATUS_OK)
        {
            last_status = status;
            yhm2712_acmd_delay_us(YHM2712_ACMD_WRITE_RETRY_IDLE_US);
            continue;
        }

        if (yhm2712_acmd_fsm_matches(fsm_mode,
                                     expected_fsm,
                                     alternate_fsm,
                                     allow_alternate))
        {
#if YHM2712_ACMD_TRACE_ENABLE
            YHM_DETAIL_LOG("[YHM2712][MODE] ok mode=0x%02x status2=0x%02x fsm=%u(%s) attempts=%u",
                       mode_value,
                       status2,
                       fsm_mode,
                       yhm2712_acmd_fsm_mode_name(fsm_mode),
                       attempt);
#endif
            return YHM2712_ACMD_STATUS_OK;
        }

        last_status = YHM2712_ACMD_STATUS_MODE_MISMATCH;
#if YHM_PRODUCTION_COMPACT
        yhm2712_acmd_value_error(YHM2712_REG_STATUS2, expected_fsm, fsm_mode, last_status);
#elif ZY100_YHM_HAS_MOS
        s_mos_value_errors++;
        YHM_DETAIL_LOG("[YHM_FSM_ERR] tx=%lu attempt=%u mode=%02x expected=%u alternate=%u allow=%u got=%u",
                   s_mos_transactions, attempt, mode_value, expected_fsm,
                   alternate_fsm, allow_alternate, fsm_mode);
#endif
        yhm2712_acmd_delay_us(YHM2712_ACMD_WRITE_RETRY_IDLE_US);
    }

    if (last_status == YHM2712_ACMD_STATUS_MODE_MISMATCH)
    {
        YHM_DETAIL_LOG("[YHM2712][MODE][ERR] fail mode=0x%02x attempts=%u status=%s status2=0x%02x fsm=%u(%s) expect=%u(%s)",
                   mode_value,
                   YHM2712_ACMD_WRITE_RETRY_MAX,
                   yhm2712_acmd_status_name(YHM2712_ACMD_STATUS_MODE_MISMATCH),
                   status2,
                   fsm_mode,
                   yhm2712_acmd_fsm_mode_name(fsm_mode),
                   expected_fsm,
                   yhm2712_acmd_fsm_mode_name(expected_fsm));
        return YHM2712_ACMD_STATUS_MODE_MISMATCH;
    }

    YHM_DETAIL_LOG("[YHM2712][MODE][ERR] fail mode=0x%02x attempts=%u status=%s",
               mode_value,
               YHM2712_ACMD_WRITE_RETRY_MAX,
               yhm2712_acmd_status_name(last_status));
    return last_status;
}

static yhm2712_acmd_status_t yhm2712_acmd_write_mode_and_verify(uint8_t mode_value,
                                                                uint8_t expected_fsm)
{
    return yhm2712_acmd_write_mode_and_verify_any(mode_value,
                                                  expected_fsm,
                                                  0U,
                                                  false);
}

yhm2712_acmd_status_t yhm2712_acmd_enter_sleep_mode(void)
{
    return yhm2712_acmd_write_mode_and_verify(YHM2712_MODE_SET_SLEEP,
                                              YHM2712_FSM_MODE_SLEEP);
}

yhm2712_acmd_status_t yhm2712_acmd_exit_sleep_to_discharge(void)
{
    return yhm2712_acmd_write_mode_and_verify(YHM2712_MODE_SET_DISCHARGE,
                                              YHM2712_FSM_MODE_DISCHARGE);
}

#if defined(ZY100_BUILD_FACTORY) && (ZY100_BUILD_FACTORY == 1)
yhm2712_acmd_status_t
yhm2712_acmd_factory_arm_battery_handover_with_input(void)
{
    /*
     * After VIN wakes the IC from Shipping, MODE=DISCHARGE re-arms the
     * battery handover request, but the live FSM remains CHARGE/CHARGE_DONE
     * while input power is still present.  Verify the command transaction
     * against those two valid VIN-present states; the subsequent VIN removal
     * is the board-level proof that BAT actually takes over SYS.
     */
    return yhm2712_acmd_write_mode_and_verify_any(
               YHM2712_MODE_SET_DISCHARGE,
               YHM2712_FSM_MODE_CHARGE,
               YHM2712_FSM_MODE_CHARGE_DONE,
               true);
}
#endif

yhm2712_acmd_status_t yhm2712_acmd_enter_charge_mode(void)
{
    return yhm2712_acmd_write_mode_and_verify_any(YHM2712_MODE_SET_CHARGE,
                                                  YHM2712_FSM_MODE_CHARGE,
                                                  YHM2712_FSM_MODE_CHARGE_DONE,
                                                  true);
}

#if defined(ZY100_BUILD_FACTORY) && ZY100_BUILD_FACTORY
static yhm2712_acmd_id_check_snapshot_t s_factory_id_check;

bool yhm2712_acmd_id_check_snapshot_get(yhm2712_acmd_id_check_snapshot_t *snapshot)
{
    if ((snapshot == NULL) || s_yhm2712_acmd.busy || !s_factory_id_check.valid)
    {
        return false;
    }
    *snapshot = s_factory_id_check;
    return true;
}
#endif

static yhm2712_acmd_status_t yhm2712_acmd_read_boot_id_stable(
    uint8_t *id,
    uint8_t *attempts,
    uint8_t *stable_count)
{
    yhm2712_acmd_status_t status = YHM2712_ACMD_STATUS_OK;
    yhm2712_acmd_status_t last_error = YHM2712_ACMD_STATUS_OK;
    uint8_t value = 0U;
    uint8_t attempt;
    uint8_t stable = 0U;

    if ((id == NULL) || (attempts == NULL) || (stable_count == NULL))
    {
        return YHM2712_ACMD_STATUS_BAD_PARAM;
    }

    *id = 0U;
    *attempts = 0U;
    *stable_count = 0U;

#if defined(ZY100_BUILD_FACTORY) && ZY100_BUILD_FACTORY
    s_factory_id_check.valid = true;
    s_factory_id_check.attempts = 0U;
    s_factory_id_check.stable_count = 0U;
    s_factory_id_check.failure_count = 0U;
#endif

    for (attempt = 0U; attempt < YHM2712_ACMD_ID_MAX_ATTEMPTS; attempt++)
    {
#if YHM_PRODUCTION_COMPACT
        if (attempt >= YHM2712_ACMD_ID_STABLE_READ_COUNT) ++s_yhm_comm_stats.id_extra_reads;
#endif
        status = yhm2712_acmd_read_reg(YHM2712_REG_ID, &value);
        *id = value;
        *attempts = (uint8_t)(attempt + 1U);

#if defined(ZY100_BUILD_FACTORY) && ZY100_BUILD_FACTORY
        /* The full register transaction has returned; never in bit capture. */
        s_factory_id_check.attempts = *attempts;
        s_factory_id_check.values[attempt] = value;
        s_factory_id_check.status[attempt] = status;
#endif

        if ((status == YHM2712_ACMD_STATUS_OK) && (value == YHM2712_ID_DEFAULT))
        {
            stable++;
            *stable_count = stable;
#if defined(ZY100_BUILD_FACTORY) && ZY100_BUILD_FACTORY
            s_factory_id_check.stable_count = stable;
#endif
            if (stable >= YHM2712_ACMD_ID_STABLE_READ_COUNT)
            {
                return YHM2712_ACMD_STATUS_OK;
            }
        }
        else
        {
#if YHM_PRESSURE_DIAG_ENABLE
            yhm_pressure_diag_freeze();
#endif
#if ZY100_YHM_HAS_MOS && !YHM_PRODUCTION_COMPACT
            if (!s_yhm_pressure_first_bad_id.valid &&
                s_yhm_pressure_rx.valid && !s_yhm2712_acmd.busy)
            {
                s_yhm_pressure_first_bad_id = s_yhm_pressure_rx;
                s_yhm_pressure_first_bad_id.status =
                    (status == YHM2712_ACMD_STATUS_OK) ?
                    YHM2712_ACMD_STATUS_ID_MISMATCH : status;
            }
#endif
            stable = 0U;
            *stable_count = 0U;
#if defined(ZY100_BUILD_FACTORY) && ZY100_BUILD_FACTORY
            s_factory_id_check.stable_count = 0U;
            s_factory_id_check.failure_count++;
#endif
            last_error = (status != YHM2712_ACMD_STATUS_OK) ?
                         status : YHM2712_ACMD_STATUS_ID_MISMATCH;
        }

        yhm2712_acmd_delay_us(YHM2712_ACMD_ID_RETRY_IDLE_US);
    }

    return (last_error != YHM2712_ACMD_STATUS_OK) ?
           last_error : YHM2712_ACMD_STATUS_ID_MISMATCH;
}

yhm2712_acmd_status_t yhm2712_acmd_read_boot_diag(yhm2712_acmd_boot_diag_t *diag)
{
    yhm2712_acmd_status_t status;
    yhm2712_acmd_selftest_trace_t selftest_trace;
    uint8_t idx;
    uint8_t value = 0U;

#if defined(ZY100_BUILD_FACTORY) && ZY100_BUILD_FACTORY
    s_factory_id_check.valid = false;
#endif
#if ZY100_YHM_HAS_MOS && !YHM_PRODUCTION_COMPACT
    s_yhm_pressure_first_bad_id.valid = false;
#endif
    if (diag == NULL)
    {
        return YHM2712_ACMD_STATUS_BAD_PARAM;
    }
    if (!s_yhm2712_acmd.inited)
    {
        return YHM2712_ACMD_STATUS_NOT_INIT;
    }

    diag->result = YHM2712_ACMD_STATUS_OK;
    diag->selftest_status = YHM2712_ACMD_STATUS_OK;
    diag->id_status = YHM2712_ACMD_STATUS_OK;
    diag->status1_status = YHM2712_ACMD_STATUS_NOT_INIT;
    diag->status2_status = YHM2712_ACMD_STATUS_NOT_INIT;
    diag->window_status = YHM2712_ACMD_STATUS_NOT_INIT;
    diag->stacmd_level_before = 0U;
    diag->stacmd_level_after_restore = 0U;
    diag->id = 0U;
    diag->id_attempts = 0U;
    diag->id_stable_count = 0U;
    diag->status1 = 0U;
    diag->status2 = 0U;
    diag->fsm_mode = 0U;
    diag->window_count = YHM2712_ACMD_BOOT_REG_COUNT;
    diag->selftest_ok = false;
    diag->id_valid = false;
    diag->id_ok = false;
    diag->status1_valid = false;
    diag->status2_valid = false;
    diag->window_all_valid = false;
    diag->comm_ok = false;
    for (idx = 0U; idx < YHM2712_ACMD_BOOT_REG_COUNT; idx++)
    {
        diag->window[idx].reg = s_yhm2712_acmd_boot_regs[idx];
        diag->window[idx].status = YHM2712_ACMD_STATUS_NOT_INIT;
        diag->window[idx].value = 0U;
        diag->window[idx].valid = false;
    }

    yhm2712_acmd_refresh_timing();
    if (!yhm2712_acmd_tick_running())
    {
        diag->result = YHM2712_ACMD_STATUS_TICK_STUCK;
        diag->selftest_status = YHM2712_ACMD_STATUS_TICK_STUCK;
#if ZY100_YHM_HAS_MOS || YHM_PRODUCTION_COMPACT
        yhm_mos_preflight_error_log(YHM2712_REG_ID, diag->selftest_status);
#endif
        return diag->result;
    }

    status = yhm2712_acmd_release_selftest(&selftest_trace);
    diag->selftest_status = status;
#if ZY100_YHM_HAS_MOS || YHM_PRODUCTION_COMPACT
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        yhm_mos_preflight_error_log(YHM2712_REG_ID, status);
    }
#endif
    diag->selftest_ok = (status == YHM2712_ACMD_STATUS_OK);
    diag->stacmd_level_before = selftest_trace.level_before_prepare;
    diag->stacmd_level_after_restore = selftest_trace.level_after_restore;
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        diag->result = status;
        diag->window_status = status;
        return status;
    }

    status = yhm2712_acmd_read_boot_id_stable(&diag->id,
                                              &diag->id_attempts,
                                              &diag->id_stable_count);
    diag->id_status = status;
    diag->id_valid = (status == YHM2712_ACMD_STATUS_OK);
    diag->id_ok = diag->id_valid && (diag->id == YHM2712_ID_DEFAULT);
    diag->window[0].reg = YHM2712_REG_ID;
    diag->window[0].status = status;
    diag->window[0].value = diag->id;
    diag->window[0].valid = diag->id_valid;
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        diag->result = status;
        diag->window_status = status;
        return status;
    }

    value = 0U;
    status = yhm2712_acmd_read_reg(YHM2712_REG_STATUS1, &value);
    diag->status1_status = status;
    diag->status1 = value;
    diag->status1_valid = (status == YHM2712_ACMD_STATUS_OK);
    diag->window[1].reg = YHM2712_REG_STATUS1;
    diag->window[1].status = status;
    diag->window[1].value = value;
    diag->window[1].valid = diag->status1_valid;
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        diag->result = status;
        diag->window_status = status;
        return status;
    }

    value = 0U;
    status = yhm2712_acmd_read_reg(YHM2712_REG_STATUS2, &value);
    diag->status2_status = status;
    diag->status2 = value;
    diag->status2_valid = (status == YHM2712_ACMD_STATUS_OK);
    diag->window[2].reg = YHM2712_REG_STATUS2;
    diag->window[2].status = status;
    diag->window[2].value = value;
    diag->window[2].valid = diag->status2_valid;
    if (diag->status2_valid)
    {
        diag->fsm_mode = (uint8_t)((value & YHM2712_STATUS2_FSM_MODE_MASK) >>
                                   YHM2712_STATUS2_FSM_MODE_SHIFT);
    }
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        diag->result = status;
        diag->window_status = status;
        return status;
    }

    diag->window_all_valid = diag->id_valid && diag->status1_valid && diag->status2_valid;
    diag->window_status = diag->window_all_valid ?
                          YHM2712_ACMD_STATUS_OK : YHM2712_ACMD_STATUS_ID_MISMATCH;
    diag->comm_ok = diag->selftest_ok && diag->id_ok && diag->window_all_valid;
    diag->result = diag->comm_ok ? YHM2712_ACMD_STATUS_OK : diag->window_status;
    return diag->result;
}

const char *yhm2712_acmd_status_name(yhm2712_acmd_status_t status)
{
    switch (status)
    {
    case YHM2712_ACMD_STATUS_OK:
        return "ok";
    case YHM2712_ACMD_STATUS_NOT_INIT:
        return "not_init";
    case YHM2712_ACMD_STATUS_BAD_PARAM:
        return "bad_param";
    case YHM2712_ACMD_STATUS_BAD_PIN:
        return "bad_pin";
    case YHM2712_ACMD_STATUS_DWT_UNAVAILABLE:
        return "dwt_unavailable";
    case YHM2712_ACMD_STATUS_TICK_STUCK:
        return "tick_stuck";
    case YHM2712_ACMD_STATUS_LINE_BUSY_LOW:
        return "line_busy_low";
    case YHM2712_ACMD_STATUS_TIMEOUT_FALL:
        return "timeout_fall";
    case YHM2712_ACMD_STATUS_TIMEOUT_RISE:
        return "timeout_rise";
    case YHM2712_ACMD_STATUS_BAD_SYMBOL:
        return "bad_symbol";
    case YHM2712_ACMD_STATUS_ACK_MISMATCH:
        return "ack_mismatch";
    case YHM2712_ACMD_STATUS_ID_MISMATCH:
        return "id_mismatch";
    case YHM2712_ACMD_STATUS_RELEASE_TIMEOUT:
        return "release_timeout";
    case YHM2712_ACMD_STATUS_RX_ENTRY_LOW:
        return "rx_entry_low";
    case YHM2712_ACMD_STATUS_BUSY:
        return "busy";
    case YHM2712_ACMD_STATUS_SELFTEST_LINE_LOW:
        return "selftest_line_low";
    case YHM2712_ACMD_STATUS_SELFTEST_RELEASE_INITIAL_FAIL:
        return "selftest_release_initial_fail";
    case YHM2712_ACMD_STATUS_SELFTEST_DRIVE_LOW_FAIL:
        return "selftest_drive_low_fail";
    case YHM2712_ACMD_STATUS_SELFTEST_RELEASE_AFTER_FAIL:
        return "selftest_release_after_fail";
    case YHM2712_ACMD_STATUS_MODE_MISMATCH:
        return "mode_mismatch";
    case YHM2712_ACMD_STATUS_READBACK_MISMATCH:
        return "readback_mismatch";
    default:
        return "unknown";
    }
}

const char *yhm2712_acmd_fsm_mode_name(uint8_t fsm_mode)
{
    switch (fsm_mode)
    {
    case 0x0U:
        return "RESET";
    case 0x1U:
        return "SHIPPING";
    case 0x2U:
        return "SLEEP";
    case 0x3U:
        return "ITEST";
    case 0x8U:
        return "DISCHARGE";
    case 0x9U:
        return "FAULT";
    case 0xAU:
        return "START";
    case 0xBU:
        return "SYS_PRE";
    case 0xCU:
        return "CHARGE";
    case 0xDU:
        return "CHARGE_DONE";
    case 0xFU:
        return "STOP_CHARGE";
    default:
        return "RESERVED";
    }
}

#if ZY100_YHM_HAS_MOS
bool yhm_mos_ack_timing_get(yhm_pressure_diag_t *out)
{
    if ((out == NULL) || s_yhm2712_acmd.busy || !s_yhm_pressure_last_ack.valid)
    {
        return false;
    }
    *out = (yhm_pressure_diag_t){0};
    out->valid = true;
    out->ack = s_yhm_pressure_last_ack;
    out->ack_start = s_yhm_pressure_diag_work.ack_start;
    out->ack_fall = s_yhm_pressure_diag_work.ack_fall;
    out->ack_rise = s_yhm_pressure_diag_work.ack_rise;
    out->ack_entry_low = s_yhm_pressure_diag_work.ack_entry_low;
    return true;
}

bool yhm_pressure_ack_sample_get(yhm_pressure_ack_sample_t *sample)
{
    if ((sample == NULL) || s_yhm2712_acmd.busy || !s_yhm_pressure_last_ack.valid)
    {
        return false;
    }
    *sample = s_yhm_pressure_last_ack;
    return true;
}
#else
/* Legacy capture keeps its original low-phase work; no new instrumentation. */
bool yhm_pressure_ack_sample_get(yhm_pressure_ack_sample_t *sample)
{
    (void)sample;
    return false;
}
bool yhm_pressure_rx_snapshot_get(bool first_id_error,
                                 yhm_pressure_rx_snapshot_t *snapshot)
{
    (void)first_id_error;
    (void)snapshot;
    return false;
}
#endif
