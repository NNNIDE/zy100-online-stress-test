#include "../service/power_diag.h"
#include "icm53611_driver.h"

#include <stddef.h>
#include <string.h>

#include "../app_flags.h"
#include "../zy100_clock_config.h"
#include "../bsp/imu_bsp.h"
#include "icm53611_reg.h"
#include "icm53611_spi.h"
#include "rtl876x_spi.h"

#define ICM53611_PWR_READY_WAIT_US      1000U
#define ICM53611_RESET_SETTLE_WAIT_US   2000U
#define ICM53611_POWER_ON_SETTLE_US     5000U
#define ICM53611_FIFO_FLUSH_WAIT_US     10U /* TDK inv_imu_flush_fifo(), DS p50 >=1.5us */
#define ICM53611_FIFO_FLUSH_POLL_US     2U
#define ICM53611_FIFO_FLUSH_TIMEOUT_US  1000U
/* Capture-start policy: board 10581 cleared in 45..51us (five samples).
 * 10ms is a conservative software limit, not an ICM guaranteed latency.
 * Keep UI capture flush independent of the runtime MCLK-lease helper. */
#define ICM53611_CAPTURE_FLUSH_WAIT_US       2U
#define ICM53611_CAPTURE_FLUSH_POLL_US       2U
#define ICM53611_CAPTURE_FLUSH_TIMEOUT_US    10000U
#define ICM53611_CAPTURE_FLUSH_OS_GUARD_MS   20U /* RTOS tick is 10ms. */
#define ICM53611_CAPTURE_FLUSH_MAX_POLLS     5000U
#define ICM53611_OIS_TMST_RAW_WRAP      0x10000ULL
#define ICM53611_OIS_TMST_RES_1US       1ULL
#define ICM53611_OIS_TMST_HIGH_IDX      14U
#define ICM53611_OIS_TMST_LOW_IDX       15U
#define ICM53611_ENABLE_DRDY_INT1       0U
#define ICM53611_MREG1_TMST_CONFIG1_V0_VERIFY_MASK \
    (ICM53611_MREG1_TMST_CONFIG1_TMST_EN | \
     ICM53611_MREG1_TMST_CONFIG1_TMST_DELTA_EN | \
     ICM53611_MREG1_TMST_CONFIG1_TMST_RES)
#define ICM53611_MREG1_FIFO_CONFIG5_V0_VERIFY_MASK \
    (ICM53611_MREG1_FIFO_CONFIG5_FIFO_ACCEL_EN | \
     ICM53611_MREG1_FIFO_CONFIG5_FIFO_GYRO_EN | \
     ICM53611_MREG1_FIFO_CONFIG5_FIFO_TMST_FSYNC_EN | \
     ICM53611_MREG1_FIFO_CONFIG5_FIFO_HIRES_EN | \
     ICM53611_MREG1_FIFO_CONFIG5_FIFO_RESUME_PARTIAL_RD)
#define ICM53611_MREG1_SENSOR_CONFIG3_V0_VERIFY_MASK \
    (ICM53611_MREG1_SENSOR_CONFIG3_OIS_CONFIG_DISABLE)
#define ICM53611_MREG1_TMST_CONFIG1_PACKET3_VERIFY_MASK \
    (ICM53611_MREG1_TMST_CONFIG1_V0_VERIFY_MASK | \
     ICM53611_MREG1_TMST_CONFIG1_TMST_ON_SREG_EN | \
     ICM53611_MREG1_TMST_CONFIG1_TMST_FSYNC_EN)
#define ICM53611_UI_FIFO_GYRO_CONFIG0_1600HZ \
    (ICM53611_GYRO_FSR_2000DPS | ICM53611_GYRO_ODR_1600HZ)
#define ICM53611_UI_FIFO_ACCEL_CONFIG0_1600HZ \
    (ICM53611_ACCEL_FSR_16G | ICM53611_ACCEL_ODR_1600HZ)
#define ICM53611_UI_FIFO_GYRO_CONFIG0_800HZ \
    (ICM53611_GYRO_FSR_2000DPS | ICM53611_GYRO_ODR_800HZ)
#define ICM53611_UI_FIFO_ACCEL_CONFIG0_800HZ \
    (ICM53611_ACCEL_FSR_16G | ICM53611_ACCEL_ODR_800HZ)
#define ICM53611_WOM_DLPS_MCLK_READY_TIMEOUT_US 20000U
#define ICM53611_WOM_DLPS_MCLK_READY_POLL_US      100U
#define ICM53611_WOM_DLPS_ODR_25HZ_PERIOD_US    40000U
#define ICM53611_WOM_DLPS_ODR_50HZ_PERIOD_US    20000U
#define ICM53611_WOM_DLPS_ODR_100HZ_PERIOD_US   10000U
#define ICM53611_WOM_DLPS_ODR_800HZ_PERIOD_US    1250U
#define ICM53611_WOM_DLPS_ODR_1600HZ_PERIOD_US    625U

static imu_status_t icm53611_wom_dlps_exit_wuosc_if_needed(uint8_t pwr, uint8_t odr);
static imu_status_t icm53611_wom_stop_clocked(void);
static imu_status_t icm53611_wom_ui_write(uint8_t reg, uint8_t value);
static imu_status_t icm53611_wom_dlps_exit_wuosc_if_needed_minimal(uint8_t pwr_mgmt0, uint8_t accel_config0);
static imu_status_t icm53611_wom_mreg_bits_minimal(uint8_t reg, uint8_t mask, uint8_t bits);
static imu_status_t icm53611_wom_stop_clocked_minimal(void);
static imu_status_t icm53611_wom_dlps_disable_fifo_tmst(void);
static imu_status_t icm53611_wom_mreg_bits(uint8_t reg, uint8_t mask, uint8_t bits);

static const icm53611_cfg_t s_icm53611_default_cfg =
{
    .pwr_mgmt0 = (ICM53611_PWR_MGMT0_ACCEL_MODE_LN |
                  ICM53611_PWR_MGMT0_GYRO_MODE_LN |
                  ICM53611_PWR_MGMT0_ACCEL_OIS_EN |
                  ICM53611_PWR_MGMT0_GYRO_OIS_EN),
    .gyro_config0 = ICM53611_GYRO_CONFIG0_DEFAULT,
    .accel_config0 = ICM53611_ACCEL_CONFIG0_DEFAULT,
    .gyro_config1 = ICM53611_GYRO_CONFIG1_DEFAULT,
    .accel_config1 = ICM53611_ACCEL_CONFIG1_DEFAULT,
};

static bool s_icm53611_ois_tmst_inited = false;
static uint16_t s_icm53611_ois_tmst_last = 0U;
static uint64_t s_icm53611_ois_tmst_wrap_base = 0ULL;
static bool s_icm53611_probe_log_valid = false;
static uint32_t s_icm53611_probe_log_last_ms = 0U;
static uint8_t s_icm53611_probe_log_who = 0U;
static imu_status_t s_icm53611_probe_log_status = IMU_STATUS_OK;
static bool s_icm53611_start_log_valid = false;
static uint32_t s_icm53611_start_log_last_ms = 0U;
static uint32_t s_icm53611_start_log_step = 0U;
static int32_t s_icm53611_start_log_ret = 0;
static icm53611_mreg_verify_error_t s_icm53611_last_mreg_verify_error = {0};

static uint32_t icm53611_diff_u64_to_u32(uint64_t end_us, uint64_t start_us)
{
    uint64_t delta;

    if (end_us < start_us)
    {
        return 0U;
    }

    delta = end_us - start_us;
    return (delta > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)delta;
}

typedef struct
{
    const char *axis_name;
    const char *reg_name;
    uint8_t addr;
    uint8_t write_value;
    uint8_t same_read;
    uint8_t normal_read;
    imu_status_t same_status;
    imu_status_t normal_status;
    bool same_pass;
    bool normal_pass;
} icm53611_wom_thr_verify_result_t;

static void icm53611_log_probe_if_due(uint8_t who_am_i, imu_status_t status)
{
    uint32_t now_ms = ((uint32_t)imu_bsp_local_timestamp_us()) / 1000U;

    if (s_icm53611_probe_log_valid &&
        (s_icm53611_probe_log_who == who_am_i) &&
        (s_icm53611_probe_log_status == status) &&
        ((uint32_t)(now_ms - s_icm53611_probe_log_last_ms) < 1000U))
    {
        return;
    }

    s_icm53611_probe_log_valid = true;
    s_icm53611_probe_log_last_ms = now_ms;
    s_icm53611_probe_log_who = who_am_i;
    s_icm53611_probe_log_status = status;
    if ((status != IMU_STATUS_OK) || IMU_RUNTIME_VERBOSE_LOG_ENABLE)
    {
        DBG_DIRECT("[IMU_PROBE] who=0x%02x st=%d", who_am_i, status);
    }
}

static void icm53611_log_start_if_due(uint32_t step, int32_t ret)
{
    uint32_t now_ms = ((uint32_t)imu_bsp_local_timestamp_us()) / 1000U;

    if (s_icm53611_start_log_valid &&
        (s_icm53611_start_log_step == step) &&
        (s_icm53611_start_log_ret == ret) &&
        ((uint32_t)(now_ms - s_icm53611_start_log_last_ms) < 1000U))
    {
        return;
    }

    s_icm53611_start_log_valid = true;
    s_icm53611_start_log_last_ms = now_ms;
    s_icm53611_start_log_step = step;
    s_icm53611_start_log_ret = ret;
    if ((ret != 0) || IMU_RUNTIME_VERBOSE_LOG_ENABLE)
    {
        DBG_DIRECT("[IMU_START] step=%u ret=%d", step, ret);
    }
}

static uint16_t icm53611_be_to_u16(const uint8_t high, const uint8_t low)
{
    return (uint16_t)(((uint16_t)high << 8) | low);
}

static int16_t icm53611_be_to_s16(const uint8_t high, const uint8_t low)
{
    return (int16_t)icm53611_be_to_u16(high, low);
}

static int32_t icm53611_sign_extend20(uint32_t u20)
{
    u20 &= 0xFFFFFU;
    return ((u20 & 0x80000U) != 0U) ?
           (int32_t)(u20 | 0xFFF00000U) : (int32_t)u20;
}

static uint16_t icm53611_get_ois_tmst_raw(const uint8_t *raw)
{
    return (uint16_t)(((uint16_t)raw[ICM53611_OIS_TMST_HIGH_IDX] << 8) |
                      raw[ICM53611_OIS_TMST_LOW_IDX]);
}

static void icm53611_reset_ois_tmst_tracker(void)
{
    s_icm53611_ois_tmst_inited = false;
    s_icm53611_ois_tmst_last = 0U;
    s_icm53611_ois_tmst_wrap_base = 0ULL;
}

static uint64_t icm53611_expand_ois_tmst_us(uint16_t tmst_raw)
{
    if (!s_icm53611_ois_tmst_inited)
    {
        s_icm53611_ois_tmst_inited = true;
        s_icm53611_ois_tmst_last = tmst_raw;
        return (uint64_t)tmst_raw * ICM53611_OIS_TMST_RES_1US;
    }

    if (tmst_raw < s_icm53611_ois_tmst_last)
    {
        s_icm53611_ois_tmst_wrap_base += ICM53611_OIS_TMST_RAW_WRAP;
    }

    s_icm53611_ois_tmst_last = tmst_raw;
    return (s_icm53611_ois_tmst_wrap_base + (uint64_t)tmst_raw) * ICM53611_OIS_TMST_RES_1US;
}

static uint8_t icm53611_ois_timestamp_desired_value(uint8_t tmst_cfg1)
{
    tmst_cfg1 |= (ICM53611_MREG1_TMST_CONFIG1_TMST_ON_SREG_EN |
                  ICM53611_MREG1_TMST_CONFIG1_TMST_EN |
                  ICM53611_MREG1_TMST_CONFIG1_TMST_FSYNC_EN);
    tmst_cfg1 &= (uint8_t)~(ICM53611_MREG1_TMST_CONFIG1_TMST_DELTA_EN |
                            ICM53611_MREG1_TMST_CONFIG1_TMST_RES);
    return tmst_cfg1;
}

static imu_status_t icm53611_enable_ois_timestamp_regs(bool *skip_out)
{
    imu_status_t status;
    uint8_t tmst_cfg1 = 0U;
    uint8_t desired;

    if (skip_out != NULL)
    {
        *skip_out = false;
    }

    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1, ICM53611_MREG1_REG_TMST_CONFIG1, &tmst_cfg1);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    /*
     * Keep OIS timestamp output deterministic for offline parse:
     * - TMST_EN=1 enables timestamp register update
     * - TMST_ON_SREG_EN=1 enables absolute timestamp on status/OIS register path
     * - TMST_DELTA_EN=0 forces absolute mode (no ODR-delta)
     * - TMST_RES=0 keeps 1us resolution
     */
    desired = icm53611_ois_timestamp_desired_value(tmst_cfg1);
    if (tmst_cfg1 == desired)
    {
        if (skip_out != NULL)
        {
            *skip_out = true;
        }
        return IMU_STATUS_OK;
    }

    return imu_spi_mreg_write(ICM53611_MREG_BLOCK_1, ICM53611_MREG1_REG_TMST_CONFIG1, desired);
}

static imu_status_t icm53611_configure_drdy_int1(void)
{
    imu_status_t status;
    uint8_t int_config = 0U;
    uint8_t int_source0 = 0U;

    status = imu_spi_read_reg(ICM53611_REG_INT_CONFIG, &int_config);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    /* Keep INT2 bits untouched; control INT1 mode only when DRDY routing is enabled. */
#if ICM53611_ENABLE_DRDY_INT1
    int_config &= (uint8_t)(~ICM53611_INT_CONFIG_INT1_MODE);
    int_config |= (ICM53611_INT_CONFIG_INT1_DRIVE_CIRCUIT |
                   ICM53611_INT_CONFIG_INT1_POLARITY);
#else
    int_config &= (uint8_t)(~(ICM53611_INT_CONFIG_INT1_MODE |
                              ICM53611_INT_CONFIG_INT1_DRIVE_CIRCUIT |
                              ICM53611_INT_CONFIG_INT1_POLARITY));
#endif
    status = icm53611_write_with_verify(ICM53611_REG_INT_CONFIG,
                                        int_config,
                                        ICM53611_INT_CONFIG_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = imu_spi_read_reg(ICM53611_REG_INT_SOURCE0, &int_source0);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    /* Route UI Data Ready interrupt to INT1 only when requested. */
#if ICM53611_ENABLE_DRDY_INT1
    int_source0 |= ICM53611_INT_SOURCE0_DRDY_INT1_EN;
#else
    int_source0 &= (uint8_t)(~ICM53611_INT_SOURCE0_DRDY_INT1_EN);
#endif
    return icm53611_write_with_verify(ICM53611_REG_INT_SOURCE0,
                                      int_source0,
                                      ICM53611_INT_SOURCE0_VERIFY_MASK);
}

static void icm53611_fill_ois_6axis_sample(const uint8_t *raw,
                                           uint64_t tmst_us,
                                           icm53611_raw_sample_t *sample)
{
    memcpy(sample->ois_raw, raw, ICM53611_OIS_RAW_FRAME_BYTES);
    sample->temp_raw = icm53611_be_to_s16(raw[0], raw[1]);
    sample->accel_x = icm53611_be_to_s16(raw[2], raw[3]);
    sample->accel_y = icm53611_be_to_s16(raw[4], raw[5]);
    sample->accel_z = icm53611_be_to_s16(raw[6], raw[7]);
    sample->gyro_x = icm53611_be_to_s16(raw[8], raw[9]);
    sample->gyro_y = icm53611_be_to_s16(raw[10], raw[11]);
    sample->gyro_z = icm53611_be_to_s16(raw[12], raw[13]);
    sample->ext_data_x = raw[16];
    sample->ext_data_y = raw[17];
    sample->ext_data_z = raw[18];
    /*
     * TODO(ZY100): map real OIS status register/bit definition once
     * datasheet-backed semantics are finalized.
     */
    sample->ois_status = 0U;
    sample->local_ts_us = tmst_us;
}

static void icm53611_fill_ui_6axis_sample(const uint8_t *raw,
                                          uint64_t read_ts_us,
                                          icm53611_raw_sample_t *sample)
{
    sample->temp_raw = 0;
    sample->accel_x = icm53611_be_to_s16(raw[0], raw[1]);
    sample->accel_y = icm53611_be_to_s16(raw[2], raw[3]);
    sample->accel_z = icm53611_be_to_s16(raw[4], raw[5]);
    sample->gyro_x = icm53611_be_to_s16(raw[6], raw[7]);
    sample->gyro_y = icm53611_be_to_s16(raw[8], raw[9]);
    sample->gyro_z = icm53611_be_to_s16(raw[10], raw[11]);
    sample->ois_status = 0U;
    sample->ext_data_x = 0U;
    sample->ext_data_y = 0U;
    sample->ext_data_z = 0U;
    memset(sample->ois_raw, 0, sizeof(sample->ois_raw));
    sample->local_ts_us = read_ts_us;
}

static imu_status_t icm53611_read_ois_6axis_sample_common(icm53611_raw_sample_t *sample,
                                                           bool isr_fast_path)
{
    uint8_t raw[ICM53611_OIS_RAW_FRAME_BYTES];
    uint16_t tmst_raw;
    imu_status_t status;

    if (sample == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    if (isr_fast_path)
    {
        status = imu_spi_read_regs_isr_fast(ICM53611_REG_TEMP_DATA1_OIS, raw, sizeof(raw));
    }
    else
    {
        status = imu_spi_read_regs(ICM53611_REG_TEMP_DATA1_OIS, raw, sizeof(raw));
    }
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    tmst_raw = icm53611_get_ois_tmst_raw(raw);
    icm53611_fill_ois_6axis_sample(raw, icm53611_expand_ois_tmst_us(tmst_raw), sample);
    return IMU_STATUS_OK;
}

imu_status_t icm53611_read_ois_raw_frame_isr_fast_locked(
    uint8_t raw[ICM53611_OIS_RAW_FRAME_BYTES])
{
    if (raw == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    return imu_spi_read_regs_isr_fast_locked(ICM53611_REG_TEMP_DATA1_OIS,
                                             raw,
                                             ICM53611_OIS_RAW_FRAME_BYTES);
}

imu_status_t icm53611_read_ois_tmst_raw_isr_fast_locked(
    uint16_t *tmst_raw_out)
{
    uint8_t raw[2];
    imu_status_t status;

    if (tmst_raw_out == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = imu_spi_read_regs_isr_fast_locked(ICM53611_REG_TMST_FSYNCH_OIS,
                                               raw,
                                               sizeof(raw));
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    *tmst_raw_out = icm53611_be_to_u16(raw[0], raw[1]);
    return IMU_STATUS_OK;
}

imu_status_t icm53611_decode_ois_raw_frame(
    const uint8_t raw[ICM53611_OIS_RAW_FRAME_BYTES],
    icm53611_raw_sample_t *sample)
{
    if ((raw == NULL) || (sample == NULL))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    icm53611_fill_ois_6axis_sample(raw,
                                   (uint64_t)icm53611_get_ois_tmst_raw(raw),
                                   sample);
    return IMU_STATUS_OK;
}

imu_status_t icm53611_decode_ois_raw_accel16(
    const uint8_t raw[ICM53611_OIS_RAW_FRAME_BYTES],
    int16_t *ax,
    int16_t *ay,
    int16_t *az)
{
    if ((raw == NULL) || (ax == NULL) || (ay == NULL) || (az == NULL))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    *ax = icm53611_be_to_s16(raw[2], raw[3]);
    *ay = icm53611_be_to_s16(raw[4], raw[5]);
    *az = icm53611_be_to_s16(raw[6], raw[7]);
    return IMU_STATUS_OK;
}

bool icm53611_decode_ois20_accel_raw(
    const uint8_t raw[ICM53611_OIS_RAW_FRAME_BYTES],
    int32_t *ax,
    int32_t *ay,
    int32_t *az)
{
    uint16_t main16;
    uint32_t u20;

    if ((raw == NULL) || (ax == NULL) || (ay == NULL) || (az == NULL))
    {
        return false;
    }

    main16 = icm53611_be_to_u16(raw[2], raw[3]);
    u20 = ((uint32_t)main16 << 4) | ((uint32_t)((raw[16] >> 4) & 0x0FU));
    *ax = icm53611_sign_extend20(u20);

    main16 = icm53611_be_to_u16(raw[4], raw[5]);
    u20 = ((uint32_t)main16 << 4) | ((uint32_t)((raw[17] >> 4) & 0x0FU));
    *ay = icm53611_sign_extend20(u20);

    main16 = icm53611_be_to_u16(raw[6], raw[7]);
    u20 = ((uint32_t)main16 << 4) | ((uint32_t)((raw[18] >> 4) & 0x0FU));
    *az = icm53611_sign_extend20(u20);

    return true;
}

imu_status_t icm53611_read_ui_6axis_sample(icm53611_raw_sample_t *sample)
{
    uint8_t raw[12];
    imu_status_t status;

    if (sample == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = imu_spi_read_regs(ICM53611_REG_ACCEL_DATA_X1, raw, sizeof(raw));
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    icm53611_fill_ui_6axis_sample(raw, imu_bsp_local_timestamp_us(), sample);
    return IMU_STATUS_OK;
}

static void icm53611_log_line_levels(const char *tag)
{
    imu_status_t cs_status;
    imu_status_t pwr_status;
    uint8_t cs_out = 0U;
    uint8_t cs_in = 0U;
    uint8_t pwr_out = 0U;
    uint8_t pwr_in = 0U;
    const char *use_tag = (tag != NULL) ? tag : "n/a";

    cs_status = imu_bsp_get_cs_level(&cs_out, &cs_in);
    pwr_status = imu_bsp_get_power_level(&pwr_out, &pwr_in);

    if ((cs_status == IMU_STATUS_OK) && (pwr_status == IMU_STATUS_OK))
    {
        IMU_LOG_INFO("line diag(%s): cs(out/in)=%u/%u pwr(out/in)=%u/%u",
                     use_tag, cs_out, cs_in, pwr_out, pwr_in);
        return;
    }

    if (cs_status == IMU_STATUS_OK)
    {
        IMU_LOG_INFO("line diag(%s): cs(out/in)=%u/%u pwr(status=%d)",
                     use_tag, cs_out, cs_in, pwr_status);
        return;
    }

    if (pwr_status == IMU_STATUS_OK)
    {
        IMU_LOG_INFO("line diag(%s): cs(status=%d) pwr(out/in)=%u/%u",
                     use_tag, cs_status, pwr_out, pwr_in);
        return;
    }

    IMU_LOG_WARN("line %s cs=%d pwr=%d",
                 use_tag, cs_status, pwr_status);
}

static imu_status_t icm53611_read_whoami_mode0(uint8_t *who_am_i_out)
{
    imu_status_t status;
    uint8_t who_am_i = 0U;

    if (who_am_i_out == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = imu_bsp_spi_set_mode(SPI_CPOL_Low, SPI_CPHA_1Edge);
    if (status != IMU_STATUS_OK)
    {
        IMU_LOG_WARN("SPI mode0 set failed, status=%d", status);
        return status;
    }

    status = icm53611_read_whoami(&who_am_i);
    if (status != IMU_STATUS_OK)
    {
        IMU_LOG_WARN("SPI mode0 WHO read st=%d", status);
        return status;
    }

    *who_am_i_out = who_am_i;
    if (who_am_i == ICM53611_WHO_AM_I_VALUE)
    {
        IMU_LOG_INFO("WHO_AM_I matched on SPI mode0: 0x%02x", who_am_i);
        return IMU_STATUS_OK;
    }

    IMU_LOG_WARN("WHO mode0=%02x exp=%02x",
                 who_am_i, ICM53611_WHO_AM_I_VALUE);
    return IMU_STATUS_NOT_FOUND;
}

const icm53611_cfg_t *icm53611_default_config(void)
{
    return &s_icm53611_default_cfg;
}

imu_status_t icm53611_init_bus(void)
{
    imu_status_t status;

    status = imu_bsp_init();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = imu_spi_bus_init();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = imu_bsp_int_init();
    return status;
}

imu_status_t icm53611_read_whoami(uint8_t *who_am_i)
{
    return imu_spi_read_reg(ICM53611_REG_WHO_AM_I, who_am_i);
}

bool icm53611_runtime_resume_probe(uint8_t *who_out, imu_status_t *status_out)
{
    imu_status_t status;
    uint8_t who_am_i = 0U;
    uint32_t fail_step = 0U;

    status = imu_bsp_resume_after_dlps();
    if (status != IMU_STATUS_OK)
    {
        fail_step = 1U;
        goto out;
    }

    status = imu_bsp_power_ctrl(true);
    if ((status != IMU_STATUS_OK) && (status != IMU_STATUS_UNSUPPORTED))
    {
        fail_step = 2U;
        goto out;
    }

    imu_bsp_delay_us(ICM53611_POWER_ON_SETTLE_US);

    status = imu_bsp_spi_set_mode(SPI_CPOL_Low, SPI_CPHA_1Edge);
    if (status != IMU_STATUS_OK)
    {
        fail_step = 3U;
        goto out;
    }

    status = icm53611_read_whoami(&who_am_i);
    if (status != IMU_STATUS_OK)
    {
        fail_step = 3U;
        goto out;
    }

    if (who_am_i != ICM53611_WHO_AM_I_VALUE)
    {
        status = IMU_STATUS_NOT_FOUND;
        fail_step = 3U;
    }

out:
    if (who_out != NULL)
    {
        *who_out = who_am_i;
    }
    if (status_out != NULL)
    {
        *status_out = status;
    }
    icm53611_log_probe_if_due(who_am_i, status);
    if ((status != IMU_STATUS_OK) || (who_am_i != ICM53611_WHO_AM_I_VALUE))
    {
        if (fail_step == 0U)
        {
            fail_step = 3U;
        }
        icm53611_log_start_if_due(fail_step, (int32_t)status);
    }
    return ((status == IMU_STATUS_OK) && (who_am_i == ICM53611_WHO_AM_I_VALUE));
}

imu_status_t icm53611_soft_reset(void)
{
    imu_status_t status;

    status = imu_spi_write_reg(ICM53611_REG_SIGNAL_PATH_RESET,
                               ICM53611_SIGNAL_PATH_RESET_SOFT_RESET_DEVICE_CONFIG);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    imu_bsp_delay_us(ICM53611_RESET_SETTLE_WAIT_US);
    return IMU_STATUS_OK;
}

#if ZY100_POWER_DIAG_ENABLE
static void icm53611_pwrd_read(uint8_t reg, uint8_t value, imu_status_t status)
{
    /* Successful register scans were measurement-only. First errors use pwrd_verify/fault. */
    (void)reg; (void)value; (void)status;
}
#else
#define icm53611_pwrd_read(...) ((void)0)
#endif

static imu_status_t icm53611_write_with_verify_impl(
    uint8_t reg, uint8_t value, uint8_t verify_mask, bool report_busy)
{
    imu_status_t status;
    uint8_t readback = 0U;

    status = report_busy ?
        imu_spi_write_reg_ex(reg, value) :
        imu_spi_write_reg(reg, value);
    if (status != IMU_STATUS_OK)
    {
        pwrd_verify(0U, 0U, reg, readback, verify_mask, value, (uint8_t)status);
        return status;
    }

    /* DS pp59-60: protect OFF->ON writes before returning to any caller. */
    if (reg == ICM53611_REG_PWR_MGMT0) imu_bsp_delay_us(200U);

    status = report_busy ?
        imu_spi_read_reg_ex(reg, &readback) :
        imu_spi_read_reg(reg, &readback);
    icm53611_pwrd_read(reg, readback, status);
    pwrd_verify(0U, 0U, reg, readback, verify_mask, value, (uint8_t)status);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    if ((readback & verify_mask) != (value & verify_mask))
    {
        imu_spi_error_record(3U, 0U, reg, readback, true, IMU_STATUS_VERIFY_FAILED);
        IMU_LOG_ERROR("verify r=%02x w=%02x got=%02x m=%02x",
                      reg, value, readback, verify_mask);
        return IMU_STATUS_VERIFY_FAILED;
    }

    return IMU_STATUS_OK;
}

imu_status_t icm53611_write_with_verify(uint8_t reg, uint8_t value, uint8_t verify_mask)
{
    return icm53611_write_with_verify_impl(reg, value, verify_mask, false);
}

static imu_status_t icm53611_update_reg_bits(uint8_t reg, uint8_t mask, uint8_t value)
{
    imu_status_t status;
    uint8_t reg_value = 0U;

    status = imu_spi_read_reg(reg, &reg_value);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    reg_value = (uint8_t)((reg_value & (uint8_t)(~mask)) | (value & mask));
    return icm53611_write_with_verify(reg, reg_value, mask);
}

static imu_status_t icm53611_update_reg_bits_readback(uint8_t reg,
                                                      uint8_t mask,
                                                      uint8_t value,
                                                      uint8_t *write_value_out,
                                                      uint8_t *readback_out)
{
    imu_status_t status;
    uint8_t reg_value = 0U;
    uint8_t readback = 0U;

    status = imu_spi_read_reg(reg, &reg_value);
    if (status != IMU_STATUS_OK)
    {
        if (write_value_out != NULL)
        {
            *write_value_out = (uint8_t)(value & mask);
        }
        if (readback_out != NULL)
        {
            *readback_out = readback;
        }
        return status;
    }

    reg_value = (uint8_t)((reg_value & (uint8_t)(~mask)) | (value & mask));
    if (write_value_out != NULL)
    {
        *write_value_out = reg_value;
    }

    status = imu_spi_write_reg(reg, reg_value);
    if (status != IMU_STATUS_OK)
    {
        if (readback_out != NULL)
        {
            *readback_out = readback;
        }
        return status;
    }

    status = imu_spi_read_reg(reg, &readback);
    if (readback_out != NULL)
    {
        *readback_out = readback;
    }
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    if ((readback & mask) != (reg_value & mask))
    {
        return IMU_STATUS_VERIFY_FAILED;
    }

    return IMU_STATUS_OK;
}

static const char *icm53611_mreg_bank_name(uint8_t bank)
{
    if (bank == ICM53611_MREG_BLOCK_1)
    {
        return "MREG1";
    }
    if (bank == ICM53611_MREG_BLOCK_2)
    {
        return "MREG2";
    }
    return "MREG?";
}

static const char *icm53611_pass_fail(bool pass)
{
    return pass ? "PASS" : "FAIL";
}

static void icm53611_log_mreg_verify_error(uint8_t bank,
                                           const char *reg_name,
                                           uint8_t maddr,
                                           uint8_t write_value,
                                           uint8_t read_value,
                                           uint8_t expected_mask,
                                           uint8_t expected_value,
                                           imu_status_t status,
                                           uint8_t retry_count)
{
    imu_spi_error_record(3U, bank, maddr, read_value,
                         status == IMU_STATUS_VERIFY_FAILED, status);
    s_icm53611_last_mreg_verify_error.valid = true;
    s_icm53611_last_mreg_verify_error.bank_name = icm53611_mreg_bank_name(bank);
    s_icm53611_last_mreg_verify_error.reg_name = reg_name;
    s_icm53611_last_mreg_verify_error.bank = bank;
    s_icm53611_last_mreg_verify_error.addr = maddr;
    s_icm53611_last_mreg_verify_error.write_value = write_value;
    s_icm53611_last_mreg_verify_error.read_value = read_value;
    s_icm53611_last_mreg_verify_error.expected_mask = expected_mask;
    s_icm53611_last_mreg_verify_error.expected_value = expected_value;
    s_icm53611_last_mreg_verify_error.status = status;
    s_icm53611_last_mreg_verify_error.retry_count = retry_count;

    DBG_DIRECT("[ERR][IMU] mreg %s/%s r=%02x st=%d",
               icm53611_mreg_bank_name(bank),
               reg_name,
               maddr,
               status);
    DBG_DIRECT("[ERR][IMU] w=%02x r=%02x "
               "m=%02x e=%02x",
               write_value,
               read_value,
               expected_mask,
               expected_value);
    DBG_DIRECT("[ERR][IMU] bank=%02x addr=%02x "
               "data=%02x retry=%u",
               bank,
               maddr,
               write_value,
               retry_count);
}

static void icm53611_log_mreg_retry(uint8_t bank,
                                    const char *reg_name,
                                    uint8_t maddr,
                                    uint8_t attempt,
                                    uint8_t write_value,
                                    uint8_t read_value,
                                    imu_status_t status,
                                    uint8_t mclk_before,
                                    uint8_t mclk_after)
{
    const bool pass = ((status == IMU_STATUS_OK) && (read_value == write_value));

#if IMU_MREG_DIAG_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE
    DBG_DIRECT("[IMU][MREG_RETRY] bank=%s reg=%s addr=0x%02x attempt=%u",
               icm53611_mreg_bank_name(bank),
               reg_name,
               maddr,
               attempt);
    DBG_DIRECT("[IMU][MREG_RETRY] write_value=0x%02x read_value=0x%02x status=%d result=%s",
               write_value,
               read_value,
               status,
               icm53611_pass_fail(pass));
    DBG_DIRECT("[IMU][MREG_RETRY] mclk_before=0x%02x mclk_after=0x%02x",
               mclk_before,
               mclk_after);
#else
    IMU_UNUSED(bank);
    IMU_UNUSED(reg_name);
    IMU_UNUSED(maddr);
    IMU_UNUSED(attempt);
    IMU_UNUSED(write_value);
    IMU_UNUSED(read_value);
    IMU_UNUSED(status);
    IMU_UNUSED(mclk_before);
    IMU_UNUSED(mclk_after);
    IMU_UNUSED(pass);
#endif
}

static imu_status_t icm53611_mreg1_write_verify(const char *reg_name,
                                                uint8_t maddr,
                                                uint8_t value,
                                                uint8_t expected_mask,
                                                uint8_t expected)
{
    imu_status_t status;
    uint8_t readback = 0U;
    bool retry_allowed = false;

    status = imu_spi_mreg_write(ICM53611_MREG_BLOCK_1, maddr, value);
    if (status != IMU_STATUS_OK)
        pwrd_verify(0U, 1U, maddr, 0U, expected_mask, expected, (uint8_t)status);
    if (status == IMU_STATUS_OK)
    {
        retry_allowed = true;
        status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1, maddr, &readback);
        pwrd_verify(0U, 1U, maddr, readback, expected_mask, expected, (uint8_t)status);
        if (maddr == ICM53611_MREG1_REG_TMST_CONFIG1)
            pwrd_imu_value(PWRD_TMST, readback, status == IMU_STATUS_OK);
        if ((status == IMU_STATUS_OK) && ((readback & expected_mask) == expected))
        {
            return IMU_STATUS_OK;
        }
    }

    if (retry_allowed)
    {
        pwrd_retry();
        status = imu_spi_mreg_write_with_maddr_delay(ICM53611_MREG_BLOCK_1, maddr, value);
        if (status == IMU_STATUS_OK)
        {
            status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1, maddr, &readback);
            pwrd_verify(0U, 1U, maddr, readback, expected_mask, expected, (uint8_t)status);
            if (maddr == ICM53611_MREG1_REG_TMST_CONFIG1)
                pwrd_imu_value(PWRD_TMST, readback, status == IMU_STATUS_OK);
            if ((status == IMU_STATUS_OK) && ((readback & expected_mask) == expected))
            {
                return IMU_STATUS_OK;
            }
        }
    }

    icm53611_log_mreg_verify_error(ICM53611_MREG_BLOCK_1,
                                   reg_name,
                                   maddr,
                                   value,
                                   readback,
                                   expected_mask,
                                   expected,
                                   status,
                                   2U);
    return (status != IMU_STATUS_OK) ? status : IMU_STATUS_VERIFY_FAILED;
}

static imu_status_t icm53611_mreg1_write_verify_readback(const char *reg_name,
                                                         uint8_t maddr,
                                                         uint8_t value,
                                                         uint8_t expected_mask,
                                                         uint8_t expected,
                                                         uint8_t *readback_out)
{
    imu_status_t status;
    uint8_t readback = 0U;
    bool retry_allowed = false;

    if (readback_out != NULL)
    {
        *readback_out = 0xFFU;
    }

    status = imu_spi_mreg_write(ICM53611_MREG_BLOCK_1, maddr, value);
    if (status == IMU_STATUS_OK)
    {
        retry_allowed = true;
        status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1, maddr, &readback);
        if (readback_out != NULL)
        {
            *readback_out = readback;
        }
        if ((status == IMU_STATUS_OK) && ((readback & expected_mask) == expected))
        {
            return IMU_STATUS_OK;
        }
    }

    if (retry_allowed)
    {
        status = imu_spi_mreg_write_with_maddr_delay(ICM53611_MREG_BLOCK_1, maddr, value);
        if (status == IMU_STATUS_OK)
        {
            status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1, maddr, &readback);
            if (readback_out != NULL)
            {
                *readback_out = readback;
            }
            if ((status == IMU_STATUS_OK) && ((readback & expected_mask) == expected))
            {
                return IMU_STATUS_OK;
            }
        }
    }

    icm53611_log_mreg_verify_error(ICM53611_MREG_BLOCK_1,
                                   reg_name,
                                   maddr,
                                   value,
                                   readback,
                                   expected_mask,
                                   expected,
                                   status,
                                   2U);
    return (status != IMU_STATUS_OK) ? status : IMU_STATUS_VERIFY_FAILED;
}

static imu_status_t icm53611_wom_thr_normal_verify(const char *reg_name,
                                                   uint8_t maddr,
                                                   uint8_t value,
                                                   uint8_t *readback_out)
{
    imu_status_t status;
    uint8_t readback = 0U;
    uint8_t mclk_before = 0U;
    uint8_t mclk_after = 0U;
    uint8_t attempt;

    for (attempt = 1U; attempt <= 3U; attempt++)
    {
        readback = 0U;
        if (attempt > 1U) pwrd_retry();
        IMU_UNUSED(imu_spi_read_reg(ICM53611_REG_MCLK_RDY, &mclk_before));
        if (attempt == 1U)
        {
            status = imu_spi_mreg_write(ICM53611_MREG_BANK_WOM_THR, maddr, value);
        }
        else
        {
            status = imu_spi_mreg_write_with_maddr_delay(ICM53611_MREG_BANK_WOM_THR, maddr, value);
        }

        if (status == IMU_STATUS_OK)
        {
            imu_bsp_delay_us((uint32_t)attempt * 10U);
            status = imu_spi_mreg_read(ICM53611_MREG_BANK_WOM_THR, maddr, &readback);
            pwrd_verify(0U, ICM53611_MREG_BANK_WOM_THR, maddr, readback, 0xffU, value, (uint8_t)status);
            if ((status == IMU_STATUS_OK) && (readback == value))
            {
                if (readback_out != NULL)
                {
                    *readback_out = readback;
                }
                return IMU_STATUS_OK;
            }
        }
        IMU_UNUSED(imu_spi_read_reg(ICM53611_REG_MCLK_RDY, &mclk_after));
        icm53611_log_mreg_retry(ICM53611_MREG_BANK_WOM_THR,
                                reg_name,
                                maddr,
                                attempt,
                                value,
                                readback,
                                status,
                                mclk_before,
                                mclk_after);
    }

    if (readback_out != NULL)
    {
        *readback_out = readback;
    }

    icm53611_log_mreg_verify_error(ICM53611_MREG_BANK_WOM_THR,
                                   reg_name,
                                   maddr,
                                   value,
                                   readback,
                                   0xFFU,
                                   value,
                                   status,
                                   3U);
    return (status != IMU_STATUS_OK) ? status : IMU_STATUS_VERIFY_FAILED;
}

static imu_status_t icm53611_wom_thr_write_verify(const char *axis_name,
                                                  const char *reg_name,
                                                  uint8_t maddr,
                                                  uint8_t value,
                                                  icm53611_wom_thr_verify_result_t *result)
{
    imu_status_t same_status;
    imu_status_t normal_status;
    uint8_t same_read = 0U;
    uint8_t mclk_before = 0U;
    uint8_t mclk_after = 0U;
    bool same_pass;
    bool normal_pass;
    uint8_t normal_read = 0U;

    same_status = imu_spi_mreg_write_read_same_session(ICM53611_MREG_BANK_WOM_THR,
                                                       maddr,
                                                       value,
                                                       &same_read,
                                                       &mclk_before,
                                                       &mclk_after);
    pwrd_verify(0U, ICM53611_MREG_BANK_WOM_THR, maddr, same_read, 0xffU, value, (uint8_t)same_status);
    same_pass = ((same_status == IMU_STATUS_OK) && (same_read == value));

    normal_status = icm53611_wom_thr_normal_verify(reg_name, maddr, value, &normal_read);
    if (maddr == ICM53611_REG_ACCEL_WOM_X_THR) pwrd_extra(4U, normal_read, (uint8_t)normal_status);
    if (maddr == ICM53611_REG_ACCEL_WOM_Y_THR) pwrd_extra(5U, normal_read, (uint8_t)normal_status);
    if (maddr == ICM53611_REG_ACCEL_WOM_Z_THR) pwrd_extra(6U, normal_read, (uint8_t)normal_status);
    normal_pass = (normal_status == IMU_STATUS_OK);

    if (result != NULL)
    {
        result->axis_name = axis_name;
        result->reg_name = reg_name;
        result->addr = maddr;
        result->write_value = value;
        result->same_read = same_read;
        result->normal_read = normal_read;
        result->same_status = same_status;
        result->normal_status = normal_status;
        result->same_pass = same_pass;
        result->normal_pass = normal_pass;
    }

#if IMU_MREG_DIAG_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE
    DBG_DIRECT("[IMU_WOM_THR_DIAG] bank=MREG1 reg=%s addr=0x%02x write=0x%02x read=0x%02x result=%s",
               reg_name,
               maddr,
               value,
               normal_read,
               icm53611_pass_fail(normal_pass));
    DBG_DIRECT("[IMU_WOM_THR_DIAG] axis=%s same_read=0x%02x same_status=%d "
               "same_session_verify=%s normal_verify=%s",
               axis_name,
               same_read,
               same_status,
               icm53611_pass_fail(same_pass),
               icm53611_pass_fail(normal_pass));
    ZY100_DIAG_LOG("[IMU_WOM_THR_DIAG] axis=%s mclk_before=0x%02x mclk_after=0x%02x",
               axis_name,
               mclk_before,
               mclk_after);
#endif

    return normal_status;
}

static void icm53611_wom_log_bank_fix_result(const icm53611_wom_thr_verify_result_t *x,
                                             const icm53611_wom_thr_verify_result_t *y,
                                             const icm53611_wom_thr_verify_result_t *z)
{
    const bool threshold_normal_pass = ((x != NULL) && (y != NULL) && (z != NULL) &&
                                        x->normal_pass &&
                                        y->normal_pass &&
                                        z->normal_pass);
    const bool same_session_pass = ((x != NULL) && (y != NULL) && (z != NULL) &&
                                    x->same_pass &&
                                    y->same_pass &&
                                    z->same_pass);

#if IMU_MREG_DIAG_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE
    DBG_DIRECT("[WOM_BANK_FIX_RESULT] wom_threshold_bank=MREG1 mreg1_blk_sel=0x%02x "
               "mreg2_blk_sel=0x%02x",
               ICM53611_MREG_BLOCK_1,
               ICM53611_MREG_BLOCK_2);
    DBG_DIRECT("[WOM_BANK_FIX_RESULT] x_same_session_verify=%s x_normal_verify=%s "
               "y_same_session_verify=%s y_normal_verify=%s",
               icm53611_pass_fail((x != NULL) && x->same_pass),
               icm53611_pass_fail((x != NULL) && x->normal_pass),
               icm53611_pass_fail((y != NULL) && y->same_pass),
               icm53611_pass_fail((y != NULL) && y->normal_pass));
    DBG_DIRECT("[WOM_BANK_FIX_RESULT] z_same_session_verify=%s z_normal_verify=%s",
               icm53611_pass_fail((z != NULL) && z->same_pass),
               icm53611_pass_fail((z != NULL) && z->normal_pass));
    DBG_DIRECT("[WOM_BANK_FIX_RESULT] threshold_normal_verify=%s same_session_diag_result=%s "
               "all_threshold_verify=%s",
               icm53611_pass_fail(threshold_normal_pass),
               same_session_pass ? "PASS" : "WARN",
               icm53611_pass_fail(threshold_normal_pass));
    DBG_DIRECT("[WOM_BANK_FIX_RESULT] main_capture_continued=%u", (uint32_t)1U);
#else
    IMU_UNUSED(threshold_normal_pass);
    IMU_UNUSED(same_session_pass);
#endif
}

static imu_status_t icm53611_wom_verify_threshold_triplet(uint8_t threshold_code)
{
    imu_status_t status;
    imu_status_t first_error = IMU_STATUS_OK;
    icm53611_wom_thr_verify_result_t x_result = {0};
    icm53611_wom_thr_verify_result_t y_result = {0};
    icm53611_wom_thr_verify_result_t z_result = {0};

    status = icm53611_wom_thr_write_verify("X",
                                           "ACCEL_WOM_X_THR",
                                           ICM53611_REG_ACCEL_WOM_X_THR,
                                           threshold_code,
                                           &x_result);
    if ((status != IMU_STATUS_OK) && (first_error == IMU_STATUS_OK))
    {
        first_error = status;
    }

    status = icm53611_wom_thr_write_verify("Y",
                                           "ACCEL_WOM_Y_THR",
                                           ICM53611_REG_ACCEL_WOM_Y_THR,
                                           threshold_code,
                                           &y_result);
    if ((status != IMU_STATUS_OK) && (first_error == IMU_STATUS_OK))
    {
        first_error = status;
    }

    status = icm53611_wom_thr_write_verify("Z",
                                           "ACCEL_WOM_Z_THR",
                                           ICM53611_REG_ACCEL_WOM_Z_THR,
                                           threshold_code,
                                           &z_result);
    if ((status != IMU_STATUS_OK) && (first_error == IMU_STATUS_OK))
    {
        first_error = status;
    }

    icm53611_wom_log_bank_fix_result(&x_result, &y_result, &z_result);
    return first_error;
}

static void icm53611_mreg2_read_only_diag_one(const char *reg_name,
                                              uint8_t addr,
                                              uint8_t *readback,
                                              imu_status_t *status_out)
{
    imu_status_t status;
    uint8_t value = 0U;

    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_2, addr, &value);
    ZY100_DIAG_LOG("[IMU_MREG2_DIAG] bank=MREG2 reg=%s addr=0x%02x read=0x%02x status=%d",
               reg_name,
               addr,
               value,
               status);
    ZY100_DIAG_LOG("[IMU_MREG2_DIAG] bank_select=0x%02x mreg_addr=0x%02x mreg_data=0x%02x op=%s",
               ICM53611_MREG_BLOCK_2,
               addr,
               value,
               "BLK_SEL_R/MADDR_R/M_R");

    if (readback != NULL)
    {
        *readback = value;
    }
    if (status_out != NULL)
    {
        *status_out = status;
    }
}

static void icm53611_mreg2_read_only_diag(void)
{
    uint8_t otp_ctrl7 = 0U;
    imu_status_t status_otp = IMU_STATUS_OK;
    const char *result;

    icm53611_mreg2_read_only_diag_one("OTP_CTRL7",
                                      ICM53611_MREG2_REG_OTP_CTRL7,
                                      &otp_ctrl7,
                                      &status_otp);
    IMU_UNUSED(otp_ctrl7);
    if (status_otp != IMU_STATUS_OK)
    {
        result = "FAIL";
    }
    else
    {
        result = "PASS";
    }

    ZY100_DIAG_LOG("[IMU_MREG2_DIAG_SUMMARY] reg=OTP_CTRL7 status=%d result=%s",
               status_otp,
               result);
}

void icm53611_wom_pre_fifo_mreg_diag(void)
{
    imu_status_t read_status;
    imu_status_t same_status = IMU_STATUS_OK;
    imu_status_t restore_status = IMU_STATUS_OK;
    uint8_t original = 0U;
    uint8_t readback = 0U;
    uint8_t mclk_before = 0U;
    uint8_t mclk_after = 0U;
    bool pass = false;

    read_status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1,
                                    ICM53611_MREG1_REG_TMST_CONFIG1,
                                    &original);
    if (read_status == IMU_STATUS_OK)
    {
        same_status = imu_spi_mreg_write_read_same_session(ICM53611_MREG_BLOCK_1,
                                                           ICM53611_MREG1_REG_TMST_CONFIG1,
                                                           original,
                                                           &readback,
                                                           &mclk_before,
                                                           &mclk_after);
        pass = ((same_status == IMU_STATUS_OK) && (readback == original));

        restore_status = imu_spi_mreg_write(ICM53611_MREG_BLOCK_1,
                                            ICM53611_MREG1_REG_TMST_CONFIG1,
                                            original);
    }
    else
    {
        same_status = read_status;
        restore_status = read_status;
    }

    ZY100_DIAG_LOG("[IMU_MREG_COMPARE_DIAG] bank=MREG1 op=same_value_write_read reg=%s addr=0x%02x",
               "TMST_CONFIG1",
               ICM53611_MREG1_REG_TMST_CONFIG1);
    ZY100_DIAG_LOG("[IMU_MREG_COMPARE_DIAG] bank_select=0x%02x mreg_addr=0x%02x mreg_data=0x%02x op=%s",
               ICM53611_MREG_BLOCK_1,
               ICM53611_MREG1_REG_TMST_CONFIG1,
               original,
               "BLK_SEL_W/MADDR_W/M_W/BLK_SEL_R/MADDR_R/M_R");
    DBG_DIRECT("[IMU_MREG] o=%02x r=%02x s=%d res=%s",
               original,
               readback,
               same_status,
               icm53611_pass_fail(pass));
    ZY100_DIAG_LOG("[IMU_MREG_COMPARE_DIAG] mclk_before=0x%02x mclk_after=0x%02x restore_status=%d",
               mclk_before,
               mclk_after,
               restore_status);

    icm53611_mreg2_read_only_diag();
}

imu_status_t icm53611_apply_basic_config(const icm53611_cfg_t *cfg)
{
    imu_status_t status;
    const uint8_t device_cfg = ICM53611_DEVICE_CONFIG_SPI_AP_4WIRE;
    uint8_t pwr_mgmt0_cfg;
    uint8_t sensor_config3 = 0U;

    if (cfg == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    pwr_mgmt0_cfg = cfg->pwr_mgmt0;
    status = icm53611_wom_stop_clocked();
    if (status != IMU_STATUS_OK) return status;

    /*
     * Bring-up sequence:
     * 1) move to sleep/off mode
     * 2) program accel/gyro base configuration
     * 3) switch accel/gyro to requested working mode
     */
    status = icm53611_write_with_verify(ICM53611_REG_DEVICE_CONFIG,
                                        device_cfg,
                                        ICM53611_DEVICE_CONFIG_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = icm53611_write_with_verify(ICM53611_REG_PWR_MGMT0,
                                        (ICM53611_PWR_MGMT0_ACCEL_MODE_OFF | ICM53611_PWR_MGMT0_GYRO_MODE_OFF),
                                        ICM53611_PWR_MGMT0_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    imu_bsp_delay_us(200U);

    status = icm53611_write_with_verify(ICM53611_REG_GYRO_CONFIG0, cfg->gyro_config0,
                                        ICM53611_GYRO_CONFIG0_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = icm53611_write_with_verify(ICM53611_REG_ACCEL_CONFIG0, cfg->accel_config0,
                                        ICM53611_ACCEL_CONFIG0_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = icm53611_write_with_verify(ICM53611_REG_GYRO_CONFIG1, cfg->gyro_config1,
                                        ICM53611_GYRO_CONFIG1_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = icm53611_write_with_verify(ICM53611_REG_ACCEL_CONFIG1, cfg->accel_config1,
                                        ICM53611_ACCEL_CONFIG1_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = icm53611_write_with_verify(ICM53611_REG_PWR_MGMT0, pwr_mgmt0_cfg,
                                        ICM53611_PWR_MGMT0_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    if ((pwr_mgmt0_cfg & (ICM53611_PWR_MGMT0_ACCEL_OIS_EN | ICM53611_PWR_MGMT0_GYRO_OIS_EN)) != 0U)
    {
        /*
         * OIS readout registers remain enabled only for explicit OIS sessions.
         * The normal 100Hz BLE stream uses UI_6AXIS with OIS bits off.
         */
        status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1, ICM53611_MREG1_REG_SENSOR_CONFIG3, &sensor_config3);
        if (status != IMU_STATUS_OK)
        {
            return status;
        }
        if ((sensor_config3 & ICM53611_MREG1_SENSOR_CONFIG3_OIS_CONFIG_DISABLE) != 0U)
        {
            sensor_config3 &= (uint8_t)(~ICM53611_MREG1_SENSOR_CONFIG3_OIS_CONFIG_DISABLE);
            status = imu_spi_mreg_write(ICM53611_MREG_BLOCK_1, ICM53611_MREG1_REG_SENSOR_CONFIG3, sensor_config3);
            if (status != IMU_STATUS_OK)
            {
                return status;
            }
        }
    }

    status = icm53611_configure_drdy_int1();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    if ((pwr_mgmt0_cfg & (ICM53611_PWR_MGMT0_ACCEL_OIS_EN | ICM53611_PWR_MGMT0_GYRO_OIS_EN)) != 0U)
    {
        status = icm53611_enable_ois_timestamp_regs(NULL);
        if (status != IMU_STATUS_OK)
        {
            return status;
        }
    }
    icm53611_reset_ois_tmst_tracker();

    imu_bsp_delay_us(ICM53611_PWR_READY_WAIT_US);
    return IMU_STATUS_OK;
}

imu_status_t icm53611_bringup(const icm53611_cfg_t *cfg)
{
    imu_status_t status;
    uint8_t who_am_i = 0U;
    const icm53611_cfg_t *use_cfg = (cfg != NULL) ? cfg : icm53611_default_config();

    status = icm53611_init_bus();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = imu_bsp_power_ctrl(true);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    imu_bsp_delay_us(ICM53611_POWER_ON_SETTLE_US);
    icm53611_log_line_levels("power-on");

    status = imu_bsp_spi_set_mode(SPI_CPOL_Low, SPI_CPHA_1Edge);
    if (status != IMU_STATUS_OK)
    {
        IMU_LOG_ERROR("WHO mode st=%d", status);
        return status;
    }
    IMU_LOG_INFO("SPI fixed to 4-wire + mode0 (CPOL=0, CPHA=0) for IMU bring-up");

    status = icm53611_read_whoami(&who_am_i);
    if (status != IMU_STATUS_OK)
    {
        IMU_LOG_ERROR("WHO read st=%d", status);
        return status;
    }

    if (who_am_i != ICM53611_WHO_AM_I_VALUE)
    {
        IMU_LOG_ERROR("WHO exp=%02x got=%02x",
                      ICM53611_WHO_AM_I_VALUE, who_am_i);
        if ((who_am_i == 0xFFU) || (who_am_i == 0x00U))
        {
            IMU_LOG_ERROR("WHO bad=%02x",
                          who_am_i);
        }

        status = icm53611_read_whoami_mode0(&who_am_i);
        if (status != IMU_STATUS_OK)
        {
            if ((who_am_i == 0xFFU) || (who_am_i == 0x00U))
            {
                IMU_LOG_ERROR("WHO retry=%02x",
                              who_am_i);
            }
            return IMU_STATUS_NOT_FOUND;
        }
    }

    status = icm53611_soft_reset();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = icm53611_apply_basic_config(use_cfg);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    if ((use_cfg->pwr_mgmt0 & (ICM53611_PWR_MGMT0_ACCEL_OIS_EN | ICM53611_PWR_MGMT0_GYRO_OIS_EN)) != 0U)
    {
        status = icm53611_alive_check();
    }
    else
    {
        icm53611_raw_sample_t ui_sample;
        status = icm53611_read_ui_6axis_sample(&ui_sample);
    }
    return status;
}

imu_status_t icm53611_alive_check(void)
{
    imu_status_t status;
    uint8_t who_am_i = 0U;
    icm53611_raw_sample_t sample_a;
    icm53611_raw_sample_t sample_b;

    status = icm53611_read_whoami(&who_am_i);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    if (who_am_i != ICM53611_WHO_AM_I_VALUE)
    {
        return IMU_STATUS_NOT_FOUND;
    }

    status = icm53611_read_raw_sample(&sample_a);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    imu_bsp_delay_us(1000U);
    status = icm53611_read_raw_sample(&sample_b);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    if ((sample_a.accel_x == sample_b.accel_x) &&
        (sample_a.accel_y == sample_b.accel_y) &&
        (sample_a.accel_z == sample_b.accel_z) &&
        (sample_a.gyro_x == sample_b.gyro_x) &&
        (sample_a.gyro_y == sample_b.gyro_y) &&
        (sample_a.gyro_z == sample_b.gyro_z))
    {
        IMU_LOG_WARN("raw unchanged");
    }

    return IMU_STATUS_OK;
}

imu_status_t icm53611_read_ois_sample(icm53611_raw_sample_t *sample)
{
    return icm53611_read_ois_6axis_sample_common(sample, false);
}

imu_status_t icm53611_read_ois_sample_isr_fast(icm53611_raw_sample_t *sample)
{
    return icm53611_read_ois_6axis_sample_common(sample, true);
}

imu_status_t icm53611_read_raw_sample(icm53611_raw_sample_t *sample)
{
    /* IMU unified read path is OIS-only by product requirement. */
    return icm53611_read_ois_sample(sample);
}

static imu_status_t icm53611_read_int_status_impl(
    icm53611_int_status_t *status, bool report_busy)
{
    imu_status_t ret;

    if (status == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    /*
     * INT_STATUS2 WOM bits are clear-on-read. Capture one snapshot in task
     * context and share it with FIFO and WOM consumers.
     */
    ret = report_busy ?
        imu_spi_read_reg_ex(ICM53611_REG_INT_STATUS, &status->int_status) :
        imu_spi_read_reg(ICM53611_REG_INT_STATUS, &status->int_status);
    if (ret != IMU_STATUS_OK)
    {
        return ret;
    }

    ret = report_busy ?
        imu_spi_read_reg_ex(ICM53611_REG_INT_STATUS2, &status->int_status2) :
        imu_spi_read_reg(ICM53611_REG_INT_STATUS2, &status->int_status2);
    if (ret != IMU_STATUS_OK)
    {
        return ret;
    }

    ret = report_busy ?
        imu_spi_read_reg_ex(ICM53611_REG_INT_STATUS3, &status->int_status3) :
        imu_spi_read_reg(ICM53611_REG_INT_STATUS3, &status->int_status3);
    if (ret != IMU_STATUS_OK)
    {
        return ret;
    }

    status->local_ts_us = imu_bsp_local_timestamp_us();
    return IMU_STATUS_OK;
}

imu_status_t icm53611_read_int_status(icm53611_int_status_t *status)
{
    return icm53611_read_int_status_impl(status, false);
}

imu_status_t icm53611_read_int_status_ex(icm53611_int_status_t *status)
{
    return icm53611_read_int_status_impl(status, true);
}

static imu_status_t icm53611_set_fifo_count_byte_mode(void)
{
    imu_status_t status;
    uint8_t intf_config0 = 0U;

    status = imu_spi_read_reg(ICM53611_REG_INTF_CONFIG0, &intf_config0);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    intf_config0 &= (uint8_t)~ICM53611_INTF_CONFIG0_FIFO_COUNT_FORMAT;
    return icm53611_write_with_verify(ICM53611_REG_INTF_CONFIG0,
                                      intf_config0,
                                      ICM53611_INTF_CONFIG0_FIFO_COUNT_FORMAT);
}

static imu_status_t icm53611_configure_v0_mreg1_fifo_policy(void)
{
    imu_status_t status;
    uint8_t sensor_config3 = 0U;
    uint8_t fdr_config = 0U;
    uint8_t expected;

    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1, ICM53611_MREG1_REG_SENSOR_CONFIG3, &sensor_config3);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    sensor_config3 |= ICM53611_MREG1_SENSOR_CONFIG3_OIS_CONFIG_DISABLE;
    expected = ICM53611_MREG1_SENSOR_CONFIG3_V0_VERIFY_MASK;
    status = icm53611_mreg1_write_verify("SENSOR_CONFIG3",
                                          ICM53611_MREG1_REG_SENSOR_CONFIG3,
                                          sensor_config3,
                                          ICM53611_MREG1_SENSOR_CONFIG3_V0_VERIFY_MASK,
                                          expected);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1, ICM53611_MREG1_REG_FDR_CONFIG, &fdr_config);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    fdr_config &= (uint8_t)~ICM53611_MREG1_FDR_CONFIG_FDR_SEL_MASK;
    return icm53611_mreg1_write_verify("FDR_CONFIG",
                                       ICM53611_MREG1_REG_FDR_CONFIG,
                                       fdr_config,
                                       ICM53611_MREG1_FDR_CONFIG_FDR_SEL_MASK,
                                       0U);
}

static void icm53611_record_first_status(imu_status_t status, imu_status_t *first_status)
{
    if ((first_status != NULL) &&
        (*first_status == IMU_STATUS_OK) &&
        (status != IMU_STATUS_OK))
    {
        *first_status = status;
    }
}

static imu_status_t icm53611_restore_ui_fifo_packet3_mreg_policy(void)
{
    imu_status_t status;
    uint8_t tmst_cfg1 = 0U;
    uint8_t fifo_cfg5 = 0U;
    uint8_t fifo_cfg5_clear;

    status = icm53611_wom_mreg_bits(ICM53611_MREG1_REG_FIFO_CONFIG6,
                 ICM53611_FIFO_RC_REQ_DISABLE, ICM53611_FIFO_RC_REQ_DISABLE);
    if (status != IMU_STATUS_OK) return status;

    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1,
                               ICM53611_MREG1_REG_TMST_CONFIG1,
                               &tmst_cfg1);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    tmst_cfg1 |= ICM53611_MREG1_TMST_CONFIG1_TMST_EN;
    tmst_cfg1 &= (uint8_t)~(ICM53611_MREG1_TMST_CONFIG1_TMST_ON_SREG_EN |
                            ICM53611_MREG1_TMST_CONFIG1_TMST_FSYNC_EN |
                            ICM53611_MREG1_TMST_CONFIG1_TMST_DELTA_EN |
                            ICM53611_MREG1_TMST_CONFIG1_TMST_RES);
    status = icm53611_mreg1_write_verify("TMST_CONFIG1_A2",
                                          ICM53611_MREG1_REG_TMST_CONFIG1,
                                          tmst_cfg1,
                                          ICM53611_MREG1_TMST_CONFIG1_PACKET3_VERIFY_MASK,
                                          ICM53611_MREG1_TMST_CONFIG1_TMST_EN);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1,
                               ICM53611_MREG1_REG_FIFO_CONFIG5,
                               &fifo_cfg5);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    fifo_cfg5_clear = (ICM53611_MREG1_FIFO_CONFIG5_FIFO_RESUME_PARTIAL_RD |
                       ICM53611_MREG1_FIFO_CONFIG5_FIFO_HIRES_EN |
                       ICM53611_MREG1_FIFO_CONFIG5_FIFO_TMST_FSYNC_EN |
                       ICM53611_MREG1_FIFO_CONFIG5_FIFO_GYRO_EN |
                       ICM53611_MREG1_FIFO_CONFIG5_FIFO_ACCEL_EN);
    fifo_cfg5 &= (uint8_t)~fifo_cfg5_clear;
    fifo_cfg5 |= (ICM53611_MREG1_FIFO_CONFIG5_FIFO_GYRO_EN |
                  ICM53611_MREG1_FIFO_CONFIG5_FIFO_ACCEL_EN);
    return icm53611_mreg1_write_verify(
        "FIFO_CONFIG5_A2",
        ICM53611_MREG1_REG_FIFO_CONFIG5,
        fifo_cfg5,
        ICM53611_MREG1_FIFO_CONFIG5_V0_VERIFY_MASK,
        (ICM53611_MREG1_FIFO_CONFIG5_FIFO_GYRO_EN |
         ICM53611_MREG1_FIFO_CONFIG5_FIFO_ACCEL_EN));
}

static imu_status_t icm53611_restore_ui_fifo_stream_mode_rmw(void)
{
    imu_status_t status;
    uint8_t fifo_config1 = 0U;

    status = imu_spi_read_reg(ICM53611_REG_FIFO_CONFIG1, &fifo_config1);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    fifo_config1 &= (uint8_t)~(ICM53611_FIFO_CONFIG1_FIFO_BYPASS |
                               ICM53611_FIFO_CONFIG1_FIFO_MODE_STOP_ON_FULL);
    return icm53611_write_with_verify(ICM53611_REG_FIFO_CONFIG1,
                                      fifo_config1,
                                      (ICM53611_FIFO_CONFIG1_FIFO_BYPASS |
                                       ICM53611_FIFO_CONFIG1_FIFO_MODE_STOP_ON_FULL));
}

static imu_status_t icm53611_restore_ui_fifo_config_regs(uint8_t gyro_config0,
                                                         uint8_t accel_config0)
{
    imu_status_t status;

    status = icm53611_write_with_verify(ICM53611_REG_GYRO_CONFIG0,
                                        gyro_config0,
                                        ICM53611_GYRO_CONFIG0_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    return icm53611_write_with_verify(ICM53611_REG_ACCEL_CONFIG0,
                                      accel_config0,
                                      ICM53611_ACCEL_CONFIG0_VERIFY_MASK);
}

static imu_status_t icm53611_restore_ui_fifo_1600_config_regs(void)
{
    return icm53611_restore_ui_fifo_config_regs(
        ICM53611_UI_FIFO_GYRO_CONFIG0_1600HZ,
        ICM53611_UI_FIFO_ACCEL_CONFIG0_1600HZ);
}

static imu_status_t icm53611_restore_ui_fifo_800_config_regs(void)
{
    return icm53611_restore_ui_fifo_config_regs(
        ICM53611_UI_FIFO_GYRO_CONFIG0_800HZ,
        ICM53611_UI_FIFO_ACCEL_CONFIG0_800HZ);
}

static imu_status_t icm53611_restore_ui_ln_power_rmw(void)
{
    imu_status_t status;
    uint8_t pwr_mgmt0 = 0U;

    status = imu_spi_read_reg(ICM53611_REG_PWR_MGMT0, &pwr_mgmt0);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    pwr_mgmt0 &= (uint8_t)~(ICM53611_PWR_MGMT0_IDLE_EN |
                            ICM53611_PWR_MGMT0_GYRO_OIS_EN |
                            ICM53611_PWR_MGMT0_ACCEL_OIS_EN |
                            ICM53611_PWR_MGMT0_GYRO_MODE_MASK |
                            ICM53611_PWR_MGMT0_ACCEL_MODE_MASK);
    pwr_mgmt0 |= (ICM53611_PWR_MGMT0_GYRO_MODE_LN |
                  ICM53611_PWR_MGMT0_ACCEL_MODE_LN);
    return icm53611_write_with_verify(ICM53611_REG_PWR_MGMT0,
                                      pwr_mgmt0,
                                      ICM53611_PWR_MGMT0_VERIFY_MASK);
}

static imu_status_t icm53611_disable_ui_sensors_rmw(bool report_busy)
{
    imu_status_t status;
    uint8_t pwr_mgmt0 = 0U;

    status = report_busy ?
        imu_spi_read_reg_ex(ICM53611_REG_PWR_MGMT0, &pwr_mgmt0) :
        imu_spi_read_reg(ICM53611_REG_PWR_MGMT0, &pwr_mgmt0);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    if ((pwr_mgmt0 & ICM53611_PWR_MGMT0_ACCEL_MODE_MASK) == ICM53611_PWR_MGMT0_ACCEL_MODE_LP)
    {
        uint8_t odr = 0U;
        status = report_busy ? imu_spi_read_reg_ex(ICM53611_REG_ACCEL_CONFIG0, &odr) :
                               imu_spi_read_reg(ICM53611_REG_ACCEL_CONFIG0, &odr);
        if (status != IMU_STATUS_OK) return status;
        status = icm53611_wom_dlps_exit_wuosc_if_needed(pwr_mgmt0, odr);
        if (status != IMU_STATUS_OK) return status;
        pwr_mgmt0 |= ICM53611_PWR_MGMT0_ACCEL_LP_CLK_SEL;
    }
    if (pwr_mgmt0 & (ICM53611_PWR_MGMT0_GYRO_MODE_MASK | ICM53611_PWR_MGMT0_GYRO_OIS_EN))
        imu_bsp_delay_us(45000U); /* DS p60; no proven gyro start timestamp here. */
    pwr_mgmt0 &= (uint8_t)~(ICM53611_PWR_MGMT0_GYRO_OIS_EN |
                            ICM53611_PWR_MGMT0_ACCEL_OIS_EN |
                            ICM53611_PWR_MGMT0_GYRO_MODE_MASK |
                            ICM53611_PWR_MGMT0_ACCEL_MODE_MASK);
    return icm53611_write_with_verify_impl(ICM53611_REG_PWR_MGMT0,
                                      pwr_mgmt0,
                                      ICM53611_PWR_MGMT0_VERIFY_MASK, report_busy);
}

imu_status_t icm53611_stop_ui_sensors_preserve_fifo(void)
{
    return icm53611_disable_ui_sensors_rmw(false);
}

imu_status_t icm53611_stop_ui_sensors_preserve_fifo_ex(void)
{
    return icm53611_disable_ui_sensors_rmw(true);
}

static imu_status_t icm53611_restore_ui_fifo_1600_after_fdr_clear(void)
{
    imu_status_t status;

    status = icm53611_wom_stop_clocked();
    if (status != IMU_STATUS_OK) return status;

    status = icm53611_restore_ui_fifo_1600_config_regs();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = icm53611_restore_ui_fifo_packet3_mreg_policy();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = icm53611_restore_ui_fifo_stream_mode_rmw();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = icm53611_restore_ui_ln_power_rmw();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    imu_bsp_delay_us(ICM53611_PWR_READY_WAIT_US);
    return icm53611_fifo_flush();
}

static imu_status_t icm53611_restore_ui_fifo_800_after_fdr_clear(void)
{
    imu_status_t status;

    status = icm53611_wom_stop_clocked();
    if (status != IMU_STATUS_OK) return status;

    status = icm53611_restore_ui_fifo_800_config_regs();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = icm53611_restore_ui_fifo_packet3_mreg_policy();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = icm53611_restore_ui_fifo_stream_mode_rmw();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = icm53611_restore_ui_ln_power_rmw();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    imu_bsp_delay_us(ICM53611_PWR_READY_WAIT_US);
    return icm53611_fifo_flush();
}

imu_status_t icm53611_read_a2_rate_regs(icm53611_a2_rate_regs_t *regs)
{
    imu_status_t status;
    imu_status_t first_status = IMU_STATUS_OK;

    if (regs == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    regs->pwr_mgmt0 = 0xFFU;
    regs->gyro_config0 = 0xFFU;
    regs->accel_config0 = 0xFFU;
    regs->fifo_config1 = 0xFFU;
    regs->fifo_config5 = 0xFFU;
    regs->fdr_config = 0xFFU;
    regs->tmst_config1 = 0xFFU;

    status = imu_spi_read_reg(ICM53611_REG_PWR_MGMT0, &regs->pwr_mgmt0);
    icm53611_record_first_status(status, &first_status);
    status = imu_spi_read_reg(ICM53611_REG_GYRO_CONFIG0, &regs->gyro_config0);
    icm53611_record_first_status(status, &first_status);
    status = imu_spi_read_reg(ICM53611_REG_ACCEL_CONFIG0, &regs->accel_config0);
    icm53611_record_first_status(status, &first_status);
    status = imu_spi_read_reg(ICM53611_REG_FIFO_CONFIG1, &regs->fifo_config1);
    icm53611_record_first_status(status, &first_status);
    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1,
                               ICM53611_MREG1_REG_FIFO_CONFIG5,
                               &regs->fifo_config5);
    icm53611_record_first_status(status, &first_status);
    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1,
                               ICM53611_MREG1_REG_FDR_CONFIG,
                               &regs->fdr_config);
    icm53611_record_first_status(status, &first_status);
    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1,
                               ICM53611_MREG1_REG_TMST_CONFIG1,
                               &regs->tmst_config1);
    icm53611_record_first_status(status, &first_status);
    return first_status;
}

imu_status_t icm53611_restore_ui_fifo_1600hz_minimal(void)
{
    imu_status_t status;
    uint8_t fdr_config = 0U;

    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1,
                               ICM53611_MREG1_REG_FDR_CONFIG,
                               &fdr_config);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    if ((fdr_config & ICM53611_MREG1_FDR_CONFIG_FDR_SEL_MASK) != 0U)
    {
        status = icm53611_disable_ui_sensors_rmw(false);
        if (status != IMU_STATUS_OK)
        {
            return status;
        }
        imu_bsp_delay_us(200U);

        fdr_config &= (uint8_t)~ICM53611_MREG1_FDR_CONFIG_FDR_SEL_MASK;
        status = icm53611_mreg1_write_verify("FDR_CONFIG_A2",
                                              ICM53611_MREG1_REG_FDR_CONFIG,
                                              fdr_config,
                                              ICM53611_MREG1_FDR_CONFIG_FDR_SEL_MASK,
                                              0U);
        if (status != IMU_STATUS_OK)
        {
            return status;
        }
        return icm53611_restore_ui_fifo_1600_after_fdr_clear();
    }

    return icm53611_restore_ui_fifo_1600_after_fdr_clear();
}

imu_status_t icm53611_restore_ui_fifo_800hz_minimal(void)
{
    imu_status_t status;
    uint8_t fdr_config = 0U;

    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1,
                               ICM53611_MREG1_REG_FDR_CONFIG,
                               &fdr_config);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    if ((fdr_config & ICM53611_MREG1_FDR_CONFIG_FDR_SEL_MASK) != 0U)
    {
        status = icm53611_disable_ui_sensors_rmw(false);
        if (status != IMU_STATUS_OK)
        {
            return status;
        }
        imu_bsp_delay_us(200U);

        fdr_config &= (uint8_t)~ICM53611_MREG1_FDR_CONFIG_FDR_SEL_MASK;
        status = icm53611_mreg1_write_verify("FDR_CONFIG_FE",
                                              ICM53611_MREG1_REG_FDR_CONFIG,
                                              fdr_config,
                                              ICM53611_MREG1_FDR_CONFIG_FDR_SEL_MASK,
                                              0U);
        if (status != IMU_STATUS_OK)
        {
            return status;
        }
        return icm53611_restore_ui_fifo_800_after_fdr_clear();
    }

    return icm53611_restore_ui_fifo_800_after_fdr_clear();
}

static imu_status_t icm53611_configure_fifo_threshold_int1_only(void)
{
    imu_status_t status;
    uint8_t int_config = 0U;

    status = imu_spi_read_reg(ICM53611_REG_INT_CONFIG, &int_config);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    int_config |= (ICM53611_INT_CONFIG_INT1_MODE |
                   ICM53611_INT_CONFIG_INT1_DRIVE_CIRCUIT |
                   ICM53611_INT_CONFIG_INT1_POLARITY);
    status = icm53611_write_with_verify(ICM53611_REG_INT_CONFIG,
                                        int_config,
                                        ICM53611_INT_CONFIG_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    return icm53611_write_with_verify(ICM53611_REG_INT_SOURCE0,
                                      ICM53611_INT_SOURCE0_FIFO_THS_INT1_EN,
                                      ICM53611_INT_SOURCE0_VERIFY_MASK);
}

/* New session only: one UI trigger, then wait for that same operation.
 * No WOM clock/configuration state or diagnostic reads enter this sequence. */
static imu_status_t icm53611_capture_fifo_flush(icm53611_capture_result_t *result)
{
#if ZY100_BUILD_PRODUCTION
    imu_status_t status;
    uint8_t value = 0U;
    uint16_t polls = 0U;
    uint32_t start_us = 0U, start_ms = 0U, elapsed_us = 0U;
    result->flush_end = ICM53611_FLUSH_IO_ERROR;
    status = imu_spi_write_reg(ICM53611_REG_SIGNAL_PATH_RESET,
                               ICM53611_SIGNAL_PATH_RESET_FIFO_FLUSH);
    if (status != IMU_STATUS_OK) goto finish;
    start_us = (uint32_t)imu_bsp_local_timestamp_us();
    start_ms = zy100_os_time_ms();
    imu_bsp_delay_us(ICM53611_CAPTURE_FLUSH_WAIT_US);
    for (;;)
    {
        status = imu_spi_read_reg(ICM53611_REG_SIGNAL_PATH_RESET, &value);
        polls++;
        elapsed_us = (uint32_t)imu_bsp_local_timestamp_us() - start_us;
        if (status != IMU_STATUS_OK) break;
        if (!(value & ICM53611_SIGNAL_PATH_RESET_FIFO_FLUSH))
        {
            result->flush_end = ICM53611_FLUSH_CLEAR;
            break;
        }
        if (elapsed_us >= ICM53611_CAPTURE_FLUSH_TIMEOUT_US)
            result->flush_end = ICM53611_FLUSH_ELAPSED;
        else if (zy100_os_time_ms() - start_ms >= ICM53611_CAPTURE_FLUSH_OS_GUARD_MS)
            result->flush_end = ICM53611_FLUSH_OS_GUARD;
        else if (polls >= ICM53611_CAPTURE_FLUSH_MAX_POLLS)
            result->flush_end = ICM53611_FLUSH_POLL_GUARD;
        if (result->flush_end != ICM53611_FLUSH_IO_ERROR)
        {
            status = IMU_STATUS_TIMEOUT;
            imu_spi_error_record(3U, 0U, ICM53611_REG_SIGNAL_PATH_RESET, value, true, status);
            break;
        }
        imu_bsp_delay_us(ICM53611_CAPTURE_FLUSH_POLL_US);
    }
finish:
    result->flush_us = elapsed_us;
    result->flush_polls = polls;
    return status;

#else
    /* Factory/diagnostic targets keep their prior finite policy and do not
     * link the Production OS-time wrapper. */
    imu_status_t status;
    uint8_t reset_reg = ICM53611_SIGNAL_PATH_RESET_FIFO_FLUSH;
    uint32_t waited_us = 0U;
    uint16_t polls = 0U;
    uint32_t start = (uint32_t)imu_bsp_local_timestamp_us();

    result->flush_end = ICM53611_FLUSH_IO_ERROR;
    status = imu_spi_write_reg(ICM53611_REG_SIGNAL_PATH_RESET,
                               ICM53611_SIGNAL_PATH_RESET_FIFO_FLUSH);
    if (status != IMU_STATUS_OK) goto finish;
    start = (uint32_t)imu_bsp_local_timestamp_us();
    imu_bsp_delay_us(ICM53611_CAPTURE_FLUSH_WAIT_US);
    waited_us += ICM53611_CAPTURE_FLUSH_WAIT_US;
    while (waited_us <= 1000U /* unchanged non-Production delay budget */)
    {
        status = imu_spi_read_reg(ICM53611_REG_SIGNAL_PATH_RESET, &reset_reg);
        polls++;
        if (status != IMU_STATUS_OK) goto finish;
        if ((reset_reg & ICM53611_SIGNAL_PATH_RESET_FIFO_FLUSH) == 0U)
        { result->flush_end = ICM53611_FLUSH_CLEAR; goto finish; }
        imu_bsp_delay_us(ICM53611_CAPTURE_FLUSH_POLL_US);
        waited_us += ICM53611_CAPTURE_FLUSH_POLL_US;
    }
    result->flush_end = ICM53611_FLUSH_POLL_GUARD;
    status = IMU_STATUS_TIMEOUT;
    imu_spi_error_record(3U, 0U, ICM53611_REG_SIGNAL_PATH_RESET,
                         reset_reg, true, status);
finish:
    result->flush_us = (uint32_t)imu_bsp_local_timestamp_us() - start;
    result->flush_polls = polls;
    return status;
#endif
}

static imu_status_t icm53611_fifo_flush_impl(uint16_t *fifo_count_after,
                                              icm53611_capture_result_t *result);

static imu_status_t icm53611_configure_packet3_fifo_stream_impl(const icm53611_cfg_t *cfg,
                                                    const icm53611_fifo_cfg_t *fifo_cfg,
                                                    icm53611_capture_result_t *result)
{
    imu_status_t status;
    const uint8_t device_cfg = ICM53611_DEVICE_CONFIG_SPI_AP_4WIRE;

    if ((cfg == NULL) || (fifo_cfg == NULL))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    if (result == NULL)
    {
        status = icm53611_write_with_verify(ICM53611_REG_DEVICE_CONFIG,
                                            device_cfg,
                                            ICM53611_DEVICE_CONFIG_VERIFY_MASK);
        if (status != IMU_STATUS_OK)
        {
            return status;
        }
        imu_spi_error_checkpoint();
    }

    status = icm53611_write_with_verify(ICM53611_REG_PWR_MGMT0,
                                        (ICM53611_PWR_MGMT0_ACCEL_MODE_OFF | ICM53611_PWR_MGMT0_GYRO_MODE_OFF),
                                        ICM53611_PWR_MGMT0_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    imu_spi_error_checkpoint();
    imu_bsp_delay_us(200U);

    status = icm53611_configure_v0_mreg1_fifo_policy();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    imu_spi_error_checkpoint();

    status = icm53611_set_fifo_count_byte_mode();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    imu_spi_error_checkpoint();

    status = icm53611_write_with_verify(ICM53611_REG_GYRO_CONFIG0, cfg->gyro_config0,
                                        ICM53611_GYRO_CONFIG0_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    imu_spi_error_checkpoint();

    status = icm53611_write_with_verify(ICM53611_REG_ACCEL_CONFIG0, cfg->accel_config0,
                                        ICM53611_ACCEL_CONFIG0_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    imu_spi_error_checkpoint();

    status = icm53611_write_with_verify(ICM53611_REG_GYRO_CONFIG1, cfg->gyro_config1,
                                        ICM53611_GYRO_CONFIG1_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    imu_spi_error_checkpoint();

    status = icm53611_write_with_verify(ICM53611_REG_ACCEL_CONFIG1, cfg->accel_config1,
                                        ICM53611_ACCEL_CONFIG1_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    imu_spi_error_checkpoint();

    status = icm53611_configure_fifo_threshold_int1_only();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    imu_spi_error_checkpoint();

    if (result != NULL) result->stage = ICM53611_CAPTURE_FLUSH;
    status = (result != NULL) ? icm53611_capture_fifo_flush(result) :
                               icm53611_fifo_flush_impl(NULL, NULL);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    imu_spi_error_checkpoint();

    if (result != NULL) result->stage = ICM53611_CAPTURE_FIFO;
    status = icm53611_fifo_config(fifo_cfg);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    imu_spi_error_checkpoint();

    if (result != NULL) result->stage = ICM53611_CAPTURE_MODE;
    status = icm53611_write_with_verify(ICM53611_REG_PWR_MGMT0, cfg->pwr_mgmt0,
                                        ICM53611_PWR_MGMT0_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    imu_spi_error_checkpoint();

    imu_bsp_delay_us(ICM53611_PWR_READY_WAIT_US);
    return IMU_STATUS_OK;
}

imu_status_t icm53611_configure_packet3_fifo_stream(const icm53611_cfg_t *cfg,
                                                    const icm53611_fifo_cfg_t *fifo_cfg)
{
    return icm53611_configure_packet3_fifo_stream_impl(cfg, fifo_cfg, NULL);
}

imu_status_t icm53611_prepare_capture(const icm53611_cfg_t *cfg,
                                     const icm53611_fifo_cfg_t *fifo,
                                     icm53611_capture_result_t *result)
{
    imu_status_t status;
    uint8_t reset_done = 0U;
    if (cfg == NULL || fifo == NULL || result == NULL) return IMU_STATUS_INVALID_PARAM;
    memset(result, 0, sizeof(*result));
    result->flush_end = ICM53611_FLUSH_NOT_RUN;
    imu_spi_error_begin();
    result->stage = ICM53611_CAPTURE_RESET;
    status = icm53611_soft_reset();
    if (status != IMU_STATUS_OK) goto finish;
    imu_spi_error_checkpoint();
    result->stage = ICM53611_CAPTURE_INTERFACE;
    status = icm53611_write_with_verify(ICM53611_REG_DEVICE_CONFIG,
        ICM53611_DEVICE_CONFIG_SPI_AP_4WIRE, ICM53611_DEVICE_CONFIG_VERIFY_MASK);
    if (status != IMU_STATUS_OK) goto finish;
    imu_spi_error_checkpoint();
    /* TDK inv_imu_soft_reset: consume RESET_DONE before configuring a new session. */
    result->stage = ICM53611_CAPTURE_RESET_DONE;
    status = imu_spi_read_reg(ICM53611_REG_INT_STATUS, &reset_done);
    if (status != IMU_STATUS_OK) goto finish;
    if (!(reset_done & ICM53611_INT_STATUS_RESET_DONE_INT))
    {
        status = IMU_STATUS_VERIFY_FAILED;
        imu_spi_error_record(3U, 0U, ICM53611_REG_INT_STATUS, reset_done, true, status);
        goto finish;
    }
    result->stage = ICM53611_CAPTURE_CONFIG;
    status = icm53611_configure_packet3_fifo_stream_impl(cfg, fifo, result);
finish:
    result->status = (uint8_t)status;
    imu_spi_error_end(&result->io);
    return status;
}

static void icm53611_ois_prearm_result_init(icm53611_ois_prearm_result_t *result)
{
    memset(result, 0, sizeof(*result));
    result->before = 0xFFU;
    result->after = 0xFFU;
    result->status = IMU_STATUS_OK;
}

imu_status_t icm53611_ois_prearm_config_disable_clear(
    icm53611_ois_prearm_result_t *result)
{
    imu_status_t status;
    uint8_t sensor_config3 = 0U;
    uint64_t total_start_us;
    uint64_t step_start_us;
    uint64_t step_end_us;

    if (result == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    icm53611_ois_prearm_result_init(result);
    total_start_us = imu_bsp_local_timestamp_us();

    step_start_us = imu_bsp_local_timestamp_us();
    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1,
                               ICM53611_MREG1_REG_SENSOR_CONFIG3,
                               &sensor_config3);
    step_end_us = imu_bsp_local_timestamp_us();
    result->read_us = icm53611_diff_u64_to_u32(step_end_us, step_start_us);
    if (status != IMU_STATUS_OK)
    {
        goto done;
    }
    result->before = sensor_config3;
    result->after = sensor_config3;

    if ((sensor_config3 & ICM53611_MREG1_SENSOR_CONFIG3_OIS_CONFIG_DISABLE) == 0U)
    {
        result->skip = 1U;
        goto done;
    }

    sensor_config3 &= (uint8_t)(~ICM53611_MREG1_SENSOR_CONFIG3_OIS_CONFIG_DISABLE);
    step_start_us = imu_bsp_local_timestamp_us();
    status = icm53611_mreg1_write_verify_readback(
                 "SENSOR_CONFIG3_PREARM",
                 ICM53611_MREG1_REG_SENSOR_CONFIG3,
                 sensor_config3,
                 ICM53611_MREG1_SENSOR_CONFIG3_V0_VERIFY_MASK,
                 0U,
                 &result->after);
    step_end_us = imu_bsp_local_timestamp_us();
    result->write_verify_us = icm53611_diff_u64_to_u32(step_end_us, step_start_us);

done:
    step_end_us = imu_bsp_local_timestamp_us();
    result->status = status;
    result->total_us = icm53611_diff_u64_to_u32(step_end_us, total_start_us);
    return status;
}

imu_status_t icm53611_ois_prearm_timestamp_regs(
    icm53611_ois_prearm_result_t *result)
{
    imu_status_t status;
    uint8_t tmst_cfg1 = 0U;
    uint8_t desired;
    uint64_t total_start_us;
    uint64_t step_start_us;
    uint64_t step_end_us;

    if (result == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    icm53611_ois_prearm_result_init(result);
    total_start_us = imu_bsp_local_timestamp_us();

    step_start_us = imu_bsp_local_timestamp_us();
    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1,
                               ICM53611_MREG1_REG_TMST_CONFIG1,
                               &tmst_cfg1);
    step_end_us = imu_bsp_local_timestamp_us();
    result->read_us = icm53611_diff_u64_to_u32(step_end_us, step_start_us);
    if (status != IMU_STATUS_OK)
    {
        goto done;
    }
    result->before = tmst_cfg1;
    result->after = tmst_cfg1;

    desired = icm53611_ois_timestamp_desired_value(tmst_cfg1);
    if (tmst_cfg1 == desired)
    {
        result->skip = 1U;
        goto done;
    }

    step_start_us = imu_bsp_local_timestamp_us();
    status = icm53611_mreg1_write_verify_readback(
                 "TMST_CONFIG1_PREARM",
                 ICM53611_MREG1_REG_TMST_CONFIG1,
                 desired,
                 ICM53611_MREG1_TMST_CONFIG1_PACKET3_VERIFY_MASK,
                 (uint8_t)(desired & ICM53611_MREG1_TMST_CONFIG1_PACKET3_VERIFY_MASK),
                 &result->after);
    step_end_us = imu_bsp_local_timestamp_us();
    result->write_verify_us = icm53611_diff_u64_to_u32(step_end_us, step_start_us);

done:
    step_end_us = imu_bsp_local_timestamp_us();
    result->status = status;
    result->total_us = icm53611_diff_u64_to_u32(step_end_us, total_start_us);
    return status;
}

imu_status_t icm53611_ois_fast_enable_from_current_bus(void)
{
    imu_status_t status = IMU_STATUS_OK;
    uint8_t sensor_config3 = 0U;
    const icm53611_cfg_t *cfg = icm53611_default_config();

    /* ABA fast switch relies on the current UI FIFO packet settings staying intact. */
    status = icm53611_write_with_verify(ICM53611_REG_GYRO_CONFIG0, cfg->gyro_config0,
                                        ICM53611_GYRO_CONFIG0_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        goto done;
    }

    status = icm53611_write_with_verify(ICM53611_REG_ACCEL_CONFIG0, cfg->accel_config0,
                                        ICM53611_ACCEL_CONFIG0_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        goto done;
    }

    status = icm53611_write_with_verify(ICM53611_REG_GYRO_CONFIG1, cfg->gyro_config1,
                                        ICM53611_GYRO_CONFIG1_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        goto done;
    }

    status = icm53611_write_with_verify(ICM53611_REG_ACCEL_CONFIG1, cfg->accel_config1,
                                        ICM53611_ACCEL_CONFIG1_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        goto done;
    }

    status = icm53611_write_with_verify(ICM53611_REG_PWR_MGMT0,
                                        cfg->pwr_mgmt0,
                                        ICM53611_PWR_MGMT0_VERIFY_MASK);
    if (status != IMU_STATUS_OK)
    {
        goto done;
    }

    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1,
                               ICM53611_MREG1_REG_SENSOR_CONFIG3,
                               &sensor_config3);
    if (status != IMU_STATUS_OK)
    {
        goto done;
    }
    if ((sensor_config3 & ICM53611_MREG1_SENSOR_CONFIG3_OIS_CONFIG_DISABLE) != 0U)
    {
        sensor_config3 &= (uint8_t)(~ICM53611_MREG1_SENSOR_CONFIG3_OIS_CONFIG_DISABLE);
        status = imu_spi_mreg_write(ICM53611_MREG_BLOCK_1,
                                    ICM53611_MREG1_REG_SENSOR_CONFIG3,
                                    sensor_config3);
        if (status != IMU_STATUS_OK)
        {
            goto done;
        }
        status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1,
                                   ICM53611_MREG1_REG_SENSOR_CONFIG3,
                                   &sensor_config3);
        if (status != IMU_STATUS_OK)
        {
            goto done;
        }
        if ((sensor_config3 & ICM53611_MREG1_SENSOR_CONFIG3_OIS_CONFIG_DISABLE) != 0U)
        {
            status = IMU_STATUS_VERIFY_FAILED;
            goto done;
        }
    }

    status = icm53611_enable_ois_timestamp_regs(NULL);
    if (status != IMU_STATUS_OK)
    {
        goto done;
    }
    icm53611_reset_ois_tmst_tracker();
    imu_bsp_delay_us(ZY100_FINAL_EDGE_OIS_READY_WAIT_US);

done:
    return status;
}

imu_status_t icm53611_fifo_config(const icm53611_fifo_cfg_t *cfg)
{
    imu_status_t status;
    uint8_t fifo_cfg2;
    uint8_t fifo_cfg3;
    uint8_t tmst_cfg1 = 0U;
    uint8_t fifo_cfg5 = 0U;
    uint8_t fifo_cfg5_clear;
    uint8_t tmst_expected;
    uint8_t fifo_expected;

    if (cfg == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    fifo_cfg2 = (uint8_t)(cfg->watermark & 0xFFU);
    fifo_cfg3 = (uint8_t)((cfg->watermark >> 8) & 0x0FU);

    status = icm53611_write_with_verify(ICM53611_REG_FIFO_CONFIG1,
                                        ICM53611_FIFO_CONFIG1_FIFO_BYPASS,
                                        0x03U);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    if (!cfg->enable)
    {
        return IMU_STATUS_OK;
    }

    status = icm53611_write_with_verify(ICM53611_REG_FIFO_CONFIG2, fifo_cfg2, 0xFFU);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = icm53611_write_with_verify(ICM53611_REG_FIFO_CONFIG3, fifo_cfg3, 0x0FU);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    /*
     * Timestamp path is controlled in MREG1 (TMST_CONFIG1/FIFO_CONFIG5).
     * Keep this in the low-level framework and avoid hard-coding higher-level packet policy.
     */
    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1, ICM53611_MREG1_REG_TMST_CONFIG1, &tmst_cfg1);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    if (cfg->timestamp_enable)
    {
        tmst_cfg1 |= ICM53611_MREG1_TMST_CONFIG1_TMST_EN;
    }
    else
    {
        tmst_cfg1 &= (uint8_t)~ICM53611_MREG1_TMST_CONFIG1_TMST_EN;
    }
    tmst_cfg1 &= (uint8_t)~(ICM53611_MREG1_TMST_CONFIG1_TMST_DELTA_EN |
                            ICM53611_MREG1_TMST_CONFIG1_TMST_RES);
    tmst_expected = cfg->timestamp_enable ? ICM53611_MREG1_TMST_CONFIG1_TMST_EN : 0U;
    status = icm53611_mreg1_write_verify("TMST_CONFIG1",
                                          ICM53611_MREG1_REG_TMST_CONFIG1,
                                          tmst_cfg1,
                                          ICM53611_MREG1_TMST_CONFIG1_V0_VERIFY_MASK,
                                          tmst_expected);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1, ICM53611_MREG1_REG_FIFO_CONFIG5, &fifo_cfg5);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    fifo_cfg5_clear = (ICM53611_MREG1_FIFO_CONFIG5_FIFO_RESUME_PARTIAL_RD |
                       ICM53611_MREG1_FIFO_CONFIG5_FIFO_HIRES_EN |
                       ICM53611_MREG1_FIFO_CONFIG5_FIFO_TMST_FSYNC_EN |
                       ICM53611_MREG1_FIFO_CONFIG5_FIFO_GYRO_EN |
                       ICM53611_MREG1_FIFO_CONFIG5_FIFO_ACCEL_EN);
    fifo_cfg5 &= (uint8_t)~fifo_cfg5_clear;
    fifo_expected = 0U;
    if (cfg->accel_enable)
    {
        fifo_cfg5 |= ICM53611_MREG1_FIFO_CONFIG5_FIFO_ACCEL_EN;
        fifo_expected |= ICM53611_MREG1_FIFO_CONFIG5_FIFO_ACCEL_EN;
    }
    if (cfg->gyro_enable)
    {
        fifo_cfg5 |= ICM53611_MREG1_FIFO_CONFIG5_FIFO_GYRO_EN;
        fifo_expected |= ICM53611_MREG1_FIFO_CONFIG5_FIFO_GYRO_EN;
    }
    status = icm53611_mreg1_write_verify("FIFO_CONFIG5",
                                          ICM53611_MREG1_REG_FIFO_CONFIG5,
                                          fifo_cfg5,
                                          ICM53611_MREG1_FIFO_CONFIG5_V0_VERIFY_MASK,
                                          fifo_expected);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    return icm53611_write_with_verify(ICM53611_REG_FIFO_CONFIG1,
                                      ICM53611_FIFO_CONFIG1_STREAM_TO_FIFO,
                                      0x03U);
}

static imu_status_t icm53611_fifo_get_count_impl(
    uint16_t *count, bool report_busy)
{
    imu_status_t status;
    uint8_t count_l_latched;
    uint8_t count_h;
    uint8_t count_l;

    if (count == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    /* Per datasheet, reading FIFO_COUNTL latches FIFO_COUNTH/FIFO_COUNTL. */
    status = report_busy ?
        imu_spi_read_reg_ex(ICM53611_REG_FIFO_COUNTL, &count_l_latched) :
        imu_spi_read_reg(ICM53611_REG_FIFO_COUNTL, &count_l_latched);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = report_busy ?
        imu_spi_read_reg_ex(ICM53611_REG_FIFO_COUNTH, &count_h) :
        imu_spi_read_reg(ICM53611_REG_FIFO_COUNTH, &count_h);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = report_busy ?
        imu_spi_read_reg_ex(ICM53611_REG_FIFO_COUNTL, &count_l) :
        imu_spi_read_reg(ICM53611_REG_FIFO_COUNTL, &count_l);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    IMU_UNUSED(count_l_latched);
    *count = (uint16_t)(((uint16_t)count_h << 8) | count_l);
    return IMU_STATUS_OK;
}

imu_status_t icm53611_fifo_get_count(uint16_t *count)
{
    return icm53611_fifo_get_count_impl(count, false);
}

imu_status_t icm53611_fifo_get_count_ex(uint16_t *count)
{
    return icm53611_fifo_get_count_impl(count, true);
}

static imu_status_t icm53611_fifo_get_lost_count_impl(
    uint16_t *lost_count, bool report_busy)
{
    imu_status_t status;
    uint8_t lost_h;
    uint8_t lost_l;

    if (lost_count == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = report_busy ?
        imu_spi_read_reg_ex(ICM53611_REG_FIFO_LOST_PKT0, &lost_h) :
        imu_spi_read_reg(ICM53611_REG_FIFO_LOST_PKT0, &lost_h);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = report_busy ?
        imu_spi_read_reg_ex(ICM53611_REG_FIFO_LOST_PKT1, &lost_l) :
        imu_spi_read_reg(ICM53611_REG_FIFO_LOST_PKT1, &lost_l);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    *lost_count = (uint16_t)(((uint16_t)lost_h << 8) | lost_l);
    return IMU_STATUS_OK;
}

imu_status_t icm53611_fifo_get_lost_count(uint16_t *lost_count)
{
    return icm53611_fifo_get_lost_count_impl(lost_count, false);
}

imu_status_t icm53611_fifo_get_lost_count_ex(uint16_t *lost_count)
{
    return icm53611_fifo_get_lost_count_impl(lost_count, true);
}

static imu_status_t icm53611_fifo_read_impl(
    uint8_t *buf, uint16_t len, bool report_busy)
{
    if ((buf == NULL) || (len == 0U))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    return report_busy ? imu_spi_read_regs_ex(ICM53611_REG_FIFO_DATA, buf, len) :
                         imu_spi_read_regs(ICM53611_REG_FIFO_DATA, buf, len);
}

imu_status_t icm53611_fifo_read(uint8_t *buf, uint16_t len)
{
    return icm53611_fifo_read_impl(buf, len, false);
}

imu_status_t icm53611_fifo_read_ex(uint8_t *buf, uint16_t len)
{
    return icm53611_fifo_read_impl(buf, len, true);
}

imu_status_t icm53611_fifo_set_bypass(bool enable)
{
    imu_status_t status;
    uint8_t fifo_config1 = 0U;
    uint8_t readback = 0U;
    uint8_t expected;

    status = imu_spi_read_reg(ICM53611_REG_FIFO_CONFIG1, &fifo_config1);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    if (enable)
    {
        fifo_config1 |= ICM53611_FIFO_CONFIG1_FIFO_BYPASS;
        expected = ICM53611_FIFO_CONFIG1_FIFO_BYPASS;
    }
    else
    {
        fifo_config1 &= (uint8_t)(~ICM53611_FIFO_CONFIG1_FIFO_BYPASS);
        expected = 0U;
    }

    status = imu_spi_write_reg(ICM53611_REG_FIFO_CONFIG1, fifo_config1);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = imu_spi_read_reg(ICM53611_REG_FIFO_CONFIG1, &readback);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    if ((readback & ICM53611_FIFO_CONFIG1_FIFO_BYPASS) != expected)
    {
        return IMU_STATUS_VERIFY_FAILED;
    }

    return IMU_STATUS_OK;
}

static imu_status_t icm53611_fifo_flush_impl(uint16_t *fifo_count_after,
                                              icm53611_capture_result_t *result)
{
    imu_spi_clock_t clock;
    imu_status_t status, acquire_status, release_status;
    uint8_t reset_reg = 0U, sample = 0U;
    uint16_t count = 0U;
    uint32_t polls = 0U, elapsed = 0U;
    uint32_t start = (uint32_t)imu_bsp_local_timestamp_us();
    bool read_valid = false;
    IMU_UNUSED(read_valid);

    pwrd_stage(25U); /* clock lease */
    acquire_status = imu_spi_clock_begin(&clock);
    status = acquire_status;
    if (status != IMU_STATUS_OK) goto finish;
    pwrd_stage(26U); /* trigger */
    status = imu_spi_write_reg(ICM53611_REG_SIGNAL_PATH_RESET,
                               ICM53611_SIGNAL_PATH_RESET_FIFO_FLUSH);
    if (status != IMU_STATUS_OK) goto finish;
    start = (uint32_t)imu_bsp_local_timestamp_us();
    imu_bsp_delay_us(ICM53611_FIFO_FLUSH_WAIT_US);
    pwrd_stage(27U); /* completion polling */
    for (;;)
    {
        status = imu_spi_read_reg(ICM53611_REG_SIGNAL_PATH_RESET, &sample);
        polls++;
        elapsed = (uint32_t)imu_bsp_local_timestamp_us() - start;
        if (status != IMU_STATUS_OK) break;
        reset_reg = sample;
        read_valid = true;
        if (!(reset_reg & ICM53611_SIGNAL_PATH_RESET_FIFO_FLUSH))
        {
            pwrd_stage(28U); /* count read, caller decides whether zero is required */
            if (fifo_count_after != NULL)
            {
                status = icm53611_fifo_get_count(&count);
                if (status == IMU_STATUS_OK) *fifo_count_after = count;
            }
            break;
        }
        /* Real elapsed time includes bus work/preemption; iteration bound also
         * prevents a broken local clock from producing an infinite loop. */
        if (elapsed >= ICM53611_FIFO_FLUSH_TIMEOUT_US ||
            polls >= ICM53611_FIFO_FLUSH_TIMEOUT_US / ICM53611_FIFO_FLUSH_POLL_US)
        {
            status = IMU_STATUS_TIMEOUT;
            imu_spi_error_record(3U, 0U, ICM53611_REG_SIGNAL_PATH_RESET,
                                 reset_reg, true, status);
            pwrd_fault(PWRD_IO_FLUSH_TIMEOUT, 0U, ICM53611_REG_SIGNAL_PATH_RESET,
                       reset_reg, true, status);
            break;
        }
        imu_bsp_delay_us(ICM53611_FIFO_FLUSH_POLL_US);
    }
finish:
    elapsed = (uint32_t)imu_bsp_local_timestamp_us() - start;
    pwrd_stage(29U); /* release only the request owned by this lease */
    release_status = imu_spi_clock_end(&clock);
    pwrd_io_detail(elapsed, polls, acquire_status, release_status, reset_reg,
                   6U | (read_valid ? 1U : 0U) | (polls ? 8U : 0U));
    if (result != NULL)
    {
        result->flush_us = elapsed;
        result->flush_polls = (uint16_t)polls;
    }
    return status != IMU_STATUS_OK ? status : release_status;
}

imu_status_t icm53611_fifo_flush_wait_clear(uint16_t *fifo_count_after)
{
    return icm53611_fifo_flush_impl(fifo_count_after, NULL);
}

imu_status_t icm53611_fifo_flush(void)
{
    return icm53611_fifo_flush_wait_clear(NULL);
}

imu_status_t icm53611_prepare_for_sleep(void)
{
    imu_status_t status = icm53611_wom_stop_clocked_minimal();
    if (status == IMU_STATUS_OK) status = icm53611_wom_dlps_disable_fifo_tmst();
    if (status == IMU_STATUS_OK) status = icm53611_wom_ui_write(ICM53611_REG_PWR_MGMT0, 0U);
    return status;
}

imu_status_t icm53611_set_wom_config(uint8_t wom_cfg)
{
    return icm53611_wom_set_config(wom_cfg);
}

imu_status_t icm53611_wom_set_threshold_code(uint8_t threshold_code)
{
    return icm53611_wom_verify_threshold_triplet(threshold_code);
}

imu_status_t icm53611_wom_set_config(uint8_t wom_cfg)
{
    imu_status_t status;
    uint8_t write_value = 0U;
    uint8_t readback = 0U;

    status = icm53611_update_reg_bits_readback(ICM53611_REG_WOM_CONFIG,
                                               ICM53611_WOM_CONFIG_FIELD_MASK,
                                               wom_cfg,
                                               &write_value,
                                               &readback);
#if IMU_WOM_RUNTIME_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE
    DBG_DIRECT("[IMU_WOM_CFG_VERIFY] reg=WOM_CONFIG addr=0x%02x write=0x%02x read=0x%02x result=%s",
               ICM53611_REG_WOM_CONFIG,
               write_value,
               readback,
               icm53611_pass_fail(status == IMU_STATUS_OK));
#elif IMU_ERROR_LOG_ENABLE
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[WARN][IMU] wom_config_verify_failed status=%d read=0x%02x",
                   status,
                   readback);
    }
#endif
    return status;
}

imu_status_t icm53611_wom_route_int1(bool enable)
{
    const uint8_t route_bits = enable ? ICM53611_INT_SOURCE1_WOM_INT1_EN_MASK : 0U;
    imu_status_t status;
    uint8_t write_value = 0U;
    uint8_t readback = 0U;

    status = icm53611_update_reg_bits_readback(ICM53611_REG_INT_SOURCE1,
                                               ICM53611_INT_SOURCE1_WOM_INT1_EN_MASK,
                                               route_bits,
                                               &write_value,
                                               &readback);
    IMU_UNUSED(write_value);
    if (enable)
    {
#if IMU_WOM_RUNTIME_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE
        DBG_DIRECT("[IMU_WOM_ROUTE_VERIFY] reg=INT_SOURCE1 addr=0x%02x write_mask=0x%02x "
                   "read=0x%02x route_xyz=%u result=%s",
                   ICM53611_REG_INT_SOURCE1,
                   ICM53611_INT_SOURCE1_WOM_INT1_EN_MASK,
                   readback,
                   1U,
                   icm53611_pass_fail(status == IMU_STATUS_OK));
#elif IMU_ERROR_LOG_ENABLE
        if (status != IMU_STATUS_OK)
        {
            DBG_DIRECT("[WARN][IMU] wom_route_verify_failed status=%d read=0x%02x",
                       status,
                       readback);
        }
#endif
    }
    return status;
}

imu_status_t icm53611_wom_disable(void)
{
    imu_status_t status;

    status = icm53611_wom_route_int1(false);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    return icm53611_update_reg_bits(ICM53611_REG_WOM_CONFIG,
                                    ICM53611_WOM_CONFIG_WOM_EN,
                                    0U);
}

static imu_status_t icm53611_wom_dlps_wait_mclk_ready(void)
{
    imu_status_t status;
    uint8_t mclk = 0U;
    uint32_t waited_us = 0U;

    do
    {
        status = imu_spi_read_reg(ICM53611_REG_MCLK_RDY, &mclk);
        if (status != IMU_STATUS_OK)
        {
            return status;
        }
        if ((mclk & ICM53611_MCLK_RDY_RUNNING_MASK) != 0U)
        {
            return IMU_STATUS_OK;
        }
        imu_bsp_delay_us(ICM53611_WOM_DLPS_MCLK_READY_POLL_US);
        waited_us += ICM53611_WOM_DLPS_MCLK_READY_POLL_US;
    }
    while (waited_us < ICM53611_WOM_DLPS_MCLK_READY_TIMEOUT_US);

    IMU_LOG_ERROR("WOM MCLK=%02x", mclk);
    return IMU_STATUS_TIMEOUT;
}

/* WOM configuration follows the vendor read/modify/write sequence. Keep
 * mandatory timing here; configuration readback is not a write prerequisite. */
static imu_status_t icm53611_wom_ui_write(uint8_t reg, uint8_t value)
{
    imu_status_t status = imu_spi_write_reg(reg, value);
    if (status == IMU_STATUS_OK && reg == ICM53611_REG_PWR_MGMT0)
        imu_bsp_delay_us(200U); /* DS pp59-60: OFF->ON register-write exclusion. */
    return status;
}

static imu_status_t icm53611_wom_ui_bits(uint8_t reg, uint8_t mask, uint8_t bits)
{
    uint8_t value = 0U;
    imu_status_t status = imu_spi_read_reg(reg, &value);
    if (status != IMU_STATUS_OK) return status;
    value = (uint8_t)((value & (uint8_t)~mask) | (bits & mask));
    return icm53611_wom_ui_write(reg, value);
}

static imu_status_t icm53611_wom_check_ui(uint32_t token, uint8_t reg,
                                        uint8_t mask, uint8_t expected)
{
    uint8_t value = 0U;
    imu_status_t status = imu_spi_read_reg(reg, &value);
    icm53611_pwrd_read(reg, value, status);
    pwrd_verify(token, 0U, reg, value, mask, expected, status);
    IMU_UNUSED(token);
    if (status != IMU_STATUS_OK) return status;
    return (value & mask) == (expected & mask) ? IMU_STATUS_OK : IMU_STATUS_VERIFY_FAILED;
}

static uint8_t icm53611_wom_dlps_threshold_code(uint16_t threshold_mg)
{
    uint32_t code;

    if (threshold_mg == 0U)
    {
        return 1U;
    }

    code = (((uint32_t)threshold_mg * 256U) + 500U) / 1000U;
    if (code == 0U)
    {
        code = 1U;
    }
    if (code > 0xFFU)
    {
        code = 0xFFU;
    }
    return (uint8_t)code;
}

static imu_status_t icm53611_wom_dlps_odr_code(uint16_t odr_hz, uint8_t *odr_code)
{
    if (odr_code == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    switch (odr_hz)
    {
    case 25U:
        *odr_code = ICM53611_ACCEL_ODR_25HZ;
        return IMU_STATUS_OK;
    case 50U:
        *odr_code = ICM53611_ACCEL_ODR_50HZ;
        return IMU_STATUS_OK;
    case 100U:
        *odr_code = ICM53611_ACCEL_ODR_100HZ;
        return IMU_STATUS_OK;
    default:
        return IMU_STATUS_INVALID_PARAM;
    }
}

static uint32_t icm53611_wom_dlps_odr_period_us_from_code(uint8_t odr_code)
{
    switch (odr_code & 0x0FU)
    {
    case ICM53611_ACCEL_ODR_25HZ:
        return ICM53611_WOM_DLPS_ODR_25HZ_PERIOD_US;
    case ICM53611_ACCEL_ODR_50HZ:
        return ICM53611_WOM_DLPS_ODR_50HZ_PERIOD_US;
    case ICM53611_ACCEL_ODR_100HZ:
        return ICM53611_WOM_DLPS_ODR_100HZ_PERIOD_US;
    /* DS-000618 p62: legal LP ODRs only; codes 5/6 are LN-only. */
    case 7U: return 2500U;   /* 400 Hz */
    case 8U: return 5000U;   /* 200 Hz */
    case 12U: return 80000U; /* 12.5 Hz */
    case 13U: return 160000U;
    case 14U: return 320000U;
    case 15U: return 640000U;
    default:
        return 0U;
    }
}

static uint8_t icm53611_wom_dlps_duration_bits(uint8_t duration_samples)
{
    uint8_t code;

    if (duration_samples <= 1U)
    {
        code = 0U;
    }
    else if (duration_samples == 2U)
    {
        code = 1U;
    }
    else if (duration_samples == 3U)
    {
        code = 2U;
    }
    else
    {
        code = 3U;
    }

    return (uint8_t)(code << ICM53611_WOM_CONFIG_INT_DUR_SHIFT);
}

static uint8_t icm53611_wom_dlps_config_value(const icm53611_wom_dlps_wake_cfg_t *cfg)
{
    uint8_t wom_cfg = ICM53611_WOM_CONFIG_WOM_EN;

    wom_cfg |= icm53611_wom_dlps_duration_bits(cfg->duration_samples);
    if (cfg->previous_sample_mode)
    {
        wom_cfg |= ICM53611_WOM_CONFIG_WOM_MODE;
    }
    return wom_cfg;
}

static imu_status_t icm53611_wom_dlps_config_int1_active_low_latched(void)
{
    return icm53611_wom_ui_bits(ICM53611_REG_INT_CONFIG,
                ICM53611_INT_CONFIG_VERIFY_MASK,
                ICM53611_INT_CONFIG_INT1_MODE | ICM53611_INT_CONFIG_INT1_DRIVE_CIRCUIT);
}

static imu_status_t icm53611_wom_dlps_set_pure_int1_route(void)
{
    /* Other routes are already cleared by complete cleanup. Preserve reserved bits. */
    return icm53611_wom_ui_bits(ICM53611_REG_INT_SOURCE1,
               ICM53611_INT_SOURCE1_VERIFY_MASK, ICM53611_INT_SOURCE1_WOM_INT1_EN_MASK);
}

static imu_status_t icm53611_wom_dlps_clear_status(void)
{
    icm53611_int_status_t status_snapshot;

    return icm53611_read_int_status(&status_snapshot);
}

static imu_status_t icm53611_wom_dlps_disable_fifo_tmst(void)
{
    imu_status_t status;
#define CLEAN(call) do { status = (call); if (status != IMU_STATUS_OK) return status; } while (0)
    pwrd_stage(20U);
    CLEAN(icm53611_wom_ui_bits(ICM53611_REG_WOM_CONFIG, ICM53611_WOM_CONFIG_WOM_EN, 0U));
    CLEAN(icm53611_wom_ui_bits(ICM53611_REG_INT_SOURCE0, ICM53611_INT_SOURCE0_VERIFY_MASK, 0U));
    CLEAN(icm53611_wom_ui_bits(ICM53611_REG_INT_SOURCE3, ICM53611_INT_SOURCE3_VERIFY_MASK, 0U));
    CLEAN(icm53611_wom_ui_bits(ICM53611_REG_INT_SOURCE4, ICM53611_INT_SOURCE4_VERIFY_MASK, 0U));
    pwrd_stage(21U);
    CLEAN(icm53611_wom_ui_bits(ICM53611_REG_FIFO_CONFIG1, 3U, ICM53611_FIFO_CONFIG1_FIFO_BYPASS));
    /* Pure WOM does not consume FIFO data. Acquisition owns reset/flush;
     * residual count is deliberately not a prerequisite for sleep or WOM. */
    pwrd_stage(22U);
    CLEAN(icm53611_wom_mreg_bits_minimal(ICM53611_MREG1_REG_FIFO_CONFIG5, ICM53611_MREG1_FIFO_CONFIG5_V0_VERIFY_MASK, 0U));
    pwrd_stage(23U);
    CLEAN(icm53611_wom_mreg_bits_minimal(ICM53611_MREG1_REG_TMST_CONFIG1, ICM53611_MREG1_TMST_CONFIG1_PACKET3_VERIFY_MASK, 0U));
    pwrd_stage(24U);
    CLEAN(icm53611_wom_mreg_bits_minimal(ICM53611_MREG1_REG_SENSOR_CONFIG3, ICM53611_MREG1_SENSOR_CONFIG3_OIS_CONFIG_DISABLE, ICM53611_MREG1_SENSOR_CONFIG3_OIS_CONFIG_DISABLE));
    CLEAN(icm53611_wom_ui_bits(ICM53611_REG_APEX_CONFIG1, ICM53611_APEX_FEATURE_MASK, 0U));
    CLEAN(icm53611_wom_mreg_bits_minimal(ICM53611_MREG1_REG_SELFTEST, ICM53611_SELFTEST_ENABLE_MASK, 0U));
    CLEAN(icm53611_wom_mreg_bits_minimal(ICM53611_MREG1_REG_INT_SOURCE6, ICM53611_APEX_ROUTE_MASK, 0U));
    CLEAN(icm53611_wom_mreg_bits_minimal(ICM53611_MREG1_REG_INT_SOURCE7, ICM53611_APEX_ROUTE_MASK, 0U));
#undef CLEAN
    return IMU_STATUS_OK;
}

static imu_status_t icm53611_wom_dlps_exit_wuosc_if_needed(uint8_t pwr_mgmt0, uint8_t accel_config0)
{
    uint8_t fifo6 = 0U;
    uint32_t period = icm53611_wom_dlps_odr_period_us_from_code(accel_config0);
    bool lp = (pwr_mgmt0 & ICM53611_PWR_MGMT0_ACCEL_MODE_MASK) == ICM53611_PWR_MGMT0_ACCEL_MODE_LP;
    bool wu = lp && !(pwr_mgmt0 & ICM53611_PWR_MGMT0_ACCEL_LP_CLK_SEL);
    imu_status_t status;
    if (lp && !period) return IMU_STATUS_INVALID_PARAM;
    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1, ICM53611_MREG1_REG_FIFO_CONFIG6, &fifo6);
    if (status != IMU_STATUS_OK) return status;
    if (!(fifo6 & ICM53611_FIFO_RC_REQ_DISABLE))
    {
        status = icm53611_mreg1_write_verify("WOM", ICM53611_MREG1_REG_FIFO_CONFIG6,
                    fifo6 | ICM53611_FIFO_RC_REQ_DISABLE, ICM53611_FIFO_RC_REQ_DISABLE, ICM53611_FIFO_RC_REQ_DISABLE);
        if (status != IMU_STATUS_OK) return status;
        if (wu) imu_bsp_delay_us(period);
    }
    if (!wu)
    {
        /* A previous transition may have failed after its write; no proven RC age. */
        if (lp) imu_bsp_delay_us(period);
        return IMU_STATUS_OK;
    }
    status = icm53611_write_with_verify(ICM53611_REG_PWR_MGMT0,
                    pwr_mgmt0 | ICM53611_PWR_MGMT0_ACCEL_LP_CLK_SEL, ICM53611_PWR_MGMT0_WOM_SLEEP_VERIFY_MASK);
    if (status == IMU_STATUS_OK) imu_bsp_delay_us(period);
    return status;
}

static imu_status_t icm53611_wom_mreg_bits(uint8_t reg, uint8_t mask, uint8_t bits)
{
    uint8_t value = 0U;
    imu_status_t status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1, reg, &value);
    if (status != IMU_STATUS_OK) { pwrd_verify(0U, 1U, reg, value, mask, bits, status); return status; }
    value = (uint8_t)((value & (uint8_t)~mask) | (bits & mask));
    status = icm53611_mreg1_write_verify("WOM", reg, value, mask, bits);
    if (status != IMU_STATUS_OK) pwrd_verify(0U, 1U, reg, value, mask, bits, status);
    return status;
}

static imu_status_t icm53611_wom_stop_clocked(void)
{
    uint8_t pwr = 0U, odr = 0U;
    imu_status_t status = imu_spi_read_reg(ICM53611_REG_PWR_MGMT0, &pwr);
    if (status != IMU_STATUS_OK) return status;
    status = imu_spi_read_reg(ICM53611_REG_ACCEL_CONFIG0, &odr);
    if (status != IMU_STATUS_OK) return status;
    status = icm53611_wom_dlps_exit_wuosc_if_needed(pwr, odr);
    if (status != IMU_STATUS_OK) return status;
    /* No reliable start timestamp at this boundary: conservative 45ms minimum. */
    if (pwr & (ICM53611_PWR_MGMT0_GYRO_MODE_MASK | ICM53611_PWR_MGMT0_GYRO_OIS_EN))
        imu_bsp_delay_us(45000U);
    status = icm53611_write_with_verify(ICM53611_REG_PWR_MGMT0,
                 ICM53611_PWR_MGMT0_IDLE_EN, ICM53611_PWR_MGMT0_WOM_SLEEP_VERIFY_MASK);
    if (status != IMU_STATUS_OK) return status;
    return icm53611_wom_dlps_wait_mclk_ready();
}

static imu_status_t icm53611_wom_dlps_exit_wuosc_if_needed_minimal(uint8_t pwr_mgmt0, uint8_t accel_config0)
{
    uint8_t fifo6 = 0U;
    uint32_t period = icm53611_wom_dlps_odr_period_us_from_code(accel_config0);
    bool lp = (pwr_mgmt0 & ICM53611_PWR_MGMT0_ACCEL_MODE_MASK) == ICM53611_PWR_MGMT0_ACCEL_MODE_LP;
    bool wu = lp && !(pwr_mgmt0 & ICM53611_PWR_MGMT0_ACCEL_LP_CLK_SEL);
    imu_status_t status;
    if (lp && !period) return IMU_STATUS_INVALID_PARAM;
    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1, ICM53611_MREG1_REG_FIFO_CONFIG6, &fifo6);
    if (status != IMU_STATUS_OK) return status;
    if (!(fifo6 & ICM53611_FIFO_RC_REQ_DISABLE))
    {
        status = imu_spi_mreg_write(ICM53611_MREG_BLOCK_1, ICM53611_MREG1_REG_FIFO_CONFIG6,
                    fifo6 | ICM53611_FIFO_RC_REQ_DISABLE);
        if (status != IMU_STATUS_OK) return status;
        if (wu) imu_bsp_delay_us(period);
    }
    if (!wu)
    {
        /* A previous transition may have failed after its write; no proven RC age. */
        if (lp) imu_bsp_delay_us(period);
        return IMU_STATUS_OK;
    }
    status = icm53611_wom_ui_write(ICM53611_REG_PWR_MGMT0,
                    pwr_mgmt0 | ICM53611_PWR_MGMT0_ACCEL_LP_CLK_SEL);
    if (status == IMU_STATUS_OK) imu_bsp_delay_us(period);
    return status;
}

/* Pure WOM setup. DS-000618 pp44,59-60,64,66,82-87,92.
 * Disable unused paths; acquisition owns FIFO reset/flush and tail handling.
 * FIFO unused: retain RC request disable=1, as TDK's no-FIFO driver branch. */
static imu_status_t icm53611_wom_mreg_bits_minimal(uint8_t reg, uint8_t mask, uint8_t bits)
{
    uint8_t value = 0U;
    imu_status_t status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1, reg, &value);
    if (status != IMU_STATUS_OK) return status;
    value = (uint8_t)((value & (uint8_t)~mask) | (bits & mask));
    return imu_spi_mreg_write(ICM53611_MREG_BLOCK_1, reg, value);
}

static imu_status_t icm53611_wom_stop_clocked_minimal(void)
{
    uint8_t pwr = 0U, odr = 0U;
    imu_status_t status = imu_spi_read_reg(ICM53611_REG_PWR_MGMT0, &pwr);
    if (status != IMU_STATUS_OK) return status;
    status = imu_spi_read_reg(ICM53611_REG_ACCEL_CONFIG0, &odr);
    if (status != IMU_STATUS_OK) return status;
    status = icm53611_wom_dlps_exit_wuosc_if_needed_minimal(pwr, odr);
    if (status != IMU_STATUS_OK) return status;
    /* No reliable start timestamp at this boundary: conservative 45ms minimum. */
    if (pwr & (ICM53611_PWR_MGMT0_GYRO_MODE_MASK | ICM53611_PWR_MGMT0_GYRO_OIS_EN))
        imu_bsp_delay_us(45000U);
    status = icm53611_wom_ui_write(ICM53611_REG_PWR_MGMT0, ICM53611_PWR_MGMT0_IDLE_EN);
    if (status != IMU_STATUS_OK) return status;
    return icm53611_wom_dlps_wait_mclk_ready();
}



static imu_status_t icm53611_configure_wom_minimal(const icm53611_wom_dlps_wake_cfg_t *cfg)
{
    imu_status_t status = IMU_STATUS_INVALID_PARAM;
    uint8_t odr_code = 0U, threshold_code, wom_cfg;
    uint32_t token = pwrd_imu_begin(1U);
    IMU_UNUSED(token);
    if (cfg == NULL) goto done;
#define WOM_STEP(stage, call) do { \
    pwrd_stage(stage); status = (call); pwrd_step(token, stage, status); \
    if (status != IMU_STATUS_OK) goto done; \
} while (0)
    WOM_STEP(1U, icm53611_wom_dlps_odr_code(cfg->accel_odr_hz, &odr_code));
    WOM_STEP(2U, icm53611_init_bus());
    WOM_STEP(3U, icm53611_wom_check_ui(token, ICM53611_REG_WHO_AM_I,
                                     0xffU, ICM53611_WHO_AM_I_VALUE));
    /* Disable the previous detector before changing its configuration. */
    WOM_STEP(9U, icm53611_wom_ui_bits(ICM53611_REG_WOM_CONFIG,
                                     ICM53611_WOM_CONFIG_WOM_EN, 0U));
    WOM_STEP(5U, icm53611_wom_stop_clocked_minimal());
    WOM_STEP(8U, icm53611_wom_dlps_disable_fifo_tmst());
    WOM_STEP(10U, icm53611_wom_dlps_config_int1_active_low_latched());
    WOM_STEP(16U, icm53611_wom_dlps_set_pure_int1_route());
    threshold_code = icm53611_wom_dlps_threshold_code(cfg->threshold_mg);
    /* DS p44: indirect writes are single-byte accesses, with transport delays. */
    WOM_STEP(14U, imu_spi_mreg_write(ICM53611_MREG_BLOCK_1,
                                    ICM53611_REG_ACCEL_WOM_X_THR, threshold_code));
    WOM_STEP(14U, imu_spi_mreg_write(ICM53611_MREG_BLOCK_1,
                                    ICM53611_REG_ACCEL_WOM_Y_THR, threshold_code));
    WOM_STEP(14U, imu_spi_mreg_write(ICM53611_MREG_BLOCK_1,
                                    ICM53611_REG_ACCEL_WOM_Z_THR, threshold_code));
    wom_cfg = icm53611_wom_dlps_config_value(cfg);
    WOM_STEP(15U, icm53611_wom_ui_bits(ICM53611_REG_WOM_CONFIG,
                ICM53611_WOM_CONFIG_VERIFY_MASK, wom_cfg & (uint8_t)~ICM53611_WOM_CONFIG_WOM_EN));
    WOM_STEP(15U, icm53611_wom_ui_bits(ICM53611_REG_WOM_CONFIG,
                                     ICM53611_WOM_CONFIG_WOM_EN, ICM53611_WOM_CONFIG_WOM_EN));
    WOM_STEP(11U, icm53611_wom_ui_bits(ICM53611_REG_ACCEL_CONFIG0,
                ICM53611_ACCEL_CONFIG0_VERIFY_MASK, ICM53611_ACCEL_FSR_16G | odr_code));
    WOM_STEP(13U, icm53611_wom_ui_bits(ICM53611_REG_ACCEL_CONFIG1,
                ICM53611_ACCEL_CONFIG1_VERIFY_MASK, ICM53611_ACCEL_UI_AVG_4X | 1U));
    WOM_STEP(17U, icm53611_wom_dlps_clear_status());
    WOM_STEP(18U, icm53611_wom_ui_write(ICM53611_REG_PWR_MGMT0,
                ICM53611_PWR_MGMT0_ACCEL_MODE_LP | ICM53611_PWR_MGMT0_ACCEL_LP_CLK_SEL));
    imu_bsp_delay_us(icm53611_wom_dlps_odr_period_us_from_code(odr_code));
    WOM_STEP(18U, icm53611_wom_ui_write(ICM53611_REG_PWR_MGMT0,
                                      ICM53611_PWR_MGMT0_ACCEL_MODE_LP));
    /* One final power check, after all temporary MREG clock leases ended.
     * Stage 19 is reserved for the former full-register contract, not run here. */
    WOM_STEP(18U, icm53611_wom_check_ui(token, ICM53611_REG_PWR_MGMT0,
                ICM53611_PWR_MGMT0_WOM_SLEEP_VERIFY_MASK, ICM53611_PWR_MGMT0_ACCEL_MODE_LP));
#undef WOM_STEP
done:
    pwrd_imu_finish(token, status);
    return status;
}

imu_status_t icm53611_prepare_wom_wake_for_dlps(const icm53611_wom_dlps_wake_cfg_t *cfg)
{
    return icm53611_configure_wom_minimal(cfg);
}

imu_status_t icm53611_verify_wom_retained(const icm53611_wom_dlps_wake_cfg_t *cfg)
{
    uint32_t token = pwrd_imu_begin(2U);
    imu_status_t status = IMU_STATUS_INVALID_PARAM;
    if (cfg != NULL)
    {
        /* A weak-rail restore can lose device state. Check identity and basic
         * operating mode only; never reconfigure from the nested callback. */
        status = icm53611_wom_check_ui(token, ICM53611_REG_WHO_AM_I,
                                      0xffU, ICM53611_WHO_AM_I_VALUE);
        if (status == IMU_STATUS_OK)
            status = icm53611_wom_check_ui(token, ICM53611_REG_PWR_MGMT0,
                ICM53611_PWR_MGMT0_WOM_SLEEP_VERIFY_MASK, ICM53611_PWR_MGMT0_ACCEL_MODE_LP);
        if (status == IMU_STATUS_OK)
            status = icm53611_wom_check_ui(token, ICM53611_REG_WOM_CONFIG,
                ICM53611_WOM_CONFIG_VERIFY_MASK, icm53611_wom_dlps_config_value(cfg));
    }
    pwrd_imu_finish(token, status);
    return status;
}

imu_status_t icm53611_disable_wom_wake_for_dlps(void)
{
    uint32_t token = pwrd_imu_begin(3U);
    IMU_UNUSED(token);
    imu_status_t status = icm53611_init_bus();
    pwrd_stage(5U);
    if (status == IMU_STATUS_OK) status = icm53611_wom_stop_clocked_minimal();
    if (status == IMU_STATUS_OK) status = icm53611_wom_ui_bits(ICM53611_REG_WOM_CONFIG, ICM53611_WOM_CONFIG_WOM_EN, 0U);
    /* OFF is not a FIFO flush: tail retention remains the caller's responsibility. */
    if (status == IMU_STATUS_OK) status = icm53611_wom_ui_write(ICM53611_REG_PWR_MGMT0, 0U);
    pwrd_imu_finish(token, status);
    return status;
}

bool icm53611_get_last_mreg_verify_error(icm53611_mreg_verify_error_t *out)
{
    if ((out == NULL) || !s_icm53611_last_mreg_verify_error.valid)
    {
        return false;
    }

    *out = s_icm53611_last_mreg_verify_error;
    return true;
}

imu_status_t icm53611_wom_read_threshold_codes(uint8_t *x_code,
                                               uint8_t *y_code,
                                               uint8_t *z_code)
{
    imu_status_t status;

    if ((x_code == NULL) || (y_code == NULL) || (z_code == NULL))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = imu_spi_mreg_read(ICM53611_MREG_BANK_WOM_THR,
                               ICM53611_REG_ACCEL_WOM_X_THR,
                               x_code);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = imu_spi_mreg_read(ICM53611_MREG_BANK_WOM_THR,
                               ICM53611_REG_ACCEL_WOM_Y_THR,
                               y_code);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    return imu_spi_mreg_read(ICM53611_MREG_BANK_WOM_THR,
                             ICM53611_REG_ACCEL_WOM_Z_THR,
                             z_code);
}

void icm53611_wom_log_apex_disable_status(void)
{
    imu_status_t status;
    uint8_t sensor_config3 = 0U;

    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1,
                               ICM53611_MREG1_REG_SENSOR_CONFIG3,
                               &sensor_config3);
#if IMU_WOM_RUNTIME_LOG_ENABLE || IMU_MREG_DIAG_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE
    DBG_DIRECT("[IMU_WOM_IRQ_CFG] bank=MREG1 blk_sel=0x%02x reg=SENSOR_CONFIG3 addr=0x%02x "
               "sensor_config3=0x%02x apex_disable=%u status=%d",
               ICM53611_MREG_BLOCK_1,
               ICM53611_MREG1_REG_SENSOR_CONFIG3,
               sensor_config3,
               ((sensor_config3 & ICM53611_MREG1_SENSOR_CONFIG3_APEX_DISABLE) != 0U) ? 1U : 0U,
               status);
#else
    IMU_UNUSED(status);
    IMU_UNUSED(sensor_config3);
#endif
}

#if IMU_MREG_DIAG_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE
static void icm53611_wom_mreg_diag_axis(const char *axis,
                                        uint8_t addr,
                                        uint8_t threshold_code)
{
    imu_status_t status;
    uint8_t readback = 0U;
    const char *reg_name = "ACCEL_WOM_?_THR";

    if (addr == ICM53611_REG_ACCEL_WOM_X_THR)
    {
        reg_name = "ACCEL_WOM_X_THR";
    }
    else if (addr == ICM53611_REG_ACCEL_WOM_Y_THR)
    {
        reg_name = "ACCEL_WOM_Y_THR";
    }
    else if (addr == ICM53611_REG_ACCEL_WOM_Z_THR)
    {
        reg_name = "ACCEL_WOM_Z_THR";
    }

    status = imu_spi_mreg_read(ICM53611_MREG_BANK_WOM_THR, addr, &readback);
    DBG_DIRECT("[IMU_WOM_MREG_DIAG] step=read_initial axis=%s bank=MREG1 addr=0x%02x "
               "read=0x%02x status=%d",
               axis,
               addr,
               readback,
               status);

    status = icm53611_wom_thr_write_verify(axis, reg_name, addr, threshold_code, NULL);
    DBG_DIRECT("[IMU_WOM_MREG_DIAG] step=threshold_verify axis=%s status=%d result=%s",
               axis,
               status,
               icm53611_pass_fail(status == IMU_STATUS_OK));
}
#endif

void icm53611_wom_mreg_diag(uint8_t threshold_code)
{
#if IMU_MREG_DIAG_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE
    ZY100_DIAG_LOG("[IMU_WOM_MREG_DIAG] begin threshold_code=0x%02x", threshold_code);
    icm53611_mreg2_read_only_diag();
    icm53611_wom_mreg_diag_axis("X", ICM53611_REG_ACCEL_WOM_X_THR, threshold_code);
    icm53611_wom_mreg_diag_axis("Y", ICM53611_REG_ACCEL_WOM_Y_THR, threshold_code);
    icm53611_wom_mreg_diag_axis("Z", ICM53611_REG_ACCEL_WOM_Z_THR, threshold_code);
    ZY100_DIAG_LOG("[IMU_WOM_MREG_DIAG] end");
#else
    IMU_UNUSED(threshold_code);
#endif
}

void icm53611_wom_debug_try_apex_enable(uint8_t threshold_code)
{
    imu_status_t status;
    imu_status_t apex_disable_1_status = IMU_STATUS_OK;
    imu_status_t apex_disable_0_status = IMU_STATUS_OK;
    imu_status_t restore_status = IMU_STATUS_OK;
    uint8_t sensor_config3 = 0U;
    uint8_t apex_disable_config3;
    uint8_t apex_enable_config3;
    uint8_t restore_readback = 0U;

    status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1,
                               ICM53611_MREG1_REG_SENSOR_CONFIG3,
                               &sensor_config3);
    ZY100_DIAG_LOG("[IMU_WOM_APEX_DIAG] sensor_config3_before=0x%02x apex_disable=%u status=%d",
               sensor_config3,
               ((sensor_config3 & ICM53611_MREG1_SENSOR_CONFIG3_APEX_DISABLE) != 0U) ? 1U : 0U,
               status);
    if (status != IMU_STATUS_OK)
    {
        return;
    }

    apex_disable_config3 = (uint8_t)(sensor_config3 | ICM53611_MREG1_SENSOR_CONFIG3_APEX_DISABLE);
    apex_enable_config3 = (uint8_t)(sensor_config3 & (uint8_t)(~ICM53611_MREG1_SENSOR_CONFIG3_APEX_DISABLE));

    status = imu_spi_mreg_write(ICM53611_MREG_BLOCK_1,
                                ICM53611_MREG1_REG_SENSOR_CONFIG3,
                                apex_disable_config3);
    if (status == IMU_STATUS_OK)
    {
        apex_disable_1_status = icm53611_wom_verify_threshold_triplet(threshold_code);
    }
    else
    {
        apex_disable_1_status = status;
    }

    ZY100_DIAG_LOG("[IMU_WOM_APEX_DIAG] try_enable_apex=1");
    status = imu_spi_mreg_write(ICM53611_MREG_BLOCK_1,
                                ICM53611_MREG1_REG_SENSOR_CONFIG3,
                                apex_enable_config3);
    if (status == IMU_STATUS_OK)
    {
        apex_disable_0_status = icm53611_wom_verify_threshold_triplet(threshold_code);
    }
    else
    {
        apex_disable_0_status = status;
    }

    restore_status = imu_spi_mreg_write(ICM53611_MREG_BLOCK_1,
                                        ICM53611_MREG1_REG_SENSOR_CONFIG3,
                                        sensor_config3);
    if (restore_status == IMU_STATUS_OK)
    {
        restore_status = imu_spi_mreg_read(ICM53611_MREG_BLOCK_1,
                                           ICM53611_MREG1_REG_SENSOR_CONFIG3,
                                           &restore_readback);
        if ((restore_status == IMU_STATUS_OK) && (restore_readback != sensor_config3))
        {
            restore_status = IMU_STATUS_VERIFY_FAILED;
        }
    }

    DBG_DIRECT("[IMU_WOM_APEX_DIAG] apex_disable_1_verify=%s apex_disable_0_verify=%s restore_status=%d",
               icm53611_pass_fail(apex_disable_1_status == IMU_STATUS_OK),
               icm53611_pass_fail(apex_disable_0_status == IMU_STATUS_OK),
               restore_status);
    ZY100_DIAG_LOG("[IMU_WOM_APEX_DIAG] sensor_config3_restore=0x%02x status=%d",
               restore_readback,
               restore_status);
}
