#include "mmc5603.h"
#include "../app_flags.h"

#include <stddef.h>
#include <string.h>

#include "../bsp/mag_bsp.h"
#include "mmc5603_i2c.h"
#include "mmc5603_reg.h"

#define MMC5603_T_OP_US                5000U
#define MMC5603_T_RESET_US             20000U
#define MMC5603_T_SR_GUARD_US          1000U
#define MMC5603_STATUS_POLL_INTERVAL_US 100U
#define MMC5603_MEAS_TIMEOUT_GUARD_US  20000U

typedef struct
{
    bool inited;
    bool auto_sr_enabled;
    bool continuous_enabled;
    mmc5603_bw_t bw;
    uint8_t odr;
    bool hpower;
    uint8_t ctrl0_shadow;
    uint8_t ctrl1_shadow;
    uint8_t ctrl2_shadow;
    uint32_t sample_seq;
} mmc5603_ctx_t;

static mmc5603_ctx_t s_mmc5603_ctx = {0};

static const mmc5603_cfg_t s_mmc5603_default_cfg =
{
    .auto_sr_enable = true,
    .bw = MMC5603_BW_LEVEL_00,
    .continuous_odr = 0U,
    .continuous_hpower = false,
};

static uint32_t mmc5603_bw_to_ttm_us(mmc5603_bw_t bw)
{
    switch (bw)
    {
    case MMC5603_BW_LEVEL_00:
        return 6600U;
    case MMC5603_BW_LEVEL_01:
        return 3500U;
    case MMC5603_BW_LEVEL_10:
        return 2000U;
    case MMC5603_BW_LEVEL_11:
        return 1200U;
    default:
        return 6600U;
    }
}

static mag_status_t mmc5603_wait_status_mask(uint8_t mask, uint32_t timeout_us, uint8_t *status1_out)
{
    const uint64_t start = mag_bsp_local_timestamp_us();
    uint8_t status1 = 0U;
    mag_status_t status;

    do
    {
        status = mag_i2c_read_reg(MMC5603_REG_STATUS1, &status1);
        if (status != MAG_STATUS_OK)
        {
            return status;
        }

        if ((status1 & mask) != 0U)
        {
            if (status1_out != NULL)
            {
                *status1_out = status1;
            }
            return MAG_STATUS_OK;
        }

        mag_bsp_delay_us(MMC5603_STATUS_POLL_INTERVAL_US);
    } while ((mag_bsp_local_timestamp_us() - start) < timeout_us);

    if (status1_out != NULL)
    {
        *status1_out = status1;
    }
    MAG_LOG_ERROR("wait status timeout: mask=0x%02x status1=0x%02x timeout_us=%u",
                  mask, status1, timeout_us);
    return MAG_STATUS_TIMEOUT;
}

static uint16_t mmc5603_status1_to_flags(uint8_t status1)
{
    uint16_t flags = 0U;

    if ((status1 & MMC5603_STATUS1_MEAS_M_DONE) != 0U)
    {
        flags |= MMC5603_SAMPLE_FLAG_MEAS_M_DONE;
    }
    if ((status1 & MMC5603_STATUS1_MEAS_T_DONE) != 0U)
    {
        flags |= MMC5603_SAMPLE_FLAG_MEAS_T_DONE;
    }
    if ((status1 & MMC5603_STATUS1_OTP_READ_DONE) != 0U)
    {
        flags |= MMC5603_SAMPLE_FLAG_OTP_READ_OK;
    }

    return flags;
}

static mag_status_t mmc5603_write_ctrl0(uint8_t value)
{
    return mag_i2c_write_reg(MMC5603_REG_INTERNAL_CTRL0, value);
}

static mag_status_t mmc5603_write_ctrl1(uint8_t value)
{
    s_mmc5603_ctx.ctrl1_shadow = value;
    return mag_i2c_write_reg(MMC5603_REG_INTERNAL_CTRL1, value);
}

static mag_status_t mmc5603_write_ctrl2(uint8_t value)
{
    s_mmc5603_ctx.ctrl2_shadow = value;
    return mag_i2c_write_reg(MMC5603_REG_INTERNAL_CTRL2, value);
}

static mag_status_t mmc5603_validate_continuous_odr(uint8_t odr, mmc5603_bw_t bw, bool hpower)
{
    if (odr == 0U)
    {
        return MAG_STATUS_INVALID_PARAM;
    }

    switch (bw)
    {
    case MMC5603_BW_LEVEL_00:
        if (odr > 75U)
        {
            return MAG_STATUS_INVALID_PARAM;
        }
        break;

    case MMC5603_BW_LEVEL_01:
        if (odr > 150U)
        {
            return MAG_STATUS_INVALID_PARAM;
        }
        break;

    case MMC5603_BW_LEVEL_10:
        if (odr > 255U)
        {
            return MAG_STATUS_INVALID_PARAM;
        }
        break;

    case MMC5603_BW_LEVEL_11:
        if (!hpower && (odr > 255U))
        {
            return MAG_STATUS_INVALID_PARAM;
        }
        break;

    default:
        return MAG_STATUS_INVALID_PARAM;
    }

    return MAG_STATUS_OK;
}

static void mmc5603_update_continuous_flag(void)
{
    s_mmc5603_ctx.continuous_enabled =
        (((s_mmc5603_ctx.ctrl2_shadow & MMC5603_CTRL2_CMM_EN) != 0U) &&
         (s_mmc5603_ctx.odr != 0U));
}

static void mmc5603_fill_sample_common(mmc5603_sample_t *sample, uint8_t status1, mmc5603_source_mode_t mode)
{
    sample->sample_seq = ++s_mmc5603_ctx.sample_seq;
    sample->status1_snapshot = status1;
    sample->status_flags = mmc5603_status1_to_flags(status1);
    sample->source_mode = mode;
    if (mode == MMC5603_SOURCE_MODE_CONTINUOUS)
    {
        sample->status_flags |= MMC5603_SAMPLE_FLAG_CONTINUOUS;
    }
}

const mmc5603_cfg_t *mmc5603_default_config(void)
{
    return &s_mmc5603_default_cfg;
}

mag_status_t mmc5603_read_product_id(uint8_t *product_id)
{
    if (product_id == NULL)
    {
        return MAG_STATUS_INVALID_PARAM;
    }

    return mag_i2c_read_reg(MMC5603_REG_PRODUCT_ID, product_id);
}

mag_status_t mmc5603_force_power_down(void)
{
    mag_status_t status;
    uint8_t product_id = 0U;

    status = mag_bsp_i2c_force_reinit(MMC5603_I2C_ADDR_7BIT);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    status = mmc5603_read_product_id(&product_id);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }
    if (product_id != MMC5603_PRODUCT_ID_VALUE)
    {
        return MAG_STATUS_NOT_FOUND;
    }

    status = mag_i2c_write_reg(MMC5603_REG_INTERNAL_CTRL2, 0x00U);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }
    s_mmc5603_ctx.ctrl2_shadow = 0x00U;

    status = mag_i2c_write_reg(MMC5603_REG_ODR, 0x00U);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }
    s_mmc5603_ctx.odr = 0x00U;

    status = mag_i2c_write_reg(MMC5603_REG_INTERNAL_CTRL0, 0x00U);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }
    s_mmc5603_ctx.ctrl0_shadow = 0x00U;

    status = mag_i2c_write_reg(MMC5603_REG_INTERNAL_CTRL1, 0x00U);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }
    s_mmc5603_ctx.ctrl1_shadow = 0x00U;

    s_mmc5603_ctx.auto_sr_enabled = false;
    s_mmc5603_ctx.continuous_enabled = false;
    s_mmc5603_ctx.hpower = false;
#if ZY100_OFFLINE_V2_WOM_START_ENABLE
    /* MMC5603NJ Rev. B pp. 8-10: ODR and CTRL0/1/2 are write-only.
     * Successful writes establish the requested mode; PRODUCT_ID only
     * checks communication afterwards, not physical low-power residency. */
    status = mmc5603_read_product_id(&product_id);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }
    return (product_id == MMC5603_PRODUCT_ID_VALUE) ?
           MAG_STATUS_OK : MAG_STATUS_NOT_FOUND;
#else
    return MAG_STATUS_OK;
#endif
}

mag_status_t mmc5603_soft_reset(void)
{
    mag_status_t status;

    status = mag_i2c_write_reg(MMC5603_REG_INTERNAL_CTRL1, MMC5603_CTRL1_SW_RESET);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    mag_bsp_delay_us(MMC5603_T_RESET_US);

    s_mmc5603_ctx.ctrl0_shadow = 0U;
    s_mmc5603_ctx.ctrl1_shadow = 0U;
    s_mmc5603_ctx.ctrl2_shadow = 0U;
    s_mmc5603_ctx.odr = 0U;
    s_mmc5603_ctx.hpower = false;
    s_mmc5603_ctx.continuous_enabled = false;
    return MAG_STATUS_OK;
}

mag_status_t mmc5603_set_bw(mmc5603_bw_t bw)
{
    mag_status_t status;
    uint8_t ctrl1 = s_mmc5603_ctx.ctrl1_shadow;

    if (bw > MMC5603_BW_LEVEL_11)
    {
        return MAG_STATUS_INVALID_PARAM;
    }

    ctrl1 &= (uint8_t)(~MMC5603_CTRL1_BW_MASK);
    ctrl1 |= (uint8_t)(bw & 0x03U);

    status = mmc5603_write_ctrl1(ctrl1);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    s_mmc5603_ctx.bw = bw;
    return MAG_STATUS_OK;
}

mag_status_t mmc5603_enable_auto_sr(bool enable)
{
    if (!s_mmc5603_ctx.inited)
    {
        return MAG_STATUS_NOT_READY;
    }

    s_mmc5603_ctx.auto_sr_enabled = enable;
    if (enable)
    {
        s_mmc5603_ctx.ctrl0_shadow |= MMC5603_CTRL0_AUTO_SR_EN;
    }
    else
    {
        s_mmc5603_ctx.ctrl0_shadow &= (uint8_t)(~MMC5603_CTRL0_AUTO_SR_EN);
    }

    return mmc5603_write_ctrl0(s_mmc5603_ctx.ctrl0_shadow);
}

mag_status_t mmc5603_set_odr(uint8_t odr)
{
    mag_status_t status;

    if (!s_mmc5603_ctx.inited)
    {
        return MAG_STATUS_NOT_READY;
    }

    if (odr != 0U)
    {
        status = mmc5603_validate_continuous_odr(odr, s_mmc5603_ctx.bw, s_mmc5603_ctx.hpower);
        if (status != MAG_STATUS_OK)
        {
            return status;
        }
    }

    status = mag_i2c_write_reg(MMC5603_REG_ODR, odr);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    s_mmc5603_ctx.odr = odr;
    mmc5603_update_continuous_flag();
    return MAG_STATUS_OK;
}

mag_status_t mmc5603_set_cmm_en(bool enable)
{
    mag_status_t status;
    uint8_t ctrl2;

    if (!s_mmc5603_ctx.inited)
    {
        return MAG_STATUS_NOT_READY;
    }

    if (enable && (s_mmc5603_ctx.odr == 0U))
    {
        return MAG_STATUS_INVALID_PARAM;
    }

    ctrl2 = s_mmc5603_ctx.ctrl2_shadow;
    if (enable)
    {
        ctrl2 |= MMC5603_CTRL2_CMM_EN;
    }
    else
    {
        ctrl2 &= (uint8_t)(~MMC5603_CTRL2_CMM_EN);
    }

    status = mmc5603_write_ctrl2(ctrl2);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    mmc5603_update_continuous_flag();
    return MAG_STATUS_OK;
}

mag_status_t mmc5603_set_hpower(bool enable)
{
    mag_status_t status;
    uint8_t ctrl2;

    if (!s_mmc5603_ctx.inited)
    {
        return MAG_STATUS_NOT_READY;
    }

    if (s_mmc5603_ctx.odr != 0U)
    {
        status = mmc5603_validate_continuous_odr(s_mmc5603_ctx.odr, s_mmc5603_ctx.bw, enable);
        if (status != MAG_STATUS_OK)
        {
            return status;
        }
    }

    ctrl2 = s_mmc5603_ctx.ctrl2_shadow;
    if (enable)
    {
        ctrl2 |= MMC5603_CTRL2_HPOWER;
    }
    else
    {
        ctrl2 &= (uint8_t)(~MMC5603_CTRL2_HPOWER);
    }

    status = mmc5603_write_ctrl2(ctrl2);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    s_mmc5603_ctx.hpower = enable;
    mmc5603_update_continuous_flag();
    return MAG_STATUS_OK;
}

mag_status_t mmc5603_set_cmm_freq_en(bool enable)
{
    if (!s_mmc5603_ctx.inited)
    {
        return MAG_STATUS_NOT_READY;
    }

    if (enable)
    {
        s_mmc5603_ctx.ctrl0_shadow |= MMC5603_CTRL0_CMM_FREQ_EN;
    }
    else
    {
        s_mmc5603_ctx.ctrl0_shadow &= (uint8_t)(~MMC5603_CTRL0_CMM_FREQ_EN);
    }

    return mmc5603_write_ctrl0(s_mmc5603_ctx.ctrl0_shadow);
}

mag_status_t mmc5603_get_state_snapshot(mmc5603_state_snapshot_t *state_out)
{
    if (state_out == NULL)
    {
        return MAG_STATUS_INVALID_PARAM;
    }
    if (!s_mmc5603_ctx.inited)
    {
        return MAG_STATUS_NOT_READY;
    }

    state_out->odr = s_mmc5603_ctx.odr;
    state_out->bw = (uint8_t)s_mmc5603_ctx.bw;
    state_out->auto_sr_en = s_mmc5603_ctx.auto_sr_enabled ? 1U : 0U;
    state_out->cmm_en = ((s_mmc5603_ctx.ctrl2_shadow & MMC5603_CTRL2_CMM_EN) != 0U) ? 1U : 0U;
    state_out->cmm_freq_en = ((s_mmc5603_ctx.ctrl0_shadow & MMC5603_CTRL0_CMM_FREQ_EN) != 0U) ? 1U : 0U;
    state_out->hpower = s_mmc5603_ctx.hpower ? 1U : 0U;
    state_out->ctrl0_shadow = s_mmc5603_ctx.ctrl0_shadow;
    state_out->ctrl1_shadow = s_mmc5603_ctx.ctrl1_shadow;
    state_out->ctrl2_shadow = s_mmc5603_ctx.ctrl2_shadow;
    return MAG_STATUS_OK;
}

mag_status_t mmc5603_manual_set(void)
{
    mag_status_t status;

    status = mmc5603_write_ctrl0((uint8_t)(s_mmc5603_ctx.ctrl0_shadow | MMC5603_CTRL0_DO_SET));
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    mag_bsp_delay_us(MMC5603_T_SR_GUARD_US);
    return MAG_STATUS_OK;
}

mag_status_t mmc5603_manual_reset(void)
{
    mag_status_t status;

    status = mmc5603_write_ctrl0((uint8_t)(s_mmc5603_ctx.ctrl0_shadow | MMC5603_CTRL0_DO_RESET));
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    mag_bsp_delay_us(MMC5603_T_SR_GUARD_US);
    return MAG_STATUS_OK;
}

mag_status_t mmc5603_take_measurement(bool read_temp, mmc5603_sample_t *sample)
{
    mag_status_t status;
    uint8_t status1 = 0U;
    uint8_t raw[MMC5603_MEAS_RAW_LEN];
    uint8_t temp = 0U;
    const uint32_t timeout_us = mmc5603_bw_to_ttm_us(s_mmc5603_ctx.bw) + MMC5603_MEAS_TIMEOUT_GUARD_US;

    if (sample == NULL)
    {
        return MAG_STATUS_INVALID_PARAM;
    }
    if (!s_mmc5603_ctx.inited)
    {
        return MAG_STATUS_NOT_READY;
    }

    sample->trig_local_ts_us = mag_bsp_local_timestamp_us();
    status = mmc5603_write_ctrl0((uint8_t)(s_mmc5603_ctx.ctrl0_shadow | MMC5603_CTRL0_TAKE_MEAS_M));
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    status = mmc5603_wait_status_mask(MMC5603_STATUS1_MEAS_M_DONE, timeout_us, &status1);
    if (status != MAG_STATUS_OK)
    {
        MAG_LOG_ERROR("single measure wait M_DONE failed, status=%d status1=0x%02x", status, status1);
        return status;
    }

    sample->meas_done_local_ts_us = mag_bsp_local_timestamp_us();

    status = mag_i2c_read_regs(MMC5603_REG_XOUT0, raw, MMC5603_MEAS_RAW_LEN);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    memcpy(sample->raw9, raw, sizeof(sample->raw9));
    mag_parse_raw20(raw, &sample->raw_x, &sample->raw_y, &sample->raw_z);
    sample->temp_valid = false;
    sample->raw_temp = 0U;

    if (read_temp)
    {
        status = mmc5603_write_ctrl0((uint8_t)(s_mmc5603_ctx.ctrl0_shadow | MMC5603_CTRL0_TAKE_MEAS_T));
        if (status != MAG_STATUS_OK)
        {
            return status;
        }

        status = mmc5603_wait_status_mask(MMC5603_STATUS1_MEAS_T_DONE, timeout_us, NULL);
        if (status != MAG_STATUS_OK)
        {
            MAG_LOG_ERROR("single measure wait T_DONE failed, status=%d", status);
            return status;
        }

        status = mag_i2c_read_reg(MMC5603_REG_TOUT, &temp);
        if (status != MAG_STATUS_OK)
        {
            return status;
        }

        sample->raw_temp = temp;
        sample->temp_valid = true;
    }

    sample->readout_local_ts_us = mag_bsp_local_timestamp_us();
    mmc5603_fill_sample_common(sample, status1, MMC5603_SOURCE_MODE_SINGLE);
    return MAG_STATUS_OK;
}

mag_status_t mmc5603_set_continuous_mode(uint8_t odr, bool hpower, bool enable)
{
    mag_status_t status;

    if (!s_mmc5603_ctx.inited)
    {
        return MAG_STATUS_NOT_READY;
    }

    if (!enable)
    {
        status = mmc5603_set_cmm_en(false);
        if (status != MAG_STATUS_OK)
        {
            return status;
        }

        status = mmc5603_set_odr(0U);
        if (status != MAG_STATUS_OK)
        {
            return status;
        }

        status = mmc5603_set_hpower(false);
        return status;
    }

    status = mmc5603_set_odr(odr);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    status = mmc5603_set_cmm_freq_en(true);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    status = mmc5603_set_hpower(hpower);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    status = mmc5603_set_cmm_en(true);
    return status;
}

static mag_status_t mmc5603_read_continuous_payload(
    bool read_temp,
    uint8_t status1,
    uint64_t trigger_ts_us,
    mmc5603_sample_t *sample)
{
    mag_status_t status;
    uint8_t raw[MMC5603_MEAS_RAW_LEN];
    uint8_t temp = 0U;

    sample->trig_local_ts_us = trigger_ts_us;
    sample->meas_done_local_ts_us = trigger_ts_us;
    status = mag_i2c_read_regs(MMC5603_REG_XOUT0, raw, MMC5603_MEAS_RAW_LEN);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    memcpy(sample->raw9, raw, sizeof(sample->raw9));
    mag_parse_raw20(raw, &sample->raw_x, &sample->raw_y, &sample->raw_z);
    sample->temp_valid = false;
    sample->raw_temp = 0U;

    if (read_temp)
    {
        status = mag_i2c_read_reg(MMC5603_REG_TOUT, &temp);
        if (status != MAG_STATUS_OK)
        {
            return status;
        }
        sample->raw_temp = temp;
        sample->temp_valid = true;
    }

    sample->readout_local_ts_us = mag_bsp_local_timestamp_us();
    mmc5603_fill_sample_common(sample, status1, MMC5603_SOURCE_MODE_CONTINUOUS);
    return MAG_STATUS_OK;
}

mag_status_t mmc5603_read_continuous_latest(bool read_temp, mmc5603_sample_t *sample)
{
    mag_status_t status;
    uint8_t status1 = 0U;
    uint64_t trigger_ts_us;

    if (sample == NULL)
    {
        return MAG_STATUS_INVALID_PARAM;
    }
    if (!s_mmc5603_ctx.inited || !s_mmc5603_ctx.continuous_enabled)
    {
        return MAG_STATUS_NOT_READY;
    }

    trigger_ts_us = mag_bsp_local_timestamp_us();
    status = mag_i2c_read_reg(MMC5603_REG_STATUS1, &status1);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    return mmc5603_read_continuous_payload(read_temp,
                                           status1,
                                           trigger_ts_us,
                                           sample);
}

mag_status_t mmc5603_read_continuous_fresh(bool read_temp,
                                           mmc5603_sample_t *sample,
                                           bool *fresh_out)
{
    mag_status_t status;
    uint8_t status1 = 0U;
    uint64_t trigger_ts_us;

    if ((sample == NULL) || (fresh_out == NULL))
    {
        return MAG_STATUS_INVALID_PARAM;
    }
    *fresh_out = false;
    if (!s_mmc5603_ctx.inited || !s_mmc5603_ctx.continuous_enabled)
    {
        return MAG_STATUS_NOT_READY;
    }

    trigger_ts_us = mag_bsp_local_timestamp_us();
    status = mag_i2c_read_reg(MMC5603_REG_STATUS1, &status1);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }
    if ((status1 & MMC5603_STATUS1_MEAS_M_DONE) == 0U)
    {
        return MAG_STATUS_OK;
    }

    status = mmc5603_read_continuous_payload(read_temp,
                                             status1,
                                             trigger_ts_us,
                                             sample);
    if (status == MAG_STATUS_OK)
    {
        *fresh_out = true;
    }
    return status;
}

mag_status_t mmc5603_alive_check(void)
{
    mag_status_t status;
    uint8_t product_id = 0U;
    mmc5603_sample_t sample;

    status = mmc5603_read_product_id(&product_id);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }
    if (product_id != MMC5603_PRODUCT_ID_VALUE)
    {
        MAG_LOG_ERROR("Product ID mismatch, expect=0x%02x got=0x%02x",
                      MMC5603_PRODUCT_ID_VALUE, product_id);
        return MAG_STATUS_NOT_FOUND;
    }

    status = mmc5603_take_measurement(false, &sample);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    return MAG_STATUS_OK;
}

mag_status_t mmc5603_init(const mmc5603_cfg_t *cfg)
{
    mag_status_t status;
    uint8_t product_id = 0U;
    const mmc5603_cfg_t *use_cfg = (cfg != NULL) ? cfg : mmc5603_default_config();

    status = mag_i2c_bus_init();
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    mag_bsp_delay_us(MMC5603_T_OP_US);

    status = mmc5603_read_product_id(&product_id);
    if (status != MAG_STATUS_OK)
    {
        MAG_LOG_ERROR("read product id(before reset) failed, status=%d", status);
        return status;
    }
    if (product_id != MMC5603_PRODUCT_ID_VALUE)
    {
        MAG_LOG_ERROR("Product ID mismatch before reset, expect=0x%02x got=0x%02x",
                      MMC5603_PRODUCT_ID_VALUE, product_id);
        return MAG_STATUS_NOT_FOUND;
    }
    ZY100_LOG_DETAIL("[MAG] product id(before reset)=0x%02x", product_id);

    status = mmc5603_soft_reset();
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    status = mmc5603_read_product_id(&product_id);
    if (status != MAG_STATUS_OK)
    {
        MAG_LOG_ERROR("read product id(after reset) failed, status=%d", status);
        return status;
    }
    if (product_id != MMC5603_PRODUCT_ID_VALUE)
    {
        MAG_LOG_ERROR("Product ID mismatch after reset, expect=0x%02x got=0x%02x",
                      MMC5603_PRODUCT_ID_VALUE, product_id);
        return MAG_STATUS_NOT_FOUND;
    }
    ZY100_LOG_DETAIL("[MAG] product id(after reset)=0x%02x", product_id);

    s_mmc5603_ctx.inited = true;
    s_mmc5603_ctx.sample_seq = 0U;

    status = mmc5603_set_bw(use_cfg->bw);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    status = mmc5603_enable_auto_sr(use_cfg->auto_sr_enable);
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    status = mmc5603_set_continuous_mode(use_cfg->continuous_odr, use_cfg->continuous_hpower,
                                         (use_cfg->continuous_odr != 0U));
    if (status != MAG_STATUS_OK)
    {
        return status;
    }

    return mmc5603_alive_check();
}

void mag_parse_raw20(const uint8_t raw9[9], uint32_t *raw_x, uint32_t *raw_y, uint32_t *raw_z)
{
    if ((raw9 == NULL) || (raw_x == NULL) || (raw_y == NULL) || (raw_z == NULL))
    {
        return;
    }

    *raw_x = ((uint32_t)raw9[0] << 12) | ((uint32_t)raw9[1] << 4) | ((uint32_t)(raw9[6] >> 4) & 0x0FU);
    *raw_y = ((uint32_t)raw9[2] << 12) | ((uint32_t)raw9[3] << 4) | ((uint32_t)(raw9[7] >> 4) & 0x0FU);
    *raw_z = ((uint32_t)raw9[4] << 12) | ((uint32_t)raw9[5] << 4) | ((uint32_t)(raw9[8] >> 4) & 0x0FU);
}
