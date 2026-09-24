#include "icm53611_spi.h"

#include <stddef.h>

#include "../bsp/imu_bsp.h"
#include "icm53611_reg.h"
#include "spi_bus_owner.h"
#include "../service/power_diag.h"

#ifndef IMU_SPI_TRACE_ENABLE
#define IMU_SPI_TRACE_ENABLE 0
#endif

#ifndef IMU_SPI_WHOAMI_TRACE_ENABLE
#define IMU_SPI_WHOAMI_TRACE_ENABLE 1
#endif

#ifndef IMU_SPI_WHOAMI_TRACE_MAX_LOG
#define IMU_SPI_WHOAMI_TRACE_MAX_LOG 24U
#endif

#ifndef ZY100_LOCKED_SPI_DEBUG_CHECK
#define ZY100_LOCKED_SPI_DEBUG_CHECK 0U
#endif

#define ICM53611_SPI_DUMMY_TX 0xFFU
#define ICM53611_OIS_WINDOW_BYTES 19U
#define ICM53611_MREG_NO_ACCESS_DELAY_US 10U
#define ICM53611_MCLK_READY_TIMEOUT_MS 20U
#define ICM53611_MCLK_READY_POLL_US 100U
#define ICM53611_PWR_MGMT0_RESTORE_DELAY_US 200U
#define ICM53611_MCLK_DEBUG_BOOT_LOG_LIMIT 4U

typedef imu_spi_clock_t imu_spi_mreg_context_t;

static uint32_t s_imu_spi_whoami_trace_cnt = 0U;
#if IMU_MREG_DIAG_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE
static bool s_imu_spi_mclk_log_seen = false;
static uint32_t s_imu_spi_mclk_debug_log_cnt = 0U;
#endif
static uint32_t s_imu_spi_idle_enable_log_cnt = 0U;
static bool s_imu_error_active;
static imu_spi_error_t s_imu_error;

void imu_spi_error_begin(void)
{
    const imu_spi_error_t empty = {0U, 0U, 0U, 0U, 0U, 0U, 0xffU, 0xffU};
    s_imu_error = empty;
    s_imu_error_active = true;
}

void imu_spi_error_checkpoint(void)
{
    if (s_imu_error_active)
    {
        s_imu_error.operation = 0U;
        s_imu_error.space = s_imu_error.reg = s_imu_error.value = 0U;
        s_imu_error.read_valid = s_imu_error.status = 0U;
        s_imu_error.acquire_status = s_imu_error.release_status = 0xffU;
    }
}

void imu_spi_error_end(imu_spi_error_t *error)
{
    s_imu_error_active = false;
    if (error != NULL) *error = s_imu_error;
}

void imu_spi_error_record(uint8_t op, uint8_t space, uint8_t reg,
                          uint8_t value, bool valid, imu_status_t status)
{
    if (!s_imu_error_active || status == IMU_STATUS_OK || s_imu_error.operation) return;
    s_imu_error.operation = op;
    s_imu_error.space = space;
    s_imu_error.reg = reg;
    s_imu_error.value = value;
    s_imu_error.read_valid = valid ? 1U : 0U;
    s_imu_error.status = (uint8_t)status;
}

static bool imu_spi_should_trace_whoami(uint8_t addr)
{
#if IMU_SPI_WHOAMI_TRACE_ENABLE
    return (addr == ICM53611_REG_WHO_AM_I) && (s_imu_spi_whoami_trace_cnt < IMU_SPI_WHOAMI_TRACE_MAX_LOG);
#else
    IMU_UNUSED(addr);
    return false;
#endif
}

static void imu_spi_log_read_raw_bytes(const char *tag,
                                       uint8_t reg,
                                       uint8_t cmd,
                                       uint8_t cmd_rx,
                                       const uint8_t *data_rx,
                                       uint16_t data_len)
{
    uint16_t i;
    const char *use_tag = (tag != NULL) ? tag : "SPI raw";

    IMU_LOG_INFO("%s reg=0x%02x cmd=0x%02x (addr=0x%02x mask=0x%02x)",
                 use_tag,
                 reg,
                 cmd,
                 reg & 0x7FU,
                 ICM53611_SPI_READ_MASK);
    IMU_LOG_INFO("%s byte[0] tx=0x%02x rx=0x%02x", use_tag, cmd, cmd_rx);

    if ((data_rx == NULL) || (data_len == 0U))
    {
        return;
    }

    for (i = 0U; i < data_len; i++)
    {
        IMU_LOG_INFO("%s byte[%u] tx=0x%02x rx=0x%02x",
                     use_tag,
                     (unsigned int)(i + 1U),
                     ICM53611_SPI_DUMMY_TX,
                     data_rx[i]);
    }
}

static imu_status_t imu_spi_acquire_task(bool report_busy)
{
    spi_bus_owner_t blocking_owner = SPI_OWNER_NONE;
    bsp_shared_spi_acquire_result_t result;

    if (!report_busy)
    {
        imu_status_t status = imu_bsp_init();
        if (status != IMU_STATUS_OK) return status;
        return spi_bus_acquire(SPI_OWNER_IMU) ? IMU_STATUS_OK : IMU_STATUS_BUS_ERROR;
    }
    result = spi_bus_acquire_ex(SPI_OWNER_IMU, &blocking_owner);

    if (result == BSP_SHARED_SPI_ACQUIRE_OK) return IMU_STATUS_OK;
    if (((result == BSP_SHARED_SPI_ACQUIRE_BUSY) ||
         (result == BSP_SHARED_SPI_ACQUIRE_PREPARE_BUSY)) &&
        (blocking_owner == SPI_OWNER_FLASH)) return IMU_STATUS_FLASH_BUSY;
    IMU_LOG_ERROR("acquire rc=%u blocker=%u", (uint32_t)result,
                  (uint32_t)blocking_owner);
    return IMU_STATUS_BUS_ERROR;
}

static imu_status_t imu_spi_read_transaction(uint8_t cmd,
                                             uint8_t *cmd_rx,
                                             uint8_t *rx,
                                             uint16_t rx_len,
                                             bool use_dma,
                                             bool report_busy)
{
    imu_status_t status;
    uint8_t discard_cmd_rx = 0U;
    uint8_t *cmd_rx_out = (cmd_rx != NULL) ? cmd_rx : &discard_cmd_rx;

    if ((rx == NULL) || (rx_len == 0U))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    /* The bus owner initializes the BSP after exclusive acquisition. */
    status = imu_spi_acquire_task(report_busy);
    if (status != IMU_STATUS_OK) return status;

    imu_bsp_cs_low();

    status = imu_bsp_spi_transfer(&cmd, cmd_rx_out, 1U);
    if (status == IMU_STATUS_OK)
    {
#if IMU_OIS_SPI_DMA_ENABLE
        if (use_dma)
        {
            status = imu_bsp_spi_read_dma(rx, rx_len);
        }
        else
#endif
        {
            status = imu_bsp_spi_transfer(NULL, rx, rx_len);
        }
    }

    imu_bsp_cs_high();
    spi_bus_release(SPI_OWNER_IMU);
    return status;
}

static bool imu_spi_mclk_is_ready(uint8_t raw)
{
    return ((raw & ICM53611_MCLK_RDY_RUNNING_MASK) != 0U);
}

static void imu_spi_log_mclk_state(uint8_t raw, bool ready, bool force)
{
#if IMU_MREG_DIAG_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE
    bool should_log = force;

    if (!s_imu_spi_mclk_log_seen)
    {
        should_log = true;
    }

    if (should_log)
    {
        DBG_DIRECT("[IMU][MCLK] raw=0x%02x otp_bit0=%u mclk_bit3=%u ready=%u",
                   raw,
                   ((raw & ICM53611_MCLK_RDY_OTP_OBSERVED_MASK) != 0U) ? 1U : 0U,
                   ((raw & ICM53611_MCLK_RDY_RUNNING_MASK) != 0U) ? 1U : 0U,
                   ready ? 1U : 0U);
        if (s_imu_spi_mclk_debug_log_cnt < 0xFFFFFFFFU)
        {
            s_imu_spi_mclk_debug_log_cnt++;
        }
    }

    s_imu_spi_mclk_log_seen = true;
#else
    IMU_UNUSED(raw);
    IMU_UNUSED(force);
#endif
    IMU_UNUSED(ready);
}

imu_status_t imu_spi_clock_end(imu_spi_clock_t *ctx)
{
    imu_status_t status = IMU_STATUS_OK;
    uint8_t pwr = 0U;
    if (ctx == NULL) return IMU_STATUS_INVALID_PARAM;
    if (ctx->release_status != 0xffU) return (imu_status_t)ctx->release_status;
    if (ctx->idle_modified)
    {
        status = imu_spi_read_reg(ICM53611_REG_PWR_MGMT0, &pwr);
        if (status == IMU_STATUS_OK)
        {
            pwr &= (uint8_t)~ICM53611_PWR_MGMT0_IDLE_EN;
            status = imu_spi_write_reg(ICM53611_REG_PWR_MGMT0, pwr);
            imu_bsp_delay_us(ICM53611_PWR_MGMT0_RESTORE_DELAY_US);
            if (status == IMU_STATUS_OK)
            {
                status = imu_spi_read_reg(ICM53611_REG_PWR_MGMT0, &pwr);
                if (status == IMU_STATUS_OK && (pwr & ICM53611_PWR_MGMT0_IDLE_EN))
                    status = IMU_STATUS_VERIFY_FAILED;
            }
        }
        if (status != IMU_STATUS_OK)
            pwrd_fault(PWRD_IO_RELEASE, 0U, ICM53611_REG_PWR_MGMT0, pwr,
                       status == IMU_STATUS_VERIFY_FAILED, status);
    }
    if (s_imu_error_active &&
        (s_imu_error.release_status == 0xffU || s_imu_error.release_status == IMU_STATUS_OK))
        s_imu_error.release_status = (uint8_t)status;
    imu_spi_error_record(5U, 0U, ICM53611_REG_PWR_MGMT0, pwr,
                         status == IMU_STATUS_VERIFY_FAILED, status);
    ctx->release_status = (uint8_t)status;
    return status;
}

static imu_status_t imu_spi_wait_mclk_ready(uint32_t timeout_ms, imu_spi_mreg_context_t *ctx)
{
    imu_status_t status;
    uint8_t raw = 0U;
    uint8_t pwr_mgmt0 = 0U;
    uint8_t pwr_mgmt0_after = 0U;
    bool ready;
    uint32_t timeout_us;
    uint32_t waited_us = 0U;

    if (ctx == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = imu_spi_read_reg(ICM53611_REG_MCLK_RDY, &raw);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    ready = imu_spi_mclk_is_ready(raw);
    imu_spi_log_mclk_state(raw, ready, false);
    /* DS p44: hold RC throughout indirect access, even if MCLK was momentarily ready. */
    status = imu_spi_read_reg(ICM53611_REG_PWR_MGMT0, &pwr_mgmt0);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    ctx->saved_pwr_mgmt0 = pwr_mgmt0;
    if ((pwr_mgmt0 & ICM53611_PWR_MGMT0_IDLE_EN) == 0U)
    {
        pwr_mgmt0_after = (uint8_t)(pwr_mgmt0 | ICM53611_PWR_MGMT0_IDLE_EN);
        ctx->idle_modified = true;
        status = imu_spi_write_reg(ICM53611_REG_PWR_MGMT0, pwr_mgmt0_after);
        if (status != IMU_STATUS_OK)
        {
            return status;
        }
        status = imu_spi_read_reg(ICM53611_REG_PWR_MGMT0, &pwr_mgmt0_after);
        if (status != IMU_STATUS_OK) return status;
        if (!(pwr_mgmt0_after & ICM53611_PWR_MGMT0_IDLE_EN))
        {
            imu_spi_error_record(4U, 0U, ICM53611_REG_PWR_MGMT0, pwr_mgmt0_after, true, IMU_STATUS_VERIFY_FAILED);
            pwrd_fault(PWRD_IO_CLOCK, 0U, ICM53611_REG_PWR_MGMT0, pwr_mgmt0_after, true, IMU_STATUS_VERIFY_FAILED);
            return IMU_STATUS_VERIFY_FAILED;
        }
        if (s_imu_spi_idle_enable_log_cnt < ICM53611_MCLK_DEBUG_BOOT_LOG_LIMIT)
        {
#if IMU_MREG_DIAG_LOG_ENABLE || IMU_RUNTIME_VERBOSE_LOG_ENABLE
            DBG_DIRECT("[IMU][MCLK] enable IDLE_EN pwr_before=0x%02x pwr_after=0x%02x",
                       pwr_mgmt0,
                       pwr_mgmt0_after);
#endif
            s_imu_spi_idle_enable_log_cnt++;
        }
    }

    timeout_us = timeout_ms * 1000U;
    while (waited_us < timeout_us)
    {
        imu_bsp_delay_us(ICM53611_MCLK_READY_POLL_US);
        waited_us += ICM53611_MCLK_READY_POLL_US;

        status = imu_spi_read_reg(ICM53611_REG_MCLK_RDY, &raw);
        if (status != IMU_STATUS_OK)
        {
            return status;
        }

        ready = imu_spi_mclk_is_ready(raw);
        imu_spi_log_mclk_state(raw, ready, false);
        if (ready)
        {
            return IMU_STATUS_OK;
        }
    }

    imu_spi_log_mclk_state(raw, false, true);
    imu_spi_error_record(4U, 0U, ICM53611_REG_MCLK_RDY, raw, true, IMU_STATUS_NOT_READY);
    pwrd_fault(PWRD_IO_CLOCK, 0U, ICM53611_REG_MCLK_RDY, raw, true, IMU_STATUS_NOT_READY);
    return IMU_STATUS_NOT_READY;
}

imu_status_t imu_spi_clock_begin(imu_spi_clock_t *ctx)
{
    imu_status_t status;
    if (ctx == NULL) return IMU_STATUS_INVALID_PARAM;
    ctx->saved_pwr_mgmt0 = 0U;
    ctx->idle_modified = false;
    ctx->release_status = 0xffU;
    status = imu_spi_wait_mclk_ready(ICM53611_MCLK_READY_TIMEOUT_MS, ctx);
    if (s_imu_error_active &&
        (s_imu_error.acquire_status == 0xffU || s_imu_error.acquire_status == IMU_STATUS_OK))
        s_imu_error.acquire_status = (uint8_t)status;
    if (status != IMU_STATUS_OK) IMU_UNUSED(imu_spi_clock_end(ctx));
    return status;
}

static imu_status_t imu_spi_prepare_mreg_access(imu_spi_mreg_context_t *ctx)
{
    imu_status_t status = imu_spi_clock_begin(ctx);
    if (status != IMU_STATUS_OK)
        pwrd_io_detail(0U, 0U, status, ctx->release_status, 0U, 6U);
    return status;
}

static imu_status_t imu_spi_finish_mreg_access(imu_spi_mreg_context_t *ctx, imu_status_t status_in)
{
    imu_status_t restore_status = imu_spi_clock_end(ctx);
    if (status_in != IMU_STATUS_OK || restore_status != IMU_STATUS_OK)
        pwrd_io_detail(0U, 0U, IMU_STATUS_OK, restore_status, 0U, 6U);
    return status_in != IMU_STATUS_OK ? status_in : restore_status;
}

static imu_status_t imu_spi_write_then_read(const uint8_t *tx, uint16_t tx_len,
                                             uint8_t *rx, uint16_t rx_len, bool report_busy)
{
    imu_status_t status;

    if (((tx_len != 0U) && (tx == NULL)) || ((rx_len != 0U) && (rx == NULL)))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    /* The bus owner initializes the BSP after exclusive acquisition. */
    status = imu_spi_acquire_task(report_busy);
    if (status != IMU_STATUS_OK) return status;

    imu_bsp_cs_low();

    if (tx_len != 0U)
    {
        status = imu_bsp_spi_transfer(tx, NULL, tx_len);
        if (status != IMU_STATUS_OK)
        {
            goto exit;
        }
    }

    if (rx_len != 0U)
    {
        status = imu_bsp_spi_transfer(NULL, rx, rx_len);
    }

exit:
    imu_bsp_cs_high();
    spi_bus_release(SPI_OWNER_IMU);
    return status;
}

imu_status_t imu_spi_bus_init(void)
{
    return imu_bsp_init();
}

static imu_status_t imu_spi_write_reg_impl(uint8_t addr, uint8_t value, bool report_busy)
{
    uint8_t tx[2];
    imu_status_t status;

    tx[0] = (uint8_t)(addr & 0x7FU);
    tx[1] = value;
    status = imu_spi_write_then_read(tx, 2U, NULL, 0U, report_busy);
    if (!report_busy) status = imu_status_legacy(status);
    imu_spi_error_record(2U, 0U, addr, 0U, false, status);
    if (status != IMU_STATUS_OK) pwrd_fault(PWRD_IO_WRITE, 0U, addr, 0U, false, status);

#if IMU_SPI_TRACE_ENABLE
    IMU_LOG_INFO("SPI W reg=0x%02x val=0x%02x status=%d", addr, value, status);
#endif
    if ((status != IMU_STATUS_OK) && (status != IMU_STATUS_FLASH_BUSY))
    {
        IMU_LOG_ERROR("write r=%02x st=%d", addr, status);
    }

    return status;
}

imu_status_t imu_spi_write_reg(uint8_t addr, uint8_t value)
{
    return imu_spi_write_reg_impl(addr, value, false);
}

imu_status_t imu_spi_write_reg_ex(uint8_t addr, uint8_t value)
{
    return imu_spi_write_reg_impl(addr, value, true);
}

imu_status_t imu_spi_read_reg(uint8_t addr, uint8_t *value)
{
    imu_status_t status;

    if (value == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = imu_spi_read_regs(addr, value, 1U);

#if IMU_SPI_TRACE_ENABLE
    IMU_LOG_INFO("SPI R reg=0x%02x val=0x%02x status=%d", addr, *value, status);
#endif
    return status;
}

static imu_status_t imu_spi_read_regs_impl(uint8_t addr, uint8_t *buf, uint16_t len, bool report_busy)
{
    uint8_t cmd;
    uint8_t cmd_rx = 0U;
    imu_status_t status;
    bool trace_whoami;
    bool use_dma;
#if IMU_SPI_WHOAMI_TRACE_ENABLE
    uint8_t extra_cmd_rx = 0U;
    uint8_t extra_rx[2] = {0U, 0U};
    imu_status_t extra_status = IMU_STATUS_OK;
#endif

    if ((buf == NULL) || (len == 0U))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    cmd = (uint8_t)((addr & 0x7FU) | ICM53611_SPI_READ_MASK);
    trace_whoami = imu_spi_should_trace_whoami(addr);
    /*
     * OIS current-register DMA transition path:
     * keep DMA scoped to the 19-byte OIS window only. This path does not
     * provide historical sample replay capability.
     */
    use_dma = (((addr & 0x7FU) == ICM53611_REG_TEMP_DATA1_OIS) &&
               (len == ICM53611_OIS_WINDOW_BYTES));
    status = imu_spi_read_transaction(cmd, trace_whoami ? &cmd_rx : NULL, buf, len, use_dma, report_busy);

    if (!report_busy) status = imu_status_legacy(status);
    imu_spi_error_record(1U, 0U, addr, 0U, false, status);
    if (status != IMU_STATUS_OK) pwrd_fault(PWRD_IO_READ, 0U, addr, 0U, false, status);

    if (trace_whoami)
    {
        IMU_LOG_INFO("WHO_AM_I cmd calc: (addr=0x%02x & 0x7f) | READ_MASK(0x%02x) => 0x%02x",
                     addr,
                     ICM53611_SPI_READ_MASK,
                     cmd);
        imu_spi_log_read_raw_bytes("WHO_AM_I std", addr, cmd, cmd_rx, buf, len);

#if IMU_SPI_WHOAMI_TRACE_ENABLE
        if (len == 1U)
        {
            extra_status = imu_spi_read_transaction(cmd, &extra_cmd_rx, extra_rx, 2U, false, report_busy);
            if (extra_status == IMU_STATUS_OK)
            {
                imu_spi_log_read_raw_bytes("WHO_AM_I +1dummy", addr, cmd, extra_cmd_rx, extra_rx, 2U);
                IMU_LOG_INFO("WHO_AM_I align probe: std=0x%02x extra[0]=0x%02x extra[1]=0x%02x",
                             buf[0], extra_rx[0], extra_rx[1]);
                if ((buf[0] != ICM53611_WHO_AM_I_VALUE) && (extra_rx[1] == ICM53611_WHO_AM_I_VALUE))
                {
                    IMU_LOG_WARN("WHO dummy2");
                }
            }
            else
            {
                IMU_LOG_WARN("WHO dummy st=%d", extra_status);
            }
        }
#endif
        s_imu_spi_whoami_trace_cnt++;
    }

    if ((status != IMU_STATUS_OK) && (status != IMU_STATUS_FLASH_BUSY))
    {
        IMU_LOG_ERROR("read r=%02x n=%u st=%d", addr, len, status);
    }

    return status;
}

imu_status_t imu_spi_read_regs(uint8_t addr, uint8_t *buf, uint16_t len)
{
    return imu_spi_read_regs_impl(addr, buf, len, false);
}

imu_status_t imu_spi_read_regs_ex(uint8_t addr, uint8_t *buf, uint16_t len)
{
    return imu_spi_read_regs_impl(addr, buf, len, true);
}

imu_status_t imu_spi_read_reg_ex(uint8_t addr, uint8_t *value)
{
    return imu_spi_read_regs_ex(addr, value, 1U);
}

imu_status_t imu_spi_read_regs_isr_fast(uint8_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t cmd;
    uint8_t cmd_rx = 0U;
    imu_status_t status;

    if ((buf == NULL) || (len == 0U))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    cmd = (uint8_t)((addr & 0x7FU) | ICM53611_SPI_READ_MASK);
    if (!spi_bus_acquire(SPI_OWNER_IMU))
    {
        return IMU_STATUS_BUS_ERROR;
    }

    imu_bsp_cs_low();

    status = imu_bsp_spi_transfer_isr_fast(&cmd, &cmd_rx, 1U);
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_transfer_isr_fast(NULL, buf, len);
    }

    imu_bsp_cs_high();
    spi_bus_release(SPI_OWNER_IMU);
    return status;
}

imu_status_t imu_spi_read_regs_isr_fast_locked(uint8_t addr,
                                               uint8_t *buf,
                                               uint16_t len)
{
    uint8_t cmd;
    imu_status_t status;

    if ((buf == NULL) || (len == 0U))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

#if (ZY100_LOCKED_SPI_DEBUG_CHECK != 0U)
    if (spi_bus_current_owner() != SPI_OWNER_IMU)
    {
        return IMU_STATUS_NOT_READY;
    }
#endif

    cmd = (uint8_t)((addr & 0x7FU) | ICM53611_SPI_READ_MASK);
    imu_bsp_cs_low();
    status = imu_bsp_spi_read_reg_window_isr_fast(cmd, buf, len);
    imu_bsp_cs_high();
    return status;
}

imu_status_t imu_spi_write_regs(uint8_t addr, const uint8_t *buf, uint16_t len)
{
    imu_status_t status;
    uint8_t cmd;

    if ((buf == NULL) || (len == 0U))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = imu_bsp_init();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    cmd = (uint8_t)(addr & 0x7FU);
    if (!spi_bus_acquire(SPI_OWNER_IMU))
    {
        return IMU_STATUS_BUS_ERROR;
    }

    imu_bsp_cs_low();

    status = imu_bsp_spi_transfer(&cmd, NULL, 1U);
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_transfer(buf, NULL, len);
    }

    imu_bsp_cs_high();
    spi_bus_release(SPI_OWNER_IMU);

    if (status != IMU_STATUS_OK)
    {
        IMU_LOG_ERROR("write r=%02x n=%u st=%d", addr, len, status);
    }

    return status;
}

static imu_status_t imu_spi_mreg_write_internal(uint8_t block,
                                                uint8_t maddr,
                                                uint8_t value,
                                                bool delay_after_maddr)
{
    imu_status_t status;
    imu_spi_mreg_context_t mreg_ctx;

    status = imu_spi_prepare_mreg_access(&mreg_ctx);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = imu_spi_write_reg(ICM53611_REG_BLK_SEL_W, block);
    if (status != IMU_STATUS_OK)
    {
        goto exit;
    }

    status = imu_spi_write_reg(ICM53611_REG_MADDR_W, maddr);
    if (status != IMU_STATUS_OK)
    {
        goto exit;
    }

    if (delay_after_maddr)
    {
        imu_bsp_delay_us(ICM53611_MREG_NO_ACCESS_DELAY_US);
    }

    status = imu_spi_write_reg(ICM53611_REG_M_W, value);
    imu_bsp_delay_us(ICM53611_MREG_NO_ACCESS_DELAY_US);
    if (status != IMU_STATUS_OK)
    {
        goto exit;
    }

exit:
    if (imu_spi_write_reg(ICM53611_REG_BLK_SEL_W, ICM53611_MREG_BLOCK_1) != IMU_STATUS_OK)
    {
        if (status == IMU_STATUS_OK)
        {
            status = IMU_STATUS_BUS_ERROR;
        }
    }

    return imu_spi_finish_mreg_access(&mreg_ctx, status);
}

imu_status_t imu_spi_mreg_write(uint8_t block, uint8_t maddr, uint8_t value)
{
    return imu_spi_mreg_write_internal(block, maddr, value, false);
}

imu_status_t imu_spi_mreg_write_with_maddr_delay(uint8_t block, uint8_t maddr, uint8_t value)
{
    return imu_spi_mreg_write_internal(block, maddr, value, true);
}

imu_status_t imu_spi_mreg_write_read_same_session(uint8_t block,
                                                  uint8_t maddr,
                                                  uint8_t value,
                                                  uint8_t *readback,
                                                  uint8_t *mclk_before,
                                                  uint8_t *mclk_after)
{
    imu_status_t status;
    imu_status_t reset_status;
    imu_spi_mreg_context_t mreg_ctx;

    if ((readback == NULL) || (mclk_before == NULL) || (mclk_after == NULL))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    *readback = 0U;
    *mclk_before = 0U;
    *mclk_after = 0U;

    IMU_UNUSED(imu_spi_read_reg(ICM53611_REG_MCLK_RDY, mclk_before));
    status = imu_spi_prepare_mreg_access(&mreg_ctx);
    if (status != IMU_STATUS_OK)
    {
        IMU_UNUSED(imu_spi_read_reg(ICM53611_REG_MCLK_RDY, mclk_after));
        return status;
    }

    status = imu_spi_write_reg(ICM53611_REG_BLK_SEL_W, block);
    if (status != IMU_STATUS_OK)
    {
        goto exit;
    }

    status = imu_spi_write_reg(ICM53611_REG_MADDR_W, maddr);
    if (status != IMU_STATUS_OK)
    {
        goto exit;
    }

    status = imu_spi_write_reg(ICM53611_REG_M_W, value);
    imu_bsp_delay_us(ICM53611_MREG_NO_ACCESS_DELAY_US);
    if (status != IMU_STATUS_OK)
    {
        goto exit;
    }

    status = imu_spi_write_reg(ICM53611_REG_BLK_SEL_R, block);
    if (status != IMU_STATUS_OK)
    {
        goto exit;
    }

    status = imu_spi_write_reg(ICM53611_REG_MADDR_R, maddr);
    if (status != IMU_STATUS_OK)
    {
        goto exit;
    }

    imu_bsp_delay_us(ICM53611_MREG_NO_ACCESS_DELAY_US);
    status = imu_spi_read_reg(ICM53611_REG_M_R, readback);
    imu_bsp_delay_us(ICM53611_MREG_NO_ACCESS_DELAY_US);

exit:
    IMU_UNUSED(imu_spi_read_reg(ICM53611_REG_MCLK_RDY, mclk_after));

    reset_status = imu_spi_write_reg(ICM53611_REG_BLK_SEL_W, ICM53611_MREG_BLOCK_1);
    if ((status == IMU_STATUS_OK) && (reset_status != IMU_STATUS_OK))
    {
        status = reset_status;
    }

    reset_status = imu_spi_write_reg(ICM53611_REG_BLK_SEL_R, ICM53611_MREG_BLOCK_1);
    if ((status == IMU_STATUS_OK) && (reset_status != IMU_STATUS_OK))
    {
        status = reset_status;
    }

    return imu_spi_finish_mreg_access(&mreg_ctx, status);
}

imu_status_t imu_spi_mreg_read(uint8_t block, uint8_t maddr, uint8_t *value)
{
    imu_status_t status;
    imu_spi_mreg_context_t mreg_ctx;

    if (value == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = imu_spi_prepare_mreg_access(&mreg_ctx);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    status = imu_spi_write_reg(ICM53611_REG_BLK_SEL_R, block);
    if (status != IMU_STATUS_OK)
    {
        goto exit;
    }

    status = imu_spi_write_reg(ICM53611_REG_MADDR_R, maddr);
    if (status != IMU_STATUS_OK)
    {
        goto exit;
    }

    imu_bsp_delay_us(ICM53611_MREG_NO_ACCESS_DELAY_US);
    status = imu_spi_read_reg(ICM53611_REG_M_R, value);
    imu_bsp_delay_us(ICM53611_MREG_NO_ACCESS_DELAY_US);
    if (status != IMU_STATUS_OK)
    {
        goto exit;
    }

exit:
    if (imu_spi_write_reg(ICM53611_REG_BLK_SEL_R, ICM53611_MREG_BLOCK_1) != IMU_STATUS_OK)
    {
        if (status == IMU_STATUS_OK)
        {
            status = IMU_STATUS_BUS_ERROR;
        }
    }

    return imu_spi_finish_mreg_access(&mreg_ctx, status);
}
