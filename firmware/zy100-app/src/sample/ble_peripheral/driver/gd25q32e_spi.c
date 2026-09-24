#include "gd25q32e_spi.h"

#include <stddef.h>

#include "trace.h"

#include "../app_flags.h"
#include "../bsp/imu_bsp.h"
#include "spi_bus_owner.h"

#define GD25Q32E_CMD_WREN             0x06U
#define GD25Q32E_CMD_RDSR1            0x05U
#define GD25Q32E_CMD_RDSR2            0x35U
#define GD25Q32E_CMD_RDSR3            0x15U
#define GD25Q32E_CMD_READ             0x03U
#define GD25Q32E_CMD_FAST_READ        0x0BU
#define GD25Q32E_CMD_PAGE_PROGRAM     0x02U
#define GD25Q32E_CMD_SECTOR_ERASE_4K  0x20U
#define GD25Q32E_CMD_BLOCK_ERASE_32K  0x52U
#define GD25Q32E_CMD_CHIP_ERASE       0xC7U
#define GD25Q32E_CMD_RDID             0x9FU
#define GD25Q32E_CMD_DEEP_POWER_DOWN  0xB9U
#define GD25Q32E_CMD_RELEASE_DPD      0xABU

#define GD25Q32E_SR1_WIP              0x01U
#define GD25Q32E_SR1_BP_MASK          0x7CU
#define GD25Q32E_SR1_SRP0             0x80U
#define GD25Q32E_SR2_CMP              0x40U
#define GD25Q32E_SR2_SRP1             0x01U
#define GD25Q32E_READY_POLL_DELAY_US  100U
#define GD25Q32E_DPD_DELAY_US         20U
#define GD25Q32E_RELEASE_DPD_DELAY_US 40U

static bool s_gd25q32e_dpd_armed = false;

#if ZY100_LOG_FLASH_VERBOSE
static uint8_t s_gd25q32e_cs_before_init = 0U;
static uint8_t s_gd25q32e_cs_after_init = 0U;

static uint8_t gd25q32e_read_flash_cs_input(void)
{
    uint8_t out_level = 0U;
    uint8_t in_level = 0U;

    if (imu_bsp_get_flash_cs_level(&out_level, &in_level) != IMU_STATUS_OK)
    {
        return 0U;
    }
    return in_level;
}
#endif

static void gd25q32e_log_spi_config(void)
{
#if ZY100_LOG_FLASH_VERBOSE
    uint32_t source_hz = 0U;
    uint32_t clk_div = 0U;
    uint32_t baud_prescaler = 0U;
    uint32_t sclk_hz = 0U;

    imu_bsp_get_spi_nominal_config(&source_hz, &clk_div, &baud_prescaler, &sclk_hz);
    DBG_DIRECT("[FLASH_SPI_CFG] source=%u clk_div=%u baud_prescaler=%u sclk=%u",
               source_hz,
               clk_div,
               baud_prescaler,
               sclk_hz);
#endif
}

static void gd25q32e_addr_to_bytes(uint32_t addr, uint8_t *out)
{
    out[0] = (uint8_t)((addr >> 16) & 0xFFU);
    out[1] = (uint8_t)((addr >> 8) & 0xFFU);
    out[2] = (uint8_t)(addr & 0xFFU);
}

imu_status_t gd25q32e_release_deep_power_down(void)
{
    imu_status_t status;
    uint8_t cmd = GD25Q32E_CMD_RELEASE_DPD;

    /* An attempted release may reach the part even if the bus reports an
     * error. Never reuse the retained-DPD shortcut after an uncertain exit. */
    s_gd25q32e_dpd_armed = false;

    if (!spi_bus_acquire(SPI_OWNER_FLASH))
    {
        return IMU_STATUS_BUS_ERROR;
    }

    status = imu_bsp_flash_cs_low();
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_transfer(&cmd, NULL, 1U);
    }
    (void)imu_bsp_flash_cs_high();
    spi_bus_release(SPI_OWNER_FLASH);

    if (status == IMU_STATUS_OK)
    {
        imu_bsp_delay_us(GD25Q32E_RELEASE_DPD_DELAY_US);
        s_gd25q32e_dpd_armed = false;
    }
    if ((status != IMU_STATUS_OK) || ZY100_LOG_FLASH_VERBOSE)
    {
        DBG_DIRECT("[FLASH_DPD] release status=%u delay_us=%u",
                   (uint32_t)status,
                   (status == IMU_STATUS_OK) ? GD25Q32E_RELEASE_DPD_DELAY_US : 0U);
    }
    return status;
}

static imu_status_t gd25q32e_send_deep_power_down_cmd(void)
{
    imu_status_t status;
    uint8_t cmd = GD25Q32E_CMD_DEEP_POWER_DOWN;

    if (!spi_bus_acquire(SPI_OWNER_FLASH))
    {
        return IMU_STATUS_BUS_ERROR;
    }

    status = imu_bsp_flash_cs_low();
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_transfer(&cmd, NULL, 1U);
    }
    (void)imu_bsp_flash_cs_high();
    spi_bus_release(SPI_OWNER_FLASH);

    if (status == IMU_STATUS_OK)
    {
        imu_bsp_delay_us(GD25Q32E_DPD_DELAY_US);
    }
    return status;
}

bool gd25q32e_dpd_is_armed(void)
{
    return s_gd25q32e_dpd_armed;
}

imu_status_t gd25q32e_enter_deep_power_down(bool verify)
{
    imu_status_t status;
    gd25q32e_jedec_id_t id_before = {0U};
    gd25q32e_jedec_id_t id_after = {0U};
    bool before_normal = false;
    bool after_normal = false;

    /* Successful DPD entry is valid until release or notified rail loss. */
    if (s_gd25q32e_dpd_armed)
    {
        return IMU_STATUS_OK;
    }

    if (verify)
    {
        status = gd25q32e_read_jedec_id_attempt(&id_before, 0U);
        before_normal = ((status == IMU_STATUS_OK) && gd25q32e_jedec_is_4mbyte(&id_before));
        if (!before_normal && !s_gd25q32e_dpd_armed)
        {
            return (status == IMU_STATUS_OK) ? IMU_STATUS_NOT_READY : status;
        }
    }

    status = gd25q32e_send_deep_power_down_cmd();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    if (verify)
    {
        status = gd25q32e_read_jedec_id_attempt(&id_after, 1U);
        if (status != IMU_STATUS_OK) return status;
        after_normal = gd25q32e_jedec_is_4mbyte(&id_after);
        if (after_normal)
        {
            return IMU_STATUS_VERIFY_FAILED;
        }
    }

    s_gd25q32e_dpd_armed = true;
    return IMU_STATUS_OK;
}

void gd25q32e_notify_power_lost(void)
{
    s_gd25q32e_dpd_armed = false;
}

imu_status_t gd25q32e_init(void)
{
    imu_status_t status;
    if (spi_bus_current_owner() != SPI_OWNER_NONE) return IMU_STATUS_NOT_READY;

#if ZY100_LOG_FLASH_VERBOSE
    DBG_DIRECT("[FLASH_SEQ] step=gd25_init_begin");
#endif
    gd25q32e_log_spi_config();
#if ZY100_LOG_FLASH_VERBOSE
    s_gd25q32e_cs_before_init = gd25q32e_read_flash_cs_input();
#endif

    /* Bus acquisition restores both chip selects under the access lease. */
    if (!spi_bus_acquire(SPI_OWNER_FLASH)) return IMU_STATUS_NOT_READY;
    spi_bus_release(SPI_OWNER_FLASH);
#if ZY100_LOG_FLASH_VERBOSE
    s_gd25q32e_cs_after_init = gd25q32e_read_flash_cs_input();
#endif

#if ZY100_LOG_FLASH_VERBOSE
    DBG_DIRECT("[FLASH_SEQ] step=force_idle_before_dpd");
#endif
    status = gd25q32e_release_deep_power_down();
#if ZY100_LOG_FLASH_VERBOSE
    DBG_DIRECT("[FLASH_SEQ] step=release_dpd status=%u", (uint32_t)status);
    DBG_DIRECT("[FLASH_SEQ] step=force_idle_after_dpd");
#endif
    return status;
}

imu_status_t gd25q32e_read_jedec_id(gd25q32e_jedec_id_t *id)
{
    return gd25q32e_read_jedec_id_attempt(id, 0U);
}

static imu_status_t gd25q32e_read_jedec_full_duplex(uint8_t *rx)
{
    imu_status_t status;
    uint8_t tx[4] = {GD25Q32E_CMD_RDID, 0U, 0U, 0U};

    if (!spi_bus_acquire(SPI_OWNER_FLASH))
    {
        return IMU_STATUS_BUS_ERROR;
    }

    status = imu_bsp_flash_cs_low();
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_transfer(tx, rx, sizeof(tx));
    }
    (void)imu_bsp_flash_cs_high();
    spi_bus_release(SPI_OWNER_FLASH);
    return status;
}

#if ZY100_FLASH_JEDEC_SPLIT_READ_DIAG
static imu_status_t gd25q32e_read_jedec_split(uint8_t *rx)
{
    imu_status_t status;
    uint8_t cmd = GD25Q32E_CMD_RDID;

    if (!spi_bus_acquire(SPI_OWNER_FLASH))
    {
        return IMU_STATUS_BUS_ERROR;
    }

    status = imu_bsp_flash_cs_low();
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_transfer(&cmd, &rx[0], 1U);
    }
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_transfer(NULL, &rx[1], 3U);
    }
    (void)imu_bsp_flash_cs_high();
    spi_bus_release(SPI_OWNER_FLASH);
    return status;
}
#endif

imu_status_t gd25q32e_read_jedec_id_attempt(gd25q32e_jedec_id_t *id, uint32_t attempt)
{
    imu_status_t status;
    uint8_t rx[4] = {0U};
#if ZY100_LOG_FLASH_VERBOSE
    uint8_t cs_before_jedec;
#endif
    bool jedec_all_ff;

    if (id == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

#if ZY100_LOG_FLASH_VERBOSE
    DBG_DIRECT("[FLASH_SEQ] step=jedec_attempt attempt=%u", attempt);
#endif
    gd25q32e_log_spi_config();
#if ZY100_LOG_FLASH_VERBOSE
    cs_before_jedec = gd25q32e_read_flash_cs_input();
    DBG_DIRECT("[FLASH_PAD] cs_before_init=%u cs_after_init=%u cs_before_jedec=%u",
               s_gd25q32e_cs_before_init,
               s_gd25q32e_cs_after_init,
               cs_before_jedec);
#endif

#if ZY100_FLASH_JEDEC_SPLIT_READ_DIAG
    status = gd25q32e_read_jedec_split(rx);
#else
    status = gd25q32e_read_jedec_full_duplex(rx);
#endif

    if (status != IMU_STATUS_OK)
    {
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[FLASH_JEDEC_RAW] attempt=%u rx0=%02x rx1=%02x rx2=%02x rx3=%02x parsed=%02x%02x%02x",
                   attempt,
                   rx[0],
                   rx[1],
                   rx[2],
                   rx[3],
                   rx[1],
                   rx[2],
                   rx[3]);
        ZY100_LOG_ERROR("[FLASH_JEDEC] attempt=%u id=%02x%02x%02x status=%u",
                   attempt,
                   rx[1],
                   rx[2],
                   rx[3],
                   (uint32_t)status);
        return status;
    }

    id->manufacturer_id = rx[1];
    id->memory_type = rx[2];
    id->density = rx[3];
    jedec_all_ff = (id->manufacturer_id == 0xFFU) &&
                   (id->memory_type == 0xFFU) &&
                   (id->density == 0xFFU);
    if (ZY100_LOG_FLASH_VERBOSE || jedec_all_ff)
    {
        ZY100_LOG_ROUTINE(DBG_DIRECT, "[FLASH_JEDEC_RAW] attempt=%u rx0=%02x rx1=%02x rx2=%02x rx3=%02x parsed=%02x%02x%02x",
                   attempt,
                   rx[0],
                   rx[1],
                   rx[2],
                   rx[3],
                   id->manufacturer_id,
                   id->memory_type,
                   id->density);
        ZY100_LOG_DETAIL("[FLASH_JEDEC] attempt=%u id=%02x%02x%02x status=%u",
                   attempt,
                   id->manufacturer_id,
                   id->memory_type,
                   id->density,
                   (uint32_t)status);
    }
    return IMU_STATUS_OK;
}

bool gd25q32e_jedec_is_4mbyte(const gd25q32e_jedec_id_t *id)
{
    if (id == NULL)
    {
        return false;
    }

    return (id->density == 0x16U);
}

static imu_status_t gd25q32e_read_status_reg(uint8_t cmd, uint8_t *value)
{
    imu_status_t status;

    if (value == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    if (!spi_bus_acquire(SPI_OWNER_FLASH))
    {
        return IMU_STATUS_BUS_ERROR;
    }

    status = imu_bsp_flash_cs_low();
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_transfer(&cmd, NULL, 1U);
    }
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_transfer(NULL, value, 1U);
    }
    (void)imu_bsp_flash_cs_high();
    spi_bus_release(SPI_OWNER_FLASH);
    return status;
}

imu_status_t gd25q32e_read_status(gd25q32e_status_regs_t *status_regs)
{
    imu_status_t status;

    if (status_regs == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = gd25q32e_read_status_reg(GD25Q32E_CMD_RDSR1, &status_regs->sr1);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    status = gd25q32e_read_status_reg(GD25Q32E_CMD_RDSR2, &status_regs->sr2);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    return gd25q32e_read_status_reg(GD25Q32E_CMD_RDSR3, &status_regs->sr3);
}

imu_status_t gd25q32e_resume_and_verify(gd25q32e_jedec_id_t *id,
                                        gd25q32e_status_regs_t *status_regs)
{
    imu_status_t status;

    if ((id == NULL) || (status_regs == NULL))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    if (spi_bus_current_owner() != SPI_OWNER_NONE) return IMU_STATUS_NOT_READY;
    status = gd25q32e_release_deep_power_down();
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[FLASH_RESUME][ERR] release status=%u", (uint32_t)status);
        return status;
    }


    status = gd25q32e_read_jedec_id(id);
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[FLASH_RESUME][ERR] jedec_read status=%u", (uint32_t)status);
        return status;
    }
    if (!gd25q32e_jedec_is_4mbyte(id))
    {
        DBG_DIRECT("[FLASH_RESUME][ERR] jedec=%02X%02X%02X capacity_invalid=1",
                   id->manufacturer_id,
                   id->memory_type,
                   id->density);
        return IMU_STATUS_VERIFY_FAILED;
    }

    status = gd25q32e_read_status(status_regs);
    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[FLASH_RESUME][ERR] status_read=%u", (uint32_t)status);
        return status;
    }
    if (status_regs->sr1 == 0xFFU)
    {
        DBG_DIRECT("[FLASH_RESUME][ERR] invalid_sr1=0xFF");
        return IMU_STATUS_NOT_READY;
    }
    if ((status_regs->sr1 & GD25Q32E_SR1_WIP) != 0U)
    {
        DBG_DIRECT("[FLASH_RESUME][ERR] busy sr1=0x%02X", status_regs->sr1);
        return IMU_STATUS_NOT_READY;
    }

    ZY100_LOG_ROUTINE(DBG_DIRECT, "[FLASH_RESUME] ready jedec=%02X%02X%02X sr1=0x%02X",
               id->manufacturer_id,
               id->memory_type,
               id->density,
               status_regs->sr1);
    return IMU_STATUS_OK;
}

bool gd25q32e_status_is_protected(const gd25q32e_status_regs_t *status_regs)
{
    if (status_regs == NULL)
    {
        return true;
    }

    if ((status_regs->sr1 & (GD25Q32E_SR1_BP_MASK | GD25Q32E_SR1_SRP0)) != 0U)
    {
        return true;
    }
    if ((status_regs->sr2 & (GD25Q32E_SR2_CMP | GD25Q32E_SR2_SRP1)) != 0U)
    {
        return true;
    }
    return false;
}

imu_status_t gd25q32e_write_enable(void)
{
    imu_status_t status;
    uint8_t cmd = GD25Q32E_CMD_WREN;

    if (!spi_bus_acquire(SPI_OWNER_FLASH))
    {
        return IMU_STATUS_BUS_ERROR;
    }

    status = imu_bsp_flash_cs_low();
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_transfer(&cmd, NULL, 1U);
    }
    (void)imu_bsp_flash_cs_high();
    spi_bus_release(SPI_OWNER_FLASH);
    return status;
}

static imu_status_t gd25q32e_chip_erase_impl(void)
{
    imu_status_t status;
    uint8_t cmd = GD25Q32E_CMD_CHIP_ERASE;

    status = gd25q32e_write_enable();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    if (!spi_bus_acquire(SPI_OWNER_FLASH))
    {
        return IMU_STATUS_BUS_ERROR;
    }

    status = imu_bsp_flash_cs_low();
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_transfer(&cmd, NULL, 1U);
    }
    (void)imu_bsp_flash_cs_high();
    spi_bus_release(SPI_OWNER_FLASH);
    return status;
}

imu_status_t gd25q32e_chip_erase(void)
{
    return gd25q32e_chip_erase_impl();
}

static imu_status_t gd25q32e_sector_erase_4k_impl(uint32_t addr)
{
    imu_status_t status;
    uint8_t cmd[4];

    if (((addr & (GD25Q32E_SECTOR_BYTES - 1U)) != 0U) ||
        (addr >= GD25Q32E_FLASH_SIZE_BYTES))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = gd25q32e_write_enable();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    cmd[0] = GD25Q32E_CMD_SECTOR_ERASE_4K;
    gd25q32e_addr_to_bytes(addr, &cmd[1]);

    if (!spi_bus_acquire(SPI_OWNER_FLASH))
    {
        return IMU_STATUS_BUS_ERROR;
    }

    status = imu_bsp_flash_cs_low();
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_transfer(cmd, NULL, sizeof(cmd));
    }
    (void)imu_bsp_flash_cs_high();
    spi_bus_release(SPI_OWNER_FLASH);
    return status;
}

imu_status_t gd25q32e_sector_erase_4k(uint32_t addr)
{
    return gd25q32e_sector_erase_4k_impl(addr);
}

static imu_status_t gd25q32e_block_erase_32k_impl(uint32_t addr)
{
    imu_status_t status;
    uint8_t cmd[4];

    if (((addr & (GD25Q32E_BLOCK32_BYTES - 1UL)) != 0UL) ||
        (addr > (GD25Q32E_FLASH_SIZE_BYTES - GD25Q32E_BLOCK32_BYTES)))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = gd25q32e_write_enable();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    cmd[0] = GD25Q32E_CMD_BLOCK_ERASE_32K;
    gd25q32e_addr_to_bytes(addr, &cmd[1]);

    if (!spi_bus_acquire(SPI_OWNER_FLASH))
    {
        return IMU_STATUS_BUS_ERROR;
    }

    status = imu_bsp_flash_cs_low();
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_transfer(cmd, NULL, sizeof(cmd));
    }
    (void)imu_bsp_flash_cs_high();
    spi_bus_release(SPI_OWNER_FLASH);
    return status;
}

imu_status_t gd25q32e_block_erase_32k(uint32_t addr)
{
    return gd25q32e_block_erase_32k_impl(addr);
}

/* Data segment only: short command/status transactions stay unchanged. */
static imu_status_t gd25q32e_transfer_data(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
#if ZY100_FLASH_FIFO_TRANSFER_ENABLE
    if (len >= 16U) { return imu_bsp_spi_transfer_fifo(tx, rx, len); }
#endif
    return imu_bsp_spi_transfer(tx, rx, len);
}

static imu_status_t gd25q32e_read_impl(uint32_t addr, uint8_t *buf, uint16_t len)
{
    imu_status_t status;
    uint8_t cmd[4];

    if ((buf == NULL) || (len == 0U) ||
        (addr >= GD25Q32E_FLASH_SIZE_BYTES) ||
        ((uint32_t)len > (GD25Q32E_FLASH_SIZE_BYTES - addr)))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    cmd[0] = GD25Q32E_CMD_READ;
    gd25q32e_addr_to_bytes(addr, &cmd[1]);

    if (!spi_bus_acquire(SPI_OWNER_FLASH))
    {
        return IMU_STATUS_BUS_ERROR;
    }

    status = imu_bsp_flash_cs_low();
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_transfer(cmd, NULL, sizeof(cmd));
    }
    if (status == IMU_STATUS_OK)
    {
        status = gd25q32e_transfer_data(NULL, buf, len);
    }
    (void)imu_bsp_flash_cs_high();
    spi_bus_release(SPI_OWNER_FLASH);
    return status;
}

imu_status_t gd25q32e_read(uint32_t addr, uint8_t *buf, uint16_t len)
{
    return gd25q32e_read_impl(addr, buf, len);
}

static imu_status_t gd25q32e_read_fast_impl(uint32_t addr, uint8_t *buf, uint16_t len)
{
    imu_status_t status;
    uint8_t cmd[5];

    if ((buf == NULL) || (len == 0U) ||
        (addr >= GD25Q32E_FLASH_SIZE_BYTES) ||
        ((uint32_t)len > (GD25Q32E_FLASH_SIZE_BYTES - addr)))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    cmd[0] = GD25Q32E_CMD_FAST_READ;
    gd25q32e_addr_to_bytes(addr, &cmd[1]);
    cmd[4] = 0x00U;

    if (!spi_bus_acquire(SPI_OWNER_FLASH))
    {
        return IMU_STATUS_BUS_ERROR;
    }

    status = imu_bsp_flash_cs_low();
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_transfer(cmd, NULL, sizeof(cmd));
    }
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_read_quiet(buf, len);
    }
    (void)imu_bsp_flash_cs_high();
    spi_bus_release(SPI_OWNER_FLASH);
    return status;
}

imu_status_t gd25q32e_read_fast(uint32_t addr, uint8_t *buf, uint16_t len)
{
    return gd25q32e_read_fast_impl(addr, buf, len);
}

static imu_status_t gd25q32e_page_program_impl(uint32_t addr,
                                               const uint8_t *buf,
                                               uint16_t len)
{
    imu_status_t status;
    uint8_t cmd[4];

    if ((buf == NULL) || (len != GD25Q32E_PAGE_BYTES) ||
        ((addr & (GD25Q32E_PAGE_BYTES - 1U)) != 0U) ||
        (addr >= GD25Q32E_FLASH_SIZE_BYTES) ||
        ((uint32_t)len > (GD25Q32E_FLASH_SIZE_BYTES - addr)))
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = gd25q32e_write_enable();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    cmd[0] = GD25Q32E_CMD_PAGE_PROGRAM;
    gd25q32e_addr_to_bytes(addr, &cmd[1]);

    if (!spi_bus_acquire(SPI_OWNER_FLASH))
    {
        return IMU_STATUS_BUS_ERROR;
    }

    status = imu_bsp_flash_cs_low();
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_transfer(cmd, NULL, sizeof(cmd));
    }
    if (status == IMU_STATUS_OK)
    {
        status = gd25q32e_transfer_data(buf, NULL, len);
    }
    (void)imu_bsp_flash_cs_high();
    spi_bus_release(SPI_OWNER_FLASH);
    return status;
}

imu_status_t gd25q32e_page_program(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    return gd25q32e_page_program_impl(addr, buf, len);
}

imu_status_t gd25q32e_page_program_partial_erased(uint32_t addr,
                                                  const uint8_t *buf,
                                                  uint16_t len)
{
    imu_status_t status;
    uint8_t existing[GD25Q32E_PAGE_BYTES];
    uint8_t cmd[4];
    uint16_t index;

    if ((buf == NULL) || (len == 0U) || (len > GD25Q32E_PAGE_BYTES) ||
        ((uint32_t)(addr & (GD25Q32E_PAGE_BYTES - 1U)) + len >
         GD25Q32E_PAGE_BYTES) ||
        (addr >= GD25Q32E_FLASH_SIZE_BYTES) ||
        ((uint32_t)len > (GD25Q32E_FLASH_SIZE_BYTES - addr)))
    {
        return IMU_STATUS_INVALID_PARAM;
    }
    status = gd25q32e_read(addr, existing, len);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    for (index = 0U; index < len; index++)
    {
        if (existing[index] != 0xFFU)
        {
            return IMU_STATUS_INVALID_PARAM;
        }
    }
    status = gd25q32e_write_enable();
    if (status != IMU_STATUS_OK)
    {
        return status;
    }
    cmd[0] = GD25Q32E_CMD_PAGE_PROGRAM;
    gd25q32e_addr_to_bytes(addr, &cmd[1]);
    if (!spi_bus_acquire(SPI_OWNER_FLASH))
    {
        return IMU_STATUS_BUS_ERROR;
    }
    status = imu_bsp_flash_cs_low();
    if (status == IMU_STATUS_OK)
    {
        status = imu_bsp_spi_transfer(cmd, NULL, sizeof(cmd));
    }
    if (status == IMU_STATUS_OK)
    {
        status = gd25q32e_transfer_data(buf, NULL, len);
    }
    (void)imu_bsp_flash_cs_high();
    spi_bus_release(SPI_OWNER_FLASH);
    return status;
}

imu_status_t gd25q32e_is_busy(bool *busy)
{
    imu_status_t status;
    uint8_t sr1 = 0U;

    if (busy == NULL)
    {
        return IMU_STATUS_INVALID_PARAM;
    }

    status = gd25q32e_read_status_reg(GD25Q32E_CMD_RDSR1, &sr1);
    if (status != IMU_STATUS_OK)
    {
        return status;
    }

    if (sr1 == 0xFFU)
    {
        *busy = false;
        DBG_DIRECT("[FLASH_BUSY] invalid_sr1=0xFF ambiguous_write=1");
        return IMU_STATUS_NOT_READY;
    }

    *busy = ((sr1 & GD25Q32E_SR1_WIP) != 0U);
    return IMU_STATUS_OK;
}

imu_status_t gd25q32e_wait_while_busy(uint32_t timeout_ms)
{
    const uint64_t start_us = imu_bsp_local_timestamp_us();
    const uint64_t timeout_us = ((uint64_t)timeout_ms) * 1000ULL;
    bool busy = false;
    imu_status_t status;

    do
    {
        status = gd25q32e_is_busy(&busy);
        if (status != IMU_STATUS_OK)
        {
            return status;
        }
        if (!busy)
        {
            return IMU_STATUS_OK;
        }
        imu_bsp_delay_us(GD25Q32E_READY_POLL_DELAY_US);
    } while ((imu_bsp_local_timestamp_us() - start_us) < timeout_us);

    return IMU_STATUS_TIMEOUT;
}
