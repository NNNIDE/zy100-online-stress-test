#include "mag_bsp.h"

#include <stddef.h>

#include "mag_board_pinmap.h"
#include "imu_bsp.h"
#include "../app_flags.h"

#include "rtl876x_gpio.h"
#include "rtl876x_i2c.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"
#include "os_sync.h"

static bool s_mag_i2c_inited = false;
static bool s_mag_i2c_dlps_recovery_required = false;
static bool s_mag_i2c_clock_enabled = false;
static bool s_mag_i2c_suspended = false;
static bool s_mag_i2c_access_active = false;

/* Existing MMC5603 bus-idle budget, now inside the transfer ownership. */
#define MAG_I2C_BUS_IDLE_TIMEOUT_US 2000U
#define MAG_I2C_BUS_IDLE_POLL_US      50U

static bool mag_bsp_i2c_claim(void)
{
    uint32_t key = os_lock();
    if (s_mag_i2c_access_active)
    {
        os_unlock(key);
        return false;
    }
    s_mag_i2c_access_active = true;
    os_unlock(key);
    return true;
}

static void mag_bsp_i2c_release(void)
{
    uint32_t key = os_lock();
    s_mag_i2c_access_active = false;
    os_unlock(key);
}

static mag_status_t mag_bsp_map_i2c_status(I2C_Status status)
{
    switch (status)
    {
    case I2C_Success:
        return MAG_STATUS_OK;

    case I2C_ABRT_TXDATA_NOACK:
    case I2C_ABRT_10ADDR2_NOACK:
    case I2C_ABRT_10ADDR1_NOACK:
    case I2C_ABRT_7B_ADDR_NOACK:
        return MAG_STATUS_NACK;

    case I2C_ERR_TIMEOUT:
        return MAG_STATUS_TIMEOUT;

    case I2C_ARB_LOST:
    case I2C_ABRT_MASTER_DIS:
    default:
        return MAG_STATUS_BUS_ERROR;
    }
}

static void mag_bsp_config_i2c_pins(void)
{
    Pad_Config(MAG_I2C_SCL_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE, PAD_OUT_HIGH);
    Pad_Config(MAG_I2C_SDA_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE, PAD_OUT_HIGH);

    Pinmux_Deinit(MAG_I2C_SCL_PIN);
    Pinmux_Deinit(MAG_I2C_SDA_PIN);
    Pinmux_Config(MAG_I2C_SCL_PIN, MAG_I2C_SCL_FUNC);
    Pinmux_Config(MAG_I2C_SDA_PIN, MAG_I2C_SDA_FUNC);
}

/* Caller holds both I2C ownership and the sensor-power access lease. */
static mag_status_t mag_bsp_i2c_init_locked(uint16_t slave_addr_7bit)
{
    I2C_InitTypeDef init;

    if (slave_addr_7bit > 0x7FU)
    {
        return MAG_STATUS_INVALID_PARAM;
    }

    if (s_mag_i2c_inited)
    {
        I2C_SetSlaveAddress(MAG_I2C_BUS_ID, slave_addr_7bit);
        return MAG_STATUS_OK;
    }

    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    RCC_PeriphClockCmd(MAG_I2C_CLOCK_ID, MAG_I2C_CLOCK_MASK, ENABLE);
    s_mag_i2c_clock_enabled = true;

    if (s_mag_i2c_dlps_recovery_required)
    {
        I2C_Cmd(MAG_I2C_BUS_ID, DISABLE);
    }

    mag_bsp_config_i2c_pins();

    I2C_StructInit(&init);
    init.I2C_ClockSpeed = MAG_I2C_SPEED_HZ;
    init.I2C_DeviveMode = I2C_DeviveMode_Master;
    init.I2C_AddressMode = I2C_AddressMode_7BIT;
    init.I2C_SlaveAddress = slave_addr_7bit;
    init.I2C_Ack = I2C_Ack_Enable;

    I2C_Init(MAG_I2C_BUS_ID, &init);
    I2C_Cmd(MAG_I2C_BUS_ID, ENABLE);
    s_mag_i2c_inited = true;
    s_mag_i2c_dlps_recovery_required = false;
    s_mag_i2c_suspended = false;
    return MAG_STATUS_OK;
}

static mag_status_t mag_bsp_i2c_access_begin(uint16_t slave_addr, bool force_reinit)
{
    mag_status_t status;
    if (slave_addr > 0x7FU) return MAG_STATUS_INVALID_PARAM;
    if (!mag_bsp_i2c_claim()) return MAG_STATUS_BUS_BUSY;
    if (imu_bsp_power_access_begin() != IMU_STATUS_OK)
    {
        mag_bsp_i2c_release();
        return MAG_STATUS_NOT_READY;
    }
    if (force_reinit)
    {
        s_mag_i2c_inited = false;
        s_mag_i2c_dlps_recovery_required = true;
    }
    status = mag_bsp_i2c_init_locked(slave_addr);
    if (status != MAG_STATUS_OK)
    {
        imu_bsp_power_access_end();
        mag_bsp_i2c_release();
    }
    return status;
}

static void mag_bsp_i2c_access_end(void)
{
    imu_bsp_power_access_end();
    mag_bsp_i2c_release();
}

mag_status_t mag_bsp_i2c_init(uint16_t slave_addr_7bit)
{
    mag_status_t status = mag_bsp_i2c_access_begin(slave_addr_7bit, false);
    if (status == MAG_STATUS_OK) mag_bsp_i2c_access_end();
    return status;
}

mag_status_t mag_bsp_i2c_force_reinit(uint8_t slave_addr)
{
    mag_status_t status = mag_bsp_i2c_access_begin(slave_addr, true);
    if (status == MAG_STATUS_OK) mag_bsp_i2c_access_end();
    return status;
}

void mag_bsp_i2c_mark_lost_after_dlps_prepare(void)
{
    uint32_t key = os_lock();
    s_mag_i2c_inited = false;
    s_mag_i2c_dlps_recovery_required = true;
    os_unlock(key);
}

mag_status_t mag_bsp_i2c_suspend(void)
{
    if (!mag_bsp_i2c_claim()) return MAG_STATUS_BUS_BUSY;
    if (s_mag_i2c_suspended)
    {
        mag_bsp_i2c_release();
        return MAG_STATUS_OK;
    }
    if (s_mag_i2c_clock_enabled)
    {
        if (I2C_GetFlagState(MAG_I2C_BUS_ID, I2C_FLAG_ACTIVITY) == SET)
        {
            mag_bsp_i2c_release();
            return MAG_STATUS_BUS_BUSY;
        }
        I2C_Cmd(MAG_I2C_BUS_ID, DISABLE);
    }
    Pinmux_Deinit(MAG_I2C_SCL_PIN);
    Pinmux_Deinit(MAG_I2C_SDA_PIN);
    Pad_Config(MAG_I2C_SCL_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE,
               PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(MAG_I2C_SDA_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE,
               PAD_OUT_DISABLE, PAD_OUT_LOW);
    RCC_PeriphClockCmd(MAG_I2C_CLOCK_ID, MAG_I2C_CLOCK_MASK, DISABLE);
    s_mag_i2c_clock_enabled = false;
    s_mag_i2c_inited = false;
    s_mag_i2c_dlps_recovery_required = true;
    s_mag_i2c_suspended = true;
    mag_bsp_i2c_release();
    return MAG_STATUS_OK;
}

bool mag_bsp_i2c_is_suspended(void)
{
    uint32_t key = os_lock();
    bool suspended = s_mag_i2c_suspended && !s_mag_i2c_access_active;
    os_unlock(key);
    return suspended;
}

static mag_status_t mag_bsp_i2c_wait_idle_locked(void)
{
    const uint64_t start = mag_bsp_local_timestamp_us();
    while (I2C_GetFlagState(MAG_I2C_BUS_ID, I2C_FLAG_ACTIVITY) == SET)
    {
        if ((mag_bsp_local_timestamp_us() - start) >= MAG_I2C_BUS_IDLE_TIMEOUT_US)
            return MAG_STATUS_BUS_BUSY;
        mag_bsp_delay_us(MAG_I2C_BUS_IDLE_POLL_US);
    }
    return MAG_STATUS_OK;
}

mag_status_t mag_bsp_i2c_master_write(uint16_t slave_addr_7bit, uint8_t *buf, uint16_t len)
{
    I2C_Status hal_status;
    mag_status_t status;

    if ((buf == NULL) || (len == 0U) || (slave_addr_7bit > 0x7FU))
    {
        return MAG_STATUS_INVALID_PARAM;
    }

    status = mag_bsp_i2c_access_begin(slave_addr_7bit, false);
    if (status != MAG_STATUS_OK) return status;
    status = mag_bsp_i2c_wait_idle_locked();
    if (status != MAG_STATUS_OK)
    {
        mag_bsp_i2c_access_end();
        return status;
    }
    hal_status = I2C_MasterWrite(MAG_I2C_BUS_ID, buf, len);
    mag_bsp_i2c_access_end();
    return mag_bsp_map_i2c_status(hal_status);
}

mag_status_t mag_bsp_i2c_repeat_read(uint16_t slave_addr_7bit,
                                     uint8_t *tx_buf,
                                     uint16_t tx_len,
                                     uint8_t *rx_buf,
                                     uint16_t rx_len)
{
    I2C_Status hal_status;
    mag_status_t status;

    if ((tx_buf == NULL) || (tx_len == 0U) || (rx_buf == NULL) || (rx_len == 0U) || (slave_addr_7bit > 0x7FU))
    {
        return MAG_STATUS_INVALID_PARAM;
    }

    status = mag_bsp_i2c_access_begin(slave_addr_7bit, false);
    if (status != MAG_STATUS_OK) return status;
    status = mag_bsp_i2c_wait_idle_locked();
    if (status != MAG_STATUS_OK)
    {
        mag_bsp_i2c_access_end();
        return status;
    }
    hal_status = I2C_RepeatRead(MAG_I2C_BUS_ID, tx_buf, tx_len, rx_buf, rx_len);
    mag_bsp_i2c_access_end();
    return mag_bsp_map_i2c_status(hal_status);
}

bool mag_bsp_i2c_is_bus_busy(void)
{
    bool busy;
    if (!mag_bsp_i2c_claim()) return true;
    busy = s_mag_i2c_clock_enabled &&
           (I2C_GetFlagState(MAG_I2C_BUS_ID, I2C_FLAG_ACTIVITY) == SET);
    mag_bsp_i2c_release();
    return busy;
}

void mag_bsp_delay_us(uint32_t us)
{
    imu_bsp_delay_us(us);
}

uint64_t mag_bsp_local_timestamp_us(void)
{
    /* Keep MMC5603 and ICM-53611 on the exact same local time source. */
    return imu_bsp_local_timestamp_us();
}
