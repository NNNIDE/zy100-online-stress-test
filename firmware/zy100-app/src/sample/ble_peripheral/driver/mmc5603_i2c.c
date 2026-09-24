#include "mmc5603_i2c.h"

#include <stddef.h>

#include "../bsp/mag_bsp.h"
#include "mmc5603_reg.h"

#ifndef MAG_I2C_TRACE_ENABLE
#define MAG_I2C_TRACE_ENABLE 0
#endif

#define MAG_I2C_MAX_WRITE_LEN        32U

static uint8_t s_mag_i2c_slave_addr = MMC5603_I2C_ADDR_7BIT;

mag_status_t mag_i2c_bus_init(void)
{
    return mag_bsp_i2c_init(s_mag_i2c_slave_addr);
}

void mag_i2c_set_slave_addr(uint8_t slave_addr_7bit)
{
    s_mag_i2c_slave_addr = slave_addr_7bit;
}

mag_status_t mag_i2c_write_reg(uint8_t addr, uint8_t value)
{
    return mag_i2c_write_regs(addr, &value, 1U);
}

mag_status_t mag_i2c_read_reg(uint8_t addr, uint8_t *value)
{
    return mag_i2c_read_regs(addr, value, 1U);
}

mag_status_t mag_i2c_read_regs(uint8_t addr, uint8_t *buf, uint16_t len)
{
    mag_status_t status;
    uint8_t reg_addr;

    if ((buf == NULL) || (len == 0U))
    {
        return MAG_STATUS_INVALID_PARAM;
    }

    /* BSP owns init, idle check and transfer as one power/bus transaction. */
    reg_addr = addr;
    status = mag_bsp_i2c_repeat_read(s_mag_i2c_slave_addr, &reg_addr, 1U, buf, len);

#if MAG_I2C_TRACE_ENABLE
    MAG_LOG_INFO("I2C R addr=0x%02x reg=0x%02x len=%u status=%d",
                 s_mag_i2c_slave_addr, addr, len, status);
#endif
    if (status != MAG_STATUS_OK)
    {
        MAG_LOG_ERROR("I2C read fail addr=0x%02x reg=0x%02x len=%u status=%d",
                      s_mag_i2c_slave_addr, addr, len, status);
    }

    return status;
}

mag_status_t mag_i2c_write_regs(uint8_t addr, const uint8_t *buf, uint16_t len)
{
    mag_status_t status;
    uint8_t payload[1U + MAG_I2C_MAX_WRITE_LEN];
    uint16_t i;

    if ((buf == NULL) || (len == 0U) || (len > MAG_I2C_MAX_WRITE_LEN))
    {
        return MAG_STATUS_INVALID_PARAM;
    }

    payload[0] = addr;
    for (i = 0U; i < len; i++)
    {
        payload[i + 1U] = buf[i];
    }

    status = mag_bsp_i2c_master_write(s_mag_i2c_slave_addr, payload, (uint16_t)(len + 1U));

#if MAG_I2C_TRACE_ENABLE
    MAG_LOG_INFO("I2C W addr=0x%02x reg=0x%02x len=%u status=%d",
                 s_mag_i2c_slave_addr, addr, len, status);
#endif
    if (status != MAG_STATUS_OK)
    {
        MAG_LOG_ERROR("I2C write fail addr=0x%02x reg=0x%02x len=%u status=%d",
                      s_mag_i2c_slave_addr, addr, len, status);
    }

    return status;
}
