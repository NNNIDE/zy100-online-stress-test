#ifndef MMC5603_I2C_H
#define MMC5603_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "../common/mag_common.h"

mag_status_t mag_i2c_bus_init(void);
void mag_i2c_set_slave_addr(uint8_t slave_addr_7bit);

mag_status_t mag_i2c_write_reg(uint8_t addr, uint8_t value);
mag_status_t mag_i2c_read_reg(uint8_t addr, uint8_t *value);
mag_status_t mag_i2c_read_regs(uint8_t addr, uint8_t *buf, uint16_t len);
mag_status_t mag_i2c_write_regs(uint8_t addr, const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* MMC5603_I2C_H */
