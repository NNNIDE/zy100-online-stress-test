#ifndef MAG_BSP_H
#define MAG_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../common/mag_common.h"

mag_status_t mag_bsp_i2c_init(uint16_t slave_addr_7bit);
mag_status_t mag_bsp_i2c_force_reinit(uint8_t slave_addr);
void mag_bsp_i2c_mark_lost_after_dlps_prepare(void);
/* Task-context, non-blocking ownership: never interrupts an active transfer. */
mag_status_t mag_bsp_i2c_suspend(void);
bool mag_bsp_i2c_is_suspended(void);
mag_status_t mag_bsp_i2c_master_write(uint16_t slave_addr_7bit, uint8_t *buf, uint16_t len);
mag_status_t mag_bsp_i2c_repeat_read(uint16_t slave_addr_7bit,
                                     uint8_t *tx_buf,
                                     uint16_t tx_len,
                                     uint8_t *rx_buf,
                                     uint16_t rx_len);
bool mag_bsp_i2c_is_bus_busy(void);

void mag_bsp_delay_us(uint32_t us);
uint64_t mag_bsp_local_timestamp_us(void);

#ifdef __cplusplus
}
#endif

#endif /* MAG_BSP_H */
