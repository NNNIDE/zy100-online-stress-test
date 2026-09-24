#ifndef ICM53611_SPI_H
#define ICM53611_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "../common/imu_common.h"

imu_status_t imu_spi_bus_init(void);

/* Passive first-error evidence for the task that exclusively owns IMU setup.
 * No I/O, retry, or return-value policy depends on this diagnostic scope. */
#include "icm53611_io_diag.h"
void imu_spi_error_begin(void);
void imu_spi_error_checkpoint(void);
void imu_spi_error_end(imu_spi_error_t *error);
void imu_spi_error_record(uint8_t op, uint8_t space, uint8_t reg,
                          uint8_t value, bool valid, imu_status_t status);

/* Task-serialized clock lease. Nested leases borrow an already-held IDLE bit.
 * end restores only our IDLE request, never another mode field. */
typedef struct
{
    uint8_t saved_pwr_mgmt0;
    bool idle_modified;
    uint8_t release_status; /* 0xff: no release attempted */
} imu_spi_clock_t;
imu_status_t imu_spi_clock_begin(imu_spi_clock_t *clock);
imu_status_t imu_spi_clock_end(imu_spi_clock_t *clock);

/* Opt-in task interfaces preserve pre-transfer Flash contention distinctly.
 * Existing APIs retain BUS_ERROR and existing logging for compatibility. */
imu_status_t imu_spi_write_reg_ex(uint8_t addr, uint8_t value);
imu_status_t imu_spi_read_reg_ex(uint8_t addr, uint8_t *value);
imu_status_t imu_spi_read_regs_ex(uint8_t addr, uint8_t *buf, uint16_t len);

imu_status_t imu_spi_write_reg(uint8_t addr, uint8_t value);
imu_status_t imu_spi_read_reg(uint8_t addr, uint8_t *value);
imu_status_t imu_spi_read_regs(uint8_t addr, uint8_t *buf, uint16_t len);
imu_status_t imu_spi_read_regs_isr_fast(uint8_t addr, uint8_t *buf, uint16_t len);
imu_status_t imu_spi_read_regs_isr_fast_locked(uint8_t addr,
                                               uint8_t *buf,
                                               uint16_t len);
imu_status_t imu_spi_write_regs(uint8_t addr, const uint8_t *buf, uint16_t len);

imu_status_t imu_spi_mreg_write(uint8_t block, uint8_t maddr, uint8_t value);
imu_status_t imu_spi_mreg_write_with_maddr_delay(uint8_t block, uint8_t maddr, uint8_t value);
imu_status_t imu_spi_mreg_read(uint8_t block, uint8_t maddr, uint8_t *value);
imu_status_t imu_spi_mreg_write_read_same_session(uint8_t block,
                                                  uint8_t maddr,
                                                  uint8_t value,
                                                  uint8_t *readback,
                                                  uint8_t *mclk_before,
                                                  uint8_t *mclk_after);

#ifdef __cplusplus
}
#endif

#endif /* ICM53611_SPI_H */
