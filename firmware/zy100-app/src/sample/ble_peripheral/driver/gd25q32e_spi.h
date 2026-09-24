#ifndef GD25Q32E_SPI_H
#define GD25Q32E_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../common/imu_common.h"

#define GD25Q32E_FLASH_SIZE_BYTES            0x400000UL
#define GD25Q32E_PAGE_BYTES                  256U
#define GD25Q32E_SECTOR_BYTES                4096U
#define GD25Q32E_BLOCK32_BYTES               (32UL * 1024UL)
#define GD25Q32E_PAGE_PROGRAM_TIMEOUT_MS     10U
#define GD25Q32E_SECTOR_ERASE_TIMEOUT_MS     5000U
#define GD25Q32E_BLOCK32_ERASE_TIMEOUT_MS    GD25Q32E_SECTOR_ERASE_TIMEOUT_MS
#define GD25Q32E_CHIP_ERASE_TIMEOUT_MS       70000U

typedef struct
{
    uint8_t manufacturer_id;
    uint8_t memory_type;
    uint8_t density;
} gd25q32e_jedec_id_t;

typedef struct
{
    uint8_t sr1;
    uint8_t sr2;
    uint8_t sr3;
} gd25q32e_status_regs_t;

imu_status_t gd25q32e_init(void);
bool gd25q32e_dpd_is_armed(void);
imu_status_t gd25q32e_enter_deep_power_down(bool verify);
imu_status_t gd25q32e_release_deep_power_down(void);
imu_status_t gd25q32e_resume_and_verify(gd25q32e_jedec_id_t *id,
                                        gd25q32e_status_regs_t *status_regs);
void gd25q32e_notify_power_lost(void);
imu_status_t gd25q32e_read_jedec_id(gd25q32e_jedec_id_t *id);
imu_status_t gd25q32e_read_jedec_id_attempt(gd25q32e_jedec_id_t *id, uint32_t attempt);
bool gd25q32e_jedec_is_4mbyte(const gd25q32e_jedec_id_t *id);
imu_status_t gd25q32e_read_status(gd25q32e_status_regs_t *status_regs);
bool gd25q32e_status_is_protected(const gd25q32e_status_regs_t *status_regs);
imu_status_t gd25q32e_write_enable(void);
imu_status_t gd25q32e_chip_erase(void);
imu_status_t gd25q32e_sector_erase_4k(uint32_t addr);
/* Issues the erase command and returns without waiting for WIP to clear. */
imu_status_t gd25q32e_block_erase_32k(uint32_t addr);
imu_status_t gd25q32e_read(uint32_t addr, uint8_t *buf, uint16_t len);
imu_status_t gd25q32e_read_fast(uint32_t addr, uint8_t *buf, uint16_t len);
imu_status_t gd25q32e_page_program(uint32_t addr, const uint8_t *buf, uint16_t len);
/* Programs a previously erased byte range within one page.  The driver reads
 * the target first and rejects non-0xFF bytes; policy/range ownership remains
 * the caller's responsibility. */
imu_status_t gd25q32e_page_program_partial_erased(uint32_t addr,
                                                  const uint8_t *buf,
                                                  uint16_t len);
imu_status_t gd25q32e_is_busy(bool *busy);
imu_status_t gd25q32e_wait_while_busy(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* GD25Q32E_SPI_H */
