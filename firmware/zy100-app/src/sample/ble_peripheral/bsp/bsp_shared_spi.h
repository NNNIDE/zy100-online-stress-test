#ifndef BSP_SHARED_SPI_H
#define BSP_SHARED_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BSP_SHARED_SPI_OWNER_NONE = 0U,
    BSP_SHARED_SPI_OWNER_IMU = 1U,
    BSP_SHARED_SPI_OWNER_FLASH = 2U,
} bsp_shared_spi_owner_t;

/* Optional board/application access preflight; called before CS ownership. */
typedef bool (*bsp_shared_spi_prepare_cb_t)(bsp_shared_spi_owner_t owner);
void bsp_shared_spi_set_prepare_callback(bsp_shared_spi_prepare_cb_t callback);
typedef enum
{
    BSP_SHARED_SPI_ACQUIRE_OK = 0,
    BSP_SHARED_SPI_ACQUIRE_BUSY,
    BSP_SHARED_SPI_ACQUIRE_PREPARE_FAILED,
    /* Another task reserves Flash preparation; no transfer was started. */
    BSP_SHARED_SPI_ACQUIRE_PREPARE_BUSY,
} bsp_shared_spi_acquire_result_t;

/* BUSY snapshots the physical owner; PREPARE_BUSY snapshots the reserving
 * owner without changing the physical owner. Neither starts a transaction.
 * PREPARE_FAILED must never be reclassified using a later owner query. */
bsp_shared_spi_acquire_result_t bsp_shared_spi_acquire_ex(
    bsp_shared_spi_owner_t owner, bsp_shared_spi_owner_t *blocking_owner);
bool bsp_shared_spi_acquire(bsp_shared_spi_owner_t owner);
void bsp_shared_spi_release(bsp_shared_spi_owner_t owner);
bsp_shared_spi_owner_t bsp_shared_spi_current_owner(void);
/* Monotonic modulo-uint32 counter; callers use session start/end differences. */
uint32_t bsp_shared_spi_imu_prepare_busy_count(void);
void bsp_shared_spi_force_idle(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SHARED_SPI_H */
