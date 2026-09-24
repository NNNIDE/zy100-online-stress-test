#ifndef SPI_BUS_OWNER_H
#define SPI_BUS_OWNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "../bsp/bsp_shared_spi.h"

typedef bsp_shared_spi_owner_t spi_bus_owner_t;

#define SPI_OWNER_NONE  BSP_SHARED_SPI_OWNER_NONE
#define SPI_OWNER_IMU   BSP_SHARED_SPI_OWNER_IMU
#define SPI_OWNER_FLASH BSP_SHARED_SPI_OWNER_FLASH

bsp_shared_spi_acquire_result_t spi_bus_acquire_ex(
    spi_bus_owner_t owner, spi_bus_owner_t *blocking_owner);
bool spi_bus_acquire(spi_bus_owner_t owner);
void spi_bus_release(spi_bus_owner_t owner);
spi_bus_owner_t spi_bus_current_owner(void);
void spi_bus_force_idle(void);

#ifdef __cplusplus
}
#endif

#endif /* SPI_BUS_OWNER_H */
