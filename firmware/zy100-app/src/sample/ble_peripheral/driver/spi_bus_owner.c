#include "spi_bus_owner.h"

bsp_shared_spi_acquire_result_t spi_bus_acquire_ex(
    spi_bus_owner_t owner, spi_bus_owner_t *blocking_owner)
{
    return bsp_shared_spi_acquire_ex(owner, blocking_owner);
}

bool spi_bus_acquire(spi_bus_owner_t owner)
{
    return bsp_shared_spi_acquire(owner);
}

void spi_bus_release(spi_bus_owner_t owner)
{
    bsp_shared_spi_release(owner);
}

spi_bus_owner_t spi_bus_current_owner(void)
{
    return bsp_shared_spi_current_owner();
}

void spi_bus_force_idle(void)
{
    bsp_shared_spi_force_idle();
}
