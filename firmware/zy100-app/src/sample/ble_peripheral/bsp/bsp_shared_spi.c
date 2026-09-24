#include "bsp_shared_spi.h"

#include "os_sync.h"
#include "os_task.h"
#include <stddef.h>
#include "trace.h"
#include "rtl876x_spi.h"

#include "imu_bsp.h"
#include "imu_board_pinmap.h"

static volatile bsp_shared_spi_owner_t s_shared_spi_owner =
    BSP_SHARED_SPI_OWNER_NONE;

#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE
static bsp_shared_spi_prepare_cb_t s_shared_spi_prepare_cb;
static bool s_shared_spi_preparing;
static void *s_shared_spi_prepare_task;
static uint32_t s_shared_spi_imu_prepare_busy_count;
#endif

uint32_t bsp_shared_spi_imu_prepare_busy_count(void)
{
#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE
    uint32_t lock_state = os_lock();
    uint32_t count = s_shared_spi_imu_prepare_busy_count;
    os_unlock(lock_state);
    return count;
#else
    return 0U;
#endif
}

void bsp_shared_spi_set_prepare_callback(bsp_shared_spi_prepare_cb_t callback)
{
#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE
    s_shared_spi_prepare_cb = callback;
#else
    (void)callback;
#endif
}

static bool bsp_shared_spi_config_owner(bsp_shared_spi_owner_t owner)
{
    imu_status_t status;

    /* Called with power and exclusive bus ownership, before any CS assertion. */
    status = imu_bsp_init();
    if (status != IMU_STATUS_OK) return false;
    imu_bsp_cs_high();
    (void)imu_bsp_flash_cs_high();

    switch (owner)
    {
    case BSP_SHARED_SPI_OWNER_IMU:
        status = imu_bsp_spi_set_mode(IMU_SPI_CPOL, IMU_SPI_CPHA);
        break;

    case BSP_SHARED_SPI_OWNER_FLASH:
        status = imu_bsp_spi_set_mode(SPI_CPOL_High, SPI_CPHA_2Edge);
        break;

    default:
        status = IMU_STATUS_INVALID_PARAM;
        break;
    }

    if (status != IMU_STATUS_OK)
    {
        DBG_DIRECT("[ERR][FLASH] spi_bus owner=%u", (uint32_t)owner);
        imu_bsp_cs_high();
        (void)imu_bsp_flash_cs_high();
        return false;
    }

    return true;
}

bsp_shared_spi_acquire_result_t bsp_shared_spi_acquire_ex(
    bsp_shared_spi_owner_t owner, bsp_shared_spi_owner_t *blocking_owner)
{
    uint32_t lock_state;
#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE
    bool prepare_owner = false;
    bool prepared = true;
    void *task = NULL;
#endif
    if (blocking_owner != NULL) *blocking_owner = BSP_SHARED_SPI_OWNER_NONE;
    if ((owner == BSP_SHARED_SPI_OWNER_NONE) || (owner > BSP_SHARED_SPI_OWNER_FLASH))
        return BSP_SHARED_SPI_ACQUIRE_PREPARE_FAILED;
    if (imu_bsp_power_access_begin() != IMU_STATUS_OK) return BSP_SHARED_SPI_ACQUIRE_PREPARE_FAILED;
    lock_state = os_lock();
#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE
    if (s_shared_spi_preparing ||
        ((owner == BSP_SHARED_SPI_OWNER_FLASH) && (s_shared_spi_prepare_cb != NULL)))
    {
        if (__get_IPSR() == 0U) (void)os_task_handle_get(&task);
        if (task == NULL)
        {
            os_unlock(lock_state);
            imu_bsp_power_access_end();
            return BSP_SHARED_SPI_ACQUIRE_PREPARE_FAILED;
        }
        if (s_shared_spi_preparing && (task != s_shared_spi_prepare_task))
        {
            /* Preparation is reserved before physical SPI ownership. Capture
             * this fact now; a later owner query can already be NONE again. */
            if (blocking_owner != NULL) *blocking_owner = BSP_SHARED_SPI_OWNER_FLASH;
            if (owner == BSP_SHARED_SPI_OWNER_IMU) s_shared_spi_imu_prepare_busy_count++;
            os_unlock(lock_state);
            imu_bsp_power_access_end();
            return BSP_SHARED_SPI_ACQUIRE_PREPARE_BUSY;
        }
    }
#endif
    if (s_shared_spi_owner != BSP_SHARED_SPI_OWNER_NONE)
    {
        if (blocking_owner != NULL) *blocking_owner = s_shared_spi_owner;
        os_unlock(lock_state);
        imu_bsp_power_access_end();
        return BSP_SHARED_SPI_ACQUIRE_BUSY;
    }
#if ZY100_WOM_SENSOR_WEAK_PULL_TEST_ENABLE
    if ((owner == BSP_SHARED_SPI_OWNER_FLASH) && (s_shared_spi_prepare_cb != NULL))
    {
        if (!s_shared_spi_preparing)
        {
            prepare_owner = true;
            s_shared_spi_preparing = true;
            s_shared_spi_prepare_task = task;
        }
        os_unlock(lock_state);
        prepared = s_shared_spi_prepare_cb(owner);
        lock_state = os_lock();
        if (prepare_owner)
        {
            s_shared_spi_preparing = false;
            s_shared_spi_prepare_task = NULL;
        }
        if (!prepared || (s_shared_spi_owner != BSP_SHARED_SPI_OWNER_NONE))
        {
            bsp_shared_spi_acquire_result_t result =
                prepared ? BSP_SHARED_SPI_ACQUIRE_BUSY :
                           BSP_SHARED_SPI_ACQUIRE_PREPARE_FAILED;
            if (prepared && (blocking_owner != NULL))
                *blocking_owner = s_shared_spi_owner;
            os_unlock(lock_state);
            imu_bsp_power_access_end();
            return result;
        }
    }
#endif
    s_shared_spi_owner = owner;
    os_unlock(lock_state);
    if (!bsp_shared_spi_config_owner(owner))
    {
        lock_state = os_lock();
        if (s_shared_spi_owner == owner) s_shared_spi_owner = BSP_SHARED_SPI_OWNER_NONE;
        os_unlock(lock_state);
        imu_bsp_power_access_end();
        return BSP_SHARED_SPI_ACQUIRE_PREPARE_FAILED;
    }
    return BSP_SHARED_SPI_ACQUIRE_OK;
}

bool bsp_shared_spi_acquire(bsp_shared_spi_owner_t owner)
{
    return bsp_shared_spi_acquire_ex(owner, NULL) == BSP_SHARED_SPI_ACQUIRE_OK;
}

void bsp_shared_spi_release(bsp_shared_spi_owner_t owner)
{
    uint32_t lock_state;

    imu_bsp_cs_high();
    (void)imu_bsp_flash_cs_high();

    lock_state = os_lock();
    if (s_shared_spi_owner == owner)
    {
        s_shared_spi_owner = BSP_SHARED_SPI_OWNER_NONE;
        imu_bsp_power_access_end();
    }
    os_unlock(lock_state);
}

bsp_shared_spi_owner_t bsp_shared_spi_current_owner(void)
{
    bsp_shared_spi_owner_t owner;
    uint32_t lock_state = os_lock();

    owner = s_shared_spi_owner;
    os_unlock(lock_state);
    return owner;
}

void bsp_shared_spi_force_idle(void)
{
    uint32_t lock_state;

    imu_bsp_cs_high();
    (void)imu_bsp_flash_cs_high();

    lock_state = os_lock();
    if (s_shared_spi_owner != BSP_SHARED_SPI_OWNER_NONE)
        imu_bsp_power_access_end();
    s_shared_spi_owner = BSP_SHARED_SPI_OWNER_NONE;
    os_unlock(lock_state);
}
