#include "drv_internal_flash.h"

#include <stdbool.h>
#include <stddef.h>

#include "flash_device.h"
#include "flash_map.h"
#include "trace.h"

#define DRV_INTERNAL_FLASH_SECTOR_BYTES    ((uint32_t)FMC_SEC_SECTION_LEN)

static bool drv_internal_flash_is_aligned(uint32_t value, uint32_t alignment)
{
    return (alignment != 0U) && ((value % alignment) == 0U);
}

static uint32_t drv_internal_flash_normalize_addr(uint32_t addr)
{
    return addr & ~((uint32_t)FLASH_OFFSET_TO_NO_CACHE);
}

static drv_internal_flash_status_t drv_internal_flash_resolve_region(
    drv_internal_flash_region_t *region)
{
    uint32_t base_addr;
    uint32_t normalized_base_addr;
    uint32_t size_bytes;

    if (region == NULL)
    {
        return DRV_INTERNAL_FLASH_STATUS_INVALID_PARAM;
    }

    base_addr = flash_get_bank_addr(FLASH_BKP_DATA1);
    normalized_base_addr = drv_internal_flash_normalize_addr(base_addr);
    size_bytes = flash_get_bank_size(FLASH_BKP_DATA1);
    if ((base_addr == 0U) || (size_bytes == 0U))
    {
        return DRV_INTERNAL_FLASH_STATUS_NO_REGION;
    }

    if ((normalized_base_addr != USER_DATA1_ADDR) ||
        (size_bytes != USER_DATA1_SIZE) ||
        !drv_internal_flash_is_aligned(normalized_base_addr, DRV_INTERNAL_FLASH_SECTOR_BYTES) ||
        !drv_internal_flash_is_aligned(size_bytes, DRV_INTERNAL_FLASH_SECTOR_BYTES))
    {
        DBG_DIRECT("[INTERNAL_FLASH] layout_mismatch runtime=0x%08lX norm=0x%08lX size=0x%08lX expect=0x%08lX/0x%08lX sector=%lu",
                   (unsigned long)base_addr,
                   (unsigned long)normalized_base_addr,
                   (unsigned long)size_bytes,
                   (unsigned long)USER_DATA1_ADDR,
                   (unsigned long)USER_DATA1_SIZE,
                   (unsigned long)DRV_INTERNAL_FLASH_SECTOR_BYTES);
        return DRV_INTERNAL_FLASH_STATUS_LAYOUT_MISMATCH;
    }

    region->base_addr = normalized_base_addr;
    region->size_bytes = size_bytes;
    region->sector_bytes = DRV_INTERNAL_FLASH_SECTOR_BYTES;
    return DRV_INTERNAL_FLASH_STATUS_OK;
}

static drv_internal_flash_status_t drv_internal_flash_check_range(
    uint32_t offset,
    uint32_t length,
    drv_internal_flash_region_t *region)
{
    drv_internal_flash_status_t status;

    if (length == 0U)
    {
        return DRV_INTERNAL_FLASH_STATUS_INVALID_PARAM;
    }

    status = drv_internal_flash_resolve_region(region);
    if (status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return status;
    }

    if ((offset > region->size_bytes) ||
        (length > (region->size_bytes - offset)))
    {
        return DRV_INTERNAL_FLASH_STATUS_OUT_OF_RANGE;
    }

    return DRV_INTERNAL_FLASH_STATUS_OK;
}

drv_internal_flash_status_t drv_internal_flash_get_region(drv_internal_flash_region_t *region)
{
    return drv_internal_flash_resolve_region(region);
}

static drv_internal_flash_status_t drv_internal_flash_erase_sector_impl(uint32_t offset)
{
    drv_internal_flash_region_t region;
    drv_internal_flash_status_t status;

    if (!drv_internal_flash_is_aligned(offset, DRV_INTERNAL_FLASH_SECTOR_BYTES))
    {
        return DRV_INTERNAL_FLASH_STATUS_INVALID_PARAM;
    }

    status = drv_internal_flash_check_range(offset, DRV_INTERNAL_FLASH_SECTOR_BYTES, &region);
    if (status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return status;
    }

    if (!flash_erase_locked(FLASH_ERASE_SECTOR, region.base_addr + offset))
    {
        return DRV_INTERNAL_FLASH_STATUS_ERASE_FAILED;
    }

    return DRV_INTERNAL_FLASH_STATUS_OK;
}

drv_internal_flash_status_t drv_internal_flash_erase_sector(uint32_t offset)
{
    return drv_internal_flash_erase_sector_impl(offset);
}

static drv_internal_flash_status_t drv_internal_flash_erase_range_impl(uint32_t offset,
                                                                       uint32_t length)
{
    drv_internal_flash_region_t region;
    drv_internal_flash_status_t status;
    uint32_t erase_offset;

    if (!drv_internal_flash_is_aligned(offset, DRV_INTERNAL_FLASH_SECTOR_BYTES) ||
        !drv_internal_flash_is_aligned(length, DRV_INTERNAL_FLASH_SECTOR_BYTES))
    {
        return DRV_INTERNAL_FLASH_STATUS_INVALID_PARAM;
    }

    status = drv_internal_flash_check_range(offset, length, &region);
    if (status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return status;
    }

    for (erase_offset = offset;
         erase_offset < (offset + length);
         erase_offset += DRV_INTERNAL_FLASH_SECTOR_BYTES)
    {
        if (!flash_erase_locked(FLASH_ERASE_SECTOR, region.base_addr + erase_offset))
        {
            return DRV_INTERNAL_FLASH_STATUS_ERASE_FAILED;
        }
    }

    return DRV_INTERNAL_FLASH_STATUS_OK;
}

drv_internal_flash_status_t drv_internal_flash_erase_range(uint32_t offset, uint32_t length)
{
    return drv_internal_flash_erase_range_impl(offset, length);
}

static drv_internal_flash_status_t drv_internal_flash_erase_all_impl(void)
{
    drv_internal_flash_region_t region;
    drv_internal_flash_status_t status;

    status = drv_internal_flash_resolve_region(&region);
    if (status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return status;
    }

    return drv_internal_flash_erase_range_impl(0U, region.size_bytes);
}

drv_internal_flash_status_t drv_internal_flash_erase_all(void)
{
    return drv_internal_flash_erase_all_impl();
}

static drv_internal_flash_status_t drv_internal_flash_write_impl(uint32_t offset,
                                                                 const uint8_t *data,
                                                                 uint32_t length)
{
    drv_internal_flash_region_t region;
    drv_internal_flash_status_t status;

    if (data == NULL)
    {
        return DRV_INTERNAL_FLASH_STATUS_INVALID_PARAM;
    }

    status = drv_internal_flash_check_range(offset, length, &region);
    if (status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return status;
    }

    if (!flash_write_locked(region.base_addr + offset, length, (uint8_t *)data))
    {
        return DRV_INTERNAL_FLASH_STATUS_WRITE_FAILED;
    }

    return DRV_INTERNAL_FLASH_STATUS_OK;
}

drv_internal_flash_status_t drv_internal_flash_write(uint32_t offset,
                                                     const uint8_t *data,
                                                     uint32_t length)
{
    return drv_internal_flash_write_impl(offset, data, length);
}

static drv_internal_flash_status_t drv_internal_flash_read_impl(uint32_t offset,
                                                                uint8_t *data,
                                                                uint32_t length)
{
    drv_internal_flash_region_t region;
    drv_internal_flash_status_t status;

    if (data == NULL)
    {
        return DRV_INTERNAL_FLASH_STATUS_INVALID_PARAM;
    }

    status = drv_internal_flash_check_range(offset, length, &region);
    if (status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return status;
    }

    if (!flash_read_locked(region.base_addr + offset, length, data))
    {
        return DRV_INTERNAL_FLASH_STATUS_READ_FAILED;
    }

    return DRV_INTERNAL_FLASH_STATUS_OK;
}

drv_internal_flash_status_t drv_internal_flash_read(uint32_t offset,
                                                    uint8_t *data,
                                                    uint32_t length)
{
    return drv_internal_flash_read_impl(offset, data, length);
}

const char *drv_internal_flash_status_name(drv_internal_flash_status_t status)
{
    switch (status)
    {
    case DRV_INTERNAL_FLASH_STATUS_OK:
        return "OK";
    case DRV_INTERNAL_FLASH_STATUS_INVALID_PARAM:
        return "INVALID_PARAM";
    case DRV_INTERNAL_FLASH_STATUS_NO_REGION:
        return "NO_REGION";
    case DRV_INTERNAL_FLASH_STATUS_LAYOUT_MISMATCH:
        return "LAYOUT_MISMATCH";
    case DRV_INTERNAL_FLASH_STATUS_OUT_OF_RANGE:
        return "OUT_OF_RANGE";
    case DRV_INTERNAL_FLASH_STATUS_ERASE_FAILED:
        return "ERASE_FAILED";
    case DRV_INTERNAL_FLASH_STATUS_WRITE_FAILED:
        return "WRITE_FAILED";
    case DRV_INTERNAL_FLASH_STATUS_READ_FAILED:
        return "READ_FAILED";
    default:
        return "UNKNOWN";
    }
}
