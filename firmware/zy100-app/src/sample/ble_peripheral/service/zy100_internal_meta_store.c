#include "zy100_internal_meta_store.h"

#include <string.h>

#include "trace.h"

#include "../app_flags.h"
#include "../driver/drv_internal_flash.h"

#if ZY100_FINAL_EDGE_MODE_ENABLE

typedef char zy100_internal_meta_region_size_check[
    (ZY100_INTERNAL_META_STORE_REGION_BYTES ==
     ZY100_FINAL_EDGE_META_REGION_BYTES) ? 1 : -1];
typedef char zy100_internal_meta_page_size_check[
    (ZY100_INTERNAL_META_STORE_PAGE_BYTES == 256U) ? 1 : -1];
typedef char zy100_internal_meta_sector_size_check[
    (ZY100_INTERNAL_META_STORE_SECTOR_BYTES == 4096U) ? 1 : -1];

static bool zy100_internal_meta_range_valid(uint32_t offset, uint32_t len)
{
    if (len == 0U)
    {
        return false;
    }
    if (offset > ZY100_INTERNAL_META_STORE_REGION_BYTES)
    {
        return false;
    }
    return len <= (ZY100_INTERNAL_META_STORE_REGION_BYTES - offset);
}

static bool zy100_internal_meta_aligned(uint32_t value, uint32_t align)
{
    return (align != 0U) && ((value % align) == 0U);
}

bool zy100_internal_meta_store_ready(void)
{
    drv_internal_flash_region_t region;
    drv_internal_flash_status_t status;

    status = drv_internal_flash_get_region(&region);
    if (status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        DBG_DIRECT("[INTERNAL_META] not_ready status=%s",
                   drv_internal_flash_status_name(status));
        return false;
    }
    if ((region.sector_bytes != ZY100_INTERNAL_META_STORE_SECTOR_BYTES) ||
        (region.size_bytes < ZY100_INTERNAL_META_STORE_REGION_BYTES))
    {
        DBG_DIRECT("[INTERNAL_META] bad_region base=0x%08lX size=%lu sector=%lu",
                   (unsigned long)region.base_addr,
                   (unsigned long)region.size_bytes,
                   (unsigned long)region.sector_bytes);
        return false;
    }
    return true;
}

bool zy100_internal_meta_store_read(uint32_t offset,
                                    uint8_t *buf,
                                    uint32_t len)
{
    drv_internal_flash_status_t status;

    if ((buf == NULL) || !zy100_internal_meta_range_valid(offset, len))
    {
        return false;
    }
    status = drv_internal_flash_read(offset, buf, len);
    if (status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        DBG_DIRECT("[INTERNAL_META] read_fail off=0x%08lX len=%lu status=%s",
                   (unsigned long)offset,
                   (unsigned long)len,
                   drv_internal_flash_status_name(status));
        return false;
    }
    return true;
}

bool zy100_internal_meta_store_read_page(
    uint32_t offset,
    uint8_t page[ZY100_INTERNAL_META_STORE_PAGE_BYTES])
{
    if ((page == NULL) ||
        !zy100_internal_meta_aligned(offset,
                                     ZY100_INTERNAL_META_STORE_PAGE_BYTES))
    {
        return false;
    }
    return zy100_internal_meta_store_read(offset,
                                         page,
                                         ZY100_INTERNAL_META_STORE_PAGE_BYTES);
}

bool zy100_internal_meta_store_program_page_overlay(
    uint32_t offset,
    const uint8_t page[ZY100_INTERNAL_META_STORE_PAGE_BYTES])
{
    uint8_t old_page[ZY100_INTERNAL_META_STORE_PAGE_BYTES];
    uint8_t verify_page[ZY100_INTERNAL_META_STORE_PAGE_BYTES];
    drv_internal_flash_status_t status;
    uint32_t idx;

    if ((page == NULL) ||
        !zy100_internal_meta_aligned(offset,
                                     ZY100_INTERNAL_META_STORE_PAGE_BYTES) ||
        !zy100_internal_meta_range_valid(offset,
                                         ZY100_INTERNAL_META_STORE_PAGE_BYTES))
    {
        return false;
    }
    if (!zy100_internal_meta_store_read_page(offset, old_page))
    {
        return false;
    }
    for (idx = 0U; idx < ZY100_INTERNAL_META_STORE_PAGE_BYTES; idx++)
    {
        if ((old_page[idx] & page[idx]) != page[idx])
        {
            DBG_DIRECT("[INTERNAL_META] overlay_reject off=0x%08lX idx=%lu old=0x%02X new=0x%02X",
                       (unsigned long)offset,
                       (unsigned long)idx,
                       old_page[idx],
                       page[idx]);
            return false;
        }
    }

    status = drv_internal_flash_write(offset,
                                      page,
                                      ZY100_INTERNAL_META_STORE_PAGE_BYTES);
    if (status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        DBG_DIRECT("[INTERNAL_META] write_fail off=0x%08lX status=%s",
                   (unsigned long)offset,
                   drv_internal_flash_status_name(status));
        return false;
    }
    if (!zy100_internal_meta_store_read_page(offset, verify_page))
    {
        return false;
    }
    return memcmp(verify_page,
                  page,
                  ZY100_INTERNAL_META_STORE_PAGE_BYTES) == 0;
}

bool zy100_internal_meta_store_sector_erased(uint32_t offset)
{
    uint8_t page[ZY100_INTERNAL_META_STORE_PAGE_BYTES];
    uint32_t page_offset;
    uint32_t idx;

    if (!zy100_internal_meta_aligned(offset,
                                     ZY100_INTERNAL_META_STORE_SECTOR_BYTES) ||
        !zy100_internal_meta_range_valid(offset,
                                         ZY100_INTERNAL_META_STORE_SECTOR_BYTES))
    {
        return false;
    }
    for (page_offset = 0U;
         page_offset < ZY100_INTERNAL_META_STORE_SECTOR_BYTES;
         page_offset += ZY100_INTERNAL_META_STORE_PAGE_BYTES)
    {
        if (!zy100_internal_meta_store_read_page(offset + page_offset, page))
        {
            return false;
        }
        for (idx = 0U; idx < ZY100_INTERNAL_META_STORE_PAGE_BYTES; idx++)
        {
            if (page[idx] != 0xFFU)
            {
                return false;
            }
        }
    }
    return true;
}

bool zy100_internal_meta_store_erase_sector(uint32_t offset)
{
    drv_internal_flash_status_t status;

    if (!zy100_internal_meta_aligned(offset,
                                     ZY100_INTERNAL_META_STORE_SECTOR_BYTES) ||
        !zy100_internal_meta_range_valid(offset,
                                         ZY100_INTERNAL_META_STORE_SECTOR_BYTES))
    {
        return false;
    }
    status = drv_internal_flash_erase_sector(offset);
    if (status != DRV_INTERNAL_FLASH_STATUS_OK)
    {
        DBG_DIRECT("[INTERNAL_META] erase_fail off=0x%08lX status=%s",
                   (unsigned long)offset,
                   drv_internal_flash_status_name(status));
        return false;
    }
    return zy100_internal_meta_store_sector_erased(offset);
}

bool zy100_internal_meta_store_erase_range(uint32_t offset, uint32_t length)
{
    uint32_t sector_offset;

    if (!zy100_internal_meta_aligned(offset,
                                     ZY100_INTERNAL_META_STORE_SECTOR_BYTES) ||
        !zy100_internal_meta_aligned(length,
                                     ZY100_INTERNAL_META_STORE_SECTOR_BYTES) ||
        !zy100_internal_meta_range_valid(offset, length))
    {
        return false;
    }
    for (sector_offset = offset;
         sector_offset < (offset + length);
         sector_offset += ZY100_INTERNAL_META_STORE_SECTOR_BYTES)
    {
        if (!zy100_internal_meta_store_erase_sector(sector_offset))
        {
            return false;
        }
    }
    return true;
}

#endif /* ZY100_FINAL_EDGE_MODE_ENABLE */
