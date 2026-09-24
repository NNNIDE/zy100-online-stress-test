#ifndef DRV_INTERNAL_FLASH_H
#define DRV_INTERNAL_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    DRV_INTERNAL_FLASH_STATUS_OK = 0U,
    DRV_INTERNAL_FLASH_STATUS_INVALID_PARAM,
    DRV_INTERNAL_FLASH_STATUS_NO_REGION,
    DRV_INTERNAL_FLASH_STATUS_LAYOUT_MISMATCH,
    DRV_INTERNAL_FLASH_STATUS_OUT_OF_RANGE,
    DRV_INTERNAL_FLASH_STATUS_ERASE_FAILED,
    DRV_INTERNAL_FLASH_STATUS_WRITE_FAILED,
    DRV_INTERNAL_FLASH_STATUS_READ_FAILED,
} drv_internal_flash_status_t;

typedef struct
{
    uint32_t base_addr;
    uint32_t size_bytes;
    uint32_t sector_bytes;
} drv_internal_flash_region_t;

drv_internal_flash_status_t drv_internal_flash_get_region(drv_internal_flash_region_t *region);
drv_internal_flash_status_t drv_internal_flash_erase_sector(uint32_t offset);
drv_internal_flash_status_t drv_internal_flash_erase_range(uint32_t offset, uint32_t length);
drv_internal_flash_status_t drv_internal_flash_erase_all(void);
drv_internal_flash_status_t drv_internal_flash_write(uint32_t offset, const uint8_t *data, uint32_t length);
drv_internal_flash_status_t drv_internal_flash_read(uint32_t offset, uint8_t *data, uint32_t length);
const char *drv_internal_flash_status_name(drv_internal_flash_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* DRV_INTERNAL_FLASH_H */
