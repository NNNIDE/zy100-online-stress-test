#ifndef ZY100_INTERNAL_META_STORE_H
#define ZY100_INTERNAL_META_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define ZY100_INTERNAL_META_STORE_PAGE_BYTES   256U
#define ZY100_INTERNAL_META_STORE_SECTOR_BYTES 4096U
#define ZY100_INTERNAL_META_STORE_REGION_BYTES (128UL * 1024UL)

bool zy100_internal_meta_store_ready(void);
bool zy100_internal_meta_store_read(uint32_t offset,
                                    uint8_t *buf,
                                    uint32_t len);
bool zy100_internal_meta_store_read_page(
    uint32_t offset,
    uint8_t page[ZY100_INTERNAL_META_STORE_PAGE_BYTES]);
bool zy100_internal_meta_store_program_page_overlay(
    uint32_t offset,
    const uint8_t page[ZY100_INTERNAL_META_STORE_PAGE_BYTES]);
bool zy100_internal_meta_store_sector_erased(uint32_t offset);
bool zy100_internal_meta_store_erase_sector(uint32_t offset);
bool zy100_internal_meta_store_erase_range(uint32_t offset,
                                           uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_INTERNAL_META_STORE_H */
