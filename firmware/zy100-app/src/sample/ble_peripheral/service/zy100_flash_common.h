#ifndef ZY100_FLASH_COMMON_H
#define ZY100_FLASH_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ZY100_FLASH_PREP_BUSY = 0U,
    ZY100_FLASH_PREP_DONE,
    ZY100_FLASH_PREP_ABORTED,
    ZY100_FLASH_PREP_ERROR,
    ZY100_FLASH_PREP_TIMEOUT,
} zy100_flash_prepare_status_t;

#ifdef __cplusplus
}
#endif

#endif /* ZY100_FLASH_COMMON_H */
