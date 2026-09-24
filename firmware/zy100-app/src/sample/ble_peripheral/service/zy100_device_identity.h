#ifndef ZY100_DEVICE_IDENTITY_H
#define ZY100_DEVICE_IDENTITY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 
*/
#ifndef ZY100_DEVICE_INTERNAL_ID
#define ZY100_DEVICE_INTERNAL_ID 5U
#endif

#define ZY100_IDENTITY_OFFSET 912367U

const char *zy100_device_ble_name(void);
const char *zy100_device_sn(void);
uint32_t zy100_device_internal_id(void);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_DEVICE_IDENTITY_H */
