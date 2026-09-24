#ifndef ZY100_CRC32_H
#define ZY100_CRC32_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ZY100_CRC32_IEEE_INIT     0xFFFFFFFFUL
#define ZY100_CRC32_IEEE_XOR_OUT  0xFFFFFFFFUL

uint32_t zy100_crc32_ieee_begin(void);
uint32_t zy100_crc32_ieee_update(uint32_t crc,
                                 const uint8_t *data,
                                 uint32_t len);
uint32_t zy100_crc32_ieee_finish(uint32_t crc);
uint32_t zy100_crc32_ieee(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_CRC32_H */
