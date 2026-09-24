#ifndef ZY100_BYTEORDER_H
#define ZY100_BYTEORDER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(ZY100_BUILD_FACTORY) && ZY100_BUILD_FACTORY

static inline uint16_t zy100_get_u16_le(const uint8_t *src)
{
    return (uint16_t)(((uint16_t)src[0]) |
                      ((uint16_t)src[1] << 8U));
}

static inline uint32_t zy100_get_u32_le(const uint8_t *src)
{
    return ((uint32_t)src[0]) |
           ((uint32_t)src[1] << 8U) |
           ((uint32_t)src[2] << 16U) |
           ((uint32_t)src[3] << 24U);
}

static inline uint64_t zy100_get_u64_le(const uint8_t *src)
{
    uint64_t lo = (uint64_t)zy100_get_u32_le(src);
    uint64_t hi = (uint64_t)zy100_get_u32_le(src + 4U);

    return lo | (hi << 32U);
}

static inline void zy100_put_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static inline void zy100_put_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16U) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static inline void zy100_put_u64_le(uint8_t *dst, uint64_t value)
{
    zy100_put_u32_le(dst, (uint32_t)(value & 0xFFFFFFFFULL));
    zy100_put_u32_le(dst + 4U, (uint32_t)(value >> 32U));
}

static inline uint16_t zy100_get_u16_be(const uint8_t *src)
{
    return (uint16_t)(((uint16_t)src[0] << 8U) |
                      ((uint16_t)src[1]));
}

static inline uint32_t zy100_get_u32_be(const uint8_t *src)
{
    return ((uint32_t)src[0] << 24U) |
           ((uint32_t)src[1] << 16U) |
           ((uint32_t)src[2] << 8U) |
           ((uint32_t)src[3]);
}

#else

uint16_t zy100_get_u16_le(const uint8_t *src);
uint32_t zy100_get_u32_le(const uint8_t *src);
uint64_t zy100_get_u64_le(const uint8_t *src);

void zy100_put_u16_le(uint8_t *dst, uint16_t value);
void zy100_put_u32_le(uint8_t *dst, uint32_t value);
void zy100_put_u64_le(uint8_t *dst, uint64_t value);

uint16_t zy100_get_u16_be(const uint8_t *src);
uint32_t zy100_get_u32_be(const uint8_t *src);

#endif

#ifdef __cplusplus
}
#endif

#endif
