#include "zy100_crc32.h"

#define ZY100_CRC32_IEEE_POLY_REF 0xEDB88320UL

uint32_t zy100_crc32_ieee_begin(void)
{
    return ZY100_CRC32_IEEE_INIT;
}

uint32_t zy100_crc32_ieee_update(uint32_t crc,
                                 const uint8_t *data,
                                 uint32_t len)
{
    uint32_t idx;
    uint8_t bit;

    if (data == 0)
    {
        return crc;
    }

    for (idx = 0U; idx < len; idx++)
    {
        crc ^= data[idx];
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (crc >> 1U) ^ ZY100_CRC32_IEEE_POLY_REF;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

uint32_t zy100_crc32_ieee_finish(uint32_t crc)
{
    return crc ^ ZY100_CRC32_IEEE_XOR_OUT;
}

uint32_t zy100_crc32_ieee(const uint8_t *data, uint32_t len)
{
    return zy100_crc32_ieee_finish(
        zy100_crc32_ieee_update(zy100_crc32_ieee_begin(), data, len));
}
