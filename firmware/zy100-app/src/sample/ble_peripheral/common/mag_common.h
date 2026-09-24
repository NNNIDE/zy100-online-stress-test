#ifndef MAG_COMMON_H
#define MAG_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "trace.h"

#ifndef MAG_LOG_ENABLE
#define MAG_LOG_ENABLE 1
#endif

#if MAG_LOG_ENABLE
#define MAG_LOG_INFO(...)  DBG_DIRECT("[MAG] " __VA_ARGS__)
#define MAG_LOG_WARN(...)  DBG_DIRECT("[MAG][WARN] " __VA_ARGS__)
#define MAG_LOG_ERROR(...) DBG_DIRECT("[MAG][ERR] " __VA_ARGS__)
#else
#define MAG_LOG_INFO(...)
#define MAG_LOG_WARN(...)
#define MAG_LOG_ERROR(...)
#endif

#ifndef MAG_UNUSED
#define MAG_UNUSED(x) ((void)(x))
#endif

typedef enum
{
    MAG_STATUS_OK = 0,
    MAG_STATUS_INVALID_PARAM = 1,
    MAG_STATUS_TIMEOUT = 2,
    MAG_STATUS_NACK = 3,
    MAG_STATUS_BUS_BUSY = 4,
    MAG_STATUS_BUS_ERROR = 5,
    MAG_STATUS_NOT_READY = 6,
    MAG_STATUS_NOT_FOUND = 7,
    MAG_STATUS_VERIFY_FAILED = 8,
    MAG_STATUS_UNSUPPORTED = 9,
} mag_status_t;

#ifdef __cplusplus
}
#endif

#endif /* MAG_COMMON_H */
