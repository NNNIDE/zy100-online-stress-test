#ifndef IMU_COMMON_H
#define IMU_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../app_flags.h"
#include "trace.h"

#ifndef IMU_RUNTIME_VERBOSE_LOG_ENABLE
#define IMU_RUNTIME_VERBOSE_LOG_ENABLE 0
#endif

#ifndef IMU_MARKER_RUNTIME_LOG_ENABLE
#define IMU_MARKER_RUNTIME_LOG_ENABLE 0
#endif

#ifndef IMU_AXIS_DIAG_RUNTIME_LOG_ENABLE
#define IMU_AXIS_DIAG_RUNTIME_LOG_ENABLE 0
#endif

#ifndef IMU_WOM_RUNTIME_LOG_ENABLE
#define IMU_WOM_RUNTIME_LOG_ENABLE 0
#endif

#ifndef IMU_MREG_DIAG_LOG_ENABLE
#define IMU_MREG_DIAG_LOG_ENABLE 0
#endif

#ifndef IMU_CAPTURE_PROGRESS_LOG_ENABLE
#define IMU_CAPTURE_PROGRESS_LOG_ENABLE 0
#endif

#ifndef IMU_ERROR_LOG_ENABLE
#define IMU_ERROR_LOG_ENABLE 1
#endif

#ifndef IMU_CAPTURE_SUMMARY_LOG_ENABLE
#define IMU_CAPTURE_SUMMARY_LOG_ENABLE 1
#endif

#ifndef IMU_LOG_ENABLE
#define IMU_LOG_ENABLE IMU_ERROR_LOG_ENABLE
#endif

#ifndef IMU_LOG_LEVEL
/* 0: off, 1: error only, 2: warn+error, 3: info+warn+error */
#define IMU_LOG_LEVEL 2
#endif

#if IMU_LOG_ENABLE && (IMU_LOG_LEVEL >= 3)
#define IMU_LOG_INFO(...)  DBG_DIRECT("[IMU] " __VA_ARGS__)
#else
#define IMU_LOG_INFO(...)  do { if (0) { DBG_DIRECT("[IMU] " __VA_ARGS__); } } while (0)
#endif

#if IMU_LOG_ENABLE && (IMU_LOG_LEVEL >= 2)
#define IMU_LOG_WARN(...)  DBG_DIRECT("[WARN][IMU] " __VA_ARGS__)
#else
#define IMU_LOG_WARN(...)  do { if (0) { DBG_DIRECT("[WARN][IMU] " __VA_ARGS__); } } while (0)
#endif

#if IMU_LOG_ENABLE && (IMU_LOG_LEVEL >= 1)
#define IMU_LOG_ERROR(...) DBG_DIRECT("[ERR][IMU] " __VA_ARGS__)
#else
#define IMU_LOG_ERROR(...) do { if (0) { DBG_DIRECT("[ERR][IMU] " __VA_ARGS__); } } while (0)
#endif

#ifndef IMU_UNUSED
#define IMU_UNUSED(x) ((void)(x))
#endif

#define IMU_PIN_UNASSIGNED 0xFFU

typedef enum
{
    IMU_STATUS_OK = 0,
    IMU_STATUS_INVALID_PARAM = 1,
    IMU_STATUS_TIMEOUT = 2,
    IMU_STATUS_BUS_ERROR = 3,
    IMU_STATUS_NOT_READY = 4,
    IMU_STATUS_NOT_FOUND = 5,
    IMU_STATUS_VERIFY_FAILED = 6,
    IMU_STATUS_UNSUPPORTED = 7,
    /* Extended task access: Flash owns/reserves SPI; no transfer was started. */
    IMU_STATUS_FLASH_BUSY = 8,
} imu_status_t;

/* Keep existing public/wire error details unchanged. */
static inline imu_status_t imu_status_legacy(imu_status_t status)
{
    return (status == IMU_STATUS_FLASH_BUSY) ? IMU_STATUS_BUS_ERROR : status;
}

#ifdef __cplusplus
}
#endif

#endif /* IMU_COMMON_H */
