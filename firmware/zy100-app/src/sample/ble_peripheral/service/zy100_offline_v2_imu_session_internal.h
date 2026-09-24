#ifndef ZY100_OFFLINE_V2_IMU_SESSION_INTERNAL_H
#define ZY100_OFFLINE_V2_IMU_SESSION_INTERNAL_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ZY100_OFFLINE_V2_SPI_RETRY_DEFER = 0U,
    ZY100_OFFLINE_V2_SPI_RETRY_FATAL,
} zy100_offline_v2_spi_retry_result_t;

typedef struct
{
    uint32_t start_us;
    uint32_t defer_count;
    uint32_t consecutive_count;
    uint32_t max_consecutive_count;
    uint32_t max_duration_us;
    bool active;
} zy100_offline_v2_spi_retry_policy_t;

static inline uint32_t zy100_offline_v2_due_merge(uint32_t current_due,
                                                   uint32_t restore_due,
                                                   uint32_t max_due)
{
    uint32_t available;

    if (current_due >= max_due)
    {
        return max_due;
    }
    available = max_due - current_due;
    if (restore_due > available)
    {
        restore_due = available;
    }
    return current_due + restore_due;
}

static inline void zy100_offline_v2_spi_retry_reset(
    zy100_offline_v2_spi_retry_policy_t *policy)
{
    if (policy == NULL)
    {
        return;
    }
    policy->start_us = 0U;
    policy->defer_count = 0U;
    policy->consecutive_count = 0U;
    policy->max_consecutive_count = 0U;
    policy->max_duration_us = 0U;
    policy->active = false;
}

static inline void zy100_offline_v2_spi_retry_complete(
    zy100_offline_v2_spi_retry_policy_t *policy,
    uint32_t now_us)
{
    uint32_t duration_us;

    if ((policy == NULL) || !policy->active)
    {
        return;
    }
    duration_us = now_us - policy->start_us;
    if (duration_us > policy->max_duration_us)
    {
        policy->max_duration_us = duration_us;
    }
    policy->start_us = 0U;
    policy->consecutive_count = 0U;
    policy->active = false;
}

static inline zy100_offline_v2_spi_retry_result_t
zy100_offline_v2_spi_retry_on_bus_error(
    zy100_offline_v2_spi_retry_policy_t *policy,
    bool flash_owner,
    uint32_t now_us,
    uint32_t budget_us)
{
    uint32_t duration_us;

    if ((policy == NULL) || !flash_owner)
    {
        return ZY100_OFFLINE_V2_SPI_RETRY_FATAL;
    }
    if (!policy->active)
    {
        policy->active = true;
        policy->start_us = now_us;
        policy->consecutive_count = 0U;
    }
    policy->consecutive_count++;
    if (policy->consecutive_count > policy->max_consecutive_count)
    {
        policy->max_consecutive_count = policy->consecutive_count;
    }
    duration_us = now_us - policy->start_us;
    if (duration_us > policy->max_duration_us)
    {
        policy->max_duration_us = duration_us;
    }
    if (duration_us >= budget_us)
    {
        return ZY100_OFFLINE_V2_SPI_RETRY_FATAL;
    }
    policy->defer_count++;
    return ZY100_OFFLINE_V2_SPI_RETRY_DEFER;
}

#endif /* ZY100_OFFLINE_V2_IMU_SESSION_INTERNAL_H */
