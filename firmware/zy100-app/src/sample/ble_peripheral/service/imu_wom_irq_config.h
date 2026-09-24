#ifndef IMU_WOM_IRQ_CONFIG_H
#define IMU_WOM_IRQ_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define IMU_WOM_THRESHOLD_MG_MAX 1000U
#define IMU_WOM_EVENT_LOG_MAX    128U
#define IMU_WOM_SWING_EVENT_LOG_MAX IMU_WOM_EVENT_LOG_MAX
#define IMU_WOM_SWING_MERGE_WINDOW_MS 500U

#define IMU_WOM_STRICT_CONFIG_VERIFY       0
#ifndef IMU_WOM_MREG_DIAG_ENABLE
#define IMU_WOM_MREG_DIAG_ENABLE           IMU_MREG_DIAG_LOG_ENABLE
#endif
#define IMU_WOM_DEBUG_TRY_APEX_ENABLE      0
#define IMU_WOM_FIFO_ORDER_DIAG_ENABLE     0

#if IMU_WOM_DEBUG_TRY_APEX_ENABLE && IMU_WOM_FIFO_ORDER_DIAG_ENABLE
#error "IMU_WOM_DEBUG_TRY_APEX_ENABLE and IMU_WOM_FIFO_ORDER_DIAG_ENABLE must not both be enabled"
#endif

typedef struct
{
    bool enable_wom_accel;

    uint16_t wom_threshold_mg;
    uint16_t wom_duration_samples;
    uint32_t cooldown_ms;

    bool latch_enable;
    bool active_high;
} imu_wom_irq_config_t;

static const imu_wom_irq_config_t g_default_imu_wom_irq_cfg =
{
    .enable_wom_accel = true,
    .wom_threshold_mg = 1000U,
    .wom_duration_samples = 1U,
    .cooldown_ms = 300U,
    .latch_enable = true,
    .active_high = true,
};

#ifdef __cplusplus
}
#endif

#endif /* IMU_WOM_IRQ_CONFIG_H */
