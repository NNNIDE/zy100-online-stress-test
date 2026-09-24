#ifndef IMU_WOM_IRQ_H
#define IMU_WOM_IRQ_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../common/imu_common.h"
#include "../driver/icm53611_driver.h"
#include "imu_wom_irq_config.h"

#define IMU_WOM_AXIS_MASK_X 0x01U
#define IMU_WOM_AXIS_MASK_Y 0x02U
#define IMU_WOM_AXIS_MASK_Z 0x04U

typedef enum
{
    IMU_WOM_IRQ_REASON_ACCEPTED = 0,
    IMU_WOM_IRQ_REASON_COOLDOWN = 1,
    IMU_WOM_IRQ_REASON_NOT_RUNNING = 2,
} imu_wom_irq_reason_t;

typedef struct
{
    uint32_t gpio_irq_count;
    uint32_t fifo_irq_count;

    uint32_t wom_raw_count;
    uint32_t wom_accepted_count;
    uint32_t wom_masked_count;

    uint32_t non_wom_gpio_count;
    uint32_t pending_overflow_count;
    uint8_t wom_hw_enabled;

    uint64_t first_wom_ts_us;
    uint64_t last_wom_ts_us;

    uint32_t first_wom_sample_seq;
    uint32_t last_wom_sample_seq;

    uint64_t last_accepted_wom_ts_us;
    uint32_t last_accepted_wom_sample_seq;

    uint32_t event_log_count;
    uint32_t event_log_dropped_count;
} imu_wom_irq_stats_t;

typedef struct
{
    uint32_t index;

    uint64_t gpio_irq_ts_us;
    uint32_t gpio_sample_seq;

    uint64_t status_read_ts_us;
    uint32_t status_sample_seq;

    uint8_t status2_raw;
    bool wom_x;
    bool wom_y;
    bool wom_z;
    bool accepted;
    imu_wom_irq_reason_t reason;
} imu_wom_irq_event_t;

typedef struct
{
    uint32_t wom_swing_count;
    uint32_t wom_duplicate_count;
    uint32_t wom_swing_event_log_count;
    uint32_t wom_swing_event_log_dropped_count;

    uint64_t first_swing_ts_us;
    uint64_t last_swing_ts_us;
    uint32_t first_swing_sample_seq;
    uint32_t last_swing_sample_seq;

    uint32_t swing_merge_window_ms;
} imu_wom_swing_stats_t;

typedef struct
{
    uint32_t index;

    uint64_t swing_start_ts_us;
    uint64_t swing_last_wom_ts_us;

    uint32_t swing_start_sample_seq;
    uint32_t swing_last_sample_seq;

    uint8_t first_axis_mask;
    uint8_t merged_axis_mask;

    uint32_t merged_wom_count;
    uint32_t duration_ms;
} imu_wom_swing_event_t;

typedef struct
{
    uint64_t candidate_ts_us;
    uint32_t sample_seq;
    uint8_t axis_mask;
    uint8_t status2_raw;
} imu_wom_irq_candidate_t;

const imu_wom_irq_config_t *imu_wom_irq_default_config(void);

void imu_wom_irq_reset(const imu_wom_irq_config_t *cfg);
imu_status_t imu_wom_irq_configure_sensor(const imu_wom_irq_config_t *cfg);
imu_status_t imu_wom_irq_enable_route(void);
imu_status_t imu_wom_irq_disable_sensor(void);

void imu_wom_irq_on_gpio_isr(uint64_t gpio_irq_ts_us, uint32_t gpio_sample_seq);
bool imu_wom_irq_process_status(const icm53611_int_status_t *status,
                                uint32_t status_sample_seq,
                                bool capture_running,
                                imu_wom_irq_candidate_t *candidate_out);

void imu_wom_irq_get_stats(imu_wom_irq_stats_t *out);
void imu_wom_irq_log_configure_failed(imu_status_t status);
void imu_wom_irq_run_pre_fifo_mreg_diag(void);
void imu_wom_irq_run_mreg_diag(void);
uint8_t imu_wom_irq_get_threshold_code_for_diag(void);
void imu_wom_irq_log_config(void);
void imu_wom_irq_log_stats(void);
void imu_wom_irq_log_events(void);
void imu_wom_irq_log_swing_stats(void);
void imu_wom_irq_log_swing_events(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_WOM_IRQ_H */
