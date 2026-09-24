#ifndef IMU_BSP_H
#define IMU_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../common/imu_common.h"

typedef enum
{
    IMU_BSP_POWER_DRIVE_LOW = 0,
    IMU_BSP_POWER_DRIVE_HIGH,
    IMU_BSP_POWER_WEAK_PULL_UP,
} imu_bsp_power_mode_t;
typedef imu_status_t (*imu_bsp_power_restore_cb_t)(void);
imu_bsp_power_mode_t imu_bsp_power_mode(void);
/* No bus access or rail restore. Generation invalidates application parking. */
uint32_t imu_bsp_power_generation(void);
uint16_t imu_bsp_power_users(void);
bool imu_bsp_power_park_begin(void);
void imu_bsp_power_park_end(void);
/* Application owns eligibility and retained-register verification. Task context only. */
void imu_bsp_power_set_restore_callback(imu_bsp_power_restore_cb_t callback);
imu_status_t imu_bsp_power_enter_weak_pull(void);
imu_status_t imu_bsp_power_access_begin(void);
void imu_bsp_power_access_end(void);

typedef void (*imu_bsp_int_irq_cb_t)(void);
typedef void (*imu_bsp_timer_irq_cb_t)(void);

typedef enum
{
    IMU_BSP_OIS_TICK_TIMER_OWNER_NONE = 0,
    IMU_BSP_OIS_TICK_TIMER_OWNER_LIVE_FIFO = 1,
    IMU_BSP_OIS_TICK_TIMER_OWNER_OIS_CAPTURE = 2,
    IMU_BSP_OIS_TICK_TIMER_OWNER_OFFLINE_V2_FIFO = 3,
    IMU_BSP_OIS_TICK_TIMER_OWNER_FINAL_EDGE_LIVE = 4,
} imu_bsp_ois_tick_timer_owner_t;

typedef struct
{
    bool valid;
    bool pin_assigned;
    bool int_enabled;
    bool int_masked;
    imu_bsp_int_irq_cb_t callback;
} imu_bsp_int_snapshot_t;

#ifndef IMU_OIS_SPI_DMA_ENABLE
#define IMU_OIS_SPI_DMA_ENABLE 0
#endif

#ifndef IMU_SPI_DMA_TIMEOUT_US
#define IMU_SPI_DMA_TIMEOUT_US 2000U
#endif

imu_status_t imu_bsp_init(void);
imu_status_t imu_bsp_resume_after_dlps(void);
void imu_bsp_mark_lost_after_dlps_prepare(void);
void imu_bsp_set_sleep_prepare_cs_diag(bool enable);
imu_status_t imu_bsp_spi_set_mode(uint16_t cpol, uint16_t cpha);
imu_status_t imu_bsp_flash_cs_hold_high(void);
imu_status_t imu_bsp_flash_cs_high(void);
imu_status_t imu_bsp_flash_cs_low(void);
imu_status_t imu_bsp_get_flash_cs_level(uint8_t *out_level, uint8_t *in_level);
void imu_bsp_get_spi_nominal_config(uint32_t *source_hz,
                                    uint32_t *clk_div,
                                    uint32_t *baud_prescaler,
                                    uint32_t *sclk_hz);

void imu_bsp_cs_low(void);
void imu_bsp_cs_high(void);
imu_status_t imu_bsp_get_cs_level(uint8_t *out_level, uint8_t *in_level);
imu_status_t imu_bsp_get_power_level(uint8_t *out_level, uint8_t *in_level);

imu_status_t imu_bsp_spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len);
/* Task context, caller owns bus/CS. Drains RX even for TX-only transfers. */
imu_status_t imu_bsp_spi_transfer_fifo(const uint8_t *tx, uint8_t *rx, uint16_t len);
imu_status_t imu_bsp_spi_transfer_isr_fast(const uint8_t *tx, uint8_t *rx, uint16_t len);
imu_status_t imu_bsp_spi_read_reg_window_isr_fast(uint8_t cmd, uint8_t *rx, uint16_t rx_len);
imu_status_t imu_bsp_spi_read_reg_window_isr_fast_fifo(uint8_t cmd,
                                                       uint8_t *rx,
                                                       uint16_t rx_len);
imu_status_t imu_bsp_spi_read_quiet(uint8_t *rx, uint16_t len);
imu_status_t imu_bsp_spi_read_dma(uint8_t *rx, uint16_t len);
void imu_bsp_spi_dma_get_stats(uint32_t *timeout_total, uint32_t *error_total);

imu_status_t imu_bsp_ois_tick_timer_config(uint32_t period_us);
imu_status_t imu_bsp_ois_tick_timer_config_ticks(uint32_t period_ticks);
imu_status_t imu_bsp_ois_tick_timer_acquire(imu_bsp_ois_tick_timer_owner_t owner);
void imu_bsp_ois_tick_timer_release(imu_bsp_ois_tick_timer_owner_t owner);
imu_bsp_ois_tick_timer_owner_t imu_bsp_ois_tick_timer_get_owner(void);
imu_status_t imu_bsp_ois_tick_timer_start(void);
void imu_bsp_ois_tick_timer_stop(void);
bool imu_bsp_ois_tick_timer_is_running(void);
void imu_bsp_ois_tick_timer_register_irq_callback(imu_bsp_timer_irq_cb_t cb);
void imu_bsp_ois_tick_timer_get_stats(uint32_t *irq_count, uint32_t *last_vendor_tick);

void imu_bsp_delay_us(uint32_t us);
uint64_t imu_bsp_local_timestamp_us(void);

bool imu_bsp_int_pin_assigned(void);
imu_status_t imu_bsp_int_init(void);
imu_status_t imu_bsp_int_init_wom_active_low(void);
imu_status_t imu_bsp_int_init_wom_active_low_masked(void);
imu_status_t imu_bsp_int_snapshot(imu_bsp_int_snapshot_t *snapshot);
imu_status_t imu_bsp_int_restore(const imu_bsp_int_snapshot_t *snapshot);
void imu_bsp_int_register_irq_callback(imu_bsp_int_irq_cb_t cb);
uint8_t imu_bsp_int_level(void);
void imu_bsp_int_clear_pending(void);
bool imu_bsp_int_enable_irq(void);
bool imu_bsp_int_disable_irq(void);
bool imu_bsp_int_disable_for_dlps(void);

imu_status_t imu_bsp_power_ctrl(bool enable);
/* Task only, after PAD parking with sensor/bus users quiescent. Does not
 * change physical PAD levels, rail mode, IRQs, or retained WOM validity. */
void imu_bsp_sleep_gpio_park(bool hold);

#ifdef __cplusplus
}
#endif

#endif /* IMU_BSP_H */
