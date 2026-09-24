#ifndef ICM53611_DRIVER_H
#define ICM53611_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../common/imu_common.h"
#include "icm53611_io_diag.h"

#ifndef ICM53611_OIS_RAW_FRAME_BYTES
#define ICM53611_OIS_RAW_FRAME_BYTES 19U
#endif

typedef struct
{
    uint8_t pwr_mgmt0;
    uint8_t gyro_config0;
    uint8_t accel_config0;
    uint8_t gyro_config1;
    uint8_t accel_config1;
} icm53611_cfg_t;

typedef struct
{
    uint16_t threshold_mg;
    uint8_t duration_samples;
    uint16_t accel_odr_hz;
    bool previous_sample_mode;
} icm53611_wom_dlps_wake_cfg_t;

typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp_raw;
    uint8_t ois_status;
    uint8_t ext_data_x;
    uint8_t ext_data_y;
    uint8_t ext_data_z;
    uint8_t ois_raw[ICM53611_OIS_RAW_FRAME_BYTES];
    uint64_t local_ts_us;
} icm53611_raw_sample_t;

typedef struct
{
    uint8_t int_status;
    uint8_t int_status2;
    uint8_t int_status3;
    uint64_t local_ts_us;
} icm53611_int_status_t;

typedef struct
{
    bool enable;
    bool timestamp_enable;
    bool accel_enable;
    bool gyro_enable;
    bool hires_enable;
    bool fsync_timestamp_enable;
    uint16_t watermark;
} icm53611_fifo_cfg_t;

typedef enum
{
    ICM53611_CAPTURE_RESET = 1,
    ICM53611_CAPTURE_INTERFACE,
    ICM53611_CAPTURE_RESET_DONE,
    ICM53611_CAPTURE_CONFIG,
    ICM53611_CAPTURE_FLUSH,
    ICM53611_CAPTURE_FIFO,
    ICM53611_CAPTURE_MODE,
    ICM53611_CAPTURE_PLATFORM,
    ICM53611_CAPTURE_LOST_COUNT,
    ICM53611_CAPTURE_IRQ,
    ICM53611_CAPTURE_TIMER,
    ICM53611_CAPTURE_FIRST_PACKET
} icm53611_capture_stage_t;

/* Cached capture-flush outcome; not a BLE or persistent wire field. */
enum { ICM53611_FLUSH_CLEAR = 0, ICM53611_FLUSH_ELAPSED,
       ICM53611_FLUSH_OS_GUARD, ICM53611_FLUSH_POLL_GUARD,
       ICM53611_FLUSH_IO_ERROR, ICM53611_FLUSH_NOT_RUN = 255 };
typedef struct
{
    imu_spi_error_t io;
    uint32_t flush_us;
    uint16_t flush_polls;
    uint8_t flush_end;
    uint8_t stage;
    uint8_t status;
} icm53611_capture_result_t;

/* New session only. Caller owns the sensor domain and has detached wake IRQ.
 * Recovery/tail-drain APIs intentionally do not call this reset entry. */
imu_status_t icm53611_prepare_capture(const icm53611_cfg_t *cfg,
                                     const icm53611_fifo_cfg_t *fifo,
                                     icm53611_capture_result_t *result);

typedef struct
{
    uint8_t pwr_mgmt0;
    uint8_t gyro_config0;
    uint8_t accel_config0;
    uint8_t fifo_config1;
    uint8_t fifo_config5;
    uint8_t fdr_config;
    uint8_t tmst_config1;
} icm53611_a2_rate_regs_t;

typedef struct
{
    uint8_t before;
    uint8_t after;
    uint8_t skip;
    imu_status_t status;
    uint32_t total_us;
    uint32_t read_us;
    uint32_t write_verify_us;
} icm53611_ois_prearm_result_t;

typedef struct
{
    bool valid;
    const char *bank_name;
    const char *reg_name;
    uint8_t bank;
    uint8_t addr;
    uint8_t write_value;
    uint8_t read_value;
    uint8_t expected_mask;
    uint8_t expected_value;
    imu_status_t status;
    uint8_t retry_count;
} icm53611_mreg_verify_error_t;

imu_status_t icm53611_init_bus(void);
imu_status_t icm53611_read_whoami(uint8_t *who_am_i);
bool icm53611_runtime_resume_probe(uint8_t *who_out, imu_status_t *status_out);
imu_status_t icm53611_soft_reset(void);
imu_status_t icm53611_apply_basic_config(const icm53611_cfg_t *cfg);
imu_status_t icm53611_bringup(const icm53611_cfg_t *cfg);
imu_status_t icm53611_alive_check(void);

imu_status_t icm53611_read_raw_sample(icm53611_raw_sample_t *sample);
imu_status_t icm53611_read_ui_6axis_sample(icm53611_raw_sample_t *sample);
imu_status_t icm53611_read_ois_sample(icm53611_raw_sample_t *sample);
imu_status_t icm53611_read_ois_sample_isr_fast(icm53611_raw_sample_t *sample);
imu_status_t icm53611_read_ois_raw_frame_isr_fast_locked(
    uint8_t raw[ICM53611_OIS_RAW_FRAME_BYTES]);
imu_status_t icm53611_read_ois_tmst_raw_isr_fast_locked(
    uint16_t *tmst_raw_out);
imu_status_t icm53611_decode_ois_raw_frame(
    const uint8_t raw[ICM53611_OIS_RAW_FRAME_BYTES],
    icm53611_raw_sample_t *sample);
imu_status_t icm53611_decode_ois_raw_accel16(
    const uint8_t raw[ICM53611_OIS_RAW_FRAME_BYTES],
    int16_t *ax,
    int16_t *ay,
    int16_t *az);
bool icm53611_decode_ois20_accel_raw(
    const uint8_t raw[ICM53611_OIS_RAW_FRAME_BYTES],
    int32_t *ax,
    int32_t *ay,
    int32_t *az);
imu_status_t icm53611_write_with_verify(uint8_t reg, uint8_t value, uint8_t verify_mask);

imu_status_t icm53611_read_int_status(icm53611_int_status_t *status);
/* Extended task API: may return IMU_STATUS_FLASH_BUSY before transfer. */
imu_status_t icm53611_read_int_status_ex(icm53611_int_status_t *status);

imu_status_t icm53611_fifo_config(const icm53611_fifo_cfg_t *cfg);
imu_status_t icm53611_fifo_get_count(uint16_t *count);
/* Extended task API: may return IMU_STATUS_FLASH_BUSY before transfer. */
imu_status_t icm53611_fifo_get_count_ex(uint16_t *count);
imu_status_t icm53611_fifo_get_lost_count(uint16_t *lost_count);
/* Extended task API: may return IMU_STATUS_FLASH_BUSY before transfer. */
imu_status_t icm53611_fifo_get_lost_count_ex(uint16_t *lost_count);
imu_status_t icm53611_fifo_read(uint8_t *buf, uint16_t len);
/* Extended task API: may return IMU_STATUS_FLASH_BUSY before transfer. */
imu_status_t icm53611_fifo_read_ex(uint8_t *buf, uint16_t len);
imu_status_t icm53611_fifo_flush(void);
imu_status_t icm53611_fifo_flush_wait_clear(uint16_t *fifo_count_after);
imu_status_t icm53611_fifo_set_bypass(bool enable);
imu_status_t icm53611_stop_ui_sensors_preserve_fifo(void);
/* Extended task API: may return IMU_STATUS_FLASH_BUSY before transfer. */
imu_status_t icm53611_stop_ui_sensors_preserve_fifo_ex(void);
imu_status_t icm53611_ois_fast_enable_from_current_bus(void);
imu_status_t icm53611_ois_prearm_config_disable_clear(
    icm53611_ois_prearm_result_t *result);
imu_status_t icm53611_ois_prearm_timestamp_regs(
    icm53611_ois_prearm_result_t *result);
imu_status_t icm53611_read_a2_rate_regs(icm53611_a2_rate_regs_t *regs);
imu_status_t icm53611_restore_ui_fifo_1600hz_minimal(void);
imu_status_t icm53611_restore_ui_fifo_800hz_minimal(void);
imu_status_t icm53611_prepare_for_sleep(void);
imu_status_t icm53611_configure_packet3_fifo_stream(const icm53611_cfg_t *cfg,
                                                    const icm53611_fifo_cfg_t *fifo_cfg);

imu_status_t icm53611_set_wom_config(uint8_t wom_cfg);
imu_status_t icm53611_wom_set_threshold_code(uint8_t threshold_code);
imu_status_t icm53611_wom_set_config(uint8_t wom_cfg);
imu_status_t icm53611_wom_route_int1(bool enable);
imu_status_t icm53611_wom_disable(void);
imu_status_t icm53611_prepare_wom_wake_for_dlps(
    const icm53611_wom_dlps_wake_cfg_t *cfg);
/* Read-only retained-WOM check; does not clear interrupt status or reset the chip. */
imu_status_t icm53611_verify_wom_retained(const icm53611_wom_dlps_wake_cfg_t *cfg);
imu_status_t icm53611_disable_wom_wake_for_dlps(void);
bool icm53611_get_last_mreg_verify_error(icm53611_mreg_verify_error_t *out);
void icm53611_wom_pre_fifo_mreg_diag(void);
void icm53611_wom_mreg_diag(uint8_t threshold_code);
imu_status_t icm53611_wom_read_threshold_codes(uint8_t *x_code,
                                               uint8_t *y_code,
                                               uint8_t *z_code);
void icm53611_wom_log_apex_disable_status(void);
void icm53611_wom_debug_try_apex_enable(uint8_t threshold_code);

const icm53611_cfg_t *icm53611_default_config(void);

#ifdef __cplusplus
}
#endif

#endif /* ICM53611_DRIVER_H */
