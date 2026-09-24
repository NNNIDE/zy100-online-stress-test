#ifndef MMC5603_H
#define MMC5603_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../common/mag_common.h"

typedef enum
{
    MMC5603_BW_LEVEL_00 = 0, /* 6.6ms */
    MMC5603_BW_LEVEL_01 = 1, /* 3.5ms */
    MMC5603_BW_LEVEL_10 = 2, /* 2.0ms */
    MMC5603_BW_LEVEL_11 = 3, /* 1.2ms */
} mmc5603_bw_t;

typedef enum
{
    MMC5603_SOURCE_MODE_SINGLE = 0,
    MMC5603_SOURCE_MODE_CONTINUOUS = 1,
} mmc5603_source_mode_t;

typedef struct
{
    bool auto_sr_enable;
    mmc5603_bw_t bw;
    uint8_t continuous_odr;
    bool continuous_hpower;
} mmc5603_cfg_t;

typedef struct
{
    uint8_t raw9[9];
    uint32_t raw_x;
    uint32_t raw_y;
    uint32_t raw_z;
    uint8_t raw_temp;
    bool temp_valid;

    uint32_t sample_seq;
    uint64_t trig_local_ts_us;
    uint64_t meas_done_local_ts_us;
    uint64_t readout_local_ts_us;
    uint8_t status1_snapshot;
    uint16_t status_flags;
    mmc5603_source_mode_t source_mode;
} mmc5603_sample_t;

typedef struct
{
    uint8_t odr;
    uint8_t bw;
    uint8_t auto_sr_en;
    uint8_t cmm_en;
    uint8_t cmm_freq_en;
    uint8_t hpower;
    uint8_t ctrl0_shadow;
    uint8_t ctrl1_shadow;
    uint8_t ctrl2_shadow;
} mmc5603_state_snapshot_t;

#define MMC5603_SAMPLE_FLAG_MEAS_M_DONE  BIT(0)
#define MMC5603_SAMPLE_FLAG_MEAS_T_DONE  BIT(1)
#define MMC5603_SAMPLE_FLAG_OTP_READ_OK  BIT(2)
#define MMC5603_SAMPLE_FLAG_CONTINUOUS   BIT(3)

const mmc5603_cfg_t *mmc5603_default_config(void);

mag_status_t mmc5603_init(const mmc5603_cfg_t *cfg);
mag_status_t mmc5603_soft_reset(void);
mag_status_t mmc5603_alive_check(void);

mag_status_t mmc5603_set_bw(mmc5603_bw_t bw);
mag_status_t mmc5603_enable_auto_sr(bool enable);
mag_status_t mmc5603_set_odr(uint8_t odr);
mag_status_t mmc5603_set_cmm_en(bool enable);
mag_status_t mmc5603_set_hpower(bool enable);
mag_status_t mmc5603_set_cmm_freq_en(bool enable);
mag_status_t mmc5603_get_state_snapshot(mmc5603_state_snapshot_t *state_out);
mag_status_t mmc5603_force_power_down(void);
mag_status_t mmc5603_manual_set(void);
mag_status_t mmc5603_manual_reset(void);

mag_status_t mmc5603_take_measurement(bool read_temp, mmc5603_sample_t *sample);

mag_status_t mmc5603_set_continuous_mode(uint8_t odr, bool hpower, bool enable);
mag_status_t mmc5603_read_continuous_latest(bool read_temp, mmc5603_sample_t *sample);
/* Poll STATUS1 first and read the magnetic payload only when Meas_m_done is
 * set.  A successful not-ready poll returns MAG_STATUS_OK with fresh_out
 * false and leaves sample unspecified. */
mag_status_t mmc5603_read_continuous_fresh(bool read_temp,
                                           mmc5603_sample_t *sample,
                                           bool *fresh_out);

mag_status_t mmc5603_read_product_id(uint8_t *product_id);

void mag_parse_raw20(const uint8_t raw9[9], uint32_t *raw_x, uint32_t *raw_y, uint32_t *raw_z);

#ifdef __cplusplus
}
#endif

#endif /* MMC5603_H */
