#ifndef ZY100_CALIBRATION_MANAGER_H
#define ZY100_CALIBRATION_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include <profile_server.h>

#define ZY100_CAL_MAG_VISUAL_NONE       0U
#define ZY100_CAL_MAG_VISUAL_ACTIVE     1U
#define ZY100_CAL_MAG_VISUAL_SUCCESS    2U
#define ZY100_CAL_MAG_VISUAL_FAILURE    3U
#define ZY100_CAL_MAG_VISUAL_CANCEL     4U

typedef enum
{
    ZY100_CAL_MAG_START_GATE_WAIT = 0U,
    ZY100_CAL_MAG_START_GATE_READY,
    ZY100_CAL_MAG_START_GATE_ABORT,
    ZY100_CAL_MAG_START_GATE_RESOURCE_FAILED,
} zy100_cal_mag_start_gate_t;

void zy100_cal_manager_init(void);
void zy100_cal_manager_on_tx_cccd(uint8_t conn_id, bool enabled);
void zy100_cal_manager_on_status_cccd(uint8_t conn_id, bool enabled);
void zy100_cal_manager_on_disconnect(uint8_t conn_id);
void zy100_cal_manager_on_write(uint8_t conn_id,
                                T_WRITE_TYPE write_type,
                                const uint8_t *data,
                                uint16_t len,
                                bool paired,
                                bool idle,
                                bool record_read_allowed);
void zy100_cal_manager_poll(uint8_t conn_id,
                            uint16_t att_mtu,
                            bool connected,
                            bool paired,
                            bool idle,
                            zy100_cal_mag_start_gate_t mag_start_gate);
bool zy100_cal_manager_mag_active(void);
bool zy100_cal_manager_mag_prepare_needed(void);
void zy100_cal_manager_mag_prepare_complete(void);
/* Application policy supplies the bounded preparation budget at acceptance. */
uint32_t zy100_cal_manager_port_prepare_budget_ms(void);
bool zy100_cal_manager_mag_flash_busy(void);
bool zy100_cal_manager_info_confirmed(uint8_t conn_id);
bool zy100_cal_manager_business_ready(uint8_t conn_id);
void zy100_cal_manager_abort_mag_for_shutdown(void);
/* Drain persistence without notifications or new calibration work. */
bool zy100_cal_manager_shutdown_poll(void);
uint8_t zy100_cal_manager_mag_visual_state(void);
void zy100_cal_manager_mag_visual_result_consumed(void);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_CALIBRATION_MANAGER_H */
