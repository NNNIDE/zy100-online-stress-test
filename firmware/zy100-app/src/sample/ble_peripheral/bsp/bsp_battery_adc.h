#ifndef BSP_BATTERY_ADC_H
#define BSP_BATTERY_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define BSP_BATTERY_ADC_RAW_MAX 4095U

typedef enum
{
    BSP_BATTERY_ADC_STATUS_OK = 0U,
    BSP_BATTERY_ADC_STATUS_INVALID_PARAM,
    BSP_BATTERY_ADC_STATUS_NOT_INITIALIZED,
    BSP_BATTERY_ADC_STATUS_NOT_ACTIVE,
    BSP_BATTERY_ADC_STATUS_TIMEOUT,
    BSP_BATTERY_ADC_STATUS_RAW_OUT_OF_RANGE,
    BSP_BATTERY_ADC_STATUS_CALIBRATION_UNAVAILABLE,
    BSP_BATTERY_ADC_STATUS_CONVERSION_ERROR,
} bsp_battery_adc_status_t;

typedef struct
{
    uint8_t pin;
    uint8_t channel;
    bool bypass_mode;
    bool calibration_ready;
    bool hw_active;
} bsp_battery_adc_info_t;

bsp_battery_adc_status_t bsp_battery_adc_init(void);
void bsp_battery_adc_prepare_runtime_pin(void);
void bsp_battery_adc_park_low_power(void);
bsp_battery_adc_status_t bsp_battery_adc_hw_enable(void);
void bsp_battery_adc_hw_disable(void);
bsp_battery_adc_status_t bsp_battery_adc_read_one_shot(uint16_t *raw_out);
bsp_battery_adc_status_t bsp_battery_adc_raw_to_vbat_mv(uint16_t raw,
                                                        uint16_t *mv_out);
void bsp_battery_adc_get_info(bsp_battery_adc_info_t *info);
const char *bsp_battery_adc_status_name(bsp_battery_adc_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* BSP_BATTERY_ADC_H */
