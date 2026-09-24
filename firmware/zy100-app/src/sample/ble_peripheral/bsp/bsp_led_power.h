#ifndef BSP_LED_POWER_H
#define BSP_LED_POWER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BSP_LED_POWER_STATUS_OK = 0U,
    BSP_LED_POWER_STATUS_INVALID_PIN,
} bsp_led_power_status_t;

typedef enum
{
    BSP_LED_POWER_CTRL_MODE_GPIO = 0U,
    BSP_LED_POWER_CTRL_MODE_PAD_SW,
} bsp_led_power_ctrl_mode_t;

bsp_led_power_status_t bsp_led_power_init(void);
bsp_led_power_status_t bsp_led_power_on(void);
bsp_led_power_status_t bsp_led_power_off(void);
bool bsp_led_power_is_on(void);
void bsp_led_power_data_low(void);
bsp_led_power_status_t bsp_led_power_debug_drive_ctrl(bool level_high,
                                                      bsp_led_power_ctrl_mode_t mode);
uint8_t bsp_led_power_ctrl_out_level(void);
uint8_t bsp_led_power_ctrl_in_level(void);
uint8_t bsp_led_power_ctrl_pad_out_level(void);
uint8_t bsp_led_power_ctrl_pad_oe_level(void);
uint8_t bsp_led_power_ctrl_pad_mode_level(void);
uint8_t bsp_led_power_ctrl_pad_pwr_level(void);
uint8_t bsp_led_power_ctrl_pull_up_enabled(void);
uint8_t bsp_led_power_data_out_level(void);
const char *bsp_led_power_status_name(bsp_led_power_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* BSP_LED_POWER_H */
