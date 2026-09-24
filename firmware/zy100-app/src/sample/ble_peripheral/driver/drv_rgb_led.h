#ifndef DRV_RGB_LED_H
#define DRV_RGB_LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../bsp/led_board_pinmap.h"

typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} zy100_rgb_color_t;

typedef enum
{
    DRV_RGB_LED_STATUS_OK = 0U,
    DRV_RGB_LED_STATUS_INVALID_PARAM,
    DRV_RGB_LED_STATUS_INVALID_PIN,
    DRV_RGB_LED_STATUS_NOT_READY,
    DRV_RGB_LED_STATUS_TICK_STUCK,
    DRV_RGB_LED_STATUS_BUSY,
    DRV_RGB_LED_STATUS_TIMEOUT,
    DRV_RGB_LED_STATUS_DMA_ERROR,
    DRV_RGB_LED_STATUS_CLOCK_ERROR,
} drv_rgb_led_status_t;

drv_rgb_led_status_t drv_rgb_led_init(void);
drv_rgb_led_status_t drv_rgb_led_set_timing_ticks(uint32_t t0h_ticks,
                                                  uint32_t t0l_ticks,
                                                  uint32_t t1h_ticks,
                                                  uint32_t t1l_ticks);
drv_rgb_led_status_t drv_rgb_led_show(const zy100_rgb_color_t *colors, uint16_t count);
drv_rgb_led_status_t drv_rgb_led_all_off(uint16_t count);
drv_rgb_led_status_t drv_rgb_led_send_edge_test(uint8_t edge_count);
drv_rgb_led_status_t drv_rgb_led_wait_us_checked(uint32_t us);
void drv_rgb_led_data_low(void);
uint32_t drv_rgb_led_gpio_mask(void);
const char *drv_rgb_led_status_name(drv_rgb_led_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* DRV_RGB_LED_H */
