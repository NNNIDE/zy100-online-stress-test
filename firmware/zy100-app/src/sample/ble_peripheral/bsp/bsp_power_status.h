#ifndef BSP_POWER_STATUS_H
#define BSP_POWER_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "rtl876x.h"

/*
 * ZY-A100 V1.1: CHG-INT is a USB/VBUS-derived board detect input on P0_1.
 * It is not the YHM2712 STACMD/CHG status pin.
 */
#ifndef ZY100_CHG_INT_PIN
#define ZY100_CHG_INT_PIN                         P0_1
#endif

/* Default from the V1.1 divider use: high means external USB/VBUS is present. */
#ifndef ZY100_CHG_INT_EXTERNAL_PRESENT_LEVEL
#define ZY100_CHG_INT_EXTERNAL_PRESENT_LEVEL      1U
#endif

typedef enum
{
    BSP_POWER_STATUS_OK = 0U,
    BSP_POWER_STATUS_INVALID_PARAM,
    BSP_POWER_STATUS_INVALID_PIN,
} bsp_power_status_t;

bsp_power_status_t bsp_power_status_init(void);
bsp_power_status_t bsp_power_status_chg_int_level(uint8_t *level);
bsp_power_status_t bsp_power_status_chg_int_external_power_present(bool *present);
bsp_power_status_t bsp_power_status_chg_int_rearm_wakeup(bool *present,
                                                         uint8_t *wake_polarity);
bool bsp_power_status_chg_int_wakeup_pending(void);
void bsp_power_status_chg_int_clear_wakeup(void);
const char *bsp_power_status_name(bsp_power_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* BSP_POWER_STATUS_H */
