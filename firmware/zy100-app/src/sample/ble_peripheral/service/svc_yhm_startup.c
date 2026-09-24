#include "svc_yhm_startup.h"
#include "bsp/bsp_power_status.h"
#include "rtl876x_gpio.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"
#include "trace.h"
#include "os_sched.h"
#include <string.h>

void svc_yhm_startup_begin(svc_yhm_startup_t *s, uint32_t now,
    uint32_t minimum_ms, svc_yhm_startup_attempt_t attempt, void *context)
{
#if !(ZY100_BUILD_PRODUCTION || ZY100_BUILD_FACTORY)
    GPIO_InitTypeDef gpio;
#endif
    memset(s, 0, sizeof(*s));
    s->started = s->last_log = now;
    s->due = now;
    s->minimum_ms = minimum_ms;
    s->last_key = 0xffU;
    s->last_status = YHM2712_ACMD_STATUS_NOT_INIT;
    s->attempt = attempt;
    s->context = context;
    s->begun = true;
    /* Same V1.2 active-low input as Factory. Do not consume key events. */
#if !(ZY100_BUILD_PRODUCTION || ZY100_BUILD_FACTORY)
    Pad_Config(P1_0, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP,
               PAD_OUT_DISABLE, PAD_OUT_HIGH);
    Pinmux_Config(P1_0, DWGPIO);
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_GetPin(P1_0);
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_ITCmd = DISABLE;
    gpio.GPIO_ControlMode = GPIO_SOFTWARE_MODE;
    GPIO_Init(&gpio);
#endif
    (void)bsp_power_status_init();
}

bool svc_yhm_startup_ready(const svc_yhm_startup_t *s)
{
    return s->begun && s->phase == YHM_STARTUP_READY;
}

bool svc_yhm_startup_poll(svc_yhm_startup_t *s, uint32_t now)
{
    uint8_t key;
    bsp_power_status_t vin_status;
    yhm2712_acmd_recovery_snapshot_t recovery;
    if (!s->begun || s->attempt == NULL) return false;
    if (svc_yhm_startup_ready(s)) return true;
    key = GPIO_ReadInputDataBit(GPIO_GetPin(P1_0));
    if (key != s->last_key || (uint32_t)(now - s->last_log) >= 30000U) {
        DBG_DIRECT("[YHM_STARTUP][WAIT] ms=%lu key=%u attempts=%lu",
            (unsigned long)(now - s->started), key, (unsigned long)s->attempts);
        s->last_key = key;
        s->last_log = now;
    }
    if (key == 0U) {
        s->saw_pressed = true;
        s->tracking = false;
        s->phase = YHM_STARTUP_WAIT_RELEASE;
        return false;
    }
    if (!s->tracking) { s->released = now; s->tracking = true; }
    if ((uint32_t)(now - s->released) < 100U ||
        (uint32_t)(now - s->started) < s->minimum_ms ||
        (int32_t)(now - s->due) < 0) return false;
    vin_status = bsp_power_status_chg_int_external_power_present(&s->external);
    if (s->attempts != UINT32_MAX) ++s->attempts;
    s->last_status = vin_status == BSP_POWER_STATUS_OK ?
        s->attempt(s->context, s->external) : YHM2712_ACMD_STATUS_NOT_INIT;
    /* Every attempt must leave transaction ownership and status input settled. */
    if (!yhm2712_acmd_converge_status_input(&recovery) &&
        s->last_status == YHM2712_ACMD_STATUS_OK)
        s->last_status = YHM2712_ACMD_STATUS_BUSY;
    now = (uint32_t)os_sys_time_get();
#if ZY100_YHM_HAS_MOS
    yhm_mos_error_counters_log("startup");
#endif
    DBG_DIRECT("[YHM_STARTUP][TRY] n=%lu status=%s vin=%u vinst=%u key=%u chg=%u",
        (unsigned long)s->attempts, yhm2712_acmd_status_name(s->last_status),
        s->external, vin_status, key, GPIO_ReadInputDataBit(GPIO_GetPin(P0_2)));
    if (s->last_status == YHM2712_ACMD_STATUS_OK) {
        s->phase = YHM_STARTUP_READY;
        DBG_DIRECT("[YHM_STARTUP][READY] ms=%lu attempts=%lu",
            (unsigned long)(now - s->started), (unsigned long)s->attempts);
        return true;
    }
    s->phase = s->attempts < 5U ? YHM_STARTUP_FAST_RETRY : YHM_STARTUP_SLOW_RETRY;
    s->due = now + (s->attempts < 5U ? 500U : 5000U);
    return false;
}

yhm2712_acmd_status_t svc_yhm_startup_read_only(void *context, bool external)
{
    yhm2712_acmd_boot_diag_t diag;
    yhm2712_acmd_status_t status;
    (void)context;
    if (!yhm2712_acmd_init(P0_2)) return YHM2712_ACMD_STATUS_BAD_PIN;
    status = yhm2712_acmd_select_external_power_profile(external);
    if (status != YHM2712_ACMD_STATUS_OK) return status;
    return yhm2712_acmd_read_boot_diag(&diag);
}
