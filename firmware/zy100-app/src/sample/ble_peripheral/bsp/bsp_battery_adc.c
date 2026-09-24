#include "bsp_battery_adc.h"

#include <stddef.h>

#include "rtl876x_adc.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"

#include "../app_flags.h"
#include "imu_bsp.h"

#define BSP_BATTERY_ADC_PIN             P2_7
#define BSP_BATTERY_ADC_CHANNEL         ((uint8_t)(BSP_BATTERY_ADC_PIN - P2_0))
#define BSP_BATTERY_ADC_SCHEDULE_INDEX  0U
#define BSP_BATTERY_ADC_WAIT_LOOP_LIMIT 20000U
#define BSP_BATTERY_ADC_SAMPLE_TIME     255U

#ifndef BATTERY_ADC_CALIBRATION_INIT_ENABLE
#define BATTERY_ADC_CALIBRATION_INIT_ENABLE 1U
#endif

#if !BATTERY_ADC_USE_BYPASS_MODE || \
    (BATTERY_ADC_DIVIDER_R_TOP_OHM != 715000UL) || \
    (BATTERY_ADC_DIVIDER_R_BOTTOM_OHM != 160000UL)
#error "bsp battery ADC requires P2_7 bypass with the Production 715k/160k divider"
#endif

static bool s_bsp_battery_adc_initialized = false;
static bool s_bsp_battery_adc_calibration_ready = false;
static bool s_bsp_battery_adc_hw_active = false;

static bool bsp_battery_adc_wait_one_shot_done(void)
{
    uint32_t guard = BSP_BATTERY_ADC_WAIT_LOOP_LIMIT;

    while (guard > 0U)
    {
        if (ADC_GetINTStatus(ADC, ADC_INT_ONE_SHOT_DONE) == SET)
        {
            return true;
        }
        guard--;
    }

    return false;
}

bsp_battery_adc_status_t bsp_battery_adc_init(void)
{
    s_bsp_battery_adc_hw_active = false;
    s_bsp_battery_adc_calibration_ready = false;
    s_bsp_battery_adc_initialized = true;

#if BATTERY_ADC_CALIBRATION_INIT_ENABLE
    s_bsp_battery_adc_calibration_ready = ADC_CalibrationInit();
#endif

    bsp_battery_adc_park_low_power();
    return s_bsp_battery_adc_calibration_ready ?
           BSP_BATTERY_ADC_STATUS_OK :
           BSP_BATTERY_ADC_STATUS_CALIBRATION_UNAVAILABLE;
}

void bsp_battery_adc_prepare_runtime_pin(void)
{
    Pinmux_Deinit(BSP_BATTERY_ADC_PIN);
    System_WakeUpPinDisable(BSP_BATTERY_ADC_PIN);
    Pad_Config(BSP_BATTERY_ADC_PIN, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE,
               PAD_OUT_DISABLE, PAD_OUT_LOW);
}

void bsp_battery_adc_park_low_power(void)
{
    Pinmux_Deinit(BSP_BATTERY_ADC_PIN);
    System_WakeUpPinDisable(BSP_BATTERY_ADC_PIN);
    Pad_Config(BSP_BATTERY_ADC_PIN, PAD_SW_MODE, PAD_NOT_PWRON, PAD_PULL_NONE,
               PAD_OUT_DISABLE, PAD_OUT_LOW);
}

bsp_battery_adc_status_t bsp_battery_adc_hw_enable(void)
{
    ADC_InitTypeDef adc_init;

    if (!s_bsp_battery_adc_initialized)
    {
        return BSP_BATTERY_ADC_STATUS_NOT_INITIALIZED;
    }
    if (s_bsp_battery_adc_hw_active)
    {
        return BSP_BATTERY_ADC_STATUS_OK;
    }

#if ZY100_OFFLINE_V2_WOM_START_ENABLE
    /* ADC may run before the app has registered sensor restore callbacks.
     * This is a GPIO-only loan; never initialize IMU/MAG/Flash for ADC. */
    if (imu_bsp_power_mode() == IMU_BSP_POWER_DRIVE_LOW &&
        imu_bsp_power_ctrl(true) != IMU_STATUS_OK)
        return BSP_BATTERY_ADC_STATUS_NOT_ACTIVE;
#endif
    if (imu_bsp_power_access_begin() != IMU_STATUS_OK)
        return BSP_BATTERY_ADC_STATUS_NOT_ACTIVE;
    ADC_PowerSupplyConfig(ENABLE);
    RCC_PeriphClockCmd(APBPeriph_ADC, APBPeriph_ADC_CLOCK, ENABLE);

    ADC_StructInit(&adc_init);
    adc_init.ADC_SchIndex[BSP_BATTERY_ADC_SCHEDULE_INDEX] =
        EXT_SINGLE_ENDED(BSP_BATTERY_ADC_CHANNEL);
    adc_init.ADC_Bitmap = 0x01U;
    adc_init.ADC_DataWriteToFifo = ADC_DATA_WRITE_TO_FIFO_DISABLE;
    adc_init.ADC_TimerTriggerEn = ADC_TIMER_TRIGGER_DISABLE;
    adc_init.ADC_DataAvgEn = ADC_DATA_AVERAGE_DISABLE;
    adc_init.ADC_PowerAlwaysOnEn = ADC_POWER_ALWAYS_ON_ENABLE;
    adc_init.ADC_SampleTime = BSP_BATTERY_ADC_SAMPLE_TIME;

    ADC_Init(ADC, &adc_init);
    ADC_BypassCmd(BSP_BATTERY_ADC_CHANNEL, ENABLE);
    ADC_INTConfig(ADC, ADC_INT_ONE_SHOT_DONE, ENABLE);
    ADC_ClearINTPendingBit(ADC, ADC_INT_ONE_SHOT_DONE);
    s_bsp_battery_adc_hw_active = true;

    return BSP_BATTERY_ADC_STATUS_OK;
}

void bsp_battery_adc_hw_disable(void)
{
    if (!s_bsp_battery_adc_hw_active)
    {
        ADC_PowerSupplyConfig(DISABLE);
        RCC_PeriphClockCmd(APBPeriph_ADC, APBPeriph_ADC_CLOCK, DISABLE);
        return;
    }

    ADC_INTConfig(ADC, ADC_INT_ONE_SHOT_DONE, DISABLE);
    ADC_ClearINTPendingBit(ADC, ADC_INT_ONE_SHOT_DONE);
    ADC_Cmd(ADC, ADC_ONE_SHOT_MODE, DISABLE);
    ADC_ManualPowerOnCmd(ADC, DISABLE);
    ADC_PowerSupplyConfig(DISABLE);
    ADC_DeInit(ADC);
    RCC_PeriphClockCmd(APBPeriph_ADC, APBPeriph_ADC_CLOCK, DISABLE);
    if (s_bsp_battery_adc_hw_active) imu_bsp_power_access_end();
    s_bsp_battery_adc_hw_active = false;
}

bsp_battery_adc_status_t bsp_battery_adc_read_one_shot(uint16_t *raw_out)
{
    if (raw_out == NULL)
    {
        return BSP_BATTERY_ADC_STATUS_INVALID_PARAM;
    }
    *raw_out = 0U;

    if (!s_bsp_battery_adc_initialized)
    {
        return BSP_BATTERY_ADC_STATUS_NOT_INITIALIZED;
    }
    if (!s_bsp_battery_adc_hw_active)
    {
        return BSP_BATTERY_ADC_STATUS_NOT_ACTIVE;
    }

    ADC_ClearINTPendingBit(ADC, ADC_INT_ONE_SHOT_DONE);
    ADC_Cmd(ADC, ADC_ONE_SHOT_MODE, ENABLE);
    if (!bsp_battery_adc_wait_one_shot_done())
    {
        ADC_Cmd(ADC, ADC_ONE_SHOT_MODE, DISABLE);
        return BSP_BATTERY_ADC_STATUS_TIMEOUT;
    }

    *raw_out = ADC_ReadRawData(ADC, BSP_BATTERY_ADC_SCHEDULE_INDEX);
    ADC_ClearINTPendingBit(ADC, ADC_INT_ONE_SHOT_DONE);
    ADC_Cmd(ADC, ADC_ONE_SHOT_MODE, DISABLE);
    return (*raw_out <= BSP_BATTERY_ADC_RAW_MAX) ?
           BSP_BATTERY_ADC_STATUS_OK :
           BSP_BATTERY_ADC_STATUS_RAW_OUT_OF_RANGE;
}

bsp_battery_adc_status_t bsp_battery_adc_raw_to_vbat_mv(uint16_t raw,
                                                        uint16_t *mv_out)
{
    ADC_ErrorStatus error_status = NO_ERROR;
    float pin_mv;
    float vbat_mv;

    if (mv_out == NULL)
    {
        return BSP_BATTERY_ADC_STATUS_INVALID_PARAM;
    }
    *mv_out = 0U;

    if (raw > BSP_BATTERY_ADC_RAW_MAX)
    {
        return BSP_BATTERY_ADC_STATUS_RAW_OUT_OF_RANGE;
    }
    if (!s_bsp_battery_adc_calibration_ready)
    {
        return BSP_BATTERY_ADC_STATUS_CALIBRATION_UNAVAILABLE;
    }

    pin_mv = ADC_GetVoltage(BYPASS_SINGLE_MODE, (int32_t)raw, &error_status);
    if ((error_status != NO_ERROR) || !(pin_mv >= 0.0f))
    {
        return BSP_BATTERY_ADC_STATUS_CONVERSION_ERROR;
    }

    vbat_mv = pin_mv *
              ((float)(BATTERY_ADC_DIVIDER_R_TOP_OHM +
                       BATTERY_ADC_DIVIDER_R_BOTTOM_OHM) /
               (float)BATTERY_ADC_DIVIDER_R_BOTTOM_OHM);
    if (!(vbat_mv >= 0.0f) || (vbat_mv > 65535.0f))
    {
        return BSP_BATTERY_ADC_STATUS_CONVERSION_ERROR;
    }

    *mv_out = (uint16_t)(vbat_mv + 0.5f);
    return BSP_BATTERY_ADC_STATUS_OK;
}

void bsp_battery_adc_get_info(bsp_battery_adc_info_t *info)
{
    if (info == NULL)
    {
        return;
    }

    info->pin = BSP_BATTERY_ADC_PIN;
    info->channel = BSP_BATTERY_ADC_CHANNEL;
    info->bypass_mode = true;
    info->calibration_ready = s_bsp_battery_adc_calibration_ready;
    info->hw_active = s_bsp_battery_adc_hw_active;
}

const char *bsp_battery_adc_status_name(bsp_battery_adc_status_t status)
{
    switch (status)
    {
    case BSP_BATTERY_ADC_STATUS_OK:
        return "ok";
    case BSP_BATTERY_ADC_STATUS_INVALID_PARAM:
        return "invalid_param";
    case BSP_BATTERY_ADC_STATUS_NOT_INITIALIZED:
        return "not_initialized";
    case BSP_BATTERY_ADC_STATUS_NOT_ACTIVE:
        return "not_active";
    case BSP_BATTERY_ADC_STATUS_TIMEOUT:
        return "timeout";
    case BSP_BATTERY_ADC_STATUS_RAW_OUT_OF_RANGE:
        return "raw_out_of_range";
    case BSP_BATTERY_ADC_STATUS_CALIBRATION_UNAVAILABLE:
        return "calibration_unavailable";
    case BSP_BATTERY_ADC_STATUS_CONVERSION_ERROR:
        return "conversion_error";
    default:
        return "unknown";
    }
}
