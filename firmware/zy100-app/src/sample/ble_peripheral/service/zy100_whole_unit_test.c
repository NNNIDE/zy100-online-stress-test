#include "zy100_whole_unit_test.h"

#include "../app_build_config.h"

#if ZY100_BUILD_PRODUCTION

#include <string.h>
#include <limits.h>

#include <gatt.h>
#include <os_sched.h>
#include <trace.h>

#include "../app_factory/factory_boot_gate.h"
#include "../app_task.h"
#include "../bsp/bsp_battery_adc.h"
#include "../bsp/bsp_led_power.h"
#include "../bsp/bsp_power_status.h"
#include "../bsp/imu_bsp.h"
#include "../bsp/imu_board_pinmap.h"
#include "../common/zy100_byteorder.h"
#include "../driver/gd25q32e_spi.h"
#include "../driver/icm53611_driver.h"
#include "../driver/icm53611_reg.h"
#include "../driver/mmc5603.h"
#include "rtl876x_gpio.h"
#include "../driver/mmc5603_reg.h"
#include "battery_adc.h"
#include "svc_led_pattern.h"
#include "svc_yhm2712_charge.h"
#include "zy100_crc32.h"
#include "zy100_mode_workspace.h"
#include "zy100_system_info_store.h"

#define WHOLE_VERSION                 2U
#define WHOLE_DATA_VERSION            1U
#define WHOLE_DATA_PAGE_VERSION       1U
#define WHOLE_OP_SESSION_BEGIN        0x01U
#define WHOLE_OP_ARM_ITEM             0x02U
#define WHOLE_OP_ITEM_FAIL            0x03U
#define WHOLE_OP_HOST_PASS            0x04U
#define WHOLE_OP_STOP                 0x05U
#define WHOLE_OP_REPORT_COMMIT_SHIP   0x06U
#define WHOLE_OP_DATA_PAGE_SELECT     0x07U
#define WHOLE_OP_POWER_STATE_QUERY    0x08U
#define WHOLE_EVENT_READY             0x80U
#define WHOLE_EVENT_ITEM_ARMED        0x81U
#define WHOLE_EVENT_ITEM_PASS         0x82U
#define WHOLE_EVENT_ITEM_FAIL         0x83U
#define WHOLE_EVENT_REPORT_COMMITTED  0x84U
#define WHOLE_EVENT_DATA_PAGE_READY   0x85U
#define WHOLE_EVENT_POWER_STATE       0x86U
#define WHOLE_EVENT_ERROR             0xE0U
#define WHOLE_STATUS_OK               0U
#define WHOLE_STATUS_BAD_FRAME        1U
#define WHOLE_STATUS_BAD_STATE        2U
#define WHOLE_STATUS_BAD_ITEM         3U
#define WHOLE_STATUS_TOKEN            4U
#define WHOLE_STATUS_DEPENDENCY       5U
#define WHOLE_STATUS_TEST             6U
#define WHOLE_STATUS_PERSIST          7U
#define WHOLE_STATUS_YHM              8U
#define WHOLE_CONTROL_INDEX           0x02U
#define WHOLE_STATUS_INDEX            0x04U
#define WHOLE_STATUS_CCCD_INDEX       0x05U
#define WHOLE_REPORT_INDEX            0x07U
#define WHOLE_DATA_INDEX              0x09U
#define WHOLE_CONN_INVALID            0xFFU
#define WHOLE_ITEM_MIN                1U
#define WHOLE_ITEM_MAX                17U
#define WHOLE_ADC_SAMPLES             10U
#define WHOLE_ADC_MIN_MV              3850U
#define WHOLE_ADC_MAX_MV              4350U
#define WHOLE_ADC_POWER_SETTLE_MS     3000ULL
#define WHOLE_ADC_SENSOR_SETTLE_MS    100ULL
#define WHOLE_ITEM_BATTERY_ADC        2U
#define WHOLE_SENSOR_SAMPLES          10U
#define WHOLE_SAMPLE_INTERVAL_MS      100ULL
#define WHOLE_IMU_COLLECTION_TIMEOUT_MS 3000ULL
#define WHOLE_FLASH_TEST_ADDR         0UL
#define WHOLE_FLASH_TEST_BYTES        GD25Q32E_PAGE_BYTES
#define WHOLE_FLASH_JEDEC             0xC84016UL
#define WHOLE_SHIP_NOTIFY_GRACE_MS    500ULL
#define WHOLE_SHIP_POWER_CUT_MS       2000ULL
#define FIRST_USER_COMPLETION_RETRY_MS 2000ULL
#define WHOLE_DATA_MAGIC              0x31445557UL /* WUD1 */
#define WHOLE_DATA_PAGE_FRAME_BYTES   20U
#define WHOLE_DATA_PAGE_PAYLOAD_BYTES 10U

typedef enum
{
    WHOLE_RUNTIME_IDLE = 0U,
    WHOLE_RUNTIME_RUNNING,
    WHOLE_RUNTIME_ADC_POWER_SETTLE,
    WHOLE_RUNTIME_ADC_SENSOR_SETTLE,
    WHOLE_RUNTIME_ADC_DISCARD,
    WHOLE_RUNTIME_ADC,
    WHOLE_RUNTIME_SENSOR,
    WHOLE_RUNTIME_SHIP_NOTIFY,
    WHOLE_RUNTIME_SHIP_EXECUTE,
    WHOLE_RUNTIME_SHIP_WAIT_CUT,
    WHOLE_RUNTIME_STOPPED,
} whole_runtime_state_t;

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t bytes;
    uint32_t session_token;
    uint32_t mfg_crc;
    uint32_t passed_mask;
    uint32_t failed_mask;
    uint16_t adc_mv[WHOLE_ADC_SAMPLES];
    int32_t mag_xyz[WHOLE_SENSOR_SAMPLES][3];
    int16_t accel_xyz[WHOLE_SENSOR_SAMPLES][3];
    int16_t gyro_xyz[WHOLE_SENSOR_SAMPLES][3];
    uint8_t adc_count;
    uint8_t mag_count;
    uint8_t accel_count;
    uint8_t gyro_count;
    uint8_t flash_jedec[3];
    uint8_t imu_id;
    uint8_t mag_id;
    uint8_t key_cycles;
    int8_t rssi_dbm;
    uint8_t reserved[3];
    uint32_t crc32;
} whole_data_v1_t;

static const uint8_t s_service_uuid[16] =
{
    0x00, 0x20, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
    0x4D, 0x4F, 0x9D, 0x4A, 0x0E, 0x00, 0xCA, 0x9E
};
static T_SERVER_ID s_service_id = 0xFFU;
static bool s_boot_active = false;
static bool s_first_user_boot_pending = false;
static bool s_first_user_advertised = false;
static uint64_t s_first_user_retry_ms = 0ULL;
static bool s_fail_closed = false;
static char s_ble_name[10] = "ZW-000000";
static whole_runtime_state_t s_runtime_state = WHOLE_RUNTIME_IDLE;
static uint8_t s_conn_id = WHOLE_CONN_INVALID;
static bool s_notify_enabled = false;
static bool s_status_pending = false;
static bool s_resume_ship_pending = false;
static uint8_t s_status[ZY100_WHOLE_UNIT_FRAME_BYTES];
static uint8_t s_report[ZY100_WHOLE_UNIT_FRAME_BYTES];
static uint8_t s_data_page[WHOLE_DATA_PAGE_FRAME_BYTES];
static whole_data_v1_t *s_data = NULL;
static uint8_t s_data_page_index = 0U;
static bool s_data_page_selected = false;
static uint32_t s_session_token = 0UL;
static uint32_t s_step_token = 0UL;
static uint8_t s_current_item = 0U;
static uint64_t s_item_started_ms = 0ULL;
static bool s_release_baseline = false;
static bool s_press_seen = false;
static uint8_t s_key_cycles = 0U;
static uint8_t s_sample_count = 0U;
static uint16_t s_imu_rejected = 0U;
static uint64_t s_imu_collection_deadline_ms = 0ULL;
static uint64_t s_next_sample_ms = 0ULL;
static uint64_t s_adc_phase_deadline_ms = 0ULL;
static battery_adc_test_summary_t s_adc_summary;
static uint64_t s_ship_deadline_ms = 0ULL;

static T_ATTRIB_APPL s_attr_tbl[] =
{
    { (ATTRIB_FLAG_VOID | ATTRIB_FLAG_LE),
      { LO_WORD(GATT_UUID_PRIMARY_SERVICE), HI_WORD(GATT_UUID_PRIMARY_SERVICE) },
      UUID_128BIT_SIZE, (void *)s_service_uuid, GATT_PERM_READ },
    { ATTRIB_FLAG_VALUE_INCL,
      { LO_WORD(GATT_UUID_CHARACTERISTIC), HI_WORD(GATT_UUID_CHARACTERISTIC),
        GATT_CHAR_PROP_WRITE }, 1, NULL, GATT_PERM_READ },
    { ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,
      { 0x00, 0x20, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
        0x4D, 0x4F, 0x9D, 0x4A, 0x0F, 0x00, 0xCA, 0x9E },
      0, NULL, GATT_PERM_WRITE },
    { ATTRIB_FLAG_VALUE_INCL,
      { LO_WORD(GATT_UUID_CHARACTERISTIC), HI_WORD(GATT_UUID_CHARACTERISTIC),
        (GATT_CHAR_PROP_READ | GATT_CHAR_PROP_NOTIFY) }, 1, NULL, GATT_PERM_READ },
    { ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,
      { 0x00, 0x20, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
        0x4D, 0x4F, 0x9D, 0x4A, 0x10, 0x00, 0xCA, 0x9E },
      0, NULL, GATT_PERM_READ },
    { ATTRIB_FLAG_VALUE_INCL | ATTRIB_FLAG_CCCD_APPL,
      { LO_WORD(GATT_UUID_CHAR_CLIENT_CONFIG), HI_WORD(GATT_UUID_CHAR_CLIENT_CONFIG),
        LO_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT),
        HI_WORD(GATT_CLIENT_CHAR_CONFIG_DEFAULT) },
      2, NULL, (GATT_PERM_READ | GATT_PERM_WRITE) },
    { ATTRIB_FLAG_VALUE_INCL,
      { LO_WORD(GATT_UUID_CHARACTERISTIC), HI_WORD(GATT_UUID_CHARACTERISTIC),
        GATT_CHAR_PROP_READ }, 1, NULL, GATT_PERM_READ },
    { ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,
      { 0x00, 0x20, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
        0x4D, 0x4F, 0x9D, 0x4A, 0x11, 0x00, 0xCA, 0x9E },
      0, NULL, GATT_PERM_READ },
    { ATTRIB_FLAG_VALUE_INCL,
      { LO_WORD(GATT_UUID_CHARACTERISTIC), HI_WORD(GATT_UUID_CHARACTERISTIC),
        GATT_CHAR_PROP_READ }, 1, NULL, GATT_PERM_READ },
    { ATTRIB_FLAG_VALUE_APPL | ATTRIB_FLAG_UUID_128BIT,
      { 0x00, 0x20, 0x9A, 0x5A, 0x0F, 0x7D, 0x1A, 0xB6,
        0x4D, 0x4F, 0x9D, 0x4A, 0x12, 0x00, 0xCA, 0x9E },
      0, NULL, GATT_PERM_READ },
};

static uint32_t whole_item_bit(uint8_t item)
{
    return ((item >= WHOLE_ITEM_MIN) && (item <= WHOLE_ITEM_MAX)) ?
           (1UL << (item - 1U)) : 0UL;
}

static void whole_led_off(const char *reason)
{
    svc_led_pattern_shutdown_for_sleep();
    ZY100_WHOLE_UNIT_LOG("[WHOLE][LED_PWR] off reason=%s",
               (reason != NULL) ? reason : "unknown");
}

static void whole_log_sensor_power(const char *stage, imu_status_t status)
{
#if ZY100_WHOLE_UNIT_LOG_ENABLE
    uint8_t out_level = 0xFFU;
    uint8_t in_level = 0xFFU;
    const uint32_t gpio_pin = GPIO_GetPin(IMU_POWER_CTRL_PIN);

    (void)imu_bsp_get_power_level(&out_level, &in_level);
    ZY100_WHOLE_UNIT_LOG("[WHOLE][SENSOR_PWR] stage=%s", stage);
    ZY100_WHOLE_UNIT_LOG("[WHOLE][SENSOR_PWR] pin=%u name=P1_1 active_high=%u",
               IMU_POWER_CTRL_PIN,
               (IMU_POWER_CTRL_ACTIVE_HIGH != 0U) ? 1U : 0U);
    ZY100_WHOLE_UNIT_LOG("[WHOLE][SENSOR_PWR] gpio_mask=0x%08lx",
               (unsigned long)gpio_pin);
    ZY100_WHOLE_UNIT_LOG("[WHOLE][SENSOR_PWR] configured_mode=PAD_SW_MODE");
    ZY100_WHOLE_UNIT_LOG("[WHOLE][SENSOR_PWR] configured_pull=PAD_PULL_UP");
    ZY100_WHOLE_UNIT_LOG("[WHOLE][SENSOR_PWR] configured_output=HIGH oe=PAD_OUT_ENABLE");
    ZY100_WHOLE_UNIT_LOG("[WHOLE][SENSOR_PWR] gpio_out=%u gpio_in=%u",
               out_level,
               in_level);
    ZY100_WHOLE_UNIT_LOG("[WHOLE][SENSOR_PWR] power_ctrl_status=%u",
               (unsigned)status);
#else
    (void)stage;
    (void)status;
#endif
}

static void whole_refresh_data_crc(void)
{
    if (s_data == NULL)
    {
        return;
    }
    s_data->passed_mask = s_data->passed_mask & ZY100_WHOLE_UNIT_REQUIRED_MASK;
    s_data->failed_mask = s_data->failed_mask & ZY100_WHOLE_UNIT_REQUIRED_MASK;
    s_data->crc32 = 0UL;
    s_data->crc32 = zy100_crc32_ieee((const uint8_t *)s_data,
                                    (uint32_t)sizeof(*s_data) - 4UL);
}

static uint8_t whole_data_page_count(void)
{
    return (uint8_t)(((uint16_t)sizeof(whole_data_v1_t) +
                      WHOLE_DATA_PAGE_PAYLOAD_BYTES - 1U) /
                     WHOLE_DATA_PAGE_PAYLOAD_BYTES);
}

static bool whole_build_data_page(uint8_t page_index)
{
    uint16_t offset;
    uint16_t remaining;
    uint8_t payload_len;

    if ((s_data == NULL) || (page_index >= whole_data_page_count()))
    {
        return false;
    }
    whole_refresh_data_crc();
    offset = (uint16_t)page_index * WHOLE_DATA_PAGE_PAYLOAD_BYTES;
    remaining = (uint16_t)sizeof(*s_data) - offset;
    payload_len = (remaining > WHOLE_DATA_PAGE_PAYLOAD_BYTES) ?
                  WHOLE_DATA_PAGE_PAYLOAD_BYTES : (uint8_t)remaining;
    memset(s_data_page, 0, sizeof(s_data_page));
    memcpy(s_data_page, "WUDP", 4U);
    s_data_page[4] = WHOLE_DATA_PAGE_VERSION;
    s_data_page[5] = page_index;
    s_data_page[6] = whole_data_page_count();
    s_data_page[7] = payload_len;
    zy100_put_u16_le(&s_data_page[8], (uint16_t)sizeof(*s_data));
    memcpy(&s_data_page[10], ((const uint8_t *)s_data) + offset,
           payload_len);
    return true;
}

static void whole_build_report(void)
{
    memset(s_report, 0, sizeof(s_report));
    memcpy(s_report, "WREP", 4U);
    s_report[4] = WHOLE_VERSION;
    s_report[5] = (uint8_t)s_runtime_state;
    s_report[6] = s_current_item;
    s_report[7] = (s_data != NULL) ? s_data->key_cycles : 0U;
    if (s_data != NULL)
    {
        zy100_put_u32_le(&s_report[8], s_data->passed_mask);
        zy100_put_u32_le(&s_report[12], s_data->failed_mask);
    }
    zy100_put_u32_le(&s_report[16], zy100_crc32_ieee(s_report, 16U));
}

static void whole_status(uint8_t event, uint8_t status, uint32_t detail)
{
    memset(s_status, 0, sizeof(s_status));
    memcpy(s_status, "WSTA", 4U);
    s_status[4] = WHOLE_VERSION;
    s_status[5] = event;
    s_status[6] = status;
    s_status[7] = s_current_item;
    zy100_put_u32_le(&s_status[8], s_step_token);
    zy100_put_u32_le(&s_status[12], detail);
    zy100_put_u32_le(&s_status[16], zy100_crc32_ieee(s_status, 16U));
    s_status_pending = true;
    whole_refresh_data_crc();
    whole_build_report();
}

static void whole_finish_item(bool pass, uint32_t detail)
{
    uint32_t bit = whole_item_bit(s_current_item);

    if ((s_data == NULL) || (bit == 0UL))
    {
        whole_status(WHOLE_EVENT_ERROR, WHOLE_STATUS_BAD_ITEM, s_current_item);
        return;
    }
    if (pass)
    {
        s_data->passed_mask |= bit;
        s_data->failed_mask &= ~bit;
    }
    else
    {
        s_data->failed_mask |= bit;
        s_data->passed_mask &= ~bit;
    }
    if (s_current_item != WHOLE_ITEM_BATTERY_ADC)
    {
        whole_led_off(pass ? "item_pass" : "item_fail");
    }
    s_runtime_state = WHOLE_RUNTIME_RUNNING;
    whole_status(pass ? WHOLE_EVENT_ITEM_PASS : WHOLE_EVENT_ITEM_FAIL,
                 pass ? WHOLE_STATUS_OK : WHOLE_STATUS_TEST,
                 detail);
}

static void whole_report_power_state(void)
{
    bool external_present = false;
    uint8_t power_state = 2U;

    if (bsp_power_status_chg_int_external_power_present(&external_present) ==
        BSP_POWER_STATUS_OK)
    {
        power_state = external_present ? 1U : 0U;
    }
    /* Keep the query non-destructive: it only reports CHG_INT and does not
     * alter the current item, session token, or any persistent state. */
    whole_status(WHOLE_EVENT_POWER_STATE, WHOLE_STATUS_OK, power_state);
}

static bool whole_flash_all_ff(uint32_t addr, uint32_t bytes, uint8_t *page)
{
    uint32_t offset;
    uint16_t idx;

    for (offset = 0UL; offset < bytes; offset += GD25Q32E_PAGE_BYTES)
    {
        if (gd25q32e_read(addr + offset, page, GD25Q32E_PAGE_BYTES) !=
            IMU_STATUS_OK)
        {
            return false;
        }
        for (idx = 0U; idx < GD25Q32E_PAGE_BYTES; idx++)
        {
            if (page[idx] != 0xFFU)
            {
                return false;
            }
        }
    }
    return true;
}

static bool whole_flash_test(void)
{
    uint8_t *page = (uint8_t *)(s_data + 1);
    uint16_t idx;
    bool verify_ok = false;
    bool erase_ok;

    if ((s_data == NULL) || (gd25q32e_init() != IMU_STATUS_OK) ||
        !whole_flash_all_ff(WHOLE_FLASH_TEST_ADDR,
                            GD25Q32E_SECTOR_BYTES, page))
    {
        ZY100_WHOLE_UNIT_LOG("[WHOLE][FLASH] refuse_write first_sector_not_blank=1");
        return false;
    }
    for (idx = 0U; idx < WHOLE_FLASH_TEST_BYTES; idx++)
    {
        page[idx] = (uint8_t)(0xA5U ^ (uint8_t)idx);
    }
    if ((gd25q32e_page_program_partial_erased(WHOLE_FLASH_TEST_ADDR,
                                               page,
                                               WHOLE_FLASH_TEST_BYTES) !=
         IMU_STATUS_OK) ||
        (gd25q32e_wait_while_busy(GD25Q32E_PAGE_PROGRAM_TIMEOUT_MS) !=
         IMU_STATUS_OK))
    {
        /* A program command may have partially changed the sector.  Always
         * execute the cleanup erase once writing was attempted. */
        verify_ok = false;
    }
    else
    {
        memset(page + GD25Q32E_PAGE_BYTES, 0, GD25Q32E_PAGE_BYTES);
        verify_ok =
            (gd25q32e_read(WHOLE_FLASH_TEST_ADDR,
                           page + GD25Q32E_PAGE_BYTES,
                           WHOLE_FLASH_TEST_BYTES) == IMU_STATUS_OK) &&
            (memcmp(page, page + GD25Q32E_PAGE_BYTES,
                    WHOLE_FLASH_TEST_BYTES) == 0);
    }
    erase_ok =
        (gd25q32e_sector_erase_4k(WHOLE_FLASH_TEST_ADDR) == IMU_STATUS_OK) &&
        (gd25q32e_wait_while_busy(GD25Q32E_SECTOR_ERASE_TIMEOUT_MS) ==
         IMU_STATUS_OK) &&
        whole_flash_all_ff(WHOLE_FLASH_TEST_ADDR,
                           GD25Q32E_SECTOR_BYTES, page);
    return verify_ok && erase_ok;
}

static bool whole_show_full_color(uint8_t red, uint8_t green, uint8_t blue)
{
    zy100_rgb_color_t frame[ZY100_RGB_LED_COUNT];
    uint8_t idx;

    for (idx = 0U; idx < ZY100_RGB_LED_COUNT; idx++)
    {
        frame[idx].red = red;
        frame[idx].green = green;
        frame[idx].blue = blue;
    }
    return svc_led_pattern_show_frame(frame, ZY100_RGB_LED_COUNT);
}

static bool whole_show_led(uint8_t item)
{
    if (item == 14U)
    {
        return whole_show_full_color(ZY100_LED_NOTIFY_BRIGHTNESS, 0U, 0U);
    }
    else if (item == 15U)
    {
        return whole_show_full_color(0U, ZY100_LED_NOTIFY_BRIGHTNESS, 0U);
    }
    else if (item == 16U)
    {
        return whole_show_full_color(0U, 0U, ZY100_LED_NOTIFY_BRIGHTNESS);
    }
    return whole_show_full_color(ZY100_LED_NOTIFY_BRIGHTNESS, 0U, 0U);
}

/* Match the Factory UI-sample policy; a stationary gyro alone may be zero. */
static bool whole_imu_sample_sentinel(const icm53611_raw_sample_t *sample)
{
    const bool all_zero =
        (sample->accel_x == 0) && (sample->accel_y == 0) &&
        (sample->accel_z == 0) && (sample->gyro_x == 0) &&
        (sample->gyro_y == 0) && (sample->gyro_z == 0);
    const bool all_max =
        (sample->accel_x == INT16_MAX) && (sample->accel_y == INT16_MAX) &&
        (sample->accel_z == INT16_MAX) && (sample->gyro_x == INT16_MAX) &&
        (sample->gyro_y == INT16_MAX) && (sample->gyro_z == INT16_MAX);
    const bool all_min =
        (sample->accel_x == INT16_MIN) && (sample->accel_y == INT16_MIN) &&
        (sample->accel_z == INT16_MIN) && (sample->gyro_x == INT16_MIN) &&
        (sample->gyro_y == INT16_MIN) && (sample->gyro_z == INT16_MIN);
    return all_zero || all_max || all_min;
}

static bool whole_prepare_sensor(uint8_t item, uint64_t runtime_ms)
{
    imu_status_t status;

    s_sample_count = 0U;
    s_imu_rejected = 0U;
    s_imu_collection_deadline_ms = 0ULL;
    s_next_sample_ms = runtime_ms;
    if (s_data == NULL)
    {
        whole_finish_item(false, 0UL);
        return false;
    }
    if (item == 10U)
    {
        s_data->mag_count = 0U;
        memset(s_data->mag_xyz, 0, sizeof(s_data->mag_xyz));
    }
    else
    {
        if (item == 11U)
        {
            s_data->accel_count = 0U;
            memset(s_data->accel_xyz, 0, sizeof(s_data->accel_xyz));
        }
        else
        {
            s_data->gyro_count = 0U;
            memset(s_data->gyro_xyz, 0, sizeof(s_data->gyro_xyz));
        }
        status = icm53611_bringup(icm53611_default_config());
        if (status != IMU_STATUS_OK)
        {
            ZY100_WHOLE_UNIT_LOG("[WHOLE][IMU] init_fail item=%u status=%u", item, status);
            whole_finish_item(false, 0UL);
            return false;
        }
        /* Same warm-up and bounded collection window as Factory P2. */
        s_next_sample_ms = runtime_ms + WHOLE_SAMPLE_INTERVAL_MS;
        s_imu_collection_deadline_ms =
            s_next_sample_ms + WHOLE_IMU_COLLECTION_TIMEOUT_MS;
        ZY100_WHOLE_UNIT_LOG("[WHOLE][IMU] ready item=%u warmup_ms=100 timeout_ms=3000", item);
    }
    s_runtime_state = WHOLE_RUNTIME_SENSOR;
    return true;
}

static void whole_arm_item(uint8_t item, uint32_t token, uint64_t runtime_ms)
{
    gd25q32e_jedec_id_t flash_id;
    yhm2712_acmd_status_t yhm_status;
    uint8_t id = 0U;
    bool external_present = false;

    s_current_item = item;
    s_item_started_ms = runtime_ms;
    s_step_token = token;
    s_release_baseline = false;
    s_press_seen = false;
    s_key_cycles = 0U;
    whole_led_off("arm_item");
    if ((item == 1U) &&
        ((bsp_power_status_chg_int_external_power_present(&external_present) !=
          BSP_POWER_STATUS_OK) || !external_present))
    {
        whole_finish_item(false, 1UL);
        return;
    }
    if (item == WHOLE_ITEM_BATTERY_ADC)
    {
        if ((bsp_power_status_chg_int_external_power_present(&external_present) !=
             BSP_POWER_STATUS_OK) || external_present || (s_data == NULL))
        {
            battery_adc_guard_session_end();
            whole_finish_item(false, 2UL);
            return;
        }

        /* W1 intentionally leaves the charger in its VIN/charge profile so
         * the operator can judge the charging current.  CHG_INT becoming
         * absent is only the electrical indication that VIN was removed; it
         * does not itself hand SYS back to the battery.  The normal
         * Production active-wake path already owns the verified YHM profile
         * transition, so use that public service API here instead of touching
         * charger registers from the whole-unit test.  This is the key
         * difference from the old W2 path, which sampled while the charger
         * still reported ac=1/tail=high and consequently saw ~4.58 V.
         */
        yhm_status = svc_yhm2712_charge_wake_for_active(
            false, "whole_w2_vin_removed");
        if (yhm_status != YHM2712_ACMD_STATUS_OK)
        {
            ZY100_WHOLE_UNIT_LOG("[WHOLE][ADC] battery_path_restore_fail status=%s",
                       yhm2712_acmd_status_name(yhm_status));
            whole_finish_item(false, (uint32_t)yhm_status);
            return;
        }
        ZY100_WHOLE_UNIT_LOG("[WHOLE][ADC] battery_path_restored mode=discharge");

        /* VIN/charger removal changes the analog operating point.  Do not
         * sample in the same tick as the third CHG_INT removal confirmation;
         * the Production ADC path already reserves a 3 s first-settle window. */
        s_adc_phase_deadline_ms = runtime_ms + WHOLE_ADC_POWER_SETTLE_MS;
        s_runtime_state = WHOLE_RUNTIME_ADC_POWER_SETTLE;
        ZY100_WHOLE_UNIT_LOG("[WHOLE][ADC] vin_removed_ms=%lu power_path_settle_ms=%lu",
                   (unsigned long)runtime_ms,
                   (unsigned long)WHOLE_ADC_POWER_SETTLE_MS);
        whole_status(WHOLE_EVENT_ITEM_ARMED, WHOLE_STATUS_OK, token);
        return;
    }
    else if (item == 6U)
    {
        memset(&flash_id, 0, sizeof(flash_id));
        if ((gd25q32e_init() == IMU_STATUS_OK) &&
            (gd25q32e_read_jedec_id(&flash_id) == IMU_STATUS_OK))
        {
            s_data->flash_jedec[0] = flash_id.manufacturer_id;
            s_data->flash_jedec[1] = flash_id.memory_type;
            s_data->flash_jedec[2] = flash_id.density;
            whole_finish_item(((((uint32_t)flash_id.manufacturer_id << 16U) |
                                ((uint32_t)flash_id.memory_type << 8U) |
                                flash_id.density) == WHOLE_FLASH_JEDEC),
                              WHOLE_FLASH_JEDEC);
        }
        else
        {
            whole_finish_item(false, 0UL);
        }
        return;
    }
    else if (item == 7U)
    {
        whole_finish_item(whole_flash_test(), WHOLE_FLASH_TEST_ADDR);
        return;
    }
    else if (item == 8U)
    {
        if ((icm53611_init_bus() == IMU_STATUS_OK) &&
            (icm53611_read_whoami(&id) == IMU_STATUS_OK))
        {
            s_data->imu_id = id;
            whole_finish_item(id == ICM53611_WHO_AM_I_VALUE, id);
        }
        else
        {
            whole_finish_item(false, id);
        }
        return;
    }
    else if (item == 9U)
    {
        if ((mmc5603_init(mmc5603_default_config()) == MAG_STATUS_OK) &&
            (mmc5603_read_product_id(&id) == MAG_STATUS_OK))
        {
            s_data->mag_id = id;
            whole_finish_item(id == MMC5603_PRODUCT_ID_VALUE, id);
        }
        else
        {
            whole_finish_item(false, id);
        }
        return;
    }
    else if ((item >= 10U) && (item <= 12U))
    {
        if (!whole_prepare_sensor(item, runtime_ms))
        {
            return;
        }
    }
    else if ((item >= 14U) && (item <= 17U))
    {
        if (!whole_show_led(item))
        {
            whole_finish_item(false, item);
            return;
        }
    }
    whole_status(WHOLE_EVENT_ITEM_ARMED, WHOLE_STATUS_OK, token);
}

static bool whole_command_valid(const uint8_t *data, uint16_t len)
{
    return (data != NULL) && (len == ZY100_WHOLE_UNIT_FRAME_BYTES) &&
           (memcmp(data, "WCTL", 4U) == 0) &&
           (data[4] == WHOLE_VERSION) &&
           (zy100_get_u32_le(&data[16]) == zy100_crc32_ieee(data, 16U));
}

static T_APP_RESULT whole_read_cb(uint8_t conn_id, T_SERVER_ID service_id,
                                  uint16_t attrib_index, uint16_t offset,
                                  uint16_t *length, uint8_t **value)
{
    uint8_t *source = NULL;
    uint16_t source_len = 0U;

    (void)conn_id;
    (void)service_id;
    if ((length == NULL) || (value == NULL))
    {
        return APP_RESULT_INVALID_VALUE_SIZE;
    }
    whole_refresh_data_crc();
    whole_build_report();
    if (attrib_index == WHOLE_STATUS_INDEX)
    {
        source = s_status;
        source_len = sizeof(s_status);
    }
    else if (attrib_index == WHOLE_REPORT_INDEX)
    {
        source = s_report;
        source_len = sizeof(s_report);
    }
    else if ((attrib_index == WHOLE_DATA_INDEX) && s_data_page_selected &&
             whole_build_data_page(s_data_page_index))
    {
        source = s_data_page;
        source_len = sizeof(s_data_page);
    }
    if ((source == NULL) || (offset > source_len))
    {
        return APP_RESULT_ATTR_NOT_FOUND;
    }
    *value = source + offset;
    *length = (uint16_t)(source_len - offset);
    return APP_RESULT_SUCCESS;
}

static T_APP_RESULT whole_write_cb(uint8_t conn_id, T_SERVER_ID service_id,
                                   uint16_t attrib_index,
                                   T_WRITE_TYPE write_type, uint16_t length,
                                   uint8_t *value,
                                   P_FUN_WRITE_IND_POST_PROC *post_proc)
{
    uint8_t op;
    uint8_t item;
    uint32_t token;
    uint32_t arg;
    uint8_t *workspace;
    uint32_t workspace_bytes;
    const zp_mfg_record_t *record = factory_boot_gate_record();

    (void)service_id;
    if (post_proc != NULL)
    {
        *post_proc = NULL;
    }
    if ((attrib_index != WHOLE_CONTROL_INDEX) ||
        (write_type != WRITE_REQUEST))
    {
        return APP_RESULT_ATTR_NOT_FOUND;
    }
    if (!whole_command_valid(value, length))
    {
        whole_status(WHOLE_EVENT_ERROR, WHOLE_STATUS_BAD_FRAME, length);
        return APP_RESULT_APP_ERR;
    }
    op = value[5];
    item = value[6];
    token = zy100_get_u32_le(&value[8]);
    arg = zy100_get_u32_le(&value[12]);
    s_conn_id = conn_id;

    if (op == WHOLE_OP_SESSION_BEGIN)
    {
        if (zy100_mode_workspace_owner() ==
            ZY100_MODE_WORKSPACE_OWNER_WHOLE_UNIT)
        {
            (void)zy100_mode_workspace_release(
                ZY100_MODE_WORKSPACE_OWNER_WHOLE_UNIT);
        }
        if (s_fail_closed || (record == NULL) || (token == 0UL) ||
            !zy100_mode_workspace_claim(ZY100_MODE_WORKSPACE_OWNER_WHOLE_UNIT,
                                        (uint32_t)sizeof(whole_data_v1_t) +
                                        (2UL * GD25Q32E_PAGE_BYTES),
                                        &workspace, &workspace_bytes))
        {
            whole_status(WHOLE_EVENT_ERROR, WHOLE_STATUS_BAD_STATE, 0UL);
            return APP_RESULT_APP_ERR;
        }
        (void)workspace_bytes;
        s_data = (whole_data_v1_t *)workspace;
        memset(s_data, 0, sizeof(*s_data));
        s_data->magic = WHOLE_DATA_MAGIC;
        s_data->version = WHOLE_DATA_VERSION;
        s_data->bytes = (uint16_t)sizeof(*s_data);
        s_data->session_token = token;
        s_data->mfg_crc = record->reserved_u32[0];
        s_session_token = token;
        s_runtime_state = WHOLE_RUNTIME_RUNNING;
        s_current_item = 0U;
        s_step_token = token;
        s_sample_count = 0U;
        s_data_page_selected = false;
        /* W3 advertising has already been observed to connect to this service. */
        s_data->passed_mask |= whole_item_bit(3U);
        if (s_conn_id != WHOLE_CONN_INVALID)
        {
            s_data->passed_mask |= whole_item_bit(4U);
        }
        if (s_resume_ship_pending)
        {
            /* The report was durably committed before the previous shipping
             * acknowledgement/physical disconnect was lost.  Reconstruct only
             * the final shipping gate; do not require or duplicate tests. */
            s_data->passed_mask = ZY100_WHOLE_UNIT_REQUIRED_MASK;
        }
        whole_status(WHOLE_EVENT_READY, WHOLE_STATUS_OK, record->reserved_u32[0]);
        return APP_RESULT_SUCCESS;
    }
    if ((s_data == NULL) || (s_runtime_state == WHOLE_RUNTIME_IDLE) ||
        (s_runtime_state == WHOLE_RUNTIME_STOPPED) ||
        (token != s_session_token))
    {
        whole_status(WHOLE_EVENT_ERROR, WHOLE_STATUS_TOKEN, token);
        return APP_RESULT_APP_ERR;
    }
    if (op == WHOLE_OP_POWER_STATE_QUERY)
    {
        if (item != 0U)
        {
            whole_status(WHOLE_EVENT_ERROR, WHOLE_STATUS_BAD_ITEM, item);
            return APP_RESULT_APP_ERR;
        }
        whole_report_power_state();
        return APP_RESULT_SUCCESS;
    }
    if (op == WHOLE_OP_ARM_ITEM)
    {
        if ((item < WHOLE_ITEM_MIN) || (item > WHOLE_ITEM_MAX) ||
            (s_runtime_state != WHOLE_RUNTIME_RUNNING))
        {
            whole_status(WHOLE_EVENT_ERROR, WHOLE_STATUS_BAD_ITEM, item);
            return APP_RESULT_APP_ERR;
        }
        whole_arm_item(item, arg, (uint64_t)os_sys_time_get());
        return APP_RESULT_SUCCESS;
    }
    if (op == WHOLE_OP_ITEM_FAIL)
    {
        if ((item != s_current_item) || (arg != s_step_token))
        {
            whole_status(WHOLE_EVENT_ERROR, WHOLE_STATUS_TOKEN, arg);
            return APP_RESULT_APP_ERR;
        }
        whole_finish_item(false, WHOLE_STATUS_TEST);
        return APP_RESULT_SUCCESS;
    }
    if (op == WHOLE_OP_HOST_PASS)
    {
        if ((item != 5U) || (s_current_item != 5U) ||
            (arg != s_step_token))
        {
            whole_status(WHOLE_EVENT_ERROR, WHOLE_STATUS_BAD_ITEM, item);
            return APP_RESULT_APP_ERR;
        }
        s_data->rssi_dbm = (int8_t)value[7];
        whole_finish_item(true, (uint32_t)(int32_t)s_data->rssi_dbm);
        return APP_RESULT_SUCCESS;
    }
    if (op == WHOLE_OP_STOP)
    {
        whole_led_off("stop");
        battery_adc_guard_session_end();
        s_runtime_state = WHOLE_RUNTIME_STOPPED;
        whole_status(WHOLE_EVENT_ERROR, WHOLE_STATUS_BAD_STATE, op);
        return APP_RESULT_SUCCESS;
    }
    if (op == WHOLE_OP_REPORT_COMMIT_SHIP)
    {
        if ((s_data->passed_mask != ZY100_WHOLE_UNIT_REQUIRED_MASK) ||
            (s_data->failed_mask != 0UL) ||
            (arg != s_data->mfg_crc) ||
            !zy100_system_info_set_manufacturing_states(
                ZY100_FACTORY_ACCEPTANCE_NONE,
                ZY100_WHOLE_UNIT_FINAL_SHIP_ARMED))
        {
            whole_status(WHOLE_EVENT_ERROR, WHOLE_STATUS_PERSIST,
                         s_data->passed_mask);
            return APP_RESULT_APP_ERR;
        }
        s_runtime_state = WHOLE_RUNTIME_SHIP_NOTIFY;
        s_ship_deadline_ms = 0ULL;
        whole_status(WHOLE_EVENT_REPORT_COMMITTED, WHOLE_STATUS_OK,
                     s_data->mfg_crc);
        return APP_RESULT_SUCCESS;
    }
    if (op == WHOLE_OP_DATA_PAGE_SELECT)
    {
        if ((arg > 0xFFUL) ||
            !whole_build_data_page((uint8_t)arg))
        {
            whole_status(WHOLE_EVENT_ERROR, WHOLE_STATUS_BAD_ITEM, arg);
            return APP_RESULT_APP_ERR;
        }
        s_data_page_index = (uint8_t)arg;
        s_data_page_selected = true;
        whole_status(WHOLE_EVENT_DATA_PAGE_READY, WHOLE_STATUS_OK, arg);
        return APP_RESULT_SUCCESS;
    }
    whole_status(WHOLE_EVENT_ERROR, WHOLE_STATUS_BAD_FRAME, op);
    return APP_RESULT_APP_ERR;
}

static void whole_cccd_cb(uint8_t conn_id, T_SERVER_ID service_id,
                          uint16_t index, uint16_t ccc_bits)
{
    (void)service_id;
    if (index != WHOLE_STATUS_CCCD_INDEX)
    {
        return;
    }
    s_conn_id = conn_id;
    s_notify_enabled =
        (ccc_bits & GATT_CLIENT_CHAR_CONFIG_NOTIFY) != 0U;
}

static const T_FUN_GATT_SERVICE_CBS s_callbacks =
{
    whole_read_cb, whole_write_cb, whole_cccd_cb
};

void zy100_whole_unit_test_boot_init(void)
{
    zy100_factory_acceptance_state_t acceptance;
    zy100_whole_unit_state_t whole_state;
    const zp_mfg_record_t *record = factory_boot_gate_record();
    size_t name_len;

    s_boot_active = false;
    s_first_user_boot_pending = false;
    s_first_user_advertised = false;
    s_first_user_retry_ms = 0ULL;
    s_fail_closed = false;
    s_resume_ship_pending = false;
    if (!factory_boot_gate_locked_user_mode_active() || (record == NULL))
    {
        return;
    }
    if (!zy100_system_info_get_manufacturing_states(&acceptance, &whole_state))
    {
        s_boot_active = true;
        s_fail_closed = true;
        ZY100_WHOLE_UNIT_LOG("[WHOLE][ERR] system_info_v6_invalid fail_closed=1");
        return;
    }
    if (whole_state == ZY100_WHOLE_UNIT_FINAL_SHIP_ARMED)
    {
        if (acceptance != ZY100_FACTORY_ACCEPTANCE_NONE)
        {
            s_boot_active = true;
            s_fail_closed = true;
            ZY100_WHOLE_UNIT_LOG("[WHOLE][ERR] final_ship_acceptance_invalid");
            return;
        }
        /* A new boot after the committed report is the user handoff, whether
         * powered by VIN or the charger's hardware button-wake path. This is
         * not proof of physical power-off for the station database. */
        s_first_user_boot_pending = true;
        DBG_DIRECT("[FIRST_USER] pending automatic_boot=1");
        return;
    }
    else if (whole_state != ZY100_WHOLE_UNIT_ACTIVE)
    {
        return;
    }
    name_len = strlen(record->ble_adv_name);
    if (name_len < 6U)
    {
        s_fail_closed = true;
        s_boot_active = true;
        return;
    }
    memcpy(s_ble_name, "ZW-", 3U);
    memcpy(&s_ble_name[3], &record->ble_adv_name[name_len - 6U], 6U);
    s_ble_name[9] = '\0';
    s_boot_active = true;
    /* Initialize the Production LED backend before W2.  Factory performs
     * this during task startup; without it the whole-unit path only toggled
     * the rail GPIO directly and could leave the LED data/power ownership in
     * a different state from the validated PCB ADC path. */
    if (!svc_led_pattern_init())
    {
        ZY100_WHOLE_UNIT_LOG("[WHOLE][WARN] led_backend_init_failed");
    }
    whole_led_off("boot");
    ZY100_WHOLE_UNIT_LOG("[WHOLE] boot active state=%u adv=%s pairing=disabled",
               (uint32_t)whole_state, s_ble_name);
}

bool zy100_whole_unit_first_user_boot_pending(void)
{
    return s_first_user_boot_pending;
}

void zy100_whole_unit_first_user_boot_note_advertising(void)
{
    if (s_first_user_boot_pending)
    {
        s_first_user_advertised = true;
    }
}

void zy100_whole_unit_first_user_boot_poll(uint64_t runtime_ms, bool ready)
{
    zy100_factory_acceptance_state_t acceptance;
    zy100_whole_unit_state_t whole;

    if (!s_first_user_boot_pending || !s_first_user_advertised ||
        !ready || (runtime_ms < s_first_user_retry_ms))
    {
        return;
    }
    /* Run in app task, never in a direct GAP callback. Keep the business gate
     * closed until both durable write and semantic readback have succeeded. */
    if (zy100_system_info_set_manufacturing_states(
            ZY100_FACTORY_ACCEPTANCE_NONE, ZY100_WHOLE_UNIT_COMPLETE) &&
        zy100_system_info_get_manufacturing_states(&acceptance, &whole) &&
        (acceptance == ZY100_FACTORY_ACCEPTANCE_NONE) &&
        (whole == ZY100_WHOLE_UNIT_COMPLETE))
    {
        s_first_user_boot_pending = false;
        DBG_DIRECT("[FIRST_USER] complete persisted=1 advertised=1");
        return;
    }
    s_first_user_retry_ms = runtime_ms + FIRST_USER_COMPLETION_RETRY_MS;
    DBG_DIRECT("[FIRST_USER][ERR] completion_pending business_blocked=1");
}

bool zy100_whole_unit_test_active(void)
{
    return s_boot_active;
}

const char *zy100_whole_unit_test_ble_name(void)
{
    return s_ble_name;
}

T_SERVER_ID zy100_whole_unit_test_add_service(void *callback)
{
    (void)callback;
    s_conn_id = WHOLE_CONN_INVALID;
    s_notify_enabled = false;
    s_status_pending = false;
    memset(s_status, 0, sizeof(s_status));
    memset(s_report, 0, sizeof(s_report));
    if (!server_add_service(&s_service_id, (uint8_t *)s_attr_tbl,
                            sizeof(s_attr_tbl), s_callbacks))
    {
        s_service_id = 0xFFU;
    }
    return s_service_id;
}

void zy100_whole_unit_test_on_connected(uint8_t conn_id)
{
    s_conn_id = conn_id;
    if (s_data != NULL)
    {
        s_data->passed_mask |= whole_item_bit(4U);
        whole_refresh_data_crc();
    }
}

void zy100_whole_unit_test_on_disconnected(uint8_t conn_id)
{
    if (s_conn_id == conn_id)
    {
        s_conn_id = WHOLE_CONN_INVALID;
        s_notify_enabled = false;
        s_status_pending = false;
        s_data_page_selected = false;
        whole_led_off("disconnect");
        battery_adc_guard_session_end();
    }
}

static void whole_poll_button(void)
{
    uint8_t level;
    bool pressed;
    uint8_t target_cycles;

    if ((s_current_item != 1U) && (s_current_item != 13U) &&
        ((s_current_item < 14U) || (s_current_item > 17U)))
    {
        return;
    }
    if (!app_button_read_level(&level))
    {
        return;
    }
    pressed = (level == (uint8_t)F_APP_BUTTON_ACTIVE_LEVEL);
    if (!s_release_baseline)
    {
        s_release_baseline = !pressed;
        return;
    }
    if (pressed)
    {
        s_press_seen = true;
        return;
    }
    if (!s_press_seen)
    {
        return;
    }
    s_press_seen = false;
    s_key_cycles++;
    target_cycles = (s_current_item == 13U) ? 3U : 1U;
    if (s_current_item == 13U)
    {
        /* Expose each completed press/release pair so the Host can show
         * 1/3, 2/3 and 3/3 progress without accepting a mouse PASS. */
        whole_status(WHOLE_EVENT_ITEM_ARMED, WHOLE_STATUS_OK, s_key_cycles);
    }
    if (s_key_cycles >= target_cycles)
    {
        if (s_data != NULL)
        {
            s_data->key_cycles = s_key_cycles;
        }
        whole_finish_item(true, s_key_cycles);
    }
}

static void whole_adc_fail(uint32_t detail)
{
    battery_adc_guard_session_end();
    bsp_battery_adc_hw_disable();
    bsp_battery_adc_park_low_power();
    whole_finish_item(false, detail);
}

/*
 * The whole-unit test runs inside the Production app task.  Production may
 * have an ADC guard/runtime attempt pending from the preceding power-state
 * transition, while the Factory test starts from a freshly-owned ADC block.
 * Explicitly terminate both ownership and the BSP hardware state here so the
 * W2 burst always starts from the same P2_7/ADC state as the PCB test.
 */
static void whole_adc_reset_hardware_for_test(void)
{
    battery_adc_guard_session_end();
    bsp_battery_adc_hw_disable();
    bsp_battery_adc_prepare_runtime_pin();
    ZY100_WHOLE_UNIT_LOG("[WHOLE][ADC] hardware_reset guard_end=1 hw_active=0 pin=P2_7");
}

static bool whole_adc_power_state_is_absent(void)
{
    bool external_present = false;

    return (bsp_power_status_chg_int_external_power_present(&external_present) ==
            BSP_POWER_STATUS_OK) && !external_present;
}

static void whole_poll_adc(uint64_t runtime_ms)
{
    uint8_t i;

    if (s_data == NULL)
    {
        whole_adc_fail(0UL);
        return;
    }

    if (s_runtime_state == WHOLE_RUNTIME_ADC_POWER_SETTLE)
    {
        bool power_absent;

        if (runtime_ms < s_adc_phase_deadline_ms)
        {
            return;
        }
        power_absent = whole_adc_power_state_is_absent();
        if (!power_absent)
        {
            ZY100_WHOLE_UNIT_LOG("[WHOLE][ADC] prepare_fail power_path_absent=%u sensor_pwr=unknown",
                       power_absent ? 1U : 0U);
            whole_adc_fail(0UL);
            return;
        }
        whole_adc_reset_hardware_for_test();
        {
            imu_status_t sensor_status = imu_bsp_power_ctrl(true);
            whole_log_sensor_power("before_adc", sensor_status);
            if (sensor_status != IMU_STATUS_OK)
            {
                ZY100_WHOLE_UNIT_LOG("[WHOLE][ADC] prepare_fail sensor_power_on=0");
                whole_adc_fail((uint32_t)sensor_status);
                return;
            }
        }
        ZY100_WHOLE_UNIT_LOG("[WHOLE][ADC] power_path_settle_done_ms=%lu sensor_power_on_ms=%lu",
                   (unsigned long)runtime_ms,
                   (unsigned long)runtime_ms);
        s_adc_phase_deadline_ms = runtime_ms + WHOLE_ADC_SENSOR_SETTLE_MS;
        s_runtime_state = WHOLE_RUNTIME_ADC_SENSOR_SETTLE;
        return;
    }

    if (s_runtime_state == WHOLE_RUNTIME_ADC_SENSOR_SETTLE)
    {
        if (runtime_ms < s_adc_phase_deadline_ms)
        {
            return;
        }
        s_runtime_state = WHOLE_RUNTIME_ADC_DISCARD;
        s_next_sample_ms = runtime_ms;
        return;
    }

    if (s_runtime_state == WHOLE_RUNTIME_ADC_DISCARD)
    {
        if (runtime_ms < s_next_sample_ms)
        {
            return;
        }
        memset(&s_adc_summary, 0, sizeof(s_adc_summary));
        if (!battery_adc_sample_test_sdk_summary(&s_adc_summary))
        {
            ZY100_WHOLE_UNIT_LOG("[WHOLE][ADC] discard_burst_fail sensor_pwr=held_high");
            whole_adc_fail(0UL);
            return;
        }
        ZY100_WHOLE_UNIT_LOG("[WHOLE][ADC] discard_burst raw_avg=%u raw_min=%u raw_max=%u "
                   "spread=%u sdk_mv=%u sensor_pwr=held_high",
                   s_adc_summary.raw_avg,
                   s_adc_summary.raw_min,
                   s_adc_summary.raw_max,
                   s_adc_summary.raw_spread,
                   s_adc_summary.sdk_mv);
        s_next_sample_ms = runtime_ms + WHOLE_SAMPLE_INTERVAL_MS;
        s_runtime_state = WHOLE_RUNTIME_ADC;
        return;
    }

    if ((s_runtime_state != WHOLE_RUNTIME_ADC) ||
        (runtime_ms < s_next_sample_ms))
    {
        return;
    }

    memset(&s_adc_summary, 0, sizeof(s_adc_summary));
    if (!battery_adc_sample_test_sdk_summary(&s_adc_summary))
    {
        ZY100_WHOLE_UNIT_LOG("[WHOLE][ADC] final_burst_fail sensor_pwr=held_high");
        whole_adc_fail(0UL);
        return;
    }

    for (i = 0U; i < WHOLE_ADC_SAMPLES; i++)
    {
        s_data->adc_mv[i] = s_adc_summary.sdk_mv;
    }
    s_data->adc_count = WHOLE_ADC_SAMPLES;
    ZY100_WHOLE_UNIT_LOG("[WHOLE][ADC] final_burst sample_count=%u raw_avg=%u raw_min=%u "
               "raw_max=%u spread=%u sdk_mv=%u sensor_pwr=held_high",
               s_adc_summary.sample_count,
               s_adc_summary.raw_avg,
               s_adc_summary.raw_min,
               s_adc_summary.raw_max,
               s_adc_summary.raw_spread,
               s_adc_summary.sdk_mv);
    whole_log_sensor_power("after_adc",
                           IMU_STATUS_OK);
    ZY100_WHOLE_UNIT_LOG("[WHOLE][ADC] done sdk_mv=%u sensor_pwr=held_high result=%s",
               s_adc_summary.sdk_mv,
               ((s_adc_summary.sdk_mv >= WHOLE_ADC_MIN_MV) &&
                (s_adc_summary.sdk_mv <= WHOLE_ADC_MAX_MV)) ? "PASS" : "FAIL");
    whole_finish_item((s_adc_summary.sdk_mv >= WHOLE_ADC_MIN_MV) &&
                      (s_adc_summary.sdk_mv <= WHOLE_ADC_MAX_MV),
                      s_adc_summary.sdk_mv);
}

static void whole_poll_sensor(uint64_t runtime_ms)
{
    mmc5603_sample_t mag;
    icm53611_raw_sample_t imu;

    if ((runtime_ms < s_next_sample_ms) || (s_data == NULL))
    {
        return;
    }
    if ((s_current_item != 10U) &&
        (runtime_ms > s_imu_collection_deadline_ms))
    {
        ZY100_WHOLE_UNIT_LOG("[WHOLE][IMU] timeout item=%u valid=%u rejected=%u",
                           s_current_item, s_sample_count, s_imu_rejected);
        whole_finish_item(false, s_sample_count);
        return;
    }
    memset(&mag, 0, sizeof(mag));
    memset(&imu, 0, sizeof(imu));
    if (s_current_item == 10U)
    {
        if (mmc5603_take_measurement(false, &mag) != MAG_STATUS_OK)
        {
            whole_finish_item(false, s_sample_count);
            return;
        }
        s_data->mag_xyz[s_sample_count][0] = (int32_t)mag.raw_x;
        s_data->mag_xyz[s_sample_count][1] = (int32_t)mag.raw_y;
        s_data->mag_xyz[s_sample_count][2] = (int32_t)mag.raw_z;
        s_data->mag_count = (uint8_t)(s_sample_count + 1U);
    }
    else
    {
        if (icm53611_read_ui_6axis_sample(&imu) != IMU_STATUS_OK)
        {
            whole_finish_item(false, s_sample_count);
            return;
        }
        if (whole_imu_sample_sentinel(&imu))
        {
            s_imu_rejected++;
            s_next_sample_ms = runtime_ms + WHOLE_SAMPLE_INTERVAL_MS;
            if (runtime_ms >= s_imu_collection_deadline_ms)
            {
                ZY100_WHOLE_UNIT_LOG("[WHOLE][IMU] invalid_timeout item=%u valid=%u rejected=%u",
                                   s_current_item, s_sample_count, s_imu_rejected);
                whole_finish_item(false, s_sample_count);
            }
            return;
        }
        if (s_current_item == 11U)
        {
            s_data->accel_xyz[s_sample_count][0] = imu.accel_x;
            s_data->accel_xyz[s_sample_count][1] = imu.accel_y;
            s_data->accel_xyz[s_sample_count][2] = imu.accel_z;
            s_data->accel_count = (uint8_t)(s_sample_count + 1U);
        }
        else
        {
            s_data->gyro_xyz[s_sample_count][0] = imu.gyro_x;
            s_data->gyro_xyz[s_sample_count][1] = imu.gyro_y;
            s_data->gyro_xyz[s_sample_count][2] = imu.gyro_z;
            s_data->gyro_count = (uint8_t)(s_sample_count + 1U);
        }
    }
    s_sample_count++;
    if (s_sample_count >= WHOLE_SENSOR_SAMPLES)
    {
        whole_finish_item(true, s_sample_count);
        return;
    }
    s_next_sample_ms = runtime_ms + WHOLE_SAMPLE_INTERVAL_MS;
}

void zy100_whole_unit_test_poll(uint64_t runtime_ms)
{
    yhm2712_acmd_status_t ship_status;

    if (s_status_pending && s_notify_enabled &&
        server_send_data(s_conn_id, s_service_id, WHOLE_STATUS_INDEX,
                         s_status, sizeof(s_status),
                         GATT_PDU_TYPE_NOTIFICATION))
    {
        s_status_pending = false;
        if (s_runtime_state == WHOLE_RUNTIME_SHIP_NOTIFY)
        {
            s_ship_deadline_ms = runtime_ms + WHOLE_SHIP_NOTIFY_GRACE_MS;
        }
    }
    if ((s_runtime_state == WHOLE_RUNTIME_ADC_POWER_SETTLE) ||
        (s_runtime_state == WHOLE_RUNTIME_ADC_SENSOR_SETTLE) ||
        (s_runtime_state == WHOLE_RUNTIME_ADC_DISCARD) ||
        (s_runtime_state == WHOLE_RUNTIME_ADC))
    {
        whole_poll_adc(runtime_ms);
    }
    else if (s_runtime_state == WHOLE_RUNTIME_SENSOR)
    {
        whole_poll_sensor(runtime_ms);
    }
    else if (s_runtime_state == WHOLE_RUNTIME_RUNNING)
    {
        if (s_current_item == 17U)
        {
            uint8_t phase = (uint8_t)(((runtime_ms - s_item_started_ms) /
                                       400ULL) % 3ULL);
            if (phase == 0U)
            {
                (void)whole_show_full_color(ZY100_LED_NOTIFY_BRIGHTNESS, 0U, 0U);
            }
            else if (phase == 1U)
            {
                (void)whole_show_full_color(0U, ZY100_LED_NOTIFY_BRIGHTNESS, 0U);
            }
            else
            {
                (void)whole_show_full_color(0U, 0U, ZY100_LED_NOTIFY_BRIGHTNESS);
            }
        }
        whole_poll_button();
    }
    if ((s_runtime_state == WHOLE_RUNTIME_SHIP_NOTIFY) &&
        !s_status_pending && (s_ship_deadline_ms != 0ULL) &&
        (runtime_ms >= s_ship_deadline_ms))
    {
        s_runtime_state = WHOLE_RUNTIME_SHIP_EXECUTE;
    }
    if (s_runtime_state == WHOLE_RUNTIME_SHIP_EXECUTE)
    {
        whole_led_off("final_shipping");
        ship_status = svc_yhm2712_charge_factory_enter_shipping();
        if (ship_status != YHM2712_ACMD_STATUS_OK)
        {
            (void)zy100_system_info_set_whole_unit_state(
                ZY100_WHOLE_UNIT_ACTIVE);
            s_runtime_state = WHOLE_RUNTIME_RUNNING;
            whole_status(WHOLE_EVENT_ERROR, WHOLE_STATUS_YHM,
                         (uint32_t)ship_status);
            return;
        }
        s_runtime_state = WHOLE_RUNTIME_SHIP_WAIT_CUT;
        s_ship_deadline_ms = runtime_ms + WHOLE_SHIP_POWER_CUT_MS;
    }
    if ((s_runtime_state == WHOLE_RUNTIME_SHIP_WAIT_CUT) &&
        (runtime_ms >= s_ship_deadline_ms))
    {
        (void)zy100_system_info_set_whole_unit_state(ZY100_WHOLE_UNIT_ACTIVE);
        s_runtime_state = WHOLE_RUNTIME_RUNNING;
        whole_status(WHOLE_EVENT_ERROR, WHOLE_STATUS_YHM, 0UL);
    }
}

#else

void zy100_whole_unit_test_boot_init(void) {}
bool zy100_whole_unit_test_active(void) { return false; }
bool zy100_whole_unit_first_user_boot_pending(void) { return false; }
void zy100_whole_unit_first_user_boot_note_advertising(void) {}
void zy100_whole_unit_first_user_boot_poll(uint64_t ms, bool ready)
{ (void)ms; (void)ready; }
const char *zy100_whole_unit_test_ble_name(void) { return ""; }
T_SERVER_ID zy100_whole_unit_test_add_service(void *callback)
{ (void)callback; return 0xFFU; }
void zy100_whole_unit_test_poll(uint64_t runtime_ms) { (void)runtime_ms; }
void zy100_whole_unit_test_on_connected(uint8_t conn_id) { (void)conn_id; }
void zy100_whole_unit_test_on_disconnected(uint8_t conn_id) { (void)conn_id; }

#endif
