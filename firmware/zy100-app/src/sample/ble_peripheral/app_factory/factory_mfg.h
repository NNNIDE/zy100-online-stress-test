#ifndef FACTORY_MFG_H
#define FACTORY_MFG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../app_build_config.h"

#include "../app_mfg/zp_mfg_record.h"

#define FACTORY_MFG_RESULT_MAX_BYTES        20U
#define FACTORY_MFG_SAFE_READ_MAX_BYTES       240U
#define FACTORY_MFG_REPORT_TEXT_MAX_BYTES     240U
#define FACTORY_MFG_EXT_INFO_TEXT_MAX_BYTES   240U
#define FACTORY_MFG_INFO_TEXT_MAX_BYTES       240U
#define FACTORY_MFG_FUNCTIONAL_DATA_MAX_BYTES 184U
#define FACTORY_CONTROL_FRAME_BYTES          20U
#define FACTORY_CONTROL_STATUS_MAX_BYTES     20U
#define FACTORY_CONTROL_PROTOCOL_VERSION     9U

typedef enum
{
    FACTORY_CONTROL_RESULT_OK = 0U,
    FACTORY_CONTROL_RESULT_BAD_FRAME,
    FACTORY_CONTROL_RESULT_BAD_SESSION,
    FACTORY_CONTROL_RESULT_BAD_SEQUENCE,
    FACTORY_CONTROL_RESULT_FAILED,
} factory_control_result_t;

typedef enum
{
    FACTORY_MFG_STATUS_OK = 0U,
    FACTORY_MFG_STATUS_ERR_NOT_FACTORY,
    FACTORY_MFG_STATUS_ERR_ALREADY_LOCKED,
    FACTORY_MFG_STATUS_ERR_TESTS_NOT_PASS,
    FACTORY_MFG_STATUS_ERR_PROTO,
    FACTORY_MFG_STATUS_ERR_OVERLAP,
    FACTORY_MFG_STATUS_ERR_RANGE,
    FACTORY_MFG_STATUS_ERR_CRC,
    FACTORY_MFG_STATUS_ERR_FLASH,
    FACTORY_MFG_STATUS_ERR_BT_ADDR,
    FACTORY_MFG_STATUS_ERR_UID,
    FACTORY_MFG_STATUS_FAIL_UNSAFE,
    FACTORY_MFG_STATUS_ERR_RECORD_INVALID_LOCKED,
    FACTORY_MFG_STATUS_ERR_IMU_CAL_MISSING,
    FACTORY_MFG_STATUS_ERR_IMU_CAL_CHARACTERIZATION,
} factory_mfg_status_t;

typedef enum
{
    FACTORY_MFG_TEST_RUN_BUSY = 0U,
    FACTORY_MFG_TEST_RUN_PASS,
    FACTORY_MFG_TEST_RUN_FAIL,
} factory_mfg_test_run_status_t;

void factory_mfg_session_begin(void);
bool factory_mfg_led_showcase_tick(uint64_t runtime_ms);
void factory_mfg_mark_led_showcase_pass(void);
void factory_mfg_mark_factory_cleanup_pass(uint8_t legacy_cleanup_suppressed);
void factory_mfg_mark_factory_cleanup_fail(void);
void factory_mfg_mark_factory_cleanup_fail_detail(uint32_t detail);
void factory_mfg_mark_key_pass(void);
void factory_mfg_mark_ble_adv_pass(void);
void factory_mfg_mark_ble_adv_fail(void);
void factory_mfg_mark_ble_connected(uint8_t conn_id);
void factory_mfg_on_ble_disconnected(uint8_t conn_id);
factory_mfg_test_run_status_t factory_mfg_required_tests_poll(
    uint64_t runtime_ms);
bool factory_mfg_required_tests_passed(void);
uint32_t factory_mfg_passed_test_mask(void);
uint32_t factory_mfg_failed_test_mask(void);
bool factory_mfg_factory_cleanup_passed(void);
bool factory_mfg_dlps_blocked(void);
void factory_mfg_poll(uint64_t runtime_ms);
void factory_mfg_control_poll(uint64_t runtime_ms, bool button_pressed);
factory_control_result_t factory_mfg_on_control_frame(uint8_t conn_id,
                                                      const uint8_t *data,
                                                      uint16_t len);
bool factory_mfg_copy_control_status(uint8_t *out,
                                     uint16_t out_len,
                                     uint16_t *copied_len);
bool factory_mfg_get_adv_confirmation(uint8_t *mode_out,
                                      uint8_t *item_out,
                                      uint8_t *state_out,
                                      uint32_t *token_out);

factory_mfg_status_t factory_mfg_on_write_chunk(uint8_t conn_id,
                                                const uint8_t *data,
                                                uint16_t len);
uint16_t factory_mfg_build_info_text(uint8_t *out, uint16_t out_len);
uint16_t factory_mfg_build_report_text(uint8_t *out, uint16_t out_len);
uint16_t factory_mfg_build_ext_info_text(uint8_t *out, uint16_t out_len);
uint16_t factory_mfg_build_functional_data(uint8_t *out, uint16_t out_len);
bool factory_mfg_copy_last_result(uint8_t *out,
                                  uint16_t out_len,
                                  uint16_t *copied_len);
const char *factory_mfg_status_name(factory_mfg_status_t status);

#if ZY100_BUILD_PRODUCTION
#define factory_mfg_session_begin() ((void)0)
#define factory_mfg_led_showcase_tick(runtime_ms) (true)
#define factory_mfg_mark_led_showcase_pass() ((void)0)
#define factory_mfg_mark_factory_cleanup_pass(suppressed) ((void)0)
#define factory_mfg_mark_factory_cleanup_fail() ((void)0)
#define factory_mfg_mark_factory_cleanup_fail_detail(detail) ((void)0)
#define factory_mfg_mark_key_pass() ((void)0)
#define factory_mfg_mark_ble_adv_pass() ((void)0)
#define factory_mfg_mark_ble_adv_fail() ((void)0)
#define factory_mfg_mark_ble_connected(conn_id) ((void)0)
#define factory_mfg_on_ble_disconnected(conn_id) ((void)0)
#define factory_mfg_required_tests_poll(runtime_ms) FACTORY_MFG_TEST_RUN_FAIL
#define factory_mfg_required_tests_passed() (false)
#define factory_mfg_passed_test_mask() (0UL)
#define factory_mfg_failed_test_mask() (0UL)
#define factory_mfg_factory_cleanup_passed() (false)
#define factory_mfg_dlps_blocked() (false)
#define factory_mfg_poll(runtime_ms) ((void)0)
#define factory_mfg_control_poll(runtime_ms, button_pressed) ((void)0)
#define factory_mfg_on_control_frame(conn_id, data, len) \
    FACTORY_CONTROL_RESULT_BAD_FRAME
#define factory_mfg_copy_control_status(out, out_len, copied_len) (false)
#endif

#ifdef __cplusplus
}
#endif

#endif /* FACTORY_MFG_H */
