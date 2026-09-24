#ifndef ZY100_WHOLE_UNIT_TEST_H
#define ZY100_WHOLE_UNIT_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include <profile_server.h>

#define ZY100_WHOLE_UNIT_FRAME_BYTES 20U
#define ZY100_WHOLE_UNIT_REQUIRED_MASK 0x0001FFFFUL

void zy100_whole_unit_test_boot_init(void);
bool zy100_whole_unit_test_active(void);
/* RAM context is created only by boot_init, never by a shipping request. */
bool zy100_whole_unit_first_user_boot_pending(void);
void zy100_whole_unit_first_user_boot_note_advertising(void);
void zy100_whole_unit_first_user_boot_poll(uint64_t runtime_ms, bool ready);
const char *zy100_whole_unit_test_ble_name(void);
T_SERVER_ID zy100_whole_unit_test_add_service(void *callback);
void zy100_whole_unit_test_poll(uint64_t runtime_ms);
void zy100_whole_unit_test_on_connected(uint8_t conn_id);
void zy100_whole_unit_test_on_disconnected(uint8_t conn_id);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_WHOLE_UNIT_TEST_H */
