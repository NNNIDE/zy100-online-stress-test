#ifndef ZY100_PRODUCTION_ACCEPTANCE_H
#define ZY100_PRODUCTION_ACCEPTANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include <profile_server.h>

#define ZY100_PRODUCTION_ACCEPTANCE_FRAME_BYTES 20U

void zy100_production_acceptance_boot_init(void);
bool zy100_production_acceptance_active(void);
const char *zy100_production_acceptance_ble_name(void);
T_SERVER_ID zy100_production_acceptance_add_service(void *callback);
void zy100_production_acceptance_poll(uint64_t runtime_ms);
void zy100_production_acceptance_on_disconnected(uint8_t conn_id);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_PRODUCTION_ACCEPTANCE_H */
