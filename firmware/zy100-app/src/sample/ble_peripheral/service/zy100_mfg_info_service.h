#ifndef ZY100_MFG_INFO_SERVICE_H
#define ZY100_MFG_INFO_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <profile_server.h>

T_SERVER_ID zy100_mfg_info_service_add(void *app_profile_callback);
void zy100_mfg_info_service_reset(uint8_t conn_id);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_MFG_INFO_SERVICE_H */
