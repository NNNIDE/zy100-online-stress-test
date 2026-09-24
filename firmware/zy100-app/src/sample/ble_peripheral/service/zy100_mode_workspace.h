#ifndef ZY100_MODE_WORKSPACE_H
#define ZY100_MODE_WORKSPACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define ZY100_MODE_WORKSPACE_BYTES (48UL * 1024UL)
#define ZY100_MODE_WORKSPACE_PERSISTENT_BYTES (8UL * 1024UL)
#define ZY100_MODE_WORKSPACE_TRANSIENT_BYTES \
    (ZY100_MODE_WORKSPACE_BYTES - ZY100_MODE_WORKSPACE_PERSISTENT_BYTES)

/* Online data only; control state stays outside the mode arena. */
#define ZY100_ONLINE_WORKSPACE_IMU_BYTES 31072UL
#define ZY100_ONLINE_WORKSPACE_MAG_BYTES 3328UL
#define ZY100_ONLINE_WORKSPACE_PAGE_OFFSET 34400UL
#define ZY100_ONLINE_WORKSPACE_VERIFY_OFFSET 34656UL
#define ZY100_ONLINE_WORKSPACE_TX_OFFSET 34912UL
#define ZY100_ONLINE_WORKSPACE_TX_BYTES 244U
#define ZY100_ONLINE_WORKSPACE_TX_SLOTS 8U
#define ZY100_ONLINE_WORKSPACE_REQUIRED_BYTES 36864UL

typedef enum
{
    ZY100_ONLINE_WORKSPACE_TX = 1U,
    ZY100_ONLINE_WORKSPACE_CAPTURE = 2U,
} zy100_online_workspace_client_t;

/* open reserves/clears once before START; join never clears live TX data. */
bool zy100_mode_workspace_online_open(uint32_t *token, uint8_t **base);
bool zy100_mode_workspace_online_join(uint32_t *token, uint8_t **base);
bool zy100_mode_workspace_online_release(uint32_t token,
                                         zy100_online_workspace_client_t client);

typedef enum
{
    ZY100_MODE_WORKSPACE_OWNER_NONE = 0U,
    ZY100_MODE_WORKSPACE_OWNER_ONLINE,
    ZY100_MODE_WORKSPACE_OWNER_OFFLINE_V2,
    ZY100_MODE_WORKSPACE_OWNER_LEGACY_OFFLINE,
    ZY100_MODE_WORKSPACE_OWNER_WHOLE_UNIT,
} zy100_mode_workspace_owner_t;

bool zy100_mode_workspace_claim(zy100_mode_workspace_owner_t owner,
                                uint32_t required_bytes,
                                uint8_t **base_out,
                                uint32_t *bytes_out);
bool zy100_mode_workspace_release(zy100_mode_workspace_owner_t owner);
zy100_mode_workspace_owner_t zy100_mode_workspace_owner(void);
bool zy100_mode_workspace_available(void);
uint8_t *zy100_mode_workspace_persistent_base(uint32_t *bytes_out);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_MODE_WORKSPACE_H */
