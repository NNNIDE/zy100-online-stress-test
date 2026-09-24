#ifndef ZY100_OFFLINE_V2_MARKER_H
#define ZY100_OFFLINE_V2_MARKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../common/zy100_internal_flash_layout.h"
#define ZY100_OFFLINE_V2_FACTORY_HANDOFF_NONE    0x00000000UL
#define ZY100_OFFLINE_V2_FACTORY_HANDOFF_PENDING 0x31484646UL

typedef enum
{
    ZY100_OFFLINE_V2_MARKER_NONE = 0U,
    ZY100_OFFLINE_V2_MARKER_MIGRATING = 1U,
    ZY100_OFFLINE_V2_MARKER_READY = 2U,
    ZY100_OFFLINE_V2_MARKER_ERROR = 3U,
    ZY100_OFFLINE_V2_MARKER_CLEARING = 4U,
} zy100_offline_v2_marker_state_t;

typedef struct
{
    uint32_t generation;
    zy100_offline_v2_marker_state_t state;
    uint32_t device_id;
    uint32_t layout_crc32;
    uint32_t online_spool_crc32;
    uint32_t next_erase_addr;
    uint32_t factory_handoff;
} zy100_offline_v2_marker_t;

typedef enum
{
    ZY100_OFFLINE_V2_HANDOFF_NOT_PENDING = 0U,
    ZY100_OFFLINE_V2_HANDOFF_CONSUMED,
    ZY100_OFFLINE_V2_HANDOFF_ERROR,
} zy100_offline_v2_handoff_consume_status_t;

bool zy100_offline_v2_marker_load(zy100_offline_v2_marker_t *marker_out);
bool zy100_offline_v2_marker_commit(
    const zy100_offline_v2_marker_t *marker);
bool zy100_offline_v2_marker_commit_fresh_ready(
    const zy100_offline_v2_marker_t *marker);
bool zy100_offline_v2_marker_factory_handoff_pending(
    const zy100_offline_v2_marker_t *marker);
zy100_offline_v2_handoff_consume_status_t
zy100_offline_v2_marker_consume_factory_handoff(
    zy100_offline_v2_marker_t *marker);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_OFFLINE_V2_MARKER_H */
