#ifndef ZY100_OFFLINE_V2_JOURNAL_H
#define ZY100_OFFLINE_V2_JOURNAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define ZY100_OFFLINE_V2_JOURNAL_HEADER_BYTES 256U
/* Factory provisioning deliberately remains v4. */
#define ZY100_OFFLINE_V2_JOURNAL_HEADER_VERSION 4U
#define ZY100_OFFLINE_V2_JOURNAL_RUNTIME_VERSION 5U

typedef struct
{
    uint32_t generation;
    uint32_t next_session_id;
    uint32_t next_session_generation;
    uint32_t journal_sequence;
    uint32_t result_high_water;
    uint32_t protected_user_id;
    uint32_t foreign_purge_batch_token;
    uint32_t foreign_purge_total_sectors;
    uint32_t foreign_purge_erased_sectors;
} zy100_offline_v2_journal_header_fields_t;

bool zy100_offline_v2_journal_header_encode(
    uint8_t page[ZY100_OFFLINE_V2_JOURNAL_HEADER_BYTES],
    const zy100_offline_v2_journal_header_fields_t *fields);
bool zy100_offline_v2_journal_header_encode_v5(
    uint8_t page[ZY100_OFFLINE_V2_JOURNAL_HEADER_BYTES],
    const zy100_offline_v2_journal_header_fields_t *fields,
    uint32_t snapshot_count);
bool zy100_offline_v2_journal_header_valid(
    const uint8_t page[ZY100_OFFLINE_V2_JOURNAL_HEADER_BYTES],
    uint32_t *generation_out);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_OFFLINE_V2_JOURNAL_H */
