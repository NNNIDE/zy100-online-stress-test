#include "zy100_offline_v2_journal.h"

#include <string.h>

#include "../common/zy100_byteorder.h"
#include "../common/zy100_offline_v2_contract.h"
#include "../service/zy100_crc32.h"

#define OFFLINE_V2_JOURNAL_MAGIC          0x324A464FUL
#define OFFLINE_V2_JOURNAL_HEADER_COMMIT  0x4A4E4C31UL
#define OFFLINE_V2_JOURNAL_CRC_OFFSET     0xF8U
#define OFFLINE_V2_JOURNAL_COMMIT_OFFSET  0xFCU

bool zy100_offline_v2_journal_header_encode(
    uint8_t page[ZY100_OFFLINE_V2_JOURNAL_HEADER_BYTES],
    const zy100_offline_v2_journal_header_fields_t *fields)
{
    uint32_t crc;

    if ((page == NULL) || (fields == NULL) ||
        (fields->generation == 0U) ||
        (fields->next_session_id == 0U) ||
        (fields->next_session_generation == 0U) ||
        (fields->journal_sequence == 0U))
    {
        return false;
    }

    memset(page, 0xFF, ZY100_OFFLINE_V2_JOURNAL_HEADER_BYTES);
    zy100_put_u32_le(&page[0x00], OFFLINE_V2_JOURNAL_MAGIC);
    zy100_put_u16_le(&page[0x04],
                     ZY100_OFFLINE_V2_JOURNAL_HEADER_VERSION);
    zy100_put_u16_le(&page[0x06],
                     ZY100_OFFLINE_V2_JOURNAL_HEADER_BYTES);
    zy100_put_u32_le(&page[0x08], fields->generation);
    zy100_put_u32_le(&page[0x0C], fields->next_session_id);
    zy100_put_u32_le(&page[0x10], fields->next_session_generation);
    zy100_put_u32_le(&page[0x14], fields->journal_sequence);
    zy100_put_u32_le(&page[0x18], fields->result_high_water);
    zy100_put_u32_le(&page[0x1C], fields->protected_user_id);
    zy100_put_u32_le(&page[0x20], fields->foreign_purge_batch_token);
    zy100_put_u32_le(&page[0x24], ZY100_OFFLINE_V2_LAYOUT_CRC32);
    zy100_put_u32_le(&page[0x28], fields->foreign_purge_total_sectors);
    zy100_put_u32_le(&page[0x2C], fields->foreign_purge_erased_sectors);
    memset(&page[OFFLINE_V2_JOURNAL_CRC_OFFSET], 0, 4U);
    zy100_put_u32_le(&page[OFFLINE_V2_JOURNAL_COMMIT_OFFSET],
                     OFFLINE_V2_JOURNAL_HEADER_COMMIT);
    crc = zy100_crc32_ieee(page, ZY100_OFFLINE_V2_JOURNAL_HEADER_BYTES);
    zy100_put_u32_le(&page[OFFLINE_V2_JOURNAL_CRC_OFFSET], crc);
    return true;
}

bool zy100_offline_v2_journal_header_encode_v5(
    uint8_t page[ZY100_OFFLINE_V2_JOURNAL_HEADER_BYTES],
    const zy100_offline_v2_journal_header_fields_t *fields,
    uint32_t snapshot_count)
{
    if (!zy100_offline_v2_journal_header_encode(page, fields) || snapshot_count > 65U)
        return false;
    zy100_put_u16_le(&page[4], ZY100_OFFLINE_V2_JOURNAL_RUNTIME_VERSION);
    zy100_put_u32_le(&page[0x30], snapshot_count);
    zy100_put_u32_le(&page[OFFLINE_V2_JOURNAL_CRC_OFFSET], 0U);
    zy100_put_u32_le(&page[OFFLINE_V2_JOURNAL_CRC_OFFSET],
        zy100_crc32_ieee(page, ZY100_OFFLINE_V2_JOURNAL_HEADER_BYTES));
    return true;
}

bool zy100_offline_v2_journal_header_valid(
    const uint8_t page[ZY100_OFFLINE_V2_JOURNAL_HEADER_BYTES],
    uint32_t *generation_out)
{
    uint8_t copy[ZY100_OFFLINE_V2_JOURNAL_HEADER_BYTES];
    uint32_t stored_crc;

    if ((page == NULL) ||
        (zy100_get_u32_le(&page[0x00]) != OFFLINE_V2_JOURNAL_MAGIC) ||
        ((zy100_get_u16_le(&page[0x04]) != 2U) &&
         (zy100_get_u16_le(&page[0x04]) != 3U) &&
         (zy100_get_u16_le(&page[0x04]) != ZY100_OFFLINE_V2_JOURNAL_RUNTIME_VERSION) &&
         (zy100_get_u16_le(&page[0x04]) !=
          ZY100_OFFLINE_V2_JOURNAL_HEADER_VERSION)) ||
        (zy100_get_u16_le(&page[0x06]) !=
         ZY100_OFFLINE_V2_JOURNAL_HEADER_BYTES) ||
        (zy100_get_u32_le(&page[0x24]) !=
         ZY100_OFFLINE_V2_LAYOUT_CRC32) ||
        (zy100_get_u32_le(&page[OFFLINE_V2_JOURNAL_COMMIT_OFFSET]) !=
         OFFLINE_V2_JOURNAL_HEADER_COMMIT))
    {
        return false;
    }

    memcpy(copy, page, sizeof(copy));
    stored_crc = zy100_get_u32_le(&copy[OFFLINE_V2_JOURNAL_CRC_OFFSET]);
    memset(&copy[OFFLINE_V2_JOURNAL_CRC_OFFSET], 0, 4U);
    if (stored_crc != zy100_crc32_ieee(copy, sizeof(copy)))
    {
        return false;
    }
    if (generation_out != NULL)
    {
        *generation_out = zy100_get_u32_le(&page[0x08]);
    }
    return true;
}
