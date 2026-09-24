#include "zy100_offline_v2_marker.h"

#include <string.h>

#include "../driver/drv_internal_flash.h"
#include "../service/zy100_crc32.h"
#include "../service/zy100_device_identity.h"
#include "../common/zy100_byteorder.h"

#define OFFLINE_V2_MARKER_MAGIC          0x314D464FUL
#define OFFLINE_V2_MARKER_VERSION        1U
#define OFFLINE_V2_MARKER_BYTES          256U
#define OFFLINE_V2_MARKER_SLOT_BYTES     ZY100_INTERNAL_FLASH_SECTOR_BYTES
#define OFFLINE_V2_MARKER_SLOT_A         ZY100_OFFLINE_V2_MARKER_PRIMARY_OFFSET
#define OFFLINE_V2_MARKER_SLOT_B         ZY100_OFFLINE_V2_MARKER_BACKUP_OFFSET
#define OFFLINE_V2_MARKER_CRC_OFFSET     0xF8U
#define OFFLINE_V2_MARKER_COMMIT_OFFSET  0xFCU
#define OFFLINE_V2_MARKER_COMMIT         0x4D4B5632UL

static bool marker_generation_newer(uint32_t candidate, uint32_t reference)
{
    return (candidate != reference) &&
           ((uint32_t)(candidate - reference) < 0x80000000UL);
}

static bool marker_decode(const uint8_t page[OFFLINE_V2_MARKER_BYTES],
                          zy100_offline_v2_marker_t *marker)
{
    uint8_t crc_page[OFFLINE_V2_MARKER_BYTES];
    uint32_t stored_crc;
    uint32_t calculated_crc;

    if ((zy100_get_u32_le(&page[0x00]) != OFFLINE_V2_MARKER_MAGIC) ||
        (zy100_get_u16_le(&page[0x04]) != OFFLINE_V2_MARKER_VERSION) ||
        (zy100_get_u16_le(&page[0x06]) != OFFLINE_V2_MARKER_BYTES) ||
        (zy100_get_u32_le(&page[OFFLINE_V2_MARKER_COMMIT_OFFSET]) !=
         OFFLINE_V2_MARKER_COMMIT))
    {
        return false;
    }
    memcpy(crc_page, page, sizeof(crc_page));
    stored_crc = zy100_get_u32_le(&crc_page[OFFLINE_V2_MARKER_CRC_OFFSET]);
    memset(&crc_page[OFFLINE_V2_MARKER_CRC_OFFSET], 0, 4U);
    calculated_crc = zy100_crc32_ieee(crc_page, sizeof(crc_page));
    if (stored_crc != calculated_crc)
    {
        return false;
    }
    marker->generation = zy100_get_u32_le(&page[0x08]);
    marker->state = (zy100_offline_v2_marker_state_t)
                    zy100_get_u32_le(&page[0x0C]);
    marker->device_id = zy100_get_u32_le(&page[0x10]);
    marker->layout_crc32 = zy100_get_u32_le(&page[0x14]);
    marker->online_spool_crc32 = zy100_get_u32_le(&page[0x18]);
    marker->next_erase_addr = zy100_get_u32_le(&page[0x1C]);
    marker->factory_handoff = zy100_get_u32_le(&page[0x20]);
    return (marker->device_id == zy100_device_internal_id()) &&
           (marker->state >= ZY100_OFFLINE_V2_MARKER_MIGRATING) &&
           (marker->state <= ZY100_OFFLINE_V2_MARKER_CLEARING);
}

bool zy100_offline_v2_marker_load(zy100_offline_v2_marker_t *marker_out)
{
    uint8_t page_a[OFFLINE_V2_MARKER_BYTES];
    uint8_t page_b[OFFLINE_V2_MARKER_BYTES];
    zy100_offline_v2_marker_t marker_a;
    zy100_offline_v2_marker_t marker_b;
    bool valid_a;
    bool valid_b;

    if (marker_out == NULL)
    {
        return false;
    }
    if ((drv_internal_flash_read(OFFLINE_V2_MARKER_SLOT_A,
                                 page_a,
                                 sizeof(page_a)) !=
         DRV_INTERNAL_FLASH_STATUS_OK) ||
        (drv_internal_flash_read(OFFLINE_V2_MARKER_SLOT_B,
                                 page_b,
                                 sizeof(page_b)) !=
         DRV_INTERNAL_FLASH_STATUS_OK))
    {
        return false;
    }
    valid_a = marker_decode(page_a, &marker_a);
    valid_b = marker_decode(page_b, &marker_b);
    if (!valid_a && !valid_b)
    {
        memset(marker_out, 0, sizeof(*marker_out));
        marker_out->state = ZY100_OFFLINE_V2_MARKER_NONE;
        return true;
    }
    if (valid_a && (!valid_b ||
                    marker_generation_newer(marker_a.generation,
                                            marker_b.generation)))
    {
        *marker_out = marker_a;
    }
    else
    {
        *marker_out = marker_b;
    }
    return true;
}

bool zy100_offline_v2_marker_commit(
    const zy100_offline_v2_marker_t *marker)
{
    uint8_t page[OFFLINE_V2_MARKER_BYTES];
    uint32_t slot_offset;
    uint32_t crc;

    if ((marker == NULL) ||
        (marker->state < ZY100_OFFLINE_V2_MARKER_MIGRATING) ||
        (marker->state > ZY100_OFFLINE_V2_MARKER_CLEARING) ||
        (marker->device_id != zy100_device_internal_id()))
    {
        return false;
    }
    slot_offset = ((marker->generation & 1U) != 0U) ?
                  OFFLINE_V2_MARKER_SLOT_B : OFFLINE_V2_MARKER_SLOT_A;
    memset(page, 0xFF, sizeof(page));
    zy100_put_u32_le(&page[0x00], OFFLINE_V2_MARKER_MAGIC);
    zy100_put_u16_le(&page[0x04], OFFLINE_V2_MARKER_VERSION);
    zy100_put_u16_le(&page[0x06], OFFLINE_V2_MARKER_BYTES);
    zy100_put_u32_le(&page[0x08], marker->generation);
    zy100_put_u32_le(&page[0x0C], (uint32_t)marker->state);
    zy100_put_u32_le(&page[0x10], marker->device_id);
    zy100_put_u32_le(&page[0x14], marker->layout_crc32);
    zy100_put_u32_le(&page[0x18], marker->online_spool_crc32);
    zy100_put_u32_le(&page[0x1C], marker->next_erase_addr);
    zy100_put_u32_le(&page[0x20], marker->factory_handoff);
    memset(&page[OFFLINE_V2_MARKER_CRC_OFFSET], 0, 4U);
    zy100_put_u32_le(&page[OFFLINE_V2_MARKER_COMMIT_OFFSET],
                     OFFLINE_V2_MARKER_COMMIT);
    crc = zy100_crc32_ieee(page, sizeof(page));
    zy100_put_u32_le(&page[OFFLINE_V2_MARKER_CRC_OFFSET], crc);
    if (drv_internal_flash_erase_sector(slot_offset) !=
        DRV_INTERNAL_FLASH_STATUS_OK)
    {
        return false;
    }
    return drv_internal_flash_write(slot_offset, page, sizeof(page)) ==
           DRV_INTERNAL_FLASH_STATUS_OK;
}

bool zy100_offline_v2_marker_commit_fresh_ready(
    const zy100_offline_v2_marker_t *marker)
{
    uint32_t stale_slot;

    if ((marker == NULL) ||
        (marker->state != ZY100_OFFLINE_V2_MARKER_READY) ||
        !zy100_offline_v2_marker_commit(marker))
    {
        return false;
    }
    stale_slot = ((marker->generation & 1U) != 0U) ?
                 OFFLINE_V2_MARKER_SLOT_A : OFFLINE_V2_MARKER_SLOT_B;
    return drv_internal_flash_erase_sector(stale_slot) ==
           DRV_INTERNAL_FLASH_STATUS_OK;
}

bool zy100_offline_v2_marker_factory_handoff_pending(
    const zy100_offline_v2_marker_t *marker)
{
    return (marker != NULL) &&
           (marker->state == ZY100_OFFLINE_V2_MARKER_READY) &&
           (marker->factory_handoff ==
            ZY100_OFFLINE_V2_FACTORY_HANDOFF_PENDING);
}

zy100_offline_v2_handoff_consume_status_t
zy100_offline_v2_marker_consume_factory_handoff(
    zy100_offline_v2_marker_t *marker)
{
    zy100_offline_v2_marker_t consumed;
    zy100_offline_v2_marker_t verified;

    if (marker == NULL)
    {
        return ZY100_OFFLINE_V2_HANDOFF_ERROR;
    }
    if (!zy100_offline_v2_marker_factory_handoff_pending(marker))
    {
        return ZY100_OFFLINE_V2_HANDOFF_NOT_PENDING;
    }

    consumed = *marker;
    consumed.generation++;
    consumed.factory_handoff = ZY100_OFFLINE_V2_FACTORY_HANDOFF_NONE;
    if (!zy100_offline_v2_marker_commit(&consumed) ||
        !zy100_offline_v2_marker_load(&verified) ||
        (verified.generation != consumed.generation) ||
        (verified.state != consumed.state) ||
        (verified.device_id != consumed.device_id) ||
        (verified.layout_crc32 != consumed.layout_crc32) ||
        (verified.online_spool_crc32 != consumed.online_spool_crc32) ||
        (verified.next_erase_addr != consumed.next_erase_addr) ||
        (verified.factory_handoff !=
         ZY100_OFFLINE_V2_FACTORY_HANDOFF_NONE))
    {
        return ZY100_OFFLINE_V2_HANDOFF_ERROR;
    }

    *marker = verified;
    return ZY100_OFFLINE_V2_HANDOFF_CONSUMED;
}
