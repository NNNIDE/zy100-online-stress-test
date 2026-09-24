#include "zy100_feature_config.h"

#include <string.h>

#include <ftl.h>
#include <trace.h>

#include "../app_mfg/zp_mfg_protocol.h"
#include "../app_mfg/zp_mfg_store.h"
#include "../common/zy100_byteorder.h"
#include "zy100_app_ftl_layout.h"
#include "zy100_crc32.h"

#define ZY100_FEATURE_RECORD_MAGIC0              'Z'
#define ZY100_FEATURE_RECORD_MAGIC1              'F'
#define ZY100_FEATURE_RECORD_MAGIC2              'C'
#define ZY100_FEATURE_RECORD_MAGIC3              '1'
#define ZY100_FEATURE_RECORD_VERSION             1U
#define ZY100_FEATURE_RECORD_BYTES               32U
#define ZY100_FEATURE_RECORD_GENERATION_OFFSET   8U
#define ZY100_FEATURE_RECORD_OWNER_OFFSET        12U
#define ZY100_FEATURE_RECORD_CONFIG_OFFSET       16U
#define ZY100_FEATURE_RECORD_CONFIG_CRC_OFFSET   24U
#define ZY100_FEATURE_RECORD_CRC_OFFSET          28U

typedef enum
{
    ZY100_FEATURE_SLOT_NOT_FOUND = 0U,
    ZY100_FEATURE_SLOT_VALID,
    ZY100_FEATURE_SLOT_INVALID,
    ZY100_FEATURE_SLOT_READ_ERROR,
} zy100_feature_slot_status_t;

typedef struct
{
    zy100_feature_slot_status_t status;
    uint32_t generation;
    uint32_t owner_user_id;
    uint32_t config_crc32;
    zy100_feature_config_t config;
} zy100_feature_slot_t;

static zy100_feature_config_t s_config;
static uint32_t s_owner_user_id;
static uint32_t s_generation;
static bool s_initialized;

typedef char zy100_feature_config_size_check[
    (sizeof(zy100_feature_config_t) == ZY100_FEATURE_CONFIG_WIRE_BYTES) ? 1 : -1];
typedef char zy100_feature_record_size_check[
    (ZY100_FEATURE_RECORD_BYTES == ZY100_APP_FTL_FEATURE_CONFIG_SLOT_BYTES) ? 1 : -1];

static bool zy100_feature_generation_newer(uint32_t candidate,
                                           uint32_t current)
{
    return ((int32_t)(candidate - current)) > 0;
}

void zy100_feature_config_default(zy100_feature_config_t *config)
{
    if (config == NULL)
    {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->schema_version = ZY100_FEATURE_CONFIG_SCHEMA_VERSION;
    config->flags = ZY100_FEATURE_CONFIG_FLAG_TRAINING_LED |
                    (ZY100_FEATURE_LED_SPEED_CAPTURE_DEFAULT <<
                     ZY100_FEATURE_CONFIG_SPEED_SHIFT);
    config->target = ZY100_FEATURE_LED_TARGET_NOTIFY;
    config->effect = ZY100_FEATURE_LED_EFFECT_BREATH;
    config->blue = 255U;
    config->brightness_percent = 100U;
}

uint8_t zy100_feature_config_speed(const zy100_feature_config_t *config)
{
    if (config == NULL)
    {
        return ZY100_FEATURE_LED_SPEED_CAPTURE_DEFAULT;
    }
    return (uint8_t)((config->flags & ZY100_FEATURE_CONFIG_SPEED_MASK) >>
                     ZY100_FEATURE_CONFIG_SPEED_SHIFT);
}

bool zy100_feature_config_validate(const zy100_feature_config_t *config)
{
    bool led_enabled;
    uint8_t speed;

    if ((config == NULL) ||
        (config->schema_version != ZY100_FEATURE_CONFIG_SCHEMA_VERSION) ||
        ((config->flags & (uint8_t)~ZY100_FEATURE_CONFIG_FLAGS_ALLOWED) != 0U) ||
        (zy100_feature_config_speed(config) > ZY100_FEATURE_LED_SPEED_FAST) ||
        (config->target < ZY100_FEATURE_LED_TARGET_NOTIFY) ||
        (config->target > ZY100_FEATURE_LED_TARGET_ALL) ||
        (config->effect < ZY100_FEATURE_LED_EFFECT_SOLID) ||
        (config->effect > ZY100_FEATURE_LED_EFFECT_MARQUEE) ||
        (config->brightness_percent == 0U) ||
        (config->brightness_percent > 100U) ||
        ((config->effect == ZY100_FEATURE_LED_EFFECT_MARQUEE) &&
         (config->target == ZY100_FEATURE_LED_TARGET_NOTIFY)))
    {
        return false;
    }
    speed = zy100_feature_config_speed(config);
    if (((config->effect == ZY100_FEATURE_LED_EFFECT_SOLID) &&
         (speed != ZY100_FEATURE_LED_SPEED_CAPTURE_DEFAULT)) ||
        ((config->effect == ZY100_FEATURE_LED_EFFECT_BLINK) &&
         (speed == ZY100_FEATURE_LED_SPEED_CAPTURE_DEFAULT)) ||
        ((config->effect == ZY100_FEATURE_LED_EFFECT_MARQUEE) &&
         (speed == ZY100_FEATURE_LED_SPEED_CAPTURE_DEFAULT)) ||
        ((config->effect != ZY100_FEATURE_LED_EFFECT_MARQUEE) &&
         ((config->flags & ZY100_FEATURE_CONFIG_FLAG_REVERSE) != 0U)))
    {
        return false;
    }
    led_enabled =
        (config->flags & ZY100_FEATURE_CONFIG_FLAG_TRAINING_LED) != 0U;
    return !led_enabled || (config->red != 0U) ||
           (config->green != 0U) || (config->blue != 0U);
}

uint32_t zy100_feature_config_crc32(uint32_t owner_user_id,
                                    const zy100_feature_config_t *config)
{
    uint8_t bytes[4U + ZY100_FEATURE_CONFIG_WIRE_BYTES];

    if ((owner_user_id == 0U) || (config == NULL))
    {
        return 0U;
    }
    zy100_put_u32_le(bytes, owner_user_id);
    memcpy(&bytes[4], config, sizeof(*config));
    return zy100_crc32_ieee(bytes, sizeof(bytes));
}

static bool zy100_feature_record_blank(
    const uint8_t record[ZY100_FEATURE_RECORD_BYTES])
{
    uint8_t index;
    bool all_zero = true;
    bool all_erased = true;

    for (index = 0U; index < ZY100_FEATURE_RECORD_BYTES; index++)
    {
        all_zero = all_zero && (record[index] == 0U);
        all_erased = all_erased && (record[index] == 0xFFU);
    }
    return all_zero || all_erased;
}

static void zy100_feature_record_encode(
    uint8_t record[ZY100_FEATURE_RECORD_BYTES],
    uint32_t generation,
    uint32_t owner_user_id,
    const zy100_feature_config_t *config,
    uint32_t config_crc32)
{
    memset(record, 0, ZY100_FEATURE_RECORD_BYTES);
    record[0] = ZY100_FEATURE_RECORD_MAGIC0;
    record[1] = ZY100_FEATURE_RECORD_MAGIC1;
    record[2] = ZY100_FEATURE_RECORD_MAGIC2;
    record[3] = ZY100_FEATURE_RECORD_MAGIC3;
    record[4] = ZY100_FEATURE_RECORD_VERSION;
    record[5] = ZY100_FEATURE_CONFIG_SCHEMA_VERSION;
    zy100_put_u16_le(&record[6], ZY100_FEATURE_RECORD_BYTES);
    zy100_put_u32_le(&record[ZY100_FEATURE_RECORD_GENERATION_OFFSET],
                     generation);
    zy100_put_u32_le(&record[ZY100_FEATURE_RECORD_OWNER_OFFSET],
                     owner_user_id);
    memcpy(&record[ZY100_FEATURE_RECORD_CONFIG_OFFSET], config, sizeof(*config));
    zy100_put_u32_le(&record[ZY100_FEATURE_RECORD_CONFIG_CRC_OFFSET],
                     config_crc32);
    zy100_put_u32_le(&record[ZY100_FEATURE_RECORD_CRC_OFFSET],
                     zy100_crc32_ieee(record,
                                      ZY100_FEATURE_RECORD_CRC_OFFSET));
}

static bool zy100_feature_record_decode(
    const uint8_t record[ZY100_FEATURE_RECORD_BYTES],
    zy100_feature_slot_t *slot)
{
    uint32_t config_crc32;

    if ((record == NULL) || (slot == NULL) ||
        (record[0] != ZY100_FEATURE_RECORD_MAGIC0) ||
        (record[1] != ZY100_FEATURE_RECORD_MAGIC1) ||
        (record[2] != ZY100_FEATURE_RECORD_MAGIC2) ||
        (record[3] != ZY100_FEATURE_RECORD_MAGIC3) ||
        (record[4] != ZY100_FEATURE_RECORD_VERSION) ||
        (record[5] != ZY100_FEATURE_CONFIG_SCHEMA_VERSION) ||
        (zy100_get_u16_le(&record[6]) != ZY100_FEATURE_RECORD_BYTES) ||
        (zy100_get_u32_le(&record[ZY100_FEATURE_RECORD_CRC_OFFSET]) !=
         zy100_crc32_ieee(record, ZY100_FEATURE_RECORD_CRC_OFFSET)))
    {
        return false;
    }
    slot->generation =
        zy100_get_u32_le(&record[ZY100_FEATURE_RECORD_GENERATION_OFFSET]);
    slot->owner_user_id =
        zy100_get_u32_le(&record[ZY100_FEATURE_RECORD_OWNER_OFFSET]);
    memcpy(&slot->config,
           &record[ZY100_FEATURE_RECORD_CONFIG_OFFSET],
           sizeof(slot->config));
    config_crc32 =
        zy100_get_u32_le(&record[ZY100_FEATURE_RECORD_CONFIG_CRC_OFFSET]);
    slot->config_crc32 = config_crc32;
    return (slot->generation != 0U) && (slot->owner_user_id != 0U) &&
           zy100_feature_config_validate(&slot->config) &&
           (config_crc32 == zy100_feature_config_crc32(
                                slot->owner_user_id, &slot->config));
}

static zy100_feature_slot_t zy100_feature_read_slot(uint16_t offset)
{
    zy100_feature_slot_t slot;
    uint8_t record[ZY100_FEATURE_RECORD_BYTES];
    uint32_t result;

    memset(&slot, 0, sizeof(slot));
    memset(record, 0, sizeof(record));
    result = ftl_load(record, offset, sizeof(record));
    if (result == FTL_READ_ERROR_READ_NOT_FOUND)
    {
        slot.status = ZY100_FEATURE_SLOT_NOT_FOUND;
    }
    else if (result != FTL_READ_SUCCESS)
    {
        slot.status = ZY100_FEATURE_SLOT_READ_ERROR;
        DBG_DIRECT("[FEATURE_CFG][ERR] read off=0x%04x result=%lu",
                   offset, (unsigned long)result);
    }
    else if (zy100_feature_record_blank(record))
    {
        slot.status = ZY100_FEATURE_SLOT_NOT_FOUND;
    }
    else if (zy100_feature_record_decode(record, &slot))
    {
        slot.status = ZY100_FEATURE_SLOT_VALID;
    }
    else
    {
        slot.status = ZY100_FEATURE_SLOT_INVALID;
    }
    return slot;
}

static zy100_feature_config_status_t zy100_feature_load_latest(
    zy100_feature_slot_t *latest,
    zy100_feature_slot_t *primary_out,
    zy100_feature_slot_t *backup_out)
{
    zy100_feature_slot_t primary = zy100_feature_read_slot(
        ZY100_APP_FTL_FEATURE_CONFIG_PRIMARY_OFFSET);
    zy100_feature_slot_t backup = zy100_feature_read_slot(
        ZY100_APP_FTL_FEATURE_CONFIG_BACKUP_OFFSET);

    if ((latest == NULL) || (primary_out == NULL) || (backup_out == NULL))
    {
        return ZY100_FEATURE_CONFIG_STATUS_INVALID;
    }
    *primary_out = primary;
    *backup_out = backup;
    memset(latest, 0, sizeof(*latest));
    if ((primary.status == ZY100_FEATURE_SLOT_VALID) &&
        (backup.status == ZY100_FEATURE_SLOT_VALID))
    {
        *latest = zy100_feature_generation_newer(backup.generation,
                                                  primary.generation) ?
                  backup : primary;
        return ZY100_FEATURE_CONFIG_STATUS_OK;
    }
    if (primary.status == ZY100_FEATURE_SLOT_VALID)
    {
        *latest = primary;
        return ZY100_FEATURE_CONFIG_STATUS_OK;
    }
    if (backup.status == ZY100_FEATURE_SLOT_VALID)
    {
        *latest = backup;
        return ZY100_FEATURE_CONFIG_STATUS_OK;
    }
    if ((primary.status == ZY100_FEATURE_SLOT_READ_ERROR) ||
        (backup.status == ZY100_FEATURE_SLOT_READ_ERROR))
    {
        return ZY100_FEATURE_CONFIG_STATUS_READ_ERROR;
    }
    if ((primary.status == ZY100_FEATURE_SLOT_INVALID) ||
        (backup.status == ZY100_FEATURE_SLOT_INVALID))
    {
        return ZY100_FEATURE_CONFIG_STATUS_INVALID;
    }
    return ZY100_FEATURE_CONFIG_STATUS_NOT_FOUND;
}

static bool zy100_feature_config_from_mfg(zy100_feature_config_t *config)
{
    zp_mfg_record_t record;
    zp_mfg_training_led_config_t mfg;

    if ((config == NULL) ||
        (zp_mfg_store_load(&record) != ZP_MFG_STORE_STATUS_OK) ||
        !zp_mfg_store_record_locked(&record) ||
        !zp_mfg_training_led_config_get_record(&record, &mfg))
    {
        return false;
    }
    memset(config, 0, sizeof(*config));
    config->schema_version = ZY100_FEATURE_CONFIG_SCHEMA_VERSION;
    config->flags = (mfg.auto_capture_enabled != 0U) ?
                    ZY100_FEATURE_CONFIG_FLAG_AUTO_CAPTURE : 0U;
    if ((mfg.flags & ZP_MFG_TRAINING_LED_FLAG_ENABLED) != 0U)
    {
        config->flags |= ZY100_FEATURE_CONFIG_FLAG_TRAINING_LED;
    }
    if ((mfg.flags & ZP_MFG_TRAINING_LED_FLAG_MARQUEE_REVERSE) != 0U)
    {
        config->flags |= ZY100_FEATURE_CONFIG_FLAG_REVERSE;
    }
    config->flags |= (uint8_t)(mfg.speed << ZY100_FEATURE_CONFIG_SPEED_SHIFT);
    config->target = mfg.target;
    config->effect = mfg.effect;
    config->red = mfg.red;
    config->green = mfg.green;
    config->blue = mfg.blue;
    config->brightness_percent = mfg.brightness_percent;
    return zy100_feature_config_validate(config);
}

void zy100_feature_config_init(void)
{
    zy100_feature_slot_t latest;
    zy100_feature_slot_t primary;
    zy100_feature_slot_t backup;
    zy100_feature_config_status_t status = zy100_feature_load_latest(
        &latest, &primary, &backup);

    s_owner_user_id = 0U;
    s_generation = 0U;
    if (status == ZY100_FEATURE_CONFIG_STATUS_OK)
    {
        s_config = latest.config;
        s_owner_user_id = latest.owner_user_id;
        s_generation = latest.generation;
        DBG_DIRECT("[FEATURE_CFG] load source=ftl owner=%lu generation=%lu",
                   (unsigned long)s_owner_user_id,
                   (unsigned long)s_generation);
    }
    else if (zy100_feature_config_from_mfg(&s_config))
    {
        DBG_DIRECT("[FEATURE_CFG] load source=mfg_default");
    }
    else
    {
        zy100_feature_config_default(&s_config);
        DBG_DIRECT("[FEATURE_CFG] load source=compiled_default status=%s",
                   zy100_feature_config_status_name(status));
    }
    s_initialized = true;
}

bool zy100_feature_config_get(zy100_feature_config_t *config,
                              uint32_t *owner_user_id,
                              uint32_t *generation)
{
    if (!s_initialized)
    {
        zy100_feature_config_init();
    }
    if (config != NULL)
    {
        *config = s_config;
    }
    if (owner_user_id != NULL)
    {
        *owner_user_id = s_owner_user_id;
    }
    if (generation != NULL)
    {
        *generation = s_generation;
    }
    return zy100_feature_config_validate(&s_config);
}

bool zy100_feature_config_matches(uint32_t owner_user_id,
                                  const zy100_feature_config_t *config,
                                  uint32_t config_crc32)
{
    if (!s_initialized)
    {
        zy100_feature_config_init();
    }
    return (owner_user_id != 0U) &&
           (owner_user_id == s_owner_user_id) &&
           (config != NULL) &&
           (config_crc32 == zy100_feature_config_crc32(owner_user_id,
                                                        config)) &&
           (memcmp(config, &s_config, sizeof(*config)) == 0);
}

zy100_feature_config_status_t zy100_feature_config_commit(
    uint32_t owner_user_id,
    const zy100_feature_config_t *config,
    uint32_t config_crc32,
    uint32_t *generation_out)
{
    zy100_feature_slot_t latest;
    zy100_feature_slot_t primary;
    zy100_feature_slot_t backup;
    zy100_feature_config_status_t load_status;
    uint8_t record[ZY100_FEATURE_RECORD_BYTES];
    uint8_t verify[ZY100_FEATURE_RECORD_BYTES];
    uint32_t generation;
    uint16_t target_offset;
    uint32_t result;

    if ((owner_user_id == 0U) || !zy100_feature_config_validate(config) ||
        (config_crc32 != zy100_feature_config_crc32(owner_user_id, config)))
    {
        return ZY100_FEATURE_CONFIG_STATUS_INVALID;
    }
    load_status = zy100_feature_load_latest(&latest, &primary, &backup);
    /* A failed verification can leave a newer record than the runtime cache.
     * Both slots must be readable before a commit can claim persistence. */
    if ((primary.status == ZY100_FEATURE_SLOT_READ_ERROR) ||
        (backup.status == ZY100_FEATURE_SLOT_READ_ERROR))
    {
        return ZY100_FEATURE_CONFIG_STATUS_READ_ERROR;
    }
    if ((load_status == ZY100_FEATURE_CONFIG_STATUS_OK) &&
        (latest.owner_user_id == owner_user_id) &&
        (latest.config_crc32 == config_crc32) &&
        (memcmp(&latest.config, config, sizeof(*config)) == 0))
    {
        s_config = latest.config;
        s_owner_user_id = latest.owner_user_id;
        s_generation = latest.generation;
        s_initialized = true;
        if (generation_out != NULL)
        {
            *generation_out = s_generation;
        }
        return ZY100_FEATURE_CONFIG_STATUS_OK;
    }
    generation = (load_status == ZY100_FEATURE_CONFIG_STATUS_OK) ?
                 (latest.generation + 1U) : 1U;
    if (generation == 0U)
    {
        generation = 1U;
    }
    if (primary.status != ZY100_FEATURE_SLOT_VALID)
    {
        target_offset = ZY100_APP_FTL_FEATURE_CONFIG_PRIMARY_OFFSET;
    }
    else if (backup.status != ZY100_FEATURE_SLOT_VALID)
    {
        target_offset = ZY100_APP_FTL_FEATURE_CONFIG_BACKUP_OFFSET;
    }
    else
    {
        target_offset = zy100_feature_generation_newer(backup.generation,
                                                        primary.generation) ?
                        ZY100_APP_FTL_FEATURE_CONFIG_PRIMARY_OFFSET :
                        ZY100_APP_FTL_FEATURE_CONFIG_BACKUP_OFFSET;
    }
    zy100_feature_record_encode(record, generation, owner_user_id,
                                config, config_crc32);
    result = ftl_save(record, target_offset, sizeof(record));
    if (result != FTL_WRITE_SUCCESS)
    {
        DBG_DIRECT("[FEATURE_CFG][ERR] write off=0x%04x result=%lu",
                   target_offset, (unsigned long)result);
        return ZY100_FEATURE_CONFIG_STATUS_WRITE_ERROR;
    }
    memset(verify, 0, sizeof(verify));
    result = ftl_load(verify, target_offset, sizeof(verify));
    if ((result != FTL_READ_SUCCESS) ||
        (memcmp(record, verify, sizeof(record)) != 0))
    {
        DBG_DIRECT("[FEATURE_CFG][ERR] verify off=0x%04x result=%lu",
                   target_offset, (unsigned long)result);
        return ZY100_FEATURE_CONFIG_STATUS_VERIFY_ERROR;
    }
    s_config = *config;
    s_owner_user_id = owner_user_id;
    s_generation = generation;
    s_initialized = true;
    if (generation_out != NULL)
    {
        *generation_out = generation;
    }
    DBG_DIRECT("[FEATURE_CFG] commit owner=%lu generation=%lu off=0x%04x crc=0x%08lx",
               (unsigned long)owner_user_id,
               (unsigned long)generation,
               target_offset,
               (unsigned long)config_crc32);
    return ZY100_FEATURE_CONFIG_STATUS_OK;
}

static zy100_feature_config_status_t zy100_feature_wipe_slot(uint16_t offset)
{
    uint8_t blank[ZY100_FEATURE_RECORD_BYTES];
    uint8_t verify[ZY100_FEATURE_RECORD_BYTES];
    uint32_t result;

    memset(blank, 0, sizeof(blank));
    result = ftl_save(blank, offset, sizeof(blank));
    if (result != FTL_WRITE_SUCCESS)
    {
        return ZY100_FEATURE_CONFIG_STATUS_WRITE_ERROR;
    }
    memset(verify, 0xFF, sizeof(verify));
    result = ftl_load(verify, offset, sizeof(verify));
    if ((result != FTL_READ_SUCCESS) ||
        (memcmp(blank, verify, sizeof(blank)) != 0))
    {
        return ZY100_FEATURE_CONFIG_STATUS_VERIFY_ERROR;
    }
    return ZY100_FEATURE_CONFIG_STATUS_OK;
}

zy100_feature_config_status_t zy100_feature_config_factory_invalidate(void)
{
    zy100_feature_config_status_t status;

    status = zy100_feature_wipe_slot(
        ZY100_APP_FTL_FEATURE_CONFIG_PRIMARY_OFFSET);
    if (status != ZY100_FEATURE_CONFIG_STATUS_OK)
    {
        return status;
    }
    status = zy100_feature_wipe_slot(
        ZY100_APP_FTL_FEATURE_CONFIG_BACKUP_OFFSET);
    if (status != ZY100_FEATURE_CONFIG_STATUS_OK)
    {
        return status;
    }
    s_initialized = false;
    zy100_feature_config_init();
    return ZY100_FEATURE_CONFIG_STATUS_OK;
}

bool zy100_feature_config_auto_capture_enabled(void)
{
    if (!s_initialized)
    {
        zy100_feature_config_init();
    }
    return (s_config.flags & ZY100_FEATURE_CONFIG_FLAG_AUTO_CAPTURE) != 0U;
}

const char *zy100_feature_config_status_name(
    zy100_feature_config_status_t status)
{
    switch (status)
    {
    case ZY100_FEATURE_CONFIG_STATUS_OK: return "OK";
    case ZY100_FEATURE_CONFIG_STATUS_NOT_FOUND: return "NOT_FOUND";
    case ZY100_FEATURE_CONFIG_STATUS_INVALID: return "INVALID";
    case ZY100_FEATURE_CONFIG_STATUS_READ_ERROR: return "READ_ERROR";
    case ZY100_FEATURE_CONFIG_STATUS_WRITE_ERROR: return "WRITE_ERROR";
    case ZY100_FEATURE_CONFIG_STATUS_VERIFY_ERROR: return "VERIFY_ERROR";
    default: return "UNKNOWN";
    }
}
