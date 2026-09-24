#ifndef ZY100_SYSTEM_INFO_STORE_H
#define ZY100_SYSTEM_INFO_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../app_flags.h"

#define ZY100_SYSTEM_INFO_BD_ADDR_BYTES 6U
#define ZY100_SYSTEM_INFO_SCHEMA_VERSION 7U
#define ZY100_SYSTEM_INFO_MAX_PAIRED_PEERS ZY100_BLE_MAX_PAIRED_CENTRALS
#define ZY100_OTA_IMAGE_MIGRATION_COMPENSATION_MS 13000ULL

typedef enum
{
    ZY100_OTA_TIME_STATE_NONE = 0U,
    ZY100_OTA_TIME_STATE_ARMED = 1U,
    ZY100_OTA_TIME_STATE_RESUME_PENDING = 2U,
} zy100_ota_time_state_t;

typedef enum
{
    ZY100_FACTORY_ACCEPTANCE_NONE = 0U,
    ZY100_FACTORY_ACCEPTANCE_PENDING = 1U,
    ZY100_FACTORY_ACCEPTANCE_SHIP_ARMED = 2U,
} zy100_factory_acceptance_state_t;

typedef enum
{
    ZY100_WHOLE_UNIT_NOT_REQUIRED = 0U,
    ZY100_WHOLE_UNIT_REQUIRED = 1U,
    ZY100_WHOLE_UNIT_ACTIVE = 2U,
    ZY100_WHOLE_UNIT_FINAL_SHIP_ARMED = 3U,
    ZY100_WHOLE_UNIT_COMPLETE = 4U,
} zy100_whole_unit_state_t;

typedef enum
{
    ZY100_PRODUCTION_SHIPPING_NONE = 0U,
    ZY100_PRODUCTION_SHIPPING_ARMED = 1U,
} zy100_production_shipping_state_t;

typedef struct
{
    uint8_t state;
    uint8_t reserved[3];
    uint64_t checkpoint_unix_ms;
    uint64_t checkpoint_rtc_ticks;
    uint64_t checkpoint_rtc_wrap_ticks;
    uint64_t resume_unix_ms;
    uint32_t checkpoint_user_id;
    uint32_t checkpoint_tick_hz;
    uint32_t source_version_code;
} zy100_system_info_ota_time_t;

typedef struct
{
    uint8_t peer_addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES];
    uint8_t peer_addr_type;
    uint8_t valid;
} zy100_system_info_paired_peer_t;

typedef struct
{
    uint8_t ble_addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES];
    uint8_t ble_addr_type;
    uint8_t ble_addr_valid;

    uint8_t pairing_summary_valid;
    uint8_t pairing_bonded;
    uint8_t last_peer_addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES];
    uint8_t last_peer_addr_type;
    uint8_t last_peer_addr_valid;

    uint8_t paired_peer_count;
    zy100_system_info_paired_peer_t paired_peers[ZY100_SYSTEM_INFO_MAX_PAIRED_PEERS];

    /* 0: normal; 1: reset transaction; 2: reset complete, first pairing required. */
    uint8_t user_reset_state;
    uint8_t first_power_seen;
    uint8_t ota_success_led_pending;
    uint8_t factory_acceptance_state;
    uint8_t whole_unit_state;
    uint8_t production_shipping_state;

    uint8_t software_version_major;
    uint8_t software_version_minor;
    uint8_t software_version_revision;
    uint8_t software_version_buildnum;
    uint32_t software_version_code;

    uint32_t latest_user_id;
    zy100_system_info_ota_time_t ota_time;
} zy100_system_info_t;

bool zy100_system_info_store_ready(void);
bool zy100_system_info_load(zy100_system_info_t *out);
bool zy100_system_info_save(const zy100_system_info_t *info);
bool zy100_system_info_ensure_current_software_version(void);
bool zy100_system_info_mark_first_power_seen(void);
bool zy100_system_info_set_ota_success_led_pending(bool enable);
bool zy100_system_info_get_ota_success_led_pending(bool *pending_out);
bool zy100_system_info_set_factory_acceptance_state(
    zy100_factory_acceptance_state_t state);
bool zy100_system_info_get_factory_acceptance_state(
    zy100_factory_acceptance_state_t *state_out);
bool zy100_system_info_set_manufacturing_states(
    zy100_factory_acceptance_state_t acceptance_state,
    zy100_whole_unit_state_t whole_unit_state);
bool zy100_system_info_get_manufacturing_states(
    zy100_factory_acceptance_state_t *acceptance_state_out,
    zy100_whole_unit_state_t *whole_unit_state_out);
bool zy100_system_info_set_whole_unit_state(zy100_whole_unit_state_t state);
bool zy100_system_info_get_whole_unit_state(zy100_whole_unit_state_t *state_out);
bool zy100_system_info_set_production_shipping_state(
    zy100_production_shipping_state_t state);
bool zy100_system_info_get_production_shipping_state(
    zy100_production_shipping_state_t *state_out);
bool zy100_system_info_ota_time_arm(
    const zy100_system_info_ota_time_t *checkpoint);
bool zy100_system_info_ota_success_finalize(uint64_t current_rtc_ticks,
                                            uint64_t current_rtc_wrap_ticks,
                                            uint32_t current_tick_hz,
                                            bool *resume_created_out);
bool zy100_system_info_ota_boot_consume(void);
bool zy100_system_info_ota_time_clear_stale(void);
bool zy100_system_info_set_pairing_bonded(
    const uint8_t peer_addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES],
    uint8_t peer_type);
bool zy100_system_info_add_paired_peer(
    const uint8_t peer_addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES],
    uint8_t peer_type);
bool zy100_system_info_remove_paired_peer(
    const uint8_t peer_addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES],
    uint8_t peer_type);
bool zy100_system_info_clear_pairing(void);
bool zy100_system_info_set_ble_addr(const uint8_t addr[ZY100_SYSTEM_INFO_BD_ADDR_BYTES],
                                    uint8_t addr_type);
bool zy100_system_info_get_latest_user_id(uint32_t *user_id_out,
                                          bool *valid_out);
bool zy100_system_info_set_latest_user_id(uint32_t user_id);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_SYSTEM_INFO_STORE_H */
