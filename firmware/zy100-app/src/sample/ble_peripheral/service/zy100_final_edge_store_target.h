#ifndef ZY100_FINAL_EDGE_STORE_TARGET_H
#define ZY100_FINAL_EDGE_STORE_TARGET_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ZY100_FE_STORE_TARGET_RAW = 1U,
    ZY100_FE_STORE_TARGET_SUMMARY = 2U,
    ZY100_FE_STORE_TARGET_EVENT = 3U,
} zy100_fe_store_target_kind_t;

typedef enum
{
    ZY100_FE_STORE_RESERVE_OK = 0U,
    ZY100_FE_STORE_RESERVE_BUSY,
    ZY100_FE_STORE_RESERVE_FULL,
    ZY100_FE_STORE_RESERVE_ERROR,
} zy100_fe_store_reserve_result_t;

typedef struct
{
    uint32_t data_addr;
    uint32_t capacity_bytes;
    uint32_t online_record_id;
    uint32_t token;
} zy100_fe_store_target_t;

typedef bool (*zy100_fe_store_can_reserve_fn)(void *context,
                                               uint32_t record_bytes);
typedef zy100_fe_store_reserve_result_t (*zy100_fe_store_reserve_fn)(
    void *context,
    zy100_fe_store_target_kind_t kind,
    uint32_t source_id,
    uint32_t record_bytes,
    uint32_t payload_bytes,
    zy100_fe_store_target_t *target);
typedef bool (*zy100_fe_store_commit_fn)(void *context,
                                         uint32_t token);
typedef void (*zy100_fe_store_abort_fn)(void *context,
                                        uint32_t token);

typedef struct
{
    void *context;
    zy100_fe_store_can_reserve_fn can_reserve;
    zy100_fe_store_reserve_fn reserve;
    zy100_fe_store_commit_fn commit;
    zy100_fe_store_abort_fn abort;
} zy100_fe_store_target_provider_t;

#ifdef __cplusplus
}
#endif

#endif /* ZY100_FINAL_EDGE_STORE_TARGET_H */
