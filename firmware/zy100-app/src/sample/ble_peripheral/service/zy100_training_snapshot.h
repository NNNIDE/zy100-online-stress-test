#ifndef ZY100_TRAINING_SNAPSHOT_H
#define ZY100_TRAINING_SNAPSHOT_H

#include <stdbool.h>
#include <stdint.h>

#define ZY100_TRAINING_TOKEN_BYTES 16U
#define ZY100_TRAINING_SNAPSHOT_BYTES 64U
#define ZY100_TRAINING_SNAPSHOT_VERSION 1U
#define ZY100_TRAINING_FLAG_ACTIVE 1U
#define ZY100_TRAINING_FLAG_TIME_VALID 2U
#define ZY100_TRAINING_FLAG_ELAPSED_VALID 4U
#define ZY100_TRAINING_FLAG_TOKEN_VALID 8U

typedef struct
{
    uint64_t start_unix_ms;
    uint64_t elapsed_ms;
    uint32_t session_id;
    uint32_t storage_generation;
    uint32_t owner_user_id;
    uint8_t token[ZY100_TRAINING_TOKEN_BYTES];
    uint8_t device_address[6];
    uint8_t phase; /* 0 idle, 1 intent, 2 starting, 3 running, 4 stopping, 5 finalizing, 7 error */
    uint8_t flags;
    uint8_t source; /* 0 button, 1 BLE, 2 motion, 255 none */
} zy100_training_snapshot_t;

#endif
