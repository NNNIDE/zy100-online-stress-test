#ifndef ZY100_OIS20_COMPRESS_H
#define ZY100_OIS20_COMPRESS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "../app_flags.h"

#define ZY100_OIS20_CMP_MAGIC 0x3043324FUL
#define ZY100_OIS20_CMP_VERSION 1U

#define ZY100_OIS20_CMP_CODEC_RAW20 0U
#define ZY100_OIS20_CMP_CODEC_DZBP 1U

#define ZY100_OIS20_CMP_AXIS_PACK_S20 0U
#define ZY100_OIS20_CMP_AXIS_PACK_E18_Q2 1U

#define ZY100_OIS20_CMP_FLAG_LSB2_NONZERO 0x0001U
#define ZY100_OIS20_CMP_FLAG_TAIL_BLOCK 0x0002U

typedef struct __attribute__((packed))
{
    uint32_t magic;
    uint8_t version;
    uint8_t header_len;
    uint8_t codec;
    uint8_t frame_count;
    uint32_t block_seq;
    uint32_t first_sample_index;
    uint16_t raw_len;
    uint16_t payload_len;
    uint16_t raw_crc16;
    uint8_t axis_pack_mode;
    uint8_t bw_temp;
    uint8_t bw_ax;
    uint8_t bw_ay;
    uint8_t bw_az;
    uint8_t bw_gx;
    uint8_t bw_gy;
    uint8_t bw_gz;
    uint8_t bw_ts_ddt;
    uint16_t ts_dt1;
    uint16_t flags;
} zy100_ois20_cmp_block_hdr_t;

typedef struct
{
    uint32_t raw_blocks;
    uint32_t cmp_blocks;
    uint32_t block_count;
    uint32_t frame_count;
    uint32_t raw_bytes;
    uint32_t stored_bytes;
    uint32_t capacity_bytes;
    uint32_t fallback_count;
    uint32_t overrun_count;
    uint32_t crc_fail_count;
    uint32_t decode_fail_count;
    uint32_t lsb2_nonzero_count;
    uint32_t max_block_bytes;
} zy100_ois20_cmp_stats_t;

typedef struct
{
    uint32_t frames;
    uint32_t checked;
    uint32_t bad;
    uint32_t zero;
    uint32_t wrap;
    uint32_t dt_min;
    uint32_t dt_max;
    uint32_t blocks;
    uint32_t pass;
    uint32_t block_seq_bad;
    uint32_t magic_bad;
    uint32_t codec_bad;
    uint32_t frame_count_bad;
    uint32_t crc_fail;
    uint32_t decode_fail;
    uint32_t payload_bad;
    uint16_t first_tmst_raw;
    uint16_t last_tmst_raw;
} zy100_ois20_decomp_verify_t;

typedef struct
{
    uint32_t codec;
    uint32_t frame_count;
    uint32_t decoded_frames;
    uint32_t decoded_bytes;
    uint32_t crc_fail;
    uint32_t decode_fail;
    uint32_t payload_bad;
} zy100_ois20_decomp_result_t;

typedef struct
{
    uint8_t *store;
    uint32_t capacity_bytes;
    uint32_t used_bytes;
    uint32_t next_block_seq;
    zy100_ois20_cmp_stats_t stats;
} zy100_ois20_block_store_t;

void zy100_ois20_block_store_init(zy100_ois20_block_store_t *ctx,
                                  uint8_t *store,
                                  uint32_t capacity_bytes);
void zy100_ois20_block_store_reset(zy100_ois20_block_store_t *ctx);
bool zy100_ois20_block_store_append_raw(zy100_ois20_block_store_t *ctx,
                                        const uint8_t *raw_frames,
                                        uint32_t frame_count,
                                        uint32_t first_sample_index);
bool zy100_ois20_block_store_append_auto_codec(zy100_ois20_block_store_t *ctx,
                                               const uint8_t *raw_frames,
                                               uint32_t frame_count,
                                               uint32_t first_sample_index);
bool zy100_ois20_block_store_finalize_raw(zy100_ois20_block_store_t *ctx);
void zy100_ois20_block_store_get_stats(const zy100_ois20_block_store_t *ctx,
                                       zy100_ois20_cmp_stats_t *out);
bool zy100_ois20_decompress_block(
    const uint8_t *block,
    uint32_t block_len,
    uint8_t out_frames[][ZY100_OIS20_CMP_FRAME_BYTES],
    zy100_ois20_decomp_result_t *result);
bool zy100_ois20_block_store_verify(const zy100_ois20_block_store_t *ctx,
                                    uint32_t expected_frames,
                                    uint32_t expected_blocks,
                                    uint32_t dt_min_us,
                                    uint32_t dt_max_us,
                                    zy100_ois20_decomp_verify_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_OIS20_COMPRESS_H */
