#include "zy100_ois20_compress.h"

#include <stddef.h>
#include <string.h>

#include "crc16btx.h"
#include "../common/zy100_byteorder.h"

#define OIS20_TEMP_HIGH_IDX 0U
#define OIS20_TEMP_LOW_IDX 1U
#define OIS20_AX_HIGH_IDX 2U
#define OIS20_AX_LOW_IDX 3U
#define OIS20_AY_HIGH_IDX 4U
#define OIS20_AY_LOW_IDX 5U
#define OIS20_AZ_HIGH_IDX 6U
#define OIS20_AZ_LOW_IDX 7U
#define OIS20_GX_HIGH_IDX 8U
#define OIS20_GX_LOW_IDX 9U
#define OIS20_GY_HIGH_IDX 10U
#define OIS20_GY_LOW_IDX 11U
#define OIS20_GZ_HIGH_IDX 12U
#define OIS20_GZ_LOW_IDX 13U
#define OIS20_TMST_HIGH_IDX 14U
#define OIS20_TMST_LOW_IDX 15U
#define OIS20_EXT_X_IDX 16U
#define OIS20_EXT_Y_IDX 17U
#define OIS20_EXT_Z_IDX 18U

#define OIS20_DZBP_MARGIN_BYTES 4U
#define OIS20_CHANNEL_COUNT 7U
#define OIS20_TS_DDT_MAX_COUNT (ZY100_OIS20_CMP_BLOCK_FRAMES - 2U)
#define OIS20_VALUE_MAX_COUNT (ZY100_OIS20_CMP_BLOCK_FRAMES - 1U)

typedef enum
{
    OIS20_CH_TEMP = 0U,
    OIS20_CH_AX,
    OIS20_CH_AY,
    OIS20_CH_AZ,
    OIS20_CH_GX,
    OIS20_CH_GY,
    OIS20_CH_GZ
} ois20_channel_t;

typedef struct
{
    int32_t temp;
    int32_t ax;
    int32_t ay;
    int32_t az;
    int32_t gx;
    int32_t gy;
    int32_t gz;
    uint16_t ts;
} ois20_sample_t;

typedef struct
{
    uint8_t *buf;
    uint32_t capacity_bytes;
    uint32_t bit_pos;
    bool overflow;
} ois20_bit_writer_t;

typedef struct
{
    const uint8_t *buf;
    uint32_t capacity_bytes;
    uint32_t bit_pos;
    bool overflow;
} ois20_bit_reader_t;

static int32_t ois20_sign_extend20(uint32_t v)
{
    v &= 0xFFFFFU;
    if ((v & 0x80000U) != 0U)
    {
        v |= 0xFFF00000U;
    }
    return (int32_t)v;
}

static uint32_t ois20_pack_signed20(int32_t v)
{
    return ((uint32_t)v) & 0xFFFFFU;
}

static uint16_t ois20_raw_tmst(const uint8_t *frame)
{
    return zy100_get_u16_be(&frame[OIS20_TMST_HIGH_IDX]);
}

static uint16_t ois20_crc16(const uint8_t *data, uint32_t len)
{
    return btxfcs(BTXFCS_INIT, (uint8_t *)data, len);
}

static uint32_t ois20_bytes_for_bits(uint32_t count, uint8_t bw)
{
    if ((count == 0U) || (bw == 0U))
    {
        return 0U;
    }
    return (((uint32_t)count * (uint32_t)bw) + 7U) / 8U;
}

static uint32_t ois20_zigzag32(int32_t v)
{
    return (((uint32_t)v) << 1) ^ ((uint32_t)(v >> 31));
}

static int32_t ois20_unzigzag32(uint32_t v)
{
    return (int32_t)((v >> 1) ^ ((uint32_t)(-(int32_t)(v & 1U))));
}

static uint8_t ois20_bits_required_u32(uint32_t v)
{
    uint8_t bits = 0U;

    while (v != 0U)
    {
        bits++;
        v >>= 1;
    }
    return bits;
}

static void ois20_parse_frame(const uint8_t raw[ZY100_OIS20_CMP_FRAME_BYTES],
                              ois20_sample_t *s)
{
    uint16_t temp16 = zy100_get_u16_be(&raw[OIS20_TEMP_HIGH_IDX]);
    uint16_t ax16 = zy100_get_u16_be(&raw[OIS20_AX_HIGH_IDX]);
    uint16_t ay16 = zy100_get_u16_be(&raw[OIS20_AY_HIGH_IDX]);
    uint16_t az16 = zy100_get_u16_be(&raw[OIS20_AZ_HIGH_IDX]);
    uint16_t gx16 = zy100_get_u16_be(&raw[OIS20_GX_HIGH_IDX]);
    uint16_t gy16 = zy100_get_u16_be(&raw[OIS20_GY_HIGH_IDX]);
    uint16_t gz16 = zy100_get_u16_be(&raw[OIS20_GZ_HIGH_IDX]);
    uint8_t ext_x = raw[OIS20_EXT_X_IDX];
    uint8_t ext_y = raw[OIS20_EXT_Y_IDX];
    uint8_t ext_z = raw[OIS20_EXT_Z_IDX];

    s->temp = (int32_t)(int16_t)temp16;
    s->ax = ois20_sign_extend20(((uint32_t)ax16 << 4) |
                                ((ext_x >> 4) & 0x0FU));
    s->gx = ois20_sign_extend20(((uint32_t)gx16 << 4) |
                                (ext_x & 0x0FU));
    s->ay = ois20_sign_extend20(((uint32_t)ay16 << 4) |
                                ((ext_y >> 4) & 0x0FU));
    s->gy = ois20_sign_extend20(((uint32_t)gy16 << 4) |
                                (ext_y & 0x0FU));
    s->az = ois20_sign_extend20(((uint32_t)az16 << 4) |
                                ((ext_z >> 4) & 0x0FU));
    s->gz = ois20_sign_extend20(((uint32_t)gz16 << 4) |
                                (ext_z & 0x0FU));
    s->ts = zy100_get_u16_be(&raw[OIS20_TMST_HIGH_IDX]);
}

static void ois20_pack_frame(const ois20_sample_t *s,
                             uint8_t raw[ZY100_OIS20_CMP_FRAME_BYTES])
{
    uint32_t ax = ois20_pack_signed20(s->ax);
    uint32_t ay = ois20_pack_signed20(s->ay);
    uint32_t az = ois20_pack_signed20(s->az);
    uint32_t gx = ois20_pack_signed20(s->gx);
    uint32_t gy = ois20_pack_signed20(s->gy);
    uint32_t gz = ois20_pack_signed20(s->gz);
    uint16_t temp = (uint16_t)(int16_t)s->temp;

    raw[OIS20_TEMP_HIGH_IDX] = (uint8_t)(temp >> 8);
    raw[OIS20_TEMP_LOW_IDX] = (uint8_t)(temp & 0xFFU);
    raw[OIS20_AX_HIGH_IDX] = (uint8_t)(ax >> 12);
    raw[OIS20_AX_LOW_IDX] = (uint8_t)(ax >> 4);
    raw[OIS20_AY_HIGH_IDX] = (uint8_t)(ay >> 12);
    raw[OIS20_AY_LOW_IDX] = (uint8_t)(ay >> 4);
    raw[OIS20_AZ_HIGH_IDX] = (uint8_t)(az >> 12);
    raw[OIS20_AZ_LOW_IDX] = (uint8_t)(az >> 4);
    raw[OIS20_GX_HIGH_IDX] = (uint8_t)(gx >> 12);
    raw[OIS20_GX_LOW_IDX] = (uint8_t)(gx >> 4);
    raw[OIS20_GY_HIGH_IDX] = (uint8_t)(gy >> 12);
    raw[OIS20_GY_LOW_IDX] = (uint8_t)(gy >> 4);
    raw[OIS20_GZ_HIGH_IDX] = (uint8_t)(gz >> 12);
    raw[OIS20_GZ_LOW_IDX] = (uint8_t)(gz >> 4);
    raw[OIS20_TMST_HIGH_IDX] = (uint8_t)(s->ts >> 8);
    raw[OIS20_TMST_LOW_IDX] = (uint8_t)(s->ts & 0xFFU);
    raw[OIS20_EXT_X_IDX] = (uint8_t)(((ax & 0x0FU) << 4) | (gx & 0x0FU));
    raw[OIS20_EXT_Y_IDX] = (uint8_t)(((ay & 0x0FU) << 4) | (gy & 0x0FU));
    raw[OIS20_EXT_Z_IDX] = (uint8_t)(((az & 0x0FU) << 4) | (gz & 0x0FU));
}

static bool ois20_frame_all_zero(const uint8_t *frame)
{
    uint32_t idx;

    for (idx = 0U; idx < ZY100_OIS20_CMP_FRAME_BYTES; idx++)
    {
        if (frame[idx] != 0U)
        {
            return false;
        }
    }
    return true;
}

static uint32_t ois20_count_lsb2_nonzero_samples(const ois20_sample_t *samples,
                                                 uint32_t frame_count)
{
    uint32_t count = 0U;
    uint32_t idx;

    for (idx = 0U; idx < frame_count; idx++)
    {
        if ((ois20_pack_signed20(samples[idx].ax) & 0x03U) != 0U)
        {
            count++;
        }
        if ((ois20_pack_signed20(samples[idx].ay) & 0x03U) != 0U)
        {
            count++;
        }
        if ((ois20_pack_signed20(samples[idx].az) & 0x03U) != 0U)
        {
            count++;
        }
        if ((ois20_pack_signed20(samples[idx].gx) & 0x03U) != 0U)
        {
            count++;
        }
        if ((ois20_pack_signed20(samples[idx].gy) & 0x03U) != 0U)
        {
            count++;
        }
        if ((ois20_pack_signed20(samples[idx].gz) & 0x03U) != 0U)
        {
            count++;
        }
    }
    return count;
}

static int32_t ois20_axis_to_e18(int32_t axis20)
{
    return axis20 / 4;
}

static int32_t ois20_e18_to_axis20(int32_t axis18)
{
    return axis18 * 4;
}

static int32_t ois20_sample_get_channel(const ois20_sample_t *s,
                                        uint32_t channel,
                                        uint8_t axis_pack_mode)
{
    switch (channel)
    {
    case OIS20_CH_TEMP:
        return s->temp;
    case OIS20_CH_AX:
        return (axis_pack_mode == ZY100_OIS20_CMP_AXIS_PACK_E18_Q2) ?
               ois20_axis_to_e18(s->ax) : s->ax;
    case OIS20_CH_AY:
        return (axis_pack_mode == ZY100_OIS20_CMP_AXIS_PACK_E18_Q2) ?
               ois20_axis_to_e18(s->ay) : s->ay;
    case OIS20_CH_AZ:
        return (axis_pack_mode == ZY100_OIS20_CMP_AXIS_PACK_E18_Q2) ?
               ois20_axis_to_e18(s->az) : s->az;
    case OIS20_CH_GX:
        return (axis_pack_mode == ZY100_OIS20_CMP_AXIS_PACK_E18_Q2) ?
               ois20_axis_to_e18(s->gx) : s->gx;
    case OIS20_CH_GY:
        return (axis_pack_mode == ZY100_OIS20_CMP_AXIS_PACK_E18_Q2) ?
               ois20_axis_to_e18(s->gy) : s->gy;
    case OIS20_CH_GZ:
        return (axis_pack_mode == ZY100_OIS20_CMP_AXIS_PACK_E18_Q2) ?
               ois20_axis_to_e18(s->gz) : s->gz;
    default:
        return 0;
    }
}

static void ois20_sample_set_channel(ois20_sample_t *s,
                                     uint32_t channel,
                                     uint8_t axis_pack_mode,
                                     int32_t value)
{
    int32_t axis_value = (axis_pack_mode == ZY100_OIS20_CMP_AXIS_PACK_E18_Q2) ?
                         ois20_e18_to_axis20(value) : value;

    switch (channel)
    {
    case OIS20_CH_TEMP:
        s->temp = value;
        break;
    case OIS20_CH_AX:
        s->ax = axis_value;
        break;
    case OIS20_CH_AY:
        s->ay = axis_value;
        break;
    case OIS20_CH_AZ:
        s->az = axis_value;
        break;
    case OIS20_CH_GX:
        s->gx = axis_value;
        break;
    case OIS20_CH_GY:
        s->gy = axis_value;
        break;
    case OIS20_CH_GZ:
        s->gz = axis_value;
        break;
    default:
        break;
    }
}

static void ois20_bit_writer_init(ois20_bit_writer_t *w,
                                  uint8_t *buf,
                                  uint32_t capacity_bytes)
{
    w->buf = buf;
    w->capacity_bytes = capacity_bytes;
    w->bit_pos = 0U;
    w->overflow = false;
}

static void ois20_bit_write(ois20_bit_writer_t *w,
                            uint32_t value,
                            uint8_t nbits)
{
    uint8_t idx;

    for (idx = 0U; idx < nbits; idx++)
    {
        uint32_t byte_index = w->bit_pos >> 3;
        uint32_t bit_index = w->bit_pos & 0x07U;

        if (byte_index >= w->capacity_bytes)
        {
            w->overflow = true;
            return;
        }
        if (((value >> idx) & 1U) != 0U)
        {
            w->buf[byte_index] |= (uint8_t)(1U << bit_index);
        }
        w->bit_pos++;
    }
}

static void ois20_bit_reader_init(ois20_bit_reader_t *r,
                                  const uint8_t *buf,
                                  uint32_t capacity_bytes)
{
    r->buf = buf;
    r->capacity_bytes = capacity_bytes;
    r->bit_pos = 0U;
    r->overflow = false;
}

static uint32_t ois20_bit_read(ois20_bit_reader_t *r, uint8_t nbits)
{
    uint32_t value = 0U;
    uint8_t idx;

    for (idx = 0U; idx < nbits; idx++)
    {
        uint32_t byte_index = r->bit_pos >> 3;
        uint32_t bit_index = r->bit_pos & 0x07U;

        if (byte_index >= r->capacity_bytes)
        {
            r->overflow = true;
            return value;
        }
        value |= (uint32_t)(((r->buf[byte_index] >> bit_index) & 1U) << idx);
        r->bit_pos++;
    }
    return value;
}

static bool ois20_write_zz_stream(uint8_t **cursor,
                                  const uint8_t *end,
                                  const uint32_t *values,
                                  uint32_t count,
                                  uint8_t bw)
{
    uint32_t bytes = ois20_bytes_for_bits(count, bw);
    ois20_bit_writer_t writer;
    uint32_t idx;

    if (bytes == 0U)
    {
        return true;
    }
    if (((uint32_t)(end - *cursor)) < bytes)
    {
        return false;
    }

    memset(*cursor, 0, bytes);
    ois20_bit_writer_init(&writer, *cursor, bytes);
    for (idx = 0U; idx < count; idx++)
    {
        ois20_bit_write(&writer, values[idx], bw);
        if (writer.overflow)
        {
            return false;
        }
    }
    *cursor += bytes;
    return true;
}

static bool ois20_read_zz_stream(const uint8_t **cursor,
                                 const uint8_t *end,
                                 uint32_t *values,
                                 uint32_t count,
                                 uint8_t bw)
{
    uint32_t bytes = ois20_bytes_for_bits(count, bw);
    ois20_bit_reader_t reader;
    uint32_t idx;

    if (bytes == 0U)
    {
        for (idx = 0U; idx < count; idx++)
        {
            values[idx] = 0U;
        }
        return true;
    }
    if (((uint32_t)(end - *cursor)) < bytes)
    {
        return false;
    }

    ois20_bit_reader_init(&reader, *cursor, bytes);
    for (idx = 0U; idx < count; idx++)
    {
        values[idx] = ois20_bit_read(&reader, bw);
        if (reader.overflow)
        {
            return false;
        }
    }
    *cursor += bytes;
    return true;
}

static void ois20_init_header(zy100_ois20_cmp_block_hdr_t *hdr,
                              uint8_t codec,
                              uint8_t frame_count,
                              uint32_t block_seq,
                              uint32_t first_sample_index,
                              uint16_t raw_len,
                              uint16_t payload_len,
                              uint16_t raw_crc16,
                              uint8_t axis_pack_mode)
{
    memset(hdr, 0, sizeof(*hdr));
    hdr->magic = ZY100_OIS20_CMP_MAGIC;
    hdr->version = ZY100_OIS20_CMP_VERSION;
    hdr->header_len = (uint8_t)sizeof(*hdr);
    hdr->codec = codec;
    hdr->frame_count = frame_count;
    hdr->block_seq = block_seq;
    hdr->first_sample_index = first_sample_index;
    hdr->raw_len = raw_len;
    hdr->payload_len = payload_len;
    hdr->raw_crc16 = raw_crc16;
    hdr->axis_pack_mode = axis_pack_mode;
    if (frame_count < ZY100_OIS20_CMP_BLOCK_FRAMES)
    {
        hdr->flags |= ZY100_OIS20_CMP_FLAG_TAIL_BLOCK;
    }
}

static void ois20_update_common_stats(zy100_ois20_block_store_t *ctx,
                                      uint8_t codec,
                                      uint32_t raw_len,
                                      uint32_t stored_len,
                                      uint32_t frame_count,
                                      uint32_t lsb2_nonzero,
                                      bool fallback)
{
    if (codec == ZY100_OIS20_CMP_CODEC_RAW20)
    {
        ctx->stats.raw_blocks++;
    }
    else
    {
        ctx->stats.cmp_blocks++;
    }
    if (fallback)
    {
        ctx->stats.fallback_count++;
    }
    ctx->stats.block_count++;
    ctx->stats.frame_count += frame_count;
    ctx->stats.raw_bytes += raw_len;
    ctx->stats.lsb2_nonzero_count += lsb2_nonzero;
    if (stored_len > ctx->stats.max_block_bytes)
    {
        ctx->stats.max_block_bytes = stored_len;
    }
    ctx->stats.stored_bytes = ctx->used_bytes;
}

static bool ois20_store_has_space(const zy100_ois20_block_store_t *ctx,
                                  uint32_t stored_len)
{
    return ((stored_len <= ctx->capacity_bytes) &&
            (ctx->used_bytes <= (ctx->capacity_bytes - stored_len)));
}

void zy100_ois20_block_store_init(zy100_ois20_block_store_t *ctx,
                                  uint8_t *store,
                                  uint32_t capacity_bytes)
{
    if (ctx == NULL)
    {
        return;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->store = store;
    ctx->capacity_bytes = capacity_bytes;
    ctx->stats.capacity_bytes = capacity_bytes;
}

void zy100_ois20_block_store_reset(zy100_ois20_block_store_t *ctx)
{
    uint8_t *store;
    uint32_t capacity_bytes;

    if (ctx == NULL)
    {
        return;
    }

    store = ctx->store;
    capacity_bytes = ctx->capacity_bytes;
    memset(ctx, 0, sizeof(*ctx));
    ctx->store = store;
    ctx->capacity_bytes = capacity_bytes;
    ctx->stats.capacity_bytes = capacity_bytes;
}

static bool ois20_commit_raw_block(zy100_ois20_block_store_t *ctx,
                                   const uint8_t *raw_frames,
                                   uint32_t frame_count,
                                   uint32_t first_sample_index,
                                   uint32_t lsb2_nonzero,
                                   bool fallback)
{
    zy100_ois20_cmp_block_hdr_t hdr;
    uint32_t raw_len;
    uint32_t stored_len;
    uint8_t *dst;

    raw_len = frame_count * ZY100_OIS20_CMP_FRAME_BYTES;
    stored_len = (uint32_t)sizeof(hdr) + raw_len;
    if (!ois20_store_has_space(ctx, stored_len))
    {
        ctx->stats.overrun_count++;
        return false;
    }

    ois20_init_header(&hdr,
                      ZY100_OIS20_CMP_CODEC_RAW20,
                      (uint8_t)frame_count,
                      ctx->next_block_seq,
                      first_sample_index,
                      (uint16_t)raw_len,
                      (uint16_t)raw_len,
                      ois20_crc16(raw_frames, raw_len),
                      ZY100_OIS20_CMP_AXIS_PACK_S20);
    if (lsb2_nonzero != 0U)
    {
        hdr.flags |= ZY100_OIS20_CMP_FLAG_LSB2_NONZERO;
    }
    if (frame_count >= 2U)
    {
        hdr.ts_dt1 = (uint16_t)(ois20_raw_tmst(&raw_frames[ZY100_OIS20_CMP_FRAME_BYTES]) -
                                ois20_raw_tmst(raw_frames));
    }

    dst = &ctx->store[ctx->used_bytes];
    memcpy(dst, &hdr, sizeof(hdr));
    memcpy(&dst[sizeof(hdr)], raw_frames, raw_len);

    ctx->used_bytes += stored_len;
    ctx->next_block_seq++;
    ois20_update_common_stats(ctx,
                              ZY100_OIS20_CMP_CODEC_RAW20,
                              raw_len,
                              stored_len,
                              frame_count,
                              lsb2_nonzero,
                              fallback);
    return true;
}

bool zy100_ois20_block_store_append_raw(zy100_ois20_block_store_t *ctx,
                                        const uint8_t *raw_frames,
                                        uint32_t frame_count,
                                        uint32_t first_sample_index)
{
    if ((ctx == NULL) || (ctx->store == NULL) || (raw_frames == NULL) ||
        (frame_count == 0U) ||
        (frame_count > ZY100_OIS20_CMP_BLOCK_FRAMES))
    {
        return false;
    }
    return ois20_commit_raw_block(ctx,
                                  raw_frames,
                                  frame_count,
                                  first_sample_index,
                                  0U,
                                  false);
}

static uint8_t ois20_select_axis_pack_mode(uint32_t lsb2_nonzero)
{
#if ZY100_OIS20_CMP_AXIS_E18_Q2_ENABLE
#ifdef ZY100_OIS20_CMP_FORCE_S20_ENABLE
    if (ZY100_OIS20_CMP_FORCE_S20_ENABLE)
    {
        return ZY100_OIS20_CMP_AXIS_PACK_S20;
    }
#endif
    if (lsb2_nonzero == 0U)
    {
        return ZY100_OIS20_CMP_AXIS_PACK_E18_Q2;
    }
#else
    (void)lsb2_nonzero;
#endif
    return ZY100_OIS20_CMP_AXIS_PACK_S20;
}

static void ois20_compute_channel_stream(const ois20_sample_t *samples,
                                         uint32_t frame_count,
                                         uint8_t axis_pack_mode,
                                         uint32_t channel,
                                         uint32_t *zz_values,
                                         uint8_t *bw)
{
    int32_t prev = ois20_sample_get_channel(&samples[0],
                                            channel,
                                            axis_pack_mode);
    uint32_t max_zz = 0U;
    uint32_t idx;

    for (idx = 1U; idx < frame_count; idx++)
    {
        int32_t cur = ois20_sample_get_channel(&samples[idx],
                                               channel,
                                               axis_pack_mode);
        uint32_t zz = ois20_zigzag32(cur - prev);

        zz_values[idx - 1U] = zz;
        if (zz > max_zz)
        {
            max_zz = zz;
        }
        prev = cur;
    }
    *bw = ois20_bits_required_u32(max_zz);
}

static void ois20_compute_ts_stream(const ois20_sample_t *samples,
                                    uint32_t frame_count,
                                    uint32_t *zz_values,
                                    uint8_t *bw,
                                    uint16_t *ts_dt1)
{
    uint16_t prev_dt;
    uint32_t max_zz = 0U;
    uint32_t idx;

    *bw = 0U;
    *ts_dt1 = 0U;
    if (frame_count < 2U)
    {
        return;
    }

    prev_dt = (uint16_t)(samples[1].ts - samples[0].ts);
    *ts_dt1 = prev_dt;
    for (idx = 2U; idx < frame_count; idx++)
    {
        uint16_t dt = (uint16_t)(samples[idx].ts - samples[idx - 1U].ts);
        int32_t ddt = (int32_t)(int16_t)(dt - prev_dt);
        uint32_t zz = ois20_zigzag32(ddt);

        zz_values[idx - 2U] = zz;
        if (zz > max_zz)
        {
            max_zz = zz;
        }
        prev_dt = dt;
    }
    *bw = ois20_bits_required_u32(max_zz);
}

static bool ois20_try_commit_dzbp(zy100_ois20_block_store_t *ctx,
                                  const uint8_t *raw_frames,
                                  uint32_t frame_count,
                                  uint32_t first_sample_index,
                                  const ois20_sample_t *samples,
                                  uint32_t lsb2_nonzero)
{
    uint32_t zz[OIS20_CHANNEL_COUNT][OIS20_VALUE_MAX_COUNT];
    uint32_t ts_zz[OIS20_TS_DDT_MAX_COUNT];
    uint8_t bw[OIS20_CHANNEL_COUNT];
    uint8_t bw_ts;
    uint8_t axis_pack_mode;
    uint16_t ts_dt1;
    uint32_t value_count;
    uint32_t ts_count;
    uint32_t payload_len;
    uint32_t raw_len;
    uint32_t stored_len;
    uint32_t raw_total_len;
    uint8_t *dst;
    uint8_t *payload;
    uint8_t *cursor;
    uint8_t *end;
    uint32_t channel;
    uint8_t decoded[ZY100_OIS20_CMP_BLOCK_FRAMES][ZY100_OIS20_CMP_FRAME_BYTES];
    zy100_ois20_decomp_result_t result;
    zy100_ois20_cmp_block_hdr_t hdr;

    axis_pack_mode = ois20_select_axis_pack_mode(lsb2_nonzero);
    value_count = frame_count - 1U;
    ts_count = (frame_count >= 3U) ? (frame_count - 2U) : 0U;
    raw_len = frame_count * ZY100_OIS20_CMP_FRAME_BYTES;

    payload_len = ZY100_OIS20_CMP_FRAME_BYTES;
    for (channel = 0U; channel < OIS20_CHANNEL_COUNT; channel++)
    {
        ois20_compute_channel_stream(samples,
                                     frame_count,
                                     axis_pack_mode,
                                     channel,
                                     zz[channel],
                                     &bw[channel]);
        payload_len += ois20_bytes_for_bits(value_count, bw[channel]);
    }
    ois20_compute_ts_stream(samples, frame_count, ts_zz, &bw_ts, &ts_dt1);
    payload_len += ois20_bytes_for_bits(ts_count, bw_ts);

    raw_total_len = (uint32_t)sizeof(hdr) + raw_len;
    stored_len = (uint32_t)sizeof(hdr) + payload_len;
    if ((stored_len + OIS20_DZBP_MARGIN_BYTES) >= raw_total_len)
    {
        return false;
    }
    if ((payload_len > 0xFFFFU) || !ois20_store_has_space(ctx, stored_len))
    {
        return false;
    }

    ois20_init_header(&hdr,
                      ZY100_OIS20_CMP_CODEC_DZBP,
                      (uint8_t)frame_count,
                      ctx->next_block_seq,
                      first_sample_index,
                      (uint16_t)raw_len,
                      (uint16_t)payload_len,
                      ois20_crc16(raw_frames, raw_len),
                      axis_pack_mode);
    hdr.bw_temp = bw[OIS20_CH_TEMP];
    hdr.bw_ax = bw[OIS20_CH_AX];
    hdr.bw_ay = bw[OIS20_CH_AY];
    hdr.bw_az = bw[OIS20_CH_AZ];
    hdr.bw_gx = bw[OIS20_CH_GX];
    hdr.bw_gy = bw[OIS20_CH_GY];
    hdr.bw_gz = bw[OIS20_CH_GZ];
    hdr.bw_ts_ddt = bw_ts;
    hdr.ts_dt1 = ts_dt1;
    if (lsb2_nonzero != 0U)
    {
        hdr.flags |= ZY100_OIS20_CMP_FLAG_LSB2_NONZERO;
    }

    dst = &ctx->store[ctx->used_bytes];
    payload = &dst[sizeof(hdr)];
    cursor = payload;
    end = &payload[payload_len];
    memset(payload, 0, payload_len);
    memcpy(dst, &hdr, sizeof(hdr));
    memcpy(cursor, raw_frames, ZY100_OIS20_CMP_FRAME_BYTES);
    cursor += ZY100_OIS20_CMP_FRAME_BYTES;

    for (channel = 0U; channel < OIS20_CHANNEL_COUNT; channel++)
    {
        if (!ois20_write_zz_stream(&cursor,
                                   end,
                                   zz[channel],
                                   value_count,
                                   bw[channel]))
        {
            return false;
        }
    }
    if (!ois20_write_zz_stream(&cursor,
                               end,
                               ts_zz,
                               ts_count,
                               hdr.bw_ts_ddt))
    {
        return false;
    }
    if (cursor != end)
    {
        return false;
    }

    memset(&result, 0, sizeof(result));
    if (!zy100_ois20_decompress_block(dst, stored_len, decoded, &result) ||
        (result.decoded_bytes != raw_len) ||
        (memcmp(raw_frames, decoded, raw_len) != 0))
    {
        return false;
    }

    ctx->used_bytes += stored_len;
    ctx->next_block_seq++;
    ois20_update_common_stats(ctx,
                              ZY100_OIS20_CMP_CODEC_DZBP,
                              raw_len,
                              stored_len,
                              frame_count,
                              lsb2_nonzero,
                              false);
    return true;
}

bool zy100_ois20_block_store_append_auto_codec(zy100_ois20_block_store_t *ctx,
                                               const uint8_t *raw_frames,
                                               uint32_t frame_count,
                                               uint32_t first_sample_index)
{
    ois20_sample_t samples[ZY100_OIS20_CMP_BLOCK_FRAMES];
    uint32_t idx;
    uint32_t lsb2_nonzero;

    if ((ctx == NULL) || (ctx->store == NULL) || (raw_frames == NULL) ||
        (frame_count == 0U) ||
        (frame_count > ZY100_OIS20_CMP_BLOCK_FRAMES))
    {
        return false;
    }

    for (idx = 0U; idx < frame_count; idx++)
    {
        ois20_parse_frame(&raw_frames[idx * ZY100_OIS20_CMP_FRAME_BYTES],
                          &samples[idx]);
    }
    lsb2_nonzero = ois20_count_lsb2_nonzero_samples(samples, frame_count);

#if ZY100_OIS20_COMPRESSION_ENABLE
    if (ois20_try_commit_dzbp(ctx,
                              raw_frames,
                              frame_count,
                              first_sample_index,
                              samples,
                              lsb2_nonzero))
    {
        return true;
    }
#endif

#if ZY100_OIS20_CMP_RAW_FALLBACK_ENABLE
    return ois20_commit_raw_block(ctx,
                                  raw_frames,
                                  frame_count,
                                  first_sample_index,
                                  lsb2_nonzero,
                                  true);
#else
    ctx->stats.overrun_count++;
    return false;
#endif
}

bool zy100_ois20_block_store_finalize_raw(zy100_ois20_block_store_t *ctx)
{
    if ((ctx == NULL) || (ctx->store == NULL))
    {
        return false;
    }
    return true;
}

void zy100_ois20_block_store_get_stats(const zy100_ois20_block_store_t *ctx,
                                       zy100_ois20_cmp_stats_t *out)
{
    if ((ctx == NULL) || (out == NULL))
    {
        return;
    }

    *out = ctx->stats;
    out->stored_bytes = ctx->used_bytes;
    out->capacity_bytes = ctx->capacity_bytes;
}

static uint8_t ois20_hdr_bw_for_channel(const zy100_ois20_cmp_block_hdr_t *hdr,
                                        uint32_t channel)
{
    switch (channel)
    {
    case OIS20_CH_TEMP:
        return hdr->bw_temp;
    case OIS20_CH_AX:
        return hdr->bw_ax;
    case OIS20_CH_AY:
        return hdr->bw_ay;
    case OIS20_CH_AZ:
        return hdr->bw_az;
    case OIS20_CH_GX:
        return hdr->bw_gx;
    case OIS20_CH_GY:
        return hdr->bw_gy;
    case OIS20_CH_GZ:
        return hdr->bw_gz;
    default:
        return 0U;
    }
}

static bool ois20_decode_dzbp_payload(const zy100_ois20_cmp_block_hdr_t *hdr,
                                      const uint8_t *payload,
                                      uint32_t payload_len,
                                      uint8_t out_frames[][ZY100_OIS20_CMP_FRAME_BYTES])
{
    ois20_sample_t samples[ZY100_OIS20_CMP_BLOCK_FRAMES];
    uint32_t zz[OIS20_VALUE_MAX_COUNT];
    uint32_t ts_zz[OIS20_TS_DDT_MAX_COUNT];
    const uint8_t *cursor;
    const uint8_t *end;
    uint32_t frame_count = hdr->frame_count;
    uint32_t value_count = frame_count - 1U;
    uint32_t ts_count = (frame_count >= 3U) ? (frame_count - 2U) : 0U;
    uint32_t channel;
    uint32_t idx;

    if (payload_len < ZY100_OIS20_CMP_FRAME_BYTES)
    {
        return false;
    }

    ois20_parse_frame(payload, &samples[0]);
    cursor = &payload[ZY100_OIS20_CMP_FRAME_BYTES];
    end = &payload[payload_len];

    for (channel = 0U; channel < OIS20_CHANNEL_COUNT; channel++)
    {
        int32_t prev = ois20_sample_get_channel(&samples[0],
                                                channel,
                                                hdr->axis_pack_mode);
        uint8_t bw = ois20_hdr_bw_for_channel(hdr, channel);

        if (!ois20_read_zz_stream(&cursor, end, zz, value_count, bw))
        {
            return false;
        }
        for (idx = 1U; idx < frame_count; idx++)
        {
            int32_t cur = prev + ois20_unzigzag32(zz[idx - 1U]);

            ois20_sample_set_channel(&samples[idx],
                                     channel,
                                     hdr->axis_pack_mode,
                                     cur);
            prev = cur;
        }
    }

    samples[0].ts = ois20_raw_tmst(payload);
    if (frame_count >= 2U)
    {
        uint16_t prev_dt = hdr->ts_dt1;

        samples[1].ts = (uint16_t)(samples[0].ts + prev_dt);
        if (!ois20_read_zz_stream(&cursor,
                                  end,
                                  ts_zz,
                                  ts_count,
                                  hdr->bw_ts_ddt))
        {
            return false;
        }
        for (idx = 2U; idx < frame_count; idx++)
        {
            int32_t ddt = ois20_unzigzag32(ts_zz[idx - 2U]);
            uint16_t dt = (uint16_t)(prev_dt + (int16_t)ddt);

            samples[idx].ts = (uint16_t)(samples[idx - 1U].ts + dt);
            prev_dt = dt;
        }
    }
    else if (!ois20_read_zz_stream(&cursor,
                                   end,
                                   ts_zz,
                                   ts_count,
                                   hdr->bw_ts_ddt))
    {
        return false;
    }

    if (cursor != end)
    {
        return false;
    }

    for (idx = 0U; idx < frame_count; idx++)
    {
        ois20_pack_frame(&samples[idx], out_frames[idx]);
    }
    return true;
}

bool zy100_ois20_decompress_block(
    const uint8_t *block,
    uint32_t block_len,
    uint8_t out_frames[][ZY100_OIS20_CMP_FRAME_BYTES],
    zy100_ois20_decomp_result_t *result)
{
    zy100_ois20_cmp_block_hdr_t hdr;
    const uint8_t *payload;
    uint32_t raw_len;
    uint32_t total_len;

    if (result == NULL)
    {
        return false;
    }
    memset(result, 0, sizeof(*result));
    if ((block == NULL) || (out_frames == NULL) ||
        (block_len < sizeof(zy100_ois20_cmp_block_hdr_t)))
    {
        result->payload_bad++;
        return false;
    }

    memcpy(&hdr, block, sizeof(hdr));
    result->codec = hdr.codec;
    result->frame_count = hdr.frame_count;
    if ((hdr.magic != ZY100_OIS20_CMP_MAGIC) ||
        (hdr.version != ZY100_OIS20_CMP_VERSION) ||
        (hdr.header_len != sizeof(zy100_ois20_cmp_block_hdr_t)) ||
        (hdr.frame_count == 0U) ||
        (hdr.frame_count > ZY100_OIS20_CMP_BLOCK_FRAMES))
    {
        result->payload_bad++;
        return false;
    }

    raw_len = ((uint32_t)hdr.frame_count) * ZY100_OIS20_CMP_FRAME_BYTES;
    total_len = (uint32_t)hdr.header_len + hdr.payload_len;
    if ((hdr.raw_len != raw_len) ||
        (total_len > block_len) ||
        (total_len < hdr.header_len))
    {
        result->payload_bad++;
        return false;
    }

    payload = &block[hdr.header_len];
    if (hdr.codec == ZY100_OIS20_CMP_CODEC_RAW20)
    {
        if (hdr.payload_len != raw_len)
        {
            result->payload_bad++;
            return false;
        }
        memcpy(out_frames, payload, raw_len);
    }
    else if (hdr.codec == ZY100_OIS20_CMP_CODEC_DZBP)
    {
        if (!ois20_decode_dzbp_payload(&hdr,
                                       payload,
                                       hdr.payload_len,
                                       out_frames))
        {
            result->decode_fail++;
            return false;
        }
    }
    else
    {
        result->decode_fail++;
        return false;
    }

    result->decoded_frames = hdr.frame_count;
    result->decoded_bytes = raw_len;
    if (ois20_crc16((const uint8_t *)out_frames, raw_len) != hdr.raw_crc16)
    {
        result->crc_fail++;
        return false;
    }
    return true;
}

static void ois20_verify_frame(zy100_ois20_decomp_verify_t *out,
                               const uint8_t *frame,
                               bool *have_prev,
                               uint16_t *prev_tmst,
                               uint32_t dt_min_us,
                               uint32_t dt_max_us)
{
    uint16_t cur_tmst;

    if (ois20_frame_all_zero(frame))
    {
        out->zero++;
    }

    cur_tmst = ois20_raw_tmst(frame);
    if (!(*have_prev))
    {
        out->first_tmst_raw = cur_tmst;
        out->last_tmst_raw = cur_tmst;
        *prev_tmst = cur_tmst;
        *have_prev = true;
        out->frames++;
        return;
    }

    {
        uint16_t delta = (uint16_t)(cur_tmst - *prev_tmst);

        out->checked++;
        if (cur_tmst < *prev_tmst)
        {
            out->wrap++;
        }
        if (delta == 0U)
        {
            out->zero++;
        }
        if (delta < out->dt_min)
        {
            out->dt_min = delta;
        }
        if (delta > out->dt_max)
        {
            out->dt_max = delta;
        }
        if ((delta < dt_min_us) || (delta > dt_max_us))
        {
            out->bad++;
        }
        *prev_tmst = cur_tmst;
        out->last_tmst_raw = cur_tmst;
        out->frames++;
    }
}

bool zy100_ois20_block_store_verify(const zy100_ois20_block_store_t *ctx,
                                    uint32_t expected_frames,
                                    uint32_t expected_blocks,
                                    uint32_t dt_min_us,
                                    uint32_t dt_max_us,
                                    zy100_ois20_decomp_verify_t *out)
{
    uint32_t offset = 0U;
    uint32_t expected_seq = 0U;
    bool have_prev = false;
    uint16_t prev_tmst = 0U;
    uint8_t decoded[ZY100_OIS20_CMP_BLOCK_FRAMES][ZY100_OIS20_CMP_FRAME_BYTES];

    if (out == NULL)
    {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->dt_min = 0xFFFFFFFFU;
    if ((ctx == NULL) || (ctx->store == NULL))
    {
        out->dt_min = 0U;
        return false;
    }

    while (offset < ctx->used_bytes)
    {
        zy100_ois20_cmp_block_hdr_t hdr;
        uint32_t block_len;
        uint32_t frame_idx;
        zy100_ois20_decomp_result_t result;

        if ((ctx->used_bytes - offset) < sizeof(zy100_ois20_cmp_block_hdr_t))
        {
            out->payload_bad++;
            break;
        }

        memcpy(&hdr, &ctx->store[offset], sizeof(hdr));
        if ((hdr.magic != ZY100_OIS20_CMP_MAGIC) ||
            (hdr.version != ZY100_OIS20_CMP_VERSION) ||
            (hdr.header_len != sizeof(zy100_ois20_cmp_block_hdr_t)))
        {
            out->magic_bad++;
            break;
        }
        if ((hdr.codec != ZY100_OIS20_CMP_CODEC_RAW20) &&
            (hdr.codec != ZY100_OIS20_CMP_CODEC_DZBP))
        {
            out->codec_bad++;
            break;
        }
        if (hdr.block_seq != expected_seq)
        {
            out->block_seq_bad++;
        }
        if ((hdr.frame_count == 0U) ||
            (hdr.frame_count > ZY100_OIS20_CMP_BLOCK_FRAMES))
        {
            out->frame_count_bad++;
            break;
        }

        block_len = (uint32_t)hdr.header_len + hdr.payload_len;
        if ((block_len > (ctx->used_bytes - offset)) ||
            (block_len < hdr.header_len))
        {
            out->payload_bad++;
            break;
        }

        memset(&result, 0, sizeof(result));
        (void)zy100_ois20_decompress_block(&ctx->store[offset],
                                           block_len,
                                           decoded,
                                           &result);
        out->crc_fail += result.crc_fail;
        out->decode_fail += result.decode_fail;
        out->payload_bad += result.payload_bad;
        if (result.decoded_frames != hdr.frame_count)
        {
            out->frame_count_bad++;
        }

        for (frame_idx = 0U; frame_idx < result.decoded_frames; frame_idx++)
        {
            ois20_verify_frame(out,
                               decoded[frame_idx],
                               &have_prev,
                               &prev_tmst,
                               dt_min_us,
                               dt_max_us);
        }

        out->blocks++;
        expected_seq++;
        offset += block_len;
    }

    if (out->checked == 0U)
    {
        out->dt_min = 0U;
    }

    out->pass = ((out->frames == expected_frames) &&
                 (out->checked == (expected_frames - 1U)) &&
                 (out->blocks == expected_blocks) &&
                 (out->bad == 0U) &&
                 (out->zero == 0U) &&
                 (out->dt_min >= dt_min_us) &&
                 (out->dt_max <= dt_max_us) &&
                 (out->block_seq_bad == 0U) &&
                 (out->magic_bad == 0U) &&
                 (out->codec_bad == 0U) &&
                 (out->frame_count_bad == 0U) &&
                 (out->crc_fail == 0U) &&
                 (out->decode_fail == 0U) &&
                 (out->payload_bad == 0U)) ? 1U : 0U;
    return (out->pass != 0U);
}
