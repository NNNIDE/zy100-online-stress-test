#include "zy100_fifo_ingress_ring.h"

#include <string.h>

#include "../app_flags.h"

#if ((ZY100_FIFO_INGRESS_RING_BYTES % ZY100_FIFO_INGRESS_PACKET_BYTES) != 0U)
#error "Ingress ring must be packet aligned"
#endif


static uint8_t s_ingress_buf[ZY100_FIFO_INGRESS_RING_BYTES];
static uint32_t s_ingress_read_pos;
static uint32_t s_ingress_write_pos;
static uint32_t s_ingress_level;
static uint32_t s_ingress_max_level;
static uint32_t s_ingress_overflow_count;

static uint32_t ingress_packet_bytes(uint32_t packet_count)
{
    return packet_count * ZY100_FIFO_INGRESS_PACKET_BYTES;
}

static uint32_t ingress_advance(uint32_t pos, uint32_t delta)
{
    pos += delta;
    if (pos >= ZY100_FIFO_INGRESS_RING_BYTES)
    {
        pos -= ZY100_FIFO_INGRESS_RING_BYTES;
    }
    return pos;
}

void zy100_fifo_ingress_reset(void)
{
    s_ingress_read_pos = 0U;
    s_ingress_write_pos = 0U;
    s_ingress_level = 0U;
    s_ingress_max_level = 0U;
    s_ingress_overflow_count = 0U;
}

uint32_t zy100_fifo_ingress_level_bytes(void)
{
    return s_ingress_level;
}

uint32_t zy100_fifo_ingress_free_bytes(void)
{
    return ZY100_FIFO_INGRESS_RING_BYTES - s_ingress_level;
}

uint32_t zy100_fifo_ingress_max_level_bytes(void)
{
    return s_ingress_max_level;
}

bool zy100_fifo_ingress_can_push_packets(uint32_t packet_count)
{
    return ingress_packet_bytes(packet_count) <= zy100_fifo_ingress_free_bytes();
}

bool zy100_fifo_ingress_push_packet(const uint8_t packet16[ZY100_FIFO_INGRESS_PACKET_BYTES])
{
    uint32_t first_part;

    if ((packet16 == NULL) || !zy100_fifo_ingress_can_push_packets(1U))
    {
        if (s_ingress_overflow_count < 0xFFFFFFFFU)
        {
            s_ingress_overflow_count++;
        }
        return false;
    }

    first_part = ZY100_FIFO_INGRESS_RING_BYTES - s_ingress_write_pos;
    if (first_part >= ZY100_FIFO_INGRESS_PACKET_BYTES)
    {
        memcpy(&s_ingress_buf[s_ingress_write_pos],
               packet16,
               ZY100_FIFO_INGRESS_PACKET_BYTES);
    }
    else
    {
        memcpy(&s_ingress_buf[s_ingress_write_pos], packet16, first_part);
        memcpy(&s_ingress_buf[0],
               &packet16[first_part],
               ZY100_FIFO_INGRESS_PACKET_BYTES - first_part);
    }

    s_ingress_write_pos = ingress_advance(s_ingress_write_pos,
                                          ZY100_FIFO_INGRESS_PACKET_BYTES);
    s_ingress_level += ZY100_FIFO_INGRESS_PACKET_BYTES;
    if (s_ingress_level > s_ingress_max_level)
    {
        s_ingress_max_level = s_ingress_level;
    }
    return true;
}

bool zy100_fifo_ingress_peek_packets(uint8_t *out, uint32_t packet_count)
{
    uint32_t bytes;
    uint32_t first_part;

    if (out == NULL)
    {
        return false;
    }

    bytes = ingress_packet_bytes(packet_count);
    if ((bytes == 0U) || (bytes > s_ingress_level))
    {
        return false;
    }

    first_part = ZY100_FIFO_INGRESS_RING_BYTES - s_ingress_read_pos;
    if (first_part >= bytes)
    {
        memcpy(out, &s_ingress_buf[s_ingress_read_pos], bytes);
    }
    else
    {
        memcpy(out, &s_ingress_buf[s_ingress_read_pos], first_part);
        memcpy(&out[first_part], &s_ingress_buf[0], bytes - first_part);
    }
    return true;
}

bool zy100_fifo_ingress_drop_packets(uint32_t packet_count)
{
    uint32_t bytes = ingress_packet_bytes(packet_count);

    if ((bytes == 0U) || (bytes > s_ingress_level))
    {
        return false;
    }

    s_ingress_read_pos = ingress_advance(s_ingress_read_pos, bytes);
    s_ingress_level -= bytes;
    if (s_ingress_level == 0U)
    {
        s_ingress_read_pos = 0U;
        s_ingress_write_pos = 0U;
    }
    return true;
}

uint32_t zy100_fifo_ingress_overflow_count(void)
{
    return s_ingress_overflow_count;
}
