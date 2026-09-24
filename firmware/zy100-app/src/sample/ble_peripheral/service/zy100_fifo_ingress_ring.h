#ifndef ZY100_FIFO_INGRESS_RING_H
#define ZY100_FIFO_INGRESS_RING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define ZY100_FIFO_INGRESS_PACKET_BYTES 16U

void zy100_fifo_ingress_reset(void);
uint32_t zy100_fifo_ingress_level_bytes(void);
uint32_t zy100_fifo_ingress_free_bytes(void);
uint32_t zy100_fifo_ingress_max_level_bytes(void);
bool zy100_fifo_ingress_can_push_packets(uint32_t packet_count);
bool zy100_fifo_ingress_push_packet(const uint8_t packet16[ZY100_FIFO_INGRESS_PACKET_BYTES]);
bool zy100_fifo_ingress_peek_packets(uint8_t *out, uint32_t packet_count);
bool zy100_fifo_ingress_drop_packets(uint32_t packet_count);
uint32_t zy100_fifo_ingress_overflow_count(void);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_FIFO_INGRESS_RING_H */
