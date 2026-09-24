#ifndef ZY100_PRODUCTION_SHIPPING_H
#define ZY100_PRODUCTION_SHIPPING_H

#include <stdbool.h>
#include <stdint.h>

void zy100_production_shipping_boot_init(void);
bool zy100_production_shipping_boot_blocked(void);
bool zy100_production_shipping_boot_wake_pending_take(void);
bool zy100_production_shipping_active(void);
bool zy100_production_shipping_arm(uint8_t conn_id,
                                   uint8_t seq,
                                   uint32_t user_id,
                                   uint32_t training_id);
void zy100_production_shipping_poll(uint64_t runtime_ms);
void zy100_production_shipping_on_disconnected(uint8_t conn_id);

#endif
