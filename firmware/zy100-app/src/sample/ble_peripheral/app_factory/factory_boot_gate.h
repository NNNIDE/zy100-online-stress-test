#ifndef FACTORY_BOOT_GATE_H
#define FACTORY_BOOT_GATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "../app_mfg/zp_mfg_record.h"

typedef enum
{
    FACTORY_BOOT_GATE_MODE_FACTORY = 0U,
    FACTORY_BOOT_GATE_MODE_LOCKED_USER,
} factory_boot_gate_mode_t;

void factory_boot_gate_init(void);
factory_boot_gate_mode_t factory_boot_gate_mode(void);
bool factory_boot_gate_factory_mode_active(void);
bool factory_boot_gate_locked_user_mode_active(void);
bool factory_boot_gate_locked_record_invalid(void);
bool factory_boot_gate_production_blocked(void);
const zp_mfg_record_t *factory_boot_gate_record(void);
const char *factory_boot_gate_reason(void);

#ifdef __cplusplus
}
#endif

#endif /* FACTORY_BOOT_GATE_H */
