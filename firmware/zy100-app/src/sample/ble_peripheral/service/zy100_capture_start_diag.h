#ifndef ZY100_CAPTURE_START_DIAG_H
#define ZY100_CAPTURE_START_DIAG_H

#include "../driver/icm53611_driver.h"

/* Task-context snapshot only; no register reads or cleanup here. mode: 0 offline,
 * 1 online. id is the offline completion generation or online capture round. */
#if defined(ZY100_BUILD_PRODUCTION) && ZY100_BUILD_PRODUCTION
void zy100_capture_start_log(uint8_t mode, uint32_t id,
                             const icm53611_capture_result_t *r);
#else
static inline void zy100_capture_start_log(uint8_t mode, uint32_t id,
                                           const icm53611_capture_result_t *r)
{
    ZY100_LOG_ERROR("[CAP_START_A] mode=%u id=%lu phase=%u st=%u op=%u",
               mode, (unsigned long)id, r->stage, r->status, r->io.operation);
    ZY100_LOG_ERROR("[CAP_START_B] bank=%u reg=%02x valid=%u val=%02x io=%u acq=%u rel=%u us=%lu polls=%u end=%u",
               r->io.space, r->io.reg, r->io.read_valid, r->io.value,
               r->io.status, r->io.acquire_status, r->io.release_status,
               (unsigned long)r->flush_us, r->flush_polls, r->flush_end);
}
#endif

#endif
