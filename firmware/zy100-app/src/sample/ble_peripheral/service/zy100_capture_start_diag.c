#include "trace.h"
#include "../app_flags.h"
#include "zy100_capture_start_diag.h"

/* Shared cold-path formatting; callers retain their original failure gates. */
void zy100_capture_start_log(uint8_t mode, uint32_t id,
                                           const icm53611_capture_result_t *r)
{
    ZY100_LOG_ERROR("[CAP_START_A] mode=%u id=%lu phase=%u st=%u op=%u",
               mode, (unsigned long)id, r->stage, r->status, r->io.operation);
    ZY100_LOG_ERROR("[CAP_START_B] bank=%u reg=%02x valid=%u val=%02x io=%u acq=%u rel=%u us=%lu polls=%u end=%u",
               r->io.space, r->io.reg, r->io.read_valid, r->io.value,
               r->io.status, r->io.acquire_status, r->io.release_status,
               (unsigned long)r->flush_us, r->flush_polls, r->flush_end);
}
