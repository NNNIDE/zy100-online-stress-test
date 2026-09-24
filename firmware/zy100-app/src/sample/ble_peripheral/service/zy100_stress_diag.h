#ifndef ZY100_STRESS_DIAG_H
#define ZY100_STRESS_DIAG_H
#include <stdbool.h>
#include <stdint.h>

/* Capture-worker-owned observations; stage dwell includes scheduler gaps. */
#define ZY100_STRESS_DIAG_RAM_LIMIT 512U
typedef enum
{
    ZY100_SD_OTHER = 0, ZY100_SD_RAW_WRITE, ZY100_SD_RAW_COMMIT,
    ZY100_SD_ERASE, ZY100_SD_RESERVE, ZY100_SD_PROGRAM,
    ZY100_SD_VERIFY, ZY100_SD_COMMIT, ZY100_SD_PAIR,
    ZY100_SD_COUNT
} zy100_stress_diag_stage_t;
void zy100_stress_diag_service(uint32_t steps, uint32_t elapsed_us, bool wait, bool clock_missing);
void zy100_stress_diag_erase(bool urgent, bool issued, uint32_t ahead);
void zy100_stress_diag_prepare(uint32_t erase_count);
void zy100_stress_diag_begin(uint32_t session, uint32_t now, uint32_t rate,
                              uint32_t erase_count);
void zy100_stress_diag_stage(zy100_stress_diag_stage_t stage, uint32_t now,
                              uint32_t water);
void zy100_stress_diag_water(uint32_t now, uint32_t water);
void zy100_stress_diag_pump(uint32_t now);
void zy100_stress_diag_release(uint32_t now, uint32_t before, uint32_t bytes);
void zy100_stress_diag_record_begin(uint32_t kind, uint32_t now, uint32_t pages);
void zy100_stress_diag_record_page(uint32_t page);
void zy100_stress_diag_record_end(uint32_t now);
void zy100_stress_diag_freeze(uint32_t now, uint32_t water, uint32_t error);
uint32_t zy100_stress_diag_erases(uint32_t cumulative);
/* Spool-owned I/O observations. kind: 0 program, 1 readback, 2 transmit read,
 * 3 status read. Times use the existing 1 MHz down counter, not CPU cycles. */
void zy100_stress_diag_io_note(uint32_t kind, uint32_t elapsed_us, bool valid);
void zy100_stress_diag_io_issue(uint32_t counter, bool valid);
void zy100_stress_diag_io_poll(uint32_t counter, bool valid, bool bus, bool wip, bool ok);
void zy100_stress_diag_io_bus(void);
/* One line per call; true after END, or if this is not the frozen session. */
bool zy100_stress_diag_log_step(uint32_t session, uint32_t index);
#endif
