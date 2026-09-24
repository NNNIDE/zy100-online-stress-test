#ifndef ZY100_ONLINE_STRESS_H
#define ZY100_ONLINE_STRESS_H
#include <stdbool.h>
#include <stdint.h>
#include "zy100_online_raw_capture.h"
#include "zy100_stress_diag.h"

#define ZY100_STRESS_STATUS_BYTES 164U
#define ZY100_STRESS_STATUS_WORDS 40U

bool zy100_online_stress_command(uint8_t conn, uint32_t operation,
                                  uint64_t argument, uint32_t *detail);
void zy100_online_stress_negotiate(uint8_t conn, uint32_t capabilities);
bool zy100_online_stress_armed(uint8_t conn);
void zy100_online_stress_invalidate(void);
void zy100_online_stress_session(uint32_t session);
bool zy100_online_stress_begin(uint8_t *workspace, uint32_t now_ms);
bool zy100_online_stress_tick(uint32_t now_ms);
bool zy100_online_stress_freeze(uint32_t now_ms);
bool zy100_online_stress_pending(void);
bool zy100_online_stress_writer_busy(void);
bool zy100_online_stress_write_step(void);
/* Phase/page cursor ignores prepared-on-BUSY bookkeeping. */
uint32_t zy100_online_stress_writer_cursor(void);
bool zy100_online_stress_erase_window(void);
bool zy100_online_stress_io_waiting(void);
void zy100_online_stress_diag_stage(zy100_stress_diag_stage_t stage);
void zy100_online_stress_diag_freeze(uint32_t error);
void zy100_online_stress_release(bool discard);
void zy100_online_stress_publish(uint32_t now_ms,
                                  const zy100_online_raw_capture_stats_t *raw,
                                  bool force);
bool zy100_online_stress_status_due(void);
void zy100_online_stress_encode(uint8_t *dst);
void zy100_online_stress_status_sent(uint32_t elapsed_ms);
void zy100_online_stress_terminal_freeze(void);
void zy100_online_stress_failure(uint32_t stop_reason);
void zy100_online_stress_ack(uint32_t bytes);
void zy100_online_stress_transport(uint32_t stop_reason, uint32_t pending,
    uint32_t pending_hi, uint32_t ack_max, uint32_t erase_count,
    uint32_t wrap_count, uint32_t heap_data, uint32_t heap_buffer,
    uint32_t mtu, uint32_t ci, uint32_t phy);
void zy100_online_stress_note_wait(bool send_wait, bool ack_wait);
void zy100_online_stress_note_stacks(uint32_t app_stack, uint32_t worker_stack,
                                     uint32_t worker_gap);
#endif
