#ifndef ZY100_POWER_DIAG_H
#define ZY100_POWER_DIAG_H
#include <stdbool.h>
#include <stdint.h>
#include "app_flags.h"

/* Observation only: no business decision may depend on this interface. */
typedef struct
{
    uint32_t generation, restores, adc, link;
    uint16_t ci, latency, users, blockers;
    uint8_t state, wom, connected, rail, pads, peripherals, weak, owner;
} pwrd_context_t;
enum { PWRD_CHANGE=1, PWRD_RESTORE, PWRD_WOM_WAKE, PWRD_FEATURE,
       PWRD_SHUTDOWN, PWRD_BOOT, PWRD_BUSINESS };
enum { PWRD_PWR, PWRD_ACC0, PWRD_ACC1, PWRD_WOM, PWRD_INT1,
       PWRD_FIFO, PWRD_TMST, PWRD_REG_COUNT };
enum { PWRD_ARM=1, PWRD_FEATURE_OFF, PWRD_FEATURE_ON, PWRD_WEAK_RESTORE,
       PWRD_ADC_RESTORE, PWRD_REARM };
enum { PWRD_IO_READ=1, PWRD_IO_WRITE, PWRD_IO_VERIFY, PWRD_IO_CLOCK,
       PWRD_IO_RELEASE, PWRD_IO_FLUSH_TIMEOUT, PWRD_IO_FIFO_COUNT };
#if ZY100_POWER_DIAG_ENABLE
#define pwrd_observe(...) ((void)0)
#define pwrd_end(...) ((void)0)
void pwrd_wake(void);
void pwrd_adc(void);
void pwrd_poll(void);
#define pwrd_enter(...) ((void)0)
#define pwrd_check(allowed, reason) (allowed)
uint32_t pwrd_imu_begin(uint8_t operation);
void pwrd_prepare(uint8_t origin, uint8_t rail, uint32_t generation, uint8_t flags);
void pwrd_wait(uint16_t ms);
void pwrd_cycle(uint8_t stage, uint8_t status, uint32_t off_ms);
void pwrd_cycle_elapsed(uint32_t ms);
void pwrd_origin(uint8_t origin);
#define pwrd_contract(...) ((void)0)
#define pwrd_pre(...) ((void)0)
#define pwrd_extra(...) ((void)0)
void pwrd_step(uint32_t token, uint8_t stage, uint8_t status);
void pwrd_stage(uint8_t stage);
void pwrd_verify(uint32_t token, uint8_t bank, uint8_t reg, uint8_t value,
                 uint8_t mask, uint8_t expected, uint8_t status);
void pwrd_retry(void);
void pwrd_fault(uint8_t operation, uint8_t bank, uint8_t reg, uint8_t value,
                bool read_valid, uint8_t status);
void pwrd_io_detail(uint32_t elapsed_us, uint32_t polls, uint8_t acquire_status,
                    uint8_t release_status, uint8_t last, uint8_t valid);
void pwrd_imu_finish(uint32_t token, uint8_t status);
#define pwrd_imu_value(...) ((void)0)
void pwrd_imu_end(uint8_t status);
/* One bounded task-context report; independent of successful sleep segments. */
void pwrd_report_failure(const pwrd_context_t *context, uint8_t domain,
                         uint8_t stage, uint8_t status);
#else
#define pwrd_observe(...) ((void)0)
#define pwrd_end(...) ((void)0)
#define pwrd_wake() ((void)0)
#define pwrd_adc() ((void)0)
#define pwrd_poll() ((void)0)
#define pwrd_enter() ((void)0)
#define pwrd_check(allowed, reason) (allowed)
#define pwrd_imu_begin(...) (0U)
#define pwrd_prepare(...) ((void)0)
#define pwrd_wait(...) ((void)0)
#define pwrd_cycle(...) ((void)0)
#define pwrd_cycle_elapsed(...) ((void)0)
#define pwrd_origin(...) ((void)0)
#define pwrd_contract(...) ((void)0)
#define pwrd_pre(...) ((void)0)
#define pwrd_extra(...) ((void)0)
#define pwrd_step(...) ((void)0)
#define pwrd_stage(...) ((void)0)
#define pwrd_verify(...) ((void)0)
#define pwrd_retry(...) ((void)0)
#define pwrd_fault(...) ((void)0)
#define pwrd_io_detail(...) ((void)0)
#define pwrd_imu_finish(...) ((void)0)

#define pwrd_imu_value(...) ((void)0)
#define pwrd_imu_end(...) ((void)0)
#define pwrd_report_failure(...) ((void)0)
#endif
#endif
