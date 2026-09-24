#ifndef CHARGE_VOLTAGE_SAMPLE_H
#define CHARGE_VOLTAGE_SAMPLE_H

#include <stdbool.h>
#include <stdint.h>

/* An accepted ADC observation, never a UI value or restored history. */
typedef struct
{
    uint32_t sequence;
    uint32_t sampled_ms;
    uint16_t voltage_mv;
    bool valid;
} charge_voltage_sample_t;

#endif
