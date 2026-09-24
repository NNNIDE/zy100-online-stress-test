#ifndef ICM53611_IO_DIAG_H
#define ICM53611_IO_DIAG_H
#include <stdint.h>
typedef struct
{
    uint8_t operation; /* 0:none 1:read 2:write 3:verify 4:MCLK 5:release */
    uint8_t space;
    uint8_t reg;
    uint8_t value;
    uint8_t read_valid;
    uint8_t status;
    uint8_t acquire_status; /* 0xff: not attempted */
    uint8_t release_status; /* first failing release, otherwise last release */
} imu_spi_error_t;
#endif
