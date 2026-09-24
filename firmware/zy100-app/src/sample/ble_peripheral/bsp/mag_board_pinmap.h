#ifndef MAG_BOARD_PINMAP_H
#define MAG_BOARD_PINMAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rtl876x.h"
#include "rtl876x_i2c.h"
#include "rtl876x_rcc.h"

/*
 * Confirmed by project schematic:
 * MMC5603_I2C_SCL -> RTL8762D P2_5
 * MMC5603_I2C_SDA -> RTL8762D P2_4
 */
#define MAG_I2C_SCL_PIN P2_5
#define MAG_I2C_SDA_PIN P2_4

/* Board-level I2C configuration for MMC5603NJ. */
#define MAG_I2C_BUS_INDEX 1U
#define MAG_I2C_BUS_ID    I2C1
#define MAG_I2C_SPEED_HZ  400000U

#if (MAG_I2C_BUS_INDEX == 0U)
#define MAG_I2C_CLOCK_ID   APBPeriph_I2C0
#define MAG_I2C_CLOCK_MASK APBPeriph_I2C0_CLOCK
#define MAG_I2C_SCL_FUNC   I2C0_CLK
#define MAG_I2C_SDA_FUNC   I2C0_DAT
#elif (MAG_I2C_BUS_INDEX == 1U)
#define MAG_I2C_CLOCK_ID   APBPeriph_I2C1
#define MAG_I2C_CLOCK_MASK APBPeriph_I2C1_CLOCK
#define MAG_I2C_SCL_FUNC   I2C1_CLK
#define MAG_I2C_SDA_FUNC   I2C1_DAT
#else
#error "Unsupported MAG_I2C_BUS_INDEX"
#endif

#if (MAG_I2C_SPEED_HZ == 0U) || (MAG_I2C_SPEED_HZ > 400000U)
#error "MAG_I2C_SPEED_HZ must be in (0, 400000]"
#endif

#if (MAG_I2C_SCL_PIN == MAG_I2C_SDA_PIN)
#error "MAG I2C pin conflict: SCL and SDA are the same pin"
#endif

#ifdef __cplusplus
}
#endif

#endif /* MAG_BOARD_PINMAP_H */
