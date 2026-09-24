#ifndef MMC5603_REG_H
#define MMC5603_REG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rtl876x.h"

/* 7-bit I2C address, default factory-programmed value 0110000b. */
#ifndef MMC5603_I2C_ADDR_7BIT
#define MMC5603_I2C_ADDR_7BIT 0x30U
#endif

/* Register map. */
#define MMC5603_REG_XOUT0              0x00U
#define MMC5603_REG_XOUT1              0x01U
#define MMC5603_REG_YOUT0              0x02U
#define MMC5603_REG_YOUT1              0x03U
#define MMC5603_REG_ZOUT0              0x04U
#define MMC5603_REG_ZOUT1              0x05U
#define MMC5603_REG_XOUT2              0x06U
#define MMC5603_REG_YOUT2              0x07U
#define MMC5603_REG_ZOUT2              0x08U
#define MMC5603_REG_TOUT               0x09U
#define MMC5603_REG_STATUS1            0x18U
#define MMC5603_REG_ODR                0x1AU
#define MMC5603_REG_INTERNAL_CTRL0     0x1BU
#define MMC5603_REG_INTERNAL_CTRL1     0x1CU
#define MMC5603_REG_INTERNAL_CTRL2     0x1DU
#define MMC5603_REG_PRODUCT_ID         0x39U

#define MMC5603_PRODUCT_ID_VALUE       0x10U

/* Status1 (0x18, read-only). */
#define MMC5603_STATUS1_MEAS_M_DONE    BIT(6)
#define MMC5603_STATUS1_MEAS_T_DONE    BIT(7)
#define MMC5603_STATUS1_OTP_READ_DONE  BIT(4)

/* Internal Control 0 (0x1B, write-only). */
#define MMC5603_CTRL0_TAKE_MEAS_M      BIT(0)
#define MMC5603_CTRL0_TAKE_MEAS_T      BIT(1)
#define MMC5603_CTRL0_DO_SET           BIT(3)
#define MMC5603_CTRL0_DO_RESET         BIT(4)
#define MMC5603_CTRL0_AUTO_SR_EN       BIT(5)
#define MMC5603_CTRL0_CMM_FREQ_EN      BIT(7)

/* Internal Control 1 (0x1C, write-only). */
#define MMC5603_CTRL1_BW0              BIT(0)
#define MMC5603_CTRL1_BW1              BIT(1)
#define MMC5603_CTRL1_X_INHIBIT        BIT(2)
#define MMC5603_CTRL1_Y_INHIBIT        BIT(3)
#define MMC5603_CTRL1_Z_INHIBIT        BIT(4)
#define MMC5603_CTRL1_SW_RESET         BIT(7)

/* Internal Control 2 (0x1D, write-only). */
#define MMC5603_CTRL2_PRD_SET0         BIT(0)
#define MMC5603_CTRL2_PRD_SET1         BIT(1)
#define MMC5603_CTRL2_PRD_SET2         BIT(2)
#define MMC5603_CTRL2_EN_PRD_SET       BIT(3)
#define MMC5603_CTRL2_CMM_EN           BIT(4)
#define MMC5603_CTRL2_HPOWER           BIT(7)

#define MMC5603_CTRL1_BW_MASK          (MMC5603_CTRL1_BW1 | MMC5603_CTRL1_BW0)

#define MMC5603_BW_00                  0x00U /* tTM=6.6ms, up to 75Hz */
#define MMC5603_BW_01                  0x01U /* tTM=3.5ms, up to 150Hz */
#define MMC5603_BW_10                  0x02U /* tTM=2.0ms, up to 255Hz */
#define MMC5603_BW_11                  0x03U /* tTM=1.2ms, up to 255Hz (1000Hz with hpower=1 and ODR=255) */

#define MMC5603_MEAS_RAW_LEN           9U

#ifdef __cplusplus
}
#endif

#endif /* MMC5603_REG_H */
