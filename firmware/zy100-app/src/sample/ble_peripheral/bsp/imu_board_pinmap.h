#ifndef IMU_BOARD_PINMAP_H
#define IMU_BOARD_PINMAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../common/imu_common.h"
#include "rtl876x.h"
#include "rtl876x_rcc.h"
#include "rtl876x_spi.h"

/*
 * Confirmed by ZY-A100-SCH.pdf (sheet 3):
 * IMU net G_SPI_CS  -> MCU 32K_XI (H_1)
 * IMU net SPI_MOSI  -> MCU P4_2
 * IMU net SPI_MISO  -> MCU P4_1
 * IMU net SPI_SCL   -> MCU P4_0
 *
 * Note:
 * MCU P4_3 is connected to FLASH_CS in this schematic, not IMU chip-select.
 */
#define IMU_SPI_CS_PIN   H_1
#define IMU_SPI_MOSI_PIN P4_2
#define IMU_SPI_MISO_PIN P4_1
#define IMU_SPI_CLK_PIN  P4_0
#define IMU_FLASH_CS_PIN P4_3

/*
 * Confirmed by ZY-A100 V1.1 schematic:
 * IMU net G_SPI_INT  -> MCU 32K_XO (H_2)
 * SENSOR-PWR         -> MCU P1_1 / SWDCLK alternate
 *
 * P0_1 is CHG-INT on V1.1 and must not be used as sensor power control.
 */
#define IMU_INT_PIN        H_2
#define IMU_POWER_CTRL_PIN P1_1

/*
 * Sensor power control polarity:
 * 1: enable -> GPIO high
 * 0: enable -> GPIO low
 */
#ifndef IMU_POWER_CTRL_ACTIVE_HIGH
#define IMU_POWER_CTRL_ACTIVE_HIGH 1U
#endif

#define IMU_SPI_PORT         SPI0
#define IMU_SPI_CLOCK_ID     APBPeriph_SPI0
#define IMU_SPI_CLOCK_MASK   APBPeriph_SPI0_CLOCK
/* IMU validation target: fixed 40MHz source with BAUDR=/2 -> 20MHz SPI SCLK. */
#define IMU_SPI_BAUD_PRESCALER SPI_BaudRatePrescaler_2

#define IMU_SPI_CPOL SPI_CPOL_Low
#define IMU_SPI_CPHA SPI_CPHA_1Edge

/*
 * IMU interrupt routing for the active board:
 * H_2 is GPIO27 on RTL8762D, so IMU INT uses GPIO27_IRQn / GPIO27_Handler.
 */
#if (IMU_INT_PIN == H_2)
#define IMU_INT_GPIO_IRQn      GPIO27_IRQn
#define IMU_INT_GPIO_ISR       GPIO27_Handler
/*
 * IMU IRQ callback notifies task via xTaskNotifyFromISR().
 * Keep IRQ priority >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (2).
 */
#define IMU_INT_NVIC_PRIORITY  3U
#else
#error "IMU_INT_PIN IRQ mapping is not defined for this board"
#endif

#if (IMU_SPI_CS_PIN == IMU_SPI_MOSI_PIN) || \
    (IMU_SPI_CS_PIN == IMU_SPI_MISO_PIN) || \
    (IMU_SPI_CS_PIN == IMU_SPI_CLK_PIN) || \
    (IMU_SPI_CS_PIN == IMU_FLASH_CS_PIN) || \
    (IMU_SPI_MOSI_PIN == IMU_SPI_MISO_PIN) || \
    (IMU_SPI_MOSI_PIN == IMU_SPI_CLK_PIN) || \
    (IMU_SPI_MOSI_PIN == IMU_FLASH_CS_PIN) || \
    (IMU_SPI_MISO_PIN == IMU_SPI_CLK_PIN) || \
    (IMU_SPI_MISO_PIN == IMU_FLASH_CS_PIN) || \
    (IMU_SPI_CLK_PIN == IMU_FLASH_CS_PIN) || \
    (IMU_INT_PIN == IMU_SPI_CS_PIN) || \
    (IMU_INT_PIN == IMU_SPI_MOSI_PIN) || \
    (IMU_INT_PIN == IMU_SPI_MISO_PIN) || \
    (IMU_INT_PIN == IMU_SPI_CLK_PIN) || \
    (IMU_INT_PIN == IMU_FLASH_CS_PIN) || \
    (IMU_POWER_CTRL_PIN == IMU_SPI_CS_PIN) || \
    (IMU_POWER_CTRL_PIN == IMU_SPI_MOSI_PIN) || \
    (IMU_POWER_CTRL_PIN == IMU_SPI_MISO_PIN) || \
    (IMU_POWER_CTRL_PIN == IMU_SPI_CLK_PIN) || \
    (IMU_POWER_CTRL_PIN == IMU_INT_PIN) || \
    (IMU_POWER_CTRL_PIN == IMU_FLASH_CS_PIN)
#error "IMU SPI pin conflict in imu_board_pinmap.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* IMU_BOARD_PINMAP_H */
