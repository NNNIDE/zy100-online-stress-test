#ifndef LED_BOARD_PINMAP_H
#define LED_BOARD_PINMAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rtl876x.h"

/*
 * Common LED electrical interface:
 * PWM_RGB-LED -> MCU P2_6, single-wire addressable RGB LED data.
 * LED_PWR -> MCU MICBIAS/P5_0, active-high unless board testing proves otherwise.
 * Current LED_POWER hardware adds an NPN stage on PWM_RGB-LED, so the MCU
 * drive level is inverted from the LED DIN level.
 *
 * The RTL8762D SDK names MICBIAS/P5_0 as H_0 (GPIO25).
 *
 * LED board profile marks the logical bead mapping. Current firmware defaults
 * to the ZY100 bead map. Define ZY100_LED_BOARD_PROFILE as
 * ZY100_LED_BOARD_PROFILE_ZY200 for 13-LED ZY200 builds.
 */
#define ZY100_LED_BOARD_PROFILE_ZY100      100U
#define ZY100_LED_BOARD_PROFILE_ZY200      200U

#ifndef ZY100_LED_BOARD_PROFILE
#define ZY100_LED_BOARD_PROFILE            ZY100_LED_BOARD_PROFILE_ZY100
#endif

#ifndef ZY100_RGB_LED_DATA_PIN
#define ZY100_RGB_LED_DATA_PIN             P2_6
#endif

#ifndef ZY100_RGB_LED_DATA_ACTIVE_LOW
#define ZY100_RGB_LED_DATA_ACTIVE_LOW      1U
#endif

#if (ZY100_LED_BOARD_PROFILE == ZY100_LED_BOARD_PROFILE_ZY100)

/* ZY100: D1-D6 are LOGO, D7-D8 are Notify. */
#ifndef ZY100_RGB_LED_COUNT
#define ZY100_RGB_LED_COUNT                8U
#endif

#ifndef ZY100_LED_LOGO_COUNT
#define ZY100_LED_LOGO_COUNT               6U
#endif

#ifndef ZY100_LED_NOTIFY_COUNT
#define ZY100_LED_NOTIFY_COUNT             2U
#endif

#define ZY100_LED_BOARD_PROFILE_NAME       "ZY100"
#define ZY100_LED_LOGO_ORDER_INIT          {2U, 1U, 0U, 5U, 4U, 3U}

#if ((ZY100_RGB_LED_COUNT != 8U) || \
     (ZY100_LED_LOGO_COUNT != 6U) || \
     (ZY100_LED_NOTIFY_COUNT != 2U))
#error "ZY100 LED board profile requires 8 total LEDs: 6 LOGO + 2 Notify"
#endif

#elif (ZY100_LED_BOARD_PROFILE == ZY100_LED_BOARD_PROFILE_ZY200)

/* ZY200: LED1-LED12 are LOGO, LED13 is the single Notify LED. */
#ifndef ZY100_RGB_LED_COUNT
#define ZY100_RGB_LED_COUNT                13U
#endif

#ifndef ZY100_LED_LOGO_COUNT
#define ZY100_LED_LOGO_COUNT               12U
#endif

#ifndef ZY100_LED_NOTIFY_COUNT
#define ZY100_LED_NOTIFY_COUNT             1U
#endif

#define ZY100_LED_BOARD_PROFILE_NAME       "ZY200"
#define ZY100_LED_LOGO_ORDER_INIT          {0U, 1U, 2U, 3U, 4U, 5U, \
                                            6U, 7U, 8U, 9U, 10U, 11U}

#if ((ZY100_RGB_LED_COUNT != 13U) || \
     (ZY100_LED_LOGO_COUNT != 12U) || \
     (ZY100_LED_NOTIFY_COUNT != 1U))
#error "ZY200 LED board profile requires 13 total LEDs: 12 LOGO + 1 Notify"
#endif

#else
#error "Unsupported ZY100_LED_BOARD_PROFILE"
#endif

#if ((ZY100_LED_LOGO_COUNT + ZY100_LED_NOTIFY_COUNT) != ZY100_RGB_LED_COUNT)
#error "LED LOGO and Notify counts must add up to the RGB LED chain count"
#endif

#ifndef ZY100_LED_POWER_CTRL_PIN
#define ZY100_LED_POWER_CTRL_PIN           H_0
#endif

#ifndef ZY100_LED_POWER_CTRL_PIN_AVAILABLE
#define ZY100_LED_POWER_CTRL_PIN_AVAILABLE 1U
#endif

#ifndef ZY100_LED_POWER_CTRL_ACTIVE_HIGH
#define ZY100_LED_POWER_CTRL_ACTIVE_HIGH   1U
#endif

#ifdef __cplusplus
}
#endif

#endif /* LED_BOARD_PINMAP_H */
