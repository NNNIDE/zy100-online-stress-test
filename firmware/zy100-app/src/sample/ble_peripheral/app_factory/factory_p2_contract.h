#ifndef FACTORY_P2_CONTRACT_H
#define FACTORY_P2_CONTRACT_H

#include <stdint.h>

/* Factory-only extensions. Keep the shared Production MFG mask unchanged. */
#define FACTORY_P2_TEST_CHARGING_CURRENT       0x00400000UL
#define FACTORY_P2_TEST_UNCONNECTED_CURRENT    0x00800000UL
#define FACTORY_P2_TEST_RAM                    0x01000000UL
#define FACTORY_P2_TEST_MAG_RAW                0x02000000UL
#define FACTORY_P2_TEST_IMU_ACCEL_RAW          0x04000000UL
#define FACTORY_P2_TEST_IMU_GYRO_RAW           0x08000000UL

#define FACTORY_P2_FUNCTIONAL_TEST_MASK        0x0F000000UL
#define FACTORY_P2_REQUIRED_TEST_MASK          0x0FB7FFF6UL

#define FACTORY_P2_ITEM_CHARGING_CURRENT       2U
#define FACTORY_P2_ITEM_UNCONNECTED_CURRENT    3U
#define FACTORY_P2_ITEM_RSSI                   4U
#define FACTORY_P2_ITEM_CONNECTED_CURRENT      5U
#define FACTORY_P2_ITEM_SHIPPING_CURRENT       6U
#define FACTORY_P2_ITEM_BATTERY_ADC            7U
#define FACTORY_P2_ITEM_BATTERY_CAL            8U
#define FACTORY_P2_ITEM_BATTERY_VERIFY         9U
#define FACTORY_P2_ITEM_RAM                    19U
#define FACTORY_P2_ITEM_KEY                    21U
#define FACTORY_P2_ITEM_MAG_RAW                22U
#define FACTORY_P2_ITEM_IMU_ACCEL_RAW          25U
#define FACTORY_P2_ITEM_IMU_GYRO_RAW           28U
#define FACTORY_P2_ITEM_LED_RED                54U
#define FACTORY_P2_ITEM_LED_GREEN              55U
#define FACTORY_P2_ITEM_LED_BLUE               56U
#define FACTORY_P2_ITEM_LED_RGB_SEQUENCE       57U

#endif /* FACTORY_P2_CONTRACT_H */
