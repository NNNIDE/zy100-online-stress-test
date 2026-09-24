#ifndef ZY100_ONLINE_IMU_CONFIG_H
#define ZY100_ONLINE_IMU_CONFIG_H

/* Online-only capture contract copied from the board-verified 836e3a43
 * implementation. Do not alias these values to Offline or Final Edge. */
#define ZY100_ONLINE_IMU_SAMPLE_RATE_HZ              800U
#define ZY100_ONLINE_IMU_GYRO_RANGE_DPS              2000U
#define ZY100_ONLINE_IMU_ACCEL_RANGE_G               16U
#define ZY100_ONLINE_MAG_SAMPLE_RATE_HZ              100U
#define ZY100_ONLINE_IMU_FIFO_PACKET_BYTES           16U
#define ZY100_ONLINE_IMU_FIFO_WATERMARK_BYTES        512U
#define ZY100_ONLINE_IMU_FIFO_EMERGENCY_BYTES        768U
#define ZY100_ONLINE_IMU_TIMER_PERIOD_US             1250U
#define ZY100_ONLINE_IMU_TIMER_MAX_DUE_BURST         4U

#endif /* ZY100_ONLINE_IMU_CONFIG_H */
