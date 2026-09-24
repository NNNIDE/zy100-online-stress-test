#ifndef ZY100_OFFLINE_V2_CONFIG_H
#define ZY100_OFFLINE_V2_CONFIG_H

/* Offline V2 owns this complete profile.  Do not alias Online, Phase-A,
 * Final Edge or Legacy Offline configuration macros here. */
#define ZY100_OFFLINE_V2_IMU_SAMPLE_RATE_HZ          800U
#define ZY100_OFFLINE_V2_IMU_SAMPLE_INTERVAL_US      1250U
#define ZY100_OFFLINE_V2_IMU_FIFO_PACKET_BYTES       16U
#define ZY100_OFFLINE_V2_IMU_FIFO_WATERMARK_BYTES    512U
#define ZY100_OFFLINE_V2_IMU_FIFO_EMERGENCY_BYTES    768U
#define ZY100_OFFLINE_V2_IMU_FIFO_RESCUE_FLOOR_BYTES 256U
#define ZY100_OFFLINE_V2_IMU_NORMAL_BURST_PACKETS    4U
#define ZY100_OFFLINE_V2_IMU_EMERGENCY_BURST_PACKETS 8U
#define ZY100_OFFLINE_V2_IMU_EMERGENCY_ATTEMPTS      2U
#define ZY100_OFFLINE_V2_IMU_SERVICE_PERIOD_US       5000U
#define ZY100_OFFLINE_V2_IMU_SERVICE_TIMEOUT_MS      10U
#define ZY100_OFFLINE_V2_SPI_BUSY_RETRY_BUDGET_US    10000U
#define ZY100_OFFLINE_V2_IMU_FINAL_DRAIN_TIMEOUT_MS  1000U
#define ZY100_OFFLINE_V2_IMU_TASK_STACK_WORDS        2048U
#define ZY100_OFFLINE_V2_IMU_TASK_PRIORITY_OFFSET    6U

#define ZY100_OFFLINE_V2_MAG_ODR_HZ                  100U
#define ZY100_OFFLINE_V2_MAG_POLL_INTERVAL_MS        10U

#if ((1000000U % ZY100_OFFLINE_V2_IMU_SAMPLE_RATE_HZ) != 0U)
#error "Offline V2 IMU rate must have an integer microsecond interval"
#endif

#if (ZY100_OFFLINE_V2_IMU_SAMPLE_INTERVAL_US != \
     (1000000U / ZY100_OFFLINE_V2_IMU_SAMPLE_RATE_HZ))
#error "Offline V2 IMU interval does not match its private sample rate"
#endif

#if ((ZY100_OFFLINE_V2_IMU_FIFO_WATERMARK_BYTES % \
      ZY100_OFFLINE_V2_IMU_FIFO_PACKET_BYTES) != 0U)
#error "Offline V2 FIFO watermark must be packet aligned"
#endif

#if ((ZY100_OFFLINE_V2_IMU_FIFO_EMERGENCY_BYTES % \
      ZY100_OFFLINE_V2_IMU_FIFO_PACKET_BYTES) != 0U)
#error "Offline V2 FIFO emergency threshold must be packet aligned"
#endif

#if (ZY100_OFFLINE_V2_IMU_FIFO_RESCUE_FLOOR_BYTES >= \
     ZY100_OFFLINE_V2_IMU_FIFO_WATERMARK_BYTES)
#error "Offline V2 rescue floor must be below its watermark"
#endif

#if (ZY100_OFFLINE_V2_IMU_FIFO_WATERMARK_BYTES >= \
     ZY100_OFFLINE_V2_IMU_FIFO_EMERGENCY_BYTES)
#error "Offline V2 watermark must be below its emergency threshold"
#endif

#if (ZY100_OFFLINE_V2_SPI_BUSY_RETRY_BUDGET_US < \
     ZY100_OFFLINE_V2_IMU_SERVICE_PERIOD_US)
#error "Offline V2 SPI busy budget must allow at least one deferred service"
#endif

#endif /* ZY100_OFFLINE_V2_CONFIG_H */
