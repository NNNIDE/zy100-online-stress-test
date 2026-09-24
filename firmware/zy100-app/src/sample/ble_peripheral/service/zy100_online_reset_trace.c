#include "zy100_online_reset_trace.h"

#include "os_sync.h"
#include "rtl876x_rtc.h"
#include "trace.h"

#define ZY100_ONLINE_TRACE_MAGIC              0xAU
#define ZY100_POWER_TRACE_MAGIC               0xBU
#define ZY100_ONLINE_TRACE_VERSION            0x1U
#define ZY100_OFFLINE_TRACE_VERSION           0x2U
#define ZY100_ONLINE_TRACE_MAGIC_SHIFT        28U
#define ZY100_ONLINE_TRACE_VERSION_SHIFT      26U
#define ZY100_ONLINE_TRACE_ARMED_BIT          (1UL << 25)
#define ZY100_ONLINE_TRACE_APP_SHIFT          21U
#define ZY100_ONLINE_TRACE_IMU_SHIFT          16U
#define ZY100_ONLINE_TRACE_FEED_SHIFT         12U
#define ZY100_ONLINE_TRACE_SPI_SHIFT          10U
#define ZY100_ONLINE_TRACE_TIMER_SHIFT        8U
#define ZY100_ONLINE_TRACE_TIMER_RUNNING_BIT  (1UL << 7)
#define ZY100_ONLINE_TRACE_B_CRITICAL_BIT     (1UL << 6)
#define ZY100_ONLINE_TRACE_RAW_PENDING_BIT    (1UL << 5)
#define ZY100_ONLINE_TRACE_REC_PENDING_BIT    (1UL << 4)
#define ZY100_ONLINE_TRACE_CHECKSUM_MASK      0xFUL
#define ZY100_OFFLINE_TRACE_APP_SHIFT         22U
#define ZY100_OFFLINE_TRACE_IMU_SHIFT         19U
#define ZY100_OFFLINE_TRACE_FEED_SHIFT        15U
#define ZY100_OFFLINE_TRACE_URGENT_BIT        (1UL << 14)
#define ZY100_OFFLINE_TRACE_ALARM_BIT         (1UL << 13)
#define ZY100_OFFLINE_TRACE_RESCUE_BIT        (1UL << 12)
#define ZY100_OFFLINE_TRACE_SERVICE_SHIFT     10U
#define ZY100_OFFLINE_TRACE_AGE_SHIFT         4U
#define ZY100_OFFLINE_TRACE_AGE_MAX           0x3FU
#define ZY100_POWER_TRACE_LED_STAGE_SHIFT     ZY100_ONLINE_TRACE_FEED_SHIFT
#define ZY100_POWER_TRACE_YHM_SHIFT           ZY100_ONLINE_TRACE_TIMER_SHIFT

static uint32_t s_online_trace_word = 0U;
static bool offline_trace_valid(uint32_t word);

static uint8_t online_trace_checksum(uint32_t word)
{
    uint8_t checksum = 0x5U;
    uint32_t shift;

    word &= ~ZY100_ONLINE_TRACE_CHECKSUM_MASK;
    for (shift = 4U; shift < 32U; shift += 4U)
    {
        checksum ^= (uint8_t)((word >> shift) & 0xFU);
    }
    return (uint8_t)(checksum & 0xFU);
}

static bool online_trace_valid(uint32_t word)
{
    return (((word >> ZY100_ONLINE_TRACE_MAGIC_SHIFT) & 0xFU) ==
            ZY100_ONLINE_TRACE_MAGIC) &&
           (((word >> ZY100_ONLINE_TRACE_VERSION_SHIFT) & 0x3U) ==
            ZY100_ONLINE_TRACE_VERSION) &&
           ((word & ZY100_ONLINE_TRACE_CHECKSUM_MASK) ==
            online_trace_checksum(word));
}

static bool power_trace_valid(uint32_t word)
{
    return (((word >> ZY100_ONLINE_TRACE_MAGIC_SHIFT) & 0xFU) ==
            ZY100_POWER_TRACE_MAGIC) &&
           (((word >> ZY100_ONLINE_TRACE_VERSION_SHIFT) & 0x3U) ==
            ZY100_ONLINE_TRACE_VERSION) &&
           ((word & ZY100_ONLINE_TRACE_CHECKSUM_MASK) ==
            online_trace_checksum(word));
}

static void online_trace_write_locked(uint32_t word)
{
    word &= ~ZY100_ONLINE_TRACE_CHECKSUM_MASK;
    word |= online_trace_checksum(word);
    s_online_trace_word = word;
    RTC_WriteBackupReg(word);
}

static void online_trace_update_field(uint32_t mask,
                                      uint32_t shift,
                                      uint32_t value)
{
    uint32_t lock_state = os_lock();
    uint32_t word = s_online_trace_word;

    if (!online_trace_valid(word) ||
        ((word & ZY100_ONLINE_TRACE_ARMED_BIT) == 0U))
    {
        os_unlock(lock_state);
        return;
    }
    word = (word & ~mask) | ((value << shift) & mask);
    online_trace_write_locked(word);
    os_unlock(lock_state);
}

void zy100_online_reset_trace_boot_report(void)
{
    uint32_t word = RTC_ReadBackupReg();

    s_online_trace_word = word;
    if (power_trace_valid(word) &&
        ((word & ZY100_ONLINE_TRACE_ARMED_BIT) != 0U))
    {
        DBG_DIRECT("[POWER_RESET_CTX] phase=%u state=%u led_stage=%u yhm=%u",
                   (uint32_t)((word >> ZY100_ONLINE_TRACE_APP_SHIFT) & 0xFU),
                   (uint32_t)((word >> ZY100_ONLINE_TRACE_IMU_SHIFT) & 0x1FU),
                   (uint32_t)((word >> ZY100_POWER_TRACE_LED_STAGE_SHIFT) & 0xFU),
                   (uint32_t)((word >> ZY100_POWER_TRACE_YHM_SHIFT) & 0xFU));
    }
    else if (offline_trace_valid(word) &&
             ((word & ZY100_ONLINE_TRACE_ARMED_BIT) != 0U))
    {
        DBG_DIRECT("[CAP_RST_A] mode=offline app=%u imu=%u feed=%u",
                   (uint32_t)((word >> ZY100_OFFLINE_TRACE_APP_SHIFT) & 0x7U),
                   (uint32_t)((word >> ZY100_OFFLINE_TRACE_IMU_SHIFT) & 0x7U),
                   (uint32_t)((word >> ZY100_OFFLINE_TRACE_FEED_SHIFT) & 0xFU));
        DBG_DIRECT("[CAP_RST_B] urgent=%u alarm=%u rescue=%u svc=%u age5=%u",
                   (word & ZY100_OFFLINE_TRACE_URGENT_BIT) ? 1U : 0U,
                   (word & ZY100_OFFLINE_TRACE_ALARM_BIT) ? 1U : 0U,
                   (word & ZY100_OFFLINE_TRACE_RESCUE_BIT) ? 1U : 0U,
                   (uint32_t)((word >> ZY100_OFFLINE_TRACE_SERVICE_SHIFT) & 0x3U),
                   (uint32_t)((word >> ZY100_OFFLINE_TRACE_AGE_SHIFT) & 0x3FU));
    }
    else if (online_trace_valid(word) &&
        ((word & ZY100_ONLINE_TRACE_ARMED_BIT) != 0U))
    {
        DBG_DIRECT("[CAP_WDG_CTX] app=%u imu=%u feed=%u spi=%u tim=%u run=%u b=%u raw=%u rec=%u",
                   (uint32_t)((word >> ZY100_ONLINE_TRACE_APP_SHIFT) & 0xFU),
                   (uint32_t)((word >> ZY100_ONLINE_TRACE_IMU_SHIFT) & 0x1FU),
                   (uint32_t)((word >> ZY100_ONLINE_TRACE_FEED_SHIFT) & 0xFU),
                   (uint32_t)((word >> ZY100_ONLINE_TRACE_SPI_SHIFT) & 0x3U),
                   (uint32_t)((word >> ZY100_ONLINE_TRACE_TIMER_SHIFT) & 0x3U),
                   (word & ZY100_ONLINE_TRACE_TIMER_RUNNING_BIT) ? 1U : 0U,
                   (word & ZY100_ONLINE_TRACE_B_CRITICAL_BIT) ? 1U : 0U,
                   (word & ZY100_ONLINE_TRACE_RAW_PENDING_BIT) ? 1U : 0U,
                   (word & ZY100_ONLINE_TRACE_REC_PENDING_BIT) ? 1U : 0U);
    }
    zy100_online_reset_trace_clear();
}

void zy100_online_reset_trace_arm(void)
{
    uint32_t lock_state = os_lock();
    uint32_t word = (ZY100_ONLINE_TRACE_MAGIC <<
                     ZY100_ONLINE_TRACE_MAGIC_SHIFT) |
                    (ZY100_ONLINE_TRACE_VERSION <<
                     ZY100_ONLINE_TRACE_VERSION_SHIFT) |
                    ZY100_ONLINE_TRACE_ARMED_BIT |
                    ((uint32_t)ZY100_ONLINE_TRACE_APP_START <<
                     ZY100_ONLINE_TRACE_APP_SHIFT);

    online_trace_write_locked(word);
    os_unlock(lock_state);
}

void zy100_online_reset_trace_clear(void)
{
    uint32_t lock_state = os_lock();

    s_online_trace_word = 0U;
    RTC_WriteBackupReg(0U);
    os_unlock(lock_state);
}

void zy100_online_reset_trace_set_app_phase(
    zy100_online_trace_app_phase_t phase)
{
    online_trace_update_field(0xFUL << ZY100_ONLINE_TRACE_APP_SHIFT,
                              ZY100_ONLINE_TRACE_APP_SHIFT,
                              (uint32_t)phase);
}

void zy100_online_reset_trace_set_imu_phase(
    zy100_online_trace_imu_phase_t phase)
{
    online_trace_update_field(0x1FUL << ZY100_ONLINE_TRACE_IMU_SHIFT,
                              ZY100_ONLINE_TRACE_IMU_SHIFT,
                              (uint32_t)phase);
}

void zy100_online_reset_trace_set_runtime(uint8_t spi_owner,
                                          uint8_t timer_owner,
                                          bool timer_running,
                                          bool b_critical,
                                          bool raw_pending,
                                          bool record_pending)
{
    uint32_t lock_state = os_lock();
    uint32_t word = s_online_trace_word;

    if (!online_trace_valid(word) ||
        ((word & ZY100_ONLINE_TRACE_ARMED_BIT) == 0U))
    {
        os_unlock(lock_state);
        return;
    }
    word &= ~((0x3UL << ZY100_ONLINE_TRACE_SPI_SHIFT) |
              (0x3UL << ZY100_ONLINE_TRACE_TIMER_SHIFT) |
              ZY100_ONLINE_TRACE_TIMER_RUNNING_BIT |
              ZY100_ONLINE_TRACE_B_CRITICAL_BIT |
              ZY100_ONLINE_TRACE_RAW_PENDING_BIT |
              ZY100_ONLINE_TRACE_REC_PENDING_BIT);
    word |= (((uint32_t)spi_owner & 0x3U) << ZY100_ONLINE_TRACE_SPI_SHIFT) |
            (((uint32_t)timer_owner & 0x3U) << ZY100_ONLINE_TRACE_TIMER_SHIFT);
    if (timer_running)
    {
        word |= ZY100_ONLINE_TRACE_TIMER_RUNNING_BIT;
    }
    if (b_critical)
    {
        word |= ZY100_ONLINE_TRACE_B_CRITICAL_BIT;
    }
    if (raw_pending)
    {
        word |= ZY100_ONLINE_TRACE_RAW_PENDING_BIT;
    }
    if (record_pending)
    {
        word |= ZY100_ONLINE_TRACE_REC_PENDING_BIT;
    }
    online_trace_write_locked(word);
    os_unlock(lock_state);
}

void zy100_online_reset_trace_note_feed(void)
{
    uint32_t lock_state = os_lock();
    uint32_t word = s_online_trace_word;
    uint32_t generation;

    if ((!online_trace_valid(word) && !offline_trace_valid(word)) ||
        ((word & ZY100_ONLINE_TRACE_ARMED_BIT) == 0U))
    {
        os_unlock(lock_state);
        return;
    }
    if (offline_trace_valid(word))
    {
        generation = ((word >> ZY100_OFFLINE_TRACE_FEED_SHIFT) + 1U) & 0xFU;
        word &= ~(0xFUL << ZY100_OFFLINE_TRACE_FEED_SHIFT);
        word |= generation << ZY100_OFFLINE_TRACE_FEED_SHIFT;
    }
    else
    {
        generation = ((word >> ZY100_ONLINE_TRACE_FEED_SHIFT) + 1U) & 0xFU;
        word &= ~(0xFUL << ZY100_ONLINE_TRACE_FEED_SHIFT);
        word |= generation << ZY100_ONLINE_TRACE_FEED_SHIFT;
    }
    online_trace_write_locked(word);
    os_unlock(lock_state);
}


void zy100_power_reset_trace_arm(zy100_power_trace_phase_t phase,
                                 uint8_t power_state)
{
    uint32_t lock_state = os_lock();
    uint32_t word = (ZY100_POWER_TRACE_MAGIC <<
                     ZY100_ONLINE_TRACE_MAGIC_SHIFT) |
                    (ZY100_ONLINE_TRACE_VERSION <<
                     ZY100_ONLINE_TRACE_VERSION_SHIFT) |
                    ZY100_ONLINE_TRACE_ARMED_BIT |
                    (((uint32_t)phase & 0xFU) <<
                     ZY100_ONLINE_TRACE_APP_SHIFT) |
                    (((uint32_t)power_state & 0x1FU) <<
                     ZY100_ONLINE_TRACE_IMU_SHIFT);

    online_trace_write_locked(word);
    os_unlock(lock_state);
}

void zy100_power_reset_trace_set_phase(zy100_power_trace_phase_t phase,
                                       uint8_t power_state)
{
    uint32_t lock_state = os_lock();
    uint32_t word = s_online_trace_word;

    if (!power_trace_valid(word) ||
        ((word & ZY100_ONLINE_TRACE_ARMED_BIT) == 0U))
    {
        os_unlock(lock_state);
        return;
    }
    word &= ~((0xFUL << ZY100_ONLINE_TRACE_APP_SHIFT) |
              (0x1FUL << ZY100_ONLINE_TRACE_IMU_SHIFT));
    word |= (((uint32_t)phase & 0xFU) << ZY100_ONLINE_TRACE_APP_SHIFT) |
            (((uint32_t)power_state & 0x1FU) << ZY100_ONLINE_TRACE_IMU_SHIFT);
    online_trace_write_locked(word);
    os_unlock(lock_state);
}

void zy100_power_reset_trace_set_stage(zy100_power_trace_phase_t phase)
{
    uint32_t lock_state = os_lock();
    uint32_t word = s_online_trace_word;

    if (!power_trace_valid(word) ||
        ((word & ZY100_ONLINE_TRACE_ARMED_BIT) == 0U))
    {
        os_unlock(lock_state);
        return;
    }
    word &= ~(0xFUL << ZY100_ONLINE_TRACE_APP_SHIFT);
    word |= ((uint32_t)phase & 0xFU) << ZY100_ONLINE_TRACE_APP_SHIFT;
    online_trace_write_locked(word);
    os_unlock(lock_state);
}

void zy100_offline_reset_trace_arm(void)
{
    uint32_t lock_state = os_lock();
    uint32_t word = (ZY100_ONLINE_TRACE_MAGIC <<
                     ZY100_ONLINE_TRACE_MAGIC_SHIFT) |
                    (ZY100_OFFLINE_TRACE_VERSION <<
                     ZY100_ONLINE_TRACE_VERSION_SHIFT) |
                    ZY100_ONLINE_TRACE_ARMED_BIT |
                    ((uint32_t)ZY100_OFFLINE_TRACE_APP_START <<
                     ZY100_OFFLINE_TRACE_APP_SHIFT) |
                    ((uint32_t)ZY100_OFFLINE_TRACE_IMU_START <<
                     ZY100_OFFLINE_TRACE_IMU_SHIFT);

    online_trace_write_locked(word);
    os_unlock(lock_state);
}

void zy100_offline_reset_trace_set_app_phase(
    zy100_offline_trace_app_phase_t phase)
{
    uint32_t lock_state = os_lock();
    uint32_t word = s_online_trace_word;

    if (offline_trace_valid(word) &&
        ((word & ZY100_ONLINE_TRACE_ARMED_BIT) != 0U))
    {
        word &= ~(0x7UL << ZY100_OFFLINE_TRACE_APP_SHIFT);
        word |= ((uint32_t)phase & 0x7U) << ZY100_OFFLINE_TRACE_APP_SHIFT;
        online_trace_write_locked(word);
    }
    os_unlock(lock_state);
}

void zy100_offline_reset_trace_set_imu_state(
    zy100_offline_trace_imu_phase_t phase,
    bool urgent,
    bool alarm,
    bool rescue,
    zy100_offline_trace_service_t service,
    uint32_t fifo_age_ms)
{
    uint32_t lock_state = os_lock();
    uint32_t word = s_online_trace_word;
    uint32_t age_bucket = fifo_age_ms / 5U;

    if (age_bucket > ZY100_OFFLINE_TRACE_AGE_MAX)
    {
        age_bucket = ZY100_OFFLINE_TRACE_AGE_MAX;
    }
    if (offline_trace_valid(word) &&
        ((word & ZY100_ONLINE_TRACE_ARMED_BIT) != 0U))
    {
        word &= ~((0x7UL << ZY100_OFFLINE_TRACE_IMU_SHIFT) |
                  ZY100_OFFLINE_TRACE_URGENT_BIT |
                  ZY100_OFFLINE_TRACE_ALARM_BIT |
                  ZY100_OFFLINE_TRACE_RESCUE_BIT |
                  (0x3UL << ZY100_OFFLINE_TRACE_SERVICE_SHIFT) |
                  (0x3FUL << ZY100_OFFLINE_TRACE_AGE_SHIFT));
        word |= ((uint32_t)phase & 0x7U) << ZY100_OFFLINE_TRACE_IMU_SHIFT;
        word |= ((uint32_t)service & 0x3U) <<
                ZY100_OFFLINE_TRACE_SERVICE_SHIFT;
        word |= age_bucket << ZY100_OFFLINE_TRACE_AGE_SHIFT;
        if (urgent)
        {
            word |= ZY100_OFFLINE_TRACE_URGENT_BIT;
        }
        if (alarm)
        {
            word |= ZY100_OFFLINE_TRACE_ALARM_BIT;
        }
        if (rescue)
        {
            word |= ZY100_OFFLINE_TRACE_RESCUE_BIT;
        }
        online_trace_write_locked(word);
    }
    os_unlock(lock_state);
}

static bool offline_trace_valid(uint32_t word)
{
    return (((word >> ZY100_ONLINE_TRACE_MAGIC_SHIFT) & 0xFU) ==
            ZY100_ONLINE_TRACE_MAGIC) &&
           (((word >> ZY100_ONLINE_TRACE_VERSION_SHIFT) & 0x3U) ==
            ZY100_OFFLINE_TRACE_VERSION) &&
           ((word & ZY100_ONLINE_TRACE_CHECKSUM_MASK) ==
            online_trace_checksum(word));
}

void zy100_power_reset_trace_set_led_stage(zy100_power_led_stage_t stage)
{
    uint32_t lock_state = os_lock();
    uint32_t word = s_online_trace_word;

    if (!power_trace_valid(word) ||
        ((word & ZY100_ONLINE_TRACE_ARMED_BIT) == 0U))
    {
        os_unlock(lock_state);
        return;
    }
    word &= ~(0xFUL << ZY100_POWER_TRACE_LED_STAGE_SHIFT);
    word |= ((uint32_t)stage & 0xFU) << ZY100_POWER_TRACE_LED_STAGE_SHIFT;
    online_trace_write_locked(word);
    os_unlock(lock_state);
}

void zy100_power_reset_trace_set_yhm_convergence(
    zy100_power_yhm_convergence_t convergence)
{
    uint32_t lock_state = os_lock();
    uint32_t word = s_online_trace_word;

    if (!power_trace_valid(word) ||
        ((word & ZY100_ONLINE_TRACE_ARMED_BIT) == 0U))
    {
        os_unlock(lock_state);
        return;
    }
    word &= ~(0xFUL << ZY100_POWER_TRACE_YHM_SHIFT);
    word |= ((uint32_t)convergence & 0xFU) << ZY100_POWER_TRACE_YHM_SHIFT;
    online_trace_write_locked(word);
    os_unlock(lock_state);
}

void zy100_power_reset_trace_clear(void)
{
    uint32_t lock_state = os_lock();

    if (power_trace_valid(s_online_trace_word))
    {
        s_online_trace_word = 0U;
        RTC_WriteBackupReg(0U);
    }
    os_unlock(lock_state);
}
