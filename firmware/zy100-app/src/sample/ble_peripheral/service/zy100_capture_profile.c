#include "zy100_capture_profile.h"

#include "os_sync.h"

static zy100_capture_profile_t s_capture_profile =
    ZY100_CAPTURE_PROFILE_NONE;

bool zy100_capture_profile_claim(zy100_capture_profile_t profile)
{
    uint32_t lock_state;

    if (profile == ZY100_CAPTURE_PROFILE_NONE)
    {
        return false;
    }
    lock_state = os_lock();
    if (s_capture_profile != ZY100_CAPTURE_PROFILE_NONE)
    {
        os_unlock(lock_state);
        return false;
    }
    s_capture_profile = profile;
    os_unlock(lock_state);
    return true;
}

bool zy100_capture_profile_release(zy100_capture_profile_t profile)
{
    uint32_t lock_state;

    if (profile == ZY100_CAPTURE_PROFILE_NONE)
    {
        return false;
    }
    lock_state = os_lock();
    if (s_capture_profile != profile)
    {
        os_unlock(lock_state);
        return false;
    }
    s_capture_profile = ZY100_CAPTURE_PROFILE_NONE;
    os_unlock(lock_state);
    return true;
}

zy100_capture_profile_t zy100_capture_profile_current(void)
{
    zy100_capture_profile_t profile;
    uint32_t lock_state = os_lock();

    profile = s_capture_profile;
    os_unlock(lock_state);
    return profile;
}
