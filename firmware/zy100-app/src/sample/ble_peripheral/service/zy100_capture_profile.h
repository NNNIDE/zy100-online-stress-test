#ifndef ZY100_CAPTURE_PROFILE_H
#define ZY100_CAPTURE_PROFILE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ZY100_CAPTURE_PROFILE_NONE = 0U,
    ZY100_CAPTURE_PROFILE_ONLINE,
    ZY100_CAPTURE_PROFILE_OFFLINE_V2,
    ZY100_CAPTURE_PROFILE_LEGACY_OFFLINE,
} zy100_capture_profile_t;

bool zy100_capture_profile_claim(zy100_capture_profile_t profile);
bool zy100_capture_profile_release(zy100_capture_profile_t profile);
zy100_capture_profile_t zy100_capture_profile_current(void);

#ifdef __cplusplus
}
#endif

#endif /* ZY100_CAPTURE_PROFILE_H */
