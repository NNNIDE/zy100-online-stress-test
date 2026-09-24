#ifndef ZY100_APP_BUILD_CONFIG_H
#define ZY100_APP_BUILD_CONFIG_H

#ifndef ZY100_BUILD_FACTORY
#define ZY100_BUILD_FACTORY 0
#endif

#ifndef ZY100_BUILD_PRODUCTION
#define ZY100_BUILD_PRODUCTION 0
#endif

#ifndef ZY100_BUILD_SHIPPING_TEST
#define ZY100_BUILD_SHIPPING_TEST 0
#endif
#if (ZY100_BUILD_SHIPPING_TEST != 0) && (ZY100_BUILD_SHIPPING_TEST != 1)
#error "ZY100_BUILD_SHIPPING_TEST must be 0 or 1"
#endif

#if ((ZY100_BUILD_FACTORY + ZY100_BUILD_PRODUCTION + ZY100_BUILD_SHIPPING_TEST) != 1)
#error "Select exactly one ZY100 build flavor"
#endif

#if ZY100_BUILD_FACTORY
#define ZY100_BUILD_FLAVOR_NAME "factory"
#define ZY100_BUILD_FLAVOR_TAG  "ZY100_FLAVOR_FACTORY"
#elif defined(ZY100_ONLINE_STRESS_TEST_ENABLE) && ZY100_ONLINE_STRESS_TEST_ENABLE
#define ZY100_BUILD_FLAVOR_NAME "online_stress_test"
#define ZY100_BUILD_FLAVOR_TAG "ZY100_FLAVOR_ONLINE_STRESS_TEST"
#elif ZY100_BUILD_PRODUCTION
#define ZY100_BUILD_FLAVOR_NAME "production"
#define ZY100_BUILD_FLAVOR_TAG  "ZY100_FLAVOR_PRODUCTION"
#else
#define ZY100_BUILD_FLAVOR_NAME "shipping_test"
#define ZY100_BUILD_FLAVOR_TAG  "ZY100_FLAVOR_SHIPPING_TEST"
#endif

/* Development Production only: persist compatible locked test masks at boot.
 * Disable for a strict release; Factory never enables the repair path. */
#ifndef ZY100_PRODUCTION_MFG_TEST_COMPAT_ENABLE
#define ZY100_PRODUCTION_MFG_TEST_COMPAT_ENABLE ZY100_BUILD_PRODUCTION
#endif
#if (ZY100_PRODUCTION_MFG_TEST_COMPAT_ENABLE != 0) && \
    (ZY100_PRODUCTION_MFG_TEST_COMPAT_ENABLE != 1)
#error "ZY100_PRODUCTION_MFG_TEST_COMPAT_ENABLE must be 0 or 1"
#endif
#if ZY100_PRODUCTION_MFG_TEST_COMPAT_ENABLE && !ZY100_BUILD_PRODUCTION
#error "MFG test compatibility is Production-only"
#endif

/* Test-candidate only: persist ST -> CG once at Production boot.
 * Normal targets default off; enable explicitly for the requested test BIN. */
#ifndef ZY100_PRODUCTION_TEST_ST_TO_CG_ENABLE
#define ZY100_PRODUCTION_TEST_ST_TO_CG_ENABLE 0
#endif
#if (ZY100_PRODUCTION_TEST_ST_TO_CG_ENABLE != 0) && \
    (ZY100_PRODUCTION_TEST_ST_TO_CG_ENABLE != 1)
#error "ZY100_PRODUCTION_TEST_ST_TO_CG_ENABLE must be 0 or 1"
#endif
#if ZY100_PRODUCTION_TEST_ST_TO_CG_ENABLE && !ZY100_BUILD_PRODUCTION
#error "ST to CG test conversion is Production-only"
#endif

#endif /* ZY100_APP_BUILD_CONFIG_H */
