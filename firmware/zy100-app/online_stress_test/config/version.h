#ifndef ZY100_ONLINE_STRESS_TEST_VERSION_H
#define ZY100_ONLINE_STRESS_TEST_VERSION_H

#define VERSION_CODE             20007U
#define VERSION_MAJOR            1
#define VERSION_MINOR            0
#define VERSION_REVISION         0
#define VERSION_BUILDNUM         2
#define VERSION_GCID             0xa602a027
#define VERSION_GCID2            0xd612d9cb
#define CUSTOMER_NAME            sdk
#define CN_1                     's'
#define CN_2                     'd'
#define CN_3                     'k'
#define CN_4                     '#'
#define CN_5                     '#'
#define CN_6                     '#'
#define CN_7                     '#'
#define CN_8                     '#'
#define BUILDING_TIME            "ZY100 Online Stress Test 1.0.0.2"
#define NAME2STR(a)              #a
#define CUSTOMER_NAME_S          #NAME2STR(CUSTOMER_NAME)
#define NUM4STR(a,b,c,d)         #a "." #b "." #c "." #d
#define VERSIONBUILDSTR(a,b,c,d) NUM4STR(a,b,c,d)
#define VERSION_BUILD_STR        VERSIONBUILDSTR(VERSION_MAJOR,VERSION_MINOR,VERSION_REVISION,VERSION_BUILDNUM)

#endif /* ZY100_ONLINE_STRESS_TEST_VERSION_H */
