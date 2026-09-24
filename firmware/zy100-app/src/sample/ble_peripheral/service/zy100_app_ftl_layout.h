#ifndef ZY100_APP_FTL_LAYOUT_H
#define ZY100_APP_FTL_LAYOUT_H

/* ftl_save/ftl_load offsets are relative to the SDK application FTL base. */
#define ZY100_APP_FTL_LOGICAL_BYTES                    0x0BF0U

#define ZY100_APP_FTL_CAL_SLOT_BYTES                   264U
#define ZY100_APP_FTL_CAL_PRIMARY_OFFSET               0x0000U
#define ZY100_APP_FTL_CAL_BACKUP_OFFSET                \
    (ZY100_APP_FTL_CAL_PRIMARY_OFFSET + ZY100_APP_FTL_CAL_SLOT_BYTES)
#define ZY100_APP_FTL_CAL_END_OFFSET                   \
    (ZY100_APP_FTL_CAL_BACKUP_OFFSET + ZY100_APP_FTL_CAL_SLOT_BYTES)

#define ZY100_APP_FTL_FACTORY_RESUME_SLOT_BYTES        32U
#define ZY100_APP_FTL_FACTORY_RESUME_PRIMARY_OFFSET    0x0210U
#define ZY100_APP_FTL_FACTORY_RESUME_BACKUP_OFFSET     0x0230U
#define ZY100_APP_FTL_FACTORY_RESUME_END_OFFSET        0x0250U

#define ZY100_APP_FTL_FEATURE_CONFIG_SLOT_BYTES        32U
#define ZY100_APP_FTL_FEATURE_CONFIG_PRIMARY_OFFSET    0x0250U
#define ZY100_APP_FTL_FEATURE_CONFIG_BACKUP_OFFSET     0x0270U
#define ZY100_APP_FTL_FEATURE_CONFIG_END_OFFSET        0x0290U

typedef char zy100_app_ftl_cal_alignment_check[
    ((ZY100_APP_FTL_CAL_SLOT_BYTES & 3U) == 0U) ? 1 : -1];
typedef char zy100_app_ftl_cal_range_check[
    (ZY100_APP_FTL_CAL_END_OFFSET <=
     ZY100_APP_FTL_FACTORY_RESUME_PRIMARY_OFFSET) ? 1 : -1];
typedef char zy100_app_ftl_factory_resume_alignment_check[
    ((ZY100_APP_FTL_FACTORY_RESUME_SLOT_BYTES & 3U) == 0U) ? 1 : -1];
typedef char zy100_app_ftl_factory_resume_layout_check[
    ((ZY100_APP_FTL_FACTORY_RESUME_PRIMARY_OFFSET +
      ZY100_APP_FTL_FACTORY_RESUME_SLOT_BYTES) ==
     ZY100_APP_FTL_FACTORY_RESUME_BACKUP_OFFSET) ? 1 : -1];
typedef char zy100_app_ftl_factory_resume_range_check[
    (ZY100_APP_FTL_FACTORY_RESUME_END_OFFSET <=
     ZY100_APP_FTL_LOGICAL_BYTES) ? 1 : -1];
typedef char zy100_app_ftl_feature_config_alignment_check[
    ((ZY100_APP_FTL_FEATURE_CONFIG_SLOT_BYTES & 3U) == 0U) ? 1 : -1];
typedef char zy100_app_ftl_feature_config_layout_check[
    ((ZY100_APP_FTL_FEATURE_CONFIG_PRIMARY_OFFSET +
      ZY100_APP_FTL_FEATURE_CONFIG_SLOT_BYTES) ==
     ZY100_APP_FTL_FEATURE_CONFIG_BACKUP_OFFSET) ? 1 : -1];
typedef char zy100_app_ftl_feature_config_range_check[
    ((ZY100_APP_FTL_FACTORY_RESUME_END_OFFSET <=
      ZY100_APP_FTL_FEATURE_CONFIG_PRIMARY_OFFSET) &&
     (ZY100_APP_FTL_FEATURE_CONFIG_END_OFFSET <=
      ZY100_APP_FTL_LOGICAL_BYTES)) ? 1 : -1];

#endif /* ZY100_APP_FTL_LAYOUT_H */
