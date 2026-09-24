
/**
*****************************************************************************************
*     Copyright(c) 2016, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file    system_rtl8762c.c
   * @brief   system init file
   * @author  lory xu
   * @date    2017-11-9
   * @version v1.0
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2016 Realtek Semiconductor Corporation</center></h2>
   * *************************************************************************************
  */

/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include <locale.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "version.h"
#include "rtl876x.h"
#include "patch_header_check.h"
#include "app_section.h"
#include "rom_uuid.h"
#include "app_define.h"
#include "trace.h"
//#include "core_cmFunc.h"
#if defined ( __CC_ARM )
#include "cmsis_armcc.h"
#elif defined ( __GNUC__ )
#include "cmsis_gcc.h"
#endif
#include "debug_monitor.h"
#include "mem_config.h"
#include "otp.h"
#include "platform_autoconf.h"
#include "rtl876x_wdg.h"
#include "overlay_mgr.h"
#include "flash_device.h"
#include "os_sched.h"
#include "otp_config.h"
#include "test_mode.h"
#include "platform_utils.h"
#include "os_mem.h"
#include "rtl876x_wdg.h"
#include "rtl876x_lib_platform.h"
#include "board.h"
#include "system_rtl876x.h"
#include "app_define.h"
#if (SUPPORT_NORMAL_OTA == 1)
#include "dfu_flash.h"
#include "dfu_main.h"
#include "dfu_power_guard.h"
#endif
#if (SYSTEM_TRACE_ENABLE == 1)
#include "system_trace.h"
#include "trace_config.h"
#endif
#if (CONFIG_LOG_FUNCTION_ENABLE == 0)
#include "rtl876x_uart.h"
#endif

/** @defgroup  SYSTEM_INIT System Init
    * @brief Start up code for user application.
    * @{
    */
/*============================================================================*
 *                              Macros
 *============================================================================*/
/** @defgroup SYSTEM_INIT_Exported_Macros System Init Exported Macros
    * @brief
    * @{
    */
void SystemInit(void);
#define SHARE_CACHE_RAM_0K          0x82F70000
#define SHARE_CACHE_RAM_8K          0x2F2D0002
#define SHARE_CACHE_RAM_16K         0xA2AA0003

#define VTOR_RAM_ADDR               0x00200000 //!< vector table address in RAM.
#define APP_FAKE_PAYLOAD_LEN        0x100

#define FreeRTOS_portBYTE_ALIGNMENT 8

#define PAGE_element_data                          ( (FMC_PAGE_SIZE/8)-1)    // 511
#define MAX_logical_address_size                   (((PAGE_element_data*((OTP->ftl_size/4/1000)-1))-1)<<2)

#if (OTA_TEST == 1)
#undef VERSION_MAJOR
#undef VERSION_MINOR
#undef VERSION_REVISION
#undef VERSION_BUILDNUM

#if(OTA_TEST_BANK_SWITCH == 0) //test not bank switch
#if(OTA_TEST_IMAGE_VERSION == 0)
#define VERSION_MAJOR            1
#define VERSION_MINOR            0
#define VERSION_REVISION         0
#define VERSION_BUILDNUM         0
#else
#define VERSION_MAJOR            2
#define VERSION_MINOR            0
#define VERSION_REVISION         0
#define VERSION_BUILDNUM         0
#endif

#else //test bank switch
#if(APP_BANK == 0)
#define VERSION_MAJOR            1
#define VERSION_MINOR            0
#define VERSION_REVISION         0
#define VERSION_BUILDNUM         0
#else
#define VERSION_MAJOR            2
#define VERSION_MINOR            0
#define VERSION_REVISION         0
#define VERSION_BUILDNUM         0
#endif
#endif //end (OTA_TEST_BANK_SWITCH == 0)

#endif //end (OTA_TEST == 1)
/** End of SYSTEM_INIT_Exported_Macros
    * @}
    */

/*============================================================================*
 *                              Types
 *============================================================================*/
/** @defgroup SYSTEM_INIT_Exported_Types System Init Exported Types
    * @{
    */
typedef struct
{
    uint8_t ic_type;
    uint8_t rsvd0[3];
    uint32_t ram_size;
    uint32_t check_ver_pattern;
    T_VERSION_FORMAT git_ver;
    uint32_t check_level_pattern;
    uint8_t rsvd1[60];
} T_UPPERSTACK_HEADER;

typedef struct A_BLOCK_LINK
{
    struct A_BLOCK_LINK *pxNextFreeBlock;   /*<< The next free block in the list. */
    size_t xBlockSize : 28;                 /*<< The size of the free block. */
    size_t xRamType : 3;
    size_t xAllocateBit : 1;
} BlockLink_t;
/** End of SYSTEM_INIT_Exported_Types
    * @}
    */


/*============================================================================*
 *                              Variables
 *============================================================================*/
/** @defgroup SYSTEM_INIT_Exported_Variables System Init Exported Variables
    * @{
    */
extern void GPIO0_Handler(void);
extern void GPIO1_Handler(void);
extern void GPIO2_Handler(void);
extern void GPIO3_Handler(void);
extern void GPIO4_Handler(void);
extern void GPIO5_Handler(void);
extern void GPIO6_Handler(void);
extern void GPIO7_Handler(void);
extern void GPIO8_Handler(void);
extern void GPIO9_Handler(void);
extern void GPIO10_Handler(void);
extern void GPIO11_Handler(void);
extern void GPIO12_Handler(void);
extern void GPIO13_Handler(void);
extern void GPIO14_Handler(void);
extern void GPIO15_Handler(void);
extern void GPIO16_Handler(void);
extern void GPIO17_Handler(void);
extern void GPIO18_Handler(void);
extern void GPIO19_Handler(void);
extern void GPIO20_Handler(void);
extern void GPIO21_Handler(void);
extern void GPIO22_Handler(void);
extern void GPIO23_Handler(void);
extern void GPIO24_Handler(void);
extern void GPIO25_Handler(void);
extern void GPIO26_Handler(void);
extern void GPIO27_Handler(void);
extern void GPIO28_Handler(void);
extern void GPIO29_Handler(void);
extern void GPIO30_Handler(void);
extern void GPIO31_Handler(void);


extern uint32_t *__encryption_ram_code_load_ad__;
extern uint32_t *__encryption_ram_code_exe_ad__;
extern uint32_t *__encryption_ram_code_length__;
extern char Image$$ENCRYPTION_RAM_CODE$$Base[];
extern char Load$$ENCRYPTION_RAM_CODE$$Base[];
extern char Load$$ENCRYPTION_RAM_CODE$$Length[];

extern char Image$$FLASH_START_ADDR$$RO$$Base[];
extern char Load$$FLASH_START_ADDR$$RO$$Base[];
extern char Load$$FLASH_START_ADDR$$RO$$Length[];

BOOL_WDG_CB user_wdg_cb __attribute__((weak)) = NULL;
USER_CALL_BACK app_pre_main_cb __attribute__((weak)) = NULL ;
USER_CALL_BACK os_patch __attribute__((weak)) = NULL ;
uint32_t random_seed_value;

uint32_t user_boot_fw_selection(uint8_t fw1_valid, uint32_t fw1_sn, uint8_t fw2_valid,
                                uint32_t fw2_sn)
{
    return 1;//return 0 or 1
//  return 0xFFFFFFFF;  //error code
}

#define APP_CB_NUMBERS    1

typedef struct
{
    uint32_t app_cb_signature;
    uint32_t app_cb_numbers;
    uint32_t app_cb_addr[APP_CB_NUMBERS];
} T_APP_CB_TABLE;

const T_APP_CB_TABLE app_cb_table =
{
    .app_cb_signature = SIGNATURE_APP_CB,
    .app_cb_numbers = APP_CB_NUMBERS,
#if defined ( __CC_ARM   )
#pragma push
#pragma diag_suppress 1296 /* disable warning 1296(extened constant initialiser used)*/
#endif
    .app_cb_addr[0] = (uint32_t)user_boot_fw_selection,
#if defined ( __CC_ARM   )
#pragma pop
#endif
    //add more cb here
};



#if FEATURE_ENCRYPTION
#if defined ( __CC_ARM ) || defined ( __GNUC__ )
#define ENC_ALIGN_SECTION __attribute__((aligned(16), used, section(".enc.dummy.align")));
const uint8_t enc_dummy_align[16] ENC_ALIGN_SECTION;
#else
//#define ENC_ALIGN_SECTION @ ".enc.dummy.align"
//#pragma data_alignment=16
//__root const uint8_t enc_dummy_align[16] ENC_ALIGN_SECTION;
#endif
#endif

#if defined ( __CC_ARM   )
#pragma push
#pragma diag_suppress 1296 /* disable warning 1296(extened constant initialiser used)*/
#endif
/**
* @brief: application header.
* @note: items in ENCRYPT_RAM_CODE macro is for encrytion solution only
*/
#if defined ( __ICCARM__   )
#pragma section = "ENCRYPTION"
#pragma section = "ENCRYPTION_init"
#endif

#if defined ( __ICCARM__   )
__root
#endif
const T_IMG_HEADER_FORMAT img_header APP_FLASH_HEADER =
{
    .ctrl_header =
    {
        .ic_type = DEFINED_IC_TYPE,
        .secure_version = 0,
#if FEATURE_ENCRYPTION
        .ctrl_flag.flag_value.enc = 1,
        .ctrl_flag.flag_value.xip = 0,
        .ctrl_flag.flag_value.load_when_boot = 1,
        .ctrl_flag.flag_value.enc_key_select = ENC_KEY_OCEK_WITH_OEMCONST,
#else
        .ctrl_flag.flag_value.xip = 1,
        .ctrl_flag.flag_value.enc = 0,
        .ctrl_flag.flag_value.load_when_boot = 0,
        .ctrl_flag.flag_value.enc_key_select = 0,
#endif
        .ctrl_flag.flag_value.enc_load = 0,
        .ctrl_flag.flag_value.not_ready = 0,
        .ctrl_flag.flag_value.not_obsolete = 1,
        .ctrl_flag.flag_value.compressed_not_ready = 0,
        .ctrl_flag.flag_value.compressed_not_obsolete = 1,
#if (BOOT_INTEGRITY_CHECK_EN == 0)
        .ctrl_flag.flag_value.integrity_check_en_in_boot = 0,
#else
        .ctrl_flag.flag_value.integrity_check_en_in_boot = 1,
#endif
        .image_id = AppPatch,
        .payload_len = APP_FAKE_PAYLOAD_LEN,    //Will modify by build tool later
    },
    .uuid = DEFINE_rom_uuid,

#if FEATURE_ENCRYPTION
#if defined ( __ICCARM__ )
    .exe_base = (uint32_t)__section_begin("ENCRYPTION"),
    .load_base = (uint32_t)__section_begin("ENCRYPTION_init"),
    .load_len = (uint32_t)__section_size("ENCRYPTION"),
#elif defined (__CC_ARM)
    .load_base = (uint32_t)Load$$ENCRYPTION_RAM_CODE$$Base,
    .exe_base = (uint32_t)Image$$ENCRYPTION_RAM_CODE$$Base,
    .load_len = (uint32_t)Load$$ENCRYPTION_RAM_CODE$$Length,
#else
    .exe_base = (uint32_t) &__encryption_ram_code_exe_ad__, //(uint32_t)IMAGE_FLASH_START_ADDR,
    .load_base = (uint32_t) &__encryption_ram_code_load_ad__,
    .load_len = (uint32_t) &__encryption_ram_code_length__,
#endif
#else
#if defined ( __ICCARM__ )
    .load_base = 0,
    .exe_base = (uint32_t)SystemInit,
#elif defined (__CC_ARM)
    .load_base = (uint32_t)Load$$FLASH_START_ADDR$$RO$$Base,
    .exe_base = (uint32_t)Image$$FLASH_START_ADDR$$RO$$Base,
#else
    .load_base = 0,
    .exe_base = (uint32_t)SystemInit, //(uint32_t)IMAGE_FLASH_START_ADDR,
#endif
    .load_len = 0,  //0 indicates all XIP
#endif
#if (APP_BANK == 0)
    .image_base = BANK0_APP_ADDR,
#else
    .image_base = BANK1_APP_ADDR,
#endif

    .git_ver =
    {
        .ver_info.sub_version._version_major = VERSION_MAJOR,
        .ver_info.sub_version._version_minor = VERSION_MINOR,
        .ver_info.sub_version._version_revision = VERSION_REVISION,
        .ver_info.sub_version._version_reserve = VERSION_BUILDNUM % 32, //only 5 bit
        ._version_commitid = VERSION_GCID,
        ._customer_name = {CN_1, CN_2, CN_3, CN_4, CN_5, CN_6, CN_7, CN_8},
    },

#ifdef OTA_MAGIC_PATTERN
    .ota_magic_pattern = OTA_MAGIC_PATTERN,
#endif

    .app_cb_signature = SIGNATURE_APP_CB,
    .app_cb_table_base_address = (uint32_t) &app_cb_table,

};
#if defined ( __ICCARM__   )
__root
#endif
#if defined ( __ICCARM__   ) || defined ( __CC_ARM   )
const T_AUTH_HEADER_FORMAT auth_header APP_FLASH_HEADER =
{
#if defined ( __ICCARM__ )
    .header_mac = {0xFF},
    .payload_mac = {0xFF},
#else
    .header_mac = {[0 ... 15] = 0xFF},
    .payload_mac = {[0 ... 15] = 0xFF},
#endif
};
#elif defined ( __GNUC__   )
const T_AUTH_HEADER_FORMAT auth_header APP_FLASH_HEADER_AUTH =
{
    .header_mac = {[0 ... 15] = 0xFF},
    .payload_mac = {[0 ... 15] = 0xFF},
};
#endif

#if defined ( __CC_ARM   )
#pragma pop
#endif
void AppUpdateVectorTable(void);
void pre_main(void);
void wdg_system_reset_app_cb(T_WDG_MODE wdg_mode, T_SW_RESET_REASON reset_reason);
void random_seed_init(void);
void show_sdk_lib_version(void);

/** End of SYSTEM_INIT_Exported_Variables
    * @}
    */

/*============================================================================*
 *                              Functions
 *============================================================================*/
/** @defgroup SYSTEM_INIT_Exported_Functions System Init Exported Functions
    * @{
    */
void common_main(void)
{
    //add common system code here before enter user defined main function
#if (SUPPORT_NORMAL_OTA == 1)
    if (dfu_check_ota_mode_flag())
    {
        //flash size >= 256Mbit, switch to 4 bytes addr mode
        if ((flash_ioctl(flash_ioctl_get_rdid, 0, 0) & 0x0F) >= 9)
        {
            //from 3 byte addr mode enter 4 byte addr mode
            flash_ioctl(flash_ioctl_set_addr_mode, ENABLE, 0);
        }
        dfu_main();
        dfu_set_ota_mode_flag(false);
        goto START_SCHEDULER;
    }
#endif

    OTP->run_in_app = 1;

    random_seed_init();

#if (RUN_APP_IN_HCIMODE_ENABLE == 0)
    if (OTP->stack_en)
    {
        DBG_DIRECT("In SoC Mode");
    }
    else
    {
        DBG_DIRECT("WARNING: In HCI Mode, will not run APP Task");
        WDG_Disable();
        goto START_SCHEDULER;
    }
#endif

    show_sdk_lib_version();

#if (CONFIG_LOG_FUNCTION_ENABLE == 0)
    void log_disable(void);
    log_disable();
#endif

#if (SYSTEM_TRACE_ENABLE == 1)
    extern void system_trace_init(void);
    system_trace_init();
#endif

#if defined ( __ICCARM__ )
    extern void __iar_program_start(const int);
    __iar_program_start(0);
#elif defined (__CC_ARM)
    extern int __main(void);
    __main();
#elif defined (__GNUC__)
    extern int main(void);
    main();
#endif
START_SCHEDULER:
    os_sched_start();
    return;
}

void random_seed_init(void)
{
    random_seed_value = platform_random(0xFFFFFFFF);
}

__attribute__((weak)) void show_sdk_lib_version(void)
{
}

#if (DEBUG_WATCHPOINT_ENABLE == 1)
/**
 * @brief  Enable Debug Monitor Function (include NVIC Enable and DWT configuration)
 * @param  none
 * @return none
 */
void debug_monitor_enable(void) DATA_RAM_FUNCTION;
void debug_monitor_enable(void)
{
    //DBG_DIRECT("debug_monitor_enable");

    //set debug monitor priority
    NVIC_SetPriority(DebugMonitor_IRQn, 3);

    //enable exception and monitor control register
    CoreDebug->DEMCR |= CoreDebug_DEMCR_MON_EN_Msk | CoreDebug_DEMCR_TRCENA_Msk;

    //set DWT compare registers (max 4 comparators)
    //watch_point_0_setting(0x1000180C, DWT_DATAVSIZE_WORD, DWT_FUNCTION_READ);
    //watch_point_1_setting(0x10000004, DWT_DATAVSIZE_HALF_WORD, DWT_FUNCTION_WRITE);
    //watch_point_2_setting(0x10000008, DWT_DATAVSIZE_BYTE, DWT_FUNCTION_READ_OR_WRITE);
    //watch_point_3_setting(0x1000000C, DWT_DATAVSIZE_WORD, DWT_FUNCTION_READ_OR_WRITE);

    //If use the following APIs, the start address shoule be aligned to the data size
    //watch_point_0_setting_align(0x201000, DWT_DATASIZE_ALIGN_4_BYTE, DWT_FUNCTION_READ);
    //watch_point_1_setting_align(0x201008, DWT_DATASIZE_ALIGN_8_BYTE, DWT_FUNCTION_WRITE);
    //watch_point_2_setting_align(0x201010, DWT_DATASIZE_ALIGN_16_BYTE, DWT_FUNCTION_READ_OR_WRITE);
    //watch_point_3_setting_align(0x201020, DWT_DATASIZE_ALIGN_32_BYTE, DWT_FUNCTION_READ_OR_WRITE);

    //enable DWT control register
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    return;
}
#endif

void ram_init(void) APP_FLASH_TEXT_SECTION;
void ram_init(void)
{
    extern uint32_t flash_split_read(uint32_t start_addr, uint32_t data_len, uint8_t *data);

    //copy data on ro
#if defined ( __CC_ARM   )
    extern char Image$$RAM_DATA_ON$$RO$$Base[];
    extern char Load$$RAM_DATA_ON$$RO$$Base[];
    extern unsigned int Image$$RAM_DATA_ON$$RO$$Length;

//    memcpy(Image$$RAM_DATA_ON$$RO$$Base,
//           Load$$RAM_DATA_ON$$RO$$Base,
//           (unsigned int)&Image$$RAM_DATA_ON$$RO$$Length);

    flash_split_read((uint32_t)Load$$RAM_DATA_ON$$RO$$Base,
                     (unsigned int)&Image$$RAM_DATA_ON$$RO$$Length, \
                     (uint8_t *)Image$$RAM_DATA_ON$$RO$$Base);

    //copy data on rw
    extern char Image$$RAM_DATA_ON$$RW$$Base[];
    extern char Load$$RAM_DATA_ON$$RW$$Base[];
    extern unsigned int Image$$RAM_DATA_ON$$RW$$Length;

//    memcpy(Image$$RAM_DATA_ON$$RW$$Base,
//           Load$$RAM_DATA_ON$$RW$$Base,
//           (unsigned int)&Image$$RAM_DATA_ON$$RW$$Length);

    flash_split_read((uint32_t)Load$$RAM_DATA_ON$$RW$$Base,
                     (unsigned int)&Image$$RAM_DATA_ON$$RW$$Length, \
                     (uint8_t *)Image$$RAM_DATA_ON$$RW$$Base);

    //clear data on zi
    extern char Image$$RAM_DATA_ON$$ZI$$Base[];
    extern unsigned int Image$$RAM_DATA_ON$$ZI$$Length;

    memset(Image$$RAM_DATA_ON$$ZI$$Base,
           0,
           (unsigned int)&Image$$RAM_DATA_ON$$ZI$$Length);
#elif defined ( __GNUC__ )
    //copy data on ro
    extern uint32_t *__ram_dataon_ro_start__;
    extern uint32_t *__ram_ro_load_ad__;
    extern uint32_t *__ram_dataon_ro_length__;

//    memcpy(&__ram_dataon_ro_start__,
//           &__ram_ro_load_ad__,
//           (unsigned int)&__ram_dataon_ro_length__);

    flash_split_read((uint32_t)&__ram_ro_load_ad__,
                     (unsigned int)&__ram_dataon_ro_length__, \
                     (uint8_t *)&__ram_dataon_ro_start__);

    //copy data on rw
    extern uint32_t *__ram_dataon_rw_start__;
    extern uint32_t *__ram_rw_load_ad__;
    extern uint32_t *__ram_dataon_rw_length__;

//    memcpy(&__ram_dataon_rw_start__,
//           &__ram_rw_load_ad__,
//           (unsigned int)&__ram_dataon_rw_length__);

    volatile uint32_t rw_data_length = (unsigned int)&__ram_dataon_rw_length__;
    if (rw_data_length)
    {
        flash_split_read((uint32_t)&__ram_rw_load_ad__,
                         (unsigned int)&__ram_dataon_rw_length__, \
                         (uint8_t *)&__ram_dataon_rw_start__);
    }

    //clear data on zi
    extern uint32_t *__ram_dataon_zi_start__;
    extern uint32_t *__ram_dataon_zi_length__;

    memset(&__ram_dataon_zi_start__,
           0,
           (unsigned int)&__ram_dataon_zi_length__);
#elif  defined (__ICCARM__)
    extern void __iar_data_init2(void);
    __iar_data_init2();
#endif

}

APP_FLASH_TEXT_SECTION
void ram_cache_init(void)
{
    //copy cache ro
#if defined ( __CC_ARM   )
    extern char Image$$CACHE_DATA_ON$$RO$$Base[];
    extern char Load$$CACHE_DATA_ON$$RO$$Base[];
    extern unsigned int Image$$CACHE_DATA_ON$$RO$$Length;

    memcpy(Image$$CACHE_DATA_ON$$RO$$Base,
           Load$$CACHE_DATA_ON$$RO$$Base,
           (unsigned int)&Image$$CACHE_DATA_ON$$RO$$Length);

    //copy share cache ram rw
    extern char Image$$CACHE_DATA_ON$$RW$$Base[];
    extern char Load$$CACHE_DATA_ON$$RW$$Base[];
    extern unsigned int Image$$CACHE_DATA_ON$$RW$$Length;

    memcpy(Image$$CACHE_DATA_ON$$RW$$Base,
           Load$$CACHE_DATA_ON$$RW$$Base,
           (unsigned int)&Image$$CACHE_DATA_ON$$RW$$Length);

    //clear share cache ram zi
    extern char Image$$CACHE_DATA_ON$$ZI$$Base[];
    extern unsigned int Image$$CACHE_DATA_ON$$ZI$$Length;

    memset(Image$$CACHE_DATA_ON$$ZI$$Base,
           0,
           (unsigned int)&Image$$CACHE_DATA_ON$$ZI$$Length);
#elif defined ( __GNUC__   )
    //copy share cache ram rw
    extern uint32_t *__cache_dataon_start__;
    extern uint32_t *__cache_data_load_ad__;
    extern uint32_t *__cache_dataon_length__;

    memcpy(&__cache_dataon_start__,
           &__cache_data_load_ad__,
           (unsigned int)&__cache_dataon_length__);
#endif
}

#ifdef SUPPORT_ALONE_UPPERSTACK_IMG
APP_FLASH_TEXT_SECTION
bool check_header_valid_imp(uint32_t header_addr, T_IMG_ID image_id)
{
    if (header_addr == 0)
    {
        return false;
    }

    T_IMG_CTRL_HEADER_FORMAT *header = (T_IMG_CTRL_HEADER_FORMAT *)header_addr;
    if (header->ic_type != DEFINED_IC_TYPE)
    {
        return false;
    }

    if (image_id > OTA && image_id < IMAGE_MAX)
    {
        char c_rom_uuid[16]  = DEFINE_rom_uuid;

        if (header->ctrl_flag.flag_value.not_ready
            || header->image_id != image_id)
        {
            return false;
        }

        if (0 == memcmp(((T_IMG_HEADER_FORMAT *)header)->uuid, c_rom_uuid, 16))
        {
            return true;
        }
    }

    return false;
}

APP_FLASH_TEXT_SECTION
uint32_t get_header_addr_by_img_id_imp(T_IMG_ID image_id)
{
    uint32_t image_addr = 0;

    if (image_id <= OTA || image_id >= IMAGE_MAX)
    {
        return image_addr;
    }

    uint32_t ota_header_addr = get_active_ota_bank_addr();

    if (!check_header_valid(ota_header_addr, OTA))
    {
        return image_addr;
    }

    T_OTA_HEADER_FORMAT *ota_header = (T_OTA_HEADER_FORMAT *)ota_header_addr; //ota_header_addr

    image_addr = HAL_READ32((uint32_t)&ota_header->secure_boot_addr, (image_id - SecureBoot) * 8);
    if (image_addr == 0)
    {
        return image_addr;
    }
    else
    {
        return (image_addr | FLASH_OFFSET_TO_NO_CACHE);
    }
}

APP_FLASH_TEXT_SECTION
APP_MAIN_FUNC get_image_entry_addr(uint16_t image_id)
{

    uint32_t active_ota_bank_addr =  get_active_ota_bank_addr();
    if (!check_header_valid(active_ota_bank_addr, OTA))
    {
        return NULL;
    }

    T_OTA_HEADER_FORMAT *ota_header = (T_OTA_HEADER_FORMAT *)active_ota_bank_addr;
    uint32_t image_size =
        HAL_READ32((uint32_t)&ota_header->secure_boot_size, (image_id - SecureBoot) * 8);

    if (image_size == 0)
    {
        return NULL;
    }

    uint32_t header_addr = get_header_addr_by_img_id_imp((T_IMG_ID)image_id);

    if (!check_header_valid_imp(header_addr, (T_IMG_ID)image_id))
    {
        /*if header_addr=0, will return here*/
        return NULL;
    }

    T_IMG_HEADER_FORMAT *header = (T_IMG_HEADER_FORMAT *)header_addr;

    if (header->ctrl_header.image_id != image_id || image_id >= IMAGE_MAX)
    {
        return NULL;
    }

    APP_MAIN_FUNC entry_func = (APP_MAIN_FUNC)(header->exe_base | 1);

    return entry_func;
}

#else
bool check_upperstack_img(void) APP_FLASH_TEXT_SECTION;
bool check_upperstack_img(void)
{
#if   defined ( __CC_ARM ) || defined (__ICCARM__)
    T_UPPERSTACK_HEADER *p_header = (T_UPPERSTACK_HEADER *)UPPERSTACK_FLASH_ADDR;
#elif defined ( __GNUC__ )
    extern uint32_t *__upperstack_flash_start_load_ad__;
    T_UPPERSTACK_HEADER *p_header = (T_UPPERSTACK_HEADER *)&__upperstack_flash_start_load_ad__;
#endif
    if (DEFINED_IC_TYPE != p_header->ic_type)
    {
        return false;
    }

    if (UPPERSTACK_GLOBAL_SIZE < p_header->ram_size)
    {
        return false;
    }

    return (((uint32_t)&upperstack_fake_data) == p_header->check_ver_pattern)
           && (((uint32_t)&upperstack_level_fake_data) == p_header->check_level_pattern);
}
#endif

#if defined ( __ICCARM__ )
#if FEATURE_ENCRYPTION
void SystemInit(void) APP_ENCRYPTION_TEXT_SECTION;
#else
void SystemInit(void) APP_FLASH_TEXT_SECTION;
#endif
#else
void SystemInit(void) APP_FLASH_TEXT_SECTION;
#endif

void SystemInit(void)
{
    if (flash_try_high_speed(FLASH_MODE_2BIT) != FLASH_SUCCESS)
    {
        DBG_DIRECT("Flash change to 2 BIT Mode Fail!");
    }

    //hci mode check and bypass app
    if (check_hci_mode_flag() || (OTP->stack_en == 0))
    {
        return;
    }

#ifndef SUPPORT_ALONE_UPPERSTACK_IMG
    if (!check_upperstack_img())
    {
        extern void LOGUARTDriverInit(void);
        LOGUARTDriverInit();
        DBG_DIRECT("upperstack img doesn't match the symbol!");
        return;
    }
#endif

    //init pre_main and main functions
    app_pre_main = (APP_MAIN_FUNC)pre_main;

#ifdef SUPPORT_ALONE_UPPERSTACK_IMG
    upperstack_entry = get_image_entry_addr(UpperStack);
#else
    upperstack_entry = (APP_MAIN_FUNC)upperstack_loader;
#endif
    app_main = (APP_MAIN_FUNC)common_main;

    app_cb_wdg_reset = (APP_CB_WDG_RESET_TYPE)wdg_system_reset_app_cb;

#if (WRITE_HARDFAULT_RECORD_TO_FLASH_ENABLE > 0) && (SYSTEM_TRACE_ENABLE == 1) && (TRACE_HARDFAULT == 1)
    patch_hardfault_save_to_flash_init();
#endif

#if ((SYSTEM_TRACE_ENABLE == 1) && ((WRITE_REASON_TO_FLASH_BEFORE_RESET_ENABLE > 0 && TRACE_WDG_TIMEOUT == 1) || (TRACE_TASK_HANG_USE_WDG_ISR == 1)))
    patch_wdg_timeout_reason_save_to_flash_init();
    patch_WDG_SystemReset_init();
#endif

    /******** update otp here**********/
    //ram config
    OTP->appDataAddr = APP_GLOBAL_ADDR;
    OTP->appDataSize = APP_GLOBAL_SIZE;
    OTP->heapDataONSize = HEAP_DATA_ON_SIZE;

    /* not share: SHARE_CACHE_RAM_0K; 8K: SHARE_CACHE_RAM_8K; 16K: SHARE_CACHE_RAM_16K */
#if ((16 * 1024) == SHARE_CACHE_RAM_SIZE)
    OTP->share_cache_ram_reg = SHARE_CACHE_RAM_16K;
#elif ((8 * 1024) == SHARE_CACHE_RAM_SIZE)
    OTP->share_cache_ram_reg = SHARE_CACHE_RAM_8K;
#else
    OTP->share_cache_ram_reg = SHARE_CACHE_RAM_0K;
#endif

#ifdef SUPPORT_SINGLE_BANK_OTA_USER_DATA
    OTP->bkp_data1_addr = USER_DATA1_ADDR;
    OTP->bkp_data1_size = USER_DATA1_SIZE;
    OTP->bkp_data2_addr = USER_DATA2_ADDR;
    OTP->bkp_data2_size = USER_DATA2_SIZE;

#if (DISABLE_AES_OTA == 1)
    OTP->ota_with_encryption_data = 0;
#endif
#endif

    OTP->ota_with_encryption_data = OTA_WITH_ENCRYPTION_DATA;

    //sw timer config
#ifdef TIMER_MAX_NUMBER
    //define TIMER_MAX_NUMBER in otp_config.h
    OTP->timerMaxNumber = TIMER_MAX_NUMBER;
#endif

    //flash config
    /*config enable flash block proect depending on flash layout and flash id*/
#if (FLASH_BLOCK_PROTECT_ENABLE == 1)
    OTP->bp_enable = 1;
#else
    OTP->bp_enable = 0;
#endif
    OTP->delay_10us_after_toggle_cs = AFTER_TOGGLE_CS_DELAY;

    //os config
    /*config enable check task stack overflow*/
#if (CHECK_STACK_OVERFLOW_ENABLE == 1)
    OTP->checkForStackOverflow = 1;
#else
    OTP->checkForStackOverflow = 0;
#endif


    //platform config
    /*config enable platform assert*/
#if (PLATFORM_ASSERT_ENABLE == 1)
    OTP->enableASSERT = 1;
#else
    OTP->enableASSERT = 0;
#endif

    /*Print all log in log buffer before entering DLPS */
#if (CHECK_LOG_BUFFER_BEFORE_DLPS_ENABLE == 1)
    OTP->printAllLogBeforeEnterDLPS = 1;
#else
    OTP->printAllLogBeforeEnterDLPS = 0;
#endif

    /*config enable log or not*/
#if (CONFIG_LOG_FUNCTION_ENABLE == 1)
    OTP->logDisable = 0;
#else
    OTP->logDisable = 1;
#endif

    /*config log baudrate*/
#ifdef LOG_BAUD_RATE
    OTP-> logBaudRate = LOG_BAUD_RATE;
#endif

    /*config ftl app logical addr base*/
#if (FTL_BT_STORAGE_SPACE_SIZE > 0)
    if (FTL_BT_STORAGE_SPACE_SIZE * 1024 <= MAX_logical_address_size)
    {
        OTP->ftl_app_logical_addr_base = FTL_BT_STORAGE_SPACE_SIZE;
    }
    else
    {
        DBG_DIRECT("ERROR! FTL BT storage space size is too large!");
    }
#endif

    /*config enable swd pinmux*/
#if (SWD_PINMUX_ENABLE == 1)
    OTP->SWD_ENABLE = 1;
#else
    OTP->SWD_ENABLE = 0;
#endif

    /*config enable watch dog in rom*/
#if (ROM_WATCH_DOG_ENABLE == 1)
    OTP->wdgEnableInRom = 1;
#else
    OTP->wdgEnableInRom = 0;
#endif

    /*config watch dog mode in rom, defualt 4s timeout and reset all*/
#if (ROM_WATCH_DOG_ENABLE == 1)
    OTP->wdgConfigDivfactor = ROM_WATCH_DOG_CFG_DIV_FACTOR;
    OTP->wdgConfigCntLimit = ROM_WATCH_DOG_CFG_CNT_LIMIT;
#if (TRACE_TASK_HANG_USE_WDG_ISR == 1)
    OTP->wdgMode = INTERRUPT_CPU;
#else
    OTP->wdgMode = ROM_WATCH_DOG_MODE;
#endif
#endif

    /*Debug: config enable write hardfault record to flash*/
#if (WRITE_HARDFAULT_RECORD_TO_FLASH_ENABLE > 0)
    OTP->write_info_to_flash_when_hardfault = WRITE_HARDFAULT_RECORD_TO_FLASH_ENABLE;
    OTP->HardFault_Record_BegAddr = HARDFAULT_RECORD_BEG_ADDR;
    OTP->HardFault_Record_EndAddr = HARDFAULT_RECORD_END_ADDR;
    OTP->HardFault_Record_CFG = HARDFAULT_RECORD_CFG;
#endif

    /*Debug: before wdg system reset, write reset reason to specific flash addr*/
#if (WRITE_REASON_TO_FLASH_BEFORE_RESET_ENABLE > 0)
    OTP->write_info_to_flash_before_reset = WRITE_REASON_TO_FLASH_BEFORE_RESET_ENABLE;
    OTP->reboot_record_address = REBOOT_REASON_RECORD_ADDRESS;
    OTP->reboot_record_item_limit_power_2 = REBOOT_REASON_RECORD_LIMIT_POWERT2;
#endif

#if (ENABLE_FLASH_READ_TURN_OFF_RF > 0)
    OTP->read_turn_on_off_rf = ENABLE_FLASH_READ_TURN_OFF_RF;
#endif

    /* defaut is not FR1*/
#if (ENABLE_FR1_IC_CONFIG > 0)
    OTP->not_FR1 = 0;
#else
    OTP->not_FR1 = 1;
#endif

#if (RESET_WHEN_HARDFAULT_ENABLE > 0)
    OTP->resetWhenHardFault = 1;
#else
    OTP->resetWhenHardFault = 0;
#endif

    //app config
    OTP->ota_timeout_total = OTA_TIMEOUT_TOTAL;
    OTP->ota_timeout_wait4_conn = OTA_TIMEOUT_WAIT4_CONN;
    OTP->ota_timeout_wait4_image_transfer = OTA_TIMEOUT_WAIT4_IMAGE_TRANS;
    OTP->ota_timeout_ctittv = OTA_TIMEOUT_CTITTV;

#if ROM_OTA_LINKLOSS_RST
    OTP->ota_link_loss_reset = 1;
#endif


    /*config bt stack parameters in rom*/
#ifdef BT_STACK_CONFIG_ENABLE
    bt_stack_config_init();
#endif
#if defined ( __ICCARM__ )
    app_pre_main_cb = 0;
#endif
//add more otp config here

    ram_init();

    if (os_patch)
    {
        os_patch();
    }
}

void print_reset_reason(void) APP_FLASH_TEXT_SECTION;
void print_reset_reason(void)
{
    BOOT_PRINT_INFO0("Notes: The Following RESET Reason Info is unreliable!");
    T_SW_RESET_REASON sw_reset_type = reset_reason_get();
    if (sw_reset_type == RESET_REASON_HW)
    {
        BOOT_PRINT_INFO0("[Debug Info]RESET Reason: HW");
    }
    else
    {
        BOOT_PRINT_INFO1("[Debug Info]RESET Reason: SW, TYPE 0x%x", sw_reset_type);
    }
}

void pre_main(void) APP_FLASH_TEXT_SECTION ;
void pre_main(void)
{
    __disable_irq();

    print_reset_reason();
    BOOT_PRINT_INFO0("[ZY100_SYS] SDK v1.1.0 system_rtl876x.c active");
#if (SUPPORT_NORMAL_OTA == 1)
    /* APP entry only: absence of this marker does not prove ROM pad state. */
    dfu_power_guard_trace_pin("app_pre_main");
#endif

    ram_cache_init();

#if defined ( __CC_ARM   )
    load_overlay(OVERLAY_SCENARIO_BOOT_ONCE);
#endif
    //reset NVIC of DMA channel used in image decryption
    NVIC_DisableIRQ(GDMA0_Channel2_IRQn);
    NVIC_DisableIRQ(GDMA0_Channel3_IRQn);

#if defined ( __CC_ARM   )
    setlocale(LC_ALL, "C");
#endif

    BOOT_PRINT_ERROR2("SDK Ver: %s, Build Time: %s",
                      TRACE_STRING(VERSION_BUILD_STR),
                      TRACE_STRING(BUILDING_TIME));

    AppUpdateVectorTable();

#if (DEBUG_WATCHPOINT_ENABLE == 1)
    debug_monitor_enable();
#endif

    if (app_pre_main_cb)
    {
        app_pre_main_cb();
    }

    return;
}

APP_FLASH_TEXT_SECTION
void wdg_system_reset_app_cb(T_WDG_MODE wdg_mode, T_SW_RESET_REASON reset_reason)
{
    //do something necessary before watch dog reset
    if (user_wdg_cb)
    {
        if (user_wdg_cb(wdg_mode, reset_reason))
        {
            return;
        }
    }

}
/**
 * @brief  update vector table in app
 * @param  none
 * @return none
  */
//void AppUpdateVectorTable(void) OVERLAY_SECTION_BOOT_ONCE;
void AppUpdateVectorTable(void)
{
#if defined ( __CC_ARM )
    extern uint32_t Load$$RAM_VECTOR_TABLE$$RO$$Base;
    extern uint32_t Image$$RAM_VECTOR_TABLE$$RO$$Length;
#elif defined ( __GNUC__ )
    extern uint32_t *__ram_vector_load_ad__;
    extern uint32_t *__ram_vector_table_length__;
#endif
    extern void Default_Handler(void);

    const char *SysException[] =
    {
        "InitialSP", "Reset", "NMI", "HardFault", "MemManage", "BusFault", "UsageFault", "Rsvd",
        "Rsvd", "Rsvd", "Rsvd", "SVC", "DebugMon", "Rsvd", "PendSV", "SysTick"
    };
    const char *ExtIrq[] =
    {
        "System", "WDG", "BTMAC", "TIM3", "TIM2", "Platform", "I2S0_TX", "I2S0_RX", "Timer4-7",
        "GPIO4", "GPIO5", "UART1", "UART0", "RTC", "SPI0", "SPI1", "I2C0", "I2C1", "ADC",
        "Peripheral", "GDMA0 Channel0", "GDMA0 Channel1", "GDMA0 Channel2", "GDMA0 Channel3",
        "GDMA0 Channel4", "GDMA0 Channel5", "GPIO_Group3", "GPIO_Group2", "IR", "GPIO_Group1",
        "GPIO_Group0", "UART2", "TIM4", "TIM5", "TIM6", "TIM7", "SPI_Flash", "Qdecode",
        "Keyscan", "SPI2W", "LPCOMP", "PTA_Mailbox", "SPORT1 TX", "SPORT1 RX", "LCD"
    };

    IRQ_Fun *pRamVector    = (IRQ_Fun *)VTOR_RAM_ADDR;
#if defined ( __ICCARM__ )
#pragma section = ".intvec"
    IRQ_Fun *pAppVector    = (IRQ_Fun *)__section_begin(".intvec");
    uint32_t AppVectorSize = (uint32_t)__section_size(".intvec");
    DBG_DIRECT("pAppVector = 0x%x, AppVectorSize = %d", pAppVector, AppVectorSize);
#elif defined (__CC_ARM)
    IRQ_Fun *pAppVector    = (IRQ_Fun *)&Load$$RAM_VECTOR_TABLE$$RO$$Base;
    uint32_t AppVectorSize = (uint32_t)&Image$$RAM_VECTOR_TABLE$$RO$$Length;
#elif defined (__GNUC__)
    IRQ_Fun *pAppVector    = (IRQ_Fun *)(&__ram_vector_load_ad__);
    uint32_t AppVectorSize = (uint32_t)&__ram_vector_table_length__;
#endif
    uint32_t i             = 0;

    if (SCB->VTOR != VTOR_RAM_ADDR)
    {
        RamVectorTableInit(VTOR_RAM_ADDR);
    }

    /* Update APP defined handlers */
    for (i = 0; i < AppVectorSize / 4; ++i)
    {
        if (i == 0 || i == 1) //skip __initial_sp and reset_handler remap
        {
            continue;
        }

        if (((pAppVector[i] != Default_Handler) && (pAppVector[i] != 0)) ||
            ((pAppVector[i] == Default_Handler) && (pRamVector[i] == (IRQ_Fun)ROM_Default_Handler)))
        {
            if (i < System_VECTORn)
            {
                OS_PRINT_WARN1("Warning! %s is updated by APP!", TRACE_STRING(SysException[i]));
            }
            else
            {
                OS_PRINT_WARN1("Warning! ISR %s is updated by APP!",
                               TRACE_STRING(ExtIrq[i - System_VECTORn]));
            }

            pRamVector[i] = pAppVector[i];
        }
    }

    __DMB();
    __DSB();
}
/**
 * @brief  GPIO Group3 Handler
 * @param  none
 * @return none
  */
void GPIO_Group3_Handler(void) DATA_RAM_FUNCTION;
void GPIO_Group3_Handler(void)
{
    uint32_t GPIOIrqStatus = GPIO->INTSTATUS;

    //Check exact IRQ function
    if (GPIOIrqStatus & BIT3)
    {
        GPIO3_Handler();
    }
    if (GPIOIrqStatus & BIT7)
    {
        GPIO7_Handler();
    }
    if (GPIOIrqStatus & BIT11)
    {
        GPIO11_Handler();
    }
    if (GPIOIrqStatus & BIT15)
    {
        GPIO15_Handler();
    }
    if (GPIOIrqStatus & BIT19)
    {
        GPIO19_Handler();
    }
    if (GPIOIrqStatus & BIT23)
    {
        GPIO23_Handler();
    }
    if (GPIOIrqStatus & BIT27)
    {
        GPIO27_Handler();
    }
    if (GPIOIrqStatus & BIT31)
    {
        GPIO31_Handler();
    }
}
/**
 * @brief  GPIO Group2 Handler
 * @param  none
 * @return none
  */
void GPIO_Group2_Handler(void) DATA_RAM_FUNCTION;
void GPIO_Group2_Handler(void)
{
    uint32_t GPIOIrqStatus = GPIO->INTSTATUS;

    //Check exact IRQ function
    if (GPIOIrqStatus & BIT2)
    {
        GPIO2_Handler();
    }
    if (GPIOIrqStatus & BIT6)
    {
        GPIO6_Handler();
    }
    if (GPIOIrqStatus & BIT10)
    {
        GPIO10_Handler();
    }
    if (GPIOIrqStatus & BIT14)
    {
        GPIO14_Handler();
    }
    if (GPIOIrqStatus & BIT18)
    {
        GPIO18_Handler();
    }
    if (GPIOIrqStatus & BIT22)
    {
        GPIO22_Handler();
    }
    if (GPIOIrqStatus & BIT26)
    {
        GPIO26_Handler();
    }
    if (GPIOIrqStatus & BIT30)
    {
        GPIO30_Handler();
    }
}
/**
 * @brief  GPIO Group1 Handler
 * @param  none
 * @return none
  */
void GPIO_Group1_Handler(void) DATA_RAM_FUNCTION;
void GPIO_Group1_Handler(void)
{
    uint32_t GPIOIrqStatus = GPIO->INTSTATUS;

    //Check exact IRQ function
    if (GPIOIrqStatus & BIT1)
    {
        GPIO1_Handler();
    }
    if (GPIOIrqStatus & BIT9)
    {
        GPIO9_Handler();
    }
    if (GPIOIrqStatus & BIT13)
    {
        GPIO13_Handler();
    }
    if (GPIOIrqStatus & BIT17)
    {
        GPIO17_Handler();
    }
    if (GPIOIrqStatus & BIT21)
    {
        GPIO21_Handler();
    }
    if (GPIOIrqStatus & BIT25)
    {
        GPIO25_Handler();
    }
    if (GPIOIrqStatus & BIT29)
    {
        GPIO29_Handler();
    }
}
/**
 * @brief  GPIO Group0 Handler
 * @param  none
 * @return none
  */
void GPIO_Group0_Handler(void) DATA_RAM_FUNCTION;
void GPIO_Group0_Handler(void)
{
    uint32_t GPIOIrqStatus = GPIO->INTSTATUS;

    //Check exact IRQ function
    if (GPIOIrqStatus & BIT0)
    {
        GPIO0_Handler();
    }
    if (GPIOIrqStatus & BIT8)
    {
        GPIO8_Handler();
    }
    if (GPIOIrqStatus & BIT12)
    {
        GPIO12_Handler();
    }
    if (GPIOIrqStatus & BIT16)
    {
        GPIO16_Handler();
    }
    if (GPIOIrqStatus & BIT20)
    {
        GPIO20_Handler();
    }
    if (GPIOIrqStatus & BIT24)
    {
        GPIO24_Handler();
    }
    if (GPIOIrqStatus & BIT28)
    {
        GPIO28_Handler();
    }
}

#if (RETARGET_PRINTF_METHOD == 0)
int __2printf(const char *fmt, ...)
{
    return 0;
}
#endif

void *malloc(size_t size)
{
    return os_mem_alloc(RAM_TYPE_DATA_ON, size);
}

void *calloc(size_t n, size_t size)
{
    return os_mem_zalloc(RAM_TYPE_DATA_ON, n * size);
}

/*only for FreeRTOS*/
void *realloc(void *ptr, size_t size)
{
    void *new_ptr;
    BlockLink_t *pxLink;
    size_t old_size, new_size = 0;

    if (ptr == NULL)
    {
        return os_mem_alloc(RAM_TYPE_DATA_ON, size);
    }

    if (size == 0)
    {
        os_mem_free(ptr);
        ptr = NULL;
        return NULL;
    }

    pxLink = (BlockLink_t *)((uint8_t *)ptr - sizeof(BlockLink_t));
    old_size = pxLink->xBlockSize;

    if ((old_size - sizeof(BlockLink_t)) ==
        (size + (FreeRTOS_portBYTE_ALIGNMENT - (size & (FreeRTOS_portBYTE_ALIGNMENT - 1)))))
    {
        return ptr;
    }

    os_sched_suspend();
    new_ptr = os_mem_alloc(RAM_TYPE_DATA_ON, size);
    if (ptr != new_ptr)
    {
        pxLink = (BlockLink_t *)((uint8_t *)new_ptr - sizeof(BlockLink_t));
        new_size = pxLink->xBlockSize;
        memcpy(new_ptr, ptr, (old_size < new_size) ? (old_size - sizeof(BlockLink_t)) : (new_size - sizeof(
                    BlockLink_t)));
        os_mem_free(ptr);
    }
    os_sched_resume();
    return new_ptr;
}

void free(void *ptr)
{
    os_mem_free(ptr);
}

/**
 * @brief  get cpu clock
 * @param  none
 * @return uint32, for example 40000000 is 40M, 20000000 is 20M.
  */
uint32_t get_cpu_clock(void)
{
    extern uint32_t SystemCpuClock;
    return SystemCpuClock;
}

/**
 * @brief  set auto slow clock when cpu sleep
 * @param  enable or disable auto slow clock
 * @return none.
  */
void set_auto_slow_clock(bool enable)
{
    //fix iar compiler warning: mixing boolean and non-boolean types in a comparision
    uint8_t set = enable ? 1 : 0;

    if (SYSBLKCTRL->u_200.BITS_200.r_cpu_slow_en != set)
    {
        OTP->r_cpu_slow_en = enable;
        SYSBLKCTRL->u_200.BITS_200.r_cpu_slow_en = enable;
    }
}


/**
 * @brief  get 14 bytes EUID
 * @param  none
 * @return uint8_t *, the pointer to a copy of EUID.
  */
uint8_t *get_ic_euid(void)
{
    static uint8_t euid[15] = {0};

    memcpy(euid, (uint8_t *)0x202545, 14);
    return euid;
}

/**
 * @brief  get ic type
 * @param  none
 * @return uint8_t, ic type:
 * #define RTL8762DJ               0x11
 * #define RTL8752DJ               0xB1
 * #define RTL8762DDF              0x59
 * #define RTL8762DKF              0x1A
 * #define RTL8762DGF              0x18
 * #define RTL8762DFA              0x2A
 * #define RTL8762DK               0x12
 * #define RTL8762CK_VD            0xB2
 * #define RTL8762DT               0x93
 * #define RTL8762DW               0x43
 * #define RTL8762CW_VD            0xB3
 * #define HQ624W                  0x73
 * #define XM624W                  0x83
 * #define RTL8762CMF_VD           0xA9
 * #define RTL8752DRF              0xC8
 * #define RTL8752DJF              0xC9
 * #define RTL8762DRF              0x68
 * #define RTL8762DJF              0x69
 * #define RS624MF                 0x39
 */
uint8_t get_ic_type(void)
{
    return *(uint8_t *)0x00202543;
}

/** @} */ /* End of group SYSTEM_INIT_Exported_Functions */
/** @} */ /* End of group SYSTEM_INIT */
