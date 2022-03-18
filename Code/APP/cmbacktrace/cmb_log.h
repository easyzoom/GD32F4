/**
 * @file cmb_log.h
 * @author Letter (zhengkeqiang@ut.cn)
 * @brief 
 * @version 0.1
 * @date 2019-02-21
 * 
 * @Copyright (c) 2019 Unicook
 * 
 */

#ifndef __CMB_LOG_C__
#define __CMG_LOG_C__

#define     CMB_LOG_USING_FLASH         1

#if CMB_LOG_USING_FLASH == 1

    #define CMB_LOG_BUFFER_SIZE         256
    #define CMB_FLASH_SECTOR_SIZE       (128 * 1024)
    #define CMB_LOG_STOARGE_SIZE        (1 * 1024)
    #define CMB_FLASH_ADDRESS           0x080E0000
    #define CMB_FLASH_SECTOR            11

    void cmbLog(const char *format, ...);
    void cmbPrintLastLog(void);
    void cmbRestore(void);
#else
    #define cmbLog(...)                 printf(__VA_ARGS__);printf("\r\n")
#endif /** CMB_LOG_USING_FLASH == 1 */

#endif
