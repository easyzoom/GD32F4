#ifndef __CONFIG_H
#define __CONFIG_H

#include "gd32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define USART0_DATA_ADDRESS     ((USART_BASE + (0x0000CC00U)) + (0x04U))
#define USART1_DATA_ADDRESS     ( USART_BASE + (0x04U))
#define USART2_DATA_ADDRESS     ((USART_BASE + (0x00000400U)) + (0x04U))
#define USART3_DATA_ADDRESS     ((USART_BASE + (0x00000800U)) + (0x04U))
#define USART4_DATA_ADDRESS     ((USART_BASE + (0x00000C00U)) + (0x04U))
#define USART5_DATA_ADDRESS     ((USART_BASE + (0x0000D000U)) + (0x04U))
#define USART6_DATA_ADDRESS     ((USART_BASE + (0x00003400U)) + (0x04U))
#define USART7_DATA_ADDRESS     ((USART_BASE + (0x00003800U)) + (0x04U))

#define MODULE_ENB_DEBUG_PRINT
//系统支持的网口数量，SW_PORTS_NUMBER = 外接网口数量 + 1(此为MCU连接交换机的Port0,外接网口从Port1开始)
#define SW_PORTS_NUMBER         5
#define SW_SOFTWARE_VERSION     "V1.1 Build "__DATE__" " //不大于26 byte
#define SW_HARDWARE_VERSION     "ROB042_MB201_V1.0"         //不大于20 byte
#define SW_DEV_NAME             "ROB042_MB201"              //不大于16 byte

#define MAC_ADDR0               2
#define MAC_ADDR1               0xA
#define MAC_ADDR2               0xF
#define MAC_ADDR3               0xE
#define MAC_ADDR4               0xD
#define MAC_ADDR5               6

#define CRC16_TABLE_CODE        5
#define FILE_MAX_BYTE           256

#define DEV_NAME                "desc"
#define MAC_SNAME               "mac"
#define IP_SNAME                "ip"
#define NETMASK_SNAME           "nm"
#define GATEWAY_SNAME           "gw"
#define SW_SNAME                "sw_ver"
#define HW_SNAME                "hw_ver"
#define PVL_STATE               "pvl_s"
#define PVLT_GR                 "tb"
#define CHECK_CRC               "ck_crc"

//#define USE_DHCP       /* enable DHCP, if disabled static address is used */



/* MII and RMII mode selection */
#define RMII_MODE  // user have to provide the 50 MHz clock by soldering a 50 MHz oscillator
//#define MII_MODE

/* clock the PHY from external 25MHz crystal (only for MII mode) */
#ifdef  MII_MODE
#define PHY_CLOCK_MCO
#endif

/* Macro to get variable aligned on 4-bytes, for __ICCARM__ the directive "#pragma data_alignment=4" must be used instead */
#if defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050) /* ARM Compiler V6 */
  #ifndef __ALIGN_BEGIN
    #define __ALIGN_BEGIN
  #endif
  #ifndef __ALIGN_END
    #define __ALIGN_END      __attribute__ ((aligned (4)))
  #endif
#elif defined ( __GNUC__ ) && !defined (__CC_ARM) /* GNU Compiler */
  #ifndef __ALIGN_END
#define __ALIGN_END    __attribute__ ((aligned (4)))
  #endif /* __ALIGN_END */
  #ifndef __ALIGN_BEGIN  
    #define __ALIGN_BEGIN
  #endif /* __ALIGN_BEGIN */
#else
  #ifndef __ALIGN_END
    #define __ALIGN_END
  #endif /* __ALIGN_END */
  #ifndef __ALIGN_BEGIN      
    #if defined   (__CC_ARM)      /* ARM Compiler V5*/
#define __ALIGN_BEGIN    __align(4)
    #elif defined (__ICCARM__)    /* IAR Compiler */
      #define __ALIGN_BEGIN 
    #endif /* __CC_ARM */
  #endif /* __ALIGN_BEGIN */
#endif /* __GNUC__ */

#endif /* __CONFIG_H */
