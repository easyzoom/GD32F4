#ifndef __CONFIG_H
#define __CONFIG_H

#include "gd32f4xx.h"
#include <stdio.h>
#include <string.h>

#define USART0_DATA_ADDRESS      ((USART_BASE + (0x0000CC00U)) + (0x04U))
#define USART1_DATA_ADDRESS      ( USART_BASE + (0x04U))
#define USART2_DATA_ADDRESS      ((USART_BASE + (0x00000400U)) + (0x04U))
#define USART3_DATA_ADDRESS      ((USART_BASE + (0x00000800U)) + (0x04U))
#define USART4_DATA_ADDRESS      ((USART_BASE + (0x00000C00U)) + (0x04U))
#define USART5_DATA_ADDRESS      ((USART_BASE + (0x0000D000U)) + (0x04U))
#define USART6_DATA_ADDRESS      ((USART_BASE + (0x00003400U)) + (0x04U))
#define USART7_DATA_ADDRESS      ((USART_BASE + (0x00003800U)) + (0x04U))

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
