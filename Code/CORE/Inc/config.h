#ifndef __CONFIG_H
#define __CONFIG_H

#include "gd32f4xx.h"
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"

#define USART0_DATA_ADDRESS      ((USART_BASE + (0x0000CC00U)) + (0x04U))
#define USART1_DATA_ADDRESS      ( USART_BASE + (0x04U))
#define USART2_DATA_ADDRESS      ((USART_BASE + (0x00000400U)) + (0x04U))
#define USART3_DATA_ADDRESS      ((USART_BASE + (0x00000800U)) + (0x04U))
#define USART4_DATA_ADDRESS      ((USART_BASE + (0x00000C00U)) + (0x04U))
#define USART5_DATA_ADDRESS      ((USART_BASE + (0x0000D000U)) + (0x04U))
#define USART6_DATA_ADDRESS      ((USART_BASE + (0x00003400U)) + (0x04U))
#define USART7_DATA_ADDRESS      ((USART_BASE + (0x00003800U)) + (0x04U))


//#define FACTORY_TEST


#define DEBUG_PRINT_UART        UART4

#define MODULE_ENB_SYSTEM_OPERATION
//#define MODULE_ENB_FLASH_OPERATION
#define MODULE_ENB_REDIRECT_TO_CAN
//#define MODULE_ENB_REDIRECT_TO_COM
//#define MODULE_ENB_DRV8801_SERVO
//#define MODULE_ENB_SAFETY_INFO_EXCHANGE

//#define DRIVER_ENB_SN74LV165A
//#define DRIVER_ENB_SN74LV595A
//#define DRIVER_ENB_LTC7000
  
//#define TRANS_TOKEN_CONTROL
#define TRANS_LOS_DETECTION
  
//#define ENABLE_INS_REDIRECT_TO_COM_USART1
//#define ENABLE_INS_REDIRECT_TO_COM_USART2
//#define ENABLE_INS_REDIRECT_TO_COM_USART3
//#define ENABLE_INS_REDIRECT_TO_COM_USART4
//#define ENABLE_INS_REDIRECT_TO_COM_USART5
//#define ENABLE_INS_REDIRECT_TO_COM_USART6

#define ENABLE_INS_REDIRECT_TO_CAN_CAN1
#define ENABLE_INS_REDIRECT_TO_CAN_CAN2

/* Exported functions prototypes ---------------------------------------------*/


/* Private defines -----------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_H */
