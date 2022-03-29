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


#endif /* __CONFIG_H */
